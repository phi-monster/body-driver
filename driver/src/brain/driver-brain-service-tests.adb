with Ada.Streams;
with Ada.Strings.Fixed;
with GNAT.Sockets;
with Driver.Action;
with Driver.Brain.Keyboard.Tests;
with Driver.Brain.Names;
with Driver.Brain.Pictures;
with Driver.Instrument;
with Driver.Bytes;
with Driver.Clock;
with Driver.Json;
with Driver.Log;
with Driver.Observations;
with Driver.Services;
with Driver.Tests;

package body Driver.Brain.Service.Tests is

   use Driver.Tests;
   use type Driver.Brain.Names.Pointing;
   use type Driver.Json.Node;
   use type Driver.Bytes.Offset;

   LF   : constant String := [ASCII.LF];
   CRLF : constant String := ASCII.CR & ASCII.LF;

   function Has (S, Part : String) return Boolean is (Ada.Strings.Fixed.Index (S, Part) > 0);

   function Tiny return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. 12) := [others => 0];
   begin
      Data (1) := 255;     --  column 0, row 0: red
      Data (11) := 255;    --  column 1, row 1: green
      return Driver.Images.Create (2, 2, Data);
   end Tiny;

   function Keys return Driver.Brain.Keyboard.Keyboard is
      Q, M : Driver.Brain.Keyboard.Word_Vectors.Vector;
   begin
      Q.Append ("height");
      M.Append ("");
      return Driver.Brain.Keyboard.Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True],
                                           False, [others => False], Driver.Brain.Keyboard.Eye_Vectors.Empty_Vector);
   end Keys;

   procedure Settings is
      Why : Unbounded_String;
   begin
      Check (Members ("{""temperature"": 0.7, ""top_p"": 0.8, ""chat_template_kwargs"": {""enable_thinking"": false}}",
                      Why) = """temperature"": 0.7, ""top_p"": 0.8, ""chat_template_kwargs"": {""enable_thinking"": false}",
             "the deployment's object is merged as it is");
      Check (Members ("", Why) = "" and then Has (To_String (Why), "not set"), "no setting: nothing is merged");
      Check (Members ("{""temperature"": 0.7", Why) = "" and then Has (To_String (Why), "not one JSON object"),
             "an object that is not JSON is not used");
      Check (Members ("{""model"": ""other"", ""top_k"": 20}", Why) = "" and then Has (To_String (Why), "model"),
             "an object that sets a member the driver writes is not used");
      declare
         Doc : Driver.Json.Document;
         Ok  : Boolean;
      begin
         Driver.Json.Parse (Program_Request ("data:image/bmp;base64,AAAA", "line one" & LF & "quote "" here", "root ::= x"),
                            Doc, Ok, Why);
         Check (Ok and then Driver.Json.Is_True (Doc, Driver.Json.Lookup (Doc, Driver.Json.Root (Doc), "stream"))
                and then Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Driver.Json.Lookup
                  (Doc, Driver.Json.Root (Doc), "structured_outputs"), "grammar")) = "root ::= x",
                "the program request is JSON, streamed, with the grammar");
         Driver.Json.Parse (Where_Request ("data:image/bmp;base64,AAAA", "the red cup"), Doc, Ok, Why);
         Check (Ok and then Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Driver.Json.Lookup
                  (Doc, Driver.Json.Root (Doc), "structured_outputs"), "grammar")) = Where_Answer_Grammar
                and then Driver.Json.Lookup (Doc, Driver.Json.Root (Doc), "stream") = Driver.Json.No_Node,
                "the where request carries the answer's grammar and is not streamed");
      end;
   end Settings;

   function Event (Content : String; Finish : String := "") return String is
     ("{""choices"":[{""index"":0,""delta"":{""content"":" & Driver.Json.Quote (Content) & "},""finish_reason"":"
      & (if Finish = "" then "null" else Driver.Json.Quote (Finish)) & "}]}");

   procedure Events is
      Text, Finish : Unbounded_String;
   begin
      Read_Event (Event ("do the cup"), Text, Finish);
      Read_Event (Event (" height up until settled" & LF, "stop"), Text, Finish);
      Read_Event ("{""choices"":[],""usage"":{""prompt_tokens"":3}}", Text, Finish);
      Read_Event ("not json", Text, Finish);
      Check (To_String (Text) = "do the cup height up until settled" & LF and then To_String (Finish) = "stop",
             "event texts are joined, the finish reason kept, an event without a choice or not JSON is skipped");
   end Events;

   function Reply_With (Content : String) return String is
     ("{""choices"":[{""index"":0,""message"":{""role"":""assistant"",""content"":" & Driver.Json.Quote (Content)
      & "},""finish_reason"":""stop""}]}");

   procedure Where_Answers is
      Found : Driver.Brain.Names.Pointing;
      Where : Driver.Brain.Names.Box;
      Why   : Unbounded_String;
   begin
      Read_Where (Reply_With ("{""found"": true, ""bbox_2d"": [100, 200, 300, 400]}"), 640, 480, Found, Where, Why);
      Check (Found = Driver.Brain.Names.Boxed, "a box");
      Check_Close (Where.Top_Left.U, 64.0, 1.0e-9, "left in pixels");
      Check_Close (Where.Top_Left.V, 96.0, 1.0e-9, "top in pixels");
      Check_Close (Where.Bottom_Right.U, 192.0, 1.0e-9, "right in pixels");
      Check_Close (Where.Bottom_Right.V, 192.0, 1.0e-9, "bottom in pixels");
      Read_Where (Reply_With ("{""found"": false, ""bbox_2d"": [0, 0, 0, 0]}"), 640, 480, Found, Where, Why);
      Check (Found = Driver.Brain.Names.Not_Here, "not here is a normal answer");
      Read_Where (Reply_With ("{""found"": true, ""bbox_2d"": [300, 200, 100, 400]}"), 640, 480, Found, Where, Why);
      Check (Found = Driver.Brain.Names.Not_Here, "an empty box points at nothing");
      Read_Where ("<html>", 640, 480, Found, Where, Why);
      Check (Found = Driver.Brain.Names.No_Answer, "a reply that is not JSON is no answer");
   end Where_Answers;

   procedure Where_Grammar is
      G : constant String := Where_Answer_Grammar;
      Found : Driver.Brain.Names.Pointing;
      Where : Driver.Brain.Names.Box;
      Why   : Unbounded_String;
   begin
      Check (Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":true,""bbox_2d"":[100,200,300,400]}")
             and then Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":false,""bbox_2d"":[0,0,0,0]}")
             and then Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":true,""bbox_2d"":[0,9,99,1000]}"),
             "every answer the driver reads can be typed");
      Check (not Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"": true,""bbox_2d"":[1,2,3,4]}")
             and then not Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":" & ASCII.HT & "true,""bbox_2d"":[1,2,3,4]}"),
             "no blank can be typed anywhere, so the answer cannot run on");
      Check (not Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":true,""bbox_2d"":[1001,2,3,4]}")
             and then not Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":true,""bbox_2d"":[01,2,3,4]}")
             and then not Driver.Brain.Keyboard.Tests.Accepts (G, "{""found"":true,""bbox_2d"":[1,2,3]}"),
             "an edge past 1000, a leading zero, or three edges cannot be typed");
      Read_Where (Reply_With ("{""found"":true,""bbox_2d"":[100,200,300,400]}"), 640, 480, Found, Where, Why);
      Check (Found = Driver.Brain.Names.Boxed, "the grammar's answer is read as a box");
   end Where_Grammar;

   --  A fake service: one connection, a canned reply written at once, then
   --  the connection held open, as a service still writing would.

   task type Fake_Service is
      entry Start (Reply : String; Hold : Duration; Port : out Natural);
      entry Finished (Request : out Unbounded_String);
   end Fake_Service;

   task body Fake_Service is
      use GNAT.Sockets;
      Listener, Client : Socket_Type;
      Address          : Sock_Addr_Type := (Family_Inet, Inet_Addr ("127.0.0.1"), 0);
      Text             : Unbounded_String;
      Got              : Unbounded_String;
      Wait             : Duration;
   begin
      accept Start (Reply : String; Hold : Duration; Port : out Natural) do
         Text := To_Unbounded_String (Reply);
         Wait := Hold;
         Create_Socket (Listener);
         Set_Socket_Option (Listener, Socket_Level, (Reuse_Address, True));
         Bind_Socket (Listener, Address);
         Listen_Socket (Listener);
         Address := Get_Socket_Name (Listener);
         Port := Natural (Address.Port);
      end Start;
      Accept_Socket (Listener, Client, Address);
      declare
         Buffer : Ada.Streams.Stream_Element_Array (1 .. 65_536);
         Last   : Ada.Streams.Stream_Element_Offset;
      begin
         --  The request: headers, then as many body bytes as they announce.
         loop
            Receive_Socket (Client, Buffer, Last);
            exit when Last < Buffer'First;
            Append (Got, Driver.Bytes.To_String (Buffer (Buffer'First .. Last)));
            declare
               G    : constant String := To_String (Got);
               Ends : constant Natural := Ada.Strings.Fixed.Index (G, CRLF & CRLF);
               Size : constant Natural := Ada.Strings.Fixed.Index (G, "Content-Length: ");
            begin
               if Ends > 0 and then Size > 0 then
                  declare
                     Stop : constant Natural := Ada.Strings.Fixed.Index (G, CRLF, Size);
                     N    : constant Natural := Natural'Value (G (Size + 16 .. Stop - 1));
                  begin
                     exit when G'Length - (Ends + 3) >= N;
                  end;
               end if;
            end;
         end loop;
         declare
            Out_Bytes : constant Ada.Streams.Stream_Element_Array := Driver.Bytes.To_Bytes (To_String (Text));
            Sent      : Ada.Streams.Stream_Element_Offset;
         begin
            Send_Socket (Client, Out_Bytes, Sent);
         end;
      exception
         when Socket_Error =>
            null;
      end;
      delay Wait;
      Close_Socket (Client);
      Close_Socket (Listener);
      accept Finished (Request : out Unbounded_String) do
         Request := Got;
      end Finished;
   end Fake_Service;

   function Streamed (Events : String) return String is
     ("HTTP/1.1 200 OK" & CRLF & "Content-Type: text/event-stream" & CRLF & "Connection: close" & CRLF & CRLF & Events);

   function Data (Content : String) return String is ("data: " & Event (Content) & LF & LF);

   procedure Ask (Reply : String; Hold : Duration; A : out Answer; Took : out Duration) is
      F       : Fake_Service;
      Port    : Natural;
      Request : Unbounded_String;
      Started : Duration;
   begin
      F.Start (Reply, Hold, Port);
      Driver.Services.Configure (Driver.Services.Brain, "127.0.0.1", Port);
      Started := Driver.Clock.Seconds;
      A := Write_Program (Tiny, "write one program", Keys);
      Took := Driver.Clock.Seconds - Started;
      F.Finished (Request);
      Check (Has (To_String (Request), """structured_outputs""") and then Has (To_String (Request), "data:image/bmp"),
             "the request carries the grammar and the picture");
   end Ask;

   procedure Streaming is
      Hold : constant Duration := 2.0;
      A    : Answer;
      Took : Duration;
   begin
      Ask (Streamed (Data ("do the cup height up until settled" & LF) & Data ("done" & LF) & Data ("done" & LF)
                     & Data ("done" & LF)), Hold, A, Took);
      Check (A.How = Complete and then To_String (A.Program) = "do the cup height up until settled" & LF & "done" & LF
             and then Took < Hold,
             "reading stops at a done outside every block, before the service would have ended:"
             & Driver.Log.Image (Real (Took), 2) & " s, kept: " & To_String (A.Program));
      Ask (Streamed (Data ("do the cup height up until settled" & LF) & Data ("do upth upth upth upth")), Hold, A, Took);
      Check (A.How = Ran_Away and then To_String (A.Program) = "do the cup height up until settled" & LF
             and then Took < Hold,
             "a copy loop is cut at once and the finished lines before it are kept");
      Ask (Streamed (Data ("say I see the cup" & LF) & "data: [DONE]" & LF & LF), Hold, A, Took);
      Check (A.How = Ended and then To_String (A.Program) = "say I see the cup" & LF,
             "a stream that ends by itself is kept whole");
   end Streaming;

   procedure Pictures_Encoded is
   begin
      Check (Driver.Brain.Pictures.Data_Url (Tiny) = "data:image/bmp;base64," & Driver.Instrument.Bitmap (Tiny),
             "the brain's picture is a data URL of the BMP the instrument gets");
   end Pictures_Encoded;

   function Wide return Driver.Images.Image is
      Data : constant Driver.Bytes.Byte_Array (1 .. 3 * 4 * 2) := [others => 200];
   begin
      return Driver.Images.Create (4, 2, Data);
   end Wide;

   function Image_Of (E : Driver.Brain.Names.Eye_Id) return Driver.Images.Image is
     (if Driver.Observations."=" (E, 2) then Wide else Tiny);

   procedure Composed is
      Rest : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      Rest.Append (2);
      Rest.Append (3);
      declare
         C : constant Driver.Images.Image := Driver.Brain.Pictures.Compose (Wide, Rest, Image_Of'Access);
      begin
         Check (Driver.Images.Width (C) = 4 and then Driver.Images.Height (C) = 2 + 2,
                "the main eye on top, a strip as tall as its tallest scaled eye below");
         Check (Driver.Images.Red (C, 0, 2) = 200 and then Driver.Images.Red (C, 2, 2) = 255
                and then Driver.Images.Green (C, 3, 3) = 255,
                "the other eyes side by side in the order given, each keeping its proportions");
      end;
   end Composed;

   procedure Register is
   begin
      Register ("brain.service.settings", "the driver adds a sampling setting of its own, or merges a broken one",
                Settings'Access);
      Register ("brain.service.events", "a streamed answer loses text or its finish reason", Events'Access);
      Register ("brain.service.where_grammar", "the where-is-it answer can run on, or cannot say what the driver reads",
                Where_Grammar'Access);
      Register ("brain.service.where", "a box is turned into the wrong pixels, or not here is taken for a failure",
                Where_Answers'Access);
      Register ("brain.service.streaming", "a runaway answer is read to the end of the stream, or a whole one cut",
                Streaming'Access);
      Register ("brain.pictures.encoding", "a picture is sent as a broken BMP or base64", Pictures_Encoded'Access);
      Register ("brain.pictures.compose", "an eye is missing from the picture or placed out of order",
                Composed'Access);
   end Register;

end Driver.Brain.Service.Tests;
