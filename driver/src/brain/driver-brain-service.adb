with Ada.Environment_Variables;
with Ada.Strings.Fixed;
with Driver.Beats;
with Driver.Brain.Pictures;
with Driver.Brain.Runaway;
with Driver.Clock;
with Driver.Json;
with Driver.Log;
with Driver.Services;

package body Driver.Brain.Service is

   use type Driver.Json.Kind;

   Path       : constant String := "/v1/chat/completions";
   Model_Name : constant String := "eye";
   --  The name the service serves the model under (docs/brain-service.md).

   Settings_Variable : constant String := "BL_BRAIN_SAMPLING";

   Own_Members : constant array (Positive range <>) of access constant String :=
     [new String'("model"), new String'("messages"), new String'("stream"), new String'("structured_outputs"),
      new String'("response_format")];
   --  The members the driver writes itself; the deployment may not set them.

   Thousandths : constant := 1000.0;
   --  Boxes are given in thousandths of the picture's width and height.

   function Members (Raw : String; Why : out Unbounded_String) return String is
      T   : constant String := Ada.Strings.Fixed.Trim (Raw, Ada.Strings.Both);
      Doc : Driver.Json.Document;
      Ok  : Boolean;
   begin
      Why := Null_Unbounded_String;
      if T'Length = 0 then
         Why := To_Unbounded_String ("it is not set, so the service samples by its own defaults");
         return "";
      end if;
      Driver.Json.Parse (T, Doc, Ok, Why);
      if not Ok or else Driver.Json.Kind_Of (Doc, Driver.Json.Root (Doc)) /= Driver.Json.Object_Value then
         Why := "it is not one JSON object (" & Why & "), so it is not used";
         return "";
      end if;
      for I in 1 .. Driver.Json.Count (Doc, Driver.Json.Root (Doc)) loop
         declare
            Member : constant String := Driver.Json.Member_Name (Doc, Driver.Json.Root (Doc), I);
         begin
            if (for some M of Own_Members => M.all = Member) then
               Why := To_Unbounded_String ("it sets """ & Member & """, which the driver writes itself, so it is not used");
               return "";
            end if;
         end;
      end loop;
      return Ada.Strings.Fixed.Trim (T (T'First + 1 .. T'Last - 1), Ada.Strings.Both);
   end Members;

   Read_Settings : Boolean := False;
   Settings      : Unbounded_String;   --  the deployment's members, without the braces

   procedure Load_Settings is
      Raw : constant String :=
        (if Ada.Environment_Variables.Exists (Settings_Variable) then Ada.Environment_Variables.Value (Settings_Variable)
         else "");
      Why : Unbounded_String;
   begin
      Read_Settings := True;
      Settings := To_Unbounded_String (Members (Raw, Why));
      Driver.Log.Line (Driver.Log.Brain, Settings_Variable & ": "
                       & (if Length (Why) > 0 then To_String (Why) else "both questions carry it as it is")
                       & (if Raw'Length > 0 then ": " & Raw else ""));
   end Load_Settings;

   function Head return String is
   begin
      if not Read_Settings then
         Load_Settings;
      end if;
      return "{""model"":" & Driver.Json.Quote (Model_Name)
        & (if Length (Settings) > 0 then "," & To_String (Settings) else "");
   end Head;

   function User_Message (Picture_Url, Text : String) return String is
     ("""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":"
      & Driver.Json.Quote (Picture_Url) & "}},{""type"":""text"",""text"":" & Driver.Json.Quote (Text) & "}]}]");

   function Program_Request (Picture_Url, Prompt, Grammar : String) return String is
     (Head & ",""stream"":true,""structured_outputs"":{""grammar"":" & Driver.Json.Quote (Grammar) & "},"
      & User_Message (Picture_Url, Prompt) & "}");

   --  The where-is-it answer exactly, with no blank anywhere and every edge
   --  a whole number of thousandths: under a JSON schema the decoder may
   --  write blanks between the members without end, and greedy decoding
   --  did (measured: an endless run of tabs after "found":).
   Where_Grammar : constant String :=
     "root ::= ""{\""found\"":"" (""true"" | ""false"") "",\""bbox_2d\"":["" e "","" e "","" e "","" e ""]}"""
     & ASCII.LF & "e ::= ""1000"" | [1-9] [0-9] [0-9] | [1-9] [0-9] | [0-9]";

   function Where_Prompt (Name : String) return String is
     ("Locate what someone would call: " & Name & ASCII.LF
      & "If you can see it in this picture, answer with the box around it. If you cannot see it here, say so - "
      & "that is a normal answer and I will look with another eye rather than guess.");

   function Where_Request (Picture_Url, Name : String) return String is
     (Head & ",""structured_outputs"":{""grammar"":" & Driver.Json.Quote (Where_Grammar) & "},"
      & User_Message (Picture_Url, Where_Prompt (Name)) & "}");

   function Where_Answer_Grammar return String is (Where_Grammar);

   --  Calls of the current episode, for the log.
   Programs_Asked, Wheres_Asked : Natural := 0;

   function Calls return String is (Driver.Log.Image (Programs_Asked + Wheres_Asked));

   procedure New_Episode is
   begin
      if Programs_Asked + Wheres_Asked > 0 then
         Driver.Log.Line (Driver.Log.Brain, "the last episode called the brain " & Calls & " times ("
                          & Driver.Log.Image (Programs_Asked) & " programs, " & Driver.Log.Image (Wheres_Asked)
                          & " where-is-it questions)");
      end if;
      Programs_Asked := 0;
      Wheres_Asked := 0;
   end New_Episode;

   procedure Read_Event (Event : String; Text : in out Unbounded_String; Finish : in out Unbounded_String) is
      Doc : Driver.Json.Document;
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      Driver.Json.Parse (Event, Doc, Ok, Why);
      if not Ok then
         return;
      end if;
      declare
         Choice : constant Driver.Json.Node :=
           Driver.Json.Element (Doc, Driver.Json.Lookup (Doc, Driver.Json.Root (Doc), "choices"), Positive'First);
         Reason : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Choice, "finish_reason");
      begin
         Append (Text, Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Driver.Json.Lookup (Doc, Choice, "delta"),
                                                                 "content")));
         if Driver.Json.Kind_Of (Doc, Reason) = Driver.Json.String_Value then
            Finish := To_Unbounded_String (Driver.Json.Text (Doc, Reason));
         end if;
      end;
   end Read_Event;

   function Write_Program
     (Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Answer
   is
      Glue    : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Keyboard.Name_Words (Keys);
      Episode : constant Natural := Driver.Beats.Episode;
      Started : constant Duration := Driver.Clock.Seconds;
      Text    : Unbounded_String;
      Finish  : Unbounded_String;
      Verdict : Driver.Brain.Runaway.Verdict;
      Over    : Boolean := False;   --  a new episode began while the brain was writing

      procedure On_Text (Chunk : String; Stop : out Boolean) is
      begin
         Read_Event (Chunk, Text, Finish);
         Verdict := Driver.Brain.Runaway.Judge (To_String (Text), Glue, Final => False);
         Over := Driver.Beats.Episode /= Episode;
         Stop := Driver.Brain.Runaway.Stop_Reading (Verdict) or else Over;
      end On_Text;

      R : Driver.Services.Reply;
      A : Answer;
   begin
      Programs_Asked := Programs_Asked + 1;
      R := Driver.Services.Call_Streaming
        (Driver.Services.Brain, Path,
         Program_Request (Driver.Brain.Pictures.Data_Url (Picture), Prompt, Driver.Brain.Keyboard.Grammar (Keys)),
         On_Text'Access);
      A.Seconds := Driver.Clock.Seconds - Started;
      if not Driver.Brain.Runaway.Stop_Reading (Verdict) and then not Over then
         --  The stream ended on its own; its last line is finished unless the
         --  service stopped at its own limit in the middle of it.
         Verdict := Driver.Brain.Runaway.Judge (To_String (Text), Glue, Final => To_String (Finish) /= "length");
      end if;
      A.Program := To_Unbounded_String (Driver.Brain.Runaway.First_Lines (To_String (Text), Verdict.Keep));
      if not R.Ok and then Length (Text) = 0 then
         A.How := Failed;
         A.Why := R.Why;
      elsif Over then
         A.How := Failed;
         A.Why := To_Unbounded_String ("a new episode began while the brain was writing");
      elsif Verdict.Fired then
         A.How := Ran_Away;
         A.Why := Verdict.Why;
      elsif Verdict.Complete then
         A.How := Complete;
         A.Why := Verdict.Why;
      elsif To_String (Finish) = "length" then
         A.How := Service_Stopped;
         A.Why := To_Unbounded_String ("the service stopped the answer at its own length limit");
      else
         A.How := Ended;
      end if;
      Driver.Log.Line (Driver.Log.Brain, "call " & Calls & " of this episode: write a program, "
                       & Driver.Log.Image (Real (A.Seconds), 1) & " s, "
                       & Image (A.How) & (if Length (A.Why) > 0 then " (" & To_String (A.Why) & ")" else "")
                       & ", " & Driver.Log.Image (Verdict.Keep) & " lines kept");
      return A;
   end Write_Program;

   procedure Read_Where
     (Reply  : String;
      Width  : Positive;
      Height : Positive;
      Found  : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String)
   is
      Outer, Inner : Driver.Json.Document;
      Ok           : Boolean;
   begin
      Found := Driver.Brain.Names.No_Answer;
      Where := (others => <>);
      Driver.Json.Parse (Reply, Outer, Ok, Why);
      if not Ok then
         Why := "the reply is not JSON: " & Why;
         return;
      end if;
      declare
         Content : constant String := Driver.Json.Text
           (Outer, Driver.Json.Lookup
              (Outer, Driver.Json.Lookup
                 (Outer, Driver.Json.Element (Outer, Driver.Json.Lookup (Outer, Driver.Json.Root (Outer), "choices"),
                                             Positive'First),
                  "message"), "content"));
      begin
         Driver.Json.Parse (Content, Inner, Ok, Why);
         if not Ok then
            Why := "the answer is not JSON: " & Why;
            return;
         end if;
      end;
      declare
         type Edge_Name is (Left, Top, Right, Bottom);   --  the order of bbox_2d
         Root : constant Driver.Json.Node := Driver.Json.Root (Inner);
         Box  : constant Driver.Json.Node := Driver.Json.Lookup (Inner, Root, "bbox_2d");

         function Edge (E : Edge_Name) return Real is
           (Real'Max (0.0, Real'Min (Thousandths, Driver.Json.Number
                                       (Inner, Driver.Json.Element (Inner, Box, Edge_Name'Pos (E) + 1))))
            / Thousandths * Real (if E in Left | Right then Width else Height));
      begin
         if not Driver.Json.Is_True (Inner, Driver.Json.Lookup (Inner, Root, "found")) then
            Found := Driver.Brain.Names.Not_Here;
            Why := To_Unbounded_String ("it says it cannot see it here");
         elsif Driver.Json.Count (Inner, Box) /= Edge_Name'Pos (Edge_Name'Last) + 1 then
            Why := To_Unbounded_String ("found, but the box does not have four edges");
         elsif Edge (Right) <= Edge (Left) or else Edge (Bottom) <= Edge (Top) then
            Found := Driver.Brain.Names.Not_Here;
            Why := To_Unbounded_String ("it gave an empty box");
         else
            Found := Driver.Brain.Names.Boxed;
            Where := (Top_Left     => (U => Edge (Left), V => Edge (Top)),
                      Bottom_Right => (U => Edge (Right), V => Edge (Bottom)));
            Why := Null_Unbounded_String;
         end if;
      end;
   end Read_Where;

   procedure Ask_Where
     (Picture : Driver.Images.Image;
      Name    : String;
      Found   : out Driver.Brain.Names.Pointing;
      Where   : out Driver.Brain.Names.Box;
      Why     : out Unbounded_String)
   is
      Started : constant Duration := Driver.Clock.Seconds;
      R       : Driver.Services.Reply;
   begin
      Wheres_Asked := Wheres_Asked + 1;
      R := Driver.Services.Call (Driver.Services.Brain, Path,
                                 Where_Request (Driver.Brain.Pictures.Data_Url (Picture), Name));
      if not R.Ok then
         Found := Driver.Brain.Names.No_Answer;
         Where := (others => <>);
         Why := R.Why & (if Length (R.Text) > 0 then ": " & R.Text else Null_Unbounded_String);
      else
         Read_Where (To_String (R.Text), Driver.Images.Width (Picture), Driver.Images.Height (Picture),
                     Found, Where, Why);
      end if;
      Driver.Log.Line
        (Driver.Log.Brain, "call " & Calls & " of this episode: where is """ & Name & """, "
         & Driver.Log.Image (Real (Driver.Clock.Seconds - Started), 1) & " s, "
         & (case Found is
              when Driver.Brain.Names.Boxed =>
                 "box from (" & Driver.Log.Image (Where.Top_Left.U, 0) & ", " & Driver.Log.Image (Where.Top_Left.V, 0)
                 & ") to (" & Driver.Log.Image (Where.Bottom_Right.U, 0) & ", "
                 & Driver.Log.Image (Where.Bottom_Right.V, 0) & ")",
              when Driver.Brain.Names.Not_Here => "not here (" & To_String (Why) & ")",
              when Driver.Brain.Names.No_Answer => "no answer (" & To_String (Why) & ")"));
   end Ask_Where;

end Driver.Brain.Service;
