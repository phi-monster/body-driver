--  replay RECORDING [--body FILE] [--estimates FILE] [--inst HOST:PORT] [--eye HOST:PORT] [--watch EYE]
--
--  Feeds a recording (Driver.Recording format, uncompressed; it is read once,
--  forward, so "zstd -dc run.rec.zst | replay /dev/stdin ..." needs no copy on
--  disk) through every estimator exactly as the main loop does: each observation is
--  given to the robot, hand and world estimators together with the last
--  command sent before it arrived (read back from the recorded replies), the
--  command in effect while it was captured. At the end the measured body is
--  written to the body file. A body file the run read is in the recording
--  (Driver.Recording, kind F), and the body is reloaded from that text where
--  the run read it, so a run that booted from a body file replays as it ran,
--  even after the run rewrote the file. Every estimate a decider
--  asked for at once is in the recording too (kind E) and is made at the
--  same point, and so is every write a decider made into the world (kind W).
--  Where the run computed the heavier estimates apart from its main loop
--  (kinds K, A, B), the models are given each message where the run gave it
--  to them (Driver.Apart), and the estimates are computed in place there.
--  With --estimates, every beat after boot writes
--  one JSON line with each arm's tool pose and each eye's pose (row-major 4 x 4,
--  world frame); two last lines hold, for each eye, the lines of sight of a
--  grid of pixels in the eye's own frame, and each hand's tips in its tool
--  frame (with the press direction that defines each tip, the closer
--  readings it belongs to, the beat of the press it rests on, whether the
--  presses have told it across its line of sight and by how much (the two
--  standard deviations across it), the same tip of the finger unloaded, and
--  the slide each press measured), so the estimates
--  can be scored against simulator truth whatever model produced them.
--  Nothing here decides anything; the recorded replies did.
--
--  It prints the estimators' own log lines, each begun with "@" and the beat
--  of the recording it was written at, and, last, the time the driver's work
--  took (parsing the messages, the robot, hand and world estimators, the
--  estimates computed apart), in seconds and milliseconds a beat, and how many
--  beats it replayed. With --watch EYE, every beat also gives a line on that
--  eye (Driver.Robot.Eye_Watch): how much its picture changed, whether it is
--  still and settled, and how far its cells moved.
--
--  The estimators' service calls are answered as Driver.Services describes
--  for a replay: by the live service when --inst or --eye names one (a
--  recording without service replies, or a new instrument asked again),
--  otherwise from the recording's own replies; in both cases ready from the
--  beat after the call, so the result never depends on how fast the replay
--  runs.

with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Strings.Unbounded;
with Driver.Apart;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Json;
with Driver.Log;
with Driver.Msgpack;
with Driver.Numerics;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Replies;
with Driver.Robot;
with Driver.Robot.Boot;
with Driver.Robot.Hand;
with Driver.Services;
with Driver.Uncertain;
with Driver.World;

procedure Replay is

   use Ada.Command_Line;
   use Ada.Strings.Unbounded;
   use Driver.Log;
   use type Driver.Msgpack.Node;
   use type Driver.Protocol.Message_Kind;

   Path      : Unbounded_String;
   Body_File : Unbounded_String;
   Estimates : Unbounded_String;
   Out_File  : Ada.Text_IO.File_Type;
   Last_Obs  : Driver.Observations.Observation;
   Current   : Driver.Observations.Observation;   --  the observation of the latest beat

   R        : Driver.Recording.Reader;
   Opened   : Boolean;
   Kind     : Driver.Recording.Record_Kind;
   Ns       : Long_Long_Integer;
   Payload  : Driver.Bytes.Buffer;
   More     : Boolean := True;

   Layout   : Driver.Observations.Layout;
   Known    : Boolean := False;
   Beat     : Natural := 0;
   Episodes : Natural := 0;
   Commanded_Beats : Natural := 0;

   Robot : aliased Driver.Robot.Model;
   Hands : aliased Driver.Robot.Hand.Hands;
   Scene : aliased Driver.World.Scene;
   Sent  : Driver.Commands.Command := Driver.Commands.Hold;   --  the last command sent so far

   function Pose_Json (P : Driver.Numerics.Rigid) return String is
      R : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            Append (R, Driver.Json.Number_Image (P.Rotation (I, J)) & ",");
         end loop;
         Append (R, Driver.Json.Number_Image (P.Translation (I)) & ",");
      end loop;
      Append (R, "0,0,0,1]");
      return To_String (R);
   end Pose_Json;

   procedure Write_Beat (O : Driver.Observations.Observation) is
      Line_Text : Unbounded_String := To_Unbounded_String ("{""beat"":" & Natural'Image (Natural (O.Beat)) & ",""tools"":[");
   begin
      for A in 1 .. Driver.Robot.Arm_Count (Robot) loop
         Append (Line_Text, (if A > 1 then "," else "")
                 & Pose_Json (Driver.Robot.Tool_Pose (Robot, Driver.Robot.Arm_Id (A), O).Pose));
      end loop;
      Append (Line_Text, "],""eyes"":[");
      for E in 1 .. Driver.Robot.Eye_Count (Robot) loop
         Append (Line_Text, (if E > 1 then "," else "")
                 & Pose_Json (Driver.Robot.Eye_Pose (Robot, Driver.Robot.Eye_Id (E), O).Pose));
      end loop;
      Ada.Text_IO.Put_Line (Out_File, To_String (Line_Text) & "]}");
   end Write_Beat;

   procedure Write_Hands is
      --  Each hand's tips in its arm's tool frame, with their covariance (null
      --  while unknown), whether a second press confirmed them, the beat of the
      --  press the tip rests on (0 while unknown), the press direction that
      --  defines them and the closer readings they belong to; the same three
      --  for the tip of the finger unloaded ("free_"), unknown until both
      --  openings of the lobe have a tip and the slides are measured; and the
      --  slide of each press made at the opening.
      use Driver.Robot.Hand;
      Line_Text : Unbounded_String := To_Unbounded_String ("{""hands"":[");

      function Vector_Json (V : Driver.Numerics.Vec3) return String is
        ("[" & Driver.Json.Number_Image (V (1)) & "," & Driver.Json.Number_Image (V (2)) & ","
         & Driver.Json.Number_Image (V (3)) & "]");

      function Covariance_Json (P : Driver.Uncertain.Point_Estimate) return String is
        (if not Driver.Uncertain.Known (P) then "null"
         else "[" & Vector_Json ([P.Covariance (1, 1), P.Covariance (1, 2), P.Covariance (1, 3)]) & ","
              & Vector_Json ([P.Covariance (2, 1), P.Covariance (2, 2), P.Covariance (2, 3)]) & ","
              & Vector_Json ([P.Covariance (3, 1), P.Covariance (3, 2), P.Covariance (3, 3)]) & "]");

      function Readings_Json (X : Driver.Real_Array) return String is
         R : Unbounded_String := To_Unbounded_String ("[");
      begin
         for I in X'Range loop
            Append (R, (if I = X'First then "" else ",") & Driver.Json.Number_Image (X (I)));
         end loop;
         return To_String (R) & "]";
      end Readings_Json;

      function Slides_Json (S : Slide_Readings) return String is
         R : Unbounded_String := To_Unbounded_String ("[");

         function Flag (B : Boolean) return String is (if B then "true" else "false");
      begin
         for I in S'Range loop
            Append (R, (if I = S'First then "" else ",") & "{""beat"":" & Natural'Image (Natural (S (I).Beat))
                    & ",""contact"":" & Flag (S (I).Contact) & ",""rests"":" & Flag (S (I).Tip_Rests)
                    & ",""known"":" & Flag (S (I).Known) & ",""pixels"":" & Driver.Json.Number_Image (S (I).Pixels)
                    & ",""pixels_sigma"":" & Driver.Json.Number_Image (S (I).Pixels_Sigma) & ",""fraction"":"
                    & Driver.Json.Number_Image (S (I).Fraction) & ",""fraction_sigma"":"
                    & Driver.Json.Number_Image (S (I).Fraction_Sigma) & "}");
         end loop;
         return To_String (R) & "]";
      end Slides_Json;
   begin
      for Id in 1 .. Hand_Count (Hands) loop
         declare
            H : constant Hand_Id := Hand_Id (Id);
         begin
            Append (Line_Text, (if Id > 1 then "," else "") & "{""arm"":" & Natural'Image (Natural (Arm_Of (Hands, H)))
                    & ",""closer"":""" & To_String (Layout.Groups (Closer_Group (Hands, H)).Path) & """,""lobes"":[");
            for Lobe in 1 .. Lobe_Count (Hands, H) loop
               Append (Line_Text, (if Lobe > 1 then ",{" else "{"));
               for At_Opening in Opening loop
                  Append (Line_Text, (if At_Opening = Opening'First then "" else ",") & """"
                          & (if At_Opening = Open then "open" else "closed") & """:{""tip"":"
                          & Vector_Json (Tip_In_Tool (Hands, H, Lobe, At_Opening).Mean) & ",""covariance"":"
                          & Covariance_Json (Tip_In_Tool (Hands, H, Lobe, At_Opening)) & ",""confirmed"":"
                          & (if Tip_Confirmed (Hands, H, Lobe, At_Opening) then "true" else "false") & ",""tested"":"
                          & (if Tip_Tested (Hands, H, Lobe, At_Opening) then "true" else "false") & ",""across"":"
                          & "[" & Driver.Json.Number_Image (Tip_Across (Hands, H, Lobe, At_Opening) (1)) & ","
                          & Driver.Json.Number_Image (Tip_Across (Hands, H, Lobe, At_Opening) (2)) & "],""beat"":"
                          & Natural'Image (Natural (Tip_Beat (Hands, H, Lobe, At_Opening))) & ",""free_tip"":"
                          & Vector_Json (Tip_In_Tool (Hands, H, Lobe, At_Opening, Free).Mean) & ",""free_covariance"":"
                          & Covariance_Json (Tip_In_Tool (Hands, H, Lobe, At_Opening, Free)) & ",""free_confirmed"":"
                          & (if Tip_Confirmed (Hands, H, Lobe, At_Opening, Free) then "true" else "false")
                          & ",""free_tested"":"
                          & (if Tip_Tested (Hands, H, Lobe, At_Opening, Free) then "true" else "false")
                          & ",""free_beat"":"
                          & Natural'Image (Natural (Tip_Beat (Hands, H, Lobe, At_Opening, Free))) & ",""slides"":"
                          & Slides_Json (Slides (Hands, H, Lobe, At_Opening)) & ",""press"":"
                          & Vector_Json (Press_Direction (Hands, H, Lobe, At_Opening).Unit_Vector) & ",""reading"":"
                          & Readings_Json (Closer_Reading (Hands, H, At_Opening)) & "}");
               end loop;
               Append (Line_Text, "}");
            end loop;
            Append (Line_Text, "]}");
         end;
      end loop;
      Ada.Text_IO.Put_Line (Out_File, To_String (Line_Text) & "]}");
   end Write_Hands;

   procedure Write_Rays (O : Driver.Observations.Observation) is
      use Driver.Numerics.Arrays;
      Line_Text : Unbounded_String := To_Unbounded_String ("{""rays"":[");
   begin
      for E in 1 .. Driver.Robot.Eye_Count (Robot) loop
         declare
            Id   : constant Driver.Robot.Eye_Id := Driver.Robot.Eye_Id (E);
            Pose : constant Driver.Numerics.Rigid := Driver.Robot.Eye_Pose (Robot, Id, O).Pose;
            W    : constant Natural := Driver.Images.Width (O.Images (Id));
            H    : constant Natural := Driver.Images.Height (O.Images (Id));
         begin
            Append (Line_Text, (if E > 1 then ",[" else "["));
            --  A grid of pixels with a margin of one cell, each ray in the eye frame.
            for Gu in 1 .. 15 loop
               for Gv in 1 .. 11 loop
                  declare
                     Px : constant Driver.Images.Pixel := (U => Driver.Real (W) * Driver.Real (Gu) / 16.0,
                                                           V => Driver.Real (H) * Driver.Real (Gv) / 12.0);
                     Ray : constant Driver.Uncertain.Ray_Estimate := Driver.Robot.Ray (Robot, Id, O, Px);
                     D   : constant Driver.Numerics.Vec3 := Transpose (Pose.Rotation) * Ray.Direction.Unit_Vector;
                  begin
                     Append (Line_Text, (if Gu = 1 and then Gv = 1 then "" else ",") & "["
                             & Driver.Json.Number_Image (Px.U) & "," & Driver.Json.Number_Image (Px.V) & ","
                             & Driver.Json.Number_Image (D (1)) & "," & Driver.Json.Number_Image (D (2)) & ","
                             & Driver.Json.Number_Image (D (3)) & "]");
                  end;
               end loop;
            end loop;
            Append (Line_Text, "]");
         end;
      end loop;
      Ada.Text_IO.Put_Line (Out_File, To_String (Line_Text) & "]}");
   end Write_Rays;


   --  What a robot message gives the models, as in the main program: a new
   --  episode, and an observation with the command in effect while it was
   --  captured. The parts are given as the run gave them (Driver.Apart).
   type Input (Observed : Boolean := False) is record
      Reset : Boolean := False;
      Sent  : Driver.Commands.Command := Driver.Commands.Hold;
      case Observed is
         when True  => O : Driver.Observations.Observation;
         when False => null;
      end case;
   end record;

   --  Where a beat's time goes, written at the end: the driver's own work on
   --  each beat, layer by layer, without the robot's.
   type Part is (Parsing, Robot_Layer, Hand_Layer, World_Layer, Estimates_Apart);
   Spent : array (Part) of Duration := [others => 0.0];

   --  With --watch, the eye followed beat by beat (Driver.Robot.Eye_Watch).
   Watched : Natural := 0;

   procedure Robot_Part (M : Input) is
      Start : Duration;
   begin
      if M.Reset then
         Episodes := Episodes + 1;
         Driver.World.New_Episode (Scene);
      end if;
      if M.Observed then
         Driver.Services.Replay_Beat (M.O.Beat);
         Start := Driver.Clock.Seconds;
         Driver.Robot.Observe (Robot, M.O, M.Sent);
         Spent (Robot_Layer) := Spent (Robot_Layer) + (Driver.Clock.Seconds - Start);
         if Watched > 0 then
            Line (Core, "eye" & Watched'Image & ": " & Driver.Robot.Eye_Watch (Robot, Driver.Robot.Eye_Id (Watched)));
         end if;
      end if;
   end Robot_Part;

   procedure Rest (M : Input) is
      Start : Duration;
   begin
      if M.Observed then
         Start := Driver.Clock.Seconds;
         Driver.Robot.Hand.Observe (Hands, Robot, M.O, M.Sent);
         Spent (Hand_Layer) := Spent (Hand_Layer) + (Driver.Clock.Seconds - Start);
         Start := Driver.Clock.Seconds;
         Driver.World.Observe (Scene, Robot, Hands, M.O, M.Sent);
         Spent (World_Layer) := Spent (World_Layer) + (Driver.Clock.Seconds - Start);
         if Length (Estimates) > 0 and then Driver.Robot.Booted (Robot) then
            Write_Beat (M.O);
            Last_Obs := M.O;
         end if;
      end if;
   end Rest;

   function Due return Boolean is (Driver.Robot.Estimates_Due (Robot));

   procedure Compute is
      Start : constant Duration := Driver.Clock.Seconds;
   begin
      Driver.Robot.Compute_Estimates (Robot);
      Spent (Estimates_Apart) := Spent (Estimates_Apart) + (Driver.Clock.Seconds - Start);
   end Compute;

   procedure Never (E : Ada.Exceptions.Exception_Occurrence) is null;
   --  A replay computes in place; the estimator task of the main program
   --  never runs here.

   package Apart is new Driver.Apart (Input, Robot_Part, Rest, Due, Compute, Never);

   procedure Robot_Message (Data : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
   begin
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok then
         Line (Core, "an undecodable robot message is skipped");
         return;
      end if;
      if Driver.Protocol.Has_Observation (Req) and then not Known then
         Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
         if Known then
            Line (Core, "layout:" & ASCII.LF & Driver.Observations.Describe (Layout));
         end if;
      end if;
      declare
         Observed : constant Boolean := Driver.Protocol.Has_Observation (Req) and then Known;
         M        : Input (Observed);
      begin
         M.Reset := Req.Kind = Driver.Protocol.Reset;
         M.Sent := Sent;
         if Observed then
            declare
               Start : constant Duration := Driver.Clock.Seconds;
            begin
               Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), M.O);
               Spent (Parsing) := Spent (Parsing) + (Driver.Clock.Seconds - Start);
            end;
            Current := M.O;
            Beat := Beat + 1;
            Driver.Log.Stamp (Beat);
         end if;
         Apart.Replay_Message (M);
      end;
   end Robot_Message;

   procedure Taken_In is
      Ok : Boolean;
   begin
      Apart.Replay_Taken_In (Ok);
      if not Ok then
         Line (Core, "the recording gives the models a message where nothing was kept back; this code and the run disagree");
      end if;
   end Taken_In;

   procedure Back is
      Ok : Boolean;
   begin
      Apart.Replay_Back (Ok);
      if not Ok then
         Line (Core, "the models came back in the recording with messages never given to them; this code and the run disagree");
      end if;
   end Back;

   procedure Driver_Message (Data : Driver.Bytes.Byte_Array) is
      use Driver.Msgpack;
      Doc : Document;
      Ok  : Boolean;
      Action : Node;
   begin
      Decode (Data, Doc, Ok);
      Action := (if Ok and then Known
                 then Element (Doc, Lookup (Doc, Lookup (Doc, Root (Doc), "payload"), "result"), 1)
                 else No_Node);
      --  Acknowledgements (of update_obs, say) carry no action.
      if Action /= No_Node then
         Sent := Driver.Replies.Read_Action (Layout, Doc, Action);
         Commanded_Beats := Commanded_Beats + 1;
      end if;
   end Driver_Message;

   Live : Driver.Services.Service_Set := [others => False];   --  services named on the command line

   procedure Configure_Service (S : Driver.Services.Service; Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      if Colon = 0 then
         Line (Core, "a service address is HOST:PORT, not " & Address);
         return;
      end if;
      Driver.Services.Configure (S, Address (Address'First .. Colon - 1),
                                 Natural'Value (Address (Colon + 1 .. Address'Last)));
      Live (S) := True;
   end Configure_Service;

   --  Service records (Driver.Recording): a first line naming the call
   --  ("instrument 17 beat 42"), then the path or the status, then the body.

   type Service_Record is record
      Known       : Boolean := False;
      S           : Driver.Services.Service := Driver.Services.Instrument;
      Call        : Unbounded_String;   --  service and call number: pairs a reply with its request
      Second      : Unbounded_String;   --  the path of a request, the status of a reply
      Rest        : Unbounded_String;   --  the body
   end record;

   function Split (Data : Driver.Bytes.Byte_Array) return Service_Record is
      Text  : constant String := Driver.Bytes.To_String (Data);
      One   : constant Natural := Ada.Strings.Fixed.Index (Text, "" & ASCII.LF);
      Two   : constant Natural := (if One = 0 then 0 else Ada.Strings.Fixed.Index (Text, "" & ASCII.LF, One + 1));
      Result : Service_Record;
   begin
      if Two = 0 then
         return Result;   --  not a call record
      end if;
      declare
         Head   : constant String := Text (Text'First .. One - 1);
         Space  : constant Natural := Ada.Strings.Fixed.Index (Head, " ");
         Second : constant Natural := (if Space = 0 then 0 else Ada.Strings.Fixed.Index (Head, " ", Space + 1));
         Name   : constant String := (if Space = 0 then Head else Head (Head'First .. Space - 1));
      begin
         Result.Known := Name = "brain" or else Name = "instrument";
         Result.S := (if Name = "brain" then Driver.Services.Brain else Driver.Services.Instrument);
         Result.Call := To_Unbounded_String (if Second = 0 then Head else Head (Head'First .. Second - 1));
         Result.Second := To_Unbounded_String (Text (One + 1 .. Two - 1));
         Result.Rest := To_Unbounded_String (Text (Two + 1 .. Text'Last));
      end;
      return Result;
   end Split;

   package Request_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Service_Record);
   Requests : Request_Maps.Map;   --  recorded requests waiting for their replies, by call

   procedure Service_Request (Data : Driver.Bytes.Byte_Array) is
      Q : constant Service_Record := Split (Data);
   begin
      if Q.Known then
         Requests.Include (To_String (Q.Call), Q);
      end if;
   end Service_Request;

   procedure Service_Reply (Data : Driver.Bytes.Byte_Array) is
      A : constant Service_Record := Split (Data);
   begin
      if A.Known and then Requests.Contains (To_String (A.Call)) then
         declare
            Q  : constant Service_Record := Requests (To_String (A.Call));
            Ok : constant Boolean := To_String (A.Second) = "ok";
         begin
            Driver.Services.Replay_Reply
              (Q.S, To_String (Q.Second), To_String (Q.Rest),
               (Ok => Ok, Text => A.Rest, Why => (if Ok then Null_Unbounded_String else A.Second), Lasting => False));
            Requests.Delete (To_String (A.Call));
         end;
      end if;
   end Service_Reply;

   --  The body file the run read, given to the body where the run read it:
   --  after the observation the decider read it at, before that beat's reply.
   procedure File_Read (Data : Driver.Bytes.Byte_Array) is
      Text : constant String := Driver.Bytes.To_String (Data);
      Head : constant Natural := Ada.Strings.Fixed.Index (Text, "" & ASCII.LF);
      Named : constant String := "body ";   --  what the file is, then its path
   begin
      if Head > Text'First + Named'Length - 1 and then Text (Text'First .. Text'First + Named'Length - 1) = Named then
         declare
            Ok  : Boolean;
            Why : Unbounded_String;
         begin
            Driver.Robot.Load_Body_Text (Robot, Text (Head + 1 .. Text'Last), Ok, Why);
            Line (Core, "the run read its body file " & Text (Text'First + Named'Length .. Head - 1) & " here"
                  & (if Ok then ", " else ", and it does not reload: ") & To_String (Why));
         end;
      end if;
   end File_Read;

   procedure World_Write (Data : Driver.Bytes.Byte_Array) is
      Ok : Boolean;
   begin
      Driver.World.Replay_Write (Scene, Robot, Current, Data, Ok);
      if not Ok then
         Line (Core, "a write into the world this code cannot read is skipped");
      end if;
   end World_Write;

   procedure Report_Services is
   begin
      for S in Driver.Services.Service loop
         Line (Core, Driver.Services.Service'Image (S) & " replies: "
               & (if Live (S) then "from the live service named on the command line" else "from the recording"));
      end loop;
   end Report_Services;

begin
   if Argument_Count < 1 then
      Line (Core, "usage: replay RECORDING [--body FILE] [--estimates FILE] [--inst HOST:PORT] [--eye HOST:PORT]"
            & " [--watch EYE]");
      Set_Exit_Status (Failure);
      return;
   end if;
   Path := To_Unbounded_String (Argument (1));
   for I in 2 .. Argument_Count - 1 loop
      if Argument (I) = "--body" then
         Body_File := To_Unbounded_String (Argument (I + 1));
      elsif Argument (I) = "--estimates" then
         Estimates := To_Unbounded_String (Argument (I + 1));
      elsif Argument (I) = "--inst" then
         Configure_Service (Driver.Services.Instrument, Argument (I + 1));
      elsif Argument (I) = "--eye" then
         Configure_Service (Driver.Services.Brain, Argument (I + 1));
      elsif Argument (I) = "--watch" then
         Watched := Natural'Value (Argument (I + 1));
      end if;
   end loop;

   if Length (Estimates) > 0 then
      Ada.Text_IO.Create (Out_File, Ada.Text_IO.Out_File, To_String (Estimates));
   end if;
   Report_Services;
   Driver.Robot.Compute_Apart (Robot);
   Driver.Services.Start_Replay (Driver.Services."not" (Live));
   Driver.Recording.Open (R, To_String (Path), Opened);
   if not Opened then
      Line (Core, "cannot open the recording " & To_String (Path));
      Set_Exit_Status (Failure);
      return;
   end if;
   while More loop
      Driver.Recording.Next (R, Kind, Ns, Payload, More);
      exit when not More;
      case Kind is
         when Driver.Recording.Robot_Message  => Payload.Query (Robot_Message'Access);
         when Driver.Recording.Driver_Message => Payload.Query (Driver_Message'Access);
         when Driver.Recording.Service_Request => Payload.Query (Service_Request'Access);
         when Driver.Recording.Service_Reply   => Payload.Query (Service_Reply'Access);
         when Driver.Recording.File_Read       => Apart.Replay_Decider; Payload.Query (File_Read'Access);
         when Driver.Recording.Estimates_Asked => Apart.Replay_Decider; Compute;
         when Driver.Recording.World_Written   => Apart.Replay_Decider; Payload.Query (World_Write'Access);
         when Driver.Recording.Estimates_Apart => Apart.Replay_Apart;
         when Driver.Recording.Taken_In        => Taken_In;
         when Driver.Recording.Estimates_Back  => Back;
         when others                          => null;
      end case;
   end loop;
   Driver.Recording.Close (R);
   Apart.Replay_End;
   if Driver.Services.Unanswered > 0 then
      Line (Core, "the recording holds no reply to" & Natural'Image (Driver.Services.Unanswered)
            & " calls this replay made, the first at beat" & Natural'Image (Natural (Driver.Services.First_Unanswered))
            & ": from there the replay asks what the run did not, unless they are only the last beats' calls");
   end if;
   declare
      Text : Unbounded_String := To_Unbounded_String ("time spent:");
   begin
      for P in Part loop
         Append (Text, " " & Part'Image (P) & " " & Image (Driver.Real (Spent (P)), 1) & " s ("
                 & Image (1000.0 * Driver.Real (Spent (P)) / Driver.Real (Natural'Max (1, Beat)), 1) & " ms a beat)");
      end loop;
      Line (Core, To_String (Text));
   end;
   Line (Core, "replayed" & Natural'Image (Beat) & " beats," & Natural'Image (Commanded_Beats)
         & " actions," & Natural'Image (Episodes) & " episode resets");
   if Length (Body_File) > 0 then
      --  The live boot saves after a final estimate; without one here the
      --  body would miss what the last beats added (the final keyframes).
      Compute;
      Driver.Robot.Boot.Save (Robot, Hands, To_String (Body_File));
   end if;
   if Length (Estimates) > 0 then
      if Driver.Robot.Booted (Robot) then
         Write_Rays (Last_Obs);
         Write_Hands;
      end if;
      Ada.Text_IO.Close (Out_File);
   end if;
   Driver.Services.Shut_Down;
end Replay;
