--  replay RECORDING [--body FILE] [--estimates FILE] [--inst HOST:PORT] [--eye HOST:PORT]
--
--  Feeds a recording (Driver.Recording format, uncompressed)
--  through every estimator exactly as the main loop does: each observation is
--  given to the robot, hand and world estimators together with the last
--  command sent before it arrived (read back from the recorded replies), the
--  command in effect while it was captured. At the end the measured body is
--  written to the body file. With --estimates, every beat after boot writes
--  one JSON line with each arm's tool pose and each eye's pose (row-major 4 x 4,
--  world frame), and the last line holds, for each eye, the lines of sight of a
--  grid of pixels in the eye's own frame, so the estimates can be scored
--  against simulator truth whatever model produced them. Nothing here decides
--  anything; the recorded replies did.
--
--  The estimators' service calls are answered as Driver.Services describes
--  for a replay: from the recording's own replies when it holds any for that
--  service, otherwise by the live service at --inst or --eye, and in both
--  cases ready from the beat after the call, so the result never depends on
--  how fast the replay runs.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Strings.Unbounded;
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
      Line_Text : Unbounded_String := To_Unbounded_String ("{""beat"":" & Natural'Image (Beat) & ",""tools"":[");
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


   procedure Robot_Message (Data : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
      O   : Driver.Observations.Observation;
   begin
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok then
         Line (Core, "an undecodable robot message is skipped");
         return;
      end if;
      if Req.Kind = Driver.Protocol.Reset then
         Episodes := Episodes + 1;
         Driver.World.New_Episode (Scene);
      end if;
      if Driver.Protocol.Has_Observation (Req) then
         if not Known then
            Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
            if Known then
               Line (Core, "layout:" & ASCII.LF & Driver.Observations.Describe (Layout));
            end if;
         end if;
         if Known then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), O);
            Driver.Services.Replay_Beat (Driver.Clock.Beat (Beat));
            Driver.Robot.Observe (Robot, O, Sent);
            Driver.Robot.Hand.Observe (Hands, Robot, O, Sent);
            Driver.World.Observe (Scene, Robot, Hands, O, Sent);
            if Length (Estimates) > 0 and then Driver.Robot.Booted (Robot) then
               Write_Beat (O);
               Last_Obs := O;
            end if;
            Beat := Beat + 1;
         end if;
      end if;
   end Robot_Message;

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

   procedure Configure_Service (S : Driver.Services.Service; Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      if Colon = 0 then
         Line (Core, "a service address is HOST:PORT, not " & Address);
         return;
      end if;
      Driver.Services.Configure (S, Address (Address'First .. Colon - 1),
                                 Natural'Value (Address (Colon + 1 .. Address'Last)));
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
               (Ok => Ok, Text => A.Rest, Why => (if Ok then Null_Unbounded_String else A.Second)));
            Requests.Delete (To_String (A.Call));
         end;
      end if;
   end Service_Reply;

   function Recorded_Services return Driver.Services.Service_Set is
      --  Which services the recording holds replies of: those are replayed,
      --  the others answered live.
      Scan    : Driver.Recording.Reader;
      Found   : Driver.Services.Service_Set := [others => False];
      Ok      : Boolean;
      Kind    : Driver.Recording.Record_Kind;
      Ns      : Long_Long_Integer;
      Payload : Driver.Bytes.Buffer;

      procedure Note (Data : Driver.Bytes.Byte_Array) is
         A : constant Service_Record := Split (Data);
      begin
         if A.Known then
            Found (A.S) := True;
         end if;
      end Note;
   begin
      Driver.Recording.Open (Scan, To_String (Path), Ok);
      while Ok loop
         Driver.Recording.Next (Scan, Kind, Ns, Payload, Ok);
         exit when not Ok;
         if Driver.Recording."=" (Kind, Driver.Recording.Service_Reply) then
            Payload.Query (Note'Access);
         end if;
      end loop;
      Driver.Recording.Close (Scan);
      for S in Driver.Services.Service loop
         Line (Core, Driver.Services.Service'Image (S) & " replies: "
               & (if Found (S) then "from the recording" else "from the live service, when it has an address"));
      end loop;
      return Found;
   end Recorded_Services;

begin
   if Argument_Count < 1 then
      Line (Core, "usage: replay RECORDING [--body FILE] [--estimates FILE] [--inst HOST:PORT] [--eye HOST:PORT]");
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
      end if;
   end loop;

   if Length (Estimates) > 0 then
      Ada.Text_IO.Create (Out_File, Ada.Text_IO.Out_File, To_String (Estimates));
   end if;
   Driver.Services.Start_Replay (Recorded_Services);
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
         when others                          => null;
      end case;
   end loop;
   Driver.Recording.Close (R);
   Line (Core, "replayed" & Natural'Image (Beat) & " beats," & Natural'Image (Commanded_Beats)
         & " actions," & Natural'Image (Episodes) & " episode resets");
   if Length (Body_File) > 0 then
      Driver.Robot.Boot.Save (Robot, Hands, To_String (Body_File));
   end if;
   if Length (Estimates) > 0 then
      if Driver.Robot.Booted (Robot) then
         Write_Rays (Last_Obs);
      end if;
      Ada.Text_IO.Close (Out_File);
   end if;
end Replay;
