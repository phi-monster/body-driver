--  replay RECORDING [--body FILE]
--
--  Feeds a recording (harness/record/wire_proxy.py format, uncompressed)
--  through every estimator exactly as the main loop does: each observation is
--  given to the robot, hand and world estimators together with the last
--  command sent before it arrived (read back from the recorded replies), the
--  command in effect while it was captured. At the end the measured body is
--  written to FILE, so it can be scored against simulator truth. Nothing here
--  decides anything; the recorded replies did.

with Ada.Command_Line;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Log;
with Driver.Msgpack;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Replies;
with Driver.Robot;
with Driver.Robot.Boot;
with Driver.Robot.Hand;
with Driver.World;

procedure Replay is

   use Ada.Command_Line;
   use Ada.Strings.Unbounded;
   use Driver.Log;
   use type Driver.Msgpack.Node;
   use type Driver.Protocol.Message_Kind;

   Path      : Unbounded_String;
   Body_File : Unbounded_String;

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
            Driver.Robot.Observe (Robot, O, Sent);
            Driver.Robot.Hand.Observe (Hands, Robot, O, Sent);
            Driver.World.Observe (Scene, Robot, Hands, O, Sent);
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

begin
   if Argument_Count < 1 then
      Line (Core, "usage: replay RECORDING [--body FILE]");
      Set_Exit_Status (Failure);
      return;
   end if;
   Path := To_Unbounded_String (Argument (1));
   for I in 2 .. Argument_Count - 1 loop
      if Argument (I) = "--body" then
         Body_File := To_Unbounded_String (Argument (I + 1));
      end if;
   end loop;

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
         when others                          => null;
      end case;
   end loop;
   Driver.Recording.Close (R);
   Line (Core, "replayed" & Natural'Image (Beat) & " beats," & Natural'Image (Commanded_Beats)
         & " actions," & Natural'Image (Episodes) & " episode resets");
   if Length (Body_File) > 0 then
      Driver.Robot.Boot.Save (Robot, Hands, To_String (Body_File));
   end if;
end Replay;
