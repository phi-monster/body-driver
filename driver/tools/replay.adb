--  replay RECORDING [--body FILE]
--
--  Feeds a recording (harness/record/wire_proxy.py format, uncompressed)
--  through every estimator exactly as the main loop does: each observation is
--  paired with the reply that followed it, read back as the command that was
--  sent, and given to the robot, hand and world estimators. At the end the
--  measured body is written to FILE, so it can be scored against simulator
--  truth. Nothing here decides anything; the recorded replies did.

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
   Pending  : Driver.Observations.Observation;
   Waiting  : Boolean := False;
   Beat     : Natural := 0;
   Episodes : Natural := 0;
   Commanded_Beats : Natural := 0;

   Robot : aliased Driver.Robot.Model;
   Hands : aliased Driver.Robot.Hand.Hands;
   Scene : aliased Driver.World.Scene;

   procedure Observe (Sent : Driver.Commands.Command) is
   begin
      Driver.Robot.Observe (Robot, Pending, Sent);
      Driver.Robot.Hand.Observe (Hands, Robot, Pending, Sent);
      Driver.World.Observe (Scene, Robot, Hands, Pending, Sent);
      Waiting := False;
      Beat := Beat + 1;
      if not Driver.Commands.Is_Hold (Sent) then
         Commanded_Beats := Commanded_Beats + 1;
      end if;
   end Observe;

   procedure Robot_Message (Data : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
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
         if Waiting then
            Observe (Driver.Commands.Hold);   --  no reply came for the previous one
         end if;
         if not Known then
            Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
            if Known then
               Line (Core, "layout:" & ASCII.LF & Driver.Observations.Describe (Layout));
            end if;
         end if;
         if Known then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), Pending);
            Waiting := True;
         end if;
      end if;
   end Robot_Message;

   procedure Driver_Message (Data : Driver.Bytes.Byte_Array) is
      use Driver.Msgpack;
      Doc : Document;
      Ok  : Boolean;
      Action : Node;
   begin
      if not Waiting then
         return;
      end if;
      Decode (Data, Doc, Ok);
      Action := (if Ok then Element (Doc, Lookup (Doc, Lookup (Doc, Root (Doc), "payload"), "result"), 1)
                 else No_Node);
      --  Acknowledgements (of update_obs, say) carry no action; the beat
      --  ends with the reply that does, or with the next observation.
      if Action /= No_Node then
         Observe (Driver.Replies.Read_Action (Layout, Doc, Action));
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
   if Waiting then
      Observe (Driver.Commands.Hold);
   end if;
   Line (Core, "replayed" & Natural'Image (Beat) & " beats," & Natural'Image (Commanded_Beats)
         & " with a command," & Natural'Image (Episodes) & " episode resets");
   if Length (Body_File) > 0 then
      Driver.Robot.Boot.Save (Robot, Hands, To_String (Body_File));
   end if;
end Replay;
