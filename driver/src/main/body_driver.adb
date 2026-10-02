--  body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE] [--record FILE]
--
--    --listen  the WebSocket port the robot connects to (docs/body-protocol.md)
--    --eye     the brain service (docs/brain-service.md)
--    --inst    the instrument service (docs/instrument-service.md)
--    --body    the body file to reload from and store into (docs/body-file.md)
--    --record  write everything that crosses the driver's boundary to FILE
--
--  The main loop owns the connection to the robot. Every observation is
--  parsed, given to every estimator together with the last command sent
--  before it, then offered to the decider task (boot, then rounds with the
--  brain); the reply carries the decider's command, or Hold when the decider
--  is busy. A disconnected robot may reconnect on the same port; nothing
--  measured is lost.

with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Driver.Action;
with Driver.Beats;
with Driver.Brain;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Log;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Replies;
with Driver.Robot;
with Driver.Robot.Boot;
with Driver.Robot.Hand;
with Driver.Services;
with Driver.Wire;
with Driver.World;

procedure Body_Driver is

   use Ada.Strings.Unbounded;
   use Driver.Log;
   use type Driver.Protocol.Message_Kind;
   use type Driver.Wire.Message_Kind;

   Robot : aliased Driver.Robot.Model;
   Hands : aliased Driver.Robot.Hand.Hands;
   Scene : aliased Driver.World.Scene;

   Port      : Natural := 0;
   Body_File : Unbounded_String;
   Record_To : Unbounded_String;

   function Option (Name : String) return String is
   begin
      for I in 1 .. Ada.Command_Line.Argument_Count - 1 loop
         if Ada.Command_Line.Argument (I) = Name then
            return Ada.Command_Line.Argument (I + 1);
         end if;
      end loop;
      return "";
   end Option;

   procedure Configure (S : Driver.Services.Service; Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      if Colon > 0 then
         Driver.Services.Configure (S, Address (Address'First .. Colon - 1),
                                    Natural'Value (Address (Colon + 1 .. Address'Last)));
      end if;
   end Configure;

   --  The task sentence of the current episode, handed to the decider once.
   --  An idle decider takes no beats, so the main loop holds on its own.
   protected Tasks is
      procedure Offer (Instruction : String; Episode : Natural);
      entry Wait (Instruction : out Unbounded_String);
   private
      Text   : Unbounded_String;
      Fresh  : Boolean := False;
      Handed : Integer := -1;   --  the episode whose sentence was handed out
   end Tasks;

   protected body Tasks is
      procedure Offer (Instruction : String; Episode : Natural) is
      begin
         if Instruction'Length > 0 and then Episode /= Handed then
            Text := To_Unbounded_String (Instruction);
            Fresh := True;
            Handed := Episode;
         end if;
      end Offer;

      entry Wait (Instruction : out Unbounded_String) when Fresh is
      begin
         Instruction := Text;
         Fresh := False;
      end Wait;
   end Tasks;

   task Decider is
      entry Start;
   end Decider;

   --  A task that ends on an exception ends silently, and the robot would hold
   --  for good with nothing in the log: every failure is written out whole. A
   --  failed episode does not take the later ones with it.
   task body Decider is
      Ok          : Boolean;
      Instruction : Unbounded_String;
      Context     : Driver.Action.Context (Robot'Access, Hands'Access, Scene'Access);
   begin
      accept Start;
      Driver.Robot.Boot.Run (Robot, Hands, To_String (Body_File), Ok);
      if Ok then
         loop
            Tasks.Wait (Instruction);
            begin
               Driver.Brain.Run_Episode (Context, To_String (Instruction));
            exception
               when E : others =>
                  Driver.Beats.Release;
                  Line (Core, "the episode failed; waiting for the next one: " & Ada.Exceptions.Exception_Information (E));
            end;
         end loop;
      else
         Line (Core, "the body could not be measured; holding still");
      end if;
   exception
      when E : others =>
         Driver.Beats.Release;
         Line (Core, "the boot failed; holding still: " & Ada.Exceptions.Exception_Information (E));
   end Decider;

   Connection : Driver.Wire.Connection;
   Ok         : Boolean;
   Kind       : Driver.Wire.Message_Kind;
   Message    : Driver.Bytes.Buffer;
   Reply      : Driver.Bytes.Buffer;

   Layout   : Driver.Observations.Layout;
   Known    : Boolean := False;
   Current  : Driver.Observations.Observation;
   Have_Obs : Boolean := False;
   Beat     : Natural := 0;
   Pending  : Driver.Commands.Command := Driver.Commands.Hold;   --  the decider's command for this beat
   Sent     : Driver.Commands.Command := Driver.Commands.Hold;   --  the last command sent
   Holds    : Driver.Replies.State;

   procedure Handle (Data : Driver.Bytes.Byte_Array) is
      Req  : Driver.Protocol.Request;
      Took : Boolean;
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Robot_Message, Data);
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok then
         Line (Core, "a robot message that is not a protocol message was ignored");
         return;
      end if;
      if Req.Kind = Driver.Protocol.Reset then
         Driver.Replies.New_Episode (Holds);
         Driver.World.New_Episode (Scene);
         Driver.Beats.New_Episode;
      end if;
      if Driver.Protocol.Has_Observation (Req) then
         if not Known then
            Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
            if Known then
               Line (Core, "the robot reports:" & ASCII.LF & Driver.Observations.Describe (Layout));
            else
               Line (Core, "waiting: the driver needs at least one camera and one group of readings, got "
                     & Driver.Observations.Describe (Layout));
            end if;
         end if;
         if Known then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), Current);
            Have_Obs := True;
            Driver.Robot.Observe (Robot, Current, Sent);
            Driver.Robot.Hand.Observe (Hands, Robot, Current, Sent);
            Driver.World.Observe (Scene, Robot, Hands, Current, Sent);
            Driver.Beats.Hear (To_String (Current.Instruction));
            Tasks.Offer (To_String (Current.Instruction), Driver.Beats.Episode);
            Driver.Beats.Offer (Driver.Clock.Beat (Beat), Current, Sent, Took);
            if Took then
               Driver.Beats.Await (Pending);
            else
               Pending := Driver.Commands.Hold;
            end if;
            Beat := Beat + 1;
         end if;
      end if;
      if Driver.Protocol.Wants_Action (Req) and then Known and then Have_Obs then
         declare
            Action : Driver.Bytes.Buffer;
         begin
            Driver.Replies.Write_Action (Holds, Layout, Current, Pending, Action, Sent);
            Driver.Protocol.Encode_Reply (Req, True, Action.To_Array, Reply);
         end;
      else
         Driver.Protocol.Encode_Reply (Req, False, Driver.Bytes.To_Bytes (""), Reply);
      end if;
      Driver.Recording.Write_Shared (Driver.Recording.Driver_Message, Reply.To_Array);
      Driver.Wire.Send (Connection, Reply.To_Array, Ok);
   end Handle;

begin
   if Option ("--listen") = "" then
      Line (Core, "usage: body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE]"
            & " [--record FILE]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Port := Natural'Value (Option ("--listen"));
   Body_File := To_Unbounded_String (Option ("--body"));
   Record_To := To_Unbounded_String (Option ("--record"));
   Configure (Driver.Services.Brain, Option ("--eye"));
   Configure (Driver.Services.Instrument, Option ("--inst"));
   if Length (Record_To) > 0 then
      Driver.Recording.Start_Shared (To_String (Record_To));
   end if;

   Driver.Wire.Listen (Connection, Port, Ok);
   if not Ok then
      Line (Core, "cannot listen on port" & Natural'Image (Port));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Line (Core, "listening on port" & Natural'Image (Port));
   Decider.Start;
   loop
      Driver.Wire.Accept_Client (Connection, Ok);
      if Ok then
         Line (Core, "the robot connected");
         Driver.Recording.Write_Shared (Driver.Recording.Connection, Driver.Bytes.To_Bytes (""));
         loop
            Driver.Wire.Receive (Connection, Kind, Message);
            exit when Kind = Driver.Wire.Closed;
            Message.Query (Handle'Access);
         end loop;
         Line (Core, "the robot disconnected; waiting for it on the same port");
      end if;
   end loop;
end Body_Driver;
