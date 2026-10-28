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
--
--  The heavier estimates (roles, kinematics) are computed by a task of their
--  own (Driver.Apart): a recompute grows to minutes, and the robot is answered
--  every beat. While the models are apart every message is answered with
--  Hold and kept back, then given to the models in order, as it would have
--  been; the decider is offered beats again once the models are back.
--
--  A failure the driver cannot go on from (an exception in the main loop or
--  the boot) is written to the log whole and ends the program with a failure
--  status, the recording closed: the tasks still waiting (the decider, the
--  service workers) would otherwise keep it alive, holding the robot with
--  nothing more in the log.

with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Task_Identification;
with Driver.Action;
with Driver.Apart;
with Driver.Beats;
with Driver.Brain;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Log;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Replies;
with Driver.Twins;
with Driver.Robot;
with Driver.Robot.Boot;
with Driver.Robot.Hand;
with Driver.Services;
with Driver.Wire;
with Driver.World;
with GNAT.OS_Lib;

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

   --  Whether a flag is given (an option with no value).
   function Flag (Name : String) return Boolean is
     (for some I in 1 .. Ada.Command_Line.Argument_Count => Ada.Command_Line.Argument (I) = Name);

   procedure Configure (S : Driver.Services.Service; Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      if Colon > 0 then
         Driver.Services.Configure (S, Address (Address'First .. Colon - 1),
                                    Natural'Value (Address (Colon + 1 .. Address'Last)));
      end if;
   end Configure;

   procedure Fail (What : String; E : Ada.Exceptions.Exception_Occurrence) is
   begin
      Line (Core, What & ": " & Ada.Exceptions.Exception_Information (E));
      Driver.Recording.Stop_Shared;
      GNAT.OS_Lib.OS_Exit (Integer (Ada.Command_Line.Failure));
   end Fail;

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
   --  failed episode does not take the later ones with it; a failed boot ends
   --  the program (Fail).
   task body Decider is
      Ok          : Boolean;
      Instruction : Unbounded_String;
      Context     : Driver.Action.Context (Robot'Access, Hands'Access, Scene'Access);
      --  The scene, held while the body was booted (Driver.World.Boot_Begins), is measured from now on.
      procedure Boot_Ended is
      begin
         Driver.World.Boot_Ends (Scene);
      end Boot_Ended;
   begin
      --  When the program ends before it starts (a port that cannot be bound), the
      --  decider ends with it rather than keeping the program alive.
      select
         accept Start;
      or
         terminate;
      end select;
      Driver.Robot.Boot.Run (Robot, Hands, To_String (Body_File), Ok);
      if Ok then
         Driver.Beats.Within_A_Beat (Boot_Ended'Access);
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
         Fail ("the boot failed", E);
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
   Twins    : Driver.Twins.Watch;   --  which group of each command key's pair is its reading

   --  Where the main loop's time goes, logged at every doubling of the beats: waiting for the robot's next message,
   --  the message's decoding and parsing, its taking in by the models (or keeping back), the decider's command for its
   --  beat, and the reply. A boot's speed is its beats a second, and these say which part holds them: the fake robot's
   --  own clock of a beat could not tell them apart (the big machine, 10-22: 5 to 9 ms a beat, the driver's, of which
   --  the Mac's samples put 2.4 in the handling).
   type Phase is (Waiting, Arriving, Parsing, Keeping, Modelling, Deciding, Replying);
   Spent       : array (Phase) of Duration := [others => 0.0];
   Counted     : array (Phase) of Natural := [others => 0];   --  the messages each phase's time is over
   Messages    : Natural := 0;
   Kept_Count  : Natural := 0;
   Next_Report : Positive := 1;
   Mark        : Duration := 0.0;   --  when the phase under way began

   procedure Account (P : Phase) is
      Now : constant Duration := Driver.Clock.Seconds;
   begin
      Spent (P) := Spent (P) + (Now - Mark);
      Counted (P) := Counted (P) + 1;
      Mark := Now;
   end Account;

   --  And of a beat taken in by the main loop, the robot's models, the hands' and the world's (the estimator takes the
   --  messages kept back in as well, and is not timed here).
   type Model_Part is (Robot_Model, Hands_Model, World_Model);
   Part_Spent : array (Model_Part) of Duration := [others => 0.0];
   Part_Count : array (Model_Part) of Natural := [others => 0];
   Main_Loop  : constant Ada.Task_Identification.Task_Id := Ada.Task_Identification.Current_Task;

   procedure Timed (P : Model_Part; Began : Duration) is
      use type Ada.Task_Identification.Task_Id;
   begin
      if Ada.Task_Identification.Current_Task = Main_Loop then
         Part_Spent (P) := Part_Spent (P) + (Driver.Clock.Seconds - Began);
         Part_Count (P) := Part_Count (P) + 1;
      end if;
   end Timed;

   procedure Report_Phases is
      function Ms (P : Phase) return String is
        (Driver.Log.Image (Driver.Real (Spent (P)) * 1000.0 / Driver.Real (Natural'Max (1, Counted (P))), 3));
      function Part_Ms (P : Model_Part) return String is
        (Driver.Log.Image (Driver.Real (Part_Spent (P)) * 1000.0 / Driver.Real (Natural'Max (1, Part_Count (P))), 3));
   begin
      Line (Core, "the main loop over" & Beat'Image & " beats," & Messages'Image & " messages (" & Kept_Count'Image
            & " kept back), ms a message of each kind: waiting " & Ms (Waiting) & ", catching up " & Ms (Arriving) & ", parsing "
            & Ms (Parsing) & ", kept back " & Ms (Keeping) & ", taken in " & Ms (Modelling) & ", decided " & Ms (Deciding)
            & ", reply " & Ms (Replying));
      Line (Core, "a beat the main loop took in, ms: the robot's models " & Part_Ms (Robot_Model) & ", the hands' "
            & Part_Ms (Hands_Model) & ", the world's " & Part_Ms (World_Model));
   end Report_Phases;

   --  What a robot message gives the models: a new episode, and an
   --  observation with the command in effect while it was captured (the last
   --  one sent before it arrived).
   type Input (Observed : Boolean := False) is record
      Reset : Boolean := False;
      Sent  : Driver.Commands.Command := Driver.Commands.Hold;
      case Observed is
         when True  => O : Driver.Observations.Observation;
         when False => null;
      end case;
   end record;

   procedure Robot_Part (M : Input) is
   begin
      if M.Reset then
         Driver.World.New_Episode (Scene);
         Driver.Beats.New_Episode;
      end if;
      if M.Observed then
         declare
            Began : constant Duration := Driver.Clock.Seconds;
         begin
            Driver.Robot.Observe (Robot, M.O, M.Sent);
            Timed (Robot_Model, Began);
         end;
      end if;
   end Robot_Part;

   procedure Rest (M : Input) is
   begin
      if M.Observed then
         declare
            Began : Duration := Driver.Clock.Seconds;
         begin
            Driver.Robot.Hand.Observe (Hands, Robot, M.O, M.Sent);
            Timed (Hands_Model, Began);
            Began := Driver.Clock.Seconds;
            Driver.World.Observe (Scene, Robot, Hands, M.O, M.Sent);
            Timed (World_Model, Began);
         end;
         Driver.Beats.Hear (To_String (M.O.Instruction));
         Tasks.Offer (To_String (M.O.Instruction), Driver.Beats.Episode);
      end if;
   end Rest;

   function Due return Boolean is (Driver.Robot.Estimates_Due (Robot) or else Driver.World.Measure_Due (Scene));

   procedure Compute is
   begin
      if Driver.Robot.Estimates_Due (Robot) then
         Driver.Robot.Compute_Estimates (Robot);
      end if;
      if Driver.World.Measure_Due (Scene) then
         Driver.World.Measure (Scene, Robot);
      end if;
   end Compute;

   procedure Estimates_Failed (E : Ada.Exceptions.Exception_Occurrence) is
   begin
      Fail ("the estimates failed", E);
   end Estimates_Failed;

   procedure Lighten (M : in out Input) is
   begin
      if M.Observed then
         for C in M.O.Images.First_Index .. M.O.Images.Last_Index loop
            M.O.Images.Replace_Element (C, Driver.Images.No_Image);
         end loop;
      end if;
   end Lighten;

   package Apart is new Driver.Apart (Input, Robot_Part, Rest, Due, Compute, Estimates_Failed, Lighten);

   In_Beat : Boolean := False;   --  --in-beat: the main loop waits for the models while they are apart (Wait_Back)

   procedure Handle (Data : Driver.Bytes.Byte_Array) is
      Req        : Driver.Protocol.Request;
      Took       : Boolean;
      Held       : Boolean;
      Went_Apart : Boolean := False;
      Place      : Positive;
   begin
      Apart.Arrive (Held);
      Account (Arriving);
      Driver.Recording.Write_Shared (Driver.Recording.Robot_Message, Data, Place);
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok then
         Line (Core, "a robot message that is not a protocol message was ignored");
         return;
      end if;
      if Req.Kind = Driver.Protocol.Reset then
         Driver.Replies.New_Episode (Holds);
      end if;
      if Driver.Protocol.Has_Observation (Req) and then not Known then
         Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
         if Known then
            Line (Core, "the robot reports:" & ASCII.LF & Driver.Observations.Describe (Layout));
         else
            Line (Core, "waiting: the driver needs at least one camera and one group of readings, got "
                  & Driver.Observations.Describe (Layout));
         end if;
      end if;
      declare
         Observed : constant Boolean := Driver.Protocol.Has_Observation (Req) and then Known;
         M        : Input (Observed);
      begin
         M.Reset := Req.Kind = Driver.Protocol.Reset;
         M.Sent := Sent;
         if Observed then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), M.O);
            Driver.Twins.Order (Twins, Layout, M.O, Sent);
            Current := M.O;
            Have_Obs := True;
         end if;
         Account (Parsing);
         if Held then
            --  The models are the estimator's: the robot holds, the message waits.
            Apart.Hold_Back (M);
            Pending := Driver.Commands.Hold;
            Account (Keeping);
         else
            --  A step for the services' replies, begun with this message's record.
            Driver.Services.Step_Begins (Place);
            Apart.Take_In (M, Went_Apart);
            if Went_Apart and then In_Beat then
               Apart.Wait_Back;
            end if;
            Account (Modelling);
            if Went_Apart then
               Pending := Driver.Commands.Hold;
            elsif Observed then
               Driver.Beats.Offer (Driver.Clock.Beat (Beat), Current, Sent, Took);
               if Took then
                  Driver.Beats.Await (Pending);
               else
                  Pending := Driver.Commands.Hold;
               end if;
            end if;
            Account (Deciding);
         end if;
         if Observed then
            Beat := Beat + 1;
            Driver.Log.Stamp (Beat);
         end if;
      end;
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
      if not Held and then not Went_Apart then
         Apart.After_Reply;
         if In_Beat then
            Apart.Wait_Back;
         end if;
      end if;
      Account (Replying);
      Messages := Messages + 1;
      Kept_Count := Kept_Count + Boolean'Pos (Held);
      if Beat >= Next_Report then
         Report_Phases;
         Next_Report := 2 * Next_Report;
      end if;
   end Handle;

begin
   if Option ("--listen") = "" then
      Line (Core, "usage: body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE]"
            & " [--record FILE] [--in-beat] [--once]");
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
   --  The estimates are made apart from the main loop, which holds the robot still while they are made: a real
   --  robot's time goes on while the driver computes. A robot that waits for every answer and keeps no time of its own
   --  (a simulator run as fast as the driver answers: --in-beat) has the main loop wait for them instead (Wait_Back),
   --  the robot's and the world's alike, so it holds still for no beats at all: there, a held beat is a picture
   --  rendered, sent and dropped. The hall's unpaced rounds (10-28) spent a third of their beats so, Panda seed 1a past
   --  11800 of its 12000 in five minutes; made in the beat for the robot's estimates only, the world's first measure
   --  still kept Panda 2321 beats back (f7s2), and taking those beats in cost another 9.09 s. The driver's own time is
   --  then its beats and its estimates' seconds, both in its log.
   In_Beat := Flag ("--in-beat");
   if In_Beat then
      Line (Core, "estimates within the beat that asks for them (--in-beat): the main loop waits for them");
   end if;
   Driver.Robot.Compute_Apart (Robot);
   Driver.World.Boot_Begins (Scene);
   Decider.Start;
   loop
      Driver.Wire.Accept_Client (Connection, Ok);
      if Ok then
         Line (Core, "the robot connected");
         Driver.Recording.Write_Shared (Driver.Recording.Connection, Driver.Bytes.To_Bytes (""));
         Mark := Driver.Clock.Seconds;
         loop
            Driver.Wire.Receive (Connection, Kind, Message);
            Account (Waiting);
            exit when Kind = Driver.Wire.Closed;
            Message.Query (Handle'Access);
         end loop;
         --  --once: the driver ends when its robot goes, by the process's own exit (a profiled build writes its
         --  profile there, and a run's killed driver writes none); the tasks are not waited for.
         if Flag ("--once") then
            Line (Core, "the robot disconnected; ending (--once)");
            GNAT.OS_Lib.OS_Exit (0);
         end if;
         Line (Core, "the robot disconnected; waiting for it on the same port");
      end if;
   end loop;
exception
   when E : others =>
      Fail ("the driver failed", E);
end Body_Driver;
