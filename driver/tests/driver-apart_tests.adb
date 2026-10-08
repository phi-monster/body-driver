with Ada.Exceptions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with GNAT.OS_Lib;
with Driver.Apart;
with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Observations;
with Driver.Recording;
with Driver.Robot;
with Driver.Tests;

package body Driver.Apart_Tests is

   use Ada.Strings.Unbounded;
   use Driver.Tests;

   --  A model that writes down what it is given: rN for the robot part of
   --  message N, sN for the rest of it, c for a computation of the estimates.
   --  The robot parts of messages First_Due and Second_Due make the estimates
   --  due, and a computation takes Compute_Time; each part takes Part_Time.

   type Msg is record
      N : Natural := 0;
   end record;

   protected Log is
      procedure Add (Step : String);
      procedure Clear;
      function Text return String;
   private
      T : Unbounded_String;
   end Log;

   protected body Log is
      procedure Add (Step : String) is
      begin
         Append (T, " " & Step);
      end Add;

      procedure Clear is
      begin
         T := Null_Unbounded_String;
      end Clear;

      function Text return String is (To_String (T));
   end Log;

   First_Due, Second_Due : Natural := 0;
   Compute_Time, Part_Time : Duration := 0.0;
   Is_Due   : Boolean := False with Atomic;
   Failures : Natural := 0 with Atomic;

   function Image (N : Natural) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));

   procedure Robot_Part (M : Msg) is
   begin
      Log.Add ("r" & Image (M.N));
      delay Part_Time;
      if M.N = First_Due or else M.N = Second_Due then
         Is_Due := True;
      end if;
   end Robot_Part;

   procedure Rest (M : Msg) is
   begin
      Log.Add ("s" & Image (M.N));
      delay Part_Time;
   end Rest;

   function Due return Boolean is (Is_Due);

   procedure Compute is
   begin
      Log.Add ("c");
      delay Compute_Time;
      Is_Due := False;
   end Compute;

   procedure Failed (E : Ada.Exceptions.Exception_Occurrence) is
      pragma Unreferenced (E);
   begin
      Failures := Failures + 1;
   end Failed;

   procedure Reset_Model is
   begin
      Log.Clear;
      Is_Due := False;
      Failures := 0;
   end Reset_Model;

   --  The steps the models go through when the estimates are computed in
   --  place: each message whole, a computation right after a robot part that
   --  made them due, and one after the message a decider asked in.
   function In_Place (Last, Asked : Natural) return String is
      T : Unbounded_String;
   begin
      for N in 1 .. Last loop
         Append (T, " r" & Image (N));
         if N = First_Due or else N = Second_Due then
            Append (T, " c");
         end if;
         Append (T, " s" & Image (N));
         if N = Asked then
            Append (T, " c");
         end if;
      end loop;
      return To_String (T);
   end In_Place;

   --  The main loop as body_driver drives Driver.Apart: messages 1, 2, ... a
   --  robot step apart, until at least Least have gone and the models are
   --  back, or Most have gone; in the first beat taken whole after message
   --  Ask_After (when it is not 0) a decider asks for the estimates. In_Time:
   --  the models were back by then. Longest: the longest the main loop took
   --  over a message. Afterwards, messages go slowly until the models are back,
   --  so the estimator task can end.
   procedure Run_Live
     (Least, Most : Natural;
      Ask_After   : Natural;
      Robot_Step  : Duration;
      Last        : out Natural;
      Asked       : out Natural;
      Longest     : out Duration;
      In_Time     : out Boolean)
   is
      package Live is new Driver.Apart (Msg, Robot_Part, Rest, Due, Compute, Failed);
      N : Natural := 0;

      procedure One (Step : Duration; Timed : Boolean) is
         Held  : Boolean;
         Went  : Boolean := False;
         Start : constant Duration := Driver.Clock.Seconds;
      begin
         N := N + 1;
         Live.Arrive (Held);
         Driver.Recording.Write_Shared (Driver.Recording.Robot_Message, Driver.Bytes.To_Bytes (Image (N)));
         if Held then
            Live.Hold_Back ((N => N));
         else
            Live.Take_In ((N => N), Went);
            if not Went and then Asked = 0 and then Ask_After > 0 and then N > Ask_After then
               --  The decider, in this beat's window (Driver.Robot.Estimate_Now).
               Driver.Recording.Write_Shared (Driver.Recording.Estimates_Asked, Driver.Bytes.To_Bytes (""));
               Is_Due := True;
               Asked := N;
            end if;
         end if;
         if Timed then
            Longest := Duration'Max (Longest, Driver.Clock.Seconds - Start);
         end if;
         if not Held and then not Went then
            Live.After_Reply;
         end if;
         delay Step;
      end One;

   begin
      Asked := 0;
      Longest := 0.0;
      loop
         One (Robot_Step, True);
         exit when N >= Most or else (N >= Least and then not Live.Is_Apart);
      end loop;
      In_Time := not Live.Is_Apart;
      while Live.Is_Apart loop
         One (Duration'Max (Robot_Step, 0.05), False);
      end loop;
      Last := N;
   end Run_Live;

   procedure Answers_While_Computing is
      Last, Asked : Natural;
      Longest     : Duration;
      In_Time     : Boolean;
   begin
      Reset_Model;
      First_Due := 5;
      Second_Due := 17;   --  kept back while the first computation runs
      Compute_Time := 1.0;
      Part_Time := 0.0;
      Run_Live (40, 20_000, 30, 0.005, Last, Asked, Longest, In_Time);
      Check (In_Time, "the models did not come back from the estimator");
      Check (Asked > 0, "no beat was taken whole after message 30 for the decider to ask in");
      Check (Longest < Compute_Time / 2, "the main loop took" & Longest'Image & " s over a message while a computation"
             & " takes" & Compute_Time'Image & " s");
      Check (Failures = 0, "the estimator failed");
      Check (Log.Text = In_Place (Last, Asked), "the models were not given the steps they are given in place:"
             & ASCII.LF & Log.Text (1 .. Natural'Min (Log.Text'Length, 400)) & ASCII.LF & "instead of"
             & ASCII.LF & In_Place (Last, Asked) (1 .. Natural'Min (In_Place (Last, Asked)'Length, 400)));
   end Answers_While_Computing;

   --  The robot sends faster than the models take its messages in: kept
   --  back without a bound, they would never come back.
   procedure Faster_Robot is
      Last, Asked : Natural;
      Longest     : Duration;
      In_Time     : Boolean;
   begin
      Reset_Model;
      First_Due := 3;
      Second_Due := 0;
      Compute_Time := 0.3;
      Part_Time := 0.005;   --  a message takes 10 ms to take in; the robot sends every 2 ms
      Run_Live (10, 3_000, 0, 0.002, Last, Asked, Longest, In_Time);
      Check (In_Time, "a robot that sends faster than the models take its messages in kept them apart for good"
             & " (3000 messages)");
      Check (Longest < Compute_Time, "the main loop took" & Longest'Image & " s over a message while catching up");
      Check (Failures = 0, "the estimator failed");
      Check (Log.Text = In_Place (Last, 0), "the models were not given the steps they are given in place");
   end Faster_Robot;

   --  A replay of what the main loop recorded gives the models the same
   --  steps, and a recording made with the estimates computed in place (no
   --  K, A, B) replays as it ran.
   procedure Replays_As_It_Ran is
      use type GNAT.OS_Lib.File_Descriptor;
      use type GNAT.OS_Lib.String_Access;
      FD          : GNAT.OS_Lib.File_Descriptor;
      Name        : GNAT.OS_Lib.String_Access;
      Gone        : Boolean;
      Last, Asked : Natural;
      Longest     : Duration;
      In_Time     : Boolean;
   begin
      GNAT.OS_Lib.Create_Temp_File (FD, Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the recording");
      GNAT.OS_Lib.Close (FD);
      if Name = null then
         return;
      end if;
      Reset_Model;
      First_Due := 5;
      Second_Due := 17;
      Compute_Time := 0.3;
      Part_Time := 0.0;
      Driver.Recording.Start_Shared (Name.all);
      Run_Live (40, 20_000, 30, 0.002, Last, Asked, Longest, In_Time);
      Driver.Recording.Stop_Shared;
      declare
         Ran      : constant String := Log.Text;
         package Replayed is new Driver.Apart (Msg, Robot_Part, Rest, Due, Compute, Failed);
         R        : Driver.Recording.Reader;
         Opened   : Boolean;
         More     : Boolean := True;
         Kind     : Driver.Recording.Record_Kind;
         Ns       : Long_Long_Integer;
         Payload  : Driver.Bytes.Buffer;
         Ok       : Boolean;
         Agree    : Boolean := True;
         Seen     : array (Driver.Recording.Record_Kind) of Natural := [others => 0];
      begin
         Reset_Model;
         Compute_Time := 0.0;
         Driver.Recording.Open (R, Name.all, Opened);
         Check (Opened, "the recording cannot be opened");
         while Opened and then More loop
            Driver.Recording.Next (R, Kind, Ns, Payload, More);
            if More then
               Seen (Kind) := Seen (Kind) + 1;
               case Kind is
                  when Driver.Recording.Robot_Message =>
                     Replayed.Replay_Message ((N => Natural'Value (Driver.Bytes.To_String (Payload.To_Array))));
                  when Driver.Recording.Estimates_Asked =>
                     Replayed.Replay_Decider;
                     Compute;
                  when Driver.Recording.Estimates_Apart =>
                     Replayed.Replay_Apart;
                  when Driver.Recording.Taken_In =>
                     Replayed.Replay_Taken_In (Ok);
                     Agree := Agree and then Ok;
                  when Driver.Recording.Estimates_Back =>
                     Replayed.Replay_Back (Ok);
                     Agree := Agree and then Ok;
                  when others =>
                     null;
               end case;
            end if;
         end loop;
         if Opened then
            Driver.Recording.Close (R);
         end if;
         Replayed.Replay_End;
         --  Twice: at message 5 and where the decider asked; message 17 made the
         --  estimates due again while it was kept back, within the first time.
         Check (Seen (Driver.Recording.Estimates_Apart) = 2 and then Seen (Driver.Recording.Estimates_Back) = 2,
                "the models went apart" & Seen (Driver.Recording.Estimates_Apart)'Image & " times and came back"
                & Seen (Driver.Recording.Estimates_Back)'Image & " times in the recording, not 2 and 2");
         Check (Agree, "the replay found a message given where nothing was kept back, or kept back and never given");
         Check (Log.Text = Ran, "the replay gave the models other steps than the run");
      end;
      GNAT.OS_Lib.Delete_File (Name.all, Gone);
      GNAT.OS_Lib.Free (Name);

      Reset_Model;
      Second_Due := 0;
      declare
         package In_Place_Run is new Driver.Apart (Msg, Robot_Part, Rest, Due, Compute, Failed);
      begin
         for N in 1 .. 10 loop
            In_Place_Run.Replay_Message ((N => N));
            if N = 8 then
               In_Place_Run.Replay_Decider;
               Compute;
            end if;
         end loop;
         In_Place_Run.Replay_End;
      end;
      Check (Log.Text = In_Place (10, 8), "a recording made in place did not replay as it ran:" & Log.Text);
   end Replays_As_It_Ran;

   --  A decider that asks for the estimates while they are computed apart
   --  answers its beat at once, takes no beat until they are in, and goes on
   --  in the first beat after.
   procedure Decider_Waits is
      After : Natural := 0 with Atomic;

      task Decider;
      task body Decider is
         B : Driver.Clock.Beat;
      begin
         Driver.Beats.Next (B);
         Driver.Beats.Wait_For_Estimates;
         After := Natural (Driver.Beats.Latest.Beat);
         Driver.Beats.Send (Driver.Commands.Hold);
      end Decider;

      O     : Driver.Observations.Observation;
      Took  : Boolean := False;
      Reply : Driver.Commands.Command;
   begin
      O.Beat := 1;
      while not Took loop
         Driver.Beats.Offer (1, O, Driver.Commands.Hold, Took);
         if not Took then
            delay 0.001;
         end if;
      end loop;
      Driver.Beats.Await (Reply);
      Check (Driver.Commands.Is_Hold (Reply), "a beat whose decider waits for the estimates was not answered hold");
      for Beat in 2 .. 3 loop
         delay 0.05;
         O.Beat := Driver.Clock.Beat (Beat);
         Driver.Beats.Offer (Driver.Clock.Beat (Beat), O, Driver.Commands.Hold, Took);
         Check (not Took, "the decider took beat" & Beat'Image & " before its estimates were in");
         if Took then
            Driver.Beats.Await (Reply);
         end if;
      end loop;
      Driver.Beats.Estimates_Adopted;
      O.Beat := 4;
      Took := False;
      while not Took loop
         Driver.Beats.Offer (4, O, Driver.Commands.Hold, Took);
         if not Took then
            delay 0.001;
         end if;
      end loop;
      Driver.Beats.Await (Reply);
      Check (After = 4, "the decider went on in beat" & After'Image & ", not in the first beat after its estimates");
   end Decider_Waits;

   --  Computed apart, the robot model marks its heavier estimates due where it
   --  would have computed them (the evidence doubled), computes them only when
   --  asked to, and in place nothing is ever due.
   procedure Robot_Marks_Due is
      Apart, In_Place : Driver.Robot.Model;
      O               : Driver.Observations.Observation;
      Due_At          : Unbounded_String;
   begin
      Driver.Robot.Compute_Apart (Apart);
      for B in 0 .. 8 loop
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => 0.0));
         Driver.Robot.Observe (Apart, O, Driver.Commands.Hold);
         Driver.Robot.Observe (In_Place, O, Driver.Commands.Hold);
         Check (not Driver.Robot.Estimates_Due (In_Place), "estimates computed in place were left due");
         if Driver.Robot.Estimates_Due (Apart) then
            Append (Due_At, " " & Image (B + 1));
            if B /= 1 then   --  at the second beat, they stay due through the third
               Driver.Robot.Compute_Estimates (Apart);
               Check (not Driver.Robot.Estimates_Due (Apart), "computed estimates are still due");
            end if;
         end if;
      end loop;
      --  Computed after beats 1, 3 (left due at 2), 6 (twice 3) and so on.
      Check (To_String (Due_At) = " 1 2 3 6", "the estimates were due after beats" & To_String (Due_At)
             & ", not 1 2 3 6");
   end Robot_Marks_Due;

   procedure Register is
   begin
      Driver.Tests.Register ("core.apart_answers",
                             "the main loop waits for the estimates computed apart, or the models are given other "
                             & "steps than in place", Answers_While_Computing'Access);
      Driver.Tests.Register ("core.apart_faster_robot",
                             "a robot sending faster than the models take its messages in keeps them apart for good",
                             Faster_Robot'Access);
      Driver.Tests.Register ("core.apart_replay",
                             "a replay gives the models other steps than the run, computed apart or in place",
                             Replays_As_It_Ran'Access);
      Driver.Tests.Register ("core.estimates_wait",
                             "a decider waiting for the estimates holds a beat, takes one before they are in, or "
                             & "goes on in a stale beat", Decider_Waits'Access);
      Driver.Tests.Register ("core.robot_marks_due",
                             "computed apart, the robot computes its estimates itself or marks them due at the "
                             & "wrong beats", Robot_Marks_Due'Access);
   end Register;

end Driver.Apart_Tests;
