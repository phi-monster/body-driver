with Ada.Characters.Handling;
with Driver.Alignment;
with Ada.Containers.Vectors;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Beats;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Robot.Body_File;
with Driver.Robot.Kinematics;
with Driver.Robot.Lag;
with Driver.Robot.Channels;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;
with Driver.Robot.Regression;
with Driver.Uncertain;

package body Driver.Robot.Boot is

   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Observations.Group_Id;

   package Real_IO is new Ada.Text_IO.Float_IO (Real);

   --  A number in the log with its magnitude: probe sizes span many decades.
   function Scientific (X : Real) return String is
      S : String (1 .. 32);
   begin
      Real_IO.Put (S, X, Aft => 2, Exp => 3);
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Scientific;

   function Hold_After_A_Push (M : Model; G : Group_Id; Guess : Natural) return Natural is
      Every_Known : Boolean := Eye_Count (M) > 0;
      Longest     : Natural := 0;
   begin
      for E in 1 .. Eye_Count (M) loop
         if Response (M, G, Eye_Id (E)) /= Nothing then
            if Lag_Known (M, Eye_Id (E)) then
               Longest := Natural'Max (Longest, Natural'Max (0, Image_Lag (M, Eye_Id (E))));
            else
               Every_Known := False;
            end if;
         end if;
      end loop;
      return Pause_After_A_Push (Every_Known, Longest, Guess);
   end Hold_After_A_Push;

   function Grew (Before, After, Cells : Integer) return Boolean is
     (After > Before and then Cells >= After
      and then Driver.Robot.Regression.Count_Significant
                 (After - Before, Cells - Before, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)));

   --  A repeatable uniform generator (Park and Miller's minimal standard),
   --  for push orders that no other group's repeats.
   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   type Push is record
      Channel : Positive;
      Sign    : Real;
   end record;

   type Push_Array is array (Positive range <>) of Push;

   procedure Shuffle (P : in out Push_Array; G : in out Generator) is
   begin
      for I in reverse P'First + 1 .. P'Last loop
         declare
            J : constant Positive := P'First + Natural (Real'Floor (Uniform (G) * Real (I - P'First + 1)));
            T : constant Push := P (I);
         begin
            P (I) := P (Natural'Min (J, I));
            P (Natural'Min (J, I)) := T;
         end;
      end loop;
   end Shuffle;

   procedure Run (M : in out Model; H : in out Driver.Robot.Hand.Hands; Body_File : String; Ok : out Boolean) is
      --  The decider reads the models only in held beats (Driver.Beats): every
      --  read below is inside one, through Within_A_Beat or Motion.
      Waited : Natural;
      --  One generator for every order the boot draws, so no two repeat.
      Rng    : Generator;

      procedure Estimate is
      begin
         Estimate_Now (M);
      end Estimate;

      --  What the body has measured goes to its file after each stage that
      --  completes, not once at the end: a boot that fails later has kept it,
      --  and the next one reloads it (docs/body-file.md). Without a file,
      --  nothing is kept.
      procedure Keep (Stage : String) is
         Written : Boolean := True;
         procedure Write is
         begin
            Driver.Robot.Body_File.Write (M, Body_File, Written);
         end Write;
      begin
         if Body_File'Length > 0 then
            Driver.Beats.Within_A_Beat (Write'Access);
            Driver.Log.Line
              (Driver.Log.Robot,
               (if Written then "boot: the body is kept in " & Body_File & " after " & Stage
                else "the body file " & Body_File & " cannot be written"));
         end if;
      end Keep;

      --  What the group holds now (Motion.Hold_Of): moves are taken from here.
      function Holds_Of (G : Group_Id; Size : Natural) return Real_Array is
         Result : Real_Array (1 .. Size) := [others => 0.0];
         procedure Read is
         begin
            if Natural (G) <= Group_Count (M) and then Group_Size (M, G) = Size then
               for C in 1 .. Size loop
                  Result (C) := Driver.Robot.Motion.Hold_Of (M, G, C);
               end loop;
            end if;
         end Read;
      begin
         Driver.Beats.Within_A_Beat (Read'Access);
         return Result;
      end Holds_Of;

      --  Land: a positioning (Motion.Step), landed on its target; the sweep's moves, whose keyframes must turn one
      --  joint alone from a hold the arm comes back to exactly. Without it, a push whose answer is measured.
      procedure Go_To
        (G : Group_Id; Target : Real_Array; Report : out Driver.Robot.Motion.Step_Report; Land : Boolean := False) is
         C : Driver.Commands.Command;
      begin
         Driver.Commands.Set_Target (C, G, Target);
         Driver.Robot.Motion.Step (M, C, Report, Land => Land);
      end Go_To;

      --  What Recognize found of each channel: the amount an eye sees it move
      --  at (0 when none does) and the ways found at their ends.
      type Recognized_Channel is record
         Ref    : Driver.Robot.Motion.Channel_Ref;
         Amount : Real := 0.0;
         Ended  : Driver.Robot.Motion.Sense_Flags := [others => False];
      end record;
      package Recognized_Vectors is new Ada.Containers.Vectors (Positive, Recognized_Channel);
      Recognized : Recognized_Vectors.Vector;

      --  Pushes every channel of the group both ways by Factor times the
      --  amount an eye sees it move at, never into a way at its end, in an
      --  order no other group shares, for the lock-in to tell the groups
      --  apart; then settles.
      --  Only: the channels to push (every channel when none is marked). The pushes are chained: each
      --  channel is pushed from where the last push left the group, settled before and after as before, and the
      --  group comes back to its hold once at the end, not after every channel (the way back cost as much as
      --  the way out: a round of a six-joint group took a thousand beats, four rounds ten thousand, item 17).
      procedure Push_Both_Ways
        (G : Group_Id; Size : Positive; Factor : Real; Moved : out Boolean; Only : Channel_Flags := [1 .. 0 => False];
         Pause : Natural := 0)
      is
         Start  : constant Real_Array := Holds_Of (G, Size);
         Here   : constant Real_Array := Start;   --  every push goes out from the hold and back to it
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         Report : Driver.Robot.Motion.Step_Report;
         Order  : Push_Array (1 .. 2 * Size);
         Count  : Natural := 0;
         Any_Only : constant Boolean := (for some F of Only => F);
         --  The round's cost, for the log: the beats it took, and those spent settling between the pushes.
         Began_At, Ended_At : Natural := 0;
         Settled  : Natural := 0;
         procedure Read_Beat is
         begin
            Began_At := M.Beats;
         end Read_Beat;
         procedure Read_End is
         begin
            Ended_At := M.Beats;
         end Read_End;
      begin
         Moved := False;
         Driver.Beats.Within_A_Beat (Read_Beat'Access);
         for R of Recognized loop
            if R.Ref.Group = G and then R.Ref.Channel <= Size and then R.Amount > 0.0
              and then (not Any_Only or else (R.Ref.Channel <= Only'Last and then Only (R.Ref.Channel)))
            then
               Amount (R.Ref.Channel) := Factor * R.Amount;
               for S in Driver.Robot.Motion.Sense loop
                  if not R.Ended (S) then
                     Count := Count + 1;
                     Order (Count) := (Channel => R.Ref.Channel,
                                       Sign    => (if S = Driver.Robot.Motion.Increasing then 1.0 else -1.0));
                  end if;
               end loop;
            end if;
         end loop;
         Shuffle (Order (1 .. Count), Rng);
         for P of Order (1 .. Count) loop
            declare
               Away : Real_Array := Here;
            begin
               Away (P.Channel) := Here (P.Channel) + P.Sign * Amount (P.Channel);
               --  Each push from a body at rest, its pictures settled: what the eyes see move is the push's own, and
               --  not the tail of the push before it, which the lock-in would credit to this one (Lockin.Measure
               --  leaves out a push that began from a picture still changing). The pose it is pushed from is
               --  wherever the pushes before left the group: the lock-in regresses on the readings' changes, and
               --  the amounts are the smallest an eye sees, so the group stays about its hold.
               Driver.Robot.Motion.Settle (M, Waited);
               Go_To (G, Away, Report);
               Moved := Moved or else Report.Outcome = Driver.Robot.Motion.Reached;
               --  Held after the push for Pause beats before the pictures are asked to settle: a picture that trails
               --  the readings (an eye of the rig ten beats behind them) is settled the beat after they stop, before it
               --  has moved, and pushes a few beats apart make one burst that no lag longer than their spacing can be
               --  told across (Lag: the shifts tried reach the stretch between bursts); the pause is the longest lag
               --  the stream could tell when the round began, until every eye's lag is measured (robot.boot.reread).
               if Pause > 0 then
                  Driver.Robot.Motion.Hold (M, Pause);
                  Settled := Settled + Pause;
               end if;
               Driver.Robot.Motion.Settle (M, Waited);
               Settled := Settled + Waited;
               --  And back to where it was pushed from, a second push the same size the other way: a round of pushes
               --  that only go on from where the last one left (item 17's chain) gives the lock-in half the evidence a
               --  round of pushes out and back does, and an eye whose middle distance showed itself at twice the
               --  amounts with the old round showed nothing with the chain, so the patch did not grow and the boot
               --  left the arm a part (robot.boot.patch, robot.boot.step: 13 of 48 cells at twice the amounts against
               --  17 before). The chain's other saving stands: only the channels an undecided eye does not see.
               Go_To (G, Here, Report);
               if Pause > 0 then
                  Driver.Robot.Motion.Hold (M, Pause);
                  Settled := Settled + Pause;
               end if;
               Driver.Robot.Motion.Settle (M, Waited);
               Settled := Settled + Waited;
            end;
         end loop;
         Go_To (G, Start, Report);
         if Pause > 0 then
            Driver.Robot.Motion.Hold (M, Pause);
            Settled := Settled + Pause;
         end if;
         Driver.Robot.Motion.Settle (M, Waited);
         Settled := Settled + Waited;
         Driver.Beats.Within_A_Beat (Read_End'Access);
         Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " pushed" & Count'Image & " ways at"
                          & Integer'Image (Integer (Factor)) & " times its amounts in" & Integer'Image (Ended_At - Began_At)
                          & " beats, of which" & Settled'Image & " settling between the pushes");
      end Push_Both_Ways;

      --  Finds how far each channel of the group must move for an eye to see
      --  it, both ways from the amount at which the whole body was first seen
      --  (Motion.Probe_Both_Ways: a way at its end stops where the other one
      --  answered; a channel is pushed on until an eye sees it or its own
      --  reading ends, whatever the other channels needed, and is dead only
      --  when its reading followed no level either way), then pushes every
      --  channel both ways by that much (Push_Both_Ways).
      procedure Recognize (G : Group_Id; Size : Positive; From : Real) is
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         function Way (S : Driver.Robot.Motion.Sense) return String is
           (if S = Driver.Robot.Motion.Increasing then "upwards" else "downwards");
      begin
         for C in 1 .. Size loop
            declare
               Ref : constant Driver.Robot.Motion.Channel_Ref := (Group => G, Channel => C);
               P   : Driver.Robot.Motion.Two_Way_Report;
            begin
               Driver.Robot.Motion.Probe_Both_Ways (M, Ref, From, P);
               if P.Seen then
                  Amount (C) := P.Excursion;
               end if;
               Recognized.Append (Recognized_Channel'(Ref => Ref, Amount => Amount (C), Ended => P.At_End));
               for S in Driver.Robot.Motion.Sense loop
                  if P.At_End (S) then
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image & " is at its end "
                        & Way (S) & ": it delivered nothing that way up to "
                        & Scientific (From * 2.0 ** (P.Levels (S) - 1))
                        & " reading units, while it answered the other way");
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image
                                & (if Amount (C) > 0.0
                                   then " is seen when moved by " & Scientific (Amount (C)) & " reading units (after"
                                        & P.Levels (Driver.Robot.Motion.Increasing)'Image & " levels up,"
                                        & P.Levels (Driver.Robot.Motion.Decreasing)'Image & " down)"
                                   elsif P.Dead
                                   then " answers neither way up to "
                                        & Scientific (From * 2.0 ** (P.Levels (Driver.Robot.Motion.Increasing) - 1))
                                        & " reading units, at every level it was asked either way: dead or"
                                        & " disconnected for this boot; it is left alone"
                                   elsif P.Blind
                                   then " moves nothing any eye sees, and its reading's noise is not measured, so how"
                                        & " its reading followed is not known"
                                   else " moves nothing any eye sees, up to where it stops following"));
            end;
         end loop;
         declare
            Moved : Boolean;
         begin
            Push_Both_Ways (G, Size, 1.0, Moved,
                            Pause => Hold_After_A_Push (M, G, Natural'Max (1, Driver.Robot.Lag.Longest (M))));
         end;
      end Recognize;

      --  Turns every joint of the arm both ways from where it rests, by steps
      --  that double from the smallest one a keyframe counts (Motion.Sweep_Start:
      --  the least turn by the step the fit judges a joint's motion by, or the
      --  largest turn the aligner follows by itself once how far the view moves
      --  per reading unit is measured, by the keyframes so far,
      --  Kinematics.Pixels_Per_Unit), until a step is blocked or short,
      --  or the eye would have turned by half its view: beyond that a view
      --  shares less than half of itself with the one it started from. Every
      --  step is held until the eye gives the keyframe (Motion.Hold_For_Keyframe).
      procedure Sweep (A : Arm_Id) is
         G    : Group_Id := 1;
         Size : Natural := 0;
         procedure Read_Size is
         begin
            G := Arm_Group (M, A);
            Size := Group_Size (M, G);
         end Read_Size;
         --  The reference of the arm's kinematics is the pose it is swept from (Kinematics.Begin_Anew): the
         --  recognition rounds left it where they found it.
         procedure Begin_Anew is
         begin
            Driver.Robot.Kinematics.Begin_Anew (M, A);
         end Begin_Anew;
      begin
         Driver.Beats.Within_A_Beat (Read_Size'Access);
         --  The arm is brought to its hold before the reference is taken. A servo holding a load stands off the
         --  hold's target by its steady error, and every push and every return lands on its target (Motion.Step
         --  sends a step again by its landing's error): a reference taken where the arm merely stood would lie off
         --  every keyframe of the sweep by that error, in every channel, and no keyframe would turn one joint alone
         --  (R5 on the Pi, 10-13: 2 to 8 units on four of arm_a's channels in every keyframe of the recognition).
         declare
            Report : Driver.Robot.Motion.Step_Report;
         begin
            Go_To (G, Holds_Of (G, Size), Report, Land => True);
         end;
         Driver.Beats.Within_A_Beat (Begin_Anew'Access);
         Driver.Robot.Motion.Hold_For_Twin (M, A);
         declare
            First, Per_Unit : Real_Array (1 .. Size) := [others => 0.0];
            --  The pose the sweep goes out from and comes back to: where the arm stands after the step to its hold,
            --  read, not the hold's targets. A channel the hold's target lies beyond the end of (arm_a's fourth on
            --  the kiwi, R5: its hold 15 units past where it rests on what stops it) would make every return a
            --  blocked step, and every push of the sweep stopped by it.
            Start  : Real_Array (1 .. Size) := [others => 0.0];
            procedure Read_Start is
               Last : constant Natural := (if M.Beats > 0 then M.Beats - 1 else 0);
            begin
               for C in 1 .. Size loop
                  Start (C) := Channels.Reading (M, G, Last, C);
               end loop;
            end Read_Start;
            Least  : Real_Array (1 .. Size) := [others => 0.0];   --  per channel: the least turn a keyframe counts
            Moves  : array (1 .. Size) of Boolean := [others => False];   --  per channel: it moves the eye's view (Lockin.Seen)
            Half   : Real := 0.0;
            Patch  : Real := 0.0;   --  a patch of the eye's view, in px: the farthest the aligner follows by itself
            Eye    : Natural := 0;
            --  How far the view moves per reading unit of each channel, as the keyframes have measured it so far
            --  (Kinematics.Pixels_Per_Unit; 0 until the keyframes determine the channel): read again after every
            --  round, since the rounds' keyframes measure it.
            procedure Read_Shifts is
            begin
               for C in 1 .. Size loop
                  Per_Unit (C) := Driver.Robot.Kinematics.Pixels_Per_Unit (M, A, C);
               end loop;
            end Read_Shifts;
            --  The channels the sweep is turning now, told to the kinematics for the keyframes it takes meanwhile
            --  (Kinematics.Asking): a push's joint from the pose it is asked to until the arm is asked elsewhere, every
            --  swept joint through a cell of Together, none while the arm is brought back.
            Asked_Now : Channel_Flags (1 .. Size) := [others => False];
            procedure Tell_Asking is
            begin
               Driver.Robot.Kinematics.Asking (M, A, Asked_Now);
            end Tell_Asking;
            --  Per channel: the eye does not see its view move with it, as the keyframes measured it
            --  (Kinematics.Shift_Unseen: a joint behind the eye, which turns the hand in its picture); read again
            --  after every round.
            Unseen_Now : array (1 .. Size) of Boolean := [others => False];
            procedure Read_Unseen is
            begin
               for C in 1 .. Size loop
                  Unseen_Now (C) := Driver.Robot.Kinematics.Shift_Unseen (M, A, C);
               end loop;
            end Read_Unseen;
            procedure Read_Plan is
            begin
               for E in 1 .. Eye_Count (M) loop
                  if Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A then
                     Eye := E;
                  end if;
               end loop;
               if Eye > 0 then
                  Half := Real (Natural'Min (M.Eyes (Eye_Id (Eye)).Grid.Width, M.Eyes (Eye_Id (Eye)).Grid.Height))
                    / 2.0;
                  Patch := 2.0 * Real (Driver.Alignment.Half_Patch (M.Eyes (Eye_Id (Eye)).Grid.Width,
                                                                     M.Eyes (Eye_Id (Eye)).Grid.Height));
                  Read_Shifts;
                  for C in 1 .. Size loop
                     Moves (C) := Lockin.Seen (M, Eye_Id (Eye), G, C);
                     First (C) := Driver.Robot.Motion.Sweep_Start (M, A, C);
                     --  And no less than the turn at which the recognition's probe saw the eye's view move with the
                     --  channel, an executed turn the eye saw (Probe_Both_Ways): the first keyframe needs the view to
                     --  have moved, and before any keyframe the step a keyframe counts from may be none (the rig of
                     --  robot.boot.short: no visible step from the lock-in, a first level of 0, nothing swept).
                     --  A channel the probe saw move an eye moves the eye, whatever the latest lock-in estimate kept as
                     --  regressors: the estimate before R6's third sweep (10-14) kept 9 columns of 12, its rows after
                     --  the rounds' pushes mostly in settling tails of 274 beats, and arm_a's second, third and fourth
                     --  joints were left unswept though the probe had seen each at 9.8 to 157 units.
                     for R of Recognized loop
                        if R.Ref.Group = G and then R.Ref.Channel = C then
                           First (C) := Real'Max (First (C), R.Amount);
                           Moves (C) := Moves (C) or else R.Amount > 0.0;
                        end if;
                     end loop;
                     Least (C) := Driver.Robot.Motion.Least_Turn (M, A, C);
                  end loop;
               end if;
            end Read_Plan;
            Report : Driver.Robot.Motion.Step_Report;

            --  The keyframes of a round are aligned, and the arm fitted again from
            --  them, before the next round is moved (Estimate_Now): what the next
            --  round asks rests on what this one showed.
            procedure Measure_The_Round is
            begin
               Driver.Robot.Motion.Hold_And_Estimate (M);
            end Measure_The_Round;

            --  Whether the keyframe taken in the beats the arm was held for a push (Ask_From .. Ask_To) was followed
            --  (Kinematics.Followed) and whether the arm is fitted now, read in a held beat.
            Ask_From, Ask_To : Natural := 0;
            Followed_Now : Boolean := False;
            Framed_Now   : Boolean := False;   --  a keyframe was taken in the span at all
            Fitted_Now   : Boolean := False;
            procedure Read_Followed is
            begin
               Followed_Now := Driver.Robot.Kinematics.Followed (M, A, Ask_From, Ask_To);
               Framed_Now := Driver.Robot.Kinematics.Framed (M, A, Ask_From, Ask_To);
               Fitted_Now := Driver.Robot.Kinematics.Fitted (M, A);
            end Read_Followed;
            --  The beats the body has seen, read in a held beat.
            Beat_Now : Natural := 0;
            procedure Read_Beat_Now is
            begin
               Beat_Now := M.Beats;
            end Read_Beat_Now;
         begin
            Driver.Beats.Within_A_Beat (Read_Start'Access);
            Driver.Beats.Within_A_Beat (Read_Plan'Access);
            if Eye = 0 then
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " carries no eye; its joints are not swept");
               return;
            end if;
            declare
               --  Joints and directions in an order no other arm shares, for
               --  the estimates over the whole stream to tell them apart.
               Order : Push_Array (1 .. 2 * Size);
               Count : Natural := 0;
               --  Drawn in a held beat: the arms swept at once (Driver.Beats.At_Once)
               --  share the boot's one generator, and their windows never overlap.
               procedure Draw_Order is
               begin
                  Shuffle (Order (1 .. Count), Rng);
               end Draw_Order;
               --  Per push: the offset of its next level, whether it climbs on, and why it stopped.
               type Stop is (Climbing, Not_Followed, Blocked, Widest, Unseen);
               Offset  : Real_Array (1 .. 2 * Size) := [others => 0.0];
               State   : array (1 .. 2 * Size) of Stop := [others => Blocked];
               --  Per push: its ladder is going down from a first level the aligner could not follow.
               Descending : array (1 .. 2 * Size) of Boolean := [others => False];
               --  Per push: the levels taken in the pass, in order: each one's offset and the beats the arm was held
               --  for its keyframe.
               type Level_Mark is record
                  Offset   : Real := 0.0;
                  From, To : Natural := 0;
                  Short    : Boolean := False;   --  the step to it was not reached: the joint met something
               end record;
               package Level_Vectors is new Ada.Containers.Vectors (Positive, Level_Mark);
               Levels : array (1 .. 2 * Size) of Level_Vectors.Vector;
               --  Per channel: the largest offset a followed keyframe of it was taken at.
               Reached : Real_Array (1 .. Size) := [others => 0.0];
               Swept   : array (1 .. Size) of Positive;
               N       : Natural := 0;
               Rounds, Passes : Natural := 0;

               --  The push's keyframes for the pass: the joint moved by the push's offset from the held pose and
               --  held still there for the kinematics, then on to twice the offset, and on, until the turn would
               --  move the view by more than half of it (Half) or the joint meets something. The next push's first
               --  level is asked from the held pose in every other channel, so the joint comes back with it: a return
               --  to the held pose after every push lands within the servo's dead band of it, and held there for the
               --  next push the arm gave a keyframe two or three units off the reference in one channel, carrying
               --  nothing the reference's still twin does not and costing an alignment each (R6's sixth start, 10-14:
               --  17 of arm_a's 35 keyframes in its first pass, 3.5 s each on the Pi, and the fit's single-joint
               --  frames confused by them). The levels used to be taken one a round, every round measured before the next
               --  (the aligner follows a level from the one before it): on the Pi an estimate costs a minute and
               --  more, and six rounds of them took longer than the whole boot should (R5, 10-14). The levels of
               --  one pass are aligned in order within one estimate, each predicted from the one before as before;
               --  what the aligner could not follow is told afterwards, and the pass's motion beyond it is a few
               --  seconds lost against the minutes of estimates. While how far the view moves per unit of the
               --  joint is not measured, one level is taken: it is the measurement, and the widest turn is not told.
               --  Blocked short of a level, the push stops there (an arm that chatters against what stops it gives
               --  no keyframe there).
               procedure Take (I : Positive) is
                  P    : Push renames Order (I);
                  Pose : Real_Array := Start;
                  Off  : Real := Offset (I);
                  Mark : Level_Mark;
               begin
                  Levels (I).Clear;
                  Asked_Now := [others => False];
                  Asked_Now (P.Channel) := True;
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
                  loop
                     Pose (P.Channel) := Start (P.Channel) + P.Sign * Off;
                     Driver.Beats.Within_A_Beat (Read_Beat_Now'Access);
                     Mark := (Offset => Off, From => Beat_Now, To => 0, Short => False);
                     Go_To (G, Pose, Report, Land => True);
                     if Report.At_Rest then
                        Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                     end if;
                     Driver.Beats.Within_A_Beat (Read_Beat_Now'Access);
                     Mark.To := (if Beat_Now > 0 then Beat_Now - 1 else 0);
                     Mark.Short := Report.Outcome /= Driver.Robot.Motion.Reached;
                     Levels (I).Append (Mark);
                     exit when Mark.Short or else Per_Unit (P.Channel) = 0.0;
                     Off := 2.0 * Off;
                     --  Before the arm is fitted, the ladder climbs only as far as the aligner follows by itself: an
                     --  extrapolation from the levels before to about a patch of displacement beyond them (A22, A23).
                     --  A doubled level's increment is the level before it, so a level whose whole motion is two
                     --  patches is one patch beyond the followed level before it; the levels beyond are followed only
                     --  by a fitted arm's prediction (Motion.Sweep_Start). Taken blind in one pass, they are keyframes
                     --  nothing finds (R6's second start, 10-14: arm_a's levels at 214, 463, 967 and 2792 units of its
                     --  elbow, 0 of 441 points each, from a view's motion per unit measured ten times too small, and
                     --  the elbow swung 245 degrees before its ladder met the servo's range). The brake stood at one
                     --  patch of whole motion, half the increment the aligner follows: the kiwi's ladders took one
                     --  doubling a pass, and the parallax that tells a wrist joint's lever (a turn of ~150 units)
                     --  lay five passes and ten minutes away (R6's tenth start, 10-14).
                     exit when Off * Per_Unit (P.Channel) > (if Fitted_Now then Half else 2.0 * Patch);
                  end loop;
                  Asked_Now := [others => False];
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
               end Take;

               --  Every swept joint at once, in cells whose signs no two share
               --  (the rows of a Sylvester-Hadamard matrix after its first, its
               --  columns after its first): a joint's own frames tell its axis
               --  but not how far that axis is from the others', which only
               --  frames that move several joints show. Each joint moves by its
               --  share: its part of the offset its own keyframes were followed
               --  to, so that the cell moves the view no farther than the aligner
               --  followed one joint, or the fitted arm predicts; and no less
               --  than the keyframe step, the smallest turn the fit counts as a
               --  joint having moved (a sixth of three steps is none, and a cell
               --  of such parts moves no two joints the fit can see).
               procedure Together (Share : Real_Array) is
                  Rows : Positive := 1;
               begin
                  while Rows < N + 1 loop
                     Rows := 2 * Rows;
                  end loop;
                  Asked_Now := [others => False];
                  for K in 1 .. N loop
                     Asked_Now (Swept (K)) := True;
                  end loop;
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
                  for Row in 1 .. Rows - 1 loop
                     declare
                        Scale : Real := 1.0;   --  of the shares: halved where the cell met something
                     begin
                        loop
                           declare
                              Pose : Real_Array := Start;
                           begin
                              for K in 1 .. N loop
                                 declare
                                    Bits : Natural := 0;
                                    R    : Natural := Row;
                                    Col  : Natural := K;
                                 begin
                                    --  H (Row, K) = (-1) ** (the bits Row and K share)
                                    while R > 0 and then Col > 0 loop
                                       if R mod 2 = 1 and then Col mod 2 = 1 then
                                          Bits := Bits + 1;
                                       end if;
                                       R := R / 2;
                                       Col := Col / 2;
                                    end loop;
                                    Pose (Swept (K)) := Start (Swept (K))
                                      + (if Bits mod 2 = 0 then 1.0 else -1.0) * Scale * Share (Swept (K));
                                 end;
                              end loop;
                              Go_To (G, Pose, Report, Land => True);
                              if Report.At_Rest then
                                 Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                              end if;
                           end;
                           --  A cell that met something (a keyframe where it stopped, if it
                           --  rested there) is tried again at half its shares, as long as some
                           --  share is still a turn the fit counts: a cell need not move every
                           --  joint, and one that always meets something gives no frame at all
                           --  (A99's arm 1 had no keyframe of several joints for that).
                           exit when Report.Outcome = Driver.Robot.Motion.Reached;
                           Scale := Scale / 2.0;
                           exit when (for all K in 1 .. N => Scale * Share (Swept (K)) < First (Swept (K)));
                        end loop;
                     end;
                  end loop;
                  Asked_Now := [others => False];
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
                  Go_To (G, Start, Report, Land => True);
               end Together;

               --  Each swept joint's share of a frame that moves them together: its part of the offset it was
               --  followed to, and no less than its first level (Together).
               function Shares return Real_Array is
                  Share  : Real_Array (1 .. Size) := [others => 0.0];
                  Motion : Real := 0.0;   --  the view's motion the cell asks, px, by the measured motion per unit
                  Fixed  : Real := 0.0;   --  of it, the part the first levels hold
               begin
                  for K in 1 .. N loop
                     Share (Swept (K)) := Real'Max (First (Swept (K)), Reached (Swept (K)) / Real (N));
                     Motion := Motion + Share (Swept (K)) * Per_Unit (Swept (K));
                     Fixed := Fixed + First (Swept (K)) * Per_Unit (Swept (K));
                  end loop;
                  --  And no more motion of the view in all than the ladder's own brake before a fit, two patches
                  --  beyond the keyframes before (Take): the shares above their first levels are scaled down to it,
                  --  the first levels themselves stand (below them a joint does not move).
                  if not Fitted_Now and then Motion > 2.0 * Patch and then Motion > Fixed then
                     declare
                        Scale : constant Real := Real'Max (0.0, (2.0 * Patch - Fixed) / (Motion - Fixed));
                     begin
                        for K in 1 .. N loop
                           Share (Swept (K)) :=
                             First (Swept (K)) + Scale * (Share (Swept (K)) - First (Swept (K)));
                        end loop;
                     end;
                  end if;
                  return Share;
               end Shares;

               function Image (X : Real_Array) return String is
                  Text : Ada.Strings.Unbounded.Unbounded_String;
               begin
                  for V of X loop
                     Ada.Strings.Unbounded.Append (Text, " " & Driver.Log.Image (V, 3));
                  end loop;
                  return Ada.Strings.Unbounded.To_String (Text);
               end Image;
               --  The plan, for the log: per channel its first level, the least turn and the view's motion per unit.
               function Plan_Text return String is
                  Text : Ada.Strings.Unbounded.Unbounded_String;
               begin
                  for C in 1 .. Size loop
                     Ada.Strings.Unbounded.Append
                       (Text, " c" & C'Image & " " & Driver.Log.Image (First (C), 3) & " (least "
                        & Driver.Log.Image (Least (C), 3) & ", "
                        & (if Per_Unit (C) > 0.0 then Driver.Log.Image (Per_Unit (C), 4) & " px a unit" else "px a unit unmeasured")
                        & (if Moves (C) then ")" else ", does not move the eye)"));
                  end loop;
                  return Ada.Strings.Unbounded.To_String (Text);
               end Plan_Text;
            begin
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " sweep plan, first levels in reading units:"
                                & Plan_Text & "; the widest turn moves the view " & Driver.Log.Image (Half, 1) & " px");
               for C in 1 .. Size loop
                  if Moves (C) and then First (C) > 0.0 then
                     Order (Count + 1) := (Channel => C, Sign => 1.0);
                     Order (Count + 2) := (Channel => C, Sign => -1.0);
                     Offset (Count + 1) := First (C);
                     Offset (Count + 2) := First (C);
                     State (Count + 1) := Climbing;
                     State (Count + 2) := Climbing;
                     Count := Count + 2;
                     N := N + 1;
                     Swept (N) := C;
                  else
                     Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " channel" & C'Image
                                      & " does not move its eye; not swept");
                  end if;
               end loop;
               Driver.Beats.Within_A_Beat (Draw_Order'Access);
               --  Keyframes over every scale from the matcher's limit to the widest
               --  turn: each push climbs a ladder of offsets that double from the
               --  keyframe step, one level a round, every push alive in the round
               --  before the next, and the round's keyframes are measured before
               --  the next round. A push climbs as far as its keyframes are followed
               --  (Kinematics.Followed: the aligner found its points from windows no
               --  wider than a patch, from what the keyframes before it showed, or
               --  from the fitted arm's own prediction), as far as the joint goes,
               --  and no farther than a turn that moves the view by half of it. A
               --  level not followed is not climbed past: the aligner follows an
               --  extrapolation to about a patch of displacement (A22, A23), and a
               --  fake body's eye over a grain with no coarser structure followed
               --  one level of four (T6, arm 1: 190 of 229 points at the first,
               --  none at the next, the sweep's 42 keyframes aligned 1689 of 9618).
               --  When every push has stopped, the joints move together at the
               --  offsets they were followed to (the frames that tell their order),
               --  and the arm is fitted from all of it. A fitted arm predicts the
               --  keyframes the aligner alone could not follow: the pushes that
               --  stopped unfollowed climb again from where they stopped, in
               --  another pass, until a pass follows nothing new.
               loop
                  Passes := Passes + 1;
                  declare
                     Followed_In_Pass : Natural := 0;
                  begin
                     if (for some I in 1 .. Count => State (I) = Climbing) then
                        Rounds := Rounds + 1;
                        for I in 1 .. Count loop
                           if State (I) = Climbing then
                              Take (I);
                           end if;
                        end loop;
                        --  Until the arm is fitted, every pass ends with the joints
                        --  moving together at the offsets taken so far: the fit
                        --  needs frames that move several joints to tell their
                        --  order, and a fitted arm predicts what the aligner alone
                        --  cannot follow, so the sooner it exists the fewer pushes
                        --  stop unfollowed (Y10, arm 1: thirteen rounds of ladders
                        --  passed stages 0 to 3 at every round and waited at stage 4
                        --  for the last).
                        --  And only once every swept joint's motion of the view per unit is measured (the pass's own
                        --  levels, aligned, measure it): a cell's shares are sized by it (Shares), and a cell taken
                        --  blind at every joint's first level moved the kiwi's view by the sum of them, 50 to 90 px,
                        --  beyond what the aligner searches from no prediction, and gave 0 of 361 points six times
                        --  of seven (R6's thirteenth start, 10-14); from the next pass on the keyframes before
                        --  predict a cell (the chain's sum of single-channel moves).
                        if N > 1 and then not Fitted_Now
                          and then (for all K in 1 .. N => Per_Unit (Swept (K)) > 0.0)
                        then
                           Together (Shares);
                        end if;
                        Measure_The_Round;
                        Driver.Beats.Within_A_Beat (Read_Shifts'Access);
                        Driver.Beats.Within_A_Beat (Read_Unseen'Access);
                        for I in 1 .. Count loop
                           if State (I) = Climbing then
                              declare
                                 P : Push renames Order (I);
                                 Lost  : Natural := 0;   --  the first level the aligner did not follow; 0 when none
                                 Last  : Level_Mark;
                              begin
                                 for K in 1 .. Natural (Levels (I).Length) loop
                                    Last := Levels (I) (K);
                                    Ask_From := Last.From;
                                    Ask_To := Last.To;
                                    Driver.Beats.Within_A_Beat (Read_Followed'Access);
                                    --  Followed as the aligner says, the first level too: a keyframe of a new direction is
                                    --  asked from the view's common motion and its answers on the keyframe's own field
                                    --  again narrowly (Keyframes.Add). The first level used to count as followed whenever
                                    --  the pass measured how far the view moves per unit of the joint, from any keyframe:
                                    --  f15i's fifth joint had its shift from the cells of the pass, its first level
                                    --  (310 units) found 0 of 242 points, and the ladder climbed on from it to 2508
                                    --  instead of halving to a level the aligner follows (10-15).
                                    if Followed_Now then
                                       Followed_In_Pass := Followed_In_Pass + 1;
                                       Reached (P.Channel) := Real'Max (Reached (P.Channel), Last.Offset);
                                       Descending (I) := False;
                                    elsif Framed_Now then
                                       --  A keyframe the aligner could not follow, too large: the ladder stops at
                                       --  this level; a fitted arm may predict it in another pass.
                                       Lost := K;
                                       exit;
                                    end if;
                                    --  No keyframe came of a level too small to count (the held pose was not apart
                                    --  from the keyframes before it by the step the fit counts): the ladder climbs
                                    --  past it.
                                 end loop;
                                 if Unseen_Now (P.Channel) then
                                    --  The view does not move with the joint: a joint behind the eye (the kiwi's
                                    --  wrist roll, whose eye is mounted before it, R5 on 10-14: unmeasured, its
                                    --  ladder doubled a level a pass to 3016 units, every level followed by the
                                    --  jaws' points), and no level of it moves the view, however far.
                                    State (I) := Unseen;
                                 elsif Lost > 0 then
                                    if Reached (P.Channel) = 0.0 and then Offset (I) / 2.0 >= Least (P.Channel) then
                                       --  Not followed before any level of the joint was: the first level is sized
                                       --  by how far the view moves per unit of the joint, and that is wrong where
                                       --  the eye the arm is thought to carry is another's (an eye credited to the
                                       --  arm by a lag it could not tell yet: the rig of robot.boot.reread, whose
                                       --  first level stopped the sweep in its first round and left arm 2 unswept).
                                       --  The ladder goes down, halving, until a level is followed, and climbs from
                                       --  there, or until the least turn a keyframe counts.
                                       Offset (I) := Offset (I) / 2.0;
                                       Descending (I) := True;
                                    else
                                       Offset (I) := Levels (I) (Lost).Offset;
                                       State (I) := Not_Followed;
                                    end if;
                                 elsif Levels (I).Is_Empty then
                                    State (I) := Blocked;
                                 elsif Last.Short then
                                    State (I) := Blocked;
                                 elsif Per_Unit (P.Channel) > 0.0 and then 2.0 * Last.Offset * Per_Unit (P.Channel) > Half then
                                    State (I) := Widest;
                                 elsif Descending (I) then
                                    --  A ladder going down that reaches a level too small to count has no level the
                                    --  aligner follows.
                                    State (I) := Not_Followed;
                                 else
                                    --  Every level followed and the pass stopped short of the widest (the view's
                                    --  motion per unit was not measured when the pass began, or every level was
                                    --  too small to count): the next pass climbs on from twice the last.
                                    Offset (I) := 2.0 * Last.Offset;
                                 end if;
                              end;
                           end if;
                        end loop;
                     end if;
                     declare
                        Stops : Ada.Strings.Unbounded.Unbounded_String;
                     begin
                        for I in 1 .. Count loop
                           Ada.Strings.Unbounded.Append
                             (Stops, " " & Ada.Characters.Handling.To_Lower (State (I)'Image));
                        end loop;
                        Driver.Log.Line
                          (Driver.Log.Robot, "boot: arm" & A'Image & " pass" & Passes'Image & " followed"
                           & Followed_In_Pass'Image & " keyframes in" & Rounds'Image & " rounds so far; the joints"
                           & " are followed to offsets" & Image (Reached) & ", the pushes stopped by"
                           & Ada.Strings.Unbounded.To_String (Stops));
                     end;
                     --  A fitted arm ends the pass with the joints together at the
                     --  widest offsets followed (the rounds before the fit had their
                     --  own); its predictions go into these frames.
                     if N > 1 and then Followed_In_Pass > 0 and then Fitted_Now then
                        Together (Shares);
                        Measure_The_Round;
                     end if;
                     Driver.Beats.Within_A_Beat (Read_Followed'Access);
                     exit when (for all I in 1 .. Count => State (I) /= Climbing)
                       and then (Followed_In_Pass = 0 or else not Fitted_Now
                                 or else (for all I in 1 .. Count => State (I) /= Not_Followed));
                     --  The fitted arm predicts: the pushes that stopped unfollowed climb again.
                     if (for all I in 1 .. Count => State (I) /= Climbing) then
                        for I in 1 .. Count loop
                           if State (I) = Not_Followed then
                              State (I) := Climbing;
                           end if;
                        end loop;
                     end if;
                  end;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " swept in" & Passes'Image & " passes of"
                                & Rounds'Image & " rounds, " & (if Fitted_Now then "fitted" else "not fitted"));
            end;
            declare
               Waited_Here : Natural;   --  each arm's own: arms are swept at once
            begin
               Driver.Robot.Motion.Settle (M, Waited_Here);
            end;
         end;
      end Sweep;

      Count  : Natural := 0;
      Breach : Boolean := False;

      procedure Read_Count is
      begin
         Count := Group_Count (M);
      end Read_Count;

      --  Whether every commandable channel's rest noise is measured: a
      --  reading still drifting at rest measures none (Channels.Measure_Channel
      --  drops the trend), and the boot holds still on until it does.
      Rest_Measured : Boolean := False;
      Drifting      : Ada.Strings.Unbounded.Unbounded_String;
      --  The rest is measured when a doubling of its beats repeats every
      --  channel's noise within its own uncertainty: the tail of a crawl
      --  left in the trend-free rest passes for jitter until more rest beats
      --  expose it (robot.boot.creep), as a measurement is accepted elsewhere
      --  when the next doubling of beats says the same.
      type Noise_Access is access Real_Array;
      Rest_Noises : Noise_Access;
      Rest_Beats  : Natural := 0;     --  the beats the body had seen at the last comparison
      Rest_Stable : Boolean := False;
      procedure Read_Rest is
         Index : Natural := 0;
         Total : Natural := 0;
      begin
         Rest_Measured := True;
         Drifting := Ada.Strings.Unbounded.Null_Unbounded_String;
         for G in 1 .. Group_Count (M) loop
            Total := Total + Group_Size (M, Group_Id (G));
            if Is_Commandable (M, Group_Id (G)) then
               for C in 1 .. Group_Size (M, Group_Id (G)) loop
                  if Reading_Noise (M, Group_Id (G), C) = Real'Last then
                     Rest_Measured := False;
                     Ada.Strings.Unbounded.Append (Drifting, " group" & G'Image & " channel" & C'Image);
                  end if;
               end loop;
            end if;
         end loop;
         if not Rest_Measured then
            Rest_Stable := False;
            return;
         end if;
         if Rest_Noises = null then
            Rest_Noises := new Real_Array'(1 .. Total => Real'Last);
         end if;
         if M.Beats >= 2 * Rest_Beats then
            Rest_Stable := Rest_Beats > 0;
            for G in 1 .. Group_Count (M) loop
               for C in 1 .. Group_Size (M, Group_Id (G)) loop
                  Index := Index + 1;
                  if Is_Commandable (M, Group_Id (G)) then
                     declare
                        Now_N  : constant Real := Reading_Noise (M, Group_Id (G), C);
                        Then_N : constant Real := Rest_Noises (Index);
                        Dof    : constant Natural :=
                          (if Index <= Natural (M.Noise_Freedom.Length) then M.Noise_Freedom (Index - 1) else 0);
                     begin
                        if Then_N = Real'Last
                          or else (if Dof = 0 then Now_N /= Then_N
                                   else Driver.Uncertain.Significant
                                          (Now_N - Then_N,
                                           Now_N / Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0 * Real (Dof)),
                                           Dof))
                        then
                           Rest_Stable := False;
                        end if;
                        Rest_Noises (Index) := Now_N;
                     end;
                  end if;
               end loop;
            end loop;
            Rest_Beats := M.Beats;
         end if;
      end Read_Rest;

      procedure Read_Body is
      begin
         Driver.Log.Line (Driver.Log.Robot, "boot: the groups as recognized:" & ASCII.LF & Describe (M));
         for G in 1 .. Group_Count (M) loop
            if Is_Commandable (M, Group_Id (G)) and then Contract_Breach (M, Group_Id (G)) > 0 then
               Breach := True;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " breaks clause"
                                & Contract_Breach (M, Group_Id (G))'Image & " of the porting contract");
            end if;
         end loop;
      end Read_Body;

      --  Every arm of the current estimate is swept with the eye it carries,
      --  every such arm at once (Driver.Beats.At_Once, an arm a lane: their
      --  sweeps move their own groups, and each step waits for the whole body
      --  to be still): the body is read again before each round of sweeps and
      --  after the fit's estimate, since later evidence can change a role or a
      --  mount (an eye can decide by a later estimate that it rides on a
      --  group). An arm is known by its group, since a new arm can renumber the
      --  others, and is swept again when it carries another eye than the one it
      --  was swept with; an arm that carries none has nothing to sweep. The fit
      --  is made when no arm is left.
      procedure Sweep_Every_Arm is
         type Arm_Eye is record
            Group : Group_Id := 1;
            Eye   : Eye_Id := 1;
            Arm   : Arm_Id := 1;
         end record;
         package Pair_Lists is new Ada.Containers.Vectors (Positive, Arm_Eye);
         Swept   : Pair_Lists.Vector;
         Unswept : Pair_Lists.Vector;

         use type Ada.Containers.Count_Type;
         use type Driver.Observations.Camera_Id;

         function Same_Pair (A, B : Arm_Eye) return Boolean is (A.Group = B.Group and then A.Eye = B.Eye);

         --  Some unswept arm's eye sees another unswept arm move (the lock-in's response of that eye to the other's
         --  group is anything but nothing): their sweeps go one at a time, since a keyframe needs the eye's picture
         --  still, and the other arm moving through it keeps it from ever being (R6's eighth start, 10-14: both arms
         --  swept at once, each in the other's view, pass 2 to 4 of each gave no keyframe at all). In a white room
         --  the other arm is most of what an eye has to go by, and it must stand still to be that.
         See_Each_Other : Boolean := False;

         procedure Find_Unswept is
         begin
            Unswept.Clear;
            for A in 1 .. Arm_Count (M) loop
               declare
                  Found : Boolean := False;
                  Pair  : Arm_Eye;
               begin
                  for E in 1 .. Eye_Count (M) loop
                     --  The eye the sweep moves: the last one the arm carries.
                     if Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried
                       and then Eye_Mount (M, Eye_Id (E)).Arm = Arm_Id (A)
                     then
                        Found := True;
                        Pair := (Group => Arm_Group (M, Arm_Id (A)), Eye => Eye_Id (E), Arm => Arm_Id (A));
                     end if;
                  end loop;
                  if Found and then not (for some S of Swept => Same_Pair (S, Pair)) then
                     Unswept.Append (Pair);
                  end if;
               end;
            end loop;
            See_Each_Other := False;
            for P of Unswept loop
               for Q of Unswept loop
                  if P.Group /= Q.Group and then Response (M, Q.Group, P.Eye) not in Nothing | Unmeasured then
                     See_Each_Other := True;
                  end if;
               end loop;
            end loop;
         end Find_Unswept;

         procedure Sweep_Lane (Lane : Positive) is
         begin
            Sweep (Unswept (Lane).Arm);
         end Sweep_Lane;

         procedure Sweep_At_Once is new Driver.Beats.At_Once (Sweep_Lane);
      begin
         loop
            Driver.Beats.Within_A_Beat (Find_Unswept'Access);
            if not Unswept.Is_Empty then
               for P of Unswept loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: sweeping arm" & P.Arm'Image & " (group" & P.Group'Image
                                   & ") with eye" & P.Eye'Image
                                   & (if Unswept.Length = 1 then ""
                                      elsif See_Each_Other then ", the" & Unswept.Length'Image
                                           & " arms one at a time (an eye sees the other arm)"
                                      else ", the" & Unswept.Length'Image & " arms at once"));
               end loop;
               if Unswept.Length = 1 or else See_Each_Other then
                  for P of Unswept loop
                     Sweep (P.Arm);
                  end loop;
               else
                  Sweep_At_Once (Positive (Unswept.Length));
               end if;
               for P of Unswept loop
                  Swept.Append (P);
               end loop;
            else
               --  The instrument answers the keyframes' matches a beat or more
               --  after they were asked: the fit waits for every answer.
               Driver.Robot.Motion.Hold_While_Matching (M);
               Driver.Beats.Within_A_Beat (Estimate'Access);
               Driver.Beats.Within_A_Beat (Find_Unswept'Access);
               exit when Unswept.Is_Empty;
               for P of Unswept loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: the estimate after the sweeps lists arm" & P.Arm'Image
                                   & " (group" & P.Group'Image & ") with eye" & P.Eye'Image & ", not swept yet");
               end loop;
            end if;
         end loop;
      end Sweep_Every_Arm;
   begin
      Ok := False;
      Driver.Log.Line (Driver.Log.Robot, "boot: holding still to measure the body at rest");
      Driver.Robot.Motion.Settle (M, Waited);
      --  A body file from an earlier boot: what it holds stands, what it does
      --  not is measured below (Load_Body). Read once the robot has answered
      --  a held beat, so the key has every group that takes commands.
      if Body_File'Length > 0 then
         declare
            Loaded : Boolean := False;
            Why    : Ada.Strings.Unbounded.Unbounded_String;
            procedure Reload is
            begin
               Load_Body (M, Body_File, Loaded, Why);
            end Reload;
         begin
            Driver.Beats.Within_A_Beat (Reload'Access);
            Driver.Log.Line (Driver.Log.Robot, "boot: " & (if Loaded then "from " & Body_File & ", " else "")
                             & Ada.Strings.Unbounded.To_String (Why));
         end;
      end if;
      --  The body is at rest when every commandable channel's noise is
      --  measured: a reading that drifts (R0, 10-11: the sticky wrist servo
      --  crawling 660 ticks towards a stale goal after torque came on, taken
      --  for jitter, made the first push 175 ticks) measures none, and the
      --  boot holds still on.
      declare
         Last : Ada.Strings.Unbounded.Unbounded_String;
      begin
         loop
            Driver.Beats.Within_A_Beat (Estimate'Access);
            Driver.Beats.Within_A_Beat (Read_Rest'Access);
            exit when Rest_Measured and then Rest_Stable;
            if Ada.Strings.Unbounded."/=" (Drifting, Last) then
               Driver.Log.Line (Driver.Log.Robot, "boot: still moving at rest:"
                                & Ada.Strings.Unbounded.To_String (Drifting) & "; holding still on");
               Last := Drifting;
            end if;
            Driver.Robot.Motion.Settle (M, Waited);
         end loop;
      end;
      Driver.Beats.Within_A_Beat (Read_Count'Access);
      declare
         Recognized_Before, Fitted_Before : Boolean := False;
         procedure Read_Reloaded is
         begin
            Recognized_Before := Reloaded (M, Stored_Graph);
            Fitted_Before := Reloaded (M, Stored_Kinematics);
         end Read_Reloaded;
      begin
         Driver.Beats.Within_A_Beat (Read_Reloaded'Access);
         if not Recognized_Before then
            declare
               Commandable : array (1 .. Count) of Boolean := [others => False];
               Sizes       : array (1 .. Count) of Natural := [others => 0];
               procedure Read_Groups is
               begin
                  for G in 1 .. Count loop
                     Commandable (G) := Is_Commandable (M, Group_Id (G));
                     Sizes (G) := Group_Size (M, Group_Id (G));
                  end loop;
               end Read_Groups;
            begin
               Driver.Beats.Within_A_Beat (Read_Groups'Access);
               declare
                  Total : Natural := 0;
               begin
                  for G in 1 .. Count loop
                     if Commandable (G) then
                        Total := Total + Sizes (G);
                     end if;
                  end loop;
                  if Total = 0 then
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: no group takes a command; nothing can be moved to be measured");
                     return;
                  end if;
                  declare
                     Refs : Driver.Robot.Motion.Channel_Refs (1 .. Total);
                     K    : Natural := 0;
                     P    : Driver.Robot.Motion.Probe_Report;
                  begin
                     for G in 1 .. Count loop
                        if Commandable (G) then
                           for C in 1 .. Sizes (G) loop
                              K := K + 1;
                              Refs (K) := (Group => Group_Id (G), Channel => C);
                           end loop;
                        end if;
                     end loop;
                     --  Every commandable channel together first, by one amount: it
                     --  stops at the first move an eye sees, so no channel has moved
                     --  more than twice what an eye needs to see it, whatever its
                     --  units; each channel alone then starts from there.
                     --  One probe runs moves back and forth on the eyes' per-beat verdicts: the ladder of every
                     --  channel together. Each channel's own probe confirms a sighting by the lock-in's regression
                     --  after one estimate (Probe_Both_Ways), which asks no rest counts of the eyes.
                     Driver.Robot.Motion.Gather_Rest (M, 1);
                     Driver.Robot.Motion.Probe_Together (M, Refs, 1.0, 0.0, P);
                     if not P.Seen then
                        Driver.Beats.Within_A_Beat (Estimate'Access);
                        Driver.Beats.Within_A_Beat (Read_Body'Access);
                        Driver.Log.Line
                          (Driver.Log.Robot, "boot: nothing any eye sees moved while every commandable channel moved"
                           & " together, up to where each stopped following its command (porting contract, clause 2);"
                           & " the boot stops");
                        return;
                     end if;
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: every commandable channel moved together is first seen at "
                        & Scientific (P.Excursion) & " reading units, after" & P.Steps'Image & " doublings");
                     --  The body's delay and the eyes' lags are measured from the pushes just executed before
                     --  the channels are pushed one by one: a push nothing answers is waited for as long as the
                     --  longest answer took, and until that is measured as long as the stream is old (R6 on
                     --  10-14: 170 to 400 beats a push, the estimates on their doubling schedule not due for
                     --  thousands of beats).
                     Driver.Beats.Within_A_Beat (Estimate'Access);
                     for G in 1 .. Count loop
                        if Commandable (G) and then Sizes (G) > 0 then
                           Recognize (Group_Id (G), Sizes (G), P.Excursion);
                        end if;
                     end loop;
                     Driver.Beats.Within_A_Beat (Estimate'Access);
                     --  A push at the smallest amount some eye saw is little evidence,
                     --  and an eye that is undecided, or that shows a patch, has not
                     --  answered: an eye on a turning joint sees its view move by
                     --  different amounts in different cells (by depth, by perspective),
                     --  so at that amount only the cells that move most respond and the
                     --  whole picture reads as a patch; pushed harder more cells
                     --  respond, where a patch that is one does not grow. A group some
                     --  such eye is open about is pushed again, at twice the amounts of
                     --  its last round, until every eye has answered (the whole picture
                     --  moves, or nothing does), or a round leaves the cells each open
                     --  eye found responding no more than chance explains (not
                     --  significantly more, at Z and the per-cell false alarm, among
                     --  the cells that did not respond before) and the cells that do not
                     --  respond would show no motion together once the push is doubled
                     --  (Resting_Motion: a far part of the view that moves by less than any
                     --  cell can tell shows in no cell and, a little, in them all, and a
                     --  push of twice the size makes four times that), or a push moves
                     --  nothing further than the one before (the channels are at their
                     --  ends).
                     declare
                        Eyes : Natural := 0;
                        procedure Read_Eyes is
                        begin
                           Eyes := Eye_Count (M);
                        end Read_Eyes;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Eyes'Access);
                        declare
                           --  The cells each open eye found responding after the group's
                           --  last push; -1 for an eye that was not open then.
                           type Count_Grid is array (1 .. Count, 1 .. Eyes) of Integer;
                           Last   : Count_Grid := [others => [others => -1]];
                           Now    : Count_Grid;
                           Cells  : Count_Grid;   --  the cells it was reached over
                           Shows  : array (1 .. Count, 1 .. Eyes) of Eye_Response;
                           type Rest_Grid is array (1 .. Count, 1 .. Eyes) of Real;
                           Rests      : Rest_Grid;   --  the rest's motion together, in sigmas
                           Pulls      : Rest_Grid;   --  the rest's pull along the push together, in sigmas (Rest_Pull)
                           Factor : array (1 .. Count) of Real := [others => 1.0];
                           --  The hold after each push (Pause_After_A_Push): the longest measured lag once every eye's
                           --  is measured, until then the longest lag the stream could tell when the round began
                           --  (Push_Both_Ways). It used to double with the rounds: an eye that sees a group too faintly
                           --  for its lag to stand out (the kiwi's head eye against either arm, R5 on the Pi, 10-13:
                           --  undecided, its lag 0 but not known) kept the hold doubling, 320 beats a push by the round
                           --  at 32 times the amounts, 3360 of a round's 3734 beats; the stream's own reach grows with
                           --  the rounds as it is.
                           Pause  : array (1 .. Count) of Natural :=
                             [for G in 1 .. Count =>
                                Hold_After_A_Push (M, Group_Id (G), Natural'Max (1, Driver.Robot.Lag.Longest (M)))];
                           --  Per group: every eye that sees it move at all has its lag measured, and the longest of
                           --  those lags (Read_Open). An eye that sees nothing of the group never gets a lag from its
                           --  pushes and must not keep the hold at the doubled guess: B0a (10-13), the head eye saw
                           --  nothing of either arm, its lag stayed unmeasured, and every round held 16 to 26 beats a
                           --  push with the arms' own eyes measured at 1 beat (B0a_2: 533 of a 766-beat round).
                           Next_Pause : array (1 .. Count) of Natural := [others => 0];   --  Hold_After_A_Push, read with the round
                           Again  : array (1 .. Count) of Boolean;
                           Spent  : array (1 .. Count) of Boolean := [others => False];   --  pushed as far as it goes
                           function Most_Channels return Natural is
                              Most : Natural := 0;
                           begin
                              for G in 1 .. Count loop
                                 Most := Natural'Max (Most, Sizes (G));
                              end loop;
                              return Most;
                           end Most_Channels;
                           --  Per group and channel: an eye still undecided about the group, or shown a patch that
                           --  may grow, does not yet see the channel move it (Lockin.Seen). A round pushes those
                           --  channels again, and every channel when no eye tells one from another: the old round
                           --  pushed every channel of the group both ways, whatever was already seen (item 17).
                           Needed : array (1 .. Count, 1 .. Natural'Max (1, Most_Channels)) of Boolean :=
                             [others => [others => False]];

                           --  Per group: some eye sees its whole view move with the group (Whole): an arm, which
                           --  the sweep pushes next, every joint by levels that double (Sweep). The eyes still open
                           --  about such a group (a patch, or undecided) are decided by those pushes: the lock-in
                           --  regresses on every push of the stream, and the body is read again before each round of
                           --  sweeps and after each estimate (Sweep_Every_Arm), so an eye whose patch grows into
                           --  the whole view under the sweep is mounted on the arm then. The rounds used to push an
                           --  arm again at twice the amounts for its other eyes alone: the kiwi's head eye and each
                           --  arm's eye on the other arm (R6's sixth start, 10-14: 655 of the recognition's 2184
                           --  beats, 55 s, both arms' own eyes whole after the first round).
                           Ridden : array (1 .. Count) of Boolean := [others => False];
                           procedure Read_Open is
                           begin
                              Needed := [others => [others => False]];
                              for G in 1 .. Count loop
                                 Again (G) := False;
                                 Ridden (G) := Commandable (G)
                                   and then (for some E in 1 .. Eyes => Response (M, Group_Id (G), Eye_Id (E)) = Whole);
                                 Next_Pause (G) :=
                                   Hold_After_A_Push (M, Group_Id (G), Natural'Max (1, Driver.Robot.Lag.Longest (M)));
                                 for E in 1 .. Eyes loop
                                    Now (G, E) := -1;
                                    if Commandable (G)
                                      and then Response (M, Group_Id (G), Eye_Id (E)) in Undecided | Patch
                                    then
                                       Now (G, E) := Responding (M, Group_Id (G), Eye_Id (E));
                                       Cells (G, E) := Textured_Cells (M, Group_Id (G), Eye_Id (E));
                                       Shows (G, E) := Response (M, Group_Id (G), Eye_Id (E));
                                       Rests (G, E) := Resting_Motion (M, Group_Id (G), Eye_Id (E));
                                       Pulls (G, E) := Rest_Pull (M, Group_Id (G), Eye_Id (E));
                                       for C in 1 .. Sizes (G) loop
                                          if not Lockin.Seen (M, Eye_Id (E), Group_Id (G), C) then
                                             Needed (G, C) := True;
                                          end if;
                                       end loop;
                                       --  The group is pushed again at twice the amounts for an eye undecided about
                                       --  it or showing a patch only while the cells that respond to it grew with the
                                       --  last doubling (Grew): a whole showing itself grows with the push. It used to
                                       --  be pushed on as well while the motion the cells that did not respond show
                                       --  together grew in significance (Eye_Effect.Resting); that is a z-score of
                                       --  their pooled energy, which grows with the push's size and the beats whatever
                                       --  moves them, and on the kiwi (R5 on the Pi, 10-13) the head eye, undecided
                                       --  about either arm (a patch of half the view on a base every push shakes:
                                       --  4.4, 5.9, 6.2, 10.5, 14.4 sigmas at 1 to 64 times the amounts, the responding
                                       --  cells 171, 261, 250, 235, 206, 189 of 525), took the rounds to 128 times the
                                       --  amounts, the arm swinging into the head camera.
                                       --  And only while one more doubling could make the eye something other than
                                       --  a patch: a patch that grows as a jaw opens (Lockin.Still_A_Patch).
                                       --  Or while the cells that do not respond are pulled along the push together
                                       --  (Rest_Pull: their signed gains pooled stand Z of their spread from zero): a
                                       --  far part of the view that moves by less than any one cell can tell, which
                                       --  the next doublings bring out, however the responding count stood over the
                                       --  last doubling (robot.boot.patch, 10-14: 12 cells at the amounts, 14 at
                                       --  twice them, the far part showing at four times; one doubling's growth
                                       --  alone stopped there, and whether it did depended on which level the ladder
                                       --  of every channel together was first seen at). A base the push shakes
                                       --  moves the cells either way and pools to nothing.
                                       --  And not at all for a group an eye rides on (Ridden): its sweep pushes it.
                                       Again (G) := not Spent (G) and then not Ridden (G)
                                         and then (Again (G) or else Last (G, E) < 0
                                                   or else Pulls (G, E) > Driver.Conventions.Z
                                                   or else (Grew (Last (G, E), Now (G, E), Cells (G, E))
                                                            and then not Lockin.Still_A_Patch (2 * Now (G, E), Cells (G, E))));
                                    end if;
                                 end loop;
                              end loop;
                           end Read_Open;
                        begin
                           loop
                              Driver.Beats.Within_A_Beat (Read_Open'Access);
                              for G in 1 .. Count loop
                                 for E in 1 .. Eyes loop
                                    if Now (G, E) >= 0 then
                                       Driver.Log.Line
                                         (Driver.Log.Robot, "boot: group" & G'Image & " eye" & E'Image & " is "
                                          & (if Shows (G, E) = Patch then "a patch" else "undecided") & " at"
                                          & Integer'Image (Integer (Factor (G))) & " times the amounts:"
                                          & Now (G, E)'Image & " of" & Cells (G, E)'Image & " cells respond"
                                          & (if Last (G, E) >= 0 then ", " & Integer'Image (Last (G, E))
                                             & " before" else "")
                                          & ", the rest move together by " & Driver.Log.Image (Rests (G, E), 2)
                                          & " sigmas and pull along the push by " & Driver.Log.Image (Pulls (G, E), 2));
                                    end if;
                                 end loop;
                              end loop;
                              for G in 1 .. Count loop
                                 if Ridden (G) and then (for some E in 1 .. Eyes => Now (G, E) >= 0) then
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "boot: group" & G'Image & " is ridden by an eye; the eyes"
                                       & " undecided about it or showing a patch are left to its sweep's pushes");
                                 end if;
                              end loop;
                              exit when (for all G in Again'Range => not Again (G));
                              for G in Again'Range loop
                                 if Again (G) then
                                    Factor (G) := 2.0 * Factor (G);
                                    Pause (G) := Next_Pause (G);
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "boot: group" & G'Image
                                       & " leaves an eye undecided or showing a patch that may grow;"
                                       & " pushed again at" & Integer'Image (Integer (Factor (G)))
                                       & " times its amounts");
                                    declare
                                       Moved : Boolean;
                                       Only  : Channel_Flags (1 .. Sizes (G));
                                    begin
                                       for C in Only'Range loop
                                          Only (C) := Needed (G, C);
                                       end loop;
                                       Push_Both_Ways (Group_Id (G), Sizes (G), Factor (G), Moved, Only, Pause (G));
                                       if not Moved then
                                          Spent (G) := True;
                                          Driver.Log.Line
                                            (Driver.Log.Robot, "boot: group" & G'Image
                                             & " moved no further when pushed at"
                                             & Integer'Image (Integer (Factor (G)))
                                             & " times its amounts: its channels are at their ends; not pushed again");
                                       end if;
                                    end;
                                 end if;
                              end loop;
                              Last := Now;
                              Driver.Beats.Within_A_Beat (Estimate'Access);
                           end loop;
                        end;
                     end;
                  end;
               end;
            end;
         end if;
         Driver.Beats.Within_A_Beat (Read_Body'Access);
         if not Recognized_Before then
            Keep ("recognizing the groups");
         end if;
         if not Fitted_Before then
            Sweep_Every_Arm;
            Keep ("sweeping the arms");
         end if;
      end;
      Driver.Robot.Hand.Measure (H, M);
      declare
         procedure Store is
         begin
            Save (M, H, Body_File);
         end Store;
      begin
         Driver.Beats.Within_A_Beat (Store'Access);
      end;
      Ok := not Breach;
   end Run;

   procedure Save (M : Model; H : Driver.Robot.Hand.Hands; Body_File : String) is
      pragma Unreferenced (H);
      Written : Boolean;
   begin
      Driver.Log.Line (Driver.Log.Robot, "the body as measured:" & ASCII.LF & Describe (M));
      if Body_File'Length > 0 then
         Driver.Robot.Body_File.Write (M, Body_File, Written);
         if not Written then
            Driver.Log.Line (Driver.Log.Robot, "the body file " & Body_File & " cannot be written");
         end if;
      end if;
   end Save;

end Driver.Robot.Boot;
