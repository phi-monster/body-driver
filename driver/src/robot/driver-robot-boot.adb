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
with Driver.Robot.Flows;
with Driver.Robot.Kinematics;
with Driver.Robot.Kinematics.Base;
with Driver.Robot.Lag;
with Driver.Robot.Channels;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;
with Driver.Robot.Regression;
with Driver.Robot.Steps;
with Driver.Stats;
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

      --  A push some listed reading went along, significantly: its readings say the body made it.
      function Answered (R : Driver.Robot.Motion.Step_Report) return Boolean is
        (R.Delivered.Value > 0.0
         and then Driver.Uncertain.Significant (R.Delivered.Value, R.Delivered.Sigma, R.Delivered.Degrees_Of_Freedom));

      --  A stay of the pictures told only once the group was sent back (Motion.Judge_Sent_Back) is no speed where the
      --  group's own readings went out with the push and did not come back more than half way (the way back reached and
      --  more than half of it delivered; a way back nothing answered is none): a joint left within its servo's dead
      --  band short of its hold, its own pose keeping the pictures away. Where the readings did not go along the push
      --  by more than their noise (the drone's flight controller's wandering estimates, 0.36 of an ask out), they
      --  tell nothing of it and the pictures decide. Stretch's arm, asked 2.4e-3 units, delivered 0.29 of it: its lift,
      --  pressed down 0.45 mm of the 0.6 asked, stayed there as it was sent back, its readings none of the way back,
      --  and its head camera stayed 1.66 from where the push began against rest's 1.56, a speed (seed 1, paced rounds
      --  p106xa to p108xb and the world's truth, d108x, 10-24); ALOHA's gripper, asked 1.07e-3 units, delivered 0.90,
      --  its way back unanswered, its wrist eye 0.96 from where the push began against rest's 0.61 (seed 1, p113a).
      --  The way back is read once the pictures have settled, from the readings then (Back_Share): read at the end of
      --  the return's step, the drone's speed estimates, answering a beat or more late, had come none of the way back,
      --  and its flight was no speed by it (seed 1, paced rounds p153a and p158a, 10-24).
      function Readings_Allow_Stay (Out_Report : Driver.Robot.Motion.Step_Report; Back : Real) return Boolean is
        (not Answered (Out_Report) or else Back > 0.5);

      --  The group's readings at the latest beat, read in a beat; Fallback where it has none.
      function Readings_Of (G : Group_Id; Size : Positive; Fallback : Real_Array) return Real_Array is
         Result : Real_Array (1 .. Size) := Fallback;
         procedure Read is
         begin
            if Natural (G) <= Group_Count (M) and then Group_Size (M, G) = Size and then M.Beats > 0
              and then Driver.Robot.Channels.Has_Reading (M, G, M.Beats - 1)
            then
               for C in 1 .. Size loop
                  Result (C) := Driver.Robot.Channels.Reading (M, G, M.Beats - 1, C);
               end loop;
            end if;
         end Read;
      begin
         Driver.Beats.Within_A_Beat (Read'Access);
         return Result;
      end Readings_Of;

      --  How much of the way from Away back to Hold the readings Now have come, along that way: 1 where there was none.
      function Back_Share (Away, Now, Hold : Real_Array) return Real is
         Along, Way : Real := 0.0;
      begin
         for C in Away'Range loop
            Along := Along + (Away (C) - Now (C)) * (Away (C) - Hold (C));
            Way := Way + (Away (C) - Hold (C)) ** 2;
         end loop;
         return (if Way > 0.0 then Along / Way else 1.0);
      end Back_Share;

      --  A body kept going moves the views it carries whole: a stay told only by eyes the group moves a patch of, as the
      --  lock-in has it (its jaw in view), is no speed by it, and the group came back as a joint does. ALOHA's gripper,
      --  pushed while the arms pressed together crept, changed its own wrist eye by 1.09 against rest's 0.71, a patch
      --  of it, stayed so as it came back, and was a speed (seed 2, paced round p148a, 10-24).
      --  Only a patch the lock-in can tell is no whole view yet: where the most of the eye's cells that may show the
      --  motion are more than half of them, the view may still move whole with the group. Stretch's base, a patch at
      --  its hold by the few rows it had then and whole by the end of the boot, was no speed by it, and a closer
      --  (seed 1, paced rounds p153a and p153b, 10-24).
      procedure Only_Whole
        (G : Group_Id; Kept : in out Boolean; Kept_Eyes : in out Channel_Flags; Came : in out Boolean)
      is
         --  An eye whose staying away the push itself left mostly unmoved, as it stood (Motion.Note_Stayed, taken as
         --  the settle sent the body back): what stayed is a part the push moved in its view (a box its jaw knocked),
         --  not the view. Not by the lock-in's call of the group a patch in that eye: a walk that moved every view beyond
         --  the flow's reach left no cell answering its pushes, and the lock-in called it a patch in all three eyes of
         --  the G1, whose hand then was its only speed and took the carrier's place (seed 3, round r374, 10-25); the
         --  base of Stretch, crawling at its first level, the same (seed 1).
         procedure Read is
         begin
            for E in 1 .. Eye_Count (M) loop
               declare
                  S : Eye_Stream renames M.Eyes (Eye_Id (E));
               begin
                  if Kept_Eyes (Kept_Eyes'First + E - 1) and then S.Last_Stayed_Group = Natural (G)
                    and then S.Last_Stayed_Cells > 0
                    and then Driver.Robot.Lockin.Mostly (S.Last_Stayed_Cells - S.Last_Stayed_Moved, S.Last_Stayed_Cells)
                  then
                     Kept_Eyes (Kept_Eyes'First + E - 1) := False;
                  end if;
               end;
            end loop;
         end Read;
      begin
         if Kept then
            Driver.Beats.Within_A_Beat (Read'Access);
            if (for all F of Kept_Eyes => not F) then
               Kept := False;
               Came := True;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s pictures stayed only in eyes its push"
                                & " left mostly unmoved: no speed by it");
            end if;
         end if;
      end Only_Whole;

      --  What Recognize found of each channel: the amount an eye sees it move
      --  at (0 when none does) and the ways found at their ends.
      type Recognized_Channel is record
         Ref    : Driver.Robot.Motion.Channel_Ref;
         Amount : Real := 0.0;
         First  : Real := 0.0;   --  the least amount an eye saw it move at (Two_Way_Report.First_Seen)
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
         Out_Report : Driver.Robot.Motion.Step_Report;   --  the latest push's, before it was sent back
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
              --  Not a speed: the lock-in these pushes are for regresses the pictures on the readings' changes, and a
              --  speed held keeps the body going (Motion.Settle); its sighting was its probe's.
              and then not Keeps_Going (M, G, R.Ref.Channel)
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
               Driver.Robot.Motion.Mark_Origins (M);
               Go_To (G, Away, Report);
               --  Further than the round before, which asked half as much, only where more than half of this push's ask
               --  was delivered: a step is reached when nothing blocked it, and a jaw at its ends, its pushes judged
               --  against landings as short as theirs, was pushed again from 64 to 2048 times its amounts, a round of
               --  45 to 55 beats each (ALOHA seed 1, round 4h2, 10-23: 400 of its 1200 beats, out of beats).
               Moved := Moved
                 or else (Report.Outcome = Driver.Robot.Motion.Reached and then Report.Delivered.Value > 0.5);
               Out_Report := Report;
               --  Held after the push for Pause beats before the pictures are asked to settle: a picture that trails
               --  the readings (an eye of the rig ten beats behind them) is settled the beat after they stop, before it
               --  has moved, and pushes a few beats apart make one burst that no lag longer than their spacing can be
               --  told across (Lag: the shifts tried reach the stretch between bursts); the pause is the longest lag
               --  the stream could tell when the round began, until every eye's lag is measured (robot.boot.reread).
               if Pause > 0 then
                  Driver.Robot.Motion.Hold (M, Pause);
                  Settled := Settled + Pause;
               end if;
               --  Held so, a push that keeps the body going drifts the pictures while every commanded reading stands, and
               --  is sent back to the hold (Motion.Settle): the Go2 walked 50 m in round 1 (10-23) through these rounds.
               --  One that drifted no beat's worth is told once sent back below (Motion.Judge_Sent_Back).
               declare
                  Back       : Driver.Commands.Command := Driver.Commands.Hold;
                  Kept, Came : Boolean := False;
                  Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
                  Told       : Boolean := True;   --  the hold told what the channel is, or it is known
                  Pushed     : Real_Array (1 .. Eye_Count (M)) := [others => Real'Last];
                  Out_At     : Real_Array (1 .. Size) := Here;   --  the readings as the push stood
                  Back_Now   : Real := 1.0;   --  how much of the way back they came once settled
                  procedure Note is
                  begin
                     if Came then
                        Note_Comes_Back (M, G, P.Channel);
                     end if;
                     if Kept then
                        Note_Keeps_Going (M, G, P.Channel, Kept_Eyes);
                        Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & P.Channel'Image
                                         & " keeps the body going while held: a speed");
                     end if;
                  end Note;
               begin
                  --  What the push left, as it stood (Settle's Noted; Graph: each eye's own group).
                  if Comes_Back (M, G, P.Channel) then
                     Driver.Robot.Motion.Settle (M, Waited, Noted => G, Way => P.Sign);
                  else
                     Driver.Commands.Set_Target (Back, G, Here);
                     Driver.Robot.Motion.Settle (M, Back, Waited, Kept, Kept_Eyes, Came, Noted => G, Way => P.Sign);
                     Only_Whole (G, Kept, Kept_Eyes, Came);
                     Told := Kept or else Came;
                     Pushed := Driver.Robot.Motion.Eyes_From_Origin (M);
                     Note;
                  end if;
                  Settled := Settled + Waited;
                  --  And back to where it was pushed from, a second push the same size the other way: a round of pushes
                  --  that only go on from where the last one left (item 17's chain) gives the lock-in half the evidence a
                  --  round of pushes out and back does, and an eye whose middle distance showed itself at twice the
                  --  amounts with the old round showed nothing with the chain, so the patch did not grow and the boot
                  --  left the arm a part (robot.boot.patch, robot.boot.step: 13 of 48 cells at twice the amounts against
                  --  17 before). The chain's other saving stands: only the channels an undecided eye does not see.
                  if not Told then
                     Out_At := Readings_Of (G, Size, Here);
                  end if;
                  Go_To (G, Here, Report);
                  if Pause > 0 then
                     Driver.Robot.Motion.Hold (M, Pause);
                     Settled := Settled + Pause;
                  end if;
                  if Told then
                     Driver.Robot.Motion.Settle (M, Waited);
                  else
                     Driver.Robot.Motion.Judge_Sent_Back (M, Pushed, Waited, Kept, Kept_Eyes, Came);
                     Only_Whole (G, Kept, Kept_Eyes, Came);
                     Back_Now := Back_Share (Out_At, Readings_Of (G, Size, Here), Here);
                     if Kept and then not Readings_Allow_Stay (Out_Report, Back_Now) then
                        Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & P.Channel'Image
                                         & "'s pictures stayed once sent back, its readings "
                                         & Driver.Log.Image (Out_Report.Delivered.Value, 2) & " of the ask out and "
                                         & Driver.Log.Image (Back_Now, 2) & " of the way back once settled: no speed by it");
                        Kept := False;
                     end if;
                     Note;
                  end if;
                  Settled := Settled + Waited;
               end;
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
         procedure Mark_Alone is
         begin
            Note_Channels_Alone (M, G);
         end Mark_Alone;
      begin
         Driver.Beats.Within_A_Beat (Mark_Alone'Access);
         for C in 1 .. Size loop
            declare
               Ref : constant Driver.Robot.Motion.Channel_Ref := (Group => G, Channel => C);
               P   : Driver.Robot.Motion.Two_Way_Report;
            begin
               Driver.Robot.Motion.Probe_Both_Ways (M, Ref, From, P);
               if P.Seen then
                  Amount (C) := P.Excursion;
               end if;
               if P.Keeps_Going then
                  Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image
                                   & " keeps the body going while held: a speed");
               end if;
               Recognized.Append
                 (Recognized_Channel'(Ref => Ref, Amount => Amount (C), First => P.First_Seen, Ended => P.At_End));
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

      --  Groups recognized whole (Recognize_Group): the amount each was seen moved by, every channel together; none
      --  for a group whose channels were probed one at a time.
      Whole_Amounts : Real_Vectors.Vector;

      function Whole_Amount (G : Group_Id) return Real is
        (if Natural (G) <= Natural (Whole_Amounts.Length) then Whole_Amounts (Natural (G) - 1) else 0.0);

      procedure Set_Whole_Amount (G : Group_Id; A : Real) is
      begin
         while Natural (Whole_Amounts.Length) < Natural (G) loop
            Whole_Amounts.Append (0.0);
         end loop;
         Whole_Amounts.Replace_Element (Natural (G) - 1, A);
      end Set_Whole_Amount;

      --  Pushes every channel of the group together by Amount, out from its hold and back, one way then the other, for
      --  the lock-in to see what the group moves, a push held until its pictures settle; a group that keeps the body
      --  going (a speed) only until its pictures drift, then sent back (Motion.Settle). Moved: some push reached.
      procedure Push_Group
        (G : Group_Id; Size : Positive; Amount : Real; Moved : out Boolean; Pause : Natural := 0; Way : Real := 0.0)
      is
         Hold   : constant Real_Array := Holds_Of (G, Size);
         Back   : Driver.Commands.Command := Driver.Commands.Hold;
         Report : Driver.Robot.Motion.Step_Report;
      begin
         Moved := False;
         Driver.Commands.Set_Target (Back, G, Hold);
         for Sign of Real_Array'[1.0, -1.0] loop
            if Way = 0.0 or else Sign = Way then
            declare
               Away       : constant Real_Array := [for C in 1 .. Size => Hold (C) + Sign * Amount];
               Kept, Came : Boolean;
               Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
            begin
               Driver.Robot.Motion.Settle (M, Waited);
               Driver.Robot.Motion.Mark_Origins (M);
               Go_To (G, Away, Report);
               --  As above: further than the round before only where more than half of this push's ask was delivered.
               Moved := Moved
                 or else (Report.Outcome = Driver.Robot.Motion.Reached and then Report.Delivered.Value > 0.5);
               if Keeps_Going (M, G) then
                  Driver.Robot.Motion.Settle (M, Back, Waited, Kept, Kept_Eyes, Came);
                  Only_Whole (G, Kept, Kept_Eyes, Came);
               else
                  if Pause > 0 then
                     Driver.Robot.Motion.Hold (M, Pause);
                  end if;
                  --  With the hold to send it back to: a speed its hold took for a joint (Stretch's base, seed 2, paced
                  --  rounds p128a and p129a, 10-24: sent back two beats into its drift, its head camera's change fell
                  --  from 7.06 to 3.93 and it "came back") drives on as it is pushed the other way, and a plain settle
                  --  waited a thousand beats for its view to still. Sent back as its pictures drift on while every
                  --  reading stands, and kept going: a speed after all.
                  Driver.Robot.Motion.Settle (M, Back, Waited, Kept, Kept_Eyes, Came, Noted => G, Way => Sign);
                  Only_Whole (G, Kept, Kept_Eyes, Came);
                  if Kept then
                     for C in 1 .. Size loop
                        Note_Keeps_Going (M, G, C, Kept_Eyes);
                     end loop;
                     Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " keeps the body going as it is"
                                      & " pushed the other way: a speed");
                  end if;
               end if;
               Go_To (G, Hold, Report);
            end;
            end if;
         end loop;
      end Push_Group;

      --  Recognizes the group as a whole (round 3, 10-23: step 1 asks what each group is and where each eye rides,
      --  "推一推每组读数"; one channel at a time, 60 to 75 beats a channel, left ALOHA, the G1, RB-Y1 and the Shadow hand,
      --  14 to 31 channels, unfinished at 1200 beats). Every channel of the group together, by one amount from where
      --  the whole body was first seen, doubling until an eye sees it and the sighting is confirmed
      --  (Motion.Probe_Together); then pushed so and held: a body that keeps going is a speed, and every channel of the
      --  group with it (Motion.Settle); then pushed out and back each way (Push_Group). Each channel keeps the group's
      --  amount, from which step 2's sweeps begin. A group no level shows whole is probed one channel at a time.
      procedure Recognize_Group
        (G : Group_Id; Size : Positive; From : Real; Common : Real := 0.0; Alone : Boolean := False) is
         Refs   : constant Driver.Robot.Motion.Channel_Refs (1 .. Size) := [for C in 1 .. Size => (Group => G, Channel => C)];
         Against : Boolean := False;   --  the hold's push was answered against its ask
         Sign   : Real := 1.0;   --  the way the group was seen moved
         P      : Driver.Robot.Motion.Probe_Report;
         Report : Driver.Robot.Motion.Step_Report;
         Moved  : Boolean;
         --  Each push after the hold, of the group whole: the hold's push, times as many as would move some eye's view
         --  as far as the flow reaches from where it stood (Motion.Reach_Ratio); the hold's amount where none was told.
         --  A just-seen push leaves a view moved by a few tenths of a pixel, and what the graph tells an eye's own
         --  group by, its pushes' cells beyond the flow's reach, never shows: the Shadow's arm, pushed whole from the
         --  4.3e-4 units its readings tell, moved its wrist eye's cells 0.31 px and none beyond reach, its hand's 3 to 7
         --  of them, and both were parts (seed 1, paced round p113a, 10-24).
         Decisive : Real := 0.0;
      begin
         --  The only group that takes commands: the ladder of every channel together was its own, seen and confirmed at
         --  From (Alone), and is not climbed again. Climbed again, the drone's motors needed 28.7 units for its noisy
         --  readings to go along where that ladder saw it at 2.13, and doubled to 1.5e4 before a look of a few beats
         --  saw anything (seed 2, paced round p62a, 10-23: it fell, and was taken for an arm).
         if Alone then
            P.Seen := True;
            P.Excursion := From;
         else
            Driver.Robot.Motion.Probe_Together (M, Refs, 1.0, From, P);
            --  A group at its end one way (a gripper booted open as far as it goes, the Panda's in round 3, 10-23)
            --  answers the other.
            if not P.Seen then
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " moved whole upwards shows nothing confirmed;"
                                & " downwards");
               declare
                  --  Where the upward ladder first moved a reading: downwards, a way that moves none up to it is at its
                  --  end (Motion.Probe_Together).
                  Up : constant Real := P.Answered;
               begin
                  Driver.Robot.Motion.Probe_Together (M, Refs, -1.0, From, P, Other_Way => Up);
               end;
               if P.Seen then
                  Sign := -1.0;
               end if;
            end if;
         end if;
         if not P.Seen then
            Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " moved whole shows nothing confirmed; its"
                             & " channels are probed one at a time");
            Recognize (G, Size, From);
            return;
         end if;
         declare
            Hold       : constant Real_Array := Holds_Of (G, Size);
            Back       : Driver.Commands.Command := Driver.Commands.Hold;
            Kept, Came : Boolean := False;
            Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
            Returned   : Driver.Robot.Motion.Step_Report;
            --  A hold that took no eye's picture beyond rest, neither a speed by it nor a joint, tells nothing of the
            --  group: held again a level up, at twice the amount as the ladder climbs, while the push still went further.
            --  At the level first seen, 0.14 units, Stretch's base moved its wrist eye 1.99 and 2.09 against a bound of
            --  rest between them: a speed in one round, a joint in the next (seed 1, paced rounds p80a and p83a, 10-23).
            --  From the push that moves some eye's view by the flow's reach, as the level seen measured it
            --  (Motion.Probe_Report.Reach): a hold at the level just seen moved ALOHA's wrist eyes 0.36 against rest's
            --  0.59 while its arms, pressed together, crept 0.02 mm a beat, and its grippers were speeds (seed 1,
            --  paced round p118a and the world's truth, d117, 10-24).
            --  At the amount every channel together was seen at (Common), as the groups were all pushed before
            --  their own least steps: one amount for every group, by which the graph tells an eye's own group by
            --  what each group's pushes left beyond the flow's reach; but never further than moves some eye's
            --  view by that reach, as the group's own probe measured it (Motion.Probe_Report.Reach): pushed so far
            --  RB-Y1's torso folded the robot to the floor, the Panda's arm, asked 17 radians' worth, swept its base
            --  camera's view as far as its wrist's and was parted, 500 beats, and the G1 swayed 30 to 70 beats a
            --  push. Never below the level its probe saw it at.
            Amount     : Real :=
              Real'Max (P.Excursion, (if Common > 0.0 then Real'Min (P.Excursion * P.Reach, Common)
                                       else P.Excursion * P.Reach));
            Doubled    : Natural := 0;       --  the hold's doublings, no more than a float has bits
            Ratio      : Real := 1.0;        --  how many times over the hold's push would move some eye's view as far
                                             --  as the flow reaches (Motion.Reach_Ratio), where it stood
            Denied     : Boolean := False;   --  its pictures stayed once sent back, its readings out and not half way
                                             --  back: held again at twice the amount, out of the servos' dead band
         begin
            Driver.Commands.Set_Target (Back, G, Hold);
            loop
               declare
                  Away : constant Real_Array := [for C in 1 .. Size => Hold (C) + Sign * Amount];
               begin
                  Denied := False;
                  Driver.Robot.Motion.Settle (M, Waited);
                  Driver.Robot.Motion.Mark_Origins (M);
                  Go_To (G, Away, Report);
                  --  What the push left, as it stood (Settle's Noted).
                  Driver.Robot.Motion.Settle (M, Back, Waited, Kept, Kept_Eyes, Came, Noted => G, Way => Sign);
                  Only_Whole (G, Kept, Kept_Eyes, Came);
                  if not Kept and then not Came then
                     declare
                        procedure Read_Ratio is
                        begin
                           Ratio := Driver.Robot.Motion.Reach_Ratio (M);
                        end Read_Ratio;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Ratio'Access);
                        Decisive := (if Common > 0.0 then Real'Min (Amount * Ratio, Real'Max (Common, Amount))
                                     else Amount * Ratio);
                     end;
                     --  And sent back with no drift seen while it was held: where its pictures settle tells what it is all
                     --  the same (Motion.Judge_Sent_Back). The drone, its motors asked 0.06 units at the level the lock-in
                     --  confirmed, flew on for 392 beats, too slowly for a beat to show its drift against its hover's sway,
                     --  and was an arm (seed 1, paced round p73a, 10-23).
                     declare
                        Pushed : constant Real_Array := Driver.Robot.Motion.Eyes_From_Origin (M);
                        Out_At : constant Real_Array := Readings_Of (G, Size, Hold);
                        Back_Now : Real;
                     begin
                        Go_To (G, Hold, Returned);
                        Driver.Robot.Motion.Judge_Sent_Back (M, Pushed, Waited, Kept, Kept_Eyes, Came);
                        Only_Whole (G, Kept, Kept_Eyes, Came);
                        Back_Now := Back_Share (Out_At, Readings_Of (G, Size, Hold), Hold);
                        if Kept and then not Readings_Allow_Stay (Report, Back_Now) then
                           Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s pictures stayed once sent"
                                            & " back, its readings " & Driver.Log.Image (Report.Delivered.Value, 2)
                                            & " of the ask out (" & Report.Outcome'Image & ") and "
                                            & Driver.Log.Image (Back_Now, 2) & " of the way back once settled: no speed by it");
                           Kept := False;
                           Denied := True;
                        end if;
                     end;
                  else
                     Go_To (G, Hold, Returned);
                  end if;
               end;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s hold at " & Scientific (Amount)
                                & " units: " & Report.Outcome'Image & ", " & Driver.Log.Image (Report.Delivered.Value, 2)
                                & " of its ask delivered; "
                                & (if Kept then "it kept the body going" elsif Came then "the pictures came back"
                                   else "no eye's picture beyond rest"));
               exit when Doubled >= Real'Machine_Mantissa
                 or else (not Denied
                          and then (Kept or else Came
                                    or else not (Report.Outcome = Driver.Robot.Motion.Reached
                                                 and then Report.Delivered.Value > 0.5)));
               Doubled := Doubled + 1;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s hold at " & Scientific (Amount)
                                & " units took no eye's picture beyond rest: held again at twice that");
               Amount := 2.0 * Amount;
            end loop;
            --  A push the readings answered against more than half its ask (Report.Delivered below minus a half), and
            --  significantly so, was no push the
            --  body made: its channels turned against each other, each read in its own units and its own sign, and the
            --  pictures coming back as it was sent back tell nothing of what each one does alone. RB-Y1's wheels, pushed
            --  together, delivered -1.02 of their ask while the robot rocked and came back, and were a joint by it,
            --  never held again (seed 2, paced rounds p84 to p96, 10-23 and 10-24). Its channels are pushed and held one
            --  at a time instead (Push_Both_Ways), each of which the body makes alone, and the group keeps no whole
            --  amount (Whole_Amount): what follows pushes it a channel at a time. Not a push the readings barely answered
            --  either way: the UR5e's arm, asked 4.3e-4 units, delivered -0.01 of it, significant by its spread, and its
            --  joints held one at a time stayed 1.21 from where their pushes began against rest's 1.17, three speeds
            --  (seed 1, paced rounds p97a and p97b, 10-24).
            Against := not Kept and then Report.Delivered.Value < -0.5
              and then Driver.Uncertain.Significant
                         (-Report.Delivered.Value, Report.Delivered.Sigma, Report.Delivered.Degrees_Of_Freedom);
            if Against then
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s hold went against its ask: its"
                                & " channels are pushed and held one at a time");
            end if;
            for C in 1 .. Size loop
               if Kept then
                  Note_Keeps_Going (M, G, C, Kept_Eyes);
               elsif Came and then not Against then
                  Note_Comes_Back (M, G, C);
               end if;
            end loop;
            if Kept then
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " keeps the body going while held: a speed");
            end if;
            --  A speed's pushes after its hold are its probe's: held, it keeps the body going whatever its amount.
            if Kept then
               Decisive := 0.0;
            end if;
         end;
         --  The hold's push went out and back one way; the other way's completes the pair. Both again after it, the
         --  group waited for its pictures to settle six times, the G1's 40 to 60 beats each after every move of its
         --  balance, 150 to 260 beats a group of its seven (round 4c, 10-23).
         --  Not a speed: its hold showed what it does (the pictures drifting on while every reading stood), and a push
         --  the other way is another walk, flight or drive to send back and wait out (the G1's walk, 275 to 372 beats a
         --  group, round 4e).
         for C in 1 .. Size loop
            Recognized.Append
              (Recognized_Channel'(Ref => (Group => G, Channel => C), Amount => Real'Max (Decisive, P.Excursion), First => P.Excursion,
                                  Ended => [others => False]));
         end loop;
         if not Keeps_Going (M, G) and then Against then
            Push_Both_Ways (G, Size, 1.0, Moved);
         elsif not Keeps_Going (M, G) then
            Push_Group (G, Size, Real'Max (Decisive, P.Excursion), Moved,
                        Pause => Hold_After_A_Push (M, G, Natural'Max (1, Driver.Robot.Lag.Longest (M))), Way => -Sign);
         end if;
         if not Against then
            Set_Whole_Amount (G, Real'Max (Decisive, P.Excursion));
         end if;
         Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " is seen when moved whole by "
                          & Scientific (P.Excursion) & " reading units (after" & P.Steps'Image & " doublings)");
         Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & "'s hold began from " & Scientific (P.Excursion * P.Reach)
                          & " reading units: " & Driver.Log.Image (P.Reach, 2) & " times the level it was seen at");
         Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " is pushed whole by "
                          & Scientific (Real'Max (Decisive, P.Excursion)) & " reading units after its hold, as far as"
                          & " moves some eye's view by the flow's reach");
      end Recognize_Group;

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
         --  The graph no longer has arm A of group G: an estimate made apart derived it again while the arm was swept
         --  (its eyes no longer told it the group carrying one). Its sweep ends there; Sweep_Every_Arm reads the body
         --  again. Read on as arm A, the drone's motors (d501, seed 501 of the default fake robot on the big machine,
         --  10-22) had their least turns asked of an arm the graph no longer had, and the boot failed on Arm_Group's
         --  precondition.
         Gone : Boolean := False;
         function Still_There return Boolean is (Natural (A) <= Arm_Count (M) and then Arm_Group (M, A) = G);
         procedure Read_Size is
         begin
            --  Gone before its sweep began: an estimate while the arm before it was swept derived the graph again
            --  without it (k208, seed 208 of the kiwi on the big machine, 10-22: arm 2 no longer an arm of group 8,
            --  and the boot failed on Arm_Group's precondition for the arm after it).
            if Natural (A) > Arm_Count (M) then
               Gone := True;
            else
               G := Arm_Group (M, A);
               Size := Group_Size (M, G);
            end if;
         end Read_Size;
         --  The reference of the arm's kinematics is the pose it is swept from (Kinematics.Begin_Anew): the
         --  recognition rounds left it where they found it.
         procedure Begin_Anew is
         begin
            Driver.Robot.Kinematics.Begin_Anew (M, A);
         end Begin_Anew;
         --  The body's walk under the arm (Kinematics.Base), before the reference keyframe: where an eye the carrier
         --  moves and the arm does not sees the body (Body_Eye_Of), every channel of what carries it is pushed one way
         --  and the other, the arm held, and once the body has come to rest after each push a still of both eyes is
         --  taken. The
         --  stills tell where that eye sits on the arm's eye and both lenses, and the base's motion under the arm at
         --  every keyframe is taken out of its sightings by them. Each push by the amount the recognition saw the
         --  channel move an eye at (Group_Stream.Seen_At): a speed held until its drift has moved some eye's view as
         --  far as the flow reaches, then sent back to the hold; a joint landed. Sent back as soon as the pictures
         --  drifted, as the recognition holds a speed, Stretch's and RB-Y1's bases turned their eyes 0.01 to 0.1
         --  degrees and stepped them a millimetre or less: nothing to tell an eye's place on another by, and the stills
         --  were adjusted to focal lengths of 3.6 and 35 px (n15s2, 10-27). The hall's G1 leaned with its arms' weight
         --  as their sweeps moved them, and its arms' focal lengths came out 16 to 18 % long on the truth's own
         --  sightings (n13s2, 10-26): its torso's motion, which its head camera sees.
         procedure Walk_Body is
            Body_Eye : Eye_Id'Base := 0;
            Carrier  : Group_Id'Base := 0;
            Size_C   : Natural := 0;
            procedure Read_Walk is
            begin
               Body_Eye := Driver.Robot.Kinematics.Base.Body_Eye_Of (M, A);
               Carrier := Carrier_Group (M);
               if Carrier > 0 and then Is_Commandable (M, Carrier) then
                  Size_C := Group_Size (M, Carrier);
               end if;
            end Read_Walk;
            procedure Begin_Walk is
            begin
               Driver.Robot.Kinematics.Base.Begin_Walk (M, A);
            end Begin_Walk;
            procedure Take_Still is
            begin
               Driver.Robot.Kinematics.Base.Take_Still (M, A);
            end Take_Still;
         begin
            Driver.Beats.Within_A_Beat (Read_Walk'Access);
            if Natural (Body_Eye) = 0 or else Size_C = 0 then
               return;
            end if;
            declare
               Amount  : Real_Array (1 .. Size_C) := [others => 0.0];
               Hold    : constant Real_Array := Holds_Of (Carrier, Size_C);
               Report  : Driver.Robot.Motion.Step_Report;
               Back    : Driver.Commands.Command := Driver.Commands.Hold;
               Speed   : Boolean := False;
               Stills  : Natural := 0;
               Began_At, Ended_At : Natural := 0;
               procedure Read_Amounts is
               begin
                  for C in 1 .. Size_C loop
                     if C <= Natural (M.Groups (Carrier).Seen_At.Length) then
                        Amount (C) := M.Groups (Carrier).Seen_At (C - 1);
                     end if;
                  end loop;
                  Speed := Keeps_Going (M, Carrier);
                  Began_At := M.Beats;
               end Read_Amounts;
               procedure Read_End is
               begin
                  Ended_At := M.Beats;
               end Read_End;
               procedure Still is
               begin
                  Driver.Beats.Within_A_Beat (Take_Still'Access);
                  Driver.Robot.Motion.Hold (M, 1);
                  Stills := Stills + 1;
               end Still;
            begin
               Driver.Beats.Within_A_Beat (Read_Amounts'Access);
               if (for all X of Amount => X <= 0.0) then
                  return;
               end if;
               Driver.Beats.Within_A_Beat (Begin_Walk'Access);
               Driver.Commands.Set_Target (Back, Carrier, Hold);
               for C in 1 .. Size_C loop
                  if Amount (C) > 0.0 then
                     for Sign of Real_Array'[1.0, -1.0] loop
                        declare
                           Away : Real_Array := Hold;
                           Kept, Came : Boolean;
                           Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
                        begin
                           Away (C) := Hold (C) + Sign * Amount (C);
                           Driver.Robot.Motion.Settle (M, Waited);
                           Driver.Robot.Motion.Mark_Origins (M);
                           Go_To (Carrier, Away, Report);
                           --  A speed is held until its drift has taken some eye's view as far as the flow reaches
                           --  (Motion.Settle's Until_Reach), and sent back to the hold then, or once the body stops.
                           if Speed then
                              Driver.Robot.Motion.Settle (M, Back, Waited, Kept, Kept_Eyes, Came, Until_Reach => True);
                              Go_To (Carrier, Hold, Report);
                           end if;
                           Driver.Robot.Motion.Settle (M, Waited);
                           Still;
                        end;
                     end loop;
                  end if;
               end loop;
               Go_To (Carrier, Hold, Report);
               Driver.Robot.Motion.Settle (M, Waited);
               Driver.Beats.Within_A_Beat (Read_End'Access);
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & "'s body walked by group" & Carrier'Image
                                & " (" & (if Speed then "speeds" else "joints") & "), seen by eye" & Body_Eye'Image
                                & ":" & Stills'Image & " stills in" & Integer'Image (Ended_At - Began_At) & " beats");
            end;
         end Walk_Body;
      begin
         Driver.Beats.Within_A_Beat (Read_Size'Access);
         if Gone then
            Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " is no longer an arm in the graph: its sweep does not"
                             & " begin");
            return;
         end if;
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
         Walk_Body;
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
            Least  : Real_Array (1 .. Size) := [others => 0.0];   --  per channel: the least turn a keyframe counts, now
            --  Per channel, for the plan's log: what its least turn is made of (the turn the fit counts, the least
            --  ask the body is known to execute, its dead band, the readings' noise), read in the plan's held beat.
            Why_Least : array (1 .. Size) of Ada.Strings.Unbounded.Unbounded_String;
            Moves  : array (1 .. Size) of Boolean := [others => False];   --  per channel: it moves the eye's view (Lockin.Seen)
            --  Per channel: the lock-in saw the eye's whole view move with it (Graph.Channels, Whole), the eye
            --  riding on the joint; read in the plan's held beat.
            Whole_View : array (1 .. Size) of Boolean := [others => False];
            Half   : Real := 0.0;
            Patch  : Real := 0.0;   --  a patch of the eye's view, in px: the farthest the aligner follows by itself
            Eye    : Natural := 0;
            --  How far the view moves per reading unit of each channel, as the keyframes have measured it so far
            --  (Kinematics.Pixels_Per_Unit; 0 until the keyframes determine the channel): read again after every
            --  round, since the rounds' keyframes measure it. And the least turn a keyframe counts with it
            --  (Motion.Least_Turn): the step a keyframe counts from is the matcher's noise over that motion, and
            --  shrinks as the keyframes measure it (s35, servo seed 6 of the fake robot, 10-17: its fifth joint's
            --  step from 53.4 units after the first two passes to 9.8 after the fifth).
            procedure Read_Shifts is
            begin
               for C in 1 .. Size loop
                  Per_Unit (C) := Driver.Robot.Kinematics.Pixels_Per_Unit (M, A, C);
                  Least (C) := Driver.Robot.Motion.Least_Turn (M, A, C);
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
                     for Item of M.Graph.Channels (Eye_Id (Eye)) loop
                        if Item.Group = G and then Item.Channel = C
                          and then Item.Effect.Verdict = Driver.Robot.Whole
                        then
                           Whole_View (C) := True;
                        end if;
                     end loop;
                     First (C) := Driver.Robot.Motion.Sweep_Start (M, A, C);
                     Why_Least (C) := Ada.Strings.Unbounded.To_Unbounded_String
                       (": counted " & Driver.Log.Image (Driver.Robot.Kinematics.Counted_Turn (M, A, C), 3)
                        & ", executed least "
                        & (if Driver.Robot.Steps.Executed_Least (M, G, C) < Real'Last
                           then Driver.Log.Image (Driver.Robot.Steps.Executed_Least (M, G, C), 3) else "none")
                        & ", dead band "
                        & (if Driver.Robot.Steps.Dead_Band (M, G, C) < Real'Last
                           then Driver.Log.Image (Driver.Robot.Steps.Dead_Band (M, G, C), 4) else "none")
                        & ", standing off "
                        & (if Driver.Robot.Steps.Standoff (M, G, C) < Real'Last
                           then Driver.Log.Image (Driver.Robot.Steps.Standoff (M, G, C), 4) else "none")
                        & ", noise "
                        & (if Driver.Robot.Channels.Noise (M, G, C) < Real'Last
                           then Driver.Log.Image (Driver.Robot.Channels.Noise (M, G, C), 4) else "unknown"));
                     --  And no less than the turn at which the recognition's probe saw the eye's view move with the
                     --  channel, an executed turn the eye saw (Probe_Both_Ways): the first keyframe needs the view to
                     --  have moved, and before any keyframe the step a keyframe counts from may be none (the rig of
                     --  robot.boot.short: no visible step from the lock-in, a first level of 0, nothing swept).
                     --  A channel the probe saw move an eye moves the eye, whatever the latest lock-in estimate kept as
                     --  regressors: the estimate before R6's third sweep (10-14) kept 9 columns of 12, its rows after
                     --  the rounds' pushes mostly in settling tails of 274 beats, and arm_a's second, third and fourth
                     --  joints were left unswept though the probe had seen each at 9.8 to 157 units.
                     --  Read from the model (Group_Stream.Seen_At), which the body file keeps: a boot begun from the
                     --  body file kept after recognizing the groups had no probe's turn here, swept from the least turn
                     --  a keyframe counts (0.001 to 0.015 units where the probe had seen 0.016 to 0.126), and the
                     --  joints whose first pushes moved no view were left as not moving the eye (f16e on 10-15).
                     if C - 1 <= M.Groups (G).Seen_At.Last_Index then
                        First (C) := Real'Max (First (C), M.Groups (G).Seen_At (C - 1));
                        Moves (C) := Moves (C) or else M.Groups (G).Seen_At (C - 1) > 0.0;
                     end if;
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

            --  The next estimate fits the arm whatever its keyframes since its last fit (Kinematics.Ask_Fit), where
            --  the next round's cells are the pairs a fit of every keyframe taken names (Contested): a fitted arm is
            --  fitted again once its keyframes have doubled otherwise (Kinematics.Refit).
            procedure Ask_For_Fit is
            begin
               Driver.Robot.Kinematics.Ask_Fit (M, A);
            end Ask_For_Fit;

            --  Whether the arm is fitted now, read in a held beat (the levels' own verdicts: Read_Round).
            Fitted_Now   : Boolean := False;
            Told_Now     : Boolean := False;   --  and its joints' order told (Kinematics.Order_Told)
            Contested    : Joint_Pair_Lists.Vector;   --  if not, the pairs it lacks
            Turns        : Real_Vectors.Vector;       --  and the turns of each one's two joints that tell it, two a pair
            Untold_Now   : Natural := 0;              --  how many orders the fit does not tell from its own
            procedure Read_Followed is
            begin
               if not Still_There then
                  Gone := True;
                  Fitted_Now := False;
                  Told_Now := False;
                  Contested.Clear;
                  return;
               end if;
               Fitted_Now := Driver.Robot.Kinematics.Fitted (M, A);
               Told_Now := Driver.Robot.Kinematics.Order_Told (M, A);
               Contested := Driver.Robot.Kinematics.Contested_Pairs (M, A);
               Turns := Driver.Robot.Kinematics.Contested_Turns (M, A);
               Untold_Now := Driver.Robot.Kinematics.Untold_Count (M, A);
            end Read_Followed;
            --  The beats the body has seen, read in a held beat.
            Beat_Now : Natural := 0;
            procedure Read_Beat_Now is
            begin
               Beat_Now := M.Beats;
            end Read_Beat_Now;
            --  The two at once: a push's channels told to the kinematics in the beat that reads where its first level
            --  begins, and the beat that reads where its last level ends tells them the push is over (Take).
            procedure Tell_And_Read is
            begin
               Tell_Asking;
               Read_Beat_Now;
            end Tell_And_Read;
            procedure Read_And_Tell is
            begin
               Read_Beat_Now;
               Tell_Asking;
            end Read_And_Tell;
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
               --  Per push: the farthest a level of it asked, and the farthest along the push its channel's reading stood
               --  after one, from Start: what tells a level that took the joint no farther than one before it (Take).
               Asked_Most : Real_Array (1 .. 2 * Size) := [others => 0.0];
               Along_Most : Real_Array (1 .. 2 * Size) := [others => 0.0];
               --  Per push: the farthest its channel's reading stood beyond what a level of it asked, or back against it,
               --  after one: how far something other than the push carries the reading (Take).
               Carried_Most : Real_Array (1 .. 2 * Size) := [others => 0.0];
               --  Per push: its ladder is going down from a first level the aligner could not follow.
               Descending : array (1 .. 2 * Size) of Boolean := [others => False];
               --  A ladder that went down from a lost first level to the least turn and followed nothing on the way is tried
               --  upward once, from twice that level (Up_From, where the descent began): the level was too small for the
               --  matcher to tell from none, not too wide to follow. The drone's z slide, a first level of 53 micrometres
               --  (the step the probe saw the eye move by: a zoom of 1E-4, a hundredth of a pixel at the edges), found 0 of
               --  255 points, and the ladder went down to 27 and stopped, the joint with no keyframe of its own and the
               --  drone never fitted.
               Up_From    : array (1 .. 2 * Size) of Real := [others => 0.0];
               Tried_Up   : array (1 .. 2 * Size) of Boolean := [others => False];
               --  Per push: the widest level the aligner followed, and the least it could not after one it did. The
               --  ladder halves the gap between them, a level a pass (Take climbs no level as wide as the second).
               Followed_Most : Real_Array (1 .. 2 * Size) := [others => 0.0];
               Lost_Least    : Real_Array (1 .. 2 * Size) := [others => Real'Last];
               --  Whether the level halfway between them would tell something: half the gap moves the view by more
               --  than a patch, the most the aligner follows beyond a level by itself (Take's brake), and is a step a
               --  keyframe counts. Halving to the counted step alone took a pass a level, each an estimate and a set
               --  of cells, for levels a few pixels apart (f16b on 10-15: arm 1's channel 1 at 3.03, 3.53, 3.78 of a
               --  gap up to 4.04 units, 3000 beats a pass).
               function Halving_Tells (I : Positive; Channel : Positive) return Boolean is
                 ((Lost_Least (I) - Followed_Most (I)) / 2.0 >= Least (Channel)
                  and then (Per_Unit (Channel) = 0.0
                            or else (Lost_Least (I) - Followed_Most (I)) / 2.0 * Per_Unit (Channel) > Patch));
               --  Per push: the levels taken in the pass, in order: each one's offset and the beats the arm was held
               --  for its keyframe.
               type Level_Mark is record
                  Offset   : Real := 0.0;
                  From, To : Natural := 0;
                  Short    : Boolean := False;   --  the step to it moved the joint no way along it: it met something
                  Followed : Boolean := False;   --  after the round's estimate: its keyframe was followed
                  Framed   : Boolean := False;   --  and one was taken at all (Kinematics.Followed, Framed: Read_Round)
               end record;
               package Level_Vectors is new Ada.Containers.Vectors (Positive, Level_Mark);
               Levels : array (1 .. 2 * Size) of Level_Vectors.Vector;
               --  Per channel: the largest offset a followed keyframe of it was taken at.
               Reached : Real_Array (1 .. Size) := [others => 0.0];
               Swept   : array (1 .. Size) of Positive;
               N       : Natural := 0;
               Rounds, Passes : Natural := 0;

               --  What the round's estimate told, read in one held beat after it: how far the view moves per unit of
               --  each channel and the channels it does not see move it (Read_Shifts, Read_Unseen), of every level the
               --  climbing pushes took whether its keyframe was followed and whether one was taken at all, and whether
               --  the arm is fitted and its order told (Read_Followed). Each read took a held beat of its own, one a
               --  level, and nothing moved meanwhile: 14 to 22 held beats after each of s43's estimates (servo seed 6 of
               --  the fake robot, 10-17), its sweep 2946 beats.
               procedure Read_Round is
               begin
                  if not Still_There then
                     Gone := True;
                     return;
                  end if;
                  Read_Shifts;
                  Read_Unseen;
                  for I in 1 .. Count loop
                     if State (I) = Climbing then
                        for K in Levels (I).First_Index .. Levels (I).Last_Index loop
                           declare
                              L : Level_Mark := Levels (I) (K);
                           begin
                              L.Followed := Driver.Robot.Kinematics.Followed (M, A, L.From, L.To);
                              L.Framed := Driver.Robot.Kinematics.Framed (M, A, L.From, L.To);
                              Levels (I).Replace_Element (K, L);
                           end;
                        end loop;
                     end if;
                  end loop;
                  Read_Followed;
               end Read_Round;

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
                  Last : Boolean;   --  the level just taken is the push's last
                  Widest_Next : Boolean := False;   --  the next level is the widest turn itself
                  Previous : Real := 0.0;   --  the level before this one in the push: its step asks the difference
               begin
                  Levels (I).Clear;
                  Asked_Now := [others => False];
                  Asked_Now (P.Channel) := True;
                  --  Each held beat that reads where the stream stands does what the beats beside it need: the one that
                  --  tells the kinematics the push's channels reads where the first level begins, the one that ends a
                  --  level begins the next, and the last tells them the push is over. Each read had a held beat of its
                  --  own, four a push of one level, where nothing moved (s43, servo seed 6 of the fake robot, 10-17: 23
                  --  beats between the keyframes of its first pass).
                  Driver.Beats.Within_A_Beat (Tell_And_Read'Access);
                  loop
                     Pose (P.Channel) := Start (P.Channel) + P.Sign * Off;
                     Mark := (Offset => Off, From => Beat_Now, To => 0, Short => False, others => <>);
                     Go_To (G, Pose, Report, Land => True);
                     if Report.At_Rest then
                        Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                     end if;
                     --  A step that took the joint part of the way is a level it went to: a servo stops short
                     --  of its target by its dead band, and an arm under load by its sag, by more than the
                     --  recognition's larger pushes did, and what the keyframe counts is where the readings
                     --  stand. Judged blocked as falling short, every first level of the fake's seed 20 (10-22,
                     --  default options) ended its push: 0.48 to 0.75 of each delivered, the arm swept in one
                     --  pass and never fitted. A step that moved it no way along its ask is where something
                     --  stops it.
                     --  Where the channel stands along the push now, by its reading: a level asked farther than any before
                     --  it that left the channel no farther along than one before it did, by a change the one test of
                     --  motion sees, met the joint's end from this pose, however the step judged itself: a servo stopped
                     --  at its end delivers part of the ask and reads Short, as one its dead band stops short of a level
                     --  it can reach does. t502 of the race (10-23): channel 4's ladder doubled once a pass from pass 4
                     --  to pass 63, its asks to 1.1E19 units, no level of it framed, the view's motion per unit of the
                     --  joint never measured, so no level was the widest either.
                     --  Neither is judged within how far the joint stands off a target at rest (Steps.Standoff: its
                     --  dead band and the load it holds): a joint carrying its arm's weight lands off its target by its
                     --  sag, which shifts with the way it came and with the other joints' poses, and a level asked no
                     --  farther than that beyond the one before tells nothing of an end. A step blocked within it is
                     --  one the joint stood still for; a reading no farther along than a level's before it, where the
                     --  ask grew by less than twice it (each level's reading standing off its target by up to it), is
                     --  no farther by the sag. Judged at every such level, the hall's loaded joints were taken for
                     --  ended at their first levels and never pushed again (round n15s2, 10-27: of the pushes judged no
                     --  farther, 35 of the Shadow's 41, 15 of the G1's 26, 10 of the UR5e's 23 had read along by more
                     --  than they asked or the other way; UR5e 3b's elbow, asked 0.0043 units, read 0.0253 along
                     --  against 0.0292 before, its sag about 0.02, turned 0.008 rad in the whole sweep, and the arm
                     --  ended not fitted).
                     --  Nor within how far something other than the push has carried the reading at the push's levels
                     --  (Carried_Most: its reading beyond what a level asked, or back against the push): the sag shifts
                     --  between the levels with the other joints' poses by more than the standoff measured at rest
                     --  shows, and a level's reading is the push's only beyond that. Round n25s2 (10-27, the standoff
                     --  judged as above): of its 136 pushes judged no farther, 63 had read along by more than twice their
                     --  ask or the other way (the Shadow's 38 of 51, the G1's 13 of 27, the UR5e's 9 of 15, Stretch's 3 of
                     --  6; none of the ALOHA's, Panda's, RB-Y1's or SO-101's); held as well to twice the farthest carry,
                     --  59 of them are judged no more, the 4 left reading within 0.03 units of nought where 0.03 to 0.46
                     --  were asked; of the other 73, the 18 whose reading had run past an ask at a level before are
                     --  judged no more, and the 55 left include every reading that stood still as its ask grew (Panda 1a's
                     --  seventh channel at 0.31 of 0.68 asked, the Shadow 1a's sixth at 50.3 of 189).
                     declare
                        Further : Boolean := False;
                        Along   : Real := 0.0;
                        Stands  : Real := 0.0;   --  the joint's standoff at rest (Steps.Standoff)
                        Bound   : Real := 0.0;   --  and no less than the reading's farthest carry (Carried_Most)
                        procedure Read_Along is
                           D : Real_Array (1 .. Size) := [others => 0.0];
                        begin
                           Stands := Driver.Robot.Steps.Standoff (M, G, P.Channel);
                           if M.Beats > 0 then
                              Along := P.Sign * (Channels.Reading (M, G, M.Beats - 1, P.Channel) - Start (P.Channel));
                              D (P.Channel) := Along - Along_Most (I);
                              Further := Along > Along_Most (I) and then Channels.Visible (M, G, D);
                           end if;
                        end Read_Along;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Along'Access);
                        Carried_Most (I) := Real'Max (Carried_Most (I), Real'Max (Along - Off, -Along));
                        Bound := Real'Max (Stands, Carried_Most (I));
                        Mark.Short := (Report.Outcome = Driver.Robot.Motion.Blocked and then Off - Previous > Bound)
                          or else (Off > Asked_Most (I) and then Asked_Most (I) > 0.0 and then not Further
                                   and then (Off - Asked_Most (I)) / 2.0 > Bound);
                        --  Said with the readings it was judged from: SO-101 seed 1's fifth channel (n7s2 1b, n9s2 1a,
                        --  10-26), its readings in steps of 4 units, was judged at its end in its second pass both ways
                        --  and never pushed again, and the log named neither reading.
                        if Mark.Short and then Report.Outcome /= Driver.Robot.Motion.Blocked then
                           Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " push c"
                                            & Driver.Log.Image (P.Channel) & (if P.Sign > 0.0 then "+" else "-")
                                            & " at " & Driver.Log.Image (Off, 4) & " units reached, its reading "
                                            & Driver.Log.Image (Along, 4) & " along it, against "
                                            & Driver.Log.Image (Along_Most (I), 4) & " at the levels before (the farthest "
                                            & Driver.Log.Image (Asked_Most (I), 4) & "), its farthest carry "
                                            & Driver.Log.Image (Carried_Most (I), 4) & ": no farther, judged at its end");
                        end if;
                        Asked_Most (I) := Real'Max (Asked_Most (I), Off);
                        Along_Most (I) := Real'Max (Along_Most (I), Along);
                        Previous := Off;
                     end;
                     if Report.Outcome /= Driver.Robot.Motion.Reached then
                        Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " push c"
                                         & Driver.Log.Image (P.Channel)
                                         & (if P.Sign > 0.0 then "+" else "-") & " at " & Driver.Log.Image (Off, 4)
                                         & " units ended "
                                         & Ada.Characters.Handling.To_Lower (Report.Outcome'Image) & ": "
                                         & Ada.Strings.Unbounded.To_String (Report.Detail));
                     end if;
                     Last := Mark.Short or else Per_Unit (P.Channel) = 0.0;
                     if not Last then
                        --  Twice the level, or the level that moves the view by a patch once the view's motion per unit is
                        --  measured, whichever is wider: a patch beyond the followed level is what the aligner follows by
                        --  itself (the brake below), and doubling from the least turn a keyframe counts took six to eight
                        --  levels a push to get there, each a keyframe aligned and fitted (f16b on 10-15: pass 2 of arm 1, 69
                        --  keyframes from 0.03 units to 2 to 8).
                        declare
                           Taken : constant Real := Off;   --  the level just taken
                        begin
                           Off := (if Per_Unit (P.Channel) > 0.0 then Real'Max (2.0 * Off, Patch / Per_Unit (P.Channel))
                                   else 2.0 * Off);
                           Last := Off >= Lost_Least (I);
                           --  The widest turn is the one that moves the view half of it (Half), and a doubling that would
                           --  pass it stops the ladder short of it, at a half to the whole of it: the turn itself is taken
                           --  next where it is a step the fit counts beyond the level just taken (Least) and short of a
                           --  level lost before. Stretch's head (n3s2, seed 2a) stopped at 66 px of its 120, 17 degrees
                           --  of pan: a turn's focal length is told from how its picture bends across the view, which
                           --  grows with the turn's square, and the order of two joints from their turns together, by the
                           --  product of the two (its cells turn each by a share of its widest).
                           if not Last and then Fitted_Now and then Per_Unit (P.Channel) > 0.0
                             and then Off * Per_Unit (P.Channel) > Half
                             and then Half / Per_Unit (P.Channel) - Taken > Least (P.Channel)
                             and then Half / Per_Unit (P.Channel) < Lost_Least (I)
                           then
                              Off := Half / Per_Unit (P.Channel);
                              Widest_Next := True;
                           end if;
                        end;
                     end if;
                     if not Last and then Widest_Next then
                        Widest_Next := False;   --  the widest level itself is taken next, and is the last
                     elsif not Last then
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
                        Last := Off * Per_Unit (P.Channel) > (if Fitted_Now then Half else 2.0 * Patch);
                     end if;
                     if Last then
                        Asked_Now := [others => False];
                        Driver.Beats.Within_A_Beat (Read_And_Tell'Access);
                     else
                        Driver.Beats.Within_A_Beat (Read_Beat_Now'Access);
                     end if;
                     Mark.To := (if Beat_Now > 0 then Beat_Now - 1 else 0);
                     Levels (I).Append (Mark);
                     exit when Last;
                  end loop;
               end Take;

               --  Every swept joint at once, in cells whose signs no two share
               --  (every row of a Sylvester-Hadamard matrix, its columns after
               --  its first): a joint's own frames tell its axis
               --  but not how far that axis is from the others', which only
               --  frames that move several joints show. Each joint moves by its
               --  share: its part of the offset its own keyframes were followed
               --  to, so that the cell moves the view no farther than the aligner
               --  followed one joint, or the fitted arm predicts; and no less
               --  than the keyframe step, the smallest turn the fit counts as a
               --  joint having moved (a sixth of three steps is none, and a cell
               --  of such parts moves no two joints the fit can see).
               --  The joints with no share stand at the start, and the matrix's columns are the ones of those with one.
               --  Every row, the first too (every joint by its share the same way): over all of them each joint turns
               --  as often one way as the other, and so does the product of any two joints' turns, which is what tells
               --  their order (the matrix's columns are orthogonal, and the product of two of them is a third). Without
               --  the first, a product's sign was the opposite of its two joints' in one cell more than the same (three
               --  of seven, of six joints; one of three, of a pair), and an error of the fitted arm that is the same in
               --  every cell (one even in the turns: as large for a joint turned either way) told the order the cells'
               --  majority leaned to: s43 (servo seed 6 of the fake robot, 10-17) told 1 4 3 2 5 6 for 1 3 4 2 5 6, the
               --  wrong order the smaller in its four cells that turned channels 3 and 4 opposite ways (by 0.00 to
               --  0.15 px a residual) and the true one in its three that turned them the same way (by 0.01 to 0.13),
               --  over 1148 residuals that the signed ranks counted as many independent ones.
               procedure Together (Share : Real_Array; Climb : Boolean) is
                  Rows   : Positive := 1;
                  Turned : array (1 .. N) of Positive;   --  the swept joints with a share, by their place in Swept
                  Count  : Natural := 0;
               begin
                  for K in 1 .. N loop
                     if Share (Swept (K)) > 0.0 then
                        Count := Count + 1;
                        Turned (Count) := K;
                     end if;
                  end loop;
                  while Rows < Count + 1 loop
                     Rows := 2 * Rows;
                  end loop;
                  Asked_Now := [others => False];
                  for J in 1 .. Count loop
                     Asked_Now (Swept (Turned (J))) := True;
                  end loop;
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
                  for Row in 0 .. Rows - 1 loop
                     declare
                        Scale : Real := 1.0;   --  of the shares: halved where the cell met something

                        --  The view's motion a cell asks at a scale of its shares, at most: every joint's part by how far
                        --  the view moves per unit of it.
                        function Cell_Motion (At_Scale : Real) return Real is
                           Sum : Real := 0.0;
                        begin
                           for K in 1 .. N loop
                              Sum := Sum + At_Scale * Share (Swept (K)) * Per_Unit (Swept (K));
                           end loop;
                           return Sum;
                        end Cell_Motion;

                        procedure Take_Cell (At_Scale : Real) is
                           Pose : Real_Array := Start;
                        begin
                           for J in 1 .. Count loop
                              declare
                                 K    : constant Positive := Turned (J);
                                 Bits : Natural := 0;
                                 R    : Natural := Row;
                                 Col  : Natural := J;
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
                                   + (if Bits mod 2 = 0 then 1.0 else -1.0) * At_Scale * Share (Swept (K));
                              end;
                           end loop;
                           Go_To (G, Pose, Report, Land => True);
                           declare
                              Said : Ada.Strings.Unbounded.Unbounded_String;
                           begin
                              for J in 1 .. Count loop
                                 Ada.Strings.Unbounded.Append
                                   (Said, " c" & Swept (Turned (J))'Image & " "
                                    & Driver.Log.Image (Pose (Swept (Turned (J))) - Start (Swept (Turned (J))), 3));
                              end loop;
                              Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " cell" & Row'Image & " asks"
                                               & Ada.Strings.Unbounded.To_String (Said) & " units: "
                                               & Ada.Characters.Handling.To_Lower (Report.Outcome'Image)
                                               & (if Report.At_Rest then ", at rest" else ", not at rest"));
                           end;
                           if Report.At_Rest then
                              Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                           end if;
                        end Take_Cell;
                     begin
                        loop
                           Take_Cell (Scale);
                           --  A cell that met something (a keyframe where it stopped, if it
                           --  rested there) is tried again at half its shares, as long as some
                           --  share is still a turn the fit counts: a cell need not move every
                           --  joint, and one that always meets something gives no frame at all
                           --  (A99's arm 1 had no keyframe of several joints for that). One that
                           --  went part of the way is a cell it went to, as a push's level is: a
                           --  servo stops short of its target by its dead band, and the readings
                           --  say where. Tried again at half when short, the fake's seed 20
                           --  (10-22, default options) had 116 of its 120 cells end short and its
                           --  pairs turned at a quarter of the turns its fit named for them.
                           exit when Report.Outcome /= Driver.Robot.Motion.Blocked;
                           Scale := Scale / 2.0;
                           exit when (for all K in 1 .. N => Scale * Share (Swept (K)) < Least (Swept (K)));
                        end loop;
                        --  A fitted arm's cell climbs on in its direction, its shares doubled a level, while the
                        --  view moves by no more than half of it (Take's brake for a fitted arm), each level
                        --  predicted from the one before as a push's levels are. The order of the joints along the arm
                        --  is told by what two joints turned together do that each alone does not, and that grows with
                        --  the product of their turns: at a sixth of each joint's widest followed offset (the shares),
                        --  f16b's cells turned every joint about 4 degrees, a pair's difference under a pixel, and its
                        --  arm 1 was fitted in the wrong order in every pass (4, 1, 5, 2, 6, 3 where it is 5, 4, 1, 2,
                        --  6, 3), its median residual twice the right order's.
                        if Climb and then Fitted_Now and then Report.Outcome /= Driver.Robot.Motion.Blocked then
                           loop
                              exit when Cell_Motion (2.0 * Scale) > Half;
                              Scale := 2.0 * Scale;
                              Take_Cell (Scale);
                              exit when Report.Outcome = Driver.Robot.Motion.Blocked;
                           end loop;
                        end if;
                     end;
                  end loop;
                  Asked_Now := [others => False];
                  Driver.Beats.Within_A_Beat (Tell_Asking'Access);
                  Go_To (G, Start, Report, Land => True);
               end Together;

               --  Each swept joint's share of a frame that moves them together: its part of the offset it was
               --  followed to, and no less than the least turn a keyframe counts (Least: the turn the fit counts as
               --  the joint having moved, with what the readings' noise and two landings in a dead band can take off
               --  it; Together). The floor was the joint's first level while that was the keyframe step (10-11);
               --  the first level has since grown to what the aligner follows by itself and to the turn at which the
               --  recognition's probe saw the view move (Read_Plan), and a cell of first levels moved the view
               --  farther than any keyframe was followed: s35 (servo seed 6 of the fake robot, 10-17), whose fifth
               --  joint was at its end upwards and seen downwards only at 627 units, its first level, turned that
               --  joint by 618 to 622 units in every cell, about 62 px of view at its 0.10 px a unit beside the other
               --  joints' first levels, and asked them from a prediction stating 68 to 76 px; no point of a cell was
               --  found in five passes, the view's common motion untold, no keyframe moved two joints at once, and
               --  the arm was never fitted.
               function Shares return Real_Array is
                  Share  : Real_Array (1 .. Size) := [others => 0.0];
                  Motion : Real := 0.0;   --  the view's motion the cell asks, px, by the measured motion per unit
                  Fixed  : Real := 0.0;   --  of it, the part the least turns hold
               begin
                  for K in 1 .. N loop
                     Share (Swept (K)) := Real'Max (Least (Swept (K)), Reached (Swept (K)) / Real (N));
                     Motion := Motion + Share (Swept (K)) * Per_Unit (Swept (K));
                     Fixed := Fixed + Least (Swept (K)) * Per_Unit (Swept (K));
                  end loop;
                  --  And no more motion of the view in all than the ladder's own brake before a fit, two patches
                  --  beyond the keyframes before (Take): the shares above their least turns are scaled down to it,
                  --  the least turns themselves stand (below them the fit counts no turn).
                  if not Fitted_Now and then Motion > 2.0 * Patch and then Motion > Fixed then
                     declare
                        Scale : constant Real := Real'Max (0.0, (2.0 * Patch - Fixed) / (Motion - Fixed));
                     begin
                        for K in 1 .. N loop
                           Share (Swept (K)) :=
                             Least (Swept (K)) + Scale * (Share (Swept (K)) - Least (Swept (K)));
                        end loop;
                     end;
                  end if;
                  return Share;
               end Shares;

               --  The shares of a cell that turns two joints alone, none of the others'. What two joints turned
               --  together do that each alone does not grows with the product of their turns (s08 on 10-16: the cells of
               --  every joint at the end of its passes turned each by about 0.08 rad, and the orders 5 1 4 3 2 6 and
               --  1 5 4 3 6 2 left residuals of median 0.361 and 0.363 px), and x7 (servo seed 7 of the fake robot,
               --  10-18) turned its base pair c4 with c5 by 18 and 12 units, a sixth of their reach: no keyframe of it
               --  was fresh, and its order stayed 5 4 1 2 3 6 for the true 4 5 1 2 3 6. Turned by the offsets its own
               --  keyframes were followed to, and climbing as a fitted arm's cells do until the view moved by half of
               --  it, a pair put its untold orders' places of the points farther apart than a narrow window reaches,
               --  and its keyframes were not followed (u6, servo seed 3 of the fake robot, 10-19: c2 with c6 by 1784 and
               --  1806 units, the orders' places 140 px apart). Each of the two by the turn the fit names
               --  (Kinematics.Contested_Turns: of the turns a narrow window follows, the one that tells the untold
               --  orders apart most), no less than the least turn a keyframe counts, and no climb past it (Together's
               --  Climb): doubled, the turns put the places four times as far apart. Where the fit names none, by the
               --  offset its own keyframes were followed to. K is the pair's place among the fit's, from 0.
               function Pair_Shares (P : Joint_Pair; K : Natural) return Real_Array is
                  Share : Real_Array (1 .. Size) := [others => 0.0];
                  Place : constant Natural := Turns.First_Index + 2 * K;   --  of the pair's first turn
                  Named : constant Boolean := Place + 1 <= Turns.Last_Index;
               begin
                  Share (P.Earlier) :=
                    Real'Max ((if Named then Turns (Place) else Reached (P.Earlier)), Least (P.Earlier));
                  Share (P.Later) :=
                    Real'Max ((if Named then Turns (Place + 1) else Reached (P.Later)), Least (P.Later));
                  return Share;
               end Pair_Shares;

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
                        & Driver.Log.Image (Least (C), 3)
                        & Ada.Strings.Unbounded.To_String (Why_Least (C)) & "; "
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
               --  Each push's first level is its own channel's, read after the pushes are drawn into their order: set
               --  by the place a push had before the draw, every push took the level of whichever channel had stood
               --  there (u5, servo seed 3 of the fake robot, 10-18: its channels' units differ fourfold, and the elbow,
               --  whose level was 28.4 units, was asked 56.4, another's; it turned 4.7 degrees where 2.5 were planned,
               --  the points its wrist eye saw moved 20.5 px on the mean and up to 75, and 106 of 255 left its picture).
               for I in 1 .. Count loop
                  Offset (I) := First (Order (I).Channel);
               end loop;
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
                     --  The levels followed beyond the widest each push had followed before the pass: what the pass
                     --  reached that no pass before it had. A level followed again where one was before tells nothing
                     --  new, and counted as a pass that followed something it kept the passes going for ever: u10 (servo
                     --  seed 3 of the fake robot, both arms, 10-19), arm 2, 26 passes from beat 9248 to 31294, each
                     --  re-taking levels its pushes had followed (c1 at 40 units, c3 at 79, c6 at 782) and losing the
                     --  ones beyond again, its offsets the same from pass 8 on, and the pair rounds that tell an order
                     --  never began; arm 1 of the same run ended its passes when one followed nothing at all.
                     New_In_Pass : Natural := 0;
                     Most_Before : constant Real_Array (1 .. 2 * Size) := Followed_Most;
                     --  The pass's end cells were taken in its round, before the round's estimate (End_Cells).
                     Ended_In_Round : Boolean := False;

                     --  A fitted arm ends the pass with the joints together at the
                     --  widest offsets followed (the rounds before the fit had their
                     --  own); its predictions go into these frames. Where the fit
                     --  names joints next to each other whose swap it cannot tell
                     --  (Contested), the pass ends turning those two, a pair at a time
                     --  (Pair_Shares), as the passes' end does, in place of every
                     --  joint at once: the fake's seed 26 (10-22) was fitted at 39
                     --  keyframes after its third pass, its order untold, and took its
                     --  cells of every joint and its single turns for 12663 beats and
                     --  four estimates more before its order was told, at 75.
                     procedure End_Cells is
                     begin
                        if not Told_Now and then not Contested.Is_Empty then
                           declare
                              Said : Ada.Strings.Unbounded.Unbounded_String;
                           begin
                              for K in Contested.First_Index .. Contested.Last_Index loop
                                 declare
                                    P     : constant Joint_Pair := Contested (K);
                                    Share : constant Real_Array := Pair_Shares (P, K - Contested.First_Index);
                                 begin
                                    Ada.Strings.Unbounded.Append
                                      (Said, " c" & Driver.Log.Image (P.Earlier) & " with c" & Driver.Log.Image (P.Later)
                                             & " by " & Driver.Log.Image (Share (P.Earlier), 3) & " and "
                                             & Driver.Log.Image (Share (P.Later), 3) & " units");
                                 end;
                              end loop;
                              Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " pass" & Passes'Image
                                               & " ends with its joints' order not told (" & Untold_Now'Image
                                               & " orders untold); its cells turn the pairs whose turns tell its untold"
                                               & " orders apart, a pair at a time:" & Ada.Strings.Unbounded.To_String (Said));
                           end;
                           for K in Contested.First_Index .. Contested.Last_Index loop
                              Together (Pair_Shares (Contested (K), K - Contested.First_Index), Climb => False);
                           end loop;
                           Driver.Beats.Within_A_Beat (Ask_For_Fit'Access);
                        else
                           Together (Shares, Climb => True);
                        end if;
                     end End_Cells;
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
                           Together (Shares, Climb => True);
                        elsif N > 1 and then not Fitted_Now then
                           Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " round" & Rounds'Image
                                            & ": no cell of several joints yet, the view's motion per unit of a joint is"
                                            & " unmeasured for some of them");
                        end if;
                        --  An arm fitted as the pass begins takes the pass's end cells in its round, by the fit it began
                        --  with, and the round's one estimate aligns and fits both: the cells' turns are named by that
                        --  fit either way, and taken after the round's estimate they waited on a second one. Panda seed
                        --  1a (round n25s2, 10-27): the estimates after its passes' end cells, passes 2 to 5, took 326 s
                        --  of the 715 its boot's estimates took, of which the kinematics' 571 (each a named fit).
                        if N > 1 and then Fitted_Now then
                           End_Cells;
                           Ended_In_Round := True;
                        end if;
                        Measure_The_Round;
                        Driver.Beats.Within_A_Beat (Read_Round'Access);
                        exit when Gone;
                        for I in 1 .. Count loop
                           if State (I) = Climbing then
                              declare
                                 P : Push renames Order (I);
                                 Lost  : Natural := 0;   --  the first level the aligner did not follow; 0 when none
                                 Last  : Level_Mark;
                                 Followed_Here : Boolean := False;   --  a level of the push was followed in this pass
                              begin
                                 for K in 1 .. Natural (Levels (I).Length) loop
                                    Last := Levels (I) (K);
                                    --  Followed as the aligner says, the first level too: a keyframe of a new direction is
                                    --  asked from the view's common motion and its answers on the keyframe's own field
                                    --  again narrowly (Keyframes.Add). The first level used to count as followed whenever
                                    --  the pass measured how far the view moves per unit of the joint, from any keyframe:
                                    --  f15i's fifth joint had its shift from the cells of the pass, its first level
                                    --  (310 units) found 0 of 242 points, and the ladder climbed on from it to 2508
                                    --  instead of halving to a level the aligner follows (10-15).
                                    if Last.Followed then
                                       Followed_In_Pass := Followed_In_Pass + 1;
                                       if Last.Offset > Most_Before (I) then
                                          New_In_Pass := New_In_Pass + 1;
                                       end if;
                                       Reached (P.Channel) := Real'Max (Reached (P.Channel), Last.Offset);
                                       Followed_Most (I) := Real'Max (Followed_Most (I), Last.Offset);
                                       Followed_Here := True;
                                       Descending (I) := False;
                                    elsif Last.Framed then
                                       --  A keyframe the aligner could not follow, too large: the ladder stops at
                                       --  this level; a fitted arm may predict it in another pass.
                                       Lost := K;
                                       exit;
                                    end if;
                                    --  No keyframe came of a level too small to count (the held pose was not apart
                                    --  from the keyframes before it by the step the fit counts): the ladder climbs
                                    --  past it.
                                 end loop;
                                 --  A channel the lock-in saw move the eye's whole view moves the eye: a level
                                 --  that left the view still lay within its gears' slack (the fake's seed 20,
                                 --  10-22: its base turned 0.10 units by its readings and its wrist eye's view not
                                 --  at all), and the ladder climbs on from it. Behind the eye is a joint whose
                                 --  whole view the lock-in never saw move.
                                 if Unseen_Now (P.Channel) and then not Whole_View (P.Channel) then
                                    --  The view does not move with the joint: a joint behind the eye (the kiwi's
                                    --  wrist roll, whose eye is mounted before it, R5 on 10-14: unmeasured, its
                                    --  ladder doubled a level a pass to 3016 units, every level followed by the
                                    --  jaws' points), and no level of it moves the view, however far.
                                    State (I) := Unseen;
                                 elsif Lost > 0 then
                                    if Reached (P.Channel) = 0.0 and then Offset (I) / 2.0 >= Least (P.Channel)
                                      and then not Tried_Up (I)
                                    then
                                       if Up_From (I) = 0.0 then
                                          Up_From (I) := 2.0 * Offset (I);
                                       end if;
                                       --  Not followed before any level of the joint was: the first level is sized
                                       --  by how far the view moves per unit of the joint, and that is wrong where
                                       --  the eye the arm is thought to carry is another's (an eye credited to the
                                       --  arm by a lag it could not tell yet: the rig of robot.boot.reread, whose
                                       --  first level stopped the sweep in its first round and left arm 2 unswept).
                                       --  The ladder goes down, halving, until a level is followed, and climbs from
                                       --  there, or until the least turn a keyframe counts.
                                       Offset (I) := Offset (I) / 2.0;
                                       Descending (I) := True;
                                    elsif Followed_Most (I) = 0.0 and then not Descending (I) and then Lost = 1
                                      and then not Levels (I) (Lost).Short
                                      and then (Per_Unit (P.Channel) = 0.0
                                                or else Levels (I) (Lost).Offset * Per_Unit (P.Channel) < Patch)
                                    then
                                       --  The push's first level, the least turn a keyframe counts, not followed, and no level
                                       --  of it followed before: the view moved by less than the aligner follows from no
                                       --  prediction, and a lesser turn is none a keyframe counts. The ladder climbs: twice the
                                       --  level, a pass at a time, until a level is followed or the joint meets something.
                                       --  While how far the view moves per unit of the joint is unmeasured (the first level is
                                       --  then the step the eye saw it move by, not one the aligner was asked to follow), or
                                       --  measured and the level moved the view by less than a patch, which the aligner follows
                                       --  by itself (Take's brake): a level the aligner lost there was too small to tell from
                                       --  none, not too wide to follow. Not at a level the joint was stopped short of.
                                       --  Stopped there, the joint had no keyframe of its own and its arm was never fitted:
                                       --  u12 (servo seed 3 of the fake robot, both arms, 10-19), arm 2's channel 6, its first
                                       --  level 43.1 units (the step the eye saw it move by), the view turned 1.7 degrees and
                                       --  moved 0 to 4 px by the truth, no common motion told, and the arm "swept in 4 passes
                                       --  of 4 rounds, not fitted"; u10, the same body, had 49.2 units for it and followed it.
                                       --  And u28 (the same body, 10-19): its first level 42.4 units, the view's motion measured
                                       --  by then from the keyframes of the other joints (0.136 px a unit, 5.8 px of a 16 px
                                       --  patch), the truth 2.8 and 4.5 px each way; at the finest size the common motion
                                       --  searches, 97 of 195 points stood and the other 98 drew two fields 49 to 24, none
                                       --  told; no point asked, and again "swept in 4 passes of 4 rounds, not fitted".
                                       Offset (I) := 2.0 * Levels (I) (Lost).Offset;
                                    else
                                       --  Followed below and lost here: the next pass takes the level halfway between,
                                       --  while half the gap is a step a keyframe counts. A level the aligner could not
                                       --  follow used to end the push until a fitted arm predicted it, and an arm whose
                                       --  wider levels the aligner never followed was never fitted (f15w on 10-15, arm 2,
                                       --  a servo arm over an oblique table: its channel 4 followed at 41.6 units, lost at
                                       --  83, and no keyframe of it lay between; the fit had one level of that joint).
                                       Lost_Least (I) := Real'Min (Lost_Least (I), Levels (I) (Lost).Offset);
                                       if Followed_Most (I) > 0.0
                                         and then Halving_Tells (I, P.Channel)
                                       then
                                          Offset (I) := (Followed_Most (I) + Lost_Least (I)) / 2.0;
                                       else
                                          Offset (I) := Levels (I) (Lost).Offset;
                                          State (I) := Not_Followed;
                                       end if;
                                    end if;
                                 elsif Levels (I).Is_Empty then
                                    State (I) := Blocked;
                                 elsif Last.Short then
                                    State (I) := Blocked;
                                 elsif Per_Unit (P.Channel) > 0.0 and then 2.0 * Last.Offset * Per_Unit (P.Channel) > Half then
                                    --  The doubling would pass the widest turn, the one that moves the view by half of
                                    --  it. A fitted arm's push takes that turn itself in the next pass, where it lies a
                                    --  counted step beyond the level taken and short of a level lost before: Take's ladder
                                    --  takes it in the pass only where the arm was fitted as the pass began, and a push
                                    --  that stopped before at its brake of two patches was left there (Stretch 2b's head,
                                    --  n9s2, 10-26: its tilt at 61 px of 120, the arm fitted within the pass).
                                    if Fitted_Now
                                      and then Half / Per_Unit (P.Channel) - Last.Offset > Least (P.Channel)
                                      and then Half / Per_Unit (P.Channel) < Lost_Least (I)
                                    then
                                       Offset (I) := Half / Per_Unit (P.Channel);
                                    else
                                       State (I) := Widest;
                                    end if;
                                 elsif Descending (I) then
                                    --  A ladder going down that reaches a level too small to count has no level the
                                    --  aligner follows below its first: once, it is tried above it.
                                    if Up_From (I) > 0.0 and then not Tried_Up (I) then
                                       Tried_Up (I) := True;
                                       Descending (I) := False;
                                       Offset (I) := Up_From (I);
                                    else
                                       State (I) := Not_Followed;
                                    end if;
                                 elsif Lost_Least (I) < Real'Last then
                                    --  A level halfway to one lost before, followed: halfway again, while half the gap
                                    --  is a step a keyframe counts. A halfway level that gave no keyframe (the joint did
                                    --  not move off the keyframes before it by a step the fit counts, or not at all: a
                                    --  servo's dead band) told nothing, and halving the same gap again asked it again
                                    --  for ever (robot.boot, 10-15: 530 passes of one level each, none followed).
                                    if Followed_Here
                                      and then Halving_Tells (I, P.Channel)
                                    then
                                       Offset (I) := (Followed_Most (I) + Lost_Least (I)) / 2.0;
                                    else
                                       Offset (I) := Lost_Least (I);
                                       State (I) := Not_Followed;
                                    end if;
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
                        --  A push still climbing says where its next level stands: t502 of the race (10-23) climbed
                        --  two pushes through 55 passes that followed nothing, and the log said only "climbing".
                        for I in 1 .. Count loop
                           Ada.Strings.Unbounded.Append
                             (Stops, " " & Ada.Characters.Handling.To_Lower (State (I)'Image));
                           if State (I) = Climbing then
                              Ada.Strings.Unbounded.Append
                                (Stops, " (c" & Driver.Log.Image (Order (I).Channel)
                                        & (if Order (I).Sign > 0.0 then "+" else "-") & " next at "
                                        & Driver.Log.Image (Offset (I), 4) & ", "
                                        & Driver.Log.Image (Natural (Levels (I).Length)) & " levels this pass, the last "
                                        & (if Levels (I).Is_Empty then "none"
                                           elsif Levels (I).Last_Element.Followed then "followed"
                                           elsif Levels (I).Last_Element.Framed then "framed, not followed"
                                           else "not framed")
                                        & ")");
                           end if;
                        end loop;
                        Driver.Log.Line
                          (Driver.Log.Robot, "boot: arm" & A'Image & " pass" & Passes'Image & " followed"
                           & Followed_In_Pass'Image & " keyframes," & New_In_Pass'Image & " beyond the levels followed before, in"
                           & Rounds'Image & " rounds so far; the joints"
                           & " are followed to offsets" & Image (Reached) & ", the pushes stopped by"
                           & Ada.Strings.Unbounded.To_String (Stops));
                     end;
                     --  The pass's end cells (End_Cells), where its round did not take them: the pass in which the
                     --  arm was first fitted.
                     if N > 1 and then New_In_Pass > 0 and then Fitted_Now and then not Ended_In_Round then
                        End_Cells;
                        Measure_The_Round;
                     end if;
                     Driver.Beats.Within_A_Beat (Read_Followed'Access);
                     exit when Gone
                       or else ((for all I in 1 .. Count => State (I) /= Climbing)
                       and then (New_In_Pass = 0 or else not Fitted_Now
                                 or else (for all I in 1 .. Count => State (I) /= Not_Followed)));
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
               --  A fitted arm whose sightings do not tell its joints' order takes its cells once more as its passes
               --  end, climbing as a fitted arm's cells do (Together): the order is told by what two joints turned
               --  together do that each alone does not, and a pass that followed nothing new took no cells at all
               --  (s06 on 10-16: fitted, its order not told, its last pass followed nothing, and its cells never
               --  climbed). Once: climbed as far as the view lets them, the cells give no more than they gave.
               --  When the fit names joints next to each other along its order whose swap it cannot tell (Contested), the
               --  cells turn those two, a pair at a time (Pair_Shares), in place of every joint at once; a pair once each:
               --  a fit after them may name others (the order it tells then may be another, with other neighbours), and
               --  those are turned in their round. Every joint at once, once, when it names none.
               --  A pair named again after its turn is turned again, at the share the fit names then, as long as the
               --  round before told some order from the fit's own (the untold orders fewer) or named other pairs (the
               --  pairs, not the turns it names them by, which move with every fit's lens and points): w7
               --  (servo seed 7 of the fake robot, 10-18) ended its sweep with two orders untold, the true one among
               --  them, the pair that told them apart most turned once, early, its keyframes with six points in view.
               if N > 1 and then Fitted_Now and then not Told_Now then
                  declare
                     All_Once  : Boolean := False;          --  every joint turned at once
                     Last_Named : Ada.Strings.Unbounded.Unbounded_String;
                     Last_Untold : Natural := Natural'Last;
                  begin
                     loop
                        declare
                           Said  : Ada.Strings.Unbounded.Unbounded_String;   --  the pairs and their turns, for the log
                           Named : Ada.Strings.Unbounded.Unbounded_String;   --  the pairs alone
                           use type Ada.Strings.Unbounded.Unbounded_String;
                        begin
                           for K in Contested.First_Index .. Contested.Last_Index loop
                              declare
                                 P     : constant Joint_Pair := Contested (K);
                                 Share : constant Real_Array := Pair_Shares (P, K - Contested.First_Index);
                              begin
                                 Ada.Strings.Unbounded.Append
                                   (Named, " c" & Driver.Log.Image (P.Earlier) & " with c" & Driver.Log.Image (P.Later));
                                 Ada.Strings.Unbounded.Append (Said, " c" & Driver.Log.Image (P.Earlier) & " with c"
                                                               & Driver.Log.Image (P.Later) & " by "
                                                               & Driver.Log.Image (Share (P.Earlier), 3) & " and "
                                                               & Driver.Log.Image (Share (P.Later), 3) & " units");
                              end;
                           end loop;
                           exit when Contested.Is_Empty and then All_Once;
                           exit when not Contested.Is_Empty and then Named = Last_Named and then Untold_Now >= Last_Untold;
                           Last_Named := Named;
                           Last_Untold := Untold_Now;
                           if Contested.Is_Empty then
                              Together (Shares, Climb => True);
                              All_Once := True;
                           else
                              Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & "'s joints' order is not told ("
                                               & Untold_Now'Image & " orders untold); its cells turn the pairs whose "
                                               & "turns tell its untold orders apart, a pair at a time:"
                                               & Ada.Strings.Unbounded.To_String (Said));
                              for K in Contested.First_Index .. Contested.Last_Index loop
                                 Together (Pair_Shares (Contested (K), K - Contested.First_Index), Climb => False);
                              end loop;
                           end if;
                        end;
                        Driver.Beats.Within_A_Beat (Ask_For_Fit'Access);
                        Measure_The_Round;
                        Driver.Beats.Within_A_Beat (Read_Followed'Access);
                        exit when not Fitted_Now or else Told_Now;
                     end loop;
                  end;
               end if;
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " swept in" & Passes'Image & " passes of"
                                & Rounds'Image & " rounds, "
                                & (if Gone then "no longer an arm of group" & G'Image & " in the graph: its sweep ends"
                                   elsif not Fitted_Now then "not fitted" elsif Told_Now then "fitted"
                                   else "fitted, its joints' order not told"));
               --  Its sweep ended, its terms' uncertainty across its joints joins its covariance at the next estimate.
               if Fitted_Now and then not Gone then
                  Driver.Robot.Kinematics.Weigh_Across_Joints (M, A);
               end if;
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
      Unrepeated  : Ada.Strings.Unbounded.Unbounded_String;   --  the channels whose noise did not repeat, for the log
      procedure Read_Rest is
         Index : Natural := 0;
         Total : Natural := 0;
         --  The least change the channel's readings showed from one beat to the next, nought where they never
         --  changed: one count of a reading that comes in counts (Differ, below).
         function Least_Change (G : Group_Id; C : Positive) return Real is
            Least : Real := Real'Last;
         begin
            for B in 1 .. M.Beats - 1 loop
               if Driver.Robot.Channels.Has_Reading (M, G, B) and then Driver.Robot.Channels.Has_Reading (M, G, B - 1) then
                  declare
                     D : constant Real :=
                       abs (Driver.Robot.Channels.Reading (M, G, B, C) - Driver.Robot.Channels.Reading (M, G, B - 1, C));
                  begin
                     if D > 0.0 and then D < Least then
                        Least := D;
                     end if;
                  end;
               end if;
            end loop;
            return (if Least = Real'Last then 0.0 else Least);
         end Least_Change;

         --  Whether two measures of a channel's rest noise differ: Then_N over the rest before its last doubling,
         --  Now_N over the whole of it on Dof degrees of freedom, of a reading whose least change was Least. A counted
         --  reading's noise is a median of whole counts (Channels.Measure_Channel: the median absolute change at
         --  rest, as a sigma, over the root of two), on whole and half counts (the median of an even number of them),
         --  and where the reading's jitter puts that median between two counts it falls on either by chance, however
         --  long the rest: the Shadow hand's channels came out at a median of one count and of two by turns as the
         --  rest doubled (round 4a, 10-23: 8.04E-04 and 1.61E-03 of a count of 7.67E-04, 4.19E-05 and 8.39E-05 of
         --  4.00E-05), and its rest held 940 to 1080 of its 1200 beats. Two noises whose medians are a count apart or
         --  less are the same noise; farther apart, they differ where the difference is significant.
         function Differ (Now_N, Then_N, Least : Real; Dof : Natural) return Boolean is
            Count_Noise : constant Real :=
              Driver.Stats.Sigma_Of_Deviation (Least) / Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0);
         begin
            if Count_Noise > 0.0 and then Real'Rounding (2.0 * abs (Now_N - Then_N) / Count_Noise) <= 2.0 then
               return False;
            end if;
            return (if Dof = 0 then Now_N /= Then_N
                    else Driver.Uncertain.Significant
                           (Now_N - Then_N, Now_N / Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0 * Real (Dof)), Dof));
         end Differ;
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
            Unrepeated := Ada.Strings.Unbounded.Null_Unbounded_String;
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
                        if Then_N = Real'Last or else Differ (Now_N, Then_N, Least_Change (Group_Id (G), C), Dof) then
                           Rest_Stable := False;
                           if Then_N < Real'Last then
                              Ada.Strings.Unbounded.Append
                                (Unrepeated, " group" & G'Image & " channel" & C'Image & " (" & Scientific (Then_N)
                                 & " then, " & Scientific (Now_N) & " now, on" & Dof'Image & " degrees, the least change "
                                 & Scientific (Least_Change (Group_Id (G), C)) & ")");
                           end if;
                        end if;
                        Rest_Noises (Index) := Now_N;
                     end;
                  end if;
               end loop;
            end loop;
            Rest_Beats := M.Beats;
         end if;
      end Read_Rest;

      --  Every commandable channel's command as the body stands still at rest, before any push: the command that
      --  holds it still (Motion.Hold_Of holds a channel that keeps the body going there).
      procedure Note_Rest is
      begin
         Note_Rest_Commands (M);
      end Note_Rest;

      --  The groups as the sweeps left them, each eye's mount among them: the mounts step 1 could not tell are told by
      --  the arms' fits (owner 10-24), and the boot is scored on the groups after the sweeps where it sweeps.
      procedure Tell_Swept is
      begin
         Driver.Log.Line (Driver.Log.Robot, "boot: the groups after sweeping the arms:" & ASCII.LF & Describe (M));
      end Tell_Swept;

      procedure Read_Body is
      begin
         --  The turn at which the recognition's probes saw each channel move an eye goes to the model
         --  (Group_Stream.Seen_At), where the sweeps' first levels and the body file read it; a boot begun from a
         --  body file recognized nothing and keeps the file's. In the beat the body is read in: a beat of its own
         --  after every probe moved every lock-in row after it (robot.boot.reread's lagging eye then saw nothing).
         --  The least turn an eye saw it move at, the lock-in's confirmation or not: a sweep's first level needs
         --  the view to move, and the ladder doubles from there if it does not; the confirmed turn floored it at
         --  1254 units of a joint an eye had seen at 39 (s09 on 10-16, a servo's elbow: its arm's own eye had no
         --  row of the push the lock-in could use, every level up to the eighth unconfirmed), and the sweep's first
         --  push of it turned it 1.9 rad, farther than any keyframe was followed.
         for R of Recognized loop
            while M.Groups (R.Ref.Group).Seen_At.Last_Index < R.Ref.Channel - 1 loop
               M.Groups (R.Ref.Group).Seen_At.Append (0.0);
            end loop;
            M.Groups (R.Ref.Group).Seen_At (R.Ref.Channel - 1) :=
              Real'Max (M.Groups (R.Ref.Group).Seen_At (R.Ref.Channel - 1),
                        (if R.First > 0.0 then R.First else R.Amount));
         end loop;
         Driver.Log.Line (Driver.Log.Robot, "boot: the groups as recognized:" & ASCII.LF & Describe (M));
         --  Each group in each eye, for the study of what tells an eye's own motion from a part's in its view (round 3,
         --  10-23: the Panda's fixed eye called moved whole by its arm, grippers whose fingers fill the view called arms):
         --  the verdict, the cells that respond, and how far the cells that did not respond moved along the push together.
         for G in 1 .. Group_Count (M) loop
            if Is_Commandable (M, Group_Id (G)) then
               for E in 1 .. Eye_Count (M) loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " eye" & E'Image & ": "
                                   & Response (M, Group_Id (G), Eye_Id (E))'Image & ","
                                   & Responding (M, Group_Id (G), Eye_Id (E))'Image & " of"
                                   & Textured_Cells (M, Group_Id (G), Eye_Id (E))'Image
                                   & " cells respond, the rest pulled along the push by "
                                   & Driver.Log.Image (Rest_Pull (M, Group_Id (G), Eye_Id (E)), 2) & " sigmas, moving by "
                                   & Driver.Log.Image (Resting_Motion (M, Group_Id (G), Eye_Id (E)), 2));
               end loop;
            end if;
         end loop;
         Lockin.Dump_Cells (M, Ada.Strings.Fixed.Trim (M.Beats'Image, Ada.Strings.Left));
         for G in 1 .. Group_Count (M) loop
            if Is_Commandable (M, Group_Id (G)) and then Contract_Breach (M, Group_Id (G)) > 0 then
               Breach := True;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " breaks clause"
                                & Contract_Breach (M, Group_Id (G))'Image & " of the porting contract");
            end if;
         end loop;
         --  The groups as recognized stand on the recognition's pushes alone: an arm's sweep does not move them again
         --  (Driver.Robot.Note_Recognized).
         Driver.Robot.Note_Recognized (M);
      end Read_Body;

      --  Every arm of the current estimate is swept with the eye it carries,
      --  one at a time (below): the body is read again before each round of
      --  sweeps and after the fit's estimate, since later evidence can change a role or a
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
         end Find_Unswept;
      begin
         loop
            Driver.Beats.Within_A_Beat (Find_Unswept'Access);
            if not Unswept.Is_Empty then
               for P of Unswept loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: sweeping arm" & P.Arm'Image & " (group" & P.Group'Image
                                   & ") with eye" & P.Eye'Image
                                   & (if Unswept.Length = 1 then "" else ", the" & Unswept.Length'Image
                                      & " arms one at a time"));
               end loop;
               --  One at a time, always. A keyframe needs the eye's picture still, and an arm moving through it keeps it
               --  from ever being. The arms swept at once, when the lock-in showed no eye the other's response, followed
               --  keyframes in the first pass (one arm of each boot in the second) and none in the 4 to 8 passes after, in
               --  all three table boots of 10-20 (t20, t21, t22: the pushes of hundreds of units entered the other's view,
               --  which the recognition's small pushes had not), and R6's eighth start (10-14) before them; swept one at
               --  a time the same seeds followed 28 to 30 keyframes in pass 2 (r20, r21). The recognition cannot tell what
               --  an arm will show an eye at the widest turn it will make.
               for P of Unswept loop
                  Sweep (P.Arm);
               end loop;
               for P of Unswept loop
                  Swept.Append (P);
               end loop;
            else
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
            --  Which channels' noise did not repeat as the rest doubled: the Shadow hand held still here for 1000 to
            --  1200 beats with no channel drifting (round 1, 10-23).
            if Ada.Strings.Unbounded.Length (Unrepeated) > 0 then
               Driver.Log.Line (Driver.Log.Robot, "boot: at rest, the noise did not repeat at" & M.Beats'Image & " beats:"
                                & Ada.Strings.Unbounded.To_String (Unrepeated) & "; holding still on");
               Unrepeated := Ada.Strings.Unbounded.Null_Unbounded_String;
            end if;
            if Ada.Strings.Unbounded."/=" (Drifting, Last) then
               Driver.Log.Line (Driver.Log.Robot, "boot: still moving at rest:"
                                & Ada.Strings.Unbounded.To_String (Drifting) & "; holding still on");
               Last := Drifting;
            end if;
            Driver.Robot.Motion.Settle (M, Waited);
         end loop;
      end;
      Driver.Beats.Within_A_Beat (Note_Rest'Access);
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
                     declare
                        Groups_Commanded : Natural := 0;
                        --  Each group's own ladder from its own least step, as many times over as every channel together
                        --  was taken to be seen (Motion.Probe_Report.Factor): each group's channels moved together so, from
                        --  the least step its own readings tell (Motion.Probe_Together).
                        From_Of : Real_Array (1 .. Count) := [others => P.Excursion];
                        procedure Read_Least is
                        begin
                           for G in 1 .. Count loop
                              if Commandable (G) and then Sizes (G) > 0 then
                                 declare
                                    Least : constant Real := Driver.Robot.Motion.Group_Least_Step (M, Group_Id (G));
                                 begin
                                    if Least > 0.0 then
                                       From_Of (G) := Least * P.Factor;
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end Read_Least;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Least'Access);
                        for G in 1 .. Count loop
                           if Commandable (G) and then Sizes (G) > 0 then
                              Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " is pushed whole from "
                                               & Scientific (From_Of (G)) & " reading units, its least step"
                                               & " taken as many times over as every channel together was");
                           end if;
                        end loop;
                        for G in 1 .. Count loop
                           if Commandable (G) and then Sizes (G) > 0 then
                              Groups_Commanded := Groups_Commanded + 1;
                           end if;
                        end loop;
                        for G in 1 .. Count loop
                           if Commandable (G) and then Sizes (G) > 0 then
                              Recognize_Group (Group_Id (G), Sizes (G), From_Of (G), Common => P.Excursion,
                                               Alone => Groups_Commanded = 1);
                           end if;
                        end loop;
                     end;
                     Driver.Beats.Within_A_Beat (Estimate'Access);
                     --  A group recognized whole that moves two eyes or more whole and keeps no body going may carry
                     --  them on channels of its own each (one group for two arms): probed again one channel at a time,
                     --  for the graph to part it (Graph.Derive, channels that move different eyes).
                     for G in 1 .. Count loop
                        if Commandable (G) and then Sizes (G) > 1 and then Whole_Amount (Group_Id (G)) > 0.0 then
                           declare
                              Whole_Eyes : Natural := 0;
                              Speed      : Boolean := False;
                              Alike      : Boolean := True;   --  no two of those eyes moved by shares apart, nor one apart
                                                            --  from the eye moved most
                              Amount     : constant Real := Whole_Amount (Group_Id (G));
                              --  Two eyes' shares apart by more than chance (Regression.Shares_Apart): the shares the
                              --  group's pushes left moved beyond the flow's reach (Motion.Note_Stayed), as the graph tells
                              --  an eye's own group (Graph.Best_Of): a view the group carries moves whole by its ego-motion,
                              --  one it is seen in moves where the group is. By the shares of cells responding instead, the
                              --  lock-in had RB-Y1's arm move its wrist eye and its head camera whole alike, the robot
                              --  swaying as the arm moved, and it was parted, 500 beats, where its pushes left the two 52
                              --  and 1 of 255 cells beyond reach (seed 1, paced round p84a, 10-23).
                              --  The cells the group's pushes left moved beyond the flow's reach in the eye, and the cells
                              --  measured, summed over them.
                              function Far_Of (E : Positive) return Natural is
                                (if Natural (G) <= Natural (M.Eyes (Eye_Id (E)).Stayed_Far_Cells.Length)
                                 then M.Eyes (Eye_Id (E)).Stayed_Far_Cells (Natural (G) - 1) else 0);
                              function Stayed_Of (E : Positive) return Natural is
                                (if Natural (G) <= Natural (M.Eyes (Eye_Id (E)).Stayed_Cells.Length)
                                 then M.Eyes (Eye_Id (E)).Stayed_Cells (Natural (G) - 1) else 0);
                              --  Each of those eyes is alike with the eye the group's pushes left most beyond reach, whole
                              --  or not as the lock-in has it yet: two arms in one group move each arm's eye as far, where
                              --  a group that carries one eye and moves the others whole by a sway or a part in view moves
                              --  its own eye furthest (RB-Y1's right arm, seed 2, paced round p89b, 10-24: whole in the left
                              --  wrist's eye, the robot swaying, and in the head's, the arm in its view, 5 of 222 and 21 of
                              --  255 cells left beyond reach, not apart; its own wrist's eye, undecided at that estimate,
                              --  58 of 246; probed one channel at a time, 900 beats, to the boot's cap).
                              --  How many eyes a group moves whole, as the lock-in has it.
                              function Wholes (H : Positive) return Natural is
                                 Moved : Natural := 0;
                              begin
                                 for E in 1 .. Eye_Count (M) loop
                                    if Response (M, Group_Id (H), Eye_Id (E)) = Whole then
                                       Moved := Moved + 1;
                                    end if;
                                 end loop;
                                 return Moved;
                              end Wholes;
                              procedure Read_Whole is
                                 Most : Natural := 0;   --  the eye its pushes left most beyond reach
                              begin
                                 Speed := Keeps_Going (M, Group_Id (G));
                                 for E in 1 .. Eye_Count (M) loop
                                    if Stayed_Of (E) > 0
                                      and then (Most = 0
                                                or else Real (Far_Of (E)) / Real (Stayed_Of (E))
                                                        > Real (Far_Of (Most)) / Real (Stayed_Of (Most)))
                                    then
                                       Most := E;
                                    end if;
                                 end loop;
                                 for E in 1 .. Eye_Count (M) loop
                                    if Response (M, Group_Id (G), Eye_Id (E)) = Whole then
                                       Whole_Eyes := Whole_Eyes + 1;
                                       if Most > 0 and then E /= Most and then Stayed_Of (E) > 0
                                         and then Driver.Robot.Regression.Shares_Apart
                                                    (Far_Of (E), Stayed_Of (E), Far_Of (Most), Stayed_Of (Most))
                                       then
                                          Alike := False;
                                       end if;
                                       for F in 1 .. E - 1 loop
                                          if Response (M, Group_Id (G), Eye_Id (F)) = Whole
                                            and then Stayed_Of (E) > 0 and then Stayed_Of (F) > 0
                                            and then Driver.Robot.Regression.Shares_Apart (Far_Of (E), Stayed_Of (E), Far_Of (F), Stayed_Of (F))
                                          then
                                             Alike := False;
                                          end if;
                                       end loop;
                                    end if;
                                 end loop;
                                 --  Alike only where the eye moved most has cells beyond reach to be alike in, more than
                                 --  none by more than chance: a push that left a cell or two beyond reach anywhere, a
                                 --  view moved by fractions of a pixel, leaves no two eyes apart by chance and tells
                                 --  nothing of them. RB-Y1's left arm, its pushes leaving 2 of its wrist eye's 243
                                 --  cells beyond reach and none of the others', was probed one channel at a time, 300
                                 --  beats, and the boot ran out of them (seed 1, paced round p133a, 10-24).
                                 if Most = 0 or else Stayed_Of (Most) = 0
                                   or else not Driver.Robot.Regression.Shares_Apart
                                                 (Far_Of (Most), Stayed_Of (Most), 0, Stayed_Of (Most))
                                 then
                                    Alike := False;
                                 end if;
                                 --  And so of every eye it moves whole: two arms in one group carry each arm's eye beyond reach
                                 --  alike, where an arm or a hand that sways the body, or moves in another eye's view, leaves
                                 --  that eye's view within reach. The G1's right hand moved its wrist eye and the head camera
                                 --  whole, 4 to 7 of the head's 255 cells beyond reach, and was probed one finger at a time,
                                 --  925 beats, to the cap; RB-Y1's left arm its wrist eye and the head's, 5 of 255 beyond reach
                                 --  in the head's, 924 beats (seed 1 of each, paced round p160b, 10-24).
                                 for E in 1 .. Eye_Count (M) loop
                                    if Response (M, Group_Id (G), Eye_Id (E)) = Whole
                                      and then (Stayed_Of (E) = 0
                                                or else not Driver.Robot.Regression.Shares_Apart
                                                              (Far_Of (E), Stayed_Of (E), 0, Stayed_Of (E)))
                                    then
                                       Alike := False;
                                    end if;
                                 end loop;
                                 --  Nor where each eye it moves whole a group moving fewer eyes whole moves whole as well: the
                                 --  group is under those (a torso under the arms and the head that carry its eyes each), and
                                 --  not two of them. RB-Y1's torso moved its three eyes whole alike, the arms their wrists'
                                 --  and the head its own, and was probed one channel at a time, 330 beats, out of beats
                                 --  (seed 3, paced round p148a, 10-24).
                                 if (for all E in 1 .. Eye_Count (M) =>
                                       Response (M, Group_Id (G), Eye_Id (E)) /= Whole
                                       or else (for some H in 1 .. Count =>
                                                  H /= G and then Commandable (H)
                                                  and then Response (M, Group_Id (H), Eye_Id (E)) = Whole
                                                  and then Wholes (H) < Wholes (G)))
                                 then
                                    Alike := False;
                                 end if;
                              end Read_Whole;
                           begin
                              Driver.Beats.Within_A_Beat (Read_Whole'Access);
                              --  Only alike: one group for two arms moves each arm's eye whole alike, and a group
                              --  that moves one eye's view significantly more than another's carries the one and
                              --  sweeps a part across the other (the Panda's arm, 246 and 204 of 255 cells in its
                              --  wrist's eye and its base camera's, its shadow and its reflection on the floor
                              --  sweeping the base camera's view; probed one channel at a time, round 4d, 10-23,
                              --  four of six boots out of beats).
                              if not Speed and then Whole_Eyes >= 2 and then not Alike then
                                 Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " moves" & Whole_Eyes'Image
                                                  & " eyes whole, by shares apart: not probed one channel at a time");
                              end if;
                              if not Speed and then Whole_Eyes >= 2 and then Alike then
                                 Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " moves" & Whole_Eyes'Image
                                                  & " eyes whole: its channels are probed one at a time");
                                 Set_Whole_Amount (Group_Id (G), 0.0);
                                 for I in reverse Recognized.First_Index .. Recognized.Last_Index loop
                                    if Recognized (I).Ref.Group = Group_Id (G) then
                                       Recognized.Delete (I);
                                    end if;
                                 end loop;
                                 Recognize (Group_Id (G), Sizes (G), Amount);
                              end if;
                           end;
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
                                       if Whole_Amount (Group_Id (G)) > 0.0 then
                                          Push_Group (Group_Id (G), Sizes (G), Factor (G) * Whole_Amount (Group_Id (G)), Moved,
                                                      Pause (G));
                                       else
                                          Push_Both_Ways (Group_Id (G), Sizes (G), Factor (G), Moved, Only, Pause (G));
                                       end if;
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
            Driver.Beats.Within_A_Beat (Tell_Swept'Access);
            Keep ("sweeping the arms");
         end if;
         --  Step 3: every fixed eye that sees a fitted arm and has no place yet is placed by the arms' edge flow
         --  (Driver.Robot.Flows), all the arms it sees through its one lens; one judged before is not captured again.
         Driver.Robot.Flows.Place (M);
         Keep ("placing the fixed eyes");
      end;
      Driver.Robot.Hand.Measure (H, M);
      declare
         procedure Store is
         begin
            Save (M, H, Body_File);
            --  The porting contract is judged on the body as measured at the end, not as first recognized: f16f's
            --  jaw (10-15), whose pushes no eye's lock-in told apart while the groups were recognized, broke clause 1
            --  then and was arm 1's closer at the end, and the boot failed on the first verdict; the brain was never
            --  asked.
            Breach := False;
            for G in 1 .. Group_Count (M) loop
               if Is_Commandable (M, Group_Id (G)) and then Contract_Breach (M, Group_Id (G)) > 0 then
                  Breach := True;
                  Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " breaks clause"
                                   & Contract_Breach (M, Group_Id (G))'Image & " of the porting contract as measured");
               end if;
            end loop;
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
