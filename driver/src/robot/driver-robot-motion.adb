with Ada.Numerics.Long_Elementary_Functions;
with Driver.Beats;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Clock;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Steps;
with Driver.Robot.Lockin;
with Driver.Robot.Kinematics;
with Driver.Uncertain;

package body Driver.Robot.Motion is

   use Ada.Numerics.Long_Elementary_Functions;


   type Count_Array is array (Positive range <>) of Natural;

   procedure Settle (M : in out Model; Beats_Waited : out Natural) is
      B : Driver.Clock.Beat;
   begin
      Beats_Waited := 0;
      loop
         Driver.Beats.Next (B);
         declare
            Done : constant Boolean := Still (M);
         begin
            Driver.Beats.Send (Driver.Commands.Hold);
            exit when Done;
         end;
         Beats_Waited := Beats_Waited + 1;
      end loop;
   end Settle;

   procedure Judge
     (M       : Model;
      Targets : Driver.Commands.Command;
      Before  : Count_Array;
      Waited  : Natural;
      Report  : out Step_Report)
   is
      Least : Estimate := (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0);
      Moved_Some : Boolean := True;
      Blocked_Any : Boolean := False;
   begin
      Report := (others => <>);
      Report.Beats := Waited;
      for G in Before'Range loop
         if Is_Commandable (M, Group_Id (G)) and then Driver.Commands.Has_Target (Targets, Group_Id (G))
           and then Steps.Episodes (M, Group_Id (G)) > Before (G)
         then
            declare
               E : constant Episode := Steps.Latest (M, Group_Id (G));
            begin
               Append (Report.Detail, "group" & G'Image & ": delivered "
                       & Driver.Log.Image (E.Delivered.Value, 3) & " of " & Driver.Log.Image (E.Length, 4)
                       & (if E.Blocked then ", blocked" else "") & "; ");
               if E.Delivered.Value < Least.Value then
                  Least := E.Delivered;
               end if;
               if E.Blocked then
                  Blocked_Any := True;
                  --  Pushing did not move it at all along the ask.
                  Moved_Some := Moved_Some
                    and then E.Delivered.Value > 0.0
                    and then Driver.Uncertain.Significant (E.Delivered.Value, E.Delivered.Sigma,
                                                           E.Delivered.Degrees_Of_Freedom);
               end if;
            end;
         end if;
      end loop;
      Report.Delivered := Least;
      Report.Outcome := (if not Blocked_Any then Reached elsif Moved_Some then Short else Blocked);
   end Judge;

   procedure Step (M : in out Model; Targets : Driver.Commands.Command; Report : out Step_Report) is
      B : Driver.Clock.Beat;
   begin
      Report := (others => <>);
      --  The models are read only between Next and Send.
      Driver.Beats.Next (B);
      declare
         Groups : constant Natural := Group_Count (M);
         --  How many pushes each group had before this step; a push of the
         --  step is the one after them.
         Before : Count_Array (1 .. Groups) := [others => 0];
         Waited : Natural := 0;
         Started : constant Natural := M.Beats;

         function Targeted (G : Group_Id) return Boolean is
           (Is_Commandable (M, G) and then Driver.Commands.Has_Target (Targets, G));

         --  Every push this step started is over.
         function Done return Boolean is
         begin
            for G in 1 .. Groups loop
               if Targeted (Group_Id (G)) and then Steps.Episodes (M, Group_Id (G)) > Before (G)
                 and then not Steps.Latest (M, Group_Id (G)).Ended
               then
                  return False;
               end if;
            end loop;
            return True;
         end Done;
      begin
         for G in 1 .. Groups loop
            Before (G) := Steps.Episodes (M, Group_Id (G));
         end loop;
         Driver.Beats.Send (Targets);
         --  The push starts at the beat whose observation was taken under the
         --  new targets; it is over when the step tracker has seen it end.
         loop
            Driver.Beats.Next (B);
            Waited := Waited + 1;
            declare
               Finished : constant Boolean := Done;
            begin
               if Finished then
                  Judge (M, Targets, Before, Waited, Report);
                  Report.Started := Started;
               end if;
               Driver.Beats.Send (Driver.Commands.Hold);
               exit when Finished;
            end;
         end loop;
      end;
   end Step;

   procedure Hold_While_Matching (M : in out Model) is
      B    : Driver.Clock.Beat;
      Done : Boolean;
   begin
      loop
         Driver.Beats.Next (B);
         Done := Kinematics.Pending (M) = 0;
         Driver.Beats.Send (Driver.Commands.Hold);
         exit when Done;
      end loop;
   end Hold_While_Matching;

   procedure Hold (M : in out Model; Beats : Positive) is
      pragma Unreferenced (M);
      B : Driver.Clock.Beat;
   begin
      for K in 1 .. Beats loop
         Driver.Beats.Next (B);
         Driver.Beats.Send (Driver.Commands.Hold);
      end loop;
   end Hold;

   function Hold_Of (M : Model; G : Group_Id; Channel : Positive) return Real is
      Last : constant Natural := (if M.Beats > 0 then M.Beats - 1 else 0);
   begin
      return (if Channels.Has_Target (M, G, Last) then Channels.Target (M, G, Last, Channel)
              else Channels.Reading (M, G, Last, Channel));
   end Hold_Of;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   --  The probes may try as many levels as a float has bits of precision.
   Levels : constant Positive := Real'Machine_Mantissa;

   function Tail return Real is (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));

   --  The chance that one verdict of the eye is a false alarm: a count at
   --  rest beats all n counts it had at rest with chance at most 1 / (n + 1);
   --  without those, the per-cell test's own level.
   function False_Alarm (M : Model; E : Eye_Id) return Real is
     (if M.Eyes (E).Rest_Counts_Known then 1.0 / Real (M.Eyes (E).Rest_Count_Beats + 1) else Tail);

   --  How many seen moves in a row make a chance run rarer than Z's tail over
   --  every level a probe may try, when one move is a false alarm with that
   --  chance; none can when the chance is not below one.
   function Run_Needed (Chance : Real) return Natural is
   begin
      if Chance <= 0.0 then
         return 1;
      elsif Chance >= 1.0 then
         return Natural'Last;
      end if;
      declare
         K : constant Real := Real'Ceiling (Ln (Tail / Real (Levels)) / Ln (Chance));
      begin
         return (if K >= Real (Natural'Last) then Natural'Last else Natural'Max (1, Natural (K)));
      end;
   end Run_Needed;

   --  Some eye's image moved at a beat from From on, and the chance that a
   --  look at that many verdicts finds one by noise alone.
   procedure Look (M : Model; From : Natural; Seen : out Boolean; Chance : out Real) is
   begin
      Seen := False;
      Chance := 0.0;
      for E in 1 .. Eye_Count (M) loop
         for B in From .. M.Beats - 1 loop
            Chance := Chance + False_Alarm (M, Eye_Id (E));
            if Lockin.Moved (M, Eye_Id (E), B) then
               Seen := True;
            end if;
         end loop;
      end loop;
   end Look;

   --  The smallest change of the channel's reading that tells from its noise
   --  (a change is the difference of two readings); without jitter, the
   --  resolution of the group's largest reading, or of one reading unit when
   --  every reading is zero.
   function Smallest_Step (M : Model; G : Group_Id; Channel : Positive) return Real is
      Sigma : constant Real := Reading_Noise (M, G, Channel);
      Scale : Real := 0.0;
   begin
      if Sigma > 0.0 and then Sigma < Real'Last then
         return Driver.Uncertain.Threshold
           (Driver.Uncertain.Scalar_Gate (Channels.Noise_Freedom (M, G, Channel))) * Sigma * Sqrt (2.0);
      end if;
      for C in 1 .. Group_Size (M, G) loop
         Scale := Real'Max (Scale, abs Channels.Reading (M, G, M.Beats - 1, C));
      end loop;
      return Real'Model_Epsilon * (if Scale > 0.0 then Scale else 1.0);
   end Smallest_Step;

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report)
   is
      package Channel_Streams renames Driver.Robot.Channels;
      type Flag_Array is array (Positive range <>) of Boolean;
      B       : Driver.Clock.Beat;
      Sign    : constant Real := (if Direction > 0.0 then 1.0 else -1.0);
      N       : constant Natural := Channels'Length;
      Listed  : constant Channel_Refs (1 .. N) := Channels;
      Usable  : Flag_Array (1 .. N) := [others => False];
      Base    : Driver.Commands.Command;            --  every listed group at its hold
      Start   : Real_Array (1 .. N) := [others => 0.0];   --  each reading when the probe began
      Noise   : Real_Array (1 .. N) := [others => 0.0];
      Freedom : Count_Array (1 .. N) := [others => 0];
      Following : Flag_Array (1 .. N) := [others => False];
      Delivered : Flag_Array (1 .. N) := [others => False];   --  some level moved it along the ask
      Best, Best_Sigma : Real_Array (1 .. N) := [others => 0.0];   --  the best fraction a level delivered
      Kept    : Real_Array (1 .. N) := [others => 0.0];   --  the offset where it last followed
      Amount  : Real := First;
      Unlooked : Natural := 0;   --  the first beat no look has judged: the looks tile the probe, so
                                 --  a response that shows later than expected falls in the next one

      --  Moves the listed channels to their holds plus the offsets and looks
      --  for as long as a response takes to show in the readings and then in
      --  the images: whether some eye saw motion, the chance that a look at
      --  so many verdicts finds one by noise, and each reading at the end.
      procedure Move_And_Look
        (Offsets : Real_Array; Seen : out Boolean; Chance : out Real;
         Now : out Real_Array; Moving : out Flag_Array)
      is
         C      : Driver.Commands.Command := Base;
         From   : Natural;
         Looked : Natural := 0;
      begin
         for I in 1 .. N loop
            if Usable (I) then
               declare
                  G : constant Group_Id := Listed (I).Group;
                  T : Real_Array := Driver.Commands.Target (C, G);
                  K : constant Positive := T'First + Listed (I).Channel - 1;
               begin
                  T (K) := Driver.Commands.Target (Base, G) (K) + Offsets (I);
                  Driver.Commands.Set_Target (C, G, T);
               end;
            end if;
         end loop;
         Driver.Beats.Next (B);
         From := Unlooked;
         Driver.Beats.Send (C);
         loop
            Driver.Beats.Next (B);
            Looked := Looked + 1;
            declare
               Wait : Natural := 1;
               Delay_Beats : Natural := 0;
               Lag : Natural := 0;
               Done : Boolean;
            begin
               for I in 1 .. N loop
                  if Usable (I) and then M.Groups (Listed (I).Group).Delay_Known then
                     Delay_Beats := Natural'Max (Delay_Beats, M.Groups (Listed (I).Group).Delay_Beats);
                  end if;
               end loop;
               for E in 1 .. Eye_Count (M) loop
                  Lag := Natural'Max (Lag, Natural'Max (0, Image_Lag (M, Eye_Id (E))));
               end loop;
               Wait := Wait + Delay_Beats + Lag;
               Done := Looked >= Wait;
               if Done then
                  Look (M, From, Seen, Chance);
                  Unlooked := M.Beats;
                  for I in 1 .. N loop
                     Now (I) := 0.0;
                     Moving (I) := False;
                     if Usable (I) and then Channel_Streams.Has_Reading (M, Listed (I).Group, M.Beats - 1) then
                        Now (I) := Channel_Streams.Reading (M, Listed (I).Group, M.Beats - 1, Listed (I).Channel);
                        Moving (I) := Channel_Streams.Moving (M, Listed (I).Group, M.Beats - 1);
                     end if;
                  end loop;
               end if;
               Driver.Beats.Send (Driver.Commands.Hold);
               exit when Done;
            end;
         end loop;
      end Move_And_Look;

   begin
      Report := (others => <>);
      --  Where every listed channel is held and reads, and the first step,
      --  read in a held beat.
      Driver.Beats.Next (B);
      Unlooked := M.Beats;
      if M.Beats > 0 then
         for I in Listed'Range loop
            declare
               G : constant Group_Id := Listed (I).Group;
               C : constant Positive := Listed (I).Channel;
            begin
               if Natural (G) <= Group_Count (M) and then Is_Commandable (M, G) and then C <= Group_Size (M, G)
                 and then Channel_Streams.Has_Reading (M, G, M.Beats - 1)
               then
                  Usable (I) := True;
                  Following (I) := True;
                  Start (I) := Channel_Streams.Reading (M, G, M.Beats - 1, C);
                  Noise (I) := Reading_Noise (M, G, C);
                  Freedom (I) := Channel_Streams.Noise_Freedom (M, G, C);
                  if not Driver.Commands.Has_Target (Base, G) then
                     declare
                        H : Real_Array (1 .. Group_Size (M, G));
                     begin
                        for K in H'Range loop
                           H (K) := Hold_Of (M, G, K);
                        end loop;
                        Driver.Commands.Set_Target (Base, G, H);
                     end;
                  end if;
                  if First = 0.0 then
                     --  The smallest step every listed reading can tell.
                     Amount := Real'Max (Amount, Smallest_Step (M, G, C));
                  end if;
               end if;
            end;
         end loop;
      end if;
      Driver.Beats.Send (Driver.Commands.Hold);
      if Amount <= 0.0 or else (for all U of Usable => not U) then
         return;
      end if;
      for Level in 1 .. Levels loop
         Report.Steps := Level;
         declare
            Offsets : Real_Array (1 .. N);
            Seen    : Boolean;
            Chance  : Real;
            Now     : Real_Array (1 .. N);
            Moving  : Flag_Array (1 .. N);
         begin
            for I in 1 .. N loop
               Offsets (I) := (if Following (I) then Sign * Amount else Kept (I));
            end loop;
            Move_And_Look (Offsets, Seen, Chance, Now, Moving);
            --  Which channels still follow: the fraction of the offset each
            --  delivered, against the best a smaller offset delivered.
            for I in 1 .. N loop
               if Usable (I) and then Following (I) then
                  declare
                     F : constant Real := Sign * (Now (I) - Start (I)) / Amount;
                     S : constant Real := Noise (I) * Sqrt (2.0) / Amount;
                  begin
                     if Delivered (I) and then F < Best (I) and then not Moving (I)
                       and then Driver.Uncertain.Significant
                         (Best (I) - F, Sqrt (S ** 2 + Best_Sigma (I) ** 2), Freedom (I))
                     then
                        --  Its own end: held where it last followed.
                        Following (I) := False;
                     else
                        Kept (I) := Sign * Amount;
                        if F > 0.0 and then Driver.Uncertain.Significant (F, S, Freedom (I))
                          and then (not Delivered (I) or else F > Best (I))
                        then
                           Delivered (I) := True;
                           Best (I) := F;
                           Best_Sigma (I) := S;
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            if Seen then
               --  Confirmed by moves back and forth by the same amount, each
               --  of them seen, as many as the chance of one false alarm asks.
               declare
                  Needed    : constant Natural := Run_Needed (Chance);
                  Run       : Natural := 1;
                  At_Amount : Boolean := True;
               begin
                  while Needed /= Natural'Last and then Run < Needed loop
                     At_Amount := not At_Amount;
                     declare
                        Back : Real_Array (1 .. N);
                     begin
                        for I in 1 .. N loop
                           Back (I) := (if At_Amount then Offsets (I) else 0.0);
                        end loop;
                        Move_And_Look (Back, Seen, Chance, Now, Moving);
                     end;
                     exit when not Seen;
                     Run := Run + 1;
                  end loop;
                  if Needed /= Natural'Last and then Run >= Needed then
                     Report.Seen := True;
                     Report.Excursion := Amount;
                  end if;
               end;
            end if;
            exit when Report.Seen or else (for all I in 1 .. N => not (Usable (I) and then Following (I)));
            Amount := 2.0 * Amount;
         end;
      end loop;
      --  Back to the hold.
      Step (M, Base, Report.Last);
   end Probe_Together;

   procedure Probe (M : in out Model; G : Group_Id; Channel : Positive; Direction : Real; Report : out Probe_Report) is
   begin
      Probe_Together (M, [1 => (Group => G, Channel => Channel)], Direction, 0.0, Report);
   end Probe;

   procedure Gather_Rest (M : in out Model; Probes : Natural) is
      B      : Driver.Clock.Beat;
      Wanted : Natural := 0;
   begin
      Driver.Beats.Next (B);
      declare
         Eyes   : constant Natural := Eye_Count (M);
         Rest   : Natural := Natural'Last;   --  the fewest counts at rest an eye has
         Window : Natural := 2;              --  the verdicts one move looks at, per eye
      begin
         for E in 1 .. Eyes loop
            Rest := Natural'Min (Rest, (if M.Eyes (Eye_Id (E)).Rest_Counts_Known
                                        then M.Eyes (Eye_Id (E)).Rest_Count_Beats else 0));
            Window := Natural'Max (Window, 2 + Natural'Max (0, Image_Lag (M, Eye_Id (E))));
         end loop;
         if Eyes > 0 and then Probes > 0 then
            declare
               Looked : constant Real := Real (Eyes * Window);
               Log_Levels : constant Real := Ln (Real (Levels) / Tail);
               --  The beats held still to have N counts at rest, plus every
               --  probe's run of moves at that chance of a false alarm.
               function Cost (N : Natural) return Real is
                 (if Real (N + 1) <= Looked then Real'Last
                  else Real (N - Rest) + Real (Probes * Window) * Log_Levels / Ln (Real (N + 1) / Looked));
               N : Natural := Rest;
            begin
               while Cost (N) = Real'Last or else Cost (N + 1) < Cost (N) loop
                  N := N + 1;
               end loop;
               Wanted := N - Rest;
            end;
         end if;
      end;
      Driver.Beats.Send (Driver.Commands.Hold);
      if Wanted > 0 then
         Hold (M, Wanted);
         Driver.Beats.Next (B);
         Estimate_Now (M);
         Driver.Beats.Send (Driver.Commands.Hold);
      end if;
   end Gather_Rest;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal) return Plan is
      pragma Unreferenced (M, A, O, Goal);
   begin
      return (State => Unmeasured, Reason => To_Unbounded_String ("the arm's kinematics are not measured yet"));
   end Plan_Reach;

   function Status (P : Plan) return Plan_Status is (P.State);
   function Why (P : Plan) return String is (To_String (P.Reason));

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report) is
      pragma Unreferenced (M, P);
   begin
      --  Only a planned path is followed (the precondition); none can be
      --  planned before the kinematics are measured.
      Report := (others => <>);
   end Follow;

end Driver.Robot.Motion;
