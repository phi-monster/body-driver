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
                       & (if E.Blocked then ", blocked" else "")
                       & (if E.Rested then "" else ", given up still moving") & "; ");
               Report.At_Rest := Report.At_Rest and then E.Rested;
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

   function Sweep_Start (M : Model; A : Arm_Id; Channel : Positive) return Real is
     (Kinematics.Keyframe_Step (M, A, Channel));

   procedure Hold_For_Keyframe (M : in out Model; A : Arm_Id) is
      B    : Driver.Clock.Beat;
      Done : Boolean;

      function Carries_An_Eye return Boolean is
        (for some E in 1 .. Eye_Count (M) =>
           Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A);
   begin
      loop
         Driver.Beats.Next (B);
         Done := not Carries_An_Eye or else (M.Beats > 0 and then Kinematics.Held_Still (M, A, M.Beats - 1));
         Driver.Beats.Send (Driver.Commands.Hold);
         exit when Done;
      end loop;
   end Hold_For_Keyframe;

   procedure Hold_For_Twin (M : in out Model; A : Arm_Id) is
      B    : Driver.Clock.Beat;
      Done : Boolean;

      function Carries_An_Eye return Boolean is
        (for some E in 1 .. Eye_Count (M) =>
           Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A);
   begin
      loop
         Driver.Beats.Next (B);
         Done := not Carries_An_Eye or else Kinematics.Twin_Answered (M, A);
         Driver.Beats.Send (Driver.Commands.Hold);
         exit when Done;
      end loop;
   end Hold_For_Twin;

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

   type Flag_Array is array (Positive range <>) of Boolean;

   --  Moves the listed channels to Base plus the offsets and looks for as
   --  long as a response takes to show in the readings and then in the
   --  images: whether some eye saw motion, the chance that a look at so many
   --  verdicts finds one by noise, and each reading at the end. Unlooked is
   --  the first beat no look has judged: the looks tile the probe, so a
   --  response that shows later than expected falls in the next one.
   procedure Move_And_Look
     (M        : in out Model;
      Listed   : Channel_Refs;
      Usable   : Flag_Array;
      Base     : Driver.Commands.Command;
      Offsets  : Real_Array;
      Unlooked : in out Natural;
      Seen     : out Boolean;
      Chance   : out Real;
      Now      : out Real_Array;
      Moving   : out Flag_Array)
   is
      N      : constant Natural := Listed'Length;
      B      : Driver.Clock.Beat;
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
                  if Usable (I) and then Channels.Has_Reading (M, Listed (I).Group, M.Beats - 1) then
                     Now (I) := Channels.Reading (M, Listed (I).Group, M.Beats - 1, Listed (I).Channel);
                     Moving (I) := Channels.Moving (M, Listed (I).Group, M.Beats - 1);
                  end if;
               end loop;
            end if;
            Driver.Beats.Send (Driver.Commands.Hold);
            exit when Done;
         end;
      end loop;
   end Move_And_Look;

   --  A probe level took the channel further along its ask than any smaller
   --  offset did, by Advance, as far as anything can tell: by a step an eye
   --  watching it can see, or, for a channel no eye watches, significantly
   --  against the readings' noise. (Not the fraction of the offset
   --  delivered: a reading held a constant hair off its target delivers a
   --  fraction that shrinks towards one as the offset grows, which exact
   --  readings call significant.)
   function Further
     (M : Model; Ref : Channel_Ref; Advance, Noise : Real; Freedom : Natural) return Boolean
   is
      V : constant Estimate := Visible_Step (M, Ref.Group, Ref.Channel);
   begin
      return (if Known (V) then Advance >= V.Value
              else Driver.Uncertain.Significant (Advance, Noise * Sqrt (2.0), Freedom));
   end Further;

   --  A level some eye saw, confirmed by moves back and forth by the same
   --  offsets, each of them seen, as many as the chance of one false alarm
   --  asks.
   procedure Confirm
     (M         : in out Model;
      Listed    : Channel_Refs;
      Usable    : Flag_Array;
      Base      : Driver.Commands.Command;
      Offsets   : Real_Array;
      Unlooked  : in out Natural;
      Chance    : Real;
      Confirmed : out Boolean)
   is
      Needed    : constant Natural := Run_Needed (Chance);
      Run       : Natural := 1;
      At_Amount : Boolean := True;
      Seen      : Boolean;
      Again     : Real;
      Now       : Real_Array (1 .. Listed'Length);
      Moving    : Flag_Array (1 .. Listed'Length);
   begin
      while Needed /= Natural'Last and then Run < Needed loop
         At_Amount := not At_Amount;
         declare
            Back : Real_Array (1 .. Listed'Length);
         begin
            for I in Back'Range loop
               Back (I) := (if At_Amount then Offsets (Offsets'First + I - 1) else 0.0);
            end loop;
            Move_And_Look (M, Listed, Usable, Base, Back, Unlooked, Seen, Again, Now, Moving);
         end;
         exit when not Seen;
         Run := Run + 1;
      end loop;
      Confirmed := Needed /= Natural'Last and then Run >= Needed;
   end Confirm;

   --  Where every listed channel is held and reads, read in a held beat:
   --  which channels can take part, their readings, their noise, the hold
   --  every listed group is moved from, and the first step when First is 0.
   procedure Begin_Probe
     (M        : in out Model;
      Listed   : Channel_Refs;
      First    : Real;
      Usable   : out Flag_Array;
      Start    : out Real_Array;
      Noise    : out Real_Array;
      Freedom  : out Count_Array;
      Base     : out Driver.Commands.Command;
      Amount   : out Real;
      Unlooked : out Natural)
   is
      B : Driver.Clock.Beat;
   begin
      Usable := [others => False];
      Start := [others => 0.0];
      Noise := [others => 0.0];
      Freedom := [others => 0];
      Base := Driver.Commands.Hold;
      Amount := First;
      Driver.Beats.Next (B);
      Unlooked := M.Beats;
      if M.Beats > 0 then
         for I in Listed'Range loop
            declare
               G : constant Group_Id := Listed (I).Group;
               C : constant Positive := Listed (I).Channel;
            begin
               if Natural (G) <= Group_Count (M) and then Is_Commandable (M, G) and then C <= Group_Size (M, G)
                 and then Channels.Has_Reading (M, G, M.Beats - 1)
               then
                  Usable (I) := True;
                  Start (I) := Channels.Reading (M, G, M.Beats - 1, C);
                  Noise (I) := Reading_Noise (M, G, C);
                  Freedom (I) := Channels.Noise_Freedom (M, G, C);
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
   end Begin_Probe;

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report;
      Answers   : out Real_Array)
   is
      Sign      : constant Real := (if Direction > 0.0 then 1.0 else -1.0);
      N         : constant Natural := Channels'Length;
      Listed    : constant Channel_Refs (1 .. N) := Channels;
      Usable    : Flag_Array (1 .. N);
      Base      : Driver.Commands.Command;            --  every listed group at its hold
      Start     : Real_Array (1 .. N);   --  each reading when the probe began
      Noise     : Real_Array (1 .. N);
      Freedom   : Count_Array (1 .. N);
      Following : Flag_Array (1 .. N);
      Delivered : Flag_Array (1 .. N) := [others => False];   --  some level moved it along the ask
      Farthest  : Real_Array (1 .. N) := [others => 0.0];   --  the farthest a level took it along the ask
      Kept      : Real_Array (1 .. N) := [others => 0.0];   --  the offset where it last followed
      Amount    : Real;
      Unlooked  : Natural;
   begin
      Report := (others => <>);
      Answers := [others => 0.0];
      Begin_Probe (M, Listed, First, Usable, Start, Noise, Freedom, Base, Amount, Unlooked);
      Following := Usable;
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
            Move_And_Look (M, Listed, Usable, Base, Offsets, Unlooked, Seen, Chance, Now, Moving);
            --  Which channels still follow: whether asking further took each
            --  further than any smaller offset did.
            for I in 1 .. N loop
               if Usable (I) and then Following (I) then
                  declare
                     Excursion : constant Real := Sign * (Now (I) - Start (I));
                     Went      : constant Boolean :=
                       Further (M, Listed (I), Excursion - Farthest (I), Noise (I), Freedom (I));
                  begin
                     if Delivered (I) and then not Went and then not Moving (I) then
                        --  Its own end: held where it last followed.
                        Following (I) := False;
                     else
                        Kept (I) := Sign * Amount;
                        if Went then
                           if not Delivered (I) then
                              Answers (Answers'First + I - 1) := Amount;
                           end if;
                           Delivered (I) := True;
                           Farthest (I) := Excursion;
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            if Seen then
               declare
                  Confirmed : Boolean;
               begin
                  Confirm (M, Listed, Usable, Base, Offsets, Unlooked, Chance, Confirmed);
                  if Confirmed then
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

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report)
   is
      Answers : Real_Array (1 .. Channels'Length);
   begin
      Probe_Together (M, Channels, Direction, First, Report, Answers);
   end Probe_Together;

   procedure Probe_Both_Ways
     (M      : in out Model;
      Ref    : Channel_Ref;
      First  : Real;
      Bound  : Real;
      Report : out Two_Way_Report)
   is
      Listed    : constant Channel_Refs (1 .. 1) := [1 => Ref];
      Usable    : Flag_Array (1 .. 1);
      Base      : Driver.Commands.Command;
      Start     : Real_Array (1 .. 1);
      Noise     : Real_Array (1 .. 1);
      Freedom   : Count_Array (1 .. 1);
      Amount    : Real;
      Unlooked  : Natural;
      Signs     : constant array (Sense) of Real := [Increasing => 1.0, Decreasing => -1.0];
      Other     : constant array (Sense) of Sense := [Increasing => Decreasing, Decreasing => Increasing];
      Open      : Sense_Flags := [others => True];
      Delivered : Sense_Flags := [others => False];
      Answered  : Sense_Counts := [others => 0];   --  the level at which it first followed that way
      Farthest  : array (Sense) of Real := [others => 0.0];
   begin
      Report := (others => <>);
      Begin_Probe (M, Listed, First, Usable, Start, Noise, Freedom, Base, Amount, Unlooked);
      if not Usable (1) then
         return;
      end if;
      loop
         declare
            Way   : Sense := Increasing;
            Found : Boolean := False;
         begin
            --  The next way: the one asked at the lower level, the increasing
            --  one first; the decreasing one only while the increasing one has
            --  delivered nothing, or once it has ended.
            for S in Sense loop
               if Open (S) and then Report.Levels (S) < Levels
                 and then (S = Increasing or else not Open (Increasing) or else not Delivered (Increasing))
                 and then (not Found or else Report.Levels (S) < Report.Levels (Way))
               then
                  Way := S;
                  Found := True;
               end if;
            end loop;
            exit when not Found;
            Report.Levels (Way) := Report.Levels (Way) + 1;
            declare
               Offset  : constant Real := Amount * 2.0 ** (Report.Levels (Way) - 1);
               Offsets : constant Real_Array (1 .. 1) := [1 => Signs (Way) * Offset];
               Seen    : Boolean;
               Chance  : Real;
               Now     : Real_Array (1 .. 1);
               Moving  : Flag_Array (1 .. 1);
            begin
               Move_And_Look (M, Listed, Usable, Base, Offsets, Unlooked, Seen, Chance, Now, Moving);
               declare
                  Excursion : constant Real := Signs (Way) * (Now (1) - Start (1));
                  Went      : constant Boolean := Further (M, Ref, Excursion - Farthest (Way), Noise (1), Freedom (1));
               begin
                  if Delivered (Way) and then not Went and then not Moving (1) then
                     --  Its own end.
                     Open (Way) := False;
                  elsif Went then
                     if not Delivered (Way) then
                        Answered (Way) := Report.Levels (Way);
                        Report.Answered := (if Report.Answered = 0.0 then Offset else Real'Min (Report.Answered, Offset));
                     end if;
                     Delivered (Way) := True;
                     Farthest (Way) := Excursion;
                  end if;
               end;
               if Seen then
                  declare
                     Confirmed : Boolean;
                  begin
                     Confirm (M, Listed, Usable, Base, Offsets, Unlooked, Chance, Confirmed);
                     if Confirmed then
                        Report.Seen := True;
                        Report.Excursion := Offset;
                        Report.Seen_Sense := Way;
                     end if;
                  end;
               end if;
               exit when Report.Seen;
               --  A limit is one-sided: a way that delivered nothing up to the
               --  level at which the other one answered is at its end.
               for S in Sense loop
                  if Open (S) and then not Delivered (S) and then Delivered (Other (S))
                    and then Report.Levels (S) >= Answered (Other (S))
                  then
                     Open (S) := False;
                     Report.At_End (S) := True;
                  end if;
               end loop;
               --  Neither way answered when both were asked as much as every
               --  other channel of the body needed: dead or disconnected.
               if not Delivered (Increasing) and then not Delivered (Decreasing)
                 and then Report.Levels (Increasing) = Report.Levels (Decreasing) and then Offset >= Bound
               then
                  Report.Dead := True;
                  exit;
               end if;
            end;
         end;
      end loop;
      --  Back to the hold.
      Step (M, Base, Report.Last);
   end Probe_Both_Ways;

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
      use Driver.Numerics.Arrays;

      function Refused (State : Plan_Status; Why : String) return Plan is
        ((State => State, Reason => To_Unbounded_String (Why), others => <>));
   begin
      if Natural (A) > Arm_Count (M) or else not Kinematics.Fitted (M, A) then
         return Refused (Unmeasured, "the arm's kinematics are not measured yet");
      end if;
      declare
         G    : constant Group_Id := Arm_Group (M, A);
         Size : constant Natural := Group_Size (M, G);
         Tool : constant Pose_Estimate := Tool_Pose (M, A, O);
      begin
         if Size = 0 or else Natural (G) > Natural (O.Readings.Length) or else O.Readings.Element (G)'Length /= Size then
            return Refused (Unmeasured, "the arm's readings are missing at that beat");
         elsif Tool.Position_Covariance (1, 1) = Real'Last then
            return Refused (Unmeasured, "the arm is not measured into the world yet");
         end if;
         declare
            --  The joints' range: the readings the arm has moved through, where
            --  it was free; beyond them nothing is known.
            Low, High : Real_Array (1 .. Size);
            Sigma : constant Real := Kinematics.Angle_Sigma (M, A);
            Start : constant Real_Array (1 .. Size) := O.Readings.Element (G);
            From  : constant Rigid := Tool.Pose;
            Result : Plan := (State => Planned, Reason => Null_Unbounded_String, Group => G, others => <>);
            Failed : Boolean := False;
            Worst_Position, Worst_Turn : Real := 0.0;

            --  The pose a fraction of the way from the start to the goal:
            --  straight in position, about one axis in turn.
            function Along (S : Real) return Rigid is
              ((Rotation    => From.Rotation
                                 * Driver.Numerics.Exp (S * Driver.Numerics.Log (Transpose (From.Rotation)
                                                                                   * Goal.Pose.Rotation)),
                Translation => (1.0 - S) * From.Translation + S * Goal.Pose.Translation));

            --  Solves the path from Fraction A (readings Q) to B; a segment the
            --  solver cannot close from where the last one ended is halved, at
            --  most as many times as a float has bits.
            procedure Reach (A_Of, B_Of : Real; Q : in out Real_Array; Depth : Natural) is
               Next : Real_Array (1 .. Size);
               Position_Off, Turn_Off : Real;
            begin
               Kinematics.Solve_Pose (M, A, Q, Along (B_Of), Goal.Position_Only and then B_Of = 1.0,
                                      Low, High, Next, Position_Off, Turn_Off);
               if not Driver.Uncertain.Significant (Position_Off, Sigma)
                 and then not Driver.Uncertain.Significant (Turn_Off, Sigma)
               then
                  Result.Waypoints.Append (Next);
                  Q := Next;
               elsif Depth < Real'Machine_Mantissa then
                  Reach (A_Of, (A_Of + B_Of) / 2.0, Q, Depth + 1);
                  if not Failed then
                     Reach ((A_Of + B_Of) / 2.0, B_Of, Q, Depth + 1);
                  end if;
               else
                  Failed := True;
                  Worst_Position := Position_Off;
                  Worst_Turn := Turn_Off;
               end if;
            end Reach;

            Q : Real_Array (1 .. Size) := Start;
         begin
            for C in 1 .. Size loop
               Low (C) := Real'Min (Channels.Lowest (M, G, C), Start (C));
               High (C) := Real'Max (Channels.Highest (M, G, C), Start (C));
            end loop;
            if Sigma = Real'Last then
               return Refused (Unmeasured, "the arm's fit has no uncertainty");
            end if;
            Reach (0.0, 1.0, Q, 0);
            if Failed then
               return Refused (Unreachable, "within the readings the arm has moved through, the goal stays"
                               & Real'Image (Worst_Position) & " model units and" & Real'Image (Worst_Turn)
                               & " rad away");
            end if;
            return Result;
         end;
      end;
   end Plan_Reach;

   function Status (P : Plan) return Plan_Status is (P.State);
   function Why (P : Plan) return String is (To_String (P.Reason));

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report) is
   begin
      Report := (Outcome => Reached, others => <>);
      for W of P.Waypoints loop
         declare
            C : Driver.Commands.Command;
         begin
            Driver.Commands.Set_Target (C, P.Group, W);
            Step (M, C, Report);
         end;
         exit when Report.Outcome /= Reached;
      end loop;
   end Follow;

end Driver.Robot.Motion;
