with Ada.Characters.Handling;
with Ada.Environment_Variables;
with Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Alignment.Keyframes;
with Driver.Clock;
with Driver.Memory;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Stillness;
with Driver.Stats;
with Driver.Uncertain;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Lockin;
with Driver.Robot.Steps;
with Driver.Recording;
with Driver.Bytes;
with Driver.Robot.Regression;

package body Driver.Robot.Kinematics is

   --  Everything sized by beats, keyframes, queries or matches lives on the
   --  heap: the estimates also run in the decider's task, whose stack is
   --  small.
   type Real_Access is access Real_Array;
   type Count_List is array (Positive range <>) of Natural;
   type Counts_Access is access Count_List;
   procedure Free is new Ada.Unchecked_Deallocation (Count_List, Counts_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
   type Play_Access is access Fit.Play_Paths;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Play_Paths, Play_Access);
   type Flags_Access is access Fit.Flag_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Flag_Matrix, Flags_Access);
   type Flag_Array is array (Natural range <>) of Boolean;
   type Flag_Access is access Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);

   --  The terms of a fixed eye's covariance: its lens's six and its pose's six (the camera's turn about its own
   --  axes and its centre).
   Fixed_Terms : constant := Fit.Lens_Terms + 6;

   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   function Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base is
   begin
      for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
         if M.Graph.Mounts (E).Kind = Arm_Carried and then M.Graph.Mounts (E).Arm = A then
            return E;
         end if;
      end loop;
      return 0;
   end Eye_Of;

   --  The centres of the eye's cells that respond to the group's push: they show the world the arm moves its eye
   --  through. Every textured cell when the lock-in has not told yet, and every textured cell of an eye the group's
   --  arm carries: the world fills its view, and a cell the recognition's small pushes did not move beyond its noise
   --  is moved by a keyframe's (f15h on 10-15: 49 of 242 textured cells responded, 19 to 30 of them were matched
   --  a keyframe, too few for the keyframe's own field, and no keyframe was followed). Responds says which cells
   --  the lock-in saw follow the push (all of them when it has not told): the others, the hand's jaws among them,
   --  are asked but tell nothing of the view's motion until the keyframes tell (Arm_Evidence.Query_Responds).
   procedure Query_Points
     (M : Model; E : Eye_Id; G : Group_Id; Picture : Driver.Images.Image; U, V : out Real_Vectors.Vector;
      Responds : out Flag_Vectors.Vector)
   is
      Carried : constant Boolean :=
        Eye_Mount (M, E).Kind = Arm_Carried and then Arm_Group (M, Eye_Mount (M, E).Arm) = G;
      S    : Eye_Stream renames M.Eyes (E);
      N    : constant Natural := Cells (S.Grid);
      Kept : constant Natural := Natural (S.Kept_Groups.Length);
      W    : constant Natural := Driver.Images.Width (Picture);
      H    : constant Natural := Driver.Images.Height (Picture);
      --  Where in a cell the aligner can place a patch best, and whether it can at all: the pixel whose patch (the
      --  aligner's, Half_Patch either side) holds the most slope energy, the quantity the aligner's own Newton step
      --  rests on, among the places whose patch has texture across both directions, the smaller eigenvalue of its
      --  gradient tensor (along an edge or a smooth gradient a patch slides, and any place along it answers), above
      --  what the eye's resting noise alone gives a patch by Z of its spread. Placed by the slope energy alone and
      --  kept only where that one place was so textured, Shadow seed 2a's arm (p273s2, 10-24) kept 71 of its 136
      --  sound tracks, the slope's greatest on edges, and its joints' order came out wrong, its focal length 5.6 %
      --  short. A cell's centre lies on a flat place as often as
      --  not (the kiwi's white table: of the points asked, 19 % failed for want of texture and 17 % for a patch over
      --  the picture's edge, R6's seventh and eighth starts, 10-14), and the patch with the most slope energy in a
      --  cell of a smooth sky, a plain floor or the arm's own jaws was matched where no point of the world was:
      --  RB-Y1 seed 2b's arm 3 (p207s2, 10-24) had 13 of its 140 tracks wrong in most of their sightings against the
      --  world's truth, every one of them below that noise, and 76 of its 86 sound tracks above it by a median of 55
      --  of its spreads; such tracks pulled arms' focal lengths 3 to 8 times off (SO-101 seeds 1b and 3b, RB-Y1 2b
      --  and 3a). Placed by that eigenvalue instead, the points sat on a checkered texture's corners, each like the
      --  next, and no common motion of the view was told (robot.sweep.start, 10-24). The sums over each patch come
      --  from integral images, one pass over the picture.
      Half : constant Natural := Driver.Alignment.Half_Patch (W, H);
      type Reals_Access is access Real_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Reals_Access);
      function Sqrt (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Sqrt;
      --  The integral images of the products of the differences between a pixel's neighbours across (Dx) and down
      --  (Dy), and of each pixel's resting variance (the eye's stillness test, Stillness); none without a picture (a
      --  rig's eye without pictures, whose cells keep their centres).
      Pictured : constant Boolean := W > 0 and then H > 0;
      Noise_Known : constant Boolean := Pictured and then S.Variances /= null and then S.Variances'Length = W * H;
      Size : constant Natural := (if Pictured then (W + 1) * (H + 1) else 0);
      Txx, Tyy, Txy, Tv : Reals_Access := (if Pictured then new Real_Array'(1 .. Size => 0.0) else null);
      --  The integral image's entry for the box up to column X and row Y (exclusive), from 1.
      function Place (X, Y : Natural) return Positive is (Y * (W + 1) + X + 1);
      --  A sum over the patch around (X, Y): the integral image's box.
      function Box (T : Reals_Access; X, Y : Natural) return Real is
        (T (Place (X + Half + 1, Y + Half + 1)) - T (Place (X - Half, Y + Half + 1))
         - T (Place (X + Half + 1, Y - Half)) + T (Place (X - Half, Y - Half)));
      --  The smaller eigenvalue of the patch's gradient tensor around (X, Y).
      function Least (X, Y : Natural) return Real is
        ((Box (Txx, X, Y) + Box (Tyy, X, Y)) / 2.0
         - Sqrt (((Box (Txx, X, Y) - Box (Tyy, X, Y)) / 2.0) ** 2 + Box (Txy, X, Y) ** 2));
      --  Whether a patch's smaller eigenvalue stands above the resting noise's: a difference of two pixels has twice
      --  a pixel's variance (the noise at rest and the rounding of the level, Driver.Alignment.Quantization), so a
      --  sum of Count squared differences averages Count times twice it, its square spreading by twice its variance
      --  squared over each (a Gaussian's square has twice its variance squared): the root of eight times Count times
      --  the pixel's variance.
      function Textured_Patch (X, Y : Natural; Smaller : Real) return Boolean is
         Count : constant Real := Real ((2 * Half + 1) ** 2);
         Pixel : constant Real := Box (Tv, X, Y) / Count + Driver.Alignment.Quantization;
      begin
         return Smaller > 2.0 * Count * Pixel + Driver.Conventions.Z * Sqrt (8.0 * Count) * Pixel;
      end Textured_Patch;
   begin
      U.Clear;
      V.Clear;
      Responds.Clear;
      if Pictured then
         declare
            Luma : Reals_Access := new Real_Array (1 .. W * H);
            function Pixel (X, Y : Natural) return Positive is (Y * W + X + 1);
            procedure Add (T : Reals_Access; X, Y : Natural; Value : Real) is
            begin
               T (Place (X + 1, Y + 1)) := Value + T (Place (X + 1, Y)) + T (Place (X, Y + 1)) - T (Place (X, Y));
            end Add;
         begin
            Driver.Images.Luma (Picture, Luma.all);
            for Y in 0 .. H - 1 loop
               for X in 0 .. W - 1 loop
                  declare
                     Dx, Dy : Real := 0.0;
                  begin
                     if X > 0 and then X < W - 1 and then Y > 0 and then Y < H - 1 then
                        Dx := Luma (Pixel (X + 1, Y)) - Luma (Pixel (X - 1, Y));
                        Dy := Luma (Pixel (X, Y + 1)) - Luma (Pixel (X, Y - 1));
                     end if;
                     Add (Txx, X, Y, Dx * Dx);
                     Add (Tyy, X, Y, Dy * Dy);
                     Add (Txy, X, Y, Dx * Dy);
                     Add (Tv, X, Y, (if Noise_Known then S.Variances (S.Variances'First + Pixel (X, Y) - 1) else 0.0));
                  end;
               end loop;
            end loop;
            Free (Luma);
         end;
      end if;
      for C in 1 .. N loop
         declare
            Follows : Boolean := Kept = 0;
            X0, X1, Y0, Y1 : Natural;
         begin
            for K in 0 .. Kept - 1 loop
               if S.Kept_Groups (K) = Natural (G) and then S.Shifts ((C - 1) * Kept + K) > 0.0 then
                  Follows := True;
               end if;
            end loop;
            if (Follows or else Carried) and then (Natural (S.Textured.Length) < N or else S.Textured (C - 1)) then
               Flow.Bounds (S.Grid, C, X0, X1, Y0, Y1);
               declare
                  Best_X : Natural := (X0 + X1) / 2;
                  Best_Y : Natural := (Y0 + Y1) / 2;
                  Best   : Real := -1.0;
                  --  A rig's eye without pictures keeps its cells' centres; a picture's cell its best place among those
                  --  whose whole patch lies in the picture and is textured above the resting noise, and none where no
                  --  place is.
                  Keep   : Boolean := not Pictured;
               begin
                  if Pictured and then W > 2 * Half and then H > 2 * Half then
                     for Y in Natural'Max (Y0, Half) .. Natural'Min (Y1, H - Half) - 1 loop
                        for X in Natural'Max (X0, Half) .. Natural'Min (X1, W - Half) - 1 loop
                           declare
                              Energy : constant Real := Box (Txx, X, Y) + Box (Tyy, X, Y);
                           begin
                              if Energy > Best and then Noise_Known and then Textured_Patch (X, Y, Least (X, Y)) then
                                 Best := Energy;
                                 Best_X := X;
                                 Best_Y := Y;
                                 Keep := True;
                              end if;
                           end;
                        end loop;
                     end loop;
                  end if;
                  if Keep then
                     U.Append (Real (Best_X) + 0.5);
                     V.Append (Real (Best_Y) + 0.5);
                     Responds.Append (Follows);
                  end if;
               end;
            end if;
         end;
      end loop;
      Free (Txx);
      Free (Tyy);
      Free (Txy);
      Free (Tv);
   end Query_Points;

   --  The query point rides with the eye by its own patch (Patch_Evidence): in the keyframes whose view moved, its
   --  patch stood still more often than the points' patches stand by chance (Patch_Rate). Told by a field of the view
   --  alone (below), a point of the jaws ride into whatever field the keyframe told: w18's arm 2 (servo seed 11 of the
   --  fake robot, 10-19) asked its jaws' points where the fitted arm moved the world to, 50 to 103 px from where they
   --  stood, stating 1.2 to 2.4 px, and found them on the table's grain there.
   function Rides_By_Patch (R : Arm_Evidence; P : Natural) return Boolean is
     (P < Natural (R.Patch_Stood.Length) and then P < Natural (R.Patch_Moving.Length) and then R.Patch_Stood (P) > 0
      and then R.Patch_Rate > 0.0 and then R.Patch_Rate < 1.0
      and then Regression.Count_Significant (R.Patch_Stood (P), R.Patch_Moving (P), R.Patch_Rate));

   --  The point moves with the view, as far as its keyframes tell (Measure_Shifts: not Arm_Evidence.Still).
   --  The query point rides with the eye by the common motions of its keyframes (Keyframes.Stood_And_Moved): it
   --  stood where it was against the field told more often, out of the times it had a place there, than the points
   --  told of the world stood so (Measure_Shifts: Stand_Rate). Before any point was told of the world, no point rides
   --  by them. At the even chance the test asked nine stands of a jaw's point, and the keyframes of the sweep's
   --  first pass were followed on the jaws; at the per-point test's false-alarm rate it took two, and a third of the
   --  world's points with them (s29 replayed, 10-17: 1544 sightings of 5000 left, the focal length 172 px of 135). Its slopes (Measure_Shifts) are taken from the
   --  answers that agree with their keyframe's field, and a point on the jaws' edges agrees with no field whose
   --  view moved: it was never told still, rode into every common motion and every field, and the keyframes that
   --  turned the wrist far, whose points of the world were found nowhere, were followed on the jaws' points alone
   --  at no motion (s30, servo seed 7, 10-17, judged against the simulator's truth: keyframes of the roll turned 8
   --  to 88 degrees, their sightings moved a mean 0.2 px).
   function Rides (R : Arm_Evidence; P : Natural) return Boolean is
     (Rides_By_Patch (R, P) or else
      (P < Natural (R.Stood_Off.Length) and then P < Natural (R.Moved_With.Length) and then R.Stood_Off (P) > 0
      and then R.Stand_Rate > 0.0
      and then Regression.Count_Significant (R.Stood_Off (P), R.Stood_Off (P) + R.Moved_With (P), R.Stand_Rate)));

   --  A point moves with the view when the lock-in sees its cell follow the arm's pushes (Query_Responds, read again at
   --  every estimate: Refresh_Responds) and its keyframes neither tell it still nor riding. The keyframes' verdict alone
   --  stood in for the lock-in's once it was told, and a point of the jaws, matched in the wrong place as often as not,
   --  moved by its answers: u6 (servo seed 3 of the fake robot, 10-19), 65 of its 255 points on the jaws, and 598
   --  sightings of them in the fit, 382 off the truth by more than 4 px, of the run's 653 such.
   function Moving_Point (R : Arm_Evidence; P : Natural) return Boolean is
     ((P >= Natural (R.Query_Responds.Length) or else R.Query_Responds (P))
      and then (P >= Natural (R.Still.Length) or else not R.Still (P)) and then not Rides (R, P));

   --  The lock-in's latest verdict on every query point's cell: whether it follows the arm's pushes (Query_Points'
   --  Follows), kept as it was while the lock-in has told nothing. A cell of the world the recognition's small pushes
   --  did not move follows once the sweep's pushes are measured; the hand's jaws in a wrist eye's view never do.
   procedure Refresh_Responds (M : Model; R : in out Arm_Evidence) is
   begin
      if R.Eye = 0 or else R.Eye > M.Eyes.Last_Index then
         return;
      end if;
      declare
         S    : Eye_Stream renames M.Eyes (R.Eye);
         G    : Cell_Grid renames S.Grid;
         Kept : constant Natural := Natural (S.Kept_Groups.Length);
      begin
         if Kept = 0 or else G.Width = 0 or else G.Height = 0 then
            return;
         end if;
         for P in 0 .. Natural'Min (Natural (R.Query_U.Length), Natural (R.Query_Responds.Length)) - 1 loop
            declare
               X       : constant Natural := Natural'Min (G.Width - 1, Natural (Real'Floor (Real'Max (0.0, R.Query_U (P)))));
               Y       : constant Natural := Natural'Min (G.Height - 1, Natural (Real'Floor (Real'Max (0.0, R.Query_V (P)))));
               Cell    : constant Natural :=
                 ((Y + 1) * G.Rows - 1) / G.Height * G.Columns + ((X + 1) * G.Columns - 1) / G.Width;
               Follows : Boolean := False;
            begin
               for K in 0 .. Kept - 1 loop
                  if S.Kept_Groups (K) = Natural (R.Group) and then Cell * Kept + K < Natural (S.Shifts.Length)
                    and then S.Shifts (Cell * Kept + K) > 0.0
                  then
                     Follows := True;
                  end if;
               end loop;
               R.Query_Responds.Replace_Element (P, Follows);
            end;
         end loop;
      end;
   end Refresh_Responds;

   --  Per point of the arm's reference, whether it does not move with the view (not Moving_Point): still at every
   --  keyframe, or riding with the eye by the keyframes' own evidence (Rides). Set_Of judges a keyframe's sightings
   --  by it: the merge of the fixed eyes' views (Set_Of's flags given apart, d449b0c6) had dropped the riding
   --  points from it, and the jaws in a wrist eye's view would be sightings of the world again (691533d2).
   function Unmoving (R : Arm_Evidence) return Flag_Vectors.Vector is
   begin
      return Flags : Flag_Vectors.Vector do
         for P in 0 .. Natural (R.Query_U.Length) - 1 loop
            Flags.Append (not Moving_Point (R, P));
         end loop;
      end return;
   end Unmoving;

   function Readings_Of (M : Model; G : Group_Id; Beat : Natural) return Real_Array is
      Result : Real_Array (1 .. M.Groups (G).Size);
   begin
      for C in Result'Range loop
         Result (C) := Channels.Reading (M, G, Beat, C);
      end loop;
      return Result;
   end Readings_Of;

   --  The arm's joints along it, from its base, as its fit found them.
   function Chain_Of (R : Arm_Fit) return Fit.Chain_Order is
     ([for K in 1 .. Natural (R.Chain.Length) => Positive (R.Chain (R.Chain.First_Index + K - 1))]);

   function Chain_Image (C : Fit.Chain_Order) return String is
     (if C'Length = 0 then ""
      elsif C'Length = 1 then Ada.Strings.Fixed.Trim (C (C'First)'Image, Ada.Strings.Left)
      else Ada.Strings.Fixed.Trim (C (C'First)'Image, Ada.Strings.Left) & ", " & Chain_Image (C (C'First + 1 .. C'Last)));

   --  Each fitted joint's half slack and where its slack stood at the reference (Fit_Report.Slacks, Slack_Places), in
   --  reading units, for the log; none when not fitted.
   function Slacks_Image (Report : Fit.Fit_Report) return String is
      Text : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for K in Report.Slacks.First_Index .. Report.Slacks.Last_Index loop
         Ada.Strings.Unbounded.Append
           (Text, (if K = Report.Slacks.First_Index then "; half slacks, by the joints fitted: " else ", ")
                  & Driver.Log.Image (Report.Slacks (K), 4));
      end loop;
      for K in Report.Slack_Places.First_Index .. Report.Slack_Places.Last_Index loop
         Ada.Strings.Unbounded.Append
           (Text, (if K = Report.Slack_Places.First_Index then "; their slacks at the reference, from the middle: "
                   else ", ")
                  & Driver.Log.Image (Report.Slack_Places (K), 4));
      end loop;
      return Ada.Strings.Unbounded.To_String (Text);
   end Slacks_Image;

   --  How long each stage of a fit took, in seconds.
   function Stages_Image (Report : Fit.Fit_Report) return String is
      Text : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for K in Report.Seconds.First_Index .. Report.Seconds.Last_Index loop
         Ada.Strings.Unbounded.Append
           (Text, (if K = Report.Seconds.First_Index then "; its stages took " else ", ")
                  & Driver.Log.Image (Report.Seconds (K), 1));
      end loop;
      return Ada.Strings.Unbounded.To_String (Text) & (if Report.Seconds.Is_Empty then "" else " s");
   end Stages_Image;

   --  The evidence is an arm's now: the graph still lists its group as an
   --  arm, under that number, carrying that eye. Evidence is kept by group,
   --  so an arm renumbered by a new one keeps it; an arm that stopped being
   --  one is not fitted.
   function Current (M : Model; R : Arm_Evidence) return Boolean is
     (R.Arm in 1 .. Arm_Id'Base (Arm_Count (M)) and then Arm_Group (M, R.Arm) = R.Group
      and then Eye_Of (M, R.Arm) = R.Eye);

   --  The evidence of arm A as the graph has it now; 0 when there is none.
   function Index_Of (M : Model; A : Arm_Id) return Natural is
   begin
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if M.Kinematics (K).Arm = A and then Current (M, M.Kinematics (K)) then
            return K;
         end if;
      end loop;
      return 0;
   end Index_Of;

   --  How far the aligner errs on the arm's keyframes (Driver.Alignment.Keyframes.Scatter).
   function Match_Sigma (R : Arm_Evidence) return Real is
     (if Driver.Alignment.Keyframes.Scatter (R.Chain) > 0.0 then Driver.Alignment.Keyframes.Scatter (R.Chain)
      else R.Scatter);

   --  The pixels the arm's eye's view moves per reading unit of a channel: the keyframes' own measurement
   --  (Measure_Shifts) when it is significant; 0 otherwise. The lock-in's slope is no measurement of it on every
   --  scene (R4 on the Pi, 10-13: a flow locked on the table's JPEG blocks gave 0.01 px a unit against 0.4 from
   --  the keyframes) and enters only as the prediction a keyframe is aligned from when a search as wide as the
   --  aligner looks has found nothing (Align_Arm).
   function Pixels_Per_Unit (M : Model; R : Arm_Evidence; Channel : Positive) return Real is
      pragma Unreferenced (M);
      Measured : constant Boolean :=
        Channel <= Natural (R.Shift_Sigmas.Length) and then R.Shift_Sigmas (Channel - 1) < Real'Last;
      Seen     : constant Boolean := Channel > Natural (R.Shift_Seen.Length) or else R.Shift_Seen (Channel - 1);
   begin
      return (if Measured and then Seen
                and then Driver.Uncertain.Significant (R.Shifts (Channel - 1), R.Shift_Sigmas (Channel - 1))
              then R.Shifts (Channel - 1) else 0.0);
   end Pixels_Per_Unit;

   function Pixels_Per_Unit (M : Model; A : Arm_Id; Channel : Positive) return Real is
     (if Index_Of (M, A) > 0 then Pixels_Per_Unit (M, M.Kinematics (Index_Of (M, A)), Channel) else 0.0);

   function Shift_Unseen (M : Model; A : Arm_Id; Channel : Positive) return Boolean is
     (Index_Of (M, A) > 0
      and then Channel <= Natural (M.Kinematics (Index_Of (M, A)).Shift_Seen.Length)
      and then not M.Kinematics (Index_Of (M, A)).Shift_Seen (Channel - 1));

   function Held_Still (M : Model; A : Arm_Id; Beat : Natural) return Boolean is
      E : constant Eye_Id'Base := Eye_Of (M, A);
      G : constant Group_Id := Arm_Group (M, A);
      --  Over the time the group's response takes to show (its measured push
      --  delay, and the beat itself): a joint on its way at a steady rate
      --  below the readings' noise per beat is not still, though no two beats
      --  tell (a reading creeping towards its command; item 21). Still at
      --  every beat of the span, and not moved over the span as a whole.
      --  And over the eye's lag: its picture shows the beat its lag before, and readings still for less than it
      --  leave the picture showing the move, its watch still the last one's: robot.boot.settling (10-17), arm 1
      --  given a keyframe at the beat it stopped from a one-beat move, its picture a beat behind and flickering.
      Span : constant Positive := 1 + (if M.Groups (G).Delay_Known then M.Groups (G).Delay_Beats else 0)
        + (if E = 0 then 0 else Natural'Max (0, Image_Lag (M, E)));
   begin
      if E = 0 or else Beat < Span then
         return False;
      end if;
      --  The eye's picture by the one stop rule (Stillness.Eye_Settled), which
      --  holds for the latest beat.
      if not Stillness.Eye_Settled (M, E) then
         return False;
      end if;
      for B in Beat - Span + 1 .. Beat loop
         if not Channels.Has_Reading (M, G, B) or else not Channels.Has_Reading (M, G, B - 1)
           or else Channels.Moving (M, G, B)
         then
            return False;
         end if;
      end loop;
      declare
         Now   : constant Real_Array := Readings_Of (M, G, Beat);
         Since : constant Real_Array := Readings_Of (M, G, Beat - Span);
         D     : Real_Array (1 .. Now'Length);
      begin
         for C in D'Range loop
            --  Less the creep of the span's beats (Channels.Drift): a reading that creeps at rest changes over the span
            --  by its creep a beat times the span, and that is rest, as Steps has it. Taken whole, it held ba_t21's
            --  arm 1 for a keyframe 5850 beats (seed 21 of the default fake robot, 10-22, from beat 6564 until the
            --  estimate of beat 12413 measured again): its six readings crept 3e-10 to 5e-9 units a beat, their noise
            --  1.05 times that, and over the span of 4 beats each crept 2.7 of its sigmas, the six together beyond
            --  the gate, while the eye's picture had settled.
            D (C) := Now (Now'First + C - 1) - Since (Since'First + C - 1) - Channels.Drift (M, G, C) * Real (Span);
         end loop;
         return not Channels.Visible (M, G, D);
      end;
   end Held_Still;

   --  The graph mounts the eye fixed in the world.
   function Is_Fixed (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Graph.Mounts.Last_Index and then M.Graph.Mounts (E).Kind = World_Fixed);

   --  Eye E's view shows group G move, as the lock-in measured it.
   function Sees (M : Model; E : Eye_Id; G : Group_Id) return Boolean is
      K : constant Natural := (Natural (G) - 1) * Natural (M.Eyes.Length) + Natural (E);
   begin
      return K in M.Graph.Effects.First_Index .. M.Graph.Effects.Last_Index
        and then M.Graph.Effects (K).Verdict in Patch | Whole;
   end Sees;

   procedure Observe (M : in out Model; O : Observation) is
      Beat : constant Natural := M.Beats;
   begin
      for A in 1 .. Arm_Count (M) loop
         declare
            Arm : constant Arm_Id := Arm_Id (A);
            G   : constant Group_Id := Arm_Group (M, Arm);
            E   : constant Eye_Id'Base := Eye_Of (M, Arm);
            Index : Natural := 0;
         begin
            if E > 0 then
               --  An arm is a group and the eye it carries: the arms of one group (the 2b split, a humanoid's two eyes)
               --  each keep their own evidence. Found by the pair; failing that, the group's evidence of an eye that no
               --  arm of the group carries now, which the graph tells differently, starts afresh for this one; failing
               --  that, a new arm starts its evidence.
               for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                  if M.Kinematics (K).Group = G and then M.Kinematics (K).Eye = E then
                     Index := K;
                  end if;
               end loop;
               if Index = 0 then
                  for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                     if M.Kinematics (K).Group = G
                       and then not (for some B in 1 .. Arm_Count (M) =>
                                       Arm_Group (M, Arm_Id (B)) = G and then Eye_Of (M, Arm_Id (B)) = M.Kinematics (K).Eye)
                     then
                        Index := K;
                     end if;
                  end loop;
                  if Index = 0 then
                     M.Kinematics.Append (Arm_Evidence'(Arm => Arm, Group => G, Eye => E, others => <>));
                     Index := M.Kinematics.Last_Index;
                  else
                     M.Kinematics.Replace_Element (Index, (Arm => Arm, Group => G, Eye => E, others => <>));
                  end if;
               elsif M.Kinematics (Index).Arm /= Arm then
                  M.Kinematics (Index).Arm := Arm;
               end if;
               declare
                  R : Arm_Evidence renames M.Kinematics (Index);
               begin
                  --  Keyframes from the arm's sweep on (Arm_Evidence.Sweeping, Begin_Anew): the recognition's rounds
                  --  held the kiwi's arms at 23 to 47 poses an arm, every one a keyframe, and the estimates between
                  --  the rounds aligned them for 21 to 115 s each (R5's fifth start, R6's second, 10-14), for a
                  --  measurement of the view's motion per unit that the sweep's first level, searched wide, makes
                  --  itself; nothing else asks the kinematics anything before the sweep.
                  --  Not while a step is under way (Motion.One_Step): the readings of a joint that has begun to move are
                  --  below the step an eye can see in the first beat or two of its push (the sweep's first level is that
                  --  step), the picture is a lag behind them, and the arm held still by that test gave a keyframe whose
                  --  reading was 62 % of the way and whose picture was the one at rest: the base arm of the cold boot b2
                  --  (10-20), channel 2 turned 157 units, the keyframe at 98 showed no motion of the view, its pixels a
                  --  unit were measured at 0.0001 and the joint was never fitted.
                  if R.Sweeping and then not M.Stepping and then Held_Still (M, Arm, Beat)
                    and then E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E)
                  then
                     declare
                        Now   : constant Real_Array := Readings_Of (M, G, Beat);
                        Fresh : Boolean := True;
                     begin
                        --  A view worth matching shows something new: against every
                        --  keyframe, some channel moved by at least the step the
                        --  matcher tells apart (Keyframe_Step: Z of the larger of the
                        --  cells' displacement noise and the aligner's own error, over
                        --  the pixels the eye's picture moves per unit of the channel),
                        --  the step the fit itself judges a keyframe's motion by. The
                        --  step the eye can see at all (Visible_Step), which the lock-in
                        --  tells over many beats and every cell, lies far below what one
                        --  pair of pictures resolves: on the fake body 7 microradians of
                        --  the wrist, a thousandth of a pixel, and a settling arm gave a
                        --  keyframe every other beat, 304 by beat 4224 against 41. Where
                        --  the matcher's step is not measured yet (no keyframe has turned the
                        --  channel alone), the step the eye can see stands in (Step_Of). None
                        --  is taken before either is measured. The second keyframe is the
                        --  reference's still twin, at its pose.
                        declare
                           Steps   : Real_Array (Now'Range) := [others => 0.0];
                           Seeable : Boolean := False;
                           At_Reference : Boolean := False;   --  at the reference's readings, to within the steps
                        begin
                           for C in Now'Range loop
                              Steps (C) := Step_Of (M, Arm, C);
                              Seeable := Seeable or else Steps (C) > 0.0;
                           end loop;
                           for F in R.Frames.First_Index .. R.Frames.Last_Index loop
                              declare
                                 Apart : Boolean := False;
                              begin
                                 for C in Now'Range loop
                                    if Steps (C) > 0.0 and then abs (Now (C) - R.Frames (F).Readings (C - 1)) >= Steps (C) then
                                       Apart := True;
                                    end if;
                                 end loop;
                                 Fresh := Fresh and then Apart;
                                 if F = R.Frames.First_Index then
                                    At_Reference := not Apart;
                                 end if;
                              end;
                           end loop;
                           --  The reference's still twin: the first keyframe after it at its readings, whenever the
                           --  arm stands there again (two pictures of one pose: the matcher's own error, Match_Noise).
                           --  A frame at other readings is no twin, held still or not: the reference's next still beat
                           --  can come after a push (robot.sweep.twin: that frame was aligned as the same pose, and 0
                           --  of 40 points were found).
                           Fresh := Seeable
                             and then (Fresh
                                       or else (not R.Twin_Taken and then not R.Frames.Is_Empty
                                                and then Beat > R.Frames (1).Beat and then At_Reference));
                           if Fresh then
                              declare
                                 K : Keyframe;
                              begin
                                 K.Beat := Beat;
                                 for X of Now loop
                                    K.Readings.Append (X);
                                 end loop;
                                 K.Image := O.Images (E);
                                 K.Asked := R.Asking;
                                 R.Frames.Append (K);
                              end;
                              if At_Reference and then Natural (R.Frames.Length) > 1 then
                                 R.Twin_Taken := True;
                              end if;
                           end if;
                        end;
                        if Fresh then
                           if Natural (R.Frames.Length) = 1 then
                              Query_Points
                                (M, E, G, R.Frames.Last_Element.Image, R.Query_U, R.Query_V, R.Query_Responds);
                              R.Still.Clear;
                              R.Stood_Off.Clear;
                              R.Moved_With.Clear;
                              R.Patch_Noise.Clear;
                              R.Patch_Error.Clear;
                              R.Patch_Stood.Clear;
                              R.Patch_Moving.Clear;
                              R.Patch_Rate := 0.0;
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Observe;

   function Match_Noise (M : Model; A : Arm_Id) return Real is
     (if Index_Of (M, A) > 0 then Match_Sigma (M.Kinematics (Index_Of (M, A))) else 0.0);

   function Keyframe_Step (M : Model; A : Arm_Id; Channel : Positive) return Real is
      E : constant Eye_Id'Base := Eye_Of (M, A);
   begin
      if E = 0 then
         return 0.0;
      end if;
      declare
         Per_Unit : constant Real := Pixels_Per_Unit (M, A, Channel);
         --  The larger of the cells' displacement noise and the matcher's: a
         --  keyframe is judged by the matcher, its view by the cells.
         Noise    : constant Real := Real'Max (Lockin.Cell_Noise (M, E), Match_Noise (M, A));
      begin
         return (if Per_Unit > 0.0 and then Noise < Real'Last then Driver.Conventions.Z * Noise / Per_Unit else 0.0);
      end;
   end Keyframe_Step;

   function Follow_Step (M : Model; A : Arm_Id; Channel : Positive) return Real is
      E : constant Eye_Id'Base := Eye_Of (M, A);
   begin
      if E = 0 then
         return 0.0;
      end if;
      declare
         Per_Unit : constant Real := Pixels_Per_Unit (M, A, Channel);
         Half     : constant Natural :=
           Driver.Alignment.Half_Patch (M.Eyes (E).Grid.Width, M.Eyes (E).Grid.Height);
      begin
         return (if Per_Unit > 0.0 then Real (Half) / Per_Unit else 0.0);
      end;
   end Follow_Step;

   function Step_Of (M : Model; A : Arm_Id; Channel : Positive) return Real is
      G      : constant Group_Id := Arm_Group (M, A);
      Told   : constant Real := Keyframe_Step (M, A, Channel);
      Seen   : constant Estimate := Visible_Step (M, G, Channel);
      Jitter : constant Real := Channels.Noise (M, G, Channel);
      --  And no nearer than the channel's dead band (Steps.Dead_Band, the least turn the body is known to execute
      --  where nothing narrower was measured): a servo lands within its dead band of a target it is sent back
      --  to, and a frame that differs from another by less than that in a channel is the same asked pose,
      --  landed elsewhere, not a turn of the joint. The
      --  kiwi's returns to its reference landed two or three units off it (R6's sixth start, 10-14), beyond the
      --  matcher's step of 1.9, and every one was a keyframe of the arm with one joint turned by a landing error.
      Least  : constant Real := Steps.Dead_Band (M, G, Channel);
      function Floored (Step : Real) return Real is
        (if Least < Real'Last then Real'Max (Step, Least) else Step);
   begin
      if Told > 0.0 then
         --  And no nearer than the readings tell apart (Z of the noise of a
         --  difference of two readings, as Visible_Step has it): a matcher
         --  that errs by a thousandth of a pixel gives a step below the
         --  readings' own jitter, and an arm at rest gave a keyframe every
         --  thirty-seven beats at one pose (fake body Z9, arm 1: 295 of
         --  them, each aligned for nothing).
         return Floored (if Jitter < Real'Last
                         then Real'Max (Told, Driver.Conventions.Z * Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0) * Jitter)
                         else Told);
      elsif Known (Seen) then
         return Floored (Seen.Value);
      else
         --  Neither the keyframes nor the lock-in tell a step yet: still none nearer than the readings tell apart and
         --  the body is known to execute, as the steps above are floored. A step of nought made the least turn a
         --  keyframe counts nought (Motion.Least_Turn), and the joint's ladder began below its dead band and was
         --  never followed: t502 of the race (10-23), its channel 4 (dead band 1.78, noise 1.15, the eye's step
         --  unknown) had no keyframe of its own, its order untold, the chain fitted wrong; with every quirk of the
         --  fake off, the same.
         return Floored (if Jitter < Real'Last
                         then Driver.Conventions.Z * Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0) * Jitter
                         else 0.0);
      end if;
   end Step_Of;

   --  f15c on 10-15, arm 2: the joints not asked stood 4 and 8 units off the reference at a dead band of 4,
   --  against a step of 4.
   function Counted_Turn (M : Model; A : Arm_Id; Channel : Positive) return Real is
      Step : constant Real := Step_Of (M, A, Channel);
      Band : constant Real := Steps.Dead_Band (M, Arm_Group (M, A), Channel);
   begin
      return (if Band < Real'Last then Real'Max (Step, 2.0 * Band) else Step);
   end Counted_Turn;

   function Twin_Answered (M : Model; A : Arm_Id) return Boolean is
   begin
      if Index_Of (M, A) = 0 then
         return False;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (Index_Of (M, A));
      begin
         return R.Twin_Taken;
      end;
   end Twin_Answered;

   procedure Still_Blocked_By (M : Model; A : Arm_Id; Cause, Detail : out Ada.Strings.Unbounded.Unbounded_String) is
      use Ada.Strings.Unbounded;
      E    : constant Eye_Id'Base := Eye_Of (M, A);
      G    : constant Group_Id := Arm_Group (M, A);
      Beat : constant Natural := (if M.Beats > 0 then M.Beats - 1 else 0);
      --  The span Held_Still holds the readings still over.
      Span : constant Positive := 1 + (if M.Groups (G).Delay_Known then M.Groups (G).Delay_Beats else 0)
        + (if E = 0 then 0 else Natural'Max (0, Image_Lag (M, E)));

      --  The channels of the group that changed at B, with the change against the noise and the step they are held to.
      function Changed_At (B : Natural) return String is
         Text : Unbounded_String;
      begin
         for C in 1 .. M.Groups (G).Size loop
            declare
               D : constant Real := Channels.Change (M, G, B, C);
               V : constant Estimate := Visible_Step (M, G, C);
            begin
               if D /= 0.0 then
                  Append (Text, " channel" & C'Image & " changed by " & Driver.Log.Image (D, 2) & " (its noise "
                          & (if Channels.Noise (M, G, C) < Real'Last then Driver.Log.Image (Channels.Noise (M, G, C), 2)
                             else "unmeasured")
                          & ", the step an eye sees it make "
                          & (if Known (V) then Driver.Log.Image (V.Value, 2) else "unknown") & ");");
               end if;
            end;
         end loop;
         return To_String (Text);
      end Changed_At;
   begin
      Cause := Null_Unbounded_String;
      Detail := Null_Unbounded_String;
      if E = 0 then
         Cause := To_Unbounded_String ("the arm has no eye");
         Detail := Cause;
         return;
      end if;
      if Beat < Span then
         Cause := To_Unbounded_String ("too early in the run to judge");
         Detail := Cause;
         return;
      end if;
      if not Stillness.Eye_Settled (M, E) then
         Append (Cause, " its eye's picture is not settled;");
         Append (Detail, " its eye's picture is not settled;");
      end if;
      for B in Beat - Span + 1 .. Beat loop
         if not Channels.Has_Reading (M, G, B) or else not Channels.Has_Reading (M, G, B - 1) then
            Append (Cause, " a beat of the span has no reading;");
            Append (Detail, " a beat of the span has no reading;");
            exit;
         elsif Channels.Moving (M, G, B) then
            Append (Cause, " the readings move;");
            Append (Detail, " the readings move at beat" & B'Image & ":" & Changed_At (B));
            exit;
         end if;
      end loop;
      if Length (Cause) = 0 then
         declare
            Now   : constant Real_Array := Readings_Of (M, G, Beat);
            Since : constant Real_Array := Readings_Of (M, G, Beat - Span);
            D     : Real_Array (1 .. Now'Length);
         begin
            for C in D'Range loop
               D (C) := Now (Now'First + C - 1) - Since (Since'First + C - 1) - Channels.Drift (M, G, C) * Real (Span);
            end loop;
            if Channels.Visible (M, G, D) then
               Append (Cause, " the readings changed over the span as a whole;");
               Append (Detail, " the readings changed over the" & Span'Image & " beats of the span as a whole:");
               for C in D'Range loop
                  if D (C) /= 0.0 then
                     Append (Detail, " channel" & C'Image & " by " & Driver.Log.Image (D (C), 2) & ";");
                  end if;
               end loop;
            end if;
         end;
      end if;
   end Still_Blocked_By;

   procedure Twin_Blocked_By (M : Model; A : Arm_Id; Cause, Detail : out Ada.Strings.Unbounded.Unbounded_String) is
      use Ada.Strings.Unbounded;
      K : constant Natural := Index_Of (M, A);
      E : constant Eye_Id'Base := Eye_Of (M, A);
      G : constant Group_Id := Arm_Group (M, A);
   begin
      Cause := Null_Unbounded_String;
      Detail := Null_Unbounded_String;
      if K = 0 or else E = 0 or else M.Beats = 0 then
         --  The evidence of an arm is current while the graph still gives the arm its group and its eye (Current).
         Cause := To_Unbounded_String
           (if M.Beats = 0 then "no beat yet"
            elsif E = 0 then "the graph gives the arm no eye that it carries"
            else "the arm has no evidence that is current in the graph (the arm's group or its eye changed since its sweep began)");
         Detail := Cause & To_Unbounded_String
           (": the graph's group of the arm" & Arm_Group (M, A)'Image & ", its eye" & E'Image & ", the evidence kept for"
            & Natural'Image (Natural (M.Kinematics.Length)) & " arms");
         return;
      end if;
      declare
         R    : Arm_Evidence renames M.Kinematics (K);
         Beat : constant Natural := M.Beats - 1;
      begin
         if R.Frames.Is_Empty then
            Cause := To_Unbounded_String ("no reference keyframe yet");
            Detail := Cause;
            return;
         end if;
         Still_Blocked_By (M, A, Cause, Detail);
         declare
            Now : constant Real_Array := Readings_Of (M, G, Beat);
         begin
            for C in Now'Range loop
               declare
                  Step : constant Real := Step_Of (M, A, C);
                  Off  : constant Real := Now (C) - R.Frames (R.Frames.First_Index).Readings (C - 1);
               begin
                  if Step > 0.0 and then abs Off >= Step then
                     Append (Cause, " channel" & C'Image & " stands off the reference's reading;");
                     Append (Detail, " channel" & C'Image & " stands" & Driver.Log.Image (Off, 2)
                             & " off the reference's reading, a step of" & Driver.Log.Image (Step, 2) & ";");
                  end if;
               end;
            end loop;
         end;
         if Length (Cause) = 0 then
            Cause := To_Unbounded_String ("still, at the reference, and no keyframe taken for it yet");
            Detail := Cause;
         end if;
      end;
   end Twin_Blocked_By;

   --  The boot's word to the kinematics goes into the recording (kinds G and Q) where it was said, so a replay
   --  says it to its models at the same point (Driver.Robot.Replay_Sweep): a replay runs no boot, and without
   --  the word no keyframe is taken (R6's seventh start, 10-14: the recording of a sweep replayed into nothing).
   procedure Asking (M : in out Model; A : Arm_Id; Channels : Channel_Flags) is
      K : constant Natural := Index_Of (M, A);
      Text : Ada.Strings.Unbounded.Unbounded_String :=
        Ada.Strings.Unbounded.To_Unbounded_String ("arm" & A'Image & " asked");
   begin
      if K > 0 then
         M.Kinematics (K).Asking.Clear;
         for F of Channels loop
            M.Kinematics (K).Asking.Append (F);
            Ada.Strings.Unbounded.Append (Text, (if F then " 1" else " 0"));
         end loop;
      end if;
      Driver.Recording.Write_Shared
        (Driver.Recording.Channels_Asked, Driver.Bytes.To_Bytes (Ada.Strings.Unbounded.To_String (Text)));
   end Asking;

   --  The channels the sweep asked a keyframe's joints to turn for, for the log: ", asked c2 c4"; nothing when none.
   function Asked_Image (K : Keyframe) return String is
      Text : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for C in 0 .. Natural (K.Asked.Length) - 1 loop
         if K.Asked (C) then
            Ada.Strings.Unbounded.Append (Text, " c" & Ada.Strings.Fixed.Trim (Natural'Image (C + 1), Ada.Strings.Left));
         end if;
      end loop;
      return (if Ada.Strings.Unbounded.Length (Text) = 0 then ""
              else ", asked" & Ada.Strings.Unbounded.To_String (Text));
   end Asked_Image;

   procedure Begin_Anew (M : in out Model; A : Arm_Id) is
      K : constant Natural := Index_Of (M, A);
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Sweep_Begun, Driver.Bytes.To_Bytes ("arm" & A'Image));
      --  An arm met here for the first time (no beat has taken a keyframe of it: none is taken before its sweep)
      --  starts its evidence now, the sweep's.
      if K = 0 then
         if Eye_Of (M, A) > 0 then
            M.Kinematics.Append (Arm_Evidence'(Arm => A, Group => Arm_Group (M, A), Eye => Eye_Of (M, A),
                                               Sweeping => True, others => <>));
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & A'Image & " begins at beat" & M.Beats'Image
                             & ": its sweep's keyframes are its first");
         end if;
         return;
      end if;
      if K > 0 then
         declare
            R : Arm_Evidence renames M.Kinematics (K);
         begin
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & A'Image & " begins anew at beat" & M.Beats'Image
                             & ", dropping" & R.Frames.Length'Image & " keyframes, keeping what they measured of how far"
                             & " the view moves per unit");
            --  What the dropped keyframes measured of the view's motion per unit holds at any reference: it is
            --  kept, without the keyframes it came from, until the new keyframes measure it again.
            declare
               Kept   : constant Real_Vectors.Vector := R.Shifts;
               Sigmas : constant Real_Vectors.Vector := R.Shift_Sigmas;
               Points : constant Count_Vectors.Vector := R.Shift_Points;
               Seen   : constant Flag_Vectors.Vector := R.Shift_Seen;
               Told   : constant Count_Vectors.Vector := R.Shift_Told;
               Error  : constant Real := Match_Sigma (R);
            begin
               M.Kinematics.Replace_Element
                 (K, (Arm => R.Arm, Group => R.Group, Eye => R.Eye, Shifts => Kept, Shift_Sigmas => Sigmas,
                      Shift_Points => Points, Shift_Seen => Seen, Shift_Told => Told, Scatter => Error,
                      Sweeping => True, others => <>));
            end;
         end;
      end if;
   end Begin_Anew;

   function Matched (M : Model; A : Arm_Id) return Natural is
     (if Index_Of (M, A) > 0 then Natural (M.Kinematics (Index_Of (M, A)).Matches.Length) else 0);

   ---------------------------------------------------------------------------
   --  Aligning the keyframes

   package Keyframes renames Driver.Alignment.Keyframes;

   use type Driver.Alignment.Verdict;
   use type Keyframes.Source;

   type Prediction_Access is access Keyframes.Prediction_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Prediction_Array, Prediction_Access);
   type Alignment_Access is access Keyframes.Answer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Answer_Array, Alignment_Access);
   type Keyframe_Point_Access is access Keyframes.Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Point_Array, Keyframe_Point_Access);
   type Agreement_Access is access Keyframes.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Flag_Array, Agreement_Access);

   --  The lens of a fit, as Fit has it.
   function Fit_Lens (L : Lens_Fit) return Fit.Lens is
     ((Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2));

   type Pixel_Derivative is array (1 .. 2) of Real;   --  of the pixel's u and v

   --  What a fitted arm's offer states of a point it does not predict (Offer_Of: one its fit gave no depth): that it
   --  may lie anywhere.
   Unpredicted : constant Real := Real'Last / 4.0;

   --  The fitted arm's own prediction of where its reference's query points are in a keyframe at those readings.
   --  A query the fit gave a depth is a point in the reference eye's frame; the eye at the readings (the fit's
   --  joints) sees it where the lens puts it. The prediction's uncertainty is the fit's whole covariance carried to
   --  the pixel: each term moves the eye at those readings, the lens, and the point's depth by its gain together,
   --  and the depth's own uncertainty with the terms held is added. Taken apart, the turn's uncertainty and the
   --  place's (the lens left out, and what each depth shares with the terms) stated three times the errors: u25 (servo
   --  seed 3 of the fake robot, 10-19), replayed, the arm's points lay 0.51 px from the truth on the median of an
   --  axis and stated 2.26 px, the error over it 0.20 on the median for the Gaussian's 0.67, and 12 of the 29
   --  keyframes the arm predicted found no point. Carried whole: 0.60 px, 0.67 on the median (2.31 and 3.55 at the
   --  90th and 99th percentiles, for 1.64 and 2.58), 3 of 94 keyframes found none, and to beat 10373 its keyframes
   --  found a quarter more sightings, as close to the truth (0.84 px on the median, 3.4 % beyond 2 px, against 0.90
   --  and 3.6 %). The warp is the plane's homography, the table's for a track that lies on it
   --  and else the plane across the line of sight at the track's depth. A point without a depth is not
   --  predicted: its uncertainty is unbounded. Known is False when the arm is not fitted.
   procedure Offer_Of
     (M : Model; R : Arm_Evidence; Readings : Real_Array; Into : out Keyframes.Prediction_Array; Known : out Boolean)
   is
      use Driver.Numerics.Arrays;
      Queries   : constant Natural := Natural (R.Query_U.Length);
      Lens      : constant Fit.Lens := Fit_Lens (R.Result.Lens);
      Pose      : constant Rigid := Eye_In_Reference (M, R.Arm, Readings);
      Step      : constant Real := Ada.Numerics.Long_Elementary_Functions."**" (Real'Model_Epsilon, 1.0 / 3.0);

      --  Where the eye stands at the readings under each order of the joints the fit does not tell from its own
      --  (Arm_Fit.Untold_Orders), its joints as fitted: a point is as uncertain as the farthest of them puts it. None
      --  where the arm is not fitted: its joints are what a fit that failed left, and taken at a keyframe they stopped
      --  the estimates (t103, seed 103 of the default fake robot on the big machine, 10-22: arm 1 not fitted, its
      --  focal length 53 +- 38 px, six orders untold; the next keyframe's offer raised ARGUMENT_ERROR in Eye_At).
      Size      : constant Natural := Readings'Length;
      Orders    : constant Natural :=
        (if R.Result.Fitted and then not R.Result.Order_Told and then Size > 0
           and then Natural (R.Result.Joints.Length) = Size and then Natural (R.Result.Reference.Length) = Size
         then Natural (R.Result.Untold_Orders.Length) / Size else 0);
      type Pose_List is array (Positive range <>) of Rigid;
      function Untold_Poses return Pose_List is
         Joints : Fit.Joint_Array (1 .. Size);
         Change : Real_Array (1 .. Size);
         Poses  : Pose_List (1 .. Orders);
      begin
         if Orders = 0 then
            return Poses;
         end if;
         for J in Joints'Range loop
            declare
               F : constant Joint_Fit := R.Result.Joints (J);
            begin
               Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
               Change (J) := Readings (Readings'First + J - 1) - R.Result.Reference (J - 1);
            end;
         end loop;
         for K in 1 .. Orders loop
            declare
               C : Fit.Chain_Order (1 .. Size);
            begin
               for J in 1 .. Size loop
                  C (J) := Positive (R.Result.Untold_Orders (R.Result.Untold_Orders.First_Index + (K - 1) * Size + J - 1));
               end loop;
               Poses (K) := Fit.Eye_At (Joints, C, Change);
            end;
         end loop;
         return Poses;
      end Untold_Poses;
      Others_At : constant Pose_List := Untold_Poses;

      --  Each term of the fit (Fit.Move_Term) moved by its standard deviation either way, and the lens and the eye at
      --  the readings so moved: a point moves with a term through both, and through its depth by its gain.
      Whole     : constant Natural := Fit.Terms (Size);
      Covariant : constant Boolean :=
        R.Result.Fitted and then Natural (R.Result.Covariance.Length) = Whole * Whole
        and then Natural (R.Result.Joints.Length) = Size and then Natural (R.Result.Reference.Length) = Size
        and then Natural (R.Result.Chain.Length) = Size and then Natural (R.Result.Track_Gains.Length) >= Queries * Whole;
      function Term_Covariance (P, Q : Positive) return Real is
        (R.Result.Covariance (R.Result.Covariance.First_Index + (P - 1) * Whole + Q - 1));
      type Moved_Term is record
         Sigma              : Real := 0.0;
         Up_Lens, Down_Lens : Fit.Lens;
         Up_Eye, Down_Eye   : Rigid := Identity;
      end record;
      type Moved_Terms is array (Positive range <>) of Moved_Term;
      function Moves_Of return Moved_Terms is
         Moves  : Moved_Terms (1 .. (if Covariant then Whole else 0));
         Joints : Fit.Joint_Array (1 .. Size);
         Change : Real_Array (1 .. Size);
      begin
         if Moves'Length = 0 then
            return Moves;
         end if;
         for J in Joints'Range loop
            declare
               F : constant Joint_Fit := R.Result.Joints (J);
            begin
               Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
               Change (J) := Readings (Readings'First + J - 1) - R.Result.Reference (J - 1);
            end;
         end loop;
         for P in Moves'Range loop
            if Term_Covariance (P, P) > 0.0 then
               declare
                  Sigma        : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Term_Covariance (P, P));
                  Up, Down     : Fit.Joint_Array := Joints;
                  Up_L, Down_L : Fit.Lens := Lens;
               begin
                  Fit.Move_Term (Up, Up_L, P, Sigma);
                  Fit.Move_Term (Down, Down_L, P, -Sigma);
                  Moves (P) := (Sigma => Sigma, Up_Lens => Up_L, Down_Lens => Down_L,
                                Up_Eye => Fit.Eye_At (Up, Chain_Of (R.Result), Change),
                                Down_Eye => Fit.Eye_At (Down, Chain_Of (R.Result), Change));
               end;
            end if;
         end loop;
         return Moves;
      end Moves_Of;
      Moves : constant Moved_Terms := Moves_Of;

      --  Where the point X of the reference eye's frame lands when the eye's turn is moved by Omega (a rotation
      --  vector, in the reference frame) and its place by Shift.
      procedure Land (X, Omega, Shift : Vec3; Where : out Driver.Images.Pixel; Front : out Boolean) is
         Moved : constant Rigid :=
           (Rotation => Driver.Numerics.Exp (Omega) * Pose.Rotation, Translation => Pose.Translation + Shift);
      begin
         Fit.Project (Lens, Inverse (Moved) * X, Where.U, Where.V, Front);
      end Land;

      --  Where the plane A (the line of sight H meets it at depth 1 / (A * H)) carries the reference pixel.
      procedure Through_Plane (A : Vec3; U0, V0 : Real; Where : out Driver.Images.Pixel; Ok : out Boolean) is
         H     : constant Vec3 := Fit.Ray (Lens, U0, V0);
         Slope : constant Real := A * H;
      begin
         Where := (U => 0.0, V => 0.0);
         Ok := Slope > 0.0;
         if Ok then
            Land ((1.0 / Slope) * H, Zero3, Zero3, Where, Ok);
         end if;
      end Through_Plane;
   begin
      Known := R.Result.Fitted and then Natural (R.Result.Track_Known.Length) >= Queries
        and then Natural (R.Result.Tracks.Length) >= 3 * Queries;
      for K in Into'Range loop
         declare
            Q  : constant Natural := K - Into'First;
            U0 : constant Real := (if Q < Queries then R.Query_U (Q) else 0.0);
            V0 : constant Real := (if Q < Queries then R.Query_V (Q) else 0.0);
         begin
            Into (K) := (From => (U => U0, V => V0), To => (U => U0, V => V0),
                         Cov => (UU => Unpredicted, UV => 0.0, VV => Unpredicted), others => <>);
            if Known and then Q < Queries and then R.Result.Track_Known (Q)
              and then Q < Natural (R.Result.Track_Sigmas.Length)
              and then R.Result.Track_Sigmas (Q) < Unpredicted and then Covariant
            then
               declare
                  X     : constant Vec3 :=
                    [R.Result.Tracks (3 * Q), R.Result.Tracks (3 * Q + 1), R.Result.Tracks (3 * Q + 2)];
                  Here  : Driver.Images.Pixel;
                  Front : Boolean;
               begin
                  Land (X, Zero3, Zero3, Here, Front);
                  if Front then
                     declare
                        --  How the pixel moves with each term, and with the point's own depth (its logarithm).
                        Rate  : array (Moves'Range) of Pixel_Derivative := [others => [0.0, 0.0]];
                        Along : Pixel_Derivative;
                        Plane : constant Vec3 :=
                          (if Q < Natural (R.Result.Table_On.Length) and then R.Result.Table_On (Q)
                              and then R.Result.Table_A * X > 0.0
                           then (1.0 / (R.Result.Table_A * X)) * R.Result.Table_A
                           else [0.0, 0.0, 1.0 / X (3)]);
                        P1, P2 : Driver.Images.Pixel;
                        Ok1, Ok2 : Boolean;
                        Variance : array (1 .. 3) of Real := [others => 0.0];   --  UU, UV, VV
                     begin
                        for P in Moves'Range loop
                           if Moves (P).Sigma > 0.0 then
                              declare
                                 Gain     : constant Real :=
                                   R.Result.Track_Gains (R.Result.Track_Gains.First_Index + Q * Whole + P - 1);
                                 Width    : constant Real := Moves (P).Sigma + Moves (P).Sigma;
                                 Up, Down : Driver.Images.Pixel;
                                 F1, F2   : Boolean;
                              begin
                                 Fit.Project (Moves (P).Up_Lens, Inverse (Moves (P).Up_Eye)
                                              * ((X (3) * Ada.Numerics.Long_Elementary_Functions.Exp (Moves (P).Sigma * Gain))
                                                 * Fit.Ray (Moves (P).Up_Lens, U0, V0)), Up.U, Up.V, F1);
                                 Fit.Project (Moves (P).Down_Lens, Inverse (Moves (P).Down_Eye)
                                              * ((X (3) * Ada.Numerics.Long_Elementary_Functions.Exp (-Moves (P).Sigma * Gain))
                                                 * Fit.Ray (Moves (P).Down_Lens, U0, V0)), Down.U, Down.V, F2);
                                 Rate (P) := [(Up.U - Down.U) / Width, (Up.V - Down.V) / Width];
                              end;
                           end if;
                        end loop;
                        declare
                           Up, Down : Driver.Images.Pixel;
                           F1, F2   : Boolean;
                        begin
                           Land ((1.0 + Step) * X, Zero3, Zero3, Up, F1);
                           Land ((1.0 - Step) * X, Zero3, Zero3, Down, F2);
                           Along := [(Up.U - Down.U) / (2.0 * Step), (Up.V - Down.V) / (2.0 * Step)];
                        end;
                        for P in Moves'Range loop
                           for P2 in Moves'Range loop
                              declare
                                 C : constant Real := Term_Covariance (P, P2);
                              begin
                                 Variance := [Variance (1) + Rate (P) (1) * C * Rate (P2) (1),
                                              Variance (2) + Rate (P) (1) * C * Rate (P2) (2),
                                              Variance (3) + Rate (P) (2) * C * Rate (P2) (2)];
                              end;
                           end loop;
                        end loop;
                        Variance := [Variance (1) + (Along (1) * R.Result.Track_Sigmas (Q)) ** 2,
                                     Variance (2) + Along (1) * Along (2) * R.Result.Track_Sigmas (Q) ** 2,
                                     Variance (3) + (Along (2) * R.Result.Track_Sigmas (Q)) ** 2];
                        --  The order the fit does not tell: the farthest place an untold order puts the point, as the
                        --  outer product of its difference from this one over the window's multiple, squared: the
                        --  window then reaches that place at its edge, and the prediction's own uncertainty beside it.
                        --  The truth is at one of the orders' places, not spread about them: taken whole, as
                        --  Keyframes.Predict widens a candidate by its largest disagreement with another, the window
                        --  reached the window's multiple of the difference beyond it, and a keyframe the untold orders
                        --  put apart by more than two pixels was searched wide, however well the fitted arm predicted
                        --  it: u6 (servo seed 3 of the fake robot, 10-19), its pairs turned to tell its untold orders,
                        --  c1 with c3 and c1 with c4 predicted within 0.39 to 0.47 px of the truth on the median, their
                        --  windows stating 10 to 16 px, and no point of them was followed. An order that puts the
                        --  point behind the eye shows it nowhere to search. A keyframe that turns two joints the fit does not order is searched where
                        --  either order puts its points, and the order told after it is the one the picture shows, not
                        --  the one it was predicted from: z7 (servo seed 7 of the fake robot, 10-18) turned c1 with c2
                        --  under the best order 4 5 2 1 3 6, each point asked within 1.8 to 6 px of where that order put
                        --  it; two of its seven keyframes found points there, the rest none, and the fit told 4 5 2 1 3 6
                        --  on them for the true 4 5 1 2 3 6.
                        declare
                           Far_U, Far_V : Real := 0.0;
                        begin
                           for Other of Others_At loop
                              declare
                                 There : Driver.Images.Pixel;
                                 Ahead : Boolean;
                              begin
                                 Fit.Project (Lens, Inverse (Other) * X, There.U, There.V, Ahead);
                                 if Ahead and then (There.U - Here.U) ** 2 + (There.V - Here.V) ** 2 > Far_U ** 2 + Far_V ** 2
                                 then
                                    Far_U := There.U - Here.U;
                                    Far_V := There.V - Here.V;
                                 end if;
                              end;
                           end loop;
                           Far_U := Far_U / Driver.Alignment.Window_Multiple;
                           Far_V := Far_V / Driver.Alignment.Window_Multiple;
                           Variance := [Variance (1) + Far_U ** 2, Variance (2) + Far_U * Far_V, Variance (3) + Far_V ** 2];
                        end;
                        Into (K).To := Here;
                        Into (K).Cov := (UU => Variance (1), UV => Variance (2), VV => Variance (3));
                        Through_Plane (Plane, U0, V0, Here, Ok1);
                        Through_Plane (Plane, U0 + 1.0, V0, P1, Ok1);
                        Through_Plane (Plane, U0, V0 + 1.0, P2, Ok2);
                        if Ok1 and then Ok2 then
                           Into (K).Linear := (UU => P1.U - Here.U, UV => P2.U - Here.U,
                                               VU => P1.V - Here.V, VV => P2.V - Here.V);
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Offer_Of;

   --  The median standard deviation, over the points, that predictions state: the larger axis of each.
   --  The view's common motion a keyframe from a new direction was asked from (Keyframes.Add), for its line.
   function Motion_Image (Common : Keyframes.Motion) return String is
     (if Common.Told and then Common.Set_Aside
      then ", the view's common motion set aside (none at size" & Common.Level'Image & ", every one of its"
           & Common.Own'Image & " points where it stood, against a prediction that moves them): asked as predicted"
      elsif Common.Told
      then " from the view's common motion at size"
           & Common.Level'Image & ": turned "
           & Driver.Log.Image (Common.Turn * 180.0 / Ada.Numerics.Pi, 1) & " deg, its centre moved "
           & Driver.Log.Image (Common.Shift_U, 1) & " " & Driver.Log.Image (Common.Shift_V, 1) & " px +- "
           & Driver.Log.Image (Common.Sigma, 2) & ";" & Common.Agree'Image & " of" & Common.Points'Image
           & " points on it," & Common.Own'Image & " on it alone against" & Common.Other'Image & " on the next best,"
           & Common.Own_Places'Image & " at a place of their own on it;" & Common.Draws'Image & " draws" & ";"
           & Common.Unmoved'Image & " where they stood"
      else ", no common motion of the view told (at size" & Common.Level'Image & "," & Common.Own'Image
           & " points on the best field alone against" & Common.Other'Image & " on the next, of"
           & Common.Points'Image & ";" & Common.Draws'Image & " draws;" & Common.Unmoved'Image & " where they stood)");

   --  The median standard deviation, over the points a prediction places, that it states: the larger axis of each.
   --  A fitted arm's offer places only the points its fit gave a depth (Offer_Of: the rest Unpredicted). Taken over
   --  every point, an arm whose fit gave fewer than half of its points a depth stated the unbounded at every keyframe,
   --  and no keyframe was ever aligned again from it (Realign_Arms, Narrowed): e20x_20 (seed 20 of the default fake
   --  robot, 10-22), 115 of its 255 points with a depth, none of its 89 keyframes aligned again, and 29 of the 43
   --  after its first fit kept 5 sightings or fewer; e20v_26 (seed 26, 176 of 255 with a depth) aligned 45 again.
   --  Where the prediction places no point, what an unplaced one states.
   function Stated_By (Given : Keyframes.Prediction_Array) return Real is
      Sigmas : Real_Access := new Real_Array (1 .. Given'Length);
      Placed : Natural := 0;
   begin
      for K in Given'Range loop
         if Real'Max (Given (K).Cov.UU, Given (K).Cov.VV) < Unpredicted then
            Placed := Placed + 1;
            Sigmas (Placed) :=
              Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (Given (K).Cov.UU, Given (K).Cov.VV));
         end if;
      end loop;
      return Result : constant Real :=
        (if Given'Length = 0 then 0.0
         elsif Placed = 0 then Ada.Numerics.Long_Elementary_Functions.Sqrt (Unpredicted)
         else Driver.Stats.Median (Sigmas (1 .. Placed)))
      do
         Free (Sigmas);
      end return;
   end Stated_By;

   --  What a keyframe's answers say of where the reference's query points went: the answers found that agree with the
   --  keyframe's field of displacements (a wrong answer is as sure of itself as a right one, and is told from it by
   --  the others), but those whose uncertainty is more than Z times the keyframe's typical one (the median of those:
   --  a point that tells its place no better than that is not a sighting the fit can take as one among the rest).
   --  The keyframe is one to rest a fit on only when the typical answer of it came from a window no wider than a
   --  patch (Narrow): a wider one is searched where several places a patch cannot tell apart may lie in it, the
   --  answer rests on the prediction as much as on the pixels, and a keyframe whose answers do is a move the
   --  prediction had not followed. Its few answers from narrower windows are no sightings either: the fit takes the
   --  widest keyframe of a joint for the one that tells its axis best, and a keyframe of a handful of points is no
   --  such. The points refused for the prediction's uncertainty, those found that do not agree and those a wide
   --  window found are Wide: the prediction may have been what failed them, and a better one may find them.
   --  Measures how far the eye's view moves per reading unit of every channel from the arm's aligned keyframes
   --  (Arm_Evidence.Shifts). The channels whose change varies over the keyframes by more than the readings' noise
   --  tells are the regressors; for every query point, its displacements in the keyframes it was answered in, from
   --  windows of any width, are regressed on those keyframes' changes (Regression.Solve, twice, for u and v: robust,
   --  so a wrong answer in one keyframe keeps little weight, and by a pseudo-inverse where channels moved only
   --  together), every row scaled by its answer's standard deviation (the least a residual can have) so that the
   --  answers weigh by their certainty: the point's motion per unit of each channel, a vector, with the variance the
   --  regression gives it. A channel is measured by the median over the points of the size of that vector, with the
   --  standard error of that median from the median of the points' standard deviations (the points err apart: the
   --  answers' own errors); a point answered in fewer keyframes than there are regressors says nothing, and a
   --  channel that moved only with another keeps a variance that says it is not told apart. A keyframe need not
   --  turn one channel alone: a real arm comes back a few units off after every push, so no keyframe does; and no
   --  intercept is fitted: at the reference's readings the points are where the reference has them.
   procedure Measure_Shifts (M : Model; R : in out Arm_Evidence) is
      use Ada.Numerics.Long_Elementary_Functions;
      use Driver.Numerics.Arrays;
      N       : constant Natural := M.Groups (R.Group).Size;
      Queries : constant Natural := Natural (R.Query_U.Length);
      Noise   : Real_Array (1 .. N);
      Active  : array (1 .. N) of Boolean := [others => False];
      K       : Natural := 0;   --  the channels the keyframes moved

      function Change (F : Positive; C : Positive) return Real is
        (R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1));

      --  The point was answered in the keyframe: from a window of any width where the set records that
      --  (Match_Set.Answered, the aligner's sets), else where it records an answer a fit rests on (Found: sets
      --  made elsewhere, which hold no wider answers).
      function Answered (S : Match_Set; P : Natural) return Boolean is
        (S.Frame <= Natural (R.Frames.Length)
         and then (if P < Natural (S.Answered.Length) then S.Answered (P)
                   else P < Natural (S.Found.Length) and then S.Found (P)));

      --  The standard deviation an answer states; an answer stating none (an instrument's, a test's) weighs as
      --  one with a pixel's, and the regression's own residual scale stands above it.
      function Sigma_Of (S : Match_Set; P : Natural) return Real is
        (if P < Natural (S.Sigma.Length) and then S.Sigma (P) > 0.0 then S.Sigma (P) else 1.0);
   begin
      --  What was measured before stands for every channel the keyframes do not determine now: the chain that began
      --  anew (Begin_Anew) has its measurements from the keyframes it dropped until its own keyframes measure them.
      while Natural (R.Shifts.Length) < N loop
         R.Shifts.Append (0.0);
      end loop;
      while Natural (R.Shift_Sigmas.Length) < N loop
         R.Shift_Sigmas.Append (Real'Last);
      end loop;
      while Natural (R.Shift_Points.Length) < N loop
         R.Shift_Points.Append (0);
      end loop;
      while Natural (R.Shift_Seen.Length) < N loop
         R.Shift_Seen.Append (True);
      end loop;
      R.Point_Gu.Clear;
      R.Point_Gv.Clear;
      R.Point_G_Told.Clear;
      for Slot_Index in 1 .. N * Queries loop
         R.Point_Gu.Append (0.0);
         R.Point_Gv.Append (0.0);
         R.Point_G_Told.Append (False);
      end loop;
      while Natural (R.Shift_Told.Length) < N loop
         R.Shift_Told.Append (0);
      end loop;
      if Queries = 0 or else R.Matches.Is_Empty or else R.Frames.Is_Empty then
         return;
      end if;
      for C in 1 .. N loop
         Noise (C) := Channels.Noise (M, R.Group, C);
      end loop;
      for C in 1 .. N loop
         declare
            Low, High : Real := 0.0;   --  the reference itself is at zero
         begin
            for S of R.Matches loop
               if S.Frame <= Natural (R.Frames.Length) then
                  Low := Real'Min (Low, Change (S.Frame, C));
                  High := Real'Max (High, Change (S.Frame, C));
               end if;
            end loop;
            --  While the readings' noise is not measured, any variation counts: a column of jitter alone is told
            --  by its rank and variance below.
            Active (C) := (if Noise (C) < Real'Last then Driver.Uncertain.Significant (High - Low, Noise (C) * Sqrt (2.0))
                           else High > Low);
         end;
      end loop;
      for C in 1 .. N loop
         if Active (C) then
            K := K + 1;
         end if;
      end loop;
      if K = 0 then
         return;
      end if;
      --  A channel is measured only over keyframes that moved it, two at the least: one keyframe, predicted from
      --  nothing and searched as wide as the aligner looks, can be answered by a false peak at many points at once
      --  (R5 on the Pi, 10-13: arm_b's third channel, one keyframe of 31 units answered at 58 points, gave 5.07
      --  px a unit against 0.59, and the next keyframe of it, predicted from that, found nothing); a second
      --  keyframe that moved the channel either agrees with it in the regression or spreads it beyond significance.
      declare
         Moving : array (1 .. N) of Natural := [others => 0];   --  per channel: the keyframes that moved it
      begin
         for S of R.Matches loop
            if S.Frame <= Natural (R.Frames.Length) then
               for C in 1 .. N loop
                  if Active (C) and then Noise (C) < Real'Last
                    and then Driver.Uncertain.Significant (Change (S.Frame, C), Noise (C) * Sqrt (2.0))
                  then
                     Moving (C) := Moving (C) + 1;
                  elsif Active (C) and then Noise (C) >= Real'Last and then Change (S.Frame, C) /= 0.0 then
                     Moving (C) := Moving (C) + 1;
                  end if;
               end loop;
            end if;
         end loop;
         for C in 1 .. N loop
            Active (C) := Active (C) and then Moving (C) >= 2;
         end loop;
      end;
      K := 0;
      for C in 1 .. N loop
         if Active (C) then
            K := K + 1;
         end if;
      end loop;
      if K = 0 then
         return;
      end if;
      declare
         --  Per point and channel: the size of its motion per unit, its standard deviation, and whether the point's
         --  keyframes tell it (Regression.Test_Block: a channel that moved only with another in this point's
         --  keyframes is not told apart from it, its coefficient's block has no rank, and the pseudo-inverse's
         --  variance says nothing of the direction it leaves free).
         Slope  : Real_Access := new Real_Array (1 .. N * Queries);
         Spread : Real_Access := new Real_Array (1 .. N * Queries);
         Told   : Flag_Access := new Flag_Array (0 .. N * Queries - 1);
         --  Per point: it moves with some joint, significantly.
         Informative : Flag_Access := new Flag_Array (0 .. Queries - 1);
         --  Per channel, the informative points' sizes of their motion per unit and the standard deviations of those.
         Values : Real_Access := new Real_Array (1 .. N * Queries);
         Sigmas : Real_Access := new Real_Array (1 .. N * Queries);
         Counts : array (1 .. N) of Natural := [others => 0];
         function Slot (C : Positive; P : Natural) return Positive is ((C - 1) * Queries + P + 1);
         --  What the per-point lines leave unexplained, by keyframe, for the log: the mean and the root mean
         --  square of the residuals in pixels over the points whose lines were accepted. A keyframe whose
         --  residuals share a sign and a size beyond the aligner's error is a pose the readings do not describe
         --  (a joint with play beyond its encoder lands elsewhere according to the way it came: the kiwi's
         --  fit left 1.7 to 4.8 px unexplained, R6's tenth start, 10-14).
         Frame_Count : constant Natural := Natural (R.Frames.Length);
         Res_U  : Real_Access := new Real_Array'(1 .. Natural'Max (1, Frame_Count) => 0.0);
         Res_V  : Real_Access := new Real_Array'(1 .. Natural'Max (1, Frame_Count) => 0.0);
         Res_Sq : Real_Access := new Real_Array'(1 .. Natural'Max (1, Frame_Count) => 0.0);
         Res_N  : Counts_Access := new Count_List'(1 .. Natural'Max (1, Frame_Count) => 0);
      begin
         Told.all := [others => False];
         Informative.all := [others => False];
         for P in 0 .. Queries - 1 loop
            declare
               Rows : Natural := 0;
            begin
               for S of R.Matches loop
                  if Answered (S, P) then
                     Rows := Rows + 1;
                  end if;
               end loop;
               --  The point is regressed on the channels that changed in its own keyframes (a column zero in every
               --  row of a point is left out for it), and its coefficients are taken only when its rows determine
               --  them all: the numerical rank of the whole block equals the columns (Regression.Test_Block). Fewer
               --  rows than columns split one keyframe's displacement among the channels that changed in it, each
               --  column alone of full rank, exactly and significantly: such slopes put arm_b's first channel at
               --  0.35 px a unit over 241 points, where the points answered in four keyframes and more said 0.93
               --  (R5 on 10-14). A point answered in one keyframe that turned one joint alone tells that joint's
               --  motion per unit.
               if Rows >= 1 then
                  declare
                     Own : array (1 .. N) of Natural := [others => 0];   --  per channel: its column for this point
                     Kp  : Natural := 0;
                  begin
                     for C in 1 .. N loop
                        if Active (C) then
                           for S of R.Matches loop
                              if Own (C) = 0 and then Answered (S, P)
                                and then (if Noise (C) < Real'Last
                                          then Driver.Uncertain.Significant (Change (S.Frame, C), Noise (C) * Sqrt (2.0))
                                          else Change (S.Frame, C) /= 0.0)
                              then
                                 Kp := Kp + 1;
                                 Own (C) := Kp;
                              end if;
                           end loop;
                        end if;
                     end loop;
                     if Kp >= 1 and then Rows >= Kp then
                        declare
                           X      : Real_Matrix (1 .. Rows, 1 .. Kp) := [others => [others => 0.0]];
                           Yu, Yv : Real_Array (1 .. Rows);
                           Own_S  : Real_Array (1 .. Rows);   --  each answer's standard deviation
                           Row_Frame : array (1 .. Rows) of Positive := [others => 1];   --  each row's keyframe
                           I      : Natural := 0;
                        begin
                           for S of R.Matches loop
                              if Answered (S, P) then
                                 I := I + 1;
                                 Row_Frame (I) := S.Frame;
                                 Own_S (I) := Sigma_Of (S, P);
                                 for C in 1 .. N loop
                                    if Own (C) > 0 then
                                       X (I, Own (C)) := Change (S.Frame, C) / Own_S (I);
                                    end if;
                                 end loop;
                                 Yu (I) := (S.To_U (P) - R.Query_U (P)) / Own_S (I);
                                 Yv (I) := (S.To_V (P) - R.Query_V (P)) / Own_S (I);
                              end if;
                           end loop;
                           --  Scaled by their standard deviations, the residuals have unit variance at the least.
                           declare
                              --  The two fits' first pass is X's alone, made once (Regression.Start_Of).
                              Pass_1 : constant Regression.Start := Regression.Start_Of (X);
                              Fu    : constant Regression.Fit := Regression.Solve (X, Yu, 1.0, Pass_1);
                              Fv    : constant Regression.Fit := Regression.Solve (X, Yv, 1.0, Pass_1);
                              Cov_U : constant Real_Matrix := Regression.Covariance_Of (Fu);
                              Var_U : constant Real_Array := Regression.Coefficient_Variances (Fu, Cov_U);
                              Var_V : constant Real_Array := Regression.Coefficient_Variances (Fv);
                              Stat  : Real;
                              Rank  : Natural;
                           begin
                              Regression.Test_Block (Fu, Cov_U, 1, Kp, Stat, Rank);
                              if Rank = Kp then
                                 for Row in 1 .. Rows loop
                                    declare
                                       Pu, Pv : Real := 0.0;
                                       F : constant Positive := Row_Frame (Row);
                                    begin
                                       for J in 1 .. Kp loop
                                          Pu := Pu + X (Row, J) * Fu.Beta (J);
                                          Pv := Pv + X (Row, J) * Fv.Beta (J);
                                       end loop;
                                       if F <= Frame_Count then
                                          Res_U (F) := Res_U (F) + (Yu (Row) - Pu) * Own_S (Row);
                                          Res_V (F) := Res_V (F) + (Yv (Row) - Pv) * Own_S (Row);
                                          Res_Sq (F) := Res_Sq (F)
                                            + ((Yu (Row) - Pu) ** 2 + (Yv (Row) - Pv) ** 2) * Own_S (Row) ** 2;
                                          Res_N (F) := Res_N (F) + 1;
                                       end if;
                                    end;
                                 end loop;
                                 for C in 1 .. N loop
                                    if Own (C) > 0 then
                                       declare
                                          J : constant Positive := Own (C);
                                          G : constant Real := Sqrt (Fu.Beta (J) ** 2 + Fv.Beta (J) ** 2);
                                          V : constant Real :=
                                            (if G > 0.0
                                             then (Fu.Beta (J) ** 2 * Var_U (J) + Fv.Beta (J) ** 2 * Var_V (J)) / G ** 2
                                             else Real'Max (Var_U (J), Var_V (J)));
                                       begin
                                          if V < Real'Last then
                                             Told (Slot (C, P) - 1) := True;
                                             Slope (Slot (C, P)) := G;
                                             Spread (Slot (C, P)) := Sqrt (Real'Max (0.0, V));
                                             R.Point_Gu.Replace_Element (Slot (C, P) - 1, Fu.Beta (J));
                                             R.Point_Gv.Replace_Element (Slot (C, P) - 1, Fv.Beta (J));
                                             R.Point_G_Told.Replace_Element (Slot (C, P) - 1, True);
                                          end if;
                                       end;
                                    end if;
                                 end loop;
                              end if;
                           end;
                        end;
                     end if;
                  end;
               end if;
            end;
         end loop;
         --  A point tells how far the view moves when it moves with some joint, significantly. One that moves with
         --  none says nothing of the view: fixed to the eye (the hand's jaws in a wrist eye's view, two fifths of
         --  the kiwi's), or answered at its place by false peaks, it is answered in every keyframe at zero motion
         --  per unit of every joint, and the median over every point put arm_b's first joint at 0.012 px a unit
         --  where its moving points said 0.87 (R5 on 10-14: the sweep took the joint for unseen and climbed its
         --  ladder to 3016 units). A channel's measurement is the median over the informative points of what
         --  they move per unit of it, whether or not that is significant in the channel itself: a point's motion
         --  with a joint that moves the view little is small, and only the points that move with some joint are
         --  the world's.
         declare
            Text : Ada.Strings.Unbounded.Unbounded_String;
         begin
            for F in 1 .. Frame_Count loop
               if Res_N (F) > 0 then
                  Ada.Strings.Unbounded.Append
                    (Text, " f" & Ada.Strings.Fixed.Trim (F'Image, Ada.Strings.Left) & " "
                     & Driver.Log.Image (Res_U (F) / Real (Res_N (F)), 2) & "/"
                     & Driver.Log.Image (Res_V (F) / Real (Res_N (F)), 2) & "/"
                     & Driver.Log.Image (Sqrt (Res_Sq (F) / Real (Res_N (F))), 2)
                     & " (" & Ada.Strings.Fixed.Trim (Res_N (F)'Image, Ada.Strings.Left) & ")");
               end if;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image
                             & " shift lines' residuals by keyframe, px (mean u/mean v/rms, points):"
                             & Ada.Strings.Unbounded.To_String (Text));
         end;
         Free (Res_U);
         Free (Res_V);
         Free (Res_Sq);
         Free (Res_N);
         declare
            Told_Of, Moving_Of : array (1 .. N) of Natural := [others => 0];
            Whole : array (1 .. N) of Boolean := [others => True];
            --  The per-point test's own false-alarm rate.
            Alarm : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
            --  The points of the world: those that move with some channel, significantly. A point that moves with
            --  none (the hand's jaws, fixed to the eye) is neither a mover nor silent for any channel: counted as
            --  silent, the jaws made a patch of every channel of arm_b (R6's third start, 10-14: its first pass
            --  stopped seven of ten pushes as unseen).
            Of_World : Flag_Access := new Flag_Array (0 .. Queries - 1);
            --  A point testifies whether the view moves with a channel only where the aligner found it, not merely
            --  answered it, in a keyframe that turned the channel: a keyframe that moved the view past what the
            --  aligner searches is answered by false peaks, at their places or anywhere (f15a on 10-15: arm 1's
            --  fourth channel, a 2.46-degree push that moved the wrist eye's view some 60 px past a 35 px reach, 0
            --  points found and 63 answered, the answers' median 487 px a unit; they made a patch of it, the sweep
            --  took the joint for one behind the eye, never pushed it again, and the arm was never fitted). Where no
            --  point was found, the keyframes tell nothing of the view, and the ladder halves the level instead.
            function Verified (C : Positive; P : Natural) return Boolean is
            begin
               for S of R.Matches loop
                  if S.Frame <= Natural (R.Frames.Length) and then P < Natural (S.Found.Length)
                    and then S.Found (P)
                    and then (if Noise (C) < Real'Last
                              then Driver.Uncertain.Significant (Change (S.Frame, C), Noise (C) * Sqrt (2.0))
                              else Change (S.Frame, C) /= 0.0)
                  then
                     return True;
                  end if;
               end loop;
               return False;
            end Verified;
         begin
            Of_World.all := [others => False];
            for P in 0 .. Queries - 1 loop
               for C in 1 .. N loop
                  if Told (Slot (C, P) - 1) and then Driver.Uncertain.Significant (Slope (Slot (C, P)), Spread (Slot (C, P)))
                  then
                     Of_World (P) := True;
                  end if;
                  if Told (Slot (C, P) - 1) and then Verified (C, P) then
                     Told_Of (C) := Told_Of (C) + 1;
                     if Driver.Uncertain.Significant (Slope (Slot (C, P)), Spread (Slot (C, P))) then
                        Moving_Of (C) := Moving_Of (C) + 1;
                     end if;
                  end if;
               end loop;
            end loop;
            --  A channel moves the view when the points that move with it are not a patch of those told: the
            --  silent ones are no significant majority (Lockin.Still_A_Patch, the lock-in's own rule for an eye).
            --  One that moves a patch moves a part of the body in the view, not the view: the kiwi's wrist eyes
            --  are mounted before the wrist roll, which turns the hand's jaws in their pictures (R5 on 10-14: 97
            --  of some 300 points moved with arm_b's fifth channel, 0.34 px a unit, and they were the jaws'; taken
            --  for the world's, they put the first channel at 0.49 px a unit where the world's points said 0.87).
            --  A patch needs movers: points that move with the channel beyond what the per-point test alarms on
            --  by chance (as Lockin.Judge asks of an eye's cells), and a silent majority among the points that
            --  could tell: a point tells the movers' motion from none when their median stands Z of its own
            --  standard deviation beyond what is significant for it (the lock-in's cells that could tell and did
            --  not, against those too noisy to tell); one that could not tell is neither silent nor moving. A
            --  channel no point moves with yet is not told, and what was told before stands (a turn too small to
            --  move the view: the rig of robot.boot.short, whose first pushes rest short of the keyframe step,
            --  and whose few points told of a joint, two of sixteen significant, the rest uncertain by a quarter
            --  of their motion, made a patch of it).
            for C in 1 .. N loop
               if Told_Of (C) > 0 and then Regression.Count_Significant (Moving_Of (C), Told_Of (C), Alarm) then
                  declare
                     Movers : Real_Array (1 .. Moving_Of (C));
                     K      : Natural := 0;
                     Silent : Natural := 0;
                  begin
                     for P in 0 .. Queries - 1 loop
                        if Told (Slot (C, P) - 1) and then Verified (C, P)
                          and then Driver.Uncertain.Significant (Slope (Slot (C, P)), Spread (Slot (C, P)))
                        then
                           K := K + 1;
                           Movers (K) := Slope (Slot (C, P));
                        end if;
                     end loop;
                     declare
                        Typical : constant Real := Driver.Stats.Median (Movers);
                     begin
                        for P in 0 .. Queries - 1 loop
                           if Of_World (P) and then Told (Slot (C, P) - 1) and then Verified (C, P)
                             and then not Driver.Uncertain.Significant (Slope (Slot (C, P)), Spread (Slot (C, P)))
                             and then Driver.Uncertain.Significant
                                        (Typical - Driver.Conventions.Z * Spread (Slot (C, P)), Spread (Slot (C, P)))
                           then
                              Silent := Silent + 1;
                           end if;
                        end loop;
                     end;
                     Whole (C) := not Lockin.Still_A_Patch (Moving_Of (C), Moving_Of (C) + Silent);
                  end;
                  R.Shift_Seen.Replace_Element (C - 1, Whole (C));
                  R.Shift_Told.Replace_Element (C - 1, Told_Of (C));
               else
                  Whole (C) := R.Shift_Seen (C - 1);
               end if;
            end loop;
            for P in 0 .. Queries - 1 loop
               for C in 1 .. N loop
                  if Whole (C) and then Told (Slot (C, P) - 1)
                    and then Driver.Uncertain.Significant (Slope (Slot (C, P)), Spread (Slot (C, P)))
                  then
                     Informative (P) := True;
                  end if;
               end loop;
            end loop;
            --  How often a point told of the world stood against its keyframe's common motion: the rate a point that
            --  rides with the eye is told apart from (Rides), by the rule of succession (one stand more than seen over
            --  two times more than counted: never nought, never certain).
            declare
               Stood, Had : Natural := 0;
            begin
               for P in 0 .. Queries - 1 loop
                  if Informative (P) and then P < Natural (R.Stood_Off.Length) and then P < Natural (R.Moved_With.Length) then
                     Stood := Stood + R.Stood_Off (P);
                     Had := Had + R.Stood_Off (P) + R.Moved_With (P);
                  end if;
               end loop;
               if Had > 0 then
                  R.Stand_Rate := (Real (Stood) + 1.0) / (Real (Had) + 2.0);
               end if;
            end;
            Free (Of_World);
         end;
         for P in 0 .. Queries - 1 loop
            if Informative (P) then
               for C in 1 .. N loop
                  if Told (Slot (C, P) - 1) then
                     Counts (C) := Counts (C) + 1;
                     Values (Slot (C, Counts (C) - 1)) := Slope (Slot (C, P));
                     Sigmas (Slot (C, Counts (C) - 1)) := Spread (Slot (C, P));
                  end if;
               end loop;
            end if;
         end loop;
         for C in 1 .. N loop
            if Counts (C) > 0 then
               declare
                  First : constant Positive := Slot (C, 0);
                  Last  : constant Positive := Slot (C, Counts (C) - 1);
                  Med   : constant Real := Driver.Stats.Median (Values (First .. Last));
                  --  The median's own standard error, from the points' typical one: the points err apart.
                  Sig   : constant Real :=
                    Driver.Stats.Median_Sigma (Driver.Stats.Median (Sigmas (First .. Last)), Counts (C));
               begin
                  R.Shifts.Replace_Element (C - 1, Med);
                  R.Shift_Sigmas.Replace_Element (C - 1, Sig);
                  R.Shift_Points.Replace_Element (C - 1, Counts (C));
               end;
            end if;
         end loop;
         --  The points that do not move with the view: moving with no joint, and told apart from a view that moves
         --  per unit of some channel (Pixels_Per_Unit) by a motion per unit of it significantly below the view's.
         --  Such a point is no evidence that the view followed a keyframe nor of where the world is (Set_Of, the
         --  fits); one whose keyframes do not tell is neither still nor informative. Told anew from every
         --  keyframe: a point of the world answered at its place by false peaks is found where it is once a
         --  prediction reaches it.
         while Natural (R.Still.Length) < Queries loop
            R.Still.Append (False);
         end loop;
         for P in 0 .. Queries - 1 loop
            declare
               Still : Boolean := False;
            begin
               if not Informative (P) then
                  for C in 1 .. N loop
                     if Told (Slot (C, P) - 1) and then Pixels_Per_Unit (M, R, C) > 0.0
                       and then Driver.Uncertain.Significant (Pixels_Per_Unit (M, R, C) - Slope (Slot (C, P)),
                                                              Spread (Slot (C, P)))
                     then
                        Still := True;
                     end if;
                  end loop;
               end if;
               R.Still.Replace_Element (P, Still);
            end;
         end loop;
         Free (Slope);
         Free (Spread);
         Free (Told);
         Free (Informative);
         Free (Values);
         Free (Sigmas);
      end;
   end Measure_Shifts;

   procedure Set_Of
     (Who : String; F : Positive; Query_U, Query_V : Real_Vectors.Vector; Still : Flag_Vectors.Vector;
      Given : Keyframes.Prediction_Array; Answers : Keyframes.Answer_Array; Agree : Keyframes.Flag_Array;
      Set : out Match_Set)
   is
      Counted : Natural := 0;
      Typical : Real := 0.0;
      function Sigma_Of (A : Driver.Alignment.Answer) return Real is
        (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (A.Cov.UU, A.Cov.VV)));
      function Agrees (I : Natural) return Boolean is (Agree (Agree'First + I - Answers'First));
      function Answered (I : Natural) return Boolean is (Answers (I).Verdict = Driver.Alignment.Found and then Agrees (I));
      --  Answered by a point that moves with the view: one that does not (Arm_Evidence.Still, Measure_Shifts) is
      --  found at its place in every keyframe, and is no evidence that the view followed, nor of where the world
      --  is (the hand's jaws in the kiwi's wrist eyes, R5 on 10-14: every keyframe of a joint turned 2277 units
      --  was followed by them).
      function Moving (I : Natural) return Boolean is
        (Answered (I) and then (I - 1 >= Natural (Still.Length) or else not Still (I - 1)));
      Followed : Boolean := False;
      function Sighted (I : Natural) return Boolean is (Followed and then Moving (I) and then Answers (I).Narrow);
   begin
      declare
         Found, From_Narrow : Natural := 0;
      begin
         for I in Answers'Range loop
            if Moving (I) then
               Found := Found + 1;
               if Answers (I).Narrow then
                  From_Narrow := From_Narrow + 1;
               end if;
            end if;
         end loop;
         Followed := From_Narrow > Found - From_Narrow;   --  more of the answers came from narrow windows than from wider
         --  A keyframe not followed, for the log: how far its found points lie from where the prediction put them,
         --  and how near it said they would be (its stated sigma), the narrow answers apart from the wide.
         if not Followed and then Found > 0 then
            declare
               Off_N, Off_W, Said : Real_Access := new Real_Array (1 .. Found);
               KN, KW, KS : Natural := 0;
               function Mid (X : Real_Access; K : Natural) return String is
                 (if K = 0 then "none" else Driver.Log.Image (Driver.Stats.Median (X (1 .. K)), 2));
            begin
               for I in Answers'Range loop
                  if Moving (I) then
                     declare
                        G : constant Driver.Alignment.Prediction := Given (Given'First + I - Answers'First);
                        D : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt
                          ((Answers (I).To.U - G.To.U) ** 2 + (Answers (I).To.V - G.To.V) ** 2);
                     begin
                        if Answers (I).Narrow then
                           KN := KN + 1;
                           Off_N (KN) := D;
                        else
                           KW := KW + 1;
                           Off_W (KW) := D;
                        end if;
                        KS := KS + 1;
                        Said (KS) := Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (G.Cov.UU, G.Cov.VV));
                     end;
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " keyframe" & F'Image & " not followed: "
                                & Found'Image & " moving points found," & From_Narrow'Image & " in narrow windows;"
                                & " found a median " & Mid (Off_N, KN) & " px from their predictions in narrow windows, "
                                & Mid (Off_W, KW) & " px in wide ones; the predictions stated " & Mid (Said, KS) & " px");
               Free (Off_N);
               Free (Off_W);
               Free (Said);
            end;
         end if;
      end;
      for I in Answers'Range loop
         if Sighted (I) then
            Counted := Counted + 1;
         end if;
      end loop;
      if Counted > 0 then
         declare
            Sigmas : Real_Access := new Real_Array (1 .. Counted);
            K      : Natural := 0;
         begin
            for I in Answers'Range loop
               if Sighted (I) then
                  K := K + 1;
                  Sigmas (K) := Sigma_Of (Answers (I));
               end if;
            end loop;
            Typical := Driver.Stats.Median (Sigmas.all);
            Free (Sigmas);
         end;
      end if;
      Set := (Frame => F, Stated => Stated_By (Given), Followed => Followed, others => <>);
      for I in Answers'Range loop
         Set.To_U.Append (Answers (I).To.U);
         Set.To_V.Append (Answers (I).To.V);
         Set.Sigma.Append (if Answers (I).Verdict = Driver.Alignment.Found then Sigma_Of (Answers (I)) else 0.0);
         Set.Freedom.Append (if Answers (I).Verdict = Driver.Alignment.Found then Answers (I).Degrees_Of_Freedom else 0);
         --  There is no way back: an answer is judged by its own uncertainty, not by another answer.
         Set.Back_U.Append (Query_U (I - 1));
         Set.Back_V.Append (Query_V (I - 1));
         Set.Found.Append (Sighted (I) and then Sigma_Of (Answers (I)) <= Driver.Conventions.Z * Typical);
         Set.Answered.Append (Answered (I));
         if Keyframes.Prediction_Caused (Answers (I)) or else (Moving (I) and then not Sighted (I)) then
            Set.Wide := Set.Wide + 1;
         end if;
      end loop;
   end Set_Of;

   function Readings_Array (F : Keyframe) return Real_Array is
      Result : Real_Array (1 .. Natural (F.Readings.Length));
   begin
      for C in Result'Range loop
         Result (C) := F.Readings (C - 1);
      end loop;
      return Result;
   end Readings_Array;

   --  Keyframe F's readings less where its joints' gears' slack stood then (Slack_Parts): the readings the fitted
   --  arm stands at, which its offer of the keyframe is made at (Offer_Of). Made at the readings as read, a fitted
   --  arm's offer put a joint where its slack had not let it go: e20x_20 (seed 20 of the default fake robot, 10-22,
   --  its base joint's half slack 0.139 units, about 2 degrees), keyframe 48 found its points a median 7.46 px from
   --  where they were predicted, the prediction stating 2.45 px.
   function Standing (M : Model; R : Arm_Evidence; F : Positive) return Real_Array is
      Readings : Real_Array := Readings_Array (R.Frames (F));
      Parts    : constant Real_Array := Slack_Parts (M, R.Arm, R.Frames (F).Beat);
   begin
      if Parts'Length = Readings'Length then
         for C in Readings'Range loop
            Readings (C) := Readings (C) - Parts (Parts'First + C - Readings'First);
         end loop;
      end if;
      return Readings;
   end Standing;

   type Tally is array (Driver.Alignment.Reason) of Natural;

   --  A tally of the aligner's reasons, for the log: "; because: matched 5358, no_texture 1200, ...".
   function Reasons_Text (Of_These : Tally) return String is
      Text : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for Why in Of_These'Range loop
         if Of_These (Why) > 0 then
            Ada.Strings.Unbounded.Append
              (Text, (if Ada.Strings.Unbounded.Length (Text) = 0 then "; because: " else ", ")
               & Ada.Characters.Handling.To_Lower (Why'Image) & Of_These (Why)'Image);
         end if;
      end loop;
      return Ada.Strings.Unbounded.To_String (Text);
   end Reasons_Text;

   --  The arm's keyframes not yet aligned, in order.
   --  Each query point's patch in keyframe F against the reference's, at the point's place there, the levels'
   --  differences averaged over the patch the aligner matches by: at a keyframe at the reference's pose (no channel
   --  changed beyond its noise) the noise alone, kept as the point's floor with its standard error; at another, the
   --  patch changed where its mean difference is significantly above its floor. Where most points' patches changed the
   --  view moved, and a point whose own did not stood with the eye: counted (Patch_Stood of Patch_Moving), and with
   --  them how often a point stands by chance (Patch_Rate). Neither a field of the view nor the arm's fit is asked:
   --  a field told from the jaws themselves, or the table's grain, put them still or moving at will (w18, below).
   procedure Patch_Evidence (R : in out Arm_Evidence; F : Positive; Change, Noise : Real_Array) is
      use Ada.Numerics.Long_Elementary_Functions;
      Queries : constant Natural := Natural (R.Query_U.Length);
      W       : constant Natural := Driver.Images.Width (R.Frames (F).Image);
      H       : constant Natural := Driver.Images.Height (R.Frames (F).Image);
   begin
      if Queries = 0 or else W = 0 or else H = 0 or else Driver.Images.Width (R.Frames.First_Element.Image) /= W
        or else Driver.Images.Height (R.Frames.First_Element.Image) /= H
      then
         return;
      end if;
      while Natural (R.Patch_Noise.Length) < Queries loop
         R.Patch_Noise.Append (0.0);
         R.Patch_Error.Append (0.0);
         R.Patch_Stood.Append (0);
         R.Patch_Moving.Append (0);
      end loop;
      declare
         Half   : constant Natural := Driver.Alignment.Half_Patch (W, H);
         Before : Real_Access := new Real_Array (1 .. W * H);
         After  : Real_Access := new Real_Array (1 .. W * H);
         Mean, Error : Real_Array (1 .. Queries) := [others => 0.0];
         Inside : array (1 .. Queries) of Boolean := [others => False];
         Same   : Boolean := True;
      begin
         for C in Change'Range loop
            if Driver.Uncertain.Significant (Change (C), Noise (Noise'First + C - Change'First) * Sqrt (2.0)) then
               Same := False;
            end if;
         end loop;
         Driver.Images.Luma (R.Frames.First_Element.Image, Before.all);
         Driver.Images.Luma (R.Frames (F).Image, After.all);
         for P in 1 .. Queries loop
            declare
               X0 : constant Integer := Integer (Real'Floor (R.Query_U (P - 1)));
               Y0 : constant Integer := Integer (Real'Floor (R.Query_V (P - 1)));
            begin
               if X0 - Half >= 0 and then Y0 - Half >= 0 and then X0 + Half < W and then Y0 + Half < H then
                  declare
                     Sum, Squares : Real := 0.0;
                     Count        : Natural := 0;
                  begin
                     for Y in Y0 - Half .. Y0 + Half loop
                        for X in X0 - Half .. X0 + Half loop
                           declare
                              D : constant Real := abs (After (Y * W + X + 1) - Before (Y * W + X + 1));
                           begin
                              Sum := Sum + D;
                              Squares := Squares + D * D;
                              Count := Count + 1;
                           end;
                        end loop;
                     end loop;
                     Mean (P) := Sum / Real (Count);
                     Error (P) := Sqrt (Real'Max (0.0, Squares / Real (Count) - Mean (P) ** 2) / Real (Count));
                     Inside (P) := True;
                  end;
               end if;
            end;
         end loop;
         Free (Before);
         Free (After);
         if Same then
            for P in 1 .. Queries loop
               if Inside (P) and then R.Patch_Noise (P - 1) = 0.0 and then Mean (P) > 0.0 then
                  R.Patch_Noise.Replace_Element (P - 1, Mean (P));
                  R.Patch_Error.Replace_Element (P - 1, Error (P));
               end if;
            end loop;
         else
            declare
               Changed   : array (1 .. Queries) of Boolean := [others => False];
               Floored   : Natural := 0;
               Moving    : Natural := 0;
            begin
               for P in 1 .. Queries loop
                  if Inside (P) and then R.Patch_Noise (P - 1) > 0.0 then
                     Floored := Floored + 1;
                     Changed (P) := Mean (P) > R.Patch_Noise (P - 1)
                       and then Driver.Uncertain.Significant
                                  (Mean (P) - R.Patch_Noise (P - 1), Sqrt (Error (P) ** 2 + R.Patch_Error (P - 1) ** 2));
                     if Changed (P) then
                        Moving := Moving + 1;
                     end if;
                  end if;
               end loop;
               if 2 * Moving > Floored then
                  declare
                     Stood, Had : Natural := 0;
                  begin
                     for P in 1 .. Queries loop
                        if Inside (P) and then R.Patch_Noise (P - 1) > 0.0 then
                           R.Patch_Moving.Replace_Element (P - 1, R.Patch_Moving (P - 1) + 1);
                           if not Changed (P) then
                              R.Patch_Stood.Replace_Element (P - 1, R.Patch_Stood (P - 1) + 1);
                           end if;
                        end if;
                        Stood := Stood + R.Patch_Stood (P - 1);
                        Had := Had + R.Patch_Moving (P - 1);
                     end loop;
                     R.Patch_Rate := (Real (Stood) + 1.0) / (Real (Had) + 2.0);
                  end;
               end if;
            end;
         end if;
      end;
   end Patch_Evidence;

   procedure Align_Arm (M : Model; R : in out Arm_Evidence) is
      N       : constant Natural := M.Groups (R.Group).Size;
      Queries : constant Natural := Natural (R.Query_U.Length);
      Noise   : Real_Array (1 .. N);
      Shift   : Real_Array (1 .. N);
      Start   : constant Duration := Driver.Clock.Seconds;
      First   : constant Natural := R.Aligned + 1;
      Found   : Natural := 0;
      Because : Tally := [others => 0];   --  why each point was or was not found
   begin
      for C in 1 .. N loop
         Noise (C) := Channels.Noise (M, R.Group, C);
      end loop;
      if (for some X of Noise => X >= Real'Last) then
         return;
      end if;
      Refresh_Responds (M, R);
      if R.Aligned = 0 then
         declare
            Points : Keyframe_Point_Access := new Keyframes.Point_Array (1 .. Queries);
         begin
            for I in Points'Range loop
               Points (I) := (U => R.Query_U (I - 1), V => R.Query_V (I - 1));
            end loop;
            Keyframes.Begin_Reference (R.Chain, R.Frames.First_Element.Image, Points.all);
            Free (Points);
         end;
         R.Aligned := 1;
      end if;
      while R.Aligned < Natural (R.Frames.Length) loop
         declare
            F       : constant Positive := R.Aligned + 1;
            Change  : Real_Array (1 .. N);
            Given   : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
            Offered : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
            Answers : Alignment_Access := new Keyframes.Answer_Array (1 .. Queries);
            Agree   : Agreement_Access := new Keyframes.Flag_Array (1 .. Queries);
            From    : Keyframes.Source;
            Known   : Boolean;
            Set     : Match_Set;
            Rss_Before, Rss_After : Natural := 0;   --  resident MB before and after the alignment, for the log
         begin
            for C in 1 .. N loop
               Change (C) := R.Frames (F).Readings (C - 1) - R.Frames.First_Element.Readings (C - 1);
               --  As measured by the keyframes aligned so far: the first keyframe that turns a channel is predicted
               --  from nothing (Keyframes.Predict: a window as wide as the aligner searches), the next from it.
               Shift (C) := Pixels_Per_Unit (M, R, C);
            end loop;
            Offer_Of (M, R, Standing (M, R, F), Offered.all, Known);
            if Known then
               Keyframes.Predict (R.Chain, Change, Noise, Shift, Offered.all, Given.all, From);
            else
               Keyframes.Predict (R.Chain, Change, Noise, Shift, Keyframes.No_Offer, Given.all, From);
            end if;
            Rss_Before := Driver.Memory.Resident_Kilobytes / 1024;
            declare
               --  The points that tell nothing of how the view moved: those that ride with the eye, as the keyframes so
               --  far told them (Measure_Shifts), or before they tell, those whose cells the lock-in did not see
               --  follow the arm (Query_Points: the hand's jaws in a wrist eye's view among them).
               Riding : Keyframes.Flag_Array (1 .. Queries);
            begin
               for I in Riding'Range loop
                  Riding (I) := not Moving_Point (R, I - 1);
               end loop;
               Keyframes.Add (R.Chain, R.Frames (F).Image, Change, Given.all, Answers.all, Agree.all, Riding);
               declare
                  Stood, Moved : Keyframes.Flag_Array (1 .. Queries);
               begin
                  Keyframes.Stood_And_Moved (R.Chain, Stood, Moved);
                  while Natural (R.Stood_Off.Length) < Queries loop
                     R.Stood_Off.Append (0);
                     R.Moved_With.Append (0);
                  end loop;
                  for I in 1 .. Queries loop
                     if Stood (I) then
                        R.Stood_Off.Replace_Element (I - 1, R.Stood_Off (I - 1) + 1);
                     end if;
                     if Moved (I) then
                        R.Moved_With.Replace_Element (I - 1, R.Moved_With (I - 1) + 1);
                     end if;
                  end loop;
               end;
               Patch_Evidence (R, F, Change, Noise);
            end;
            Rss_After := Driver.Memory.Resident_Kilobytes / 1024;
            Set_Of ("arm" & R.Arm'Image, F, R.Query_U, R.Query_V, Unmoving (R), Given.all, Answers.all, Agree.all,
                    Set);
            --  For a study against a fake robot's truth (tools/truth_points): with BD_DUMP_KEYS naming a folder, every
            --  point of the keyframe, where it stood in the reference, where the prediction put it and stated it, and what
            --  the aligner answered, in a file of the arm and the keyframe. u5's elbow was found so to have turned twice the
            --  turn planned for it (10-19), the view's points 20 px on the mean and 106 of 255 out of the picture.
            if Ada.Environment_Variables.Exists ("BD_DUMP_KEYS") then
               declare
                  File : Ada.Text_IO.File_Type;
                  Name : constant String := Ada.Environment_Variables.Value ("BD_DUMP_KEYS") & "/arm"
                    & Ada.Strings.Fixed.Trim (R.Arm'Image, Ada.Strings.Left) & "_kf"
                    & Ada.Strings.Fixed.Trim (F'Image, Ada.Strings.Left) & ".txt";
               begin
                  Ada.Text_IO.Create (File, Ada.Text_IO.Out_File, Name);
                  Ada.Text_IO.Put_Line (File, "beat" & R.Frames (F).Beat'Image & " reference"
                                        & R.Frames.First_Element.Beat'Image & " eye" & R.Eye'Image);
                  for I in 1 .. Queries loop
                     declare
                        G : Driver.Alignment.Prediction renames Given (Given'First + I - 1);
                        A : Driver.Alignment.Answer renames Answers (Answers'First + I - 1);
                     begin
                        Ada.Text_IO.Put_Line
                          (File, I'Image & " " & Driver.Log.Image (R.Query_U (I - 1), 2) & " "
                           & Driver.Log.Image (R.Query_V (I - 1), 2) & " given " & Driver.Log.Image (G.To.U, 2) & " "
                           & Driver.Log.Image (G.To.V, 2) & " "
                           & Driver.Log.Image (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (G.Cov.UU, G.Cov.VV)), 2)
                           & " answer " & Driver.Log.Image (A.To.U, 2) & " " & Driver.Log.Image (A.To.V, 2) & " "
                           & Ada.Characters.Handling.To_Lower (A.Verdict'Image) & " "
                           & (if A.Narrow then "narrow" else "wide") & " "
                           & (if Agree (Agree'First + I - 1) then "agree" else "apart") & " "
                           & Ada.Characters.Handling.To_Lower (A.Because'Image) & " "
                           & (if Set.Found (I - 1) then "sighting" else "-") & " "
                           & (if I - 1 < Natural (R.Query_Responds.Length) and then not R.Query_Responds (I - 1)
                               then "still" else "follows"));
                     end;
                  end loop;
                  Ada.Text_IO.Close (File);
               end;
            end if;
            --  A change of a channel whose shift the keyframes have not measured is a new direction: the keyframe is
            --  asked from the view's common motion (Keyframes.Add). Before, a new direction searched every point
            --  alone as wide as the aligner looks, and when that found nothing it was aligned again from the lock-in's
            --  slope: the flow had locked on what does not move with the view, the window was a fortieth of the
            --  motion, and its answers stood false in the chain (f15c on 10-15: arm 1's base turned ten degrees,
            --  the view moved 90 px, the slope said 2 px; all 35 answers lay 78 px from the truth).
            declare
               --  This keyframe's own reasons, on its line: the batch's tally cannot say which keyframe found nothing
               --  and why (f15w on 10-15: arm 2's pushes of channel 4 after the first found 0 of 255, 0 answered).
               Its_Because : Tally := [others => 0];
               Here    : Natural := 0;
               Replied : Natural := 0;   --  answered from a window of any width (Match_Set.Answered)
               Text    : Ada.Strings.Unbounded.Unbounded_String;
            begin
               for A of Answers.all loop
                  Because (A.Because) := Because (A.Because) + 1;
                  Its_Because (A.Because) := Its_Because (A.Because) + 1;
               end loop;
               for Seen of Set.Found loop
                  if Seen then
                     Here := Here + 1;
                  end if;
               end loop;
               Found := Found + Here;
               for Seen of Set.Answered loop
                  if Seen then
                     Replied := Replied + 1;
                  end if;
               end loop;
               --  Each keyframe's line, for the study of what the aligner finds at what motion: the change of every
               --  channel from the reference, how the prediction was made, and how many points were found.
               for C in 1 .. N loop
                  Ada.Strings.Unbounded.Append (Text, " " & Driver.Log.Image (Change (C), 5));
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe" & F'Image & " at beat"
                                & R.Frames (F).Beat'Image & ", change" & Ada.Strings.Unbounded.To_String (Text)
                                & Asked_Image (R.Frames (F))
                                & ", predicted " & Ada.Characters.Handling.To_Lower (From'Image)
                                & (if From = Keyframes.New_Direction then Motion_Image (Keyframes.Last_Motion (R.Chain))
                                   else " stating " & Driver.Log.Image (Stated_By (Given.all), 1) & " px"
                                        & (if Keyframes.Last_Motion (R.Chain).Searched
                                           then ", asked" & Motion_Image (Keyframes.Last_Motion (R.Chain)) else ""))
                                & ":" & Here'Image
                                & " of" & Queries'Image & " points found," & Replied'Image & " answered; resident"
                                & Rss_Before'Image & " /"
                                & Rss_After'Image & " /" & Natural'Image (Driver.Memory.Resident_Kilobytes / 1024)
                                & " MB before the alignment, after it, now" & Reasons_Text (Its_Because));
            end;
            R.Matches.Append (Set);
            R.Aligned := F;
            --  What the keyframes measure of the view's motion per unit, with this one: the next is predicted by it.
            Measure_Shifts (M, R);
            --  A keyframe that refused nothing for its prediction needs its picture no more.
            if Set.Wide = 0 then
               R.Frames (F).Image := Driver.Images.No_Image;
            end if;
            Free (Given);
            Free (Offered);
            Free (Answers);
            Free (Agree);
         end;
      end loop;
      Driver.Log.Line
        (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " aligned keyframes" & First'Image & " to"
         & R.Aligned'Image & ":" & Found'Image & " of" & Natural'Image ((R.Aligned - First + 1) * Queries)
         & " points found, the aligner errs by " & Driver.Log.Image (Match_Sigma (R), 3) & " px, in "
         & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s ("
         & Driver.Log.Image (Real (Driver.Clock.Seconds - Start) / Real (Natural'Max (1, R.Aligned - First + 1)), 2)
         & " s a keyframe)" & Reasons_Text (Because)
         & "; the aligner's work: " & Driver.Alignment.Work_Report & "; resident"
         & Natural'Image (Driver.Memory.Resident_Kilobytes / 1024) & " MB");
      --  The steps a keyframe of each channel needs, for the log: what the matcher tells apart against what the eye sees.
      declare
         Text : Ada.Strings.Unbounded.Unbounded_String;
      begin
         for C in 1 .. N loop
            declare
               Seen : constant Estimate := Visible_Step (M, R.Group, C);
            begin
               Ada.Strings.Unbounded.Append
                 (Text, " c" & C'Image & " " & Driver.Log.Image (Keyframe_Step (M, R.Arm, C), 6) & " (eye "
                  & (if Known (Seen) then Driver.Log.Image (Seen.Value, 6) else "?") & "; px a unit: keyframes "
                  & (if C <= Natural (R.Shift_Sigmas.Length) and then R.Shift_Sigmas (C - 1) < Real'Last
                     then Driver.Log.Image (R.Shifts (C - 1), 4) & " +- " & Driver.Log.Image (R.Shift_Sigmas (C - 1), 4)
                          & " over" & Natural'Image (R.Shift_Points (C - 1)) & " points"
                          & (if C <= Natural (R.Shift_Seen.Length) and then not R.Shift_Seen (C - 1)
                             then ", a patch of the" & Natural'Image (R.Shift_Told (C - 1)) & " told: not the view"
                             else "")
                     else "none")
                  & ", lock-in " & Driver.Log.Image (Lockin.Shift (M, R.Eye, R.Group, C), 4) & ", used "
                  & Driver.Log.Image (Pixels_Per_Unit (M, R, C), 4) & ")");
            end;
         end loop;
         declare
            Fixed : Natural := 0;
         begin
            for F of R.Still loop
               if F then
                  Fixed := Fixed + 1;
               end if;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe steps, matcher (eye):"
                             & Ada.Strings.Unbounded.To_String (Text) & ";" & Fixed'Image & " of"
                             & Natural'Image (Natural (R.Query_U.Length)) & " points do not move with the view");
         end;
      end;
   end Align_Arm;

   ---------------------------------------------------------------------------
   --  A fixed eye's view of an arm

   --  The lens moved along its term Term (in Fit.Lens_Terms' order) by By.
   function Moved (L : Fit.Lens; Term : Positive; By : Real) return Fit.Lens is
     (case Term is
         when 1      => (L with delta Fx => L.Fx * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
         when 2      => (L with delta Fy => L.Fy * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
         when 3      => (L with delta Cx => L.Cx + By),
         when 4      => (L with delta Cy => L.Cy + By),
         when 5      => (L with delta K1 => L.K1 + By),
         when others => (L with delta K2 => L.K2 + By));

   procedure Align_New (M : in out Model) is
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R : Arm_Evidence renames M.Kinematics (Index);
         begin
            if Current (M, R) and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index then
               if Natural (R.Frames.Length) > R.Aligned and then not R.Query_U.Is_Empty then
                  Align_Arm (M, R);
               end if;
            end if;
         end;
      end loop;
   end Align_New;

   --  Whether a prediction that states Sigma has a window narrower than the one a keyframe was aligned from, by enough to
   --  be worth aligning it again.
   function Narrowed (Was : Match_Set; Sigma : Real) return Boolean is
      Then_Radius : constant Real := Driver.Alignment.Window_Radius (Was.Stated);
      Now_Radius  : constant Real := Driver.Alignment.Window_Radius (Sigma);
   begin
      return (if Was.Followed then Now_Radius * Real (Driver.Alignment.Halving) <= Then_Radius
              else Then_Radius - Now_Radius >= 1.0);
   end Narrowed;

   --  Of a keyframe's answers a fit may rest on (Found), those within the gate of where a fit puts them (Offered) and
   --  of what it states of them there. A keyframe aligned again from a fit (Realign_Arms) takes the new
   --  answers when as many of them as of the old lie there: it gains what its first window refused, and loses nothing
   --  it had. A keyframe of a joint the fit hardly knows, aligned again from a window wider than the one its first
   --  answers came from, lost them: u3 replayed (servo seed 3 of the fake robot, 10-18), channel 4's keyframe 13, 97
   --  points found from the view's common motion, 13 from the fitted arm's window of 5.3 px, not one from a window a
   --  patch carries, and the next fit had no keyframe of channel 4's own. A keyframe kept takes the window it was
   --  offered as the one to narrow on, so it is aligned again only from a narrower one.
   function Agreeing (X : Match_Set; Offered : Keyframes.Prediction_Array) return Natural is
      Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
      N    : Natural := 0;
   begin
      for I in 0 .. Natural'Min (Natural (X.Found.Length), Offered'Length) - 1 loop
         if X.Found (I) then
            declare
               P : Driver.Alignment.Prediction renames Offered (Offered'First + I);
            begin
               if not Driver.Uncertain.Significant
                        (Gate, Ada.Numerics.Long_Elementary_Functions.Sqrt
                                 ((X.To_U (I) - P.To.U) ** 2 + (X.To_V (I) - P.To.V) ** 2),
                         Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (P.Cov.UU, P.Cov.VV)))
               then
                  N := N + 1;
               end if;
            end;
         end if;
      end loop;
      return N;
   end Agreeing;

   function Realign_Arms (M : in out Model) return Boolean is
      Changed : Boolean := False;
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R       : Arm_Evidence renames M.Kinematics (Index);
            Queries : constant Natural := Natural (R.Query_U.Length);
            Done    : Natural := 0;
            Kept    : Natural := 0;   --  aligned again and kept as they were
            Start   : constant Duration := Driver.Clock.Seconds;
         begin
            if Current (M, R) and then R.Result.Fitted and then R.Aligned >= 2 and then Queries > 0 then
               for S in R.Matches.First_Index .. R.Matches.Last_Index loop
                  if R.Matches (S).Wide > 0 and then Driver.Images.Width (R.Frames (R.Matches (S).Frame).Image) > 0 then
                     declare
                        F       : constant Positive := R.Matches (S).Frame;
                        Offered : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
                        Answers : Alignment_Access := new Keyframes.Answer_Array (1 .. Queries);
                        Agree   : Agreement_Access := new Keyframes.Flag_Array (1 .. Queries);
                        Known   : Boolean;
                     begin
                        Offer_Of (M, R, Standing (M, R, F), Offered.all, Known);
                        --  A keyframe is aligned again when the fitted arm gives it a window narrower by a pixel (the grid the
                        --  aligner's candidates lie on) than the one it was aligned from, and one a size of the pictures narrower if
                        --  what it answered was what a fit may rest on already (a keyframe the fit takes in, or not, by whether most
                        --  of its answers came from a window no wider than a patch: a better fit may narrow the window of the
                        --  next one to that). The windows narrow by a pixel at the least each time, so the repeating ends.
                        if Known and then Narrowed (R.Matches (S), Stated_By (Offered.all)) then
                           Keyframes.Realign (R.Chain, R.Frames (F).Image, Offered.all, Answers.all, Agree.all);
                           --  The new answers stand in for the old when as many lie where the fitted arm puts them
                           --  (Agreeing).
                           declare
                              Set : Match_Set;
                           begin
                              Set_Of ("arm" & R.Arm'Image, F, R.Query_U, R.Query_V, Unmoving (R), Offered.all, Answers.all, Agree.all,
                                      Set);
                              if Agreeing (Set, Offered.all) >= Agreeing (R.Matches (S), Offered.all) then
                                 R.Matches.Replace_Element (S, Set);
                                 if Set.Wide = 0 then
                                    R.Frames (F).Image := Driver.Images.No_Image;
                                 end if;
                                 R.Revision := R.Revision + 1;
                                 Done := Done + 1;
                                 Changed := True;
                              else
                                 declare
                                    Was : Match_Set := R.Matches (S);
                                 begin
                                    Was.Stated := Set.Stated;
                                    R.Matches.Replace_Element (S, Was);
                                 end;
                                 Kept := Kept + 1;
                              end if;
                           end;
                        end if;
                        Free (Offered);
                        Free (Answers);
                        Free (Agree);
                     end;
                  end if;
               end loop;
               if Done > 0 then
                  Measure_Shifts (M, R);
               end if;
               if Done > 0 or else Kept > 0 then
                  Driver.Log.Line
                    (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " aligned" & Done'Image
                     & " keyframes again from the fitted arm"
                     & (if Kept > 0
                        then ", and" & Kept'Image & " more kept as they were (fewer of the new answers where the arm"
                             & " puts them)"
                        else "")
                     & ", in " & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s");
               end if;
            end if;
         end;
      end loop;
      return Changed;
   end Realign_Arms;

   function Realign (M : in out Model) return Boolean is (Realign_Arms (M));

   --  Every arm but the first placed in the world, through a fixed eye that saw it and the first arm move (In_World).
   procedure Place (M : in out Model);

   type Sight_Access is access Fit.Sight_Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Sight_Point_Array, Sight_Access);
   type Fit_Flag_Access is access Fit.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Flag_Array, Fit_Flag_Access);

   --  The arm's tracks as its reference eye has them through lens L: on
   --  their reference lines of sight, at the depths the fit refined.
   procedure Sights_Of (U, V : Real_Vectors.Vector; F : Arm_Fit; L : Fit.Lens; S : out Fit.Sight_Point_Array)
     with Pre => S'First = 1 and then S'Length = Natural (U.Length) and then Natural (V.Length) = Natural (U.Length)
   is
   begin
      for I in S'Range loop
         declare
            K : constant Natural := I - 1;
         begin
            S (I) :=
              (H     => Fit.Ray (L, U (K), V (K)),
               Depth => (if K < Natural (F.Track_Known.Length) and then F.Track_Known (K)
                         then F.Tracks (3 * K + 2) else 0.0),
               Sigma => (if K < Natural (F.Track_Sigmas.Length) then F.Track_Sigmas (K) else Real'Last));
         end;
      end loop;
   end Sights_Of;

   --  Which of the arm's tracks lie on its table (Fit.Dominant_Plane).
   procedure Table_Of (F : Arm_Fit; On : out Fit.Flag_Array)
     with Pre => On'First = 1
   is
   begin
      for I in On'Range loop
         On (I) := I - 1 < Natural (F.Table_On.Length) and then F.Table_On (I - 1);
      end loop;
   end Table_Of;

   --  The table in the arm's frame with its whole uncertainty: its points'
   --  scatter about it (Plane's, the sandwich of Refit_Plane) and the fit's
   --  terms, carried with their covariance through how they move it: every
   --  term through every depth together (Fit.Plane_Response), and each lens
   --  term through the lines of sight as well, the depths held (the plane
   --  refitted on the same points with the term moved by its standard
   --  deviation either way), and that response (Response: three rows of as
   --  many as the covariance has terms, row by row). Unknown, and no
   --  response, without the fit's covariance.
   procedure Table_Estimate
     (U, V       : Real_Vectors.Vector;
      F          : Arm_Fit;
      L          : Fit.Lens;
      Sights     : Fit.Sight_Point_Array;
      Plane      : Fit.Sight_Plane;
      Gains      : Fit.Real_Lists.Vector;
      Covariance : Fit.Real_Lists.Vector;
      Estimate   : out Driver.Geometry.Plane_Estimate;
      Response   : out Real_Vectors.Vector)
   is
      use Driver.Numerics.Arrays;
      Q     : constant Natural := Natural (U.Length);
      Terms : constant Natural := Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                             (Real (Natural (Covariance.Length))));

      function Vc (P, C : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Terms + C - 1));
   begin
      if not Plane.Found or else Terms < Fit.Lens_Terms or else Terms * Terms /= Natural (Covariance.Length) then
         Estimate := (others => <>);
         Response.Clear;
         return;
      end if;
      declare
         On : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q);
         S  : Sight_Access := new Fit.Sight_Point_Array (1 .. Q);
      begin
         Table_Of (F, On.all);
         declare
            T     : Real_Matrix := Fit.Plane_Response (Sights, On.all, Gains, Terms);
            Cv    : Real_Matrix (1 .. Terms, 1 .. Terms);
            Total : Mat3;
         begin
            for K in 1 .. Fit.Lens_Terms loop
               if Vc (K, K) > 0.0 then
                  declare
                     Sigma_K : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Vc (K, K));
                     Ends    : array (1 .. 2) of Vec3;
                     Ok      : Boolean := True;
                  begin
                     for E in 1 .. 2 loop
                        declare
                           P : Fit.Sight_Plane;
                        begin
                           Sights_Of (U, V, F, Moved (L, K, (if E = 1 then -Sigma_K else Sigma_K)), S.all);
                           --  The depths held, the points move with their lines of sight.
                           Fit.Refit_Plane (S.all, On.all, P);
                           Ok := Ok and then P.Found;
                           Ends (E) := P.A;
                        end;
                     end loop;
                     if Ok then
                        for D in 1 .. 3 loop
                           T (T'First (1) + D - 1, T'First (2) + K - 1) :=
                             T (T'First (1) + D - 1, T'First (2) + K - 1) + (Ends (2) (D) - Ends (1) (D)) / (2.0 * Sigma_K);
                        end loop;
                     end if;
                  end;
               end if;
            end loop;
            for P in 1 .. Terms loop
               for C in 1 .. Terms loop
                  Cv (P, C) := Vc (P, C);
               end loop;
            end loop;
            Total := Plane.Covariance + T * Cv * Transpose (T);
            Free (On);
            Free (S);
            Estimate := Fit.Plane_Estimate_Of (Plane, Total);
            Response.Clear;
            for D in T'Range (1) loop
               for K in T'Range (2) loop
                  Response.Append (T (D, K));
               end loop;
            end loop;
         end;
      end;
   end Table_Estimate;

   procedure Weigh_Across_Joints (M : in out Model; A : Arm_Id) is
   begin
      --  A decider's call: noted in the recording, so a replay ends the sweep where the boot did (Replay_Note).
      --  Without it a replay never fitted an arm across its joints, and its last fit was not the boot's (rec315
      --  replayed, 10-26: RB-Y1 seed 1a's arm2_joints not fitted, and fitted -0.1 % once it was; 10 arms within 2 %
      --  of 19, against 9).
      Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("across" & A'Image));
      for R of M.Kinematics loop
         if R.Arm = A then
            R.Across_Due := True;
            --  Its answers count as changed, so the next estimate fits it again (Refit).
            R.Revision := R.Revision + 1;
         end if;
      end loop;
   end Weigh_Across_Joints;

   procedure Refit (M : in out Model; Again : Boolean := False) is
      Start : constant Duration := Driver.Clock.Seconds;
      Fitted : Duration;

      --  Each arm's fit reads the model and writes nothing but its own evidence's result: the arms are fitted
      --  at once, and the world is placed from them afterwards.
      procedure Fit_Arm (Index : Positive) is
         R : Arm_Evidence renames M.Kinematics (Index);
      begin
         if not Current (M, R) then
            R.Result := (others => <>);
         elsif not R.Matches.Is_Empty and then Natural (R.Matches.Length) + R.Revision /= R.Result.Matches
           and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index
         then
            declare
               N       : constant Natural := M.Groups (R.Group).Size;
               Frames  : constant Natural := Natural (R.Frames.Length);
               Queries : constant Natural := Natural (R.Query_U.Length);
               Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
               --  Per keyframe and channel: the sweep asked the joint to turn for it (Keyframe.Asked; a frame with
               --  none asked nothing).
               Asked   : Flags_Access := new Fit.Flag_Matrix'(1 .. Frames => [1 .. N => False]);
               --  The readings' way from the reference keyframe on (Fit's Play): per channel the stretches its readings
               --  went one way, a keyframe's beat closing one, and how many came before each keyframe. A servo's gears
               --  let the joint stand within its slack of its reading, the slack going with the reading's every change
               --  (a play operator); read as where it stands, the fake's seed 20 (10-22, every quirk on) left its fit's
               --  sightings 0.59 px off and its focal length 8 % off, against 0.11 px and 0.8 % with no slack. Taken up
               --  whole on the side it was last turned from (a half slack times that side, against the reference's),
               --  seed 26's wrist stood 4.07 units off that, of the 17.7 of its slack: its reference stood within it,
               --  and pushes smaller than the slack took up a part of it.
               Play    : Play_Access;
               Visible : Real_Array (1 .. N);
               Sigma   : constant Real := Match_Sigma (R);
            begin
               --  The fit judges a keyframe's joints by the keyframe step, which rests on how far the view moves
               --  per unit: measured from these answers (the aligner's measure it as it aligns; answers that came
               --  otherwise, an instrument's or a test's, are measured here).
               Measure_Shifts (M, R);
               declare
                  Text : Ada.Strings.Unbounded.Unbounded_String;
               begin
                  for C in 1 .. N loop
                     Ada.Strings.Unbounded.Append
                       (Text, " c" & C'Image & " "
                        & (if C <= Natural (R.Shift_Sigmas.Length) and then R.Shift_Sigmas (C - 1) < Real'Last
                           then Driver.Log.Image (R.Shifts (C - 1), 4) & " +- " & Driver.Log.Image (R.Shift_Sigmas (C - 1), 4)
                                & " over" & Natural'Image (R.Shift_Points (C - 1)) & " points"
                           else "none")
                        & ", step " & Driver.Log.Image (Step_Of (M, R.Arm, C), 4));
                  end loop;
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " before its fit, px a unit (keyframes)"
                                   & " and the step a keyframe counts from:" & Ada.Strings.Unbounded.To_String (Text));
               end;
               for F in 1 .. Frames loop
                  for C in 1 .. N loop
                     Changes (F, C) := R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1);
                     Asked (F, C) := C - 1 < Natural (R.Frames (F).Asked.Length) and then R.Frames (F).Asked (C - 1);
                  end loop;
               end loop;
               declare
                  type Stretch_Lists is array (1 .. N) of Real_Vectors.Vector;
                  Lists  : Stretch_Lists;
                  type Count_Grid is array (1 .. Frames, 1 .. N) of Natural;
                  Counts : Count_Grid := [others => [others => 0]];
                  Order  : array (1 .. Frames) of Positive;   --  the keyframes by their beats
                  Most   : Natural := 0;
               begin
                  for F in 1 .. Frames loop
                     Order (F) := F;
                  end loop;
                  for I in 2 .. Frames loop
                     declare
                        F : constant Positive := Order (I);
                        P : Natural := I - 1;
                     begin
                        while P >= 1 and then R.Frames (Order (P)).Beat > R.Frames (F).Beat loop
                           Order (P + 1) := Order (P);
                           P := P - 1;
                        end loop;
                        Order (P + 1) := F;
                     end;
                  end loop;
                  for C in 1 .. N loop
                     declare
                        From : constant Natural := R.Frames (Order (1)).Beat;
                        Last : Real := R.Frames (Order (1)).Readings (C - 1);
                        Run  : Real := 0.0;   --  the stretch under way: the readings' change since its start
                        I    : Positive := 1;   --  the next keyframe by its beat
                     begin
                        while I <= Frames and then R.Frames (Order (I)).Beat <= From loop
                           I := I + 1;
                        end loop;
                        for B in From + 1 .. R.Frames (Order (Frames)).Beat loop
                           if Channels.Has_Reading (M, R.Group, B) then
                              declare
                                 Now : constant Real := Channels.Reading (M, R.Group, B, C);
                                 D   : constant Real := Now - Last;
                              begin
                                 if D /= 0.0 and then Run /= 0.0 and then (D > 0.0) /= (Run > 0.0) then
                                    Lists (C).Append (Run);
                                    Run := 0.0;
                                 end if;
                                 Run := Run + D;
                                 Last := Now;
                              end;
                           end if;
                           while I <= Frames and then R.Frames (Order (I)).Beat <= B loop
                              if Run /= 0.0 then
                                 Lists (C).Append (Run);
                                 Run := 0.0;
                              end if;
                              Counts (Order (I), C) := Natural (Lists (C).Length);
                              I := I + 1;
                           end loop;
                        end loop;
                     end;
                     Most := Natural'Max (Most, Natural (Lists (C).Length));
                  end loop;
                  Play := new Fit.Play_Paths (Stretches => Most, Frames => Frames, Joints => N);
                  Play.Path := [others => [others => 0.0]];
                  for C in 1 .. N loop
                     for K in 1 .. Natural (Lists (C).Length) loop
                        Play.Path (K, C) := Lists (C) (K - 1);
                     end loop;
                     for F in 1 .. Frames loop
                        Play.Upto (F, C) := Real (Counts (F, C));
                     end loop;
                  end loop;
               end;
               --  The turn the fit counts (Counted_Turn): no nearer than two landings of one asked pose can differ.
               for C in 1 .. N loop
                  Visible (C) := Counted_Turn (M, R.Arm, C);
               end loop;
               declare
                  function Round_Trip (S : Match_Set; I : Natural) return Boolean is
                    (S.Found (I) and then Moving_Point (R, I)
                     and then (Sigma = 0.0
                               or else (not Driver.Uncertain.Significant (S.Back_U (I) - R.Query_U (I), Sigma)
                                        and then not Driver.Uncertain.Significant (S.Back_V (I) - R.Query_V (I), Sigma))));

                  --  A keyframe tells the fit something only when its points
                  --  moved by more than the matcher errs: the median of their
                  --  displacements significant against a round trip's noise.
                  function Moved (S : Match_Set) return Boolean is
                     D : Real_Access := new Real_Array (1 .. Queries);
                     K : Natural := 0;
                  begin
                     for I in 0 .. Queries - 1 loop
                        if Round_Trip (S, I) then
                           K := K + 1;
                           D (K) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                             ((S.To_U (I) - R.Query_U (I)) ** 2 + (S.To_V (I) - R.Query_V (I)) ** 2);
                        end if;
                     end loop;
                     return Result : constant Boolean :=
                       K > 0 and then Driver.Uncertain.Significant (Driver.Stats.Median (D (1 .. K)), Sigma)
                     do
                        Free (D);
                     end return;
                  end Moved;

                  Moving : Flag_Access := new Flag_Array (R.Matches.First_Index .. R.Matches.Last_Index);

                  function Returns (S : Match_Set; I : Natural) return Boolean is
                    (Round_Trip (S, I) and then (for some K in Moving'Range => Moving (K) and then R.Matches (K).Frame = S.Frame));
                  Kept : Natural := 0;
               begin
                  for K in Moving'Range loop
                     Moving (K) := Moved (R.Matches (K));
                  end loop;
                  for S of R.Matches loop
                     for I in 0 .. Queries - 1 loop
                        if Returns (S, I) then
                           Kept := Kept + 1;
                        end if;
                     end loop;
                  end loop;
                  declare
                     type Sighting_Access is access Fit.Sighting_Array;
                     procedure Free is new Ada.Unchecked_Deallocation (Fit.Sighting_Array, Sighting_Access);
                     Seen   : Sighting_Access := new Fit.Sighting_Array (1 .. Kept);
                     K      : Natural := 0;
                     Slack  : Real_Access;     --  the joints' slack as measured, two a joint (Fit.Measure_Slack)
                     Linked : Matrix_Access;   --  the keyframes' changes less the slack's
                     Joints : Fit.Joint_Array (1 .. N);
                     Chain  : Fit.Chain_Order (1 .. N);
                     Lens   : Fit.Lens;
                     Report : Fit.Fit_Report;
                     Result : Arm_Fit;
                     --  Begun from the arm's fit when it has one: within the estimate whose fit holds (Again), its last
                     --  stage alone; in a later estimate, the stages after the search (Fit_Start.Whole).
                     Start  : Fit.Fit_Start (if R.Result.Fitted and then Natural (R.Result.Joints.Length) = N
                                             and then Natural (R.Result.Chain.Length) = N then N else 0);
                  begin
                     if Start.N > 0 then
                        Start.Valid := True;
                        for J in 1 .. N loop
                           declare
                              F : constant Joint_Fit := R.Result.Joints (R.Result.Joints.First_Index + J - 1);
                           begin
                              Start.Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                              Start.Chain (J) := R.Result.Chain (R.Result.Chain.First_Index + J - 1);
                           end;
                        end loop;
                        Start.L := (Fx => R.Result.Lens.Fx, Fy => R.Result.Lens.Fy, Cx => R.Result.Lens.Cx,
                                    Cy => R.Result.Lens.Cy, K1 => R.Result.Lens.K1, K2 => R.Result.Lens.K2);
                        Start.Focal_Within := R.Result.Focal_Within;
                        Start.Searched_Least := R.Result.Searched_Least;
                        Start.Searched_Most := R.Result.Searched_Most;
                        Start.Profiled_Over := R.Result.Profiled_Over;
                        Start.Order_Told := R.Result.Order_Told;
                        for P of R.Result.Untold_Orders loop
                           Start.Untold_Orders.Append (P);
                        end loop;
                        if Natural (R.Result.Slide_Turns.Length) = 3 * N then
                           for J in 1 .. N loop
                              for A in 1 .. 3 loop
                                 Start.Slide_Turns (J) (A) :=
                                   R.Result.Slide_Turns (R.Result.Slide_Turns.First_Index + 3 * (J - 1) + A - 1);
                              end loop;
                           end loop;
                        end if;
                        for J of R.Result.Turns_Told loop
                           if J in 1 .. N then
                              Start.Turns_Told (J) := True;
                           end if;
                        end loop;
                        --  A fit of an estimate before this one is begun from at the stages after the search; within
                        --  the estimate whose fit holds (Again), at its last stage alone.
                        Start.Whole := not (Again and then R.Result.Made_At = M.Beats);
                     end if;
                     --  The joints' slack measured from the keyframes' sightings alone (Fit.Measure_Slack), before the
                     --  sightings are taken less the motion of the joints their keyframes did not ask: that motion is
                     --  of the joints' turns, the readings' changes less the slack's (Linked), and so is the fit's.
                     for S of R.Matches loop
                        for I in 0 .. Queries - 1 loop
                           if Returns (S, I) then
                              K := K + 1;
                              Seen (K) := (Frame => S.Frame, Track => I + 1,
                                           U0 => R.Query_U (I), V0 => R.Query_V (I),
                                           U => S.To_U (I), V => S.To_V (I),
                                           Own => (if I < Natural (S.Sigma.Length) then S.Sigma (I) else 0.0),
                                           others => <>);
                           end if;
                        end loop;
                     end loop;
                     K := 0;
                     --  Measured where the arm has no fit of its own reference yet; else its last fit's, refined over every
                     --  keyframe by the whole model, where the measure, a line through each track, loses a slack among the
                     --  large turns' bends: seed 26 (10-22) measured its wrist's 8.84 as 7.51 from its first 50 keyframes
                     --  and as nought from 79.
                     if R.Result.Fitted and then R.Result.Reference_Beat = R.Frames (1).Beat
                       and then Natural (R.Result.Joints.Length) = N
                     then
                        Slack := new Real_Array'
                          ([for P in 1 .. 2 * N =>
                              (if P mod 2 = 1 then R.Result.Joints (R.Result.Joints.First_Index + (P - 1) / 2).Slack
                               else R.Result.Joints (R.Result.Joints.First_Index + (P - 1) / 2).Slack_Place)]);
                     else
                        Slack := new Real_Array'(Fit.Measure_Slack (Changes.all, Seen.all, Play.all));
                     end if;
                     Linked := new Driver.Numerics.Arrays.Real_Matrix'(Changes.all);
                     if Slack'Length = 2 * N then
                        declare
                           Parts : constant Driver.Numerics.Arrays.Real_Matrix := Fit.Play_Parts (Play.all, Slack.all);
                        begin
                           for F in 1 .. Frames loop
                              for C in 1 .. N loop
                                 Linked (F, C) := Changes (F, C) - Parts (F, C);
                              end loop;
                           end loop;
                        end;
                        declare
                           Text : Ada.Strings.Unbounded.Unbounded_String;
                        begin
                           for C in 1 .. N loop
                              Ada.Strings.Unbounded.Append
                                (Text, " c" & C'Image & " " & Driver.Log.Image (Slack (2 * C - 1), 4) & " at "
                                       & Driver.Log.Image (Slack (2 * C), 4));
                           end loop;
                           Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " slack begun from, of its"
                                            & Frames'Image & " keyframes, half slack at the reference's place, in reading"
                                            & " units:" & Ada.Strings.Unbounded.To_String (Text));
                        end;
                     end if;
                     for S of R.Matches loop
                        for I in 0 .. Queries - 1 loop
                           if Returns (S, I) then
                              K := K + 1;
                              Seen (K) := (Frame => S.Frame, Track => I + 1,
                                           U0 => R.Query_U (I), V0 => R.Query_V (I),
                                           U => S.To_U (I), V => S.To_V (I),
                                           Own => (if I < Natural (S.Sigma.Length) then S.Sigma (I) else 0.0),
                                           Own_Freedom => (if I < Natural (S.Freedom.Length) then S.Freedom (I) else 0),
                                           others => <>);
                              --  How far the joints the keyframe did not ask moved the point (Fit.Sighting's Off_U,
                              --  Off_V): each that stands off the reference by more than its noise tells and by enough
                              --  to move the view beyond what the matcher errs, times the point's motion per unit of
                              --  it (Measure_Shifts). A servo arm's every keyframe has some (f15w on 10-15, arm 2: the
                              --  joints not asked stood 1 to 4 units off, moving the view 1 to 6 px against the
                              --  asked joint's 4 to 50, and its one-joint stages scored every focal length alike).
                              for C in 1 .. N loop
                                 declare
                                    Off   : constant Real := Linked (S.Frame, C);
                                    Index : constant Natural := (C - 1) * Queries + I;
                                 begin
                                    if not Asked (S.Frame, C)
                                      and then (if Channels.Noise (M, R.Group, C) < Real'Last
                                                then Driver.Uncertain.Significant
                                                       (Off, Channels.Noise (M, R.Group, C) * Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0))
                                                else Off /= 0.0)
                                      and then (Sigma = 0.0
                                                or else Driver.Uncertain.Significant (Off * Pixels_Per_Unit (M, R, C), Sigma))
                                    then
                                       if Index < Natural (R.Point_G_Told.Length) and then R.Point_G_Told (Index) then
                                          Seen (K).Off_U := Seen (K).Off_U + Off * R.Point_Gu (Index);
                                          Seen (K).Off_V := Seen (K).Off_V + Off * R.Point_Gv (Index);
                                       else
                                          Seen (K).Off_Told := False;
                                       end if;
                                    end if;
                                 end;
                              end loop;
                           end if;
                        end loop;
                     end loop;
                     --  The unit of length is the first fit's (Fit's Unit_Frames): a
                     --  keyframe taken after it refines every term, and moves none of
                     --  the lengths the world was measured in.
                     --  A joint the keyframes never turned by what the fit counts as a turn (Counted_Turn) tells the fit
                     --  nothing of itself: it is held at its reference, rigid, out of the fit and out of the order of the
                     --  joints, and the arm is fitted by the joints that turned; its own terms stand at nought, as its
                     --  turn does while its reading stays where the keyframes had it. s19 (servo seed 4, 10-17): the wrist
                     --  (channel 3) moved nothing any eye saw up to where it stopped following, the sweep left it, and
                     --  every fit ended at its stage 0 ("a joint has no keyframe of its own"): the arm was never fitted.
                     --  So is a joint behind the eye, as the sweep tells one (Boot's Unseen): the points that move with it
                     --  are a patch of the view (Shift_Unseen), and the lock-in never saw the view move whole with it. It
                     --  turns the hand in the eye's picture and moves no camera, and its reading changes as other joints
                     --  are pushed turned it in their keyframes: the Shadow's sixth channel, its sweep stopped as behind
                     --  the eye at its first levels, had no keyframe of its own, and every fit after its sweep ended at
                     --  stage 0 (seeds 1b and 2b, round p373s2, 10-25); held out, their last fits' inputs fitted, focal
                     --  +2.1 and -1.5 %.
                     declare
                        Turned : array (1 .. N) of Boolean := [others => False];
                        Count  : Natural := 0;
                        function Behind_The_Eye (C : Positive) return Boolean is
                          (C <= Natural (R.Shift_Seen.Length) and then not R.Shift_Seen (C - 1)
                           and then (R.Eye > M.Graph.Channels.Last_Index
                                     or else not (for some Item of M.Graph.Channels (R.Eye) =>
                                                    Item.Group = R.Group and then Item.Channel = C
                                                    and then Item.Effect.Verdict = Driver.Robot.Whole)));
                     begin
                        for C in 1 .. N loop
                           for F in 1 .. Frames loop
                              if abs Changes (F, C) >= Visible (C) and then not Behind_The_Eye (C) then
                                 Turned (C) := True;
                              end if;
                           end loop;
                           if Turned (C) then
                              Count := Count + 1;
                           end if;
                        end loop;
                        --  A joint turned now that the fit begun from held (out of that fit, its turn per reading unit
                        --  nought there): that fit has nothing of it to begin from, and this one begins from nothing.
                        --  Begun from it, stage 1 took the held joint's nought for its turn and refused the arm, and every
                        --  fit after it the same, the arm left at its fit from before the joint turned: t502 of the race
                        --  (10-23), its channel 4 first turned in pass 1, the arm not fitted again over 58 keyframes.
                        if Start.N > 0 then
                           Start.Valid := Start.Valid
                             and then (for all C in 1 .. N =>
                                         not Turned (C) or else Start.Joints (C).Slide
                                         or else abs Start.Joints (C).C > 0.0);
                        end if;
                        if Count = N or else Count = 0 then
                           Fit.Fit (Changes.all, Asked.all, Visible, Seen.all, M.Eyes (R.Eye).Grid.Width,
                                    M.Eyes (R.Eye).Grid.Height, R.Result.Unit_Frames, Joints, Chain, Lens, Report,
                                    Who => "arm" & R.Arm'Image, From => Start, Play => Play.all,
                                    Slack_Start => Slack.all, Across_Joints => R.Across_Due);
                        else
                           declare
                              Index     : array (1 .. Count) of Positive;   --  the channel of each joint fitted
                              Into      : array (1 .. N) of Natural := [others => 0];   --  and back, 0 for one held
                              K         : Natural := 0;
                              Changes_R : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. Count);
                              Asked_R   : Flags_Access := new Fit.Flag_Matrix'(1 .. Frames => [1 .. Count => False]);
                              Play_R    : Play_Access :=
                                new Fit.Play_Paths (Stretches => Play.Stretches, Frames => Frames, Joints => Count);
                              Visible_R : Real_Array (1 .. Count);
                              Joints_R  : Fit.Joint_Array (1 .. Count);
                              Chain_R   : Fit.Chain_Order (1 .. Count);
                              Start_R   : Fit.Fit_Start (if Start.N > 0 then Count else 0);
                              Held      : Ada.Strings.Unbounded.Unbounded_String;
                           begin
                              for C in 1 .. N loop
                                 if Turned (C) then
                                    K := K + 1;
                                    Index (K) := C;
                                    Into (C) := K;
                                 else
                                    Ada.Strings.Unbounded.Append (Held, " c" & C'Image);
                                 end if;
                              end loop;
                              for F in 1 .. Frames loop
                                 for J in 1 .. Count loop
                                    Changes_R (F, J) := Changes (F, Index (J));
                                    Asked_R (F, J) := Asked (F, Index (J));
                                    Play_R.Upto (F, J) := Play.Upto (F, Index (J));
                                 end loop;
                              end loop;
                              for J in 1 .. Count loop
                                 Visible_R (J) := Visible (Index (J));
                                 for K in 1 .. Play.Stretches loop
                                    Play_R.Path (K, J) := Play.Path (K, Index (J));
                                 end loop;
                              end loop;
                              if Start.N > 0 then
                                 Start_R.Valid := Start.Valid;
                                 for J in 1 .. Count loop
                                    Start_R.Joints (J) := Start.Joints (Index (J));
                                 end loop;
                                 K := 0;
                                 for P of Start.Chain loop
                                    if Into (P) > 0 then
                                       K := K + 1;
                                       Start_R.Chain (K) := Into (P);
                                    end if;
                                 end loop;
                                 Start_R.L := Start.L;
                                 Start_R.Focal_Within := Start.Focal_Within;
                                 Start_R.Searched_Least := Start.Searched_Least;
                                 Start_R.Searched_Most := Start.Searched_Most;
                                 Start_R.Whole := Start.Whole;
                                 Start_R.Profiled_Over := Start.Profiled_Over;
                                 Start_R.Order_Told := Start.Order_Told;
                                 for J in 1 .. Count loop
                                    Start_R.Turns_Told (J) := Start.Turns_Told (Index (J));
                                 end loop;
                                 --  Each untold order as the reduced chain has it: its turned joints' places, in order.
                                 for W in 0 .. Natural (Start.Untold_Orders.Length) / N - 1 loop
                                    for J in 1 .. N loop
                                       declare
                                          C : constant Positive :=
                                            Start.Untold_Orders (Start.Untold_Orders.First_Index + W * N + J - 1);
                                       begin
                                          if Into (C) > 0 then
                                             Start_R.Untold_Orders.Append (Into (C));
                                          end if;
                                       end;
                                    end loop;
                                 end loop;
                              end if;
                              Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " holds"
                                               & Ada.Strings.Unbounded.To_String (Held) & " at the reference: no keyframe"
                                               & " turned it by what the fit counts as a turn; the arm is fitted by the"
                                               & Count'Image & " that turned");
                              Fit.Fit (Changes_R.all, Asked_R.all, Visible_R, Seen.all, M.Eyes (R.Eye).Grid.Width,
                                       M.Eyes (R.Eye).Grid.Height, R.Result.Unit_Frames, Joints_R, Chain_R, Lens, Report,
                                       Who => "arm" & R.Arm'Image, From => Start_R, Play => Play_R.all,
                                       Slack_Start =>
                                         (if Slack'Length = 2 * N
                                          then [for K in 1 .. 2 * Count => Slack (2 * Index ((K + 1) / 2) - 1 + (K - 1) mod 2)]
                                          else Fit.No_Slack),
                                       Across_Joints => R.Across_Due);
                              --  Back to every channel: the joints fitted where they are, the held ones rigid at the end
                              --  of the chain, their terms nought.
                              for C in 1 .. N loop
                                 Joints (C) := (if Into (C) > 0 then Joints_R (Into (C))
                                                else (W => [0.0, 0.0, 1.0], P => Zero3, C => 0.0, Slide => False));
                              end loop;
                              --  And their slacks: the held ones' none.
                              declare
                                 Slacks : constant Fit.Real_Lists.Vector := Report.Slacks;
                                 Places : constant Fit.Real_Lists.Vector := Report.Slack_Places;
                              begin
                                 if Natural (Slacks.Length) = Count and then Natural (Places.Length) = Count then
                                    Report.Slacks.Clear;
                                    Report.Slack_Places.Clear;
                                    for C in 1 .. N loop
                                       Report.Slacks.Append (if Into (C) > 0 then Slacks (Into (C)) else 0.0);
                                       Report.Slack_Places.Append (if Into (C) > 0 then Places (Into (C)) else 0.0);
                                    end loop;
                                 end if;
                              end;
                              K := 0;
                              for P of Chain_R loop
                                 K := K + 1;
                                 Chain (K) := Index (P);
                              end loop;
                              for C in 1 .. N loop
                                 if Into (C) = 0 then
                                    K := K + 1;
                                    Chain (K) := C;
                                 end if;
                              end loop;
                              declare
                                 Small : constant Natural := Fit.Terms (Count);
                                 Whole : constant Natural := Fit.Terms (N);
                                 --  The term of the whole arm a term of the fitted joints is.
                                 function Term_Of (T : Positive) return Positive is
                                   (if T <= Fit.Lens_Terms then T
                                    else Fit.Lens_Terms + Fit.Joint_Terms * (Index ((T - Fit.Lens_Terms - 1) / Fit.Joint_Terms + 1) - 1)
                                         + (T - Fit.Lens_Terms - 1) mod Fit.Joint_Terms + 1);
                                 Was : constant Fit.Real_Lists.Vector := Report.Covariance;
                                 Gains : constant Fit.Real_Lists.Vector := Report.Depth_Gains;
                                 Grid : array (1 .. Whole, 1 .. Whole) of Real := [others => [others => 0.0]];
                              begin
                                 if Natural (Was.Length) = Small * Small then
                                    for I in 1 .. Small loop
                                       for J in 1 .. Small loop
                                          Grid (Term_Of (I), Term_Of (J)) := Was (Was.First_Index + (I - 1) * Small + J - 1);
                                       end loop;
                                    end loop;
                                    Report.Covariance.Clear;
                                    for I in 1 .. Whole loop
                                       for J in 1 .. Whole loop
                                          Report.Covariance.Append (Grid (I, J));
                                       end loop;
                                    end loop;
                                 end if;
                                 if Small > 0 and then Natural (Gains.Length) mod Small = 0 then
                                    Report.Depth_Gains.Clear;
                                    for Track in 1 .. Natural (Gains.Length) / Small loop
                                       declare
                                          Row : array (1 .. Whole) of Real := [others => 0.0];
                                       begin
                                          for T in 1 .. Small loop
                                             Row (Term_Of (T)) := Gains (Gains.First_Index + (Track - 1) * Small + T - 1);
                                          end loop;
                                          for X of Row loop
                                             Report.Depth_Gains.Append (X);
                                          end loop;
                                       end;
                                    end loop;
                                 end if;
                              end;
                              declare
                                 Pairs : constant Joint_Pair_Lists.Vector := Report.Untold;
                                 Ways  : constant Fit.Joint_Lists.Vector := Report.Untold_Orders;
                              begin
                                 Report.Untold.Clear;
                                 for P of Pairs loop
                                    Report.Untold.Append (Joint_Pair'(Earlier => Index (P.Earlier), Later => Index (P.Later)));
                                 end loop;
                                 declare
                                    Told : constant Fit.Joint_Lists.Vector := Report.Turns_Told;
                                 begin
                                    Report.Turns_Told.Clear;
                                    for J of Told loop
                                       Report.Turns_Told.Append (Index (J));
                                    end loop;
                                 end;
                                 --  Each untold order as the chain is: the turned joints' channels in it, then the held ones.
                                 Report.Untold_Orders.Clear;
                                 for W in 0 .. Natural (Ways.Length) / Count - 1 loop
                                    for J in 1 .. Count loop
                                       Report.Untold_Orders.Append (Index (Ways (Ways.First_Index + W * Count + J - 1)));
                                    end loop;
                                    for C in 1 .. N loop
                                       if Into (C) = 0 then
                                          Report.Untold_Orders.Append (C);
                                       end if;
                                    end loop;
                                 end loop;
                              end;
                              Free (Changes_R);
                              Free (Play_R);
                              Free (Asked_R);
                           end;
                        end if;
                     end;
                     --  The table its eye sees, away from it towards the eye,
                     --  and its tracks' points: Up is the first arm's (the world
                     --  is that eye's reference frame), and the points place the
                     --  other arms (Place).
                     if Report.Fitted then
                        declare
                           Plane  : Fit.Sight_Plane;
                           Sights : Sight_Access := new Fit.Sight_Point_Array (1 .. Queries);
                           On     : Fit_Flag_Access := new Fit.Flag_Array (1 .. Queries);
                        begin
                           --  The points where the fit put them: each track's refined
                           --  depth along its reference line of sight.
                           for I in 0 .. Queries - 1 loop
                              declare
                                 D : constant Real :=
                                   (if I < Natural (Report.Depths.Length) then Report.Depths (Report.Depths.First_Index + I)
                                    else 0.0);
                                 S : constant Real :=
                                   (if I < Natural (Report.Depth_Sigmas.Length)
                                    then Report.Depth_Sigmas (Report.Depth_Sigmas.First_Index + I) else Real'Last);
                                 H : constant Vec3 := Fit.Ray (Lens, R.Query_U (I), R.Query_V (I));
                              begin
                                 Result.Track_Known.Append (Fit.Depth_Known (D, S));
                                 Result.Track_Sigmas.Append (S);
                                 for X of H loop
                                    Result.Tracks.Append (D * X);
                                 end loop;
                                 Sights (I + 1) := (H => H, Depth => D, Sigma => S);
                              end;
                           end loop;
                           --  How every depth moves with the fit's terms together, which the table's uncertainty carries
                           --  (Table_Estimate). The fit has a track to the last query that has a sighting: the
                           --  queries after it, that no keyframe answered, have no depth (above) and none that
                           --  moves with a term.
                           declare
                              Terms : constant Natural :=
                                Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                           (Real (Natural (Report.Covariance.Length))));
                           begin
                              while Natural (Report.Depth_Gains.Length) < Queries * Terms loop
                                 Report.Depth_Gains.Append (0.0);
                              end loop;
                              for X of Report.Depth_Gains loop
                                 Result.Track_Gains.Append (X);
                              end loop;
                           end;
                           --  The table: the plane most of them lie on, with its
                           --  whole uncertainty (Table_In_Arm).
                           Fit.Dominant_Plane (Sights.all, Plane, On.all);
                           Result.Table_Own := Plane.Covariance;
                           for B of On.all loop
                              Result.Table_On.Append (B);
                           end loop;
                           Table_Estimate (R.Query_U, R.Query_V, Result, Lens, Sights.all, Plane,
                                           Report.Depth_Gains, Report.Covariance, Result.Table,
                                           Result.Table_Response);
                           --  For the arm's placement: the plane as its eye sees it, with its points' scatter and
                           --  what the fit moves every depth by together.
                           Plane.Covariance :=
                             Fit.Plane_Covariance (Sights.all, On.all, Plane, Report.Depth_Gains, Report.Covariance);
                           Result.Table_A := Plane.A;
                           Result.Table_Covariance := Plane.Covariance;
                           Free (Sights);
                           Free (On);
                        end;
                     end if;
                     Free (Seen);
                     Free (Slack);
                     Free (Linked);
                     Result.Fitted := Report.Fitted;
                     Result.Focal_Within := Report.Focal_Within;
                     Result.Searched_Least := Report.Searched_Least;
                     Result.Searched_Most := Report.Searched_Most;
                     Result.Profiled_Over := Report.Profiled_Over;
                     Result.Order_Told := Report.Order_Told;
                     --  The untold pairs of the fit that searched the orders; a fit begun from another keeps the ones before.
                     if Report.Order_Told then
                        Result.Untold.Clear;
                     elsif not Report.Untold.Is_Empty then
                        Result.Untold := Report.Untold;
                     else
                        Result.Untold := R.Result.Untold;
                     end if;
                     Result.Slide_Turns.Clear;
                     for X of Report.Slide_Turns loop
                        Result.Slide_Turns.Append (X);
                     end loop;
                     Result.Turns_Told.Clear;
                     for J of Report.Turns_Told loop
                        Result.Turns_Told.Append (J);
                     end loop;
                     Result.Untold_Turns.Clear;
                     if not Report.Order_Told then
                        if not Report.Untold.Is_Empty then
                           for X of Report.Untold_Turns loop
                              Result.Untold_Turns.Append (X);
                           end loop;
                        else
                           Result.Untold_Turns := R.Result.Untold_Turns;
                        end if;
                     end if;
                     Result.Untold_Orders.Clear;
                     if not Report.Order_Told then
                        if not Report.Untold_Orders.Is_Empty then
                           for P of Report.Untold_Orders loop
                              Result.Untold_Orders.Append (P);
                           end loop;
                        else
                           Result.Untold_Orders := R.Result.Untold_Orders;
                        end if;
                     end if;
                     Result.Made_At := M.Beats;
                     Result.Unit_Frames :=
                       (if R.Result.Unit_Frames > 0 then R.Result.Unit_Frames elsif Report.Fitted then Frames else 0);
                     if R.Result.Unit_Frames > 0 and then not R.Result.Unit_Changes.Is_Empty then
                        Result.Unit_Changes := R.Result.Unit_Changes;
                     else
                        Result.Unit_Changes.Clear;
                        for F in 1 .. Result.Unit_Frames loop
                           for C in 1 .. N loop
                              Result.Unit_Changes.Append (R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1));
                           end loop;
                        end loop;
                     end if;
                     Result.Matches := Natural (R.Matches.Length) + R.Revision;
                     Result.Used := Report.Used;
                     Result.Median_Px := Report.Median_Px;
                     Result.Sigma_Px := Report.Sigma_Px;
                     Result.Why := Report.Why;
                     Result.Reference := R.Frames (1).Readings;
                     Result.Reference_Beat := R.Frames (1).Beat;
                     for K in Joints'Range loop
                        declare
                           J : constant Natural := K - Joints'First;   --  from nought
                        begin
                           Result.Joints.Append
                             (Joint_Fit'(W => Joints (K).W, P => Joints (K).P, C => Joints (K).C, Slide => Joints (K).Slide,
                                         Slack       =>
                                           (if J < Natural (Report.Slacks.Length)
                                            then Report.Slacks (Report.Slacks.First_Index + J) else 0.0),
                                         Slack_Place =>
                                           (if J < Natural (Report.Slack_Places.Length)
                                            then Report.Slack_Places (Report.Slack_Places.First_Index + J) else 0.0)));
                        end;
                     end loop;
                     for P of Chain loop
                        Result.Chain.Append (P);
                     end loop;
                     Result.Lens := (Fx => Lens.Fx, Fy => Lens.Fy, Cx => Lens.Cx, Cy => Lens.Cy, K1 => Lens.K1, K2 => Lens.K2);
                     for X of Report.Covariance loop
                        Result.Covariance.Append (X);
                     end loop;
                     --  A fit that failed keeps the last one that held.
                     if Report.Fitted or else not R.Result.Fitted then
                        R.Result := Result;
                     else
                        R.Result.Matches := Result.Matches;
                        R.Result.Why := Result.Why;
                     end if;
                     R.Across_Due := False;
                     Driver.Log.Line
                       (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " "
                        & (if Report.Fitted
                           then "fitted from" & Kept'Image & " sightings of" & Frames'Image & " keyframes,"
                                & Report.Used'Image & " fit, median " & Driver.Log.Image (Report.Median_Px, 3)
                                & " px, noise " & Driver.Log.Image (Report.Sigma_Px, 3) & " px and "
                               & Driver.Log.Image (Report.Own_Gain, 2) & " of what the matcher states; focal "
                                & Driver.Log.Image (Lens.Fx, 2) & " x " & Driver.Log.Image (Lens.Fy, 2) & " px"
                                & (if Report.Focal_Sigma < Real'Last then " +- " & Driver.Log.Image (Report.Focal_Sigma, 2)
                                   & " px" else "")
                                & "; its joints from the base, by their readings: " & Chain_Image (Chain)
                                & Slacks_Image (Report)
                                & (if Report.Errors.Measured
                                   then "; errors, px: a sighting's own "
                                        & Driver.Log.Image (Report.Errors.Alone, 3) & ", a point's in every keyframe "
                                        & Driver.Log.Image (Report.Errors.Persistent, 3) & " (half as alike at "
                                        & Driver.Log.Image (Report.Errors.Persistent_Half, 0)
                                        & " px apart), a keyframe's added for its points "
                                        & Driver.Log.Image (Report.Errors.Keyframe, 3) & "; the clip took "
                                        & Driver.Log.Image (Report.Errors.Clipped, 3)
                                   else "")
                           else "not fitted (stage" & Report.Stage'Image & "): "
                                & Ada.Strings.Unbounded.To_String (Report.Why))
                        & Stages_Image (Report));
                  end;
                  Free (Moving);
               end;
               Free (Changes);
               Free (Asked);
               Free (Play);
            end;
         end if;
      end Fit_Arm;

      procedure Fit_Arms is new Driver.Parallel_For (Fit_Arm);
   begin
      Fit_Arms (M.Kinematics.First_Index, M.Kinematics.Last_Index);
      Fitted := Driver.Clock.Seconds;
      Place (M);
      Driver.Log.Line (Driver.Log.Robot, "kinematics: the arms fitted in "
                       & Driver.Log.Image (Real (Fitted - Start), 1) & " s, placed in "
                       & Driver.Log.Image (Real (Driver.Clock.Seconds - Fitted), 1) & " s");
   end Refit;

   ---------------------------------------------------------------------------
   --  Eyes fixed in the world

   function Square_Root (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Sqrt;

   ---------------------------------------------------------------------------
   --  The arms placed in the world

   Outputs : constant := 7;   --  turn, centre, log scale

   type Output_Covariance is array (1 .. Outputs, 1 .. Outputs) of Real;
   type Output_Vector is array (1 .. Outputs) of Real;

   --  How well the arm's fit unit is known against its depths (Fit.Unit_Sigma):
   --  the unit's keyframes, the first fit's.
   function Unit_Sigma_Of (R : Arm_Evidence) return Real is
      N      : constant Natural := Natural (R.Result.Joints.Length);
      Frames : constant Natural := (if N = 0 then 0 else Natural (R.Result.Unit_Changes.Length) / N);
   begin
      if N = 0 or else Frames = 0 or else Natural (R.Result.Reference.Length) /= N
        or else Natural (R.Result.Chain.Length) /= N
      then
         return Real'Last;
      end if;
      declare
         Joints  : Fit.Joint_Array (1 .. N);
         Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
         Cov     : Fit.Real_Lists.Vector;
      begin
         for J in 1 .. N loop
            declare
               F : constant Joint_Fit := R.Result.Joints (J);
            begin
               Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
            end;
         end loop;
         for F in 1 .. Frames loop
            for C in 1 .. N loop
               Changes (F, C) := R.Result.Unit_Changes ((F - 1) * N + C - 1);
            end loop;
         end loop;
         for X of R.Result.Covariance loop
            Cov.Append (X);
         end loop;
         return Result : constant Real := Fit.Unit_Sigma (Joints, Chain_Of (R.Result), Changes.all, Cov) do
            Free (Changes);
         end return;
      end;
   end Unit_Sigma_Of;

   --  The axis along which V has its smallest component: a direction that is not nearly V's own.
   function Smallest_Axis (V : Vec3) return Vec3 is
     (if abs V (1) <= abs V (2) and then abs V (1) <= abs V (3) then [1.0, 0.0, 0.0]
      elsif abs V (2) <= abs V (3) then [0.0, 1.0, 0.0]
      else [0.0, 0.0, 1.0]);

   --  Arm 2 placed in arm 1's frame through the fixed eye that saw both turn (F: its fit, S1 and S2: the two arms'
   --  places in it, Fixed_Fit.Arms).
   --  The eye stands at C1 in arm 1's frame, turned R1, and at C2 in arm 2's, turned R2: an arm-2 point X2 is at the
   --  eye's frame R2^T (X2 - C2) in arm 2's unit and K times that in arm 1's, so X1 = R1 K R2^T (X2 - C2) + C1:
   --  turn R1 R2^T, scale K and centre C1 - K R1 R2^T C2. The two units are compared by the table: it is one plane,
   --  and the eye's distance from it (along its normal) is D1 units of arm 1 and D2 of arm 2 (a table plane is
   --  A.X = 1 in the arm's frame, the eye at C is (1 - A.C) / |A| from it), so K = D1 / D2. Both arms' tables must
   --  also be one plane to the eye: their normals, in the eye's frame, agree to within their sigmas.
   --  Covariance is that of (the turn about the world's axes, the centre, the logarithm of the scale), from the
   --  fit's covariance of both poses (their correlations through the lens kept), the two tables' planes, and how
   --  well each arm's unit is known against its depths (the eye's centre is in the joints' unit, the table in the
   --  depths').
   procedure Through
     (R1, R2     : Arm_Evidence;
      F          : Fixed_Fit;
      S1, S2     : Positive;
      Placement  : out Rigid;
      Scale      : out Real;
      Covariance : out Output_Covariance;
      Response   : out Output_Vector;
      Placed     : out Boolean;
      Why        : out Ada.Strings.Unbounded.Unbounded_String)
   is
      use Driver.Numerics.Arrays;
      Pose_Terms : constant := 6;   --  an arm's in the eye's fit: the camera's turn about its own axes, its centre
      Total : constant Natural := Natural (Square_Root (Real (Natural (F.Joint.Length))));
      O1    : constant Natural := Fit.Lens_Terms + Pose_Terms * (S1 - 1);
      O2    : constant Natural := Fit.Lens_Terms + Pose_Terms * (S2 - 1);
   begin
      Placement := Driver.Numerics.Identity;
      Scale := 1.0;
      Covariance := [others => [others => 0.0]];
      Response := [others => 0.0];
      Placed := False;
      Why := Ada.Strings.Unbounded.Null_Unbounded_String;
      if Total * Total /= Natural (F.Joint.Length) or else O1 + Pose_Terms > Total or else O2 + Pose_Terms > Total
        or else Natural (F.Poses.Length) < Natural'Max (S1, S2)
      then
         Why := Ada.Strings.Unbounded.To_Unbounded_String ("the eye's fit does not hold both arms' poses");
         return;
      end if;
      if not (Driver.Geometry.Known (R1.Result.Table) and then Driver.Geometry.Known (R2.Result.Table)) then
         Why := Ada.Strings.Unbounded.To_Unbounded_String ("an arm's table is not found");
         return;
      end if;
      declare
         Pose1 : constant Rigid := F.Poses (F.Poses.First_Index + S1 - 1);
         Pose2 : constant Rigid := F.Poses (F.Poses.First_Index + S2 - 1);
         A1    : constant Vec3 := R1.Result.Table_A;
         A2    : constant Vec3 := R2.Result.Table_A;
         Gap1  : constant Real := 1.0 - A1 * Pose1.Translation;
         Gap2  : constant Real := 1.0 - A2 * Pose2.Translation;
      begin
         if not (Gap1 > 0.0 and then Gap2 > 0.0) then
            Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("the eye lies on the far side of an arm's table plane from that arm's eye");
            return;
         end if;
         declare
            Norm1   : constant Real := abs A1;
            Norm2   : constant Real := abs A2;
            D1      : constant Real := Gap1 / Norm1;
            D2      : constant Real := Gap2 / Norm2;
            Kappa   : constant Real := D1 / D2;
            Rp      : constant Mat3 := Pose1.Rotation * Transpose (Pose2.Rotation);
            Lever   : constant Vec3 := Rp * Pose2.Translation;   --  the eye's centre in arm 2's frame, turned to arm 1's
            Spread  : constant Vec3 := Kappa * Lever;
            --  How the logarithm of each distance moves with the eye's centre and the plane.
            G_C1    : constant Vec3 := (-1.0 / Gap1) * A1;
            G_A1    : constant Vec3 := (-1.0 / Gap1) * Pose1.Translation - (1.0 / Norm1 ** 2) * A1;
            G_C2    : constant Vec3 := (-1.0 / Gap2) * A2;
            G_A2    : constant Vec3 := (-1.0 / Gap2) * Pose2.Translation - (1.0 / Norm2 ** 2) * A2;
            --  The inputs, in this order: the first pose's turn about the eye's axes and its centre, the second's, the
            --  first plane, the second; each a number of terms in.
            Turn_1  : constant := 0;
            Place_1 : constant := 3;
            Turn_2  : constant := 6;
            Place_2 : constant := 9;
            Plane_1 : constant := 12;
            Plane_2 : constant := 15;
            Inputs  : constant := 18;
            --  The outputs: the turn, then the centre, then the logarithm of the scale.
            Centre_Row : constant := 3;
            Sigma_U : Real_Matrix (1 .. Inputs, 1 .. Inputs) := [others => [others => 0.0]];
            J       : Real_Matrix (1 .. Outputs, 1 .. Inputs) := [others => [others => 0.0]];
            Rel1    : constant Real := Unit_Sigma_Of (R1);
            Rel2    : constant Real := Unit_Sigma_Of (R2);

            function Fit_Term (T : Positive) return Positive is
              (if T <= Pose_Terms then O1 + T else O2 + T - Pose_Terms);

            function Cv (P, Q : Positive) return Real is
              (F.Joint (F.Joint.First_Index + (Fit_Term (P) - 1) * Total + Fit_Term (Q) - 1));

            function Cv_Lens (A, B : Positive) return Real is
              (F.Joint (F.Joint.First_Index + (A - 1) * Total + B - 1));

            --  The covariance of pose term P (of the 12) with the logarithm of the focal length Lens_Term.
            function Cv_Pose_Lens (P, Lens_Term : Positive) return Real is
              (F.Joint (F.Joint.First_Index + (Fit_Term (P) - 1) * Total + Lens_Term - 1));

            Skewed : constant Mat3 := Kappa * (Driver.Numerics.Skew (Lever) * Pose1.Rotation);
         begin
            for P in 1 .. Plane_1 loop
               for Q in 1 .. Plane_1 loop
                  Sigma_U (P, Q) := Cv (P, Q);
               end loop;
            end loop;
            --  The centres are in the joints' unit, the tables in the depths': the unit's uncertainty.
            if Rel1 < Real'Last then
               for P in 1 .. 3 loop
                  for Q in 1 .. 3 loop
                     Sigma_U (Place_1 + P, Place_1 + Q) := Sigma_U (Place_1 + P, Place_1 + Q)
                       + Rel1 ** 2 * Pose1.Translation (P) * Pose1.Translation (Q);
                  end loop;
               end loop;
            end if;
            if Rel2 < Real'Last then
               for P in 1 .. 3 loop
                  for Q in 1 .. 3 loop
                     Sigma_U (Place_2 + P, Place_2 + Q) := Sigma_U (Place_2 + P, Place_2 + Q)
                       + Rel2 ** 2 * Pose2.Translation (P) * Pose2.Translation (Q);
                  end loop;
               end loop;
            end if;
            for P in 1 .. 3 loop
               for Q in 1 .. 3 loop
                  Sigma_U (Plane_1 + P, Plane_1 + Q) := R1.Result.Table_Covariance (P, Q);
                  Sigma_U (Plane_2 + P, Plane_2 + Q) := R2.Result.Table_Covariance (P, Q);
               end loop;
            end loop;
            --  The turn about the world's axes: R1 (theta1 - theta2).
            for A in 1 .. 3 loop
               for B in 1 .. 3 loop
                  J (A, Turn_1 + B) := Pose1.Rotation (A, B);
                  J (A, Turn_2 + B) := -Pose1.Rotation (A, B);
               end loop;
            end loop;
            --  The centre: C1 - K R_p C2.
            for A in 1 .. 3 loop
               for B in 1 .. 3 loop
                  J (Centre_Row + A, Turn_1 + B) := Skewed (A, B);
                  J (Centre_Row + A, Turn_2 + B) := -Skewed (A, B);
                  J (Centre_Row + A, Place_1 + B) := (if A = B then 1.0 else 0.0) - Spread (A) * G_C1 (B);
                  J (Centre_Row + A, Place_2 + B) := -Kappa * Rp (A, B) + Spread (A) * G_C2 (B);
                  J (Centre_Row + A, Plane_1 + B) := -Spread (A) * G_A1 (B);
                  J (Centre_Row + A, Plane_2 + B) := Spread (A) * G_A2 (B);
               end loop;
            end loop;
            --  The logarithm of the scale: ln D1 - ln D2.
            for B in 1 .. 3 loop
               J (Outputs, Place_1 + B) := G_C1 (B);
               J (Outputs, Place_2 + B) := -G_C2 (B);
               J (Outputs, Plane_1 + B) := G_A1 (B);
               J (Outputs, Plane_2 + B) := -G_A2 (B);
            end loop;
            declare
               Result : constant Real_Matrix := J * Sigma_U * Transpose (J);
            begin
               for P in 1 .. Outputs loop
                  for Q in 1 .. Outputs loop
                     Covariance (P, Q) := (Result (P, Q) + Result (Q, P)) / 2.0;
                  end loop;
               end loop;
            end;
            --  How the placement moves with the eye's lens: the regression of its outputs on the logarithms of the
            --  two focal lengths (their covariance with the lens's through the poses), for both focal lengths a
            --  hundredth too long.
            declare
               Fx_Fy : constant Real := Cv_Lens (1, 1) * Cv_Lens (2, 2) - Cv_Lens (1, 2) ** 2;
            begin
               if Fx_Fy > 0.0 then
                  declare
                     Cross : Real_Matrix (1 .. Outputs, 1 .. 2) := [others => [others => 0.0]];
                  begin
                     for Row in 1 .. Outputs loop
                        for Lens_Term in 1 .. 2 loop
                           for P in 1 .. Plane_1 loop
                              Cross (Row, Lens_Term) := Cross (Row, Lens_Term) + J (Row, P) * Cv_Pose_Lens (P, Lens_Term);
                           end loop;
                        end loop;
                     end loop;
                     for Row in 1 .. Outputs loop
                        Response (Row) :=
                          0.01 * ((Cross (Row, 1) * (Cv_Lens (2, 2) - Cv_Lens (1, 2))
                                   + Cross (Row, 2) * (Cv_Lens (1, 1) - Cv_Lens (1, 2))) / Fx_Fy);
                     end loop;
                  end;
               end if;
            end;
            --  The tables as one plane to the eye: its normals in the eye's frame, M1 and M2, differ by what their
            --  sigmas allow, across the line between them (two dimensions: the normals are unit).
            declare
               N1     : constant Vec3 := A1 / Norm1;
               N2     : constant Vec3 := A2 / Norm2;
               M1     : constant Vec3 := Transpose (Pose1.Rotation) * N1;
               M2     : constant Vec3 := Transpose (Pose2.Rotation) * N2;
               Mid    : constant Vec3 := M1 + M2;
               Gap_N  : constant Real := abs (M1 - M2);
            begin
               if abs Mid <= 0.0 then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("the two tables face opposite ways to the eye");
                  return;
               end if;
               declare
                  Across_1 : constant Vec3 := Unit (Cross (Mid, Smallest_Axis (Mid)));
                  Across_2 : constant Vec3 := Unit (Cross (Mid, Across_1));
                  Delta_N  : constant Vec3 := M1 - M2;
                  --  How the difference moves with the inputs (3 x 18).
                  Jd       : Real_Matrix (1 .. 3, 1 .. Inputs) := [others => [others => 0.0]];
                  P1       : constant Mat3 := Identity3 - Driver.Numerics.Outer (N1, N1);
                  P2       : constant Mat3 := Identity3 - Driver.Numerics.Outer (N2, N2);
                  Sk1      : constant Mat3 := Driver.Numerics.Skew (M1);
                  Sk2      : constant Mat3 := Driver.Numerics.Skew (M2);
                  By_A1    : constant Mat3 := (1.0 / Norm1) * (Transpose (Pose1.Rotation) * P1);
                  By_A2    : constant Mat3 := (1.0 / Norm2) * (Transpose (Pose2.Rotation) * P2);
               begin
                  for A in 1 .. 3 loop
                     for B in 1 .. 3 loop
                        Jd (A, Turn_1 + B) := Sk1 (A, B);
                        Jd (A, Turn_2 + B) := -Sk2 (A, B);
                        Jd (A, Plane_1 + B) := By_A1 (A, B);
                        Jd (A, Plane_2 + B) := -By_A2 (A, B);
                     end loop;
                  end loop;
                  declare
                     Sigma_D : constant Real_Matrix := Jd * Sigma_U * Transpose (Jd);
                     D1v     : constant Real := Across_1 * Delta_N;
                     D2v     : constant Real := Across_2 * Delta_N;
                     C11, C12, C22 : Real := 0.0;
                  begin
                     for A in 1 .. 3 loop
                        for B in 1 .. 3 loop
                           C11 := C11 + Across_1 (A) * Sigma_D (A, B) * Across_1 (B);
                           C12 := C12 + Across_1 (A) * Sigma_D (A, B) * Across_2 (B);
                           C22 := C22 + Across_2 (A) * Sigma_D (A, B) * Across_2 (B);
                        end loop;
                     end loop;
                     declare
                        Det : constant Real := C11 * C22 - C12 * C12;
                        Chi : constant Real :=
                          (if Det > 0.0 then (C22 * D1v * D1v - 2.0 * C12 * D1v * D2v + C11 * D2v * D2v) / Det
                           else Real'Last);
                     begin
                        if Chi >= Real'Last
                          or else (Chi > 0.0 and then Driver.Distributions.Chi_Square_Deviate (Chi, 2) > Driver.Conventions.Z)
                        then
                           Why := Ada.Strings.Unbounded.To_Unbounded_String
                             ("the two tables are not one plane to the eye: their normals differ by"
                              & Real'Image (Gap_N) & " rad, more than their sigmas allow");
                           return;
                        end if;
                     end;
                  end;
               end;
            end;
            Placement := (Rotation => Rp, Translation => Pose1.Translation - Spread);
            Scale := Kappa;
            Placed := Kappa > 0.0 and then Covariance (Outputs, Outputs) < Real'Last
              and then Covariance (4, 4) < Real'Last and then Covariance (1, 1) < Real'Last;
         end;
      end;
   end Through;

   procedure Place (M : in out Model) is
      W : constant Natural := Index_Of (M, 1);
   begin
      --  The first arm is the world as it is.
      if W > 0 then
         declare
            R : Arm_Fit renames M.Kinematics (W).Result;
         begin
            R.Placed := R.Fitted;
            R.Placement := Identity;
            R.Scale := 1.0;
            R.Scale_Sigma := 0.0;
            R.Placed_Through := 0;
            R.Placement_Covariance.Clear;
            for Entry_Of in 1 .. Outputs * Outputs loop
               R.Placement_Covariance.Append (0.0);
            end loop;
         end;
      end if;
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if K /= W then
            declare
               R2  : Arm_Evidence renames M.Kinematics (K);
               Why : Ada.Strings.Unbounded.Unbounded_String;
            begin
               R2.Result.Placed := False;
               if W = 0 or else not M.Kinematics (W).Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("the first arm is not fitted");
               elsif not Current (M, R2) or else not R2.Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("it is not fitted");
               else
                  declare
                     use type Ada.Strings.Unbounded.Unbounded_String;
                     R1   : Arm_Evidence renames M.Kinematics (W);
                     Best : Real := Real'Last;
                     Ways : Natural := 0;
                  begin
                     --  Every fixed eye placed by its fit to both arms.
                     for E in M.Fixed_Eyes.First_Index .. M.Fixed_Eyes.Last_Index loop
                        declare
                           F      : Fixed_Fit renames M.Fixed_Eyes (E);
                           S1, S2 : Natural := 0;
                        begin
                           if F.Known then
                              for S in F.Arms.First_Index .. F.Arms.Last_Index loop
                                 if F.Arms (S) = Natural (R1.Arm) then
                                    S1 := S - F.Arms.First_Index + 1;
                                 end if;
                                 if F.Arms (S) = Natural (R2.Arm) then
                                    S2 := S - F.Arms.First_Index + 1;
                                 end if;
                              end loop;
                           end if;
                           if S1 > 0 and then S2 > 0 then
                              declare
                                 Placement  : Rigid;
                                 Scale      : Real;
                                 Covariance : Output_Covariance;
                                 Response   : Output_Vector;
                                 Placed     : Boolean;
                                 Not_Placed : Ada.Strings.Unbounded.Unbounded_String;
                              begin
                                 Ways := Ways + 1;
                                 Through
                                   (R1, R2, F, S1, S2, Placement, Scale, Covariance, Response, Placed, Not_Placed);
                                 if not Placed then
                                    Why := Why & "through eye" & E'Image & ", " & Not_Placed & "; ";
                                 elsif Covariance (4, 4) + Covariance (5, 5) + Covariance (6, 6) < Best then
                                    Best := Covariance (4, 4) + Covariance (5, 5) + Covariance (6, 6);
                                    R2.Result.Placed := True;
                                    R2.Result.Placement := Placement;
                                    R2.Result.Scale := Scale;
                                    R2.Result.Scale_Sigma := Scale * Square_Root (Real'Max (0.0, Covariance (Outputs, Outputs)));
                                    R2.Result.Placement_Covariance.Clear;
                                    for P in 1 .. Outputs loop
                                       for Q in 1 .. Outputs loop
                                          R2.Result.Placement_Covariance.Append (Covariance (P, Q));
                                       end loop;
                                    end loop;
                                    R2.Result.Placed_Px := F.Sigma_Px;
                                    R2.Result.Placed_Points := F.Used;
                                    R2.Result.Placed_Through := Natural (E);
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "kinematics: arm" & R2.Arm'Image & " placed in the world"
                                       & " through eye" & E'Image & ", which saw both arms turn; its scale "
                                       & Driver.Log.Image (Scale, 4) & " +- " & Driver.Log.Image (R2.Result.Scale_Sigma, 4)
                                       & ", its centre to "
                                       & Driver.Log.Image (Square_Root (Covariance (4, 4) + Covariance (5, 5)
                                                                        + Covariance (6, 6)), 4)
                                       & " world units and "
                                       & Driver.Log.Image (Square_Root (Covariance (1, 1) + Covariance (2, 2)
                                                                        + Covariance (3, 3)), 4)
                                       & " rad; both focal lengths a hundredth too long move its centre by "
                                       & Driver.Log.Image (Square_Root (Response (4) ** 2 + Response (5) ** 2
                                                                        + Response (6) ** 2), 4)
                                       & " world units, turn it by "
                                       & Driver.Log.Image (Square_Root (Response (1) ** 2 + Response (2) ** 2
                                                                        + Response (3) ** 2), 4)
                                       & " rad and its scale by " & Driver.Log.Image (Response (Outputs), 4));
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                     if Ways = 0 and then Ada.Strings.Unbounded.Length (Why) = 0 then
                        Why := Ada.Strings.Unbounded.To_Unbounded_String
                          ("no fixed eye was fitted to both it and the first arm");
                     end if;
                  end;
               end if;
               if not R2.Result.Placed and then Current (M, R2) and then R2.Result.Fitted then
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R2.Arm'Image & " not placed in the world: "
                                   & Ada.Strings.Unbounded.To_String (Why));
               end if;
            end;
         end if;
      end loop;
   end Place;

   function Fixed_Known (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Fixed_Eyes.Last_Index and then Is_Fixed (M, E) and then M.Fixed_Eyes (E).Known);

   function Fixed_Why (M : Model; E : Eye_Id) return String is
     (if not Is_Fixed (M, E) or else Fixed_Known (M, E) then ""
      elsif E > M.Fixed_Eyes.Last_Index or else not M.Fixed_Eyes (E).Judged then "not measured yet"
      else Ada.Strings.Unbounded.To_String (M.Fixed_Eyes (E).Why));

   --  The fixed eye's lens.
   function Fixed_Lens (M : Model; E : Eye_Id) return Fit.Lens is
     (if E <= M.Fixed_Eyes.Last_Index then Fit_Lens (M.Fixed_Eyes (E).Lens) else (others => <>));

   function Fixed_Pose (M : Model; E : Eye_Id) return Pose_Estimate is
      Terms : constant Natural := Fixed_Terms;
   begin
      if not Fixed_Known (M, E) or else Natural (M.Fixed_Eyes (E).Covariance.Length) /= Terms * Terms then
         return (others => <>);
      end if;
      declare
         use Driver.Numerics.Arrays;
         F : Fixed_Fit renames M.Fixed_Eyes (E);
         function C (P, Q : Positive) return Real is
           (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
         Turn, Place : Mat3;
      begin
         --  The turn is about the camera's own axes: in the world it is the pose's rotation of it.
         for P in 1 .. 3 loop
            for Q in 1 .. 3 loop
               Turn (P, Q) := C (Fit.Lens_Terms + P, Fit.Lens_Terms + Q);
               Place (P, Q) := C (Fit.Lens_Terms + 3 + P, Fit.Lens_Terms + 3 + Q);
            end loop;
         end loop;
         return (Pose                => F.Pose,
                 Position_Covariance => Place,
                 Rotation_Covariance => F.Pose.Rotation * Turn * Transpose (F.Pose.Rotation));
      end;
   end Fixed_Pose;

   function Fixed_Ray_In_Eye (M : Model; E : Eye_Id; U, V : Real) return Vec3 is
     (Unit (Fit.Ray (Fixed_Lens (M, E), U, V)));

   procedure Fixed_Project_In_Eye (M : Model; E : Eye_Id; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      Fit.Project (Fixed_Lens (M, E), P, U, V, In_Front);
   end Fixed_Project_In_Eye;

   function Fixed_Angle_Sigma (M : Model; E : Eye_Id) return Real is
      L : constant Fit.Lens := Fixed_Lens (M, E);
   begin
      return (if Fixed_Known (M, E) and then L.Fx > 0.0 and then L.Fy > 0.0
              then M.Fixed_Eyes (E).Sigma_Px / Real'Min (L.Fx, L.Fy) else Real'Last);
   end Fixed_Angle_Sigma;

   function Fixed_Line_Sigma (M : Model; E : Eye_Id; U, V : Real; In_World : Boolean) return Real is
      use Driver.Numerics.Arrays;
      Terms : constant Natural := Fixed_Terms;
   begin
      if not Fixed_Known (M, E) or else Natural (M.Fixed_Eyes (E).Covariance.Length) /= Terms * Terms then
         return Real'Last;
      end if;
      declare
         F     : Fixed_Fit renames M.Fixed_Eyes (E);
         L     : constant Fit.Lens := Fixed_Lens (M, E);
         Free_Terms : constant Natural := Fit.Lens_Terms + (if In_World then 3 else 0);
         Turned : constant Mat3 := (if In_World then F.Pose.Rotation else Driver.Numerics.Identity3);
         D0     : constant Vec3 := Unit (Fit.Ray (L, U, V));
         --  How the direction (in the world, or in the eye) moves with each term: columns, lens terms then turn.
         J      : array (1 .. 3, 1 .. Free_Terms) of Real := [others => [others => 0.0]];
         Spin   : constant Mat3 := -(Turned * Driver.Numerics.Skew (D0));
         Var    : Real := 0.0;
         function C (P, Q : Positive) return Real is
           (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
      begin
         for Term in 1 .. Fit.Lens_Terms loop
            declare
               S : constant Real := Square_Root (Real'Max (0.0, C (Term, Term)));
            begin
               if S > 0.0 then
                  declare
                     Change : constant Vec3 :=
                       (Turned * (Unit (Fit.Ray (Moved (L, Term, S), U, V))
                                  - Unit (Fit.Ray (Moved (L, Term, -S), U, V)))) / (2.0 * S);
                  begin
                     for A in 1 .. 3 loop
                        J (A, Term) := Change (A);
                     end loop;
                  end;
               end if;
            end;
         end loop;
         if In_World then
            for A in 1 .. 3 loop
               for B in 1 .. 3 loop
                  J (A, Fit.Lens_Terms + B) := Spin (A, B);
               end loop;
            end loop;
         end if;
         --  Half the trace of the direction's covariance: the variance per axis across the line.
         for A in 1 .. 3 loop
            for P in 1 .. Free_Terms loop
               for Q in 1 .. Free_Terms loop
                  Var := Var + J (A, P) * C (P, Q) * J (A, Q);
               end loop;
            end loop;
         end loop;
         return Square_Root (Fixed_Angle_Sigma (M, E) ** 2 + Var / 2.0);
      end;
   end Fixed_Line_Sigma;

   procedure In_World (M : Model; A : Arm_Id; Placement : out Rigid; Scale : out Real; Known : out Boolean) is
      K : constant Natural := Index_Of (M, A);
   begin
      Placement := Identity;
      Scale := 1.0;
      Known := K > 0 and then M.Kinematics (K).Result.Fitted and then M.Kinematics (K).Result.Placed;
      if Known then
         Placement := M.Kinematics (K).Result.Placement;
         Scale := M.Kinematics (K).Result.Scale;
      end if;
   end In_World;

   function Scale_In_World (M : Model; A : Arm_Id) return Estimate is
      K : constant Natural := Index_Of (M, A);
   begin
      if K = 0 or else not M.Kinematics (K).Result.Fitted or else not M.Kinematics (K).Result.Placed then
         return Driver.Uncertain.Unknown;
      end if;
      return (Value              => M.Kinematics (K).Result.Scale,
              Sigma              => M.Kinematics (K).Result.Scale_Sigma,
              Degrees_Of_Freedom => 0);
   end Scale_In_World;

   procedure Placement_Covariance (M : Model; A : Arm_Id; Turn, Centre : out Mat3) is
      K : constant Natural := Index_Of (M, A);
   begin
      Turn := [others => [others => 0.0]];
      Centre := [others => [others => 0.0]];
      for I in 1 .. 3 loop
         Turn (I, I) := Real'Last;
         Centre (I, I) := Real'Last;
      end loop;
      if K = 0 or else not M.Kinematics (K).Result.Fitted or else not M.Kinematics (K).Result.Placed
        or else Natural (M.Kinematics (K).Result.Placement_Covariance.Length) < 36
      then
         return;
      end if;
      declare
         C : Real_Vectors.Vector renames M.Kinematics (K).Result.Placement_Covariance;
      begin
         --  6 x 6, row by row: the turn first, then the centre.
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Turn (I, J) := C (C.First_Index + 6 * (I - 1) + (J - 1));
               Centre (I, J) := C (C.First_Index + 6 * (I + 2) + (J + 2));
            end loop;
         end loop;
      end;
   end Placement_Covariance;

   function World_Table (M : Model) return Driver.Geometry.Plane_Estimate is
   begin
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if M.Kinematics (K).Result.Fitted and then M.Kinematics (K).Result.Placed
           and then M.Kinematics (K).Result.Placed_Through = 0
         then
            return M.Kinematics (K).Result.Table;
         end if;
      end loop;
      return (others => <>);
   end World_Table;

   procedure World_Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3) is
      use Driver.Numerics.Arrays;
      K : constant Natural := Index_Of (M, A);
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
   begin
      Turn := Unknown;
      Place := Unknown;
      if K = 0 or else not M.Kinematics (K).Result.Placed
        or else Natural (M.Kinematics (K).Result.Placement_Covariance.Length) /= Outputs * Outputs
      then
         return;
      end if;
      declare
         R      : Arm_Fit renames M.Kinematics (K).Result;
         Own_Turn, Own_Place : Mat3;
         Rp     : constant Mat3 := R.Placement.Rotation;
         T      : constant Vec3 := Eye_In_Reference (M, A, Readings).Translation;
         --  The eye's offset from the placement's centre, in the world.
         Lever  : constant Vec3 := R.Scale * (Rp * T);
         Spin   : constant Mat3 := -Driver.Numerics.Skew (Lever);
         --  How the eye's place in the world moves with the placement's turn, its centre and the logarithm of its
         --  scale (the scale grows every length in the arm's frame, the lever to the eye with it).
         By     : Real_Matrix (1 .. 3, 1 .. Outputs) := [others => [others => 0.0]];
         Whole  : Real_Matrix (1 .. Outputs, 1 .. Outputs);
         Turned : Mat3;
         Moved  : Real_Matrix (1 .. 3, 1 .. 3);
      begin
         Pose_Covariance (M, A, Readings, Own_Turn, Own_Place);
         if Own_Turn (1, 1) = Real'Last or else Own_Place (1, 1) = Real'Last then
            return;
         end if;
         for P in 1 .. Outputs loop
            for Q in 1 .. Outputs loop
               Whole (P, Q) := R.Placement_Covariance ((P - 1) * Outputs + Q - 1);
            end loop;
         end loop;
         for P in 1 .. 3 loop
            for Q in 1 .. 3 loop
               By (P, Q) := Spin (P, Q);
               By (P, 3 + Q) := (if P = Q then 1.0 else 0.0);
            end loop;
            By (P, Outputs) := Lever (P);
         end loop;
         for P in 1 .. 3 loop
            for Q in 1 .. 3 loop
               Turned (P, Q) := Whole (P, Q);
            end loop;
         end loop;
         Moved := By * Whole * Transpose (By);
         --  The eye in the world: turn Rp R_a, place Scale Rp t_a + centre. Its turn takes the arm's own and the
         --  placement's; its place the arm's own scaled, and the placement's (its turn on the lever to the eye, its
         --  centre, its scale along the lever), their correlations kept.
         Turn := Rp * Own_Turn * Transpose (Rp) + Turned;
         Place := R.Scale ** 2 * (Rp * Own_Place * Transpose (Rp)) + Moved;
      end;
   end World_Pose_Covariance;

   function Sees_World (M : Model; A : Arm_Id; U, V : Real) return Boolean is
      K : constant Natural := Index_Of (M, A);
   begin
      if K = 0 or else M.Kinematics (K).Eye = 0 or else M.Kinematics (K).Eye > M.Eyes.Last_Index then
         return True;
      end if;
      declare
         R    : Arm_Evidence renames M.Kinematics (K);
         S    : Eye_Stream renames M.Eyes (R.Eye);
         G    : Cell_Grid renames S.Grid;
         Kept : constant Natural := Natural (S.Kept_Groups.Length);
      begin
         if Kept = 0 or else G.Width = 0 or else G.Height = 0 then
            return True;
         end if;
         declare
            X    : constant Natural := Natural'Min (G.Width - 1, Natural (Real'Floor (Real'Max (0.0, U))));
            Y    : constant Natural := Natural'Min (G.Height - 1, Natural (Real'Floor (Real'Max (0.0, V))));
            Cell : constant Natural :=
              ((Y + 1) * G.Rows - 1) / G.Height * G.Columns + ((X + 1) * G.Columns - 1) / G.Width;
         begin
            for J in 0 .. Kept - 1 loop
               if S.Kept_Groups (J) = Natural (R.Group) and then Cell * Kept + J < Natural (S.Shifts.Length)
                 and then S.Shifts (Cell * Kept + J) > 0.0
               then
                  return True;
               end if;
            end loop;
            return False;
         end;
      end;
   end Sees_World;

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid is
   begin
      for R of M.Kinematics loop
         if R.Arm = A and then Current (M, R) and then R.Result.Fitted
           and then Natural (R.Result.Joints.Length) = Readings'Length
           and then Natural (R.Result.Chain.Length) = Readings'Length
           and then Natural (R.Result.Reference.Length) = Readings'Length
         then
            declare
               Joints : Fit.Joint_Array (1 .. Readings'Length);
               Change : Real_Array (1 .. Readings'Length);
            begin
               for J in Joints'Range loop
                  declare
                     F : constant Joint_Fit := R.Result.Joints (J);
                  begin
                     Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                     Change (J) := Readings (Readings'First + J - 1) - R.Result.Reference (J - 1);
                  end;
               end loop;
               return Fit.Eye_At (Joints, Chain_Of (R.Result), Change);
            end;
         end if;
      end loop;
      return Identity;
   end Eye_In_Reference;

   function Slack_Parts (M : Model; A : Arm_Id; Beat : Natural) return Real_Array is
   begin
      for R of M.Kinematics loop
         if R.Arm = A and then Current (M, R) and then R.Result.Fitted and then R.Result.Reference_Beat > 0
           and then Beat > R.Result.Reference_Beat and then Beat <= M.Beats
           and then Natural (R.Result.Joints.Length) = Group_Size (M, R.Group)
           and then Natural (R.Result.Reference.Length) = Group_Size (M, R.Group)
         then
            declare
               N     : constant Natural := Natural (R.Result.Joints.Length);
               From  : constant Natural := R.Result.Reference_Beat;
               Parts : Real_Array (1 .. N) := [others => 0.0];
            begin
               for C in 1 .. N loop
                  declare
                     H    : constant Real := Real'Max (0.0, R.Result.Joints (C).Slack);
                     V0   : constant Real := Real'Max (-H, Real'Min (H, R.Result.Joints (C).Slack_Place));
                     V    : Real := V0;
                     Last : Real := R.Result.Reference (C - 1);
                  begin
                     if H > 0.0 then
                        for B in From + 1 .. Beat loop
                           if Channels.Has_Reading (M, R.Group, B) then
                              declare
                                 Now : constant Real := Channels.Reading (M, R.Group, B, C);
                              begin
                                 V := Real'Max (-H, Real'Min (H, V + (Now - Last)));
                                 Last := Now;
                              end;
                           end if;
                        end loop;
                        Parts (C) := V - V0;
                     end if;
                  end;
               end loop;
               return Parts;
            end;
         end if;
      end loop;
      return [1 .. 0 => 0.0];
   end Slack_Parts;

   procedure Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3) is
      R : constant Arm_Fit := (if Index_Of (M, A) > 0 then M.Kinematics (Index_Of (M, A)).Result else (others => <>));
   begin
      Turn := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      Place := Turn;
      if R.Fitted and then Natural (R.Joints.Length) = Readings'Length
        and then Natural (R.Chain.Length) = Readings'Length and then Natural (R.Reference.Length) = Readings'Length
      then
         declare
            Joints : Fit.Joint_Array (1 .. Readings'Length);
            Change : Real_Array (1 .. Readings'Length);
            Cov    : Fit.Real_Lists.Vector;
         begin
            for J in Joints'Range loop
               declare
                  F : constant Joint_Fit := R.Joints (J);
               begin
                  Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                  Change (J) := Readings (Readings'First + J - 1) - R.Reference (J - 1);
               end;
            end loop;
            for X of R.Covariance loop
               Cov.Append (X);
            end loop;
            Fit.Pose_Covariance (Joints, Chain_Of (R), Change, Cov, Turn, Place);
         end;
      end if;
   end Pose_Covariance;

   function Step_Covariance (M : Model; A : Arm_Id; From, To : Real_Array) return Mat3 is
      R : constant Arm_Fit := (if Index_Of (M, A) > 0 then M.Kinematics (Index_Of (M, A)).Result else (others => <>));
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
   begin
      if not (R.Fitted and then Natural (R.Joints.Length) = From'Length and then To'Length = From'Length
              and then Natural (R.Chain.Length) = From'Length and then Natural (R.Reference.Length) = From'Length)
      then
         return Unknown;
      end if;
      declare
         Joints   : Fit.Joint_Array (1 .. From'Length);
         From_Chg : Real_Array (1 .. From'Length);
         To_Chg   : Real_Array (1 .. From'Length);
         Cov      : Fit.Real_Lists.Vector;
      begin
         for J in Joints'Range loop
            declare
               F : constant Joint_Fit := R.Joints (J);
            begin
               Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
               From_Chg (J) := From (From'First + J - 1) - R.Reference (J - 1);
               To_Chg (J) := To (To'First + J - 1) - R.Reference (J - 1);
            end;
         end loop;
         for X of R.Covariance loop
            Cov.Append (X);
         end loop;
         return Fit.Step_Covariance (Joints, Chain_Of (R), From_Chg, To_Chg, Cov);
      end;
   end Step_Covariance;

   function Place_Covariance_In_Unit (M : Model; A : Arm_Id; Readings : Real_Array) return Mat3 is
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      K       : constant Natural := Index_Of (M, A);
   begin
      if K = 0 then
         return Unknown;
      end if;
      declare
         E      : Arm_Evidence renames M.Kinematics (K);
         R      : Arm_Fit renames E.Result;
         N      : constant Natural := Readings'Length;
         Frames : constant Natural := (if N = 0 then 0 else Natural (R.Unit_Changes.Length) / N);
      begin
         if not R.Fitted or else Natural (R.Joints.Length) /= N or else Natural (R.Chain.Length) /= N
           or else Natural (R.Reference.Length) /= N or else Frames = 0
         then
            return Unknown;
         end if;
         declare
            Joints  : Fit.Joint_Array (1 .. N);
            Change  : Real_Array (1 .. N);
            Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
            Cov     : Fit.Real_Lists.Vector;
         begin
            for J in 1 .. N loop
               declare
                  F : constant Joint_Fit := R.Joints (J);
               begin
                  Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                  Change (J) := Readings (Readings'First + J - 1) - R.Reference (J - 1);
               end;
            end loop;
            for F in 1 .. Frames loop
               for C in 1 .. N loop
                  Changes (F, C) := R.Unit_Changes ((F - 1) * N + C - 1);
               end loop;
            end loop;
            for X of R.Covariance loop
               Cov.Append (X);
            end loop;
            return Result : constant Mat3 := Fit.Place_Covariance_In_Unit (Joints, Chain_Of (R), Change, Changes.all, Cov) do
               Free (Changes);
            end return;
         end;
      end;
   end Place_Covariance_In_Unit;

   function Framed (M : Model; A : Arm_Id; From, To : Natural) return Boolean is
      K : constant Natural := Index_Of (M, A);
   begin
      return K > 0 and then (for some F of M.Kinematics (K).Frames => F.Beat in From .. To);
   end Framed;

   function Followed (M : Model; A : Arm_Id; From, To : Natural) return Boolean is
      K : constant Natural := Index_Of (M, A);
   begin
      if K = 0 then
         return False;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (K);
      begin
         for F in reverse 2 .. Natural (R.Frames.Length) loop
            if R.Frames (F).Beat in From .. To then
               for S of R.Matches loop
                  if S.Frame = F then
                     return S.Followed;
                  end if;
               end loop;
               return False;
            end if;
         end loop;
         return False;
      end;
   end Followed;

   function Fitted (M : Model; A : Arm_Id) return Boolean is
     (Index_Of (M, A) > 0 and then M.Kinematics (Index_Of (M, A)).Result.Fitted);

   function Contested_Pairs (M : Model; A : Arm_Id) return Joint_Pair_Lists.Vector is
      None : Joint_Pair_Lists.Vector;
   begin
      if not Fitted (M, A) or else M.Kinematics (Index_Of (M, A)).Result.Order_Told then
         return None;
      end if;
      return M.Kinematics (Index_Of (M, A)).Result.Untold;
   end Contested_Pairs;

   function Contested_Turns (M : Model; A : Arm_Id) return Real_Vectors.Vector is
      None : Real_Vectors.Vector;
   begin
      if not Fitted (M, A) or else M.Kinematics (Index_Of (M, A)).Result.Order_Told then
         return None;
      end if;
      return M.Kinematics (Index_Of (M, A)).Result.Untold_Turns;
   end Contested_Turns;

   function Untold_Count (M : Model; A : Arm_Id) return Natural is
   begin
      if not Fitted (M, A) or else M.Kinematics (Index_Of (M, A)).Result.Order_Told
        or else M.Kinematics (Index_Of (M, A)).Result.Chain.Is_Empty
      then
         return 0;
      end if;
      return Natural (M.Kinematics (Index_Of (M, A)).Result.Untold_Orders.Length)
        / Natural (M.Kinematics (Index_Of (M, A)).Result.Chain.Length);
   end Untold_Count;

   function Order_Told (M : Model; A : Arm_Id) return Boolean is
     (Fitted (M, A) and then M.Kinematics (Index_Of (M, A)).Result.Order_Told);

   function Table (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate is
     (if Fitted (M, A) then M.Kinematics (Index_Of (M, A)).Result.Table else (others => <>));

   function Standing_Near (M : Model; A : Arm_Id; Point : Vec3; Within : Real) return Natural is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      K     : constant Natural := Index_Of (M, A);
      Count : Natural := 0;
   begin
      if not Fitted (M, A) or else not Driver.Geometry.Known (M.Kinematics (K).Result.Table) then
         return 0;
      end if;
      declare
         F : Arm_Fit renames M.Kinematics (K).Result;
         P : Driver.Geometry.Plane_Estimate renames F.Table;
      begin
         for T in 0 .. Natural (F.Track_Known.Length) - 1 loop
            if F.Track_Known (T) and then 3 * T + 2 < Natural (F.Tracks.Length)
              and then not (T < Natural (F.Table_On.Length) and then F.Table_On (T))
              and then T < Natural (F.Track_Sigmas.Length) and then F.Track_Sigmas (T) < Real'Last
            then
               declare
                  X : constant Vec3 := [F.Tracks (3 * T), F.Tracks (3 * T + 1), F.Tracks (3 * T + 2)];
                  H : constant Real := Driver.Geometry.Height (P, X);
                  --  The track's depth is uncertain by its logarithm's sigma: along its line of sight (the
                  --  reference eye is the origin), its length times that, of which the part along the normal
                  --  is the height's; with the plane's own there.
                  Length  : constant Real := abs X;
                  Along   : constant Real := Length * F.Track_Sigmas (T);
                  Facing  : constant Real := (if Length > 0.0 then Real'(Unit (X) * P.Normal) else 0.0);
                  Sigma   : constant Real :=
                    (if Length > 0.0
                     then Sqrt ((Along * abs Facing) ** 2 + Driver.Geometry.Height_Sigma (P, X) ** 2)
                     else Real'Last);
                  Off     : constant Vec3 := X - Point;
                  Across  : constant Real := Real'(Off * P.Normal);
                  Lateral : constant Real := abs (Off - Across * P.Normal);
               begin
                  if Sigma < Real'Last and then H > 0.0 and then Driver.Uncertain.Significant (H, Sigma)
                    and then Lateral <= Within
                  then
                     Count := Count + 1;
                  end if;
               end;
            end if;
         end loop;
      end;
      return Count;
   end Standing_Near;

   procedure Clear_Spot (M : Model; A : Arm_Id; Near : Vec3; Within : Real; Spot : out Vec3; Found : out Boolean) is
      use Driver.Numerics.Arrays;
      K       : constant Natural := Index_Of (M, A);
      Nearest : Real := Real'Last;
   begin
      Spot := [0.0, 0.0, 0.0];
      Found := False;
      if not Fitted (M, A) or else not Driver.Geometry.Known (M.Kinematics (K).Result.Table) then
         return;
      end if;
      declare
         F : Arm_Fit renames M.Kinematics (K).Result;
         P : Driver.Geometry.Plane_Estimate renames F.Table;
      begin
         for T in 0 .. Natural (F.Track_Known.Length) - 1 loop
            if F.Track_Known (T) and then 3 * T + 2 < Natural (F.Tracks.Length)
              and then T < Natural (F.Table_On.Length) and then F.Table_On (T)
            then
               declare
                  X       : constant Vec3 := [F.Tracks (3 * T), F.Tracks (3 * T + 1), F.Tracks (3 * T + 2)];
                  Off     : constant Vec3 := X - Near;
                  Lateral : constant Real := abs (Off - Real'(Off * P.Normal) * P.Normal);
               begin
                  if Lateral < Nearest and then Standing_Near (M, A, X, Within) = 0 then
                     Nearest := Lateral;
                     Spot := X;
                     Found := True;
                  end if;
               end;
            end if;
         end loop;
      end;
   end Clear_Spot;

   procedure Solve_Pose
     (M             : Model;
      A             : Arm_Id;
      Start         : Real_Array;
      Goal          : Rigid;
      Position_Only : Boolean;
      Q             : out Real_Array;
      Position_Off  : out Real;
      Turn_Off      : out Real;
      Free_About    : Vec3 := [0.0, 0.0, 0.0])
   is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      N    : constant Natural := Start'Length;
      --  What a residual holds: the eye's place alone, its place and its whole turn, or its place and where
      --  Free_About points (their cross product: the sine of the angle between them, along the axis it turns about).
      type Held is (Place, Whole, Pointing);
      Rows : constant Positive := (if Position_Only then 3 else 6);

      function Residual (V : Real_Array; H : Held) return Real_Vector is
         T : constant Rigid := Eye_In_Reference (M, A, V);
         R : Real_Vector (1 .. Rows);
      begin
         R (1 .. 3) := T.Translation - Goal.Translation;
         case H is
            when Place =>
               null;
            when Whole =>
               R (4 .. 6) := Driver.Numerics.Log (Transpose (T.Rotation) * Goal.Rotation);
            when Pointing =>
               declare
                  Is_At : constant Vec3 := T.Rotation * Free_About;
                  Want  : constant Vec3 := Goal.Rotation * Free_About;
               begin
                  R (4 .. 6) := (1.0 / (Real'(abs Is_At) * Real'(abs Want))) * Driver.Numerics.Cross (Is_At, Want);
               end;
         end case;
         return R;
      end Residual;

      function Cost (R : Real_Vector) return Real is (R * R);

      --  Damped least squares from X on what H holds, until a step lowers what is left by less than the unchanged
      --  fraction of it.
      procedure Minimize (X : in out Real_Array; H : Held; R0 : out Real_Vector) is
         Lambda : Real := Real'Model_Epsilon;
      begin
         R0 := Residual (X, H);
         loop
            declare
               J : Real_Matrix (1 .. Rows, 1 .. N);
               Improved, Lowered : Boolean := False;
            begin
               for K in 1 .. N loop
                  declare
                     Xp : Real_Array := X;
                     Hk : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (K));
                     Rp : Real_Vector (1 .. Rows);
                  begin
                     Xp (K) := Xp (K) + Hk;
                     Rp := Residual (Xp, H);
                     for I in 1 .. Rows loop
                        J (I, K) := (Rp (I) - R0 (I)) / Hk;
                     end loop;
                  end;
               end loop;
               declare
                  JtJ : constant Real_Matrix := Transpose (J) * J;
                  G   : constant Real_Vector := -(Transpose (J) * R0);
               begin
                  loop
                     declare
                        D  : Real_Matrix := JtJ;
                        Lf : Real_Matrix (1 .. N, 1 .. N);
                        Pd : Boolean;
                        Moves : Boolean := False;
                     begin
                        for P in 1 .. N loop
                           D (P, P) := JtJ (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                        end loop;
                        Driver.Numerics.Dense.Cholesky (D, Lf, Pd);
                        if Pd then
                           declare
                              Step : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, G);
                              Xn   : Real_Array (1 .. N);
                           begin
                              for P in 1 .. N loop
                                 Xn (P) := X (P) + Step (P);
                              end loop;
                              Moves := (for some P in 1 .. N => Xn (P) /= X (P));
                              if Moves then
                                 declare
                                    Rn : constant Real_Vector := Residual (Xn, H);
                                 begin
                                    if Cost (Rn) < Cost (R0) then
                                       Improved := Cost (R0) - Cost (Rn) > Driver.Conventions.Unchanged_Fraction * Cost (R0);
                                       X := Xn;
                                       R0 := Rn;
                                       Lambda := Lambda / 2.0;
                                       Lowered := True;
                                    end if;
                                 end;
                              end if;
                           end;
                        else
                           Moves := True;
                        end if;
                        --  No further than a float goes (as Fixed.Refine): past it the damping is infinite, the damped
                        --  matrix no number, never positive definite, and nothing would end the loop.
                        exit when Lowered or else not Moves or else Lambda >= Real'Last / 2.0;
                        Lambda := 2.0 * Lambda;
                     end;
                  end loop;
               end;
               exit when not Improved;
            end;
         end loop;
      end Minimize;

      X  : Real_Array (1 .. N) := Start;
      R0 : Real_Vector (1 .. Rows);
   begin
      if Position_Only then
         Minimize (X, Place, R0);
      elsif abs Free_About = 0.0 then
         Minimize (X, Whole, R0);
      else
         --  Where Free_About points first, from Start: the nearest pose that points it. Then the whole goal from there,
         --  taken where the arm reaches it within what its fit tells (Angle_Sigma), the place and the turn both, and
         --  the nearest pose that points it where it does not: an arm with a joint to spare reaches the pointing along
         --  a line of poses, and the goal's turn about the direction held as a weighed residual drew the solve along
         --  that line from where the arm stood (h25, servo seed 3, 10-17: aims a few thousandths from where the tool
         --  stood asked 4500 to 4800 units of the joints, and the arm was blocked at once).
         Minimize (X, Pointing, R0);
         declare
            Sigma : constant Real := Angle_Sigma (M, A);
            W     : Real_Array (1 .. N) := X;
            RW    : Real_Vector (1 .. Rows);
         begin
            Minimize (W, Whole, RW);
            if Sigma < Real'Last
              and then not Driver.Uncertain.Significant (Sqrt (RW (1) ** 2 + RW (2) ** 2 + RW (3) ** 2), Sigma)
              and then not Driver.Uncertain.Significant (Sqrt (RW (4) ** 2 + RW (5) ** 2 + RW (6) ** 2), Sigma)
            then
               X := W;
               R0 := Residual (X, Pointing);
            end if;
         end;
      end if;
      Q := Nearest_Readings (M, A, Start, X);
      Position_Off := Sqrt (R0 (1) ** 2 + R0 (2) ** 2 + R0 (3) ** 2);
      Turn_Off := (if Position_Only then 0.0 else Sqrt (R0 (4) ** 2 + R0 (5) ** 2 + R0 (6) ** 2));
   end Solve_Pose;

   function Conditioning (M : Model; A : Arm_Id; Q : Real_Array) return Real is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      N : constant Natural := Q'Length;
   begin
      if N = 0 or else Index_Of (M, A) = 0 or else not M.Kinematics (Index_Of (M, A)).Result.Fitted then
         return 0.0;
      end if;
      declare
         T0 : constant Rigid := Eye_In_Reference (M, A, Q);
         J  : Real_Matrix (1 .. 6, 1 .. N);
      begin
         for K in 1 .. N loop
            declare
               Xp : Real_Array := Q;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Q (Q'First + K - 1));
               Tp : Rigid;
               D  : Vec3;
               W  : Vec3;
            begin
               Xp (Xp'First + K - 1) := Xp (Xp'First + K - 1) + H;
               Tp := Eye_In_Reference (M, A, Xp);
               D := Tp.Translation - T0.Translation;
               W := Driver.Numerics.Log (Transpose (T0.Rotation) * Tp.Rotation);
               for I in 1 .. 3 loop
                  J (I, K) := D (I) / H;
                  J (I + 3, K) := W (I) / H;
               end loop;
            end;
         end loop;
         declare
            Values : constant Real_Vector := Eigenvalues (Transpose (J) * J);
            Least  : Real := Real'Last;
            Most   : Real := 0.0;
         begin
            for V of Values loop
               Least := Real'Min (Least, V);
               Most := Real'Max (Most, V);
            end loop;
            return (if Most > 0.0 then Sqrt (Real'Max (0.0, Least) / Most) else 0.0);
         end;
      end;
   end Conditioning;

   function Easiest_Way (M : Model; A : Arm_Id; Q : Real_Array) return Vec3 is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      N : constant Natural := Q'Length;
      I : constant Natural := Index_Of (M, A);
   begin
      if N = 0 or else I = 0 or else not M.Kinematics (I).Result.Fitted
        or else Natural (M.Kinematics (I).Result.Joints.Length) /= N
      then
         return Zero3;
      end if;
      declare
         T0     : constant Rigid := Eye_In_Reference (M, A, Q);
         Spread : Mat3 := [others => [others => 0.0]];   --  J J'
         Values : Vec3;
         Ways   : Mat3;
      begin
         for K in 1 .. N loop
            declare
               Xp : Real_Array := Q;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Q (Q'First + K - 1));
               C  : constant Real := M.Kinematics (I).Result.Joints (M.Kinematics (I).Result.Joints.First_Index + K - 1).C;
               D  : Vec3;
            begin
               Xp (Xp'First + K - 1) := Xp (Xp'First + K - 1) + H;
               --  The place's change for a turn of the joint, not for a change of its reading.
               D := (1.0 / (H * (if C = 0.0 then 1.0 else C))) * (Eye_In_Reference (M, A, Xp).Translation - T0.Translation);
               for R in 1 .. 3 loop
                  for S in 1 .. 3 loop
                     Spread (R, S) := Spread (R, S) + D (R) * D (S);
                  end loop;
               end loop;
            end;
         end loop;
         Driver.Numerics.Symmetric_Eigensystem (Spread, Values, Ways);
         return [Ways (1, 1), Ways (2, 1), Ways (3, 1)];
      end;
   end Easiest_Way;

   function Nearest_Readings (M : Model; A : Arm_Id; Near, Readings : Real_Array) return Real_Array is
      Result : Real_Array (Readings'Range) := Readings;
      I      : constant Natural := Index_Of (M, A);
   begin
      if I > 0 and then Natural (M.Kinematics (I).Result.Joints.Length) = Readings'Length then
         for J in 1 .. Readings'Length loop
            declare
               F : constant Joint_Fit := M.Kinematics (I).Result.Joints (J);
            begin
               if not F.Slide and then F.C /= 0.0 then
                  declare
                     Period : constant Real := 2.0 * Ada.Numerics.Pi / abs F.C;
                     Here   : constant Real := Readings (Readings'First + J - 1);
                     Turns  : constant Real := Real'Rounding ((Here - Near (Near'First + J - 1)) / Period);
                  begin
                     Result (Result'First + J - 1) := Here - Turns * Period;
                  end;
               end if;
            end;
         end loop;
      end if;
      return Result;
   end Nearest_Readings;

   function Result_Of (M : Model; A : Arm_Id) return Arm_Fit is
     (if Index_Of (M, A) > 0 then M.Kinematics (Index_Of (M, A)).Result else (others => <>));

   function Lens_Of (M : Model; A : Arm_Id) return Fit.Lens is
      L : constant Lens_Fit := Result_Of (M, A).Lens;
   begin
      return (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2);
   end Lens_Of;

   --  A sighting's error as an angle: the typical sighting's error over the focal length. That error is the larger of
   --  the noise the fit leaves beside what the sightings state (Sigma_Px, the floor) and what the median residual says of
   --  the whole, a standard deviation a direction (the median of a two-dimensional Gaussian's radius is its sigma
   --  times the root of 2 ln 2). The floor alone is zero whenever what the matcher states accounts for the residuals,
   --  and every plan judged a miss of 1e-16 significant against it (f15w on 10-15: the fitted hook arm's every press
   --  unplanned, no tip measured, no part of the body a role, the brain given only speech).
   function Angle_Sigma (M : Model; A : Arm_Id) return Real is
      R : constant Arm_Fit := Result_Of (M, A);
      Typical : constant Real :=
        Real'Max (R.Sigma_Px, R.Median_Px / Ada.Numerics.Long_Elementary_Functions.Sqrt
                                              (2.0 * Ada.Numerics.Long_Elementary_Functions.Log (2.0)));
   begin
      return (if R.Fitted and then R.Lens.Fx > 0.0 and then R.Lens.Fy > 0.0
              then Typical / Real'Min (R.Lens.Fx, R.Lens.Fy) else Real'Last);
   end Angle_Sigma;

   function Ray_In_Eye (M : Model; A : Arm_Id; U, V : Real) return Vec3 is
      H : constant Vec3 := Fit.Ray (Lens_Of (M, A), U, V);
   begin
      return Unit (H);
   end Ray_In_Eye;

   procedure Project_In_Eye (M : Model; A : Arm_Id; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      Fit.Project (Lens_Of (M, A), P, U, V, In_Front);
   end Project_In_Eye;

end Driver.Robot.Kinematics;
