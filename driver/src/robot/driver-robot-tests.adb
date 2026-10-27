with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Parallel_For;
with Driver.Observations;
with Driver.Stats;
with Driver.Beats;
with Driver.Alignment;
with Driver.Log;
with Driver.Numerics.Dense;
with Ada.Strings.Fixed;
with Driver.Robot.Body_File;
with Driver.Robot.Kinematics.Base.Tests;
with Driver.Robot.Kinematics.Errors.Tests;
with Driver.Robot.Surfaces.Tests;
with Driver.Recording;
with Ada.Text_IO;
with GNAT.OS_Lib;
with Driver.Robot.Boot;
with Driver.Robot.Kinematics;
with Driver.Robot.Kinematics.Base;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Flow_Eye;
with Ada.Strings.Unbounded;
with Driver.Robot.Channels;
with Driver.Robot.Hand;
with Driver.Robot.Flow;
with Driver.Robot.Graph;
with Driver.Robot.Regression;
with Driver.Robot.Steps;
with Driver.Robot.Stillness;
with Driver.Tests;

package body Driver.Robot.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   --  The joints of a synthetic arm along it: the tests build their arms in the order of their readings.
   function Along (J : Driver.Robot.Kinematics.Fit.Joint_Array) return Driver.Robot.Kinematics.Fit.Chain_Order is
     ([for K in 1 .. J'Length => K]);

   function Chain_Text (C : Driver.Robot.Kinematics.Fit.Chain_Order) return String is
     (if C'Length = 0 then "" else C (C'First)'Image & Chain_Text (C (C'First + 1 .. C'Last)));

   --  A repeatable uniform generator for test noise (Park and Miller).
   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   function Gaussian (G : in out Generator) return Real is
      U1 : constant Real := Real'Max (Uniform (G), 1.0e-12);
      U2 : constant Real := Uniform (G);
   begin
      return Sqrt (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  A smooth texture with gradients in every direction.
   function Texture (X, Y : Real; Seed : Real := 0.0) return Real is
     (128.0 + 40.0 * Sin (0.7 * X + 0.3 * Y + Seed) + 30.0 * Sin (-0.4 * X + 0.9 * Y + 1.0 + 2.0 * Seed)
      + 20.0 * Sin (1.3 * X - 0.8 * Y + 2.0 + 3.0 * Seed));

   function Smooth (X, Y : Real) return Real is
     (128.0 + 50.0 * Sin (0.35 * X + 0.15 * Y) + 40.0 * Sin (-0.2 * X + 0.45 * Y + 1.0)
      + 25.0 * Sin (0.5 * X - 0.3 * Y + 2.0));

   --  ── Flow ──

   procedure Flow_Recovers_Shifts is
      W : constant := 64;
      H : constant := 48;
      G : constant Cell_Grid := Flow.Grid_Of (W, H);
      A, B : Real_Array (1 .. W * H);
      Du, Dv, Cond : Real_Array (1 .. Cells (G));
      Resolved : Flow.Flag_Array (1 .. Cells (G));
      --  Noiseless frames: a pixel varies by its 8-bit rounding alone, the
      --  floor Driver.Pixels applies.
      Quantization : constant Real_Array (1 .. Cells (G)) := [others => 1.0 / 12.0];

      --  B shows A moved by (Su, Sv): B at x shows A at x - (Su, Sv).
      procedure Shift (Su, Sv : Real) is
      begin
         for Y in 0 .. H - 1 loop
            for X in 0 .. W - 1 loop
               A (Y * W + X + 1) := Smooth (Real (X), Real (Y));
               B (Y * W + X + 1) := Smooth (Real (X) - Su, Real (Y) - Sv);
            end loop;
         end loop;
         Flow.Displacements (G, A, B, Quantization, Du, Dv, Cond, Resolved);
      end Shift;
   begin
      Check (G.Columns = 8 and then G.Rows = 6, "grid of a 64 x 48 image is 8 x 6");
      --  Bilinear interpolation is exact at whole pixels, so a whole-pixel
      --  shift comes back exact; a single linearized step would miss this
      --  one by a large fraction.
      Shift (1.0, -1.0);
      for C in Du'Range loop
         --  Cells at the image border miss pixels that moved out of view.
         if C > G.Columns and then C <= Cells (G) - G.Columns and then (C - 1) mod G.Columns in 1 .. G.Columns - 2
         then
            --  Iteration stops once a step changes it by under 1 % of itself.
            Check_Close (Du (C), 1.0, 0.015, "horizontal whole-pixel shift of cell" & C'Image);
            Check_Close (Dv (C), -1.0, 0.015, "vertical whole-pixel shift of cell" & C'Image);
            Check (Resolved (C), "a one-pixel shift is resolved in cell" & C'Image);
         end if;
         Check (Cond (C) > 0.0, "a textured cell is conditioned");
      end loop;
      --  Between whole pixels the interpolation biases the estimate by up to
      --  about k f (1 - f) / 2 pixels (texture frequency k, fraction f); the
      --  lock-in needs the sign and the growth with the motion, not metric
      --  accuracy.
      Shift (0.3, -0.2);
      for C in Du'Range loop
         Check (Du (C) > 0.0 and then Dv (C) < 0.0, "a sub-pixel shift has the right sign in cell" & C'Image);
         Check_Close (Du (C), 0.3, 0.1, "horizontal sub-pixel shift of cell" & C'Image);
         Check_Close (Dv (C), -0.2, 0.07, "vertical sub-pixel shift of cell" & C'Image);
      end loop;
      --  Six pixels in eight-pixel cells: the template leaves its window.
      Shift (6.0, 0.0);
      for C in Du'Range loop
         Check (not Resolved (C), "a shift of most of a cell is not resolved in cell" & C'Image);
      end loop;
      A := [others => 100.0];
      Flow.Displacements (G, A, A, Quantization, Du, Dv, Cond, Resolved);
      Check (Cond (1) = 0.0 and then Du (1) = 0.0 and then not Resolved (1),
             "a flat cell has no displacement, no condition and resolves nothing");
   end Flow_Recovers_Shifts;

   --  ── Regression ──

   procedure Regression_Ignores_Outliers is
      N : constant := 200;
      X : Real_Matrix (1 .. N, 1 .. 4);
      Y : Real_Array (1 .. N);
      Rng : Generator;
   begin
      for I in 1 .. N loop
         X (I, 1) := 1.0;
         X (I, 2) := Gaussian (Rng);
         X (I, 3) := Gaussian (Rng);
         X (I, 4) := 2.0 * X (I, 3);   --  a copy of the third column, scaled
         Y (I) := 2.0 + 3.0 * X (I, 2) - X (I, 3) + 0.01 * Gaussian (Rng);
         if I mod 10 = 0 then
            Y (I) := Y (I) + 100.0;      --  a tenth are wild
         end if;
      end loop;
      declare
         F : constant Regression.Fit := Regression.Solve (X, Y, 0.0);
         S : Real;
         K : Natural;
      begin
         Check_Close (F.Beta (1), 2.0, 0.01, "intercept");
         Check_Close (F.Beta (2), 3.0, 0.01, "slope of the second column");
         --  Columns 3 and 4 are one direction: only their combination is determined.
         Check_Close (F.Beta (3) + 2.0 * F.Beta (4), -1.0, 0.01, "combination of the collinear columns");
         Check (F.Scale < 0.02, "scale from the inliers:" & F.Scale'Image);
         Regression.Test_Block (F, 3, 4, S, K);
         Check (K = 1, "two collinear columns carry one degree of freedom, got" & K'Image);
         Check (Driver.Distributions.Chi_Square_Deviate (S, K) > Driver.Conventions.Z, "the collinear block is significant");
      end;
      --  A response unrelated to the columns gives an insignificant block.
      for I in 1 .. N loop
         Y (I) := 0.5 + Gaussian (Rng);
      end loop;
      declare
         F : constant Regression.Fit := Regression.Solve (X, Y, 0.0);
         S : Real;
         K : Natural;
      begin
         Regression.Test_Block (F, 2, 2, S, K);
         Check (Driver.Distributions.Chi_Square_Deviate (S, K) < Driver.Conventions.Z, "noise is not a response");
      end;
   end Regression_Ignores_Outliers;

   --  ── A synthetic body ──
   --
   --  Three eyes of 64 x 48 pixels. Eye 1 rides on arm 1 (group 1, two
   --  channels: one reading unit moves its view four pixels across and four
   --  down); a closer (group 3) moves a finger patch that rides with eye 1.
   --  Eye 2 rides on arm 2 (group 2). Eye 3 is fixed and sees both arms as
   --  patches and a part (group 4). Group 5 takes commands and moves
   --  nothing; group 6 is not commanded and reads the arms' sum (a sensor);
   --  group 7 is a constant. Every reading reaches its target at once, the
   --  images show the readings of the beat before (lag 1), and a push of the
   --  closer shakes arm 1's reading by a ten-millionth of the closer's
   --  change (a reaction).

   --  Of the latest Boot_On_Rig: how many of arm 1's keyframes were taken at a beat where eye 1's picture still
   --  changed by a level or more from the beat before, after the arm's last move (the rig's own account of its
   --  flicker): a keyframe waits for the picture to settle, so this must stay zero.
   Unsettled_Stills : Natural := 0;

   Rig_Width  : constant := 64;
   Rig_Height : constant := 48;
   Px_Per_Unit : constant := 4.0;

   type Rig_State is record
      Arm_1, Arm_2 : Real_Array (1 .. 2) := [0.0, 0.0];
      Closer, Part, Idle : Real := 0.0;
   end record;

   --  Eye 2 shows arm 2's motion by Near times the pixels a reading unit moves its view in the right quarter of the
   --  view (a near object), by Mid times them in the middle half (a middle distance) and by Far times them in the left
   --  quarter (the far background); all 1 is the same everywhere. An eye on a turning joint sees different amounts in
   --  different cells, by depth and by perspective.
   function Render (Eye : Positive; S : Rig_State; Near : Real := 1.0; Mid : Real := 1.0; Far : Real := 1.0)
     return Driver.Images.Image is
      use type Driver.Bytes.Offset;
      Data : Driver.Bytes.Byte_Array (1 .. 3 * Rig_Width * Rig_Height);
   begin
      for Y in 0 .. Rig_Height - 1 loop
         for X in 0 .. Rig_Width - 1 loop
            declare
               Xr : constant Real := Real (X);
               Yr : constant Real := Real (Y);
               L  : Real;
            begin
               case Eye is
                  when 1 =>
                     if Y >= 32 and then X in 16 .. 47 then
                        L := Texture (Xr + Px_Per_Unit * S.Closer, Yr, 1.0);
                     else
                        L := Texture (Xr + Px_Per_Unit * S.Arm_1 (1), Yr + Px_Per_Unit * S.Arm_1 (2));
                     end if;
                  when 2 =>
                     declare
                        Gain : constant Real :=
                          (if Xr >= 0.75 * Real (Rig_Width) then Near
                           elsif Xr >= 0.25 * Real (Rig_Width) then Mid else Far);
                     begin
                        L := Texture (Xr + Px_Per_Unit * Gain * S.Arm_2 (1), Yr + Px_Per_Unit * Gain * S.Arm_2 (2),
                                      2.0);
                     end;
                  when others =>
                     if Y < 16 and then X < 16 then
                        L := Texture (Xr + Px_Per_Unit * S.Arm_1 (1), Yr + Px_Per_Unit * S.Arm_1 (2), 3.0);
                     elsif Y < 16 and then X >= 48 then
                        L := Texture (Xr + Px_Per_Unit * S.Arm_2 (1), Yr + Px_Per_Unit * S.Arm_2 (2), 4.0);
                     elsif Y >= 32 and then X in 24 .. 39 then
                        L := Texture (Xr + Px_Per_Unit * S.Part, Yr, 5.0);
                     else
                        L := Texture (Xr, Yr, 6.0);
                     end if;
               end case;
               declare
                  V : constant Driver.Bytes.Byte := Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                  K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Rig_Width + X) + 1);
               begin
                  Data (K) := V;
                  Data (K + 1) := V;
                  Data (K + 2) := V;
               end;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Rig_Width, Rig_Height, Data);
   end Render;

   type Rig is record
      Shown    : Rig_State;   --  what the images show this beat (the readings of the beat before)
      Now      : Rig_State;   --  the readings this beat
      Previous : Rig_State;
      Beat     : Natural := 0;
      Timing   : Generator;   --  how long each push is held: a lock-in needs pushes that no
                              --  other group's train lines up with at any shift
   end record;

   --  Two to four beats.
   function Hold_Beats (R : in out Rig) return Positive is (2 + Natural (Real'Floor (3.0 * Uniform (R.Timing))));

   --  One beat: the robot is commanded to Target and reads Reading (where its
   --  response has got to), reports, and the model observes.
   procedure Step (M : in out Model; R : in out Rig; Target, Reading : Rig_State) is
      O    : Observation;
      Sent : Driver.Commands.Command;
   begin
      R.Previous := R.Now;
      R.Shown := R.Now;
      R.Now := Reading;
      --  The reaction: arm 1 shakes when the closer moves.
      R.Now.Arm_1 (1) := R.Now.Arm_1 (1) + 1.0e-7 * (Target.Closer - R.Previous.Closer);
      Driver.Commands.Set_Target (Sent, 1, Target.Arm_1);
      Driver.Commands.Set_Target (Sent, 2, Target.Arm_2);
      Driver.Commands.Set_Target (Sent, 3, [Target.Closer]);
      Driver.Commands.Set_Target (Sent, 4, [Target.Part]);
      Driver.Commands.Set_Target (Sent, 5, [Target.Idle]);
      O.Beat := Driver.Clock.Beat (R.Beat);
      for E in 1 .. 3 loop
         O.Images.Append (Render (E, R.Shown));
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      end loop;
      O.Readings.Append (R.Now.Arm_1);
      O.Readings.Append (R.Now.Arm_2);
      O.Readings.Append (Real_Array'(1 => R.Now.Closer));
      O.Readings.Append (Real_Array'(1 => R.Now.Part));
      O.Readings.Append (Real_Array'(1 => R.Now.Idle));
      O.Readings.Append (Real_Array'(1 => R.Now.Arm_1 (1) + R.Now.Arm_2 (1)));
      O.Readings.Append (Real_Array'(1 => 7.0));
      for G in 1 .. 7 loop
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      end loop;
      Observe (M, O, Sent);
      R.Beat := R.Beat + 1;
   end Step;

   --  The robot reaches the targets at once.
   procedure Step (M : in out Model; R : in out Rig; Target : Rig_State) is
   begin
      Step (M, R, Target, Target);
   end Step;

   type Push_Kind is (Arm_1, Arm_2, Arm_1_Joint_1, Arm_1_Joint_2, Arm_2_Joint_1, Arm_2_Joint_2, Closer, Part, Idle,
                      Lockstep, Closer_With_Arm_1);

   --  Pushes one group away by Amount and back, Times times, holding two
   --  beats after each move; Lockstep moves both arms together. Arm_1 and
   --  Arm_2 move both joints of the arm at once, in one ratio; Arm_1_Joint_2
   --  and Arm_2_Joint_1 move the arm's other joint alone, as the recognition's
   --  directions rounds push one channel at a time: keyframes of joints that
   --  only ever moved together in one ratio cannot tell how far the view
   --  moves per unit of each (Kinematics.Measure_Shifts takes a point's
   --  coefficients only when its keyframes determine them all).
   procedure Exercise (M : in out Model; R : in out Rig; Kind : Push_Kind; Amount : Real; Times : Positive) is
      Rest : constant Rig_State := (others => <>);
      Away : Rig_State := Rest;
   begin
      case Kind is
         when Arm_1    => Away.Arm_1 := [Amount, Amount / 2.0];
         when Arm_2    => Away.Arm_2 := [Amount / 2.0, Amount];
         when Arm_1_Joint_1 => Away.Arm_1 := [Amount, 0.0];
         when Arm_1_Joint_2 => Away.Arm_1 := [0.0, Amount];
         when Arm_2_Joint_1 => Away.Arm_2 := [Amount, 0.0];
         when Arm_2_Joint_2 => Away.Arm_2 := [0.0, Amount];
         when Closer   => Away.Closer := Amount;
         when Part     => Away.Part := Amount;
         when Idle     => Away.Idle := Amount;
         when Lockstep =>
            Away.Arm_1 := [Amount, 0.0];
            Away.Arm_2 := [Amount, 0.0];
         when Closer_With_Arm_1 =>
            Away.Arm_1 := [Amount, Amount / 2.0];
            Away.Closer := Amount;
      end case;
      for T in 1 .. Times loop
         if Kind = Closer_With_Arm_1 then
            --  Commanded together, the closer answering a beat after the arm:
            --  at that beat it alone still moves.
            for K in 1 .. 2 loop
               declare
                  Goal  : constant Rig_State := (if K = 1 then Away else Rest);
                  First : Rig_State := Goal;
               begin
                  First.Closer := (if K = 1 then Amount / 2.0 else Amount / 2.0);
                  Step (M, R, Goal, First);
                  for B in 1 .. Hold_Beats (R) loop
                     Step (M, R, Goal);
                  end loop;
               end;
            end loop;
         elsif Kind = Lockstep then
            --  Arm 1 overshoots and comes back while arm 2 is still on its way:
            --  arm 1 moves away from its target while its own push is still
            --  being answered.
            for K in 1 .. 2 loop
               declare
                  Goal : constant Rig_State := (if K = 1 then Away else Rest);
                  From : constant Real := (if K = 1 then 0.0 else Amount);
                  To   : constant Real := (if K = 1 then Amount else 0.0);
                  Arm_1_Path : constant Real_Array (1 .. 3) := [0.97, 1.07, 1.0];
                  Arm_2_Path : constant Real_Array (1 .. 3) := [0.5, 0.9, 1.0];
               begin
                  for B in 1 .. 3 loop
                     declare
                        Now : Rig_State := Goal;
                     begin
                        Now.Arm_1 (1) := From + (To - From) * Arm_1_Path (B);
                        Now.Arm_2 (1) := From + (To - From) * Arm_2_Path (B);
                        Step (M, R, Goal, Now);
                     end;
                  end loop;
                  for B in 1 .. Hold_Beats (R) loop
                     Step (M, R, Goal);
                  end loop;
               end;
            end loop;
         else
            for B in 1 .. Hold_Beats (R) loop
               Step (M, R, Away);
            end loop;
            for B in 1 .. Hold_Beats (R) loop
               Step (M, R, Rest);
            end loop;
         end if;
      end loop;
   end Exercise;

   procedure Exercise_Rig (M : in out Model; With_Lockstep : Boolean := True) is
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
   begin
      for B in 1 .. 5 loop
         Step (M, R, Rest);
      end loop;
      Exercise (M, R, Arm_1, 0.1, 8);
      Exercise (M, R, Arm_2, 0.1, 8);
      Exercise (M, R, Closer, 0.1, 8);
      Exercise (M, R, Part, 0.1, 8);
      Exercise (M, R, Idle, 0.1, 8);
      if With_Lockstep then
         --  Ten pixels and more per beat: far beyond a one-step displacement.
         Exercise (M, R, Lockstep, 3.0, 10);
      end if;
      Estimate_Now (M);
      --  The arms recognized, their sweeps begin (the boot's Begin_Anew at the sweep's hold): an arm takes no
      --  keyframe before. The joints pushed again from here, together and alone, give the keyframes that measure
      --  how far the view moves per unit of each.
      Driver.Robot.Kinematics.Begin_Anew (M, 1);
      Driver.Robot.Kinematics.Begin_Anew (M, 2);
      Exercise (M, R, Arm_1, 0.1, 8);
      Exercise (M, R, Arm_2, 0.1, 8);
      --  A joint alone by a third of the amount: the rig's view moves 7 px per 0.1 of its second joint, beyond
      --  what the aligner follows in a 64 by 48 picture from a prediction of nothing.
      Exercise (M, R, Arm_1_Joint_1, 0.03, 8);
      Exercise (M, R, Arm_1_Joint_2, 0.03, 8);
      Exercise (M, R, Arm_2_Joint_1, 0.03, 8);
      Exercise (M, R, Arm_2_Joint_2, 0.03, 8);
      Estimate_Now (M);
   end Exercise_Rig;

   --  The decider runs the estimates in a task of its own, whose stack is
   --  GNAT's default (2 MiB), a fraction of the environment task's where the
   --  other tests estimate: nothing an estimate keeps per beat, cell or pixel
   --  may live on a stack. A long history, every group pushed in turn, then
   --  the estimate in such a task. The estimators that kept their samples on
   --  the stack overflowed it on this rig between 10 600 and 12 000 beats.
   procedure Estimate_In_A_Task is
      M       : Model;
      R       : Rig;
      Rest    : constant Rig_State := (others => <>);
      History : constant := 16_000;
      Done    : Boolean := False with Atomic;
      Failure : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for B in 1 .. 5 loop
         Step (M, R, Rest);
      end loop;
      while R.Beat < History loop
         Exercise (M, R, Arm_1, 0.1, 8);
         Exercise (M, R, Arm_2, 0.1, 8);
         Exercise (M, R, Closer, 0.1, 8);
         Exercise (M, R, Part, 0.1, 8);
         Exercise (M, R, Idle, 0.1, 8);
      end loop;
      declare
         task Decider;
         task body Decider is
         begin
            Estimate_Now (M);
            Done := True;
         exception
            when E : others =>
               Failure := Ada.Strings.Unbounded.To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
         end Decider;
      begin
         null;
      end;
      Check (Done, "the estimate over" & R.Beat'Image & " beats failed in a task: "
             & Ada.Strings.Unbounded.To_String (Failure));
      if Done then
         Check (Role (M, 1) = Arm, "arm 1 is an arm, got " & Role (M, 1)'Image);
         Check (Role (M, 2) = Arm, "arm 2 is an arm, got " & Role (M, 2)'Image);
      end if;
   end Estimate_In_A_Task;

   --  A gripper held at an opening while the arm moves the whole hand in its own eye (A29: 8136 beats of presses turned
   --  both grippers into arms, and the boot died of it): the rows of a gripper's push that count are the ones where
   --  nothing else moves its picture. Two kinds of press leave the picture moving under the gripper's push without a
   --  push of the arm under way: the arm moves though no push of it is under way (one that ended or was given up while
   --  it kept moving), and the picture goes on changing after the arm has stopped (the tail of its motion, a hand that
   --  settles against the table); in both the picture's motion is the arm's, and credited to the gripper it makes the
   --  whole picture its effect. Twenty presses of each kind after the body was recognized: the gripper is a closer
   --  still, as it was.
   procedure Presses_Leave_A_Closer_A_Closer is
      type Kind_Of_Press is (Arm_Moves_Unpushed, Picture_Tail);

      --  One beat of the rig whose picture follows the arm a halving behind it, as a rendered view settles: the gripper
      --  shows at once, the arm's view closes half of what is left to the arm's reading every beat.
      procedure Step_With_Tail (M : in out Model; R : in out Rig; Target : Rig_State; Tail : in out Rig_State) is
         O    : Observation;
         Sent : Driver.Commands.Command;
      begin
         R.Previous := R.Now;
         R.Now := Target;
         R.Shown := Tail;
         R.Shown.Closer := R.Previous.Closer;
         for C in 1 .. 2 loop
            Tail.Arm_1 (C) := Tail.Arm_1 (C) + 0.5 * (R.Now.Arm_1 (C) - Tail.Arm_1 (C));
            Tail.Arm_2 (C) := Tail.Arm_2 (C) + 0.5 * (R.Now.Arm_2 (C) - Tail.Arm_2 (C));
         end loop;
         Driver.Commands.Set_Target (Sent, 1, Target.Arm_1);
         Driver.Commands.Set_Target (Sent, 2, Target.Arm_2);
         Driver.Commands.Set_Target (Sent, 3, [Target.Closer]);
         Driver.Commands.Set_Target (Sent, 4, [Target.Part]);
         Driver.Commands.Set_Target (Sent, 5, [Target.Idle]);
         O.Beat := Driver.Clock.Beat (R.Beat);
         for E in 1 .. 3 loop
            O.Images.Append (Render (E, R.Shown));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         O.Readings.Append (R.Now.Arm_1);
         O.Readings.Append (R.Now.Arm_2);
         O.Readings.Append (Real_Array'(1 => R.Now.Closer));
         O.Readings.Append (Real_Array'(1 => R.Now.Part));
         O.Readings.Append (Real_Array'(1 => R.Now.Idle));
         O.Readings.Append (Real_Array'(1 => R.Now.Arm_1 (1) + R.Now.Arm_2 (1)));
         O.Readings.Append (Real_Array'(1 => 7.0));
         for G in 1 .. 7 loop
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         Observe (M, O, Sent);
         R.Beat := R.Beat + 1;
      end Step_With_Tail;

      procedure Run (Kind : Kind_Of_Press; Before, After : out Group_Role) is
         M    : Model;
         R    : Rig;
         Rest : constant Rig_State := (others => <>);
         Tail : Rig_State := Rest;
      begin
         for B in 1 .. 5 loop
            Step (M, R, Rest);
         end loop;
         Exercise (M, R, Arm_1, 0.1, 8);
         Exercise (M, R, Arm_2, 0.1, 8);
         Exercise (M, R, Closer, 0.1, 8);
         Exercise (M, R, Part, 0.1, 8);
         Exercise (M, R, Idle, 0.1, 8);
         Estimate_Now (M);
         Before := Role (M, 3);
         for Press in 1 .. 20 loop
            case Kind is
               when Arm_Moves_Unpushed =>
                  --  The gripper is asked away and back, and the arm, asked nothing, moves with it each time: out
                  --  as the gripper is asked away, and back as it is asked back.
                  for K in 1 .. 6 loop
                     declare
                        Goal    : Rig_State := Rest;
                        Reading : Rig_State := Rest;
                     begin
                        Goal.Closer := (if K in 1 .. 3 then 0.1 else 0.0);
                        Reading.Closer := Goal.Closer;
                        if K in 1 .. 3 then
                           Reading.Arm_1 := [0.1, 0.05];
                        end if;
                        Step (M, R, Goal, Reading);
                     end;
                  end loop;
                  for K in 1 .. 4 loop
                     Step (M, R, Rest);
                  end loop;
               when Picture_Tail =>
                  --  The arm is asked away and back, and the gripper asked two beats after it stopped, while the
                  --  picture is still closing on the arm.
                  declare
                     Out_Goal : Rig_State := Rest;
                  begin
                     Out_Goal.Arm_1 := [0.1, 0.05];
                     Step_With_Tail (M, R, Out_Goal, Tail);
                     Step_With_Tail (M, R, Out_Goal, Tail);
                     Step_With_Tail (M, R, Rest, Tail);
                     Step_With_Tail (M, R, Rest, Tail);
                     Out_Goal := Rest;
                     Out_Goal.Closer := 0.1;
                     Step_With_Tail (M, R, Out_Goal, Tail);
                     Step_With_Tail (M, R, Out_Goal, Tail);
                     Step_With_Tail (M, R, Rest, Tail);
                     Step_With_Tail (M, R, Rest, Tail);
                  end;
                  for K in 1 .. 12 loop
                     Step_With_Tail (M, R, Rest, Tail);
                  end loop;
            end case;
         end loop;
         Estimate_Now (M);
         After := Role (M, 3);
      end Run;

      Before, After : Group_Role;
   begin
      for Kind in Kind_Of_Press loop
         Run (Kind, Before, After);
         Check (Before = Closer, "the gripper is a closer before its presses (" & Kind'Image & "), got " & Before'Image);
         Check (After = Closer, "the gripper is" & After'Image & " after twenty presses (" & Kind'Image
                & "), not a closer");
      end loop;
   end Presses_Leave_A_Closer_A_Closer;

   --  A group that moves a patch in arm 1's eye while another eye is still
   --  undecided about it (a live x5's right arm at its first read: a patch
   --  in the left wrist's eye, the right wrist's eye undecided at 267 of 525
   --  cells) may carry that eye: it is no closer until the eye decides, and
   --  a closer once it decides that nothing moved.
   procedure Undecided_Eye_Leaves_Group_Unclassified is
      M : Model;
      Undecided_Third : constant Eye_Effect :=
        (Verdict => Undecided, Responding => 24, Textured => 48,
         Fraction => (Value => 0.5, Sigma => 0.07, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
      Nothing_Third : constant Eye_Effect :=
        (Verdict => Nothing, Responding => 0, Textured => 48,
         Fraction => (Value => 0.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
   begin
      Exercise_Rig (M);
      Check (Role (M, 3) = Closer, "the closer is a closer, got " & Role (M, 3)'Image);
      M.Graph.Effects.Replace_Element ((3 - 1) * Eye_Count (M) + 3, Undecided_Third);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, 3) = Unclassified, "a group an eye is undecided about is " & Role (M, 3)'Image);
      M.Graph.Effects.Replace_Element ((3 - 1) * Eye_Count (M) + 3, Nothing_Third);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, 3) = Closer, "once the eye decided nothing moved, the group is " & Role (M, 3)'Image);
   end Undecided_Eye_Leaves_Group_Unclassified;

   --  A fixed eye sees an arm wherever the arm's pushes move cells of its view, whatever the lock-in made of how many:
   --  an arm swept across most of the view leaves more than half its cells responding, neither a patch nor the whole
   --  (the hall's fixed eyes of round n15s2, 10-27: a patch of 50 to 97 cells after step 1, undecided at 100 to 148
   --  once the sweeps were in the lock-in, and step 3 placed none of them).
   procedure Fixed_Eye_Sees_An_Undecided_Arm is
      M : Model;
      function Effect (V : Eye_Response; Responding : Natural) return Eye_Effect is
        (Verdict => V, Responding => Responding, Textured => 48,
         Fraction => (Value => Real (Responding) / 48.0, Sigma => 0.07, Degrees_Of_Freedom => 0), Resting => 0.0,
         Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
      Index : Positive;
   begin
      Exercise_Rig (M);
      Check (Eye_Mount (M, 3).Kind = World_Fixed, "eye 3 is fixed");
      Index := (1 - 1) * Eye_Count (M) + 3;
      M.Graph.Effects.Replace_Element (Index, Effect (Undecided, 30));
      Check (Driver.Robot.Kinematics.Sees (M, 3, 1), "a fixed eye undecided about an arm does not see it");
      M.Graph.Effects.Replace_Element (Index, Effect (Patch, 12));
      Check (Driver.Robot.Kinematics.Sees (M, 3, 1), "a fixed eye the arm moves a patch of does not see it");
      M.Graph.Effects.Replace_Element (Index, Effect (Nothing, 0));
      Check (not Driver.Robot.Kinematics.Sees (M, 3, 1), "a fixed eye the arm moves nothing of sees it");
      M.Graph.Effects.Replace_Element (Index, (Verdict => Unmeasured, others => <>));
      Check (not Driver.Robot.Kinematics.Sees (M, 3, 1), "a fixed eye not yet measured with the arm sees it");
   end Fixed_Eye_Sees_An_Undecided_Arm;

   --  The walk's body eye (Kinematics.Base.Body_Eye_Of): an eye the carrier carries, though the arm's pushes move
   --  its view whole, as a swaying body's every push does (the hall's G1, n16s2, 10-27: its head's view moved whole
   --  by every arm, and no walk); an eye on another arm only where the arm's own pushes do not move it whole (an arm
   --  under that one carries it).
   procedure Body_Eye_By_Mounts is
      M : Model;
      function Effect (V : Eye_Response) return Eye_Effect is
        (Verdict => V, Responding => (if V = Nothing then 0 else 40), Textured => 48,
         Fraction => (Value => (if V = Nothing then 0.0 else 40.0 / 48.0), Sigma => 0.05, Degrees_Of_Freedom => 0),
         Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
      Carrier : constant Group_Id := 5;   --  the rig's idle group, taken for what carries the body
      function Index (G : Group_Id; E : Eye_Id) return Positive is ((Natural (G) - 1) * Eye_Count (M) + Natural (E));
   begin
      Exercise_Rig (M);
      M.Graph.Carrier := Carrier;
      M.Graph.Mounts.Replace_Element (3, Mount'(Kind => Carrier_Carried));
      M.Graph.Effects.Replace_Element (Index (Carrier, 3), Effect (Whole));
      M.Graph.Effects.Replace_Element (Index (Arm_Group (M, 1), 3), Effect (Whole));
      Check (Driver.Robot.Kinematics.Base.Body_Eye_Of (M, 1) = 3,
             "an eye the carrier carries, its view moved whole by the arm's pushes too, is not the arm's body eye: got"
             & Driver.Robot.Kinematics.Base.Body_Eye_Of (M, 1)'Image);
      M.Graph.Mounts.Replace_Element (3, Mount'(Kind => Arm_Carried, Arm => 2));
      Check (Driver.Robot.Kinematics.Base.Body_Eye_Of (M, 1) /= 3,
             "an eye on another arm, its view moved whole by this arm's pushes, is this arm's body eye");
      M.Graph.Effects.Replace_Element (Index (Arm_Group (M, 1), 3), Effect (Patch));
      Check (Driver.Robot.Kinematics.Base.Body_Eye_Of (M, 1) = 3,
             "an eye on another arm that the carrier moves whole and this arm does not is not its body eye: got"
             & Driver.Robot.Kinematics.Base.Body_Eye_Of (M, 1)'Image);
   end Body_Eye_By_Mounts;

   --  A group whose channels move different eyes (a humanoid's neck moves the head's eye and its arm a wrist's, one
   --  group of every joint) has an arm for each such eye, each carrying its eye; a channel an eye is undecided about
   --  is no evidence that it does not move it, and the group stays one arm.
   procedure Group_Of_Different_Eyes_Has_An_Arm_Each is
      M : Model;
      Whole_Effect : constant Eye_Effect :=
        (Verdict => Whole, Responding => 48, Textured => 48,
         Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0,
         Could_Be_Whole => True, Far_Chance => 1.0);
      Nothing_Effect : constant Eye_Effect :=
        (Verdict => Nothing, Responding => 0, Textured => 48,
         Fraction => (Value => 0.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0,
         Could_Be_Whole => True, Far_Chance => 1.0);
      Undecided_Effect : constant Eye_Effect :=
        (Verdict => Undecided, Responding => 24, Textured => 48,
         Fraction => (Value => 0.5, Sigma => 0.07, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0,
         Could_Be_Whole => True, Far_Chance => 1.0);
      Group : constant Group_Id := 1;   --  the rig's first arm, of two channels

      procedure Set (Eye_1_Channel_1, Eye_1_Channel_2, Eye_2_Channel_1, Eye_2_Channel_2, Third : Eye_Effect) is
      begin
         M.Graph.Channels.Clear;
         for E in 1 .. Eye_Count (M) loop
            M.Graph.Channels.Append (Channel_Effect_Vectors.Empty_Vector);
         end loop;
         M.Graph.Channels (1).Append (Channel_Effect'(Group => Group, Channel => 1, Effect => Eye_1_Channel_1));
         M.Graph.Channels (1).Append (Channel_Effect'(Group => Group, Channel => 2, Effect => Eye_1_Channel_2));
         M.Graph.Channels (2).Append (Channel_Effect'(Group => Group, Channel => 1, Effect => Eye_2_Channel_1));
         M.Graph.Channels (2).Append (Channel_Effect'(Group => Group, Channel => 2, Effect => Eye_2_Channel_2));
         for E in 3 .. Eye_Count (M) loop
            M.Graph.Channels (Eye_Id (E)).Append (Channel_Effect'(Group => Group, Channel => 1, Effect => Third));
            M.Graph.Channels (Eye_Id (E)).Append (Channel_Effect'(Group => Group, Channel => 2, Effect => Third));
         end loop;
         Driver.Robot.Graph.Derive (M);
      end Set;

      function Arms_Of_Group (Of_Model : Model) return Natural is
      begin
         return N : Natural := 0 do
            for A in 1 .. Arm_Count (Of_Model) loop
               if Arm_Group (Of_Model, Arm_Id (A)) = Group then
                  N := N + 1;
               end if;
            end loop;
         end return;
      end Arms_Of_Group;
   begin
      Exercise_Rig (M);
      Check (Arms_Of_Group (M) = 1, "the rig's first group is" & Natural'Image (Arms_Of_Group (M)) & " arms, not one");
      --  Channel 1 moves eye 1 whole and eye 2 not; channel 2 eye 2 whole and eye 1 not.
      M.Groups (Group).Channels_Alone := True;
      Set (Whole_Effect, Nothing_Effect, Nothing_Effect, Whole_Effect, Nothing_Effect);
      Check (Arms_Of_Group (M) = 2, "a group whose channels move two eyes apart is" & Natural'Image (Arms_Of_Group (M)) & " arms, not two");
      Check (Eye_Mount (M, 1).Kind = Arm_Carried and then Eye_Mount (M, 2).Kind = Arm_Carried
             and then Eye_Mount (M, 1).Arm /= Eye_Mount (M, 2).Arm
             and then Arm_Group (M, Eye_Mount (M, 1).Arm) = Group and then Arm_Group (M, Eye_Mount (M, 2).Arm) = Group,
             "the two eyes do not ride on two arms of the group");
      --  The two arms of the group each keep their own evidence through the beats that follow: the evidence was found
      --  by the group alone, and two arms of one group took each other's over every beat (the humanoid's arm 1 had
      --  no current evidence for 7700 beats, and its reference never got a still twin, m1020_h7, 10-20).
      declare
         R    : Rig;
         Rest : constant Rig_State := (others => <>);
         Cause, Detail : Ada.Strings.Unbounded.Unbounded_String;
      begin
         R.Beat := M.Beats;
         for B in 1 .. 3 loop
            Step (M, R, Rest);
         end loop;
         for A in 1 .. Arm_Count (M) loop
            if Arm_Group (M, Arm_Id (A)) = Group then
               Driver.Robot.Kinematics.Twin_Blocked_By (M, Arm_Id (A), Cause, Detail);
               Check (Ada.Strings.Unbounded.Index (Cause, "no evidence") = 0,
                      "arm" & A'Image & " of the split group has no current evidence: "
                      & Ada.Strings.Unbounded.To_String (Detail));
            end if;
         end loop;
      end;
      --  Both channels move both eyes (an arm with two cameras): one arm.
      Set (Whole_Effect, Whole_Effect, Whole_Effect, Whole_Effect, Nothing_Effect);
      Check (Arms_Of_Group (M) = 1, "channels that move the same two eyes split the group into" & Natural'Image (Arms_Of_Group (M)) & " arms");
      --  Channel 1 is no evidence against eye 2 when every other eye is undecided about it: not told apart.
      Set (Whole_Effect, Nothing_Effect, Undecided_Effect, Whole_Effect, Undecided_Effect);
      Check (Arms_Of_Group (M) = 1, "a channel the other eyes are undecided about split the group into" & Natural'Image (Arms_Of_Group (M)) & " arms");
      --  The split is kept in the body file, and held while the lock-in says nothing of the group's channels.
      Set (Whole_Effect, Nothing_Effect, Nothing_Effect, Whole_Effect, Nothing_Effect);
      declare
         Back : Model;
         Ok   : Boolean;
         Why  : Ada.Strings.Unbounded.Unbounded_String;
      begin
         Driver.Robot.Body_File.Read (Back, Driver.Robot.Body_File.Text (M), Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         Check (Arms_Of_Group (Back) = 2, "the reloaded body has" & Natural'Image (Arms_Of_Group (Back)) & " arms of the group, not two");
         Check (Eye_Mount (Back, 1).Kind = Arm_Carried and then Eye_Mount (Back, 2).Kind = Arm_Carried
                and then Eye_Mount (Back, 1).Arm /= Eye_Mount (Back, 2).Arm,
                "the reloaded body's two eyes do not ride on two arms");
      end;
      --  And held while the lock-in tells nothing of the channels (an estimate whose regression had too few rows).
      M.Graph.Channels.Clear;
      Driver.Robot.Graph.Derive (M);
      Check (Arms_Of_Group (M) = 2,
             "a derivation with no channel told lost the split: " & Natural'Image (Arms_Of_Group (M)) & " arms");
   end Group_Of_Different_Eyes_Has_An_Arm_Each;

   --  A closer's remembered eye (Group_Stream.Closer_Eye) yields to another eye on an arm whose view the group's pushes
   --  left significantly more of beyond the flow's reach, and only to such a one; where the lock-in has not measured
   --  the group in the remembered eye, it tells nothing and keeps the group (w11, servo seed 11 of the fake robot, 10-19:
   --  arm 1's closer stayed on arm 2 by arm 2's eye, 6 to 8 cells, though arm 1's own eye showed it in 36; Stretch's
   --  gripper, seed 2, paced round p89a, 10-24: on the head's arm by 216 cells responding in the head's eye to 107 in
   --  its wrist's, the robot rocking as it moved, where its pushes left 1 % and 14 % of the two views beyond reach).
   procedure Clearer_Eye_Takes_A_Closer is
      M     : Model;
      Group : constant Group_Id := 3;   --  the rig's closer
      Own   : Eye_Effect;
      Cells : constant Natural := 1000;   --  the textured cells the group's pushes were counted over, in each eye
      Kept  : constant Natural := 100;    --  of them, those its pushes left beyond the flow's reach in eye 1
      Least : Natural := 0;   --  the fewest left beyond reach in eye 2 that are significantly more than eye 1's
      function Index_Of (E : Eye_Id) return Positive is ((Natural (Group) - 1) * Eye_Count (M) + Natural (E));
      --  The group's pushes left Far of eye E's textured cells beyond the flow's reach (Motion.Note_Stayed).
      procedure Leave (E : Eye_Id; Far : Natural) is
         S : Eye_Stream renames M.Eyes (E);
      begin
         while Natural (S.Stayed_Far.Length) < Natural (Group) loop
            S.Stayed_Far.Append (0.0);
         end loop;
         while Natural (S.Stayed_Pushes.Length) < Natural (Group) loop
            S.Stayed_Pushes.Append (0);
         end loop;
         while Natural (S.Stayed_Cells.Length) < Natural (Group) loop
            S.Stayed_Cells.Append (0);
         end loop;
         while Natural (S.Stayed_Far_Cells.Length) < Natural (Group) loop
            S.Stayed_Far_Cells.Append (0);
         end loop;
         while Natural (S.Stayed_Moved.Length) < Natural (Group) loop
            S.Stayed_Moved.Append (0);
         end loop;
         S.Stayed_Far.Replace_Element (Natural (Group) - 1, Real (Far) / Real (Cells));
         S.Stayed_Pushes.Replace_Element (Natural (Group) - 1, 1);
         S.Stayed_Cells.Replace_Element (Natural (Group) - 1, Cells);
         S.Stayed_Far_Cells.Replace_Element (Natural (Group) - 1, Far);
         S.Stayed_Moved.Replace_Element (Natural (Group) - 1, Far);
         while Natural (S.Stayed_Least_Cells.Length) < Natural (Group) loop
            S.Stayed_Least_Far.Append (0);
            S.Stayed_Least_Cells.Append (0);
         end loop;
         S.Stayed_Least_Far.Replace_Element (Natural (Group) - 1, Far);
         S.Stayed_Least_Cells.Replace_Element (Natural (Group) - 1, Cells);
      end Leave;
   begin
      Exercise_Rig (M);
      Own := M.Graph.Effects (Index_Of (1));
      Check (Role (M, Group) = Closer and then Own.Verdict = Patch
             and then Eye_Mount (M, 1).Kind = Arm_Carried and then Eye_Mount (M, 2).Kind = Arm_Carried
             and then Eye_Mount (M, 1).Arm /= Eye_Mount (M, 2).Arm
             and then M.Graph.Arm_Of (Group) = Eye_Mount (M, 1).Arm,
             "the rig's closer is no patch of eye 1 on eye 1's arm, or eyes 1 and 2 ride no two arms");
      if Own.Verdict /= Patch or else Eye_Mount (M, 2).Kind /= Arm_Carried then
         return;
      end if;
      for N in Kept + 1 .. Cells loop
         if Driver.Robot.Regression.Shares_Apart (N, Cells, Kept, Cells) then
            Least := N;
            exit;
         end if;
      end loop;
      Check (Least > Kept + 1,
             "of" & Cells'Image & " cells, none left beyond reach outnumber eye 1's" & Kept'Image
             & " both by one at least and not significantly");
      if Least <= Kept + 1 then
         return;
      end if;
      --  Eye 2 shows the group a patch as eye 1 does, its pushes leaving none of it beyond reach: eye 1 keeps it, weighed
      --  against eye 2 from here on.
      M.Graph.Effects.Replace_Element (Index_Of (2), Own);
      Leave (1, Kept);
      Leave (2, 0);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, Group) = Closer and then M.Graph.Arm_Of (Group) = Eye_Mount (M, 1).Arm,
             "eye 2 left none beyond reach against" & Kept'Image & " in eye 1 and took the closer, "
             & Role (M, Group)'Image);
      Leave (2, Least - 1);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, Group) = Closer and then M.Graph.Arm_Of (Group) = Eye_Mount (M, 1).Arm,
             "eye 2 left" & Natural'Image (Least - 1) & " of" & Cells'Image & " beyond reach against" & Kept'Image
             & " in eye 1 and took the closer, " & Role (M, Group)'Image);
      Leave (2, Least);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, Group) = Closer and then M.Graph.Arm_Of (Group) = Eye_Mount (M, 2).Arm,
             "eye 2 left" & Least'Image & " of" & Cells'Image & " beyond reach against" & Kept'Image
             & " in eye 1 and the closer stayed on eye 1's arm, " & Role (M, Group)'Image);
      M.Graph.Effects.Replace_Element (Index_Of (2), (Verdict => Unmeasured, others => <>));
      Leave (1, Cells);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, Group) = Closer and then M.Graph.Arm_Of (Group) = Eye_Mount (M, 2).Arm,
             "eye 2 not measured, the closer left its arm for eye 1's, " & Role (M, Group)'Image);
   end Clearer_Eye_Takes_A_Closer;

   --  An arm whose own eye the lock-in calls undecided keeps that eye while its keyframes tell that the eye's view
   --  moves with it (Arm_Evidence.Shift_Seen): the aligner measures over the whole picture what the lock-in's flow
   --  measures a beat at a time (f15o on 10-15: the lock-in, regressing the sweep's and the hand's beats again,
   --  read arm 1's eye as undecided, the graph took the eye off the arm, and the hand measured in it was dropped).
   --  Where no keyframe told, the lock-in alone decides: undecided is no verdict against the eye the arm carried
   --  (Group_Stream.Rider), which stays; an eye in which the arm moves nothing goes (robot.boot.patch, 10-17: an
   --  arm under its sweep, its keyframes not yet telling, lost its eye to an undecided verdict between two passes).
   procedure Keyframes_Hold_An_Eye_On_Its_Arm is
      M : Model;
      Undecided_Own : constant Eye_Effect :=
        (Verdict => Undecided, Responding => 24, Textured => 48,
         Fraction => (Value => 0.5, Sigma => 0.07, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
      Nothing_Own : constant Eye_Effect :=
        (Verdict => Nothing, Responding => 0, Textured => 48,
         Fraction => (Value => 0.0, Sigma => 0.07, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => False, Far_Chance => 1.0);
      G : Group_Id;
      E : Eye_Id'Base := 0;
      K : Natural := 0;   --  arm 1's evidence
   begin
      Exercise_Rig (M);
      G := Arm_Group (M, 1);
      for I in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if M.Kinematics (I).Group = G then
            K := I;
            E := M.Kinematics (I).Eye;
         end if;
      end loop;
      Check (K > 0 and then E > 0, "arm 1 has no evidence with an eye after its sweep");
      if K = 0 or else E = 0 then
         return;
      end if;
      Check ((for some S of M.Kinematics (K).Shift_Seen => S),
             "no keyframe of arm 1 told that its eye's view moves with a joint");
      M.Graph.Effects.Replace_Element ((Natural (G) - 1) * Eye_Count (M) + Natural (E), Undecided_Own);
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, G) = Arm, "arm 1, its own eye undecided and its keyframes telling, is " & Role (M, G)'Image);
      Check (Eye_Mount (M, E).Kind = Arm_Carried and then Eye_Mount (M, E).Arm = M.Graph.Arm_Of (G),
             "arm 1's eye, undecided in the lock-in while the keyframes tell, is " & Eye_Mount (M, E).Kind'Image);
      for C in 0 .. Natural (M.Kinematics (K).Shift_Seen.Length) - 1 loop
         M.Kinematics (K).Shift_Seen.Replace_Element (C, False);
      end loop;
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, G) = Arm and then Eye_Mount (M, E).Kind = Arm_Carried
             and then Eye_Mount (M, E).Arm = M.Graph.Arm_Of (G),
             "with no keyframe telling, an eye the lock-in is undecided about no longer rides arm 1: "
             & Role (M, G)'Image & ", " & Eye_Mount (M, E).Kind'Image);
      M.Graph.Effects.Replace_Element ((Natural (G) - 1) * Eye_Count (M) + Natural (E), Nothing_Own);
      Driver.Robot.Graph.Derive (M);
      Check (not (Eye_Mount (M, E).Kind = Arm_Carried and then Eye_Mount (M, E).Arm = M.Graph.Arm_Of (G)),
             "with no keyframe telling, an eye in which arm 1 moves nothing still rides it");
   end Keyframes_Hold_An_Eye_On_Its_Arm;

   --  A group that carries an eye moves the whole of its picture, though much
   --  of the picture is so faintly textured that its cells cannot tell pushes
   --  as small as the first ones from their own noise (a live x5's right arm,
   --  A14: 218 of the 525 cells of the eye it carries responded, and the eye
   --  was called a patch of the group's, so the arm was never swept). The
   --  cells that cannot tell are no evidence against the whole picture moving;
   --  together they show it. The same picture with only the well-textured
   --  quarter moving is a patch, however faint the rest: there the faint
   --  cells together show nothing. And when the pushes are too small for even
   --  the faint cells together to show the motion, the well-textured quarter
   --  cannot say a patch from the whole: the verdict is undecided, so the boot
   --  pushes harder, rather than a patch.
   procedure Carried_Eye_Partly_Textureless is
      W       : constant := 256;
      H       : constant := 192;
      Strong  : constant := 48;      --  columns, from the left, with the texture the rig's eyes have
      Faint   : constant := 0.02;    --  the texture of the others, as a share of that
      Pushes  : constant := 14;
      Push    : constant := 0.006;   --  reading units, a shift of the picture of 0.024 pixels
      Weaker  : constant := 0.0025;  --  the same pushes, 2.4 times smaller

      function Frame (Shown : Real; Faint_Moves : Boolean) return Driver.Images.Image is
         use type Driver.Bytes.Offset;
         Data : Driver.Bytes.Byte_Array (1 .. 3 * W * H);
      begin
         for Y in 0 .. H - 1 loop
            for X in 0 .. W - 1 loop
               declare
                  Moved : constant Real := (if X < Strong or else Faint_Moves then Px_Per_Unit * Shown else 0.0);
                  Rich  : constant Real := Texture (Real (X) + Moved, Real (Y));
                  L     : constant Real := (if X < Strong then Rich else 128.0 + Faint * (Rich - 128.0));
                  V     : constant Driver.Bytes.Byte := Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                  K     : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * W + X) + 1);
               begin
                  Data (K) := V;
                  Data (K + 1) := V;
                  Data (K + 2) := V;
               end;
            end loop;
         end loop;
         return Driver.Images.Create (W, H, Data);
      end Frame;

      --  The group is pushed away and back Pushes times, holding three beats
      --  each way; its picture shows the reading of the beat before.
      procedure Run (M : in out Model; Faint_Moves : Boolean; By : Real) is
         Reading, Shown : Real := 0.0;
         Beat : Natural := 0;

         procedure Step (Target : Real) is
            O    : Observation;
            Sent : Driver.Commands.Command;
         begin
            Shown := Reading;
            Reading := Target;
            Driver.Commands.Set_Target (Sent, 1, [Target]);
            O.Beat := Driver.Clock.Beat (Beat);
            O.Images.Append (Frame (Shown, Faint_Moves));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Readings.Append (Real_Array'(1 => Reading));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            Observe (M, O, Sent);
            Beat := Beat + 1;
         end Step;
      begin
         for B in 1 .. 5 loop
            Step (0.0);
         end loop;
         for P in 1 .. Pushes loop
            for B in 1 .. 3 loop
               Step (By);
            end loop;
            for B in 1 .. 3 loop
               Step (0.0);
            end loop;
         end loop;
         Estimate_Now (M);
      end Run;

      Carried, Patched, Weak : Model;
   begin
      Run (Carried, Faint_Moves => True, By => Push);
      declare
         F : constant Eye_Effect := Driver.Robot.Graph.Effect (Carried, 1, 1);
      begin
         Check (F.Responding * 2 < F.Textured,
                "fewer than half of the cells tell the push, or the picture does not show the weakness:"
                & F.Responding'Image & " of" & F.Textured'Image);
         Check (F.Verdict = Whole, "an eye that moves whole, its faint cells too noisy to tell, is" & F.Verdict'Image
                & " with" & F.Responding'Image & " of" & F.Textured'Image & " cells responding");
      end;
      Run (Patched, Faint_Moves => False, By => Push);
      declare
         F : constant Eye_Effect := Driver.Robot.Graph.Effect (Patched, 1, 1);
      begin
         Check (F.Verdict = Patch, "a picture whose well-textured quarter alone moves is" & F.Verdict'Image
                & " with" & F.Responding'Image & " of" & F.Textured'Image & " cells responding");
      end;
      Run (Weak, Faint_Moves => True, By => Weaker);
      declare
         F : constant Eye_Effect := Driver.Robot.Graph.Effect (Weak, 1, 1);
      begin
         Check (F.Verdict = Undecided,
                "an eye that moves whole under pushes too small for the faint cells to show it together, which the "
                & "well-textured quarter cannot tell from a patch, is" & F.Verdict'Image & " with" & F.Responding'Image
                & " of" & F.Textured'Image & " cells responding");
      end;
   end Carried_Eye_Partly_Textureless;

   --  Cells too noisy to tell a motion show it together, by the mean of their energies over the typical energy. A
   --  mean is moved as far as one cell likes: the fingers of a hand in view are a few cells that show 10 ** 4 times
   --  what the rest of the picture does (A29's closer, at 8136 beats: 37 of 371 noisy cells carried 99.8 % of their
   --  energy, the mean was 560 times the typical, every noisy cell was credited with showing the motion, and the
   --  hand's closer was called an arm). The cells far above are the ones that show it, and when they carry the
   --  energy the rest is read without them; a whole picture's nearer parts, a few cells that show several times the
   --  typical energy, do not take the rest of the picture's share from it.
   procedure Noisy_Cells_Far_Above_The_Rest is
      use Driver.Robot.Lockin;
      G       : Generator;
      Cells   : constant := 400;
      Typical : constant Real := 1.0;
      Var     : constant Real := 0.5;   --  with one degree of freedom: a spread of 1.6 typical energies a cell

      --  Noisy cells: a share of them show the typical energy, a few show Far times it and respond.
      function Pool_Of (Showing : Real; Far_Cells : Natural; Far : Real) return Noisy_Cell_Array is
         Spread : constant Real := Sqrt (2.0 * Var ** 2 + 4.0 * Var * Typical);
         Result : Noisy_Cell_Array (1 .. Cells);
      begin
         for K in Result'Range loop
            Result (K) :=
              (Energy   => (if K <= Far_Cells then Far * Typical else (if Uniform (G) < Showing then Typical else 0.0))
                           + Spread * Gaussian (G),
               Variance => Var, Freedom => 1, Responds => K <= Far_Cells);
         end loop;
         return Result;
      end Pool_Of;

      Hands   : constant Noisy_Cell_Array := Pool_Of (0.0, 40, 1.0E4);
      All_Show : constant Noisy_Cell_Array := Pool_Of (1.0, 4, 8.0);
      None_Show : constant Noisy_Cell_Array := Pool_Of (0.0, 0, 0.0);
      Heavy   : constant Noisy_Cell_Array := Pool_Of (1.0, 4, 100.0);
      By_Hands, By_Agreed, By_All, By_None, By_Heavy : Shown;
   begin
      --  The cells that can tell do not show the motion between them, as the hand's are not the whole picture.
      By_Hands := Cells_Shown (Hands, Typical, Able_Disagree => True);
      --  They do, and a few cells that show a great deal are a whole picture's nearer parts.
      By_Agreed := Cells_Shown (Hands, Typical, Able_Disagree => False);
      By_All := Cells_Shown (All_Show, Typical, Able_Disagree => True);
      By_None := Cells_Shown (None_Show, Typical, Able_Disagree => True);
      By_Heavy := Cells_Shown (Heavy, Typical, Able_Disagree => True);
      Check (By_Hands.Least >= 40, "the cells far above the rest are not counted as showing the motion:"
             & By_Hands.Least'Image);
      Check (By_Hands.Most * 2 < Cells, "forty cells that show ten thousand times the typical energy make the other"
             & " 360, that show nothing, show it: up to" & By_Hands.Most'Image & " of" & Cells'Image);
      Check (By_Agreed.Least * 2 > Cells, "the cells that can tell agree that the picture moves whole, and the cells"
             & " far above the rest are read apart from it: up to" & By_Agreed.Least'Image & " of" & Cells'Image);
      Check (By_None.Most * 2 < Cells, "cells that show nothing show it:" & By_None.Most'Image & " of"
             & Cells'Image);
      Check (By_All.Least * 2 > Cells, "cells that all show the typical energy, four of them eight times as much,"
             & " are not credited with showing it:" & By_All.Least'Image & " of" & Cells'Image);
      Check (By_Heavy.Least * 2 > Cells, "four cells at a hundred times the typical energy, among cells that all show"
             & " it, take the others' share from them:" & By_Heavy.Least'Image & " of" & Cells'Image);
      --  A29's closer at 8136 beats: 75 of its 154 cells that can tell showed the motion; A29's second arm in the first
      --  arm's eye: 41 of 64; the arms in their own eyes, at their first recomputes: 122 of 134, 103 of 108 (A31).
      Check (Cells_Disagree (75, 79), "75 of 154 cells that can tell show a motion, and they agree that it is whole");
      Check (Cells_Disagree (41, 23), "41 of 64 cells that can tell show a motion, and they agree that it is whole");
      Check (not Cells_Disagree (122, 12), "122 of 134 cells that can tell show a motion, and they disagree");
      Check (not Cells_Disagree (103, 5), "103 of 108 cells that can tell show a motion, and they disagree");
      Check (not Cells_Disagree (5, 0) and then not Cells_Disagree (0, 0),
             "five cells that can tell, all showing it, or none, are enough to disagree");
      Check (Judge (168, 525, 446, 446, Able_Disagree => True) /= Whole,
             "a picture that most of the cells that can tell do not show is called whole by the others");
      Check (Judge (168, 525, 446, 446, Able_Disagree => False) = Whole,
             "a picture that most of the cells that can tell show, and most of the rest are credited, is not whole");
      Check (Judge (168, 525, 168, 202, Able_Disagree => True) = Patch,
             "a minority of cells showing the motion, the rest not (a hand's fingers), is not called a patch");
      Check (Judge (3, 525, 3, 14, Able_Disagree => False) = Nothing,
             "three cells of 525 respond to a push, which the per-cell test alarms on by chance: the group moves it");
   end Noisy_Cells_Far_Above_The_Rest;

   --  A group only ever pushed together with another is not classified:
   --  what the eyes saw cannot be told from what its partner did.
   procedure Unprobed_Group_Stays_Unclassified is
      M    : Model;
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
   begin
      for B in 1 .. 5 loop
         Step (M, R, Rest);
      end loop;
      Exercise (M, R, Arm_1, 0.1, 8);
      Exercise (M, R, Arm_2, 0.1, 8);
      Exercise (M, R, Closer_With_Arm_1, 0.1, 8);
      Estimate_Now (M);
      Check (Role (M, 1) = Arm, "arm 1 is an arm, got " & Role (M, 1)'Image);
      --  What a lock-in could credit the closer with while it only ever moved
      --  beside arm 1 (as a live boot's first shared probe did): a patch in
      --  arm 1's eye. Its role must still wait for its own pushes.
      M.Graph.Effects.Replace_Element
        ((3 - 1) * Eye_Count (M) + 1,
         (Verdict => Patch, Responding => 8, Textured => 48,
          Fraction => (Value => 8.0 / 48.0, Sigma => 0.05, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
      Driver.Robot.Graph.Derive (M);
      Check (Role (M, 3) = Unclassified, "the closer pushed only with arm 1 is " & Role (M, 3)'Image);
      Check (Closer_Arm (M, 3) = 0, "the closer pushed only with arm 1 is given arm" & Closer_Arm (M, 3)'Image);
   end Unprobed_Group_Stays_Unclassified;

   --  An arm held away from rest whose reading jitters far more than it did
   --  at rest (here: exact at rest, flipping by a quarter of its visible step
   --  when held) still gives
   --  its kinematics a keyframe there once its eye is still.
   procedure Keyframe_Despite_Held_Jitter is
      M    : Model;
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
      Away : Rig_State := Rest;
   begin
      for B in 1 .. 5 loop
         Step (M, R, Rest);
      end loop;
      Exercise (M, R, Arm_1, 0.1, 8);
      Exercise (M, R, Arm_2, 0.1, 8);
      Exercise (M, R, Closer, 0.1, 8);
      Estimate_Now (M);
      for B in 1 .. 4 loop
         Step (M, R, Rest);
      end loop;
      Away.Arm_1 := [0.3, 0.15];
      Check (Known (Visible_Step (M, 1, 1)), "the arm's visible step is measured");
      for B in 1 .. 12 loop
         declare
            Held : Rig_State := Away;
            --  A quarter of what the eye can see: invisible, yet far beyond the
            --  exact readings' noise at rest.
            Jitter : constant Real := (if Known (Visible_Step (M, 1, 1)) then Visible_Step (M, 1, 1).Value / 4.0 else 1.0e-4);
         begin
            Held.Arm_1 (1) := Away.Arm_1 (1) + (if B mod 2 = 0 then Jitter else -Jitter);
            Step (M, R, Away, Held);
         end;
      end loop;
      Check (Eye_Mount (M, 1) = (Kind => Arm_Carried, Arm => 1), "eye 1 rides on arm 1");
      Check (Driver.Robot.Kinematics.Held_Still (M, 1, M.Beats - 1),
             "the arm held still with an invisible jitter is not taken as still for a keyframe");
   end Keyframe_Despite_Held_Jitter;

   --  No keyframe is taken in a step under way (Model.Stepping): an arm whose readings and picture are still, away from
   --  every keyframe it has, is a fresh keyframe, and is one once the step is over. In the first beats of a push the
   --  readings are below the step an eye can see (the sweep's first level is that step) and the picture a lag behind
   --  them: the keyframe held there paired a reading 62 % of the way with a picture at rest (the cold boot b2, 10-20:
   --  4 of 76 keyframes of the base arm, 11 of 49 of seed 21's arm 1, 41 of 164 of the humanoid's).
   procedure No_Keyframe_In_A_Step is
      M    : Model;
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
      Away : Rig_State := Rest;
      Before, During, After : Natural := 0;
      function Frames return Natural is
         N : Natural := 0;
      begin
         for E of M.Kinematics loop
            if E.Group = 1 then
               N := Natural (E.Frames.Length);
            end if;
         end loop;
         return N;
      end Frames;
   begin
      Exercise_Rig (M);
      for B in 1 .. 12 loop
         Step (M, R, Rest);
      end loop;
      Away.Arm_1 := [0.9, 0.7];
      Before := Frames;
      M.Stepping := True;
      for B in 1 .. 14 loop
         Step (M, R, Away);
      end loop;
      During := Frames;
      M.Stepping := False;
      for B in 1 .. 14 loop
         Step (M, R, Away);
      end loop;
      After := Frames;
      Check (Before > 0, "the rig's arm has keyframes before the step");
      Check (During = Before, "a keyframe was taken in a step under way:" & Before'Image & " frames, then" & During'Image);
      Check (After > During, "no keyframe was taken once the step was over:" & During'Image & " frames, then" & After'Image);
   end No_Keyframe_In_A_Step;

   --  An arm at rest takes its reference keyframe and then a still twin of
   --  it at the same pose (the sweep waits for it). The aligner's own error
   --  is not known from two identical pictures, so the sweep starts where the
   --  cells tell a move, and where the aligner can once keyframes the arm moved
   --  to are aligned: Z times the larger of the two over the view's shift.
   procedure Twin_Before_The_Sweep is
      M    : Model;
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
   begin
      Exercise_Rig (M);
      --  Back to the pose the reference was taken at, and held there, as the boot holds an arm for its still twin
      --  (Motion.Hold_For_Twin): the twin is the first keyframe at the reference's readings, whenever the arm stands
      --  there again, and a keyframe is held still over the group's response time (item 21), one beat longer than the
      --  rig's holds between its pushes.
      declare
         Back : Rig_State := Rest;
      begin
         for E of M.Kinematics loop
            if E.Group = 1 and then not E.Frames.Is_Empty then
               Back.Arm_1 := [E.Frames (E.Frames.First_Index).Readings (0), E.Frames (E.Frames.First_Index).Readings (1)];
            end if;
         end loop;
         for B in 1 .. 12 loop
            Step (M, R, Back);
         end loop;
      end;
      Check (Eye_Mount (M, 1) = (Kind => Arm_Carried, Arm => 1), "eye 1 rides on arm 1");
      declare
         Twins : Natural := 0;   --  keyframes after the reference at its pose, within the visible steps
      begin
         for E of M.Kinematics loop
            if E.Group = 1 then
               for K in E.Frames.First_Index + 1 .. E.Frames.Last_Index loop
                  if (for all C in 1 .. 2 =>
                        Known (Visible_Step (M, 1, C))
                        and then abs (E.Frames (K).Readings (C - 1) - E.Frames (E.Frames.First_Index).Readings (C - 1))
                                 < Visible_Step (M, 1, C).Value)
                  then
                     Twins := Twins + 1;
                  end if;
               end loop;
            end if;
         end loop;
         declare
            Frames_Text : Ada.Strings.Unbounded.Unbounded_String;
         begin
            for E of M.Kinematics loop
               if E.Group = 1 then
                  for K in E.Frames.First_Index .. E.Frames.Last_Index loop
                     Ada.Strings.Unbounded.Append
                       (Frames_Text, " [beat" & E.Frames (K).Beat'Image & ":"
                        & Real'Image (E.Frames (K).Readings (0) - E.Frames (E.Frames.First_Index).Readings (0)) & ","
                        & Real'Image (E.Frames (K).Readings (1) - E.Frames (E.Frames.First_Index).Readings (1)) & "]");
                  end loop;
               end if;
            end loop;
            Check (Twins = 1, "arm 1 at rest has" & Twins'Image & " still twins of its reference, not one; its frames:"
                   & Ada.Strings.Unbounded.To_String (Frames_Text) & "; the visible steps "
                   & Real'Image (Visible_Step (M, 1, 1).Value) & Real'Image (Visible_Step (M, 1, 2).Value));
         end;
      end;
      Check (Driver.Robot.Kinematics.Twin_Answered (M, 1), "the arm's still twin was not taken");
      declare
         Noise : constant Real :=
           Real'Max (Lockin.Cell_Noise (M, 1), Driver.Robot.Kinematics.Match_Noise (M, 1));
      begin
         for C in 1 .. 2 loop
            declare
               Start  : constant Real := Driver.Robot.Motion.Sweep_Start (M, 1, C);
               Shift  : constant Real := Driver.Robot.Kinematics.Pixels_Per_Unit (M, 1, C);
               Jitter : constant Real := Driver.Conventions.Z * Sqrt (2.0) * Driver.Robot.Channels.Noise (M, 1, C);
            begin
               Check (Shift > 0.0, "arm 1's view's motion per unit of joint" & C'Image
                      & " is not measured by the keyframes before the sweep");
               if Shift > 0.0 then
                  --  The larger of half the aligner's patch of view and the least turn a keyframe counts: the step
                  --  the matcher tells apart, no nearer than the readings tell apart, plus what the readings' noise
                  --  can take off a turn asked of exactly it (Motion.Sweep_Start, Kinematics.Step_Of).
                  declare
                     Least  : constant Real := Real'Max (Driver.Conventions.Z * Noise / Shift, Jitter) + Jitter;
                     Follow : constant Real := Real (Driver.Alignment.Half_Patch (Rig_Width, Rig_Height)) / Shift;
                  begin
                     Check_Close (Start, Real'Max (Least, Follow), 1.0e-9 * Real'Max (Least, Follow),
                                  "the first level of joint" & C'Image
                                  & " is not half a patch of view, nor the least turn a keyframe counts");
                  end;
               end if;
            end;
         end loop;
      end;
   end Twin_Before_The_Sweep;

   --  A picture that keeps changing after the body stopped: the push moves
   --  the view by 2 pixels, and from then on a flicker on a tenth of the
   --  pixels, flipping sign every beat, decays from 40 luma levels by 30 % a
   --  beat to a lasting 4, far above the still frames' noise before the push.
   --  The picture has stopped once the flicker stops shrinking, though it
   --  never comes back to the noise it had at rest.
   --  The run of that picture, with a sixteen-pixel square in its corner that
   --  flickers by 30 luma levels every beat from the first, at rest as after
   --  the push, when Patch: the beat the body settled, and the last beat the
   --  push's flicker shrank by more than a hundredth.
   procedure Settle_Run (Patch : Boolean; M : in out Model; Settled_At, Decaying_Until : out Natural) is
      Width  : constant := 64;
      Height : constant := 48;
      Reading, Target : Real := 0.0;

      function Frame (Beat : Natural; Shift, Flicker : Real) return Driver.Images.Image is
         use type Driver.Bytes.Offset;
         Data : Driver.Bytes.Byte_Array (1 .. 3 * Width * Height);
      begin
         for Y in 0 .. Height - 1 loop
            for X in 0 .. Width - 1 loop
               declare
                  L : Real := Texture (Real (X) + Shift, Real (Y));
                  K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Width + X) + 1);
               begin
                  if Patch and then X < 16 and then Y < 16 then
                     L := 128.0 + (if Beat mod 2 = 0 then 30.0 else -30.0);
                  elsif (X * 7 + Y * 13) mod 10 = 0 then
                     L := L + (if (Beat + X) mod 2 = 0 then Flicker else -Flicker);
                  end if;
                  Data (K) := Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                  Data (K + 1) := Data (K);
                  Data (K + 2) := Data (K);
               end;
            end loop;
         end loop;
         return Driver.Images.Create (Width, Height, Data);
      end Frame;
   begin
      Settled_At := 0;
      Decaying_Until := 0;
      for B in 0 .. 199 loop
         declare
            O       : Observation;
            Sent    : Driver.Commands.Command;
            Flicker : Real := 0.0;
         begin
            if B = 100 then
               Target := 1.0;
            end if;
            if B > 100 then
               Reading := Target;
               Flicker := 4.0 + 36.0 * 0.7 ** (B - 101);
               if 36.0 * 0.7 ** (B - 101) * 0.3 > 0.01 * Flicker then
                  Decaying_Until := B;
               end if;
            end if;
            O.Beat := Driver.Clock.Beat (B);
            O.Images.Append (Frame (B, (if B > 100 then 2.0 else 0.0), Flicker));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            O.Readings.Append (Real_Array'(1 => Reading));
            Driver.Commands.Set_Target (Sent, 1, [Target]);
            Observe (M, O, Sent);
            if B > 101 and then Settled_At = 0 and then Stillness.All_Still (M) then
               Settled_At := B;
            end if;
         end;
      end loop;
   end Settle_Run;

   procedure Settle_After_A_Slow_Tail is
      M : Model;
      Settled_At, Decaying_Until : Natural;
   begin
      Settle_Run (False, M, Settled_At, Decaying_Until);
      Check (not M.Eyes (1).Is_Still, "the lasting flicker is still to the rest-noise test: the test shows nothing");
      Check (Settled_At > 0, "the body never settled while its eye's picture flickered at a lasting level");
      Check (Settled_At = 0 or else Settled_At > Decaying_Until,
             "the body settled at beat" & Settled_At'Image & " while the flicker still shrank, until" & Decaying_Until'Image);
   end Settle_After_A_Slow_Tail;

   --  The same picture with a patch that flickers at rest, far more than the
   --  push's own tail ever changes the picture: the body settles at the beat
   --  it settles without the patch, to the beat: the plain rig is noiseless,
   --  so the patch is the only thing the change wanders by at a lasting
   --  level, and the watch ends the tail where its step falls under Z of that
   --  wander, one beat sooner than where the noiseless tail stops changing.
   procedure Settle_Past_A_Patch_Flickering_At_Rest is
      Plain, Patched : Model;
      Plain_At, Patched_At, Decaying_Until : Natural;
   begin
      Settle_Run (False, Plain, Plain_At, Decaying_Until);
      Settle_Run (True, Patched, Patched_At, Decaying_Until);
      Check (Plain_At > 0 and then abs (Patched_At - Plain_At) <= 1,
             "with a patch flickering at rest the body settled at beat" & Patched_At'Image & ", without it at"
             & Plain_At'Image);
   end Settle_Past_A_Patch_Flickering_At_Rest;

   --  The sweep starts each joint where its eye's view moves by what one cell
   --  of it tells and the aligner can tell: Z times the larger of the cells'
   --  displacement noise and the aligner's scatter on the keyframes the arm has
   --  moved to. The visible step, which a matched filter over every cell sees,
   --  moves the view by far less than any one point of a match can be told.
   procedure Sweep_Starts_Where_A_Cell_Tells is
      M : Model;
   begin
      Exercise_Rig (M);
      for C in 1 .. 2 loop
         declare
            Start  : constant Real := Driver.Robot.Motion.Sweep_Start (M, 1, C);
            Shift  : constant Real := Driver.Robot.Kinematics.Pixels_Per_Unit (M, 1, C);
            Jitter : constant Real := Driver.Conventions.Z * Sqrt (2.0) * Driver.Robot.Channels.Noise (M, 1, C);
            Noise  : constant Real :=
              Real'Max (Lockin.Cell_Noise (M, 1), Driver.Robot.Kinematics.Match_Noise (M, 1));
         begin
            Check (Start > 0.0 and then Shift > 0.0 and then Noise < Real'Last, "arm 1's sweep plan is not measured");
            if Start > 0.0 and then Shift > 0.0 and then Noise < Real'Last then
               --  The larger of: half the aligner's patch of view (the largest turn it follows by itself), and the
               --  step the matcher tells apart, no nearer than the readings tell apart, plus what the readings'
               --  noise can take off a turn asked of exactly it (Motion.Sweep_Start: a turn of the step itself
               --  lands below it half the time).
               declare
                  Least  : constant Real := Real'Max (Driver.Conventions.Z * Noise / Shift, Jitter) + Jitter;
                  Follow : constant Real := Real (Driver.Alignment.Half_Patch (Rig_Width, Rig_Height)) / Shift;
               begin
                  Check_Close (Start, Real'Max (Least, Follow), 1.0e-9 * Real'Max (Least, Follow),
                               "the first level of joint" & C'Image & " is not half a patch of view (or the least turn)");
                  Check (Follow > Least, "the rig's half patch" & Driver.Log.Image (Follow, 6)
                         & " is not above the least turn" & Driver.Log.Image (Least, 6) & ": the rule is not exercised");
               end;
               Check (Known (Visible_Step (M, 1, C)) and then Start > Visible_Step (M, 1, C).Value,
                      "joint" & C'Image & "'s sweep starts at its visible step or below");
            end if;
         end;
      end loop;
   end Sweep_Starts_Where_A_Cell_Tells;

   --  The joints next to each other along an arm whose swap the fit cannot tell (Fit_Report's Untold). Two joints
   --  never turned together are exactly that: one of them at its reference in every keyframe, the two orders give
   --  every keyframe the same pose (they differ in the product of the two turns alone). A fit of every joint alone
   --  and of every pair together but 2 with 3 names 2 with 3 and no other pair; with 2 and 3 turned together too,
   --  the order is told and it names none. The pairs were taken from the order nearest the best's in residuals
   --  before: s08's was the true order, two swaps away, s10's another, three pairs off the true ones (10-16).
   procedure Untold_Neighbours is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Levels  : constant Real_Array := [0.05, -0.05, 0.1, -0.1, 0.2, -0.2];
      Widest  : constant Real := 0.2;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant Real := 0.2;
      Table   : constant Vec3 := [0.0, -0.6, -0.8];   --  its normal, towards the eye
      Lens    : constant Fit.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Truth   : Fit.Joint_Array (1 .. N);
      type Sighting_Access is access Fit.Sighting_Array;

      --  Each pair turned together either way, as the boot's rows of a Sylvester-Hadamard matrix turn it: the order
      --  search tells an order from another only where both halves of the keyframes do (Judge), and one turn of a
      --  pair lies in one half alone.
      procedure Fit_Of (Both : Boolean; Report : out Fit.Fit_Report) is
         Pairs   : constant Natural := 2 * (N * (N - 1) / 2 - (if Both then 0 else 1));
         Frames  : constant Positive := 1 + N * Levels'Length + Pairs;
         Changes : Real_Matrix (1 .. Frames, 1 .. N) := [others => [others => 0.0]];
         Asked   : Fit.Flag_Matrix (1 .. Frames, 1 .. N);
         All_Seen : constant Sighting_Access := new Fit.Sighting_Array (1 .. (Frames - 1) * Columns * Rows);
         Seen    : Natural := 0;
         Row     : Positive := 1 + N * Levels'Length;
         Rng     : Generator;
         Joints  : Fit.Joint_Array (1 .. N);
         Chain   : Fit.Chain_Order (1 .. N);
         Found   : Fit.Lens;
      begin
         for J in 1 .. N loop
            for L in Levels'Range loop
               Changes (1 + (J - 1) * Levels'Length + (L - Levels'First + 1), J) := Levels (L);
            end loop;
         end loop;
         for I in 1 .. N loop
            for J in I + 1 .. N loop
               if Both or else not (I = 2 and then J = 3) then
                  for Way of Real_Array'[1.0, -1.0] loop
                     Row := Row + 1;
                     Changes (Row, I) := Way * Widest;
                     Changes (Row, J) := Way * Widest;
                  end loop;
               end if;
            end loop;
         end loop;
         for F in 1 .. Frames loop
            for J in 1 .. N loop
               Asked (F, J) := Changes (F, J) /= 0.0;
            end loop;
         end loop;
         for F in 2 .. Frames loop
            declare
               D : Real_Array (1 .. N);
               T : Rigid;
            begin
               for J in 1 .. N loop
                  D (J) := Changes (F, J);
               end loop;
               T := Inverse (Fit.Eye_At (Truth, Along (Truth), D));
               for Gy in 1 .. Rows loop
                  for Gx in 1 .. Columns loop
                     declare
                        K  : constant Positive := (Gy - 1) * Columns + Gx;
                        U0 : constant Real := (Real (Gx) - 0.5) * 640.0 / Real (Columns);
                        V0 : constant Real := (Real (Gy) - 0.5) * 480.0 / Real (Rows);
                        H  : constant Vec3 := Fit.Ray (Lens, U0, V0);
                        X  : constant Vec3 := (-1.0 / Real'(Unit (Table) * H)) * H;
                        U, V : Real;
                        Ahead : Boolean;
                     begin
                        Fit.Project (Lens, T * X, U, V, Ahead);
                        U := U + Noise * Gaussian (Rng);
                        V := V + Noise * Gaussian (Rng);
                        if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 then
                           Seen := Seen + 1;
                           All_Seen (Seen) := (Frame => F, Track => K, U0 => U0, V0 => V0, U => U, V => V, Own => 0.0,
                                               Own_Freedom => 0, others => <>);
                        end if;
                     end;
                  end loop;
               end loop;
            end;
         end loop;
         Fit.Fit (Changes, Asked, [for J in 1 .. N => 0.01], All_Seen (1 .. Seen), 640, 480, 0, Joints, Chain, Found,
                  Report);
      end Fit_Of;
   begin
      declare
         --  Synthetic_Sweep's arm: no two axes alike, none through another's point.
         Axes   : constant array (1 .. N) of Vec3 :=
           [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
         Points : constant array (1 .. N) of Vec3 :=
           [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      begin
         for J in 1 .. N loop
            declare
               W : constant Vec3 := Unit (Axes (J));
            begin
               Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
            end;
         end loop;
      end;
      declare
         Report : Fit.Fit_Report;
      begin
         Fit_Of (Both => False, Report => Report);
         Check (Report.Fitted, "the arm turned a joint and a pair at a time, but 2 with 3, was not fitted: "
                & Ada.Strings.Unbounded.To_String (Report.Why));
         Check (not Report.Order_Told, "the order is told where 2 and 3 never turned together");
         Check (Natural (Report.Untold.Length) = 1
                and then (for some P of Report.Untold => (P.Earlier = 2 and then P.Later = 3)
                                                         or else (P.Earlier = 3 and then P.Later = 2)),
                "the pairs whose swap is not told are" & Report.Untold.Length'Image & ", not 2 with 3 alone");
         --  Named by the sizes of its turns (Boot turns the pair every way at them).
         Check (Natural (Report.Untold_Turns.Length) = 2 * Natural (Report.Untold.Length)
                and then (for all T of Report.Untold_Turns => T > 0.0),
                "the pair's turns are named" & Report.Untold_Turns.Length'Image & " for" & Report.Untold.Length'Image
                & " pairs, or not by their sizes");
         Fit_Of (Both => True, Report => Report);
         Check (Report.Fitted and then Report.Order_Told and then Report.Untold.Is_Empty,
                "every pair turned together leaves the order untold, or names a pair:" & Report.Untold.Length'Image);
      end;
   end Untold_Neighbours;

   procedure Roles_Of_A_Synthetic_Body is
      M : Model;
   begin
      Exercise_Rig (M);
      Check (Role (M, 1) = Arm, "arm 1 is an arm, got " & Role (M, 1)'Image);
      Check (Role (M, 2) = Arm, "arm 2 is an arm, got " & Role (M, 2)'Image);
      --  Each arm moves nothing in the other's eye, though they were also
      --  pushed together and one overshot while the other was still on its way.
      Check (Response (M, 1, 2) = Nothing, "arm 1 moves nothing in arm 2's eye, got " & Response (M, 1, 2)'Image);
      Check (Response (M, 2, 1) = Nothing, "arm 2 moves nothing in arm 1's eye, got " & Response (M, 2, 1)'Image);
      --  An eye sees steps of an arm far smaller than the pushes it was shown,
      --  and nothing of a group that moves nothing.
      Check (Known (Visible_Step (M, 1, 1)) and then Visible_Step (M, 1, 1).Value < 0.1,
             "arm 1 has a visible step below its pushes of 0.1");
      Check (Known (Visible_Step (M, 3, 1)) and then Visible_Step (M, 3, 1).Value < 0.1,
             "the closer has a visible step below its pushes of 0.1");
      Check (not Known (Visible_Step (M, 5, 1)), "a group that moves nothing has no visible step");
      Check (Role (M, 3) = Closer, "the finger group is a closer, got " & Role (M, 3)'Image);
      Check (Closer_Arm (M, 3) = 1, "the closer belongs to arm 1, got" & Closer_Arm (M, 3)'Image);
      Check (Role (M, 4) = Part, "the part is a part, got " & Role (M, 4)'Image);
      Check (Role (M, 5) = Inert, "the idle group is inert, got " & Role (M, 5)'Image);
      Check (Role (M, 6) = Sensor, "the arms' sum is a sensor, got " & Role (M, 6)'Image);
      Check (Role (M, 7) = Inert, "a constant is inert, got " & Role (M, 7)'Image);
      Check (Eye_Mount (M, 1) = (Kind => Arm_Carried, Arm => 1), "eye 1 rides on arm 1");
      Check (Eye_Mount (M, 2) = (Kind => Arm_Carried, Arm => 2), "eye 2 rides on arm 2");
      Check (Eye_Mount (M, 3).Kind = World_Fixed, "eye 3 is fixed");
      Check (Response (M, 2, 1) /= Whole, "arm 2 does not move eye 1 whole");
      Check (Response (M, 1, 2) /= Whole, "arm 1 does not move eye 2 whole");
      for E in 1 .. 3 loop
         Check (Image_Lag (M, Eye_Id (E)) = 1, "eye" & E'Image & " lags one beat, got" & Image_Lag (M, Eye_Id (E))'Image);
      end loop;
      Check (Carrier_Group (M) = 0, "no group carries every eye");
   end Roles_Of_A_Synthetic_Body;

   --  A reading that crawls for the first 25 beats (a servo on its way to a stale goal: R0, 10-11, 660 ticks in 25
   --  beats) and then repeats exactly, never pushed: its rest noise is zero, measured on the trend-free rest, and the
   --  crawl is motion.
   procedure Channel_Noise_Of_A_Crawl is
      M    : Model;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Creep_Beats : constant := 25;
   begin
      for B in 0 .. 79 loop
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => 1.0 + (if B < Creep_Beats then 0.5 * Real (Creep_Beats - B) / Real (Creep_Beats) else 0.0)));
         Sent := Driver.Commands.Hold;
         Observe (M, O, Sent);
         if B in 2 .. 60 and then B mod 20 = 0 then
            Estimate_Now (M);
         end if;
      end loop;
      Estimate_Now (M);
      Check (Reading_Noise (M, 1, 1) = 0.0,
             "the crawling reading's rest noise is " & Real'Image (Reading_Noise (M, 1, 1))
             & ", not 0: its crawl was taken for jitter");
      Check (Channels.Moving (M, 1, 10), "the crawl at beat 10 is not motion");
      Check (not Channels.Moving (M, 1, 60), "the exact repeats at beat 60 are motion");
   end Channel_Noise_Of_A_Crawl;

   --  One integer observation of the given groups' readings, each group one array, every group's wire an integer.
   procedure Observe_Integers (M : in out Model; B : Natural; Values : Real_Array; Sizes : Natural_Array) is
      O    : Observation;
      Sent : constant Driver.Commands.Command := Driver.Commands.Hold;
      At_V : Natural := Values'First;
   begin
      O.Beat := Driver.Clock.Beat (B);
      O.Images.Append (Driver.Images.No_Image);
      O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      for S of Sizes loop
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Values (At_V .. At_V + S - 1));
         O.Precisions.Append (0);
         At_V := At_V + S;
      end loop;
      Observe (M, O, Sent);
   end Observe_Integers;

   --  A reading that steps by four units (a joint whose ticks the body reports as four units each) and flips one
   --  step at rest, to the next value and back, has the rounding's noise of its own step: a change of two steps in
   --  a beat is motion, one step is not (w11, servo seed 11 of the fake robot, 10-19: 24 such flips set arm 1's
   --  fourth channel's noise at 4.19 units, and a change had to pass 17.8 units a beat to be motion).
   procedure Noise_Of_Single_Step_Flips is
      M    : Model;
      Step : constant Real := 4.0;
      function Value (B : Natural) return Real is
        (2000.0 + (if B >= 110 then 2.0 * Step * Real (B - 109) elsif B mod 20 in 5 .. 9 then Step else 0.0));
   begin
      for B in 0 .. 119 loop
         Observe_Integers (M, B, [1 => Value (B)], [1 => 1]);
         if B in 2 .. 100 and then B mod 20 = 0 then
            Estimate_Now (M);
         end if;
      end loop;
      Check (Reading_Noise (M, 1, 1) = Step / Sqrt (12.0),
             "a reading flipping one step of" & Step'Image & " at rest has noise" & Real'Image (Reading_Noise (M, 1, 1))
             & ", not its step's rounding" & Real'Image (Step / Sqrt (12.0)));
      Check (not Channels.Moving (M, 1, 25) and then not Channels.Moving (M, 1, 30), "a flip of one step is motion");
      Check (Channels.Moving (M, 1, 112), "a change of two steps in a beat is no motion");
   end Noise_Of_Single_Step_Flips;

   --  A group whose every channel's noise rests on few changes (one degree of freedom each) still tells a change
   --  well beyond them: the noise is taken as measured, not widened by the fewest degrees of freedom (w12, servo seed
   --  11 of the fake robot, 10-19: its estimate of beat 5940 found arm 1 pushed at 766 beats in one round and at
   --  none in the next, every channel's noise then on 1 to 4 degrees of freedom).
   procedure Change_Against_Noise_Of_Few_Changes is
      M : Model;
      --  Both flip a unit once (their step: Channels.Step_Unit), then two units out and back every ten beats, as evenly
      --  as no half of the rest trends; the second then moves 20 units in a beat.
      function First (B : Natural) return Real is
        (2000.0 + (if B = 2 then 1.0 elsif B < 60 and then B mod 10 in 5 .. 7 then 2.0 else 0.0));
      function Second (B : Natural) return Real is (First (B) + (if B >= 65 then 20.0 else 0.0));
   begin
      for B in 0 .. 70 loop
         Observe_Integers (M, B, [First (B), Second (B)], [1 => 2]);
         if B = 60 then
            Estimate_Now (M);
         end if;
      end loop;
      Check (Channels.Noise_Freedom (M, 1, 1) > 0 and then Channels.Noise_Freedom (M, 1, 2) > 0,
             "the rig's noises rest on" & Channels.Noise_Freedom (M, 1, 1)'Image & " and"
             & Channels.Noise_Freedom (M, 1, 2)'Image & " degrees of freedom, not on few changes");
      Check (Channels.Moving (M, 1, 65),
             "a change of 20 units against noises of" & Real'Image (Reading_Noise (M, 1, 1)) & " and"
             & Real'Image (Reading_Noise (M, 1, 2)) & " is no motion");
      Check (not Channels.Moving (M, 1, 15), "a change of the channels' own jitter is motion");
   end Change_Against_Noise_Of_Few_Changes;

   --  Readings that come as integers (a servo's ticks: the SO-101 pair, R0) resolve to one unit, so the smallest push
   --  of such a joint without jitter is one unit; a rig's Reals resolve to the unit in the last place of the group's
   --  largest reading.
   procedure Resolution_Of_Integer_Readings is
      M    : Model;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Last : constant Natural := 9;
   begin
      for B in 0 .. Last loop
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         for G in 1 .. 2 loop
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         O.Readings.Append (Real_Array'(1 => 2048.0, 2 => 10.0));
         O.Readings.Append (Real_Array'(1 => 2048.0));
         O.Precisions.Append (0);
         O.Precisions.Append (Real'Machine_Mantissa);
         Sent := Driver.Commands.Hold;
         Observe (M, O, Sent);
      end loop;
      Estimate_Now (M);
      Check (Channels.Resolution (M, 1, Last) = 1.0,
             "integer readings resolve to one unit, got " & Real'Image (Channels.Resolution (M, 1, Last)));
      Check (Channels.Resolution (M, 2, Last) = Real'Scaling (1.0, Real'Exponent (2048.0) - Real'Machine_Mantissa)
             and then Channels.Resolution (M, 2, Last) < 1.0e-9,
             "a Real of 2048 resolves to its unit in the last place, got " & Real'Image (Channels.Resolution (M, 2, Last)));
   end Resolution_Of_Integer_Readings;

   procedure Channel_Noise_And_Pushes is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      --  Group 1 reads with noise of sigma 0.01 and is pushed by 1 at beat 20,
      --  answered from beat 21. Group 2 echoes its target exactly but no
      --  further than 1 (a clipped echo): its push to 0.5 at beat 10 is
      --  answered at once, its push to 1.5 at beat 40 never. Group 3 is a
      --  simulator's joint at rest: two beats in three it repeats exactly, the
      --  third it moves by a jitter of sigma 1e-5; it is pushed by 0.1 at beat
      --  30, answered from beat 31. One camera.
      Jitter : constant Real := 1.0e-5;
      Third  : Real := 0.0;
   begin
      for B in 0 .. 59 loop
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         for G in 1 .. 3 loop
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         if B mod 3 = 0 then
            Third := Third + Jitter * Gaussian (Rng);
         end if;
         O.Readings.Append (Real_Array'(1 => (if B >= 21 then 1.0 else 0.0) + 0.01 * Gaussian (Rng)));
         O.Readings.Append (Real_Array'(1 => (if B >= 40 then 1.0 elsif B >= 10 then 0.5 else 0.0)));
         O.Readings.Append (Real_Array'(1 => (if B >= 31 then 0.1 else 0.0) + Third));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [(if B >= 20 then 1.0 else 0.0)]);
         Driver.Commands.Set_Target (Sent, 2, [(if B >= 40 then 1.5 elsif B >= 10 then 0.5 else 0.0)]);
         Driver.Commands.Set_Target (Sent, 3, [(if B >= 30 then 0.1 else 0.0)]);
         Observe (M, O, Sent);
      end loop;
      Estimate_Now (M);
      --  The jitter, not zero: the exact repeats say nothing about how far a
      --  resting reading moves when it does.
      Check (Reading_Noise (M, 3, 1) > 0.0, "a jittering reading has noise, got zero");
      Check_Close (Reading_Noise (M, 3, 1), Jitter / Sqrt (2.0), Jitter / Sqrt (2.0) / 2.0,
                   "noise of a reading that mostly repeats exactly");
      for B in 1 .. 29 loop
         Check (not Channels.Moving (M, 3, B), "a jitter is not motion, at beat" & B'Image);
      end loop;
      Check (Channels.Moving (M, 3, 31), "a push of the jittering reading is motion");
      Check_Close (Reading_Noise (M, 1, 1), 0.01, 0.003, "noise of a reading");
      Check (Channels.Asked (M, 1, 20), "a target one unit away asks for motion");
      Check (not Channels.Asked (M, 1, 30), "a target held where the reading is asks for nothing");
      Check (Channels.Pushed (M, 1, 20) and then Channels.Pushed (M, 1, 21), "the push lasts until answered");
      Check (not Channels.Pushed (M, 1, 25), "the push ends when the reading stops closing in");
      Check (Stillness.Group_Still (M, 1, 40), "a resting reading with noise is still");
      Check (not Stillness.Group_Still (M, 1, 21), "the answer to a push is motion");
      Check (Channels.Asked (M, 2, 40), "a target beyond the clip asks for motion");
      Check (not Channels.Pushed (M, 2, 41) and then not Channels.Pushed (M, 2, 50),
             "a push that is not answered within the measured delay is over");
   end Channel_Noise_And_Pushes;

   --  The one test of motion needs a visible step and the noise, each guarding
   --  the other, and the step is itself no smaller than what the reading tells
   --  from its noise. One commandable group creeps by 1e-9 a beat with a
   --  jitter of 1e-12 about it (a simulator's arm that never comes quite to
   --  rest), its target changing at each of the first thirty beats and then
   --  held, one camera. A lock-in credits the group's first channel with a
   --  visible step (Z over the root of Gain: eye 1 watches it).
   procedure Visible_Step_Needs_The_Noise is
      Creep_Per_Beat : constant Real := 1.0e-9;   --  what the readings change by at every beat

      --  The group's stream, given to M: Size channels' readings and one camera.
      procedure Creep (M : in out Model; Size : Positive) is
         Rng  : Generator;
         O    : Observation;
         Sent : Driver.Commands.Command;
      begin
         for B in 0 .. 119 loop
            O := (others => <>);
            O.Beat := Driver.Clock.Beat (B);
            O.Images.Append (Driver.Images.No_Image);
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            declare
               Readings : Real_Array (1 .. Size);
               Target   : Real_Array (1 .. Size);
            begin
               for C in 1 .. Size loop
                  Readings (C) := Creep_Per_Beat * Real (B) + 1.0e-12 * Gaussian (Rng);
                  --  The target changes at every beat of the first thirty, then stays.
                  Target (C) := (if B in 1 .. 29 and then B mod 2 = 1 then 1.0e-6 else 0.0);
               end loop;
               O.Readings.Append (Readings);
               Sent := Driver.Commands.Hold;
               Driver.Commands.Set_Target (Sent, 1, Target);
            end;
            Observe (M, O, Sent);
         end loop;
      end Creep;

      --  What a lock-in that credits eye 1 with a response to the group's
      --  first channel gives it.
      procedure Credit (M : in out Model; Gain : Real) is
         Effect : constant Eye_Effect :=
           (Verdict => Patch, Responding => 1, Textured => 1,
            Fraction => (Value => 1.0, Sigma => 0.1, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0);
      begin
         M.Eyes (1).Kept_Groups.Append (1);
         M.Eyes (1).Kept_Channels.Append (1);
         M.Eyes (1).Gains.Append (Gain);
         M.Eyes (1).Gain_Variances.Append (1.0);
         if M.Graph.Effects.Is_Empty then
            M.Graph.Effects.Append (Effect);
         else
            M.Graph.Effects.Replace_Element (1, Effect);
         end if;
      end Credit;

      --  What a reading tells from its noise: Z sigmas of the change of two readings.
      function Tellable (M : Model) return Real is
        (Driver.Conventions.Z * Reading_Noise (M, 1, 1) * Sqrt (2.0));
   begin
      --  A step of 3e-17, far below what the readings jitter by: it must not
      --  make every beat a push, leave the group no beat at rest, and its
      --  noise unmeasured, as it did A15's second arm.
      declare
         M : Model;
      begin
         Creep (M, 1);
         Credit (M, 1.0e34);
         Channels.Measure (M);
         Check (Reading_Noise (M, 1, 1) < Real'Last,
                "a visible step far below the readings' noise left the group's noise unmeasured: every beat was a push");
         Check (Known (Visible_Step (M, 1, 1)) and then Visible_Step (M, 1, 1).Value >= Tellable (M),
                "a visible step of 3e-17 is no larger than what the reading tells from its noise");
         Check (not Channels.Visible (M, 1, [Creep_Per_Beat]),
                "a change of the readings' own size, which a visible step of 3e-17 cannot make motion, is motion");
         Check (Channels.Visible (M, 1, [1.0]), "a push of 1 is not motion");
         --  A noise that is not measured gives no evidence, whatever the step,
         --  and tells no step either.
         M.Noise.Replace_Element (0, Real'Last);
         Check (not Known (Visible_Step (M, 1, 1)), "a channel whose noise is not measured has a visible step");
         Check (not Channels.Visible (M, 1, [1.0]), "a change is motion where no noise is measured, by the step alone");
      end;
      --  A step far above the noise is unchanged, and a change between the
      --  noise and the step, which the noise alone calls motion (a joint held
      --  away from rest jitters far more than it did at rest), is none. The
      --  gain credited is significant (a hundred, its variance one): a gain
      --  within its own spread of zero is no response (Lockin.Seen), and a
      --  gain of one with a variance of one gives no step at all.
      declare
         M : Model;
      begin
         Creep (M, 1);
         Credit (M, 1.0);
         Channels.Measure (M);
         Check (not Known (Visible_Step (M, 1, 1)), "a gain within its spread of zero gives a visible step");
         Check (Lockin.Shift (M, 1, 1, 1) = 0.0, "a gain within its spread of zero gives a shift");
         M.Eyes (1).Gains.Replace_Element (0, 100.0);
         Check (Reading_Noise (M, 1, 1) < Real'Last, "the group's noise is unmeasured");
         Check_Close (Visible_Step (M, 1, 1).Value, Driver.Conventions.Z / 10.0, 1.0e-12,
                      "the lock-in's step of Z over the root of a gain of 100 is not kept");
         Check (not Channels.Visible (M, 1, [0.5 * Visible_Step (M, 1, 1).Value]),
                "a change far above the noise but below what any eye sees is motion");
         Check (Channels.Visible (M, 1, [2.0 * Visible_Step (M, 1, 1).Value]),
                "a change above the visible step and the noise is not motion");
      end;
      --  A group of six channels, the step of the first the least that can be
      --  told from its noise: a change of that channel past the step, but
      --  within what six channels' noises together can make, is no motion
      --  (the step alone would give a group of many channels as many false
      --  alarms as it has channels).
      declare
         M : Model;
      begin
         Creep (M, 6);
         Credit (M, 1.0e34);
         Channels.Measure (M);
         Check (Reading_Noise (M, 1, 1) < Real'Last and then Reading_Noise (M, 1, 6) < Real'Last,
                "the six channels' noise is unmeasured");
         declare
            Step : constant Real := Visible_Step (M, 1, 1).Value;
            Past : constant Real_Array := [1.1 * Step, 0.0, 0.0, 0.0, 0.0, 0.0];
         begin
            Check (Step >= Tellable (M), "the least step is below what the reading tells from its noise");
            Check (not Channels.Visible (M, 1, Past),
                   "a change just past the visible step of one channel of six, within the six noises, is motion");
            Check (Channels.Visible (M, 1, [100.0 * Step, 0.0, 0.0, 0.0, 0.0, 0.0]),
                   "a change far past the visible step and the noises is not motion");
         end;
      end;
   end Visible_Step_Needs_The_Noise;

   procedure Eye_Stillness is
      Rng : Generator;
      S   : Eye_Stream;

      --  The texture with Gaussian noise of one level per pixel, a 16 x 16
      --  patch moved Shift pixels to the right.
      function Frame (Shift : Real) return Driver.Images.Image is
         use type Driver.Bytes.Offset;
         Data : Driver.Bytes.Byte_Array (1 .. 3 * Rig_Width * Rig_Height);
      begin
         for Y in 0 .. Rig_Height - 1 loop
            for X in 0 .. Rig_Width - 1 loop
               declare
                  Moved : constant Real := (if X in 20 .. 35 and then Y in 16 .. 31 then Shift else 0.0);
                  L : constant Real := Texture (Real (X) - Moved, Real (Y)) + Gaussian (Rng);
                  V : constant Driver.Bytes.Byte := Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                  K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Rig_Width + X) + 1);
               begin
                  Data (K) := V;
                  Data (K + 1) := V;
                  Data (K + 2) := V;
               end;
            end loop;
         end loop;
         return Driver.Images.Create (Rig_Width, Rig_Height, Data);
      end Frame;

      procedure Judge (I : Driver.Images.Image) is
      begin
         declare
            L : Real_Array (1 .. Driver.Images.Width (I) * Driver.Images.Height (I));
         begin
            Driver.Images.Luma (I, L);
            Stillness.Judge_Eye (S, I, L, Moved => False);
         end;
      end Judge;
   begin
      Judge (Frame (0.0));
      Check (not S.Is_Still, "one frame cannot say an eye is still");
      for B in 2 .. 8 loop
         Judge (Frame (0.0));
      end loop;
      --  A still eye alarms by chance at the rate Z has for a Gaussian
      --  (0.27 % a frame); two alarms in twenty frames would happen 0.14 %
      --  of the time.
      declare
         Alarms : Natural := 0;
      begin
         for B in 1 .. 20 loop
            Judge (Frame (0.0));
            if not S.Is_Still then
               Alarms := Alarms + 1;
            end if;
         end loop;
         Check (Alarms <= 1, "a still view with camera noise comes to rest; alarms in twenty frames:" & Alarms'Image);
      end;
      Judge (Frame (2.0));
      Check (not S.Is_Still, "a patch moving two pixels is a change");
      for B in 1 .. 3 loop
         Judge (Frame (2.0));
         Check (S.Is_Still, "the moved patch rests again at frame" & B'Image);
      end loop;
   end Eye_Stillness;

   --  A frame taken while the body moved, by its readings, starts the still view afresh though one frame's test sees no
   --  change in it; the same frame of a body that stood joins the view; the picture's own verdict is the same either
   --  way. SO-101's base camera on the hall (10-27): the elbow's least visible turn for step 3, unseen frame by frame,
   --  was averaged into the still view of the turn before it.
   procedure Eye_Still_View_Starts_At_Motion is
      Rng          : Generator;
      Stood, Moved : Eye_Stream;

      --  Eye_Stillness's picture: the texture with a level of Gaussian noise, a 16 x 16 patch moved Shift pixels.
      function Frame (Shift : Real) return Driver.Images.Image is
         use type Driver.Bytes.Offset;
         Data : Driver.Bytes.Byte_Array (1 .. 3 * Rig_Width * Rig_Height);
      begin
         for Y in 0 .. Rig_Height - 1 loop
            for X in 0 .. Rig_Width - 1 loop
               declare
                  Off : constant Real := (if X in 20 .. 35 and then Y in 16 .. 31 then Shift else 0.0);
                  L   : constant Real := Texture (Real (X) - Off, Real (Y)) + Gaussian (Rng);
                  V   : constant Driver.Bytes.Byte :=
                    Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                  K   : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Rig_Width + X) + 1);
               begin
                  Data (K) := V;
                  Data (K + 1) := V;
                  Data (K + 2) := V;
               end;
            end loop;
         end loop;
         return Driver.Images.Create (Rig_Width, Rig_Height, Data);
      end Frame;

      procedure Judge (S : in out Eye_Stream; I : Driver.Images.Image; Body_Moved : Boolean) is
         L : Real_Array (1 .. Driver.Images.Width (I) * Driver.Images.Height (I));
      begin
         Driver.Images.Luma (I, L);
         Stillness.Judge_Eye (S, I, L, Body_Moved);
      end Judge;
   begin
      for B in 1 .. 10 loop
         declare
            F : constant Driver.Images.Image := Frame (0.0);
         begin
            Judge (Stood, F, Body_Moved => False);
            Judge (Moved, F, Body_Moved => False);
         end;
      end loop;
      declare
         Before : constant Natural := Driver.Pixels.Frames (Stood.Settled);
         F      : constant Driver.Images.Image := Frame (0.02);
      begin
         Judge (Stood, F, Body_Moved => False);
         Judge (Moved, F, Body_Moved => True);
         Check (Stood.Is_Still, "a patch moved by a fiftieth of a pixel is a change one frame's test sees");
         Check (Driver.Pixels.Frames (Stood.Settled) = Before + 1,
                "a frame of a body that stood, its change unseen, was left out of the still view");
         Check (Moved.Is_Still = Stood.Is_Still, "the body's readings changed the picture's own verdict on a frame");
         Check (Driver.Pixels.Frames (Moved.Settled) = 1,
                "a frame taken while the body moved joined the still view of before it:"
                & Driver.Pixels.Frames (Moved.Settled)'Image & " frames");
      end;
   end Eye_Still_View_Starts_At_Motion;

   --  The one stop rule on pictures of three kinds, each after a move that began the watch: noise that wanders
   --  (every pixel a random walk, as A22's three eyes were at rest: a change over two beats 1.43 to 1.46 times one
   --  over one beat), noise that flickers (every frame its own), and a drift (a texture moved a little further every
   --  beat). The pictures that only make noise must be taken for at rest within a few beats of their move, whichever
   --  way their noise is; the drift never. A22's hand eye kept 235, 94, 102, 171 and 332 beats waiting for a picture
   --  that wandered. And a move that really decays is not at rest until it has.
   procedure Eye_Watch_Stops_On_Noise_Not_On_Drift is
      N     : constant := 2000;
      Level : constant Real := 100.0;   --  a pixel's luma variance over the longest run it has rested through
      Rng   : Generator;
      S     : Eye_Stream;
      Walk  : Real_Array (1 .. N) := [others => 0.0];
      type Picture is (Wandering, Flickering, Drifting);

      --  The next frame of a picture, Beat beats into the run and Bump levels of a move that decays by half a beat.
      procedure Next (Kind : Picture; Beat : Natural; Bump : Real) is
         Spare : constant Luma_Access := S.Before;
      begin
         for I in 1 .. N loop
            declare
               Base : constant Real := 128.0 + 40.0 * Sin (0.37 * Real (I));
            begin
               case Kind is
                  when Wandering =>
                     Walk (I) := Walk (I) + Gaussian (Rng);
                     S.Current (I) := Base + Walk (I);
                  when Flickering =>
                     S.Current (I) := Base + Gaussian (Rng);
                  when Drifting =>
                     S.Current (I) := 128.0 + 40.0 * Sin (0.37 * (Real (I) - 0.2 * Real (Beat)));
               end case;
               if I <= N / 10 then
                  S.Current (I) := S.Current (I) + Bump;
               end if;
            end;
         end loop;
         --  What the eye's judge would say of the frame: still unless the bump on a tenth of the pixels stands
         --  Z of their noise above the rest they are judged against (the judge reads each pixel against the
         --  rest view; the rig has no view, so it reads the bump).
         S.Is_Still := abs Bump <= Driver.Conventions.Z * Sqrt (Level);
         Stillness.Watch (S, Began_Moving => False);
         S.Before := S.Previous;
         S.Previous := S.Current;
         S.Current := Spare;
         S.Has_Before := S.Has_Previous;
         S.Has_Previous := True;
      end Next;

      --  How many beats after a move that began at Beat the watch took to stop, at most Within.
      function Stops_After (Kind : Picture; Beat : Natural; Bump : Real; Within : Natural) return Natural is
         Taken : Natural := 0;
         Level_Now : Real := Bump;
      begin
         S.Watch_Done := False;
         S.Watch_Have := False;
         while Taken < Within loop
            Next (Kind, Beat + Taken, Level_Now);
            Taken := Taken + 1;
            Level_Now := Level_Now / 2.0;
            exit when S.Watch_Done;
         end loop;
         return (if S.Watch_Done then Taken else Natural'Last);
      end Stops_After;
   begin
      S.Current := new Real_Array'(1 .. N => 0.0);
      S.Previous := new Real_Array'(1 .. N => 0.0);
      S.Before := new Real_Array'(1 .. N => 0.0);
      S.Variances := new Real_Array'(1 .. N => Level);
      S.Has_Judged := True;
      for Kind in Picture loop
         declare
            Taken : Natural;
         begin
            --  Each picture is an eye of its own: what rest changes by is measured on it alone.
            S.Watch_Have := False;
            S.Watch_Run := 0;
            S.Watch_Diffs.Clear;
            S.Watch_Levels.Clear;
            S.Watch_Scatter := 0.0;
            S.Watch_Scatter_Counted := 0;
            S.Watch_Rest := Real'Last;
            S.Watch_Rest_Scale := 0.0;
            S.Watch_Rest_Counted := 0;
            Next (Kind, 0, 0.0);
            Next (Kind, 1, 0.0);
            Next (Kind, 2, 0.0);
            for Move in 1 .. 6 loop
               Taken := Stops_After (Kind, 3 + 100 * Move, 0.0, (if Kind = Drifting then 60 else 12));
               if Kind = Drifting then
                  Check (Taken = Natural'Last, "a picture that drifts was taken for at rest after" & Taken'Image & " beats");
               else
                  Check (Taken <= 12, "a " & Kind'Image & " picture was still unsettled 12 beats after move" & Move'Image);
               end if;
            end loop;
         end;
      end loop;
      --  A change that decays by half every beat from a thousand levels: not at rest while it is as large as the
      --  noise, at rest within a few beats of its falling to it.
      declare
         Early : Natural;
         Late  : Natural;
      begin
         Next (Wandering, 0, 0.0);
         Early := Stops_After (Wandering, 900, 1000.0, 3);
         Check (Early = Natural'Last, "a move still a hundred levels large was settled after" & Early'Image & " beats (rest "
                & Driver.Log.Image (S.Watch_Rest, 4) & " +- " & Driver.Log.Image (S.Watch_Rest_Scale, 4) & ", wander "
                & Driver.Log.Image (S.Watch_Scatter, 4) & ", change now " & Driver.Log.Image (S.Change_1, 4) & ", two-beat "
                & Driver.Log.Image (S.Change_2, 4) & ", run" & S.Watch_Run'Image & ")");
         Late := Stops_After (Wandering, 1000, 1000.0, 40);
         Check (Late >= 8 and then Late <= 30, "a move decaying from a thousand levels was settled after" & Late'Image
                & " beats, not between 8 and 30");
      end;
   end Eye_Watch_Stops_On_Noise_Not_On_Drift;

   --  A joint held away from where it rested can jitter far more than it did
   --  at rest (a live x5 arm: 7e-18 at rest, 8e-17 held 1.5e-5 away, flipping
   --  its last bits each beat). Its push must still end once what remains of
   --  it is below what its eye can see.
   procedure Step_Ends_Despite_New_Jitter is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading : Real := 0.0;
   begin
      for B in 0 .. 199 loop
         if B = 40 then
            Target := 1.0;
            --  What a lock-in measured of an eye watching the channel: ten
            --  cells moving 100 pixels per reading unit (Visible_Step 0.009).
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               for Cell in 1 .. 10 loop
                  S.Gains.Append (1.0e4);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 10, Textured => 10,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B > 40 then
            --  Each beat closes seven eighths of the gap.
            Reading := Target - (Target - Reading) / 8.0;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         --  At rest a little noise; held, the last bits flip back and forth, so
         --  every beat's change is a hundred times the noise measured at rest.
         O.Readings.Append (Real_Array'(1 => Reading + (if B > 40 then (if B mod 2 = 0 then 1.0e-4 else -1.0e-4)
                                                       else 1.0e-6 * Gaussian (Rng))));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 1, "one push, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 1 then
         declare
            E : constant Episode := M.Groups (1).Episodes (1);
         begin
            Check (E.Ended, "the push never ends while the held reading jitters more than at rest");
            Check (not E.Ended or else E.End_At - E.Start <= 12,
                   "the push ends" & Natural'Image (E.End_At - E.Start) & " beats after it began");
         end;
      end if;
   end Step_Ends_Despite_New_Jitter;

   --  A group that never answered a push yet, asked beyond the limit it
   --  rests at (a live x5's closer at 1.0, asked 1.000244 by Recognize):
   --  nothing answers, and the push must be given up once the longest wait
   --  any push of the body took for its answer is over, not after as many
   --  beats as the stream had (A8 held that closer 3763 beats). Group 1
   --  answers its pushes a beat after each; group 2 is pushed once, at 100.
   procedure Unanswered_Push_Of_An_Unanswered_Group is
      M      : Model;
      Rng    : Generator;
      O      : Observation;
      Sent   : Driver.Commands.Command;
      Limit  : constant Real := 1.0;
      Arm, Arm_Target : Real := 0.0;
      Closer_Target   : Real := Limit;
   begin
      for B in 0 .. 140 loop
         if B in 40 | 52 then
            Arm_Target := 0.01;
         elsif B in 46 | 58 then
            Arm_Target := 0.0;
         elsif B in 41 | 47 | 53 | 59 then
            Arm := Arm_Target;
         elsif B = 100 then
            Closer_Target := Limit + 2.44e-4;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Arm + 1.0e-12 * Gaussian (Rng)));
         O.Readings.Append (Real_Array'(1 => Limit + 1.0e-12 * Gaussian (Rng)));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Arm_Target]);
         Driver.Commands.Set_Target (Sent, 2, [Closer_Target]);
         Observe (M, O, Sent);
      end loop;
      Check (M.Groups (1).Delay_Known, "group 1's delay is not measured");
      Check (Steps.Episodes (M, 2) = 1, "group 2 is pushed once, got" & Steps.Episodes (M, 2)'Image);
      if M.Groups (1).Delay_Known and then Steps.Episodes (M, 2) = 1 then
         declare
            E : constant Episode := M.Groups (2).Episodes (1);
         begin
            Check (E.Ended and then E.End_At - E.Start <= M.Groups (1).Delay_Beats + 1,
                   "the push nothing answers is still waited for"
                   & Natural'Image ((if E.Ended then E.End_At else 140) - E.Start) & " beats after it began; the body"
                   & " answered within" & M.Groups (1).Delay_Beats'Image);
            Check (not E.Ended or else E.Blocked, "the push nothing answered is not called blocked");
         end;
      end if;
   end Unanswered_Push_Of_An_Unanswered_Group;

   --  The closer of the test above, its reading creeping up 1e-11 a beat and otherwise exact (a simulator's readings
   --  creep for good: the fake robot's jaw, 10-15), measured at rest before it is pushed beyond its limit; the body
   --  answers eight beats after a command. The creep is measured, and the push nothing answers is not taken as
   --  answered by the creep since it began: the jaw, asked open at its open end, was answered at every level, went
   --  "further" each time, and was asked open to 7e13 units, the way shut never asked (f16f).
   procedure Unanswered_Push_Of_A_Creeping_Group is
      M      : Model;
      O      : Observation;
      Sent   : Driver.Commands.Command;
      Limit  : constant Real := 1.0;
      Creep  : constant Real := 1.0e-11;
      Arm, Arm_Target : Real := 0.0;
      Closer_Target   : Real := Limit;
   begin
      for B in 0 .. 140 loop
         if B = 20 then
            Arm_Target := 0.01;
         elsif B = 50 then
            Arm_Target := 0.0;
         elsif B in 28 | 58 then
            Arm := Arm_Target;
         elsif B = 100 then
            Closer_Target := Limit + 2.44e-4;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Arm));
         O.Readings.Append (Real_Array'(1 => Limit + Creep * Real (B)));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Arm_Target]);
         Driver.Commands.Set_Target (Sent, 2, [Closer_Target]);
         Observe (M, O, Sent);
         if B = 90 then
            Driver.Robot.Channels.Measure (M);
         end if;
      end loop;
      Check (abs (Driver.Robot.Channels.Drift (M, 2, 1) - Creep) <= 0.01 * Creep,
             "the closer's creep of" & Creep'Image & " a beat is measured as" & Driver.Robot.Channels.Drift (M, 2, 1)'Image);
      Check (M.Groups (1).Delay_Known and then M.Groups (1).Delay_Beats >= 7,
             "the body's delay of eight beats is not measured: " & M.Groups (1).Delay_Beats'Image);
      Check (Steps.Episodes (M, 2) = 1, "group 2 is pushed once, got" & Steps.Episodes (M, 2)'Image);
      if Steps.Episodes (M, 2) = 1 then
         declare
            E : constant Episode := M.Groups (2).Episodes (1);
         begin
            Check (not E.Moved, "the push nothing answered is answered" & Natural'Image (E.Moved_At - E.Start)
                   & " beats after it began, by the creep of the beats since");
            Check (E.Ended, "the push nothing answered never ends");
         end;
      end if;
   end Unanswered_Push_Of_A_Creeping_Group;

   --  A push sent while its group is still on its way from the push before is answered at its own start beat by
   --  that motion. Measured from such pushes the body's delay was 0 (seed 2 of the fake robot, 10-14: each probe
   --  level sent a beat after the one before), and every look thereafter read the body a beat before it had
   --  moved. The first push, cut short by the second, has no delivered fraction and does not count; the second,
   --  born moving, must not either: the delay stays unmeasured rather than measured as nothing.
   procedure Push_Born_Moving_Gives_No_Delay is
      M      : Model;
      Rng    : Generator;
      O      : Observation;
      Sent   : Driver.Commands.Command;
      Arm, Arm_Target : Real := 0.0;
   begin
      for B in 0 .. 200 loop
         if B = 40 then
            Arm_Target := 0.01;
         elsif B = 42 then
            Arm_Target := 0.02;
         end if;
         --  The reading follows a beat late, a quarter of the first ask a beat: it is on its way when the second
         --  push arrives, and goes on to the second target.
         if B in 41 .. 48 then
            Arm := 0.0025 * Real (B - 40);
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Arm + 1.0e-12 * Gaussian (Rng)));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Arm_Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 2, "the group is pushed twice, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 2 then
         Check (not M.Groups (1).Episodes (1).Born_Moving, "the first push, from rest, is called born moving");
         Check (M.Groups (1).Episodes (2).Born_Moving,
                "the second push, sent while the group was on its way from the first, is not called born moving");
      end if;
      Check (not M.Groups (1).Delay_Known or else M.Groups (1).Delay_Beats >= 1,
             "the body's delay was measured as" & M.Groups (1).Delay_Beats'Image
             & " beats from a push born into the motion of the push before");
   end Push_Born_Moving_Gives_No_Delay;

   --  The noise and push rounds of the channel measure can alternate for
   --  good (A9's replay spent 540 s in them at 1024 beats): a six-channel
   --  group read exactly but for five jitters of 1e-16 and 2e-16 at rest,
   --  pushed by about 1 and answering at once, with a tail of 4e-15 the beat
   --  after. Against the noise of the five jitters (one degree of freedom)
   --  the tail is no motion; taken among the rest beats it is a sixth
   --  sample (two degrees of freedom), against which it is motion: rest one
   --  round, pushed the next. The rounds must stop and take the tail as
   --  pushed, not as rest, whatever the stream's length (which decided where
   --  they were cut off: as many rounds as beats).
   procedure Alternating_Rounds_Stop is
      Tail : array (0 .. 1) of Boolean;
   begin
      for Extra in 0 .. 1 loop
         declare
            M    : Model;
            O    : Observation;
            Sent : Driver.Commands.Command;
            Reading, Target : Real := 0.0;
         begin
            for B in 0 .. 40 + Extra loop
               if B in 3 | 5 | 7 | 9 | 11 then
                  Reading := (if B mod 4 = 1 then 1.0e-16 else 2.0e-16);
               elsif B = 20 then
                  Target := 1.0;
               elsif B = 21 then
                  Reading := 1.0;
               elsif B = 22 then
                  Reading := 1.0 + 4.0e-15;
               end if;
               O := (others => <>);
               O.Beat := Driver.Clock.Beat (B);
               O.Images.Append (Driver.Images.No_Image);
               O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
               O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
               O.Readings.Append (Real_Array'[Reading, 1.1 * Reading, 0.9 * Reading, 1.05 * Reading, 0.95 * Reading,
                                               1.02 * Reading]);
               Sent := Driver.Commands.Hold;
               Driver.Commands.Set_Target (Sent, 1, [Target, 1.1 * Target, 0.9 * Target, 1.05 * Target, 0.95 * Target,
                                                     1.02 * Target]);
               Observe (M, O, Sent);
            end loop;
            Driver.Robot.Channels.Measure (M);
            Tail (Extra) := Driver.Robot.Channels.Pushed (M, 1, 22);
         end;
      end loop;
      Check (Tail (0) and then Tail (1), "the tail whose mark alternates is taken as rest, with"
             & (if Tail (0) then " 42" else " 41") & " beats");
   end Alternating_Rounds_Stop;

   --  A group the driver never commanded is held by sending it its current reading, beat after beat (Driver.Replies), and
   --  a joint that creeps by a hair every beat makes that target change every beat. A beat whose target changed was
   --  taken for no rest beat, so that the arms of A22 had no rest and no noise until the estimate at the doubling of
   --  the stream (736 beats), Moving was false, and each push was waited for as long as the stream was old: three holds
   --  of 292, 661 and 913 beats in the probes, 1,866 of the boot's 17,465. A hold asks for nothing: a beat is at rest
   --  when no push is under way. A real push, after the rest, is still a push.
   procedure Held_Group_Reaches_Its_Noise is
      M       : Model;
      Rng     : Generator;
      Reading : Real := 5.0e-10;
      Echo    : Real := Reading;   --  what the hold sends: the reading of the beat before
      Pushed_At : constant := 140;

      --  One beat: the group's reading creeps by a hair and jitters; the hold sends the reading of the beat before,
      --  until a real command, at Pushed_At, asks 0.01 of it.
      procedure Beat (B : Natural) is
         O    : Observation;
         Sent : Driver.Commands.Command := Driver.Commands.Hold;
      begin
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         if B >= Pushed_At then
            Reading := Reading + (if B = Pushed_At then 0.01 else 0.0);
         else
            Reading := Reading + 1.4e-12 + 1.0e-13 * Gaussian (Rng);
         end if;
         O.Readings.Append (Real_Array'(1 => Reading));
         Driver.Commands.Set_Target (Sent, 1, [(if B < Pushed_At then Echo else Echo + 0.01)]);
         Observe (M, O, Sent);
         Echo := Reading;
      end Beat;
   begin
      for B in 0 .. 119 loop
         Beat (B);
      end loop;
      Driver.Robot.Channels.Measure (M);
      Check (Driver.Robot.Channels.Noise (M, 1, 1) < 1.0e-9,
             "the noise of a held group never commanded is" & Real'Image (Driver.Robot.Channels.Noise (M, 1, 1))
             & ", not that of a creep of a hair a beat, after 120 beats");
      for B in 120 .. 160 loop
         Beat (B);
      end loop;
      Driver.Robot.Channels.Measure (M);
      Check (Driver.Robot.Channels.Pushed (M, 1, Pushed_At), "the push after the rest is not a push");
      Check (not Driver.Robot.Channels.Pushed (M, 1, 100), "a beat of the rest is part of a push");
   end Held_Group_Reaches_Its_Noise;

   --  A joint stopped short of its target by something it keeps chattering
   --  against (a live x5's arm 2 in its first Hadamard cell: joint 3 moved by
   --  20 to 240 visible steps every beat for 800 beats and never came to
   --  rest) moves every beat and never comes closer: its push must end soon
   --  after it stopped coming closer, judged blocked and not at rest. A free
   --  push that then rings about its target for longer than it took to get
   --  there still ends at rest, not blocked. Pushes 1 to 4 go back and forth
   --  by 0.01 for the joint's delay to be measured; push 5 asks 1.0 and meets
   --  the obstacle at 0.5; push 6 goes back to 0.
   procedure Step_Ends_Against_Chatter is
      M        : Model;
      Rng      : Generator;
      O        : Observation;
      Sent     : Driver.Commands.Command;
      Step     : constant Real := 0.009;   --  the visible step of the eye put in below
      Obstacle : constant Real := 0.5;
      Target, Reading, Ring : Real := 0.0;
   begin
      --  Short of 128 beats, whose estimate would measure the lock-in afresh.
      for B in 0 .. 126 loop
         --  The estimate at 64 beats measured the delay and the lock-in afresh;
         --  then the eye: ten cells moving 100 pixels per reading unit.
         if B = 64 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               for Cell in 1 .. 10 loop
                  S.Gains.Append (1.0e4);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 10, Textured => 10,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 40 | 46 | 52 | 58 then
            Target := (if B in 40 | 52 then 0.01 else 0.0);
         elsif B in 41 | 47 | 53 | 59 then
            Reading := Target;
         elsif B = 66 then
            Target := 1.0;
         elsif B in 67 .. 68 then
            --  Seven eighths of the way to the obstacle each beat.
            Reading := Obstacle - (Obstacle - Reading) / 8.0;
         elsif B in 69 .. 100 then
            --  Bouncing off it: 2 and 24 visible steps short of it in turn.
            Reading := Obstacle - Step * (if B mod 2 = 0 then 2.0 else 24.0);
         end if;
         if B = 100 then
            Target := 0.0;
         elsif B = 101 then
            Ring := 0.1;
            Reading := Ring;
         elsif B > 101 then
            --  About the target, each swing the other way and 0.6 as wide.
            Ring := -0.6 * Ring;
            Reading := Ring;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 6, "six pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 6 then
         for K in 1 .. 4 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then E.Rested and then not E.Blocked,
                      "free push" & K'Image & " did not end at rest, unblocked");
            end;
         end loop;
         declare
            E : constant Episode := M.Groups (1).Episodes (5);
         begin
            Check (E.Ended and then E.Settled, "the push against the obstacle never ends while the joint chatters");
            Check (not E.Ended or else not E.Settled or else E.End_At - E.Progress_At <= 2 * (E.Progress_At - E.Start) + 1,
                   "the push against the obstacle ends" & Natural'Image (E.End_At - E.Progress_At)
                   & " beats after it last came nearer or farther," & Natural'Image (E.Progress_At - E.Start) & " beats after it began");
            Check (E.Blocked and then not E.Rested, "the push against the obstacle is not judged blocked, still moving");
         end;
         declare
            E : constant Episode := M.Groups (1).Episodes (6);
         begin
            Check (E.Ended and then E.Settled and then E.Rested and then not E.Blocked,
                   "the push that rang about its target did not end at rest, unblocked");
         end;
      end if;
   end Step_Ends_Against_Chatter;

   --  The eye the step tests put in: every channel of group 1 (one, or as many
   --  as given) credited a visible step of 1.5 millionths (every estimate
   --  measures the lock-in afresh, so it is put back after each).
   procedure Put_Eye (M : in out Model; Count : Positive := 1) is
      S : Eye_Stream renames M.Eyes (1);
   begin
      S.Kept_Groups.Clear;
      S.Kept_Channels.Clear;
      S.Gains.Clear;
      S.Gain_Variances.Clear;
      for C in 1 .. Count loop
         S.Kept_Groups.Append (1);
         S.Kept_Channels.Append (C);
         S.Gains.Append (4.0e12);
         S.Gain_Variances.Append (1.0);
      end loop;
      M.Graph.Effects.Replace_Element
        (1, (Verdict => Whole, Responding => 1, Textured => 1,
             Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
   end Put_Eye;

   --  A joint pressed into a table moves about against it (A16's arm 1,
   --  joint 2: between 1.9758 and 1.9985 for 3,460 beats, and no end to the
   --  push) and now and then sets a new extreme, a few thousandths of a
   --  radian beyond the last, a step far beyond what an eye can see, at
   --  beats that come later and later, each within twice the beats since the
   --  push began, so that waiting as long as the push took to come as close
   --  as it did never ends. Such an extreme is no progress: the joint moves
   --  about by several times as much, and the push ends soon after it
   --  stopped coming closer, blocked. Pushes 1 to 4 go back and forth by
   --  0.01 for the group's free pushes and delay to be known; push 5 asks
   --  1.0, comes to 0.45 in three beats and moves about in a band 0.0227 wide beneath a
   --  top that rises by 0.002 at beats 2, 5, 10, 18, 31, 51, 84, 136, 219, 352
   --  and 565 of its chatter (each within twice the beats the push has gone
   --  on by then, which is as soon as the push may be given up); push 6 goes
   --  back to 0; push 7 asks 1.0 of a joint that moves about the same for
   --  two beats and gives way, 0.95 and 1.0: a gain beyond the chatter is
   --  progress, and the push is not given up.
   procedure Step_Ends_Against_Rising_Chatter is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Band : constant Real := 0.0227;
      Rises : constant array (1 .. 11) of Natural := [2, 5, 10, 18, 31, 51, 84, 136, 219, 352, 565];
      Target, Reading, Top : Real := 0.0;
   begin
      for B in 0 .. 799 loop
         --  The estimates at 64, 128, 256 and 512 beats measured the lock-in afresh
         --  (the first, the delay of the first four pushes too).
         if B = 64 or else B = 128 or else B = 256 or else B = 512 then
            Put_Eye (M);
         end if;
         if B in 40 | 46 | 52 | 58 then
            Target := (if B in 40 | 52 then 0.01 else 0.0);
         elsif B in 41 | 47 | 53 | 59 then
            Reading := Target;
         elsif B = 90 then
            Target := 1.0;
         elsif B in 91 .. 93 then
            Reading := (case B is when 91 => 0.25, when 92 => 0.4, when others => 0.45);
            Top := 0.45;
         elsif B in 94 .. 700 then
            if (for some R of Rises => R = B - 93) then
               Top := Top + 0.002;
               Reading := Top;
            else
               Reading := Top - Band * Uniform (Rng);
            end if;
         elsif B = 701 then
            Target := 0.0;
         elsif B = 702 then
            Reading := 0.0;
         elsif B = 740 then
            Target := 1.0;
         elsif B in 741 .. 743 then
            Reading := (case B is when 741 => 0.25, when 742 => 0.4, when others => 0.45);
            Top := 0.45;
         elsif B in 744 .. 745 then
            Reading := Top - Band * Uniform (Rng);
         elsif B in 746 .. 747 then
            Reading := (if B = 746 then 0.95 else 1.0);
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 7, "seven pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 7 then
         declare
            E    : constant Episode := M.Groups (1).Episodes (5);
            Wait : constant Natural := M.Groups (1).Delay_Beats;
         begin
            Check (E.Ended and then E.Settled, "the push against the table never ends while its joint moves about it");
            if E.Ended and then E.Settled then
               Check (E.Blocked and then not E.Rested, "the push against the table is not judged blocked, still moving");
               Check (E.Progress_At - E.Start <= 3,
                      "an extreme of the chatter is taken for the push's closest point, at"
                      & Natural'Image (E.Progress_At - E.Start));
               Check (E.End_At - E.Progress_At <= Integer'Max (E.Progress_At - E.Start, Wait) + 1,
                      "the push ends" & Natural'Image (E.End_At - E.Progress_At) & " beats after it last came nearer or farther");
            end if;
         end;
         declare
            E : constant Episode := M.Groups (1).Episodes (7);
         begin
            Check (E.Ended and then E.Rested and then not E.Blocked,
                   "the push that gave way, from 0.45 to its target in two beats, was given up or called blocked");
         end;
      end if;
   end Step_Ends_Against_Rising_Chatter;

   --  A push whose own joint holds still, but another joint of its group moves
   --  about by a hundredth of a radian every beat (so that the group never
   --  comes to rest), and whose own reading, exact but for a noise of 1e-12,
   --  sets a new extreme now and then, by 1e-7: a step that stands far out
   --  of the push's own scatter (the reading repeats between the rises) but
   --  is below what an eye can see, 1.5 millionths. It is no progress either:
   --  no one could tell where the push stands from where it stood, and the
   --  push ends soon after it stopped coming closer, blocked. Group 1 has
   --  two channels; pushes 1 to 4 take the first back and forth by 0.01;
   --  push 5 asks it 1.0, and it comes to 0.45 in three beats and rises by
   --  1e-7 at beats 2, 5, 10, 18, 31, 51, 84, 136, 219, 352 and 565, a sum
   --  that stays below the step.
   procedure Step_Ends_Beside_A_Moving_Joint is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Rises : constant array (1 .. 11) of Natural := [2, 5, 10, 18, 31, 51, 84, 136, 219, 352, 565];
      Target, Reading : Real_Array (1 .. 2) := [0.0, 0.0];
   begin
      for B in 0 .. 700 loop
         --  The estimates at 64, 128, 256 and 512 beats measured the lock-in afresh.
         if B = 64 or else B = 128 or else B = 256 or else B = 512 then
            Put_Eye (M, 2);
         end if;
         if B in 40 | 46 | 52 | 58 then
            Target (1) := (if B in 40 | 52 then 0.01 else 0.0);
         elsif B in 41 | 47 | 53 | 59 then
            Reading (1) := Target (1);
         elsif B = 90 then
            Target (1) := 1.0;
         elsif B in 91 .. 93 then
            Reading (1) := (case B is when 91 => 0.25, when 92 => 0.4, when others => 0.45);
         elsif B in 94 .. 700 and then (for some R of Rises => R = B - 93) then
            Reading (1) := Reading (1) + 1.0e-7;
         end if;
         if B >= 91 then
            Reading (2) := 0.01 * (2.0 * Uniform (Rng) - 1.0);
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading (1) + 1.0e-12 * Gaussian (Rng), Reading (2) + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target (1), Target (2)]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 5, "five pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 5 then
         declare
            E    : constant Episode := M.Groups (1).Episodes (5);
            Wait : constant Natural := M.Groups (1).Delay_Beats;
         begin
            Check (E.Ended and then E.Settled, "the push never ends while another joint of its group moves about");
            if E.Ended and then E.Settled then
               Check (E.Blocked and then not E.Rested, "the push is not judged blocked, still moving");
               Check (E.Progress_At - E.Start <= 3,
                      "a rise of 1e-7 is taken for the push's closest point, at" & Natural'Image (E.Progress_At - E.Start));
               Check (E.End_At - E.Progress_At <= Integer'Max (E.Progress_At - E.Start, Wait) + 1,
                      "the push ends" & Natural'Image (E.End_At - E.Progress_At) & " beats after it last came nearer or farther");
            end if;
         end;
      end if;
   end Step_Ends_Beside_A_Moving_Joint;

   --  A86's arm 2 let go from a press: the hold took the readings two beats
   --  before it took effect, while the arm chattered against the table, and
   --  asked a step of that chatter; the arm sprang back off the table, past
   --  that step along its ask and farther off it, and then moved about by
   --  1e-4 every three beats for as long as it was watched. Pushes 1 to 4 go
   --  back and forth by 0.01 on the first channel; push 5 asks it 1.0 and
   --  stops at 0.45; push 6, the hold, asks 0.002 of each channel, and the
   --  arm springs back by 0.02 along the first and 0.005 against the second,
   --  0.85 of what is left at every beat, with a cycle of 1e-4, 0 and -1e-4
   --  on both from its first beat on. The hold is under way while the arm
   --  springs back by more than it moves about, and then given up, still
   --  moving: followed along its ask alone, it came nearer its target at its
   --  first beat by more than it asked and was never given up.
   procedure Step_Ends_After_A_Spring_Back is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Cycle : constant Real := 1.0e-4;
      Hold  : constant Natural := 120;
      Target, Reading : Real_Array (1 .. 2) := [0.0, 0.0];

      --  How far the arm has sprung back, as a part of all it springs back, B beats into the hold.
      function Sprung (B : Natural) return Real is (1.0 - 0.85 ** (B - Hold));
   begin
      for B in 0 .. 400 loop
         --  The estimates at 64, 128 and 256 beats measured the lock-in afresh.
         if B = 64 or else B = 128 or else B = 256 then
            Put_Eye (M, 2);
         end if;
         if B in 40 | 46 | 52 | 58 then
            Target (1) := (if B in 40 | 52 then 0.01 else 0.0);
         elsif B in 41 | 47 | 53 | 59 then
            Reading (1) := Target (1);
         elsif B = 90 then
            Target (1) := 1.0;
         elsif B in 91 .. 93 then
            Reading (1) := (case B is when 91 => 0.25, when 92 => 0.4, when others => 0.45);
         elsif B = Hold then
            Target := [0.452, 0.002];
         elsif B > Hold then
            Reading := [0.45 + 0.02 * Sprung (B) + Cycle * Real (1 - B mod 3), -0.005 * Sprung (B) + Cycle * Real (1 - B mod 3)];
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading (1) + 1.0e-12 * Gaussian (Rng), Reading (2) + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target (1), Target (2)]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 6, "six pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 6 then
         declare
            E    : constant Episode := M.Groups (1).Episodes (6);
            Wait : constant Natural := M.Groups (1).Delay_Beats;
         begin
            Check (E.Ended and then E.Settled, "the hold never ends while the arm moves about where it sprang back to");
            if E.Ended and then E.Settled then
               Check (not E.Rested, "the hold is not given up still moving");
               Check (0.02 * (Sprung (E.End_At) - Sprung (E.End_At - 1)) < Cycle,
                      "the hold ends at beat" & Natural'Image (E.End_At - Hold) & " of the spring back, while the arm"
                      & " springs back by more than it moves about");
               Check (E.End_At - E.Progress_At <= Integer'Max (E.Progress_At - E.Start, Wait) + 1,
                      "the hold ends" & Natural'Image (E.End_At - E.Progress_At)
                      & " beats after it last came nearer or farther");
            end if;
         end;
      end if;
   end Step_Ends_After_A_Spring_Back;

   --  A joint an eye watches, read exactly as a simulator reads it: every push
   --  closes all but 1.45 % of its ask in one beat and stops there, which is
   --  far beyond the readings' noise and, for a long push, far beyond what
   --  short free pushes fell short by, yet below the step the eye can see.
   --  Pushes 1 to 6 go back and forth by 0.01, push 7 goes 0.5 out and push
   --  8 back, both freely; pushes 9 and 10 ask less than the eye can see, so
   --  their motion, all but 1.45 % of it, is never seen; push 11 asks 0.5 and
   --  is stopped half way; push 12 asks 0.35 and nothing answers it.
   procedure Step_Short_Of_Sight is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Asks : constant Real_Array (1 .. 12) := [0.01, 0.0, 0.01, 0.0, 0.01, 0.0, 0.5, 0.0, -0.001, 0.0, 0.5, 0.6];
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 160 loop
         --  Every estimate measures the lock-in afresh; the eye is put back.
         if B = 64 or else B = 128 then
            --  What a lock-in measured of an eye watching the channel: ten
            --  cells moving 100 pixels per reading unit (Visible_Step 0.009).
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               for Cell in 1 .. 10 loop
                  S.Gains.Append (1.0e4);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 10, Textured => 10,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         --  Pushes 1 to 8, then (once an estimate has measured how long the
         --  joint takes to answer) pushes 9 to 12.
         if B in 66 .. 105 | 130 .. 149 then
            declare
               K     : constant Natural := (if B < 128 then (B - 66) / 5 + 1 else (B - 130) / 5 + 9);
               Phase : constant Natural := (if B < 128 then (B - 66) mod 5 else (B - 130) mod 5);
            begin
               if K in Asks'Range then
                  if Phase = 0 then
                     Target := Asks (K);
                     From := Reading;
                  elsif Phase = 1 then
                     Reading := From + (case K is when 11 => 0.5, when 12 => 0.0, when others => 0.9855) * (Target - From);
                  end if;
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 12, "twelve pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 12 then
         for K in 1 .. 10 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " stopped short by" & Real'Image (E.Shortfall.Value)
                      & ", less than the eye can see, and is called blocked");
            end;
         end loop;
         Check (M.Groups (1).Episodes (11).Blocked, "a push stopped half way is not called blocked");
         Check (M.Groups (1).Episodes (12).Blocked, "a push the eye could see and nothing answered is not called blocked");
      end if;
   end Step_Short_Of_Sight;

   --  A press let go (A16's Press_Once): the arm held against a surface is asked to stand where it is, a push of a
   --  millionth, less than any eye can see, and relaxes by five steps an eye can see meanwhile, the way the surface
   --  lets it. What moved it was not the push, and there is no delivery to fall short of: it is neither blocked nor
   --  a free push (the group's free baseline does not take its shortfall), and its delivered fraction is unknown.
   --  Where the ask is one the test sees and the joint is moved against it by a motion the test sees, nothing was
   --  delivered either: that push is blocked, its delivered fraction below zero. Pushes 1 to 6 go out and back by
   --  0.01 (the eye sees 0.009), push 7 is the let-go, push 8 asks 0.5 and is delivered, push 9 asks 0.4 and the
   --  joint is moved 0.2 the other way.
   procedure Step_Let_Go_Is_Not_Blocked is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 140 loop
         if B = 64 or else B = 128 then
            --  What a lock-in measured of an eye watching the channel: ten cells moving 100 pixels per reading unit
            --  (Visible_Step 0.009).
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               for Cell in 1 .. 10 loop
                  S.Gains.Append (1.0e4);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 10, Textured => 10,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 110 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target := (case K is
                                when 7 => Reading + 1.0e-6,
                                when 8 => 0.5,
                                when 9 => Reading + 0.4,
                                when others => (if K mod 2 = 1 then 0.01 else 0.0));
                  From := Reading;
               elsif Phase = 1 then
                  Reading := From + (case K is
                                        when 7 => -0.05,
                                        when 9 => -0.2,
                                        when others => 0.9855 * (Target - From));
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 9, "nine pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 9 then
         for K in 1 .. 8 loop
            Check (M.Groups (1).Episodes (K).Ended and then not M.Groups (1).Episodes (K).Blocked,
                   "push" & K'Image & " is called blocked");
         end loop;
         declare
            Let_Go : constant Episode := M.Groups (1).Episodes (7);
            Against : constant Episode := M.Groups (1).Episodes (9);
         begin
            Check (not Let_Go.Blocked,
                   "a push of a millionth, which the arm relaxed" & Real'Image (Let_Go.Shortfall.Value)
                   & " away from, is called blocked");
            Check (not Known (Let_Go.Delivered) and then not Known (Let_Go.Shortfall),
                   "a push that asked less than any eye can see has a delivered fraction");
            Check (Natural (M.Groups (1).Free_Shortfalls.Length) = 7,
                   "the free pushes are" & Natural'Image (Natural (M.Groups (1).Free_Shortfalls.Length))
                   & ", not seven (pushes 1 to 6 and 8: the let-go and the push moved against are not among them)");
            Check (Against.Blocked and then Known (Against.Delivered) and then Against.Delivered.Value < 0.0,
                   "a push the joint was moved against by a visible motion is not blocked with a negative delivered "
                   & "fraction:" & Real'Image (Against.Delivered.Value));
         end;
      end if;
   end Step_Let_Go_Is_Not_Blocked;

   --  A servo's dead band: a push that asks less than the body is known to execute (Steps.Executed_Least, the
   --  least ask a push moved the channel for) and is not answered is not judged, neither blocked nor delivered;
   --  one that asks at least that and is not answered is blocked. Pushes 1 to 3 ask 0.5 and are followed (the
   --  body executes 0.5); after the estimate at beat 128 has measured the body's delay from them (a push nothing
   --  answers ends after that wait), push 4 asks 0.1 and nothing moves, push 5 asks 0.6 and nothing moves (the
   --  kiwi's joints, asked 2.4 to 3.8 units within their dead band at the sweep's first levels, were every one
   --  called blocked where they stood, R5 on 10-14).
   procedure Step_Within_The_Dead_Band_Is_Not_Judged is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 160 loop
         if B = 64 or else B = 128 then
            --  What a lock-in measured of an eye watching the channel: ten cells moving 100 pixels per reading unit
            --  (Visible_Step 0.009), set again after the estimate at 128 has measured the eye.
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               for Cell in 1 .. 10 loop
                  S.Gains.Append (1.0e4);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 10, Textured => 10,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 80 or else B in 130 .. 141 then
            declare
               K     : constant Natural := (if B < 130 then (B - 66) / 5 + 1 else 4 + (B - 130) / 6);
               Phase : constant Natural := (if B < 130 then (B - 66) mod 5 else (B - 130) mod 6);
            begin
               if Phase = 0 then
                  Target := Reading + (case K is when 4 => 0.1, when 5 => 0.6, when others => 0.5);
                  From := Reading;
               elsif Phase = 1 and then K <= 3 then
                  Reading := From + 0.9855 * (Target - From);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Steps.Episodes (M, 1) = 5, "five pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 5 then
         declare
            Within : constant Episode := M.Groups (1).Episodes (4);
            Beyond : constant Episode := M.Groups (1).Episodes (5);
         begin
            Check (abs (Steps.Executed_Least (M, 1, 1) - 0.5) < 0.01,
                   "the least ask the body executed is" & Real'Image (Steps.Executed_Least (M, 1, 1)) & ", not 0.5");
            Check (Within.Ended and then not Within.Blocked and then not Known (Within.Delivered),
                   "a push of 0.1, within the dead band of a body known to execute 0.5, that nothing answered is "
                   & "judged (blocked " & Within.Blocked'Image & ", delivered known " & Known (Within.Delivered)'Image
                   & ")");
            Check (Beyond.Ended and then Beyond.Blocked,
                   "a push of 0.6, as large as the body executes, that nothing answered is not called blocked");
         end;
      end if;
   end Step_Within_The_Dead_Band_Is_Not_Judged;


   --  The same joint, read as exactly, with a lock-in's step of 3e-17
   --  credited to it (what a lock-in that credits a group with the pictures'
   --  motion beside its tiny readings can fit): every push asks 1e-10, closes
   --  all but 1.45 % of it in one beat and stops there, short by 1.45e-12, a
   --  hair beside the readings' noise of 1e-12. A step of 3e-17 would take
   --  that shortfall for one an eye can see and the push for blocked; the
   --  step is no smaller than the reading tells from its noise (Visible_Step),
   --  and no push is blocked.
   procedure Step_Short_By_A_Hair is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Asks : constant Real_Array (1 .. 8) := [1.0e-10, 0.0, 1.0e-10, 0.0, 1.0e-10, 0.0, 1.0e-10, 0.0];
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 130 loop
         --  Every estimate measures the lock-in afresh; the eye is put back.
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               S.Gains.Append (1.0e34);
               S.Gain_Variances.Append (1.0);
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 105 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target := Asks (K);
                  From := Reading;
               elsif Phase = 1 then
                  Reading := From + 0.9855 * (Target - From);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)), "the eye's visible step is known");
      Check (Visible_Step (M, 1, 1).Value > 3.0e-12,
             "the credited step is below what the reading tells from its noise:"
             & Real'Image (Visible_Step (M, 1, 1).Value));
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         for K in 1 .. 8 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " stopped short by" & Real'Image (E.Shortfall.Value)
                      & ", a hair beside the readings' noise, and is called blocked");
            end;
         end loop;
      end if;
   end Step_Short_By_A_Hair;

   --  A joint an eye watches whose every push stops a few millionths of a
   --  radian short, whatever its size, as a simulator's controller does (A15's
   --  arms: 1 to 5 millionths against a visible step of the same size): the
   --  joint's own error is as large as the step an eye can see, and a push is
   --  not blocked for it. Pushes 1 to 8 go out and back by 0.005, 0.02, 0.1 and
   --  0.9, stopping 4 to 6 millionths short; push 9 asks 0.5 and is stopped
   --  half way. The joint is credited a step of 3 millionths, read exactly with
   --  a noise of 1e-12.
   procedure Step_Free_Error_As_Big_As_The_Step is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Asks   : constant Real_Array (1 .. 9) := [0.005, 0.0, 0.02, 0.0, 0.1, 0.0, 0.9, 0.0, 0.5];
      Errors : constant Real_Array (1 .. 8) := [4.0e-6, 6.0e-6, 5.0e-6, 4.5e-6, 6.0e-6, 5.5e-6, 4.0e-6, 5.0e-6];
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 130 loop
         --  Every estimate measures the lock-in afresh; the eye is put back.
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               S.Gains.Append (1.0e12);
               S.Gain_Variances.Append (1.0);
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 110 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target := Asks (K);
                  From := Reading;
               elsif Phase = 1 then
                  Reading := (if K = 9 then From + 0.5 * (Target - From)
                              else Target - (if Target > From then 1.0 else -1.0) * Errors (K));
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Known (Visible_Step (M, 1, 1)) and then Visible_Step (M, 1, 1).Value < Errors (1),
             "the joint's error is not as large as the step an eye can see");
      Check (Steps.Episodes (M, 1) = 9, "nine pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 9 then
         for K in 1 .. 8 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " stopped short by" & Real'Image (E.Shortfall.Value)
                      & ", as every free push of the joint does, and is called blocked");
            end;
         end loop;
         Check (M.Groups (1).Episodes (9).Blocked, "a push stopped half way is not called blocked");
      end if;
   end Step_Free_Error_As_Big_As_The_Step;

   --  A joint an eye watches whose free pushes stop short by more later than
   --  they did at first, as A15's arm did once its hand met the table (a tenth
   --  to a third of a millionth of a radian in the boot, two or three
   --  millionths after), the step an eye can see being 1.5 millionths: the
   --  group's free pushes before say what its joints fall short by on their
   --  own, and a level a little above their largest must not call every push
   --  after it blocked, as a baseline that only the pushes judged free ever
   --  extend does, however tight the scatter of the many pushes before it.
   --  Pushes 1 to 40 go out and back by 0.1, 0.02, 0.3 and 0.005, stopping 0.1 to
   --  0.3 millionth short (push 17 by 0.8); pushes 41 to 48 stop 1.6 to 2.2
   --  millionths short; push 49 asks 0.5 and is stopped half way.
   procedure Step_Free_Level_Moves is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Sizes  : constant Real_Array (1 .. 4) := [0.1, 0.02, 0.3, 0.005];
      Target, Reading, From : Real := 0.0;
      Error  : Real := 0.0;
   begin
      for B in 0 .. 330 loop
         --  Every estimate measures the lock-in afresh; the eye is put back.
         if B = 64 or else B = 128 or else B = 256 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               S.Kept_Groups.Append (1);
               S.Kept_Channels.Append (1);
               S.Gains.Append (4.0e12);
               S.Gain_Variances.Append (1.0);
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 310 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  --  Out by the size, back to nothing.
                  Target := (if K mod 2 = 1 then Sizes ((K - 1) / 2 mod 4 + 1) else 0.0);
                  if K = 49 then
                     Target := 0.5;
                  end if;
                  From := Reading;
                  Error := (if K = 17 then 8.0e-7
                            elsif K <= 40 then 1.0e-7 + 2.0e-7 * Uniform (Rng)
                            else 1.6e-6 + 6.0e-7 * Uniform (Rng));
               elsif Phase = 1 then
                  Reading := (if K = 49 then From + 0.5 * (Target - From)
                              else Target - (if Target > From then 1.0 else -1.0) * Error);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 49, "forty-nine pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 49 then
         for K in 1 .. 48 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " stopped short by" & Real'Image (E.Shortfall.Value)
                      & ", as the joint's free pushes do, and is called blocked");
            end;
         end loop;
         Check (M.Groups (1).Episodes (49).Blocked, "a push stopped half way is not called blocked");
      end if;
   end Step_Free_Level_Moves;

   --  A joint that stands off its target by a few millionths of a radian
   --  while another is pushed: what it does not deliver is its share of the
   --  push's ask, which is nothing, not a shortfall of 3 millionths that an eye
   --  could see. Group 1 has two channels, both credited a step of 1.5
   --  millionths; the second stays 3 millionths short of its target
   --  throughout, and the first goes out and back by 0.1 and 0.02, stopping
   --  short by nothing in its first four pushes and by half a millionth in
   --  the next four: more than the pushes before it fell short by, and less
   --  than the step an eye can see.
   procedure Step_Standing_Error_Is_No_Push is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Asks   : constant Real_Array (1 .. 8) := [0.1, 0.0, 0.02, 0.0, 0.1, 0.0, 0.02, 0.0];
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 130 loop
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               for C in 1 .. 2 loop
                  S.Kept_Groups.Append (1);
                  S.Kept_Channels.Append (C);
                  S.Gains.Append (4.0e12);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 105 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target := Asks (K);
                  From := Reading;
               elsif Phase = 1 then
                  Reading := Target - (if Target > From then 1.0 else -1.0) * (if K <= 4 then 0.0 else 5.0e-7);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading + 1.0e-12 * Gaussian (Rng), 0.3 - 3.0e-6 + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target, 0.3]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         for K in 1 .. 8 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " is called blocked for a joint that stands 3 millionths off its target");
            end;
         end loop;
      end if;
   end Step_Standing_Error_Is_No_Push;

   --  A joint stopped by something stays stopped while the others move on,
   --  its target the same from one push to the next (A15's arm with its hand
   --  on the table: joint 3 stood 0.014 short of its target through the
   --  pushes that followed, which moved the others by 0.1 to 0.4, and their
   --  own free shortfalls were a few millionths): what it does not move is in
   --  every one of those pushes' shortfall, by its share of their ask, and
   --  they are blocked, the one that sent it there and each that leaves it
   --  there. Group 1 has two channels, both credited a step of 1.5
   --  millionths. Pushes 1 to 4 take the first joint out and back by 0.4,
   --  freely, the second staying at 0; push 5 sends the first out and the
   --  second to 0.0139, which it never reaches; pushes 6 to 8 take the first
   --  back and out and back, the second asked for no more than before.
   procedure Step_Stuck_Joint_Stays_Stuck is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading : Real_Array (1 .. 2) := [0.0, 0.0];
      From : Real := 0.0;
   begin
      for B in 0 .. 130 loop
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               for C in 1 .. 2 loop
                  S.Kept_Groups.Append (1);
                  S.Kept_Channels.Append (C);
                  S.Gains.Append (4.0e12);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 105 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target (1) := (if K mod 2 = 1 then 0.4 else 0.0);
                  Target (2) := (if K >= 5 then 0.0139 else 0.0);
                  From := Reading (1);
               elsif Phase = 1 then
                  --  The first joint stops 5e-7 short of where it was sent.
                  Reading (1) := Target (1) - (if Target (1) > From then 1.0 else -1.0) * 5.0e-7;
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading (1) + 1.0e-12 * Gaussian (Rng), Reading (2) + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target (1), Target (2)]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         for K in 1 .. 4 loop
            Check (M.Groups (1).Episodes (K).Ended and then not M.Groups (1).Episodes (K).Blocked,
                   "free push" & K'Image & " is called blocked");
         end loop;
         for K in 5 .. 8 loop
            Check (M.Groups (1).Episodes (K).Blocked,
                   "push" & K'Image & ", whose second joint is stopped by something, is not called blocked");
         end loop;
      end if;
   end Step_Stuck_Joint_Stays_Stuck;

   --  A joint on a position loop that holds a load lands off its target by the load, the same way whichever way it is
   --  pushed (the hall's G1, 10-26: its elbows 0.08 rad under their forearms); a servo with a dead band lands short of
   --  its target whichever way it comes. Pushes 1 to 8 go up by 0.5 and back down: the first joint lands 0.05 above
   --  each target, the second 0.05 short of it along each ask. The first has no dead band and stands 0.05 off its
   --  targets; the second has a band of 0.05 and stands as far off.
   procedure Load_Is_No_Dead_Band is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading : Real_Array (1 .. 2) := [0.0, 0.0];
      Load : constant Real := 0.05;
   begin
      for B in 0 .. 130 loop
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               for C in 1 .. 2 loop
                  S.Kept_Groups.Append (1);
                  S.Kept_Channels.Append (C);
                  S.Gains.Append (4.0e12);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         if B in 66 .. 105 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
               Up    : constant Boolean := K mod 2 = 1;
            begin
               if Phase = 0 then
                  Target := [others => (if Up then 0.5 else 0.0)];
               elsif Phase = 1 then
                  Reading (1) := Target (1) + Load;
                  Reading (2) := Target (2) - (if Up then Load else -Load);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading (1) + 1.0e-12 * Gaussian (Rng), Reading (2) + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target (1), Target (2)]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      Check (Steps.Dead_Band (M, 1, 1) < 0.1 * Load,
             "the joint that holds a load of" & Load'Image & " has a dead band of" & Steps.Dead_Band (M, 1, 1)'Image);
      Check (abs (Steps.Standoff (M, 1, 1) - Load) < 0.1 * Load,
             "the joint that holds a load of" & Load'Image & " stands" & Steps.Standoff (M, 1, 1)'Image & " off");
      Check (abs (Steps.Dead_Band (M, 1, 2) - Load) < 0.1 * Load,
             "the joint with a dead band of" & Load'Image & " has a band of" & Steps.Dead_Band (M, 1, 2)'Image);
      Check (abs (Steps.Standoff (M, 1, 2) - Load) < 0.1 * Load,
             "the joint with a dead band of" & Load'Image & " stands" & Steps.Standoff (M, 1, 2)'Image & " off");
   end Load_Is_No_Dead_Band;

   --  A joint no push has moved yet, standing 0.05 off its target under a load from the start: its target moved 0.02,
   --  within how far it stands off, and standing still, it is not judged blocked (a servo's band or the friction a
   --  held load leaves may keep it); moved 0.2 and standing still, it is. Pushes 1 to 4 move the first joint up by 0.5
   --  and back, and rest; pushes 5 and 7 move the second's target 0.02 and 0.2 from its first, 6 and 8 back.
   procedure Standing_Off_Is_No_Block is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading : Real_Array (1 .. 2) := [0.0, 0.05];
   begin
      for B in 0 .. 170 loop
         if B = 64 or else B = 128 then
            declare
               S : Eye_Stream renames M.Eyes (1);
            begin
               S.Kept_Groups.Clear;
               S.Kept_Channels.Clear;
               S.Gains.Clear;
               S.Gain_Variances.Clear;
               for C in 1 .. 2 loop
                  S.Kept_Groups.Append (1);
                  S.Kept_Channels.Append (C);
                  S.Gains.Append (4.0e12);
                  S.Gain_Variances.Append (1.0);
               end loop;
               M.Graph.Effects.Replace_Element
                 (1, (Verdict => Whole, Responding => 1, Textured => 1,
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            end;
         end if;
         --  Pushes 1 to 4 five beats each from beat 66, pushes 5 to 8 twenty each from beat 86.
         if B in 66 .. 85 and then (B - 66) mod 5 = 0 then
            Target (1) := (if ((B - 66) / 5) mod 2 = 0 then 0.5 else 0.0);
         elsif B in 66 .. 85 and then (B - 66) mod 5 = 1 then
            Reading (1) := Target (1);
         elsif B in 86 .. 165 and then (B - 86) mod 20 = 0 then
            Target (2) := (case (B - 86) / 20 is when 0 => 0.02, when 2 => 0.2, when others => 0.0);
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(Reading (1) + 1.0e-12 * Gaussian (Rng), Reading (2) + 1.0e-12 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target (1), Target (2)]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         Check (M.Groups (1).Episodes (5).Ended and then not M.Groups (1).Episodes (5).Blocked,
                "push 5, asking 0.02 of a joint no push has moved, standing 0.05 off its target, is called blocked");
         Check (M.Groups (1).Episodes (7).Ended and then M.Groups (1).Episodes (7).Blocked,
                "push 7, asking 0.2 of a joint no push has moved, standing 0.05 off its target, is not called blocked");
      end if;
   end Standing_Off_Is_No_Block;

   --  A joint no eye watches, read exactly as a simulator reads it: its reading
   --  repeats exactly at rest (noise zero), and every free push lands on its
   --  target to the last bit, so that what the group's free pushes fell short
   --  by is nothing at all. A push that lands one place of the float short of
   --  its target (1.1e-16 of an ask of 1) falls short by no more than the
   --  float can tell, and is not blocked; a push stopped half way is. Pushes 1
   --  to 6 go out to 1 and back to 0; push 7 asks 1 and stops one place short;
   --  push 8 asks 0 and is stopped half way.
   procedure Step_Short_By_Rounding is
      M    : Model;
      O    : Observation;
      Sent : Driver.Commands.Command;
      Target, Reading, From : Real := 0.0;
   begin
      for B in 0 .. 120 loop
         if B in 66 .. 105 then
            declare
               K     : constant Natural := (B - 66) / 5 + 1;
               Phase : constant Natural := (B - 66) mod 5;
            begin
               if Phase = 0 then
                  Target := (if K mod 2 = 1 then 1.0 else 0.0);
                  From := Reading;
               elsif Phase = 1 then
                  Reading := (if K = 7 then Real'Pred (1.0)
                              elsif K = 8 then From + 0.5 * (Target - From)
                              else Target);
               end if;
            end;
         end if;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Driver.Robot.Channels.Noise (M, 1, 1) = 0.0, "the exactly repeating reading's noise is zero");
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         for K in 1 .. 7 loop
            declare
               E : constant Episode := M.Groups (1).Episodes (K);
            begin
               Check (E.Ended and then not E.Blocked,
                      "push" & K'Image & " stopped short by" & Real'Image (E.Shortfall.Value)
                      & ", no more than the float can tell, and is called blocked");
            end;
         end loop;
         Check (M.Groups (1).Episodes (8).Blocked, "a push stopped half way is not called blocked");
      end if;
   end Step_Short_By_Rounding;

   procedure Step_Responses is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      --  Pushes every twelve beats. The reading answers two beats after a push
      --  and settles three beats later, 0.05 short of the target along the
      --  ask with a scatter of 0.001, as a servo's dead band leaves it, which
      --  is many times the readings' noise. Pushes 1 to 5 and 7 move freely; push 6
      --  meets an obstacle at 2.0; push 8, asking more than any push the
      --  joint executed (a push within a servo's dead band is not judged,
      --  Steps.Executed_Least), is never answered.
      Targets : constant Real_Array (1 .. 8) := [1.0, 2.0, 1.0, 2.0, 1.0, 3.0, 1.0, 2.5];
      Stop_At : constant Real := 2.0;
      Target, Reading, From, To : Real := 0.0;
      Blocked_From : Real := 0.0;   --  where push 6 started
   begin
      for B in 0 .. 109 loop
         declare
            K     : constant Natural := (if B >= 10 then (B - 10) / 12 + 1 else 0);
            Phase : constant Natural := (if B >= 10 then (B - 10) mod 12 else 0);
         begin
            if K in Targets'Range then
               if Phase = 0 then
                  Target := Targets (K);
                  From := Reading;
                  To := Target - (if Target > From then 1.0 else -1.0) * (0.05 + 0.001 * Gaussian (Rng));
                  if K = 6 then
                     To := Stop_At;
                     Blocked_From := From;
                  elsif K = 8 then
                     To := From;
                  end if;
               elsif Phase in 2 .. 4 then
                  Reading := From + (To - From) * (case Phase is when 2 => 0.5, when 3 => 0.9, when others => 1.0);
               end if;
            end if;
         end;
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => Reading + 1.0e-4 * Gaussian (Rng)));
         Sent := Driver.Commands.Hold;
         Driver.Commands.Set_Target (Sent, 1, [Target]);
         Observe (M, O, Sent);
      end loop;
      Check (Steps.Episodes (M, 1) = 8, "eight pushes, got" & Steps.Episodes (M, 1)'Image);
      if Steps.Episodes (M, 1) = 8 then
         declare
            E : array (1 .. 8) of Episode;
         begin
            for K in E'Range loop
               E (K) := M.Groups (1).Episodes (K);
               Check (E (K).Ended, "push" & K'Image & " is over");
            end loop;
            Check (E (1).Moved and then E (1).Moved_At - E (1).Start = 2, "the first push answers after two beats");
            for K in 1 .. 5 loop
               Check (not E (K).Blocked, "a push that falls short as free pushes do is free: push" & K'Image);
            end loop;
            Check (E (6).Blocked, "a push stopped by an obstacle is blocked");
            Check_Close (E (6).Delivered.Value, (Stop_At - Blocked_From) / (3.0 - Blocked_From), 0.01,
                         "how much of the blocked push came through");
            Check (not E (7).Blocked, "a free push after the obstacle is free again");
            Check (E (8).Blocked and then not E (8).Moved, "a push nothing answers is blocked");
            Check (E (8).End_At - E (8).Start = 3, "an unanswered push is given up after the longest delay, at"
                   & Natural'Image (E (8).End_At - E (8).Start));
         end;
      end if;
   end Step_Responses;

   --  ── Boot ──
   --
   --  The rig as a robot: each beat the decider's command (holds keep the last
   --  target) is reached at once, the images show the beat before, and the
   --  test plays the main loop, offering every beat until the decider takes it.

   --  The rig booted from zero by Boot.Run. With Settling, eye 1's picture
   --  keeps changing after every move of arm 1 the way a rendered view does:
   --  a flicker on a tenth of its pixels, flipping sign every beat, decaying
   --  by 30 % a beat to nothing from up to 40 luma levels (half that after a
   --  move of 0.01, a twentieth of a pixel, and the more the longer the move).
   procedure Boot_On_Rig
     (M : in out Model; Settling : Boolean; Done, Ok : out Boolean; Beats : out Natural; Still_Poses : out Natural;
      Arm_2_Poses : out Natural; Eye_2_Noise : Real := 0.0; Eye_2_Lag : Positive := 1;
      Eye_2_Near : Real := 1.0; Eye_2_Mid : Real := 1.0; Eye_2_Far : Real := 1.0;
      File : String := ""; Stop_Once_Kept : Boolean := False; Delivers : Real := 1.0;
      Tick : Real := 0.0; Deadband : Natural := 0; Speed : Natural := 0)
   is
      --  File is the body file the boot is given. With Stop_Once_Kept, the main
      --  loop stops answering a few beats after the file first exists, as if
      --  the boot had died there, and the decider is aborted. The arms' joints
      --  read Delivers of what they are sent (1: exactly it; a body under load,
      --  or whose command and reading scales differ, reads less).
      --  Tick > 0 puts servos on the arms as the SO-101's on the Pi (R1 to R4, 10-13): the wire carries the arms'
      --  readings and targets as whole ticks of Tick rig units each (the integer the driver reads as exact), a
      --  target within Deadband ticks of the reading moves the servo nothing (it stands two to four ticks off a
      --  goal and the first levels of a ladder are swallowed), and a servo moves at most Speed ticks a beat
      --  (0: at once). The rig's own state stays in its units.
      function Ticked (X : Real_Array) return Real_Array is
        (if Tick > 0.0 then [for V of X => Real'Rounding (V / Tick)] else X);
      function Unticked (X : Real_Array) return Real_Array is
        (if Tick > 0.0 then [for V of X => V * Tick] else X);
      --  What an arm's servos stand at a beat after they were sent Target (in the wire's units), from Reading.
      function Served (Reading, Target : Real_Array) return Real_Array is
         Goal   : constant Real_Array := [for V of Unticked (Target) => Delivers * V];
         Result : Real_Array := Reading;
      begin
         for I in Result'Range loop
            declare
               Gap : constant Real := Goal (Goal'First + I - Result'First) - Reading (I);
            begin
               if Tick = 0.0 then
                  Result (I) := Goal (Goal'First + I - Result'First);
               elsif abs Gap > Real (Deadband) * Tick then
                  Result (I) :=
                    (if Speed = 0 or else abs Gap <= Real (Speed) * Tick then Goal (Goal'First + I - Result'First)
                     else Reading (I) + Real (Speed) * Tick * (if Gap > 0.0 then 1.0 else -1.0));
               end if;
            end;
         end loop;
         return Result;
      end Served;
      Since_Kept : Natural := 0;
      --  What each beat showed, for an eye that lags more than one beat.
      History   : array (0 .. Eye_2_Lag - 1) of Rig_State;
      Noise_Rng : Generator;
      --  The same for the arm of group 2, whatever its number.
      Poses_2   : array (1 .. 1_000) of Real_Array (1 .. 2) := [others => [0.0, 0.0]];
      --  The poses of arm 1 away from rest at which it could give a keyframe
      --  (Kinematics.Held_Still): the rig has no instrument, so the keyframes
      --  themselves stop after the first match is refused.
      Max_Poses : constant := 1_000;
      Poses     : array (1 .. Max_Poses) of Real_Array (1 .. 2) := [others => [0.0, 0.0]];
      H        : Driver.Robot.Hand.Hands;
      Finished : Boolean := False with Atomic;
      Fine_Run : Boolean := False with Atomic;

      task Decider;
      task body Decider is
         Fine : Boolean;
      begin
         Boot.Run (M, H, File, Fine);
         Fine_Run := Fine;
         Finished := True;
      exception
         when others =>
            Driver.Beats.Release;
            Finished := True;
      end Decider;

      Now, Shown : Rig_State;
      Sent  : Driver.Commands.Command;
      Since : Natural := Natural'Last;   --  beats since arm 1 last moved in eye 1's picture
      Trail : Real := 0.0;               --  how much the picture flickers after that move
      --  As many beats as the boot may take: every channel of the rig probed
      --  from the resolution of one reading unit, pushed both ways, and swept.
      Bound : constant := 40_000;
      --  Per beat: eye 1's picture changed by a level or more (the flicker's step from the beat before: what a byte
      --  picture can show), as the rig made it. The flicker's remaining offset is no change.
      Changing : array (0 .. Bound) of Boolean := [others => False];
   begin
      Beats := 0;
      Still_Poses := 0;
      Unsettled_Stills := 0;
      Arm_2_Poses := 0;
      begin
      for B in 0 .. Bound loop
         if Stop_Once_Kept and then GNAT.OS_Lib.Is_Regular_File (File) then
            Since_Kept := Since_Kept + 1;
         end if;
         exit when Finished or else Since_Kept > 20;
         declare
            O       : Observation;
            Took    : Boolean := False;
            Pending : Driver.Commands.Command;
         begin
            O.Beat := Driver.Clock.Beat (B);
            --  The finger patch is drawn where it started: the closer moves
            --  nothing an eye sees, so no hand is measured (the hand's measure
            --  has its own tests and needs the instrument).
            declare
               Drawn : Rig_State := Shown;
            begin
               Drawn.Closer := 0.0;
               for E in 1 .. 3 loop
                  declare
                     Picture : Driver.Images.Image :=
                       Render (E, (if E = 2 and then Eye_2_Lag > 1 then History (B mod Eye_2_Lag) else Drawn),
                               (if E = 2 then Eye_2_Near else 1.0), (if E = 2 then Eye_2_Mid else 1.0),
                               (if E = 2 then Eye_2_Far else 1.0));
                  begin
                     if Settling and then E = 1 and then Since < Natural'Last then
                        declare
                           use type Driver.Bytes.Offset;
                           Flicker : constant Real := Trail * 0.7 ** Since;
                           Data    : Driver.Bytes.Byte_Array (1 .. 3 * Rig_Width * Rig_Height);
                        begin
                           for Y in 0 .. Rig_Height - 1 loop
                              for X in 0 .. Rig_Width - 1 loop
                                 declare
                                    K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Rig_Width + X) + 1);
                                    L : constant Real := Real (Driver.Images.Red (Picture, X, Y))
                                      + (if (X * 7 + Y * 13) mod 10 /= 0 then 0.0
                                         elsif (B + X) mod 2 = 0 then Flicker else -Flicker);
                                    V : constant Driver.Bytes.Byte :=
                                      Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                                 begin
                                    Data (K) := V;
                                    Data (K + 1) := V;
                                    Data (K + 2) := V;
                                 end;
                              end loop;
                           end loop;
                           Picture := Driver.Images.Create (Rig_Width, Rig_Height, Data);
                        end;
                     end if;
                     if E = 2 and then Eye_2_Noise > 0.0 then
                        declare
                           use type Driver.Bytes.Offset;
                           Data : Driver.Bytes.Byte_Array (1 .. 3 * Rig_Width * Rig_Height);
                        begin
                           for Y in 0 .. Rig_Height - 1 loop
                              for X in 0 .. Rig_Width - 1 loop
                                 declare
                                    K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Rig_Width + X) + 1);
                                    L : constant Real := Real (Driver.Images.Red (Picture, X, Y)) + Eye_2_Noise * Gaussian (Noise_Rng);
                                    V : constant Driver.Bytes.Byte :=
                                      Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, L))));
                                 begin
                                    Data (K) := V;
                                    Data (K + 1) := V;
                                    Data (K + 2) := V;
                                 end;
                              end loop;
                           end loop;
                           Picture := Driver.Images.Create (Rig_Width, Rig_Height, Data);
                        end;
                     end if;
                     O.Images.Append (Picture);
                  end;
                  O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
               end loop;
            end;
            O.Readings.Append (Ticked (Now.Arm_1));
            O.Readings.Append (Ticked (Now.Arm_2));
            O.Readings.Append (Real_Array'(1 => Now.Closer));
            O.Readings.Append (Real_Array'(1 => Now.Part));
            O.Readings.Append (Real_Array'(1 => Now.Idle));
            O.Readings.Append (Ticked (Real_Array'(1 => Now.Arm_1 (1) + Now.Arm_2 (1))));
            O.Readings.Append (Real_Array'(1 => 7.0));
            for G in 1 .. 7 loop
               O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            end loop;
            if Tick > 0.0 then
               --  The wire's formats: the arms and the sum sensor in whole ticks, the rest floats.
               for G in 1 .. 7 loop
                  O.Precisions.Append (if G in 1 | 2 | 6 then 0 else Real'Machine_Mantissa);
               end loop;
            end if;
            if B = 0 then
               Driver.Commands.Set_Target (Sent, 1, Ticked (Now.Arm_1));
               Driver.Commands.Set_Target (Sent, 2, Ticked (Now.Arm_2));
               Driver.Commands.Set_Target (Sent, 3, [Now.Closer]);
               Driver.Commands.Set_Target (Sent, 4, [Now.Part]);
               Driver.Commands.Set_Target (Sent, 5, [Now.Idle]);
            end if;
            Observe (M, O, Sent);
            Driver.Robot.Hand.Observe (H, M, O, Sent);
            if Arm_Count (M) >= 1 and then Now.Arm_1 /= [0.0, 0.0]
              and then Driver.Robot.Kinematics.Held_Still (M, 1, M.Beats - 1)
              and then (for all K in 1 .. Still_Poses => Poses (K) /= Now.Arm_1)
              and then Still_Poses < Max_Poses
            then
               Still_Poses := Still_Poses + 1;
               Poses (Still_Poses) := Now.Arm_1;
            end if;
            Changing (B) := Settling and then Since < Natural'Last
              and then (if Since = 0 then Trail else Trail * 0.3 * 0.7 ** (Since - 1)) >= 1.0;
            --  Arm 2 is swept with its eye where the model has eye 2 on it: a pose counted while the estimate had eye
            --  2 on another arm, or on none, is a pose of a group the boot was not sweeping with that eye (a beat of the
            --  first probe at which the lock-in, its lag not yet told, briefly read eye 2 whole on group 2 passed this
            --  count for a boot that never swept arm 2, robot.boot.reread on 10-17).
            for A in 1 .. Arm_Count (M) loop
               if Arm_Group (M, Arm_Id (A)) = 2 and then Now.Arm_2 /= [0.0, 0.0]
                 and then Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = Arm_Id (A)
                 and then Driver.Robot.Kinematics.Held_Still (M, Arm_Id (A), M.Beats - 1)
                 and then (for all K in 1 .. Arm_2_Poses => Poses_2 (K) /= Now.Arm_2)
                 and then Arm_2_Poses < Poses_2'Length
               then
                  Arm_2_Poses := Arm_2_Poses + 1;
                  Poses_2 (Arm_2_Poses) := Now.Arm_2;
               end if;
            end loop;
            loop
               Driver.Beats.Offer (O.Beat, O, Sent, Took);
               exit when Took or else Finished;
               delay 0.0;
            end loop;
            exit when not Took;
            Driver.Beats.Await (Pending);
            --  The robot reaches what it was sent; a group without a target holds.
            for G in Group_Id range 1 .. 5 loop
               if Driver.Commands.Has_Target (Pending, G) then
                  Driver.Commands.Set_Target (Sent, G, Driver.Commands.Target (Pending, G));
               end if;
            end loop;
            --  Eye 1's next picture shows Now; it settles from a move of arm 1.
            if Now.Arm_1 /= Shown.Arm_1 then
               --  The longer the move, the more the picture flickers after it.
               declare
                  D : constant Real := Real'Max (abs (Now.Arm_1 (1) - Shown.Arm_1 (1)), abs (Now.Arm_1 (2) - Shown.Arm_1 (2)));
               begin
                  Trail := 40.0 * D / (D + 0.01);
               end;
               Since := 0;
            elsif Since < Natural'Last then
               Since := Since + 1;
            end if;
            History (B mod Eye_2_Lag) := Now;
            Shown := Now;
            Now.Arm_1 := Served (Now.Arm_1, Driver.Commands.Target (Sent, 1));
            Now.Arm_2 := Served (Now.Arm_2, Driver.Commands.Target (Sent, 2));
            Now.Closer := Driver.Commands.Target (Sent, 3) (1);
            Now.Part := Driver.Commands.Target (Sent, 4) (1);
            Now.Idle := Driver.Commands.Target (Sent, 5) (1);
            Beats := B + 1;
         end;
      end loop;
      exception
         when others =>
            --  A failure on the main side must not leave the decider waiting.
            abort Decider;
            raise;
      end;
      if not Finished then
         abort Decider;
      end if;
      --  The keyframes arm 1 was given, each at the beat its picture was taken: one taken while the picture still
      --  changed was held on a picture still changing. Counted from the keyframes once the boot is over, not from
      --  the model while the decider ran: the rig's main side read Held_Still between the decider's estimates and
      --  counted beats no keyframe was taken at, one of them with the eye's own watch unsettled (robot.boot.settling,
      --  10-17, beat 916, the keyframes at 899 and 924).
      for R of M.Kinematics loop
         if R.Arm = 1 then
            for F of R.Frames loop
               if F.Beat in Changing'Range and then Changing (F.Beat) then
                  Unsettled_Stills := Unsettled_Stills + 1;
                  Driver.Log.Line (Driver.Log.Robot, "rig: arm 1 keyframe at beat" & F.Beat'Image
                                   & " taken while its picture still changed");
               end if;
            end loop;
         end if;
      end loop;
      Done := Finished;
      Ok := Fine_Run;
   end Boot_On_Rig;

   procedure Boot_From_Zero is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses : Natural;
      Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2);
      Check (Done, "the boot did not finish");
      --  The rig's idle group takes commands and moves nothing: the boot must
      --  name the clause it breaks and hold still.
      Check (not Ok and then Contract_Breach (M, 5) = 2, "the boot did not report the idle group as breaking clause 2");
      Check (Role (M, 5) = Inert, "the idle group is inert, got " & Role (M, 5)'Image);
      Check (Contract_Breach (M, 3) = 2, "the boot did not report the frozen closer as breaking clause 2");
      Check (Eye_Mount (M, 1).Kind = Arm_Carried and then Eye_Mount (M, 1).Arm = 1, "eye 1 rides on arm 1");
      Check (Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = 2, "eye 2 rides on arm 2");
      Check (Eye_Mount (M, 3).Kind = World_Fixed, "eye 3 is fixed");
      Check (Role (M, 1) = Arm and then Role (M, 2) = Arm, "the boot recognized both arms, got "
             & Role (M, 1)'Image & " and " & Role (M, 2)'Image);
      Check (Role (M, 4) = Part, "the boot recognized the part, got " & Role (M, 4)'Image);
      Check (Known (Visible_Step (M, 1, 1)), "the boot measured how far arm 1 must move to be seen");
      --  The fixed eye 3 saw both arms: the arms' edge flow judged it (Driver.Robot.Flows), told or not, when an arm
      --  was fitted to place it from (the rig's arms are not: its pictures' patches are too small for the aligner).
      Check ((for some A in 1 .. 2 => Driver.Robot.Kinematics.Fitted (M, Arm_Id (A)))
             = (Driver.Robot.Kinematics.Fixed_Why (M, 3) /= "not measured yet"),
             "the fixed eye 3 was judged with no arm fitted, or not judged with one");
      Driver.Log.Line (Driver.Log.Robot, "boot from zero: eye 3 "
                       & (if Driver.Robot.Kinematics.Fixed_Known (M, 3) then "placed"
                          else "not placed: " & Driver.Robot.Kinematics.Fixed_Why (M, 3)));
      Driver.Log.Line (Driver.Log.Robot, "boot from zero took" & Beats'Image & " beats");
   end Boot_From_Zero;

   --  The rig's boot with the arms' joints reading nine tenths of what they
   --  are sent. The sweep sizes a joint's first push by its keyframe step (the
   --  readings' noise is nil here, as a simulator's are), so the pose it rests
   --  at is short of the step, not apart from the reference by it, and no
   --  keyframe comes of it. The ladder must climb there, not stop: B0_2 (x5,
   --  10-12) pushed arm 1's channels 1, 2 and 5 by exactly their step, the
   --  step was re-estimated higher before the poses were judged, no keyframe
   --  came of them, the ladder took them for unfollowed, and the arm never
   --  fitted. Every joint of both arms must have a keyframe at least its step
   --  from the reference.
   --  The hold after a recognition push: the longest measured lag once every
   --  eye's lag is measured, whatever the doubled guess has grown to; the
   --  guess until then.
   procedure Pause_Follows_The_Measured_Lag is
      use Driver.Robot.Boot;
   begin
      Check (Pause_After_A_Push (Every_Lag_Known => True, Longest_Lag => 1, Guess => 384) = 1,
             "with every lag measured at 1 beat the hold is 1 beat, not the doubled guess of 384");
      Check (Pause_After_A_Push (Every_Lag_Known => True, Longest_Lag => 10, Guess => 12) = 10,
             "with an eye measured 10 beats behind the hold is 10 beats");
      Check (Pause_After_A_Push (Every_Lag_Known => True, Longest_Lag => 0, Guess => 7) = 0,
             "with every lag measured at 0 the hold is 0: the settle alone decides");
      Check (Pause_After_A_Push (Every_Lag_Known => False, Longest_Lag => 1, Guess => 24) = 24,
             "with a lag still unmeasured the hold is the guess");
   end Pause_Follows_The_Measured_Lag;

   --  The longest an eye's picture has taken to settle after a move, from the
   --  per-beat flags: moves at beats 10 and 30, the eye settled at 14 and 31,
   --  a move at 50 not settled yet (not counted), and a second move at 70
   --  before the first settled (the spell counts from the later move).
   procedure Longest_Settling_Of_An_Eye is
      M : Model;
      S : Eye_Stream;
   begin
      for B in 0 .. 80 loop
         M.Began_Moving.Append (B = 10 or else B = 30 or else B = 50 or else B = 70 or else B = 72);
         S.Settled_At.Append (B in 14 .. 29 or else B in 31 .. 49 or else B in 75 .. 80);
      end loop;
      M.Eyes.Append (S);
      Check (Longest_Settling (M, 1) = 4,
             "the longest settling is" & Longest_Settling (M, 1)'Image & ", not 4 (beats 10 to 14)");
      Check (Longest_Settling (M, 2) = 0, "an eye the model does not have settled in" & Longest_Settling (M, 2)'Image);
      M.Eyes (1).Settled_At.Replace_Element (73, True);
      Check (Longest_Settling (M, 1) = 4,
             "a move at 72 settled at 73 counts from the later move: longest" & Longest_Settling (M, 1)'Image);
   end Longest_Settling_Of_An_Eye;

   --  The hold after a push for one group: only the eyes that see the group
   --  move count. Two eyes, group 1 seen whole by eye 1 (lag measured at 1)
   --  and not at all by eye 2 (lag unmeasured): the hold is 1, not the
   --  doubled guess; eye 2 seeing a patch of it while unmeasured: the guess;
   --  eye 2 measured at 10 beats: 10.
   procedure Hold_Counts_The_Seeing_Eyes is
      use Driver.Robot.Boot;
      M : Model;
      S : Eye_Stream;
      function Seen (V : Eye_Response) return Eye_Effect is
        ((Verdict => V, Responding => 1, Textured => 1,
          Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
   begin
      M.Eyes.Append (S);
      M.Eyes.Append (S);
      M.Lags.Append (1);
      M.Lag_Known.Append (True);
      M.Lags.Append (0);
      M.Lag_Known.Append (False);
      M.Graph.Effects.Append (Seen (Whole));
      M.Graph.Effects.Append (Seen (Nothing));
      Check (Hold_After_A_Push (M, 1, 24) = 1,
             "an eye that sees nothing of the group, unmeasured, kept the hold at the guess:"
             & Hold_After_A_Push (M, 1, 24)'Image);
      M.Graph.Effects.Replace_Element (2, Seen (Patch));
      Check (Hold_After_A_Push (M, 1, 24) = 24,
             "an eye that sees a patch of the group, unmeasured, did not keep the hold at the guess:"
             & Hold_After_A_Push (M, 1, 24)'Image);
      M.Lags.Replace_Element (2, 10);
      M.Lag_Known.Replace_Element (2, True);
      Check (Hold_After_A_Push (M, 1, 24) = 10,
             "two seeing eyes measured at 1 and 10 beats: the hold is 10, got" & Hold_After_A_Push (M, 1, 24)'Image);
   end Hold_Counts_The_Seeing_Eyes;

   procedure Boot_With_Short_Deliveries is
      M       : Model;
      Done    : Boolean;
      Ok      : Boolean;
      Beats   : Natural;
      Poses   : Natural;
      Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Delivers => 0.9);
      Check (Done, "the boot did not finish");
      Check (Role (M, 1) = Arm and then Role (M, 2) = Arm, "the boot recognized both arms, got "
             & Role (M, 1)'Image & " and " & Role (M, 2)'Image);
      for E of M.Kinematics loop
         if E.Arm in 1 .. 2 then
            Check (not E.Frames.Is_Empty, "arm" & E.Arm'Image & " has no keyframe at all");
            if not E.Frames.Is_Empty then
               for C in 1 .. 2 loop
                  declare
                     Step     : constant Real := Driver.Robot.Kinematics.Keyframe_Step (M, E.Arm, C);
                     Farthest : Real := 0.0;
                  begin
                     for F of E.Frames loop
                        Farthest := Real'Max (Farthest, abs (F.Readings (C - 1) - E.Frames.First_Element.Readings (C - 1)));
                     end loop;
                     Check (Step > 0.0 and then Farthest >= Step,
                            "arm" & E.Arm'Image & " channel" & C'Image & " has no keyframe as far as its step from the"
                            & " reference: farthest " & Driver.Log.Image (Farthest, 6) & ", step "
                            & Driver.Log.Image (Step, 6) & " (the first push, nine tenths of the step, left no"
                            & " keyframe and the ladder stopped there)");
                  end;
               end loop;
            end if;
         end if;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "boot with short deliveries took" & Beats'Image & " beats");
   end Boot_With_Short_Deliveries;

   --  The rig's boot with the SO-101's servos on its arms: readings and targets in whole ticks, a deadband of
   --  three ticks and forty ticks a beat (the Pi's arms, R1 to R4, 10-13: the deadband swallowed the ladder's
   --  first levels, the pushes into stops and the jaws' creep were taken for responses hundreds of beats late,
   --  and every look waited that long). The boot must recognize both arms with their eyes, and the delay it
   --  measures must be what a servo takes to answer, not what a stop or a creep took.
   procedure Boot_With_Servos is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses, Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Tick => 2.0 * Ada.Numerics.Pi / 4096.0, Deadband => 3, Speed => 40);
      Check (Done, "the boot did not finish");
      Check (Role (M, 1) = Arm and then Role (M, 2) = Arm, "the boot recognized both arms, got "
             & Role (M, 1)'Image & " and " & Role (M, 2)'Image);
      Check (Eye_Mount (M, 1).Kind = Arm_Carried and then Eye_Mount (M, 1).Arm = 1, "eye 1 rides on arm 1");
      Check (Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = 2, "eye 2 rides on arm 2");
      for G in Group_Id range 1 .. 2 loop
         Check (M.Groups (G).Delay_Known and then M.Groups (G).Delay_Beats <= 2,
                "group" & G'Image & " answers within the beat a servo takes, got "
                & (if M.Groups (G).Delay_Known then M.Groups (G).Delay_Beats'Image else " unknown"));
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "boot with servos took" & Beats'Image & " beats, arm 1 swept at" & Poses'Image
                       & " poses, arm 2 at" & Poses_2'Image);
   end Boot_With_Servos;

   --  The rig's boot with arm 2's eye noisy (luma noise of 2 in its every
   --  pixel): pushed by the amount another eye first sees it at, arm 2 leaves
   --  its own eye undecided (31 of 48 cells), as a live x5's right arm left
   --  its wrist's eye (267 of 525). Undecided is too little evidence: the
   --  boot pushes the group again at twice its amounts until the eye decides,
   --  and sweeps arm 2. (Without that, the eye stays undecided to the end, 32
   --  of 48, and arm 2 is never swept.)
   procedure Boot_With_An_Undecided_Eye is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses, Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Eye_2_Noise => 2.0);
      Check (Done, "the boot did not finish");
      Check (Role (M, 2) = Arm and then Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = 2,
             "arm 2 carrying its noisy eye is not recognized: " & Role (M, 2)'Image & ", eye 2 "
             & Eye_Mount (M, 2).Kind'Image);
      Check (Poses_2 > 0, "arm 2 was never swept with its noisy eye");
   end Boot_With_An_Undecided_Eye;

   --  An eye that rides on a turning joint sees its view move by different amounts in different cells (by depth, by
   --  perspective). The rig's eye 2 shows a near object, the right quarter of its view, move about seventeen times as
   --  much as a middle distance (the middle half) and a hundred times as much as the far background (the rest).
   --  Pushed by the amount some eye first sees arm 2 at, only the cells that move most respond (13 of 48), and the
   --  eye shows a patch of its view; pushed harder, the middle distance and then the background do too, until it is
   --  the whole. The boot must not take the patch for an answer, as it did A17's right arm: the same first push gave
   --  134 of 525 cells, undecided, in A16 and 132, a patch, in A17; A16's arm was pushed again and came to 282, the
   --  whole, while A17's was left a part and never swept. A patch that grows with the push is pushed on.
   procedure Boot_With_A_Patch_That_Grows is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses, Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Eye_2_Mid => 0.06, Eye_2_Far => 0.01);
      Check (Done, "the boot did not finish");
      Driver.Log.Line (Driver.Log.Robot, "boot with a patch that grows: eye 2 shows arm 2 as "
                       & Response (M, 2, 2)'Image & ", " & Responding (M, 2, 2)'Image & " of"
                       & Textured_Cells (M, 2, 2)'Image & " cells, arm 2 swept at" & Poses_2'Image & " poses");
      Check (Role (M, 2) = Arm and then Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = 2,
             "arm 2 carrying an eye whose view moves by different amounts is not recognized: " & Role (M, 2)'Image
             & ", eye 2 " & Eye_Mount (M, 2).Kind'Image & ", " & Response (M, 2, 2)'Image & " "
             & Responding (M, 2, 2)'Image & " of" & Textured_Cells (M, 2, 2)'Image);
      Check (Poses_2 > 0, "arm 2 was never swept with its eye");
   end Boot_With_A_Patch_That_Grows;

   --  A view whose far part moves twenty times less than its near part, with nothing between. Pushed by the amount some
   --  eye first sees arm 2 at, 13 of the rig's 48 cells respond (the near object, a quarter of the view); pushed at
   --  twice that, 15: two more than the false alarms among the 35 that did not could make is not growth (p 0.004
   --  against 0.0027), and the far part shows itself in a cell only at four times the amounts. The cells that did not
   --  respond move all the same, and together they show it at the first push (Resting_Motion): the boot pushes on,
   --  finds the whole at four times and sweeps arm 2. Without that test (the growth of the responding cells alone) it
   --  stopped at 15 with arm 2 a part.
   procedure Boot_With_A_Step_Wider_Than_A_Doubling is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses, Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Eye_2_Mid => 0.05, Eye_2_Far => 0.05);
      Check (Done, "the boot did not finish");
      Driver.Log.Line (Driver.Log.Robot, "boot with a step wider than a doubling: eye 2 shows arm 2 as "
                       & Response (M, 2, 2)'Image & ", " & Responding (M, 2, 2)'Image & " of"
                       & Textured_Cells (M, 2, 2)'Image & " cells, the rest moving together by"
                       & Real'Image (Resting_Motion (M, 2, 2)) & " sigmas, arm 2 swept at" & Poses_2'Image & " poses");
      Check (Role (M, 2) = Arm and then Eye_Mount (M, 2).Kind = Arm_Carried and then Eye_Mount (M, 2).Arm = 2,
             "arm 2 carrying an eye whose far part moves twenty times less is not recognized: " & Role (M, 2)'Image
             & ", eye 2 " & Eye_Mount (M, 2).Kind'Image & ", " & Response (M, 2, 2)'Image & " "
             & Responding (M, 2, 2)'Image & " of" & Textured_Cells (M, 2, 2)'Image);
      Check (Poses_2 > 0, "arm 2 was never swept with its eye");
   end Boot_With_A_Step_Wider_Than_A_Doubling;

   --  When the cells that respond have grown. A patch of 13 of 48 cells that is 17 after the push is doubled has: the
   --  four new are more than the false alarms among the 35 that did not respond could make; 15 has not (two are).
   --  A16's arm 2 in eye 3, 134 of 525 undecided, was 282 pushed again; 132 and 135 is a patch that is one. A count
   --  that falls, or does not change, or fills the eye, has not grown beyond what there was room for.
   procedure Growth_Of_The_Responding_Cells is
   begin
      Check (Boot.Grew (13, 17, 48), "13 of 48 cells become 17 and have not grown");
      Check (not Boot.Grew (13, 15, 48), "13 of 48 cells become 15 and have grown");
      Check (Boot.Grew (134, 282, 525), "134 of 525 cells become 282 and have not grown");
      Check (not Boot.Grew (132, 135, 525), "132 of 525 cells become 135 and have grown");
      Check (Boot.Grew (13, 48, 48), "13 of 48 cells become all 48 and have not grown");
      Check (not Boot.Grew (4, 4, 48), "4 of 48 cells stay 4 and have grown");
      Check (not Boot.Grew (17, 13, 48), "17 of 48 cells become 13 and have grown");
      Check (not Boot.Grew (48, 48, 48), "all 48 of 48 cells have grown");
   end Growth_Of_The_Responding_Cells;

   --  The rig's boot with arm 2's eye ten beats behind its readings, more
   --  than the stretch between pushes at first: until the lag can be told,
   --  that eye's motion is credited to arm 1, so at the first reading of the
   --  body arm 1 carries both eyes and arm 2 none; the estimate after the
   --  sweeps has eye 2 on arm 2. The boot follows the current estimate, not
   --  the first reading: arm 2 is swept with its eye, and arm 1 again with
   --  its own. (A boot that swept the arms read at first never sweeps arm 2.
   --  Where the lagging eye ends up after arm 2's own sweep is the lag's
   --  business, not the sweep's.)
   procedure Boot_With_A_Late_Mount is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Poses, Poses_2 : Natural;
   begin
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, Eye_2_Lag => 10);
      Check (Done, "the boot did not finish");
      Check (Poses_2 > 0, "arm 2 was never swept with its eye");
   end Boot_With_A_Late_Mount;

   --  The rig's boot with eye 1's picture settling for beats after every move
   --  of arm 1: every level held for its keyframe (every other one, the first
   --  and the widest included) and every cell is held until the picture has
   --  stopped, so each of them gives arm 1 a keyframe.
   procedure Boot_With_Settling_Views is
      M     : Model;
      Done  : Boolean;
      Ok    : Boolean;
      Beats : Natural;
      Levels : Natural := 0;   --  the sweep's single-joint levels of arm 1 held for keyframes, both ways
      Poses  : Natural;
      Poses_2 : Natural;
   begin
      Boot_On_Rig (M, True, Done, Ok, Beats, Poses, Poses_2);
      Check (Done, "the boot did not finish");
      declare
         Half : constant Real := Real (Natural'Min (M.Eyes (1).Grid.Width, M.Eyes (1).Grid.Height)) / 2.0;
      begin
         for C in 1 .. 2 loop
            declare
               First  : constant Real := Driver.Robot.Motion.Sweep_Start (M, 1, C);
               Shift  : constant Real := Driver.Robot.Kinematics.Pixels_Per_Unit (M, 1, C);
               Offset : Real := First;
               Level  : Positive := 1;
            begin
               if First > 0.0 and then Shift > 0.0 then
                  while Offset * Shift <= Half loop
                     if Level mod 2 = 1 or else 2.0 * Offset * Shift > Half then
                        Levels := Levels + 2;
                     end if;
                     Offset := 2.0 * Offset;
                     Level := Level + 1;
                  end loop;
               end if;
            end;
         end loop;
      end;
      Driver.Log.Line (Driver.Log.Robot, "boot with settling views took" & Beats'Image & " beats; arm 1 could give"
                       & Poses'Image & " keyframes away from rest for" & Levels'Image & " sweep levels");
      Check (Levels > 0, "arm 1 was not swept");
      Check (Poses >= Levels, "arm 1 could give" & Poses'Image & " keyframes away from rest for" & Levels'Image & " sweep levels");
      --  A keyframe waits for the picture to settle: arm 1 is given none while eye 1's picture still flickers by a
      --  level or more after its move (the rig counts those). The same boot with pictures
      --  that settle at once finishes too. (The sweep once visited every other level and passed the rest before
      --  the picture settled, so that this boot held fewer poses than the ideal one; now every level is held until
      --  the picture settles, and the counts tell nothing.)
      Check (Unsettled_Stills = 0, "arm 1 was given" & Unsettled_Stills'Image
             & " keyframes at beats where its picture still changed by a level or more");
      declare
         Ideal                                 : Model;
         Ideal_Done, Ideal_Ok                  : Boolean;
         Ideal_Beats, Ideal_Poses, Ideal_Poses_2 : Natural;
      begin
         Boot_On_Rig (Ideal, False, Ideal_Done, Ideal_Ok, Ideal_Beats, Ideal_Poses, Ideal_Poses_2);
         Check (Ideal_Done, "the boot with pictures that settle at once did not finish");
         Check (Ideal_Poses > 0, "arm 1 was never held still for a keyframe when pictures settle at once");
      end;
   end Boot_With_Settling_Views;

   --  A boot keeps what it measured when it fails later. The rig's boot is
   --  stopped a few beats after its body file first exists, as if it had died
   --  there, in the sweeps that follow the recognition: the file holds the
   --  body recognized by then, and a model that reloads it has the groups'
   --  roles and the eyes' mounts without measuring them again. (A boot that
   --  writes its file only when it ends never gets here: it finishes.)
   procedure Boot_Keeps_What_It_Measured is
      M, Back : Model;
      Done, Ok : Boolean;
      Beats, Poses, Poses_2 : Natural;
      FD    : GNAT.OS_Lib.File_Descriptor;
      Name  : GNAT.OS_Lib.String_Access;
      Gone  : Boolean;
      Why   : Ada.Strings.Unbounded.Unbounded_String;
      use type GNAT.OS_Lib.File_Descriptor;
   begin
      GNAT.OS_Lib.Create_Temp_File (FD, Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the body file");
      if FD = GNAT.OS_Lib.Invalid_FD then
         return;
      end if;
      GNAT.OS_Lib.Close (FD);
      --  The body file is the scratch file's name with a suffix, which no one has yet: the boot finds no body file to
      --  reload. The scratch file stays, so that no other selftest makes one of its name meanwhile: the suite runs
      --  them side by side in one folder, each counting its scratch names from the same first, and the scratch file
      --  this test used to delete for its boot to find nothing was the next test's to make, and to write over this
      --  boot's body file (the big machine's suite, 10-22 and 10-23: "not a body file: expected a value at
      --  character 1"; alone it passes).
      declare
         Body_Name : constant String := Name.all & ".body";
      begin
         Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, File => Body_Name, Stop_Once_Kept => True);
         Check (not Done, "the boot finished before a body file was left to stop it at");
         Check (GNAT.OS_Lib.Is_Regular_File (Body_Name),
                "a boot that died during the arms' sweeps left no body file, though it had recognized the body");
         if GNAT.OS_Lib.Is_Regular_File (Body_Name) then
            Load_Body (Back, Body_Name, Ok, Why);
            Check (Ok, "the body file was not loaded: " & Ada.Strings.Unbounded.To_String (Why));
            Check (Reloaded (Back, Stored_Graph), "the recognized body was not reloaded");
            Check (Role (Back, 1) = Arm and then Role (Back, 2) = Arm and then Role (Back, 4) = Part,
                   "the reloaded body has other roles than the boot recognized: " & Role (Back, 1)'Image & ", "
                   & Role (Back, 2)'Image & ", " & Role (Back, 4)'Image);
            Check (Eye_Mount (Back, 1) = Eye_Mount (M, 1) and then Eye_Mount (Back, 2) = Eye_Mount (M, 2),
                   "the reloaded body mounts its eyes elsewhere");
            --  And the turns at which the recognition saw each channel move an eye, from which the sweeps begin: a boot
            --  begun from the file without them swept from the least turn a keyframe counts, which moved no view, and
            --  left those joints as not moving the eye (f16e on 10-15).
            Check ((for some G in 1 .. Group_Count (M) => (for some X of M.Groups (Group_Id (G)).Seen_At => X > 0.0)),
                   "the boot recognized no channel an eye saw move");
            for G in 1 .. Group_Count (M) loop
               declare
                  Was : Real_Vectors.Vector renames M.Groups (Group_Id (G)).Seen_At;
                  Now : Real_Vectors.Vector renames Back.Groups (Group_Id (G)).Seen_At;
               begin
                  Check (Natural (Now.Length) = Natural (Was.Length)
                         and then (for all C in Was.First_Index .. Was.Last_Index =>
                                     abs (Now (C) - Was (C)) <= 1.0E-9 * abs Was (C)),
                         "group" & G'Image & "'s turns its recognition saw it move an eye at were not reloaded");
               end;
            end loop;
            GNAT.OS_Lib.Delete_File (Body_Name, Gone);
         end if;
         Check (not GNAT.OS_Lib.Is_Regular_File (Body_Name & ".part"), "a write left its half behind");
      end;
      GNAT.OS_Lib.Delete_File (Name.all, Gone);
   end Boot_Keeps_What_It_Measured;

   --  The rig of a joint read exactly (noise 1e-13) whose reading settles a
   --  hair off its target, the more the further it goes (by the square of the
   --  offset, as a joint held against a spring does), and that stops at 1e-3:
   --  the fraction of each offset it delivers shrinks with every doubling,
   --  though it follows each one. No eye sees the joint. It is probed from
   --  First (0: from the smallest step its noise tells) after the estimates
   --  have measured its noise, with one more held beat before the probe when
   --  Before says so, in which the model loses the noise (Lost: the probe
   --  measures it again as it begins) or keeps it (Idle_Beat: the same beats
   --  as Lost).
   type Noise_Before is (As_Measured, Idle_Beat, Lost_Noise);

   --  Late is how many beats after a command is in effect the readings show it (0: the beat after it is sent, as
   --  the lab's simulator; 1: a beat later, as the fake robot and a servo bus); Primed says whether the body is
   --  pushed a few times before the probe, so that its delay and the eyes' lag are measured, or the probe's
   --  first level is the body's first push. Delay_Seen is the first push of the joint's wait from its start to
   --  its readings' first motion (-1 when it never moved), Chained how many of the joint's pushes began while it
   --  was still moving from the push before.
   procedure Probe_Drooping_Joint
     (First : Real; Before : Noise_Before; Late : Natural; Primed : Boolean;
      Taken : out Natural; Finished : out Boolean; Delay_Seen : out Integer; Chained : out Natural)
   is
      M     : Model;
      Done  : Boolean := False with Atomic;
      Steps : Natural := 0 with Atomic;
      Limit : constant Real := 1.0e-3;

      task Decider;
      task body Decider is
         W : Natural;
         R : Driver.Robot.Motion.Probe_Report;
         procedure Estimate is
         begin
            Estimate_Now (M);
         end Estimate;
         procedure Keep_Noise is
         begin
            null;
         end Keep_Noise;
         procedure Lose_Noise is
            Index : Natural := 0;   --  group 5's channel after every channel of the groups before it
         begin
            for G in 1 .. 4 loop
               Index := Index + Group_Size (M, Group_Id (G));
            end loop;
            M.Noise.Replace_Element (Index, Real'Last);
            M.Noise_Freedom.Replace_Element (Index, 0);
         end Lose_Noise;
      begin
         --  Long enough at rest for the readings' noise to be measured, and
         --  a few pushes for the joint's delay and the eyes' lag: the probe
         --  looks once the reading has had time to follow and the eyes to
         --  show it.
         Driver.Robot.Motion.Settle (M, W);
         Driver.Robot.Motion.Hold (M, 100);
         if Primed then
            for K in 1 .. 16 loop
               declare
                  C  : Driver.Commands.Command;
                  SR : Driver.Robot.Motion.Step_Report;
               begin
                  Driver.Commands.Set_Target (C, 5, [(if K mod 2 = 1 then 1.0e-6 else 0.0)]);
                  Driver.Robot.Motion.Step (M, C, SR);
                  Driver.Robot.Motion.Hold (M, 2 + K mod 3);
                  --  Arm 1, which eye 1 sees, for the eyes' lag.
                  Driver.Commands.Set_Target (C, 1, [(if K mod 2 = 1 then 0.1 else 0.0), 0.0]);
                  Driver.Robot.Motion.Step (M, C, SR);
                  Driver.Robot.Motion.Hold (M, 2 + K mod 4);
               end;
            end loop;
         end if;
         Driver.Beats.Within_A_Beat (Estimate'Access);
         Driver.Robot.Motion.Gather_Rest (M, 2);
         case Before is
            when As_Measured => null;
            when Idle_Beat   => Driver.Beats.Within_A_Beat (Keep_Noise'Access);
            when Lost_Noise  => Driver.Beats.Within_A_Beat (Lose_Noise'Access);
         end case;
         Driver.Robot.Motion.Probe_Together (M, [1 => (Group => 5, Channel => 1)], 1.0, First, R);
         Steps := R.Steps;
         Done := True;
      exception
         when others =>
            Driver.Beats.Release;
            Done := True;
      end Decider;

      Now, Shown, Due : Rig_State;
      Sent  : Driver.Commands.Command;
      Rng   : Generator;
   begin
      begin
         for B in 0 .. 5_000 loop
            exit when Done;
            declare
               O       : Observation;
               Took    : Boolean := False;
               Pending : Driver.Commands.Command;
            begin
               O.Beat := Driver.Clock.Beat (B);
               for E in 1 .. 3 loop
                  O.Images.Append (Render (E, Shown));
                  O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
               end loop;
               O.Readings.Append (Now.Arm_1);
               O.Readings.Append (Now.Arm_2);
               O.Readings.Append (Real_Array'(1 => Now.Closer));
               O.Readings.Append (Real_Array'(1 => Now.Part));
               O.Readings.Append (Real_Array'(1 => Now.Idle + 1.0e-13 * Gaussian (Rng)));
               for G in 1 .. 5 loop
                  O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
               end loop;
               if B = 0 then
                  Driver.Commands.Set_Target (Sent, 1, Now.Arm_1);
                  Driver.Commands.Set_Target (Sent, 2, Now.Arm_2);
                  Driver.Commands.Set_Target (Sent, 3, [Now.Closer]);
                  Driver.Commands.Set_Target (Sent, 4, [Now.Part]);
                  Driver.Commands.Set_Target (Sent, 5, [Now.Idle]);
               end if;
               Observe (M, O, Sent);
               loop
                  Driver.Beats.Offer (O.Beat, O, Sent, Took);
                  exit when Took or else Done;
                  delay 0.0;
               end loop;
               exit when not Took;
               Driver.Beats.Await (Pending);
               for G in Group_Id range 1 .. 5 loop
                  if Driver.Commands.Has_Target (Pending, G) then
                     Driver.Commands.Set_Target (Sent, G, Driver.Commands.Target (Pending, G));
                  end if;
               end loop;
               Shown := Now;
               declare
                  Next : Rig_State;
                  T    : constant Real := Real'Min (Driver.Commands.Target (Sent, 5) (1), Limit);
               begin
                  Next.Arm_1 := Driver.Commands.Target (Sent, 1);
                  Next.Arm_2 := Driver.Commands.Target (Sent, 2);
                  Next.Closer := Driver.Commands.Target (Sent, 3) (1);
                  Next.Part := Driver.Commands.Target (Sent, 4) (1);
                  Next.Idle := T - T * abs T;
                  --  The readings show a command the beat after it is in effect, or Late beats later.
                  if Late = 0 then
                     Now := Next;
                  else
                     Now := Due;
                     Due := Next;
                  end if;
               end;
            end;
         end loop;
      exception
         when others =>
            abort Decider;
            raise;
      end;
      if not Done then
         abort Decider;
      end if;
      Finished := Done;
      Taken := Steps;
      Delay_Seen := -1;
      Chained := 0;
      if Driver.Robot.Steps.Episodes (M, 5) > 0 then
         declare
            E : constant Episode := M.Groups (5).Episodes.First_Element;
         begin
            if E.Moved then
               Delay_Seen := Integer (E.Moved_At) - Integer (E.Start);
            end if;
         end;
         for E of M.Groups (5).Episodes loop
            if E.Born_Moving then
               Chained := Chained + 1;
            end if;
         end loop;
      end if;
   end Probe_Drooping_Joint;

   --  The probe must double until the joint stops, not call the second
   --  doubling its end.
   procedure Probe_A_Drooping_Joint is
      Steps    : Natural;
      Finished : Boolean;
      Seen     : Integer;
      Chained  : Natural;
   begin
      --  From an offset of 1e-5, as a group's probe starts from the amount
      --  the probe of every channel together was first seen at.
      Probe_Drooping_Joint (1.0e-5, As_Measured, 0, True, Steps, Finished, Seen, Chained);
      Check (Finished, "the probe did not finish");
      --  From 1e-5 the joint follows seven doublings, to 1.28e-3, and stops at
      --  1e-3: the ninth level is the first that takes it no further.
      Check (Steps = 9, "the probe called the joint's end after" & Steps'Image & " levels, not 9");
   end Probe_A_Drooping_Joint;

   --  The same joint on a body whose readings show a command a beat later than the lab's simulator's, probed as
   --  the body's first push, before any delay is measured: each level must be read after the body has answered
   --  it, not a beat after it was sent. Read too soon, every level reads the one before it (the first level is
   --  judged not executed, the second is sent into the first's answer and credited with it, and so on), the
   --  joint's end comes a level late, no push begins with the joint still, and the body's delay, measured from
   --  pushes born into the motion of the push before, is 0 (seed 2 of the fake robot, 10-14: the boot ended with
   --  nothing seen).
   procedure Probe_A_Delayed_Joint is
      Steps    : Natural;
      Finished : Boolean;
      Seen     : Integer;
      Chained  : Natural;
   begin
      Probe_Drooping_Joint (1.0e-5, As_Measured, 1, False, Steps, Finished, Seen, Chained);
      Check (Finished, "the probe of the delayed joint did not finish");
      Check (Steps = 9, "the probe called the delayed joint's end after" & Steps'Image & " levels, not 9: its levels"
             & " were read before the joint answered them");
      Check (Seen = 1, "the delayed joint's first push was answered" & Seen'Image & " beats after its start, not 1:"
             & " the first level was cut short by the second before its answer");
      Check (Chained = 0, Chained'Image & " of the delayed joint's pushes began while it was still moving from the"
             & " push before: a level was sent before the one before it was answered");
   end Probe_A_Delayed_Joint;

   --  A probe that begins at the smallest step its channel's noise tells takes
   --  that step from the noise the estimates have measured when it begins,
   --  also when the model had lost it and the probe measures it again before
   --  it reads anything. Read first, the step came from the readings'
   --  resolution (a float's epsilon of a reading of 1e-13: 1e-29), and the
   --  probe doubled it as many times as a float has bits without the joint
   --  ever being seen to follow; after the estimates the step is a few times
   --  the noise (5e-13) and the probe reaches the joint's end.
   procedure Probe_Reads_After_It_Measures is
      Kept, Lost         : Natural;
      Kept_Done, Lost_Done : Boolean;
      Kept_Seen, Lost_Seen : Integer;
      Kept_Chained, Lost_Chained : Natural;
   begin
      Probe_Drooping_Joint (0.0, Idle_Beat, 0, True, Kept, Kept_Done, Kept_Seen, Kept_Chained);
      Probe_Drooping_Joint (0.0, Lost_Noise, 0, True, Lost, Lost_Done, Lost_Seen, Lost_Chained);
      Check (Kept_Done and then Lost_Done, "a probe from the smallest step the noise tells did not finish");
      Check (Kept < Real'Machine_Mantissa,
             "the probe from the measured noise doubled as many times as a float has bits (" & Kept'Image
             & " levels) and did not reach the joint's end");
      Check (abs (Integer (Lost) - Integer (Kept)) <= 1,
             "the probe whose noise was lost took" & Lost'Image & " levels, the same probe with its noise kept"
             & Kept'Image & ": its first step was taken before the noise was measured again");
   end Probe_Reads_After_It_Measures;

   --  The rig of the drooping-joint probe, its arm 1 (which eye 1 sees) probed from 0.2, which the eye sees at once, after
   --  the model has lost the noise of the arm's first joint, so that the probe asks for the estimates as it begins. With Wait
   --  beats, the estimates are computed apart from the main loop (Driver.Robot.Compute_Apart): the decider that asks for them
   --  waits (Driver.Beats), the model takes in Wait beats of rest, computes them, and the decider goes on in the beat after,
   --  as Driver.Apart has the main loop do. Seen: the probe confirmed the level the eye saw.
   --  Alone probes arm 1's first channel on its own (Probe_Both_Ways) instead of every listed channel together;
   --  Pushes counts the pushes of group 1 the probe made.
   procedure Probe_Through_Apart_Estimates
     (Wait : Natural; Alone : Boolean; Seen : out Boolean; Excursion : out Real; Finished : out Boolean;
      Pushes : out Natural)
   is
      M     : Model;
      Done  : Boolean := False with Atomic;
      Slow  : Boolean := False with Atomic;   --  the estimates the probe asks for take the Wait beats
      Got   : Driver.Robot.Motion.Probe_Report;
      Two   : Driver.Robot.Motion.Two_Way_Report;
      Before, After : Natural := 0 with Atomic;

      task Decider;
      task body Decider is
         W : Natural;
         procedure Estimate is
         begin
            Estimate_Now (M);
         end Estimate;
         procedure Lose_Noise is
         begin
            M.Noise.Replace_Element (0, Real'Last);   --  group 1's first channel
            M.Noise_Freedom.Replace_Element (0, 0);
         end Lose_Noise;
         procedure Read_Before is
         begin
            Before := Steps.Episodes (M, 1);
         end Read_Before;
         procedure Read_After is
         begin
            After := Steps.Episodes (M, 1);
         end Read_After;
      begin
         Driver.Robot.Motion.Settle (M, W);
         Driver.Robot.Motion.Hold (M, 100);
         for K in 1 .. 16 loop
            declare
               C  : Driver.Commands.Command;
               SR : Driver.Robot.Motion.Step_Report;
            begin
               --  Arm 1, which eye 1 sees, for the body's delay and the eyes' lag.
               Driver.Commands.Set_Target (C, 1, [(if K mod 2 = 1 then 0.1 else 0.0), 0.0]);
               Driver.Robot.Motion.Step (M, C, SR);
               Driver.Robot.Motion.Hold (M, 2 + K mod 4);
            end;
         end loop;
         Driver.Beats.Within_A_Beat (Estimate'Access);
         Driver.Robot.Motion.Gather_Rest (M, 2);
         Driver.Beats.Within_A_Beat (Lose_Noise'Access);
         Slow := True;
         Driver.Beats.Within_A_Beat (Read_Before'Access);
         if Alone then
            Driver.Robot.Motion.Probe_Both_Ways (M, (Group => 1, Channel => 1), 0.2, Two);
            Got.Seen := Two.Seen;
            Got.Excursion := Two.Excursion;
         else
            Driver.Robot.Motion.Probe_Together (M, [1 => (Group => 1, Channel => 1)], 1.0, 0.2, Got);
         end if;
         Driver.Beats.Within_A_Beat (Read_After'Access);
         Done := True;
      exception
         when others =>
            Driver.Beats.Release;
            Done := True;
      end Decider;

      Now, Shown : Rig_State;
      Sent    : Driver.Commands.Command;
      Rng     : Generator;
      Counted : Natural := 0;   --  observations given to the model

      function Observation_At (Beat : Natural) return Observation is
         O : Observation;
      begin
         O.Beat := Driver.Clock.Beat (Beat);
         for E in 1 .. 3 loop
            O.Images.Append (Render (E, Shown));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         O.Readings.Append (Now.Arm_1);
         O.Readings.Append (Now.Arm_2);
         O.Readings.Append (Real_Array'(1 => Now.Closer));
         O.Readings.Append (Real_Array'(1 => Now.Part));
         O.Readings.Append (Real_Array'(1 => Now.Idle + 1.0e-13 * Gaussian (Rng)));
         for G in 1 .. 5 loop
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         end loop;
         return O;
      end Observation_At;
   begin
      Seen := False;
      Excursion := 0.0;
      if Wait > 0 then
         Driver.Robot.Compute_Apart (M);
      end if;
      begin
         loop
            exit when Done;
            declare
               O       : constant Observation := Observation_At (Counted);
               Took    : Boolean := False;
               Pending : Driver.Commands.Command;
            begin
               if Counted = 0 then
                  Driver.Commands.Set_Target (Sent, 1, Now.Arm_1);
                  Driver.Commands.Set_Target (Sent, 2, Now.Arm_2);
                  Driver.Commands.Set_Target (Sent, 3, [Now.Closer]);
                  Driver.Commands.Set_Target (Sent, 4, [Now.Part]);
                  Driver.Commands.Set_Target (Sent, 5, [Now.Idle]);
               end if;
               Observe (M, O, Sent);
               Counted := Counted + 1;
               --  The estimates due (the evidence doubled, or a decider asked): the model goes on taking in rest
               --  while they are computed (all the Wait beats only for the probe's: the rest the model has counted
               --  stays what a boot has by then), and the decider that waited goes on in the beat after.
               if Driver.Robot.Estimates_Due (M) then
                  for K in 1 .. (if Slow then Wait else 0) loop
                     Observe (M, Observation_At (Counted), Sent);
                     Counted := Counted + 1;
                  end loop;
                  Driver.Robot.Compute_Estimates (M);
                  Driver.Beats.Estimates_Adopted;
               end if;
               loop
                  Driver.Beats.Offer (O.Beat, O, Sent, Took);
                  exit when Took or else Done;
                  delay 0.0;
               end loop;
               exit when not Took;
               Driver.Beats.Await (Pending);
               for G in Group_Id range 1 .. 5 loop
                  if Driver.Commands.Has_Target (Pending, G) then
                     Driver.Commands.Set_Target (Sent, G, Driver.Commands.Target (Pending, G));
                  end if;
               end loop;
               Shown := Now;
               Now.Arm_1 := Driver.Commands.Target (Sent, 1);
               Now.Arm_2 := Driver.Commands.Target (Sent, 2);
               Now.Closer := Driver.Commands.Target (Sent, 3) (1);
               Now.Part := Driver.Commands.Target (Sent, 4) (1);
               Now.Idle := Driver.Commands.Target (Sent, 5) (1);
            end;
         end loop;
      exception
         when others =>
            abort Decider;
            raise;
      end;
      if not Done then
         abort Decider;
      end if;
      Finished := Done;
      Seen := Got.Seen;
      Excursion := Got.Excursion;
      Pushes := (if After >= Before then After - Before else 0);
   end Probe_Through_Apart_Estimates;

   --  A channel probed on its own that an eye sees at the first amount is confirmed by the lock-in's regression
   --  after one estimate, not by a run of moves back and forth: the probe pushes it up once and back once.
   procedure Probe_Confirms_By_The_Lockin is
      Seen     : Boolean;
      Amount   : Real;
      Finished : Boolean;
      Pushes   : Natural;
   begin
      Probe_Through_Apart_Estimates (0, True, Seen, Amount, Finished, Pushes);
      Check (Finished, "the probe of the channel alone did not finish");
      Check (Seen and then Amount = 0.2,
             "the channel alone, which the eye sees at 0.2, was sure of" & Amount'Image & " (seen:" & Seen'Image & ")");
      Check (Pushes <= 3, "the probe of the channel alone pushed it" & Pushes'Image
             & " times: a run of moves back and forth confirmed it instead of the lock-in");
   end Probe_Confirms_By_The_Lockin;

   --  An eye sees the joint of an arm move at the first amount: the probe confirms that amount, whether the estimates it
   --  asks for as it begins come back in the same beat or beats later. Read before the estimates, the first beat no look has
   --  judged is as old as the wait, the first look counts a false alarm's chance for every beat of it, the chance passes
   --  one, and no run of moves makes that level certain: the probe is sure only at the next, twice the amount an eye needs
   --  to see the joint, where the boot's pushes after it are taken from.
   procedure Probe_Confirms_Through_Apart_Estimates is
      In_Place, Apart                        : Boolean;
      In_Place_Amount, Apart_Amount          : Real;
      Done_1, Done_2                         : Boolean;
      Pushes_1, Pushes_2                     : Natural;
   begin
      Probe_Through_Apart_Estimates (0, False, In_Place, In_Place_Amount, Done_1, Pushes_1);
      Probe_Through_Apart_Estimates (300, False, Apart, Apart_Amount, Done_2, Pushes_2);
      Check (Done_1 and then Done_2, "a probe through the estimates did not finish");
      Check (In_Place and then In_Place_Amount = 0.2,
             "the probe of an arm's joint, the estimates in place, was sure of" & In_Place_Amount'Image
             & " (seen:" & In_Place'Image & "), not of the 0.2 the eye saw at once");
      Check (Apart and then Apart_Amount = In_Place_Amount,
             "the probe of an arm's joint, the estimates back 300 beats later, was sure of" & Apart_Amount'Image
             & " (seen:" & Apart'Image & "), not of the" & In_Place_Amount'Image & " it was sure of in place");
   end Probe_Confirms_Through_Apart_Estimates;

   --  ── Probing a channel both ways ──
   --
   --  The rig of the drooping-joint probe, its group 5 (which no eye sees)
   --  resting at 0 and reading its target as Kind says, probed both ways
   --  from 1e-5, the amount at which the rest of the body was first seen.
   --  The largest targets asked of it each way are kept.

   type Idle_Kind is (At_Upper_Limit, Deadband, Wide_Deadband, Disconnected);

   --  At its upper limit, 0, and free down to -1e-3; a deadband of 5e-5 each
   --  way, free beyond to 1e-3; the same with a deadband of 5e-4; disconnected,
   --  never moving.
   function Idle_Reading (Kind : Idle_Kind; Target : Real) return Real is
     (case Kind is
         when At_Upper_Limit => Real'Max (-1.0e-3, Real'Min (Target, 0.0)),
         when Deadband       => (if abs Target < 5.0e-5 then 0.0 else Real'Max (-1.0e-3, Real'Min (Target, 1.0e-3))),
         when Wide_Deadband  => (if abs Target < 5.0e-4 then 0.0 else Real'Max (-1.0e-3, Real'Min (Target, 1.0e-3))),
         when Disconnected   => 0.0);

   --  What the model has of group 5 when the probe begins: its noise measured
   --  (the estimates before it), lost (unmeasured, as a group that never came to
   --  rest leaves it, though measuring again would find it), unmeasurable (a
   --  noise that stands as stored, unmeasured), or measured with a lock-in's
   --  visible step of 3e-17 credited to the channel, far below its noise (which
   --  is where a lock-in that credits a group with the pictures' motion beside
   --  its tiny readings leaves them), kept for good.
   type Noise_State is (Measured, Lost, Unmeasurable, Credited);

   procedure Probe_Idle_Both_Ways
     (Kind     : Idle_Kind;
      Noise    : Noise_State;
      Report   : out Driver.Robot.Motion.Two_Way_Report;
      Up, Down : out Real;
      Finished : out Boolean)
   is
      M     : Model;
      Done  : Boolean := False with Atomic;
      Got   : Driver.Robot.Motion.Two_Way_Report;

      task Decider;
      task body Decider is
         W : Natural;
         procedure Estimate is
         begin
            Estimate_Now (M);
         end Estimate;
         procedure Lose_Noise is
            Index : Natural := 0;   --  group 5's channel after every channel of the groups before it
         begin
            for G in 1 .. 4 loop
               Index := Index + Group_Size (M, Group_Id (G));
            end loop;
            M.Noise.Replace_Element (Index, Real'Last);
            M.Noise_Freedom.Replace_Element (Index, 0);
            M.From_File (Stored_Noise) := Noise = Unmeasurable;
         end Lose_Noise;
         procedure Credit_Step is
            Eyes : constant Natural := Natural (M.Eyes.Length);
         begin
            --  Eye 1's lock-in as it would stand if it knew only this channel: one cell,
            --  a gain of 1e34, a step of Z over its root.
            M.Eyes (1).Kept_Groups.Clear;
            M.Eyes (1).Kept_Channels.Clear;
            M.Eyes (1).Gains.Clear;
            M.Eyes (1).Gain_Variances.Clear;
            M.Eyes (1).Kept_Groups.Append (5);
            M.Eyes (1).Kept_Channels.Append (1);
            M.Eyes (1).Gains.Append (1.0e34);
            M.Eyes (1).Gain_Variances.Append (1.0);
            M.Graph.Effects.Replace_Element
              ((5 - 1) * Eyes + 1,
               (Verdict => Patch, Responding => 1, Textured => 1,
                Fraction => (Value => 1.0, Sigma => 0.1, Degrees_Of_Freedom => 0), Resting => 0.0, Rest_Pull => 0.0, Could_Be_Whole => True, Far_Chance => 1.0));
            M.From_File (Stored_Responses) := True;
         end Credit_Step;
      begin
         Driver.Robot.Motion.Settle (M, W);
         Driver.Robot.Motion.Hold (M, 100);
         for K in 1 .. 16 loop
            declare
               C  : Driver.Commands.Command;
               SR : Driver.Robot.Motion.Step_Report;
            begin
               --  Arm 1, which eye 1 sees, for the body's delay and the eyes' lag.
               Driver.Commands.Set_Target (C, 1, [(if K mod 2 = 1 then 0.1 else 0.0), 0.0]);
               Driver.Robot.Motion.Step (M, C, SR);
               Driver.Robot.Motion.Hold (M, 2 + K mod 4);
            end;
         end loop;
         Driver.Beats.Within_A_Beat (Estimate'Access);
         Driver.Robot.Motion.Gather_Rest (M, 2);
         if Noise = Credited then
            Driver.Beats.Within_A_Beat (Credit_Step'Access);
         elsif Noise /= Measured then
            Driver.Beats.Within_A_Beat (Lose_Noise'Access);
         end if;
         Driver.Robot.Motion.Probe_Both_Ways (M, (Group => 5, Channel => 1), 1.0e-5, Got);
         Done := True;
      exception
         when others =>
            Driver.Beats.Release;
            Done := True;
      end Decider;

      Now, Shown : Rig_State;
      Sent  : Driver.Commands.Command;
      Rng   : Generator;
   begin
      Up := 0.0;
      Down := 0.0;
      begin
         for B in 0 .. 5_000 loop
            exit when Done;
            declare
               O       : Observation;
               Took    : Boolean := False;
               Pending : Driver.Commands.Command;
            begin
               O.Beat := Driver.Clock.Beat (B);
               for E in 1 .. 3 loop
                  O.Images.Append (Render (E, Shown));
                  O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
               end loop;
               O.Readings.Append (Now.Arm_1);
               O.Readings.Append (Now.Arm_2);
               O.Readings.Append (Real_Array'(1 => Now.Closer));
               O.Readings.Append (Real_Array'(1 => Now.Part));
               O.Readings.Append (Real_Array'(1 => Now.Idle + 1.0e-13 * Gaussian (Rng)));
               for G in 1 .. 5 loop
                  O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
               end loop;
               if B = 0 then
                  Driver.Commands.Set_Target (Sent, 1, Now.Arm_1);
                  Driver.Commands.Set_Target (Sent, 2, Now.Arm_2);
                  Driver.Commands.Set_Target (Sent, 3, [Now.Closer]);
                  Driver.Commands.Set_Target (Sent, 4, [Now.Part]);
                  Driver.Commands.Set_Target (Sent, 5, [Now.Idle]);
               end if;
               Observe (M, O, Sent);
               loop
                  Driver.Beats.Offer (O.Beat, O, Sent, Took);
                  exit when Took or else Done;
                  delay 0.0;
               end loop;
               exit when not Took;
               Driver.Beats.Await (Pending);
               for G in Group_Id range 1 .. 5 loop
                  if Driver.Commands.Has_Target (Pending, G) then
                     Driver.Commands.Set_Target (Sent, G, Driver.Commands.Target (Pending, G));
                  end if;
               end loop;
               Shown := Now;
               Now.Arm_1 := Driver.Commands.Target (Sent, 1);
               Now.Arm_2 := Driver.Commands.Target (Sent, 2);
               Now.Closer := Driver.Commands.Target (Sent, 3) (1);
               Now.Part := Driver.Commands.Target (Sent, 4) (1);
               Up := Real'Max (Up, Driver.Commands.Target (Sent, 5) (1));
               Down := Real'Max (Down, -Driver.Commands.Target (Sent, 5) (1));
               Now.Idle := Idle_Reading (Kind, Driver.Commands.Target (Sent, 5) (1));
            end;
         end loop;
      exception
         when others =>
            abort Decider;
            raise;
      end;
      if not Done then
         abort Decider;
      end if;
      Finished := Done;
      Report := Got;
   end Probe_Idle_Both_Ways;

   --  A closer at its upper limit (a live one rests at 1.0 and was asked
   --  6.87e10 upwards): it answers downwards at the first level, so the upward
   --  way stops there, never asked more than that first level, also when the
   --  model has lost the channel's noise (the probe measures it again before
   --  it begins). A deadband is two-sided: small asks fail both ways, a larger
   --  one succeeds, and the channel is found answering, not called dead or at
   --  an end, however much wider the band is than what the rest of the body
   --  needed (the rest of the body bounds a channel in no way). A disconnected
   --  channel answers neither way at any level: asked as many levels each way
   --  as a float has bits, it is called dead, also when a lock-in credits it with a
   --  visible step far below its noise; with its noise unmeasured, so that
   --  nothing tells whether its reading followed, it is asked the same and
   --  called blind, not dead.
   procedure Probe_Limits_And_Deadbands is
      package Mo renames Driver.Robot.Motion;
      R        : Mo.Two_Way_Report;
      Up, Down : Real;
      Finished : Boolean;

      procedure Check_Upper_Limit (Noise : Noise_State; Name : String) is
      begin
         Probe_Idle_Both_Ways (At_Upper_Limit, Noise, R, Up, Down, Finished);
         Check (Finished, "the probe of a channel at its upper limit, its noise " & Name & ", did not finish");
         Check (R.At_End (Mo.Increasing) and then R.Levels (Mo.Increasing) = 1,
                "the upward way of a channel at its upper limit, its noise " & Name & ", was asked"
                & R.Levels (Mo.Increasing)'Image & " levels, not stopped at the first, where the downward way answered");
         Check (Up <= 1.0e-5, "the channel at its upper limit, its noise " & Name & ", was asked" & Up'Image & " upwards");
         Check (R.Answered = 1.0e-5 and then not R.Dead and then not R.Blind and then not R.At_End (Mo.Decreasing),
                "the channel at its upper limit, its noise " & Name & ", is not found answering downwards from the"
                & " first level");
         --  Down from 1e-5 it follows to 1e-3 (level 8, 1.28e-3, takes it there);
         --  level 9 takes it no further: its own end.
         Check (R.Levels (Mo.Decreasing) = 9,
                "the downward way, the channel's noise " & Name & ", ended after" & R.Levels (Mo.Decreasing)'Image
                & " levels, not 9");
      end Check_Upper_Limit;
   begin
      Check_Upper_Limit (Measured, "measured");
      Check_Upper_Limit (Lost, "lost");

      --  The rest of the body was first seen at 1e-5; the deadband yields at
      --  level 4, 8e-5, eight times that.
      Probe_Idle_Both_Ways (Deadband, Measured, R, Up, Down, Finished);
      Check (Finished, "the probe of a channel with a deadband did not finish");
      Check (not R.Dead and then R.Answered = 8.0e-5,
             "a deadband of 5e-5 is not found answering at 8e-5: answered at" & R.Answered'Image
             & (if R.Dead then ", called dead" else ""));
      Check (not R.At_End (Mo.Increasing) and then not R.At_End (Mo.Decreasing),
             "a channel with a deadband is called at an end");

      --  A deadband of 5e-4 yields at level 7, 6.4e-4, sixty-four times what
      --  the rest of the body needed.
      Probe_Idle_Both_Ways (Wide_Deadband, Measured, R, Up, Down, Finished);
      Check (Finished, "the probe of a channel with a wide deadband did not finish");
      Check (not R.Dead and then R.Answered = 6.4e-4,
             "a deadband of 5e-4 is not found answering at 6.4e-4: answered at" & R.Answered'Image
             & (if R.Dead then ", called dead" else ""));
      Check (not R.At_End (Mo.Increasing) and then not R.At_End (Mo.Decreasing),
             "a channel with a wide deadband is called at an end");

      --  Neither way answers at any level: every level each way is asked, the
      --  last 1e-5 times two to the power of one less than the bits of a float.
      Probe_Idle_Both_Ways (Disconnected, Measured, R, Up, Down, Finished);
      Check (Finished, "the probe of a disconnected channel did not finish");
      Check (R.Dead and then not R.Blind and then R.Levels (Mo.Increasing) = Real'Machine_Mantissa
             and then R.Levels (Mo.Decreasing) = Real'Machine_Mantissa,
             "a channel that answers neither way is not called dead once asked every level each way: levels"
             & R.Levels (Mo.Increasing)'Image & R.Levels (Mo.Decreasing)'Image);
      Check (Up = 1.0e-5 * 2.0 ** (Real'Machine_Mantissa - 1) and then Down = Up,
             "the dead channel was asked" & Up'Image & " up and" & Down'Image & " down");

      --  A lock-in's visible step of 3e-17 credited to it, far below its noise, is no
      --  evidence of a following: half the jitters of its readings are positive.
      Probe_Idle_Both_Ways (Disconnected, Credited, R, Up, Down, Finished);
      Check (Finished, "the probe of a disconnected channel with a tiny visible step did not finish");
      Check (R.Dead and then R.Levels (Mo.Increasing) = Real'Machine_Mantissa
             and then R.Levels (Mo.Decreasing) = Real'Machine_Mantissa,
             "a visible step far below the noise made a disconnected channel's jitter a following: levels"
             & R.Levels (Mo.Increasing)'Image & R.Levels (Mo.Decreasing)'Image);

      --  Its noise unmeasured, nothing tells a following from none.
      Probe_Idle_Both_Ways (Disconnected, Unmeasurable, R, Up, Down, Finished);
      Check (Finished, "the probe of a disconnected channel with no noise did not finish");
      Check (R.Blind and then not R.Dead, "a channel whose noise is unmeasured is called dead, or not blind");
   end Probe_Limits_And_Deadbands;

   --  ── The kinematics of a synthetic arm ──
   --
   --  Six turning joints carry an eye of 640 x 480 pixels with a focal length
   --  of 400 pixels over a slanted table. The arm turns every joint alone both
   --  ways by 0.05, 0.1 and 0.2 radians, then all of them in seven cells whose
   --  signs are the rows of a Sylvester-Hadamard matrix; a 16 x 12 grid of the
   --  reference view is followed into every keyframe with 0.2 pixels of noise.

   --  How far a fitted lens lies from the true one in units of the fit's own
   --  covariance of it: the chi square of its Lens_Terms terms (the logarithms
   --  of the focal lengths, the principal point, the two distortion terms).
   --  Real'Last when the covariance is not that of a fit or is not positive
   --  definite. A lens fitted as a pinhole with square pixels (Fit_Report's
   --  Pinhole) is judged on the terms it has: the first focal length and the
   --  principal point (Pinhole_Terms).
   Pinhole_Terms : constant array (1 .. 3) of Positive := [1, 3, 4];

   function Lens_Chi_Square
     (Found, Truth : Driver.Robot.Kinematics.Fit.Lens;
      Covariance   : Driver.Robot.Kinematics.Fit.Real_Lists.Vector;
      Pinhole      : Boolean := False) return Real
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      Terms : constant Natural := Natural (Sqrt (Real (Natural (Covariance.Length))));
      All_Error : constant Real_Vector (1 .. Fit.Lens_Terms) :=
        [Ada.Numerics.Long_Elementary_Functions.Log (Found.Fx / Truth.Fx),
         Ada.Numerics.Long_Elementary_Functions.Log (Found.Fy / Truth.Fy),
         Found.Cx - Truth.Cx, Found.Cy - Truth.Cy, Found.K1 - Truth.K1, Found.K2 - Truth.K2];
      Kept  : constant Positive := (if Pinhole then Pinhole_Terms'Length else Fit.Lens_Terms);
      function Term (K : Positive) return Positive is (if Pinhole then Pinhole_Terms (K) else K);
      Error : constant Real_Vector (1 .. Kept) := [for K in 1 .. Kept => All_Error (Term (K))];
      V, L  : Real_Matrix (1 .. Kept, 1 .. Kept) := [others => [others => 0.0]];
      Ok    : Boolean;
   begin
      if Terms < Fit.Lens_Terms or else Terms * Terms /= Natural (Covariance.Length) then
         return Real'Last;
      end if;
      for P in 1 .. Kept loop
         for Q in 1 .. Kept loop
            V (P, Q) := Covariance (Covariance.First_Index + (Term (P) - 1) * Terms + Term (Q) - 1);
         end loop;
      end loop;
      Driver.Numerics.Dense.Cholesky (V, L, Ok);
      if not Ok then
         return Real'Last;
      end if;
      declare
         X : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, Error);
      begin
         return Error * X;
      end;
   end Lens_Chi_Square;

   type Factor_Access is access Real_Matrix;

   --  The Cholesky factor of the covariance exp (-d / Range_Px) of the errors
   --  of points on a grid of Columns x Rows over a picture of 640 x 480
   --  pixels, one in the middle of each cell: a draw of correlated errors is
   --  the factor times independent ones.
   function Field_Factor (Columns, Rows : Positive; Range_Px : Real) return Factor_Access is
      N      : constant Positive := Columns * Rows;
      Cov    : Factor_Access := new Real_Matrix (1 .. N, 1 .. N);
      Result : constant Factor_Access := new Real_Matrix (1 .. N, 1 .. N);
      Ok     : Boolean;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Factor_Access);
   begin
      for A in 1 .. N loop
         for B in 1 .. N loop
            declare
               Du : constant Real := Real ((A - 1) mod Columns - (B - 1) mod Columns) * 640.0 / Real (Columns);
               Dv : constant Real := Real ((A - 1) / Columns - (B - 1) / Columns) * 480.0 / Real (Rows);
            begin
               Cov (A, B) := Exp (-Sqrt (Du ** 2 + Dv ** 2) / Range_Px) + (if A = B then 1.0e-9 else 0.0);
            end;
         end loop;
      end loop;
      Driver.Numerics.Dense.Cholesky (Cov.all, Result.all, Ok);
      Check (Ok, "the covariance of a field of errors over the picture is not positive definite");
      Free (Cov);
      return Result;
   end Field_Factor;

   --  Field = Spread times the factor times independent normal draws.
   procedure Draw_Field (G : in out Generator; Factor : Real_Matrix; Spread : Real; Field : out Real_Array) is
      Z : Real_Array (1 .. Factor'Length (1));
   begin
      for A in Z'Range loop
         Z (A) := Gaussian (G);
      end loop;
      for A in Z'Range loop
         Field (A) := 0.0;
         for B in 1 .. A loop
            Field (A) := Field (A) + Factor (A, B) * Z (B);
         end loop;
         Field (A) := Spread * Field (A);
      end loop;
   end Draw_Field;

   --  Frame_Error is the spread of an error shared by every point of a
   --  keyframe (its rendering, its view), each keyframe's drawn at random;
   --  Track_Error the spread of an error shared by one point's sightings in
   --  every keyframe (where the matcher finds it), drawn at random as a smooth
   --  field over the picture, near points alike (the covariance of two points
   --  falls as exp (-d / 120 px)); Local_Error the spread of a smooth field of
   --  a keyframe's own (exp (-d / 70 px)), drawn for every keyframe: the fit's
   --  reported uncertainty must still cover its errors.
   --  Pending: the matches of joint 1's widest keyframes, both ways, have not
   --  come back (a boot refits between a keyframe and its answer): they have
   --  no sightings yet.
   --  Drawn: the fit is given the readings as A93's scrambler drew its left
   --  arm's (harness/robodojo_scramble): reading I is joint Order_Of (I) of
   --  the chain times Gain (I), a power of two of either sign, so that
   --  neither the readings' order is the chain's nor their unit radians.
   --  Matcher_Noise is the spread of every sighting's own error, in pixels (0.2 unless given; the kiwi's aligner
   --  left 0.6 to 1.2 px a keyframe, R6 10-14); Kept_Share keeps that share of the grid's points, drawn once for the
   --  sweep (a white table leaves the aligner a fifth of its query points, R6).
   procedure Synthetic_Sweep
     (Scale : Real; Expect_Fit : Boolean; Frame_Error : Real := 0.0; Track_Error : Real := 0.0;
      Local_Error : Real := 0.0; Pending : Boolean := False; Drawn : Boolean := False;
      Matcher_Noise : Real := 0.2; Kept_Share : Real := 1.0; Either : Boolean := False; Distortion : Real := 0.0;
      Quantum : Real := 0.0; Relief : Real := 0.0; Order_Needed : Boolean := True)
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Order_Of : constant array (1 .. N) of Positive := (if Drawn then [6, 5, 2, 1, 4, 3] else [1, 2, 3, 4, 5, 6]);
      Gain     : constant Real_Array (1 .. N) :=
        (if Drawn then [-0.5, -1.0, 4.0, -1.0, -0.25, 4.0] else [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
      Levels  : constant Real_Array := [0.05 * Scale, -0.05 * Scale, 0.1 * Scale, -0.1 * Scale, 0.2 * Scale, -0.2 * Scale];
      Cells   : constant := 7;
      Frames  : constant Positive := 1 + N * Levels'Length + Cells;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant Real := Matcher_Noise;
      Keep_Rng : Generator := (State => 13);
      Kept_Point : constant array (1 .. Columns * Rows) of Boolean :=
        [for K in 1 .. Columns * Rows => Kept_Share >= 1.0 or else Uniform (Keep_Rng) < Kept_Share];
      Table   : constant Vec3 := [0.0, -0.6, -0.8];   --  its normal, towards the eye
      Truth   : Fit.Joint_Array (1 .. N);
      Lens    : constant Fit.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => Distortion, K2 => 0.0);
      --  The checks against fixed fractions of the noise hold for the fine sightings (0.2 px, every point, no
      --  distortion) they were set on; elsewhere the fit is judged by its own uncertainty alone.
      Fine    : constant Boolean := Matcher_Noise <= 0.2 and then Kept_Share >= 1.0 and then Distortion = 0.0;
      Changes : Real_Matrix (1 .. Frames, 1 .. N) := [others => [others => 0.0]];
      Rng     : Generator;
      type Sighting_Access is access Fit.Sighting_Array;
      All_Seen : constant Sighting_Access := new Fit.Sighting_Array (1 .. (Frames - 1) * Columns * Rows);
      Seen     : Natural := 0;
      --  The smooth fields, from generators of their own so that the other
      --  errors are drawn as they are without them.
      Track_Rng        : Generator := (State => 7);
      Local_Rng        : Generator := (State => 11);
      Track_U, Track_V : Real_Array (1 .. Columns * Rows) := [others => 0.0];
      Local_U, Local_V : Real_Array (1 .. Columns * Rows) := [others => 0.0];
      Track_Factor     : Factor_Access;
      Local_Factor     : Factor_Access;
   begin
      if Track_Error > 0.0 then
         Track_Factor := Field_Factor (Columns, Rows, 120.0);
         Draw_Field (Track_Rng, Track_Factor.all, Track_Error, Track_U);
         Draw_Field (Track_Rng, Track_Factor.all, Track_Error, Track_V);
      end if;
      if Local_Error > 0.0 then
         Local_Factor := Field_Factor (Columns, Rows, 70.0);
      end if;
      declare
         Axes   : constant array (1 .. N) of Vec3 :=
           [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
         Points : constant array (1 .. N) of Vec3 :=
           [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      begin
         for J in 1 .. N loop
            declare
               W : constant Vec3 := Unit (Axes (J));
            begin
               Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
            end;
         end loop;
      end;
      for J in 1 .. N loop
         for L in Levels'Range loop
            Changes (1 + (J - 1) * Levels'Length + (L - Levels'First + 1), J) := Levels (L);
         end loop;
      end loop;
      for Row in 1 .. Cells loop
         for J in 1 .. N loop
            declare
               Bits : Natural := 0;
               R    : Natural := Row;
               C    : Natural := J;
            begin
               while R > 0 and then C > 0 loop
                  if R mod 2 = 1 and then C mod 2 = 1 then
                     Bits := Bits + 1;
                  end if;
                  R := R / 2;
                  C := C / 2;
               end loop;
               Changes (1 + N * Levels'Length + Row, J) := (if Bits mod 2 = 0 then 0.05 else -0.05) * Scale;
            end;
         end loop;
      end loop;
      for F in 2 .. Frames loop
         declare
            D : Real_Array (1 .. N);
            T : Rigid;
         begin
            for J in 1 .. N loop
               D (J) := Changes (F, J);
            end loop;
            T := Inverse (Fit.Eye_At (Truth, Along (Truth), D));
            declare
               Shared_U : constant Real := Frame_Error * Gaussian (Rng);
               Shared_V : constant Real := Frame_Error * Gaussian (Rng);
            begin
               if Local_Error > 0.0 then
                  Draw_Field (Local_Rng, Local_Factor.all, Local_Error, Local_U);
                  Draw_Field (Local_Rng, Local_Factor.all, Local_Error, Local_V);
               end if;
               for Gy in 1 .. Rows loop
                  for Gx in 1 .. Columns loop
                     declare
                        K  : constant Positive := (Gy - 1) * Columns + Gx;
                        U0 : constant Real := (Real (Gx) - 0.5) * 640.0 / Real (Columns);
                        V0 : constant Real := (Real (Gy) - 0.5) * 480.0 / Real (Rows);
                        --  The table: the plane one unit from the eye along its normal.
                        H     : constant Vec3 := Fit.Ray (Lens, U0, V0);
                        --  Relief: the scene no plane, each point's depth off the table's by up to that share, by the
                        --  golden angle from one point to the next (no two alike).
                        Plane_Depth : constant Real := -1.0 / Real'(Unit (Table) * H);
                        Depth : constant Real := Plane_Depth * (1.0 + Relief * Sin (Real (K) * Ada.Numerics.Pi * (3.0 - Sqrt (5.0))));
                        X  : constant Vec3 := Depth * H;
                        U, V : Real;
                        Ahead : Boolean;
                     begin
                        Fit.Project (Lens, T * X, U, V, Ahead);
                        U := U + Noise * Gaussian (Rng) + Shared_U + Track_U (K) + Local_U (K);
                        V := V + Noise * Gaussian (Rng) + Shared_V + Track_V (K) + Local_V (K);
                        if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 and then Kept_Point (K) then
                           Seen := Seen + 1;
                           All_Seen (Seen) := (Frame => F, Track => K, U0 => U0, V0 => V0, U => U, V => V, Own => 0.0, Own_Freedom => 0,
                                               others => <>);
                        end if;
                     end;
                  end loop;
               end loop;
            end;
         end;
      end loop;
      if Pending then
         declare
            Up, Down : Positive := 2;
            Kept     : Natural := 0;
         begin
            --  Joint 1's keyframes are the first of the sweep.
            for F in 2 .. 1 + Levels'Length loop
               if Changes (F, 1) > Changes (Up, 1) then
                  Up := F;
               end if;
               if Changes (F, 1) < Changes (Down, 1) then
                  Down := F;
               end if;
            end loop;
            for I in 1 .. Seen loop
               if All_Seen (I).Frame /= Up and then All_Seen (I).Frame /= Down then
                  Kept := Kept + 1;
                  All_Seen (Kept) := All_Seen (I);
               end if;
            end loop;
            Seen := Kept;
         end;
      end if;
      declare
         Joints : Fit.Joint_Array (1 .. N);
         Chain  : Fit.Chain_Order (1 .. N);
         Found  : Fit.Lens;
         Report : Fit.Fit_Report;
         --  What the fit is given: the readings' changes as drawn, and the
         --  arm it should find, by reading: joint Order_Of (I) turning by one
         --  over Gain (I) per reading unit, and the chain through the draw.
         Given      : Real_Matrix (1 .. Frames, 1 .. N);
         Asked      : Fit.Flag_Matrix (1 .. Frames, 1 .. N);   --  the draw's joints that turned: what a sweep asked
         Want_Joints : Fit.Joint_Array (1 .. N);
         Want_Chain  : Fit.Chain_Order (1 .. N);
         function Drawn_Of (D : Real_Array) return Real_Array is
           ([for I in 1 .. N => Gain (I) * D (Order_Of (I))]);
      begin
         for F in 1 .. Frames loop
            for I in 1 .. N loop
               --  A servo reads its turn to the nearest tick (Quantum, 0 for none): the fit is given that.
               Given (F, I) := Gain (I) * (if Quantum > 0.0 then Quantum * Real'Rounding (Changes (F, Order_Of (I)) / Quantum)
                                           else Changes (F, Order_Of (I)));
               Asked (F, I) := Changes (F, Order_Of (I)) /= 0.0;
            end loop;
         end loop;
         for I in 1 .. N loop
            Want_Joints (I) := Truth (Order_Of (I));
            Want_Joints (I).C := 1.0 / Gain (I);
            Want_Chain (Order_Of (I)) := I;
         end loop;
         Fit.Fit (Given, Asked, [for I in 1 .. N => abs Gain (I) * 0.01 * Scale], All_Seen (1 .. Seen), 640, 480, 0, Joints,
                  Chain, Found, Report);
         Driver.Log.Line (Driver.Log.Robot, "kinematics test: widest turn" & Real'Image (0.2 * Scale) & " rad, noise"
                          & Real'Image (Noise) & " px, " & Seen'Image & " sightings: " & (if Report.Fitted then "fitted, focal "
                          & Driver.Log.Image (Found.Fx, 2) & " +- " & Driver.Log.Image (Report.Focal_Sigma, 2) & " px"
                          else "not fitted at stage" & Report.Stage'Image & ", " & Ada.Strings.Unbounded.To_String (Report.Why)));
         if not Expect_Fit then
            Check (not Report.Fitted, "a sweep of" & Real'Image (0.2 * Scale) & " rad at most was fitted, focal"
                   & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy));
            return;
         end if;
         --  Either: a sweep that may or may not tell the arm; what the fit reports, it reports right.
         if Either and then not Report.Fitted then
            return;
         end if;
         Check (Report.Fitted, "the fit did not succeed: stage" & Report.Stage'Image & ", "
                & Ada.Strings.Unbounded.To_String (Report.Why));
         if not Report.Fitted then
            return;
         end if;
         --  The fit's own uncertainty covers its errors: within Z of its sigmas.
         Check (abs (Found.Fx - Lens.Fx) <= Driver.Conventions.Z * Report.Focal_Sigma,
                "the focal length is off by" & Real'Image (Found.Fx - Lens.Fx) & " px, its sigma"
                & Report.Focal_Sigma'Image);
         --  The whole lens, its terms together: a chi square as rare as Z.
         declare
            Chi  : constant Real := Lens_Chi_Square (Found, Lens, Report.Covariance, Report.Pinhole);
            Kept : constant Positive := (if Report.Pinhole then Pinhole_Terms'Length else Fit.Lens_Terms);
         begin
            Check (Driver.Distributions.Chi_Square_Deviate (Chi, Kept) <= Driver.Conventions.Z,
                   "the lens is off by a chi square of" & Real'Image (Chi) & " on" & Kept'Image
                   & " terms: focal" & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy) & ", centre"
                   & Real'Image (Found.Cx) & "," & Real'Image (Found.Cy) & ", distortion" & Real'Image (Found.K1)
                   & "," & Real'Image (Found.K2));
            Driver.Log.Line (Driver.Log.Robot, "kinematics test lens: chi square" & Real'Image (Chi) & " on"
                             & Fit.Lens_Terms'Image & " terms");
         end;
         if Frame_Error = 0.0 and then Fine then
            Check_Close (Found.Fx, Lens.Fx, Lens.Fx * Noise / 40.0, "the focal length across");
            Check_Close (Found.Fy, Lens.Fy, Lens.Fy * Noise / 40.0, "the focal length down");
            --  Each reading's turn per unit, as a vector: an axis and its
            --  opposite with the scale's sign turned are one joint.
            for J in 1 .. N loop
               declare
                  Got_Turn  : constant Vec3 := Joints (J).C * Joints (J).W;
                  Want_Turn : constant Vec3 := Want_Joints (J).C * Want_Joints (J).W;
                  Off       : constant Real := Arccos (Real'Max (-1.0, Real'Min (1.0, Unit (Got_Turn) * Unit (Want_Turn))));
               begin
                  Check (Off < Noise / 40.0, "reading" & J'Image & "'s axis is off by" & Off'Image & " rad");
                  Check (abs (abs Got_Turn / abs Want_Turn - 1.0) < Noise / 40.0,
                         "reading" & J'Image & " turns by" & Real'Image (abs Got_Turn) & " rad per unit, not"
                         & Real'Image (abs Want_Turn));
               end;
            end loop;
         end if;
         --  The order, and the eye where it rests on the order: where the sweep's keyframes leave the true order the
         --  best (Order_Needed), the best is the true one, told or not; and a told one is right. Whether keyframes tell
         --  an order at Z is the order exams' (robot.order.exam): these sweeps turn no pair the fit names (the boot's
         --  Pair_Shares), and the fits that told an order from them told it by sightings counted one by one, the test
         --  that told s83's wrong order (servo seed 6 of the fake robot, 10-18).
         if Order_Needed or else Report.Order_Told then
         Check (Fit."=" (Chain, Want_Chain), "the joints along the arm, by their readings, came out as " & Chain_Text (Chain)
                & ", not " & Chain_Text (Want_Chain));
         --  The eye at a pose no keyframe had, lengths in the fit's units.
         declare
            Test  : constant Real_Array (1 .. N) := [0.15, -0.1, 0.08, -0.12, 0.1, -0.15];
            Scale : Real := 0.0;
            Want, Got : Rigid;
         begin
            for F in 1 .. Frames loop
               declare
                  D : Real_Array (1 .. N);
               begin
                  for J in 1 .. N loop
                     D (J) := Changes (F, J);
                  end loop;
                  Scale := Scale + Fit.Eye_At (Truth, Along (Truth), D).Translation * Fit.Eye_At (Truth, Along (Truth), D).Translation;
               end;
            end loop;
            Scale := Sqrt (Scale / Real (Frames));
            Want := Fit.Eye_At (Truth, Along (Truth), Test);
            Got := Fit.Eye_At (Joints, Chain, Drawn_Of (Test));
            if Frame_Error = 0.0 and then Fine then
               Check (abs (Scale * Got.Translation - Want.Translation) < Scale * Noise / 40.0,
                      "the eye at a new pose is off by" & Real'Image (abs (Scale * Got.Translation - Want.Translation) / Scale)
                      & " of the arm's reach");
               Check (Driver.Numerics.Angle (Transpose (Got.Rotation) * Want.Rotation) < Noise / 40.0,
                      "the eye at a new pose is turned by"
                      & Real'Image (Driver.Numerics.Angle (Transpose (Got.Rotation) * Want.Rotation)) & " rad");
            end if;
            --  The pose's own uncertainty covers its error: within Z of the
            --  root of its covariance's trace (model units, radians).
            declare
               Turn, Place : Mat3;
               Off  : constant Real := abs (Got.Translation - (1.0 / Scale) * Want.Translation);
               Turned : constant Real := Driver.Numerics.Angle (Transpose (Got.Rotation) * Want.Rotation);
            begin
               Fit.Pose_Covariance (Joints, Chain, Drawn_Of (Test), Report.Covariance, Turn, Place);
               --  The fit's covariance covers every joint it was given: the
               --  pose's uncertainty is known.
               Check (Turn (1, 1) < Real'Last and then Place (1, 1) < Real'Last,
                      "the fit's covariance does not cover its joints:" & Report.Covariance.Length'Image & " entries for"
                      & N'Image & " joints");
               Check (Off <= Driver.Conventions.Z * Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3)),
                      "the eye at a new pose is off by" & Off'Image & " model units, its sigma"
                      & Real'Image (Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3))));
               Check (Turned <= Driver.Conventions.Z * Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3)),
                      "the eye at a new pose is turned by" & Turned'Image & " rad, its sigma"
                      & Real'Image (Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3))));
               Driver.Log.Line (Driver.Log.Robot, "kinematics test honesty: focal off" & Real'Image (Found.Fx - Lens.Fx)
                                & " sigma" & Report.Focal_Sigma'Image & "; pose off" & Off'Image & " sigma"
                                & Real'Image (Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3))) & "; turn" & Turned'Image
                                & " sigma" & Real'Image (Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3))));
            end;
         end;
         end if;
         --  The table, where the scene is one (no relief).
         if Frame_Error = 0.0 and then Relief = 0.0 then
         declare
            Normal : Vec3;
            Offset, Offset_Sigma : Real;
            Sigma  : Real;
            Flat   : Boolean;
            Sights : Fit.Sight_Point_Array (1 .. Columns * Rows);
            On     : Fit.Flag_Array (1 .. Columns * Rows);
            Plane  : Fit.Sight_Plane;
         begin
            --  The table: the plane most of the tracks lie on, at the depths
            --  the fit refined.
            for Gy in 1 .. Rows loop
               for Gx in 1 .. Columns loop
                  declare
                     T : constant Positive := (Gy - 1) * Columns + Gx;
                  begin
                     Sights (T) :=
                       (H     => Fit.Ray (Found, (Real (Gx) - 0.5) * 640.0 / Real (Columns),
                                          (Real (Gy) - 0.5) * 480.0 / Real (Rows)),
                        Depth => (if T <= Natural (Report.Depths.Length) then Report.Depths (T) else 0.0),
                        Sigma => (if T <= Natural (Report.Depth_Sigmas.Length) then Report.Depth_Sigmas (T)
                                  else Real'Last));
                  end;
               end loop;
            end loop;
            Fit.Dominant_Plane (Sights, Plane, On);
            Flat := Plane.Found;
            Normal := Fit.Plane_Normal (Plane);
            Offset := Fit.Plane_Offset (Plane);
            Offset_Sigma := Fit.Plane_Offset_Sigma (Plane);
            Sigma := Fit.Plane_Tilt_Sigma (Plane);
            Check (Flat, "no table found");
            --  The reference eye, at the origin, lies on the side the normal points
            --  to: its distance from the table is minus the offset.
            Check (Offset < 0.0 and then Offset_Sigma < -Offset, "the table's offset is" & Offset'Image & " +-"
                   & Offset_Sigma'Image);
            if Fine then
               Check (Arccos (Real'Min (1.0, Normal * Unit (Table))) < Noise / 40.0,
                      "the table's normal is off by" & Real'Image (Arccos (Real'Min (1.0, Normal * Unit (Table)))) & " rad");
               Check (Sigma < Noise / 40.0, "the table's normal is uncertain by" & Sigma'Image & " rad");
            else
               --  The table a desktop arm's eye sees: its normal within Z of its uncertainty as the driver takes it,
               --  the points' scatter and what the fit's own uncertainty moves every depth by together
               --  (Plane_Covariance, as Kinematics has the table): sparse sightings a pixel off leave the second the
               --  larger.
               declare
                  Whole : Fit.Sight_Plane := Plane;
               begin
                  Whole.Covariance := Fit.Plane_Covariance (Sights, On, Plane, Report.Depth_Gains, Report.Covariance);
                  Check (Arccos (Real'Min (1.0, Normal * Unit (Table))) <= Driver.Conventions.Z * Fit.Plane_Tilt_Sigma (Whole),
                         "the table's normal is off by" & Real'Image (Arccos (Real'Min (1.0, Normal * Unit (Table))))
                         & " rad, its sigma" & Real'Image (Fit.Plane_Tilt_Sigma (Whole)) & " (the points' scatter alone"
                         & Sigma'Image & ")");
               end;
            end if;
         end;
         end if;
         Driver.Log.Line (Driver.Log.Robot, "kinematics test: focal " & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy)
                          & ", median " & Real'Image (Report.Median_Px) & " px over" & Report.Used'Image & " sightings");
      end;
   end Synthetic_Sweep;

   --  A track's depth is known when it is positive and its logarithm is uncertain by less than 1 / Z (it is
   --  significant against its own relative uncertainty); a track the sightings leave with no depth at all is not
   --  known, whatever the size of the point it makes: A17's first fit had one at 1.7E97 units, its logarithm
   --  uncertain by 2.5E6, which the head eye's fit took for a point and raised on.
   procedure Depth_Is_Known_Or_Not is
      package Fit renames Driver.Robot.Kinematics.Fit;
      Z : constant Real := Driver.Conventions.Z;
   begin
      Check (Fit.Depth_Known (1.0, 0.01), "a depth known to a hundredth is not known");
      Check (Fit.Depth_Known (3.7, 0.5 / Z), "a depth known to half of 1 / Z of itself is not known");
      Check (not Fit.Depth_Known (3.7, 2.0 / Z), "a depth known to twice 1 / Z of itself is known");
      Check (not Fit.Depth_Known (1.7E97, 2.5E6), "a depth of 1.7E97 uncertain by 2.5E6 is known");
      Check (not Fit.Depth_Known (1.0, Real'Last), "a depth with no uncertainty stated is known");
      Check (not Fit.Depth_Known (0.0, 0.01), "no depth is known");
      Check (not Fit.Depth_Known (-2.0, 0.01), "a depth behind the eye is known");
   end Depth_Is_Known_Or_Not;

   --  The arm of the kinematics test, taken as measured: a pose it can reach
   --  is reached, one beyond the readings it moved through is not.
   procedure Reach_A_Pose is
      package Fit renames Driver.Robot.Kinematics.Fit;
      M      : Model;
      Axes   : constant array (1 .. 6) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Points : constant array (1 .. 6) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      Truth  : Fit.Joint_Array (1 .. 6);
      Arm    : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
      Goal_Q : constant Real_Array (1 .. 6) := [0.2, -0.15, 0.1, 0.25, -0.2, 0.3];
      Zero   : constant Real_Array (1 .. 6) := [others => 0.0];
      Goal   : Rigid;
      Q      : Real_Array (1 .. 6);
      Position_Off, Turn_Off : Real;
   begin
      for J in 1 .. 6 loop
         declare
            W : constant Vec3 := Unit (Axes (J));
         begin
            Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
            Arm.Result.Joints.Append (Joint_Fit'(W => Truth (J).W, P => Truth (J).P, C => 1.0, Slide => False, others => <>));
            Arm.Result.Chain.Append (J);
            Arm.Result.Reference.Append (0.0);
         end;
      end loop;
      Arm.Result.Fitted := True;
      M.Kinematics.Append (Arm);
      --  The graph lists group 1 as arm 1, carrying eye 1: the fit is the
      --  arm's now.
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      Goal := Fit.Eye_At (Truth, Along (Truth), Goal_Q);
      Driver.Robot.Kinematics.Solve_Pose (M, 1, Zero, Goal, False, Q, Position_Off, Turn_Off);
      Check (Position_Off < 1.0e-9 and then Turn_Off < 1.0e-9,
             "a reachable pose is missed by" & Position_Off'Image & " and" & Turn_Off'Image & " rad");
      declare
         Got : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q);
      begin
         Check (abs (Got.Translation - Goal.Translation) < 1.0e-9, "the readings found do not put the eye at the goal");
      end;
   end Reach_A_Pose;

   --  A joint that turns repeats its pose over a turn of its axis, which the fit reads as a period of readings:
   --  two pi over the scale it found for the joint. The readings that put the eye at a pose are the ones nearest
   --  where the arm is, however many turns the solver's steps made on the way: A16's press 3 was planned as one
   --  waypoint 5.68 rad from the readings, where the same pose is 3.26 rad from them.
   procedure Reach_The_Nearest_Turn is
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Motion renames Driver.Robot.Motion;
      use type Motion.Plan_Status;
      M      : Model;
      Axes   : constant array (1 .. 6) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Points : constant array (1 .. 6) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      --  Readings that are not radians (two joints' scales), and a sixth joint that slides.
      Scales : constant Real_Array (1 .. 6) := [1.0, 0.9, 1.0, 1.1, 1.0, 1.0];
      Slides : constant array (1 .. 6) of Boolean := [False, False, False, False, False, True];
      Truth  : Fit.Joint_Array (1 .. 6);
      Arm    : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
      Rng    : Generator;

      function Period (J : Positive) return Real is (2.0 * Ada.Numerics.Pi / Scales (J));
   begin
      for J in 1 .. 6 loop
         declare
            W : constant Vec3 := Unit (Axes (J));
         begin
            Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => Scales (J), Slide => Slides (J));
            Arm.Result.Joints.Append (Joint_Fit'(W => W, P => Truth (J).P, C => Scales (J), Slide => Slides (J), others => <>));
            Arm.Result.Chain.Append (J);
            Arm.Result.Reference.Append (0.0);
         end;
      end loop;
      Arm.Result.Fitted := True;
      Arm.Result.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Arm.Result.Sigma_Px := 0.1;
      M.Kinematics.Append (Arm);
      M.Groups.Append (Group_Stream'(Size => 6, Commandable => True, others => <>));
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      --  The nearest representative: whole periods of the fit, not of a turn of readings taken as radians.
      declare
         Near : constant Real_Array (1 .. 6) := [0.1, -0.2, 0.3, 0.0, -0.1, 0.4];
         Far  : constant Real_Array (1 .. 6) :=
           [0.1 + 3.0 * Period (1), -0.2 + 2.0 * Period (2), 0.3 - Period (3), 0.0 - 2.0 * Period (4),
            -0.1 + 0.49 * Period (5), 0.4 + 5.0];
         Back : constant Real_Array := Driver.Robot.Kinematics.Nearest_Readings (M, 1, Near, Far);
      begin
         for J in 1 .. 5 loop
            Check (abs (Back (J) - (if J = 5 then Far (5) else Near (J))) < 1.0e-9,
                   "joint" & J'Image & " is given as reading" & Real'Image (Back (J)) & ", the nearest to" & Real'Image (Near (J))
                   & " that puts the eye where the far one does is" & Real'Image (if J = 5 then Far (5) else Near (J)));
         end loop;
         Check (Back (6) = Far (6), "a joint that slides is given as a nearer reading");
      end;
      --  Far goals: whatever the solver's steps came to, no joint that turns ends more than half a period from
      --  where it started.
      declare
         Reached, Wrapped : Natural := 0;
      begin
         for Trial in 1 .. 60 loop
            declare
               Goal_Q, Start, Q : Real_Array (1 .. 6);
               Position_Off, Turn_Off : Real;
            begin
               for J in 1 .. 6 loop
                  Goal_Q (J) := (2.0 * Uniform (Rng) - 1.0) * 4.5;
                  Start (J) := (2.0 * Uniform (Rng) - 1.0) * 0.5;
               end loop;
               Driver.Robot.Kinematics.Solve_Pose (M, 1, Start, Fit.Eye_At (Truth, Along (Truth), Goal_Q), False, Q, Position_Off, Turn_Off);
               if Position_Off < 1.0e-6 and then Turn_Off < 1.0e-6 then
                  Reached := Reached + 1;
                  for J in 1 .. 5 loop
                     Check (abs (Q (J) - Start (J)) <= Period (J) / 2.0 + 1.0e-9,
                            "goal" & Trial'Image & ": joint" & J'Image & " ends" & Real'Image (Q (J) - Start (J))
                            & " from where it started, over half its period" & Real'Image (Period (J) / 2.0));
                  end loop;
                  for J in 1 .. 5 loop
                     if abs (Goal_Q (J) - Start (J)) > Period (J) / 2.0 then
                        Wrapped := Wrapped + 1;
                     end if;
                  end loop;
               end if;
            end;
         end loop;
         Driver.Log.Line (Driver.Log.Robot, "reach test: of 60 far goals" & Reached'Image & " were reached, and" & Wrapped'Image
                          & " joints of them lay over half a period from the start");
         Check (Reached >= 10, "only" & Reached'Image & " of 60 far goals were reached");
         Check (Wrapped >= 5, "only" & Wrapped'Image & " joints of the goals needed a nearer representative");
      end;
      --  A reach planned there ends at the nearest readings too.
      declare
         Start  : constant Real_Array (1 .. 6) := [0.2232, 0.5132, 0.2175, -0.4909, 0.7443, -1.3984];
         Goal_Q : constant Real_Array (1 .. 6) := [0.2232, 0.5132 - 2.0 * Period (2), 0.2175, -0.4909 + Period (4), 0.7443, -1.3984 + 1.0];
         O      : Observation;
      begin
         O.Readings.Append (Start);
         declare
            P : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 1, O, (Pose => Fit.Eye_At (Truth, Along (Truth), Goal_Q), Position_Only => False, Free_About => <>));
         begin
            Check (Motion.Status (P) = Motion.Planned, "a goal a period away is not planned: " & Motion.Why (P));
            if Motion.Status (P) = Motion.Planned then
               declare
                  End_At : constant Real_Array := Motion.Last_Readings (P);
               begin
                  Check (abs (End_At (2) - Start (2)) < 1.0e-6 and then abs (End_At (4) - Start (4)) < 1.0e-6,
                         "the plan ends at joint 2 =" & Real'Image (End_At (2)) & " and joint 4 =" & Real'Image (End_At (4))
                         & ", not where the arm is");
               end;
            end if;
         end;
      end;
   end Reach_The_Nearest_Turn;

   --  A six-joint arm for the planner's tests, its fit exact: the model, the
   --  joints it was made of, and the readings it starts at.
   procedure Planner_Arm (M : in out Model; Truth : out Driver.Robot.Kinematics.Fit.Joint_Array; O : out Observation;
                          Start : Real_Array)
   is
      Axes   : constant array (1 .. 6) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Points : constant array (1 .. 6) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      Arm    : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
   begin
      for J in 1 .. 6 loop
         declare
            W : constant Vec3 := Unit (Axes (J));
         begin
            Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
            Arm.Result.Joints.Append (Joint_Fit'(W => Truth (J).W, P => Truth (J).P, C => 1.0, Slide => False, others => <>));
            Arm.Result.Chain.Append (J);
            Arm.Result.Reference.Append (0.0);
         end;
      end loop;
      Arm.Result.Fitted := True;
      Arm.Result.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Arm.Result.Sigma_Px := 0.1;
      M.Kinematics.Append (Arm);
      M.Groups.Append (Group_Stream'(Size => 6, Commandable => True, others => <>));
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      O.Readings.Append (Start);
   end Planner_Arm;

   --  How far the arm strays from the straight line through From along Line, going from waypoint to waypoint of
   --  the plan along the joints' straight lines, from the readings Start.
   function Plan_Stray (M : Model; P : Driver.Robot.Motion.Plan; Start : Real_Array; From : Rigid; Line : Vec3)
     return Real
   is
      Worst : Real := 0.0;
      Back  : Real_Array (Start'Range) := Start;
   begin
      for K in 1 .. Driver.Robot.Motion.Waypoint_Count (P) loop
         declare
            Next : constant Real_Array := Driver.Robot.Motion.Waypoint (P, K);
         begin
            for S in 1 .. 16 loop
               declare
                  Q : Real_Array (Start'Range);
               begin
                  for J in Start'Range loop
                     Q (J) := Back (J) + (Real (S) / 16.0) * (Next (J) - Back (J));
                  end loop;
                  declare
                     T   : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q);
                     D   : constant Vec3 := T.Translation - From.Translation;
                     Off : constant Vec3 := D - Real'(D * Line) * Line;
                  begin
                     Worst := Real'Max (Worst, abs Off);
                  end;
               end;
            end loop;
            Back := Next;
         end;
      end loop;
      return Worst;
   end Plan_Stray;

   --  The arm goes from one waypoint to the next along its joints' own
   --  straight line, which bows from the straight line of the tool (A17's left
   --  arm, asked for moves of 16 to 23 units in one beat, bowed by 3.7 to 14.7
   --  of them, and the straight line is what the action layer clears
   --  obstacles by): a reach given a clearance is cut until the tool, at the
   --  middle of the joints' straight line, is within it of the straight path.
   procedure Plan_Stays_Within_Its_Clearance is
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Motion renames Driver.Robot.Motion;
      use type Motion.Plan_Status;
      M      : Model;
      Truth  : Fit.Joint_Array (1 .. 6);
      Start  : constant Real_Array (1 .. 6) := [others => 0.0];
      Goal_Q : constant Real_Array (1 .. 6) := [1.2, -0.9, 1.1, 0.8, -1.0, 0.9];
      O      : Observation;
      From, Goal : Rigid;
      Line   : Vec3;

      function Stray (P : Motion.Plan) return Real is (Plan_Stray (M, P, Start, From, Line));
   begin
      Planner_Arm (M, Truth, O, Start);
      From := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Start);
      Goal := Fit.Eye_At (Truth, Along (Truth), Goal_Q);
      Line := Unit (Goal.Translation - From.Translation);
      declare
         Sigma : constant Real := Driver.Robot.Kinematics.Angle_Sigma (M, 1);
         Free  : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 1, O, (Pose => Goal, Position_Only => False, Free_About => <>));
         Bow   : Real;
      begin
         Check (Motion.Status (Free) = Motion.Planned, "a pose the arm can reach is not planned: " & Motion.Why (Free));
         Bow := Stray (Free);
         Driver.Log.Line (Driver.Log.Robot, "plan test: a move of" & Real'Image (abs (Goal.Translation - From.Translation))
                          & " bows by" & Real'Image (Bow) & " in" & Motion.Waypoint_Count (Free)'Image & " waypoints");
         Check (Bow > 10.0 * Sigma, "the joints' straight line does not bow from the tool's here, so the test shows nothing");
         declare
            Asked : constant Real := Bow / 8.0;
            Held  : constant Motion.Plan := Motion.Plan_Reach_In_Arm
              (M, 1, O, (Pose => Goal, Position_Only => False, Free_About => <>), Clearance => Asked);
         begin
            Check (Motion.Status (Held) = Motion.Planned, "a pose the arm can reach is not planned with a clearance: "
                   & Motion.Why (Held));
            Driver.Log.Line (Driver.Log.Robot, "plan test: with a clearance of" & Real'Image (Asked) & " it strays by"
                             & Real'Image (Stray (Held)) & " in" & Motion.Waypoint_Count (Held)'Image & " waypoints");
            Check (Stray (Held) <= Asked + Driver.Conventions.Z * Sigma,
                   "the arm strays" & Real'Image (Stray (Held)) & " from the straight path though it was asked to keep within"
                   & Real'Image (Asked));
            Check (Motion.Waypoint_Count (Held) > Motion.Waypoint_Count (Free),
                   "a reach asked to keep to the straight path is not cut");
            --  The goal is still where the plan ends.
            declare
               End_At : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Motion.Last_Readings (Held));
            begin
               Check (abs (End_At.Translation - Goal.Translation) < Driver.Conventions.Z * Sigma,
                      "a plan kept to the straight path ends away from the goal");
            end;
         end;
         --  A goal the arm cannot reach is refused when halving the path has
         --  come down to what the fit can tell apart; halving it on to the
         --  float's last bit learns nothing (it cost 100 ms of a turned cup's
         --  search, hundreds of times over).
         declare
            Far    : constant Rigid := (Rotation => Goal.Rotation, Translation => From.Translation + 100.0 * Line);
            Lost   : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 1, O, (Pose => Far, Position_Only => False, Free_About => <>));
            Levels : constant Natural :=
              Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (100.0 / Sigma)
                                     / Ada.Numerics.Long_Elementary_Functions.Log (2.0)));
         begin
            Check (Motion.Status (Lost) = Motion.Unreachable, "a goal a hundred units off is planned");
            Driver.Log.Line (Driver.Log.Robot, "plan test: refusing a goal 100 away took" & Motion.Solve_Count (Lost)'Image
                             & " solves; the path halves" & Levels'Image & " times down to the fit's sigma" & Real'Image (Sigma));
            Check (Motion.Solve_Count (Lost) <= 2 * (Levels + 1),
                   "refusing a goal out of reach took" & Motion.Solve_Count (Lost)'Image & " solves, more than halving "
                   & "the path down to the fit's sigma needs (" & Natural'Image (2 * (Levels + 1)) & ")");
         end;
      end;
   end Plan_Stays_Within_Its_Clearance;

   --  Two joints about parallel axes, the eye A beyond the first axis's reach of the second and B beyond that, at
   --  readings zero: the eye is at e^(i q1) (A + B e^(i q2)) from the first axis.
   procedure Planar_Arm (M : in out Model; Truth : out Driver.Robot.Kinematics.Fit.Joint_Array; O : out Observation;
                         Start : Real_Array; A, B : Real)
   is
      Arm : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
      Pts : constant array (1 .. 2) of Vec3 := [[-(A + B), 0.0, 0.0], [-B, 0.0, 0.0]];
   begin
      for J in 1 .. 2 loop
         Truth (J) := (W => [0.0, 0.0, 1.0], P => Pts (J), C => 1.0, Slide => False);
         Arm.Result.Joints.Append (Joint_Fit'(W => Truth (J).W, P => Truth (J).P, C => 1.0, Slide => False, others => <>));
         Arm.Result.Chain.Append (J);
         Arm.Result.Reference.Append (0.0);
      end loop;
      Arm.Result.Fitted := True;
      Arm.Result.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Arm.Result.Sigma_Px := 0.1;
      M.Kinematics.Append (Arm);
      M.Groups.Append (Group_Stream'(Size => 2, Commandable => True, others => <>));
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      O.Readings.Append (Start);
   end Planar_Arm;

   --  The joints' straight line can take the tool out along its path ahead of
   --  where the straight path has it and back behind it, and be exactly on it
   --  at the middle: two joints about parallel axes with equal links, from
   --  readings zero to 3 and -6, put the eye on a line with its share at the
   --  quarter well ahead and at the three quarters well behind. A chain of
   --  halved segments can wind a joint so, though the nearest solution of one
   --  does not. Tested at the middle alone it is not seen.
   procedure Stray_Is_Seen_Off_The_Middle is
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Motion renames Driver.Robot.Motion;
      M      : Model;
      Truth  : Fit.Joint_Array (1 .. 2);
      O      : Observation;
      Start  : constant Real_Array (1 .. 2) := [others => 0.0];
      Goal_Q : constant Real_Array (1 .. 2) := [3.0, -6.0];
   begin
      Planar_Arm (M, Truth, O, Start, 1.0, 1.0);
      declare
         From   : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Start);
         To     : constant Rigid := Fit.Eye_At (Truth, Along (Truth), Goal_Q);
         Middle : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, [1.5, -3.0]);
         At_Mid : constant Real := abs (Middle.Translation - (From.Translation + To.Translation) / 2.0);
         Seen   : constant Real := Motion.Stray (M, 1, From, To, Start, Goal_Q, Position_Only => True);
      begin
         Driver.Log.Line (Driver.Log.Robot, "stray test: at the middle" & Real'Image (At_Mid) & ", seen" & Real'Image (Seen));
         Check (Seen > 2.0 * At_Mid,
                "a bow that is on the straight path at its middle and well off it at its quarters is seen as"
                & Real'Image (Seen) & ", the middle alone showing" & Real'Image (At_Mid));
      end;
   end Stray_Is_Seen_Off_The_Middle;

   --  A plan that is given a clearance and cannot keep to it, for the straight
   --  path leaves what the arm can reach, says how much it is left with: it
   --  strays no more than the clearance, or no more than the bow it says.
   procedure A_Bow_That_Is_Left_Is_Said is
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Motion renames Driver.Robot.Motion;
      use type Motion.Plan_Status;
      M      : Model;
      Truth  : Fit.Joint_Array (1 .. 6);
      O      : Observation;
      Start  : constant Real_Array (1 .. 6) := [others => 0.0];
      Asked  : constant Real := 0.001;
      G      : Generator;
      Planned_Count, Left_Count, Lying : Natural := 0;
   begin
      Planner_Arm (M, Truth, O, Start);
      declare
         From  : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Start);
         Sigma : constant Real := Driver.Robot.Kinematics.Angle_Sigma (M, 1);
      begin
         for K in 1 .. 200 loop
            declare
               Q    : Real_Array (1 .. 6);
            begin
               for J in 1 .. 6 loop
                  Q (J) := 2.4 * Uniform (G) - 1.2;
               end loop;
               declare
                  Goal : constant Rigid := Fit.Eye_At (Truth, Along (Truth), Q);
                  Held : constant Motion.Plan :=
                    Motion.Plan_Reach_In_Arm (M, 1, O, (Pose => Goal, Position_Only => True, Free_About => <>), Clearance => Asked);
               begin
                  if Motion.Status (Held) = Motion.Planned and then abs (Goal.Translation - From.Translation) > Sigma then
                     declare
                        Line : constant Vec3 := Unit (Goal.Translation - From.Translation);
                        Off  : constant Real := Plan_Stray (M, Held, Start, From, Line);
                        Bow  : constant Real := Motion.Worst_Bow (Held);
                     begin
                        Planned_Count := Planned_Count + 1;
                        if Bow > 0.0 then
                           Left_Count := Left_Count + 1;
                        end if;
                        --  Off the straight line by more than asked is a bow, and a bow left is said.
                        if Off > Asked + Driver.Conventions.Z * Sigma and then Bow = 0.0 then
                           Lying := Lying + 1;
                        end if;
                     end;
                  end if;
               end;
            end;
         end loop;
         Driver.Log.Line (Driver.Log.Robot, "bow test: planned" & Planned_Count'Image & " of 200, a bow left in" & Left_Count'Image
                          & ", strayed unsaid in" & Lying'Image);
         Check (Left_Count > 0, "no goal here leaves a bow, so the test shows nothing");
         Check (Lying = 0, Lying'Image & " plans stray off the straight path by more than the clearance and say they do not");
      end;
   end A_Bow_That_Is_Left_Is_Said;

   --  A fit belongs to an arm only while the graph has its group as that arm,
   --  carrying that eye: once the group stops being an arm, or the eye rides
   --  on another, the fit is no arm's, and the refit clears it.
   procedure Stale_Fit_Is_No_Arms is
      M   : Model;
      Arm : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
   begin
      for J in 1 .. 2 loop
         Arm.Result.Joints.Append (Joint_Fit'(W => [0.0, 0.0, 1.0], P => [0.1, 0.0, 0.0], C => 1.0, Slide => False, others => <>));
         Arm.Result.Chain.Append (J);
         Arm.Result.Reference.Append (0.0);
      end loop;
      Arm.Result.Fitted := True;
      M.Kinematics.Append (Arm);
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      Check (Driver.Robot.Kinematics.Fitted (M, 1), "the arm's own fit is not found");
      --  Eye 1 is found fixed in the world: group 1 carries no eye.
      M.Graph.Mounts.Replace_Element (1, Mount'(Kind => World_Fixed));
      Check (not Driver.Robot.Kinematics.Fitted (M, 1), "a fit of an eye the arm no longer carries is the arm's");
      M.Graph.Mounts.Replace_Element (1, Mount'(Kind => Arm_Carried, Arm => 1));
      --  Group 1 stops being an arm.
      M.Graph.Arms.Clear;
      Check (not Driver.Robot.Kinematics.Fitted (M, 1), "the fit of a group that stopped being an arm is an arm's");
      Driver.Robot.Kinematics.Refit (M);
      Check (not M.Kinematics (1).Result.Fitted, "the refit keeps the fit of a group that is no arm");
   end Stale_Fit_Is_No_Arms;

   --  The arm of Build_Fitted_Arm: its eye's lens, the table it sees, and
   --  the matcher's error, pixels per coordinate.
   Arm_Lens  : constant Driver.Robot.Kinematics.Fit.Lens :=
     (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
   Arm_Table : constant Vec3 := [0.0, -0.6, -0.8];
   Arm_Noise : constant := 0.1;

   --  A keyframe of the arm at Readings, and the matches of the reference's
   --  query points into it: what the eye of the true joints sees of the
   --  table, with the matcher's error. The reference is the first keyframe,
   --  at Ref.
   procedure Add_Keyframe
     (R        : in out Arm_Evidence;
      Readings : Real_Array;
      Ref      : Real_Array;
      Truth    : Driver.Robot.Kinematics.Fit.Joint_Array;
      Rng      : in out Generator;
      Base     : Real_Array)
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      K : Keyframe;
   begin
      K.Beat := Natural (R.Frames.Length) + 1;
      for X of Readings loop
         K.Readings.Append (X);
      end loop;
      --  What the sweep asked: every joint away from the sweep's base (the reference may lie off it).
      for J in Readings'Range loop
         K.Asked.Append (Readings (J) /= Base (Base'First + J - Readings'First));
      end loop;
      R.Frames.Append (K);
      if Natural (R.Frames.Length) > 1 then
         declare
            D   : Real_Array (1 .. Readings'Length);
            T   : Rigid;
            Set : Match_Set;
         begin
            for J in D'Range loop
               D (J) := Readings (Readings'First + J - 1) - Ref (Ref'First + J - 1);
            end loop;
            T := Inverse (Fit.Eye_At (Truth, Along (Truth), D));
            Set.Frame := Natural (R.Frames.Length);
            for I in 0 .. Natural (R.Query_U.Length) - 1 loop
               declare
                  H     : constant Vec3 := Fit.Ray (Arm_Lens, R.Query_U (I), R.Query_V (I));
                  X     : constant Vec3 := (-1.0 / Real'(Unit (Arm_Table) * H)) * H;
                  U, V  : Real;
                  Ahead : Boolean;
               begin
                  Fit.Project (Arm_Lens, T * X, U, V, Ahead);
                  Set.To_U.Append (U + Arm_Noise * Gaussian (Rng));
                  Set.To_V.Append (V + Arm_Noise * Gaussian (Rng));
                  Set.Back_U.Append (R.Query_U (I) + Arm_Noise * Gaussian (Rng));
                  Set.Back_V.Append (R.Query_V (I) + Arm_Noise * Gaussian (Rng));
                  Set.Found.Append (Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0);
               end;
            end loop;
            R.Matches.Append (Set);
         end;
      end if;
   end Add_Keyframe;

   --  A10's arm 2: its reference keyframe came from a push of the
   --  recognition rounds, 3e-5 rad down on one joint, so every keyframe of
   --  its sweep differs from the reference in that joint too. That is ten
   --  times the step the eye's lock-in can tell over many beats, and a
   --  twentieth of what one keyframe's match can: the fit must judge the
   --  keyframes by the match, or no joint has a keyframe of its own (the
   --  replay of A10, its lock-in steps grown finer, fitted no arm 2).
   --  Builds an arm of six joints, its eye's lock-in, its graph, and its
   --  kinematics fitted from synthetic matches of a sweep whose reference
   --  keyframe lies Offset rad off the sweep's base on joint 3.
   procedure Build_Fitted_Arm (M : in out Model; Offset : Real) is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Levels  : constant Real_Array := [0.05, -0.05, 0.1, -0.1, 0.2, -0.2];
      Rows_H  : constant := 7;          --  the Hadamard keyframes
      Columns : constant := 16;
      Rows    : constant := 12;
      Truth   : Fit.Joint_Array (1 .. N);
      Axes    : constant array (1 .. N) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Points  : constant array (1 .. N) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      Base    : constant Real_Array (1 .. N) := [others => 0.0];
      Ref     : Real_Array (1 .. N) := Base;
      R       : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
      Rng     : Generator;
      Cells   : constant := 4;

      procedure Add_Frame (Readings : Real_Array) is
      begin
         Add_Keyframe (R, Readings, Ref, Truth, Rng, Base);
      end Add_Frame;
   begin
      for J in 1 .. N loop
         declare
            W : constant Vec3 := Unit (Axes (J));
         begin
            Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
         end;
      end loop;
      --  The body: one arm of six channels carrying one eye, whose lock-in,
      --  like any over many beats, sees steps far finer than a keyframe's
      --  match: 1.3e-8 rad against 6.5e-4.
      M.Groups.Append (Group_Stream'(Size => N, Commandable => True, others => <>));
      declare
         S : Eye_Stream;
      begin
         S.Grid := (Width => 640, Height => 480, Columns => 2, Rows => 2);
         for C in 1 .. N loop
            S.Kept_Groups.Append (1);
            S.Kept_Channels.Append (C);
         end loop;
         for Cell in 1 .. Cells loop
            S.Noise.Append (Arm_Noise);
            for C in 1 .. N loop
               S.Gains.Append (1.0e16);
               S.Gain_Variances.Append (1.0);
               S.Shifts.Append (400.0);
            end loop;
         end loop;
         M.Eyes.Append (S);
      end;
      M.Graph.Effects.Append (Eye_Effect'(Verdict => Whole, Responding => Cells, Textured => Cells, others => <>));
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      for Gy in 1 .. Rows loop
         for Gx in 1 .. Columns loop
            R.Query_U.Append ((Real (Gx) - 0.5) * 640.0 / Real (Columns));
            R.Query_V.Append ((Real (Gy) - 0.5) * 480.0 / Real (Rows));
         end loop;
      end loop;
      --  The reference off the base, its still twin, then the sweep from the
      --  base: every joint at every level, and the Hadamard rows.
      Ref (3) := -Offset;
      Add_Frame (Ref);
      Add_Frame (Ref);
      for J in 1 .. N loop
         for L of Levels loop
            declare
               Q : Real_Array := Base;
            begin
               Q (J) := L;
               Add_Frame (Q);
            end;
         end loop;
      end loop;
      for Row in 1 .. Rows_H loop
         declare
            Q : Real_Array := Base;
         begin
            for J in 1 .. N loop
               declare
                  Bits : Natural := 0;
                  A    : Natural := Row;
                  C    : Natural := J;
               begin
                  while A > 0 and then C > 0 loop
                     if A mod 2 = 1 and then C mod 2 = 1 then
                        Bits := Bits + 1;
                     end if;
                     A := A / 2;
                     C := C / 2;
                  end loop;
                  Q (J) := (if Bits mod 2 = 0 then 0.05 else -0.05);
               end;
            end loop;
            Add_Frame (Q);
         end;
      end loop;
      M.Kinematics.Append (R);
      Driver.Robot.Kinematics.Refit (M);
   end Build_Fitted_Arm;

   --  Two arms of six joints, each carrying its eye, the second one and a half
   --  times the first's size, seeing one table: the first arm's reference eye
   --  is the world, the second's stands at Second (X_world = Second * X_eye,
   --  in the units the scene is drawn in). Both sweeps' keyframes and the
   --  first arm's reference matched into the second's are synthetic matches
   --  with 0.1 px of noise. Truth_Of gives each arm's true joints.
   Sizes : constant array (1 .. 2) of Real := [1.0, 1.5];

   function Truth_Of (A : Positive) return Driver.Robot.Kinematics.Fit.Joint_Array is
      Axes   : constant array (1 .. 6) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Points : constant array (1 .. 6) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      Truth  : Driver.Robot.Kinematics.Fit.Joint_Array (1 .. 6);
   begin
      for J in 1 .. 6 loop
         declare
            W : constant Vec3 := Unit (Axes (J));
            P : constant Vec3 := Sizes (A) * Points (J);
         begin
            Truth (J) := (W => W, P => P - Real'(P * W) * W, C => 1.0, Slide => False);
         end;
      end loop;
      return Truth;
   end Truth_Of;

   --  The keyframes both arms' sweeps take: the reference and its still twin,
   --  every joint at every level, the Hadamard rows.
   function Sweep_Changes return Real_Matrix is
      N      : constant := 6;
      Levels : constant Real_Array := [0.05, -0.05, 0.1, -0.1, 0.2, -0.2];
      Rows_H : constant := 7;
      C      : Real_Matrix (1 .. 2 + N * Levels'Length + Rows_H, 1 .. N) := [others => [others => 0.0]];
   begin
      for J in 1 .. N loop
         for L in Levels'Range loop
            C (2 + (J - 1) * Levels'Length + L, J) := Levels (L);
         end loop;
      end loop;
      for Row in 1 .. Rows_H loop
         for J in 1 .. N loop
            declare
               Bits : Natural := 0;
               A    : Natural := Row;
               B    : Natural := J;
            begin
               while A > 0 and then B > 0 loop
                  if A mod 2 = 1 and then B mod 2 = 1 then
                     Bits := Bits + 1;
                  end if;
                  A := A / 2;
                  B := B / 2;
               end loop;
               C (2 + N * Levels'Length + Row, J) := (if Bits mod 2 = 0 then 0.05 else -0.05);
            end;
         end loop;
      end loop;
      return C;
   end Sweep_Changes;

   --  The scene the arms see: a table one unit from the first eye, its normal
   --  Table_N towards that eye in the world, and boxes standing on it. A box
   --  is a top at Height above the table over the rectangle X0 .. X1 along
   --  Table_X and Y0 .. Y1 along Table_Y (its sides are not drawn: a line of
   --  sight past a box top meets the table).
   Table_N : constant Vec3 := Unit ([0.0, -0.6, -0.8]);
   Table_O : constant Real := -1.0;
   Table_X : constant Vec3 := [1.0, 0.0, 0.0];
   Table_Y : constant Vec3 := Cross (Table_N, Table_X);
   Rig_Lens : constant Driver.Robot.Kinematics.Fit.Lens :=
     (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);

   type Box is record
      X0, X1, Y0, Y1, Height : Real := 0.0;
   end record;

   type Box_Array is array (Positive range <>) of Box;

   --  Three under each arm's view, about half of what each eye sees.
   Boxes : constant Box_Array :=
     [(-0.7, 0.3, 0.2, 0.9, 0.2), (0.35, 1.4, 0.8, 1.7, 0.3), (-1.5, -0.2, 1.2, 2.1, 0.15),
      (3.9, 4.4, 0.2, 0.8, 0.25), (4.5, 5.0, 0.25, 0.75, 0.12), (4.85, 5.9, 0.8, 1.7, 0.1), (3.0, 4.3, 1.2, 2.1, 0.2)];

   --  What the line of sight through (U, V) of an eye at Placed in the world
   --  (X_world = Placed * X_eye) meets, in that eye's frame: the nearest box
   --  top over its rectangle when With_Boxes, else the table; On_Table says
   --  which.
   procedure Scene_Point
     (Placed : Rigid; U, V : Real; With_Boxes : Boolean; X : out Vec3; On_Table : out Boolean;
      Standing : Box_Array := Boxes)
   is
      H : constant Vec3 := Driver.Robot.Kinematics.Fit.Ray (Rig_Lens, U, V);
      D : constant Vec3 := Placed.Rotation * H;
      C : constant Vec3 := Placed.Translation;
      --  How far along H the plane of offset O (Table_N * X = O) is.
      function Along (O : Real) return Real is ((O - Real'(Table_N * C)) / Real'(Table_N * D));
      Best : Real := Along (Table_O);
   begin
      On_Table := True;
      if With_Boxes then
         for B of Standing loop
            declare
               S : constant Real := Along (Table_O + B.Height);
               P : constant Vec3 := C + S * D;
               Px : constant Real := P * Table_X;
               Py : constant Real := P * Table_Y;
            begin
               if S > 0.0 and then S < Best and then Px in B.X0 .. B.X1 and then Py in B.Y0 .. B.Y1 then
                  Best := S;
                  On_Table := False;
               end if;
            end;
         end loop;
      end if;
      X := Best * H;
   end Scene_Point;

   --  How the arms' world is drawn. The first arm's reference eye is the world; the second's stands at Second.
   --  Head, when there is one, is a third eye fixed in the world at Head_Pose that saw both arms turn: placed as the
   --  arms' edge flow places it (Driver.Robot.Flows), its lens and its camera in each arm's frame drawn about the
   --  truth with the sigmas it states (Head_Noise pixels at its lens).
   type Rig_Scene is record
      Second        : Rigid := Driver.Numerics.Identity;
      With_Boxes    : Boolean := False;
      Head          : Boolean := False;
      Head_Pose     : Rigid := Driver.Numerics.Identity;
      Head_Noise    : Real := 0.1;        --  how far the head's fit leaves a point in its picture, pixels
      Head_Lens     : Driver.Robot.Kinematics.Fit.Lens := Rig_Lens;   --  the head's own lens
      Seed          : Long_Long_Integer := 1;   --  the generator of the matches' errors starts here
      Arm_Noise     : Real := 0.1;        --  how far the arms' own matches err, pixels per coordinate
   end record;

   --  How long the arm's fit unit is in the units the scene is drawn in: its
   --  fitted eye positions over its keyframes against the true ones, by least
   --  squares (the fit makes their root mean square one unit).
   function Unit_Of (M : Model; A : Positive) return Real is
      Changes : constant Real_Matrix := Sweep_Changes;
      Num, Den : Real := 0.0;
   begin
      for F in Changes'Range (1) loop
         declare
            D : Real_Array (1 .. 6);
         begin
            for J in 1 .. 6 loop
               D (J) := Changes (F, J);
            end loop;
            declare
               T : constant Vec3 :=
                 Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (A), Along (Truth_Of (A)), D).Translation;
               G : constant Vec3 := Driver.Robot.Kinematics.Eye_In_Reference (M, Arm_Id (A), D).Translation;
            begin
               Num := Num + T * G;
               Den := Den + G * G;
            end;
         end;
      end loop;
      return Num / Den;
   end Unit_Of;

   procedure Build_Two_Arms (M : in out Model; Scene : Rig_Scene) is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant := 0.1;
      Changes : constant Real_Matrix := Sweep_Changes;
      Placed  : constant array (1 .. 2) of Rigid := [Driver.Numerics.Identity, Scene.Second];
      Arms    : array (1 .. 2) of Arm_Evidence;
      Rng     : Generator := (State => Scene.Seed);
      Cells   : constant := 4;
      Eyes    : constant Positive := (if Scene.Head then 3 else 2);

      --  From_Arm's reference query points, seen by an eye whose frame
      --  Seen_From maps From_Arm's reference frame into.
      procedure Match (Seen_From : Rigid; Frame : Positive; Into : in out Match_Set_Vectors.Vector;
                       From_Arm : Positive; Error : Real := Noise)
      is
         Set : Match_Set;
      begin
         Set.Frame := Frame;
         for I in 0 .. Natural (Arms (From_Arm).Query_U.Length) - 1 loop
            declare
               U0    : constant Real := Arms (From_Arm).Query_U (I);
               V0    : constant Real := Arms (From_Arm).Query_V (I);
               X     : Vec3;
               On_Table : Boolean;
               U, V  : Real;
               Ahead : Boolean;
               Shown : Boolean;
            begin
               Scene_Point (Placed (From_Arm), U0, V0, Scene.With_Boxes, X, On_Table);
               Fit.Project (Rig_Lens, Seen_From * X, U, V, Ahead);
               Shown := Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0;
               Set.To_U.Append (U + Error * Gaussian (Rng));
               Set.To_V.Append (V + Error * Gaussian (Rng));
               Set.Back_U.Append (U0 + Error * Gaussian (Rng));
               Set.Back_V.Append (V0 + Error * Gaussian (Rng));
               Set.Found.Append (Shown);
            end;
         end loop;
         Into.Append (Set);
      end Match;

   begin
      for A in 1 .. 2 loop
         M.Groups.Append (Group_Stream'(Size => N, Commandable => True, others => <>));
      end loop;
      for E in 1 .. Eyes loop
         declare
            S : Eye_Stream;
         begin
            S.Grid := (Width => 640, Height => 480, Columns => 2, Rows => 2);
            if E <= 2 then
               for C in 1 .. N loop
                  S.Kept_Groups.Append (E);
                  S.Kept_Channels.Append (C);
               end loop;
               for Cell in 1 .. Cells loop
                  S.Noise.Append (Noise);
                  for C in 1 .. N loop
                     S.Gains.Append (1.0e16);
                     S.Gain_Variances.Append (1.0);
                     S.Shifts.Append (400.0);
                  end loop;
               end loop;
            end if;
            M.Eyes.Append (S);
         end;
      end loop;
      --  Each group moves its own eye's whole view; the head shows both arms move.
      for G in 1 .. 2 loop
         for E in 1 .. Eyes loop
            M.Graph.Effects.Append
              (if E = G then Eye_Effect'(Verdict => Whole, Responding => Cells, Textured => Cells, others => <>)
               elsif E = 3 then Eye_Effect'(Verdict => Patch, Responding => 1, Textured => Cells, others => <>)
               else Eye_Effect'(Verdict => Nothing, Textured => Cells, others => <>));
         end loop;
      end loop;
      for A in 1 .. 2 loop
         M.Graph.Arms.Append (Group_Id (A));
         M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => Arm_Id (A)));
      end loop;
      if Scene.Head then
         M.Graph.Mounts.Append (Mount'(Kind => World_Fixed));
      end if;
      for A in 1 .. 2 loop
         Arms (A) := (Arm => Arm_Id (A), Group => Group_Id (A), Eye => Eye_Id (A), others => <>);
         for Gy in 1 .. Rows loop
            for Gx in 1 .. Columns loop
               Arms (A).Query_U.Append ((Real (Gx) - 0.5) * 640.0 / Real (Columns));
               Arms (A).Query_V.Append ((Real (Gy) - 0.5) * 480.0 / Real (Rows));
            end loop;
         end loop;
         for F in Changes'Range (1) loop
            declare
               K : Keyframe;
               D : Real_Array (1 .. N);
            begin
               K.Beat := F;
               for J in 1 .. N loop
                  K.Readings.Append (Changes (F, J));
                  --  What the sweep asked: every joint away from the base the frames are drawn about.
                  K.Asked.Append (Changes (F, J) /= 0.0);
                  D (J) := Changes (F, J);
               end loop;
               Arms (A).Frames.Append (K);
               if F > 1 then
                  Match (Inverse (Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (A), Along (Truth_Of (A)), D)), F, Arms (A).Matches, A,
                         Scene.Arm_Noise);
               end if;
            end;
         end loop;
      end loop;
      M.Kinematics.Append (Arms (1));
      M.Kinematics.Append (Arms (2));
      Driver.Robot.Kinematics.Refit (M);
      --  The head as the arms' flow fit leaves it: its camera in each arm's reference frame, in that arm's unit, turned
      --  about its own axes by Head_Noise pixels over its focal length and its centre moved by that over its distance,
      --  each way at random, the lens its own; then the arms are placed again, through it.
      if Scene.Head then
         declare
            Terms      : constant Natural := Fit.Lens_Terms + 6 * 2;
            Turn_Sigma : constant Real := Scene.Head_Noise / Scene.Head_Lens.Fx;
            Joint      : Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];
            F          : Fixed_Fit;
         begin
            F.Lens := (Fx => Scene.Head_Lens.Fx, Fy => Scene.Head_Lens.Fy, Cx => Scene.Head_Lens.Cx,
                       Cy => Scene.Head_Lens.Cy, K1 => Scene.Head_Lens.K1, K2 => Scene.Head_Lens.K2);
            for T in 1 .. 2 loop
               Joint (T, T) := Turn_Sigma ** 2;   --  the focal lengths' logarithms
            end loop;
            for T in 3 .. 4 loop
               Joint (T, T) := Scene.Head_Noise ** 2;   --  the principal point
            end loop;
            for A in 1 .. 2 loop
               declare
                  Unit         : constant Real := Unit_Of (M, A);
                  True_In_A    : constant Rigid := Inverse (Placed (A)) * Scene.Head_Pose;
                  Centre_Sigma : constant Real := Turn_Sigma * abs True_In_A.Translation / Unit;
                  O            : constant Natural := Fit.Lens_Terms + 6 * (A - 1);
                  Turn         : constant Vec3 :=
                    [Turn_Sigma * Gaussian (Rng), Turn_Sigma * Gaussian (Rng), Turn_Sigma * Gaussian (Rng)];
                  Shift        : constant Vec3 :=
                    [Centre_Sigma * Gaussian (Rng), Centre_Sigma * Gaussian (Rng), Centre_Sigma * Gaussian (Rng)];
               begin
                  F.Arms.Append (A);
                  F.Poses.Append (Rigid'(Rotation    => True_In_A.Rotation * Driver.Numerics.Exp (Turn),
                                         Translation => (1.0 / Unit) * True_In_A.Translation + Shift));
                  for K in 1 .. 3 loop
                     Joint (O + K, O + K) := Turn_Sigma ** 2;
                     Joint (O + 3 + K, O + 3 + K) := Centre_Sigma ** 2;
                  end loop;
               end;
            end loop;
            for P in 1 .. Terms loop
               for Q in 1 .. Terms loop
                  F.Joint.Append (Joint (P, Q));
               end loop;
            end loop;
            for P in 1 .. Fit.Lens_Terms + 6 loop
               for Q in 1 .. Fit.Lens_Terms + 6 loop
                  F.Covariance.Append (Joint (P, Q));
               end loop;
            end loop;
            F.Known := True;
            F.Arm := 1;
            F.Pose := F.Poses (F.Poses.First_Index);
            F.Sigma_Px := Scene.Head_Noise;
            F.Judged := True;
            while M.Fixed_Eyes.Last_Index < 3 loop
               M.Fixed_Eyes.Append (Fixed_Fit'(others => <>));
            end loop;
            M.Fixed_Eyes (3) := F;
         end;
         Driver.Robot.Kinematics.Refit (M);
      end if;
   end Build_Two_Arms;

   --  The second arm placed in the first arm's world: where its eye stands,
   --  how it is turned, the scale of its lengths, and its eye at a pose no
   --  keyframe had, against the true scene in the first arm's units, each
   --  within Z of its own sigma; and the eye that placed it.
   procedure Check_Placement (M : Model; Second : Rigid; Through : Natural; What : String) is
      Placement : Rigid;
      Scale     : Real;
      Known     : Boolean;
   begin
      Check (Driver.Robot.Kinematics.Fitted (M, 1) and then Driver.Robot.Kinematics.Fitted (M, 2),
             What & ": the arms are not fitted");
      Driver.Robot.Kinematics.In_World (M, 2, Placement, Scale, Known);
      Check (Known, What & ": the second arm is not placed in the world");
      if not Known then
         return;
      end if;
      declare
         U1 : constant Real := Unit_Of (M, 1);
         U2 : constant Real := Unit_Of (M, 2);
         R  : Arm_Fit renames M.Kinematics (2).Result;
         function C (P, Q : Positive) return Real is (R.Placement_Covariance ((P - 1) * 7 + Q - 1));
         Turn_Sigma   : constant Real := Sqrt (C (1, 1) + C (2, 2) + C (3, 3));
         Centre_Sigma : constant Real := Sqrt (C (4, 4) + C (5, 5) + C (6, 6));
         Turned : constant Real := Driver.Numerics.Angle (Transpose (Placement.Rotation) * Second.Rotation);
         Moved  : constant Real := abs (Placement.Translation - (1.0 / U1) * Second.Translation);
      begin
         Check (R.Placed_Through = Through,
                What & ": placed through eye" & R.Placed_Through'Image & ", not eye" & Through'Image);
         Check (Turned <= Driver.Conventions.Z * Turn_Sigma,
                What & ": the second eye is turned by" & Turned'Image & " rad, its sigma" & Turn_Sigma'Image);
         Check (Moved <= Driver.Conventions.Z * Centre_Sigma,
                What & ": the second eye is off by" & Moved'Image & " world units, its sigma" & Centre_Sigma'Image);
         Check (abs (Scale - U2 / U1) <= Driver.Conventions.Z * R.Scale_Sigma,
                What & ": the second arm's scale is" & Scale'Image & " against" & Real'Image (U2 / U1) & ", its sigma"
                & R.Scale_Sigma'Image);
         --  Its eye at a pose no keyframe had, in the world.
         declare
            Q        : constant Real_Array (1 .. 6) := [0.15, -0.1, 0.08, -0.12, 0.1, -0.15];
            True_Eye : constant Rigid := Second * Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (2), Along (Truth_Of (2)), Q);
            E        : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 2, Q);
            Got      : constant Rigid := (Rotation    => Placement.Rotation * E.Rotation,
                                          Translation => Placement.Rotation * (Scale * E.Translation) + Placement.Translation);
            Turn, Place : Mat3;
            Off      : constant Real := abs (Got.Translation - (1.0 / U1) * True_Eye.Translation);
            Off_Turn : constant Real := Driver.Numerics.Angle (Transpose (Got.Rotation) * True_Eye.Rotation);
         begin
            Driver.Robot.Kinematics.World_Pose_Covariance (M, 2, Q, Turn, Place);
            Check (Off <= Driver.Conventions.Z * Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3)),
                   What & ": the second eye at a new pose is off by" & Off'Image & " world units, its sigma"
                   & Real'Image (Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3))));
            Check (Off_Turn <= Driver.Conventions.Z * Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3)),
                   What & ": the second eye at a new pose is turned by" & Off_Turn'Image & " rad, its sigma"
                   & Real'Image (Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3))));
            Driver.Log.Line (Driver.Log.Robot, "placement test (" & What & "): turned" & Turned'Image & " sigma"
                             & Turn_Sigma'Image & "; off" & Moved'Image & " sigma" & Centre_Sigma'Image & "; scale"
                             & Scale'Image & " against" & Real'Image (U2 / U1) & " sigma" & R.Scale_Sigma'Image
                             & "; new pose off" & Off'Image & " sigma"
                             & Real'Image (Sqrt (Place (1, 1) + Place (2, 2) + Place (3, 3)))
                             & ", turned" & Off_Turn'Image & " sigma" & Real'Image (Sqrt (Turn (1, 1) + Turn (2, 2) + Turn (3, 3))));
         end;
      end;
   end Check_Placement;

   --  Two arms whose eyes never see each other's table, four and a half units
   --  apart, and a head: an eye fixed behind and above them that watches both
   --  arms move, a few units from them.
   Far_Second : constant Rigid := (Rotation => Driver.Numerics.Exp ([0.05, -0.1, 0.03]), Translation => [4.5, 0.05, 0.1]);

   function Head_Pose return Rigid is
      Between : constant Vec3 := [2.4, 0.3, 0.35];   --  between the two arms' hands
      Eye     : constant Vec3 := Between + [0.0, -1.5, -4.0];
      Z_Axis  : constant Vec3 := Unit (Between - Eye);
      Y_Axis  : constant Vec3 := Cross (Z_Axis, [1.0, 0.0, 0.0]);
   begin
      return (Rotation    => [[1.0, Y_Axis (1), Z_Axis (1)],
                              [0.0, Y_Axis (2), Z_Axis (2)],
                              [0.0, Y_Axis (3), Z_Axis (3)]],
              Translation => Eye);
   end Head_Pose;

   --  The scene the head tests share: two arms four and a half units apart with boxes on their table, and the head
   --  above and behind them, placed by its fit to both arms.
   procedure Head_Scene (M : in out Model) is
   begin
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose, others => <>));
      Check (Driver.Robot.Kinematics.Fixed_Known (M, 3), "the head is not placed");
   end Head_Scene;

   --  Placed through the head, against the truth.
   procedure Place_Through_A_Head is
      M : Model;
   begin
      Head_Scene (M);
      Check_Placement (M, Far_Second, 3, "through the head");
   end Place_Through_A_Head;

   --  The head's answers five times as precise as the arms' own matches: the arms' lenses and tables, measured from
   --  their own sweeps, now err by more than the head's view of them does. Placed through the head, within its sigma:
   --  the placement holds each arm's table to its own uncertainty, not to its estimate exactly.
   procedure Place_Through_A_Precise_Head is
      M : Model;
   begin
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose,
                          Head_Noise => 0.02, others => <>));
      Check_Placement (M, Far_Second, 3, "through a precise head");
   end Place_Through_A_Precise_Head;

   --  A head whose lens is not the wrists': its principal point lies 32 and 35 pixels from the picture's middle and
   --  its two focal lengths differ.
   Off_Centre_Head : constant Driver.Robot.Kinematics.Fit.Lens :=
     (Fx => 330.0, Fy => 345.0, Cx => 352.0, Cy => 205.0, K1 => 0.0, K2 => 0.0);

   --  The scene the head tests share: two arms with a table and boxes on it, and the head above and behind them with
   --  a lens of its own, its fit leaving points Head_Noise pixels uncertain in its picture.
   function Head_Rig (Head_Noise : Real := 0.1; Seed : Long_Long_Integer := 1) return Rig_Scene is
     ((Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose, Head_Lens => Off_Centre_Head,
       Head_Noise => Head_Noise, Seed => Seed, others => <>));

   --  The rest of what a boot measures of the two-arm rig, set to values it could have measured, for its body file to
   --  be reloaded whole.
   procedure Measured_Rig (M : in out Model) is
   begin
      for G in Group_Id range 1 .. 2 loop
         for C in 1 .. 6 loop
            M.Noise.Append (1.0e-6 * Real (C) / 3.0);
            M.Noise_Freedom.Append (100 + C);
            M.Groups (G).Low_Seen.Append (-0.2 - 0.01 * Real (C));
            M.Groups (G).High_Seen.Append (0.2 + 0.01 * Real (C));
         end loop;
         M.Groups (G).Delay_Beats := 2;
         M.Groups (G).Delay_Known := True;
         M.Graph.Roles.Append (Arm);
         M.Graph.Arm_Of.Append (Arm_Id (G));
         M.Graph.Breach.Append (0);
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         M.Lags.Append (1);
         M.Lag_Known.Append (True);
         M.Eyes (E).Rest_Factor := 1.0 + 1.0 / 7.0;
         M.Eyes (E).Rest_Counts_Known := True;
         M.Eyes (E).Rest_Count_Max := 3;
         M.Eyes (E).Rest_Count_Beats := 41;
         for Cell in 1 .. 4 loop
            M.Eyes (E).Textured.Append (Cell /= 2);
         end loop;
      end loop;
   end Measured_Rig;

   --  The head's place is part of the body: written, read back, it is the same fit with the same covariance, and
   --  the reloaded body answers as the measured one did.
   procedure Head_In_The_Body_File is
      M, Back : Model;
      O       : Observation;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Build_Two_Arms (M, Head_Rig);
      Check (Driver.Robot.Kinematics.Fixed_Known (M, 3), "the head to write is not placed");
      Measured_Rig (M);
      declare
         Written : constant String := Driver.Robot.Body_File.Text (M);
      begin
         Driver.Robot.Body_File.Read (Back, Written, Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         Check (Reloaded (Back, Stored_Kinematics), "the kinematics were not reloaded: "
                & Ada.Strings.Unbounded.To_String (Why));
         Check (Driver.Robot.Kinematics.Fixed_Known (Back, 3), "the reloaded body has no place for the head");
         Check (Driver.Robot.Body_File.Text (Back) = Written, "the reloaded body writes another file");
         if Driver.Robot.Kinematics.Fixed_Known (Back, 3) then
            declare
               Px   : constant Driver.Images.Pixel := (U => 100.0, V => 400.0);
               Was  : constant Ray_Estimate := Ray (M, 3, O, Px);
               Now  : constant Ray_Estimate := Ray (Back, 3, O, Px);
               PW   : constant Pose_Estimate := Eye_Pose (M, 3, O);
               PN   : constant Pose_Estimate := Eye_Pose (Back, 3, O);
            begin
               Check (Was.Direction.Unit_Vector = Now.Direction.Unit_Vector
                      and then Was.Direction.Sigma = Now.Direction.Sigma
                      and then Was.Origin.Mean = Now.Origin.Mean and then Was.Origin.Covariance = Now.Origin.Covariance,
                      "the reloaded head gives another line of sight");
               Check (PW.Pose.Rotation = PN.Pose.Rotation and then PW.Pose.Translation = PN.Pose.Translation
                      and then PW.Position_Covariance = PN.Position_Covariance
                      and then PW.Rotation_Covariance = PN.Rotation_Covariance,
                      "the reloaded head stands elsewhere, or less certainly");
            end;
         end if;
      end;
   end Head_In_The_Body_File;

   --  How far a plane estimate lies from the true plane True_N * X = True_O
   --  (True_N towards the eye) in units of its own uncertainty: the length of
   --  its whitened error, its offset at its centre and its tilt.
   function Plane_Off (P : Driver.Geometry.Plane_Estimate; True_N : Vec3; True_O : Real) return Real is
      A   : constant Real := True_N * P.Tangent_1;
      B   : constant Real := True_N * P.Tangent_2;
      H   : constant Real := True_N * P.Centre - True_O;
      Det : constant Real := P.Tilt_11 * P.Tilt_22 - P.Tilt_12 ** 2;
   begin
      return Sqrt ((H / P.Offset_Sigma) ** 2 + (P.Tilt_22 * A * A - 2.0 * P.Tilt_12 * A * B + P.Tilt_11 * B * B) / Det);
   end Plane_Off;

   --  The true table in the frame of arm A's reference eye, in the arm's unit:
   --  its normal towards the eye and its offset.
   procedure True_Table (M : Model; A : Positive; Placed : Rigid; Normal : out Vec3; Offset : out Real) is
   begin
      Normal := Transpose (Placed.Rotation) * Table_N;
      Offset := (Table_O - Real'(Table_N * Placed.Translation)) / Unit_Of (M, A);
   end True_Table;

   --  Boxes on the table, more than half of each arm's view: each arm's
   --  table is still the plane its table points lie on, none of the boxes'
   --  points with it, and the plane in the arm's frame (its normal, and its
   --  offset and tilt together) within Z of its own uncertainty of the truth.
   procedure Table_Among_Boxes is
      M : Model;
   begin
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose, others => <>));
      for A in 1 .. 2 loop
         declare
            R      : Arm_Fit renames M.Kinematics (A).Result;
            P      : constant Driver.Geometry.Plane_Estimate := Table_In_Arm (M, Arm_Id (A));
            Up_A   : constant Direction_Estimate := Up_In_Arm (M, Arm_Id (A));
            Placed : constant Rigid := (if A = 1 then Driver.Numerics.Identity else Far_Second);
            True_N : Vec3;
            True_O : Real;
            Off    : Real;
            Boxed, Table, On : Natural := 0;
         begin
            True_Table (M, A, Placed, True_N, True_O);
            Off := Arccos (Real'Max (-1.0, Real'Min (1.0, P.Normal * True_N)));
            Check (Driver.Geometry.Known (P), "arm" & A'Image & "'s table is not found");
            Check (Up_A.Unit_Vector = P.Normal, "arm" & A'Image & "'s up is not its table's normal");
            for I in 0 .. Natural (M.Kinematics (A).Query_U.Length) - 1 loop
               declare
                  X : Vec3;
                  On_Table : Boolean;
               begin
                  Scene_Point (Placed, M.Kinematics (A).Query_U (I), M.Kinematics (A).Query_V (I), True, X, On_Table);
                  if I < Natural (R.Table_On.Length) and then R.Table_On (I) then
                     On := On + 1;
                     if not On_Table then
                        Boxed := Boxed + 1;
                     end if;
                  end if;
                  if On_Table then
                     Table := Table + 1;
                  end if;
               end;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "table test: arm" & A'Image & ":" & On'Image & " points on its table of"
                             & Table'Image & " truly there," & Boxed'Image & " on boxes; normal off" & Off'Image
                             & " rad, its sigma" & Up_A.Sigma'Image & "; the plane off by"
                             & Real'Image (Plane_Off (P, True_N, True_O)) & " of its own uncertainty");
            Check (Boxed = 0, "arm" & A'Image & "'s table holds" & Boxed'Image & " points of the boxes");
            Check (2 * On > Table, "arm" & A'Image & "'s table holds" & On'Image & " of its" & Table'Image & " table points");
            Check (Off <= Driver.Conventions.Z * Up_A.Sigma,
                   "arm" & A'Image & "'s table normal is off by" & Off'Image & " rad, its sigma" & Up_A.Sigma'Image);
            Check (Plane_Off (P, True_N, True_O) <= Threshold (Vector_Gate (3)),
                   "arm" & A'Image & "'s table is off by" & Real'Image (Plane_Off (P, True_N, True_O))
                   & " of its own uncertainty in its offset and tilt");
         end;
      end loop;
   end Table_Among_Boxes;

   function Trace (S : Mat3) return Real is (S (1, 1) + S (2, 2) + S (3, 3));

   --  ── An arm's own frame, free of its placement ──
   --
   --  The head rig: the second arm placed through the head, four and a half
   --  units from the first and turned. In its own frame its tool at a pose no
   --  keyframe had, its table and its up come out within Z of their own
   --  uncertainty of the truth, in the arm's unit. Then the second arm is
   --  misplaced in the world far beyond its placement's own sigma (the head
   --  places its eye to half a unit): turned by a hundredth of a radian, its
   --  centre moved by ten units, its scale five per cent too large.
   --  Its tool in the world moves by all of that; in its own frame nothing
   --  moves, neither its tool nor its table nor its up nor a reach planned
   --  there, and that reach still plans with the arm not placed at all,
   --  where one given in the world cannot.
   procedure Arm_Frame_Free_Of_Placement is
      use type Driver.Robot.Motion.Plan_Status;
      use type Driver.Geometry.Plane_Estimate;
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Motion renames Driver.Robot.Motion;
      M : Model;
      Q : constant Real_Array (1 .. 6) := [0.15, -0.1, 0.08, -0.12, 0.1, -0.15];
      O : Observation;
   begin
      Head_Scene (M);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      O.Readings.Append (Q);
      declare
         R2     : Arm_Fit renames M.Kinematics (2).Result;
         Tool   : constant Pose_Estimate := Tool_In_Arm (M, 2, O);
         Table  : constant Driver.Geometry.Plane_Estimate := Table_In_Arm (M, 2);
         Up_2   : constant Direction_Estimate := Up_In_Arm (M, 2);
         World  : constant Pose_Estimate := Tool_Pose (M, 2, O);
         Truth  : constant Rigid := Fit.Eye_At (Truth_Of (2), Along (Truth_Of (2)), Q);
         Off    : constant Real := abs (Tool.Pose.Translation - (1.0 / Unit_Of (M, 2)) * Truth.Translation);
         Turned : constant Real := Driver.Numerics.Angle (Transpose (Tool.Pose.Rotation) * Truth.Rotation);
         Goal   : constant Motion.Pose_Goal :=
           (Pose          => (Rotation    => Tool.Pose.Rotation * Driver.Numerics.Exp ([0.0, 0.02, 0.0]),
                              Translation => Tool.Pose.Translation + [0.02, -0.01, 0.0]),
            Position_Only => False,
            Free_About    => <>);
         Plan   : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 2, O, Goal);
         True_N : Vec3;
         True_O : Real;
      begin
         Check (R2.Placed and then Known (Arm_Unit (M, 2)), "the second arm is not placed through the head");
         Check (Off <= Driver.Conventions.Z * Sqrt (Trace (Tool.Position_Covariance)),
                "the second arm's tool in its own frame is off by" & Off'Image & " units, its sigma"
                & Real'Image (Sqrt (Trace (Tool.Position_Covariance))));
         Check (Turned <= Driver.Conventions.Z * Sqrt (Trace (Tool.Rotation_Covariance)),
                "the second arm's tool in its own frame is turned by" & Turned'Image & " rad, its sigma"
                & Real'Image (Sqrt (Trace (Tool.Rotation_Covariance))));
         True_Table (M, 2, Far_Second, True_N, True_O);
         Check (Plane_Off (Table, True_N, True_O) <= Threshold (Vector_Gate (3)),
                "the second arm's table in its own frame is off by" & Real'Image (Plane_Off (Table, True_N, True_O))
                & " of its own uncertainty");
         Check (Up_2.Unit_Vector = Table.Normal and then Up_2.Sigma < Real'Last,
                "the second arm's up is not its table's normal");
         Check (Motion.Status (Plan) = Motion.Planned,
                "a reach in the second arm's own frame is not planned: " & Motion.Why (Plan));
         Driver.Log.Line (Driver.Log.Robot, "arm frame test: tool off" & Off'Image & " sigma"
                          & Real'Image (Sqrt (Trace (Tool.Position_Covariance))) & ", turned" & Turned'Image
                          & " sigma" & Real'Image (Sqrt (Trace (Tool.Rotation_Covariance))) & "; table off"
                          & Real'Image (Plane_Off (Table, True_N, True_O)) & " of its uncertainty");
         --  Misplaced in the world.
         R2.Placement := (Rotation    => Driver.Numerics.Exp ([0.0, 0.0, 0.01]) * R2.Placement.Rotation,
                          Translation => R2.Placement.Translation + [10.0, 0.0, 0.0]);
         R2.Scale := 1.05 * R2.Scale;
         declare
            Moved_By : constant Real := abs (Tool_Pose (M, 2, O).Pose.Translation - World.Pose.Translation);
         begin
            Check (Moved_By > Driver.Conventions.Z * Sqrt (Trace (World.Position_Covariance)),
                   "misplacing the second arm moves its tool in the world by only" & Moved_By'Image
                   & " units, its sigma" & Real'Image (Sqrt (Trace (World.Position_Covariance))));
         end;
         Check (Tool_In_Arm (M, 2, O) = Tool, "misplacing the second arm in the world moves its tool in its own frame");
         Check (Table_In_Arm (M, 2) = Table, "misplacing the second arm in the world moves its table in its own frame");
         Check (Up_In_Arm (M, 2) = Up_2, "misplacing the second arm in the world turns its up in its own frame");
         if Motion.Status (Plan) = Motion.Planned then
            declare
               Again : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 2, O, Goal);
            begin
               Check (Motion.Status (Again) = Motion.Planned
                      and then Motion.Last_Readings (Again) = Motion.Last_Readings (Plan),
                      "misplacing the second arm in the world changes a reach planned in its own frame");
            end;
            --  Not placed at all.
            R2.Placed := False;
            declare
               Unplaced : constant Motion.Plan := Motion.Plan_Reach_In_Arm (M, 2, O, Goal);
            begin
               Check (Motion.Status (Unplaced) = Motion.Planned
                      and then Motion.Last_Readings (Unplaced) = Motion.Last_Readings (Plan),
                      "a reach in the arm's own frame does not plan, or plans elsewhere, with the arm not placed");
               Check (Tool_In_Arm (M, 2, O) = Tool, "the tool in its own frame changes with the arm not placed");
               Check (Motion.Status (Motion.Plan_Reach (M, 2, O, (Pose => World.Pose, Position_Only => False, Free_About => <>)))
                      = Motion.Unmeasured, "a reach given in the world plans for an arm not placed in it");
            end;
         end if;
      end;
   end Arm_Frame_Free_Of_Placement;

   --  A reach given in the world for the head rig's second arm, placed four
   --  and a half units from the first and turned: the readings it plans put
   --  the arm's tool where the goal is in the world, by the arm's model and
   --  placement, within what the planner leaves (Z times the angle a pixel
   --  subtends, in the arm's unit). A goal taken as if the arm's own frame
   --  were the world lands the placement away, or nowhere.
   procedure Reach_Through_The_Placement is
      use type Driver.Robot.Motion.Plan_Status;
      package Motion renames Driver.Robot.Motion;
      M       : Model;
      Q_Goal  : constant Real_Array (1 .. 6) := [0.1, -0.05, 0.08, 0.05, 0.1, -0.1];
      O, Want : Observation;
   begin
      Head_Scene (M);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      Want.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      Want.Readings.Append (Q_Goal);
      declare
         Goal : constant Rigid := Tool_Pose (M, 2, Want).Pose;
         P    : constant Motion.Plan := Motion.Plan_Reach (M, 2, O, (Pose => Goal, Position_Only => False, Free_About => <>));
      begin
         Check (Motion.Status (P) = Motion.Planned, "a reach given in the world for the placed arm is not planned: "
                & Motion.Why (P));
         if Motion.Status (P) = Motion.Planned then
            declare
               Reached : Observation;
            begin
               Reached.Readings.Append (Real_Array'(1 .. 6 => 0.0));
               Reached.Readings.Append (Motion.Last_Readings (P));
               declare
                  Got   : constant Rigid := Tool_Pose (M, 2, Reached).Pose;
                  Sigma : constant Real := Driver.Robot.Kinematics.Angle_Sigma (M, 2);
                  Off   : constant Real := abs (Got.Translation - Goal.Translation);
                  Turn  : constant Real := Driver.Numerics.Angle (Transpose (Got.Rotation) * Goal.Rotation);
               begin
                  Check (Off <= Driver.Conventions.Z * Arm_Unit (M, 2).Value * Sigma
                         and then Turn <= Driver.Conventions.Z * Sigma,
                         "the plan puts the placed arm's tool" & Off'Image & " world units and" & Turn'Image
                         & " rad from its goal in the world");
               end;
            end;
         end if;
      end;
   end Reach_Through_The_Placement;

   procedure Kinematics_With_An_Offset_Reference is
      M : Model;
   begin
      Build_Fitted_Arm (M, 3.0e-5);
      Check (Driver.Robot.Kinematics.Fitted (M, 1),
             "the arm whose reference lay 3e-5 rad off its sweep's base is not fitted: "
             & Ada.Strings.Unbounded.To_String (M.Kinematics (1).Result.Why));
      if Driver.Robot.Kinematics.Fitted (M, 1) then
         Check (abs (M.Kinematics (1).Result.Lens.Fx - 400.0) < 1.0,
                "its focal length came out" & M.Kinematics (1).Result.Lens.Fx'Image);
      end if;
   end Kinematics_With_An_Offset_Reference;

   --  A body as a boot leaves it: the fitted arm, and the rest of what the
   --  boot measures, set to values it could have measured.
   procedure Measured_Body (M : in out Model) is
   begin
      Build_Fitted_Arm (M, 0.0);
      for C in 1 .. 6 loop
         M.Noise.Append (1.0e-6 * Real (C) / 3.0);
         M.Noise_Freedom.Append (100 + C);
         M.Groups (1).Low_Seen.Append (-0.2 - 0.01 * Real (C));
         M.Groups (1).High_Seen.Append (0.2 + 0.01 * Real (C));
      end loop;
      M.Groups (1).Delay_Beats := 2;
      M.Groups (1).Delay_Known := True;
      M.Groups (1).Free_Shortfalls.Append (0.0125);
      M.Groups (1).Free_Shortfalls.Append (1.0 / 300.0);
      --  The free pushes a boot leaves are many: enough to tell what falls
      --  short by more than they do (the last ones delivered all they were asked).
      for K in 1 .. 4 loop
         M.Groups (1).Free_Shortfalls.Append (0.0);
      end loop;
      --  And channel by channel, as the step tracker keeps them: each channel asked by all six, the most 0.0125.
      for C in 1 .. 6 loop
         M.Groups (1).Free_Short_Most.Append (0.0125);
         M.Groups (1).Free_Short_Count.Append (6);
      end loop;
      M.Lags.Append (1);
      M.Lag_Known.Append (True);
      M.Graph.Roles.Append (Arm);
      M.Graph.Arm_Of.Append (1);
      M.Graph.Breach.Append (0);
      M.Eyes (1).Rest_Factor := 1.0 + 1.0 / 7.0;
      M.Eyes (1).Rest_Counts_Known := True;
      M.Eyes (1).Rest_Count_Max := 3;
      M.Eyes (1).Rest_Count_Beats := 41;
      for Cell in 1 .. 4 loop
         M.Eyes (1).Textured.Append (Cell /= 2);
      end loop;
   end Measured_Body;

   function Replaced (S, From, To : String) return String is
      At_From : constant Natural := Ada.Strings.Fixed.Index (S, From);
   begin
      return (if At_From = 0 then S else S (S'First .. At_From - 1) & To & S (At_From + From'Length .. S'Last));
   end Replaced;

   --  Written and read back, the body is the one that was written: every
   --  number to the bit, and the file the reloaded body writes is the same.
   procedure Body_File_Round_Trip is
      M, Back : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Measured_Body (M);
      --  An aligner's error as keyframes measure it, which the synthetic keyframes of this body do not show.
      M.Kinematics (M.Kinematics.First_Index).Scatter := 0.3;
      Check (Driver.Robot.Kinematics.Fitted (M, 1), "the body to write has no fitted arm");
      declare
         Written : constant String := Driver.Robot.Body_File.Text (M);
      begin
         Driver.Robot.Body_File.Read (Back, Written, Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         for Q in Stored loop
            Check (Reloaded (Back, Q), Q'Image & " was not reloaded");
         end loop;
         Check (Driver.Robot.Body_File.Text (Back) = Written, "the reloaded body writes another file");
         Check (Driver.Robot.Kinematics.Fitted (Back, 1), "the reloaded arm is not fitted");
         Check (Driver.Robot.Kinematics.Match_Noise (M, 1) > 0.0
                and then Driver.Robot.Kinematics.Match_Noise (Back, 1) = Driver.Robot.Kinematics.Match_Noise (M, 1),
                "the aligner's error on the arm's eye, " & Driver.Robot.Kinematics.Match_Noise (M, 1)'Image
                & " px, was reloaded as" & Driver.Robot.Kinematics.Match_Noise (Back, 1)'Image);
         declare
            Q    : constant Real_Array (1 .. 6) := [0.1, -0.05, 0.08, 0.05, 0.1, -0.1];
            Was  : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q);
            Is_Now : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (Back, 1, Q);
            T1, P1, T2, P2 : Mat3;
         begin
            Driver.Robot.Kinematics.Pose_Covariance (M, 1, Q, T1, P1);
            Driver.Robot.Kinematics.Pose_Covariance (Back, 1, Q, T2, P2);
            Check (Was.Rotation = Is_Now.Rotation and then Was.Translation = Is_Now.Translation
                   and then T1 = T2 and then P1 = P2,
                   "the reloaded arm puts its eye elsewhere, or less certainly");
            for C in 1 .. 6 loop
               Check (Visible_Step (Back, 1, C) = Visible_Step (M, 1, C)
                      and then Driver.Robot.Kinematics.Keyframe_Step (Back, 1, C)
                               = Driver.Robot.Kinematics.Keyframe_Step (M, 1, C)
                      and then Reading_Noise (Back, 1, C) = Reading_Noise (M, 1, C),
                      "channel" & C'Image & "'s steps or noise changed in the file");
            end loop;
         end;
      end;
   end Body_File_Round_Trip;

   --  A body file the driver reads goes into the recording where it was
   --  read, once, and the recorded text reloads the body the file gave.
   procedure Body_File_In_The_Recording is
      use type Driver.Recording.Record_Kind;
      use type GNAT.OS_Lib.File_Descriptor;
      use type GNAT.OS_Lib.String_Access;
      M, Live, Again : Model;
      Ok    : Boolean;
      Why   : Ada.Strings.Unbounded.Unbounded_String;
      FD    : GNAT.OS_Lib.File_Descriptor;
      File_Name, Recording_Name : GNAT.OS_Lib.String_Access;
      Found : Natural := 0;
      Gone  : Boolean;
   begin
      Measured_Body (M);
      GNAT.OS_Lib.Create_Temp_File (FD, File_Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the body file");
      GNAT.OS_Lib.Close (FD);
      GNAT.OS_Lib.Create_Temp_File (FD, Recording_Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the recording");
      GNAT.OS_Lib.Close (FD);
      if File_Name = null or else Recording_Name = null then
         return;
      end if;
      declare
         F : Ada.Text_IO.File_Type;
      begin
         Ada.Text_IO.Open (F, Ada.Text_IO.Out_File, File_Name.all);
         Ada.Text_IO.Put (F, Driver.Robot.Body_File.Text (M));
         Ada.Text_IO.Close (F);
      end;
      Driver.Recording.Start_Shared (Recording_Name.all);
      Load_Body (Live, File_Name.all, Ok, Why);
      Driver.Recording.Stop_Shared;
      Check (Ok, "the body file was not loaded: " & Ada.Strings.Unbounded.To_String (Why));
      declare
         R       : Driver.Recording.Reader;
         Opened  : Boolean;
         More    : Boolean := True;
         Kind    : Driver.Recording.Record_Kind;
         Ns      : Long_Long_Integer;
         Payload : Driver.Bytes.Buffer;
         Head    : constant String := "body " & File_Name.all & ASCII.LF;
         procedure Reload_Recorded (Data : Driver.Bytes.Byte_Array) is
            Text : constant String := Driver.Bytes.To_String (Data);
         begin
            Found := Found + 1;
            if Text'Length < Head'Length or else Text (Text'First .. Text'First + Head'Length - 1) /= Head then
               Check (False, "the file record does not name the body file it holds");
               return;
            end if;
            Load_Body_Text (Again, Text (Text'First + Head'Length .. Text'Last), Ok, Why);
            Check (Ok, "the recorded text does not reload: " & Ada.Strings.Unbounded.To_String (Why));
            Check (Driver.Robot.Body_File.Text (Again) = Driver.Robot.Body_File.Text (Live),
                   "the recorded text reloads another body than the file did");
         end Reload_Recorded;
      begin
         Driver.Recording.Open (R, Recording_Name.all, Opened);
         Check (Opened, "the recording cannot be opened");
         while Opened and then More loop
            Driver.Recording.Next (R, Kind, Ns, Payload, More);
            if More and then Kind = Driver.Recording.File_Read then
               Payload.Query (Reload_Recorded'Access);
            end if;
         end loop;
         if Opened then
            Driver.Recording.Close (R);
         end if;
      end;
      Check (Found = 1, "the body file read is in the recording" & Found'Image & " times, not once");
      GNAT.OS_Lib.Delete_File (File_Name.all, Gone);
      GNAT.OS_Lib.Delete_File (Recording_Name.all, Gone);
      GNAT.OS_Lib.Free (File_Name);
      GNAT.OS_Lib.Free (Recording_Name);
   end Body_File_In_The_Recording;

   --  A body file is one line, and a body of thirty joints writes a megabyte of it: the humanoid's reload died of a
   --  stack overflow in the function form of Get_Line, which reads a line's rest by recursion (10-20). A file of the
   --  body's text and thirty-two megabytes of blanks (8192 pieces of 4096) before its last brace, one line, loads and
   --  is the body it held.
   procedure Body_File_One_Long_Line is
      use type GNAT.OS_Lib.File_Descriptor;
      use type GNAT.OS_Lib.String_Access;
      M, Live : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
      FD      : GNAT.OS_Lib.File_Descriptor;
      File_Name : GNAT.OS_Lib.String_Access;
      Gone    : Boolean;
   begin
      Measured_Body (M);
      GNAT.OS_Lib.Create_Temp_File (FD, File_Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the body file");
      GNAT.OS_Lib.Close (FD);
      if File_Name = null then
         return;
      end if;
      declare
         Written : constant String := Driver.Robot.Body_File.Text (M);
         Brace   : constant Natural := Ada.Strings.Fixed.Index (Written, "}", Ada.Strings.Backward);
         Piece   : constant String (1 .. 4096) := [others => ' '];
         F       : Ada.Text_IO.File_Type;
      begin
         Check (Brace > 0, "the body's text has no closing brace");
         Ada.Text_IO.Open (F, Ada.Text_IO.Out_File, File_Name.all);
         Ada.Text_IO.Put (F, Written (Written'First .. Brace - 1));
         for Pieces in 1 .. 8192 loop
            Ada.Text_IO.Put (F, Piece);
         end loop;
         Ada.Text_IO.Put (F, Written (Brace .. Written'Last));
         Ada.Text_IO.Close (F);
         Load_Body (Live, File_Name.all, Ok, Why);
         Check (Ok, "the long body file was not loaded: " & Ada.Strings.Unbounded.To_String (Why));
         Check (Driver.Robot.Body_File.Text (Live) = Written, "the long body file loaded another body than it held");
      end;
      GNAT.OS_Lib.Delete_File (File_Name.all, Gone);
      GNAT.OS_Lib.Free (File_Name);
   end Body_File_One_Long_Line;

   --  A decider's Estimate_Now goes into the recording once per call; the
   --  recomputations Observe makes as the evidence doubles do not, since a
   --  replay makes those itself.
   procedure Estimate_In_The_Recording is
      use type Driver.Recording.Record_Kind;
      use type GNAT.OS_Lib.File_Descriptor;
      use type GNAT.OS_Lib.String_Access;
      M     : Model;
      O     : Observation;
      FD    : GNAT.OS_Lib.File_Descriptor;
      Name  : GNAT.OS_Lib.String_Access;
      Found : Natural := 0;
      Gone  : Boolean;
   begin
      GNAT.OS_Lib.Create_Temp_File (FD, Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the recording");
      GNAT.OS_Lib.Close (FD);
      if Name = null then
         return;
      end if;
      Driver.Recording.Start_Shared (Name.all);
      --  Eight beats: Observe recomputes at beats 1, 2, 4 and 8 on its own.
      for B in 0 .. 7 loop
         O := (others => <>);
         O.Beat := Driver.Clock.Beat (B);
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Readings.Append (Real_Array'(1 => 0.0));
         Observe (M, O, Driver.Commands.Hold);
      end loop;
      Estimate_Now (M);
      Estimate_Now (M);
      Driver.Recording.Stop_Shared;
      declare
         R       : Driver.Recording.Reader;
         Opened  : Boolean;
         More    : Boolean := True;
         Kind    : Driver.Recording.Record_Kind;
         Ns      : Long_Long_Integer;
         Payload : Driver.Bytes.Buffer;
      begin
         Driver.Recording.Open (R, Name.all, Opened);
         Check (Opened, "the recording cannot be opened");
         while Opened and then More loop
            Driver.Recording.Next (R, Kind, Ns, Payload, More);
            if More and then Kind = Driver.Recording.Estimates_Asked then
               Found := Found + 1;
            end if;
         end loop;
         if Opened then
            Driver.Recording.Close (R);
         end if;
      end;
      Check (Found = 2, "two calls of Estimate_Now after eight observed beats are in the recording"
             & Found'Image & " times, not twice");
      GNAT.OS_Lib.Delete_File (Name.all, Gone);
      GNAT.OS_Lib.Free (Name);
   end Estimate_In_The_Recording;

   --  A quantity measured by another method than the code's is measured
   --  again, with what rests on it; the rest stands, and the estimates leave
   --  it as reloaded.
   procedure Body_File_Method_Change is
      M       : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Measured_Body (M);
      declare
         Written : constant String := Driver.Robot.Body_File.Text (M);
         Kin     : constant String := """kinematics"": {""method"": "
           & Driver.Log.Image (Driver.Robot.Body_File.Kinematics_Method);
         Gra     : constant String := """graph"": {""method"": " & Driver.Log.Image (Driver.Robot.Body_File.Graph_Method);
      begin
         declare
            Back : Model;
         begin
            Driver.Robot.Body_File.Read (Back, Replaced (Written, Kin, """kinematics"": {""method"": 0"), Ok, Why);
            Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
            Check (not Reloaded (Back, Stored_Kinematics) and then Back.Kinematics.Is_Empty,
                   "kinematics of another method were reloaded");
            for Q in Stored_Noise .. Stored_Graph loop
               Check (Reloaded (Back, Q), Q'Image & " was not reloaded with only the kinematics' method changed");
            end loop;
            declare
               Noise_Was : constant Real_Vectors.Vector := Back.Noise;
            begin
               Estimate_Now (Back);
               Check (Real_Vectors."=" (Back.Noise, Noise_Was), "a reloaded noise was measured again");
               Check (Role (Back, 1) = Arm and then Eye_Mount (Back, 1).Kind = Arm_Carried,
                      "a reloaded graph was derived again");
            end;
         end;
         declare
            Back : Model;
         begin
            Driver.Robot.Body_File.Read (Back, Replaced (Written, Gra, """graph"": {""method"": 0"), Ok, Why);
            Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
            Check (not Reloaded (Back, Stored_Graph) and then not Reloaded (Back, Stored_Kinematics),
                   "a graph of another method, or the kinematics on it, were reloaded");
            for Q in Stored_Noise .. Stored_Responses loop
               Check (Reloaded (Back, Q), Q'Image & " was not reloaded with only the graph's method changed");
            end loop;
         end;
      end;
   end Body_File_Method_Change;

   --  What is reloaded stands and is not measured again, so the kinematics are
   --  reloaded only when every arm that carries an eye is fitted in them. A
   --  file written while the arms were being swept holds the fits there were
   --  and an arm without one; reloaded whole, that arm would be left unswept
   --  and unfitted for the session. The rest of that file stands. Written to
   --  a file, the body replaces the file whole.
   procedure Body_File_Unfinished_Sweeps is
      M, Back, Whole_Back : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
      FD      : GNAT.OS_Lib.File_Descriptor;
      Name    : GNAT.OS_Lib.String_Access;
      Gone    : Boolean;
      use type GNAT.OS_Lib.File_Descriptor;
   begin
      Measured_Body (M);
      declare
         Written  : constant String := Driver.Robot.Body_File.Text (M);
         Unfitted : constant String := Replaced (Written, """fitted"": true", """fitted"": false");
      begin
         Check (Unfitted /= Written, "the file does not say that the arm is fitted");
         Driver.Robot.Body_File.Read (Back, Unfitted, Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         Check (not Reloaded (Back, Stored_Kinematics) and then not Driver.Robot.Kinematics.Fitted (Back, 1),
                "the kinematics of an arm that carries an eye and has no fit in the file were reloaded");
         for Q in Stored_Noise .. Stored_Graph loop
            Check (Reloaded (Back, Q), Q'Image & " was not reloaded beside an arm without a fit");
         end loop;
         Driver.Robot.Body_File.Read (Whole_Back, Written, Ok, Why);
         Check (Reloaded (Whole_Back, Stored_Kinematics) and then Driver.Robot.Kinematics.Fitted (Whole_Back, 1),
                "the kinematics of a fitted arm were not reloaded");
      end;
      GNAT.OS_Lib.Create_Temp_File (FD, Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD, "no scratch file for the body file");
      if FD /= GNAT.OS_Lib.Invalid_FD then
         GNAT.OS_Lib.Close (FD);
         Driver.Robot.Body_File.Write (M, Name.all, Ok);
         Check (Ok, "the body file was not written");
         Check (not GNAT.OS_Lib.Is_Regular_File (Name.all & ".part"), "a write left its half behind");
         GNAT.OS_Lib.Delete_File (Name.all, Gone);
      end if;
   end Body_File_Unfinished_Sweeps;

   --  The arm reaches a pose within its travel on a body reloaded from a
   --  file, with no stream behind it and no instrument.
   procedure Plan_On_A_Reloaded_Body is
      M, Back : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
      O       : Observation;
      Q_Goal  : constant Real_Array (1 .. 6) := [0.1, -0.05, 0.08, 0.05, 0.1, -0.1];
   begin
      Measured_Body (M);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      declare
         Goal : constant Driver.Robot.Motion.Pose_Goal :=
           (Pose => Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q_Goal), Position_Only => False, Free_About => <>);
         use type Driver.Robot.Motion.Plan_Status;
      begin
         Check (Driver.Robot.Motion.Status (Driver.Robot.Motion.Plan_Reach (Back, 1, O, Goal))
                /= Driver.Robot.Motion.Planned, "a body that measured nothing plans a reach");
         Driver.Robot.Body_File.Read (Back, Driver.Robot.Body_File.Text (M), Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         declare
            P : constant Driver.Robot.Motion.Plan := Driver.Robot.Motion.Plan_Reach (Back, 1, O, Goal);
         begin
            Check (Driver.Robot.Motion.Status (P) = Driver.Robot.Motion.Planned,
                   "the reloaded body cannot reach a pose within its travel: " & Driver.Robot.Motion.Why (P));
         end;
      end;
   end Plan_On_A_Reloaded_Body;

   --  ── A reloaded body creeping below its visible step ──
   --
   --  A12: the body reloaded from its file, its readings' noise measured
   --  while they were held to a few parts in 1e17 (2.4e-16), and the arm
   --  creeping uncommanded by 1.2e-12 a beat, a ten-thousandth of the step its
   --  eye can see (A12's was a millionth). Its eye shows the same picture every beat. The body is still:
   --  every group by the one motion test (a channel an eye watches moves only
   --  by a step it can see), Still holds, and Settle and Hold_For_Keyframe
   --  end.
   procedure Reloaded_Creep_Is_Still is
      Measured, M : Model;
      Ok   : Boolean;
      Why  : Ada.Strings.Unbounded.Unbounded_String;
      Done_Settle, Done_Hold : Boolean := False with Atomic;
      Finished : Boolean := False with Atomic;
      Still_Seen : Boolean := False;
      Creep : constant Real := 1.2e-12;
      Width  : constant := 640;
      Height : constant := 480;

      function Picture return Driver.Images.Image is
         use type Driver.Bytes.Offset;
         Data : Driver.Bytes.Byte_Array (1 .. 3 * Width * Height);
      begin
         for Y in 0 .. Height - 1 loop
            for X in 0 .. Width - 1 loop
               declare
                  K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Y * Width + X) + 1);
               begin
                  Data (K) := Driver.Bytes.Byte (Integer (Real'Max (0.0, Real'Min (255.0, Texture (Real (X), Real (Y))))));
                  Data (K + 1) := Data (K);
                  Data (K + 2) := Data (K);
               end;
            end loop;
         end loop;
         return Driver.Images.Create (Width, Height, Data);
      end Picture;

      Shown : constant Driver.Images.Image := Picture;
   begin
      Measured_Body (Measured);
      for C in 1 .. 6 loop
         Measured.Noise.Replace_Element (C - 1, 2.4e-16);
      end loop;
      Driver.Robot.Body_File.Read (M, Driver.Robot.Body_File.Text (Measured), Ok, Why);
      Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
      for C in 1 .. 6 loop
         Check (Known (Visible_Step (M, 1, C)) and then Visible_Step (M, 1, C).Value > 1.0e3 * Creep,
                "the reloaded arm's channel" & C'Image & " has no visible step a thousand times its creep");
      end loop;
      declare
         task Decider;
         task body Decider is
            W : Natural;
         begin
            Driver.Robot.Motion.Settle (M, W);
            Done_Settle := True;
            Driver.Robot.Motion.Hold_For_Keyframe (M, 1);
            Done_Hold := True;
            Finished := True;
         exception
            when others =>
               Driver.Beats.Release;
               Finished := True;
         end Decider;

         Now  : Real_Array (1 .. 6) := [others => 0.0];
         Sent : Driver.Commands.Command;
      begin
         begin
            for B in 0 .. 60 loop
               exit when Finished;
               declare
                  Ob      : Observation;
                  Took    : Boolean := False;
                  Pending : Driver.Commands.Command;
               begin
                  Ob.Beat := Driver.Clock.Beat (B);
                  Ob.Images.Append (Shown);
                  Ob.Depth.Append (Real_Array'(1 .. 0 => 0.0));
                  Ob.Readings.Append (Now);
                  Ob.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
                  if B = 0 then
                     Driver.Commands.Set_Target (Sent, 1, Now);
                  end if;
                  Observe (M, Ob, Sent);
                  if B > 2 and then Stillness.All_Still (M) then
                     Still_Seen := True;
                  end if;
                  loop
                     Driver.Beats.Offer (Ob.Beat, Ob, Sent, Took);
                     exit when Took or else Finished;
                     delay 0.0;
                  end loop;
                  exit when not Took;
                  Driver.Beats.Await (Pending);
                  --  Uncommanded: the arm creeps whatever it is asked to hold.
                  for X of Now loop
                     X := X - Creep;
                  end loop;
               end;
            end loop;
         exception
            when others =>
               abort Decider;
               raise;
         end;
         if not Finished then
            abort Decider;
         end if;
      end;
      Check (Still_Seen, "the reloaded body creeping a ten-thousandth of its visible step a beat is never still");
      Check (Done_Settle, "Settle does not end on the reloaded body creeping below its visible step");
      Check (Done_Hold, "Hold_For_Keyframe does not end on the reloaded body creeping below its visible step");
   end Reloaded_Creep_Is_Still;

   --  ── A plan beyond the travel ──
   --
   --  The measured body's arm has shown at most a quarter radian each way;
   --  the goal here needs every joint well past that. The body is the one
   --  its file brings back, as a boot that reloads it runs (the noise, the
   --  lock-in and the kinematics stand). The plan is the fitted model's, and
   --  Follow takes the arm there on a rig whose group reads its targets
   --  exactly, one beat later. On the rig whose first joint ends at End_At,
   --  inside the path, the step that meets the end is Blocked or Short, and
   --  Follow stops there.

   Beyond_Goal : constant Real_Array (1 .. 6) := [0.45, -0.4, 0.5, 0.35, 0.4, -0.45];

   procedure Follow_Beyond_The_Travel
     (End_At   : Real;
      Planned  : out Boolean;
      Report   : out Driver.Robot.Motion.Step_Report;
      Reached  : out Real_Array;
      Finished : out Boolean)
   is
      Measured, M : Model;
      O   : Observation;
      Ok  : Boolean;
      Why : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Measured_Body (Measured);
      Driver.Robot.Body_File.Read (M, Driver.Robot.Body_File.Text (Measured), Ok, Why);
      Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      declare
         use type Driver.Robot.Motion.Plan_Status;
         Goal : constant Driver.Robot.Motion.Pose_Goal :=
           (Pose => Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Beyond_Goal), Position_Only => False, Free_About => <>);
         P    : constant Driver.Robot.Motion.Plan := Driver.Robot.Motion.Plan_Reach (M, 1, O, Goal);
         Done : Boolean := False with Atomic;
         Got  : Driver.Robot.Motion.Step_Report;

         task Decider;
         task body Decider is
         begin
            if Driver.Robot.Motion.Status (P) = Driver.Robot.Motion.Planned then
               Driver.Robot.Motion.Follow (M, P, Got);
            end if;
            Done := True;
         exception
            when others =>
               Driver.Beats.Release;
               Done := True;
         end Decider;

         Now  : Real_Array (1 .. 6) := [others => 0.0];
         Sent : Driver.Commands.Command;
      begin
         Planned := Driver.Robot.Motion.Status (P) = Driver.Robot.Motion.Planned;
         begin
            for B in 0 .. 5_000 loop
               exit when Done;
               declare
                  Ob      : Observation;
                  Took    : Boolean := False;
                  Pending : Driver.Commands.Command;
               begin
                  Ob.Beat := Driver.Clock.Beat (B);
                  Ob.Readings.Append (Now);
                  Ob.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
                  if B = 0 then
                     Driver.Commands.Set_Target (Sent, 1, Now);
                  end if;
                  Observe (M, Ob, Sent);
                  loop
                     Driver.Beats.Offer (Ob.Beat, Ob, Sent, Took);
                     exit when Took or else Done;
                     delay 0.0;
                  end loop;
                  exit when not Took;
                  Driver.Beats.Await (Pending);
                  if Driver.Commands.Has_Target (Pending, 1) then
                     Driver.Commands.Set_Target (Sent, 1, Driver.Commands.Target (Pending, 1));
                  end if;
                  Now := Driver.Commands.Target (Sent, 1);
                  Now (1) := Real'Min (Now (1), End_At);
               end;
            end loop;
         exception
            when others =>
               abort Decider;
               raise;
         end;
         if not Done then
            abort Decider;
         end if;
         Finished := Done;
         Report := Got;
         Reached := Now;
      end;
   end Follow_Beyond_The_Travel;

   procedure Plan_And_Follow_Beyond_The_Travel is
      use type Driver.Robot.Motion.Step_Outcome;
      M        : Model;
      Planned  : Boolean;
      Finished : Boolean;
      R        : Driver.Robot.Motion.Step_Report;
      Reached  : Real_Array (1 .. 6);
   begin
      Measured_Body (M);
      Follow_Beyond_The_Travel (Real'Last, Planned, R, Reached, Finished);
      Check (Planned, "a goal past the readings the arm has shown is not planned");
      Check (Finished, "following the plan past the travel did not finish");
      if Planned and then Finished then
         Check (R.Outcome = Driver.Robot.Motion.Reached,
                "the plan past the travel was not followed to its end: " & Ada.Strings.Unbounded.To_String (R.Detail));
         declare
            Goal  : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Beyond_Goal);
            Got   : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Reached);
            Sigma : constant Real := Driver.Robot.Kinematics.Angle_Sigma (M, 1);
            Off   : constant Real := abs (Got.Translation - Goal.Translation);
            Turn  : constant Real := Driver.Numerics.Angle (Transpose (Got.Rotation) * Goal.Rotation);
         begin
            Check (Off <= Driver.Conventions.Z * Sigma and then Turn <= Driver.Conventions.Z * Sigma,
                   "the arm followed its plan to an eye" & Off'Image & " model units and" & Turn'Image
                   & " rad from the goal");
         end;
         Check ((for some C in 1 .. 6 => Reached (C) > M.Groups (1).High_Seen (C - 1)
                                         or else Reached (C) < M.Groups (1).Low_Seen (C - 1)),
                "the arm reached the goal without leaving the readings it had shown");
      end if;
      --  The first joint ends at 0.33, past its travel (0.21) and short of the
      --  goal's 0.45.
      Follow_Beyond_The_Travel (0.33, Planned, R, Reached, Finished);
      Check (Planned and then Finished, "the plan past the travel was not made or not followed on the rig with an end");
      if Planned and then Finished then
         Check (R.Outcome /= Driver.Robot.Motion.Reached,
                "a joint whose end lies inside the path is not met as Blocked or Short: "
                & Ada.Strings.Unbounded.To_String (R.Detail));
      end if;
   end Plan_And_Follow_Beyond_The_Travel;

   --  The eye's place in the arm's own unit (Fit.Place_Covariance_In_Unit): what the fit leaves uncertain of every
   --  length of the arm together moves every place with the unit, and is none of a place's uncertainty in that unit;
   --  one length alone is. The pose's covariance (Fit.Pose_Covariance) holds both (a18 on 10-16: 12 per cent of
   --  the eye's place, 6.5 units from the reference, where the truth put the tool within a seventieth of a unit).
   procedure Place_In_Own_Unit is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N      : constant := 3;
      Joints : constant Fit.Joint_Array (1 .. N) :=
        [(W => [0.0, 0.0, 1.0], P => [0.3, -0.2, 0.0], C => 1.0, Slide => False),
         (W => [1.0, 0.0, 0.0], P => [0.0, 0.5, -1.0], C => 1.0, Slide => False),
         (W => Driver.Numerics.Unit ([0.0, 1.0, 0.2]), P => [0.4, 0.1, -0.6], C => 1.0, Slide => False)];
      Chain  : constant Fit.Chain_Order := [1, 2, 3];
      --  The unit's keyframes, each joint turned alone and all together, near the reference; the place asked, far.
      Unit_Changes : constant Real_Matrix (1 .. 4, 1 .. N) :=
        [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1], [0.05, -0.05, 0.05]];
      Far     : constant Real_Array (1 .. N) := [0.4, -0.3, 0.5];
      Reduced : constant Natural := Fit.Terms (N);
      Sigma   : constant Real := 0.01;
      G       : Real_Array (1 .. Reduced) := [others => 0.0];
      Every_Length, One_Length : Fit.Real_Lists.Vector;

      function Trace (X : Driver.Numerics.Mat3) return Real is (X (1, 1) + X (2, 2) + X (3, 3));
   begin
      --  Every length scaled together: each joint's point moved across its axis by its own share (Fit.Across).
      for J in 1 .. N loop
         declare
            E1, E2 : Driver.Numerics.Vec3;
         begin
            Fit.Across (Joints (J).W, E1, E2);
            G (Fit.Term_Of (J, Fit.Point_1)) := Driver.Numerics.Dot (Joints (J).P, E1);
            G (Fit.Term_Of (J, Fit.Point_2)) := Driver.Numerics.Dot (Joints (J).P, E2);
         end;
      end loop;
      for P in 1 .. Reduced loop
         for Q in 1 .. Reduced loop
            Every_Length.Append (Sigma ** 2 * G (P) * G (Q));
            One_Length.Append (if P = Q and then P = Fit.Term_Of (2, Fit.Point_1) then Sigma ** 2 else 0.0);
         end loop;
      end loop;
      declare
         Turn, Every_Pose, One_Pose : Driver.Numerics.Mat3;
         Every_Unit : constant Driver.Numerics.Mat3 :=
           Fit.Place_Covariance_In_Unit (Joints, Chain, Far, Unit_Changes, Every_Length);
         One_Unit   : constant Driver.Numerics.Mat3 :=
           Fit.Place_Covariance_In_Unit (Joints, Chain, Far, Unit_Changes, One_Length);
      begin
         Fit.Pose_Covariance (Joints, Chain, Far, Every_Length, Turn, Every_Pose);
         Fit.Pose_Covariance (Joints, Chain, Far, One_Length, Turn, One_Pose);
         Check (Trace (Every_Pose) > 0.0, "every length scaled together does not move the eye's place: the test shows nothing");
         Check (Trace (Every_Unit) <= Driver.Conventions.Unchanged_Fraction ** 2 * Trace (Every_Pose),
                "every length scaled together leaves the eye's place in the arm's own unit uncertain by"
                & Trace (Every_Unit)'Image & " of the" & Trace (Every_Pose)'Image & " it leaves the pose");
         Check (Trace (One_Unit) >= Driver.Conventions.Unchanged_Fraction * Trace (One_Pose),
                "one length alone leaves the eye's place in the arm's own unit uncertain by" & Trace (One_Unit)'Image
                & " of the" & Trace (One_Pose)'Image & " it leaves the pose");
      end;
   end Place_In_Own_Unit;

   --  The unit of length is the first fit's. A keyframe taken after the arm
   --  was fitted (every rest of a hand's presses, every stop of an action)
   --  refines every term and moves no length: the eye's position at a pose
   --  stays where it was, within its sigmas. The unit used to be the root mean
   --  square of the eye positions over every keyframe there was, so each new
   --  pose changed it, and with it every length of the arm's frame, which for
   --  the first arm is the world's.
   procedure Unit_Holds_As_Keyframes_Arrive is
      package Fit renames Driver.Robot.Kinematics.Fit;
      M      : Model;
      Truth  : constant Fit.Joint_Array := Truth_Of (1);
      Base   : constant Real_Array (1 .. 6) := [others => 0.0];
      Pose   : constant Real_Array (1 .. 6) := [0.1, -0.05, 0.08, 0.05, 0.1, -0.1];
      Rows   : constant array (1 .. 6) of Natural := [1, 2, 4, 5, 7, 8];
      Rng    : Generator := (State => 99);
      First_Frames, First_Used : Natural;
      Was, Now : Vec3;
      Turn, Place_1, Place_2 : Mat3;
   begin
      Build_Fitted_Arm (M, 0.0);
      Check (Driver.Robot.Kinematics.Fitted (M, 1), "the arm to refit is not fitted");
      if not Driver.Robot.Kinematics.Fitted (M, 1) then
         return;
      end if;
      First_Frames := Natural (M.Kinematics (1).Frames.Length);
      First_Used := M.Kinematics (1).Result.Used;
      Was := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Pose).Translation;
      Driver.Robot.Kinematics.Pose_Covariance (M, 1, Pose, Turn, Place_1);
      --  Poses a long way off the sweep's: every joint at 0.4 rad one way, the
      --  other or not at all, so that the eye's positions over all the
      --  keyframes have another root mean square.
      for Row of Rows loop
         declare
            Q : Real_Array (1 .. 6);
         begin
            for J in Q'Range loop
               Q (J) := 0.4 * Real ((Row * J) mod 3 - 1);
            end loop;
            Add_Keyframe (M.Kinematics (1), Q, Base, Truth, Rng, Base);
         end;
      end loop;
      --  Six keyframes on a sweep's many double nothing: a fitted arm is fitted again once its keyframes have doubled,
      --  or when a fit is asked (Kinematics.Refit).
      Driver.Robot.Kinematics.Ask_Fit (M, 1);
      Driver.Robot.Kinematics.Refit (M);
      Check (Driver.Robot.Kinematics.Fitted (M, 1), "the arm is not fitted with the keyframes taken after its fit");
      Check (M.Kinematics (1).Result.Matches
             = Natural (Ada.Containers.Count_Type'(M.Kinematics (1).Matches.Length)) + M.Kinematics (1).Revision
             and then M.Kinematics (1).Result.Used > First_Used,
             "the refit did not use the keyframes taken after the fit:" & M.Kinematics (1).Result.Used'Image
             & " sightings against" & First_Used'Image);
      Now := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Pose).Translation;
      Driver.Robot.Kinematics.Pose_Covariance (M, 1, Pose, Turn, Place_2);
      declare
         Sigma : constant Real := Sqrt (Place_1 (1, 1) + Place_1 (2, 2) + Place_1 (3, 3)
                                        + Place_2 (1, 1) + Place_2 (2, 2) + Place_2 (3, 3));
      begin
         Check (abs (Now - Was) <= Driver.Conventions.Z * Sigma,
                "the eye at a given pose moved by" & Real'Image (abs (Now - Was)) & " model units of" & Real'Image (abs Was)
                & " as keyframes arrived after the fit; its sigmas" & Real'Image (Sigma));
      end;
      --  The unit is the root mean square of the eye's positions over the keyframes of the first fit.
      declare
         Joints : Fit.Joint_Array (1 .. 6);
         Chain  : constant Fit.Chain_Order (1 .. 6) :=
           [for K in 1 .. 6 => Positive (M.Kinematics (1).Result.Chain (K - 1))];
         Sum    : Real := 0.0;
      begin
         for J in Joints'Range loop
            declare
               F : Joint_Fit renames M.Kinematics (1).Result.Joints (J);
            begin
               Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
            end;
         end loop;
         for F in 1 .. First_Frames loop
            declare
               D : Real_Array (1 .. 6);
            begin
               for C in D'Range loop
                  D (C) := M.Kinematics (1).Frames (F).Readings (C - 1) - M.Kinematics (1).Result.Reference (C - 1);
               end loop;
               Sum := Sum + Fit.Eye_At (Joints, Chain, D).Translation * Fit.Eye_At (Joints, Chain, D).Translation;
            end;
         end loop;
         Check_Close (Sqrt (Sum / Real (First_Frames)), 1.0, 1.0e-9,
                      "the root mean square of the eye's positions over the keyframes of the first fit");
      end;
   end Unit_Holds_As_Keyframes_Arrive;

   --  A relay (10-25): an arm swept wide over a table, each joint to 0.6 rad either way, the reference's points mostly
   --  out of its widest pictures; at 0.4 rad either way a keyframe chooses points of its own (a grid of its picture on
   --  the table) and follows them into the keyframe at 0.6. Fitted with the relayed sightings beside the reference's,
   --  begun from the reference's fit as a sweep's later estimate is, the arm is fitted, its focal length within Z of
   --  its sigma of the truth, and the relayed sightings are among those it keeps (measured: 399.96 +- 0.14 px, the
   --  reference's points alone 399.94; 10-25).
   procedure Kinematics_Of_A_Relayed_Sweep is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 5;
      Levels  : constant Real_Array := [0.1, -0.1, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6];
      Cells   : constant := 7;   --  keyframes of every joint at once, each joint's sign by its row's bits (the order)
      Frames  : constant Positive := 1 + N * Levels'Length + Cells;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant Real := 0.3;
      Table   : constant Vec3 := Unit ([0.0, -0.6, -0.8]);   --  the table's normal, towards the eye; n . X = -1
      Lens    : constant Fit.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Truth   : Fit.Joint_Array (1 .. N);
      Changes : Real_Matrix (1 .. Frames, 1 .. N) := [others => [others => 0.0]];
      Asked   : Fit.Flag_Matrix (1 .. Frames, 1 .. N);
      Rng     : Generator;
      type Sighting_Access is access Fit.Sighting_Array;
      All_Seen : constant Sighting_Access := new Fit.Sighting_Array (1 .. 2 * Frames * Columns * Rows);
      Seen     : Natural := 0;
      Own_Seen : Natural := 0;   --  of them, the reference's points' (the first, in order)
      Eyes     : array (1 .. Frames) of Rigid;   --  each keyframe's eye in the reference's frame
      --  The keyframe of joint J at level L.
      function Frame_Of (J, L : Positive) return Positive is (1 + (J - 1) * Levels'Length + L);
      --  Where a point of the reference's frame is seen in keyframe F, if it is.
      procedure Sight (F, Track, Anchor : Positive; U0, V0 : Real; X : Vec3) is
         U, V  : Real;
         Ahead : Boolean;
      begin
         Fit.Project (Lens, Inverse (Eyes (F)) * X, U, V, Ahead);
         U := U + Noise * Gaussian (Rng);
         V := V + Noise * Gaussian (Rng);
         if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 then
            Seen := Seen + 1;
            All_Seen (Seen) := (Frame => F, Track => Track, U0 => U0, V0 => V0, Anchor => Anchor, U => U, V => V,
                                Own => 0.0, Own_Freedom => 0, others => <>);
         end if;
      end Sight;
      --  The point of the table on keyframe F's line of sight through (U0, V0), if the line meets it ahead.
      procedure On_Table (F : Positive; U0, V0 : Real; X : out Vec3; Found : out Boolean) is
         H : constant Vec3 := Eyes (F).Rotation * Fit.Ray (Lens, U0, V0);
         D : constant Real := Real'(Table * H);
      begin
         Found := D < 0.0;
         X := (if Found then Eyes (F).Translation + ((-1.0 - Real'(Table * Eyes (F).Translation)) / D) * H else Zero3);
         Found := Found and then (-1.0 - Real'(Table * Eyes (F).Translation)) / D > 0.0;
      end On_Table;
   begin
      declare
         Axes   : constant array (1 .. N) of Vec3 :=
           [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5]];
         Points : constant array (1 .. N) of Vec3 :=
           [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1]];
      begin
         for J in 1 .. N loop
            declare
               W : constant Vec3 := Unit (Axes (J));
            begin
               Truth (J) := (W => W, P => Points (J) - Real'(Points (J) * W) * W, C => 1.0, Slide => False);
            end;
         end loop;
      end;
      for J in 1 .. N loop
         for L in Levels'Range loop
            Changes (Frame_Of (J, L), J) := Levels (L);
         end loop;
      end loop;
      for Row in 1 .. Cells loop
         for J in 1 .. N loop
            declare
               Bits : Natural := 0;
               R    : Natural := Row;
               C    : Natural := J;
            begin
               while R > 0 and then C > 0 loop
                  if R mod 2 = 1 and then C mod 2 = 1 then
                     Bits := Bits + 1;
                  end if;
                  R := R / 2;
                  C := C / 2;
               end loop;
               Changes (1 + N * Levels'Length + Row, J) := (if Bits mod 2 = 0 then 0.1 else -0.1);
            end;
         end loop;
      end loop;
      for F in 1 .. Frames loop
         declare
            D : Real_Array (1 .. N);
         begin
            for J in 1 .. N loop
               D (J) := Changes (F, J);
               Asked (F, J) := Changes (F, J) /= 0.0;
            end loop;
            Eyes (F) := Fit.Eye_At (Truth, Along (Truth), D);
         end;
      end loop;
      --  The reference's points, in every keyframe that sees them.
      for F in 2 .. Frames loop
         for Gy in 1 .. Rows loop
            for Gx in 1 .. Columns loop
               declare
                  U0 : constant Real := (Real (Gx) - 0.5) * 640.0 / Real (Columns);
                  V0 : constant Real := (Real (Gy) - 0.5) * 480.0 / Real (Rows);
                  X  : Vec3;
                  Found : Boolean;
               begin
                  On_Table (1, U0, V0, X, Found);
                  if Found then
                     Sight (F, (Gy - 1) * Columns + Gx, 1, U0, V0, X);
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      Own_Seen := Seen;
      --  Each joint's keyframes at 0.4 either way choose their own and follow them into those at 0.6.
      for J in 1 .. N loop
         for L in 5 .. 6 loop
            declare
               Anchor : constant Positive := Frame_Of (J, L);
               Next   : constant Positive := Frame_Of (J, L + 2);
            begin
               for Gy in 1 .. Rows loop
                  for Gx in 1 .. Columns loop
                     declare
                        U0 : constant Real := (Real (Gx) - 0.5) * 640.0 / Real (Columns);
                        V0 : constant Real := (Real (Gy) - 0.5) * 480.0 / Real (Rows);
                        X  : Vec3;
                        Found : Boolean;
                     begin
                        On_Table (Anchor, U0, V0, X, Found);
                        if Found then
                           Sight (Next, Columns * Rows * (1 + 2 * (J - 1) + (L - 5)) + (Gy - 1) * Columns + Gx, Anchor,
                                  U0, V0, X);
                        end if;
                     end;
                  end loop;
               end loop;
            end;
         end loop;
      end loop;
      declare
         Joints : Fit.Joint_Array (1 .. N);
         Chain  : Fit.Chain_Order (1 .. N);
         Found_All, Found_Own : Fit.Lens;
         Report_All, Report_Own : Fit.Fit_Report;
         Visible : constant Real_Array (1 .. N) := [others => 0.01];
      begin
         Fit.Fit (Changes, Asked, Visible, All_Seen (1 .. Own_Seen), 640, 480, 0, Joints, Chain, Found_Own, Report_Own);
         --  The relayed sightings come in a later estimate's fit, begun from the one before it, as a sweep's do.
         declare
            Start : Fit.Fit_Start (N);
         begin
            Start.Valid := Report_Own.Fitted;
            Start.Whole := True;
            Start.Joints := Joints;
            Start.Chain := Chain;
            Start.L := Found_Own;
            Start.Focal_Within := Report_Own.Focal_Within;
            Start.Searched_Least := Report_Own.Searched_Least;
            Start.Searched_Most := Report_Own.Searched_Most;
            Start.Profiled_Over := Report_Own.Profiled_Over;
            Start.Profile_Var := Report_Own.Profile_Var;
            Start.Order_Told := Report_Own.Order_Told;
            Fit.Fit (Changes, Asked, Visible, All_Seen (1 .. Seen), 640, 480, 0, Joints, Chain, Found_All, Report_All,
                     From => Start);
         end;
         Driver.Log.Line (Driver.Log.Robot, "kinematics relay test:" & Own_Seen'Image & " sightings of the reference's"
                          & " points, focal " & Driver.Log.Image (Found_Own.Fx, 3) & " +- "
                          & Driver.Log.Image (Report_Own.Focal_Sigma, 3) & " px (fitted " & Report_Own.Fitted'Image
                          & ");" & Natural'Image (Seen - Own_Seen) & " relayed beside them, focal "
                          & Driver.Log.Image (Found_All.Fx, 3) & " +- " & Driver.Log.Image (Report_All.Focal_Sigma, 3)
                          & " px (fitted " & Report_All.Fitted'Image & ")");
         Check (Report_All.Fitted, "the relayed sweep was not fitted: stage" & Report_All.Stage'Image & ", "
                & Ada.Strings.Unbounded.To_String (Report_All.Why));
         if Report_All.Fitted then
            Check (abs (Found_All.Fx - Lens.Fx) <= Driver.Conventions.Z * Report_All.Focal_Sigma,
                   "the relayed sweep's focal length is off by" & Real'Image (Found_All.Fx - Lens.Fx) & " px, its sigma"
                   & Report_All.Focal_Sigma'Image);
            Check (Report_All.Used > Report_Own.Used,
                   "the relayed sightings are not among those the fit kept:" & Report_All.Used'Image & " kept with them,"
                   & Report_Own.Used'Image & " of the reference's alone");
         end if;
      end;
   end Kinematics_Of_A_Relayed_Sweep;

   procedure Kinematics_Of_A_Synthetic_Arm is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True);
   end Kinematics_Of_A_Synthetic_Arm;

   --  A boot refits whenever the evidence has doubled, which can fall between
   --  a keyframe and its matches' return. The widest keyframes of a joint
   --  with no sightings yet are no widest ones: the search scores the widest
   --  that have some (the first boot of A13 died with a range check in the
   --  standard error of an empty median, the arm unfitted).
   procedure Kinematics_With_Matches_Pending is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True, Pending => True);
   end Kinematics_With_Matches_Pending;

   --  The same arm swept a five-hundredth as far: the image moves by less
   --  than its noise, and the fit must say it cannot tell.
   procedure Kinematics_Of_A_Small_Sweep is
   begin
      Synthetic_Sweep (0.002, Expect_Fit => False);
   end Kinematics_Of_A_Small_Sweep;

   --  The kiwi's regime (R6, 10-14): the first passes turned each joint by 1.5 to 3.4 degrees, the aligner's
   --  sightings erred by about a pixel, and a white table left it a quarter of its points. Swept that little, the
   --  fit may or may not tell the arm, but what it reports it must report right: a wrong lens taken for fitted
   --  (the kiwi's 27, 57 and 24453 px) puts every later measurement off. Swept to 11.5 degrees, as the passes'
   --  ladder climbs to, it must tell it.
   --  A plane's points seen before and after a turn about an axis and a shift across it: the turn, and the plane
   --  times the shift, come back from their lines of sight alone, whatever way the axis points against the eye's
   --  (along its sight, across it, and between).
   procedure Turn_On_A_Plane is
      package Fit renames Driver.Robot.Kinematics.Fit;
      type Case_Record is record
         W     : Vec3;
         Turn  : Real;
         Point : Vec3;   --  on the axis
      end record;
      Cases : constant array (1 .. 4) of Case_Record :=
        [(Unit ([0.0, 0.0, 1.0]), 0.3, [0.05, -0.02, 0.0]),
         (Unit ([1.0, 0.0, 0.1]), -0.6, [0.0, 0.2, -0.1]),
         (Unit ([0.3, -0.8, 0.4]), 0.15, [0.1, 0.1, 0.3]),
         (Unit ([0.0, 1.0, 0.0]), 0.6, [0.0, 0.0, -0.4])];
      --  The plane N . X = 1, below and ahead of the eye.
      N : constant Vec3 := [0.0, -0.6, -0.8] / 0.9;   --  its normal over its distance
   begin
      for C of Cases loop
         declare
            R : constant Mat3 := Driver.Numerics.Exp (-C.Turn * C.W);
            T : constant Vec3 := C.Point - R * C.Point;
            Before, After : Fit.Vec3_Array (1 .. 48);
            Weight : constant Real_Array (1 .. 48) := [others => 1.0];
            K      : Natural := 0;
         begin
            for Gy in 1 .. 6 loop
               for Gx in 1 .. 8 loop
                  declare
                     H0 : constant Vec3 := [(Real (Gx) - 4.5) * 0.12, (Real (Gy) - 3.5) * 0.12, 1.0];
                     X  : constant Vec3 := (1.0 / Dot (N, H0)) * H0;
                     Y  : constant Vec3 := R * X + T;
                  begin
                     K := K + 1;
                     Before (K) := H0;
                     After (K) := (1.0 / Y (3)) * Y;
                  end;
               end loop;
            end loop;
            declare
               Turn  : Real;
               Shift, Normal : Vec3;
               Found : Boolean;
               Want  : constant Mat3 := Driver.Numerics.Outer (T, N);
               Got   : Mat3;
               Off   : Real := 0.0;
            begin
               Fit.Turn_On_Plane (Before, After, Weight, C.W, Turn, Shift, Normal, Found);
               Check (Found, "the turn on a plane was not found about" & C.W (1)'Image & C.W (2)'Image & C.W (3)'Image);
               if Found then
                  Got := Driver.Numerics.Outer (Shift, Normal);
                  for P in 1 .. 3 loop
                     for Q in 1 .. 3 loop
                        Off := Real'Max (Off, abs (Got (P, Q) - Want (P, Q)));
                     end loop;
                  end loop;
                  Check (abs (Turn - C.Turn) < 1.0e-9, "the turn came out" & Turn'Image & " for" & C.Turn'Image);
                  Check (Off < 1.0e-9, "the shift times the plane is off by" & Off'Image);
               end if;
            end;
         end;
      end loop;
   end Turn_On_A_Plane;

   --  The desktop arm swept to 34 degrees, its readings a servo's ticks (4096 a turn), its eye on a plane of points
   --  (the table under a wrist): every keyframe's sightings a homography of the reference's, whose epipolar lines hold
   --  under a family of motions, and stage 1 chose 69 px of 400 by them, the fit ending at 777 +- 57 px (10-16); and
   --  the same scene given relief, whose keyframes are no homography where the eye travels.
   procedure Kinematics_On_A_Plane_In_Ticks is
      Tick : constant Real := 2.0 * Ada.Numerics.Pi / 4096.0;
   begin
      Synthetic_Sweep (3.0, Expect_Fit => True, Matcher_Noise => 1.0, Kept_Share => 0.25, Quantum => Tick);
      Synthetic_Sweep (3.0, Expect_Fit => True, Matcher_Noise => 1.0, Kept_Share => 0.25, Quantum => Tick,
                       Relief => 0.3);
   end Kinematics_On_A_Plane_In_Ticks;

   --  The order of the joints is not asked of the sweep to 11.5 degrees: its keyframes that move several joints turn
   --  each by 0.05 units, 2.9 degrees, and what the order changes grows with the product of two joints' turns. Every
   --  order of its six joints scored (10-16), joints 2 and 3 exchanged left residuals of median 0.966 and 0.945 px
   --  against 0.960 and 0.955 px in their true order, not significantly the larger by their signed ranks; the search
   --  that ended at the true order had started there (the axes' distances from the eye) and found no move it could
   --  tell from it.
   procedure Kinematics_In_A_Desktop_Regime is
   begin
      Synthetic_Sweep (0.3, Expect_Fit => True, Matcher_Noise => 1.0, Kept_Share => 0.25, Either => True,
                       Order_Needed => False);
      Synthetic_Sweep (1.0, Expect_Fit => True, Matcher_Noise => 1.0, Kept_Share => 0.25, Order_Needed => False);
      --  The same with a wide-angle wrist camera's barrel distortion.
      Synthetic_Sweep (1.0, Expect_Fit => True, Matcher_Noise => 1.0, Kept_Share => 0.25, Distortion => -0.2,
                       Order_Needed => False);
   end Kinematics_In_A_Desktop_Regime;

   --  The same arm, every keyframe's points sharing an error of 0.3 pixels
   --  (a rendering, a view, the matcher on that pair), as A9's matches did:
   --  sightings taken as independent hide it, and the fit's focal length
   --  and pose come out many of their sigmas off; clustered by keyframe, the
   --  fit's own uncertainty covers its errors.
   procedure Kinematics_With_Shared_Errors is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True, Frame_Error => 0.3);
   end Kinematics_With_Shared_Errors;

   --  The same arm with the errors A11's matcher left: a shift every point of a
   --  keyframe has, an error every point has in all its keyframes (where the
   --  matcher finds it is the same in every view; near points alike), and a
   --  smooth error of each keyframe's own. Sightings counted as independent, or
   --  clustered by keyframe, are far too sure of the lens (A11: chi squares of
   --  94 and 51 on its six terms, Z's tail 21); the covariance read from the
   --  residuals as a function of distance (Errors) covers its errors.
   procedure Kinematics_With_Spreading_Errors is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True, Frame_Error => 0.15, Track_Error => 0.35, Local_Error => 0.20);
   end Kinematics_With_Spreading_Errors;

   --  The fit of an arm on the inputs of a recording of the simulator (tests/data: the readings' changes at each
   --  keyframe, every sighting, the picture, and the number of keyframes the unit rests on). The camera's true focal
   --  length is 397.04 pixels and its principal point the middle of the picture; the fit must come out fitted, within
   --  a percent of both and keeping three quarters of its sightings.
   --  Drawn: the readings as A93's scrambler drew its left arm's (Synthetic_Sweep): the fit must find the same lens, and
   --  the order of the joints through the draw.
   procedure Fit_Of_A_Recording (Path : String; Drawn : Boolean := False) is
      package Fit renames Driver.Robot.Kinematics.Fit;
      package Real_IO is new Ada.Text_IO.Float_IO (Real);
      package Int_IO is new Ada.Text_IO.Integer_IO (Integer);
      F    : Ada.Text_IO.File_Type;
      N, Frames, S, Width, Height, Unit : Integer;
   begin
      begin
         Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      exception
         when Ada.Text_IO.Name_Error =>
            Check (False, "the recorded inputs of an arm's fit are not at " & Path
                          & " (the tests run from the driver's directory)");
            return;
      end;
      Int_IO.Get (F, N);
      Int_IO.Get (F, Frames);
      Int_IO.Get (F, S);
      Int_IO.Get (F, Width);
      Int_IO.Get (F, Height);
      Int_IO.Get (F, Unit);
      declare
         Visible   : Real_Array (1 .. N);
         Changes   : Real_Matrix (1 .. Frames, 1 .. N);
         type Sighting_Access is access Fit.Sighting_Array;
         Seen      : constant Sighting_Access := new Fit.Sighting_Array (1 .. S);
         Joints    : Fit.Joint_Array (1 .. N);
         Chain     : Fit.Chain_Order (1 .. N);
         Found     : Fit.Lens;
         Report    : Fit.Fit_Report;
         Truth     : constant Real := 397.04;
         --  The draw, by reading: joint Order_Of (I) of the chain times Gain (I).
         Order_Of  : constant array (1 .. N) of Positive :=
           (if Drawn and then N = 6 then [6, 5, 2, 1, 4, 3] else [for I in 1 .. N => I]);
         Gain      : constant Real_Array (1 .. N) :=
           (if Drawn and then N = 6 then [-0.5, -1.0, 4.0, -1.0, -0.25, 4.0] else [1 .. N => 1.0]);
         Given     : Real_Matrix (1 .. Frames, 1 .. N);
         Asked     : Fit.Flag_Matrix (1 .. Frames, 1 .. N);   --  the recorded joints that turned: what the sweep asked
         Want_Chain : Fit.Chain_Order (1 .. N);
      begin
         Check (not Drawn or else N = 6, "the recorded arm has" & N'Image & " joints, the draw six");
         for J in 1 .. N loop
            Real_IO.Get (F, Visible (J));
         end loop;
         for Frame in 1 .. Frames loop
            for J in 1 .. N loop
               Real_IO.Get (F, Changes (Frame, J));
            end loop;
         end loop;
         for I in 1 .. S loop
            declare
               Frame, Track : Integer;
            begin
               Int_IO.Get (F, Frame);
               Int_IO.Get (F, Track);
               Seen (I).Frame := Frame;
               Seen (I).Track := Track;
               Real_IO.Get (F, Seen (I).U0);
               Real_IO.Get (F, Seen (I).V0);
               Real_IO.Get (F, Seen (I).U);
               Real_IO.Get (F, Seen (I).V);
            end;
         end loop;
         Ada.Text_IO.Close (F);
         for F in 1 .. Frames loop
            for I in 1 .. N loop
               Given (F, I) := Gain (I) * Changes (F, Order_Of (I));
               Asked (F, I) := Changes (F, Order_Of (I)) /= 0.0;
            end loop;
         end loop;
         for I in 1 .. N loop
            Want_Chain (Order_Of (I)) := I;
         end loop;
         Fit.Fit (Given, Asked, [for I in 1 .. N => abs Gain (I) * Visible (Order_Of (I))], Seen.all, Width, Height, Unit, Joints,
                  Chain, Found, Report);
         Check (Fit."=" (Chain, Want_Chain), "the joints along the recorded arm, by their readings, came out as "
                & Chain_Text (Chain) & ", not " & Chain_Text (Want_Chain));
         Driver.Log.Line (Driver.Log.Robot, "kinematics test (recorded fit of" & Frames'Image & " keyframes): focal"
                          & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy) & ", centre" & Real'Image (Found.Cx)
                          & "," & Real'Image (Found.Cy) & ", kept" & Report.Used'Image & " of" & S'Image
                          & ", noise" & Real'Image (Report.Sigma_Px) & " px");
         Check (Report.Fitted and then Report.Determined,
                "the fit of the recorded arm is not determined: stage" & Report.Stage'Image & ", "
                & Ada.Strings.Unbounded.To_String (Report.Why));
         Check (abs (Found.Fx - Truth) <= 0.01 * Truth and then abs (Found.Fy - Truth) <= 0.01 * Truth,
                "the focal length is" & Real'Image (Found.Fx) & " by" & Real'Image (Found.Fy)
                & " pixels where the camera's is" & Real'Image (Truth));
         Check (abs (Found.Cx - Real (Width) / 2.0) <= 0.01 * Real (Width)
                and then abs (Found.Cy - Real (Height) / 2.0) <= 0.01 * Real (Height),
                "the principal point is" & Real'Image (Found.Cx) & "," & Real'Image (Found.Cy)
                & " where the camera's is the middle of the picture");
         Check (4 * Report.Used >= 3 * S, "the fit keeps only" & Report.Used'Image & " of" & S'Image & " sightings");
      end;
   end Fit_Of_A_Recording;

   --  An arm's refit when its hand has pressed (A17): the 78 keyframes of the sweep and 56 of the presses, the arm
   --  up to 1.4 radians from where it was swept, with 1 to 48 sightings each and some of them wrong. The stages
   --  before the track refinement crossed the flat valley between the focal lengths, the centre and the distortion
   --  in jumps from a damping of the float's epsilon, and the fit ended with a noise of 1.6 pixels, no joint
   --  determined, and its normal equations singular.
   procedure Kinematics_On_A_Recorded_Refit is
   begin
      Fit_Of_A_Recording ("tests/data/refit_with_presses.txt");
   end Kinematics_On_A_Recorded_Refit;

   --  The same recording with its readings as A93's scrambler drew its left arm's: in another order than the chain's,
   --  each its own power of two of either sign. A93's boot fitted that arm with a focal length of 886.65 by 1483.88
   --  pixels (the camera's is 397.04): the fit took the readings for radians in the chain's order.
   procedure Kinematics_On_A_Drawn_Recording is
   begin
      Fit_Of_A_Recording ("tests/data/refit_with_presses.txt", Drawn => True);
   end Kinematics_On_A_Drawn_Recording;

   --  The order exam (10-18): many arms, each swept as Synthetic_Sweep sweeps its own (every joint alone at six
   --  levels, then every joint at once in seven cells), each a draw about that arm: its axes tilted, its joints'
   --  places scaled and moved, its readings in a drawn order and unit; its sightings with the matcher's noise, an
   --  error shared by a keyframe and a smooth field of each keyframe's own. Fitted; while the fit does not tell the
   --  order, the pairs it names (Fit_Report.Untold) are turned together as Boot turns them, in every row of a
   --  Hadamard matrix of the pair's signs at the sweep's widest level, and the arm fitted again with them; until
   --  the order is told or no pair is named that was not turned. Scored against each arm's true order: told and
   --  right, told and wrong (the fit claims what is false), not told (its best order right or wrong).
   --  Named: each pair is turned by the sizes of the turns the fit names for it (Fit_Report.Untold_Turns), as Boot
   --  turns it, in place of the sweep's widest level.
   procedure Order_Exam_At
     (Width, Height : Positive; Focal, Kept_Share, Matcher_Noise, Frame_Error, Tilt : Real; Seed : Long_Long_Integer;
      Widest_Level : Real := 0.2; Named : Boolean := False)
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N        : constant := 6;
      Arms     : constant := 12;
      Columns  : constant := 16;
      Rows     : constant := 12;
      Widest   : constant Real := Widest_Level;
      Levels   : constant Real_Array := [0.25 * Widest, -0.25 * Widest, 0.5 * Widest, -0.5 * Widest, Widest, -Widest];
      Cells    : constant := 7;
      Noise    : constant Real := Matcher_Noise;
      Lens     : constant Fit.Lens := (Fx => Focal, Fy => Focal, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0,
                                       K1 => 0.0, K2 => 0.0);
      Table    : constant Vec3 := [0.0, -0.6, -0.8];
      Base_Axes : constant array (1 .. N) of Vec3 :=
        [[0.1, -0.9, 0.4], [1.0, 0.1, 0.05], [0.95, -0.1, 0.1], [1.0, 0.05, -0.1], [0.05, 0.85, 0.5], [0.0, 0.05, 1.0]];
      Base_Points : constant array (1 .. N) of Vec3 :=
        [[0.3, 0.5, 0.2], [0.0, 0.4, 0.4], [0.0, 0.25, 0.3], [0.0, 0.1, 0.15], [0.05, 0.05, 0.1], [0.02, 0.03, 0.0]];
      Most_Frames : constant := 1 + N * 6 + Cells + 4 * N * (N - 1) / 2;
      Told_Right, Told_Wrong, Untold_Right, Untold_Wrong, Unfitted : Natural := 0;
      type Verdict is (Not_Fitted, Right_Told, Wrong_Told, Right_Untold, Wrong_Untold);
      Outcome : array (1 .. Arms) of Verdict := [others => Not_Fitted];

      --  One arm: drawn, swept, fitted and scored. The arms draw from generators of their own and share nothing, so they
      --  are fitted at once on the workers: twelve one after another took a hundred seconds, past the two-minute cap
      --  whenever anything else ran (10-20).
      procedure One_Arm (Arm : Positive) is
         Rng      : Generator := (State => Seed + 7919 * Long_Long_Integer (Arm));
         Keep_Rng : Generator := (State => Seed + 104_729 * Long_Long_Integer (Arm));
         Kept_Point : constant array (1 .. Columns * Rows) of Boolean :=
           [for K in 1 .. Columns * Rows => Kept_Share >= 1.0 or else Uniform (Keep_Rng) < Kept_Share];
         Truth    : Fit.Joint_Array (1 .. N);
         Order_Of : array (1 .. N) of Positive;
         Gain     : Real_Array (1 .. N);
         Changes  : Real_Matrix (1 .. Most_Frames, 1 .. N) := [others => [others => 0.0]];
         Frames   : Natural := 1;
         type Sighting_Access is access Fit.Sighting_Array;
         All_Seen : constant Sighting_Access := new Fit.Sighting_Array (1 .. Most_Frames * Columns * Rows);
         Seen     : Natural := 0;
         Turned   : array (1 .. N, 1 .. N) of Boolean := [others => [others => False]];
         Rounds   : Natural := 0;
         Want_Chain : Fit.Chain_Order (1 .. N);
         Joints   : Fit.Joint_Array (1 .. N);
         Chain    : Fit.Chain_Order (1 .. N);
         Found    : Fit.Lens;
         Report   : Fit.Fit_Report;

         --  The sightings of the keyframes from First on, at the true arm, with their errors.
         procedure Sight (First : Positive) is
         begin
            for F in First .. Frames loop
               declare
                  D : Real_Array (1 .. N);
                  T : Rigid;
                  Shared_U : constant Real := Frame_Error * Gaussian (Rng);
                  Shared_V : constant Real := Frame_Error * Gaussian (Rng);
               begin
                  for J in 1 .. N loop
                     D (J) := Changes (F, J);
                  end loop;
                  T := Inverse (Fit.Eye_At (Truth, Along (Truth), D));
                  for Gy in 1 .. Rows loop
                     for Gx in 1 .. Columns loop
                        declare
                           K  : constant Positive := (Gy - 1) * Columns + Gx;
                           U0 : constant Real := (Real (Gx) - 0.5) * Real (Width) / Real (Columns);
                           V0 : constant Real := (Real (Gy) - 0.5) * Real (Height) / Real (Rows);
                           H  : constant Vec3 := Fit.Ray (Lens, U0, V0);
                           X  : constant Vec3 := (-1.0 / Real'(Unit (Table) * H)) * H;
                           U, V : Real;
                           Ahead : Boolean;
                        begin
                           Fit.Project (Lens, T * X, U, V, Ahead);
                           U := U + Noise * Gaussian (Rng) + Shared_U;
                           V := V + Noise * Gaussian (Rng) + Shared_V;
                           if Ahead and then U in 0.0 .. Real (Width) and then V in 0.0 .. Real (Height)
                             and then Kept_Point (K)
                           then
                              Seen := Seen + 1;
                              All_Seen (Seen) := (Frame => F, Track => K, U0 => U0, V0 => V0, U => U, V => V,
                                                  Own => 0.0, Own_Freedom => 0, others => <>);
                           end if;
                        end;
                     end loop;
                  end loop;
               end;
            end loop;
         end Sight;

         procedure Fit_Now is
            Given : Real_Matrix (1 .. Frames, 1 .. N);
            Asked : Fit.Flag_Matrix (1 .. Frames, 1 .. N);
         begin
            for F in 1 .. Frames loop
               for I in 1 .. N loop
                  Given (F, I) := Gain (I) * Changes (F, Order_Of (I));
                  Asked (F, I) := Changes (F, Order_Of (I)) /= 0.0;
               end loop;
            end loop;
            Fit.Fit (Given, Asked, [for I in 1 .. N => abs Gain (I) * 0.01], All_Seen (1 .. Seen), Width, Height, 0,
                     Joints, Chain, Found, Report);
         end Fit_Now;
      begin
         --  The arm: each axis tilted and each joint's place scaled and moved by a draw.
         for J in 1 .. N loop
            declare
               W : constant Vec3 := Unit (Base_Axes (J) + Tilt * Vec3'[Gaussian (Rng), Gaussian (Rng), Gaussian (Rng)]);
               P : constant Vec3 := (0.6 + 0.8 * Uniform (Rng)) * Base_Points (J)
                 + 0.03 * Vec3'[Gaussian (Rng), Gaussian (Rng), Gaussian (Rng)];
            begin
               Truth (J) := (W => W, P => P - Real'(P * W) * W, C => 1.0, Slide => False);
            end;
         end loop;
         --  The readings: a drawn order (Fisher-Yates) and unit, a power of two of either sign.
         for I in 1 .. N loop
            Order_Of (I) := I;
         end loop;
         for I in reverse 2 .. N loop
            declare
               J : constant Positive := 1 + Natural (Real'Floor (Uniform (Rng) * Real (I))) mod I;
               X : constant Positive := Order_Of (I);
            begin
               Order_Of (I) := Order_Of (J);
               Order_Of (J) := X;
            end;
         end loop;
         for I in 1 .. N loop
            Gain (I) := (if Uniform (Rng) < 0.5 then -1.0 else 1.0) * 2.0 ** (Integer (Real'Floor (Uniform (Rng) * 5.0)) - 2);
            Want_Chain (Order_Of (I)) := I;
         end loop;
         --  The sweep: every joint alone at its levels, then the cells.
         for J in 1 .. N loop
            for L in Levels'Range loop
               Frames := Frames + 1;
               Changes (Frames, J) := Levels (L);
            end loop;
         end loop;
         for Row in 1 .. Cells loop
            Frames := Frames + 1;
            for J in 1 .. N loop
               declare
                  Bits : Natural := 0;
                  R    : Natural := Row;
                  C    : Natural := J;
               begin
                  while R > 0 and then C > 0 loop
                     if R mod 2 = 1 and then C mod 2 = 1 then
                        Bits := Bits + 1;
                     end if;
                     R := R / 2;
                     C := C / 2;
                  end loop;
                  Changes (Frames, J) := (if Bits mod 2 = 0 then 0.25 else -0.25) * Widest;
               end;
            end loop;
         end loop;
         Sight (2);
         loop
            Fit_Now;
            Rounds := Rounds + 1;
            exit when not Report.Fitted or else Report.Order_Told;
            declare
               Fresh : Natural := 0;
               First : constant Positive := Frames + 1;
            begin
               for K in Report.Untold.First_Index .. Report.Untold.Last_Index loop
                  declare
                     P     : constant Joint_Pair := Report.Untold (K);
                     Place : constant Natural := Report.Untold_Turns.First_Index + 2 * (K - Report.Untold.First_Index);
                     --  Each joint's turn by the truth's numbering: a reading is its joint's change times its gain.
                     function Size_Of (Turn : Positive; Reading : Positive) return Real is
                       (if Named and then Place + 1 <= Report.Untold_Turns.Last_Index
                        then Report.Untold_Turns (Turn) / abs Gain (Reading) else Widest);
                  begin
                     --  A pair is turned once, whichever way along the arm the fit puts it: the order the fit keeps
                     --  changes between rounds, and a pair named both ways was turned twice, past the keyframes the
                     --  exam keeps (arm 5 of the hard exam, 10-20).
                     if not Turned (P.Earlier, P.Later) and then not Turned (P.Later, P.Earlier) then
                        Turned (P.Earlier, P.Later) := True;
                        Fresh := Fresh + 1;
                        --  The pair's joints by the truth's numbering: the readings are drawn from them.
                        for S1 of Real_Array'[1.0, -1.0] loop
                           for S2 of Real_Array'[1.0, -1.0] loop
                              Frames := Frames + 1;
                              Changes (Frames, Order_Of (P.Earlier)) := S1 * Size_Of (Place, P.Earlier);
                              Changes (Frames, Order_Of (P.Later)) := S2 * Size_Of (Place + 1, P.Later);
                           end loop;
                        end loop;
                     end if;
                  end;
               end loop;
               exit when Fresh = 0;
               Sight (First);
            end;
         end loop;
         declare
            Right : constant Boolean := Report.Fitted and then Fit."=" (Chain, Want_Chain);
         begin
            if not Report.Fitted then
               Outcome (Arm) := Not_Fitted;
            elsif Report.Order_Told and then Right then
               Outcome (Arm) := Right_Told;
            elsif Report.Order_Told then
               Outcome (Arm) := Wrong_Told;
            elsif Right then
               Outcome (Arm) := Right_Untold;
            else
               Outcome (Arm) := Wrong_Untold;
            end if;
            Driver.Log.Line (Driver.Log.Robot, "order exam: arm" & Arm'Image & ": true" & Chain_Text (Want_Chain)
                             & ", fitted" & (if Report.Fitted then Chain_Text (Chain) else " (not fitted: "
                                             & Ada.Strings.Unbounded.To_String (Report.Why) & ")")
                             & (if Report.Order_Told then ", told" else ", not told") & " after" & Rounds'Image
                             & " fits and" & Natural'Image ((Frames - 1 - N * 6 - Cells) / 4) & " pairs turned: "
                             & (if Right then "right" else "wrong"));
         end;
      end One_Arm;
      procedure Every_Arm is new Driver.Parallel_For (One_Arm);
   begin
      Every_Arm (1, Arms);
      for Arm in 1 .. Arms loop
         case Outcome (Arm) is
            when Not_Fitted => Unfitted := Unfitted + 1;
            when Right_Told => Told_Right := Told_Right + 1;
            when Wrong_Told => Told_Wrong := Told_Wrong + 1;
            when Right_Untold => Untold_Right := Untold_Right + 1;
            when Wrong_Untold => Untold_Wrong := Untold_Wrong + 1;
         end case;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "order exam:" & Arms'Image & " arms: told and right" & Told_Right'Image
                       & ", told and wrong" & Told_Wrong'Image & ", not told but right" & Untold_Right'Image
                       & ", not told and wrong" & Untold_Wrong'Image & ", not fitted" & Unfitted'Image);
      Check (Told_Wrong = 0, "the fit told a wrong order for" & Told_Wrong'Image & " arms");
   end Order_Exam_At;

   procedure Order_Exam is
   begin
      Order_Exam_At (640, 480, 400.0, 1.0, 0.3, 0.1, 0.3, 1000);
   end Order_Exam;

   --  As the fake robot's eyes are: 320 by 240 pixels, a focal length of about 143 (its wrist eyes'), a third of the
   --  points textured (a white table leaves the aligner a fifth of its query points, R6), the matcher's noise 0.5 px
   --  and a keyframe's shared error 0.2, and the arms' axes tilted further from the template.
   procedure Order_Exam_Hard is
   begin
      Order_Exam_At (320, 240, 143.0, 0.33, 0.5, 0.2, 0.5, 5000);
   end Order_Exam_Hard;

   --  The order exam's arms swept a quarter as wide (the hall's wrist eyes near their tables, 10-25: a shoulder turns a
   --  few degrees before its eye's view has moved half the picture), each pair the fit names turned by the sizes of the
   --  turns it names, as Boot turns it: no wrong order may be told. Boot had taken a turn named with a minus sign for
   --  none (10-25, Fit_Report.Untold_Turns).
   procedure Order_Exam_Named is
   begin
      Order_Exam_At (640, 480, 400.0, 1.0, 0.3, 0.1, 0.3, 1000, Widest_Level => 0.05, Named => True);
   end Order_Exam_Named;

   procedure Kinematics_Of_A_Drawn_Synthetic_Arm is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True, Drawn => True);
   end Kinematics_Of_A_Drawn_Synthetic_Arm;

   --  The table that the tracks of an arm give: the tracks of a recording's fit (tests/data: per track the line of
   --  sight in the reference eye, its refined depth and the depth's sigma), whose plane the arm calls its table. The
   --  wrist cameras of those recordings look 30 degrees below the horizontal, so the true vertical in the eye's frame
   --  is (0, -0.866, -0.5), and the table's normal, which points to the eye, is that.
   function Table_Off_The_Vertical (Path : String; Tracks : out Natural) return Real is
      package Fit renames Driver.Robot.Kinematics.Fit;
      F      : Ada.Text_IO.File_Type;
      Points : Fit.Sight_Point_Array (1 .. 1000);
      Plane  : Fit.Sight_Plane;
      On     : Fit.Flag_Array (1 .. 1000) := [others => False];
      Up     : constant Vec3 := [0.0, -0.8660254037844386, -0.5];
   begin
      Tracks := 0;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            Line  : constant String := Ada.Text_IO.Get_Line (F);
            Value : array (1 .. 7) of Real;
            Pos   : Positive := Line'First + 5;   --  after SIGHT
         begin
            for V in Value'Range loop
               while Pos <= Line'Last and then Line (Pos) = ' ' loop
                  Pos := Pos + 1;
               end loop;
               declare
                  Stop : Positive := Pos;
               begin
                  while Stop <= Line'Last and then Line (Stop) /= ' ' loop
                     Stop := Stop + 1;
                  end loop;
                  Value (V) := Real'Value (Line (Pos .. Stop - 1));
                  Pos := Stop;
               end;
            end loop;
            Tracks := Tracks + 1;
            Points (Tracks) := (H => [Value (3), Value (4), Value (5)], Depth => Value (6), Sigma => Value (7));
         end;
      end loop;
      Ada.Text_IO.Close (F);
      Fit.Dominant_Plane (Points (1 .. Tracks), Plane, On (1 .. Tracks));
      if not Plane.Found then
         return Ada.Numerics.Pi;
      end if;
      return Ada.Numerics.Long_Elementary_Functions.Arccos (Real'Min (1.0, Fit.Plane_Normal (Plane) * Up));
   end Table_Off_The_Vertical;

   --  In a picture that looks along the table the room's far wall holds about as many of the tracks as the table does
   --  (A23's second arm: 102 on the wall, 116 on the table), and the plane the most tracks lie on, scored by its
   --  truncated cost, was the wall: the arm's up 87 degrees off the vertical (the wall is vertical), and a table that
   --  its link to the other arm could not find in the head's view. Another recording holds a thing on the table 1 to
   --  1.5 units nearer than the table, with 14 tracks against the table's 82: the table stays the table.
   procedure Kinematics_Table_Of_The_Tracks is
      Tracks : Natural;
      Off    : Real;
   begin
      Off := Table_Off_The_Vertical ("tests/data/table_and_far_wall.txt", Tracks);
      Check (Off < 0.01745, "the table of an arm that sees the far wall is" & Real'Image (Off * 57.29578)
             & " degrees off the vertical, of" & Tracks'Image & " tracks");
      Off := Table_Off_The_Vertical ("tests/data/table_and_a_thing.txt", Tracks);
      Check (Off < 0.01745, "the table of an arm that sees a thing on it is" & Real'Image (Off * 57.29578)
             & " degrees off the vertical, of" & Tracks'Image & " tracks");
   end Kinematics_Table_Of_The_Tracks;

   --  The final refinement takes a step when it moves some combination of the
   --  parameters by more than Unchanged_Fraction of its standard error,
   --  whatever the cost: stopped by a hundredth of the cost, A10's and A11's
   --  fits (a cost of some 20,000, a hundredth of it 200) stood several
   --  standard errors short on the axes the data barely determine, the focal
   --  length against the distortion (a step of 3 standard errors lowers the
   --  cost by 4.5 at most), and A11's second arm's focal lengths moved by half
   --  a pixel, 1.5 of their sigmas, once they were let run on.
   procedure Refinement_Takes_Steps_That_Move_The_Fit is
      Fraction : constant Real := Driver.Conventions.Unchanged_Fraction;
      package Fit renames Driver.Robot.Kinematics.Fit;
   begin
      --  Moving m standard errors along an axis lowers the cost by m squared over two at most.
      Check (Fit.Moves_The_Fit (0.5 * (2.0 * Fraction) ** 2),
             "a step of two Unchanged_Fractions of a standard error is not taken");
      Check (not Fit.Moves_The_Fit (0.5 * (0.5 * Fraction) ** 2),
             "a step of half an Unchanged_Fraction of a standard error is taken");
      Check (Fit.Moves_The_Fit (4.5),
             "a step of 3 standard errors along a weak axis is not taken for it is a small part of the cost");
      Check (not Fit.Moves_The_Fit (0.0) and then not Fit.Moves_The_Fit (-1.0),
             "a step that does not lower the cost is taken");
   end Refinement_Takes_Steps_That_Move_The_Fit;

   ---------------------------------------------------------------------------
   --  The ends a channel shows by stopping (Motion.Note_Stopped, End_Of, the planner's refusal).
   --
   --  The pushes are the arm's own, from A22's and A27's runs (third joint, beats in the names): a push the arm was
   --  stopped in by itself leaves one channel far short of the rest, and one the table stopped leaves the shortfall
   --  spread over the channels asked as the ask was.

   Six : constant := 6;

   --  A group of six channels, its noise measured and its free pushes few and tiny, as a boot leaves them, first seen at 1
   --  (beyond every reading the pushes below stop at, so that what the channels have been seen at is theirs).
   procedure Ends_Body (M : in out Model) is
      Idle : Driver.Commands.Command;
      O    : Observation;
   begin
      O.Readings.Append (Real_Array'(1 .. Six => 1.0));
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      O.Images.Append (Driver.Images.No_Image);
      O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      Driver.Commands.Set_Target (Idle, 1, Real_Array'(1 .. Six => 1.0));
      Driver.Robot.Channels.Append (M, O, Idle);
      M.Beats := M.Beats + 1;
      for C in 1 .. Six loop
         M.Noise.Append (1.0e-6);
         M.Noise_Freedom.Append (100);
      end loop;
      for F of Real_Array'[1.0e-5, 2.0e-5, 0.0, 0.0, 0.0] loop
         M.Groups (1).Free_Shortfalls.Append (F);
      end loop;
   end Ends_Body;

   --  One push of the group as the step tracker would have followed it: held at Start, asked Aim, stopped at Stop and
   --  at rest there. The episode is the one it would have judged.
   procedure Ends_Push
     (M : in out Model; Start, Aim, Stop : Real_Array; Blocked : Boolean := True; Rested : Boolean := True)
   is
      Sent   : Driver.Commands.Command;
      Began  : Natural;
      Length : Real := 0.0;
      procedure Beat (Reading, Target : Real_Array) is
         O : Observation;
      begin
         O.Readings.Append (Reading);
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
         Driver.Commands.Set_Target (Sent, 1, Target);
         Driver.Robot.Channels.Append (M, O, Sent);
         M.Beats := M.Beats + 1;
      end Beat;
   begin
      Beat (Start, Start);
      Beat (Start, Start);
      Began := M.Beats;
      Beat (Start, Aim);
      Beat (Stop, Aim);
      Beat (Stop, Aim);
      for C in 1 .. Six loop
         Length := Length + (Aim (C) - Start (C)) ** 2;
      end loop;
      M.Groups (1).Episodes.Append
        (Episode'(Start => Began, Ended => True, End_At => M.Beats - 1, Settled => True, Rested => Rested,
                  Blocked => Blocked, Length => Ada.Numerics.Long_Elementary_Functions.Sqrt (Length), others => <>));
   end Ends_Push;

   --  Beat numbers are A27's (right arm, hand 2, third joint last but one; the fifth is the third joint of A22's press 2).
   A27_6152_Start : constant Real_Array := [-0.0428, 0.4451, 0.3234, -0.7702, -0.2622, 0.4558];
   A27_6152_Aim   : constant Real_Array := [-0.0403, 0.5985, -0.0485, -0.2437, -0.2609, 0.4581];
   A27_6152_Stop  : constant Real_Array := [-0.0440, 0.5844, 0.0705, -0.2321, -0.2601, 0.4579];
   A22_13827_Start : constant Real_Array := [0.1172, -0.0794, -0.0249, 0.0552, 0.5253, -0.9512];
   A22_13827_Aim   : constant Real_Array := [0.1172, -0.1354, -0.0578, 0.0320, 0.5253, -0.9512];
   A22_13827_Stop  : constant Real_Array := [0.1172, -0.1353, -0.0359, 0.0320, 0.5253, -0.9512];

   procedure Limiter_Finds_The_Channel_That_Stopped is
      --  (a) A27's aim at 6152: the third joint 0.119 short of 0.372 asked (32 per cent), the second 0.014 of 0.153
      --      (9), the fourth gone past its target: the third is the limiter.
      --  (b) A22's press 2 at 13827: the third joint 0.0219 of 0.0329 short, the rest on their targets: the third.
      --  (c) a push that asked the third joint only: the third.
      --  (d) the shares equal in three channels: none. (A27's push at 6277, the table's, fell short by 16, 28 and 22 per cent
      --      of the three joints' asks; those differ by many sigmas of the readings' noise, so the limiter alone does
      --      not tell a table's push from a joint's: the caller says whether the body stopped it, and the hand does
      --      not say so of a press that rests on the table.)
      --  (f) a push given up while moving, or not blocked: none.
      function Limiter_Of (Start, Aim, Stop : Real_Array; Blocked, Rested : Boolean := True) return Natural is
         M : Model;
      begin
         Ends_Body (M);
         Ends_Push (M, Start, Aim, Stop, Blocked, Rested);
         return Driver.Robot.Steps.Limiter (M, 1, Driver.Robot.Steps.Latest (M, 1));
      end Limiter_Of;
   begin
      Check (Limiter_Of (A27_6152_Start, A27_6152_Aim, A27_6152_Stop) = 3,
             "the aim the third joint stopped (A27, 6152) has limiter" & Limiter_Of (A27_6152_Start, A27_6152_Aim, A27_6152_Stop)'Image);
      Check (Limiter_Of (A22_13827_Start, A22_13827_Aim, A22_13827_Stop) = 3,
             "the push the third joint's end stopped (A22, 13827) has limiter"
             & Limiter_Of (A22_13827_Start, A22_13827_Aim, A22_13827_Stop)'Image);
      Check (Limiter_Of ([0.0, 0.0, -0.0228, 0.0, 0.0, 0.0], [0.0, 0.0, -0.1016, 0.0, 0.0, 0.0],
                         [0.0, 0.0, -0.0368, 0.0, 0.0, 0.0]) = 3,
             "the push that asked the third joint only (A27, 6567) has another limiter");
      Check (Limiter_Of ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.1, 0.1, 0.0, 0.0, 0.0], [0.05, 0.05, 0.05, 0.0, 0.0, 0.0]) = 0,
             "a push short by the same share in three channels has a limiter");
      Check (Limiter_Of ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.1, 0.0, 0.0, 0.0, 0.0], [0.05, 0.050001, 0.0, 0.0, 0.0, 0.0]) = 0,
             "a channel short by a millionth more than another, within the scatter of free pushes, is the limiter");
      Check (Limiter_Of ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0999999, 0.0, 0.0, 0.0, 0.0, 0.0]) = 0,
             "the only channel asked, short by a ten-millionth of a unit, within the scatter of free pushes, is the limiter");
      Check (Limiter_Of (A27_6152_Start, A27_6152_Aim, A27_6152_Stop, Rested => False) = 0,
             "a push given up while its readings moved has a limiter");
      Check (Limiter_Of (A27_6152_Start, A27_6152_Aim, A27_6152_Stop, Blocked => False) = 0,
             "a push that was not blocked has a limiter");
      --  (g) The scatter of the free pushes (2e-5 here) is the whole shortfall's, and counts once in the difference of
      --      two channels' parts. Two channels asked alike have parts of the shortfall along the ask that are their
      --      shortfalls over the root of two. Short by parts 1.0e-4 and 3.0e-5 they differ by 7e-5: more than Z of the
      --      two channels' noise with the scatter once (6.0e-5), less than Z of it counted for each channel (8.5e-5),
      --      and the first is the limiter. Short by 1.0e-4 and 5.0e-5 they differ by 5e-5, less than either: none.
      declare
         Root_Two : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0);
         function Two_Short_By (First, Second : Real) return Natural is
           (Limiter_Of ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.1, 0.0, 0.0, 0.0, 0.0],
                        [0.1 - First * Root_Two, 0.1 - Second * Root_Two, 0.0, 0.0, 0.0, 0.0]));
      begin
         Check (Two_Short_By (1.0e-4, 3.0e-5) = 1,
                "two channels whose parts differ by more than Z of their noise and the scatter of free pushes once"
                & " have limiter" & Two_Short_By (1.0e-4, 3.0e-5)'Image);
         Check (Two_Short_By (1.0e-4, 5.0e-5) = 0,
                "two channels whose parts differ by less than the scatter of free pushes have limiter"
                & Two_Short_By (1.0e-4, 5.0e-5)'Image);
      end;
   end Limiter_Finds_The_Channel_That_Stopped;

   procedure A_Lone_Channel_Is_The_Limiter_Only_If_Short_By_More_Than_Its_Scatter is
      --  A group of one channel (a closer): nothing to compare it with, so it is the limiter when it fell short by
      --  more than Z standard deviations of its noise and the group's free pushes, and not when it fell short by less.
      function Limiter_Of (Aim, Stop : Real) return Natural is
         M    : Model;
         Sent : Driver.Commands.Command;
         procedure Beat (Reading, Target : Real) is
            O : Observation;
         begin
            O.Readings.Append (Real_Array'(1 => Reading));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            O.Images.Append (Driver.Images.No_Image);
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            Driver.Commands.Set_Target (Sent, 1, Real_Array'(1 => Target));
            Driver.Robot.Channels.Append (M, O, Sent);
            M.Beats := M.Beats + 1;
         end Beat;
      begin
         Beat (1.0, 1.0);
         Beat (0.0, 0.0);
         Beat (0.0, 0.0);
         M.Noise.Append (1.0e-6);
         M.Noise_Freedom.Append (100);
         for F of Real_Array'[1.0e-5, 2.0e-5, 0.0, 0.0, 0.0] loop
            M.Groups (1).Free_Shortfalls.Append (F);
         end loop;
         Beat (0.0, Aim);
         Beat (Stop, Aim);
         Beat (Stop, Aim);
         M.Groups (1).Episodes.Append
           (Episode'(Start => 3, Ended => True, End_At => 5, Settled => True, Rested => True, Blocked => True,
                     Length => abs Aim, others => <>));
         return Driver.Robot.Steps.Limiter (M, 1, Driver.Robot.Steps.Latest (M, 1));
      end Limiter_Of;
   begin
      Check (Limiter_Of (0.1, 0.0999999) = 0, "a lone channel short by 1e-7, within the scatter of free pushes, is the limiter");
      Check (Limiter_Of (0.1, 0.05) = 1, "a lone channel short by half its ask is not the limiter");
   end A_Lone_Channel_Is_The_Limiter_Only_If_Short_By_More_Than_Its_Scatter;

   procedure Stops_Are_Noted_As_Ends is
      --  The third joint, A27 hand 2's: it stopped against the arm itself at +0.0705 (beat 6152), at its low end
      --  -0.0368 (6567), against the forearm at +0.0194 (6642), and at its low end again, -0.0359 (7041), from another
      --  pose. A stop is an end of its channel when the channel stops at the same reading, to within what two contacts
      --  would not meet by chance, from another pose of the other channels: until then it is noted, and no plan
      --  is refused. Nothing for a channel whose noise is not measured, and the way up is its own.
      M : Model;
      Noted : Boolean;
      Third_Joint_Down : Boolean;
   begin
      Ends_Body (M);
      Check (not Known (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing)), "an end before any stop");
      --  (a) A27's aim at 6152, the third joint stopped at +0.0705: one stop, noted, no end.
      Ends_Push (M, A27_6152_Start, A27_6152_Aim, A27_6152_Stop);
      Driver.Robot.Motion.Note_Stopped (M, 1, Noted);
      Check (not Noted and then Natural (M.Groups (1).Stops.Length) = 1, "the first stop of the third joint moved an end or was not kept");
      Check (not Known (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing)), "one stop made an end");
      --  (b) -0.0368 from a pose with the other joints elsewhere: the readings differ by a tenth of a radian, not an end.
      Ends_Push (M, [0.0, 0.0, 0.0705, 0.0, 0.0, 0.0], [0.0, 0.0, -0.1016, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0368, 0.0, 0.0, 0.0]);
      Driver.Robot.Motion.Note_Stopped (M, 1, Noted);
      Check (not Noted and then Natural (M.Groups (1).Stops.Length) = 2, "two stops a tenth of a radian apart made an end");
      Check (not Known (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing)), "two stops at different readings made an end");
      --  (c) The same reading again from the same pose is the same contact seen twice.
      Ends_Push (M, [0.0, 0.0, 0.0705, 0.0, 0.0, 0.0], [0.0, 0.0, -0.1016, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0368, 0.0, 0.0, 0.0]);
      Driver.Robot.Motion.Note_Stopped (M, 1, Noted);
      Check (not Noted and then Natural (M.Groups (1).Stops.Length) = 2, "the same stop from the same pose was kept or made an end");
      Check (not Known (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing)), "a stop seen twice from one pose made an end");
      --  (d) -0.0359 from another pose, 0.9 thousandths of a radian from the second: found again, an end.
      Ends_Push (M, [0.1, 0.2, 0.3, 0.3, 0.1, 0.1], [0.1, 0.2, -0.1016, 0.3, 0.1, 0.1], [0.1, 0.2, -0.0359, 0.3, 0.1, 0.1]);
      Driver.Robot.Motion.Note_Stopped (M, 1, Noted);
      Check (Noted, "a stop found again from another pose moved no end");
      declare
         End_Is : constant Estimate := Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing);
      begin
         Third_Joint_Down := Known (End_Is) and then abs (End_Is.Value + 0.0368) < 1.0e-12 and then abs (End_Is.Sigma - 1.0e-6) < 1.0e-12;
      end;
      Check (Third_Joint_Down, "the end found twice (-0.0368, -0.0359) is not the furthest of the two, or its sigma is not the channel's noise");
      Check (Natural (M.Groups (1).Stops.Length) = 1, "the stops that made the end were kept as stops as well");
      Check (not Known (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Increasing)), "a stop going down is an end going up");
      Check (not Known (Driver.Robot.End_Of (M, 1, 2, Driver.Robot.Decreasing)), "the second joint, short but not the limiter, has an end");
      --  (e) A stop where the arm met itself, against an end the channel has: the end explains it, nothing is kept.
      Ends_Push (M, [0.0, 0.0, 0.3, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0273, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0194, 0.0, 0.0, 0.0]);
      Driver.Robot.Motion.Note_Stopped (M, 1, Noted);
      Check (not Noted and then Natural (M.Groups (1).Stops.Length) = 1, "a stop short of the end it explained was kept or moved the end");
      Check (abs (Driver.Robot.End_Of (M, 1, 3, Driver.Robot.Decreasing).Value + 0.0368) < 1.0e-12,
             "a stop where the arm met itself (+0.0194) moved the end it had found (-0.0368)");
      --  The stops are readings, and a channel is never taken to end nearer than its readings seen.
      Check (M.Groups (1).Has_Stopped_Low (2) and then abs (M.Groups (1).Stopped_Low (2) + 0.0368) < 1.0e-12
             and then not M.Groups (1).Has_Stopped_High (2),
             "the end kept is not the furthest down (-0.0368) of the two, or one is kept going up");
      --  A push is a stop when its caller says so: the sixth joint asked up and stopped short, on a body of its own.
      declare
         Fresh : Model;
      begin
         Ends_Body (Fresh);
         Ends_Push (Fresh, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]);
         Check (Natural (Fresh.Groups (1).Stops.Length) = 0, "a stop was kept without the caller's word");
         Driver.Robot.Motion.Note_Stopped (Fresh, 1);
         Check (Natural (Fresh.Groups (1).Stops.Length) = 1 and then not Known (Driver.Robot.End_Of (Fresh, 1, 6, Driver.Robot.Increasing)),
                "the sixth joint asked up and stopped short at 0.1 made an end, or was not kept");
      end;
      declare
         Unmeasured : Model;
      begin
         Ends_Body (Unmeasured);
         Unmeasured.Noise.Clear;   --  the noise is not measured
         Ends_Push (Unmeasured, A27_6152_Start, A27_6152_Aim, A27_6152_Stop);
         Driver.Robot.Motion.Note_Stopped (Unmeasured, 1);
         Check (Natural (Unmeasured.Groups (1).Stops.Length) = 0 and then not Known (Driver.Robot.End_Of (Unmeasured, 1, 3, Driver.Robot.Decreasing)),
                "a push of a channel whose noise was not measured was kept or showed an end");
      end;
      --  A stop going up does not make an end with one going down at the same reading, and the other way about.
      declare
         Ways : Model;
      begin
         Ends_Body (Ways);
         Ends_Push (Ways, [0.0, 0.0, 0.3, 0.0, 0.0, 0.0], [0.0, 0.0, -0.1016, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0368, 0.0, 0.0, 0.0]);
         Driver.Robot.Motion.Note_Stopped (Ways, 1);
         Ends_Push (Ways, [0.1, 0.2, -0.3, 0.3, 0.1, 0.1], [0.1, 0.2, 0.2, 0.3, 0.1, 0.1], [0.1, 0.2, -0.0368, 0.3, 0.1, 0.1]);
         Driver.Robot.Motion.Note_Stopped (Ways, 1);
         Check (Natural (Ways.Groups (1).Stops.Length) = 2 and then not Known (Driver.Robot.End_Of (Ways, 1, 3, Driver.Robot.Decreasing))
                and then not Known (Driver.Robot.End_Of (Ways, 1, 3, Driver.Robot.Increasing)),
                "a stop going up and one going down at one reading made an end");
      end;
      --  Seen, the channel is no nearer: an end found at -0.0368 and -0.0359 is not above a reading the channel has been
      --  seen at (-0.0415, the home pose of A27's boot).
      declare
         Seen : Model;
      begin
         Ends_Body (Seen);
         Ends_Push (Seen, [0.0, 0.0, -0.0415, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0415, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0415, 0.0, 0.0, 0.0]);
         Ends_Push (Seen, [0.0, 0.0, 0.3, 0.0, 0.0, 0.0], [0.0, 0.0, -0.1016, 0.0, 0.0, 0.0], [0.0, 0.0, -0.0368, 0.0, 0.0, 0.0]);
         Driver.Robot.Motion.Note_Stopped (Seen, 1);
         Ends_Push (Seen, [0.1, 0.2, 0.3, 0.3, 0.1, 0.1], [0.1, 0.2, -0.1016, 0.3, 0.1, 0.1], [0.1, 0.2, -0.0359, 0.3, 0.1, 0.1]);
         Driver.Robot.Motion.Note_Stopped (Seen, 1);
         Check (abs (Driver.Robot.End_Of (Seen, 1, 3, Driver.Robot.Decreasing).Value + 0.0415) < 1.0e-12,
                "an end above a reading the channel was seen at, which it can reach");
      end;
   end Stops_Are_Noted_As_Ends;

   procedure Plan_Refuses_Past_An_End is
      --  A goal past the first joint's travel (0.45) is planned when no end is known, refused when the first joint
      --  has shown an end up at 0.3 inside the path, and planned when the end is at the goal's reading within the
      --  noise or past it.
      M : Model;
      O : Observation;
      function Planned_With (End_At : Real) return Driver.Robot.Motion.Plan is
         S : Group_Stream renames M.Groups (1);
      begin
         S.Stopped_High.Clear;
         S.Has_Stopped_High.Clear;
         S.Stopped_Low.Clear;
         S.Has_Stopped_Low.Clear;
         for C in 1 .. 6 loop
            S.Stopped_High.Append (if C = 1 then End_At else 0.0);
            S.Has_Stopped_High.Append (C = 1 and then End_At < Real'Last);
            S.Stopped_Low.Append (0.0);
            S.Has_Stopped_Low.Append (False);
         end loop;
         return Driver.Robot.Motion.Plan_Reach
           (M, 1, O, (Pose => Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Beyond_Goal), Position_Only => False, Free_About => <>));
      end Planned_With;
      use type Driver.Robot.Motion.Plan_Status;
   begin
      Measured_Body (M);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      Check (Driver.Robot.Motion.Status (Planned_With (Real'Last)) = Driver.Robot.Motion.Planned,
             "a goal past the readings seen is not planned while no end is known");
      declare
         Refused : constant Driver.Robot.Motion.Plan := Planned_With (0.3);
      begin
         Check (Driver.Robot.Motion.Status (Refused) = Driver.Robot.Motion.Unreachable
                and then Ada.Strings.Fixed.Index (Driver.Robot.Motion.Why (Refused), "would pass its end") > 0,
                "a path past an end the first joint showed is planned, or refused for another reason: "
                & Driver.Robot.Motion.Why (Refused));
      end;
      Check (Driver.Robot.Motion.Status (Planned_With (Beyond_Goal (1))) = Driver.Robot.Motion.Planned,
             "a path that ends at the end (within the noise) is refused");
      Check (Driver.Robot.Motion.Status (Planned_With (2.0)) = Driver.Robot.Motion.Planned,
             "a path wholly short of an end is refused: " & Driver.Robot.Motion.Why (Planned_With (2.0)));
   end Plan_Refuses_Past_An_End;

   procedure Ends_Are_Kept_In_The_Body_File is
      --  Written and read back, the body has the ends it showed, and the file the reloaded body writes is the same.
      M, Back : Model;
      Ok      : Boolean;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Measured_Body (M);
      for C in 1 .. 6 loop
         M.Groups (1).Stopped_Low.Append (-1.0 - 0.1 * Real (C));
         M.Groups (1).Stopped_High.Append (0.5 * Real (C));
         M.Groups (1).Has_Stopped_Low.Append (C mod 2 = 1);
         M.Groups (1).Has_Stopped_High.Append (C mod 3 = 0);
      end loop;
      --  Two stops not yet found again, each with the pose it was made from.
      for K in 1 .. 2 loop
         declare
            Pose : Real_Vectors.Vector;
         begin
            for C in 1 .. 6 loop
               Pose.Append (0.1 * Real (K) + 0.01 * Real (C));
            end loop;
            M.Groups (1).Stops.Append (Stop_Record'(Channel => 1 + K, Up => K = 1, Value => 0.7 * Real (K), Pose => Pose));
         end;
      end loop;
      declare
         Written : constant String := Driver.Robot.Body_File.Text (M);
      begin
         Driver.Robot.Body_File.Read (Back, Written, Ok, Why);
         Check (Ok, "the body file was not read: " & Ada.Strings.Unbounded.To_String (Why));
         Check (Reloaded (Back, Stored_Ends), "the ends were not reloaded");
         for C in 1 .. 6 loop
            Check (Known (Driver.Robot.End_Of (Back, 1, C, Driver.Robot.Decreasing)) = (C mod 2 = 1)
                   and then Known (Driver.Robot.End_Of (Back, 1, C, Driver.Robot.Increasing)) = (C mod 3 = 0),
                   "the reloaded body has other ends than the written one's on channel" & C'Image);
         end loop;
         Check (abs (Driver.Robot.End_Of (Back, 1, 3, Driver.Robot.Increasing).Value - 1.5) < 1.0e-9
                and then abs (Driver.Robot.End_Of (Back, 1, 5, Driver.Robot.Decreasing).Value + 1.5) < 1.0e-9,
                "the reloaded ends are other numbers than the written ones");
         Check (Natural (Back.Groups (1).Stops.Length) = 2 and then Back.Groups (1).Stops (2).Channel = 3
                and then not Back.Groups (1).Stops (2).Up and then abs (Back.Groups (1).Stops (2).Value - 1.4) < 1.0e-9
                and then abs (Back.Groups (1).Stops (2).Pose (3) - 0.24) < 1.0e-9 and then Back.Groups (1).Stops (1).Up,
                "the reloaded body has other stops than the written one's, or their poses");
         Check (Driver.Robot.Body_File.Text (Back) = Written, "the reloaded body writes another file");
      end;
   end Ends_Are_Kept_In_The_Body_File;

   procedure Scales_Beside_Stated_Sigmas is
      --  Residuals each of the spread of a common floor (what the model leaves of any sighting) and a gain times what
      --  its sighting states, which grows from nothing to a pixel and a half as the view moved the more: the floor and
      --  the gain read from them are theirs, whether what is stated is borne out in full, in half or not at all (the
      --  aligner's stated sigmas, 10-19: 0.50 of them and none on two arms of the fake robot). Read so, the sightings
      --  that state much are not outliers by the noise of those that state little, nor those that state little by what
      --  others state. With nothing stated the floor is the noise the residuals show together, several times the floor.
      --  The bounds are four of the spreads the reading showed over 60 such draws (a floor of 0.10 +- 0.025 and a gain
      --  of 1.00 +- 0.041 at a gain of one, 0.10 +- 0.014 and 0.50 +- 0.021 at a half, 0.49 +- 0.015 and 0.06 +- 0.07
      --  at none).
      Count : constant := 2000;
      type Truth is record
         Floor, Gain, Floor_Within, Gain_Within : Real;
      end record;
      Cases : constant array (1 .. 3) of Truth :=
        [(Floor => 0.1, Gain => 1.0, Floor_Within => 0.1, Gain_Within => 0.16),
         (Floor => 0.1, Gain => 0.5, Floor_Within => 0.056, Gain_Within => 0.084),
         (Floor => 0.5, Gain => 0.0, Floor_Within => 0.06, Gain_Within => 0.28)];
      Rng   : Generator;
   begin
      for C of Cases loop
         declare
            R, Own : Real_Array (1 .. Count);
         begin
            for I in 1 .. Count loop
               Own (I) := 1.5 * Real (I) / Real (Count);
               R (I) := Ada.Numerics.Long_Elementary_Functions.Sqrt (C.Floor ** 2 + (C.Gain * Own (I)) ** 2)
                 * Gaussian (Rng);
            end loop;
            declare
               Read : constant Driver.Robot.Kinematics.Fit.Error_Model := Driver.Robot.Kinematics.Fit.Model_Of (R, Own);
            begin
               Check_Close (Read.Floor, C.Floor, C.Floor_Within, "the floor read beside what the sightings state, from "
                            & "residuals of floor" & Real'Image (C.Floor) & " and gain" & Real'Image (C.Gain) & ", is"
                            & Real'Image (Read.Floor));
               Check_Close (Read.Gain, C.Gain, C.Gain_Within, "the gain read beside the floor, from residuals of floor"
                            & Real'Image (C.Floor) & " and gain" & Real'Image (C.Gain) & ", is" & Real'Image (Read.Gain));
            end;
         end;
      end loop;
      declare
         R, None : Real_Array (1 .. Count);
         Shown   : Real_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            None (I) := 0.0;
            R (I) := Ada.Numerics.Long_Elementary_Functions.Sqrt (0.1 ** 2 + (1.5 * Real (I) / Real (Count)) ** 2)
              * Gaussian (Rng);
            Shown (I) := abs R (I);
         end loop;
         declare
            Alone : constant Driver.Robot.Kinematics.Fit.Error_Model := Driver.Robot.Kinematics.Fit.Model_Of (R, None);
         begin
            Check (Alone.Floor > 3.0 * 0.1,
                   "the noise the residuals show together is not several times their floor:" & Real'Image (Alone.Floor));
            Check_Close (Alone.Floor, Driver.Stats.Median (Shown) / Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5),
                         1.0E-9, "with nothing stated the floor is not the noise the residuals show");
            Check (Alone.Gain = 1.0, "with nothing stated the gain is not one:" & Real'Image (Alone.Gain));
         end;
      end;
      Check (Driver.Robot.Kinematics.Fit.Scale_Of ((Floor => 0.0, Gain => 0.0), 0.0) > 0.0,
             "a residual with neither floor nor stated has no scale");
      Check_Close (Driver.Robot.Kinematics.Fit.Scale_Of ((Floor => 0.3, Gain => 1.0), 0.4), 0.5, 1.0E-12,
                   "a scale is the floor and what is stated, as variances");
      Check_Close (Driver.Robot.Kinematics.Fit.Scale_Of ((Floor => 0.3, Gain => 0.5), 0.8), 0.5, 1.0E-12,
                   "a scale is the floor and the gain times what is stated, as variances");
   end Scales_Beside_Stated_Sigmas;

   --  A fixed eye placed by the edge flow of a synthetic arm (Kinematics.Flow_Eye): three joints (a turn about the
   --  vertical, two about level axes), points of its second and third links seen by an eye 0.9 away looking down at the
   --  arm, each with a gradient of 30 levels a pixel in a direction of its own and, for every joint, the change of the
   --  picture there a reading unit as the arm's motion has it (central differences of the point's projection, not the
   --  fit's own formula) with Gaussian noise of the sigma each states; the joints that do not move a point change it by
   --  noise alone. The lens and the pose come back within Z of their sigmas, and the eye is told.
   procedure Flow_Eye_On_A_Synthetic_Arm_With (Profiled : Boolean) is
      package Flow renames Driver.Robot.Kinematics.Flow_Eye;
      package KFit renames Driver.Robot.Kinematics.Fit;
      Width  : constant := 320;
      Height : constant := 240;
      Focal  : constant Real := 150.0;
      --  The pixels not square, by more than a camera's (the fake's eyes 0.995 to 1.005) so that the check tells the
      --  fit's aspect from square pixels: its sigma is 0.86 % (at 1.03 the fit said 1.0233), and at 1.004 and 1.03 the
      --  check passed against square pixels too (10-21).
      Aspect : constant Real := 1.06;
      Joints : aliased constant KFit.Joint_Array :=
        [(W => [0.0, 0.0, 1.0], P => [0.0, 0.0, 0.0], C => 0.001, Slide => False),
         (W => [1.0, 0.0, 0.0], P => [0.0, 0.0, 0.1], C => 0.001, Slide => False),
         (W => [1.0, 0.0, 0.0], P => [0.0, 0.25, 0.1], C => 0.001, Slide => False)];
      Eye_At_Place : constant Vec3 := [0.15, -0.55, 0.75];
      Target       : constant Vec3 := [0.0, 0.2, 0.08];
      Forward      : constant Vec3 := Unit (Minus (Target, Eye_At_Place));
      Right        : constant Vec3 := Unit (Cross (Forward, [0.0, 0.0, 1.0]));
      Down         : constant Vec3 := Cross (Forward, Right);
      Truth        : constant Rigid :=
        (Rotation    => [[Right (1), Down (1), Forward (1)], [Right (2), Down (2), Forward (2)],
                         [Right (3), Down (3), Forward (3)]],
         Translation => Eye_At_Place);
      Sigma  : constant Real := 0.04;
      G      : Generator := (State => 7);
      Poses  : constant := 5;
      Points : constant := 60;
      Views  : aliased Flow.Pose_Array := [1 .. Poses => null];
      Result : Flow.Report (Arms => 1, Count => Flow.Terms_For (1));
      --  Where the camera sees the point of link K (at the reference) with the joints at Readings.
      procedure Project (Point : Vec3; K : Positive; Readings : Real_Array; U, V : out Real; Ahead : out Boolean) is
         Moved : constant Rigid := KFit.Eye_At (Joints (1 .. K), [for I in 1 .. K => I], Readings (1 .. K));
         C     : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (Moved * Point, Truth.Translation));
      begin
         Ahead := C (3) > 0.0;
         U := (if Ahead then Focal * C (1) / C (3) + Real (Width) / 2.0 else 0.0);
         V := (if Ahead then Focal * Aspect * C (2) / C (3) + Real (Height) / 2.0 else 0.0);
      end Project;
   begin
      for P in 1 .. Poses loop
         declare
            Readings : Real_Array (1 .. 3);
            View     : constant Flow.Pose_Access := new Flow.Pose_View (3, Points);
         begin
            for J in 1 .. 3 loop
               Readings (J) := (if P = 1 then 0.0 else 300.0 * (2.0 * Uniform (G) - 1.0));
            end loop;
            View.Readings := Readings;
            View.Back := [others => 0.0];
            for I in 1 .. Points loop
               declare
                  K     : constant Positive := (if I mod 2 = 0 then 2 else 3);
                  Along : constant Real := Uniform (G);
                  Point : constant Vec3 :=
                    (if K = 2
                     then [0.03 * (2.0 * Uniform (G) - 1.0), 0.25 * Along, 0.1 + 0.03 * (2.0 * Uniform (G) - 1.0)]
                     else [0.03 * (2.0 * Uniform (G) - 1.0), 0.25 + 0.2 * Along,
                           0.1 + 0.03 * (2.0 * Uniform (G) - 1.0)]);
                  Angle : constant Real := 2.0 * Ada.Numerics.Pi * Uniform (G);
                  U, V  : Real;
                  Ahead : Boolean;
               begin
                  Project (Point, K, Readings, U, V, Ahead);
                  Check (Ahead, "a point of the synthetic arm is behind its eye");
                  View.U (I) := U;
                  View.V (I) := V;
                  View.Gu (I) := 30.0 * Cos (Angle);
                  View.Gv (I) := 30.0 * Sin (Angle);
                  View.Link (I) := K;
                  for J in 1 .. 3 loop
                     View.Sigma (I, J) := Sigma;
                     if J <= K then
                        declare
                           Step     : constant Real := 1.0E-3;
                           Up, Back : Real_Array := Readings;
                           U1, V1, U2, V2 : Real;
                           A1, A2   : Boolean;
                        begin
                           Up (J) := Readings (J) + Step;
                           Back (J) := Readings (J) - Step;
                           Project (Point, K, Up, U1, V1, A1);
                           Project (Point, K, Back, U2, V2, A2);
                           View.Rate (I, J) := -(View.Gu (I) * (U1 - U2) + View.Gv (I) * (V1 - V2)) / (2.0 * Step)
                             + Sigma * Gaussian (G);
                        end;
                     else
                        View.Rate (I, J) := Sigma * Gaussian (G);
                     end if;
                  end loop;
               end;
            end loop;
            Views (P) := View;
         end;
      end loop;
      Flow.Fit_Eye ([1 => (Poses => Views'Unchecked_Access, Joints => Joints'Unchecked_Access, Unit_Sigma => null,
                           Profiled => Profiled)], Width, Height, Result);
      Check (Result.Determined, "the synthetic arm's eye is not told: " & Ada.Strings.Unbounded.To_String (Result.Why));
      if Result.Determined then
         declare
            Off_F  : constant Real := Ada.Numerics.Long_Elementary_Functions.Log (Result.Lens.Fx / Focal);
            Off_C  : constant Vec3 := Minus (Result.Poses (1).Translation, Truth.Translation);
            O      : constant Natural := Flow.Terms_For (1) - 6;   --  the lens's terms, before the arm's
            Turned : constant Vec3 :=
              Driver.Numerics.Log (Times (Transposed (Truth.Rotation), Result.Poses (1).Rotation));
         begin
            Check (abs Off_F <= Driver.Conventions.Z * Sqrt (Result.Covariance (1, 1)),
                   "the focal length is off its sigma:" & Real'Image (Result.Lens.Fx));
            Check (abs Ada.Numerics.Long_Elementary_Functions.Log (Result.Lens.Fy / Result.Lens.Fx / Aspect)
                   <= Driver.Conventions.Z * Sqrt (Result.Covariance (4, 4)),
                   "the aspect is off its sigma:" & Real'Image (Result.Lens.Fy / Result.Lens.Fx));
            for A in 1 .. 3 loop
               Check (abs Off_C (A) <= Driver.Conventions.Z * Sqrt (Result.Covariance (O + 3 + A, O + 3 + A)),
                      "the centre is off its sigma along axis" & A'Image & ":" & Real'Image (Off_C (A)));
               Check (abs Turned (A) <= Driver.Conventions.Z * Sqrt (Result.Covariance (O + A, O + A)),
                      "the turn is off its sigma about axis" & A'Image & ":" & Real'Image (Turned (A)));
            end loop;
         end;
      end if;
   end Flow_Eye_On_A_Synthetic_Arm_With;

   procedure Flow_Eye_On_A_Synthetic_Arm is
   begin
      Flow_Eye_On_A_Synthetic_Arm_With (Profiled => False);
   end Flow_Eye_On_A_Synthetic_Arm;

   --  The same with every pixel's link the model's (Arm_Flow.Profiled): on an arm whose rates are its links' own, the
   --  links the model chooses are those, and the eye comes back as well.
   procedure Flow_Eye_Profiled_On_A_Synthetic_Arm is
   begin
      Flow_Eye_On_A_Synthetic_Arm_With (Profiled => True);
   end Flow_Eye_Profiled_On_A_Synthetic_Arm;

   --  A fixed eye placed again by the table two arms' own eyes see (Kinematics.Flow_Eye.Refine_By_Scene): the arm of
   --  robot.flow.eye twice, side by side above a table 0.25 below their bases, each flow as that test makes it but
   --  with 200 points a pose spread 8 cm about the links, a sigma of 0.02 and turns of 0.4 rad either way (with that
   --  test's flows the eye's distance and focal length grew together without end along the table's open direction); the
   --  last link of each carries an eye looking down at the table, its pictures at its flow's poses and at as many
   --  more; the table's texture blobs a few centimetres wide, the fixed eye's picture of it with no arm in the way. The
   --  eye placed again is told, within Z of its sigmas in every term, and the table's points it puts in its picture
   --  lie nearer the true eye's places than the flows' eye put them.
   procedure Scene_Places_A_Fixed_Eye is
      package Flow renames Driver.Robot.Kinematics.Flow_Eye;
      package KFit renames Driver.Robot.Kinematics.Fit;
      Width  : constant := 320;
      Height : constant := 240;
      Focal  : constant Real := 150.0;
      --  The pixels not square, a camera's little: the table's places tell it from square pixels (checked against
      --  them instead, the aspect's check fails, 10-21).
      Aspect : constant Real := 0.996;
      Wrist_W : constant := 160;
      Wrist_H : constant := 120;
      Wrist_Lens : constant KFit.Lens := (Fx | Fy => 120.0, Cx => 80.0, Cy => 60.0, K1 | K2 => 0.0);
      Table_Z : constant Real := -0.25;
      Arms    : constant := 2;
      Bases   : constant array (1 .. Arms) of Vec3 := [[-0.15, 0.0, 0.0], [0.15, 0.0, 0.0]];
      type Joints_Access is access KFit.Joint_Array;
      Joints  : array (1 .. Arms) of Joints_Access;
      Eye_At_Place : constant Vec3 := [0.15, -0.55, 0.75];
      Target       : constant Vec3 := [0.0, 0.25, -0.1];
      Forward      : constant Vec3 := Unit (Minus (Target, Eye_At_Place));
      Right        : constant Vec3 := Unit (Cross (Forward, [0.0, 0.0, 1.0]));
      Down         : constant Vec3 := Cross (Forward, Right);
      Truth        : constant Rigid :=
        (Rotation    => [[Right (1), Down (1), Forward (1)], [Right (2), Down (2), Forward (2)],
                         [Right (3), Down (3), Forward (3)]],
         Translation => Eye_At_Place);
      --  The wrist eye at the reference, at the third link's far end: x along the world's x, looking straight down.
      Wrist_Turn : constant Mat3 := [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
      Sigma  : constant Real := 0.02;
      G      : Generator := (State => 11);
      Poses  : constant := 6;
      Points : constant := 200;
      type Views_Access is access Flow.Pose_Array;
      Views  : array (1 .. Arms) of Views_Access;
      Result : Flow.Report (Arms => Arms, Count => Flow.Terms_For (Arms));
      --  The table's texture: blobs of 2 to 5 cm, each of its own brightness, over every place either eye sees.
      Blobs  : constant := 900;
      type Blob is record
         X, Y, S, A : Real;
      end record;
      Bank : array (1 .. Blobs) of Blob;
      function Texture_At (X, Y : Real) return Real is
         Sum : Real := 128.0;
      begin
         for B of Bank loop
            declare
               R2 : constant Real := (X - B.X) ** 2 + (Y - B.Y) ** 2;
            begin
               if R2 < 16.0 * B.S ** 2 then
                  Sum := Sum + B.A * Exp (-R2 / (2.0 * B.S ** 2));
               end if;
            end;
         end loop;
         return Sum;
      end Texture_At;
      --  An eye's picture of the table (luma, row after row): its pose in the world and its lens.
      function Picture_Of (Pose : Rigid; L : KFit.Lens; W, H : Positive) return Flow.Luma_Access is
         Luma : constant Flow.Luma_Access := new Real_Array (1 .. W * H);
      begin
         for V in 0 .. H - 1 loop
            for U in 0 .. W - 1 loop
               declare
                  D : constant Vec3 := Times (Pose.Rotation, KFit.Ray (L, Real (U) + 0.5, Real (V) + 0.5));
               begin
                  Luma (V * W + U + 1) :=
                    (if D (3) < 0.0
                     then Texture_At (Pose.Translation (1) + (Table_Z - Pose.Translation (3)) / D (3) * D (1),
                                      Pose.Translation (2) + (Table_Z - Pose.Translation (3)) / D (3) * D (2))
                     else 0.0);
               end;
            end loop;
         end loop;
         return Luma;
      end Picture_Of;
      --  Where the camera sees the point of arm A's link K (at the reference) with the joints at Readings.
      procedure Project
        (A : Positive; Point : Vec3; K : Positive; Readings : Real_Array; U, V : out Real; Ahead : out Boolean)
      is
         Moved : constant Rigid := KFit.Eye_At (Joints (A) (1 .. K), [for I in 1 .. K => I], Readings (1 .. K));
         C     : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (Moved * Point, Truth.Translation));
      begin
         Ahead := C (3) > 0.0;
         U := (if Ahead then Focal * C (1) / C (3) + Real (Width) / 2.0 else 0.0);
         V := (if Ahead then Focal * Aspect * C (2) / C (3) + Real (Height) / 2.0 else 0.0);
      end Project;
      type Pictures_Access is access Flow.Wrist_Picture_Array;
      Pictures : array (1 .. Arms) of Pictures_Access;
      Worlds   : Flow.Arm_World_Array (1 .. Arms);
      In_Arms  : Flow.Arm_Flow_Array (1 .. Arms);
      --  Every picture's readings, and where the wrist eye stands at the reference.
      Readings_At : array (1 .. Arms, 1 .. 2 * Poses) of Real_Array (1 .. 3);
      Rest_At     : array (1 .. Arms) of Rigid;
      Fixed    : Flow.Luma_Access;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Flow.Luma_Access);
      --  How far an eye of those terms puts the table's points (a grid 4 cm apart the true eye sees) from the true
      --  eye's places, at most.
      function Table_Off (R : Flow.Report) return Real is
         Worst : Real := 0.0;
      begin
         for I in -10 .. 10 loop
            for J in -5 .. 20 loop
               declare
                  X  : constant Vec3 := [0.04 * Real (I), 0.04 * Real (J), Table_Z];
                  Ct : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (X, Truth.Translation));
                  Cf : constant Vec3 :=
                    Transposed_Times (R.Poses (1).Rotation, Minus (X, R.Poses (1).Translation));
               begin
                  if Ct (3) > 0.0 and then Cf (3) > 0.0 then
                     declare
                        Tu : constant Real := Focal * Ct (1) / Ct (3) + Real (Width) / 2.0;
                        Tv : constant Real := Focal * Aspect * Ct (2) / Ct (3) + Real (Height) / 2.0;
                     begin
                        if Tu in 0.0 .. Real (Width) and then Tv in 0.0 .. Real (Height) then
                           Worst := Real'Max
                             (Worst, Sqrt ((R.Lens.Cx + R.Lens.Fx * Cf (1) / Cf (3) - Tu) ** 2
                                           + (R.Lens.Cy + R.Lens.Fy * Cf (2) / Cf (3) - Tv) ** 2));
                        end if;
                     end;
                  end if;
               end;
            end loop;
         end loop;
         return Worst;
      end Table_Off;
   begin
      for B of Bank loop
         B := (X => -0.8 + 1.6 * Uniform (G), Y => -0.5 + 1.6 * Uniform (G), S => 0.02 + 0.03 * Uniform (G),
               A => 70.0 * (2.0 * Uniform (G) - 1.0));
      end loop;
      for A in 1 .. Arms loop
         Joints (A) := new KFit.Joint_Array'
           [(W => [0.0, 0.0, 1.0], P => Bases (A), C => 0.001, Slide => False),
            (W => [1.0, 0.0, 0.0], P => Plus (Bases (A), [0.0, 0.0, 0.1]), C => 0.001, Slide => False),
            (W => [1.0, 0.0, 0.0], P => Plus (Bases (A), [0.0, 0.25, 0.1]), C => 0.001, Slide => False)];
         Views (A) := new Flow.Pose_Array (1 .. Poses);
         Pictures (A) := new Flow.Wrist_Picture_Array (1 .. 2 * Poses);
         for P in 1 .. 2 * Poses loop
            declare
               Readings : Real_Array (1 .. 3);
            begin
               for J in 1 .. 3 loop
                  Readings (J) := (if P = 1 then 0.0 else 400.0 * (2.0 * Uniform (G) - 1.0));
               end loop;
               --  The wrist eye's picture at the pose, and where it stood.
               declare
                  Moved : constant Rigid := KFit.Eye_At (Joints (A).all, [1, 2, 3], Readings);
                  Pose  : constant Rigid :=
                    Moved * Rigid'(Rotation => Wrist_Turn, Translation => Plus (Bases (A), [0.0, 0.45, 0.12]));
               begin
                  Readings_At (A, P) := Readings;
                  Rest_At (A) := (Rotation => Wrist_Turn, Translation => Plus (Bases (A), [0.0, 0.45, 0.12]));
                  Pictures (A) (P) := (Luma => Picture_Of (Pose, Wrist_Lens, Wrist_W, Wrist_H), Width => Wrist_W,
                                       Height => Wrist_H, Eye => Pose, World => null);
               end;
               --  The flow at the first poses, as robot.flow.eye makes it.
               if P <= Poses then
                  declare
                     View : constant Flow.Pose_Access := new Flow.Pose_View (3, Points);
                  begin
                     View.Readings := Readings;
                     View.Back := [others => 0.0];
                     for I in 1 .. Points loop
                        declare
                           K     : constant Positive := (if I mod 2 = 0 then 2 else 3);
                           Along : constant Real := Uniform (G);
                           Point : constant Vec3 :=
                             Plus (Bases (A),
                                   (if K = 2
                                    then [0.08 * (2.0 * Uniform (G) - 1.0), 0.25 * Along,
                                          0.1 + 0.08 * (2.0 * Uniform (G) - 1.0)]
                                    else [0.08 * (2.0 * Uniform (G) - 1.0), 0.25 + 0.2 * Along,
                                          0.1 + 0.08 * (2.0 * Uniform (G) - 1.0)]));
                           Angle : constant Real := 2.0 * Ada.Numerics.Pi * Uniform (G);
                           U, V  : Real;
                           Ahead : Boolean;
                        begin
                           Project (A, Point, K, Readings, U, V, Ahead);
                           Check (Ahead, "a point of a synthetic arm is behind the fixed eye");
                           View.U (I) := U;
                           View.V (I) := V;
                           View.Gu (I) := 30.0 * Cos (Angle);
                           View.Gv (I) := 30.0 * Sin (Angle);
                           View.Link (I) := K;
                           for J in 1 .. 3 loop
                              View.Sigma (I, J) := Sigma;
                              if J <= K then
                                 declare
                                    Step     : constant Real := 1.0E-3;
                                    Up, Back : Real_Array := Readings;
                                    U1, V1, U2, V2 : Real;
                                    A1, A2   : Boolean;
                                 begin
                                    Up (J) := Readings (J) + Step;
                                    Back (J) := Readings (J) - Step;
                                    Project (A, Point, K, Up, U1, V1, A1);
                                    Project (A, Point, K, Back, U2, V2, A2);
                                    View.Rate (I, J) :=
                                      -(View.Gu (I) * (U1 - U2) + View.Gv (I) * (V1 - V2)) / (2.0 * Step)
                                      + Sigma * Gaussian (G);
                                 end;
                              else
                                 View.Rate (I, J) := Sigma * Gaussian (G);
                              end if;
                           end loop;
                        end;
                     end loop;
                     Views (A) (P) := View;
                  end;
               end if;
            end;
         end loop;
         In_Arms (A) := (Poses => Views (A).all'Unchecked_Access, Joints => Joints (A).all'Unchecked_Access,
                         Unit_Sigma => null, Profiled => False);
         Worlds (A) := (Pictures => Pictures (A).all'Unchecked_Access, Lens => Wrist_Lens,
                        Table    => (Centre => [0.0, 0.25, Table_Z], Normal => [0.0, 0.0, 1.0],
                                     Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
                                     Offset_Sigma => 1.0E-4, Tilt_11 => 1.0E-8, Tilt_12 => 0.0, Tilt_22 => 1.0E-8,
                                     Points => 1000, Scatter => 1.0),
                        Variants => null);
      end loop;
      Fixed := Picture_Of (Truth, (Fx => Focal, Fy => Focal * Aspect, Cx => Real (Width) / 2.0,
                                   Cy => Real (Height) / 2.0, K1 | K2 => 0.0), Width, Height);
      Flow.Fit_Eye (In_Arms, Width, Height, Result);
      declare
         Flows_Off : constant Real := Table_Off (Result);
      begin
         Flow.Refine_By_Scene (In_Arms, Worlds, Fixed.all, Width, Height, Result);
         Check (Result.Determined, "the eye placed again is not told: " & Ada.Strings.Unbounded.To_String (Result.Why));
         --  A table's picture is a homography, eight terms of the eye's ten, every further arm's six told by its own
         --  places and the one plane: the focal length against the distance and the principal point against the tilt
         --  are left, and the places tell every other direction of the flows' terms (Report.Places_Told).
         Check (Result.Places_Told = Flow.Terms_For (Arms) - 2, "the places tell" & Result.Places_Told'Image
                & " of the flows' terms' directions, not all but the plane's two");
         Check (Table_Off (Result) < Flows_Off, "the table's points lie no nearer the true eye's places than the flows' "
                & "eye put them:" & Real'Image (Table_Off (Result)) & " px against" & Real'Image (Flows_Off));
      end;
      if Result.Determined then
         Check (abs Ada.Numerics.Long_Elementary_Functions.Log (Result.Lens.Fx / Focal)
                <= Driver.Conventions.Z * Sqrt (Result.Covariance (1, 1)),
                "the focal length is off its sigma:" & Real'Image (Result.Lens.Fx));
         Check (abs (Result.Lens.Cx - Real (Width) / 2.0) <= Driver.Conventions.Z * Sqrt (Result.Covariance (2, 2))
                and then abs (Result.Lens.Cy - Real (Height) / 2.0)
                         <= Driver.Conventions.Z * Sqrt (Result.Covariance (3, 3)),
                "the principal point is off its sigma:" & Real'Image (Result.Lens.Cx) & Real'Image (Result.Lens.Cy));
         Check (abs Ada.Numerics.Long_Elementary_Functions.Log (Result.Lens.Fy / Result.Lens.Fx / Aspect)
                <= Driver.Conventions.Z * Sqrt (Result.Covariance (4, 4)),
                "the aspect is off its sigma:" & Real'Image (Result.Lens.Fy / Result.Lens.Fx));
         for A in 1 .. Arms loop
            declare
               Off_C  : constant Vec3 := Minus (Result.Poses (A).Translation, Truth.Translation);
               Turned : constant Vec3 :=
                 Driver.Numerics.Log (Times (Transposed (Truth.Rotation), Result.Poses (A).Rotation));
               O      : constant Natural := Flow.Terms_For (A) - 6;   --  the terms before arm A's
            begin
               for X in 1 .. 3 loop
                  Check (abs Off_C (X) <= Driver.Conventions.Z * Sqrt (Result.Covariance (O + 3 + X, O + 3 + X)),
                         "arm" & A'Image & "'s centre is off its sigma along axis" & X'Image & ":"
                         & Real'Image (Off_C (X)));
                  Check (abs Turned (X) <= Driver.Conventions.Z * Sqrt (Result.Covariance (O + X, O + X)),
                         "arm" & A'Image & "'s turn is off its sigma about axis" & X'Image & ":"
                         & Real'Image (Turned (X)));
               end loop;
            end;
         end loop;
      end if;
      --  The same scene placed by the arms' joints off the truth as a fit would hand them (each axis tilted along a
      --  direction across it, its point moved along the other and its unit scaled, the sense turning joint by joint),
      --  the wrist eyes' places by those joints, and their sigmas handed over as each arm's variants
      --  (Arm_World.Variants, every term of every joint a sigma either way): the eye's spread grows with them, and
      --  holds the arms' joints where the true eye sees them.
      declare
         Off_Tilt : constant Real := 0.004;   --  radians
         Off_Move : constant Real := 0.002;   --  metres
         Off_Unit : constant Real := 0.005;   --  of the unit
         type Variants_Access is access Flow.Arm_Variant_Array;
         type Rigids_Access is access Flow.Rigid_Array;
         Off      : array (1 .. Arms) of Joints_Access;
         Shown    : array (1 .. Arms) of Pictures_Access;
         Variants : array (1 .. Arms) of Variants_Access;
         Arms_Off : Flow.Arm_Flow_Array (1 .. Arms);
         Worlds_Off : Flow.Arm_World_Array (1 .. Arms);
         Placed   : Flow.Report (Arms => Arms, Count => Flow.Terms_For (Arms));
         --  Joints J moved by By along term T of joint K (KFit.Joint_Term's: a tilt or a move along a direction across
         --  its axis, or its unit).
         procedure Move (J : in out KFit.Joint_Array; K : Positive; T : KFit.Joint_Term; By : Real) is
            E1, E2 : Vec3;
         begin
            KFit.Across (J (K).W, E1, E2);
            case T is
               when KFit.Tilt_1  => J (K).W := Unit (Plus (J (K).W, Scaled (By, E1)));
               when KFit.Tilt_2  => J (K).W := Unit (Plus (J (K).W, Scaled (By, E2)));
               when KFit.Point_1 => J (K).P := Plus (J (K).P, Scaled (By, E1));
               when KFit.Point_2 => J (K).P := Plus (J (K).P, Scaled (By, E2));
               when KFit.Scale   => J (K).C := J (K).C * (1.0 + By);
            end case;
         end Move;
         function Sigma_Of (T : KFit.Joint_Term) return Real is
           (case T is when KFit.Tilt_1 | KFit.Tilt_2 => Off_Tilt, when KFit.Point_1 | KFit.Point_2 => Off_Move,
                      when KFit.Scale => Off_Unit);
         --  The wrist eye's places by joints J, every picture's.
         function Eyes_By (A : Positive; J : KFit.Joint_Array) return Rigids_Access is
            E : constant Rigids_Access := new Flow.Rigid_Array (1 .. 2 * Poses);
         begin
            for P in E'Range loop
               E (P) := KFit.Eye_At (J, [1, 2, 3], Readings_At (A, P)) * Rest_At (A);
            end loop;
            return E;
         end Eyes_By;
         --  How far the eye of R puts the arms' joints as given (at every flow pose) from where the true eye sees the
         --  true joints, at most.
         function Joints_Off (R : Flow.Report) return Real is
            Worst : Real := 0.0;
         begin
            for A in 1 .. Arms loop
               for P in 1 .. Poses loop
                  for K in 1 .. 3 loop
                     declare
                        Given : constant Vec3 :=
                          KFit.Eye_At (Off (A) (1 .. K - 1), [for I in 1 .. K - 1 => I],
                                       Readings_At (A, P) (1 .. K - 1))
                          * Off (A) (K).P;
                        True_X : constant Vec3 :=
                          KFit.Eye_At (Joints (A) (1 .. K - 1), [for I in 1 .. K - 1 => I],
                                       Readings_At (A, P) (1 .. K - 1)) * Joints (A) (K).P;
                        Ct : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (True_X, Truth.Translation));
                        Cf : constant Vec3 :=
                          Transposed_Times (R.Poses (A).Rotation, Minus (Given, R.Poses (A).Translation));
                     begin
                        if Ct (3) > 0.0 and then Cf (3) > 0.0 then
                           Worst := Real'Max
                             (Worst, Sqrt ((R.Lens.Cx + R.Lens.Fx * Cf (1) / Cf (3)
                                            - (Focal * Ct (1) / Ct (3) + Real (Width) / 2.0)) ** 2
                                           + (R.Lens.Cy + R.Lens.Fy * Cf (2) / Cf (3)
                                              - (Focal * Ct (2) / Ct (3) + Real (Height) / 2.0)) ** 2));
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
            return Worst;
         end Joints_Off;
      begin
         for A in 1 .. Arms loop
            Off (A) := new KFit.Joint_Array'(Joints (A).all);
            for K in 1 .. 3 loop
               declare
                  Sense : constant Real := (if K mod 2 = 0 then -1.0 else 1.0);
               begin
                  Move (Off (A).all, K, KFit.Tilt_1, Sense * Off_Tilt);
                  Move (Off (A).all, K, KFit.Point_2, Sense * Off_Move);
                  Move (Off (A).all, K, KFit.Scale, Sense * Off_Unit);
               end;
            end loop;
            Shown (A) := new Flow.Wrist_Picture_Array'(Pictures (A).all);
            declare
               E : constant Rigids_Access := Eyes_By (A, Off (A).all);
            begin
               for P in Shown (A)'Range loop
                  Shown (A) (P).Eye := E (P);
               end loop;
            end;
            Variants (A) := new Flow.Arm_Variant_Array (1 .. 2 * KFit.Joint_Terms * 3);
            for K in 1 .. 3 loop
               for T in KFit.Joint_Term loop
                  for E in 1 .. 2 loop
                     declare
                        J : constant Joints_Access := new KFit.Joint_Array'(Off (A).all);
                     begin
                        Move (J.all, K, T, (if E = 1 then -1.0 else 1.0) * Sigma_Of (T));
                        Variants (A) (2 * (KFit.Joint_Terms * (K - 1) + KFit.Joint_Term'Pos (T)) + E) :=
                          (Joints => J.all'Unchecked_Access, Eyes => Eyes_By (A, J.all).all'Unchecked_Access,
                           Lens => Wrist_Lens, Table => Worlds (A).Table);
                     end;
                  end loop;
               end loop;
            end loop;
            Arms_Off (A) := (Poses => Views (A).all'Unchecked_Access, Joints => Off (A).all'Unchecked_Access,
                             Unit_Sigma => null, Profiled => False);
            Worlds_Off (A) := (Pictures => Shown (A).all'Unchecked_Access, Lens => Wrist_Lens,
                               Table => Worlds (A).Table, Variants => Variants (A).all'Unchecked_Access);
         end loop;
         Flow.Fit_Eye (Arms_Off, Width, Height, Placed);
         Flow.Refine_By_Scene (Arms_Off, Worlds_Off, Fixed.all, Width, Height, Placed);
         Check (Placed.Spread > Result.Spread, "the arms' fits' variants leave the eye's spread no wider:"
                & Real'Image (Placed.Spread) & " px against" & Real'Image (Result.Spread));
         Check (Placed.Spread < Real'Last
                and then Joints_Off (Placed) <= Driver.Conventions.Z * Placed.Spread,
                "the arms' joints off the truth leave them farther from the true eye's places than Z spreads:"
                & Real'Image (Joints_Off (Placed)) & " px against a spread of" & Real'Image (Placed.Spread));
      end;
      for A in 1 .. Arms loop
         for P of Pictures (A).all loop
            declare
               L : Flow.Luma_Access := P.Luma;
            begin
               Free (L);
            end;
         end loop;
      end loop;
      Free (Fixed);
   end Scene_Places_A_Fixed_Eye;

   --  A synthetic arm (Flow_Eye_On_A_Synthetic_Arm's) with exact rates: at the true eye each joint's predicted motion
   --  (Flow_Eye.Joint_Motions) is the median over the pixels on the links it moves of their projected motions a
   --  reading unit, to a central difference's truncation.
   procedure Flow_Motions_On_A_Synthetic_Arm is
      package Flow renames Driver.Robot.Kinematics.Flow_Eye;
      package KFit renames Driver.Robot.Kinematics.Fit;
      Width  : constant := 320;
      Height : constant := 240;
      Focal  : constant Real := 150.0;
      Joints : aliased constant KFit.Joint_Array :=
        [(W => [0.0, 0.0, 1.0], P => [0.0, 0.0, 0.0], C => 0.001, Slide => False),
         (W => [1.0, 0.0, 0.0], P => [0.0, 0.0, 0.1], C => 0.001, Slide => False),
         (W => [1.0, 0.0, 0.0], P => [0.0, 0.25, 0.1], C => 0.001, Slide => False)];
      Eye_At_Place : constant Vec3 := [0.15, -0.55, 0.75];
      Target       : constant Vec3 := [0.0, 0.2, 0.08];
      Forward      : constant Vec3 := Unit (Minus (Target, Eye_At_Place));
      Right        : constant Vec3 := Unit (Cross (Forward, [0.0, 0.0, 1.0]));
      Down         : constant Vec3 := Cross (Forward, Right);
      Truth        : constant Rigid :=
        (Rotation    => [[Right (1), Down (1), Forward (1)], [Right (2), Down (2), Forward (2)],
                         [Right (3), Down (3), Forward (3)]],
         Translation => Eye_At_Place);
      Sigma  : constant Real := 0.04;
      G      : Generator := (State => 11);
      Points : constant := 60;
      Step   : constant Real := 1.0E-3;
      View   : constant Flow.Pose_Access := new Flow.Pose_View (3, Points);
      Views  : constant Flow.Pose_Array := [1 => View];
      Moves  : array (1 .. 3, 1 .. Points) of Real := [others => [others => 0.0]];
      Counts : array (1 .. 3) of Natural := [others => 0];
      procedure Project (Point : Vec3; K : Positive; Readings : Real_Array; U, V : out Real) is
         Moved : constant Rigid := KFit.Eye_At (Joints (1 .. K), [for I in 1 .. K => I], Readings (1 .. K));
         C     : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (Moved * Point, Truth.Translation));
      begin
         U := Focal * C (1) / C (3) + Real (Width) / 2.0;
         V := Focal * C (2) / C (3) + Real (Height) / 2.0;
      end Project;
   begin
      View.Readings := [0.0, 0.0, 0.0];
      View.Steps := [1.0, 1.0, 1.0];
      View.Back := [others => 0.0];
      for I in 1 .. Points loop
         declare
            K     : constant Positive := (if I mod 2 = 0 then 2 else 3);
            Along : constant Real := Uniform (G);
            Point : constant Vec3 :=
              (if K = 2
               then [0.03 * (2.0 * Uniform (G) - 1.0), 0.25 * Along, 0.1 + 0.03 * (2.0 * Uniform (G) - 1.0)]
               else [0.03 * (2.0 * Uniform (G) - 1.0), 0.25 + 0.2 * Along, 0.1 + 0.03 * (2.0 * Uniform (G) - 1.0)]);
            Angle : constant Real := 2.0 * Ada.Numerics.Pi * Uniform (G);
            U, V  : Real;
         begin
            Project (Point, K, View.Readings, U, V);
            View.U (I) := U;
            View.V (I) := V;
            View.Gu (I) := 30.0 * Cos (Angle);
            View.Gv (I) := 30.0 * Sin (Angle);
            View.Link (I) := K;
            for J in 1 .. 3 loop
               View.Sigma (I, J) := Sigma;
               View.Rate (I, J) := 0.0;
               if J <= K then
                  declare
                     Up, Back : Real_Array := View.Readings;
                     U1, V1, U2, V2 : Real;
                  begin
                     Up (J) := Step;
                     Back (J) := -Step;
                     Project (Point, K, Up, U1, V1);
                     Project (Point, K, Back, U2, V2);
                     View.Rate (I, J) := -(View.Gu (I) * (U1 - U2) + View.Gv (I) * (V1 - V2)) / (2.0 * Step);
                     Counts (J) := Counts (J) + 1;
                     Moves (J, Counts (J)) := Sqrt ((U1 - U2) ** 2 + (V1 - V2) ** 2) / (2.0 * Step);
                  end;
               end if;
            end loop;
         end;
      end loop;
      declare
         Motions : constant Real_Array :=
           Flow.Joint_Motions (Views, Joints, Width, Height, Truth, (Fx => Focal, Fy => Focal, Cx => Real (Width) / 2.0,
                               Cy => Real (Height) / 2.0, K1 | K2 => 0.0), 1.0, 0.0);
      begin
         for J in 1 .. 3 loop
            declare
               Truths : Real_Array (1 .. Counts (J));
            begin
               for K in 1 .. Counts (J) loop
                  Truths (K) := Moves (J, K);
               end loop;
               declare
                  Median : constant Real := Driver.Stats.Median_In_Place (Truths);
               begin
                  Check (abs (Motions (Motions'First + J - 1) - Median) <= Sqrt (Real'Epsilon) * Median,
                         "joint" & J'Image & "'s predicted motion" & Real'Image (Motions (Motions'First + J - 1))
                         & " is not its links' pixels'" & Real'Image (Median));
               end;
            end;
         end loop;
      end;
   end Flow_Motions_On_A_Synthetic_Arm;

   procedure Register is
   begin
      Driver.Tests.Register ("robot.flow.eye", "a fixed eye placed by a synthetic arm's edge flow is not told or comes "
                             & "back off its own sigmas", Flow_Eye_On_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.flow.eye.profiled", "a fixed eye placed by a synthetic arm's edge flow with each "
                             & "pixel's link the model's is not told or comes back off its own sigmas",
                             Flow_Eye_Profiled_On_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.flow.motions", "a joint's predicted motion at a synthetic arm's true eye is not its "
                             & "links' pixels' projected motion", Flow_Motions_On_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.flow.scene", "a fixed eye placed again by the table two arms' own eyes see is "
                             & "not told, comes back off its own sigmas, or puts the table's points no nearer their "
                             & "places than the flows' eye; placed by the arms' joints off the truth and their sigmas, "
                             & "its spread is no wider or leaves the arms' joints farther than Z spreads from the true "
                             & "eye's places", Scene_Places_A_Fixed_Eye'Access);
      Driver.Tests.Register ("robot.kinematics.scales", "the floor and gain of the scales residuals are judged at are "
                             & "read beside what their sightings state, whatever part of it the residuals bear out, so "
                             & "the sightings that state much are no outliers by those that state little", Scales_Beside_Stated_Sigmas'Access);
      Driver.Robot.Kinematics.Errors.Tests.Register;
      Driver.Robot.Kinematics.Base.Tests.Register;
      Driver.Robot.Surfaces.Tests.Register;
      Driver.Tests.Register ("robot.estimate.task", "an estimate over a long history fails in a task with the default "
                             & "stack, as the decider's does", Estimate_In_A_Task'Access);
      Driver.Tests.Register ("robot.probe.limits", "a channel at its limit one way is asked ever further that way though "
                             & "it answered the other way (its noise lost or not), a deadband is not found however much "
                             & "wider than what the rest of the body needed, a channel that answers neither way is called "
                             & "dead before every level was asked each way (or when a visible step far below its noise credits it with a following), or one whose noise is unmeasured is called dead",
                             Probe_Limits_And_Deadbands'Access);
      Driver.Tests.Register ("robot.probe.droop", "a probe calls a joint at its end when the fraction of each offset "
                             & "it delivers shrinks, though it still follows", Probe_A_Drooping_Joint'Access);
      Driver.Tests.Register ("robot.probe.delayed", "a probe of a body whose readings show a command a beat later, "
                             & "before any delay is measured, reads each level a beat after sending it: the joint's "
                             & "end comes a level late, the first level is cut short before its answer, and levels "
                             & "are sent into the answer of the one before", Probe_A_Delayed_Joint'Access);
      Driver.Tests.Register ("robot.probe.apart", "a probe that asks for the estimates as it begins loses what an eye saw "
                             & "when they come back beats later", Probe_Confirms_Through_Apart_Estimates'Access);
      Driver.Tests.Register ("robot.probe.lost", "a probe whose channel's noise was lost measures it again before "
                             & "it reads anything and takes its first step from it",
                             Probe_Reads_After_It_Measures'Access);
      Driver.Tests.Register ("robot.probe.lockin", "a channel probed on its own that an eye sees is confirmed by a "
                             & "run of moves back and forth instead of by the lock-in's regression after one estimate",
                             Probe_Confirms_By_The_Lockin'Access);
      Driver.Tests.Register ("robot.reach", "the readings that put an arm's eye at a pose are not found",
                             Reach_A_Pose'Access);
      Driver.Tests.Register ("robot.reach.turn", "a reach ends a joint that turns a whole period of its fit from "
                             & "where the arm is, the same pose as a reading that is not the nearest",
                             Reach_The_Nearest_Turn'Access);
      Driver.Tests.Register ("robot.plan.stray", "a joints' straight line that is on the straight path at its middle and off it at its "
                             & "quarters is taken to keep to it", Stray_Is_Seen_Off_The_Middle'Access);
      Driver.Tests.Register ("robot.plan.bow", "a plan that strays off the straight path by more than its clearance does not "
                             & "say so", A_Bow_That_Is_Left_Is_Said'Access);
      Driver.Tests.Register ("robot.plan.straight", "a reach given a clearance is not cut where the arm's joints would "
                             & "take the tool farther off the straight path than that, or one given none is cut",
                             Plan_Stays_Within_Its_Clearance'Access);
      Driver.Tests.Register ("robot.still.reloaded", "a reloaded body creeping uncommanded below its visible step is "
                             & "not still, or Settle or Hold_For_Keyframe does not end on it", Reloaded_Creep_Is_Still'Access);
      Driver.Tests.Register ("robot.ends.limiter", "a push stopped by one channel far short of the rest names another, or "
                             & "one that spread its shortfall over the channels, one given up moving, or one not "
                             & "blocked, names one", Limiter_Finds_The_Channel_That_Stopped'Access);
      Driver.Tests.Register ("robot.ends.lone", "a lone channel short by less than the scatter of free pushes is the limiter, or "
                             & "one short by half its ask is not", A_Lone_Channel_Is_The_Limiter_Only_If_Short_By_More_Than_Its_Scatter'Access);
      Driver.Tests.Register ("robot.ends.noted", "a stop the caller called the body's is not an end, the furthest stop of "
                             & "a sense does not stand, an end is nearer than the readings seen, or an unmeasured "
                             & "channel shows one", Stops_Are_Noted_As_Ends'Access);
      Driver.Tests.Register ("robot.ends.plan", "a path past an end a channel showed is planned, or one that ends at it is not",
                             Plan_Refuses_Past_An_End'Access);
      Driver.Tests.Register ("robot.ends.file", "the ends a body showed are not in its file, or the reloaded body writes another",
                             Ends_Are_Kept_In_The_Body_File'Access);
      Driver.Tests.Register ("robot.plan.beyond", "a goal past the readings the arm has shown is not planned, its plan "
                             & "is not followed there, or a joint's end on the way is not met as Blocked or Short",
                             Plan_And_Follow_Beyond_The_Travel'Access);
      Driver.Tests.Register ("robot.kinematics.stale", "the fit of a group that stopped being an arm, or of an eye it no "
                             & "longer carries, is still taken for the arm's", Stale_Fit_Is_No_Arms'Access);
      Driver.Tests.Register ("robot.kinematics.shared", "the fit's focal length or eye pose is off by more than Z of "
                             & "its own sigmas when every keyframe's points share an error", Kinematics_With_Shared_Errors'Access);
      Driver.Tests.Register ("robot.kinematics.fields", "the fit's lens or eye pose is off by more than Z of its own "
                             & "sigmas when the matcher's errors are smooth fields over the picture, a point's in "
                             & "all its keyframes and a keyframe's own, and a shift of each keyframe",
                             Kinematics_With_Spreading_Errors'Access);
      Driver.Tests.Register ("robot.kinematics.pressed", "an arm's refit with the keyframes of its hand's presses, "
                             & "from the inputs of a recording, is not determined or not within a percent of the "
                             & "camera's focal length and centre", Kinematics_On_A_Recorded_Refit'Access);
      Driver.Tests.Register ("robot.kinematics.drawn_recording", "the same recording with its readings in another order "
                             & "and in other units is not determined, its lens off by a percent, or its joints' order "
                             & "along the arm not found", Kinematics_On_A_Drawn_Recording'Access);
      Driver.Tests.Register ("robot.order.exam", "the order exam's arms come out with a wrong order told", Order_Exam'Access);
      Driver.Tests.Register ("robot.order.exam_named", "the order exam's arms, swept a quarter as wide, each pair the fit "
                             & "names turned by the sizes it names as Boot turns it, come out with a wrong order told",
                             Order_Exam_Named'Access);
      Driver.Tests.Register ("robot.order.exam_hard", "the order exam's arms, seen as the fake robot's eyes see, come out with a wrong order told", Order_Exam_Hard'Access);
      Driver.Tests.Register ("robot.kinematics.drawn", "the synthetic arm with its readings in another order and in "
                             & "other units comes out with its axes, scales, lens, order or eye at a new pose wrong",
                             Kinematics_Of_A_Drawn_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.kinematics.wall", "an arm whose picture holds the room's far wall as many tracks "
                             & "as the table is given the wall for its table, or one that sees a thing on the table, "
                             & "the thing", Kinematics_Table_Of_The_Tracks'Access);
      Driver.Tests.Register ("robot.kinematics.resolution", "the final refinement stops while a step still moves a "
                             & "combination of the parameters by more than Unchanged_Fraction of its standard error, "
                             & "because the step is a small part of the cost",
                             Refinement_Takes_Steps_That_Move_The_Fit'Access);
      Driver.Tests.Register ("robot.kinematics.pending", "a joint whose widest keyframes have no sightings yet (their "
                             & "matches have not come back) raises in the search of its axis, or leaves the arm "
                             & "unfitted", Kinematics_With_Matches_Pending'Access);
      Driver.Tests.Register ("robot.kinematics.own_unit", "every length of an arm scaled together leaves the eye's place "
                             & "uncertain in the arm's own unit, or one length alone does not", Place_In_Own_Unit'Access);
      Driver.Tests.Register ("robot.kinematics.unit", "a keyframe taken after the arm was fitted moves the lengths of "
                             & "its frame (the unit follows every keyframe, not the first fit's), or the refit leaves "
                             & "it out", Unit_Holds_As_Keyframes_Arrive'Access);
      Driver.Tests.Register ("robot.body.file", "a body written to its file and read back is not the body that was "
                             & "written", Body_File_Round_Trip'Access);
      Driver.Tests.Register ("robot.body.method", "a quantity measured by another method is reloaded, or the "
                             & "quantities of unchanged methods are measured again", Body_File_Method_Change'Access);
      Driver.Tests.Register ("robot.body.unfinished", "the kinematics of a file written while an arm that carries an "
                             & "eye had no fit are reloaded as final, leaving that arm unswept, or a write "
                             & "leaves half a file", Body_File_Unfinished_Sweeps'Access);
      Driver.Tests.Register ("robot.body.plan", "a body reloaded from its file cannot plan a reach without a "
                             & "stream or an instrument", Plan_On_A_Reloaded_Body'Access);
      Driver.Tests.Register ("robot.body.recorded", "a body file the driver reads is not in the recording once, "
                             & "where it was read, or its recorded text reloads another body",
                             Body_File_In_The_Recording'Access);
      Driver.Tests.Register ("robot.body.line", "a body file of one line of tens of megabytes is not loaded, or is "
                             & "another body than it held", Body_File_One_Long_Line'Access);
      Driver.Tests.Register ("robot.estimate.recorded", "a decider's Estimate_Now is not in the recording once per "
                             & "call, or Observe's own recomputations are", Estimate_In_The_Recording'Access);
      Driver.Tests.Register ("robot.world.head", "two arms whose eyes never see each other's table are not placed "
                             & "through a fixed eye that sees both, or beyond the placement's own sigma",
                             Place_Through_A_Head'Access);
      Driver.Tests.Register ("robot.world.precise", "a head more precise than the arms' own fits does not place the "
                             & "second arm within its sigma: the arms' lenses and tables are taken as exact",
                             Place_Through_A_Precise_Head'Access);
      Driver.Tests.Register ("robot.head.file",
                             "a fixed eye's place is lost, or changed in the covariance or the lines of sight, by the "
                             & "body file", Head_In_The_Body_File'Access);
      Driver.Tests.Register ("robot.arm.frame", "an arm's tool, table or up in its own frame, or a reach planned there, "
                             & "moves when the arm is misplaced in the world or not placed, or comes out off the truth "
                             & "beyond Z of its own sigma", Arm_Frame_Free_Of_Placement'Access);
      Driver.Tests.Register ("robot.reach.world", "a reach given in the world for an arm placed away from the first "
                             & "does not bring its tool to the goal in the world", Reach_Through_The_Placement'Access);
      Driver.Tests.Register ("robot.kinematics.table", "with boxes on more than half of a view, an arm's table takes in "
                             & "box points or its normal is off by more than Z of its sigma", Table_Among_Boxes'Access);
      Driver.Tests.Register ("robot.kinematics.offset", "an arm whose reference keyframe lies off its sweep's base by "
                             & "less than a keyframe's match can tell is not fitted", Kinematics_With_An_Offset_Reference'Access);
      Driver.Tests.Register ("robot.kinematics.small", "a sweep too small to determine the lens and the joints is "
                             & "reported fitted", Kinematics_Of_A_Small_Sweep'Access);
      Driver.Tests.Register ("robot.kinematics.plane", "a desktop arm swept to 34 degrees in a servo's ticks, its eye on "
                             & "a plane of points or on the same with relief, is not fitted right",
                             Kinematics_On_A_Plane_In_Ticks'Access);
      Driver.Tests.Register ("robot.kinematics.plane_turn", "a plane's points seen before and after a turn about an "
                             & "axis do not give back the turn, or the plane and the shift", Turn_On_A_Plane'Access);
      Driver.Tests.Register ("robot.kinematics.relay", "an arm swept wide, its keyframes at 0.4 rad choosing points of "
                             & "their own and following them into those at 0.6, is not fitted, its focal length off by "
                             & "more than Z of its sigma, or the fit keeps none of the relayed sightings",
                             Kinematics_Of_A_Relayed_Sweep'Access);
      Driver.Tests.Register ("robot.kinematics.desktop", "a desktop servo arm swept by 1.5 to 3.4 degrees, its sightings "
                             & "erring by a pixel, a quarter of the points left, is reported fitted with a lens off by "
                             & "more than Z of its sigma, or swept to 11.5 degrees is not fitted right",
                             Kinematics_In_A_Desktop_Regime'Access);
      Driver.Tests.Register ("robot.kinematics", "the arm's axes, the eye's lens or the eye's pose at a new pose come "
                             & "out wrong from noisy matches of single-joint and Hadamard keyframes",
                             Kinematics_Of_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.boot", "the boot does not finish, deadlocks with the main loop, or does not "
                             & "recognize the rig's groups when it pushes them itself", Boot_From_Zero'Access);
      Driver.Tests.Register ("robot.settling.longest", "the longest an eye took to settle after a move is misread "
                             & "from the per-beat flags", Longest_Settling_Of_An_Eye'Access);
      Driver.Tests.Register ("robot.boot.pause", "a recognition round's hold after a push keeps doubling with the "
                             & "rounds after every eye's lag is measured: B0_1 held 384 beats a push at 32 times the "
                             & "amounts with the lags measured at 1", Pause_Follows_The_Measured_Lag'Access);
      Driver.Tests.Register ("robot.boot.hold", "an eye that sees nothing of a group keeps its recognition hold at "
                             & "the doubled guess because its own lag is never measured (B0a: the head eye)",
                             Hold_Counts_The_Seeing_Eyes'Access);
      Driver.Tests.Register ("robot.boot.servos", "arms on servos that read whole ticks, stand off a goal by a "
                             & "deadband and move so many ticks a beat are not recognized, or their delay is taken "
                             & "from a push a stop swallowed", Boot_With_Servos'Access);
      Driver.Tests.Register ("robot.boot.short", "an arm whose joints read less than they are sent is left unswept: "
                             & "its first pushes, sized by the keyframe step, rest short of it, leave no keyframe, "
                             & "and the sweep's ladder stops there as if the aligner had lost them",
                             Boot_With_Short_Deliveries'Access);
      Driver.Tests.Register ("robot.boot.undecided", "an arm whose eye is undecided after the first pushes is left "
                             & "unswept: the boot reads the body before the eye decides", Boot_With_An_Undecided_Eye'Access);
      Driver.Tests.Register ("robot.boot.patch", "an arm whose eye shows only a patch of its view after the first "
                             & "pushes, the cells that move most, is taken for a part and left unswept",
                             Boot_With_A_Patch_That_Grows'Access);
      Driver.Tests.Register ("robot.boot.step", "an arm whose eye shows a near object and a far part that moves twenty "
                             & "times less is left a part: the cells that respond do not grow by a doubling, though "
                             & "those that do not respond move together",
                             Boot_With_A_Step_Wider_Than_A_Doubling'Access);
      Driver.Tests.Register ("robot.boot.grew", "the cells that respond to a push are found to have grown by no more "
                             & "than the false alarms among those that did not, or not to have when they grew by more",
                             Growth_Of_The_Responding_Cells'Access);
      Driver.Tests.Register ("robot.kinematics.depth", "a track the sightings leave with no depth at all is a known "
                             & "point of little weight, or one that is determined is not known",
                             Depth_Is_Known_Or_Not'Access);
      Driver.Tests.Register ("robot.boot.reread", "an arm the estimate finds carrying its eye only after the body "
                             & "was first read is never swept with it", Boot_With_A_Late_Mount'Access);
      Driver.Tests.Register ("robot.boot.settling", "a sweep level whose eye's picture keeps changing for beats after "
                             & "the arm stopped gives no keyframe", Boot_With_Settling_Views'Access);
      Driver.Tests.Register ("robot.boot.keeps", "a boot that fails after recognizing the body leaves no body file to "
                             & "reload it from", Boot_Keeps_What_It_Measured'Access);
      Driver.Tests.Register ("robot.sweep.untold", "the fit names joints next to each other whose swap it tells, or "
                             & "misses two never turned together", Untold_Neighbours'Access);
      Driver.Tests.Register ("robot.sweep.stepping", "no keyframe is taken in a step under way, and one once the step "
                             & "is over", No_Keyframe_In_A_Step'Access);
      Driver.Tests.Register ("robot.sweep.twin", "an arm at rest takes no still twin of its reference, or its sweep "
                             & "starts where the view moves less than the matcher errs", Twin_Before_The_Sweep'Access);
      Driver.Tests.Register ("robot.sweep.start", "a joint's sweep starts at the turn that moves its eye's view by half "
                             & "the aligner's patch, never below the least turn a keyframe counts, and above the step one "
                             & "cell of its eye tells", Sweep_Starts_Where_A_Cell_Tells'Access);
      Driver.Tests.Register ("robot.steps.jitter", "a push never ends when the held reading jitters more than it did at "
                             & "rest", Step_Ends_Despite_New_Jitter'Access);
      Driver.Tests.Register ("robot.steps.unanswered", "a push of a group that never answered yet, which nothing "
                             & "answers, is waited for longer than any push of the body took to answer",
                             Unanswered_Push_Of_An_Unanswered_Group'Access);
      Driver.Tests.Register ("robot.steps.creeping", "a push of a closer at its limit whose reading creeps, which "
                             & "nothing answers, is taken as answered by the creep since it began",
                             Unanswered_Push_Of_A_Creeping_Group'Access);
      Driver.Tests.Register ("robot.steps.born", "a push sent while its group is still on its way from the push "
                             & "before is answered at its own start, and the body's delay is measured as nothing "
                             & "from it", Push_Born_Moving_Gives_No_Delay'Access);
      Driver.Tests.Register ("robot.channels.rounds", "the channels' noise and push rounds alternate for good, or take a "
                             & "beat whose mark alternates for rest", Alternating_Rounds_Stop'Access);
      Driver.Tests.Register ("robot.channels.echo", "a group held by sending it its own reading, never commanded, has no rest and no noise because its target changes at every beat", Held_Group_Reaches_Its_Noise'Access);
      Driver.Tests.Register ("robot.steps.chatter", "a push against something its joint keeps chattering against never "
                             & "ends, or a free push that rings about its target is given up or called blocked",
                             Step_Ends_Against_Chatter'Access);
      Driver.Tests.Register ("robot.steps.creep", "a push pressed into something its joint keeps moving about, which sets a new "
                             & "extreme of that movement now and then, never ends, or a push whose joint gives way beyond "
                             & "its chatter is given up", Step_Ends_Against_Rising_Chatter'Access);
      Driver.Tests.Register ("robot.steps.aside", "a push whose own reading sets a new extreme now and then by less than the step an "
                             & "eye can see never ends while another joint of its group moves about", Step_Ends_Beside_A_Moving_Joint'Access);
      Driver.Tests.Register ("robot.steps.springback", "a hold that asked a step of the chatter it read never ends while the arm "
                             & "moves about where it sprang back to, or ends while the arm still springs back by more than it moves about",
                             Step_Ends_After_A_Spring_Back'Access);
      Driver.Tests.Register ("robot.steps.sight", "a push of a joint an eye watches is called blocked though it stopped "
                             & "short by less than the eye can see, or asked less than the eye can see", Step_Short_Of_Sight'Access);
      Driver.Tests.Register ("robot.steps.letgo",
                             "a push that asks less than any eye can see is called blocked because the arm relaxed "
                             & "meanwhile, or joins the free pushes, or a push the joint is moved against by a visible "
                             & "motion is not blocked with a negative delivered fraction",
                             Step_Let_Go_Is_Not_Blocked'Access);
      Driver.Tests.Register ("robot.steps.deadband",
                             "a push that asks less than the least ask the body is known to execute, and is not "
                             & "answered, is judged blocked or delivered, or one that asks as much is not blocked",
                             Step_Within_The_Dead_Band_Is_Not_Judged'Access);
      Driver.Tests.Register ("robot.steps.free", "a push of a joint an eye watches is called blocked though it stopped short by "
                             & "no more than every free push of the joint does, because the joint's own error is as large "
                             & "as the step an eye can see", Step_Free_Error_As_Big_As_The_Step'Access);
      Driver.Tests.Register ("robot.steps.level", "a push of a joint an eye watches is called blocked though it stopped short by "
                             & "a little more than any free push before it, as free pushes come to fall short by more "
                             & "than they did at first", Step_Free_Level_Moves'Access);
      Driver.Tests.Register ("robot.steps.standing", "a push is called blocked because a joint of its group stands a few "
                             & "millionths off its target, beside a shortfall that no eye can see", Step_Standing_Error_Is_No_Push'Access);
      Driver.Tests.Register ("robot.steps.stuck", "a push is called free though a joint of its group stays stopped by something while the "
                             & "others move on, its target no different from the push before's", Step_Stuck_Joint_Stays_Stuck'Access);
      Driver.Tests.Register ("robot.steps.load", "a joint that holds a load, landing off its target the same way whichever "
                             & "way it is pushed, is given a dead band, or one that lands short either way is not",
                             Load_Is_No_Dead_Band'Access);
      Driver.Tests.Register ("robot.steps.standing_off", "a joint no push has moved, standing off its target, is called "
                             & "blocked for an ask within how far it stands off, or not for one beyond",
                             Standing_Off_Is_No_Block'Access);
      Driver.Tests.Register ("robot.steps.exact", "a push of a joint no eye watches whose readings repeat exactly is called blocked "
                             & "though it stopped short by one place of the float, no more than the float can tell",
                             Step_Short_By_Rounding'Access);
      Driver.Tests.Register ("robot.steps.hair", "a push of a joint an eye watches is called blocked though it stopped short by "
                             & "a hair, less than its reading tells from its noise, because a lock-in credited the joint "
                             & "a step of 3e-17", Step_Short_By_A_Hair'Access);
      Driver.Tests.Register ("robot.steps", "a free push that falls as short as free pushes do is called blocked, a "
                             & "push stopped by an obstacle or never answered is called free, or the wait for an "
                             & "answer is not the measured delay", Step_Responses'Access);
      Driver.Tests.Register ("robot.stillness", "an eye with ordinary camera noise never comes to rest, or a moving "
                             & "patch goes unnoticed", Eye_Stillness'Access);
      Driver.Tests.Register ("robot.stillness.moved", "a frame taken while the body moved joins the still view of "
                             & "before it, or a frame of a body that stood is left out of it, or the body's readings "
                             & "change the picture's own verdict", Eye_Still_View_Starts_At_Motion'Access);
      Driver.Tests.Register ("robot.stillness.watch", "a picture whose noise wanders from beat to beat, or flickers, is "
                             & "not at rest within a few beats of a move, or a drift is, or a move that decays is before "
                             & "it has", Eye_Watch_Stops_On_Noise_Not_On_Drift'Access);
      Driver.Tests.Register ("robot.roles", "a group is given the wrong role, an eye the wrong mount or lag, an arm "
                             & "is credited with a lockstep partner's eye, a reaction to another push is taken for "
                             & "a push, the tail of a slow response is taken for rest, or the step an eye can see is "
                             & "misjudged", Roles_Of_A_Synthetic_Body'Access);
      Driver.Tests.Register ("robot.settle.tail", "a body never settles while its eye's picture keeps changing at a "
                             & "level above its noise at rest, or settles while that change still shrinks",
                             Settle_After_A_Slow_Tail'Access);
      Driver.Tests.Register ("robot.settle.patch", "a patch of an eye's picture that flickers at rest moves the beat "
                             & "the body settles at", Settle_Past_A_Patch_Flickering_At_Rest'Access);
      Driver.Tests.Register ("robot.keyframe.jitter", "an arm held away from rest whose reading jitters more than it did "
                             & "at rest gives no keyframe though its eye is still", Keyframe_Despite_Held_Jitter'Access);
      Driver.Tests.Register ("robot.roles.presses", "a gripper held at an opening while the arm moves the whole hand in its "
                             & "own eye is called an arm after enough presses", Presses_Leave_A_Closer_A_Closer'Access);
      Driver.Tests.Register ("robot.roles.undecided", "a group some eye is still undecided about is called a closer or a "
                             & "part, though that eye may ride on it", Undecided_Eye_Leaves_Group_Unclassified'Access);
      Driver.Tests.Register ("robot.base.body_eye", "the walk's body eye is not an eye the carrier carries though the "
                             & "arm moves its view whole, or is an eye on another arm this arm moves whole",
                             Body_Eye_By_Mounts'Access);
      Driver.Tests.Register ("robot.kinematics.sees", "a fixed eye an arm's pushes move cells of does not see the arm, "
                             & "or one they move none of does", Fixed_Eye_Sees_An_Undecided_Arm'Access);
      Driver.Tests.Register ("robot.roles.split", "a group whose channels move two eyes apart is not one arm, and a "
                             & "channel an eye is undecided about does not split it",
                             Group_Of_Different_Eyes_Has_An_Arm_Each'Access);
      Driver.Tests.Register ("robot.roles.clearer", "a closer's remembered eye yields to another eye on an arm whose "
                             & "patch of it significantly outnumbers its own, and only to such a one",
                             Clearer_Eye_Takes_A_Closer'Access);
      Driver.Tests.Register ("robot.roles.keyframes", "an arm loses the eye its keyframes tell moves with it when the "
                             & "lock-in is undecided about it, or keeps one the lock-in sees it move nothing in",
                             Keyframes_Hold_An_Eye_On_Its_Arm'Access);
      Driver.Tests.Register ("robot.roles.noisy", "an eye that a group carries is called a patch of it because part of its "
                             & "picture is too faintly textured for its cells to tell the group's push, or a patch "
                             & "among well-textured cells is called the whole picture",
                             Carried_Eye_Partly_Textureless'Access);
      Driver.Tests.Register ("robot.roles.far", "a few cells that show a motion thousands of times the typical energy "
                             & "(the fingers of a hand in view) make every cell too noisy to tell show it, and a "
                             & "closer is called an arm",
                             Noisy_Cells_Far_Above_The_Rest'Access);
      Driver.Tests.Register ("robot.unprobed", "a group never pushed on its own is given a role from what moved "
                             & "with it", Unprobed_Group_Stays_Unclassified'Access);
      Driver.Tests.Register ("robot.channels.creep", "a reading that crawls at the start and then repeats exactly, never "
                             & "pushed, is given its crawl as rest noise", Channel_Noise_Of_A_Crawl'Access);
      Driver.Tests.Register ("robot.channels.flips", "a reading that steps by four units and flips one step at rest is "
                             & "given a jitter of its step, so a change of two steps in a beat is no motion",
                             Noise_Of_Single_Step_Flips'Access);
      Driver.Tests.Register ("robot.channels.gate", "a group whose channels' noises rest on few changes is blind to a "
                             & "change far beyond them (the gate widened by the fewest degrees of freedom)",
                             Change_Against_Noise_Of_Few_Changes'Access);
      Driver.Tests.Register ("robot.channels.integers", "readings that come as integers are given a float's resolution, "
                             & "so the first push of a tick-counting joint is a trillionth of a tick (R0)",
                             Resolution_Of_Integer_Readings'Access);
      Driver.Tests.Register ("robot.channels", "reading noise is misjudged (a reading that mostly repeats exactly is "
                             & "given noise zero, so its jitter passes for motion), a hold is taken for a push, or a "
                             & "push never ends", Channel_Noise_And_Pushes'Access);
      Driver.Tests.Register ("robot.channels.visible", "a visible step far below the readings' noise makes every jitter "
                             & "motion, so a group that creeps has no beat at rest and no noise, a change below what any "
                             & "eye sees is motion by the noise alone, or a change is motion where no noise is measured",
                             Visible_Step_Needs_The_Noise'Access);
      Driver.Tests.Register ("robot.flow", "a cell's displacement between two frames is misestimated or a flat cell "
                             & "reports one", Flow_Recovers_Shifts'Access);
      Driver.Tests.Register ("robot.regression", "wild observations or collinear regressors bend the robust fit, "
                             & "or a block test calls noise significant", Regression_Ignores_Outliers'Access);
   end Register;

end Driver.Robot.Tests;
