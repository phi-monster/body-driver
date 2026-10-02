with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Instrument;
with Driver.Observations;
with Driver.Beats;
with Driver.Log;
with Driver.Robot.Boot;
with Driver.Robot.Kinematics;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;
with Driver.Robot.Kinematics.Fit;
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

   Rig_Width  : constant := 64;
   Rig_Height : constant := 48;
   Px_Per_Unit : constant := 4.0;

   type Rig_State is record
      Arm_1, Arm_2 : Real_Array (1 .. 2) := [0.0, 0.0];
      Closer, Part, Idle : Real := 0.0;
   end record;

   function Render (Eye : Positive; S : Rig_State) return Driver.Images.Image is
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
                     L := Texture (Xr + Px_Per_Unit * S.Arm_2 (1), Yr + Px_Per_Unit * S.Arm_2 (2), 2.0);
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

   type Push_Kind is (Arm_1, Arm_2, Closer, Part, Idle, Lockstep, Closer_With_Arm_1);

   --  Pushes one group away by Amount and back, Times times, holding two
   --  beats after each move; Lockstep moves both arms together.
   procedure Exercise (M : in out Model; R : in out Rig; Kind : Push_Kind; Amount : Real; Times : Positive) is
      Rest : constant Rig_State := (others => <>);
      Away : Rig_State := Rest;
   begin
      case Kind is
         when Arm_1    => Away.Arm_1 := [Amount, Amount / 2.0];
         when Arm_2    => Away.Arm_2 := [Amount / 2.0, Amount];
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

   --  A group that moves a patch in arm 1's eye while another eye is still
   --  undecided about it (a live x5's right arm at its first read: a patch
   --  in the left wrist's eye, the right wrist's eye undecided at 267 of 525
   --  cells) may carry that eye: it is no closer until the eye decides, and
   --  a closer once it decides that nothing moved.
   procedure Undecided_Eye_Leaves_Group_Unclassified is
      M : Model;
      Undecided_Third : constant Eye_Effect :=
        (Verdict => Undecided, Responding => 24, Textured => 48,
         Fraction => (Value => 0.5, Sigma => 0.07, Degrees_Of_Freedom => 0));
      Nothing_Third : constant Eye_Effect :=
        (Verdict => Nothing, Responding => 0, Textured => 48,
         Fraction => (Value => 0.0, Sigma => 0.0, Degrees_Of_Freedom => 0));
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
          Fraction => (Value => 8.0 / 48.0, Sigma => 0.05, Degrees_Of_Freedom => 0)));
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

   --  An arm at rest takes its reference keyframe and then a still twin of
   --  it at the same pose, whose match measures the matcher's own error
   --  before the arm moves (on the rig, which has no instrument, the match
   --  is refused for good, and the twin counts as answered). The sweep then
   --  starts where both the view and the matcher can tell a move: given a
   --  matcher that errs by 0.5 pixels, far more than the cells' noise, at Z
   --  times that over the view's shift.
   procedure Twin_Before_The_Sweep is
      M    : Model;
      R    : Rig;
      Rest : constant Rig_State := (others => <>);
   begin
      Exercise_Rig (M);
      for B in 1 .. 12 loop
         Step (M, R, Rest);
      end loop;
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
         Check (Twins = 1, "arm 1 at rest has" & Twins'Image & " still twins of its reference, not one");
      end;
      Check (Driver.Robot.Kinematics.Twin_Answered (M, 1), "the twin of an arm whose matches are refused for good is waited for");
      --  A matcher whose round trips come back half a pixel off, both ways.
      for E of M.Kinematics loop
         if E.Group = 1 then
            declare
               Set : Match_Set;
            begin
               Set.Frame := 2;
               for I in 0 .. Natural (E.Query_U.Length) - 1 loop
                  Set.To_U.Append (E.Query_U (I));
                  Set.To_V.Append (E.Query_V (I));
                  Set.Back_U.Append (E.Query_U (I) + (if I mod 2 = 0 then 0.5 else -0.5));
                  Set.Back_V.Append (E.Query_V (I) + (if I mod 2 = 0 then -0.5 else 0.5));
                  Set.Found.Append (True);
               end loop;
               E.Matches.Append (Set);
            end;
         end if;
      end loop;
      declare
         Noise : constant Real := Driver.Robot.Kinematics.Match_Noise (M, 1);
      begin
         Check (Noise > Lockin.Cell_Noise (M, 1), "the matcher's half-pixel error is not measured above the cells' noise:"
                & Noise'Image);
         for C in 1 .. 2 loop
            declare
               Start : constant Real := Driver.Robot.Motion.Sweep_Start (M, 1, C);
               Shift : constant Real := Lockin.Shift (M, 1, 1, C);
            begin
               Check (Shift > 0.0, "arm 1's shift is not measured");
               if Shift > 0.0 then
                  Check_Close (Start * Shift, Driver.Conventions.Z * Noise, 1.0e-9 * Noise,
                               "the first level of joint" & C'Image & " moves the view by what the matcher can tell");
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
   procedure Settle_After_A_Slow_Tail is
      M      : Model;
      Width  : constant := 64;
      Height : constant := 48;
      Reading, Target : Real := 0.0;
      Settled_At : Natural := 0;
      Decaying_Until : Natural := 0;   --  the last beat the flicker shrank by more than a hundredth

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
                  if (X * 7 + Y * 13) mod 10 = 0 then
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
      Check (not M.Eyes (1).Is_Still, "the lasting flicker is still to the rest-noise test: the test shows nothing");
      Check (Settled_At > 0, "the body never settled while its eye's picture flickered at a lasting level");
      Check (Settled_At = 0 or else Settled_At > Decaying_Until,
             "the body settled at beat" & Settled_At'Image & " while the flicker still shrank, until" & Decaying_Until'Image);
   end Settle_After_A_Slow_Tail;

   --  The sweep starts each joint where its eye's view moves by what one cell
   --  of it tells: Z times the cells' displacement noise. The visible step,
   --  which a matched filter over every cell sees, moves the view by far less
   --  than any one point of a match can be told.
   procedure Sweep_Starts_Where_A_Cell_Tells is
      M : Model;
   begin
      Exercise_Rig (M);
      for C in 1 .. 2 loop
         declare
            Start : constant Real := Driver.Robot.Motion.Sweep_Start (M, 1, C);
            Shift : constant Real := Lockin.Shift (M, 1, 1, C);
            Noise : constant Real := Lockin.Cell_Noise (M, 1);
         begin
            Check (Start > 0.0 and then Shift > 0.0 and then Noise < Real'Last, "arm 1's sweep plan is not measured");
            if Start > 0.0 and then Shift > 0.0 and then Noise < Real'Last then
               Check_Close (Start * Shift, Driver.Conventions.Z * Noise, 1.0e-9 * Noise,
                            "the first level moves the view of joint" & C'Image);
               Check (Known (Visible_Step (M, 1, C)) and then Start > Visible_Step (M, 1, C).Value,
                      "joint" & C'Image & "'s sweep starts at its visible step or below");
            end if;
         end;
      end loop;
   end Sweep_Starts_Where_A_Cell_Tells;

   --  An arm's match was asked, then the graph stopped listing the arm (an
   --  estimate told its eye or group otherwise): its answer must still be
   --  read, or the boot's wait for every answer never ends.
   procedure Answers_Read_For_An_Unlisted_Arm is
      M : Model;
      R : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
   begin
      R.Pending.Append
        (Pending_Match'(Frame  => 2,
                        Ticket => Driver.Instrument.Submit_Match
                          ((Stored => False, Image => Driver.Images.No_Image),
                           (Stored => False, Image => Driver.Images.No_Image),
                           [1 => (U => 1.0, V => 1.0)], True, 0)));
      M.Kinematics.Append (R);
      Check (Driver.Robot.Kinematics.Pending (M) = 1, "the match was not asked");
      for B in 0 .. 3 loop
         declare
            O    : Observation;
            Sent : Driver.Commands.Command;
         begin
            O.Beat := Driver.Clock.Beat (B);
            O.Readings.Append (Real_Array'(1 => 0.0));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            Driver.Commands.Set_Target (Sent, 1, [0.0]);
            Observe (M, O, Sent);
         end;
      end loop;
      Check (Arm_Count (M) = 0, "the graph lists an arm");
      Check (Driver.Robot.Kinematics.Pending (M) = 0,
             "the answer to an arm the graph no longer lists is never read: the boot waits forever");
   end Answers_Read_For_An_Unlisted_Arm;

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
            Stillness.Judge_Eye (S, I, L);
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
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0)));
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
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0)));
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
            Check (not E.Ended or else not E.Settled or else E.End_At - E.Closest_At <= 2 * (E.Closest_At - E.Start) + 1,
                   "the push against the obstacle ends" & Natural'Image (E.End_At - E.Closest_At)
                   & " beats after it last came closer," & Natural'Image (E.Closest_At - E.Start) & " beats after it began");
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
                      Fraction => (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0)));
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

   procedure Step_Responses is
      M    : Model;
      Rng  : Generator;
      O    : Observation;
      Sent : Driver.Commands.Command;
      --  Pushes every twelve beats. The reading answers two beats after a push
      --  and settles three beats later, 0.05 short of the target with a
      --  scatter of 0.001, as a joint held against gravity does, which is many
      --  times the readings' noise. Pushes 1 to 5 and 7 move freely; push 6
      --  meets an obstacle at 2.0; push 8 is never answered.
      Targets : constant Real_Array (1 .. 8) := [1.0, 2.0, 1.0, 2.0, 1.0, 3.0, 1.0, 1.5];
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
      Arm_2_Poses : out Natural; Eye_2_Noise : Real := 0.0; Eye_2_Lag : Positive := 1)
   is
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
         Boot.Run (M, H, "", Fine);
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
   begin
      Beats := 0;
      Still_Poses := 0;
      Arm_2_Poses := 0;
      begin
      for B in 0 .. Bound loop
         exit when Finished;
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
                       Render (E, (if E = 2 and then Eye_2_Lag > 1 then History (B mod Eye_2_Lag) else Drawn));
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
            O.Readings.Append (Now.Arm_1);
            O.Readings.Append (Now.Arm_2);
            O.Readings.Append (Real_Array'(1 => Now.Closer));
            O.Readings.Append (Real_Array'(1 => Now.Part));
            O.Readings.Append (Real_Array'(1 => Now.Idle));
            O.Readings.Append (Real_Array'(1 => Now.Arm_1 (1) + Now.Arm_2 (1)));
            O.Readings.Append (Real_Array'(1 => 7.0));
            for G in 1 .. 7 loop
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
            Driver.Robot.Hand.Observe (H, M, O, Sent);
            if Arm_Count (M) >= 1 and then Now.Arm_1 /= [0.0, 0.0]
              and then Driver.Robot.Kinematics.Held_Still (M, 1, M.Beats - 1)
              and then (for all K in 1 .. Still_Poses => Poses (K) /= Now.Arm_1)
              and then Still_Poses < Max_Poses
            then
               Still_Poses := Still_Poses + 1;
               Poses (Still_Poses) := Now.Arm_1;
            end if;
            for A in 1 .. Arm_Count (M) loop
               if Arm_Group (M, Arm_Id (A)) = 2 and then Now.Arm_2 /= [0.0, 0.0]
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
            Now.Arm_1 := Driver.Commands.Target (Sent, 1);
            Now.Arm_2 := Driver.Commands.Target (Sent, 2);
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
      Driver.Log.Line (Driver.Log.Robot, "boot from zero took" & Beats'Image & " beats");
   end Boot_From_Zero;

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
      Every  : Natural := 0;   --  all its single-joint levels, both ways
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
               Shift  : constant Real := Lockin.Shift (M, 1, 1, C);
               Offset : Real := First;
               Level  : Positive := 1;
            begin
               if First > 0.0 and then Shift > 0.0 then
                  while Offset * Shift <= Half loop
                     Every := Every + 2;
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
      --  The levels between are passed as soon as the arm stops, before the
      --  picture settles: no keyframe there.
      Check (Poses < Every, "arm 1 was held for a keyframe at every one of its" & Every'Image & " levels (" & Poses'Image
             & " poses)");
   end Boot_With_Settling_Views;

   --  A probe of a joint read exactly (noise 1e-13) whose reading settles a
   --  hair off its target, the more the further it goes (by the square of the
   --  offset, as a joint held against a spring does), and that stops at 1e-3:
   --  the fraction of each offset it delivers shrinks with every doubling,
   --  though it follows each one. The probe must double until the joint stops,
   --  not call the second doubling its end. No eye sees the joint.
   procedure Probe_A_Drooping_Joint is
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
      begin
         --  Long enough at rest for the readings' noise to be measured, and
         --  a few pushes for the joint's delay and the eyes' lag: the probe
         --  looks once the reading has had time to follow and the eyes to
         --  show it.
         Driver.Robot.Motion.Settle (M, W);
         Driver.Robot.Motion.Hold (M, 100);
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
         Driver.Beats.Within_A_Beat (Estimate'Access);
         Driver.Robot.Motion.Gather_Rest (M, 2);
         --  From an offset of 1e-5, as a group's probe starts from the amount
         --  the probe of every channel together was first seen at.
         Driver.Robot.Motion.Probe_Together (M, [1 => (Group => 5, Channel => 1)], 1.0, 1.0e-5, R);
         Steps := R.Steps;
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
               declare
                  T : constant Real := Real'Min (Driver.Commands.Target (Sent, 5) (1), Limit);
               begin
                  Now.Idle := T - T * abs T;
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
      Check (Done, "the probe did not finish");
      --  From 1e-5 the joint follows seven doublings, to 1.28e-3, and stops at
      --  1e-3: the ninth level is the first that takes it no further.
      Check (Steps = 9, "the probe called the joint's end after" & Steps'Image & " levels, not 9");
   end Probe_A_Drooping_Joint;

   --  ── Probing a channel both ways ──
   --
   --  The rig of the drooping-joint probe, its group 5 (which no eye sees)
   --  resting at 0 and reading its target as Kind says, probed both ways
   --  from 1e-5 against Bound (where every other channel of the body has
   --  answered). The largest targets asked of it each way are kept.

   type Idle_Kind is (At_Upper_Limit, Deadband, Disconnected);

   --  At its upper limit, 0, and free down to -1e-3; a deadband of 5e-5 each
   --  way, free beyond to 1e-3; disconnected, never moving.
   function Idle_Reading (Kind : Idle_Kind; Target : Real) return Real is
     (case Kind is
         when At_Upper_Limit => Real'Max (-1.0e-3, Real'Min (Target, 0.0)),
         when Deadband       => (if abs Target < 5.0e-5 then 0.0 else Real'Max (-1.0e-3, Real'Min (Target, 1.0e-3))),
         when Disconnected   => 0.0);

   procedure Probe_Idle_Both_Ways
     (Kind     : Idle_Kind;
      Bound    : Real;
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
         Driver.Robot.Motion.Probe_Both_Ways (M, (Group => 5, Channel => 1), 1.0e-5, Bound, Got);
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

   --  A closer at its upper limit (a live x5's closer rests at 1.0 and was
   --  asked 6.87e10 upwards): it answers downwards at the first level, so
   --  the upward way stops there, never asked more than that first level.
   --  A deadband is two-sided: small asks fail both ways, a larger one
   --  succeeds, and the channel is found answering, not called dead or at
   --  an end. A disconnected channel answers neither way: once both ways
   --  have been asked as much as every other channel of the body needed, it
   --  is called dead and asked no more.
   procedure Probe_Limits_And_Deadbands is
      use type Driver.Robot.Motion.Sense;
      package Mo renames Driver.Robot.Motion;
      R        : Mo.Two_Way_Report;
      Up, Down : Real;
      Finished : Boolean;
   begin
      Probe_Idle_Both_Ways (At_Upper_Limit, Real'Last, R, Up, Down, Finished);
      Check (Finished, "the probe of a channel at its upper limit did not finish");
      Check (R.At_End (Mo.Increasing) and then R.Levels (Mo.Increasing) = 1,
             "the upward way of a channel at its upper limit was asked" & R.Levels (Mo.Increasing)'Image
             & " levels, not stopped at the first, where the downward way answered");
      Check (Up <= 1.0e-5, "the channel at its upper limit was asked" & Up'Image & " upwards");
      Check (R.Answered = 1.0e-5 and then not R.Dead and then not R.At_End (Mo.Decreasing),
             "the channel at its upper limit is not found answering downwards from the first level");
      --  Down from 1e-5 it follows to 1e-3 (level 8, 1.28e-3, takes it there);
      --  level 9 takes it no further: its own end.
      Check (R.Levels (Mo.Decreasing) = 9, "the downward way ended after" & R.Levels (Mo.Decreasing)'Image & " levels, not 9");

      --  Every other channel has answered by 1e-4; the deadband yields at
      --  level 4, 8e-5.
      Probe_Idle_Both_Ways (Deadband, 1.0e-4, R, Up, Down, Finished);
      Check (Finished, "the probe of a channel with a deadband did not finish");
      Check (not R.Dead and then R.Answered = 8.0e-5,
             "a deadband of 5e-5 is not found answering at 8e-5: answered at" & R.Answered'Image
             & (if R.Dead then ", called dead" else ""));
      Check (not R.At_End (Mo.Increasing) and then not R.At_End (Mo.Decreasing),
             "a channel with a deadband is called at an end");

      --  Every other channel has answered by 4e-5 (level 3).
      Probe_Idle_Both_Ways (Disconnected, 4.0e-5, R, Up, Down, Finished);
      Check (Finished, "the probe of a disconnected channel did not finish");
      Check (R.Dead and then R.Levels (Mo.Increasing) = 3 and then R.Levels (Mo.Decreasing) = 3,
             "a channel that answers neither way is not called dead once both ways were asked 4e-5: levels"
             & R.Levels (Mo.Increasing)'Image & R.Levels (Mo.Decreasing)'Image);
      Check (Up <= 4.0e-5 and then Down <= 4.0e-5,
             "the dead channel was asked" & Up'Image & " up and" & Down'Image & " down");
   end Probe_Limits_And_Deadbands;

   --  ── The kinematics of a synthetic arm ──
   --
   --  Six turning joints carry an eye of 640 x 480 pixels with a focal length
   --  of 400 pixels over a slanted table. The arm turns every joint alone both
   --  ways by 0.05, 0.1 and 0.2 radians, then all of them in seven cells whose
   --  signs are the rows of a Sylvester-Hadamard matrix; a 16 x 12 grid of the
   --  reference view is followed into every keyframe with 0.2 pixels of noise.

   --  Frame_Error is the spread of an error shared by every point of a
   --  keyframe (its rendering, its view), each keyframe's drawn at random:
   --  the fit's reported uncertainty must still cover its errors.
   procedure Synthetic_Sweep (Scale : Real; Expect_Fit : Boolean; Frame_Error : Real := 0.0) is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Levels  : constant Real_Array := [0.05 * Scale, -0.05 * Scale, 0.1 * Scale, -0.1 * Scale, 0.2 * Scale, -0.2 * Scale];
      Cells   : constant := 7;
      Frames  : constant Positive := 1 + N * Levels'Length + Cells;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant := 0.2;
      Table   : constant Vec3 := [0.0, -0.6, -0.8];   --  its normal, towards the eye
      Truth   : Fit.Joint_Array (1 .. N);
      Lens    : constant Fit.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Changes : Real_Matrix (1 .. Frames, 1 .. N) := [others => [others => 0.0]];
      Rng     : Generator;
      type Sighting_Access is access Fit.Sighting_Array;
      All_Seen : constant Sighting_Access := new Fit.Sighting_Array (1 .. (Frames - 1) * Columns * Rows);
      Seen     : Natural := 0;
   begin
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
            T := Inverse (Fit.Eye_At (Truth, D));
            declare
               Shared_U : constant Real := Frame_Error * Gaussian (Rng);
               Shared_V : constant Real := Frame_Error * Gaussian (Rng);
            begin
            for Gy in 1 .. Rows loop
               for Gx in 1 .. Columns loop
                  declare
                     U0 : constant Real := (Real (Gx) - 0.5) * 640.0 / Real (Columns);
                     V0 : constant Real := (Real (Gy) - 0.5) * 480.0 / Real (Rows);
                     --  The table: the plane one unit from the eye along its normal.
                     H     : constant Vec3 := Fit.Ray (Lens, U0, V0);
                     Depth : constant Real := -1.0 / Real'(Unit (Table) * H);
                     X  : constant Vec3 := Depth * H;
                     U, V : Real;
                     Ahead : Boolean;
                  begin
                     Fit.Project (Lens, T * X, U, V, Ahead);
                     U := U + Noise * Gaussian (Rng) + Shared_U;
                     V := V + Noise * Gaussian (Rng) + Shared_V;
                     if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 then
                        Seen := Seen + 1;
                        All_Seen (Seen) := (Frame => F, Track => (Gy - 1) * Columns + Gx, U0 => U0, V0 => V0, U => U, V => V);
                     end if;
                  end;
               end loop;
            end loop;
            end;
         end;
      end loop;
      declare
         Joints : Fit.Joint_Array (1 .. N);
         Found  : Fit.Lens;
         Report : Fit.Fit_Report;
      begin
         Fit.Fit (Changes, [1 .. N => 0.01 * Scale], All_Seen (1 .. Seen), 640, 480, Joints, Found, Report);
         if not Expect_Fit then
            Check (not Report.Fitted, "a sweep of" & Real'Image (0.2 * Scale) & " rad at most was fitted, focal"
                   & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy));
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
         if Frame_Error = 0.0 then
            Check_Close (Found.Fx, Lens.Fx, Lens.Fx * Noise / 40.0, "the focal length across");
            Check_Close (Found.Fy, Lens.Fy, Lens.Fy * Noise / 40.0, "the focal length down");
            for J in 1 .. N loop
               Check (Arccos (Real'Min (1.0, Joints (J).W * Truth (J).W)) < Noise / 40.0,
                      "joint" & J'Image & "'s axis is off by"
                      & Real'Image (Arccos (Real'Min (1.0, Joints (J).W * Truth (J).W))) & " rad");
            end loop;
         end if;
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
                  Scale := Scale + Fit.Eye_At (Truth, D).Translation * Fit.Eye_At (Truth, D).Translation;
               end;
            end loop;
            Scale := Sqrt (Scale / Real (Frames));
            Want := Fit.Eye_At (Truth, Test);
            Got := Fit.Eye_At (Joints, Test);
            if Frame_Error = 0.0 then
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
               Fit.Pose_Covariance (Joints, Test, Report.Covariance, Turn, Place);
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
         if Frame_Error = 0.0 then
         declare
            Normal : Vec3;
            Sigma  : Real;
            Flat   : Boolean;
         begin
            Fit.Table (Changes, All_Seen (1 .. Seen), Joints, Found, Normal, Sigma, Flat);
            Check (Flat, "no table found");
            Check (Arccos (Real'Min (1.0, Normal * Unit (Table))) < Noise / 40.0,
                   "the table's normal is off by" & Real'Image (Arccos (Real'Min (1.0, Normal * Unit (Table)))) & " rad");
            Check (Sigma < Noise / 40.0, "the table's normal is uncertain by" & Sigma'Image & " rad");
         end;
         end if;
         Driver.Log.Line (Driver.Log.Robot, "kinematics test: focal " & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy)
                          & ", median " & Real'Image (Report.Median_Px) & " px over" & Report.Used'Image & " sightings");
      end;
   end Synthetic_Sweep;

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
            Arm.Result.Joints.Append (Joint_Fit'(W => Truth (J).W, P => Truth (J).P, C => 1.0, Slide => False));
            Arm.Result.Reference.Append (0.0);
         end;
      end loop;
      Arm.Result.Fitted := True;
      M.Kinematics.Append (Arm);
      --  The graph lists group 1 as arm 1, carrying eye 1: the fit is the
      --  arm's now.
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      Goal := Fit.Eye_At (Truth, Goal_Q);
      Driver.Robot.Kinematics.Solve_Pose (M, 1, Zero, Goal, False, [1 .. 6 => -1.0], [1 .. 6 => 1.0], Q, Position_Off, Turn_Off);
      Check (Position_Off < 1.0e-9 and then Turn_Off < 1.0e-9,
             "a reachable pose is missed by" & Position_Off'Image & " and" & Turn_Off'Image & " rad");
      declare
         Got : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q);
      begin
         Check (abs (Got.Translation - Goal.Translation) < 1.0e-9, "the readings found do not put the eye at the goal");
      end;
      --  The same goal with the readings held within a tenth of a radian.
      Driver.Robot.Kinematics.Solve_Pose (M, 1, Zero, Goal, False, [1 .. 6 => -0.1], [1 .. 6 => 0.1], Q, Position_Off, Turn_Off);
      Check (Position_Off > 1.0e-3 or else Turn_Off > 1.0e-3, "a pose beyond the readings' range is reached");
      Check ((for all X of Q => abs X <= 0.1), "the readings found leave their range");
   end Reach_A_Pose;

   --  A fit belongs to an arm only while the graph has its group as that arm,
   --  carrying that eye: once the group stops being an arm, or the eye rides
   --  on another, the fit is no arm's, and the refit clears it.
   procedure Stale_Fit_Is_No_Arms is
      M   : Model;
      Arm : Arm_Evidence := (Arm => 1, Group => 1, Eye => 1, others => <>);
   begin
      for J in 1 .. 2 loop
         Arm.Result.Joints.Append (Joint_Fit'(W => [0.0, 0.0, 1.0], P => [0.1, 0.0, 0.0], C => 1.0, Slide => False));
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

   procedure Kinematics_Of_A_Synthetic_Arm is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True);
   end Kinematics_Of_A_Synthetic_Arm;

   --  The same arm swept a five-hundredth as far: the image moves by less
   --  than its noise, and the fit must say it cannot tell.
   procedure Kinematics_Of_A_Small_Sweep is
   begin
      Synthetic_Sweep (0.002, Expect_Fit => False);
   end Kinematics_Of_A_Small_Sweep;

   --  The same arm, every keyframe's points sharing an error of 0.3 pixels
   --  (a rendering, a view, the matcher on that pair), as A9's matches did:
   --  sightings taken as independent hide it, and the fit's focal length
   --  and pose come out many of their sigmas off; clustered by keyframe, the
   --  fit's own uncertainty covers its errors.
   procedure Kinematics_With_Shared_Errors is
   begin
      Synthetic_Sweep (1.0, Expect_Fit => True, Frame_Error => 0.3);
   end Kinematics_With_Shared_Errors;

   procedure Register is
   begin
      Driver.Tests.Register ("robot.estimate.task", "an estimate over a long history fails in a task with the default "
                             & "stack, as the decider's does", Estimate_In_A_Task'Access);
      Driver.Tests.Register ("robot.probe.limits", "a channel at its limit one way is asked ever further that way though "
                             & "it answered the other way, a deadband is not found, or a channel that answers neither way "
                             & "is asked past where every other channel answered", Probe_Limits_And_Deadbands'Access);
      Driver.Tests.Register ("robot.probe.droop", "a probe calls a joint at its end when the fraction of each offset "
                             & "it delivers shrinks, though it still follows", Probe_A_Drooping_Joint'Access);
      Driver.Tests.Register ("robot.reach", "the readings that put an arm's eye at a pose are not found, or are "
                             & "found beyond the range the arm moved through", Reach_A_Pose'Access);
      Driver.Tests.Register ("robot.kinematics.stale", "the fit of a group that stopped being an arm, or of an eye it no "
                             & "longer carries, is still taken for the arm's", Stale_Fit_Is_No_Arms'Access);
      Driver.Tests.Register ("robot.kinematics.shared", "the fit's focal length or eye pose is off by more than Z of "
                             & "its own sigmas when every keyframe's points share an error", Kinematics_With_Shared_Errors'Access);
      Driver.Tests.Register ("robot.kinematics.small", "a sweep too small to determine the lens and the joints is "
                             & "reported fitted", Kinematics_Of_A_Small_Sweep'Access);
      Driver.Tests.Register ("robot.kinematics", "the arm's axes, the eye's lens or the eye's pose at a new pose come "
                             & "out wrong from noisy matches of single-joint and Hadamard keyframes",
                             Kinematics_Of_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.boot", "the boot does not finish, deadlocks with the main loop, or does not "
                             & "recognize the rig's groups when it pushes them itself", Boot_From_Zero'Access);
      Driver.Tests.Register ("robot.boot.undecided", "an arm whose eye is undecided after the first pushes is left "
                             & "unswept: the boot reads the body before the eye decides", Boot_With_An_Undecided_Eye'Access);
      Driver.Tests.Register ("robot.boot.reread", "an arm the estimate finds carrying its eye only after the body "
                             & "was first read is never swept with it", Boot_With_A_Late_Mount'Access);
      Driver.Tests.Register ("robot.boot.settling", "a sweep level whose eye's picture keeps changing for beats after "
                             & "the arm stopped gives no keyframe", Boot_With_Settling_Views'Access);
      Driver.Tests.Register ("robot.sweep.twin", "an arm at rest takes no still twin of its reference, or its sweep "
                             & "starts where the view moves less than the matcher errs", Twin_Before_The_Sweep'Access);
      Driver.Tests.Register ("robot.sweep.start", "a joint's sweep starts below where one cell of its eye tells the "
                             & "view moved", Sweep_Starts_Where_A_Cell_Tells'Access);
      Driver.Tests.Register ("robot.answers.unlisted", "the answers to an arm the graph no longer lists are never read, "
                             & "so the wait for every answer never ends", Answers_Read_For_An_Unlisted_Arm'Access);
      Driver.Tests.Register ("robot.steps.jitter", "a push never ends when the held reading jitters more than it did at "
                             & "rest", Step_Ends_Despite_New_Jitter'Access);
      Driver.Tests.Register ("robot.steps.unanswered", "a push of a group that never answered yet, which nothing "
                             & "answers, is waited for longer than any push of the body took to answer",
                             Unanswered_Push_Of_An_Unanswered_Group'Access);
      Driver.Tests.Register ("robot.channels.rounds", "the channels' noise and push rounds alternate for good, or take a "
                             & "beat whose mark alternates for rest", Alternating_Rounds_Stop'Access);
      Driver.Tests.Register ("robot.steps.chatter", "a push against something its joint keeps chattering against never "
                             & "ends, or a free push that rings about its target is given up or called blocked",
                             Step_Ends_Against_Chatter'Access);
      Driver.Tests.Register ("robot.steps.sight", "a push of a joint an eye watches is called blocked though it stopped "
                             & "short by less than the eye can see, or asked less than the eye can see", Step_Short_Of_Sight'Access);
      Driver.Tests.Register ("robot.steps", "a free push that falls as short as free pushes do is called blocked, a "
                             & "push stopped by an obstacle or never answered is called free, or the wait for an "
                             & "answer is not the measured delay", Step_Responses'Access);
      Driver.Tests.Register ("robot.stillness", "an eye with ordinary camera noise never comes to rest, or a moving "
                             & "patch goes unnoticed", Eye_Stillness'Access);
      Driver.Tests.Register ("robot.roles", "a group is given the wrong role, an eye the wrong mount or lag, an arm "
                             & "is credited with a lockstep partner's eye, a reaction to another push is taken for "
                             & "a push, the tail of a slow response is taken for rest, or the step an eye can see is "
                             & "misjudged", Roles_Of_A_Synthetic_Body'Access);
      Driver.Tests.Register ("robot.settle.tail", "a body never settles while its eye's picture keeps changing at a "
                             & "level above its noise at rest, or settles while that change still shrinks",
                             Settle_After_A_Slow_Tail'Access);
      Driver.Tests.Register ("robot.keyframe.jitter", "an arm held away from rest whose reading jitters more than it did "
                             & "at rest gives no keyframe though its eye is still", Keyframe_Despite_Held_Jitter'Access);
      Driver.Tests.Register ("robot.roles.undecided", "a group some eye is still undecided about is called a closer or a "
                             & "part, though that eye may ride on it", Undecided_Eye_Leaves_Group_Unclassified'Access);
      Driver.Tests.Register ("robot.unprobed", "a group never pushed on its own is given a role from what moved "
                             & "with it", Unprobed_Group_Stays_Unclassified'Access);
      Driver.Tests.Register ("robot.channels", "reading noise is misjudged (a reading that mostly repeats exactly is "
                             & "given noise zero, so its jitter passes for motion), a hold is taken for a push, or a "
                             & "push never ends", Channel_Noise_And_Pushes'Access);
      Driver.Tests.Register ("robot.flow", "a cell's displacement between two frames is misestimated or a flat cell "
                             & "reports one", Flow_Recovers_Shifts'Access);
      Driver.Tests.Register ("robot.regression", "wild observations or collinear regressors bend the robust fit, "
                             & "or a block test calls noise significant", Regression_Ignores_Outliers'Access);
   end Register;

end Driver.Robot.Tests;
