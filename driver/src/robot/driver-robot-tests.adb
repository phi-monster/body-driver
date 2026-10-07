with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
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
with Driver.Numerics.Dense;
with Ada.Strings.Fixed;
with Driver.Robot.Body_File;
with Driver.Robot.Kinematics.Errors.Tests;
with Driver.Recording;
with Ada.Text_IO;
with GNAT.OS_Lib;
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
   --  it settles without the patch.
   procedure Settle_Past_A_Patch_Flickering_At_Rest is
      Plain, Patched : Model;
      Plain_At, Patched_At, Decaying_Until : Natural;
   begin
      Settle_Run (False, Plain, Plain_At, Decaying_Until);
      Settle_Run (True, Patched, Patched_At, Decaying_Until);
      Check (Plain_At > 0 and then Patched_At = Plain_At,
             "with a patch flickering at rest the body settled at beat" & Patched_At'Image & ", without it at"
             & Plain_At'Image);
   end Settle_Past_A_Patch_Flickering_At_Rest;

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
                        Eye    => 0,
                        Points => 1,
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
      Arm_2_Poses : out Natural; Eye_2_Noise : Real := 0.0; Eye_2_Lag : Positive := 1;
      File : String := ""; Stop_Once_Kept : Boolean := False)
   is
      --  File is the body file the boot is given. With Stop_Once_Kept, the main
      --  loop stops answering a few beats after the file first exists, as if
      --  the boot had died there, and the decider is aborted.
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
   begin
      Beats := 0;
      Still_Poses := 0;
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
      GNAT.OS_Lib.Delete_File (Name.all, Gone);   --  the boot finds no body file to reload
      Boot_On_Rig (M, False, Done, Ok, Beats, Poses, Poses_2, File => Name.all, Stop_Once_Kept => True);
      Check (not Done, "the boot finished before a body file was left to stop it at");
      Check (GNAT.OS_Lib.Is_Regular_File (Name.all),
             "a boot that died during the arms' sweeps left no body file, though it had recognized the body");
      if GNAT.OS_Lib.Is_Regular_File (Name.all) then
         Load_Body (Back, Name.all, Ok, Why);
         Check (Ok, "the body file was not loaded: " & Ada.Strings.Unbounded.To_String (Why));
         Check (Reloaded (Back, Stored_Graph), "the recognized body was not reloaded");
         Check (Role (Back, 1) = Arm and then Role (Back, 2) = Arm and then Role (Back, 4) = Part,
                "the reloaded body has other roles than the boot recognized: " & Role (Back, 1)'Image & ", "
                & Role (Back, 2)'Image & ", " & Role (Back, 4)'Image);
         Check (Eye_Mount (Back, 1) = Eye_Mount (M, 1) and then Eye_Mount (Back, 2) = Eye_Mount (M, 2),
                "the reloaded body mounts its eyes elsewhere");
         GNAT.OS_Lib.Delete_File (Name.all, Gone);
      end if;
      Check (not GNAT.OS_Lib.Is_Regular_File (Name.all & ".part"), "a write left its half behind");
   end Boot_Keeps_What_It_Measured;

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

   --  What the model has of group 5's noise when the probe begins: measured
   --  (the estimates before it), lost (its noise unmeasured, as a group that
   --  never came to rest leaves it, though measuring again would find it), or
   --  unmeasurable (a noise that stands as stored, unmeasured).
   type Noise_State is (Measured, Lost, Unmeasurable);

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
         if Noise /= Measured then
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
   --  as a float has bits, it is called dead; with its noise unmeasured, so that
   --  nothing tells whether its reading followed, it is asked the same and
   --  called blind, not dead.
   procedure Probe_Limits_And_Deadbands is
      use type Driver.Robot.Motion.Sense;
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
   --  definite.
   function Lens_Chi_Square
     (Found, Truth : Driver.Robot.Kinematics.Fit.Lens;
      Covariance   : Driver.Robot.Kinematics.Fit.Real_Lists.Vector) return Real
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      Terms : constant Natural := Natural (Sqrt (Real (Natural (Covariance.Length))));
      Error : constant Real_Vector (1 .. Fit.Lens_Terms) :=
        [Ada.Numerics.Long_Elementary_Functions.Log (Found.Fx / Truth.Fx),
         Ada.Numerics.Long_Elementary_Functions.Log (Found.Fy / Truth.Fy),
         Found.Cx - Truth.Cx, Found.Cy - Truth.Cy, Found.K1 - Truth.K1, Found.K2 - Truth.K2];
      V, L  : Real_Matrix (1 .. Fit.Lens_Terms, 1 .. Fit.Lens_Terms) := [others => [others => 0.0]];
      Ok    : Boolean;
   begin
      if Terms < Fit.Lens_Terms or else Terms * Terms /= Natural (Covariance.Length) then
         return Real'Last;
      end if;
      for P in 1 .. Fit.Lens_Terms loop
         for Q in 1 .. Fit.Lens_Terms loop
            V (P, Q) := Covariance (Covariance.First_Index + (P - 1) * Terms + Q - 1);
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
   procedure Synthetic_Sweep
     (Scale : Real; Expect_Fit : Boolean; Frame_Error : Real := 0.0; Track_Error : Real := 0.0;
      Local_Error : Real := 0.0; Pending : Boolean := False)
   is
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
            T := Inverse (Fit.Eye_At (Truth, D));
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
                        Depth : constant Real := -1.0 / Real'(Unit (Table) * H);
                        X  : constant Vec3 := Depth * H;
                        U, V : Real;
                        Ahead : Boolean;
                     begin
                        Fit.Project (Lens, T * X, U, V, Ahead);
                        U := U + Noise * Gaussian (Rng) + Shared_U + Track_U (K) + Local_U (K);
                        V := V + Noise * Gaussian (Rng) + Shared_V + Track_V (K) + Local_V (K);
                        if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 then
                           Seen := Seen + 1;
                           All_Seen (Seen) := (Frame => F, Track => K, U0 => U0, V0 => V0, U => U, V => V);
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
         Found  : Fit.Lens;
         Report : Fit.Fit_Report;
      begin
         Fit.Fit (Changes, [1 .. N => 0.01 * Scale], All_Seen (1 .. Seen), 640, 480, 0, Joints, Found, Report);
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
         --  The whole lens, its terms together: a chi square as rare as Z.
         declare
            Chi : constant Real := Lens_Chi_Square (Found, Lens, Report.Covariance);
         begin
            Check (Driver.Distributions.Chi_Square_Deviate (Chi, Fit.Lens_Terms) <= Driver.Conventions.Z,
                   "the lens is off by a chi square of" & Real'Image (Chi) & " on" & Fit.Lens_Terms'Image
                   & " terms: focal" & Real'Image (Found.Fx) & " x" & Real'Image (Found.Fy) & ", centre"
                   & Real'Image (Found.Cx) & "," & Real'Image (Found.Cy) & ", distortion" & Real'Image (Found.K1)
                   & "," & Real'Image (Found.K2));
            Driver.Log.Line (Driver.Log.Robot, "kinematics test lens: chi square" & Real'Image (Chi) & " on"
                             & Fit.Lens_Terms'Image & " terms");
         end;
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
         if Frame_Error = 0.0 then
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
      Driver.Robot.Kinematics.Solve_Pose (M, 1, Zero, Goal, False, Q, Position_Off, Turn_Off);
      Check (Position_Off < 1.0e-9 and then Turn_Off < 1.0e-9,
             "a reachable pose is missed by" & Position_Off'Image & " and" & Turn_Off'Image & " rad");
      declare
         Got : constant Rigid := Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q);
      begin
         Check (abs (Got.Translation - Goal.Translation) < 1.0e-9, "the readings found do not put the eye at the goal");
      end;
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
      Rng      : in out Generator)
   is
      package Fit renames Driver.Robot.Kinematics.Fit;
      K : Keyframe;
   begin
      K.Beat := Natural (R.Frames.Length) + 1;
      for X of Readings loop
         K.Readings.Append (X);
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
            T := Inverse (Fit.Eye_At (Truth, D));
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
         Add_Keyframe (R, Readings, Ref, Truth, Rng);
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
   procedure Scene_Point (Placed : Rigid; U, V : Real; With_Boxes : Boolean; X : out Vec3; On_Table : out Boolean) is
      H : constant Vec3 := Driver.Robot.Kinematics.Fit.Ray (Rig_Lens, U, V);
      D : constant Vec3 := Placed.Rotation * H;
      C : constant Vec3 := Placed.Translation;
      --  How far along H the plane of offset O (Table_N * X = O) is.
      function Along (O : Real) return Real is ((O - Real'(Table_N * C)) / Real'(Table_N * D));
      Best : Real := Along (Table_O);
   begin
      On_Table := True;
      if With_Boxes then
         for B of Boxes loop
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

   --  How the arms' world is drawn. The first arm's reference eye is the
   --  world; the second's stands at Second. Head, when there is one, is a
   --  third eye fixed in the world at Head_Pose that shows both arms move;
   --  each arm's reference is matched into it. The second arm's eye shows the
   --  first arm move when Wrist_Sees. A point an eye does not show is
   --  answered nonetheless when Answer_Unseen holds for that eye: where an
   --  eye at the query eye's centre, turned as the asked eye is, would see it
   --  (what a dense matcher spreads over what it cannot see); otherwise it
   --  has no answer.
   type Rig_Scene is record
      Second        : Rigid := Driver.Numerics.Identity;
      With_Boxes    : Boolean := False;
      Head          : Boolean := False;
      Head_Pose     : Rigid := Driver.Numerics.Identity;
      Wrist_Sees    : Boolean := True;
      Unseen_Wrist  : Boolean := False;   --  the second eye answers the first arm's points it does not show
      Unseen_Head   : Boolean := True;    --  the head answers the points it does not show
      Head_Noise    : Real := 0.1;        --  how far the head's answers err, pixels per coordinate
      Head_Moved    : Rigid := Driver.Numerics.Identity;   --  how far it moved before the second arm's reference
   end record;

   procedure Build_Two_Arms (M : in out Model; Scene : Rig_Scene) is
      package Fit renames Driver.Robot.Kinematics.Fit;
      N       : constant := 6;
      Columns : constant := 16;
      Rows    : constant := 12;
      Noise   : constant := 0.1;
      Changes : constant Real_Matrix := Sweep_Changes;
      Placed  : constant array (1 .. 2) of Rigid := [Driver.Numerics.Identity, Scene.Second];
      Arms    : array (1 .. 2) of Arm_Evidence;
      Rng     : Generator;
      Cells   : constant := 4;
      Eyes    : constant Positive := (if Scene.Head then 3 else 2);

      function Point_Of (A : Positive; U, V : Real) return Vec3 is
         X : Vec3;
         On_Table : Boolean;
      begin
         Scene_Point (Placed (A), U, V, Scene.With_Boxes, X, On_Table);
         return X;
      end Point_Of;

      --  From_Arm's reference query points, seen by an eye whose frame
      --  Seen_From maps From_Arm's reference frame into; Unseen: what it
      --  answers for those it does not show.
      procedure Match (Seen_From : Rigid; Frame : Positive; Eye : Natural; Into : in out Match_Set_Vectors.Vector;
                       From_Arm : Positive; Unseen : Boolean; Error : Real := Noise) is
         Set : Match_Set;
      begin
         Set.Frame := Frame;
         Set.Eye := Eye;
         for I in 0 .. Natural (Arms (From_Arm).Query_U.Length) - 1 loop
            declare
               U0    : constant Real := Arms (From_Arm).Query_U (I);
               V0    : constant Real := Arms (From_Arm).Query_V (I);
               X     : constant Vec3 := Point_Of (From_Arm, U0, V0);
               U, V  : Real;
               Ahead : Boolean;
               Shown : Boolean;
            begin
               Fit.Project (Rig_Lens, Seen_From * X, U, V, Ahead);
               Shown := Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0;
               if not Shown and then Unseen then
                  --  As from the query eye's centre: its line of sight, turned.
                  Fit.Project (Rig_Lens, Seen_From.Rotation * Fit.Ray (Rig_Lens, U0, V0), U, V, Ahead);
                  Shown := Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0;
               end if;
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
      --  Each group moves its own eye's whole view; the second eye shows the
      --  first arm move when Wrist_Sees, and the head shows both.
      for G in 1 .. 2 loop
         for E in 1 .. Eyes loop
            M.Graph.Effects.Append
              (if E = G then Eye_Effect'(Verdict => Whole, Responding => Cells, Textured => Cells, others => <>)
               elsif E = 3 or else (G = 1 and then E = 2 and then Scene.Wrist_Sees)
               then Eye_Effect'(Verdict => Patch, Responding => 1, Textured => Cells, others => <>)
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
                  D (J) := Changes (F, J);
               end loop;
               Arms (A).Frames.Append (K);
               if F > 1 then
                  Match (Inverse (Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (A), D)), F, 0, Arms (A).Matches, A, False);
               end if;
            end;
         end loop;
      end loop;
      --  The first arm's reference matched into the second's.
      Match (Inverse (Scene.Second), 1, 0, Arms (2).World_Matches, 1, Scene.Unseen_Wrist);
      Arms (2).World_Asked := True;
      Arms (2).World_Group := 1;
      Arms (2).World_Reference := Arms (1).Frames.First_Element.Beat;
      --  Each arm's reference matched into the head.
      if Scene.Head then
         for A in 1 .. 2 loop
            Match (Inverse ((if A = 2 then Scene.Head_Moved else Driver.Numerics.Identity) * Scene.Head_Pose) * Placed (A),
                   1, 3, Arms (A).Eye_Matches, A, Scene.Unseen_Head, Scene.Head_Noise);
         end loop;
      end if;
      M.Kinematics.Append (Arms (1));
      M.Kinematics.Append (Arms (2));
      Driver.Robot.Kinematics.Refit (M);
   end Build_Two_Arms;

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
               T : constant Vec3 := Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (A), D).Translation;
               G : constant Vec3 := Driver.Robot.Kinematics.Eye_In_Reference (M, Arm_Id (A), D).Translation;
            begin
               Num := Num + T * G;
               Den := Den + G * G;
            end;
         end;
      end loop;
      return Num / Den;
   end Unit_Of;

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
         function C (P, Q : Positive) return Real is (R.Placement_Covariance ((P - 1) * 6 + Q - 1));
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
            True_Eye : constant Rigid := Second * Driver.Robot.Kinematics.Fit.Eye_At (Truth_Of (2), Q);
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

   --  A second arm, its eye at its reference keyframe turned and moved from the
   --  first's and its body half again as large, its eye showing the first arm
   --  move and much of what the first eye shows: placed through its own eye's
   --  view of the first arm's table. Both lenses' errors are in it.
   procedure Place_A_Second_Arm is
      M      : Model;
      Second : constant Rigid := (Rotation => Driver.Numerics.Exp ([0.12, -0.2, 0.08]), Translation => [0.25, 0.04, 0.05]);
   begin
      Build_Two_Arms (M, (Second => Second, others => <>));
      Check_Placement (M, Second, 2, "its own eye");
   end Place_A_Second_Arm;

   --  Two arms whose eyes never see each other's table, four and a half units
   --  apart, and a head: an eye fixed above and behind them that shows both
   --  arms move and most of both tables, but not their far sides. Boxes stand
   --  on the table, about half of each view.
   Far_Second : constant Rigid := (Rotation => Driver.Numerics.Exp ([0.05, -0.1, 0.03]), Translation => [4.5, 0.05, 0.1]);

   function Head_Pose return Rigid is
      Centre : constant Vec3 := 2.25 * Table_X + 1.2 * Table_Y + Table_O * Table_N;   --  on the table, between the views
      Eye    : constant Vec3 := Centre + 3.2 * Table_N - 1.2 * Table_Y;
      Z_Axis : constant Vec3 := Unit (Centre - Eye);
      Y_Axis : constant Vec3 := Cross (Z_Axis, Table_X);
   begin
      return (Rotation    => [[Table_X (1), Y_Axis (1), Z_Axis (1)],
                              [Table_X (2), Y_Axis (2), Z_Axis (2)],
                              [Table_X (3), Y_Axis (3), Z_Axis (3)]],
              Translation => Eye);
   end Head_Pose;

   --  How many of the arm's reference points an eye's match set shows.
   function Shown (S : Match_Set) return Natural is
      K : Natural := 0;
   begin
      for F of S.Found loop
         if F then
            K := K + 1;
         end if;
      end loop;
      return K;
   end Shown;

   procedure Head_Scene (M : in out Model; Unseen_Wrist : Boolean) is
   begin
      Build_Two_Arms (M, (Second       => Far_Second,
                          With_Boxes   => True,
                          Head         => True,
                          Head_Pose    => Head_Pose,
                          Wrist_Sees   => False,
                          Unseen_Wrist => Unseen_Wrist,
                          Unseen_Head  => True,
                          others       => <>));
      --  What the scene is: the wrist views share no point, and the head does
      --  not show every table point of either arm.
      declare
         Truly : Model;
      begin
         Build_Two_Arms (Truly, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose,
                                 Wrist_Sees => False, Unseen_Wrist => False, Unseen_Head => False, others => <>));
         Check (Shown (Truly.Kinematics (2).World_Matches.First_Element) = 0,
                "the rig's wrist views share" & Shown (Truly.Kinematics (2).World_Matches.First_Element)'Image & " points");
         for A in 1 .. 2 loop
            declare
               Table, Seen : Natural := 0;
               S : constant Match_Set := Truly.Kinematics (A).Eye_Matches.First_Element;
            begin
               for I in 0 .. Natural (Truly.Kinematics (A).Query_U.Length) - 1 loop
                  declare
                     X : Vec3;
                     On_Table : Boolean;
                  begin
                     Scene_Point ((if A = 1 then Driver.Numerics.Identity else Far_Second),
                                  Truly.Kinematics (A).Query_U (I), Truly.Kinematics (A).Query_V (I), True, X, On_Table);
                     if On_Table then
                        Table := Table + 1;
                        if S.Found (I) then
                           Seen := Seen + 1;
                        end if;
                     end if;
                  end;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "head rig: arm" & A'Image & ": " & Table'Image & " of"
                                & Truly.Kinematics (A).Query_U.Length'Image & " points on the table, the head shows"
                                & Seen'Image & " of them");
               Check (Seen < Table, "the rig's head shows every table point of arm" & A'Image);
               Check (2 * Table < Natural (Truly.Kinematics (A).Query_U.Length),
                      "the rig's table holds" & Table'Image & " of arm" & A'Image & "'s points, not less than half");
            end;
         end loop;
      end;
   end Head_Scene;

   --  Placed through the head, against the truth.
   procedure Place_Through_A_Head is
      M : Model;
   begin
      Head_Scene (M, Unseen_Wrist => False);
      Check_Placement (M, Far_Second, 3, "through the head");
   end Place_Through_A_Head;

   --  The same, the second eye answering every one of the first arm's points
   --  it does not show as from the first eye's centre: one turn explains them
   --  all, which places the second eye at the first's. The head still places
   --  it right.
   procedure Place_Despite_False_Wrist_Matches is
      M : Model;
   begin
      Head_Scene (M, Unseen_Wrist => True);
      Check_Placement (M, Far_Second, 3, "through the head, the wrist answering what it does not show");
   end Place_Despite_False_Wrist_Matches;

   --  The head's answers five times as precise as the arms' own matches: the
   --  arms' lenses and tables, measured from their own sweeps, now err by
   --  more than the head's view of them does. Placed through the head, within
   --  its sigma: the link holds each arm's lens and table to its own
   --  uncertainty, not to its estimate exactly.
   procedure Place_Through_A_Precise_Head is
      M : Model;
   begin
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose,
                          Wrist_Sees => False, Head_Noise => 0.02, others => <>));
      Check_Placement (M, Far_Second, 3, "through a precise head");
   end Place_Through_A_Precise_Head;

   --  The head turned by a fiftieth of a radian and moved by a twentieth of a
   --  unit between the two arms' reference beats: its two views are no views
   --  of one plane through one eye, and the second arm is not placed.
   procedure Moved_Head_Places_Nothing is
      M : Model;
      Placement : Rigid;
      Scale : Real;
      Known : Boolean;
   begin
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose,
                          Wrist_Sees => False,
                          Head_Moved => (Rotation => Driver.Numerics.Exp ([0.02, 0.0, 0.0]), Translation => [0.0, 0.05, 0.0]),
                          others => <>));
      Driver.Robot.Kinematics.In_World (M, 2, Placement, Scale, Known);
      Check (not Known, "the second arm is placed through a head that moved between the two arms' views, off by"
             & Real'Image (abs (Placement.Translation - (1.0 / Unit_Of (M, 1)) * Far_Second.Translation)) & " units");
   end Moved_Head_Places_Nothing;

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
      Build_Two_Arms (M, (Second => Far_Second, With_Boxes => True, Head => True, Head_Pose => Head_Pose,
                          Wrist_Sees => False, others => <>));
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
   --  places its eye to a fifth of a unit): turned by a hundredth of a
   --  radian, its centre moved by a unit, its scale five per cent too large.
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
      Head_Scene (M, Unseen_Wrist => False);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      O.Readings.Append (Q);
      declare
         R2     : Arm_Fit renames M.Kinematics (2).Result;
         Tool   : constant Pose_Estimate := Tool_In_Arm (M, 2, O);
         Table  : constant Driver.Geometry.Plane_Estimate := Table_In_Arm (M, 2);
         Up_2   : constant Direction_Estimate := Up_In_Arm (M, 2);
         World  : constant Pose_Estimate := Tool_Pose (M, 2, O);
         Truth  : constant Rigid := Fit.Eye_At (Truth_Of (2), Q);
         Off    : constant Real := abs (Tool.Pose.Translation - (1.0 / Unit_Of (M, 2)) * Truth.Translation);
         Turned : constant Real := Driver.Numerics.Angle (Transpose (Tool.Pose.Rotation) * Truth.Rotation);
         Goal   : constant Motion.Pose_Goal :=
           (Pose          => (Rotation    => Tool.Pose.Rotation * Driver.Numerics.Exp ([0.0, 0.02, 0.0]),
                              Translation => Tool.Pose.Translation + [0.02, -0.01, 0.0]),
            Position_Only => False);
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
                          Translation => R2.Placement.Translation + [1.0, 0.0, 0.0]);
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
               Check (Motion.Status (Motion.Plan_Reach (M, 2, O, (Pose => World.Pose, Position_Only => False)))
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
      Head_Scene (M, Unseen_Wrist => False);
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      O.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      Want.Readings.Append (Real_Array'(1 .. 6 => 0.0));
      Want.Readings.Append (Q_Goal);
      declare
         Goal : constant Rigid := Tool_Pose (M, 2, Want).Pose;
         P    : constant Motion.Plan := Motion.Plan_Reach (M, 2, O, (Pose => Goal, Position_Only => False));
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
      M.Groups (1).Free_Shortfalls.Append (1.0 / 3.0);
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
           (Pose => Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Q_Goal), Position_Only => False);
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
           (Pose => Driver.Robot.Kinematics.Eye_In_Reference (M, 1, Beyond_Goal), Position_Only => False);
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
            Add_Keyframe (M.Kinematics (1), Q, Base, Truth, Rng);
         end;
      end loop;
      Driver.Robot.Kinematics.Refit (M);
      Check (Driver.Robot.Kinematics.Fitted (M, 1), "the arm is not fitted with the keyframes taken after its fit");
      Check (M.Kinematics (1).Result.Matches = Natural (M.Kinematics (1).Matches.Length)
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
               Sum := Sum + Fit.Eye_At (Joints, D).Translation * Fit.Eye_At (Joints, D).Translation;
            end;
         end loop;
         Check_Close (Sqrt (Sum / Real (First_Frames)), 1.0, 1.0e-9,
                      "the root mean square of the eye's positions over the keyframes of the first fit");
      end;
   end Unit_Holds_As_Keyframes_Arrive;

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

   procedure Register is
   begin
      Driver.Robot.Kinematics.Errors.Tests.Register;
      Driver.Tests.Register ("robot.estimate.task", "an estimate over a long history fails in a task with the default "
                             & "stack, as the decider's does", Estimate_In_A_Task'Access);
      Driver.Tests.Register ("robot.probe.limits", "a channel at its limit one way is asked ever further that way though "
                             & "it answered the other way (its noise lost or not), a deadband is not found however much "
                             & "wider than what the rest of the body needed, a channel that answers neither way is called "
                             & "dead before every level was asked each way, or one whose noise is unmeasured is called dead",
                             Probe_Limits_And_Deadbands'Access);
      Driver.Tests.Register ("robot.probe.droop", "a probe calls a joint at its end when the fraction of each offset "
                             & "it delivers shrinks, though it still follows", Probe_A_Drooping_Joint'Access);
      Driver.Tests.Register ("robot.reach", "the readings that put an arm's eye at a pose are not found",
                             Reach_A_Pose'Access);
      Driver.Tests.Register ("robot.still.reloaded", "a reloaded body creeping uncommanded below its visible step is "
                             & "not still, or Settle or Hold_For_Keyframe does not end on it", Reloaded_Creep_Is_Still'Access);
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
      Driver.Tests.Register ("robot.kinematics.resolution", "the final refinement stops while a step still moves a "
                             & "combination of the parameters by more than Unchanged_Fraction of its standard error, "
                             & "because the step is a small part of the cost",
                             Refinement_Takes_Steps_That_Move_The_Fit'Access);
      Driver.Tests.Register ("robot.kinematics.pending", "a joint whose widest keyframes have no sightings yet (their "
                             & "matches have not come back) raises in the search of its axis, or leaves the arm "
                             & "unfitted", Kinematics_With_Matches_Pending'Access);
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
      Driver.Tests.Register ("robot.estimate.recorded", "a decider's Estimate_Now is not in the recording once per "
                             & "call, or Observe's own recomputations are", Estimate_In_The_Recording'Access);
      Driver.Tests.Register ("robot.world.place", "a second arm whose eye sees the first arm's table is placed in the "
                             & "wrong spot, turn or scale, beyond its own sigma, or not through its own eye",
                             Place_A_Second_Arm'Access);
      Driver.Tests.Register ("robot.world.head", "two arms whose eyes never see each other's table are not placed "
                             & "through a fixed eye that sees both, or beyond the placement's own sigma",
                             Place_Through_A_Head'Access);
      Driver.Tests.Register ("robot.world.false", "answers for points the second eye does not show, as from the first "
                             & "eye's centre, place the second arm there instead of through the fixed eye",
                             Place_Despite_False_Wrist_Matches'Access);
      Driver.Tests.Register ("robot.world.precise", "a head more precise than the arms' own fits does not place the "
                             & "second arm within its sigma: the arms' lenses and tables are taken as exact",
                             Place_Through_A_Precise_Head'Access);
      Driver.Tests.Register ("robot.world.moved", "a head that moved between the two arms' views places the second arm",
                             Moved_Head_Places_Nothing'Access);
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
      Driver.Tests.Register ("robot.boot.keeps", "a boot that fails after recognizing the body leaves no body file to "
                             & "reload it from", Boot_Keeps_What_It_Measured'Access);
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
      Driver.Tests.Register ("robot.settle.patch", "a patch of an eye's picture that flickers at rest moves the beat "
                             & "the body settles at", Settle_Past_A_Patch_Flickering_At_Rest'Access);
      Driver.Tests.Register ("robot.keyframe.jitter", "an arm held away from rest whose reading jitters more than it did "
                             & "at rest gives no keyframe though its eye is still", Keyframe_Despite_Held_Jitter'Access);
      Driver.Tests.Register ("robot.roles.undecided", "a group some eye is still undecided about is called a closer or a "
                             & "part, though that eye may ride on it", Undecided_Eye_Leaves_Group_Unclassified'Access);
      Driver.Tests.Register ("robot.roles.noisy", "an eye that a group carries is called a patch of it because part of its "
                             & "picture is too faintly textured for its cells to tell the group's push, or a patch "
                             & "among well-textured cells is called the whole picture",
                             Carried_Eye_Partly_Textureless'Access);
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
