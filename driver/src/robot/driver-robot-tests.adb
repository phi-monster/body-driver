with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Observations;
with Driver.Beats;
with Driver.Log;
with Driver.Robot.Boot;
with Driver.Robot.Kinematics;
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

   procedure Boot_From_Zero is
      M    : Model;
      H    : Driver.Robot.Hand.Hands;
      Done : Boolean := False with Atomic;
      Ok   : Boolean := False with Atomic;

      task Decider;
      task body Decider is
         Fine : Boolean;
      begin
         Boot.Run (M, H, "", Fine);
         Ok := Fine;
         Done := True;
      exception
         when others =>
            Driver.Beats.Release;
            Done := True;
      end Decider;

      Now, Shown : Rig_State;
      Sent  : Driver.Commands.Command;
      Beats : Natural := 0;
      --  As many beats as the boot may take: every channel of the rig probed
      --  from the resolution of one reading unit, pushed both ways, and swept.
      Bound : constant := 20_000;
   begin
      begin
      for B in 0 .. Bound loop
         exit when Done;
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
                  O.Images.Append (Render (E, Drawn));
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
            loop
               Driver.Beats.Offer (O.Beat, O, Sent, Took);
               exit when Took or else Done;
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
      if not Done then
         abort Decider;
      end if;
      Check (Done, "the boot did not finish within" & Bound'Image & " beats");
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

   --  ── The kinematics of a synthetic arm ──
   --
   --  Six turning joints carry an eye of 640 x 480 pixels with a focal length
   --  of 400 pixels over a slanted table. The arm turns every joint alone both
   --  ways by 0.05, 0.1 and 0.2 radians, then all of them in seven cells whose
   --  signs are the rows of a Sylvester-Hadamard matrix; a 16 x 12 grid of the
   --  reference view is followed into every keyframe with 0.2 pixels of noise.

   procedure Synthetic_Sweep (Scale : Real; Expect_Fit : Boolean) is
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
                     U := U + Noise * Gaussian (Rng);
                     V := V + Noise * Gaussian (Rng);
                     if Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0 then
                        Seen := Seen + 1;
                        All_Seen (Seen) := (Frame => F, Track => (Gy - 1) * Columns + Gx, U0 => U0, V0 => V0, U => U, V => V);
                     end if;
                  end;
               end loop;
            end loop;
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
         Check_Close (Found.Fx, Lens.Fx, Lens.Fx * Noise / 40.0, "the focal length across");
         Check_Close (Found.Fy, Lens.Fy, Lens.Fy * Noise / 40.0, "the focal length down");
         for J in 1 .. N loop
            Check (Arccos (Real'Min (1.0, Joints (J).W * Truth (J).W)) < Noise / 40.0,
                   "joint" & J'Image & "'s axis is off by"
                   & Real'Image (Arccos (Real'Min (1.0, Joints (J).W * Truth (J).W))) & " rad");
         end loop;
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
            Check (abs (Scale * Got.Translation - Want.Translation) < Scale * Noise / 40.0,
                   "the eye at a new pose is off by" & Real'Image (abs (Scale * Got.Translation - Want.Translation) / Scale)
                   & " of the arm's reach");
            Check (Driver.Numerics.Angle (Transpose (Got.Rotation) * Want.Rotation) < Noise / 40.0,
                   "the eye at a new pose is turned by"
                   & Real'Image (Driver.Numerics.Angle (Transpose (Got.Rotation) * Want.Rotation)) & " rad");
         end;
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

   procedure Register is
   begin
      Driver.Tests.Register ("robot.reach", "the readings that put an arm's eye at a pose are not found, or are "
                             & "found beyond the range the arm moved through", Reach_A_Pose'Access);
      Driver.Tests.Register ("robot.kinematics.small", "a sweep too small to determine the lens and the joints is "
                             & "reported fitted", Kinematics_Of_A_Small_Sweep'Access);
      Driver.Tests.Register ("robot.kinematics", "the arm's axes, the eye's lens or the eye's pose at a new pose come "
                             & "out wrong from noisy matches of single-joint and Hadamard keyframes",
                             Kinematics_Of_A_Synthetic_Arm'Access);
      Driver.Tests.Register ("robot.boot", "the boot does not finish, deadlocks with the main loop, or does not "
                             & "recognize the rig's groups when it pushes them itself", Boot_From_Zero'Access);
      Driver.Tests.Register ("robot.steps.jitter", "a push never ends when the held reading jitters more than it did at "
                             & "rest", Step_Ends_Despite_New_Jitter'Access);
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
      Driver.Tests.Register ("robot.keyframe.jitter", "an arm held away from rest whose reading jitters more than it did "
                             & "at rest gives no keyframe though its eye is still", Keyframe_Despite_Held_Jitter'Access);
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
