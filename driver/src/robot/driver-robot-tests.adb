with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Observations;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Regression;
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
         Flow.Displacements (G, A, B, Quantization, Du, Dv, Cond);
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
      A := [others => 100.0];
      Flow.Displacements (G, A, A, Quantization, Du, Dv, Cond);
      Check (Cond (1) = 0.0 and then Du (1) = 0.0, "a flat cell has no displacement and no condition");
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
   end record;

   --  One beat: the robot reaches the targets of Sent, reports, and the model observes.
   procedure Step (M : in out Model; R : in out Rig; Target : Rig_State) is
      O    : Observation;
      Sent : Driver.Commands.Command;
   begin
      R.Previous := R.Now;
      R.Shown := R.Now;
      R.Now := Target;
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

   type Push_Kind is (Arm_1, Arm_2, Closer, Part, Idle, Lockstep);

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
      end case;
      for T in 1 .. Times loop
         Step (M, R, Away);
         Step (M, R, Away);
         Step (M, R, Away);
         Step (M, R, Rest);
         Step (M, R, Rest);
         Step (M, R, Rest);
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

   procedure Roles_Of_A_Synthetic_Body is
      M : Model;
   begin
      Exercise_Rig (M);
      Check (Role (M, 1) = Arm, "arm 1 is an arm, got " & Role (M, 1)'Image);
      Check (Role (M, 2) = Arm, "arm 2 is an arm, got " & Role (M, 2)'Image);
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

   procedure Register is
   begin
      Driver.Tests.Register ("robot.stillness", "an eye with ordinary camera noise never comes to rest, or a moving "
                             & "patch goes unnoticed", Eye_Stillness'Access);
      Driver.Tests.Register ("robot.roles", "a group is given the wrong role, an eye the wrong mount or lag, an arm "
                             & "is credited with a lockstep partner's eye, or a reaction to another push is taken "
                             & "for a push", Roles_Of_A_Synthetic_Body'Access);
      Driver.Tests.Register ("robot.channels", "reading noise is misjudged (a reading that mostly repeats exactly is "
                             & "given noise zero, so its jitter passes for motion), a hold is taken for a push, or a "
                             & "push never ends", Channel_Noise_And_Pushes'Access);
      Driver.Tests.Register ("robot.flow", "a cell's displacement between two frames is misestimated or a flat cell "
                             & "reports one", Flow_Recovers_Shifts'Access);
      Driver.Tests.Register ("robot.regression", "wild observations or collinear regressors bend the robust fit, "
                             & "or a block test calls noise significant", Regression_Ignores_Outliers'Access);
   end Register;

end Driver.Robot.Tests;
