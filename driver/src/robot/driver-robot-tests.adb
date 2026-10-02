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
with Driver.Robot.Channels;
with Driver.Robot.Hand;
with Driver.Robot.Flow;
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
         if Kind = Lockstep then
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

   procedure Register is
   begin
      Driver.Tests.Register ("robot.boot", "the boot does not finish, deadlocks with the main loop, or does not "
                             & "recognize the rig's groups when it pushes them itself", Boot_From_Zero'Access);
      Driver.Tests.Register ("robot.steps", "a free push that falls as short as free pushes do is called blocked, a "
                             & "push stopped by an obstacle or never answered is called free, or the wait for an "
                             & "answer is not the measured delay", Step_Responses'Access);
      Driver.Tests.Register ("robot.stillness", "an eye with ordinary camera noise never comes to rest, or a moving "
                             & "patch goes unnoticed", Eye_Stillness'Access);
      Driver.Tests.Register ("robot.roles", "a group is given the wrong role, an eye the wrong mount or lag, an arm "
                             & "is credited with a lockstep partner's eye, a reaction to another push is taken for "
                             & "a push, the tail of a slow response is taken for rest, or the step an eye can see is "
                             & "misjudged", Roles_Of_A_Synthetic_Body'Access);
      Driver.Tests.Register ("robot.channels", "reading noise is misjudged (a reading that mostly repeats exactly is "
                             & "given noise zero, so its jitter passes for motion), a hold is taken for a push, or a "
                             & "push never ends", Channel_Noise_And_Pushes'Access);
      Driver.Tests.Register ("robot.flow", "a cell's displacement between two frames is misestimated or a flat cell "
                             & "reports one", Flow_Recovers_Shifts'Access);
      Driver.Tests.Register ("robot.regression", "wild observations or collinear regressors bend the robust fit, "
                             & "or a block test calls noise significant", Regression_Ignores_Outliers'Access);
   end Register;

end Driver.Robot.Tests;
