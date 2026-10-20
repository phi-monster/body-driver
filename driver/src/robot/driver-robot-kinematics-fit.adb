with Ada.Strings.Fixed;
with Ada.Calendar;
with Ada.Environment_Variables;
with Ada.Long_Float_Text_IO;
with Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Robot.Regression;
with Driver.Robot.Kinematics.Errors;
with Driver.Alignment;
with Driver.Stats;
with Driver.Uncertain;
with Driver.Workers;

package body Driver.Robot.Kinematics.Fit is

   --  Everything sized by sightings, tracks, keyframes or points lives on
   --  the heap: the fit runs in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   type Flags is array (Positive range <>) of Boolean;
   type Flags_Access is access Flags;
   procedure Free is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
   type Count_Access is access Count_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Count_Array, Count_Access);
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);
   type Grid_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Grid_Access);
   type Flag_Grid is array (Positive range <>, Positive range <>) of Boolean;
   type Flag_Grid_Access is access Flag_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Grid, Flag_Grid_Access);

   ---------------------------------------------------------------------------
   --  Geometry

   --  Two unit vectors that make a right-handed frame with W.
   procedure Perp (W : Vec3; E1, E2 : out Vec3) is
      A : constant Vec3 := (if abs W (1) <= abs W (2) and then abs W (1) <= abs W (3) then [1.0, 0.0, 0.0]
                            elsif abs W (2) <= abs W (3) then [0.0, 1.0, 0.0]
                            else [0.0, 0.0, 1.0]);
   begin
      E1 := Unit (Cross (W, A));
      E2 := Cross (W, E1);
   end Perp;

   procedure Across (W : Vec3; E1, E2 : out Vec3) is
   begin
      Perp (W, E1, E2);
   end Across;

   function Rot (W : Vec3; Theta : Real) return Mat3 is (Driver.Numerics.Exp (Scaled (Theta, W)));

   function Dot (A, B : Vec3) return Real renames Driver.Numerics.Dot;

   function Eye_At (J : Joint_Array; Chain : Chain_Order; Change : Real_Array) return Rigid is
      T : Rigid := Identity;
   begin
      for Place of Chain loop
         declare
            A     : Joint renames J (J'First + Place - 1);
            Theta : constant Real := A.C * Change (Change'First + Place - 1);
            Step  : Rigid;
         begin
            if A.Slide then
               Step := (Rotation => Identity3, Translation => [Theta * A.W (1), Theta * A.W (2), Theta * A.W (3)]);
            else
               Step.Rotation := Rot (A.W, Theta);
               Step.Translation := Minus (A.P, Times (Step.Rotation, A.P));
            end if;
            T := T * Step;
         end;
      end loop;
      return T;
   end Eye_At;

   --  A joint's step at a reading change: its turn about its axis through its point, or its slide along it.
   function Step_Of (A : Joint; Change : Real) return Rigid is
      Theta : constant Real := A.C * Change;
   begin
      if A.Slide then
         return (Rotation => Identity3, Translation => [Theta * A.W (1), Theta * A.W (2), Theta * A.W (3)]);
      end if;
      declare
         R : constant Mat3 := Rot (A.W, Theta);
      begin
         return (Rotation => R, Translation => Minus (A.P, Times (R, A.P)));
      end;
   end Step_Of;

   --  How a point of eye A's frame appears in eye B's: X_B = R X_A + T.
   procedure Relative (A, B : Rigid; R : out Mat3; T : out Vec3) is
   begin
      R := Times (Transposed (B.Rotation), A.Rotation);
      T := Transposed_Times (B.Rotation, Minus (A.Translation, B.Translation));
   end Relative;

   --  The Sampson residual of the lines of sight H1 (eye A) and H2 (eye B)
   --  under X_B = R X_A + T, in the units of a pixel at focal length F.
   function Sampson (R : Mat3; T : Vec3; H1, H2 : Vec3; F : Real) return Real is
      Y    : constant Vec3 := Times (R, H1);
      Ex1  : constant Vec3 := Cross (T, Y);
      Etx2 : constant Vec3 := Transposed_Times (R, Cross (H2, T));
      Den  : constant Real := Sqrt (Ex1 (1) ** 2 + Ex1 (2) ** 2 + Etx2 (1) ** 2 + Etx2 (2) ** 2);
   begin
      return (if Den > 0.0 then F * Dot (H2, Ex1) / Den else 0.0);
   end Sampson;

   --  How far eye B saw the point from where the model allows it, in pixels at focal length F: under
   --  X_B = R X_A + T, the point of eye A's line of sight H1 appears in eye B somewhere on a half-line of its
   --  picture, at the line's vanishing point V (the rotated line of sight, the point infinitely far) when it is
   --  infinitely far, and from there along the direction D of the epipole as its depth comes down. The residual
   --  is the distance from H2 to that half-line: the perpendicular to it where H2 projects onto it at a positive
   --  depth, and the distance to V itself where H2 lies beyond V (a depth the model has none for). The Sampson
   --  residual measures the distance to the whole epipolar line, both ways from V: an eye that turns without
   --  travelling (a wrist on a short lever, small turns) keeps its points on their lines under any translation
   --  once its rotation is right, and a wrong focal length that bends the rotation's field moved V along the
   --  line without cost, so the focal search found 27, 57 and 24453 px on the kiwi (R6's starts of 10-14) at
   --  scores below the true lens's. Where parallax is there the two measures agree.
   --  Two residuals a sighting. Du: the distance to the whole epipolar line, signed by its side (Sampson), which
   --  is what tells the lens where the eye travels (a long lever: the synthetic arms' searches land at 394 and
   --  409 px of 400 on it, and on the distance to the half-line alone their curves lay flat from 6 to 450 px,
   --  10-14). Dv: how far the sighting lies behind the line's vanishing point along it, in pixels, zero where it
   --  lies at a positive depth: the one-sided penalty that tells the lens where the eye hardly travels (the
   --  kiwi's wrist on its short lever: the Sampson residual alone let a wrong lens move V along the line at no
   --  cost, 27 to 24453 px), and that is all a sighting costs there, since with no translation the half-line is
   --  a point and the distance to V is the reprojection error of the rotation.
   procedure Reprojected (R : Mat3; T : Vec3; H1, H2 : Vec3; F : Real; Du, Dv : out Real) is
      Y  : constant Vec3 := Times (R, H1);
   begin
      Du := Sampson (R, T, H1, H2, F);
      Dv := 0.0;
      if Y (3) <= 0.0 or else H2 (3) <= 0.0 then
         --  A line of sight turned behind eye B: as far off as the lines of sight are long.
         Dv := F * Sqrt (Dot (Y, Y) + Dot (H2, H2));
         return;
      end if;
      declare
         Vx  : constant Real := Y (1) / Y (3);
         Vy  : constant Real := Y (2) / Y (3);
         Hx  : constant Real := H2 (1) / H2 (3);
         Hy  : constant Real := H2 (2) / H2 (3);
         --  The way the point's image moves from V as its depth comes down from infinity: towards the epipole.
         Dx  : constant Real := T (1) - Vx * T (3);
         Dy  : constant Real := T (2) - Vy * T (3);
         Dd  : constant Real := Dx * Dx + Dy * Dy;
         Ex  : constant Real := Hx - Vx;
         Ey  : constant Real := Hy - Vy;
      begin
         --  The penalty for a sighting behind V, F * (-Along) * Sqrt (Dd) where Along < 0, is not charged: at
         --  the true lens every sighting at no parallax (a far point, Along 0 within the noise) lies behind V
         --  half the time and pays the noise, while a wrong lens that moves every V back along its line pays
         --  nothing, so the penalty is biased against the truth wherever far points abound (the head rig's seed
         --  1 arm, 10-14: 73 px of 400 with it, 377 to 409 without). The no-parallax regime (the kiwi's wrist on
         --  its short lever, where Dd is nearly zero and the distance to the line says nothing of the lens)
         --  needs a penalty charged only beyond the noise, to be validated on the fake robot's short levers
         --  against its truth before it is charged at all. Until then the sighting with no translation costs
         --  its distance to V, the reprojection error of the rotation.
         if Dd = 0.0 then
            Dv := F * Sqrt (Ex * Ex + Ey * Ey);
         end if;
      end;
   end Reprojected;

   function Reprojected (R : Mat3; T : Vec3; H1, H2 : Vec3; F : Real) return Real is
      Du, Dv : Real;
   begin
      Reprojected (R, T, H1, H2, F, Du, Dv);
      return Sqrt (Du * Du + Dv * Dv);
   end Reprojected;

   --  One keyframe's signed distances less what its sightings share: their median over the keyframe. A shift of
   --  the whole picture, from a small turn the model has no term for or from an error the aligner gives every
   --  point of a keyframe alike, says nothing of the lens or the axis; what differs between the points (the
   --  field's bend, the parallax) does (robot.kinematics.shared: every keyframe's points share an error, and the
   --  search of stage 1 bent the lens to 123 px to meet it). The penalty for a sighting behind its vanishing
   --  point (Reprojected's Dv) is one-sided and is not shared out.
   procedure Share_Out (Du : in out Real_Array) is
      Copy : Real_Array := Du;   --  a keyframe's sightings: on the stack
      Mu   : constant Real := Driver.Stats.Median_In_Place (Copy);
   begin
      for I in Du'Range loop
         Du (I) := Du (I) - Mu;
      end loop;
   end Share_Out;

   ---------------------------------------------------------------------------
   --  The lens

   function Ray (L : Lens; U, V : Real) return Vec3 is
      Xd : constant Real := (U - L.Cx) / L.Fx;
      Yd : constant Real := (V - L.Cy) / L.Fy;
      X  : Real := Xd;
      Y  : Real := Yd;
   begin
      --  Undistort along the pixel's direction from the centre: its undistorted radius R has R (1 + K1 R^2 + K2 R^4)
      --  the distorted one, found by Newton's method from the distorted one, as many steps at most as the float has
      --  bits (a lens whose distortion is large is slow to a fixed point and never settles to one that is not a
      --  contraction), and no further where the lens turns back (its slope no longer positive).
      if L.K1 /= 0.0 or else L.K2 /= 0.0 then
         declare
            Rd : constant Real := Sqrt (Xd * Xd + Yd * Yd);
            R  : Real := Rd;
         begin
            for Step in 1 .. Real'Machine_Mantissa loop
               declare
                  R2    : constant Real := R * R;
                  D     : constant Real := 1.0 + L.K1 * R2 + L.K2 * R2 * R2;
                  Miss  : constant Real := R * D - Rd;
                  Slope : constant Real := D + R * (2.0 * L.K1 * R + 4.0 * L.K2 * R2 * R);
               begin
                  exit when Miss = 0.0 or else Slope <= 0.0;
                  R := R - Miss / Slope;
                  exit when abs (Miss / Slope) <= Real'Epsilon * Real'Max (1.0, abs R);
               end;
            end loop;
            if Rd > 0.0 then
               X := Xd * R / Rd;
               Y := Yd * R / Rd;
            end if;
         end;
      end if;
      return [X, Y, 1.0];
   end Ray;

   procedure Project (L : Lens; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      In_Front := P (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if In_Front then
         declare
            X  : constant Real := P (1) / P (3);
            Y  : constant Real := P (2) / P (3);
            R2 : constant Real := X * X + Y * Y;
            D  : constant Real := 1.0 + L.K1 * R2 + L.K2 * R2 * R2;
         begin
            U := L.Fx * X * D + L.Cx;
            V := L.Fy * Y * D + L.Cy;
         end;
      end if;
   end Project;

   ---------------------------------------------------------------------------
   --  Statistics of residuals

   --  The median of the absolute values, on the heap: there may be many.
   function Median_Abs (R : Real_Array) return Real is
   begin
      if R'Length = 0 then
         return 0.0;
      end if;
      declare
         A : Real_Access := new Real_Array (1 .. R'Length);
         M : Real;
      begin
         for I in 0 .. R'Length - 1 loop
            A (I + 1) := abs R (R'First + I);
         end loop;
         M := Driver.Stats.Median (A.all);
         Free (A);
         return M;
      end;
   end Median_Abs;

   --  The normal's upper quartile, the median of a unit Gaussian's absolute value: computed once (the search of
   --  stage 1 asked for it twice for every direction it scanned, 6% of the fit's time in its profile of 10-15).
   Upper_Quartile : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5);

   --  The noise of residuals that have no sign of their own: the robust scale
   --  about zero, the median absolute residual over the normal's upper
   --  quartile (what Robust_Sigma gives the residuals with their negations).
   function Noise_Of (R : Real_Array) return Real is
     (Median_Abs (R) / Upper_Quartile);

   --  The standard error of the median of N absolute residuals of Gaussian
   --  noise whose median is Median: the median of |r| is q sigma with q the
   --  normal's upper quartile, and a sample median varies by
   --  1 / (2 sqrt (N) f) with f the density there, 2 phi (q) / sigma.
   --  Whether the sightings determine a joint, by the variances of its axis's tilt along the two directions across
   --  it and of its reading scale C: the axis known to a cone whose Z-sigma edge stays within a quarter turn of it,
   --  and the scale significant against its uncertainty.
   function Joint_Determined (Tilt_1, Tilt_2, Scale_Variance, C : Real) return Boolean is
     (Driver.Conventions.Z * Sqrt (Tilt_1 + Tilt_2) < Ada.Numerics.Pi / 2.0
      and then Driver.Uncertain.Significant (C, Sqrt (Scale_Variance)));

   --  Whether a lens's aspect and distortion (the logarithm of Fy over Fx, K1, K2) are significant together by the
   --  fit's covariance (R.Covariance, which holds how the sightings' errors depend on each other): a chi square on
   --  the three as rare as Z. Not when the covariance is not the fit's of N joints.
   function Extras_Significant (L : Lens; R : Fit_Report; N : Natural) return Boolean is
      Reduced : constant Natural := Terms (N);
   begin
      if Natural (R.Covariance.Length) /= Reduced * Reduced then
         return False;
      end if;
      declare
         function V (P, Q : Positive) return Real is
           (R.Covariance (R.Covariance.First_Index + (P - 1) * Reduced + Q - 1));
         X  : constant Real_Vector (1 .. 3) := [Ln (L.Fy / L.Fx), L.K1, L.K2];
         C  : Driver.Numerics.Arrays.Real_Matrix (1 .. 3, 1 .. 3);
         Lf : Driver.Numerics.Arrays.Real_Matrix (1 .. 3, 1 .. 3);
         Pd : Boolean;
      begin
         --  The aspect's logarithm is the second focal length's less the first's.
         C (1, 1) := V (1, 1) + V (2, 2) - 2.0 * V (1, 2);
         C (1, 2) := V (2, 5) - V (1, 5);
         C (1, 3) := V (2, 6) - V (1, 6);
         C (2, 2) := V (5, 5);
         C (2, 3) := V (5, 6);
         C (3, 3) := V (6, 6);
         C (2, 1) := C (1, 2);
         C (3, 1) := C (1, 3);
         C (3, 2) := C (2, 3);
         Driver.Numerics.Dense.Cholesky (C, Lf, Pd);
         if not Pd then
            return False;
         end if;
         declare
            Y   : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, X);
            Chi : Real := 0.0;
         begin
            for I in X'Range loop
               Chi := Chi + X (I) * Y (I);
            end loop;
            return Chi > 0.0 and then Driver.Distributions.Chi_Square_Deviate (Chi, X'Length) > Driver.Conventions.Z;
         end;
      end;
   end Extras_Significant;

   function Median_Error (Median : Real; Count : Positive) return Real is
      Q     : constant Real := Upper_Quartile;
      Sigma : constant Real := Median / Q;
      F     : constant Real := 2.0 * Exp (-Q * Q / 2.0) / Sqrt (2.0 * Ada.Numerics.Pi);
   begin
      return Sigma / (2.0 * Sqrt (Real (Count)) * F);
   end Median_Error;

   function Scale_Of (Model : Error_Model; Own : Real) return Real is
     (Real'Max (Sqrt (Model.Floor ** 2 + (Model.Gain * Own) ** 2), Real'Model_Small));

   --  What the matcher states of a sighting rests on up to three other fits of its patch (the aligner's variants) and
   --  says little of how far the sighting errs: against the fake robot's truth (10-19), the sightings stating under a
   --  quarter of a pixel erred a median 0.37 to 0.64 px, those stating 1 to 2 px 0.66 to 2.1 px (w18's two arms and
   --  q15's second). How much of it the residuals bear out is read beside the floor, and both, read from thousands of
   --  residuals, are judged as known. Judged beside a floor alone at Student's t of the variants' three degrees of
   --  freedom (nine scales at Z), a sighting stating 1.4 px stood 13 px off its fit and was kept; the tracks' fits kept
   --  76 and 91 % of the sightings the truth puts more than 4 px off (w18's arm 1, q15's arm 2) and dropped 0.6 and
   --  5.8 % of those within 1.5 px; at the floor and gain read so (gains of 0.50 and none), 46 and 45 % kept and 3.1
   --  and 1.7 % dropped, and w18's arm 2, its order untold among 17, untold among 2 (its dumped fits replayed).
   --  At a gain, the floor is a median: a residual lies within Typical scales when the part of its square, over
   --  Typical's, that the gain times what is stated leaves is at most the floor's square, and half of them do at the
   --  median of those parts. The gain is then halved toward the one at which the half that states the more lies at
   --  Typical scales by its median: the larger the gain, the larger that half's scales against the rest's, whose
   --  median the floor holds.
   function Model_Of (R, Own : Real_Array) return Error_Model is
      Typical : constant Real := Upper_Quartile;
      N       : constant Natural := R'Length;
      Cut     : Real := 0.0;   --  the median of what is stated: the half that states the more stands at or above it
      Part    : Real_Access;

      function Floor_At (Gain : Real) return Real is
      begin
         for I in 1 .. N loop
            Part (I) := (R (R'First + I - 1) / Typical) ** 2 - (Gain * Own (Own'First + I - 1)) ** 2;
         end loop;
         return Sqrt (Real'Max (0.0, Driver.Stats.Median (Part.all)));
      end Floor_At;

      --  Whether the half that states the more lies beyond Typical scales by its median, at Gain and its floor.
      function Above_Typical (Gain : Real) return Boolean is
         Model   : constant Error_Model := (Floor => Floor_At (Gain), Gain => Gain);
         At_Most : Natural := 0;
         Total   : Natural := 0;
      begin
         for I in 1 .. N loop
            declare
               O : constant Real := Own (Own'First + I - 1);
            begin
               if O > 0.0 and then O >= Cut then
                  Total := Total + 1;
                  if abs R (R'First + I - 1) <= Typical * Scale_Of (Model, O) then
                     At_Most := At_Most + 1;
                  end if;
               end if;
            end;
         end loop;
         return 2 * At_Most < Total;
      end Above_Typical;

      Low, High : Real := 0.0;
      Result    : Error_Model;
   begin
      if N = 0 then
         return (Floor => 0.0, Gain => 1.0);
      elsif (for all O of Own => O = 0.0) then
         return (Floor => Noise_Of (R), Gain => 1.0);
      end if;
      Cut := Driver.Stats.Median (Own);
      Part := new Real_Array (1 .. N);
      if Above_Typical (0.0) then
         High := 1.0;
         while Above_Typical (High) loop
            Low := High;
            High := 2.0 * High;
         end loop;
         for Step in 1 .. Real'Machine_Mantissa loop
            declare
               Middle : constant Real := (Low + High) / 2.0;
            begin
               exit when Middle <= Low or else Middle >= High;
               if Above_Typical (Middle) then
                  Low := Middle;
               else
                  High := Middle;
               end if;
            end;
         end loop;
      end if;
      Result := (Floor => Floor_At (High), Gain => High);
      Free (Part);
      return Result;
   end Model_Of;

   function Huber (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then 1.0 else Driver.Conventions.Z / abs Z_Value);

   function Huber_Cost (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then 0.5 * Z_Value ** 2
      else Driver.Conventions.Z * (abs Z_Value - 0.5 * Driver.Conventions.Z));

   ---------------------------------------------------------------------------
   --  Robust least squares: Levenberg-Marquardt on a numerical Jacobian, Huber
   --  weights at Z on residuals in units of Sigma. It stops when a step lowers
   --  the cost by less than the unchanged fraction of it, or when no damping
   --  gives a step that lowers it at all; To_The_End, when a step no longer
   --  moves any combination of the parameters by more than the unchanged
   --  fraction of its standard error (the last stage of an arm's fit: stopped
   --  at the fraction of the cost, inputs apart by 1e-9 px gave fits apart by
   --  far more, A23's arm 2, item 3).

   generic
      Parameters : Positive;
      Residuals  : Positive;
      Damping    : Real;   --  what the first step starts at; the fits that cross a flat valley start high
      with procedure Evaluate (X : Real_Array; R : out Real_Array);
      At_Once    : Boolean := False;
      --  Evaluate writes nothing but R, so the Jacobian's columns may be taken
      --  at once (Driver.Parallel_For), each with its own residual array.
      To_The_End : Boolean := False;
      --  Go on while a step moves the estimate by more than the unchanged fraction of its standard error.
   procedure Robust_Fit (X : in out Real_Array; Sigma : Real);

   procedure Robust_Fit (X : in out Real_Array; Sigma : Real) is
      type Matrix_Access is access Real_Matrix;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
      R0     : Real_Access := new Real_Array (1 .. Residuals);
      Rn     : Real_Access := new Real_Array (1 .. Residuals);
      J      : Matrix_Access := new Real_Matrix (1 .. Residuals, 1 .. Parameters);
      Lambda : Real := Damping;
      Cost0  : Real;

      function Cost (R : Real_Array) return Real is
         S : Real := 0.0;
      begin
         for V of R loop
            S := S + Huber_Cost (V / Sigma);
         end loop;
         return S;
      end Cost;
   begin
      Evaluate (X, R0.all);
      Cost0 := Cost (R0.all);
      loop
         --  Forward differences at the square root of the float's resolution
         --  relative to each parameter's size.
         declare
            procedure Column (K : Positive) is
               Xp : Real_Array := X;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (X'First + K - 1));
               Rk : Real_Access := new Real_Array (1 .. Residuals);
            begin
               Xp (Xp'First + K - 1) := Xp (Xp'First + K - 1) + H;
               Evaluate (Xp, Rk.all);
               for I in 1 .. Residuals loop
                  J (I, K) := (Rk (I) - R0 (I)) / H;
               end loop;
               Free (Rk);
            end Column;
            procedure Columns is new Driver.Parallel_For (Column);
         begin
            if At_Once then
               Columns (1, Parameters);
            else
               for K in 1 .. Parameters loop
                  Column (K);
               end loop;
            end if;
         end;
         declare
            A : Real_Matrix (1 .. Parameters, 1 .. Parameters) := [others => [others => 0.0]];
            G : Real_Vector (1 .. Parameters) := [others => 0.0];
            Improved : Boolean := False;
            Lowered  : Boolean := False;
         begin
            --  The normal equations, each parameter's row at once (Driver.Parallel_For), every sum over the
            --  residuals in their order whichever task makes it, so that it is the sum one task made, to the last
            --  bit (as Refine_Tracks sums its own). One task summed every residual into every row: the stage that
            --  fits everything at once on the sightings that fit spent 2.6 s of a 60 s sample doing it while the
            --  workers waited (a13's fit of 10452 sightings replayed from its dump, 10-16).
            declare
               Weights : Real_Access := new Real_Array (1 .. Residuals);
               procedure Row (P : Positive) is
                  Sum_G : Real := 0.0;
                  Sum_A : Real_Array (P .. Parameters) := [others => 0.0];
               begin
                  for I in 1 .. Residuals loop
                     if J (I, P) /= 0.0 then
                        Sum_G := Sum_G - Weights (I) * J (I, P) * R0 (I);
                        for Q in P .. Parameters loop
                           Sum_A (Q) := Sum_A (Q) + Weights (I) * J (I, P) * J (I, Q);
                        end loop;
                     end if;
                  end loop;
                  G (P) := Sum_G;
                  for Q in P .. Parameters loop
                     A (P, Q) := Sum_A (Q);
                  end loop;
               end Row;
               procedure Rows is new Driver.Parallel_For (Row);
            begin
               for I in 1 .. Residuals loop
                  Weights (I) := Huber (R0 (I) / Sigma) / Sigma ** 2;
               end loop;
               Rows (1, Parameters);
               Free (Weights);
            end;
            for P in 1 .. Parameters loop
               for Q in 1 .. P - 1 loop
                  A (P, Q) := A (Q, P);
               end loop;
            end loop;
            --  Marquardt's damping, doubled until a step lowers the cost or
            --  the step no longer moves any parameter.
            loop
               declare
                  D  : Real_Matrix := A;
                  L  : Real_Matrix (1 .. Parameters, 1 .. Parameters);
                  Pd : Boolean;
                  Moves : Boolean := False;
               begin
                  for P in 1 .. Parameters loop
                     D (P, P) := A (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                  end loop;
                  Driver.Numerics.Dense.Cholesky (D, L, Pd);
                  if Pd then
                     declare
                        Delta_X : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, G);
                        Xn      : Real_Array := X;
                     begin
                        for P in 1 .. Parameters loop
                           Xn (Xn'First + P - 1) := X (X'First + P - 1) + Delta_X (P);
                           Moves := Moves or else Xn (Xn'First + P - 1) /= X (X'First + P - 1);
                        end loop;
                        if Moves then
                           declare
                              --  A step to a place the model cannot be evaluated at (a scale gone to infinity, an axis to
                              --  nothing: a rotation vector of NaN raises in the norm) lowers nothing. The humanoid
                              --  cold boot h3 (10-20) ended at beat 15560 on one such trial of an order's quick fit,
                              --  "the estimates failed: ARGUMENT_ERROR", and the driver closed the connection.
                              function Trial_Cost return Real is
                              begin
                                 Evaluate (Xn, Rn.all);
                                 return Cost (Rn.all);
                              exception
                                 when Ada.Numerics.Argument_Error =>
                                    return Real'Last;
                              end Trial_Cost;
                              C : constant Real := Trial_Cost;
                           begin
                              if C < Cost0 then
                                 --  To the end: a step that moves some combination of the parameters by more than
                                 --  the unchanged fraction of its standard error (the step's squared length in the
                                 --  metric of the normal equations, the inverse covariance), whatever the cost: a
                                 --  decrease of the cost is no measure of how far the estimate still moves on the
                                 --  axes the data barely determine, and any decrease at all never ends on a flat
                                 --  valley (robot.boot ground sixteen minutes in its last stage).
                                 Improved := (if To_The_End
                                              then Delta_X * (A * Delta_X) > Driver.Conventions.Unchanged_Fraction ** 2
                                              else Cost0 - C > Driver.Conventions.Unchanged_Fraction * Cost0);
                                 X := Xn;
                                 R0.all := Rn.all;
                                 Cost0 := C;
                                 Lambda := Lambda / 2.0;
                                 Lowered := True;
                              end if;
                           end;
                        end if;
                     end;
                  else
                     Moves := True;
                  end if;
                  --  No further than a float goes (as Fixed.Refine): past it the damping is infinite, the damped matrix no
                  --  number, never positive definite, and nothing would end the loop.
                  exit when Lowered or else not Moves or else Lambda >= Real'Last / 2.0;
                  Lambda := 2.0 * Lambda;
               end;
            end loop;
            exit when not Improved;
         end;
      end loop;
      Free (R0);
      Free (Rn);
      Free (J);
   end Robust_Fit;

   ---------------------------------------------------------------------------
   --  The covariance of an estimate from the spread of its gradient. The
   --  spread, summed from covariances estimated pair by pair (Errors), need
   --  not be positive semi-definite, and a covariance must be. Its negative
   --  eigenvalues are set to zero (Cameron, Gelbach and Miller) in
   --  the frame where the inverse normal equations are the identity, the
   --  estimate's own units: a unit chosen for a term (a pixel, a radian, a
   --  length of the fit) then does not move what the clip takes away.

   procedure Sandwich
     (Inverse, Meat : Driver.Numerics.Arrays.Real_Matrix;
      Covariance    : out Driver.Numerics.Arrays.Real_Matrix;
      Clipped       : out Real;
      Ok            : out Boolean;
      Floor         : Real := 0.0)
   is
      N       : constant Natural := Inverse'Length (1);
      Factor  : Real_Matrix (1 .. N, 1 .. N);
      Values  : Real_Vector (1 .. N);
      Vectors : Real_Matrix (1 .. N, 1 .. N);
   begin
      Covariance := [others => [others => 0.0]];
      Clipped := 0.0;
      Driver.Numerics.Dense.Cholesky (Inverse, Factor, Ok);
      if not Ok then
         return;
      end if;
      declare
         --  The spread in the frame where the inverse is the identity.
         Whitened : constant Real_Matrix := Transpose (Factor) * Meat * Factor;
      begin
         Eigensystem (0.5 * (Whitened + Transpose (Whitened)), Values, Vectors);
      end;
      declare
         Taken, All_Of : Real := 0.0;
      begin
         for E in 1 .. N loop
            All_Of := All_Of + abs Values (E);
            if Values (E) < 0.0 then
               Taken := Taken - Values (E);
               Values (E) := 0.0;
            end if;
            Values (E) := Real'Max (Values (E), Floor);
         end loop;
         Clipped := (if All_Of > 0.0 then Taken / All_Of else 0.0);
      end;
      declare
         Kept : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      begin
         for P in 1 .. N loop
            for Q in 1 .. N loop
               for E in 1 .. N loop
                  Kept (P, Q) := Kept (P, Q) + Vectors (P, E) * Values (E) * Vectors (Q, E);
               end loop;
            end loop;
         end loop;
         declare
            Product : constant Real_Matrix := Factor * Kept * Transpose (Factor);
         begin
            Covariance := 0.5 * (Product + Transpose (Product));   --  symmetric to the last bit
         end;
      end;
   end Sandwich;

   --  What the covariance rests on, in pixels (the root of the mean of the two
   --  components' variances).
   function Summary_Of (Kinds : Errors.Model; Clipped : Real) return Error_Summary is
      function Level (E : Errors.Entries) return Real is
        (Sqrt (Real'Max (0.0, E (Errors.Both_Across) + E (Errors.Both_Down)) / 2.0));
   begin
      return (Measured         => True,
              Alone            => Level (Errors.Alone (Kinds)),
              Persistent       => Level (Errors.Same_Track (Kinds)),
              Persistent_Half  => Errors.Half_Distance_Persistent (Kinds),
              Keyframe         => Level (Errors.Added_By_Keyframe (Kinds)),
              Clipped          => Clipped);
   end Summary_Of;

   ---------------------------------------------------------------------------
   --  Stage 5: the tracks seen from many keyframes. Every followed point has a
   --  depth along its reference ray, and every sighting must land where the
   --  model puts that point: the reprojection residuals, two per sighting,
   --  over the lens, the joints and every depth at once (the depths solved by
   --  their Schur complement, each a block of its own). Where the epipolar
   --  residuals of a pair cannot tell a flat scene's motions apart, the points
   --  seen from many keyframes can.

   type Rigid_Array is array (Positive range <>) of Rigid;
   type Rigid_Access is access Rigid_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Rigid_Array, Rigid_Access);

   --  Spread: whether the estimate's covariance is wanted (Report.Covariance, Errors, Determined). A caller that
   --  takes only the median residual or the depths (the focal length's profile, the depths of stage 3) is spared
   --  it: measuring how the sightings' errors depend on each other cost a third of every refinement's time, made
   --  for the profile's every step and never read (the fit's profile, 10-15). Without it the estimate is reported
   --  not determined.
   --  Pinhole: the lens is a pinhole with square pixels, its second focal length its first and no distortion; the
   --  terms it does not have stay out of the solution and of the covariance (Fit's stage 5 keeps them only when
   --  they are significant).
   --  Judge_Floor, Judge_Gain: the error model Report.Cost_Judged is taken at (another fit's, to compare with it);
   --  the refinement's own when the floor is 0.
   procedure Refine_Tracks
     (Changes      : Driver.Numerics.Arrays.Real_Matrix;
      Sight        : Sighting_Array;
      Joints       : in out Joint_Array;
      Chain        : Chain_Order;
      L            : in out Lens;
      Report       : in out Fit_Report;
      Focal_Within : Real := 0.0;
      Spread       : Boolean := True;
      Pinhole      : Boolean := False;
      Judge_Floor  : Real := 0.0;
      Judge_Gain   : Real := 0.0)
   is
      N         : constant Natural := Joints'Length;
      --  The focal lengths the fit is given are the focal search's (stage 1), told within Focal_Within in the
      --  logarithm: they enter the fit as a measurement of that width, two residuals of unit scale (no measurement
      --  when the width is 0). Without it the fit walked the focal length along a direction the sightings do not
      --  determine: B0a_1 (x5, 10-13), the search at 490 px and the fit ending at "focal lengths undetermined:
      --  1.84 +- 259 and 1.53 +- 215 px"; B0a_2 at 0.0004 px after 4042 s.
      Ln_Fx0    : constant Real := Ln (L.Fx);
      Ln_Fy0    : constant Real := Ln (L.Fy);
      Prior_W   : constant Real := (if Focal_Within > 0.0 then 1.0 / Focal_Within ** 2 else 0.0);
      Frames    : constant Natural := Changes'Length (1);
      S         : constant Natural := Sight'Length;
      Per_Joint : constant := 6;
      Count     : constant Positive := 6 + Per_Joint * N;
      Tracks    : Natural := 0;
      Base      : Joint_Array := Joints;

      type Real_Access is access Real_Array;
      type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
      procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);

      function Changes_Of (Frame : Positive) return Real_Array is
         D : Real_Array (1 .. N);
      begin
         for J in 1 .. N loop
            D (J) := Changes (Changes'First (1) + Frame - 1, Changes'First (2) + J - 1);
         end loop;
         return D;
      end Changes_Of;

      function Lens_Of (X : Real_Array) return Lens is
        ((Fx => Exp (X (X'First)), Fy => Exp (X (X'First + (if Pinhole then 0 else 1))), Cx => X (X'First + 2),
          Cy => X (X'First + 3), K1 => (if Pinhole then 0.0 else X (X'First + 4)),
          K2 => (if Pinhole then 0.0 else X (X'First + 5))));

      function Joints_Of (X : Real_Array) return Joint_Array is
         Out_J : Joint_Array := Base;
      begin
         for J in 1 .. N loop
            declare
               O : constant Natural := X'First + 6 + Per_Joint * (J - 1);
               E1, E2 : Vec3;
            begin
               Perp (Base (Base'First + J - 1).W, E1, E2);
               Out_J (Out_J'First + J - 1).W := Unit (Base (Base'First + J - 1).W + X (O) * E1 + X (O + 1) * E2);
               Out_J (Out_J'First + J - 1).P := [X (O + 2), X (O + 3), X (O + 4)];
               Out_J (Out_J'First + J - 1).C := X (O + 5);
            end;
         end loop;
         return Out_J;
      end Joints_Of;

      procedure To_X (Lx : Lens; Jx : Joint_Array; X : out Real_Array) is
      begin
         X (X'First .. X'First + 5) := [Ln (Lx.Fx), Ln (Lx.Fy), Lx.Cx, Lx.Cy, Lx.K1, Lx.K2];
         for J in 1 .. N loop
            declare
               O : constant Natural := X'First + 6 + Per_Joint * (J - 1);
            begin
               X (O) := 0.0;
               X (O + 1) := 0.0;
               X (O + 2) := Jx (Jx'First + J - 1).P (1);
               X (O + 3) := Jx (Jx'First + J - 1).P (2);
               X (O + 4) := Jx (Jx'First + J - 1).P (3);
               X (O + 5) := Jx (Jx'First + J - 1).C;
            end;
         end loop;
      end To_X;
   begin
      for X of Sight loop
         Tracks := Natural'Max (Tracks, X.Track);
      end loop;
      if Tracks = 0 or else N = 0 then
         return;
      end if;
      declare
         Depth  : Real_Access := new Real_Array'(1 .. Tracks => 0.0);   --  the log of each track's depth
         --  How uncertain each log depth is at the solution, the other
         --  parameters held: the root of the inverse of its own information.
         Depth_Sigma : Real_Access := new Real_Array'(1 .. Tracks => Real'Last);
         Has    : Flags_Access := new Flags'(1 .. Tracks => False);
         Inlier : Flags_Access := new Flags'(1 .. S => False);
         X      : Real_Array (1 .. Count);
         --  The sightings each pass began with: the passes end where the choice settles or returns to one it began
         --  from (a choice that cycles has no fixed point to wait for), and at most as many as the float has bits.
         type Choice_List is array (1 .. Real'Machine_Mantissa) of Flags_Access;
         Began  : Choice_List := [others => null];
         Passes : Natural := 0;
         Rounds : Natural := 0;   --  descent rounds over every pass, for the log (what the stage's time is made of)
         --  Each parameter's variance at the solution, in the reduced set: the
         --  lens (6), then per joint its tilt (2), its point across the axis
         --  (2) and its reading scale (1).
         Variance   : Real_Array (1 .. 6 + 5 * N) := [others => Real'Last];
         Determined : Boolean := False;
         --  Why not, when it is not: the last pass that could not be made, and how.
         Why_Not    : Ada.Strings.Unbounded.Unbounded_String :=
           Ada.Strings.Unbounded.To_Unbounded_String ("too few sightings fit the tracks to determine the lens and the joints");

         --  What the covariance of the last pass is made of: the gradient of
         --  every sighting (its row) and its error, the inverse normal
         --  equations, and where the sighting is (its keyframe, its point, the
         --  point's reference pixel).
         type Pass_Evidence is record
            Ready               : Boolean := False;
            Inv, Rows           : Grid_Access;
            Error, Ref_U, Ref_V : Real_Access;
            In_Frame, Of_Track  : Natural_Access;
         end record;
         Last_Pass : Pass_Evidence;

         procedure Release (E : in out Pass_Evidence) is
         begin
            Free (E.Inv);
            Free (E.Rows);
            Free (E.Error);
            Free (E.Ref_U);
            Free (E.Ref_V);
            Free (E.In_Frame);
            Free (E.Of_Track);
            E.Ready := False;
         end Release;

         --  The reprojection residuals of the chosen sightings (2 each). With Joint, only those of the keyframes that
         --  turned that joint, the rest of R left as it is: in a keyframe that did not turn it the joint's step is the
         --  identity whatever its parameters (its turn is zero, Rot gives the identity exactly, and its point less the
         --  identity's image of it is nothing), so the residuals there are the same to the bit and the Jacobian's
         --  column of the joint's parameters is zero there.
         procedure Residuals
           (Xv : Real_Array; Dv : Real_Array; Index : Real_Access; R : in out Real_Array; Joint : Natural := 0)
         is
            Lx    : constant Lens := Lens_Of (Xv);
            Jx    : constant Joint_Array := Joints_Of (Xv);
            Views : Rigid_Access := new Rigid_Array (1 .. Frames);
            --  Per track, its point at its depth along the line of sight of a reference pixel, and that pixel: the
            --  sightings of a track share its reference pixel, and its line of sight (Ray, Newton's method through
            --  the lens's distortion) and depth were made again for every sighting, a third of the time of every
            --  residual (the fit's profile, 10-15). A sighting whose reference pixel is another makes its own.
            type Point_Array is array (1 .. Tracks) of Vec3;
            type Point_Access is access Point_Array;
            procedure Free is new Ada.Unchecked_Deallocation (Point_Array, Point_Access);
            Points : Point_Access := new Point_Array;
            Made   : Flags_Access := new Flags'(1 .. Tracks => False);
            Made_U : Real_Access := new Real_Array (1 .. Tracks);
            Made_V : Real_Access := new Real_Array (1 .. Tracks);
            Turned : Flags_Access := new Flags'(1 .. Frames => True);
         begin
            if Joint > 0 then
               for F in 1 .. Frames loop
                  Turned (F) := Changes (Changes'First (1) + F - 1, Changes'First (2) + Joint - 1) /= 0.0;
               end loop;
            end if;
            for F in 1 .. Frames loop
               if Turned (F) then
                  Views (F) := Inverse (Eye_At (Jx, Chain, Changes_Of (F)));
               end if;
            end loop;
            for K in 1 .. Index'Length loop
               declare
                  Sg : Sighting renames Sight (Sight'First + Natural (Index (K)) - 1);
                  U, V : Real;
                  Ahead : Boolean;
               begin
                  if Turned (Sg.Frame) then
                     if not Made (Sg.Track) or else Made_U (Sg.Track) /= Sg.U0 or else Made_V (Sg.Track) /= Sg.V0
                     then
                        declare
                           D0 : constant Real := Exp (Dv (Sg.Track));
                           H0 : constant Vec3 := Ray (Lx, Sg.U0, Sg.V0);
                        begin
                           Points (Sg.Track) := [D0 * H0 (1), D0 * H0 (2), D0 * H0 (3)];
                           Made (Sg.Track) := True;
                           Made_U (Sg.Track) := Sg.U0;
                           Made_V (Sg.Track) := Sg.V0;
                        end;
                     end if;
                     Project (Lx, Views (Sg.Frame) * Points (Sg.Track), U, V, Ahead);
                     R (2 * K - 1) := U - Sg.U;
                     R (2 * K) := V - Sg.V;
                  end if;
               end;
            end loop;
            Free (Turned);
            Free (Views);
            Free (Points);
            Free (Made);
            Free (Made_U);
            Free (Made_V);
         end Residuals;

         --  What the rotations leave of the sightings, found in one linear solve: a depth along every track's reference
         --  ray, and for every joint the two numbers its axis point has across its axis. The keyframe's eye stands at
         --  the rotation R and translation t of the joints' steps (t is linear in the axis points, whatever the
         --  rotations are), and sees the point d H0 along its own ray B where B x R^T (d H0 - t) = 0: linear in the depth
         --  and the axis points. The depths are eliminated track by track; the axis points are the vector the reduced
         --  normal matrix has least (the scale of all lengths is free), with the sightings that fit weighted as
         --  Huber weights them at their own angular residuals, until the sightings that fit are the same. The
         --  nonlinear descent then starts from lengths that agree with one another, which the stages before it, that
         --  tell the axes' directions and not their distances apart, leave to chance.
         procedure Linear_Start is
            --  The unknowns: a turning joint's two (its axis point across its axis, on E_1 and E_2), a sliding
            --  joint's one (its length per reading unit along its direction, E_1 that direction).
            function Slides (J : Positive) return Boolean is (Joints (Joints'First + J - 1).Slide);
            function Firsts return Natural_Array is
               Result : Natural_Array (1 .. N);
               Before : Natural := 0;
            begin
               for K in 1 .. N loop
                  Result (K) := Before + 1;
                  Before := Before + (if Slides (K) then 1 else 2);
               end loop;
               return Result;
            end Firsts;
            First_Index : constant Natural_Array (1 .. N) := Firsts;
            function First_Of (J : Positive) return Positive is (First_Index (J));
            Reduced : constant Positive := First_Of (N) + (if Slides (N) then 0 else 1);
            type Mat3_Array is array (Positive range <>) of Mat3;
            type Mat3_Access is access Mat3_Array;
            procedure Free is new Ada.Unchecked_Deallocation (Mat3_Array, Mat3_Access);
            Turns   : Mat3_Access := new Mat3_Array (1 .. Frames);
            Maps    : Mat3_Access := new Mat3_Array (1 .. Frames * N);   --  per keyframe and joint: t = sum Maps * P
            E_1, E_2 : array (1 .. N) of Vec3;
            Poses   : Rigid_Access := new Rigid_Array (1 .. Frames);
            Along   : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. S, 1 .. 3);
            Across_Columns : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. 3 * S, 1 .. Reduced);
            Weight  : Real_Access := new Real_Array'(1 .. S => 0.0);
            Own     : Real_Access := new Real_Array (1 .. S);
            Fit_Now : Flags_Access := new Flags'(1 .. S => False);
            Saa     : Real_Access := new Real_Array (1 .. Tracks);
            Sag     : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Tracks, 1 .. Reduced);
            Q       : Real_Vector (1 .. Reduced) := [others => 0.0];
            Lengths : Real_Access := new Real_Array (1 .. Tracks);
            Focal   : constant Real := (L.Fx + L.Fy) / 2.0;

            --  The third-order column of the sighting I's equation for the unknown C.
            function Col (I, C : Positive) return Vec3 is
              ([Across_Columns (3 * I - 2, C), Across_Columns (3 * I - 1, C), Across_Columns (3 * I, C)]);

            --  The axes' points of q, in the frame of the eye at rest.
            function Point_Of (J : Positive; V : Real_Vector) return Vec3 is
              (if Slides (J) then V (First_Of (J)) * E_1 (J)
               else V (First_Of (J)) * E_1 (J) + V (First_Of (J) + 1) * E_2 (J));

            --  Where the keyframe's eye stands for the points q gives the joints.
            function Translation_Of (F : Positive; V : Real_Vector) return Vec3 is
               T : Vec3 := Zero3;
            begin
               for J in 1 .. N loop
                  T := T + Maps ((F - 1) * N + J) * Point_Of (J, V);
               end loop;
               return T;
            end Translation_Of;

            --  The weights of the sightings, the depths and the points they give, for the weights in Weight.
            procedure Solve is
               K : Driver.Numerics.Arrays.Real_Matrix (1 .. Reduced, 1 .. Reduced) := [others => [others => 0.0]];
               Values  : Real_Vector (1 .. Reduced);
               Vectors : Driver.Numerics.Arrays.Real_Matrix (1 .. Reduced, 1 .. Reduced);
            begin
               Saa.all := [others => 0.0];
               Sag.all := [others => [others => 0.0]];
               for I in 1 .. S loop
                  if Weight (I) > 0.0 then
                     declare
                        T : constant Positive := Sight (Sight'First + I - 1).Track;
                        A : constant Vec3 := [Along (I, 1), Along (I, 2), Along (I, 3)];
                        W : constant Real := Weight (I);
                     begin
                        Saa (T) := Saa (T) + W * (A * A);
                        for C in 1 .. Reduced loop
                           Sag (T, C) := Sag (T, C) + W * (A * Col (I, C));
                           for C2 in C .. Reduced loop
                              K (C, C2) := K (C, C2) + W * (Col (I, C) * Col (I, C2));
                           end loop;
                        end loop;
                     end;
                  end if;
               end loop;
               for C in 1 .. Reduced loop
                  for C2 in 1 .. C - 1 loop
                     K (C, C2) := K (C2, C);
                  end loop;
               end loop;
               for T in 1 .. Tracks loop
                  if Saa (T) > 0.0 then
                     for C in 1 .. Reduced loop
                        for C2 in 1 .. Reduced loop
                           K (C, C2) := K (C, C2) - Sag (T, C) * Sag (T, C2) / Saa (T);
                        end loop;
                     end loop;
                  end if;
               end loop;
               Eigensystem (0.5 * (K + Transpose (K)), Values, Vectors);
               for C in 1 .. Reduced loop
                  Q (C) := Vectors (C, Reduced);
               end loop;
               declare
                  Ahead, Behind : Natural := 0;
               begin
                  for T in 1 .. Tracks loop
                     Lengths (T) := 0.0;
                     if Saa (T) > 0.0 then
                        declare
                           Sum : Real := 0.0;
                        begin
                           for C in 1 .. Reduced loop
                              Sum := Sum + Sag (T, C) * Q (C);
                           end loop;
                           Lengths (T) := Sum / Saa (T);
                           if Lengths (T) > 0.0 then
                              Ahead := Ahead + 1;
                           elsif Lengths (T) < 0.0 then
                              Behind := Behind + 1;
                           end if;
                        end;
                     end if;
                  end loop;
                  if Behind > Ahead then
                     Q := -Q;
                     for T in 1 .. Tracks loop
                        Lengths (T) := -Lengths (T);
                     end loop;
                  end if;
               end;
            end Solve;

            --  Each sighting's angular residual at these lengths, in pixels at the focal length, the weights it now has
            --  (Huber's at its own scale) and whether it fits (not significant against that scale); how many sightings
            --  changed from fitting to not or the other way.
            procedure Judge (Now : out Natural) is
               Rho : Real_Access := new Real_Array (1 .. S);
            begin
               Now := 0;
               for I in 1 .. S loop
                  declare
                     Sg : Sighting renames Sight (Sight'First + I - 1);
                     T  : constant Positive := Sg.Track;
                  begin
                     Rho (I) := Real'Last;
                     if Saa (T) > 0.0 and then Lengths (T) > 0.0 then
                        declare
                           H0   : constant Vec3 := Ray (L, Sg.U0, Sg.V0);
                           B    : constant Vec3 := Ray (L, Sg.U, Sg.V);
                           Seen : constant Vec3 :=
                             Transpose (Turns (Sg.Frame)) * (Lengths (T) * H0 - Translation_Of (Sg.Frame, Q));
                        begin
                           if Dot (B, Seen) > 0.0 and then abs Seen > 0.0 then
                              Rho (I) := Focal * Real'(abs Cross (B, Seen)) / (Real'(abs B) * Real'(abs Seen));
                           end if;
                        end;
                     end if;
                  end;
               end loop;
               declare
                  Sub     : Real_Access := new Real_Array (1 .. S);
                  Own_Sub : Real_Access := new Real_Array (1 .. S);
                  Used    : Natural := 0;
               begin
                  for I in 1 .. S loop
                     if Rho (I) < Real'Last then
                        Used := Used + 1;
                        Sub (Used) := Rho (I);
                        Own_Sub (Used) := Own (I);
                     end if;
                  end loop;
                  declare
                     Model : constant Error_Model :=
                       (if Used > 0 then Model_Of (Sub (1 .. Used), Own_Sub (1 .. Used)) else (Floor => 0.0, Gain => 1.0));
                  begin
                     for I in 1 .. S loop
                        declare
                           Scale : constant Real := Scale_Of (Model, Own (I));
                           Seen  : constant Boolean := Rho (I) < Real'Last and then Scale > Real'Model_Small;
                           Fits  : constant Boolean := Seen and then not Driver.Uncertain.Significant (Rho (I), Scale);
                        begin
                           Weight (I) := (if Seen then Huber (Rho (I) / Scale) else 0.0);
                           if Fits /= Fit_Now (I) then
                              Now := Now + 1;
                              Fit_Now (I) := Fits;
                           end if;
                        end;
                     end loop;
                  end;
                  Free (Sub);
                  Free (Own_Sub);
               end;
               Free (Rho);
            end Judge;

            Changed : Natural := Natural'Last;
         begin
            for F in 1 .. Frames loop
               declare
                  Change : constant Real_Array := Changes_Of (F);
                  Turn   : Mat3 := Identity3;
               begin
                  --  The joints compose in their order along the arm (Chain).
                  for J of Chain loop
                     declare
                        A    : Joint renames Joints (Joints'First + J - 1);
                        Step : constant Mat3 := (if A.Slide then Identity3 else Rot (A.W, A.C * Change (J)));
                     begin
                        --  A slide moves the eye by its reading change along its direction, its length the unknown.
                        Maps ((F - 1) * N + J) := (if A.Slide then Change (J) * Turn else Turn * (Identity3 - Step));
                        Turn := Turn * Step;
                     end;
                  end loop;
                  Turns (F) := Turn;
                  Poses (F) := Eye_At (Joints, Chain, Change);
               end;
            end loop;
            for J in 1 .. N loop
               if Slides (J) then
                  E_1 (J) := Joints (Joints'First + J - 1).W;
                  E_2 (J) := Zero3;
               else
                  Perp (Joints (Joints'First + J - 1).W, E_1 (J), E_2 (J));
               end if;
            end loop;
            for I in 1 .. S loop
               declare
                  Sg   : Sighting renames Sight (Sight'First + I - 1);
                  H0   : constant Vec3 := Ray (L, Sg.U0, Sg.V0);
                  B    : constant Vec3 := Ray (L, Sg.U, Sg.V);
                  Back : constant Mat3 := Transpose (Turns (Sg.Frame));
                  A    : constant Vec3 := Cross (B, Back * H0);
               begin
                  Own (I) := Sg.Own;
                  for D in 1 .. 3 loop
                     Along (I, D) := A (D);
                  end loop;
                  for J in 1 .. N loop
                     for Axis in 1 .. (if Slides (J) then 1 else 2) loop
                        declare
                           Col : constant Vec3 :=
                             Cross (B, Back * (Maps ((Sg.Frame - 1) * N + J) * (if Axis = 1 then E_1 (J) else E_2 (J))));
                        begin
                           for D in 1 .. 3 loop
                              Across_Columns (3 * I - 3 + D, First_Of (J) + Axis - 1) := Col (D);
                           end loop;
                        end;
                     end loop;
                  end loop;
               end;
            end loop;
            --  The sightings the stages before it fit to begin with: the epipolar residual (which does not depend on
            --  the lengths) not significant against the noise the residuals show.
            declare
               Rs : Real_Access := new Real_Array (1 .. S);
            begin
               for I in 1 .. S loop
                  declare
                     Sg     : Sighting renames Sight (Sight'First + I - 1);
                     Rr     : Mat3;
                     Tt     : Vec3;
                  begin
                     Relative (Poses (1), Poses (Sg.Frame), Rr, Tt);
                     Rs (I) := Sampson (Rr, Tt, Ray (L, Sg.U0, Sg.V0), Ray (L, Sg.U, Sg.V), Focal);
                  end;
               end loop;
               declare
                  Model : constant Error_Model := Model_Of (Rs.all, Own.all);
               begin
                  for I in 1 .. S loop
                     declare
                        Scale : constant Real := Scale_Of (Model, Own (I));
                     begin
                        if not Driver.Uncertain.Significant (Rs (I), Scale) then
                           Weight (I) := 1.0;
                        end if;
                     end;
                  end loop;
               end;
               Free (Rs);
            end;
            for Round in 1 .. Real'Machine_Mantissa loop
               Solve;
               declare
                  Now : Natural;
               begin
                  Judge (Now);
                  exit when Now = 0 or else Now >= Changed;
                  Changed := Now;
               end;
            end loop;
            Solve;
            declare
               Now : Natural;
            begin
               Judge (Now);
            end;
            --  The lengths as the stages before it had them, in size: the eye's travel over the keyframes.
            declare
               Old, Now : Real := 0.0;
            begin
               for F in 1 .. Frames loop
                  Old := Old + Poses (F).Translation * Poses (F).Translation;
                  Now := Now + Translation_Of (F, Q) * Translation_Of (F, Q);
               end loop;
               declare
                  Size : constant Real := (if Now > 0.0 and then Old > 0.0 then Sqrt (Old / Now) else 1.0);
               begin
                  for J in 1 .. N loop
                     if Slides (J) then
                        Joints (Joints'First + J - 1).C := Size * Q (First_Of (J));
                     else
                        Joints (Joints'First + J - 1).P := Size * Point_Of (J, Q);
                     end if;
                  end loop;
                  for T in 1 .. Tracks loop
                     Has (T) := Saa (T) > 0.0 and then Lengths (T) > 0.0;
                     Depth (T) := (if Has (T) then Ln (Size * Lengths (T)) else 0.0);
                  end loop;
                  --  The descent starts from the sightings that fit these lengths.
                  for I in 1 .. S loop
                     Inlier (I) := Has (Sight (Sight'First + I - 1).Track) and then Fit_Now (I);
                  end loop;
               end;
            end;
            Free (Turns);
            Free (Maps);
            Free (Poses);
            Free (Along);
            Free (Across_Columns);
            Free (Weight);
            Free (Own);
            Free (Fit_Now);
            Free (Saa);
            Free (Sag);
            Free (Lengths);
         end Linear_Start;
      begin
         Linear_Start;
         To_X (L, Joints, X);
         loop
            declare
               Used  : Natural := 0;
            begin
               for B of Inlier.all loop
                  if B then
                     Used := Used + 1;
                  end if;
               end loop;
               exit when 2 * Used <= Count + Tracks;
               Passes := Passes + 1;
               Began (Passes) := new Flags'(Inlier.all);
               declare
                  Index : Real_Access := new Real_Array (1 .. Used);
                  R0    : Real_Access := new Real_Array (1 .. 2 * Used);
                  Rn    : Real_Access := new Real_Array (1 .. 2 * Used);
                  Jp    : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. 2 * Used, 1 .. Count);
                  Jd    : Real_Access := new Real_Array (1 .. 2 * Used);
                  Owns  : Real_Access := new Real_Array (1 .. 2 * Used);   --  what the matcher stated, per residual
                  Scales : Real_Access := new Real_Array (1 .. 2 * Used);  --  what each residual is judged at
                  K     : Natural := 0;
                  Lambda : Real := Real'Model_Epsilon;
                  Cost0 : Real;
                  --  The sightings' cost at the parameters Xv, and the focal search's measurement's (Prior_W).
                  function Cost (R : Real_Array; Xv : Real_Array) return Real is
                     C : Real := 0.0;
                  begin
                     for I in R'Range loop
                        C := C + Huber_Cost (R (I) / Scales (I));
                     end loop;
                     return C + 0.5 * Prior_W * ((Xv (Xv'First) - Ln_Fx0) ** 2 + (Xv (Xv'First + 1) - Ln_Fy0) ** 2);
                  end Cost;
               begin
                  for I in 1 .. S loop
                     if Inlier (I) then
                        K := K + 1;
                        Index (K) := Real (I);
                        Owns (2 * K - 1) := Sight (Sight'First + I - 1).Own;
                        Owns (2 * K) := Sight (Sight'First + I - 1).Own;
                     end if;
                  end loop;
                  Residuals (X, Depth.all, Index, R0.all);
                  declare
                     Model : constant Error_Model := Model_Of (R0.all, Owns.all);
                  begin
                     for I in 1 .. 2 * Used loop
                        Scales (I) := Scale_Of (Model, Owns (I));
                     end loop;
                  end;
                  if (for all Scale of Scales.all => Scale <= Real'Model_Small) then
                     Free (Index);
                     Free (R0);
                     Free (Rn);
                     Free (Jp);
                     Free (Jd);
                     Free (Owns);
                     Free (Scales);
                     exit;
                  end if;
                  Cost0 := Cost (R0.all, X);
                  --  A step that still lowers the cost is taken again, as many times as the float has bits: each
                  --  step of a convergent descent gains digits, and one that crawls down a valley the sightings
                  --  leave flat has not found the fit they determine (the covariance then says what is left
                  --  undetermined).
                  for Round in 1 .. Real'Machine_Mantissa loop
                     Rounds := Rounds + 1;
                     --  The Jacobian: the parameters one by one; the depths all at
                     --  once, since each residual has one depth of its own.
                     --  The columns at once (Driver.Parallel_For): each moves its own
                     --  parameter, takes every residual again into its own array and
                     --  writes only its own column.
                     declare
                        procedure Column (P : Positive) is
                           Xp : Real_Array := X;
                           H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (P));
                           Rp : Real_Access := new Real_Array'(R0.all);
                        begin
                           --  The terms a pinhole with square pixels has not, the lens's second focal length and its two
                           --  distortion terms, have no column, and a term with none stays where it is (a sliding joint's
                           --  axis point's).
                           if Pinhole and then P in 2 | 5 | 6 then
                              for I in 1 .. 2 * Used loop
                                 Jp (I, P) := 0.0;
                              end loop;
                           else
                              Xp (P) := Xp (P) + H;
                              Residuals (Xp, Depth.all, Index, Rp.all,
                                         Joint => (if P <= 6 then 0 else (P - 7) / Per_Joint + 1));
                              for I in 1 .. 2 * Used loop
                                 Jp (I, P) := (Rp (I) - R0 (I)) / H;
                              end loop;
                           end if;
                           Free (Rp);
                        end Column;
                        procedure Columns is new Driver.Parallel_For (Column);
                     begin
                        Columns (1, Count);
                     end;
                     declare
                        Dp : Real_Access := new Real_Array'(Depth.all);
                        H  : constant Real := Sqrt (Real'Model_Epsilon);
                     begin
                        for T in 1 .. Tracks loop
                           Dp (T) := Dp (T) + H * Real'Max (1.0, abs Depth (T));
                        end loop;
                        Residuals (X, Dp.all, Index, Rn.all);
                        Free (Dp);
                        for I in 1 .. 2 * Used loop
                           declare
                              T : constant Positive := Sight (Sight'First + Natural (Index ((I + 1) / 2)) - 1).Track;
                           begin
                              Jd (I) := (Rn (I) - R0 (I)) / (H * Real'Max (1.0, abs Depth (T)));
                           end;
                        end loop;
                     end;
                     declare
                        A  : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Count) := [others => [others => 0.0]];
                        Bm : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Tracks);
                        C  : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
                        Gp : Real_Vector (1 .. Count) := [others => 0.0];
                        Gd : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
                        Improved, Lowered : Boolean := False;
                     begin
                        Bm.all := [others => [others => 0.0]];
                        --  The normal equations, each parameter's row of them at once (Driver.Parallel_For), and the
                        --  depths' own sums as one more: every sum runs over the residuals in their order whichever
                        --  task makes it, so that it is the sum one task made, to the last bit (the fit's path turns
                        --  on its last bits: summed track by track, robot.kinematics.desktop ended at 0.0014 px). One
                        --  task summed every residual into every row, the most of a tracks' fit's time (s02's fit
                        --  replayed from its dump and sampled, 10-16: 7.8 s of 18 in stage 5 in Refine_Tracks itself,
                        --  the workers waiting).
                        declare
                           Weights  : Real_Access := new Real_Array (1 .. 2 * Used);
                           Track_Of : Natural_Access := new Natural_Array (1 .. 2 * Used);
                           Frame_Of : Natural_Access := new Natural_Array (1 .. 2 * Used);
                           Moved_In : Flag_Grid_Access := new Flag_Grid (1 .. Frames, 1 .. N);
                           procedure Row (P : Positive) is
                           begin
                              if P > Count then
                                 for I in 1 .. 2 * Used loop
                                    C (Track_Of (I)) := C (Track_Of (I)) + Weights (I) * Jd (I) ** 2;
                                    Gd (Track_Of (I)) := Gd (Track_Of (I)) - Weights (I) * Jd (I) * R0 (I);
                                 end loop;
                              else
                                 declare
                                    Sum_G : Real := 0.0;
                                    Sum_A : Real_Array (P .. Count) := [others => 0.0];
                                 begin
                                    for I in 1 .. 2 * Used loop
                                       if Jp (I, P) /= 0.0 then
                                          declare
                                             W : constant Real := Weights (I);
                                             F : constant Positive := Frame_Of (I);
                                          begin
                                             Sum_G := Sum_G - W * Jp (I, P) * R0 (I);
                                             Bm (P, Track_Of (I)) := Bm (P, Track_Of (I)) + W * Jp (I, P) * Jd (I);
                                             for Q in P .. Natural'Min (6, Count) loop
                                                Sum_A (Q) := Sum_A (Q) + W * Jp (I, P) * Jp (I, Q);
                                             end loop;
                                             for J in 1 .. N loop
                                                if Moved_In (F, J) and then 6 + Per_Joint * J >= P then
                                                   for Q in Natural'Max (P, 7 + Per_Joint * (J - 1)) .. 6 + Per_Joint * J loop
                                                      Sum_A (Q) := Sum_A (Q) + W * Jp (I, P) * Jp (I, Q);
                                                   end loop;
                                                end if;
                                             end loop;
                                          end;
                                       end if;
                                    end loop;
                                    Gp (P) := Sum_G;
                                    for Q in P .. Count loop
                                       A (P, Q) := Sum_A (Q);
                                    end loop;
                                 end;
                              end if;
                           end Row;
                           procedure Rows is new Driver.Parallel_For (Row);
                        begin
                           for I in 1 .. 2 * Used loop
                              Weights (I) := Huber (R0 (I) / Scales (I)) / Scales (I) ** 2;
                              Track_Of (I) := Sight (Sight'First + Natural (Index ((I + 1) / 2)) - 1).Track;
                              Frame_Of (I) := Sight (Sight'First + Natural (Index ((I + 1) / 2)) - 1).Frame;
                           end loop;
                           for F in 1 .. Frames loop
                              for J in 1 .. N loop
                                 Moved_In (F, J) := Changes (Changes'First (1) + F - 1, Changes'First (2) + J - 1) /= 0.0;
                              end loop;
                           end loop;
                           Rows (1, Count + 1);
                           Free (Weights);
                           Free (Track_Of);
                           Free (Frame_Of);
                           Free (Moved_In);
                        end;
                        --  The focal search's measurement of the two focal lengths (Prior_W, the header).
                        if Prior_W > 0.0 then
                           A (1, 1) := A (1, 1) + Prior_W;
                           Gp (1) := Gp (1) - Prior_W * (X (1) - Ln_Fx0);
                           A (2, 2) := A (2, 2) + Prior_W;
                           Gp (2) := Gp (2) - Prior_W * (X (2) - Ln_Fy0);
                        end if;
                        for P in 1 .. Count loop
                           for Q in 1 .. P - 1 loop
                              A (P, Q) := A (Q, P);
                           end loop;
                        end loop;
                        loop
                           declare
                              Sm    : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Count) := A;
                              Rhs   : Real_Vector (1 .. Count) := Gp;
                              Cl    : Real_Access := new Real_Array (1 .. Tracks);
                              Lf    : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Count);
                              Pd    : Boolean;
                              Moves : Boolean := False;
                           begin
                              for P in 1 .. Count loop
                                 Sm (P, P) := A (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                              end loop;
                              for T in 1 .. Tracks loop
                                 Cl (T) := C (T) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                              end loop;
                              --  The depths eliminated, each parameter's row at once (Driver.Parallel_For), every row's
                              --  sums over the tracks in their order, as the normal equations' are (above).
                              declare
                                 procedure Eliminate (P : Positive) is
                                 begin
                                    for T in 1 .. Tracks loop
                                       if Cl (T) > 0.0 and then Bm (P, T) /= 0.0 then
                                          Rhs (P) := Rhs (P) - Bm (P, T) * Gd (T) / Cl (T);
                                          for Q in 1 .. Count loop
                                             Sm (P, Q) := Sm (P, Q) - Bm (P, T) * Bm (Q, T) / Cl (T);
                                          end loop;
                                       end if;
                                    end loop;
                                 end Eliminate;
                                 procedure Eliminate_All is new Driver.Parallel_For (Eliminate);
                              begin
                                 Eliminate_All (1, Count);
                              end;
                              Driver.Numerics.Dense.Cholesky (Sm, Lf, Pd);
                              if Pd then
                                 declare
                                    Dx : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, Rhs);
                                    Xn : Real_Array := X;
                                    Dn : Real_Access := new Real_Array'(Depth.all);
                                 begin
                                    for P in 1 .. Count loop
                                       Xn (P) := X (P) + Dx (P);
                                       Moves := Moves or else Xn (P) /= X (P);
                                    end loop;
                                    for T in 1 .. Tracks loop
                                       if Cl (T) > 0.0 then
                                          declare
                                             Sum : Real := Gd (T);
                                          begin
                                             for P in 1 .. Count loop
                                                Sum := Sum - Bm (P, T) * Dx (P);
                                             end loop;
                                             Dn (T) := Depth (T) + Sum / Cl (T);
                                             Moves := Moves or else Dn (T) /= Depth (T);
                                          end;
                                       end if;
                                    end loop;
                                    if Moves then
                                       Residuals (Xn, Dn.all, Index, Rn.all);
                                       declare
                                          Cn : constant Real := Cost (Rn.all, Xn);
                                       begin
                                          if Cn < Cost0 then
                                             Improved := Moves_The_Fit (Cost0 - Cn);
                                             X := Xn;
                                             Depth.all := Dn.all;
                                             R0.all := Rn.all;
                                             Cost0 := Cn;
                                             Lambda := Lambda / 2.0;
                                             Lowered := True;
                                          end if;
                                       end;
                                    end if;
                                    Free (Dn);
                                 end;
                              else
                                 Moves := True;
                              end if;
                              Free (Cl);
                              --  No further than a float goes (as Fixed.Refine): past it the damping is infinite, the damped matrix no
                              --  number, never positive definite, and nothing would end the loop.
                              exit when Lowered or else not Moves or else Lambda >= Real'Last / 2.0;
                              Lambda := 2.0 * Lambda;
                           end;
                        end loop;
                        --  At the solution: what the sightings leave each
                        --  parameter uncertain by, the undamped normal equations
                        --  inverted after the two freedoms the images cannot fix
                        --  are removed: an axis point slides along its axis
                        --  (only the point nearest the eye is kept, two numbers in
                        --  the plane across the axis), and every length scales
                        --  together (the depth of the track seen most is held).
                        if not Improved then
                           for T in 1 .. Tracks loop
                              Depth_Sigma (T) := (if C (T) > 0.0 then 1.0 / Sqrt (C (T)) else Real'Last);
                           end loop;
                           declare
                              Reduced : constant Positive := 6 + 5 * N;
                              Tm  : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Reduced) :=
                                [others => [others => 0.0]];
                              Anchor : Positive := 1;
                              Seen_Of : Count_Access := new Count_Array'(1 .. Tracks => 0);
                           begin
                              for K in 1 .. Used loop
                                 declare
                                    T : constant Positive := Sight (Sight'First + Natural (Index (K)) - 1).Track;
                                 begin
                                    Seen_Of (T) := Seen_Of (T) + 1;
                                    if Seen_Of (T) > Seen_Of (Anchor) then
                                       Anchor := T;
                                    end if;
                                 end;
                              end loop;
                              for P in 1 .. 6 loop
                                 Tm (P, P) := 1.0;
                              end loop;
                              for J in 1 .. N loop
                                 declare
                                    O  : constant Natural := 6 + Per_Joint * (J - 1);
                                    Ro : constant Natural := 6 + 5 * (J - 1);
                                    E1, E2 : Vec3;
                                 begin
                                    Perp (Base (Base'First + J - 1).W, E1, E2);
                                    Tm (O + 1, Ro + 1) := 1.0;
                                    Tm (O + 2, Ro + 2) := 1.0;
                                    for D in 1 .. 3 loop
                                       Tm (O + 2 + D, Ro + 3) := E1 (D);
                                       Tm (O + 2 + D, Ro + 4) := E2 (D);
                                    end loop;
                                    Tm (O + 6, Ro + 5) := 1.0;
                                 end;
                              end loop;
                              declare
                                 Ar : Driver.Numerics.Arrays.Real_Matrix := Transpose (Tm) * A * Tm;
                                 Br : Grid_Access := new Real_Matrix'[1 .. Reduced => [1 .. Tracks => 0.0]];
                                 Lf : Driver.Numerics.Arrays.Real_Matrix (1 .. Reduced, 1 .. Reduced);
                                 Pd : Boolean;
                                 --  The reduced terms are in units that differ by as many orders of magnitude as the
                                 --  fit's parameters do (a log focal length, pixels, radians, lengths): the matrix
                                 --  is made positive definite or not by its correlations, each term in units of the
                                 --  standard error it would have were the others known, and not by the sizes of its entries.
                                 Equal : Real_Array (1 .. Reduced) := [others => 0.0];
                              begin
                                 Free (Seen_Of);
                                 for P in 1 .. Reduced loop
                                    for Q in 1 .. Count loop
                                       if Tm (Q, P) /= 0.0 then
                                          for T in 1 .. Tracks loop
                                             Br (P, T) := Br (P, T) + Tm (Q, P) * Bm (Q, T);
                                          end loop;
                                       end if;
                                    end loop;
                                 end loop;
                                 for T in 1 .. Tracks loop
                                    if T /= Anchor and then C (T) > 0.0 then
                                       for P in 1 .. Reduced loop
                                          if Br (P, T) /= 0.0 then
                                             for Q in 1 .. Reduced loop
                                                Ar (P, Q) := Ar (P, Q) - Br (P, T) * Br (Q, T) / C (T);
                                             end loop;
                                          end if;
                                       end loop;
                                    end if;
                                 end loop;
                                 --  A sliding joint has no axis point: its two terms of one are no terms of it, each
                                 --  apart from every other with information of its own unit (its covariance then
                                 --  nought, the sandwich's gradients of it being nought).
                                 for J in 1 .. N loop
                                    if Joints (Joints'First + J - 1).Slide then
                                       for P in 6 + 5 * (J - 1) + 3 .. 6 + 5 * (J - 1) + 4 loop
                                          for Q in 1 .. Reduced loop
                                             Ar (P, Q) := 0.0;
                                             Ar (Q, P) := 0.0;
                                          end loop;
                                          Ar (P, P) := 1.0;
                                       end loop;
                                    end if;
                                 end loop;
                                 if Pinhole then
                                    for P in 1 .. 6 loop
                                       if P in 2 | 5 | 6 then
                                          for Q in 1 .. Reduced loop
                                             Ar (P, Q) := 0.0;
                                             Ar (Q, P) := 0.0;
                                          end loop;
                                          Ar (P, P) := 1.0;
                                       end if;
                                    end loop;
                                 end if;
                                 declare
                                    Scaled : Driver.Numerics.Arrays.Real_Matrix (1 .. Reduced, 1 .. Reduced);
                                 begin
                                    for P in 1 .. Reduced loop
                                       Equal (P) := (if Ar (P, P) > 0.0 then 1.0 / Sqrt (Ar (P, P)) else 0.0);
                                    end loop;
                                    for P in 1 .. Reduced loop
                                       for Q in 1 .. Reduced loop
                                          Scaled (P, Q) := Equal (P) * Ar (P, Q) * Equal (Q);
                                       end loop;
                                    end loop;
                                    Driver.Numerics.Dense.Cholesky (Scaled, Lf, Pd);
                                    Pd := Pd and then (for all E of Equal => E > 0.0);
                                    --  When the equations are singular, which term is the first that the terms
                                    --  before it already determine, for the log: the pivots of an elimination over
                                    --  the scaled matrix in the terms' order (the lens's six, then each joint's two
                                    --  of its axis, two of its point across the axis, and its scale), and the terms
                                    --  with no information at all. What the sightings fail to determine is the
                                    --  question (the kiwi, R6's tenth start, 10-14: a lens, or a joint's point that
                                    --  no parallax reaches).
                                    if not Pd then
                                       declare
                                          Work  : Driver.Numerics.Arrays.Real_Matrix := Scaled;
                                          Text  : Ada.Strings.Unbounded.Unbounded_String;
                                          Least_Pivot : Real := Real'Last;
                                          Least_At    : Natural := 0;
                                          function Name_Of (P : Positive) return String is
                                            (if P <= 6 then
                                               (case P is
                                                  when 1 => "fx", when 2 => "fy", when 3 => "cx",
                                                  when 4 => "cy", when 5 => "k1", when others => "k2")
                                             else "j" & Ada.Strings.Fixed.Trim (Positive'Image ((P - 7) / 5 + 1), Ada.Strings.Left)
                                                  & (case (P - 7) mod 5 is
                                                       when 0 => " axis a", when 1 => " axis b", when 2 => " point a",
                                                       when 3 => " point b", when others => " scale"));
                                       begin
                                          for P in 1 .. Reduced loop
                                             if Equal (P) = 0.0 then
                                                Ada.Strings.Unbounded.Append (Text, " " & Name_Of (P) & ": nothing");
                                             end if;
                                          end loop;
                                          for P in 1 .. Reduced loop
                                             declare
                                                Pivot : constant Real := Work (P, P);
                                             begin
                                                if Pivot < Least_Pivot then
                                                   Least_Pivot := Pivot;
                                                   Least_At := P;
                                                end if;
                                                if Pivot > 0.0 then
                                                   for Q in P + 1 .. Reduced loop
                                                      declare
                                                         Factor : constant Real := Work (Q, P) / Pivot;
                                                      begin
                                                         if Factor /= 0.0 then
                                                            for Rr in P .. Reduced loop
                                                               Work (Q, Rr) := Work (Q, Rr) - Factor * Work (P, Rr);
                                                            end loop;
                                                         end if;
                                                      end;
                                                   end loop;
                                                end if;
                                             end;
                                          end loop;
                                          Driver.Log.Line
                                            (Driver.Log.Robot, "kinematics: the tracks' fit's normal equations are singular; the"
                                             & " least pivot of the scaled terms in order is "
                                             & Driver.Log.Image (Least_Pivot, 6) & " at "
                                             & (if Least_At > 0 then Name_Of (Least_At) else "none")
                                             & (if Ada.Strings.Unbounded.Length (Text) > 0
                                                then "; terms without information:" & Ada.Strings.Unbounded.To_String (Text)
                                                else "") & "; focal " & Driver.Log.Image (L.Fx, 1) & " px");
                                       end;
                                    end if;
                                 end;
                                 Determined := Pd;
                                 Why_Not := Ada.Strings.Unbounded.To_Unbounded_String
                                   ("the sightings do not determine the lens and the joints (the normal equations are singular)");
                                 Variance := [others => Real'Last];
                                 Report.Covariance.Clear;
                                 --  The covariance of the estimate: the inverse normal
                                 --  equations around the spread of the gradient, which
                                 --  weights every pair of residual rows by the covariance
                                 --  of their errors. How the matcher's errors depend on
                                 --  each other is read from the residuals themselves, as
                                 --  a function of the distance between their points
                                 --  (Errors): a point errs alike in every keyframe and so
                                 --  do points near each other, and the points of a
                                 --  keyframe err alike. Sightings taken as independent
                                 --  hide it: A9's focal length and reading scales came
                                 --  out 10 to 20 of the keyframes' sigmas off the truth,
                                 --  and A11's lenses 20 to 40 chi squares on six terms
                                 --  clustered by keyframe. Only the last pass's is wanted
                                 --  and it costs more than the pass: what it is made of
                                 --  is kept, and the passes end before it is made.
                                 Release (Last_Pass);
                                 Report.Covariance.Clear;
                                 if Pd and then not Spread then
                                    Determined := False;
                                    Why_Not := Ada.Strings.Unbounded.To_Unbounded_String
                                      ("the estimate's covariance was not asked for");
                                 elsif Pd then
                                    Last_Pass.Ready := True;
                                    Last_Pass.Inv := new Real_Matrix (1 .. Reduced, 1 .. Reduced);
                                    Last_Pass.Rows := new Real_Matrix'[1 .. 2 * Used => [1 .. Reduced => 0.0]];
                                    Last_Pass.Error := new Real_Array'(1 .. 2 * Used => 0.0);
                                    Last_Pass.In_Frame := new Natural_Array'(1 .. Used => 1);
                                    Last_Pass.Of_Track := new Natural_Array'(1 .. Used => 1);
                                    Last_Pass.Ref_U := new Real_Array'(1 .. Used => 0.0);
                                    Last_Pass.Ref_V := new Real_Array'(1 .. Used => 0.0);
                                    declare
                                       --  Every sighting's gradient (the reduced terms, the depths
                                       --  eliminated) in units of the noise, and its error as the
                                       --  score sees it: clipped at Z of the noise.
                                       Inv      : Real_Matrix renames Last_Pass.Inv.all;
                                       Rows     : Real_Matrix renames Last_Pass.Rows.all;
                                       Error    : Real_Array renames Last_Pass.Error.all;
                                       In_Frame : Natural_Array renames Last_Pass.In_Frame.all;
                                       Of_Track : Natural_Array renames Last_Pass.Of_Track.all;
                                    begin
                                       for P in 1 .. Reduced loop
                                          declare
                                             E : Real_Vector (1 .. Reduced) := [others => 0.0];
                                          begin
                                             E (P) := 1.0;
                                             declare
                                                Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, E);
                                             begin
                                                --  The inverse of the matrix is the inverse of its scaled form,
                                                --  scaled again.
                                                for Q in 1 .. Reduced loop
                                                   Inv (Q, P) := Equal (Q) * Column (Q) * Equal (P);
                                                end loop;
                                             end;
                                          end;
                                       end loop;
                                       for K in 1 .. Used loop
                                          declare
                                             Sg : Sighting renames Sight (Sight'First + Natural (Index (K)) - 1);
                                          begin
                                             In_Frame (K) := Sg.Frame;
                                             Of_Track (K) := Sg.Track;
                                             Last_Pass.Ref_U (K) := Sg.U0;
                                             Last_Pass.Ref_V (K) := Sg.V0;
                                          end;
                                       end loop;
                                       for I in 1 .. 2 * Used loop
                                          declare
                                             T : constant Positive := Of_Track ((I + 1) / 2);
                                          begin
                                             Error (I) := Huber (R0 (I) / Scales (I)) * R0 (I);
                                             for P in 1 .. Reduced loop
                                                declare
                                                   Jr : Real := 0.0;
                                                begin
                                                   for Q in 1 .. Count loop
                                                      if Tm (Q, P) /= 0.0 then
                                                         Jr := Jr + Tm (Q, P) * Jp (I, Q);
                                                      end if;
                                                   end loop;
                                                   if T /= Anchor and then C (T) > 0.0 then
                                                      Jr := Jr - Br (P, T) / C (T) * Jd (I);
                                                   end if;
                                                   Rows (I, P) := Jr / Scales (I) ** 2;
                                                end;
                                             end loop;
                                          end;
                                       end loop;
                                    end;
                                 end if;
                                 --  How each track's log depth moves with the reduced
                                 --  terms, the others' at their best: minus its row of
                                 --  the cross information over its own (the held
                                 --  track's does not move).
                                 Report.Depth_Gains.Clear;
                                 for T in 1 .. Tracks loop
                                    for P in 1 .. Reduced loop
                                       Report.Depth_Gains.Append
                                         (if T /= Anchor and then C (T) > 0.0 then -Br (P, T) / C (T) else 0.0);
                                    end loop;
                                 end loop;
                                 Free (Br);
                              end;
                           end;
                        end if;
                        Free (Bm);
                        Free (C);
                        Free (Gd);
                        exit when not Improved;
                     end;
                  end loop;
                  Free (Index);
                  Free (R0);
                  Free (Rn);
                  Free (Jp);
                  Free (Jd);
                  Free (Owns);
                  Free (Scales);
               end;
            end;
            --  The axes' tangent planes anew at the solution.
            Base := Joints_Of (X);
            for J of Base loop
               J.P := J.P - Dot (J.P, J.W) * J.W;
            end loop;
            To_X (Lens_Of (X), Base, X);
            --  The sightings that fit, re-chosen against every residual's noise.
            declare
               All_Index : Real_Access := new Real_Array (1 .. S);
               All_R     : Real_Access := new Real_Array (1 .. 2 * S);
               All_Own   : Real_Access := new Real_Array (1 .. 2 * S);
               Sigma     : Real;
               Model     : Error_Model;
               Now_Changed : Natural := 0;
               Fit_Abs   : Real_Access;
               Kf        : Natural := 0;
            begin
               for I in 1 .. S loop
                  All_Index (I) := Real (I);
                  All_Own (2 * I - 1) := Sight (Sight'First + I - 1).Own;
                  All_Own (2 * I) := Sight (Sight'First + I - 1).Own;
               end loop;
               --  A track none of whose sightings fit has its depth again from the parameters as they stand, by least
               --  squares along the lines of sight of its sightings (as the start solves every depth), before they are
               --  judged: the depth it kept was made for parameters long gone, and judged by it they never fit again. A
               --  drone's fit begun at the focal search's 234 px (the truth 173) ended with 2900 of its 3755 sightings
               --  fitting, where the profile's steps begun from its end kept 3360 (the bench, 10-20).
               declare
                  Lx      : constant Lens := Lens_Of (X);
                  Jx      : constant Joint_Array := Joints_Of (X);
                  Views   : Rigid_Access := new Rigid_Array (1 .. Frames);
                  Fitting : Flags_Access := new Flags'(1 .. Tracks => False);
                  Saa_T   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
                  Sac_T   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
               begin
                  for I in 1 .. S loop
                     if Inlier (I) then
                        Fitting (Sight (Sight'First + I - 1).Track) := True;
                     end if;
                  end loop;
                  for F in 1 .. Frames loop
                     Views (F) := Eye_At (Jx, Chain, Changes_Of (F));
                  end loop;
                  for I in 1 .. S loop
                     declare
                        Sg : Sighting renames Sight (Sight'First + I - 1);
                     begin
                        if Has (Sg.Track) and then not Fitting (Sg.Track) then
                           declare
                              Back : constant Mat3 := Transpose (Views (Sg.Frame).Rotation);
                              B    : constant Vec3 := Ray (Lx, Sg.U, Sg.V);
                              A    : constant Vec3 := Cross (B, Back * Ray (Lx, Sg.U0, Sg.V0));
                              C    : constant Vec3 := Cross (B, Back * Views (Sg.Frame).Translation);
                           begin
                              Saa_T (Sg.Track) := Saa_T (Sg.Track) + A * A;
                              Sac_T (Sg.Track) := Sac_T (Sg.Track) + A * C;
                           end;
                        end if;
                     end;
                  end loop;
                  for T in 1 .. Tracks loop
                     if Has (T) and then not Fitting (T) and then Saa_T (T) > 0.0 and then Sac_T (T) > 0.0 then
                        Depth (T) := Ln (Sac_T (T) / Saa_T (T));
                     end if;
                  end loop;
                  Free (Views);
                  Free (Fitting);
                  Free (Saa_T);
                  Free (Sac_T);
               end;
               Residuals (X, Depth.all, All_Index, All_R.all);
               Model := Model_Of (All_R.all, All_Own.all);
               Sigma := Model.Floor;
               for I in 1 .. S loop
                  declare
                     Scale : constant Real := Scale_Of (Model, All_Own (2 * I));
                     Fits  : constant Boolean := Has (Sight (Sight'First + I - 1).Track) and then Scale > Real'Model_Small
                       and then not Driver.Uncertain.Significant (All_R (2 * I - 1), Scale)
                       and then not Driver.Uncertain.Significant (All_R (2 * I), Scale);
                  begin
                     if Fits /= Inlier (I) then
                        Now_Changed := Now_Changed + 1;
                        Inlier (I) := Fits;
                     end if;
                     if Fits then
                        Kf := Kf + 1;
                     end if;
                  end;
               end loop;
               Report.Sigma_Px := Sigma;
               Report.Own_Gain := Model.Gain;
               Report.Used := Kf;
               --  Once the inliers stand (none changed in this pass), what the model leaves unexplained, by
               --  keyframe, for the log: the root mean square of the inliers' residuals in pixels. A keyframe
               --  that stands out is a pose the readings do not describe; a residual that grows with the size
               --  of the move is a model short of a term (the kiwi, R6's tenth start, 10-14: 1.7 px left at a
               --  focal length the search could not tell, against an aligner erring 0.12 px).
               if Now_Changed = 0 and then Kf > 0 then
                  declare
                     Sum : Real_Array (1 .. Frames) := [others => 0.0];
                     Cnt : array (1 .. Frames) of Natural := [others => 0];
                     Text : Ada.Strings.Unbounded.Unbounded_String;
                  begin
                     for I in 1 .. S loop
                        if Inlier (I) then
                           declare
                              F : constant Positive := Sight (Sight'First + I - 1).Frame;
                           begin
                              if F <= Frames then
                                 Sum (F) := Sum (F) + All_R (2 * I - 1) ** 2 + All_R (2 * I) ** 2;
                                 Cnt (F) := Cnt (F) + 1;
                              end if;
                           end;
                        end if;
                     end loop;
                     for F in 1 .. Frames loop
                        if Cnt (F) > 0 then
                           Ada.Strings.Unbounded.Append
                             (Text, " f" & Ada.Strings.Fixed.Trim (F'Image, Ada.Strings.Left) & " "
                              & Driver.Log.Image (Sqrt (Sum (F) / Real (Cnt (F))), 2)
                              & " (" & Ada.Strings.Fixed.Trim (Cnt (F)'Image, Ada.Strings.Left) & ")");
                        end if;
                     end loop;
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: the tracks' fit leaves, by keyframe, px rms over"
                                      & Kf'Image & " inliers of" & S'Image & " sightings at focal "
                                      & Driver.Log.Image (L.Fx, 1) & " px (floor " & Driver.Log.Image (Sigma, 2) & "):"
                                      & Ada.Strings.Unbounded.To_String (Text));
                  end;
               end if;
               if Kf > 0 then
                  Fit_Abs := new Real_Array (1 .. Kf);
                  Kf := 0;
                  for I in 1 .. S loop
                     if Inlier (I) then
                        Kf := Kf + 1;
                        Fit_Abs (Kf) := Sqrt (All_R (2 * I - 1) ** 2 + All_R (2 * I) ** 2);
                     end if;
                  end loop;
                  Report.Median_Px := Driver.Stats.Median (Fit_Abs.all);
                  Free (Fit_Abs);
               end if;
               if S > 0 then
                  Fit_Abs := new Real_Array (1 .. S);
                  for I in 1 .. S loop
                     Fit_Abs (I) := Sqrt (All_R (2 * I - 1) ** 2 + All_R (2 * I) ** 2);
                  end loop;
                  Report.All_Median_Px := Driver.Stats.Median (Fit_Abs.all);
                  Report.All_Count := S;
                  --  And the robust cost of every sighting at the scales asked for, by which the focal length's profile
                  --  tells a step from the solution.
                  declare
                     Judge : constant Error_Model :=
                       (if Judge_Floor > 0.0 then (Floor => Judge_Floor, Gain => Judge_Gain) else Model);
                     Cost  : Real := 0.0;
                  begin
                     for I in 1 .. S loop
                        declare
                           Scale : constant Real := Scale_Of (Judge, All_Own (2 * I));
                        begin
                           if Scale > Real'Model_Small then
                              --  Beyond Z sigmas a residual counts as one at Z: a sighting that fits neither of two
                              --  fits counts alike in both (one with no point, its residual a placeholder, made the
                              --  desktop arm's solution 95345 above its own profile's steps, 10-20).
                              Cost := Cost + Real'Min (Huber_Cost (All_R (2 * I - 1) / Scale), Huber_Cost (Driver.Conventions.Z))
                                + Real'Min (Huber_Cost (All_R (2 * I) / Scale), Huber_Cost (Driver.Conventions.Z));
                           end if;
                        end;
                     end loop;
                     Report.Cost_Judged := Cost;
                     Report.Judge_Floor := Judge.Floor;
                     Report.Judge_Gain := Judge.Gain;
                  end;
                  Free (Fit_Abs);
               end if;
               Free (All_Index);
               Free (All_R);
               Free (All_Own);
               exit when Now_Changed = 0 or else Passes = Began'Last
                 or else (for some Earlier of Began (1 .. Passes) => Earlier.all = Inlier.all);
            end;
         end loop;
         --  What the stage's time was made of (B0a: 14 to 115 s a stage on 95 workers, the rounds unknown).
         Driver.Log.Line (Driver.Log.Robot, "kinematics: the tracks' fit took" & Passes'Image & " passes and"
                          & Rounds'Image & " rounds over" & S'Image & " sightings of" & Tracks'Image & " tracks,"
                          & Count'Image & " parameters");
         --  The covariance of the last pass's estimate, from what the pass
         --  kept. Too few keyframes or points, or no point seen twice, to tell
         --  how the errors depend on each other: not determined.
         if Last_Pass.Ready then
            declare
               Reduced : constant Positive := Variance'Length;
               Kinds   : Errors.Model;
            begin
               Errors.Measure (Last_Pass.In_Frame.all, Last_Pass.Of_Track.all, Last_Pass.Ref_U.all,
                               Last_Pass.Ref_V.all, Last_Pass.Error.all, Kinds);
               if Errors.Measured (Kinds) then
                  declare
                     Meat    : Real_Matrix (1 .. Reduced, 1 .. Reduced);
                     V       : Real_Matrix (1 .. Reduced, 1 .. Reduced);
                     Clipped : Real;
                     Ok      : Boolean;
                  begin
                     Errors.Meat (Kinds, Last_Pass.In_Frame.all, Last_Pass.Of_Track.all, Last_Pass.Ref_U.all,
                                  Last_Pass.Ref_V.all, Last_Pass.Rows.all, Meat);
                     Sandwich (Last_Pass.Inv.all, Meat, V, Clipped, Ok);
                     if Ok then
                        for P in 1 .. Reduced loop
                           Variance (P) := V (P, P);
                           for Q in 1 .. Reduced loop
                              Report.Covariance.Append (V (P, Q));
                           end loop;
                        end loop;
                        Report.Errors := Summary_Of (Kinds, Clipped);
                     else
                        Determined := False;
                        Why_Not := Ada.Strings.Unbounded.To_Unbounded_String
                          ("the sightings' covariance has no positive semi-definite form");
                     end if;
                  end;
               else
                  Determined := False;
                  Why_Not := Ada.Strings.Unbounded.To_Unbounded_String
                    ("the sightings are too few to tell how their errors depend on each other");
               end if;
               Errors.Free (Kinds);
            end;
            Release (Last_Pass);
         end if;
         L := Lens_Of (X);
         Joints := Joints_Of (X);
         --  Fitted only when the sightings determine every parameter that has a
         --  value of its own: each focal length and each reading scale
         --  significant against its uncertainty, and each axis known to a
         --  cone whose Z-sigma edge stays within a quarter turn of it.
         Report.Determined := Determined;
         Report.Pinhole := Pinhole;
         if not Determined then
            Report.Why := Why_Not;
         else
            Report.Focal_Sigma := L.Fx * Sqrt (Variance (1));
            if not Driver.Uncertain.Significant (L.Fx, L.Fx * Sqrt (Variance (1)))
              or else not Driver.Uncertain.Significant (L.Fy, L.Fy * Sqrt (Variance (if Pinhole then 1 else 2)))
            then
               Report.Determined := False;
               Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                 ("the sightings leave the focal lengths undetermined: " & Real'Image (L.Fx) & " +-"
                  & Real'Image (L.Fx * Sqrt (Variance (1))) & " and " & Real'Image (L.Fy) & " +-"
                  & Real'Image (L.Fy * Sqrt (Variance (2))) & " px");
            end if;
            for J in 1 .. N loop
               declare
                  O    : constant Natural := 6 + 5 * (J - 1);
                  Tilt : constant Real := Sqrt (Variance (O + 1) + Variance (O + 2));
                  C    : constant Real := Joints (Joints'First + J - 1).C;
               begin
                  if Report.Determined
                    and then not Joint_Determined (Variance (O + 1), Variance (O + 2), Variance (O + 5), C)
                  then
                     declare
                        --  What the joint had to show itself by: the sightings that fit in the keyframes that moved
                        --  it, and the most it turned in them by the scale it has now.
                        Seen_Moving : Natural := 0;
                        Swept       : Real := 0.0;
                     begin
                        for I in 1 .. S loop
                           declare
                              Move : constant Real :=
                                Changes (Changes'First (1) + Sight (Sight'First + I - 1).Frame - 1, Changes'First (2) + J - 1);
                           begin
                              if Inlier (I) and then Move /= 0.0 then
                                 Seen_Moving := Seen_Moving + 1;
                                 Swept := Real'Max (Swept, abs (C * Move));
                              end if;
                           end;
                        end loop;
                        Report.Determined := False;
                        Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                          ("the sightings leave joint" & J'Image & " undetermined: its axis to" & Real'Image (Tilt)
                           & " rad, its scale" & Real'Image (C) & " +-" & Real'Image (Sqrt (Variance (O + 5)))
                           & "; of the sightings that fit," & Seen_Moving'Image & " are in keyframes that moved it, at most"
                           & Real'Image (Swept) & " rad by that scale");
                     end;
                  end if;
               end;
            end loop;
         end if;
         Report.Depths.Clear;
         Report.Depth_Sigmas.Clear;
         for T in 1 .. Tracks loop
            Report.Depths.Append ((if Has (T) then Exp (Depth (T)) else 0.0));
            Report.Depth_Sigmas.Append ((if Has (T) then Depth_Sigma (T) else Real'Last));
         end loop;
         Free (Depth);
         Free (Depth_Sigma);
         Free (Has);
         for Choice of Began loop
            Free (Choice);
         end loop;
         Free (Inlier);
      end;
   end Refine_Tracks;

   ---------------------------------------------------------------------------
   --  The fit

   --  For a fit studied alone (tools/fit_replay): with BD_DUMP_FITS naming a folder, every fit's inputs, in a file
   --  of what is fitted and the fit's number in the run, every real to its last bit.
   protected Dumps is
      procedure Next (Number : out Positive);
   private
      Count : Natural := 0;
   end Dumps;

   protected body Dumps is
      procedure Next (Number : out Positive) is
      begin
         Count := Count + 1;
         Number := Count;
      end Next;
   end Dumps;

   procedure Dump_Inputs
     (Changes : Driver.Numerics.Arrays.Real_Matrix; Asked : Flag_Matrix; Visible : Real_Array;
      Sightings : Sighting_Array; Width, Height : Positive; Unit_Frames : Natural; Who : String; From : Fit_Start)
   is
      use Ada.Text_IO;
      File   : File_Type;
      Number : Positive;
      Name   : String := Who;
      procedure Put_Real (X : Real) is
      begin
         Put (File, " ");
         Ada.Long_Float_Text_IO.Put (File, X, Fore => 1, Aft => Real'Digits + 1, Exp => 1);
      end Put_Real;
      procedure Put_Flag (B : Boolean) is
      begin
         Put (File, Natural'Image (Boolean'Pos (B)));
      end Put_Flag;
   begin
      for C of Name loop
         if C = ' ' then
            C := '_';
         end if;
      end loop;
      Dumps.Next (Number);
      Create (File, Out_File, Ada.Environment_Variables.Value ("BD_DUMP_FITS") & "/" & Name & "_"
              & Ada.Strings.Fixed.Trim (Number'Image, Ada.Strings.Left) & ".txt");
      Put_Line (File, "who " & Who);
      Put_Line (File, "sizes" & Natural'Image (Changes'Length (1)) & Natural'Image (Visible'Length)
                & Natural'Image (Sightings'Length) & Width'Image & Height'Image & Unit_Frames'Image);
      Put_Line (File, "changes");
      for I in Changes'Range (1) loop
         for J in Changes'Range (2) loop
            Put_Real (Changes (I, J));
         end loop;
         New_Line (File);
      end loop;
      Put_Line (File, "asked");
      for I in Asked'Range (1) loop
         for J in Asked'Range (2) loop
            Put_Flag (Asked (I, J));
         end loop;
         New_Line (File);
      end loop;
      Put_Line (File, "visible");
      for X of Visible loop
         Put_Real (X);
      end loop;
      New_Line (File);
      Put_Line (File, "sightings");
      for X of Sightings loop
         Put (File, X.Frame'Image & X.Track'Image);
         Put_Real (X.U0);
         Put_Real (X.V0);
         Put_Real (X.U);
         Put_Real (X.V);
         Put_Real (X.Own);
         Put (File, X.Own_Freedom'Image);
         Put_Real (X.Off_U);
         Put_Real (X.Off_V);
         Put_Flag (X.Off_Told);
         New_Line (File);
      end loop;
      Put_Line (File, "from" & From.N'Image);
      Put_Flag (From.Valid);
      Put_Flag (From.Whole);
      Put_Flag (From.Order_Told);
      Put (File, From.Profiled_Over'Image);
      Put_Real (From.Focal_Within);
      Put_Real (From.Searched_Least);
      Put_Real (From.Searched_Most);
      New_Line (File);
      Put_Real (From.L.Fx);
      Put_Real (From.L.Fy);
      Put_Real (From.L.Cx);
      Put_Real (From.L.Cy);
      Put_Real (From.L.K1);
      Put_Real (From.L.K2);
      New_Line (File);
      for J of From.Joints loop
         for X of J.W loop
            Put_Real (X);
         end loop;
         for X of J.P loop
            Put_Real (X);
         end loop;
         Put_Real (J.C);
         Put_Flag (J.Slide);
         New_Line (File);
      end loop;
      for C of From.Chain loop
         Put (File, C'Image);
      end loop;
      New_Line (File);
      for T of From.Slide_Turns loop
         for X of T loop
            Put_Real (X);
         end loop;
      end loop;
      New_Line (File);
      Close (File);
   end Dump_Inputs;

   procedure Fit
     (Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Asked      : Flag_Matrix;
      Visible    : Real_Array;
      Sightings  : Sighting_Array;
      Width, Height : Positive;
      Unit_Frames : Natural;
      Joints     : out Joint_Array;
      Chain      : out Chain_Order;
      L          : out Lens;
      Report     : out Fit_Report;
      Who        : String := "";
      From       : Fit_Start := No_Start)
   is
      N      : constant Natural := Visible'Length;
      Frames : constant Natural := Changes'Length (1);
      --  The keyframes that define the unit of length.
      Unit_Count : constant Natural := (if Unit_Frames = 0 then Frames else Unit_Frames);
      S      : constant Natural := Sightings'Length;
      Sight  : Sighting_Array renames Sightings;
      type Sighting_Access is access Sighting_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Sighting_Array, Sighting_Access);

      function Change (Frame, J : Positive) return Real is
        (Changes (Changes'First (1) + Frame - 1, Changes'First (2) + J - 1));

      function Changes_Of (Frame : Positive) return Real_Array is
         D : Real_Array (1 .. N);
      begin
         for J in 1 .. N loop
            D (J) := Change (Frame, J);
         end loop;
         return D;
      end Changes_Of;

      --  The keyframe turned joint J: the sweep asked it to (Asked), and its reading changed by a step its eye can
      --  see. What the sweep asked tells which joint a keyframe is of; the readings alone cannot: a servo sent
      --  back to the sweep's start lands within its dead band of it, two or three units on the kiwi against a
      --  matcher's step of two (R6's sixth and seventh starts, 10-14), and by the readings every such landing was
      --  a turn of that joint, no keyframe was any other joint's own, and a joint that never turned had two
      --  hundred tracks of its own from its reading's jitter. A median rest over the keyframes (the fix before
      --  this one) left the landings at the matcher's step still counted. The model below uses the readings as
      --  they are.
      function Moved (Frame, J : Positive) return Boolean is
        (Asked (Asked'First (1) + Frame - 1, Asked'First (2) + J - 1)
         and then Visible (Visible'First + J - 1) > 0.0
         and then abs Change (Frame, J) >= Visible (Visible'First + J - 1));

      --  A joint's reading in the keyframe stands from the reference's by a step its eye can see, asked or not: a
      --  joint the sweep left where an earlier push took it is turned from the reference as much as one it asks now
      --  (f15c on 10-15: arm 1's keyframe 51 asked joint 6 by 6.2 units with joint 5 left at -24.5; taken for joint
      --  6 alone, its two-joint motion fitted no turn of joint 6, that joint's score at stage 1 was three times the
      --  others' and fell with the focal length, and the search put a 194 px lens at 12 px). A landing error stays
      --  below the step, which is floored at the least turn the body executes (Step_Of in Kinematics).
      function Away (Frame, J : Positive) return Boolean is
        (Visible (Visible'First + J - 1) > 0.0 and then abs Change (Frame, J) >= Visible (Visible'First + J - 1));

      --  The keyframe stands from the reference by two joints or more, as far as their eyes can tell, asked now or
      --  left there before: what tells the joints' order.
      function Several (Frame : Positive) return Boolean is
        (for some J in 1 .. N => Away (Frame, J) and then (for some K in J + 1 .. N => Away (Frame, K)));

      --  Each keyframe's joint of its own, and the keyframe it shows that joint's turn against: the reference when
      --  every other joint stands where it stood there or its motion is taken out (Alone); else, of the keyframes
      --  with sightings, the one that differs from it in that joint alone and by the most. A servo sent back to the
      --  sweep's start lands within its dead band of it, so two landings differ by up to twice that, and the reference
      --  is a landing too (f15c on 10-15, arm 2: the joints not asked stood 4 and 8 units from the reference at a dead
      --  band of 4, and five joints of six had no keyframe of their own against it). Two pushes of one joint from one
      --  start share every other joint's landing. Stages 1 and 2 fit one joint at a time and take a keyframe's
      --  sightings against its partner (the point's place there: Pairs); the stages after take them against the
      --  reference, with every joint's reading.
      Own_Joint : array (1 .. Frames) of Natural := [others => 0];
      Partner   : array (1 .. Frames) of Natural := [others => 0];

      function Own_Change (Frame, J : Positive) return Real is
        (Change (Frame, J) - (if Partner (Frame) > 0 then Change (Partner (Frame), J) else 0.0));

      --  Joint J alone differs between keyframes A and B, as far as their eyes can tell, and B asked it to turn.
      function Pair_Only (A, B, J : Positive) return Boolean is
        (Asked (Asked'First (1) + B - 1, Asked'First (2) + J - 1)
         and then Visible (Visible'First + J - 1) > 0.0
         and then abs (Change (B, J) - Change (A, J)) >= Visible (Visible'First + J - 1)
         and then (for all C in 1 .. N =>
                     C = J or else Visible (Visible'First + C - 1) <= 0.0
                     or else abs (Change (B, C) - Change (A, C)) < Visible (Visible'First + C - 1)));

      Pairs : Sighting_Access;

      Lens0  : Lens := (Fx | Fy => 1.0, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, others => 0.0);
      --  Half the width, in the logarithm, the focal search of stage 1 could tell the focal length within
      --  (its ended bracket); 0 before the search. The last stage takes the search's focal length as a
      --  measurement of this width (Refine_Tracks).
      Focal_Within : Real := 0.0;
      --  The focal lengths the search of stage 1 ranged over, least and most: how far the last stage's profile of
      --  the focal length may go (Profile_Focal).
      Searched_Least, Searched_Most : Real := 0.0;
      Counts : array (1 .. N) of Natural := [others => 0];
      --  The joints along the arm, from the base (stage 4 finds it; the joints alone are fitted before it).
      Order  : Chain_Order (1 .. N) := [for K in 1 .. N => K];
      Usable : Flags (1 .. N) := [others => False];
      --  Every track's depth from the keyframes of one joint each (stage 4's start), which no order changes.
      Alone_Depths : Real_Lists.Vector;
      --  Per joint taken for a slide, the turn per reading unit (its axis times its rate) its own keyframes were
      --  explained by nearly as well: stage 1's, or the fit's begun from (Fit_Start); weighed against the slide once
      --  the lens is refined (stage 4's start).
      Slide_Turns : Vec3_Array (1 .. N) := [others => Zero3];

      --  How long each stage took, for the boot's account of where its time went (A98's second arm sat in its fit
      --  for more than 45 minutes with nothing to say where).
      Stage_Began : Ada.Calendar.Time := Ada.Calendar.Clock;
      procedure Close_Stage is
         use type Ada.Calendar.Time;
         Now : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         if Report.Stage > 0 then
            Report.Seconds.Append (Real (Now - Stage_Began));
         end if;
         Stage_Began := Now;
      end Close_Stage;
      procedure Enter (Stage : Positive) is
      begin
         Close_Stage;
         Report.Stage := Stage;
         if Who /= "" then
            Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " enters stage" & Stage'Image & " of its fit"
                             & (if Report.Seconds.Is_Empty then ""
                                else ", the stage before it having taken "
                                     & Driver.Log.Image (Report.Seconds.Last_Element, 1) & " s")
                             & ", the focal length " & Driver.Log.Image (Lens0.Fx, 1) & " px");
         end if;
      end Enter;

      procedure Fail (Why : String) is
      begin
         Free (Pairs);
         Report.Why := Ada.Strings.Unbounded.To_Unbounded_String (Why);
         L := Lens0;
         Close_Stage;
      end Fail;

      --  The eye at every keyframe under the joints.
      subtype Pose_Array is Rigid_Array (1 .. Frames);
      procedure Poses_Of (Jx : Joint_Array; P : out Pose_Array; C : Chain_Order := Order) is
      begin
         for Frame in 1 .. Frames loop
            P (Frame) := Eye_At (Jx, C, Changes_Of (Frame));
         end loop;
      end Poses_Of;

      --  Lengths in model units: the eye positions over the unit's keyframes
      --  have a root mean square of one.
      procedure Normalize (Jx : in out Joint_Array; C : Chain_Order := Order) is
         Sum : Real := 0.0;
      begin
         for Frame in 1 .. Unit_Count loop
            declare
               T : constant Vec3 := Eye_At (Jx, C, Changes_Of (Frame)).Translation;
            begin
               Sum := Sum + T * T;
            end;
         end loop;
         if Sum > 0.0 then
            declare
               F : constant Real := 1.0 / Sqrt (Sum / Real (Unit_Count));
            begin
               for J of Jx loop
                  if J.Slide then
                     J.C := F * J.C;
                  else
                     J.P := F * J.P;
                  end if;
               end loop;
            end;
         end if;
      end Normalize;

      --  The last of the fit, from the joints, their order and the lens the stages before it left: every
      --  translation's sign, the tracks (stage 5), the focal length's profile, the lengths in model units; and
      --  what a fit begun from this one takes up (From: the focal search's width and range). Profiled: whether the
      --  focal length's profile is walked. It belongs to the fit that searched: a fit begun from it within the
      --  same estimate (From) starts in the basin that profile chose, on a few more sightings, and walked it again
      --  at most of its time (the profile is most of the last stage: ten refinements of the tracks to its one).
      --  Whether the sightings tell the joints' order (stage 4), or the fit begun from tells it (From).
      Order_Is_Told : Boolean := False;
      Untold        : Joint_Pair_Lists.Vector;   --  the joints next to each other in it whose swap stage 4 did not tell
      Untold_Orders : Joint_Lists.Vector;        --  every order stage 4 did not tell from it (Fit_Report.Untold_Orders)
      Untold_Turns : Real_Lists.Vector;          --  per pair of Untold, its two joints' turns that tell it best

      --  The every-joint fit of the joints Jx in order C (the text where it is called says what it is and why): Jx in
      --  model units after it, the noise its sightings that fit leave (Sigma_Out), how many fit, and the median
      --  residual of those (Median). A procedure for the order search to fit each order it weighs as the fit does its
      --  own (Search_Order): the tracks' refinement begun from the joints of the keyframes of one joint each, in an
      --  order those keyframes cannot tell, set the keyframes of several joints aside as not fitting and never
      --  fitted them (the bench's s36, 10-20: 1.2 px on them, the same order fitted after this 0.4).
      procedure Together
        (C : Chain_Order; Jx : in out Joint_Array; Lx : Lens; Sigma_Out : out Real; Used_Count : out Natural;
         Median : out Real)
      is
         Per_Joint : constant := 6;
         Count     : constant Positive := Per_Joint * N;
         Inlier    : Flags_Access := new Flags'(1 .. S => False);
         Base      : Joint_Array := Jx;
         --  The choices of sightings made so far: a round that comes back to one of them is a cycle.
         package Selection_Lists is new Ada.Containers.Vectors (Positive, Flags_Access);
         procedure Free_Flags is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
         Seen      : Selection_Lists.Vector;


         function Joints_Of (X : Real_Array) return Joint_Array is
            Out_J : Joint_Array := Base;
         begin
            for J in 1 .. N loop
               declare
                  O : constant Natural := X'First + Per_Joint * (J - 1);
                  E1, E2 : Vec3;
               begin
                  Perp (Base (Base'First + J - 1).W, E1, E2);
                  Out_J (Out_J'First + J - 1).W := Unit (Base (Base'First + J - 1).W + X (O) * E1 + X (O + 1) * E2);
                  Out_J (Out_J'First + J - 1).P := [X (O + 2), X (O + 3), X (O + 4)];
                  Out_J (Out_J'First + J - 1).C := X (O + 5);
               end;
            end loop;
            return Out_J;
         end Joints_Of;

         procedure To_X (Jx : Joint_Array; X : out Real_Array) is
         begin
            for J in 1 .. N loop
               declare
                  O : constant Natural := X'First + Per_Joint * (J - 1);
               begin
                  X (O) := 0.0;
                  X (O + 1) := 0.0;
                  X (O + 2) := Jx (Jx'First + J - 1).P (1);
                  X (O + 3) := Jx (Jx'First + J - 1).P (2);
                  X (O + 4) := Jx (Jx'First + J - 1).P (3);
                  X (O + 5) := Jx (Jx'First + J - 1).C;
               end;
            end loop;
         end To_X;

         procedure Residuals_Of (Lx : Lens; Jx : Joint_Array; R : out Real_Array) is
            Poses : Rigid_Access := new Pose_Array;
         begin
            Poses_Of (Jx, Poses.all, C);
            for I in 1 .. S loop
               declare
                  Rr : Mat3;
                  Tt : Vec3;
               begin
                  Relative (Poses (1), Poses (Sight (I).Frame), Rr, Tt);
                  R (R'First + I - 1) := Sampson (Rr, Tt, Ray (Lx, Sight (I).U0, Sight (I).V0),
                                                  Ray (Lx, Sight (I).U, Sight (I).V), (Lx.Fx + Lx.Fy) / 2.0);
               end;
            end loop;
            Free (Poses);
         end Residuals_Of;

         X : Real_Array (1 .. Count);
      begin
         Sigma_Out := 0.0;
         Used_Count := 0;
         To_X (Jx, X);
         loop
            declare
               All_R : Real_Access := new Real_Array (1 .. S);
               Sigma : Real;
               Now_Changed : Natural := 0;
               Used  : Natural := 0;
            begin
               Residuals_Of (Lx, Joints_Of (X), All_R.all);
               Sigma := Noise_Of (All_R.all);
               --  A sighting fits when its residual is not significant against
               --  the noise of all of them.
               for I in 1 .. S loop
                  declare
                     Fits : constant Boolean := Sigma > 0.0 and then not Driver.Uncertain.Significant (All_R (I), Sigma);
                  begin
                     if Fits /= Inlier (I) then
                        Now_Changed := Now_Changed + 1;
                        Inlier (I) := Fits;
                     end if;
                     if Fits then
                        Used := Used + 1;
                     end if;
                  end;
               end loop;
               Free (All_R);
               Sigma_Out := Sigma;
               Used_Count := Used;
               --  Re-chosen until the choice no longer changes, or comes back to one made before (a cycle between
               --  choices, each giving the same fit again: no round after it tells more; item 3), or too few are
               --  left to fit.
               exit when Now_Changed = 0 or else Used <= Count
                 or else (for some Earlier of Seen => Earlier.all = Inlier.all);
               Seen.Append (new Flags'(Inlier.all));
               declare
                  Kept : Sighting_Access := new Sighting_Array (1 .. Used);
                  K    : Natural := 0;
                  --  The lines of sight of every kept sighting's pixel and of its reference pixel through the lens the
                  --  stage holds, made once: made again in every column of the Jacobian (Ray, Newton's method through the
                  --  distortion) they were most of the stage's time (s29's driver sampled for 10 s in its fit, 10-17: 6165
                  --  of the 6686 samples of this Evaluate in Ray).
                  type Vec3_Array_Access is access Vec3_Array;
                  procedure Free is new Ada.Unchecked_Deallocation (Vec3_Array, Vec3_Array_Access);
                  Own_Lines : Vec3_Array_Access := new Vec3_Array (1 .. Used);
                  Ref_Lines : Vec3_Array_Access := new Vec3_Array (1 .. Used);
                  Focal     : constant Real := (Lx.Fx + Lx.Fy) / 2.0;

                  procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
                     Poses : Rigid_Access := new Pose_Array;
                     --  Per keyframe, its motion from the reference eye.
                     type Motion is record
                        Rr : Mat3;
                        Tt : Vec3;
                     end record;
                     type Motion_Array is array (1 .. Frames) of Motion;
                     type Motion_Access is access Motion_Array;
                     procedure Free is new Ada.Unchecked_Deallocation (Motion_Array, Motion_Access);
                     Motions : Motion_Access := new Motion_Array;
                  begin
                     Poses_Of (Joints_Of (Xv), Poses.all, C);
                     for Frame in 1 .. Frames loop
                        Relative (Poses (1), Poses (Frame), Motions (Frame).Rr, Motions (Frame).Tt);
                     end loop;
                     for I in 1 .. Used loop
                        R (R'First + I - 1) := Sampson (Motions (Kept (I).Frame).Rr, Motions (Kept (I).Frame).Tt,
                                                        Ref_Lines (I), Own_Lines (I), Focal);
                     end loop;
                     Free (Motions);
                     Free (Poses);
                  end Evaluate;

                  procedure Solve is new Robust_Fit
                    (Count, Used, Driver.Conventions.Initial_Damping, Evaluate, At_Once => True, To_The_End => True);
               begin
                  for I in 1 .. S loop
                     if Inlier (I) then
                        K := K + 1;
                        Kept (K) := Sight (I);
                     end if;
                  end loop;
                  for I in 1 .. Used loop
                     Own_Lines (I) := Ray (Lx, Kept (I).U, Kept (I).V);
                     Ref_Lines (I) := Ray (Lx, Kept (I).U0, Kept (I).V0);
                  end loop;
                  Solve (X, Sigma);
                  Free (Kept);
                  Free (Own_Lines);
                  Free (Ref_Lines);
                  --  The axes' tangent planes anew at the solution, the points
                  --  on them nearest the reference eye.
                  Base := Joints_Of (X);
                  for J of Base loop
                     J.P := J.P - Dot (J.P, J.W) * J.W;
                  end loop;
                  To_X (Base, X);
               end;
            end;
         end loop;
         for Earlier of Seen loop
            declare
               F : Flags_Access := Earlier;
            begin
               Free_Flags (F);
            end;
         end loop;
         Jx := Joints_Of (X);
         Normalize (Jx, C);
         declare
            All_R : Real_Access := new Real_Array (1 .. S);
            Fit_R : Real_Access := new Real_Array (1 .. S);
            K     : Natural := 0;
         begin
            Residuals_Of (Lx, Jx, All_R.all);
            for I in 1 .. S loop
               if Inlier (I) then
                  K := K + 1;
                  Fit_R (K) := abs All_R (I);
               end if;
            end loop;
            Median := 0.0;
            if K > 0 then
               declare
                  Part : Real_Access := new Real_Array'(Fit_R (1 .. K));
               begin
                  Median := Driver.Stats.Median (Part.all);
                  Free (Part);
               end;
            end if;
            Free (All_R);
            Free (Fit_R);
         end;
         Free (Inlier);
      end Together;

      procedure Finish (Profiled : Boolean) is
      begin
         --  The sign of every translation at once, which the epipolar constraint
         --  cannot tell: the matched points lie in front of both eyes.
         declare
            Front, Back : Natural := 0;
         begin
            for X of Sight loop
               declare
                  T   : constant Rigid := Eye_At (Joints, Order, Changes_Of (X.Frame));
                  A   : constant Vec3 := Ray (L, X.U0, X.V0);
                  Bv  : constant Vec3 := T.Rotation * Ray (L, X.U, X.V);
                  --  Lambda1 A = T.Translation + Lambda2 Bv, least squares.
                  Aa  : constant Real := A * A;
                  Ab  : constant Real := A * Bv;
                  Bb  : constant Real := Bv * Bv;
                  Ra  : constant Real := A * T.Translation;
                  Rb  : constant Real := Bv * T.Translation;
                  Det : constant Real := Aa * Bb - Ab * Ab;
               begin
                  if Det > 0.0 then
                     declare
                        L1 : constant Real := (Ra * Bb - Ab * Rb) / Det;
                        L2 : constant Real := (Ab * Ra - Aa * Rb) / Det;
                     begin
                        if L1 > 0.0 and then L2 > 0.0 then
                           Front := Front + 1;
                        elsif L1 < 0.0 and then L2 < 0.0 then
                           Back := Back + 1;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Who /= "" then
               Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " has" & Front'Image
                                & " sightings whose point lies in front of both eyes and" & Back'Image & " behind both"
                                & (if Back > Front then ": every translation is turned over" else ""));
            end if;
            if Back > Front then
               for J of Joints loop
                  if J.Slide then
                     J.C := -J.C;
                  else
                     J.P := -J.P;
                  end if;
               end loop;
               Report.Flipped := True;
            end if;
         end;

         --  Stage 5: the tracks.
         Enter (5);
         --  The lens's aspect and distortion are kept only when the fit with them is determined and they are significant
         --  together by its covariance (Extras_Significant: which holds how the sightings' errors depend on each other,
         --  as a fixed eye's distortion is kept, Hand_Eye); the pinhole otherwise, refined from the same start. Judged by
         --  the sightings' residuals instead, the median over every sighting kept the pinhole for the desktop arm's wide
         --  barrel lens, its centre 33 px off, and a likelihood ratio of their robust costs counts every sighting's
         --  error as its own, the errors a keyframe's points share and a point's in every keyframe as evidence (10-20).
         --  One axis of turn (a drone's, a wheeled base's carrying its eye over the floor) tells
         --  the focal length, the distortion and the turn's scale only along a valley, and the pixels' aspect not at all
         --  where the eye looks level: free, they left the bench's drones and mast eyes undetermined, 5 of 6 and 2 of 3,
         --  the focal length 73 +- 39 px of the true 171, where a pinhole told the mast eyes to 1 to 3 % (10-20).
         declare
            Joints_P : Joint_Array := Joints;
            L_P      : Lens;
            Report_P : Fit_Report := Report;
         begin
            Refine_Tracks (Changes, Sight, Joints, Order, L, Report, Focal_Within);
            if not (Report.Determined and then Extras_Significant (L, Report, N)) then
               --  The pinhole nearest the lens with them, refined from its joints: begun from the stage before, a
               --  wheeled base's head (bench seed 1, 10-20) went from 166 px to 25 (the truth 171) in 18 passes while
               --  the lens with them stayed at 166, and was fitted there.
               Joints_P := Joints;
               L_P := (Fx | Fy => Sqrt (L.Fx * L.Fy), Cx => L.Cx, Cy => L.Cy, others => 0.0);
               Refine_Tracks (Changes, Sight, Joints_P, Order, L_P, Report_P, Focal_Within, Pinhole => True);
               if Report_P.All_Count > 0 then
                  Joints := Joints_P;
                  L := L_P;
                  Report := Report_P;
               end if;
            end if;
         end;
         --  The focal length's profile. The covariance at the solution says how far the focal length can move within
         --  the basin the solution lies in, nothing of another basin along the direction the sightings tell worst:
         --  with a wide lens's barrel distortion free, the focal length, the distortion and the joints' turns per unit
         --  trade along a valley, and a desktop arm swept to 11.5 degrees settled at 330 px of 400, its sigma 4.5 px
         --  (robot.kinematics.desktop, 10-15). Each way from the solution, the focal length is held at steps
         --  doubling from Z of its own sigma (held within that sigma, the search's measurement of it), everything
         --  else refitted; a step whose sightings' median residual, over every sighting, lies below the solution's
         --  by more than Z of the two medians' standard errors is a better basin, refitted from there and profiled
         --  again; a step above by as much ends that way, as does the range the focal search ranged over.
         declare
            type Sign_Way is (Up, Down);
            Moved : Boolean := True;
            Base  : Real;
         begin
            --  Only a solution that would be reported fitted is profiled: one the sightings leave undetermined is refitted
            --  when the evidence grows, and its profile would cost the boot that time for nothing.
            while Profiled and then Moved and then Report.Determined and then Report.All_Count > 0
              and then Report.Focal_Sigma < Real'Last
              and then Report.Focal_Sigma > 0.0 and then Searched_Least > 0.0
            loop
               Moved := False;
               Base := Report.All_Median_Px;
               declare
                  Held     : constant Real := Report.Focal_Sigma / L.Fx;   --  the focal length's own sigma, in the logarithm
                  Base_Err : constant Real := Median_Error (Base, Report.All_Count);
               begin
                  --  The two ways at once (Driver.Parallel_For), each from the solution, writing only its own result;
                  --  the way up's better basin is taken before the way down's, as when the way down was walked only
                  --  where the way up found none. Walked one after the other, the two took most of the last stage
                  --  (s02's fit replayed from its dump, 10-16: eleven steps of 63 to 456 rounds each, five up and six
                  --  down), and each step's own sums use the processors only part of the time.
                  declare
                     type Walk_Result is record
                        Moved  : Boolean := False;
                        L      : Lens;
                        Report : Fit_Report;
                        --  The way reached the end of the search's range, a step or more from the solution, with no
                        --  step's residual significantly above the solution's and none told from it by the keyframes
                        --  (Told_Apart): the profile is flat to Flat_To.
                        Flat    : Boolean := False;
                        Flat_To : Real := 0.0;
                        Told    : Boolean := False;   --  a step of the way the keyframes told from the solution
                     end record;
                     Walked        : array (Sign_Way) of Walk_Result;
                     Walked_Joints : array (Sign_Way) of Joint_Array (Joints'Range);

                     --  Whether the sightings tell a step from the solution: the robust cost of every sighting at the
                     --  solution's scales above the solution's by more than Z squared over two, the likelihood ratio of
                     --  one term at Z. The median over every sighting cannot where the keyframes that tell the focal
                     --  length are a few (a drone's turn, 14 of 33 keyframes, every one of whose sightings a step at
                     --  117 px set aside, the median 0.009 px above the solution's, the bench's drone, 10-20); each
                     --  keyframe's median cannot where a wide lens's distortion trades with its focal length over a
                     --  narrow sweep (robot.kinematics.desktop, 10-20: from 404 px to 358 no keyframe's median moved, the
                     --  median over all 1.727 against 1.726 px, where the covariance put the focal length at 401 +- 2.5).
                     function Told_Apart (Step : Fit_Report) return Boolean is
                       (Step.Cost_Judged - Report.Cost_Judged > Driver.Conventions.Z ** 2 / 2.0);

                     procedure Walk (Index : Positive) is
                        Way    : constant Sign_Way := Sign_Way'Val (Index - 1);
                        Offset : Real := Driver.Conventions.Z * Held;
                        Last_F : Real := 0.0;   --  the focal length of the last step taken, none yet
                        Was_Told : Boolean := False;   --  the step before this one was told apart
                     begin
                        loop
                           declare
                              Factor : constant Real := Exp ((if Way = Up then Offset else -Offset));
                              Lp     : Lens := L;
                              Jp     : Joint_Array := Joints;
                              Rp     : Fit_Report;
                           begin
                              --  Each way ends past the range searched on its own side: a solution below the range walks
                              --  up into it (the base's head above, at 25 px of the range's least 39, ended its walk at
                              --  its first step and was fitted there).
                              if (Way = Down and then L.Fx * Factor < Searched_Least)
                                or else (Way = Up and then L.Fx * Factor > Searched_Most)
                              then
                                 Walked (Way).Flat := Last_F > 0.0 and then not Walked (Way).Told;
                                 Walked (Way).Flat_To := Last_F;
                                 exit;
                              end if;
                              Lp.Fx := L.Fx * Factor;
                              Lp.Fy := L.Fy * Factor;
                              Refine_Tracks (Changes, Sight, Jp, Order, Lp, Rp, Held, Spread => False,
                                             Pinhole => Report.Pinhole, Judge_Floor => Report.Judge_Floor,
                                             Judge_Gain => Report.Judge_Gain);
                              exit when Rp.All_Count = 0;
                              --  Where the step's focal length went, held within its own sigma: a profile is flat as far
                              --  as its steps went untold, not as far as they were asked (the sightings pulled a step at
                              --  315 px back to 178, s22's arm 2, 10-20).
                              Last_F := Lp.Fx;
                              declare
                                 Err : constant Real :=
                                   Sqrt (Median_Error (Rp.All_Median_Px, Rp.All_Count) ** 2 + Base_Err ** 2);
                              begin
                                 Driver.Log.Line (Driver.Log.Robot, "kinematics: the focal length's profile at "
                                                  & Driver.Log.Image (Lp.Fx, 1) & " px leaves a median "
                                                  & Driver.Log.Image (Rp.All_Median_Px, 3) & " px over every sighting, "
                                                  & "against " & Driver.Log.Image (Base, 3) & " px at "
                                                  & Driver.Log.Image (L.Fx, 1) & " (+- " & Driver.Log.Image (Err, 3) & ")"
                                                  & "; its cost over every sighting " & Driver.Log.Image (Rp.Cost_Judged - Report.Cost_Judged, 1)
                                                  & " above the solution's" & (if Told_Apart (Rp) then ", told apart" else ""));
                                 Walked (Way).Told := Walked (Way).Told or else Told_Apart (Rp);
                                 --  A better basin: the step's cost below the solution's by a likelihood ratio at Z,
                                 --  refitted from there, the focal length free within the search's width, and taken when
                                 --  it stays below. The way ends at the second step told apart in a row, or where a step
                                 --  leaves every sighting a median significantly above the solution's: one step past a ridge
                                 --  falls into what lies beyond it or rises with the wall. Ended at its first step told
                                 --  apart, the walk kept the desktop arm's wide lens at 327 px, between steps 8 and 11 above
                                 --  the solution, where the next step (399 px) lay 884 below it (robot.kinematics.desktop,
                                 --  10-20); ended by the median alone, which a step barely moves, the head rig's arm walked
                                 --  twelve steps of 7693 sightings, to 1400 above the solution's cost, and the order exams
                                 --  ran past the two-minute cap.
                                 if Report.Cost_Judged - Rp.Cost_Judged > Driver.Conventions.Z ** 2 / 2.0 then
                                    Refine_Tracks (Changes, Sight, Jp, Order, Lp, Rp, Focal_Within, Pinhole => Report.Pinhole,
                                                   Judge_Floor => Report.Judge_Floor, Judge_Gain => Report.Judge_Gain);
                                    if Rp.All_Count > 0 and then Rp.Cost_Judged < Report.Cost_Judged then
                                       Walked (Way) := (Moved => True, L => Lp, Report => Rp, others => <>);
                                       Walked_Joints (Way) := Jp;
                                    end if;
                                    exit;
                                 end if;
                                 exit when Rp.All_Median_Px > Base + Driver.Conventions.Z * Err
                                   or else (Was_Told and then Told_Apart (Rp));
                                 Was_Told := Told_Apart (Rp);
                              end;
                              Offset := 2.0 * Offset;
                           end;
                        end loop;
                     end Walk;
                     procedure Walks is new Driver.Parallel_For (Walk);
                  begin
                     Walks (1, Sign_Way'Pos (Sign_Way'Last) + 1);
                     for Way in Sign_Way loop
                        if Walked (Way).Moved then
                           --  The refit's report in place of the solution's, the fit's own account kept: the stage it
                           --  reached, the seconds of each, the sign of its translations (a refit's report has none of
                           --  them, and a drone's fit that moved to a better basin said it stopped at stage 0, 10-20).
                           declare
                              Stage_Was   : constant Natural := Report.Stage;
                              Seconds_Was : constant Real_Lists.Vector := Report.Seconds;
                              Flipped_Was : constant Boolean := Report.Flipped;
                           begin
                              L := Walked (Way).L;
                              Joints := Walked_Joints (Way);
                              Report := Walked (Way).Report;
                              Report.Stage := Stage_Was;
                              Report.Seconds := Seconds_Was;
                              Report.Flipped := Flipped_Was;
                           end;
                           Moved := True;
                           exit;
                        end if;
                     end loop;
                     --  A profile that stays as low as the solution's to the end of the range the search ranged over
                     --  leaves the focal length undetermined there, whatever the covariance at the solution says: its
                     --  curvature is the basin's, not the valley's. A drone's eye (the fake robot's, its motors moving
                     --  it along three axes and turning it about the upright) was fitted at 67 px for the true 163, as
                     --  low in residual, and said fitted (10-20).
                     if not Moved then
                        for Way in Sign_Way loop
                           if Walked (Way).Flat then
                              Report.Determined := False;
                              Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                                ("the sightings leave the focal length undetermined: its profile from "
                                 & Driver.Log.Image (L.Fx, 1) & " px stays as low as there to "
                                 & Driver.Log.Image (Walked (Way).Flat_To, 1) & " px, the end of the range searched");
                              Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Ada.Strings.Unbounded.To_String (Report.Why));
                              exit;
                           end if;
                        end loop;
                     end if;
                  end;
               end;
            end loop;
         end;
         declare
            Sum : Real := 0.0;
         begin
            for Frame in 1 .. Unit_Count loop
               declare
                  T : constant Vec3 := Eye_At (Joints, Order, Changes_Of (Frame)).Translation;
               begin
                  Sum := Sum + T * T;
               end;
            end loop;
            Normalize (Joints);
            --  The depths follow the lengths into model units.
            if Sum > 0.0 then
               for D of Report.Depths loop
                  D := D / Sqrt (Sum / Real (Unit_Count));
               end loop;
            end if;
            --  The covariance follows the lengths into model units.
            if Sum > 0.0 and then not Report.Covariance.Is_Empty then
               declare
                  F       : constant Real := 1.0 / Sqrt (Sum / Real (Unit_Count));
                  Reduced : constant Natural := Terms (N);
                  --  The terms that are lengths: the points, and a slide's scale.
                  Length  : array (1 .. Reduced) of Boolean := [others => False];
               begin
                  for J in 1 .. N loop
                     Length (Term_Of (J, Point_1)) := True;
                     Length (Term_Of (J, Point_2)) := True;
                     Length (Term_Of (J, Scale)) := Joints (Joints'First + J - 1).Slide;
                  end loop;
                  for P in 1 .. Reduced loop
                     for Q in 1 .. Reduced loop
                        declare
                           K : constant Positive := Report.Covariance.First_Index + (P - 1) * Reduced + Q - 1;
                           X : Real := Report.Covariance (K);
                        begin
                           if Length (P) then
                              X := F * X;
                           end if;
                           if Length (Q) then
                              X := F * X;
                           end if;
                           Report.Covariance.Replace_Element (K, X);
                        end;
                     end loop;
                  end loop;
               end;
            end if;
         end;
         Report.Focal_Within := Focal_Within;
         Report.Searched_Least := Searched_Least;
         Report.Searched_Most := Searched_Most;
         Report.Profiled_Over := (if Profiled then Sightings'Length elsif From.Valid then From.Profiled_Over else 0);
         Report.Order_Told := Order_Is_Told;
         Report.Untold := Untold;
         Report.Untold_Turns := Untold_Turns;
         Report.Untold_Orders := (if Order_Is_Told then Joint_Lists.Empty_Vector else Untold_Orders);
         Report.Slide_Turns.Clear;
         for J in 1 .. N loop
            declare
               Turn : constant Vec3 := (if Joints (Joints'First + J - 1).Slide then Slide_Turns (J) else Zero3);
            begin
               for X of Turn loop
                  Report.Slide_Turns.Append (X);
               end loop;
            end;
         end loop;
         Chain := Order;
         Report.Fitted := Report.Determined;
         Close_Stage;
      end Finish;

   begin
      if Ada.Environment_Variables.Exists ("BD_DUMP_FITS") then
         Dump_Inputs (Changes, Asked, Visible, Sightings, Width, Height, Unit_Frames, Who, From);
      end if;
      Report := (others => <>);
      Joints := [others => <>];
      Chain := Order;
      L := Lens0;
      if N = 0 or else S = 0 or else Frames < 2 then
         Fail ("no keyframes or no matches");
         return;
      end if;
      declare
         Sighted : array (1 .. Frames) of Boolean := [others => False];
         --  Every sighting of the keyframe has the motion of the joints it did not ask taken out (Off_Told).
         Offs_Told : array (1 .. Frames) of Boolean := [others => True];
         Tracks  : Natural := 0;

         --  The one joint the keyframe turned alone, 0 for none: it was asked to (Moved), and every other joint stands
         --  where it stood in the reference as far as its eye can tell, or was not asked and its motion is taken out
         --  of every sighting (Sighting's Off_U, Off_V), as the stages of one joint take the sightings. A servo lands
         --  off its target, and as the fit came to its joints' own steps those landings were steps its eye sees:
         --  v3 (servo seed 3 of the fake robot, replayed 10-18) had joint 4's three keyframes stand 6 and 12 units off
         --  in joints 1 and 2 against their steps of 4 and 8, every sighting with those joints' motion taken out,
         --  and once its pairs were turned every fit ended at stage 0 ("a joint has no keyframe of its own").
         function Alone (Frame : Positive) return Natural is
         begin
            for J in 1 .. N loop
               if Moved (Frame, J)
                 and then (for all C in 1 .. N =>
                             C = J or else not Away (Frame, C)
                             or else (Offs_Told (Frame)
                                      and then not Asked (Asked'First (1) + Frame - 1, Asked'First (2) + C - 1)))
               then
                  return J;
               end if;
            end loop;
            return 0;
         end Alone;
      begin
         for X of Sight loop
            Sighted (X.Frame) := True;
            Offs_Told (X.Frame) := Offs_Told (X.Frame) and then X.Off_Told;
            Tracks := Natural'Max (Tracks, X.Track);
         end loop;
         for B in 2 .. Frames loop
            if Sighted (B) then
               if Alone (B) > 0 then
                  Own_Joint (B) := Alone (B);
                  Partner (B) := 1;
               else
                  for J in 1 .. N loop
                     for A in 2 .. Frames loop
                        if A /= B and then Sighted (A) and then Pair_Only (A, B, J)
                          and then (Partner (B) = 0
                                    or else abs (Change (B, J) - Change (A, J))
                                            > abs (Change (B, Own_Joint (B)) - Change (Partner (B), Own_Joint (B))))
                        then
                           Own_Joint (B) := J;
                           Partner (B) := A;
                        end if;
                     end loop;
                  end loop;
               end if;
            end if;
         end loop;
         declare
            type Index_Grid is array (1 .. Frames, 1 .. Tracks) of Natural;
            type Grid_Access is access Index_Grid;
            procedure Free is new Ada.Unchecked_Deallocation (Index_Grid, Grid_Access);
            --  Where each keyframe saw each track: its sighting's index, 0 for none.
            Where : Grid_Access := new Index_Grid'(others => [others => 0]);
            Count : Natural := 0;
            --  A pair stands for its keyframe's own joint alone: each place where that joint alone would have put the
            --  point (Sighting's Off_U, Off_V: the joints the keyframe did not ask taken out), so both must be told.
            function Pairable (X : Sighting) return Boolean is
              (Own_Joint (X.Frame) > 0 and then X.Off_Told
               and then (Partner (X.Frame) = 1
                         or else (Where (Partner (X.Frame), X.Track) > 0
                                  and then Sight (Where (Partner (X.Frame), X.Track)).Off_Told)));
         begin
            for I in Sight'Range loop
               Where (Sight (I).Frame, Sight (I).Track) := I;
            end loop;
            for X of Sight loop
               if Pairable (X) then
                  Count := Count + 1;
               end if;
            end loop;
            Pairs := new Sighting_Array (1 .. Count);
            Count := 0;
            for X of Sight loop
               if Pairable (X) then
                  Count := Count + 1;
                  if Partner (X.Frame) = 1 then
                     Pairs (Count) := (X with delta U => X.U - X.Off_U, V => X.V - X.Off_V);
                  else
                     --  The point's place in the partner, and both places' uncertainty.
                     declare
                        P : Sighting renames Sight (Where (Partner (X.Frame), X.Track));
                     begin
                        Pairs (Count) :=
                          (X with delta U0 => P.U - P.Off_U, V0 => P.V - P.Off_V, U => X.U - X.Off_U, V => X.V - X.Off_V,
                                        Own => Sqrt (X.Own ** 2 + P.Own ** 2),
                                        Own_Freedom =>
                                          (if X.Own_Freedom = 0 or else P.Own_Freedom = 0
                                           then Natural'Max (X.Own_Freedom, P.Own_Freedom)
                                           else Natural'Min (X.Own_Freedom, P.Own_Freedom)));
                     end;
                  end if;
               end if;
            end loop;
            Free (Where);
         end;
      end;
      for X of Pairs.all loop
         Counts (Own_Joint (X.Frame)) := Counts (Own_Joint (X.Frame)) + 1;
      end loop;
      for J in 1 .. N loop
         Usable (J) := Counts (J) > 0;
      end loop;
      if (for some U of Usable => not U) then
         declare
            Which : Ada.Strings.Unbounded.Unbounded_String;
         begin
            for J in 1 .. N loop
               if not Usable (J) then
                  Ada.Strings.Unbounded.Append (Which, J'Image);
                  --  Each keyframe that asked the joint, and why none of its sightings stands for the joint alone: how
                  --  many points it saw, the joint it was taken for and against which keyframe (0: none), each other
                  --  joint that stands off the reference by a step its eye sees (by how much, against that step), and
                  --  how many of its sightings leave out a joint's motion that is not told (Off_Told). x3 (servo seed 3
                  --  of the fake robot, 10-18) failed here at every fit of its boot, its channel 4 with three keyframes
                  --  of its own asking, and the line said only which channel.
                  declare
                     Said : Ada.Strings.Unbounded.Unbounded_String;
                  begin
                     for B in 2 .. Frames loop
                        if Asked (Asked'First (1) + B - 1, Asked'First (2) + J - 1) then
                           declare
                              Seen, Untold : Natural := 0;
                           begin
                              for X of Sight loop
                                 if X.Frame = B then
                                    Seen := Seen + 1;
                                    if not X.Off_Told then
                                       Untold := Untold + 1;
                                    end if;
                                 end if;
                              end loop;
                              Ada.Strings.Unbounded.Append
                                (Said, "; keyframe" & B'Image & ":" & Seen'Image & " points, taken for joint"
                                 & Own_Joint (B)'Image & " against keyframe" & Partner (B)'Image & ","
                                 & Untold'Image & " of them with another joint's motion untold");
                              for C in 1 .. N loop
                                 if C /= J and then Away (B, C) then
                                    Ada.Strings.Unbounded.Append
                                      (Said, ", joint" & C'Image & " off by " & Driver.Log.Image (Change (B, C), 2)
                                       & " against its step " & Driver.Log.Image (Visible (Visible'First + C - 1), 2));
                                 end if;
                              end loop;
                           end;
                        end if;
                     end loop;
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " joint" & J'Image
                                      & " has no keyframe of its own, of those that asked it"
                                      & Ada.Strings.Unbounded.To_String (Said));
                  end;
               end if;
            end loop;
            Fail ("a joint has no keyframe of its own (channel" & Ada.Strings.Unbounded.To_String (Which) & ")");
         end;
         return;
      end if;
      --  Without a keyframe that moved two joints at once the order of the joints is not told and the fit cannot
      --  end (stage 4): it ends here, on the keyframes' readings alone, before the focal search of stage 1 spends
      --  its seconds on keyframes that cannot order the joints (R2 on the Pi, 10-13: 11 s, and a focal length of
      --  15052 px from the recognition's small pushes), and before the refinement, which from a focal length the
      --  joints alone did not tell can creep for most of an hour (A93's right arm at 3184 beats, its readings
      --  drawn: two replays on the box sat in it for more than half an hour).
      --  A fit of one order kept as given (From.Fixed) needs none: the joints of the keyframes that moved one each
      --  are the same in any order, and a search of the orders asks for them alone (10-20).
      if N > 1 and then not From.Fixed and then (for all X of Sight => not Several (X.Frame)) then
         Fail ("no keyframe moved two joints at once: the order of the joints along the arm is not told");
         return;
      end if;

      --  A fit begun from one of the same arm just made (From): its joints, their order along the arm and its lens,
      --  with its focal search's width and range, refined by the last stage alone on these sightings. Within one
      --  estimate the arm is fitted again each time keyframes are aligned again from its fit, a few sightings more
      --  each time, and every such fit searched the focal length from nothing again and ordered the joints again:
      --  an arm fitted six times in one estimate, 7395 to 7403 sightings, 20 to 28 s of search each (f15w
      --  replayed on 10-15); the search's seconds were nearly half of all the fits'.
      if From.Valid and then From.N = N and then not From.Whole then
         Free (Pairs);
         Joints := From.Joints;
         Order := From.Chain;
         L := From.L;
         Lens0 := From.L;
         Focal_Within := From.Focal_Within;
         Searched_Least := From.Searched_Least;
         Searched_Most := From.Searched_Most;
         Order_Is_Told := From.Order_Told;
         Slide_Turns := From.Slide_Turns;
         Finish (Profiled => False);
         return;
      end if;

      ---------------------------------------------------------------------------
      --  Stage 1: every joint alone.
      Enter (1);
      if From.Valid and then From.N = N and then From.Whole then
         --  Begun from the arm's fit of an estimate before this one: the search's result is that fit's joints, each
         --  axis with the side of the eye it lies on, and its lens; the stages after the search run on these
         --  sightings as ever, the order searched again (stage 4) and the focal length's profile walked (stage 5).
         --  Every estimate searched the focal length from nothing again: f16b's arm 1, 160 keyframes, 131 to 197 s a
         --  fit under the boot's load, and the sweep held still through 2700 of the 3000 beats of each pass.
         Lens0 := (Fx | Fy => Sqrt (From.L.Fx * From.L.Fy), Cx => From.L.Cx, Cy => From.L.Cy, others => 0.0);
         Focal_Within := From.Focal_Within;
         Searched_Least := From.Searched_Least;
         Searched_Most := From.Searched_Most;
         Slide_Turns := From.Slide_Turns;
         for J in 1 .. N loop
            declare
               Was  : Joint renames From.Joints (From.Joints'First + J - 1);
               Side : constant Vec3 := Was.P - Dot (Was.P, Was.W) * Was.W;
               E1, E2 : Vec3;
            begin
               Perp (Was.W, E1, E2);
               Joints (Joints'First + J - 1) :=
                 (W => Was.W, P => (if Dot (Side, Side) > 0.0 then Unit (Side) else E1), C => Was.C, Slide => Was.Slide);
            end;
         end loop;
      else
         declare
            Moving : Boolean := False;

            --  The directions searched: a Fibonacci lattice on the sphere, as
            --  dense as an icosahedron split four times (2562 points, about
            --  four degrees apart), of which the half on one side: a direction
            --  and its opposite are one axis, and the sign of the turn is
            --  measured with its size (Planar_About).
            Directions : constant Natural := 2562;
            Sphere     : array (1 .. Directions) of Vec3;

            --  A joint's sightings as lines of sight at one focal length, with
            --  the joint's reading change at their keyframe, grouped by keyframe.
            type Sight_Line is record
               Frame   : Positive := 1;
               Reading : Real := 0.0;
               H0, H   : Vec3 := [0.0, 0.0, 1.0];
            end record;
            type Sight_Line_Array is array (Positive range <>) of Sight_Line;
            type Sight_Line_Access is access Sight_Line_Array;
            procedure Free is new Ada.Unchecked_Deallocation (Sight_Line_Array, Sight_Line_Access);
            Lines : array (1 .. N) of Sight_Line_Access;
            --  Every keyframe of the joint alone, at one focal length.
            Every : array (1 .. N) of Sight_Line_Access;
            --  Per joint, its sightings' pixel offsets from the lens centre, gathered once: a focal length only
            --  scales them (Ray without distortion is (U - Cx) / F, (V - Cy) / F, 1). Prepare used to scan every
            --  sighting once per joint, allocate and fill the lines anew and call Ray for each, at every focal
            --  length the search tried: B0a_2 (x5, 10-13) tried about 190 in one fit and spent 224 s of the
            --  fit's 194 s stage in Ray and that scan (15.5 million calls, the profile of its replay).
            type Centred_Sighting is record
               Frame   : Positive := 1;
               Reading : Real := 0.0;
               X0, Y0  : Real := 0.0;   --  the reference pixel less the centre
               X1, Y1  : Real := 0.0;   --  the keyframe pixel less the centre
               Wide    : Boolean := False;
            end record;
            type Centred_Array is array (Positive range <>) of Centred_Sighting;
            type Centred_Access is access Centred_Array;
            procedure Free is new Ada.Unchecked_Deallocation (Centred_Array, Centred_Access);
            Centred   : array (1 .. N) of Centred_Access := [others => null];
            Gathered  : Boolean := False;

            --  The grid scores a joint on its widest turn each way: the keyframes
            --  that tell its axis best. Its refinement, and the score it gives the
            --  focal length, take every keyframe of the joint alone: turns of
            --  many sizes, whose sizes trade with the focal length in the small
            --  ones and not in the large (the image moves by the focal length
            --  times a turn's tangent, not its angle).
            Up_Frame, Down_Frame : array (1 .. N) of Natural := [others => 0];
            Grid_Counts : array (1 .. N) of Natural := [others => 0];

            --  How many sightings each keyframe has: one whose matches have not
            --  come back yet (the newest, widest keyframe of a boot's refit) has
            --  none, and is no widest one.
            Seen_In : array (1 .. Frames) of Natural := [others => 0];

            function Widest (Frame : Positive) return Boolean is
              (Own_Joint (Frame) > 0
               and then (Frame = Up_Frame (Own_Joint (Frame)) or else Frame = Down_Frame (Own_Joint (Frame))));

            --  Whether a keyframe's sightings are a homography of the reference's: a plane's points (the table under a
            --  wrist), or an eye that turns without travelling (a wrist on a short lever), whatever the scene. There
            --  the epipolar line of every sighting holds under a family of motions, not one: the turn about a direction
            --  and the focal length that the residuals below score are not told by it (the desktop regime's synthetic
            --  arm on a plane of points, 10-16: the search chose 69 px of 400; R6's kiwi wrist, 10-14: 27 to 24453 px),
            --  and the plane's homography tells them (Turn_On_Plane). Judged on the pixels alone, so the same at every
            --  focal length and direction the search tries: the least squares homography and fundamental matrix of the
            --  keyframe (each in coordinates centred on its points' mean and scaled to unit spread per axis, solved
            --  again with each sighting weighted to its distance in pixels and by Huber at the noise of the first),
            --  and the noise each leaves: a homography's transfer of the reference pixel, two axes, its median over
            --  the Rayleigh median; the fundamental matrix's distance of the keyframe pixel from its line, one axis,
            --  its median over the normal's upper quartile. Both are the sightings' noise where the keyframe is a
            --  homography (the fundamental matrix fits a homography's pairs as well as any other's), and the
            --  homography's is the larger by the parallax where it is not: homographic unless its noise is
            --  significantly the larger, the two medians' standard errors combined.
            Homographic : array (1 .. Frames) of Boolean := [others => False];

            procedure Judge_Homography (C : Centred_Array) is
               M       : constant Natural := C'Length;
               Entries : constant := 9;   --  a 3 by 3 matrix's
               type Pixel_Array is array (1 .. M) of Real;
               X0, Y0, X1, Y1 : Pixel_Array;
               S0, S1  : Real := 0.0;
               Mx0, My0, Mx1, My1 : Real := 0.0;
               Wh, Wf  : Pixel_Array := [others => 1.0];
               Eh, Ef  : Pixel_Array := [others => 0.0];   --  the residuals in pixels
               Hm, Fm  : Real_Matrix (1 .. 3, 1 .. 3);

               --  The unit vector the weighted rows' normal matrix has least.
               type Row is array (1 .. Entries) of Real;
               function Least (Rows : access function (I : Positive; Second : Boolean) return Row;
                               Weight : Pixel_Array; Two : Boolean) return Real_Matrix
               is
                  A       : Real_Matrix (1 .. Entries, 1 .. Entries) := [others => [others => 0.0]];
                  Values  : Real_Vector (1 .. Entries);
                  Vectors : Real_Matrix (1 .. Entries, 1 .. Entries);
                  Low     : Positive := 1;
                  Result  : Real_Matrix (1 .. 3, 1 .. 3);
               begin
                  for I in 1 .. M loop
                     for Second in Boolean loop
                        if Second <= Two then
                           declare
                              G : constant Row := Rows (I, Second);
                           begin
                              for P in 1 .. Entries loop
                                 for Q in P .. Entries loop
                                    A (P, Q) := A (P, Q) + Weight (I) * G (P) * G (Q);
                                 end loop;
                              end loop;
                           end;
                        end if;
                     end loop;
                  end loop;
                  for P in 2 .. Entries loop
                     for Q in 1 .. P - 1 loop
                        A (P, Q) := A (Q, P);
                     end loop;
                  end loop;
                  Driver.Numerics.Dense.Small_Eigensystem (A, Values, Vectors);
                  for P in 2 .. Entries loop
                     if Values (P) < Values (Low) then
                        Low := P;
                     end if;
                  end loop;
                  for P in 1 .. 3 loop
                     for Q in 1 .. 3 loop
                        Result (P, Q) := Vectors (3 * (P - 1) + Q, Low);
                     end loop;
                  end loop;
                  return Result;
               end Least;

               --  A homography: two rows a sighting, the keyframe pixel's two coordinates.
               function H_Row (I : Positive; Second : Boolean) return Row is
                 (if not Second then [-X0 (I), -Y0 (I), -1.0, 0.0, 0.0, 0.0, X1 (I) * X0 (I), X1 (I) * Y0 (I), X1 (I)]
                  else [0.0, 0.0, 0.0, -X0 (I), -Y0 (I), -1.0, Y1 (I) * X0 (I), Y1 (I) * Y0 (I), Y1 (I)]);
               --  A fundamental matrix: one row a sighting, X1' F X0 = 0.
               function F_Row (I : Positive; Second : Boolean) return Row is
                  pragma Unreferenced (Second);
               begin
                  return [X1 (I) * X0 (I), X1 (I) * Y0 (I), X1 (I), Y1 (I) * X0 (I), Y1 (I) * Y0 (I), Y1 (I), X0 (I), Y0 (I), 1.0];
               end F_Row;

               --  Each sighting's residual in pixels under Hm and Fm, and the weight that makes the next least squares
               --  of each that of those residuals.
               procedure Residuals is
               begin
                  for I in 1 .. M loop
                     declare
                        Px : constant Real := Hm (1, 1) * X0 (I) + Hm (1, 2) * Y0 (I) + Hm (1, 3);
                        Py : constant Real := Hm (2, 1) * X0 (I) + Hm (2, 2) * Y0 (I) + Hm (2, 3);
                        Pz : constant Real := Hm (3, 1) * X0 (I) + Hm (3, 2) * Y0 (I) + Hm (3, 3);
                        Lx : constant Real := Fm (1, 1) * X0 (I) + Fm (1, 2) * Y0 (I) + Fm (1, 3);
                        Ly : constant Real := Fm (2, 1) * X0 (I) + Fm (2, 2) * Y0 (I) + Fm (2, 3);
                        Lz : constant Real := Fm (3, 1) * X0 (I) + Fm (3, 2) * Y0 (I) + Fm (3, 3);
                        Ll : constant Real := Sqrt (Lx * Lx + Ly * Ly);
                     begin
                        if Pz /= 0.0 then
                           Eh (I) := S1 * Sqrt ((Px / Pz - X1 (I)) ** 2 + (Py / Pz - Y1 (I)) ** 2);
                           Wh (I) := 1.0 / Pz ** 2;
                        else
                           Eh (I) := Real'Last;
                           Wh (I) := 0.0;
                        end if;
                        if Ll > 0.0 then
                           Ef (I) := S1 * abs (X1 (I) * Lx + Y1 (I) * Ly + Lz) / Ll;
                           Wf (I) := 1.0 / Ll ** 2;
                        else
                           Ef (I) := Real'Last;
                           Wf (I) := 0.0;
                        end if;
                     end;
                  end loop;
               end Residuals;

               --  The fundamental matrix of rank two: its least singular direction taken out.
               procedure Rank_Two is
                  Ftf     : Real_Matrix (1 .. 3, 1 .. 3) := Transpose (Fm) * Fm;
                  Values  : Real_Vector (1 .. 3);
                  Vectors : Real_Matrix (1 .. 3, 1 .. 3);
                  Low     : Positive := 1;
               begin
                  for P in 2 .. 3 loop
                     for Q in 1 .. P - 1 loop
                        Ftf (P, Q) := Ftf (Q, P);
                     end loop;
                  end loop;
                  Driver.Numerics.Dense.Small_Eigensystem (Ftf, Values, Vectors);
                  for P in 2 .. 3 loop
                     if Values (P) < Values (Low) then
                        Low := P;
                     end if;
                  end loop;
                  declare
                     V : constant Real_Vector (1 .. 3) := [for P in 1 .. 3 => Vectors (P, Low)];
                     Fv : constant Real_Vector (1 .. 3) := Fm * V;
                  begin
                     for P in 1 .. 3 loop
                        for Q in 1 .. 3 loop
                           Fm (P, Q) := Fm (P, Q) - Fv (P) * V (Q);
                        end loop;
                     end loop;
                  end;
               end Rank_Two;
            begin
               if M < Entries then
                  return;
               end if;
               for I in 1 .. M loop
                  Mx0 := Mx0 + C (C'First + I - 1).X0;
                  My0 := My0 + C (C'First + I - 1).Y0;
                  Mx1 := Mx1 + C (C'First + I - 1).X1;
                  My1 := My1 + C (C'First + I - 1).Y1;
               end loop;
               Mx0 := Mx0 / Real (M);
               My0 := My0 / Real (M);
               Mx1 := Mx1 / Real (M);
               My1 := My1 / Real (M);
               for I in 1 .. M loop
                  S0 := S0 + (C (C'First + I - 1).X0 - Mx0) ** 2 + (C (C'First + I - 1).Y0 - My0) ** 2;
                  S1 := S1 + (C (C'First + I - 1).X1 - Mx1) ** 2 + (C (C'First + I - 1).Y1 - My1) ** 2;
               end loop;
               S0 := Sqrt (S0 / Real (2 * M));
               S1 := Sqrt (S1 / Real (2 * M));
               if S0 <= 0.0 or else S1 <= 0.0 then
                  return;
               end if;
               for I in 1 .. M loop
                  X0 (I) := (C (C'First + I - 1).X0 - Mx0) / S0;
                  Y0 (I) := (C (C'First + I - 1).Y0 - My0) / S0;
                  X1 (I) := (C (C'First + I - 1).X1 - Mx1) / S1;
                  Y1 (I) := (C (C'First + I - 1).Y1 - My1) / S1;
               end loop;
               for Pass in 1 .. 2 loop
                  Hm := Least (H_Row'Access, Wh, Two => True);
                  Fm := Least (F_Row'Access, Wf, Two => False);
                  Rank_Two;
                  Residuals;
                  if Pass = 1 then
                     declare
                        Sh : constant Real := Driver.Stats.Median (Real_Array (Eh)) / Rayleigh_Median;
                        Sf : constant Real := Driver.Stats.Median (Real_Array (Ef)) / Upper_Quartile;
                     begin
                        for I in 1 .. M loop
                           if Sh > 0.0 and then Eh (I) < Real'Last then
                              Wh (I) := Wh (I) * Huber (Eh (I) / Sh);
                           end if;
                           if Sf > 0.0 and then Ef (I) < Real'Last then
                              Wf (I) := Wf (I) * Huber (Ef (I) / Sf);
                           end if;
                        end loop;
                     end;
                  end if;
               end loop;
               declare
                  Mh : constant Real := Driver.Stats.Median (Real_Array (Eh));
                  Mf : constant Real := Driver.Stats.Median (Real_Array (Ef));
                  --  A median's standard error over the median, of the Rayleigh's (1 / (2 ln 2 sqrt M)) and of the
                  --  normal's absolute value (Median_Error).
                  Rh : constant Real := 1.0 / (Rayleigh_Median ** 2 * Sqrt (Real (M)));
                  Rf : constant Real := (if Mf > 0.0 then Median_Error (Mf, M) / Mf else 0.0);
               begin
                  Homographic (C (C'First).Frame) :=
                    Mh = 0.0
                    or else (Mf > 0.0 and then Mh < Real'Last
                             and then Ln ((Mh / Rayleigh_Median) / (Mf / Upper_Quartile))
                                      <= Driver.Conventions.Z * Sqrt (Rh ** 2 + Rf ** 2));
               end;
            end Judge_Homography;

            procedure Prepare (F : Real) is
            begin
               if not Gathered then
                  for J in 1 .. N loop
                     Free (Lines (J));
                     Free (Every (J));
                     Lines (J) := new Sight_Line_Array (1 .. Grid_Counts (J));
                     Every (J) := new Sight_Line_Array (1 .. Counts (J));
                     Centred (J) := new Centred_Array (1 .. Counts (J));
                  end loop;
                  declare
                     E : array (1 .. N) of Natural := [others => 0];
                  begin
                     for X of Pairs.all loop
                        if Own_Joint (X.Frame) in 1 .. N then
                           declare
                              J : constant Positive := Own_Joint (X.Frame);
                           begin
                              E (J) := E (J) + 1;
                              Centred (J) (E (J)) :=
                                (Frame => X.Frame, Reading => Own_Change (X.Frame, J),
                                 X0 => X.U0 - Lens0.Cx, Y0 => X.V0 - Lens0.Cy,
                                 X1 => X.U - Lens0.Cx, Y1 => X.V - Lens0.Cy, Wide => Widest (X.Frame));
                           end;
                        end if;
                     end loop;
                  end;
                  for J in 1 .. N loop
                     declare
                        From : Positive := Centred (J)'First;
                     begin
                        while From <= Centred (J)'Last loop
                           declare
                              To : Positive := From;
                           begin
                              while To < Centred (J)'Last and then Centred (J) (To + 1).Frame = Centred (J) (From).Frame loop
                                 To := To + 1;
                              end loop;
                              Judge_Homography (Centred (J) (From .. To));
                              From := To + 1;
                           end;
                        end loop;
                     end;
                     declare
                        Keyframes, Homographies : Natural := 0;
                        Last : Natural := 0;
                     begin
                        for C of Centred (J).all loop
                           if C.Frame /= Last then
                              Last := C.Frame;
                              Keyframes := Keyframes + 1;
                              if Homographic (C.Frame) then
                                 Homographies := Homographies + 1;
                              end if;
                           end if;
                        end loop;
                        Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " joint" & J'Image & ":"
                                         & Homographies'Image & " of" & Keyframes'Image
                                         & " keyframes a homography of the reference's (a plane, or no travel)");
                     end;
                  end loop;
                  Gathered := True;
               end if;
               for J in 1 .. N loop
                  declare
                     K : Natural := 0;
                  begin
                     for E in Centred (J)'Range loop
                        declare
                           C : Centred_Sighting renames Centred (J) (E);
                        begin
                           Every (J) (E) := (Frame => C.Frame, Reading => C.Reading,
                                             H0 => [C.X0 / F, C.Y0 / F, 1.0], H => [C.X1 / F, C.Y1 / F, 1.0]);
                           if C.Wide then
                              K := K + 1;
                              Lines (J) (K) := Every (J) (E);
                           end if;
                        end;
                     end loop;
                  end;
               end loop;
            end Prepare;

            type Best_Record is record
               Score : Real := Real'Last;
               W     : Vec3 := [0.0, 0.0, 1.0];
               Phi   : Real := 0.0;
               Scale : Real := 0.0;   --  the turn per reading unit
            end record;
            type Best_Array is array (1 .. N) of Best_Record;

            --  The side of the eye the axis W lies on, the joint turning by Scale
            --  per reading unit, solved in closed form (the epipolar constraint is
            --  linear in the axis point), the constraints weighted to Sampson
            --  residuals and by Huber at the noise of a first solution; the median
            --  residual left.
            procedure Best_Phi
              (J : Positive; F : Real; W : Vec3; Scale : Real; Phi : out Real; Median_Px : out Real;
               Ga, Gb, Wt, Rs : out Real_Array)
            is
               L      : Sight_Line_Array renames Lines (J).all;
               K      : constant Natural := L'Length;
               E1, E2 : Vec3;
               C      : Real := 1.0;
               Sn     : Real := 0.0;
               R      : Mat3 := Identity3;
               Last   : Natural := 0;
            begin
               Perp (W, E1, E2);
               for I in 1 .. K loop
                  if L (I).Frame /= Last then
                     Last := L (I).Frame;
                     R := Rot (W, Scale * L (I).Reading);
                  end if;
                  declare
                     Y  : constant Vec3 := Transpose (R) * L (I).H0;
                     Yh : constant Vec3 := Cross (Y, L (I).H);
                     Gv : constant Vec3 := Yh - R * Yh;
                  begin
                     Ga (I) := Dot (Gv, E1);
                     Gb (I) := Dot (Gv, E2);
                     Wt (I) := (if Ga (I) ** 2 + Gb (I) ** 2 > 0.0 then 1.0 / (Ga (I) ** 2 + Gb (I) ** 2) else 0.0);
                  end;
               end loop;
               for Pass in 1 .. 2 loop
                  declare
                     A11, A12, A22 : Real := 0.0;
                  begin
                     for I in 1 .. K loop
                        A11 := A11 + Wt (I) * Ga (I) * Ga (I);
                        A12 := A12 + Wt (I) * Ga (I) * Gb (I);
                        A22 := A22 + Wt (I) * Gb (I) * Gb (I);
                     end loop;
                     declare
                        Spread : constant Real := Sqrt (Real'Max (0.0, ((A11 - A22) / 2.0) ** 2 + A12 ** 2));
                        Least  : constant Real := (A11 + A22) / 2.0 - Spread;
                        Vx, Vy : Real;
                     begin
                        if A12 /= 0.0 then
                           Vx := A12;
                           Vy := Least - A11;
                        elsif A11 <= A22 then
                           Vx := 1.0;
                           Vy := 0.0;
                        else
                           Vx := 0.0;
                           Vy := 1.0;
                        end if;
                        C := Vx / Sqrt (Vx ** 2 + Vy ** 2);
                        Sn := Vy / Sqrt (Vx ** 2 + Vy ** 2);
                     end;
                  end;
                  --  The axis point P: (C, Sn) is an eigenvector of a 2 by 2 system, its sign arbitrary. Which side
                  --  of the axis the eye is on is not told here: every residual is even in the translation P - R P
                  --  (Reprojected: the Sampson distance changes sign with it, and no penalty is charged behind the
                  --  vanishing point), so the two signs scored alike to the last bit and the first was always kept,
                  --  at twice the time (10-15). The points in front tell the side (after stage 4).
                  declare
                     P : constant Vec3 := C * E1 + Sn * E2;
                     --  The keyframe's motion, once a keyframe: its sightings are together.
                     Rr : Mat3;
                     Tt : Vec3;
                  begin
                     Last := 0;
                     for I in 1 .. K loop
                        if L (I).Frame /= Last then
                           Last := L (I).Frame;
                           R := Rot (W, Scale * L (I).Reading);
                           Relative (Identity, (Rotation => R, Translation => P - R * P), Rr, Tt);
                        end if;
                        Rs (I) := Reprojected (Rr, Tt, L (I).H0, L (I).H, F);
                     end loop;
                  end;
                  if Pass = 1 then
                     declare
                        Sigma : constant Real := Noise_Of (Rs);
                     begin
                        for I in 1 .. K loop
                           declare
                              Algebraic : constant Real := C * Ga (I) + Sn * Gb (I);
                           begin
                              if Algebraic /= 0.0 and then Rs (I) /= 0.0 then
                                 Wt (I) := (Rs (I) / Algebraic) ** 2;
                              end if;
                              if Sigma > 0.0 then
                                 Wt (I) := Wt (I) * Huber (Rs (I) / Sigma);
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
               Phi := Arctan (Sn, C);
               for I in 1 .. K loop
                  Rs (I) := abs Rs (I);
               end loop;
               Median_Px := Driver.Stats.Median (Rs);
            end Best_Phi;

            --  How well joint J's widest keyframes are each a turn about the
            --  direction W and a shift across it, at focal length F, and how far
            --  the joint turns per reading unit about W: no reading is taken to be
            --  in any unit, and the direction is judged before any turn is tied to
            --  the readings. About W, in a frame whose third axis is W, the
            --  essential matrix of such a motion has four entries, (A, B) down its
            --  last column and (E, F) along its last row, (E, F) being (A, B)
            --  turned by the keyframe's turn and reversed, and the epipolar
            --  constraint is linear in the four. They are the least squares of the
            --  keyframe's sightings, weighted to Sampson residuals and by Huber at
            --  the noise of a first solution, as Best_Phi weights its own; the
            --  turn is the angle from (A, B) to -(E, F). Score is the median
            --  Sampson residual of the sightings under their keyframe's motion
            --  (Real'Last when no keyframe's is told), Scale the least squares of
            --  the turns over the reading changes: 0 when no keyframe's turn is
            --  told (fewer sightings than the three a motion about W has, or none
            --  of the four entries' pairs is nonzero).
            --  Beat: the score to beat. A direction whose median is surely not below it (no more than half of its
            --  residuals below it) is scored Real'Last, its median not taken: the scan keeps a direction only when it
            --  scores below the best so far, and the medians of every direction were a fifth of stage 1's time (the
            --  order exam's arms, 10-20).
            procedure Planar_About
              (J : Positive; F : Real; W : Vec3; Wt, Rs, Fits : in out Real_Array; Beat : Real; Scale, Score : out Real)
            is
               L      : Sight_Line_Array renames Lines (J).all;
               K      : constant Natural := L'Length;
               E1, E2 : Vec3;
               Num, Den : Real := 0.0;
               Filled   : Natural := 0;   --  residuals in Fits
               --  Per way (the turn up, the turn down): whether its frame's motion was told, the frame, its rotation
               --  and its translation before the sign is chosen.
               Told_Of  : array (1 .. 2) of Boolean := [others => False];
               Frame_Of : array (1 .. 2) of Natural := [others => 0];
               Rr_Of    : array (1 .. 2) of Mat3;
               T0_Of    : array (1 .. 2) of Vec3;
               --  A homography's keyframe (Homographic): its plane, as Turn_On_Plane gives it, and that it has one.
               Plane_Of  : array (1 .. 2) of Vec3 := [others => Zero3];
               Plane_Way : array (1 .. 2) of Boolean := [others => False];

               --  The sighting's row of the constraint: its lines of sight in the frame of W.
               type Four is array (1 .. 4) of Real;
               function Row (I : Positive) return Four is
                  H0 : constant Vec3 := [Dot (E1, L (I).H0), Dot (E2, L (I).H0), Dot (W, L (I).H0)];
                  H  : constant Vec3 := [Dot (E1, L (I).H), Dot (E2, L (I).H), Dot (W, L (I).H)];
               begin
                  return [H (1) * H0 (3), H (2) * H0 (3), H (3) * H0 (1), H (3) * H0 (2)];
               end Row;

               function Turn_Of (V : Real_Vector) return Real is
                 (Arctan (V (2) * V (3) - V (1) * V (4), -(V (1) * V (3) + V (2) * V (4))));

               --  The translation's sign: V is an eigenvector, its sign arbitrary; the turn is even in V and the
               --  translation odd. No sign is chosen: every residual here is even in the translation (Best_Phi says
               --  why), and the sums that chose one, a bit a frame and then a bit a joint (10-14), came out equal to
               --  the last bit and never chose (10-15).
            begin
               Perp (W, E1, E2);
               for Way in 1 .. 2 loop
                  if (if Way = 1 then Up_Frame (J) else Down_Frame (J)) > 0 then
                     declare
                        Frame : constant Positive := (if Way = 1 then Up_Frame (J) else Down_Frame (J));
                        V     : Real_Vector (1 .. 4) := [others => 0.0];
                        Count : Natural := 0;
                        Told  : Boolean := False;
                        Change_Here : Real := 0.0;
                     begin
                        for I in 1 .. K loop
                           if L (I).Frame = Frame then
                              Wt (I) := 1.0;
                              Count := Count + 1;
                              Change_Here := L (I).Reading;
                           end if;
                        end loop;
                        if Count >= 3 and then not Homographic (Frame) then
                           for Pass in 1 .. 2 loop
                              declare
                                 A       : Real_Matrix (1 .. 4, 1 .. 4) := [others => [others => 0.0]];
                                 Values  : Real_Vector (1 .. 4);
                                 Vectors : Real_Matrix (1 .. 4, 1 .. 4);
                                 Least   : Positive := 1;
                              begin
                                 for I in 1 .. K loop
                                    if L (I).Frame = Frame then
                                       declare
                                          G : constant Four := Row (I);
                                       begin
                                          for P in 1 .. 4 loop
                                             for Q in P .. 4 loop
                                                A (P, Q) := A (P, Q) + Wt (I) * G (P) * G (Q);
                                             end loop;
                                          end loop;
                                       end;
                                    end if;
                                 end loop;
                                 --  Symmetric to the last bit, as Eigensystem asks: its lower half the upper's.
                                 for P in 2 .. 4 loop
                                    for Q in 1 .. P - 1 loop
                                       A (P, Q) := A (Q, P);
                                    end loop;
                                 end loop;
                                 Driver.Numerics.Dense.Small_Eigensystem (A, Values, Vectors);
                                 for P in 2 .. 4 loop
                                    if Values (P) < Values (Least) then
                                       Least := P;
                                    end if;
                                 end loop;
                                 V := [for P in 1 .. 4 => Vectors (P, Least)];
                                 Told := V (1) ** 2 + V (2) ** 2 > 0.0 and then V (3) ** 2 + V (4) ** 2 > 0.0;
                                 exit when not Told or else Pass = 2;
                                 --  The weights for the second solution, from the motion of the first.
                                 declare
                                    Turn : constant Real := Turn_Of (V);
                                    Rr   : constant Mat3 := Rot (W, -Turn);
                                    Tt   : constant Vec3 := Plus (Scaled (-V (2), E1), Scaled (V (1), E2));
                                    M    : Natural := 0;
                                    Du, Dv : Real_Array (1 .. K);
                                 begin
                                    for I in 1 .. K loop
                                       if L (I).Frame = Frame then
                                          M := M + 1;
                                          Reprojected (Rr, Tt, L (I).H0, L (I).H, F, Du (M), Dv (M));
                                       end if;
                                    end loop;
                                    Share_Out (Du (1 .. M));
                                    for Q in 1 .. M loop
                                       Rs (Q) := Sqrt (Du (Q) ** 2 + Dv (Q) ** 2);
                                    end loop;
                                    --  Noise_Of the residuals (each a length, none negative), Fits the scratch for its median
                                    --  until the residuals fill it below.
                                    Fits (1 .. M) := Rs (1 .. M);
                                    declare
                                       Sigma : constant Real := Driver.Stats.Median_In_Place (Fits (1 .. M)) / Upper_Quartile;
                                    begin
                                       M := 0;
                                       for I in 1 .. K loop
                                          if L (I).Frame = Frame then
                                             M := M + 1;
                                             declare
                                                G         : constant Four := Row (I);
                                                Algebraic : constant Real := G (1) * V (1) + G (2) * V (2) + G (3) * V (3)
                                                                             + G (4) * V (4);
                                             begin
                                                if Algebraic /= 0.0 and then Rs (M) /= 0.0 then
                                                   Wt (I) := (Rs (M) / Algebraic) ** 2;
                                                end if;
                                                if Sigma > 0.0 then
                                                   Wt (I) := Wt (I) * Huber (Rs (M) / Sigma);
                                                end if;
                                             end;
                                          end if;
                                       end loop;
                                    end;
                                 end;
                              end;
                           end loop;
                        end if;
                        if Count >= 3 and then Homographic (Frame) then
                           --  The plane's homography (Turn_On_Plane), weighted for its second solution as the epipolar
                           --  one is: each sighting to its transfer in pixels over its algebraic residual, and by Huber
                           --  at the noise of the first (the Rayleigh's: the transfer has two axes).
                           declare
                              Before, After : Vec3_Array (1 .. Count);
                              Wp     : Real_Array (1 .. Count) := [others => 1.0];
                              Turn_H : Real := 0.0;
                              Sh, Nm : Vec3 := Zero3;
                              Found  : Boolean := False;
                              M      : Natural := 0;
                           begin
                              for I in 1 .. K loop
                                 if L (I).Frame = Frame then
                                    M := M + 1;
                                    Before (M) := L (I).H0;
                                    After (M) := L (I).H;
                                 end if;
                              end loop;
                              for Pass in 1 .. 2 loop
                                 Turn_On_Plane (Before, After, Wp, W, Turn_H, Sh, Nm, Found);
                                 exit when not Found or else Pass = 2;
                                 declare
                                    Rr  : constant Mat3 := Rot (W, -Turn_H);
                                    Ep  : Real_Array (1 .. M);
                                    Alg : Real_Array (1 .. M);
                                 begin
                                    for Q in 1 .. M loop
                                       declare
                                          Y  : constant Vec3 := Plus (Times (Rr, Before (Q)), Scaled (Dot (Nm, Before (Q)), Sh));
                                          Hb : constant Vec3 := After (Q);
                                          A1 : constant Real := Dot (W, Hb) * Dot (E1, Y) - Dot (E1, Hb) * Dot (W, Y);
                                          A2 : constant Real := Dot (W, Hb) * Dot (E2, Y) - Dot (E2, Hb) * Dot (W, Y);
                                       begin
                                          Alg (Q) := Sqrt (A1 ** 2 + A2 ** 2);
                                          Ep (Q) := (if Y (3) > 0.0 and then Hb (3) > 0.0
                                                     then F * Sqrt ((Y (1) / Y (3) - Hb (1) / Hb (3)) ** 2
                                                                    + (Y (2) / Y (3) - Hb (2) / Hb (3)) ** 2)
                                                     else Real'Last);
                                       end;
                                    end loop;
                                    declare
                                       Sigma : constant Real := Driver.Stats.Median (Ep) / Rayleigh_Median;
                                    begin
                                       for Q in 1 .. M loop
                                          if Ep (Q) = Real'Last then
                                             Wp (Q) := 0.0;
                                          else
                                             if Alg (Q) > 0.0 and then Ep (Q) > 0.0 then
                                                Wp (Q) := (Ep (Q) / Alg (Q)) ** 2;
                                             end if;
                                             if Sigma > 0.0 then
                                                Wp (Q) := Wp (Q) * Huber (Ep (Q) / Sigma);
                                             end if;
                                          end if;
                                       end loop;
                                    end;
                                 end;
                              end loop;
                              if Found then
                                 Num := Num + Turn_H * Change_Here;
                                 Den := Den + Change_Here ** 2;
                                 Told_Of (Way) := True;
                                 Plane_Way (Way) := True;
                                 Frame_Of (Way) := Frame;
                                 Rr_Of (Way) := Rot (W, -Turn_H);
                                 T0_Of (Way) := Sh;
                                 Plane_Of (Way) := Nm;
                              end if;
                           end;
                        elsif Told then
                           Num := Num + Turn_Of (V) * Change_Here;
                           Den := Den + Change_Here ** 2;
                           Told_Of (Way) := True;
                           Frame_Of (Way) := Frame;
                           Rr_Of (Way) := Rot (W, -Turn_Of (V));
                           T0_Of (Way) := Plus (Scaled (-V (2), E1), Scaled (V (1), E2));
                        end if;
                     end;
                  end if;
               end loop;
               --  Each way's residuals under its translation, shared out, are its frame's fits.
               for Way in 1 .. 2 loop
                  if Told_Of (Way) then
                     declare
                        Du, Dv : Real_Array (1 .. K);
                        M      : Natural := 0;
                     begin
                        for I in 1 .. K loop
                           if L (I).Frame = Frame_Of (Way) then
                              M := M + 1;
                              if Plane_Way (Way) then
                                 --  The homography's transfer, two axes; behind either eye as far off as the lines of
                                 --  sight are long, as Reprojected has it.
                                 declare
                                    Y : constant Vec3 :=
                                      Plus (Times (Rr_Of (Way), L (I).H0), Scaled (Dot (Plane_Of (Way), L (I).H0), T0_Of (Way)));
                                 begin
                                    if Y (3) > 0.0 and then L (I).H (3) > 0.0 then
                                       Du (M) := F * (Y (1) / Y (3) - L (I).H (1) / L (I).H (3));
                                       Dv (M) := F * (Y (2) / Y (3) - L (I).H (2) / L (I).H (3));
                                    else
                                       Du (M) := F * Sqrt (Dot (Y, Y) + Dot (L (I).H, L (I).H));
                                       Dv (M) := 0.0;
                                    end if;
                                 end;
                              else
                                 Reprojected (Rr_Of (Way), T0_Of (Way), L (I).H0, L (I).H, F, Du (M), Dv (M));
                              end if;
                           end if;
                        end loop;
                        Share_Out (Du (1 .. M));
                        if Plane_Way (Way) then
                           Share_Out (Dv (1 .. M));
                        end if;
                        --  A transfer's two axes on the scale of the epipolar distance's one: its median over the
                        --  Rayleigh's, times the normal's upper quartile, where both are the sightings' noise.
                        for Q in 1 .. M loop
                           Filled := Filled + 1;
                           Fits (Filled) := (if Plane_Way (Way) then Upper_Quartile / Rayleigh_Median else 1.0)
                                            * Sqrt (Du (Q) ** 2 + Dv (Q) ** 2);
                        end loop;
                     end;
                  end if;
               end loop;
               Scale := (if Den > 0.0 then Num / Den else 0.0);
               declare
                  Below : Natural := 0;
               begin
                  for Q in 1 .. Filled loop
                     if Fits (Q) < Beat then
                        Below := Below + 1;
                     end if;
                  end loop;
                  Score := (if Filled = 0 or else 2 * Below < Filled then Real'Last
                            else Driver.Stats.Median_In_Place (Fits (1 .. Filled)));
               end;
            end Planar_About;

            --  The best direction of the grid refined, its tilt, its side and the size of its turn per
            --  reading unit, by robust least squares on every sighting of joint J alone at focal length F,
            --  whose median residual scores it: the grid is four degrees apart, and with the turn measured
            --  about each direction its best score jumps from one focal length to the next as much as it
            --  changes with them, which a search of the focal length cannot follow (A93's left arm, its
            --  readings drawn: the search ended at a focal length of 118 pixels of 400).
            --
            --  A procedure of its own, not a block of the scanner task: the fit's Evaluate reads the
            --  sightings' count and the start of the search from the frame around it, and built with
            --  AddressSanitizer a task body's frame had that count poisoned by the time Evaluate read it
            --  (robot.body.plan, .file, .method: "unknown-crash" at the count's four bytes), which the
            --  same construct in a plain procedure never showed (the three other instances of Robust_Fit).
            procedure Refine (J : Positive; F : Real; Result : in out Best_Record) is
               L      : Sight_Line_Array renames Every (J).all;
               M      : constant Positive := L'Length;
               Rm     : Real_Access := new Real_Array (1 .. 2 * M);   --  two residuals a sighting (Reprojected)
               W0     : constant Vec3 := Result.W;
               Sign   : constant Real := (if Result.Scale < 0.0 then -1.0 else 1.0);
               E1, E2 : Vec3;

               function W_Of (X : Real_Array) return Vec3 is (Unit (W0 + X (1) * E1 + X (2) * E2));

               procedure Evaluate (X : Real_Array; R : out Real_Array) is
                  W      : constant Vec3 := W_Of (X);
                  C      : constant Real := Sign * Exp (X (4));
                  F1, F2 : Vec3;
                  P      : Vec3;
                  Rj     : Mat3 := Identity3;
                  Last   : Natural := 0;
                  Rr     : Mat3;
                  Tt     : Vec3;
               begin
                  Perp (W, F1, F2);
                  P := Cos (X (3)) * F1 + Sin (X (3)) * F2;
                  for I in L'Range loop
                     if L (I).Frame /= Last then
                        Last := L (I).Frame;
                        Rj := Rot (W, C * L (I).Reading);
                     end if;
                     Relative (Identity, (Rotation => Rj, Translation => P - Rj * P), Rr, Tt);
                     Reprojected (Rr, Tt, L (I).H0, L (I).H, F,
                                  R (R'First + 2 * (I - L'First)), R (R'First + 2 * (I - L'First) + 1));
                  end loop;
                  --  Less what each keyframe's sightings share (Share_Out); the sightings come grouped by keyframe.
                  declare
                     From : Positive := L'First;
                  begin
                     while From <= L'Last loop
                        declare
                           To : Positive := From;
                        begin
                           while To < L'Last and then L (To + 1).Frame = L (From).Frame loop
                              To := To + 1;
                           end loop;
                           declare
                              Du, Dv : Real_Array (1 .. To - From + 1);
                           begin
                              for I in From .. To loop
                                 Du (I - From + 1) := R (R'First + 2 * (I - L'First));
                                 Dv (I - From + 1) := R (R'First + 2 * (I - L'First) + 1);
                              end loop;
                              Share_Out (Du);
                              for I in From .. To loop
                                 R (R'First + 2 * (I - L'First)) := Du (I - From + 1);
                                 R (R'First + 2 * (I - L'First) + 1) := Dv (I - From + 1);
                              end loop;
                           end;
                           From := To + 1;
                        end;
                     end loop;
                  end;
               end Evaluate;

               procedure Solve is new Robust_Fit (4, 2 * M, Driver.Conventions.Initial_Damping, Evaluate);
               X : Real_Array (1 .. 4) := [0.0, 0.0, Result.Phi, Ln (abs Result.Scale)];
            begin
               Perp (W0, E1, E2);
               Evaluate (X, Rm.all);
               Solve (X, Noise_Of (Rm.all));
               Evaluate (X, Rm.all);
               declare
                  Norms : Real_Array (1 .. M);
               begin
                  for I in 1 .. M loop
                     Norms (I) := Sqrt (Rm (2 * I - 1) ** 2 + Rm (2 * I) ** 2);
                  end loop;
                  Result := (Score => Driver.Stats.Median (Norms), W => W_Of (X), Phi => X (3),
                             Scale => Sign * Exp (X (4)));
               end;
               Free (Rm);
            end Refine;

            --  Joint J as a slide at focal length F: the eye moved along one direction without turning, by its reading
            --  change times a length of either sign. The direction is the least eigenvector of the sightings' rows
            --  under a translation alone (the two lines of sight and the direction in one plane, (H0 x H) . D = 0),
            --  refined by robust least squares on the residuals Refine scores a turn by (Reprojected, less what each
            --  keyframe's sightings share), with either sign (a half-line's residual has one), the lesser median taken.
            procedure Slide_Of (J : Positive; F : Real; D_Out : out Vec3; Score : out Real) is
               L      : Sight_Line_Array renames Every (J).all;
               M      : constant Positive := L'Length;
               Rm     : Real_Access := new Real_Array (1 .. 2 * M);
               D0     : Vec3 := [0.0, 0.0, 1.0];
               E1, E2 : Vec3;
               Sign   : Real := 1.0;

               function D_Of (X : Real_Array) return Vec3 is (Unit (D0 + X (1) * E1 + X (2) * E2));

               procedure Evaluate (X : Real_Array; R : out Real_Array) is
                  D  : constant Vec3 := D_Of (X);
                  Rr : Mat3;
                  Tt : Vec3;
               begin
                  for I in L'Range loop
                     Relative (Identity, (Rotation => Identity3, Translation => (Sign * L (I).Reading) * D), Rr, Tt);
                     Reprojected (Rr, Tt, L (I).H0, L (I).H, F,
                                  R (R'First + 2 * (I - L'First)), R (R'First + 2 * (I - L'First) + 1));
                  end loop;
                  declare
                     From : Positive := L'First;
                  begin
                     while From <= L'Last loop
                        declare
                           To : Positive := From;
                        begin
                           while To < L'Last and then L (To + 1).Frame = L (From).Frame loop
                              To := To + 1;
                           end loop;
                           declare
                              Du, Dv : Real_Array (1 .. To - From + 1);
                           begin
                              for I in From .. To loop
                                 Du (I - From + 1) := R (R'First + 2 * (I - L'First));
                                 Dv (I - From + 1) := R (R'First + 2 * (I - L'First) + 1);
                              end loop;
                              Share_Out (Du);
                              for I in From .. To loop
                                 R (R'First + 2 * (I - L'First)) := Du (I - From + 1);
                                 R (R'First + 2 * (I - L'First) + 1) := Dv (I - From + 1);
                              end loop;
                           end;
                           From := To + 1;
                        end;
                     end loop;
                  end;
               end Evaluate;

               procedure Solve is new Robust_Fit (2, 2 * M, Driver.Conventions.Initial_Damping, Evaluate);
            begin
               declare
                  A       : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
                  Values  : Real_Vector (1 .. 3);
                  Vectors : Real_Matrix (1 .. 3, 1 .. 3);
                  Low     : Positive := 1;
               begin
                  for I in L'Range loop
                     declare
                        G : constant Vec3 := Cross (L (I).H0, L (I).H);
                     begin
                        for P in 1 .. 3 loop
                           for Q in 1 .. 3 loop
                              A (P, Q) := A (P, Q) + G (P) * G (Q);
                           end loop;
                        end loop;
                     end;
                  end loop;
                  Driver.Numerics.Dense.Small_Eigensystem (A, Values, Vectors);
                  for P in 2 .. 3 loop
                     if Values (P) < Values (Low) then
                        Low := P;
                     end if;
                  end loop;
                  D0 := Unit ([Vectors (1, Low), Vectors (2, Low), Vectors (3, Low)]);
               end;
               Perp (D0, E1, E2);
               D_Out := D0;
               Score := Real'Last;
               for Way in 1 .. 2 loop
                  Sign := (if Way = 1 then 1.0 else -1.0);
                  declare
                     X     : Real_Array (1 .. 2) := [0.0, 0.0];
                     Norms : Real_Array (1 .. M);
                  begin
                     Evaluate (X, Rm.all);
                     Solve (X, Noise_Of (Rm.all));
                     Evaluate (X, Rm.all);
                     for I in 1 .. M loop
                        Norms (I) := Sqrt (Rm (2 * I - 1) ** 2 + Rm (2 * I) ** 2);
                     end loop;
                     declare
                        Md : constant Real := Driver.Stats.Median (Norms);
                     begin
                        if Md < Score then
                           Score := Md;
                           D_Out := Sign * D_Of (X);
                        end if;
                     end;
                  end;
               end loop;
               Free (Rm);
            end Slide_Of;

            --  Every joint's best direction at focal length F, the joints scanned
            --  at once, one task each: their sightings are their own.
            Searches        : Natural := 0;          --  focal lengths tried, for the log
            F_Least, F_Most : Real := 0.0;           --  the least and the most of them, px
            Trace           : Ada.Strings.Unbounded.Unbounded_String;   --  every focal length tried and its score
            Tried_Ln, Tried_Total, Tried_Error : Real_Lists.Vector;   --  every focal length tried (its logarithm), its score and the score's error

            procedure Search (F : Real; Best : out Best_Array) is
            begin
               Searches := Searches + 1;
               F_Least := (if Searches = 1 then F else Real'Min (F_Least, F));
               F_Most := Real'Max (F_Most, F);
               Prepare (F);
               declare
                  task type Scanner is
                     entry Start (Joint : Positive);
                  end Scanner;

                  task body Scanner is
                     J      : Positive := 1;
                     Result : Best_Record;
                  begin
                     accept Start (Joint : Positive) do
                        J := Joint;
                     end Start;
                     declare
                        K  : constant Natural := Lines (J)'Length;
                        Ga : Real_Access := new Real_Array (1 .. K);
                        Gb : Real_Access := new Real_Array (1 .. K);
                        Wt : Real_Access := new Real_Array (1 .. K);
                        Rs : Real_Access := new Real_Array (1 .. K);
                        Fs : Real_Access := new Real_Array (1 .. K);
                     begin
                        --  The axis: the direction about which the widest keyframes are each most nearly a turn and a
                        --  shift across it, their turns free; then its scale, and its side at that scale.
                        for D of Sphere (1 .. Directions / 2) loop
                           declare
                              Sc, Sd : Real;
                           begin
                              Planar_About (J, F, D, Wt.all, Rs.all, Fs.all, Result.Score, Sc, Sd);
                              if Sc /= 0.0 and then Sd < Result.Score then
                                 Result := (Score => Sd, W => D, Phi => 0.0, Scale => Sc);
                              end if;
                           end;
                        end loop;
                        if Result.Scale /= 0.0 then
                           declare
                              Ph, Md : Real;
                           begin
                              Best_Phi (J, F, Result.W, Result.Scale, Ph, Md, Ga.all, Gb.all, Wt.all, Rs.all);
                              Result.Phi := Ph;
                              Result.Score := Md;
                           end;
                        end if;
                        --  The grid's best direction refined by least squares on every sighting (Refine).
                        if Result.Scale /= 0.0 and then Every (J)'Length > 4 then
                           Refine (J, F, Result);
                        end if;
                        Free (Ga);
                        Free (Gb);
                        Free (Wt);
                        Free (Rs);
                        Free (Fs);
                     end;
                     Best (J) := Result;
                  end Scanner;

                  Scanners : array (1 .. N) of Scanner;
               begin
                  for J in 1 .. N loop
                     Scanners (J).Start (J);
                  end loop;
               end;
               --  The curve the search walked, for the log: every focal length tried and its score (the joints' medians
               --  summed), so a wrong basin can be read against the true lens where that is known (the synthetic arms:
               --  115 px of 400 without the sign of the translation chosen, 20.7 with it, 10-14).
               declare
                  Sum : Real := 0.0;
                  Var : Real := 0.0;
               begin
                  for X of Best loop
                     Sum := Sum + X.Score;
                  end loop;
                  for J in 1 .. N loop
                     Var := Var + Median_Error (Best (J).Score, Counts (J)) ** 2;
                  end loop;
                  Ada.Strings.Unbounded.Append (Trace, " " & Driver.Log.Image (F, 1) & ":" & Driver.Log.Image (Sum, 3));
                  Tried_Ln.Append (Ln (F));
                  Tried_Total.Append (Sum);
                  Tried_Error.Append (Sqrt (Var));
               end;
            end Search;

            function Total (B : Best_Array) return Real is
               Sum : Real := 0.0;
            begin
               for X of B loop
                  Sum := Sum + X.Score;
               end loop;
               return Sum;
            end Total;

            --  The standard error of that sum, from each joint's median.
            function Total_Error (B : Best_Array) return Real is
               Sum : Real := 0.0;
            begin
               for J in 1 .. N loop
                  Sum := Sum + Median_Error (B (J).Score, Counts (J)) ** 2;
               end loop;
               return Sqrt (Sum);
            end Total_Error;

            --  Per joint: whether it slides (after the focal search), and its direction.
            Slid    : array (1 .. N) of Boolean := [others => False];
            Slide_W : array (1 .. N) of Vec3 := [others => [0.0, 0.0, 1.0]];

            Golden    : constant Real := (3.0 - Sqrt (5.0)) * Ada.Numerics.Pi;
            Inv_Ratio : constant Real := (Sqrt (5.0) - 1.0) / 2.0;
         begin
            for X of Pairs.all loop
               Seen_In (X.Frame) := Seen_In (X.Frame) + 1;
            end loop;
            for Frame in 2 .. Frames loop
               declare
                  J : constant Natural := Own_Joint (Frame);
               begin
                  if J > 0 and then Seen_In (Frame) > 0 then
                     if Own_Change (Frame, J) > 0.0
                       and then (Up_Frame (J) = 0 or else Own_Change (Frame, J) > Own_Change (Up_Frame (J), J))
                     then
                        Up_Frame (J) := Frame;
                     elsif Own_Change (Frame, J) < 0.0
                       and then (Down_Frame (J) = 0 or else Own_Change (Frame, J) < Own_Change (Down_Frame (J), J))
                     then
                        Down_Frame (J) := Frame;
                     end if;
                  end if;
               end;
            end loop;
            for X of Pairs.all loop
               if Widest (X.Frame) then
                  Grid_Counts (Own_Joint (X.Frame)) := Grid_Counts (Own_Joint (X.Frame)) + 1;
               end if;
            end loop;
            for I in 1 .. Directions loop
               declare
                  Z  : constant Real := 1.0 - 2.0 * (Real (I) - 0.5) / Real (Directions);
                  Rr : constant Real := Sqrt (Real'Max (0.0, 1.0 - Z * Z));
               begin
                  Sphere (I) := [Rr * Cos (Golden * Real (I)), Rr * Sin (Golden * Real (I)), Z];
               end;
            end loop;
            --  Some keyframe of a joint alone moved its matches, or there is nothing to fit.
            for Frame in 2 .. Frames loop
               if Own_Joint (Frame) > 0 then
                  declare
                     D : Real_Access := new Real_Array (1 .. S);
                     K : Natural := 0;
                  begin
                     for X of Pairs.all loop
                        if X.Frame = Frame then
                           K := K + 1;
                           D (K) := Sqrt ((X.U - X.U0) ** 2 + (X.V - X.V0) ** 2);
                        end if;
                     end loop;
                     Moving := Moving or else (K > 0 and then Driver.Stats.Median (D (1 .. K)) > 0.0);
                     Free (D);
                  end;
               end if;
            end loop;
            if not Moving then
               Fail ("the matches do not move");
               return;
            end if;
            --  Where to look for the focal length. The image moves by about the
            --  focal length times the eye's turn, but how far the eye turned is
            --  not the reading's change in any known unit (the turns are
            --  measured, Planar_About), so the motion says nothing of it: the
            --  search starts where the image's half width is seen at a right
            --  angle's half, and widens (below).
            --  Golden-section search in the logarithm of the focal length. The
            --  bracket, an octave about the start, first widens by its own width
            --  on the side whose end scores best, until an inner point scores
            --  better than both ends; the search ends when its two inner points
            --  cannot be told apart.
            declare
               --  From a start: the bracket, an octave about it, first widens by
               --  its own width on the side whose end scores best, until an inner
               --  point scores better than both ends (Low and High are that
               --  bracket: the basin the start fell in, as the search found it);
               --  then the golden section narrows it until its two inner points
               --  cannot be told apart, and Best_At is the better of them.
               procedure Settle (Start : Real; Low, High, Best_At, Within : out Real; Found : out Best_Array) is
                  A, B, X1, X2   : Real;
                  BA, BB, B1, B2 : Best_Array;
               begin
                  A := Start - Ln (2.0) / 2.0;
                  B := Start + Ln (2.0) / 2.0;
                  Search (Exp (A), BA);
                  Search (Exp (B), BB);
                  loop
                     X1 := B - Inv_Ratio * (B - A);
                     Search (Exp (X1), B1);
                     exit when Total (B1) <= Total (BA) and then Total (B1) <= Total (BB);
                     exit when B - A >= Ln (Real'Last);
                     if Total (BA) <= Total (BB) then
                        --  Lower: the inner point becomes the upper end.
                        B := X1;
                        BB := B1;
                        A := A - (B - A);
                        Search (Exp (A), BA);
                     else
                        A := X1;
                        BA := B1;
                        B := B + (B - A);
                        Search (Exp (B), BB);
                     end if;
                  end loop;
                  Low := A;
                  High := B;
                  X2 := A + Inv_Ratio * (B - A);
                  Search (Exp (X2), B2);
                  loop
                     exit when abs (Total (B1) - Total (B2)) <= Total_Error ((if Total (B1) < Total (B2) then B1 else B2));
                     exit when B - A <= Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs A);
                     if Total (B1) < Total (B2) then
                        B := X2;
                        X2 := X1;
                        B2 := B1;
                        X1 := B - Inv_Ratio * (B - A);
                        Search (Exp (X1), B1);
                     else
                        A := X1;
                        X1 := X2;
                        B1 := B2;
                        X2 := A + Inv_Ratio * (B - A);
                        Search (Exp (X2), B2);
                     end if;
                  end loop;
                  --  Half the bracket the search ended in, in the logarithm: the two inner points could not be told
                  --  apart by the score's error, so the focal length is told to about this.
                  Within := (B - A) / 2.0;
                  if Total (B1) < Total (B2) then
                     Best_At := X1;
                     Found := B1;
                  else
                     Best_At := X2;
                     Found := B2;
                  end if;
               end Settle;

               Low, High, Best_At : Real;
               Told_Within        : Real;
               Best          : Best_Array;
            begin
               Settle (Ln (Lens0.Cx), Low, High, Best_At, Told_Within, Best);
               --  The search finds the minimum of the basin its start falls in,
               --  and a start in the wrong basin finds the wrong focal length and,
               --  from it, the wrong order of the joints (A98's arm 1: starts of
               --  200 to 566 px were right, 100, 141 and 800 wrong). So the
               --  neighbouring basins are searched too, one basin's width below
               --  and above (the width the search itself found), as long as each
               --  scores better than the last: of all the minima found, the
               --  best is kept.
               for Side in 1 .. 2 loop
                  loop
                     declare
                        Width : constant Real := High - Low;
                        Start : constant Real := (if Side = 1 then Low - Width else High + Width);
                        L2, H2, A2, W2 : Real;
                        F2             : Best_Array;
                     begin
                        exit when Width <= 0.0 or else abs Start >= Ln (Real'Last) / 2.0;
                        Settle (Start, L2, H2, A2, W2, F2);
                        exit when Total (F2) >= Total (Best);
                        Low := L2;
                        High := H2;
                        Best_At := A2;
                        Told_Within := W2;
                        Best := F2;
                     end;
                  end loop;
               end loop;
               --  The focal lengths the score cannot tell from the best: every one tried whose score is not
               --  significantly the larger, at Z of the two scores' errors together. The golden section ends when its
               --  two inner points cannot be told apart, but each step that narrowed it to them compared two scores by
               --  which was lower, however little: on b05 (seed 7, 10-17) it ended at 111.5 px "told within 100.8 to
               --  123.3" where every focal length from 86.8 to 147.4 scored within the error of the best, the true
               --  135.3 among them, and the fit after it, held to that width by its prior (Refine_Tracks), ordered the
               --  joints wrong at the wrong focal length and was not fitted. The width given on is what the curve tells.
               declare
                  Best_Total : constant Real := Total (Best);
                  Best_Error : constant Real := Total_Error (Best);
               begin
                  for I in Tried_Ln.First_Index .. Tried_Ln.Last_Index loop
                     if not Driver.Uncertain.Significant
                              (Tried_Total (I) - Best_Total, Sqrt (Tried_Error (I) ** 2 + Best_Error ** 2))
                     then
                        Told_Within := Real'Max (Told_Within, abs (Tried_Ln (I) - Best_At));
                     end if;
                  end loop;
               end;
               Focal_Within := Told_Within;
               Searched_Least := F_Least;
               Searched_Most := F_Most;
               declare
                  Focal : constant Real := Exp (Best_At);
               begin
               --  What the search did, for the log: how many focal lengths it tried and how far they ranged
               --  (B0a_1's arms ended at 0.002 px: the search had run to the bracket's end).
               Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " focal search tried" & Searches'Image
                                & " lengths from " & Driver.Log.Image (F_Least, 3) & " to " & Driver.Log.Image (F_Most, 1)
                                & " px, best " & Driver.Log.Image (Focal, 1) & " px, score "
                                & Driver.Log.Image (Total (Best), 4) & " +- " & Driver.Log.Image (Total_Error (Best), 4)
                                & " px; the basin " & Driver.Log.Image (Exp (Low), 1) & " to " & Driver.Log.Image (Exp (High), 1)
                                & ", told within " & Driver.Log.Image (Exp (Best_At - Told_Within), 1) & " to "
                                & Driver.Log.Image (Exp (Best_At + Told_Within), 1));
               Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " focal search curve, px:score"
                                & Ada.Strings.Unbounded.To_String (Trace));
               --  A joint that slides: one whose keyframes the eye's travel along one direction without turning
               --  explains as well as a turn does, the turn's own freedom (its size) not lowering the median residual
               --  of the joint's keyframes significantly (Z of the two medians' errors together): a wheeled base's
               --  travel, a lift, a drone's motors. A turning joint turns the eye by its whole turn, which every
               --  keyframe of it shows.
               Prepare (Focal);
               for J in 1 .. N loop
                  if Every (J)'Length > 4 then
                     declare
                        D  : Vec3;
                        Sd : Real;
                     begin
                        Slide_Of (J, Focal, D, Sd);
                        if Sd < Real'Last
                          and then not (Sd - Best (J).Score
                                        > Driver.Conventions.Z
                                          * Sqrt (Median_Error (Sd, Counts (J)) ** 2
                                                  + (if Best (J).Score < Real'Last
                                                     then Median_Error (Best (J).Score, Counts (J)) ** 2 else 0.0)))
                        then
                           Slid (J) := True;
                           Slide_W (J) := D;
                           Slide_Turns (J) := Best (J).Scale * Best (J).W;
                           Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " joint" & J'Image
                                            & " slides: a slide leaves its keyframes " & Driver.Log.Image (Sd, 3)
                                            & " px, a turn " & Driver.Log.Image (Best (J).Score, 3) & " px");
                        end if;
                     end;
                  end if;
               end loop;
               for J in 1 .. N loop
                  if Best (J).Scale = 0.0 and then not Slid (J) then
                     for K in 1 .. N loop
                        Free (Lines (K));
                        Free (Every (K));
                        Free (Centred (K));
                     end loop;
                     Fail ("joint" & J'Image & "'s widest keyframes tell its turn about no direction");
                     return;
                  end if;
               end loop;
               Lens0.Fx := Focal;
               Lens0.Fy := Focal;
               for J in 1 .. N loop
                  declare
                     E1, E2 : Vec3;
                  begin
                     Perp (Best (J).W, E1, E2);
                     Joints (Joints'First + J - 1) :=
                       (if Slid (J) then (W => Slide_W (J), P => Zero3, C => 1.0, Slide => True)
                        else (W => Best (J).W, P => Cos (Best (J).Phi) * E1 + Sin (Best (J).Phi) * E2,
                              C => Best (J).Scale, Slide => False));
                  end;
                  Free (Lines (J));
                  Free (Every (J));
                  Free (Centred (J));
               end loop;
               end;
            end;
         end;
      end if;

      ---------------------------------------------------------------------------
      --  Stage 2: the joints together with the focal length, on the keyframes
      --  that moved one joint each. Per joint: the axis's tilt in its tangent
      --  plane (2), the side of the eye (1) and the logarithm of the size of
      --  its turn per reading unit (1; its sign is stage 1's); then the log
      --  focal length.
      --  A joint whose own keyframes left its turn per reading unit nought (none of their sightings told the view turned
      --  with it) has no size whose logarithm stage 2 could fit: the arm is not fitted, and why is said. The fit took the
      --  logarithm of nought and the driver ended (s29 replayed, 10-17, with the points that ride with the eye left out:
      --  the wrist roll's keyframes far out kept no point of the world).
      for J in 1 .. N loop
         if not (abs Joints (Joints'First + J - 1).C > 0.0) then
            Fail ("joint" & J'Image & "'s own keyframes leave its turn per reading unit nought: none of their sightings"
                  & " tells how far it turns the view");
            return;
         end if;
      end loop;
      Enter (2);
      declare
         Count : constant Positive := 4 * N + 1;
         Base  : constant Joint_Array := Joints;
         Used  : Natural := 0;
      begin
         for X of Pairs.all loop
            if Own_Joint (X.Frame) > 0 then
               Used := Used + 1;
            end if;
         end loop;
         declare
            Kept : Sighting_Access := new Sighting_Array (1 .. Used);
            K    : Natural := 0;

            function Joints_Of (X : Real_Array) return Joint_Array is
               Out_J : Joint_Array := Base;
            begin
               for J in 1 .. N loop
                  declare
                     O : constant Natural := X'First + 4 * (J - 1);
                     E1, E2, F1, F2 : Vec3;
                     W : Vec3;
                  begin
                     Perp (Base (Base'First + J - 1).W, E1, E2);
                     W := Unit (Base (Base'First + J - 1).W + X (O) * E1 + X (O + 1) * E2);
                     Perp (W, F1, F2);
                     Out_J (Out_J'First + J - 1).W := W;
                     Out_J (Out_J'First + J - 1).P :=
                       (if Base (Base'First + J - 1).Slide then Zero3 else Cos (X (O + 2)) * F1 + Sin (X (O + 2)) * F2);
                     --  A slide's length per reading unit is not what an epipolar residual sees (it scales the
                     --  translation): held, its term no term at all, which its rounding alone moved to 1E294 (10-20).
                     Out_J (Out_J'First + J - 1).C :=
                       (if Base (Base'First + J - 1).Slide then Base (Base'First + J - 1).C
                        elsif Base (Base'First + J - 1).C < 0.0 then -Exp (X (O + 3)) else Exp (X (O + 3)));
                  end;
               end loop;
               return Out_J;
            end Joints_Of;

            procedure Evaluate (X : Real_Array; R : out Real_Array) is
               Jx : constant Joint_Array := Joints_Of (X);
               F  : constant Real := Exp (X (X'Last));
               Lx : constant Lens := (Fx | Fy => F, Cx => Lens0.Cx, Cy => Lens0.Cy, others => 0.0);
            begin
               for I in 1 .. Used loop
                  declare
                     J  : constant Positive := Own_Joint (Kept (I).Frame);
                     A  : Joint renames Jx (Jx'First + J - 1);
                     Rr : Mat3;
                     Tt : Vec3;
                  begin
                     Relative (Identity, Step_Of (A, Own_Change (Kept (I).Frame, J)), Rr, Tt);
                     R (R'First + I - 1) :=
                       Sampson (Rr, Tt, Ray (Lx, Kept (I).U0, Kept (I).V0), Ray (Lx, Kept (I).U, Kept (I).V), F);
                  end;
               end loop;
            end Evaluate;

            procedure Solve is new Robust_Fit
              (Count, Used, Driver.Conventions.Initial_Damping, Evaluate, At_Once => True);
            X : Real_Array (1 .. Count) := [others => 0.0];
         begin
            for Xs of Pairs.all loop
               if Own_Joint (Xs.Frame) > 0 then
                  K := K + 1;
                  Kept (K) := Xs;
               end if;
            end loop;
            for J in 1 .. N loop
               declare
                  E1, E2 : Vec3;
               begin
                  --  The side as an angle in the tangent frame of the axis.
                  Perp (Base (Base'First + J - 1).W, E1, E2);
                  X (4 * (J - 1) + 3) :=
                    (if Base (Base'First + J - 1).Slide then 0.0
                     else Arctan (Base (Base'First + J - 1).P * E2, Base (Base'First + J - 1).P * E1));
                  X (4 * (J - 1) + 4) := Ln (abs Base (Base'First + J - 1).C);
               end;
            end loop;
            X (Count) := Ln (Lens0.Fx);
            if Used > Count then
               declare
                  R : Real_Access := new Real_Array (1 .. Used);
               begin
                  Evaluate (X, R.all);
                  Solve (X, Noise_Of (R.all));
                  Free (R);
               end;
               Joints := Joints_Of (X);
               Lens0.Fx := Exp (X (Count));
               Lens0.Fy := Lens0.Fx;
            end if;
            Free (Kept);
         end;
      end;

      ---------------------------------------------------------------------------
      --  Stage 3: how far every axis lies from the eye, relative to the
      --  others. One joint's keyframes tell its axis but not its distance:
      --  scaling that distance scales the depths its keyframes triangulate.
      --  A point the reference keyframe tracks into keyframes of two joints
      --  has one depth, so the ratio of the depths each joint's keyframes give
      --  it is the inverse ratio of their distances; the sign of each joint's
      --  distance puts the points in front.
      Enter (3);
      declare
         Tracks : Natural := 0;
      begin
         for X of Sight loop
            Tracks := Natural'Max (Tracks, X.Track);
         end loop;
         declare
            --  Per joint and track: the depth along the reference ray its own
            --  keyframes triangulate, with the joint at distance one.
            Depth : Grid_Access := new Real_Matrix'[1 .. N => [1 .. Tracks => 0.0]];
            Seen  : Flag_Grid_Access := new Flag_Grid'[1 .. N => [1 .. Tracks => False]];
            Num   : Grid_Access := new Real_Matrix'[1 .. N => [1 .. Tracks => 0.0]];
            Den   : Grid_Access := new Real_Matrix'[1 .. N => [1 .. Tracks => 0.0]];
            Rho   : Real_Array (1 .. N) := [others => 1.0];

            procedure Free_All is
            begin
               Free (Depth);
               Free (Seen);
               Free (Num);
               Free (Den);
            end Free_All;
         begin
            for X of Pairs.all loop
               declare
                  J : constant Natural := Own_Joint (X.Frame);
               begin
                  if J > 0 then
                     declare
                        A  : Joint renames Joints (Joints'First + J - 1);
                        St : constant Rigid := Step_Of (A, Own_Change (X.Frame, J));
                        R  : constant Mat3 := St.Rotation;
                        T  : constant Vec3 := St.Translation;
                        H0 : constant Vec3 := Ray (Lens0, X.U0, X.V0);
                        Bv : constant Vec3 := R * Ray (Lens0, X.U, X.V);
                        --  d H0 = T + s Bv: the part of each across Bv.
                        function Across (V : Vec3) return Vec3 is (V - (Dot (V, Bv) / Dot (Bv, Bv)) * Bv);
                     begin
                        Num (J, X.Track) := Num (J, X.Track) + Across (H0) * Across (T);
                        Den (J, X.Track) := Den (J, X.Track) + Across (H0) * Across (H0);
                     end;
                  end if;
               end;
            end loop;
            for J in 1 .. N loop
               for I in 1 .. Tracks loop
                  if Den (J, I) > 0.0 then
                     Depth (J, I) := Num (J, I) / Den (J, I);
                     Seen (J, I) := Depth (J, I) /= 0.0;
                  end if;
               end loop;
            end loop;
            --  Each joint's sign: most of its points in front.
            for J in 1 .. N loop
               declare
                  Ahead, Behind : Natural := 0;
               begin
                  for I in 1 .. Tracks loop
                     if Seen (J, I) then
                        if Depth (J, I) > 0.0 then
                           Ahead := Ahead + 1;
                        else
                           Behind := Behind + 1;
                        end if;
                     end if;
                  end loop;
                  if Behind > Ahead then
                     if Joints (Joints'First + J - 1).Slide then
                        Joints (Joints'First + J - 1).C := -Joints (Joints'First + J - 1).C;
                     else
                        Joints (Joints'First + J - 1).P := -Joints (Joints'First + J - 1).P;
                     end if;
                     for I in 1 .. Tracks loop
                        Depth (J, I) := -Depth (J, I);
                     end loop;
                  end if;
               end;
            end loop;
            --  The distances' logarithms from every pair of joints that share
            --  points (log rho_b - log rho_a = log (d_a / d_b)), the median of
            --  each pair, solved by least squares with the joint that shares
            --  the most at log 1.
            declare
               type Pair is record
                  A, B  : Positive := 1;
                  Value : Real := 0.0;
                  Count : Natural := 0;
               end record;
               Pairs : array (1 .. N * N) of Pair;
               P_Count : Natural := 0;
               Shared  : array (1 .. N) of Natural := [others => 0];
            begin
               for A in 1 .. N loop
                  for B in A + 1 .. N loop
                     declare
                        Logs : Real_Access := new Real_Array (1 .. Tracks);
                        K    : Natural := 0;
                     begin
                        for I in 1 .. Tracks loop
                           if Seen (A, I) and then Seen (B, I) and then Depth (A, I) > 0.0 and then Depth (B, I) > 0.0 then
                              K := K + 1;
                              Logs (K) := Ln (Depth (A, I) / Depth (B, I));
                           end if;
                        end loop;
                        if K > 0 then
                           P_Count := P_Count + 1;
                           Pairs (P_Count) := (A => A, B => B, Value => Driver.Stats.Median (Logs (1 .. K)), Count => K);
                           Shared (A) := Shared (A) + K;
                           Shared (B) := Shared (B) + K;
                        end if;
                        Free (Logs);
                     end;
                  end loop;
               end loop;
               declare
                  Anchor : Positive := 1;
               begin
                  for J in 2 .. N loop
                     if Shared (J) > Shared (Anchor) then
                        Anchor := J;
                     end if;
                  end loop;
                  --  Fewer pairs than joints beyond the anchor cannot determine the distances: the least
                  --  squares below needs at least as many rows as unknowns (its precondition; a random body,
                  --  B0aF1 of 10-13, reached it with too few pairs and the estimate died on the assertion
                  --  instead of saying so).
                  --  What the joints share, for the log: a point seen in two joints' own keyframes, with a depth
                  --  from each, ties their distances; a pair with none leaves a joint's distance free.
                  declare
                     Text : Ada.Strings.Unbounded.Unbounded_String;
                  begin
                     for K in 1 .. P_Count loop
                        Ada.Strings.Unbounded.Append
                          (Text, " " & Pairs (K).A'Image & "-" & Ada.Strings.Fixed.Trim (Pairs (K).B'Image, Ada.Strings.Left)
                           & ":" & Pairs (K).Count'Image);
                     end loop;
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " stage 3 shares tracks between joints"
                                      & (if P_Count > 0 then Ada.Strings.Unbounded.To_String (Text) else " none")
                                      & ", of" & Tracks'Image & " tracks");
                  end;
                  if N > 1 and then P_Count > 0 and then P_Count + 1 < N then
                     Fail ("the joints share too few tracked points to tell their distances apart");
                     Free_All;
                     return;
                  end if;
                  if N > 1 and then P_Count > 0 then
                     declare
                        --  Unknowns: log rho of every joint but the anchor.
                        M_A : Real_Matrix (1 .. P_Count + 1, 1 .. N) := [others => [others => 0.0]];
                        M_B : Real_Vector (1 .. P_Count + 1) := [others => 0.0];
                        Sol : Real_Vector (1 .. N);
                        Full : Boolean;
                     begin
                        for K in 1 .. P_Count loop
                           declare
                              Weight : constant Real := Sqrt (Real (Pairs (K).Count));
                           begin
                              M_A (K, Pairs (K).B) := Weight;
                              M_A (K, Pairs (K).A) := -Weight;
                              M_B (K) := Weight * Pairs (K).Value;
                           end;
                        end loop;
                        --  The anchor's row: its log is zero.
                        M_A (P_Count + 1, Anchor) := 1.0;
                        Driver.Numerics.Dense.Least_Squares (M_A, M_B, Sol, Full);
                        if not Full then
                           Fail ("the joints share too few tracked points to tell their distances apart");
                           Free_All;
                           return;
                        end if;
                        for J in 1 .. N loop
                           Rho (J) := Exp (Sol (J));
                        end loop;
                     end;
                  elsif N > 1 then
                     Fail ("no point is tracked into the keyframes of two joints");
                     Free_All;
                     return;
                  end if;
               end;
            end;
            for J in 1 .. N loop
               if Joints (Joints'First + J - 1).Slide then
                  Joints (Joints'First + J - 1).C := Rho (J) * Joints (Joints'First + J - 1).C;
               else
                  Joints (Joints'First + J - 1).P := Rho (J) * Joints (Joints'First + J - 1).P;
               end if;
            end loop;
            Free_All;
         end;
      end;

      Free (Pairs);

      ---------------------------------------------------------------------------
      --  Stage 4 begins with every keyframe of one joint in one refinement of
      --  the tracks (Refine_Tracks), which any order of the joints explains
      --  alike (one whose partner is not the reference, nearly: what it turned
      --  besides its own joint is a landing, within the steps the eyes see). A joint whose turn shifts the view the way its shift moves it
      --  (a pitch joint whose axis lies behind the eye) has its turn told by
      --  no epipolar residual: a point's depth along its line of sight takes it
      --  up. The depth every point has for all the joints pins it (the
      --  synthetic arm, its readings scaled: two joints at 0.60 and 0.82 of
      --  their turns after stage 2). The lens, the joints and every point's
      --  depth come out of it, for the order.
      Enter (4);
      declare
         Alone_Count : Natural := 0;
      begin
         for X of Sight loop
            if Own_Joint (X.Frame) > 0 then
               Alone_Count := Alone_Count + 1;
            end if;
         end loop;
         declare
            Alone : Sighting_Access := new Sighting_Array (1 .. Alone_Count);
            K     : Natural := 0;
            Lx    : Lens := Lens0;
            Rep   : Fit_Report;
         begin
            for X of Sight loop
               if Own_Joint (X.Frame) > 0 then
                  K := K + 1;
                  Alone (K) := X;
               end if;
            end loop;
            Refine_Tracks (Changes, Alone.all, Joints, Order, Lx, Rep, Focal_Within, Spread => False);
            --  A joint taken for a slide is weighed again against the turn its own keyframes were explained by nearly
            --  as well (Slide_Turns), now that the lens is refined: the turn whose rotation is that turn's across the
            --  slide's travel and whose eye travels as the slide's does (its axis through the point that makes it so),
            --  refined on the same sightings, takes the slide's place when the two are not the same joint within what
            --  its keyframes show (at its widest turn either way they put the points apart by Z of their median's error,
            --  as two orders are the same arm in stage 4), it leaves the sightings a robust cost below the slide's (as
            --  the turn's best must, the slide being a turn's limit) and the refinement's covariance (which holds how
            --  the sightings' errors depend on each other) determines it as a turn (Joint_Determined: its axis within a
            --  quarter turn at Z, its turn per reading unit significant). Determined alone, the base arm of the bench's
            --  seed 3 took a turn of 2.2E-6 +- 4.4E-7 rad a unit, its axis to 0.51 rad, for a slide the fake robot moves
            --  without turning, and was not fitted (10-20). By a likelihood ratio of the robust
            --  costs instead, which counts every sighting's error as its own, the drones' slides of the step 2 bench were
            --  turned into turns of 8E-7 +- 4E-7 rad a unit that left them undetermined (seeds 3, 4 and 6, 10-20); by
            --  the turn's rate alone, into turns of 6E-7 +- 9E-8 rad a unit (0.0004 rad over the joint's keyframes)
            --  whose axes the sightings told within no quarter turn (seeds 1, 3 and 5). Stage 1 weighs the two at the
            --  focal search's lens: a
            --  wide lens's barrel distortion left out and the focal at 248 px of 400, a desktop arm's second joint was
            --  taken for a slide (0.821 px against a turn's 0.690, not significantly the larger), and the fit ended at
            --  366 +- 9 px, its lens off by a chi square of 1957 on six terms (robot.kinematics.desktop, 10-20).
            for J in 1 .. N loop
               declare
                  A    : constant Joint := Joints (Joints'First + J - 1);
                  V    : constant Vec3 := A.C * A.W;   --  the slide's travel per reading unit
                  Turn : constant Vec3 := Slide_Turns (J);
                  Omega : constant Vec3 :=
                    (if Dot (V, V) > 0.0 then Turn + (-(Dot (Turn, V) / Dot (V, V))) * V else Turn);
               begin
                  if A.Slide and then Rep.All_Count > 0 and then Dot (Omega, Omega) > 0.0 then
                     declare
                        Rate : constant Real := Sqrt (Dot (Omega, Omega));
                        Jt   : Joint_Array := Joints;
                        Lt   : Lens := Lx;
                        Rt   : Fit_Report;
                     begin
                        Jt (Jt'First + J - 1) :=
                          (W => (1.0 / Rate) * Omega, P => (1.0 / Rate ** 2) * Cross (Omega, V), C => Rate,
                           Slide => False);
                        Refine_Tracks (Changes, Alone.all, Jt, Order, Lt, Rt, Focal_Within,
                                       Judge_Floor => Rep.Judge_Floor, Judge_Gain => Rep.Judge_Gain);
                        declare
                           Known : constant Boolean := Natural (Rt.Covariance.Length) = Terms (N) ** 2;
                           function Variance_Of (T : Joint_Term) return Real is
                             (Rt.Covariance (Rt.Covariance.First_Index + (Term_Of (J, T) - 1) * Terms (N) + Term_Of (J, T) - 1));
                           Rate_Sigma : constant Real := (if Known then Sqrt (Variance_Of (Scale)) else Real'Last);
                           --  How far apart the slide's fit and the turn's put the points at the joint's widest turn in
                           --  its keyframes, either way, each by its own lens and depths: the larger median over the
                           --  error of a median of that many at the slide's noise.
                           function Apart return Real is
                              Reach : Real := 0.0;
                              Most  : Real := 0.0;
                           begin
                              for F in 1 .. Frames loop
                                 if Own_Joint (F) = J then
                                    Reach := Real'Max (Reach, abs Change (F, J));
                                 end if;
                              end loop;
                              for Sign of Real_Array'[1.0, -1.0] loop
                                 declare
                                    D      : Real_Array (1 .. N) := [others => 0.0];
                                    Back_S, Back_T : Rigid;
                                    Seen_T : array (1 .. Natural (Rep.Depths.Length)) of Boolean := [others => False];
                                    Dist   : Real_Access := new Real_Array (1 .. Natural'Max (1, Seen_T'Length));
                                    Count  : Natural := 0;
                                 begin
                                    D (J) := Sign * Reach;
                                    Back_S := Inverse (Eye_At (Joints, Order, D));
                                    Back_T := Inverse (Eye_At (Jt, Order, D));
                                    for X of Alone.all loop
                                       if X.Track in Seen_T'Range and then not Seen_T (X.Track)
                                         and then X.Track <= Natural (Rt.Depths.Length)
                                       then
                                          Seen_T (X.Track) := True;
                                          declare
                                             Ds : constant Real := Rep.Depths (Rep.Depths.First_Index + X.Track - 1);
                                             Dt : constant Real := Rt.Depths (Rt.Depths.First_Index + X.Track - 1);
                                             U1, V1, U2, V2 : Real;
                                             A1, A2         : Boolean;
                                          begin
                                             if Ds > 0.0 and then Dt > 0.0 then
                                                Project (Lx, Back_S * (Ds * Ray (Lx, X.U0, X.V0)), U1, V1, A1);
                                                Project (Lt, Back_T * (Dt * Ray (Lt, X.U0, X.V0)), U2, V2, A2);
                                                if A1 and then A2 and then U1 in 0.0 .. Real (Width)
                                                  and then V1 in 0.0 .. Real (Height)
                                                then
                                                   Count := Count + 1;
                                                   Dist (Count) := Sqrt ((U1 - U2) ** 2 + (V1 - V2) ** 2);
                                                end if;
                                             end if;
                                          end;
                                       end if;
                                    end loop;
                                    if Count > 0 and then Rep.Sigma_Px > 0.0 then
                                       Most := Real'Max (Most, Driver.Stats.Median (Dist (1 .. Count))
                                                               / Driver.Stats.Median_Sigma (Rep.Sigma_Px, Count));
                                    end if;
                                    Free (Dist);
                                 end;
                              end loop;
                              return Most;
                           end Apart;
                           Power   : constant Real := Apart;
                           Turns   : constant Boolean :=
                             Rt.All_Count > 0 and then Power > Driver.Conventions.Z
                             and then Rt.Cost_Judged < Rep.Cost_Judged and then Known
                             and then Joint_Determined (Variance_Of (Tilt_1), Variance_Of (Tilt_2), Variance_Of (Scale),
                                                        Jt (Jt'First + J - 1).C);
                        begin
                           Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " joint" & J'Image
                                            & " weighed as a turn against its slide: every sighting's cost "
                                            & Driver.Log.Image (Rt.Cost_Judged - Rep.Cost_Judged, 1)
                                            & " against the slide's, median " & Driver.Log.Image (Rt.All_Median_Px, 3)
                                            & " px against " & Driver.Log.Image (Rep.All_Median_Px, 3) & ", its turn"
                                            & Real'Image (Jt (Jt'First + J - 1).C) & " +-"
                                            & (if Rate_Sigma < Real'Last then Real'Image (Rate_Sigma) else " unknown")
                                            & " rad a unit, its axis to"
                                            & (if Known then Real'Image (Sqrt (Variance_Of (Tilt_1) + Variance_Of (Tilt_2)))
                                               else " unknown")
                                            & " rad, the two apart by " & Driver.Log.Image (Power, 1)
                                            & " of their error" & (if Turns then "; it turns" else "; it slides"));
                           if Turns then
                              Joints := Jt;
                              Lx := Lt;
                              Rep := Rt;
                              Slide_Turns (J) := Zero3;
                           end if;
                        end;
                     end;
                  end if;
               end;
            end loop;
            Lens0 := Lx;
            Alone_Depths := Rep.Depths;
            Free (Alone);
         end;
      end;

      ---------------------------------------------------------------------------
      --  Stage 4: the order of the joints along the arm. The evidence is the sightings of the keyframes that moved
      --  several joints at once (Several): every order explains a keyframe of one joint alike, and in one of several
      --  the joints' steps compose in the order's sequence. An order is judged by its own fit: the tracks refined in
      --  it (Refine_Tracks) on every sighting, from the joints of the keyframes of one joint each, and the median
      --  residual it leaves over every sighting. Judged with joints fitted in another order the true order lost to the
      --  order they were fitted in: every order scored with the joints of the keyframes of one joint each told wrong
      --  orders of four arms of the step 2 bench (10-19: seeds 15, 33, 36 and 39), each the least of its 720 once
      --  fitted in itself, and the order a fit then refined it in told itself again.
      --  The search: from the order kept before (From) or the joints' own, the order the cheap score finds least over
      --  every order (Cheap with the order kept's own joints, lens and depths: Global) and every order one joint's move
      --  from the order kept that it puts as near, each fitted in itself; the order kept gives way to the one whose
      --  fit leaves every sighting least, until none does. When the order kept explains the keyframes of several
      --  joints worse than those of one (Adequacy), every order one move from it is weighed: the cheap score with
      --  joints fitted to a wrong order screened the right one out (s36 of the bench, 10-20: one swap from the truth).
      --  Moved by single moves alone, the search stood in wrong orders whose every neighbour, its joints fitted to
      --  the order, scored worse (f16b's arm 1, 10-15; the bench's s39, 10-20, at 1 2 5 3 4 6 for 5 4 1 6 2 3), where
      --  the cheap score over every order with those same joints put the truth first.
      --  The order is told when it explains the keyframes of several joints as it does those of one, and every order
      --  weighed against it is told from it by the keyframes that put the two apart (Told_Over: over every keyframe,
      --  and over each half alone) or is the same arm within what its joints can turn: no turn of the pairs it swaps
      --  that the aligner follows puts the points apart by Z of the median's error (two slides; a slide along a
      --  turn's axis). The pairs the untold ones swap are turned by Boot together (Untold, Untold_Turns), each by the
      --  turn that tells one of them most within what the aligner follows.
      Order_Is_Told := N <= 1;
      if N > 1 and then From.Fixed then
         --  A fit of one order kept as given: its search is the caller's.
         Order := From.Chain;
      elsif N > 1 then
         if (for all X of Sight => not Several (X.Frame)) then
            Fail ("no keyframe moved two joints at once: the order of the joints along the arm is not told");
            return;
         end if;
         Search_Order : declare
            type Order_Fit is record
               Order  : Chain_Order (1 .. N);
               Joints : Joint_Array (1 .. N);
               L      : Lens;
               Report : Fit_Report;
            end record;

            type Step_Table is array (Positive range <>, Positive range <>) of Rigid;
            type Step_Access is access Step_Table;
            procedure Free is new Ada.Unchecked_Deallocation (Step_Table, Step_Access);
            type Point_Table is array (Positive range <>) of Vec3;
            type Point_Access is access Point_Table;
            procedure Free is new Ada.Unchecked_Deallocation (Point_Table, Point_Access);
            type Fit_Table is array (Positive range <>) of Order_Fit;
            type Fit_Table_Access is access Fit_Table;
            procedure Free is new Ada.Unchecked_Deallocation (Fit_Table, Fit_Table_Access);
            type Order_Table is array (Positive range <>) of Chain_Order (1 .. N);
            type Order_Table_Access is access Order_Table;
            procedure Free is new Ada.Unchecked_Deallocation (Order_Table, Order_Table_Access);

            function Image (O : Chain_Order) return String is
               Text : Ada.Strings.Unbounded.Unbounded_String;
            begin
               for J of O loop
                  Ada.Strings.Unbounded.Append (Text, J'Image);
               end loop;
               return Ada.Strings.Unbounded.To_String (Text);
            end Image;

            Arm : constant String := (if Who = "" then "the arm" else Who);

            --  The sightings by keyframe: those of keyframe F are By_Frame (First_Of (F) .. First_Of (F) + Count_Of (F)
            --  - 1), indices into Sight.
            Count_Of, First_Of : array (1 .. Frames) of Natural := [others => 0];
            By_Frame : Count_Access := new Count_Array (1 .. S);
            --  The keyframes of several joints with sightings, and how many sightings they have.
            Multi_Frames : Count_Access;
            Multi_Count  : Natural := 0;

            function Depth_Of (X : Order_Fit; Track : Positive) return Real is
              (if Track <= Natural (X.Report.Depths.Length)
               then X.Report.Depths (X.Report.Depths.First_Index + Track - 1) else 0.0);

            Fits_Made : Natural := 0;

            --  Where X's joints, lens and depths in order C put every sighting (Real'Last: nowhere, or no depth).
            procedure Predict (X : Order_Fit; C : Chain_Order; Pu, Pv : out Real_Array) is
            begin
               Pu := [others => Real'Last];
               Pv := [others => Real'Last];
               for F in 1 .. Frames loop
                  if Count_Of (F) > 0 then
                     declare
                        Back : constant Rigid := Inverse (Eye_At (X.Joints, C, Changes_Of (F)));
                     begin
                        for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                           declare
                              I     : constant Positive := By_Frame (Q);
                              Sg    : Sighting renames Sight (Sight'First + I - 1);
                              Depth : constant Real := Depth_Of (X, Sg.Track);
                              U, V  : Real;
                              Ahead : Boolean;
                           begin
                              if Depth > 0.0 then
                                 Project (X.L, Back * (Depth * Ray (X.L, Sg.U0, Sg.V0)), U, V, Ahead);
                                 if Ahead then
                                    Pu (Pu'First + I - 1) := U;
                                    Pv (Pv'First + I - 1) := V;
                                 end if;
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
            end Predict;

            --  The cheap score of an order with X's joints, lens and depths, fast: each joint's step at every
            --  keyframe of several joints and the point every sighting there has, made once (Prepare); an order's
            --  pose at a keyframe is the product of the steps in its sequence, as Eye_At composes them, and its score
            --  the median residual across and down of those sightings (Cheap), Cheap_Count of them.
            Steps      : Step_Access := new Step_Table (1 .. Frames, 1 .. N);
            Points     : Point_Access := new Point_Table (1 .. S);
            Has_Point  : Flags_Access := new Flags'(1 .. S => False);
            Lens_Now   : Lens;
            Residuals  : Real_Access;
            Cheap_Count : Natural := 0;
            Cheaps     : Natural := 0;   --  orders scored so, for the log

            procedure Prepare (X : Order_Fit) is
            begin
               Lens_Now := X.L;
               for K in 1 .. Multi_Count loop
                  declare
                     F : constant Positive := Multi_Frames (K);
                  begin
                     for J in 1 .. N loop
                        Steps (F, J) := Step_Of (X.Joints (J), Change (F, J));
                     end loop;
                     for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                        declare
                           I     : constant Positive := By_Frame (Q);
                           Sg    : Sighting renames Sight (Sight'First + I - 1);
                           Depth : constant Real := Depth_Of (X, Sg.Track);
                        begin
                           Has_Point (I) := Depth > 0.0;
                           if Has_Point (I) then
                              Points (I) := Depth * Ray (X.L, Sg.U0, Sg.V0);
                           end if;
                        end;
                     end loop;
                  end;
               end loop;
            end Prepare;

            function Cheap (C : Chain_Order) return Real is
               K : Natural := 0;
            begin
               Cheaps := Cheaps + 1;
               for M in 1 .. Multi_Count loop
                  declare
                     F    : constant Positive := Multi_Frames (M);
                     T    : Rigid := Identity;
                     Back : Rigid;
                  begin
                     for J of C loop
                        T := T * Steps (F, J);
                     end loop;
                     Back := Inverse (T);
                     for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                        declare
                           I : constant Positive := By_Frame (Q);
                           U, V  : Real;
                           Ahead : Boolean;
                        begin
                           if Has_Point (I) then
                              Project (Lens_Now, Back * Points (I), U, V, Ahead);
                              Residuals (K + 1) := (if Ahead then abs (U - Sight (Sight'First + I - 1).U) else Real'Last);
                              Residuals (K + 2) := (if Ahead then abs (V - Sight (Sight'First + I - 1).V) else Real'Last);
                              K := K + 2;
                           end if;
                        end;
                     end loop;
                  end;
               end loop;
               Cheap_Count := K;
               return (if K = 0 then Real'Last else Driver.Stats.Median (Residuals (1 .. K)));
            end Cheap;

            --  Order C fitted in itself on the lens and every track's depth of the joints of the keyframes of one joint
            --  each (X), which no order changes: its joints alone, from X's, by robust least squares of every sighting's
            --  residual across and down (Robust_Fit, Huber at the noise X's residuals in C show), the terms of each
            --  joint its axis's tilt (two), a turning joint's point across its axis (two) and its scale (one). Every
            --  order weighed shares the lens and the depths, so that what tells two orders apart is their joints'
            --  composing alone; and a refinement of the tracks in each (Refine_Tracks, 1 to 3 s an order of six joints)
            --  took the search past the two minutes a bench run has (the bench's s36 and the base's ten-joint arm,
            --  10-20), set the keyframes of several joints aside as not fitting where the joints of one joint's keyframes
            --  began far from them, and fitted them no closer (s36: every order, the true one too, at 1.2 px on them).
            procedure Quick_Fit (C : Chain_Order; X : Order_Fit; Into : out Order_Fit) is
               Per   : constant := 5;   --  terms a joint: its axis's tilt (2), its point across the axis (2), its scale
               Count : constant Positive := Per * N;
               Tilt_1, Tilt_2, Side_1, Side_2 : array (1 .. N) of Vec3;
               Track_Points : Point_Access := new Point_Table (1 .. S);   --  per sighting, its point at X's depth
               Has_Track    : Flags_Access := new Flags'(1 .. S => False);

               function Joints_Of (V : Real_Array) return Joint_Array is
                  Out_J : Joint_Array (1 .. N) := X.Joints;
               begin
                  for J in 1 .. N loop
                     declare
                        O : constant Natural := V'First + Per * (J - 1);
                        A : Joint renames X.Joints (J);
                     begin
                        Out_J (J).W := Unit (A.W + V (O) * Tilt_1 (J) + V (O + 1) * Tilt_2 (J));
                        if not A.Slide then
                           Out_J (J).P := A.P + V (O + 2) * Side_1 (J) + V (O + 3) * Side_2 (J);
                        end if;
                        Out_J (J).C := A.C * Exp (V (O + 4));
                     end;
                  end loop;
                  return Out_J;
               end Joints_Of;

               --  Every sighting's residual across and down with the joints of V in order C; a point the order puts
               --  behind the eye, as far off as the picture is wide.
               procedure Evaluate (V : Real_Array; R : out Real_Array) is
                  Jx : constant Joint_Array := Joints_Of (V);
               begin
                  R := [others => 0.0];
                  for F in 1 .. Frames loop
                     if Count_Of (F) > 0 then
                        declare
                           Back : constant Rigid := Inverse (Eye_At (Jx, C, Changes_Of (F)));
                        begin
                           for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                              declare
                                 I     : constant Positive := By_Frame (Q);
                                 U, Vv : Real;
                                 Ahead : Boolean;
                              begin
                                 if Has_Track (I) then
                                    Project (X.L, Back * Track_Points (I), U, Vv, Ahead);
                                    R (R'First + 2 * I - 2) :=
                                      (if Ahead then U - Sight (Sight'First + I - 1).U else Real (Width));
                                    R (R'First + 2 * I - 1) :=
                                      (if Ahead then Vv - Sight (Sight'First + I - 1).V else Real (Width));
                                 end if;
                              end;
                           end loop;
                        end;
                     end if;
                  end loop;
               end Evaluate;

               procedure Solve is new Robust_Fit (Count, 2 * S, Driver.Conventions.Initial_Damping, Evaluate,
                                                  At_Once => True, To_The_End => True);
               V0 : Real_Array (1 .. Count) := [others => 0.0];
               R0 : Real_Access := new Real_Array (1 .. 2 * S);
            begin
               for J in 1 .. N loop
                  Perp (X.Joints (J).W, Tilt_1 (J), Tilt_2 (J));
                  Side_1 (J) := Tilt_1 (J);
                  Side_2 (J) := Tilt_2 (J);
               end loop;
               for I in 1 .. S loop
                  declare
                     Sg    : Sighting renames Sight (Sight'First + I - 1);
                     Depth : constant Real := Depth_Of (X, Sg.Track);
                  begin
                     Has_Track (I) := Depth > 0.0;
                     if Has_Track (I) then
                        Track_Points (I) := Depth * Ray (X.L, Sg.U0, Sg.V0);
                     end if;
                  end;
               end loop;
               Evaluate (V0, R0.all);
               --  The noise at the start, over the sightings with a point (the others' residuals are nought).
               declare
                  Valid : Real_Access := new Real_Array (1 .. 2 * S);
                  K     : Natural := 0;
               begin
                  for I in 1 .. S loop
                     if Has_Track (I) then
                        Valid (K + 1) := R0 (2 * I - 1);
                        Valid (K + 2) := R0 (2 * I);
                        K := K + 2;
                     end if;
                  end loop;
                  if K > 0 then
                     Solve (V0, Noise_Of (Valid (1 .. K)));
                  end if;
                  Free (Valid);
               end;
               Into.Order := C;
               Into.Joints := Joints_Of (V0);
               Into.L := X.L;
               Into.Report := (others => <>);
               Into.Report.Depths := X.Report.Depths;
               Into.Report.All_Count := S;
               Free (R0);
               Free (Track_Points);
               Free (Has_Track);
            end Quick_Fit;

            --  Order C with the joint at place I taken out and put back at place K.
            function Moved (C : Chain_Order; I, K : Positive) return Chain_Order is
               Out_Of : Chain_Order (1 .. N - 1);
               Next   : Natural := 0;
               Result : Chain_Order (1 .. N);
            begin
               for P in 1 .. N loop
                  if P /= I then
                     Next := Next + 1;
                     Out_Of (Next) := C (P);
                  end if;
               end loop;
               Next := 0;
               for P in 1 .. N loop
                  if P = K then
                     Result (P) := C (I);
                  else
                     Next := Next + 1;
                     Result (P) := Out_Of (Next);
                  end if;
               end loop;
               return Result;
            end Moved;

            --  The order the cheap score finds least: every order scored, in the lexicographic sequence, when there
            --  are no more of them than N steps of a descent by two moves would score ((N - 1) ** 4 orders each);
            --  else descents by single moves, the first from From and the others from orders drawn by the fit's own
            --  generator, until a start ends where the least found ends, and from that least a descent by two moves:
            --  two swaps each of which alone scores worse (the bench's s22, 10-20, 1 5 2 6 3 4 for 5 1 2 6 4 3).
            procedure Global (From_Order : Chain_Order; Best : out Chain_Order; Best_M : out Real; Starts : out Natural)
            is
               Every      : Real := 1.0;
               Steps_Cost : constant Real := Real (N) * Real (N - 1) ** 4;
               Drawn      : Sampler;

               --  From C, to the least order one move from it (Two: or two moves), until none is less.
               procedure Descend (C : in out Chain_Order; M : in out Real; Two : Boolean) is
               begin
                  loop
                     declare
                        Next   : Chain_Order (1 .. N) := C;
                        Next_M : Real := M;
                     begin
                        for I in 1 .. N loop
                           for K in 1 .. N loop
                              if K /= I and then K /= I - 1 then
                                 declare
                                    C1 : constant Chain_Order := Moved (C, I, K);
                                    M1 : constant Real := Cheap (C1);
                                 begin
                                    if M1 < Next_M then
                                       Next_M := M1;
                                       Next := C1;
                                    end if;
                                    if Two then
                                       for I2 in 1 .. N loop
                                          for K2 in 1 .. N loop
                                             if K2 /= I2 and then K2 /= I2 - 1 then
                                                declare
                                                   C2 : constant Chain_Order := Moved (C1, I2, K2);
                                                   M2 : constant Real := Cheap (C2);
                                                begin
                                                   if M2 < Next_M then
                                                      Next_M := M2;
                                                      Next := C2;
                                                   end if;
                                                end;
                                             end if;
                                          end loop;
                                       end loop;
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end loop;
                        exit when Next_M >= M;
                        C := Next;
                        M := Next_M;
                     end;
                  end loop;
               end Descend;
            begin
               Best := From_Order;
               Best_M := Cheap (From_Order);
               Starts := 0;
               for K in 2 .. N loop
                  exit when Every > Steps_Cost;
                  Every := Every * Real (K);
               end loop;
               if Every <= Steps_Cost then
                  declare
                     C : Chain_Order (1 .. N) := [for I in 1 .. N => I];
                  begin
                     loop
                        declare
                           M : constant Real := Cheap (C);
                        begin
                           if M < Best_M then
                              Best_M := M;
                              Best := C;
                           end if;
                        end;
                        declare
                           I : Natural := N - 1;
                        begin
                           while I >= 1 and then C (I) > C (I + 1) loop
                              I := I - 1;
                           end loop;
                           exit when I = 0;
                           declare
                              J  : Positive := N;
                              T  : Positive;
                              Lo : Positive := I + 1;
                              Hi : Positive := N;
                           begin
                              while C (J) < C (I) loop
                                 J := J - 1;
                              end loop;
                              T := C (I);
                              C (I) := C (J);
                              C (J) := T;
                              while Lo < Hi loop
                                 T := C (Lo);
                                 C (Lo) := C (Hi);
                                 C (Hi) := T;
                                 Lo := Lo + 1;
                                 Hi := Hi - 1;
                              end loop;
                           end;
                        end;
                     end loop;
                  end;
                  return;
               end if;
               loop
                  Starts := Starts + 1;
                  declare
                     C : Chain_Order (1 .. N) := From_Order;
                     M : Real;
                  begin
                     if Starts > 1 then
                        declare
                           Picked : Count_Array (1 .. N);
                        begin
                           Pick (Drawn, N, Picked);
                           for P in 1 .. N loop
                              C (P) := Picked (P);
                           end loop;
                        end;
                     end if;
                     M := Cheap (C);
                     Descend (C, M, Two => False);
                     if M < Best_M then
                        Best_M := M;
                        Best := C;
                     elsif Starts > 1 and then C = Best then
                        exit;
                     end if;
                  end;
               end loop;
               Descend (Best, Best_M, Two => True);
            end Global;

            --  Whether X's order explains the keyframes of several joints as the joints explain their own: those
            --  keyframes' median residuals ranked against the keyframes of one joint's (Mann and Whitney's U, the
            --  pairs in which the several's is the larger), not above half the pairs by more than Z of U's spread
            --  when the two come from one distribution. Judged by the medians' own spread, an order that missed some
            --  keyframes by pixels and others not at all spread them so wide that 1.255 px against 0.230 passed (s36
            --  of the bench, 10-20).
            procedure Adequacy (X : Order_Fit; Single_M, Multi_M : out Real; Adequate : out Boolean) is
               Pu, Pv : Real_Access := new Real_Array (1 .. S);
               Ms, Mm : Real_Array (1 .. Frames);
               Ks, Km : Natural := 0;
            begin
               Predict (X, X.Order, Pu.all, Pv.all);
               for F in 1 .. Frames loop
                  if Count_Of (F) > 0 then
                     declare
                        D : Real_Array (1 .. 2 * Count_Of (F));
                        K : Natural := 0;
                     begin
                        for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                           declare
                              I : constant Positive := By_Frame (Q);
                           begin
                              D (K + 1) := (if Pu (I) < Real'Last then abs (Pu (I) - Sight (Sight'First + I - 1).U)
                                            else Real'Last);
                              D (K + 2) := (if Pv (I) < Real'Last then abs (Pv (I) - Sight (Sight'First + I - 1).V)
                                            else Real'Last);
                              K := K + 2;
                           end;
                        end loop;
                        if Several (F) then
                           Km := Km + 1;
                           Mm (Km) := Driver.Stats.Median (D (1 .. K));
                        elsif Own_Joint (F) > 0 then
                           Ks := Ks + 1;
                           Ms (Ks) := Driver.Stats.Median (D (1 .. K));
                        end if;
                     end;
                  end if;
               end loop;
               Free (Pu);
               Free (Pv);
               Single_M := (if Ks > 0 then Driver.Stats.Median (Ms (1 .. Ks)) else 0.0);
               Multi_M := (if Km > 0 then Driver.Stats.Median (Mm (1 .. Km)) else 0.0);
               if Ks = 0 or else Km = 0 then
                  Adequate := True;
                  return;
               end if;
               declare
                  U : Real := 0.0;
               begin
                  for A of Mm (1 .. Km) loop
                     for B of Ms (1 .. Ks) loop
                        U := U + (if A > B then 1.0 elsif A = B then 0.5 else 0.0);
                     end loop;
                  end loop;
                  Adequate := not (U - Real (Ks * Km) / 2.0
                                   > Driver.Conventions.Z * Sqrt (Real (Ks * Km * (Ks + Km + 1)) / 12.0));
               end;
            end Adequacy;

            --  Whether A's places of the sightings (Au, Av) tell it from B's (Bu, Bv) as the better, over the keyframes
            --  of Half (0 every one, 1 the odd, 2 the even): only a keyframe where the two put the sightings apart, by
            --  the median, more than Z of A's residuals' scatter tells one order from the other (elsewhere both
            --  predict what was seen alike, and its sign is the noise's); over those, B's median residual larger than
            --  A's by more than Z of how much A's keyframe medians vary from one keyframe to the next, times the root
            --  of their count. A keyframe neither explains (its median no larger than A's keyframes' typical one by Z
            --  of their spread) is aside: an aligner's answers on another place of the picture (u11, 10-19, eight
            --  keyframes of 268 more wrong than right). One whose points A puts behind the eye tells against A; one
            --  where only B does, for it.
            --  What the test found, for the log to say why an order is not told.
            type Judgement is record
               Smaller : Boolean := False;
               Telling : Natural := 0;     --  keyframes that put the two apart and one of them explains
               Aside   : Natural := 0;     --  that put them apart and neither explains
               Gap     : Real := 0.0;      --  B's medians less A's, summed over the telling keyframes, px
               Needed  : Real := 0.0;      --  what the test asks of that sum, px
               Behind, Beyond : Boolean := False;
            end record;

            function Judge (Au, Av, Bu, Bv : Real_Array; Half : Natural) return Judgement is
               Z       : constant Real := Driver.Conventions.Z;
               Aside   : Natural := 0;
               Medians : Real_Array (1 .. Frames);
               Km      : Natural := 0;
               Spread  : Real := Real'Last;
               Typical : Real := 0.0;
               Noise   : Real := 0.0;
               Sum     : Real := 0.0;
               M       : Natural := 0;
               Behind, Beyond : Boolean := False;
               function In_Half (F : Positive) return Boolean is (Half = 0 or else F mod 2 = Half mod 2);
               function Sizes (Pu, Pv : Real_Array; F : Positive) return Real_Array is
                  D : Real_Array (1 .. 2 * Count_Of (F));
                  K : Natural := 0;
               begin
                  for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                     declare
                        I : constant Positive := By_Frame (Q);
                     begin
                        D (K + 1) := (if Pu (I) >= Real'Last then Real'Last else abs (Pu (I) - Sight (Sight'First + I - 1).U));
                        D (K + 2) := (if Pv (I) >= Real'Last then Real'Last else abs (Pv (I) - Sight (Sight'First + I - 1).V));
                        K := K + 2;
                     end;
                  end loop;
                  return D;
               end Sizes;
               function Explains (V : Real) return Boolean is
                 (V < Real'Last and then (Spread >= Real'Last or else V <= Typical + Z * Spread));
            begin
               declare
                  Finite : Real_Access := new Real_Array (1 .. 2 * S);
                  K      : Natural := 0;
               begin
                  for I in 1 .. S loop
                     if Au (I) < Real'Last then
                        Finite (K + 1) := Au (I) - Sight (Sight'First + I - 1).U;
                        Finite (K + 2) := Av (I) - Sight (Sight'First + I - 1).V;
                        K := K + 2;
                     end if;
                  end loop;
                  Noise := (if K > 1 then Driver.Stats.Robust_Sigma (Finite (1 .. K)) else 0.0);
                  Free (Finite);
               end;
               for F in 1 .. Frames loop
                  if Count_Of (F) > 0 and then In_Half (F) then
                     declare
                        X : constant Real := Driver.Stats.Median (Sizes (Au, Av, F));
                     begin
                        if X < Real'Last then
                           Km := Km + 1;
                           Medians (Km) := X;
                        end if;
                     end;
                  end if;
               end loop;
               if Km > 1 then
                  Spread := Driver.Stats.Robust_Sigma (Medians (1 .. Km));
                  Typical := Driver.Stats.Median (Medians (1 .. Km));
               end if;
               for F in 1 .. Frames loop
                  if Count_Of (F) > 0 and then In_Half (F) then
                     declare
                        Apart : Real_Array (1 .. Count_Of (F));
                     begin
                        for Q in First_Of (F) .. First_Of (F) + Count_Of (F) - 1 loop
                           declare
                              I : constant Positive := By_Frame (Q);
                           begin
                              Apart (Q - First_Of (F) + 1) :=
                                (if Au (I) >= Real'Last and then Bu (I) >= Real'Last then 0.0
                                 elsif Au (I) >= Real'Last or else Bu (I) >= Real'Last then Real'Last
                                 else Sqrt ((Au (I) - Bu (I)) ** 2 + (Av (I) - Bv (I)) ** 2));
                           end;
                        end loop;
                        if Driver.Stats.Median (Apart) > Z * Noise then
                           declare
                              X : constant Real := Driver.Stats.Median (Sizes (Au, Av, F));
                              Y : constant Real := Driver.Stats.Median (Sizes (Bu, Bv, F));
                           begin
                              if X >= Real'Last and then Y >= Real'Last then
                                 null;
                              elsif not Explains (X) and then not Explains (Y) then
                                 Aside := Aside + 1;
                              elsif X >= Real'Last then
                                 Behind := True;
                              elsif Y >= Real'Last then
                                 Beyond := True;
                              else
                                 M := M + 1;
                                 Sum := Sum + (Y - X);
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
               return
                 (Smaller => (if Behind then False
                              elsif Beyond then True
                              else M > 0 and then Spread > 0.0 and then Spread < Real'Last
                                   and then Sum > Z * Spread * Sqrt (Real (M))),
                  Telling => M, Aside => Aside, Gap => Sum,
                  Needed  => (if Spread < Real'Last then Z * Spread * Sqrt (Real (M)) else Real'Last),
                  Behind  => Behind, Beyond => Beyond);
            end Judge;

            function Told_Over (Au, Av, Bu, Bv : Real_Array) return Boolean is
              (Judge (Au, Av, Bu, Bv, 0).Smaller and then Judge (Au, Av, Bu, Bv, 1).Smaller
               and then Judge (Au, Av, Bu, Bv, 2).Smaller);

            --  Why B is not told from A: the first of every keyframe, the odd and the even that does not tell it.
            function Why_Not_Told (Au, Av, Bu, Bv : Real_Array) return String is
            begin
               for Half in 0 .. 2 loop
                  declare
                     J : constant Judgement := Judge (Au, Av, Bu, Bv, Half);
                  begin
                     if not J.Smaller then
                        return (case Half is when 0 => "every keyframe", when 1 => "the odd keyframes",
                                                       when others => "the even keyframes")
                          & (if J.Behind then ": the order kept puts a keyframe's points behind the eye"
                             else ":" & J.Telling'Image & " put the two apart, larger by " & Driver.Log.Image (J.Gap, 2)
                                  & " px against " & Driver.Log.Image (J.Needed, 2) & "," & J.Aside'Image
                                  & " aside (neither explains them)");
                     end if;
                  end;
               end loop;
               return "told";
            end Why_Not_Told;

            Here       : Order_Fit;
            Unordered  : Order_Fit;   --  the joints and lens of the keyframes of one joint each, which no order changes
            Start      : Chain_Order (1 .. N) := [for I in 1 .. N => I];
            Neighbours : constant Positive := (N - 1) ** 2 + 1;   --  the orders one move away, and the global least
            Close      : Order_Table_Access := new Order_Table (1 .. Neighbours);
            Close_M    : Real_Access := new Real_Array (1 .. Neighbours);   --  each one's cheap score
            Close_Fit  : Fit_Table_Access := new Fit_Table (1 .. Neighbours);
            Closes     : Natural := 0;
            Passes     : Natural := 0;
            Adequate   : Boolean := True;
            Single_M, Multi_M : Real := 0.0;
            Fitted_Now : Natural := 0;   --  of the pass's orders weighed, those fitted
         begin
            --  The sightings by keyframe (a counting sort), and the keyframes of several joints.
            for X of Sight loop
               Count_Of (X.Frame) := Count_Of (X.Frame) + 1;
            end loop;
            declare
               Next : Natural := 1;
               Fill : array (1 .. Frames) of Natural := [others => 0];
            begin
               for F in 1 .. Frames loop
                  First_Of (F) := Next;
                  Next := Next + Count_Of (F);
               end loop;
               for I in 1 .. S loop
                  declare
                     F : constant Positive := Sight (Sight'First + I - 1).Frame;
                  begin
                     By_Frame (First_Of (F) + Fill (F)) := I;
                     Fill (F) := Fill (F) + 1;
                  end;
               end loop;
               for F in 1 .. Frames loop
                  if Several (F) and then Count_Of (F) > 0 then
                     Multi_Count := Multi_Count + 1;
                  end if;
               end loop;
               Multi_Frames := new Count_Array (1 .. Multi_Count);
               Multi_Count := 0;
               for F in 1 .. Frames loop
                  if Several (F) and then Count_Of (F) > 0 then
                     Multi_Count := Multi_Count + 1;
                     Multi_Frames (Multi_Count) := F;
                  end if;
               end loop;
               declare
                  Multi_Sightings : Natural := 0;
               begin
                  for K in 1 .. Multi_Count loop
                     Multi_Sightings := Multi_Sightings + Count_Of (Multi_Frames (K));
                  end loop;
                  Residuals := new Real_Array (1 .. Natural'Max (1, 2 * Multi_Sightings));
               end;
            end;
            --  The order to begin from: the one kept before, or the joints' own; fitted in itself.
            if From.Valid and then From.N = N then
               Start := From.Chain;
            end if;
            --  Every order is fitted from the joints and lens of the keyframes of one joint each (Unordered), which
            --  no order changes: from the joints of the order kept, fitted to it, the true order of the bench's s36
            --  (10-20) came to 1.3 px on the keyframes of several joints, its own fit from those of one joint's to 0.24.
            Unordered.Order := Start;
            Unordered.Joints := Joints;
            Unordered.L := Lens0;
            Unordered.Report.Depths := Alone_Depths;
            Quick_Fit (Start, Unordered, Here);
            Fits_Made := Fits_Made + 1;
            loop
               Passes := Passes + 1;
               declare
                  Mo, Gm : Real;
                  Co     : Natural;
                  G      : Chain_Order (1 .. N);
                  Starts : Natural;
                  Best   : Natural := 0;
                  Best_M : Real;   --  the least median residual of the keyframes of several joints, each order in its own fit
               begin
                  Prepare (Here);
                  Mo := Cheap (Here.Order);
                  Co := Cheap_Count;
                  Best_M := Mo;
                  Adequacy (Here, Single_M, Multi_M, Adequate);
                  Global (Here.Order, G, Gm, Starts);
                  Closes := 0;
                  if G /= Here.Order then
                     Closes := 1;
                     Close (1) := G;
                     Close_M (1) := Gm;
                  end if;
                  for I in 1 .. N loop
                     for K in 1 .. N loop
                        if K /= I and then K /= I - 1 then
                           declare
                              C  : constant Chain_Order := Moved (Here.Order, I, K);
                              Mc : constant Real := Cheap (C);
                              Cc : constant Natural := Cheap_Count;
                           begin
                              if C /= G
                                and then (not Adequate
                                          or else (Mc < Real'Last and then Co > 0 and then Cc > 0
                                                   and then Mc - Mo <= Driver.Conventions.Z
                                                                      * Sqrt (Median_Error (Mo, Co) ** 2
                                                                              + Median_Error (Mc, Cc) ** 2)))
                              then
                                 Closes := Closes + 1;
                                 Close (Closes) := C;
                                 Close_M (Closes) := Mc;
                              end if;
                           end;
                        end if;
                     end loop;
                  end loop;
                  --  Fitted in the order of their cheap scores, the least first; the first whose fit leaves every
                  --  sighting less than the order kept takes its place at once. Each of them fitted first took a
                  --  pass from an order the keyframes of several joints showed wrong (every move of one, 25 to 64 fits
                  --  of 2 to 3 s), where the least of the cheap score was the order to go on to.
                  for K in 2 .. Closes loop
                     declare
                        C : constant Chain_Order (1 .. N) := Close (K);
                        M : constant Real := Close_M (K);
                        P : Natural := K - 1;
                     begin
                        while P >= 1 and then Close_M (P) > M loop
                           Close (P + 1) := Close (P);
                           Close_M (P + 1) := Close_M (P);
                           P := P - 1;
                        end loop;
                        Close (P + 1) := C;
                        Close_M (P + 1) := M;
                     end;
                  end loop;
                  Fitted_Now := 0;
                  --  An order's own fit is judged where the orders differ: the median residual of the sightings of the
                  --  keyframes of several joints (Cheap with its own joints, lens and depths). Every order explains
                  --  those of one joint alike, and they are most of the sightings: by the median over every sighting
                  --  the search walked from 0.473 px to 0.469 through orders that missed the keyframes of several
                  --  joints by 1.4 to 4.5 px (the bench's s36, 10-20), one move from the truth at 0.24.
                  --  As many fitted at once as the workers and the caller are (each fit its own), then judged in that
                  --  order: the first that leaves less is the one a fit at a time would have taken. One at a time, a
                  --  nine-joint arm's rounds took 10 to 20 s each of the bench's two minutes (s02 of the humanoid,
                  --  10-20: 10 to 28 fits a round).
                  declare
                     First : Positive := 1;
                  begin
                     while First <= Closes and then Best = 0 loop
                        declare
                           Last : constant Positive := Natural'Min (Closes, First + Driver.Workers.Size - 1);
                           procedure One (K : Positive) is
                           begin
                              Quick_Fit (Close (K), Unordered, Close_Fit (K));
                           end One;
                           procedure Every_One is new Driver.Parallel_For (One);
                        begin
                           Every_One (First, Last);
                           Fits_Made := Fits_Made + (Last - First + 1);
                           for K in First .. Last loop
                              Fitted_Now := K;
                              if Close_Fit (K).Report.All_Count > 0 then
                                 Prepare (Close_Fit (K));
                                 declare
                                    Own_M : constant Real := Cheap (Close (K));
                                    Own_C : constant Natural := Cheap_Count;
                                 begin
                                    --  Taken only when it leaves the sightings less by Z of the two medians' errors: an
                                    --  order less by a hair is as good as the one kept, and the search walked on such hairs
                                    --  from the true order of an arm whose keyframes shared an error to one 0.001 px less
                                    --  (robot.kinematics.shared, 10-20: 0.236 against 0.237 px, neither told from the other).
                                    if Own_C > 0 and then Co > 0
                                      and then Own_M + Driver.Conventions.Z
                                                       * Sqrt (Median_Error (Own_M, Own_C) ** 2 + Median_Error (Mo, Co) ** 2)
                                               < Best_M
                                    then
                                       Best_M := Own_M;
                                       Best := K;
                                       exit;
                                    end if;
                                 end;
                              end if;
                           end loop;
                           First := Last + 1;
                        end;
                     end loop;
                  end;
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Arm & ": the order" & Image (Here.Order)
                                   & " (focal " & Driver.Log.Image (Here.L.Fx, 1) & " px)"
                                   & " leaves the sightings of keyframes of several joints " & Driver.Log.Image (Mo, 3)
                                   & " px, those keyframes " & Driver.Log.Image (Multi_M, 3)
                                   & " px (the median of their medians) against those of one "
                                   & Driver.Log.Image (Single_M, 3) & (if Adequate then "" else ", NOT as well")
                                   & "; the cheap score's least order" & Image (G) & " ("
                                   & Driver.Log.Image (Gm, 3) & " px against " & Driver.Log.Image (Mo, 3) & ","
                                   & Cheaps'Image & " orders scored"
                                   & (if Starts > 0 then "," & Starts'Image & " starts" else "") & ");"
                                   & Fitted_Now'Image & " of" & Closes'Image & " orders weighed fitted in themselves"
                                   & (if Best > 0 then ", the least " & Image (Close (Best)) & " at "
                                      & Driver.Log.Image (Best_M, 3) & " px"
                                      else ", none less"));
                  exit when Best = 0;
                  Here := Close_Fit (Best);
               end;
            end loop;
            --  The verdict over the orders weighed in the last pass.
            declare
               Hu, Hv : Real_Access := new Real_Array (1 .. S);
               Cu, Cv : Real_Access := new Real_Array (1 .. S);
               Untold_Of   : array (1 .. Neighbours) of Boolean := [others => False];
               Untold_Here : Natural := 0;
               Same_Arm    : Natural := 0;
               Scatter     : Real := Real'Last;
               Followable  : Real := 0.0;
               Reach       : Real_Array (1 .. N) := [others => 0.0];
               Said        : Ada.Strings.Unbounded.Unbounded_String;
               --  The points, where Here's depths put them along their reference lines of sight.
               Tracks      : Natural := 0;

               --  A move of two joints together: either way each, by a share of its widest turn.
               type Move is record
                  I, J   : Positive := 1;
                  Si, Sj : Real := 1.0;
                  Share  : Real := 1.0;
               end record;
               function Change_Of (Mv : Move) return Real_Array is
                  D : Real_Array (1 .. N) := [others => 0.0];
               begin
                  D (Mv.I) := Mv.Si * Mv.Share * Reach (Mv.I);
                  D (Mv.J) := Mv.Sj * Mv.Share * Reach (Mv.J);
                  return D;
               end Change_Of;

               function In_Picture (U, V : Real) return Boolean is
                 (U >= 0.0 and then U < Real (Width) and then V >= 0.0 and then V < Real (Height));

               --  After move Mv, how far apart Here's joints in its own order and in order C put the points both
               --  put in the picture (the median, in pixels) and how many; and the median, over the points Here's
               --  order puts in the picture, of how far C puts each from there (what the keyframe's window must reach).
               procedure Separation (C : Chain_Order; Mv : Move; Median, Reached : out Real; Count : out Natural) is
                  Back_1 : constant Rigid := Inverse (Eye_At (Here.Joints, Here.Order, Change_Of (Mv)));
                  Back_2 : constant Rigid := Inverse (Eye_At (Here.Joints, C, Change_Of (Mv)));
                  Dist : Real_Access := new Real_Array (1 .. Natural'Max (1, Tracks));
                  Far  : Real_Access := new Real_Array (1 .. Natural'Max (1, Tracks));
                  Seen : array (1 .. Tracks) of Boolean := [others => False];
                  Kf   : Natural := 0;
               begin
                  Count := 0;
                  for X of Sight loop
                     if not Seen (X.Track) and then Depth_Of (Here, X.Track) > 0.0 then
                        Seen (X.Track) := True;
                        declare
                           P              : constant Vec3 := Depth_Of (Here, X.Track) * Ray (Here.L, X.U0, X.V0);
                           U1, V1, U2, V2 : Real;
                           A1, A2         : Boolean;
                        begin
                           Project (Here.L, Back_1 * P, U1, V1, A1);
                           Project (Here.L, Back_2 * P, U2, V2, A2);
                           if A1 and then In_Picture (U1, V1) then
                              Kf := Kf + 1;
                              Far (Kf) := (if A2 then Sqrt ((U1 - U2) ** 2 + (V1 - V2) ** 2) else Real'Last);
                              if A2 and then In_Picture (U2, V2) then
                                 Count := Count + 1;
                                 Dist (Count) := Sqrt ((U1 - U2) ** 2 + (V1 - V2) ** 2);
                              end if;
                           end if;
                        end;
                     end if;
                  end loop;
                  Median := (if Count > 0 then Driver.Stats.Median (Dist (1 .. Count)) else 0.0);
                  Reached := (if Kf > 0 then Driver.Stats.Median (Far (1 .. Kf)) else Real'Last);
                  Free (Dist);
                  Free (Far);
               end Separation;

               --  Per pair of joints (Earlier, Later as Here has them): the untold orders it tells most, and the turns
               --  of its two joints that tell one of them most.
               Tells    : array (1 .. N, 1 .. N) of Natural := [others => [others => 0]];
               Turn_I, Turn_J : array (1 .. N, 1 .. N) of Real := [others => [others => 0.0]];
               Power_Of : array (1 .. N, 1 .. N) of Real := [others => [others => 0.0]];
            begin
               for X of Sight loop
                  Tracks := Natural'Max (Tracks, X.Track);
               end loop;
               Predict (Here, Here.Order, Hu.all, Hv.all);
               declare
                  Finite : Real_Access := new Real_Array (1 .. 2 * S);
                  K      : Natural := 0;
               begin
                  for I in 1 .. S loop
                     if Hu (I) < Real'Last then
                        Finite (K + 1) := Hu (I) - Sight (Sight'First + I - 1).U;
                        Finite (K + 2) := Hv (I) - Sight (Sight'First + I - 1).V;
                        K := K + 2;
                     end if;
                  end loop;
                  if K > 1 then
                     Scatter := Driver.Stats.Robust_Sigma (Finite (1 .. K));
                  end if;
                  Free (Finite);
               end;
               --  How far an order's places of the points may lie from Here's for a keyframe of a turn to be followed:
               --  the window that reaches there, the fit's own scatter beside it, no wider than one a patch carries
               --  where the pictures are finest (Driver.Alignment.Finest_Radius). A turn whose places lie farther
               --  apart is searched wide and tells nothing: u6 (10-19) turned c2 with c6 where they told its untold
               --  orders apart most, 140 px apart, and its four keyframes found none of their points.
               if Scatter < Real'Last then
                  Followable := Sqrt (Real'Max (0.0, Driver.Alignment.Finest_Radius (Width, Height) ** 2
                                                     - Driver.Alignment.Window_Radius (Scatter) ** 2));
               end if;
               for F in 1 .. Frames loop
                  for J in 1 .. N loop
                     Reach (J) := Real'Max (Reach (J), abs Change (F, J));
                  end loop;
               end loop;
               for K in 1 .. Closes loop
                  Predict (Close_Fit (K), Close (K), Cu.all, Cv.all);
                  if not Told_Over (Hu.all, Hv.all, Cu.all, Cv.all) then
                     --  Untold: the most telling move of the pairs it swaps, by the separation's size over the error
                     --  of its median, among the moves the aligner follows; none that tells it by Z, the same arm.
                     declare
                        Pos       : array (1 .. N) of Natural := [others => 0];
                        Best_Mv   : Move;
                        Best_Pow  : Real := 0.0;
                     begin
                        for P in 1 .. N loop
                           Pos (Close (K) (P)) := P;
                        end loop;
                        for Ia in 1 .. N loop
                           for Ib in Ia + 1 .. N loop
                              declare
                                 X : constant Positive := Here.Order (Ia);
                                 Y : constant Positive := Here.Order (Ib);
                                 Share : Real := 1.0;
                              begin
                                 if Pos (Y) < Pos (X) then
                                    loop
                                       for Si of Real_Array'[1.0, -1.0] loop
                                          for Sj of Real_Array'[1.0, -1.0] loop
                                             declare
                                                Mv : constant Move := (I => X, J => Y, Si => Si, Sj => Sj,
                                                                       Share => Share);
                                                D, Far : Real;
                                                Cnt    : Natural;
                                             begin
                                                Separation (Close (K), Mv, D, Far, Cnt);
                                                if Cnt > 0 and then Far <= Followable then
                                                   declare
                                                      Power : constant Real :=
                                                        D / Driver.Stats.Median_Sigma (Scatter, Cnt);
                                                   begin
                                                      if Power > Best_Pow then
                                                         Best_Pow := Power;
                                                         Best_Mv := Mv;
                                                      end if;
                                                   end;
                                                end if;
                                             end;
                                          end loop;
                                       end loop;
                                       Share := Share / 2.0;
                                       exit when Visible (Visible'First + X - 1) <= 0.0
                                         or else Visible (Visible'First + Y - 1) <= 0.0
                                         or else Share * Reach (X) < Visible (Visible'First + X - 1)
                                         or else Share * Reach (Y) < Visible (Visible'First + Y - 1);
                                    end loop;
                                 end if;
                              end;
                           end loop;
                        end loop;
                        if Best_Pow > Driver.Conventions.Z then
                           Untold_Of (K) := True;
                           Untold_Here := Untold_Here + 1;
                           Tells (Best_Mv.I, Best_Mv.J) := Tells (Best_Mv.I, Best_Mv.J) + 1;
                           if Best_Pow > Power_Of (Best_Mv.I, Best_Mv.J) then
                              Power_Of (Best_Mv.I, Best_Mv.J) := Best_Pow;
                              Turn_I (Best_Mv.I, Best_Mv.J) := Best_Mv.Si * Best_Mv.Share * Reach (Best_Mv.I);
                              Turn_J (Best_Mv.I, Best_Mv.J) := Best_Mv.Sj * Best_Mv.Share * Reach (Best_Mv.J);
                           end if;
                           Ada.Strings.Unbounded.Append
                             (Said, " [" & Image (Close (K)) & ", not told on "
                                    & Why_Not_Told (Hu.all, Hv.all, Cu.all, Cv.all)
                                    & "; c" & Ada.Strings.Fixed.Trim (Best_Mv.I'Image,
                                                                                           Ada.Strings.Left)
                                    & " with c" & Ada.Strings.Fixed.Trim (Best_Mv.J'Image, Ada.Strings.Left)
                                    & " tells it by " & Driver.Log.Image (Best_Pow, 1) & " of its error]");
                        else
                           Same_Arm := Same_Arm + 1;
                           Ada.Strings.Unbounded.Append (Said, " (" & Image (Close (K)) & ": the same arm, no turn"
                                                         & " followed tells it by Z)");
                        end if;
                     end;
                  end if;
               end loop;
               Order_Is_Told := Adequate and then Untold_Here = 0;
               Untold.Clear;
               Untold_Turns.Clear;
               Untold_Orders.Clear;
               for K in 1 .. Closes loop
                  if Untold_Of (K) then
                     for P of Close (K) loop
                        Untold_Orders.Append (P);
                     end loop;
                  end if;
               end loop;
               --  The pairs Boot turns: the pair that tells the most untold orders first, each by its best turns.
               loop
                  declare
                     Bi, Bj : Natural := 0;
                  begin
                     for I in 1 .. N loop
                        for J in 1 .. N loop
                           if Tells (I, J) > 0 and then (Bi = 0 or else Tells (I, J) > Tells (Bi, Bj)) then
                              Bi := I;
                              Bj := J;
                           end if;
                        end loop;
                     end loop;
                     exit when Bi = 0;
                     Untold.Append (Joint_Pair'(Earlier => Bi, Later => Bj));
                     Untold_Turns.Append (Turn_I (Bi, Bj));
                     Untold_Turns.Append (Turn_J (Bi, Bj));
                     Tells (Bi, Bj) := 0;
                  end;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "kinematics: the order of the joints along " & Arm & ","
                                & Image (Here.Order) & ", is " & (if Order_Is_Told then "told" else "NOT told")
                                & ": " & Closes'Image & " orders weighed," & Untold_Here'Image & " untold,"
                                & Same_Arm'Image & " the same arm"
                                & (if Adequate then "" else "; it explains the keyframes of several joints worse")
                                & "; passes" & Passes'Image & "," & Fits_Made'Image & " fits"
                                & Ada.Strings.Unbounded.To_String (Said));
               Free (Hu);
               Free (Hv);
               Free (Cu);
               Free (Cv);
            end;
            Order := Here.Order;
            Joints := Here.Joints;
            Lens0 := Here.L;
            Free (By_Frame);
            Free (Multi_Frames);
            Free (Steps);
            Free (Points);
            Free (Has_Point);
            Free (Residuals);
            Free (Close);
            Free (Close_M);
            Free (Close_Fit);
         end Search_Order;
      end if;
      Normalize (Joints);

      --  Then every joint at once, on the sightings that fit. Parameters: log Fx, log Fy, Cx, Cy, K1, K2, then per
      --  joint the axis's tilt in its tangent plane (2), its point (3) and its reading scale (1). The lens stands as the
      --  stages before left it: the Sampson residual is no pixel of the picture but of the lens it is taken through, and
      --  a lens that sends every pixel along nearly one line of sight (K1 7.8e16, the focal lengths nought) meets every
      --  epipolar constraint at once; the fit walked into it and the tracks' fit (stage 5), whose residuals are pixels
      --  of the picture, began from it and never left (s30, servo seed 7, replayed and its input fitted alone, 10-17:
      --  3153 sightings none of which lay 2 px off the true motion; with the lens standing here the tracks' fit ended
      --  at 133.9 x 135.0 px against the truth's 135.3 x 135.9). The lens is the tracks' fit's to fit.
      Together (Order, Joints, Lens0, Report.Sigma_Px, Report.Used, Report.Median_Px);
      L := Lens0;

      --  The focal length's profile is walked by a fit that searched it, and by one begun from an earlier estimate's
      --  once the sightings have doubled since the profile was last walked: until then its basin stands, and walked
      --  at every estimate it cost the boot a hundred beats a step, fourteen steps an estimate, held still (f16h on
      --  10-15, two runs and the self tests sharing the machine).
      Finish (Profiled => not From.Valid or else Sightings'Length >= 2 * From.Profiled_Over);
   end Fit;

   function Depth_Known (Depth, Sigma : Real) return Boolean is
     (Depth > 0.0 and then Sigma < Real'Last and then Driver.Uncertain.Significant (1.0, Sigma));

   --  The root mean square of the eye positions over the keyframes (one a row of Changes) the joints make.
   function Unit_Of (Jx : Joint_Array; Chain : Chain_Order; Changes : Driver.Numerics.Arrays.Real_Matrix) return Real is
      N   : constant Natural := Jx'Length;
      Sum : Real := 0.0;
   begin
      for F in Changes'Range (1) loop
         declare
            D : Real_Array (1 .. N);
         begin
            for J in 1 .. N loop
               D (J) := Changes (F, Changes'First (2) + J - 1);
            end loop;
            declare
               T : constant Vec3 := Eye_At (Jx, Chain, D).Translation;
            begin
               Sum := Sum + Dot (T, T);
            end;
         end;
      end loop;
      return Sqrt (Sum / Real (Natural'Max (1, Changes'Length (1))));
   end Unit_Of;

   function Unit_Sigma
     (Joints     : Joint_Array;
      Chain      : Chain_Order;
      Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Covariance : Real_Lists.Vector) return Real
   is
      N       : constant Natural := Joints'Length;
      Reduced : constant Natural := Terms (N);
      function V (P, Q : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Reduced + Q - 1));
      function Unit_Of (Jx : Joint_Array) return Real is (Unit_Of (Jx, Chain, Changes));
      D : array (1 .. N, Joint_Term) of Real := [others => [others => 0.0]];
      Variance : Real := 0.0;
      Here     : constant Real := Unit_Of (Joints);
   begin
      if Natural (Covariance.Length) /= Reduced * Reduced or else Here <= 0.0 then
         return Real'Last;
      end if;
      for J in 1 .. N loop
         for T in Joint_Term loop
            declare
               P     : constant Positive := Term_Of (J, T);
               Sigma : constant Real := (if V (P, P) > 0.0 then Sqrt (V (P, P)) else 0.0);
            begin
               if Sigma > 0.0 then
                  declare
                     function Moved (H : Real) return Real is
                        Jx     : Joint_Array := Joints;
                        A      : Joint renames Jx (Jx'First + J - 1);
                        E1, E2 : Vec3;
                     begin
                        Perp (A.W, E1, E2);
                        case T is
                           when Tilt_1  => A.W := Unit (A.W + H * E1);
                           when Tilt_2  => A.W := Unit (A.W + H * E2);
                           when Point_1 => A.P := A.P + H * E1;
                           when Point_2 => A.P := A.P + H * E2;
                           when Scale   => A.C := A.C + H;
                        end case;
                        return Unit_Of (Jx);
                     end Moved;
                  begin
                     D (J, T) := (Moved (Sigma) - Moved (-Sigma)) / (Sigma + Sigma);
                  end;
               end if;
            end;
         end loop;
      end loop;
      for J in 1 .. N loop
         for T in Joint_Term loop
            for K in 1 .. N loop
               for U in Joint_Term loop
                  Variance := Variance + D (J, T) * V (Term_Of (J, T), Term_Of (K, U)) * D (K, U);
               end loop;
            end loop;
         end loop;
      end loop;
      return Sqrt (Real'Max (0.0, Variance)) / Here;
   end Unit_Sigma;

   function Place_Covariance_In_Unit
     (Joints       : Joint_Array;
      Chain        : Chain_Order;
      Change       : Real_Array;
      Unit_Changes : Driver.Numerics.Arrays.Real_Matrix;
      Covariance   : Real_Lists.Vector) return Mat3
   is
      N       : constant Natural := Joints'Length;
      Reduced : constant Natural := Terms (N);
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      function V (P, Q : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Reduced + Q - 1));
      Here    : constant Real := Unit_Of (Joints, Chain, Unit_Changes);
      D       : array (1 .. N, Joint_Term) of Vec3 := [others => [others => [0.0, 0.0, 0.0]]];
      Place   : Mat3 := [others => [others => 0.0]];
   begin
      if Natural (Covariance.Length) /= Reduced * Reduced or else Here <= 0.0 then
         return Unknown;
      end if;
      for J in 1 .. N loop
         for T in Joint_Term loop
            declare
               P     : constant Positive := Term_Of (J, T);
               Sigma : constant Real := (if V (P, P) > 0.0 then Sqrt (V (P, P)) else 0.0);
            begin
               if Sigma > 0.0 then
                  declare
                     --  The place the joints moved by H make, in the unit they make.
                     function Moved (H : Real) return Vec3 is
                        Jx     : Joint_Array := Joints;
                        A      : Joint renames Jx (Jx'First + J - 1);
                        E1, E2 : Vec3;
                     begin
                        Perp (A.W, E1, E2);
                        case T is
                           when Tilt_1  => A.W := Unit (A.W + H * E1);
                           when Tilt_2  => A.W := Unit (A.W + H * E2);
                           when Point_1 => A.P := A.P + H * E1;
                           when Point_2 => A.P := A.P + H * E2;
                           when Scale   => A.C := A.C + H;
                        end case;
                        declare
                           Unit_There : constant Real := Unit_Of (Jx, Chain, Unit_Changes);
                        begin
                           return (if Unit_There > 0.0 then (Here / Unit_There) * Eye_At (Jx, Chain, Change).Translation
                                   else Eye_At (Jx, Chain, Change).Translation);
                        end;
                     end Moved;
                  begin
                     D (J, T) := (1.0 / (Sigma + Sigma)) * (Moved (Sigma) - Moved (-Sigma));
                  end;
               end if;
            end;
         end loop;
      end loop;
      for J in 1 .. N loop
         for T in Joint_Term loop
            for K in 1 .. N loop
               for U in Joint_Term loop
                  Place := Place + V (Term_Of (J, T), Term_Of (K, U)) * Driver.Numerics.Outer (D (J, T), D (K, U));
               end loop;
            end loop;
         end loop;
      end loop;
      return Place;
   end Place_Covariance_In_Unit;

   procedure Move_Term (J : in out Joint_Array; L : in out Lens; Term : Positive; By : Real) is
   begin
      if Term <= Lens_Terms then
         case Lens_Term'Val (Term - 1) is
            when Log_Fx   => L.Fx := L.Fx * Exp (By);
            when Log_Fy   => L.Fy := L.Fy * Exp (By);
            when Centre_X => L.Cx := L.Cx + By;
            when Centre_Y => L.Cy := L.Cy + By;
            when Radial_1 => L.K1 := L.K1 + By;
            when Radial_2 => L.K2 := L.K2 + By;
         end case;
      else
         declare
            A      : Joint renames J (J'First + (Term - Lens_Terms - 1) / Joint_Terms);
            E1, E2 : Vec3;
         begin
            Perp (A.W, E1, E2);
            case Joint_Term'Val ((Term - Lens_Terms - 1) mod Joint_Terms) is
               when Tilt_1  => A.W := Unit (A.W + By * E1);
               when Tilt_2  => A.W := Unit (A.W + By * E2);
               when Point_1 => A.P := A.P + By * E1;
               when Point_2 => A.P := A.P + By * E2;
               when Scale   => A.C := A.C + By;
            end case;
         end;
      end if;
   end Move_Term;

   procedure Pose_Covariance
     (Joints      : Joint_Array;
      Chain       : Chain_Order;
      Change      : Real_Array;
      Covariance  : Real_Lists.Vector;
      Turn, Place : out Mat3)
   is
      N       : constant Natural := Joints'Length;
      Reduced : constant Natural := Terms (N);
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      function V (P, Q : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Reduced + Q - 1));
      --  How the eye's turn and place move per unit of each joint term.
      type Derivative is record
         Turn, Place : Vec3 := [0.0, 0.0, 0.0];
      end record;
      D : array (1 .. N, Joint_Term) of Derivative;
   begin
      Turn := Unknown;
      Place := Unknown;
      if Natural (Covariance.Length) /= Reduced * Reduced then
         return;
      end if;
      for J in 1 .. N loop
         for T in Joint_Term loop
            declare
               P     : constant Positive := Term_Of (J, T);
               Sigma : constant Real := (if V (P, P) > 0.0 then Sqrt (V (P, P)) else 0.0);
            begin
               if Sigma > 0.0 then
                  declare
                     function Moved (H : Real) return Rigid is
                        Jx : Joint_Array := Joints;
                        Lx : Lens;
                     begin
                        Move_Term (Jx, Lx, P, H);
                        return Eye_At (Jx, Chain, Change);
                     end Moved;
                     Up    : constant Rigid := Moved (Sigma);
                     Down  : constant Rigid := Moved (-Sigma);
                     Width : constant Real := Sigma + Sigma;
                  begin
                     --  Central differences, the turn taken in the reference frame.
                     D (J, T).Turn := (1.0 / Width) * Driver.Numerics.Log (Up.Rotation * Transpose (Down.Rotation));
                     D (J, T).Place := (1.0 / Width) * (Up.Translation - Down.Translation);
                  end;
               end if;
            end;
         end loop;
      end loop;
      Turn := [others => [others => 0.0]];
      Place := [others => [others => 0.0]];
      for J in 1 .. N loop
         for T in Joint_Term loop
            for K in 1 .. N loop
               for U in Joint_Term loop
                  declare
                     C : constant Real := V (Term_Of (J, T), Term_Of (K, U));
                  begin
                     Turn := Turn + C * Driver.Numerics.Outer (D (J, T).Turn, D (K, U).Turn);
                     Place := Place + C * Driver.Numerics.Outer (D (J, T).Place, D (K, U).Place);
                  end;
               end loop;
            end loop;
         end loop;
      end loop;
   end Pose_Covariance;

   function Step_Covariance
     (Joints     : Joint_Array;
      Chain      : Chain_Order;
      From, To   : Real_Array;
      Covariance : Real_Lists.Vector) return Mat3
   is
      N       : constant Natural := Joints'Length;
      Reduced : constant Natural := Terms (N);
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      function V (P, Q : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Reduced + Q - 1));
      --  How the displacement moves per unit of each joint term.
      D : array (1 .. N, Joint_Term) of Vec3 := [others => [others => [0.0, 0.0, 0.0]]];
      Result : Mat3 := [others => [others => 0.0]];
   begin
      if Natural (Covariance.Length) /= Reduced * Reduced then
         return Unknown;
      end if;
      for J in 1 .. N loop
         for T in Joint_Term loop
            declare
               P     : constant Positive := Term_Of (J, T);
               Sigma : constant Real := (if V (P, P) > 0.0 then Sqrt (V (P, P)) else 0.0);
            begin
               if Sigma > 0.0 then
                  declare
                     function Moved (H : Real) return Vec3 is
                        Jx     : Joint_Array := Joints;
                        A      : Joint renames Jx (Jx'First + J - 1);
                        E1, E2 : Vec3;
                     begin
                        Perp (A.W, E1, E2);
                        case T is
                           when Tilt_1  => A.W := Unit (A.W + H * E1);
                           when Tilt_2  => A.W := Unit (A.W + H * E2);
                           when Point_1 => A.P := A.P + H * E1;
                           when Point_2 => A.P := A.P + H * E2;
                           when Scale   => A.C := A.C + H;
                        end case;
                        return Eye_At (Jx, Chain, To).Translation - Eye_At (Jx, Chain, From).Translation;
                     end Moved;
                     Width : constant Real := Sigma + Sigma;
                  begin
                     --  Central differences.
                     D (J, T) := (1.0 / Width) * (Moved (Sigma) - Moved (-Sigma));
                  end;
               end if;
            end;
         end loop;
      end loop;
      for J in 1 .. N loop
         for T in Joint_Term loop
            for K in 1 .. N loop
               for U in Joint_Term loop
                  Result := Result + V (Term_Of (J, T), Term_Of (K, U)) * Driver.Numerics.Outer (D (J, T), D (K, U));
               end loop;
            end loop;
         end loop;
      end loop;
      return Result;
   end Step_Covariance;

   ---------------------------------------------------------------------------
   --  Consensus

   function Consensus_Samples (Minimal : Positive; Fraction : Real) return Positive is
      Missed : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      --  The chance that one sample is all of agreeing points.
      All_In : constant Real := Fraction ** Minimal;
      N      : Real;
   begin
      if All_In >= 1.0 then
         return 1;
      end if;
      --  log (1 - p) is -p to working precision for p below the root of the
      --  float's resolution.
      N := Real'Ceiling (Ln (Missed) / (if All_In < Sqrt (Real'Model_Epsilon) then -All_In else Ln (1.0 - All_In)));
      return (if N >= Real (Positive'Last) then Positive'Last else Positive (Real'Max (1.0, N)));
   end Consensus_Samples;

   --  The generator steps by Park and Miller's minimal standard.
   function Draw (S : in out Sampler; N : Positive) return Positive is
   begin
      S.State := (S.State * 48_271) mod 2_147_483_647;
      return Positive (1 + S.State mod Long_Long_Integer (N));
   end Draw;

   --  Into'Length distinct numbers of 1 .. N.
   procedure Pick (S : in out Sampler; N : Positive; Into : out Count_Array) is
   begin
      Into := [others => 0];
      for K in Into'Range loop
         loop
            Into (K) := Draw (S, N);
            exit when (for all J in Into'First .. K - 1 => Into (J) /= Into (K));
         end loop;
      end loop;
   end Pick;

   ---------------------------------------------------------------------------
   --  The dominant plane

   function Plane_Normal (P : Sight_Plane) return Vec3 is
     (if P.Found then (-1.0 / Sqrt (P.A * P.A)) * P.A else [0.0, 0.0, -1.0]);

   function Plane_Offset (P : Sight_Plane) return Real is
     (if P.Found then -1.0 / Sqrt (P.A * P.A) else 0.0);

   function Plane_Offset_Sigma (P : Sight_Plane) return Real is
      Length : constant Real := Sqrt (P.A * P.A);
   begin
      if not P.Found then
         return Real'Last;
      end if;
      declare
         N : constant Vec3 := (1.0 / Length) * P.A;
      begin
         return Sqrt (Real'Max (0.0, N * (P.Covariance * N))) / Length ** 2;
      end;
   end Plane_Offset_Sigma;

   function Plane_Tilt_Sigma (P : Sight_Plane) return Real is
      Length : constant Real := Sqrt (P.A * P.A);
   begin
      if not P.Found then
         return Real'Last;
      end if;
      declare
         N  : constant Vec3 := (1.0 / Length) * P.A;
         Pr : constant Mat3 := Identity3 - Outer (N, N);
         C  : constant Mat3 := (1.0 / Length ** 2) * (Pr * P.Covariance * Pr);
         Values  : Vec3;
         Vectors : Mat3;
      begin
         Symmetric_Eigensystem (C, Values, Vectors);
         return Sqrt (Real'Max (0.0, Real'Max (Values (1), Real'Max (Values (2), Values (3)))));
      end;
   end Plane_Tilt_Sigma;

   procedure Plane_Axes (P : Sight_Plane; Normal, E1, E2 : out Vec3) is
   begin
      Normal := Plane_Normal (P);
      Perp (Normal, E1, E2);
   end Plane_Axes;

   --  A point's inverse depth and its sigma.
   function Inverse_Depth (P : Sight_Point) return Real is (1.0 / P.Depth);
   function Inverse_Sigma (P : Sight_Point) return Real is (P.Sigma / P.Depth);

   function Usable (P : Sight_Point) return Boolean is
     (P.Depth > 0.0 and then P.Sigma > 0.0 and then P.Sigma < Real'Last);

   --  The residual of a point's inverse depth from the plane A, in units of
   --  its sigma.
   function Standard_Residual (A : Vec3; P : Sight_Point) return Real is
     ((Inverse_Depth (P) - A * P.H) / Inverse_Sigma (P));

   procedure Refit_Plane (Points : Sight_Point_Array; On : Flag_Array; Plane : out Sight_Plane) is
      M  : Mat3 := [others => [others => 0.0]];
      B  : Vec3 := [0.0, 0.0, 0.0];
      Count : Natural := 0;
   begin
      Plane := (others => <>);
      for I in Points'Range loop
         if On (I) and then Usable (Points (I)) then
            declare
               W : constant Real := 1.0 / Inverse_Sigma (Points (I)) ** 2;
            begin
               M := M + W * Outer (Points (I).H, Points (I).H);
               B := B + (W * Inverse_Depth (Points (I))) * Points (I).H;
               Count := Count + 1;
            end;
         end if;
      end loop;
      if Count <= 3 or else Determinant (M) = 0.0 then
         return;
      end if;
      declare
         Mi : constant Mat3 := Inverse (M);
         A  : constant Vec3 := Mi * B;
         --  The sandwich: the spread of every point's share of the gradient.
         Spread  : Mat3 := [others => [others => 0.0]];
         Squares : Real := 0.0;
      begin
         for I in Points'Range loop
            if On (I) and then Usable (Points (I)) then
               declare
                  W : constant Real := 1.0 / Inverse_Sigma (Points (I)) ** 2;
                  R : constant Real := Inverse_Depth (Points (I)) - A * Points (I).H;
               begin
                  Spread := Spread + (W * W * R * R) * Outer (Points (I).H, Points (I).H);
                  Squares := Squares + W * R * R;
               end;
            end if;
         end loop;
         Plane := (Found      => A * A > 0.0,
                   A          => A,
                   Covariance => (Real (Count) / Real (Count - 3)) * (Mi * Spread * Mi),
                   Points     => Count,
                   Scatter    => Squares / Real (Count - 3));
      end;
   end Refit_Plane;

   function Plane_Response
     (Points : Sight_Point_Array;
      On     : Flag_Array;
      Gains  : Real_Lists.Vector;
      Terms  : Natural) return Real_Matrix
   is
      Rows : constant Natural := (if Terms > 0 then Natural (Gains.Length) / Terms else 0);
      M    : Mat3 := [others => [others => 0.0]];
      K    : Real_Matrix (1 .. 3, 1 .. Terms) := [others => [others => 0.0]];
   begin
      if Terms = 0 or else Rows * Terms /= Natural (Gains.Length) then
         return K;
      end if;
      for I in Points'Range loop
         if On (I) and then Usable (Points (I)) then
            declare
               W   : constant Real := 1.0 / Inverse_Sigma (Points (I)) ** 2;
               Row : constant Natural := I - Points'First;
            begin
               M := M + W * Outer (Points (I).H, Points (I).H);
               --  An inverse depth moves by minus itself times its log depth's move.
               if Row < Rows then
                  for P in 1 .. Terms loop
                     declare
                        G : constant Real := Gains (Gains.First_Index + Row * Terms + P - 1);
                     begin
                        for D in 1 .. 3 loop
                           K (D, P) := K (D, P) - W * Inverse_Depth (Points (I)) * G * Points (I).H (D);
                        end loop;
                     end;
                  end loop;
               end if;
            end;
         end if;
      end loop;
      if Determinant (M) = 0.0 then
         return Real_Matrix'(1 .. 3 => [1 .. Terms => 0.0]);
      end if;
      return Inverse (M) * K;
   end Plane_Response;

   function Plane_Covariance
     (Points     : Sight_Point_Array;
      On         : Flag_Array;
      Plane      : Sight_Plane;
      Gains      : Real_Lists.Vector;
      Covariance : Real_Lists.Vector) return Mat3
   is
      Terms : constant Natural := Natural (Sqrt (Real (Natural (Covariance.Length))));
   begin
      if not Plane.Found or else Terms = 0 or else Terms * Terms /= Natural (Covariance.Length)
        or else (Natural (Gains.Length) / Terms) * Terms /= Natural (Gains.Length)
      then
         return Plane.Covariance;
      end if;
      declare
         Kf : constant Real_Matrix := Plane_Response (Points, On, Gains, Terms);
         V  : Real_Matrix (1 .. Terms, 1 .. Terms);
      begin
         for P in 1 .. Terms loop
            for Q in 1 .. Terms loop
               V (P, Q) := Covariance (Covariance.First_Index + (P - 1) * Terms + Q - 1);
            end loop;
         end loop;
         return Plane.Covariance + Kf * V * Transpose (Kf);
      end;
   end Plane_Covariance;

   function Plane_Estimate_Of (P : Sight_Plane; Covariance : Mat3) return Driver.Geometry.Plane_Estimate is
      Normal, E1, E2 : Vec3;
   begin
      if not P.Found then
         return (others => <>);
      end if;
      Plane_Axes (P, Normal, E1, E2);
      declare
         --  A point X of the plane (A * X = 1) is (1 - A * X) / |A| high, so
         --  its height moves by -(dA * X) / |A| and the normal by the part of
         --  -dA / |A| along the plane.
         Length  : constant Real := Sqrt (P.A * P.A);
         Foot    : constant Vec3 := (1.0 / Length ** 2) * P.A;   --  the plane's point nearest the eye
         S11     : constant Real := E1 * (Covariance * E1);
         S12     : constant Real := E1 * (Covariance * E2);
         S22     : constant Real := E2 * (Covariance * E2);
         Cross_1 : constant Real := E1 * (Covariance * Foot);
         Cross_2 : constant Real := E2 * (Covariance * Foot);
         Det     : constant Real := S11 * S22 - S12 * S12;
         --  The least uncertain height, at Foot + Along_1 E1 + Along_2 E2: the
         --  change of X' Covariance X along the plane is zero there. With the
         --  tilt known exactly, every point of the plane is as uncertain as
         --  every other.
         Along_1 : constant Real := (if Det > 0.0 then -(S22 * Cross_1 - S12 * Cross_2) / Det else 0.0);
         Along_2 : constant Real := (if Det > 0.0 then -(S11 * Cross_2 - S12 * Cross_1) / Det else 0.0);
         Centre  : constant Vec3 := Foot + Along_1 * E1 + Along_2 * E2;
      begin
         return (Centre       => Centre,
                 Normal       => Normal,
                 Tangent_1    => E1,
                 Tangent_2    => E2,
                 Offset_Sigma => Sqrt (Real'Max (0.0, Centre * (Covariance * Centre))) / Length,
                 Tilt_11      => S11 / Length ** 2,
                 Tilt_12      => S12 / Length ** 2,
                 Tilt_22      => S22 / Length ** 2,
                 Points       => P.Points,
                 Scatter      => P.Scatter);
      end;
   end Plane_Estimate_Of;

   --  How far a plane is from the eye.
   function Distance (P : Sight_Plane) return Real is (1.0 / Sqrt (P.A * P.A));

   procedure Dominant_Plane (Points : Sight_Point_Array; Plane : out Sight_Plane; On : out Flag_Array) is
      Minimal : constant := 3;
      Half    : constant := 0.5;   --  an even split: the null of two planes that hold as many points
      Taken   : Flag_Array (Points'Range) := [others => False];   --  on a plane found already
      First   : Boolean := True;
      Reference  : Natural := 0;   --  the most points on any plane found so far
      This       : Sight_Plane;
      This_On    : Flag_Array (Points'Range);
      This_Count : Natural;        --  the points on it

      --  Whether two planes are one way up to what their own tilts can tell: the angle between their normals is within Z
      --  times the root of the sum of their tilt variances.
      function Parallel (A, B : Sight_Plane) return Boolean is
         Cosine : constant Real := Real'Min (1.0, Real'Max (-1.0, Plane_Normal (A) * Plane_Normal (B)));
         Spread : constant Real := Sqrt (Plane_Tilt_Sigma (A) ** 2 + Plane_Tilt_Sigma (B) ** 2);
      begin
         return Arccos (Cosine) <= Driver.Conventions.Z * Spread;
      end Parallel;

      --  The plane most of the points not taken lie on: by consensus over samples of three, each scored by the squares
      --  of every point's residual in units of its sigma, none counted past Z squared; then by weighted least squares
      --  on the inverse depths of the points that lie on it (This_On), re-chosen at the residuals' own spread until
      --  the choice settles.
      procedure Consensus is
         Index : Count_Access := new Count_Array (1 .. Points'Length);
         Valid : Natural := 0;
      begin
         This := (others => <>);
         This_On := [others => False];
         This_Count := 0;
         for I in Points'Range loop
            if not Taken (I) and then Usable (Points (I)) then
               Valid := Valid + 1;
               Index (Valid) := I;
            end if;
         end loop;
         if Valid <= Minimal then
            Free (Index);
            return;
         end if;
         declare
            Sm      : Sampler;
            Best    : Real := Real'Last;
            Best_A  : Vec3 := [0.0, 0.0, 0.0];
            Support : Natural := 0;
            Needed  : Positive := Positive'Last;
            Drawn   : Natural := 0;
            Bound   : constant Real := Driver.Conventions.Z ** 2;
         begin
            --  Consensus: as many samples as the support found so far asks for.
            while Drawn < Needed loop
               Drawn := Drawn + 1;
               declare
                  Pick3 : Count_Array (1 .. Minimal);
                  M     : Mat3;
                  Q     : Vec3;
               begin
                  Pick (Sm, Valid, Pick3);
                  for R in 1 .. Minimal loop
                     declare
                        P : Sight_Point renames Points (Index (Pick3 (R)));
                     begin
                        for C in 1 .. 3 loop
                           M (R, C) := P.H (C);
                        end loop;
                        Q (R) := Inverse_Depth (P);
                     end;
                  end loop;
                  if Determinant (M) /= 0.0 then
                     declare
                        A     : constant Vec3 := Inverse (M) * Q;
                        Cost  : Real := 0.0;
                        Agree : Natural := 0;
                     begin
                        for K in 1 .. Valid loop
                           declare
                              Z2 : constant Real := Standard_Residual (A, Points (Index (K))) ** 2;
                           begin
                              Cost := Cost + Real'Min (Z2, Bound);
                              if Z2 <= Bound then
                                 Agree := Agree + 1;
                              end if;
                           end;
                        end loop;
                        if Cost < Best then
                           Best := Cost;
                           Best_A := A;
                           if Agree > Support then
                              Support := Agree;
                              Needed := Consensus_Samples (Minimal, Real (Support) / Real (Valid));
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Support <= Minimal then
               Free (Index);
               return;
            end if;
            --  From the consensus: the points within Z of their own sigma, the
            --  plane through them by weighted least squares, the choice renewed
            --  at the spread of their residuals until it settles.
            for K in 1 .. Valid loop
               This_On (Index (K)) := Standard_Residual (Best_A, Points (Index (K))) ** 2 <= Bound;
            end loop;
            declare
               Changed : Natural := Natural'Last;
            begin
               loop
                  Refit_Plane (Points, This_On, This);
                  exit when not This.Found;
                  declare
                     Z_All  : Real_Access := new Real_Array (1 .. Valid);
                     Count  : Natural := 0;
                     Spread : Real;
                     Now_Changed : Natural := 0;
                  begin
                     for K in 1 .. Valid loop
                        if This_On (Index (K)) then
                           Count := Count + 1;
                           Z_All (Count) := Standard_Residual (This.A, Points (Index (K)));
                        end if;
                     end loop;
                     Spread := Noise_Of (Z_All (1 .. Count));
                     Free (Z_All);
                     for K in 1 .. Valid loop
                        declare
                           Fits : constant Boolean :=
                             Spread > 0.0 and then not Driver.Uncertain.Significant
                                                         (Standard_Residual (This.A, Points (Index (K))), Spread);
                        begin
                           if Fits /= This_On (Index (K)) then
                              Now_Changed := Now_Changed + 1;
                              This_On (Index (K)) := Fits;
                           end if;
                        end;
                     end loop;
                     exit when Now_Changed = 0 or else Now_Changed >= Changed;
                     Changed := Now_Changed;
                  end;
               end loop;
               if This.Found then
                  Refit_Plane (Points, This_On, This);
               end if;
            end;
            for K in 1 .. Valid loop
               if This_On (Index (K)) then
                  This_Count := This_Count + 1;
               end if;
            end loop;
         end;
         Free (Index);
      end Consensus;
   begin
      Plane := (others => <>);
      On := [others => False];
      --  The table, among the planes found one after another (each the consensus of the points the earlier ones left,
      --  until one holds significantly fewer points than the most found, to within what chance explains: it would be
      --  the dominant plane of the rest), is not the plane most points lie on: a textured thing on a plain table holds
      --  more (A84 and A77 took a keyboard for the table). The choice begins at the plane that spans the most of the
      --  picture (the angular box of its points across and down, as the eye sees them: the table is the largest
      --  surface in a picture that looks at it); every other plane found then replaces it when it is nearer and not
      --  parallel to it (the room's far wall spans a picture that looks along it, and is no surface an arm works on),
      --  or when it is parallel, farther, and most of the choice's points lie within its box in the picture: the
      --  choice stands on it, a thing on the table (item 9).
      declare
         type Flag_Array_Access is access Flag_Array;
         procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Array_Access);
         type Candidate is record
            P      : Sight_Plane;
            On     : Flag_Array_Access;
            Count  : Natural := 0;
            X_Low, Y_Low   : Real := Real'Last;    --  the box its points span in the picture, as angles
            X_High, Y_High : Real := Real'First;
            Extent : Real := 0.0;    --  that box's area, across times down
         end record;
         package Candidate_Vectors is new Ada.Containers.Vectors (Positive, Candidate);
         Found  : Candidate_Vectors.Vector;
         --  Most of Q's points lie within C's box in the picture.
         function Stands_On (Q, C : Candidate) return Boolean is
            Inside, Total : Natural := 0;
         begin
            for I in Points'Range loop
               if Q.On (I) and then Points (I).H (3) > 0.0 then
                  Total := Total + 1;
                  declare
                     X : constant Real := Points (I).H (1) / Points (I).H (3);
                     Y : constant Real := Points (I).H (2) / Points (I).H (3);
                  begin
                     if X >= C.X_Low and then X <= C.X_High and then Y >= C.Y_Low and then Y <= C.Y_High then
                        Inside := Inside + 1;
                     end if;
                  end;
               end if;
            end loop;
            return 2 * Inside > Total;
         end Stands_On;
         Choice : Natural := 0;
      begin
         loop
            Consensus;
            exit when not This.Found;
            exit when not First
              and then Driver.Robot.Regression.Count_Significant (Reference, Reference + This_Count, Half);
            declare
               C : Candidate := (P => This, On => new Flag_Array'(This_On), Count => This_Count, others => <>);
            begin
               for I in Points'Range loop
                  if This_On (I) and then Points (I).H (3) > 0.0 then
                     C.X_Low := Real'Min (C.X_Low, Points (I).H (1) / Points (I).H (3));
                     C.X_High := Real'Max (C.X_High, Points (I).H (1) / Points (I).H (3));
                     C.Y_Low := Real'Min (C.Y_Low, Points (I).H (2) / Points (I).H (3));
                     C.Y_High := Real'Max (C.Y_High, Points (I).H (2) / Points (I).H (3));
                  end if;
               end loop;
               if C.X_High > C.X_Low and then C.Y_High > C.Y_Low then
                  C.Extent := (C.X_High - C.X_Low) * (C.Y_High - C.Y_Low);
               end if;
               Found.Append (C);
            end;
            Reference := Natural'Max (Reference, This_Count);
            First := False;
            for I in Points'Range loop
               Taken (I) := Taken (I) or else This_On (I);
            end loop;
         end loop;
         --  The choice begins at the plane that spans the most of the picture.
         for K in Found.First_Index .. Found.Last_Index loop
            if Choice = 0 or else Found (K).Extent > Found (Choice).Extent then
               Choice := K;
            end if;
         end loop;
         --  A nearer plane not parallel to the choice replaces it (the far wall); so does a farther parallel one
         --  that most of the choice's points lie within in the picture (the choice stands on it).
         if Choice > 0 then
            for K in Found.First_Index .. Found.Last_Index loop
               if K /= Choice then
                  if Parallel (Found (K).P, Found (Choice).P) then
                     if Distance (Found (K).P) > Distance (Found (Choice).P) and then Stands_On (Found (Choice), Found (K)) then
                        Choice := K;
                     end if;
                  elsif Distance (Found (K).P) < Distance (Found (Choice).P) then
                     Choice := K;
                  end if;
               end if;
            end loop;
            Plane := Found (Choice).P;
            On := Found (Choice).On.all;
         end if;
         for K in Found.First_Index .. Found.Last_Index loop
            declare
               C : Candidate := Found (K);
            begin
               Free (C.On);
            end;
         end loop;
      end;
   end Dominant_Plane;

   ---------------------------------------------------------------------------
   --  A plane through an eye

   procedure Turn_On_Plane
     (Before, After : Vec3_Array;
      Weight        : Real_Array;
      W             : Vec3;
      Turn          : out Real;
      Shift         : out Vec3;
      Normal        : out Vec3;
      Found         : out Boolean)
   is
      E1, E2 : Vec3;
      A      : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
      B1, B2 : Real_Vector (1 .. 3) := [others => 0.0];
      L      : Real_Matrix (1 .. 3, 1 .. 3);
      Pd     : Boolean;
   begin
      Turn := 0.0;
      Shift := [0.0, 0.0, 0.0];
      Normal := [0.0, 0.0, 0.0];
      Found := False;
      Perp (W, E1, E2);
      for I in Before'Range loop
         if Weight (I) > 0.0 then
            declare
               H0 : constant Vec3 := [Dot (E1, Before (I)), Dot (E2, Before (I)), Dot (W, Before (I))];
               H  : constant Vec3 := [Dot (E1, After (I)), Dot (E2, After (I)), Dot (W, After (I))];
               G  : constant Vec3 := [H (3) * H0 (1), H (3) * H0 (2), H (3) * H0 (3)];
            begin
               for P in 1 .. 3 loop
                  for Q in 1 .. 3 loop
                     A (P, Q) := A (P, Q) + Weight (I) * G (P) * G (Q);
                  end loop;
                  B1 (P) := B1 (P) + Weight (I) * G (P) * H (1) * H0 (3);
                  B2 (P) := B2 (P) + Weight (I) * G (P) * H (2) * H0 (3);
               end loop;
            end;
         end if;
      end loop;
      Driver.Numerics.Dense.Cholesky (A, L, Pd);
      if not Pd then
         return;
      end if;
      declare
         X1 : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, B1);   --  a, b, e
         X2 : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, B2);   --  c, d, f
         Pp : constant Real := X1 (1) + X2 (2);
         Qq : constant Real := X2 (1) - X1 (2);
         Gg : constant Real := X1 (1) * X2 (2) - X1 (2) * X2 (1) + 1.0;
         Rr : constant Real := Sqrt (Pp * Pp + Qq * Qq);
         Best_Phi    : Real := 0.0;
         Best_Minors : Real := Real'Last;
         Best_M      : Real_Matrix (1 .. 2, 1 .. 3) := [others => [others => 0.0]];
      begin
         if Rr = 0.0 then
            return;
         end if;
         declare
            Psi    : constant Real := Arctan (Qq, Pp);
            Spread : constant Real := Arccos (Real'Max (-1.0, Real'Min (1.0, Gg / Rr)));
         begin
            for Sign of Real_Array'[-1.0, 1.0] loop
               declare
                  Phi : constant Real := Psi + Sign * Spread;
                  C   : constant Real := Cos (Phi);
                  S   : constant Real := Sin (Phi);
                  M   : constant Real_Matrix (1 .. 2, 1 .. 3) :=
                    [[X1 (1) - C, X1 (2) + S, X1 (3)], [X2 (1) - S, X2 (2) - C, X2 (3)]];
                  --  Of rank one: every minor nought; their squares summed, what is left of it.
                  Minors : constant Real :=
                    (M (1, 1) * M (2, 2) - M (1, 2) * M (2, 1)) ** 2 + (M (1, 1) * M (2, 3) - M (1, 3) * M (2, 1)) ** 2
                    + (M (1, 2) * M (2, 3) - M (1, 3) * M (2, 2)) ** 2;
               begin
                  if Minors < Best_Minors then
                     Best_Minors := Minors;
                     Best_Phi := Phi;
                     Best_M := M;
                  end if;
               end;
            end loop;
         end;
         --  The rank-one part's factors: its row space's direction (the plane's normal) and the column that gives it.
         declare
            Mt_M : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
            Values  : Real_Vector (1 .. 3);
            Vectors : Real_Matrix (1 .. 3, 1 .. 3);
            Most    : Positive := 1;
         begin
            for P in 1 .. 3 loop
               for Q in 1 .. 3 loop
                  Mt_M (P, Q) := Best_M (1, P) * Best_M (1, Q) + Best_M (2, P) * Best_M (2, Q);
               end loop;
            end loop;
            Driver.Numerics.Dense.Small_Eigensystem (Mt_M, Values, Vectors);
            for P in 2 .. 3 loop
               if Values (P) > Values (Most) then
                  Most := P;
               end if;
            end loop;
            declare
               V  : constant Vec3 := [Vectors (1, Most), Vectors (2, Most), Vectors (3, Most)];
               U1 : constant Real := Best_M (1, 1) * V (1) + Best_M (1, 2) * V (2) + Best_M (1, 3) * V (3);
               U2 : constant Real := Best_M (2, 1) * V (1) + Best_M (2, 2) * V (2) + Best_M (2, 3) * V (3);
            begin
               Normal := V (1) * E1 + V (2) * E2 + V (3) * W;
               Shift := U1 * E1 + U2 * E2;
            end;
         end;
         Turn := -Best_Phi;
         Found := True;
      end;
   end Turn_On_Plane;

   --  The median of the lengths of two-coordinate residuals of Gaussian noise
   --  is sigma times the root of 2 ln 2.
   function Rayleigh_Median return Real is (Sqrt (2.0 * Ln (2.0)));

   function Slide_Turns_Of (Report : Fit_Report; N : Natural) return Vec3_Array is
      Turns : Vec3_Array (1 .. N) := [others => Zero3];
   begin
      if Natural (Report.Slide_Turns.Length) = 3 * N then
         for J in 1 .. N loop
            for A in 1 .. 3 loop
               Turns (J) (A) := Report.Slide_Turns (Report.Slide_Turns.First_Index + 3 * (J - 1) + A - 1);
            end loop;
         end loop;
      end if;
      return Turns;
   end Slide_Turns_Of;

end Driver.Robot.Kinematics.Fit;
