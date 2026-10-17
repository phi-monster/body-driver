with Ada.Containers.Generic_Array_Sort;
with Ada.Strings.Fixed;
with Ada.Calendar;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Robot.Regression;
with Driver.Robot.Kinematics.Errors;
with Driver.Stats;
with Driver.Uncertain;

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

   function Rot (W : Vec3; Theta : Real) return Mat3 is (Driver.Numerics.Exp (Theta * W));

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
      Mu : constant Real := Driver.Stats.Median (Du);
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
   function Median_Error (Median : Real; Count : Positive) return Real is
      Q     : constant Real := Upper_Quartile;
      Sigma : constant Real := Median / Q;
      F     : constant Real := 2.0 * Exp (-Q * Q / 2.0) / Sqrt (2.0 * Ada.Numerics.Pi);
   begin
      return Sigma / (2.0 * Sqrt (Real (Count)) * F);
   end Median_Error;

   --  The scale a residual is judged at, whose own sighting has a stated standard deviation Own: what the model leaves
   --  of any sighting (Floor) and what the matcher states of this one, as variances.
   function Scale_Of (Floor, Own : Real) return Real is (Real'Max (Sqrt (Floor ** 2 + Own ** 2), Real'Model_Small));

   --  How many degrees of freedom the scale a residual is judged at rests on, as Scale_Of makes it: the sighting's own
   --  standard deviation rests on Own_Freedom of them and the floor on none to speak of (it is read from thousands of
   --  residuals), so the scale's variance has as many as the Welch-Satterthwaite formula gives; none (the scale is
   --  known) when the sighting's own is.
   function Scale_Freedom (Floor, Own : Real; Own_Freedom : Natural) return Natural is
     (if Own_Freedom = 0 or else Own <= 0.0 then 0
      else Driver.Uncertain.Satterthwaite_Count (Real (Own_Freedom) * ((Floor ** 2 + Own ** 2) / Own ** 2) ** 2));

   --  The floor of the scales the residuals R are judged at, beside the standard deviations Own their sightings
   --  state (one per residual): the one at which the median of a residual over its scale is the Gaussian's, so a
   --  residual of the size typical of its sighting is typical. With none stated it is Noise_Of; a floor of zero
   --  when what the sightings state accounts for the residuals by itself.
   function Floor_Of (R, Own : Real_Array) return Real is
      Typical : constant Real := Upper_Quartile;
      High    : constant Real := Noise_Of (R);
      Low     : Real := 0.0;
      Top     : Real := High;

      --  Whether the median of the residuals over their scales at that floor is above the Gaussian's, told by
      --  counting them as Driver.Stats.Median would take it, without selecting it: of an odd count it is above
      --  when fewer than its rank lie at or below; of an even count, when fewer than half do, or when exactly half
      --  do and the mean of the greatest of them and the least of the rest is above. Each step of the bisection took
      --  a median of every residual: 1.1 s of the fit's thread of 19 s (s02's dump replayed and sampled, 10-16).
      function Above_Typical (Floor : Real) return Boolean is
         N       : constant Natural := R'Length;
         At_Most : Natural := 0;
         Lower   : Real := -Real'Last;   --  the greatest ratio at or below Typical
         Upper   : Real := Real'Last;    --  the least above it
      begin
         for I in 1 .. N loop
            declare
               X : constant Real := abs R (R'First + I - 1) / Scale_Of (Floor, Own (Own'First + I - 1));
            begin
               if X <= Typical then
                  At_Most := At_Most + 1;
                  Lower := Real'Max (Lower, X);
               else
                  Upper := Real'Min (Upper, X);
               end if;
            end;
         end loop;
         if N mod 2 = 1 then
            return At_Most < (N + 1) / 2;
         elsif At_Most /= N / 2 then
            return At_Most < N / 2;
         else
            return (Lower + Upper) / 2.0 > Typical;
         end if;
      end Above_Typical;
   begin
      if R'Length = 0 or else High <= 0.0 then
         return 0.0;
      end if;
      if (for all O of Own => O = 0.0) then
         return High;
      end if;
      if not Above_Typical (0.0) then
         return 0.0;
      end if;
      --  The ratio falls as the floor rises, from above the Gaussian's at none to below it at High.
      for Step in 1 .. Real'Machine_Mantissa loop
         declare
            Middle : constant Real := (Low + Top) / 2.0;
         begin
            exit when Middle <= Low or else Middle >= Top;
            if Above_Typical (Middle) then
               Low := Middle;
            else
               Top := Middle;
            end if;
         end;
      end loop;
      return Top;
   end Floor_Of;

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
                           Evaluate (Xn, Rn.all);
                           declare
                              C : constant Real := Cost (Rn.all);
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
   procedure Refine_Tracks
     (Changes      : Driver.Numerics.Arrays.Real_Matrix;
      Sight        : Sighting_Array;
      Joints       : in out Joint_Array;
      Chain        : Chain_Order;
      L            : in out Lens;
      Report       : in out Fit_Report;
      Focal_Within : Real := 0.0;
      Spread       : Boolean := True)
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
        ((Fx => Exp (X (X'First)), Fy => Exp (X (X'First + 1)), Cx => X (X'First + 2), Cy => X (X'First + 3),
          K1 => X (X'First + 4), K2 => X (X'First + 5)));

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
            Reduced : constant Positive := 2 * N;
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
              (V (2 * J - 1) * E_1 (J) + V (2 * J) * E_2 (J));

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
                     Floor : constant Real := (if Used > 0 then Floor_Of (Sub (1 .. Used), Own_Sub (1 .. Used)) else 0.0);
                  begin
                     for I in 1 .. S loop
                        declare
                           Scale : constant Real := Scale_Of (Floor, Own (I));
                           Seen  : constant Boolean := Rho (I) < Real'Last and then Scale > Real'Model_Small;
                           Fits  : constant Boolean := Seen and then not Driver.Uncertain.Significant (Rho (I), Scale,
                                    Scale_Freedom (Floor, Own (I), Sight (Sight'First + I - 1).Own_Freedom));
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
                        Step : constant Mat3 := Rot (A.W, A.C * Change (J));
                     begin
                        Maps ((F - 1) * N + J) := Turn * (Identity3 - Step);
                        Turn := Turn * Step;
                     end;
                  end loop;
                  Turns (F) := Turn;
                  Poses (F) := Eye_At (Joints, Chain, Change);
               end;
            end loop;
            for J in 1 .. N loop
               Perp (Joints (Joints'First + J - 1).W, E_1 (J), E_2 (J));
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
                     for Axis in 1 .. 2 loop
                        declare
                           Col : constant Vec3 :=
                             Cross (B, Back * (Maps ((Sg.Frame - 1) * N + J) * (if Axis = 1 then E_1 (J) else E_2 (J))));
                        begin
                           for D in 1 .. 3 loop
                              Across_Columns (3 * I - 3 + D, 2 * J - 2 + Axis) := Col (D);
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
                  Floor : constant Real := Floor_Of (Rs.all, Own.all);
               begin
                  for I in 1 .. S loop
                     declare
                        Scale : constant Real := Scale_Of (Floor, Own (I));
                     begin
                        if not Driver.Uncertain.Significant (Rs (I), Scale,
                                                          Scale_Freedom (Floor, Own (I), Sight (Sight'First + I - 1).Own_Freedom))
                        then
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
                     Joints (Joints'First + J - 1).P := Size * Point_Of (J, Q);
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
                  Sigma : Real;
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
                  Sigma := Floor_Of (R0.all, Owns.all);
                  for I in 1 .. 2 * Used loop
                     Scales (I) := Scale_Of (Sigma, Owns (I));
                  end loop;
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
                           Xp (P) := Xp (P) + H;
                           Residuals (Xp, Depth.all, Index, Rp.all,
                                      Joint => (if P <= 6 then 0 else (P - 7) / Per_Joint + 1));
                           for I in 1 .. 2 * Used loop
                              Jp (I, P) := (Rp (I) - R0 (I)) / H;
                           end loop;
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
                                          begin
                                             Sum_G := Sum_G - W * Jp (I, P) * R0 (I);
                                             Bm (P, Track_Of (I)) := Bm (P, Track_Of (I)) + W * Jp (I, P) * Jd (I);
                                             for Q in P .. Count loop
                                                Sum_A (Q) := Sum_A (Q) + W * Jp (I, P) * Jp (I, Q);
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
                           end loop;
                           Rows (1, Count + 1);
                           Free (Weights);
                           Free (Track_Of);
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
               Now_Changed : Natural := 0;
               Fit_Abs   : Real_Access;
               Kf        : Natural := 0;
            begin
               for I in 1 .. S loop
                  All_Index (I) := Real (I);
                  All_Own (2 * I - 1) := Sight (Sight'First + I - 1).Own;
                  All_Own (2 * I) := Sight (Sight'First + I - 1).Own;
               end loop;
               Residuals (X, Depth.all, All_Index, All_R.all);
               Sigma := Floor_Of (All_R.all, All_Own.all);
               for I in 1 .. S loop
                  declare
                     Scale : constant Real := Scale_Of (Sigma, All_Own (2 * I));
                     Freedom : constant Natural :=
                       Scale_Freedom (Sigma, All_Own (2 * I), Sight (Sight'First + I - 1).Own_Freedom);
                     Fits  : constant Boolean := Has (Sight (Sight'First + I - 1).Track) and then Scale > Real'Model_Small
                       and then not Driver.Uncertain.Significant (All_R (2 * I - 1), Scale, Freedom)
                       and then not Driver.Uncertain.Significant (All_R (2 * I), Scale, Freedom);
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
         if not Determined then
            Report.Why := Why_Not;
         else
            Report.Focal_Sigma := L.Fx * Sqrt (Variance (1));
            if not Driver.Uncertain.Significant (L.Fx, L.Fx * Sqrt (Variance (1)))
              or else not Driver.Uncertain.Significant (L.Fy, L.Fy * Sqrt (Variance (2)))
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
                    and then (Driver.Conventions.Z * Tilt >= Ada.Numerics.Pi / 2.0
                              or else not Driver.Uncertain.Significant (C, Sqrt (Variance (O + 5))))
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

      --  Only joint J moved in the keyframe, as far as its eye can tell: it was asked to, and every other joint
      --  stands where it stood in the reference.
      function Only (Frame, J : Positive) return Boolean is
        (Moved (Frame, J) and then (for all C in 1 .. N => C = J or else not Away (Frame, C)));

      function Single (Frame : Positive) return Natural is
      begin
         for J in 1 .. N loop
            if Only (Frame, J) then
               return J;
            end if;
         end loop;
         return 0;
      end Single;

      --  The keyframe stands from the reference by two joints or more, as far as their eyes can tell, asked now or
      --  left there before: what tells the joints' order.
      function Several (Frame : Positive) return Boolean is
        (for some J in 1 .. N => Away (Frame, J) and then (for some K in J + 1 .. N => Away (Frame, K)));

      --  Each keyframe's joint of its own, and the keyframe it shows that joint's turn against: the reference when
      --  every other joint stands where it stood there (Single); else, of the keyframes with sightings, the one that
      --  differs from it in that joint alone and by the most. A servo sent back to the sweep's start lands within its
      --  dead band of it, so two landings differ by up to twice that, and the reference is a landing too (f15c on
      --  10-15, arm 2: the joints not asked stood 4 and 8 units from the reference at a dead band of 4, and five
      --  joints of six had no keyframe of their own against it). Two pushes of one joint from one start share every
      --  other joint's landing. Stages 1 and 2 fit one joint at a time and take a keyframe's sightings against its
      --  partner (the point's place there: Pairs); the stages after take them against the reference, with every
      --  joint's reading.
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
      --  Every track's depth from the keyframes of one joint each (stage 4).
      Track_Depths : Real_Lists.Vector;
      Usable : Flags (1 .. N) := [others => False];

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
      procedure Poses_Of (Jx : Joint_Array; P : out Pose_Array) is
      begin
         for Frame in 1 .. Frames loop
            P (Frame) := Eye_At (Jx, Order, Changes_Of (Frame));
         end loop;
      end Poses_Of;

      --  Lengths in model units: the eye positions over the unit's keyframes
      --  have a root mean square of one.
      procedure Normalize (Jx : in out Joint_Array) is
         Sum : Real := 0.0;
      begin
         for Frame in 1 .. Unit_Count loop
            declare
               T : constant Vec3 := Eye_At (Jx, Order, Changes_Of (Frame)).Translation;
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
         Refine_Tracks (Changes, Sight, Joints, Order, L, Report, Focal_Within);
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
            pragma Unreferenced (Down);
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
                     end record;
                     Walked        : array (Sign_Way) of Walk_Result;
                     Walked_Joints : array (Sign_Way) of Joint_Array (Joints'Range);

                     procedure Walk (Index : Positive) is
                        Way    : constant Sign_Way := Sign_Way'Val (Index - 1);
                        Offset : Real := Driver.Conventions.Z * Held;
                     begin
                        loop
                           declare
                              Factor : constant Real := Exp ((if Way = Up then Offset else -Offset));
                              Lp     : Lens := L;
                              Jp     : Joint_Array := Joints;
                              Rp     : Fit_Report;
                           begin
                              exit when L.Fx * Factor < Searched_Least or else L.Fx * Factor > Searched_Most;
                              Lp.Fx := L.Fx * Factor;
                              Lp.Fy := L.Fy * Factor;
                              Refine_Tracks (Changes, Sight, Jp, Order, Lp, Rp, Held, Spread => False);
                              exit when Rp.All_Count = 0;
                              declare
                                 Err : constant Real :=
                                   Sqrt (Median_Error (Rp.All_Median_Px, Rp.All_Count) ** 2 + Base_Err ** 2);
                              begin
                                 Driver.Log.Line (Driver.Log.Robot, "kinematics: the focal length's profile at "
                                                  & Driver.Log.Image (Lp.Fx, 1) & " px leaves a median "
                                                  & Driver.Log.Image (Rp.All_Median_Px, 3) & " px over every sighting, "
                                                  & "against " & Driver.Log.Image (Base, 3) & " px at "
                                                  & Driver.Log.Image (L.Fx, 1) & " (+- " & Driver.Log.Image (Err, 3) & ")");
                                 if Rp.All_Median_Px + Driver.Conventions.Z * Err < Base then
                                    --  A better basin: refitted from there, the focal length free within the search's width.
                                    Refine_Tracks (Changes, Sight, Jp, Order, Lp, Rp, Focal_Within);
                                    if Rp.All_Count > 0 and then Rp.All_Median_Px < Base then
                                       Walked (Way) := (Moved => True, L => Lp, Report => Rp);
                                       Walked_Joints (Way) := Jp;
                                    end if;
                                    exit;
                                 end if;
                                 exit when Rp.All_Median_Px > Base + Driver.Conventions.Z * Err;
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
                           L := Walked (Way).L;
                           Joints := Walked_Joints (Way);
                           Report := Walked (Way).Report;
                           Moved := True;
                           exit;
                        end if;
                     end loop;
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
         Chain := Order;
         Report.Fitted := Report.Determined;
         Close_Stage;
      end Finish;

   begin
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
         Tracks  : Natural := 0;
      begin
         for X of Sight loop
            Sighted (X.Frame) := True;
            Tracks := Natural'Max (Tracks, X.Track);
         end loop;
         for B in 2 .. Frames loop
            if Sighted (B) then
               if Single (B) > 0 then
                  Own_Joint (B) := Single (B);
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
      if N > 1 and then (for all X of Sight => not Several (X.Frame)) then
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
                  Eigensystem (A, Values, Vectors);
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
                  Eigensystem (Ftf, Values, Vectors);
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
            procedure Planar_About
              (J : Positive; F : Real; W : Vec3; Wt, Rs, Fits : in out Real_Array; Scale, Score : out Real)
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
                                 Eigensystem (A, Values, Vectors);
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
                                    Tt   : constant Vec3 := (-V (2)) * E1 + V (1) * E2;
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
                                    declare
                                       Sigma : constant Real := Noise_Of (Rs (1 .. M));
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
                                          Y  : constant Vec3 := Rr * Before (Q) + Dot (Nm, Before (Q)) * Sh;
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
                           T0_Of (Way) := (-V (2)) * E1 + V (1) * E2;
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
                                      Rr_Of (Way) * L (I).H0 + Dot (Plane_Of (Way), L (I).H0) * T0_Of (Way);
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
               Score := (if Filled > 0 then Driver.Stats.Median (Fits (1 .. Filled)) else Real'Last);
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

            --  Every joint's best direction at focal length F, the joints scanned
            --  at once, one task each: their sightings are their own.
            Searches        : Natural := 0;          --  focal lengths tried, for the log
            F_Least, F_Most : Real := 0.0;           --  the least and the most of them, px
            Warm            : Best_Array;            --  per joint: its axis at the focal length tried before
            Warm_Angle      : array (1 .. N) of Real := [others => -1.0];   --  per joint, this search: see Scanner
            Warm_Better     : array (1 .. N) of Boolean := [others => False];
            Warm_Tries, Warm_Wins : Natural := 0;    --  over the searches, for the log
            Warm_Far        : Real := 0.0;           --  the farthest a warm start lay from the scan, radians
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
                              Planar_About (J, F, D, Wt.all, Rs.all, Fs.all, Sc, Sd);
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
                        --  Measured and not yet used: the joint's axis at the focal length tried before, refined at
                        --  this one, and how far it lies from the scan's (whether the scan could be skipped between the
                        --  focal lengths of one search). A lattice ladder in place of the scan was tried on 10-13 and
                        --  put 7 of 16 fit tests red: a refined axis is as stable in a wrong basin as in the right one.
                        if Result.Scale /= 0.0 and then Warm (J).Scale /= 0.0 and then Every (J)'Length > 4 then
                           declare
                              Again : Best_Record := Warm (J);
                           begin
                              Refine (J, F, Again);
                              Warm_Angle (J) := Arccos (Real'Min (1.0, abs Dot (Again.W, Result.W)));
                              Warm_Better (J) := Again.Score < Result.Score;
                           end;
                        else
                           Warm_Angle (J) := -1.0;
                           Warm_Better (J) := False;
                        end if;
                        Warm (J) := Result;
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
               for J in 1 .. N loop
                  if Warm_Angle (J) >= 0.0 then
                     Warm_Tries := Warm_Tries + 1;
                     Warm_Far := Real'Max (Warm_Far, Warm_Angle (J));
                     if Warm_Better (J) then
                        Warm_Wins := Warm_Wins + 1;
                     end if;
                  end if;
               end loop;
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
                                & Driver.Log.Image (Exp (Best_At + Told_Within), 1) & "; warm starts" & Warm_Tries'Image
                                & ", farthest " & Driver.Log.Image (Warm_Far * 180.0 / Ada.Numerics.Pi, 2)
                                & " deg from the scan, better" & Warm_Wins'Image & " times");
               Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & " focal search curve, px:score"
                                & Ada.Strings.Unbounded.To_String (Trace));
               for J in 1 .. N loop
                  if Best (J).Scale = 0.0 then
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
                       (W => Best (J).W, P => Cos (Best (J).Phi) * E1 + Sin (Best (J).Phi) * E2, C => Best (J).Scale,
                        Slide => False);
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
                     Out_J (Out_J'First + J - 1).P := Cos (X (O + 2)) * F1 + Sin (X (O + 2)) * F2;
                     Out_J (Out_J'First + J - 1).C :=
                       (if Base (Base'First + J - 1).C < 0.0 then -Exp (X (O + 3)) else Exp (X (O + 3)));
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
                     Rj : constant Mat3 := Rot (A.W, A.C * Own_Change (Kept (I).Frame, J));
                     Rr : Mat3;
                     Tt : Vec3;
                  begin
                     Relative (Identity, (Rotation => Rj, Translation => A.P - Rj * A.P), Rr, Tt);
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
                  X (4 * (J - 1) + 3) := Arctan (Base (Base'First + J - 1).P * E2, Base (Base'First + J - 1).P * E1);
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
                        R  : constant Mat3 := Rot (A.W, A.C * Own_Change (X.Frame, J));
                        T  : constant Vec3 := A.P - R * A.P;
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
                     Joints (Joints'First + J - 1).P := -Joints (Joints'First + J - 1).P;
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
               Joints (Joints'First + J - 1).P := Rho (J) * Joints (Joints'First + J - 1).P;
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
            Lens0 := Lx;
            Track_Depths := Rep.Depths;
            Free (Alone);
         end;
      end;

      ---------------------------------------------------------------------------
      --  Stage 4: first the order of the joints along the arm. The evidence is
      --  the sightings of the keyframes that moved several joints at once;
      --  without one the order is not told. Their epipolar residual cannot
      --  tell it: the order moves the eye mostly in how far it went, which
      --  the epipolar constraint does not see (A17's presses, the readings
      --  drawn: a wrong order left its median at 0.058 px). Where the points
      --  land does: every point the reference eye saw, at the depth the
      --  keyframes of one joint each gave it, reprojected into those keyframes,
      --  scores the order. Lengths in any unit: the depths are in the joints'.
      Order_Is_Told := N <= 1;
      if N > 1 then
         declare
            Multi, Tracks : Natural := 0;
         begin
            for X of Sight loop
               Tracks := Natural'Max (Tracks, X.Track);
               if Several (X.Frame) then
                  Multi := Multi + 1;
               end if;
            end loop;
            if Multi = 0 then
               Fail ("no keyframe moved two joints at once: the order of the joints along the arm is not told");
               return;
            end if;
            declare
               Depth  : Real_Access := new Real_Array'(1 .. Tracks => 0.0);   --  0: not placed
               R      : Real_Access := new Real_Array (1 .. 2 * Multi);
               Here_R : Real_Access := new Real_Array (1 .. 2 * Multi);   --  the residuals of the order found
               Poses  : Rigid_Access := new Pose_Array;
               Placed : Natural := 0;   --  sightings of placed points in those keyframes

               --  The placed sightings, each with its point where its track's depth puts it along its reference line
               --  of sight: no order changes them, and the line of sight (Ray, through the lens's distortion) was found
               --  again for every sighting of every order, 1440 orders a fit of six joints (a13 on 10-16: stage 4 took
               --  7.8 to 9.9 s a fit against 1.4 to 2.1 before every order was scored).
               type Placed_Sighting is record
                  Frame : Positive := 1;
                  Point : Vec3 := Zero3;
                  U, V  : Real := 0.0;
               end record;
               type Placed_Array is array (Positive range <>) of Placed_Sighting;
               type Placed_Access is access Placed_Array;
               procedure Free is new Ada.Unchecked_Deallocation (Placed_Array, Placed_Access);
               Placed_At : Placed_Access;

               --  The median across and down of the reprojections of those sightings with the joints in order C,
               --  in pixels, and its standard error, the residuals into Into, the eye's poses into At_Poses. A point
               --  the order puts behind the eye lands nowhere.
               procedure Score_Into
                 (C : Chain_Order; At_Poses : in out Rigid_Array; Into : in out Real_Array; Median, Error : out Real)
               is
                  K : Natural := 0;
               begin
                  for Frame in 1 .. Frames loop
                     At_Poses (Frame) := Eye_At (Joints, C, Changes_Of (Frame));
                  end loop;
                  for P of Placed_At.all loop
                     declare
                        T     : Rigid renames At_Poses (P.Frame);
                        U, V  : Real;
                        Ahead : Boolean;
                     begin
                        Project (Lens0, Transpose (T.Rotation) * (P.Point - T.Translation), U, V, Ahead);
                        Into (K + 1) := (if Ahead then U - P.U else Real'Last);
                        Into (K + 2) := (if Ahead then V - P.V else Real'Last);
                        K := K + 2;
                     end;
                  end loop;
                  Median := Median_Abs (Into (1 .. K));
                  --  The median point lands behind the eye: no residual, and none to lower by a standard error.
                  if Median >= Real'Last then
                     Median := Real'Last;
                     Error := 0.0;
                  else
                     Error := Median_Error (Median, K);
                  end if;
               end Score_Into;

               --  Whether the residuals A are significantly the smaller than B, residual by residual (the same
               --  sightings, in the same order): Wilcoxon's signed ranks of |A| - |B|, at Z of their normal
               --  approximation. Two orders' medians are each as uncertain as their residuals are few, but they are
               --  read off the same sightings, and what tells one order from another is how each sighting moves:
               --  compared by the standard error of one median, the true order was no move from a wrong one
               --  (robot.kinematics.desktop with stage 1's plane, 10-16: 0.647 against 0.660 +- 0.030 px), and taken
               --  wherever a median was lower, the search wandered to wrong orders whose medians the noise put lower
               --  still (all three of its sweeps).
               function Smaller (A, B : Real_Array) return Boolean is
                  type Signed is record
                     Size     : Real := 0.0;
                     Negative : Boolean := False;
                  end record;
                  type Signed_Array is array (Positive range <>) of Signed;
                  function "<" (L, R : Signed) return Boolean is (L.Size < R.Size);
                  procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Signed, Signed_Array);
                  type Signed_Access is access Signed_Array;
                  procedure Free is new Ada.Unchecked_Deallocation (Signed_Array, Signed_Access);
                  --  On the heap: a worker task's stack does not hold 2 a placed sighting.
                  D_Of : Signed_Access := new Signed_Array (1 .. A'Length);
                  D    : Signed_Array renames D_Of.all;
                  M    : Natural := 0;
               begin
                  for I in A'Range loop
                     declare
                        X : constant Real := abs A (I);
                        Y : constant Real := abs B (B'First + I - A'First);
                     begin
                        if X /= Y then
                           M := M + 1;
                           D (M) := (Size     => (if X >= Real'Last or else Y >= Real'Last then Real'Last
                                                  else abs (X - Y)),
                                     Negative => X < Y);
                        end if;
                     end;
                  end loop;
                  if M = 0 then
                     Free (D_Of);
                     return False;
                  end if;
                  Sort (D (1 .. M));
                  declare
                     Smaller_Ranks : Real := 0.0;   --  the ranks of the differences where A is the smaller
                     From          : Positive := 1;
                     Mm            : constant Real := Real (M);
                  begin
                     --  Ties share the mean of their ranks.
                     while From <= M loop
                        declare
                           To : Positive := From;
                        begin
                           while To < M and then D (To + 1).Size = D (From).Size loop
                              To := To + 1;
                           end loop;
                           for I in From .. To loop
                              if D (I).Negative then
                                 Smaller_Ranks := Smaller_Ranks + Real (From + To) / 2.0;
                              end if;
                           end loop;
                           From := To + 1;
                        end;
                     end loop;
                     return Is_Smaller : constant Boolean :=
                       (Smaller_Ranks - Mm * (Mm + 1.0) / 4.0)
                       > Driver.Conventions.Z * Sqrt (Mm * (Mm + 1.0) * (2.0 * Mm + 1.0) / 24.0)
                     do
                        Free (D_Of);
                     end return;
                  end;
               end Smaller;


               procedure Free_All is
               begin
                  Free (Depth);
                  Free (R);
                  Free (Here_R);
                  Free (Poses);
                  Free (Placed_At);
               end Free_All;

               Here : Real;
            begin
               for I in 1 .. Natural'Min (Tracks, Natural (Track_Depths.Length)) loop
                  Depth (I) := Real'Max (0.0, Track_Depths (Track_Depths.First_Index + I - 1));
               end loop;
               for X of Sight loop
                  if Several (X.Frame) and then Depth (X.Track) > 0.0 then
                     Placed := Placed + 1;
                  end if;
               end loop;
               if Placed = 0 then
                  Free_All;
                  Fail ("no point the keyframes of one joint placed is seen in a keyframe that moved several: the "
                        & "order of the joints along the arm is not told");
                  return;
               end if;
               Placed_At := new Placed_Array (1 .. Placed);
               declare
                  K : Natural := 0;
               begin
                  for X of Sight loop
                     if Several (X.Frame) and then Depth (X.Track) > 0.0 then
                        K := K + 1;
                        Placed_At (K) := (Frame => X.Frame, Point => Depth (X.Track) * Ray (Lens0, X.U0, X.V0),
                                          U => X.U, V => X.V);
                     end if;
                  end loop;
               end;
               --  Every order of the joints, scored (Score): the search moved one joint at a time from a start ordered
               --  by the axes' distances from the eye and stopped where no move lowered the median by more than its
               --  standard error, and f16b's arm 1 came out in the wrong order in every pass at twice the right order's
               --  median (10-15), the desktop's arm at 11.5 degrees in two wrong orders once stage 1 told its lens
               --  (10-16). N! orders of one reprojection each: 720 for six joints. The order is the one of the least
               --  median, the first of equals in the orders' lexicographic sequence.
               --  Every first two joints at once (Driver.Parallel_For), each pair its own orders in their lexicographic
               --  sequence with buffers of its own; the least median over them taken in the pairs' order, so that it is
               --  the first of equals in the whole sequence, as one search of it in turn finds. A way a first joint
               --  left six of them to ten processors and the Pi's four two rounds (a19 on 10-16: 3.8 to 17.2 s a fit);
               --  a way a pair, 30.
               declare
                  type Chain_Access is access Chain_Order;
                  procedure Free is new Ada.Unchecked_Deallocation (Chain_Order, Chain_Access);
                  type Way_Result is record
                     Found       : Boolean := False;
                     Median      : Real := Real'Last;
                     Order       : Chain_Access;
                     Residuals   : Real_Access;
                     Fails       : Boolean := False;   --  some order of its own is not significantly the larger
                     Fail_Order  : Chain_Access;
                     Fail_Median : Real := 0.0;
                  end record;
                  --  The ways: every first joint, and when there are two or more joints, every second one after it.
                  Kept    : constant Positive := Natural'Min (2, N);
                  Ways    : constant Positive := (if N >= 2 then N * (N - 1) else N);
                  Results : array (1 .. Ways) of Way_Result;
                  Found   : Boolean := False;

                  --  The first order of way W: its first joints (the W-th pair of them in lexicographic order), the
                  --  rest in their own order.
                  procedure First_Of (W : Positive; C : out Chain_Order) is
                     J : constant Positive := (if Kept = 2 then (W - 1) / (N - 1) + 1 else W);
                     S : constant Natural := (if Kept = 2 then (W - 1) mod (N - 1) + 1 else 0);   --  of the others
                     K : Positive := Kept + 1;
                     Seen : Natural := 0;
                  begin
                     C (1) := J;
                     for Other in 1 .. N loop
                        if Other /= J then
                           Seen := Seen + 1;
                           if Seen = S then
                              C (2) := Other;
                           end if;
                        end if;
                     end loop;
                     for Other in 1 .. N loop
                        if Other /= J and then (Kept = 1 or else Other /= C (2)) then
                           C (K) := Other;
                           K := K + 1;
                        end if;
                     end loop;
                  end First_Of;

                  --  C made the next order in lexicographic sequence with its first Kept joints kept; False after the
                  --  last.
                  function Next (C : in out Chain_Order) return Boolean is
                     I : Natural := N - 1;
                  begin
                     while I >= Kept + 1 and then C (I) > C (I + 1) loop
                        I := I - 1;
                     end loop;
                     if I < Kept + 1 then
                        return False;
                     end if;
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
                     return True;
                  end Next;

                  function Image (O : Chain_Order) return String is
                     Text : Ada.Strings.Unbounded.Unbounded_String;
                  begin
                     for K of O loop
                        Ada.Strings.Unbounded.Append (Text, K'Image);
                     end loop;
                     return Ada.Strings.Unbounded.To_String (Text);
                  end Image;

                  procedure Search_From (W : Positive) is
                     C      : Chain_Order (1 .. N);
                     Own    : Rigid_Access := new Pose_Array;
                     Into   : Real_Access := new Real_Array (1 .. 2 * Multi);
                     M, E   : Real;
                     Result : Way_Result renames Results (W);
                  begin
                     Result.Order := new Chain_Order (1 .. N);
                     Result.Residuals := new Real_Array (1 .. 2 * Multi);
                     First_Of (W, C);
                     loop
                        Score_Into (C, Own.all, Into.all, M, E);
                        if not Result.Found or else M < Result.Median then
                           Result.Found := True;
                           Result.Median := M;
                           Result.Order.all := C;
                           Result.Residuals.all := Into.all;
                        end if;
                        exit when not Next (C);
                     end loop;
                     Free (Own);
                     Free (Into);
                  end Search_From;
                  procedure Search_All is new Driver.Parallel_For (Search_From);

                  --  Told where every other order leaves residuals significantly the larger (Smaller): an order the
                  --  sightings cannot tell from it leaves the joints' order to chance (the desktop's sweep of 3.4
                  --  degrees, 10-16: no other order's residuals significantly the larger, and a wrong one reported
                  --  fitted once stage 1 found the lens; s02's arm, 4157 sightings: the fit ordered its joints 1 5 4 2
                  --  3 6 where they are 1 5 4 3 6 2, by the fake robot's scramble of their channels). An order not told
                  --  is still the arm's: its residuals are the least, and it predicts the views the keyframes took;
                  --  Order_Told says that it is not told, and the cells that move several joints, taken larger once
                  --  the arm is fitted (Boot), are what tell it. Every first joint at once, each its own orders until
                  --  the first that is not significantly the larger; the first such in the whole sequence is said.
                  procedure Tell_From (W : Positive) is
                     C      : Chain_Order (1 .. N);
                     Own    : Rigid_Access := new Pose_Array;
                     Into   : Real_Access := new Real_Array (1 .. 2 * Multi);
                     M, E   : Real;
                     Result : Way_Result renames Results (W);
                  begin
                     First_Of (W, C);
                     loop
                        if C /= Order then
                           Score_Into (C, Own.all, Into.all, M, E);
                           if not Smaller (Here_R (1 .. 2 * Placed), Into (1 .. 2 * Placed)) then
                              Result.Fails := True;
                              Result.Fail_Order := new Chain_Order'(C);
                              Result.Fail_Median := M;
                              exit;
                           end if;
                        end if;
                        exit when not Next (C);
                     end loop;
                     Free (Own);
                     Free (Into);
                  end Tell_From;
                  procedure Tell_All is new Driver.Parallel_For (Tell_From);

                  --  An arm fitted before whose order was told keeps it while every order a swap of two joints next to
                  --  each other away from it still leaves residuals significantly the larger (Smaller, the test that
                  --  told it): those are the orders the sightings tell apart last (s10's untold orders were two such
                  --  swaps, of joints whose axes meet). Every order searched and then scored again to tell the best took
                  --  7.8 to 9.9 s of a fit (a13), fit after fit, while the sweep went on adding keyframes to an order
                  --  already told: a22 (10-16) told 5 4 1 2 6 3 in its third pass and fitted twenty times more, its
                  --  thirteen fits that gave their stages' times spending 112 s in this stage. An order one of those
                  --  swaps comes near is searched again as ever.
                  Kept_Told : Boolean := False;
               begin
                  if From.Valid and then From.N = N and then From.Order_Told then
                     declare
                        Own     : Rigid_Access := new Pose_Array;
                        Into    : Real_Access := new Real_Array (1 .. 2 * Multi);
                        M, E    : Real;
                        Swapped : Chain_Order (1 .. N);
                     begin
                        Score_Into (From.Chain, Own.all, Here_R.all, Here, E);
                        Kept_Told := Here < Real'Last;
                        for K in 1 .. N - 1 loop
                           exit when not Kept_Told;
                           Swapped := From.Chain;
                           Swapped (K) := From.Chain (K + 1);
                           Swapped (K + 1) := From.Chain (K);
                           Score_Into (Swapped, Own.all, Into.all, M, E);
                           Kept_Told := Smaller (Here_R (1 .. 2 * Placed), Into (1 .. 2 * Placed));
                        end loop;
                        Free (Own);
                        Free (Into);
                     end;
                  end if;
                  if Kept_Told then
                     Found := True;
                     Order := From.Chain;
                     Order_Is_Told := True;
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: the order of the joints along the arm,"
                                      & Image (Order) & ", told before, is told still against every swap of two joints"
                                      & " next to each other; no other order is searched");
                  else
                     Here := Real'Last;
                     Search_All (1, Ways);
                     for J in 1 .. Ways loop
                        if Results (J).Found and then (not Found or else Results (J).Median < Here) then
                           Found := True;
                           Order := Results (J).Order.all;
                           Here := Results (J).Median;
                           Here_R.all := Results (J).Residuals.all;
                        end if;
                     end loop;
                     --  An arm fitted before keeps the order it had (From.Chain) unless the least here leaves residuals
                     --  significantly the smaller (Smaller, the test that tells an order): a later estimate's orders can
                     --  all score alike, and the least of equals, the first of the sequence, then took the place of the
                     --  order the fits before had found (a19 on 10-16, at beat 7568: every order's median about 0.52 px
                     --  where the right one had left 0.17, the order 1 2 3 4 5 6 taken, and the arm's fit fell to focal
                     --  lengths of 130 and 145 px; s08's best order went back and forth between two, fit after fit).
                     if Found and then From.Valid and then From.N = N and then From.Chain /= Order then
                        declare
                           Own    : Rigid_Access := new Pose_Array;
                           Before : Real_Access := new Real_Array (1 .. 2 * Multi);
                           M, E   : Real;
                        begin
                           Score_Into (From.Chain, Own.all, Before.all, M, E);
                           if not Smaller (Here_R (1 .. 2 * Placed), Before (1 .. 2 * Placed)) then
                              Order := From.Chain;
                              Here := M;
                              Here_R.all := Before.all;
                           end if;
                           Free (Own);
                           Free (Before);
                        end;
                     end if;
                     Order_Is_Told := Here < Real'Last;
                     if Order_Is_Told then
                        Tell_All (1, Ways);
                        for J in 1 .. Ways loop
                           if Results (J).Fails then
                              Order_Is_Told := False;
                              Driver.Log.Line (Driver.Log.Robot, "kinematics: the order of the joints along the arm,"
                                               & Image (Order) & ", is not told: the order"
                                               & Image (Results (J).Fail_Order.all) & " leaves residuals of median "
                                               & Driver.Log.Image (Results (J).Fail_Median, 3) & " px against "
                                               & Driver.Log.Image (Here, 3)
                                               & " px, not significantly the larger by their signed ranks");
                              exit;
                           end if;
                        end loop;
                        if Order_Is_Told then
                           Driver.Log.Line (Driver.Log.Robot, "kinematics: the order of the joints along the arm,"
                                            & Image (Order) & ", is told: every other order searched leaves residuals"
                                            & " significantly the larger than its " & Driver.Log.Image (Here, 3) & " px");
                        end if;
                        --  An order not told: the joints next to each other along it whose swap leaves residuals not
                        --  significantly the larger (Smaller) are the ones it lacks the evidence of, and Boot turns them
                        --  together, a pair at a time. Two joints apart by a swap are told only by what they do turned
                        --  together: the orders differ in the product of the two turns, not in either alone. The order
                        --  nearest in residuals was taken for the pairs: s08's was the true order, two swaps away (5 with 1,
                        --  2 with 6), s10's another (5 4 3 1 2 6 for 5 1 4 3 2 6, where the true order was 1 5 4 3 6 2),
                        --  and s10 turned c1 with c4 and c1 with c3, its order still not told.
                        if not Order_Is_Told then
                           declare
                              Own     : Rigid_Access := new Pose_Array;
                              Into    : Real_Access := new Real_Array (1 .. 2 * Multi);
                              M, E    : Real;
                              Swapped : Chain_Order (1 .. N);
                              Said    : Ada.Strings.Unbounded.Unbounded_String;
                           begin
                              for K in 1 .. N - 1 loop
                                 Swapped := Order;
                                 Swapped (K) := Order (K + 1);
                                 Swapped (K + 1) := Order (K);
                                 Score_Into (Swapped, Own.all, Into.all, M, E);
                                 if not Smaller (Here_R (1 .. 2 * Placed), Into (1 .. 2 * Placed)) then
                                    Untold.Append (Joint_Pair'(Earlier => Order (K), Later => Order (K + 1)));
                                    Ada.Strings.Unbounded.Append
                                      (Said, " " & Driver.Log.Image (Order (K)) & " with " & Driver.Log.Image (Order (K + 1))
                                             & " (" & Driver.Log.Image (M, 3) & " px)");
                                 end if;
                              end loop;
                              Free (Own);
                              Free (Into);
                              Driver.Log.Line (Driver.Log.Robot, "kinematics: the joints next to each other along"
                                               & Image (Order) & " whose swap is not told:"
                                               & (if Untold.Is_Empty then " none"
                                                  else Ada.Strings.Unbounded.To_String (Said)));
                           end;
                        end if;
                     end if;
                  end if;
                  for J in 1 .. Ways loop
                     Free (Results (J).Order);
                     Free (Results (J).Residuals);
                     Free (Results (J).Fail_Order);
                  end loop;
               end;
               if Here >= Real'Last then
                  Free_All;
                  Fail ("no order of the joints puts the points the keyframes of one joint placed before the eye in "
                        & "the keyframes that moved several: the order of the joints along the arm is not told");
                  return;
               end if;
               Free_All;
            end;
         end;
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
      declare
         Per_Joint : constant := 6;
         Count     : constant Positive := Per_Joint * N;
         Inlier    : Flags_Access := new Flags'(1 .. S => False);
         Base      : Joint_Array := Joints;
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
            Poses_Of (Jx, Poses.all);
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
         To_X (Joints, X);
         loop
            declare
               All_R : Real_Access := new Real_Array (1 .. S);
               Sigma : Real;
               Now_Changed : Natural := 0;
               Used  : Natural := 0;
            begin
               Residuals_Of (Lens0, Joints_Of (X), All_R.all);
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
               Report.Sigma_Px := Sigma;
               Report.Used := Used;
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
                  Focal     : constant Real := (Lens0.Fx + Lens0.Fy) / 2.0;

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
                     Poses_Of (Joints_Of (Xv), Poses.all);
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
                     Own_Lines (I) := Ray (Lens0, Kept (I).U, Kept (I).V);
                     Ref_Lines (I) := Ray (Lens0, Kept (I).U0, Kept (I).V0);
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
         L := Lens0;
         Joints := Joints_Of (X);
         Normalize (Joints);
         declare
            All_R : Real_Access := new Real_Array (1 .. S);
            Fit_R : Real_Access := new Real_Array (1 .. S);
            K     : Natural := 0;
         begin
            Residuals_Of (L, Joints, All_R.all);
            for I in 1 .. S loop
               if Inlier (I) then
                  K := K + 1;
                  Fit_R (K) := abs All_R (I);
               end if;
            end loop;
            Report.Median_Px := 0.0;
            if K > 0 then
               declare
                  Part : Real_Access := new Real_Array'(Fit_R (1 .. K));
               begin
                  Report.Median_Px := Driver.Stats.Median (Part.all);
                  Free (Part);
               end;
            end if;
            Free (All_R);
            Free (Fit_R);
         end;
         Free (Inlier);
      end;

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
            Eigensystem (Mt_M, Values, Vectors);
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

end Driver.Robot.Kinematics.Fit;
