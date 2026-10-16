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

      function Median_Ratio (Floor : Real) return Real is
         Ratios : Real_Access := new Real_Array (1 .. R'Length);
         Result : Real;
      begin
         for I in 1 .. R'Length loop
            Ratios (I) := abs R (R'First + I - 1) / Scale_Of (Floor, Own (Own'First + I - 1));
         end loop;
         Result := Driver.Stats.Median (Ratios.all);
         Free (Ratios);
         return Result;
      end Median_Ratio;
   begin
      if R'Length = 0 or else High <= 0.0 then
         return 0.0;
      end if;
      if (for all O of Own => O = 0.0) then
         return High;
      end if;
      if Median_Ratio (0.0) <= Typical then
         return 0.0;
      end if;
      --  The ratio falls as the floor rises, from above the Gaussian's at none to below it at High.
      for Step in 1 .. Real'Machine_Mantissa loop
         declare
            Middle : constant Real := (Low + Top) / 2.0;
         begin
            exit when Middle <= Low or else Middle >= Top;
            if Median_Ratio (Middle) > Typical then
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
            for I in 1 .. Residuals loop
               declare
                  W : constant Real := Huber (R0 (I) / Sigma) / Sigma ** 2;
               begin
                  for P in 1 .. Parameters loop
                     if J (I, P) /= 0.0 then
                        G (P) := G (P) - W * J (I, P) * R0 (I);
                        for Q in P .. Parameters loop
                           A (P, Q) := A (P, Q) + W * J (I, P) * J (I, Q);
                        end loop;
                     end if;
                  end loop;
               end;
            end loop;
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

         --  The reprojection residuals of the chosen sightings (2 each).
         procedure Residuals (Xv : Real_Array; Dv : Real_Array; Index : Real_Access; R : out Real_Array) is
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
         begin
            for F in 1 .. Frames loop
               Views (F) := Inverse (Eye_At (Jx, Chain, Changes_Of (F)));
            end loop;
            for K in 1 .. Index'Length loop
               declare
                  Sg : Sighting renames Sight (Sight'First + Natural (Index (K)) - 1);
                  U, V : Real;
                  Ahead : Boolean;
               begin
                  if not Made (Sg.Track) or else Made_U (Sg.Track) /= Sg.U0 or else Made_V (Sg.Track) /= Sg.V0 then
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
               end;
            end loop;
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
                           Rp : Real_Access := new Real_Array (1 .. 2 * Used);
                        begin
                           Xp (P) := Xp (P) + H;
                           Residuals (Xp, Depth.all, Index, Rp.all);
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
                        for I in 1 .. 2 * Used loop
                           declare
                              W : constant Real := Huber (R0 (I) / Scales (I)) / Scales (I) ** 2;
                              T : constant Positive := Sight (Sight'First + Natural (Index ((I + 1) / 2)) - 1).Track;
                           begin
                              for P in 1 .. Count loop
                                 if Jp (I, P) /= 0.0 then
                                    Gp (P) := Gp (P) - W * Jp (I, P) * R0 (I);
                                    Bm (P, T) := Bm (P, T) + W * Jp (I, P) * Jd (I);
                                    for Q in P .. Count loop
                                       A (P, Q) := A (P, Q) + W * Jp (I, P) * Jp (I, Q);
                                    end loop;
                                 end if;
                              end loop;
                              C (T) := C (T) + W * Jd (I) ** 2;
                              Gd (T) := Gd (T) - W * Jd (I) * R0 (I);
                           end;
                        end loop;
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
                                 if Cl (T) > 0.0 then
                                    for P in 1 .. Count loop
                                       if Bm (P, T) /= 0.0 then
                                          Rhs (P) := Rhs (P) - Bm (P, T) * Gd (T) / Cl (T);
                                          for Q in 1 .. Count loop
                                             Sm (P, Q) := Sm (P, Q) - Bm (P, T) * Bm (Q, T) / Cl (T);
                                          end loop;
                                       end if;
                                    end loop;
                                 end if;
                              end loop;
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
                  for Way in Sign_Way loop
                     declare
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
                                       L := Lp;
                                       Joints := Jp;
                                       Report := Rp;
                                       Moved := True;
                                    end if;
                                    exit;
                                 end if;
                                 exit when Rp.All_Median_Px > Base + Driver.Conventions.Z * Err;
                              end;
                              Offset := 2.0 * Offset;
                           end;
                        end loop;
                     end;
                     exit when Moved;
                  end loop;
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
                        if Count >= 3 then
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
                        if Told then
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
                              Reprojected (Rr_Of (Way), T0_Of (Way), L (I).H0, L (I).H, F, Du (M), Dv (M));
                           end if;
                        end loop;
                        Share_Out (Du (1 .. M));
                        for Q in 1 .. M loop
                           Filled := Filled + 1;
                           Fits (Filled) := Sqrt (Du (Q) ** 2 + Dv (Q) ** 2);
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
               begin
                  for X of Best loop
                     Sum := Sum + X.Score;
                  end loop;
                  Ada.Strings.Unbounded.Append (Trace, " " & Driver.Log.Image (F, 1) & ":" & Driver.Log.Image (Sum, 3));
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
               Poses  : Rigid_Access := new Pose_Array;
               Placed : Natural := 0;   --  sightings of placed points in those keyframes

               --  The median across and down of the reprojections of those sightings with the joints in order C,
               --  in pixels, and its standard error. A point the order puts behind the eye lands nowhere.
               procedure Score (C : Chain_Order; Median, Error : out Real) is
                  K : Natural := 0;
               begin
                  for Frame in 1 .. Frames loop
                     Poses (Frame) := Eye_At (Joints, C, Changes_Of (Frame));
                  end loop;
                  for X of Sight loop
                     if Several (X.Frame) and then Depth (X.Track) > 0.0 then
                        declare
                           T     : Rigid renames Poses (X.Frame);
                           Point : constant Vec3 := Depth (X.Track) * Ray (Lens0, X.U0, X.V0);
                           U, V  : Real;
                           Ahead : Boolean;
                        begin
                           Project (Lens0, Transpose (T.Rotation) * (Point - T.Translation), U, V, Ahead);
                           R (K + 1) := (if Ahead then U - X.U else Real'Last);
                           R (K + 2) := (if Ahead then V - X.V else Real'Last);
                           K := K + 2;
                        end;
                     end if;
                  end loop;
                  Median := Median_Abs (R (1 .. K));
                  --  The median point lands behind the eye: no residual, and none to lower by a standard error.
                  if Median >= Real'Last then
                     Median := Real'Last;
                     Error := 0.0;
                  else
                     Error := Median_Error (Median, K);
                  end if;
               end Score;

               --  C with the joint at place From moved to place To, the joints between shifted over.
               function Moved (C : Chain_Order; From, To : Positive) return Chain_Order is
                  Result : Chain_Order := C;
               begin
                  if From < To then
                     Result (From .. To - 1) := C (From + 1 .. To);
                  else
                     Result (To + 1 .. From) := C (To .. From - 1);
                  end if;
                  Result (To) := C (From);
                  return Result;
               end Moved;

               procedure Free_All is
               begin
                  Free (Depth);
                  Free (R);
                  Free (Poses);
               end Free_All;

               Here, Here_Error : Real;
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
               --  The start: the axes farthest from the eye first, as they most
               --  often are on an arm that carries its eye at the end; the
               --  search does not rest on it.
               for K in 2 .. N loop
                  declare
                     Item : constant Positive := Order (K);
                     I    : Natural := K - 1;
                  begin
                     while I >= 1
                       and then abs Joints (Joints'First + Order (I) - 1).P < abs Joints (Joints'First + Item - 1).P
                     loop
                        Order (I + 1) := Order (I);
                        I := I - 1;
                     end loop;
                     Order (I + 1) := Item;
                  end;
               end loop;
               Score (Order, Here, Here_Error);
               --  A joint moved to the place that lowers the residual most, while
               --  that lowers it by more than its standard error.
               loop
                  declare
                     Best_Order : Chain_Order (1 .. N) := Order;
                     Best       : Real := Here;
                     Best_Error : Real := Here_Error;
                  begin
                     for From in 1 .. N loop
                        for To in 1 .. N loop
                           if To /= From then
                              declare
                                 C    : constant Chain_Order := Moved (Order, From, To);
                                 M, E : Real;
                              begin
                                 Score (C, M, E);
                                 if M < Best then
                                    Best := M;
                                    Best_Error := E;
                                    Best_Order := C;
                                 end if;
                              end;
                           end if;
                        end loop;
                     end loop;
                     --  Written so that it ends on a score that is no number: the median of residuals of which half
                     --  or more lay behind the eye was the mean of the two largest reals, infinite, and the
                     --  difference of two of them none, which no comparison stopped (f15c replayed on 10-15: the
                     --  search went on for ten minutes at one order).
                     exit when not (Best < Here - Here_Error);
                     Order := Best_Order;
                     Here := Best;
                     Here_Error := Best_Error;
                  end;
               end loop;
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

      --  Then everything at once, on the sightings that fit. Parameters:
      --  log Fx, log Fy, Cx, Cy, K1, K2, then per joint the axis's tilt in its
      --  tangent plane (2), its point (3) and its reading scale (1).
      declare
         Per_Joint : constant := 6;
         Count     : constant Positive := 6 + Per_Joint * N;
         Inlier    : Flags_Access := new Flags'(1 .. S => False);
         Base      : Joint_Array := Joints;
         --  The choices of sightings made so far: a round that comes back to one of them is a cycle.
         package Selection_Lists is new Ada.Containers.Vectors (Positive, Flags_Access);
         procedure Free_Flags is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
         Seen      : Selection_Lists.Vector;

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
         To_X (Lens0, Joints, X);
         loop
            declare
               All_R : Real_Access := new Real_Array (1 .. S);
               Sigma : Real;
               Now_Changed : Natural := 0;
               Used  : Natural := 0;
            begin
               Residuals_Of (Lens_Of (X), Joints_Of (X), All_R.all);
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

                  procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
                     Lx         : constant Lens := Lens_Of (Xv);
                     Focal_Here : constant Real := (Lx.Fx + Lx.Fy) / 2.0;
                     Poses      : Rigid_Access := new Pose_Array;
                     --  Per keyframe, its motion from the reference eye; per track, the line of sight of its reference
                     --  pixel, and the pixel: made once, not once a sighting (Refine_Tracks' Residuals says why).
                     type Motion is record
                        Rr : Mat3;
                        Tt : Vec3;
                     end record;
                     type Motion_Array is array (1 .. Frames) of Motion;
                     type Motion_Access is access Motion_Array;
                     procedure Free is new Ada.Unchecked_Deallocation (Motion_Array, Motion_Access);
                     Motions     : Motion_Access := new Motion_Array;
                     Track_Count : Natural := 0;
                  begin
                     Poses_Of (Joints_Of (Xv), Poses.all);
                     for Frame in 1 .. Frames loop
                        Relative (Poses (1), Poses (Frame), Motions (Frame).Rr, Motions (Frame).Tt);
                     end loop;
                     for I in 1 .. Used loop
                        Track_Count := Natural'Max (Track_Count, Kept (I).Track);
                     end loop;
                     declare
                        type Ray_Array is array (1 .. Track_Count) of Vec3;
                        type Ray_Access is access Ray_Array;
                        procedure Free is new Ada.Unchecked_Deallocation (Ray_Array, Ray_Access);
                        Rays   : Ray_Access := new Ray_Array;
                        Made   : Flags_Access := new Flags'(1 .. Track_Count => False);
                        Made_U : Real_Access := new Real_Array (1 .. Track_Count);
                        Made_V : Real_Access := new Real_Array (1 .. Track_Count);
                     begin
                        for I in 1 .. Used loop
                           declare
                              X : Sighting renames Kept (I);
                           begin
                              if not Made (X.Track) or else Made_U (X.Track) /= X.U0 or else Made_V (X.Track) /= X.V0
                              then
                                 Rays (X.Track) := Ray (Lx, X.U0, X.V0);
                                 Made (X.Track) := True;
                                 Made_U (X.Track) := X.U0;
                                 Made_V (X.Track) := X.V0;
                              end if;
                              R (R'First + I - 1) := Sampson (Motions (X.Frame).Rr, Motions (X.Frame).Tt, Rays (X.Track),
                                                              Ray (Lx, X.U, X.V), Focal_Here);
                           end;
                        end loop;
                        Free (Rays);
                        Free (Made);
                        Free (Made_U);
                        Free (Made_V);
                     end;
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
                  Solve (X, Sigma);
                  Free (Kept);
                  --  The axes' tangent planes anew at the solution, the points
                  --  on them nearest the reference eye.
                  Base := Joints_Of (X);
                  for J of Base loop
                     J.P := J.P - Dot (J.P, J.W) * J.W;
                  end loop;
                  To_X (Lens_Of (X), Base, X);
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
         L := Lens_Of (X);
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

   function On_Plane (P : Sight_Plane; H : Vec3) return Vec3 is
      Q : constant Real := P.A * H;
   begin
      return (if Q > 0.0 then (1.0 / Q) * H else [0.0, 0.0, 0.0]);
   end On_Plane;

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

   --  The median of the lengths of two-coordinate residuals of Gaussian noise
   --  is sigma times the root of 2 ln 2.
   function Rayleigh_Median return Real is (Sqrt (2.0 * Ln (2.0)));

   --  H applied to a plane point; Ahead is False on the line at infinity.
   procedure Apply (H : Mat3; X, Y : Real; U, V : out Real; Ahead : out Boolean) is
      W : constant Real := H (3, 1) * X + H (3, 2) * Y + H (3, 3);
   begin
      Ahead := W /= 0.0;
      U := (if Ahead then (H (1, 1) * X + H (1, 2) * Y + H (1, 3)) / W else 0.0);
      V := (if Ahead then (H (2, 1) * X + H (2, 2) * Y + H (2, 3)) / W else 0.0);
   end Apply;

   function Residual_Length (H : Mat3; P : Plane_Point) return Real is
      U, V  : Real;
      Ahead : Boolean;
   begin
      Apply (H, P.X, P.Y, U, V, Ahead);
      return (if Ahead then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) else Real'Last);
   end Residual_Length;

   --  The eight free entries of a homography whose last entry is one.
   function Homography_Of (X : Real_Array) return Mat3 is
     ([[X (X'First), X (X'First + 1), X (X'First + 2)],
       [X (X'First + 3), X (X'First + 4), X (X'First + 5)],
       [X (X'First + 6), X (X'First + 7), 1.0]]);

   function Entries_Of (H : Mat3) return Real_Array is
     ([H (1, 1), H (1, 2), H (1, 3), H (2, 1), H (2, 2), H (2, 3), H (3, 1), H (3, 2)]);

   --  The homography through the points Use_It selects, by the direct linear
   --  transform on coordinates centred and scaled to unit spread (Hartley's
   --  conditioning), with its last entry made one.
   function Linear_Homography (Points : Plane_Point_Array; Use_It : Flag_Array) return Mat3 is
      Cx, Cy, Cu, Cv, Sxy, Suv : Real := 0.0;
      N : Natural := 0;
   begin
      for I in Points'Range loop
         if Use_It (I) then
            N := N + 1;
            Cx := Cx + Points (I).X;
            Cy := Cy + Points (I).Y;
            Cu := Cu + Points (I).U;
            Cv := Cv + Points (I).V;
         end if;
      end loop;
      Cx := Cx / Real (N);
      Cy := Cy / Real (N);
      Cu := Cu / Real (N);
      Cv := Cv / Real (N);
      for I in Points'Range loop
         if Use_It (I) then
            Sxy := Sxy + Sqrt ((Points (I).X - Cx) ** 2 + (Points (I).Y - Cy) ** 2);
            Suv := Suv + Sqrt ((Points (I).U - Cu) ** 2 + (Points (I).V - Cv) ** 2);
         end if;
      end loop;
      Sxy := (if Sxy > 0.0 then Real (N) / Sxy else 1.0);
      Suv := (if Suv > 0.0 then Real (N) / Suv else 1.0);
      declare
         A       : Real_Matrix (1 .. 9, 1 .. 9) := [others => [others => 0.0]];
         Values  : Real_Vector (1 .. 9);
         Vectors : Real_Matrix (1 .. 9, 1 .. 9);
         Least   : Positive := 1;
         Hn      : Mat3;
      begin
         for I in Points'Range loop
            if Use_It (I) then
               declare
                  X  : constant Real := (Points (I).X - Cx) * Sxy;
                  Y  : constant Real := (Points (I).Y - Cy) * Sxy;
                  U  : constant Real := (Points (I).U - Cu) * Suv;
                  V  : constant Real := (Points (I).V - Cv) * Suv;
                  R1 : constant Real_Vector (1 .. 9) := [X, Y, 1.0, 0.0, 0.0, 0.0, -U * X, -U * Y, -U];
                  R2 : constant Real_Vector (1 .. 9) := [0.0, 0.0, 0.0, X, Y, 1.0, -V * X, -V * Y, -V];
               begin
                  for P in 1 .. 9 loop
                     for Q in 1 .. 9 loop
                        A (P, Q) := A (P, Q) + R1 (P) * R1 (Q) + R2 (P) * R2 (Q);
                     end loop;
                  end loop;
               end;
            end if;
         end loop;
         Eigensystem (A, Values, Vectors);
         for K in 2 .. 9 loop
            if Values (K) < Values (Least) then
               Least := K;
            end if;
         end loop;
         for R in 1 .. 3 loop
            for C in 1 .. 3 loop
               Hn (R, C) := Vectors (3 * (R - 1) + C, Least);
            end loop;
         end loop;
         --  Undo the conditioning: H = Tuv^-1 Hn Txy.
         declare
            Tuv_Inv : constant Mat3 := [[1.0 / Suv, 0.0, Cu], [0.0, 1.0 / Suv, Cv], [0.0, 0.0, 1.0]];
            Txy     : constant Mat3 := [[Sxy, 0.0, -Cx * Sxy], [0.0, Sxy, -Cy * Sxy], [0.0, 0.0, 1.0]];
            H       : constant Mat3 := Tuv_Inv * Hn * Txy;
         begin
            return (if H (3, 3) /= 0.0 then (1.0 / H (3, 3)) * H else H);
         end;
      end;
   end Linear_Homography;

   --  The homography (its eight free entries, X) by robust least squares at
   --  Sigma on the points Fits selects; Sigma becomes their noise. Not
   --  inlined: inlined into its caller (GNAT 16.1 at -O2), the nested
   --  Evaluate reads past the frame its up-level variables are given, which
   --  AddressSanitizer reports.
   procedure Refine_Homography
     (Points : Plane_Point_Array;
      Fits   : Flag_Array;
      X      : in out Real_Array;
      Sigma  : in out Real)
     with No_Inline
   is
      Used  : Natural := 0;
      Index : Count_Access;
   begin
      for I in Points'Range loop
         if Fits (I) then
            Used := Used + 1;
         end if;
      end loop;
      Index := new Count_Array (1 .. Used);
      declare
         K : Natural := 0;
      begin
         for I in Points'Range loop
            if Fits (I) then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
      end;
      declare
         procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
            Hv : constant Mat3 := Homography_Of (Xv);
         begin
            for J in 1 .. Used loop
               declare
                  P     : Plane_Point renames Points (Index (J));
                  U, V  : Real;
                  Ahead : Boolean;
               begin
                  Apply (Hv, P.X, P.Y, U, V, Ahead);
                  R (R'First + 2 * J - 2) := (if Ahead then U - P.U else Real'Last);
                  R (R'First + 2 * J - 1) := (if Ahead then V - P.V else Real'Last);
               end;
            end loop;
         end Evaluate;

         procedure Solve is new Robust_Fit (8, 2 * Used, Real'Model_Epsilon, Evaluate, At_Once => True);
         R : Real_Access := new Real_Array (1 .. 2 * Used);
      begin
         Solve (X, Sigma);
         Evaluate (X, R.all);
         Sigma := Noise_Of (R.all);
         Free (R);
      end;
      Free (Index);
   end Refine_Homography;

   procedure Plane_Homography
     (Points : Plane_Point_Array;
      H      : out Mat3;
      Fits   : out Flag_Array;
      Sigma  : out Real;
      Found  : out Boolean)
   is
      N       : constant Natural := Points'Length;
      Minimal : constant := 4;
      Best    : Real := Real'Last;
   begin
      H := Identity3;
      Fits := [others => False];
      Sigma := Real'Last;
      Found := False;
      if N <= Minimal then
         return;
      end if;
      --  Consensus: the least median, so up to half the points may be
      --  anything at all.
      declare
         Lengths : Real_Access := new Real_Array (1 .. N);
         Sm      : Sampler;
      begin
         for Sample in 1 .. Consensus_Samples (Minimal, 0.5) loop
            declare
               Index  : Count_Array (1 .. Minimal);
               Use_It : Flag_Array (Points'Range) := [others => False];
               Hs     : Mat3;
            begin
               Pick (Sm, N, Index);
               for K of Index loop
                  Use_It (Points'First + K - 1) := True;
               end loop;
               Hs := Linear_Homography (Points, Use_It);
               for I in 1 .. N loop
                  Lengths (I) := Residual_Length (Hs, Points (Points'First + I - 1));
               end loop;
               declare
                  Median : constant Real := Driver.Stats.Median (Lengths.all);
               begin
                  if Median < Best then
                     Best := Median;
                     H := Hs;
                  end if;
               end;
            end;
         end loop;
         Free (Lengths);
      end;
      if Best = 0.0 or else Best = Real'Last then
         return;
      end if;
      --  From the consensus: the points within Z of its noise, the homography
      --  by robust least squares on them, the choice renewed at the noise of
      --  the points that fit until it settles.
      Sigma := Best / Rayleigh_Median;
      declare
         X       : Real_Array (1 .. 8) := Entries_Of (H);
         Changed : Natural := Natural'Last;
         Used    : Natural;
      begin
         loop
            declare
               Hx          : constant Mat3 := Homography_Of (X);
               Now_Changed : Natural := 0;
            begin
               Used := 0;
               for I in Points'Range loop
                  declare
                     U, V  : Real;
                     Ahead : Boolean;
                     Fit   : Boolean;
                  begin
                     Apply (Hx, Points (I).X, Points (I).Y, U, V, Ahead);
                     Fit := Ahead and then not Driver.Uncertain.Significant (U - Points (I).U, Sigma)
                       and then not Driver.Uncertain.Significant (V - Points (I).V, Sigma);
                     if Fit /= Fits (I) then
                        Now_Changed := Now_Changed + 1;
                        Fits (I) := Fit;
                     end if;
                     if Fit then
                        Used := Used + 1;
                     end if;
                  end;
               end loop;
               exit when Used <= Minimal or else Now_Changed = 0 or else Now_Changed >= Changed;
               Changed := Now_Changed;
               Refine_Homography (Points, Fits, X, Sigma);
               exit when Sigma <= 0.0;
            end;
         end loop;
         H := Homography_Of (X);
         Found := Used > Minimal and then Sigma > 0.0 and then Sigma < Real'Last;
      end;
   end Plane_Homography;

   ---------------------------------------------------------------------------
   --  The joint fit of a link

   function Lens_Terms_Of (L : Lens) return Real_Array is ([Ln (L.Fx), Ln (L.Fy), L.Cx, L.Cy, L.K1, L.K2]);

   function Lens_Of_Terms (T : Real_Array) return Lens is
     ((Fx => Exp (T (T'First)), Fy => Exp (T (T'First + 1)), Cx => T (T'First + 2), Cy => T (T'First + 3),
       K1 => T (T'First + 4), K2 => T (T'First + 5)));

   --  A whitening of a covariance C: W with W C W^T the identity (W = L^-1
   --  for C = L L^T), so that |W d| is the change d in standard deviations.
   --  The diagonal is first raised by the float's resolution of its largest
   --  entry, so that a term known exactly is held, not lost. All zero when C
   --  is no covariance: then nothing is held.
   function Whitening (C : Real_Matrix) return Real_Matrix is
      N   : constant Natural := C'Length (1);
      Cf  : Real_Matrix (1 .. N, 1 .. N);
      Lf  : Real_Matrix (1 .. N, 1 .. N);
      W   : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      Top : Real := 0.0;
      Pd  : Boolean;
   begin
      for P in 1 .. N loop
         for Q in 1 .. N loop
            Cf (P, Q) := C (C'First (1) + P - 1, C'First (2) + Q - 1);
         end loop;
         Top := Real'Max (Top, abs Cf (P, P));
      end loop;
      for P in 1 .. N loop
         Cf (P, P) := Cf (P, P) + Real'Model_Epsilon * Real'Max (Top, Real'Model_Small);
      end loop;
      Driver.Numerics.Dense.Cholesky (Cf, Lf, Pd);
      if not Pd then
         return W;
      end if;
      --  L^-1 column by column, by forward substitution.
      for K in 1 .. N loop
         declare
            Z : Real_Vector (1 .. N) := [others => 0.0];
         begin
            for R in 1 .. N loop
               declare
                  S : Real := (if R = K then 1.0 else 0.0);
               begin
                  for Q in 1 .. R - 1 loop
                     S := S - Lf (R, Q) * Z (Q);
                  end loop;
                  Z (R) := S / Lf (R, R);
               end;
            end loop;
            for R in 1 .. N loop
               W (R, K) := Z (R);
            end loop;
         end;
      end loop;
      return W;
   end Whitening;

   type Vec3_Array is array (Positive range <>) of Vec3;

   --  One side as the joint fit uses it: its tracks and kept answers, its
   --  estimates (lens terms, plane) and how they are held.
   type Side_Data (Tracks, Answers : Natural) is record
      U0, V0, Depth, Sigma : Real_Array (1 .. Tracks);
      Track      : Count_Array (1 .. Answers);   --  per answer, its track
      U, V       : Real_Array (1 .. Answers);
      Terms0     : Real_Array (1 .. Lens_Terms);
      W_Lens     : Real_Matrix (1 .. Lens_Terms, 1 .. Lens_Terms);
      Plane0     : Sight_Plane;                  --  through the tracks, at the estimated lens
      W_Plane    : Mat3;
      E1_0, E2_0 : Vec3;                         --  its axes at the estimates
      Rays       : Vec3_Array (1 .. Tracks);     --  work: the tracks' lines of sight
      Sights     : Sight_Point_Array (1 .. Tracks);
      All_On     : Flag_Array (1 .. Tracks);
   end record;

   type Side_Access is access Side_Data;
   procedure Free is new Ada.Unchecked_Deallocation (Side_Data, Side_Access);

   function Prepare (S : Chain_Side) return Side_Access is
      D : constant Side_Access :=
        new Side_Data (Tracks => Natural (S.Tracks.Length), Answers => Natural (S.Answers.Length));
      Normal : Vec3;
   begin
      for T in 1 .. D.Tracks loop
         declare
            Tr : constant Table_Track := S.Tracks (S.Tracks.First_Index + T - 1);
         begin
            D.U0 (T) := Tr.U0;
            D.V0 (T) := Tr.V0;
            D.Depth (T) := Tr.Depth;
            D.Sigma (T) := Tr.Sigma;
         end;
      end loop;
      D.All_On := [others => True];
      for A in 1 .. D.Answers loop
         declare
            An : constant Eye_Answer := S.Answers (S.Answers.First_Index + A - 1);
         begin
            D.Track (A) := An.Track;
            D.U (A) := An.U;
            D.V (A) := An.V;
         end;
      end loop;
      D.Terms0 := Lens_Terms_Of (S.L);
      declare
         C : Real_Matrix (1 .. Lens_Terms, 1 .. Lens_Terms) := [others => [others => 0.0]];
      begin
         if Natural (S.Lens_Covariance.Length) = Lens_Terms * Lens_Terms then
            for P in 1 .. Lens_Terms loop
               for Q in 1 .. Lens_Terms loop
                  C (P, Q) := S.Lens_Covariance (S.Lens_Covariance.First_Index + (P - 1) * Lens_Terms + Q - 1);
               end loop;
            end loop;
         end if;
         D.W_Lens := Whitening (C);
      end;
      D.Plane0 := Side_Plane (S, S.L);
      D.W_Plane := Whitening (S.Plane_Covariance);
      Plane_Axes (D.Plane0, Normal, D.E1_0, D.E2_0);
      return D;
   end Prepare;

   function Side_Plane (S : Chain_Side; L : Lens) return Sight_Plane is
      N      : constant Natural := Natural (S.Tracks.Length);
      Sights : Sight_Point_Array (1 .. N);
      On     : constant Flag_Array (1 .. N) := [others => True];
      P      : Sight_Plane;
   begin
      for T in 1 .. N loop
         declare
            Tr : constant Table_Track := S.Tracks (S.Tracks.First_Index + T - 1);
         begin
            Sights (T) := (H => Ray (L, Tr.U0, Tr.V0), Depth => Tr.Depth, Sigma => Tr.Sigma);
         end;
      end loop;
      Refit_Plane (Sights, On, P);
      return P;
   end Side_Plane;

   --  The side's plane and axes with its lens terms T and its plane moved by
   --  Move: the tracks' lines of sight through that lens (D.Rays), the plane
   --  through them with their depths held, moved; its axes those at the
   --  estimates turned onto it.
   procedure Side_State (D : in out Side_Data; T : Real_Array; Move : Vec3; P : out Sight_Plane; E1, E2 : out Vec3) is
      L : constant Lens := Lens_Of_Terms (T);
   begin
      for K in 1 .. D.Tracks loop
         D.Rays (K) := Ray (L, D.U0 (K), D.V0 (K));
         D.Sights (K) := (H => D.Rays (K), Depth => D.Depth (K), Sigma => D.Sigma (K));
      end loop;
      Refit_Plane (D.Sights, D.All_On, P);
      P.A := P.A + Move;
      declare
         N    : constant Vec3 := Plane_Normal (P);
         Flat : constant Vec3 := D.E1_0 - Real'(D.E1_0 * N) * N;
      begin
         E1 := (if abs Flat > 0.0 then Unit (Flat) else D.E1_0);
         E2 := Cross (N, E1);
      end;
   end Side_State;

   --  Where the line of sight H meets plane P, along the axes E1, E2.
   procedure Coordinates (P : Sight_Plane; E1, E2 : Vec3; H : Vec3; X, Y : out Real) is
      Q : constant Vec3 := On_Plane (P, H);
   begin
      X := Q * E1;
      Y := Q * E2;
   end Coordinates;

   --  The positions of the joint parameters.
   S_First    : constant := 9;                          --  log scale, turn, two shifts
   Side_First : constant array (1 .. 2) of Positive := [S_First + Similarity_Terms,
                                                        S_First + Similarity_Terms + Side_Terms];

   function Terms_Of (Xv : Real_Array; Side : Positive) return Real_Array is
     (Xv (Xv'First + Side_First (Side) - 1 .. Xv'First + Side_First (Side) + Lens_Terms - 2));

   function Move_Of (Xv : Real_Array; Side : Positive) return Vec3 is
     ([Xv (Xv'First + Side_First (Side) + Lens_Terms - 1), Xv (Xv'First + Side_First (Side) + Lens_Terms),
       Xv (Xv'First + Side_First (Side) + Lens_Terms + 1)]);

   --  The similarity of the joint parameters applied to (X, Y).
   procedure Similar (Xv : Real_Array; X, Y : in out Real) is
      Sc : constant Real := Exp (Xv (Xv'First + S_First - 1));
      Th : constant Real := Xv (Xv'First + S_First);
      Px : constant Real := X;
   begin
      X := Sc * (Cos (Th) * Px - Sin (Th) * Y) + Xv (Xv'First + S_First + 1);
      Y := Sc * (Sin (Th) * Px + Cos (Th) * Y) + Xv (Xv'First + S_First + 2);
   end Similar;

   --  The similarity's inverse applied to (X, Y): the first plane's
   --  coordinates to the second's.
   procedure Unsimilar (Xv : Real_Array; X, Y : in out Real) is
      Sc : constant Real := Exp (Xv (Xv'First + S_First - 1));
      Th : constant Real := Xv (Xv'First + S_First);
      Dx : constant Real := X - Xv (Xv'First + S_First + 1);
      Dy : constant Real := Y - Xv (Xv'First + S_First + 2);
   begin
      X := (Cos (Th) * Dx + Sin (Th) * Dy) / Sc;
      Y := (-Sin (Th) * Dx + Cos (Th) * Dy) / Sc;
   end Unsimilar;

   --  Every residual of the joint fit at Xv, each in units of its own noise:
   --  the first side's kept answers (across and down), then the second's
   --  unless Second_Eye, then how far each side's lens and plane moved, in
   --  standard deviations. With Second_Eye the first side's answers are where
   --  the second side's own eye sees its points: through the similarity back
   --  onto the second plane, then through the second lens; otherwise the
   --  first plane's homography takes each side's coordinates (the second's
   --  through the similarity) into the eye.
   procedure Chain_Residuals
     (D1, D2           : in out Side_Data;
      Kept1, Kept2     : Flag_Array;
      Second_Eye       : Boolean;
      Sigma_1, Sigma_2 : Real;
      Xv               : Real_Array;
      R                : out Real_Array)
   is
      H  : constant Mat3 := Homography_Of (Xv (Xv'First .. Xv'First + 7));
      K  : Natural := R'First - 1;
      P1, P2 : Sight_Plane;
      A1, B1, A2, B2 : Vec3;

      procedure Put (U, V : Real; Ahead : Boolean; Obs_U, Obs_V, Sigma : Real) is
      begin
         R (K + 1) := (if Ahead then (U - Obs_U) / Sigma else Real'Last);
         R (K + 2) := (if Ahead then (V - Obs_V) / Sigma else Real'Last);
         K := K + 2;
      end Put;

      procedure Hold (D : Side_Data; Which : Positive) is
         Dt : Real_Vector (1 .. Lens_Terms);
         T  : constant Real_Array := Terms_Of (Xv, Which);
         Wl : Real_Vector (1 .. Lens_Terms);
         Wp : Vec3;
      begin
         for P in 1 .. Lens_Terms loop
            Dt (P) := T (T'First + P - 1) - D.Terms0 (P);
         end loop;
         Wl := D.W_Lens * Dt;
         Wp := D.W_Plane * Move_Of (Xv, Which);
         for P in 1 .. Lens_Terms loop
            R (K + P) := Wl (P);
         end loop;
         for P in 1 .. 3 loop
            R (K + Lens_Terms + P) := Wp (P);
         end loop;
         K := K + Side_Terms;
      end Hold;
   begin
      Side_State (D1, Terms_Of (Xv, 1), Move_Of (Xv, 1), P1, A1, B1);
      Side_State (D2, Terms_Of (Xv, 2), Move_Of (Xv, 2), P2, A2, B2);
      for A in 1 .. D1.Answers loop
         if Kept1 (A) then
            declare
               X, Y, U, V : Real;
               Ahead : Boolean;
            begin
               Coordinates (P1, A1, B1, D1.Rays (D1.Track (A)), X, Y);
               if Second_Eye then
                  Unsimilar (Xv, X, Y);
                  Project (Lens_Of_Terms (Terms_Of (Xv, 2)),
                           X * A2 + Y * B2 + Plane_Offset (P2) * Plane_Normal (P2), U, V, Ahead);
               else
                  Apply (H, X, Y, U, V, Ahead);
               end if;
               Put (U, V, Ahead, D1.U (A), D1.V (A), Sigma_1);
            end;
         end if;
      end loop;
      if not Second_Eye then
         for A in 1 .. D2.Answers loop
            if Kept2 (A) then
               declare
                  X, Y, U, V : Real;
                  Ahead : Boolean;
               begin
                  Coordinates (P2, A2, B2, D2.Rays (D2.Track (A)), X, Y);
                  Similar (Xv, X, Y);
                  Apply (H, X, Y, U, V, Ahead);
                  Put (U, V, Ahead, D2.U (A), D2.V (A), Sigma_2);
               end;
            end if;
         end loop;
      end if;
      Hold (D1, 1);
      Hold (D2, 2);
   end Chain_Residuals;

   --  The similarity nearest a map of the plane (Umeyama's closed form in two
   --  dimensions) from its point pairs: To = Scale Rot (Turn) From + Shift.
   procedure Similarity_2D
     (From_X, From_Y, To_X, To_Y : Real_Array;
      Scale, Turn, Shift_X, Shift_Y : out Real;
      Found : out Boolean)
     with Pre => From_Y'Length = From_X'Length and then To_X'Length = From_X'Length
                 and then To_Y'Length = From_X'Length
   is
      N : constant Natural := From_X'Length;
      Mx, My, Nx, Ny, A, B, Q : Real := 0.0;
   begin
      Scale := 1.0;
      Turn := 0.0;
      Shift_X := 0.0;
      Shift_Y := 0.0;
      Found := False;
      if N < 2 then
         return;
      end if;
      for I in 0 .. N - 1 loop
         Mx := Mx + From_X (From_X'First + I);
         My := My + From_Y (From_Y'First + I);
         Nx := Nx + To_X (To_X'First + I);
         Ny := Ny + To_Y (To_Y'First + I);
      end loop;
      Mx := Mx / Real (N);
      My := My / Real (N);
      Nx := Nx / Real (N);
      Ny := Ny / Real (N);
      for I in 0 .. N - 1 loop
         declare
            Fx : constant Real := From_X (From_X'First + I) - Mx;
            Fy : constant Real := From_Y (From_Y'First + I) - My;
            Gx : constant Real := To_X (To_X'First + I) - Nx;
            Gy : constant Real := To_Y (To_Y'First + I) - Ny;
         begin
            A := A + Fx * Gx + Fy * Gy;
            B := B + Fx * Gy - Fy * Gx;
            Q := Q + Fx ** 2 + Fy ** 2;
         end;
      end loop;
      if Q <= 0.0 or else (A = 0.0 and then B = 0.0) then
         return;
      end if;
      Scale := Sqrt (A * A + B * B) / Q;
      Turn := Arctan (B, A);
      Shift_X := Nx - Scale * (Cos (Turn) * Mx - Sin (Turn) * My);
      Shift_Y := Ny - Scale * (Sin (Turn) * Mx + Cos (Turn) * My);
      Found := True;
   end Similarity_2D;

   procedure Plane_Chain (First, Second : Chain_Side; Second_Eye : Boolean; Link : out Plane_Link) is
      D1 : Side_Access := Prepare (First);
      D2 : Side_Access := Prepare (Second);
      C1 : Plane_Point_Array (1 .. D1.Answers);
      C2 : Plane_Point_Array (1 .. D2.Answers);
      H1, H2 : Mat3 := Identity3;
      F1 : Flag_Array (1 .. D1.Answers) := [others => False];
      F2 : Flag_Array (1 .. D2.Answers) := [others => False];
      S1, S2 : Real := Real'Last;
      Ok1, Ok2 : Boolean := False;
      Kept1, Kept2 : Natural := 0;
      --  The free terms of what each side's answers are tested against: a
      --  homography each.
      Apart_Terms : constant Positive := (if Second_Eye then 8 else 16);
      Free_First  : constant Positive := (if Second_Eye then S_First else 1);
      P0_1, P0_2  : Sight_Plane;
      A1, B1, A2, B2 : Vec3;

      procedure Release is
      begin
         Free (D1);
         Free (D2);
      end Release;
   begin
      Link := (others => <>);
      if not (D1.Plane0.Found and then D2.Plane0.Found) then
         Release;
         return;
      end if;
      --  Each side's coordinates at its estimates, and the homography of each
      --  side that has answers.
      Side_State (D1.all, D1.Terms0, [0.0, 0.0, 0.0], P0_1, A1, B1);
      Side_State (D2.all, D2.Terms0, [0.0, 0.0, 0.0], P0_2, A2, B2);
      for A in 1 .. D1.Answers loop
         Coordinates (P0_1, A1, B1, D1.Rays (D1.Track (A)), C1 (A).X, C1 (A).Y);
         C1 (A).U := D1.U (A);
         C1 (A).V := D1.V (A);
      end loop;
      for A in 1 .. D2.Answers loop
         Coordinates (P0_2, A2, B2, D2.Rays (D2.Track (A)), C2 (A).X, C2 (A).Y);
         C2 (A).U := D2.U (A);
         C2 (A).V := D2.V (A);
      end loop;
      Plane_Homography (C1, H1, F1, S1, Ok1);
      if not Second_Eye then
         Plane_Homography (C2, H2, F2, S2, Ok2);
      end if;
      if not Ok1 or else (not Second_Eye and then not Ok2) then
         Release;
         return;
      end if;
      for A in 1 .. D1.Answers loop
         Link.First_Fits.Append (F1 (A));
         if F1 (A) then
            Kept1 := Kept1 + 1;
         end if;
      end loop;
      for A in 1 .. D2.Answers loop
         Link.Second_Fits.Append (F2 (A));
         if F2 (A) then
            Kept2 := Kept2 + 1;
         end if;
      end loop;
      Link.Used := Kept1 + Kept2;
      if 2 * Link.Used <= Apart_Terms then
         Release;
         return;
      end if;
      --  Each side's own noise, the homography's residuals over its kept
      --  answers at its degrees of freedom.
      declare
         function Noise (C : Plane_Point_Array; F : Flag_Array; H : Mat3; Kept : Natural) return Real is
            Sum : Real := 0.0;
         begin
            for A in C'Range loop
               if F (A) then
                  Sum := Sum + Residual_Length (H, C (A)) ** 2;
               end if;
            end loop;
            return (if 2 * Kept > 8 and then Sum > 0.0 then Sqrt (Sum / Real (2 * Kept - 8)) else Real'Last);
         end Noise;
      begin
         S1 := Noise (C1, F1, H1, Kept1);
         S2 := (if Second_Eye then 1.0 else Noise (C2, F2, H2, Kept2));
      end;
      if S1 = Real'Last or else S2 = Real'Last then
         Release;
         return;
      end if;
      Link.Sigma := (if Second_Eye then S1 else Sqrt ((S1 ** 2 * Real (2 * Kept1 - 8) + S2 ** 2 * Real (2 * Kept2 - 8))
                                                       / Real (2 * Link.Used - 16)));
      declare
         X     : Real_Array (1 .. Chain_Terms) := [others => 0.0];
         Rows  : constant Positive := 2 * Link.Used + 2 * Side_Terms;
         Terms : constant Positive := Chain_Terms - Free_First + 1;
         Scale, Turn, Shift_X, Shift_Y : Real;
         Started : Boolean;
      begin
         --  The start: pairs of each answer's coordinates on the second plane
         --  and on the first, and the similarity nearest them. Through the
         --  eye's two homographies for the second side's answers; through the
         --  second lens back onto the second plane for the first side's in
         --  the second side's own eye.
         declare
            Pairs : constant Natural := (if Second_Eye then Kept1 else Kept2);
            Fx, Fy, Tx, Ty : Real_Array (1 .. Natural'Max (1, Pairs)) := [others => 0.0];
            K : Natural := 0;
         begin
            if Second_Eye then
               for A in C1'Range loop
                  if F1 (A) then
                     declare
                        Q : constant Vec3 := On_Plane (P0_2, Ray (Second.L, C1 (A).U, C1 (A).V));
                     begin
                        K := K + 1;
                        Fx (K) := Q * A2;
                        Fy (K) := Q * B2;
                        Tx (K) := C1 (A).X;
                        Ty (K) := C1 (A).Y;
                     end;
                  end if;
               end loop;
            else
               declare
                  G : constant Mat3 := Inverse (H1) * H2;
                  Ahead : Boolean;
               begin
                  for A in C2'Range loop
                     if F2 (A) then
                        K := K + 1;
                        Fx (K) := C2 (A).X;
                        Fy (K) := C2 (A).Y;
                        Apply (G, C2 (A).X, C2 (A).Y, Tx (K), Ty (K), Ahead);
                     end if;
                  end loop;
               end;
            end if;
            Similarity_2D (Fx (1 .. K), Fy (1 .. K), Tx (1 .. K), Ty (1 .. K), Scale, Turn, Shift_X, Shift_Y, Started);
         end;
         if not Started then
            Release;
            return;
         end if;
         X (1 .. 8) := Entries_Of (H1);
         X (S_First .. S_First + 3) := [Ln (Scale), Turn, Shift_X, Shift_Y];
         X (Side_First (1) .. Side_First (1) + Lens_Terms - 1) := D1.Terms0;
         X (Side_First (2) .. Side_First (2) + Lens_Terms - 1) := D2.Terms0;
         --  Everything at once: the terms from Free_First on (the homography's
         --  too when an eye of unknown lens is the link).
         declare
            procedure Evaluate (Xf : Real_Array; R : out Real_Array) is
               Xv : Real_Array := X;
            begin
               Xv (Free_First .. Chain_Terms) := Xf;
               Chain_Residuals (D1.all, D2.all, F1, F2, Second_Eye, S1, S2, Xv, R);
            end Evaluate;

            procedure Solve is new Robust_Fit (Terms, Rows, Real'Model_Epsilon, Evaluate);
            Xf  : Real_Array (1 .. Terms) := X (Free_First .. Chain_Terms);
            R   : Real_Access := new Real_Array (1 .. Rows);
            Cost, Rss : Real := 0.0;
         begin
            Solve (Xf, 1.0);
            X (Free_First .. Chain_Terms) := Xf;
            Evaluate (Xf, R.all);
            for I in 1 .. Rows loop
               Cost := Cost + R (I) ** 2;
               if I <= 2 * Link.Used then
                  Rss := Rss + R (I) ** 2;
               end if;
            end loop;
            if Cost < Real'Last then
               --  The joint fit's own noise, in the units of the first side.
               Link.Joint := S1 * Sqrt (Rss / Real (2 * Link.Used - (Apart_Terms - 4)));
               --  The four constraints the link adds to a free homography of
               --  each side that has answers, by the extra sum of squares
               --  (with what the holds cost), every residual in units of its
               --  side's own noise.
               Link.F := (Cost - Real (2 * Link.Used - Apart_Terms)) / 4.0;
               Link.Consistent := Link.F <= 0.0
                 or else Driver.Distributions.F_Upper_Tail (Link.F, 4, 2 * Link.Used - Apart_Terms)
                         >= Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
               for V of X loop
                  Link.X.Append (V);
               end loop;
               --  The sandwich: the inverse information around the spread of
               --  every answer's share of the gradient, with the holds' own
               --  information (their estimates' uncertainty).
               declare
                  J  : Grid_Access := new Real_Matrix (1 .. Rows, 1 .. Terms);
                  Rn : Real_Access := new Real_Array (1 .. Rows);
                  A, Bm, Prior : Grid_Access := new Real_Matrix'[1 .. Terms => [1 .. Terms => 0.0]];
               begin
                  for P in 1 .. Terms loop
                     declare
                        Xp : Real_Array := Xf;
                        Hd : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Xf (P));
                     begin
                        Xp (P) := Xp (P) + Hd;
                        Evaluate (Xp, Rn.all);
                        for I in 1 .. Rows loop
                           J (I, P) := (Rn (I) - R (I)) / Hd;
                        end loop;
                     end;
                  end loop;
                  for Answer in 1 .. Link.Used loop
                     declare
                        G : Real_Vector (1 .. Terms) := [others => 0.0];
                     begin
                        for I in 2 * Answer - 1 .. 2 * Answer loop
                           declare
                              W : constant Real := Huber (R (I));
                           begin
                              for P in 1 .. Terms loop
                                 if J (I, P) /= 0.0 then
                                    G (P) := G (P) + W * R (I) * J (I, P);
                                    for Q in 1 .. Terms loop
                                       A (P, Q) := A (P, Q) + W * J (I, P) * J (I, Q);
                                    end loop;
                                 end if;
                              end loop;
                           end;
                        end loop;
                        for P in 1 .. Terms loop
                           for Q in 1 .. Terms loop
                              Bm (P, Q) := Bm (P, Q) + G (P) * G (Q);
                           end loop;
                        end loop;
                     end;
                  end loop;
                  for I in 2 * Link.Used + 1 .. Rows loop
                     for P in 1 .. Terms loop
                        if J (I, P) /= 0.0 then
                           for Q in 1 .. Terms loop
                              Prior (P, Q) := Prior (P, Q) + J (I, P) * J (I, Q);
                           end loop;
                        end if;
                     end loop;
                  end loop;
                  declare
                     Total : constant Real_Matrix := A.all + Prior.all;
                     Lf    : Real_Matrix (1 .. Terms, 1 .. Terms);
                     Pd    : Boolean;
                  begin
                     Driver.Numerics.Dense.Cholesky (Total, Lf, Pd);
                     if Pd then
                        declare
                           Inv   : Grid_Access := new Real_Matrix (1 .. Terms, 1 .. Terms);
                           Small : constant Real := Real (Link.Used) / Real (Link.Used - 1);
                        begin
                           for P in 1 .. Terms loop
                              declare
                                 E : Real_Vector (1 .. Terms) := [others => 0.0];
                              begin
                                 E (P) := 1.0;
                                 declare
                                    Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, E);
                                 begin
                                    for Q in 1 .. Terms loop
                                       Inv (Q, P) := Column (Q);
                                    end loop;
                                 end;
                              end;
                           end loop;
                           declare
                              V : constant Real_Matrix := Inv.all * (Small * Bm.all + Prior.all) * Inv.all;
                              Offset : constant Natural := S_First - Free_First;
                           begin
                              for P in 1 .. Link_Terms loop
                                 for Q in 1 .. Link_Terms loop
                                    Link.Covariance.Append (V (Offset + P, Offset + Q));
                                 end loop;
                              end loop;
                           end;
                           Free (Inv);
                           Link.Found := True;
                        end;
                     end if;
                  end;
                  Free (J);
                  Free (Rn);
                  Free (A);
                  Free (Bm);
                  Free (Prior);
               end;
            end if;
            Free (R);
         end;
      end;
      Release;
   end Plane_Chain;

   procedure Chain_Placement
     (First, Second : Chain_Side;
      Link          : Plane_Link;
      Placement     : out Rigid;
      Scale         : out Real;
      Covariance    : out Real_Lists.Vector)
   is
      D1 : Side_Access := Prepare (First);
      D2 : Side_Access := Prepare (Second);
      X  : Real_Array (1 .. Chain_Terms);

      procedure Place_At (Xv : Real_Array; P : out Rigid; S : out Real) is
         P1, P2 : Sight_Plane;
         A1, B1, A2, B2 : Vec3;
      begin
         Side_State (D1.all, Terms_Of (Xv, 1), Move_Of (Xv, 1), P1, A1, B1);
         Side_State (D2.all, Terms_Of (Xv, 2), Move_Of (Xv, 2), P2, A2, B2);
         declare
            N1 : constant Vec3 := Plane_Normal (P1);
            N2 : constant Vec3 := Plane_Normal (P2);
            Th : constant Real := Xv (S_First + 1);
            Rz : constant Mat3 := [[Cos (Th), -Sin (Th), 0.0], [Sin (Th), Cos (Th), 0.0], [0.0, 0.0, 1.0]];
            Ba : constant Mat3 := [[A1 (1), B1 (1), N1 (1)], [A1 (2), B1 (2), N1 (2)], [A1 (3), B1 (3), N1 (3)]];
            Bb : constant Mat3 := [[A2 (1), B2 (1), N2 (1)], [A2 (2), B2 (2), N2 (2)], [A2 (3), B2 (3), N2 (3)]];
         begin
            S := Exp (Xv (S_First));
            P := (Rotation    => Ba * Rz * Transpose (Bb),
                  Translation => Xv (S_First + 2) * A1 + Xv (S_First + 3) * B1
                                 + (Plane_Offset (P1) - S * Plane_Offset (P2)) * N1);
         end;
      end Place_At;

      --  The turn (first frame), the centre and the log scale of B against A.
      function Change (A, B : Rigid; Scale_A, Scale_B : Real) return Real_Vector is
         Turn  : constant Vec3 := Driver.Numerics.Log (B.Rotation * Transpose (A.Rotation));
         Shift : constant Vec3 := B.Translation - A.Translation;
      begin
         return [Turn (1), Turn (2), Turn (3), Shift (1), Shift (2), Shift (3), Ln (Scale_B / Scale_A)];
      end Change;

      Outputs : constant := 7;
      J : Real_Matrix (1 .. Outputs, 1 .. Link_Terms) := [others => [others => 0.0]];
      V : Real_Matrix (1 .. Link_Terms, 1 .. Link_Terms);
   begin
      for P in 1 .. Chain_Terms loop
         X (P) := Link.X (Link.X.First_Index + P - 1);
      end loop;
      Place_At (X, Placement, Scale);
      Covariance.Clear;
      if Natural (Link.Covariance.Length) = Link_Terms * Link_Terms then
         for P in 1 .. Link_Terms loop
            for Q in 1 .. Link_Terms loop
               V (P, Q) := Link.Covariance (Link.Covariance.First_Index + (P - 1) * Link_Terms + Q - 1);
            end loop;
         end loop;
         --  Each term moved by its standard deviation either way.
         for P in 1 .. Link_Terms loop
            if V (P, P) > 0.0 then
               declare
                  Step : constant Real := Sqrt (V (P, P));
                  Ends : array (1 .. 2) of Real_Vector (1 .. Outputs);
               begin
                  for E in 1 .. 2 loop
                     declare
                        Xv : Real_Array := X;
                        Pv : Rigid;
                        Sv : Real;
                     begin
                        Xv (S_First + P - 1) := X (S_First + P - 1) + (if E = 1 then -Step else Step);
                        Place_At (Xv, Pv, Sv);
                        Ends (E) := Change (Placement, Pv, Scale, Sv);
                     end;
                  end loop;
                  for O in 1 .. Outputs loop
                     J (O, P) := (Ends (2) (O) - Ends (1) (O)) / (2.0 * Step);
                  end loop;
               end;
            end if;
         end loop;
         declare
            C : constant Real_Matrix := J * V * Transpose (J);
         begin
            for P in 1 .. Outputs loop
               for Q in 1 .. Outputs loop
                  Covariance.Append (C (P, Q));
               end loop;
            end loop;
         end;
      end if;
      Free (D1);
      Free (D2);
   end Chain_Placement;

end Driver.Robot.Kinematics.Fit;
