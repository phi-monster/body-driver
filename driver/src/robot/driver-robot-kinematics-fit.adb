with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Robot.Regression;
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
   type Count_Array is array (Positive range <>) of Natural;
   type Count_Access is access Count_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Count_Array, Count_Access);
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

   function Dot (A, B : Vec3) return Real is (A * B);

   function Eye_At (J : Joint_Array; Change : Real_Array) return Rigid is
      T : Rigid := Identity;
   begin
      for K in J'Range loop
         declare
            A     : Joint renames J (K);
            Theta : constant Real := A.C * Change (Change'First + (K - J'First));
            Step  : Rigid;
         begin
            if A.Slide then
               Step := (Rotation => Identity3, Translation => Theta * A.W);
            else
               Step.Rotation := Rot (A.W, Theta);
               Step.Translation := A.P - Step.Rotation * A.P;
            end if;
            T := T * Step;
         end;
      end loop;
      return T;
   end Eye_At;

   --  How a point of eye A's frame appears in eye B's: X_B = R X_A + T.
   procedure Relative (A, B : Rigid; R : out Mat3; T : out Vec3) is
   begin
      R := Transpose (B.Rotation) * A.Rotation;
      T := Transpose (B.Rotation) * (A.Translation - B.Translation);
   end Relative;

   --  The Sampson residual of the lines of sight H1 (eye A) and H2 (eye B)
   --  under X_B = R X_A + T, in the units of a pixel at focal length F.
   function Sampson (R : Mat3; T : Vec3; H1, H2 : Vec3; F : Real) return Real is
      Y    : constant Vec3 := R * H1;
      Ex1  : constant Vec3 := Cross (T, Y);
      Etx2 : constant Vec3 := Transpose (R) * Cross (H2, T);
      Den  : constant Real := Sqrt (Ex1 (1) ** 2 + Ex1 (2) ** 2 + Etx2 (1) ** 2 + Etx2 (2) ** 2);
   begin
      return (if Den > 0.0 then F * (H2 * Ex1) / Den else 0.0);
   end Sampson;

   ---------------------------------------------------------------------------
   --  The lens

   function Ray (L : Lens; U, V : Real) return Vec3 is
      Xd : constant Real := (U - L.Cx) / L.Fx;
      Yd : constant Real := (V - L.Cy) / L.Fy;
      X  : Real := Xd;
      Y  : Real := Yd;
      Last_Change : Real := Real'Last;
   begin
      --  Undistort by fixed-point iteration, for as long as each step still
      --  shrinks the change.
      if L.K1 /= 0.0 or else L.K2 /= 0.0 then
         loop
            declare
               R2 : constant Real := X * X + Y * Y;
               D  : constant Real := 1.0 + L.K1 * R2 + L.K2 * R2 * R2;
               Xn : constant Real := (if D > 0.0 then Xd / D else X);
               Yn : constant Real := (if D > 0.0 then Yd / D else Y);
               Change : constant Real := abs (Xn - X) + abs (Yn - Y);
            begin
               exit when Change >= Last_Change;
               X := Xn;
               Y := Yn;
               Last_Change := Change;
               exit when Change = 0.0;
            end;
         end loop;
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

   --  The noise of residuals that have no sign of their own: the robust scale
   --  about zero, the median absolute residual over the normal's upper
   --  quartile (what Robust_Sigma gives the residuals with their negations).
   function Noise_Of (R : Real_Array) return Real is
     (Median_Abs (R) / Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5));

   --  The standard error of the median of N absolute residuals of Gaussian
   --  noise whose median is Median: the median of |r| is q sigma with q the
   --  normal's upper quartile, and a sample median varies by
   --  1 / (2 sqrt (N) f) with f the density there, 2 phi (q) / sigma.
   function Median_Error (Median : Real; Count : Positive) return Real is
      Q     : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5);
      Sigma : constant Real := Median / Q;
      F     : constant Real := 2.0 * Exp (-Q * Q / 2.0) / Sqrt (2.0 * Ada.Numerics.Pi);
   begin
      return Sigma / (2.0 * Sqrt (Real (Count)) * F);
   end Median_Error;

   function Huber (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then 1.0 else Driver.Conventions.Z / abs Z_Value);

   function Huber_Cost (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then 0.5 * Z_Value ** 2
      else Driver.Conventions.Z * (abs Z_Value - 0.5 * Driver.Conventions.Z));

   ---------------------------------------------------------------------------
   --  Robust least squares: Levenberg-Marquardt on a numerical Jacobian, Huber
   --  weights at Z on residuals in units of Sigma. It stops when a step lowers
   --  the cost by less than the unchanged fraction of it, or when no damping
   --  gives a step that lowers it at all.

   generic
      Parameters : Positive;
      Residuals  : Positive;
      with procedure Evaluate (X : Real_Array; R : out Real_Array);
   procedure Robust_Fit (X : in out Real_Array; Sigma : Real);

   procedure Robust_Fit (X : in out Real_Array; Sigma : Real) is
      type Matrix_Access is access Real_Matrix;
      procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
      R0     : Real_Access := new Real_Array (1 .. Residuals);
      Rn     : Real_Access := new Real_Array (1 .. Residuals);
      J      : Matrix_Access := new Real_Matrix (1 .. Residuals, 1 .. Parameters);
      Lambda : Real := Real'Model_Epsilon;
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
         for K in 1 .. Parameters loop
            declare
               Xp : Real_Array := X;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (X'First + K - 1));
            begin
               Xp (Xp'First + K - 1) := Xp (Xp'First + K - 1) + H;
               Evaluate (Xp, Rn.all);
               for I in 1 .. Residuals loop
                  J (I, K) := (Rn (I) - R0 (I)) / H;
               end loop;
            end;
         end loop;
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
                                 Improved := Cost0 - C > Driver.Conventions.Unchanged_Fraction * Cost0;
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
                  exit when Lowered or else not Moves;
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

   procedure Refine_Tracks
     (Changes : Driver.Numerics.Arrays.Real_Matrix;
      Sight   : Sighting_Array;
      Joints  : in out Joint_Array;
      L       : in out Lens;
      Report  : in out Fit_Report)
   is
      N         : constant Natural := Joints'Length;
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
         Changed : Natural := Natural'Last;
         --  Each parameter's variance at the solution, in the reduced set: the
         --  lens (6), then per joint its tilt (2), its point across the axis
         --  (2) and its reading scale (1).
         Variance   : Real_Array (1 .. 6 + 5 * N) := [others => Real'Last];
         Determined : Boolean := False;

         --  The reprojection residuals of the chosen sightings (2 each).
         procedure Residuals (Xv : Real_Array; Dv : Real_Array; Index : Real_Access; R : out Real_Array) is
            Lx    : constant Lens := Lens_Of (Xv);
            Jx    : constant Joint_Array := Joints_Of (Xv);
            Views : Rigid_Access := new Rigid_Array (1 .. Frames);
         begin
            for F in 1 .. Frames loop
               Views (F) := Inverse (Eye_At (Jx, Changes_Of (F)));
            end loop;
            for K in 1 .. Index'Length loop
               declare
                  Sg : Sighting renames Sight (Sight'First + Natural (Index (K)) - 1);
                  Pw : constant Vec3 := Exp (Dv (Sg.Track)) * Ray (Lx, Sg.U0, Sg.V0);
                  U, V : Real;
                  Ahead : Boolean;
               begin
                  Project (Lx, Views (Sg.Frame) * Pw, U, V, Ahead);
                  R (2 * K - 1) := U - Sg.U;
                  R (2 * K) := V - Sg.V;
               end;
            end loop;
            Free (Views);
         end Residuals;

         procedure Triangulate is
            Num : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
            Den : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         begin
            for Sg of Sight loop
               declare
                  T  : constant Rigid := Eye_At (Joints, Changes_Of (Sg.Frame));
                  H0 : constant Vec3 := Ray (L, Sg.U0, Sg.V0);
                  Bv : constant Vec3 := T.Rotation * Ray (L, Sg.U, Sg.V);
                  function Across (V : Vec3) return Vec3 is (V - (Dot (V, Bv) / Dot (Bv, Bv)) * Bv);
               begin
                  Num (Sg.Track) := Num (Sg.Track) + Dot (Across (H0), Across (T.Translation));
                  Den (Sg.Track) := Den (Sg.Track) + Dot (Across (H0), Across (H0));
               end;
            end loop;
            for T in 1 .. Tracks loop
               Has (T) := Den (T) > 0.0 and then Num (T) / Den (T) > 0.0;
               Depth (T) := (if Has (T) then Ln (Num (T) / Den (T)) else 0.0);
            end loop;
            Free (Num);
            Free (Den);
         end Triangulate;
      begin
         Triangulate;
         for I in 1 .. S loop
            Inlier (I) := Has (Sight (Sight'First + I - 1).Track);
         end loop;
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
               declare
                  Index : Real_Access := new Real_Array (1 .. Used);
                  R0    : Real_Access := new Real_Array (1 .. 2 * Used);
                  Rn    : Real_Access := new Real_Array (1 .. 2 * Used);
                  Jp    : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. 2 * Used, 1 .. Count);
                  Jd    : Real_Access := new Real_Array (1 .. 2 * Used);
                  K     : Natural := 0;
                  Sigma : Real;
                  Lambda : Real := Real'Model_Epsilon;
                  Cost0 : Real;

                  function Cost (R : Real_Array) return Real is
                     C : Real := 0.0;
                  begin
                     for V of R loop
                        C := C + Huber_Cost (V / Sigma);
                     end loop;
                     return C;
                  end Cost;
               begin
                  for I in 1 .. S loop
                     if Inlier (I) then
                        K := K + 1;
                        Index (K) := Real (I);
                     end if;
                  end loop;
                  Residuals (X, Depth.all, Index, R0.all);
                  Sigma := Noise_Of (R0.all);
                  if Sigma <= 0.0 then
                     Free (Index);
                     Free (R0);
                     Free (Rn);
                     Free (Jp);
                     Free (Jd);
                     exit;
                  end if;
                  Cost0 := Cost (R0.all);
                  loop
                     --  The Jacobian: the parameters one by one; the depths all at
                     --  once, since each residual has one depth of its own.
                     for P in 1 .. Count loop
                        declare
                           Xp : Real_Array := X;
                           H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (P));
                        begin
                           Xp (P) := Xp (P) + H;
                           Residuals (Xp, Depth.all, Index, Rn.all);
                           for I in 1 .. 2 * Used loop
                              Jp (I, P) := (Rn (I) - R0 (I)) / H;
                           end loop;
                        end;
                     end loop;
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
                              W : constant Real := Huber (R0 (I) / Sigma) / Sigma ** 2;
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
                                          Cn : constant Real := Cost (Rn.all);
                                       begin
                                          if Cn < Cost0 then
                                             Improved := Cost0 - Cn > Driver.Conventions.Unchanged_Fraction * Cost0;
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
                              exit when Lowered or else not Moves;
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
                                 Driver.Numerics.Dense.Cholesky (Ar, Lf, Pd);
                                 Determined := Pd;
                                 Variance := [others => Real'Last];
                                 Report.Covariance.Clear;
                                 --  The covariance clustered by keyframe: the inverse
                                 --  normal equations around the spread of every
                                 --  keyframe's own share of the gradient (each
                                 --  residual's, the depths eliminated). A keyframe's
                                 --  matches err together (its rendering, its view),
                                 --  which sightings taken as independent hide: A9's
                                 --  focal length and reading scales came out 10 to 20
                                 --  of those sigmas off the truth.
                                 if Pd then
                                    declare
                                       Inv   : Grid_Access := new Real_Matrix (1 .. Reduced, 1 .. Reduced);
                                       Share : Grid_Access := new Real_Matrix'[1 .. Frames => [1 .. Reduced => 0.0]];
                                       Spread : Grid_Access := new Real_Matrix'[1 .. Reduced => [1 .. Reduced => 0.0]];
                                       Clusters : Natural := 0;
                                    begin
                                       for P in 1 .. Reduced loop
                                          declare
                                             E : Real_Vector (1 .. Reduced) := [others => 0.0];
                                          begin
                                             E (P) := 1.0;
                                             declare
                                                Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, E);
                                             begin
                                                for Q in 1 .. Reduced loop
                                                   Inv (Q, P) := Column (Q);
                                                end loop;
                                             end;
                                          end;
                                       end loop;
                                       for I in 1 .. 2 * Used loop
                                          declare
                                             Sg  : Sighting renames Sight (Sight'First + Natural (Index ((I + 1) / 2)) - 1);
                                             Psi : constant Real := Huber (R0 (I) / Sigma) / Sigma ** 2 * R0 (I);
                                          begin
                                             if Psi /= 0.0 then
                                                for P in 1 .. Reduced loop
                                                   declare
                                                      Jr : Real := 0.0;
                                                   begin
                                                      for Q in 1 .. Count loop
                                                         if Tm (Q, P) /= 0.0 then
                                                            Jr := Jr + Tm (Q, P) * Jp (I, Q);
                                                         end if;
                                                      end loop;
                                                      if Sg.Track /= Anchor and then C (Sg.Track) > 0.0 then
                                                         Jr := Jr - Br (P, Sg.Track) / C (Sg.Track) * Jd (I);
                                                      end if;
                                                      Share (Sg.Frame, P) := Share (Sg.Frame, P) + Psi * Jr;
                                                   end;
                                                end loop;
                                             end if;
                                          end;
                                       end loop;
                                       for F in 1 .. Frames loop
                                          if (for some P in 1 .. Reduced => Share (F, P) /= 0.0) then
                                             Clusters := Clusters + 1;
                                             for P in 1 .. Reduced loop
                                                for Q in 1 .. Reduced loop
                                                   Spread (P, Q) := Spread (P, Q) + Share (F, P) * Share (F, Q);
                                                end loop;
                                             end loop;
                                          end if;
                                       end loop;
                                       --  Too few keyframes to tell how theirs spread: not
                                       --  determined.
                                       if Clusters > 1 then
                                          declare
                                             --  The spread of a mean over the clusters, unbiased.
                                             Small : constant Real := Real (Clusters) / Real (Clusters - 1);
                                             V     : constant Real_Matrix := Inv.all * Spread.all * Inv.all;
                                          begin
                                             for P in 1 .. Reduced loop
                                                Variance (P) := Small * V (P, P);
                                                for Q in 1 .. Reduced loop
                                                   Report.Covariance.Append (Small * V (P, Q));
                                                end loop;
                                             end loop;
                                          end;
                                       else
                                          Determined := False;
                                       end if;
                                       Free (Inv);
                                       Free (Share);
                                       Free (Spread);
                                    end;
                                 end if;
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
               Sigma     : Real;
               Now_Changed : Natural := 0;
               Fit_Abs   : Real_Access;
               Kf        : Natural := 0;
            begin
               for I in 1 .. S loop
                  All_Index (I) := Real (I);
               end loop;
               Residuals (X, Depth.all, All_Index, All_R.all);
               Sigma := Noise_Of (All_R.all);
               for I in 1 .. S loop
                  declare
                     Fits : constant Boolean := Has (Sight (Sight'First + I - 1).Track) and then Sigma > 0.0
                       and then not Driver.Uncertain.Significant (All_R (2 * I - 1), Sigma)
                       and then not Driver.Uncertain.Significant (All_R (2 * I), Sigma);
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
               Free (All_Index);
               Free (All_R);
               exit when Now_Changed = 0 or else Now_Changed >= Changed;
               Changed := Now_Changed;
            end;
         end loop;
         L := Lens_Of (X);
         Joints := Joints_Of (X);
         --  Fitted only when the sightings determine every parameter that has a
         --  value of its own: each focal length and each reading scale
         --  significant against its uncertainty, and each axis known to a
         --  cone whose Z-sigma edge stays within a quarter turn of it.
         Report.Determined := Determined;
         if not Determined then
            Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("the sightings do not determine the lens and the joints (the normal equations are singular)");
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
                     Report.Determined := False;
                     Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                       ("the sightings leave joint" & J'Image & " undetermined: its axis to" & Real'Image (Tilt)
                        & " rad, its scale" & Real'Image (C) & " +-" & Real'Image (Sqrt (Variance (O + 5))));
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
         Free (Inlier);
      end;
   end Refine_Tracks;

   ---------------------------------------------------------------------------
   --  The fit

   procedure Fit
     (Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Visible    : Real_Array;
      Sightings  : Sighting_Array;
      Width, Height : Positive;
      Joints     : out Joint_Array;
      L          : out Lens;
      Report     : out Fit_Report)
   is
      N      : constant Natural := Visible'Length;
      Frames : constant Natural := Changes'Length (1);
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

      --  The keyframe moved joint J by a step its eye can see.
      function Moved (Frame, J : Positive) return Boolean is
        (Visible (Visible'First + J - 1) > 0.0 and then abs Change (Frame, J) >= Visible (Visible'First + J - 1));

      --  Only joint J moved in the keyframe, as far as its eye can tell.
      function Only (Frame, J : Positive) return Boolean is
        (Moved (Frame, J) and then (for all C in 1 .. N => C = J or else not Moved (Frame, C)));

      function Single (Frame : Positive) return Natural is
      begin
         for J in 1 .. N loop
            if Only (Frame, J) then
               return J;
            end if;
         end loop;
         return 0;
      end Single;

      Lens0  : Lens := (Fx | Fy => 1.0, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, others => 0.0);
      Counts : array (1 .. N) of Natural := [others => 0];
      Usable : Flags (1 .. N) := [others => False];

      procedure Fail (Why : String) is
      begin
         Report.Why := Ada.Strings.Unbounded.To_Unbounded_String (Why);
         L := Lens0;
      end Fail;

      --  The eye at every keyframe under the joints.
      subtype Pose_Array is Rigid_Array (1 .. Frames);
      procedure Poses_Of (Jx : Joint_Array; P : out Pose_Array) is
      begin
         for Frame in 1 .. Frames loop
            P (Frame) := Eye_At (Jx, Changes_Of (Frame));
         end loop;
      end Poses_Of;

      --  Lengths in model units: the eye positions over the keyframes have a
      --  root mean square of one.
      procedure Normalize (Jx : in out Joint_Array) is
         Sum : Real := 0.0;
      begin
         for Frame in 1 .. Frames loop
            declare
               T : constant Vec3 := Eye_At (Jx, Changes_Of (Frame)).Translation;
            begin
               Sum := Sum + T * T;
            end;
         end loop;
         if Sum > 0.0 then
            declare
               F : constant Real := 1.0 / Sqrt (Sum / Real (Frames));
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

   begin
      Report := (others => <>);
      Joints := [others => <>];
      L := Lens0;
      if N = 0 or else S = 0 or else Frames < 2 then
         Fail ("no keyframes or no matches");
         return;
      end if;
      for X of Sight loop
         if Single (X.Frame) > 0 then
            Counts (Single (X.Frame)) := Counts (Single (X.Frame)) + 1;
         end if;
      end loop;
      for J in 1 .. N loop
         Usable (J) := Counts (J) > 0;
      end loop;
      if (for some U of Usable => not U) then
         Fail ("a joint has no keyframe of its own");
         return;
      end if;

      ---------------------------------------------------------------------------
      --  Stage 1: every joint alone.
      Report.Stage := 1;
      declare
         F_Low  : Real := Real'Last;
         F_High : Real := 0.0;

         --  The directions searched: a Fibonacci lattice on the sphere, as
         --  dense as an icosahedron split four times (2562 points, about
         --  four degrees apart); a direction and its opposite are both kept,
         --  since the turn's sign is the reading's.
         Directions : constant Natural := 2562;
         Sphere     : array (1 .. Directions) of Vec3;

         --  A joint's sightings as lines of sight at one focal length, with
         --  the turn of their keyframe, grouped by keyframe.
         type Sight_Line is record
            Frame : Positive := 1;
            Theta : Real := 0.0;
            H0, H : Vec3 := [0.0, 0.0, 1.0];
         end record;
         type Sight_Line_Array is array (Positive range <>) of Sight_Line;
         type Sight_Line_Access is access Sight_Line_Array;
         procedure Free is new Ada.Unchecked_Deallocation (Sight_Line_Array, Sight_Line_Access);
         Lines : array (1 .. N) of Sight_Line_Access;

         --  The search scores a joint on its widest turn each way: the keyframes
         --  that tell its axis best; the later stages use every keyframe.
         Up_Frame, Down_Frame : array (1 .. N) of Natural := [others => 0];
         Grid_Counts : array (1 .. N) of Natural := [others => 0];

         function Widest (Frame : Positive) return Boolean is
           (Single (Frame) > 0
            and then (Frame = Up_Frame (Single (Frame)) or else Frame = Down_Frame (Single (Frame))));

         procedure Prepare (F : Real) is
            Lx : constant Lens := (Fx | Fy => F, Cx => Lens0.Cx, Cy => Lens0.Cy, others => 0.0);
         begin
            for J in 1 .. N loop
               Free (Lines (J));
               Lines (J) := new Sight_Line_Array (1 .. Grid_Counts (J));
               declare
                  K : Natural := 0;
               begin
                  for X of Sight loop
                     if Single (X.Frame) = J and then Widest (X.Frame) then
                        K := K + 1;
                        Lines (J) (K) := (Frame => X.Frame, Theta => Change (X.Frame, J),
                                          H0 => Ray (Lx, X.U0, X.V0), H => Ray (Lx, X.U, X.V));
                     end if;
                  end loop;
               end;
            end loop;
         end Prepare;

         type Best_Record is record
            Score : Real := Real'Last;
            W     : Vec3 := [0.0, 0.0, 1.0];
            Phi   : Real := 0.0;
         end record;
         type Best_Array is array (1 .. N) of Best_Record;

         --  The side of the eye the axis W lies on, solved in closed form (the
         --  epipolar constraint is linear in the axis point), the constraints
         --  weighted to Sampson residuals and by Huber at the noise of a first
         --  solution; the median residual left.
         procedure Best_Phi
           (J : Positive; F : Real; W : Vec3; Phi : out Real; Median_Px : out Real; Ga, Gb, Wt, Rs : out Real_Array)
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
                  R := Rot (W, L (I).Theta);
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
               declare
                  P : constant Vec3 := C * E1 + Sn * E2;
               begin
                  Last := 0;
                  for I in 1 .. K loop
                     if L (I).Frame /= Last then
                        Last := L (I).Frame;
                        R := Rot (W, L (I).Theta);
                     end if;
                     declare
                        Rr : Mat3;
                        Tt : Vec3;
                     begin
                        Relative (Identity, (Rotation => R, Translation => P - R * P), Rr, Tt);
                        Rs (I) := Sampson (Rr, Tt, L (I).H0, L (I).H, F);
                     end;
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

         --  Every joint's best direction at focal length F, the joints scanned
         --  at once, one task each: their sightings are their own.
         procedure Search (F : Real; Best : out Best_Array) is
         begin
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
                  begin
                     for D of Sphere loop
                        declare
                           Ph, Md : Real;
                        begin
                           Best_Phi (J, F, D, Ph, Md, Ga.all, Gb.all, Wt.all, Rs.all);
                           if Md < Result.Score then
                              Result := (Score => Md, W => D, Phi => Ph);
                           end if;
                        end;
                     end loop;
                     Free (Ga);
                     Free (Gb);
                     Free (Wt);
                     Free (Rs);
                  end;
                  Best (J) := Result;
               end Scanner;

               Scanners : array (1 .. N) of Scanner;
            begin
               for J in 1 .. N loop
                  Scanners (J).Start (J);
               end loop;
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
               Sum := Sum + Median_Error (B (J).Score, Grid_Counts (J)) ** 2;
            end loop;
            return Sqrt (Sum);
         end Total_Error;

         Golden    : constant Real := (3.0 - Sqrt (5.0)) * Ada.Numerics.Pi;
         Inv_Ratio : constant Real := (Sqrt (5.0) - 1.0) / 2.0;
         A, B, X1, X2 : Real;
         BA, BB, B1, B2 : Best_Array;
      begin
         for Frame in 2 .. Frames loop
            declare
               J : constant Natural := Single (Frame);
            begin
               if J > 0 then
                  if Change (Frame, J) > 0.0
                    and then (Up_Frame (J) = 0 or else Change (Frame, J) > Change (Up_Frame (J), J))
                  then
                     Up_Frame (J) := Frame;
                  elsif Change (Frame, J) < 0.0
                    and then (Down_Frame (J) = 0 or else Change (Frame, J) < Change (Down_Frame (J), J))
                  then
                     Down_Frame (J) := Frame;
                  end if;
               end if;
            end;
         end loop;
         for X of Sight loop
            if Widest (X.Frame) then
               Grid_Counts (Single (X.Frame)) := Grid_Counts (Single (X.Frame)) + 1;
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
         --  Where to look for the focal length: a turn moves the image by
         --  about the focal length times its angle, so the keyframes' ratios
         --  of the two bracket it.
         for Frame in 2 .. Frames loop
            if Single (Frame) > 0 then
               declare
                  D : Real_Access := new Real_Array (1 .. S);
                  K : Natural := 0;
               begin
                  for X of Sight loop
                     if X.Frame = Frame then
                        K := K + 1;
                        D (K) := Sqrt ((X.U - X.U0) ** 2 + (X.V - X.V0) ** 2);
                     end if;
                  end loop;
                  if K > 0 and then Driver.Stats.Median (D (1 .. K)) > 0.0 then
                     declare
                        Ratio : constant Real := Driver.Stats.Median (D (1 .. K)) / abs Change (Frame, Single (Frame));
                     begin
                        F_Low := Real'Min (F_Low, Ratio);
                        F_High := Real'Max (F_High, Ratio);
                     end;
                  end if;
                  Free (D);
               end;
            end if;
         end loop;
         if F_High <= 0.0 then
            Fail ("the matches do not move");
            return;
         end if;
         --  Golden-section search in the logarithm of the focal length. The
         --  bracket first widens by its own width on the side whose end scores
         --  best, until an inner point scores better than both ends; the
         --  search ends when its two inner points cannot be told apart.
         A := Ln (F_Low);
         B := Ln (F_High);
         if B <= A then
            A := A - Ln (2.0) / 2.0;
            B := B + Ln (2.0) / 2.0;
         end if;
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
         declare
            Best  : constant Best_Array := (if Total (B1) < Total (B2) then B1 else B2);
            Focal : constant Real := Exp (if Total (B1) < Total (B2) then X1 else X2);
         begin
            Lens0.Fx := Focal;
            Lens0.Fy := Focal;
            for J in 1 .. N loop
               declare
                  E1, E2 : Vec3;
               begin
                  Perp (Best (J).W, E1, E2);
                  Joints (Joints'First + J - 1) :=
                    (W => Best (J).W, P => Cos (Best (J).Phi) * E1 + Sin (Best (J).Phi) * E2, C => 1.0, Slide => False);
               end;
               Free (Lines (J));
            end loop;
         end;
      end;

      ---------------------------------------------------------------------------
      --  Stage 2: the joints together with the focal length, on the keyframes
      --  that moved one joint each. Per joint: the axis's tilt in its tangent
      --  plane (2) and the side of the eye (1); then the log focal length.
      Report.Stage := 2;
      declare
         Count : constant Positive := 3 * N + 1;
         Base  : constant Joint_Array := Joints;
         Used  : Natural := 0;
      begin
         for X of Sight loop
            if Single (X.Frame) > 0 then
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
                     O : constant Natural := X'First + 3 * (J - 1);
                     E1, E2, F1, F2 : Vec3;
                     W : Vec3;
                  begin
                     Perp (Base (Base'First + J - 1).W, E1, E2);
                     W := Unit (Base (Base'First + J - 1).W + X (O) * E1 + X (O + 1) * E2);
                     Perp (W, F1, F2);
                     Out_J (Out_J'First + J - 1).W := W;
                     Out_J (Out_J'First + J - 1).P := Cos (X (O + 2)) * F1 + Sin (X (O + 2)) * F2;
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
                     J  : constant Positive := Single (Kept (I).Frame);
                     A  : Joint renames Jx (Jx'First + J - 1);
                     Rj : constant Mat3 := Rot (A.W, Change (Kept (I).Frame, J));
                     Rr : Mat3;
                     Tt : Vec3;
                  begin
                     Relative (Identity, (Rotation => Rj, Translation => A.P - Rj * A.P), Rr, Tt);
                     R (R'First + I - 1) :=
                       Sampson (Rr, Tt, Ray (Lx, Kept (I).U0, Kept (I).V0), Ray (Lx, Kept (I).U, Kept (I).V), F);
                  end;
               end loop;
            end Evaluate;

            procedure Solve is new Robust_Fit (Count, Used, Evaluate);
            X : Real_Array (1 .. Count) := [others => 0.0];
         begin
            for Xs of Sight loop
               if Single (Xs.Frame) > 0 then
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
                  X (3 * (J - 1) + 3) := Arctan (Base (Base'First + J - 1).P * E2, Base (Base'First + J - 1).P * E1);
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
      Report.Stage := 3;
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
            for X of Sight loop
               declare
                  J : constant Natural := Single (X.Frame);
               begin
                  if J > 0 then
                     declare
                        A  : Joint renames Joints (Joints'First + J - 1);
                        R  : constant Mat3 := Rot (A.W, Change (X.Frame, J));
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
      Normalize (Joints);

      ---------------------------------------------------------------------------
      --  Stage 4: everything at once, on the sightings that fit. Parameters:
      --  log Fx, log Fy, Cx, Cy, K1, K2, then per joint the axis's tilt in its
      --  tangent plane (2), its point (3) and its reading scale (1).
      Report.Stage := 4;
      declare
         Per_Joint : constant := 6;
         Count     : constant Positive := 6 + Per_Joint * N;
         Inlier    : Flags_Access := new Flags'(1 .. S => False);
         Base      : Joint_Array := Joints;
         Changed   : Natural := Natural'Last;

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
               --  Re-chosen until the choice no longer changes, or changes no
               --  less than it did the round before.
               exit when Now_Changed = 0 or else Now_Changed >= Changed or else Used <= Count;
               Changed := Now_Changed;
               declare
                  Kept : Sighting_Access := new Sighting_Array (1 .. Used);
                  K    : Natural := 0;

                  procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
                     Lx    : constant Lens := Lens_Of (Xv);
                     Poses : Rigid_Access := new Pose_Array;
                  begin
                     Poses_Of (Joints_Of (Xv), Poses.all);
                     for I in 1 .. Used loop
                        declare
                           Rr : Mat3;
                           Tt : Vec3;
                        begin
                           Relative (Poses (1), Poses (Kept (I).Frame), Rr, Tt);
                           R (R'First + I - 1) := Sampson (Rr, Tt, Ray (Lx, Kept (I).U0, Kept (I).V0),
                                                           Ray (Lx, Kept (I).U, Kept (I).V), (Lx.Fx + Lx.Fy) / 2.0);
                        end;
                     end loop;
                     Free (Poses);
                  end Evaluate;

                  procedure Solve is new Robust_Fit (Count, Used, Evaluate);
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

      --  The sign of every translation at once, which the epipolar constraint
      --  cannot tell: the matched points lie in front of both eyes.
      declare
         Front, Back : Natural := 0;
      begin
         for X of Sight loop
            declare
               T   : constant Rigid := Eye_At (Joints, Changes_Of (X.Frame));
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
      Report.Stage := 5;
      Refine_Tracks (Changes, Sight, Joints, L, Report);
      declare
         Sum : Real := 0.0;
      begin
         for Frame in 1 .. Frames loop
            declare
               T : constant Vec3 := Eye_At (Joints, Changes_Of (Frame)).Translation;
            begin
               Sum := Sum + T * T;
            end;
         end loop;
         Normalize (Joints);
         --  The depths follow the lengths into model units.
         if Sum > 0.0 then
            for D of Report.Depths loop
               D := D / Sqrt (Sum / Real (Frames));
            end loop;
         end if;
         --  The covariance follows the lengths into model units.
         if Sum > 0.0 and then not Report.Covariance.Is_Empty then
            declare
               F       : constant Real := 1.0 / Sqrt (Sum / Real (Frames));
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
      Report.Fitted := Report.Determined;
   end Fit;

   function Unit_Sigma
     (Joints     : Joint_Array;
      Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Covariance : Real_Lists.Vector) return Real
   is
      N       : constant Natural := Joints'Length;
      Reduced : constant Natural := Terms (N);
      function V (P, Q : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Reduced + Q - 1));
      --  The root mean square of the eye positions over the keyframes.
      function Unit_Of (Jx : Joint_Array) return Real is
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
                  T : constant Vec3 := Eye_At (Jx, D).Translation;
               begin
                  Sum := Sum + Dot (T, T);
               end;
            end;
         end loop;
         return Sqrt (Sum / Real (Natural'Max (1, Changes'Length (1))));
      end Unit_Of;
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

   procedure Pose_Covariance
     (Joints      : Joint_Array;
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
                        return Eye_At (Jx, Change);
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

   --  A repeatable uniform generator: Park and Miller's minimal standard.
   type Sampler is record
      State : Long_Long_Integer := 1;
   end record;

   function Draw (S : in out Sampler; N : Positive) return Positive is
   begin
      S.State := (S.State * 48_271) mod 2_147_483_647;
      return Positive (1 + S.State mod Long_Long_Integer (N));
   end Draw;

   --  Into'Length distinct numbers of 1 .. N.
   procedure Pick (S : in out Sampler; N : Positive; Into : out Count_Array)
     with Pre => Into'Length <= N
   is
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
         Spread : Mat3 := [others => [others => 0.0]];
      begin
         for I in Points'Range loop
            if On (I) and then Usable (Points (I)) then
               declare
                  W : constant Real := 1.0 / Inverse_Sigma (Points (I)) ** 2;
                  R : constant Real := Inverse_Depth (Points (I)) - A * Points (I).H;
               begin
                  Spread := Spread + (W * W * R * R) * Outer (Points (I).H, Points (I).H);
               end;
            end if;
         end loop;
         Plane := (Found      => A * A > 0.0,
                   A          => A,
                   Covariance => (Real (Count) / Real (Count - 3)) * (Mi * Spread * Mi),
                   Points     => Count);
      end;
   end Refit_Plane;

   procedure Dominant_Plane (Points : Sight_Point_Array; Plane : out Sight_Plane; On : out Flag_Array) is
      Minimal : constant := 3;
      Index   : Count_Access := new Count_Array (1 .. Points'Length);
      Valid   : Natural := 0;
   begin
      Plane := (others => <>);
      On := [others => False];
      for I in Points'Range loop
         if Usable (Points (I)) then
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
            On (Index (K)) := Standard_Residual (Best_A, Points (Index (K))) ** 2 <= Bound;
         end loop;
         declare
            Changed : Natural := Natural'Last;
         begin
            loop
               Refit_Plane (Points, On, Plane);
               exit when not Plane.Found;
               declare
                  Z_All  : Real_Access := new Real_Array (1 .. Valid);
                  Count  : Natural := 0;
                  Spread : Real;
                  Now_Changed : Natural := 0;
               begin
                  for K in 1 .. Valid loop
                     if On (Index (K)) then
                        Count := Count + 1;
                        Z_All (Count) := Standard_Residual (Plane.A, Points (Index (K)));
                     end if;
                  end loop;
                  Spread := Noise_Of (Z_All (1 .. Count));
                  Free (Z_All);
                  for K in 1 .. Valid loop
                     declare
                        Fits : constant Boolean :=
                          Spread > 0.0 and then not Driver.Uncertain.Significant
                                                      (Standard_Residual (Plane.A, Points (Index (K))), Spread);
                     begin
                        if Fits /= On (Index (K)) then
                           Now_Changed := Now_Changed + 1;
                           On (Index (K)) := Fits;
                        end if;
                     end;
                  end loop;
                  exit when Now_Changed = 0 or else Now_Changed >= Changed;
                  Changed := Now_Changed;
               end;
            end loop;
            if Plane.Found then
               Refit_Plane (Points, On, Plane);
            end if;
         end;
      end;
      Free (Index);
   end Dominant_Plane;

   ---------------------------------------------------------------------------
   --  A plane through an eye

   --  The median of the lengths of two-coordinate residuals of Gaussian noise
   --  is sigma times the root of 2 ln 2.
   Rayleigh_Median : constant Real := Sqrt (2.0 * Ln (2.0));

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

         procedure Solve is new Robust_Fit (8, 2 * Used, Evaluate);
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

   --  The joint fit of a chain: the first frame's homography (eight entries)
   --  and the similarity (log scale, turn, two shifts) by robust least
   --  squares at Huber_Sigma over the points each homography kept, from X.
   --  Rss is the sum of the squared residuals there, Beyond how many of
   --  their coordinates lie further than Z of Huber_Sigma.
   procedure Joint_Chain
     (First, Second : Plane_Point_Array;
      First_Fits, Second_Fits : Flag_Lists.Vector;
      X           : in out Real_Array;
      Huber_Sigma : Real;
      Rss         : out Real;
      Beyond      : out Natural;
      Used        : out Natural;
      Covariance  : out Real_Lists.Vector)
   is
      N1 : constant Natural := First'Length;
      N2 : constant Natural := Second'Length;
      Index : Count_Access;

      --  The residuals of point I of both sets (the first's, then the second's).
      procedure One (Xv : Real_Array; I : Positive; Du, Dv : out Real) is
         Hv : constant Mat3 := Homography_Of (Xv (Xv'First .. Xv'First + 7));
         P  : constant Plane_Point :=
           (if I <= N1 then First (First'First + I - 1) else Second (Second'First + I - N1 - 1));
         Px, Py, U, V : Real;
         Ahead : Boolean;
      begin
         if I <= N1 then
            Px := P.X;
            Py := P.Y;
         else
            declare
               Sc : constant Real := Exp (Xv (Xv'First + 8));
               Th : constant Real := Xv (Xv'First + 9);
            begin
               Px := Sc * (Cos (Th) * P.X - Sin (Th) * P.Y) + Xv (Xv'First + 10);
               Py := Sc * (Sin (Th) * P.X + Cos (Th) * P.Y) + Xv (Xv'First + 11);
            end;
         end if;
         Apply (Hv, Px, Py, U, V, Ahead);
         Du := (if Ahead then U - P.U else Real'Last);
         Dv := (if Ahead then V - P.V else Real'Last);
      end One;
   begin
      Used := 0;
      Beyond := 0;
      Covariance.Clear;
      Rss := Real'Last;
      for I in 1 .. N1 loop
         if First_Fits (I) then
            Used := Used + 1;
         end if;
      end loop;
      for I in 1 .. N2 loop
         if Second_Fits (I) then
            Used := Used + 1;
         end if;
      end loop;
      if 2 * Used <= 12 then
         return;
      end if;
      Index := new Count_Array (1 .. Used);
      declare
         K : Natural := 0;
      begin
         for I in 1 .. N1 loop
            if First_Fits (I) then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
         for I in 1 .. N2 loop
            if Second_Fits (I) then
               K := K + 1;
               Index (K) := N1 + I;
            end if;
         end loop;
      end;
      declare
         procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
         begin
            for J in 1 .. Used loop
               One (Xv, Index (J), R (R'First + 2 * J - 2), R (R'First + 2 * J - 1));
            end loop;
         end Evaluate;

         procedure Solve is new Robust_Fit (12, 2 * Used, Evaluate);
      begin
         Solve (X, Huber_Sigma);
      end;
      Rss := 0.0;
      for J in 1 .. Used loop
         declare
            Du, Dv : Real;
         begin
            One (X, Index (J), Du, Dv);
            Rss := Rss + Du * Du + Dv * Dv;
            for D of Real_Array'[Du, Dv] loop
               if Driver.Uncertain.Significant (D, Huber_Sigma) then
                  Beyond := Beyond + 1;
               end if;
            end loop;
         end;
      end loop;
      --  The sandwich over the points: the inverse normal equations around
      --  the spread of every point's share of the gradient; the similarity's
      --  block of it, with the small-sample factor of a mean over the points.
      declare
         Sigma : constant Real := Sqrt (Rss / Real (2 * Used - 12));
         A, Bm : Real_Matrix (1 .. 12, 1 .. 12) := [others => [others => 0.0]];
      begin
         if Sigma > 0.0 then
            for J in 1 .. Used loop
               declare
                  R0 : Real_Array (1 .. 2);
                  Jc : Real_Matrix (1 .. 2, 1 .. 12);
                  G  : Real_Vector (1 .. 12) := [others => 0.0];
               begin
                  One (X, Index (J), R0 (1), R0 (2));
                  for P in 1 .. 12 loop
                     declare
                        Xp : Real_Array := X;
                        Hd : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (X'First + P - 1));
                        Rn : Real_Array (1 .. 2);
                     begin
                        Xp (Xp'First + P - 1) := Xp (Xp'First + P - 1) + Hd;
                        One (Xp, Index (J), Rn (1), Rn (2));
                        Jc (1, P) := (Rn (1) - R0 (1)) / Hd;
                        Jc (2, P) := (Rn (2) - R0 (2)) / Hd;
                     end;
                  end loop;
                  for K in 1 .. 2 loop
                     declare
                        W : constant Real := Huber (R0 (K) / Sigma) / Sigma ** 2;
                     begin
                        for P in 1 .. 12 loop
                           G (P) := G (P) + W * R0 (K) * Jc (K, P);
                           for Q in 1 .. 12 loop
                              A (P, Q) := A (P, Q) + W * Jc (K, P) * Jc (K, Q);
                           end loop;
                        end loop;
                     end;
                  end loop;
                  for P in 1 .. 12 loop
                     for Q in 1 .. 12 loop
                        Bm (P, Q) := Bm (P, Q) + G (P) * G (Q);
                     end loop;
                  end loop;
               end;
            end loop;
            declare
               Lf : Real_Matrix (1 .. 12, 1 .. 12);
               Pd : Boolean;
            begin
               Driver.Numerics.Dense.Cholesky (A, Lf, Pd);
               if Pd then
                  declare
                     Inv : Real_Matrix (1 .. 12, 1 .. 12);
                  begin
                     for P in 1 .. 12 loop
                        declare
                           E : Real_Vector (1 .. 12) := [others => 0.0];
                        begin
                           E (P) := 1.0;
                           declare
                              Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, E);
                           begin
                              for Q in 1 .. 12 loop
                                 Inv (Q, P) := Column (Q);
                              end loop;
                           end;
                        end;
                     end loop;
                     declare
                        Small : constant Real := Real (Used) / Real (Used - 1);
                        V     : constant Real_Matrix := Inv * Bm * Inv;
                     begin
                        for P in 9 .. 12 loop
                           for Q in 9 .. 12 loop
                              Covariance.Append (Small * V (P, Q));
                           end loop;
                        end loop;
                     end;
                  end;
               end if;
            end;
         end if;
      end;
      Free (Index);
   end Joint_Chain;

   procedure Plane_Chain (First, Second : Plane_Point_Array; Link : out Plane_Link) is
      H1, H2 : Mat3;
      F1 : Flag_Array (First'Range);
      F2 : Flag_Array (Second'Range);
      S1, S2 : Real;
      Ok1, Ok2 : Boolean;
      Kept : Natural := 0;
   begin
      Link := (others => <>);
      Plane_Homography (First, H1, F1, S1, Ok1);
      Plane_Homography (Second, H2, F2, S2, Ok2);
      if not (Ok1 and then Ok2) then
         return;
      end if;
      for I in First'Range loop
         Link.First_Fits.Append (F1 (I));
         if F1 (I) then
            Kept := Kept + 1;
         end if;
      end loop;
      for I in Second'Range loop
         Link.Second_Fits.Append (F2 (I));
         if F2 (I) then
            Kept := Kept + 1;
         end if;
      end loop;
      if 2 * Kept <= 12 then
         return;
      end if;
      --  The noise both sets are judged by: the larger of the two
      --  homographies' own (an eye's own lines of sight are exact, a matcher's
      --  answers are not).
      Link.Apart := Real'Max (S1, S2);
      --  The start: the second's points through H1^-1 H2 into the first's
      --  plane, and the similarity nearest that map (its closed form in two
      --  dimensions).
      declare
         G  : constant Mat3 := Inverse (H1) * H2;
         Mx, My, Nx, Ny, A, B, Q : Real := 0.0;
         K  : Natural := 0;
         Gx, Gy : Real;
         Ahead  : Boolean;
      begin
         for I in Second'Range loop
            if F2 (I) then
               Apply (G, Second (I).X, Second (I).Y, Gx, Gy, Ahead);
               if Ahead then
                  K := K + 1;
                  Mx := Mx + Second (I).X;
                  My := My + Second (I).Y;
                  Nx := Nx + Gx;
                  Ny := Ny + Gy;
               end if;
            end if;
         end loop;
         if K < 2 then
            return;
         end if;
         Mx := Mx / Real (K);
         My := My / Real (K);
         Nx := Nx / Real (K);
         Ny := Ny / Real (K);
         for I in Second'Range loop
            if F2 (I) then
               Apply (G, Second (I).X, Second (I).Y, Gx, Gy, Ahead);
               if Ahead then
                  A := A + (Second (I).X - Mx) * (Gx - Nx) + (Second (I).Y - My) * (Gy - Ny);
                  B := B + (Second (I).X - Mx) * (Gy - Ny) - (Second (I).Y - My) * (Gx - Nx);
                  Q := Q + (Second (I).X - Mx) ** 2 + (Second (I).Y - My) ** 2;
               end if;
            end if;
         end loop;
         if Q <= 0.0 or else (A = 0.0 and then B = 0.0) then
            return;
         end if;
         Link.Scale := Sqrt (A * A + B * B) / Q;
         Link.Turn := Arctan (B, A);
         Link.Shift_X := Nx - Link.Scale * (Cos (Link.Turn) * Mx - Sin (Link.Turn) * My);
         Link.Shift_Y := Ny - Link.Scale * (Sin (Link.Turn) * Mx + Cos (Link.Turn) * My);
      end;
      Link.H := H1;
      Plane_Chain_Again (First, Second, Link);
   end Plane_Chain;

   procedure Plane_Chain_Again (First, Second : Plane_Point_Array; Link : in out Plane_Link) is
      X : Real_Array (1 .. 12) :=
        [Link.H (1, 1), Link.H (1, 2), Link.H (1, 3), Link.H (2, 1), Link.H (2, 2), Link.H (2, 3),
         Link.H (3, 1), Link.H (3, 2), Ln (Link.Scale), Link.Turn, Link.Shift_X, Link.Shift_Y];
      Rss    : Real;
      Beyond : Natural;
      Used   : Natural;
      Cov    : Real_Lists.Vector;
   begin
      Link.Found := False;
      Link.Consistent := False;
      if Link.Scale <= 0.0 or else Link.Apart = Real'Last or else Link.Apart <= 0.0 then
         return;
      end if;
      Joint_Chain (First, Second, Link.First_Fits, Link.Second_Fits, X, Link.Apart, Rss, Beyond, Used, Cov);
      if Rss = Real'Last or else Natural (Cov.Length) /= 16 then
         return;
      end if;
      Link.H := Homography_Of (X (1 .. 8));
      Link.Scale := Exp (X (9));
      Link.Turn := X (10);
      Link.Shift_X := X (11);
      Link.Shift_Y := X (12);
      Link.Covariance := Cov;
      Link.Used := Used;
      Link.Sigma := Sqrt (Rss / Real (2 * Used - 12));
      Link.Found := True;
      --  One plane through one eye explains every point the two homographies
      --  kept: no more of their coordinates lie beyond Z of the homographies'
      --  own noise than Z's tail lets chance put there. A link through a
      --  view that is not of both planes leaves the points of one of them
      --  unexplained; the small error each plane's fit adds to its points'
      --  coordinates, which the placement's sigma carries, does not.
      Link.Consistent :=
        not Driver.Robot.Regression.Count_Significant
              (Beyond, 2 * Used, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));
      Link.Beyond := Beyond;
   end Plane_Chain_Again;

   procedure Chain_Placement
     (First_Plane, Second_Plane : Sight_Plane;
      Link      : Plane_Link;
      Placement : out Rigid;
      Scale     : out Real)
   is
      N1, A1, B1, N2, A2, B2 : Vec3;
   begin
      Plane_Axes (First_Plane, N1, A1, B1);
      Plane_Axes (Second_Plane, N2, A2, B2);
      declare
         Rz : constant Mat3 :=
           [[Cos (Link.Turn), -Sin (Link.Turn), 0.0], [Sin (Link.Turn), Cos (Link.Turn), 0.0], [0.0, 0.0, 1.0]];
         Ba : constant Mat3 := [[A1 (1), B1 (1), N1 (1)], [A1 (2), B1 (2), N1 (2)], [A1 (3), B1 (3), N1 (3)]];
         Bb : constant Mat3 := [[A2 (1), B2 (1), N2 (1)], [A2 (2), B2 (2), N2 (2)], [A2 (3), B2 (3), N2 (3)]];
      begin
         Scale := Link.Scale;
         Placement :=
           (Rotation    => Ba * Rz * Transpose (Bb),
            Translation => Link.Shift_X * A1 + Link.Shift_Y * B1
                           + (Plane_Offset (First_Plane) - Link.Scale * Plane_Offset (Second_Plane)) * N1);
      end;
   end Chain_Placement;

end Driver.Robot.Kinematics.Fit;
