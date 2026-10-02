with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Fit is

   --  Samples too large for a stack live on the heap.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   type Flags is array (Positive range <>) of Boolean;

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
      type Real_Access is access Real_Array;
      type Matrix_Access is access Real_Matrix;
      R0     : constant Real_Access := new Real_Array (1 .. Residuals);
      Rn     : constant Real_Access := new Real_Array (1 .. Residuals);
      J      : constant Matrix_Access := new Real_Matrix (1 .. Residuals, 1 .. Parameters);
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
   end Robust_Fit;

   ---------------------------------------------------------------------------
   --  Stage 5: the tracks seen from many keyframes. Every followed point has a
   --  depth along its reference ray, and every sighting must land where the
   --  model puts that point: the reprojection residuals, two per sighting,
   --  over the lens, the joints and every depth at once (the depths solved by
   --  their Schur complement, each a block of its own). Where the epipolar
   --  residuals of a pair cannot tell a flat scene's motions apart, the points
   --  seen from many keyframes can.

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
         Depth  : Real_Array (1 .. Tracks) := [others => 0.0];   --  the log of each track's depth
         Has    : array (1 .. Tracks) of Boolean := [others => False];
         Inlier : array (1 .. S) of Boolean := [others => False];
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
            type Rigid_Array is array (1 .. Frames) of Rigid;
            Views : Rigid_Array;
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
         end Residuals;

         procedure Triangulate is
            Num, Den : Real_Array (1 .. Tracks) := [others => 0.0];
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
               for B of Inlier loop
                  if B then
                     Used := Used + 1;
                  end if;
               end loop;
               exit when 2 * Used <= Count + Tracks;
               declare
                  Index : Real_Access := new Real_Array (1 .. Used);
                  R0, Rn : Real_Access := new Real_Array (1 .. 2 * Used);
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
                  Rn := new Real_Array (1 .. 2 * Used);
                  for I in 1 .. S loop
                     if Inlier (I) then
                        K := K + 1;
                        Index (K) := Real (I);
                     end if;
                  end loop;
                  Residuals (X, Depth, Index, R0.all);
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
                           Residuals (Xp, Depth, Index, Rn.all);
                           for I in 1 .. 2 * Used loop
                              Jp (I, P) := (Rn (I) - R0 (I)) / H;
                           end loop;
                        end;
                     end loop;
                     declare
                        Dp : Real_Array := Depth;
                        H  : constant Real := Sqrt (Real'Model_Epsilon);
                     begin
                        for T in 1 .. Tracks loop
                           Dp (T) := Dp (T) + H * Real'Max (1.0, abs Depth (T));
                        end loop;
                        Residuals (X, Dp, Index, Rn.all);
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
                        C  : Real_Array (1 .. Tracks) := [others => 0.0];
                        Gp : Real_Vector (1 .. Count) := [others => 0.0];
                        Gd : Real_Array (1 .. Tracks) := [others => 0.0];
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
                              Cl    : Real_Array (1 .. Tracks);
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
                                    Dn : Real_Array := Depth;
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
                                       Residuals (Xn, Dn, Index, Rn.all);
                                       declare
                                          Cn : constant Real := Cost (Rn.all);
                                       begin
                                          if Cn < Cost0 then
                                             Improved := Cost0 - Cn > Driver.Conventions.Unchanged_Fraction * Cost0;
                                             X := Xn;
                                             Depth := Dn;
                                             R0.all := Rn.all;
                                             Cost0 := Cn;
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
                        --  At the solution: what the sightings leave each
                        --  parameter uncertain by, the undamped normal equations
                        --  inverted after the two freedoms the images cannot fix
                        --  are removed: an axis point slides along its axis
                        --  (only the point nearest the eye is kept, two numbers in
                        --  the plane across the axis), and every length scales
                        --  together (the depth of the track seen most is held).
                        if not Improved then
                           declare
                              Reduced : constant Positive := 6 + 5 * N;
                              Tm  : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Reduced) :=
                                [others => [others => 0.0]];
                              Anchor : Positive := 1;
                              Seen_Of : array (1 .. Tracks) of Natural := [others => 0];
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
                                 Br : constant Driver.Numerics.Arrays.Real_Matrix := Transpose (Tm) * Bm.all;
                                 Lf : Driver.Numerics.Arrays.Real_Matrix (1 .. Reduced, 1 .. Reduced);
                                 Pd : Boolean;
                              begin
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
                                 if Pd then
                                    for P in 1 .. Reduced loop
                                       declare
                                          E : Real_Vector (1 .. Reduced) := [others => 0.0];
                                       begin
                                          E (P) := 1.0;
                                          Variance (P) := Driver.Numerics.Dense.Cholesky_Solve (Lf, E) (P);
                                       end;
                                    end loop;
                                 end if;
                              end;
                           end;
                        end if;
                        Free (Bm);
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
               Residuals (X, Depth, All_Index, All_R.all);
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
      type Pose_Array is array (1 .. Frames) of Rigid;
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
         procedure Best_Phi (J : Positive; F : Real; W : Vec3; Phi : out Real; Median_Px : out Real) is
            L      : Sight_Line_Array renames Lines (J).all;
            K      : constant Natural := L'Length;
            E1, E2 : Vec3;
            Ga, Gb, Wt, Rs : Real_Array (1 .. K);
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
                  for D of Sphere loop
                     declare
                        Ph, Md : Real;
                     begin
                        Best_Phi (J, F, D, Ph, Md);
                        if Md < Result.Score then
                           Result := (Score => Md, W => D, Phi => Ph);
                        end if;
                     end;
                  end loop;
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
                  D : Real_Array (1 .. S);
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
            Depth : Real_Matrix (1 .. N, 1 .. Tracks) := [others => [others => 0.0]];
            Seen  : array (1 .. N, 1 .. Tracks) of Boolean := [others => [others => False]];
            Num, Den : Real_Matrix (1 .. N, 1 .. Tracks) := [others => [others => 0.0]];
            Rho   : Real_Array (1 .. N) := [others => 1.0];
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
                        Logs : Real_Array (1 .. Tracks);
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
                           return;
                        end if;
                        for J in 1 .. N loop
                           Rho (J) := Exp (Sol (J));
                        end loop;
                     end;
                  elsif N > 1 then
                     Fail ("no point is tracked into the keyframes of two joints");
                     return;
                  end if;
               end;
            end;
            for J in 1 .. N loop
               Joints (Joints'First + J - 1).P := Rho (J) * Joints (Joints'First + J - 1).P;
            end loop;
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
         Inlier    : Flags (1 .. S) := [others => False];
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
            Poses : Pose_Array;
         begin
            Poses_Of (Jx, Poses);
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
                     Poses : Pose_Array;
                  begin
                     Poses_Of (Joints_Of (Xv), Poses);
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
      Normalize (Joints);
      Report.Fitted := Report.Determined;
   end Fit;

   procedure Table
     (Changes   : Driver.Numerics.Arrays.Real_Matrix;
      Sightings : Sighting_Array;
      Joints    : Joint_Array;
      L         : Lens;
      Normal    : out Vec3;
      Sigma     : out Real;
      Found     : out Boolean)
   is
      N      : constant Natural := Joints'Length;
      Tracks : Natural := 0;
   begin
      Normal := [0.0, 0.0, -1.0];
      Sigma := Real'Last;
      Found := False;
      for X of Sightings loop
         Tracks := Natural'Max (Tracks, X.Track);
      end loop;
      if Tracks < 3 then
         return;
      end if;
      declare
         type Vec3_Array is array (Positive range <>) of Vec3;
         type Vec3_Access is access Vec3_Array;
         procedure Free is new Ada.Unchecked_Deallocation (Vec3_Array, Vec3_Access);
         Num, Den : Real_Array (1 .. Tracks) := [others => 0.0];
         Rays     : Vec3_Access := new Vec3_Array (1 .. Tracks);
         Points   : Vec3_Access := new Vec3_Array (1 .. Tracks);
         Count    : Natural := 0;
      begin
         --  Every track's depth along its reference ray, by least squares over
         --  the keyframes it was followed into.
         for X of Sightings loop
            declare
               D : Real_Array (1 .. N);
            begin
               for J in 1 .. N loop
                  D (J) := Changes (Changes'First (1) + X.Frame - 1, Changes'First (2) + J - 1);
               end loop;
               declare
                  T  : constant Rigid := Eye_At (Joints, D);
                  H0 : constant Vec3 := Ray (L, X.U0, X.V0);
                  Bv : constant Vec3 := T.Rotation * Ray (L, X.U, X.V);
                  function Across (V : Vec3) return Vec3 is (V - (Dot (V, Bv) / Dot (Bv, Bv)) * Bv);
               begin
                  Rays (X.Track) := H0;
                  Num (X.Track) := Num (X.Track) + Dot (Across (H0), Across (T.Translation));
                  Den (X.Track) := Den (X.Track) + Dot (Across (H0), Across (H0));
               end;
            end;
         end loop;
         for I in 1 .. Tracks loop
            if Den (I) > 0.0 and then Num (I) / Den (I) > 0.0 then
               Count := Count + 1;
               Points (Count) := (Num (I) / Den (I)) * Rays (I);
            end if;
         end loop;
         Free (Rays);
         if Count < 4 then
            Free (Points);
            return;
         end if;
         declare
            Golden     : constant Real := (3.0 - Sqrt (5.0)) * Ada.Numerics.Pi;
            Directions : constant Natural := 2562;
            Best       : Real := Real'Last;
            Offsets    : Real_Array (1 .. Count);
            Gaps       : Real_Array (1 .. Count);
            Center     : Vec3 := [0.0, 0.0, 0.0];
         begin
            --  The direction whose points lie closest about their median.
            for I in 1 .. Directions loop
               declare
                  Z  : constant Real := 1.0 - 2.0 * (Real (I) - 0.5) / Real (Directions);
                  Rr : constant Real := Sqrt (Real'Max (0.0, 1.0 - Z * Z));
                  Nv : constant Vec3 := [Rr * Cos (Golden * Real (I)), Rr * Sin (Golden * Real (I)), Z];
               begin
                  for K in 1 .. Count loop
                     Offsets (K) := Dot (Nv, Points (K));
                  end loop;
                  declare
                     C : constant Real := Driver.Stats.Median (Offsets);
                  begin
                     for K in 1 .. Count loop
                        Gaps (K) := abs (Offsets (K) - C);
                     end loop;
                     declare
                        Score : constant Real := Driver.Stats.Median (Gaps);
                     begin
                        if Score < Best then
                           Best := Score;
                           Normal := Nv;
                        end if;
                     end;
                  end;
               end;
            end loop;
            --  Huber-weighted least squares from there: the weighted centroid
            --  and the least eigenvector of the weighted scatter, until the
            --  normal turns by less than the unchanged fraction of its own
            --  uncertainty.
            loop
               declare
                  Spread : Real;
                  W      : Real_Array (1 .. Count);
                  Sum_W  : Real := 0.0;
                  Scatter : Mat3 := [others => [others => 0.0]];
                  Values : Vec3;
                  Vectors : Mat3;
                  Next   : Vec3;
               begin
                  for K in 1 .. Count loop
                     Offsets (K) := Dot (Normal, Points (K));
                  end loop;
                  declare
                     C : constant Real := Driver.Stats.Median (Offsets);
                  begin
                     for K in 1 .. Count loop
                        Gaps (K) := Offsets (K) - C;
                     end loop;
                  end;
                  Spread := Noise_Of (Gaps);
                  exit when Spread <= 0.0;
                  Center := [0.0, 0.0, 0.0];
                  for K in 1 .. Count loop
                     W (K) := Huber (Gaps (K) / Spread);
                     Sum_W := Sum_W + W (K);
                     Center := Center + W (K) * Points (K);
                  end loop;
                  Center := (1.0 / Sum_W) * Center;
                  for K in 1 .. Count loop
                     declare
                        D : constant Vec3 := Points (K) - Center;
                     begin
                        Scatter := Scatter + W (K) * Outer (D, D);
                     end;
                  end loop;
                  Symmetric_Eigensystem (Scatter, Values, Vectors);
                  Next := [Vectors (1, 3), Vectors (2, 3), Vectors (3, 3)];
                  if Dot (Next, Normal) < 0.0 then
                     Next := -Next;
                  end if;
                  --  The normal's uncertainty: the spread over the points'
                  --  extent within the plane (the smaller in-plane eigenvalue).
                  Sigma := (if Values (2) > 0.0 then Spread / Sqrt (Values (2)) else Real'Last);
                  declare
                     Turn : constant Real := Arccos (Real'Min (1.0, Dot (Next, Normal)));
                  begin
                     Normal := Next;
                     exit when Turn <= Driver.Conventions.Unchanged_Fraction * Sigma;
                  end;
               end;
            end loop;
            --  Towards the reference eye, at the origin.
            if Dot (Normal, -Center) < 0.0 then
               Normal := -Normal;
            end if;
            Found := Sigma < Real'Last;
         end;
         Free (Points);
      end;
   end Table;

   procedure Resect
     (Points        : Correspondence_Array;
      Width, Height : Positive;
      Pose          : out Rigid;
      L             : out Lens;
      Sigma         : out Real;
      Found         : out Boolean)
   is
      N      : constant Natural := Points'Length;
      Inlier : array (1 .. N) of Boolean := [others => True];
      Weight : constant Real_Array (1 .. N) := [others => 1.0];
      P_Mat  : Real_Matrix (1 .. 3, 1 .. 4) := [others => [others => 0.0]];

      function Pt (I : Positive) return Correspondence is (Points (Points'First + I - 1));

      --  The weighted direct linear transform: the projection matrix as the
      --  least eigenvector of the conditioned equations, the points and the
      --  pixels first centred and scaled to unit spread.
      procedure Linear is
         Cx, Cy : Real := 0.0;
         C3     : Vec3 := [0.0, 0.0, 0.0];
         Sp, Sx : Real := 0.0;
         Sw     : Real := 0.0;
      begin
         for I in 1 .. N loop
            if Inlier (I) then
               Sw := Sw + Weight (I);
               Cx := Cx + Weight (I) * Pt (I).U;
               Cy := Cy + Weight (I) * Pt (I).V;
               C3 := C3 + Weight (I) * Pt (I).X;
            end if;
         end loop;
         Cx := Cx / Sw;
         Cy := Cy / Sw;
         C3 := (1.0 / Sw) * C3;
         for I in 1 .. N loop
            if Inlier (I) then
               Sp := Sp + Weight (I) * ((Pt (I).U - Cx) ** 2 + (Pt (I).V - Cy) ** 2);
               Sx := Sx + Weight (I) * Dot (Pt (I).X - C3, Pt (I).X - C3);
            end if;
         end loop;
         Sp := Sqrt (Sp / Sw);
         Sx := Sqrt (Sx / Sw);
         declare
            A : Real_Matrix (1 .. 12, 1 .. 12) := [others => [others => 0.0]];
            Values  : Real_Vector (1 .. 12);
            Vectors : Real_Matrix (1 .. 12, 1 .. 12);
         begin
            for I in 1 .. N loop
               if Inlier (I) then
                  declare
                     Xn : constant Vec3 := (1.0 / Sx) * (Pt (I).X - C3);
                     Xh : constant Real_Vector (1 .. 4) := [Xn (1), Xn (2), Xn (3), 1.0];
                     Un : constant Real := (Pt (I).U - Cx) / Sp;
                     Vn : constant Real := (Pt (I).V - Cy) / Sp;
                     R1, R2 : Real_Vector (1 .. 12) := [others => 0.0];
                  begin
                     for K in 1 .. 4 loop
                        R1 (K) := Xh (K);
                        R1 (8 + K) := -Un * Xh (K);
                        R2 (4 + K) := Xh (K);
                        R2 (8 + K) := -Vn * Xh (K);
                     end loop;
                     for P in 1 .. 12 loop
                        for Q in P .. 12 loop
                           A (P, Q) := A (P, Q) + Weight (I) * (R1 (P) * R1 (Q) + R2 (P) * R2 (Q));
                        end loop;
                     end loop;
                  end;
               end if;
            end loop;
            for P in 1 .. 12 loop
               for Q in P + 1 .. 12 loop
                  A (Q, P) := A (P, Q);
               end loop;
            end loop;
            Eigensystem (A, Values, Vectors);
            declare
               Least : Positive := 1;
               Pn    : Real_Matrix (1 .. 3, 1 .. 4);
               --  Undo the conditioning: P = Tp^-1 Pn Tx.
               Tp_Inv : constant Real_Matrix (1 .. 3, 1 .. 3) := [[Sp, 0.0, Cx], [0.0, Sp, Cy], [0.0, 0.0, 1.0]];
               Tx     : constant Real_Matrix (1 .. 4, 1 .. 4) :=
                 [[1.0 / Sx, 0.0, 0.0, -C3 (1) / Sx], [0.0, 1.0 / Sx, 0.0, -C3 (2) / Sx],
                  [0.0, 0.0, 1.0 / Sx, -C3 (3) / Sx], [0.0, 0.0, 0.0, 1.0]];
            begin
               for K in 2 .. 12 loop
                  if Values (K) < Values (Least) then
                     Least := K;
                  end if;
               end loop;
               for R in 1 .. 3 loop
                  for C in 1 .. 4 loop
                     Pn (R, C) := Vectors (4 * (R - 1) + C, Least);
                  end loop;
               end loop;
               P_Mat := Tp_Inv * Pn * Tx;
            end;
         end;
      end Linear;

      --  P = K [R | t]: K from the upper Cholesky factor of M M^T (through the
      --  exchange of rows and columns), R = K^-1 M, t = K^-1 p4.
      procedure Factor (K_Out : out Mat3; R_Out : out Mat3; T_Out : out Vec3) is
         Mm : Mat3;
         P4 : Vec3;
      begin
         for R in 1 .. 3 loop
            for C in 1 .. 3 loop
               Mm (R, C) := P_Mat (R, C);
            end loop;
            P4 (R) := P_Mat (R, 4);
         end loop;
         --  The scale\x27s sign: the points lie in front, so the depth row of the
         --  first point projects positive.
         if Mm (3, 1) * Pt (1).X (1) + Mm (3, 2) * Pt (1).X (2) + Mm (3, 3) * Pt (1).X (3) + P4 (3) < 0.0 then
            Mm := -Mm;
            P4 := -P4;
         end if;
         declare
            Aa : constant Mat3 := Mm * Transpose (Mm);
            J  : constant Mat3 := [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
            Lf : Real_Matrix (1 .. 3, 1 .. 3);
            Pd : Boolean;
         begin
            Driver.Numerics.Dense.Cholesky (J * Aa * J, Lf, Pd);
            K_Out := J * Lf * J;
            if K_Out (3, 3) /= 0.0 then
               K_Out := (1.0 / K_Out (3, 3)) * K_Out;
            end if;
            declare
               K_Inv : constant Mat3 := Inverse (K_Out);
               S     : constant Real := 1.0 / Sqrt (abs Determinant (K_Inv * Mm)) ** (1.0 / 3.0);
            begin
               R_Out := Driver.Numerics.Orthonormalize (S * (K_Inv * Mm));
               T_Out := S * (K_Inv * P4);
            end;
         end;
      end Factor;

      K0 : Mat3;
      R0 : Mat3;
      T0 : Vec3;
   begin
      Pose := Identity;
      L := (Fx | Fy => 1.0, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, others => 0.0);
      Sigma := Real'Last;
      Found := False;
      if N < 7 then
         return;
      end if;
      Linear;
      Factor (K0, R0, T0);
      if K0 (1, 1) <= 0.0 or else K0 (2, 2) <= 0.0 then
         return;
      end if;
      --  Everything by robust least squares: the turn (a small rotation on
      --  the linear one), the place, then log Fx, log Fy, Cx, Cy, K1, K2.
      declare
         Base_R : Mat3 := R0;
         X      : Real_Array (1 .. 12) :=
           [0.0, 0.0, 0.0, T0 (1), T0 (2), T0 (3), Ln (K0 (1, 1)), Ln (K0 (2, 2)), K0 (1, 3), K0 (2, 3), 0.0, 0.0];
         Changed : Natural := Natural'Last;

         function Lens_Of (Xv : Real_Array) return Lens is
           ((Fx => Exp (Xv (7)), Fy => Exp (Xv (8)), Cx => Xv (9), Cy => Xv (10), K1 => Xv (11), K2 => Xv (12)));

         function Pose_Of (Xv : Real_Array) return Rigid is
           ((Rotation => Driver.Numerics.Exp ([Xv (1), Xv (2), Xv (3)]) * Base_R, Translation => [Xv (4), Xv (5), Xv (6)]));

         procedure Residual (Xv : Real_Array; I : Positive; Du, Dv : out Real) is
            Lx : constant Lens := Lens_Of (Xv);
            U, V : Real;
            Ahead : Boolean;
         begin
            Project (Lx, Pose_Of (Xv) * Pt (I).X, U, V, Ahead);
            Du := U - Pt (I).U;
            Dv := V - Pt (I).V;
         end Residual;
      begin
         loop
            declare
               All_R : Real_Array (1 .. 2 * N);
               Used  : Natural := 0;
               Now_Changed : Natural := 0;
               S     : Real;
            begin
               for I in 1 .. N loop
                  Residual (X, I, All_R (2 * I - 1), All_R (2 * I));
               end loop;
               S := Noise_Of (All_R);
               for I in 1 .. N loop
                  declare
                     Fits : constant Boolean := S > 0.0
                       and then not Driver.Uncertain.Significant (All_R (2 * I - 1), S)
                       and then not Driver.Uncertain.Significant (All_R (2 * I), S);
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
               Sigma := S;
               exit when Used < 7 or else Now_Changed >= Changed or else (Now_Changed = 0 and then Changed /= Natural'Last);
               Changed := Now_Changed;
               declare
                  Index : array (1 .. Used) of Positive;
                  K     : Natural := 0;

                  procedure Evaluate (Xv : Real_Array; R : out Real_Array) is
                  begin
                     for J in 1 .. Used loop
                        Residual (Xv, Index (J), R (R'First + 2 * J - 2), R (R'First + 2 * J - 1));
                     end loop;
                  end Evaluate;

                  procedure Solve is new Robust_Fit (12, 2 * Used, Evaluate);
               begin
                  for I in 1 .. N loop
                     if Inlier (I) then
                        K := K + 1;
                        Index (K) := I;
                     end if;
                  end loop;
                  Solve (X, S);
                  Base_R := Pose_Of (X).Rotation;
                  X (1 .. 3) := [0.0, 0.0, 0.0];
               end;
            end;
         end loop;
         Pose := Pose_Of (X);
         L := Lens_Of (X);
         Found := Sigma < Real'Last and then L.Fx > 0.0 and then L.Fy > 0.0;
      end;
   end Resect;

end Driver.Robot.Kinematics.Fit;
