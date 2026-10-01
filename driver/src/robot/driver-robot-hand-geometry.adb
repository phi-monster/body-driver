with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Numerics.Dense;
with Driver.Stats;

package body Driver.Robot.Hand.Geometry is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Half : constant := 0.5;

   subtype Mat2 is Real_Matrix (1 .. 2, 1 .. 2);

   procedure Perpendicular_Basis (U : Vec3; E1, E2 : out Vec3);
   --  Two unit vectors completing U to an orthonormal frame.

   procedure Perpendicular_Basis (U : Vec3; E1, E2 : out Vec3) is
      Smallest : Positive := U'First;
   begin
      --  Cross U with the axis it is least aligned with: well conditioned for any U.
      for I in U'Range loop
         if abs U (I) < abs U (Smallest) then
            Smallest := I;
         end if;
      end loop;
      declare
         Axis : Vec3 := Zero3;
      begin
         Axis (Smallest) := 1.0;
         E1 := Unit (Cross (U, Axis));
         E2 := Cross (U, E1);
      end;
   end Perpendicular_Basis;

   function Inverse2 (M : Mat2; Ok : out Boolean) return Mat2;

   function Inverse2 (M : Mat2; Ok : out Boolean) return Mat2 is
      Det : constant Real := M (1, 1) * M (2, 2) - M (1, 2) * M (2, 1);
      Scale : constant Real := abs M (1, 1) * abs M (2, 2) + abs M (1, 2) * abs M (2, 1);
   begin
      --  A determinant lost in the round-off of its own terms is no determinant.
      Ok := abs Det > Real'Model_Epsilon * Scale;
      if not Ok then
         return [others => [others => 0.0]];
      end if;
      return [[M (2, 2) / Det, -M (1, 2) / Det], [-M (2, 1) / Det, M (1, 1) / Det]];
   end Inverse2;

   procedure Meet (Rays : Ray_Array; Point : out Point_Estimate; Ok : out Boolean) is
      X        : Vec3 := Zero3;
      Info     : Mat3 := [others => [others => 0.0]];
      Previous : Real := Real'Last;

      --  One weighted least-squares pass with the rays' perpendicular
      --  uncertainty taken where they pass X (Weighted) or ignored (not Weighted).
      procedure Solve (Weighted : Boolean; Solved : out Boolean);

      procedure Solve (Weighted : Boolean; Solved : out Boolean) is
         A : Mat3 := [others => [others => 0.0]];
         B : Vec3 := Zero3;
      begin
         Solved := False;
         for R of Rays loop
            declare
               U      : constant Vec3 := R.Direction.Unit_Vector;
               O      : constant Vec3 := R.Origin.Mean;
               E1, E2 : Vec3;
               W      : Mat3;
            begin
               Perpendicular_Basis (U, E1, E2);
               if Weighted then
                  declare
                     T : constant Real := (X - O) * U;
                     M : constant Mat3 := R.Origin.Covariance + ((T * R.Direction.Sigma) ** 2) * Identity3;
                     C : constant Mat2 := [[E1 * (M * E1), E1 * (M * E2)], [E2 * (M * E1), E2 * (M * E2)]];
                     Inv_Ok : Boolean;
                     Ci : constant Mat2 := Inverse2 (C, Inv_Ok);
                  begin
                     if T <= 0.0 or else not Inv_Ok then
                        return;
                     end if;
                     W := Ci (1, 1) * Outer (E1, E1) + Ci (1, 2) * Outer (E1, E2) + Ci (2, 1) * Outer (E2, E1)
                       + Ci (2, 2) * Outer (E2, E2);
                  end;
               else
                  W := Outer (E1, E1) + Outer (E2, E2);
               end if;
               A := A + W;
               B := B + W * O;
            end;
         end loop;
         declare
            L  : Real_Matrix (1 .. 3, 1 .. 3);
            PD : Boolean;
         begin
            Driver.Numerics.Dense.Cholesky (A, L, PD);
            if not PD then
               return;
            end if;
            X := Driver.Numerics.Dense.Cholesky_Solve (L, B);
            Info := A;
            Solved := True;
         end;
      end Solve;

      Solved : Boolean;
   begin
      Point := (others => <>);
      Ok := False;
      if Rays'Length < 2 then
         return;
      end if;
      for R of Rays loop
         if not Known (R.Origin) or else R.Direction.Sigma >= Real'Last then
            return;
         end if;
      end loop;
      Solve (Weighted => False, Solved => Solved);
      if not Solved then
         return;
      end if;
      --  Reweight at the current point until a pass moves it by a negligible
      --  part of its own uncertainty, or stops shrinking the move (a strictly
      --  shrinking sequence of floating-point numbers is finite).
      loop
         declare
            Before : constant Vec3 := X;
            Move   : Real;
         begin
            Solve (Weighted => True, Solved => Solved);
            if not Solved then
               return;
            end if;
            Move := (X - Before) * (Info * (X - Before));
            exit when Move <= Driver.Conventions.Unchanged_Fraction ** 2 or else Move >= Previous;
            Previous := Move;
         end;
      end loop;
      Point := (Mean => X, Covariance => Inverse (Info));
      Ok := True;
   end Meet;

   function Height (P : Plane_Estimate; X : Vec3) return Real is (P.Normal * (X - P.Centre));

   function Height_Sigma (P : Plane_Estimate; X : Vec3) return Real is
   begin
      if not Known (P) then
         return Real'Last;
      end if;
      declare
         A : constant Real := P.Tangent_1 * (X - P.Centre);
         B : constant Real := P.Tangent_2 * (X - P.Centre);
      begin
         return Sqrt (P.Offset_Sigma ** 2 + A * A * P.Tilt_11 + 2.0 * A * B * P.Tilt_12 + B * B * P.Tilt_22);
      end;
   end Height_Sigma;

   function Height (P : Plane_Estimate; X : Point_Estimate) return Estimate is
   begin
      if not Known (P) or else not Known (X) then
         return (Value => Height (P, X.Mean), Sigma => Real'Last);
      end if;
      return (Value => Height (P, X.Mean),
              Sigma => Sqrt (Height_Sigma (P, X.Mean) ** 2 + P.Normal * (X.Covariance * P.Normal)));
   end Height;

   procedure Intersect (P : Plane_Estimate; R : Ray_Estimate; Point : out Point_Estimate; Distance : out Estimate;
                        Ok : out Boolean)
   is
      U      : constant Vec3 := R.Direction.Unit_Vector;
      O      : constant Vec3 := R.Origin.Mean;
      Facing : constant Real := P.Normal * U;
   begin
      Point := (others => <>);
      Distance := Unknown;
      Ok := False;
      --  A ray along the plane meets it nowhere, or everywhere.
      if abs Facing <= Real'Model_Epsilon then
         return;
      end if;
      declare
         T : constant Real := (P.Normal * (P.Centre - O)) / Facing;
         X : constant Vec3 := O + T * U;
      begin
         if T <= 0.0 then
            return;
         end if;
         Ok := True;
         if not Known (P) or else not Known (R.Origin) or else R.Direction.Sigma >= Real'Last then
            Point := (Mean => X, Covariance => [others => [others => Real'Last]]);
            Distance := (Value => T, Sigma => Real'Last);
            return;
         end if;
         declare
            --  Moving the origin or turning the ray slides the point within
            --  the plane along U; moving the plane slides it along U as well.
            J : constant Mat3 := Identity3 - (1.0 / Facing) * Outer (U, P.Normal);
            Turn : constant Mat3 := (R.Direction.Sigma ** 2) * (Identity3 - Outer (U, U));
            Along_Plane : constant Real := Height_Sigma (P, X) / Facing;
            C : constant Mat3 :=
              J * R.Origin.Covariance * Transpose (J) + (T * T) * (J * Turn * Transpose (J))
              + (Along_Plane ** 2) * Outer (U, U);
            --  The distance moves with the origin and the plane along U.
            Dt : constant Vec3 := (-1.0 / Facing) * P.Normal;
         begin
            Point := (Mean => X, Covariance => C);
            Distance := (Value => T,
                         Sigma => Sqrt (Dt * (R.Origin.Covariance * Dt) + Along_Plane ** 2
                                        + (T * R.Direction.Sigma / Facing) ** 2 * (1.0 - Facing ** 2)));
         end;
      end;
   end Intersect;

   procedure Fit (Points : Point_Array; Use_Point : Flag_Array; P : out Plane_Estimate; Ok : out Boolean) is
      Count  : Natural := 0;
      N      : Vec3 := Zero3;
      Centre : Vec3 := Zero3;
      Weight : array (Points'Range) of Real := [others => 0.0];

      --  Weighted centroid and the normal of the weighted scatter; the
      --  smallest eigenvalue is last (Eigensystem sorts them descending).
      procedure Solve_Normal;

      procedure Solve_Normal is
         Total   : Real := 0.0;
         Sum     : Vec3 := Zero3;
         Scatter : Mat3 := [others => [others => 0.0]];
         Values  : Real_Vector (1 .. 3);
         Vectors : Real_Matrix (1 .. 3, 1 .. 3);
      begin
         for I in Points'Range loop
            if Use_Point (I) then
               Total := Total + Weight (I);
               Sum := Sum + Weight (I) * Points (I).Mean;
            end if;
         end loop;
         Centre := Sum / Total;
         for I in Points'Range loop
            if Use_Point (I) then
               Scatter := Scatter + Weight (I) * Outer (Points (I).Mean - Centre, Points (I).Mean - Centre);
            end if;
         end loop;
         Eigensystem ((Scatter + Transpose (Scatter)) * Half, Values, Vectors);
         N := [Vectors (1, 3), Vectors (2, 3), Vectors (3, 3)];
      end Solve_Normal;

      Unknowns : constant := 3;   --  the plane's offset and its two tilts
   begin
      P := (others => <>);
      Ok := False;
      for I in Points'Range loop
         if Use_Point (I) then
            if not Known (Points (I)) then
               return;
            end if;
            Count := Count + 1;
            Weight (I) := 1.0;
         end if;
      end loop;
      --  One residual beyond the unknowns measures the scatter.
      if Count <= Unknowns then
         return;
      end if;
      Solve_Normal;
      declare
         Previous : Real := Real'Last;
      begin
         --  Reweight by each point's variance along the current normal until
         --  the normal stops turning (or stops turning less).
         loop
            for I in Points'Range loop
               if Use_Point (I) then
                  Weight (I) := 1.0 / Real'Max (N * (Points (I).Covariance * N), Real'Model_Small);
               end if;
            end loop;
            declare
               Before : constant Vec3 := N;
               Turn   : Real;
            begin
               Solve_Normal;
               if N * Before < 0.0 then
                  N := -N;
               end if;
               Turn := abs (N - Before);
               exit when Turn <= Real'Model_Epsilon or else Turn >= Previous;
               Previous := Turn;
            end;
         end loop;
      end;
      declare
         E1, E2  : Vec3;
         Chi     : Real := 0.0;
         Total   : Real := 0.0;
         M       : Mat2 := [others => [others => 0.0]];
         Inv_Ok  : Boolean;
         Scale   : Real;
      begin
         Perpendicular_Basis (N, E1, E2);
         for I in Points'Range loop
            if Use_Point (I) then
               declare
                  D : constant Vec3 := Points (I).Mean - Centre;
                  A : constant Real := D * E1;
                  B : constant Real := D * E2;
               begin
                  Chi := Chi + Weight (I) * (D * N) ** 2;
                  Total := Total + Weight (I);
                  M := M + Weight (I) * Mat2'([[A * A, A * B], [A * B, B * B]]);
               end;
            end if;
         end loop;
         Scale := Chi / Real (Count - Unknowns);
         declare
            Mi : constant Mat2 := Inverse2 (M, Inv_Ok);
         begin
            if not Inv_Ok then
               return;
            end if;
            P := (Centre       => Centre,
                  Normal       => N,
                  Tangent_1    => E1,
                  Tangent_2    => E2,
                  Offset_Sigma => Sqrt (Scale / Total),
                  Tilt_11      => Scale * Mi (1, 1),
                  Tilt_12      => Scale * Mi (1, 2),
                  Tilt_22      => Scale * Mi (2, 2),
                  Points       => Count,
                  Scatter      => Scale);
            Ok := True;
         end;
      end;
   end Fit;

   procedure Fit_Robust
     (Points    : Point_Array;
      Start     : Flag_Array;
      P         : out Plane_Estimate;
      Inliers   : out Flag_Array;
      Ok        : out Boolean)
   is
      Chosen : Flag_Array := Start;
   begin
      Inliers := Start;
      P := (others => <>);
      Ok := False;
      --  Re-selections settle in a few passes in practice; the cap of one
      --  pass per point only guarantees an end, and a selection that has not
      --  settled by then is reported as no fit.
      for Pass in Points'Range loop
         Fit (Points, Chosen, P, Ok);
         if not Ok then
            return;
         end if;
         declare
            Z      : Real_Array (1 .. P.Points);
            K      : Natural := 0;
            Next   : Flag_Array (Points'Range);
            Centre : Real;
            Noise  : Real;
            function Sigma_Along_Normal (I : Positive) return Real is
              (Sqrt (Real'Max (P.Normal * (Points (I).Covariance * P.Normal), Real'Model_Small)));
         begin
            --  Heights in units of each point's own sigma. The bulk of the
            --  selection sits at their median, which a plane still pulled by
            --  outliers does not pass through; the robust scale around it
            --  rescales the points' sigmas to how they actually scatter.
            for I in Points'Range loop
               if Chosen (I) then
                  K := K + 1;
                  Z (K) := Height (P, Points (I).Mean) / Sigma_Along_Normal (I);
               end if;
            end loop;
            Centre := Driver.Stats.Median (Z);
            Noise := Driver.Stats.Robust_Sigma (Z);
            for I in Points'Range loop
               Next (I) := not Significant
                 (Height (P, Points (I).Mean) - Centre * Sigma_Along_Normal (I),
                  Sqrt ((Noise * Sigma_Along_Normal (I)) ** 2 + Height_Sigma (P, Points (I).Mean) ** 2));
            end loop;
            if Next = Chosen then
               Inliers := Chosen;
               return;
            end if;
            Chosen := Next;
         end;
      end loop;
      Ok := False;
   end Fit_Robust;

   procedure Orient (P : in out Plane_Estimate; Towards : Vec3) is
   begin
      if Height (P, Towards) < 0.0 then
         P.Normal := -P.Normal;
         P.Tangent_2 := -P.Tangent_2;
      end if;
   end Orient;

end Driver.Robot.Hand.Geometry;
