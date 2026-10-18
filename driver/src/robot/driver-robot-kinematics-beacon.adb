with Ada.Numerics.Long_Elementary_Functions;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Beacon is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;

   type Parameters is array (1 .. Terms) of Real;
   type Flags is array (Positive range <>) of Boolean;

   --  The rotation is carried as a base the fit returns to after every accepted step and a small turn about the
   --  camera's own axes on top of it (terms 5 to 7), so the turn stays small and its covariance is about those axes.
   function Lens_Of (P : Parameters) return Fit.Lens is
     ((Fx => Exp (P (1)), Fy => Exp (P (2)), Cx => P (3), Cy => P (4), K1 => 0.0, K2 => 0.0));

   function Pose_Of (Base : Mat3; P : Parameters) return Rigid is
     ((Rotation => Base * Driver.Numerics.Exp (Vec3'[P (5), P (6), P (7)]), Translation => Vec3'[P (8), P (9), P (10)]));

   function Beacon_Of (P : Parameters) return Vec3 is (Vec3'[P (11), P (12), P (13)]);

   --  The sightings kept, each's residuals in units of its sigma, u then v; Ahead False when the camera puts some
   --  sighting's beacon behind it.
   procedure Residuals
     (Seen : Sighting_Array; Keep : Flags; Base : Mat3; P : Parameters; R : out Real_Vector; Ahead : out Boolean)
   is
      L    : constant Fit.Lens := Lens_Of (P);
      Pose : constant Rigid := Pose_Of (Base, P);
      A    : constant Vec3 := Beacon_Of (P);
      K    : Natural := 0;
   begin
      Ahead := True;
      for I in Seen'Range loop
         if Keep (I) then
            declare
               X     : constant Vec3 := Seen (I).Carrier * A;
               In_Eye : constant Vec3 := Transpose (Pose.Rotation) * (X - Pose.Translation);
               U, V  : Real;
               Front : Boolean;
            begin
               Fit.Project (L, In_Eye, U, V, Front);
               Ahead := Ahead and then Front;
               R (R'First + K) := (Seen (I).U - U) / Seen (I).Sigma;
               R (R'First + K + 1) := (Seen (I).V - V) / Seen (I).Sigma;
               K := K + 2;
            end;
         end if;
      end loop;
   end Residuals;

   function Count (Keep : Flags) return Natural is
      N : Natural := 0;
   begin
      for K of Keep loop
         if K then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Count;

   --  The direct linear start: the projection that takes the carriers' places (the beacon at the arm's eye) to the
   --  centres, by the least eigenvector of the normalized equations (Hartley), split into a lens and a pose by an RQ
   --  of its left 3 x 3 (Gram-Schmidt from its last row). Ok False with fewer than six sightings, or when the start
   --  puts the carriers behind the camera or its focal lengths are not positive.
   procedure Start (Seen : Sighting_Array; Keep : Flags; Base : out Mat3; P : out Parameters; Ok : out Boolean) is
      N      : constant Natural := Count (Keep);
      C3     : Vec3 := [0.0, 0.0, 0.0];
      C2u, C2v : Real := 0.0;
      S3, S2 : Real := 0.0;
   begin
      Base := Identity3;
      P := [others => 0.0];
      Ok := False;
      if N < 6 then
         return;
      end if;
      for I in Seen'Range loop
         if Keep (I) then
            C3 := C3 + Seen (I).Carrier.Translation;
            C2u := C2u + Seen (I).U;
            C2v := C2v + Seen (I).V;
         end if;
      end loop;
      C3 := (1.0 / Real (N)) * C3;
      C2u := C2u / Real (N);
      C2v := C2v / Real (N);
      for I in Seen'Range loop
         if Keep (I) then
            S3 := S3 + abs (Seen (I).Carrier.Translation - C3);
            S2 := S2 + Sqrt ((Seen (I).U - C2u) ** 2 + (Seen (I).V - C2v) ** 2);
         end if;
      end loop;
      if S3 <= 0.0 or else S2 <= 0.0 then
         return;
      end if;
      S3 := Sqrt (3.0) * Real (N) / S3;
      S2 := Sqrt (2.0) * Real (N) / S2;
      declare
         AtA : Real_Matrix (1 .. 12, 1 .. 12) := [others => [others => 0.0]];
         Values  : Real_Vector (1 .. 12);
         Vectors : Real_Matrix (1 .. 12, 1 .. 12);
         Pn, Pm  : Real_Matrix (1 .. 3, 1 .. 4);
         T3      : Real_Matrix (1 .. 4, 1 .. 4) := [others => [others => 0.0]];
         T2i     : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
         M3      : Mat3;
      begin
         for I in Seen'Range loop
            if Keep (I) then
               declare
                  X  : constant Vec3 := S3 * (Seen (I).Carrier.Translation - C3);
                  Un : constant Real := S2 * (Seen (I).U - C2u);
                  Vn : constant Real := S2 * (Seen (I).V - C2v);
                  R1 : constant Real_Vector (1 .. 12) :=
                    [X (1), X (2), X (3), 1.0, 0.0, 0.0, 0.0, 0.0, -Un * X (1), -Un * X (2), -Un * X (3), -Un];
                  R2 : constant Real_Vector (1 .. 12) :=
                    [0.0, 0.0, 0.0, 0.0, X (1), X (2), X (3), 1.0, -Vn * X (1), -Vn * X (2), -Vn * X (3), -Vn];
               begin
                  for J in 1 .. 12 loop
                     for L in 1 .. 12 loop
                        AtA (J, L) := AtA (J, L) + R1 (J) * R1 (L) + R2 (J) * R2 (L);
                     end loop;
                  end loop;
               end;
            end if;
         end loop;
         Eigensystem (AtA, Values, Vectors);
         for J in 1 .. 3 loop
            for L in 1 .. 4 loop
               Pn (J, L) := Vectors (4 * (J - 1) + L, Vectors'Last (2));
            end loop;
         end loop;
         for J in 1 .. 3 loop
            T3 (J, J) := S3;
            T3 (J, 4) := -S3 * C3 (J);
         end loop;
         T3 (4, 4) := 1.0;
         T2i (1, 1) := 1.0 / S2;
         T2i (2, 2) := 1.0 / S2;
         T2i (1, 3) := C2u;
         T2i (2, 3) := C2v;
         T2i (3, 3) := 1.0;
         Pm := T2i * Pn * T3;
         for J in 1 .. 3 loop
            for L in 1 .. 3 loop
               M3 (J, L) := Pm (J, L);
            end loop;
         end loop;
         if Determinant (M3) < 0.0 then
            Pm := -Pm;
            M3 := -M3;
         end if;
         declare
            Row1 : constant Vec3 := [M3 (1, 1), M3 (1, 2), M3 (1, 3)];
            Row2 : constant Vec3 := [M3 (2, 1), M3 (2, 2), M3 (2, 3)];
            Row3 : constant Vec3 := [M3 (3, 1), M3 (3, 2), M3 (3, 3)];
            K33  : constant Real := abs Row3;
            R3   : constant Vec3 := (1.0 / K33) * Row3;
            K23  : constant Real := Row2 * R3;
            M2   : constant Vec3 := Row2 - K23 * R3;
            K22  : constant Real := abs M2;
            R2   : constant Vec3 := (1.0 / K22) * M2;
            K13  : constant Real := Row1 * R3;
            K12  : constant Real := Row1 * R2;
            M1   : constant Vec3 := Row1 - K13 * R3 - K12 * R2;
            K11  : constant Real := abs M1;
            R1   : constant Vec3 := (1.0 / K11) * M1;
            Centre : constant Vec3 := -(Inverse (M3) * Vec3'[Pm (1, 4), Pm (2, 4), Pm (3, 4)]);
            Ahead  : Natural := 0;
         begin
            if K11 <= 0.0 or else K22 <= 0.0 or else K33 <= 0.0 then
               return;
            end if;
            --  The camera's axes are the rows of the world-to-camera turn.
            Base := Transpose (Mat3'[[R1 (1), R1 (2), R1 (3)], [R2 (1), R2 (2), R2 (3)], [R3 (1), R3 (2), R3 (3)]]);
            for I in Seen'Range loop
               if Keep (I) and then R3 * (Seen (I).Carrier.Translation - Centre) > 0.0 then
                  Ahead := Ahead + 1;
               end if;
            end loop;
            if 2 * Ahead <= N then
               return;
            end if;
            P (1) := Log (K11 / K33);
            P (2) := Log (K22 / K33);
            P (3) := K13 / K33;
            P (4) := K23 / K33;
            P (8 .. 10) := [Centre (1), Centre (2), Centre (3)];
            Ok := True;
         end;
      end;
   end Start;

   --  Damped Gauss-Newton from P (Levenberg-Marquardt), central differences for the Jacobian: each step's damping
   --  grows by Damping_Ratio while a step raises the cost and falls by it after one that lowers it; the fit ends when a
   --  step lowers the cost by less than its rounding, or no damping finds one that lowers it. Normal holds the
   --  undamped normal equations at the end.
   procedure Converge
     (Seen : Sighting_Array; Keep : Flags; Base : in out Mat3; P : in out Parameters; Normal : out Real_Matrix;
      Ok : out Boolean)
   is
      M      : constant Natural := 2 * Count (Keep);
      Eps3   : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);
      Damping_Ratio : constant Real := 10.0;
      Lambda : Real := Real'Model_Epsilon ** 0.5;
      R      : Real_Vector (1 .. M);
      Ahead  : Boolean;
      Cost   : Real;
      Scale  : Real := 0.0;   --  the carriers' spread, for the steps of lengths
   begin
      Ok := False;
      for I in Seen'Range loop
         if Keep (I) then
            Scale := Real'Max (Scale, abs (Seen (I).Carrier.Translation - Vec3'[P (8), P (9), P (10)]));
         end if;
      end loop;
      Residuals (Seen, Keep, Base, P, R, Ahead);
      if not Ahead then
         return;
      end if;
      Cost := R * R;
      loop
         declare
            J   : Real_Matrix (1 .. M, 1 .. Terms);
            JtJ : Real_Matrix (1 .. Terms, 1 .. Terms);
            Jtr : Real_Vector (1 .. Terms);
         begin
            for T in 1 .. Terms loop
               declare
                  H      : constant Real :=
                    (case T is when 1 .. 2 | 5 .. 7 => Eps3, when 3 .. 4 => Eps3 * Real'Max (1.0, abs P (T)),
                     when others => Eps3 * Real'Max (Scale, abs P (T)));
                  Up, Down : Parameters := P;
                  Ru, Rd : Real_Vector (1 .. M);
                  Au, Ad : Boolean;
               begin
                  Up (T) := P (T) + H;
                  Down (T) := P (T) - H;
                  Residuals (Seen, Keep, Base, Up, Ru, Au);
                  Residuals (Seen, Keep, Base, Down, Rd, Ad);
                  for I in 1 .. M loop
                     J (I, T) := (Ru (I) - Rd (I)) / (2.0 * H);
                  end loop;
               end;
            end loop;
            JtJ := Transpose (J) * J;
            Jtr := Transpose (J) * R;
            Normal := JtJ;
            declare
               Better : Boolean := False;
            begin
               loop
                  declare
                     A        : Real_Matrix := JtJ;
                     Lower    : Real_Matrix (1 .. Terms, 1 .. Terms);
                     Definite : Boolean;
                  begin
                     for T in 1 .. Terms loop
                        A (T, T) := A (T, T) * (1.0 + Lambda);
                     end loop;
                     Driver.Numerics.Dense.Cholesky (A, Lower, Definite);
                     if Definite then
                        declare
                           Step  : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, -Jtr);
                           Trial : Parameters := P;
                           Rt    : Real_Vector (1 .. M);
                           At_Front : Boolean;
                        begin
                           for T in 1 .. Terms loop
                              Trial (T) := P (T) + Step (T);
                           end loop;
                           Residuals (Seen, Keep, Base, Trial, Rt, At_Front);
                           if At_Front and then Rt * Rt < Cost then
                              declare
                                 Fell : constant Real := Cost - Rt * Rt;
                              begin
                                 --  The turn folds into the base and starts again from nought.
                                 Base := Base * Driver.Numerics.Exp (Vec3'[Trial (5), Trial (6), Trial (7)]);
                                 Trial (5 .. 7) := [0.0, 0.0, 0.0];
                                 P := Trial;
                                 Residuals (Seen, Keep, Base, P, R, Ahead);
                                 Cost := R * R;
                                 Lambda := Lambda / Damping_Ratio;
                                 Better := True;
                                 if Fell <= Real'Model_Epsilon * Cost then
                                    Ok := True;
                                    return;
                                 end if;
                              end;
                              exit;
                           end if;
                        end;
                     end if;
                     Lambda := Lambda * Damping_Ratio;
                     exit when Lambda > 1.0 / Real'Model_Epsilon;
                  end;
               end loop;
               if not Better then
                  --  No damping lowers the cost: P is where it is least.
                  Ok := True;
                  return;
               end if;
            end;
         end;
      end loop;
   end Converge;

   procedure Fit_Eye (Seen : Sighting_Array; Width, Height : Positive; Result : out Report) is
      pragma Unreferenced (Width, Height);
      Keep   : Flags (Seen'Range) := [others => True];
      Base   : Mat3;
      P      : Parameters;
      Normal : Real_Matrix (1 .. Terms, 1 .. Terms);
      Ok     : Boolean;
   begin
      Result := (Offered => Seen'Length, others => <>);
      loop
         Start (Seen, Keep, Base, P, Ok);
         if not Ok then
            Result.Why := To_Unbounded_String
              ("no direct start from" & Count (Keep)'Image & " sightings (six at least, the camera ahead of them)");
            return;
         end if;
         Converge (Seen, Keep, Base, P, Normal, Ok);
         if not Ok then
            Result.Why := To_Unbounded_String ("the fit puts some sighting's beacon behind the camera");
            return;
         end if;
         declare
            M        : constant Natural := 2 * Count (Keep);
            R        : Real_Vector (1 .. M);
            Ahead    : Boolean;
            Scatter  : Real;
            Freedom  : constant Integer := Count (Keep) - Terms;
            Dropped  : Natural := 0;
            K        : Natural := 0;
         begin
            Residuals (Seen, Keep, Base, P, R, Ahead);
            Scatter := Real'Max (1.0, Driver.Stats.Robust_Sigma (Driver.Real_Array (R)));
            declare
               Gate_Of : constant Driver.Uncertain.Gate :=
                 Driver.Uncertain.Vector_Gate (2, Natural'Max (0, Freedom), Tests => Count (Keep));
            begin
               for I in Seen'Range loop
                  if Keep (I) then
                     if Driver.Uncertain.Significant
                          (Gate_Of, Sqrt (R (R'First + K) ** 2 + R (R'First + K + 1) ** 2), Scatter)
                     then
                        Keep (I) := False;
                        Dropped := Dropped + 1;
                     end if;
                     K := K + 2;
                  end if;
               end loop;
            end;
            if Dropped = 0 then
               Result.Lens := Lens_Of (P);
               Result.Pose := Pose_Of (Base, P);
               Result.Beacon := Beacon_Of (P);
               Result.Scatter := Scatter;
               Result.Used := Count (Keep);
               if Freedom <= 0 then
                  Result.Why := To_Unbounded_String
                    ("too few sightings to measure its" & Terms'Image & " terms from:" & Result.Used'Image & " kept");
                  return;
               end if;
               declare
                  Lower    : Real_Matrix (1 .. Terms, 1 .. Terms);
                  Definite : Boolean;
               begin
                  Driver.Numerics.Dense.Cholesky (Normal, Lower, Definite);
                  if not Definite then
                     Result.Why := To_Unbounded_String ("the sightings leave some term undetermined");
                     return;
                  end if;
                  for T in 1 .. Terms loop
                     declare
                        E : Real_Vector (1 .. Terms) := [others => 0.0];
                        Column : Real_Vector (1 .. Terms);
                     begin
                        E (T) := 1.0;
                        Column := Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
                        for Q in 1 .. Terms loop
                           Result.Covariance (Q, T) := Scatter ** 2 * Column (Q);
                        end loop;
                     end;
                  end loop;
               end;
               Result.Determined := True;
               return;
            end if;
         end;
      end loop;
   end Fit_Eye;

end Driver.Robot.Kinematics.Beacon;
