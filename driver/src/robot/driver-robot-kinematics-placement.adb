with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Placement is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;

   type Parameters is array (1 .. Terms) of Real;
   type Flags is array (Positive range <>) of Boolean;

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

   --  The arm's frame in the world: the turn about the world's axes on top of a base the fit returns to after every
   --  accepted step, so that the turn stays small and its covariance is about the world's axes.
   function Placed (Base : Mat3; P : Parameters) return Rigid is
     ((Rotation => Driver.Numerics.Exp (Vec3'[P (1), P (2), P (3)]) * Base, Translation => Vec3'[P (4), P (5), P (6)]));

   --  The sightings kept and the table, each residual in units of its sigma: the pixels, u then v, of every sighting,
   --  then the table's two tilts and its distance. Ahead False when some sighting's beacon is behind the camera.
   procedure Residuals
     (Seen : Beacon.Sighting_Array; Keep : Flags; Lens : Fit.Lens; Eye : Rigid; World, Arm : Plane; Base : Mat3;
      P : Parameters; R : out Real_Vector; Ahead : out Boolean)
   is
      Where : constant Rigid := Placed (Base, P);
      S     : constant Real := Exp (P (7));
      B     : constant Vec3 := [P (8), P (9), P (10)];
      K     : Natural := 0;
   begin
      Ahead := True;
      for I in Seen'Range loop
         if Keep (I) then
            declare
               In_Arm : constant Vec3 := Seen (I).Carrier * B;
               X      : constant Vec3 := Where.Rotation * (S * In_Arm) + Where.Translation;
               In_Eye : constant Vec3 := Transpose (Eye.Rotation) * (X - Eye.Translation);
               U, V   : Real;
               Front  : Boolean;
            begin
               Fit.Project (Lens, In_Eye, U, V, Front);
               Ahead := Ahead and then Front;
               R (R'First + K) := (Seen (I).U - U) / Seen (I).Sigma;
               R (R'First + K + 1) := (Seen (I).V - V) / Seen (I).Sigma;
               K := K + 2;
            end;
         end if;
      end loop;
      declare
         Turned : constant Vec3 := Where.Rotation * Arm.Normal;
         Off    : constant Vec3 := Cross (Turned, World.Normal);   --  the small turn that takes Turned to the world's
         Axis   : constant Vec3 :=
           (if abs (World.Normal (1)) < 0.5 then [1.0, 0.0, 0.0] elsif abs (World.Normal (2)) < 0.5 then [0.0, 1.0, 0.0]
            else [0.0, 0.0, 1.0]);
         E1     : constant Vec3 := Cross (World.Normal, Axis);
         T1     : constant Vec3 := (1.0 / abs E1) * E1;
         T2     : constant Vec3 := Cross (World.Normal, T1);
         Sigma_Normal : constant Real := Sqrt (World.Sigma_Normal ** 2 + Arm.Sigma_Normal ** 2);
         Sigma_Distance : constant Real := Sqrt (World.Sigma_Distance ** 2 + (S * Arm.Sigma_Distance) ** 2);
      begin
         R (R'First + K) := (if Sigma_Normal < Real'Last then (Off * T1) / Sigma_Normal else 0.0);
         R (R'First + K + 1) := (if Sigma_Normal < Real'Last then (Off * T2) / Sigma_Normal else 0.0);
         R (R'First + K + 2) :=
           (if Sigma_Distance < Real'Last
            then (World.Distance - (S * Arm.Distance + World.Normal * Where.Translation)) / Sigma_Distance else 0.0);
      end;
   end Residuals;

   --  The turn that takes the unit vector From to the unit vector To by the shortest way.
   function Align (From, To : Vec3) return Mat3 is
      Axis  : constant Vec3 := Cross (From, To);
      Sine  : constant Real := abs Axis;
      Cosine : constant Real := From * To;
   begin
      if Sine <= Real'Model_Epsilon then
         if Cosine > 0.0 then
            return Identity3;
         end if;
         --  Opposite: a half turn about any axis across them.
         declare
            Across : constant Vec3 :=
              (if abs From (1) < 0.5 then Cross (From, [1.0, 0.0, 0.0]) else Cross (From, [0.0, 1.0, 0.0]));
         begin
            return Driver.Numerics.Exp (Ada.Numerics.Pi / abs Across * Across);
         end;
      end if;
      return Driver.Numerics.Exp (Arctan (Sine, Cosine) / Sine * Axis);
   end Align;

   --  A start, linear in what it asks: the table turns the arm's frame but for its turn about the table's normal (the
   --  turn that takes the arm's table to the world's, Align), and what is left of that turn, its angle Psi, enters with
   --  the arm's unit S as A = S cos Psi and B = S sin Psi: a point of the arm, in the aligned frame, is A times its part
   --  across the normal plus B times that part turned a quarter, plus S times its part along the normal, plus the arm's
   --  place. The point lies on the ray of its sighting (the ray's cross product with the point less the camera is nought,
   --  linear in the six unknowns), and the arm's table lies on the world's (linear in S and the place), with the beacon
   --  taken at the carriers. A and B tell the turn and the unit.
   procedure Start
     (Seen : Beacon.Sighting_Array; Keep : Flags; Lens : Fit.Lens; Eye : Rigid; World, Arm : Plane;
      Base : out Mat3; P : out Parameters; Ok : out Boolean)
   is
      Unknowns : constant := 6;
      Aligned  : constant Mat3 := Align (Arm.Normal, World.Normal);
      AtA      : Real_Matrix (1 .. Unknowns, 1 .. Unknowns) := [others => [others => 0.0]];
      Atb      : Real_Vector (1 .. Unknowns) := [others => 0.0];
      Row      : Real_Vector (1 .. Unknowns);
      Lower    : Real_Matrix (1 .. Unknowns, 1 .. Unknowns);
      Definite : Boolean;

      --  One equation of the system: the coefficients of A, B, S and the three places, and its right side.
      procedure Add (Coefficients : Real_Vector; Right : Real) is
      begin
         for J in 1 .. Unknowns loop
            Atb (J) := Atb (J) + Coefficients (J) * Right;
            for L in 1 .. Unknowns loop
               AtA (J, L) := AtA (J, L) + Coefficients (J) * Coefficients (L);
            end loop;
         end loop;
      end Add;
   begin
      Base := Identity3;
      P := [others => 0.0];
      Ok := False;
      if Count (Keep) < Least_Sightings then
         return;
      end if;
      for I in Seen'Range loop
         if Keep (I) then
            declare
               Ray    : constant Vec3 := Eye.Rotation * Fit.Ray (Lens, Seen (I).U, Seen (I).V);
               R      : constant Vec3 := (1.0 / abs Ray) * Ray;
               Turned : constant Vec3 := Aligned * Seen (I).Carrier.Translation;
               Height : constant Real := World.Normal * Turned;
               Along  : constant Vec3 := Height * World.Normal;
               Across : constant Vec3 := Turned - Along;
               Quarter : constant Vec3 := Cross (World.Normal, Turned);
               --  The cross product with the ray as a matrix: Skew * V is R x V.
               Skew   : constant Mat3 := [[0.0, -R (3), R (2)], [R (3), 0.0, -R (1)], [-R (2), R (1), 0.0]];
               Cross_Across  : constant Vec3 := Skew * Across;
               Cross_Quarter : constant Vec3 := Skew * Quarter;
               Cross_Along   : constant Vec3 := Skew * Along;
               Right  : constant Vec3 := Skew * Eye.Translation;
            begin
               for Axis in 1 .. 3 loop
                  Row := [Cross_Across (Axis), Cross_Quarter (Axis), Cross_Along (Axis),
                          Skew (Axis, 1), Skew (Axis, 2), Skew (Axis, 3)];
                  Add (Row, Right (Axis));
               end loop;
            end;
         end if;
      end loop;
      Row := [0.0, 0.0, Arm.Distance, World.Normal (1), World.Normal (2), World.Normal (3)];
      Add (Row, World.Distance);
      Driver.Numerics.Dense.Cholesky (AtA, Lower, Definite);
      if not Definite then
         return;
      end if;
      declare
         Solution : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, Atb);
         Unit     : constant Real := Sqrt (Solution (1) ** 2 + Solution (2) ** 2);
         Where    : Parameters := [others => 0.0];
         Turn     : Mat3;
         Rt       : Real_Vector (1 .. 2 * Count (Keep) + Tables);
         Front    : Boolean;
      begin
         if Unit <= 0.0 then
            return;
         end if;
         Turn := Driver.Numerics.Exp (Arctan (Solution (2), Solution (1)) * World.Normal) * Aligned;
         Where (4 .. 6) := [Solution (4), Solution (5), Solution (6)];
         Where (7) := Ada.Numerics.Long_Elementary_Functions.Log (Unit);
         Residuals (Seen, Keep, Lens, Eye, World, Arm, Turn, Where, Rt, Front);
         if Front then
            Base := Turn;
            P := Where;
            Ok := True;
         end if;
      end;
   exception
      when Constraint_Error | Ada.Numerics.Argument_Error =>
         Ok := False;
   end Start;

   --  Damped Gauss-Newton from P (Levenberg-Marquardt), central differences, as the eye's fit (Beacon.Converge).
   procedure Converge
     (Seen : Beacon.Sighting_Array; Keep : Flags; Lens : Fit.Lens; Eye : Rigid; World, Arm : Plane; Base : in out Mat3;
      P : in out Parameters; Normal : out Real_Matrix; Ok : out Boolean)
   is
      M      : constant Natural := 2 * Count (Keep) + Tables;
      Eps3   : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);
      Damping_Ratio : constant Real := 10.0;
      Lambda : Real := Real'Model_Epsilon ** 0.5;
      R      : Real_Vector (1 .. M);
      Ahead  : Boolean;
      Cost   : Real;
      Scale  : Real := 0.0;   --  the carriers' spread, for the steps of lengths
   begin
      Ok := False;
      Normal := [others => [others => 0.0]];
      for I in Seen'Range loop
         if Keep (I) then
            Scale := Real'Max (Scale, abs Seen (I).Carrier.Translation);
         end if;
      end loop;
      Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, P, R, Ahead);
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
                  H        : constant Real :=
                    (case T is when 1 .. 3 | 7 => Eps3, when others => Eps3 * Real'Max (Scale, abs P (T)));
                  Up, Down : Parameters := P;
                  Ru, Rd   : Real_Vector (1 .. M);
                  Au, Ad   : Boolean;
               begin
                  Up (T) := P (T) + H;
                  Down (T) := P (T) - H;
                  Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, Up, Ru, Au);
                  Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, Down, Rd, Ad);
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
                           Step     : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, -Jtr);
                           Trial    : Parameters := P;
                           Rt       : Real_Vector (1 .. M);
                           At_Front : Boolean;
                        begin
                           for T in 1 .. Terms loop
                              Trial (T) := P (T) + Step (T);
                           end loop;
                           Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, Trial, Rt, At_Front);
                           if At_Front and then Rt * Rt < Cost then
                              declare
                                 Fell : constant Real := Cost - Rt * Rt;
                              begin
                                 --  The turn folds into the base and starts again from nought.
                                 Base := Driver.Numerics.Exp (Vec3'[Trial (1), Trial (2), Trial (3)]) * Base;
                                 Trial (1 .. 3) := [0.0, 0.0, 0.0];
                                 P := Trial;
                                 Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, P, R, Ahead);
                                 Cost := R * R;
                                 Lambda := Lambda / Damping_Ratio;
                                 Better := True;
                                 --  The fit has converged when a step lowers the sum of squares by less than the
                                 --  sightings' own variance (their residuals' robust spread, never below the one they
                                 --  state): it moves nothing they can tell. Where the focal length and the distance
                                 --  trade, the cost goes on falling toward a camera at infinity by less at each step, a
                                 --  flat valley 16 s long for 12 sightings (k36, 10-20: the estimates held the boot).
                                 if Fell <= Real'Max (1.0, Driver.Stats.Robust_Sigma (Driver.Real_Array (R))) ** 2 then
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
                  Ok := True;
                  return;
               end if;
            end;
         end;
      end loop;
   end Converge;

   procedure Fit_Arm
     (Seen   : Beacon.Sighting_Array;
      Lens   : Fit.Lens;
      Eye    : Rigid;
      World  : Plane;
      Arm    : Plane;
      Result : out Report)
   is
      Keep   : Flags (Seen'Range) := [others => True];
      Base   : Mat3;
      P      : Parameters;
      Normal : Real_Matrix (1 .. Terms, 1 .. Terms);
      Ok     : Boolean;
   begin
      Result := (Offered => Seen'Length, others => <>);
      loop
         Start (Seen, Keep, Lens, Eye, World, Arm, Base, P, Ok);
         if not Ok then
            Result.Why := To_Unbounded_String
              ("no start from" & Count (Keep)'Image & " sightings (five at least) and the two tables");
            return;
         end if;
         Converge (Seen, Keep, Lens, Eye, World, Arm, Base, P, Normal, Ok);
         if not Ok then
            Result.Why := To_Unbounded_String ("the fit puts some sighting's beacon behind the camera");
            return;
         end if;
         --  The sightings that do not fit are left out until none is: what a flick's centre is off by when part of the
         --  closer hides, or the arm's fit is off at an extreme pose.
         declare
            M       : constant Natural := 2 * Count (Keep) + Tables;
            R       : Real_Vector (1 .. M);
            Ahead   : Boolean;
            Spread  : Real;
            Dropped : Natural := 0;
            K       : Natural := 0;
         begin
            Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, P, R, Ahead);
            Spread := Real'Max (1.0, Driver.Stats.Robust_Sigma (Driver.Real_Array (R (1 .. M - Tables))));
            declare
               Gate_Of : constant Driver.Uncertain.Gate :=
                 Driver.Uncertain.Vector_Gate (2, Natural'Max (0, M - Terms), Tests => Count (Keep));
            begin
               for I in Seen'Range loop
                  if Keep (I) then
                     if Driver.Uncertain.Significant
                          (Gate_Of, Sqrt (R (R'First + K) ** 2 + R (R'First + K + 1) ** 2), Spread)
                     then
                        Keep (I) := False;
                        Dropped := Dropped + 1;
                     end if;
                     K := K + 2;
                  end if;
               end loop;
            end;
            exit when Dropped = 0 or else Count (Keep) <= Least_Sightings;
         end;
      end loop;
      declare
         M       : constant Natural := 2 * Count (Keep) + Tables;
         R       : Real_Vector (1 .. M);
         Ahead   : Boolean;
         Used    : constant Natural := Count (Keep);
         Freedom : constant Integer := M - Terms;
         Scatter : Real;
      begin
         Residuals (Seen, Keep, Lens, Eye, World, Arm, Base, P, R, Ahead);
         Scatter := Real'Max (1.0, Driver.Stats.Robust_Sigma (Driver.Real_Array (R (1 .. M - Tables))));
         Result.Placement := Placed (Base, P);
         Result.Scale := Exp (P (7));
         Result.Beacon := [P (8), P (9), P (10)];
         Result.Scatter := Scatter;
         Result.Used := Used;
         if Freedom <= 0 then
            Result.Why := To_Unbounded_String ("too few sightings to measure its" & Terms'Image & " terms from");
            return;
         end if;
         declare
            Lower    : Real_Matrix (1 .. Terms, 1 .. Terms);
            Definite : Boolean;
         begin
            Driver.Numerics.Dense.Cholesky (Normal, Lower, Definite);
            if not Definite then
               Result.Why := To_Unbounded_String ("the sightings and the tables leave some term undetermined");
               return;
            end if;
            for T in 1 .. Terms loop
               declare
                  E      : Real_Vector (1 .. Terms) := [others => 0.0];
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
         declare
            Turn_Var : constant Real := Result.Covariance (1, 1) + Result.Covariance (2, 2) + Result.Covariance (3, 3);
            T        : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Natural (Freedom)));
         begin
            if not (T * Sqrt (Turn_Var) < Ada.Numerics.Pi / 2.0
                    and then Driver.Uncertain.Significant
                               (Result.Scale, Result.Scale * Sqrt (Result.Covariance (7, 7)), Natural (Freedom)))
            then
               Result.Why := To_Unbounded_String
                 ("the flicks and the tables leave the arm untold: turn +-" & Driver.Log.Image (Sqrt (Turn_Var), 3)
                  & " rad, unit" & Driver.Log.Image (Result.Scale, 3) & " +-"
                  & Driver.Log.Image (Result.Scale * Sqrt (Result.Covariance (7, 7)), 3));
               return;
            end if;
         end;
         Result.Determined := True;
      end;
   exception
      when Constraint_Error | Ada.Numerics.Argument_Error =>
         Result := (Offered => Seen'Length, others => <>);
         Result.Why := To_Unbounded_String ("the sightings could not be fitted: a number the fit cannot take");
   end Fit_Arm;

end Driver.Robot.Kinematics.Placement;
