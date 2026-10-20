with Ada.Environment_Variables;
with Ada.Long_Float_Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Hand_Eye;
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
   --  The lens as a model of two kinds, the simpler tried first: a pinhole (one focal length, square pixels, the
   --  principal point at the picture's middle) and the full lens (both focal lengths and the principal point free),
   --  the full one kept only when what it adds is significant (Fit_Eye), as the arm's lens is kept a pinhole unless its
   --  extras are. Flicks over a part of the picture (the arm's reach in an eye that sees only a corner of it) tell the
   --  four terms of the full lens apart no better than a pinhole is told, and the nine of them with the pose.
   type Model_Kind is (Pinhole, Full);

   function Lens_Of (P : Parameters; Kind : Model_Kind; Width, Height : Positive) return Fit.Lens is
     (if Kind = Full
      then (Fx => Exp (P (1)), Fy => Exp (P (2)), Cx => P (3), Cy => P (4), K1 => 0.0, K2 => 0.0)
      else (Fx => Exp (P (1)), Fy => Exp (P (1)), Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, K1 => 0.0,
            K2 => 0.0));

   --  Whether a term is free in the model: the pinhole ties the second focal length to the first and fixes the centre.
   function Active (T : Positive; Kind : Model_Kind) return Boolean is (Kind = Full or else T not in 2 .. 4);

   function Pose_Of (Base : Mat3; P : Parameters) return Rigid is
     ((Rotation => Base * Driver.Numerics.Exp (Vec3'[P (5), P (6), P (7)]), Translation => Vec3'[P (8), P (9), P (10)]));

   function Beacon_Of (P : Parameters) return Vec3 is (Vec3'[P (11), P (12), P (13)]);

   --  The sightings kept, each's residuals in units of its sigma, u then v; Ahead False when the camera puts some
   --  sighting's beacon behind it.
   procedure Residuals
     (Seen : Sighting_Array; Keep : Flags; Base : Mat3; P : Parameters; R : out Real_Vector; Ahead : out Boolean;
      Kind : Model_Kind; Width, Height : Positive)
   is
      L    : constant Fit.Lens := Lens_Of (P, Kind, Width, Height);
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
            P (1) := Ada.Numerics.Long_Elementary_Functions.Log (K11 / K33);
            P (2) := Ada.Numerics.Long_Elementary_Functions.Log (K22 / K33);
            P (3) := K13 / K33;
            P (4) := K23 / K33;
            P (8 .. 10) := [Centre (1), Centre (2), Centre (3)];
            Ok := True;
         end;
      end;
   exception
      --  Sightings that do not fix a projection (the carriers on one plane: the first poses of joints about parallel
      --  axes) leave the normal equations singular, and the least eigenvector or the inverse of the left block has
      --  none: there is no start yet, and the next sighting may give one (seed 3 of the bench, 10-20).
      when Constraint_Error | Ada.Numerics.Argument_Error =>
         Base := Identity3;
         P := [others => 0.0];
         Ok := False;
   end Start;

   --  Damped Gauss-Newton from P (Levenberg-Marquardt), central differences for the Jacobian: each step's damping
   --  grows by Damping_Ratio while a step raises the cost and falls by it after one that lowers it; the fit ends when a
   --  step lowers the cost by less than its rounding, or no damping finds one that lowers it. Normal holds the
   --  undamped normal equations at the end.
   procedure Converge
     (Seen : Sighting_Array; Keep : Flags; Base : in out Mat3; P : in out Parameters; Normal : out Real_Matrix;
      Ok : out Boolean; Kind : Model_Kind; Width, Height : Positive)
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
      Residuals (Seen, Keep, Base, P, R, Ahead, Kind, Width, Height);
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
               if not Active (T, Kind) then
                  for I in 1 .. M loop
                     J (I, T) := 0.0;
                  end loop;
               else
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
                  Residuals (Seen, Keep, Base, Up, Ru, Au, Kind, Width, Height);
                  Residuals (Seen, Keep, Base, Down, Rd, Ad, Kind, Width, Height);
                  for I in 1 .. M loop
                     J (I, T) := (Ru (I) - Rd (I)) / (2.0 * H);
                  end loop;
               end;
               end if;
            end loop;
            JtJ := Transpose (J) * J;
            Jtr := Transpose (J) * R;
            for T in 1 .. Terms loop
               if not Active (T, Kind) then
                  JtJ (T, T) := 1.0;   --  a term the model holds is held: no step, nothing coupled to it
               end if;
            end loop;
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
                           Residuals (Seen, Keep, Base, Trial, Rt, At_Front, Kind, Width, Height);
                           if At_Front and then Rt * Rt < Cost then
                              declare
                                 Fell : constant Real := Cost - Rt * Rt;
                              begin
                                 --  The turn folds into the base and starts again from nought.
                                 Base := Base * Driver.Numerics.Exp (Vec3'[Trial (5), Trial (6), Trial (7)]);
                                 Trial (5 .. 7) := [0.0, 0.0, 0.0];
                                 P := Trial;
                                 Residuals (Seen, Keep, Base, P, R, Ahead, Kind, Width, Height);
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
                  --  No damping lowers the cost: P is where it is least.
                  Ok := True;
                  return;
               end if;
            end;
         end;
      end loop;
   end Converge;

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

   procedure Dump (Seen : Sighting_Array; Width, Height : Positive; Name : String) is
      use Ada.Text_IO;
      File   : File_Type;
      Number : Positive;
      procedure Put_Real (X : Real) is
      begin
         Put (File, " ");
         Ada.Long_Float_Text_IO.Put (File, X, Fore => 1, Aft => Real'Digits + 1, Exp => 1);
      end Put_Real;
   begin
      if not Ada.Environment_Variables.Exists ("BD_DUMP_FITS") then
         return;
      end if;
      Dumps.Next (Number);
      Create (File, Out_File, Ada.Environment_Variables.Value ("BD_DUMP_FITS") & "/beacon_" & Name & "_"
              & Ada.Strings.Fixed.Trim (Number'Image, Ada.Strings.Left) & ".txt");
      Put_Line (File, "sizes" & Seen'Length'Image & Width'Image & Height'Image);
      for X of Seen loop
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Put_Real (X.Carrier.Rotation (I, J));
            end loop;
         end loop;
         for I in 1 .. 3 loop
            Put_Real (X.Carrier.Translation (I));
         end loop;
         Put_Real (X.U);
         Put_Real (X.V);
         Put_Real (X.Sigma);
         New_Line (File);
      end loop;
      Close (File);
   end Dump;

   procedure Fit_Eye (Seen : Sighting_Array; Width, Height : Positive; Result : out Report) is
      Keep    : Flags (Seen'Range) := [others => True];
      Base_F  : Mat3;
      P_F     : Parameters;
      Normal_F : Real_Matrix (1 .. Terms, 1 .. Terms);
      Ok      : Boolean;
      Scatter_F : Real := 1.0;
      Cost_F  : Real := 0.0;
      --  The full lens's covariance, when it is not told.
      Cov_Full : Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];

      --  The residuals of the sightings kept under a model, their robust spread (never below one: the sigmas the
      --  flicks state are lower bounds) and their cost.
      procedure Judge (Base : Mat3; P : Parameters; Kind : Model_Kind; Scatter, Cost : out Real; R_Out : out Real_Vector;
                       Ahead : out Boolean) is
      begin
         Residuals (Seen, Keep, Base, P, R_Out, Ahead, Kind, Width, Height);
         Scatter := Real'Max (1.0, Driver.Stats.Robust_Sigma (Driver.Real_Array (R_Out)));
         Cost := R_Out * R_Out;
      end Judge;

      --  The report of a fit that converged: the lens, the pose and the beacon, the covariance of its free terms scaled
      --  by the scatter (the tied and the held terms take the covariance of what they follow, a held centre none),
      --  and whether the lens and the place are told by the standard of the points on the arms.
      procedure Report_Of (Base : Mat3; P : Parameters; Normal : Real_Matrix; Kind : Model_Kind; Scatter : Real) is
         Used    : constant Natural := Count (Keep);
         Free    : constant Natural := (if Kind = Full then Terms else Terms - 3);
         Freedom : constant Integer := Used - Free;
      begin
         Result.Lens := Lens_Of (P, Kind, Width, Height);
         Result.Pose := Pose_Of (Base, P);
         Result.Beacon := Beacon_Of (P);
         Result.Scatter := Scatter;
         Result.Used := Used;
         Result.Pinhole := Kind = Pinhole;
         if Freedom <= 0 then
            Result.Why := To_Unbounded_String
              ("too few sightings to measure its" & Free'Image & " terms from:" & Result.Used'Image & " kept");
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
               if Active (T, Kind) then
                  declare
                     E : Real_Vector (1 .. Terms) := [others => 0.0];
                     Column : Real_Vector (1 .. Terms);
                  begin
                     E (T) := 1.0;
                     Column := Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
                     for Q in 1 .. Terms loop
                        if Active (Q, Kind) then
                           Result.Covariance (Q, T) := Scatter ** 2 * Column (Q);
                        end if;
                     end loop;
                  end;
               end if;
            end loop;
            if Kind = Pinhole then
               --  The second focal length is the first: it varies with it, and with whatever the first varies with.
               for Q in 1 .. Terms loop
                  if Active (Q, Kind) then
                     Result.Covariance (2, Q) := Result.Covariance (1, Q);
                     Result.Covariance (Q, 2) := Result.Covariance (Q, 1);
                  end if;
               end loop;
               Result.Covariance (2, 2) := Result.Covariance (1, 1);
            end if;
         end;
         --  The standard every measurement of a fixed eye meets (Hand_Eye.Judge_Lens, Judge_Pose): 14 flicks of 13
         --  terms and a positive definite matrix placed an eye at 58 by 20 px for a truth of 153 by 153, its principal
         --  point outside the picture (e14, 10-19).
         declare
            Lens_Is : constant Hand_Eye.Lens_Verdict :=
              Hand_Eye.Judge_Lens (Result.Lens, Result.Covariance, Natural (Freedom), Width, Height);
            Pose_Is : constant Hand_Eye.Pose_Verdict :=
              Hand_Eye.Judge_Pose (Result.Pose.Translation, Result.Covariance, Lens_Terms + 1, Natural (Freedom));
         begin
            if not (Lens_Is.Focal_Told and then Lens_Is.Centre_Told and then Pose_Is.Told) then
               Result.Why := To_Unbounded_String
                 ("the flicks leave the eye untold: focal lengths" & Driver.Log.Image (Result.Lens.Fx, 1) & " +-"
                  & Driver.Log.Image (Lens_Is.Sigma_Fx, 1) & " and" & Driver.Log.Image (Result.Lens.Fy, 1) & " +-"
                  & Driver.Log.Image (Lens_Is.Sigma_Fy, 1) & " px, principal point +-"
                  & Driver.Log.Image (Lens_Is.Sigma_Cx, 1) & " and" & Driver.Log.Image (Lens_Is.Sigma_Cy, 1)
                  & " px, turn +-" & Driver.Log.Image (Pose_Is.Turn_Sigma, 3) & " rad, distance"
                  & Driver.Log.Image (Pose_Is.Away, 3) & " +-" & Driver.Log.Image (Pose_Is.Away_Sigma, 3)
                  & (if Kind = Pinhole then " (a pinhole lens)" else " (the full lens)"));
               return;
            end if;
         end;
         Result.Determined := True;
      end Report_Of;
   begin
      Result := (Offered => Seen'Length, others => <>);
      --  The full lens first, with the sightings that do not fit it left out until none is: the extras are judged
      --  on the sightings that stay.
      loop
         Start (Seen, Keep, Base_F, P_F, Ok);
         if not Ok then
            Result.Why := To_Unbounded_String
              ("no direct start from" & Count (Keep)'Image & " sightings (six at least, the camera ahead of them)");
            return;
         end if;
         Converge (Seen, Keep, Base_F, P_F, Normal_F, Ok, Full, Width, Height);
         if not Ok then
            Result.Why := To_Unbounded_String ("the fit puts some sighting's beacon behind the camera");
            return;
         end if;
         declare
            M        : constant Natural := 2 * Count (Keep);
            R        : Real_Vector (1 .. M);
            Ahead    : Boolean;
            Freedom  : constant Integer := Count (Keep) - Terms;
            Dropped  : Natural := 0;
            K        : Natural := 0;
         begin
            Judge (Base_F, P_F, Full, Scatter_F, Cost_F, R, Ahead);
            declare
               Gate_Of : constant Driver.Uncertain.Gate :=
                 Driver.Uncertain.Vector_Gate (2, Natural'Max (0, Freedom), Tests => Count (Keep));
            begin
               for I in Seen'Range loop
                  if Keep (I) then
                     if Driver.Uncertain.Significant
                          (Gate_Of, Sqrt (R (R'First + K) ** 2 + R (R'First + K + 1) ** 2), Scatter_F)
                     then
                        Keep (I) := False;
                        Dropped := Dropped + 1;
                     end if;
                     K := K + 2;
                  end if;
               end loop;
            end;
            exit when Dropped = 0;
         end;
      end loop;
      --  The full lens, when the sightings tell it: its sigmas are those of the lens that was fitted, and the pose is
      --  not made to carry what a lens off a pinhole does (robot.beacon.place: a principal point 10 px off the middle
      --  and focal lengths 3 % apart, fitted as a pinhole from 40 sightings, turned the camera 2 degrees off its sigma
      --  of a third of one).
      Report_Of (Base_F, P_F, Normal_F, Full, Scatter_F);
      if Result.Determined then
         return;
      end if;
      Cov_Full := Result.Covariance;
      --  Not told: the pinhole on the same sightings, and whether what the full lens adds lowers their cost by more
      --  than the noise allows (three terms, an F test at the freedom the sightings have, at the false alarm rate of
      --  Z): else the lens is taken as the pinhole, whose nine other terms the flicks tell better than the full lens's
      --  twelve, and later flicks that tell the full lens replace it.
      declare
         Base_P   : Mat3 := Base_F;
         P_P      : Parameters := P_F;
         Normal_P : Real_Matrix (1 .. Terms, 1 .. Terms);
         Ok_P     : Boolean;
         M        : constant Natural := 2 * Count (Keep);
         R        : Real_Vector (1 .. M);
         Scatter_P, Cost_P : Real;
         Ahead    : Boolean;
         Freedom_2 : constant Integer := M - Terms;
         Extras   : Boolean := True;
      begin
         P_P (1) := (P_F (1) + P_F (2)) / 2.0;
         P_P (2) := P_P (1);
         Converge (Seen, Keep, Base_P, P_P, Normal_P, Ok_P, Pinhole, Width, Height);
         if Ok_P then
            Judge (Base_P, P_P, Pinhole, Scatter_P, Cost_P, R, Ahead);
            if Freedom_2 >= 1 and then Cost_F > 0.0 then
               Extras := Driver.Distributions.F_Upper_Tail
                           (Real'Max (0.0, (Cost_P - Cost_F) / 3.0) / (Cost_F / Real (Freedom_2)), 3, Freedom_2)
                         < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
            else
               Extras := False;
            end if;
            if not Extras then
               Result := (Offered => Seen'Length, others => <>);
               Report_Of (Base_P, P_P, Normal_P, Pinhole, Scatter_P);
               --  What is claimed of a lens taken as a pinhole is what the full lens leaves uncertain, not what the
               --  pinhole's nine terms do when the lens is one: a camera whose principal point is off the middle or
               --  whose pixels are not square turns the pinhole's camera to make up for it, and the sigmas of the
               --  pinhole alone do not say so (robot.beacon.few: 16 sightings, the turn 4 sigmas off).
               if Result.Determined and then Cov_Full (1, 1) > 0.0 then
                  Result.Covariance := Cov_Full;
               end if;
            end if;
         end if;
      end;
   exception
      --  A number the fit cannot take (a covariance of sightings too few or too flat to invert) is no placement.
      when Constraint_Error | Ada.Numerics.Argument_Error =>
         Result := (others => <>);
         Result.Why := Ada.Strings.Unbounded.To_Unbounded_String
           ("the sightings could not be fitted: a number the fit cannot take");
   end Fit_Eye;

end Driver.Robot.Kinematics.Beacon;
