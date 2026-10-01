with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Geometry.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  Box-Muller from two uniforms in (0, 1].
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Ray_To (Origin, Target : Vec3; Origin_Sigma, Angle_Sigma : Real) return Ray_Estimate is
     ((Origin    => (Mean => Origin, Covariance => (Origin_Sigma ** 2) * Identity3),
       Direction => (Unit_Vector => Unit (Target - Origin), Sigma => Angle_Sigma)));

   function Turned (U : Vec3; Sigma : Real) return Vec3 is
      --  U turned by a random small rotation of the given angular sigma per axis.
      E1, E2 : Vec3;
      Smallest : Positive := U'First;
   begin
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
      return Unit (U + Sigma * Gaussian * E1 + Sigma * Gaussian * E2);
   end Turned;

   procedure Meet_Exact is
      Target : constant Vec3 := [1.0, 2.0, 3.0];
      P  : Point_Estimate;
      Ok : Boolean;
   begin
      Meet ([Ray_To ([0.0, 0.0, 0.0], Target, 1.0e-6, 1.0e-6), Ray_To ([4.0, 0.0, 0.0], Target, 1.0e-6, 1.0e-6),
             Ray_To ([0.0, 5.0, -1.0], Target, 1.0e-6, 1.0e-6)], P, Ok);
      Check (Ok, "three rays through one point did not meet");
      Check (abs (P.Mean - Target) < 1.0e-9, "three rays through one point met elsewhere");
   end Meet_Exact;

   procedure Meet_Weights is
      --  Two rays crossing at right angles near (0, 0, 10); the second passes
      --  0.1 off the first, and only the first is precise.
      Precise : constant Ray_Estimate := Ray_To ([0.0, 0.0, 0.0], [0.0, 0.0, 10.0], 1.0e-6, 1.0e-4);
      Rough   : constant Ray_Estimate := Ray_To ([-10.0, 0.1, 10.0], [0.0, 0.1, 10.0], 1.0e-6, 1.0e-2);
      P  : Point_Estimate;
      Ok : Boolean;
   begin
      Meet ([Precise, Rough], P, Ok);
      Check (Ok, "two crossing rays did not meet");
      --  Equal weights would put it halfway (0.05 off each); weighting by
      --  the 100 times smaller sigma puts it within 1 / 100^2 of the gap.
      Check (abs P.Mean (2) < 1.0e-4, "the meeting point is not pulled to the precise ray: y ="
             & Real'Image (P.Mean (2)));
   end Meet_Weights;

   procedure Meet_Covariance is
      --  Two eyes 0.2 apart look at a point 1 away: depth is poorly fixed.
      Target : constant Vec3 := [0.0, 0.0, 1.0];
      O1     : constant Vec3 := [-0.1, 0.0, 0.0];
      O2     : constant Vec3 := [0.1, 0.0, 0.0];
      Sigma  : constant Real := 1.0e-3;
      Trials : constant := 400;
      Sum    : Mat3 := [others => [others => 0.0]];
      Reported : Point_Estimate;
      Ok     : Boolean;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 7);
      Meet ([Ray_To (O1, Target, 1.0e-9, Sigma), Ray_To (O2, Target, 1.0e-9, Sigma)], Reported, Ok);
      Check (Ok, "two converging rays did not meet");
      for T in 1 .. Trials loop
         declare
            R1 : constant Ray_Estimate :=
              (Origin => (Mean => O1, Covariance => (1.0e-18) * Identity3),
               Direction => (Unit_Vector => Turned (Unit (Target - O1), Sigma), Sigma => Sigma));
            R2 : constant Ray_Estimate :=
              (Origin => (Mean => O2, Covariance => (1.0e-18) * Identity3),
               Direction => (Unit_Vector => Turned (Unit (Target - O2), Sigma), Sigma => Sigma));
            P  : Point_Estimate;
            Good : Boolean;
         begin
            Meet ([R1, R2], P, Good);
            Check (Good, "a noisy pair of rays did not meet");
            Sum := Sum + Outer (P.Mean - Target, P.Mean - Target);
         end;
      end loop;
      --  The empirical variance along depth (z) and across (x) against the
      --  reported ones; a variance from 400 samples is good to about 7 %.
      declare
         Vz : constant Real := Sum (3, 3) / Real (Trials);
         Vx : constant Real := Sum (1, 1) / Real (Trials);
      begin
         Check_Close (Vz / Reported.Covariance (3, 3), 1.0, 0.25, "depth variance against the reported one");
         Check_Close (Vx / Reported.Covariance (1, 1), 1.0, 0.25, "lateral variance against the reported one");
         Check (Reported.Covariance (3, 3) > 10.0 * Reported.Covariance (1, 1),
                "a narrow baseline did not make depth the uncertain direction");
      end;
   end Meet_Covariance;

   procedure Meet_Behind is
      P  : Point_Estimate;
      Ok : Boolean;
   begin
      --  The two lines cross at the origin, behind where the first ray starts.
      Meet ([(Origin => (Mean => [0.0, 0.0, 1.0], Covariance => 1.0e-6 * Identity3),
              Direction => (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 1.0e-3)),
             Ray_To ([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0e-3, 1.0e-3)], P, Ok);
      Check (not Ok, "a meeting point behind a ray was accepted");
      Meet ([Ray_To ([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0e-3, 1.0e-3)], P, Ok);
      Check (not Ok, "a single ray was said to meet");
   end Meet_Behind;

   --  Points on the plane z = 0.3 x - 0.2 y + 1 with noise of sigma 0.01
   --  along z; Outliers of them are lifted by a random height above it.
   procedure Make_Points (Pts : out Point_Array; Outliers : Natural; Lift : Real) is
      Sigma : constant Real := 0.01;
   begin
      for I in Pts'Range loop
         declare
            X : constant Real := 2.0 * Real (Ada.Numerics.Float_Random.Random (Gen)) - 1.0;
            Y : constant Real := 2.0 * Real (Ada.Numerics.Float_Random.Random (Gen)) - 1.0;
            Z : Real := 0.3 * X - 0.2 * Y + 1.0 + Sigma * Gaussian;
         begin
            if I - Pts'First < Outliers then
               Z := Z + Lift * (1.0 + Real (Ada.Numerics.Float_Random.Random (Gen)));
            end if;
            Pts (I) := (Mean       => [X, Y, Z],
                        Covariance => [[1.0e-8, 0.0, 0.0], [0.0, 1.0e-8, 0.0], [0.0, 0.0, Sigma ** 2]]);
         end;
      end loop;
   end Make_Points;

   True_Normal : constant Vec3 := Unit ([-0.3, 0.2, 1.0]);

   procedure Plane_Fit is
      Trials : constant := 200;
      Sum_Offset, Sum_Tilt : Real := 0.0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      for T in 1 .. Trials loop
         declare
            Pts : Point_Array (1 .. 40);
            P   : Plane_Estimate;
            Ok  : Boolean;
         begin
            Make_Points (Pts, 0, 0.0);
            Fit (Pts, [Pts'Range => True], P, Ok);
            Check (Ok, "forty points on a plane gave no fit");
            Orient (P, [0.0, 0.0, 10.0]);
            --  The true plane's height at the fitted centre, against the offset sigma.
            Sum_Offset := Sum_Offset
              + ((True_Normal * P.Centre - True_Normal * [0.0, 0.0, 1.0]) / P.Offset_Sigma) ** 2;
            declare
               A : constant Real := True_Normal * P.Tangent_1;
               B : constant Real := True_Normal * P.Tangent_2;
               Det : constant Real := P.Tilt_11 * P.Tilt_22 - P.Tilt_12 ** 2;
            begin
               --  Squared Mahalanobis length of the normal's error, two degrees of freedom.
               Sum_Tilt := Sum_Tilt + (A * A * P.Tilt_22 - 2.0 * A * B * P.Tilt_12 + B * B * P.Tilt_11) / Det;
            end;
         end;
      end loop;
      Check_Close (Sum_Offset / Real (Trials), 1.0, 0.3, "offset errors against their reported sigma (mean square)");
      Check_Close (Sum_Tilt / Real (Trials), 2.0, 0.6, "tilt errors against their reported covariance (chi square, 2 dof)");
   end Plane_Fit;

   procedure Plane_Robust is
      Pts : Point_Array (1 .. 50);
      P   : Plane_Estimate;
      In_Plane : Flag_Array (Pts'Range);
      Ok  : Boolean;
      Kept_Outliers : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 13);
      Make_Points (Pts, 20, 0.2);
      Fit_Robust (Pts, [Pts'Range => True], P, In_Plane, Ok);
      Check (Ok, "a plane with 40 % of the points above it gave no fit");
      for I in 1 .. 20 loop
         if In_Plane (I) then
            Kept_Outliers := Kept_Outliers + 1;
         end if;
      end loop;
      Check (Kept_Outliers = 0, "points well above the plane were kept on it:" & Natural'Image (Kept_Outliers));
      Check (abs (P.Normal * True_Normal) > 1.0 - 1.0e-3, "the robust normal is off the true one");
      --  The plain fit over every point is pulled up by the lifted ones.
      Fit (Pts, [Pts'Range => True], P, Ok);
      Orient (P, [0.0, 0.0, 10.0]);
      Check (P.Normal * P.Centre - True_Normal * [0.0, 0.0, 1.0] * (P.Normal * True_Normal) > 0.05,
             "the plain fit was not disturbed: the robust test has no teeth");
   end Plane_Robust;

   procedure Ray_Plane is
      Plane : Plane_Estimate;
      Pts   : Point_Array (1 .. 30);
      Ok    : Boolean;
      Trials : constant := 300;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 17);
      Make_Points (Pts, 0, 0.0);
      Fit (Pts, [Pts'Range => True], Plane, Ok);
      Orient (Plane, [0.0, 0.0, 10.0]);
      declare
         R : constant Ray_Estimate := Ray_To ([0.5, 0.5, 3.0], [0.2, 0.1, 1.04], 1.0e-3, 2.0e-3);
         X : Point_Estimate;
         D : Estimate;
         Sum : Real := 0.0;
      begin
         Intersect (Plane, R, X, D, Ok);
         Check (Ok, "a ray towards the plane did not meet it");
         Check (abs Height (Plane, X.Mean) < 1.0e-9, "the meeting point is not on the plane");
         --  Monte Carlo over the ray's own noise (the plane held fixed) against
         --  the reported distance sigma with the plane's part taken out.
         for T in 1 .. Trials loop
            declare
               Rt : constant Ray_Estimate :=
                 (Origin => (Mean => R.Origin.Mean + 1.0e-3 * [Gaussian, Gaussian, Gaussian],
                             Covariance => R.Origin.Covariance),
                  Direction => (Unit_Vector => Turned (R.Direction.Unit_Vector, 2.0e-3), Sigma => 2.0e-3));
               Xt : Point_Estimate;
               Dt : Estimate;
               Good : Boolean;
            begin
               Intersect (Plane, Rt, Xt, Dt, Good);
               Sum := Sum + (Dt.Value - D.Value) ** 2;
            end;
         end loop;
         declare
            Facing : constant Real := Plane.Normal * R.Direction.Unit_Vector;
            Own    : constant Real := D.Sigma ** 2 - (Height_Sigma (Plane, X.Mean) / Facing) ** 2;
         begin
            Check_Close (Sum / Real (Trials) / Own, 1.0, 0.3, "distance variance from the ray's noise");
         end;
      end;
      declare
         X : Point_Estimate;
         D : Estimate;
      begin
         Intersect (Plane, Ray_To ([0.0, 0.0, 3.0], [0.0, 0.0, 6.0], 1.0e-3, 1.0e-3), X, D, Ok);
         Check (not Ok, "a ray pointing away from the plane met it");
      end;
   end Ray_Plane;

   procedure Register is
   begin
      Driver.Tests.Register ("geometry.meet", "rays through one point are said to meet elsewhere",
                             Meet_Exact'Access);
      Driver.Tests.Register ("geometry.meet_weights", "a ray counts the same whatever its own uncertainty",
                             Meet_Weights'Access);
      Driver.Tests.Register ("geometry.meet_covariance",
                             "the reported uncertainty of a meeting point is not what its rays' noise gives",
                             Meet_Covariance'Access);
      Driver.Tests.Register ("geometry.meet_behind", "a point behind a ray, or one ray alone, counts as a meeting",
                             Meet_Behind'Access);
      Driver.Tests.Register ("geometry.plane", "a plane's reported offset and tilt sigmas are not its real error",
                             Plane_Fit'Access);
      Driver.Tests.Register ("geometry.plane_robust", "things standing on a surface pull its plane up",
                             Plane_Robust'Access);
      Driver.Tests.Register ("geometry.ray_plane", "a ray meets a plane at the wrong distance or uncertainty",
                             Ray_Plane'Access);
   end Register;

end Driver.Geometry.Tests;
