with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Robot.Hand.Touch.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  A two-finger hand in its tool frame: the eye at O sees the tip along U.
   Tip_True : constant Vec3 := [0.13, 0.03, 0.0];
   Eye      : constant Vec3 := [0.05, 0.0, 0.05];
   Ray_Dir  : constant Vec3 := Unit (Tip_True - Eye);

   Pose_Sigma : constant Real := 5.0e-5;   --  the arm's reported position noise
   Turn_Sigma : constant Real := 1.0e-4;   --  and rotation noise

   Table : constant Geometry.Plane_Estimate :=
     (Centre => Zero3, Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
      Offset_Sigma => 2.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);

   function Pointing_Down return Mat3 is
      --  The turn that points the tip's line of sight straight into the table.
      Down : constant Vec3 := [0.0, 0.0, -1.0];
      Axis : constant Vec3 := Cross (Ray_Dir, Down);
   begin
      return Exp (Arcsin (abs Axis) * Unit (Axis));
   end Pointing_Down;

   --  A press with the tool turned by Tilt about a horizontal axis at
   --  azimuth Azimuth, the tip landing at (X, Y), Lift above the table (an
   --  early stop when positive, a sunk contact when negative) plus Contact
   --  noise; the reported pose carries the arm's noise.
   function Make_Press (Tilt, Azimuth, X, Y, Lift, Contact : Real;
                        Surface : Geometry.Plane_Estimate := Table) return Press is
      Horizontal : constant Vec3 := [Cos (Azimuth), Sin (Azimuth), 0.0];
      R : constant Mat3 := Exp (Tilt * Horizontal) * Pointing_Down;
      Landing : constant Vec3 := [X, Y, Lift + Contact * Gaussian];
      T : constant Vec3 := Landing - R * Tip_True;
      Noisy : constant Rigid :=
        (Rotation    => Exp (Turn_Sigma * [Gaussian, Gaussian, Gaussian]) * R,
         Translation => T + Pose_Sigma * [Gaussian, Gaussian, Gaussian]);
   begin
      return (Tool    => (Pose                => Noisy,
                          Position_Covariance => (Pose_Sigma ** 2) * Identity3,
                          Rotation_Covariance => (Turn_Sigma ** 2) * Identity3),
              Surface => Surface);
   end Make_Press;

   function Surface_Estimate (Offset_Sigma, Offset_Error : Real) return Geometry.Plane_Estimate is
      --  The table as measured: really at z = 0, estimated Offset_Error higher.
      P : Geometry.Plane_Estimate := Table;
   begin
      P.Centre := [0.0, 0.0, Offset_Error];
      P.Offset_Sigma := Offset_Sigma;
      return P;
   end Surface_Estimate;

   Origin : constant Point_Estimate := (Mean => Eye, Covariance => 1.0e-10 * Identity3);
   Sight  : constant Direction_Estimate := (Unit_Vector => Ray_Dir, Sigma => 1.0e-5);

   procedure On_Ray_With_Early_Stop is
      Presses : Press_Array (1 .. 7);
      F : Fit_Result;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 3);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0);
      end loop;
      --  The other finger rested on something 8 mm high.
      Presses (7) := Make_Press (0.0, 0.0, 0.5, 0.2, 0.008, 0.0);
      F := Fit_On_Ray (Presses, Origin, Sight);
      Check (F.Ok, "six good presses and one early stop gave no tip");
      Check (F.Stopped = 1 and then F.Sunk = 0 and then F.Used = 6,
             "the early stop was not the one left out:" & Natural'Image (F.Used) & Natural'Image (F.Stopped)
             & Natural'Image (F.Sunk));
      Check (not Uncertain.Significant (F.Distance.Value - abs (Tip_True - Eye), F.Distance.Sigma),
             "the tip's distance along its line of sight is off by more than its own sigma allows");
   end On_Ray_With_Early_Stop;

   procedure Free_With_Tilts is
      Presses : Press_Array (1 .. 8);
      F : Fit_Result;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (0.35, 2.0 * Ada.Numerics.Pi * Real (I) / 6.0, 0.4, 0.1 + 0.03 * Real (I), 0.0, 0.0);
      end loop;
      Presses (7) := Make_Press (0.0, 0.0, 0.45, 0.1, 0.0, 0.0);
      Presses (8) := Make_Press (0.2, 1.0, 0.45, 0.2, -0.004, 0.0);   --  pressed 4 mm into a soft spot
      F := Fit_Free (Presses);
      Check (F.Ok, "tilted presses gave no free tip");
      Check (F.Sunk = 1 and then F.Stopped = 0, "the sunk press was not the one left out");
      declare
         D : constant Vec3 := F.Tip.Mean - Tip_True;
         Mahalanobis : constant Real := D * (Inverse (F.Tip.Covariance) * D);
      begin
         --  Chi square with three degrees of freedom, beyond 3 sigma at about 14.2.
         Check (Mahalanobis < 14.2, "the free tip is off by more than its covariance allows:" & Real'Image (Mahalanobis));
      end;
   end Free_With_Tilts;

   procedure Scatter_Calibrated is
      --  Two regimes, each a thousand fits of six presses. Contacts that
      --  scatter 0.5 mm, ten times the arm's noise, on a well-known surface:
      --  the error over its reported sigma follows Student's t with five
      --  degrees of freedom, P (|t| > 1) = 0.3632. A surface known only to
      --  0.5 mm (and really off by that much) under exact contacts: its error
      --  is common to all presses and does not average away, the error over
      --  its sigma is Gaussian, P (|z| > 1) = 0.3173. Binomial sigma 0.015.
      Trials : constant := 1000;
      Beyond_Contact, Beyond_Surface : Natural := 0;
      Rejected : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 9);
      for T in 1 .. Trials loop
         declare
            Rough, Uncertain_Surface : Press_Array (1 .. 6);
            Surface : constant Geometry.Plane_Estimate := Surface_Estimate (5.0e-4, 5.0e-4 * Gaussian);
            F : Fit_Result;
         begin
            for I in Rough'Range loop
               Rough (I) := Make_Press (0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 5.0e-4,
                                        Surface_Estimate (1.0e-7, 0.0));
               Uncertain_Surface (I) := Make_Press (0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0, Surface);
            end loop;
            F := Fit_On_Ray (Rough, Origin, Sight);
            Check (F.Ok, "scattering contacts gave no tip");
            Rejected := Rejected + F.Stopped + F.Sunk;
            if abs (F.Distance.Value - abs (Tip_True - Eye)) > F.Distance.Sigma then
               Beyond_Contact := Beyond_Contact + 1;
            end if;
            F := Fit_On_Ray (Uncertain_Surface, Origin, Sight);
            Check (F.Ok, "an uncertain surface gave no tip");
            if abs (F.Distance.Value - abs (Tip_True - Eye)) > F.Distance.Sigma then
               Beyond_Surface := Beyond_Surface + 1;
            end if;
         end;
      end loop;
      --  At Z = 3 about 0.27 % of good presses would be rejected (16 of 6000).
      Check (Rejected <= 40, "good presses were rejected:" & Natural'Image (Rejected));
      Check_Close (Real (Beyond_Contact) / Real (Trials), 0.3632, 0.046, "scattering contacts: share beyond one sigma");
      Check_Close (Real (Beyond_Surface) / Real (Trials), 0.3173, 0.046, "uncertain surface: share beyond one sigma");
   end Scatter_Calibrated;

   procedure Undetermined is
      Presses : Press_Array (1 .. 6);
      F : Fit_Result;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 15);
      for I in Presses'Range loop
         Presses (I) := Make_Press (0.0, 0.0, 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0);
      end loop;
      F := Fit_Free (Presses);
      --  Presses from one orientation fix the tip only along the surface's
      --  normal; the other directions differ only by the arm's turn noise.
      if F.Ok then
         declare
            Along : constant Vec3 := Transpose (Pointing_Down) * [0.0, 0.0, 1.0];
            Across : constant Vec3 := Unit (Cross (Along, [1.0, 0.0, 0.0]));
            S_Along : constant Real := Sqrt (Along * (F.Tip.Covariance * Along));
            S_Across : constant Real := Sqrt (Across * (F.Tip.Covariance * Across));
         begin
            Check (S_Across > 100.0 * S_Along,
                   "presses from one orientation claimed to fix the tip across the press direction:"
                   & Real'Image (S_Across) & Real'Image (S_Along));
         end;
      end if;
      F := Fit_On_Ray (Presses (1 .. 1), Origin, Sight);
      Check (not F.Ok, "a single press gave a checked tip");
   end Undetermined;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.touch.on_ray", "a press that stopped on something else moves the tip",
                             On_Ray_With_Early_Stop'Access);
      Driver.Tests.Register ("hand.touch.free", "a press sunk into a soft contact moves the free tip",
                             Free_With_Tilts'Access);
      Driver.Tests.Register ("hand.touch.scatter",
                             "contacts less repeatable than the arm get rejected, or report the arm's precision",
                             Scatter_Calibrated'Access);
      Driver.Tests.Register ("hand.touch.undetermined", "a tip is claimed precise from presses that cannot fix it",
                             Undetermined'Access);
   end Register;

end Driver.Robot.Hand.Touch.Tests;
