with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Tests;

package body Driver.Robot.Hand.Touch.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  The generator returns [0, 1] with 1 included: U1 is drawn on (0, 1].
      U1 : Real;
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      loop
         U1 := Real (Ada.Numerics.Float_Random.Random (Gen));
         exit when U1 > 0.0;
      end loop;
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  A two-finger hand in its tool frame: the eye at Eye sees each tip
   --  along its own line of sight.
   Tips_True : constant array (1 .. 2) of Vec3 := [[0.13, 0.03, 0.0], [0.13, -0.03, 0.0]];
   Eye       : constant Vec3 := [0.05, 0.0, 0.05];

   function Direction_Of (Lobe : Positive) return Vec3 is (Unit (Tips_True (Lobe) - Eye));

   Pose_Sigma : constant Real := 5.0e-5;   --  the arm's reported position noise
   Turn_Sigma : constant Real := 1.0e-4;   --  and rotation noise

   Table : constant Geometry.Plane_Estimate :=
     (Centre => Zero3, Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
      Offset_Sigma => 2.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);

   function Sights return Sight_Array is
     ([for L in 1 .. 2 => (Origin    => (Mean => Eye, Covariance => 1.0e-10 * Identity3),
                           Direction => (Unit_Vector => Direction_Of (L), Sigma => 1.0e-5))]);

   function Pointing_Down (Lobe : Positive) return Mat3 is
      --  The turn that points the lobe's line of sight straight into the table.
      Down : constant Vec3 := [0.0, 0.0, -1.0];
      Axis : constant Vec3 := Cross (Direction_Of (Lobe), Down);
   begin
      return Exp (Arcsin (abs Axis) * Unit (Axis));
   end Pointing_Down;

   --  A press aimed at a lobe, the tool turned by Tilt about a horizontal
   --  axis at azimuth Azimuth, the tip landing at (X, Y), Lift above the
   --  table (an early stop when positive, a sunk contact when negative) plus
   --  Contact noise; the reported pose carries the arm's noise.
   function Make_Press (Lobe : Positive; Tilt, Azimuth, X, Y, Lift, Contact : Real; Slope : Real := 0.0)
     return Press
   is
      --  Slope tilts the table about the y axis: the table is z = -tan (Slope) x.
      Horizontal : constant Vec3 := [Cos (Azimuth), Sin (Azimuth), 0.0];
      R : constant Mat3 := Exp (Tilt * Horizontal) * Pointing_Down (Lobe);
      Normal : constant Vec3 := [Sin (Slope), 0.0, Cos (Slope)];
      Landing : constant Vec3 := [X, Y, -Tan (Slope) * X] + (Lift + Contact * Gaussian) * Normal;
      T : constant Vec3 := Landing - R * Tips_True (Lobe);
      Noisy : constant Rigid :=
        (Rotation    => Exp (Turn_Sigma * [Gaussian, Gaussian, Gaussian]) * R,
         Translation => T + Pose_Sigma * [Gaussian, Gaussian, Gaussian]);
   begin
      return (Tool    => (Pose                => Noisy,
                          Position_Covariance => (Pose_Sigma ** 2) * Identity3,
                          Rotation_Covariance => (Turn_Sigma ** 2) * Identity3),
              Sight   => Lobe,
              Surface => 1);
   end Make_Press;

   function Measured (Offset_Sigma, Offset_Error : Real) return Surface_Prior_Array is
      --  The table as measured: really at z = 0, estimated Offset_Error higher.
      P : Geometry.Plane_Estimate := Table;
   begin
      P.Centre := [0.0, 0.0, Offset_Error];
      P.Offset_Sigma := Offset_Sigma;
      return [1 => (Measured => True, Plane => P)];
   end Measured;

   Unknown_Table : constant Surface_Prior_Array (1 .. 1) := [1 => (Measured => False)];

   function Distance_True (Lobe : Positive) return Real is (abs (Tips_True (Lobe) - Eye));

   procedure On_Sight_With_Early_Stop is
      Presses : Press_Array (1 .. 7);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 3);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (1, 0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0);
      end loop;
      --  The other finger rested on something 8 mm high.
      Presses (7) := Make_Press (1, 0.0, 0.0, 0.5, 0.2, 0.008, 0.0);
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Ok, "six good presses and one early stop gave no tip");
         Check (F.Tips (1).Stopped = 1 and then F.Tips (1).Sunk = 0 and then F.Tips (1).Used = 6,
                "the early stop was not the one left out:" & Natural'Image (F.Tips (1).Used)
                & Natural'Image (F.Tips (1).Stopped) & Natural'Image (F.Tips (1).Sunk));
         Check (not Uncertain.Significant (F.Tips (1).Distance.Value - Distance_True (1), F.Tips (1).Distance.Sigma),
                "the tip's distance along its line of sight is off by more than its own sigma allows");
      end;
   end On_Sight_With_Early_Stop;

   procedure Free_With_Tilts is
      Presses : Press_Array (1 .. 8);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (1, 0.35, 2.0 * Ada.Numerics.Pi * Real (I) / 6.0, 0.4, 0.1 + 0.03 * Real (I),
                                    0.0, 0.0);
      end loop;
      Presses (7) := Make_Press (1, 0.0, 0.0, 0.45, 0.1, 0.0, 0.0);
      Presses (8) := Make_Press (1, 0.2, 1.0, 0.45, 0.2, -0.004, 0.0);   --  pressed 4 mm into a soft spot
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0), Free);
      begin
         Check (F.Ok and then F.Tips (1).Ok, "tilted presses gave no free tip");
         Check (F.Tips (1).Sunk = 1 and then F.Tips (1).Stopped = 0, "the sunk press was not the one left out");
         if F.Tips (1).Ok then
            declare
               D : constant Vec3 := F.Tips (1).Tip.Mean - Tips_True (1);
               Mahalanobis : constant Real := D * (Inverse (F.Tips (1).Tip.Covariance) * D);
            begin
               --  Chi square with three degrees of freedom, beyond 3 sigma at about 14.2.
               Check (Mahalanobis < 14.2, "the free tip is off by more than its covariance allows:"
                      & Real'Image (Mahalanobis));
            end;
         end if;
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
            Rough, Exact : Press_Array (1 .. 6);
            Surface : constant Surface_Prior_Array := Measured (5.0e-4, 5.0e-4 * Gaussian);
         begin
            for I in Rough'Range loop
               Rough (I) := Make_Press (1, 0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 5.0e-4);
               Exact (I) := Make_Press (1, 0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0);
            end loop;
            declare
               F : constant Fit_Result := Fit (Rough, Sights (1 .. 1), Measured (1.0e-7, 0.0));
            begin
               Check (F.Ok, "scattering contacts gave no tip");
               Rejected := Rejected + F.Tips (1).Stopped + F.Tips (1).Sunk;
               if abs (F.Tips (1).Distance.Value - Distance_True (1)) > F.Tips (1).Distance.Sigma then
                  Beyond_Contact := Beyond_Contact + 1;
               end if;
            end;
            declare
               F : constant Fit_Result := Fit (Exact, Sights (1 .. 1), Surface);
            begin
               Check (F.Ok, "an uncertain surface gave no tip");
               if abs (F.Tips (1).Distance.Value - Distance_True (1)) > F.Tips (1).Distance.Sigma then
                  Beyond_Surface := Beyond_Surface + 1;
               end if;
            end;
         end;
      end loop;
      --  At Z = 3 about 0.27 % of good presses would be rejected (16 of 6000).
      Check (Rejected <= 40, "good presses were rejected:" & Natural'Image (Rejected));
      Check_Close (Real (Beyond_Contact) / Real (Trials), 0.3632, 0.046, "scattering contacts: share beyond one sigma");
      Check_Close (Real (Beyond_Surface) / Real (Trials), 0.3173, 0.046, "uncertain surface: share beyond one sigma");
   end Scatter_Calibrated;

   --  Presses on a table nothing measured before, both lobes, each at six
   --  places and tilted by 0, 0.3 and 0.6 rad: a tip's distance along its
   --  line shows only in how much the tilt lifts the eye, 1 - cos (tilt),
   --  so the tilts must differ in size, not only in direction. The contacts
   --  scatter ten times the arm's noise; the table and both tips come out
   --  together. The table slopes, so the lines pressed into it are not along
   --  its normal and the start's guess of it has to be corrected.
   Table_Slope : constant Real := 0.15;

   function Unknown_Table_Presses return Press_Array is
     ([for I in 1 .. 12 =>
         Make_Press ((if I <= 6 then 1 else 2), 0.3 * Real (I mod 3), Ada.Numerics.Pi * Real (I mod 2),
                     0.4 + 0.03 * Real (I mod 6), 0.1 + 0.05 * Real ((I - 1) / 6) + 0.02 * Real (I mod 4), 0.0, 5.0e-4,
                     Slope => Table_Slope)]);

   procedure Table_From_Presses is
      --  Five hundred fits: each tip's distance and the table's height under
      --  the first tip, against their sigmas. The scatter sets the noise, so
      --  each error over its sigma follows Student's t with the presses' 7
      --  degrees of freedom (12 presses, 2 distances and 3 for the table).
      Trials  : constant := 500;
      Nominal : constant Real := Driver.Distributions.Student_T_Two_Sided_Tail (1.0, 7);
      Spread  : constant Real := Sqrt (Nominal * (1.0 - Nominal) / Real (Trials));
      Beyond_Tip, Beyond_Table : Natural := 0;
      Fitted  : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 21);
      for T in 1 .. Trials loop
         declare
            F : constant Fit_Result := Fit (Unknown_Table_Presses, Sights, Unknown_Table);
         begin
            if F.Ok and then F.Tips (1).Ok and then F.Tips (2).Ok then
               Fitted := Fitted + 1;
               for L in 1 .. 2 loop
                  if abs (F.Tips (L).Distance.Value - Distance_True (L)) > F.Tips (L).Distance.Sigma then
                     Beyond_Tip := Beyond_Tip + 1;
                  end if;
               end loop;
               declare
                  Under : constant Vec3 := [0.45, 0.12, -Tan (Table_Slope) * 0.45];
               begin
                  if abs Geometry.Height (F.Planes (1), Under) > Geometry.Height_Sigma (F.Planes (1), Under) then
                     Beyond_Table := Beyond_Table + 1;
                  end if;
               end;
            end if;
         end;
      end loop;
      Check (Fitted = Trials, "presses at several places and tilts gave no table in"
             & Natural'Image (Trials - Fitted) & " trials");
      Check (abs (Real (Beyond_Tip) / Real (2 * Fitted) - Nominal) <= Driver.Conventions.Z * Spread,
             "tips on an unknown table: share beyond one sigma" & Real'Image (Real (Beyond_Tip) / Real (2 * Fitted))
             & " against" & Real'Image (Nominal));
      Check (abs (Real (Beyond_Table) / Real (Fitted) - Nominal) <= Driver.Conventions.Z * Spread,
             "the table from presses: share beyond one sigma" & Real'Image (Real (Beyond_Table) / Real (Fitted))
             & " against" & Real'Image (Nominal));
   end Table_From_Presses;

   procedure Undetermined is
      Presses : Press_Array (1 .. 6);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 15);
      for I in Presses'Range loop
         Presses (I) := Make_Press (1, 0.0, 0.0, 0.4 + 0.02 * Real (I), 0.1 + 0.01 * Real (I mod 2), 0.0, 0.0);
      end loop;
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0), Free);
      begin
         --  Presses from one orientation fix the tip only along the surface's
         --  normal; the other directions differ only by the arm's turn noise.
         if F.Ok and then F.Tips (1).Ok then
            declare
               Along    : constant Vec3 := Transpose (Pointing_Down (1)) * [0.0, 0.0, 1.0];
               Across   : constant Vec3 := Unit (Cross (Along, [1.0, 0.0, 0.0]));
               S_Along  : constant Real := Sqrt (Along * (F.Tips (1).Tip.Covariance * Along));
               S_Across : constant Real := Sqrt (Across * (F.Tips (1).Tip.Covariance * Across));
            begin
               Check (S_Across > 100.0 * S_Along,
                      "presses from one orientation claimed to fix the tip across the press direction:"
                      & Real'Image (S_Across) & Real'Image (S_Along));
            end;
         end if;
      end;
      --  On a table nothing measured, one orientation cannot tell how far
      --  along its line the tip is from how high the table is.
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Unknown_Table);
      begin
         Check (not F.Ok or else not F.Tips (1).Ok or else F.Tips (1).Distance.Sigma > F.Tips (1).Distance.Value,
                "presses from one orientation on an unknown table fixed the tip's distance to"
                & Real'Image (F.Tips (1).Distance.Sigma));
      end;
      Check (not Fit (Presses (1 .. 1), Sights (1 .. 1), Measured (2.0e-4, 0.0)).Ok, "a single press gave a checked tip");
   end Undetermined;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.touch.on_sight", "a press that stopped on something else moves the tip",
                             On_Sight_With_Early_Stop'Access);
      Driver.Tests.Register ("hand.touch.free", "a press sunk into a soft contact moves the free tip",
                             Free_With_Tilts'Access);
      Driver.Tests.Register ("hand.touch.scatter",
                             "contacts less repeatable than the arm get rejected, or report the arm's precision",
                             Scatter_Calibrated'Access);
      Driver.Tests.Register ("hand.touch.table", "tips and a table found together carry the wrong uncertainty",
                             Table_From_Presses'Access);
      Driver.Tests.Register ("hand.touch.undetermined", "a tip is claimed precise from presses that cannot fix it",
                             Undetermined'Access);
   end Register;

end Driver.Robot.Hand.Touch.Tests;
