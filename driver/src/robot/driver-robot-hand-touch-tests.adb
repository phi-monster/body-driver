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
   function Make_Press (Lobe : Positive; Tilt, Azimuth, X, Y, Lift, Contact : Real; Slope : Real := 0.0;
                        Offset : Vec3 := Zero3)
     return Press
   is
      --  Slope tilts the table about the y axis: the table is z = -tan (Slope) x.
      Horizontal : constant Vec3 := [Cos (Azimuth), Sin (Azimuth), 0.0];
      R : constant Mat3 := Exp (Tilt * Horizontal) * Pointing_Down (Lobe);
      Normal : constant Vec3 := [Sin (Slope), 0.0, Cos (Slope)];
      Landing : constant Vec3 := [X, Y, -Tan (Slope) * X] + (Lift + Contact * Gaussian) * Normal;
      --  Offset is how far the point of the lobe that touches stands from the tip on the line of sight, in the tool
      --  frame: the lobe's tip region is wider than the line.
      T : constant Vec3 := Landing - R * (Tips_True (Lobe) + Offset);
      Noisy : constant Rigid :=
        (Rotation    => Exp (Turn_Sigma * [Gaussian, Gaussian, Gaussian]) * R,
         Translation => T + Pose_Sigma * [Gaussian, Gaussian, Gaussian]);
   begin
      return (Tool    => (Pose                => Noisy,
                          Position_Covariance => (Pose_Sigma ** 2) * Identity3,
                          Rotation_Covariance => (Turn_Sigma ** 2) * Identity3),
              Sight   => Lobe,
              Surface => 1,
              others  => <>);
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
         Check (F.Tips (1).Confirmed, "six presses at distinct poses on one tip did not confirm it");
         Check (not F.Agrees (7) and then (for all I in 1 .. 6 => F.Agrees (I)), "the presses the tip rests on are not said");
         Check (not Uncertain.Significant (F.Tips (1).Distance.Value - Distance_True (1), F.Tips (1).Distance.Sigma),
                "the tip's distance along its line of sight is off by more than its own sigma allows");
      end;
   end On_Sight_With_Early_Stop;

   procedure Behind_The_Eye is
      --  A press whose line of sight to the tip points away from the surface: it meets it behind the eye, at
      --  a negative distance (A19: a press aimed at one lobe and given to the other fitted the tip -79.4 units
      --  along its sight). That is not the line of a tip that stopped the arm on the surface: no tip, and the
      --  press is not one a tip rests on. The same press of the lobe turned the right way gives its tip.
      Backwards : Press_Array (1 .. 1);
      Forwards  : Press_Array (1 .. 1);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      Backwards (1) := Make_Press (1, Ada.Numerics.Pi, 0.0, 0.40, 0.10, 0.25, 0.0);
      Forwards (1) := Make_Press (1, 0.0, 0.0, 0.40, 0.10, 0.0, 0.0);
      declare
         F : constant Fit_Result := Fit (Backwards, Sights (1 .. 1), Measured (2.0e-4, 0.0));
         G : constant Fit_Result := Fit (Forwards, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (not F.Tips (1).Ok and then not F.Agrees (1),
                "a line of sight that meets the surface behind the eye gave a tip at" & Real'Image (F.Tips (1).Distance.Value)
                & ", which a press rests on:" & F.Agrees (1)'Image);
         Check (G.Tips (1).Ok and then G.Agrees (1) and then G.Tips (1).Distance.Value > 0.0,
                "the same press turned the right way gave no tip in front of the eye");
      end;
   end Behind_The_Eye;

   procedure Provisional_And_Stalls is
      --  The presses of A16's first hand, as the arm made them: the first
      --  straight along the line of sight and stopped by the tip, the other
      --  two tilted and stopped by the arm against itself, 60 and 30 mm short
      --  of the table. The tip is the first press's and nothing has checked
      --  it: provisional. The others only bound it from above. (The old fit
      --  raised its noise to the scatter of the three, five hundred times
      --  its own, took all three for agreeing and put the tip where none
      --  of them had.)
      Presses : Press_Array (1 .. 3);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 4);
      Presses (1) := Make_Press (1, 0.0, 0.0, 0.45, 0.10, 0.0, 0.0);
      Presses (2) := Make_Press (1, 0.6, 1.0, 0.40, 0.15, 0.06, 0.0);
      Presses (3) := Make_Press (1, 0.9, 4.0, 0.50, 0.12, 0.03, 0.0);
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Ok, "a press the tip stopped and two that stopped above the table gave no tip");
         Check (F.Tips (1).Used = 1 and then F.Tips (1).Stopped = 2 and then not F.Tips (1).Confirmed,
                "the tip rests on" & F.Tips (1).Used'Image & " presses," & F.Tips (1).Stopped'Image
                & " stopped above the table, confirmed " & F.Tips (1).Confirmed'Image);
         Check (F.Agrees (1) and then not F.Agrees (2) and then not F.Agrees (3), "the press the tip rests on is not said");
         Check (F.Hits (1) > 0.0 and then F.Hits (1) < F.Hits (3) and then F.Hits (3) < F.Hits (2),
                "the hits of a contact and two stops 30 and 60 mm short are not in the order of their lifts:"
                & Real'Image (F.Hits (1)) & Real'Image (F.Hits (3)) & Real'Image (F.Hits (2)));
         Check (abs (F.Hits (1) - F.Tips (1).Distance.Value) <= F.Tips (1).Distance.Sigma,
                "the hit of the press the tip rests on is not the tip's distance");
         Check (not Uncertain.Significant (F.Tips (1).Distance.Value - Distance_True (1), F.Tips (1).Distance.Sigma),
                "the tip is off by" & Real'Image (F.Tips (1).Distance.Value - Distance_True (1)) & ", more than its sigma"
                & Real'Image (F.Tips (1).Distance.Sigma));
      end;
      --  The same two stops made twice from one pose are one stop twice: they
      --  do not stand for a second press, and a contact made twice from one
      --  pose is not checked by itself.
      Presses (3) := Presses (2);
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Used = 1 and then F.Tips (1).Stopped = 2 and then not F.Tips (1).Confirmed,
                "two stops from one pose changed the tip: used" & F.Tips (1).Used'Image & ", stopped"
                & F.Tips (1).Stopped'Image);
      end;
      Presses (2) := Presses (1);
      declare
         F : constant Fit_Result := Fit (Presses (1 .. 2), Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Used = 2 and then not F.Tips (1).Confirmed,
                "a contact made twice from one pose confirmed itself: used" & F.Tips (1).Used'Image);
      end;
   end Provisional_And_Stalls;

   procedure Confirmed_By_A_Second_Pose is
      --  Two contacts from poses apart land on one tip, a third stopped 50 mm
      --  short of the table: the tip is confirmed, the third only bounds it.
      Presses : Press_Array (1 .. 3);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 6);
      Presses (1) := Make_Press (1, 0.0, 0.0, 0.45, 0.10, 0.0, 0.0);
      Presses (2) := Make_Press (1, 0.4, 2.0, 0.40, 0.15, 0.0, 0.0);
      Presses (3) := Make_Press (1, 0.7, 4.0, 0.50, 0.12, 0.05, 0.0);
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Ok and then F.Tips (1).Used = 2 and then F.Tips (1).Stopped = 1
                and then F.Tips (1).Confirmed,
                "two contacts and a stop: used" & F.Tips (1).Used'Image & ", stopped" & F.Tips (1).Stopped'Image
                & ", confirmed " & F.Tips (1).Confirmed'Image);
         Check (not Uncertain.Significant (F.Tips (1).Distance.Value - Distance_True (1), F.Tips (1).Distance.Sigma),
                "the confirmed tip is off by more than its sigma");
      end;
      --  The second contact 4 mm short: it does not land on the first, and a
      --  tip that two presses were meant to check is left to the lower one.
      Presses (2) := Make_Press (1, 0.4, 2.0, 0.40, 0.15, 0.004, 0.0);
      declare
         F : constant Fit_Result := Fit (Presses (1 .. 2), Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Used = 1 and then F.Tips (1).Stopped = 1 and then not F.Tips (1).Confirmed,
                "a second press 4 mm short confirmed the first: used" & F.Tips (1).Used'Image);
      end;
   end Confirmed_By_A_Second_Pose;

   procedure Sunk_Press_Lowers_The_Tip is
      --  A press that finds the tip 4 mm below the surface: the tip cannot be
      --  there, so the presses that left it above the surface are the ones
      --  that stopped on something else, and the tip is the sunk press's. It
      --  stays provisional, nothing agreeing with it.
      Presses : Press_Array (1 .. 7);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (1, 0.35, 2.0 * Ada.Numerics.Pi * Real (I) / 6.0, 0.4, 0.1 + 0.03 * Real (I), 0.0, 0.0);
      end loop;
      Presses (7) := Make_Press (1, 0.0, 0.0, 0.45, 0.2, -0.004, 0.0);
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Ok, "six contacts and a sunk press gave no tip");
         Check (F.Tips (1).Used = 1 and then F.Tips (1).Stopped = 6 and then F.Agrees (7)
                and then not F.Tips (1).Confirmed,
                "the sunk press was not the tip's:" & F.Tips (1).Used'Image & F.Tips (1).Stopped'Image);
         Check (F.Tips (1).Distance.Value < Distance_True (1) - 0.003,
                "the tip is not 4 mm nearer the eye than the contacts left it:"
                & Real'Image (Distance_True (1) - F.Tips (1).Distance.Value));
      end;
   end Sunk_Press_Lowers_The_Tip;

   procedure Free_With_Tilts is
      Presses : Press_Array (1 .. 8);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      for I in 1 .. 6 loop
         Presses (I) := Make_Press (1, 0.35, 2.0 * Ada.Numerics.Pi * Real (I) / 6.0, 0.4, 0.1 + 0.03 * Real (I),
                                    0.0, 0.0);
      end loop;
      Presses (7) := Make_Press (1, 0.0, 0.0, 0.45, 0.1, 0.0, 0.0);
      Presses (8) := Make_Press (1, 0.2, 1.0, 0.45, 0.2, 0.006, 0.0);   --  stopped 6 mm above the table
      declare
         F : constant Fit_Result := Fit (Presses, Sights (1 .. 1), Measured (2.0e-4, 0.0), Free);
      begin
         Check (F.Ok and then F.Tips (1).Ok, "tilted presses gave no free tip");
         Check (F.Tips (1).Stopped = 1 and then F.Tips (1).Sunk = 0 and then not F.Agrees (8),
                "the press that stopped above the table was not the one left out");
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

   procedure Calibrated is
      --  Two regimes, each a thousand fits of six presses. Contacts exact to
      --  the arm's noise on a surface known only to 0.5 mm (and really off by
      --  that much): the surface's error is common to all presses and does
      --  not average away, and the error over the tip's reported sigma is
      --  Gaussian, P (|z| > 1) = 0.3173, binomial sigma 0.015.
      --  Contacts that scatter 0.5 mm, ten times the arm's noise: the noise
      --  is not raised to meet them, so they read as the lowest of them, the
      --  tip within the scatter of the truth and never above it by more.
      Trials : constant := 1000;
      Beyond_Surface : Natural := 0;
      Scatter : constant Real := 5.0e-4;
      Worst   : Real := 0.0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 9);
      for T in 1 .. Trials loop
         declare
            Rough, Exact : Press_Array (1 .. 6);
            Surface : constant Surface_Prior_Array := Measured (5.0e-4, 5.0e-4 * Gaussian);
         begin
            for I in Rough'Range loop
               Rough (I) := Make_Press (1, 0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, Scatter);
               Exact (I) := Make_Press (1, 0.0, Real (I), 0.4 + 0.02 * Real (I), 0.1, 0.0, 0.0);
            end loop;
            declare
               F : constant Fit_Result := Fit (Rough, Sights (1 .. 1), Measured (1.0e-7, 0.0));
            begin
               Check (F.Ok and then F.Tips (1).Ok, "scattering contacts gave no tip");
               Worst := Real'Max (Worst, abs (F.Tips (1).Distance.Value - Distance_True (1)));
            end;
            declare
               F : constant Fit_Result := Fit (Exact, Sights (1 .. 1), Surface);
            begin
               Check (F.Ok and then F.Tips (1).Ok, "an uncertain surface gave no tip");
               if abs (F.Tips (1).Distance.Value - Distance_True (1)) > F.Tips (1).Distance.Sigma then
                  Beyond_Surface := Beyond_Surface + 1;
               end if;
            end;
         end;
      end loop;
      Check (Worst <= 5.0 * Scatter, "contacts scattering" & Real'Image (Scatter) & " put the tip off by"
             & Real'Image (Worst));
      Check_Close (Real (Beyond_Surface) / Real (Trials), Driver.Distributions.Gaussian_Two_Sided_Tail (1.0), 0.046,
                   "uncertain surface: share beyond one sigma");
   end Calibrated;

   --  Presses on a table nothing measured before, both lobes, each at six
   --  places and tilted by 0, 0.3 and 0.6 rad: a tip's distance along its
   --  line shows only in how much the tilt lifts the eye, 1 - cos (tilt),
   --  so the tilts must differ in size, not only in direction. The contacts
   --  are exact to the arm's noise; the table and both tips come out
   --  together. The table slopes, so the lines pressed into it are not along
   --  its normal and the start's guess of it has to be corrected.
   Table_Slope : constant Real := 0.15;

   function Unknown_Table_Presses return Press_Array is
     ([for I in 1 .. 12 =>
         Make_Press ((if I <= 6 then 1 else 2), 0.3 * Real (I mod 3), Ada.Numerics.Pi * Real (I mod 2),
                     0.4 + 0.03 * Real (I mod 6), 0.1 + 0.05 * Real ((I - 1) / 6) + 0.02 * Real (I mod 4), 0.0, 0.0,
                     Slope => Table_Slope)]);

   procedure Table_From_Presses is
      --  Five hundred fits: each tip's distance and the table's height under
      --  the first tip, against their sigmas, which the arm's noise sets: each
      --  error over its sigma is Gaussian, P (|z| > 1) = 0.3173.
      Trials  : constant := 500;
      Nominal : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (1.0);
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
      --  A single press, the surface measured before, fixes the tip at the
      --  distance its line of sight meets the surface; nothing checks it.
      declare
         F : constant Fit_Result := Fit (Presses (1 .. 1), Sights (1 .. 1), Measured (2.0e-4, 0.0));
      begin
         Check (F.Ok and then F.Tips (1).Ok and then F.Tips (1).Used = 1 and then not F.Tips (1).Confirmed,
                "a single press did not give a provisional tip");
         Check (not Uncertain.Significant (F.Tips (1).Distance.Value - Distance_True (1), F.Tips (1).Distance.Sigma),
                "a single press put the tip off by more than its sigma");
      end;
      Check (not Fit (Presses (1 .. 1), Sights (1 .. 1), Unknown_Table).Ok, "a single press on an unknown table gave a tip");
   end Undetermined;

   procedure Lateral_Offset_Needs_Tilts is
      --  The point of the lobe that touches is not on the line of sight to its tip pixel but somewhere in the tip
      --  region (0.07 rad wide here: 0.007 at that distance); here it stands 0.012 across the line. A press at a
      --  pose turned about the eye meets the surface at a different distance for a tip off the line, by the offset
      --  times the difference of the tangents of the tilts, so presses tell the offset along the turn between
      --  them and nothing across it, however many there are or however far apart in place:
      --  (a) straight presses from six places agree on the distance and tell nothing across: not Tested, not
      --      Confirmed, and the tip is as wide across the line as the region;
      --  (b) presses straight and tilted 0.6 rad one way tell one axis: the tip narrows along it, and is not
      --      Tested nor Confirmed;
      --  (c) tilted both ways, 0.6 rad about two axes, they tell both: Tested and Confirmed, and the tip is the
      --      vertex that touched, not the line's point;
      --  (d) tilted by a hair, 0.003 rad about both axes (thirty times the arm's turning noise, and still too
      --      little to tell an offset of the tip region's size from the noise of the heights), they tell no more
      --      than the straight ones.
      Region : constant Real := 0.07;
      U      : constant Vec3 := Direction_Of (1);
      Across : constant Vec3 := Unit (Cross (U, [0.0, 0.0, 1.0]));
      Along_Z : constant Vec3 := Cross (U, Across);
      Offset : constant Vec3 := 0.012 * Across;
      Wide   : constant Sight_Array :=
        [1 => (Origin    => (Mean => Eye, Covariance => 1.0e-10 * Identity3),
               Direction => (Unit_Vector => U, Sigma => Region))];
      Prior_Length : constant Real := Distance_True (1) * Region;

      --  The variances of the tip along the two axes across its line, the least and the most told.
      procedure Across_Variances (T : Point_Estimate; Least, Most : out Real) is
         A : constant Real := Across * (T.Covariance * Across);
         B : constant Real := Along_Z * (T.Covariance * Along_Z);
         C : constant Real := Across * (T.Covariance * Along_Z);
         Mid : constant Real := (A + B) / 2.0;
         Gap : constant Real := Sqrt (((A - B) / 2.0) ** 2 + C ** 2);
      begin
         Least := Mid - Gap;
         Most := Mid + Gap;
      end Across_Variances;

      function Fitted (Tilt_1, Tilt_2 : Real; Straight, Each : Positive) return Fit_Result is
         Presses : Press_Array (1 .. Straight + 2 * Each);
         N       : Natural := 0;
      begin
         Ada.Numerics.Float_Random.Reset (Gen, 8);
         for I in 1 .. Straight loop
            N := N + 1;
            Presses (N) := Make_Press (1, 0.0, 0.0, 0.40 + 0.03 * Real (I), 0.10 + 0.02 * Real (I), 0.0, 0.0, Offset => Offset);
         end loop;
         for I in 1 .. Each loop
            N := N + 1;
            Presses (N) := Make_Press (1, Tilt_1, 0.0, 0.42 + 0.03 * Real (I), 0.12 + 0.02 * Real (I), 0.0, 0.0, Offset => Offset);
            N := N + 1;
            Presses (N) := Make_Press (1, Tilt_2, Ada.Numerics.Pi / 2.0, 0.44 + 0.03 * Real (I), 0.14 + 0.02 * Real (I), 0.0, 0.0,
                                       Offset => Offset);
         end loop;
         return Fit (Presses, Wide, Measured (2.0e-4, 0.0));
      end Fitted;
   begin
      --  (a)
      declare
         F : constant Fit_Result := Fitted (0.0, 0.0, 6, 1);
         Least, Most : Real;
      begin
         Check (F.Ok and then F.Tips (1).Ok, "straight presses gave no tip");
         if F.Tips (1).Ok then
            Across_Variances (F.Tips (1).Tip, Least, Most);
            Check (not F.Tips (1).Tested and then not F.Tips (1).Confirmed,
                   "presses all straight along the line, from distinct places, tested the tip across it:"
                   & F.Tips (1).Tested'Image & " confirmed:" & F.Tips (1).Confirmed'Image);
            Check (Least > 0.8 * Prior_Length ** 2,
                   "straight presses narrowed the tip across its line to" & Real'Image (Sqrt (Least)) & " from" & Real'Image (Prior_Length));
         end if;
      end;
      --  (b) Tilted about one axis only (the second tilt is none).
      declare
         F : constant Fit_Result := Fitted (0.6, 0.0, 2, 3);
         Least, Most : Real;
      begin
         if F.Tips (1).Ok then
            Across_Variances (F.Tips (1).Tip, Least, Most);
            Check (Most > 0.8 * Prior_Length ** 2 and then Least < 0.02 * Prior_Length ** 2,
                   "tilts about one axis did not narrow the tip along it and only it: across" & Real'Image (Sqrt (Least))
                   & " and" & Real'Image (Sqrt (Most)) & " of a prior" & Real'Image (Prior_Length));
            Check (not F.Tips (1).Tested and then not F.Tips (1).Confirmed,
                   "tilts about one axis tested the tip across the line:" & F.Tips (1).Tested'Image);
         else
            Check (False, "tilts about one axis gave no tip");
         end if;
      end;
      --  (c)
      declare
         F : constant Fit_Result := Fitted (0.6, 0.6, 2, 3);
         Least, Most : Real;
      begin
         Check (F.Ok and then F.Tips (1).Ok, "tilts about two axes gave no tip");
         if F.Tips (1).Ok then
            Across_Variances (F.Tips (1).Tip, Least, Most);
            Check (F.Tips (1).Tested and then F.Tips (1).Confirmed,
                   "tilts about both axes did not test and confirm the tip: tested " & F.Tips (1).Tested'Image
                   & ", confirmed " & F.Tips (1).Confirmed'Image & ", across" & Real'Image (Sqrt (Least)) & " and"
                   & Real'Image (Sqrt (Most)) & " of a prior" & Real'Image (Prior_Length));
            declare
               D : constant Vec3 := F.Tips (1).Tip.Mean - (Tips_True (1) + Offset);
               Mahalanobis : constant Real := D * (Inverse (F.Tips (1).Tip.Covariance) * D);
               Line_Point : constant Vec3 := Tips_True (1);
            begin
               Check (Mahalanobis < 14.2, "the tip is not the vertex that touched: off by" & Real'Image (abs D)
                      & ", Mahalanobis" & Real'Image (Mahalanobis) & ", the line's point is off by" & Real'Image (abs Offset));
               Check (abs D < 0.5 * abs (Offset) and then abs (F.Tips (1).Tip.Mean - Line_Point) > 0.5 * abs Offset,
                      "the tip did not move off the line to the vertex that touched:" & Real'Image (abs D));
            end;
         end if;
      end;
      --  (d)
      declare
         F : constant Fit_Result := Fitted (0.003, 0.003, 2, 3);
         Least, Most : Real := 0.0;
      begin
         if F.Tips (1).Ok then
            Across_Variances (F.Tips (1).Tip, Least, Most);
         end if;
         Check (F.Ok and then F.Tips (1).Ok and then not F.Tips (1).Tested and then not F.Tips (1).Confirmed,
                "presses tilted by a hair tested the tip across its line: across" & Real'Image (Sqrt (Least)) & " and"
                & Real'Image (Sqrt (Most)) & " of a prior" & Real'Image (Prior_Length) & ", along "
                & Real'Image (F.Tips (1).Distance.Sigma));
      end;
   end Lateral_Offset_Needs_Tilts;

   procedure Poses_Apart is
      --  Two poses are apart when their positions or their turns are by more
      --  than their uncertainty tells them from.
      Pose : constant Pose_Estimate :=
        (Pose                => (Rotation => Identity3, Translation => [0.3, 0.1, 0.2]),
         Position_Covariance => (Pose_Sigma ** 2) * Identity3,
         Rotation_Covariance => (Turn_Sigma ** 2) * Identity3);
      Moved, Turned : Pose_Estimate := Pose;
   begin
      Moved.Pose.Translation := Pose.Pose.Translation + [0.0, 0.0, 20.0 * Pose_Sigma];
      Turned.Pose.Rotation := Exp ([0.0, 20.0 * Turn_Sigma, 0.0]);
      Check (not Distinct (Pose, Pose), "a pose is apart from itself");
      Check (not Distinct (Pose, Pose_Estimate'(Pose => (Rotation => Exp ([0.0, 0.1 * Turn_Sigma, 0.0]),
                                                         Translation => Pose.Pose.Translation + [0.1 * Pose_Sigma, 0.0, 0.0]),
                                                Position_Covariance => Pose.Position_Covariance,
                                                Rotation_Covariance => Pose.Rotation_Covariance)),
             "a pose a tenth of a sigma away is apart");
      Check (Distinct (Pose, Moved), "a pose twenty sigmas along is not apart");
      Check (Distinct (Pose, Turned), "a pose turned twenty sigmas is not apart");
   end Poses_Apart;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.touch.on_sight", "a press that stopped on something else moves the tip",
                             On_Sight_With_Early_Stop'Access);
      Driver.Tests.Register ("hand.touch.behind", "a line of sight that meets the surface behind the eye gives a tip",
                             Behind_The_Eye'Access);
      Driver.Tests.Register ("hand.touch.provisional",
                             "stops on something else are taken for a tip's presses, or one press is taken for checked",
                             Provisional_And_Stalls'Access);
      Driver.Tests.Register ("hand.touch.confirmed",
                             "a second press from another pose does not confirm a tip, or one 4 mm short does",
                             Confirmed_By_A_Second_Pose'Access);
      Driver.Tests.Register ("hand.touch.sunk", "a press that finds the tip below the surface is left out for the ones above",
                             Sunk_Press_Lowers_The_Tip'Access);
      Driver.Tests.Register ("hand.touch.free", "a press that stopped above the table moves the free tip",
                             Free_With_Tilts'Access);
      Driver.Tests.Register ("hand.touch.calibrated",
                             "contacts that scatter get averaged into a tip they did not make, or the surface's error is lost",
                             Calibrated'Access);
      Driver.Tests.Register ("hand.touch.table", "tips and a table found together carry the wrong uncertainty",
                             Table_From_Presses'Access);
      Driver.Tests.Register ("hand.touch.undetermined", "a tip is claimed precise from presses that cannot fix it",
                             Undetermined'Access);
      Driver.Tests.Register ("hand.touch.apart", "two poses are told apart that the uncertainty cannot, or the reverse",
                             Poses_Apart'Access);
      Driver.Tests.Register ("hand.touch.lateral", "presses straight or tilted by a hair confirm a tip across its line, "
                             & "or tilts about one axis do, or tilts about two do not move it to the vertex that touched",
                             Lateral_Offset_Needs_Tilts'Access);
   end Register;

end Driver.Robot.Hand.Touch.Tests;
