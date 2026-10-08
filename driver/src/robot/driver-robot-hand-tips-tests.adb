with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Robot.Hand.Tips.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Clock.Beat;

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
      return Sqrt (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  Two fingers in the tool frame, apart when open, nearly touching when
   --  closed; the eye looks at both from behind and above.
   type Tip_Table is array (1 .. 2, Opening) of Vec3;
   Tips_True : constant Tip_Table :=
     [1 => [Open => [0.13, 0.04, 0.0], Closed_Empty => [0.14, 0.008, 0.0]],
      2 => [Open => [0.13, -0.04, 0.0], Closed_Empty => [0.14, -0.008, 0.0]]];
   Eye : constant Vec3 := [0.05, 0.0, 0.05];

   Pose_Sigma    : constant Real := 5.0e-5;
   Turn_Sigma    : constant Real := 1.0e-4;
   Contact_Sigma : constant Real := 2.0e-5;   --  contacts as repeatable as the arm is (see Driver.Robot.Hand.Touch)

   --  The table z = 0 as the arm's own eye saw it before any press, its
   --  height known to a fifth of a millimetre and its tilt to a ten-thousandth.
   Table : constant Driver.Geometry.Plane_Estimate :=
     (Centre => Zero3, Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
      Offset_Sigma => 2.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);

   function No_Pose (Arm : Real_Array) return Pose_Estimate is
      pragma Unreferenced (Arm);
      Never_Measured : Pose_Estimate;
   begin
      return Never_Measured;
   end No_Pose;
   --  The presses of these tests keep no arm readings: their poses stay.

   function Line (L : Positive; O : Opening) return Vec3 is (Unit (Tips_True (L, O) - Eye));

   --  The lobes' lines of sight, each with the spread of its tip region across it (an angle).
   function Sights (Spread : Real := 0.0) return Sight_Table is
     ([for L in 1 .. 2 =>
         [for O in Opening => (Known  => True,
                               Ray    => (Origin    => (Mean => Eye, Covariance => 1.0e-10 * Identity3),
                                          Direction => (Unit_Vector => Line (L, O), Sigma => 1.0e-5)),
                               Spread => Spread)]]);

   Down : constant Vec3 := [0.0, 0.0, -1.0];

   function Pointing_Down (L : Positive; O : Opening) return Mat3 is
      Axis : constant Vec3 := Cross (Line (L, O), Down);
   begin
      return Exp (Arcsin (abs Axis) * Unit (Axis));
   end Pointing_Down;

   --  A press aimed at one lobe on the table z = 0, made only when that lobe
   --  is really the one that touches; the tool reports its pose with the
   --  arm's noise, and was pressing straight down. Lift is how far above the
   --  table the tip stopped: the arm stopped on something else.
   procedure Press_At (L : Positive; O : Opening; Tilt, Azimuth, X, Y : Real; Made : out Boolean;
                       Press : out Driver.Robot.Hand.Presses.Event; Lift : Real := 0.0; Slide : Vec3 := Zero3)
   is
      R       : constant Mat3 := Exp (Tilt * [Cos (Azimuth), Sin (Azimuth), 0.0]) * Pointing_Down (L, O);
      Landing : constant Vec3 := [X, Y, Lift + Contact_Sigma * Gaussian];
      T       : constant Vec3 := Landing - R * (Tips_True (L, O) + Slide);   --  the finger slid by Slide under the press
      Other   : constant Positive := 3 - L;
      Beside  : constant Vec3 := R * Tips_True (Other, O) + T;
   begin
      Made := Beside (3) > Landing (3);
      Press := (Tool     => (Pose                => (Rotation    => Exp (Turn_Sigma * [Gaussian, Gaussian, Gaussian]) * R,
                                                     Translation => T + Pose_Sigma * [Gaussian, Gaussian, Gaussian]),
                             Position_Covariance => (Pose_Sigma ** 2) * Identity3,
                             Rotation_Covariance => (Turn_Sigma ** 2) * Identity3),
                Arm      => <>,
                Approach => (Unit_Vector => Transpose (R) * Down, Sigma => Turn_Sigma),
                Closer   => Driver.Robot.Hand.Presses.Reading_Holders.To_Holder ([1 => (if O = Open then 1.0 else 0.0)]),
                Beat     => 0);
   end Press_At;

   procedure Pressed_At_Both_Openings (Mislead : Boolean) is
      B     : Book;
      Moved : Boolean;
      Aimed : array (1 .. 2, Opening) of Natural := [others => [others => 0]];
      --  The true directions into the table of the presses aimed at each tip.
      Pressed_Along : array (1 .. 2, Opening) of Vec3 := [others => [others => Zero3]];
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 31);
      Set_Sights (B, Sights);
      --  The surface is the table the arm's own eye saw: a press is made on
      --  no other.
      Set_Frame (B, Table, No_Pose'Access, Moved);
      for L in 1 .. 2 loop
         for O in Opening loop
            for K in 0 .. 11 loop
               declare
                  Made  : Boolean;
                  Press : Driver.Robot.Hand.Presses.Event;
               begin
                  Press_At (L, O, 0.3 * Real (K mod 3), Ada.Numerics.Pi * Real (K mod 4) / 2.0,
                            0.4 + 0.02 * Real (K), 0.1 + 0.03 * Real (L) + 0.01 * Real (K mod 5), Made, Press);
                  if Made then
                     Pressed_Along (L, O) := Pressed_Along (L, O) + Press.Approach.Unit_Vector;
                     if Mislead and then K mod 3 = 1 then
                        --  The tool barely moved before the block: no direction.
                        Press.Approach := (Unit_Vector => Zero3, Sigma => Real'Last);
                     elsif Mislead and then K mod 5 = 4 then
                        --  It came in sideways, along the other lobe's line of sight.
                        Press.Approach := (Unit_Vector => Line (3 - L, O), Sigma => Turn_Sigma);
                     end if;
                     Add (B, Press, O);
                     Aimed (L, O) := Aimed (L, O) + 1;
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      Check (Pressed (B) = Aimed (1, Open) + Aimed (2, Open) + Aimed (1, Closed_Empty) + Aimed (2, Closed_Empty),
             "presses were lost");
      for L in 1 .. 2 loop
         for O in Opening loop
            declare
               T : constant Point_Estimate := Tip (B, L, O);
            begin
               Check (Known (T), "lobe" & L'Image & " " & O'Image & ": no tip from" & Aimed (L, O)'Image & " presses");
               Check (Confirmed (B, L, O), "lobe" & L'Image & " " & O'Image & ": a tip " & Aimed (L, O)'Image
                      & " presses made is not confirmed");
               if Known (T) then
                  declare
                     D : constant Vec3 := T.Mean - Tips_True (L, O);
                     Mahalanobis : constant Real := D * (Inverse (T.Covariance) * D);
                  begin
                     --  Chi square with three degrees of freedom, beyond 3 sigma at about 14.2.
                     Check (Mahalanobis < 14.2, "lobe" & L'Image & " " & O'Image & " tip off by"
                            & Real'Image (abs D) & ", Mahalanobis" & Real'Image (Mahalanobis));
                  end;
               end if;
               --  Each lobe got its own presses, less one that disagreed by chance.
               Check (Agreeing (B, L, O) <= Aimed (L, O) and then Agreeing (B, L, O) + 1 >= Aimed (L, O),
                      "lobe" & L'Image & " " & O'Image & " got" & Agreeing (B, L, O)'Image & " of its"
                      & Aimed (L, O)'Image & " presses");
               --  The direction is the presses' own, through the fitted table:
               --  off only by the table's tilt error, and by the share of the
               --  presses the tip does not rest on, each of which leans from the
               --  mean by the largest tilt made (0.6 rad).
               declare
                  Left_Out : constant Real := Real (Aimed (L, O) - Agreeing (B, L, O));
                  Allowed  : constant Real := 0.01 + 0.6 * Left_Out / Real'Max (1.0, Real (Agreeing (B, L, O)));
               begin
                  Check (Direction (B, L, O).Sigma < Real'Last
                         and then abs Cross (Direction (B, L, O).Unit_Vector, Unit (Pressed_Along (L, O))) < Allowed,
                         "lobe" & L'Image & " " & O'Image & ": the press direction is not the presses' own:"
                         & Real'Image (abs Cross (Direction (B, L, O).Unit_Vector, Unit (Pressed_Along (L, O))))
                         & " agreeing" & Agreeing (B, L, O)'Image & " of" & Aimed (L, O)'Image);
               end;
            end;
         end loop;
      end loop;
   end Pressed_At_Both_Openings;

   procedure Both_Openings is
   begin
      Pressed_At_Both_Openings (Mislead => False);
   end Both_Openings;

   --  Three presses of one lobe, each from another orientation: a lobe's tip
   --  and the table under it are four unknowns, and three presses leave them
   --  undetermined; with the table's prior the three presses fix the tip, and
   --  the table they fix is the prior's, corrected.
   procedure Table_Seen_Before is
   begin
      for Prior in Boolean loop
         declare
            B     : Book;
            Taken : Natural := 0;
            Moved : Boolean;
         begin
            Ada.Numerics.Float_Random.Reset (Gen, 17);
            Set_Sights (B, Sights);
            if Prior then
               Set_Frame (B, Table, No_Pose'Access, Moved);
               Check (Moved, "a table seen for the first time left the presses as they were");
            end if;
            for K in 0 .. 11 loop
               declare
                  Made  : Boolean;
                  Press : Driver.Robot.Hand.Presses.Event;
               begin
                  Press_At (1, Open, 0.3 * Real (K mod 3), Ada.Numerics.Pi * Real (K mod 4) / 2.0, 0.4 + 0.02 * Real (K),
                            0.1 + 0.03 + 0.01 * Real (K mod 5), Made, Press);
                  if Made and then Taken < 3 then
                     Add (B, Press, Open);
                     Taken := Taken + 1;
                  end if;
               end;
            end loop;
            Check (Taken = 3, "the rig made only" & Taken'Image & " presses of the lobe");
            if Prior then
               declare
                  T : constant Point_Estimate := Tip (B, 1, Open);
                  D : constant Vec3 := T.Mean - Tips_True (1, Open);
               begin
                  Check (Known (T), "three presses on a table seen before gave no tip: kept" & Pressed (B)'Image
                         & ", agreeing" & Agreeing (B, 1, Open)'Image);
                  Check (Driver.Geometry.Known (Surface (B)), "three presses on a table seen before gave no surface");
                  if Known (T) then
                     Check (Sqrt (D * (Inverse (T.Covariance) * D)) <= Threshold (Vector_Gate (3)),
                            "the tip from three presses on a table seen before is off by" & Real'Image (abs D)
                            & " beyond its sigma");
                  end if;
               end;
            else
               Check (not Known (Tip (B, 1, Open)), "three presses on a table nothing measured gave a tip");
            end if;
            --  A table other than the one given refits the presses kept.
            if Prior then
               declare
                  Higher : Driver.Geometry.Plane_Estimate := Table;
                  Before : constant Point_Estimate := Tip (B, 1, Open);
               begin
                  Higher.Centre := [0.0, 0.0, 3.0e-4];
                  Set_Frame (B, Higher, No_Pose'Access, Moved);
                  Check (Moved and then Known (Tip (B, 1, Open)) and then Tip (B, 1, Open).Mean /= Before.Mean,
                         "a table other than the one the presses were fitted with left the tip where it was");
                  Set_Frame (B, Higher, No_Pose'Access, Moved);
                  Check (not Moved, "the table the presses were fitted with moved them");
               end;
            end if;
         end;
      end loop;
   end Table_Seen_Before;

   procedure Directions_Unknown_Or_Misleading is
   begin
      Pressed_At_Both_Openings (Mislead => True);
   end Directions_Unknown_Or_Misleading;

   procedure Stalls_Neither_Agree_Nor_Confirm is
      --  The book as A16's first hand filled it: a straight press the tip
      --  stopped, then two tilted presses the arm stopped on itself, short
      --  of the table. The tip is the first press's, provisional; the
      --  presses that stopped short are kept and are not ones it rests on;
      --  a later press from another pose that the tip stopped confirms it.
      B     : Book;
      Moved : Boolean;
      Made  : Boolean;
      Press : Driver.Robot.Hand.Presses.Event;
      First : Point_Estimate;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 33);
      Set_Sights (B, Sights);
      Set_Frame (B, Table, No_Pose'Access, Moved);
      Press_At (1, Open, 0.0, 0.0, 0.45, 0.10, Made, Press);
      Press.Beat := 100;
      Add (B, Press, Open);
      First := Tip (B, 1, Open);
      Check (Made and then Known (First) and then not Confirmed (B, 1, Open) and then Latest_Agrees (B),
             "one press the tip stopped did not give a provisional tip");
      Check (Beat (B, 1, Open) = 100, "the beat of the press the tip rests on is" & Beat (B, 1, Open)'Image);
      Press_At (1, Open, 0.3, 1.0, 0.40, 0.15, Made, Press, Lift => 0.06);
      Press.Beat := 200;
      Add (B, Press, Open);
      Check (Made and then not Latest_Agrees (B) and then Agreeing (B, 1, Open) = 1 and then not Confirmed (B, 1, Open),
             "a press that stopped 60 mm short is one the tip rests on");
      Press_At (1, Open, 0.3, 4.0, 0.50, 0.12, Made, Press, Lift => 0.03);
      Press.Beat := 300;
      Add (B, Press, Open);
      Check (Made and then not Latest_Agrees (B) and then Agreeing (B, 1, Open) = 1 and then Pressed (B) = 3,
             "a press that stopped 30 mm short is one the tip rests on");
      Check (Tip (B, 1, Open).Mean = First.Mean,
             "presses that stopped short moved the tip by" & Real'Image (abs (Tip (B, 1, Open).Mean - First.Mean)));
      Check (Beat (B, 1, Open) = 100, "the beat of the press the tip rests on, with two stops after it, is"
             & Beat (B, 1, Open)'Image);
      Press_At (1, Open, 0.6, 2.0, 0.42, 0.13, Made, Press);
      Press.Beat := 400;
      Add (B, Press, Open);
      Check (Made and then Latest_Agrees (B) and then Agreeing (B, 1, Open) = 2 and then Confirmed (B, 1, Open),
             "a second press from another pose that the tip stopped did not confirm it:" & Agreeing (B, 1, Open)'Image);
      Check (Beat (B, 1, Open) = 100 or else Beat (B, 1, Open) = 400,
             "the beat of the tip two presses rest on is" & Beat (B, 1, Open)'Image);
   end Stalls_Neither_Agree_Nor_Confirm;

   procedure Spread_Widens_Across_The_Line is
      --  A tip lies on its line of sight at the distance a press put it; the
      --  lobe's tip region lies within a spread of that line, so across the
      --  line the tip's covariance grows by that spread at that distance, and
      --  along the line it does not.
      Spread : constant Real := 0.05;
      Narrow, Wide : Point_Estimate;
      Distance_Of_Wide : Real := 0.0;
   begin
      for Pass in 1 .. 2 loop
         declare
            B     : Book;
            Moved : Boolean;
            Made  : Boolean;
            Press : Driver.Robot.Hand.Presses.Event;
         begin
            Ada.Numerics.Float_Random.Reset (Gen, 41);
            Set_Sights (B, Sights (if Pass = 1 then 0.0 else Spread));
            Set_Frame (B, Table, No_Pose'Access, Moved);
            Press_At (1, Open, 0.0, 0.0, 0.45, 0.10, Made, Press);
            Add (B, Press, Open);
            if Pass = 1 then
               Narrow := Tip (B, 1, Open);
            else
               Wide := Tip (B, 1, Open);
               Distance_Of_Wide := Distance (B, 1, Open).Value;
            end if;
         end;
      end loop;
      declare
         U      : constant Vec3 := Line (1, Open);
         V      : constant Vec3 := Unit (Cross (U, [0.0, 0.0, 1.0]));
         Along  : constant Real := U * (Wide.Covariance * U) - U * (Narrow.Covariance * U);
         Across : constant Real := V * (Wide.Covariance * V) - V * (Narrow.Covariance * V);
      begin
         Check (Known (Narrow) and then Known (Wide) and then Narrow.Mean = Wide.Mean, "the spread moved the tip");
         Check (abs Along < 1.0e-12, "the spread widened the tip along its line of sight by" & Real'Image (Along));
         Check (abs (Across - (Distance_Of_Wide * Spread) ** 2) < 1.0e-9 * (Distance_Of_Wide * Spread) ** 2 + 1.0e-15,
                "the spread widened the tip across its line by" & Real'Image (Across) & " against"
                & Real'Image ((Distance_Of_Wide * Spread) ** 2));
      end;
   end Spread_Widens_Across_The_Line;


   procedure Slides_Kept_With_Their_Presses is
      --  Every press carries how far each lobe's finger stood from where the
      --  closer's reading puts it; they come back in the order the presses were made,
      --  with the lobe the press went to marked.
      B     : Book;
      Moved : Boolean;
      Made  : Boolean;
      Press : Driver.Robot.Hand.Presses.Event;
      Slid_20 : constant Slid := (Known => True, Pixels => 20.0, Pixels_Sigma => 0.5, Fraction => 0.1,
                                  Fraction_Sigma => 0.0025);
      Slid_Nil : constant Slid := (Known => True, Pixels => 0.2, Pixels_Sigma => 0.4, Fraction => 0.001,
                                   Fraction_Sigma => 0.002);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 57);
      Set_Sights (B, Sights);
      Set_Frame (B, Table, No_Pose'Access, Moved);
      Press_At (1, Open, 0.0, 0.0, 0.45, 0.10, Made, Press);
      Press.Beat := 100;
      Add (B, Press, Open, [1 => Slid_20, 2 => Slid_Nil]);
      Press_At (1, Open, 0.6, 2.0, 0.42, 0.13, Made, Press);
      Press.Beat := 200;
      Add (B, Press, Open, [1 => Slid_20, 2 => (others => <>)]);
      Press_At (2, Closed_Empty, 0.0, 0.0, 0.40, -0.10, Made, Press);
      Press.Beat := 300;
      Add (B, Press, Closed_Empty);
      declare
         First  : constant Press_Slides := Slides_Of (B, 1, Open);
         Second : constant Press_Slides := Slides_Of (B, 2, Open);
         Closed : constant Press_Slides := Slides_Of (B, 2, Closed_Empty);
      begin
         Check (First'Length = 2 and then First (1).Beat = 100 and then First (2).Beat = 200,
                "the slides at the open opening are of" & First'Length'Image & " presses");
         Check (First (1).Contact and then First (2).Contact, "the first lobe pressed at the open opening twice");
         Check (First (1).Slid = Slid_20 and then First (2).Slid = Slid_20, "the first lobe's slides were not kept");
         Check (not Second (1).Contact and then Second (1).Slid = Slid_Nil, "the other lobe's slide is not its own");
         Check (not Second (2).Slid.Known, "a slide that was not measured is known");
         Check (Closed'Length = 1 and then Closed (1).Beat = 300 and then not Closed (1).Slid.Known,
                "a press made without slides has one");
      end;
   end Slides_Kept_With_Their_Presses;

   --  The share of its travel between the openings each lobe's finger slides inward by under a press.
   Slid_By : constant array (1 .. 2) of Real := [0.12, 0.28];

   function Travel (L : Positive) return Vec3 is (Tips_True (L, Closed_Empty) - Tips_True (L, Open));

   --  What the eye measured of the slide under a press of lobe L: its own, and nothing of the other's.
   function Measured_Slide (L : Positive; Known_Slide : Boolean := True) return Slid_Row is
     ([for Which in 1 .. 2 => (if Which = L and then Known_Slide
                               then Slid'(Known => True, Pixels => 0.0, Pixels_Sigma => 1.0, Fraction => Slid_By (L),
                                         Fraction_Sigma => 0.01)
                               else Slid'(others => <>))]);

   --  Presses at both openings, or at the open one only, the fingers sliding by their shares under each, the
   --  slides measured or not.
   function Book_With_Slides (Openings : Boolean; Measured : Boolean) return Book is
      B     : Book;
      Moved : Boolean;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 53);
      Set_Sights (B, Sights);
      Set_Frame (B, Table, No_Pose'Access, Moved);
      for L in 1 .. 2 loop
         for O in Opening loop
            if O = Open or else Openings then
               for K in 0 .. 11 loop
                  declare
                     Made  : Boolean;
                     Press : Driver.Robot.Hand.Presses.Event;
                  begin
                     Press_At (L, O, 0.3 * Real (K mod 3), Ada.Numerics.Pi * Real (K mod 4) / 2.0,
                               0.4 + 0.02 * Real (K), 0.1 + 0.03 * Real (L) + 0.01 * Real (K mod 5), Made, Press,
                               Slide => Slid_By (L) * Travel (L));
                     if Made then
                        Add (B, Press, O, Measured_Slide (L, Measured));
                     end if;
                  end;
               end loop;
            end if;
         end loop;
      end loop;
      return B;
   end Book_With_Slides;

   procedure Free_Tips_From_Slides is
      --  Two fingers slide inward under every press by 12 and 28 per cent of their travel (3 to 9 mm). The
      --  loaded tips are those of the fingers as they stood under the presses, off the free fingers by that;
      --  the free tips, from the presses taken with the tips they slid to, are the fingers as they stand free,
      --  and confirmed. They are unknown until both openings of a lobe have a tip and the presses have their
      --  slides measured: pressed at one opening only, or with nothing measured of the slides, there is none.
      Both : constant Book := Book_With_Slides (Openings => True, Measured => True);
   begin
      for L in 1 .. 2 loop
         for O in Opening loop
            declare
               Free_Tip   : constant Point_Estimate := Tip (Both, L, O, Free);
               Loaded_Tip : constant Point_Estimate := Tip (Both, L, O, Loaded);
            begin
               Check (Known (Free_Tip), "lobe" & L'Image & " " & O'Image & ": no free tip");
               if Known (Free_Tip) and then Known (Loaded_Tip) then
                  declare
                     D : constant Vec3 := Free_Tip.Mean - Tips_True (L, O);
                  begin
                     Check (abs D < 1.0e-3, "lobe" & L'Image & " " & O'Image & ": the free tip is off by" & Real'Image (abs D));
                     Check (abs (Loaded_Tip.Mean - Tips_True (L, O)) > 2.0 * abs D,
                            "lobe" & L'Image & " " & O'Image & ": the loaded tip is off by" & Real'Image (abs (Loaded_Tip.Mean - Tips_True (L, O)))
                            & ", the free tip by" & Real'Image (abs D));
                  end;
                  Check (Confirmed (Both, L, O, Free), "lobe" & L'Image & " " & O'Image & ": the free tip is not confirmed");
                  Check (Beat (Both, L, O, Free) > 0 or else Pressed (Both) > 0, "the free tip rests on no press");
               end if;
            end;
         end loop;
      end loop;
      declare
         One_Opening : constant Book := Book_With_Slides (Openings => False, Measured => True);
         Unmeasured  : constant Book := Book_With_Slides (Openings => True, Measured => False);
      begin
         Check (Known (Tip (One_Opening, 1, Open)) and then not Known (Tip (One_Opening, 1, Open, Free)),
                "a lobe pressed at one opening only has a free tip, or no loaded one");
         Check (Known (Tip (Unmeasured, 1, Open)) and then not Known (Tip (Unmeasured, 1, Open, Free)),
                "presses whose slides nothing measured give a free tip, or no loaded one");
      end;
   end Free_Tips_From_Slides;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.tips.free", "a finger that slid under its presses has its free tip off, or unknown when "
                             & "it can be told", Free_Tips_From_Slides'Access);
      Driver.Tests.Register ("hand.tips.slides", "the slides of a press are not kept with it, or not given by lobe "
                             & "and opening", Slides_Kept_With_Their_Presses'Access);
      Driver.Tests.Register ("hand.tips.book", "presses go to the wrong lobe, or the tips they give are off",
                             Both_Openings'Access);
      Driver.Tests.Register ("hand.tips.reassign",
                             "a press whose approach is unknown or misleading stays with the wrong lobe or none",
                             Directions_Unknown_Or_Misleading'Access);
      Driver.Tests.Register ("hand.tips.spread",
                             "the lobe's tip region does not widen the tip across its line of sight, or does along it",
                             Spread_Widens_Across_The_Line'Access);
      Driver.Tests.Register ("hand.tips.stalls",
                             "presses that stopped short of the table move the tip, agree with it or confirm it",
                             Stalls_Neither_Agree_Nor_Confirm'Access);
      Driver.Tests.Register ("hand.tips.table", "a table the arm's own eye saw is not the prior of the presses on it, "
                             & "or a change of it leaves the tips where they were", Table_Seen_Before'Access);
   end Register;

end Driver.Robot.Hand.Tips.Tests;
