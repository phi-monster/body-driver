with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Robot.Hand.Tips.Tests is

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

   --  Two fingers in the tool frame, apart when open, nearly touching when
   --  closed; the eye looks at both from behind and above.
   type Tip_Table is array (1 .. 2, Opening) of Vec3;
   Tips_True : constant Tip_Table :=
     [1 => [Open => [0.13, 0.04, 0.0], Closed_Empty => [0.14, 0.008, 0.0]],
      2 => [Open => [0.13, -0.04, 0.0], Closed_Empty => [0.14, -0.008, 0.0]]];
   Eye : constant Vec3 := [0.05, 0.0, 0.05];

   Pose_Sigma    : constant Real := 5.0e-5;
   Turn_Sigma    : constant Real := 1.0e-4;
   Contact_Sigma : constant Real := 2.0e-4;

   function Line (L : Positive; O : Opening) return Vec3 is (Unit (Tips_True (L, O) - Eye));

   function Sights return Sight_Table is
     ([for L in 1 .. 2 =>
         [for O in Opening => (Known => True,
                               Ray   => (Origin    => (Mean => Eye, Covariance => 1.0e-10 * Identity3),
                                         Direction => (Unit_Vector => Line (L, O), Sigma => 1.0e-5)))]]);

   Down : constant Vec3 := [0.0, 0.0, -1.0];

   function Pointing_Down (L : Positive; O : Opening) return Mat3 is
      Axis : constant Vec3 := Cross (Line (L, O), Down);
   begin
      return Exp (Arcsin (abs Axis) * Unit (Axis));
   end Pointing_Down;

   --  A press aimed at one lobe on the table z = 0, made only when that lobe
   --  is really the one that touches; the tool reports its pose with the
   --  arm's noise, and was pressing straight down.
   procedure Press_At (L : Positive; O : Opening; Tilt, Azimuth, X, Y : Real; Made : out Boolean;
                       Press : out Driver.Robot.Hand.Presses.Event)
   is
      R       : constant Mat3 := Exp (Tilt * [Cos (Azimuth), Sin (Azimuth), 0.0]) * Pointing_Down (L, O);
      Landing : constant Vec3 := [X, Y, Contact_Sigma * Gaussian];
      T       : constant Vec3 := Landing - R * Tips_True (L, O);
      Other   : constant Positive := 3 - L;
      Beside  : constant Vec3 := R * Tips_True (Other, O) + T;
   begin
      Made := Beside (3) > Landing (3);
      Press := (Tool     => (Pose                => (Rotation    => Exp (Turn_Sigma * [Gaussian, Gaussian, Gaussian]) * R,
                                                     Translation => T + Pose_Sigma * [Gaussian, Gaussian, Gaussian]),
                             Position_Covariance => (Pose_Sigma ** 2) * Identity3,
                             Rotation_Covariance => (Turn_Sigma ** 2) * Identity3),
                Approach => (Unit_Vector => Transpose (R) * Down, Sigma => Turn_Sigma),
                Closer   => Driver.Robot.Hand.Presses.Reading_Holders.To_Holder ([1 => (if O = Open then 1.0 else 0.0)]),
                Beat     => 0);
   end Press_At;

   procedure Pressed_At_Both_Openings (Mislead : Boolean) is
      B     : Book;
      Aimed : array (1 .. 2, Opening) of Natural := [others => [others => 0]];
      --  The true directions into the table of the presses aimed at each tip.
      Pressed_Along : array (1 .. 2, Opening) of Vec3 := [others => [others => Zero3]];
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 31);
      Set_Sights (B, Sights);
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
               --  off only by the table's tilt error.
               Check (Direction (B, L, O).Sigma < Real'Last
                      and then abs Cross (Direction (B, L, O).Unit_Vector, Unit (Pressed_Along (L, O))) < 0.01,
                      "lobe" & L'Image & " " & O'Image & ": the press direction is not the presses' own");
            end;
         end loop;
      end loop;
   end Pressed_At_Both_Openings;

   procedure Both_Openings is
   begin
      Pressed_At_Both_Openings (Mislead => False);
   end Both_Openings;

   procedure Directions_Unknown_Or_Misleading is
   begin
      Pressed_At_Both_Openings (Mislead => True);
   end Directions_Unknown_Or_Misleading;


   procedure Register is
   begin
      Driver.Tests.Register ("hand.tips.book", "presses go to the wrong lobe, or the tips they give are off",
                             Both_Openings'Access);
      Driver.Tests.Register ("hand.tips.reassign",
                             "a press whose approach is unknown or misleading stays with the wrong lobe or none",
                             Directions_Unknown_Or_Misleading'Access);
   end Register;

end Driver.Robot.Hand.Tips.Tests;
