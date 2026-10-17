with Ada.Numerics.Long_Elementary_Functions;
with Driver.Robot.Regression;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Tests;

package body Driver.Robot.Surfaces.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Bytes.Offset;

   --  The eyes: pinholes over a picture of Width by Height, lens 1 of Focal pixels (the arm's eye), lens 2 of half
   --  that (a wider eye that stands in the scene).
   Width  : constant := 320;
   Height : constant := 240;
   Focal  : constant := 300.0;

   function Focal_Of (Lens : Positive) return Real is (if Lens = 1 then Focal else Focal / 2.0);

   function Lens_Ray (Lens : Positive; U, V : Real) return Vec3 is
     (Unit ([(U - Real (Width) / 2.0) / Focal_Of (Lens), (V - Real (Height) / 2.0) / Focal_Of (Lens), 1.0]));

   procedure Lens_Project (Lens : Positive; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      In_Front := P (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if In_Front then
         U := Focal_Of (Lens) * P (1) / P (3) + Real (Width) / 2.0;
         V := Focal_Of (Lens) * P (2) / P (3) + Real (Height) / 2.0;
      end if;
   end Lens_Project;

   --  An eye at Position looking at Target, its picture's down towards -z: x right, y down, z ahead.
   function Look (Position, Target : Vec3) return Rigid is
      Ahead : constant Vec3 := Unit (Target - Position);
      Right : constant Vec3 := Unit (Cross (Ahead, [0.0, 0.0, 1.0]));
      Down  : constant Vec3 := Cross (Ahead, Right);
   begin
      return (Rotation    => [[Right (1), Down (1), Ahead (1)],
                              [Right (2), Down (2), Ahead (2)],
                              [Right (3), Down (3), Ahead (3)]],
              Translation => Position);
   end Look;

   --  The scene: the table z = 0 and a box on it, both textured by smooth value noise at several scales (a texture
   --  the aligner can tell, as a cloth or a print is), the box a texture of its own.
   Box_Half : constant Vec3 := [0.04, 0.03, 0.025];
   Box_At   : constant Vec3 := [0.0, 0.0, 0.025];

   function Hash (X, Y, Seed : Integer) return Real is
      H : Long_Long_Integer := Long_Long_Integer (X) * 73_856_093 + Long_Long_Integer (Y) * 19_349_663
                               + Long_Long_Integer (Seed) * 83_492_791;
   begin
      H := (H mod 1_000_003 + 1_000_003) mod 1_000_003;
      H := (H * 48_271) mod 2_147_483_647;
      return Real (H mod 10_007) / 10_006.0;
   end Hash;

   function Noise (X, Y : Real; Seed : Integer) return Real is
      X0 : constant Integer := Integer (Real'Floor (X));
      Y0 : constant Integer := Integer (Real'Floor (Y));
      Fx : constant Real := X - Real (X0);
      Fy : constant Real := Y - Real (Y0);
      Sx : constant Real := Fx * Fx * (3.0 - 2.0 * Fx);
      Sy : constant Real := Fy * Fy * (3.0 - 2.0 * Fy);
      A  : constant Real := Hash (X0, Y0, Seed);
      B  : constant Real := Hash (X0 + 1, Y0, Seed);
      C  : constant Real := Hash (X0, Y0 + 1, Seed);
      D  : constant Real := Hash (X0 + 1, Y0 + 1, Seed);
   begin
      return A + (B - A) * Sx + (C - A) * Sy + (A - B - C + D) * Sx * Sy;
   end Noise;

   function Texture (X, Y : Real; Seed : Integer) return Real is
     (40.0 + 60.0 * Noise (X * 150.0, Y * 150.0, Seed) + 50.0 * Noise (X * 400.0, Y * 400.0, Seed + 1)
      + 40.0 * Noise (X * 900.0, Y * 900.0, Seed + 2));

   --  Where a line of sight from Origin along D first meets the scene: its distance, whether it meets the box, and
   --  the luma there.
   procedure Hit (Origin, D : Vec3; T : out Real; On_Box : out Boolean; Luma : out Real) is
      Near : Real := 0.0;
      Far  : Real := Real'Last;
      Face : Natural := 0;
   begin
      On_Box := False;
      Luma := 0.0;
      T := Real'Last;
      for A in 1 .. 3 loop
         if D (A) /= 0.0 then
            declare
               T1 : Real := (Box_At (A) - Box_Half (A) - Origin (A)) / D (A);
               T2 : Real := (Box_At (A) + Box_Half (A) - Origin (A)) / D (A);
            begin
               if T1 > T2 then
                  declare
                     Swap : constant Real := T1;
                  begin
                     T1 := T2;
                     T2 := Swap;
                  end;
               end if;
               if T1 > Near then
                  Near := T1;
                  Face := A;
               end if;
               Far := Real'Min (Far, T2);
            end;
         elsif abs (Origin (A) - Box_At (A)) > Box_Half (A) then
            Far := -1.0;
         end if;
      end loop;
      if Near <= Far and then Near > 0.0 then
         T := Near;
         On_Box := True;
         declare
            P : constant Vec3 := Origin + Near * D;
         begin
            Luma := (case Face is
                        when 3      => Texture (P (1), P (2), 7),
                        when 1      => Texture (P (2), P (3), 11),
                        when others => Texture (P (1), P (3), 13));
         end;
      elsif D (3) < 0.0 then
         T := -Origin (3) / D (3);
         declare
            P : constant Vec3 := Origin + T * D;
         begin
            Luma := Texture (P (1), P (2), 3);
         end;
      end if;
   end Hit;

   function Render (Pose : Rigid; On_Box : out Driver.Images.Mask; Lens : Positive := 1) return Driver.Images.Image is
      Bytes : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Width * Height));
   begin
      On_Box := Driver.Images.Create (Width, Height);
      for Row in 0 .. Height - 1 loop
         for Column in 0 .. Width - 1 loop
            declare
               D    : constant Vec3 := Times (Pose.Rotation, Lens_Ray (Lens, Real (Column) + 0.5, Real (Row) + 0.5));
               T    : Real;
               Box  : Boolean;
               Luma : Real;
               Byte : Driver.Bytes.Byte;
               K    : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * Width + Column));
            begin
               Hit (Pose.Translation, D, T, Box, Luma);
               Byte := Driver.Bytes.Byte (Natural'Min (255, Natural'Max (0, Natural (Real'Floor (Luma)))));
               Bytes (K + 1) := Byte;
               Bytes (K + 2) := Byte;
               Bytes (K + 3) := Byte;
               if Box then
                  Driver.Images.Include (On_Box, Column, Row);
               end if;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Width, Height, Bytes);
   end Render;

   procedure Judge
     (Now_Pose  : Rigid;
      Lens      : Positive;
      Queries   : Pixel_Array;
      Result    : Placed_Array;
      Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere : out Natural);

   --  The rate at which a point beyond the gate is one by chance: what the gate itself lets through (the lock-in's
   --  and the kinematics' own rule for a count of tests).
   Alarm : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);

   --  The eye now above and before the box, the sweep's keyframes a few centimetres to either side and above.
   procedure Place_A_Box is
      Target : constant Vec3 := [0.0, 0.0, 0.0];
      Now_Pose : constant Rigid := Look ([0.0, -0.18, 0.22], Target);
      Poses    : constant array (1 .. 4) of Rigid :=
        [Look ([0.07, -0.17, 0.22], Target), Look ([-0.07, -0.17, 0.22], Target),
         Look ([0.0, -0.13, 0.27], Target), Look ([0.01, -0.18, 0.22], Target)];
      Region  : Driver.Images.Mask;
      Ignored : Driver.Images.Mask;
      Now     : constant View := (Pose => Now_Pose, Picture => Driver.Alignment.Pyramid_Of (Render (Now_Pose, Region)),
                                  Lens => 1, Noise => 0.3);
      Views   : View_Array (1 .. 4);
      Step    : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
      Columns : constant Positive := (Width + Step - 1) / Step;
      Rows    : constant Positive := (Height + Step - 1) / Step;
      Queries : Pixel_Array (1 .. Columns * Rows);
      Counts  : Placing_Counts;
      Asked   : Natural := 0;
      Table   : constant Driver.Geometry.Plane_Estimate :=
        (Centre => [0.0, 0.0, 0.0], Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
         Offset_Sigma => 1.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);
      Result  : Placed_Array (Queries'Range);
      Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere : Natural;
      Kept, Interior, Facing_Up : Natural := 0;
   begin
      for K in Views'Range loop
         Views (K) := (Pose => Poses (K), Picture => Driver.Alignment.Pyramid_Of (Render (Poses (K), Ignored)),
                       Lens => 1, Noise => 0.3);
      end loop;
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            declare
               U : constant Natural := C * Step + Step / 2;
               V : constant Natural := R * Step + Step / 2;
               K : constant Positive := R * Columns + C + 1;
            begin
               if U < Width and then V < Height and then Driver.Images.Contains (Region, U, V) then
                  Queries (K) := (U => Real (U) + 0.5, V => Real (V) + 0.5);
                  Asked := Asked + 1;
               else
                  Queries (K) := (U => -1.0, V => -1.0);
               end if;
            end;
         end loop;
      end loop;
      Check (Asked > 0, "the box covers no query of the grid");
      Place (Now, Views, Queries, Columns, Table, Lens_Ray'Access, Lens_Project'Access, Result, Counts);
      Judge (Now_Pose, 1, Queries, Result, Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere);
      --  What In_Region keeps of it: the points above the table, the box's own.
      for R of Result loop
         if R.Found and then not R.On_Table then
            Kept := Kept + 1;
            if abs R.Normal > 0.0 then
               Interior := Interior + 1;
               if R.Normal (3) > 0.0 then
                  Facing_Up := Facing_Up + 1;
               end if;
            end if;
         end if;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "surfaces test:" & Asked'Image & " queries on the box," & Found'Image
                       & " placed," & Kept'Image & " of them above the table," & Off'Image
                       & " off every surface beyond Z," & Told_Wrong'Image & " told on or off the table wrongly,"
                       & Shown_Elsewhere'Image & " on a surface other than the one their pixel shows first,"
                       & Interior'Image & " with a normal," & Facing_Up'Image & " of them facing up");
      Check (Kept * 2 >= Asked, "only" & Kept'Image & " of" & Asked'Image & " queries on a textured box were placed on it");
      Check (not Driver.Robot.Regression.Count_Significant (Off, Natural'Max (1, Found), Alarm),
             Off'Image & " of" & Found'Image & " placed points lie beyond Z of their own uncertainty from every surface, "
             & "more than the gate passes by chance");
      Check (not Driver.Robot.Regression.Count_Significant (Told_Wrong, Natural'Max (1, Found), Alarm),
             Told_Wrong'Image & " of" & Found'Image & " points told on the table where they are not, or not where they are, "
             & "more than the gate passes by chance");
   end Place_A_Box;

   --  How far X, with its covariance, is from the scene's surfaces, as the significance of the nearest: from a plane
   --  (the table's, or a face of the box it lies over) by its distance along the plane's normal against its own
   --  uncertainty along that normal, one degree of freedom; from the box's edges and corners by its distance to their
   --  nearest point, three. On_Box: the nearest is the box.
   function Off_Scene (X : Vec3; Cov : Mat3; On_Box : out Boolean) return Real is
      function Along (N : Vec3; D : Real) return Real is
        (Driver.Distributions.Chi_Square_Deviate (D ** 2 / Real'Max (Real'Model_Small, N * Times (Cov, N)), 1));
      Best : Real := Along ([0.0, 0.0, 1.0], X (3));
   begin
      On_Box := False;
      for A in 1 .. 3 loop
         for Sign of Real_Array'[-1.0, 1.0] loop
            declare
               Plane : constant Real := Box_At (A) + Sign * Box_Half (A);
               Over  : Boolean := True;   --  X lies over the face, within its extent along the other two axes
               N     : Vec3 := Zero3;
            begin
               for B in 1 .. 3 loop
                  if B /= A then
                     Over := Over and then abs (X (B) - Box_At (B)) <= Box_Half (B);
                  end if;
               end loop;
               N (A) := 1.0;
               if Over and then Along (N, X (A) - Plane) < Best then
                  Best := Along (N, X (A) - Plane);
                  On_Box := True;
               end if;
            end;
         end loop;
      end loop;
      declare
         Clamped : Vec3;
      begin
         for A in 1 .. 3 loop
            Clamped (A) := Real'Max (Box_At (A) - Box_Half (A), Real'Min (Box_At (A) + Box_Half (A), X (A)));
         end loop;
         declare
            E   : constant Vec3 := X - Clamped;
            Chi : constant Real := Driver.Distributions.Chi_Square_Deviate (E * Times (Inverse (Cov), E), 3);
         begin
            if Chi < Best then
               Best := Chi;
               On_Box := True;
            end if;
         end;
      end;
      return Best;
   end Off_Scene;

   --  Every query of a grid placed from Elsewhere and judged against the scene: those placed beyond Z of their own
   --  uncertainty from every surface of the scene (Off: no point of it), those told on the table (or not) when the
   --  surface they lie on is the box (or the table), and those placed on a surface other than the one their pixel shows
   --  first along its line of sight (Shown_Elsewhere: at the edge of the box, a patch that takes in the box and what
   --  lies behind it is borne out by the views at either depth; such a point is a point of the scene all the same).
   procedure Judge
     (Now_Pose  : Rigid;
      Lens      : Positive;
      Queries   : Pixel_Array;
      Result    : Placed_Array;
      Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere : out Natural)
   is
      Gate : constant Real := Driver.Conventions.Z;
   begin
      Found := 0;
      Off := 0;
      Told_Wrong := 0;
      On_Box := 0;
      On_Table := 0;
      Shown_Elsewhere := 0;
      for K in Result'Range loop
         if Result (K).Found then
            Found := Found + 1;
            declare
               D     : constant Vec3 := Times (Now_Pose.Rotation, Lens_Ray (Lens, Queries (K).U, Queries (K).V));
               T     : Real;
               Box   : Boolean;
               Luma  : Real;
               X     : constant Vec3 := Result (K).Point.Mean;
               Lies_On_Box : Boolean;
               Dev   : constant Real := Off_Scene (X, Result (K).Point.Covariance, Lies_On_Box);
            begin
               Hit (Now_Pose.Translation, D, T, Box, Luma);
               if Dev > Gate then
                  Off := Off + 1;
                  Driver.Log.Line (Driver.Log.Robot, "surfaces test: query" & K'Image & " off every surface, at"
                                   & Real'Image (Dev) & " deviations");
               end if;
               declare
                  Truth : constant Vec3 := Now_Pose.Translation + T * D;
                  Et    : constant Vec3 := X - Truth;
                  Chi_T : constant Real := Et * Times (Inverse (Result (K).Point.Covariance), Et);
               begin
                  if Driver.Distributions.Chi_Square_Deviate (Chi_T, 3) > Gate then
                     Shown_Elsewhere := Shown_Elsewhere + 1;
                  end if;
               end;
               if Lies_On_Box then
                  On_Box := On_Box + 1;
               else
                  On_Table := On_Table + 1;
               end if;
               --  On the table and told so; on the box and not, but at its foot, where the box's side meets the table
               --  within the point's own uncertainty and either is right.
               declare
                  Sigma  : constant Real := Sqrt (Real'Max (0.0, Result (K).Point.Covariance (3, 3)));
                  Either : constant Boolean := Lies_On_Box and then X (3) <= Gate * Sigma;
               begin
                  if not Either and then Result (K).On_Table = Lies_On_Box then
                     Told_Wrong := Told_Wrong + 1;
                  end if;
               end;
            end;
         end if;
      end loop;
   end Judge;

   function Grid (Region : Driver.Images.Mask; Whole : Boolean; Columns, Rows : Positive; Asked : out Natural)
     return Pixel_Array
   is
      Step : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
   begin
      Asked := 0;
      return Queries : Pixel_Array (1 .. Columns * Rows) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               declare
                  U : constant Natural := C * Step + Step / 2;
                  V : constant Natural := R * Step + Step / 2;
                  K : constant Positive := R * Columns + C + 1;
               begin
                  if U < Width and then V < Height and then (Whole or else Driver.Images.Contains (Region, U, V)) then
                     Queries (K) := (U => Real (U) + 0.5, V => Real (V) + 0.5);
                     Asked := Asked + 1;
                  else
                     Queries (K) := (U => -1.0, V => -1.0);
                  end if;
               end;
            end loop;
         end loop;
      end return;
   end Grid;

   Scene_Table : constant Driver.Geometry.Plane_Estimate :=
     (Centre => [0.0, 0.0, 0.0], Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
      Offset_Sigma => 1.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);

   --  The whole picture of the eye now, the table and the box: what the scene's measurement asks.
   procedure Place_The_Scene is
      Target   : constant Vec3 := [0.0, 0.0, 0.0];
      Now_Pose : constant Rigid := Look ([0.0, -0.18, 0.22], Target);
      Poses    : constant array (1 .. 4) of Rigid :=
        [Look ([0.07, -0.17, 0.22], Target), Look ([-0.07, -0.17, 0.22], Target),
         Look ([0.0, -0.13, 0.27], Target), Look ([0.01, -0.18, 0.22], Target)];
      Ignored  : Driver.Images.Mask;
      Now      : constant View := (Pose => Now_Pose, Picture => Driver.Alignment.Pyramid_Of (Render (Now_Pose, Ignored)),
                                   Lens => 1, Noise => 0.3);
      Views    : View_Array (1 .. 4);
      Step     : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
      Columns  : constant Positive := (Width + Step - 1) / Step;
      Rows     : constant Positive := (Height + Step - 1) / Step;
      Asked    : Natural;
      Queries  : constant Pixel_Array := Grid (Ignored, True, Columns, Rows, Asked);
      Result   : Placed_Array (Queries'Range);
      Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere : Natural;
      Seeable  : Natural := 0;
      Counts   : Placing_Counts;
   begin
      for K in Views'Range loop
         Views (K) := (Pose => Poses (K), Picture => Driver.Alignment.Pyramid_Of (Render (Poses (K), Ignored)),
                       Lens => 1, Noise => 0.3);
      end loop;
      Place (Now, Views, Queries, Columns, Scene_Table, Lens_Ray'Access, Lens_Project'Access, Result, Counts);
      Judge (Now_Pose, 1, Queries, Result, Found, Off, Told_Wrong, On_Box, On_Table, Shown_Elsewhere);
      --  What can be placed at all: a query whose point two other views see, in their pictures and not hidden behind
      --  the box from them (their own line of sight to it meets the scene there first).
      for K in Queries'Range loop
         if Queries (K).U >= 0.0 then
            declare
               D    : constant Vec3 := Times (Now_Pose.Rotation, Lens_Ray (1, Queries (K).U, Queries (K).V));
               T    : Real;
               Box  : Boolean;
               Luma : Real;
               Seen : Natural := 0;
            begin
               Hit (Now_Pose.Translation, D, T, Box, Luma);
               if T > 0.0 then
                  declare
                     X : constant Vec3 := Now_Pose.Translation + T * D;
                  begin
                     for P of Poses loop
                        declare
                           U, W     : Real;
                           In_Front : Boolean;
                           To_X     : constant Vec3 := X - P.Translation;
                           T2       : Real;
                           Box2     : Boolean;
                           Luma2    : Real;
                        begin
                           Lens_Project (1, Times (Transposed (P.Rotation), To_X), U, W, In_Front);
                           Hit (P.Translation, Unit (To_X), T2, Box2, Luma2);
                           if In_Front and then U >= 0.0 and then W >= 0.0 and then U <= Real (Width)
                             and then W <= Real (Height) and then T2 > 0.0
                             and then abs (P.Translation + T2 * Unit (To_X) - X) <= Driver.Conventions.Z * 1.0e-4
                           then
                              Seen := Seen + 1;
                           end if;
                        end;
                     end loop;
                  end;
               end if;
               if Seen >= 2 then
                  Seeable := Seeable + 1;
               end if;
            end;
         end if;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "surfaces test (scene):" & Asked'Image & " queries," & Seeable'Image
                       & " of them seen by two other views," & Found'Image
                       & " placed (" & On_Box'Image & " on the box," & On_Table'Image & " on the table),"
                       & Off'Image & " off every surface beyond Z," & Told_Wrong'Image & " told on or off the table wrongly,"
                       & Shown_Elsewhere'Image & " on a surface other than the one their pixel shows first");
      Check (Found * 2 >= Seeable, "only" & Found'Image & " of the" & Seeable'Image
             & " queries of the scene that two other views see were placed");
      Check (On_Table * 2 >= Found - On_Box, "the table's points were not placed");
      Check (not Driver.Robot.Regression.Count_Significant (Off, Natural'Max (1, Found), Alarm),
             Off'Image & " of" & Found'Image & " placed points lie beyond Z of their own uncertainty from every surface, "
             & "more than the gate passes by chance");
      Check (not Driver.Robot.Regression.Count_Significant (Told_Wrong, Natural'Max (1, Found), Alarm),
             Told_Wrong'Image & " of" & Found'Image & " points told on the table where they are not, or not where they are, "
             & "more than the gate passes by chance");
   end Place_The_Scene;

   procedure Register is
   begin
      Register ("robot.surfaces.scene", "the whole picture of a table and a box, four keyframes given, is placed "
                & "with fewer than half the queries two other views see, the table's points missing, any off by more than Z of their own "
                & "uncertainty, or told on the table where they are not", Place_The_Scene'Access);
      Register ("robot.surfaces.box", "a textured box on a table, its eye's picture now and four keyframes of the arm's "
                & "sweep given, is placed above the table with fewer than half its queries, any point off every surface "
                & "by more than Z of its own uncertainty, or told on the table where it is not", Place_A_Box'Access);
   end Register;

end Driver.Robot.Surfaces.Tests;
