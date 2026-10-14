with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Tests;

package body Driver.Robot.Surfaces.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Bytes.Offset;

   --  The eye: a pinhole of Focal pixels over a picture of Width by Height.
   Width  : constant := 320;
   Height : constant := 240;
   Focal  : constant := 300.0;

   function Lens_Ray (U, V : Real) return Vec3 is
     (Unit ([(U - Real (Width) / 2.0) / Focal, (V - Real (Height) / 2.0) / Focal, 1.0]));

   procedure Lens_Project (P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      In_Front := P (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if In_Front then
         U := Focal * P (1) / P (3) + Real (Width) / 2.0;
         V := Focal * P (2) / P (3) + Real (Height) / 2.0;
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

   function Render (Pose : Rigid; On_Box : out Driver.Images.Mask) return Driver.Images.Image is
      Bytes : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Width * Height));
   begin
      On_Box := Driver.Images.Create (Width, Height);
      for Row in 0 .. Height - 1 loop
         for Column in 0 .. Width - 1 loop
            declare
               D    : constant Vec3 := Times (Pose.Rotation, Lens_Ray (Real (Column) + 0.5, Real (Row) + 0.5));
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

   --  The eye now above and before the box, the sweep's keyframes a few centimetres to either side and above.
   procedure Place_A_Box is
      Target : constant Vec3 := [0.0, 0.0, 0.0];
      Now_Pose : constant Rigid := Look ([0.0, -0.18, 0.22], Target);
      Poses    : constant array (1 .. 4) of Rigid :=
        [Look ([0.07, -0.17, 0.22], Target), Look ([-0.07, -0.17, 0.22], Target),
         Look ([0.0, -0.13, 0.27], Target), Look ([0.01, -0.18, 0.22], Target)];
      Region  : Driver.Images.Mask;
      Ignored : Driver.Images.Mask;
      Now     : constant View := (Pose => Now_Pose, Picture => Driver.Alignment.Pyramid_Of (Render (Now_Pose, Region)));
      Views   : View_Array (1 .. 4);
      Step    : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
      Columns : constant Positive := (Width + Step - 1) / Step;
      Rows    : constant Positive := (Height + Step - 1) / Step;
      Queries : Pixel_Array (1 .. Columns * Rows);
      Asked   : Natural := 0;
      Table   : constant Driver.Geometry.Plane_Estimate :=
        (Centre => [0.0, 0.0, 0.0], Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
         Offset_Sigma => 1.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);
      Result  : Placed_Array (Queries'Range);
      Found, Off, On_Table, Facing_Up : Natural := 0;
      Interior : Natural := 0;
   begin
      for K in Views'Range loop
         Views (K) := (Pose => Poses (K), Picture => Driver.Alignment.Pyramid_Of (Render (Poses (K), Ignored)));
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
      Place (Now, Views, Queries, Columns, Table, Lens_Ray'Access, Lens_Project'Access, 0.3, Result);
      for K in Result'Range loop
         if Result (K).Found then
            Found := Found + 1;
            declare
               D    : constant Vec3 := Times (Now_Pose.Rotation, Lens_Ray (Queries (K).U, Queries (K).V));
               T    : Real;
               Box  : Boolean;
               Luma : Real;
            begin
               Hit (Now_Pose.Translation, D, T, Box, Luma);
               if Box then
                  declare
                     Truth : constant Vec3 := Now_Pose.Translation + T * D;
                     E     : constant Vec3 := Result (K).Point.Mean - Truth;
                     Chi   : constant Real := E * Times (Inverse (Result (K).Point.Covariance), E);
                  begin
                     if Driver.Distributions.Chi_Square_Deviate (Chi, 3) > Driver.Conventions.Z then
                        Off := Off + 1;
                        Driver.Log.Line (Driver.Log.Robot, "surfaces test: query" & K'Image & " off by"
                                         & Real'Image (abs E) & " m, chi square" & Chi'Image);
                     end if;
                     if Truth (3) >= 2.0 * Box_Half (3) - 1.0e-9 and then abs Result (K).Normal > 0.0 then
                        Interior := Interior + 1;
                        if Result (K).Normal (3) > 0.0 then
                           Facing_Up := Facing_Up + 1;
                        end if;
                     end if;
                  end;
               else
                  On_Table := On_Table + 1;
               end if;
            end;
         end if;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "surfaces test:" & Asked'Image & " queries on the box," & Found'Image
                       & " placed," & Off'Image & " off beyond Z," & On_Table'Image & " placed on the table,"
                       & Interior'Image & " on its top with a normal," & Facing_Up'Image & " of them facing up");
      Check (Found * 2 >= Asked, "only" & Found'Image & " of" & Asked'Image & " queries on a textured box were placed");
      Check (Off = 0, Off'Image & " placed points lie beyond Z of their own uncertainty from the box");
      Check (On_Table = 0, On_Table'Image & " points placed where the line of sight meets the table");
      Check (Interior = 0 or else Facing_Up = Interior,
             Natural'Image (Interior - Facing_Up) & " of the box top's normals face down");
   end Place_A_Box;

   procedure Register is
   begin
      Register ("robot.surfaces.box", "a textured box on a table, its eye's picture now and four keyframes of the arm's "
                & "sweep given, is placed with fewer than half its points, any off by more than Z of their own "
                & "uncertainty or on the table, or its top facing down", Place_A_Box'Access);
   end Register;

end Driver.Robot.Surfaces.Tests;
