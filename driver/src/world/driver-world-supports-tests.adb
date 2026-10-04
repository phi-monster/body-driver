with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Tests;

package body Driver.World.Supports.Tests is

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

   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Gen)));

   Sigma : constant Real := 0.001;   --  a point's own uncertainty, per axis

   Stated : Real := Sigma;
   --  The uncertainty the scene's points state, which may understate how
   --  they scatter, as a matcher's noise measured low would.

   function Seen (X : Vec3) return Point_Estimate is
     ((Mean => X + Sigma * [Gaussian, Gaussian, Gaussian], Covariance => (Stated ** 2) * Identity3));

   function Seen_Truly (X : Vec3) return Point_Estimate is
     ((Mean => X + Sigma * [Gaussian, Gaussian, Gaussian], Covariance => (Sigma ** 2) * Identity3));

   Up   : constant Direction_Estimate := (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 0.001);
   Eye  : constant Vec3 := [0.2, 0.0, 0.6];

   --  The table leans by more than Up's own uncertainty, as a level table does
   --  under an Up measured that far off: its heights along Up span several of
   --  the windows a height is level within.
   Lean : constant Real := 0.03;

   function Table_Z (X : Real) return Real is (Lean * (X - 0.5));

   --  The scene as an eye's sampling grid sees it: rows 0 to 19 the table,
   --  0.6 by 0.6, with a box top 0.1 by 0.06 standing 0.05 above it, and a
   --  small box three cells by three standing 0.08 above it, each hiding the
   --  table under it; rows 20 to 29 a wall at x = 0.8, each row one height;
   --  and a corner of rows 0 to 5 the robot itself, at no height in common.
   Count : constant := 20 * 30 + 10 * 40;

   type Part is (Table, Box_Top, Small_Top, Wall, Robot);
   Parts : array (1 .. Count) of Part := [others => Table];

   procedure Scene (Points : out Driver.Geometry.Point_Array; Grid : out Grid_Array) is
      K : Natural := 0;
   begin
      for R in 0 .. 19 loop
         for C in 0 .. 29 loop
            declare
               X : constant Real := 0.2 + 0.6 * Real (C) / 30.0;
               Y : constant Real := -0.3 + 0.6 * Real (R) / 20.0;
            begin
               K := K + 1;
               Grid (K) := (Column => C, Row => R);
               if C >= 25 and then R <= 5 then
                  Parts (K) := Robot;
                  Points (K) := Seen ([0.1 + 0.1 * Uniform, -0.1 + 0.2 * Uniform, 0.1 + 0.3 * Uniform]);
               elsif abs (X - 0.5) <= 0.05 and then abs Y <= 0.05 then
                  Parts (K) := Box_Top;
                  Points (K) := Seen ([X, Y, Table_Z (X) + 0.05]);
               elsif C in 5 .. 7 and then R in 14 .. 16 then
                  Parts (K) := Small_Top;
                  Points (K) := Seen ([X, Y, Table_Z (X) + 0.08]);
               else
                  Parts (K) := Table;
                  Points (K) := Seen ([X, Y, Table_Z (X)]);
               end if;
            end;
         end loop;
      end loop;
      for R in 20 .. 29 loop
         for C in 0 .. 39 loop
            K := K + 1;
            Grid (K) := (Column => C, Row => R);
            Parts (K) := Wall;
            Points (K) := Seen ([0.8, -0.3 + 0.6 * Real (C) / 40.0, 0.3 * Real (R - 20) / 10.0]);
         end loop;
      end loop;
   end Scene;

   --  Every check below held on forty seeds of the generator with the points'
   --  spreads stated right, and on forty with them stated at half.
   Thing_Points : constant := 30;

   procedure Table_And_Boxes_Seen (Seed : Integer; With_Things : Boolean) is
      Found  : Surface_Vectors.Vector;
      Points : Driver.Geometry.Point_Array (1 .. Count);
      Grid   : Grid_Array (1 .. Count);

      function Mostly (S : Surface; Of_Part : Part) return Boolean is
         On : Natural := 0;
      begin
         for M of S.Members loop
            On := On + Boolean'Pos (Parts (M) = Of_Part);
         end loop;
         return 2 * On > Natural (S.Members.Length);
      end Mostly;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, Seed);
      Scene (Points, Grid);
      Find (Points, Grid, Up, Eye, Found);
      Check (Natural (Found.Length) = 3,
             "the scene held" & Found.Length'Image & " supports, not the table and the two box tops");
      if Natural (Found.Length) = 3 then
         Check (Mostly (Found (1), Table) and then Mostly (Found (2), Box_Top) and then Mostly (Found (3), Small_Top),
                "the supports are not the table, the box top and the small box top, largest first");
         declare
            --  The table's normal as it leans, against the fit's own tilt
            --  uncertainty: the true normal's turn from the fitted one along the
            --  fit's two tangents, whitened by their covariance.
            Truth : constant Vec3 := Unit (Vec3'[-Lean, 0.0, 1.0]);
            N     : constant Driver.Geometry.Plane_Estimate := Found (1).Plane;
            A     : constant Real := Truth * N.Tangent_1;
            B     : constant Real := Truth * N.Tangent_2;
            Det   : constant Real := N.Tilt_11 * N.Tilt_22 - N.Tilt_12 * N.Tilt_12;
            Off   : constant Real := Sqrt ((N.Tilt_22 * A * A - 2.0 * N.Tilt_12 * A * B + N.Tilt_11 * B * B) / Det);
         begin
            Check (not Significant (Vector_Gate (2, N.Points - 3), Off, 1.0),
                   "the table's normal is off its lean by" & Off'Image & " sigma");
            Check (Off > 0.0 and then abs Cross (N.Normal, [0.0, 0.0, 1.0]) > 0.5 * Lean,
                   "the table was fitted level, not leaning");
         end;
         if not With_Things then
            return;
         end if;
         declare
            --  Things: one on the table, one on the box, one held above the
            --  table beside the box, one above the box, one on the small box.
            --  A thing seen from the side: a third of its points on the rim
            --  it stands on, the rest up its sides.
            function Thing (X, Y, Bottom : Real; Points : Positive := Thing_Points) return Driver.Geometry.Point_Array is
               --  Its base lies flat on what it stands on, leaning with the table.
               Result : Driver.Geometry.Point_Array (1 .. Points);
            begin
               for I in Result'Range loop
                  declare
                     Dx : constant Real := 0.02 * Uniform;
                     Dy : constant Real := 0.02 * Uniform;
                  begin
                     Result (I) := Seen_Truly ([X + Dx, Y + Dy, Bottom + Lean * Dx
                                                + (if 3 * I <= Points then 0.0 else 0.03 * Real (I) / Real (Points))]);
                  end;
               end loop;
               return Result;
            end Thing;
            function Resting (S : Support) return Boolean is
              (not Significant (Scalar_Gate (S.Height.Degrees_Of_Freedom, Tests => Thing_Points), S.Height.Value,
                                S.Height.Sigma));
            --  Its lowest point is as low as the lowest of that many points on it can be.
            function None (Member : Positive) return Boolean;
            function None (Member : Positive) return Boolean is
               pragma Unreferenced (Member);
            begin
               return False;
            end None;
            function Box_Top_Part (Member : Positive) return Boolean is (Parts (Member) = Box_Top);
            Table_Thing : constant Driver.Geometry.Point_Array := Thing (0.3, 0.2, Table_Z (0.3));
            On_Table : constant Support := Under (Found, Table_Thing, Up, None'Access);
            On_Box   : constant Support := Under (Found, Thing (0.49, 0.0, Table_Z (0.49) + 0.05), Up, None'Access);
            Lifted   : constant Support := Under (Found, Thing (0.3, 0.2, Table_Z (0.3) + 0.2), Up, None'Access);
            Over_Box : constant Support := Under (Found, Thing (0.49, 0.0, Table_Z (0.49) + 0.25), Up, None'Access);
            On_Small : constant Support := Under (Found, Thing (0.31, 0.13, Table_Z (0.31) + 0.08), Up, None'Access);
            --  The box itself, of the points its top showed: it does not rest
            --  on its own top face, but on the table, as high as its top.
            Top      : Driver.Geometry.Point_Array (1 .. Count);
            Tops     : Natural := 0;
         begin
            for K in 1 .. Count loop
               if Parts (K) = Box_Top then
                  Tops := Tops + 1;
                  Top (Tops) := Points (K);
               end if;
            end loop;
            declare
               Itself : constant Support := Under (Found, Top (1 .. Tops), Up, Box_Top_Part'Access);
            begin
               --  Seen only from above: its bottom unseen, its height above the
               --  table a bound, never a height it is seen to float at.
               Check (Itself.Index = 1 and then abs (Itself.Height.Value - 0.05) < 0.005 and then not Itself.Touching,
                      "the box seen from above rests on its own top face, not on the table at its top's height, or"
                      & " is said to touch the table");
            end;
            Check (On_Table.Index = 1 and then Resting (On_Table) and then On_Table.Touching,
                   "a thing on the table does not rest on it");
            declare
               --  A flat thing at the table's near edge: its top over the
               --  table's last points, its rim, the lowest of it, touching the
               --  table beyond them, where the thing itself hides the table. It
               --  rests on the table. The same thing a hand's breadth further
               --  out, over none of the table, rests on nothing.
               At_Edge, Beyond : Driver.Geometry.Point_Array (1 .. Thing_Points);
            begin
               for I in At_Edge'Range loop
                  declare
                     Rim : constant Boolean := 3 * I <= Thing_Points;
                     X   : constant Real := (if Rim then 0.17 + 0.02 * Uniform else 0.2 + 0.05 * Uniform);
                     Y   : constant Real := 0.04 * (Uniform - 0.5);
                  begin
                     At_Edge (I) := Seen_Truly ([X, Y, Table_Z (X) + (if Rim then 0.0 else 0.02)]);
                     Beyond (I) := Seen_Truly ([X - 0.1, Y, Table_Z (X - 0.1) + (if Rim then 0.0 else 0.02)]);
                  end;
               end loop;
               declare
                  Edge_On : constant Support := Under (Found, At_Edge, Up, None'Access);
                  Off     : constant Support := Under (Found, Beyond, Up, None'Access);
               begin
                  Check (Edge_On.Index = 1 and then Edge_On.Touching,
                         "a thing whose rim touches the table just beyond the table's own points, where the thing hides"
                         & " it, does not rest on it: surface" & Edge_On.Index'Image);
                  Check (Off.Index = 0, "a thing over none of the table rests on surface" & Off.Index'Image);
               end;
            end;
            declare
               --  One wild point among them, its mean far under the table but
               --  uncertain by a hand's breadth (as a wrong match leaves one):
               --  it says nothing of how low the thing reaches. On the table,
               --  the thing still touches it at its own points' height; held
               --  above it, its bottom is still not seen.
               function With_Wild (Thing : Driver.Geometry.Point_Array) return Driver.Geometry.Point_Array is
                  Result : Driver.Geometry.Point_Array (1 .. Thing'Length + 1);
               begin
                  Result (1 .. Thing'Length) := Thing;
                  Result (Result'Last) :=
                    (Mean       => [0.3, 0.2, Table_Z (0.3) - 0.164],
                     Covariance => [[0.099 ** 2, 0.0, 0.0], [0.0, 0.099 ** 2, 0.0], [0.0, 0.0, 0.099 ** 2]]);
                  return Result;
               end With_Wild;
               Down   : constant Support := Under (Found, With_Wild (Table_Thing), Up, None'Access);
               Raised : constant Support :=
                 Under (Found, With_Wild (Thing (0.3, 0.2, Table_Z (0.3) + 0.05)), Up, None'Access);
            begin
               Check (Down.Index = 1 and then Down.Touching and then Down.Height.Sigma < 0.01
                        and then Resting (Down),
                      "a wild point under a thing on the table set its height:" & Down.Height.Value'Image & " +-"
                      & Down.Height.Sigma'Image);
               Check (Raised.Index = 1 and then not Raised.Touching and then abs (Raised.Height.Value - 0.05) < 0.01,
                      "a wild point under a thing above the table made it touch, at" & Raised.Height.Value'Image);
            end;
            declare
               --  A face tilted 30 degrees, found 0.3 m off with its tilt
               --  uncertain by a tenth of a radian, whose reach takes in the
               --  thing on the table: carried out that far, its plane passes
               --  2 mm over the table at the thing, uncertain there by some
               --  3 cm. The thing still rests on the table, which it is surely
               --  closest above.
               With_Slope : Surface_Vectors.Vector := Found;
               Normal     : constant Vec3 := [0.0, -0.5, Sqrt (0.75)];
               Here       : constant Vec3 := [0.3, 0.2, Table_Z (0.3) + 0.002];
               Away       : constant Vec3 := [0.0, 0.3, 0.0];
               Slope      : Surface;
            begin
               Slope.Plane := (Centre       => Here + Away - Real'(Normal * Away) * Normal,
                               Normal       => Normal,
                               Tangent_1    => [1.0, 0.0, 0.0],
                               Tangent_2    => Cross (Normal, [1.0, 0.0, 0.0]),
                               Offset_Sigma => 0.001, Tilt_11 => 0.01, Tilt_12 => 0.0, Tilt_22 => 0.01,
                               Points       => 40, Scatter => 1.0);
               Slope.Low_1 := -1.0;
               Slope.High_1 := 1.0;
               Slope.Low_2 := -1.0;
               Slope.High_2 := 1.0;
               With_Slope.Append (Slope);
               declare
                  On : constant Support := Under (With_Slope, Table_Thing, Up, None'Access);
               begin
                  Check (On.Index = 1,
                         "a tilted face carried out far beyond its points, uncertain there by centimetres, was taken"
                         & " for the table's thing's support: surface" & On.Index'Image & " at" & On.Height.Value'Image);
               end;
            end;
            Check (On_Box.Index = 2 and then Resting (On_Box) and then On_Box.Touching,
                   "a thing on the box does not rest on the box");
            Check (Lifted.Index = 1 and then abs (Lifted.Height.Value - 0.2) < 0.01 and then not Lifted.Touching,
                   "a thing held over the table is not over it at its height");
            Check (Over_Box.Index = 2 and then abs (Over_Box.Height.Value - 0.2) < 0.01,
                   "a thing held over the box is not over the box at its height");
            Check (On_Small.Index = 3 and then Resting (On_Small),
                   "a thing on the small box does not rest on it");
            declare
               --  Many things resting on the table, each seen densely, with a
               --  hundred points on its rim: the lowest of them falls as low
               --  as the lowest of so many can, so the table must still hold
               --  all but as many as the test's tail lets go.
               Tries  : constant := 40;
               Dense  : constant := 300;
               Missed : Natural := 0;
               Alpha  : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
               Most   : constant Real := Real (Tries) * Alpha;
            begin
               for T in 1 .. Tries loop
                  declare
                     X : constant Real := 0.25 + 0.1 * Uniform;
                     Y : constant Real := -0.25 + 0.1 * Uniform;
                  begin
                     Missed := Missed + Boolean'Pos (Under (Found, Thing (X, Y, Table_Z (X), Dense), Up, None'Access).Index /= 1);
                  end;
               end loop;
               Check (not (Real (Missed) > Most and then Significant (Real (Missed) - Most, Sqrt (Most * (1.0 - Alpha)))),
                      Missed'Image & " of" & Tries'Image & " things resting on the table were not held by it");
            end;
         end;
      end if;
   end Table_And_Boxes_Seen;

   procedure Table_And_Boxes is
   begin
      Stated := Sigma;
      Table_And_Boxes_Seen (71, With_Things => True);
   end Table_And_Boxes;

   procedure Understated is
      --  The points state half the spread they scatter by. Without scaling
      --  their spread up to the patch's own scatter, most scenes break the
      --  table into pieces; five scenes, so that no lucky one hides it.
   begin
      Stated := Sigma / 2.0;
      for Seed in 71 .. 75 loop
         Table_And_Boxes_Seen (Seed, With_Things => False);
      end loop;
      Stated := Sigma;
   end Understated;

   procedure Flipping is
      --  A scene whose table, seen through spreads stated at half, makes the
      --  refits flip points at the patch's edge back and forth for good: the
      --  points held all through that are the table, not nothing.
   begin
      Stated := Sigma / 2.0;
      Table_And_Boxes_Seen (1, With_Things => False);
      Stated := Sigma;
   end Flipping;

   procedure Register is
   begin
      Driver.Tests.Register ("world.supports.flipping",
                             "a patch whose refits keep flipping points at its edge loses its surface",
                             Flipping'Access);
      Driver.Tests.Register ("world.supports.table", "the planes things rest on, or what rests on which, are wrong",
                             Table_And_Boxes'Access);
      Driver.Tests.Register ("world.supports.understated",
                             "points that scatter more than their stated spread break a surface into pieces",
                             Understated'Access);
   end Register;

end Driver.World.Supports.Tests;
