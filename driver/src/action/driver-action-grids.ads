--  Measured points filed by the cube of space they lie in, so that the ones
--  near a point are found without looking at all of them.
--
--  Each point carries the margin a moving body must keep from it. With
--  cubes at least as large as the largest margin, every point closer to a
--  query than its margin lies in the query's cube or one of its neighbours:
--  a gap below zero found there is the true least gap, and none found means
--  the true least gap is not below zero either.

with Ada.Containers.Ordered_Maps;
with Ada.Containers.Vectors;
with Driver.Numerics;

package Driver.Action.Grids is

   use Driver.Numerics;

   type Grid is private;

   procedure Add (G : in out Grid; Point : Vec3; Margin : Real);
   --  Files a point; the cube size is fixed by Start.

   procedure Start (G : out Grid; Cube : Real)
     with Pre => Cube > 0.0;

   function Cube (G : Grid) return Real;

   function Least_Gap (G : Grid; Q : Vec3) return Real;
   --  Over the points in Q's cube and its neighbours, the least distance
   --  from Q less the point's margin; Real'Last when there are none.

private

   type Key is record
      I, J, K : Integer := 0;
   end record;

   function "<" (A, B : Key) return Boolean is
     (A.I < B.I or else (A.I = B.I and then (A.J < B.J or else (A.J = B.J and then A.K < B.K))));

   type Entry_Point is record
      Point  : Vec3 := Zero3;
      Margin : Real := 0.0;
   end record;

   package Entry_Vectors is new Ada.Containers.Vectors (Positive, Entry_Point);

   package Cube_Maps is new Ada.Containers.Ordered_Maps
     (Key_Type => Key, Element_Type => Entry_Vectors.Vector, "=" => Entry_Vectors."=");

   type Grid is record
      Size  : Real := 0.0;
      Cubes : Cube_Maps.Map;
   end record;

end Driver.Action.Grids;
