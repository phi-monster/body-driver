--  The surfaces things rest on, and what rests on which.
--
--  The scene's points come from a grid of pixels of one eye, matched into
--  another (Driver.World.Pairs), each with its place in that grid. A support
--  faces up, so it holds its points at one height along Up. The search
--  starts from the point the most points left are level with (at heights
--  their own uncertainties cannot tell apart). A surface at that height
--  shows there as a patch of the grid: the connected grid neighbours among
--  the level points that hold a two by two block of the grid, where a slice
--  of a wall at one height is a single row. From that seed the patch grows
--  over its grid neighbours for as long as the plane through it holds them,
--  so a surface that leans, or an Up measured off, is still found whole; it
--  is a support when its normal, turned to the side the eyes saw it from,
--  leans significantly towards Up. It reaches as far as its points along
--  its two tangents. Then the next densest point left: one with no patch
--  around it is set aside alone, so the points level with it may still
--  seed a surface around a height of their own. A thing's support is the
--  highest support under it, and its height is its lowest point's height
--  above it, along Up. The lowest point is the lowest one two eyes saw: a
--  face the thing rests on is not seen.

with Ada.Containers.Vectors;
with Driver.Geometry;

package Driver.World.Supports is

   package Member_Vectors is new Ada.Containers.Vectors (Positive, Positive);

   type Surface is record
      Plane          : Driver.Geometry.Plane_Estimate;
      Low_1, High_1  : Real := 0.0;   --  its points' reach along the plane's first tangent, from its centre
      Low_2, High_2  : Real := 0.0;   --  and along its second
      Members        : Member_Vectors.Vector;   --  the points it holds, by their place in what Find was given
   end record;

   package Surface_Vectors is new Ada.Containers.Vectors (Positive, Surface);

   type Grid_Point is record
      Column, Row : Integer := 0;     --  the place of a point's pixel in the sampling grid
   end record;

   type Grid_Array is array (Positive range <>) of Grid_Point;

   procedure Find
     (Points    : Driver.Geometry.Point_Array;
      Grid      : Grid_Array;
      Up        : Direction_Estimate;
      Seen_From : Vec3;
      Found     : out Surface_Vectors.Vector)
     with Pre => Grid'First = Points'First and then Grid'Last = Points'Last;
   --  The supports among the points, largest first; Seen_From is where an
   --  eye that saw them stands.

   type Support is record
      Index    : Natural := 0;      --  in the surfaces given; 0: none under it
      Height   : Estimate;          --  of the lowest point seen above it, along Up
      Touching : Boolean := False;  --  that point on it, within their uncertainties
   end record;

   function Mostly (S : Surface; Of_It : not null access function (Member : Positive) return Boolean)
     return Boolean;
   --  Most of the surface's members are of it.

   function Under
     (Surfaces : Surface_Vectors.Vector;
      Points   : Driver.Geometry.Point_Array;
      Up       : Direction_Estimate;
      Own      : not null access function (Member : Positive) return Boolean) return Support;
   --  The support a thing of these points rests on, or would land on: the
   --  highest one its lowest point is over and not below, that point being
   --  the lowest of them all, so tested as one of a family of as many. Own
   --  says whether a surface's member is a point of the thing itself: a
   --  surface most of whose members are is the thing's own face, seen
   --  before the thing was known, and holds nothing up. Touching says the
   --  lowest point seen is on the support within their uncertainties: the
   --  thing's bottom is seen there. When it is not, Height bounds how far
   --  above the support the thing's bottom can be: the eyes see nothing of it
   --  lower, and what they do not see (the side and the underside of a box
   --  seen from above, the underside of a ball) may reach down to it.

end Driver.World.Supports;
