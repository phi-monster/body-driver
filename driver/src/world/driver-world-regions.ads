--  Regions of an eye: the pixels a thing covers.
--
--  Two regions are the same thing when each one's own point lies on the
--  other (LANGUAGE.md 17.7): the point is the region's pixel farthest from
--  its outside, which lies inside it whatever its shape, where a centroid of
--  a bent or hollow region may not.

with Driver.Images;

package Driver.World.Regions is

   subtype Mask is Driver.Images.Mask;
   subtype Pixel is Driver.Images.Pixel;

   function Inner_Point (M : Mask) return Pixel
     with Pre => Driver.Images.Count (M) > 0;
   --  The centre of the region's pixel deepest inside it (distances along
   --  the region in steps of 1 and the square root of 2); of several equally
   --  deep, the first in row order.

   function Radius (M : Mask) return Real
     with Pre => Driver.Images.Count (M) > 0;
   --  How far the inner point lies from the region's outside, in pixels:
   --  the region's own half-width.

   function Same_Pixels (A, B : Mask) return Boolean;
   --  Each region's inner point lies on the other; regions of other sizes or
   --  empty ones are never the same.

   type Box is record
      Column_0, Row_0, Column_1, Row_1 : Natural := 0;   --  inclusive
   end record;

   function Bounds (M : Mask) return Box
     with Pre => Driver.Images.Count (M) > 0;

   function Overlap (A, B : Mask) return Natural;
   --  Pixels in both; zero for regions of other sizes.

end Driver.World.Regions;
