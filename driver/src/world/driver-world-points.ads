--  A point an eye placed by itself (Driver.World.Sight): where it is, and the pixel of the eye's picture it was asked
--  at.

with Ada.Containers.Vectors;
with Driver.Images;

package Driver.World.Points is

   type Placed is record
      Pixel : Driver.Images.Pixel;          --  where it was asked in the eye's picture
      Point : Point_Estimate;               --  world frame
      Eye   : Eye_Id := Eye_Id'First;       --  the eye that placed it
   end record;

   package Placed_Vectors is new Ada.Containers.Vectors (Positive, Placed);

end Driver.World.Points;
