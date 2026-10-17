--  Where the points an eye shows lie, placed by the body's own eyes from pictures of the scene seen from elsewhere:
--  no instrument service, no matcher of one camera's picture into another's.
--
--  A pixel of the picture an eye has now lies somewhere along its line of sight, between the eye and the table. In a
--  picture seen from elsewhere (a view: the same eye held at another place, or another eye), that line is a segment
--  from where the table would end it towards where the eye stands now. Each view that sees the segment's end at the
--  table scans it, the widest baseline first, from the nearest depth it still sees towards the table: the aligner
--  (Driver.Alignment) searches windows of its finest size along it, and an answer stands only when the view's place,
--  aligned back into the picture now, comes to the query again (a texture repeated along the way is taken for the
--  point once, not in both directions). An answer is a line of sight more; where it and the eye's own line meet,
--  every other view is asked whether it shows the same patch there, within the meeting's uncertainty carried into
--  its picture and its own error. The point is placed where three lines of sight or more meet within their
--  uncertainties (a chi square, the one line that does not meet the rest left out when four or more do), two of the
--  views telling its depth by themselves about as well as all of them together, with the covariance of their
--  meeting; until then more views scan. The depths of the neighbours on the grid placed already are asked first:
--  a surface goes on from one pixel to the next but where it ends.
--
--  A point at the table within its uncertainty is the table there and is told so; one below it is not placed (the
--  line of sight ends at the table). A point whose patch takes in two surfaces, where its neighbours' depths do not
--  go on through it, is placed by neither and left out. A point's normal is the cross product of the differences
--  between its four neighbours on the grid, turned towards the eye, where all four were placed.
--
--  Points are asked on a grid a patch's half width apart (Driver.Alignment.Half_Patch: as far apart as the
--  aligner's patches can overlap by half and still each tell its own place), less the pixels that show the robot.
--  Driver.World.Sight gives this the views the world keeps.

with Driver.Alignment;
with Driver.Geometry;
with Driver.Images;

package Driver.Robot.Surfaces is

   type Surface_Point is record
      Point  : Point_Estimate;          --  in the world frame
      Normal : Vec3 := Zero3;           --  unit, towards the eye that asked it; zero where its neighbours do not tell it
      Pixel  : Driver.Images.Pixel;     --  where it was asked in the eye's picture
   end record;

   type Surface_Point_Array is array (Positive range <>) of Surface_Point;

   type Lens_Ray is access function (Lens : Positive; U, V : Real) return Vec3;
   --  The unit line of sight through a pixel, in the eye's own frame, by the lens of that number.

   type Lens_Project is access procedure (Lens : Positive; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the eye's own frame lands in its picture, by the lens of that number.

   --  An eye's picture with all that places a point of it: where the eye stood, its lens (by number, the caller's),
   --  and how far the aligner errs on its pictures. Views of different eyes meet as views of one eye do: a point is a
   --  point of the world, seen along each eye's own line of sight through it.
   type View is record
      Pose    : Rigid;                       --  the eye in the frame the points are placed in
      Picture : Driver.Alignment.Pyramid;
      Lens    : Positive := 1;               --  its lens, by the number the caller's Lens_Ray and Lens_Project take
      Noise   : Real := 0.0;                 --  the aligner's error on its pictures, in pixels
   end record;

   type View_Array is array (Positive range <>) of View;

   type Pixel_Array is array (Positive range <>) of Driver.Images.Pixel;

   type Placed is record
      Found    : Boolean := False;
      Point    : Point_Estimate;   --  in the frame of the views' poses
      Normal   : Vec3 := Zero3;
      On_Table : Boolean := False;   --  on the table within their uncertainties: the table there, nothing on it
   end record;

   type Placed_Array is array (Positive range <>) of Placed;

   procedure Place
     (Now       : View;
      Elsewhere : View_Array;
      Queries   : Pixel_Array;
      Columns   : Positive;
      Table     : Driver.Geometry.Plane_Estimate;
      Ray       : not null Lens_Ray;
      Project   : not null Lens_Project;
      Result    : out Placed_Array)
     with Pre => Result'First = Queries'First and then Result'Last = Queries'Last;
   --  Each query pixel of Now's picture placed from the views Elsewhere, of any eyes, as above. Table is in the frame
   --  of the poses, its normal towards the eyes. The queries are a grid of Columns per row, row by row, a query
   --  missing from the grid given as a pixel with a negative U (it is not asked and has no neighbours). A point on
   --  the table within their uncertainties is placed and told so (On_Table); one below it is not placed.

end Driver.Robot.Surfaces;
