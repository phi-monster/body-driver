--  The surface of what an arm's eye shows in part of its picture, placed by that eye alone from several of the
--  places the arm's sweep held it at.
--
--  The body measures what it is to reach for with its own eyes and nothing else: no instrument service, no other
--  camera seeing the same instant. An eye that rides on a fitted arm has seen the scene from every keyframe of the
--  arm's sweep, each at a pose the fit knows (Kinematics.Eye_In_Reference), its picture kept. A point of the region
--  the eye shows now lies somewhere along its line of sight between the eye and the table; in a keyframe that line
--  is a segment, from where the table would put the point towards where the eye stands now. The aligner
--  (Driver.Alignment) looks for the point along that segment in the keyframes that saw the region from the
--  farthest places, and each answer is kept only when the aligner, asked back from the keyframe, finds it again
--  where it was asked: a texture repeated along the segment can be taken for the point once, not in both
--  directions. The lines of sight through the answers meet where the point is, each weighed by its angular
--  uncertainty over its range; the point's covariance is that of their meeting. A point that lies at the table
--  within its own uncertainty is the table, not what stands on it, and is left out.
--
--  Points are asked on a grid of the region a patch's half width apart (Driver.Alignment.Half_Patch: as far apart
--  as the aligner's patches can overlap by half and still each tell its own place), less the pixels that show the
--  robot itself; a point's normal is the cross product of the differences between its four neighbours on that grid,
--  turned towards the eye, where all four were placed.
--
--  The farthest view is the keyframe whose eye stood farthest from where the eye stands now among those whose
--  picture holds the region's middle at the table; the second is the one standing farthest off the line between
--  those two, so that the lines of sight do not all lie in one plane with the point when the arm's sweep moved the
--  eye in more than one direction.

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

   function In_Region (M : Model; E : Eye_Id; O : Observation; Region : Driver.Images.Mask)
     return Surface_Point_Array;
   --  The surface points of what eye E shows in Region at the beat of O: empty unless the eye rides on an arm that
   --  is fitted, placed in the world and has a table, and O holds the eye's picture and the arm's readings.

   --  The core, on explicit inputs, so that a self test can give it pictures of a scene it made.

   type Lens_Ray is access function (U, V : Real) return Vec3;
   --  The unit line of sight through a pixel, in the eye's own frame.

   type Lens_Project is access procedure (P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the eye's own frame lands in its picture.

   type View is record
      Pose    : Rigid;                       --  the eye in the frame the points are placed in
      Picture : Driver.Alignment.Pyramid;
   end record;

   type View_Array is array (Positive range <>) of View;

   type Pixel_Array is array (Positive range <>) of Driver.Images.Pixel;

   type Placed is record
      Found  : Boolean := False;
      Point  : Point_Estimate;   --  in the frame of the views' poses
      Normal : Vec3 := Zero3;
   end record;

   type Placed_Array is array (Positive range <>) of Placed;

   procedure Place
     (Now      : View;
      Elsewhere : View_Array;
      Queries  : Pixel_Array;
      Columns  : Positive;
      Table    : Driver.Geometry.Plane_Estimate;
      Ray      : not null Lens_Ray;
      Project  : not null Lens_Project;
      Noise    : Real;
      Result   : out Placed_Array)
     with Pre => Result'First = Queries'First and then Result'Last = Queries'Last;
   --  Each query pixel of Now's picture placed from the views Elsewhere (the farthest two are used), as In_Region
   --  places them: Table is in the frame of the poses, its normal towards the eyes; Noise the aligner's error in
   --  pixels. The queries are a grid of Columns per row, row by row, a query missing from the grid given as a pixel
   --  with a negative U (it is not asked and has no neighbours).

end Driver.Robot.Surfaces;
