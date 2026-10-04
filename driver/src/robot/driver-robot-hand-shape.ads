--  A lobe's shape: the surface of it its own eye sees, in the tool frame,
--  and the hand's sizes that follow from it.
--
--  The eye rides on the arm and the closer moves the lobe against it, so to
--  the eye the lobe is a rigid body seen from two places, one at each end of
--  its channel's travel; the matcher says where each of its pixels went
--  (Driver.Robot.Hand.Lobes, the moves). Two views of a rigid body fix its
--  motion between them and how far along its line of sight every pixel is,
--  all but their common scale (Fit). The presses fix that scale: at each
--  opening the lobe's tip lies on its line of sight at a measured distance
--  (Driver.Robot.Hand.Tips), and the two distances must agree on it. With
--  the scale, every point of the lobe the eye sees is known in the tool
--  frame, and with it the lobe's extents (Measure):
--
--    its closing direction, the way its open tip's point of it moves as it
--                   closes, by the fitted motion;
--    its width,     the least extent of what is seen of it across that
--                   direction (rotating calipers on its points' hull there);
--    its thickness, its extent along that direction;
--    its face,      how far ahead of its tip along that direction it
--                   reaches: where it meets what it closes on;
--
--  and the hand's axis, the way the lobes point: along each lobe, across its
--  closing direction and its width, toward its tip, averaged; and its depth,
--  how far back from the middle of the lobes' tips along the axis their
--  moving parts reach: how far a thing may go in between them before it
--  meets them where they begin.
--
--  Only what the eye sees is in these. A side of a lobe facing away from the
--  eye, a part hidden behind another or outside the picture, is not: each
--  extent is at least what is seen of it, the touching finds the rest. A
--  lobe whose pixels do not all move as one rigid piece leaves out those
--  that do not fit, by the same test as everything else. Not measured, and
--  saying why: a lobe whose tip does not move with the rest of it, whose
--  two tips disagree on its scale, or whose sizes do not stand out of their
--  own uncertainty. The last is a lobe that turns about a hinge and is seen
--  mostly on one face: from the start of a pure shift its two views fit a
--  wrong valley (a turn about another axis, the shift reversed), its
--  sightings scatter twice their noise, and that noise, raised with it,
--  leaves its sizes saying nothing.

with Ada.Strings.Unbounded;
with Driver.Images;
with Driver.Robot.Hand.Lobes;

private with Ada.Containers.Indefinite_Holders;

package Driver.Robot.Hand.Shape is

   type Sighting is record
      At_Pixel : Driver.Images.Pixel;   --  the pixel at the open end
      Open     : Vec3;          --  unit, tool frame: from the eye to a point of the lobe at the open end
      Closed   : Vec3;          --  unit: to the same point at the closed end, where the matcher put it
      Sigma    : Real := 0.0;   --  the matcher's angular noise on Closed, in radians
      Pitch    : Real := 0.0;   --  the angle one pixel spans at Open, in radians
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   function From_Moves
     (Moves       : Driver.Robot.Hand.Lobes.Move_Array;
      Back        : Driver.Robot.Hand.Lobes.Move;
      Line        : not null access function (Px : Driver.Images.Pixel) return Vec3;
      Pixel_Sigma : Real) return Sighting_Array
     with Post => From_Moves'Result'Length = Moves'Length + 1;
   --  A lobe's sightings: every pixel of it at the open end with where the
   --  matcher put it at the closed end (Moves, in order), then its closed tip
   --  seen from where that matched back to at the open end (Back: From the
   --  closed tip, To where it went), the last one. Line is a pixel's unit
   --  line of sight in the tool frame, Pixel_Sigma the matcher's noise in
   --  pixels; the angle a pixel spans is taken to its neighbours.

   type Lobe_Shape is private;
   --  A lobe's motion between its ends and its points' distances, up to
   --  their common scale; small to copy, its points live on the heap.

   function Fit
     (Eye           : Vec3;
      Sightings     : Sighting_Array;
      Open_Anchor   : Positive;
      Closed_Anchor : Positive;
      Bordered      : Boolean) return Lobe_Shape
     with Pre => Open_Anchor in Sightings'Range and then Closed_Anchor in Sightings'Range;
   --  Eye is the eye's centre in the tool frame. The anchors are the
   --  sightings of the lobe's tip at the open end and at the closed end, the
   --  points the presses measure. Bordered: the lobe touches the picture's
   --  border at its open end, so where it begins is not seen.

   function Unfitted (Why : String) return Lobe_Shape;
   --  A lobe whose shape cannot be fitted, and why.

   function Fitted (S : Lobe_Shape) return Boolean;
   function Why (S : Lobe_Shape) return String;
   --  Why it is not fitted; empty when it is.

   function Kept (S : Lobe_Shape) return Natural;
   --  The sightings that fit the lobe's rigid motion.

   function Scatter (S : Lobe_Shape) return Real;
   --  How far the kept sightings scatter about the fit, in units of the
   --  matcher's own noise; never below one.

   type Lobe_Size is record
      Width     : Estimate;
      Thickness : Estimate;
      Face      : Estimate;
   end record;
   --  Unknown until measured.

   type Lobe_Size_Array is array (Positive range <>) of Lobe_Size;
   type Lobe_Shape_Array is array (Positive range <>) of Lobe_Shape;
   type Tip_Array is array (Positive range <>, Opening range <>) of Point_Estimate;

   type Hand_Size (Lobes : Natural) is record
      Sizes : Lobe_Size_Array (1 .. Lobes);
      Depth : Estimate;
      Axis  : Direction_Estimate;          --  tool frame: from the lobes' middle toward their tips
      Why   : Ada.Strings.Unbounded.Unbounded_String;   --  what is not measured and why, for the log
   end record;

   function Measure (Shapes : Lobe_Shape_Array; Tips : Tip_Array) return Hand_Size
     with Pre => Tips'First (1) = Shapes'First and then Tips'Last (1) = Shapes'Last;
   --  The hand's sizes from its lobes' shapes and their tips at both
   --  openings (tool frame, from the presses). A lobe is measured when its
   --  shape is fitted and both its tips are known; the depth and the axis,
   --  when every lobe is.

private

   type Point is record
      Depth : Real := 0.0;        --  along Open from the eye, in the fit's unit
      Own   : Real := Real'Last;  --  its variance from its own match alone, in that unit, for unit noise
      Kept  : Boolean := False;
   end record;

   type Point_Array is array (Positive range <>) of Point;

   package Sighting_Holders is new Ada.Containers.Indefinite_Holders (Sighting_Array);
   package Point_Holders is new Ada.Containers.Indefinite_Holders (Point_Array);

   subtype Mat5 is Driver.Numerics.Arrays.Real_Matrix (1 .. 5, 1 .. 5);

   type Lobe_Shape is record
      Fitted        : Boolean := False;
      Why           : Ada.Strings.Unbounded.Unbounded_String;
      Eye           : Vec3 := Zero3;
      Sightings     : Sighting_Holders.Holder;
      Points        : Point_Holders.Holder;
      Rotation      : Mat3 := Identity3;   --  the lobe's turn from the open end to the closed, tool frame
      Shift         : Vec3 := Zero3;       --  unit: X' - Eye = Rotation (X - Eye) + scale * Shift
      Across_1      : Vec3 := Zero3;       --  the two directions the shift's uncertainty is given in
      Across_2      : Vec3 := Zero3;
      Covariance    : Mat5 := [others => [others => 0.0]];
      --  of the turn's small rotation vector and the shift's two components
      --  across itself, for unit noise
      Scatter       : Real := 1.0;
      Kept          : Natural := 0;
      Open_Anchor   : Positive := 1;
      Closed_Anchor : Positive := 1;
      Bordered      : Boolean := False;
   end record;

end Driver.Robot.Hand.Shape;
