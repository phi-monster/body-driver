--  Two eyes the body carries as one. Moved together while neither moves on the other (a wrist eye on an arm held
--  still and an eye on the body under it, the body walked about by what carries it), what each sees again of the
--  points of its own reference picture at a few still moments tells where the second eye sits on the first, both
--  lenses, how each moment moved them, and how far each point lies (Calibrate). And an eye's pose in another
--  picture, from points whose places are known (Locate).
--
--  Frames as Driver.Numerics has them: x across the picture, y down it, z along the line of sight, a Rigid mapping a
--  child frame's points into its parent's. The first eye's reference frame holds everything. A point of either
--  eye's reference picture lies along that pixel's ray at a depth whose inverse is measured: an eye at (R, t) of its
--  reference frame sees it along R^T (r - inverse depth * t), r the ray with z one, so a point at no finite depth is
--  the inverse depth nought. The scale is the pictures' own: the median inverse depth of the first eye's points one.
--  The lenses are pinholes with square pixels (no distortion), their centres given.

with Driver.Numerics;

package Driver.Rig is

   use Driver.Numerics;

   type Lens is record
      Focal  : Real := 0.0;   --  pixels
      Cx, Cy : Real := 0.0;   --  the principal point, pixels
   end record;

   type Place is record
      U, V : Real := 0.0;
   end record;

   type Place_Array is array (Positive range <>) of Place;

   type Sighting is record
      Eye   : Positive := 1;   --  1 the first eye, 2 the second
      Point : Positive := 1;   --  of that eye's reference points
      Still : Positive := 1;   --  of the stills, from 1
      U, V  : Real := 0.0;     --  where it was seen at that still
      Sigma : Real := 0.0;     --  the standard deviation the aligner stated of that place, per axis, pixels
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   type Rigid_Array is array (Positive range <>) of Rigid;

   function Ray (L : Lens; P : Place) return Vec3;
   --  The pixel's line of sight, its z one.

   procedure Project (L : Lens; Q : Vec3; P : out Place; Ahead : out Boolean);
   --  Where a point (or a direction) in the eye's frame is seen; Ahead False for one behind the eye.

   function Seen (Motion : Rigid; R : Vec3; Inverse_Depth : Real) return Vec3 is
     (Transposed_Times (Motion.Rotation, Minus (R, Scaled (Inverse_Depth, Motion.Translation))));
   --  Where an eye that moved by Motion (its pose in its reference frame) sees the point of its reference picture along
   --  ray R at that inverse depth, up to a positive factor: the direction Project takes.

   type Calibration (Stills, First_Points, Second_Points : Natural) is record
      Ok              : Boolean := False;
      Second_On_First : Rigid := Identity;   --  the second eye's frame in the first's
      First_Lens, Second_Lens : Lens;
      Poses           : Rigid_Array (1 .. Stills) := [others => Identity];   --  the first eye at each still
      First_Inverse   : Real_Array (1 .. First_Points) := [others => 0.0];
      Second_Inverse  : Real_Array (1 .. Second_Points) := [others => 0.0];
      --  Each point's inverse depth, in the scale above; nought also for a point no still saw, which Placed says.
      First_Placed    : Driver.Natural_Array (1 .. First_Points) := [others => 0];
      Second_Placed   : Driver.Natural_Array (1 .. Second_Points) := [others => 0];
      --  How many of the stills saw each point within the robust cut at the end; a point seen by none is not placed.
      Turn_Sigma      : Real := Real'Last;
      --  How far the second eye's turn on the first is known: the root of the largest eigenvalue of its covariance,
      --  radians, the residuals' scale taken in.
      Covariance      : Driver.Numerics.Arrays.Real_Matrix (1 .. 8, 1 .. 8) := [others => [others => 0.0]];
      --  Of the second eye's frame on the first (its turn as Exp takes it, then its place, in the scale above) and
      --  both focal lengths' logarithms, the first eye's then the second's: the residuals' scale taken in, every
      --  still's pose and every inverse depth left free, and nothing along the scale (no residual moves it).
      Misfit          : Real := 0.0;   --  the median over the sightings of the larger residual over its sigma
      Scale           : Real := 1.0;
      --  the residuals' own scale in those sigmas, no less than one (Huber's cut is at Z of it)
      Kept, Used      : Natural := 0;  --  the sightings within Z of that scale at the end, of those it could use
      Cost            : Real := 0.0;   --  the adjustment's at its end: Huber's, the residuals over their sigmas
   end record;

   procedure Calibrate
     (First_Points  : Place_Array;
      Second_Points : Place_Array;
      Sightings     : Sighting_Array;
      Stills        : Positive;
      First_Start   : Lens;
      Second_Centre : Place;
      Result        : out Calibration)
     with Pre => Result.Stills = Stills and then Result.First_Points = First_Points'Length
                 and then Result.Second_Points = Second_Points'Length and then First_Start.Focal >= 0.0
                 and then (for all S of Sightings =>
                             S.Eye in 1 .. 2 and then S.Still <= Stills
                             and then S.Point <= (if S.Eye = 1 then First_Points'Length else Second_Points'Length));
   --  First_Start's focal length nought: none is known, and the one at which the stills' turns alone explain the
   --  first eye's sightings best is the start.
   --  The bundle adjustment of both eyes' stills: every still's pose of the first eye, the second eye's frame on the
   --  first, both focal lengths and every sighted point's inverse depth, minimizing the sightings' residuals over the
   --  sigmas they state with Huber's weight at Z (Levenberg and Marquardt; each point's inverse depth taken out of the
   --  normal equations, one number a point). It begins from each eye's turn at every still with no step (a turn alone
   --  explains a view whose points lie far), the second eye's focal length the one that turns it as far as the first
   --  at the stills that turn (a focal length scales the view's turn in pixels), each still's direction of motion given
   --  its turn (the plane through both rays of every point holds it), and the second eye's frame on the first from
   --  every still's motion of both, linear (Andreff, Horaud and Espiau 2001: the turns and the steps together, so that
   --  a body that only turns about the vertical and steps on the floor tells it). Ok when the median sighting lies
   --  within Z of its sigma at the end and the second eye's turn is determined.

   type Locate_Result is record
      Ok         : Boolean := False;
      Pose       : Rigid := Identity;   --  the eye's pose in its reference frame
      Covariance : Driver.Numerics.Arrays.Real_Matrix (1 .. 6, 1 .. 6) := [others => [others => 0.0]];
      --  of the turn (a rotation vector applied on the left of the pose's) and the place, the residuals' scale taken in
      Misfit     : Real := 0.0;   --  the median of the larger residual over its sigma
      Scale      : Real := 1.0;   --  the residuals' own scale in those sigmas, no less than one (Huber's at Z of it)
      Kept, Used : Natural := 0;
   end record;

   procedure Locate
     (L         : Lens;
      Rays      : Driver.Numerics.Arrays.Real_Matrix;   --  per point, a row: its ray (x, y, 1) and inverse depth
      Seen_At   : Place_Array;
      Sigmas    : Real_Array;
      Start     : Rigid;
      Result    : out Locate_Result)
     with Pre => Rays'Length (2) = 4 and then Seen_At'Length = Rays'Length (1) and then Sigmas'Length = Seen_At'Length;
   --  The eye's pose from points whose rays and inverse depths in its reference frame are known and where it sees them:
   --  Gauss and Newton from Start, Huber's weight at Z of the residuals' own scale (the stated sigmas widened by what
   --  the inverse depths and the model leave). Ok when its normal equations are positive definite and six points or
   --  more lie within that cut.

   procedure Triangulate
     (L : Lens; R : Vec3; Poses : Rigid_Array; Seen_At : Place_Array; Sigmas : Real_Array; Start : Real;
      Inverse_Depth : out Real; Variance : out Real; Ok : out Boolean)
     with Pre => Poses'Length = Seen_At'Length and then Sigmas'Length = Seen_At'Length and then Start >= 0.0;
   --  A point's inverse depth along ray R of the reference picture, from where eyes at Poses (in its reference frame)
   --  saw it: Gauss and Newton on the one number from Start, Huber at Z of the residuals' own scale, no less than
   --  nought (no finite depth). Ok when its normal equation is positive: a sighting's two residuals tell the one
   --  number with one left over (the reference's own sighting tells nothing: the point lies on its ray at any depth).
   --  Variance: of the inverse depth, the residuals' own scale taken in; Real'Last where not Ok.

end Driver.Rig;
