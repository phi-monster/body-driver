--  The kinematics of an arm and the lens of the eye it carries, fitted from
--  its joint sweeps alone: the method the legacy driver proved on the x5
--  (legacy/driver/src/kinem.adb), rewritten to measured noise, the one
--  significance rule and no tuning numbers.
--
--  The arm is a chain of joints read in the order of its readings. Joint j
--  turns the eye about an axis (unit W, through the point P nearest the
--  reference eye) by C times its reading change, or slides it along W by C
--  times its reading change. Everything is in the reference eye's frame: the
--  eye at the reference readings (the first keyframe), z along its optical
--  axis, x along +U, y along +V. The eye at readings Q is
--
--     T (Q) = exp (S_1 (Q_1 - Q0_1)) ... exp (S_N (Q_N - Q0_N)),
--
--  so a point X of the reference frame is seen by the eye at Q at
--  Inverse (T (Q)) * X. Images fix lengths only up to one factor; the fit
--  fixes it by making the root mean square of the eye positions over the
--  keyframes one model unit.
--
--  The stages, as the legacy driver had them:
--  1. every joint alone, from the keyframes where only it moved (the others
--     within the step their eye can see): its axis direction and on which
--     side of the eye it lies, over a grid of directions and focal lengths,
--     the side solved in closed form, scored by the median Sampson residual;
--     the focal length shared by the joints;
--  2. the joints together with the focal length, by robust least squares;
--  3. how far every axis lies from the eye, relative to the others: a point
--     the reference keyframe tracks into the keyframes of two joints has one
--     depth, while each joint's keyframes alone give it a depth that scales
--     with that joint's distance; each distance's sign puts the points in
--     front;
--  4. everything at once (focal lengths, principal point, two radial
--     distortion terms, axes, distances, reading scales) by robust least
--     squares on the Sampson residuals, the matches that fit re-chosen until
--     the choice no longer changes; the sign of all translations by which
--     side of both eyes the matched points lie on.
--  Robust means Huber weights at Z (Driver.Conventions) on residuals divided
--  by the noise measured from them.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Numerics;

package Driver.Robot.Kinematics.Fit is

   package Real_Lists is new Ada.Containers.Vectors (Positive, Real);



   type Lens is record
      Fx, Fy, Cx, Cy : Real := 0.0;   --  focal lengths and principal point, pixels
      K1, K2         : Real := 0.0;   --  radial distortion: r_d = r (1 + K1 r^2 + K2 r^4)
   end record;

   function Ray (L : Lens; U, V : Real) return Vec3;
   --  The direction (x, y, 1) of the line of sight through pixel (U, V).

   procedure Project (L : Lens; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the eye's frame lands; In_Front is False behind it.

   type Joint is record
      W     : Vec3 := [0.0, 0.0, 1.0];
      P     : Vec3 := [0.0, 0.0, 0.0];
      C     : Real := 1.0;     --  radians or model units per reading unit
      Slide : Boolean := False;
   end record;

   type Joint_Array is array (Positive range <>) of Joint;

   function Eye_At (J : Joint_Array; Change : Real_Array) return Rigid
     with Pre => Change'Length = J'Length;
   --  The eye at readings Q0 + Change, in the reference eye's frame.

   procedure Across (W : Vec3; E1, E2 : out Vec3);
   --  Two unit directions across W that make a right-handed frame with it.

   --  The fit reports the covariance of these parameters, in this order: the
   --  lens (Lens_Terms: the logarithms of Fx and Fy, Cx, Cy, K1, K2), then per
   --  joint its Joint_Terms: the tilt of its axis along the two directions
   --  across it (Across), the point on it nearest the reference eye moved
   --  along the same two, and its reading scale.
   Lens_Terms : constant := 6;
   type Joint_Term is (Tilt_1, Tilt_2, Point_1, Point_2, Scale);
   Joint_Terms : constant := Joint_Term'Pos (Joint_Term'Last) + 1;
   function Terms (Joints : Natural) return Natural is (Lens_Terms + Joint_Terms * Joints);
   function Term_Of (Joint : Positive; T : Joint_Term) return Positive is
     (Lens_Terms + Joint_Terms * (Joint - 1) + Joint_Term'Pos (T) + 1);

   function Unit_Sigma
     (Joints     : Joint_Array;
      Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Covariance : Real_Lists.Vector) return Real
     with Pre => Changes'Length (2) = Joints'Length;
   --  How uncertain the fit's unit is against its own depths, relative: the
   --  unit is the root mean square of the eye positions over the keyframes
   --  (Changes, one keyframe a row), and the fit's covariance holds the
   --  joints' lengths against its depths; each joint term is moved by its
   --  standard deviation either way. Real'Last when the covariance is not
   --  the fit's of these joints.

   procedure Pose_Covariance
     (Joints      : Joint_Array;
      Change      : Real_Array;
      Covariance  : Real_Lists.Vector;
      Turn, Place : out Mat3)
     with Pre => Change'Length = Joints'Length;
   --  What the fit's uncertainty leaves the eye at readings Q0 + Change
   --  (Eye_At) uncertain by: the covariance of its turn (the rotation vector
   --  that takes it to the truth, in the reference frame, as
   --  Driver.Uncertain has it) and of its place, from the joints'
   --  parameters, each moved by its standard deviation either way. Turn and
   --  Place are Real'Last on the diagonal when the covariance is not the
   --  fit's of these joints.

   --  A point the reference keyframe shows at (U0, V0), seen again at (U, V)
   --  in keyframe Frame (2 or more; the reference is 1).
   type Sighting is record
      Frame  : Positive := 2;
      Track  : Positive := 1;
      U0, V0 : Real := 0.0;
      U, V   : Real := 0.0;
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   type Fit_Report is record
      Fitted     : Boolean := False;
      Stage      : Natural := 0;       --  the last stage reached
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Used       : Natural := 0;       --  sightings in the last fit
      Median_Px  : Real := 0.0;        --  their median Sampson residual
      Sigma_Px   : Real := 0.0;        --  the noise measured from them
      Flipped    : Boolean := False;   --  every translation changed sign to put the points in front
      Determined : Boolean := False;   --  the sightings determine every parameter that has a value of its own
      Focal_Sigma : Real := Real'Last; --  the uncertainty of the focal length across
      Covariance : Real_Lists.Vector;  --  of the parameters above, row by row; empty when not determined
      Depths     : Real_Lists.Vector;  --  per track, the depth of its point along its reference line of
                                       --  sight as the track refinement found it; 0 where it has none
      Depth_Sigmas : Real_Lists.Vector;   --  per track, how uncertain the logarithm of that depth is, the
                                          --  other parameters held; Real'Last where it has none
   end record;

   procedure Fit
     (Changes    : Driver.Numerics.Arrays.Real_Matrix;   --  per keyframe (row), every joint's reading change from the reference
      Visible    : Real_Array;         --  per joint, the reading change its eye can just see (0: unknown)
      Sightings  : Sighting_Array;
      Width, Height : Positive;
      Joints     : out Joint_Array;
      L          : out Lens;
      Report     : out Fit_Report)
     with Pre => Changes'Length (2) = Visible'Length and then Joints'Length = Visible'Length;

   ---------------------------------------------------------------------------
   --  Consensus. A plane holds only some of a view's points, and a dense
   --  matcher answers points the other eye does not show: the estimates
   --  below first find the model the agreeing points share, from samples of
   --  as few points as fix it, then refine it on those points alone. The
   --  samples are drawn by a repeatable generator, and there are as many as
   --  make the chance that none was all of agreeing points at most Z's
   --  two-sided tail.

   function Consensus_Samples (Minimal : Positive; Fraction : Real) return Positive
     with Pre => Fraction > 0.0 and then Fraction <= 1.0;
   --  How many samples of Minimal points that takes when that Fraction of the
   --  points agree.

   type Flag_Array is array (Positive range <>) of Boolean;

   --  A track as the reference eye has it: on the line of sight H (H (3) = 1)
   --  through its reference pixel, at Depth along it (0 when it has none),
   --  the logarithm of the depth uncertain by Sigma.
   type Sight_Point is record
      H     : Vec3 := [0.0, 0.0, 1.0];
      Depth : Real := 0.0;
      Sigma : Real := Real'Last;
   end record;

   type Sight_Point_Array is array (Positive range <>) of Sight_Point;

   --  A plane as the reference eye sees it: the line of sight H meets it at
   --  depth 1 / (A * H). It holds the points X with Normal * X = Offset, the
   --  normal -A / |A| towards the eye and the offset -1 / |A|.
   type Sight_Plane is record
      Found      : Boolean := False;
      A          : Vec3 := [0.0, 0.0, 0.0];
      Covariance : Mat3 := [others => [others => 0.0]];   --  of A
      Points     : Natural := 0;                           --  how many points lie on it
   end record;

   procedure Dominant_Plane (Points : Sight_Point_Array; Plane : out Sight_Plane; On : out Flag_Array)
     with Pre => On'First = Points'First and then On'Last = Points'Last;
   --  The plane most of the points lie on, each point judged by its own
   --  uncertainty: by consensus over samples of three, each scored by the
   --  squares of every point's residual in units of its sigma, none counted
   --  past Z squared; then by weighted least squares on the inverse depths of
   --  the points that lie on it (On), re-chosen at the residuals' own spread
   --  until the choice settles. Covariance is the sandwich over those points.

   procedure Refit_Plane (Points : Sight_Point_Array; On : Flag_Array; Plane : out Sight_Plane)
     with Pre => On'First = Points'First and then On'Last = Points'Last;
   --  The weighted least squares of Dominant_Plane over the points On, the
   --  choice held.

   function Plane_Normal (P : Sight_Plane) return Vec3;
   function Plane_Offset (P : Sight_Plane) return Real;
   function Plane_Offset_Sigma (P : Sight_Plane) return Real;
   --  Along the normal, at the eye.
   function Plane_Tilt_Sigma (P : Sight_Plane) return Real;
   --  The normal's angular uncertainty, along the direction it is least sure of.

   procedure Plane_Axes (P : Sight_Plane; Normal, E1, E2 : out Vec3);
   --  The plane's normal and two directions along it, right-handed with it.

   function On_Plane (P : Sight_Plane; H : Vec3) return Vec3;
   --  Where the line of sight H meets the plane.

   ---------------------------------------------------------------------------
   --  Linking two frames through an eye that sees one plane in both: each
   --  frame's points on the plane, in that plane's own coordinates (along its
   --  axes, Plane_Axes), and where the eye sees them.

   type Plane_Point is record
      X, Y : Real := 0.0;
      U, V : Real := 0.0;
   end record;

   type Plane_Point_Array is array (Positive range <>) of Plane_Point;

   procedure Plane_Homography
     (Points : Plane_Point_Array;
      H      : out Mat3;
      Fits   : out Flag_Array;
      Sigma  : out Real;
      Found  : out Boolean)
     with Pre => Fits'First = Points'First and then Fits'Last = Points'Last;
   --  The homography that takes the plane's coordinates to where the eye sees
   --  them (exact for an eye without distortion, or for coordinates already
   --  freed of it): by the least median of the residuals over samples of
   --  four (the direct linear transform of each, on coordinates centred and
   --  scaled), so up to half the points may be anything; then by robust least
   --  squares on the points that fit (Fits), re-chosen at their own noise
   --  until the choice settles. Sigma is their noise.

   package Flag_Lists is new Ada.Containers.Vectors (Positive, Boolean);

   --  The similarity S that takes the second frame's plane coordinates to
   --  the first's: x -> Scale Rot (Turn) x + Shift.
   type Plane_Link is record
      Found      : Boolean := False;
      Consistent : Boolean := False;   --  one homography explains both frames' points
      Scale, Turn, Shift_X, Shift_Y : Real := 0.0;
      Covariance : Real_Lists.Vector;  --  4 x 4, row by row: log Scale, Turn, Shift_X, Shift_Y
      H          : Mat3 := [others => [others => 0.0]];   --  the first frame's homography into the eye
      Sigma      : Real := Real'Last;  --  the joint fit's noise
      Apart      : Real := Real'Last;  --  the larger of the two homographies' own noise
      Beyond     : Natural := 0;       --  coordinates of the joint fit further than Z of that
      First_Fits, Second_Fits : Flag_Lists.Vector;   --  the points each homography kept
      Used       : Natural := 0;       --  of both
   end record;

   procedure Plane_Chain (First, Second : Plane_Point_Array; Link : out Plane_Link);
   --  One homography H of the first frame's plane into the eye explains both
   --  sets: H (x) for the first's points, H (S (x)) for the second's. From
   --  each set's own homography (Plane_Homography) and the similarity nearest
   --  the map between them, both by robust least squares on the points the two
   --  homographies kept. Consistent is False when more of that joint fit's
   --  residual coordinates lie beyond Z of the two homographies' own noise
   --  than chance at Z's tail explains: then the two are no views of one
   --  plane through one eye. Covariance is the sandwich over the points.

   procedure Plane_Chain_Again (First, Second : Plane_Point_Array; Link : in out Plane_Link)
     with Pre => Natural (Link.First_Fits.Length) = First'Length
                 and then Natural (Link.Second_Fits.Length) = Second'Length;
   --  The joint fit of Plane_Chain alone, from Link and on the points it
   --  kept: how the similarity moves with its inputs.

   procedure Chain_Placement
     (First_Plane, Second_Plane : Sight_Plane;
      Link      : Plane_Link;
      Placement : out Rigid;
      Scale     : out Real);
   --  The second frame in the first, X_first = Placement * (Scale * X_second),
   --  from each frame's plane and the similarity between their coordinates:
   --  the turn takes the second plane's axes to the first's turned by Turn,
   --  and the plane's offsets give the shift along the normal.

   --  A point of a frame and where an eye sees it.
   type Correspondence is record
      X    : Vec3 := [0.0, 0.0, 0.0];
      U, V : Real := 0.0;
   end record;

   type Correspondence_Array is array (Positive range <>) of Correspondence;

end Driver.Robot.Kinematics.Fit;
