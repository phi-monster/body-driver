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

   type Track_Point is record
      Known : Boolean := False;   --  triangulated in front of the reference eye
      X     : Vec3 := [0.0, 0.0, 0.0];
   end record;

   type Track_Point_Array is array (Positive range <>) of Track_Point;

   procedure Track_Points
     (Changes   : Driver.Numerics.Arrays.Real_Matrix;
      Sightings : Sighting_Array;
      Joints    : Joint_Array;
      L         : Lens;
      Points    : out Track_Point_Array)
     with Pre => Changes'Length (2) = Joints'Length;
   --  Every track's point in the reference eye's frame, indexed by track: on
   --  its reference line of sight, at the depth least squares over the
   --  keyframes it was followed into gives it.

   procedure Table
     (Changes      : Driver.Numerics.Arrays.Real_Matrix;
      Sightings    : Sighting_Array;
      Joints       : Joint_Array;
      L            : Lens;
      Normal       : out Vec3;
      Offset       : out Real;
      Offset_Sigma : out Real;
      Sigma        : out Real;
      Found        : out Boolean)
     with Pre => Changes'Length (2) = Joints'Length;
   --  The plane most of the tracked points lie on, in the reference eye's
   --  frame: every track triangulated from its keyframes, the plane's normal
   --  the direction whose median distance of the points from their median
   --  offset is least (over the same lattice of directions the fit searches),
   --  then refined by Huber-weighted least squares at the measured spread.
   --  Normal points to the side of the reference eye, and the plane holds the
   --  points X with Normal * X = Offset, so the eye at the origin is -Offset
   --  from it (Offset_Sigma its uncertainty along Normal); Sigma is the
   --  normal's angular uncertainty.

   --  One point in two frames.
   type Point_Pair is record
      From, To : Vec3 := [0.0, 0.0, 0.0];
   end record;

   type Point_Pair_Array is array (Positive range <>) of Point_Pair;

   procedure Similarity
     (Pairs       : Point_Pair_Array;
      Rotation    : out Mat3;
      Translation : out Vec3;
      Scale       : out Real;
      Scale_Sigma : out Real;
      Spread      : out Real;
      Used        : out Natural;
      Found       : out Boolean);
   --  To = Scale * Rotation * From + Translation: the closed form for the
   --  weighted pairs (Umeyama's), then Huber weights on the residuals at
   --  their measured spread (Spread, per coordinate), the pairs that fit
   --  re-chosen until the choice settles. Scale_Sigma is the scale's
   --  uncertainty: the spread over the points' own spread about their centre.

   --  A point of the reference eye's frame and where another eye sees it.
   type Correspondence is record
      X    : Vec3 := [0.0, 0.0, 0.0];
      U, V : Real := 0.0;
   end record;

   type Correspondence_Array is array (Positive range <>) of Correspondence;

   procedure Resect_Pose
     (Points     : Correspondence_Array;
      L          : Lens;
      Initial    : Rigid;
      Placement  : out Rigid;
      Covariance : out Real_Lists.Vector;
      Sigma      : out Real;
      Used       : out Natural;
      Found      : out Boolean);
   --  Where an eye of known lens stands among points of known position it
   --  sees. Placement maps the eye's frame into the points' (its rotation
   --  holds the eye's axes, its translation the eye's centre), from Initial
   --  by robust least squares on the reprojection, the points that fit
   --  re-chosen until the choice settles. Covariance (6 x 6, row by row) is
   --  that of Placement's turn (the rotation vector in the points' frame that
   --  takes it to the truth) and of its centre: the sandwich over the points.
   --  Sigma is the pixel noise of the points that fit.

   procedure Resect
     (Points        : Correspondence_Array;
      Width, Height : Positive;
      Pose          : out Rigid;
      L             : out Lens;
      Sigma         : out Real;
      Found         : out Boolean);
   --  Another eye's lens and where it stands, from points of known position
   --  it sees: Pose maps the points' frame into the eye's (X_eye = Pose * X).
   --  The direct linear transform of the projection matrix, factored into the
   --  lens and the pose, then everything (the lens with its two radial terms)
   --  by robust least squares on the reprojection, the points that fit
   --  re-chosen until the choice no longer changes. Sigma is the measured
   --  pixel noise; Found is False when the points cannot determine it.

end Driver.Robot.Kinematics.Fit;
