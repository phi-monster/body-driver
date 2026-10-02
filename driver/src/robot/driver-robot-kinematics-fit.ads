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
   --  lens (the logarithms of Fx and Fy, Cx, Cy, K1, K2), then per joint the
   --  tilt of its axis along the two directions across it (Across), the
   --  point on it nearest the reference eye moved along the same two, and
   --  its reading scale.

   procedure Pose_Covariance
     (Joints      : Joint_Array;
      Change      : Real_Array;
      Covariance  : Real_Lists.Vector;
      Turn, Place : out Mat3)
     with Pre => Change'Length = Joints'Length;
   --  What the fit's uncertainty leaves the eye at readings Q0 + Change
   --  (Eye_At) uncertain by: the covariance of its turn (as a rotation
   --  vector) and of its place, from the joints' parameters, each moved by
   --  its standard deviation either way. Turn and Place are Real'Last on
   --  the diagonal when the covariance is not the fit's of these joints.

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

   procedure Table
     (Changes   : Driver.Numerics.Arrays.Real_Matrix;
      Sightings : Sighting_Array;
      Joints    : Joint_Array;
      L         : Lens;
      Normal    : out Vec3;
      Sigma     : out Real;
      Found     : out Boolean)
     with Pre => Changes'Length (2) = Joints'Length;
   --  The plane most of the tracked points lie on, in the reference eye's
   --  frame: every track triangulated from its keyframes, the plane's normal
   --  the direction whose median distance of the points from their median
   --  offset is least (over the same lattice of directions the fit searches),
   --  then refined by Huber-weighted least squares at the measured spread.
   --  Normal points to the side of the reference eye; Sigma is its angular
   --  uncertainty.

   --  A point of the reference eye's frame and where another eye sees it.
   type Correspondence is record
      X    : Vec3 := [0.0, 0.0, 0.0];
      U, V : Real := 0.0;
   end record;

   type Correspondence_Array is array (Positive range <>) of Correspondence;

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
