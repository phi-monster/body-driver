--  The lens and pose of an eye that stands still in the world, measured from
--  points that an arm's own eye has already placed, and the answers the fixed
--  eye gave to where they are.
--
--  An observation is a point, the pose of the link it rides on when the eye
--  saw it, and the pixel it was seen at. A point at rest rides on the base
--  (the identity pose); a feature tracked on the hand would ride on a link,
--  and a pose of the arm's own fit would carry it. The eye's camera frame has
--  z along its optical axis, x along +U and y along +V, and its pose takes a
--  point of that frame into the frame the points are in (the world, which is
--  the first arm's reference eye).
--
--  The fit finds the six terms of the lens (as the arms' fits have them:
--  the logarithms of Fx and Fy, Cx, Cy, K1, K2) and the six of the pose (a
--  turn about the camera's own axes, and the camera's centre in the world) by
--  robust least squares on the pixels' reprojection residuals, in units of the
--  noise the residuals show, with Huber weights at Z. The observations that
--  fit (a residual in neither direction significant against that noise) are
--  chosen again at the new noise until the choice stops changing. The two
--  distortion terms are kept only when, as a pair, they are significant.
--
--  Its covariance is the sandwich of the normal equations around the spread of
--  the gradient, the spread read from the residuals (their covariance as a
--  falling function of the distance between the points in the reference
--  picture, as Driver.Robot.Kinematics.Errors reads the matcher's errors), and
--  the spread the points' own uncertainty adds: each point's own, and what
--  the arm's fit moves every point by together (Common, the Jacobian of the
--  points' positions in the arm fit's terms, and Common_Covariance, those
--  terms' covariance). A point the arm's fit placed with a depth uncertain by a
--  hundredth moves the eye's terms by what that does, not by what the
--  matcher's errors do.
--
--  The eye is determined when its normal equations are positive definite, its
--  covariance finite, both focal lengths significant against their own
--  sigma, its turn known to a cone within a quarter turn at Z sigmas and its
--  centre known. Else it is not, and Why says which. Points of one plane fix
--  the eye only up to one term (the focal length trades against the pose):
--  the points' own uncertainty then leaves the focal length with a sigma as
--  large as itself, and the eye is not determined.
--
--  Start finds where to begin from the plane some of the points lie on and
--  the points off it: the homography of that plane's coordinates into the
--  eye's picture, and what the points at a height above the plane add to it
--  (a point at height Z is seen at H (X, Y, 1) + Z e, in the homogeneous
--  pixel, with e the camera's own third column in H's scale, linear in the
--  pixels), which completes the camera's matrix and with it the focal lengths,
--  the principal point and the pose, wherever the principal point is and
--  however little the plane is tilted. That is a start only: the fit moves
--  every term of the lens.

with Ada.Strings.Unbounded;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Fixed is

   Terms : constant := Fit.Lens_Terms + 6;
   --  The lens's terms in the order of Fit.Lens_Terms, then the turn (three)
   --  and the centre (three).

   type Point is record
      Position : Vec3 := [0.0, 0.0, 0.0];   --  in the frame of the link it rides on
      Own      : Mat3 := [others => [others => 0.0]];   --  its position's covariance, apart from the others'
      U0, V0   : Real := 0.0;   --  where the arm's own eye saw it: points near each other there err alike
   end record;

   type Point_Array is array (Positive range <>) of Point;

   type Pose_Array is array (Positive range <>) of Rigid;
   --  The poses the links carry their points at: X_world = Pose * Position.

   type Sighting is record
      Point : Positive := 1;
      Pose  : Positive := 1;     --  of Poses: the link the point rides on, as it stood
      U, V  : Real := 0.0;       --  where the eye saw it
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   type Fit_Report is record
      Determined  : Boolean := False;
      Why         : Ada.Strings.Unbounded.Unbounded_String;
      L           : Fit.Lens;
      Pose        : Rigid := Driver.Numerics.Identity;   --  the eye in the world
      Covariance  : Fit.Real_Lists.Vector;               --  Terms x Terms, row by row (zeros for the terms not kept)
      Sigma_Px    : Real := 0.0;                         --  the noise the residuals show, per pixel direction
      Used        : Natural := 0;                        --  the sightings that fit
      Offered     : Natural := 0;
      Distorted   : Boolean := False;                    --  the distortion terms are kept
   end record;

   procedure Fit_Eye
     (Points            : Point_Array;
      Poses             : Pose_Array;
      Sightings         : Sighting_Array;
      Common            : Driver.Numerics.Arrays.Real_Matrix;
      Common_Covariance : Driver.Numerics.Arrays.Real_Matrix;
      Width, Height     : Positive;
      Start_Lens        : Fit.Lens;
      Start_Pose        : Rigid;
      Report            : out Fit_Report)
     with Pre => Common'Length (1) in 0 | 3 * Points'Length
                 and then Common'Length (2) = Common_Covariance'Length (1)
                 and then Common_Covariance'Length (1) = Common_Covariance'Length (2)
                 and then (for all S of Sightings => S.Point in Points'Range and then S.Pose in Poses'Range);
   --  Common has three rows per point (its position's change with each of
   --  the arm fit's terms) and a column per term, or no rows at all when
   --  the points' positions have nothing in common beyond Own.

   procedure Start
     (Points     : Point_Array;
      Poses      : Pose_Array;
      Sightings  : Sighting_Array;
      On_Plane   : Fit.Flag_Array;
      Plane      : Vec3;
      Lens       : out Fit.Lens;
      Pose       : out Rigid;
      Found      : out Boolean)
     with Pre => On_Plane'First = Sightings'First and then On_Plane'Last = Sightings'Last;
   --  The eye as the plane A * X = 1 (Plane = A) of the world and the points off
   --  it give it: the homography of the sightings On_Plane (points that lie on
   --  the plane), and from the other points' parallax, their pixels against
   --  where the homography puts their feet on the plane, the rest of the
   --  camera's matrix; its decomposition is the lens (the two focal lengths and
   --  the principal point, no distortion) and the pose. Not Found when the
   --  points on the plane are too few or too alike to give it a homography, or
   --  the others are too few, or all on the plane, to give the parallax: a
   --  plane alone leaves the lens undetermined.

end Driver.Robot.Kinematics.Fixed;
