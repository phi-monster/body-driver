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
with Driver.Geometry;
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
      Depth_Gains  : Real_Lists.Vector;   --  per track, row by row, how the logarithm of its depth moves
                                          --  with each term of Covariance (the others' depths at their
                                          --  best): the share of its uncertainty every depth has in common
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
      Scatter    : Real := Real'Last;                      --  their residuals' chi square per degree of freedom
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

   function Plane_Response
     (Points : Sight_Point_Array;
      On     : Flag_Array;
      Gains  : Real_Lists.Vector;
      Terms  : Natural) return Driver.Numerics.Arrays.Real_Matrix
     with Pre  => On'First = Points'First and then On'Last = Points'Last,
          Post => Plane_Response'Result'Length (1) = 3 and then Plane_Response'Result'Length (2) = Terms;
   --  How the A of Refit_Plane over the points On moves with the fit's
   --  Terms through the depths alone, the lines of sight held: point I's log
   --  depth moves by the I-th row of Gains (Fit_Report.Depth_Gains) times the
   --  terms' change, carried through the weighted least squares. Three rows
   --  by Terms columns; zero when Gains does not have Terms columns per point
   --  or the points do not fix a plane.

   function Plane_Covariance
     (Points     : Sight_Point_Array;
      On         : Flag_Array;
      Plane      : Sight_Plane;
      Gains      : Real_Lists.Vector;
      Covariance : Real_Lists.Vector) return Mat3
     with Pre => On'First = Points'First and then On'Last = Points'Last;
   --  The covariance of Plane's A (Refit_Plane over the points On), its
   --  lines of sight held: its own (the points' scatter about it) and what
   --  the fit's uncertainty moves every depth by together, which no scatter
   --  shows (Plane_Response, carried with Fit_Report.Covariance). Plane's own
   --  when Gains and Covariance do not fit together.

   function Plane_Estimate_Of (P : Sight_Plane; Covariance : Mat3) return Driver.Geometry.Plane_Estimate;
   --  P, its A uncertain by Covariance, as Driver.Geometry has a plane: its
   --  normal towards the eye, its centre the point of it whose height is
   --  least uncertain (there its offset and its tilt are uncorrelated), the
   --  offset's sigma there and the tilt's covariance, P's points and their
   --  scatter. Unknown when P was not found.

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
   --  Linking two frames through an eye that sees one plane in both.

   --  A point on a plane, in the plane's coordinates, and where an eye sees it.
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
   --  them (exact for an eye without distortion): by the least median of the
   --  residuals over samples of four (the direct linear transform of each, on
   --  coordinates centred and scaled), so up to half the points may be
   --  anything; then by robust least squares on the points that fit (Fits),
   --  re-chosen at their own noise until the choice settles. Sigma is their
   --  noise.

   --  One frame's side of a link: its eye's lens and that lens's covariance,
   --  the tracks on its plane, and where the linking eye answered them.
   type Table_Track is record
      U0, V0 : Real := 0.0;          --  the track's reference pixel
      Depth  : Real := 0.0;          --  along its reference line of sight
      Sigma  : Real := Real'Last;    --  of the logarithm of the depth
   end record;

   package Track_Lists is new Ada.Containers.Vectors (Positive, Table_Track);

   type Eye_Answer is record
      Track : Positive := 1;         --  which of the side's tracks
      U, V  : Real := 0.0;           --  where the linking eye sees it
   end record;

   package Answer_Lists is new Ada.Containers.Vectors (Positive, Eye_Answer);

   type Chain_Side is record
      L                : Lens;
      Lens_Covariance  : Real_Lists.Vector;    --  Lens_Terms x Lens_Terms, row by row (the fit's)
      Plane_Covariance : Mat3 := [others => [others => 0.0]];   --  of its plane's A beyond its lens's
                                                                  --  lines of sight (Plane_Covariance)
      Tracks           : Track_Lists.Vector;    --  the tracks on its plane
      Answers          : Answer_Lists.Vector;
   end record;

   function Side_Plane (S : Chain_Side; L : Lens) return Sight_Plane;
   --  The side's plane through its tracks seen through lens L, their depths
   --  held (Refit_Plane over all of them).

   package Flag_Lists is new Ada.Containers.Vectors (Positive, Boolean);

   --  The joint parameters of a link, in this order: the first plane's
   --  homography into the eye (its eight free entries), the similarity that
   --  takes the second plane's coordinates to the first's (log scale, turn,
   --  two shifts), then per side its lens (Lens_Terms) and how far the eye's
   --  view moves its plane from the one its tracks give (three).
   Similarity_Terms : constant := 4;
   Side_Terms       : constant := Lens_Terms + 3;
   Link_Terms       : constant := Similarity_Terms + 2 * Side_Terms;   --  what the placement rests on
   Chain_Terms      : constant := 8 + Link_Terms;

   type Plane_Link is record
      Found      : Boolean := False;
      Consistent : Boolean := False;   --  one homography explains both sides' points
      X          : Real_Lists.Vector;  --  the joint solution, Chain_Terms
      Covariance : Real_Lists.Vector;  --  of X's last Link_Terms, row by row
      Sigma      : Real := Real'Last;  --  the noise the sides' own homographies leave their answers, pooled
      Joint      : Real := Real'Last;  --  the joint fit's, its answers' residuals only, in the first side's units
      F          : Real := 0.0;        --  the test of the four constraints one homography adds
      First_Fits, Second_Fits : Flag_Lists.Vector;   --  per answer: its side's homography kept it
      Used       : Natural := 0;       --  answers of both sides kept
   end record;

   procedure Plane_Chain (First, Second : Chain_Side; Second_Eye : Boolean; Link : out Plane_Link);
   --  The similarity S that takes the second side's plane coordinates to the
   --  first's (x a track's coordinates on its own side's plane: where its line
   --  of sight through its lens meets the plane, along the plane's axes), from
   --  an eye's view of both. An eye whose lens is unknown answered both sides'
   --  tracks: one homography H of the first plane into it explains both, H (x)
   --  for the first's answers and H (S (x)) for the second's. With
   --  Second_Eye, the second side's own eye answered the first side's tracks,
   --  and its lens explains them: S^-1 (x) back on the second plane, seen
   --  through the second lens. Each side's lens and plane are its own frame's
   --  estimates, held to them by their covariances: the eye's view moves them
   --  only as far as their uncertainty lets, so that their errors are not
   --  taken for the link's. From the homography of each side's answers
   --  (Plane_Homography, at the estimates) and the similarity nearest the map
   --  they give, then all of it by robust least squares on the answers those
   --  homographies kept, each in units of its side's own noise. Consistent is
   --  False when the joint fit leaves more than a free homography for each
   --  side's answers does, by more than Z's tail allows (the extra sum of
   --  squares of the four constraints, F (4, 2 n - 8 per homography)): then
   --  the two sides are no views of one plane through one eye. Covariance is
   --  the sandwich over the answers with the estimates' own.

   procedure Chain_Placement
     (First, Second : Chain_Side;
      Link          : Plane_Link;
      Placement     : out Rigid;
      Scale         : out Real;
      Covariance    : out Real_Lists.Vector)
     with Pre => Natural (Link.X.Length) = Chain_Terms;
   --  The second frame in the first, X_first = Placement * (Scale * X_second):
   --  the turn takes the second plane's axes to the first's turned by the
   --  similarity's turn, the shift follows the similarity's along the first
   --  plane and the planes' offsets across it, each plane and lens as the
   --  link left them. Covariance (7 x 7, row by row) is that of the turn (a
   --  rotation vector in the first frame), the centre and the log of the
   --  scale, carried from the link's.

end Driver.Robot.Kinematics.Fit;
