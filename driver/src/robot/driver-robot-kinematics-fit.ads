--  The kinematics of an arm and the lens of the eye it carries, fitted from
--  its joint sweeps alone: the method the legacy driver proved on the x5
--  (legacy/driver/src/kinem.adb), rewritten to measured noise, the one
--  significance rule and no tuning numbers.
--
--  The arm is a chain of joints. Joint j turns the eye about an axis (unit
--  W, through the point P nearest the reference eye) by C times its reading
--  change, or slides it along W by C times its reading change. C is the
--  reading's unit, which no robot is asked to report in any one way (radians,
--  degrees, an encoder's steps, a scale the scrambler drew): the fit measures
--  it with the rest. The joints compose in their order along the arm, from
--  its base to the eye, which is not the order of the readings either (a
--  scrambled run gives them in any order): the fit measures that too (Chain).
--  Everything is in the reference eye's frame: the eye at the reference
--  readings (the first keyframe), z along its optical axis, x along +U, y
--  along +V. The eye at readings Q is
--
--     T (Q) = exp (S_c1 (Q_c1 - Q0_c1)) ... exp (S_cN (Q_cN - Q0_cN)),
--
--  c1 the joint nearest the base and cN the one nearest the eye, so a point X
--  of the reference frame is seen by the eye at Q at Inverse (T (Q)) * X.
--  Images fix lengths only up to one factor; the fit fixes it by making the
--  root mean square of the eye positions over the unit's keyframes one model
--  unit: all the keyframes of an arm's first fit, and afterwards those of the
--  first fit (Fit's Unit_Frames), so that a keyframe taken later refines
--  every term and moves no length.
--
--  The stages, as the legacy driver had them but for the readings' units and
--  order, which it took from the x5 (radians, in the order of its chain):
--  1. every joint alone, from the keyframes where only it moved (the others
--     within the step their eye can see): its axis direction, how far it
--     turns per reading unit, and on which side of the eye the axis lies, over
--     a grid of directions and focal lengths. About each direction each of
--     its widest keyframes is fitted as a turn about it and a shift across
--     it, the turn free (the essential matrix of such a motion has four
--     entries, linear in the sightings); the direction they fit best is the
--     axis, the turns over the reading changes its scale, and its side is
--     solved in closed form; then axis, side and scale are refined by robust
--     least squares on every keyframe of the joint alone, whose median
--     Sampson residual scores the focal length shared by the joints;
--  2. the joints together with the focal length, the scales among them, by
--     robust least squares;
--  3. how far every axis lies from the eye, relative to the others: a point
--     the reference keyframe tracks into the keyframes of two joints has one
--     depth, while each joint's keyframes alone give it a depth that scales
--     with that joint's distance; each distance's sign puts the points in
--     front;
--  4. the keyframes of one joint each in one refinement of the tracks, as
--     stage 5 has it, which any order of the joints explains alike: an
--     epipolar residual cannot tell a turn that shifts the view the way the
--     joint's shift moves it (a pitch joint whose axis lies behind the eye),
--     for a point's depth along its line of sight takes it up; the one depth
--     a point has for every joint tells it. Then the order of the joints
--     along the arm, from the keyframes that moved several at once: a
--     joint's own keyframes tell its axis wherever it is along the arm, but a
--     turn of a joint nearer the base carries the axes of the joints after
--     it, so the eye's pose depends on the order its joints compose in, and
--     it moves the eye mostly in how far it goes, which no epipolar residual
--     sees: the points, at their depths, reprojected into those keyframes
--     tell it. From the joints ordered by their axes' distance from the eye,
--     the farthest first, a joint is moved to another place in the order
--     while that lowers the median reprojection residual by more than its
--     standard error. Then everything at once (focal
--     lengths, principal point, two radial distortion terms, axes, distances,
--     reading scales) by robust least squares on the Sampson residuals, the
--     matches that fit re-chosen until the choice no longer changes; the sign
--     of all translations by which side of both eyes the matched points lie
--     on;
--  5. the points the keyframes follow, each with a depth of its own along its
--     reference line of sight, and every sighting's reprojection residual
--     (the depths solved by their Schur complement), until a step cannot move
--     any combination of the parameters by more than Unchanged_Fraction of its
--     standard error; the covariance is made at that solution.
--  Robust means Huber weights at Z (Driver.Conventions) on residuals divided
--  by the noise measured from them.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Conventions;
with Driver.Geometry;
with Driver.Numerics;

package Driver.Robot.Kinematics.Fit is

   package Real_Lists is new Ada.Containers.Vectors (Positive, Real);
   package Joint_Lists is new Ada.Containers.Vectors (Positive, Positive);

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

   type Vec3_Array is array (Positive range <>) of Vec3;
   --  An arm's joints in the order of their readings.

   type Chain_Order is array (Positive range <>) of Positive;
   --  An arm's joints along it, from its base to the eye: each by its place
   --  among the readings, 1 for the first.

   function Eye_At (J : Joint_Array; Chain : Chain_Order; Change : Real_Array) return Rigid
     with Pre => Change'Length = J'Length and then Chain'Length = J'Length;
   --  The eye at readings Q0 + Change, in the reference eye's frame: the
   --  joints composed in Chain's order, each by its own reading's change.

   procedure Across (W : Vec3; E1, E2 : out Vec3);
   --  Two unit directions across W that make a right-handed frame with it.

   --  The fit reports the covariance of these parameters, in this order: the
   --  lens (Lens_Terms: the logarithms of Fx and Fy, Cx, Cy, K1, K2), then per
   --  joint its Joint_Terms: the tilt of its axis along the two directions
   --  across it (Across), the point on it nearest the reference eye moved
   --  along the same two, and its reading scale.
   type Lens_Term is (Log_Fx, Log_Fy, Centre_X, Centre_Y, Radial_1, Radial_2);
   Lens_Terms : constant := Lens_Term'Pos (Lens_Term'Last) + 1;
   type Joint_Term is (Tilt_1, Tilt_2, Point_1, Point_2, Scale);
   Joint_Terms : constant := Joint_Term'Pos (Joint_Term'Last) + 1;
   function Moves_The_Fit (Decrease : Real) return Boolean is
     (Decrease > 0.5 * Driver.Conventions.Unchanged_Fraction ** 2);
   --  Whether a step of the final refinement that lowers its cost by Decrease
   --  (half the robust chi square, in units of the noise) is worth taking: it
   --  moves some combination of the parameters by more than Unchanged_Fraction
   --  of its standard error (a step of m standard errors along an axis of the
   --  information lowers the cost by m squared over two, at most). A fraction
   --  of the cost itself would stop short, by several standard errors, on the
   --  axes the data barely determine (the focal length against the distortion),
   --  for the cost they carry is far below any fraction of it.

   type Error_Model is record
      Floor : Real := 0.0;   --  what the model leaves of any sighting, in pixels
      Gain  : Real := 1.0;   --  how much of what the matcher states of a sighting's place its residual bears out
   end record;
   --  How far a sighting's residual lies, as the residuals show it: a floor common to every sighting, and the gain
   --  times what the matcher states of the sighting (a sighting's own standard deviation), as variances.

   function Scale_Of (Model : Error_Model; Own : Real) return Real;
   --  The scale a residual of a sighting stating Own is judged at: Model's floor and its gain times Own, as variances.
   --  Never zero. Read from the residuals of thousands of sightings, it is judged as known (at Z).

   function Model_Of (R, Own : Real_Array) return Error_Model
     with Pre => R'Length = Own'Length;
   --  The floor and the gain read from the residuals R beside the standard deviations Own their sightings state (one
   --  per residual): those at which the median of a residual over its scale is the Gaussian's, over all of them and
   --  over the half that states the more, so a residual is as typical of its scale whatever its sighting states. With
   --  none stated the floor is the noise the residuals show (the robust scale about zero) and the gain one; a floor of
   --  zero when what the sightings state accounts for the residuals by itself, a gain of zero when it accounts for
   --  nothing the floor does not.

   function Terms (Joints : Natural) return Natural is (Lens_Terms + Joint_Terms * Joints);
   function Term_Of (Joint : Positive; T : Joint_Term) return Positive is
     (Lens_Terms + Joint_Terms * (Joint - 1) + Joint_Term'Pos (T) + 1);

   function Unit_Sigma
     (Joints     : Joint_Array;
      Chain      : Chain_Order;
      Changes    : Driver.Numerics.Arrays.Real_Matrix;
      Covariance : Real_Lists.Vector) return Real
     with Pre => Changes'Length (2) = Joints'Length and then Chain'Length = Joints'Length;
   --  How uncertain the fit's unit is against its own depths, relative: the
   --  unit is the root mean square of the eye positions over the unit's
   --  keyframes (Changes, one keyframe a row), and the fit's covariance holds
   --  the joints' lengths against its depths; each joint term is moved by its
   --  standard deviation either way. Real'Last when the covariance is not
   --  the fit's of these joints.

   procedure Pose_Covariance
     (Joints      : Joint_Array;
      Chain       : Chain_Order;
      Change      : Real_Array;
      Covariance  : Real_Lists.Vector;
      Turn, Place : out Mat3)
     with Pre => Change'Length = Joints'Length and then Chain'Length = Joints'Length;
   --  What the fit's uncertainty leaves the eye at readings Q0 + Change
   --  (Eye_At) uncertain by: the covariance of its turn (the rotation vector
   --  that takes it to the truth, in the reference frame, as
   --  Driver.Uncertain has it) and of its place, from the joints'
   --  parameters, each moved by its standard deviation either way. Turn and
   --  Place are Real'Last on the diagonal when the covariance is not the
   --  fit's of these joints.

   procedure Move_Term (J : in out Joint_Array; L : in out Lens; Term : Positive; By : Real)
     with Pre => Term <= Terms (J'Length);
   --  The fit's term Term, in its covariance's order (the lens's Lens_Terms, then each joint's Joint_Terms), moved by
   --  By: the logarithm of a focal length, the principal point or a distortion term of L, or a joint's tilt along the
   --  two directions across its axis (Across), its point moved along them, or its reading scale.

   function Step_Covariance
     (Joints     : Joint_Array;
      Chain      : Chain_Order;
      From, To   : Real_Array;
      Covariance : Real_Lists.Vector) return Mat3
     with Pre => From'Length = Joints'Length and then To'Length = Joints'Length
                 and then Chain'Length = Joints'Length;
   --  What the fit's uncertainty leaves the eye's displacement from readings Q0 + From to Q0 + To uncertain by (the
   --  difference of Eye_At's places), from the joints' parameters, each moved by its standard deviation either way:
   --  one fit's errors move both ends, and a step short beside where the eye stands is known far better than either
   --  end. Real'Last on the diagonal when the covariance is not the fit's of these joints.

   function Place_Covariance_In_Unit
     (Joints       : Joint_Array;
      Chain        : Chain_Order;
      Change       : Real_Array;
      Unit_Changes : Driver.Numerics.Arrays.Real_Matrix;
      Covariance   : Real_Lists.Vector) return Mat3
     with Pre => Change'Length = Joints'Length and then Chain'Length = Joints'Length
                 and then Unit_Changes'Length (2) = Joints'Length;
   --  The covariance of the eye's place at readings Q0 + Change (Eye_At) in the arm's own unit, the root mean
   --  square of the eye's positions over the unit's keyframes (Unit_Changes, one a row, as Unit_Sigma has them):
   --  each joint term is moved by its standard deviation either way and the place is taken over the unit the moved
   --  joints make, times the unit the fit's joints make. What the fit leaves uncertain of the arm's lengths against
   --  its depths moves every place with the unit, the farther from the reference the more, and every length the
   --  hand compares (a tip's distance, a press's hit, the eye's room above the table) is in that unit: in it, that
   --  part is none (a18 on 10-16, a14's body: the eye 6.5 units below the reference, its place uncertain by 0.82
   --  units by Pose_Covariance, 12 per cent, where the truth puts the arm's tool within 0.16 mm, a seventieth of a
   --  unit, once its scale is taken out). Real'Last on the diagonal as Pose_Covariance.

   --  A point keyframe Anchor shows at (U0, V0), seen again at (U, V) in
   --  keyframe Frame (2 or more; the reference is 1).
   type Sighting is record
      Frame  : Positive := 2;
      Track  : Positive := 1;
      U0, V0 : Real := 0.0;
      Anchor : Positive := 1;
      --  The keyframe whose picture (U0, V0) is a pixel of, its eye the one the point's depth is taken from: the
      --  reference unless the point was chosen in a later keyframe, which followed it into others (a relay). Every
      --  sighting of a track has its track's.
      U, V   : Real := 0.0;
      Own    : Real := 0.0;
      --  The standard deviation the matcher states of where it saw the point again, in pixels, per axis; zero when
      --  it states none. A matcher is the less sure of a point the farther it moved (its patch is warped more), and a
      --  residual of such a sighting is typical at a larger size than one of a sighting that moved little.
      Own_Freedom : Natural := 0;
      --  How many degrees of freedom that standard deviation rests on (the matcher estimated it from so many other
      --  fits of the point): a residual over it follows Student's t with that many, not the Gaussian; zero when it is
      --  known.
      Off_U, Off_V : Real := 0.0;
      --  How far the joints its keyframe did not ask moved the point, in pixels: where they stand off the reference
      --  (a servo's landing within its dead band, its sag under a load that changed) times how far the point moves
      --  per unit of each (first order). The stages that fit one joint at a time take the point at U - Off_U,
      --  V - Off_V, where the keyframe's own joint alone would have put it.
      Off_Told : Boolean := True;
      --  Every such joint's motion of the point per unit is told; when not, the one-joint stages leave the sighting
      --  out.
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   --  The errors of the sightings as the residuals of a fit show them (Errors), in
   --  pixels, across and down together: a sighting's own, the part of it a point
   --  has in every keyframe (and the distance in pixels at which that has fallen
   --  to half between points), and what a keyframe adds for its points; and how much of the covariance's spread the clip to
   --  positive semi-definiteness took away (its negative eigenvalues over all, 0
   --  when it took none).
   type Error_Summary is record
      Measured         : Boolean := False;
      Alone            : Real := 0.0;
      Persistent       : Real := 0.0;
      Persistent_Half  : Real := 0.0;
      Keyframe         : Real := 0.0;
      Clipped          : Real := 0.0;
   end record;

   type Fit_Report is record
      Fitted     : Boolean := False;
      Stage      : Natural := 0;       --  the last stage reached
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Used       : Natural := 0;       --  sightings in the last fit
      Median_Px  : Real := 0.0;        --  their median Sampson residual
      All_Median_Px : Real := 0.0;     --  the median residual over every sighting given, inlier or not: comparable
                                       --  between two fits of the same sightings that keep different inliers
      All_Count  : Natural := 0;       --  the sightings that median is over
      Cost_Judged : Real := 0.0;
      --  The robust cost (Huber at Z, no residual counting more than one at Z) of every sighting's residuals across and
      --  down, inlier or not, each in units of the
      --  scale the error model the refinement was asked to judge by gives it (Refine_Tracks' Judge_Floor and Judge_Gain;
      --  its own when none): comparable between two fits of the same sightings that keep different inliers
      Judge_Floor, Judge_Gain : Real := 0.0;   --  the error model Cost_Judged was taken at
      Pinhole    : Boolean := False;   --  the lens fitted as a pinhole with square pixels: its aspect and distortion
                                       --  not significant as a group (Fit's stage 5)
      Sigma_Px   : Real := 0.0;        --  the noise measured from them (the error model's floor)
      Own_Gain   : Real := 1.0;        --  and how much of the matcher's stated sigmas they bear out (its gain)
      Flipped    : Boolean := False;   --  every translation changed sign to put the points in front
      Order_Told : Boolean := False;   --  the sightings tell the joints' order along the arm, not only that it is
                                       --  the best of the orders (stage 4; a fit begun from another, that one's)
      Untold     : Joint_Pair_Lists.Vector;  --  when stage 4 did not tell the order: the joints next to each other along
                                             --  the best order whose swap leaves residuals not significantly the
                                             --  larger; empty when told, or when no stage 4 was run
      Slacks       : Real_Lists.Vector;
      --  Per joint, half its gears' slack in reading units (Fit's Play); empty when the fit was given no way.
      Slack_Places : Real_Lists.Vector;
      --  Per joint, where its slack stood at the reference keyframe, from the slack's middle, in reading units.
      Untold_Turns : Real_Lists.Vector;
      --  Per pair of Untold, two: the turns of its Earlier joint and of its Later one, in their readings' units, that
      --  tell the untold orders apart most (by the pixels between their places of the points times the root of how
      --  many stay in the picture) of those that put the points no farther apart than a narrow window follows.
      Untold_Orders : Joint_Lists.Vector;
      --  Every order of the joints the sightings do not tell from the best (stage 4's search over all of them),
      --  one after another, each as many joints long as the fit has: where the arm may be at a keyframe still to
      --  be aligned, as far as the order goes (Kinematics.Offer_Of); empty when told, or when no stage 4 was run.
      Determined : Boolean := False;   --  the sightings determine every parameter that has a value of its own
      Focal_Sigma : Real := Real'Last; --  the uncertainty of the focal length across
      Covariance : Real_Lists.Vector;  --  of the parameters above, row by row, from the errors the residuals
                                       --  show (Errors); empty when not determined
      Errors     : Error_Summary;      --  what the covariance rests on
      Focal_Design : Real := 1.0;
      --  How much the errors' dependence (Errors) widens the focal length's variance over what its errors as if
      --  independent would give, along the focal length's own direction: one where it does not widen it.
      Depths     : Real_Lists.Vector;  --  per track, the depth of its point along its reference line of
                                       --  sight as the track refinement found it; 0 where it has none
      Depth_Sigmas : Real_Lists.Vector;   --  per track, how uncertain the logarithm of that depth is, the
                                          --  other parameters held; Real'Last where it has none
      Depth_Gains  : Real_Lists.Vector;   --  per track, row by row, how the logarithm of its depth moves
                                          --  with each term of Covariance (the others' depths at their
                                          --  best): the share of its uncertainty every depth has in common
      Seconds      : Real_Lists.Vector;   --  per stage reached, from the first, how long the fit spent in it
      --  The focal search's width (half, in the logarithm) and range, least and most, which a fit begun from this one
      --  takes up (Fit_Start); 0 where there was no search.
      Focal_Within, Searched_Least, Searched_Most : Real := 0.0;
      --  The sightings of the last fit that walked the focal length's profile, this one or one it was begun from; 0
      --  where none did.
      Profiled_Over : Natural := 0;
      --  Per joint, three numbers: for a joint that slides, the turn per reading unit (its axis times its rate) of the
      --  turn its own keyframes were explained by nearly as well when stage 1 took it for a slide, which every fit
      --  begun from this one weighs against the slide again (Fit_Start); zero for a joint that turns, or has none.
      Slide_Turns : Real_Lists.Vector;
      --  The joints whose turn stage 4 told from the slide that moves the eye as it does (either way: the two apart
      --  by more than Z, the turn determined), in increasing order: a fit begun from this one keeps them turning
      --  without weighing them again (Fit_Start). A joint that slides is never among them.
      Turns_Told : Joint_Lists.Vector;
   end record;

   type Flag_Array is array (Positive range <>) of Boolean;

   function Slide_Turns_Of (Report : Fit_Report; N : Natural) return Vec3_Array;
   --  Report.Slide_Turns, a turn a joint, for a fit begun from it (Fit_Start); every one zero when it has none.

   function Turns_Told_Of (Report : Fit_Report; N : Natural) return Flag_Array;
   --  Report.Turns_Told, a flag a joint, for a fit begun from it (Fit_Start).

   function Depth_Known (Depth, Sigma : Real) return Boolean;
   --  Whether a track's depth is one: positive, and significant against its own relative uncertainty (Sigma is that
   --  of its logarithm), which is to say known to better than 1 / Z of itself. A track the sightings leave with no
   --  depth at all (the first fit of an arm once had one at 1.7E97 units, its logarithm uncertain by 2.5E6) is not
   --  known; it is not a point of little weight.

   type Flag_Matrix is array (Positive range <>, Positive range <>) of Boolean;

   --  Where a fit may begin from (Fit's From): a fit of the same arm just made, on nearly the same sightings, its joints,
   --  their order along the arm, its lens, and its focal search's width and range (Fit_Report); none unless Valid.
   type Fit_Start (N : Natural) is record
      Valid          : Boolean := False;
      Joints         : Joint_Array (1 .. N);
      Chain          : Chain_Order (1 .. N);
      L              : Lens;
      Focal_Within   : Real := 0.0;
      Searched_Least : Real := 0.0;
      Searched_Most  : Real := 0.0;
      --  A fit of an estimate before this one: the stages after the focal search (2 to 5) run from it, its joints and
      --  lens taken for the search's result. Otherwise a fit of this estimate: the last stage alone.
      Whole          : Boolean := False;
      Profiled_Over  : Natural := 0;   --  the sightings its focal length's profile was last walked over (Fit_Report)
      Order_Told     : Boolean := False;   --  its joints' order was told (Fit_Report)
      Slide_Turns    : Vec3_Array (1 .. N) := [others => [0.0, 0.0, 0.0]];   --  per joint (Fit_Report)
      Turns_Told     : Flag_Array (1 .. N) := [others => False];   --  per joint (Fit_Report)
      Untold_Orders  : Joint_Lists.Vector;
      --  The orders its search left untold beside its order (Fit_Report.Untold_Orders, each N joints long): an order
      --  one move from that order and not among them was told from it, and stays told while it is kept.
      Fixed          : Boolean := False;
      --  Its order is kept as given (Chain, whether Valid or not: a first fit of a given order runs every stage but
      --  the search), searched for no other and not said to be told: the fit of one order of the joints, every term
      --  refined in it, to be set beside the fits of other orders.
   end record;

   No_Start : constant Fit_Start := (N => 0, Valid => False, others => <>);

   --  The readings' way from the reference keyframe on, for the joints' gears' slack (Fit's Play). Per joint (a
   --  column of Path), what its reading did between the keyframes, stretch by stretch in time order: each the change
   --  over a stretch it went one way, a keyframe's beat ending one; per keyframe (a row of Upto) and joint, how many
   --  stretches came before it. A joint's slack goes with every change of its reading and stays within a half slack
   --  either way of its middle (a play operator: the motor turns within the slack before it takes the link along),
   --  from where it stood at the reference, and the joint stands at its reading less that. A joint its load holds to
   --  one side of its slack is fitted a half slack of nought. With no rows, the slack is not fitted.
   type Play_Paths (Stretches, Frames, Joints : Natural) is record
      Path : Driver.Numerics.Arrays.Real_Matrix (1 .. Stretches, 1 .. Joints);
      Upto : Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. Joints);
   end record;

   No_Play : constant Play_Paths :=
     (Stretches => 0, Frames => 0, Joints => 0, Path => [others => [others => 0.0]],
      Upto => [others => [others => 0.0]]);

   No_Slack : constant Real_Array (1 .. 0) := [others => 0.0];

   function Play_Parts (Play : Play_Paths; Slack : Real_Array) return Driver.Numerics.Arrays.Real_Matrix
     with Pre => Slack'Length = 2 * Play.Joints;
   --  Per keyframe (row) and joint, where its slack stands less where it stood at the reference, in reading units:
   --  the play operator along Play, for the half slacks and places at the reference Slack (two a joint, in that
   --  order), the place clamped within the half slack.

   function Measure_Slack
     (Changes : Driver.Numerics.Arrays.Real_Matrix; Sightings : Sighting_Array; Play : Play_Paths) return Real_Array;
   --  A measure of every joint's slack from the keyframes alone, before any geometry: per joint its half slack and
   --  where its slack stood at the reference (two a joint), those that let every track's sightings move most nearly
   --  in proportion to the joints' turns (the changes less the slack's, Play_Parts): each track's displacements from
   --  its reference pixel regressed on the turns of the joints that turned in its keyframes, by least squares, over
   --  every track, found joint by joint in turn, each joint's from the halvings of the most its reading went from the
   --  reference down to nought and its place at either end or the middle, until a round changes none. The fit's first
   --  stages take the changes less these and its refinement begins from them (Fit's Slack_Start): taken from the
   --  readings, the fake's seed 20 (10-22, every quirk on) searched its focal length down to 20 px and its fit to
   --  1.6, its base turn's four degrees of slack taken for the joint's turns. No_Slack where Play has no rows.

   procedure Fit
     (Changes    : Driver.Numerics.Arrays.Real_Matrix;   --  per keyframe (row), every joint's reading change from the reference
      Asked      : Flag_Matrix;        --  per keyframe (row) and joint: the sweep asked the joint to turn for it
      Visible    : Real_Array;         --  per joint, the reading change its eye can just see (0: unknown)
      Sightings  : Sighting_Array;
      Width, Height : Positive;
      Unit_Frames : Natural;           --  how many of the first keyframes define the unit of length (0: all of them)
      Joints     : out Joint_Array;
      Chain      : out Chain_Order;
      L          : out Lens;
      Report     : out Fit_Report;
      Who        : String := "";   --  what is fitted, for a line at every stage it enters; none when empty
      From       : Fit_Start := No_Start;   --  a fit just made to begin from: the last stage alone (Fit_Start)
      Play       : Play_Paths := No_Play;
      --  the readings' way between the keyframes (Play_Paths); given, the nonlinear stages fit each joint's half slack
      --  and where its slack stood at the reference (Fit_Report.Slacks, Slack_Places), every keyframe's change taken
      --  less its slack's since the reference
      Slack_Start : Real_Array := No_Slack)
      --  with Play, per joint its half slack and place at the reference as measured (Measure_Slack): the stages before
      --  the refinement take the changes less the slack's, and the refinement begins from it
     with Pre => Changes'Length (2) = Visible'Length and then Joints'Length = Visible'Length
                 and then Asked'Length (1) = Changes'Length (1) and then Asked'Length (2) = Visible'Length
                 and then Chain'Length = Visible'Length and then Unit_Frames <= Changes'Length (1);
   --  The covariance is the inverse normal equations around the spread of the
   --  gradient (Conley's sandwich), made positive semi-definite (Sandwich). The
   --  spread weights every pair of residual rows by the covariance of their
   --  errors, read from the residuals as a function of the distance between
   --  their points (Errors): a point errs alike in every keyframe, so do points
   --  near each other, and the points of a keyframe err alike. Sightings counted
   --  as independent, or clustered by keyframe, hide the first two and come
   --  out far too sure. It describes random errors: it does not cover a bias
   --  the fit absorbs (the lens's weakest axis, a mix of the focal length and
   --  the distortion terms, came out 2 to 4 sigmas off on A10 and A11:
   --  docs/body-file.md, what the lens's sigmas do not cover).

   procedure Sandwich
     (Inverse, Meat : Driver.Numerics.Arrays.Real_Matrix;
      Covariance    : out Driver.Numerics.Arrays.Real_Matrix;
      Clipped       : out Real;
      Ok            : out Boolean;
      Floor         : Real := 0.0)
     with Pre => Inverse'Length (1) = Inverse'Length (2) and then Meat'Length (1) = Inverse'Length (1)
                 and then Meat'Length (2) = Inverse'Length (1) and then Covariance'Length (1) = Inverse'Length (1)
                 and then Covariance'Length (2) = Inverse'Length (1);
   --  Inverse * Meat * Inverse, the covariance of an estimate whose normal
   --  equations Inverse inverts and whose gradient spreads as Meat, made
   --  positive semi-definite: a spread of estimated covariances (Fit) need not
   --  be, and its negative eigenvalues are set to zero in the frame where
   --  Inverse is the identity, so that the units of the terms do not matter
   --  (Cameron, Gelbach and Miller). Clipped is what that took away: the
   --  negative eigenvalues over the sum of all, in size; 0 when the spread
   --  was positive semi-definite. Floor, in that same frame, is the least any
   --  eigenvalue of the spread may have: the level of the noise itself when
   --  the errors are independent (the Cramer-Rao bound), under which no
   --  estimate of the spread from a few sightings can put a direction. Ok is
   --  False, and Covariance zero, when Inverse is not positive definite.

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

   type Count_Array is array (Positive range <>) of Natural;

   type Sampler is private;
   --  A repeatable generator of samples (Park and Miller's minimal standard, from the same state every time).

   procedure Pick (S : in out Sampler; N : Positive; Into : out Count_Array)
     with Pre => Into'Length <= N;
   --  Into'Length distinct numbers of 1 .. N, the next of the generator's.

   procedure Turn_On_Plane
     (Before, After : Vec3_Array;
      Weight        : Real_Array;
      W             : Vec3;
      Turn          : out Real;
      Shift         : out Vec3;
      Normal        : out Vec3;
      Found         : out Boolean)
     with Pre => Before'First = After'First and then Before'Last = After'Last
                 and then Weight'First = Before'First and then Weight'Last = Before'Last;
   --  A plane's points seen before and after a turn of the eye about W and a shift across it, by their lines of
   --  sight in the eye's frame: X' = R X + T, R = Rot (W, -Turn), T across W, and the plane N . X = 1 in the first
   --  frame, so that a point's line of sight after is (R + T N^T) times its line before. In the frame of W the
   --  homography's last row is (0, 0, 1) and its first two rows are six unknowns, linear in the lines of sight (the
   --  line after crossed with the homography's image of the line before is nought): weighted least squares. The
   --  turn is the one of the two that leave the first two rows less the turn's of rank one (T N^T) whose other
   --  minors the fit leaves the least; Shift and Normal are that rank-one part's factors, Normal of unit length
   --  (the plane's distance is in Shift). A plane's points give the turn where a turn's epipolar lines do not:
   --  they are one plane, and the motion that maps them is told by the plane, not left a family (the desktop
   --  regime's synthetic arm, 10-16: a 0.6 rad sweep's first stage chose 69 px of 400 by its epipolar lines on a
   --  plane of points, 421 when the points' depths were off the plane). Not Found when the points do not determine
   --  the six unknowns.

   function Rayleigh_Median return Real;
   --  The median of the length of a vector of two components that are Gaussian with the same sigma, over that
   --  sigma: the sigma of what a least median of lengths found.

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
   --  The plane most of the points lie on, or a nearer surface that holds about as many
   --  and is not parallel to it, each point judged by its own uncertainty. A plane is
   --  found by consensus over samples of three, each scored by the squares of every
   --  point's residual in units of its sigma, none counted past Z squared; then by
   --  weighted least squares on the inverse depths of the points that lie on it (On),
   --  re-chosen at the residuals' own spread until the choice settles. The plane the
   --  consensus favours is found first, then another among the points it leaves, and so
   --  on until one holds significantly fewer points than the most found
   --  (Regression.Count_Significant of the larger against an even split). Of the planes
   --  found, one replaces the choice only when it is not parallel to it (its normal
   --  beyond Z of the planes' tilt sigmas) and is nearer the eye: the room's wall is as
   --  large as the table in a picture that looks along it, and the top of a thing on
   --  the table is no other table. Covariance is the sandwich
   --  over its points.

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

private

   type Sampler is record
      State : Long_Long_Integer := 1;
   end record;

end Driver.Robot.Kinematics.Fit;
