--  The evidence the kinematics are fitted from, collected as the body moves.
--
--  For every arm whose eye rides on it (Driver.Robot.Graph), a keyframe is
--  kept whenever the arm and its eye are both at rest and the arm's readings
--  differ from every keyframe kept so far: the readings, and what the eye
--  saw. The first keyframe is the reference. Driver.Alignment finds where
--  the reference's query points went in every later keyframe (Align_New), each
--  keyframe predicted from the ones before it (Driver.Alignment.Keyframes), and
--  the answers are kept with the keyframe they belong to. The query points are
--  the centres of the reference eye's cells that respond to the arm's own push
--  (the lock-in): those show the world, not the hand that rides with the eye.
--
--  What other eyes show of the reference is asked of the instrument: a request
--  is submitted and its reply read on a later beat (Driver.Services), so the
--  same evidence follows from a recording.

private package Driver.Robot.Kinematics is

   procedure Observe (M : in out Model; O : Observation);
   --  One beat, after the readings and the eyes were taken in.

   procedure Align_New (M : in out Model);

   function Realign (M : in out Model) return Boolean;
   --  Aligns again the keyframes whose first alignment refused points for the uncertainty of the prediction it was
   --  given (a keyframe the arm's earlier keyframes could not carry the prediction to), now that the fitted arm
   --  predicts them: those it predicts with less uncertainty than they were aligned from. True when it changed any.
   --  Aligns every keyframe of every arm that was taken and not aligned yet,
   --  in the order they were taken: where the reference's query points are in
   --  it, with the answer's own uncertainty. Called with the estimates, after
   --  the noise of the readings and the lock-in's shifts are measured and
   --  before Refit; a replay therefore aligns the same keyframes at the same
   --  places of the stream. Nothing is aligned while a channel's noise is not
   --  measured.

   function Held_Still (M : Model; A : Arm_Id; Beat : Natural) return Boolean;
   --  A keyframe can be taken of the arm at the latest beat: its readings do
   --  not move (Channels.Moving) and its eye's picture has stopped changing
   --  since the body last began to move (Stillness.Eye_Settled, the one stop
   --  rule; a rendered view keeps changing for beats after the camera stops,
   --  and a real camera's exposure does too).

   function Matched (M : Model; A : Arm_Id) return Natural;
   --  How many keyframes of the arm have their answers.

   function Match_Noise (M : Model; A : Arm_Id) return Real;
   --  How far the aligner errs on the arm's eye: the scatter of what a smooth
   --  field of displacements leaves of the points it found in the keyframes
   --  the arm moved to (Driver.Alignment.Keyframes.Scatter), in pixels (Refit
   --  judges by it which keyframes moved enough to tell the fit anything); 0
   --  before any such keyframe is aligned. A still twin of the reference
   --  cannot say: the aligner's own error on two identical pictures is far
   --  under its error on pictures that moved.

   function Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base;
   --  The arm's eye: the one the graph mounts on it, the view its keyframes are seen in; 0 when none.

   function Pixels_Per_Unit (M : Model; A : Arm_Id; Channel : Positive) return Real;

   function Shift_Unseen (M : Model; A : Arm_Id; Channel : Positive) return Boolean;
   --  The arm's eye does not see its view move with the channel, as the keyframes measured it: the points that
   --  move with it are a patch of those told (Measure_Shifts), a part of the body in the view and not the view
   --  (a wrist roll behind the eye turns the hand's jaws in its picture). False until the keyframes tell.
   --  How far the view of the eye the arm carries moves per reading unit of
   --  the channel, in pixels: what the arm's aligned keyframes measure of it
   --  (per point, the displacements the aligner found regressed on the
   --  changes of every channel that moved; the median over the points) when
   --  that is significant; else the lock-in's slope (Lockin.Shift) when the
   --  keyframes have measured the channel at all and the slope lies within
   --  what they measured; 0 otherwise. The keyframes' measurement is kept
   --  when the arm's kinematics begins anew (Begin_Anew), and in the body
   --  file.

   function Keyframe_Step (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The smallest change of the arm's channel that moves the view of the eye
   --  it carries by more than a keyframe's match can tell: Z times the larger
   --  of the eye's cell displacement noise and the matcher's (Match_Noise),
   --  over the pixels its view moves per reading unit (Pixels_Per_Unit); 0
   --  while either is unmeasured. The sweep starts from it, and the fit judges
   --  by it which joints a keyframe moved: a lock-in sees far smaller steps
   --  over many beats than one pair of keyframes shows (A10's arm 2: its
   --  reference keyframe, taken in a push of the recognition rounds, lay
   --  3e-5 rad off the sweep's base, ten times the lock-in's step, a
   --  twentieth of this).

   function Step_Of (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The change of the arm's channel from which a keyframe counts as having
   --  moved it, and the fit judges a keyframe's joints by: the keyframe step
   --  when it is measured, and no nearer than the readings tell apart (Z of
   --  the noise of a difference of two readings); else the step the eye can
   --  see at all (Channels.Visible_Step), which the lock-in tells over many
   --  beats; 0 while neither is measured.

   function Counted_Turn (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The change of the arm's channel from the reference from which the fit counts the joint as turned in a
   --  keyframe (Fit.Fit's Visible): the step a keyframe counts from (Step_Of), and no nearer than two landings of
   --  one asked pose can differ, twice the least turn the body is known to execute (Steps.Executed_Least): a
   --  servo lands within its dead band of where it is sent, every time it is sent there, and the reference is
   --  a landing too.

   function Follow_Step (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The change of the arm's channel that moves the view of the eye it
   --  carries by half the aligner's patch at the full size: the largest turn
   --  whose keyframe the aligner follows from the reference by itself
   --  (Followed), over the pixels the view moves per reading unit
   --  (Pixels_Per_Unit); 0 while that is unmeasured. The sweep's ladder
   --  starts here once it is (Motion.Sweep_Start). From the keyframe step it
   --  took a doubling a round, a fit a round, to reach a turn the fit could
   --  use (B0a_1, x5, 10-13: the step 1e-4 reading units, the eye seeing
   --  5.7e-6, four rounds and 1100 beats to turns of 0.0013 units, every fit
   --  of them undetermined: "at most 2.7e-4 rad by that scale").

   function Twin_Answered (M : Model; A : Arm_Id) return Boolean;
   --  The arm's still twin of its reference was taken.

   procedure Asking (M : in out Model; A : Arm_Id; Channels : Channel_Flags);
   --  The channels of the arm the sweep is turning for the keyframes taken from now on; none (an empty array)
   --  when no push of the sweep's is under way. A keyframe carries them (Keyframe.Asked), and the fit tells
   --  which joint a keyframe is of by what the sweep asked, and by its readings only whether the joint turned
   --  by a step its eye sees (Fit.Fit): a servo sent back to the sweep's start lands within its dead band of
   --  it, and by the readings alone every such landing was a turn of that joint (R6's seventh start, 10-14).

   procedure Begin_Anew (M : in out Model; A : Arm_Id);
   --  Drops what the arm's kinematics gathered so far, but what its keyframes
   --  measured of how far the view moves per unit (Pixels_Per_Unit), which
   --  holds at any reference: the next still frame
   --  of its eye is its reference, the one after its twin. The boot calls it
   --  in a held beat where the arm's sweep begins, so that the reference is
   --  the pose the arm is swept from: the fit tells which joints a keyframe
   --  moved by its change from the reference, and the recognition rounds
   --  leave an arm where they found it, not where its first frame was taken
   --  (fake body T6, arm 1: 173 units of channel 2 off, one and a half
   --  steps, so every keyframe of its sweep moved two joints and no joint
   --  but that one had a keyframe of its own; A10's arm 2 lay a twentieth
   --  of a step off and fitted). An arm that carries no eye has nothing to
   --  drop.

   procedure Refit (M : in out Model; Again : Boolean := False);
   --  Fits every arm whose answers changed since its last fit
   --  (Driver.Robot.Kinematics.Fit). Its sightings are the answers of
   --  keyframes the points moved in by more than the aligner errs, whose
   --  uncertainty is not more than Z times the keyframe's typical one.
   --  Again: the keyframes were just aligned again from the fits of this
   --  estimate, and an arm fitted in it is fitted again from that fit (Fit's
   --  From: its last stage alone).

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid;
   --  The arm's eye at those readings, in the frame of its eye at the
   --  reference keyframe; the identity until the arm is fitted.

   function Fitted (M : Model; A : Arm_Id) return Boolean;

   function Contested_Pairs (M : Model; A : Arm_Id) return Joint_Pair_Lists.Vector;
   --  The joints next to each other along the fitted order whose swap leaves residuals not significantly the
   --  larger (Fit_Report's Untold): what the order lacks the evidence of. None when the order is told, or every
   --  such swap is told.

   function Contested_Turns (M : Model; A : Arm_Id) return Real_Vectors.Vector;
   --  Per pair of Contested_Pairs, two: the turns of its Earlier joint and of its Later one, in their readings'
   --  units, that tell its untold orders apart most of those a narrow window follows (Fit_Report's Untold_Turns).

   function Untold_Count (M : Model; A : Arm_Id) return Natural;
   --  How many orders of its joints the arm's fit does not tell from its own (Fit_Report's Untold_Orders); 0 when
   --  told or not fitted.

   function Order_Told (M : Model; A : Arm_Id) return Boolean;
   --  The arm is fitted and its sightings tell its joints' order, not only that it is the best of the orders
   --  (Fit.Fit_Report's Order_Told).

   function Framed (M : Model; A : Arm_Id; From, To : Natural) return Boolean;
   --  A keyframe of the arm was taken at a beat in From .. To at all. A pose
   --  held after a push gives none when it is not fresh: not apart from every
   --  keyframe before it by the step the fit counts as a joint having moved
   --  (Keyframe_Step, re-estimated at every estimate). The boot tells such a
   --  push, too small, from one whose keyframe the aligner could not follow,
   --  too large (B0_2, arm 1: channels 1, 2 and 5 were pushed by the step
   --  known when the sweep began, the step rose before the poses were judged,
   --  no keyframe came of them, and the ladder stopped as if unfollowed).

   function Followed (M : Model; A : Arm_Id; From, To : Natural) return Boolean;
   --  The arm's latest keyframe taken at a beat in From .. To is aligned and
   --  was followed: most of its points were found from windows no wider than
   --  a patch, from the keyframes before it or from the fitted arm's own
   --  prediction (Match_Set.Followed). False without such a keyframe, or
   --  before it is aligned. The boot climbs a joint's ladder of keyframes as
   --  far as they are followed, and names the beats it held the arm for each:
   --  the readings it asked for are not what the keyframe reads (the hold an
   --  arm is moved from and the readings it rests at differ by what the body
   --  gives under load; fake body Z6, arm 2: every push's keyframe found,
   --  none at the readings asked).

   function Table (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate;
   --  The table the arm's eye saw, in its reference frame, as the fit that
   --  holds found it (Refit); unknown until then.

   function Standing_Near (M : Model; A : Arm_Id; Point : Vec3; Within : Real) return Natural;
   --  How many of the arm's tracks stand above its table within Within of Point across the table's normal, in
   --  the arm's frame: tracks with a depth (Track_Known) that do not lie on the table and whose height above it
   --  is significant against the height's own sigma (the track's depth and the plane's there). A press lands on
   --  what stands there, not on the table (A79's hand 1: five presses of eleven never met the table, item 7). 0
   --  without a fit or a table.

   procedure Clear_Spot (M : Model; A : Arm_Id; Near : Vec3; Within : Real; Spot : out Vec3; Found : out Boolean);
   --  Of the arm's tracks that lie on its table (Table_On), the one nearest Near across the table's normal with no
   --  track standing above the table within Within of it (Standing_Near): a place where a press meets the table and
   --  nothing on it. Found is False without a fit, a table, or such a track.

   procedure In_World (M : Model; A : Arm_Id; Placement : out Rigid; Scale : out Real; Known : out Boolean);
   --  The arm's reference frame in the world: X_world = Placement * (Scale *
   --  X). The world is the first arm's reference frame (identity, scale
   --  one, once fitted); every other arm is placed by Refit: the first arm's
   --  tracked points, matched into the arm's reference view, are where its
   --  eye stood among them (a resection with the arm's own lens), and the
   --  points both arms tracked give the scale of its lengths.

   function Scale_In_World (M : Model; A : Arm_Id) return Estimate;
   --  In_World's Scale with its sigma; Unknown until the arm is placed.

   procedure Placement_Covariance (M : Model; A : Arm_Id; Turn, Centre : out Mat3);
   --  The covariance of the arm's placement in the world: of its turn (a rotation vector, world frame) and of its
   --  centre, in world units; Real'Last on the diagonals until it is placed.

   function World_Table (M : Model) return Driver.Geometry.Plane_Estimate;
   --  The table in the world: the first arm's (the world is its reference frame, In_World), as its eye saw it;
   --  unknown until that arm is fitted with a table. An arm whose own eye saw none presses on this one, turned
   --  into its frame by its placement (Driver.Robot.Table_In_Arm, item 14).

   procedure World_Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3);
   --  Pose_Covariance carried into the world: the arm's own, turned and
   --  scaled by its placement, with the placement's own (its turn, its
   --  centre, its scale) added; Real'Last on the diagonal until it is placed.

   procedure Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3);
   --  What the fit's own uncertainty (the covariance of the matcher's errors read from
   --  its residuals, Errors) leaves the arm's eye at those readings uncertain by, in the reference frame:
   --  the covariance of its turn (a rotation vector) and of its place
   --  (model units); Real'Last on the diagonal until the arm is fitted.

   function Step_Covariance (M : Model; A : Arm_Id; From, To : Real_Array) return Mat3;
   --  What the fit's own uncertainty leaves the arm's eye's displacement from readings From to readings To uncertain
   --  by, in the reference frame (Fit.Step_Covariance); Real'Last on the diagonal until the arm is fitted.

   function Place_Covariance_In_Unit (M : Model; A : Arm_Id; Readings : Real_Array) return Mat3;
   --  The covariance of the arm's eye's place at those readings in the arm's own unit (Fit.Place_Covariance_In_Unit:
   --  the part every place shares with the unit taken out); Real'Last on the diagonal until the arm is fitted.

   procedure Solve_Pose
     (M             : Model;
      A             : Arm_Id;
      Start         : Real_Array;
      Goal          : Rigid;
      Position_Only : Boolean;
      Q             : out Real_Array;
      Position_Off  : out Real;
      Turn_Off      : out Real;
      Free_About    : Vec3 := [0.0, 0.0, 0.0])
     with Pre => Q'Length = Start'Length;
   --  The readings that bring the arm's eye (in its reference frame) nearest
   --  Goal by the fitted model, within the readings the arm has shown or
   --  beyond them, by damped least squares from Start on the position (model
   --  units) and the turn (radians) left, until a step lowers what is left
   --  by less than the unchanged fraction of it; how far the eye remains
   --  from Goal. Q gives every joint that turns as the reading nearest Start
   --  (Nearest_Readings): the same pose, the way the arm is nearest to go.
   --  With Free_About (a direction of the eye's frame, not nought) the turn left is how far that direction points
   --  from where Goal points it (the sine of the angle between them); the eye's turn about it is Goal's where the
   --  arm reaches the whole of Goal within what its fit tells, and that of the nearest pose pointing it elsewhere.

   function Conditioning (M : Model; A : Arm_Id; Q : Real_Array) return Real;
   --  How near the arm stands at readings Q to a pose where its eye cannot move every way: the least singular value
   --  of the fitted model's Jacobian of the eye's place (model units) and turn (radians) by the readings, over the
   --  greatest; 0 where some way of the eye is lost, 1 where every way is as easy. Zero for a model not fitted.

   function Easiest_Way (M : Model; A : Arm_Id; Q : Real_Array) return Vec3;
   --  The unit direction, in the arm's reference frame, in which the fitted model at readings Q moves the eye's place
   --  the furthest for a turn of its joints of a given size: the leading eigenvector of J J', J the Jacobian of the
   --  place by each joint's own turn (its reading's change times the joint's radians, or units of slide, per reading
   --  unit), so the readings' units do not weigh the ways. Its sign is the eigenvector's; zero for a model not fitted.

   function Nearest_Readings (M : Model; A : Arm_Id; Near, Readings : Real_Array) return Real_Array
     with Pre => Near'Length = Readings'Length,
          Post => Nearest_Readings'Result'Length = Readings'Length;
   --  Readings that put the arm's eye where Readings put it, each joint that
   --  turns given as the reading nearest Near. A joint that turns repeats its
   --  pose over a turn of its axis, which the fit reads as a period of
   --  readings: two pi over the scale it found for the joint, in the reading's
   --  own units, which is no more two pi than the reading is radians. A joint
   --  that slides has none, and a model that is not the arm's leaves
   --  Readings as they are.

   function Angle_Sigma (M : Model; A : Arm_Id) return Real;
   --  The angle one pixel of the fit's measured noise subtends at the arm's
   --  eye: the uncertainty of a line of sight, and of the eye's turn.

   function Ray_In_Eye (M : Model; A : Arm_Id; U, V : Real) return Vec3;
   --  The unit line of sight through pixel (U, V) of the arm's eye.

   procedure Project_In_Eye (M : Model; A : Arm_Id; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the arm's eye frame lands in its image.

   --  An eye that stands still in the world (the graph mounts it World_Fixed) has its lens and its place in the
   --  world measured from the first arm's tracked points and the answers the eye gave to where they are
   --  (Kinematics.Fixed), by Refit, once the arm is fitted and the eye's set of answers is there, and again when
   --  either changed. The world is that arm's reference frame, in its unit.

   function Fixed_Known (M : Model; E : Eye_Id) return Boolean;
   --  The fixed eye's lens and place are determined (Fixed.Fit_Report.Determined).

   function Fixed_Why (M : Model; E : Eye_Id) return String;
   --  What the evidence leaves out of the fixed eye when it is not determined; empty when it is.

   function Fixed_Pose (M : Model; E : Eye_Id) return Pose_Estimate;
   --  The fixed eye's camera frame in the world with its covariance (the centre's, and the turn's carried into
   --  world axes); unknown (covariances Real'Last) until determined.

   function Fixed_Ray_In_Eye (M : Model; E : Eye_Id; U, V : Real) return Vec3;
   --  The unit line of sight through pixel (U, V) of the fixed eye, in its frame.

   procedure Fixed_Project_In_Eye (M : Model; E : Eye_Id; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the fixed eye's frame lands in its image.

   function Fixed_Angle_Sigma (M : Model; E : Eye_Id) return Real;
   --  The angle one pixel of the noise its answers showed subtends at the fixed eye; Real'Last until determined.

   function Fixed_Line_Sigma (M : Model; E : Eye_Id; U, V : Real; In_World : Boolean) return Real;
   --  How uncertain, as an angle, the line of sight through the pixel is: Fixed_Angle_Sigma with what the
   --  covariance of the fit leaves the direction uncertain by at that pixel, the lens's terms alone (in the eye's
   --  frame) or, In_World, together with the turn, their correlations kept: a line through a pixel the points
   --  pin is far surer than the principal point and the turn are each. Real'Last until determined.

end Driver.Robot.Kinematics;
