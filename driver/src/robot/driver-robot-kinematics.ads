--  The evidence the kinematics are fitted from, collected as the body moves.
--
--  For every arm whose eye rides on it (Driver.Robot.Graph), a keyframe is
--  kept whenever the arm and its eye are both at rest and the arm's readings
--  differ from every keyframe kept so far: the readings, and what the eye
--  saw. The first keyframe is the reference. The instrument is asked where
--  the reference's query points went in every later keyframe, both ways
--  round, and the answers are kept with the keyframe they belong to. The
--  query points are the centres of the reference eye's cells that respond
--  to the arm's own push (the lock-in): those show the world, not the hand
--  that rides with the eye.
--
--  Every request is submitted and its reply read on a later beat
--  (Driver.Services), so the same evidence follows from a recording.

private package Driver.Robot.Kinematics is

   procedure Observe (M : in out Model; O : Observation);
   --  One beat, after the readings and the eyes were taken in.

   function Held_Still (M : Model; A : Arm_Id; Beat : Natural) return Boolean;
   --  A keyframe can be taken of the arm at the latest beat: its readings do
   --  not move (Channels.Moving) and its eye's picture has stopped changing
   --  since the body last began to move (Stillness.Eye_Settled, the one stop
   --  rule; a rendered view keeps changing for beats after the camera stops,
   --  and a real camera's exposure does too).

   function Matched (M : Model; A : Arm_Id) return Natural;
   --  How many keyframes of the arm have their matches back.

   function Match_Noise (M : Model; A : Arm_Id) return Real;
   --  How far the matcher errs on the arm's eye: the noise of a round trip,
   --  the robust scale about zero of every answer's return to its query,
   --  both coordinates, in pixels (Refit judges by it which keyframes moved
   --  enough to tell the fit anything); 0 before any answer. The arm's
   --  second keyframe is a still twin of its reference, taken at the same
   --  pose, so the matcher's own error is known before any move.

   function Keyframe_Step (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The smallest change of the arm's channel that moves the view of the eye
   --  it carries by more than a keyframe's match can tell: Z times the larger
   --  of the eye's cell displacement noise and the matcher's (Match_Noise),
   --  over the pixels its view moves per reading unit (Lockin.Shift); 0 while
   --  either is unmeasured. The sweep starts from it, and the fit judges by it
   --  which joints a keyframe moved: a lock-in sees far smaller steps over
   --  many beats than one pair of keyframes shows (A10's arm 2: its reference
   --  keyframe, taken in a push of the recognition rounds, lay 3e-5 rad off
   --  the sweep's base, ten times the lock-in's step, a twentieth of this).

   function Twin_Answered (M : Model; A : Arm_Id) return Boolean;
   --  The arm's still twin was taken and its match answered or refused, or
   --  the instrument can never answer: Match_Noise will not change before
   --  the arm moves.

   function Pending (M : Model) return Natural;
   --  Match requests of every arm not answered yet.

   procedure Refit (M : in out Model);
   --  Fits every arm whose matches changed since its last fit
   --  (Driver.Robot.Kinematics.Fit). Its sightings are the matches whose
   --  round trip comes back to where it started, within the noise of all the
   --  round trips.

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid;
   --  The arm's eye at those readings, in the frame of its eye at the
   --  reference keyframe; the identity until the arm is fitted.

   function Fitted (M : Model; A : Arm_Id) return Boolean;

   function Table (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate;
   --  The table the arm's eye saw, in its reference frame, as the fit that
   --  holds found it (Refit); unknown until then.

   procedure In_World (M : Model; A : Arm_Id; Placement : out Rigid; Scale : out Real; Known : out Boolean);
   --  The arm's reference frame in the world: X_world = Placement * (Scale *
   --  X). The world is the first arm's reference frame (identity, scale
   --  one, once fitted); every other arm is placed by Refit: the first arm's
   --  tracked points, matched into the arm's reference view, are where its
   --  eye stood among them (a resection with the arm's own lens), and the
   --  points both arms tracked give the scale of its lengths.

   function Scale_In_World (M : Model; A : Arm_Id) return Estimate;
   --  In_World's Scale with its sigma; Unknown until the arm is placed.

   procedure World_Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3);
   --  Pose_Covariance carried into the world: the arm's own, turned and
   --  scaled by its placement, with the placement's own (its turn, its
   --  centre, its scale) added; Real'Last on the diagonal until it is placed.

   procedure Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3);
   --  What the fit's own uncertainty (the covariance of the matcher's errors read from
   --  its residuals, Errors) leaves the arm's eye at those readings uncertain by, in the reference frame:
   --  the covariance of its turn (a rotation vector) and of its place
   --  (model units); Real'Last on the diagonal until the arm is fitted.

   procedure Solve_Pose
     (M             : Model;
      A             : Arm_Id;
      Start         : Real_Array;
      Goal          : Rigid;
      Position_Only : Boolean;
      Q             : out Real_Array;
      Position_Off  : out Real;
      Turn_Off      : out Real)
     with Pre => Q'Length = Start'Length;
   --  The readings that bring the arm's eye (in its reference frame) nearest
   --  Goal by the fitted model, within the readings the arm has shown or
   --  beyond them, by damped least squares from Start on the position (model
   --  units) and the turn (radians) left, until a step lowers what is left
   --  by less than the unchanged fraction of it; how far the eye remains
   --  from Goal. Q gives every joint that turns as the reading nearest Start
   --  (Nearest_Readings): the same pose, the way the arm is nearest to go.

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

end Driver.Robot.Kinematics;
