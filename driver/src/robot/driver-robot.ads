--  Layer 2: the robot's body, measured from its own readings, commands and
--  images.
--
--  Nothing here is declared by a user. Boot (Driver.Robot.Boot) measures the
--  body from zero, or reloads what an earlier boot measured and re-measures
--  what changed; Observe then refines the estimates every beat. Every
--  quantity carries its uncertainty.
--
--  Frames and units. The world frame is fixed to the scene; how boot anchors
--  it is documented in Driver.Robot.Boot. Lengths are in the body's own unit,
--  fixed at boot; no layer may assume metres or degrees, and nothing may
--  assume that +z is up (use Up). Camera frames have z along the optical
--  axis, x along +U and y along +V of the image (Driver.Images).
--
--  Estimation is separate from decision: Observe is fed every beat with the
--  observation and the command in effect, whoever chose it, so the same
--  estimates follow from a recording of any driver. The heavier estimates
--  (roles, kinematics) are recomputed by Observe whenever the evidence
--  behind them has doubled, and a decider may ask for them at once with
--  Estimate_Now; the main program has them computed apart from its loop
--  (Compute_Apart).
--
--  Before a quantity is measured it reads as an unmeasured body reports it,
--  never as an exception, so callers can ask from the first beat: role
--  Unclassified, mount and response Unmeasured, zero arms, a reading noise
--  of Real'Last, an image lag of 0, unknown estimates (Driver.Uncertain:
--  Unknown, or the infinite covariances of an estimate's defaults) for every
--  pose, ray, point, distance and Visible_Step, an empty Self_Mask of the
--  image's size, Still and Blocked False, Closer_Arm and Carrier_Group 0.
--
--  Ownership: path A owns this layer except Driver.Robot.Hand (path B).
--  Upper layers use only what this specification and Driver.Robot.Motion and
--  Driver.Robot.Hand export.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Commands;
with Driver.Geometry;
with Driver.Images;
with Driver.Numerics;
with Driver.Observations;
private with Driver.Alignment.Keyframes;
private with Driver.Pixels;
private with Driver.Rig;
with Driver.Uncertain;

with Driver.Bytes;
with Driver.Recording;
package Driver.Robot is

   use Driver.Numerics;
   use Driver.Uncertain;

   subtype Group_Id is Driver.Observations.Group_Id;
   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Observation is Driver.Observations.Observation;

   type Group_Role is (Unclassified, Arm, Carrier, Closer, Part, Sensor, Inert);
   --  Measured by pushing each commandable group (docs/body-protocol.md, 3):
   --    Arm      moving it moves the whole image of the eyes it carries, not of all
   --    Carrier  moving it moves the whole image of every eye (base, torso, drone)
   --    Closer   moving it moves a patch inside an arm's own eye (fingers, gripper)
   --    Part     moving it moves a patch outside every arm's eye
   --    Sensor   its readings change though it is not commanded
   --    Inert    nothing it is pushed to changes anything any eye sees, or,
   --             not commandable, its readings never change
   --  With a single eye nothing tells carrying the whole body apart from
   --  carrying that eye, so a group moving it is an Arm.

   type Arm_Id is new Positive;

   type Mount_Kind is (Unmeasured, World_Fixed, Arm_Carried, Carrier_Carried);

   type Mount (Kind : Mount_Kind := Unmeasured) is record
      case Kind is
         when Arm_Carried => Arm : Arm_Id;
         when others      => null;
      end case;
   end record;

   type Model is tagged limited private;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command);
   --  One beat: the observation, and Sent, the last command sent to the
   --  robot before it arrived (the one in effect while it was captured;
   --  holds included). Estimators only; never sends anything.

   procedure Estimate_Now (M : in out Model);

   procedure Replay_Sweep (M : in out Model; Kind : Driver.Recording.Record_Kind; Data : Driver.Bytes.Byte_Array);
   --  Applies the boot's word to the kinematics as the run said it (a recording's kind G, "arm N", or Q,
   --  "arm N asked 0 1 ...": Kinematics.Begin_Anew, Kinematics.Asking); anything else is ignored.

   procedure Replay_Note (M : in out Model; Data : Driver.Bytes.Byte_Array);
   --  Makes a decider's note into the model as the run made it (a recording's kind N): "keeps G C" and a 0 or 1 an eye
   --  (Note_Keeps_Going), "back G C" (Note_Comes_Back), "origins" (Motion.Mark_Origins), "stayed G W" with W 1 or -1
   --  (Motion.Note_Stayed), "alone G" (Note_Channels_Alone); anything else is ignored. A replay of the estimators alone
   --  derives the graph from the notes as the run did.
   --  Recomputes the heavier estimates from everything observed so far.
   --  Deciders call it between Driver.Beats.Next and Send; it can take
   --  minutes, during which the robot holds. The call goes into the
   --  recording (Driver.Recording, kind E), so a replay recomputes where
   --  the run did. When the estimates are computed apart (Compute_Apart),
   --  it marks them due and waits in Driver.Beats.Wait_For_Estimates: it
   --  returns in a later held beat, with them in the model, so a beat number
   --  or an observation read before the call is no longer the latest after it.

   procedure Compute_Apart (M : in out Model);
   --  From now on Observe and Estimate_Now do not compute the heavier
   --  estimates but mark them due (Estimates_Due), for whoever holds the
   --  model to compute them (Compute_Estimates): the main program, through
   --  Driver.Apart, gives the model to a task of its own for that and answers
   --  the robot every beat meanwhile, and a replay computes them in place, at
   --  the same point of the stream.

   function Estimates_Due (M : Model) return Boolean;
   --  The heavier estimates are due: the evidence behind them has doubled,
   --  or a decider asked (Estimate_Now). Always False unless computed apart.

   procedure Compute_Estimates (M : in out Model);
   --  Computes the heavier estimates now, due or not, and they are no longer
   --  due. The caller holds the model alone meanwhile.

   function Booted (M : Model) return Boolean;
   --  The kinematics of every arm that carries an eye are measured.

   --  What a body file holds (docs/body-file.md), quantity by quantity:
   --  the readings' noise, the readings each channel has moved through, the
   --  step responses, the image lags, what each
   --  group's push does to each eye (the lock-in), the body's graph (roles,
   --  arms, mounts) and the kinematics with their lenses.
   type Stored is
     (Stored_Noise, Stored_Travel, Stored_Ends, Stored_Steps, Stored_Lags, Stored_Responses, Stored_Graph, Stored_Shifts,
      Stored_Kinematics);

   procedure Load_Body
     (M   : in out Model;
      Path : String;
      Ok   : out Boolean;
      Why  : out Ada.Strings.Unbounded.Unbounded_String);
   --  Reloads from a body file every quantity whose method version is the
   --  code's and whose inputs were reloaded too; the others stay to be
   --  measured again. A model with no groups yet takes the file's; one that
   --  has them must match the file's key, or nothing is reloaded. Ok is
   --  False when the file cannot be read or is not a body file of this
   --  body; Why says what was reloaded and what is to be measured again.
   --  The text read goes into the recording (Driver.Recording, kind F), so a
   --  replay of the run reloads it where the run read it.

   procedure Load_Body_Text
     (M    : in out Model;
      Text : String;
      Ok   : out Boolean;
      Why  : out Ada.Strings.Unbounded.Unbounded_String);
   --  Load_Body from the text of a body file: what a replay calls where the
   --  recording shows the run read its body file.

   function Reloaded (M : Model; Q : Stored) return Boolean;
   --  Q came from a body file and stands: the estimators do not measure it
   --  again in this session (the travel and the step responses go on
   --  accumulating from it).

   function Role (M : Model; G : Group_Id) return Group_Role;

   function Arm_Count (M : Model) return Natural;
   function Arm_Group (M : Model; A : Arm_Id) return Group_Id
     with Pre => Natural (A) <= Arm_Count (M);

   function Eye_Count (M : Model) return Natural;
   function Eye_Mount (M : Model; E : Eye_Id) return Mount;

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate;
   --  The camera frame in the world frame at the beat of O.

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean);
   --  Where a world point appears in the eye; Visible is False when it falls
   --  behind the eye or outside the image.

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate;
   --  The line of sight through a pixel, in the world frame.

   function Ray_In_Eye (M : Model; E : Eye_Id; U, V : Real) return Vec3;
   --  The unit line of sight through a pixel of the eye, in its own frame, by its measured lens: an arm's eye's
   --  fitted with the arm, a fixed eye's fitted where it was placed; zero when the lens is not measured.

   procedure Project_In_Eye (M : Model; E : Eye_Id; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the eye's own frame lands in its picture by its measured lens, inside the picture or not; not
   --  In_Front behind the eye or when the lens is not measured.

   function Eye_Noise (M : Model; E : Eye_Id) return Real;
   --  How far the aligner errs on the eye's pictures, in pixels: what the keyframes of the arm it rides on measured
   --  (Kinematics.Match_Noise), or what its placing left of its sightings (a fixed eye); 0 when not measured.

   function Sweep_Views (M : Model; E : Eye_Id) return Natural;
   --  How many still pictures of eye E the sweep of the arm that carries it kept (Kinematics: the arm's keyframes,
   --  the first its reference), once the arm is fitted; 0 before, and for an eye no arm carries.

   function Sweep_View (M : Model; E : Eye_Id; K : Positive; Like : Observation) return Observation
     with Pre => K <= Sweep_Views (M, E) and then Natural (E) <= Natural (Like.Images.Length)
                 and then Natural (Like.Readings.Length) = Group_Count (M);
   --  The K-th, as an observation: Like's, with the arm's readings and eye E's picture as they were then, and no
   --  other picture.

   function Eye_Ray (M : Model; E : Eye_Id; Px : Driver.Images.Pixel) return Ray_Estimate;
   --  The same line of sight in the eye's own frame (from its centre of
   --  projection), carrying only the lens's uncertainty: with Eye_In_Tool it
   --  takes a pixel into the tool frame without the arm's kinematics.

   function Up (M : Model) return Direction_Estimate;
   --  Away from gravity, in the world frame, as measured.

   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate;
   --  The last link of the arm in the world frame at the beat of O.

   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate
     with Pre => Eye_Mount (M, E).Kind = Arm_Carried;
   --  The eye's frame in its arm's tool frame at the beat of O, carrying only
   --  the uncertainty of what lies between them (the mount, and any joints
   --  between the last link and the eye), so a measurement made in the eye is
   --  taken into the tool frame without counting the arm's kinematics twice.
   --  Constant when the eye rides on the last link.

   --  Each arm's own frame: the frame its eye had at its reference keyframe,
   --  held by the arm's base, in the arm's own unit; its kinematics are
   --  fitted there. What is measured of one arm and its hand alone (a
   --  fingertip, the surface it pressed) is measured there, free of the
   --  placement that takes the arm into the world: the world is the first
   --  arm's frame, and every other arm stands in it at its placement,
   --  X_world = Placement * (Arm_Unit * X_arm), which only what spans arms
   --  needs.

   function Tool_In_Arm (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate;
   --  The last link of the arm at the beat of O in the arm's own frame,
   --  uncertain only by the arm's own fit; known once the arm is fitted,
   --  placed in the world or not. Tool_Pose is this, placed.

   function Tool_Step_In_Arm (M : Model; A : Arm_Id; From, To : Real_Array) return Point_Estimate;
   --  How far the last link moved from the arm's readings From to the readings To, in the arm's own frame, uncertain
   --  by the arm's fit as a displacement (Kinematics.Step_Covariance), not as two places; unknown until the arm is
   --  fitted.

   function Up_In_Arm (M : Model; A : Arm_Id) return Direction_Estimate;
   --  Away from gravity in the arm's own frame: the normal of the table its
   --  eye saw (Table_In_Arm), its sigma the tilt's along the direction it is
   --  least sure of. Up is the first arm's.

   function Table_In_Arm (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate;

   function World_Table (M : Model) return Driver.Geometry.Plane_Estimate;
   --  The table in the world: the first arm's, as its eye saw it (the world is that arm's frame); its normal
   --  towards what rests on it. Not Known until that arm is fitted with a table.
   --  The table the arm's eye saw at its reference keyframe, in the arm's own
   --  frame, its normal towards that eye: the plane most of the eye's tracked
   --  points lie on, uncertain by their scatter about it, by what the fit
   --  moves every depth by together, and by the lens's lines of sight. Not
   --  Known until the arm is fitted and its table found.

   function Arm_Unit (M : Model; A : Arm_Id) return Estimate;
   --  The world length of the arm frame's unit: a length L measured in the
   --  arm's frame is Arm_Unit * L in the world. One, exactly, for the first
   --  arm; Unknown until the arm is placed.

   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean;
   --  At the beat of O the arm was commanded further than it went, by a step
   --  its eye can see (by more than its free motion falls short, for joints
   --  no eye watches): the estimators' view of the judgment
   --  Driver.Robot.Motion.Step reports, so a replay sees the blocked beats of
   --  the run it replays.

   function Stopped_By_An_End (M : Model; A : Arm_Id; O : Observation) return Boolean;
   --  At the beat of O the arm's push in effect, ended blocked at rest, was stopped by a joint's end the body has
   --  shown: the channel that stopped it (Steps.Limiter) stands, where the push ended, at or past the end it was
   --  asked towards (End_Of), or short of it by no more than the noise of two readings. Such a stop is the body's
   --  own and no contact: the hand makes no press of it and waits for none (A79's hand 1: five presses of eleven
   --  never met the table, item 7). False while that end is not known: the first stop at an end is told from a
   --  contact only by the pose it is found again from (Note_Stopped).

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask;
   --  The pixels of the eye that show the robot itself at the beat of O.

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate;
   --  The distance from a world point to the nearest surface of the body.

   function Still (M : Model) return Boolean;
   --  The one stillness judgment: at the latest beat no group and no eye
   --  changes significantly against its own measured noise.

   --  What the body reported and how each group behaves.

   type Channel_Flags is array (Positive range <>) of Boolean;   --  per channel of a group

   function Group_Count (M : Model) return Natural;
   function Group_Size (M : Model; G : Group_Id) return Natural;
   function Is_Commandable (M : Model; G : Group_Id) return Boolean;
   --  A command in effect has carried a target for it.

   procedure Note_Rest_Commands (M : in out Model);
   --  Every commandable channel's command as the body stands still before any push: its echoed target at the latest
   --  beat, else its reading. The command that holds the body still (Motion.Hold_Of, for a channel that keeps it going).

   function Rest_Command (M : Model; G : Group_Id; Channel : Positive) return Real;
   --  The channel's rest command (Note_Rest_Commands); its reading at the latest beat before one was noted.

   procedure Note_Keeps_Going (M : in out Model; G : Group_Id; Channel : Positive; Eyes : Channel_Flags);
   --  The channel, held away from its rest command, kept the body going: Eyes (one a Boolean per eye, from 1) the eyes
   --  whose pictures drifted then (Motion.Settle).

   function Keeps_Going (M : Model; G : Group_Id; Channel : Positive) return Boolean;
   function Keeps_Going (M : Model; G : Group_Id) return Boolean;
   --  The channel (some channel of the group) kept the body going while held (Note_Keeps_Going): a speed, not a place.

   function Kept_Eye (M : Model; G : Group_Id; E : Eye_Id) return Boolean;
   --  The eye's picture drifted while a channel of the group kept the body going.

   procedure Note_Channels_Alone (M : in out Model; G : Group_Id);
   --  The group's channels are pushed one at a time (Group_Stream.Channels_Alone).

   procedure Note_Comes_Back (M : in out Model; G : Group_Id; Channel : Positive);
   function Comes_Back (M : Model; G : Group_Id; Channel : Positive) return Boolean;
   --  A push of the channel, held and sent back, came back with the eyes' pictures (Motion.Settle): a joint.

   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real;
   --  The standard deviation of the channel's reading at rest, in reading
   --  units; zero for a reading that repeats exactly.

   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate;
   --  The smallest change of the channel's command whose effect the eyes that
   --  see it tell from their own noise, in reading units: where a probe or a
   --  sweep of the channel starts. It is also a change the channel's own
   --  reading tells from its noise: no smaller than Z sigmas of the change of
   --  two readings (a lock-in that credits a group with the pictures' motion
   --  beside its tiny readings can fit a step of 1e-17, which no reading can
   --  show). Unknown while the channel's noise is not measured, since nothing
   --  tells a step from it.

   type Sense is (Increasing, Decreasing);
   --  The way a channel's reading is asked to go.

   function End_Of (M : Model; G : Group_Id; Channel : Positive; S : Sense) return Estimate;
   --  The reading beyond which the channel was found not to follow when asked further that way: where pushes that
   --  the body itself stopped (Driver.Robot.Motion.Note_Stopped) left it at the same reading from two poses of the
   --  other channels, the furthest of them, and no nearer than the readings the channel has been seen at; its sigma
   --  is the channel's reading noise. A joint's end is a property of that joint, which no pose of the others
   --  changes; a stop that one pose explains (the arm on itself, the palm on the table) is a contact, noted
   --  until another pose shows the same reading, and plans stay free past the readings seen meanwhile. Unknown
   --  until the channel has shown an end, or while its noise is not measured, and Driver.Robot.Motion.Plan_Reach
   --  refuses a path past an end. An end is the extreme ever READ in that sense where the arm has read beyond it,
   --  here or in the runs the body file kept: only a reading made by something other than Plan_Reach, a command
   --  given directly or the boot's own steps, can widen it.

   type Eye_Response is (Unmeasured, Nothing, Patch, Undecided, Whole);
   --  What pushing a group does to what an eye sees: nothing, a patch of the
   --  image moves, or the whole image moves (the eye rides on the group).

   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response;

   function Responding (M : Model; G : Group_Id; E : Eye_Id) return Natural;
   --  How many of the eye's textured cells the verdict found responding to
   --  the group's push.

   function Resting_Motion (M : Model; G : Group_Id; E : Eye_Id) return Real;
   function Rest_Pull (M : Model; G : Group_Id; E : Eye_Id) return Real;
   --  How significantly the eye's cells that do not respond to the group are pulled along its pushes together
   --  (Eye_Effect.Rest_Pull): a far part of the view moved by less than any one cell can tell.
   --  How many sigmas the cells that did not respond to the group's push move together: the inverse-variance sum
   --  of their displacement energies over the root of the sum of their weights. Each cell's energy is an unbiased
   --  estimate of the motion it shows, however little (a cell that moves a tenth of what it takes to respond
   --  does not respond, and its energy is still a tenth), so a view whose far part moves less than any cell can
   --  tell, but moves, shows it here when it shows it nowhere else. About a standard normal when none moves;
   --  zero while there is no verdict.

   function Textured_Cells (M : Model; G : Group_Id; E : Eye_Id) return Natural;
   --  How many cells of the eye the verdict was reached over: those with the
   --  texture to show a displacement. Zero while there is no verdict.

   function Image_Lag (M : Model; E : Eye_Id) return Integer;

   function Eye_Watch (M : Model; E : Eye_Id) return String;
   --  For a replay that follows an eye beat by beat: its picture's mean change
   --  against the frame before and the one before that (each pixel in units of
   --  its noise), whether it is still and whether it has settled, and the
   --  largest displacement of a resolved cell at this beat, in pixels and in
   --  units of that cell's noise.

   function Group_Watch (M : Model; G : Group_Id) return String;
   --  For a replay that follows a group beat by beat: its readings at the
   --  latest beat, each channel's change from the beat before in units of
   --  its noise, whether the group moved (Channels.Moving) and whether it
   --  is being pushed, and its target where one is in effect.
   --  How many beats the eye's images trail the readings they belong to:
   --  the image of beat B shows the body as read at beat B - Image_Lag; 0
   --  until measured.

   function Longest_Settling (M : Model; E : Eye_Id) return Natural;
   --  The most beats the eye's picture has taken to settle after the body
   --  began to move, over the whole stream (Began_Moving to the eye's next
   --  Settled_At); 0 before any move has settled. What a wait for the eye's
   --  picture may be measured against: the hand's sweep waited for a push's
   --  view only as long as the longest view before it had taken, and B0_2's
   --  arm 1 closer, whose first view formed in one beat, was given one beat
   --  for the next and left unswept.

   function Lag_Known (M : Model; E : Eye_Id) return Boolean;
   --  The lag stood out of every shift tried: the eye's motion follows some
   --  push at that delay.

   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base;
   --  For a Closer, the arm in whose eye it moves; 0 otherwise.

   function Carrier_Group (M : Model) return Group_Id'Base;
   --  The group that carries every eye, or 0 when there is none.

   function Contract_Breach (M : Model; G : Group_Id) return Natural;
   --  The clause of the porting contract (docs/body-protocol.md, 3) the
   --  group breaks: 1 its reading does not follow its command, 2 nothing
   --  any eye sees changes when it moves, 3 it reports moving where an eye
   --  sees nothing move although the opposite push showed; 0 none.

   function Describe (M : Model) return String;
   --  The measured body in a few lines, for the log.

private



   type Stored_Flags is array (Stored) of Boolean;

   --  The streams are read element by element in every estimator's inner
   --  loops; a tampering check makes each read a controlled reference (built,
   --  finalized, abort deferred and undeferred: a tenth of the driver's time
   --  over A37's first 2324 beats, Visible_Step alone reading 575 million
   --  elements). These instances keep their index checks and drop that one.
   pragma Suppress (Tampering_Check);
   package Real_Vectors is new Ada.Containers.Vectors (Natural, Real);
   package Flag_Vectors is new Ada.Containers.Vectors (Natural, Boolean);
   type Luma_Access is access Real_Array;
   --  A frame's luma (Driver.Images.Luma), on the heap: two per eye, swapped
   --  every beat, never on a stack.
   package Count_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   package Rigid_Vectors is new Ada.Containers.Vectors (Natural, Driver.Numerics.Rigid);

   type Joint_Pair is record
      Earlier, Later : Positive;   --  two joints by their readings, Earlier before Later along the arm as fitted
   end record;
   package Joint_Pair_Lists is new Ada.Containers.Vectors (Positive, Joint_Pair);

   --  The extreme a quantity of a push has reached (Driver.Robot.Steps.Follow), and the quantity at the beats since,
   --  that one included: their count, mean and the sum of the squares of their deviations from it (Welford's).
   type Extreme is record
      Value    : Real := 0.0;
      Followed : Natural := 0;
      Mean     : Real := 0.0;
      Spread   : Real := 0.0;
      Last     : Real := 0.0;       --  the quantity at the latest beat counted
      Turned   : Boolean := False;  --  since the extreme it went back the way it had come at some beat, by a step the
                                    --  one test of motion sees: the group moves about (Driver.Robot.Steps)
   end record;

   --  One push of a group and how it went (Driver.Robot.Steps).
   type Episode is record
      Start     : Natural := 0;       --  the beat its target changed
      Moved     : Boolean := False;
      Moved_At  : Natural := 0;       --  the first beat its reading moved
      Born_Moving : Boolean := False; --  its group was still moving from the push before at the beat it started: its
                                      --  first motion is that push's answer, not its own, and tells nothing of the delay
      Ended     : Boolean := False;
      End_At    : Natural := 0;       --  the first still beat, or where it was given up or cut short
      Settled   : Boolean := False;   --  judged: not cut short by the next push
      Rested    : Boolean := False;   --  ended with its readings still (not given up while they kept moving)
      Progress_At : Natural := 0;     --  the latest beat its group came nearer its target or farther from where it began
                                      --  than it had been, in the channels it asked a step of, by a step the one test
                                      --  of motion sees and the group's own chatter does not make
      Nearest   : Extreme;            --  how near its target the group has come, in reading units
      Farthest  : Extreme;            --  how far from where it began the group has gone, in reading units
      Length    : Real := 0.0;        --  how far it asked, in reading units
      Shortfall : Estimate;           --  how far short of the target it stopped, along the ask
      Delivered : Estimate;           --  the fraction of the ask it delivered, along the ask
      Blocked   : Boolean := False;   --  it fell short by more than the group's free pushes do, and by a shortfall the one
                                      --  test of motion sees, or nothing answered an ask that test would see
   end record;

   package Episode_Vectors is new Ada.Containers.Vectors (Positive, Episode);

   --  A stop the body made of a push (Driver.Robot.Motion.Note_Stopped) that is not yet an end: one value of one
   --  channel, in one pose of the group. It is an end when the channel stops at the same reading from another pose.
   type Stop_Record is record
      Channel : Positive := 1;
      Up      : Boolean := False;
      Value   : Real := 0.0;
      Pose    : Real_Vectors.Vector;   --  the group's readings where it stopped
   end record;

   package Stop_Vectors is new Ada.Containers.Vectors (Positive, Stop_Record);
   --  Stops are kept for the groups of an arm (Driver.Robot.Motion.Note_Stopped): a group of one channel, a closer's,
   --  has no other channel to stand elsewhere, so none of its stops is found again and End_Of stays Unknown for it.
   --  The readings' noise is the scale of two readings of a channel here, and the channel's travel the scale of the
   --  chance that two contacts meet at one reading; nothing but the stream sets either. The travel is the range seen so far, so
   --  early in a boot two stops of one soft limit are further apart than D and are kept, and agree once the range
   --  has grown. A boot that moves one joint at a
   --  time from the rest pose confirms no end by itself, its stops share one pose: an end is found by a later stop
   --  from another configuration (presses, tilts, a task), and Unknown after the boot is not a fault.

   --  A group's readings and the target in effect, beat after beat; beat K
   --  of the stream is the K-th observation (counted from zero).
   type Group_Stream is record
      Size        : Natural := 0;
      Commandable : Boolean := False;
      Held        : Real_Vectors.Vector;   --  per channel: the pose a step last sent the group to, as a reading
                                           --  (Motion.Step; the command that lands it there may differ by what
                                           --  the servo stands short of its goal); empty before any step
      Landing     : Real_Vectors.Vector;   --  per channel: the command less the reading where the group last
                                           --  landed on a step's target (the servo's steady error under its
                                           --  load, Motion.Step); a step's first sending adds it; empty before
      Values      : Real_Vectors.Vector;   --  Size readings per beat
      Twin_Values : Real_Vectors.Vector;   --  Size values per beat of the other group of its command key's pair (the robot's
                                           --  echo of its command, Driver.Twins), kept to turn Values round with
      Precisions  : Count_Vectors.Vector;  --  per beat: the significant bits of the float format the readings came in over
                                           --  the wire, 0 when they came as integers (the last one's at a beat without)
      Twin_Precisions : Count_Vectors.Vector;   --  the same for Twin_Values
      Present     : Flag_Vectors.Vector;   --  a reading arrived that beat
      Targets     : Real_Vectors.Vector;   --  Size targets per beat
      Targeted    : Flag_Vectors.Vector;   --  the command in effect carried a target
      Pushed      : Flag_Vectors.Vector;   --  per beat, once measured: moving because it was pushed
      Delay_Beats : Natural := 0;          --  the longest wait from a push to its first motion
      Delay_Known : Boolean := False;      --  some push was answered
      Own_Push    : Boolean := False;
      --  some push of it was its own: answered, and no other commandable group began one within the longest lag the
      --  stream could tell of then (Graph.Derive)
      Rider       : Eye_Id'Base := 0;
      --  the eye last mounted on the group as an arm (Graph.Derive): it rides on it while it shows the group a patch
      --  of its view or is undecided about it
      Closer_Eye  : Eye_Id'Base := 0;
      --  the eye on an arm that last showed the group a closer of that arm (Graph.Derive): it does while it tells
      --  nothing against it (not measured, undecided, a patch)
      Closer_Rivals : Flag_Vectors.Vector;
      --  per eye (from 0): it rode an arm when Closer_Eye was chosen, and was weighed against it (Graph.Derive)
      Channels_Alone : Boolean := False;
      --  its channels were pushed one at a time (Boot): the lock-in's verdicts on each tell them apart, and the graph
      --  may part the group by the eyes they move (Graph.Derive)
      Episodes    : Episode_Vectors.Vector;   --  every push and how it went (Driver.Robot.Steps)
      From, Ask   : Real_Vectors.Vector;   --  of the push under way: the readings before it, and target minus them
      Asks        : Flag_Vectors.Vector;   --  of the push under way: the channels it asked a step the one test of motion sees
      Changed     : Flag_Vectors.Vector;   --  of the push under way: the channels whose target it changed
      Rest_Offset : Real_Vectors.Vector;   --  per channel: the load it held where a push last came to rest, its reading
                                           --  less its target less its dead band's part (Driver.Robot.Steps.Start: a
                                           --  load it holds sets it off its target by as much again); empty before any
                                           --  push came to rest
      Last_Way    : Real_Vectors.Vector;   --  per channel: +1 or -1, the way a push last moved it along its ask, 0 before
                                           --  one did (its dead band leaves it short that way: Steps.Finish)
      Stuck       : Flag_Vectors.Vector;   --  per channel: a push judged blocked asked it to go and it stood where it
                                           --  stood, and none has moved it since (Steps.Finish)
      Free_Shortfalls : Real_Vectors.Vector;   --  of every answered push that was not blocked, along its ask, in order
      Free_Short_Most  : Real_Vectors.Vector;    --  per channel: the most an answered push that was not blocked and asked it a
                                                 --  step fell short of its ask of it, along that ask (Driver.Robot.Steps)
      Free_Short_Count : Count_Vectors.Vector;   --  per channel: those pushes, counted; both empty before any
      Pause       : Natural := 0;          --  the most beats the group stood still within a push, short of its target,
                                           --  and then went on toward it with no new ask (Driver.Robot.Steps)
      Executed    : Real_Vectors.Vector;   --  per channel: the least ask of it a push moved it for (Driver.Robot.Steps:
                                           --  the body executes asks that large; a servo stands still for one within
                                           --  its dead band), Real'Last until one did; empty before any push
      Held_Off    : Real_Vectors.Vector;   --  per channel: the farthest its reading stood from its target where a push
                                           --  came to rest (Driver.Robot.Steps.Standoff: its dead band and any load it
                                           --  holds there); 0 until one did, empty before any push
      Short_Up, Short_Down : Real_Vectors.Vector;   --  per channel: the most it stood short of its target, along the
                                                    --  ask, where a push that asked it up (down) and moved it along
                                                    --  that ask came to rest (Driver.Robot.Steps.Dead_Band); Real'First
                                                    --  until one did, empty before any push
      Low_Seen, High_Seen : Real_Vectors.Vector;   --  per channel, the lowest and highest reading so far
      Seen_At     : Real_Vectors.Vector;   --  per channel: the turn at which the recognition's probe saw an eye move
                                           --  with it (Boot), 0 where none did; empty before the groups are recognized
      Stopped_Low, Stopped_High : Real_Vectors.Vector;   --  per channel, the ends: where the body stopped a push of the
                                                         --  channel at the same reading from two poses, the furthest
                                                         --  down and up (End_Of); valid where the flags are
      Has_Stopped_Low, Has_Stopped_High : Flag_Vectors.Vector;   --  an end of that sense was found for the channel
      Stops       : Stop_Vectors.Vector;   --  the stops that have not been found again from another pose
      Rest_Command : Real_Vectors.Vector;   --  per channel: its command while the body stood still before any push (Boot:
                                            --  its echoed target then, else its reading), the command that holds the
                                            --  body still; empty before
      Keeps_Going  : Flag_Vectors.Vector;   --  per channel: held away from its rest command, it kept the body moving (an
                                            --  eye's picture drifting while every reading stood), and the body stopped
                                            --  once it was sent back (Motion.Settle): a speed, not a place; empty before
      Kept_Eyes    : Flag_Vectors.Vector;   --  per eye (from 0): the eyes whose pictures drifted while one of its channels
                                            --  kept the body moving; empty before
      Comes_Back   : Flag_Vectors.Vector;   --  per channel: a push of it, held and sent back, came back (Motion.Settle): a
                                            --  joint, whose pushes need no holding to tell a speed; empty before
   end record;

   package Group_Stream_Vectors is new Ada.Containers.Vectors (Group_Id, Group_Stream);

   --  The cells an image is divided into for measuring where it moves.
   type Cell_Grid is record
      Width, Height : Natural := 0;
      Columns, Rows : Natural := 0;
   end record;

   function Cells (G : Cell_Grid) return Natural is (G.Columns * G.Rows);

   --  An eye's motion, beat after beat: for every cell, the displacement of
   --  the image content since the previous beat (pixels) and the smaller
   --  eigenvalue of the cell's gradient tensor (how well that displacement
   --  is determined).
   type Eye_Stream is record
      Grid          : Cell_Grid;
      Previous      : Luma_Access;           --  luma of the last frame
      Current       : Luma_Access;           --  luma of this beat's frame
      Before        : Luma_Access;           --  luma of the frame two beats ago
      Means, Variances : Luma_Access;        --  the stillness test's per-pixel reads of its views
      Deviations       : Luma_Access;        --  the square roots of Variances, for the settle watch
      Has_Previous  : Boolean := False;      --  Previous is the frame of the beat before, of the grid's size
      Has_Before    : Boolean := False;      --  Before is the frame two beats ago, of the grid's size
      --  Those are so while the eye measures a beat; once its frames moved on (One_Eye), Previous holds that beat's
      --  frame, Before the one before, and Current a spare with the frame of two beats before, to be written over:
      --  between beats the latest frame is Latest_Frame's.
      Latest_In_Previous : Boolean := False;   --  the latest frame was moved on into Previous; else it is Current
      --  Whether the picture has stopped changing since the body last began to
      --  move (Driver.Robot.Stillness, Settled): this beat's mean luma change
      --  against the beat before and against two beats before, and the watch
      --  over them.
      Change_1, Change_2 : Real := -1.0;     --  at the latest beat; negative when not measured
      Watch_Last    : Real := 0.0;          --  the change of the beat before (Stillness.Watch)
      Watch_Have    : Boolean := False;     --  a beat before has a change to compare with
      Watch_Diffs   : Real_Vectors.Vector;  --  the beat-to-beat differences of the change at the stationary beats
      Watch_Scatter : Real := 0.0;          --  their median: what the change wanders by at a lasting level
      Watch_Scatter_Counted : Natural := 0; --  how many of them that rests on; taken again as they double
      Watch_Levels  : Real_Vectors.Vector;  --  the change at the stationary beats: what rest makes
      Watch_Rest    : Real := Real'Last;    --  their median (Real'Last before two), and
      Watch_Rest_Scale : Real := 0.0;       --  their robust scale
      Watch_Rest_Counted : Natural := 0;    --  how many those rest on; taken again as they double
      Watch_Run     : Natural := 0;         --  stationary beats in a row, now
      Watch_Done    : Boolean := False;      --  the picture has stopped since then
      Drift_Run     : Natural := 0;         --  beats in a row whose change went on from the last one's, not shrinking (as
                                            --  rest's does not): the picture drifting, as a body that keeps going makes it
      Anchor        : Luma_Access;           --  the frame taken when the readings last stood (Stillness.Set_Anchor), and
      Has_Anchor    : Boolean := False;      --  it is in effect
      Away_Last     : Real := 0.0;           --  the picture's change from it at the beat before
      Away_Run      : Natural := 0;          --  beats in a row the picture went farther from it than the beat before, and
                                            --  farther than rest moves a picture: a drift too slow for one beat to show
      Origin        : Luma_Access;           --  the frame a push began from (Stillness.Set_Origin), and
      Has_Origin    : Boolean := False;      --  it is in effect
      Origin_At     : Natural := 0;          --  the beat it was taken at
      From_Origin   : Real := Real'Last;     --  the picture's change from it at the latest beat; Real'Last unmeasured
      Run_Moved     : Boolean := False;      --  the still view holds frames the noise called changed, after the
                                             --  picture stopped (Stillness.Judge_Eye): it measures no noise
      Du, Dv        : Real_Vectors.Vector;   --  Cells values per beat
      Condition     : Real_Vectors.Vector;   --  Cells values per beat
      Resolved      : Flag_Vectors.Vector;   --  Cells values per beat: the displacement was measured (Flow)
      Measured      : Flag_Vectors.Vector;   --  per beat: both frames were there
      Noise         : Real_Vectors.Vector;   --  per cell: displacement noise at rest, once measured
      Textured      : Flag_Vectors.Vector;   --  per cell: can show a displacement, once measured
      Kept_Groups   : Count_Vectors.Vector;  --  the lock-in's regressors: each one's group
      Kept_Channels : Count_Vectors.Vector;  --  and channel
      Gains         : Real_Vectors.Vector;   --  per cell and regressor: squared displacement per reading
                                             --  unit over the cell's noise, less its estimation variance;
                                             --  zero where the cell does not respond to that group
      Gain_Variances : Real_Vectors.Vector;  --  their variances
      Gain_Totals   : Real_Vectors.Vector;   --  per regressor: its gains summed over the cells, and their variances'
      Spread_Totals : Real_Vectors.Vector;   --  (Lockin.Total_Gains), as Lockin.Gain_Of gives them
      Shifts        : Real_Vectors.Vector;   --  per cell and regressor: pixels moved per reading unit,
                                             --  zero where the cell does not respond to that group
      Shift_U, Shift_V : Real_Vectors.Vector;   --  per cell and regressor: the signed pixels moved across and down per
                                                --  reading unit, zero where the cell does not respond to that group
      Col_Responds  : Flag_Vectors.Vector;   --  per cell and regressor: the cell follows that channel alone (the
                                             --  lock-in's test of its one column), whatever its group's others do
      Luma_Variance : Real_Vectors.Vector;   --  per cell: a resting pixel's luma variance (Stillness)
      Settled       : Driver.Pixels.View;    --  the frames since the eye last saw a change
      Noise_View    : Driver.Pixels.View;    --  the longest still run before the current one
      Noise_Is_Settled : Boolean := True;    --  the settled view is the longest run so far
      Has_Settled   : Boolean := False;
      Is_Still      : Boolean := False;      --  at the latest beat, once judged (Driver.Robot.Stillness)
      Has_Judged    : Boolean := False;      --  the latest frame was judged, not only added to the first run
      Judged        : Flag_Vectors.Vector;   --  per beat: the eye had a frame and was judged
      Rest_Factor   : Real := 1.0;           --  a resting cell's displacement noise over its floor (Lockin): the
                                             --  typical cell's, for a cell that never resolved one at rest
      Rest_Factors  : Real_Vectors.Vector;   --  per cell, the same measured on that cell alone (Lockin.Measure_Rest_Noise);
                                             --  empty until measured
      Settling_Tail : Natural := 0;          --  how many beats the picture goes on settling after the body stops
                                             --  moving (Lockin.Measure); 0 until measured
      Rest_Counts_Known  : Boolean := False; --  how many cells move at a beat when nothing is pushed (Lockin):
      Rest_Count_Max     : Natural := 0;     --  the most of them at any such beat,
      Rest_Count_Level   : Real := 0.0;      --  the count a beat's moved cells must exceed to be motion: the rest
                                             --  counts' median plus Z of their robust scatter (not their most: one
                                             --  flickering frame among 116 at rest put the most at 273 of 525 cells
                                             --  for the kiwi's wrist eye, and a jaw opening 1250 units moved 130 to
                                             --  156 cells, unseen, R6 on 10-14)
      Rest_Count_Beats   : Natural := 0;     --  over how many beats
      Rest_Count_Above   : Natural := 0;     --  of those beats, how many counted more than the level: as often a
                                             --  beat at rest is called motion
      Rest_Counts        : Count_Vectors.Vector;   --  every count of those beats, least first: a beat counting as many
                                             --  is as rare at rest as the share of them that reach it
                                             --  (Lockin.Rest_Share)
      Stayed_Far    : Real_Vectors.Vector;   --  per group: the shares of the textured cells its pushes left moved beyond
                                             --  the flow's reach, summed over them (Motion.Note_Stayed), and
      Stayed_Pushes : Count_Vectors.Vector;  --  how many pushes those are
      Stayed_Moved  : Count_Vectors.Vector;  --  per group: the textured cells its pushes left moved, summed over them, and
      Stayed_Cells  : Count_Vectors.Vector;  --  the textured cells measured, summed
      Stayed_Far_Cells : Count_Vectors.Vector;   --  per group: the textured cells its pushes left moved beyond the flow's
                                                 --  reach, summed over them
      Stayed_Best_Moved : Count_Vectors.Vector;   --  per group: of the push that left the largest share of the view moved,
      Stayed_Best_Cells : Count_Vectors.Vector;   --  the cells it left moved and the textured cells it was counted of
      Stayed_Least_Far   : Count_Vectors.Vector;   --  per group: of the push that left the least share of the view
      Stayed_Least_Cells : Count_Vectors.Vector;   --  beyond the flow's reach, the cells it left so and the textured
                                                   --  cells it was counted of
      Stayed_Pulled : Count_Vectors.Vector;  --  per group: how many of its pushes left the cells that did not move alone
                                             --  pulled together the same way (Motion.Note_Stayed)
      Last_Stayed_Group : Natural := 0;   --  the group of the latest push noted (Motion.Note_Stayed), none at first,
      Last_Stayed_Moved : Natural := 0;   --  the cells it left moved, beyond the flow's reach or not, and
      Last_Stayed_Cells : Natural := 0;   --  the textured cells they were counted of
      Still_At      : Flag_Vectors.Vector;   --  per beat: judged still
      Settled_At    : Flag_Vectors.Vector;   --  per beat: its picture had stopped changing since the body last began
                                             --  to move (the watch's Done at that beat, Stillness)
   end record;

   function Rest_Factor_Of (S : Eye_Stream; Cell : Natural) return Real;
   --  The cell's displacement noise at rest over its floor: its own where measured, else the eye's typical.

   function Latest_Frame (S : Eye_Stream) return Luma_Access;
   --  The eye's latest frame between beats, after its frames moved on: Previous once a frame of the grid's size moved
   --  into it, Current where one of another size broke the chain; null before any frame. A frame the decider takes as
   --  the latest (Stillness.Set_Anchor, Set_Origin, Motion.Note_Stayed) read from Current was two beats old: the flows
   --  from such an anchor equalled the eye's beat to beat flows of two beats before, summed (the G1's head camera,
   --  seed 3, paced round p170a, 10-24).

   package Eye_Stream_Vectors is new Ada.Containers.Vectors (Eye_Id, Eye_Stream);

   --  What one group's push does to one eye (Driver.Robot.Lockin).
   type Eye_Effect is record
      Verdict    : Eye_Response := Unmeasured;
      Responding : Natural := 0;   --  cells whose displacement follows the group
      Textured   : Natural := 0;   --  cells that can show a displacement
      Fraction   : Estimate;       --  Responding / Textured, with its binomial sigma
      Resting    : Real := 0.0;    --  the motion of the cells that did not respond, together, in sigmas (Lockin)
      Rest_Pull  : Real := 0.0;    --  how significantly the cells that did not respond are pulled along the push together:
                                   --  their signed gains pooled, over the spread of the pool, in sigmas (Lockin); zero
                                   --  when none
      Could_Be_Whole : Boolean := True;
      --  The most cells that may show the motion are significantly more than half the textured ones: the eye's view
      --  may yet move whole with the group, so the eye may ride on it (Lockin.Judge's test of a whole view, at the
      --  most); false when even the most could not make it whole
      Far_Chance : Real := 1.0;
      --  The chance that rest alone put as many of the group's pushed beats' pictures beyond the flow's reach as they
      --  were (Lockin: the binomial tail at rest's rate of such beats; one for none): a motion too large for the
      --  regression, which leaves such beats out
   end record;

   package Effect_Vectors is new Ada.Containers.Vectors (Positive, Eye_Effect);
   --  Indexed (G - 1) * Eyes + E.

   --  What one channel's push does to one eye: the effect of a group, for one of its channels alone (Driver.Robot.Lockin).
   --  A group whose channels move different eyes (a humanoid's torso moves every eye, its neck the head's, an arm its
   --  wrist's) is told apart by these.
   type Channel_Effect is record
      Group   : Group_Id;
      Channel : Positive;
      Effect  : Eye_Effect;
   end record;

   package Channel_Effect_Vectors is new Ada.Containers.Vectors (Positive, Channel_Effect);
   package Eye_Channels_Vectors is new Ada.Containers.Vectors (Eye_Id, Channel_Effect_Vectors.Vector, Channel_Effect_Vectors."=");

   package Role_Vectors is new Ada.Containers.Vectors (Group_Id, Group_Role);
   package Arm_Number_Vectors is new Ada.Containers.Vectors (Group_Id, Arm_Id'Base);
   package Clause_Vectors is new Ada.Containers.Vectors (Group_Id, Natural);
   package Arm_Group_Vectors is new Ada.Containers.Vectors (Arm_Id, Group_Id, Driver.Observations."=");
   package Arm_Eye_Vectors is new Ada.Containers.Vectors (Arm_Id, Eye_Id'Base, Driver.Observations."=");
   package Mount_Vectors is new Ada.Containers.Vectors (Eye_Id, Mount);
   package Lag_Vectors is new Ada.Containers.Vectors (Eye_Id, Integer);
   package Eye_Flag_Vectors is new Ada.Containers.Vectors (Eye_Id, Boolean);

   --  The groups and eyes as measured: what each group's push does to each
   --  eye, and what follows from that (Driver.Robot.Graph).
   type Body_Graph is record
      Effects : Effect_Vectors.Vector;
      Channels : Eye_Channels_Vectors.Vector;   --  per eye, the effect of each channel the lock-in kept
      Roles   : Role_Vectors.Vector;
      Arm_Of  : Arm_Number_Vectors.Vector;   --  an Arm's own number, a Closer's arm, else 0
      Breach  : Clause_Vectors.Vector;
      Arms    : Arm_Group_Vectors.Vector;
      Arm_Eye : Arm_Eye_Vectors.Vector;   --  per arm: the eye it is for when its group's channels move different eyes
                                          --  (an arm of the group for each), else 0: the arm of a whole group
      Mounts  : Mount_Vectors.Vector;
      Carrier : Group_Id'Base := 0;
   end record;

   --  The kinematics' evidence for one arm and its eye (Driver.Robot.Kinematics).
   type Keyframe is record
      Beat     : Natural := 0;
      Readings : Real_Vectors.Vector;   --  the arm's readings, still
      Image    : Driver.Images.Image;    --  what its eye saw, still
      Asked    : Flag_Vectors.Vector;    --  per channel: the sweep asked the joint to turn for this frame
                                         --  (Kinematics.Asking); none for a frame no push of the sweep's asked for
      Body_Image : Driver.Images.Image;  --  what the body's eye under the arm saw then (Arm_Evidence.Body_Eye); none
                                         --  without one, or once aligned (Kinematics.Base)
   end record;

   package Keyframe_Vectors is new Ada.Containers.Vectors (Positive, Keyframe);

   --  A still of the body's walk under an arm (Driver.Robot.Kinematics.Base): the arm held, the body moved by what
   --  carries it and come to rest, what the arm's eye and the body's eye saw.
   type Body_Still is record
      Beat       : Natural := 0;
      Image      : Driver.Images.Image;
      Body_Image : Driver.Images.Image;
   end record;

   package Body_Still_Vectors is new Ada.Containers.Vectors (Positive, Body_Still);

   --  How the body under an arm moved at one of its keyframes since the reference keyframe, as its eye saw it, and
   --  where each of the keyframe's points would have been seen had it not (Driver.Robot.Kinematics.Base): per query
   --  point, what is added to the aligner's answer before the fit.
   type Base_Shift is record
      Known            : Boolean := False;
      Turn, Step       : Real := 0.0;   --  how far the base turned (radians) and moved (the walk's scale), for the log
      Shift_U, Shift_V : Real_Vectors.Vector;
   end record;

   package Base_Shift_Vectors is new Ada.Containers.Vectors (Positive, Base_Shift);

   --  Where the reference keyframe's query points went in one keyframe of the arm's own eye.
   type Match_Set is record
      Frame    : Positive := 1;          --  the keyframe
      To_U, To_V     : Real_Vectors.Vector;   --  per query point, where it went
      Back_U, Back_V : Real_Vectors.Vector;   --  and where that matched back to in the reference
      Sigma    : Real_Vectors.Vector;    --  and the standard deviation the aligner stated of the answer
      Freedom  : Count_Vectors.Vector;   --  and how many degrees of freedom that standard deviation rests on (0: it is known)
      Found    : Flag_Vectors.Vector;    --  the aligner gave an answer a fit may rest on (an own keyframe: from a
                                         --  narrow window, Followed, within Z of the typical uncertainty)
      Answered : Flag_Vectors.Vector;    --  the aligner gave an answer at all, from a window of any width (own keyframes)
      Stated   : Real := Real'Last;      --  an own keyframe: the median standard deviation its prediction stated
      Followed : Boolean := False;       --  and whether most of its answers came from a window no wider than a patch
      Wide     : Natural := 0;           --  and how many points it refused for that prediction's uncertainty
   end record;

   package Match_Set_Vectors is new Ada.Containers.Vectors (Positive, Match_Set);

   --  An arm's kinematics and its eye's lens as last fitted (Driver.Robot.Kinematics.Fit).
   type Joint_Fit is record
      W, P  : Vec3 := [0.0, 0.0, 0.0];
      C     : Real := 1.0;
      Slide : Boolean := False;
      Slack       : Real := 0.0;   --  half its gears' slack, in reading units (Kinematics.Fit's Play); 0 where none
      Slack_Place : Real := 0.0;   --  where its slack stood at the reference keyframe, from the slack's middle
   end record;

   package Joint_Fit_Vectors is new Ada.Containers.Vectors (Positive, Joint_Fit);

   type Lens_Fit is record
      Fx, Fy, Cx, Cy, K1, K2 : Real := 0.0;
   end record;

   type Arm_Fit is record
      Fitted    : Boolean := False;
      Reference : Real_Vectors.Vector;    --  the readings of the reference keyframe
      Reference_Beat : Natural := 0;      --  its beat in this stream; 0 for a fit read from a body file, whose stream
                                          --  is gone (Kinematics.Slack_Parts)
      Joints    : Joint_Fit_Vectors.Vector;
      Chain     : Count_Vectors.Vector;   --  the joints along the arm from its base, by their readings' places (Kinematics.Fit)
      Lens      : Lens_Fit;
      Used      : Natural := 0;           --  sightings in the last fit
      Median_Px, Sigma_Px : Real := 0.0;
      Matches   : Natural := 0;           --  keyframes with matches behind it
      --  How many of the arm's first keyframes define its unit of length: the
      --  eye's positions over them have a root mean square of one
      --  (Kinematics.Fit). Those of its first fit, kept as that set: a
      --  keyframe taken later refines every term and moves no length. 0 until
      --  the arm is fitted.
      Unit_Frames : Natural := 0;
      --  Those keyframes' readings less the reference's, a row of the arm's channels each: the unit kept with the
      --  fit, so that a body begun from its file, which keeps no keyframes, still has it (Kinematics.Unit_Sigma_Of,
      --  Place_Covariance_In_Unit).
      Unit_Changes : Real_Vectors.Vector;
      --  The focal search's width and range of the fit that holds (Fit.Fit_Report), and the beat of the estimate it was
      --  made in: a fit made again within that estimate begins from it (Kinematics.Refit's Again).
      Focal_Within, Searched_Least, Searched_Most : Real := 0.0;
      Profiled_Over : Natural := 0;   --  the sightings its focal length's profile was last walked over (Fit_Report)
      Profile_Var   : Real := 0.0;   --  the variance of its focal length's logarithm that profile said (Fit_Report)
      Order_Told    : Boolean := False;   --  its joints' order is told by the sightings, not only the best (Fit_Report)
      Untold        : Joint_Pair_Lists.Vector;   --  when not: the joints next to each other along it whose swap is not
                                                 --  told (Fit_Report)
      Untold_Turns  : Real_Vectors.Vector;       --  per pair of Untold, its two joints' turns that tell it (Fit_Report)
      Untold_Orders : Count_Vectors.Vector;      --  when not: every order not told from Chain, one after another, as
                                                 --  long as Chain each, by the readings' places (Fit_Report)
      Slide_Turns   : Real_Vectors.Vector;       --  per joint, three numbers: the turn a slide is weighed against
                                                 --  again (Fit_Report)
      Turns_Told    : Count_Vectors.Vector;      --  the joints whose turn was told from its slide, not weighed again
                                                 --  (Fit_Report)
      Made_At : Natural := 0;
      Why       : Ada.Strings.Unbounded.Unbounded_String;
      Covariance : Real_Vectors.Vector;   --  of the fit's parameters, row by row (Kinematics.Fit)
      --  The table its eye saw, in its reference frame (Table_In_Arm): the
      --  plane most of its tracks lie on (Kinematics.Fit.Dominant_Plane), with
      --  its whole uncertainty.
      Table        : Driver.Geometry.Plane_Estimate;
      Table_A      : Vec3 := [0.0, 0.0, 0.0];                 --  the same plane as its eye sees it (Fit.Sight_Plane)
      Table_Covariance : Mat3 := [others => [others => 0.0]];  --  of Table_A, its lens's lines of sight held
      Table_On     : Flag_Vectors.Vector;                      --  per track: it lies on the table
      --  How Table_A moves with each of the fit's terms, as the table takes the fit's uncertainty (three rows of as
      --  many as Covariance has terms, row by row: every depth together and the lens's lines of sight,
      --  Kinematics.Table_Estimate), and what its points' scatter about it leaves it uncertain by alone: the arm's fit
      --  moved along its covariance moves its table so (the arm's variants a fixed eye is placed with,
      --  Flow_Eye.Arm_Variant).
      Table_Response : Real_Vectors.Vector;
      Table_Own      : Mat3 := [others => [others => 0.0]];
      --  Every track's point in its reference frame, three numbers each,
      --  where Track_Known holds (its depth is a depth: Fit.Depth_Known), at
      --  the depth the fit refined, its logarithm uncertain by Track_Sigmas.
      Tracks       : Real_Vectors.Vector;
      Track_Known  : Flag_Vectors.Vector;
      Track_Sigmas : Real_Vectors.Vector;
      --  Per track, as many numbers as Covariance has terms: how the logarithm of its depth moves with each, the
      --  other tracks' depths at their best (Fit.Fit_Report.Depth_Gains); Track_Sigmas is what is left of its
      --  uncertainty with the terms held (Kinematics.Offer_Of).
      Track_Gains  : Real_Vectors.Vector;
      --  Where its reference frame is in the world (Kinematics.In_World):
      --  X_world = Placement * (Scale * X). The world is the first arm's
      --  reference frame, so that arm is placed as it is.
      Placed       : Boolean := False;
      Placement    : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Scale        : Real := 1.0;
      Scale_Sigma  : Real := 0.0;
      Placement_Covariance : Real_Vectors.Vector;   --  7 x 7, row by row: its turn (world frame), its centre, the
                                                    --  logarithm of its scale
      Placed_Px    : Real := 0.0;                  --  the noise of the fixed eye that placed it, in its pixels
      Placed_Points : Natural := 0;                --  the sightings that eye's fit rested on
      Placed_Through : Natural := 0;               --  the eye that saw both it and the first arm
   end record;

   --  The lens and pose of an eye fixed in the world, measured from the edge flow of the arms it watched turn
   --  (Driver.Robot.Flows, Kinematics.Flow_Eye): its lens, and its camera in the reference frame of every arm it saw
   --  (in that arm's unit), with the covariance of all of them (Joint, row by row): the lens's six terms as an arm's
   --  fit has them (Kinematics.Fit.Lens_Terms: the logarithms of Fx and Fy, Cx, Cy, K1, K2; the distortion terms held
   --  at nought), then per arm the camera's turn about its own axes and its centre. Arm, Pose and Covariance: the
   --  first arm it saw and its camera in that arm's frame, the covariance of the lens and that pose (12 x 12), as
   --  the fixed eye's users read them. Known only when the flow told the eye; Why says what it leaves out when not.
   type Fixed_Fit is record
      Known      : Boolean := False;
      Arm        : Arm_Id'Base := 0;            --  the arm whose frame Pose is in
      Lens       : Lens_Fit;
      Pose       : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Covariance : Real_Vectors.Vector;
      Arms       : Count_Vectors.Vector;        --  every arm the eye saw, in the order of Poses
      Poses      : Rigid_Vectors.Vector;        --  the camera in each one's frame
      Joint      : Real_Vectors.Vector;
      Used       : Natural := 0;               --  the pixels the fit kept
      Offered    : Natural := 0;               --  the pixels two joints or more moved
      Sigma_Px   : Real := 0.0;                --  how far it leaves the arms' points in its picture
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Judged     : Boolean := False;           --  the flow was fitted (a restart does not capture it again)
   end record;

   package Fixed_Fit_Vectors is new Ada.Containers.Vectors (Eye_Id, Fixed_Fit);

   type Arm_Evidence is record
      Arm      : Arm_Id'Base := 0;
      Group    : Group_Id'Base := 0;
      Eye      : Eye_Id'Base := 0;
      Frames   : Keyframe_Vectors.Vector;   --  the first is the reference
      Query_U, Query_V : Real_Vectors.Vector;   --  the reference's query points
      Query_Responds : Flag_Vectors.Vector;
      --  Per query point: the lock-in saw its cell follow the arm's pushes when the points were chosen (or had not
      --  told yet). Before its keyframes tell which points ride with the eye (Still), these are the points that
      --  tell the view's common motion (Keyframes.Add's Riding).
      Chain    : Driver.Alignment.Keyframes.Chain;   --  the reference's pyramid and what its keyframes showed
      Aligned  : Natural := 0;   --  how many of Frames are aligned into Matches (Kinematics.Align_New)
      Twin_Taken : Boolean := False;   --  a keyframe at the reference's readings was taken after it: its still twin
      Revision : Natural := 0;
      --  How many keyframes were aligned again from the fitted arm (Kinematics.Realign), and fits asked for again
      --  (Kinematics.Weigh_Across_Joints): a count above its matches', so that the next estimate fits it again.
      Across_Due : Boolean := False;
      --  The arm's next fit takes the jackknife over its joints into its covariance (Fit's Across_Joints): asked once
      --  its sweep has ended (Kinematics.Weigh_Across_Joints).
      Matches  : Match_Set_Vectors.Vector;   --  one per aligned keyframe after the reference, in order
      Shifts   : Real_Vectors.Vector;
      --  Per channel: how far the eye's view moves per reading unit of it, in pixels, as the aligner measured it over
      --  the keyframes (Kinematics.Measure_Shifts: per point, the displacements regressed on the changes of every
      --  channel that moved; the median over the points); 0 while not measured. Measured by the aligner, whose
      --  windows reach half the picture through its pyramid, not by the lock-in's flow of one beat: that flow locks
      --  on what does not move with the scene (a near-textureless table under JPEG blocks, the jaws the camera
      --  carries) and reads a still picture where the camera moved (R4 on the Pi, 10-13: its slope came out 50 to
      --  300 times under what the keyframes showed, the sweep's first level was 1087 reading units, and every
      --  prediction's window was sized by it). Kept when the chain begins anew, and in the body file.
      Shift_Sigmas : Real_Vectors.Vector;
      --  Per channel: the standard deviation of that measurement; Real'Last while the keyframes do not determine the
      --  channel at all. A channel's measurement may be insignificant yet bound the lock-in's slope
      --  (Kinematics.Pixels_Per_Unit).
      Shift_Points : Count_Vectors.Vector;   --  per channel: how many points that measurement rests on
      Shift_Seen   : Flag_Vectors.Vector;    --  per channel, once told: the view moves with it (the points that move
                                             --  with it are not a patch of those told: Kinematics.Measure_Shifts);
                                             --  False for a channel that moves a part of the body in the view
      Shift_Told   : Count_Vectors.Vector;   --  per channel: how many points were told of it
      Scatter      : Real := 0.0;
      --  How far the aligner errs on the arm's eye's pictures (Driver.Alignment.Keyframes.Scatter) as the keyframes
      --  last measured it: kept when the sweep begins anew and in the body file, the eye's error until the keyframes
      --  measure it again (Kinematics.Match_Noise); 0 until measured. An arm begun from a body file keeps no keyframes,
      --  and its eye's error was unknown: s58 (servo seed 6, 10-18) placed none of the 336 points of its scene, and its
      --  eye could not look around (Driver.Robot.Look), every place judged with an error of 0.
      Asking   : Flag_Vectors.Vector;   --  per channel: the sweep is turning it now (Kinematics.Asking), copied into
                                        --  the keyframes taken meanwhile (Keyframe.Asked)
      Sweeping : Boolean := False;   --  the arm's sweep has begun (Kinematics.Begin_Anew): its keyframes are taken
                                     --  from then on, none before (the recognition's holds would give the aligner
                                     --  a keyframe at every pose, minutes of work, for a measurement the sweep's
                                     --  first level makes itself)
      Still    : Flag_Vectors.Vector;   --  per query point, once its keyframes tell: it does not move with the view
      Stood_Off, Moved_With : Count_Vectors.Vector;
      --  per query point: the keyframes whose common motion it stood against, and moved with
      --  (Keyframes.Stood_And_Moved)
      Stand_Rate : Real := 0.0;
      Patch_Noise, Patch_Error : Real_Vectors.Vector;
      --  per query point: the mean difference of its patch's levels between the reference and a keyframe at the
      --  reference's pose (the noise alone), and that mean's standard error; zero before such a keyframe
      Patch_Stood, Patch_Moving : Count_Vectors.Vector;
      --  per query point: the keyframes in which most points' patches changed beyond their noise (the view moved), and
      --  of those, the ones in which its own did not (Kinematics.Patch_Evidence)
      Patch_Rate : Real := 0.0;
      --  how often a point's patch stood in such a keyframe, over every point (Laplace's: one more over two more)
      --  how often a point of the world stood against its keyframe's common motion (Measure_Shifts); 0 before any
      --  point was told of the world
                                        --  (Kinematics.Measure_Shifts: fixed to the eye, as the hand's jaws in a
                                        --  wrist eye's view are, or answered at its place by false peaks); nothing
                                        --  is known of a point beyond its length
      --  Per channel and query point (the channel's block of the query points, Kinematics.Measure_Shifts' lines): how
      --  far the point moves across and down the picture per unit of the channel, and whether its keyframes tell it.
      --  The fit's one-joint stages take a keyframe's points to where its own joint alone would have put them
      --  (Refit: Fit.Sighting's Off_U, Off_V).
      Point_Gu, Point_Gv : Real_Vectors.Vector;
      Point_G_Told       : Flag_Vectors.Vector;
      Result   : Arm_Fit;
      --  The body under the arm (Driver.Robot.Kinematics.Base): an eye the carrier carries and no arm (Body_Eye, 0
      --  without one), which moves with the arm's base; the stills of the walk the boot took with the arm held, before
      --  the reference keyframe (Walking while it walks, Still_Due for the next beat's pictures); the body eye's points
      --  of its picture at the reference keyframe; the chains that align both eyes' stills and the body eye's
      --  pictures at the keyframes, and what they found; the calibration the stills gave (Driver.Rig); and per
      --  keyframe the base's motion and the sightings' shift that takes it out (Base, one a keyframe from the
      --  reference).
      Body_Eye     : Eye_Id'Base := 0;
      Walking      : Boolean := False;
      Still_Due    : Boolean := False;
      Stills       : Body_Still_Vectors.Vector;
      Stills_Aligned : Boolean := False;
      Body_Query_U, Body_Query_V : Real_Vectors.Vector;
      Walk_Chain, Body_Walk_Chain, Body_Chain : Driver.Alignment.Keyframes.Chain;
      Still_Matches, Body_Still_Matches : Match_Set_Vectors.Vector;   --  one per still
      Body_Matches : Match_Set_Vectors.Vector;   --  one per keyframe after the reference, as Matches
      Body_Aligned : Natural := 0;   --  the keyframes whose body picture is aligned, the reference counted
      Calibrated   : Boolean := False;   --  the walk's stills were adjusted (Driver.Rig.Calibrate), Ok or not
      Rig_With_Fit : Boolean := False;   --  from the arm's fit's lens (else from none: the stills' own search)
      Rig_Ok       : Boolean := False;   --  and they told the body's eye on the arm's (Driver.Rig's Ok)
      Rig_Stills   : Natural := 0;   --  the stills the calibration rests on
      Rig_X        : Driver.Numerics.Rigid := Driver.Numerics.Identity;   --  the body's eye's frame in the arm's eye's
      Rig_Arm_Lens, Rig_Body_Lens : Driver.Rig.Lens;
      Rig_Arm_Inverse, Rig_Body_Inverse : Real_Vectors.Vector;   --  per query point, its inverse depth; 0: not placed
      Rig_Turn_Sigma, Rig_Misfit : Real := 0.0;
      Base         : Base_Shift_Vectors.Vector;
   end record;

   package Arm_Evidence_Vectors is new Ada.Containers.Vectors (Positive, Arm_Evidence);

   type Model is tagged limited record
      Beats          : Natural := 0;               --  observations seen
      Stepping       : Boolean := False;           --  a step is under way (Motion.One_Step): no keyframe is taken in it
      Groups         : Group_Stream_Vectors.Vector;
      Eyes           : Eye_Stream_Vectors.Vector;
      Noise          : Real_Vectors.Vector;        --  per channel of every group, in group order
      Measure_Rounds : Natural := 0;               --  the rounds of noise and pushes the last measurement took (Channels.Measure)
      Noise_Freedom  : Count_Vectors.Vector;       --  the degrees of freedom each noise rests on
      Drift          : Real_Vectors.Vector;        --  per channel, as Noise: the readings' steady change a beat at
                                                   --  rest (Channels.Drift), 0 where none stands out
      Lags           : Lag_Vectors.Vector;
      Lag_Known      : Eye_Flag_Vectors.Vector;    --  per eye: its lag stood out of every shift tried
      Settling_Kept  : Lag_Vectors.Vector;         --  per eye: the longest it took to settle after a move, as the
                                                   --  body file kept it (Longest_Settling); empty when none was
      Began_Moving   : Flag_Vectors.Vector;        --  per beat: some commandable group began to move
      Graph          : Body_Graph;
      Graph_Evidence : Natural := 0;               --  push beats behind the current graph
      Lag_Evidence   : Natural := 0;               --  the beats the eyes' lags were last measured from (Recompute)
      Kinematics     : Arm_Evidence_Vectors.Vector;   --  per arm with an eye
      Fixed_Eyes     : Fixed_Fit_Vectors.Vector;     --  per eye: what is measured of it when it is fixed in the world
      Is_Booted      : Boolean := False;
      Apart          : Boolean := False;   --  the heavier estimates are computed apart (Compute_Apart)
      Due            : Boolean := False;   --  computed apart: they are due
      From_File      : Stored_Flags := [others => False];   --  reloaded, so not measured again (Load_Body)
      Report         : Ada.Strings.Unbounded.Unbounded_String;   --  what the last estimate found, for Describe
   end record;

end Driver.Robot;
