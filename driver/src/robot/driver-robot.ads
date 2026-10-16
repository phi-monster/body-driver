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
private with Driver.Services;
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

   function Group_Count (M : Model) return Natural;
   function Group_Size (M : Model; G : Group_Id) return Natural;
   function Is_Commandable (M : Model; G : Group_Id) return Boolean;
   --  A command in effect has carried a target for it.

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
   type Channel_Flags is array (Positive range <>) of Boolean;   --  per channel of a group
   type Luma_Access is access Real_Array;
   --  A frame's luma (Driver.Images.Luma), on the heap: two per eye, swapped
   --  every beat, never on a stack.
   package Count_Vectors is new Ada.Containers.Vectors (Natural, Natural);

   --  The extreme a quantity of a push has reached (Driver.Robot.Steps.Follow), and the quantity at the beats since,
   --  that one included: their count, mean and the sum of the squares of their deviations from it (Welford's).
   type Extreme is record
      Value    : Real := 0.0;
      Followed : Natural := 0;
      Mean     : Real := 0.0;
      Spread   : Real := 0.0;
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
      Episodes    : Episode_Vectors.Vector;   --  every push and how it went (Driver.Robot.Steps)
      From, Ask   : Real_Vectors.Vector;   --  of the push under way: the readings before it, and target minus them
      Asks        : Flag_Vectors.Vector;   --  of the push under way: the channels it asked a step the one test of motion sees
      Free_Shortfalls : Real_Vectors.Vector;   --  of every answered push that was not blocked, along its ask, in order
      Executed    : Real_Vectors.Vector;   --  per channel: the least ask of it a push moved it for (Driver.Robot.Steps:
                                           --  the body executes asks that large; a servo stands still for one within
                                           --  its dead band), Real'Last until one did; empty before any push
      Low_Seen, High_Seen : Real_Vectors.Vector;   --  per channel, the lowest and highest reading so far
      Seen_At     : Real_Vectors.Vector;   --  per channel: the turn at which the recognition's probe saw an eye move
                                           --  with it (Boot), 0 where none did; empty before the groups are recognized
      Stopped_Low, Stopped_High : Real_Vectors.Vector;   --  per channel, the ends: where the body stopped a push of the
                                                         --  channel at the same reading from two poses, the furthest
                                                         --  down and up (End_Of); valid where the flags are
      Has_Stopped_Low, Has_Stopped_High : Flag_Vectors.Vector;   --  an end of that sense was found for the channel
      Stops       : Stop_Vectors.Vector;   --  the stops that have not been found again from another pose
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
      Shifts        : Real_Vectors.Vector;   --  per cell and regressor: pixels moved per reading unit,
                                             --  zero where the cell does not respond to that group
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
      Still_At      : Flag_Vectors.Vector;   --  per beat: judged still
      Settled_At    : Flag_Vectors.Vector;   --  per beat: its picture had stopped changing since the body last began
                                             --  to move (the watch's Done at that beat, Stillness)
   end record;

   function Rest_Factor_Of (S : Eye_Stream; Cell : Natural) return Real;
   --  The cell's displacement noise at rest over its floor: its own where measured, else the eye's typical.

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
   end record;

   package Effect_Vectors is new Ada.Containers.Vectors (Positive, Eye_Effect);
   --  Indexed (G - 1) * Eyes + E.

   package Role_Vectors is new Ada.Containers.Vectors (Group_Id, Group_Role);
   package Arm_Number_Vectors is new Ada.Containers.Vectors (Group_Id, Arm_Id'Base);
   package Clause_Vectors is new Ada.Containers.Vectors (Group_Id, Natural);
   package Arm_Group_Vectors is new Ada.Containers.Vectors (Arm_Id, Group_Id, Driver.Observations."=");
   package Mount_Vectors is new Ada.Containers.Vectors (Eye_Id, Mount);
   package Lag_Vectors is new Ada.Containers.Vectors (Eye_Id, Integer);
   package Eye_Flag_Vectors is new Ada.Containers.Vectors (Eye_Id, Boolean);

   --  The groups and eyes as measured: what each group's push does to each
   --  eye, and what follows from that (Driver.Robot.Graph).
   type Body_Graph is record
      Effects : Effect_Vectors.Vector;
      Roles   : Role_Vectors.Vector;
      Arm_Of  : Arm_Number_Vectors.Vector;   --  an Arm's own number, a Closer's arm, else 0
      Breach  : Clause_Vectors.Vector;
      Arms    : Arm_Group_Vectors.Vector;
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
   end record;

   package Keyframe_Vectors is new Ada.Containers.Vectors (Positive, Keyframe);

   --  Where the reference keyframe's query points went in one keyframe, or
   --  in another eye's view.
   type Match_Set is record
      Frame    : Positive := 1;          --  the keyframe
      Eye      : Natural := 0;           --  the other eye whose view it is; 0 for the arm's own keyframes
      To_U, To_V     : Real_Vectors.Vector;   --  per query point, where it went
      Back_U, Back_V : Real_Vectors.Vector;   --  and where that matched back to in the reference
      Sigma    : Real_Vectors.Vector;    --  and the standard deviation the aligner stated of the answer (none from the instrument)
      Freedom  : Count_Vectors.Vector;   --  and how many degrees of freedom that standard deviation rests on (0: it is known)
      Found    : Flag_Vectors.Vector;    --  the aligner or the instrument gave an answer a fit may rest on (an own keyframe: from a
                                         --  narrow window, Followed, within Z of the typical uncertainty)
      Answered : Flag_Vectors.Vector;    --  the aligner gave an answer at all, from a window of any width (own keyframes)
      Stated   : Real := Real'Last;      --  an own keyframe: the median standard deviation its prediction stated
      Followed : Boolean := False;       --  and whether most of its answers came from a window no wider than a patch
      Wide     : Natural := 0;           --  and how many points it refused for that prediction's uncertainty
   end record;

   package Match_Set_Vectors is new Ada.Containers.Vectors (Positive, Match_Set);


   type Pending_Match is record
      Frame  : Positive := 1;
      Eye    : Natural := 0;           --  as Match_Set has it
      Points : Natural;                --  how many query points were asked: the answer's size (no default,
                                       --  so no request can leave it out)
      Ticket : Driver.Services.Ticket;
   end record;

   package Pending_Vectors is new Ada.Containers.Vectors (Positive, Pending_Match);

   --  An arm's kinematics and its eye's lens as last fitted (Driver.Robot.Kinematics.Fit).
   type Joint_Fit is record
      W, P  : Vec3 := [0.0, 0.0, 0.0];
      C     : Real := 1.0;
      Slide : Boolean := False;
   end record;

   package Joint_Fit_Vectors is new Ada.Containers.Vectors (Positive, Joint_Fit);

   type Lens_Fit is record
      Fx, Fy, Cx, Cy, K1, K2 : Real := 0.0;
   end record;

   type Arm_Fit is record
      Fitted    : Boolean := False;
      Reference : Real_Vectors.Vector;    --  the readings of the reference keyframe
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
      Order_Told    : Boolean := False;   --  its joints' order is told by the sightings, not only the best (Fit_Report)
      Contender     : Count_Vectors.Vector;   --  when not: the order nearest the best's not told from it (Fit_Report)
      Made_At : Natural := 0;
      Why       : Ada.Strings.Unbounded.Unbounded_String;
      Covariance : Real_Vectors.Vector;   --  of the fit's parameters, row by row (Kinematics.Fit)
      --  The table its eye saw, in its reference frame (Table_In_Arm): the
      --  plane most of its tracks lie on (Kinematics.Fit.Dominant_Plane), with
      --  its whole uncertainty.
      Table        : Driver.Geometry.Plane_Estimate;
      Table_A      : Vec3 := [0.0, 0.0, 0.0];                 --  the same plane as its eye sees it (Fit.Sight_Plane)
      Table_Covariance : Mat3 := [others => [others => 0.0]];  --  of Table_A, its lens's lines of sight held
      --  What the plane moves by with the fit's terms (the depths and the lens's lines of sight): 3 x Terms,
      --  row by row; and the covariance of its points' scatter about it, which no term moves. A fixed eye
      --  that sees points on the table carries them (Kinematics.Fixed).
      Table_Response : Real_Vectors.Vector;
      Table_Scatter  : Mat3 := [others => [others => 0.0]];
      Table_On     : Flag_Vectors.Vector;                      --  per track: it lies on the table
      --  Every track's point in its reference frame, three numbers each,
      --  where Track_Known holds (its depth is a depth: Fit.Depth_Known), at
      --  the depth the fit refined, its logarithm uncertain by Track_Sigmas.
      Tracks       : Real_Vectors.Vector;
      Track_Known  : Flag_Vectors.Vector;
      Track_Sigmas : Real_Vectors.Vector;
      --  Where its reference frame is in the world (Kinematics.In_World):
      --  X_world = Placement * (Scale * X). The world is the first arm's
      --  reference frame, so that arm is placed as it is.
      Placed       : Boolean := False;
      Placement    : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Scale        : Real := 1.0;
      Scale_Sigma  : Real := 0.0;
      Placement_Covariance : Real_Vectors.Vector;   --  6 x 6, row by row: its turn (world frame), its centre
      Placed_Px    : Real := 0.0;                  --  the noise of the link that placed it, in its eye's units
      Placed_Points : Natural := 0;                --  the points that placed it
      Placed_Through : Natural := 0;               --  the eye that saw both it and the first arm
      --  Per track, row by row: how the logarithm of its depth moves with each term of Covariance, the others'
      --  depths at their best (Fit_Report.Depth_Gains): the share of every depth's uncertainty all of them have
      --  in common, which a fixed eye placed from these points carries (Kinematics.Fixed).
      Depth_Gains  : Real_Vectors.Vector;
   end record;

   --  The lens and pose of an eye fixed in the world, measured from points the first arm's eye placed and the
   --  answers it gave to where they are (Kinematics.Fixed): its camera in the world (the first arm's reference
   --  frame, in that arm's unit) and its lens, with the covariance of both: 12 x 12, row by row, the lens's six
   --  terms as an arm's fit has them (Kinematics.Fit.Lens_Terms: the logarithms of Fx and Fy, Cx, Cy, K1, K2; the
   --  distortion terms zero when not kept), then the camera's turn about its own axes and its centre in the world.
   --  Known only when the points determine them; Why says what they leave out when not.
   type Fixed_Fit is record
      Known      : Boolean := False;
      Arm        : Arm_Id'Base := 0;            --  the arm whose points measured it
      Lens       : Lens_Fit;
      Pose       : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Covariance : Real_Vectors.Vector;
      Used       : Natural := 0;               --  the answers that fit
      Offered    : Natural := 0;               --  the answers it was given
      Sigma_Px   : Real := 0.0;                --  the noise they show
      Distorted  : Boolean := False;           --  the distortion terms are kept
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      --  What it was measured from, so that it is measured again only when that changed: the world arm's fit
      --  (its keyframes with matches) and the sets of its reference matched into other eyes.
      Judged       : Boolean := False;
      From_Matches : Natural := 0;
      From_Sets    : Natural := 0;
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
      Revision : Natural := 0;   --  how many keyframes were aligned again from the fitted arm (Kinematics.Realign)
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
      Asking   : Flag_Vectors.Vector;   --  per channel: the sweep is turning it now (Kinematics.Asking), copied into
                                        --  the keyframes taken meanwhile (Keyframe.Asked)
      Sweeping : Boolean := False;   --  the arm's sweep has begun (Kinematics.Begin_Anew): its keyframes are taken
                                     --  from then on, none before (the recognition's holds would give the aligner
                                     --  a keyframe at every pose, minutes of work, for a measurement the sweep's
                                     --  first level makes itself)
      Still    : Flag_Vectors.Vector;   --  per query point, once its keyframes tell: it does not move with the view
                                        --  (Kinematics.Measure_Shifts: fixed to the eye, as the hand's jaws in a
                                        --  wrist eye's view are, or answered at its place by false peaks); nothing
                                        --  is known of a point beyond its length
      --  Per channel and query point (the channel's block of the query points, Kinematics.Measure_Shifts' lines): how
      --  far the point moves across and down the picture per unit of the channel, and whether its keyframes tell it.
      --  The fit's one-joint stages take a keyframe's points to where its own joint alone would have put them
      --  (Refit: Fit.Sighting's Off_U, Off_V).
      Point_Gu, Point_Gv : Real_Vectors.Vector;
      Point_G_Told       : Flag_Vectors.Vector;
      Unanswerable : Boolean := False;   --  the instrument can never answer (no address): ask no more
      --  The first arm's reference view matched into this arm's, its query
      --  points into this arm's reference image (Kinematics.Observe), and of
      --  which reference of which group it was asked.
      World_Pending   : Pending_Vectors.Vector;
      World_Matches   : Match_Set_Vectors.Vector;
      World_Asked     : Boolean := False;
      World_Group     : Group_Id'Base := 0;
      World_Reference : Natural := 0;
      --  The reference's query points matched into every other eye's view at
      --  the reference beat, one set per eye (Kinematics.Observe).
      Eye_Pending     : Pending_Vectors.Vector;
      Eye_Matches     : Match_Set_Vectors.Vector;
      Result   : Arm_Fit;
   end record;

   package Arm_Evidence_Vectors is new Ada.Containers.Vectors (Positive, Arm_Evidence);

   type Model is tagged limited record
      Beats          : Natural := 0;               --  observations seen
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
      Kinematics     : Arm_Evidence_Vectors.Vector;   --  per arm with an eye
      Fixed_Eyes     : Fixed_Fit_Vectors.Vector;     --  per eye: what is measured of it when it is fixed in the world
      Is_Booted      : Boolean := False;
      Apart          : Boolean := False;   --  the heavier estimates are computed apart (Compute_Apart)
      Due            : Boolean := False;   --  computed apart: they are due
      From_File      : Stored_Flags := [others => False];   --  reloaded, so not measured again (Load_Body)
      Report         : Ada.Strings.Unbounded.Unbounded_String;   --  what the last estimate found, for Describe
   end record;

end Driver.Robot;
