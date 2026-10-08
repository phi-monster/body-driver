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
--  Estimate_Now.
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
private with Driver.Pixels;
private with Driver.Services;
with Driver.Uncertain;

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
   --  Recomputes the heavier estimates from everything observed so far.
   --  Deciders call it between Driver.Beats.Next and Send; it can take
   --  seconds, during which the robot holds. The call goes into the
   --  recording (Driver.Recording, kind E), so a replay recomputes where
   --  the run did.

   function Booted (M : Model) return Boolean;
   --  The kinematics of every arm that carries an eye are measured.

   --  What a body file holds (docs/body-file.md), quantity by quantity:
   --  the readings' noise, the readings each channel has moved through, the
   --  step responses, the image lags, what each
   --  group's push does to each eye (the lock-in), the body's graph (roles,
   --  arms, mounts) and the kinematics with their lenses.
   type Stored is
     (Stored_Noise, Stored_Travel, Stored_Steps, Stored_Lags, Stored_Responses, Stored_Graph, Stored_Kinematics);

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

   type Eye_Response is (Unmeasured, Nothing, Patch, Undecided, Whole);
   --  What pushing a group does to what an eye sees: nothing, a patch of the
   --  image moves, or the whole image moves (the eye rides on the group).

   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response;

   function Responding (M : Model; G : Group_Id; E : Eye_Id) return Natural;
   --  How many of the eye's textured cells the verdict found responding to
   --  the group's push.

   function Textured_Cells (M : Model; G : Group_Id; E : Eye_Id) return Natural;
   --  How many cells of the eye the verdict was reached over: those with the
   --  texture to show a displacement. Zero while there is no verdict.

   function Image_Lag (M : Model; E : Eye_Id) return Integer;
   --  How many beats the eye's images trail the readings they belong to:
   --  the image of beat B shows the body as read at beat B - Image_Lag; 0
   --  until measured.

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

   package Real_Vectors is new Ada.Containers.Vectors (Natural, Real);
   package Flag_Vectors is new Ada.Containers.Vectors (Natural, Boolean);
   type Luma_Access is access Real_Array;
   --  A frame's luma (Driver.Images.Luma), on the heap: two per eye, swapped
   --  every beat, never on a stack.
   package Count_Vectors is new Ada.Containers.Vectors (Natural, Natural);

   --  One push of a group and how it went (Driver.Robot.Steps).
   type Episode is record
      Start     : Natural := 0;       --  the beat its target changed
      Moved     : Boolean := False;
      Moved_At  : Natural := 0;       --  the first beat its reading moved
      Ended     : Boolean := False;
      End_At    : Natural := 0;       --  the first still beat, or where it was given up or cut short
      Settled   : Boolean := False;   --  judged: not cut short by the next push
      Rested    : Boolean := False;   --  ended with its readings still (not given up while they kept moving)
      Closest_At : Natural := 0;      --  the beat it came closest to its target, by a step it could be seen to make and its
                                      --  own chatter could not
      Closest   : Real := 0.0;        --  how far along its ask it had come then, in reading units
      Followed  : Natural := 0;       --  the beats followed since then (that one included), and their progress along
      Mean_Along : Real := 0.0;       --  the ask: its mean,
      Spread_Along : Real := 0.0;     --  and the sum of the squares of its deviations from it
      Length    : Real := 0.0;        --  how far it asked, in reading units
      Shortfall : Estimate;           --  how far short of the target it stopped, along the ask
      Delivered : Estimate;           --  the fraction of the ask it delivered, along the ask
      Blocked   : Boolean := False;   --  it fell short by more than the group's free pushes do, and by a shortfall the one
                                      --  test of motion sees, or nothing answered an ask that test would see
   end record;

   package Episode_Vectors is new Ada.Containers.Vectors (Positive, Episode);

   --  A group's readings and the target in effect, beat after beat; beat K
   --  of the stream is the K-th observation (counted from zero).
   type Group_Stream is record
      Size        : Natural := 0;
      Commandable : Boolean := False;
      Values      : Real_Vectors.Vector;   --  Size readings per beat
      Present     : Flag_Vectors.Vector;   --  a reading arrived that beat
      Targets     : Real_Vectors.Vector;   --  Size targets per beat
      Targeted    : Flag_Vectors.Vector;   --  the command in effect carried a target
      Pushed      : Flag_Vectors.Vector;   --  per beat, once measured: moving because it was pushed
      Delay_Beats : Natural := 0;          --  the longest wait from a push to its first motion
      Delay_Known : Boolean := False;      --  some push was answered
      Episodes    : Episode_Vectors.Vector;   --  every push and how it went (Driver.Robot.Steps)
      From, Ask   : Real_Vectors.Vector;   --  of the push under way: the readings before it, and target minus them
      Free_Shortfalls : Real_Vectors.Vector;   --  of every answered push that was not blocked, along its ask, in order
      Low_Seen, High_Seen : Real_Vectors.Vector;   --  per channel, the lowest and highest reading so far
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
      Has_Previous  : Boolean := False;      --  Previous is the frame of the beat before, of the grid's size
      Has_Before    : Boolean := False;      --  Before is the frame two beats ago, of the grid's size
      --  Whether the picture has stopped changing since the body last began to
      --  move (Driver.Robot.Stillness, Settled): this beat's mean luma change
      --  against the beat before and against two beats before, and the watch
      --  over them.
      Change_1, Change_2 : Real := -1.0;     --  at the latest beat; negative when not measured
      Watch_Last    : Real := 0.0;           --  the last measured Change_1
      Watch_Peak    : Real := 0.0;           --  the largest Change_1 since the body began to move
      Watch_Have    : Boolean := False;      --  a last Change_1 was measured since then
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
      Rest_Factor   : Real := 1.0;           --  a resting cell's displacement noise over its floor (Lockin)
      Rest_Counts_Known  : Boolean := False; --  how many cells move at a beat when nothing is pushed (Lockin):
      Rest_Count_Max     : Natural := 0;     --  the most of them at any such beat,
      Rest_Count_Beats   : Natural := 0;     --  over how many beats
      Still_At      : Flag_Vectors.Vector;   --  per beat: judged still
   end record;

   package Eye_Stream_Vectors is new Ada.Containers.Vectors (Eye_Id, Eye_Stream);

   --  What one group's push does to one eye (Driver.Robot.Lockin).
   type Eye_Effect is record
      Verdict    : Eye_Response := Unmeasured;
      Responding : Natural := 0;   --  cells whose displacement follows the group
      Textured   : Natural := 0;   --  cells that can show a displacement
      Fraction   : Estimate;       --  Responding / Textured, with its binomial sigma
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
   end record;

   package Keyframe_Vectors is new Ada.Containers.Vectors (Positive, Keyframe);

   --  Where the reference keyframe's query points went in one keyframe, or
   --  in another eye's view.
   type Match_Set is record
      Frame    : Positive := 1;          --  the keyframe
      Eye      : Natural := 0;           --  the other eye whose view it is; 0 for the arm's own keyframes
      To_U, To_V     : Real_Vectors.Vector;   --  per query point, where it went
      Back_U, Back_V : Real_Vectors.Vector;   --  and where that matched back to in the reference
      Found    : Flag_Vectors.Vector;    --  the instrument gave an answer
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
      --  where Track_Known holds, at the depth the fit refined, its logarithm
      --  uncertain by Track_Sigmas.
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
      Pending  : Pending_Vectors.Vector;
      Matches  : Match_Set_Vectors.Vector;
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
      Noise_Freedom  : Count_Vectors.Vector;       --  the degrees of freedom each noise rests on
      Lags           : Lag_Vectors.Vector;
      Lag_Known      : Eye_Flag_Vectors.Vector;    --  per eye: its lag stood out of every shift tried
      Began_Moving   : Flag_Vectors.Vector;        --  per beat: some commandable group began to move
      Graph          : Body_Graph;
      Graph_Evidence : Natural := 0;               --  push beats behind the current graph
      Kinematics     : Arm_Evidence_Vectors.Vector;   --  per arm with an eye
      Fixed_Eyes     : Fixed_Fit_Vectors.Vector;     --  per eye: what is measured of it when it is fixed in the world
      Is_Booted      : Boolean := False;
      From_File      : Stored_Flags := [others => False];   --  reloaded, so not measured again (Load_Body)
      Report         : Ada.Strings.Unbounded.Unbounded_String;   --  what the last estimate found, for Describe
   end record;

end Driver.Robot;
