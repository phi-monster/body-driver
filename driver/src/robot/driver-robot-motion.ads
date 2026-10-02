--  The motion primitives every decider uses. Each exists once: there is one
--  way to wait for the body to settle, one way to take a step and judge what
--  happened, and one solver for reaching a pose, used both to ask whether a
--  pose is reachable and to move there.
--
--  These are decider operations: they exchange beats with the main loop
--  through Driver.Beats and must only be called from the decider task.

with Ada.Containers.Indefinite_Vectors;
with Ada.Strings.Unbounded;
with Driver.Commands;

package Driver.Robot.Motion is

   use Ada.Strings.Unbounded;

   procedure Settle (M : in out Model; Beats_Waited : out Natural);
   --  Holds until Still (M) and returns how many beats that took.

   type Step_Outcome is (Reached, Blocked, Short);
   --  Reached  every targeted channel arrived within its noise
   --  Blocked  a channel stopped short and pushing further does not move it
   --  Short    the body moved but delivered only part of the step

   type Step_Report is record
      Outcome   : Step_Outcome := Short;
      At_Rest   : Boolean := True;   --  every push of the step ended with its readings still
      Beats     : Natural := 0;
      Delivered : Estimate;          --  delivered fraction of the commanded step
      Started   : Natural := 0;      --  how many beats the body had seen when the step was sent
      Detail    : Unbounded_String;  --  which channels, and by how much, for the log
   end record;

   procedure Step (M : in out Model; Targets : Driver.Commands.Command; Report : out Step_Report);
   --  Sends the targets, waits until the body settles, and judges the step.

   procedure Hold (M : in out Model; Beats : Positive);
   --  Holds the body for that many beats.

   procedure Hold_While_Matching (M : in out Model);
   --  Holds the body until the instrument has answered every match the
   --  estimators asked of it (the kinematics' keyframes).

   procedure Hold_For_Twin (M : in out Model; A : Arm_Id);
   --  Holds the body still until the arm's still twin of its reference has
   --  its match answered or refused (Kinematics.Twin_Answered), so the
   --  matcher's noise is known before the arm is swept; an arm that carries
   --  no eye is not held.

   function Sweep_Start (M : Model; A : Arm_Id; Channel : Positive) return Real;
   --  The smallest turn of a joint of the arm that moves its eye's view by
   --  what both the view and the matcher can tell: Z times the larger of
   --  the cells' displacement noise (Lockin.Cell_Noise) and the matcher's
   --  (Kinematics.Match_Noise, from the still twin), over how far the view
   --  moves per reading unit of the joint (Lockin.Shift); a keyframe that
   --  moves less tells the kinematics nothing, and A9's showed it makes the
   --  fit worse. Zero when the arm carries no eye or the shift or the cells'
   --  noise is not measured.

   procedure Hold_For_Keyframe (M : in out Model; A : Arm_Id);
   --  Holds the body until the arm can give its kinematics a keyframe: its
   --  readings do not move and its eye is still, at the latest beat and the
   --  one before (Kinematics.Held_Still). A rendered view can keep changing
   --  for beats after the camera stopped. Like Settle it waits as long as
   --  that takes; an arm that carries no eye is not held.

   type Probe_Report is record
      Seen      : Boolean := False;   --  some eye saw the channel move
      Excursion : Real := 0.0;        --  how far it was taken, in reading units, when seen
      Steps     : Natural := 0;       --  how many doublings that took
      Last      : Step_Report;        --  the last step's report
   end record;

   function Hold_Of (M : Model; G : Group_Id; Channel : Positive) return Real;
   --  What the group holds the channel at now: the target in effect, or its
   --  reading when the group was never commanded. Moves are taken from here,
   --  so a return lands where the body was held, not where it sagged to.

   type Channel_Ref is record
      Group   : Group_Id := 1;
      Channel : Positive := 1;
   end record;

   type Channel_Refs is array (Positive range <>) of Channel_Ref;

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report)
     with Pre => Direction /= 0.0 and then First >= 0.0;
   --  Takes every listed channel away from its hold by one common amount, in
   --  the sign of Direction, doubling the amount from First (from the
   --  smallest step every listed reading can tell from its noise when First
   --  is 0) until some eye sees the body move: the smallest move worth
   --  pushing by, found without knowing any unit. A level counts as seen when
   --  that many moves in a row, alternating between the hold and the amount,
   --  were each seen as make such a run rarer than Z's tail over every level
   --  a probe may try; one move is a false alarm with a chance of at most the
   --  sum, over the verdicts it looked at, of one in (rest counts + 1) for an
   --  eye whose counts at rest are measured, else Z's tail (Lockin.Moved).
   --  Each move looks for as long as a response takes to show: the longest
   --  measured push delay of a listed group, plus the longest image lag, plus
   --  the beat itself. A channel stops following at its own end: asked
   --  further, its reading went no further than a smaller offset took it, as
   --  far as anything can tell (by a step an eye watching it can see, or,
   --  watched by none, significantly against the readings' noise), and no
   --  longer moves; it is held where it last followed and takes no further
   --  part. The probe ends when an eye saw the
   --  body, when no channel follows any more, or after as many doublings as a
   --  float has bits of precision, and returns the body to the hold. A
   --  channel of a group that is not commandable, or that the group does not
   --  have, takes no part; with none left, nothing is probed (Steps = 0).

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report;
      Answers   : out Real_Array)
     with Pre => Direction /= 0.0 and then First >= 0.0 and then Answers'Length = Channels'Length;
   --  The same, and for every listed channel the amount of the level at
   --  which its reading first followed (went further along the ask than any
   --  smaller offset took it, by the test above); 0 for one that never did.

   procedure Probe (M : in out Model; G : Group_Id; Channel : Positive; Direction : Real; Report : out Probe_Report)
     with Pre => Direction /= 0.0;
   --  Probe_Together with that one channel, from its own noise.

   type Sense is (Increasing, Decreasing);
   type Sense_Counts is array (Sense) of Natural;
   type Sense_Flags is array (Sense) of Boolean;

   type Two_Way_Report is record
      Seen       : Boolean := False;   --  some eye saw the channel move
      Excursion  : Real := 0.0;        --  how far it was taken, in reading units, when seen
      Seen_Sense : Sense := Increasing;   --  which way, when seen
      Answered   : Real := 0.0;        --  the smallest amount at which its reading first followed, either way; 0 when never
      Levels     : Sense_Counts := [others => 0];    --  how many levels each way was asked
      At_End     : Sense_Flags := [others => False];  --  the way stopped because the other one answered while it delivered nothing
      Dead       : Boolean := False;   --  it answered neither way up to where every other channel of the body did
      Last       : Step_Report;        --  the last step's report
   end record;

   procedure Probe_Both_Ways
     (M      : in out Model;
      Ref    : Channel_Ref;
      First  : Real;
      Bound  : Real;
      Report : out Two_Way_Report)
     with Pre => First > 0.0;
   --  Probes one channel both ways, each way by the levels Probe_Together
   --  takes (doubling from First, looked at and confirmed alike, each way
   --  ending at its own end), until an eye sees it. The increasing way goes
   --  first at every level; the decreasing way is asked at a level only
   --  while the increasing one has delivered nothing (a channel that
   --  follows one way needs no other), or once it has ended. A limit is
   --  one-sided: a way that has delivered nothing up to the level at which
   --  the other one answered is at its end there, and its doubling stops (a
   --  closer resting at its upper limit answers downwards at once). A
   --  deadband is two-sided: small asks fail both ways and larger ones
   --  succeed, so both ways go on doubling while neither answers. Bound is
   --  where every other channel of the body has answered (the largest
   --  amount at which another channel first followed; Real'Last when none
   --  did): a channel that has answered neither way when both have been
   --  asked at least that much is dead or disconnected, and the probe stops.
   --  The body returns to the hold.

   procedure Gather_Rest (M : in out Model; Probes : Natural);
   --  Holds the body still for as long as one more still beat shortens the
   --  confirmations of that many probes by more beats than it costs (an
   --  eye's false alarms are bounded by its counts at rest, so a seen run is
   --  shorter the more of them there are), then re-estimates.

   type Pose_Goal is record
      Pose          : Rigid;
      Position_Only : Boolean := False;   --  orientation free
   end record;

   type Plan_Status is (Planned, Unreachable, Unmeasured);

   type Plan is private;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal) return Plan;
   --  A joint path from the arm's configuration at O to the goal, solved
   --  along the way so it never jumps between solution branches.

   function Status (P : Plan) return Plan_Status;
   function Why (P : Plan) return String;
   --  For Unreachable: which limit, joint or distance stopped it.

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report)
     with Pre => Status (P) = Planned;
   --  Moves along a planned path, step by step, judging every step.

private

   package Waypoint_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Real_Array);

   type Plan is record
      State     : Plan_Status := Unmeasured;
      Reason    : Unbounded_String;
      Group     : Group_Id := 1;
      Waypoints : Waypoint_Vectors.Vector;   --  the arm's targets in turn, the goal last
   end record;

end Driver.Robot.Motion;
