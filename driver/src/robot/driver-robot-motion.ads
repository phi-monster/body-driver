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

   procedure Note_Stopped (M : in out Model; G : Group_Id; Noted : out Boolean);
   --  The decider's word that the group's latest push, ended Blocked or Short, was stopped by the body itself and
   --  not by a surface it was pressing: an aim, a step in free air, a probe. Only the caller can know that: a
   --  table and a joint's end both leave a push short. It finds, from the stream alone, the channel that stopped
   --  the push (Driver.Robot.Steps.Limiter: the channel whose part of the shortfall along the ask, its share of the
   --  ask times its own shortfall, is more than Z standard deviations above zero and above that of every other
   --  channel, from the readings' noise and the scatter of the group's free pushes; the only channel asked is that
   --  channel) and notes where it stopped, in the sense it was asked, as an end of the channel
   --  (Driver.Robot.End_Of). Nothing is noted for
   --  a push given up while its readings kept moving, one that was not blocked, or one no channel dominates (the
   --  shortfall spread over the channels). The furthest stop seen of a sense stands: a stop where the arm met
   --  itself is relaxed by a later one that went further. Like every decider call it is made between Next and
   --  Send; it waits for nothing and asks for no estimate.
   --  Noted is True when the stop is a new end or moves one further: plans past it are refused from now, and a
   --  decider whose aim stopped may plan the same aim again, now turned about the way down past the end it showed.

   procedure Note_Stopped (M : in out Model; G : Group_Id);
   --  The same, for a caller that does not ask whether it moved an end.

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
   --  part. A reading whose noise the model has not measured tells nothing of
   --  whether it followed (the test above is against the noise), so a listed
   --  channel in that state has the noise measured first, from every beat so
   --  far (Estimate_Now), before anything is moved. The probe ends when an
   --  eye saw the body, when no channel follows any more, or after as many
   --  doublings as a float has bits of precision, and returns the body to the
   --  hold. A channel of a group that is not commandable, or that the group
   --  does not have, takes no part; with none left, nothing is probed
   --  (Steps = 0).

   procedure Probe (M : in out Model; G : Group_Id; Channel : Positive; Direction : Real; Report : out Probe_Report)
     with Pre => Direction /= 0.0;
   --  Probe_Together with that one channel, from its own noise.

   subtype Sense is Driver.Robot.Sense;
   function Increasing return Sense renames Driver.Robot.Increasing;
   function Decreasing return Sense renames Driver.Robot.Decreasing;
   --  The way a reading is asked to go (Driver.Robot.Sense: a channel's end is found that way).
   type Sense_Counts is array (Sense) of Natural;
   type Sense_Flags is array (Sense) of Boolean;

   type Two_Way_Report is record
      Seen       : Boolean := False;   --  some eye saw the channel move
      Excursion  : Real := 0.0;        --  how far it was taken, in reading units, when seen
      Seen_Sense : Sense := Increasing;   --  which way, when seen
      Answered   : Real := 0.0;        --  the smallest amount at which its reading first followed, either way; 0 when never
      Levels     : Sense_Counts := [others => 0];    --  how many levels each way was asked
      At_End     : Sense_Flags := [others => False];  --  the way stopped because the other one answered while it delivered nothing
      Dead       : Boolean := False;   --  its reading followed no ask either way, at any level, though its noise is measured
      Blind      : Boolean := False;   --  its reading's noise is not measured: nothing is known of how its reading followed
      Last       : Step_Report;        --  the last step's report
   end record;

   procedure Probe_Both_Ways
     (M      : in out Model;
      Ref    : Channel_Ref;
      First  : Real;
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
   --  succeed, so both ways go on doubling while neither answers. Nothing
   --  the other channels of the body needed bounds a channel: it is pushed on
   --  until an eye sees it, or its own reading ends each way (a way that
   --  followed and then went no further), or, answering neither way, both
   --  ways have been asked as many levels as a float has bits of precision;
   --  then it is dead or disconnected for this boot (Dead), as far as its
   --  reading's noise tells a following from none. A reading whose noise the
   --  model has not measured has it measured first, from every beat so far
   --  (as Probe_Together does); a channel whose noise stays unmeasured is
   --  Blind, pushed on by what the eyes see alone, never called dead. The
   --  body returns to the hold.

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

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal;
                        Clearance : Real := Real'Last; Lever : Real := 0.0) return Plan;
   --  A joint path from the arm's configuration at O to the goal, a pose of
   --  the tool in the world (Tool_Pose), solved along the way so it never
   --  jumps between solution branches: a joint that turns is given as the
   --  reading nearest where the last waypoint left it (Kinematics.
   --  Nearest_Readings), not as one of the turns the solver's steps came to.
   --  The path is the fitted model's, within the readings the arm has shown or
   --  beyond them: a joint's end on the way is met when the path is followed,
   --  and the step that meets it ends Blocked or Short. The goal is taken into
   --  the arm's own frame by the arm's placement and planned there
   --  (Plan_Reach_In_Arm), so a move given relative to the tool's pose in the
   --  world is the same move whatever the placement.
   --
   --  The arm is taken from one waypoint to the next along the joints' own
   --  straight line, which bows from the tool's straight line (A17's left
   --  arm bowed by up to a fifth of a move). Clearance is how far the tool
   --  may leave the straight path to the goal (straight in position, about
   --  one axis in turn), and with its turn the body out to Lever from it,
   --  in the unit of the goal's pose (Real'Last: not bounded): a segment is
   --  cut in two until the tool, at the quarter, the middle and the three
   --  quarters of the joints' straight line, is within the clearance of where
   --  the straight path has it. A segment the solver cannot close is cut in
   --  two only while it is longer than the fit can tell apart (its
   --  Angle_Sigma): shorter, cutting it again learns nothing, and the goal is
   --  called unreachable there. A bow that cannot be cut away because the
   --  straight path leaves what the arm can reach is left: the joints' own
   --  line, which does reach, is taken, and the plan says how large it is
   --  (Worst_Bow).
   --
   --  A path that passes an end a channel has shown (Driver.Robot.End_Of) by
   --  more than the end's sigma and the waypoint's is refused as Unreachable,
   --  saying which channel and where; until a channel has shown one a path is
   --  free past the readings seen. A waypoint at the end, within the noise, is
   --  planned.
   --
   --  The segments of a chain are solved one from where the last ended, so
   --  the chain is continuous: with a clearance, a joint that the straight
   --  tool path winds can end a whole period from where one solve for the
   --  goal would put it. The driver knows no joint limits (a limit is met as
   --  Blocked or Short), so a wrist limited to plus or minus pi, asked for a
   --  straight path that winds it, stops at its end. That is the trade a
   --  caller makes by asking for a clearance.

   function Plan_Reach_In_Arm (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal;
                               Clearance : Real := Real'Last; Lever : Real := 0.0) return Plan;
   --  The same, the goal a pose of the tool in the arm's own frame
   --  (Tool_In_Arm): it needs only the arm's kinematics, not its placement in
   --  the world; the Clearance and the Lever are in the arm's own unit.

   function Status (P : Plan) return Plan_Status;
   function Why (P : Plan) return String;
   --  For Unreachable: how far the model leaves the goal; for Unmeasured:
   --  what is not measured yet.

   function Last_Readings (P : Plan) return Real_Array
     with Pre => Status (P) = Planned;
   --  The readings the plan ends at: where the fitted model puts the goal.

   function Waypoint_Count (P : Plan) return Natural;
   function Waypoint (P : Plan; K : Positive) return Real_Array
     with Pre => K <= Waypoint_Count (P);
   --  The arm's targets in turn, the goal last: what Follow sends.

   function Solve_Count (P : Plan) return Natural;
   --  How many times the solver was asked to close a segment of the path:
   --  what planning cost, whether it ended Planned or Unreachable.

   function Stray
     (M : Model; A : Arm_Id; From, To : Rigid; Q0, Q1 : Real_Array; Lever : Real := 0.0; Position_Only : Boolean := False)
      return Real
     with Pre => Q0'Length = Q1'Length and then Q0'First = Q1'First;
   --  How far the tool strays from the straight path from From to To (straight
   --  in position, about one axis in turn, in the arm's own frame and unit)
   --  when the arm goes from the readings Q0 to Q1 along the joints' straight
   --  line: at its quarter, its middle and its three quarters (a bow that goes
   --  out one side and back across has no stray at its middle), the largest
   --  distance of the tool from where the straight path has it at the same
   --  share of the way, and with a turn the body out to Lever from the tool.
   --  What Plan_Reach cuts a segment by.

   function Worst_Bow (P : Plan) return Real;
   --  The largest bow a segment of the plan is left with above the clearance
   --  it was asked for, because the straight path there leaves what the arm
   --  can reach: how far the tool may stray from the straight path, in the
   --  goal's unit (the arm's own for Plan_Reach_In_Arm). Zero when every
   --  segment keeps within the clearance, or none was asked.

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
      Solves    : Natural := 0;              --  segments the solver was asked to close
      Bow       : Real := 0.0;               --  the largest bow left above the clearance
   end record;

end Driver.Robot.Motion;
