--  Hands: closer groups whose channels bring two or more lobes towards each
--  other around a region that can hold something (LANGUAGE.md 3.2, the
--  grasper). A two-finger gripper has two lobes, a five-finger hand has
--  several closable groups of fingers, a suction cup has one lobe.
--
--  Measured by the body itself, from the recorded stream alone: the two ends
--  of each closer channel's travel seen still by an eye on its arm, what
--  changed between them, what that eye shows of the robot itself as its arm
--  moves, and the lobes they give (Driver.Robot.Hand.Sweep, Selfsight); each
--  lobe's tip at both ends, as a pixel and as
--  a line of sight in the tool frame; and, from the beats a press on a
--  surface was blocked, where along that line the tip is
--  (Driver.Robot.Hand.Touch). Until a quantity is measured it is reported
--  unknown. Path B owns this package; Driver.Robot.Boot calls Measure, the
--  decider that sweeps and presses.
--
--  What a hand is, it is in its tool frame and its arm's own unit: the
--  presses that measure it are made, and its surface fitted, in the arm's own
--  frame (Driver.Robot.Tool_In_Arm, Table_In_Arm), which needs the arm's fit
--  and nothing of where the arm stands in the world. Only Tip, Tip_Now and
--  Grip_Centre take it into the world, through the arm's placement and unit.

with Driver.Clock;
with Driver.Commands;
with Driver.Images;

private with Ada.Finalization;

package Driver.Robot.Hand is

   type Hand_Id is new Positive;

   type Hands is tagged limited private;

   procedure Observe (H : in out Hands; M : Model; O : Observation; Sent : Driver.Commands.Command);
   --  Estimators only, one beat; Sent as in Driver.Robot.Observe.

   procedure Measure (H : in out Hands; M : in out Model);
   --  Decider: finds and measures every hand of the booted body.

   function Hand_Count (H : Hands) return Natural;
   function Exists (H : Hands; Id : Hand_Id) return Boolean;
   --  The hand Id is there now. The estimators drop and renumber hands between two held beats (the roles are
   --  read again, a recompute of the heavier estimates; A29), so a decider that keeps a Hand_Id from an earlier
   --  beat asks before it reads: the queries below, asked of an Id past the last hand, raise.

   function Asks_A_Translation (M : Model; A : Arm_Id; From, Target : Real_Array) return Boolean;
   --  A push of the arm from readings From to readings Target asks the tool a translation: the rotation of the tool
   --  at Target is that at From, to what the arm's fit can tell (Kinematics.Angle_Sigma, Z times it), as the steps of a
   --  descent are (Pressing.Lowered) and the turn of an aim about the eye is not. True when the poses are not known.
   --  A block counts toward a press only when the push that was blocked asked a translation (Observe).
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id;
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id;
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive;
   function Presses_Kept (H : Hands; Id : Hand_Id) return Natural;
   --  How many presses the hand's book keeps: the blocks of its descents, found in the stream.

   type Opening is (Open, Closed_Empty);

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate;
   --  A lobe's tip in the world frame with the arm as at O, in the world's
   --  lengths (the arm's unit taken in, Frames.Into_World). Not known while the
   --  arm is not placed in the world.

   function Tip_Now (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate;
   --  A lobe's tip at the closer reading of O, on the straight path between
   --  its two measured ends in the proportion its channel is closed; in the
   --  world as Tip is.

   type Tip_Kind is (Loaded, Free);
   --  Which finger a tip is of. A pressed finger slides along its own axis
   --  under the press (A16: 5 and 17.5 mm inward, the closer's reading
   --  exactly where it was), so the tip a press gives is that of the finger
   --  as it stood under it:
   --    Loaded  the contact of the finger under the press, at the beat of the
   --            press the tip rests on (Tip_Beat);
   --    Free    the finger unloaded, at the closer's reading: every press
   --            taken with the tip it had slid to (the share of the lobe's
   --            travel between its openings the eye measured under it).
   --            Unknown until the lobe has a tip at each opening (the
   --            travel is their difference) and the presses the tip rests
   --            on have their slide measured.

   function Tip_In_Tool
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded)
     return Point_Estimate;
   --  A lobe's tip in the tool frame of the hand's arm (Driver.Robot.Tool_Pose),
   --  in the arm's own unit (Driver.Robot.Arm_Unit): what the hand measured,
   --  before any arm pose is applied. Known from the first press the tip
   --  stopped, and provisional until Tip_Confirmed. Its covariance along the
   --  surface's normal is the contact's, and across the line of sight to the
   --  tip pixel the spread of the lobe's tip region as well as the eye's.

   function Tip_Beat
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Driver.Clock.Beat;
   --  The beat of the press the tip of that kind rests on, the one among them
   --  that gave the lowest hit: the finger as it stood then is the loaded
   --  tip's. Zero when the tip is not known.

   type Slide_Reading is record
      Beat           : Driver.Clock.Beat;   --  of the press
      Contact        : Boolean;             --  the press went to this lobe: it was the one that pressed
      Tip_Rests      : Boolean;             --  and the lobe's tip rests on it
      Known          : Boolean;
      Pixels         : Real;                --  how far the lobe's tip region moved in its eye's picture along the way
      Pixels_Sigma   : Real;                --  it closes in, from where the closer's reading puts it: positive inward
      Fraction       : Real;                --  as a share of the travel between its openings in that picture
      Fraction_Sigma : Real;
   end record;
   --  The finger's slide under a press. It is what a press loaded the finger
   --  with, as the finger gives way: the compliance of the hand, for the
   --  action layer's contact model, kept for every press at the opening.

   type Slide_Readings is array (Positive range <>) of Slide_Reading;

   function Slides (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Slide_Readings;
   --  Every press made at that opening, in the order they were made, with
   --  how far the lobe's finger had slid under it.

   function Tip_Confirmed
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean;
   --  Tip_In_Tool's tip of that kind is checked: a second press, from a pose
   --  distinct from the first's, landed on it within the noise (the free tip's
   --  holds the uncertainty of the slides the presses were corrected by). A tip
   --  one press fixes is a bound the tip is not beyond, and the lowest one the
   --  presses gave.

   function Tip_Tested
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean;
   --  The presses the tip of that kind rests on have told where it is across its line of sight, along both axes
   --  (Tips.Tested): a tip is Confirmed only when they have.

   function Tip_Across
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Real_Array;
   --  The tip's standard deviations across its line of sight, the axis the presses told most and the one they told
   --  least (Tips.Across), in the arm's own unit.

   function Press_Direction (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening)
     return Direction_Estimate;
   --  The direction the tip was pressed along when it was measured, in the
   --  tool frame: the tip is the point of the lobe that leads along it.

   function Closer_Reading (H : Hands; Id : Hand_Id; At_Opening : Opening) return Real_Array;
   --  The closer group's readings at that opening, the ones Tip refers to.

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate;
   --  The middle of the region the lobes close on, at the opening of O, in
   --  the world as Tip is.

   function Own_Eye (H : Hands; Id : Hand_Id) return Eye_Id;
   --  The eye on the hand's arm its lobes were found in.

   function Tip_Pixel (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Driver.Images.Pixel;
   --  Where the lobe's tip is in the own eye at that opening.

   function Tip_Sight (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Ray_Estimate;
   --  The line of sight to that tip, in the tool frame: the tip lies on it.

   --  The hand's sizes (Driver.Robot.Hand.Shape): from the surface of each
   --  lobe its own eye sees, fixed by the two views of the closer's sweep
   --  and scaled by the tips the presses measure at both openings. Each is
   --  at least what is seen of it: a side of a lobe facing away from its eye,
   --  or a part hidden or outside the picture, is not in it. Unknown until
   --  measured; the log (Describe) says why.

   function Lobe_Width (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate;
   --  How wide the lobe is across its closing direction (from its open tip
   --  toward its closed one) and across the hand's axis.

   function Lobe_Thickness (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate;
   --  How thick it is along its closing direction.

   function Lobe_Face (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate;
   --  How far ahead of its tip along its closing direction it reaches: where
   --  its face meets what it closes on.

   function Grip_Depth (H : Hands; Id : Hand_Id) return Estimate;
   --  How far back from the lobes' tips along the hand's axis their moving
   --  parts reach: how far a thing may go in between them before it meets
   --  them where they begin, from the middle of the tips at the open opening.
   --  Known when every lobe is measured.

   function Grip_Axis (H : Hands; Id : Hand_Id) return Direction_Estimate;
   --  The way the lobes point, in the tool frame: from the middle of their
   --  points toward their tips. A thing goes into the hand against it.

   function Describe (H : Hands) return String;
   --  The hands found so far, one line each, for the log.

private

   Mad_Efficiency : constant := 0.367_5;
   --  The asymptotic efficiency of the median absolute deviation for Gaussian
   --  data: a scale from it is worth that share of as many degrees of freedom.

   type Hand_Data;
   --  Completed in the body: the hand's measurements use its child packages.

   type Hand_Data_Access is access Hand_Data;

   type Hands is new Ada.Finalization.Limited_Controlled with record
      Data : Hand_Data_Access;
   end record;

   overriding procedure Finalize (H : in out Hands);

   --  A lobe's tip at one opening: its pixel in the own eye and its line of
   --  sight in the tool frame.
   type Sight is record
      Known  : Boolean := False;
      Pixel  : Driver.Images.Pixel;
      Ray    : Ray_Estimate;
      Spread : Real := 0.0;   --  of the lobe's tip region from that pixel, per axis, as an angle (Tips.Sight_Of)
      Pitch  : Real := 0.0;   --  the angle a pixel spans there
   end record;

   type Sight_Array is array (Opening) of Sight;

   type Sight_Rows is array (Positive range <>) of Sight_Array;
   --  One row per lobe.

   procedure Adopt
     (H         : in out Hands;
      Group     : Group_Id;
      Arm       : Arm_Id;
      Eye       : Eye_Id;
      Open_At   : Real_Array;
      Closed_At : Real_Array;
      Lobes     : Sight_Rows);
   --  The hand of a closer group as its sweep leaves it: the group's
   --  readings at each opening and every lobe's tips seen at both. What the
   --  sweep makes of the views ends here, and what is done with a hand begins
   --  (its presses, its sizes); the tests start there.

   procedure Press_Beat
     (H          : in out Hands;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean := False);
   --  One beat of one hand's arm for its presses: Observe's step, given the
   --  judgments of the beat on its arm (Driver.Robot.Blocked, its readings
   --  at rest, its latest push under way), from which a press is found.

   type Showing is (Not_Yet, Nothing_New, Something_New);
   --  What the views of a closer's own eye make of a push: Not_Yet while the
   --  view taken since it cannot be judged (Driver.Robot.Hand.Views.Gathered),
   --  or whether it shows the eye something its last end did not.

   procedure Sweep_Way
     (Way          : Real;
      Step         : Real;
      Seen_By      : Real;
      Wait_At_Most : Positive;
      Push         : not null access procedure (Offset : Real; Followed : out Boolean);
      Shows        : not null access function return Showing;
      Pushes       : out Natural;
      Unseen       : out Natural;
      Longest_Wait : out Natural;
      Formed       : out Boolean;
      Answered     : out Boolean)
     with Pre => Way /= 0.0 and then Step > 0.0 and then Seen_By >= 0.0;
   --  One way of a closer channel's sweep: pushed Way by Step, then by twice
   --  as much each time, from where the sweep began, for as long as the
   --  channel's reading follows each push (Push says whether the body judged
   --  it moved along the ask) and each shows the eye something the last end
   --  did not. The first push the reading does not follow is the channel's
   --  end this way, and the doubling stops there: a controller that does not
   --  clamp takes a command past the end as full effort.
   --
   --  After each push Shows is asked again, a beat later each time, for as
   --  long as it cannot say: the push's view forms only once the eye's
   --  picture has come to rest from it. A11 asked it once, two beats after
   --  each push, while its view was a frame old: nothing new, whatever the
   --  push. It is asked at most Wait_At_Most times (the caller measures how
   --  long a view takes to form); when the view has not formed by then,
   --  Formed is False and the way ends there, nothing more pushed: A12's
   --  views never formed, and its sweep waited for them for an hour.
   --  Longest_Wait is the most askings any push of this way needed.
   --
   --  Step is the smallest push any eye can see the channel make at all
   --  (Visible_Step): the lock-in's, which tells it from many beats of
   --  pushing, far below what one pair of still views shows. So the first
   --  pushes may show the views nothing; they double on while the reading
   --  follows until one does, and the views tell the ends from then on. They
   --  go no further than Seen_By, the push that moves the channel's view by
   --  a pixel: a channel that moves shows a pair of views by then, and one
   --  that shows nothing is stuck there, at an end whose reading echoes the
   --  command. A11's first push, 1.7e-5 of the travel, moved its view by
   --  two thousandths of a pixel. Pushes counts the pushes, Unseen those
   --  before the views showed anything; Answered, whether the first push
   --  was followed.

   procedure Free_Closer
     (Arrived    : not null access function return Boolean;
      Reading    : not null access function return Real;
      Moved      : not null access function (Before, After : Real) return Boolean;
      Ask        : not null access procedure;
      Raise_Hand : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises     : out Natural);
   --  A closer asked back to a reading it has been at: asked (Ask settles it),
   --  and while it has not Arrived, held by something. A finger resting on the
   --  table is held by it, and cannot slide along it (A17: asked back to 1.0,
   --  it stood at 0.59 to 0.686 for seventy beats, and reached 1.0 in four
   --  once the aim had lifted the hand). The hand is raised (Raise_Hand, the
   --  first raise the least, each after it twice the one before; Raised False
   --  when it cannot be raised further) and the closer asked again, while
   --  each raise sets the closer moving, until it arrives: a raise after which
   --  its reading has not moved (Moved) was not what held it, and the hand is
   --  raised no more. Raises is how many raises were made.

   procedure Gather_Poses
     (Wanted       : Positive;
      Wait_At_Most : Positive;
      Poses        : not null access function return Natural;
      Raise_Eye    : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises       : out Natural;
      Reached      : out Boolean);
   --  The arm moves the eye so that it sees a closer's readings from Wanted
   --  poses of the rest of the body (Selfsight: what a deviation of the robot
   --  from its surroundings is taken over). A hand is measured from its own
   --  arm's motion and not from what the boot did before it: the boot's sweeps
   --  gave A22's closers dozens of poses at their start readings, and a body
   --  file that is reloaded gives none (A25h logged "0 poses, two are needed"
   --  at every round). While the eye has fewer, the eye is raised (Raise_Eye,
   --  the first raise the least that shows, each after it twice the one before;
   --  Raised False when it cannot be raised) and the pose is asked after, a beat
   --  later each time, until the eye's picture has rested and its frame is kept
   --  (Poses grew), or Wait_At_Most beats went by (the caller measures how long
   --  a view takes to form). Poses is read inside a held beat, so each asking
   --  is a beat. Raises is how many raises were made, Reached whether the eye
   --  has Wanted poses at the end.

   function Seen_By (Shift : Real) return Real is (if Shift /= 0.0 then 1.0 / abs Shift else 0.0);
   --  The push that moves an eye's view by one pixel when the channel moves
   --  it Shift pixels a reading unit (Driver.Robot.Lockin.Shift): a whole
   --  pixel's step of a textured patch changes its pixels by the texture's
   --  own contrast, which a still view tells; a fraction of a pixel may
   --  change none. Zero, no blind push, when the shift is not measured.

   type Descent_Steps is record
      Fast    : Natural := 0;   --  doubling, each ending Z sigma or more above the predicted contact
      Band    : Natural := 0;   --  within that band, each the larger of the sigma and Least
      Blind   : Natural := 0;   --  doubling, nothing predicting the contact (before a prediction, or past its band)
      Capped  : Natural := 0;   --  of those, the steps cut to the eye's room above the surface
      Spent   : Boolean := False;   --  ended with the eye no room left to go down, and nothing met
      Stalled : Boolean := False;   --  ended by a step the arm followed and the hand did not go down with
   end record;

   function Total (S : Descent_Steps) return Natural is (S.Fast + S.Band + S.Blind);

   type Heights is record
      Tip     : Estimate := Unknown;   --  the tip above the contact predicted under it, along the way down
      Eye     : Estimate := Unknown;   --  the eye above the surface, along the way down
      Stalled : Boolean := False;      --  the step before was followed by the arm and the hand did not go down with it
   end record;
   --  Stalled is read with the heights, from the judgment of the push the stream gave
   --  (Driver.Robot.Hand.Lowering): the arm's readings went where the step asked and the
   --  hand lies on what it met.

   procedure Descend
     (Above : not null access function return Heights;
      Least : Real;
      Lower : not null access procedure (By : Real; Reached : out Boolean);
      Steps : out Descent_Steps)
     with Pre => Least > 0.0;
   --  A press's descent: each step lowers the tool By, until one does not
   --  reach (Lower says so: the arm met something, or cannot go there) or the
   --  hand does not go down with it (stalled: the press is made, a bound or a
   --  contact, whichever the presses tell, and the descent is not spent). Above
   --  is read before each step. Tip is the tip's height above the contact
   --  predicted under it, with its sigma, Unknown when nothing predicts it.
   --  With a prediction, the steps double from Least for as long as each ends
   --  Z sigma or more above the predicted contact, the last cut to end there;
   --  within that band each step is the larger of the sigma and Least, so the
   --  tip meets the surface at most that far short of a step's end: the
   --  overshoot the prediction already admits, and less force and less sinking
   --  in where the tip is read. The band is as wide as the prediction says, Z
   --  sigma either side of the contact: past it, with nothing met, the
   --  prediction was a bound the surface is farther than (the press that fixed
   --  the tip was stopped by something under it, A17's cutter), and the steps
   --  double again from the band's, counted as blind; steps of the band's size
   --  went on for 6000 beats there, each reached as the hand slid along the
   --  table. Without a prediction the steps double from Least until
   --  one is not reached, the overshoot as it comes (the owner's rule, 10-05:
   --  the most aggressive choice everywhere; creeping by Least took A11's first
   --  presses into the thousands of pushes). Least is the smallest move of the
   --  tool that tells from its noise.
   --
   --  Eye is how far the eye can still go down before it reaches the surface
   --  the arm's own eye saw: whatever the tip is, the hand can be lowered no
   --  further than that, and a step to a pose with the eye under the table
   --  asks for a pose no hand can be in. Every step is cut to end Z sigma of
   --  that height above the surface, and the descent ends, Spent, when less
   --  than Least is left. (A16's third press doubled to 5.85 units after
   --  5.80 with the eye 10.0 units above the table: a lowering to 11.65, 5.68
   --  rad of joint motion, of which the arm made 13.6 per cent before it
   --  folded on itself.)

   function Sweepable (H : Hands; M : Model; G : Group_Id) return Boolean;
   --  The group is a closer by the body's roles now, and the hand watches it
   --  in an eye its arm, as the body has it now, carries: the only groups
   --  swept as closers.

end Driver.Robot.Hand;
