--  Hands: closer groups whose channels bring two or more lobes towards each
--  other around a region that can hold something (LANGUAGE.md 3.2, the
--  grasper). A two-finger gripper has two lobes, a five-finger hand has
--  several closable groups of fingers, a suction cup has one lobe.
--
--  Measured by the body itself, from the recorded stream alone: the two ends
--  of each closer channel's travel seen still by an eye on its arm, the
--  instrument's correspondences between them and the lobes they show
--  (Driver.Robot.Hand.Sweep); each lobe's tip at both ends, as a pixel and as
--  a line of sight in the tool frame; and, from the beats a press on a
--  surface was blocked, where along that line the tip is
--  (Driver.Robot.Hand.Touch). Until a quantity is measured it is reported
--  unknown. Path B owns this package; Driver.Robot.Boot calls Measure, the
--  decider that sweeps and presses.

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
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id;
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id;
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive;

   type Opening is (Open, Closed_Empty);

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate;
   --  A lobe's tip in the world frame with the arm as at O.

   function Tip_Now (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate;
   --  A lobe's tip at the closer reading of O, on the straight path between
   --  its two measured ends in the proportion its channel is closed.

   function Tip_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Point_Estimate;
   --  A lobe's tip in the tool frame of the hand's arm (Driver.Robot.Tool_Pose):
   --  what the hand measured, before any arm pose is applied.

   function Press_Direction (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening)
     return Direction_Estimate;
   --  The direction the tip was pressed along when it was measured, in the
   --  tool frame: the tip is the point of the lobe that leads along it.

   function Closer_Reading (H : Hands; Id : Hand_Id; At_Opening : Opening) return Real_Array;
   --  The closer group's readings at that opening, the ones Tip refers to.

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate;
   --  The middle of the region the lobes close on, at the opening of O.

   function Own_Eye (H : Hands; Id : Hand_Id) return Eye_Id;
   --  The eye on the hand's arm its lobes were found in.

   function Tip_Pixel (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Driver.Images.Pixel;
   --  Where the lobe's tip is in the own eye at that opening.

   function Tip_Sight (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Ray_Estimate;
   --  The line of sight to that tip, in the tool frame: the tip lies on it.

   function Describe (H : Hands) return String;
   --  The hands found so far, one line each, for the log.

private

   type Hand_Data;
   --  Completed in the body: the hand's measurements use its child packages.

   type Hand_Data_Access is access Hand_Data;

   type Hands is new Ada.Finalization.Limited_Controlled with record
      Data : Hand_Data_Access;
   end record;

   overriding procedure Finalize (H : in out Hands);

   procedure Sweep_Way
     (Way      : Real;
      Step     : Real;
      Push     : not null access procedure (Offset : Real; Followed : out Boolean);
      Extends  : not null access function return Boolean;
      Pushes   : out Natural;
      Answered : out Boolean)
     with Pre => Way /= 0.0 and then Step > 0.0;
   --  One way of a closer channel's sweep: pushed Way by Step, then by twice
   --  as much each time, from where the sweep began, for as long as each push
   --  shows the eye something the last end did not (Extends) and the
   --  channel's reading follows it (Push says whether the body judged it
   --  moved along the ask). The first push it does not follow is the
   --  channel's end this way, and the doubling stops there: a controller that
   --  does not clamp takes a command past the end as full effort. Pushes
   --  counts the pushes; Answered, whether the first one followed.

   type Descent_Steps is record
      Fast  : Natural := 0;   --  doubling, each ending Z sigma or more above the predicted contact
      Band  : Natural := 0;   --  within that band, each the larger of the sigma and Least
      Crept : Natural := 0;   --  by Least, nothing predicting the contact
   end record;

   function Total (S : Descent_Steps) return Natural is (S.Fast + S.Band + S.Crept);

   procedure Descend
     (Gap   : not null access function return Estimate;
      Least : Real;
      Lower : not null access procedure (By : Real; Reached : out Boolean);
      Steps : out Descent_Steps)
     with Pre => Least > 0.0;
   --  A press's descent: each step lowers the tool By, until one does not
   --  reach (Lower says so: the arm met something, or cannot go there). Gap is
   --  the tip's height above the contact predicted under it, with its sigma,
   --  Unknown when nothing predicts it. With a prediction, the steps double
   --  from Least for as long as each ends Z sigma or more above the predicted
   --  contact, the last cut to end there; within that band each step is the
   --  larger of the sigma and Least, so the tip meets the surface at most
   --  that far short of a step's end: the overshoot the prediction already
   --  admits. Without one, every step is Least, the smallest move of the tool
   --  that tells from its noise: nothing says where the surface is.

   function Sweepable (H : Hands; M : Model; G : Group_Id) return Boolean;
   --  The group is a closer by the body's roles now, and the hand watches it
   --  in an eye its arm, as the body has it now, carries: the only groups
   --  swept as closers.

end Driver.Robot.Hand;
