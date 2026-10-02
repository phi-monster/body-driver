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

end Driver.Robot.Hand;
