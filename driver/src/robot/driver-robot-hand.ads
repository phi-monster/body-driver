--  Hands: closer groups whose channels bring two or more lobes towards each
--  other around a region that can hold something (LANGUAGE.md 3.2, the
--  grasper). A two-finger gripper has two lobes, a five-finger hand has
--  several closable groups of fingers, a suction cup has one lobe.
--
--  Measured by the body itself: which lobes a closer moves, where each lobe's
--  tip is when open and when closed on nothing, and the grip region between
--  the lobes. Path B owns this package; Driver.Robot.Boot calls Measure.

with Driver.Commands;

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
   --  A lobe's tip at the closer reading of O.

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate;
   --  The middle of the region the lobes close on, at the opening of O.

private

   type Hands is tagged limited record
      Count : Natural := 0;
   end record;

end Driver.Robot.Hand;
