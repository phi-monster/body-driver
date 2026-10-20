--  What a closer's flicks place: the eyes fixed in the world, by the first arm's, and the other arms.
--
--  A fixed eye is placed from where it sees a point of the arm stand at many poses of the arm. The point is the
--  closer's: the arm holds still at a pose, the closer goes to its other end and the eye's two still pictures, before
--  and after, differ where the closer's fingers moved; the centre of what differs is one sighting of a point fixed to
--  the arm's own eye (Kinematics.Watch_Flicks, Kinematics.Beacon). The sightings are the poses' worth: a lens and a
--  place are told by a point that goes through the eye's picture and through its depth, and the range the arm's joints
--  were read over while the arm was measured is a few centimetres of a metre (k31, 10-20: 41 flicks over 7 by 3 by 8
--  cm at 90 cm told the focal length of a camera at 153 pixels as 352 +- 160). Place first finds how far each joint
--  goes, then flicks at the poses between and at its ends.
--
--  Once an eye is placed the same poses and flicks of another arm's closer put that arm into the world
--  (Kinematics.Placement): X_world = Placement * (Scale * X_arm).

package Driver.Robot.Flicks is

   procedure Place (M : in out Model; Arm : Arm_Id);
   --  Moves the arm through poses and flicks its closer at them while what its flicks place is not placed, and the
   --  poses last: for the first arm, some fixed eye that sees it move that is not placed (Kinematics.Fixed_Known);
   --  for another, the arm itself, not in the world (Kinematics.In_World) while some placed fixed eye sees it.
   --  Nothing happens when the arm is not fitted, has no closer, or there is nothing to place. A decider operation
   --  (Driver.Beats): called from the boot only.

end Driver.Robot.Flicks;
