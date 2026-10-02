--  Points and lines of sight taken from one frame into another, with their
--  uncertainty.
--
--  A lobe's tip is seen in an eye; the hand keeps it in the tool frame of the
--  eye's arm, so it moves with the arm, and gives it in the world through the
--  tool's pose. Each move adds only the uncertainty of the pose it goes
--  through: the eye's pose in the tool frame carries the mount alone, since
--  the arm's own kinematics carries both the eye and the tool and cancels.

package Driver.Robot.Hand.Frames is

   function Into (Frame : Pose_Estimate; Point : Point_Estimate) return Point_Estimate;
   --  A point given in a frame, in that frame's parent: the pose's position
   --  and its turn's lever on the point add to the point's own uncertainty.
   --  Unknown in, unknown out.

   function Into (Frame : Pose_Estimate; Line : Ray_Estimate) return Ray_Estimate;
   --  A line of sight given in a frame, in that frame's parent: its origin
   --  as a point, and the pose's turn across the line adds to the direction's
   --  uncertainty. Unknown in, unknown out.

end Driver.Robot.Hand.Frames;
