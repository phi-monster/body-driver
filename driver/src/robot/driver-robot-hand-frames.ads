--  Points and lines of sight taken from one frame into another, with their
--  uncertainty.
--
--  A lobe's tip is seen in an eye; the hand keeps it in the tool frame of the
--  eye's arm, so it moves with the arm, in the arm's unit, and gives it in
--  the world through the tool's pose and the arm's unit. Each move adds only
--  the uncertainty of the pose it goes through: the eye's pose in the tool
--  frame carries the mount alone, since the arm's own kinematics carries both
--  the eye and the tool and cancels.

package Driver.Robot.Hand.Frames is

   function Into (Frame : Pose_Estimate; Point : Point_Estimate) return Point_Estimate;
   --  A point given in a frame, in that frame's parent: the pose's position
   --  and its turn's lever on the point add to the point's own uncertainty.
   --  Unknown in, unknown out.

   function Into (Frame : Pose_Estimate; Line : Ray_Estimate) return Ray_Estimate;
   --  A line of sight given in a frame, in that frame's parent: its origin
   --  as a point, and the pose's turn across the line adds to the direction's
   --  uncertainty. Unknown in, unknown out.

   function Into_World
     (Tool   : Pose_Estimate;
      In_Arm : Pose_Estimate;
      Unit   : Estimate;
      Point  : Point_Estimate) return Point_Estimate;
   --  A point of an arm's tool frame, measured in the arm's own unit, in the
   --  world. Tool is the tool's pose in the world (Driver.Robot.Tool_Pose),
   --  its position in world lengths; In_Arm its pose in the arm's own frame
   --  (Tool_In_Arm); Unit the world length of the arm's unit (Arm_Unit). The
   --  point's offset from the tool is multiplied by the unit, where the pose
   --  already has it. The unit's uncertainty moves the tool's place and the
   --  point's offset together, so what it adds to the point beyond the
   --  pose's own is the offset's share and the share the two have in common.
   --  Unknown in, unknown out: a tool not placed in the world, a unit not
   --  measured.

end Driver.Robot.Hand.Frames;
