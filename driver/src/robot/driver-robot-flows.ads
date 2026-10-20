--  Step 3 for the eyes no arm carries: every eye fixed in the world that sees a fitted arm move and is not placed is
--  placed by the arms' edge flow (Kinematics.Flow_Eye): each arm in turn goes to poses, and at each every joint of its
--  chain turns a little either way while the rest of the body holds still; the eye's still views at the pose and at the
--  two turns of each joint give, at every pixel the arm moves, the picture's change a reading unit of each joint. An
--  eye is fitted to every arm it saw at once, through its one lens.

private package Driver.Robot.Flows is

   procedure Place (M : in out Model);
   --  Nothing when no arm is fitted or every fixed eye that sees one has a place.

end Driver.Robot.Flows;
