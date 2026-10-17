--  Self test of Driver.Robot.Kinematics.Hand_Eye: the lens and pose of an eye that stands still, measured from the
--  arms it watches (points that ride on their links, seen at the arms' keyframes), the links read from the
--  keyframes, two arms through one lens, the arms' own uncertainty carried, and the covariance against the truth.

with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Hand_Eye.Tests is

   procedure Register;

   type Pose_Array is array (Positive range <>) of Rigid;

   function Deviate (R : Fit_Report; L : Fit.Lens; Poses : Pose_Array) return Real;
   --  How many sigmas away from the truth (the lens L, the eye's pose in each arm's frame in Poses) a fit lies: the
   --  error of its terms against its own covariance, as a Gaussian deviate of the same tail as the chi square of the
   --  whole error. Z is the most it may be. Real'Last when the covariance is not positive.

end Driver.Robot.Kinematics.Hand_Eye.Tests;
