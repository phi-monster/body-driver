--  Self test of Driver.Robot.Kinematics.Fixed: the lens and pose of an eye
--  that stands still, measured from points it shares with an arm's eye, the
--  start from a plane, the distortion kept only when significant, and the
--  covariance against the truth.

with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Fixed.Tests is

   procedure Register;

   function Deviate (R : Fit_Report; L : Fit.Lens; T : Rigid) return Real;
   --  How many sigmas away from the truth (the lens L, the eye's pose T in the
   --  world) a fit lies: the error of its terms against its own covariance,
   --  as a Gaussian deviate of the same tail as the chi square of the whole
   --  error. Z is the most it may be. Real'Last when the covariance is not
   --  positive.

end Driver.Robot.Kinematics.Fixed.Tests;
