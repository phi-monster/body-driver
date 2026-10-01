with Driver.Robot.Hand.Lobes.Tests;
with Driver.Robot.Hand.Sweep.Tests;
with Driver.Robot.Hand.Touch.Tests;
with Driver.Robot.Hand.Views.Tests;

package body Driver.Robot.Hand.Tests is

   procedure Register is
   begin
      Driver.Robot.Hand.Views.Tests.Register;
      Driver.Robot.Hand.Lobes.Tests.Register;
      Driver.Robot.Hand.Sweep.Tests.Register;
      Driver.Robot.Hand.Touch.Tests.Register;
   end Register;

end Driver.Robot.Hand.Tests;
