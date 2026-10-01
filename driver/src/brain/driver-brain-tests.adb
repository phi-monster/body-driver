with Driver.Brain.Execution.Tests;
with Driver.Brain.Keyboard.Tests;
with Driver.Brain.Names.Tests;
with Driver.Brain.Parser.Tests;
with Driver.Brain.Runaway.Tests;
with Driver.Brain.Service.Tests;
with Driver.Brain.Termination.Tests;

package body Driver.Brain.Tests is

   procedure Register is
   begin
      Driver.Brain.Parser.Tests.Register;
      Driver.Brain.Keyboard.Tests.Register;
      Driver.Brain.Runaway.Tests.Register;
      Driver.Brain.Names.Tests.Register;
      Driver.Brain.Service.Tests.Register;
      Driver.Brain.Termination.Tests.Register;
      Driver.Brain.Execution.Tests.Register;
   end Register;

end Driver.Brain.Tests;
