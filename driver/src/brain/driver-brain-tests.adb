with Driver.Brain.Keyboard.Tests;
with Driver.Brain.Parser.Tests;
with Driver.Brain.Runaway.Tests;

package body Driver.Brain.Tests is

   procedure Register is
   begin
      Driver.Brain.Parser.Tests.Register;
      Driver.Brain.Keyboard.Tests.Register;
      Driver.Brain.Runaway.Tests.Register;
   end Register;

end Driver.Brain.Tests;
