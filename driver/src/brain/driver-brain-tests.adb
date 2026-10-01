with Driver.Brain.Keyboard.Tests;
with Driver.Brain.Parser.Tests;

package body Driver.Brain.Tests is

   procedure Register is
   begin
      Driver.Brain.Parser.Tests.Register;
      Driver.Brain.Keyboard.Tests.Register;
   end Register;

end Driver.Brain.Tests;
