with Driver.Action.Contact.Search.Tests;
with Driver.Action.Contact.Simplex.Tests;
with Driver.Action.Contact.Wrench.Tests;

package body Driver.Action.Tests is

   procedure Register is
   begin
      Driver.Action.Contact.Simplex.Tests.Register;
      Driver.Action.Contact.Wrench.Tests.Register;
      Driver.Action.Contact.Search.Tests.Register;
   end Register;

end Driver.Action.Tests;
