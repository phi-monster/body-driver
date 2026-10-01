--  Runs every behavior specification: selftest [name prefix].
--  Exits with a failure status when any test fails.

with Ada.Command_Line;
with Driver.Action.Tests;
with Driver.Brain.Tests;
with Driver.Core_Tests;
with Driver.Geometry.Tests;
with Driver.Robot.Hand.Tests;
with Driver.Robot.Tests;
with Driver.Tests;
with Driver.World.Tests;

procedure Selftest is
   Failed : Natural;
begin
   Driver.Core_Tests.Register;
   Driver.Geometry.Tests.Register;
   Driver.Robot.Tests.Register;
   Driver.Robot.Hand.Tests.Register;
   Driver.World.Tests.Register;
   Driver.Action.Tests.Register;
   Driver.Brain.Tests.Register;
   Failed := Driver.Tests.Run_All (if Ada.Command_Line.Argument_Count > 0 then Ada.Command_Line.Argument (1) else "");
   Ada.Command_Line.Set_Exit_Status (if Failed = 0 then Ada.Command_Line.Success else Ada.Command_Line.Failure);
end Selftest;
