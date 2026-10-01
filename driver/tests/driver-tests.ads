--  The self test: behavior specifications for every layer.
--
--  Each test states the defect it guards against, in one sentence, so a
--  failure reads as a diagnosis. A test that cannot fail is worthless: every
--  test must turn red when the behavior it names is broken, and merges are
--  checked by breaking one line and watching the matching test fail.

package Driver.Tests is

   type Procedure_Access is access procedure;

   procedure Register (Name : String; Guards : String; Run : not null Procedure_Access);
   --  Name: a short identifier, "<layer>.<subject>". Guards: the defect, e.g.
   --  "a joint that lags its command by one beat is reported as blocked".

   procedure Check (Condition : Boolean; What : String);
   --  Records a failure of the running test when Condition is False.

   procedure Check_Close (Actual, Expected, Tolerance : Real; What : String);
   --  Check (abs (Actual - Expected) <= Tolerance), reporting both values.

   function Run_All (Filter : String := "") return Natural;
   --  Runs every registered test whose name starts with Filter; prints one
   --  line per test and returns the number of failed tests.

end Driver.Tests;
