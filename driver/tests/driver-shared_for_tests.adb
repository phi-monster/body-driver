with Driver.Shared_For;
with Driver.Tests;

package body Driver.Shared_For_Tests is

   use Driver.Tests;

   type Sums is array (Positive range <>) of Long_Long_Integer;

   --  Each index sums its own range of numbers into its own slot, the later
   --  indices far longer than the first: shared, every slot holds what it
   --  holds done one after another, and each index was done once.
   procedure Each_Index_Once is
      Count : constant := 300;
      Got, Wanted : Sums (1 .. Count) := [others => 0];
      Times : array (1 .. Count) of Natural := [others => 0];
      procedure Sum (Index : Positive) is
      begin
         Times (Index) := Times (Index) + 1;
         for K in 1 .. Index * Index loop
            Got (Index) := Got (Index) + Long_Long_Integer (K);
         end loop;
      end Sum;
      procedure Shared is new Driver.Shared_For (Sum);
   begin
      for I in Wanted'Range loop
         for K in 1 .. I * I loop
            Wanted (I) := Wanted (I) + Long_Long_Integer (K);
         end loop;
      end loop;
      Shared (1, Count);
      Check (Got = Wanted, "a slot done shared differs from it done alone");
      Check ((for all T of Times => T = 1), "an index was done other than once");
      Got := [others => 0];
      Shared (1, 0);
      Check (Got = [1 .. Count => 0], "an empty range did some work");
      Shared (3, 3);
      Check (Got (3) = Wanted (3) and then Got (1) = 0 and then Got (4) = 0,
             "a range of one did not do just its index");
   end Each_Index_Once;

   --  An index that fails makes the call fail, once every task has ended.
   procedure A_Failure_Is_Raised is
      Count  : constant := 40;
      Raised : Boolean := False;
      procedure Fail_Seven (Index : Positive) is
      begin
         if Index = 7 then
            raise Program_Error with "index seven";
         end if;
      end Fail_Seven;
      procedure Shared is new Driver.Shared_For (Fail_Seven);
   begin
      begin
         Shared (1, Count);
      exception
         when Program_Error =>
            Raised := True;
      end;
      Check (Raised, "the failure of one index was not raised by the call");
   end A_Failure_Is_Raised;

   procedure Register is
   begin
      Driver.Tests.Register ("core.shared_for", "work shared index by index is what it is done alone, once each",
                             Each_Index_Once'Access);
      Driver.Tests.Register ("core.shared_for_failure", "the failure of one index is raised by the call",
                             A_Failure_Is_Raised'Access);
   end Register;

end Driver.Shared_For_Tests;
