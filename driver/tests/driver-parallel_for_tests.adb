with Driver.Parallel_For;
with Driver.Tests;

package body Driver.Parallel_For_Tests is

   use Driver.Tests;

   type Sums is array (Positive range <>) of Long_Long_Integer;

   --  Each index sums its own range of numbers into its own slot: done at
   --  once, every slot holds what it holds done one after another.
   procedure Each_Index_Alone is
      Count : constant := 5;
      Terms : constant := 200_000;
      Got, Wanted : Sums (1 .. Count) := [others => 0];
      procedure Sum (Index : Positive) is
      begin
         for K in 1 .. Terms loop
            Got (Index) := Got (Index) + Long_Long_Integer (K * Index);
         end loop;
      end Sum;
      procedure At_Once is new Driver.Parallel_For (Sum);
   begin
      for I in Wanted'Range loop
         for K in 1 .. Terms loop
            Wanted (I) := Wanted (I) + Long_Long_Integer (K * I);
         end loop;
      end loop;
      At_Once (1, Count);
      Check (Got = Wanted, "a slot done at once differs from it done alone");
      Got := [others => 0];
      At_Once (1, 0);
      Check (Got = [1 .. Count => 0], "an empty range did some work");
      At_Once (3, 3);
      Check (Got (3) = Wanted (3) and then Got (1) = 0 and then Got (5) = 0,
             "a range of one did not do just its index");
   end Each_Index_Alone;

   --  An index that fails makes the call fail, once the others have ended:
   --  every other slot is written when the exception arrives.
   procedure A_Failure_Waits_For_The_Rest is
      Count : constant := 4;
      Done  : array (1 .. Count) of Boolean := [others => False];
      Raised : Boolean := False;
      procedure Fail_Two (Index : Positive) is
      begin
         if Index = 2 then
            raise Program_Error with "index two";
         end if;
         for K in 1 .. 100_000 loop
            Done (Index) := K = 100_000;
         end loop;
      end Fail_Two;
      procedure At_Once is new Driver.Parallel_For (Fail_Two);
   begin
      begin
         At_Once (1, Count);
      exception
         when Program_Error =>
            Raised := True;
      end;
      Check (Raised, "the failure of one index was not raised by the call");
      Check (Done (1) and then Done (3) and then Done (4) and then not Done (2),
             "the call returned before the other indices ended");
   end A_Failure_Waits_For_The_Rest;

   --  A work that itself does work at once: every inner index is done, and
   --  the calls return (the workers are a pool that nothing waits for; an
   --  index no worker is free for is done by whoever asked).
   procedure Nested_Calls_End is
      Outer, Inner : constant := 12;
      Got : array (1 .. Outer, 1 .. Inner) of Long_Long_Integer := [others => [others => 0]];
      procedure Row (I : Positive) is
         procedure Cell (J : Positive) is
         begin
            for K in 1 .. 20_000 loop
               Got (I, J) := Got (I, J) + Long_Long_Integer (K * I * J);
            end loop;
         end Cell;
         procedure Cells is new Driver.Parallel_For (Cell);
      begin
         Cells (1, Inner);
      end Row;
      procedure Rows is new Driver.Parallel_For (Row);
      Wanted : Long_Long_Integer;
   begin
      Rows (1, Outer);
      for I in 1 .. Outer loop
         for J in 1 .. Inner loop
            Wanted := 0;
            for K in 1 .. 20_000 loop
               Wanted := Wanted + Long_Long_Integer (K * I * J);
            end loop;
            Check (Got (I, J) = Wanted, "a nested index was not done, or not done alone:" & I'Image & J'Image);
         end loop;
      end loop;
   end Nested_Calls_End;

   procedure Register is
   begin
      Driver.Tests.Register ("core.parallel_for", "work done at once for several indices is what it is done alone",
                             Each_Index_Alone'Access);
      Driver.Tests.Register ("core.parallel_for_nested", "work at once inside work at once deadlocks or skips an index",
                             Nested_Calls_End'Access);
      Driver.Tests.Register ("core.parallel_for_failure",
                             "the failure of one index is raised once every other one has ended",
                             A_Failure_Waits_For_The_Rest'Access);
   end Register;

end Driver.Parallel_For_Tests;
