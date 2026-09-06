pragma SPARK_Mode (On);
package body Backup is
   procedure Clear (R : in out Ring) is
   begin
      R.N := 0;
      R.Head := 0;
   end Clear;

   function Count (R : Ring) return Natural is (R.N);

   procedure Remember (R : in out Ring; S : Step_Vec) is
   begin
      R.S (R.Head) := S;
      R.Head := (R.Head + 1) mod Depth;
      if R.N < Depth then
         R.N := R.N + 1;
      end if;
   end Remember;

   procedure Retreat (R : in out Ring; S : out Step_Vec; Had : out Boolean) is
   begin
      S := [others => 0.0];
      Had := False;
      if R.N = 0 then
         return;
      end if;
      R.Head := (if R.Head = 0 then Depth - 1 else R.Head - 1);
      for I in S'Range loop
         S (I) := -R.S (R.Head) (I);
      end loop;
      R.N := R.N - 1;
      Had := True;
   end Retreat;
end Backup;
