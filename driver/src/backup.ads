--  备份(SPARK):自适应的那一半不可信时的三件事 —— 冻住、不松手、沿来路退。记最近的几步命令,倒着发回去。
pragma SPARK_Mode (On);
package Backup is
   Depth : constant := 32;
   Max_Ch : constant := 64;
   type Step_Vec is array (0 .. Max_Ch - 1) of Long_Float;
   type Ring is private;
   procedure Clear (R : in out Ring);
   procedure Remember (R : in out Ring; S : Step_Vec)
     with Post => Count (R) = Natural'Min (Depth, Count (R'Old) + 1);
   function Count (R : Ring) return Natural;
   --  取最后一步的反向;没有就全零(冻住)。取出后那一步作废。
   procedure Retreat (R : in out Ring; S : out Step_Vec; Had : out Boolean)
     with Post => (if Had then Count (R) = Count (R'Old) - 1 else Count (R) = Count (R'Old));
   function Freeze return Step_Vec is ([others => 0.0]);
private
   type Slots is array (0 .. Depth - 1) of Step_Vec;
   type Ring is record
      S : Slots := [others => [others => 0.0]];
      N : Natural := 0;    --  存了几步(≤ Depth)
      Head : Natural := 0; --  下一个写入位置
   end record
     with Predicate => Ring.N <= Depth and Ring.Head < Depth;
end Backup;
