--  执行器:把脑说的"第几号 → 去哪 → 直到什么事件为止"解成通道命令。
--  跟踪的点只有两种:我的握区(世界相机里靠光流跟;手上相机里是固定像素)、世界里的一块(每步重切)。
--  解算 = 带限的加权最小二乘分配;表每步递推重估;碰上/推不动 = 零表比走的表更准;抓 = 块装进握区,笼住了才合。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Plug;
with Selfmap;
with Zone;
with World;
with Brain;
with Picture;
with Table;
with Memory;
package Act is
   type Item_Kind is (Finger, Grip, Thing, Thing_Remembered, Thing_Held);
   type Item is record
      Kind : Item_Kind := Thing;
      Arm : Natural := 0;
      Which : Natural := 0;
      Slot : Integer := -1;
      Located : Boolean := False;
      Cu, Cv : Long_Float := 0.0;
      X0, Y0, X1, Y1 : Natural := 0;
      Depth, Height : Long_Float := 0.0;
      Count : Natural := 0;
   end record;
   package Item_Vectors is new Ada.Containers.Vectors (Natural, Item);

   type Track_Kind is (Zone_Pt, Thing_Pt);
   type Stored_Effect is record
      Arm, Cam : Natural := 0;
      Kind : Track_Kind := Zone_Pt;
      Lobe : Integer := -1;      --  握区的哪一瓣(-1 = 整个/世界块)
      E : Table.Effect;
      Trust : Table.Mask := [others => True];   --  探针时这个点真跑过地板的通道
      Reach : Long_Float := 1.0;   --  这张表被核实过的步幅(探针上限的倍数):表比零表准就翻倍,不准就减半;存进身体文件,越用越强
   end record;
   package Effect_Vectors is new Ada.Containers.Vectors (Natural, Stored_Effect);
   type Zone_Track is record
      Valid : Boolean := False;
      Cu, Cv, Z : Long_Float := 0.0;
      Stale : Natural := 0;
   end record;
   package Zone_Track_Vectors is new Ada.Containers.Vectors (Natural, Zone_Track);

   type Context is record
      Map : Selfmap.Body_Map;
      Hands : Zone.Hand_Vectors.Vector;
      Wld : World.State;
      Mem : Memory.Store;
      Cam : Natural := 0;
      Tables : Effect_Vectors.Vector;
      Zones : Zone_Track_Vectors.Vector;     --  (臂 × N_Cams + 相机)
      Recent : Unbounded_String;
      Task_Text : Unbounded_String;
      Items : Item_Vectors.Vector;
      Cells_U, Cells_V : Floats;
      Cols : Natural := 6;
      Rows : Natural := 4;
      Look_Only : Boolean := False;
      Eye_Host : Unbounded_String;
      Eye_Port : Natural := 8079;
      Dump_Dir : Unbounded_String;
      Round_N : Natural := 0;
      Fast : Boolean := False;
      Boot_Steps : Natural := 0;   --  开机量身体用掉的拍数(记账,不是上限)
   end record;

   procedure Init_Tracks (C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
end Act;
