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
with Schema;
with Chan;
with Learned;
with Plan;
package Act is
   type Item_Kind is (Finger, Grip, Piece, Thing, Thing_Remembered, Thing_Held);   --  Piece = 我身上某个通道带的一块(Which = 通道号)
   type Item is record
      Kind : Item_Kind := Thing;
      Arm : Natural := 0;
      Which : Natural := 0;
      Jaw_K : Natural := 0;          --  Finger/Grip:这是这条臂的第几个抓握通道(五指手一根手指一个)
      Slot : Integer := -1;
      Located : Boolean := False;
      Cu, Cv : Long_Float := 0.0;
      X0, Y0, X1, Y1 : Natural := 0;
      Depth, Height : Long_Float := 0.0;
      Count : Natural := 0;
      Au, Av : Long_Float := 0.0;    --  这一块自己的主轴(画面里的单位向量)
      Elong : Long_Float := 1.0;     --  长轴/短轴
      Gray : Long_Float := -1.0;     --  框里的平均灰度(< 0 = 没量到)
   end record;
   package Item_Vectors is new Ada.Containers.Vectors (Natural, Item);

   --  响应表已挪进 Learned(体检要审判它,执行器要用它 —— 谁也不该依赖谁的上层)
   subtype Track_Kind is Learned.Track_Kind;
   subtype Stored_Effect is Learned.Stored_Effect;
   package Effect_Vectors renames Learned.Effect_Vectors;
   type Known_Array is array (0 .. Chan.Per_Arm) of Boolean;
   type Zone_Track is record
      Valid : Boolean := False;
      Cu, Cv, Z : Long_Float := 0.0;
      Stale : Natural := 0;
      Au, Av, Bu, Bv : Long_Float := 0.0;   --  两瓣各自的位置(从身体图按此刻位姿算出)
      Has_Lobes : Boolean := False;
      Known : Boolean := False;             --  此刻位姿离某个真看过的样本不超过一步核实过的步幅 ⇒ 不用看就知道
      Pieces : Schema.Part_Array;           --  这只手每个通道带的零件此刻在这台相机里的位置(按位姿从身体图算)
      Pieces_Known : Known_Array := [others => False];
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
      Sch : Schema.Map;            --  身体图:位姿 → 手指在各相机画面里的位置(只存真看见过的)
      Want_Size : Long_Float := 0.0;   --  正在跟的那块东西现在看着多大(画幅):切块的窗口要比它大,否则闭运算把它填平、只剩一圈边(EV 实测球被切成三块)
      Cut_Seq : Natural := 0;      --  切块缓存:这一帧的编号(同一帧同一台相机不重切,颜色切块很贵)
      Cut_Cam : Integer := -1;
      Cut_Regs : Picture.Regions;
      --  🔴 脑不再一轮填一张表,而是交【一段程序】。程序编译过了就存在这里,一轮跑一小节,
      --  跑完才回去问下一段 —— 这才是"少问几百次"的来源。
      Prog : Plan.Compiled;
      Prog_At : Natural := 0;          --  下一条要跑的动作是第几条
      Have_Prog : Boolean := False;
      Refused : Unbounded_String;      --  上一段被退回的话:理由 + 能照抄的替代,随下一轮一起给脑
   end record;

   procedure Init_Tracks (C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
end Act;
