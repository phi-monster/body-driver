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
with Sinew;
with Runtime;
with Monitor;
package Act is
   --  🔴 脑写的结局词 → 身体的判法。**只有这一处**。
   --  以前它散在两个局部函数里(Outcome → 字符串 → Until_Kind),中间那一跳把 lost / free / refused
   --  三个词悄悄并进了兜底的"走够步数":脑写一个词,身体做的是另一个词的事,还回报"步子走完还没到"。
   --  自检逐词钉死这张表(selfcheck「每个结局词都有自己的判法」),再想并词就会当场红。
   function Until_Of (O : Sinew.Outcome) return Monitor.Until_Kind;
   --  只有脑真写了 arrived,身体才准因为"约束满足了"而停(timeout 同样走步数上限,但不许自称到了)
   function Wants_Arrive (O : Sinew.Outcome) return Boolean;
   function Until_Word (O : Sinew.Outcome) return String;   --  给旧的 Brain.Say 用的同一张表
   function Kind_Of_Word (W : String) return Monitor.Until_Kind;   --  字符串那一跳的反向表(自检钉死它和 Until_Of 一致)
   --  关系词 → 执行器认得的那个字。四个词(close / open / clear / still)不走这里,它们各有各的分支。
   --  剩下的每一个都必须有自己的、非空非 "?" 的字 —— 自检钉死,防的是和结局词同一类的悄悄降级。
   function Rel_Cmd (R : Sinew.Rel) return String;
   function Rel_Has_Own_Branch (R : Sinew.Rel) return Boolean;
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
   --  角色 → 它肯收哪种自己的零件。**只有这一处**,自检钉死 grasper 和 pusher 不许收同一种
   --  (语言里 pusher 就是"推得动东西、但【合不拢】的部件";以前它把 Grip 也收了,
   --  于是两个角色绑到同一块,语言里的角色区分是假的)。
   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean;
   --  🔴 这一段到底能走几步。脑写了 or N steps 就是 N,没写就用安全上限。
   --  **永远不许返回 0** —— 上限 0 交给 Monitor.Fired,U_Steps 判 W.Steps >= 0 第一步就成立,
   --  一段只走一推(GM:三段 until arrived 各 1 推 5 拍,身体却回报"步子走完还没到")。自检钉死。
   function Effective_Cap (Say_Steps : Natural) return Positive;
   function Safety_Cap return Positive;   --  脑没写步数时用的那个上限(自检钉死它不是 1 —— "没写"不等于"只走一步")
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

   --  记住的一个地方:那一刻它在这台相机画面里的位置和远近。名字是脑起的,数留在身体里。
   type Place is record
      Name : Unbounded_String;
      Cam : Natural := 0;
      Cu, Cv, Z : Long_Float := 0.0;
   end record;
   package Place_Vectors is new Ada.Containers.Vectors (Natural, Place);

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
      Prog : Sinew.Program;            --  脑交的那一段程序(带循环/分支/定义)
      M : Runtime.Machine;             --  跑到哪一条了
      Binds : Plan.Bind_Vectors.Vector;--  每个名词落到了哪一块
      Have_Prog : Boolean := False;
      Refused : Unbounded_String;      --  上一段被退回的话:理由 + 能照抄的替代,随下一轮一起给脑
      Places : Place_Vectors.Vector;   --  remember 记下的地方:身体自己能重新找到的位置,不是坐标
      Last_Outcome : Sinew.Outcome := Sinew.Oc_None;   --  上一节的结局(八个词之一)
      Blind_Say : Unbounded_String;    --  身体照走了,但有件事要如实说给脑(不是停,是说)
      Eye_Chosen : Boolean := False;   --  这一集已经自己换过一次眼睛了(不许来回弹)
      Reckless : Boolean := False;     --  这一节写了 anyway:身体的一切谨慎作废
      Prog_Log : Unbounded_String;     --  🔴 这一段程序里【每一节】的结果都攒在这儿。
                                       --  以前只留最后一节,而最后那一轮恰好是"程序跑完了"的空话,
                                       --  于是前几节说了什么全被冲掉,脑只能去翻日志 —— 等于身体不会说话。
   end record;

   procedure Init_Tracks (C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
end Act;
