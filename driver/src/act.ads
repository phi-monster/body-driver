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
      --  🔴 这一块是在【哪台相机】里看见的。以前整张清单默认就是"当前这台",
      --  于是脑只能点名当前那台里的东西 —— GM 里我答"一个都不是",而【头顶相机里球一直看得见】,
      --  只是它没有号可点。带上这一位,清单才谈得上跨相机。
      Cam : Natural := 0;
   end record;
   --  角色 → 它肯收哪种自己的零件。**只有这一处**,自检钉死 grasper 和 pusher 不许收同一种
   --  (语言里 pusher 就是"推得动东西、但【合不拢】的部件";以前它把 Grip 也收了,
   --  于是两个角色绑到同一块,语言里的角色区分是假的)。
   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean;
   --  🔴 从身体图外推手的位置,炸没炸。Was = 样本里那两瓣本来隔多远,Now = 外推之后隔多远。
   --  差得比它本身还大 ⇒ 这次外推不作数(零系数:两个都是量出来的长度)。
   --  箱上真数据:样本存的是 0.137,而身体报给脑的是四分之三个画面 —— 就是这里炸的。
   function Extrapolation_Blew (Was, Now : Long_Float) return Boolean;
   --  🔴 拿住了没,唯一分得开的那一条:抬手时它跟着我的手走了【同样一段】。
   --  只看"它原来待的地方空了"分不开【撞跑】—— 球被我撞到画面角落,原地照样空了,
   --  身体照样报"拿住"并开始抬爪,而两指之间什么都没有(FM/FO 实测,三次假拿住全是这么来的)。
   --  零系数:两段位移的差比【我的手自己挪了多远】的一半还小 ⇒ 它跟着我走了。
   --  手一步没挪 ⇒ 判不了(Hand 位移为 0 时恒假),由调用方报"我说不准",不许自称拿住。
   function Came_With_Me (Obj_Du, Obj_Dv, Hand_Du, Hand_Dv : Long_Float) return Boolean;
   --  🔴 这一帧读到的远近收不收。Old_Z = 上一次【真读到】的(不是按位姿猜的),Pred_Z = 表预测这一步该到哪儿,
   --  Noise = 这一点自己量到的读深抖动。没有预测(Pred_Z<=0)时【不许整条放行】—— 那正是 FS 实测
   --  "手指离相机 0.454 m 一步跳到 0.010 m(一厘米,物理上不可能)"被收下的原因;退回"一步最多变自己抖动那么多"。
   function Depth_Ok (Zd, Old_Z, Pred_Z, Noise : Long_Float) return Boolean;
   --  🔴 画面上重合 ≠ 真的在一起。目标在它自己那个远近上,我在我的远近上;同一段真实横移,
   --  离相机越近在画面里跑得越多。所以要比的不是画面坐标本身,而是【把目标搬到我这个远近平面上】之后的坐标。
   --  焦距在两边同样出现、自动约掉 —— 一个标定参数都不要。
   --  FZ 实测:头顶相机报"差 0.062 幅、几乎压上了",而爪子在球上方 30 厘米。只比画面坐标就是在比影子。
   function On_My_Plane (T_Pic, T_Depth, My_Depth : Long_Float) return Long_Float;
   --  🔴 这一步的命令上限:眼睛跟得住的那个天花板,底下垫一块【身体自己动得起来】的地板。
   --  地板 = 身体噪声的两倍(量出来的)。命令比这还小 ⇒ 发出去身体一动不动,这一步白走。
   --  地板【不是】探针那一档 —— 探针那一档是 FO 用的四倍(0.026 vs 0.006),
   --  按探针那一档当地板,球被甩出视野;FO 正是拿它的四分之一,一步推进 8 厘米、44 推抓到球。
   --  Dead = 这个通道自己量出来的死区(命令比它小,身体不动);还没学到就是 0。
   function Push_Cap (Ceiling, Noise, Dead : Long_Float) return Long_Float;
   --  🔴 into 瞄哪儿:它自己的皮(这块的中位深度)和它站着的那个面,正中间。两个都是量出来的深度。
   function Into_Depth (Skin, Surface : Long_Float) return Long_Float;
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
      Blew_Up : Boolean := False;           --  这次从样本外推炸了(算出来的两瓣间距和样本里的差得比它本身还大)
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

   package Buf_Vectors is new Ada.Containers.Vectors (Natural, Buf, U8_Vectors."=");
   type Context is record
      Map : Selfmap.Body_Map;
      Hands : Zone.Hand_Vectors.Vector;
      Wld : World.State;
      Mem : Memory.Store;
      Cam : Natural := 0;
      Tables : Effect_Vectors.Vector;
      Zones : Zone_Track_Vectors.Vector;     --  (臂 × N_Cams + 相机)
      --  🔴 每个通道自己的【死区】:命令小于它,身体根本不动(实测随姿势和关节而变 ——
      --  FO 时 0.006 能动,GV 时同一批通道 0.013 实到 0.000)。身体本来就看得见"我命令了多少、实到多少",
      --  只是从来没拿它去调下限。命令发了实到为零 ⇒ 把这一档抬上去;真动了 ⇒ 把它压下来。
      Dead : Floats;                         --  按全局通道号索引(机体自己的单位;只和自己比)
      Recent : Unbounded_String;
      Task_Text : Unbounded_String;
      Items : Item_Vectors.Vector;
      --  每台相机【画过框、编过号】的那一份图。条带里给脑看的就是它 ——
      --  以前条带只给原图,别的相机里的东西看得见却没有号,脑点不了名。
      Shown : Buf_Vectors.Vector;
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
      Eye_Want : Sinew.Eye_Pick := Sinew.Ey_None;   --  这一节脑点了用哪只眼睛(没点 = 身体自己挑)
      Tgt_Cam : Integer := -1;         --  脑点名的那一块在哪台相机里(-1 = 这一节没点名东西)
      Prog_Log : Unbounded_String;     --  🔴 这一段程序里【每一节】的结果都攒在这儿。
                                       --  以前只留最后一节,而最后那一轮恰好是"程序跑完了"的空话,
                                       --  于是前几节说了什么全被冲掉,脑只能去翻日志 —— 等于身体不会说话。
   end record;

   procedure Init_Tracks (C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
end Act;
