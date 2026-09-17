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
with Monitor;
with Sinew;
with Runtime;
with Plan;
with Geom;
package Act is
   type Item_Kind is (Finger, Grip, Piece, Thing, Thing_Remembered, Thing_Held, Spot);   --  Spot = 脑让我记住的一个地方(指尖当时的位置)   --  Piece = 我身上某个通道带的一块(Which = 通道号)
   type Item is record
      Kind : Item_Kind := Thing;
      Arm : Natural := 0;
      Which : Natural := 0;
      Slot : Integer := -1;
      Located : Boolean := False;
      Cu, Cv : Long_Float := 0.0;
      X0, Y0, X1, Y1 : Natural := 0;
      Depth, Height : Long_Float := 0.0;
      Top : Long_Float := 0.0;      --  这块顶面的深度(米):抓在"顶面到桌面的一半"处,而不是贴着顶面
      Count : Natural := 0;
      Au, Av : Long_Float := 0.0;    --  这一块自己的主轴(画面里的单位向量)
      Elong : Long_Float := 1.0;     --  长轴/短轴
      Gray : Long_Float := -1.0;     --  框里的平均灰度(< 0 = 没量到)
      Pw : Geom.V3 := [others => 0.0];   --  Spot:记住的那一点(世界坐标)
      Name : Unbounded_String;           --  Spot:脑给它起的名字
   end record;
   package Item_Vectors is new Ada.Containers.Vectors (Natural, Item);

   type Track_Kind is (Piece_Pt, Thing_Pt);   --  Piece_Pt:我身上的一块零件(Chan_K = 带它的通道;握合通道 = Chan.Per_Arm,那块就是手指)
   type Stored_Effect is record
      Arm, Cam : Natural := 0;
      Kind : Track_Kind := Piece_Pt;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道)
      Blob : Integer := -1;      --  这块的第几团(-1 = 整块;手指 0/1 = 两指各自)
      E : Table.Effect;
      Trust : Table.Mask := [others => True];   --  探针时这个点真跑过地板的通道
      Reach : Table.Vec := [others => 1.0];   --  每个通道各自被核实过的步幅(探针上限的倍数):那个通道用到上限一半以上且表报准了才翻倍;报错/没照做/认丢了减半;存进身体文件
      Pose : Plug.Arm_Pose := [others => 0.0];   --  这张表是在哪个位姿下量的:表是【就地】的,离得远了不成立(EP:远近那一列小了 7 倍,是别处量的)
      Has_Pose : Boolean := False;
      Held : Integer := -1;      --  量这张表的时候手里是什么(-1 = 空手,否则是那一槽)。拿着东西以后同一条命令后果不同 ⇒ 换了就当没有、重量
   end record;
   package Effect_Vectors is new Ada.Containers.Vectors (Natural, Stored_Effect);
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
      --  ── 脑交的是一段 Sinew 程序(FO 那套 JSON 表只在 BL_JSON=1 时用)──
      --  Sinew 只是脑的嘴:一段区间编译成 FO 执行核那一轮的命令,段末事件翻回结局词;循环/分支/try 由它自己的状态机走。
      Use_Json : Boolean := False;
      Prog : Sinew.Program;
      M : Runtime.Machine;
      Binds : Plan.Bind_Vectors.Vector;   --  每个名词落到了第几号(编号只活在身体里,从不进语言)
      Have_Prog : Boolean := False;
      Refused : Unbounded_String;         --  上一段被退回的话:理由 + 能照抄的替代,随下一轮一起给脑
      Prog_Log : Unbounded_String;        --  这一段程序里每一节的结果都攒在这儿,一起给脑
      Eye_Want : Sinew.Eye_Pick := Sinew.Ey_None;
      Name_Cam : Integer := -1;           --  脑最近一次真认出一个名字时,身体在哪只眼里
      Blind_Cam : Integer := -1;          --  脑刚说过"这只眼里没有它"的那只眼
      Blind_Mask : Natural := 0;          --  这一集里脑说过"这只眼里没有它"的眼(按位:第 k 位 = 第 k 台相机);认出来就清
      --  ── 几何驾驶(腕眼里只用彩色图 + 手的位姿读数 + 焦距;不读深度)──
      Geo : Geom.Geo_Vectors.Vector;      --  每台相机一份:焦距、朝向、指尖
      Geo_Path : Unbounded_String;        --  几何常数存哪(身体文件旁边)
      Geo_Dist : Long_Float := -1.0;      --  上一次几何逼近结束时,它离"指尖该到的那一点"还差多少米(< 0 = 没有)
      Geo_Round : Natural := 0;           --  那是第几轮
      Geo_Came : Long_Float := 0.0;       --  几何逼近一共走了多远(米);"离远点"就沿原路退这么远
      Geo_Dir : Geom.V3 := [others => 0.0];   --  逼近的方向(世界系单位向量)
      Geo_Obs : Geom.Obs_Vectors.Vector;  --  这一集里点名那块在腕眼里的历次观测(位姿 + 像素)
      Spots : Item_Vectors.Vector;        --  脑让我记住的地方(这一集里留着,换集清空)
      Geo_Slot : Integer := -1;
      Geo_Slot_Obs : Geom.Slot_Obs_Vectors.Vector;   --  这一集里腕眼看见过的【每一样东西】的历次观测(按世界槽号),用来算它们在三维哪儿
      Geo_Map_Cam : Integer := -1;
   end record;

   procedure Init_Tracks (C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
   --  开机:装回几何常数;观测里带了焦距就记下;有深度的开机帧里量一次指尖(之后不再读深度)
   procedure Geo_Boot (F : Plug.Frame; C : in out Context; Body_Path : String);
   --  新的一集:几何账和三维记忆清空(身体常数留着)
   procedure Geo_New_Episode (C : in out Context);

   --  ── 下面几个是接缝,离线自检要逐条钉死 ──
   --  拿住了没,唯一分得开的那一条:抬手时它跟着我的手走了【同样一段】。
   --  "它原来待的地方空了"分不开【撞跑】(FM/FO 三次假拿住全是它)。零系数:两段位移的差比手自己挪的一半还小。
   --  手一步没挪 ⇒ 判不了(恒假),由调用方报"我说不准"。
   function Came_With_Me (Obj_Du, Obj_Dv, Hand_Du, Hand_Dv : Long_Float) return Boolean;
   --  抓在这块的哪个高度 = 顶面到它站着的那个面之间的一半(球 = 赤道;平的 = 表面)。两个数都是这块自己量的。
   function Grab_Depth (O : Item) return Long_Float;
   --  结局词 → 判法,唯一的一处。每个词必须有自己的判法,不许并进兜底的步数上限。
   function Until_Of (O : Sinew.Outcome) return Monitor.Until_Kind;
   function Until_Word (O : Sinew.Outcome) return String;
   function Kind_Of_Word (W : String) return Monitor.Until_Kind;
   --  关系词 → FO 执行核的字,唯一的一处
   function Rel_Cmd (R : Sinew.Rel) return String;
   function Rel_Has_Own_Branch (R : Sinew.Rel) return Boolean;
   --  身体报的那句事件 + 合手那句话,归到结局词里的哪一个
   function Classify (Event, Grip_Note : String) return Sinew.Outcome;
   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean;
end Act;
