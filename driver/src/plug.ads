--  插头:这台机器人通过 msgpack/WebSocket 说话。收观测、回应答、在对方问"给我动作"时把攥着的命令交出去。
--  规矩:①应答的形状由对方定;②没有新命令就重发上一条(空动作 = 这一集到此为止);③线断了在同一个口上等它重接。
with Bytes; use Bytes;
with Layout;
with Msgpack;
with Websocket;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package Plug is
   type Arm_Pose is array (0 .. 6) of Long_Float;
   package Pose_Vectors is new Ada.Containers.Vectors (Natural, Arm_Pose);
   package Floats_Vectors is new Ada.Containers.Vectors (Natural, Floats, F64_Vectors."=");
   type Cam is record
      W, H : Natural := 0;
      Gray, RGB : Buf;
      Has_Depth : Boolean := False;
      Depth : Floats;
      --  观测里带了这台相机的内参(焦距、主点);没有就由身体自己量
      Has_K : Boolean := False;
      Focal, Cx, Cy : Long_Float := 0.0;        --  像素
   end record;
   package Cam_Vectors is new Ada.Containers.Vectors (Natural, Cam);
   type Frame is record
      Joints : Floats_Vectors.Vector;   --  每个关节组一串
      EE : Pose_Vectors.Vector;          --  每条臂 xyz + wxyz(V1b 3c 之后:由运动学按关节读数算出来的腕眼位姿,Pose_Hook 填)
      Reported_EE : Pose_Vectors.Vector; --  身体自己报的位姿(有的身体报):只落盘给离线打分,驱动不读
      Jaw : Floats_Vectors.Vector;       --  每条臂一串:这条臂【全部】抓握通道的读数
                                         --  (以前只留第一个 ⇒ 五指手的后四根手指整组丢掉)
      Cams : Cam_Vectors.Vector;
      Seq : Natural := 0;
      Instruction : Unbounded_String;    --  观测里带的任务句
   end record;

   --  逐拍记下的东西(2026-09-26 V1B10:仿真的画面比关节读数晚一拍 —— 扫描时手还在转就存了格子,画面和读数不是同一刻,
   --  第 5 个关节那几格差到 4°,解出来的运动学一只手整组错;要量"画面晚几拍",再按它给每一格配读数)。
   --  每一拍:帧号、各组关节读数、身体报的位姿(只落盘打分)、各台相机这一拍画面变了多少(和上一拍比的灰度差平均,隔 Img_Stride 个像素取一个)、
   --  各组读数这一拍变了多少(变得最多的那个关节)
   type Beat is record
      Seq : Natural := 0;
      Joints : Floats_Vectors.Vector;
      Reported_EE : Pose_Vectors.Vector;
      Img_Chg : Floats;
      Q_Chg : Floats;
   end record;
   package Beat_Vectors is new Ada.Containers.Vectors (Natural, Beat);
   package Buf_Vectors is new Ada.Containers.Vectors (Natural, Buf, U8_Vectors."=");
   Img_Stride : constant := 4;     --  量画面变了多少时隔几个像素取一个(采样密度,次数)
   Keep_Beats : constant := 4096;  --  最多记最近几拍(次数)
   Max_Lag : constant := 4;        --  画面和读数最多查到差几拍(前后各 4 拍,次数)

   type Cmd_Kind is (Hold, Ee, Joint, Base);
   type Cmd is record
      Kind : Cmd_Kind := Hold;
      Arm : Natural := 0;
      Pose : Arm_Pose := [others => 0.0];   --  Ee:绝对位姿
      Jaw : Floats;                          --  这条臂全部抓握通道的目标(空 = 保持读数)
      Q : Floats;                            --  Joint:这一组的绝对关节角
      Groups : Ints;                         --  Joint:一条命令同时给几组关节读数的目标(开机几只手一起扫,V1b 2026-09-26);空 = 只看 Group / Arm
      Qs : Floats_Vectors.Vector;            --  和 Groups 一一对应:每组的绝对关节角
      Group : Integer := -1;                 --  Joint 且身体也报位姿时:Q 是第几组关节读数(Lay.Joints 里的下标)的目标;
                                             --  开机量胳膊一个关节一个关节转用(V1b,2026-09-26)。-1 = 按臂(只报关节的身体)
      V : Floats;                            --  Base:速度
   end record;

   type Link is record
      Conn : Websocket.Conn;
      Lay : Layout.Body_Layout;
      Have_Layout : Boolean := False;
      Last : Msgpack.Doc;
      Last_Obs : Integer := -1;
      Pending : Buf;
      Has_Pending : Boolean := False;
      Last_Sent : Buf;
      Has_Last : Boolean := False;
      Reset_Flag : Boolean := False;
      Seq : Natural := 0;
      Ep_Seq0 : Natural := 0;           --  这一集开始时的帧号(对方说 reset 时记下)⇒ 本集用了几拍 = Seq - Ep_Seq0
      Vid_N : Natural := 0;
      Film_N : Natural := 0;
      Wait_Us, Parse_Us : Long_Float := 0.0;
      Frame_S : Long_Float := 0.0;      --  量出来的帧时(秒/帧)
      Beats : Beat_Vectors.Vector;      --  最近 Keep_Beats 拍(帧号连着)
      Prev_Gray : Buf_Vectors.Vector;   --  上一拍各台相机的灰度图
      --  每个抓握读数组(按 Lay.Jaw 的下标)最后一次给过的目标;空 = 这一集还没给过。没给命令的通道照发它,不照发此刻的读数:
      --  读数会被外力推着走(V1B24 2026-09-27:碰桌面时手指被桌面顶着沿滑轨往里推,"保持此刻的读数"把推合了的读数锁住,爪子合上,
      --  后一瓣按张开的手指去量、短了 13 mm;拿着东西时它也会把夹紧的目标换成夹着东西的读数、卸掉夹紧力)。对方复位(新的一集)时清空
      Jaw_Set : Floats_Vectors.Vector;
   end record;

   procedure Boot (Port : Natural; L : in out Link; Ok : out Boolean);
   --  第 Ji 个抓握读数组第 K 个数这回发什么:这条命令给了(Mine)⇒ 发它并记进 L.Jaw_Set;没给 ⇒ 这一集给过的最后一个目标;一次没给过 ⇒ 此刻的读数 Cur。导出只为自检
   function Jaw_Value (L : in out Link; Ji : Natural; K : Natural; Mine : Boolean; C : Cmd; Cur : Floats) return Long_Float;
   function Sense (L : in out Link; F : out Frame) return Boolean;
   function Act (L : in out Link; C : Cmd) return Boolean;
   --  几只手按拍对齐(Lockstep,2026-09-28 PLAN ⑧ (g)):在手的任务里 Act 只记下目标(位姿命令先按 Cmd_Hook 解成关节;每组关节一个目标、
   --  每只手的爪子一个目标),Sense 把棒交还主线程、醒来拿主线程收的那一帧。主线程每拍 Lock_Beat:记下的有变 ⇒ 合成一条关节动作发出去,
   --  再收一帧给大家。Lock_Begin / Lock_End 清掉记下的(一段开始 / 结束;最后发出去的那条动作照旧每拍重发 = 每只手停在最后的目标)
   procedure Lock_Begin;
   procedure Lock_Beat (L : in out Link; F : in out Frame; Ok : out Boolean);
   --  自检用:假身体在主线程里给这一拍的帧(代替 Lock_Beat 从链路收的那一帧;帧里手的位姿由调用方按关节读数算好)
   procedure Lock_Feed (F : Frame; Ok : Boolean := True);
   procedure Lock_End;
   --  记下的目标合成的那条关节动作(Groups / Qs;爪子另按手记,不在这条里)。导出只为自检
   function Lock_Merged return Cmd;
   --  V1b 3c(2026-09-26):身体报的"手在哪"驱动不读。运动学量好以后,每一帧手的位姿由上面按关节读数算好填进来(Pose_Hook,
   --  Sense 收完一帧最后调它),发下来的位姿命令由上面解成关节目标(Cmd_Hook:把 Kind 换成 Joint、填好 Arm / Group / Q / Jaw;
   --  Ok = False = 解不出来,这条命令不发)。Plug 不认识运动学(Kinem 用 Geom,Geom 用 Plug)⇒ 由上面登记;null = 不换
   type Pose_Hook is access procedure (F : in out Frame);
   type Cmd_Hook is access procedure (C : in out Cmd; Ok : out Boolean);
   procedure Set_Hooks (P : Pose_Hook; Q : Cmd_Hook);
   --  这只手能不能到这个位姿(不发命令,只解):反解按量到的关节限位解出来以后还差多少(位置按世界单位、朝向按弧度)。
   --  Ok = False = 上面没登记(没有运动学)。开机碰桌面量指尖挑落点时用:挑到的地方先问一句,解不出就换下一处(V1B22 2026-09-27:
   --  挑的那处转 1.02 rad、挪 0.17 m,反解在限位里解不到,实到转 1.20 rad、挪差 3.4 单位,那一瓣没量成)
   type Reach_Hook is access procedure (Arm : Natural; Pose : Arm_Pose; Pos_Err, Rot_Err : out Long_Float);
   procedure Set_Reach (R : Reach_Hook);
   procedure Reach (Arm : Natural; Pose : Arm_Pose; Pos_Err, Rot_Err : out Long_Float; Ok : out Boolean);
   --  到过的范围(09-29):上一条位姿命令的反解是不是被"到过的范围往外一步"截住了(没解到目标、有关节停在这道界上而那不是记下的尽头):
   --  Free = 没截住;Held = 截住了、从那一条以来到过的范围还没长(手还没动起来:命令隔一两拍才起效)⇒ 先别收;
   --  Held_Grown = 截住了、范围长了 ⇒ 同一个目标按此刻的读数再解一次还能往前 ⇒ Selfmap.Go 这一拍就重发(大转跟着手连着走完)。
   --  手停在真的尽头 / 碰上东西 ⇒ 范围不再长 ⇒ 一直是 Held,照常等停下、核尽头。没登记(没有运动学)⇒ Free
   type Limit_State is (Free, Held, Held_Grown);
   type Limit_Hook is access function (Arm : Natural) return Limit_State;
   procedure Set_Limit (H : Limit_Hook);
   function Held_Back (Arm : Natural) return Limit_State;
   function Take_Reset (L : in out Link) return Boolean;
   --  只看不清:对方是不是刚复位了(新的一集)。走路的那些段每一步看一眼,复位了就当场收段,不把这一段的动作发到新的一集里
   function Reset_Pending (L : Link) return Boolean;
   function Steps (L : Link) return Natural;            --  这一集到现在收了几拍画面(一拍 = 对方走一步;只数,不停)
   --  画面比读数晚几拍:帧号 ≥ From_Seq 的那些拍里,第 Cam 台相机每拍画面变了多少 和 lag 拍之前第 Group 组读数变了多少 的相关系数,
   --  lag = −Max_Lag … Max_Lag(Corr (lag + Max_Lag));返回相关最大的 lag(负 = 读数比画面晚)。拍数不够 / 没动过 ⇒ 0,Corr 全 0
   function Image_Lag (L : Link; Cam, Group, From_Seq : Natural; Corr : out Floats) return Integer;
   --  帧号 Seq 那一拍记下的关节读数 / 身体报的位姿(不在记着的那些拍里 ⇒ 空)
   function Joints_At (L : Link; Seq : Natural) return Floats_Vectors.Vector;
   function Reported_At (L : Link; Seq : Natural) return Pose_Vectors.Vector;
   function Arms (L : Link) return Natural;
   function Joint_Mode (L : Link) return Boolean;     --  没有末端位姿、只有关节角
end Plug;
