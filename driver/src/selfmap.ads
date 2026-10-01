--  开机量身体:每个通道推一下再推回来,看每台相机里哪片画面跟着动(部件图)、哪台相机长在哪只手上、
--  一条命令实际交付多少、本体读数抖多少、画面抖多少。探针幅度从极小起翻倍,直到走得出来又看得见。
with Bytes; use Bytes;
with Plug;
with Picture;
with Table;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Selfmap is
   type Part is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
      Frac : Long_Float := 0.0;
   end record;
   package Part_Vectors is new Ada.Containers.Vectors (Natural, Part);
   package Floor_Vectors is new Ada.Containers.Vectors (Natural, Picture.Floor_Map, Picture."=");
   type Body_Map is record
      Arms : Natural := 0;
      N_Cams : Natural := 0;
      Per_Arm : Natural := 6;
      Channels : Natural := 0;             --  Arms × Per_Arm
      Parts : Part_Vectors.Vector;         --  (通道 × N_Cams + 相机)
      Cam_Frac : Floats;                   --  (臂 × N_Cams + 相机):这只手一动,那台相机变了多少画面
      Cam_On_Arm : Ints;                   --  每只手:长在它上面的相机号,-1 = 没有
      World_Cam : Natural := 0;            --  变得最少的那台
      Amp : Floats;                        --  每通道:走得出来又看得见的探针幅度
      Delivered : Floats;                  --  每通道:那一幅度实际交付了多少
      Seen : Bools;                        --  每通道:有没有一台相机看见它动
      EE_Noise : Long_Float := 0.0;        --  本体位置读数抖多少(米)
      Rot_Noise : Long_Float := 0.0;       --  本体姿态读数抖多少(弧度)
      Jaw_Noise : Long_Float := 0.0;
      Joint_Noise : Long_Float := 0.0;     --  关节读数不动时抖多少(读数的单位;V1b 2026-09-26:按关节目标挪手时"停稳"看它)

      Floors : Floor_Vectors.Vector;       --  每台相机的静止噪声地板
      Pic_Floor : Ints;                    --  每台相机:整幅画静止时最大灰度差
      Jaws : Ints;                         --  每条臂量到几个抓握通道(五指手 5,两指手 1)
      --  一条命令发出后读数稳下来要几拍(量出来的:Settle_Since,每一次探针各量一回、取最多的;0 = 还没量到。
      --  09-30:原来缺省 2、开机前半段从来不量,后半段又夹在 6 拍以内)
      Settle : Natural := 0;
      --  越用越强:历次量到的幅度/实到(现值取中位数),量过几次
      Amp_Hist : Plug.Floats_Vectors.Vector;
      Deliv_Hist : Plug.Floats_Vectors.Vector;
      Measured_Times : Natural := 0;
   end record;
   --  快速核对:每只手推一个通道(存的幅度),实到和存的差一半以内且画面里看得见 ⇒ 身体没变
   type String_Note is record
      Text : Ada.Strings.Unbounded.Unbounded_String;
   end record;
   procedure Verify (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Ok_Body, Ok_Link : out Boolean; Note : out String_Note);

   --  一步里可以忽略的那一丝 = 这一步的百分之一(比例):Go 判"停了"(一拍挪不到这条命令的百分之一)和"碰到没有"(Blocked)用同一个
   Negligible : constant := 0.01;
   --  往前压的这一步有没有被挡住(纯函数,导出给自检):Short = 这一步沿命令方向少走了多少;Prev / Prev2 = 这一段里前两步空走的少走量,
   --  N_Free = 前面有几步空走的(0 ⇒ 这一步是第一步,没有可比的,判不了 ⇒ False);Lstep = 这一步多大;Noise = 静止读数噪声。
   --  被挡住 = 比上一步空走时多少走的量超过"这一步的百分之一、3 倍读数噪声、3 倍前两步空走之差"三样里最大的那样(倍数无量纲,同踢离群)。
   --  09-29 台架(V1B66 满精度):x5 同样大小的两步空走少走的量前后只差约 1e-5 单位;碰上的第一步多少走至少 0.0014(一档的一成)。
   --  原来的门 = 第一步 + 3 × 静止噪声,仿真读数不抖 ⇒ 门 = 第一步,差一丝就认成碰到(V1B60 虚认 37 次、V1B65 18 次)
   function Blocked (Short, Prev, Prev2 : Long_Float; N_Free : Natural; Lstep, Noise : Long_Float) return Boolean;
   --  同一个判法写成统计的样子(Blocked 就是它:Mean = 上一步空走的少走量、Sd = 前两步空走之差):
   --  被挡住 = Short 比空走时的少走量 Mean 多出"这一步的百分之一、Stats.Z(3)倍读数噪声、Stats.Z 倍空走的散布 Sd"里最大的那样;
   --  N_Free = 0 ⇒ 判不了 ⇒ False;
   --  N_Free = 1 ⇒ 一个样本没有散布(Sd 不用)。走一步(Step)拿一段路上所有空走的那几步(连开机探针)的平均和标准差当 Mean / Sd
   function Blocked_Stats (Short, Mean, Sd : Long_Float; N_Free : Natural; Lstep, Noise : Long_Float) return Boolean;
   --  发一条位姿命令并等它稳:返回实际交付(按通道)与用掉的拍数。F 更新到最后一帧。
   --  Press:往前压、碰到为止的那一步 —— 停没停只看沿命令平移方向的挪动:动起来以后连着两拍挪不到这一步的百分之一(Negligible)= 停了,
   --  就读;被顶住的软手指那点转动蠕动不算(原来等它慢下来要 10–11 拍);胳膊还在沿这个方向挪(伸远了跟不上、还在往回收)就接着等,
   --  不到固定拍数就读(09-29 V1B65:原来的"快读"到了 5 拍就读,胳膊伸远了还在漂,读到的少走量随时刻乱跳,被当成碰到 18 次)
   --  Watch:走的途中每一拍问一句"出事了没"(被跟的东西快出画面 / 看不见了);说出事就当拍把目标改成"停在这儿",不走完这一步
   --  Group >= 0:这一条发的不是位姿,是第 Group 组关节读数的目标 Joints(开机一个关节一个关节扫,V1b 2026-09-26);
   --  Groups / Qs 非空:同一条命令给几组读数各自的目标(几只手一起扫)。同一条发命令的路(Target / Jaw 这时不用)。
   --  关节目标的"停稳":读数到了目标 Tol 以内再有一拍不动就算到(Tol = 0 不这样判),否则连着两拍不动
   --  位姿目标给了 Tol(平移)/ Tol_Rot(转动)⇒ 位姿到了目标这么近连着两拍就算到;没到 ⇒ 连着两拍每拍挪不到这一档就算停(被顶住 / 到头);
   --  Tol = 0 照旧:连着两拍挪不到读数噪声才算停(V1B21 2026-09-27:位姿读数按关节算,停下以后还有十几微米的蠕动,空中一步要等 13 拍、压到桌面那一步 24 拍)
   type Watcher is access function (F : Plug.Frame) return Boolean;
   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Press : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0; Tol_Rot : Long_Float := 0.0;
                 Tols : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector);
   --  Tols(和 Qs 同形:每组每个关节一道门)给了 ⇒ "到了" = 每个关节差不到它自己那道门(关节目标);"停了"的门照旧按 Tol。
   --  开机扫描用:扫的那根差不到这一格的三分之一,别的关节差不到每根轴单独起步收格子的门(Kinem.Clean_Tol;H1 2026-09-28)
   --  一组关节这一拍"到了没有"(Go 里用的就是它;纯函数,导出给自检):Tols 这一位 > 0 ⇒ 这个关节按它自己的门,否则按 Tol;门 ≤ 0 的关节永远不算到
   function Joints_Arrived (Now, Target, Tols : Floats; Tol : Long_Float) return Boolean;
   --  一条命令从发出到读数停住用了几拍(纯函数,导出给自检):Moves (I) = 发出后第 I + 1 拍读数挪了多少(那一拍挪得最多的那个关节)。
   --  停住 = 动起来以后(挪过超过 Noise 的一拍),第一次"这一拍挪动不超过 Noise、而且不比上一拍小"(不再变小)的那一拍;返回它是第几拍。
   --  一直没动起来(探针小得读数跟不上)/ 看到的那几拍里还在变小(还没停住)⇒ 0:这一条量不出,不算(不拿没停住的拍数顶)
   function Settle_Beats (Moves : Floats; Noise : Long_Float) return Natural;
   --  量 Settle 的唯一办法:帧号 From_Seq(发命令之前那一拍,L.Seq)以后 Plug 逐拍记下的读数(Beats.Q_Chg,每一拍取各组里挪得最多的)⇒ Settle_Beats。
   --  开机前半段认手的每一次探针、后半段逐通道推的每一次(推过去、推回来各一条)都这样量,M.Settle 取量到的最多的
   function Settle_Since (L : Plug.Link; From_Seq : Natural; Noise : Long_Float) return Natural;
   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean);   --  不下命令空等 N 拍
   --  什么都不做时读数抖多少、画面抖多少(静止对,4 拍):位姿 / 姿态 / 抓握 / 关节读数的噪声 + 每台相机的灰度地板。
   --  先等上一个动作的尾巴收住才量:每组读数(每条臂的平移、转动,每组关节、每组抓握)一拍挪的不再比上一拍少 = 那一组收住了,
   --  每组都收住了才开始量(还在慢慢挪的那一截不算噪声;出口是量到的"不再变小",不设拍数)。
   --  地板用这几拍里最后一对"两帧都收到了画面"的静止对;一对都没有的那台 ⇒ 地板记成 U8'Last(量不到它的噪声 ⇒ 它的画面里什么都不算动了,
   --  不拿空画面当静止对、不编一个 0 的地板)。
   --  Measure 开头用它;只报关节的身体开机前半段(还没有位姿)也用它(同一种量法)。M.Arms 条臂的位姿噪声(没有位姿 = 0)
   procedure Measure_Idle (L : in out Plug.Link; F : in out Plug.Frame; M : in out Body_Map; Ok : out Boolean);
   --  等到画面连着两拍都不再变(各自的灰度地板以内),最多 Max 拍;返回用了几拍。
   --  Ok = 停稳了。等满 Max 拍还在变 ⇒ Ok = False、Used = Max(照实说没停稳 —— 09-30:原来超时照样 Ok = True,握区在还在动的画面上量);
   --  线断了 ⇒ Ok = False、Used < Max
   --  Prev_Pic 给了 ⇒ 等完时里面是最后一帧之前那一帧(两帧都是画面停下以后的:抓握通道推到头时"看没看见动了"两次比较、不共用一帧用,Picture.Seen_Twice)
   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean;
                         Prev_Pic : access Plug.Cam_Vectors.Vector := null);
   --  每台判得了的相机(有地板、两帧都收到了画面)都静止,而且至少有一台判得了;一台都判不了 ⇒ False(没有证据不说静止)
   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean;
   --  只看第 Cam 台:两帧之间超过噪声地板的像素凑不成一团。两帧里有一帧没收到这台的画面(占位)⇒ False(判不了,不说静止)
   function Picture_Still (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean;
   --  Eyes / World:开机前半段只用关节命令已经认出了"哪台相机长在哪只手上、哪台是世界相机"(Jointboot.Find_Arms)⇒ 照用,
   --  这里不再按位姿探针另认一遍(一个量一种量法);空 = 这里认
   --  Step_Px:每只手"一步看得见"的幅度 = 在它自己那只眼里画面挪 1 像素(V1b 09-27;第 A 个 = [平移, 转动]:平移 = 眼离桌面的距离 ÷ 焦距,
   --  转动 = 1 ÷ 焦距 弧度,开机前半段量的)。每个通道按它推一次再推回来:量走没走到(Delivered)、哪块跟着动(零件)。
   --  原来从极小起翻倍、推过去推回来两张图都变了一块就算看见 —— V1B17 仿真渲染噪声下第 1 只手推 0.0003 单位(约 0.016 mm)就被噪声凑成"看见了"。
   --  没有这一项的手(没量成运动学)⇒ 它的通道量不了
   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean;
                      Step_Px : Plug.Floats_Vectors.Vector;
                      Eyes : Ints := Int_Vectors.Empty_Vector; World : Integer := -1);
   function Jaw_Count (F : Plug.Frame; Arm : Natural) return Natural;   --  这条臂量到几个抓握通道
   --  这一帧里有没有这条臂第 K 个抓握通道的读数
   function Has_Jaw (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Boolean is (K < Jaw_Count (F, Arm));
   --  这条臂第 K 个抓握通道此刻的读数。没读数就不许问(09-30:原来没读数返回 1.0 = x5 夹爪"1 = 张开"的约定,被当成读数、又被当成目标发出去):
   --  先问 Has_Jaw,没有就照实说这一拍没读数
   function Jaw_Of (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Long_Float
     with Pre => Has_Jaw (F, Arm, K);
   function Jaw_All (F : Plug.Frame; Arm : Natural) return Floats;      --  这条臂全部抓握通道此刻的读数
   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural;
   --  Blocked 拿这一段前面两步空走当底(前两步空走之差就是散布):往前压的时候,碰上之前至少要空走这么多步,Blocked 才判得出(结构)
   Free_Base : constant := 2;

   --  ── 走一步(I6,大并行 §4;路 2 / 5 / 6 挪手都走它,碰到没有只有 Blocked 一个判法)──
   --  一条命令给一组或几组通道各自一个目标(一组 = 一条臂的位姿:平移 3 + 转动 3;Goal 是绝对位姿),每组从此刻的读数起走还差的一部分(Lim.Frac),
   --  再按三道上限缩,都是量的、没有的那一道 = 不限:
   --    反解够得到(Lim.Reach):这一步走完的位姿按量到的关节范围问反解(Plug.Reach),解不到就沿这一步缩到解得到的那一截
   --      (二分,细到这只手一步看得见的那一档);一截都解不到 ⇒ 这一组这一步不走(Reach_Cut)。
   --      超出到过的范围的那一截照旧由 Go 截在"到过的范围 + 往外一步"、手一动就重发(Plug.Held_Back);
   --    眼跟得住(Lim.Track / Track_Rot):这一步平移最多多长、转动最多多少 —— 调用方按眼算(跟着的东西在眼里挪不出跟得住的那一片);
   --    离可能碰到的地方远(Lim.Clear):沿这一步的方向离"可能碰到"的那条带子还有多远,这一步不进带子(带子里调用方给小步)。
   --  整步按一个比例缩(平移、转动一起缩:转着补偿指尖的那种步缩了还是同一条路)。
   --  交付不满的身体(每条命令只走到七八成就停,同一个目标再发也不再走):按它空走一步最多交付几成的上界把这一步放大(平移、转动各按各的;
   --  上界 = 这一段空走的底的平均少走 − Stats.Z 倍散布),交付得最多的那一步也不走过头;交付满的(x5 的开机探针)⇒ 不放大;
   --  这一步比量过的最长那一步还长 ⇒ 不放大(少走的是比例还是死区,只有不同长的几步分得出:一格长的探针不许拿去放大一大步)。
   --  一组 ⇒ 同 Go 的一条位姿命令(今天 Act.Step_Arm / Geo_Move 发的就是这一条,行为不变);
   --  几组 ⇒ 每一拍一条命令带几组的目标(Plug 按拍合成,同几只手按拍对齐),每组各自判停、判到没到;
   --  在主线程里调 ⇒ 这里自己开一只手的任务按拍对齐
   type Leg is record
      Arm  : Natural := 0;
      Goal : Plug.Arm_Pose := [others => 0.0];
      Jaw  : Floats;                                --  这条臂抓握通道的目标(空 = 保持,同 Go)
   end record;
   package Leg_Vectors is new Ada.Containers.Vectors (Natural, Leg);
   type Limits is record
      Frac      : Long_Float := 1.0;                --  这一步走还差的几成(今天的调用方照今天的给:Geo_Approach 远时 0.6、近了 1)
      Track     : Long_Float := Long_Float'Last;    --  眼跟得住:平移最多多长(世界单位;Last = 不限)
      Track_Rot : Long_Float := Long_Float'Last;    --  眼跟得住:转动最多多少(弧度)
      Clear     : Long_Float := Long_Float'Last;    --  离可能碰到的地方还有多远
      Reach     : Boolean := False;                 --  问反解够不够得到
      Press     : Boolean := False;                 --  压的那种步(同 Go)
      Watch     : Watcher := null;                  --  途中每拍问一句出事了没(同 Go)
      Loose     : Boolean := True;                  --  到了这只手一步看得见的那一档以内就算到(同 Act.Step_Arm 的 Geo_Settle);False = 等读数不动
   end record;
   --  空走的底(Blocked 拿它当"前面空走的那几步"):一种动(平移 / 转动)空走时一步少走它自己的几成 —— 几步、平均、平方差和(Welford)。
   --  一段路一开头装进开机探针量的那几步(Measure:每个通道推一步、停下再读,Tol = 0),这一段里每一步判成空走的再加进来。
   --  平均和标准差当 Blocked 的"上一步"和"散布"(Blocked_Stats):交付每步不一样的身体(真机每步 70–85%)拿"最近两步"当底,
   --  两步碰巧挨得近时门只剩一丝,下一步交付少一点就被认成挡住;走到了(Loose 的那一档以内)、被 Watch 叫停、等满了还在动的那几步不进底:
   --  它们少走多少是 Go 在哪一刻收的,不是身体空走交付多少
   type Free_Part is record
      N : Natural := 0;
      Mean, M2 : Long_Float := 0.0;
      Len_Hi : Long_Float := 0.0; --  这几步里最长的那一步多长(平移按长度、转动按弧度):交付的比例只在量过的长度以内当证据
   end record;
   type Free_Leg is record
      Arm : Natural := 0;
      Tr, Rot : Free_Part;
   end record;
   package Free_Vectors is new Ada.Containers.Vectors (Natural, Free_Leg);
   type Walk is record
      Legs : Free_Vectors.Vector;
   end record;
   procedure Note_Free (P : in out Free_Part; Short, Len : Long_Float);   --  Short = 这一步少走了它自己的几成;Len = 它的长
   function Free_Sd (P : Free_Part) return Long_Float;               --  标准差(不到两步 = 0)
   --  这一步(长 Len,少走 Short,读数噪声 Noise,都是同一种单位)按这一段空走的底判挡没挡(= Blocked_Stats,底按 Len 折回长度)
   function Blocked_By (P : Free_Part; Short, Len, Noise : Long_Float) return Boolean;
   --  这一段路第一次走 Arm 这条臂:装进开机探针量的那几步(导出给自检)
   procedure Seed (W : in out Walk; M : Body_Map; Arm : Natural);
   type Leg_Step is record
      Arm        : Natural := 0;
      From       : Plug.Arm_Pose := [others => 0.0];   --  这一步开始时的读数
      Aim        : Plug.Arm_Pose := [others => 0.0];   --  这一步发出去的目标(缩过以后)
      Cmd, Got   : Table.Vec := Table.Zero_Vec;        --  要走的 / 实到的(都从 From 算,按通道:平移 3 + 转动 3)
      Len, Ang   : Long_Float := 0.0;                  --  这一步要平移多长、转多少
      Went, Turned : Long_Float := 0.0;                --  沿要的方向实到的平移、转动
      Arrived    : Boolean := False;                   --  走完在 Aim 一步看得见的那一档以内(Loose 才判)
      Blocked_T, Blocked_R : Boolean := False;         --  平移 / 转动被挡住了(Blocked_By;走到了、被叫停、还在动都不判)
      Reach_Cut  : Boolean := False;                   --  反解够不到,缩过(Len = Ang = 0 ⇒ 一截都够不到,没发)
      Halted     : Boolean := False;                   --  途中 Watch 叫停
      Moving     : Boolean := False;                   --  等满了还在动(不是它自己停下的)
      Left, Left_Rot : Long_Float := 0.0;              --  走完离 Goal 还差的平移、转动
   end record;
   package Leg_Step_Vectors is new Ada.Containers.Vectors (Natural, Leg_Step);
   --  走到一个目标 = 同一个 Walk 一步一步 Step(每步从此刻的读数起走还差的):每组都差不到分辨率就到了;一步下去哪组都没再近过分辨率
   --  = 这就是此刻能到的最近(身体到头 / 被挡住),照实报还差多少;有一组 Blocked_T / Blocked_R 就是被挡住(同接触集里"眼走到悬停点"那一段)
   procedure Step (L : in out Plug.Link; M : Body_Map; Legs : Leg_Vectors.Vector; Lim : Limits; F : in out Plug.Frame;
                   W : in out Walk; Rep : out Leg_Step_Vectors.Vector; Frames : out Natural; Ok : out Boolean);

   --  ── 走近一件东西每一步多大(I6 的调用方用;纯函数,导出给自检)── 全是量到的量,没有拍的数:
   --  小步(到"可能碰到它"的那条带子里每一步多大)= 手自己的不准(Stats.Z × √(指尖 Tip_Sd² + 这一次到位差 Miss² + 读数噪声²))
   --  分给 Blocked 当底的那几步(Free_Base);再小也得是这只眼看得出的一步:地板 = Stats.Z × max(读数噪声, 一档 × 眼的像素残差),
   --  而且不小于一档(比一档小的一步在它自己那只眼里不到一个像素)。同接触集往下伸的小步(Act.Plan_Descent 同一个式子)
   function Careful_Step (Tip_Sd, Miss, Noise, Notch, Eye_Rms : Long_Float) return Long_Float;
   type Approach_Plan is record
      Res   : Long_Float := 0.0;   --  到没到的分辨率:还差的不到"量它、量我自己"的不准(Stats.Z 倍)⇒ 分不出还差,就是到了(不小于小步的地板)
      Band  : Long_Float := 0.0;   --  可能碰到它的那条带子从它的中心往外多远 = 它朝我这边的半径 + Res
      Clear : Long_Float := 0.0;   --  这一步最多走多远还不进带子(带子前留出 Free_Base 步小步给 Blocked 当底);≤ 0 = 已经在带子里
      Lstep : Long_Float := 0.0;   --  带子里每一步多大(Careful_Step)
   end record;
   --  Dist = 指尖该到的那一点(它的中心)还差多远;R_Obj = 它朝我这边的半径(画面里的框按远近折出来的;量不出 ⇒ Long_Float'Last:全程小步);
   --  Sd_Target = 它的位置沿走的方向有多不准(两眼交点按各眼的误差算的;量不出 ⇒ Long_Float'Last:全程小步,照实;
   --  到没到只按我自己看得出的那一步判)
   function Plan_Approach (Dist, R_Obj, Sd_Target, Tip_Sd, Miss, Noise, Notch, Eye_Rms : Long_Float) return Approach_Plan;
   --  脑说的步子档位 = 这一步最多多大(语言 §17.6;大并行 §2 第 23 条):small = 小步(Small,Careful_Step),large = 一条命令走得到的
   --  最大一档(Large,开机量的步幅),medium = 两者的几何中点(对数尺上的正中;两头都是量的);别的词 / 没说 ⇒ 不加上限(Long_Float'Last)
   function Gear_Bound (Gear : String; Small, Large : Long_Float) return Long_Float;
   --  这一步比上一步之前又近了没有:Prev = 上一步之前还差多少(Long_Float'Last = 还没走过),Now = 此刻还差多少,Res = 分辨率。
   --  近了不到一个分辨率 = 看不出近了 ⇒ False(走到头了 / 被什么拦着,不再拿同一步去撞;原来是拍一个"最多走几步")
   function Gained (Prev, Now, Res : Long_Float) return Boolean is (Prev = Long_Float'Last or else Prev - Now > Res);
   --  走到一个定了的目标(一组,Goal 是绝对位姿):同一个 Walk 一步一步 Step,每步走还差的全部,上限按 Lim;
   --  到了 = 平移、转动都差不到 Res / Res_Rot;被挡住(Blocked)、一步下去哪样都没再近过分辨率(Gained)、走满 Max_Steps(> 0)、线断了就收
   type Walk_End is (Arrived, Was_Blocked, No_Gain, Max_Steps_Done, Lost_Link);
   procedure Walk_To (L : in out Plug.Link; M : Body_Map; G : Leg; Lim : Limits; Res, Res_Rot : Long_Float; Max_Steps : Natural;
                      F : in out Plug.Frame; W : in out Walk; Went, Turned : out Long_Float; Steps : out Natural; Why : out Walk_End);
end Selfmap;
