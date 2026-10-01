--  部件和轴(大并行.md §2 第 12 条,路 6)。一件东西由几块组成、块和块之间绕哪根轴转 / 沿哪根轴走,全靠看它动量出来。
--  ① 量(Fit):东西身上被跟住的每个点、每一帧在世界里在哪(两眼同一刻的交点 + 它的协方差,Geom.Meet / Meet_Cov 那种;路 3 的"东西"交来),
--     按刚体运动把一起动的点归成一块;两块之间的相对运动拟合成一根轴 —— 转轴(方向 W、过哪一点 P)或滑轴(方向 W),带不确定度。
--     和量身体关节是同一段拟合:一根轴就是 Kinem.Axis,两块之间的相对位姿就是只有这一根轴的 Kinem.FK(转:绕 W 过 P 转 q 弧度;走:沿 W 走 q)。
--     身体关节的转角是读数给的;东西的没人给,和轴一起解。解法是最大似然:每一块每一帧的位姿、每个点在它那一块上的位置(形状)、轴,
--     残差 = 观测减预测按给的协方差白化;形状对位姿是线性的,按式子消掉,剩下的交给 Kinem.Robust_LM(同身体关节那一份抗野点的 LM)。
--  ② 照实说:没见过它动(或只见过它整块动)⇒ 一块,"当一整块";两块之间动得太少、分不出是转还是走 ⇒ Undecided("定不下");
--     一根轴说不清两块的相对运动 ⇒ Not_One_Axis;哪块都说不通的点(跟错了的、混进来的)⇒ Unexplained,不硬塞进哪一块;
--     两块都说得通、两块对它的预测又隔不开的点(轴附近的:两种动法预测到同一处)⇒ Ambiguous;
--     同一个刚体被拿成了两块(隔得远的两小团点:一团自己的位姿外推不到另一团)⇒ 合成一块比各自一块多出来的卡方不显著,就合起来。
--     成团地看才分得出、一个个看都分不出的一小团(离轴远、只转了零点几度的一小块)这里分不出来,会并进旁边那一块
--     (09-30 离线 40 组种子:0.3° 那一场 12 / 40 次并成一块;并成一块时照实说"只见过它整块动",不报假的轴)。
--  ③ 走(Follow):第一次碰、还没见它动过 ⇒ 轴不知道 ⇒ 顺着它让的方向走:按脑要的方向走一小步,量它真往哪挪了就接着往哪挪;
--     每一步都从手此刻在哪起算(手被挡住时顶着的那一截不超过一步,力不累加);哪边都不让 ⇒ 照实说试过哪几个方向、它往哪让过。
--  不认单位和尺度:所有的门 = 这一次量到的噪声(给的协方差 × 量出来的倍数 Sigma)× 统计的 Z(Stats.Z);数据自己的尺度只用来定数值差分的步子。
--  不认零件个数:几块都行,每两块之间各拟合一根。代码里没有东西的名字、没有动作的名字
with Geom;
with Kinem;
with Bytes;
with Ada.Containers.Vectors;
package Linkage is
   subtype V3 is Geom.V3;
   subtype M3 is Geom.M3;

   --  ── 输入:东西身上被跟住的点 ──
   --  一个点在一帧里:看见没有、在世界里在哪(世界单位,驱动用什么单位就是什么)、协方差(世界单位²)
   type Obs is record
      Seen : Boolean := False;
      X : V3 := [others => 0.0];
      Cov : M3 := [others => [others => 0.0]];
   end record;
   package Obs_Vectors is new Ada.Containers.Vectors (Natural, Obs);
   --  一条轨迹 = 同一个真实的点在各帧里(下标 = 帧号;比别的短 = 后面几帧没看见)
   package Track_Vectors is new Ada.Containers.Vectors (Natural, Obs_Vectors.Vector, Obs_Vectors."=");

   --  ── 输出 ──
   --  一块在一帧里的位姿:这一块上的点在参照帧那一刻的世界位置 x ⇒ 这一帧在世界里 R x + T(参照帧 = 单位阵、零平移)。
   --  Ok = False:这一帧这一块看见的点不够铺开(显著地不在一条线上),定不住它
   type Pose is record
      Ok : Boolean := False;
      R : M3 := Geom.Identity;
      T : V3 := [others => 0.0];
   end record;
   package Pose_Vectors is new Ada.Containers.Vectors (Natural, Pose);
   type Piece is record
      Members : Geom.Nat_Vectors.Vector;   --  哪几条轨迹(下标)
      Poses : Pose_Vectors.Vector;         --  每一帧
   end record;
   package Piece_Vectors is new Ada.Containers.Vectors (Natural, Piece);

   --  两块之间:Found = 找到一根轴(Ax.Slide 说是转轴还是滑轴);Undecided = 动得太少,转还是走分不出(定不下);
   --  Not_One_Axis = 一根轴说不清它俩的相对运动(比各走各的差得显著);No_Common_Frame = 除了参照帧,没有一帧两块都定得住
   type Axis_Status is (Found, Undecided, Not_One_Axis, No_Common_Frame);
   type Joint is record
      A, B : Natural := 0;                    --  B 相对 A(A 是先归出来的、点多的那一块)
      Status : Axis_Status := Undecided;
      Ax : Kinem.Axis;                        --  Found:同身体关节的轴(参照帧那一刻的世界系);转:W 单位向量、P 轴上离 C 最近的点;走:W 单位向量
      W_Sd : Long_Float := Long_Float'Last;   --  轴方向的不确定度(弧度,一倍标准差,最不准的那个方向)
      P_Sd : Long_Float := Long_Float'Last;   --  转轴:轴的位置垂直于轴的不确定度(世界单位,一倍标准差,最不准的那个方向)
      Q, Q_Sd : Bytes.Floats;                 --  每一帧的关节量(转:弧度;走:世界单位;参照帧 = 0)和不确定度;那一帧没量 = 0 / Long_Float'Last
      C : V3 := [others => 0.0];              --  量曲率用的那一点:B 上相对 A 挪得最远的那个点(参照帧那一刻)
      Kappa, Kappa_Sd : Long_Float := 0.0;    --  C 走的那条路的曲率 |k|(1 / 世界单位;转 = 1 / C 离轴多远,走 = 0)和曲率向量 k 最不准那个方向的一倍标准差
      Reach : Long_Float := 0.0;              --  这两块看得见的点离 C 最远多远(轴要是长在这件东西上,离 C 不会比它远)
      Min_Radius : Long_Float := 0.0;         --  要是转轴,它离 C 至少这么远:1 / (|k| + √(Gate (2) λmax))(k 的置信椭圆离 0 最远处);定不住 ⇒ 0
      Chi, Chi_Gate : Long_Float := 0.0;      --  一根轴比两块各走各的多出来的代价(卡方)、它的门;Chi > Chi_Gate ⇒ Not_One_Axis
      Dof : Natural := 0;                     --  卡方的自由度 = 各走各的比一根轴多的参数个数
      Common : Natural := 0;                  --  两块都定得住的帧数(不算参照帧)
   end record;
   package Joint_Vectors is new Ada.Containers.Vectors (Natural, Joint);

   --  每条轨迹:归进了一块 / 两块都说得通 / 哪块都说不通 / 用不上(看见不到两帧)
   type Role is (Member, Ambiguous, Unexplained, Unused);
   package Role_Vectors is new Ada.Containers.Vectors (Natural, Role);

   type Report is record
      Frames : Natural := 0;                  --  几帧
      Ref : Natural := 0;                     --  参照帧(看见的点最多的那一帧)
      Pieces : Piece_Vectors.Vector;
      Joints : Joint_Vectors.Vector;          --  每两块一根:(0,1)、(0,2)、…、(1,2)、…
      Roles : Role_Vectors.Vector;            --  每条轨迹
      Piece_Of : Bytes.Ints;                  --  每条轨迹归哪一块(-1 = 不归)
      Sigma : Long_Float := 0.0;              --  量出来的噪声倍数:真的协方差 = 给的 × Sigma²(给的协方差准 ⇒ ≈ 1)
      Sigma_Dof : Long_Float := 0.0;          --  它有多准:量它的有效自由度(信给的协方差 = Long_Float'Last);门按它放宽(F 分布)
      Moved : Boolean := False;               --  只有一块时:它整块动过没有(显著地)
      Settled : Boolean := True;              --  归块 / 量噪声做到了不再变(False = 保险上限到了还在变,照实报)
      Bad_Cov : Natural := 0;                 --  协方差不是正定的观测几笔(当没看见)
      Scale : Long_Float := 0.0;              --  数据自己的尺度(各点离形心的均方根;只定数值差分的步子)
   end record;

   --  量:归块、每一块每一帧的位姿、每两块之间的轴
   procedure Fit (Tracks : Track_Vectors.Vector; Rep : out Report);
   --  日志里一句话说清量出了什么(世界单位照印,不换成米)
   function Say (Rep : Report) return String;

   --  ── 统计:同一个置信度(一维正态单侧 Stats.Z 倍)下 ν 个自由度的卡方分位 ──
   --  Wilson–Hilferty(1931):(χ²/ν)^(1/3) 近似正态,均值 1 − 2/(9ν)、方差 2/(9ν) ⇒ 分位 ≈ ν (1 − 2/(9ν) + Z √(2/(9ν)))³。
   --  ν 维白化残差的平方和超过它 = 和一维超过 Z 倍一样少见。ν = 3 时 15.9(准的 15.6),ν = 1 时 10.5(准的 10.3)
   function Gate (Nu : Positive) return Long_Float;
   --  同一个置信度,但分母的噪声是量的(有效自由度 Dof):白化残差平方和 ÷ 量出来的噪声² 服从 ν F(ν, Dof),不是卡方 ——
   --  Paulson(1942)的立方根近似:[(1 − b) x − (1 − a)] / √(b x² + a) 近似正态,x = F^(1/3),a = 2/(9ν),b = 2/(9 Dof),令它 = Z 解 x(大根)。
   --  Dof = Long_Float'Last(噪声没量、信给的)⇒ 正好是 Gate;量噪声的样本少 ⇒ 门宽;少到连"有多准"都定不住 ⇒ 无穷(Long_Float'Last)
   function Gate_F (Nu : Positive; Dof : Long_Float) return Long_Float;
   --  反过来:ν 个自由度的平方和 Chi(÷ 量出来的噪声²,有效自由度 Dof)相当于一维正态的几倍(同一个 Paulson / Wilson–Hilferty 近似;
   --  Chi > Gate_F (Nu, Dof) ⇔ Z_Of (Chi, Nu, Dof) > Stats.Z)。几个平方和比"哪个最不显著",比这个(自由度不同也能比)
   function Z_Of (Chi : Long_Float; Nu : Positive; Dof : Long_Float) return Long_Float;
   --  马氏距离的平方 rᵀ C⁻¹ r。C 求不了逆(比如给的是 0:这个量量得毫无误差)⇒ r 不是 0 就当无穷远,是 0 就是 0
   function Mahal (R : V3; C : M3) return Long_Float;

   --  ── 顺着它让的方向走 ──
   --  走一步的结果(调用方按今天的 Geo_Move 写:命令手沿 Dir 挪 Len,量手实到多少、东西那一处挪了多少;路 4 交"走一步"以后换那一个)
   type Step_Report is record
      Ok : Boolean := False;                        --  身体照做了(线没断、命令发出去了)
      Hand : V3 := [others => 0.0];                 --  手这一步实到的位移(关节读数按运动学算的,同 Geo_Move 印的"实到")
      Hand_Cov : M3 := [others => [others => 0.0]]; --  它的协方差(读数噪声)
      Thing : V3 := [others => 0.0];                --  推着 / 拿着的那一处这一步挪了多少(眼量的:两次交点之差)
      Thing_Cov : M3 := [others => [others => 0.0]];--  它的协方差
      Blocked : Boolean := False;                   --  这一步被挡住了没有(Selfmap.Blocked,调用方按这一段空走的历史判;碰到没有只有这一个判法)
   end record;
   --  Arrived = Goal 说到了;Stuck = 哪边都不让(Tried 是那一轮试过的方向,Yields 是它往哪让过、但不顺着脑要的方向);
   --  Left_Behind = 手走了、它没跟来(拿着的 = 滑了,推着的 = 离开它了);Out_Of_Steps = 步数用完;Body_Failed = 身体没照做;
   --  No_Direction = 给的 Want 不是一个方向(零向量 / 不是数)、步长不是正数、或者没给走一步的办法:一步没走
   type Follow_End is (Arrived, Stuck, Left_Behind, Out_Of_Steps, Body_Failed, No_Direction);
   type Follow_Report is record
      How : Follow_End := Out_Of_Steps;
      Steps : Natural := 0;                         --  走了几步(试方向的也算)
      Along : Long_Float := 0.0;                    --  它沿脑要的方向一共挪了多少(世界单位;每一步量到的都加上)
      Moved : V3 := [others => 0.0];                --  它一共挪了多少(每一步量到的加起来)
      Dirs : Geom.V3_Vectors.Vector;                --  每一步命令的方向
      Tried : Geom.V3_Vectors.Vector;               --  最后那一轮(上一次让了以后)试过、没顺着让的方向
      Yields : Geom.V3_Vectors.Vector;              --  最后那一轮里它挪了、但不顺着脑要的方向的那几次,它往哪挪的
   end record;
   --  Want = 脑要它往哪挪(世界系,不必是单位向量);Len = 一步多长(Light_Len);Budget = 最多几步(脑说的"或者 N 步")。
   --  每一步沿此刻的方向走 Len,量它挪了没有("挪了"= 马氏距离平方过 Gate (3),而且不比这一步的百分之一 —— Selfmap.Negligible,
   --  "一步里可以忽略的那一丝" —— 还小):
   --    沿 Want 推的那一步挪了 ⇒ 下一步沿它真挪的方向(不会自己动的约束只会让它往推的那一侧挪);
   --    垂直于 Want 试的那一步挪了、而且这一步就显著地顺着 Want ⇒ 同上;挪了但不顺 ⇒ 记进 Yields,接着试;
   --    正顺着它挪的方向走时:它沿 Want 挪的一段一段累计(序贯):显著地顺了 ⇒ 这一段算数、试过的方向清掉;显著地往回了 ⇒ 停下来试;
   --    没挪 ⇒ 这个方向记进 Tried,按 Want、垂直于 Want 的 ±E1、±E2 依次试(每一个都是从手此刻在哪起的一小步);都试过 ⇒ Stuck。
   --  手显著地走了、它没挪、也没被挡 ⇒ Left_Behind
   --  Step = 走一步(沿单位方向 Dir 挪 Len);Goal = 脑要的到了没有(调用方按脑说的"到什么为止"判;null = 不判)
   procedure Follow (Want : V3; Len : Long_Float; Budget : Natural;
                     Step : access procedure (Dir : V3; Len : Long_Float; R : out Step_Report);
                     Goal : access function return Boolean;
                     Rep : out Follow_Report);
   --  一小步多长:它挪了 Len 在最不准的方向上也看得出来(Len² ≥ Gate (3) × Thing_Cov 最大的特征值),而且不比手靠得住的最小一步 Floor 小。
   --  力 ∝ 顶着的那一截 ≤ 一步 ⇒ 用看得出的最小一步推,就是最轻的推
   function Light_Len (Floor : Long_Float; Thing_Cov : M3) return Long_Float;
end Linkage;
