--  运动学(V1b 第三步,2026-09-26):一只手 = 一串转轴,只凭【关节读数】+【手上那只眼的画面】量出来 ——
--  不要图纸、不要零点、不要身体报的"手在哪"。开机关节扫描(Act.Geo_Boot_Sweep)每个关节单独一格一格转,
--  每一格的转角 = 关节读数的差(已知),画面里桌面挪了多少由配点仪器给(Instrument.Match)。
--  每根轴:方向 W(单位向量)、过哪一点 P,都在"参照读数 Q0 那一刻手上那只眼"的相机系里(同 Geom:-z 朝前、+y 朝上)。
--  T(q) = exp([S_0](q_0 − Q0_0)) · … · exp([S_n−1](q_n−1 − Q0_n−1)) = 那只眼在参照眼系里的位姿(X_参照 = R · X_眼 + t)。
--  只靠相机和转角量不出米:长度只差一个倍数,解的时候钉"参与拟合的各帧眼的位置均方根 = 1"(模型单位)。
--  量法只有一种(owner 2026-09-26):① 每根轴单独(已知转角绕一根轴转:方向铺网格,"轴在眼哪边"直接解,焦距各轴共用一起定);
--  ② 各轴离眼远近的比例(至少两个关节转了的格子之间的配点;方程对比例是线性的 ⇒ 任取三对起步、全局挑最好的);③ 全部配点按像素一起解(焦距放开)。离线同一套在 x5 仿真上:焦距 396.5 / 396.9(真 397),
--  只给关节读数算手的位置,离标定点 40° 以上最大 2.47 mm(LAB 09-26 V1B2)。驱动外面那份 Python 原型 09-26 按 owner 删了:解法只有这一份
with Geom; use Geom;
with Plug;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Kinem is
   --  统计换算(正态噪声):一维残差的中位绝对偏差 × Mad_Sigma = 标准差(1 / Φ⁻¹(3/4))
   Mad_Sigma : constant := 1.4826;
   --  Huber 的门,以量到的 σ 为单位:正态噪声下效率 95% 的那个数(Huber 1964 的约定)。Robust_LM、每根轴起步的网格(Best_Phi)、
   --  ② 定比例都用它 —— 残差一律先除以量到的 σ 再比(09-30 以前 kinem 自己喂的是像素,门 1.0 = "1 像素";jointboot 喂白化的,同一个 1 是 1σ)
   Huber_K : constant := 1.345;
   --  Tukey 双权的门,以量到的 σ 为单位:门外的残差权为 0(正态噪声下效率 95% 的那个数,Beaton & Tukey 1974 的约定)。
   --  只在"谁占多数就落到谁那边、另一拨一点都不拽"的地方用(Rides_On_Eye);Huber 门外的点还按 δ/|r| 拽着
   Tukey_C : constant := 4.685;
   --  一根轴是"转"还是"沿轴走"(平移)是开机扫描量出来的(① 同一批配点按两样各解一次,残差小的那样;09-27 无人机那一半):
   --  转:W = 转轴方向(单位向量),P = 轴上一点,读数差 = 转角(弧度);
   --  走:W = 走的方向 × 每一个读数单位走多远(模型单位 / 读数单位,长短是量出来的),P 不用(0)
   type Axis is record
      W : V3 := [0.0, 0.0, 1.0];   --  转轴方向(单位向量,参照眼系);走的关节:方向 × 每个读数单位走多远
      P : V3 := [0.0, 0.0, 0.0];   --  轴上一点(参照眼系,模型单位);走的关节不用
      Slide : Boolean := False;    --  True = 沿 W 走(平移关节)
   end record;
   package Axis_Vectors is new Ada.Containers.Vectors (Natural, Axis);
   --  一只手的一串轴:根数跟着这组读数有几个走,不设上限(09-30:原来按 12 根开死,读数多于 12 个的那一组,
   --  多出来的关节被悄悄截掉 —— 它们照样带着眼动,运动学错而且不报)。
   --  读 Ax (J):还没写过的那根 = 缺省的轴;写 Ax (J):根数不够就先添到 J + 1 根(添的都是缺省的轴)
   type Axes is tagged record
      V : aliased Axis_Vectors.Vector;
   end record
     with Constant_Indexing => Axis_Of, Variable_Indexing => Axis_Ref;
   function Axis_Of (A : Axes; J : Natural) return Axis;
   function Axis_Ref (A : aliased in out Axes; J : Natural) return Axis_Vectors.Reference_Type;
   --  一个像素位置(配点在它那一帧里问的那个格点)
   type Px is record
      U, V : Long_Float := 0.0;
   end record;
   package Px_Vectors is new Ada.Containers.Vectors (Natural, Px);
   type Model is record
      Valid : Boolean := False;
      N : Natural := 0;                --  几根轴(= 这组关节读数几个)
      Ax : Axes;
      F, Cx, Cy : Long_Float := 0.0;   --  手上那只眼的焦距(像素)、主点
      Q0 : Floats;                     --  参照读数
      Eye : Px_Vectors.Vector;         --  长在这只眼上的像素(Fit 按 Eye_Pixels 量;解、三角都不用从它们出发的配点);身体文件不存(开机以后用不着)
   end record;

   --  只给关节读数 ⇒ 那只眼在参照眼系里的位姿
   procedure FK (M : Model; Q : Floats; R : out M3; T : out V3);

   --  一对配点:第 I 帧的像素 (Ua, Va) 和第 J 帧的 (Ub, Vb) 是同一个真实的点。
   --  Pt = 它属于哪条轨迹:同一个号 = 第 I 帧里同一个像素(仪器按问的点配)配进了好几帧,是同一个真实的点(-1 = 不成轨迹)
   type Corr is record
      I, J : Natural := 0;
      Ua, Va, Ub, Vb : Long_Float := 0.0;
      Pt : Integer := -1;
   end record;
   package Corr_Vectors is new Ada.Containers.Vectors (Natural, Corr);

   --  一帧:这一刻这组关节的读数,和它是扫哪个关节扫出来的(-1 = 起点 / 不是单独扫一个关节的帧)
   type Frame_Info is record
      Q : Floats;
      Joint : Integer := -1;
   end record;
   package Frame_Vectors is new Ada.Containers.Vectors (Natural, Frame_Info);

   type Fit_Report is record
      F_Start : Long_Float := 0.0;     --  ① 各轴一起定的焦距(网格那一档)
      F_Axes : Long_Float := 0.0;      --  ①b 焦距和各轴一起精修以后
      F : Long_Float := 0.0;           --  ③ 最后一起解的焦距
      Joint_Med : Floats;              --  ① 每根轴单独精修后的残差中位(像素;不能量的轴 = -1):按它最后认的那样(转 / 走)
      Joint_Med_Turn, Joint_Med_Slide : Floats;   --  ① 同一批配点按"转"、按"走"各解一次的残差中位(像素;这一样试不了 = -1)
      Slide : Bools;                   --  ① 每根轴认成了"走"(平移关节)
      Joint_Frames : Nat_Vectors.Vector;   --  ① 每根轴用了几帧(别的关节被顶偏的格子不用)
      Rho : Floats;                    --  ② 各轴离眼远近的比例(以 Ref_Joint 为 1)
      Ref_Joint : Natural := 0;
      Rho_Pairs : Natural := 0;        --  ② 用了几对(两帧加起来至少两个关节离开参照读数的)
      Rho_Start_Px, Rho_Px : Long_Float := 0.0;   --  ② 三对起步最好的那个(截断到 3 px 的均方根)、全部重解以后(中位):这些配点的 Sampson 残差(像素)
      Med_Px, P90_Px : Long_Float := 0.0;   --  ③ 最后一起解的 Sampson 残差(像素)中位 / 九成
      Sig_Px : Long_Float := 0.0;      --  ③ 最后一轮量到的配点噪声 σ(像素;Mad_Sigma × 全部配点 Sampson 残差的中位,残差除以它再进 Huber)
      Rounds : Natural := 0;           --  ③ "按模型重挑内点 + 一起解"做了几轮(做到内点集不再变)
      N_Corr, N_Used : Natural := 0;   --  配点总数 / 进最后一起解的内点数
      Eye_Px, Eye_Corrs : Natural := 0;   --  长在眼上的像素几个、从它们出发的配点几笔(不进解;见 Eye_Pixels)
      Flipped : Boolean := False;      --  平移整体反了一次号(Sampson 分不出,按点在不在两只眼前面定)
      Mv_Tracks, Mv_Obs : Natural := 0;               --  ④ 多视图一起解用了几条轨迹、几笔(轨迹在别的帧里的像素)
      Mv_Start_Px, Mv_Px, Mv_P90_Px : Long_Float := 0.0;   --  ④ 重投影残差(像素):起步中位、解完中位 / 九成
      Mv_Iters : Natural := 0;
      Mv_Passes : Natural := 0;        --  ④ 做了几遍(从上一遍的结果再做,直到残差中位不再降;最后一遍只是确认)
      Mv_Rounds : Natural := 0;        --  ④ 留下的那一遍里"重挑内点 + 解"做了几轮(做到内点集不再变)
      Mv_Sig_Px : Long_Float := 0.0;   --  ④ 量到的配点噪声 σ(像素;重投影残差垂直于对极线那一分量的 Mad_Sigma × 中位 —— 远近解掉的只是沿对极线那一分量)
      Secs : Floats;                   --  各步用了几秒(墙上时间):① 网格、① 精修、①b 焦距和各轴一起、② 比例、③ 一起解、④ 多视图
      Unsettled : Ada.Strings.Unbounded.Unbounded_String;   --  碰到保险上限还没收住的那几步(空 = 每一步都做到了不再变);不空就照实印出来
   end record;

   --  每根轴单独起步收格子的门:别的关节偏得让画面挪不到 1 像素(按焦距网格最长那档算,最严)= 1 ÷ 最长焦距(弧度 / 读数单位)。
   --  开机扫描"到了"时别的关节也按这道门等(Jointboot,H1 2026-09-28:人形别的关节偏 0.001–0.009 就读,格子全不干净,两只手运动学没量成)
   function Clean_Tol (Width : Long_Float) return Long_Float;
   --  这一帧是不是只动了一个关节:是扫那个关节扫出来的(Frames (Fr).Joint),别的关节离参照读数都不到 Dmax(画面挪不到 1 像素)⇒ 那个关节;
   --  参照帧自己、几个关节一起动的帧、别的关节偏了的帧 ⇒ -1。每根轴单独起步收格子(Fit)、认长在眼上的像素(Eye_Pixels)都按它
   function Single_Joint (Frames : Frame_Vectors.Vector; Ref, Fr : Natural; Dmax : Long_Float) return Integer;
   --  长在眼上的像素(2026-09-28 人形 H2 / H3):参照帧上问的一个格点,在【两个以上关节】各自单独转的格子里【各至少有一格】
   --  (参照帧 ↔ 只动了这一个关节的帧;这一对里挪了的配点比没挪的多 = 眼确实动了)没挪过配点精度(Geom.Trip_Px)⇒ 它跟着眼走
   --  (自己的手、夹爪),不是静止的世界。世界里的点只有落在一根转轴的方向附近才可能不挪;两根轴的方向附近都落,只有两根轴几乎平行、
   --  共同的方向又在画面里时才会有 —— 那几个背景像素对这两根轴本来就几乎没信息,认成手只是少用几个点。不要"每一格都不挪":
   --  人形的手指软,有的格子抖 1–3 px,按"每一格"漏掉三成手上的像素,H3 第一只手照样被拉歪(8.7 mm);按"有一格"全认出、0.21 mm。
   --  "这一对眼动了没有"按多数:相机下游的关节转时世界不动、手在动 ⇒ 那几对不算数,背景不会被认成手。
   --  人形腕眼画面三分之一是自己的手:这些配点满足"眼没转"的解,把定比例那一步拉歪,两只手运动学错 16–20 mm;
   --  x5 同一条规则只认出两边的夹爪,运动学不变(LAB H2、H3)。扫描每一对问的是同一张格子(Jointboot)⇒ 同一个像素在哪一对里都是它
   function Eye_Pixels (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Width : Long_Float) return Px_Vectors.Vector;
   --  配点里去掉从 Eye 这些像素出发的(不管哪一对)
   function Off_Eye (Eye : Px_Vectors.Vector; Cs : Corr_Vectors.Vector) return Corr_Vectors.Vector;
   --  Frames(Ref) = 参照帧(扫描起点);Width = 画幅宽(像素,焦距网格按它铺:视场 30°–110°)。
   --  先按 Eye_Pixels 认出长在眼上的像素(记进 M.Eye、Rep.Eye_Px / Eye_Corrs),从它们出发的配点不进解。
   --  Ok = False:能量的轴不够 / 配点不够(Rep 里照实写到哪一步)
   procedure Fit (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                  M : out Model; Rep : out Fit_Report; Ok : out Boolean; Per_Pair : Positive := 60);
   --  Per_Pair:最后一起解时每一对最多取几个内点(次数;驱动开机永远用默认 —— 离线回放 kinexam 的 KINEXAM_PER_PAIR 才改,做对照实验)

   --  Fit 的最后一步(④ 多视图:轨迹按重投影一起解,M 当起步)单独拿出来,给自检焊点用;从 M.Eye 那些像素出发的配点不用(同 Fit)
   procedure Refine_Tracks (Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; M : in out Model; Rep : in out Fit_Report);

   --  轨迹的点(参照眼系,模型单位):在它起点那帧(I)的视线上,远近按它进的每一帧的像素一起解(多视图三角;抗野点)
   type Track_Pt is record
      X : V3 := [0.0, 0.0, 0.0];
      Var_Along : Long_Float := 0.0;   --  沿视线的方差(模型单位²):配点噪声 ÷ 这一维的曲率
      I : Natural := 0;                --  起点那帧
      U, V : Long_Float := 0.0;        --  在起点那帧里的像素
      Far : Natural := 0;              --  看见它的帧里离起点那帧的眼最远的那一帧
      Views : Natural := 0;            --  连起点那帧几帧看见
      Med_Px : Long_Float := 0.0;      --  这条轨迹在各帧的重投影残差中位(像素)
   end record;
   package Track_Pt_Vectors is new Ada.Containers.Vectors (Natural, Track_Pt);
   --  按模型把每条轨迹(Pt >= 0 的配点,按 Pt 归到一起)的点解出来;只给起点在 Only_I 那帧的(-1 = 全部)、至少 Min_Views 帧看见的;
   --  Sig_Px = 这些轨迹配点的噪声 σ(像素,每个方向):重投影残差垂直于对极线那一分量的 Mad_Sigma × 中位 —— 远近怎么解都动不了这一分量,
   --  它就是一维正态(09-30 改:原来 1.4826 × 二维残差长度的中位,多视图轨迹上长度的中位 = 1.1774σ ⇒ 偏大 1.75 倍;
   --  只有两帧的轨迹沿对极线那一分量被远近解掉、长度只剩一维 ⇒ 按二维换也不对);从 M.Eye 那些像素出发的配点不用(同 Fit)
   procedure Track_Points (M : Model; Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; Only_I : Integer; Min_Views : Natural;
                           Tracks : out Track_Pt_Vectors.Vector; Sig_Px : out Long_Float);

   --  ── 两只手的系对齐到一个世界(V1b 3c)──
   type V3_Array is array (Natural range <>) of V3;
   --  一团点里的那张面(同样的最小中位数):面上一点 P0、单位法向 Nrm
   procedure Robust_Plane (X : V3_Array; P0, Nrm : out V3; Inliers : out Natural; Med : out Long_Float);

   --  抗野点的 LM(数值雅可比):Resid 把全部残差填进 R(长度 N_R);前 N_Rob 个按 Huber(门 Huber_K)迭代加权,后面的(约束行)原样。
   --  调用约定(09-30 统一):前 N_Rob 个残差一律是除以量到的 σ 之后的(以 σ 为单位)—— kinem 自己的 Sampson 像素残差除以当场量到的
   --  Mad_Sigma × 中位,jointboot 放进世界的是白化过的。拿出来给别的包用(几只手放进一个世界,2026-09-26)。
   --  一轮里阻尼一直往上调,直到代价降了、或者步子小到参数的数值分辨率以下(= 到底了;09-30 以前最多调 8 次就判到底,平谷里停早)。
   --  Done = 收住了(这一步降得不到十亿分之一,或者步子已经小到数值分辨率以下);False = 做满 Iters 还在降:Iters 只当保险,调用方照实报
   type Vec is array (Natural range <>) of Long_Float;
   procedure Robust_LM (X : in out Vec; N_R, N_Rob : Natural; Iters : Positive; Step : Vec;
                        Resid : not null access procedure (X : Vec; R : out Vec); Done : out Boolean);

   --  配点问的格点(09-30 从 Jointboot 挪来:一张格子全仓一份):整幅 Gx × Gy 个格子,问每一格的中心(采样密度,次数;4:3 画幅上 20 px 一格)。
   --  开机扫描、认长在眼上的像素(Eye_Pixels / On_Eye_Grid)、判抓握通道哪头张开(Zone.Measure)问的都是这一张 —— 同一个像素在哪一对里都是它
   Gx : constant := 32;
   Gy : constant := 24;
   function Grid_U (I : Natural; W : Positive) return Long_Float is ((Long_Float (I) + 0.5) * Long_Float (W) / Long_Float (Gx));
   function Grid_V (J : Natural; H : Positive) return Long_Float is ((Long_Float (J) + 0.5) * Long_Float (H) / Long_Float (Gy));
   --  (U, V) 落在 W × H 画面、Gx × Gy 格子的哪一格,那一格的格点是不是 Eye 里的(长在眼上的 ⇒ 这一处被自己的手挡着)。
   --  配进一只手的腕眼时落在这儿的不可能是桌面上的点(09-28 H4 / H5:第二只手的桌面点配到第一只手画面里它自己那只白手上的 69 / 29 对,按真值全错、差 300–450 px)
   function On_Eye_Grid (Eye : Px_Vectors.Vector; U, V : Long_Float; W, H : Positive) return Boolean;

   --  眼转了一下,画面里哪些点长在眼上(09-30,判抓握通道哪头张开、瓣按长在眼上补全):同一只眼转之前 / 转之后两帧,问的点 (Pu, Pv)、配到的 (Bu, Bv);
   --  G = 这只眼量过的焦距、主点、畸变(开机运动学量的)。驱动发的手的位姿就是这只眼的位姿,绕它转 = 眼在原地转:
   --  世界在画面里按一个转动挪(3 个数:视线 d 转成 R·d 再投回去,畸变照算),和世界多远无关;长在眼上的点(自己的手指、手里的东西)不挪。
   --  转动按全部点抗野点拟合:先最小二乘,再按 Tukey 双权迭代加权(残差除以当场量的 σ = Mad_Sigma × 残差分量的中位,门 Tukey_C),
   --  重量 σ、重解到门外的那批残差不再变。门外的权为 0 ⇒ 谁占多数就落到谁那边,另一拨一点都不拽。
   --  只拟合转动(3 个数)不拟合单应(8 个数):单应能用一个剪切同时凑"下半幅不挪、上半幅挪 64 px"(09-30 合成的眼,手指铺满下面六成画面:
   --  Huber 拟合停在两边中间、156 个点被一个两边都不是的单应判了;换成 Tukey 还剩 179 个);一个转动凑不出来。
   --  世界点占多数时它就是世界的挪法,长在眼上的那些是野点。每个点两种说法:没挪 / 按这个转动挪 ⇒ 配到的地方离哪个近就是哪个。
   --  两种说法本身挨得不到 2·Z·σ(按近的判,判错的概率超过 Z 的单边尾巴)的点 ⇒ Unknown:眼没转、转回了原处、转轴方向附近都这样。
   --  长在眼上的点占了多数时转动拟合成"没转"⇒ 全是 Unknown,照实判不了,不猜。去不了畸变的点、G 没有焦距 ⇒ 那几个点 / 全部 Unknown。
   --  Sig_Px = 量到的配点噪声(像素,每个方向);Settled = False:门外那批换了残差条数那么多轮还在变(保险,照实报)。
   --  原来(Zone.Measure)按灰度判"没跟着变的像素":手一转光照角度就变,手指没挪也整片亮暗十几级(V1B69 2026-09-30 第 1 只手)
   type Ride is (Rides, World, Unknown);
   type Ride_Vec is array (Natural range <>) of Ride;
   --  分两步(09-30):转动只拿铺满整幅的格点拟合(Fit_Eye_Turn;Fitted = False ⇒ 点不够 / 没有焦距),再拿它判任意一批点(Classify_Rides)。
   --  判手指像素时要在手指那一块里逐像素问:那一批全挤在手指上,拿它们一起拟合,长在眼上的占了多数 ⇒ 拟合成"没转"(离线 V1B69:σ 17.9 px、一个都判不出)
   --  W × H = 画幅:每一轮按此刻的转动会转出画幅的点不进拟合、不进量 σ(它们在转出去那一帧里没有真对应)
   procedure Fit_Eye_Turn (G : Cam_Geo; W, H : Natural; Pu, Pv, Bu, Bv : Vec; Rot : out V3; Sig_Px : out Long_Float; Settled, Fitted : out Boolean)
     with Pre => Pv'Length = Pu'Length and then Bu'Length = Pu'Length and then Bv'Length = Pu'Length;
   --  W × H = 画幅:按转动算、它要是世界就会转出画幅的点 ⇒ Unknown(转出去那一帧里没有它的真对应)
   procedure Classify_Rides (G : Cam_Geo; Rot : V3; Sig_Px : Long_Float; W, H : Natural; Pu, Pv, Bu, Bv : Vec; R : out Ride_Vec)
     with Pre => Pv'Length = Pu'Length and then Bu'Length = Pu'Length and then Bv'Length = Pu'Length and then R'Length = Pu'Length;

   --  转动 + 平移 ⇒ 驱动的位姿格式 [x, y, z, qw, qx, qy, qz](四元数取 w ≥ 0 那一半)
   function To_Pose (R : M3; T : V3) return Plug.Arm_Pose;

   --  反解(V1b 3c):想让那只眼到 (Rt, Tt)(参照眼系)⇒ 从 Q_Start 起用阻尼最小二乘解关节读数。
   --  残差 = 位置差(模型单位)+ 朝向差(弧度 × 模型单位的 1,两者同一个量级:眼的位置均方根钉在 1)。
   --  Lo / Hi 空 = 不限;不空 = 每个关节的读数不出这个范围(扫描时实际到过的两头)。
   --  Pos_Err / Rot_Err = 解完还差多少(够不着的目标 = 最近能到的那一个)
   procedure IK (M : Model; Rt : M3; Tt : V3; Q_Start : Floats; Lo, Hi : Floats; Q : out Floats; Pos_Err, Rot_Err : out Long_Float);
end Kinem;
