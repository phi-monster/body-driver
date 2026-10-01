--  拿着的东西 = 身体的一部分(大并行.md §2 第 20 条,路 6;I7 里路 6 填的那一份)。
--  ① 拿住以后认哪些点跟着手走(Take):这件东西被跟住的每个点、每一帧在世界里在哪(+ 协方差),手每一帧在世界里在哪(按关节读数算的位姿 +
--     它的不准)。每个点两种说法 —— 长在手上(在手的系里一直待在同一处 s:每一帧在 R s + T)/ 留在世界里(一直在世界的同一处 w);
--     两种各按最大似然解出 s / w,白化残差平方和各过不过门。手的位姿不准按这一点离手多远(杠杆)算进"长在手上"那一说的协方差。
--     两种说法对它的预测隔开 2 Z 倍才判(手动得够多):近的那一种过门 ⇒ 归它;隔不开 ⇒ Unknown(手没动、或动了但在这一点上看不出);
--     两种都不过门 ⇒ Neither(它自己在动:在手里转着、滑着,或者是跟错了的点)。—— 和开机认"长在眼上"的像素同一个判法
--     (Kinem.Classify_Rides:每个点两种说法,隔得开就按近的判,隔不开就照实说判不了),这里是三维的、带着手的不准。
--  ② 长在手上的那些点在手的系里的位置 + 不准 = 它在手上的形状(Part):进"身体能碰东西的地方"那张单子,挂在这只手上,
--     此刻在哪 = 手此刻的位姿接上形状(In_World,不准 = 形状的不准 + 手此刻位姿的不准)。松手:调用方把它从单子上拿掉。
--  ③ 每看一眼查滑没滑(Check_Slip):这一眼看见的那几个点换到手的系里,按一个刚体的小挪动 δ(转 3 + 移 3)拟合到存下的形状上;
--     手此刻位姿的不准在手的系里看,正好也是这样一个 δ(所有点一起挪)⇒ 拟合出来的 δ 和"手不准凑得出来的"比(马氏距离,
--     只比看见的点定得住的那几个方向),过门 = 滑了。一个个点比的话:整件东西一起挪了一点点,每个点都在门里,成团地看明明挪了。
--     看见的点一个方向都定不住(一个都没看见)⇒ 照实说判不了。
--  噪声倍数:这件东西的点量到的(Linkage.Fit 的 Sigma / Sigma_Dof,或者路 3 的估计;没量 ⇒ 1、Long_Float'Last = 信给的)。
--  不认单位和尺度、不认手指个数;代码里没有东西的名字、没有动作的名字
with Geom;
with Linkage;
with Ada.Containers.Vectors;
package Held is
   subtype V3 is Geom.V3;
   subtype M3 is Geom.M3;
   --  位姿的不准:6 × 6,左乘的小转动 3 + 平移 3(同 Linkage 里一块的位姿)
   type Cov6 is array (0 .. 5, 0 .. 5) of Long_Float;
   Zero6 : constant Cov6 := [others => [others => 0.0]];
   --  手在一帧里:手的系里的点 y ⇒ 世界里 R y + T;Cov = 这个位姿的不准(按关节读数算的,运动学量过多准就填多准;0 = 当它准);
   --  Ok = False:这一帧手在哪不知道
   type Hand_Pose is record
      Ok : Boolean := False;
      R : M3 := Geom.Identity;
      T : V3 := [others => 0.0];
      Cov : Cov6 := Zero6;
   end record;
   package Hand_Vectors is new Ada.Containers.Vectors (Natural, Hand_Pose);

   type Ride is (Rides, World, Unknown, Neither, Unused);   --  Unused:手的位姿知道的帧里看见不到两次,没有可比的
   package Ride_Vectors is new Ada.Containers.Vectors (Natural, Ride);
   use type Geom.M3;
   package M3_Vectors is new Ada.Containers.Vectors (Natural, M3);

   --  拿在手上的一件东西(身体的一部分)
   type Part is record
      Arm : Natural := 0;                    --  挂在哪条臂上(Selfmap.Graph 的臂号,调用方给)
      Tracks : Geom.Nat_Vectors.Vector;      --  这件东西的第几条轨迹跟着手走
      Pts : Geom.V3_Vectors.Vector;          --  它们在手的系里在哪
      Covs : M3_Vectors.Vector;              --  每一个自己的不准(手的系;眼的噪声 × 噪声倍数,一个点一份、点和点之间不相关)
      --  形状整体的不准(手的系,同 δ 的 6 个数):拿住时那几帧手的位姿不准平均下来,所有点一起挪的那一份(点和点之间相关的那一份单独记,
      --  不摊进每个点:摊进去就当成各点独立的了,好多个点一平均,整体挪的不准就被说小了)
      Common : Cov6 := Zero6;
      Sigma : Long_Float := 1.0;             --  这件东西的点的噪声倍数(Take 时给的)
      Sigma_Dof : Long_Float := Long_Float'Last;
   end record;

   type Take_Report is record
      Roles : Ride_Vectors.Vector;           --  每条轨迹
      N_Rides, N_World, N_Unknown, N_Neither, N_Unused : Natural := 0;
   end record;
   --  Hands (F) = 第 F 帧手在哪;Tracks (I) (F) = 这件东西第 I 条轨迹第 F 帧
   procedure Take (Hands : Hand_Vectors.Vector; Tracks : Linkage.Track_Vectors.Vector; Arm : Natural;
                   Sigma : Long_Float; Sigma_Dof : Long_Float; P : out Part; Rep : out Take_Report);

   --  第 K 个点在手的系里一共多不准(它自己的 + 形状整体的那一份)
   function Shape_Cov (P : Part; K : Natural) return M3;
   --  此刻在世界里:每一个点在哪、多不准(形状的不准 + 手此刻位姿的不准)
   type World_View is record
      Pts : Geom.V3_Vectors.Vector;
      Covs : M3_Vectors.Vector;
   end record;
   function In_World (P : Part; H : Hand_Pose) return World_View;

   type Slip_Report is record
      Seen : Natural := 0;                   --  这一眼看见了几个
      Dof : Natural := 0;                    --  看见的点定得住 δ 的几个方向(0 = 判不了)
      Chi, Gate : Long_Float := 0.0;         --  δ 比手的不准多出来的(马氏距离平方)和它的门
      Slipped : Boolean := False;
      Turn, Shift : V3 := [others => 0.0];   --  拟合出来的 δ(手的系):转动向量、平移
   end record;
   --  Now (K) = 第 K 个点(P.Tracks (K) 那一条)这一眼在世界里;H = 手这一眼在哪
   procedure Check_Slip (P : Part; H : Hand_Pose; Now : Linkage.Obs_Vectors.Vector; Rep : out Slip_Report);

   function Say (Rep : Take_Report) return String;
   function Say (Rep : Slip_Report) return String;
end Held;
