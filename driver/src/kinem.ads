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
package Kinem is
   Max_Joints : constant := 12;   --  一组关节读数最多几个(次数)
   type Axis is record
      W : V3 := [0.0, 0.0, 1.0];   --  转轴方向(单位向量,参照眼系)
      P : V3 := [0.0, 0.0, 0.0];   --  轴上一点(参照眼系,模型单位)
   end record;
   type Axis_Array is array (0 .. Max_Joints - 1) of Axis;
   type Model is record
      Valid : Boolean := False;
      N : Natural := 0;                --  几根轴(= 这组关节读数几个)
      Ax : Axis_Array;
      F, Cx, Cy : Long_Float := 0.0;   --  手上那只眼的焦距(像素)、主点
      Q0 : Floats;                     --  参照读数
   end record;

   --  只给关节读数 ⇒ 那只眼在参照眼系里的位姿
   procedure FK (M : Model; Q : Floats; R : out M3; T : out V3);

   --  一对配点:第 I 帧的像素 (Ua, Va) 和第 J 帧的 (Ub, Vb) 是同一个真实的点
   type Corr is record
      I, J : Natural := 0;
      Ua, Va, Ub, Vb : Long_Float := 0.0;
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
      F : Long_Float := 0.0;           --  ③ 最后一起解的焦距
      Joint_Med : Floats;              --  ① 每根轴单独精修后的残差中位(像素;不能量的轴 = -1)
      Joint_Frames : Nat_Vectors.Vector;   --  ① 每根轴用了几帧(别的关节被顶偏的格子不用)
      Rho : Floats;                    --  ② 各轴离眼远近的比例(以 Ref_Joint 为 1)
      Ref_Joint : Natural := 0;
      Rho_Pairs : Natural := 0;        --  ② 用了几对(两帧加起来至少两个关节离开参照读数的)
      Rho_Start_Px, Rho_Px : Long_Float := 0.0;   --  ② 三对起步最好的那个(截断到 3 px 的均方根)、全部重解以后(中位):这些配点的 Sampson 残差(像素)
      Med_Px, P90_Px : Long_Float := 0.0;   --  ③ 最后一起解的 Sampson 残差(像素)中位 / 九成
      N_Corr, N_Used : Natural := 0;   --  配点总数 / 进最后一起解的内点数
      Flipped : Boolean := False;      --  平移整体反了一次号(Sampson 分不出,按点在不在两只眼前面定)
      Secs : Floats;                   --  各步用了几秒(墙上时间):① 网格、① 精修、② 比例、③ 一起解(两轮)
   end record;

   --  Frames(Ref) = 参照帧(扫描起点);Width = 画幅宽(像素,焦距网格按它铺:视场 30°–110°)。
   --  Ok = False:能量的轴不够 / 配点不够(Rep 里照实写到哪一步)
   procedure Fit (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                  M : out Model; Rep : out Fit_Report; Ok : out Boolean);

   --  一个配点在模型下的 Sampson 残差(像素)
   function Residual (M : Model; Frames : Frame_Vectors.Vector; C : Corr) return Long_Float;

   --  ── 两只手的系对齐到一个世界(V1b 3c)──
   type V3_Array is array (Natural range <>) of V3;
   --  多条视线交一点(最小二乘):第 I 条 = 起点 O (I) + 单位方向 D (I)
   procedure Meet_Rays (O, D : V3_Array; X : out V3; Ok : out Boolean);
   --  两团一一对应的点 ⇒ B ≈ S · R · A + T(相似变换,Horn 四元数法:4×4 对称阵最大特征值的特征向量)
   procedure Similarity (A, B : V3_Array; S : out Long_Float; R : out M3; T : out V3);
   --  抗野点(最小中位数:随机抽 3 对解一次、取残差中位数最小的那个,再拿残差 < 2.5 × 1.4826 × 中位数的那些重解 ——
   --  2.5 和 1.4826 是正态分布下中位数换标准差、2.5 倍标准差的统计常数,不是拍的门槛)
   procedure Robust_Similarity (A, B : V3_Array; S : out Long_Float; R : out M3; T : out V3; Inliers : out Natural; Med : out Long_Float);
   --  一团点里的那张面(同样的最小中位数):面上一点 P0、单位法向 Nrm
   procedure Robust_Plane (X : V3_Array; P0, Nrm : out V3; Inliers : out Natural; Med : out Long_Float);

   --  抗野点的 LM(数值雅可比):Resid 把全部残差填进 R(长度 N_R);前 N_Rob 个按 Huber(1,残差要先按自己的噪声归一)迭代加权,后面的原样。
   --  拿出来给别的包用(几只手放进一个世界,2026-09-26)
   type Vec is array (Natural range <>) of Long_Float;
   procedure Robust_LM (X : in out Vec; N_R, N_Rob : Natural; Iters : Positive; Step : Vec;
                        Resid : not null access procedure (X : Vec; R : out Vec));

   --  转动 + 平移 ⇒ 驱动的位姿格式 [x, y, z, qw, qx, qy, qz](四元数取 w ≥ 0 那一半)
   function To_Pose (R : M3; T : V3) return Plug.Arm_Pose;

   --  反解(V1b 3c):想让那只眼到 (Rt, Tt)(参照眼系)⇒ 从 Q_Start 起用阻尼最小二乘解关节读数。
   --  残差 = 位置差(模型单位)+ 朝向差(弧度 × 模型单位的 1,两者同一个量级:眼的位置均方根钉在 1)。
   --  Lo / Hi 空 = 不限;不空 = 每个关节的读数不出这个范围(扫描时实际到过的两头)。
   --  Pos_Err / Rot_Err = 解完还差多少(够不着的目标 = 最近能到的那一个)
   procedure IK (M : Model; Rt : M3; Tt : V3; Q_Start : Floats; Lo, Hi : Floats; Q : out Floats; Pos_Err, Rot_Err : out Long_Float);
end Kinem;
