--  运动学(V1b 第三步,2026-09-26):一只手 = 一串转轴,只凭【关节读数】+【手上那只眼的画面】量出来 ——
--  不要图纸、不要零点、不要身体报的"手在哪"。开机关节扫描(Act.Geo_Boot_Sweep)每个关节单独一格一格转,
--  每一格的转角 = 关节读数的差(已知),画面里桌面挪了多少由配点仪器给(Instrument.Match)。
--  每根轴:方向 W(单位向量)、过哪一点 P,都在"参照读数 Q0 那一刻手上那只眼"的相机系里(同 Geom:-z 朝前、+y 朝上)。
--  T(q) = exp([S_0](q_0 − Q0_0)) · … · exp([S_n−1](q_n−1 − Q0_n−1)) = 那只眼在参照眼系里的位姿(X_参照 = R · X_眼 + t)。
--  只靠相机和转角量不出米:长度只差一个倍数,解的时候钉"参与拟合的各帧眼的位置均方根 = 1"(模型单位)。
--  量法只有一种(owner 2026-09-26):① 每根轴单独(已知转角绕一根轴转:方向铺网格,"轴在眼哪边"直接解,焦距各轴共用一起定);
--  ② 各轴离眼远近的比例(不同轴的格子之间的配点);③ 全部配点按像素一起解(焦距放开)。离线同一套在 x5 仿真上:焦距 396.5 / 396.9(真 397),
--  只给关节读数算手的位置,离标定点 40° 以上最大 2.47 mm(LAB 09-26 V1B2;离线脚本 harness/v1b/sweep_init.py + v1b_sweep.py)
with Geom; use Geom;
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
end Kinem;
