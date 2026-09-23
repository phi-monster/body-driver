--  几何:腕眼里只用【彩色图 + 手的位姿读数 + 焦距】把东西的三维位置算出来。深度通道一个字不读。
--  原理 = 大拇指测距:相机跟着手挪一段已知的米数(位姿读数说的),看东西在画面里跳了多少像素,两条视线一交就是它在哪。
--  每具身体要量一次的常数:相机装在手上的朝向(R_ce)、指尖在相机里的位置(Tip)。都由身体自己动一动量出来(指尖那一条要一次尺子/深度)。
--  相机约定与 USD 一致:-z 朝前,+y 朝上;像素 u = cx + f·x/(-z),v = cy - f·y/(-z)。
with Plug;
with Ada.Containers.Vectors;
package Geom is
   type V3 is array (0 .. 2) of Long_Float;
   type M3 is array (0 .. 2, 0 .. 2) of Long_Float;
   Identity : constant M3 := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
   type Obs is record
      Pose : Plug.Arm_Pose := [others => 0.0];   --  看它那一刻手的位姿(读数)
      U, V : Long_Float := 0.0;                  --  它在画面里的像素
   end record;
   package Obs_Vectors is new Ada.Containers.Vectors (Natural, Obs);
   type Cam_Geo is record
      Valid : Boolean := False;        --  相机朝向量过了
      F, Cx, Cy : Long_Float := 0.0;   --  焦距(像素)、主点
      R_Ce : M3 := Identity;           --  相机 → 手(列 = 相机轴在手坐标系里)
      Rms : Long_Float := 0.0;         --  量朝向时的像素残差
      Tip_Valid : Boolean := False;
      Tip : V3 := [others => 0.0];     --  指尖中点在相机系(米)
      Gap : Long_Float := 0.0;         --  张开时两指尖间距(米)
      --  不长在任何胳膊上的眼(头顶眼):它在世界里的位置和朝向,由身体看着【自己的手】挪出来(Fit_Fixed)。
      --  Fixed = True 时 R_Ce 就是 相机 → 世界,Pos 是相机在世界里的位置(米)。
      Fixed : Boolean := False;
      Pos : V3 := [others => 0.0];
   end record;
   No_Geo : constant Cam_Geo := (others => <>);
   package Geo_Vectors is new Ada.Containers.Vectors (Natural, Cam_Geo);

   function Quat_To_R (P : Plug.Arm_Pose) return M3;           --  手 → 世界
   function Mul (A, B : M3) return M3;
   function Tr (A : M3) return M3;
   function Ap (A : M3; X : V3) return V3;
   function Rodrigues (R : V3) return M3;
   function Rot_Vec (A : M3) return V3;
   function Norm (X : V3) return Long_Float;
   function Angle_Between (P, Q : Plug.Arm_Pose) return Long_Float;   --  两个位姿的姿态差(弧度)
   function Cam_R (G : Cam_Geo; P : Plug.Arm_Pose) return M3;         --  相机 → 世界 = R_e · R_ce
   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3;   --  世界系里的单位视线
   --  几条视线的最小二乘交点(相机原点 = 手的位置;只走平移时相机在手上的偏移对结果没影响)
   function Triangulate (G : Cam_Geo; O : Obs_Vectors.Vector) return V3;
   function To_Cam (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3) return V3;         --  世界点 → 相机系
   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean);
   --  量相机朝向:手做几次【平移】,同一个不动的东西在画面里的像素 ⇒ 解朝向 + 那东西的位置。盲搜初值 + 最小二乘。
   procedure Fit (G : in out Cam_Geo; O : Obs_Vectors.Vector; Ok : out Boolean);
   --  ── 不动的眼 ──:它看见我身上一个【世界位置已知】的点(指尖:手的位姿读数 + 量过的指尖偏置)落在画面哪儿
   type Mark is record
      Pw : V3 := [others => 0.0];
      U, V : Long_Float := 0.0;
   end record;
   package Mark_Vectors is new Ada.Containers.Vectors (Natural, Mark);
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float) return V3;        --  世界系单位视线,从 G.Pos 出发
   procedure Project_Fixed (G : Cam_Geo; Pw : V3; U, V : out Long_Float; In_Front : out Boolean);
   --  量不动的眼:几次看见指尖在哪(世界位置 + 像素)⇒ 解它的位置和朝向。盲搜初值 + 最小二乘,和 Fit 同一套。
   procedure Fit_Fixed (G : in out Cam_Geo; O : Mark_Vectors.Vector; Ok : out Boolean);
   --  ── 没有深度时量指尖 ──:指尖在这只手自己眼里的像素给出相机系里的一条视线 Dir_C(单位向量,从手的位姿点出发);指尖 = S · Dir_C,只差 S(米)。
   --  不动的眼在几停里看见这只手的指尖落在 (U,V)(O 里的 Pose = 那一停手的位姿读数):指尖的世界位置必须落在不动眼那条视线上
   --  ⇒ 每停两条线性方程、一个未知数 S,最小二乘。两条视线平行(解不出)或一停都没有 ⇒ Ok = False。Rms_Px = 解出来之后指尖投回不动眼的像素残差。
   procedure Fit_Tip_Scale (Fixed, Hand : Cam_Geo; Dir_C : V3; O : Obs_Vectors.Vector; S, Rms_Px : out Long_Float; Ok : out Boolean);
   --  视线与一个面的交点(面 = 过 P0、法向 N);视线和面平行或交在身后 ⇒ Ok = False
   function Hit_Plane (Origin, Dir, P0, N : V3; Ok : out Boolean) return V3;
   --  ── 几条视线同一时刻交在哪 ──:每条视线 = 世界系里的起点 + 单位方向,来自哪只眼都行(不动的眼、任何一只手上的眼)。
   --  两只眼同时看见 ⇒ 距离当场出来,东西动不动都一样;只有一只眼 ⇒ 交不出来(Ok = False),调用方得靠自己挪、并如实说前提是它没动。
   type Sight is record
      O, D : V3 := [others => 0.0];
   end record;
   package Sight_Vectors is new Ada.Containers.Vectors (Natural, Sight);
   function Meet (Rays : Sight_Vectors.Vector; Ok : out Boolean; Spread : out Long_Float) return V3;   --  Spread = 交点到各视线的最远距离(米)
   procedure Save (Path : String; Gs : Geo_Vectors.Vector);
   procedure Load (Path : String; Gs : in out Geo_Vectors.Vector; N_Cams : Natural; Note : out String);
end Geom;
