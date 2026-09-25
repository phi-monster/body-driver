--  几何:腕眼里只用【彩色图 + 手的位姿读数 + 焦距】把东西的三维位置算出来。深度通道一个字不读。
--  原理 = 大拇指测距:相机跟着手挪一段已知的米数(位姿读数说的),看东西在画面里跳了多少像素,两条视线一交就是它在哪。
--  每具身体要量一次的常数:相机装在手上的朝向(R_ce)、指尖在相机里的位置(Tip)。都由身体自己动一动量出来(指尖那一条要一次尺子/深度)。
--  相机约定与 USD 一致:-z 朝前,+y 朝上;像素 u = cx + f·x/(-z),v = cy - f·y/(-z)。
with Plug;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Geom is
   type V3 is array (0 .. 2) of Long_Float;
   type M3 is array (0 .. 2, 0 .. 2) of Long_Float;
   Identity : constant M3 := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
   type Obs is record
      Pose : Plug.Arm_Pose := [others => 0.0];   --  看它那一刻手的位姿(读数)
      U, V : Long_Float := 0.0;                  --  它在画面里的像素
   end record;
   package Obs_Vectors is new Ada.Containers.Vectors (Natural, Obs);
   --  多点观测:第 Pt 个点在这一停里的像素(标定时仪器跟着的所有点,谁丢了谁缺席)
   type Obs_Pt is record
      Pt : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];
      U, V : Long_Float := 0.0;
      Seq : Natural := 0;    --  哪一帧看见的(落盘对图用)
      Kind : Natural := 0;   --  不动的眼给指尖做的标记:这一笔里它看见这只手的手指分成几瓣(和 Pt 一起定"手上哪个点");别的观测不用,0
   end record;
   package Obs_Pt_Vectors is new Ada.Containers.Vectors (Natural, Obs_Pt);
   type Cam_Geo is record
      Valid : Boolean := False;        --  相机朝向量过了
      F, Cx, Cy : Long_Float := 0.0;   --  焦距(像素)、主点。焦距:身体给了就用;没给(官方 RoboDojo 观测就没有)就在量朝向时一起解出来
      F_Meas : Long_Float := 0.0;      --  量朝向时顺带解出来的焦距(和给的那份对账用;没给时它就是 F)
      --  仪器看一张图报的焦距 ± 不确定度(像素;0 = 没有)。没给内参时联合解里当一条残差 (F - 先验) / 不确定度:
      --  基线短、焦距和距离分不开时把焦距按在仪器的范围里;基线够长时观测压过它(V1B 2026-09-24:2.6 cm 星形基线把 397 解成 992 / 59)
      F_Prior, F_Prior_Sd : Long_Float := 0.0;
      R_Ce : M3 := Identity;           --  相机 → 手(列 = 相机轴在手坐标系里)
      Off : V3 := [others => 0.0];     --  相机中心离手的位姿原点的偏移(手系,米;转手时近处的东西才分得出它,没量就是 0)
      Rms : Long_Float := 0.0;         --  量朝向时的像素残差
      F_Sd : Long_Float := 0.0;        --  焦距的不确定度(像素;解焦距时从 JᵀJ 算出,0 = 没解焦距)
      Rot_Sd : Long_Float := 0.0;      --  朝向的不确定度(弧度)
      Off_Sd : Long_Float := 0.0;      --  相机偏移的不确定度(米;手上的眼)
      Pos_Sd : Long_Float := 0.0;      --  相机位置的不确定度(米;不动的眼)
      Dropped : Natural := 0;          --  量朝向时被判离群踢掉的观测笔数(记账)
      Tip_Valid : Boolean := False;
      Tip : V3 := [others => 0.0];     --  指尖中点在相机系(米)
      Gap : Long_Float := 0.0;         --  张开时两指尖间距(米)
      Stride : Long_Float := 0.0;      --  长着这只眼的那条臂一条命令能走多远还走得到(米;开机按阶梯探出来的最大一档,0 = 没量)
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
   function Cam_Pos (G : Cam_Geo; P : Plug.Arm_Pose) return V3;       --  相机中心在世界里 = 手的位置 + R_e · Off
   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3;   --  世界系里的单位视线
   --  几条视线的最小二乘交点(相机原点 = 手的位置;只走平移时相机在手上的偏移对结果没影响)
   function Triangulate (G : Cam_Geo; O : Obs_Vectors.Vector) return V3;
   function To_Cam (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3) return V3;         --  世界点 → 相机系
   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean);
   --  量相机朝向:手做几次【平移】,同一个不动的东西在画面里的像素 ⇒ 解朝向 + 那东西的位置(+ 焦距,当 G.F 没给时)。盲搜初值 + 最小二乘。
   procedure Fit (G : in out Cam_Geo; O : Obs_Vectors.Vector; Ok : out Boolean);
   --  手上的眼,多点一起解(2026-09-24):朝向 R_Ce、相机偏移 Off、焦距(没给就一起解)、每个点的世界位置。
   --  横着挪只给 焦距/远近 的比;转动的停让焦距和远近分开;转动下近处的点让 Off 分得出来。观测不足 4 停的点不进;Used = 进了几个点
   procedure Fit_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; N_Pts : Natural; Ok : out Boolean; Used : out Natural);
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
   package V3_Vectors is new Ada.Containers.Vectors (Natural, V3);
   --  上一次 Fit_Rig / Fit_Fixed_Rig 没解出来的原因(解出来时是空);开机日志原样打出来,不猜
   Why : Ada.Strings.Unbounded.Unbounded_String;
   --  不动的眼给手上的一个点做的标记解出来的那个点(手系,米):哪条臂、它看见几瓣手指时的那个点、用了几笔、这些笔的像素残差
   type Tip_Class is record
      Arm, Kind : Natural := 0;
      Tip : V3 := [others => 0.0];
      N : Natural := 0;
      Rms : Long_Float := 0.0;
      On_Ray : Boolean := False;   --  这个点按定义在这条臂腕眼的视线上(瓣数同它自己眼里的),只解离眼多远
   end record;
   package Tip_Class_Vectors is new Ada.Containers.Vectors (Natural, Tip_Class);
   package Nat_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   --  不动的眼,连手上被它标记的点一起解(2026-09-24):相机在世界里的朝向 + 位置、焦距(没给就解)、每个被标记的点在手系里的位置。
   --  点的身份 = (Pt = 臂号, Kind = 这一笔里它看见这只手的手指分成几瓣):瓣数不同是手上不同的点(G1S 2026-09-24:同一只手一瓣、两瓣的标记当一个点解,
   --  残差 7.5 px;分开解 1.7 px)。一个点至少 4 笔才进(次数)。
   --  瓣数和这条臂自己那只眼里一样的点(Own_Kind (k)),就是腕眼认的那个指尖,按定义落在腕眼那条视线上(手系里起点 Ray_O、单位方向 Ray_D)⇒ 只解离眼多远;
   --  别的点在手系里 3 个数都解。只有自由的点时"点在手上哪儿"和"相机在哪"能一起平移、分不太开(合成:自报 ± 1.7 cm);视线上的点把这个方向钉住。
   --  手几乎只平移时分不开(V1J 2026-09-24:相机差 24 cm)——不确定度比手挪过的量程还大 ⇒ 判解不出。Ray_D 为零向量的臂没有视线,它的点都自由。
   --  先把所有点当在 Ray_O(腕眼离手腕原点;没有就 0),用单点法定一个相机的起点;再按这个相机把每个点在手上三角出来;最后全部一起精修。Tips = 笔数够的每个点
   procedure Fit_Fixed_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; Ray_O, Ray_D : V3_Vectors.Vector; Own_Kind : Nat_Vectors.Vector;
                            Tips : out Tip_Class_Vectors.Vector; Ok : out Boolean);
   --  手上的点按"落不落在它自己那只眼的某条瓣视线上"来认(2026-09-24):不动的眼已经解好(Fixed);O = 它每一笔里每一瓣手指的尖(Pose = 那一停手的位姿);
   --  这条臂自己那只眼里每一瓣的尖在手系里是一条视线(起点 Ray_O,单位方向 Ray_D (k))。每个尖和每条视线:两条空间直线求最近点,视线上那个最近点投回不动的眼,
   --  离这个尖不到 Gate_Px 像素、又在眼前面的,归最近的那条视线,给出离眼多远 S;每条视线取归给它的 S 的中位数。
   --  (G2C 2026-09-24:手 1 的 18 笔,四根手指那一瓣的尖离腕眼指尖视线中位 5.5 px、大拇指那一瓣 25 px、两瓣中点 15 px ⇒ 瓣数不是点的身份,视线才是)
   type Ray_Tip is record
      S : Long_Float := 0.0;        --  离眼多远(米)
      Spread : Long_Float := 0.0;   --  归给它的 S 的中位绝对偏差(米)
      N : Natural := 0;             --  归给它几个尖
   end record;
   package Ray_Tip_Vectors is new Ada.Containers.Vectors (Natural, Ray_Tip);
   function Tips_On_Rays (Fixed : Cam_Geo; O : Obs_Pt_Vectors.Vector; Ray_O : V3; Ray_D : V3_Vectors.Vector; Gate_Px : Long_Float) return Ray_Tip_Vectors.Vector;
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
