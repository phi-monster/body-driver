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
   function Solve3 (A : M3; B : V3) return V3;                       --  3×3 线性方程组(列主元;奇异 ⇒ 零向量)
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
   --  ── 桌面当标定板(2026-09-25)──:手上那只眼已经解好,它在几停里看同一个桌上的点(仪器配点),每停手的位姿读数已知
   --  ⇒ 三角出这个点在世界里的位置和不确定度;同一批停里不动的眼也各拍一张,每停的图和它自己那一刻不动的眼配一次 ⇒ 这个点在不动的眼里的像素。
   --  只靠看手定不动的眼,焦距在 ±8% 里翻(分割出来的指尖不是手上一个固定的点,G2E 2026-09-24);桌上的点是真点
   type Board_Obs is record
      Pt : Natural := 0;                        --  哪一个点(参考停里的第几个查询点)
      Pose : Plug.Arm_Pose := [others => 0.0];  --  这一停手的位姿读数
      U, V : Long_Float := 0.0;                 --  它在手上那只眼里的像素
      Hu, Hv : Long_Float := -1.0;              --  这一停不动的眼里它在哪(负 = 没配到)
   end record;
   package Board_Obs_Vectors is new Ada.Containers.Vectors (Natural, Board_Obs);
   type Scene_Pt is record
      Pw : V3 := [others => 0.0];                  --  世界位置(米)
      Cov : M3 := [others => [others => 0.0]];     --  它的协方差(米²):手上那只眼的配点噪声经三角传过来
      U, V : Long_Float := 0.0;                    --  不动的眼里的像素(各停配到的中位)
      Sh : Long_Float := 0.0;                      --  不动的眼里配点的噪声(像素,每轴;同一批点量出来的)
      Views : Natural := 0;                        --  几停三角的
   end record;
   package Scene_Pt_Vectors is new Ada.Containers.Vectors (Natural, Scene_Pt);
   type Board_Stats is record
      Tracks : Natural := 0;       --  查了几个点
      Tri_Ok : Natural := 0;       --  三角成了(至少两停、各停重投都在门内、远近定得住)
      Kept : Natural := 0;         --  再加上各停在不动的眼里配到同一处 ⇒ 进标定板
      Sigma_W : Long_Float := 0.0; --  手上那只眼的配点噪声(像素,每轴)
      Sigma_H : Long_Float := 0.0; --  不动的眼里各停配到的离散(像素,每轴)
   end record;
   --  一只手上的眼、同一集里的几停(O = 每个点在每一停里的观测)⇒ 进标定板的点追加到 Scene。门槛全从这批点自己量:
   --  重投超过全体中位 3 倍的那一停不要;远近的不确定度比远近本身还大的点不要(同"不确定度比量本身还大 = 分不开");
   --  各停在不动的眼里配到的像素离它们的中位,超过全体这个离散的中位 3 倍的点不要(倍数无量纲,同踢离群那一条)
   --  一条进了板的点:它在那只腕眼里门内的几停(位姿 + 像素)和它在不动的眼里的像素。腕眼几何一变,就按这几停重新三角(Refine_Board / Board_Points)
   type Board_View is record
      Pose : Plug.Arm_Pose := [others => 0.0];
      U, V : Long_Float := 0.0;
   end record;
   package Board_View_Vectors is new Ada.Containers.Vectors (Natural, Board_View);
   type Board_Track is record
      Cam : Natural := 0;                  --  哪台腕眼(相机号)
      Views : Board_View_Vectors.Vector;
      Hu, Hv : Long_Float := 0.0;          --  不动的眼里的像素(各停配到的中位)
      Sw, Sh : Long_Float := 0.0;          --  这台腕眼、不动的眼的配点噪声(像素,每轴)
   end record;
   package Board_Track_Vectors is new Ada.Containers.Vectors (Natural, Board_Track);
   --  Cam = 这台腕眼的相机号;进板的点同时追加到 Scene(按 G 三角好的世界点)和 Tracks(几停原样,留着一起解时重新三角)
   procedure Build_Board (G : Cam_Geo; Cam : Natural; O : Board_Obs_Vectors.Vector; Scene : in out Scene_Pt_Vectors.Vector;
                          Tracks : in out Board_Track_Vectors.Vector; St : out Board_Stats);
   --  按给定的腕眼几何(Geos,按相机号)把 Tracks 重新三角成板上的点(协方差同 Build_Board)
   procedure Board_Points (Geos : Geo_Vectors.Vector; Tracks : Board_Track_Vectors.Vector; Scene : out Scene_Pt_Vectors.Vector);
   --  腕眼 + 不动的眼 + 板上的点一起解(2026-09-25):参数 = 每台腕眼(朝向改正 3、偏移改正 3、焦距 1,焦距是身体给的就不动)+ 不动的眼(朝向 3、位置 3、焦距 1);
   --  板上的点不是未知数:每换一次参数,按它在腕眼里的几停重新三角。残差 = 腕眼各停的重投 ÷ 腕眼配点噪声 + 不动的眼里的像素 ÷ 它的配点噪声
   --  + 腕眼标定(Fit_Rig)量到的偏移、焦距当先验(÷ 它们自己报的不确定度;没报就不加)。
   --  为什么要一起解:板上的点的远近随腕眼焦距缩放,不动的眼的焦距跟着错(G2E 离线:腕眼 −1.2% ⇒ 头 −2.8%;把腕眼焦距换成真值 ⇒ 头 +0.2%);
   --  而不动的眼从另一个方向看同一批点,点的形状对不上就把腕眼焦距拉回来(离线一起解:腕眼 392.2 → 396.9、头 281.7 → 288.4,真 397 / 288.1)。
   --  只平移的停定不住偏移(整块板跟着平移,不动的眼一起挪),转动的停定得住 ⇒ 板里要有转动停。不动的眼的点每遍按 3 倍中位重挑,两遍(次数)。
   --  不确定度比量本身还大(焦距 ± 比焦距大、位置 ± 比板铺开的量程大)⇒ 不改几何,Ok = False
   type Refine_Report is record
      Tracks, Head_Used : Natural := 0;
      Wrist_Rms, Head_Rms : Long_Float := 0.0;   --  像素
   end record;
   procedure Refine_Board (Geos : in out Geo_Vectors.Vector; Head : in out Cam_Geo; Tracks : Board_Track_Vectors.Vector; Rep : out Refine_Report; Ok : out Boolean);
   --  不动的眼还是不是标定时那样(2026-09-25,V1:头顶眼被转了、被挡了一半 ⇒ 身体自己发现、重新标、接着干):
   --  Scene = 板上的点(世界位置已知,U/V = 它们在上一次核对时的像素),Now = 同一批点此刻在画面里配到的像素(同序;负 = 没配到)。
   --  按板再解一次它的位姿(焦距不动:转一下、挡一下都不改焦距):一份从原来的位姿起步(Start_Here)、一份从零盲搜,对得上的点多的那份算数
   --  (位姿 = 最多的点同意的那一个)。挪没挪按点数判:拿现在的位姿去投,离此刻配到的像素在"这一次核对自己的配点噪声"3 倍内(倍数无量纲)的点数,
   --  不到新解对得上的一半(比例)= 现在的位姿已经解释不了这只眼看见的东西 ⇒ Moved,G 换成新解。噪声 = 新解的像素残差和这只眼标定时的残差里大的那个:
   --  转过、挡过的画面配得比标定时粗,按标定时的噪声判会把配点的抖动当成挪动(X5B 2026-09-25:挡住左半边后每轮报"挪了 1°、2 cm");
   --  按位姿自报的不确定度判则相反,配得几乎完美时它小得离谱(G2G:每轮报"挪了 0.0°",1586 次)。
   --  Best = 这只眼这一次放好以来看见过的最多对得上的点数(调用方存着,挪过就重来):此刻对得上的比它少了四分之一以上(比例)= 被挡住了一大块或看不见了 ⇒ Covered
   type Fixed_Check is record
      Asked, Matched, Consistent : Natural := 0;   --  问了几个点、配到几个、和新解对得上几个
      Consistent_Now : Natural := 0;               --  和现在的位姿对得上几个
      Moved, Covered : Boolean := False;
      Turn_Deg, Move_M : Long_Float := 0.0;        --  新解离原来的:转了几度、挪了多远
      Shift_Px, Shift_Sd : Long_Float := 0.0;      --  新解把板上的点投到的地方比原来挪了多少(中位,像素 / 以每个点自己的预测噪声为单位)
      Rms : Long_Float := 0.0;                     --  新解的像素残差
   end record;
   procedure Check_Fixed (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Now : Scene_Pt_Vectors.Vector; Best : in out Natural; Rep : out Fixed_Check);
   --  不动的眼解完之后每组观测各自的像素残差(记账、给认指尖定门槛)
   type Fixed_Report is record
      Scene_N, Scene_Used : Natural := 0;   --  标定板的点:给了几个、进解几个
      Scene_Rms : Long_Float := 0.0;        --  它们的像素残差
      Hand_N, Hand_Used : Natural := 0;     --  手上的标记:给了几笔、进解几笔
      Hand_Rms : Long_Float := 0.0;
   end record;
   --  不动的眼按标定板解(2026-09-25):相机在世界里的朝向 + 位置、焦距(没给就解)。板上的点世界位置已知 ⇒ 单点法(盲搜 + 精修)起步,
   --  再按每个点自己的噪声加权精修(Scene_Var:配点噪声 ⊕ 三角的不确定度投进这只眼);加权残差超过中位 3 倍的踢掉再解(倍数无量纲)。
   --  视场界(焦距 ≥ 半幅宽 / √3)、不确定度界(位置 ± 比板铺开的量程还大、焦距 ± 比焦距还大 = 分不开)同手上的眼。板不到 4 个点 ⇒ 解不出
   --  Start_Here = 从 G 现在的位姿起步(不盲搜):挑点按每个点自己的预测噪声的 3 倍(倍数无量纲)——起点就在真值附近时这条门和挡住多少无关;
   --  从零盲搜时起点离得远,只能按全体残差中位的 3 倍挑(超过一半是乱点就失灵)
   procedure Fit_Fixed_Board (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Rep : in out Fixed_Report; Ok : out Boolean; Start_Here : Boolean := False);
   --  不动的眼已知 ⇒ 手上被它标记的点在手系里在哪。点的身份 = (Pt = 臂号, Kind = 这一笔里它看见这只手的手指分成几瓣)
   --  (G1S 2026-09-24:同一只手一瓣、两瓣的标记当一个点解,残差 7.5 px;分开解 1.7 px)。一个点至少 4 笔(次数)。
   --  瓣数和这条臂自己那只眼里一样的点(Own_Kind (k))按定义在腕眼那条视线上(手系起点 Ray_O、单位方向 Ray_D)⇒ 只解离眼多远;别的点 3 个数都解。
   --  每笔:点在世界里 = p_j + R_j t 必须落在不动的眼过 (u_j, v_j) 的视线上 ⇒ 对 t 线性,最小二乘;像素残差超过这个点中位 3 倍的那笔踢掉再解一次。
   --  解到离手腕原点比手腕离这只眼还远的点不在手上 ⇒ 不要。Rep 的手那一半 = 进解的笔数和它们的像素残差
   procedure Hand_Points (G : Cam_Geo; O : Obs_Pt_Vectors.Vector; Ray_O, Ray_D : V3_Vectors.Vector; Own_Kind : Nat_Vectors.Vector;
                          Tips : out Tip_Class_Vectors.Vector; Rep : in out Fixed_Report);
   --  开机用的两步:先按板解不动的眼,再按解好的眼认手上的点。手上的点不进眼的解 —— 分割出来的指尖在手换角度、换远近时会在手上滑,
   --  而每个点在手系里的位置是自由的,滑出来的偏差被点的位置吃掉、残差看着很小,却顺着"焦距 ↔ 远近"把相机拽走
   --  (G2E 2026-09-24:只靠它们,焦距随放进哪几笔在 ±8% 里翻;合成:板 + 放大 2% 的手上标记一起解,焦距 275.5,比板单独 289.5、手单独 287.0 都偏)
   procedure Fit_Fixed_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; Scene : Scene_Pt_Vectors.Vector; Ray_O, Ray_D : V3_Vectors.Vector;
                            Own_Kind : Nat_Vectors.Vector; Tips : out Tip_Class_Vectors.Vector; Rep : out Fixed_Report; Ok : out Boolean);
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
