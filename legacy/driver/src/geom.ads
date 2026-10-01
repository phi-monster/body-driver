--  几何:腕眼里只用【彩色图 + 手的位姿读数 + 焦距】把东西的三维位置算出来。深度通道一个字不读。
--  原理 = 大拇指测距:相机跟着手挪一段已知的米数(位姿读数说的),看东西在画面里跳了多少像素,两条视线一交就是它在哪。
--  每具身体要量一次的常数:相机装在手上的朝向(R_ce)、指尖在相机里的位置(Tip)。都由身体自己动一动量出来(指尖那一条要一次尺子/深度)。
--  相机约定与 USD 一致:-z 朝前,+y 朝上;像素 u = cx + f·x/(-z),v = cy - f·y/(-z)。
with Bytes;
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
   --  一瓣手指碰东西的那一截(相机系、世界单位,碰桌面量的;09-29 起存 —— 接触集的手按每一瓣的尖和它那一截的截面来):
   --  Tip = 这一瓣的尖;Wide / Thin = 尖那一小截在自己那只眼里看得见的两个跨度(像素 × 深 ÷ 焦距):宽的那个 = 指肚宽,窄的那个 = 看得见的厚
   --  (手指背后还有多厚不预先量:往前伸、合拢时被挡住就知道,记进来)
   type Lobe_Geo is record
      Tip : V3 := [others => 0.0];
      Wide, Thin : Long_Float := 0.0;
      --  合空时这一瓣的尖(相机系、世界单位;大并行 §2 第 4 条"合拢那一路:张到头、合空两头都量",10-01 路 5 要的):爪子合空、碰桌面量的
      --  (Zone.Hand_Zone.Shut 那一截的视线朝下压,沿那条视线解多远,Fit_On_Ray)。几瓣合空时到一起 ⇒ 量的是会合的那一点,每一瓣都是它;
      --  一边不动的夹爪 ⇒ 动的那一瓣合到不动的那一根旁边的那一点。张开时的尖(Tip)→ 它 = 这一瓣合拢那一路的方向和行程。
      --  Shut_Ok = 量过(老文件没有 ⇒ 没量,开机补碰)
      Shut : V3 := [others => 0.0];
      Shut_Ok : Boolean := False;
   end record;
   package Lobe_Geo_Vectors is new Ada.Containers.Vectors (Natural, Lobe_Geo);
   type Cam_Geo is record
      Valid : Boolean := False;        --  相机朝向量过了
      F, Cx, Cy : Long_Float := 0.0;   --  焦距(像素)、主点。焦距:身体给了就用;没给(官方 RoboDojo 观测就没有)就在量朝向时一起解出来
      --  镜头径向畸变(2026-09-26):归一化平面上畸变后的点 = 理想的点 × (1 + K1 r² + K2 r⁴)。0 = 理想针孔(仿真就是);真机的镜头都有。
      --  原来按板上铺满画面的几百个点解(Refine_Board,九月那条标定板的路;V1b 换成只读关节读数的开机后它走不到,09-30 随死代码删了 ——
      --  从那以后哪儿都不解,P8WD 2026-10-01 三台眼都加了 k1 −0.15 / k2 0.03,腕眼焦距解成 645 / 634(真 397)、头顶眼放不进世界)。
      --  10-01 起:不动的眼在 Fit_Fixed_Board 里和焦距一起解(K1_Sd / K2_Sd 带出来);腕眼在运动学多视图那一步(Kinem ④)一起解
      --  投影 / 视线全走 Project / Ray / Cam_Dir,不许在别处按针孔自己算
      K1, K2 : Long_Float := 0.0;
      K1_Sd : Long_Float := 0.0;       --  K1 的不确定度(一起解时从 JᵀJ 算出;0 = 没解;K2 的在 Fixed_Report.K2_Sd)
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
      Tip_Touch : Boolean := False;    --  指尖是碰桌面量的(2026-09-26 起的量法;旧文件里按头顶眼交的不算)
      Tip : V3 := [others => 0.0];     --  指尖中点在相机系(米)
      Gap : Long_Float := 0.0;         --  张开时两指尖间距(米)
      Lobes : Lobe_Geo_Vectors.Vector; --  每一瓣手指的尖和尖那一截的截面(碰桌面量的;空 = 没量)
      Tip_Sd : Long_Float := 0.0;      --  尖的位置误差(碰指尖几下一起解的不确定度,各瓣各维里最大的;世界单位)
      Stride : Long_Float := 0.0;      --  长着这只眼的那条臂一条命令能走多远还走得到(米;开机按阶梯探出来的最大一档,0 = 没量)
      Stride_Rot : Long_Float := 0.0;  --  同上,转:一条命令能转多远还转得到(弧度;开机按阶梯探,0 = 没量)
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
   function Cam_R (G : Cam_Geo; P : Plug.Arm_Pose) return M3;         --  相机 → 世界 = R_e · R_ce
   function Cam_Pos (G : Cam_Geo; P : Plug.Arm_Pose) return V3;       --  相机中心在世界里 = 手的位置 + R_e · Off
   --  相机系里的单位视线(去掉畸变;驱动的相机系 z 朝后 ⇒ 前方 -1)。这个像素去不了畸变 ⇒ Ok = False,返回零向量(不是一个方向):
   --  畸变后离主点比镜头模型在折回半径处能到的还远,没有哪条视线落在这儿(09-30,见 geom.adb 的 Undistort)。
   --  不带 Ok 的那一份同样返回零向量;Meet、Hit_Plane、Tips_On_Plane、Triangulate 都不拿零向量当视线
   function Cam_Dir (G : Cam_Geo; U, V : Long_Float; Ok : out Boolean) return V3;
   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float; Ok : out Boolean) return V3;   --  世界系里的单位视线(去不了畸变同 Cam_Dir)
   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3;
   procedure Cam_Pixel (G : Cam_Geo; Pc : V3; U, V : out Long_Float; In_Front : out Boolean);   --  相机系的点 → 像素(加上畸变)
   function To_Cam (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3) return V3;         --  世界点 → 相机系
   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean);
   --  ── 两种标定(Fit_Rig、Fit_Fixed_Board)共用的几样(导出给自检)──
   type Param_Vec is array (Natural range <>) of Long_Float;
   --  朝向定没定住(09-30 换掉"朝向 ± ≥ 1 弧度"):朝向差 Rot_Sd(弧度)一阶让投影挪 焦距 F × Rot_Sd 像素;挪得比半幅对角线
   --  (主点到画幅角;画幅 = 两倍主点,驱动的约定)还远 = 连它朝哪看都定不住 ⇒ True。F 和画幅都是量的:长焦的眼门自动收紧、广角的放宽
   function Pointing_Lost (G : Cam_Geo; F, Rot_Sd : Long_Float) return Boolean;
   --  正态分布单侧上尾 P(N(0,1) > X)(X > 0:Mills 比的连分式,做到不再变;X ≤ 0 按对称;纯数学)
   function Normal_Tail (X : Long_Float) return Long_Float;
   --  多出两个参数(比如镜头畸变 K1 / K2)的那个模型,比少两个的显著好吗:嵌套模型的 F 检验(d1 = 2)。
   --  Rss0 / Rss1 = 少两个 / 多两个参数的加权残差平方和(同一批点),D2 = 多的那个模型剩下的自由度(方程数 − 参数数)。
   --  门按 Stats.Z 的单侧置信度:d1 = 2 时 F 的上尾有闭式 P(F > f) = (1 + 2f / D2)^(−D2 / 2) ⇒ f_c = (D2 / 2)(α^(−2 / D2) − 1)
   function Two_More_Significant (Rss0, Rss1 : Long_Float; D2 : Natural) return Boolean;
   --  离群重挑(09-30,Fit_Rig 和 Fit_Fixed_Board 并成一套)。Reselect = 一遍:Rs = 每一笔在现在这个解下的残差(全体,先前踢掉的也算;
   --  在眼后这类算不出的 = Long_Float'Last),门 = 上一遍进解那些的残差中位 × 3(统计门),从全体重挑(先前踢错的能回来);中位是 0 ⇒ 不挑。
   --  Changed = 这一遍进解的和上一遍不一样,Kept = 这一遍进解几笔
   type Flags is array (Natural range <>) of Boolean;
   procedure Reselect (Rs : Param_Vec; Skip : in out Flags; Changed : out Boolean; Kept : out Natural)
     with Pre => Rs'First = Skip'First and then Rs'Last = Skip'Last;
   --  Reselect_Loop = 挑到不再变:Errs 按现在的解填 Rs,Solve 按 Skip 重解;每遍 Errs → Reselect,变了就 Solve 再来。
   --  Settled = 进解的那一批不再变;Broken = 进解的不到全体一半(过了中位数的崩溃点 1/2:一半以上都在门外就不是"离群",是整个解不对);
   --  Stuck = 重解的遍数到了笔数还在变(上限只当保险,碰到照实报)。Rounds = 重解了几遍
   type Reselect_End is (Settled, Broken, Stuck);
   procedure Reselect_Loop (Errs : access procedure (Rs : out Param_Vec); Solve : access procedure (Skip : Flags);
                            Skip : in out Flags; Kept, Rounds : out Natural; How : out Reselect_End);
   --  ── 不动的眼 ──:它看见我身上一个【世界位置已知】的点(指尖:手的位姿读数 + 量过的指尖偏置)落在画面哪儿
   type Mark is record
      Pw : V3 := [others => 0.0];
      U, V : Long_Float := 0.0;
   end record;
   package Mark_Vectors is new Ada.Containers.Vectors (Natural, Mark);
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float; Ok : out Boolean) return V3;   --  世界系单位视线,从 G.Pos 出发(去不了畸变同 Cam_Dir)
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float) return V3;
   procedure Project_Fixed (G : Cam_Geo; Pw : V3; U, V : out Long_Float; In_Front : out Boolean);
   --  量不动的眼:几次看见指尖在哪(世界位置 + 像素)⇒ 解它的位置和朝向。盲搜初值 + 最小二乘,和 Fit 同一套。
   procedure Fit_Fixed (G : in out Cam_Geo; O : Mark_Vectors.Vector; Ok : out Boolean);
   package V3_Vectors is new Ada.Containers.Vectors (Natural, V3);
   --  上一次 Fit_Rig / Fit_Fixed_Board 没解出来的原因(解出来时是空);开机日志原样打出来,不猜
   Why : Ada.Strings.Unbounded.Unbounded_String;
   --  上一次 Fit_Rig / Fit_Fixed_Board 挑点重解了几遍才定下来(离群重挑 / 从现位姿起步时门从粗到细;记账,同 Why)
   Refits : Natural := 0;
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
   --  不动的眼还是不是标定时那样(2026-09-25,V1:头顶眼被转了、被挡了一半 ⇒ 身体自己发现、重新标、接着干):
   --  Scene = 板上的点(世界位置已知,U/V = 它们在上一次核对时的像素),Now = 同一批点此刻在画面里配到的像素(同序;负 = 没配到)。
   --  按板再解一次它的位姿(焦距不动:转一下、挡一下都不改焦距):一份从原来的位姿起步(Start_Here)、一份从零盲搜,对得上的点多的那份算数
   --  (位姿 = 最多的点同意的那一个)。挪没挪按点数判:拿现在的位姿去投,离此刻配到的像素在"这一次核对自己的配点噪声"3 倍内(倍数无量纲)的点数,
   --  不到新解对得上的一半(比例)= 现在的位姿已经解释不了这只眼看见的东西 ⇒ Moved,G 换成新解。噪声 = 新解的像素残差和这只眼标定时的残差里大的那个:
   --  转过、挡过的画面配得比标定时粗,按标定时的噪声判会把配点的抖动当成挪动(X5B 2026-09-25:挡住左半边后每轮报"挪了 1°、2 cm");
   --  按位姿自报的不确定度判则相反,配得几乎完美时它小得离谱(G2G:每轮报"挪了 0.0°",1586 次)。
   --  Best = 这只眼这一次放好以来看见过的最多对得上的点数(调用方存着,挪过就重来):此刻对得上的比它少了四分之一以上(比例)= 被挡住了一大块或看不见了 ⇒ Covered
   --  挡没挡按"放好以来"比(09-27 V1):整幅对得上最多的那次(All_N),和画面每一块(左 / 右 / 上 / 下半、四个四分之一)各自对得上最多的那次(Region;
   --  按点在此刻画面里该落在哪分)。每一块的只在这一次开机里记(位姿换了,点就分到别的块了),挪过就重记
   N_Regions : constant := 8;   --  块数(次数)
   type Region_Counts is array (0 .. N_Regions - 1) of Natural;
   type Fixed_Best is record
      All_N : Natural := 0;
      Region : Region_Counts := [others => 0];
   end record;
   Min_Pts : constant := 10;    --  一块里放好以来至少看见过 10 个点才判得了它(次数;同对齐 / 装回核对的"至少 10 个内点")
   --  配点仪器配过去、再从配到的地方配回来,离问的那一点 1 px 以内才算真配上(像素,协议;扫描、对齐、核对不动的眼都按它)。
   --  09-27 V1B41:核对不动的眼原来只配单程,镜头挡住的那半边仪器顺着看得见的半边"编"出一片平滑的配点(纯转动时编得很准,0.96 px 的细门都过得去),
   --  往返:编出来的那半边只有 8% 在 1 px 内(往返中位 3.75 px),看得见的 87%
   Trip_Px : constant := 1.0;
   function Round_Trip_Ok (Qu, Qv, Bu, Bv : Long_Float) return Boolean;   --  问的点 (Qu, Qv),配回来落在 (Bu, Bv)(< 0 = 配不回来)
   --  这只不动的眼按板配得多细:点(给的像素)按位姿投回去的像素误差,门以内的取中位 × 1/√ln2 = 1.2011(换算:二维高斯误差离原点的距离服从瑞利分布,
   --  中位 = σ√(2 ln 2)、均方根 = σ√2,两者之比正好 1/√ln2;09-30 以前写成 1.2,差 0.1%)。
   --  开机标完、每次重标完都按它定核对的细门(09-27 V1B41:开机那份原来用解的时候的均方根,几个坏点把它抬到 2.48 px、按真值只差 0.33 px,细门放到 7.4 px);
   --  门以内一个都没有 ⇒ 0
   function Board_Rms (G : Cam_Geo; Pts : Scene_Pt_Vectors.Vector; Gate : Long_Float) return Long_Float;
   function Region_Name (R : Natural) return String;
   type Fixed_Check is record
      Asked, Matched, Consistent : Natural := 0;   --  问了几个点、配到几个、和新解对得上几个
      Consistent_Now : Natural := 0;               --  和现在的位姿对得上几个
      Moved, Covered : Boolean := False;
      Dark : Integer := -1;                        --  看不见了的那一块(-1 = 没有;块号见 Region_Name)
      Dark_Now, Dark_Best : Natural := 0;          --  那一块此刻对得上几个 / 放好以来最多几个
      Turn_Deg, Move_M : Long_Float := 0.0;        --  新解离原来的:转了几度、挪了多远
      Shift_Px, Shift_Sd : Long_Float := 0.0;      --  新解把板上的点投到的地方比原来挪了多少(中位,像素 / 以每个点自己的预测噪声为单位)
      Rms : Long_Float := 0.0;                     --  新解的像素残差
      Gate : Long_Float := 0.0;                    --  数原来的位姿用的细门(像素)
   end record;
   --  Turn_Sd:仪器把这张参考图配到"它自己转了 90°"那张时的配点噪声(像素,均方根;开机标好时量一次、随板存,0 = 没量)。
   --  原来的位姿按标定时的细门数点(小挪也抓得到);新位姿按 max(细门, 转着看的噪声)数(真被转了也数得全)
   --  Base_Now ≥ 0:"原来的位姿能解释几个点"不按这一份 Now 数,按调用方给的(画面转过再配时:转过的配点天生更糙,原来的位姿按它数吃亏,
   --  要按没转的画面里数的那份比;X5E 2026-09-26 挡左半时转 90° 再配出一份只差 0.3°、6 mm 的位姿就被当成挪过)
   procedure Check_Fixed (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Now : Scene_Pt_Vectors.Vector; Best : in out Fixed_Best; Rep : out Fixed_Check;
                          Turn_Sd : Long_Float := 0.0; Base_Now : Integer := -1);
   --  碰到桌面那一刻量指尖(2026-09-26):手上那只眼里每一瓣手指尖的像素是一条视线(Views 里的 Pose = 碰到那一刻手的位姿);
   --  指尖碰在面上(过 P0、单位法向 N、面内离散 Sd_Plane 米)⇒ 视线和面的交点就是那一瓣的指尖:离眼 S(米),不确定度 Sd_Plane ÷ |视线·法向|。
   --  视线不朝着面(平行或背着)⇒ 那一瓣 Ok = False。指尖本来就是"手碰到东西的那一点";面是标定板量的(1 mm 级),不用深度、不用尺子
   type Plane_Tip is record
      S, Sd : Long_Float := 0.0;
      Pw : V3 := [others => 0.0];   --  世界里落在面上的那一点
      Ok : Boolean := False;
   end record;
   package Plane_Tip_Vectors is new Ada.Containers.Vectors (Natural, Plane_Tip);
   function Tips_On_Plane (G : Cam_Geo; Views : Board_View_Vectors.Vector; P0, N : V3; Sd_Plane : Long_Float) return Plane_Tip_Vectors.Vector;
   --  换倾角碰量指尖(2026-09-28,PLAN 开机后半段 ③)。压到被顶住、歇下来那一刻,手上最低的那一点落在桌面上(过 P0、单位法向 N 朝上):
   --  它在手上那只眼的相机系里在 x ⇒ 世界里 = t + R x(t、R = 那一刻眼的位置、相机 → 世界)⇒ (Rᵀ N)·x = N·(P0 − t),一下一条(A·x = B)。
   --  指尖不必在哪条像素视线上。别的东西先顶住(另一根手指、手掌、胳膊到头)只会让手停得更高 ⇒ 那一下 x 还在面之上:A·x > B(只错一边)。
   --  Aimed = 这一下是对准这一瓣压的(可以进解);别的瓣压的那几下,这一瓣也不能在面之下 ⇒ 只当"A·x ≥ B"核
   type Press_Eq is record
      A : V3 := [others => 0.0];
      B : Long_Float := 0.0;
      Aimed : Boolean := True;
   end record;
   package Press_Eq_Vectors is new Ada.Containers.Vectors (Natural, Press_Eq);
   function Press_Of (G : Cam_Geo; P : Plug.Arm_Pose; P0, N : V3) return Press_Eq;
   --  按压过的几下解 x:找对得上的最大的一组 —— 至少 4 下(3 个未知数 + 至少 1 条自己核)、组里每一下都拿组里别的几下解、预测它,
   --  |预测 − 它| ≤ Gate(去掉它重解的预测残差:同"两处对不对得上",差的是整个量);组外每一下(对准这一瓣没进组的、别的瓣的)
   --  都只能是停早了:A·x − B ≥ −Gate(那一刻 x 在面之上,不许在面之下)。一样大的组不止一组、解出来互相对不上
   --  (组里哪一下按另一组的解差过 Gate)⇒ 认不出哪一下是坏的(Ambiguous,调用方补压);一组都没有 ⇒ Ok = False。
   --  为什么不按组里的残差收(09-28 离线,x5 手指网格):4 下只有 1 条自己核,四个残差按同一个比例摆着(朝下那一下永远最大),
   --  斜着的一下偏 3–10 mm 时组里残差只有 1–3 mm、解却偏 6–22 mm。Sd = x 三个分量的不确定度(组里残差定的噪声 × (AᵀA)⁻¹)
   type Press_Fit is record
      X, Sd : V3 := [others => 0.0];
      Ok, Ambiguous : Boolean := False;
      Used : Nat_Vectors.Vector;     --  进解的那几下(Eqs 的下标)
      Worst : Long_Float := 0.0;     --  组里每一下被别的几下预测、差得最多的那一下差多少
      Low : Long_Float := 0.0;       --  组外最低的 A·x − B(没有组外的 = 0)
   end record;
   --  这一瓣在它那只眼里看得见的手指像素(按行 W × H;Mask 空 = 不核)。解出来的尖是手指上的一点,投回这只眼一定落在手指的剪影里:
   --  投到眼后面、画面外、或者离最近的手指像素超过 Z 倍"它投回来的不确定度"(焦距 × 解的不确定度 ÷ 它离眼多远)⇒ 这一组不收。
   --  09-30 V1B74 第 2 只手第 2 瓣:8 下里 3 下压在东西上(剪刀轴、扁勺的边,离桌面 15–22 mm),斜 17.8° 分不出"那一下停早了"和
   --  "尖横着偏 5 cm",含坏的三下那一组对得上,解出的尖离视线 1.41 单位、投到画面外 (−194, 679),存进了身体文件。
   --  尖不在画面里的手指(穿过画面,Zone.Lobe_Mask 的 Through)调用方给空的 Mask
   type Finger_View is record
      G : Cam_Geo;
      W, H : Natural := 0;
      Mask : Bytes.Bools;
   end record;
   No_View : constant Finger_View := (G => No_Geo, W => 0, H => 0, Mask => Bytes.Bool_Vectors.Empty_Vector);   --  不核(只给自检的合成方程)
   function Fit_Presses (Eqs : Press_Eq_Vectors.Vector; Gate : Long_Float; View : Finger_View) return Press_Fit;
   --  同一套碰法,那一点在眼系里的视线 D(单位)已知、只差多远(1 个未知数 λ,X = λ D;合空时手指到的那一截,像素量的):
   --  对准它的几下里找对得上的最大的一组 —— 至少 2 下(1 个未知数 + 1 条自己核)、组里每一下拿组里别的几下解的 λ 预测它,差不过 Gate;
   --  λ 在眼前面;一样大的组不止一组、解出来对组里哪一下差过 Gate ⇒ Ambiguous。别的手形压的几下(Aimed = False)不核。
   --  Sd = λ 的不确定度(组里残差定的噪声 ÷ √Σ(A·D)²)沿 D 摊到三个分量
   function Fit_On_Ray (Eqs : Press_Eq_Vectors.Vector; D : V3; Gate : Long_Float) return Press_Fit;
   --  换倾角碰每一下让手上哪一个方向朝正下(相机系,单位):这一瓣的视线 D(相机系,单位)朝方位 Azim 斜 Tilt(弧度)——
   --  方位从"眼的 x 轴扣掉沿 D 的那一截"起量、绕 D 转(手自己的方向,每只手、每具身体一样的定法)。Tilt = 0 ⇒ D 本身
   function Tilt_Dir (D : V3; Tilt, Azim : Long_Float) return V3;
   --  和 D 垂直的方向 U(相机系)按 Tilt_Dir 同一个量法的方位(弧度):从"眼的 x 轴扣掉沿 D 的那一截"起、绕 D 转。U 不垂直于 D 时先扣掉沿 D 的那一截
   function Azim_Of (D, U : V3) return Long_Float;
   --  换倾角碰斜多少(弧度):第 K 瓣的视线 D (K) 和最近的另一瓣视线夹角 β 的三分之一 —— 取法,不是量的:朝最近的另一瓣斜 θ 时,
   --  它离朝下至少 β − θ = 2θ(两根一样长的手指,它一直比压的这一瓣高;θ 到 β/2 时一样高);离线按 x5 手指网格验过
   --  (β = 54°:斜 18° 时最低点一直是这一瓣的尖;斜 25° 时最低点在刀口的两个角之间换,解差 4.8 mm)。只有一瓣 ⇒ Single(调用方给量过的那一档)
   function Tilt_Angle (D : V3_Vectors.Vector; K : Natural; Single : Long_Float) return Long_Float;
   --  解出来的尖是哪一瓣的:离哪一瓣的视线(相机系单位方向,过眼)最近(只算尖在它前方的那几条)。碰着的不是对准的那一瓣
   --  (另一根手指长得多、斜着压时一直是它先碰到 ⇒ 几下照样互相对得上,解的是它的尖)时,解离它那条视线更近。没有一条在前方 ⇒ Natural'Last
   function Ray_Owner (X : V3; D : V3_Vectors.Vector) return Natural;
   --  把世界里的方向 Fwd(单位)转到 Down(单位)的最小转动(世界轴转动向量,绕眼转);正好反向时绕一根和它垂直的轴(同 Geo_Turn)
   function Turn_To (Fwd, Down : V3) return V3;
   --  不动的眼解完之后每组观测各自的像素残差(记账、给认指尖定门槛)
   type Fixed_Report is record
      Scene_N, Scene_Used : Natural := 0;   --  标定板的点:给了几个、进解几个
      Scene_Rms : Long_Float := 0.0;        --  它们的像素残差
      Hand_N, Hand_Used : Natural := 0;     --  手上的标记:给了几笔、进解几笔
      Hand_Rms : Long_Float := 0.0;
      K2_Sd : Long_Float := 0.0;            --  解出来的 K2 的不确定度(K1 的在 Cam_Geo.K1_Sd;Cam_Geo 不加字段 —— 别路的文件里有列全了字段的聚合)
      K_Kept : Boolean := False;            --  镜头畸变进了解(F 检验显著)
   end record;
   --  不动的眼按标定板解(2026-09-25):相机在世界里的朝向 + 位置、焦距(没给就解)。板上的点世界位置已知 ⇒ 单点法(盲搜 + 精修)起步,
   --  再按每个点自己的噪声加权精修(Scene_Var:配点噪声 ⊕ 三角的不确定度投进这只眼);加权残差按 Reselect_Loop 踢到不再变(同 Fit_Rig 那一套;
   --  Broken / Stuck / 剩下不到 4 个点 ⇒ 解不出)。
   --  视场界(焦距 ≥ 半幅宽 / √3)、不确定度界(位置 ± 比板铺开的量程还大、焦距 ± 比焦距还大 = 分不开、Pointing_Lost)同手上的眼。
   --  板不到 4 个点 ⇒ 解不出
   --  Start_Here = 从 G 现在的位姿起步(不盲搜):门从粗到细 —— 第一遍画幅宽的 1/16,以后每遍 = 上一遍门内误差均方根的 3 倍,
   --  收到挑出来的那一批不再变(遍数到了点数还在变 ⇒ 解不出,照实报)——起点就在真值附近时这条门和挡住多少无关;
   --  从零盲搜时起点离得远,只能按中位的 3 倍挑(超过一半是乱点就失灵 ⇒ Broken,照实报)
   procedure Fit_Fixed_Board (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Rep : in out Fixed_Report; Ok : out Boolean; Start_Here : Boolean := False);
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
   --  视线与一个面的交点(面 = 过 P0、法向 N);视线和面平行或交在身后 ⇒ Ok = False
   function Hit_Plane (Origin, Dir, P0, N : V3; Ok : out Boolean) return V3;
   --  ── 几条视线同一时刻交在哪 ──:每条视线 = 世界系里的起点 + 单位方向,来自哪只眼都行(不动的眼、任何一只手上的眼)。
   --  两只眼同时看见 ⇒ 距离当场出来,东西动不动都一样;只有一只眼 ⇒ 交不出来(Ok = False),调用方得靠自己挪、并如实说前提是它没动。
   --  方向是零向量的(那个像素去不了畸变,没有视线,见 Cam_Dir)不算一条
   type Sight is record
      O, D : V3 := [others => 0.0];
   end record;
   package Sight_Vectors is new Ada.Containers.Vectors (Natural, Sight);
   function Meet (Rays : Sight_Vectors.Vector; Ok : out Boolean; Spread : out Long_Float) return V3;   --  Spread = 交点到各视线的最远距离(米)
   --  交点沿 U 方向有多不准(世界单位,一倍标准差):第 I 条视线的角度噪声 Sds (I)(弧度 = 那只眼量朝向时的像素残差 ÷ 焦距)
   --  到交点那么远(t_I)就是垂直于视线的位置噪声 σ_I·t_I;几条视线的最小二乘交点的协方差 = (Σ (I − d dᵀ) / (σ_I t_I)²)⁻¹。
   --  两条视线近乎平行时(头顶眼和腕眼都近乎竖直地看,交点的远近病态 —— H53 交点在桌面之下 9–28 cm)沿视线那个方向就很不准,照实交出来。
   --  Sds 的条数和视线对不上、有哪条是 0(那只眼没量过误差)、交点在某只眼背后 ⇒ 量不出,交 Long_Float'Last
   function Meet_Sd (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P, U : V3) return Long_Float;
   --  同一个协方差整个交出来(世界单位²;压之前先看底下时新看见的点带着它进挑空地,和板点一样按沿面法向的那一份判高不高出面)。
   --  量不出(同 Meet_Sd 的几种)⇒ Ok = False
   function Meet_Cov (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P : V3; Ok : out Boolean) return M3;
   procedure Save (Path : String; Gs : Geo_Vectors.Vector);
   procedure Load (Path : String; Gs : in out Geo_Vectors.Vector; N_Cams : Natural; Note : out String);
end Geom;
