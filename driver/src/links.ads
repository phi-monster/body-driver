--  身体每一节的形状(大并行 §2 第 3 条、I7;路 1,10-01):开机扫描时,不动的眼看着每条臂一个关节一个关节地转 ——
--  跟着哪一节一起动的点,就是那一节的表面点(不要图纸、不要网格:都是看出来的)。
--  量法:不动的眼在扫描起点那一刻的画面上铺全仓那一张格子(Kinem.Gx × Gy),配点仪器把每个格点配进扫描的每一格(往返 1 px 以内才算)。
--  一个格点在某一格挪了(离起点超过配点噪声的 Stats.Z 倍)= 那一格扫的那个关节一转它就动;它属于"一转它就动的关节里最远的那一个"那一节(第 ℓ 节)。
--  第 ℓ 节在每一格的位姿只算到第 ℓ 个关节(更远的关节放在参照读数上)⇒ 每一格的那条视线按这一格的位姿搬回起点那一刻,
--  和起点那条视线交出这个点(Geom.Meet;协方差按配点噪声,Geom.Meet_Cov)。离每条视线都在 Stats.Z 倍噪声以内、远近定得住
--  (远近的不确定度不比远近本身大)才收。几条臂一起扫:每条臂的假设都试,交得上、离视线最近的那条认它。
--  点存在这条臂的参照系里(运动学的参照眼系、读数 = 参照读数 Q0 那一刻);此刻在哪 = 按此刻的读数算到那一节,再放进世界。
--  配点噪声从同一批配点自己量:每一格里大半格点是不动的背景,它们挪了多少的中位 ÷ 瑞利中位 = 每轴 σ。没有不动的眼 ⇒ 量不了(照实说)。
--  用它的:净空(给"走一步"的 Lim.Clear:这一步走多远以后有一点进了"可能碰到"的那条带子)、这只眼里哪些像素是自己(给认东西的那一路)
with Geom; use Geom;
with Kinem;
with Plug;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Links is
   --  一条臂怎么放进世界(同 Jointboot.Arm_World 的 S、Ra、Ta 和装上时的 Rw、O):X_世界 = Rw · (S · Ra · X + Ta − O)
   type Placement is record
      Model : Kinem.Model;
      S : Long_Float := 1.0;
      Ra : M3 := Identity;
      Ta : V3 := [0.0, 0.0, 0.0];
      Rw : M3 := Identity;
      O : V3 := [0.0, 0.0, 0.0];
      Valid : Boolean := False;
      Group : Natural := 0;    --  这条臂的关节读数是 F.Joints 的第几组(干活时按它从一帧里取读数)
      Eye : Integer := -1;     --  长在这条臂上的那只眼(相机号;-1 = 没有):这只眼此刻在哪 = 这条臂此刻的运动学
   end record;
   package Placement_Vectors is new Ada.Containers.Vectors (Natural, Placement);

   --  一节上的一个表面点:第几条臂、第几节、参照系里的位置和协方差(模型单位、模型单位²)、几条视线交出来的、
   --  量它时的采样间距(不动的眼里相邻两个格点在这个点那么远处隔多远,模型单位:这个点代表它周围这么大一片)
   type Link_Pt is record
      Arm, Link : Natural := 0;
      P : V3 := [0.0, 0.0, 0.0];
      Cov : M3 := [others => [others => 0.0]];
      Views : Natural := 0;
      Spacing : Long_Float := 0.0;
   end record;
   package Link_Pt_Vectors is new Ada.Containers.Vectors (Natural, Link_Pt);

   --  扫描的一格(不动的眼那一拍拍了一张):每条臂那一拍的读数(空 = 不知道)、每条臂那一拍在单独扫第几个关节
   --  (-1 = 没在单独扫一个关节:几个关节一起动、这一段它已经停了),Seq = 那一拍的帧号(画面比读数晚几拍,扫完按量到的拍数重配读数)
   type Cell is record
      Qs : Plug.Floats_Vectors.Vector;
      Joints : Ints;
      Seq : Natural := 0;
   end record;
   package Cell_Vectors is new Ada.Containers.Vectors (Natural, Cell);
   --  一个格点:起点那一张里的像素,每一格里配到哪(负 = 没配到;下标同 Cells)
   type Track is record
      U0, V0 : Long_Float := 0.0;
      U, V : Floats;
   end record;
   package Track_Vectors is new Ada.Containers.Vectors (Natural, Track);

   --  ① 三角(纯函数,导出给自检):Pls = 每条臂怎么放进世界(Valid = False 的那条不试),G = 不动的眼(世界系),W = 它的画幅宽
   --  (格点间距 = W / Kinem.Gx,定每个点的采样间距),Cells / Tracks 见上。挪没挪按背景的配点噪声判(Track_Noise);
   --  交得上没有按跟着身体动的点自己的噪声判:先每个格点交一次、留最好的假设,各条视线离交点多远的中位 ÷ 瑞利中位 = 每轴 σ
   --  (配的是一块在转的东西,运动学自己也有误差:比背景配得粗;背景那一份是它的下限),再按 Stats.Z 倍它收。
   --  Pts = 收下的点;Sd_Used = 定门用的那份噪声(像素)
   procedure Triangulate (Pls : Placement_Vectors.Vector; G : Cam_Geo; W : Natural; Cells : Cell_Vectors.Vector; Tracks : Track_Vectors.Vector;
                          Pts : out Link_Pt_Vectors.Vector; Sd_Used : out Long_Float);
   --  配点噪声(像素,每轴):每一格里格点挪了多少的中位 ÷ 瑞利中位(大半格点是不动的背景),各格取中位;量不出 ⇒ 0
   function Track_Noise (Tracks : Track_Vectors.Vector; N_Cells : Natural) return Long_Float;

   --  ② 开机扫描时攒下的(Jointboot.Sweep_All 填):不动的眼起点那一张在仪器那边的编号、画幅;每一格的编号和读数
   procedure Sweep_Begin (World_Id : Integer; W, H : Natural);
   function Sweep_On return Boolean;                      --  这一回扫描攒着(有不动的眼、起点那一张存成了)
   function Last_Seq return Integer;                      --  最后攒的那一格的帧号(-1 = 还没有)
   procedure Sweep_Cell (Id : Integer; C : Cell);
   procedure Sweep_Set_Joint (Arm : Natural; Joint : Integer);   --  最后那一格:第 Arm 条臂在单独扫第几个关节
   --  画面比读数晚 Lag 拍(扫描量的):每一格改配那一刻的读数(Qa = 那一刻每组关节读数;Groups (A) = 第 A 条臂是第几组)
   procedure Sweep_Repair (Seq_Of : access function (Seq : Natural) return Plug.Floats_Vectors.Vector; Lag : Integer; Groups : Ints);
   --  扫完以后:起点那一张的格点配进每一格(仪器;往返 1 px 以内才算)⇒ 一串格点。Host / Port = 配点仪器;Note = 配了几对、几个格点配上
   procedure Sweep_Match (Host : String; Port : Natural; Note : out Ada.Strings.Unbounded.Unbounded_String);
   function Sweep_Cells return Cell_Vectors.Vector;
   function Sweep_Tracks return Track_Vectors.Vector;
   --  落盘攒下的扫描(给离线重放:开机给了 BL_DUMP 才写)
   procedure Dump_Sweep (Path : String);

   --  ③ 量(开机前半段对齐以后):按攒下的扫描三角,装上;Note = 开机报告那一段(每条臂每一节几个点、不确定度中位)
   procedure Measure (Pls : Placement_Vectors.Vector; G : Cam_Geo; Note : out Ada.Strings.Unbounded.Unbounded_String);
   --  装上(从零量的 / 从前半段存的装回):每条臂怎么放进世界 + 表面点
   procedure Install (Pls : Placement_Vectors.Vector; Pts : Link_Pt_Vectors.Vector);
   procedure Set_Points (Pts : Link_Pt_Vectors.Vector);    --  只换表面点(从前半段存的读回来时;放进世界的那一份装回核对过再给)
   procedure Place (Pls : Placement_Vectors.Vector);        --  只换每条臂怎么放进世界
   function Points return Link_Pt_Vectors.Vector;   --  装上的表面点(存盘用)
   --  存盘的一行(前半段的文件里 link 开头的那一行:臂 节 位置 3 协方差 9 视线数 采样间距)和读回
   function Line_Of (P : Link_Pt) return String;
   function From_Fields (F : Bytes.Strs; First : Natural; P : out Link_Pt) return Boolean;

   --  ④ 此刻(读数 Q)第 Arm 条臂一个表面点在世界里在哪、协方差(世界单位²)
   procedure World_Of (Pl : Placement; Q : Floats; Pt : Link_Pt; X : out V3; Cov : out M3);
   --  ⑤ 净空:第 Arm 条臂此刻(Qs = 每条臂的读数)离 Scene(量过的场景点,世界系)、离别的臂的表面点最近多远 ——
   --  最近的那一对按"距离 − Stats.Z 倍这一对沿连线的不确定度"挑;Dist = 那一对的距离,Sd = 沿连线的不确定度(世界单位)
   type Clearance is record
      Dist : Long_Float := Long_Float'Last;
      Sd : Long_Float := 0.0;
      To_Scene : Boolean := True;   --  最近的是场景点(False = 别的臂)
      Other_Arm : Integer := -1;
      Link : Integer := -1;         --  这条臂的第几节
      Valid : Boolean := False;     --  这条臂量过表面点
   end record;
   function Clear_Of (Arm : Natural; Qs : Plug.Floats_Vectors.Vector; Scene : Scene_Pt_Vectors.Vector) return Clearance;
   --  第 Arm 条臂整条沿世界里的方向 Dir(单位向量)平移,走多远以后有一个表面点进了"可能碰到"的那条带子
   --  (离一个场景点 / 别的臂的表面点不到 Stats.Z 倍这一对的不确定度)。已经在带子里 ⇒ 0;哪个都碰不上 ⇒ Long_Float'Last;
   --  这条臂没量过表面点 ⇒ Long_Float'Last 且 Known = False(说不出,不当成"不会碰")
   function Free_Along (Arm : Natural; Qs : Plug.Floats_Vectors.Vector; Dir : V3; Scene : Scene_Pt_Vectors.Vector; Known : out Boolean) return Long_Float;
   --  ⑥ 自己:一只眼(几何 Geo、此刻的位姿 Pose;不动的眼 Pose 不用)W × H 的画面里,此刻身体的表面点落在哪些像素:
   --  每个点按它在不动的眼里格点的间距(量的时候的采样密度)在这只眼里的大小画一个圆
   function Self_Mask (Geo : Cam_Geo; Pose : Plug.Arm_Pose; W, H : Natural; Qs : Plug.Floats_Vectors.Vector) return Bools;
   --  干活时一帧里直接问(10-01 路 7 要的:Round 里只有 F.Joints,它的下标是布局的,不是臂的):
   --  Readings_Now = 装上的每条臂此刻的关节读数(按装上时记下的读数组从 F.Joints 取;这一拍没有那组 = 空);
   --  Self_Mask_Now = 第 Cam 台眼(几何 Geo)此刻 W × H 的画面里哪些像素是身体自己 —— 不动的眼按它自己的几何,
   --  长在臂上的眼按那条臂此刻的运动学放;哪条臂都不带着它、又不是不动的眼 ⇒ 放不了这只眼,一个像素都不说是自己
   function Readings_Now (F : Plug.Frame) return Plug.Floats_Vectors.Vector;
   function Self_Mask_Now (F : Plug.Frame; Cam : Natural; Geo : Cam_Geo; W, H : Natural) return Bools;
   --  同 Self_Mask_Now,读数直接给(每条臂一串,下标同装上的臂):开机报告拿参照读数那一刻(扫描起点)比真值用
   function Self_Mask_At (Qs : Plug.Floats_Vectors.Vector; Cam : Natural; Geo : Cam_Geo; W, H : Natural) return Bools;
   --  装上的每条臂的参照读数(运动学的 Q0 = 开机扫描起点那一刻)
   function Reference_Readings return Plug.Floats_Vectors.Vector;
   --  第 Arm 条臂量过表面点没有(没量过 ⇒ 它在哪只眼里占多少、净空、Max_Shift 都说不出)
   function Has_Shape (Arm : Natural) return Boolean;

   --  ⑦ 走一段关节直线,身体的表面挪多远(10-01 路 4 要的:保守推进时每一步走多远)——
   --  第 Arm 条臂沿 Q0 → Q1 这段关节直线走过去,它每一个表面点走过的路长的上界(世界长度,单位同 World_Of)。
   --  上界 = Σ_j |Δq_j| × (转的关节:关节 j 往外那几节的表面点离它的轴最远多远;走的关节:每个读数单位走多远)。
   --  "离轴最远多远"按运动学的链算、走到哪都成立(不是只在参照那一刻量):点到第 j 根轴 ≤ 点到它自己那一节最后一根转轴上一点的距离
   --  + 一路往里每两根相邻转轴上那两点的距离(两点在同一节上,刚体,这段长度不随关节变)+ 中间每个走的关节最多走出去多远
   --  (这一段关节直线两头离参照读数远的那一头)。每根转轴上那一点取这根轴往外那几节表面点的形心在轴上的垂足(取哪一点上界都成立,取得近就紧)。
   --  转的关节读数按弧度算(同 Kinem.FK)。这条臂没量过表面点 ⇒ Known = False、返回 Long_Float'Last(说不出,不当成"挪不了")
   function Max_Shift (Arm : Natural; Q0, Q1 : Floats; Known : out Boolean) return Long_Float;
end Links;
