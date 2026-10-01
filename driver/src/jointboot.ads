--  开机前半段:只用关节命令(V1b 第三步,2026-09-26)。身体报不报"手在哪"都不读 —— 运动学量好之前,手是按关节目标挪的。
--  ① 认身体:每组关节读数一起转一小格,哪台相机整幅都变、而且比第二名多一倍 = 长在这只手上的眼(同 Selfmap.Measure 的判法);
--     跟着一起变的别的组 = 同一只手的回声组(命令的回显),不单算一只手;所有手动时变得最少的那台 = 世界相机。
--  ② 每只有眼的手做关节扫描(每个关节单独两个方向一格一格转;到头 / 被顶住 / 别的关节被顶偏 / 走满 3 格停;
--     再几个关节一起动 N + 1 格),每一格的画面和读数留在内存;
--  ③ 手指遮罩:整段扫描里画面一次都没变过的那些像素 = 跟着眼一起动的自己的手指(或者什么都没有的空白),配点不要它们。
with Plug;
with Selfmap;
with Kinem;
with Geom;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Jointboot is
   type Arm_Info is record
      Group : Natural := 0;          --  这只手的关节读数是第几组(F.Joints 的下标)
      Eye : Integer := -1;           --  长在它上面的那台相机(-1 = 没有)
      Frac : Floats;                 --  这只手一动,每台相机变了多少画面(比例)
      Probe : Long_Float := 0.0;     --  认出来时每个关节一起转了多少(读数的单位)
      Echoes : Ints;                 --  跟着一起变的别的组(回声)
   end record;
   package Arm_Vectors is new Ada.Containers.Vectors (Natural, Arm_Info);

   --  World_Cam = 不长在哪只手上的相机里、手动时变得最少的那台(每台都长在手上 ⇒ -1)
   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean);

   --  一只手扫描下来的全部格子
   type Sweep_Data is record
      Frames : Kinem.Frame_Vectors.Vector;   --  每一格的读数 + 扫的是哪个关节(起点 = -1)
      Imgs : Plug.Cam_Vectors.Vector;        --  每一格手上那只眼的画面
      Runs : Ints;                           --  第几段(同一个关节同一个方向算一段;起点 = 0)
      W, H : Natural := 0;
      Ids : Ints;                            --  每一格在配点仪器那边存的编号(Instrument.Frame_Put;-1 = 没存成)
      Has_Lo, Has_Hi : Bools;                --  每个关节:往负 / 往正扫的时候是"关节到头"停的(量到了这一边的界;没有 = 走满格数停的,或碰上东西了 —— 不是关节尽头,09-28 H4)
      Step_Lo, Step_Hi : Floats;             --  每个关节往负 / 往正扫的最后一格命令的步子(身体开机时一条命令走过的量):反解往到过的范围外最多走这么一步(到过的范围,09-29)
      World_Img : Plug.Cam;                  --  不动的眼(头顶眼)在扫描起点那一刻的画面(没有不动的眼 = 空)
      World_Id : Integer := -1;              --  它在配点仪器那边的编号
   end record;
   package Sweep_Vectors is new Ada.Containers.Vectors (Natural, Sweep_Data);
   package Corr_Set_Vectors is new Ada.Containers.Vectors (Natural, Kinem.Corr_Vectors.Vector, Kinem.Corr_Vectors."=");

   --  ② 关节扫描:有眼的几只手同时扫(一条命令带几组目标),每个关节两个方向一格一格转;到头 / 被顶住 / 别的关节被顶偏 / 走满 3 格就停,
   --  直接去下一段的头一格(回起点和下一段头一格是同一个动作)。每一段头一格和起点那一对当场配点:画面挪了多少 ÷ 实到的转角 = 这个关节
   --  每个读数单位挪几像素,往后每格按它定步子(一格挪画幅宽的 1/5)。配点(起点 ↔ 每一格、每段头两格、相邻关节头一格之间、几个关节一起动的相邻两格)
   --  由配点仪器配(粗配、单向);头一格那几对扫描时配,别的扫完再配。
   --  Host / Port = 仪器;Dump 非空 = 落盘 sweep_*.bmp + sweep.txt。Ds / Css 和 Arms 里有眼的手一一对应(没眼的那只 Ds 空)
   --  World_Cam = 不动的眼是第几台(Find_Arms 认的;-1 = 没有):起点那一刻它的画面也存下,对齐几只手用
   procedure Sweep_All (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Arms : Arm_Vectors.Vector;
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector;
                        World_Cam : Integer := -1);
   --  单关节扫完以后几个关节一起动的格子(09-30 改;纯函数,导出给自检):一组 N 个关节走 N + 1 格,第 C 格(从 0 数)第 J 个关节往正还是往负
   --  = Sylvester 型 Hadamard 矩阵第 C 行、第 J + 1 列的正负(C 和 J + 1 的二进制里同为 1 的位数是偶数 ⇒ 正;第 0 列全正,不用)。
   --  取前 N + 1 行:[全 1 | 各关节的正负] 满秩 ⇒ 每个关节的效果都分得开(N + 1 正好是 2 的幂时各列两两正交,正负各一半)。
   --  原来 8 格、正负按 (格子号 × 37 + 关节号 × 11) mod 16 排:秩最多 8,第 j 与 j + 8 个关节正负恰好相反、j 与 j + 16 完全相同 ——
   --  关节多的一组(人形一条胳膊加手)几个关节的效果分不开
   function Multi_Cells (N_Joints : Natural) return Natural is (N_Joints + 1);
   function Multi_Up (Cell, Joint : Natural) return Boolean;
   --  这样的一格里一个关节往它那一边走多远:预计画面挪 Gw(和单关节每格同一个判据;Px_Per = 这个关节这一边那一段头一格和起点配点量的
   --  每个读数单位挪几像素),不越过这一边扫到过的最远那一格(Reach = 离起点多远)。Px_Per = 0(这一边头一格没量到画面挪)⇒ 画面管不住它,只按 Reach。
   --  原来一律走到扫到过的那一头的一半:头一格小、后面几格被"最多放大四倍"压住的关节(离眼近的腕转)画面只挪四分之一格,离眼远的又多挪一成
   function Multi_Offset (Gw, Px_Per, Reach : Long_Float) return Long_Float is (if Px_Per > 0.0 then Long_Float'Min (Gw / Px_Per, Reach) else Reach);

   --  ④ 这只手的运动学:Kinem.Fit(配点来自扫描的配对)
   --  Note = 这一步的报告(几只手各开一个线程同时解 ⇒ 不在这里打印,解完由调用方按顺序打)
   --  单独扫一个关节时这一格停不停、停了记不记界(纯函数,导出给自检):这一格命令 Step、这个关节实到 Got、别的关节离起点最多偏 Pushed。
   --  停 = 没转到命令的三分之一(到头)或别的关节被顶偏超过这一格的三分之一(碰上东西了);记界 = 到头而别的关节没被顶偏(碰上东西不是关节尽头,09-28 H4)
   function Sweep_Stops (Got, Pushed, Step : Long_Float) return Boolean is (3.0 * Got < Step or else 3.0 * Pushed > Step);
   function Sweep_Stop_Is_End (Got, Pushed, Step : Long_Float) return Boolean is (3.0 * Got < Step and then 3.0 * Pushed <= Step);
   procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                      Note : out Ada.Strings.Unbounded.Unbounded_String);

   --  ⑤ 世界:每只手的运动学在它自己参照读数时那只眼的系里 ⇒ 拿看得见整张桌子的不动的眼(头顶眼)当桥对到一个系:
   --  每只手从自己扫描的格子里三角出桌面点;这些点在头顶眼里的像素 = 腕眼那一格配到头顶眼、再配回来,往返 1 px 内的才算;
   --  世界那只手(能定世界的第一只,World_Arm)的点当板解头顶眼(焦距 + 在它系里的位姿,Geom.Fit_Fixed_Board);别的手:落在它自己桌面上的点,
   --  头顶眼那条视线交世界那只手系里的桌面 ⇒ 同一个点在两个系里 ⇒ 相似变换(抗野点)。长度倍数靠同一张桌面(只靠一只不动的眼,绕它缩放分不出来)。
   --  (V1B11 2026-09-26:原来用两只腕眼起点那一格互相配 —— 两只眼看桌子两头、一点不重叠,倍数解成 0.72、真 1.007)
   --  "上" = 桌面法向(朝世界那只手的眼那边),原点 = 那只手参照眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上;长度单位 = 那只手的模型单位。
   --  Fixed_Eye = 解出来的头顶眼(世界系:R_Ce = 相机 → 世界,Pos;Valid = False 就是没解成)
   type Arm_World is record
      Group : Natural := 0;
      Model : Kinem.Model;
      S : Long_Float := 1.0;                  --  这只手参照眼系 → 世界那只手参照眼系:X0 = S · Ra · X + Ta(世界那只手自己 = 恒等)
      Ra : Geom.M3 := Geom.Identity;
      Ta : Geom.V3 := [0.0, 0.0, 0.0];
      Lo, Hi : Floats;                        --  每个关节记下的尽头(扫描时这一边关节到头 = 扫到的最远那一格;干活时"往范围外走、走不到一半、别的关节都到了"也记;
                                              --  没记 = 不设界;读数越过它 ⇒ 删掉)
      --  到过的范围(09-29,owner:"已知范围,越用越大"):到过的范围 = 开机扫描的各格起、之后每一帧的读数并进来(只会变大);往外一步 = 扫描时那一边最后一格的步子。
      --  发命令时按记下的尽头解出关节目标,每个关节再夹到"到过的范围 + 往外一步"里(大转拆成几条命令、每条都在走过的地方边上,手走过去范围长了就跟着往前);
      --  问"够不够得着"(Reach)只按记下的尽头
      Got_Lo, Got_Hi : Floats;
      Step_Lo, Step_Hi : Floats;
      Eye_W : Natural := 0;                   --  长在它上面那只眼的画幅宽(判"到了"的最小一档 = Kinem.Clean_Tol)
      Valid : Boolean := False;
      Sweep : Natural := 0;                   --  扫描数据(Sweep_All 的 Ds)里是第几只手
      --  放进世界那一步的账(10-01,§2 第 7 条后半;对齐填,世界那只手 = 世界本身,不填):它干活的地方(它的桌面中心)放进世界后最不准的方向、
      --  沿那个方向真的有多不准(形式的 ⊕ 系统那一份按全相关线性加,世界单位)、它自己的眼在那儿量一个点有多不准(桌面上的点的远近,按量到的放大)。
      --  Place_Ok = 前者不比后者大(放法不是它干活时最不准的那一环);False ⇒ 该两只手一起去看共同的近处再放
      Place_Dir : Geom.V3 := [0.0, 0.0, 0.0];
      Place_Sd, Place_Eye_Sd : Long_Float := 0.0;
      Place_Ok : Boolean := True;
   end record;
   package Arm_World_Vectors is new Ada.Containers.Vectors (Natural, Arm_World);
   --  Dump 非空 = 落盘 align_arm<k>.txt(第一行 = 相似变换;每一条配点:世界里哪只眼(第几只手、第几格;-1 = 不长在手上的眼)、这只手的第几个三角点、
   --  那只眼里的像素、这只手那一格里的像素、往返差、放进世界后的点、这只手系里的点)、
   --  fixed_eye.txt(头顶眼,世界那只手的系里)、world.txt(世界系:Rw、O)、world_arm.txt(世界那只手是第几只),离线回放 / 打分用
   --  Board / Plane_* 交给开机后半段(V1b 09-27):板 = 放进世界的手三角出、配进不动的眼的点(世界系位置和协方差、在不动的眼那张起点画面里的像素、
   --  那一批往返差换成的每轴噪声);Plane_Pt / Plane_N = 世界系里的桌面(原点就在桌面上、法向 = +z),Plane_Rms = 桌面上的点离面的离散(标准差)
   --  对齐里两处精修(一只手放进世界、全部放完以后一起精修)重挑内点、重解,做到门里的那一组不再变(09-30 改:原来固定 3 轮,那一组还在变就交了)。
   --  每一轮 Pick 按此刻的解挑出门里的那一组(Enough = 够不够解);和上一轮挑的一样 ⇒ 定下来了(此刻的解就是按这一组解出来的);
   --  和更早的某一轮一样 ⇒ 来回转、定不下来;都不是 ⇒ Solve 按这一组重解,再下一轮。保险:最多解"观测条数 + 1"遍
   --  (那一组要是单调地收或放,N 条观测最多变 N 回;比这还多 = 乱跳)。Rounds = 解了几遍;没定下来(Cycled / Capped)由调用方照实说
   type Settle_Verdict is (Settled, Cycled, Capped, Too_Few);
   generic
      with procedure Pick (Use_Set : out Bools; Enough : out Boolean);
      with procedure Solve (Use_Set : Bools);
   procedure Until_Settled (Rounds : out Natural; Verdict : out Settle_Verdict);
   --  ── I3 眼:每只眼长在谁身上(大并行第 0 步 I3,路 3;10-01 P8A)──
   --  一只眼长在谁身上是开机量出来的(Find_Arms:每组读数一起转一小格,哪台相机整幅都变 = 长在这组上;所有手动时都变得最少的 = 不动的眼),
   --  量清了就不再改判 —— 一条臂扫坏了(运动学没量成 / 放不进世界),它的眼照样是那条臂上的,只是这一回用不了,不改判成"不动的眼"。
   --  (P8A 10-01:右臂扫描撞上柜子把手、卡偏,第 2 只手运动学没量成;往下的身体图只收了量成的手上的眼,第 2 台相机就成了"不长在臂上的",
   --  告诉脑 "through the eye that does not move with me (camera index 2)")
   --  On_Group = 长在第 Group 组读数上(Arm = 装上以后的第几只手,同 Install 按 Valid 的先后数;没装上 = -1);Still = 推哪组都不动(Placed = 放进世界了);
   --  Unclear = 量不清(推哪组都没认出它跟着动,也不是那只最不动的)⇒ 照实说量不清,不猜
   type Eye_Carrier is (On_Group, Still, Unclear);
   type Eye_Info is record
      Kind : Eye_Carrier := Unclear;
      Group : Integer := -1;
      Arm : Integer := -1;
      Placed : Boolean := False;
      World : Boolean := False;   --  它长在世界那只手上(世界 = 那只手参照眼系)
   end record;
   package Eye_Vectors is new Ada.Containers.Vectors (Natural, Eye_Info);
   --  Eyes (A) / Groups (A) / Valid (A) = 第 A 只认出来的手上的相机(-1 = 没有)、它的读数组、它装没装上(运动学量成并且放进了世界);World_Cam = 不动的眼(-1 = 没有);
   --  Ref = 世界那只手(-1 = 没定成);Fixed_Placed = 不动的眼放进了世界。纯函数(导出给自检)
   function Eye_List (N_Cams : Natural; Eyes, Groups : Ints; Valid : Bools; World_Cam, Ref : Integer; Fixed_Placed : Boolean) return Eye_Vectors.Vector;
   function Eye_Say (L : Eye_Vectors.Vector) return String;
   --  世界取哪只手(大并行 §2 第 7 条 / 旧不足 4:原来第一组读数写死当世界,它扫坏了整个开机就退出,"定不了世界"):
   --  Tilt_Sd (A) = 第 A 只手自己的桌面拟合出来"上"有多准(倾角的标准误差,弧度:离面散布 ÷ √面上的点数 ÷ 面铺开的大小);< 0 = 这只手当不了世界
   --  (运动学没量成 / 三角出的点不够)。取当得了的第一只(按开机认出来的先后);没有一只当得了 ⇒ -1。纯函数(导出给自检)。
   --  不按"上"最准的挑(10-01 量过):V1B68 / V1B69 / P3B / P3C 四炮按它都挑第 2 只(0.00002–0.00003 对 0.00003–0.00004 弧度,两只都远好过用得着的),
   --  世界换到第 2 只以后,拿掉头顶眼的两炮把第 1 只手放坏(差 1036 / 957 mm、转 20° / 18°),另两炮 2.8 / 4.7 mm(第 1 只当世界 2.2 / 3.6),
   --  有头顶眼的两炮 2.0 / 2.0 mm(第 1 只当世界 0.4 / 0.7)⇒ 放法对哪只手当世界不对称,"上"准不准不代表"当世界准不准";
   --  哪个量能代表,要先量出来(照实写在报告里),量出来之前不按一个没证过的量换世界
   function World_Arm (Tilt_Sd : Floats) return Integer;

   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                    Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "";
                    Pin_Fixed_F : Long_Float := 0.0;
                    Eyes : Ints := Int_Vectors.Empty_Vector; World_Cam : Integer := -1; N_Cams : Natural := 0);
   --  Eyes / World_Cam / N_Cams(10-01,可以不给):每只认出来的手上的相机(和 Worlds 一一对应)、不动的眼、一共几台 ⇒ 对齐完照实说每只眼长在谁身上(Eye_List)
   --  Pin_Fixed_F > 0:不动的眼的焦距钉在这个值上不解(只给离线回放做对照实验用 —— alignexam 的 ALIGNEXAM_PIN_F;驱动开机永远不给,焦距一起解)

   --  ── 放一只手进世界(10-01,大并行 §2 第 7 条后半)──
   --  V1B68 / V1B69 拿掉头顶眼:第 2 只手沿两手连线偏 38 mm。离线按真值查出三件事:
   --  ① 两只腕眼共同看见的 97% 是远处的墙,这些点的远近是第 2 只手自己扫描(基线几厘米)三角出来的,按真值错 1–1.3%,是它自报的 2.6–3.5 倍,
   --     而且在画面里成片(左边、右边 −1.2 … −2.4%,中间 +0.1 … +0.6%);残差主要沿对极线(0.9–1.1 px,垂直的只有 0.24–0.34 px)。
   --  ② "沿连线挪 + 反着转"挪远点,恰好也是沿对极线挪 ⇒ 远点远近那片误差和这个方向分不开;形式上的不确定度说沿连线 1.9 mm,实际偏 38 mm。
   --  ③ 近处桌面上共同看见的 31 / 10 个点按真值只差 0.9–1.8 px,按偏的解差 24–30 px、要改远近 −7%、100% 同号 —— 不是野点,被门当野点挑掉了。
   --  所以:配点噪声按量的分两份(Noise_Of);精修定了以后,被门挡掉的点要是一致地指向另一个解,就去那一坑再解、按混合似然比(Place_Hand);
   --  放完照实算它干活的地方沿最不准的方向真的有多不准(Hand_Sd:系统那一份按全相关线性加)。
   --
   --  一条配点:这只手三角出的一个点(它自己系里 X、协方差 Cov;远近那一维沿 X 的方向 —— 点都是从它的参照眼三角出来的)配进世界里一只眼
   --  (Cam:世界系位姿、焦距、主点)的像素 (U, V)。W × H = 那只眼的画幅(混合似然里门外的点均匀落在画面上);
   --  Grp = 配点噪声按组各自量(配进手的格子 / 配进不长在手上的眼,视角差得远,噪声不一样);Rd = 那只眼过 (U, V) 的视线(世界系单位方向,起点 Cam.Pos;网格起步用)
   type Hand_Ob is record
      X : Geom.V3 := [0.0, 0.0, 0.0];
      Cov : Geom.M3 := [others => [others => 0.0]];
      Cam : Geom.Cam_Geo;
      U, V : Long_Float := 0.0;
      W, H : Natural := 0;
      Grp : Natural := 0;
      Rd : Geom.V3 := [0.0, 0.0, 0.0];
   end record;
   package Hand_Ob_Vectors is new Ada.Containers.Vectors (Natural, Hand_Ob);
   --  它自己的桌面对上世界的桌面(3 条残差:两个倾角、一个高度):它的桌面中心 Cb、朝它的眼的法向 Nb(它自己系),倾角(弧度)/ 高度(世界单位)的不确定度 Sn / Sd;
   --  世界的桌面 P0、朝世界那只手的眼的法向 N0
   type Plane_Tie is record
      Cb, Nb : Geom.V3 := [0.0, 0.0, 0.0];
      Sn, Sd : Long_Float := 0.0;
      P0, N0 : Geom.V3 := [0.0, 0.0, 0.0];
   end record;
   type Hand_Place is record
      S : Long_Float := 1.0;                  --  这只手系 → 世界:X_世界 = S · R · X + T
      R : Geom.M3 := Geom.Identity;
      T : Geom.V3 := [0.0, 0.0, 0.0];
      Inl : Natural := 0;                     --  门里的配点
      Md : Long_Float := 0.0;                 --  门里的白化残差中位
      Sm : Floats;                            --  每组配点噪声(像素,按垂直对极线那一份量)
      K : Long_Float := 1.0;                  --  这只手三角点的远近真的不准是它自报的几倍(按沿对极线那一份量)
      Cost : Long_Float := 0.0;               --  混合似然代价(门里 = 二维正态、门外 = 均匀落在画面上;各自按自己量的噪声,在像素里比)
      Escapes : Natural := 0;                 --  换到代价更低的一坑,换了几次
      Weak : Geom.V3 := [0.0, 0.0, 0.0];      --  它干活的地方(它的桌面中心)放进世界后最不准的方向(世界系单位向量)
      Sd_Formal, Sd_Sys, Sd_Real : Long_Float := 0.0;   --  沿那个方向:形式的、系统那一份按全相关线性加的、合起来的(世界单位)
      Lm_Capped : Boolean := False;           --  哪一遍的 LM 做满保险的次数代价还在降(没解到底,调用方照实说)
      Unsettled : Boolean := False;           --  精修门里的那一组没定下来(来回转 / 解满保险的遍数)
   end record;
   --  从 P 的放法(网格起步挑的那一格,S / R / T)起:① 精修 —— 每轮按此刻的解量方差分量(Noise_Of)、按混合模型的分界挑门里的(像门里的比像
   --  均匀落在画面上的乱配似然大,γ 按 EM)、Huber 解,做到门里的那一组不再变;
   --  ② 出坑 —— 在它干活的地方最不准的方向上,每条被门挡掉的配点各报一个"沿这个方向挪多少我就对上了"(带它自己的不确定度),
   --  核密度最高的那一处 + 提议落在那儿(Z 倍自己的不确定度以内)的那几条 = 另一坑的共识,先按共识解、再照 ① 精修;
   --  混合似然代价(各自按自己量的噪声)低过似然比的门(Z² / 2)才换,换了再找,到换不动为止;③ Hand_Sd。
   --  Sig0 = 精修第一轮之前每条配点的噪声(像素;往返差的中位)。Ok = False:门里的配点不到 Min_Inl 条
   procedure Place_Hand (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; Sig0 : Long_Float; P : in out Hand_Place; Ok : out Boolean);
   --  在 P 的放法、P 的噪声(Sm、K)上重算:门里的配点、沿最不准的方向形式的 / 系统的 / 真的不准(一起精修以后也按这个再算一遍)
   procedure Hand_Sd (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; P : in out Hand_Place);
   --  方差分量(纯函数,导出给自检):每条配点的像素残差拆成垂直对极线(Ep,方差 Sm² + Vp)和沿对极线(Ea,方差 Sm² + Va + K² · Vd:Vd = 自报的远近投进来的方差)。
   --  每组的 Sm:让 Mad_Sigma × 中位 |Ep| / √(Sm² + Vp) = 1;再按各组的 Sm 让 Mad_Sigma × 中位 |Ea| / √(Sm² + Va + K² Vd) = 1 定 K
   --  (都单调 ⇒ 二分到区间不再缩)。Live = 算哪几条(门里的);某组一条都没有 ⇒ 那组 Sm = Sig0;没有一条带远近 ⇒ K = 1(没法量,照自报的)
   procedure Noise_Of (Ep, Vp, Ea, Va, Vd : Floats; Grp : Ints; Live : Bools; N_Grp : Natural; Sig0 : Long_Float; Sm : out Floats; K : out Long_Float);
   --  一串提议(T,各自的不确定度 St):每个人按自己的不确定度当宽度的核密度,在每个提议处取值,最高的那一个(下标;空 ⇒ -1)
   function Peak_Of (T, St : Floats) return Integer;
   --  门里还是乱配(混合模型自己的分界,纯函数,导出给自检):W2 = 每条配点白化残差的 |w|²,Dens = 乱配(均匀落在那只眼的画面上)在白化单位里的密度
   --  (这条配点的像素面积 ÷ 画幅面积);门里的占比 Gamma 按 EM 解到不再变;Inl = 这一条像门里的(二维单位正态)比像乱配的似然大
   procedure Mix_Gate (W2, Dens : Floats; Inl : out Bools; Gamma : out Long_Float);

   --  到过的范围(09-29):开机扫描的一只手 ⇒ 记下的尽头(Lo / Hi:这一边是关节到头停的 ⇒ 扫到的最远那一格,否则不设界)、到过的范围(各格读数的两头)、
   --  往外一步(Step_Lo / Step_Hi)、眼的画幅宽
   procedure Set_Ranges (D : Sweep_Data; W : in out Arm_World);
   --  发命令时每个关节夹到的界 = 到过的范围往外一步,不越过记下的尽头(纯函数,导出给自检)
   procedure Cmd_Bounds (W : Arm_World; Lo, Hi : out Floats);
   --  一条命令走完、停下以后,有没有碰到关节的尽头(纯函数,导出给自检)。Q_Cmd = 这条命令的关节目标,Q_At = 发命令时的读数,Q_Now = 停下时的读数,
   --  Got_Lo / Got_Hi = 发命令时到过的范围,Tol = 判"到了"的最小一档(Kinem.Clean_Tol)。
   --  "要到范围外"= 目标出了到过的范围 Tol 以上。End_Hit = 正好一个这样的关节走到的不到它要往外走的那一截的一半,别的关节都到了
   --  (差不到 Tol 或它这一下要走的三分之一,同扫描)⇒ J = 那个关节,Hi_Side = 正那一边,记尽头;Blocked = 有关节没到 / 被顶偏 = 手碰上东西了,
   --  不是关节尽头(同 Sweep_Stop_Is_End;J = 那个没走到的关节,没有则 -1);Ambiguous = 两个以上要到范围外的关节都没走到,分不清是哪一个;
   --  Reached = 都到了。只有 End_Hit 记
   type End_Verdict is (Reached, End_Hit, Blocked, Ambiguous);
   --  Step_Lo / Step_Hi = 每个关节那一边量过的步子:要往范围外走的不到半步的关节不核(走没走到都判不准)
   function Judge_End (Q_Cmd, Q_At, Q_Now, Got_Lo, Got_Hi, Step_Lo, Step_Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return End_Verdict;
   --  读数越过了记下的尽头(超过 Tol)⇒ 那个尽头记错了,J / Hi_Side = 哪一个(纯函数,导出给自检)
   function End_Passed (Q, Lo, Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return Boolean;

   --  ⑥ 装上:从此插头每一帧的手的位姿 = 按关节读数算出的世界里的腕眼位姿;位姿命令 = 按记下的尽头解出关节目标、每个关节夹到"到过的范围 + 往外一步"里(到过的范围)。
   --  Joint_Noise = 开机量的关节读数噪声(判"停下了"的下限)
   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3; Joint_Noise : Long_Float := 0.0);
   --  上一条位姿命令的反解被"到过的范围往外一步"截住了没有、之后范围长了没有(Plug.Limit_State;Selfmap.Go 据此重发;插头的 Held_Back 钩子)
   function Held_Back (Arm : Natural) return Plug.Limit_State;
   --  插头的两个钩子(Install 登记)
   procedure Pose_Hook (F : in out Plug.Frame);
   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean);
   --  这个位姿在量到的关节限位里解得出来吗(不发命令):解完还差多少(位置按世界单位,朝向按弧度);同一条 Pose_To_Q
   procedure Reach_Hook (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float);
   --  开机自检(V1b 的 ②):每只装上的手走到扫描时没去过的几处 —— 两格"几个关节一起动"的读数的正中(每个关节都在量过的范围里),
   --  按运动学算出那一处眼的位姿当目标,按位姿命令同一条路(Pose_To_Q:在量过的范围里反解)解成关节目标;几只手同时走(一条命令带几组目标),
   --  停稳后记:目标、反解还差多少、实到的读数。身体报的位姿只落盘给离线打分(Dump/ik_check.txt),驱动不读
   procedure Self_Check (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Ds : Sweep_Vectors.Vector; Dump : String);

   --  ⑦ 存 / 装回(开机后半段第 ⑤ 条,09-27):前半段量到的写进 <身体文件>.kin.txt,核对用的图写成 <身体文件>.kin.txt_arm<k>.bmp(每只手扫描起点那一格的腕眼图)
   --  和 .kin.txt_world.bmp(不动的眼那一刻的图)。装回:钥匙对得上 ⇒ 每只手按关节命令回到存的参照读数、停稳,拍一张和存的比
   --  (配点仪器问格点,往返 1 px 内的留下,Same_View);不动的眼同样。都过 ⇒ 不扫描、不解,直接装上;任何一项不过 ⇒ 说出哪只挪了多少,从零量
   type Kin_Store is record
      Key : Ada.Strings.Unbounded.Unbounded_String;
      Worlds : Arm_World_Vectors.Vector;
      Eyes : Ints;                               --  每只手:长在它上面的相机
      Ds : Sweep_Vectors.Vector;                 --  每只手:扫描各格的读数(开机自检要)、画幅、起点那一格的腕眼图(Imgs 只存第 0 格);第 0 只的 World_Img = 不动的眼的图
      Rw : Geom.M3 := Geom.Identity;
      O : Geom.V3 := [0.0, 0.0, 0.0];
      World_Cam : Integer := -1;
      Fixed_Eye : Geom.Cam_Geo;
      Board : Geom.Scene_Pt_Vectors.Vector;
      Plane_Pt, Plane_N : Geom.V3 := [0.0, 0.0, 0.0];
      Plane_Rms : Long_Float := 0.0;
   end record;
   --  钥匙:几台相机、每台画幅、几组关节读数、每组几个、几个抓握通道、关节读数的字段名(不含身体报的位姿:只报关节的身体装上以后才有位姿)
   function Kin_Key (L : Plug.Link; F : Plug.Frame) return String;
   --  Images = False:只写字(干活时范围长了 / 尽头变了写回,核对用的图开机时已经写过)
   procedure Save_Kin (Path : String; K : Kin_Store; Images : Boolean := True);
   --  装上以后记住身体文件和这一份:干活时到过的范围长了一步以上、或者记下 / 删掉一个尽头,就写回去(越用越准,装回接着用)
   procedure Remember_Kin (Path : String; K : Kin_Store);
   procedure Load_Kin (Path : String; K : out Kin_Store; Ok : out Boolean; Note : out Ada.Strings.Unbounded.Unbounded_String);
   procedure Check_Kin (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; K : Kin_Store; Host : String; Port : Natural;
                        Ok : out Boolean; Note : out Ada.Strings.Unbounded.Unbounded_String);
   --  核对的判法:一对图(存的 → 此刻)配上的点的位移(像素)⇒ 没动 = 至少 10 个、位移中位 < 1 px(同往返门,协议)
   function Same_View (Disp : Floats) return Boolean;
   --  装回时把存的几何照驱动落盘的格式写进 Dump(kinem_arm<k>.txt、align_arm<k>.txt 第一行、world.txt、fixed_eye.txt),离线打分用
   procedure Dump_Kin (Dump : String; K : Kin_Store);
end Jointboot;
