--  握区:合空一次,每台相机里"合拢通道扫过的像素"= 手指;两瓣之间那片 = 能装东西的区域;
--  一瓣(吸盘、腔)= 那一块自己。区的中心、主轴、张幅、深度全从画面量;两指、五指、吸盘同一段代码。
with Bytes; use Bytes;
with Plug;
with Picture;
with Selfmap;
with Table;
with Geom;
with Json;
with Ada.Containers.Vectors;
package Zone is
   --  一瓣 = 这只眼里一根(或并在一起看不开的几根)手指张开时那一块:框、形心(归一化画幅)、像素数
   type Lobe is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
   end record;
   package Lobe_Vectors is new Ada.Containers.Vectors (Natural, Lobe);
   No_Lobe : constant Lobe := (Valid => False, others => <>);
   --  一小截(手指尖那一截):像素中心 (U, V)、两个跨度(宽的那个 / 窄的那个,像素),和它那一整块手指像素从画面哪儿伸进来
   --  (贴画面边那些像素的中点 (Eu, Ev))。Ok = False:没有这一截
   type Section is record
      Ok : Boolean := False;
      U, V, Wide, Thin, Eu, Ev : Long_Float := 0.0;
   end record;
   type Hand_Zone is record
      Valid : Boolean := False;
      --  区心(归一化)= 这几瓣合拢时会合到的那一点("东西会被夹在哪"):每一对瓣在连线中点会合的最小二乘解(= 各瓣形心的平均),
      --  这些对定不下来的方向(只有一瓣)按手指合拢时到的那片(合到的区)的形心 —— 伪逆解,一条规则
      Cu, Cv : Long_Float := 0.0;
      --  主轴(像素系单位向量,和 Picture.Region 的主轴同一个系、同一个正负号约定):瓣排开的方向(各瓣形心的主方向);
      --  瓣的排法定不出方向时(一瓣,或者几瓣均匀排一圈)= 各瓣自己伸长的方向
      Au, Av : Long_Float := 0.0;
      Span : Long_Float := 0.0;        --  张幅:能装东西的那片沿主轴的伸展(归一化画幅)
      Depth : Long_Float := 0.0;       --  手指深度(米);NaN = 读不到
      N_Lobes : Natural := 0;          --  = Lobes 的个数(Set_Lobes 填;旧写法写的握区 = 它自己填的那个数)
      A, B : Lobe;                     --  旧的两格:Set_Lobes 照今天的身体照旧填(= 第 0、1 瓣),别人不坏;合并时主代理删
      Fingers : Bools;                 --  扫过的像素(手指本身)
      X0, Y0, X1, Y1 : Natural := 0;   --  区框
      --  I2(大并行 路 2):量到的每一瓣都在这一串里,一个不少、没有上限(0 个 = 这只眼里没有手指)。
      --  只经 Set_Lobes / Set_Lobe 写、只经 Lobe_Of 读;瓣数 = 它的个数,驱动里没有按瓣数的分支
      Lobes : Lobe_Vectors.Vector;
      --  合空时手指到的那一截(大并行 §2 第 4 条"合拢那一路:张到头、合空两头都量"):合到的那片里离"手从画面外伸进来的地方"最远的那一小截
      --  (同 Tip_Section 的认法:合到的那片所在的那一整块手指像素,贴画面边的那些 = 伸进来的地方)。几瓣合空时到一起 ⇒ 这一截就在会合的那一处;
      --  一边不动的夹爪(只有一瓣在动)⇒ 动的那一根合到不动的那一根旁边。三维位置碰桌面量(Geom.Lobe_Geo.Shut)。
      --  Ok = False:合到的那片没有,或者它那一整块不贴画面边(看不出手从哪伸进来,不猜)
      Shut : Section;
      --  量握区那一刻(From_Frames)哪一类是张开时的手指定没定下来:那一类的几块散得比另一类开(两两形心之间),或者离两头都长在眼上、
      --  合拢时没跟着动的部分(手掌、不动的那根手指)比另一类远。两类一样开 ⇒ False(只有一块在动、又不知道不动的部分在哪 ——
      --  张开、合上两头各一块,画面里分不出哪头是张开的,不猜)。只在量的那一刻用,不存
      Open_Known : Boolean := False;
      Lobes_Darker : Boolean := False;   --  瓣是 Closed_G 里变暗的那一类(量的那一刻;另有证据要换成另一类时用,不存)
   end record;
   package Zone_Vectors is new Ada.Containers.Vectors (Natural, Hand_Zone);
   --  🔴 第 I 瓣(I ≥ 瓣数 ⇒ No_Lobe)。别处一律走这个口子,不许直接读写 Z.A / Z.B / Z.Lobes ——
   --  一瓣(吸盘)、两指、五指、七指,同一段代码。瓣数一律读 Z.N_Lobes,不许写死。
   --  Lobes 空着、A / B 却填了的握区是旧写法写的(bodyfile-load 读身体文件那一段、selfcheck.adb 里自己拼握区的几条焊点):
   --  那时照旧读它的两格。它们换成 Set_Lobes / Lobes_From_Json 以后(主代理合并时),连同 A、B 一起删
   function Lobe_Of (Z : Hand_Zone; I : Natural) return Lobe;
   --  换掉整串瓣:Lobes := Ls,N_Lobes := 个数,A / B := 第 0 / 1 瓣(旧的两格照旧填)。Ls 里每一瓣都是量到的(Valid)
   procedure Set_Lobes (Z : in out Hand_Zone; Ls : Lobe_Vectors.Vector);
   --  换掉第 I 瓣(I < 瓣数;旧写法写的握区先按它的两格转成一串)
   procedure Set_Lobe (Z : in out Hand_Zone; I : Natural; Lb : Lobe);
   --  一串瓣 ↔ 身体文件(JSON)。每瓣一个 7 个数的数组 [x0, y0, x1, y1, cu, cv, 像素数](和旧文件里 "a" / "b" 那两格一个排法),
   --  浮点按 Json.Number 写(写出去读回来一个比特不差)。Lobes_Json 返回 {"rule": 这一版握区算法的号, "list": [[…],[…],…]}(0 瓣 = 空串),
   --  存的时候写成这只眼握区里的 "lobes" 键;Lobes_From_Json 从握区节点 Zn 读回:"lobes" 是这样一个对象按它;是一串(I2 第一版)、
   --  或者没有(I2 以前的文件:两格 "a" / "b",按 "n_lobes" 取前几格)也读回来。读完 Set_Lobes。
   --  存它的算法不是这一版的(区心、主轴按旧算法算的,文件里没有能重算的画面)⇒ 照实说、Z.Valid := False:开机把这一格当没量过,合空重量
   function Lobes_Json (Z : Hand_Zone) return String;
   procedure Lobes_From_Json (D : Json.Doc; Zn : Integer; Z : in out Hand_Zone);
   type Hand is record
      Arm : Natural := 0;
      K : Natural := 0;                --  这条臂的第几个抓握通道(五指手有五个,两指手只有 0 号)
      Zones : Zone_Vectors.Vector;     --  每台相机一个
      Empty_Close : Long_Float := 0.0; --  合空时的读数
      Open_Reading : Long_Float := 1.0;
      Close_Steps : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];   --  合空时这只手的位姿(别的相机里的握区只在这个位姿下成立)
      --  开机真推到两头量过(Zone.Measure 量成,或者从身体文件装回的量过的那份)。没量过的 Empty_Close / Open_Reading 没有意义,
      --  谁都不许拿它们当目标或当门(09-30:原来缺省"合 0、张 1"= x5 的约定,找不到这只手时就被当成量过的发出去)
      Measured : Boolean := False;
   end record;
   package Hand_Vectors is new Ada.Containers.Vectors (Natural, Hand);

   --  合【第 K 个抓握通道】一次(其余通道保持不动),看它扫过哪些像素 = 那一根(或那一组)手指。
   --  一条臂上有几个通道是量出来的:两指手 1 个,五指手 5 个,代码一处都不用改。
   --  Host / Port = 配点仪器(判哪头张开要问转之前 / 转之后格点配到哪;没配仪器 ⇒ 哪头张开量不出,照实说);
   --  Eyes = 每台相机量过的几何(按相机号;手上那只眼的焦距、主点是开机运动学量的,判哪头张开时拿它把"眼转了多少"投回画面)
   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm, K : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean;
                      Host : String; Port : Natural; Eyes : Geom.Geo_Vectors.Vector);
   --  判哪头张开时手绕世界竖直轴(z)转出去的那一下(按通道,纯函数,导出给自检):只有绕 z 转的那个通道(平移三个之后的第三个)有数,
   --  = 64 倍它自己的探针幅度。这只手的那个通道没量过 ⇒ 全零(转不了)
   function Turn_Step (M : Selfmap.Body_Map; Arm : Natural) return Table.Vec;
   --  没有深度时的握区(纯函数,可离线测):只看两张停住的画面(张开 vs 合上)。
   --  变化大的像素才是手指来去(分界 = 变化量的两拨分界,算出来的;不用噪声地板 —— 腕上的相机一合爪整幅画面都抖,按地板算下半幅全"动了",
   --  S5 2026-09-23 实测 12.7 万像素被记成手指)。变暗的一拨和变亮的一拨是两类:一类是手指离开露出背景,一类是手指到来盖住背景;
   --  张开时的手指分得开、合上时挤在一起 ⇒ 几块形心散得开的那一类是"张开时的手指"(瓣),另一类是手指合到的地方(区)。
   --  Static = 两头都长在眼上、合拢时没跟着动的像素(开机按转一下眼判的格点:手掌、不动的那根手指;空 = 不知道):
   --  "散得开"也算手指伸出去离它们最远多远 —— 一边不动的夹爪只有一块在动,张开那头离不动的那根远、合上那头贴着它(10-01 路 5 要的"一边固定、一边动");
   --  五指手一个通道一起合,张开那头四指伸直、离手掌远,合上那头蜷在手掌边上(H4)。
   --  两类一样开 ⇒ Open_Known = False(照原来按像素多少排一个出来,调用方不许照用)。
   --  Open_Class = 调用方另有证据时指定哪一类是张开时的手指(1 = Closed_G 里变暗的那一类,-1 = 变亮的那一类;0 = 按上面判)
   function From_Frames (Open_G, Closed_G : Buf; W, Hh : Natural; Static : Bools := Bool_Vectors.Empty_Vector;
                         Open_Class : Integer := 0) return Hand_Zone;
   --  同一小截,两个跨度都给:Wide = 宽的那个(= Tip_Band 的 Width,指肚宽的像素),Thin = 窄的那个(看得见的厚的像素)
   procedure Tip_Section (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V, Wide, Thin : out Long_Float; Ok : out Boolean);
   --  每一瓣自己那一块手指像素(同 Tip_Band 的认法:手指像素里和瓣框重合最多的那一整块,8 邻连通)的并集
   function Lobe_Pixels (Z : Hand_Zone; W, Hh : Natural) return Bools;
   --  这一瓣自己那一块(同上)= 它在这只眼里的剪影(张开到合上扫过的,包住张开那一头);Through = 这一块沿画面边贴着不止一段(穿过画面:
   --  尖在画面外,看得见的那一截里没有尖)。碰指尖几下一起解时拿它核解出来的尖(Geom.Fit_Presses 的 Finger_View)
   function Lobe_Mask (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; Through : out Boolean) return Bools;
   --  这一瓣从哪儿伸进画面:它那一块(同上)贴画面边的那些像素的中点(Eu, Ev)。手指的身子在画面里从尖往这儿去;没有贴边的 ⇒ Ok = False
   --  (Tip_Section 那时也认不出尖)。碰指尖斜着压时只朝它的反方向那半边斜(Geom.Azim_Of)
   procedure Lobe_Entry (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; Eu, Ev : out Long_Float; Ok : out Boolean);
   --  把手指像素从深度切块结果里剔掉(块心落在手指框或区框里 = 我自己)
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean;

   --  ── 瓣按"长在眼上"补全(09-30)──
   --  瓣原来只按"张开 / 合上两头之间变了的像素"认:手指身后的背景和手指一样暗的那一截认不出。V1B69 第 1 只手左边那根手指上半截贴着暗的墙,
   --  瓣的尖认低了 46 px(117,289 → 108,335),顺着它那条视线压下去解出的尖离视线 0.25 单位(V1B66–68 0.04–0.08),指尖量歪;
   --  那一炮量握区时手停的姿态是开机自检最后走到的那一处(扫描格改了以后它跟着变),背景是随手的。手指跟着眼走 ⇒ 转一下眼,在转之前 / 转之后两帧里
   --  不挪的像素就是手指,和背景亮暗无关(离线 V1B69:左边那根整根认出来,尖顶到 y = 256)。
   --  一个要问的像素:在哪(按像素中心问)、归哪一瓣
   type Probe is record
      U, V : Long_Float := 0.0;
      Lobe : Natural := 0;
   end record;
   package Probe_Vectors is new Ada.Containers.Vectors (Natural, Probe);
   --  要逐像素问的那些(纯函数):格点(Kinem.Gx × Gy,按格子)里判成长在眼上的(Grid_Ride),从和某一瓣的框重叠的那几格起,沿长在眼上的格子
   --  (8 邻)往外连,每一格归最先连到它的那一瓣;连到的格子、再加它们四周不长在眼上的那一圈(手指边上的半格),里面每一个像素都问。
   --  挨着别的瓣的格子的那一格不问(两瓣在手指像素里不许连成一块,不然两瓣的尖按同一块算);这一格没收到就不补。
   function Refine_Probes (Z : Hand_Zone; W, Hh : Natural; Grid_Ride : Bools) return Probe_Vectors.Vector;
   --  问回来的(纯函数):Mt (I) = Ps (I) 在转出去那一帧里配到的像素(U < 0 = 配不出);按格点拟合的转动 Rot、配点噪声 Sig(Kinem.Fit_Eye_Turn)
   --  逐个判(Kinem.Classify_Rides),长在眼上的并进手指像素(Z.Fingers),那一瓣的框扩到连它们;只收能整块盖住一个格子的(开运算:
   --  补全的证据是格点,比一格还薄的 —— 遮挡边上配点仪器往外带的那一圈 —— 判不了)。Added = 新并进来的像素数
   procedure Apply_Refine (Z : in out Hand_Zone; W, Hh : Natural; Ps : Probe_Vectors.Vector; Mu, Mv : Bytes.Floats; G : Geom.Cam_Geo; Rot : Geom.V3;
                           Sig : Long_Float; Added : out Natural)
     with Pre => Natural (Mu.Length) = Natural (Ps.Length) and then Natural (Mv.Length) = Natural (Ps.Length);
end Zone;
