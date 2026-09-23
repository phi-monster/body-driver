--  握区:合空一次,每台相机里"合拢通道扫过的像素"= 手指;两瓣之间那片 = 能装东西的区域;
--  一瓣(吸盘、腔)= 那一块自己。区的中心、主轴、张幅、深度全从画面量;两指、五指、吸盘同一段代码。
with Bytes; use Bytes;
with Plug;
with Picture;
with Selfmap;
with Ada.Containers.Vectors;
package Zone is
   type Lobe is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
   end record;
   type Hand_Zone is record
      Valid : Boolean := False;
      Cu, Cv : Long_Float := 0.0;      --  区心(归一化)
      Au, Av : Long_Float := 0.0;      --  瓣到瓣的方向(单瓣时为主轴)
      Span : Long_Float := 0.0;        --  瓣心距(归一化画幅)
      Depth : Long_Float := 0.0;       --  手指深度(米);NaN = 读不到
      N_Lobes : Natural := 0;
      A, B : Lobe;
      Fingers : Bools;                 --  扫过的像素(手指本身)
      X0, Y0, X1, Y1 : Natural := 0;   --  区框
   end record;
   package Zone_Vectors is new Ada.Containers.Vectors (Natural, Hand_Zone);
   --  🔴 第 I 瓣。别处一律走这个口子,不许直接写 Z.A / Z.B ——
   --  今天这具身体的握区只记得两瓣(A/B),以后长出五瓣、七瓣、吸盘一个点,只改这一处,
   --  上面所有"每瓣一个接触点"的代码一个字都不用动。瓣数一律读 Z.N_Lobes,不许写死。
   function Lobe_Of (Z : Hand_Zone; I : Natural) return Lobe is
     (if I = 0 then Z.A elsif I = 1 then Z.B else (Valid => False, others => <>));
   type Hand is record
      Arm : Natural := 0;
      K : Natural := 0;                --  这条臂的第几个抓握通道(五指手有五个,两指手只有 0 号)
      Zones : Zone_Vectors.Vector;     --  每台相机一个
      Empty_Close : Long_Float := 0.0; --  合空时的读数
      Open_Reading : Long_Float := 1.0;
      Close_Steps : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];   --  合空时这只手的位姿(别的相机里的握区只在这个位姿下成立)
   end record;
   package Hand_Vectors is new Ada.Containers.Vectors (Natural, Hand);

   --  合【第 K 个抓握通道】一次(其余通道保持不动),看它扫过哪些像素 = 那一根(或那一组)手指。
   --  一条臂上有几个通道是量出来的:两指手 1 个,五指手 5 个,代码一处都不用改。
   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm, K : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean);
   --  从"合空扫过的像素 + 张开时的深度 + 合上时的深度"算出握区(纯函数,可离线测):
   --  近的那一拨(张开时就在近处)= 手指;扫过但张开时是远处 = 手指合拢时要盖过的地方 = 能装东西的区。
   function From_Sweep (Swept : Bools; Depth_Open, Depth_Closed : Floats; Has_Depth : Boolean; W, Hh : Natural) return Hand_Zone;
   --  没有深度时的握区(纯函数,可离线测):只看两张停住的画面(张开 vs 合上)。
   --  变化大的像素才是手指来去(分界 = 变化量的两拨分界,算出来的;不用噪声地板 —— 腕上的相机一合爪整幅画面都抖,按地板算下半幅全"动了",
   --  S5 2026-09-23 实测 12.7 万像素被记成手指)。变暗的一拨和变亮的一拨是两类:一类是手指离开露出背景,一类是手指到来盖住背景;
   --  张开时的手指分得开、合上时挤在一起 ⇒ 几块形心散得开的那一类是"张开时的手指"(瓣),另一类是手指合到的地方(区)
   function From_Frames (Open_G, Closed_G : Buf; W, Hh : Natural) return Hand_Zone;
   --  一瓣手指的指尖落在画面哪个像素(纯函数):指尖 = 这根手指【伸向手指合拢处】的那一头 —— 从这瓣的形心指向区框中心(手指合到的地方)的方向上,
   --  投影最远的那一小截(1/80 画幅高,比例无量纲)的形心。不假设指尖在画面上方或下方:腕上的眼、头顶的眼、任何身体同一条定义
   procedure Tip_Px (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V : out Long_Float; Ok : out Boolean);
   --  把手指像素从深度切块结果里剔掉(块心落在手指框或区框里 = 我自己)
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean;
end Zone;
