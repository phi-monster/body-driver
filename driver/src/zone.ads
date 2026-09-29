--  握区:合空一次,每台相机里"合拢通道扫过的像素"= 手指;两瓣之间那片 = 能装东西的区域;
--  一瓣(吸盘、腔)= 那一块自己。区的中心、主轴、张幅、深度全从画面量;两指、五指、吸盘同一段代码。
with Bytes; use Bytes;
with Plug;
with Picture;
with Selfmap;
with Table;
with Geom;
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
   --  张开时的手指分得开、合上时挤在一起 ⇒ 几块形心散得开的那一类是"张开时的手指"(瓣),另一类是手指合到的地方(区)
   function From_Frames (Open_G, Closed_G : Buf; W, Hh : Natural) return Hand_Zone;
   --  同一小截,两个跨度都给:Wide = 宽的那个(= Tip_Band 的 Width,指肚宽的像素),Thin = 窄的那个(看得见的厚的像素)
   procedure Tip_Section (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V, Wide, Thin : out Long_Float; Ok : out Boolean);
   --  每一瓣自己那一块手指像素(同 Tip_Band 的认法:手指像素里和瓣框重合最多的那一整块,8 邻连通)的并集
   function Lobe_Pixels (Z : Hand_Zone; W, Hh : Natural) return Bools;
   --  把手指像素从深度切块结果里剔掉(块心落在手指框或区框里 = 我自己)
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean;
end Zone;
