--  画面上的量:深度切块(闭运算找"鼓起来的块")、块掩膜、近侧深度、动过的像素、连通块与主轴。
--  不认识任何物体,没有相机内参,没有一个身体量;两个无量纲数(窗口比例、σ 倍数)的含义写在各自用处。
with Bytes; use Bytes;
with Ada.Containers.Vectors;
package Picture is
   type Region is record
      X0, Y0, X1, Y1 : Natural := 0;        --  像素框(闭区间)
      Count : Natural := 0;
      Cu, Cv : Long_Float := 0.0;           --  形心(归一化画幅)
      Depth : Long_Float := 0.0;            --  中位深度(米)
      Height : Long_Float := 0.0;           --  比背景鼓出多少(米)
      Au, Av : Long_Float := 0.0;           --  主轴单位向量(像素系)
      Elong : Long_Float := 1.0;            --  长轴 σ / 短轴 σ(像素当单位方块:一像素宽、ℓ 长的线 = ℓ,就是长宽比)
      Sig_U, Sig_V : Long_Float := 0.0;     --  各向 1σ(归一化画幅)
   end record;
   package Region_Vectors is new Ada.Containers.Vectors (Natural, Region);
   subtype Regions is Region_Vectors.Vector;

   type Floor_Map is record
      Per_Pixel : Buf;            --  静止两帧各像素自己抖多少
      Global : U8 := 0;           --  全图分位门槛(静止对里超过它的像素少于最少像素数)
      W, H : Natural := 0;
   end record;

   function Min_Pixels (W, H : Natural) return Natural;
   --  Keep_Edge:贴着画面边的块要不要留。世界相机里丢掉(从画面外伸进来的胳膊、整条背景带都贴边);
   --  长在动手那条胳膊上的相机里【必须留】—— 手一凑近,要抓的东西必然被画面切掉一角,丢掉它 = 最后一步瞎掉。
   function Cut (Depth : Floats; W, H : Natural; Win_Frac, Sigma_Mult : Long_Float;
                 Keep_Edge : Boolean := False) return Regions;
   --  按颜色切:颜色连成一片的算一块。细的东西(线、缝、刀口)在深度图上鼓不出来,只有这条能把它们切出来。
   --  门槛不是写死的:先量"静止时同一块地方颜色抖多少"(噪声地板),差过它的几倍才算换了一块。
   function Cut_Colour (RGB : Buf; W, H : Natural; Floor_Level : Long_Float; Min_Count : Natural) return Regions;
   --  这张画面自己的纹理有多粗:相邻像素颜色差的中位数(木纹、布纹都在这个量级)。切块的门槛要比它大才不会把纹理切成块
   function Texture_Level (RGB : Buf; W, H : Natural) return Long_Float;
   function Near_Depth (Depth : Floats; W, H : Natural; U, V, Win_Frac : Long_Float) return Long_Float;  --  NaN = 读不到
   function Null_Floor (A, B : Buf; W, H : Natural; Min_Px : Natural) return Floor_Map;
   function Moved (A, B : Buf; F : Floor_Map) return Bools;
   function Both (M1, M2 : Bools) return Bools;
   function Either (M1, M2 : Bools) return Bools;
   function Components (Mask : Bools; W, H : Natural; Min_Count : Natural) return Regions;
   --  这台相机看没看见一样东西动了(一种判法:开机认手、逐通道推、抓握通道推到头三处都用它):两次比较、不共用一帧 ——
   --  A1 对 B1、A2 对 B2(A1 / A2 = 动之前那头的两帧,B1 / B2 = 动之后那头的两帧),两次都超过静止地板的像素连成的块(≥ 最小连通块,大的在前);空 = 没看见。
   --  真东西每次都变同一片像素;渲染闪烁各帧各闪各的,几乎不会落进同一块。原来"推过去、推回来"两次比较共用推过去那一帧,那一帧自己闪一下两次都算变了
   --  (DR1 2026-09-28:无人机的抓握通道什么都不带,头顶眼里闪的 0.04% 画面被当成两瓣手指存进身体图;离线按这个判法 DR1 那段最大一块 9 像素,
   --  x5 V1B57 真手指头顶眼 6358 以上、自己那只眼 2.5 万、另一只手的眼 0–1)
   function Seen_Twice (A1, B1, A2, B2 : Buf; F : Floor_Map; W, H : Natural) return Regions;
   function Fraction (Mask : Bools) return Long_Float;
   function Max_Diff (A, B : Buf) return Natural;
   function Mean_Gray (G : Buf; W, H : Natural; R : Region) return Long_Float;   --  这一块框里的平均灰度(0..255)
   --  一张整幅掩膜(W*H,是 = 它)⇒ 它的框、像素数、形心、主轴、伸长比(同 Measure_In_Box 的算法;2026-09-26 分割仪器 SAM 出掩膜后用)。一个像素都没有 ⇒ Ok = False
   procedure Region_Of_Mask (M : Bools; W, H : Natural; R : out Region; Ok : out Boolean);
   function Quantile (F : in out Floats; Q : Long_Float) return Long_Float;
   function Region_Depth (Depth : Floats; W, H : Natural; Mask : Bools; Q : Long_Float) return Long_Float;  --  掩膜上的深度分位;NaN = 无
   function Inside (R : Region; U, V : Long_Float; W, H : Natural; Grow : Long_Float) return Boolean;
   function Is_Nan (X : Long_Float) return Boolean;
   --  一堆 8 位灰度级(0..255,按最近的一级算)分成两拨:返回分界(两级正中,下面那拨 < 分界 < 上面那拨);
   --  分不开返回 NaN。分得开 = 直方图里有一道按置信界站得住的谷(单峰的分布没有),分界 = 谷里类间方差最大(Otsu)的那一刀
   function Split (F : Floats) return Long_Float;
end Picture;
