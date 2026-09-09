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
      Top : Long_Float := 0.0;              --  最靠近相机的那一档深度(米)= 这块的顶面
      Height : Long_Float := 0.0;           --  比背景鼓出多少(米)
      Au, Av : Long_Float := 0.0;           --  主轴单位向量(像素系)
      Elong : Long_Float := 1.0;            --  长轴 σ / 短轴 σ
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
   --  Keep_Edge = False:贴到任何一条画面边的块都丢(第三方相机里从画面外伸进来的胳膊、桌沿、墙都贴边)。
   --  Keep_Edge = True:只丢【横跨整幅】的(左右都贴边或上下都贴边)—— 凑近了要抓的东西必然被画面切掉一角,
   --  严格规则下它会整块消失。只在严格规则一块都没切出来时才放宽。
   function Cut (Depth : Floats; W, H : Natural; Win_Frac, Sigma_Mult : Long_Float; Keep_Edge : Boolean := False) return Regions;
   --  按颜色切:颜色连成一片的算一块。细的东西(线、缝、刀口)在深度图上鼓不出来,只有这条能把它们切出来。
   --  门槛不是写死的:先量"静止时同一块地方颜色抖多少"(噪声地板),差过它的几倍才算换了一块。
   function Cut_Colour (RGB : Buf; W, H : Natural; Floor_Level : Long_Float; Min_Count : Natural) return Regions;
   --  这张画面自己的纹理有多粗:相邻像素颜色差的中位数(木纹、布纹都在这个量级)。切块的门槛要比它大才不会把纹理切成块
   function Texture_Level (RGB : Buf; W, H : Natural) return Long_Float;
   procedure Mean_Colour (RGB : Buf; W, H : Natural; R : Region; Cr, Cg, Cb : out Long_Float);
   function Region_Mask (Depth : Floats; W, H : Natural; R : Region) return Bools;
   function Near_Depth (Depth : Floats; W, H : Natural; U, V, Win_Frac : Long_Float) return Long_Float;  --  NaN = 读不到
   function Null_Floor (A, B : Buf; W, H : Natural; Min_Px : Natural) return Floor_Map;
   function Moved (A, B : Buf; F : Floor_Map) return Bools;
   function Both (M1, M2 : Bools) return Bools;
   function Either (M1, M2 : Bools) return Bools;
   function Components (Mask : Bools; W, H : Natural; Min_Count : Natural) return Regions;
   function Fraction (Mask : Bools) return Long_Float;
   function Max_Diff (A, B : Buf) return Natural;
   function Mean_Gray (G : Buf; W, H : Natural; R : Region) return Long_Float;   --  这一块框里的平均灰度(0..255)
   function Quantile (F : in out Floats; Q : Long_Float) return Long_Float;
   function Region_Depth (Depth : Floats; W, H : Natural; Mask : Bools; Q : Long_Float) return Long_Float;  --  掩膜上的深度分位;NaN = 无
   function Inside (R : Region; U, V : Long_Float; W, H : Natural; Grow : Long_Float) return Boolean;
   --  两块【挨着没有】:画面上的框贴住(留一条缝的宽容),且顶面的远近对得上(不是一前一后错开)。
   --  这是"动作词表"的唯一原始事实:谁和谁挨着,以及这个关系什么时候变。不需要知道它们是什么东西。
   function Adjacent (A, B : Region; W, H : Natural; Gap : Long_Float) return Boolean;
   function Is_Nan (X : Long_Float) return Boolean;
   --  一堆数分成两拨(Otsu):返回分界;分不开(单峰)返回 NaN
   function Split (F : Floats) return Long_Float;
end Picture;
