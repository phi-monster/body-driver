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
   function Cut (Depth : Floats; W, H : Natural; Win_Frac, Sigma_Mult : Long_Float) return Regions;
   function Region_Mask (Depth : Floats; W, H : Natural; R : Region) return Bools;
   function Near_Depth (Depth : Floats; W, H : Natural; U, V, Win_Frac : Long_Float) return Long_Float;  --  NaN = 读不到
   function Null_Floor (A, B : Buf; W, H : Natural; Min_Px : Natural) return Floor_Map;
   function Moved (A, B : Buf; F : Floor_Map) return Bools;
   function Both (M1, M2 : Bools) return Bools;
   function Either (M1, M2 : Bools) return Bools;
   function Components (Mask : Bools; W, H : Natural; Min_Count : Natural) return Regions;
   function Fraction (Mask : Bools) return Long_Float;
   function Max_Diff (A, B : Buf) return Natural;
   function Quantile (F : in out Floats; Q : Long_Float) return Long_Float;
   function Region_Depth (Depth : Floats; W, H : Natural; Mask : Bools; Q : Long_Float) return Long_Float;  --  掩膜上的深度分位;NaN = 无
   function Inside (R : Region; U, V : Long_Float; W, H : Natural; Grow : Long_Float) return Boolean;
   function Is_Nan (X : Long_Float) return Boolean;
end Picture;
