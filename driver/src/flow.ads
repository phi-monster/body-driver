--  稠密光流(金字塔 Horn–Schunck):在没花纹的一大片上也解得出它在动 —— 平滑项把边缘的运动灌进肚子里。
--  层数和迭代次数是次数;平滑权重从这幅图自己的梯度尺度量出来。没有一个身体量。
with Bytes; use Bytes;
package Flow is
   type Field is record
      U, V : Floats;
      W, H : Natural := 0;
   end record;
   function Compute (A, B : Buf; W, H, Levels, Iters : Natural) return Field;
   --  某个归一化位置附近一小片的平均位移(归一化画幅单位;Win 是画幅比例)
   procedure Sample (F : Field; U, V, Win : Long_Float; Du, Dv : out Long_Float);
end Flow;
