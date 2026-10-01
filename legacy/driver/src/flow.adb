with Ada.Containers.Vectors;
with Ada.Unchecked_Deallocation;
package body Flow is
   --  热循环全在普通数组上做(容器逐元素访问慢一个量级,EF 实测一次光流让仿真等超时断线)
   type F32 is new Float;
   type Grid is array (Natural range <>) of F32;
   type Grid_Ptr is access Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Grid, Grid_Ptr);

   function Sample_At (Src : Grid; W, H : Natural; X, Y : F32) return F32 is
      Xc : constant F32 := F32'Max (0.0, F32'Min (F32 (W - 1), X));
      Yc : constant F32 := F32'Max (0.0, F32'Min (F32 (H - 1), Y));
      X0 : constant Natural := Natural (F32'Floor (Xc));
      Y0 : constant Natural := Natural (F32'Floor (Yc));
      X1 : constant Natural := Natural'Min (X0 + 1, W - 1);
      Y1 : constant Natural := Natural'Min (Y0 + 1, H - 1);
      Fx : constant F32 := Xc - F32 (X0);
      Fy : constant F32 := Yc - F32 (Y0);
      A : constant F32 := Src (Y0 * W + X0) * (1.0 - Fx) + Src (Y0 * W + X1) * Fx;
      B : constant F32 := Src (Y1 * W + X0) * (1.0 - Fx) + Src (Y1 * W + X1) * Fx;
   begin
      return A * (1.0 - Fy) + B * Fy;
   end Sample_At;

   procedure Shrink (Src : Grid; W, H : Natural; Dst : out Grid_Ptr; Nw, Nh : out Natural) is
   begin
      Nw := W / 2; Nh := H / 2;
      Dst := new Grid (0 .. Nw * Nh - 1);
      for Y in 0 .. Nh - 1 loop
         for X in 0 .. Nw - 1 loop
            declare
               A : constant Natural := 2 * X;
               B : constant Natural := 2 * Y;
            begin
               Dst (Y * Nw + X) := 0.25 * (Src (B * W + A) + Src (B * W + A + 1) + Src ((B + 1) * W + A) + Src ((B + 1) * W + A + 1));
            end;
         end loop;
      end loop;
   end Shrink;

   procedure Grow (U : Grid; W, H, Nw, Nh : Natural; Dst : out Grid_Ptr) is
   begin
      Dst := new Grid (0 .. Nw * Nh - 1);
      for Y in 0 .. Nh - 1 loop
         for X in 0 .. Nw - 1 loop
            Dst (Y * Nw + X) := Sample_At (U, W, H, F32 (X) * 0.5, F32 (Y) * 0.5) * 2.0;
         end loop;
      end loop;
   end Grow;

   procedure One_Level (I1, I2 : Grid; W, H : Natural; Du, Dv : in out Grid; Iters : Natural; Alpha2 : F32) is
      Ix, Iy, It, Au, Av : Grid_Ptr;
   begin
      Ix := new Grid'(0 .. W * H - 1 => 0.0);
      Iy := new Grid'(0 .. W * H - 1 => 0.0);
      It := new Grid'(0 .. W * H - 1 => 0.0);
      Au := new Grid'(0 .. W * H - 1 => 0.0);
      Av := new Grid'(0 .. W * H - 1 => 0.0);
      for Y in 1 .. H - 2 loop
         for X in 1 .. W - 2 loop
            declare
               I : constant Natural := Y * W + X;
            begin
               Ix (I) := 0.25 * (I1 (I + 1) - I1 (I - 1) + I2 (I + 1) - I2 (I - 1));
               Iy (I) := 0.25 * (I1 (I + W) - I1 (I - W) + I2 (I + W) - I2 (I - W));
               It (I) := I2 (I) - I1 (I);
            end;
         end loop;
      end loop;
      for K in 1 .. Iters loop
         for Y in 1 .. H - 2 loop
            for X in 1 .. W - 2 loop
               declare
                  I : constant Natural := Y * W + X;
               begin
                  --  Horn–Schunck 原文的拉普拉斯核(四邻 1/6 + 对角 1/12,和为 1,无量纲)
                  Au (I) := (Du (I - 1) + Du (I + 1) + Du (I - W) + Du (I + W)) / 6.0
                          + (Du (I - W - 1) + Du (I - W + 1) + Du (I + W - 1) + Du (I + W + 1)) / 12.0;
                  Av (I) := (Dv (I - 1) + Dv (I + 1) + Dv (I - W) + Dv (I + W)) / 6.0
                          + (Dv (I - W - 1) + Dv (I - W + 1) + Dv (I + W - 1) + Dv (I + W + 1)) / 12.0;
               end;
            end loop;
         end loop;
         for Y in 1 .. H - 2 loop
            for X in 1 .. W - 2 loop
               declare
                  I : constant Natural := Y * W + X;
                  Num : constant F32 := Ix (I) * Au (I) + Iy (I) * Av (I) + It (I);
                  Den : constant F32 := Alpha2 + Ix (I) * Ix (I) + Iy (I) * Iy (I);
                  Kf : constant F32 := Num / Den;
               begin
                  Du (I) := Au (I) - Ix (I) * Kf;
                  Dv (I) := Av (I) - Iy (I) * Kf;
               end;
            end loop;
         end loop;
      end loop;
      Free (Ix); Free (Iy); Free (It); Free (Au); Free (Av);
   end One_Level;

   function Compute (A, B : Buf; W, H, Levels, Iters : Natural) return Field is
      F : Field;
      package Grid_Vectors is new Ada.Containers.Vectors (Natural, Grid_Ptr);
      P1, P2 : Grid_Vectors.Vector;
      Ws, Hs : Ints;
      Alpha2 : F32 := 1.0;
   begin
      if W < 8 or else H < 8 or else Natural (A.Length) < W * H or else Natural (B.Length) < W * H then
         return F;
      end if;
      declare
         L1 : constant Grid_Ptr := new Grid (0 .. W * H - 1);
         L2 : constant Grid_Ptr := new Grid (0 .. W * H - 1);
      begin
         for I in 0 .. W * H - 1 loop
            L1 (I) := F32 (A.Element (I));
            L2 (I) := F32 (B.Element (I));
         end loop;
         P1.Append (L1); P2.Append (L2); Ws.Append (W); Hs.Append (H);
      end;
      for Lv in 1 .. Natural'Max (1, Levels) - 1 loop
         declare
            Cw : constant Natural := Ws.Last_Element;
            Ch : constant Natural := Hs.Last_Element;
            N1, N2 : Grid_Ptr;
            Nw, Nh : Natural;
         begin
            exit when Cw < 16 or else Ch < 16;
            Shrink (P1.Last_Element.all, Cw, Ch, N1, Nw, Nh);
            Shrink (P2.Last_Element.all, Cw, Ch, N2, Nw, Nh);
            P1.Append (N1); P2.Append (N2); Ws.Append (Nw); Hs.Append (Nh);
         end;
      end loop;
      --  α² = 这幅图梯度平方的均值(边缘主导,正好是它真实的梯度尺度);取中位数会塌到零,平滑项失效
      declare
         Sum : F32 := 0.0;
         N : F32 := 0.0;
         I0 : Grid renames P1 (0).all;
      begin
         for Y in 1 .. H - 2 loop
            for X in 1 .. W - 2 loop
               declare
                  I : constant Natural := Y * W + X;
                  Gx : constant F32 := 0.5 * (I0 (I + 1) - I0 (I - 1));
                  Gy : constant F32 := 0.5 * (I0 (I + W) - I0 (I - W));
               begin
                  Sum := Sum + Gx * Gx + Gy * Gy;
                  N := N + 1.0;
               end;
            end loop;
         end loop;
         Alpha2 := (if N < 1.0 then 1.0 else F32'Max (1.0e-3, Sum / N));
      end;
      declare
         Top : constant Natural := Natural (P1.Length) - 1;
         Cw : Natural := Ws (Top);
         Ch : Natural := Hs (Top);
         U : Grid_Ptr := new Grid'(0 .. Cw * Ch - 1 => 0.0);
         V : Grid_Ptr := new Grid'(0 .. Cw * Ch - 1 => 0.0);
      begin
         for Lv in reverse 0 .. Top loop
            declare
               Aw : constant Natural := Ws (Lv);
               Ah : constant Natural := Hs (Lv);
               Img1 : Grid renames P1 (Lv).all;
               Img2 : Grid renames P2 (Lv).all;
               Warp : Grid_Ptr := new Grid (0 .. Aw * Ah - 1);
               Du : Grid_Ptr := new Grid'(0 .. Aw * Ah - 1 => 0.0);
               Dv : Grid_Ptr := new Grid'(0 .. Aw * Ah - 1 => 0.0);
            begin
               if Lv /= Top then
                  declare
                     Nu, Nv : Grid_Ptr;
                  begin
                     Grow (U.all, Cw, Ch, Aw, Ah, Nu);
                     Grow (V.all, Cw, Ch, Aw, Ah, Nv);
                     Free (U); Free (V);
                     U := Nu; V := Nv;
                  end;
               end if;
               Cw := Aw; Ch := Ah;
               for Y in 0 .. Ah - 1 loop
                  for X in 0 .. Aw - 1 loop
                     declare
                        I : constant Natural := Y * Aw + X;
                     begin
                        Warp (I) := Sample_At (Img2, Aw, Ah, F32 (X) + U (I), F32 (Y) + V (I));
                     end;
                  end loop;
               end loop;
               One_Level (Img1, Warp.all, Aw, Ah, Du.all, Dv.all, Iters, Alpha2);
               for I in 0 .. Aw * Ah - 1 loop
                  U (I) := U (I) + Du (I);
                  V (I) := V (I) + Dv (I);
               end loop;
               Free (Warp); Free (Du); Free (Dv);
            end;
         end loop;
         F.U := Zeros (Cw * Ch); F.V := Zeros (Cw * Ch);
         for I in 0 .. Cw * Ch - 1 loop
            F.U.Replace_Element (I, Long_Float (U (I)));
            F.V.Replace_Element (I, Long_Float (V (I)));
         end loop;
         F.W := Cw; F.H := Ch;
         Free (U); Free (V);
      end;
      for Lv in 0 .. Natural (P1.Length) - 1 loop
         declare
            G1 : Grid_Ptr := P1 (Lv);
            G2 : Grid_Ptr := P2 (Lv);
         begin
            Free (G1); Free (G2);
         end;
      end loop;
      return F;
   end Compute;

   procedure Sample (F : Field; U, V, Win : Long_Float; Du, Dv : out Long_Float) is
      Cx : constant Integer := Integer (U * Long_Float (F.W));
      Cy : constant Integer := Integer (V * Long_Float (F.H));
      R : constant Integer := Integer'Max (1, Integer (Win * Long_Float (F.W)));
      Su, Sv, N : Long_Float := 0.0;
   begin
      Du := 0.0; Dv := 0.0;
      if F.W = 0 then
         return;
      end if;
      for Y in Cy - R .. Cy + R loop
         for X in Cx - R .. Cx + R loop
            if X >= 0 and then Y >= 0 and then X < F.W and then Y < F.H then
               Su := Su + F.U.Element (Y * F.W + X);
               Sv := Sv + F.V.Element (Y * F.W + X);
               N := N + 1.0;
            end if;
         end loop;
      end loop;
      if N >= 1.0 then
         Du := Su / N / Long_Float (F.W);
         Dv := Sv / N / Long_Float (F.H);
      end if;
   end Sample;
end Flow;
