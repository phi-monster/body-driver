with Ada.Containers.Vectors;
package body Flow is
   procedure Shrink (Src : Floats; W, H : Natural; Dst : out Floats; Nw, Nh : out Natural) is
   begin
      Nw := W / 2; Nh := H / 2;
      Dst := Zeros (Nw * Nh);
      for Y in 0 .. Nh - 1 loop
         for X in 0 .. Nw - 1 loop
            declare
               A : constant Natural := 2 * X;
               B : constant Natural := 2 * Y;
            begin
               Dst.Replace_Element (Y * Nw + X, 0.25 * (Src.Element (B * W + A) + Src.Element (B * W + A + 1) + Src.Element ((B + 1) * W + A) + Src.Element ((B + 1) * W + A + 1)));
            end;
         end loop;
      end loop;
   end Shrink;

   function Sample_At (Src : Floats; W, H : Natural; X, Y : Long_Float) return Long_Float is
      Xc : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Long_Float (W - 1), X));
      Yc : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Long_Float (H - 1), Y));
      X0 : constant Natural := Natural (Long_Float'Floor (Xc));
      Y0 : constant Natural := Natural (Long_Float'Floor (Yc));
      X1 : constant Natural := Natural'Min (X0 + 1, W - 1);
      Y1 : constant Natural := Natural'Min (Y0 + 1, H - 1);
      Fx : constant Long_Float := Xc - Long_Float (X0);
      Fy : constant Long_Float := Yc - Long_Float (Y0);
      A : constant Long_Float := Src.Element (Y0 * W + X0) * (1.0 - Fx) + Src.Element (Y0 * W + X1) * Fx;
      B : constant Long_Float := Src.Element (Y1 * W + X0) * (1.0 - Fx) + Src.Element (Y1 * W + X1) * Fx;
   begin
      return A * (1.0 - Fy) + B * Fy;
   end Sample_At;

   procedure Grow (U : Floats; W, H, Nw, Nh : Natural; Dst : out Floats) is
   begin
      Dst := Zeros (Nw * Nh);
      for Y in 0 .. Nh - 1 loop
         for X in 0 .. Nw - 1 loop
            Dst.Replace_Element (Y * Nw + X, Sample_At (U, W, H, Long_Float (X) * 0.5, Long_Float (Y) * 0.5) * 2.0);
         end loop;
      end loop;
   end Grow;

   procedure One_Level (I1, I2 : Floats; W, H : Natural; Du, Dv : in out Floats; Iters : Natural; Alpha2 : Long_Float) is
      Ix, Iy, It, Au, Av : Floats := Zeros (W * H);
   begin
      for Y in 1 .. H - 2 loop
         for X in 1 .. W - 2 loop
            declare
               I : constant Natural := Y * W + X;
            begin
               Ix.Replace_Element (I, 0.25 * (I1.Element (I + 1) - I1.Element (I - 1) + I2.Element (I + 1) - I2.Element (I - 1)));
               Iy.Replace_Element (I, 0.25 * (I1.Element (I + W) - I1.Element (I - W) + I2.Element (I + W) - I2.Element (I - W)));
               It.Replace_Element (I, I2.Element (I) - I1.Element (I));
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
                  Au.Replace_Element (I, (Du.Element (I - 1) + Du.Element (I + 1) + Du.Element (I - W) + Du.Element (I + W)) / 6.0
                          + (Du.Element (I - W - 1) + Du.Element (I - W + 1) + Du.Element (I + W - 1) + Du.Element (I + W + 1)) / 12.0);
                  Av.Replace_Element (I, (Dv.Element (I - 1) + Dv.Element (I + 1) + Dv.Element (I - W) + Dv.Element (I + W)) / 6.0
                          + (Dv.Element (I - W - 1) + Dv.Element (I - W + 1) + Dv.Element (I + W - 1) + Dv.Element (I + W + 1)) / 12.0);
               end;
            end loop;
         end loop;
         for Y in 1 .. H - 2 loop
            for X in 1 .. W - 2 loop
               declare
                  I : constant Natural := Y * W + X;
                  Num : constant Long_Float := Ix.Element (I) * Au.Element (I) + Iy.Element (I) * Av.Element (I) + It.Element (I);
                  Den : constant Long_Float := Alpha2 + Ix.Element (I) * Ix.Element (I) + Iy.Element (I) * Iy.Element (I);
                  Kf : constant Long_Float := Num / Den;
               begin
                  Du.Replace_Element (I, Au.Element (I) - Ix.Element (I) * Kf);
                  Dv.Replace_Element (I, Av.Element (I) - Iy.Element (I) * Kf);
               end;
            end loop;
         end loop;
      end loop;
   end One_Level;

   function Compute (A, B : Buf; W, H, Levels, Iters : Natural) return Field is
      F : Field;
      package Level_Vectors is new Ada.Containers.Vectors (Natural, Floats, F64_Vectors."=");
      P1, P2 : Level_Vectors.Vector;
      Ws, Hs : Ints;
      Alpha2 : Long_Float := 1.0;
   begin
      if W < 8 or else H < 8 or else Natural (A.Length) < W * H or else Natural (B.Length) < W * H then
         return F;
      end if;
      declare
         L1, L2 : Floats := Zeros (W * H);
      begin
         for I in 0 .. W * H - 1 loop
            L1.Replace_Element (I, Long_Float (A.Element (I)));
            L2.Replace_Element (I, Long_Float (B.Element (I)));
         end loop;
         P1.Append (L1); P2.Append (L2); Ws.Append (W); Hs.Append (H);
      end;
      for Lv in 1 .. Natural'Max (1, Levels) - 1 loop
         declare
            Cw : constant Natural := Ws.Last_Element;
            Ch : constant Natural := Hs.Last_Element;
            N1, N2 : Floats;
            Nw, Nh : Natural;
         begin
            exit when Cw < 16 or else Ch < 16;
            Shrink (P1.Last_Element, Cw, Ch, N1, Nw, Nh);
            Shrink (P2.Last_Element, Cw, Ch, N2, Nw, Nh);
            P1.Append (N1); P2.Append (N2); Ws.Append (Nw); Hs.Append (Nh);
         end;
      end loop;
      --  α² = 这幅图梯度平方的均值(边缘主导,正好是它真实的梯度尺度);取中位数会塌到零,平滑项失效
      declare
         Sum : Long_Float := 0.0;
         N : Long_Float := 0.0;
         I0 : constant Floats := P1 (0);
      begin
         for Y in 1 .. H - 2 loop
            for X in 1 .. W - 2 loop
               declare
                  I : constant Natural := Y * W + X;
                  Gx : constant Long_Float := 0.5 * (I0.Element (I + 1) - I0.Element (I - 1));
                  Gy : constant Long_Float := 0.5 * (I0.Element (I + W) - I0.Element (I - W));
               begin
                  Sum := Sum + Gx * Gx + Gy * Gy;
                  N := N + 1.0;
               end;
            end loop;
         end loop;
         Alpha2 := (if N < 1.0 then 1.0 else Long_Float'Max (1.0e-3, Sum / N));
      end;
      declare
         Top : constant Natural := Natural (P1.Length) - 1;
         Cw : Natural := Ws.Element (Top);
         Ch : Natural := Hs.Element (Top);
         U : Floats := Zeros (Cw * Ch);
         V : Floats := Zeros (Cw * Ch);
      begin
         for Lv in reverse 0 .. Top loop
            declare
               Aw : constant Natural := Ws.Element (Lv);
               Ah : constant Natural := Hs.Element (Lv);
               Img1 : constant Floats := P1 (Lv);
               Img2 : constant Floats := P2 (Lv);
               Warp : Floats := Zeros (Aw * Ah);
               Du, Dv : Floats := Zeros (Aw * Ah);
            begin
               if Lv /= Top then
                  declare
                     Nu, Nv : Floats;
                  begin
                     Grow (U, Cw, Ch, Aw, Ah, Nu);
                     Grow (V, Cw, Ch, Aw, Ah, Nv);
                     U := Nu; V := Nv;
                  end;
               end if;
               Cw := Aw; Ch := Ah;
               for Y in 0 .. Ah - 1 loop
                  for X in 0 .. Aw - 1 loop
                     declare
                        I : constant Natural := Y * Aw + X;
                     begin
                        Warp.Replace_Element (I, Sample_At (Img2, Aw, Ah, Long_Float (X) + U.Element (I), Long_Float (Y) + V.Element (I)));
                     end;
                  end loop;
               end loop;
               One_Level (Img1, Warp, Aw, Ah, Du, Dv, Iters, Alpha2);
               for I in 0 .. Aw * Ah - 1 loop
                  U.Replace_Element (I, U.Element (I) + Du.Element (I));
                  V.Replace_Element (I, V.Element (I) + Dv.Element (I));
               end loop;
            end;
         end loop;
         F.U := U; F.V := V; F.W := Cw; F.H := Ch;
      end;
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
