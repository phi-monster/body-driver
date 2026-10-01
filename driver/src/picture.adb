with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Interfaces;
package body Picture is
   function To_LF is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, Long_Float);
   NaN : constant Long_Float := To_LF (16#7FF8000000000000#);

   function Is_Nan (X : Long_Float) return Boolean is (X /= X);

   --  全仓同一个置信倍数:3σ(单侧尾 1 − Φ(3) ≈ 0.135%;同 act.adb 的 Sigma_Mult、geom 的 3 倍门)
   Conf_K : constant := 3.0;

   --  标准正态的上侧尾 1 − Φ(z),z ≥ 0。Φ(z) − ½ = φ(z)·(z + z³/3 + z⁵/(3·5) + …):一项一项加,加到再加和也不变为止
   function Upper_Tail (Z : Long_Float) return Long_Float is
      Term : Long_Float := Z;
      Sum : Long_Float := 0.0;
      Odd : Long_Float := 1.0;   --  这一项分母里最后那个奇数
   begin
      while Sum + Term /= Sum loop
         Sum := Sum + Term;
         Odd := Odd + 2.0;
         Term := Term * Z * Z / Odd;
      end loop;
      return 0.5 - Exp (-0.5 * Z * Z) / Sqrt (2.0 * Ada.Numerics.Pi) * Sum;
   end Upper_Tail;

   --  标准正态的分位 Φ⁻¹(P),½ ≤ P < 1:先把上界翻倍到够,再二分,分到中点和一头重合(再分也不变)为止
   function Normal_Quantile (P : Long_Float) return Long_Float is
      Lo : Long_Float := 0.0;
      Hi : Long_Float := 1.0;
      Mid : Long_Float;
   begin
      while Upper_Tail (Hi) > 1.0 - P loop
         Hi := 2.0 * Hi;
      end loop;
      loop
         Mid := 0.5 * (Lo + Hi);
         exit when Mid <= Lo or else Mid >= Hi;
         if Upper_Tail (Mid) > 1.0 - P then
            Lo := Mid;
         else
            Hi := Mid;
         end if;
      end loop;
      return Mid;
   end Normal_Quantile;

   --  一块像素的形状:形心、各向 σ、主轴、伸长比(三处量块的地方共用这一份)。
   --  每个像素当成一个单位方块,不当成一个点:二阶矩各加 1/12(单位方块沿一条轴的方差,数学)。
   --  这样一像素宽、ℓ 长的线伸长比 = ℓ,两像素宽的 = ℓ/2(就是长宽比),短轴永远不为零。
   --  原来按点算:一像素宽的线短轴为零,伸长比记成哨兵 1000,两像素宽的约 ℓ/√3 —— 进了 act.adb 的"像不像",宽一个像素就被当成完全不同的东西
   procedure Fill_Shape (R : in out Region; Cnt : Natural; Sx, Sy, Sxx, Syy, Sxy : Long_Float; W, H : Natural) is
      C : constant Long_Float := Long_Float (Cnt);
      Mx : constant Long_Float := Sx / C;
      My : constant Long_Float := Sy / C;
      Pix : constant Long_Float := 1.0 / 12.0;
      Vxx : constant Long_Float := Long_Float'Max (0.0, Sxx / C - Mx * Mx) + Pix;
      Vyy : constant Long_Float := Long_Float'Max (0.0, Syy / C - My * My) + Pix;
      Vxy : constant Long_Float := Sxy / C - Mx * My;
      Tr : constant Long_Float := Vxx + Vyy;
      Det : constant Long_Float := Vxx * Vyy - Vxy * Vxy;
      Disc : constant Long_Float := Long_Float'Max (0.0, 0.25 * Tr * Tr - Det);
      L1 : constant Long_Float := 0.5 * Tr + Sqrt (Disc);
      --  小特征值 ≥ 1/12(协方差加上 1/12 倍单位阵),夹在这个数学下界上只防舍入
      L2 : constant Long_Float := Long_Float'Max (Pix, 0.5 * Tr - Sqrt (Disc));
      Ax, Ay : Long_Float;
   begin
      R.Count := Cnt;
      R.Cu := Mx / Long_Float (W);
      R.Cv := My / Long_Float (H);
      R.Sig_U := Sqrt (Vxx) / Long_Float (W);
      R.Sig_V := Sqrt (Vyy) / Long_Float (H);
      --  主轴 = 协方差最大特征值的特征向量(三种情形都给出非零向量)
      if abs Vxy > 1.0e-12 then
         Ax := L1 - Vyy; Ay := Vxy;
      elsif Vxx >= Vyy then
         Ax := 1.0; Ay := 0.0;
      else
         Ax := 0.0; Ay := 1.0;
      end if;
      R.Au := Ax / Sqrt (Ax * Ax + Ay * Ay);
      R.Av := Ay / Sqrt (Ax * Ax + Ay * Ay);
      R.Elong := Sqrt (L1 / L2);
   end Fill_Shape;

   function Min_Pixels (W, H : Natural) return Natural is
      --  3e-5 是画幅的比例(无量纲):比这还小的斑块读不出形状
      V : constant Long_Float := Long_Float (W * H) * 3.0e-5;
   begin
      return Natural'Max (4, Natural (Long_Float'Ceiling (V)));
   end Min_Pixels;

   function Quantile (F : in out Floats; Q : Long_Float) return Long_Float is
      N : constant Natural := Natural (F.Length);
   begin
      if N = 0 then
         return NaN;
      end if;
      --  简单选择:插入排序对小串;大串用 nth_element 的粗版(全排序,O(n log n) 足够)
      declare
         package Sorter is new F64_Vectors.Generic_Sorting;
         Idx : Natural;
      begin
         Sorter.Sort (F);
         Idx := Natural (Long_Float (N - 1) * Long_Float'Max (0.0, Long_Float'Min (1.0, Q)));
         return F.Element (Idx);
      end;
   end Quantile;

   --  一维滑窗极值(单调队列),窗口 [c-r, c+r];无读数按 Empty 参与。
   procedure Slide (Src : Floats; W, H, R : Natural; Horizontal, Take_Max : Boolean; Empty : Long_Float; Dst : in out Floats) is
      Outer : constant Natural := (if Horizontal then H else W);
      Inner : constant Natural := (if Horizontal then W else H);
      Dq : array (0 .. Inner) of Natural;
      Head, Tail : Natural;
      function At_Idx (O, K : Natural) return Natural is (if Horizontal then O * W + K else K * W + O);
      function Val (O, K : Natural) return Long_Float is
         V : constant Long_Float := Src.Element (At_Idx (O, K));
      begin
         return (if Is_Nan (V) then Empty else V);
      end Val;
      procedure Push (O, K : Natural) is
      begin
         while Tail > Head loop
            declare
               Last : constant Natural := Dq (Tail - 1);
            begin
               if (Take_Max and then Val (O, Last) <= Val (O, K)) or else (not Take_Max and then Val (O, Last) >= Val (O, K)) then
                  Tail := Tail - 1;
               else
                  exit;
               end if;
            end;
         end loop;
         Dq (Tail) := K;
         Tail := Tail + 1;
      end Push;
   begin
      if Inner = 0 then
         return;
      end if;
      for O in 0 .. Outer - 1 loop
         Head := 0; Tail := 0;
         for K in 0 .. Natural'Min (R, Inner - 1) loop
            Push (O, K);
         end loop;
         for C in 0 .. Inner - 1 loop
            if C > 0 and then C + R < Inner then
               Push (O, C + R);
            end if;
            declare
               Left : constant Natural := (if C > R then C - R else 0);
            begin
               while Tail > Head and then Dq (Head) < Left loop
                  Head := Head + 1;
               end loop;
            end;
            if Tail > Head then
               Dst.Replace_Element (At_Idx (O, C), Val (O, Dq (Head)));
            end if;
         end loop;
      end loop;
   end Slide;

   function Texture_Level (RGB : Buf; W, H : Natural) return Long_Float is
      Ds : Floats;
      I : Natural := 0;
      Step : constant Natural := Natural'Max (1, (W * H) / 20000);   --  抽样步长(次数,无量纲)
   begin
      if Natural (RGB.Length) < W * H * 3 or else W < 2 then
         return 0.0;
      end if;
      while I < W * H - 1 loop
         if (I + 1) mod W /= 0 then
            Ds.Append (Long_Float'Max (Long_Float'Max (abs (Long_Float (RGB.Element (3 * I)) - Long_Float (RGB.Element (3 * I + 3))),
                                                       abs (Long_Float (RGB.Element (3 * I + 1)) - Long_Float (RGB.Element (3 * I + 4)))),
                                       abs (Long_Float (RGB.Element (3 * I + 2)) - Long_Float (RGB.Element (3 * I + 5)))));
         end if;
         I := I + Step;
      end loop;
      if Natural (Ds.Length) < 16 then
         return 0.0;
      end if;
      return Quantile (Ds, 0.5);
   end Texture_Level;

   --  颜色连片:和右边、下面的邻居颜色差在门槛内就连成一块(并查集式的两遍扫描,零依赖)
   function Cut_Colour (RGB : Buf; W, H : Natural; Floor_Level : Long_Float; Min_Count : Natural) return Regions is
      N : constant Natural := W * H;
      Out_R : Regions;
      Lab : Ints;
      procedure Find (X : in out Integer) is
      begin
         while Lab (X) /= X loop
            X := Lab (X);
         end loop;
      end Find;
      procedure Union (A, B : Integer) is
         Ra : Integer := A;
         Rb : Integer := B;
      begin
         Find (Ra); Find (Rb);
         if Ra /= Rb then
            Lab.Replace_Element (Natural (Integer'Max (Ra, Rb)), Integer'Min (Ra, Rb));
         end if;
      end Union;
      function Diff (I, J : Natural) return Long_Float is
        (Long_Float'Max (Long_Float'Max (abs (Long_Float (RGB.Element (3 * I)) - Long_Float (RGB.Element (3 * J))),
                                         abs (Long_Float (RGB.Element (3 * I + 1)) - Long_Float (RGB.Element (3 * J + 1)))),
                         abs (Long_Float (RGB.Element (3 * I + 2)) - Long_Float (RGB.Element (3 * J + 2)))));
   begin
      if W = 0 or else H = 0 or else Natural (RGB.Length) < N * 3 then
         return Out_R;
      end if;
      Lab := Int_Vectors.To_Vector (0, Ada.Containers.Count_Type (N));
      for I in 0 .. N - 1 loop
         Lab.Replace_Element (I, I);
      end loop;
      for Y in 0 .. H - 1 loop
         for X in 0 .. W - 1 loop
            declare
               I : constant Natural := Y * W + X;
            begin
               if X + 1 < W and then Diff (I, I + 1) <= Floor_Level then
                  Union (I, I + 1);
               end if;
               if Y + 1 < H and then Diff (I, I + W) <= Floor_Level then
                  Union (I, I + W);
               end if;
            end;
         end loop;
      end loop;
      --  收成块:一遍扫描,每个根各自累计像素数、外框、一阶二阶矩(不许对每个根再扫一遍全图 —— 那是平方级)
      declare
         type Acc is record
            Cnt : Natural := 0;
            X0, Y0, X1, Y1 : Natural := 0;
            Sx, Sy, Sxx, Syy, Sxy : Long_Float := 0.0;
            Slot : Integer := -1;
         end record;
         package Acc_Vectors is new Ada.Containers.Vectors (Natural, Acc);
         Accs : Acc_Vectors.Vector;
         Slot_Of : Ints := Int_Vectors.To_Vector (-1, Ada.Containers.Count_Type (N));
      begin
         for Y in 0 .. H - 1 loop
            for X in 0 .. W - 1 loop
               declare
                  I : constant Natural := Y * W + X;
                  R : Integer := I;
                  Sl : Integer;
               begin
                  Find (R);
                  Sl := Slot_Of (Natural (R));
                  if Sl < 0 then
                     declare
                        A0 : Acc;
                     begin
                        A0.X0 := X; A0.Y0 := Y; A0.X1 := X; A0.Y1 := Y;
                        Accs.Append (A0);
                     end;
                     Sl := Integer (Accs.Length) - 1;
                     Slot_Of.Replace_Element (Natural (R), Sl);
                  end if;
                  declare
                     A1 : Acc := Accs (Natural (Sl));
                     Fx : constant Long_Float := Long_Float (X);
                     Fy : constant Long_Float := Long_Float (Y);
                  begin
                     A1.Cnt := A1.Cnt + 1;
                     A1.X0 := Natural'Min (A1.X0, X); A1.Y0 := Natural'Min (A1.Y0, Y);
                     A1.X1 := Natural'Max (A1.X1, X); A1.Y1 := Natural'Max (A1.Y1, Y);
                     A1.Sx := A1.Sx + Fx; A1.Sy := A1.Sy + Fy;
                     A1.Sxx := A1.Sxx + Fx * Fx; A1.Syy := A1.Syy + Fy * Fy; A1.Sxy := A1.Sxy + Fx * Fy;
                     Accs.Replace_Element (Natural (Sl), A1);
                  end;
               end;
            end loop;
         end loop;
         for A1 of Accs loop
            if A1.Cnt >= Natural'Max (1, Min_Count) then
               declare
                  R : Region;
               begin
                  R.X0 := A1.X0; R.Y0 := A1.Y0; R.X1 := A1.X1; R.Y1 := A1.Y1;
                  Fill_Shape (R, A1.Cnt, A1.Sx, A1.Sy, A1.Sxx, A1.Syy, A1.Sxy, W, H);
                  Out_R.Append (R);
               end;
            end if;
         end loop;
      end;
      declare
         function Bigger (A, B : Region) return Boolean is (A.Count > B.Count);
         package Sorter is new Region_Vectors.Generic_Sorting (Bigger);
      begin
         Sorter.Sort (Out_R);
      end;
      return Out_R;
   end Cut_Colour;

   function Cut (Depth : Floats; W, H : Natural; Win_Frac, Sigma_Mult : Long_Float;
                 Keep_Edge : Boolean := False) return Regions is
      Out_R : Regions;
      N : constant Natural := W * H;
   begin
      if W = 0 or else H = 0 or else Natural (Depth.Length) < N then
         return Out_R;
      end if;
      declare
         R : constant Natural := Natural'Max (2, Natural (Long_Float (W) * Win_Frac));
         Big : constant Long_Float := 1.0e30;
         D1, D2, E1, Back, Bump : Floats := Filled (N, NaN);
         Samples : Floats;
         Step : constant Natural := Natural'Max (1, N / 20000);
         Mid, Sigma, Gate : Long_Float;
      begin
         --  闭运算 = 膨胀(窗内最大深度)再腐蚀(窗内最小深度):把比窗口小的坑填平 = 没放东西时的背景面
         Slide (Depth, W, H, R, True, True, -Big, D1);
         Slide (D1, W, H, R, False, True, -Big, D2);
         Slide (D2, W, H, R, True, False, Big, E1);
         Slide (E1, W, H, R, False, False, Big, Back);
         for I in 0 .. N - 1 loop
            declare
               Z : constant Long_Float := Depth.Element (I);
               B : constant Long_Float := Back.Element (I);
            begin
               if not Is_Nan (Z) and then Z > 1.0e-6 and then not Is_Nan (B) and then abs B < Big then
                  Bump.Replace_Element (I, B - Z);
               end if;
            end;
         end loop;
         declare
            I : Natural := 0;
         begin
            while I < N loop
               if not Is_Nan (Bump.Element (I)) then
                  Samples.Append (Bump.Element (I));
               end if;
               I := I + Step;
            end loop;
         end;
         if Natural (Samples.Length) < 16 then
            return Out_R;
         end if;
         Mid := Quantile (Samples, 0.5);
         declare
            Absd : Floats;
            --  排序里的位置(分位,无量纲)
            Quantiles : constant array (1 .. 4) of Long_Float := [0.5, 0.75, 0.9, 0.99];
         begin
            for V of Samples loop
               Absd.Append (abs (V - Mid));
            end loop;
            --  中位绝对偏差可能是 0(量化)⇒ 往上取分位数直到拿到正的尺度。
            --  正态下 |x − 中位| 的 q 分位 = σ·Φ⁻¹((1+q)/2) ⇒ σ = 分位 ÷ Φ⁻¹((1+q)/2):q = 0.5 时就是常说的 1.4826 × 中位绝对偏差。
            --  🔴 原来每一档都乘 1.4826(只对 q = 0.5 成立):q = 0.75 / 0.9 / 0.99 时 σ 被放大 1.71 / 2.43 / 3.82 倍,门偏高,小东西切不出来
            Sigma := 0.0;
            for Q of Quantiles loop
               declare
                  V : constant Long_Float := Quantile (Absd, Q) / Normal_Quantile (0.5 * (1.0 + Q));
               begin
                  if V > 0.0 then
                     Sigma := V;
                     exit;
                  end if;
               end;
            end loop;
         end;
         if not (Sigma > 0.0) then
            return Out_R;
         end if;
         Gate := Mid + Sigma_Mult * Sigma;
         declare
            Mask : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
            Comps : Regions;
         begin
            for I in 0 .. N - 1 loop
               if not Is_Nan (Bump.Element (I)) and then Bump.Element (I) > Gate then
                  Mask.Replace_Element (I, True);
               end if;
            end loop;
            Comps := Components (Mask, W, H, Min_Pixels (W, H));
            for C of Comps loop
               declare
                  Rg : Region := C;
                  Ds, Hs : Floats;
               begin
                  --  贴着画面边的块丢掉:整条背景带、细缝、我自己的胳膊都贴边。
                  --  🔴 但在长着动手那条胳膊的相机里不许丢:手一凑近,要抓的东西必然被画面切掉一角
                  --  (GM 实测:球的下沿正好压在画面最后一行 ⇒ 整块消失 ⇒ 脑连它的名字都点不出来)。
                  if Keep_Edge
                    or else (Rg.X0 > 0 and then Rg.Y0 > 0 and then Rg.X1 + 1 < W and then Rg.Y1 + 1 < H)
                  then
                     for Y in Rg.Y0 .. Rg.Y1 loop
                        for X in Rg.X0 .. Rg.X1 loop
                           if Mask.Element (Y * W + X) then
                              Ds.Append (Depth.Element (Y * W + X));
                              Hs.Append (Bump.Element (Y * W + X));
                           end if;
                        end loop;
                     end loop;
                     Rg.Depth := Quantile (Ds, 0.5);
                     Rg.Height := Quantile (Hs, 0.5);
                     Out_R.Append (Rg);
                  end if;
               end;
            end loop;
         end;
      end;
      --  按像素数从多到少
      declare
         function Bigger (A, B : Region) return Boolean is (A.Count > B.Count);
         package Sorter is new Region_Vectors.Generic_Sorting (Bigger);
      begin
         Sorter.Sort (Out_R);
      end;
      return Out_R;
   end Cut;

   function Near_Depth (Depth : Floats; W, H : Natural; U, V, Win_Frac : Long_Float) return Long_Float is
      Rw : constant Natural := Natural'Max (1, Natural (Long_Float (W) * Win_Frac));
      Cx : constant Integer := Integer (U * Long_Float (W));
      Cy : constant Integer := Integer (V * Long_Float (H));
      Vals : Floats;
   begin
      for Y in Cy - Integer (Rw) .. Cy + Integer (Rw) loop
         for X in Cx - Integer (Rw) .. Cx + Integer (Rw) loop
            if X >= 0 and then Y >= 0 and then X < W and then Y < H then
               declare
                  Z : constant Long_Float := Depth.Element (Y * W + X);
               begin
                  if not Is_Nan (Z) and then Z > 1.0e-6 then
                     Vals.Append (Z);
                  end if;
               end;
            end if;
         end loop;
      end loop;
      if Vals.Is_Empty then
         return NaN;
      end if;
      return Quantile (Vals, 0.25);     --  近侧:四分位里靠近的那一档(块的顶面,不是它旁边的桌面)
   end Near_Depth;

   function Null_Floor (A, B : Buf; W, H : Natural; Min_Px : Natural) return Floor_Map is
      F : Floor_Map;
      Hist : array (0 .. 255) of Natural := [others => 0];
      N : constant Natural := W * H;
      Above : Natural := 0;
   begin
      F.W := W; F.H := H;
      F.Per_Pixel.Reserve_Capacity (Ada.Containers.Count_Type (N));
      for I in 0 .. N - 1 loop
         declare
            D : constant Natural := abs (Integer (A.Element (I)) - Integer (B.Element (I)));
         begin
            F.Per_Pixel.Append (U8 (D));
            Hist (D) := Hist (D) + 1;
         end;
      end loop;
      --  全图门槛:静止那一对里超过它的像素少于最少像素数(一团 ≥ 最少像素的斑块不可能由静止噪声造出来)
      F.Global := 0;
      for D in reverse 0 .. 255 loop
         if Above >= Min_Px then
            F.Global := U8 (Natural'Min (255, D + 1));
            exit;
         end if;
         Above := Above + Hist (D);
      end loop;
      return F;
   end Null_Floor;

   function Moved (A, B : Buf; F : Floor_Map) return Bools is
      N : constant Natural := Natural'Min (Natural (A.Length), Natural (B.Length));
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
   begin
      for I in 0 .. N - 1 loop
         declare
            D : constant Natural := abs (Integer (A.Element (I)) - Integer (B.Element (I)));
            Gate : constant Natural := Natural'Max (Natural (F.Global), (if I < Natural (F.Per_Pixel.Length) then Natural (F.Per_Pixel.Element (I)) else 0));
         begin
            M.Replace_Element (I, D > Gate);
         end;
      end loop;
      return M;
   end Moved;

   function Seen_Twice (A1, B1, A2, B2 : Buf; F : Floor_Map; W, H : Natural) return Regions is
     (Components (Both (Moved (A1, B1, F), Moved (A2, B2, F)), W, H, Min_Pixels (W, H)));

   function Both (M1, M2 : Bools) return Bools is
      N : constant Natural := Natural'Min (Natural (M1.Length), Natural (M2.Length));
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
   begin
      for I in 0 .. N - 1 loop
         M.Replace_Element (I, M1.Element (I) and then M2.Element (I));
      end loop;
      return M;
   end Both;

   function Either (M1, M2 : Bools) return Bools is
      N : constant Natural := Natural'Min (Natural (M1.Length), Natural (M2.Length));
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
   begin
      for I in 0 .. N - 1 loop
         M.Replace_Element (I, M1.Element (I) or else M2.Element (I));
      end loop;
      return M;
   end Either;

   function Fraction (Mask : Bools) return Long_Float is
      C : Natural := 0;
   begin
      for B of Mask loop
         if B then
            C := C + 1;
         end if;
      end loop;
      if Mask.Is_Empty then
         return 0.0;
      end if;
      return Long_Float (C) / Long_Float (Mask.Length);
   end Fraction;

   function Mean_Gray (G : Buf; W, H : Natural; R : Region) return Long_Float is
      S : Long_Float := 0.0;
      N : Natural := 0;
   begin
      if Natural (G.Length) < W * H then
         return -1.0;
      end if;
      for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
         for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
            S := S + Long_Float (G.Element (Y * W + X));
            N := N + 1;
         end loop;
      end loop;
      return (if N > 0 then S / Long_Float (N) else -1.0);
   end Mean_Gray;

   function Max_Diff (A, B : Buf) return Natural is
      N : constant Natural := Natural'Min (Natural (A.Length), Natural (B.Length));
      M : Natural := 0;
   begin
      for I in 0 .. N - 1 loop
         M := Natural'Max (M, abs (Integer (A.Element (I)) - Integer (B.Element (I))));
      end loop;
      return M;
   end Max_Diff;

   function Components (Mask : Bools; W, H : Natural; Min_Count : Natural) return Regions is
      N : constant Natural := W * H;
      Label : Ints := Int_Vectors.To_Vector (-1, Ada.Containers.Count_Type (N));
      Stack : Ints;
      Out_R : Regions;
   begin
      if Natural (Mask.Length) < N then
         return Out_R;
      end if;
      for Start in 0 .. N - 1 loop
         if Mask.Element (Start) and then Label.Element (Start) < 0 then
            declare
               Id : constant Integer := Integer (Out_R.Length);
               Cnt : Natural := 0;
               Sx, Sy, Sxx, Syy, Sxy : Long_Float := 0.0;
               X0 : Natural := W; Y0 : Natural := H; X1 : Natural := 0; Y1 : Natural := 0;
            begin
               Stack.Clear;
               Stack.Append (Start);
               Label.Replace_Element (Start, Id);
               while not Stack.Is_Empty loop
                  declare
                     I : constant Natural := Stack.Last_Element;
                     X : constant Natural := I mod W;
                     Y : constant Natural := I / W;
                     procedure Visit (J : Natural) is
                     begin
                        if Mask.Element (J) and then Label.Element (J) < 0 then
                           Label.Replace_Element (J, Id);
                           Stack.Append (J);
                        end if;
                     end Visit;
                  begin
                     Stack.Delete_Last;
                     Cnt := Cnt + 1;
                     Sx := Sx + Long_Float (X); Sy := Sy + Long_Float (Y);
                     Sxx := Sxx + Long_Float (X) * Long_Float (X);
                     Syy := Syy + Long_Float (Y) * Long_Float (Y);
                     Sxy := Sxy + Long_Float (X) * Long_Float (Y);
                     X0 := Natural'Min (X0, X); X1 := Natural'Max (X1, X);
                     Y0 := Natural'Min (Y0, Y); Y1 := Natural'Max (Y1, Y);
                     if X > 0 then
                        Visit (I - 1);
                     end if;
                     if X + 1 < W then
                        Visit (I + 1);
                     end if;
                     if Y > 0 then
                        Visit (I - W);
                     end if;
                     if Y + 1 < H then
                        Visit (I + W);
                     end if;
                  end;
               end loop;
               if Cnt >= Natural'Max (1, Min_Count) then
                  declare
                     R : Region;
                  begin
                     R.X0 := X0; R.Y0 := Y0; R.X1 := X1; R.Y1 := Y1;
                     Fill_Shape (R, Cnt, Sx, Sy, Sxx, Syy, Sxy, W, H);
                     Out_R.Append (R);
                  end;
               end if;
            end;
         end if;
      end loop;
      --  按像素数从多到少(和 Cut 一样)。🔴 以前不排:调用方全都拿 (0) 当"最大的一块",实际拿到的是扫描线里最先碰到的那块
      --  ⇒ EL:合空时一个 4×4 的碎点被当成一根手指,握区中心算到画面左中,球被推去了错地方
      declare
         function Bigger (A, B : Region) return Boolean is (A.Count > B.Count);
         package Sorter is new Region_Vectors.Generic_Sorting (Bigger);
      begin
         Sorter.Sort (Out_R);
      end;
      return Out_R;
   end Components;

   function Region_Depth (Depth : Floats; W, H : Natural; Mask : Bools; Q : Long_Float) return Long_Float is
      Vals : Floats;
      N : constant Natural := Natural'Min (W * H, Natural'Min (Natural (Depth.Length), Natural (Mask.Length)));
   begin
      for I in 0 .. N - 1 loop
         if Mask.Element (I) and then not Is_Nan (Depth.Element (I)) and then Depth.Element (I) > 1.0e-6 then
            Vals.Append (Depth.Element (I));
         end if;
      end loop;
      if Vals.Is_Empty then
         return NaN;
      end if;
      return Quantile (Vals, Q);
   end Region_Depth;

   --  一堆 8 位灰度级分两拨。
   --  ① 一级一格的直方图:8 位灰度就是 256 级(图像格式)。原来按量程分 64 格,门的分辨率只有量程 / 64。
   --  ② 分不分得开 = 有没有一道真谷:左边一段、右边一段、中间夹着一段,中间那段的平均密度比两边都低 ——
   --     而且是按置信界比:两边按密度的下界、中间按上界。单峰的分布做不到这一条(密度先升后降,夹在中间的那段不可能比两边都低),
   --     所以置信界一成立,"有谷"就不是抽样抖出来的。置信界 = 每一段样本份额的伯恩斯坦界;
   --     8 位的所有区间(256 × 257 / 2 段)要一起成立,总的置信度用全仓同一个 3σ 的单侧尾 1 − Φ(3),平摊到每一段。
   --  ③ 返回:落在真谷里的所有切口中类间方差最大(Otsu)的那一刀,放在两级正中;一道真谷都没有(单峰)⇒ NaN。
   --  🔴 原来的判法是"类间方差 ≥ 总方差一半":单峰高斯 0.64、均匀 0.75、灰度噪声的半正态 0.67 全过,几乎从不说分不开
   --  (审计 G4 仿真量过);反过来桌面上只占 1% 的白东西(一眼分得开)只有 0.45,被拒
   function Split (F : Floats) return Long_Float is
      Levels : constant := 256;                                         --  8 位灰度的级数(图像格式)
      H : array (0 .. Levels - 1) of Long_Float := [others => 0.0];
      Cum : array (0 .. Levels) of Long_Float := [others => 0.0];      --  Cum (K) = 级 0 .. K − 1 一共几个样本
      N : Long_Float := 0.0;
      Lo : Natural := Levels - 1;
      Hi : Natural := 0;
   begin
      for X of F loop
         if not Is_Nan (X) then
            declare
               K : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Levels - 1), X)));
            begin
               H (K) := H (K) + 1.0;
               N := N + 1.0;
               Lo := Natural'Min (Lo, K);
               Hi := Natural'Max (Hi, K);
            end;
         end if;
      end loop;
      if Hi <= Lo then
         return NaN;   --  没有样本,或者全在同一级
      end if;
      for K in 0 .. Levels - 1 loop
         Cum (K + 1) := Cum (K) + H (K);
      end loop;
      declare
         --  每一段的失败概率 = 总的(3σ 单侧尾)÷ 段数,双侧 ⇒ Ln = ln(2 × 段数 ÷ 总失败概率)
         Ln : constant Long_Float := Log (2.0 * Long_Float (Levels * (Levels + 1) / 2) / Upper_Tail (Conf_K));
         --  段 [A, E)(级 A .. E − 1)的样本份额 p̂ 的置信界 ÷ 段宽 = 这一段平均密度的界。
         --  伯恩斯坦:|p̂ − P| ≥ t 的概率 ≤ 2·exp(−N t² ÷ (2P(1 − P) + 2t/3)),方差用界端点自己的 P(1 − P);
         --  端点 P = p̂ ± u 解 (N + 2Ln)·u² − B·u − 2Ln·p̂(1 − p̂) = 0,B = 2Ln·(1/3 + (1 − 2p̂))(上界)/ 2Ln·(1/3 + (2p̂ − 1))(下界)
         function Dens (A, E : Natural; Upper : Boolean) return Long_Float is
            P : constant Long_Float := (Cum (E) - Cum (A)) / N;
            B : constant Long_Float := 2.0 * Ln * (1.0 / 3.0 + (if Upper then 1.0 - 2.0 * P else 2.0 * P - 1.0));
            Qa : constant Long_Float := N + 2.0 * Ln;
            U : constant Long_Float := (B + Sqrt (B * B + 8.0 * Ln * Qa * P * (1.0 - P))) / (2.0 * Qa);
         begin
            return (if Upper then Long_Float'Min (1.0, P + U) else Long_Float'Max (0.0, P - U)) / Long_Float (E - A);
         end Dens;
         --  Left (B) = 在级 B 之前结束的所有段里,密度下界最大的那个;Right (C) = 从级 C 起的所有段里
         Left : array (0 .. Levels) of Long_Float := [others => 0.0];
         Right : array (0 .. Levels) of Long_Float := [others => 0.0];
         In_Valley : array (0 .. Levels - 1) of Boolean := [others => False];   --  切在级 K 和 K + 1 之间,落在某一道真谷里
      begin
         for A in Lo .. Hi loop
            for E in A + 1 .. Hi + 1 loop
               declare
                  D : constant Long_Float := Dens (A, E, Upper => False);
               begin
                  Left (E) := Long_Float'Max (Left (E), D);
                  Right (A) := Long_Float'Max (Right (A), D);
               end;
            end loop;
         end loop;
         for B in Lo + 1 .. Hi + 1 loop
            Left (B) := Long_Float'Max (Left (B), Left (B - 1));
         end loop;
         for C in reverse Lo .. Hi - 1 loop
            Right (C) := Long_Float'Max (Right (C), Right (C + 1));
         end loop;
         --  谷 = 段 [B, C):左边的段在 B 之前结束,右边的段从 C 起;谷里任何一刀(切在 B − 1 .. C − 1 之后)都把两边分开
         for B in Lo + 1 .. Hi - 1 loop
            declare
               Last : Natural := B;   --  从 B 起的真谷,最远到哪(= B 就是一道都没有)
            begin
               for C in B + 1 .. Hi loop
                  if Dens (B, C, Upper => True) < Long_Float'Min (Left (B), Right (C)) then
                     Last := C;
                  end if;
               end loop;
               if Last > B then
                  for K in B - 1 .. Last - 1 loop
                     In_Valley (K) := True;
                  end loop;
               end if;
            end;
         end loop;
         declare
            W0, S0, S_All : Long_Float := 0.0;
            Best_Var : Long_Float := 0.0;
            Best_K : Natural := Lo;
            Found : Boolean := False;
         begin
            for K in Lo .. Hi loop
               S_All := S_All + H (K) * Long_Float (K);
            end loop;
            --  切在 K 和 K + 1 之间:K ≥ Lo、K < Hi,两边都有样本(不会除零)
            for K in Lo .. Hi - 1 loop
               W0 := W0 + H (K);
               S0 := S0 + H (K) * Long_Float (K);
               if In_Valley (K) then
                  declare
                     W1 : constant Long_Float := N - W0;
                     Var : constant Long_Float := W0 * W1 * (S0 / W0 - (S_All - S0) / W1) ** 2;
                  begin
                     if not Found or else Var > Best_Var then
                        Best_Var := Var;
                        Best_K := K;
                        Found := True;
                     end if;
                  end;
               end if;
            end loop;
            return (if Found then Long_Float (Best_K) + 0.5 else NaN);
         end;
      end;
   end Split;

   function Inside (R : Region; U, V : Long_Float; W, H : Natural; Grow : Long_Float) return Boolean is
      X0 : constant Long_Float := Long_Float (R.X0) / Long_Float (W);
      X1 : constant Long_Float := Long_Float (R.X1 + 1) / Long_Float (W);
      Y0 : constant Long_Float := Long_Float (R.Y0) / Long_Float (H);
      Y1 : constant Long_Float := Long_Float (R.Y1 + 1) / Long_Float (H);
      Gw : constant Long_Float := (X1 - X0) * Grow;
      Gh : constant Long_Float := (Y1 - Y0) * Grow;
   begin
      return U >= X0 - Gw and then U <= X1 + Gw and then V >= Y0 - Gh and then V <= Y1 + Gh;
   end Inside;
   procedure Region_Of_Mask (M : Bools; W, H : Natural; R : out Region; Ok : out Boolean) is
      Cnt : Natural := 0;
      Sx, Sy, Sxx, Syy, Sxy : Long_Float := 0.0;
      X0, Y0 : Natural := Natural'Last;
      X1, Y1 : Natural := 0;
   begin
      R := (others => <>);
      Ok := False;
      if W = 0 or else H = 0 or else Natural (M.Length) /= W * H then
         return;
      end if;
      for Y in 0 .. H - 1 loop
         for X in 0 .. W - 1 loop
            if M (Y * W + X) then
               Cnt := Cnt + 1;
               Sx := Sx + Long_Float (X); Sy := Sy + Long_Float (Y);
               Sxx := Sxx + Long_Float (X) * Long_Float (X); Syy := Syy + Long_Float (Y) * Long_Float (Y); Sxy := Sxy + Long_Float (X) * Long_Float (Y);
               X0 := Natural'Min (X0, X); Y0 := Natural'Min (Y0, Y); X1 := Natural'Max (X1, X); Y1 := Natural'Max (Y1, Y);
            end if;
         end loop;
      end loop;
      if Cnt = 0 then
         return;
      end if;
      R.X0 := X0; R.Y0 := Y0; R.X1 := X1; R.Y1 := Y1;
      Fill_Shape (R, Cnt, Sx, Sy, Sxx, Syy, Sxy, W, H);
      Ok := True;
   end Region_Of_Mask;

   procedure Grow_Window (R : Region; W, H : Natural; X0, Y0, X1, Y1 : in out Natural; Grew : out Boolean) is
      Tl : constant Boolean := R.X0 <= X0 and then X0 > 0;
      Tt : constant Boolean := R.Y0 <= Y0 and then Y0 > 0;
      Tr : constant Boolean := R.X1 >= X1 and then X1 + 1 < W;
      Tb : constant Boolean := R.Y1 >= Y1 and then Y1 + 1 < H;
      Rw : constant Natural := R.X1 - R.X0 + 1;
      Rh : constant Natural := R.Y1 - R.Y0 + 1;
   begin
      Grew := Tl or else Tt or else Tr or else Tb;
      if Tl then
         X0 := (if X0 > Rw then X0 - Rw else 0);
      end if;
      if Tt then
         Y0 := (if Y0 > Rh then Y0 - Rh else 0);
      end if;
      if Tr then
         X1 := Natural'Min (W - 1, X1 + Rw);
      end if;
      if Tb then
         Y1 := Natural'Min (H - 1, Y1 + Rh);
      end if;
   end Grow_Window;

   function Covers_Interior (New_M, Old_M : Bools; W, H : Natural; R : Region) return Boolean is
   begin
      if W < 3 or else H < 3 or else Natural (New_M.Length) /= W * H or else Natural (Old_M.Length) /= W * H then
         return False;
      end if;
      for Y in Natural'Max (1, R.Y0) .. Natural'Min (R.Y1, H - 2) loop
         for X in Natural'Max (1, R.X0) .. Natural'Min (R.X1, W - 2) loop
            declare
               I : constant Natural := Y * W + X;
            begin
               if Old_M (I) and then Old_M (I - 1) and then Old_M (I + 1) and then Old_M (I - W) and then Old_M (I + W) and then not New_M (I) then
                  return False;
               end if;
            end;
         end loop;
      end loop;
      return True;
   end Covers_Interior;

end Picture;
