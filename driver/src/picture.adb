with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Interfaces;
package body Picture is
   function To_LF is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, Long_Float);
   NaN : constant Long_Float := To_LF (16#7FF8000000000000#);

   function Is_Nan (X : Long_Float) return Boolean is (X /= X);

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

   procedure Mean_Colour (RGB : Buf; W, H : Natural; R : Region; Cr, Cg, Cb : out Long_Float) is
      Sr, Sg, Sb : Long_Float := 0.0;
      N : Natural := 0;
   begin
      Cr := -1.0; Cg := -1.0; Cb := -1.0;
      if Natural (RGB.Length) < W * H * 3 then
         return;
      end if;
      for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
         for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
            Sr := Sr + Long_Float (RGB.Element (3 * (Y * W + X)));
            Sg := Sg + Long_Float (RGB.Element (3 * (Y * W + X) + 1));
            Sb := Sb + Long_Float (RGB.Element (3 * (Y * W + X) + 2));
            N := N + 1;
         end loop;
      end loop;
      if N > 0 then
         Cr := Sr / Long_Float (N); Cg := Sg / Long_Float (N); Cb := Sb / Long_Float (N);
      end if;
   end Mean_Colour;

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
                  C : constant Long_Float := Long_Float (A1.Cnt);
                  Mx : constant Long_Float := A1.Sx / C;
                  My : constant Long_Float := A1.Sy / C;
                  Vxx : constant Long_Float := Long_Float'Max (0.0, A1.Sxx / C - Mx * Mx);
                  Vyy : constant Long_Float := Long_Float'Max (0.0, A1.Syy / C - My * My);
                  Vxy : constant Long_Float := A1.Sxy / C - Mx * My;
                  Tr : constant Long_Float := Vxx + Vyy;
                  Det : constant Long_Float := Long_Float'Max (0.0, Vxx * Vyy - Vxy * Vxy);
                  Disc : constant Long_Float := Long_Float'Max (0.0, 0.25 * Tr * Tr - Det);
                  L1 : constant Long_Float := 0.5 * Tr + Sqrt (Disc);
                  L2 : constant Long_Float := Long_Float'Max (0.0, 0.5 * Tr - Sqrt (Disc));
                  Ax, Ay, Ln : Long_Float;
               begin
                  R.X0 := A1.X0; R.Y0 := A1.Y0; R.X1 := A1.X1; R.Y1 := A1.Y1;
                  R.Count := A1.Cnt;
                  R.Cu := Mx / Long_Float (W); R.Cv := My / Long_Float (H);
                  R.Sig_U := Sqrt (Vxx) / Long_Float (W); R.Sig_V := Sqrt (Vyy) / Long_Float (H);
                  if abs Vxy > 1.0e-12 then
                     Ax := L1 - Vyy; Ay := Vxy;
                  elsif Vxx >= Vyy then
                     Ax := 1.0; Ay := 0.0;
                  else
                     Ax := 0.0; Ay := 1.0;
                  end if;
                  Ln := Sqrt (Ax * Ax + Ay * Ay);
                  if Ln > 1.0e-12 then
                     R.Au := Ax / Ln; R.Av := Ay / Ln;
                  end if;
                  --  短轴为零时伸长比记成一个大数(无量纲)
                  R.Elong := (if L2 > 1.0e-9 then Sqrt (L1 / L2) else 1.0e3);
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

   function Cut (Depth : Floats; W, H : Natural; Win_Frac, Sigma_Mult : Long_Float) return Regions is
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
            --  中位绝对偏差可能是 0(量化)⇒ 往上取分位数直到拿到正的尺度;1.4826 = MAD→σ 的固定换算,无量纲
            Sigma := 0.0;
            for Q of Quantiles loop
               declare
                  V : constant Long_Float := Quantile (Absd, Q) * 1.4826;
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
                  --  贴着画面边的块丢掉:整条背景带、细缝、我自己的胳膊都贴边;能拿的东西完整地在画面里
                  if Rg.X0 > 0 and then Rg.Y0 > 0 and then Rg.X1 + 1 < W and then Rg.Y1 + 1 < H then
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

   function Region_Mask (Depth : Floats; W, H : Natural; R : Region) return Bools is
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      Thick : constant Long_Float := Long_Float'Max (1.0e-4, abs R.Height);
   begin
      for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
         for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
            declare
               I : constant Natural := Y * W + X;
               Z : constant Long_Float := (if I < Natural (Depth.Length) then Depth.Element (I) else NaN);
            begin
               if not Is_Nan (Z) and then Z > 1.0e-6 and then abs (Z - R.Depth) <= Thick then
                  M.Replace_Element (I, True);
               end if;
            end;
         end loop;
      end loop;
      return M;
   end Region_Mask;

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
                     C : constant Long_Float := Long_Float (Cnt);
                     Mx : constant Long_Float := Sx / C;
                     My : constant Long_Float := Sy / C;
                     Vxx : constant Long_Float := Long_Float'Max (0.0, Sxx / C - Mx * Mx);
                     Vyy : constant Long_Float := Long_Float'Max (0.0, Syy / C - My * My);
                     Vxy : constant Long_Float := Sxy / C - Mx * My;
                     Tr : constant Long_Float := Vxx + Vyy;
                     Det : constant Long_Float := Long_Float'Max (0.0, Vxx * Vyy - Vxy * Vxy);
                     Disc : constant Long_Float := Long_Float'Max (0.0, 0.25 * Tr * Tr - Det);
                     L1 : constant Long_Float := 0.5 * Tr + Sqrt (Disc);
                     L2 : constant Long_Float := Long_Float'Max (0.0, 0.5 * Tr - Sqrt (Disc));
                     Ax, Ay : Long_Float;
                  begin
                     R.X0 := X0; R.Y0 := Y0; R.X1 := X1; R.Y1 := Y1;
                     R.Count := Cnt;
                     R.Cu := Mx / Long_Float (W);
                     R.Cv := My / Long_Float (H);
                     R.Sig_U := Sqrt (Vxx) / Long_Float (W);
                     R.Sig_V := Sqrt (Vyy) / Long_Float (H);
                     --  主轴 = 协方差最大特征值的特征向量
                     if abs Vxy > 1.0e-12 then
                        Ax := L1 - Vyy; Ay := Vxy;
                     elsif Vxx >= Vyy then
                        Ax := 1.0; Ay := 0.0;
                     else
                        Ax := 0.0; Ay := 1.0;
                     end if;
                     declare
                        Ln : constant Long_Float := Sqrt (Ax * Ax + Ay * Ay);
                     begin
                        if Ln > 0.0 then
                           R.Au := Ax / Ln; R.Av := Ay / Ln;
                        else
                           R.Au := 1.0; R.Av := 0.0;
                        end if;
                     end;
                     --  短轴为零时伸长比记成一个大数(无量纲)
                     R.Elong := (if L2 > 1.0e-9 then Sqrt (L1 / L2) else 1.0e3);
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

   function Split (F : Floats) return Long_Float is
      N : constant Natural := Natural (F.Length);
      Lo : Long_Float := 1.0e30;
      Hi : Long_Float := -1.0e30;
      Bins : constant := 64;      --  直方图格数(次数,无量纲)
      H : array (0 .. Bins - 1) of Long_Float := [others => 0.0];
      Best_T : Long_Float := NaN;
      Best_Var : Long_Float := -1.0;
      Total : Long_Float := 0.0;
      Sum_All : Long_Float := 0.0;
   begin
      if N < 16 then
         return NaN;
      end if;
      for X of F loop
         if not Is_Nan (X) then
            Lo := Long_Float'Min (Lo, X);
            Hi := Long_Float'Max (Hi, X);
         end if;
      end loop;
      if not (Hi > Lo) then
         return NaN;
      end if;
      for X of F loop
         if not Is_Nan (X) then
            declare
               B : constant Natural := Natural'Min (Bins - 1, Natural (Long_Float'Floor ((X - Lo) / (Hi - Lo) * Long_Float (Bins))));
            begin
               H (B) := H (B) + 1.0;
               Total := Total + 1.0;
               Sum_All := Sum_All + Long_Float (B);
            end;
         end if;
      end loop;
      declare
         W0, Sum0 : Long_Float := 0.0;
      begin
         for B in 0 .. Bins - 2 loop
            W0 := W0 + H (B);
            Sum0 := Sum0 + H (B) * Long_Float (B);
            declare
               W1 : constant Long_Float := Total - W0;
            begin
               if W0 > 0.0 and then W1 > 0.0 then
                  declare
                     M0 : constant Long_Float := Sum0 / W0;
                     M1 : constant Long_Float := (Sum_All - Sum0) / W1;
                     Var : constant Long_Float := W0 * W1 * (M0 - M1) * (M0 - M1);
                  begin
                     if Var > Best_Var then
                        Best_Var := Var;
                        Best_T := Lo + (Long_Float (B) + 1.0) / Long_Float (Bins) * (Hi - Lo);
                     end if;
                  end;
               end if;
            end;
         end loop;
      end;
      --  两拨要真的分得开:类间方差得占总方差的大头(比例,无量纲),否则是单峰
      declare
         Mean : constant Long_Float := Sum_All / Total;
         Tot_Var : Long_Float := 0.0;
      begin
         for B in 0 .. Bins - 1 loop
            Tot_Var := Tot_Var + H (B) * (Long_Float (B) - Mean) ** 2;
         end loop;
         if Tot_Var <= 0.0 or else Best_Var / Total < 0.5 * Tot_Var then
            return NaN;
         end if;
      end;
      return Best_T;
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
end Picture;
