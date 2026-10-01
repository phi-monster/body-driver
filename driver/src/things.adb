with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Codec;
with Stats;
package body Things is
   use Geom;

   Store : Estimate_Vectors.Vector;

   --  像素量化本身的不准:一个像素里均匀分布的标准差 1 / √12(纯数学)—— 量到的轮廓不准(Px_Sd)再小也不比它小,
   --  不然放宽的那一圈是 0,八叉树永远"还分得出"
   Quant_Px : constant Long_Float := 1.0 / Sqrt (12.0);

   Half_Px : constant Long_Float := 0.5;   --  半个像素(像素中心和像素边差半格,纯几何)

   function Px_Of (V : View) return Long_Float is (Long_Float'Max (V.Px_Sd, Quant_Px));
   --  这一眼在离它 Depth 远的地方的一个标准差(像素):轮廓的像素不准 ⊕ 眼的位置不准投进去
   function Sd_Px (V : View; Depth : Long_Float) return Long_Float is
     (Sqrt (Px_Of (V) ** 2 + (if Depth > 0.0 then (V.Pos_Sd * V.Cam.F / Depth) ** 2 else 0.0)));
   --  放宽多少像素:Z 个标准差
   function Dilate (V : View; Depth : Long_Float) return Long_Float is (Stats.Z * Sd_Px (V, Depth));
   --  这一眼在 Depth 远处一个标准差合多少世界单位
   function Sd_World (V : View; Depth : Long_Float) return Long_Float is
     (Sd_Px (V, Depth) * Depth / Long_Float'Max (V.Cam.F, Long_Float'Model_Small));

   function Dist (A, B : V3) return Long_Float is (Norm ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]));
   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));

   --  ── 积分图:矩形 [Ox, Ox + Iw) × [Oy, Oy + Ih) 里数像素,任一矩形 O(1)(Iw = 0 ⇒ 什么都没有)──
   type Nat_Arr is array (Natural range <>) of Natural;
   type Nat_Ptr is access Nat_Arr;
   procedure Free_Arr is new Ada.Unchecked_Deallocation (Nat_Arr, Nat_Ptr);
   type Integral is record
      Ox, Oy : Integer := 0;
      Iw, Ih : Natural := 0;
      A : Nat_Ptr;
   end record;
   type Integral_Arr is array (Natural range <>) of Integral;
   type Integral_Ptr is access Integral_Arr;
   procedure Free_Ints is new Ada.Unchecked_Deallocation (Integral_Arr, Integral_Ptr);
   procedure Free_Integral (I : in out Integral) is
   begin
      if I.A /= null then
         Free_Arr (I.A);
      end if;
   end Free_Integral;
   --  Of_Unknown = False:窗里它的像素(只在窗 ∩ 画幅里建);True:整幅里它可能在、可又没量到的像素
   function Build (V : View; Of_Unknown : Boolean := False) return Integral is
      R : Integral;
   begin
      if Of_Unknown then
         if Natural (V.Unknown.Length) /= V.W * V.H or else V.W = 0 then
            return R;
         end if;
         R.Ox := 0; R.Oy := 0; R.Iw := V.W; R.Ih := V.H;
      else
         declare
            X0 : constant Integer := Integer'Max (0, V.X0);
            Y0 : constant Integer := Integer'Max (0, V.Y0);
            X1 : constant Integer := Integer'Min (V.W - 1, V.X1);
            Y1 : constant Integer := Integer'Min (V.H - 1, V.Y1);
         begin
            if X1 < X0 or else Y1 < Y0 then
               return R;
            end if;
            R.Ox := X0; R.Oy := Y0; R.Iw := Natural (X1 - X0 + 1); R.Ih := Natural (Y1 - Y0 + 1);
         end;
      end if;
      R.A := new Nat_Arr (0 .. (R.Iw + 1) * (R.Ih + 1) - 1);
      R.A.all := [others => 0];
      for Y in 0 .. R.Ih - 1 loop
         declare
            Row : Natural := 0;
         begin
            for X in 0 .. R.Iw - 1 loop
               declare
                  I : constant Natural := (R.Oy + Y) * V.W + R.Ox + X;
               begin
                  if (if Of_Unknown then V.Unknown (I) else V.Mask (I)) then
                     Row := Row + 1;
                  end if;
               end;
               R.A ((Y + 1) * (R.Iw + 1) + X + 1) := R.A (Y * (R.Iw + 1) + X + 1) + Row;
            end loop;
         end;
      end loop;
      return R;
   end Build;
   function Count_In (I : Integral; X0, Y0, X1, Y1 : Integer) return Natural is
      Ax0 : constant Integer := Integer'Max (X0, I.Ox) - I.Ox;
      Ay0 : constant Integer := Integer'Max (Y0, I.Oy) - I.Oy;
      Ax1 : constant Integer := Integer'Min (X1, I.Ox + Integer (I.Iw) - 1) - I.Ox;
      Ay1 : constant Integer := Integer'Min (Y1, I.Oy + Integer (I.Ih) - 1) - I.Oy;
      W1 : constant Natural := I.Iw + 1;
   begin
      if I.A = null or else Ax1 < Ax0 or else Ay1 < Ay0 then
         return 0;
      end if;
      return I.A (Natural (Ay1 + 1) * W1 + Natural (Ax1 + 1)) + I.A (Natural (Ay0) * W1 + Natural (Ax0))
        - I.A (Natural (Ay0) * W1 + Natural (Ax1 + 1)) - I.A (Natural (Ay1 + 1) * W1 + Natural (Ax0));
   end Count_In;

   --  一格的八个角 / 一个点(栈上的定长数组:八叉树一格一格问,不许每问一次就上堆)
   type Pt_Arr is array (Natural range <>) of V3;
   Corners_Of_Cell : constant := 8;   --  一格(立方体)八个角
   function Corners (Cl : Cell) return Pt_Arr is
      R : Pt_Arr (0 .. Corners_Of_Cell - 1);
      K : Natural := 0;
   begin
      for Sx in 0 .. 1 loop
         for Sy in 0 .. 1 loop
            for Sz in 0 .. 1 loop
               R (K) := [Cl.C (0) + (if Sx = 0 then -Cl.H else Cl.H), Cl.C (1) + (if Sy = 0 then -Cl.H else Cl.H), Cl.C (2) + (if Sz = 0 then -Cl.H else Cl.H)];
               K := K + 1;
            end loop;
         end loop;
      end loop;
      return R;
   end Corners;
   --  一块(或一点)投进这一眼:U / V 的范围,Depth = 离眼最近多远;Front = False:整个在眼身后(或眼的平面上),这只眼看不见它。
   --  一格跨在眼的像平面上(有的角在眼前、有的在身后):眼前那一截的投影一直伸到画幅外 —— 跨过像平面的那几条棱,
   --  在像平面上那一点离光轴往哪边偏,投影就往哪边伸到无穷;棱穿过眼心 ⇒ 四面都伸到无穷(整格把眼包在里面)。
   --  伸到画幅外的那几边记成画幅外一个画幅那么远(只当记号:Judge 按画幅剪);Depth 连跨点一起取最近的。
   --  (10-01 箱上实测:只有一只眼整个看见它时,起步格把那只眼自己也兜进去,原来"有一个角在眼身后 ⇒ 投不出"⇒ 哪只眼都说不上话 ⇒
   --  整格扔掉,外包是空的,又被当成"几眼对不上 = 它动过了"把眼全丢了)。Rt = 世界 → 这只眼(R_Ce 的转置,调用方算一次)
   procedure Footprint (V : View; Rt : M3; Pts : Pt_Arr; U0, U1, V0, V1, Depth : out Long_Float; Front : out Boolean) is
      type Pc_Arr is array (Pts'Range) of V3;
      type Fr_Arr is array (Pts'Range) of Boolean;
      Pc : Pc_Arr;
      Fr : Fr_Arr;
      N_Front : Natural := 0;
      Far : constant Long_Float := Long_Float (V.W + V.H);   --  画幅外:比画幅还远一个画幅
   begin
      U0 := Long_Float'Last; U1 := Long_Float'First; V0 := Long_Float'Last; V1 := Long_Float'First; Depth := Long_Float'Last; Front := True;
      for K in Pts'Range loop
         declare
            P : constant V3 := Pts (K);
            D : constant V3 := [P (0) - V.Cam.Pos (0), P (1) - V.Cam.Pos (1), P (2) - V.Cam.Pos (2)];
            U, Vv : Long_Float;
         begin
            Pc (K) := [Rt (0, 0) * D (0) + Rt (0, 1) * D (1) + Rt (0, 2) * D (2),
                       Rt (1, 0) * D (0) + Rt (1, 1) * D (1) + Rt (1, 2) * D (2),
                       Rt (2, 0) * D (0) + Rt (2, 1) * D (1) + Rt (2, 2) * D (2)];
            Cam_Pixel (V.Cam, Pc (K), U, Vv, Fr (K));
            if Fr (K) then
               N_Front := N_Front + 1;
               U0 := Long_Float'Min (U0, U); U1 := Long_Float'Max (U1, U);
               V0 := Long_Float'Min (V0, Vv); V1 := Long_Float'Max (V1, Vv);
               Depth := Long_Float'Min (Depth, Norm (D));
            end if;
         end;
      end loop;
      if N_Front = 0 then
         Front := False;
         return;
      end if;
      if N_Front = Pts'Length then
         return;
      end if;
      if Pts'Length /= Corners_Of_Cell then
         Front := False;   --  不是一格(没有棱可跨):说不出
         return;
      end if;
      --  一格的棱:两个角的下标只差一位(Corners 的排法:下标 = Sx·4 + Sy·2 + Sz)
      for K in Pts'Range loop
         for Bit_Pos in 0 .. 2 loop
            declare
               Bit : constant Natural := 2 ** Bit_Pos;
            begin
               if (K / Bit) mod 2 = 0 and then Fr (K) /= Fr (K + Bit) then
                  declare
                     A : constant V3 := Pc (K);
                     B : constant V3 := Pc (K + Bit);
                     T : constant Long_Float := A (2) / (A (2) - B (2));   --  相机系 z = 0(像平面)那一点
                     Q : constant V3 := [A (0) + T * (B (0) - A (0)), A (1) + T * (B (1) - A (1)), 0.0];
                  begin
                     Depth := Long_Float'Min (Depth, Norm (Q));
                     if Q (0) = 0.0 and then Q (1) = 0.0 then
                        U0 := -Far; U1 := Long_Float (V.W) + Far; V0 := -Far; V1 := Long_Float (V.H) + Far;
                     else
                        if Q (0) > 0.0 then
                           U1 := Long_Float (V.W) + Far;
                        elsif Q (0) < 0.0 then
                           U0 := -Far;
                        end if;
                        if Q (1) > 0.0 then
                           V0 := -Far;   --  相机系 +y 朝上,像素 v 朝下
                        elsif Q (1) < 0.0 then
                           V1 := Long_Float (V.H) + Far;
                        end if;
                     end if;
                  end;
               end if;
            end;
         end loop;
      end loop;
   end Footprint;

   --  放宽 D 像素以后的投影矩形按这一眼判:一个它的像素都没有、也没落在它可能在却没量到的像素上、伸到画幅外时它又伸不出画幅 ⇒ Free;
   --  整个落在画幅里、整个是它的像素 ⇒ Inside;有它的像素 ⇒ Mixed;别的(只落在说不出的像素上、或伸到它也可能伸到的画幅外)⇒ No_Info
   function Judge (V : View; Im, Iu : Integral; U0, U1, V0, V1, D : Long_Float) return Verdict is
      --  像素 X 的中心在 U = X(驱动的约定:视线按整数像素坐标发,投影四舍五入回像素)⇒ 它管 [X − ½, X + ½);换成"第几个像素"的坐标 = U + ½
      Lu : constant Long_Float := U0 - D + Half_Px;
      Hu : constant Long_Float := U1 + D + Half_Px;
      Lv : constant Long_Float := V0 - D + Half_Px;
      Hv : constant Long_Float := V1 + D + Half_Px;
      Wf : constant Long_Float := Long_Float (V.W);
      Hf : constant Long_Float := Long_Float (V.H);
   begin
      if Hu < 0.0 or else Hv < 0.0 or else Lu >= Wf or else Lv >= Hf then
         return (if V.Beyond then No_Info else Free);
      end if;
      declare
         In_Img : constant Boolean := Lu >= 0.0 and then Lv >= 0.0 and then Hu < Wf and then Hv < Hf;
         Px0 : constant Integer := Integer (Long_Float'Floor (Long_Float'Max (Lu, 0.0)));
         Px1 : constant Integer := Integer (Long_Float'Floor (Long_Float'Min (Hu, Wf - Half_Px)));
         Py0 : constant Integer := Integer (Long_Float'Floor (Long_Float'Max (Lv, 0.0)));
         Py1 : constant Integer := Integer (Long_Float'Floor (Long_Float'Min (Hv, Hf - Half_Px)));
         Area : constant Long_Float := Long_Float (Px1 - Px0 + 1) * Long_Float (Py1 - Py0 + 1);
         Nm : constant Natural := Count_In (Im, Px0, Py0, Px1, Py1);
         Nu : constant Natural := Count_In (Iu, Px0, Py0, Px1, Py1);
      begin
         if Nm = 0 then
            return (if Nu = 0 and then (In_Img or else not V.Beyond) then Free else No_Info);
         end if;
         return (if In_Img and then Long_Float (Nm) = Area then Inside else Mixed);
      end;
   end Judge;

   function Point_In (V : View; X : V3) return Verdict is
      Im : Integral := Build (V);
      Iu : Integral := Build (V, Of_Unknown => True);
      U0, U1, V0, V1, Dp : Long_Float;
      Fr : Boolean;
      R : Verdict := No_Info;
   begin
      Footprint (V, Tr (V.Cam.R_Ce), Pt_Arr'(0 => X), U0, U1, V0, V1, Dp, Fr);
      if Fr then
         R := Judge (V, Im, Iu, U0, U1, V0, V1, Dilate (V, Dp));
      elsif not V.Beyond then
         R := Free;   --  在这只眼身后:它伸不出这只眼的画幅 ⇒ 整个在眼前 ⇒ 那儿不是它
      end if;
      Free_Integral (Im); Free_Integral (Iu);
      return R;
   end Point_In;

   procedure Reach (V : in out View) is
      N : constant Natural := V.W * V.H;
      X0 : constant Integer := Integer'Max (0, V.X0);
      Y0 : constant Integer := Integer'Max (0, V.Y0);
      X1 : constant Integer := Integer'Min (V.W - 1, V.X1);
      Y1 : constant Integer := Integer'Min (V.H - 1, V.Y1);
      Has_Occl : constant Boolean := Natural (V.Occl.Length) = N;
      Touch_Win : Boolean := False;
      Any_Unknown : Boolean := False;
      Queue : Ints;
      Head : Natural := 0;
      function On_Border (I : Natural) return Boolean is
        (I mod V.W = 0 or else I mod V.W = V.W - 1 or else I / V.W = 0 or else I / V.W = V.H - 1);
      --  挨着的像素 J 是挡着的、还没走过 ⇒ 算进 Unknown,接着从它往外走
      procedure Visit (J : Natural) is
      begin
         if Has_Occl and then V.Occl (J) and then not V.Unknown (J) and then not V.Mask (J) then
            V.Unknown (J) := True; Any_Unknown := True;
            Queue.Append (Integer (J));
         end if;
      end Visit;
      procedure Neighbours (I : Natural) is
         X : constant Natural := I mod V.W;
         Y : constant Natural := I / V.W;
      begin
         if X > 0 then
            Visit (I - 1);
         end if;
         if X + 1 < V.W then
            Visit (I + 1);
         end if;
         if Y > 0 then
            Visit (I - V.W);
         end if;
         if Y + 1 < V.H then
            Visit (I + V.W);
         end if;
      end Neighbours;
   begin
      V.Unknown.Clear; V.Beyond := False; V.Whole := False;
      if Natural (V.Mask.Length) /= N or else N = 0 or else X1 < X0 or else Y1 < Y0 then
         return;
      end if;
      V.Unknown.Set_Length (Ada.Containers.Count_Type (N));
      for I in 0 .. N - 1 loop
         V.Unknown (I) := False;
      end loop;
      --  掩膜只在窗里作数;它的像素若是我自己挡着的(仪器把手指也圈进去了)⇒ 那是我,不是它
      for I in 0 .. N - 1 loop
         if V.Mask (I) then
            declare
               X : constant Integer := Integer (I mod V.W);
               Y : constant Integer := Integer (I / V.W);
            begin
               if X < X0 or else X > X1 or else Y < Y0 or else Y > Y1 or else (Has_Occl and then V.Occl (I)) then
                  V.Mask (I) := False;
               end if;
            end;
         end if;
      end loop;
      for Y in Y0 .. Y1 loop
         for X in X0 .. X1 loop
            declare
               I : constant Natural := Natural (Y) * V.W + Natural (X);
            begin
               if V.Mask (I) then
                  if On_Border (I) then
                     V.Beyond := True;   --  顶到画幅边
                  end if;
                  if (X = X0 and then X0 > 0) or else (X = X1 and then X1 < V.W - 1) or else (Y = Y0 and then Y0 > 0) or else (Y = Y1 and then Y1 < V.H - 1) then
                     Touch_Win := True;  --  顶到窗边(那条窗边不在画幅边上)
                  end if;
                  Neighbours (I);
               end if;
            end;
         end loop;
      end loop;
      --  挡着的像素一片连一片地挨过去
      while Head < Natural (Queue.Length) loop
         declare
            I : constant Natural := Natural (Queue (Head));
         begin
            Head := Head + 1;
            if On_Border (I) then
               V.Beyond := True;
            end if;
            Neighbours (I);
         end;
      end loop;
      if Touch_Win then
         for I in 0 .. N - 1 loop
            declare
               X : constant Integer := Integer (I mod V.W);
               Y : constant Integer := Integer (I / V.W);
            begin
               if X < X0 or else X > X1 or else Y < Y0 or else Y > Y1 then
                  V.Unknown (I) := True; Any_Unknown := True;
               end if;
            end;
         end loop;
         V.Beyond := True;   --  窗外那一圈一直连到画幅边
      end if;
      V.Whole := not V.Beyond and then not Any_Unknown;
   end Reach;

   function Whole_In_Window (V : View) return Boolean is
      Vv : View := V;
   begin
      Reach (Vv);
      return Vv.Whole;
   end Whole_In_Window;

   --  ── 每台相机轮廓的像素不准(量的):同一只眼、眼没挪没转的两眼,掩膜的边挪了多少。
   --  两张掩膜不一样的像素数 ÷ 边界长 = 两条边各自抖 σ 时两边距离的平均绝对值 = √2 σ × √(2/π) = 2σ / √π(正态,纯数学)⇒ σ = 它 × √π / 2 ──
   type Cam_Noise is record
      Samples : Floats;
   end record;
   package Noise_Vectors is new Ada.Containers.Vectors (Natural, Cam_Noise);
   Noise : Noise_Vectors.Vector;
   function Median (X : Floats) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      S : Floats := X;
   begin
      if S.Is_Empty then
         return 0.0;
      end if;
      Sorting.Sort (S);
      return S (Natural (S.Length) / 2);
   end Median;
   function Boundary_Px (Cam : Natural) return Long_Float is
     (if Cam < Natural (Noise.Length) then Median (Noise (Cam).Samples) else 0.0);
   function Edge_Count (V : View) return Natural is
      N : Natural := 0;
   begin
      for Y in Integer'Max (0, V.Y0) .. Integer'Min (V.H - 1, V.Y1) loop
         for X in Integer'Max (0, V.X0) .. Integer'Min (V.W - 1, V.X1) loop
            declare
               I : constant Natural := Natural (Y) * V.W + Natural (X);
            begin
               if V.Mask (I) and then (X = 0 or else Y = 0 or else X = V.W - 1 or else Y = V.H - 1
                                       or else not V.Mask (I - 1) or else not V.Mask (I + 1) or else not V.Mask (I - V.W) or else not V.Mask (I + V.W))
               then
                  N := N + 1;
               end if;
            end;
         end loop;
      end loop;
      return N;
   end Edge_Count;
   function Edge_Sd (Cam : Natural) return Long_Float is (Long_Float'Max (Boundary_Px (Cam), Quant_Px));
   procedure Note_Boundary (Prev, Now : View) is
      Diff : Natural := 0;
      Edge : constant Natural := Edge_Count (Now);
   begin
      if Prev.W /= Now.W or else Prev.H /= Now.H or else Natural (Prev.Mask.Length) /= Prev.W * Prev.H then
         return;
      end if;
      for I in 0 .. Now.W * Now.H - 1 loop
         if Prev.Mask (I) /= Now.Mask (I) then
            Diff := Diff + 1;
         end if;
      end loop;
      if Edge > 0 then
         while Natural (Noise.Length) <= Now.Cam_Index loop
            Noise.Append (Cam_Noise'(others => <>));
         end loop;
         declare
            Cn : Cam_Noise := Noise (Now.Cam_Index);
         begin
            Cn.Samples.Append (Long_Float (Diff) / Long_Float (Edge) * Sqrt (Pi) / 2.0);
            Noise.Replace_Element (Now.Cam_Index, Cn);
         end;
      end if;
   end Note_Boundary;

   --  它离这一眼多远(看"眼心挪没挪出放宽那一圈"用):解过 ⇒ 到外包形心;没解过 ⇒ 这一眼掩膜形心那条视线落到它躺的面上;都没有 ⇒ 0(说不出)
   function Look_Depth (E : Estimate; V : View) return Long_Float is
   begin
      if not E.Solid.Is_Empty then
         return Dist (E.Center, V.Cam.Pos);
      end if;
      if E.Has_Support then
         declare
            Su, Sv : Long_Float := 0.0;
            N : Natural := 0;
         begin
            for Y in Integer'Max (0, V.Y0) .. Integer'Min (V.H - 1, V.Y1) loop
               for X in Integer'Max (0, V.X0) .. Integer'Min (V.W - 1, V.X1) loop
                  if V.Mask (Natural (Y) * V.W + Natural (X)) then
                     Su := Su + Long_Float (X); Sv := Sv + Long_Float (Y); N := N + 1;
                  end if;
               end loop;
            end loop;
            if N > 0 then
               declare
                  Ok, Okh : Boolean;
                  D : constant V3 := Ray_Fixed (V.Cam, Su / Long_Float (N), Sv / Long_Float (N), Ok);
                  P : constant V3 := (if Ok then Hit_Plane (V.Cam.Pos, D, E.Support_P, E.Support_N, Okh) else V.Cam.Pos);
               begin
                  if Ok and then Okh then
                     return Dist (P, V.Cam.Pos);
                  end if;
               end;
            end if;
         end;
      end if;
      return 0.0;
   end Look_Depth;
   --  眼心挪了多少像素(在它那儿):挪的距离 × 焦距 ÷ 远近。远近说不出 ⇒ 挪了就当挪了很多
   function Shift_Px (A, B : View; Depth : Long_Float) return Long_Float is
     (if Dist (A.Cam.Pos, B.Cam.Pos) = 0.0 then 0.0
      elsif Depth > 0.0 then Dist (A.Cam.Pos, B.Cam.Pos) * B.Cam.F / Depth
      else Long_Float'Last);
   --  转了多少像素:两次朝向之间的转角 × 焦距
   function Turn_Px (A, B : View) return Long_Float is
     (Norm (Rot_Vec (Mul (Tr (A.Cam.R_Ce), B.Cam.R_Ce))) * B.Cam.F);

   --  新的一眼和雕出来的外包对不上吗:外包(没被雕掉的格子,各放宽这一眼的那一圈)投进这一眼,它的像素落在外头的那些,
   --  平均每个边界像素合多少 —— 比这一眼轮廓自己的一个标准差还多 ⇒ 对不上(只抖的话放宽了 Z 倍以后几乎落不到外头)。
   function Off_Hull (E : Estimate; V : View) return Boolean is
      Cover : Bools;
      N_Out : Natural := 0;
      Edge : constant Natural := Edge_Count (V);
      Rt : constant M3 := Tr (V.Cam.R_Ce);
      procedure Paint (Cl : Cell) is
         U0, U1, V0, V1, Dp : Long_Float;
         Fr : Boolean;
      begin
         Footprint (V, Rt, Corners (Cl), U0, U1, V0, V1, Dp, Fr);
         if not Fr then
            return;   --  整格在这只眼身后:它不挡这只眼里的哪个像素
         end if;
         declare
            D : constant Long_Float := Dilate (V, Dp);
            Lu : constant Long_Float := Long_Float'Max (U0 - D + Half_Px, 0.0);
            Hu : constant Long_Float := Long_Float'Min (U1 + D + Half_Px, Long_Float (V.W - 1));
            Lv : constant Long_Float := Long_Float'Max (V0 - D + Half_Px, 0.0);
            Hv : constant Long_Float := Long_Float'Min (V1 + D + Half_Px, Long_Float (V.H - 1));
         begin
            if Lu <= Hu and then Lv <= Hv then
               for Y in Natural (Long_Float'Floor (Lv)) .. Natural (Long_Float'Floor (Hv)) loop
                  for X in Natural (Long_Float'Floor (Lu)) .. Natural (Long_Float'Floor (Hu)) loop
                     Cover (Y * V.W + X) := True;
                  end loop;
               end loop;
            end if;
         end;
      end Paint;
   begin
      Cover.Set_Length (Ada.Containers.Count_Type (V.W * V.H));
      for I in 0 .. V.W * V.H - 1 loop
         Cover (I) := False;
      end loop;
      for Cl of E.Solid loop
         Paint (Cl);
      end loop;
      for Cl of E.Unseen loop
         Paint (Cl);
      end loop;
      for Y in Integer'Max (0, V.Y0) .. Integer'Min (V.H - 1, V.Y1) loop
         for X in Integer'Max (0, V.X0) .. Integer'Min (V.W - 1, V.X1) loop
            declare
               I : constant Natural := Natural (Y) * V.W + Natural (X);
            begin
               if V.Mask (I) and then not Cover (I) then
                  N_Out := N_Out + 1;
               end if;
            end;
         end loop;
      end loop;
      return Edge > 0 and then Long_Float (N_Out) / Long_Float (Edge) > Px_Of (V);
   end Off_Hull;

   procedure Add_View (E : in out Estimate; V : View) is
      Vv : View := V;
   begin
      if Natural (V.Mask.Length) /= V.W * V.H or else V.W = 0 or else V.H = 0 or else V.Cam.F <= 0.0 then
         return;
      end if;
      Reach (Vv);
      if Edge_Count (Vv) = 0 then
         return;   --  窗里没有它的像素:这一眼没话说
      end if;
      declare
         Depth : constant Long_Float := Look_Depth (E, Vv);
         Fresh : Boolean := True;   --  新的一处(不是哪一眼的同一个锥)
      begin
         --  同一台相机、眼心没挪出放宽那一圈的那几眼:同一个锥 ⇒ 换成这一眼;眼也没转(画面里挪不到一个量化像素)⇒ 顺带量一次轮廓的抖
         for I in reverse 0 .. Natural (E.Views.Length) - 1 loop
            if E.Views (I).Cam_Index = Vv.Cam_Index and then Shift_Px (E.Views (I), Vv, Depth) < Dilate (Vv, Depth) then
               if Shift_Px (E.Views (I), Vv, Depth) + Turn_Px (E.Views (I), Vv) <= Quant_Px then
                  Note_Boundary (E.Views (I), Vv);
               end if;
               E.Views.Delete (I);
               Fresh := False;
            end if;
         end loop;
         Vv.Px_Sd := Sqrt (V.Px_Sd ** 2 + Boundary_Px (V.Cam_Index) ** 2);
         --  每一眼都和已经雕出来的外包核一下它动没动(外包得有一只兜得住它的眼 —— 它伸不出那只眼的画幅 —— 不然外包本来就不全);
         --  不动的眼一拍一拍换掉自己上一眼时也核 —— 只有它看着的时候东西被推走,别的眼留下的锥就和它对不上了
         if not E.Views.Is_Empty then
            declare
               Any_Whole : Boolean := False;
            begin
               for W of E.Views loop
                  Any_Whole := Any_Whole or else not W.Beyond;
               end loop;
               if Any_Whole then
                  --  拿上一次解出来的外包核(外包只会越雕越小,旧一点的照样兜得住它 —— 前提是它没动,正是要核的);没解过 ⇒ 这一眼不核
                  if Fresh and then not E.Solid.Is_Empty and then Off_Hull (E, Vv) then
                     Moved (E);
                     Append (E.Note, "; a new look did not fit the hull carved before: it moved, the older looks were dropped");
                  end if;
               end if;
            end;
         end if;
      end;
      E.Views.Append (Vv);
      E.Valid := False;
   end Add_View;

   procedure Add_Touch (E : in out Estimate; T : Touch) is
   begin
      E.Touches.Append (T);
      E.Valid := False;
   end Add_Touch;

   procedure Add_Point (E : in out Estimate; P : Seen_Pt) is
   begin
      E.Points.Append (P);
      E.Valid := False;
   end Add_Point;

   procedure Set_Support (E : in out Estimate; P, N : V3) is
      Nn : constant Long_Float := Norm (N);
   begin
      if Nn > 0.0 then
         declare
            Un : constant V3 := [N (0) / Nn, N (1) / Nn, N (2) / Nn];
         begin
            if not E.Has_Support or else E.Support_P /= P or else E.Support_N /= Un then
               E.Has_Support := True; E.Support_P := P; E.Support_N := Un;
               E.Valid := False;
            end if;
         end;
      end if;
   end Set_Support;

   procedure Moved (E : in out Estimate) is
   begin
      E.Views.Clear; E.Touches.Clear; E.Points.Clear; E.Valid := False; E.Surface.Clear; E.Solid.Clear; E.Unseen.Clear; E.Inconsistent := False;
      E.Moves := E.Moves + 1;
   end Moved;

   procedure Solve_Once (E : in out Estimate; Need : Long_Float) is separate;

   procedure Solve (E : in out Estimate; Need : Long_Float := 0.0) is
      Newest : Natural := 0;
      Older : Boolean := False;
   begin
      Solve_Once (E, Need);
      if not E.Inconsistent then
         return;
      end if;
      --  几眼对不上:它在几眼之间动过 ⇒ 只留最新那一拍的几眼(和那一拍以后碰到的点、量到的点无从分拍,一起作废)再解一次
      for V of E.Views loop
         Newest := Natural'Max (Newest, V.Seq);
      end loop;
      for V of E.Views loop
         Older := Older or else V.Seq < Newest;
      end loop;
      if not Older then
         return;
      end if;
      for I in reverse 0 .. Natural (E.Views.Length) - 1 loop
         if E.Views (I).Seq < Newest then
            E.Views.Delete (I);
         end if;
      end loop;
      E.Touches.Clear; E.Points.Clear;
      E.Moves := E.Moves + 1;
      Solve_Once (E, Need);
      Append (E.Note, "; the looks did not agree with each other: it moved between them, only the newest beat's looks were kept");
   end Solve;

   procedure Clear_All is
   begin
      Store.Clear;
   end Clear_All;
   function Index_Of (Name : String) return Integer is
   begin
      for I in 0 .. Natural (Store.Length) - 1 loop
         if To_String (Store (I).Name) = Name then
            return I;
         end if;
      end loop;
      return -1;
   end Index_Of;
   function Get (Name : String) return Estimate is
      I : constant Integer := Index_Of (Name);
      E : Estimate;
   begin
      if I >= 0 then
         return Store (Natural (I));
      end if;
      E.Name := To_Unbounded_String (Name);
      return E;
   end Get;
   function Solved (Name : String; Need : Long_Float := 0.0) return Estimate is
      I : constant Integer := Index_Of (Name);
   begin
      if I < 0 then
         return Get (Name);
      end if;
      declare
         E : Estimate := Store (Natural (I));
      begin
         if not E.Valid then
            Solve (E, Need);
            Store.Replace_Element (Natural (I), E);
         end if;
         return E;
      end;
   end Solved;
   procedure Put (E : Estimate) is
      I : constant Integer := Index_Of (To_String (E.Name));
   begin
      if I >= 0 then
         Store.Replace_Element (Natural (I), E);
      else
         Store.Append (E);
      end if;
   end Put;
   procedure Rename (From, To : String) is
      Fi : constant Integer := Index_Of (From);
      Ti : constant Integer := Index_Of (To);
   begin
      if Fi < 0 or else From = To then
         return;
      end if;
      if Ti < 0 then
         declare
            E : Estimate := Store (Natural (Fi));
         begin
            E.Name := To_Unbounded_String (To);
            Store.Replace_Element (Natural (Fi), E);
         end;
         return;
      end if;
      declare
         T : Estimate := Store (Natural (Ti));
         F : constant Estimate := Store (Natural (Fi));
      begin
         for V of F.Views loop
            T.Views.Append (V);
         end loop;
         for Tc of F.Touches loop
            T.Touches.Append (Tc);
         end loop;
         for Pc of F.Points loop
            T.Points.Append (Pc);
         end loop;
         if not T.Has_Support and then F.Has_Support then
            T.Has_Support := True; T.Support_P := F.Support_P; T.Support_N := F.Support_N;
         end if;
         T.Valid := False;
         Store.Replace_Element (Natural (Ti), T);
         Store.Delete (Natural (Fi));
      end;
   end Rename;
   function Count return Natural is (Natural (Store.Length));
   function Get_At (I : Natural) return Estimate is (Store (I));
end Things;
