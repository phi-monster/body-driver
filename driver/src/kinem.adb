with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Calendar;
with Ada.Containers.Ordered_Maps;
with Ada.Containers.Ordered_Sets;
with Stats;
package body Kinem is

   function Axis_Of (A : Axes; J : Natural) return Axis is
     (if J < Natural (A.V.Length) then A.V.Element (J) else (others => <>));
   function Axis_Ref (A : aliased in out Axes; J : Natural) return Axis_Vectors.Reference_Type is
   begin
      if J >= Natural (A.V.Length) then
         A.V.Append (Axis'(others => <>), Ada.Containers.Count_Type (J + 1 - Natural (A.V.Length)));
      end if;
      return A.V.Reference (J);
   end Axis_Ref;

   --  ── Huber(残差以量到的 σ 为单位,门 Huber_K)──
   --  代价:门里 ½r²,门外 δ(|r| − δ/2);迭代加权的权 = ψ(r) / r:门里 1,门外 δ/|r|
   function Huber_Rho (R : Long_Float) return Long_Float is
     (if abs R <= Huber_K then 0.5 * R ** 2 else Huber_K * (abs R - 0.5 * Huber_K));
   function Huber_W (R : Long_Float) return Long_Float is (if abs R <= Huber_K then 1.0 else Huber_K / abs R);
   --  Tukey 双权的权(残差以量到的 σ 为单位):门里 (1 − (r/c)²)²,门外 0
   function Tukey_W (R : Long_Float) return Long_Float is (if abs R >= Tukey_C then 0.0 else (1.0 - (R / Tukey_C) ** 2) ** 2);

   --  ── 小向量 ──
   function Cross (A, B : V3) return V3 is
     ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Scl (A : V3; S : Long_Float) return V3 is ([A (0) * S, A (1) * S, A (2) * S]);
   function ApT (A : M3; X : V3) return V3 is
     ([A (0, 0) * X (0) + A (1, 0) * X (1) + A (2, 0) * X (2),
       A (0, 1) * X (0) + A (1, 1) * X (1) + A (2, 1) * X (2),
       A (0, 2) * X (0) + A (1, 2) * X (1) + A (2, 2) * X (2)]);
   function Unit (A : V3) return V3 is
      N : constant Long_Float := Norm (A);
   begin
      return (if N > 1.0e-15 then Scl (A, 1.0 / N) else [0.0, 0.0, 1.0]);
   end Unit;
   function Rot (W : V3; Th : Long_Float) return M3 is (Rodrigues ([W (0) * Th, W (1) * Th, W (2) * Th]));
   --  垂直于 W 的一对单位向量(轴在眼的哪边按它俩的角度 φ 记)
   procedure Perp (W : V3; E1, E2 : out V3) is
      --  挑一个不跟 W 平行的辅助方向(纯数学,无量纲:|W_x| < 0.9 就用 x 轴)
      A : constant V3 := (if abs W (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
   begin
      E1 := Unit (Cross (W, A));
      E2 := Cross (W, E1);
   end Perp;
   --  球面上铺得最匀的 N 个方向(斐波那契)
   function Sphere (K, N : Natural) return V3 is
      Z : constant Long_Float := 1.0 - 2.0 * (Long_Float (K) + 0.5) / Long_Float (N);
      Rr : constant Long_Float := Sqrt (Long_Float'Max (0.0, 1.0 - Z * Z));
      --  黄金角(纯数学,无量纲):π(1 + √5)
      Ph : constant Long_Float := Ada.Numerics.Pi * (1.0 + Sqrt (5.0)) * (Long_Float (K) + 0.5);
   begin
      return [Rr * Cos (Ph), Rr * Sin (Ph), Z];
   end Sphere;
   function Ang_W (A, B : Long_Float) return V3 is ([Sin (A) * Cos (B), Sin (A) * Sin (B), Cos (A)]);

   type Unsigned_Seed is mod 2 ** 64;
   type Vec_Ptr is access Vec;
   procedure Free is new Ada.Unchecked_Deallocation (Vec, Vec_Ptr);
   type Mat is array (Natural range <>, Natural range <>) of Long_Float;
   type Mat_Ptr is access Mat;
   procedure Free is new Ada.Unchecked_Deallocation (Mat, Mat_Ptr);

   --  中位数(拷一份,按值排;数不多,插入排序足够快 —— 大的用快速选择)
   function Median_Abs (R : Vec) return Long_Float is
      W : Vec_Ptr := new Vec (0 .. R'Length - 1);
      K : Natural;
      Lo, Hi : Integer;
      Res : Long_Float;
   begin
      if R'Length = 0 then
         Free (W);
         return 0.0;
      end if;
      for I in R'Range loop
         W (I - R'First) := abs R (I);
      end loop;
      K := R'Length / 2;
      Lo := 0; Hi := R'Length - 1;
      while Lo < Hi loop
         declare
            Pv : constant Long_Float := W ((Lo + Hi) / 2);
            I : Integer := Lo;
            J : Integer := Hi;
            T : Long_Float;
         begin
            while I <= J loop
               while W (I) < Pv loop
                  I := I + 1;
               end loop;
               while W (J) > Pv loop
                  J := J - 1;
               end loop;
               if I <= J then
                  T := W (I); W (I) := W (J); W (J) := T;
                  I := I + 1; J := J - 1;
               end if;
            end loop;
            if K <= J then
               Hi := J;
            elsif K >= I then
               Lo := I;
            else
               exit;
            end if;
         end;
      end loop;
      Res := W (K);
      Free (W);
      return Res;
   end Median_Abs;

   function Quantile_Abs (R : Vec; Q : Long_Float) return Long_Float is
      --  九成分位只报数,插入排序在几十万个上太慢 ⇒ 按直方图(对数分档)估,分辨率 1%(比例)
      Nb : constant := 2000;   --  分档数(次数)
      Cnt : array (0 .. Nb - 1) of Natural := [others => 0];
      Mx : Long_Float := 0.0;
   begin
      if R'Length = 0 then
         return 0.0;
      end if;
      for X of R loop
         Mx := Long_Float'Max (Mx, abs X);
      end loop;
      if Mx <= 0.0 then
         return 0.0;
      end if;
      for X of R loop
         Cnt (Natural'Min (Nb - 1, Natural (Long_Float'Floor (abs X / Mx * Long_Float (Nb - 1))))) :=
           Cnt (Natural'Min (Nb - 1, Natural (Long_Float'Floor (abs X / Mx * Long_Float (Nb - 1))))) + 1;
      end loop;
      declare
         Need : constant Natural := Natural (Long_Float'Ceiling (Q * Long_Float (R'Length)));
         Acc : Natural := 0;
      begin
         for B in Cnt'Range loop
            Acc := Acc + Cnt (B);
            if Acc >= Need then
               return (Long_Float (B) + 1.0) / Long_Float (Nb - 1) * Mx;
            end if;
         end loop;
      end;
      return Mx;
   end Quantile_Abs;

   --  ── 位姿 ──
   procedure FK (M : Model; Q : Floats; R : out M3; T : out V3) is
   begin
      R := Identity; T := [0.0, 0.0, 0.0];
      for I in 0 .. M.N - 1 loop
         declare
            A : constant Axis := M.Ax (I);
            Th : constant Long_Float := (if I < Natural (Q.Length) and then I < Natural (M.Q0.Length) then Q (I) - M.Q0 (I) else 0.0);
         begin
            if A.Slide then
               T := Add (T, Ap (R, Scl (A.W, Th)));   --  沿 W 走 θ 个读数单位,不转
            else
               declare
                  Ri : constant M3 := Rot (A.W, Th);
                  Ti : constant V3 := Sub (A.P, Ap (Ri, A.P));
               begin
                  T := Add (T, Ap (R, Ti));
                  R := Mul (R, Ri);
               end;
            end if;
         end;
      end loop;
   end FK;

   --  Sampson 残差(像素):X_j = Rij · X_i + Tij;像素按 Geom 的约定换成相机系视线(-z 朝前、+y 朝上)。Tij 不用归一(分子分母同比例)
   function Samp (Rij : M3; Tij : V3; F, Cx, Cy : Long_Float; C : Corr) return Long_Float is
      H1 : constant V3 := [(C.Ua - Cx) / F, -(C.Va - Cy) / F, -1.0];
      H2 : constant V3 := [(C.Ub - Cx) / F, -(C.Vb - Cy) / F, -1.0];
      Y : constant V3 := Ap (Rij, H1);
      Ex1 : constant V3 := Cross (Tij, Y);
      Etx2 : constant V3 := ApT (Rij, Cross (H2, Tij));
      Den : constant Long_Float := Sqrt (Ex1 (0) ** 2 + Ex1 (1) ** 2 + Etx2 (0) ** 2 + Etx2 (1) ** 2) + 1.0e-18;
   begin
      return F * Dot (H2, Ex1) / Den;
   end Samp;

   procedure Rel (Ri : M3; Ti : V3; Rj : M3; Tj : V3; Rij : out M3; Tij : out V3) is
   begin
      Rij := Mul (Tr (Rj), Ri);
      Tij := ApT (Rj, Sub (Ti, Tj));
   end Rel;

   type Frame_Pose is record
      R : M3 := Identity;
      T : V3 := [0.0, 0.0, 0.0];
   end record;
   type Pose_Array is array (Natural range <>) of Frame_Pose;
   procedure All_Poses (M : Model; Frames : Frame_Vectors.Vector; P : out Pose_Array) is
   begin
      for Fr in P'Range loop
         FK (M, Frames (Fr).Q, P (Fr).R, P (Fr).T);
      end loop;
   end All_Poses;
   function Res_Cached (M : Model; P : Pose_Array; C : Corr) return Long_Float is
      Rij : M3;
      Tij : V3;
   begin
      Rel (P (C.I).R, P (C.I).T, P (C.J).R, P (C.J).T, Rij, Tij);
      return Samp (Rij, Tij, M.F, M.Cx, M.Cy, C);
   end Res_Cached;

   --  配点按对挨着放:每个配点在它那一对里排第几、那一对一共几个
   procedure Pair_Pos (Cs : Corr_Vectors.Vector; Pos, Size : out Nat_Vectors.Vector) is
      Start : Natural := 0;
   begin
      Pos.Clear; Size.Clear;
      for K in 0 .. Natural (Cs.Length) - 1 loop
         if K > 0 and then (Cs (K).I /= Cs (K - 1).I or else Cs (K).J /= Cs (K - 1).J) then
            for X in Start .. K - 1 loop
               Size.Append (K - Start);
            end loop;
            Start := K;
         end if;
         Pos.Append (K - Start);
      end loop;
      for X in Start .. Natural (Cs.Length) - 1 loop
         Size.Append (Natural (Cs.Length) - Start);
      end loop;
   end Pair_Pos;
   --  这一对里要不要这个(一对最多 K 个,在对里均匀隔着取 —— 格点是一行一行排的,取前 K 个就全挤在画面最上面几行)
   function Take (Pos, Size, K : Natural) return Boolean is
      Stride : constant Positive := Positive'Max (1, (Size + K - 1) / Natural'Max (1, K));
   begin
      return Pos mod Stride = 0 and then Pos / Stride < K;
   end Take;
   function Thin (Cs : Corr_Vectors.Vector; K : Natural) return Corr_Vectors.Vector is
      Out_Cs : Corr_Vectors.Vector;
      Pos, Size : Nat_Vectors.Vector;
   begin
      Pair_Pos (Cs, Pos, Size);
      for I in 0 .. Natural (Cs.Length) - 1 loop
         if Take (Pos (I), Size (I), K) then
            Out_Cs.Append (Cs (I));
         end if;
      end loop;
      return Out_Cs;
   end Thin;

   --  这批配点像素坐标的数值分辨率:ε × 坐标里绝对值最大的那个(残差比它小就分不出是不是 0)。
   --  量到的噪声拿来当除数时按它兜底 —— 只防零:数据一点不差时残差一半以上正好是 0,中位就是 0
   function Px_Res (Cs : Corr_Vectors.Vector) return Long_Float is
      Mx : Long_Float := 0.0;
   begin
      for C of Cs loop
         Mx := Long_Float'Max (Mx, Long_Float'Max (Long_Float'Max (abs C.Ua, abs C.Va), Long_Float'Max (abs C.Ub, abs C.Vb)));
      end loop;
      return Long_Float'Max (Long_Float'Model_Epsilon * Mx, Long_Float'Model_Small);
   end Px_Res;
   --  一维残差的中位 ⇒ 量到的噪声 σ(Mad_Sigma 换算;下限 Floor 只防零,见 Px_Res)
   function Sigma_Of (Med, Floor : Long_Float) return Long_Float is (Long_Float'Max (Mad_Sigma * Med, Floor));

   --  ── 抗野点的 LM(数值雅可比;Huber 迭代加权,残差以量到的 σ 为单位)──
   --  Resid 把全部残差填进 R(长度 N_R);只有前 N_Rob 个按 Huber 加权(配点),后面的(约束行)原样。
   --  阻尼升降的两个倍数只管这次拟合怎么迭代,不影响身体动不动
   procedure Robust_LM (X : in out Vec; N_R, N_Rob : Natural; Iters : Positive; Step : Vec;
                        Resid : not null access procedure (X : Vec; R : out Vec); Done : out Boolean) is
      Np : constant Natural := X'Length;
      R0 : Vec_Ptr := new Vec (0 .. N_R - 1);
      Rp : Vec_Ptr := new Vec (0 .. N_R - 1);
      Rn : Vec_Ptr := new Vec (0 .. N_R - 1);
      Jc : Mat_Ptr := new Mat (0 .. N_R - 1, 0 .. Np - 1);
      Wt : Vec_Ptr := new Vec (0 .. N_R - 1);
      Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
      Up : constant := 10.0;        --  阻尼放大倍数(次数)
      Dn : constant := 3.0;         --  阻尼缩小倍数(次数)
      function Cost (R : Vec) return Long_Float is
         S : Long_Float := 0.0;
      begin
         for I in R'Range loop
            S := S + (if I < N_Rob then Huber_Rho (R (I)) else 0.5 * R (I) ** 2);
         end loop;
         return S;
      end Cost;
      C0 : Long_Float;
   begin
      Done := False;
      Resid (X, R0.all);
      C0 := Cost (R0.all);
      for It in 1 .. Iters loop
         for I in 0 .. N_R - 1 loop
            Wt (I) := (if I < N_Rob then Huber_W (R0 (I)) else 1.0);
         end loop;
         for K in 0 .. Np - 1 loop
            declare
               Xp : Vec := X;
            begin
               Xp (X'First + K) := Xp (X'First + K) + Step (Step'First + K);
               Resid (Xp, Rp.all);
               for I in 0 .. N_R - 1 loop
                  Jc (I, K) := (Rp (I) - R0 (I)) / Step (Step'First + K);
               end loop;
            end;
         end loop;
         declare
            A : Mat (0 .. Np - 1, 0 .. Np - 1) := [others => [others => 0.0]];
            B : Vec (0 .. Np - 1) := [others => 0.0];
            Improved : Boolean := False;
         begin
            for I in 0 .. N_R - 1 loop
               for K in 0 .. Np - 1 loop
                  declare
                     Jk : constant Long_Float := Jc (I, K) * Wt (I);
                  begin
                     if Jk /= 0.0 then
                        for L in K .. Np - 1 loop
                           A (K, L) := A (K, L) + Jk * Jc (I, L);
                        end loop;
                        B (K) := B (K) - Jk * R0 (I);
                     end if;
                  end;
               end loop;
            end loop;
            for K in 0 .. Np - 1 loop
               for L in 0 .. K - 1 loop
                  A (K, L) := A (L, K);
               end loop;
            end loop;
            --  阻尼一直往上调,直到代价降了、或者步子小到参数的数值分辨率以下(Tiny:X 的每一个数挪不到 ε × 它自己 / 它的差分步那么大)——
            --  不设次数:阻尼按倍数涨,步子一定会小下去(09-30 以前最多调 8 次,阻尼从 1e-9 起只到 0.1 就判到底,平谷里停早)
            loop
               declare
                  Aa : Mat := A;
                  Bb : Vec := B;
                  D : Vec (0 .. Np - 1) := [others => 0.0];
                  Xn : Vec := X;
                  Cn : Long_Float;
                  Tiny : Boolean := True;
               begin
                  for K in 0 .. Np - 1 loop
                     Aa (K, K) := Aa (K, K) * (1.0 + Lam) + 1.0e-12;
                  end loop;
                  --  高斯消元(列主元)
                  for Col in 0 .. Np - 1 loop
                     declare
                        Pv : Natural := Col;
                     begin
                        for Rw in Col + 1 .. Np - 1 loop
                           if abs Aa (Rw, Col) > abs Aa (Pv, Col) then
                              Pv := Rw;
                           end if;
                        end loop;
                        if Pv /= Col then
                           for Cc in 0 .. Np - 1 loop
                              declare
                                 T : constant Long_Float := Aa (Col, Cc);
                              begin
                                 Aa (Col, Cc) := Aa (Pv, Cc); Aa (Pv, Cc) := T;
                              end;
                           end loop;
                           declare
                              T : constant Long_Float := Bb (Col);
                           begin
                              Bb (Col) := Bb (Pv); Bb (Pv) := T;
                           end;
                        end if;
                        if abs Aa (Col, Col) > 1.0e-300 then   --  主元为零保护(数值,无量纲)
                           for Rw in Col + 1 .. Np - 1 loop
                              declare
                                 Fct : constant Long_Float := Aa (Rw, Col) / Aa (Col, Col);
                              begin
                                 if Fct /= 0.0 then
                                    for Cc in Col .. Np - 1 loop
                                       Aa (Rw, Cc) := Aa (Rw, Cc) - Fct * Aa (Col, Cc);
                                    end loop;
                                    Bb (Rw) := Bb (Rw) - Fct * Bb (Col);
                                 end if;
                              end;
                           end loop;
                        end if;
                     end;
                  end loop;
                  for K in reverse 0 .. Np - 1 loop
                     declare
                        S : Long_Float := Bb (K);
                     begin
                        for Cc in K + 1 .. Np - 1 loop
                           S := S - Aa (K, Cc) * D (Cc);
                        end loop;
                        D (K) := (if abs Aa (K, K) > 1.0e-300 then S / Aa (K, K) else 0.0);   --  同上(数值,无量纲)
                     end;
                  end loop;
                  for K in 0 .. Np - 1 loop
                     Xn (X'First + K) := X (X'First + K) + D (K);
                     --  写成"不大于"的反面:算坏了的步子(NaN)也算挪不动,不会在这里转不出去
                     if abs D (K) > Long_Float'Model_Epsilon * Long_Float'Max (abs X (X'First + K), abs Step (Step'First + K)) then
                        Tiny := False;
                     end if;
                  end loop;
                  exit when Tiny;   --  到底了:再小的步子已经挪不动 X
                  Resid (Xn, Rn.all);
                  Cn := Cost (Rn.all);
                  if Cn < C0 then
                     X := Xn; R0.all := Rn.all;
                     Improved := abs (C0 - Cn) > 1.0e-9 * Long_Float'Max (C0, 1.0e-30);
                     C0 := Cn;
                     Lam := Long_Float'Max (1.0e-9, Lam / Dn);
                     exit;
                  else
                     Lam := Lam * Up;
                  end if;
               end;
            end loop;
            if not Improved then
               Done := True;   --  降不动了(这一步降得不到十亿分之一,或者步子已经挪不动 X)
               exit;
            end if;
         end;
      end loop;
      Free (R0); Free (Rp); Free (Rn); Free (Jc); Free (Wt);
   end Robust_LM;

   procedure Fit_Eye_Turn (G : Cam_Geo; W, H : Natural; Pu, Pv, Bu, Bv : Vec; Rot : out V3; Sig_Px : out Long_Float; Settled, Fitted : out Boolean) is
      N : constant Natural := Pu'Length;
      Np : constant := 3;   --  转动向量的三个数(结构)
      type Dir_Arr is array (Natural range <>) of V3;
      type Flag_Arr is array (Natural range <>) of Boolean;
      D : Dir_Arr (0 .. Natural'Max (1, N) - 1);            --  每个问的点的视线(这只眼的相机系,去了畸变)
      Ix : array (0 .. Natural'Max (1, N) - 1) of Natural := [others => 0];  --  进拟合的第 J 个点是第几个问的点
      Nu : Natural := 0;   --  进拟合的点数
      X : Vec (0 .. Np - 1) := [0.0, 0.0, 0.0];   --  从"没转"起
      --  差分步:转角是弧度、1 的量级以下,前向差分最好的步子 = √ε(数值)
      Steps : constant Vec (0 .. Np - 1) := [others => Sqrt (Long_Float'Model_Epsilon)];
      Sig : Long_Float := 1.0;   --  残差除以它(像素);头一遍最小二乘不按它,量出来以后才换成真的
      procedure Map (Xx : Vec; K : Natural; Hu, Hv : out Long_Float; Front : out Boolean) is
         Rm : constant M3 := Rodrigues ([Xx (Xx'First), Xx (Xx'First + 1), Xx (Xx'First + 2)]);
      begin
         Cam_Pixel (G, Ap (Rm, D (K)), Hu, Hv, Front);
      end Map;
   begin
      Rot := [0.0, 0.0, 0.0];
      Sig_Px := 0.0;
      Settled := True;
      Fitted := False;
      if not (G.F > 0.0) then
         return;   --  这只眼没有量过的焦距:转动投不回去
      end if;
      for I in 0 .. N - 1 loop
         declare
            Ok : Boolean;
         begin
            D (I) := Cam_Dir (G, Pu (Pu'First + I), Pv (Pv'First + I), Ok);
            if Ok then
               Ix (Nu) := I;
               Nu := Nu + 1;
            end if;
         end;
      end loop;
      if 2 * Nu <= Np then
         return;   --  方程不比转动的数多:解不出、也量不了 σ
      end if;
      declare
         Wt : Vec (0 .. 2 * Nu - 1) := [others => 1.0];   --  每一条残差的权(Tukey;头一遍全是 1 = 最小二乘)
         --  不加权的残差(像素);转到眼后面去的点(小转动不会有)记成 0 残差、权也给 0
         procedure Raw (Xx : Vec; Rr : out Vec) is
            Hu, Hv : Long_Float;
            Front : Boolean;
         begin
            for J in 0 .. Nu - 1 loop
               Map (Xx, Ix (J), Hu, Hv, Front);
               if Front then
                  Rr (Rr'First + 2 * J) := Hu - Bu (Bu'First + Ix (J));
                  Rr (Rr'First + 2 * J + 1) := Hv - Bv (Bv'First + Ix (J));
               else
                  Rr (Rr'First + 2 * J) := 0.0;
                  Rr (Rr'First + 2 * J + 1) := 0.0;
               end if;
            end loop;
         end Raw;
         --  进最小二乘的:√权 × 残差 / σ
         procedure Resid (Xx : Vec; Rr : out Vec) is
         begin
            Raw (Xx, Rr);
            for K in Rr'Range loop
               Rr (K) := Sqrt (Wt (K - Rr'First)) * Rr (K) / Sig;
            end loop;
         end Resid;
         Rr : Vec (0 .. 2 * Nu - 1);
         Outside, Prev : Flag_Arr (0 .. 2 * Nu - 1) := [others => False];
         Done : Boolean;
         Stopped : Boolean := False;
      begin
         --  头一遍最小二乘(权全是 1):世界点占多数时落得离它们近
         Robust_LM (X, 2 * Nu, 0, Positive'Last, Steps, Resid'Access, Done);
         for Round in 1 .. 2 * Nu loop   --  保险:门外那批残差每一轮至少换掉一条,换满残差条数那么多轮还在变 ⇒ 照实报
            Raw (X, Rr);
            --  按此刻的转动,它要是世界就会转出画幅的点:转出去那一帧里没有它的真对应,配点仪器交回的是编的 ⇒ 这一轮不进拟合、也不进量 σ
            --  (09-30 合成的眼:出了画幅的 83 个配回原处附近,连同手指占了三成,不排除 ⇒ 起点停在两边中间、σ 量成 31 px、全判不了)
            declare
               Gone : Flag_Arr (0 .. Nu - 1) := [others => False];
               N_In : Natural := 0;
            begin
               for J in 0 .. Nu - 1 loop
                  declare
                     Hu, Hv : Long_Float;
                     Front : Boolean;
                  begin
                     Map (X, Ix (J), Hu, Hv, Front);
                     Gone (J) := not Front or else Hu < 0.0 or else Hv < 0.0 or else Hu >= Long_Float (W) or else Hv >= Long_Float (H);
                     if not Gone (J) then
                        N_In := N_In + 1;
                     end if;
                  end;
               end loop;
               exit when 2 * N_In <= Np;   --  留在画幅里的不够解:照上一遍的交(Stopped 仍是 False ⇒ 照实报没收住)
               declare
                  Rin : Vec (0 .. 2 * N_In - 1);
                  Jn : Natural := 0;
               begin
                  for J in 0 .. Nu - 1 loop
                     if not Gone (J) then
                        Rin (2 * Jn) := Rr (Rr'First + 2 * J);
                        Rin (2 * Jn + 1) := Rr (Rr'First + 2 * J + 1);
                        Jn := Jn + 1;
                     end if;
                  end loop;
                  declare
                     Md : constant Long_Float := Median_Abs (Rin);
                  begin
                     if not (Md > 0.0) then
                        Stopped := True;   --  拟合得分毫不差(合成的无噪声点):没有野点要抗
                        exit;
                     end if;
                     Sig := Mad_Sigma * Md;
                  end;
               end;
               for J in 0 .. Nu - 1 loop
                  for C in 0 .. 1 loop
                     Wt (2 * J + C) := (if Gone (J) then 0.0 else Tukey_W (Rr (Rr'First + 2 * J + C) / Sig));
                     Outside (2 * J + C) := Wt (2 * J + C) = 0.0;
                  end loop;
               end loop;
            end;
            if Round > 1 and then Outside = Prev then
               Stopped := True;   --  门外那批不再变:上一遍按的就是这批权(门里的权值跟着残差走,门外的是 0)
               exit;
            end if;
            Prev := Outside;
            Robust_LM (X, 2 * Nu, 0, Positive'Last, Steps, Resid'Access, Done);
         end loop;
         Settled := Stopped;
      end;
      Sig_Px := Sig;
      Rot := [X (0), X (1), X (2)];
      Fitted := True;
   end Fit_Eye_Turn;

   procedure Classify_Rides (G : Cam_Geo; Rot : V3; Sig_Px : Long_Float; W, H : Natural; Pu, Pv, Bu, Bv : Vec; R : out Ride_Vec) is
      Rm : constant M3 := Rodrigues (Rot);
   begin
      R := [others => Unknown];
      if not (G.F > 0.0) then
         return;
      end if;
      for I in 0 .. Pu'Length - 1 loop
         declare
            Ok : Boolean;
            Dc : constant V3 := Cam_Dir (G, Pu (Pu'First + I), Pv (Pv'First + I), Ok);
            Hu, Hv : Long_Float;
            Front : Boolean;
            U0 : constant Long_Float := Pu (Pu'First + I);
            V0 : constant Long_Float := Pv (Pv'First + I);
            U1 : constant Long_Float := Bu (Bu'First + I);
            V1 : constant Long_Float := Bv (Bv'First + I);
         begin
            if Ok then
               Cam_Pixel (G, Ap (Rm, Dc), Hu, Hv, Front);
               if not Front or else Sqrt ((Hu - U0) ** 2 + (Hv - V0) ** 2) < 2.0 * Stats.Z * Sig_Px then
                  R (R'First + I) := Unknown;   --  两种说法挨得太近:按近的判,判错的概率超过 Z 的单边尾巴
               elsif Hu < 0.0 or else Hv < 0.0 or else Hu >= Long_Float (W) or else Hv >= Long_Float (H) then
                  --  它要是世界,转出去以后就出了画面:转出去那一帧里没有它的真对应,配点仪器交回来的是编的(V1B73 第 1 只手:
                  --  画面整片往右下挪 60 px,右边那一条被判成长在眼上、连进右边那一瓣,瓣框长到半幅画面)⇒ 判不了
                  R (R'First + I) := Unknown;
               elsif Sqrt ((U1 - U0) ** 2 + (V1 - V0) ** 2) < Sqrt ((U1 - Hu) ** 2 + (V1 - Hv) ** 2) then
                  R (R'First + I) := Rides;
               else
                  R (R'First + I) := World;
               end if;
            end if;
         end;
      end loop;
   end Classify_Rides;


   --  ── ① 每根轴单独 ──
   --  一根轴的一组配点(已按帧换成这根轴的转角):θ_a, θ_b = 两帧相对参照的转角
   type Jc_Rec is record
      Ta, Tb : Long_Float := 0.0;
      C : Corr;
   end record;
   package Jc_Vectors is new Ada.Containers.Vectors (Natural, Jc_Rec);
   --  网格 / 精修里按下标反复取:容器每取一次都做一次防篡改的原子计数(自检:网格 129 秒几乎全花在这上面)⇒ 先拷进普通数组
   type Jc_Array is array (Natural range <>) of Jc_Rec;
   type Jc_Array_Ptr is access Jc_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Jc_Array, Jc_Array_Ptr);
   function To_Array (V : Jc_Vectors.Vector) return Jc_Array_Ptr is
      A : constant Jc_Array_Ptr := new Jc_Array (0 .. Natural'Max (1, Natural (V.Length)) - 1);
   begin
      for K in 0 .. Natural (V.Length) - 1 loop
         A (K) := V (K);
      end loop;
      return A;
   end To_Array;

   --  一个线程自己的工作区(一次备好,反复用:多线程时每次在堆上申请会抢分配锁,6 个线程比 1 个还慢)
   type Work is record
      G1, G2, Wt, Rs, Tmp : Vec_Ptr;
   end record;
   function New_Work (N : Natural) return Work is
     (G1 => new Vec (0 .. Natural'Max (1, N) - 1), G2 => new Vec (0 .. Natural'Max (1, N) - 1), Wt => new Vec (0 .. Natural'Max (1, N) - 1),
      Rs => new Vec (0 .. Natural'Max (1, N) - 1), Tmp => new Vec (0 .. Natural'Max (1, N) - 1));
   procedure Free_Work (Wk : in out Work) is
   begin
      Free (Wk.G1); Free (Wk.G2); Free (Wk.Wt); Free (Wk.Rs); Free (Wk.Tmp);
   end Free_Work;
   --  中位数:拷进 Tmp 就地快速选择(不申请内存)
   function Median_In (R : Vec; N : Natural; Tmp : Vec_Ptr) return Long_Float is
      K : Natural;
      Lo, Hi : Integer;
   begin
      if N = 0 then
         return 0.0;
      end if;
      for I in 0 .. N - 1 loop
         Tmp (I) := abs R (R'First + I);
      end loop;
      K := N / 2;
      Lo := 0; Hi := N - 1;
      while Lo < Hi loop
         declare
            Pv : constant Long_Float := Tmp ((Lo + Hi) / 2);
            I : Integer := Lo;
            J : Integer := Hi;
            T : Long_Float;
         begin
            while I <= J loop
               while Tmp (I) < Pv loop
                  I := I + 1;
               end loop;
               while Tmp (J) > Pv loop
                  J := J - 1;
               end loop;
               if I <= J then
                  T := Tmp (I); Tmp (I) := Tmp (J); Tmp (J) := T;
                  I := I + 1; J := J - 1;
               end if;
            end loop;
            if K <= J then
               Hi := J;
            elsif K >= I then
               Lo := I;
            else
               exit;
            end if;
         end;
      end loop;
      return Tmp (K);
   end Median_In;

   --  轴方向 W、焦距 F 给定 ⇒ "轴在眼的哪边"(φ)直接解:x2ᵀ[t]×R x1 = 0,t = (I − R) p ⇒ 对 p 线性:gᵀp = 0,
   --  g = (I − R)ᵀ((R h1) × h2);p 只在垂直于轴的平面里 ⇒ 2×2 的最小特征向量;按 Sampson 换算加权、Huber 重解一遍
   --  (Huber 的残差除以这一遍量到的噪声;Floor = 噪声的下限,只防零,见 Px_Res)
   procedure Best_Phi (Cs : Jc_Array; N : Natural; W : V3; F, Cx, Cy, Floor : Long_Float; Phi : out Long_Float; Med : out Long_Float;
                       Wk : Work; Iters : Natural := 1) is
      E1, E2 : V3;
      G1 : constant Vec_Ptr := Wk.G1;
      G2 : constant Vec_Ptr := Wk.G2;
      Wt : constant Vec_Ptr := Wk.Wt;
      Rs : constant Vec_Ptr := Wk.Rs;
      Cp, Sp : Long_Float := 1.0;
   begin
      Perp (W, E1, E2);
      if N = 0 then
         Phi := 0.0; Med := 1.0e9;   --  没有配点(无量纲的大数)
         return;
      end if;
      declare
         Last_D : Long_Float := Long_Float'Last;
         Rij : M3 := Identity;
      begin
         for K in 0 .. N - 1 loop
            declare
               Cc : constant Jc_Rec := Cs (K);
            begin
               if Cc.Ta - Cc.Tb /= Last_D then   --  同一对的配点挨着放 ⇒ 一对只算一次转动
                  Last_D := Cc.Ta - Cc.Tb; Rij := Rot (W, Last_D);
               end if;
               declare
                  H1 : constant V3 := [(Cc.C.Ua - Cx) / F, -(Cc.C.Va - Cy) / F, -1.0];
                  H2 : constant V3 := [(Cc.C.Ub - Cx) / F, -(Cc.C.Vb - Cy) / F, -1.0];
                  Y : constant V3 := Ap (Rij, H1);
                  C : constant V3 := Cross (Y, H2);
                  G : constant V3 := Sub (C, ApT (Rij, C));
               begin
                  G1 (K) := Dot (G, E1); G2 (K) := Dot (G, E2);
                  Wt (K) := 1.0 / Long_Float'Max (G1 (K) ** 2 + G2 (K) ** 2, 1.0e-30);
               end;
            end;
         end loop;
      end;
      for It in 0 .. Iters loop
         declare
            A11, A12, A22 : Long_Float := 0.0;
         begin
            for K in 0 .. N - 1 loop
               A11 := A11 + Wt (K) * G1 (K) * G1 (K);
               A12 := A12 + Wt (K) * G1 (K) * G2 (K);
               A22 := A22 + Wt (K) * G2 (K) * G2 (K);
            end loop;
            --  2×2 对称矩阵最小特征值的特征向量
            declare
               Tr2 : constant Long_Float := 0.5 * (A11 + A22);
               Dd : constant Long_Float := Sqrt (Long_Float'Max (0.0, 0.25 * (A11 - A22) ** 2 + A12 ** 2));
               Lm : constant Long_Float := Tr2 - Dd;
               Vx, Vy, Nv : Long_Float;
            begin
               if abs A12 > 1.0e-30 then
                  Vx := A12; Vy := Lm - A11;
               elsif A11 <= A22 then
                  Vx := 1.0; Vy := 0.0;
               else
                  Vx := 0.0; Vy := 1.0;
               end if;
               Nv := Sqrt (Vx * Vx + Vy * Vy);
               if Nv > 0.0 then
                  Cp := Vx / Nv; Sp := Vy / Nv;
               end if;
            end;
         end;
         declare
            P : constant V3 := Add (Scl (E1, Cp), Scl (E2, Sp));
            Last_D : Long_Float := Long_Float'Last;
            Rij : M3 := Identity;
         begin
            for K in 0 .. N - 1 loop
               if Cs (K).Ta - Cs (K).Tb /= Last_D then
                  Last_D := Cs (K).Ta - Cs (K).Tb; Rij := Rot (W, Last_D);
               end if;
               declare
                  Cc : constant Jc_Rec := Cs (K);
                  Tij : constant V3 := Sub (P, Ap (Rij, P));
                  H1 : constant V3 := [(Cc.C.Ua - Cx) / F, -(Cc.C.Va - Cy) / F, -1.0];
                  H2 : constant V3 := [(Cc.C.Ub - Cx) / F, -(Cc.C.Vb - Cy) / F, -1.0];
                  Y : constant V3 := Ap (Rij, H1);
                  Ex1 : constant V3 := Cross (Tij, Y);
                  Etx2 : constant V3 := ApT (Rij, Cross (H2, Tij));
                  Den : constant Long_Float := Sqrt (Ex1 (0) ** 2 + Ex1 (1) ** 2 + Etx2 (0) ** 2 + Etx2 (1) ** 2) + 1.0e-18;
                  R : constant Long_Float := F * Dot (H2, Ex1) / Den;
               begin
                  Rs (K) := R;
                  Wt (K) := (F / Den) ** 2;   --  下一遍的权:代数残差换成像素的比例的平方(Huber 那一截等这一遍的噪声量完再乘)
               end;
            end loop;
         end;
         --  下一遍的权再乘 Huber:残差除以这一遍量到的噪声(Mad_Sigma × 中位),门 Huber_K;最后一遍的权用不着,不量
         if It < Iters then
            declare
               Sig : constant Long_Float := Sigma_Of (Median_In (Rs.all, N, Wk.Tmp), Floor);
            begin
               for K in 0 .. N - 1 loop
                  Wt (K) := Wt (K) * Huber_W (Rs (K) / Sig);
               end loop;
            end;
         end if;
      end loop;
      Phi := Arctan (Sp, Cp);
      Med := Median_In (Rs.all, N, Wk.Tmp);
   end Best_Phi;

   --  一根轴、一组配点、给定 (ω 两个角, φ):全部 Sampson 残差
   procedure Joint_Res (Cs : Jc_Array; N : Natural; X : Vec; F, Cx, Cy : Long_Float; R : out Vec) is
      W : constant V3 := Ang_W (X (X'First), X (X'First + 1));
      E1, E2 : V3;
   begin
      Perp (W, E1, E2);
      declare
         P : constant V3 := Add (Scl (E1, Cos (X (X'First + 2))), Scl (E2, Sin (X (X'First + 2))));
         Last_D : Long_Float := Long_Float'Last;
         Rij : M3 := Identity;
      begin
         for K in 0 .. N - 1 loop
            if Cs (K).Ta - Cs (K).Tb /= Last_D then
               Last_D := Cs (K).Ta - Cs (K).Tb; Rij := Rot (W, Last_D);
            end if;
            R (R'First + K) := Samp (Rij, Sub (P, Ap (Rij, P)), F, Cx, Cy, Cs (K).C);
         end loop;
      end;
   end Joint_Res;

   --  "沿轴走"那样:一根轴、一组配点、给定走的方向(两个角):两帧之间不转、只平移,平移的方向 = W(长短、正负 Sampson 分不出)⇒ 全部 Sampson 残差
   procedure Slide_Res (Cs : Jc_Array; N : Natural; X : Vec; F, Cx, Cy : Long_Float; R : out Vec) is
      W : constant V3 := Ang_W (X (X'First), X (X'First + 1));
   begin
      for K in 0 .. N - 1 loop
         R (R'First + K) := Samp (Identity, W, F, Cx, Cy, Cs (K).C);
      end loop;
   end Slide_Res;

   type Cand is record
      Score : Long_Float := 1.0e18;   --  空位(比所有真分数都大;无量纲)
      W : V3 := [0.0, 0.0, 1.0];
      Phi : Long_Float := 0.0;
   end record;
   Keep : constant := 20;   --  每档焦距留几个候选(次数)
   type Cand_Array is array (0 .. Keep - 1) of Cand;
   N_F : constant := 25;    --  焦距档数(次数):视场 30°–110°,每档约 7%
   type Cand_Table is array (0 .. N_F - 1) of Cand_Array;

   --  候选插进一档:按分数排好,彼此差不到 10° 的只留好的那个(同一个坑只留一个);Axial = W 和 −W 算同一个(走的方向)
   procedure Insert (T : in out Cand_Array; C : Cand; Axial : Boolean := False) is
      Same : constant := 0.984807753012208;   --  cos 10°(协议:同一个坑的宽度)
   begin
      for I in T'Range loop
         if (if Axial then abs Dot (T (I).W, C.W) else Dot (T (I).W, C.W)) > Same then
            if C.Score < T (I).Score then
               T (I) := C;
               --  重新排
               for J in reverse T'First + 1 .. I loop
                  if T (J).Score < T (J - 1).Score then
                     declare
                        Tmp : constant Cand := T (J);
                     begin
                        T (J) := T (J - 1); T (J - 1) := Tmp;
                     end;
                  end if;
               end loop;
            end if;
            return;
         end if;
      end loop;
      if C.Score < T (T'Last).Score then
         T (T'Last) := C;
         for J in reverse T'First + 1 .. T'Last loop
            if T (J).Score < T (J - 1).Score then
               declare
                  Tmp : constant Cand := T (J);
               begin
                  T (J) := T (J - 1); T (J - 1) := Tmp;
               end;
            end if;
         end loop;
      end if;
   end Insert;

   N_Sph : constant := 3000;       --  轴方向网格点数(次数;相邻约 3.7°)
   Grid_Rad : constant := 0.279252680319093;   --  网格只用两帧之间转角 ≤ 16°(= 0.2793 弧度)的配点(协议:坑宽 —— 转角大的对坑太窄,网格点落不进去,LAB 09-26)
   Per_Pair_Grid : constant := 30; --  网格上每一对最多取几个配点(次数)
   Per_Pair_All : constant := 200; --  精修 / 定比例时每一对最多取几个配点(次数)
   Grid_Target : constant := 1200; --  每根轴单独起步时网格一共用多少个配点(次数)
   All_Target : constant := 8000;  --  每根轴单独精修时一共用多少个配点(次数)
   Axis_Iters : constant := 60;    --  ①、①b 每一次 LM 最多几次(保险:收住了就停;做满还在降照实记进 Rep.Unsettled)

   --  "沿轴走"那样的网格:走的方向铺半个球面(斐波那契点的前一半 z > 0;W 和 −W 是同一种走法),分数 = 残差中位,焦距给定。
   --  纯平移时焦距只改方向的斜度:像素里的对极点(平移的"消失点")不随焦距变 ⇒ 最好的分数跟焦距无关,焦距只能由"转"的轴定
   procedure Slide_Grid (Cs : Jc_Array; N : Natural; F, Cx, Cy : Long_Float; T : out Cand_Array; Wk : Work) is
   begin
      T := [others => <>];
      if N = 0 then
         return;
      end if;
      for Ks in 0 .. N_Sph / 2 - 1 loop
         declare
            W : constant V3 := Sphere (Ks, N_Sph);
         begin
            for K in 0 .. N - 1 loop
               Wk.Rs (K) := Samp (Identity, W, F, Cx, Cy, Cs (K).C);
            end loop;
            Insert (T, (Score => Median_In (Wk.Rs.all, N, Wk.Tmp), W => W, Phi => 0.0), Axial => True);
         end;
      end loop;
   end Slide_Grid;

   --  ── ② 定比例用 ──
   type Nat_Array is array (Natural range <>) of Natural;
   --  一个配点:对极约束 g · ρ = 0,Sampson 的分母 = |(E0, E1, E2, E3) · ρ|(Ex1、Etx2 的前两个分量,同 Samp);每段一根轴一个数(Last = 轴数 − 1)
   type Rho_Row (Last : Integer) is record
      G, E0, E1, E2, E3 : Vec (0 .. Last) := [others => 0.0];
   end record;
   function Rho_Res (R : Rho_Row; Rho : Vec; F : Long_Float; Den : out Long_Float) return Long_Float is
      Num, D0, D1, D2, D3 : Long_Float := 0.0;
   begin
      for J in 0 .. R.Last loop
         Num := Num + R.G (J) * Rho (Rho'First + J);
         D0 := D0 + R.E0 (J) * Rho (Rho'First + J);
         D1 := D1 + R.E1 (J) * Rho (Rho'First + J);
         D2 := D2 + R.E2 (J) * Rho (Rho'First + J);
         D3 := D3 + R.E3 (J) * Rho (Rho'First + J);
      end loop;
      Den := Sqrt (D0 * D0 + D1 * D1 + D2 * D2 + D3 * D3) + 1.0e-18;
      return F * Num / Den;
   end Rho_Res;
   --  对称阵(前 N × N)最小特征值的特征向量(循环 Jacobi,同 Max_Eigvec4 的转法)
   function Min_Eig (A0 : Mat; N : Natural) return Vec is
      A : Mat (0 .. N - 1, 0 .. N - 1);
      V : Mat (0 .. N - 1, 0 .. N - 1) := [others => [others => 0.0]];
      Best : Natural := 0;
      Out_V : Vec (0 .. N - 1) := [others => 0.0];
   begin
      for I in 0 .. N - 1 loop
         for J in 0 .. N - 1 loop
            A (I, J) := A0 (A0'First (1) + I, A0'First (2) + J);
         end loop;
         V (I, I) := 1.0;
      end loop;
      for Sweep in 1 .. 100 loop   --  最多 100 遍(次数)
         declare
            Off, Dg : Long_Float := 0.0;
         begin
            for P in 0 .. N - 1 loop
               Dg := Dg + A (P, P) ** 2;
               for Q in P + 1 .. N - 1 loop
                  Off := Off + A (P, Q) ** 2;
               end loop;
            end loop;
            exit when Off <= 1.0e-30 * Long_Float'Max (Dg, 1.0e-300);   --  非对角元相对已经是零(数值,无量纲)
            for P in 0 .. N - 2 loop
               for Q in P + 1 .. N - 1 loop
                  if abs A (P, Q) > 1.0e-300 then   --  数值保护(无量纲)
                     declare
                        Th : constant Long_Float := 0.5 * Arctan (2.0 * A (P, Q), A (Q, Q) - A (P, P));
                        C : constant Long_Float := Cos (Th);
                        Sn : constant Long_Float := Sin (Th);
                     begin
                        for K in 0 .. N - 1 loop
                           declare
                              Akp : constant Long_Float := A (K, P);
                              Akq : constant Long_Float := A (K, Q);
                           begin
                              A (K, P) := C * Akp - Sn * Akq;
                              A (K, Q) := Sn * Akp + C * Akq;
                           end;
                        end loop;
                        for K in 0 .. N - 1 loop
                           declare
                              Apk : constant Long_Float := A (P, K);
                              Aqk : constant Long_Float := A (Q, K);
                           begin
                              A (P, K) := C * Apk - Sn * Aqk;
                              A (Q, K) := Sn * Apk + C * Aqk;
                           end;
                        end loop;
                        for K in 0 .. N - 1 loop
                           declare
                              Vkp : constant Long_Float := V (K, P);
                              Vkq : constant Long_Float := V (K, Q);
                           begin
                              V (K, P) := C * Vkp - Sn * Vkq;
                              V (K, Q) := Sn * Vkp + C * Vkq;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
            end loop;
         end;
      end loop;
      for I in 1 .. N - 1 loop
         if A (I, I) < A (Best, Best) then
            Best := I;
         end if;
      end loop;
      for I in 0 .. N - 1 loop
         Out_V (I) := V (I, Best);
      end loop;
      return Out_V;
   end Min_Eig;

   --  ── ④ 多视图:轨迹(起点那帧里同一个像素,仪器按问的点配进了好几帧)按重投影一起解 ──
   --  每条轨迹的点在它起点那帧(Tq)的视线上:X = T_q + λ · R_q · d(d = 那个像素的视线,z = -1);λ 随模型当场解掉(每条一维、抗野点的高斯牛顿,
   --  按 log λ 解),外层只对运动学那 6N + 1 个数做 LM(数值差分)。③ 的两两对极只管配点垂直于对极线那一分量,轴离眼多远(= 每一帧平移多大)管不住;
   --  同一个点跨很多帧、远近共用,就管住了(09-27 V1B32 离线同一算法:第一只手按真值最大 0.81 → 0.11 mm,不动的眼 4.0 → 1.3 mm,第 2 只手放进世界 4.0 → 1.2 mm)
   type Nat_Ptr is access Nat_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Nat_Array, Nat_Ptr);
   type V3_Ptr is access V3_Array;
   procedure Free is new Ada.Unchecked_Deallocation (V3_Array, V3_Ptr);
   type M3_Array is array (Natural range <>) of M3;
   type Mv_Set is record
      Nt, No : Natural := 0;
      Tq : Nat_Ptr;              --  每条轨迹:起点那帧
      Du, Dv : Vec_Ptr;          --  每条轨迹:在起点那帧里的像素
      Ot, Of_Fr : Nat_Ptr;       --  每一笔:哪条轨迹、哪一帧
      Ou, Ov : Vec_Ptr;          --  每一笔:在那一帧里的像素
      Live : Nat_Ptr;            --  每条轨迹:1 = 用(远近解得出、在眼前面),0 = 不用
      Obs_In : Nat_Ptr;          --  每一笔:1 = 在门里(进解),0 = 野点(同 ③:残差 ≥ max(3 px, 3 倍中位))
      Floor : Long_Float := 0.0; --  这些配点像素坐标的数值分辨率(Px_Res):量到的噪声的下限,只防零
   end record;
   procedure Free_Mv (S : in out Mv_Set) is
   begin
      Free (S.Tq); Free (S.Du); Free (S.Dv); Free (S.Ot); Free (S.Of_Fr); Free (S.Ou); Free (S.Ov); Free (S.Live); Free (S.Obs_In);
   end Free_Mv;
   --  配点里 Pt >= 0 的按 Pt 归成轨迹(Only_I >= 0:只要起点在那一帧的)
   procedure Build_Mv (Cs : Corr_Vectors.Vector; Only_I : Integer; S : out Mv_Set) is
      package Id_Maps is new Ada.Containers.Ordered_Maps (Integer, Natural);
      Ids : Id_Maps.Map;
      Nt, No : Natural := 0;
      function Want (C : Corr) return Boolean is (C.Pt >= 0 and then C.I /= C.J and then (Only_I < 0 or else C.I = Natural (Only_I)));
   begin
      for C of Cs loop
         if Want (C) then
            if not Ids.Contains (C.Pt) then
               Ids.Insert (C.Pt, Nt);
               Nt := Nt + 1;
            end if;
            No := No + 1;
         end if;
      end loop;
      S.Nt := Nt; S.No := No; S.Floor := Px_Res (Cs);
      S.Tq := new Nat_Array (0 .. Natural'Max (1, Nt) - 1); S.Du := new Vec (0 .. Natural'Max (1, Nt) - 1); S.Dv := new Vec (0 .. Natural'Max (1, Nt) - 1);
      S.Live := new Nat_Array'(0 .. Natural'Max (1, Nt) - 1 => 1);
      S.Ot := new Nat_Array (0 .. Natural'Max (1, No) - 1); S.Of_Fr := new Nat_Array (0 .. Natural'Max (1, No) - 1);
      S.Ou := new Vec (0 .. Natural'Max (1, No) - 1); S.Ov := new Vec (0 .. Natural'Max (1, No) - 1);
      S.Obs_In := new Nat_Array'(0 .. Natural'Max (1, No) - 1 => 1);
      declare
         K : Natural := 0;
      begin
         for C of Cs loop
            if Want (C) then
               declare
                  T : constant Natural := Ids.Element (C.Pt);
               begin
                  S.Tq (T) := C.I; S.Du (T) := C.Ua; S.Dv (T) := C.Va;
                  S.Ot (K) := T; S.Of_Fr (K) := C.J; S.Ou (K) := C.Ub; S.Ov (K) := C.Vb;
                  K := K + 1;
               end;
            end if;
         end loop;
      end;
   end Build_Mv;
   Behind_Px : constant := 1.0e3;   --  点落在那一帧眼后面:按这么大的像素残差记(远大于任何真残差的哨兵,无量纲)
   --  soft-l1 的代价(抗野点;Tau = 尺度,像素)
   function Rho_Sl (E2, Tau : Long_Float) return Long_Float is (2.0 * Tau * Tau * (Sqrt (1.0 + E2 / (Tau * Tau)) - 1.0));
   --  给定各帧位姿(Pr, Pt),解每条轨迹的 log 远近(Lz,就地更新),回填每一笔的像素残差 (Ru, Rv)、残差垂直于对极线的那一分量 Rn。
   --  远近一条一条按抗野点(soft-l1,尺度 Tau)的高斯牛顿解到不再变(09-30 改:原来按调用方给的遍数,④ 试步解 4 遍、基准只解 2 遍,
   --  两边不一样,"代价降了"可能只是远近多收了一点):这一步已经挪不动什么了就停 —— 它能让这一条的代价降的那一点(G² / H)小于代价自己的
   --  数值分辨率(ε × 这一条的代价),或者 log λ 挪不到 ε(λ 自己挪不到 ε 倍),或者这一步让各笔的像素加起来挪不到坐标的数值分辨率(S.Floor)。
   --  (只看 log λ 挪不挪不行:梯度的舍入让步子在 ±2 ~ 3 个 ulp 之间来回跳,永远停不下来 —— 09-30 实测 x5 焊点里几千条这样)
   --  上限只当保险:一步至少减半才叫在收,从最大的一步(0.5)减到 ε 要 Long_Float'Machine_Mantissa 遍;做满还没停的条数 = Stuck(照实交出去)。
   --  Rn:这一笔的点只能落在那一帧的对极线上(远近只让它沿线走)⇒ 垂直于线的那一分量远近怎么解都不动,是一维的配点噪声(量噪声用它,见 Mv_Sig)。
   --  Wa / Wb:每一笔在那一帧眼系里 X = Wb + λ Wa(工作区);G / H:每条轨迹一维的梯度 / 曲率(工作区;出来时 H 是不加权的曲率)
   --  Solve = False:远近不动,只按现在的算一遍(Rn 跟远近无关,起步量噪声就这么量)。Moving 不空:出来时每条轨迹 1 = 做满还没停
   procedure Mv_Eval (S : Mv_Set; F, Cx, Cy : Long_Float; Pr : M3_Array; Pt : V3_Array; Lz : in out Vec; Tau : Long_Float;
                      Wa, Wb : V3_Ptr; Ru, Rv, Rn, G, H, Lt : Vec_Ptr; Stuck : out Natural; Solve : Boolean := True; Moving : Nat_Ptr := null) is
      --  Lt:每条轨迹这一遍的 λ(工作区,一条只算一次 exp)
      Act : Nat_Ptr := new Nat_Array'(0 .. Natural'Max (1, S.Nt) - 1 => 0);   --  1 = 这一条还在解
      Cst : Vec_Ptr := new Vec (0 .. Natural'Max (1, S.Nt) - 1);             --  这一条这一遍的代价(门里各笔的 soft-l1)
      --  一遍:还在解的那几条(Final = 最后一遍:全部都算、曲率不加权、顺带算 Rn)
      procedure Pass (Final : Boolean) is
      begin
         for T in 0 .. S.Nt - 1 loop
            if Final or else Act (T) = 1 then
               G (T) := 0.0; H (T) := 0.0; Cst (T) := 0.0;
               Lt (T) := (if S.Live (T) = 1 then Exp (Lz (Lz'First + T)) else 0.0);
            end if;
         end loop;
         for N in 0 .. S.No - 1 loop
            declare
               T : constant Natural := S.Ot (N);
            begin
               if not Final and then Act (T) = 0 then
                  null;   --  这一条已经解到不再变:残差留着上一遍的(就是它现在的)
               elsif S.Live (T) = 1 then
                  declare
                     Lam : constant Long_Float := Lt (T);
                     X : constant V3 := Add (Wb (N), Scl (Wa (N), Lam));
                     Zp : constant Long_Float := -X (2);
                  begin
                     if Zp > 1.0e-9 then   --  在那一帧眼前面(数值,无量纲)
                        Ru (N) := F * X (0) / Zp + Cx - S.Ou (N);
                        Rv (N) := -F * X (1) / Zp + Cy - S.Ov (N);
                        declare
                           Dx : constant V3 := Scl (Wa (N), Lam);   --  ∂X/∂log λ
                           Dzp : constant Long_Float := -Dx (2);
                           Du : constant Long_Float := F * (Dx (0) * Zp - X (0) * Dzp) / (Zp * Zp);
                           Dv : constant Long_Float := -F * (Dx (1) * Zp - X (1) * Dzp) / (Zp * Zp);
                           Dn : constant Long_Float := Sqrt (Du * Du + Dv * Dv);
                        begin
                           if S.Obs_In (N) = 1 then   --  门外的笔不进这一维的解(残差照算,重挑内点要用)
                              declare
                                 W : constant Long_Float := (if Final then 1.0 else 1.0 / Sqrt (1.0 + (Ru (N) ** 2 + Rv (N) ** 2) / (Tau * Tau)));
                              begin
                                 G (T) := G (T) + W * (Du * Ru (N) + Dv * Rv (N));
                                 H (T) := H (T) + W * (Du * Du + Dv * Dv);
                                 if not Final then
                                    Cst (T) := Cst (T) + Rho_Sl (Ru (N) ** 2 + Rv (N) ** 2, Tau);
                                 end if;
                              end;
                           end if;
                           if Final then
                              --  远近一点不管这一笔的像素(两帧的眼在同一处):它两个方向都是噪声,取 u 那一个
                              Rn (N) := (if Dn > 0.0 then (Rv (N) * Du - Ru (N) * Dv) / Dn else Ru (N));
                           end if;
                        end;
                     else
                        Ru (N) := Behind_Px; Rv (N) := 0.0; Rn (N) := 0.0;
                     end if;
                  end;
               else
                  Ru (N) := 0.0; Rv (N) := 0.0; Rn (N) := 0.0;
               end if;
            end;
         end loop;
      end Pass;
   begin
      for N in 0 .. S.No - 1 loop
         declare
            T : constant Natural := S.Ot (N);
            Q : constant Natural := S.Tq (T);
            K : constant Natural := S.Of_Fr (N);
            D : constant V3 := [(S.Du (T) - Cx) / F, -(S.Dv (T) - Cy) / F, -1.0];
         begin
            Wa (N) := ApT (Pr (K), Ap (Pr (Q), D));
            Wb (N) := ApT (Pr (K), Sub (Pt (Q), Pt (K)));
         end;
      end loop;
      for T in 0 .. S.Nt - 1 loop
         Act (T) := (if Solve then S.Live (T) else 0);
      end loop;
      Stuck := 0;
      for It in 1 .. Long_Float'Machine_Mantissa loop
         exit when not Solve;
         Pass (Final => False);
         Stuck := 0;
         for T in 0 .. S.Nt - 1 loop
            if Act (T) = 1 then
               if H (T) > 1.0e-18 then   --  数值保护(无量纲)
                  declare
                     --  一步最多挪 log λ ±0.5(远近一步最多差 1.65 倍;高斯牛顿不越过坑,无量纲)
                     Stp : constant Long_Float := Long_Float'Max (-0.5, Long_Float'Min (0.5, -G (T) / H (T)));
                     Gain : constant Long_Float := Stp * Stp * H (T);   --  这一步能让这一条的代价降多少(高斯牛顿估的)、各笔像素一共挪多少的平方
                  begin
                     if abs Stp > Long_Float'Model_Epsilon and then Gain > Long_Float'Model_Epsilon * Cst (T) and then Gain > S.Floor * S.Floor then
                        Lz (Lz'First + T) := Lz (Lz'First + T) + Stp;
                        Stuck := Stuck + 1;
                     else
                        Act (T) := 0;   --  这一步已经挪不动什么了
                     end if;
                  end;
               else
                  Act (T) := 0;   --  门里没有管得着远近的笔:远近不动
               end if;
            end if;
         end loop;
         exit when Stuck = 0;
      end loop;
      Pass (Final => True);
      if Moving /= null then
         for T in 0 .. S.Nt - 1 loop
            Moving (T) := Act (T);   --  停下来的都已经清成 0;还是 1 的就是做满还在挪的
         end loop;
      end if;
      Free (Act); Free (Cst);
   end Mv_Eval;
   --  起步远近:每条轨迹按"x + u z = 0、y + v z = 0"对 λ 线性最小二乘(u, v = 那一帧里像素换成视线);解出来 ≤ 0 的轨迹不用
   procedure Mv_Init (S : Mv_Set; F, Cx, Cy : Long_Float; Pr : M3_Array; Pt : V3_Array; Lz : out Vec) is
      Num : Vec (0 .. Natural'Max (1, S.Nt) - 1) := [others => 0.0];
      Den : Vec (0 .. Natural'Max (1, S.Nt) - 1) := [others => 0.0];
   begin
      for N in 0 .. S.No - 1 loop
         declare
            T : constant Natural := S.Ot (N);
            Q : constant Natural := S.Tq (T);
            K : constant Natural := S.Of_Fr (N);
            D : constant V3 := [(S.Du (T) - Cx) / F, -(S.Dv (T) - Cy) / F, -1.0];
            A : constant V3 := ApT (Pr (K), Ap (Pr (Q), D));
            B : constant V3 := ApT (Pr (K), Sub (Pt (Q), Pt (K)));
            U : constant Long_Float := (S.Ou (N) - Cx) / F;
            V : constant Long_Float := -(S.Ov (N) - Cy) / F;
            A1 : constant Long_Float := A (0) + U * A (2);
            A2 : constant Long_Float := A (1) + V * A (2);
            B1 : constant Long_Float := B (0) + U * B (2);
            B2 : constant Long_Float := B (1) + V * B (2);
         begin
            Num (T) := Num (T) - (A1 * B1 + A2 * B2);
            Den (T) := Den (T) + A1 * A1 + A2 * A2;
         end;
      end loop;
      for T in 0 .. S.Nt - 1 loop
         declare
            L : constant Long_Float := (if Den (T) > 1.0e-18 then Num (T) / Den (T) else 0.0);   --  数值保护(无量纲)
         begin
            if L > 0.0 then
               Lz (Lz'First + T) := Log (L);
            else
               Lz (Lz'First + T) := 0.0; S.Live (T) := 0;
            end if;
         end;
      end loop;
   end Mv_Init;
   --  中位数(像素残差长度,只算用着的轨迹的笔)
   function Mv_Med (S : Mv_Set; Ru, Rv : Vec_Ptr; Q : Long_Float := 0.5) return Long_Float is
      E : Vec_Ptr := new Vec (0 .. Natural'Max (1, S.No) - 1);
      N : Natural := 0;
      R : Long_Float;
   begin
      for K in 0 .. S.No - 1 loop
         if S.Live (S.Ot (K)) = 1 and then S.Obs_In (K) = 1 and then Ru (K) /= Behind_Px then
            E (N) := Sqrt (Ru (K) ** 2 + Rv (K) ** 2); N := N + 1;
         end if;
      end loop;
      R := (if N = 0 then 0.0 elsif Q = 0.5 then Median_Abs (E (0 .. N - 1)) else Quantile_Abs (E (0 .. N - 1), Q));
      Free (E);
      return R;
   end Mv_Med;
   --  量到的配点噪声 σ(像素,每个方向):门里各笔 Rn(垂直于对极线的那一分量,见 Mv_Eval;一维正态)的 Mad_Sigma × 中位。
   --  不拿二维残差长度的中位换:多视图轨迹上长度的中位 = 1.1774σ,只有两帧的轨迹沿线那一分量被远近解掉、长度只剩一维(中位 0.6745σ),
   --  一批里两样都有,长度的中位按哪一样换都不对
   function Mv_Sig (S : Mv_Set; Ru, Rn : Vec_Ptr) return Long_Float is
      E : Vec_Ptr := new Vec (0 .. Natural'Max (1, S.No) - 1);
      N : Natural := 0;
      R : Long_Float;
   begin
      for K in 0 .. S.No - 1 loop
         if S.Live (S.Ot (K)) = 1 and then S.Obs_In (K) = 1 and then Ru (K) /= Behind_Px then
            E (N) := Rn (K); N := N + 1;
         end if;
      end loop;
      R := Sigma_Of ((if N = 0 then 0.0 else Median_Abs (E (0 .. N - 1))), S.Floor);
      Free (E);
      return R;
   end Mv_Sig;

   --  挑内点(同 ③):先把每一笔都放回门里、按当前的残差重挑 —— 残差 < max(3 px, 3 倍中位)的进解(协议:配点残差按像素记);
   --  一笔都不剩的轨迹不用。返回门里几笔
   procedure Mv_Gate (S : Mv_Set; Ru, Rv : Vec_Ptr; N_In : out Natural) is
      Md : Long_Float;
      Gate : Long_Float;
      Cnt : Nat_Array (0 .. Natural'Max (1, S.Nt) - 1) := [others => 0];
   begin
      for K in 0 .. S.No - 1 loop
         S.Obs_In (K) := 1;
      end loop;
      Md := Mv_Med (S, Ru, Rv);
      Gate := Long_Float'Max (3.0, 3.0 * Md);   --  3 px / 3 倍中位(协议)
      N_In := 0;
      for K in 0 .. S.No - 1 loop
         if Ru (K) = Behind_Px or else Sqrt (Ru (K) ** 2 + Rv (K) ** 2) >= Gate then
            S.Obs_In (K) := 0;
         else
            Cnt (S.Ot (K)) := Cnt (S.Ot (K)) + 1;
            if S.Live (S.Ot (K)) = 1 then
               N_In := N_In + 1;
            end if;
         end if;
      end loop;
      for T in 0 .. S.Nt - 1 loop
         if Cnt (T) = 0 then
            S.Live (T) := 0;
         end if;
      end loop;
   end Mv_Gate;

   Mv_Iters : constant := 40;   --  ④ 外层 LM 每轮最多几次(次数;V1B32 离线 10 次后只再降 0.5%)
   Mv_Up : constant := 10.0;   --  ④ 阻尼放大倍数(次数,同 Robust_LM)
   Mv_Dn : constant := 3.0;    --  ④ 阻尼缩小倍数(次数,同 Robust_LM)
   --  "做到不再变"那几个循环的保险上限:③ ④ 重挑内点到挑出来的不再变、④ 从上一遍的结果再做到残差不再降(实测 2–4 轮到底);
   --  碰到了照实记进 Rep.Unsettled,不静悄悄交结果
   Round_Cap : constant := 8;
   procedure Note (Rep : in out Fit_Report; S : String) is
   begin
      Ada.Strings.Unbounded.Append (Rep.Unsettled, S & ";");
   end Note;

   --  ③ ④ 一起解的数(09-27 加"走"的关节):X = [每根轴的 W(3 个;走的关节 = 方向 × 每单位走多远), 转的轴各自的 P(3 个,按轴的次序), 对数焦距]。
   --  全是"转"的手:X = [W_0 … W_n−1, P_0 … P_n−1, log f],和以前一样
   function N_Turn (M : Model) return Natural is
      K : Natural := 0;
   begin
      for J in 0 .. M.N - 1 loop
         if not M.Ax (J).Slide then
            K := K + 1;
         end if;
      end loop;
      return K;
   end N_Turn;
   function N_Params (M : Model) return Natural is (3 * M.N + 3 * N_Turn (M) + 1);
   --  第 J 根(转的)轴的 P 在 X 里从第几个起:3N + 3 × 它前面转的轴有几根
   function P_Off (M : Model; J : Natural) return Natural is
      K : Natural := 3 * M.N;
   begin
      for I in 0 .. J - 1 loop
         if not M.Ax (I).Slide then
            K := K + 3;
         end if;
      end loop;
      return K;
   end P_Off;
   procedure To_X (M : Model; X : out Vec) is
   begin
      for J in 0 .. M.N - 1 loop
         for K in 0 .. 2 loop
            X (X'First + 3 * J + K) := M.Ax (J).W (K);
            if not M.Ax (J).Slide then
               X (X'First + P_Off (M, J) + K) := M.Ax (J).P (K);
            end if;
         end loop;
      end loop;
      X (X'Last) := Log (M.F);
   end To_X;
   function From_X (M : Model; X : Vec) return Model is
      Mm : Model := M;
   begin
      for J in 0 .. M.N - 1 loop
         Mm.Ax (J).W := [X (X'First + 3 * J), X (X'First + 3 * J + 1), X (X'First + 3 * J + 2)];
         if not M.Ax (J).Slide then
            declare
               O : constant Natural := X'First + P_Off (M, J);
            begin
               Mm.Ax (J).P := [X (O), X (O + 1), X (O + 2)];
            end;
         end if;
      end loop;
      Mm.F := Exp (X (X'Last));
      return Mm;
   end From_X;
   --  约束行(不加权):每根转的轴两行 —— 方向是单位向量、P 取轴上离参照眼最近那点;走的关节没有(方向的长短就是每单位走多远);
   --  最后一行 = 尺度(调用的地方填)。1e3 = 这几行比像素残差重得多(比例)
   procedure Axis_Rows (M : Model; R : in out Vec) is
      K : Natural := R'First;
   begin
      for J in 0 .. M.N - 1 loop
         if not M.Ax (J).Slide then
            R (K) := 1.0e3 * (Norm (M.Ax (J).W) - 1.0);
            R (K + 1) := 1.0e3 * Dot (M.Ax (J).W, M.Ax (J).P);
            K := K + 2;
         end if;
      end loop;
   end Axis_Rows;
   --  整只手的长度一起乘一个数(模型的预测不变,只换单位):转的轴乘 P,走的关节乘 W
   procedure Scale_Model (M : in out Model; S : Long_Float) is
   begin
      for J in 0 .. M.N - 1 loop
         if M.Ax (J).Slide then
            M.Ax (J).W := Scl (M.Ax (J).W, S);
         else
            M.Ax (J).P := Scl (M.Ax (J).P, S);
         end if;
      end loop;
   end Scale_Model;

   procedure Refine_Mv (Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; M : in out Model; Rep : in out Fit_Report) is
      Nf : constant Natural := Natural (Frames.Length);
      N : constant Natural := M.N;
      --  ④ 解的数 = 运动学那一份(轴 + 对数焦距,同 ③)再加镜头中心(主点 Cx、Cy):扫描时几个关节绕不同的轴转、同一片静止的场景进很多格,
      --  主点看得出来(画幅中心只当起步;10-01 起不再钉死在正中 —— 仿真的主点碰巧就在正中,钉死了验收查不出它)
      Nb : constant Natural := N_Params (M);
      Np : constant Natural := Nb + 2;
      Nt_Ax : constant Natural := N_Turn (M);
      N_Reg : constant Natural := 2 * Nt_Ax + 1;   --  约束行:每根转的轴两行 + 尺度一行
      S : Mv_Set;
      Used : array (0 .. Natural'Max (1, Nf) - 1) of Boolean := [others => False];
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      Build_Mv (Cs, -1, S);
      --  够不够解:行数(每一笔两行 + 约束行)要多于待解的数(运动学的 Np 个 + 每条轨迹一个远近);
      --  09-30 以前只比 2 × 笔数 > Np,漏数了每条轨迹的远近(约束行也是方程:它们定住的正是 Np 里多出来的那几个)
      if S.Nt = 0 or else 2 * S.No + N_Reg <= Np + S.Nt then
         Free_Mv (S);
         return;
      end if;
      for T in 0 .. S.Nt - 1 loop
         Used (S.Tq (T)) := True;
      end loop;
      for K in 0 .. S.No - 1 loop
         Used (S.Of_Fr (K)) := True;
      end loop;
      declare
         Nr : constant Natural := 2 * S.No + N_Reg;
         Wa, Wb : V3_Ptr := new V3_Array (0 .. S.No - 1);
         Ru, Rv, Rn, Ru2, Rv2, Rn2 : Vec_Ptr := new Vec (0 .. S.No - 1);
         G, H, Lt : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Sw : Vec_Ptr := new Vec (0 .. S.No - 1);   --  外层 IRLS 的 √权(按当前残差定,求导时不动)
         Lz, Lz2 : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Jc : Mat_Ptr := new Mat (0 .. Nr - 1, 0 .. Np - 1);
         R0 : Vec_Ptr := new Vec (0 .. Nr - 1);
         X : Vec (0 .. Np - 1);
         Tau : Long_Float;   --  抗野点(soft-l1)的尺度 = 量到的配点噪声(Mv_Sig):起步按起步的远近就量,每轮挑完内点重量
         Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
         C0 : Long_Float;
         Stuck, Stuck2, Stuck_X : Natural := 0;   --  远近做满保险上限还没解到不再变的条数(Stuck_X = 现在交出去的这一份的)
         Pr : M3_Array (0 .. Nf - 1);
         Pt : V3_Array (0 .. Nf - 1);
         --  上一轮挑出来的:每一笔在不在门里、每条轨迹用不用(做到这一轮挑出来的和它一样为止)
         Prev_In : Nat_Array (0 .. Natural'Max (1, S.No) - 1) := [others => 0];
         Prev_Live : Nat_Array (0 .. Natural'Max (1, S.Nt) - 1) := [others => 0];
         function Same_Set return Boolean is
         begin
            for K in 0 .. S.No - 1 loop
               if S.Obs_In (K) /= Prev_In (K) then
                  return False;
               end if;
            end loop;
            for T in 0 .. S.Nt - 1 loop
               if S.Live (T) /= Prev_Live (T) then
                  return False;
               end if;
            end loop;
            return True;
         end Same_Set;
         function Hstep (V : Long_Float) return Long_Float is (1.0e-6 * Long_Float'Max (1.0, abs V));   --  差分步(相对 1e-6,无量纲)
         function To_Model (Xx : Vec) return Model is
            Mm : Model := From_X (M, Xx (Xx'First .. Xx'First + Nb - 1));
         begin
            Mm.Cx := Xx (Xx'First + Nb); Mm.Cy := Xx (Xx'First + Nb + 1);
            return Mm;
         end To_Model;
         procedure Poses (Mm : Model) is
         begin
            for Fr in 0 .. Nf - 1 loop
               FK (Mm, Frames (Fr).Q, Pr (Fr), Pt (Fr));
            end loop;
         end Poses;
         --  约束行(不加权,同 ③):轴是单位向量、P 取轴上离参照眼最近那点、尺度钉住"参与的各帧眼的位置均方根 = 1"(1e3 = 比像素残差重得多,比例)
         --  (Poses 之后调:尺度那一行用 Pt)
         procedure Reg (Mm : Model; R : out Vec) is
            S2 : Long_Float := 0.0;
            Cnt : Natural := 0;
         begin
            Axis_Rows (Mm, R);
            for Fr in 0 .. Nf - 1 loop
               if Used (Fr) then
                  S2 := S2 + Dot (Pt (Fr), Pt (Fr)); Cnt := Cnt + 1;
               end if;
            end loop;
            R (R'First + 2 * Nt_Ax) := 1.0e3 * ((if Cnt > 0 then Sqrt (S2 / Long_Float (Cnt)) else 1.0) - 1.0);   --  同上(比例)
         end Reg;
         function Cost (Rru, Rrv : Vec_Ptr; Rr : Vec) return Long_Float is
            C : Long_Float := 0.0;
         begin
            for K in 0 .. S.No - 1 loop
               if S.Live (S.Ot (K)) = 1 and then S.Obs_In (K) = 1 then
                  C := C + Rho_Sl (Rru (K) ** 2 + Rrv (K) ** 2, Tau);
               end if;
            end loop;
            for I in Rr'Range loop
               C := C + Rr (I) ** 2;
            end loop;
            return C;
         end Cost;
         Rg0 : Vec (0 .. 2 * Nt_Ax);   --  当前 X 下的约束行
         --  当前 X、Lz 下:残差、√权、加权残差向量
         procedure Base is
         begin
            for K in 0 .. S.No - 1 loop
               --  在那一帧眼后面的笔不进外层(它的残差是哨兵,不是像素)
               Sw (K) := (if S.Live (S.Ot (K)) = 1 and then S.Obs_In (K) = 1 and then Ru (K) /= Behind_Px
                          then Sqrt (1.0 / Sqrt (1.0 + (Ru (K) ** 2 + Rv (K) ** 2) / (Tau * Tau))) else 0.0);
               R0 (K) := Sw (K) * Ru (K); R0 (S.No + K) := Sw (K) * Rv (K);
            end loop;
            for I in Rg0'Range loop
               R0 (2 * S.No + I) := Rg0 (I);
            end loop;
         end Base;
      begin
         --  先把每根轴规整成约束行要的样子(方向归一、轴上那一点取离参照眼最近的垂足 —— 同一根轴,模型不变):
         --  起步不合约定时约束行一上来就是千倍的违反,LM 为了压它会把模型带歪(焊点:真模型起步走开 24 mm)
         for J in 0 .. N - 1 loop
            if not M.Ax (J).Slide then   --  走的关节:方向的长短就是每单位走多远,不归一
               M.Ax (J).W := Unit (M.Ax (J).W);
               M.Ax (J).P := Sub (M.Ax (J).P, Scl (M.Ax (J).W, Dot (M.Ax (J).W, M.Ax (J).P)));
            end if;
         end loop;
         --  尺度也按这一步用的帧钉成"眼的位置均方根 = 1"(整只手一起乘一个数,模型的预测不变;③ 钉尺度用的是它的内点那批帧,不一定是这一批)
         declare
            S2 : Long_Float := 0.0;
            Cnt : Natural := 0;
         begin
            Poses (M);
            for Fr in 0 .. Nf - 1 loop
               if Used (Fr) then
                  S2 := S2 + Dot (Pt (Fr), Pt (Fr)); Cnt := Cnt + 1;
               end if;
            end loop;
            if Cnt > 0 and then S2 > 0.0 then
               Scale_Model (M, 1.0 / Sqrt (S2 / Long_Float (Cnt)));
            end if;
         end;
         To_X (M, X (0 .. Nb - 1));
         X (Nb) := M.Cx; X (Nb + 1) := M.Cy;
         Poses (M);
         Mv_Init (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all);
         --  起步的抗野点尺度按起步的远近就量(Rn 跟远近无关;不解远近时用不着尺度,给什么都一样)—— 不拍一个像素数起步(09-30 以前 3 px)
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, S.Floor, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck, Solve => False);
         Tau := Mv_Sig (S, Ru, Rn);
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, Tau, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck);   --  起步远近解到不再变
         --  在哪一帧眼后面的笔多于一半的轨迹不用(起步模型下这条轨迹的远近解不出来)
         declare
            Bad : array (0 .. S.Nt - 1) of Natural := [others => 0];
            All_N : array (0 .. S.Nt - 1) of Natural := [others => 0];
         begin
            for K in 0 .. S.No - 1 loop
               All_N (S.Ot (K)) := All_N (S.Ot (K)) + 1;
               if Ru (K) = Behind_Px then
                  Bad (S.Ot (K)) := Bad (S.Ot (K)) + 1;
               end if;
            end loop;
            for T in 0 .. S.Nt - 1 loop
               if 2 * Bad (T) > All_N (T) then
                  S.Live (T) := 0;
               end if;
            end loop;
         end;
         --  重挑内点、解,做到挑出来的不再变(同 ③;09-30 以前固定两轮):每轮先按当前的残差重挑(门外的笔不进解)、重量抗野点尺度,再解
         --  (合成焊点:5% 乱配时 soft-l1 对大残差还有恒定的拉力,几千条乱配一起把模型拉歪 24 mm —— 要先挑掉)
         Rep.Mv_Iters := 0; Rep.Mv_Rounds := 0; Stuck_X := Stuck;
         for Round in 1 .. Round_Cap + 1 loop
            declare
               N_In, Live_N : Natural := 0;
               Settled : Boolean := False;
            begin
               Poses (To_Model (X));
               Mv_Gate (S, Ru, Rv, N_In);
               exit when Round > 1 and then Same_Set;   --  挑出来的和上一轮一样:上一轮解出的就是
               if Round > Round_Cap then
                  Note (Rep, "④ 重挑内点" & Natural'Image (Round_Cap) & " 轮还在变");
                  exit;
               end if;
               Rep.Mv_Rounds := Round;
               Prev_In (0 .. S.No - 1) := S.Obs_In (0 .. S.No - 1);
               Prev_Live (0 .. S.Nt - 1) := S.Live (0 .. S.Nt - 1);
               Mv_Eval (S, Exp (X (Nb - 1)), X (Nb), X (Nb + 1), Pr, Pt, Lz.all, Tau, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck);   --  门里的重解远近
               if Round = 1 then
                  Rep.Mv_Start_Px := Mv_Med (S, Ru, Rv);
               end if;
               Tau := Mv_Sig (S, Ru, Rn);   --  门里重量噪声
               Mv_Eval (S, Exp (X (Nb - 1)), X (Nb), X (Nb + 1), Pr, Pt, Lz.all, Tau, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck);
               Stuck_X := Stuck;
               for T in 0 .. S.Nt - 1 loop
                  Live_N := Live_N + S.Live (T);
               end loop;
               Rep.Mv_Obs := N_In;
               Rep.Mv_Tracks := Live_N;
               --  挑完还够不够解(同上的数法:门里每一笔两行 + 约束行,要多于 Np + 用着的轨迹数)
               if 2 * N_In + N_Reg <= Np + Live_N then
                  Note (Rep, "④ 挑完内点剩的笔不够解");
                  exit;
               end if;
               Reg (To_Model (X), Rg0);
               Base;
               C0 := Cost (Ru, Rv, Rg0);
            for It in 1 .. Mv_Iters loop
               Rep.Mv_Iters := Rep.Mv_Iters + 1;
               --  数值雅可比:每个数挪一点,远近从当前解(已解到不再变)起再解到不再变,外层权不动;基准、试步也一样解到不再变 ——
               --  三处都是"这个模型下远近的最优",差分里、比代价时都没有"远近还在收"的那一点(09-27 龙门架:基准和挪一点的那份远近解的遍数
               --  不一样,远近自己还在收的那一点被除以 1e-6 当成导数,多视图 800 轮才从焦距 419 爬到 410;09-30 以前试步解 4 遍、基准 2 遍,
               --  同样的毛病落在比代价上:试步多收的那一点也算成"降了")
               Poses (To_Model (X));
               Mv_Eval (S, Exp (X (Nb - 1)), X (Nb), X (Nb + 1), Pr, Pt, Lz.all, Tau, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck);
               Stuck_X := Stuck;
               Base;
               C0 := Cost (Ru, Rv, Rg0);
               for Jp in 0 .. Np - 1 loop
                  declare
                     Xp : Vec := X;
                     Hh : constant Long_Float := Hstep (X (Jp));
                     Mm : Model;
                  begin
                     Xp (Jp) := Xp (Jp) + Hh;
                     Mm := To_Model (Xp);
                     Poses (Mm);
                     Lz2.all := Lz.all;
                     Mv_Eval (S, Mm.F, Mm.Cx, Mm.Cy, Pr, Pt, Lz2.all, Tau, Wa, Wb, Ru2, Rv2, Rn2, G, H, Lt, Stuck2);
                     for K in 0 .. S.No - 1 loop
                        Jc (K, Jp) := (if Sw (K) > 0.0 and then Ru2 (K) /= Behind_Px then Sw (K) * (Ru2 (K) - Ru (K)) / Hh else 0.0);
                        Jc (S.No + K, Jp) := (if Sw (K) > 0.0 and then Ru2 (K) /= Behind_Px then Sw (K) * (Rv2 (K) - Rv (K)) / Hh else 0.0);
                     end loop;
                     declare
                        Rg : Vec (0 .. 2 * Nt_Ax);
                     begin
                        Reg (Mm, Rg);
                        for I in Rg'Range loop
                           Jc (2 * S.No + I, Jp) := (Rg (I) - Rg0 (I)) / Hh;
                        end loop;
                     end;
                  end;
               end loop;
               declare
                  A : Mat (0 .. Np - 1, 0 .. Np - 1) := [others => [others => 0.0]];
                  B : Vec (0 .. Np - 1) := [others => 0.0];
                  Improved : Boolean := False;
               begin
                  for I in 0 .. Nr - 1 loop
                     for K in 0 .. Np - 1 loop
                        declare
                           Jk : constant Long_Float := Jc (I, K);
                        begin
                           if Jk /= 0.0 then
                              for L in K .. Np - 1 loop
                                 A (K, L) := A (K, L) + Jk * Jc (I, L);
                              end loop;
                              B (K) := B (K) - Jk * R0 (I);
                           end if;
                        end;
                     end loop;
                  end loop;
                  for K in 0 .. Np - 1 loop
                     for L in 0 .. K - 1 loop
                        A (K, L) := A (L, K);
                     end loop;
                  end loop;
                  --  阻尼一直往上调,直到代价降了、或者步子小到 X 的数值分辨率以下(同 Robust_LM;09-30 以前最多 8 次就判到底)
                  loop
                     declare
                        Aa : Mat := A;
                        Bb : Vec := B;
                        D : Vec (0 .. Np - 1) := [others => 0.0];
                        Xn : Vec := X;
                        Cn : Long_Float;
                        Mm : Model;
                        Rg : Vec (0 .. 2 * Nt_Ax);
                        Tiny : Boolean := True;
                     begin
                        for K in 0 .. Np - 1 loop
                           Aa (K, K) := Aa (K, K) * (1.0 + Lam) + 1.0e-12;
                        end loop;
                        for Col in 0 .. Np - 1 loop
                           declare
                              Pv : Natural := Col;
                           begin
                              for Rw in Col + 1 .. Np - 1 loop
                                 if abs Aa (Rw, Col) > abs Aa (Pv, Col) then
                                    Pv := Rw;
                                 end if;
                              end loop;
                              if Pv /= Col then
                                 for Cc in 0 .. Np - 1 loop
                                    declare
                                       Tmp : constant Long_Float := Aa (Col, Cc);
                                    begin
                                       Aa (Col, Cc) := Aa (Pv, Cc); Aa (Pv, Cc) := Tmp;
                                    end;
                                 end loop;
                                 declare
                                    Tmp : constant Long_Float := Bb (Col);
                                 begin
                                    Bb (Col) := Bb (Pv); Bb (Pv) := Tmp;
                                 end;
                              end if;
                              if abs Aa (Col, Col) > 1.0e-300 then   --  主元为零保护(数值,无量纲)
                                 for Rw in Col + 1 .. Np - 1 loop
                                    declare
                                       Fct : constant Long_Float := Aa (Rw, Col) / Aa (Col, Col);
                                    begin
                                       if Fct /= 0.0 then
                                          for Cc in Col .. Np - 1 loop
                                             Aa (Rw, Cc) := Aa (Rw, Cc) - Fct * Aa (Col, Cc);
                                          end loop;
                                          Bb (Rw) := Bb (Rw) - Fct * Bb (Col);
                                       end if;
                                    end;
                                 end loop;
                              end if;
                           end;
                        end loop;
                        for K in reverse 0 .. Np - 1 loop
                           declare
                              Sm : Long_Float := Bb (K);
                           begin
                              for Cc in K + 1 .. Np - 1 loop
                                 Sm := Sm - Aa (K, Cc) * D (Cc);
                              end loop;
                              D (K) := (if abs Aa (K, K) > 1.0e-300 then Sm / Aa (K, K) else 0.0);   --  同上(数值,无量纲)
                           end;
                        end loop;
                        for K in 0 .. Np - 1 loop
                           Xn (K) := X (K) + D (K);
                           --  "不大于"的反面:算坏了的步子(NaN)也算挪不动(X 的每个数的分辨率 = ε × 它自己 / 它的差分步,同 Robust_LM)
                           if abs D (K) > Long_Float'Model_Epsilon * Long_Float'Max (abs X (K), Hstep (X (K))) then
                              Tiny := False;
                           end if;
                        end loop;
                        exit when Tiny;   --  到底了:再小的步子已经挪不动 X
                        Mm := To_Model (Xn);
                        Poses (Mm);
                        Lz2.all := Lz.all;
                        Mv_Eval (S, Mm.F, Mm.Cx, Mm.Cy, Pr, Pt, Lz2.all, Tau, Wa, Wb, Ru2, Rv2, Rn2, G, H, Lt, Stuck2);   --  试的这一步远近也解到不再变
                        Reg (Mm, Rg);
                        Cn := Cost (Ru2, Rv2, Rg);
                        if Cn < C0 then
                           Improved := (C0 - Cn) > 1.0e-7 * C0;   --  这一轮代价降得不到千万分之一就算到底了(比例)
                           X := Xn; Lz.all := Lz2.all; Ru.all := Ru2.all; Rv.all := Rv2.all; Rn.all := Rn2.all; Rg0 := Rg; Stuck_X := Stuck2;
                           Base;
                           C0 := Cn;
                           Lam := Long_Float'Max (1.0e-9, Lam / Mv_Dn);
                           exit;
                        else
                           Lam := Lam * Mv_Up;
                        end if;
                     end;
                  end loop;
                  if not Improved then
                     Settled := True;
                     exit;
                  end if;
               end;
            end loop;
               if not Settled then
                  Note (Rep, "④ LM" & Natural'Image (Mv_Iters) & " 次还在降");
               end if;
            end;
         end loop;
         M := To_Model (X);
         --  主点的不确定度(像素):最后一轮的雅可比(加权)⇒ (JᵀJ)⁻¹ × 每行残差的方差(残差平方和 ÷ (行数 − 待解的数));
         --  远近已经按模型解掉(只剩模型这一份的协方差)
         declare
            A : Mat (0 .. Np - 1, 0 .. Np - 1) := [others => [others => 0.0]];
            Inv : Mat (0 .. Np - 1, 0 .. Np - 1) := [others => [others => 0.0]];
            Ss : Long_Float := 0.0;
            Ok_Inv : Boolean := True;
         begin
            for I in 0 .. Nr - 1 loop
               Ss := Ss + R0 (I) ** 2;
               for K in 0 .. Np - 1 loop
                  if Jc (I, K) /= 0.0 then
                     for L2 in 0 .. Np - 1 loop
                        A (K, L2) := A (K, L2) + Jc (I, K) * Jc (I, L2);
                     end loop;
                  end if;
               end loop;
            end loop;
            for K in 0 .. Np - 1 loop
               Inv (K, K) := 1.0;
            end loop;
            --  高斯–约当(列主元)
            for Col in 0 .. Np - 1 loop
               declare
                  Pv : Natural := Col;
               begin
                  for Rw in Col + 1 .. Np - 1 loop
                     if abs A (Rw, Col) > abs A (Pv, Col) then
                        Pv := Rw;
                     end if;
                  end loop;
                  if abs A (Pv, Col) <= 1.0e-300 then   --  主元为零保护(数值,无量纲)
                     Ok_Inv := False;
                     exit;
                  end if;
                  if Pv /= Col then
                     for Cc in 0 .. Np - 1 loop
                        declare
                           T1 : constant Long_Float := A (Col, Cc);
                           T2 : constant Long_Float := Inv (Col, Cc);
                        begin
                           A (Col, Cc) := A (Pv, Cc); A (Pv, Cc) := T1;
                           Inv (Col, Cc) := Inv (Pv, Cc); Inv (Pv, Cc) := T2;
                        end;
                     end loop;
                  end if;
                  declare
                     Dg : constant Long_Float := A (Col, Col);
                  begin
                     for Cc in 0 .. Np - 1 loop
                        A (Col, Cc) := A (Col, Cc) / Dg; Inv (Col, Cc) := Inv (Col, Cc) / Dg;
                     end loop;
                  end;
                  for Rw in 0 .. Np - 1 loop
                     if Rw /= Col and then A (Rw, Col) /= 0.0 then
                        declare
                           Fct : constant Long_Float := A (Rw, Col);
                        begin
                           for Cc in 0 .. Np - 1 loop
                              A (Rw, Cc) := A (Rw, Cc) - Fct * A (Col, Cc); Inv (Rw, Cc) := Inv (Rw, Cc) - Fct * Inv (Col, Cc);
                           end loop;
                        end;
                     end if;
                  end loop;
               end;
            end loop;
            if Ok_Inv and then Nr > Np then
               declare
                  S2 : constant Long_Float := Ss / Long_Float (Nr - Np);
               begin
                  Rep.Cx_Sd := Sqrt (Long_Float'Max (0.0, S2 * Inv (Nb, Nb)));
                  Rep.Cy_Sd := Sqrt (Long_Float'Max (0.0, S2 * Inv (Nb + 1, Nb + 1)));
               end;
            end if;
         end;
         for J in 0 .. N - 1 loop
            if not M.Ax (J).Slide then
               M.Ax (J).W := Unit (M.Ax (J).W);
            end if;
         end loop;
         if Stuck_X > 0 then
            Note (Rep, "④" & Natural'Image (Stuck_X) & " 条轨迹的远近" & Natural'Image (Long_Float'Machine_Mantissa) & " 遍还没解到不再变");
         end if;
         Rep.Mv_Sig_Px := Tau;
         Rep.Mv_Px := Mv_Med (S, Ru, Rv);
         Rep.Mv_P90_Px := Mv_Med (S, Ru, Rv, 0.9);   --  九成分位(比例,只报数)
         Rep.F := M.F;
         Free (Wa); Free (Wb); Free (Ru); Free (Rv); Free (Rn); Free (Ru2); Free (Rv2); Free (Rn2); Free (G); Free (H); Free (Lt); Free (Sw); Free (Lz); Free (Lz2);
         Free (Jc); Free (R0);
      end;
      Free_Mv (S);
      Rep.Secs.Append (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)));
   end Refine_Mv;

   --  ④ 做到不再变好(09-28 龙门架焊点):一遍 Refine_Mv 在平谷里会停早 —— 约束行是软的(轴单位长、轴上点取垂足、尺度钉 1,千倍权),
   --  平谷里模型一边往下走一边偏离约束,约束把步子顶回来,阻尼越调越大就判"降不动"停了(同一份数据多去掉一个点,焦距就停在 415.5 或走到 400;
   --  停在 415.5 那份接着再做一遍 409.9、再一遍 400.1,残差 0.365 → 0.343 → 0.287 px ⇒ 不是另一个坑,是没走完)。
   --  再做一遍 = 把轴和尺度重新规整到约束上、远近从头三角、重挑内点、阻尼归位 ⇒ 从它自己的结果再做,直到残差中位不再降,留最好的那遍。
   --  最多 Round_Cap 遍(保险;龙门架 3 遍到底,x5 / 人形实测 2 遍 —— 第二遍只是确认不再降);做满还在降就照实记进 Rep.Unsettled
   procedure Refine_Until_Done (Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; M : in out Model; Rep : in out Fit_Report) is
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      Secs0 : constant Floats := Rep.Secs;   --  前几步的秒数;④ 记一个数 = 几遍加起来
      Best_M : Model := M;
      Best_R : Fit_Report := Rep;
      Passes, Iters : Natural := 0;
      Settled : Boolean := False;
   begin
      for Pass in 1 .. Round_Cap loop
         declare
            Mm : Model := Best_M;
            Rr : Fit_Report := Best_R;
         begin
            Rr.Mv_Tracks := 0;
            Rr.Unsettled := Rep.Unsettled;   --  每一遍从进 ④ 时的记账起记,只留下留下的那一遍的
            Refine_Mv (Frames, Cs, Mm, Rr);
            if Rr.Mv_Tracks = 0 then   --  没有轨迹:这一步做不了
               Settled := True;
               exit;
            end if;
            Passes := Passes + 1;
            Iters := Iters + Rr.Mv_Iters;
            if Pass = 1 then
               Best_M := Mm; Best_R := Rr;
            elsif Rr.Mv_Px < Best_R.Mv_Px then
               Rr.Mv_Start_Px := Best_R.Mv_Start_Px;   --  起步中位报第一遍的
               Best_M := Mm; Best_R := Rr;
            else
               Settled := True;
               exit;
            end if;
         end;
      end loop;
      if Passes > 0 then
         if not Settled then
            Note (Best_R, "④ 从上一遍的结果再做" & Natural'Image (Round_Cap) & " 遍还在降");
         end if;
         M := Best_M;
         Rep := Best_R;
         Rep.Mv_Iters := Iters;
         Rep.Mv_Passes := Passes;
         Rep.Secs := Secs0;
         Rep.Secs.Append (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)));
      end if;
   end Refine_Until_Done;

   procedure Refine_Tracks (Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; M : in out Model; Rep : in out Fit_Report) is
   begin
      Refine_Until_Done (Frames, Off_Eye (M.Eye, Cs), M, Rep);
   end Refine_Tracks;

   procedure Track_Points (M : Model; Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; Only_I : Integer; Min_Views : Natural;
                           Tracks : out Track_Pt_Vectors.Vector; Sig_Px : out Long_Float) is
      Nf : constant Natural := Natural (Frames.Length);
      S : Mv_Set;
   begin
      Tracks.Clear; Sig_Px := 0.0;
      Build_Mv (Off_Eye (M.Eye, Cs), Only_I, S);
      if S.Nt = 0 then
         Free_Mv (S);
         return;
      end if;
      declare
         Wa, Wb : V3_Ptr := new V3_Array (0 .. S.No - 1);
         Ru, Rv, Rn : Vec_Ptr := new Vec (0 .. S.No - 1);
         G, H, Lt : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Lz : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Pr : M3_Array (0 .. Nf - 1);
         Pt : V3_Array (0 .. Nf - 1);
         Sig : Long_Float;
         Stuck : Natural;
         Moving : Nat_Ptr := new Nat_Array (0 .. Natural'Max (1, S.Nt) - 1);   --  做满保险上限远近还在挪的轨迹:点不交出去
      begin
         for Fr in 0 .. Nf - 1 loop
            FK (M, Frames (Fr).Q, Pr (Fr), Pt (Fr));
         end loop;
         Mv_Init (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all);
         --  配点噪声按起步的远近就量得出来(Rn 跟远近无关;不解远近时用不着尺度,给什么都一样),拿它当抗野点的尺度把远近解到不再变
         --  (09-30 以前先按 3 px 解 10 遍、量一次、再解 5 遍、再量)
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, S.Floor, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck, Solve => False);
         Sig := Mv_Sig (S, Ru, Rn);
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, Sig, Wa, Wb, Ru, Rv, Rn, G, H, Lt, Stuck, Moving => Moving);
         Sig := Mv_Sig (S, Ru, Rn);   --  远近解完那些"眼后面"的笔可能少了几笔,按解完的再量一次
         Sig_Px := Sig;
         --  每条轨迹:看见的帧数、残差中位、离起点那帧的眼最远的一帧
         declare
            type Tr is record
               N : Natural := 0;
               Far : Natural := 0;
               Far_D : Long_Float := -1.0;
               Behind : Boolean := False;
            end record;
            Tt : array (0 .. S.Nt - 1) of Tr;
            Es : array (0 .. S.Nt - 1) of Floats;
         begin
            for K in 0 .. S.No - 1 loop
               declare
                  T : constant Natural := S.Ot (K);
                  Fk : constant Natural := S.Of_Fr (K);
                  Dd : constant Long_Float := Norm (Sub (Pt (Fk), Pt (S.Tq (T))));
               begin
                  if S.Live (T) = 1 then
                     if Ru (K) = Behind_Px then
                        Tt (T).Behind := True;
                     else
                        Tt (T).N := Tt (T).N + 1;
                        Es (T).Append (Sqrt (Ru (K) ** 2 + Rv (K) ** 2));
                        if Dd > Tt (T).Far_D then
                           Tt (T).Far_D := Dd; Tt (T).Far := Fk;
                        end if;
                     end if;
                  end if;
               end;
            end loop;
            for T in 0 .. S.Nt - 1 loop
               if S.Live (T) = 1 and then Moving (T) = 0 and then not Tt (T).Behind and then Tt (T).N + 1 >= Min_Views and then H (T) > 1.0e-18 then   --  数值保护(无量纲)
                  declare
                     Q : constant Natural := S.Tq (T);
                     D : constant V3 := [(S.Du (T) - M.Cx) / M.F, -(S.Dv (T) - M.Cy) / M.F, -1.0];
                     Lam : constant Long_Float := Exp (Lz (T));
                     Dw : constant V3 := Ap (Pr (Q), D);
                     E : Vec (0 .. Natural (Es (T).Length) - 1);
                  begin
                     for I in E'Range loop
                        E (I) := Es (T) (I);
                     end loop;
                     --  沿视线:log λ 的方差 = σ² / 曲率 ⇒ λ 的方差 × |d|²(d 的 z = -1,沿视线的长度 = λ |d|)
                     Tracks.Append (Track_Pt'(X => Add (Pt (Q), Scl (Dw, Lam)), Var_Along => Dot (D, D) * Lam * Lam * Sig * Sig / H (T),
                                           I => Q, U => S.Du (T), V => S.Dv (T), Far => Tt (T).Far, Views => Tt (T).N + 1, Med_Px => Median_Abs (E)));
                  end;
               end if;
            end loop;
         end;
         Free (Wa); Free (Wb); Free (Ru); Free (Rv); Free (Rn); Free (G); Free (H); Free (Lt); Free (Lz); Free (Moving);
      end;
      Free_Mv (S);
   end Track_Points;

   --  最长那档焦距 = 半幅宽 ÷ tan(15°)(焦距网格的上头,见 Fit;视场 30° 是协议:针孔相机的常见范围)
   function Clean_Tol (Width : Long_Float) return Long_Float is
     (Tan (0.261799387799490) / (0.5 * Width));

   function Single_Joint (Frames : Frame_Vectors.Vector; Ref, Fr : Natural; Dmax : Long_Float) return Integer is
      Q0 : constant Floats := Frames (Ref).Q;
      N : constant Natural := Natural (Q0.Length);   --  这组读数有几个就查几个(09-30 以前最多查 12 个,多出来的关节偏了也不知道)
      J : constant Integer := Frames (Fr).Joint;
   begin
      if Fr = Ref or else J < 0 or else J >= Integer (N) then
         return -1;
      end if;
      for K in 0 .. N - 1 loop
         if K /= Natural (J) and then abs (Frames (Fr).Q (K) - Q0 (K)) >= Dmax then
            return -1;
         end if;
      end loop;
      return J;
   end Single_Joint;

   function "<" (A, B : Px) return Boolean is (A.U < B.U or else (A.U = B.U and then A.V < B.V));
   package Px_Sets is new Ada.Containers.Ordered_Sets (Px);

   function Eye_Pixels (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Width : Long_Float) return Px_Vectors.Vector is
      Nf : constant Natural := Natural (Frames.Length);
      N : constant Natural := Natural (Frames (Ref).Q.Length);
      Sj : array (0 .. Nf - 1) of Integer := [others => -1];                  --  只动了一个关节的帧 ⇒ 那个关节
      Still_N, Moved_N : array (0 .. Nf - 1) of Natural := [others => 0];     --  参照帧 ↔ 这一帧那一对里没挪 / 挪了的配点(笔数)
      function Moved (C : Corr) return Boolean is (Norm ([C.Ub - C.Ua, C.Vb - C.Va, 0.0]) >= Trip_Px);
      --  这个像素在第 J 个关节单独转的格子里有一格没挪(几个关节就几格)
      type Px_Seen is array (0 .. N - 1) of Boolean;
      package Seen_Maps is new Ada.Containers.Ordered_Maps (Px, Px_Seen);
      Ev : Seen_Maps.Map;
      R : Px_Vectors.Vector;
   begin
      for Fr in 0 .. Nf - 1 loop
         Sj (Fr) := Single_Joint (Frames, Ref, Fr, Clean_Tol (Width));
      end loop;
      for C of Cs loop
         if C.I = Ref and then C.J < Nf and then Sj (C.J) >= 0 then
            if Moved (C) then
               Moved_N (C.J) := Moved_N (C.J) + 1;
            else
               Still_N (C.J) := Still_N (C.J) + 1;
            end if;
         end if;
      end loop;
      for C of Cs loop
         if C.I = Ref and then C.J < Nf and then Sj (C.J) >= 0 and then Moved_N (C.J) > Still_N (C.J) then
            declare
               P : constant Px := (C.Ua, C.Va);
               Cur : constant Seen_Maps.Cursor := Ev.Find (P);
               E : Px_Seen := (if Seen_Maps.Has_Element (Cur) then Seen_Maps.Element (Cur) else [others => False]);
               J : constant Natural := Natural (Sj (C.J));
            begin
               E (J) := E (J) or else not Moved (C);
               Ev.Include (P, E);
            end;
         end if;
      end loop;
      for Cur in Ev.Iterate loop
         declare
            E : constant Px_Seen := Seen_Maps.Element (Cur);
            Stay : Natural := 0;   --  有一格没挪的关节几个
         begin
            for J in 0 .. N - 1 loop
               if E (J) then
                  Stay := Stay + 1;
               end if;
            end loop;
            if Stay >= 2 then
               R.Append (Seen_Maps.Key (Cur));
            end if;
         end;
      end loop;
      return R;
   end Eye_Pixels;

   function Off_Eye (Eye : Px_Vectors.Vector; Cs : Corr_Vectors.Vector) return Corr_Vectors.Vector is
      Set : Px_Sets.Set;
      R : Corr_Vectors.Vector;
   begin
      if Eye.Is_Empty then
         return Cs;
      end if;
      for P of Eye loop
         Set.Include (P);
      end loop;
      for C of Cs loop
         if not Set.Contains (Px'(C.Ua, C.Va)) then
            R.Append (C);
         end if;
      end loop;
      return R;
   end Off_Eye;

   function On_Eye_Grid (Eye : Px_Vectors.Vector; U, V : Long_Float; W, H : Positive) return Boolean is
      Iu : constant Integer := Integer (Long_Float'Floor (U * Long_Float (Gx) / Long_Float (W)));
      Iv : constant Integer := Integer (Long_Float'Floor (V * Long_Float (Gy) / Long_Float (H)));
   begin
      if Iu < 0 or else Iv < 0 or else Iu >= Gx or else Iv >= Gy then
         return False;
      end if;
      declare
         --  问格点的那一个式子(Grid_U / Grid_V ⇒ 同一个数)
         Pu : constant Long_Float := Grid_U (Natural (Iu), W);
         Pv : constant Long_Float := Grid_V (Natural (Iv), H);
      begin
         for P of Eye loop
            if P.U = Pu and then P.V = Pv then
               return True;
            end if;
         end loop;
      end;
      return False;
   end On_Eye_Grid;

   procedure Fit_World (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                        M : out Model; Rep : out Fit_Report; Ok : out Boolean; Per_Pair : Positive) is
      Q0 : constant Floats := Frames (Ref).Q;
      N : constant Natural := Natural (Q0.Length);   --  几根轴 = 这组读数几个(09-30 以前最多 12,多出来的悄悄截掉)
      Nf : constant Natural := Natural (Frames.Length);
      Floor : constant Long_Float := Px_Res (Cs);    --  量到的配点噪声的下限,只防零(见 Px_Res)
      Fg : array (0 .. N_F - 1) of Long_Float;
      Tab : array (0 .. N - 1) of Cand_Table;
      Js : array (0 .. N - 1) of Jc_Vectors.Vector;     --  每根轴起步用的配点(全部)
      Jg : array (0 .. N - 1) of Jc_Vectors.Vector;     --  网格用的(转角小、每对抽样)
      Jgs : array (0 .. N - 1) of Jc_Vectors.Vector;    --  "走"那样网格用的(每对抽样;不按转角挑:按"走"解时读数差不是转角)
      Usable : array (0 .. N - 1) of Boolean := [others => False];
      Use_T, Use_S : array (0 .. N - 1) of Boolean := [others => False];   --  按"转" / 按"走"试得了(至少两格、网格有 200 个配点)
      Slide_Mid : array (0 .. N - 1) of Long_Float := [others => 1.0e18];   --  "走"那样网格最好的分数(焦距中间那档;跟焦距无关);没试 = 空位(无量纲)
      Sl : array (0 .. N - 1) of Boolean := [others => False];             --  认成"走"
      Dmax : Long_Float;
      function Dq (Fr, J : Natural) return Long_Float is (Frames (Fr).Q (J) - Q0 (J));
      --  这一帧能不能给第 J 根轴起步用:它是参照帧,或者只动了 J(Single_Joint:别的关节偏得让画面挪不到 1 像素,按最长那档焦距算,最严)
      function Clean (Fr, J : Natural) return Boolean is (Fr = Ref or else Single_Joint (Frames, Ref, Fr, Dmax) = Integer (J));
      T_Mark : Ada.Calendar.Time := Ada.Calendar.Clock;
      procedure Lap is
         use type Ada.Calendar.Time;
         Now : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         Rep.Secs.Append (Long_Float (Now - T_Mark));
         T_Mark := Now;
      end Lap;
      Wj : array (0 .. N - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Pj : array (0 .. N - 1) of V3 := [others => [1.0, 0.0, 0.0]];
      Xj : array (0 .. N - 1) of Vec (0 .. 2) := [others => [0.0, 0.0, 0.0]];   --  每根轴单独精修完的(两个方向角, φ)
      F0 : Long_Float := 0.0;
   begin
      Ok := False;
      Rep := (others => <>);
      M := (others => <>);
      M.N := N; M.Cx := Cx; M.Cy := Cy; M.Q0 := Q0;
      Rep.N_Corr := Natural (Cs.Length);
      --  焦距网格:视场从 110° 到 30°(协议:针孔相机的常见范围;半视场 55° → 15° = 0.9599 → 0.2618 弧度),焦距 = 半幅宽 ÷ tan(半视场),
      --  按对数等分 N_F 档(每档约 7%)
      declare
         F_Lo : constant Long_Float := 0.5 * Width / Tan (0.959931088596881);
         F_Hi : constant Long_Float := 0.5 * Width / Tan (0.261799387799490);
      begin
         for K in Fg'Range loop
            Fg (K) := F_Lo * Exp (Long_Float (K) / Long_Float (N_F - 1) * Log (F_Hi / F_Lo));
         end loop;
      end;
      Dmax := Clean_Tol (Width);
      --  配点分到各根轴
      declare
         Pos, Size : Nat_Vectors.Vector;
         --  每根轴有几对、其中转角小的有几对 ⇒ 每对拿几个:这根轴一共要 Grid_Target / All_Target 个,按对数平均分,每对至少 Per_Pair_Grid / Per_Pair_All
         --  (V1B7 2026-09-26:一段只扫 5 格,转角小的对一根轴只剩 6 对,每对 30 个凑不满 200 个,两根轴没量成)
         N_Pairs, N_Small : array (0 .. N - 1) of Natural := [others => 0];
         K_Grid, K_All, K_Grid_S : array (0 .. N - 1) of Natural := [others => 0];
      begin
         Pair_Pos (Cs, Pos, Size);
         for Ci in 0 .. Natural (Cs.Length) - 1 loop
            if Pos (Ci) = 0 then
               for J in 0 .. N - 1 loop
                  if Clean (Cs (Ci).I, J) and then Clean (Cs (Ci).J, J) and then (Frames (Cs (Ci).I).Joint = Integer (J) or else Frames (Cs (Ci).J).Joint = Integer (J)) then
                     N_Pairs (J) := N_Pairs (J) + 1;
                     if abs (Dq (Cs (Ci).I, J) - Dq (Cs (Ci).J, J)) <= Grid_Rad then
                        N_Small (J) := N_Small (J) + 1;
                     end if;
                  end if;
               end loop;
            end if;
         end loop;
         for J in 0 .. N - 1 loop
            K_Grid (J) := Natural'Max (Per_Pair_Grid, Grid_Target / Natural'Max (1, N_Small (J)));
            K_All (J) := Natural'Max (Per_Pair_All, All_Target / Natural'Max (1, N_Pairs (J)));
            K_Grid_S (J) := Natural'Max (Per_Pair_Grid, Grid_Target / Natural'Max (1, N_Pairs (J)));
         end loop;
         for Ci in 0 .. Natural (Cs.Length) - 1 loop
            declare
               C : constant Corr := Cs (Ci);
            begin
               for J in 0 .. N - 1 loop
                  if Clean (C.I, J) and then Clean (C.J, J) and then (Frames (C.I).Joint = Integer (J) or else Frames (C.J).Joint = Integer (J)) then
                     declare
                        R : constant Jc_Rec := (Ta => Dq (C.I, J), Tb => Dq (C.J, J), C => C);
                     begin
                        if Take (Pos (Ci), Size (Ci), K_All (J)) then
                           Js (J).Append (R);
                        end if;
                        if abs (R.Ta - R.Tb) <= Grid_Rad and then Take (Pos (Ci), Size (Ci), K_Grid (J)) then
                           Jg (J).Append (R);
                        end if;
                        if Take (Pos (Ci), Size (Ci), K_Grid_S (J)) then
                           Jgs (J).Append (R);
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end loop;
      end;
      for J in 0 .. N - 1 loop
         declare
            Nfr : Natural := 0;
         begin
            for Fr in 0 .. Nf - 1 loop
               if Fr /= Ref and then Clean (Fr, J) then
                  Nfr := Nfr + 1;
               end if;
            end loop;
            Rep.Joint_Frames.Append (Nfr + 1);
            Use_T (J) := Nfr >= 2 and then Natural (Jg (J).Length) >= 200;   --  至少两格、200 个配点才铺网格(次数)
            Use_S (J) := Nfr >= 2 and then Natural (Jgs (J).Length) >= 200;  --  同上(次数)
            Usable (J) := Use_T (J) or else Use_S (J);
         end;
      end loop;
      T_Mark := Ada.Calendar.Clock;
      --  ① 网格:每根轴、每个方向、每档焦距,φ 直接解,分数 = 残差中位。各根轴互不相干 ⇒ 一根轴一个线程
      --  (每个线程只碰自己那根轴的配点和候选表)
      declare
         task type Grid_Task is
            entry Start (J : Natural);
         end Grid_Task;
         task body Grid_Task is
            Jj : Natural := 0;
            Wk : Work;
            Ga : Jc_Array_Ptr;
         begin
            accept Start (J : Natural) do
               Jj := J;
            end Start;
            if Usable (Jj) then
               Wk := New_Work (Natural'Max (Natural (Jg (Jj).Length), Natural (Jgs (Jj).Length)));
               if Use_T (Jj) then
                  Ga := To_Array (Jg (Jj));
                  for Ks in 0 .. N_Sph - 1 loop
                     declare
                        W : constant V3 := Sphere (Ks, N_Sph);
                     begin
                        for Kf in Fg'Range loop
                           declare
                              Ph, Md : Long_Float;
                           begin
                              Best_Phi (Ga.all, Natural (Jg (Jj).Length), W, Fg (Kf), Cx, Cy, Floor, Ph, Md, Wk);
                              Insert (Tab (Jj) (Kf), (Score => Md, W => W, Phi => Ph));
                           end;
                        end loop;
                     end;
                  end loop;
                  Free (Ga);
               end if;
               --  "走"那样:分数跟焦距无关 ⇒ 只铺中间那一档,挑焦距时每档都用它
               if Use_S (Jj) then
                  declare
                     Gs : Jc_Array_Ptr := To_Array (Jgs (Jj));
                     Ts : Cand_Array;
                  begin
                     Slide_Grid (Gs.all, Natural (Jgs (Jj).Length), Fg (N_F / 2), Cx, Cy, Ts, Wk);
                     Slide_Mid (Jj) := Ts (0).Score;
                     Free (Gs);
                  end;
               end if;
               Free_Work (Wk);
            end if;
         end Grid_Task;
         Workers : array (0 .. N - 1) of Grid_Task;
      begin
         for J in 0 .. N - 1 loop
            Workers (J).Start (J);
         end loop;
      end;   --  这里等全部线程做完
      Lap;
      --  焦距各轴共用:每档各轴最好的分数加起来,取最小的那档(每根轴在每一档取"转""走"两样里好的那个:"走"的分数每档一样,不左右焦距)
      declare
         Best : Long_Float := Long_Float'Last;
         Kb : Natural := 0;
      begin
         for Kf in Fg'Range loop
            declare
               S : Long_Float := 0.0;
            begin
               for J in 0 .. N - 1 loop
                  if Usable (J) then
                     S := S + Long_Float'Min (Tab (J) (Kf) (0).Score, Slide_Mid (J));
                  end if;
               end loop;
               if S < Best then
                  Best := S; Kb := Kf;
               end if;
            end;
         end loop;
         F0 := Fg (Kb);
         Rep.F_Start := F0;
         --  焦距固定,每根轴从这一档的候选各自就地精修(先只用网格那批,再全部),按全部配点的残差中位挑。
         --  "转""走"两样各解一次(同一批全部配点 Js 上比):残差中位小的那样就是这根轴(一个量一种量法:两样都按同一批配点的 Sampson 残差比;
         --  试不了的那样 = 没有)。"走"的候选按这一档焦距重铺网格(方向的斜度随焦距变)
         for J in 0 .. N - 1 loop
            if not Usable (J) then
               Rep.Joint_Med.Append (-1.0); Rep.Joint_Med_Turn.Append (-1.0); Rep.Joint_Med_Slide.Append (-1.0); Rep.Slide.Append (False);
            else
               declare
                  Bm_T, Bm_S : Long_Float := 1.0e18;   --  空位(无量纲)
                  Bx_T, Bx_S : Vec (0 .. 2) := [0.0, 0.0, 0.0];
                  Bd_T, Bd_S : Boolean := True;        --  挑中的那个候选精修收住了没有
                  Sa : Jc_Array_Ptr := To_Array (Js (J));
                  --  从一个起点(X 的前 Nx 个数)精修:先网格那批(Gg),再全部;返回全部配点上的残差中位、两次 LM 都收住了没有。
                  --  每一次先按起点量配点噪声(Mad_Sigma × Sampson 残差的中位;网格那批按网格上的候选量,全部那批按网格那批解完的量),
                  --  残差除以它再进 Huber(09-30 以前直接喂像素,门 = 1 像素)
                  generic
                     Nx : Positive;
                     with procedure Res (Cs : Jc_Array; N : Natural; X : Vec; F, Cx, Cy : Long_Float; R : out Vec);
                  procedure Polish (Gg : Jc_Array_Ptr; Ng : Natural; X : in out Vec; Md : out Long_Float; Settled : out Boolean);
                  procedure Polish (Gg : Jc_Array_Ptr; Ng : Natural; X : in out Vec; Md : out Long_Float; Settled : out Boolean) is
                     Nall : constant Natural := Natural (Js (J).Length);
                     Xx : Vec (0 .. Nx - 1) := X (X'First .. X'First + Nx - 1);
                     Steps : constant Vec (0 .. Nx - 1) := [others => 1.0e-6];   --  差分步(弧度,极小量)
                     Sig : Long_Float := Floor;   --  这一次量到的配点噪声(像素)
                     D1, D2 : Boolean;
                     procedure R_Small (Xa : Vec; R : out Vec) is
                     begin
                        Res (Gg.all, Ng, Xa, F0, Cx, Cy, R);
                        for I in R'Range loop
                           R (I) := R (I) / Sig;
                        end loop;
                     end R_Small;
                     procedure R_All (Xa : Vec; R : out Vec) is
                     begin
                        Res (Sa.all, Nall, Xa, F0, Cx, Cy, R);
                        for I in R'Range loop
                           R (I) := R (I) / Sig;
                        end loop;
                     end R_All;
                     R : Vec_Ptr := new Vec (0 .. Natural'Max (Ng, Nall) - 1);
                  begin
                     Res (Gg.all, Ng, Xx, F0, Cx, Cy, R (0 .. Ng - 1));
                     Sig := Sigma_Of (Median_Abs (R (0 .. Ng - 1)), Floor);
                     Robust_LM (Xx, Ng, Ng, Axis_Iters, Steps, R_Small'Access, D1);
                     Res (Sa.all, Nall, Xx, F0, Cx, Cy, R (0 .. Nall - 1));
                     Sig := Sigma_Of (Median_Abs (R (0 .. Nall - 1)), Floor);
                     Robust_LM (Xx, Nall, Nall, Axis_Iters, Steps, R_All'Access, D2);
                     Res (Sa.all, Nall, Xx, F0, Cx, Cy, R (0 .. Nall - 1));
                     Md := Median_Abs (R (0 .. Nall - 1));
                     Settled := D1 and then D2;
                     Free (R);
                     X (X'First .. X'First + Nx - 1) := Xx;
                  end Polish;
                  procedure Polish_T is new Polish (3, Joint_Res);
                  procedure Polish_S is new Polish (2, Slide_Res);
               begin
                  if Use_T (J) then
                     declare
                        Ga : Jc_Array_Ptr := To_Array (Jg (J));
                     begin
                        for Ci in 0 .. Keep - 1 loop
                           --  空位 = 后面没有候选了(无量纲)。先判再取:空位的方向是占位的 (0, 0, 1),方位角算不出来(Arctan (0, 0) 抛异常;
                           --  原来先取后判,表一直是满的才没碰上 —— 09-27 试网格切份并行时,并表把彼此不到 10° 的两个并成一个,表不满,崩了)
                           exit when Tab (J) (Kb) (Ci).Score >= 1.0e17;
                           declare
                              Cd : constant Cand := Tab (J) (Kb) (Ci);
                              X : Vec (0 .. 2) := [Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Cd.W (2)))), Arctan (Cd.W (1), Cd.W (0)), Cd.Phi];
                              Md : Long_Float;
                              Dn : Boolean;
                           begin
                              Polish_T (Ga, Natural (Jg (J).Length), X, Md, Dn);
                              if Md < Bm_T then
                                 Bm_T := Md; Bx_T := X; Bd_T := Dn;
                              end if;
                           end;
                        end loop;
                        Free (Ga);
                     end;
                  end if;
                  if Use_S (J) then
                     declare
                        Gs : Jc_Array_Ptr := To_Array (Jgs (J));
                        Ts : Cand_Array;
                        Wk : Work := New_Work (Natural (Jgs (J).Length));
                     begin
                        Slide_Grid (Gs.all, Natural (Jgs (J).Length), F0, Cx, Cy, Ts, Wk);
                        Free_Work (Wk);
                        for Ci in 0 .. Keep - 1 loop
                           exit when Ts (Ci).Score >= 1.0e17;   --  空位(无量纲,同上)
                           declare
                              Cd : constant Cand := Ts (Ci);
                              X : Vec (0 .. 2) := [Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Cd.W (2)))), Arctan (Cd.W (1), Cd.W (0)), 0.0];
                              Md : Long_Float;
                              Dn : Boolean;
                           begin
                              Polish_S (Gs, Natural (Jgs (J).Length), X, Md, Dn);
                              if Md < Bm_S then
                                 Bm_S := Md; Bx_S := X; Bd_S := Dn;
                              end if;
                           end;
                        end loop;
                        Free (Gs);
                     end;
                  end if;
                  Free (Sa);
                  Sl (J) := Bm_S < Bm_T;
                  Rep.Joint_Med_Turn.Append (if Use_T (J) then Bm_T else -1.0);
                  Rep.Joint_Med_Slide.Append (if Use_S (J) then Bm_S else -1.0);
                  Rep.Slide.Append (Sl (J));
                  Rep.Joint_Med.Append (if Sl (J) then Bm_S else Bm_T);
                  if not (if Sl (J) then Bd_S else Bd_T) then
                     Note (Rep, "① 第" & Natural'Image (J) & " 根轴精修 LM" & Natural'Image (Axis_Iters) & " 次还在降");
                  end if;
                  Xj (J) := (if Sl (J) then Bx_S else Bx_T);
                  Wj (J) := Ang_W (Xj (J) (0), Xj (J) (1));
                  if Sl (J) then
                     Pj (J) := [0.0, 0.0, 0.0];
                  else
                     declare
                        E1, E2 : V3;
                     begin
                        Perp (Wj (J), E1, E2);
                        Pj (J) := Add (Scl (E1, Cos (Xj (J) (2))), Scl (E2, Sin (Xj (J) (2))));
                     end;
                  end if;
               end;
            end if;
         end loop;
      end;
      Lap;
      for J in 0 .. N - 1 loop
         if not Usable (J) then
            return;   --  有一根轴量不了 ⇒ 整只手的模型不完整;Rep.Joint_Frames / Joint_Med 里照实写着是哪根
         end if;
      end loop;
      --  ①b 焦距和各轴一起精修(焦距是几根轴共用的):网格那一档到真焦距最多差半档(约 3.5%),各轴在那一档上各自精修会一起歪
      --  (V1B13 2026-09-26:起步挑到 419.7、真 397 ⇒ 每根轴单独的残差翻倍,后面定比例解错,运动学差 14 mm;V1B12 同样的起步,碰巧解回来)
      --  每根轴的数:"转" 3 个(方向两个角 + 轴在眼哪边),"走" 2 个(方向两个角);X = [对数焦距, 第 0 根的, 第 1 根的, …]
      declare
         function Size_Of (J : Natural) return Positive is (if Sl (J) then 2 else 3);
         Off : array (0 .. N - 1) of Natural := [others => 0];
         Nx : Natural := 1;
      begin
         for J in 0 .. N - 1 loop
            Off (J) := Nx; Nx := Nx + Size_Of (J);
         end loop;
         declare
            X : Vec (0 .. Nx - 1);
            Steps : constant Vec (0 .. Nx - 1) := [others => 1.0e-6];   --  差分步(弧度 / 对数焦距,极小量,无量纲)
            Arrs : array (0 .. N - 1) of Jc_Array_Ptr;
            Ns : array (0 .. N - 1) of Natural := [others => 0];
            N_Tot : Natural := 0;
            Sig : Long_Float := Floor;   --  量到的配点噪声(像素):一台相机、一个配点仪器,各轴的配点一个噪声(起步各轴解完的残差一起量)
            procedure R_Px (Xx : Vec; R : out Vec) is
               Fx : constant Long_Float := Exp (Xx (Xx'First));
               K : Natural := R'First;
            begin
               for J in 0 .. N - 1 loop
                  if Sl (J) then
                     Slide_Res (Arrs (J).all, Ns (J), Xx (Xx'First + Off (J) .. Xx'First + Off (J) + 1), Fx, Cx, Cy, R (K .. K + Ns (J) - 1));
                  else
                     Joint_Res (Arrs (J).all, Ns (J), Xx (Xx'First + Off (J) .. Xx'First + Off (J) + 2), Fx, Cx, Cy, R (K .. K + Ns (J) - 1));
                  end if;
                  K := K + Ns (J);
               end loop;
            end R_Px;
            --  进 Huber 的:像素残差除以量到的噪声
            procedure R_All (Xx : Vec; R : out Vec) is
            begin
               R_Px (Xx, R);
               for I in R'Range loop
                  R (I) := R (I) / Sig;
               end loop;
            end R_All;
         begin
            for J in 0 .. N - 1 loop
               Arrs (J) := To_Array (Js (J)); Ns (J) := Natural (Js (J).Length); N_Tot := N_Tot + Ns (J);
               X (Off (J) .. Off (J) + Size_Of (J) - 1) := Xj (J) (0 .. Size_Of (J) - 1);
            end loop;
            X (0) := Log (F0);
            if N_Tot > Nx then
               declare
                  R : Vec_Ptr := new Vec (0 .. N_Tot - 1);
                  Dn : Boolean;
               begin
                  R_Px (X, R.all);
                  Sig := Sigma_Of (Median_Abs (R.all), Floor);
                  Free (R);
                  Robust_LM (X, N_Tot, N_Tot, Axis_Iters, Steps, R_All'Access, Dn);
                  if not Dn then
                     Note (Rep, "①b 焦距和各轴一起精修 LM" & Natural'Image (Axis_Iters) & " 次还在降");
                  end if;
               end;
            end if;
            F0 := Exp (X (0));
            for J in 0 .. N - 1 loop
               Wj (J) := Ang_W (X (Off (J)), X (Off (J) + 1));
               if not Sl (J) then
                  declare
                     E1, E2 : V3;
                  begin
                     Perp (Wj (J), E1, E2);
                     Pj (J) := Add (Scl (E1, Cos (X (Off (J) + 2))), Scl (E2, Sin (X (Off (J) + 2))));
                  end;
               end if;
               Free (Arrs (J));
            end loop;
            Rep.F_Axes := F0;
         end;
      end;
      Lap;
      --  ② 各轴离眼远近的比例 ρ(可正可负:Sampson 分不出轴在眼的这边还是那边)。
      --  ① 定了每根轴的方向 W、"轴在眼哪边"的方向 p̂ ⇒ 眼的位置 t(q) = Σ ρ_j a_j(q)(a_j = 前面各轴的转动 ·(I − 这根轴的转动)· p̂_j)对 ρ 是线性的:
      --  两帧之间的平移 Tij = Σ ρ_j b_j,每个配点的对极约束 Tij · (y × h2) = 0 就是一条 g · ρ = 0(g_j = b_j · (y × h2)),Sampson 的分母 |E ρ| 也对 ρ 线性。
      --  只用两帧加起来至少有两个关节离开参照读数的对(只转一个关节的对跟 ρ 无关)。全局解,不从哪根轴一根接一根定:
      --  任取三对(一对管两个数,三对管得住五个比例)解一次(最小特征向量,各列先按大小归一),拿全部这些配点打分、取最好的:
      --  分数 = Σ min(r², (3 px)²)(截断平方和,3 px 同 ③ 挑内点的门)—— 不用中位数:这些配点多数来自相邻关节头一格那几对(各转 1.7°、平移很小,
      --  比例怎么取残差都小),中位数被它们占住分不出好坏(V1B10 回放:第 1 只手按中位挑到错的比例);乱配的对每个 ρ 都是满额,不影响比较。
      --  再在门里(max(3 px, 3 × 中位))按 Sampson 加权反复重解。
      --  (V1B10 2026-09-26:原来从相邻关节头一格一根接一根定、再局部精修,第 2 只手落进错的坑:各轴比例 −0.97 0.35 1 0.33 0.65 0.001,真的约 0.42 0.45 1 0.43 0.26 0.11)
      declare
         Cs_Thin : constant Corr_Vectors.Vector := Thin (Cs, Per_Pair_All);
         Af : array (0 .. Nf - 1, 0 .. N - 1) of V3;   --  每一帧各轴的 a_j(q)(参照眼系,p̂ 为单位长)
         Rq : array (0 .. Nf - 1) of M3;                         --  每一帧整串的转动
         Seen : array (0 .. N - 1) of Boolean := [others => False];
         subtype Row_N is Rho_Row (N - 1);
         type Row_Array is array (Natural range <>) of Row_N;
         type Row_Ptr is access Row_Array;
         procedure Free is new Ada.Unchecked_Deallocation (Row_Array, Row_Ptr);
         Rows : Row_Ptr;
         N_Rows : Natural := 0;
         P_First, P_Count : Nat_Vectors.Vector;   --  每一对:在 Rows 里从第几行起、几行
         Rho, Best_Rho : Vec (0 .. N - 1) := [others => 0.0];
         Rf : Natural := 0;
         function Away (Fr, J : Natural) return Boolean is (abs Dq (Fr, J) >= Dmax);
         function Informative (I, J : Natural) return Boolean is
            K : Natural := 0;
         begin
            for X in 0 .. N - 1 loop
               if Away (I, X) or else Away (J, X) then
                  K := K + 1;
               end if;
            end loop;
            return K >= 2;
         end Informative;
         --  按选中的行、权解一次:各列先按加权均方根归一(不然哪根轴的列小,解就全落到那根轴上)
         procedure Solve (Sel : Nat_Array; Wt : Vec; Out_Rho : out Vec) is
            A : Mat (0 .. N - 1, 0 .. N - 1) := [others => [others => 0.0]];
            D : Vec (0 .. N - 1) := [others => 0.0];
            Sw : Long_Float := 0.0;
         begin
            for K in Sel'Range loop
               for X in 0 .. N - 1 loop
                  D (X) := D (X) + Wt (Wt'First + K - Sel'First) * Rows (Sel (K)).G (X) ** 2;
               end loop;
               Sw := Sw + Wt (Wt'First + K - Sel'First);
            end loop;
            for X in 0 .. N - 1 loop
               D (X) := (if D (X) > 0.0 and then Sw > 0.0 then Sqrt (D (X) / Sw) else 1.0);
            end loop;
            for K in Sel'Range loop
               declare
                  W : constant Long_Float := Wt (Wt'First + K - Sel'First);
                  Gs : Vec (0 .. N - 1) := [others => 0.0];
               begin
                  if W > 0.0 then
                     for X in 0 .. N - 1 loop
                        Gs (X) := Rows (Sel (K)).G (X) / D (X);
                     end loop;
                     for X in 0 .. N - 1 loop
                        for Y in X .. N - 1 loop
                           A (X, Y) := A (X, Y) + W * Gs (X) * Gs (Y);
                        end loop;
                     end loop;
                  end if;
               end;
            end loop;
            for X in 0 .. N - 1 loop
               for Y in 0 .. X - 1 loop
                  A (X, Y) := A (Y, X);
               end loop;
            end loop;
            declare
               E : constant Vec := Min_Eig (A, N);
               Nr : Long_Float := 0.0;
            begin
               Out_Rho := [others => 0.0];
               for X in 0 .. N - 1 loop
                  Out_Rho (Out_Rho'First + X) := E (X) / D (X);
                  Nr := Nr + Out_Rho (Out_Rho'First + X) ** 2;
               end loop;
               if Nr > 0.0 then
                  for X in 0 .. N - 1 loop
                     Out_Rho (Out_Rho'First + X) := Out_Rho (Out_Rho'First + X) / Sqrt (Nr);
                  end loop;
               end if;
            end;
         end Solve;
         --  按 Sampson 加权反复重解(Huber:残差除以这一遍量到的噪声 Mad_Sigma × 中位,门 Huber_K;Gated = 门外的不要,
         --  门 = max(3 px, 3 × 中位),每遍重算)。Settled = 做到不再变了(False = 做满 Iters 还在变)
         procedure Refine (Sel : Nat_Array; R : in out Vec; Iters : Natural; Gated : Boolean; Settled : out Boolean) is
            Cnt : constant Natural := Sel'Length;
            Wt : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
            Rs : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
            Tmp : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
         begin
            Settled := False;
            for It in 1 .. Iters loop
               declare
                  Den : Long_Float;
                  Gate : Long_Float := Long_Float'Last;
                  New_R : Vec (0 .. N - 1);
                  Dot_R, Ch : Long_Float := 0.0;
                  Med, Sig : Long_Float;
               begin
                  for K in 0 .. Cnt - 1 loop
                     Rs (K) := Rho_Res (Rows (Sel (Sel'First + K)), R, F0, Den);
                     Wt (K) := (F0 / Den) ** 2;
                  end loop;
                  Med := Median_In (Rs.all, Cnt, Tmp);
                  Sig := Sigma_Of (Med, Floor);
                  if Gated then
                     Gate := Long_Float'Max (3.0, 3.0 * Med);   --  3 px / 3 倍中位(协议,同 ③)
                  end if;
                  for K in 0 .. Cnt - 1 loop
                     Wt (K) := (if abs Rs (K) >= Gate then 0.0 else Wt (K) * Huber_W (Rs (K) / Sig));
                  end loop;
                  Solve (Sel, Wt (0 .. Cnt - 1), New_R);
                  for X in 0 .. N - 1 loop
                     Dot_R := Dot_R + New_R (X) * R (R'First + X);
                  end loop;
                  if Dot_R < 0.0 then
                     for X in 0 .. N - 1 loop
                        New_R (X) := -New_R (X);
                     end loop;
                  end if;
                  for X in 0 .. N - 1 loop
                     Ch := Ch + (New_R (X) - R (R'First + X)) ** 2;
                  end loop;
                  R := New_R;
                  if Ch < 1.0e-20 then   --  不再变了(数值,无量纲)
                     Settled := True;
                     exit;
                  end if;
               end;
            end loop;
            Free (Wt); Free (Rs); Free (Tmp);
         end Refine;
      begin
         for Fr in 0 .. Nf - 1 loop
            declare
               R : M3 := Identity;
            begin
               for J in 0 .. N - 1 loop
                  if Sl (J) then
                     Af (Fr, J) := Ap (R, Scl (Wj (J), Dq (Fr, J)));   --  走的关节:沿方向走 θ 个读数单位(每单位走多远 = ρ),不转
                  else
                     declare
                        Rj : constant M3 := Rot (Wj (J), Dq (Fr, J));
                     begin
                        Af (Fr, J) := Ap (R, Sub (Pj (J), Ap (Rj, Pj (J))));
                        R := Mul (R, Rj);
                     end;
                  end if;
               end loop;
               Rq (Fr) := R;
            end;
         end loop;
         for C of Cs_Thin loop
            if Informative (C.I, C.J) then
               N_Rows := N_Rows + 1;
            end if;
         end loop;
         Rows := new Row_Array (0 .. Natural'Max (1, N_Rows) - 1);
         declare
            K : Natural := 0;
            Last_I, Last_J : Integer := -1;
         begin
            for C of Cs_Thin loop
               if Informative (C.I, C.J) then
                  if Integer (C.I) /= Last_I or else Integer (C.J) /= Last_J then
                     P_First.Append (K); P_Count.Append (0);
                     Last_I := Integer (C.I); Last_J := Integer (C.J);
                     for X in 0 .. N - 1 loop
                        if Away (C.I, X) or else Away (C.J, X) then
                           Seen (X) := True;
                        end if;
                     end loop;
                  end if;
                  P_Count.Replace_Element (P_Count.Last_Index, P_Count.Last_Element + 1);
                  declare
                     Rij : constant M3 := Mul (Tr (Rq (C.J)), Rq (C.I));
                     H1 : constant V3 := [(C.Ua - Cx) / F0, -(C.Va - Cy) / F0, -1.0];
                     H2 : constant V3 := [(C.Ub - Cx) / F0, -(C.Vb - Cy) / F0, -1.0];
                     Y : constant V3 := Ap (Rij, H1);
                     Yh : constant V3 := Cross (Y, H2);
                  begin
                     for X in 0 .. N - 1 loop
                        declare
                           B : constant V3 := ApT (Rq (C.J), Sub (Af (C.I, X), Af (C.J, X)));
                           Ex : constant V3 := Cross (B, Y);
                           Et : constant V3 := ApT (Rij, Cross (H2, B));
                        begin
                           Rows (K).G (X) := Dot (B, Yh);
                           Rows (K).E0 (X) := Ex (0); Rows (K).E1 (X) := Ex (1);
                           Rows (K).E2 (X) := Et (0); Rows (K).E3 (X) := Et (1);
                        end;
                     end loop;
                  end;
                  K := K + 1;
               end if;
            end loop;
         end;
         Rep.Rho_Pairs := Natural (P_First.Length);
         for X in 0 .. N - 1 loop
            if not Seen (X) then
               Rep.Rho.Clear;
               for I in 0 .. N - 1 loop
                  Rep.Rho.Append (if Seen (I) then 1.0 else 0.0);   --  0 = 这根轴在哪一对里都没离开参照读数,比例定不了
               end loop;
               Free (Rows);
               return;
            end if;
         end loop;
         --  任取三对:每对最多 Per_Pair_Grid 行来解、来打分
         declare
            Np : constant Natural := Natural (P_First.Length);
            Sc : Nat_Array (0 .. Natural'Max (1, N_Rows) - 1);
            Sc_First, Sc_Cnt : Nat_Vectors.Vector;
            Nsc : Natural := 0;
            Best_Score : Long_Float := Long_Float'Last;
            Tau : constant := 3.0;   --  截断的门(像素,协议:同 ③ 挑内点的 3 px)
            Rs : Vec_Ptr;
            Tmp : Vec_Ptr;
         begin
            for P in 0 .. Np - 1 loop
               Sc_First.Append (Nsc);
               for K in 0 .. P_Count (P) - 1 loop
                  if Take (K, P_Count (P), Per_Pair_Grid) then
                     Sc (Nsc) := P_First (P) + K; Nsc := Nsc + 1;
                  end if;
               end loop;
               Sc_Cnt.Append (Nsc - Sc_First (P));
            end loop;
            Rs := new Vec (0 .. Natural'Max (1, Nsc) - 1);
            Tmp := new Vec (0 .. Natural'Max (1, Nsc) - 1);
            for Pa in 0 .. Np - 1 loop
               for Pb in Pa + 1 .. Np - 1 loop
                  for Pc in Pb + 1 .. Np - 1 loop
                     declare
                        Nt : constant Natural := Sc_Cnt (Pa) + Sc_Cnt (Pb) + Sc_Cnt (Pc);
                        Tri : Nat_Array (0 .. Natural'Max (1, Nt) - 1);
                        K : Natural := 0;
                        R0 : Vec (0 .. N - 1);
                        Den : Long_Float;
                        Dn : Boolean;
                     begin
                        for P of Nat_Array'[Pa, Pb, Pc] loop
                           for I in 0 .. Sc_Cnt (P) - 1 loop
                              Tri (K) := Sc (Sc_First (P) + I); K := K + 1;
                           end loop;
                        end loop;
                        if Nt > 0 then
                           Solve (Tri (0 .. Nt - 1), Vec'(0 .. Nt - 1 => 1.0), R0);
                           Refine (Tri (0 .. Nt - 1), R0, 2, Gated => False, Settled => Dn);   --  两遍 Sampson 加权(次数;起步只挑候选)
                           declare
                              Sm : Long_Float := 0.0;
                           begin
                              for I in 0 .. Nsc - 1 loop
                                 Sm := Sm + Long_Float'Min (Rho_Res (Rows (Sc (I)), R0, F0, Den) ** 2, Tau ** 2);
                              end loop;
                              if Sm < Best_Score then
                                 Best_Score := Sm; Best_Rho := R0;
                              end if;
                           end;
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
            Free (Rs); Free (Tmp);
            Rep.Rho_Start_Px := Sqrt (Best_Score / Long_Float (Natural'Max (1, Nsc)));
         end;
         --  全部这些配点,门里按 Sampson 加权反复重解
         declare
            All_Sel : Nat_Array (0 .. Natural'Max (1, N_Rows) - 1);
            Rs : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_Rows) - 1);
            Tmp : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_Rows) - 1);
            Den : Long_Float;
         begin
            for K in All_Sel'Range loop
               All_Sel (K) := K;
            end loop;
            Rho := Best_Rho;
            if N_Rows > 0 then
               declare
                  Rho_Iters : constant := 30;   --  最多几遍(保险:不再变就停;做满还在变照实记进 Rep.Unsettled)
                  Dn : Boolean;
               begin
                  Refine (All_Sel (0 .. N_Rows - 1), Rho, Rho_Iters, Gated => True, Settled => Dn);
                  if not Dn then
                     Note (Rep, "② 定比例重解" & Natural'Image (Rho_Iters) & " 遍还在变");
                  end if;
               end;
               for K in 0 .. N_Rows - 1 loop
                  Rs (K) := Rho_Res (Rows (K), Rho, F0, Den);
               end loop;
               Rep.Rho_Px := Median_In (Rs.all, N_Rows, Tmp);
            end if;
            Free (Rs); Free (Tmp);
         end;
         Free (Rows);
         for J in 1 .. N - 1 loop
            if abs Rho (J) > abs Rho (Rf) then
               Rf := J;
            end if;
         end loop;
         Rep.Ref_Joint := Rf;
         for J in 0 .. N - 1 loop
            Rep.Rho.Append (if Rho (Rf) /= 0.0 then Rho (J) / Rho (Rf) else 0.0);
            if Sl (J) then
               Wj (J) := Scl (Wj (J), Rho (J));   --  走的关节:ρ = 每个读数单位走多远(带正负)
            else
               Pj (J) := Scl (Pj (J), Rho (J));
            end if;
         end loop;
      end;
      Lap;
      --  ③ 挑内点(按起步模型:残差 < max(3 px, 3 倍中位)),全部一起按像素解(焦距放开;尺度钉"参与的各帧眼的位置均方根 = 1")
      declare
         M0 : Model := M;
         Inl : Corr_Vectors.Vector;
         Used_Frame : array (0 .. Nf - 1) of Boolean := [others => False];
         Nc : constant Natural := Natural (Cs.Length);
         type Set is array (0 .. Natural'Max (1, Nc) - 1) of Boolean;
         In_Set, Prev_Set : Set := [others => False];   --  这一轮 / 上一轮挑出来的内点
         Sig : Long_Float := Floor;   --  这一轮量到的配点噪声(像素):全部配点 Sampson 残差的 Mad_Sigma × 中位(同挑内点那个中位)
         Joint_Iters : constant := 300;   --  一起解的 LM 最多几次(保险:收住了就停;做满还在降照实记进 Rep.Unsettled)
      begin
         M0.F := F0;
         for J in 0 .. N - 1 loop
            M0.Ax (J) := (W => Wj (J), P => Pj (J), Slide => Sl (J));
         end loop;
         --  按模型挑内点、一起解,做到挑出来的不再变(09-30 以前固定两轮:第一轮按起步模型、第二轮按第一轮解出的;
         --  自检:起步焦距差 5% 时考试中位 0.57 → 0.91 mm —— 起步差的时候两轮不一定收得住)
         for Round in 1 .. Round_Cap + 1 loop
            declare
               R : Vec_Ptr := new Vec (0 .. Natural'Max (1, Nc) - 1);
               Gate, Med : Long_Float;
               Same : Boolean := Round > 1;
            begin
               declare
                  Pc : Pose_Array (0 .. Nf - 1);
               begin
                  All_Poses (M0, Frames, Pc);
                  for I in 0 .. Nc - 1 loop
                     R (I) := Res_Cached (M0, Pc, Cs (I));
                  end loop;
               end;
               Med := Median_Abs (R (0 .. Nc - 1));
               Gate := Long_Float'Max (3.0, 3.0 * Med);   --  3 px / 3 倍中位(协议:配点残差按像素记)
               for I in 0 .. Nc - 1 loop
                  In_Set (I) := abs R (I) < Gate;
                  if In_Set (I) /= Prev_Set (I) then
                     Same := False;
                  end if;
               end loop;
               Free (R);
               exit when Same;   --  挑出来的和上一轮一样:上一轮解出的就是
               if Round > Round_Cap then
                  Note (Rep, "③ 重挑内点" & Natural'Image (Round_Cap) & " 轮还在变");
                  exit;
               end if;
               Rep.Rounds := Round;
               Prev_Set := In_Set;
               Sig := Sigma_Of (Med, Floor);
               Inl.Clear;
               Used_Frame := [others => False];
               for I in 0 .. Nc - 1 loop
                  if In_Set (I) then
                     Inl.Append (Cs (I));
                     Used_Frame (Cs (I).I) := True; Used_Frame (Cs (I).J) := True;
                  end if;
               end loop;
            end;
            --  一起解的时候每一对最多 Per_Pair 个(均匀隔着取;5 分钟一炮:V1B3 一起解用了 11 万个配点、75–146 秒)
            Inl := Thin (Inl, Per_Pair);
            Rep.N_Used := Natural (Inl.Length);
            if Inl.Is_Empty then
               return;
            end if;
            --  先把尺度钉到"参与的各帧眼的位置均方根 = 1"
            declare
               S2 : Long_Float := 0.0;
               Cnt : Natural := 0;
               Rr : M3;
               Tt : V3;
            begin
               for Fr in 0 .. Nf - 1 loop
                  if Used_Frame (Fr) then
                     FK (M0, Frames (Fr).Q, Rr, Tt);
                     S2 := S2 + Dot (Tt, Tt); Cnt := Cnt + 1;
                  end if;
               end loop;
               if Cnt > 0 and then S2 > 0.0 then
                  Scale_Model (M0, 1.0 / Sqrt (S2 / Long_Float (Cnt)));
               end if;
            end;
            declare
               Np : constant Natural := N_Params (M0);
               X : Vec (0 .. Np - 1);
               Steps : Vec (0 .. Np - 1);
               Ni : constant Natural := Natural (Inl.Length);
               Nt_Ax : constant Natural := N_Turn (M0);
               N_Reg : constant Natural := 2 * Nt_Ax + 1;
               function To_Model (Xx : Vec) return Model is (From_X (M0, Xx));
               procedure R_Px (Xx : Vec; R : out Vec) is
                  Mm : constant Model := To_Model (Xx);
                  Rf : array (0 .. Nf - 1) of M3;
                  Tf : array (0 .. Nf - 1) of V3;
                  S2 : Long_Float := 0.0;
                  Cnt : Natural := 0;
               begin
                  for Fr in 0 .. Nf - 1 loop
                     if Used_Frame (Fr) then
                        FK (Mm, Frames (Fr).Q, Rf (Fr), Tf (Fr));
                        S2 := S2 + Dot (Tf (Fr), Tf (Fr)); Cnt := Cnt + 1;
                     end if;
                  end loop;
                  for I in 0 .. Ni - 1 loop
                     declare
                        C : constant Corr := Inl (I);
                        Rij : M3;
                        Tij : V3;
                     begin
                        Rel (Rf (C.I), Tf (C.I), Rf (C.J), Tf (C.J), Rij, Tij);
                        R (R'First + I) := Samp (Rij, Tij, Mm.F, Cx, Cy, C);
                     end;
                  end loop;
                  --  约束行(不加权):转的轴是单位向量、P 取轴上离参照眼最近那点;尺度钉住(倍数只管数值,1e3 = 这几行比像素残差重得多,比例)
                  Axis_Rows (Mm, R (R'First + Ni .. R'First + Ni + 2 * Nt_Ax - 1));
                  R (R'First + Ni + 2 * Nt_Ax) := 1.0e3 * ((if Cnt > 0 then Sqrt (S2 / Long_Float (Cnt)) else 1.0) - 1.0);
               end R_Px;
               --  进 Huber 的:配点的像素残差除以这一轮量到的噪声(09-30 以前直接喂像素,门 = 1 像素);约束行原样
               procedure R_All (Xx : Vec; R : out Vec) is
               begin
                  R_Px (Xx, R);
                  for I in 0 .. Ni - 1 loop
                     R (R'First + I) := R (R'First + I) / Sig;
                  end loop;
               end R_All;
               Dn : Boolean;
            begin
               for J in 0 .. N - 1 loop
                  if not M0.Ax (J).Slide then   --  转的轴:P 取轴上离参照眼最近那点
                     M0.Ax (J).P := Sub (M0.Ax (J).P, Scl (M0.Ax (J).W, Dot (M0.Ax (J).W, M0.Ax (J).P)));
                  end if;
               end loop;
               To_X (M0, X);
               for K in Steps'Range loop
                  Steps (K) := 1.0e-7;   --  差分步(无量纲 / 模型单位 / 对数焦距,极小量)
               end loop;
               Robust_LM (X, Ni + N_Reg, Ni, Joint_Iters, Steps, R_All'Access, Dn);
               if not Dn then
                  Note (Rep, "③ 一起解 LM" & Natural'Image (Joint_Iters) & " 次还在降(第" & Natural'Image (Round) & " 轮)");
               end if;
               M := To_Model (X);
               for J in 0 .. N - 1 loop
                  if not M.Ax (J).Slide then
                     M.Ax (J).W := Unit (M.Ax (J).W);
                  end if;
               end loop;
               declare
                  R : Vec_Ptr := new Vec (0 .. Ni + N_Reg - 1);
               begin
                  R_Px (X, R.all);
                  Rep.Med_Px := Median_Abs (R (0 .. Ni - 1));
                  Rep.P90_Px := Quantile_Abs (R (0 .. Ni - 1), 0.9);   --  九成分位(比例,只报数)
                  Free (R);
               end;
            end;
            M0 := M;
         end loop;
         Lap;
         Rep.F := M.F;
         Rep.Sig_Px := Sig;
         --  平移整体的正负号:Sampson 分不出(t → −t 残差不变)⇒ 按"配点三角出来的点在两只眼前面"定;多数在后面就整体反号
         declare
            Front, Back : Natural := 0;
            Stride : constant Positive := Positive'Max (1, Natural (Inl.Length) / 2000);   --  抽 2000 个查(次数)
            I : Natural := 0;
         begin
            while I < Natural (Inl.Length) loop
               declare
                  C : constant Corr := Inl (I);
                  Ri, Rj, Rij : M3;
                  Ti, Tj, Tij : V3;
                  D1 : constant V3 := [(C.Ua - Cx) / M.F, -(C.Va - Cy) / M.F, -1.0];
                  D2 : constant V3 := [(C.Ub - Cx) / M.F, -(C.Vb - Cy) / M.F, -1.0];
               begin
                  FK (M, Frames (C.I).Q, Ri, Ti);
                  FK (M, Frames (C.J).Q, Rj, Tj);
                  Rel (Ri, Ti, Rj, Tj, Rij, Tij);
                  declare
                     A : constant V3 := Ap (Rij, D1);
                     --  λ1 A + Tij = λ2 D2(最小二乘)
                     Aa : constant Long_Float := Dot (A, A);
                     Ab : constant Long_Float := -Dot (A, D2);
                     Bb : constant Long_Float := Dot (D2, D2);
                     R1 : constant Long_Float := -Dot (A, Tij);
                     R2 : constant Long_Float := Dot (D2, Tij);
                     Det : constant Long_Float := Aa * Bb - Ab * Ab;
                  begin
                     if abs Det > 1.0e-12 and then Norm (Tij) > 1.0e-9 then
                        declare
                           L1 : constant Long_Float := (R1 * Bb - Ab * R2) / Det;
                           L2 : constant Long_Float := (Aa * R2 - Ab * R1) / Det;
                        begin
                           if L1 > 0.0 and then L2 > 0.0 then
                              Front := Front + 1;
                           elsif L1 < 0.0 and then L2 < 0.0 then
                              Back := Back + 1;
                           end if;
                        end;
                     end if;
                  end;
               end;
               I := I + Stride;
            end loop;
            if Back > Front then
               Scale_Model (M, -1.0);   --  转的轴 P 反号、走的关节 W 反号:所有平移一起反号,转动不变
               Rep.Flipped := True;
            end if;
         end;
         --  ④ 多视图:有轨迹就按重投影一起解(③ 的结果当起步),做到不再变好
         Refine_Until_Done (Frames, Cs, M, Rep);
         M.Valid := True;
         Ok := True;
      end;
   end Fit_World;

   procedure Fit (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                  M : out Model; Rep : out Fit_Report; Ok : out Boolean; Per_Pair : Positive := 60) is
      Eye : constant Px_Vectors.Vector := Eye_Pixels (Frames, Ref, Cs, Width);
      Kept : constant Corr_Vectors.Vector := Off_Eye (Eye, Cs);
   begin
      Fit_World (Frames, Ref, Kept, Cx, Cy, Width, M, Rep, Ok, Per_Pair);
      M.Eye := Eye;
      Rep.N_Corr := Natural (Cs.Length);
      Rep.Eye_Px := Natural (Eye.Length);
      Rep.Eye_Corrs := Natural (Cs.Length) - Natural (Kept.Length);
   end Fit;

   --  4×4 对称阵的特征分解(循环 Jacobi):返回最大特征值的特征向量
   function Max_Eigvec4 (N0 : Mat) return Vec is
      A : Mat := N0;
      V : Mat (0 .. 3, 0 .. 3) := [others => [others => 0.0]];
      Best : Natural := 0;
      Out_V : Vec (0 .. 3);
   begin
      for I in 0 .. 3 loop
         V (I, I) := 1.0;
      end loop;
      for Sweep in 1 .. 100 loop   --  最多 100 遍(次数)
         declare
            Off : Long_Float := 0.0;
         begin
            for P in 0 .. 2 loop
               for Q in P + 1 .. 3 loop
                  Off := Off + A (P, Q) ** 2;
               end loop;
            end loop;
            exit when Off < 1.0e-30;   --  非对角元已经是零(数值,无量纲)
            for P in 0 .. 2 loop
               for Q in P + 1 .. 3 loop
                  if abs A (P, Q) > 1.0e-300 then   --  数值保护(无量纲)
                     declare
                        Th : constant Long_Float := 0.5 * Arctan (2.0 * A (P, Q), A (Q, Q) - A (P, P));
                        C : constant Long_Float := Cos (Th);
                        Sn : constant Long_Float := Sin (Th);
                     begin
                        for K in 0 .. 3 loop
                           declare
                              Akp : constant Long_Float := A (K, P);
                              Akq : constant Long_Float := A (K, Q);
                           begin
                              A (K, P) := C * Akp - Sn * Akq;
                              A (K, Q) := Sn * Akp + C * Akq;
                           end;
                        end loop;
                        for K in 0 .. 3 loop
                           declare
                              Apk : constant Long_Float := A (P, K);
                              Aqk : constant Long_Float := A (Q, K);
                           begin
                              A (P, K) := C * Apk - Sn * Aqk;
                              A (Q, K) := Sn * Apk + C * Aqk;
                           end;
                        end loop;
                        for K in 0 .. 3 loop
                           declare
                              Vkp : constant Long_Float := V (K, P);
                              Vkq : constant Long_Float := V (K, Q);
                           begin
                              V (K, P) := C * Vkp - Sn * Vkq;
                              V (K, Q) := Sn * Vkp + C * Vkq;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
            end loop;
         end;
      end loop;
      for I in 1 .. 3 loop
         if A (I, I) > A (Best, Best) then
            Best := I;
         end if;
      end loop;
      for I in 0 .. 3 loop
         Out_V (I) := V (I, Best);
      end loop;
      return Out_V;
   end Max_Eigvec4;

   --  确定性的伪随机(同一份数据同一个结果):线性同余
   procedure Next (Seed : in out Unsigned_Seed; K : Natural; Out_I : out Natural) is
   begin
      Seed := Seed * 6364136223846793005 + 1442695040888963407;   --  线性同余的乘数 / 增量(Knuth MMIX,协议)
      Out_I := Natural ((Seed / 2 ** 33) mod Unsigned_Seed (K));
   end Next;

   procedure Robust_Plane (X : V3_Array; P0, Nrm : out V3; Inliers : out Natural; Med : out Long_Float) is
      N : constant Natural := X'Length;
      Seed : Unsigned_Seed := 20260926;
      Best_Med : Long_Float := Long_Float'Last;
      Res : Vec (0 .. Natural'Max (1, N) - 1);
      Trials : constant := 500;   --  抽 500 次(次数)
      function Med_Of (Pp, Nn : V3) return Long_Float is
      begin
         for I in 0 .. N - 1 loop
            Res (I) := Dot (Sub (X (X'First + I), Pp), Nn);
         end loop;
         return Median_Abs (Res (0 .. N - 1));
      end Med_Of;
   begin
      P0 := [0.0, 0.0, 0.0]; Nrm := [0.0, 0.0, 1.0]; Inliers := 0; Med := 0.0;
      if N < 3 then
         return;
      end if;
      for Tr_I in 1 .. Trials loop
         declare
            I1, I2, I3 : Natural;
         begin
            Next (Seed, N, I1); Next (Seed, N, I2); Next (Seed, N, I3);
            if I1 /= I2 and then I2 /= I3 and then I1 /= I3 then
               declare
                  Nn : constant V3 := Cross (Sub (X (X'First + I2), X (X'First + I1)), Sub (X (X'First + I3), X (X'First + I1)));
               begin
                  if Norm (Nn) > 0.0 then
                     declare
                        Nu : constant V3 := Unit (Nn);
                        Md : constant Long_Float := Med_Of (X (X'First + I1), Nu);
                     begin
                        if Md < Best_Med then
                           Best_Med := Md; P0 := X (X'First + I1); Nrm := Nu;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
      --  内点(门 = 2.5 × Mad_Sigma × 中位,统计常数,无量纲)按最小二乘重拟合:中心 + 协方差最小特征向量(用 4×4 那个求解器:补一行一列 0)。
      --  散布按门里的点重估、再挑再拟合,两遍(09-27 V1B32:格点铺满全画幅以后桌面点只占五成多,全体点的中位 = 2.6 mm,真桌面点只散 0.5 mm ——
      --  最小中位数的中位在内点不到一半多时量的是门,不是面;交出去的 Med = 门里的点离面的中位)
      declare
         Sig : Long_Float := Mad_Sigma * Best_Med;   --  正态下中位换标准差(统计换算)
         Cnt : Natural := 0;
         Ext : Long_Float;   --  这团点多大(各点离起步那一点的距离的中位):散布的下限按它的 1e-9 算,只防精确共面的点把门算成 0
      begin
         declare
            D : Vec (0 .. Natural'Max (1, N) - 1);
         begin
            for I in 0 .. N - 1 loop
               D (I) := Norm (Sub (X (X'First + I), P0));
            end loop;
            Ext := Median_Abs (D (0 .. N - 1));
         end;
         Sig := Long_Float'Max (Sig, 1.0e-9 * Ext);   --  数值保护(比例,无量纲)
         for Pass in 1 .. 2 loop   --  两遍(次数)
            declare
               Gate : constant Long_Float := 2.5 * Sig;   --  2.5 倍标准差(统计常数,无量纲)
               Ctr : V3 := [0.0, 0.0, 0.0];
               Nin : Natural := 0;
            begin
               Med := Med_Of (P0, Nrm);   --  顺带把每个点的残差算进 Res
               Cnt := 0;
               for I in 0 .. N - 1 loop
                  if abs Res (I) <= Gate then
                     Cnt := Cnt + 1; Ctr := Add (Ctr, X (X'First + I));
                  end if;
               end loop;
               exit when Cnt < 3;
               Ctr := Scl (Ctr, 1.0 / Long_Float (Cnt));
               declare
                  Cv : Mat (0 .. 3, 0 .. 3) := [others => [others => 0.0]];
               begin
                  for I in 0 .. N - 1 loop
                     if abs Res (I) <= Gate then
                        declare
                           D : constant V3 := Sub (X (X'First + I), Ctr);
                        begin
                           for Rr in 0 .. 2 loop
                              for Cc in 0 .. 2 loop
                                 Cv (Rr, Cc) := Cv (Rr, Cc) - D (Rr) * D (Cc);   --  取负:最大特征值 = 原来最小的那个
                              end loop;
                           end loop;
                        end;
                     end if;
                  end loop;
                  Cv (3, 3) := -1.0e300;   --  第四维不许被选中(无量纲)
                  declare
                     E : constant Vec := Max_Eigvec4 (Cv);
                     Nn : constant V3 := [E (0), E (1), E (2)];
                  begin
                     if Norm (Nn) > 0.0 then
                        Nrm := Unit (Nn);
                        P0 := Ctr;
                     end if;
                  end;
               end;
               --  按新的面、同一个门里的点重估散布
               declare
                  Ins : Vec (0 .. Natural'Max (1, N) - 1);
               begin
                  Med := Med_Of (P0, Nrm);
                  for I in 0 .. N - 1 loop
                     if abs Res (I) <= Gate then
                        Ins (Nin) := Res (I); Nin := Nin + 1;
                     end if;
                  end loop;
                  --  散布要比面自己的 3 个数多出点来才量得出:正好 3 个点拟出的面残差恒为 0(09-30 以前 3 个点也量,散布成了 0)
                  if Nin > 3 then
                     Sig := Long_Float'Max (Mad_Sigma * Median_Abs (Ins (0 .. Nin - 1)), 1.0e-9 * Ext);   --  中位换标准差(统计换算);下限同上(比例,无量纲)
                  end if;
               end;
            end;
         end loop;
         --  交出去:门里几个点、门里的点离面的中位
         declare
            Gate : constant Long_Float := 2.5 * Sig;   --  同上的门(统计常数,无量纲)
            Ins : Vec (0 .. Natural'Max (1, N) - 1);
            Nin : Natural := 0;
         begin
            Med := Med_Of (P0, Nrm);
            for I in 0 .. N - 1 loop
               if abs Res (I) <= Gate then
                  Ins (Nin) := Res (I); Nin := Nin + 1;
               end if;
            end loop;
            Inliers := Nin;
            Med := (if Nin > 0 then Median_Abs (Ins (0 .. Nin - 1)) else Med);
         end;
      end;
   end Robust_Plane;

   function To_Pose (R : M3; T : V3) return Plug.Arm_Pose is
      --  Shepperd:按迹和对角线里最大的那一项取根,数值最稳
      Tr0 : constant Long_Float := R (0, 0) + R (1, 1) + R (2, 2);
      W, X, Y, Z : Long_Float;
   begin
      if Tr0 > 0.0 then
         declare
            S : constant Long_Float := 2.0 * Sqrt (1.0 + Tr0);
         begin
            W := 0.25 * S; X := (R (2, 1) - R (1, 2)) / S; Y := (R (0, 2) - R (2, 0)) / S; Z := (R (1, 0) - R (0, 1)) / S;
         end;
      elsif R (0, 0) > R (1, 1) and then R (0, 0) > R (2, 2) then
         declare
            S : constant Long_Float := 2.0 * Sqrt (Long_Float'Max (0.0, 1.0 + R (0, 0) - R (1, 1) - R (2, 2)));
         begin
            W := (R (2, 1) - R (1, 2)) / S; X := 0.25 * S; Y := (R (0, 1) + R (1, 0)) / S; Z := (R (0, 2) + R (2, 0)) / S;
         end;
      elsif R (1, 1) > R (2, 2) then
         declare
            S : constant Long_Float := 2.0 * Sqrt (Long_Float'Max (0.0, 1.0 + R (1, 1) - R (0, 0) - R (2, 2)));
         begin
            W := (R (0, 2) - R (2, 0)) / S; X := (R (0, 1) + R (1, 0)) / S; Y := 0.25 * S; Z := (R (1, 2) + R (2, 1)) / S;
         end;
      else
         declare
            S : constant Long_Float := 2.0 * Sqrt (Long_Float'Max (0.0, 1.0 + R (2, 2) - R (0, 0) - R (1, 1)));
         begin
            W := (R (1, 0) - R (0, 1)) / S; X := (R (0, 2) + R (2, 0)) / S; Y := (R (1, 2) + R (2, 1)) / S; Z := 0.25 * S;
         end;
      end if;
      if W < 0.0 then
         W := -W; X := -X; Y := -Y; Z := -Z;
      end if;
      return [T (0), T (1), T (2), W, X, Y, Z];
   end To_Pose;

   procedure IK (M : Model; Rt : M3; Tt : V3; Q_Start : Floats; Lo, Hi : Floats; Q : out Floats; Pos_Err, Rot_Err : out Long_Float) is
      N : constant Natural := M.N;
      Lam : Long_Float := 1.0e-3;    --  阻尼(无量纲)
      Up : constant := 10.0;         --  阻尼放大倍数(次数)
      Dn : constant := 3.0;          --  阻尼缩小倍数(次数)
      H : constant := 1.0e-6;        --  差分步(弧度,极小量;无量纲)
      procedure Res (Qq : Floats; R : out Vec) is
         Rr : M3;
         Tq : V3;
      begin
         FK (M, Qq, Rr, Tq);
         declare
            E : constant V3 := Rot_Vec (Mul (Tr (Rr), Rt));
         begin
            for K in 0 .. 2 loop
               R (K) := Tq (K) - Tt (K);
               R (3 + K) := -E (K);
            end loop;
         end;
      end Res;
      function Clamp (Qq : Floats) return Floats is
         Out_Q : Floats := Qq;
      begin
         if Natural (Lo.Length) >= N and then Natural (Hi.Length) >= N then
            for J in 0 .. N - 1 loop
               Out_Q.Replace_Element (J, Long_Float'Max (Lo (J), Long_Float'Min (Hi (J), Qq (J))));
            end loop;
         end if;
         return Out_Q;
      end Clamp;
      function Sq (R : Vec) return Long_Float is
         S : Long_Float := 0.0;
      begin
         for X of R loop
            S := S + X * X;
         end loop;
         return S;
      end Sq;
      R0 : Vec (0 .. 5);
      C0 : Long_Float;
   begin
      Q := Clamp (Q_Start);
      Res (Q, R0);
      C0 := Sq (R0);
      for It in 1 .. 200 loop   --  最多 200 步(次数)
         exit when C0 < 1.0e-20;   --  到了(数值,无量纲)
         declare
            Jc : Mat (0 .. 5, 0 .. N - 1);
            Improved : Boolean := False;
         begin
            for J in 0 .. N - 1 loop
               declare
                  Qp : Floats := Q;
                  Rp : Vec (0 .. 5);
               begin
                  Qp.Replace_Element (J, Q (J) + H);
                  Res (Qp, Rp);
                  for K in 0 .. 5 loop
                     Jc (K, J) := (Rp (K) - R0 (K)) / H;
                  end loop;
               end;
            end loop;
            for Try in 1 .. 12 loop   --  一步里最多调 12 次阻尼(次数)
               declare
                  A : Mat (0 .. N - 1, 0 .. N - 1) := [others => [others => 0.0]];
                  B : Vec (0 .. N - 1) := [others => 0.0];
                  D : Vec (0 .. N - 1) := [others => 0.0];
                  Qn : Floats;
                  Rn : Vec (0 .. 5);
                  Cn : Long_Float;
               begin
                  for I in 0 .. N - 1 loop
                     for J in 0 .. N - 1 loop
                        for K in 0 .. 5 loop
                           A (I, J) := A (I, J) + Jc (K, I) * Jc (K, J);
                        end loop;
                     end loop;
                     for K in 0 .. 5 loop
                        B (I) := B (I) - Jc (K, I) * R0 (K);
                     end loop;
                     A (I, I) := A (I, I) * (1.0 + Lam) + Lam;
                  end loop;
                  --  高斯消元(列主元)
                  for Col in 0 .. N - 1 loop
                     declare
                        Pv : Natural := Col;
                     begin
                        for Rw in Col + 1 .. N - 1 loop
                           if abs A (Rw, Col) > abs A (Pv, Col) then
                              Pv := Rw;
                           end if;
                        end loop;
                        if Pv /= Col then
                           for Cc in 0 .. N - 1 loop
                              declare
                                 T : constant Long_Float := A (Col, Cc);
                              begin
                                 A (Col, Cc) := A (Pv, Cc); A (Pv, Cc) := T;
                              end;
                           end loop;
                           declare
                              T : constant Long_Float := B (Col);
                           begin
                              B (Col) := B (Pv); B (Pv) := T;
                           end;
                        end if;
                        if abs A (Col, Col) > 1.0e-300 then   --  主元为零保护(数值,无量纲)
                           for Rw in Col + 1 .. N - 1 loop
                              declare
                                 Fct : constant Long_Float := A (Rw, Col) / A (Col, Col);
                              begin
                                 for Cc in Col .. N - 1 loop
                                    A (Rw, Cc) := A (Rw, Cc) - Fct * A (Col, Cc);
                                 end loop;
                                 B (Rw) := B (Rw) - Fct * B (Col);
                              end;
                           end loop;
                        end if;
                     end;
                  end loop;
                  for K in reverse 0 .. N - 1 loop
                     declare
                        S : Long_Float := B (K);
                     begin
                        for Cc in K + 1 .. N - 1 loop
                           S := S - A (K, Cc) * D (Cc);
                        end loop;
                        D (K) := (if abs A (K, K) > 1.0e-300 then S / A (K, K) else 0.0);   --  同上(数值,无量纲)
                     end;
                  end loop;
                  Qn := Q;
                  for J in 0 .. N - 1 loop
                     Qn.Replace_Element (J, Q (J) + D (J));
                  end loop;
                  Qn := Clamp (Qn);
                  Res (Qn, Rn);
                  Cn := Sq (Rn);
                  if Cn < C0 then
                     Q := Qn; R0 := Rn;
                     Improved := C0 - Cn > 1.0e-15 * C0;   --  还在降(比例)
                     C0 := Cn;
                     Lam := Long_Float'Max (1.0e-9, Lam / Dn);
                     exit;
                  else
                     Lam := Lam * Up;
                  end if;
               end;
            end loop;
            exit when not Improved;
         end;
      end loop;
      Pos_Err := Sqrt (R0 (0) ** 2 + R0 (1) ** 2 + R0 (2) ** 2);
      Rot_Err := Sqrt (R0 (3) ** 2 + R0 (4) ** 2 + R0 (5) ** 2);
   end IK;
end Kinem;
