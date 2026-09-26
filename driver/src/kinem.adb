with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Calendar;
package body Kinem is

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
   type Vec is array (Natural range <>) of Long_Float;
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
            Th : constant Long_Float := (if I < Natural (Q.Length) and then I < Natural (M.Q0.Length) then Q (I) - M.Q0 (I) else 0.0);
            Ri : constant M3 := Rot (M.Ax (I).W, Th);
            Ti : constant V3 := Sub (M.Ax (I).P, Ap (Ri, M.Ax (I).P));
         begin
            T := Add (T, Ap (R, Ti));
            R := Mul (R, Ri);
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

   --  每一对最多留 K 个配点(配点按对挨着放)
   function Thin (Cs : Corr_Vectors.Vector; K : Natural) return Corr_Vectors.Vector is
      Out_Cs : Corr_Vectors.Vector;
      Li, Lj : Natural := Natural'Last;
      Cnt : Natural := 0;
   begin
      for C of Cs loop
         if C.I /= Li or else C.J /= Lj then
            Li := C.I; Lj := C.J; Cnt := 0;
         end if;
         if Cnt < K then
            Out_Cs.Append (C);
            Cnt := Cnt + 1;
         end if;
      end loop;
      return Out_Cs;
   end Thin;

   function Residual (M : Model; Frames : Frame_Vectors.Vector; C : Corr) return Long_Float is
      Ri, Rj, Rij : M3;
      Ti, Tj, Tij : V3;
   begin
      FK (M, Frames (C.I).Q, Ri, Ti);
      FK (M, Frames (C.J).Q, Rj, Tj);
      Rel (Ri, Ti, Rj, Tj, Rij, Tij);
      return Samp (Rij, Tij, M.F, M.Cx, M.Cy, C);
   end Residual;

   --  ── 抗野点的 LM(数值雅可比;Huber 1 px 迭代加权)──
   --  Resid 把全部残差填进 R(长度 N_R);只有前 N_Rob 个按 Huber 加权(配点),后面的(约束行)原样。
   --  阻尼升降的两个倍数只管这次拟合怎么迭代,不影响身体动不动
   procedure Robust_LM (X : in out Vec; N_R, N_Rob : Natural; Iters : Positive; Step : Vec;
                        Resid : not null access procedure (X : Vec; R : out Vec)) is
      Np : constant Natural := X'Length;
      R0 : Vec_Ptr := new Vec (0 .. N_R - 1);
      Rp : Vec_Ptr := new Vec (0 .. N_R - 1);
      Jc : Mat_Ptr := new Mat (0 .. N_R - 1, 0 .. Np - 1);
      Wt : Vec_Ptr := new Vec (0 .. N_R - 1);
      Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
      Up : constant := 10.0;        --  阻尼放大倍数(次数)
      Dn : constant := 3.0;         --  阻尼缩小倍数(次数)
      function Cost (R : Vec) return Long_Float is
         S : Long_Float := 0.0;
      begin
         for I in R'Range loop
            if I < N_Rob then
               S := S + (if abs R (I) <= 1.0 then 0.5 * R (I) ** 2 else abs R (I) - 0.5);   --  Huber,门 1 像素(协议:配点残差按像素记)
            else
               S := S + 0.5 * R (I) ** 2;
            end if;
         end loop;
         return S;
      end Cost;
      C0 : Long_Float;
   begin
      Resid (X, R0.all);
      C0 := Cost (R0.all);
      for It in 1 .. Iters loop
         for I in 0 .. N_R - 1 loop
            Wt (I) := (if I < N_Rob and then abs R0 (I) > 1.0 then 1.0 / abs R0 (I) else 1.0);
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
            for Try in 1 .. 8 loop   --  一轮里最多调 8 次阻尼(次数)
               declare
                  Aa : Mat := A;
                  Bb : Vec := B;
                  D : Vec (0 .. Np - 1) := [others => 0.0];
                  Xn : Vec := X;
                  Rn : Vec_Ptr := new Vec (0 .. N_R - 1);
                  Cn : Long_Float;
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
                  end loop;
                  Resid (Xn, Rn.all);
                  Cn := Cost (Rn.all);
                  if Cn < C0 then
                     X := Xn; R0.all := Rn.all;
                     Improved := abs (C0 - Cn) > 1.0e-9 * Long_Float'Max (C0, 1.0e-30);
                     C0 := Cn;
                     Lam := Long_Float'Max (1.0e-9, Lam / Dn);
                     Free (Rn);
                     exit;
                  else
                     Lam := Lam * Up;
                     Free (Rn);
                  end if;
               end;
            end loop;
            exit when not Improved;
         end;
      end loop;
      Free (R0); Free (Rp); Free (Jc); Free (Wt);
   end Robust_LM;

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
   --  g = (I − R)ᵀ((R h1) × h2);p 只在垂直于轴的平面里 ⇒ 2×2 的最小特征向量;按 Sampson 换算加权、Huber 1 px 重解一遍
   procedure Best_Phi (Cs : Jc_Array; N : Natural; W : V3; F, Cx, Cy : Long_Float; Phi : out Long_Float; Med : out Long_Float;
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
                  --  下一遍的权:Huber(1 px)× 代数残差换成像素的比例的平方
                  Wt (K) := (if abs R <= 1.0 then 1.0 else 1.0 / abs R) * (F / Den) ** 2;
               end;
            end loop;
         end;
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

   type Cand is record
      Score : Long_Float := 1.0e18;   --  空位(比所有真分数都大;无量纲)
      W : V3 := [0.0, 0.0, 1.0];
      Phi : Long_Float := 0.0;
   end record;
   Keep : constant := 20;   --  每档焦距留几个候选(次数)
   type Cand_Array is array (0 .. Keep - 1) of Cand;
   N_F : constant := 25;    --  焦距档数(次数):视场 30°–110°,每档约 7%
   type Cand_Table is array (0 .. N_F - 1) of Cand_Array;

   --  候选插进一档:按分数排好,彼此差不到 10° 的只留好的那个(同一个坑只留一个)
   procedure Insert (T : in out Cand_Array; C : Cand) is
      Same : constant := 0.984807753012208;   --  cos 10°(协议:同一个坑的宽度)
   begin
      for I in T'Range loop
         if Dot (T (I).W, C.W) > Same then
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
   Per_Pair_All : constant := 200; --  精修 / 定比例时每一对最多取几个配点(次数;最后一起解用全部内点)

   procedure Fit (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                  M : out Model; Rep : out Fit_Report; Ok : out Boolean) is
      Q0 : constant Floats := Frames (Ref).Q;
      N : constant Natural := Natural'Min (Max_Joints, Natural (Q0.Length));
      Nf : constant Natural := Natural (Frames.Length);
      Fg : array (0 .. N_F - 1) of Long_Float;
      Tab : array (0 .. Max_Joints - 1) of Cand_Table;
      Js : array (0 .. Max_Joints - 1) of Jc_Vectors.Vector;     --  每根轴起步用的配点(全部)
      Jg : array (0 .. Max_Joints - 1) of Jc_Vectors.Vector;     --  网格用的(转角小、每对抽样)
      Usable : array (0 .. Max_Joints - 1) of Boolean := [others => False];
      Dmax : Long_Float;
      function Dq (Fr, J : Natural) return Long_Float is (Frames (Fr).Q (J) - Q0 (J));
      --  这一帧能不能给第 J 根轴起步用:它是扫 J 扫出来的(或参照帧),别的关节偏得让画面挪不到 1 像素(按最长那档焦距算,最严)
      function Clean (Fr, J : Natural) return Boolean is
      begin
         if Fr = Ref then
            return True;
         end if;
         if Frames (Fr).Joint /= Integer (J) then
            return False;
         end if;
         for K in 0 .. N - 1 loop
            if K /= J and then abs Dq (Fr, K) >= Dmax then
               return False;
            end if;
         end loop;
         return True;
      end Clean;
      T_Mark : Ada.Calendar.Time := Ada.Calendar.Clock;
      procedure Lap is
         use type Ada.Calendar.Time;
         Now : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         Rep.Secs.Append (Long_Float (Now - T_Mark));
         T_Mark := Now;
      end Lap;
      Wj : array (0 .. Max_Joints - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Pj : array (0 .. Max_Joints - 1) of V3 := [others => [1.0, 0.0, 0.0]];
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
      Dmax := 1.0 / Fg (Fg'Last);
      --  配点分到各根轴
      declare
         Cnt : array (0 .. Max_Joints - 1) of Natural := [others => 0];
         Cnt_All : array (0 .. Max_Joints - 1) of Natural := [others => 0];
         Last_Pair : array (0 .. Max_Joints - 1) of Natural := [others => Natural'Last];
      begin
         for Ci in 0 .. Natural (Cs.Length) - 1 loop
            declare
               C : constant Corr := Cs (Ci);
               Key : constant Natural := C.I * Nf + C.J;
            begin
               for J in 0 .. N - 1 loop
                  if Clean (C.I, J) and then Clean (C.J, J) and then (Frames (C.I).Joint = Integer (J) or else Frames (C.J).Joint = Integer (J)) then
                     declare
                        R : constant Jc_Rec := (Ta => Dq (C.I, J), Tb => Dq (C.J, J), C => C);
                     begin
                        if Last_Pair (J) /= Key then
                           Last_Pair (J) := Key; Cnt (J) := 0; Cnt_All (J) := 0;
                        end if;
                        if Cnt_All (J) < Per_Pair_All then
                           Js (J).Append (R);
                           Cnt_All (J) := Cnt_All (J) + 1;
                        end if;
                        if abs (R.Ta - R.Tb) <= Grid_Rad and then Cnt (J) < Per_Pair_Grid then
                           Jg (J).Append (R);
                           Cnt (J) := Cnt (J) + 1;
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
            Usable (J) := Nfr >= 2 and then Natural (Jg (J).Length) >= 200;   --  至少两格、200 个配点才铺网格(次数)
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
               Wk := New_Work (Natural (Jg (Jj).Length));
               Ga := To_Array (Jg (Jj));
               for Ks in 0 .. N_Sph - 1 loop
                  declare
                     W : constant V3 := Sphere (Ks, N_Sph);
                  begin
                     for Kf in Fg'Range loop
                        declare
                           Ph, Md : Long_Float;
                        begin
                           Best_Phi (Ga.all, Natural (Jg (Jj).Length), W, Fg (Kf), Cx, Cy, Ph, Md, Wk);
                           Insert (Tab (Jj) (Kf), (Score => Md, W => W, Phi => Ph));
                        end;
                     end loop;
                  end;
               end loop;
               Free_Work (Wk);
               Free (Ga);
            end if;
         end Grid_Task;
         Workers : array (0 .. N - 1) of Grid_Task;
      begin
         for J in 0 .. N - 1 loop
            Workers (J).Start (J);
         end loop;
      end;   --  这里等全部线程做完
      Lap;
      --  焦距各轴共用:每档各轴最好的分数加起来,取最小的那档
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
                     S := S + Tab (J) (Kf) (0).Score;
                  end if;
               end loop;
               if S < Best then
                  Best := S; Kb := Kf;
               end if;
            end;
         end loop;
         F0 := Fg (Kb);
         Rep.F_Start := F0;
         --  焦距固定,每根轴从这一档的候选各自就地精修(先只用转角小的,再全部),按全部配点的残差中位挑
         for J in 0 .. N - 1 loop
            if not Usable (J) then
               Rep.Joint_Med.Append (-1.0);
            else
               declare
                  Bm : Long_Float := Long_Float'Last;
                  Bx : Vec (0 .. 2) := [0.0, 0.0, 0.0];
                  Ga : Jc_Array_Ptr := To_Array (Jg (J));
                  Sa : Jc_Array_Ptr := To_Array (Js (J));
               begin
                  for Ci in 0 .. Keep - 1 loop
                     declare
                        Cd : constant Cand := Tab (J) (Kb) (Ci);
                        X : Vec (0 .. 2) := [Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Cd.W (2)))), Arctan (Cd.W (1), Cd.W (0)), Cd.Phi];
                        Steps : constant Vec (0 .. 2) := [1.0e-6, 1.0e-6, 1.0e-6];   --  差分步(弧度,极小量)
                        procedure R_Small (Xx : Vec; R : out Vec) is
                        begin
                           Joint_Res (Ga.all, Natural (Jg (J).Length), Xx, F0, Cx, Cy, R);
                        end R_Small;
                        procedure R_All (Xx : Vec; R : out Vec) is
                        begin
                           Joint_Res (Sa.all, Natural (Js (J).Length), Xx, F0, Cx, Cy, R);
                        end R_All;
                     begin
                        exit when Cd.Score >= 1.0e17;   --  空位,后面没有候选了(无量纲)
                        Robust_LM (X, Natural (Jg (J).Length), Natural (Jg (J).Length), 60, Steps, R_Small'Access);
                        Robust_LM (X, Natural (Js (J).Length), Natural (Js (J).Length), 60, Steps, R_All'Access);
                        declare
                           R : Vec_Ptr := new Vec (0 .. Natural (Js (J).Length) - 1);
                           Md : Long_Float;
                        begin
                           R_All (X, R.all);
                           Md := Median_Abs (R.all);
                           Free (R);
                           if Md < Bm then
                              Bm := Md; Bx := X;
                           end if;
                        end;
                     end;
                  end loop;
                  Free (Ga); Free (Sa);
                  Rep.Joint_Med.Append (Bm);
                  Wj (J) := Ang_W (Bx (0), Bx (1));
                  declare
                     E1, E2 : V3;
                  begin
                     Perp (Wj (J), E1, E2);
                     Pj (J) := Add (Scl (E1, Cos (Bx (2))), Scl (E2, Sin (Bx (2))));
                  end;
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
      --  ② 各轴离眼远近的比例 ρ(可正可负:Sampson 分不出轴在眼的这边还是那边)
      declare
         Cs_Thin : constant Corr_Vectors.Vector := Thin (Cs, Per_Pair_All);
         Cross_Cnt : array (0 .. Max_Joints - 1) of Natural := [others => 0];
         Rf : Natural := 0;
         Rho : array (0 .. Max_Joints - 1) of Long_Float := [others => 1.0];
         function Model_Of (Rh : Vec) return Model is
            Mm : Model := M;
         begin
            Mm.F := F0;
            for J in 0 .. N - 1 loop
               Mm.Ax (J) := (W => Wj (J), P => Scl (Pj (J), Rh (Rh'First + J)));
            end loop;
            return Mm;
         end Model_Of;
      begin
         for C of Cs loop
            declare
               A : constant Integer := Frames (C.I).Joint;
               B : constant Integer := Frames (C.J).Joint;
            begin
               if A >= 0 and then B >= 0 and then A /= B and then A < Integer (N) and then B < Integer (N) then
                  Cross_Cnt (Natural (A)) := Cross_Cnt (Natural (A)) + 1;
                  Cross_Cnt (Natural (B)) := Cross_Cnt (Natural (B)) + 1;
               end if;
            end;
         end loop;
         for J in 1 .. N - 1 loop
            if Cross_Cnt (J) > Cross_Cnt (Rf) then
               Rf := J;
            end if;
         end loop;
         Rep.Ref_Joint := Rf;
         --  顺着"哪两根轴的格子之间有配点"一根接一根定:从参照轴出发,每一根相对一根已经定好的轴,只用这两根轴的格子之间的配点,
         --  一维网格(±0.01…10 倍,对数 31 档)。跟定好的哪一根都没有配点的轴定不了 ⇒ 如实报(Ok = False),不拿初值往下算
         --  (自检 2026-09-26:四根轴跟参照轴没有配点,比例停在初值 1,一起精修时没东西管,跑到几十万倍)
         declare
            Done : array (0 .. Max_Joints - 1) of Boolean := [others => False];
            Progress : Boolean := True;
            function Link_Cs (A, B : Natural) return Corr_Vectors.Vector is
               Sub_Cs : Corr_Vectors.Vector;
            begin
               for C of Cs_Thin loop
                  if (Frames (C.I).Joint = Integer (A) and then Frames (C.J).Joint = Integer (B))
                    or else (Frames (C.I).Joint = Integer (B) and then Frames (C.J).Joint = Integer (A))
                  then
                     Sub_Cs.Append (C);
                  end if;
               end loop;
               return Sub_Cs;
            end Link_Cs;
         begin
            Done (Rf) := True;
            while Progress loop
               Progress := False;
               for J in 0 .. N - 1 loop
                  if not Done (J) then
                     --  和已经定好的哪一根配点最多,就相对它定
                     declare
                        Best_Link : Corr_Vectors.Vector;
                     begin
                        for I in 0 .. N - 1 loop
                           if Done (I) then
                              declare
                                 L : constant Corr_Vectors.Vector := Link_Cs (J, I);
                              begin
                                 if Natural (L.Length) > Natural (Best_Link.Length) then
                                    Best_Link := L;
                                 end if;
                              end;
                           end if;
                        end loop;
                        if Natural (Best_Link.Length) >= 30 then   --  至少 30 个配点才定比例(次数)
                           declare
                              Best : Long_Float := Long_Float'Last;
                              Bg : Long_Float := 1.0;
                           begin
                              for Sg in 0 .. 1 loop
                                 for K in 0 .. 30 loop
                                    declare
                                       G : constant Long_Float := (if Sg = 0 then -1.0 else 1.0) * 0.01 * Exp (Long_Float (K) / 30.0 * Log (1000.0));
                                       Rh : Vec (0 .. N - 1);
                                       Mm : Model;
                                       R : Vec_Ptr := new Vec (0 .. Natural (Best_Link.Length) - 1);
                                       Md : Long_Float;
                                    begin
                                       for I in 0 .. N - 1 loop
                                          Rh (I) := Rho (I);
                                       end loop;
                                       Rh (J) := G;
                                       Mm := Model_Of (Rh);
                                       declare
                                          Pc : Pose_Array (0 .. Nf - 1);
                                       begin
                                          All_Poses (Mm, Frames, Pc);
                                          for I in 0 .. Natural (Best_Link.Length) - 1 loop
                                             R (I) := Res_Cached (Mm, Pc, Best_Link (I));
                                          end loop;
                                       end;
                                       Md := Median_Abs (R.all);
                                       Free (R);
                                       if Md < Best then
                                          Best := Md; Bg := G;
                                       end if;
                                    end;
                                 end loop;
                              end loop;
                              Rho (J) := Bg;
                              Done (J) := True;
                              Progress := True;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
            end loop;
            for J in 0 .. N - 1 loop
               if not Done (J) then
                  Rep.Rho.Clear;
                  for I in 0 .. N - 1 loop
                     Rep.Rho.Append (if Done (I) then Rho (I) else 0.0);   --  0 = 这根轴跟别的轴的格子之间没有配点,比例定不了
                  end loop;
                  return;
               end if;
            end loop;
         end;
         --  全部一起精修 ρ(参照轴那个钉 1)
         declare
            Nx : constant Natural := N - 1;
            X : Vec (0 .. Nx - 1);
            Steps : constant Vec (0 .. Nx - 1) := [others => 1.0e-7];   --  差分步(比例的极小量,无量纲)
            Nc : constant Natural := Natural (Cs_Thin.Length);
            procedure R_Rho (Xx : Vec; R : out Vec) is
               Rh : Vec (0 .. N - 1);
               K : Natural := Xx'First;
               Mm : Model;
               Pc : Pose_Array (0 .. Nf - 1);
            begin
               for J in 0 .. N - 1 loop
                  if J = Rf then
                     Rh (J) := 1.0;
                  else
                     Rh (J) := Xx (K); K := K + 1;
                  end if;
               end loop;
               Mm := Model_Of (Rh);
               All_Poses (Mm, Frames, Pc);
               for I in 0 .. Nc - 1 loop
                  R (R'First + I) := Res_Cached (Mm, Pc, Cs_Thin (I));
               end loop;
            end R_Rho;
         begin
            declare
               K : Natural := 0;
            begin
               for J in 0 .. N - 1 loop
                  if J /= Rf then
                     X (K) := Rho (J); K := K + 1;
                  end if;
               end loop;
            end;
            if Nx > 0 and then Nc > 0 then
               Robust_LM (X, Nc, Nc, 60, Steps, R_Rho'Access);
            end if;
            declare
               K : Natural := 0;
            begin
               for J in 0 .. N - 1 loop
                  if J /= Rf then
                     Rho (J) := X (K); K := K + 1;
                  end if;
                  Rep.Rho.Append (Rho (J));
                  Pj (J) := Scl (Pj (J), Rho (J));
               end loop;
            end;
         end;
      end;
      Lap;
      --  ③ 挑内点(按起步模型:残差 < max(3 px, 3 倍中位)),全部一起按像素解(焦距放开;尺度钉"参与的各帧眼的位置均方根 = 1")
      declare
         M0 : Model := M;
         Inl : Corr_Vectors.Vector;
         Used_Frame : array (0 .. Nf - 1) of Boolean := [others => False];
      begin
         M0.F := F0;
         for J in 0 .. N - 1 loop
            M0.Ax (J) := (W => Wj (J), P => Pj (J));
         end loop;
         --  两轮:第一轮按起步模型挑内点、一起解;第二轮按第一轮解出的模型重挑内点再解(结果不靠起步准不准;自检:起步焦距差 5% 时考试中位 0.57 → 0.91 mm)
         for Round in 1 .. 2 loop
            Inl.Clear;
            Used_Frame := [others => False];
            declare
               R : Vec_Ptr := new Vec (0 .. Natural (Cs.Length) - 1);
               Gate : Long_Float;
            begin
               declare
                  Pc : Pose_Array (0 .. Nf - 1);
               begin
                  All_Poses (M0, Frames, Pc);
                  for I in 0 .. Natural (Cs.Length) - 1 loop
                     R (I) := Res_Cached (M0, Pc, Cs (I));
                  end loop;
               end;
               Gate := Long_Float'Max (3.0, 3.0 * Median_Abs (R.all));   --  3 px / 3 倍中位(协议:配点残差按像素记)
               for I in 0 .. Natural (Cs.Length) - 1 loop
                  if abs R (I) < Gate then
                     Inl.Append (Cs (I));
                     Used_Frame (Cs (I).I) := True; Used_Frame (Cs (I).J) := True;
                  end if;
               end loop;
               Free (R);
            end;
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
                  for J in 0 .. N - 1 loop
                     M0.Ax (J).P := Scl (M0.Ax (J).P, 1.0 / Sqrt (S2 / Long_Float (Cnt)));
                  end loop;
               end if;
            end;
            declare
               Np : constant Natural := 6 * N + 1;
               X : Vec (0 .. Np - 1);
               Steps : Vec (0 .. Np - 1);
               Ni : constant Natural := Natural (Inl.Length);
               N_Reg : constant Natural := 2 * N + 1;
               function To_Model (Xx : Vec) return Model is
                  Mm : Model := M0;
               begin
                  for J in 0 .. N - 1 loop
                     Mm.Ax (J).W := [Xx (Xx'First + 3 * J), Xx (Xx'First + 3 * J + 1), Xx (Xx'First + 3 * J + 2)];
                     Mm.Ax (J).P := [Xx (Xx'First + 3 * N + 3 * J), Xx (Xx'First + 3 * N + 3 * J + 1), Xx (Xx'First + 3 * N + 3 * J + 2)];
                  end loop;
                  Mm.F := Exp (Xx (Xx'First + 6 * N));
                  return Mm;
               end To_Model;
               procedure R_All (Xx : Vec; R : out Vec) is
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
                  --  约束行(不加权):轴是单位向量、P 取轴上离参照眼最近那点、尺度钉住(倍数只管数值,1e3 = 这几行比像素残差重得多,比例)
                  for J in 0 .. N - 1 loop
                     R (R'First + Ni + 2 * J) := 1.0e3 * (Norm (Mm.Ax (J).W) - 1.0);
                     R (R'First + Ni + 2 * J + 1) := 1.0e3 * Dot (Mm.Ax (J).W, Mm.Ax (J).P);
                  end loop;
                  R (R'First + Ni + 2 * N) := 1.0e3 * ((if Cnt > 0 then Sqrt (S2 / Long_Float (Cnt)) else 1.0) - 1.0);
               end R_All;
            begin
               for J in 0 .. N - 1 loop
                  for K in 0 .. 2 loop
                     X (3 * J + K) := M0.Ax (J).W (K);
                     X (3 * N + 3 * J + K) := M0.Ax (J).P (K) - Dot (M0.Ax (J).W, M0.Ax (J).P) * M0.Ax (J).W (K);
                  end loop;
               end loop;
               X (6 * N) := Log (M0.F);
               for K in Steps'Range loop
                  Steps (K) := 1.0e-7;   --  差分步(无量纲 / 模型单位 / 对数焦距,极小量)
               end loop;
               Robust_LM (X, Ni + N_Reg, Ni, 300, Steps, R_All'Access);
               M := To_Model (X);
               for J in 0 .. N - 1 loop
                  M.Ax (J).W := Unit (M.Ax (J).W);
               end loop;
               declare
                  R : Vec_Ptr := new Vec (0 .. Ni + N_Reg - 1);
               begin
                  R_All (X, R.all);
                  Rep.Med_Px := Median_Abs (R (0 .. Ni - 1));
                  Rep.P90_Px := Quantile_Abs (R (0 .. Ni - 1), 0.9);   --  九成分位(比例,只报数)
                  Free (R);
               end;
            end;
            M0 := M;
         end loop;
         Lap;
         Rep.F := M.F;
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
               for J in 0 .. N - 1 loop
                  M.Ax (J).P := Scl (M.Ax (J).P, -1.0);
               end loop;
               Rep.Flipped := True;
            end if;
         end;
         M.Valid := True;
         Ok := True;
      end;
   end Fit;
   procedure Meet_Rays (O, D : V3_Array; X : out V3; Ok : out Boolean) is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [0.0, 0.0, 0.0];
   begin
      Ok := False; X := [0.0, 0.0, 0.0];
      if O'Length < 2 then
         return;
      end if;
      for I in O'Range loop
         declare
            Dd : constant V3 := Unit (D (I));
         begin
            for R in 0 .. 2 loop
               for C in 0 .. 2 loop
                  declare
                     Pr : constant Long_Float := (if R = C then 1.0 else 0.0) - Dd (R) * Dd (C);   --  I − d dᵀ
                  begin
                     A (R, C) := A (R, C) + Pr;
                     B (R) := B (R) + Pr * O (I) (C);
                  end;
               end loop;
            end loop;
         end;
      end loop;
      X := Solve3 (A, B);
      Ok := Norm (X) > 0.0 or else Norm (B) = 0.0;
   end Meet_Rays;

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

   procedure Similarity (A, B : V3_Array; S : out Long_Float; R : out M3; T : out V3) is
      Ca, Cb : V3 := [0.0, 0.0, 0.0];
      N : constant Long_Float := Long_Float (A'Length);
      Sm : M3 := [others => [others => 0.0]];
      Saa, Sba : Long_Float := 0.0;
   begin
      S := 1.0; R := Identity; T := [0.0, 0.0, 0.0];
      if A'Length = 0 then
         return;
      end if;
      for I in A'Range loop
         Ca := Add (Ca, A (I)); Cb := Add (Cb, B (I - A'First + B'First));
      end loop;
      Ca := Scl (Ca, 1.0 / N); Cb := Scl (Cb, 1.0 / N);
      for I in A'Range loop
         declare
            Pa : constant V3 := Sub (A (I), Ca);
            Pb : constant V3 := Sub (B (I - A'First + B'First), Cb);
         begin
            for X in 0 .. 2 loop
               for Y in 0 .. 2 loop
                  Sm (X, Y) := Sm (X, Y) + Pa (X) * Pb (Y);
               end loop;
            end loop;
            Saa := Saa + Dot (Pa, Pa);
         end;
      end loop;
      declare
         Nm : constant Mat (0 .. 3, 0 .. 3) :=
           [[Sm (0, 0) + Sm (1, 1) + Sm (2, 2), Sm (1, 2) - Sm (2, 1), Sm (2, 0) - Sm (0, 2), Sm (0, 1) - Sm (1, 0)],
            [Sm (1, 2) - Sm (2, 1), Sm (0, 0) - Sm (1, 1) - Sm (2, 2), Sm (0, 1) + Sm (1, 0), Sm (2, 0) + Sm (0, 2)],
            [Sm (2, 0) - Sm (0, 2), Sm (0, 1) + Sm (1, 0), -Sm (0, 0) + Sm (1, 1) - Sm (2, 2), Sm (1, 2) + Sm (2, 1)],
            [Sm (0, 1) - Sm (1, 0), Sm (2, 0) + Sm (0, 2), Sm (1, 2) + Sm (2, 1), -Sm (0, 0) - Sm (1, 1) + Sm (2, 2)]];
         Q : constant Vec := Max_Eigvec4 (Nm);
      begin
         R := Quat_To_R ([0.0, 0.0, 0.0, Q (0), Q (1), Q (2), Q (3)]);
      end;
      for I in A'Range loop
         Sba := Sba + Dot (Sub (B (I - A'First + B'First), Cb), Ap (R, Sub (A (I), Ca)));
      end loop;
      S := (if Saa > 0.0 then Sba / Saa else 1.0);
      T := Sub (Cb, Scl (Ap (R, Ca), S));
   end Similarity;

   --  确定性的伪随机(同一份数据同一个结果):线性同余
   procedure Next (Seed : in out Unsigned_Seed; K : Natural; Out_I : out Natural) is
   begin
      Seed := Seed * 6364136223846793005 + 1442695040888963407;   --  线性同余的乘数 / 增量(Knuth MMIX,协议)
      Out_I := Natural ((Seed / 2 ** 33) mod Unsigned_Seed (K));
   end Next;

   procedure Robust_Similarity (A, B : V3_Array; S : out Long_Float; R : out M3; T : out V3; Inliers : out Natural; Med : out Long_Float) is
      N : constant Natural := A'Length;
      Seed : Unsigned_Seed := 20260926;
      Best_Med : Long_Float := Long_Float'Last;
      Res : Vec (0 .. Natural'Max (1, N) - 1);
      Trials : constant := 500;   --  抽 500 次(次数)
      function Med_Of (Ss : Long_Float; Rr : M3; Tt : V3) return Long_Float is
      begin
         for I in 0 .. N - 1 loop
            Res (I) := Norm (Sub (B (B'First + I), Add (Scl (Ap (Rr, A (A'First + I)), Ss), Tt)));
         end loop;
         return Median_Abs (Res (0 .. N - 1));
      end Med_Of;
   begin
      S := 1.0; R := Identity; T := [0.0, 0.0, 0.0]; Inliers := 0; Med := 0.0;
      if N < 3 then
         return;
      end if;
      for Tr_I in 1 .. Trials loop
         declare
            I1, I2, I3 : Natural;
            Ss : Long_Float;
            Rr : M3;
            Tt : V3;
         begin
            Next (Seed, N, I1); Next (Seed, N, I2); Next (Seed, N, I3);
            if I1 /= I2 and then I2 /= I3 and then I1 /= I3 then
               Similarity ([A (A'First + I1), A (A'First + I2), A (A'First + I3)], [B (B'First + I1), B (B'First + I2), B (B'First + I3)], Ss, Rr, Tt);
               declare
                  Md : constant Long_Float := Med_Of (Ss, Rr, Tt);
               begin
                  if Md < Best_Med then
                     Best_Med := Md; S := Ss; R := Rr; T := Tt;
                  end if;
               end;
            end if;
         end;
      end loop;
      --  拿"残差 < 2.5 × 1.4826 × 中位数"的那些重解(统计常数,无量纲,见规格说明)
      declare
         Gate : constant Long_Float := 2.5 * 1.4826 * Best_Med;
         Cnt : Natural := 0;
      begin
         Med := Med_Of (S, R, T);
         for I in 0 .. N - 1 loop
            if Res (I) <= Gate then
               Cnt := Cnt + 1;
            end if;
         end loop;
         if Cnt >= 3 then
            declare
               Ai, Bi : V3_Array (0 .. Cnt - 1);
               K : Natural := 0;
            begin
               for I in 0 .. N - 1 loop
                  if Res (I) <= Gate then
                     Ai (K) := A (A'First + I); Bi (K) := B (B'First + I); K := K + 1;
                  end if;
               end loop;
               Similarity (Ai, Bi, S, R, T);
            end;
         end if;
         Inliers := Cnt;
         Med := Med_Of (S, R, T);
      end;
   end Robust_Similarity;

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
      --  内点(同上的统计常数,无量纲)按最小二乘重拟合:中心 + 协方差最小特征向量(用 4×4 那个求解器:补一行一列 0)
      declare
         Gate : constant Long_Float := 2.5 * 1.4826 * Best_Med;
         Cnt : Natural := 0;
         Ctr : V3 := [0.0, 0.0, 0.0];
      begin
         Med := Med_Of (P0, Nrm);
         for I in 0 .. N - 1 loop
            if abs Res (I) <= Gate then
               Cnt := Cnt + 1; Ctr := Add (Ctr, X (X'First + I));
            end if;
         end loop;
         if Cnt >= 3 then
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
         end if;
         Inliers := Cnt;
         Med := Med_Of (P0, Nrm);
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
