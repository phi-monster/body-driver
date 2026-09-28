with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Calendar;
with Ada.Containers.Ordered_Maps;
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
         begin
            if M.Ax (I).Slide then
               T := Add (T, Ap (R, Scl (M.Ax (I).W, Th)));   --  沿 W 走 θ 个读数单位,不转
            else
               declare
                  Ri : constant M3 := Rot (M.Ax (I).W, Th);
                  Ti : constant V3 := Sub (M.Ax (I).P, Ap (Ri, M.Ax (I).P));
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
   type V12 is array (0 .. Max_Joints - 1) of Long_Float;
   type Nat_Array is array (Natural range <>) of Natural;
   --  一个配点:对极约束 g · ρ = 0,Sampson 的分母 = |(E0, E1, E2, E3) · ρ|(Ex1、Etx2 的前两个分量,同 Samp)
   type Rho_Row is record
      G, E0, E1, E2, E3 : V12 := [others => 0.0];
   end record;
   type Rho_Rows is array (Natural range <>) of Rho_Row;
   type Rho_Rows_Ptr is access Rho_Rows;
   procedure Free is new Ada.Unchecked_Deallocation (Rho_Rows, Rho_Rows_Ptr);
   function Rho_Res (R : Rho_Row; Rho : V12; N : Natural; F : Long_Float; Den : out Long_Float) return Long_Float is
      Num, D0, D1, D2, D3 : Long_Float := 0.0;
   begin
      for J in 0 .. N - 1 loop
         Num := Num + R.G (J) * Rho (J);
         D0 := D0 + R.E0 (J) * Rho (J);
         D1 := D1 + R.E1 (J) * Rho (J);
         D2 := D2 + R.E2 (J) * Rho (J);
         D3 := D3 + R.E3 (J) * Rho (J);
      end loop;
      Den := Sqrt (D0 * D0 + D1 * D1 + D2 * D2 + D3 * D3) + 1.0e-18;
      return F * Num / Den;
   end Rho_Res;
   --  对称阵(前 N × N)最小特征值的特征向量(循环 Jacobi,同 Max_Eigvec4 的转法)
   function Min_Eig (A0 : Mat; N : Natural) return V12 is
      A : Mat (0 .. N - 1, 0 .. N - 1);
      V : Mat (0 .. N - 1, 0 .. N - 1) := [others => [others => 0.0]];
      Best : Natural := 0;
      Out_V : V12 := [others => 0.0];
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
      S.Nt := Nt; S.No := No;
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
   --  给定各帧位姿(Pr, Pt),解每条轨迹的 log 远近(Lz,就地更新,Iters 遍高斯牛顿),回填每一笔的像素残差 (Ru, Rv)。
   --  Wa / Wb:每一笔在那一帧眼系里 X = Wb + λ Wa(工作区);G / H:每条轨迹一维的梯度 / 曲率(工作区;出来时是最后一遍的曲率,不加权)
   procedure Mv_Eval (S : Mv_Set; F, Cx, Cy : Long_Float; Pr : M3_Array; Pt : V3_Array; Lz : in out Vec; Iters : Natural; Tau : Long_Float;
                      Wa, Wb : V3_Ptr; Ru, Rv, G, H, Lt : Vec_Ptr) is
      --  Lt:每条轨迹这一遍的 λ(工作区,一条只算一次 exp)
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
      for It in 0 .. Iters loop   --  最后一遍只算残差和曲率
         for T in 0 .. S.Nt - 1 loop
            G (T) := 0.0; H (T) := 0.0;
            Lt (T) := (if S.Live (T) = 1 then Exp (Lz (Lz'First + T)) else 0.0);
         end loop;
         for N in 0 .. S.No - 1 loop
            declare
               T : constant Natural := S.Ot (N);
            begin
               if S.Live (T) = 1 then
                  declare
                     Lam : constant Long_Float := Lt (T);
                     X : constant V3 := Add (Wb (N), Scl (Wa (N), Lam));
                     Zp : constant Long_Float := -X (2);
                  begin
                     if Zp > 1.0e-9 then   --  在那一帧眼前面(数值,无量纲)
                        Ru (N) := F * X (0) / Zp + Cx - S.Ou (N);
                        Rv (N) := -F * X (1) / Zp + Cy - S.Ov (N);
                        if S.Obs_In (N) = 1 then   --  门外的笔不进这一维的解(残差照算,重挑内点要用)
                           declare
                              Dx : constant V3 := Scl (Wa (N), Lam);   --  ∂X/∂log λ
                              Dzp : constant Long_Float := -Dx (2);
                              Du : constant Long_Float := F * (Dx (0) * Zp - X (0) * Dzp) / (Zp * Zp);
                              Dv : constant Long_Float := -F * (Dx (1) * Zp - X (1) * Dzp) / (Zp * Zp);
                              W : constant Long_Float := (if It < Iters then 1.0 / Sqrt (1.0 + (Ru (N) ** 2 + Rv (N) ** 2) / (Tau * Tau)) else 1.0);
                           begin
                              G (T) := G (T) + W * (Du * Ru (N) + Dv * Rv (N));
                              H (T) := H (T) + W * (Du * Du + Dv * Dv);
                           end;
                        end if;
                     else
                        Ru (N) := Behind_Px; Rv (N) := 0.0;
                     end if;
                  end;
               else
                  Ru (N) := 0.0; Rv (N) := 0.0;
               end if;
            end;
         end loop;
         if It < Iters then
            for T in 0 .. S.Nt - 1 loop
               if S.Live (T) = 1 and then H (T) > 1.0e-18 then   --  数值保护(无量纲)
                  --  一步最多挪 log λ ±0.5(远近一步最多差 1.65 倍;高斯牛顿不越过坑,无量纲)
                  Lz (Lz'First + T) := Lz (Lz'First + T) + Long_Float'Max (-0.5, Long_Float'Min (0.5, -G (T) / H (T)));
               end if;
            end loop;
         end if;
      end loop;
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
      Np : constant Natural := N_Params (M);
      Nt_Ax : constant Natural := N_Turn (M);
      S : Mv_Set;
      Used : array (0 .. Natural'Max (1, Nf) - 1) of Boolean := [others => False];
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      Build_Mv (Cs, -1, S);
      if S.Nt = 0 or else 2 * S.No <= Np then
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
         Nr : constant Natural := 2 * S.No + 2 * Nt_Ax + 1;
         Wa, Wb : V3_Ptr := new V3_Array (0 .. S.No - 1);
         Ru, Rv, Ru2, Rv2 : Vec_Ptr := new Vec (0 .. S.No - 1);
         G, H, Lt : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Sw : Vec_Ptr := new Vec (0 .. S.No - 1);   --  外层 IRLS 的 √权(按当前残差定,求导时不动)
         Lz, Lz2, Lz3 : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Jc : Mat_Ptr := new Mat (0 .. Nr - 1, 0 .. Np - 1);
         R0 : Vec_Ptr := new Vec (0 .. Nr - 1);
         X : Vec (0 .. Np - 1);
         Tau : Long_Float := 3.0;   --  起步的抗野点尺度 3 px(协议,同 ③ 挑内点的门);起步远近解完按中位重定
         Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
         C0 : Long_Float;
         Pr : M3_Array (0 .. Nf - 1);
         Pt : V3_Array (0 .. Nf - 1);
         function To_Model (Xx : Vec) return Model is (From_X (M, Xx));
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
         To_X (M, X);
         Poses (M);
         Mv_Init (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all);
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, 10, Tau, Wa, Wb, Ru, Rv, G, H, Lt);   --  起步远近解 10 遍(次数)
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
         --  两轮(同 ③):每轮先按当前的残差重挑内点(门外的笔不进解)、按门里的残差中位重定抗野点尺度,再解;第二轮按第一轮解出的模型重挑
         --  (合成焊点:5% 乱配时 soft-l1 对大残差还有恒定的拉力,几千条乱配一起把模型拉歪 24 mm —— 要先挑掉)
         Rep.Mv_Iters := 0;
         for Round in 1 .. 2 loop
            declare
               N_In : Natural;
            begin
               Poses (To_Model (X));
               Mv_Gate (S, Ru, Rv, N_In);
               Mv_Eval (S, Exp (X (Np - 1)), M.Cx, M.Cy, Pr, Pt, Lz.all, 5, Tau, Wa, Wb, Ru, Rv, G, H, Lt);   --  门里的重解远近 5 遍(次数)
               if Round = 1 then
                  Rep.Mv_Start_Px := Mv_Med (S, Ru, Rv);
               end if;
               Tau := Long_Float'Max (1.4826 * Mv_Med (S, Ru, Rv), 1.0e-3);   --  抗野点尺度 = 门里残差中位换标准差(1.4826 统计常数;下限只防零,无量纲)
               Mv_Eval (S, Exp (X (Np - 1)), M.Cx, M.Cy, Pr, Pt, Lz.all, 3, Tau, Wa, Wb, Ru, Rv, G, H, Lt);
               Rep.Mv_Obs := N_In;
               Rep.Mv_Tracks := 0;
               for T in 0 .. S.Nt - 1 loop
                  Rep.Mv_Tracks := Rep.Mv_Tracks + S.Live (T);
               end loop;
               Reg (To_Model (X), Rg0);
               Base;
               C0 := Cost (Ru, Rv, Rg0);
            for It in 1 .. Mv_Iters loop
               Rep.Mv_Iters := Rep.Mv_Iters + 1;
               --  数值雅可比:每个数挪一点,远近从当前解起再解 2 遍(次数),外层权不动。基准也从同一个远近起、同样解 2 遍再算:
               --  远近还没解到底时两边一起挪,差分里就没有它(09-27 龙门架:基准用上一步的远近、挪一点的那份多解 2 遍 ⇒ 远近自己还在收的那一点
               --  被除以 1e-6 当成导数,阻尼到了 1e-9 代价还降得很慢,多视图 800 轮才从焦距 419 爬到 410;真模型起步时远近是收好的,看不出来)
               Lz3.all := Lz.all;
               Poses (To_Model (X));
               Mv_Eval (S, Exp (X (Np - 1)), M.Cx, M.Cy, Pr, Pt, Lz.all, 2, Tau, Wa, Wb, Ru, Rv, G, H, Lt);
               Base;
               C0 := Cost (Ru, Rv, Rg0);
               for Jp in 0 .. Np - 1 loop
                  declare
                     Xp : Vec := X;
                     Hh : constant Long_Float := 1.0e-6 * Long_Float'Max (1.0, abs X (Jp));   --  差分步(相对 1e-6,无量纲)
                     Mm : Model;
                  begin
                     Xp (Jp) := Xp (Jp) + Hh;
                     Mm := To_Model (Xp);
                     Poses (Mm);
                     Lz2.all := Lz3.all;
                     Mv_Eval (S, Mm.F, Mm.Cx, Mm.Cy, Pr, Pt, Lz2.all, 2, Tau, Wa, Wb, Ru2, Rv2, G, H, Lt);
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
                  for Try in 1 .. 8 loop   --  一轮里最多调 8 次阻尼(次数)
                     declare
                        Aa : Mat := A;
                        Bb : Vec := B;
                        D : Vec (0 .. Np - 1) := [others => 0.0];
                        Xn : Vec := X;
                        Cn : Long_Float;
                        Mm : Model;
                        Rg : Vec (0 .. 2 * Nt_Ax);
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
                        end loop;
                        Mm := To_Model (Xn);
                        Poses (Mm);
                        Lz2.all := Lz.all;
                        Mv_Eval (S, Mm.F, Mm.Cx, Mm.Cy, Pr, Pt, Lz2.all, 4, Tau, Wa, Wb, Ru2, Rv2, G, H, Lt);   --  试的这一步远近再解 4 遍(次数)
                        Reg (Mm, Rg);
                        Cn := Cost (Ru2, Rv2, Rg);
                        if Cn < C0 then
                           Improved := (C0 - Cn) > 1.0e-7 * C0;   --  这一轮代价降得不到千万分之一就算到底了(比例)
                           X := Xn; Lz.all := Lz2.all; Ru.all := Ru2.all; Rv.all := Rv2.all; Rg0 := Rg;
                           Base;
                           C0 := Cn;
                           Lam := Long_Float'Max (1.0e-9, Lam / Mv_Dn);
                           exit;
                        else
                           Lam := Lam * Mv_Up;
                        end if;
                     end;
                  end loop;
                  exit when not Improved;
               end;
            end loop;
            end;
         end loop;
         M := To_Model (X);
         for J in 0 .. N - 1 loop
            if not M.Ax (J).Slide then
               M.Ax (J).W := Unit (M.Ax (J).W);
            end if;
         end loop;
         Rep.Mv_Px := Mv_Med (S, Ru, Rv);
         Rep.Mv_P90_Px := Mv_Med (S, Ru, Rv, 0.9);   --  九成分位(比例,只报数)
         Rep.F := M.F;
         Free (Wa); Free (Wb); Free (Ru); Free (Rv); Free (Ru2); Free (Rv2); Free (G); Free (H); Free (Lt); Free (Sw); Free (Lz); Free (Lz2); Free (Lz3); Free (Jc); Free (R0);
      end;
      Free_Mv (S);
      Rep.Secs.Append (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)));
   end Refine_Mv;

   procedure Refine_Tracks (Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; M : in out Model; Rep : in out Fit_Report) is
   begin
      Refine_Mv (Frames, Cs, M, Rep);
   end Refine_Tracks;

   procedure Track_Points (M : Model; Frames : Frame_Vectors.Vector; Cs : Corr_Vectors.Vector; Only_I : Integer; Min_Views : Natural;
                           Tracks : out Track_Pt_Vectors.Vector; Sig_Px : out Long_Float) is
      Nf : constant Natural := Natural (Frames.Length);
      S : Mv_Set;
   begin
      Tracks.Clear; Sig_Px := 0.0;
      Build_Mv (Cs, Only_I, S);
      if S.Nt = 0 then
         Free_Mv (S);
         return;
      end if;
      declare
         Wa, Wb : V3_Ptr := new V3_Array (0 .. S.No - 1);
         Ru, Rv : Vec_Ptr := new Vec (0 .. S.No - 1);
         G, H, Lt : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Lz : Vec_Ptr := new Vec (0 .. S.Nt - 1);
         Pr : M3_Array (0 .. Nf - 1);
         Pt : V3_Array (0 .. Nf - 1);
         Tau : Long_Float := 3.0;   --  起步的抗野点尺度 3 px(协议,同 ③ 挑内点的门)
         Sig : Long_Float;
      begin
         for Fr in 0 .. Nf - 1 loop
            FK (M, Frames (Fr).Q, Pr (Fr), Pt (Fr));
         end loop;
         Mv_Init (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all);
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, 10, Tau, Wa, Wb, Ru, Rv, G, H, Lt);   --  10 遍(次数)
         Sig := 1.4826 * Mv_Med (S, Ru, Rv);   --  正态下中位换标准差(统计常数)
         Tau := Long_Float'Max (Sig, 1.0e-3);   --  下限只防零(无量纲)
         Mv_Eval (S, M.F, M.Cx, M.Cy, Pr, Pt, Lz.all, 5, Tau, Wa, Wb, Ru, Rv, G, H, Lt);   --  按量到的尺度再解 5 遍(次数)
         Sig := 1.4826 * Mv_Med (S, Ru, Rv);
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
               if S.Live (T) = 1 and then not Tt (T).Behind and then Tt (T).N + 1 >= Min_Views and then H (T) > 1.0e-18 then   --  数值保护(无量纲)
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
         Free (Wa); Free (Wb); Free (Ru); Free (Rv); Free (G); Free (H); Free (Lt); Free (Lz);
      end;
      Free_Mv (S);
   end Track_Points;

   --  最长那档焦距 = 半幅宽 ÷ tan(15°)(焦距网格的上头,见 Fit;视场 30° 是协议:针孔相机的常见范围)
   function Clean_Tol (Width : Long_Float) return Long_Float is
     (Tan (0.261799387799490) / (0.5 * Width));

   procedure Fit (Frames : Frame_Vectors.Vector; Ref : Natural; Cs : Corr_Vectors.Vector; Cx, Cy, Width : Long_Float;
                  M : out Model; Rep : out Fit_Report; Ok : out Boolean; Per_Pair : Positive := 60) is
      Q0 : constant Floats := Frames (Ref).Q;
      N : constant Natural := Natural'Min (Max_Joints, Natural (Q0.Length));
      Nf : constant Natural := Natural (Frames.Length);
      Fg : array (0 .. N_F - 1) of Long_Float;
      Tab : array (0 .. Max_Joints - 1) of Cand_Table;
      Js : array (0 .. Max_Joints - 1) of Jc_Vectors.Vector;     --  每根轴起步用的配点(全部)
      Jg : array (0 .. Max_Joints - 1) of Jc_Vectors.Vector;     --  网格用的(转角小、每对抽样)
      Jgs : array (0 .. Max_Joints - 1) of Jc_Vectors.Vector;    --  "走"那样网格用的(每对抽样;不按转角挑:按"走"解时读数差不是转角)
      Usable : array (0 .. Max_Joints - 1) of Boolean := [others => False];
      Use_T, Use_S : array (0 .. Max_Joints - 1) of Boolean := [others => False];   --  按"转" / 按"走"试得了(至少两格、网格有 200 个配点)
      Slide_Mid : array (0 .. Max_Joints - 1) of Long_Float := [others => 1.0e18];   --  "走"那样网格最好的分数(焦距中间那档;跟焦距无关);没试 = 空位(无量纲)
      Sl : array (0 .. Max_Joints - 1) of Boolean := [others => False];             --  认成"走"
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
      Xj : array (0 .. Max_Joints - 1) of Vec (0 .. 2) := [others => [0.0, 0.0, 0.0]];   --  每根轴单独精修完的(两个方向角, φ)
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
         N_Pairs, N_Small : array (0 .. Max_Joints - 1) of Natural := [others => 0];
         K_Grid, K_All, K_Grid_S : array (0 .. Max_Joints - 1) of Natural := [others => 0];
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
                              Best_Phi (Ga.all, Natural (Jg (Jj).Length), W, Fg (Kf), Cx, Cy, Ph, Md, Wk);
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
                  Sa : Jc_Array_Ptr := To_Array (Js (J));
                  --  从一个起点(X 的前 Nx 个数)精修:先网格那批(Gg),再全部;返回全部配点上的残差中位
                  generic
                     Nx : Positive;
                     with procedure Res (Cs : Jc_Array; N : Natural; X : Vec; F, Cx, Cy : Long_Float; R : out Vec);
                  procedure Polish (Gg : Jc_Array_Ptr; Ng : Natural; X : in out Vec; Md : out Long_Float);
                  procedure Polish (Gg : Jc_Array_Ptr; Ng : Natural; X : in out Vec; Md : out Long_Float) is
                     Xx : Vec (0 .. Nx - 1) := X (X'First .. X'First + Nx - 1);
                     Steps : constant Vec (0 .. Nx - 1) := [others => 1.0e-6];   --  差分步(弧度,极小量)
                     procedure R_Small (Xa : Vec; R : out Vec) is
                     begin
                        Res (Gg.all, Ng, Xa, F0, Cx, Cy, R);
                     end R_Small;
                     procedure R_All (Xa : Vec; R : out Vec) is
                     begin
                        Res (Sa.all, Natural (Js (J).Length), Xa, F0, Cx, Cy, R);
                     end R_All;
                     R : Vec_Ptr := new Vec (0 .. Natural (Js (J).Length) - 1);
                  begin
                     Robust_LM (Xx, Ng, Ng, 60, Steps, R_Small'Access);
                     Robust_LM (Xx, Natural (Js (J).Length), Natural (Js (J).Length), 60, Steps, R_All'Access);
                     R_All (Xx, R.all);
                     Md := Median_Abs (R.all);
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
                           begin
                              Polish_T (Ga, Natural (Jg (J).Length), X, Md);
                              if Md < Bm_T then
                                 Bm_T := Md; Bx_T := X;
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
                           begin
                              Polish_S (Gs, Natural (Jgs (J).Length), X, Md);
                              if Md < Bm_S then
                                 Bm_S := Md; Bx_S := X;
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
         Off : array (0 .. Max_Joints - 1) of Natural := [others => 0];
         Nx : Natural := 1;
      begin
         for J in 0 .. N - 1 loop
            Off (J) := Nx; Nx := Nx + Size_Of (J);
         end loop;
         declare
            X : Vec (0 .. Nx - 1);
            Steps : constant Vec (0 .. Nx - 1) := [others => 1.0e-6];   --  差分步(弧度 / 对数焦距,极小量,无量纲)
            Arrs : array (0 .. Max_Joints - 1) of Jc_Array_Ptr;
            Ns : array (0 .. Max_Joints - 1) of Natural := [others => 0];
            N_Tot : Natural := 0;
            procedure R_All (Xx : Vec; R : out Vec) is
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
            end R_All;
         begin
            for J in 0 .. N - 1 loop
               Arrs (J) := To_Array (Js (J)); Ns (J) := Natural (Js (J).Length); N_Tot := N_Tot + Ns (J);
               X (Off (J) .. Off (J) + Size_Of (J) - 1) := Xj (J) (0 .. Size_Of (J) - 1);
            end loop;
            X (0) := Log (F0);
            if N_Tot > Nx then
               Robust_LM (X, N_Tot, N_Tot, 60, Steps, R_All'Access);
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
         Af : array (0 .. Nf - 1, 0 .. Max_Joints - 1) of V3;   --  每一帧各轴的 a_j(q)(参照眼系,p̂ 为单位长)
         Rq : array (0 .. Nf - 1) of M3;                         --  每一帧整串的转动
         Seen : array (0 .. Max_Joints - 1) of Boolean := [others => False];
         Rows : Rho_Rows_Ptr;
         N_Rows : Natural := 0;
         P_First, P_Count : Nat_Vectors.Vector;   --  每一对:在 Rows 里从第几行起、几行
         Rho, Best_Rho : V12 := [others => 0.0];
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
         procedure Solve (Sel : Nat_Array; Wt : Vec; Out_Rho : out V12) is
            A : Mat (0 .. N - 1, 0 .. N - 1) := [others => [others => 0.0]];
            D : V12 := [others => 0.0];
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
                  Gs : V12 := [others => 0.0];
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
               E : constant V12 := Min_Eig (A, N);
               Nr : Long_Float := 0.0;
            begin
               Out_Rho := [others => 0.0];
               for X in 0 .. N - 1 loop
                  Out_Rho (X) := E (X) / D (X);
                  Nr := Nr + Out_Rho (X) ** 2;
               end loop;
               if Nr > 0.0 then
                  for X in 0 .. N - 1 loop
                     Out_Rho (X) := Out_Rho (X) / Sqrt (Nr);
                  end loop;
               end if;
            end;
         end Solve;
         --  按 Sampson 加权反复重解(Huber 1 px;Gated = 门外的不要,门 = max(3 px, 3 × 中位),每遍重算)
         procedure Refine (Sel : Nat_Array; R : in out V12; Iters : Natural; Gated : Boolean) is
            Cnt : constant Natural := Sel'Length;
            Wt : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
            Rs : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
            Tmp : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
         begin
            for It in 1 .. Iters loop
               declare
                  Den : Long_Float;
                  Gate : Long_Float := Long_Float'Last;
                  New_R : V12;
                  Dot_R, Ch : Long_Float := 0.0;
               begin
                  for K in 0 .. Cnt - 1 loop
                     Rs (K) := Rho_Res (Rows (Sel (Sel'First + K)), R, N, F0, Den);
                     Wt (K) := (F0 / Den) ** 2;
                  end loop;
                  if Gated then
                     Gate := Long_Float'Max (3.0, 3.0 * Median_In (Rs.all, Cnt, Tmp));   --  3 px / 3 倍中位(协议,同 ③)
                  end if;
                  for K in 0 .. Cnt - 1 loop
                     Wt (K) := (if abs Rs (K) >= Gate then 0.0 elsif abs Rs (K) <= 1.0 then Wt (K) else Wt (K) / abs Rs (K));
                  end loop;
                  Solve (Sel, Wt (0 .. Cnt - 1), New_R);
                  for X in 0 .. N - 1 loop
                     Dot_R := Dot_R + New_R (X) * R (X);
                  end loop;
                  if Dot_R < 0.0 then
                     for X in 0 .. N - 1 loop
                        New_R (X) := -New_R (X);
                     end loop;
                  end if;
                  for X in 0 .. N - 1 loop
                     Ch := Ch + (New_R (X) - R (X)) ** 2;
                  end loop;
                  R := New_R;
                  exit when Ch < 1.0e-20;   --  不再变了(数值,无量纲)
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
         Rows := new Rho_Rows (0 .. Natural'Max (1, N_Rows) - 1);
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
                        R0 : V12;
                        Den : Long_Float;
                     begin
                        for P of Nat_Array'[Pa, Pb, Pc] loop
                           for I in 0 .. Sc_Cnt (P) - 1 loop
                              Tri (K) := Sc (Sc_First (P) + I); K := K + 1;
                           end loop;
                        end loop;
                        if Nt > 0 then
                           Solve (Tri (0 .. Nt - 1), Vec'(0 .. Nt - 1 => 1.0), R0);
                           Refine (Tri (0 .. Nt - 1), R0, 2, Gated => False);   --  两遍 Sampson 加权(次数)
                           declare
                              Sm : Long_Float := 0.0;
                           begin
                              for I in 0 .. Nsc - 1 loop
                                 Sm := Sm + Long_Float'Min (Rho_Res (Rows (Sc (I)), R0, N, F0, Den) ** 2, Tau ** 2);
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
               Refine (All_Sel (0 .. N_Rows - 1), Rho, 30, Gated => True);   --  最多 30 遍(次数)
               for K in 0 .. N_Rows - 1 loop
                  Rs (K) := Rho_Res (Rows (K), Rho, N, F0, Den);
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
      begin
         M0.F := F0;
         for J in 0 .. N - 1 loop
            M0.Ax (J) := (W => Wj (J), P => Pj (J), Slide => Sl (J));
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
                  --  约束行(不加权):转的轴是单位向量、P 取轴上离参照眼最近那点;尺度钉住(倍数只管数值,1e3 = 这几行比像素残差重得多,比例)
                  Axis_Rows (Mm, R (R'First + Ni .. R'First + Ni + 2 * Nt_Ax - 1));
                  R (R'First + Ni + 2 * Nt_Ax) := 1.0e3 * ((if Cnt > 0 then Sqrt (S2 / Long_Float (Cnt)) else 1.0) - 1.0);
               end R_All;
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
               Robust_LM (X, Ni + N_Reg, Ni, 300, Steps, R_All'Access);
               M := To_Model (X);
               for J in 0 .. N - 1 loop
                  if not M.Ax (J).Slide then
                     M.Ax (J).W := Unit (M.Ax (J).W);
                  end if;
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
               Scale_Model (M, -1.0);   --  转的轴 P 反号、走的关节 W 反号:所有平移一起反号,转动不变
               Rep.Flipped := True;
            end if;
         end;
         --  ④ 多视图:有轨迹就按重投影一起解(③ 的结果当起步)
         Refine_Mv (Frames, Cs, M, Rep);
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
      --  内点(门 = 2.5 × 1.4826 × 中位,统计常数,无量纲)按最小二乘重拟合:中心 + 协方差最小特征向量(用 4×4 那个求解器:补一行一列 0)。
      --  散布按门里的点重估、再挑再拟合,两遍(09-27 V1B32:格点铺满全画幅以后桌面点只占五成多,全体点的中位 = 2.6 mm,真桌面点只散 0.5 mm ——
      --  最小中位数的中位在内点不到一半多时量的是门,不是面;交出去的 Med = 门里的点离面的中位)
      declare
         Sig : Long_Float := 1.4826 * Best_Med;   --  正态下中位换标准差(统计常数)
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
                  if Nin >= 3 then
                     Sig := Long_Float'Max (1.4826 * Median_Abs (Ins (0 .. Nin - 1)), 1.0e-9 * Ext);   --  中位换标准差(统计常数);下限同上(比例,无量纲)
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
