with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Codec;
with Contact;
with Plug;
with Selfmap;
with Stats;
package body Linkage is

   Dim : constant := 3;        --  三维:一个点三个分量
   Pose_N : constant := 6;     --  一块一帧的位姿几个数:转 3 + 移 3
   Geo_N : constant := 4;      --  一根轴的几何几个数(三维里一条直线 4 个数):C 走的方向 2、曲率向量 2(在垂直于 C 走的方向的平面里)
   Ka : constant := 2;         --  这 4 个里曲率向量的两个分量从第几个起(第 Ka、Ka + 1 个)

   subtype Vec is Kinem.Vec;
   type Mat_Ptr is access Mat;
   procedure Free is new Ada.Unchecked_Deallocation (Mat, Mat_Ptr);
   type Vec_Ptr is access Vec;
   procedure Free is new Ada.Unchecked_Deallocation (Vec, Vec_Ptr);

   function Gate (Nu : Positive) return Long_Float is
      V : constant Long_Float := 2.0 / (9.0 * Long_Float (Nu));
   begin
      return Long_Float (Nu) * (1.0 - V + Stats.Z * Sqrt (V)) ** 3;
   end Gate;

   function Gate_F (Nu : Positive; Dof : Long_Float) return Long_Float is
      A : constant Long_Float := 2.0 / (9.0 * Long_Float (Nu));
      B : constant Long_Float := (if Dof < Long_Float'Last then 2.0 / (9.0 * Dof) else 0.0);
      Z : constant Long_Float := Stats.Z;
      Qa : constant Long_Float := (1.0 - B) ** 2 - Z ** 2 * B;
      Qb : constant Long_Float := 2.0 * (1.0 - B) * (1.0 - A);
      Qc : constant Long_Float := (1.0 - A) ** 2 - Z ** 2 * A;
   begin
      if B = 0.0 then
         return Gate (Nu);
      end if;
      if not (Qa > 0.0) then
         return Long_Float'Last;
      end if;
      return Long_Float (Nu) * ((Qb + Sqrt (Long_Float'Max (0.0, Qb ** 2 - 4.0 * Qa * Qc))) / (2.0 * Qa)) ** 3;
   end Gate_F;

   function Z_Of (Chi : Long_Float; Nu : Positive; Dof : Long_Float) return Long_Float is
      A : constant Long_Float := 2.0 / (9.0 * Long_Float (Nu));
      B : constant Long_Float := (if Dof < Long_Float'Last then 2.0 / (9.0 * Dof) else 0.0);
      X : constant Long_Float := (Long_Float'Max (0.0, Chi) / Long_Float (Nu)) ** (1.0 / 3.0);
   begin
      return ((1.0 - B) * X - (1.0 - A)) / Sqrt (B * X ** 2 + A);
   end Z_Of;


   --  3×3 求逆(伴随阵 ÷ 行列式)。行列式不比这个矩阵自己的数值分辨率(ε × 最大元³)大 ⇒ 求不了
   function Inv3 (A : M3; Ok : out Boolean) return M3 is
      C00 : constant Long_Float := A (1, 1) * A (2, 2) - A (1, 2) * A (2, 1);
      C01 : constant Long_Float := A (1, 2) * A (2, 0) - A (1, 0) * A (2, 2);
      C02 : constant Long_Float := A (1, 0) * A (2, 1) - A (1, 1) * A (2, 0);
      Det : constant Long_Float := A (0, 0) * C00 + A (0, 1) * C01 + A (0, 2) * C02;
      Mx : Long_Float := 0.0;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Mx := Long_Float'Max (Mx, abs A (I, J));
         end loop;
      end loop;
      Ok := abs Det > Long_Float'Model_Epsilon * Mx ** 3;
      if not Ok then
         return Geom.Identity;
      end if;
      return [[C00 / Det, (A (0, 2) * A (2, 1) - A (0, 1) * A (2, 2)) / Det, (A (0, 1) * A (1, 2) - A (0, 2) * A (1, 1)) / Det],
              [C01 / Det, (A (0, 0) * A (2, 2) - A (0, 2) * A (2, 0)) / Det, (A (0, 2) * A (1, 0) - A (0, 0) * A (1, 2)) / Det],
              [C02 / Det, (A (0, 1) * A (2, 0) - A (0, 0) * A (2, 1)) / Det, (A (0, 0) * A (1, 1) - A (0, 1) * A (1, 0)) / Det]];
   end Inv3;

   function Mahal (R : V3; C : M3) return Long_Float is
      Ok : Boolean;
      Ci : constant M3 := Inv3 (C, Ok);
   begin
      if not Ok then
         return (if Geom.Norm (R) > 0.0 then Long_Float'Last else 0.0);
      end if;
      return Contact.Dot (R, Geom.Ap (Ci, R));
   end Mahal;

   --  对称阵的特征分解(循环 Jacobi,转法同 Kinem.Min_Eig):A(下标从 0 起)就地转成对角 = 特征值,V 的列 = 对应的特征向量。
   --  转一次只会让非对角元的平方和变小 ⇒ 做到它不再变小为止(停在舍入那一层;不设遍数)
   procedure Eig_Sym (A : in out Mat; V : out Mat) is
      N : constant Natural := A'Length (1);
      Prev : Long_Float := Long_Float'Last;
   begin
      for I in 0 .. N - 1 loop
         for J in 0 .. N - 1 loop
            V (I, J) := (if I = J then 1.0 else 0.0);
         end loop;
      end loop;
      loop
         declare
            Off : Long_Float := 0.0;
         begin
            for P in 0 .. N - 1 loop
               for Q in P + 1 .. N - 1 loop
                  Off := Off + A (P, Q) ** 2;
               end loop;
            end loop;
            exit when not (Off > 0.0 and then Off < Prev);
            Prev := Off;
         end;
         for P in 0 .. N - 1 loop
            for Q in P + 1 .. N - 1 loop
               if A (P, Q) /= 0.0 then
                  declare
                     Th : constant Long_Float := 0.5 * Arctan (2.0 * A (P, Q), A (Q, Q) - A (P, P));
                     C : constant Long_Float := Cos (Th);
                     S : constant Long_Float := Sin (Th);
                  begin
                     for K in 0 .. N - 1 loop
                        declare
                           Akp : constant Long_Float := A (K, P);
                           Akq : constant Long_Float := A (K, Q);
                        begin
                           A (K, P) := C * Akp - S * Akq;
                           A (K, Q) := S * Akp + C * Akq;
                        end;
                     end loop;
                     for K in 0 .. N - 1 loop
                        declare
                           Apk : constant Long_Float := A (P, K);
                           Aqk : constant Long_Float := A (Q, K);
                        begin
                           A (P, K) := C * Apk - S * Aqk;
                           A (Q, K) := S * Apk + C * Aqk;
                        end;
                     end loop;
                     for K in 0 .. N - 1 loop
                        declare
                           Vkp : constant Long_Float := V (K, P);
                           Vkq : constant Long_Float := V (K, Q);
                        begin
                           V (K, P) := C * Vkp - S * Vkq;
                           V (K, Q) := S * Vkp + C * Vkq;
                        end;
                     end loop;
                  end;
               end if;
            end loop;
         end loop;
      end loop;
   end Eig_Sym;

   --  3×3 对称阵(协方差)最大的特征值
   function Max_Eig3 (C : M3) return Long_Float is
      A : Mat (M3'Range (1), M3'Range (2));
      V : Mat (M3'Range (1), M3'Range (2));
      Mx : Long_Float := Long_Float'First;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            A (I, J) := 0.5 * (C (I, J) + C (J, I));
         end loop;
      end loop;
      Eig_Sym (A, V);
      for I in 0 .. 2 loop
         Mx := Long_Float'Max (Mx, A (I, I));
      end loop;
      return Mx;
   end Max_Eig3;

   --  垂直于 W(单位向量)的一对单位向量:辅助方向取 W 分量绝对值最小的那根坐标轴(和 W 最不平行,叉乘永远不退化)
   procedure Perp (W : V3; E1, E2 : out V3) is
      I : Natural := W'First;
      A : V3 := [others => 0.0];
      Ok : Boolean;
   begin
      for J in W'Range loop
         if abs W (J) < abs W (I) then
            I := J;
         end if;
      end loop;
      A (I) := 1.0;
      E1 := Contact.Unit (Contact.Cross (W, A), Ok);
      if not Ok then
         E1 := A;
      end if;
      E2 := Contact.Cross (W, E1);
   end Perp;

   procedure Fit (Tracks : Track_Vectors.Vector; Rep : out Report) is
      N : constant Natural := Natural (Tracks.Length);
      K : Natural := 0;
   begin
      Rep := (others => <>);
      for T of Tracks loop
         K := Natural'Max (K, Natural (T.Length));
      end loop;
      Rep.Frames := K;
      for I in 0 .. N - 1 loop
         Rep.Roles.Append (Unused);
         Rep.Piece_Of.Append (-1);
      end loop;
      if N = 0 or else K = 0 then
         return;
      end if;
      declare
         function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
         function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
         function Scl (A : V3; S : Long_Float) return V3 is ([A (0) * S, A (1) * S, A (2) * S]);
         function Unit3 (A : V3) return V3 is
            Ok : Boolean;
            U : constant V3 := Contact.Unit (A, Ok);
         begin
            return (if Ok then U else [others => 0.0]);
         end Unit3;
         --  sin x / x(x = 0 ⇒ 1)
         function Sinc (X : Long_Float) return Long_Float is (if X = 0.0 then 1.0 else Sin (X) / X);

         --  每一笔观测:看见没有、位置、协方差的逆、白化(下三角 L⁻¹,协方差 = L Lᵀ ⇒ 白化后的平方和 = 马氏距离平方)、
         --  最不准那个方向上的噪声(√ 最大特征值,给的协方差)
         type Cell is record
            Seen : Boolean := False;
            X : V3 := [others => 0.0];
            Cv : M3 := [others => [others => 0.0]];
            Ci : M3 := [others => [others => 0.0]];
            Wh : M3 := [others => [others => 0.0]];
            Sd : Long_Float := 0.0;
         end record;
         type Grid is array (Natural range <>, Natural range <>) of Cell;
         type Grid_Ptr is access Grid;
         procedure Free is new Ada.Unchecked_Deallocation (Grid, Grid_Ptr);
         G : Grid_Ptr := new Grid (0 .. N - 1, 0 .. K - 1);
         type Pose_Arr is array (Natural range <>) of Pose;
         type Nat_Arr is array (Natural range <>) of Natural;
         type Bool_Arr is array (Natural range <>) of Boolean;
         Usable : Bool_Arr (0 .. N - 1) := [others => False];
         Ref : Natural := 0;
         Scale : Long_Float := 0.0;
         Sig : Long_Float := 1.0;       --  噪声倍数(量不出来 —— 没有一对最近的点看见两帧 —— 就信给的协方差)
         --  这个噪声倍数自己有多准:量它用的有效自由度(信给的协方差 = 无穷,Long_Float'Last)
         Sig_Dof : Long_Float := Long_Float'Last;
         --  中位法量 σ 比标准差法差多少(渐近相对效率):8 m² φ(m)²,m = Φ⁻¹(3/4) = 1 / Kinem.Mad_Sigma(正态下中位法的方差 = 1 / (16 m² φ(m)² n),
         --  标准差法 = 1 / (2n))= 0.3675
         M_Med : constant Long_Float := 1.0 / Kinem.Mad_Sigma;
         Mad_Eff : constant Long_Float := 8.0 * M_Med ** 2 * (Exp (-0.5 * M_Med ** 2) / Sqrt (2.0 * Ada.Numerics.Pi)) ** 2;
         function Gate_F (Nu : Positive) return Long_Float is (Linkage.Gate_F (Nu, Sig_Dof));
         Floor_W : Long_Float := 0.0;   --  噪声倍数的数值下限(只防零:数据一点不差时残差一半以上正好是 0)
         Hs : constant Long_Float := Sqrt (Long_Float'Model_Epsilon);   --  前向差分最好的相对步子(数值)
         No_Pose : constant Pose := (Ok => False, R => Geom.Identity, T => [others => 0.0]);
         Id_Pose : constant Pose := (Ok => True, R => Geom.Identity, T => [others => 0.0]);
         --  一块一帧位姿的协方差(6×6,真的单位;顺序同 Fit_Piece 的参数:左乘的小转动 3、平移 3)
         type Cov6 is array (0 .. Pose_N - 1, 0 .. Pose_N - 1) of Long_Float;
         type Cov6_Arr is array (Natural range <>) of Cov6;
         Zero6 : constant Cov6 := [others => [others => 0.0]];

         type Work_Piece is record
            Ids : Geom.Nat_Vectors.Vector;
            P : Pose_Arr (0 .. K - 1);
            C6 : Cov6_Arr (0 .. K - 1) := [others => Zero6];
            Cost : Long_Float := -1.0;   --  P 正好是按 Ids 解出来的 ⇒ 那时的代价;不是(-1)⇒ 要用就重解
         end record;
         --  一轮归法的样子(每条轨迹:角色 + 归哪一块),用来认"回到了以前的某一轮"
         package State_Vectors is new Ada.Containers.Vectors (Natural, Bytes.Ints, Bytes.Int_Vectors."=");
         package WP_Vectors is new Ada.Containers.Vectors (Natural, Work_Piece);

         function To_Arr (V : Geom.Nat_Vectors.Vector) return Nat_Arr is
            A : Nat_Arr (0 .. Natural (V.Length) - 1);
         begin
            for I in A'Range loop
               A (I) := V (I);
            end loop;
            return A;
         end To_Arr;

         --  ── 一条轨迹按一组位姿:形状、残差 ──
         --  形状(它在这一块上、参照帧那一刻的世界位置)按最大似然:对位姿是线性的 ⇒ S = (Σ Rᵀ C⁻¹ R)⁻¹ Σ Rᵀ C⁻¹ (x − T);Nf = 用上了几帧
         procedure Solve_Shape (I : Natural; P : Pose_Arr; S : out V3; Nf : out Natural; Info : out M3) is
            B : V3 := [others => 0.0];
         begin
            S := [others => 0.0];
            Nf := 0;
            Info := [others => [others => 0.0]];
            for F in 0 .. K - 1 loop
               if G (I, F).Seen and then P (F).Ok then
                  declare
                     Rc : constant M3 := Geom.Mul (Geom.Tr (P (F).R), G (I, F).Ci);
                     Af : constant M3 := Geom.Mul (Rc, P (F).R);
                     Bf : constant V3 := Geom.Ap (Rc, Sub (G (I, F).X, P (F).T));
                  begin
                     for R in 0 .. 2 loop
                        B (R) := B (R) + Bf (R);
                        for C in 0 .. 2 loop
                           Info (R, C) := Info (R, C) + Af (R, C);
                        end loop;
                     end loop;
                     Nf := Nf + 1;
                  end;
               end if;
            end loop;
            if Nf > 0 then
               S := Geom.Solve3 (Info, B);
            end if;
         end Solve_Shape;
         --  白化残差的平方和。按解出的位姿当准的(Pred = False):这条轨迹参与了这一块的拟合,或者是在归块时成团地比 ——
         --  一块位姿的不准是这一帧所有点共同的误差,按单点加进门,离这一块远的一小团点每个点单看都"说得通",成团地看明明不是
         --  (09-30 路 6 离线 50 组种子:门上离轴远的一小块 9 / 10 次被并进底座)。
         --  Pred = True:这条轨迹没参与这一块的拟合,问"这一块把它预测到哪、有多不准" —— 每一帧加上这一块位姿的不准投到这一点上
         --  (Jp C6 Jpᵀ,Jp = [−[R S]×, I]:位姿按左乘的小转动、平移动一下,这一点挪多少);不加的话,块边上的点一被挡在外面,
         --  这一块按剩下的点重解、外推到它那儿的不准不算,它就一直被挡在外面(三块那条 3 / 6 次两个真点说不通)
         function D2_Of (I : Natural; P : Pose_Arr; C6 : Cov6_Arr; Pred : Boolean; S : V3) return Long_Float is
            D2 : Long_Float := 0.0;
         begin
            for F in 0 .. K - 1 loop
               if G (I, F).Seen and then P (F).Ok then
                  declare
                     V : constant V3 := Geom.Ap (P (F).R, S);
                     E : constant V3 := Sub (G (I, F).X, Add (V, P (F).T));
                  begin
                     if not Pred then
                        D2 := D2 + Contact.Dot (E, Geom.Ap (G (I, F).Ci, E)) / Sig ** 2;
                     else
                        declare
                           Jp : constant array (0 .. 2, 0 .. Pose_N - 1) of Long_Float :=
                             [[0.0, V (2), -V (1), 1.0, 0.0, 0.0],
                              [-V (2), 0.0, V (0), 0.0, 1.0, 0.0],
                              [V (1), -V (0), 0.0, 0.0, 0.0, 1.0]];
                           Ct : M3;
                           Ok : Boolean;
                        begin
                           for R in V3'Range loop
                              for C in V3'Range loop
                                 Ct (R, C) := Sig ** 2 * G (I, F).Cv (R, C);
                                 for A in 0 .. Pose_N - 1 loop
                                    for B in 0 .. Pose_N - 1 loop
                                       Ct (R, C) := Ct (R, C) + Jp (R, A) * C6 (F) (A, B) * Jp (C, B);
                                    end loop;
                                 end loop;
                              end loop;
                           end loop;
                           declare
                              Inv : constant M3 := Inv3 (Ct, Ok);
                           begin
                              D2 := D2 + (if Ok then Contact.Dot (E, Geom.Ap (Inv, E)) elsif Geom.Norm (E) > 0.0 then Long_Float'Last else 0.0);
                           end;
                        end;
                     end if;
                  end;
               end if;
            end loop;
            return D2;
         end D2_Of;
         --  这一条说得通这一块:至少两帧(一帧没有可比的),白化残差平方和不过 Gate (3 × (帧数 − 1))(3 个数的形状是从这些帧解出来的)
         function Fits (I : Natural; P : Pose_Arr; C6 : Cov6_Arr; Pred : Boolean; D2 : out Long_Float) return Boolean is
            S : V3;
            Nf : Natural;
            Info : M3;
         begin
            Solve_Shape (I, P, S, Nf, Info);
            D2 := (if Nf >= 2 then D2_Of (I, P, C6, Pred, S) else Long_Float'Last);
            return Nf >= 2 and then D2 <= Gate_F (Dim * (Nf - 1));
         end Fits;
         function Fits (I : Natural; P : Pose_Arr) return Boolean is
            D2 : Long_Float;
            No : constant Cov6_Arr (0 .. K - 1) := [others => Zero6];
         begin
            return Fits (I, P, No, False, D2);
         end Fits;
         --  一条轨迹的白化残差(÷ Sig)依次填进 R (J ..);J 往后挪
         procedure Res_Track (I : Natural; P : Pose_Arr; R : in out Vec; J : in out Natural) is
            S : V3;
            Nf : Natural;
            Info : M3;
         begin
            Solve_Shape (I, P, S, Nf, Info);
            for F in 0 .. K - 1 loop
               if G (I, F).Seen and then P (F).Ok then
                  declare
                     E : constant V3 := Sub (G (I, F).X, Add (Geom.Ap (P (F).R, S), P (F).T));
                     W : constant V3 := Geom.Ap (G (I, F).Wh, E);
                  begin
                     for C in 0 .. 2 loop
                        R (J + C) := W (C) / Sig;
                     end loop;
                     J := J + Dim;
                  end;
               end if;
            end loop;
         end Res_Track;
         function N_Res (Ids : Nat_Arr; P : Pose_Arr) return Natural is
            C : Natural := 0;
         begin
            for I of Ids loop
               for F in 0 .. K - 1 loop
                  if G (I, F).Seen and then P (F).Ok then
                     C := C + Dim;
                  end if;
               end loop;
            end loop;
            return C;
         end N_Res;
         function Sum_Sq (R : Vec) return Long_Float is
            S : Long_Float := 0.0;
         begin
            for X of R loop
               S := S + X ** 2;
            end loop;
            return S;
         end Sum_Sq;

         --  一串数的中位(拷一份,快速选择)
         function Median_Of (R : Vec) return Long_Float is
            W : Vec_Ptr := new Vec (0 .. R'Length - 1);
            Kx : constant Natural := R'Length / 2;
            Lo : Integer := 0;
            Hi : Integer := R'Length - 1;
            Md : Long_Float;
         begin
            for Q in 0 .. R'Length - 1 loop
               W (Q) := abs R (R'First + Q);
            end loop;
            while Lo < Hi loop
               declare
                  Pv : constant Long_Float := W ((Lo + Hi) / 2);
                  Ii : Integer := Lo;
                  Jj : Integer := Hi;
                  Tt : Long_Float;
               begin
                  while Ii <= Jj loop
                     while W (Ii) < Pv loop
                        Ii := Ii + 1;
                     end loop;
                     while W (Jj) > Pv loop
                        Jj := Jj - 1;
                     end loop;
                     if Ii <= Jj then
                        Tt := W (Ii);
                        W (Ii) := W (Jj);
                        W (Jj) := Tt;
                        Ii := Ii + 1;
                        Jj := Jj - 1;
                     end if;
                  end loop;
                  if Kx <= Jj then
                     Hi := Jj;
                  elsif Kx >= Ii then
                     Lo := Ii;
                  else
                     exit;
                  end if;
               end;
            end loop;
            Md := W (Kx);
            Free (W);
            return Md;
         end Median_Of;

         --  这几个点显著地不在一条线上(绕哪根轴转都量得出):离最好的那条线的平方和(散布阵第二、三个特征值之和)超过
         --  "它们正好在一条线上、只有噪声(最不准那个方向的 Sd_Max × Sig)"时的门 Gate (2 (n − 2))(直线 4 个数,每点垂直于线 2 个残差)
         function Spread_Ok (Xs : Kinem.V3_Array; Sd_Max : Long_Float) return Boolean is
            Nn : constant Natural := Xs'Length;
            C : V3 := [others => 0.0];
            Sc : Mat (M3'Range (1), M3'Range (2)) := [others => [others => 0.0]];
            Ev : Mat (M3'Range (1), M3'Range (2));
            Tr_S, Mx : Long_Float := 0.0;
         begin
            if Nn < Dim then
               return False;
            end if;
            for X of Xs loop
               C := Add (C, X);
            end loop;
            C := Scl (C, 1.0 / Long_Float (Nn));
            for X of Xs loop
               declare
                  D : constant V3 := Sub (X, C);
               begin
                  for R in 0 .. 2 loop
                     for Cc in 0 .. 2 loop
                        Sc (R, Cc) := Sc (R, Cc) + D (R) * D (Cc);
                     end loop;
                  end loop;
               end;
            end loop;
            Eig_Sym (Sc, Ev);
            for R in 0 .. 2 loop
               Tr_S := Tr_S + Sc (R, R);
               Mx := Long_Float'Max (Mx, Sc (R, R));
            end loop;
            return Tr_S - Mx > (Sig * Sd_Max) ** 2 * Gate_F (2 * (Nn - 2));
         end Spread_Ok;

         --  带权的 Horn(1987,单位四元数):A_i ↦ B_i 的最小二乘刚体运动(B ≈ R A + T),4×4 对称阵最大特征值的特征向量 = 转动的四元数
         procedure Horn (A, B : Kinem.V3_Array; Wt : Vec; R : out M3; T : out V3) is
            Sw : Long_Float := 0.0;
            Ca, Cb : V3 := [others => 0.0];
            S : M3 := [others => [others => 0.0]];
            Nm, Ev : Mat (0 .. Dim, 0 .. Dim);   --  单位四元数 Dim + 1 个数
            Best : Natural := 0;
         begin
            for I in A'Range loop
               Sw := Sw + Wt (I);
               Ca := Add (Ca, Scl (A (I), Wt (I)));
               Cb := Add (Cb, Scl (B (I), Wt (I)));
            end loop;
            Ca := Scl (Ca, 1.0 / Sw);
            Cb := Scl (Cb, 1.0 / Sw);
            for I in A'Range loop
               declare
                  Da : constant V3 := Sub (A (I), Ca);
                  Db : constant V3 := Sub (B (I), Cb);
               begin
                  for P in 0 .. 2 loop
                     for Q in 0 .. 2 loop
                        S (P, Q) := S (P, Q) + Wt (I) * Da (P) * Db (Q);
                     end loop;
                  end loop;
               end;
            end loop;
            Nm := [[S (0, 0) + S (1, 1) + S (2, 2), S (1, 2) - S (2, 1), S (2, 0) - S (0, 2), S (0, 1) - S (1, 0)],
                   [S (1, 2) - S (2, 1), S (0, 0) - S (1, 1) - S (2, 2), S (0, 1) + S (1, 0), S (2, 0) + S (0, 2)],
                   [S (2, 0) - S (0, 2), S (0, 1) + S (1, 0), S (1, 1) - S (0, 0) - S (2, 2), S (1, 2) + S (2, 1)],
                   [S (0, 1) - S (1, 0), S (2, 0) + S (0, 2), S (1, 2) + S (2, 1), S (2, 2) - S (0, 0) - S (1, 1)]];
            Eig_Sym (Nm, Ev);
            for I in Nm'Range (1) loop
               if Nm (I, I) > Nm (Best, Best) then
                  Best := I;
               end if;
            end loop;
            R := Geom.Quat_To_R ([0.0, 0.0, 0.0, Ev (0, Best), Ev (1, Best), Ev (2, Best), Ev (3, Best)]);
            T := Sub (Cb, Geom.Ap (R, Ca));
         end Horn;

         --  这几条轨迹(Ids)在参照帧和第 F 帧都看见的那几个:参照帧 ↦ 第 F 帧按 Horn 求一个位姿(权 = 两帧噪声平方和的倒数);
         --  看见的不够铺开(Spread_Ok)⇒ Ok = False
         function Horn_Pose (Ids : Nat_Arr; F : Natural) return Pose is
            Cnt : Natural := 0;
            Sd_Max : Long_Float := 0.0;
         begin
            for I of Ids loop
               if G (I, Ref).Seen and then G (I, F).Seen then
                  Cnt := Cnt + 1;
               end if;
            end loop;
            declare
               A, B : Kinem.V3_Array (0 .. Natural'Max (1, Cnt) - 1);
               Wt : Vec (0 .. Natural'Max (1, Cnt) - 1);
               J : Natural := 0;
               Pz : Pose := No_Pose;
            begin
               if Cnt < Dim then
                  return No_Pose;
               end if;
               for I of Ids loop
                  if G (I, Ref).Seen and then G (I, F).Seen then
                     A (J) := G (I, Ref).X;
                     B (J) := G (I, F).X;
                     Wt (J) := 1.0 / (G (I, Ref).Sd ** 2 + G (I, F).Sd ** 2);
                     Sd_Max := Long_Float'Max (Sd_Max, Long_Float'Max (G (I, Ref).Sd, G (I, F).Sd));
                     J := J + 1;
                  end if;
               end loop;
               if not Spread_Ok (A, Sd_Max) then
                  return No_Pose;
               end if;
               Horn (A, B, Wt, Pz.R, Pz.T);
               Pz.Ok := True;
               return Pz;
            end;
         end Horn_Pose;

         --  对称半正定矩阵的(广义)逆,原地:按对角元缩放后高斯–约当,主元小到数值分辨率以下的那几维当定不住
         --  (Undet;逆里那几行几列 = 0)。下标从 0 起
         procedure Pinv (H : in out Mat; Undet : out Bool_Arr) is
            Np : constant Natural := H'Length (1);
         begin
            Undet := [others => False];
            if Np = 0 then
               return;
            end if;
            declare
               Dg : Vec (0 .. Np - 1);
               Aug : Mat_Ptr := new Mat (0 .. Np - 1, 0 .. 2 * Np - 1);
            begin
               Aug.all := [others => [others => 0.0]];
               for P in 0 .. Np - 1 loop
                  Dg (P) := (if H (P, P) > 0.0 then Sqrt (H (P, P)) else 0.0);
                  Undet (P) := not (Dg (P) > 0.0);
               end loop;
               for P in 0 .. Np - 1 loop
                  for Q in 0 .. Np - 1 loop
                     Aug (P, Q) := (if Undet (P) or else Undet (Q) then 0.0 else H (P, Q) / (Dg (P) * Dg (Q)));
                  end loop;
                  Aug (P, Np + P) := 1.0;
               end loop;
               for Col in 0 .. Np - 1 loop
                  if not Undet (Col) then
                     declare
                        Piv : Natural := Col;
                     begin
                        for Rw in Col + 1 .. Np - 1 loop
                           if abs Aug (Rw, Col) > abs Aug (Piv, Col) then
                              Piv := Rw;
                           end if;
                        end loop;
                        if not (abs Aug (Piv, Col) > Long_Float (Np) * Long_Float'Model_Epsilon) then
                           Undet (Col) := True;
                        else
                           if Piv /= Col then
                              for Q in 0 .. 2 * Np - 1 loop
                                 declare
                                    Tt : constant Long_Float := Aug (Col, Q);
                                 begin
                                    Aug (Col, Q) := Aug (Piv, Q);
                                    Aug (Piv, Q) := Tt;
                                 end;
                              end loop;
                           end if;
                           declare
                              Dv : constant Long_Float := Aug (Col, Col);
                           begin
                              for Q in 0 .. 2 * Np - 1 loop
                                 Aug (Col, Q) := Aug (Col, Q) / Dv;
                              end loop;
                           end;
                           for Rw in 0 .. Np - 1 loop
                              if Rw /= Col and then Aug (Rw, Col) /= 0.0 then
                                 declare
                                    Fc : constant Long_Float := Aug (Rw, Col);
                                 begin
                                    for Q in 0 .. 2 * Np - 1 loop
                                       Aug (Rw, Q) := Aug (Rw, Q) - Fc * Aug (Col, Q);
                                    end loop;
                                 end;
                              end if;
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
               for P in 0 .. Np - 1 loop
                  for Q in 0 .. Np - 1 loop
                     H (P, Q) := (if Undet (P) or else Undet (Q) then 0.0 else Aug (P, Np + Q) / (Dg (P) * Dg (Q)));
                  end loop;
               end loop;
               Free (Aug);
            end;
         end Pinv;

         --  参数的协方差 = 解处的 (Jᵀ W J)⁻¹:J = 残差对参数的数值雅可比(前向差分,步子同 LM),W = Huber 权(同 LM;残差已经以 σ 为单位)。
         --  先按对角缩放成全 1 再求逆(高斯-约当,列主元);缩放后主元小到数值分辨率以下 ⇒ 这个参数定不住(Undet),它那一行一列记 0,别的照算
         --  (比如 κ = 0 时轴绕 C 走的方向的角对残差毫无影响)
         procedure Param_Cov (X, St : Vec; N_R : Natural; Resid : not null access procedure (Xx : Vec; R : out Vec);
                              H : out Mat; Undet : out Bool_Arr) is
            Np : constant Natural := X'Length;
            R0 : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_R) - 1);
            R1 : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_R) - 1);
            Jc : Mat_Ptr := new Mat (0 .. Natural'Max (1, N_R) - 1, 0 .. Natural'Max (1, Np) - 1);
         begin
            H := [others => [others => 0.0]];
            Undet := [others => False];
            if Np = 0 or else N_R = 0 then
               Free (R0);
               Free (R1);
               Free (Jc);
               return;
            end if;
            Resid (X, R0 (0 .. N_R - 1));
            for P in 0 .. Np - 1 loop
               declare
                  Xp : Vec := X;
               begin
                  Xp (Xp'First + P) := Xp (Xp'First + P) + St (St'First + P);
                  Resid (Xp, R1 (0 .. N_R - 1));
                  for Rw in 0 .. N_R - 1 loop
                     Jc (Rw, P) := (R1 (Rw) - R0 (Rw)) / St (St'First + P);
                  end loop;
               end;
            end loop;
            for Rw in 0 .. N_R - 1 loop
               declare
                  Wt : constant Long_Float := (if abs R0 (Rw) <= Kinem.Huber_K then 1.0 else Kinem.Huber_K / abs R0 (Rw));
               begin
                  for P in 0 .. Np - 1 loop
                     if Jc (Rw, P) /= 0.0 then
                        for Q in P .. Np - 1 loop
                           H (P, Q) := H (P, Q) + Wt * Jc (Rw, P) * Jc (Rw, Q);
                        end loop;
                     end if;
                  end loop;
               end;
            end loop;
            for P in 0 .. Np - 1 loop
               for Q in 0 .. P - 1 loop
                  H (P, Q) := H (Q, P);
               end loop;
            end loop;
            Pinv (H, Undet);
            Free (R0);
            Free (R1);
            Free (Jc);
         end Param_Cov;

         --  ── 一块的位姿:它所有成员一起按最大似然解 ──
         --  参照帧不动;P 里 Ok 的每一帧 6 个数(转:左乘的转动向量;移:加上去)交给 Kinem.Robust_LM,形状每一次按式子消掉(Res_Track)。
         --  P 进来是起点,出去是解;C6 = 每一帧位姿的协方差(Param_Cov 的对角块;定不住的数记 0);Cost = 解处白化残差的平方和(÷ Sig²,卡方)
         procedure Fit_Piece (Ids : Nat_Arr; P : in out Pose_Arr; C6 : out Cov6_Arr; Cost : out Long_Float; With_Cov : Boolean := True) is
            Fr : Nat_Arr (0 .. K - 1) := [others => 0];
            Nfr : Natural := 0;
            P0 : constant Pose_Arr := P;
            N_R : constant Natural := N_Res (Ids, P);
         begin
            for F in 0 .. K - 1 loop
               if P (F).Ok and then F /= Ref then
                  Fr (Nfr) := F;
                  Nfr := Nfr + 1;
               end if;
            end loop;
            declare
               Np : constant Natural := Pose_N * Nfr;
               X : Vec (0 .. Np - 1) := [others => 0.0];
               St : Vec (0 .. Np - 1);
               procedure Poses_Of (Xx : Vec; Q : out Pose_Arr) is
               begin
                  Q := P0;
                  for J in 0 .. Nfr - 1 loop
                     declare
                        B : constant Natural := Xx'First + Pose_N * J;
                        F : constant Natural := Fr (J);
                     begin
                        Q (F).R := Geom.Mul (Geom.Rodrigues (V3 (Xx (B .. B + Dim - 1))), P0 (F).R);
                        Q (F).T := Add (P0 (F).T, V3 (Xx (B + Dim .. B + Pose_N - 1)));
                     end;
                  end loop;
               end Poses_Of;
               procedure Resid (Xx : Vec; R : out Vec) is
                  Q : Pose_Arr (0 .. K - 1);
                  J : Natural := R'First;
               begin
                  Poses_Of (Xx, Q);
                  for I of Ids loop
                     Res_Track (I, Q, R, J);
                  end loop;
               end Resid;
               Rv : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_R) - 1);
               Done : Boolean;
            begin
               for J in 0 .. Nfr - 1 loop
                  for C in V3'Range loop
                     St (Pose_N * J + C) := Hs;
                     St (Pose_N * J + Dim + C) := Hs * Scale;
                  end loop;
               end loop;
               C6 := [others => Zero6];
               if Np > 0 and then N_R > Np then
                  Kinem.Robust_LM (X, N_R, N_R, Positive'Last, St, Resid'Access, Done);
               end if;
               if With_Cov and then Np > 0 and then N_R > Np then
                  declare
                     H : Mat (0 .. Np - 1, 0 .. Np - 1);
                     Undet : Bool_Arr (0 .. Np - 1);
                  begin
                     Param_Cov (X, St, N_R, Resid'Access, H, Undet);
                     for J in 0 .. Nfr - 1 loop
                        for A in 0 .. Pose_N - 1 loop
                           for B in 0 .. Pose_N - 1 loop
                              C6 (Fr (J)) (A, B) := H (Pose_N * J + A, Pose_N * J + B);
                           end loop;
                        end loop;
                     end loop;
                  end;
               end if;
               Poses_Of (X, P);
               if N_R > 0 then
                  Resid (X, Rv (0 .. N_R - 1));
                  Cost := Sum_Sq (Rv (0 .. N_R - 1));
               else
                  Cost := 0.0;
               end if;
               Free (Rv);
            end;
         end Fit_Piece;

         --  成员定了 ⇒ 位姿:参照帧 ↦ 每一帧按 Horn 起步,一起解;再用解出来的形状把起步时定不住的帧(成员在参照帧没看见)补上,补上了就再解,
         --  做到补不上为止
         --  Allowed = 只在这几帧解位姿(两种分法比代价时要在同一批帧上比);缺省 = 每一帧
         All_Frames : constant Bool_Arr (0 .. K - 1) := [others => True];
         procedure Refine (Ids : Nat_Arr; P : out Pose_Arr; C6 : out Cov6_Arr; Cost : out Long_Float; Allowed : Bool_Arr := All_Frames;
                           With_Cov : Boolean := True) is
            Added : Boolean;
         begin
            P := [others => No_Pose];
            P (Ref) := Id_Pose;
            for F in 0 .. K - 1 loop
               if F /= Ref and then Allowed (F) then
                  P (F) := Horn_Pose (Ids, F);
               end if;
            end loop;
            loop
               Fit_Piece (Ids, P, C6, Cost, With_Cov);
               Added := False;
               for F in 0 .. K - 1 loop
                  if not P (F).Ok and then Allowed (F) then
                     declare
                        Cnt : Natural := 0;
                     begin
                        for I of Ids loop
                           if G (I, F).Seen then
                              Cnt := Cnt + 1;
                           end if;
                        end loop;
                        if Cnt >= Dim then
                           declare
                              A, B : Kinem.V3_Array (0 .. Cnt - 1);
                              Wt : Vec (0 .. Cnt - 1);
                              J : Natural := 0;
                              Sd_Max : Long_Float := 0.0;
                              Nf : Natural;
                              Info : M3;
                              S : V3;
                              Pz : Pose := No_Pose;
                           begin
                              for I of Ids loop
                                 if G (I, F).Seen then
                                    Solve_Shape (I, P, S, Nf, Info);
                                    if Nf > 0 then
                                       A (J) := S;
                                       B (J) := G (I, F).X;
                                       Wt (J) := 1.0 / G (I, F).Sd ** 2;
                                       Sd_Max := Long_Float'Max (Sd_Max, G (I, F).Sd);
                                       J := J + 1;
                                    end if;
                                 end if;
                              end loop;
                              if J >= Dim and then Spread_Ok (A (0 .. J - 1), Sd_Max) then
                                 Horn (A (0 .. J - 1), B (0 .. J - 1), Wt (0 .. J - 1), Pz.R, Pz.T);
                                 Pz.Ok := True;
                                 P (F) := Pz;
                                 Added := True;
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
               exit when not Added;
            end loop;
         end Refine;

         --  ── 归块 ──
         --  一块一块往外拿。每条在参照帧看见的轨迹当种子,按参照帧的距离把离它最近的一条条加进来,加到这一小团显著铺开为止(Spread_Ok)⇒
         --  这一团参照帧 ↦ 各帧的位姿(Horn)= 一个起步;池子里说得通它的有几条 = 它的一致集。按一致集从大到小依次局部精修:
         --  按说得通的那些一起解位姿、再数说得通的,做到成员集回到以前出现过的某一批;立住了(成员 ≥ 3、除参照帧外至少一帧定得住)
         --  ⇒ 拿走这一块,池子里剩下的再来;精修塌了(混了两块的起步解出一个两边都不像的位姿,只剩几个点说得通)⇒ 丢掉它,试下一个
         --  (09-30 离线三块那一场:一致集最大的起步是抽屉 14 个点 + 门上靠轴的 4 个点,精修塌成 3 个,原来一塌就收工,一块都没归出来)。
         --  一个都立不住才收工。种子挨着找:一块东西上的点挨在一起,挨着的一小团多半在同一块上;起步的个数 = 轨迹条数,不是拍的
         procedure Segment (Out_P : out WP_Vectors.Vector) is
            Pool : Bool_Arr (0 .. N - 1) := Usable;
            function Dist2 (I, J : Natural) return Long_Float is
              (Contact.Dot (Sub (G (I, Ref).X, G (J, Ref).X), Sub (G (I, Ref).X, G (J, Ref).X)));
            type Hyp is record
               Seed, Cnt : Natural := 0;
               P : Pose_Arr (0 .. K - 1);
            end record;
            package Hyp_Vectors is new Ada.Containers.Vectors (Natural, Hyp);
            function Before (A, B : Hyp) return Boolean is (A.Cnt > B.Cnt or else (A.Cnt = B.Cnt and then A.Seed < B.Seed));
            package Hyp_Sort is new Hyp_Vectors.Generic_Sorting (Before);
            --  种子 I 的起步:I 和离它最近的几条,一条条加到显著铺开为止 + 这一团各帧的位姿;池子里凑不出铺开的一团、
            --  或者除了参照帧一帧也定不住 ⇒ Ok = False。
            --  不只配两个最近的点:三点里两点挨着、第三点再远,三角形的宽也超不过那两点的间距 —— 点比噪声密的时候一个真点也凑不出
            --  铺开的三点(09-30 离线铰链:间距 0.37、视线方向的噪声 0.06、噪声倍数起步量成 1.31 ⇒ 96 个真点一个起步都没有,
            --  只剩坏点配远处两个真点的那几个)
            procedure Seed_Hyp (I : Natural; P : out Pose_Arr; Ok : out Boolean) is
               Grp : Nat_Arr (0 .. N - 1) := [others => 0];
               Ng : Natural := 1;
               Tried : Bool_Arr (0 .. N - 1) := [others => False];
               Sd_Max : Long_Float := G (I, Ref).Sd;
            begin
               P := [others => No_Pose];
               Ok := False;
               Grp (0) := I;
               Tried (I) := True;
               loop
                  declare
                     Nx : Integer := -1;
                  begin
                     for J in 0 .. N - 1 loop
                        if not Tried (J) and then Pool (J) and then G (J, Ref).Seen and then (Nx < 0 or else Dist2 (I, J) < Dist2 (I, Nx)) then
                           Nx := J;
                        end if;
                     end loop;
                     exit when Nx < 0;
                     Tried (Nx) := True;
                     Grp (Ng) := Nx;
                     Ng := Ng + 1;
                     Sd_Max := Long_Float'Max (Sd_Max, G (Nx, Ref).Sd);
                     if Ng >= Dim then
                        declare
                           Xs : Kinem.V3_Array (0 .. Ng - 1);
                        begin
                           for Q in 0 .. Ng - 1 loop
                              Xs (Q) := G (Grp (Q), Ref).X;
                           end loop;
                           if Spread_Ok (Xs, Sd_Max) then
                              P (Ref) := Id_Pose;
                              for F in 0 .. K - 1 loop
                                 if F /= Ref then
                                    P (F) := Horn_Pose (Grp (0 .. Ng - 1), F);
                                    Ok := Ok or else P (F).Ok;
                                 end if;
                              end loop;
                              return;
                           end if;
                        end;
                     end if;
                  end;
               end loop;
            end Seed_Hyp;
            --  从起步 P0 局部精修成一块;立住了 ⇒ Ok
            procedure Grow (P0 : Pose_Arr; W : out Work_Piece; Ok : out Boolean) is
               Mem : Geom.Nat_Vectors.Vector;
               P : Pose_Arr (0 .. K - 1) := P0;
               C6 : Cov6_Arr (0 .. K - 1) := [others => Zero6];
               Cost : Long_Float;
               Solved : Boolean := False;          --  P 正好是按此刻的 Mem 解出来的
               Seen_Sets : State_Vectors.Vector;   --  精修里出现过的成员集(回到其中一个 = 做到头了)
            begin
               Ok := False;
               for T in 0 .. N - 1 loop
                  if Pool (T) and then Fits (T, P) then
                     Mem.Append (T);
                  end if;
               end loop;
               loop
                  exit when Natural (Mem.Length) < Dim;
                  Refine (To_Arr (Mem), P, C6, Cost);
                  declare
                     New_M : Geom.Nat_Vectors.Vector;
                     Code : Bytes.Ints;
                  begin
                     for T in 0 .. N - 1 loop
                        if Pool (T) and then Fits (T, P) then
                           New_M.Append (T);
                        end if;
                     end loop;
                     Solved := Geom.Nat_Vectors."=" (New_M, Mem);
                     exit when Solved;
                     for T of Mem loop
                        Code.Append (T);
                     end loop;
                     Seen_Sets.Append (Code);
                     Code.Clear;
                     for T of New_M loop
                        Code.Append (T);
                     end loop;
                     Mem := New_M;
                     exit when Seen_Sets.Contains (Code);   --  回到了以前的某一批:在几批之间来回,停在这一批
                  end;
               end loop;
               if Natural (Mem.Length) >= Dim then
                  for F in 0 .. K - 1 loop
                     Ok := Ok or else (F /= Ref and then P (F).Ok);
                  end loop;
               end if;
               W := (Ids => Mem, P => P, C6 => C6, Cost => (if Solved then Cost else -1.0));
            end Grow;
         begin
            Out_P.Clear;
            loop
               declare
                  Hs : Hyp_Vectors.Vector;
                  Took : Boolean := False;
               begin
                  for I in 0 .. N - 1 loop
                     if Pool (I) and then G (I, Ref).Seen then
                        declare
                           H : Hyp;
                           Ok : Boolean;
                        begin
                           Seed_Hyp (I, H.P, Ok);
                           if Ok then
                              H.Seed := I;
                              for T in 0 .. N - 1 loop
                                 if Pool (T) and then Fits (T, H.P) then
                                    H.Cnt := H.Cnt + 1;
                                 end if;
                              end loop;
                              if H.Cnt >= Dim then
                                 Hs.Append (H);
                              end if;
                           end if;
                        end;
                     end if;
                  end loop;
                  Hyp_Sort.Sort (Hs);
                  for H of Hs loop
                     declare
                        W : Work_Piece;
                        Ok : Boolean;
                     begin
                        Grow (H.P, W, Ok);
                        if Ok then
                           Out_P.Append (W);
                           for T of W.Ids loop
                              Pool (T) := False;
                           end loop;
                           Took := True;
                           exit;
                        end if;
                     end;
                  end loop;
                  exit when not Took;
               end;
            end loop;
         end Segment;

         --  每条轨迹对每一块,先按解出的位姿判(严;成团地归块靠它):正好一块说得通 ⇒ 归它(Member);两块以上 ⇒ 说得最好的两块
         --  对它的预测隔得开(Sep2 ≥ (2 Z)²)就归近的那块,隔不开 ⇒ Ambiguous(两种动法把它预测到同一处,分不清)。
         --  严判哪块都说不通的,再按"那一块把它预测到哪、有多不准"判一次(Pred:它没参与那一块的拟合,
         --  外推到它那儿的位姿不准要算上 —— 块边上的真点被挡在外面以后,这一块按剩下的点重解,不这样判它就一直被挡在外面):
         --  正好一块 ⇒ 归它;两块以上 ⇒ Ambiguous;没有 ⇒ Unexplained。
         --  只有 Member 进各块的拟合:分不清的点都挨着两块的交界、都跟着另一块挪一点点,一个个看在噪声以内,成群地放进一块就是系统偏差
         --  (09-30 离线:铰链靠轴的 8–15 个点放进底座,轴偏到自报的 2.9 倍)。
         --  按新的成员重解每一块,做到归法回到以前出现过的某一轮为止(包括没变);成员不够 3 条的块拿掉,它的轨迹下一轮重新归
         --  两块对第 I 条轨迹的预测相隔多远(马氏距离,按这条轨迹自己的噪声):A 按它的观测解出形状、预测各帧的位置 μ;
         --  B 拿 μ 当观测解形状,μ 离 B 的说法的白化平方和 = 间隔²(两边都定得住位姿、它又看见了的帧;不到两帧 ⇒ 0:分不出)
         function Sep2 (I : Natural; Pa, Pb : Pose_Arr) return Long_Float is
            S : V3;
            Nf : Natural;
            Info : M3;
            Info_B : M3 := [others => [others => 0.0]];
            Bv : V3 := [others => 0.0];
            Mu : array (0 .. K - 1) of V3;
            Both : Natural := 0;
            D2 : Long_Float := 0.0;
         begin
            Solve_Shape (I, Pa, S, Nf, Info);
            for F in 0 .. K - 1 loop
               if G (I, F).Seen and then Pa (F).Ok and then Pb (F).Ok then
                  Mu (F) := Add (Geom.Ap (Pa (F).R, S), Pa (F).T);
                  declare
                     Rc : constant M3 := Geom.Mul (Geom.Tr (Pb (F).R), G (I, F).Ci);
                     Af : constant M3 := Geom.Mul (Rc, Pb (F).R);
                     Bf : constant V3 := Geom.Ap (Rc, Sub (Mu (F), Pb (F).T));
                  begin
                     for R in V3'Range loop
                        Bv (R) := Bv (R) + Bf (R);
                        for C in V3'Range loop
                           Info_B (R, C) := Info_B (R, C) + Af (R, C);
                        end loop;
                     end loop;
                  end;
                  Both := Both + 1;
               end if;
            end loop;
            if Both < 2 then
               return 0.0;
            end if;
            declare
               Sb : constant V3 := Geom.Solve3 (Info_B, Bv);
            begin
               for F in 0 .. K - 1 loop
                  if G (I, F).Seen and then Pa (F).Ok and then Pb (F).Ok then
                     declare
                        E : constant V3 := Sub (Mu (F), Add (Geom.Ap (Pb (F).R, Sb), Pb (F).T));
                     begin
                        D2 := D2 + Contact.Dot (E, Geom.Ap (G (I, F).Ci, E));
                     end;
                  end if;
               end loop;
            end;
            return D2 / Sig ** 2;
         end Sep2;
         --  两块是不是同一个刚体(Try_Split 的判法反过来,一对一比、不挑):合成一块比各自一块多出来的卡方(自由度 = 多出来的位姿个数,
         --  两块都定得住位姿的那几帧,只在这几帧上解)不显著(Z_Of ≤ Z)⇒ 同一块,合起来。最不显著的一对先合,合到没有该合的为止。
         --  两块除参照帧没有共同的帧 ⇒ 比不了,不合。
         --  (09-30 离线铰链:一次分错把底座拆成两块,两块对底座上每个点的说法一样 ⇒ 一个个都成了"分不清"⇒ 两块都没了成员、一起被拿掉,
         --  底座 45 个真点全说不通)
         --  Tested = 这一趟归块里比过、不该合的那几对(按两块的成员记:成员没变、噪声倍数没变 ⇒ 结果不变,不重比)
         procedure Merge_Same (Ps : in out WP_Vectors.Vector; Tested : in out State_Vectors.Vector) is
         begin
            loop
               declare
                  Best_A, Best_B : Integer := -1;
                  Best_Z : Long_Float := Stats.Z;
               begin
                  for A in 0 .. Natural (Ps.Length) - 1 loop
                     for B in A + 1 .. Natural (Ps.Length) - 1 loop
                        declare
                           Common : Bool_Arr (0 .. K - 1);
                           Nc : Natural := 0;
                           Code : Bytes.Ints;
                        begin
                           for T of Ps (A).Ids loop
                              Code.Append (T);
                           end loop;
                           Code.Append (-1);
                           for T of Ps (B).Ids loop
                              Code.Append (T);
                           end loop;
                           for F in 0 .. K - 1 loop
                              Common (F) := F = Ref or else (Ps (A).P (F).Ok and then Ps (B).P (F).Ok);
                              if Common (F) and then F /= Ref then
                                 Nc := Nc + 1;
                              end if;
                           end loop;
                           if Nc > 0 and then not Tested.Contains (Code) then
                              declare
                                 U : Geom.Nat_Vectors.Vector := Ps (A).Ids;
                                 Pu, Pa, Pb : Pose_Arr (0 .. K - 1);
                                 Cu, Ca, Cb : Cov6_Arr (0 .. K - 1);
                                 Cost_U, Cost_A, Cost_B : Long_Float;
                              begin
                                 for T of Ps (B).Ids loop
                                    U.Append (T);
                                 end loop;
                                 Refine (To_Arr (U), Pu, Cu, Cost_U, Common, With_Cov => False);
                                 --  一块定得住位姿的帧全在共同的帧里 ⇒ 只在共同的帧上解它 = 它自己那一解,代价现成
                                 if Ps (A).Cost >= 0.0 and then (for all F in 0 .. K - 1 => Common (F) or else not Ps (A).P (F).Ok) then
                                    Cost_A := Ps (A).Cost;
                                 else
                                    Refine (To_Arr (Ps (A).Ids), Pa, Ca, Cost_A, Common, With_Cov => False);
                                 end if;
                                 if Ps (B).Cost >= 0.0 and then (for all F in 0 .. K - 1 => Common (F) or else not Ps (B).P (F).Ok) then
                                    Cost_B := Ps (B).Cost;
                                 else
                                    Refine (To_Arr (Ps (B).Ids), Pb, Cb, Cost_B, Common, With_Cov => False);
                                 end if;
                                 declare
                                    Zab : constant Long_Float := Z_Of (Cost_U - Cost_A - Cost_B, Pose_N * Nc, Sig_Dof);
                                 begin
                                    if Zab <= Best_Z then
                                       Best_Z := Zab;
                                       Best_A := A;
                                       Best_B := B;
                                    end if;
                                    if Zab > Stats.Z then
                                       Tested.Append (Code);
                                    end if;
                                 end;
                              end;
                           end if;
                        end;
                     end loop;
                  end loop;
                  exit when Best_A < 0;
                  declare
                     W : Work_Piece := Ps (Best_A);
                     Cost : Long_Float;
                  begin
                     W.Ids.Clear;
                     for T in 0 .. N - 1 loop
                        if Ps (Best_A).Ids.Contains (T) or else Ps (Best_B).Ids.Contains (T) then
                           W.Ids.Append (T);
                        end if;
                     end loop;
                     Refine (To_Arr (W.Ids), W.P, W.C6, Cost);
                     W.Cost := Cost;
                     Ps.Replace_Element (Best_A, W);
                     Ps.Delete (Best_B);
                  end;
               end;
            end loop;
         end Merge_Same;
         procedure Assign (Ps : in out WP_Vectors.Vector) is
            History : State_Vectors.Vector;
            Tested : State_Vectors.Vector;
            Width : constant Natural := Role'Pos (Role'Last) + 1;
            function Code_Of return Bytes.Ints is
               C : Bytes.Ints;
            begin
               for T in 0 .. N - 1 loop
                  C.Append (Role'Pos (Rep.Roles (T)) + Width * (Rep.Piece_Of (T) + 1));
               end loop;
               return C;
            end Code_Of;
         begin
            loop
               Merge_Same (Ps, Tested);
               declare
                  New_Ids : array (0 .. Natural'Max (1, Natural (Ps.Length)) - 1) of Geom.Nat_Vectors.Vector;
               begin
                  for T in 0 .. N - 1 loop
                     declare
                        Rl : Role := Unused;
                        Of_P : Integer := -1;
                     begin
                        if Usable (T) then
                           for Pred in Boolean loop
                              declare
                                 Hits : Natural := 0;
                                 Best, Second : Integer := -1;
                                 Best_D2, Second_D2 : Long_Float := Long_Float'Last;
                                 D2 : Long_Float;
                              begin
                                 for Pi in 0 .. Natural (Ps.Length) - 1 loop
                                    if Fits (T, Ps (Pi).P, Ps (Pi).C6, Pred, D2) then
                                       Hits := Hits + 1;
                                       if D2 < Best_D2 then
                                          Second := Best;
                                          Second_D2 := Best_D2;
                                          Best_D2 := D2;
                                          Best := Pi;
                                       elsif D2 < Second_D2 then
                                          Second_D2 := D2;
                                          Second := Pi;
                                       end if;
                                    end if;
                                 end loop;
                                 if Hits > 0 then
                                    --  几块都说得通:说得最好的两块对它的预测隔得开(≥ 2 Z,判错的机会不超过一维单侧 Z 的尾巴;
                                    --  同 Kinem.Classify_Rides)⇒ 归近的;隔不开 ⇒ 分不清
                                    if Hits = 1 or else Sqrt (Sep2 (T, Ps (Best).P, Ps (Second).P)) >= 2.0 * Stats.Z then
                                       Rl := Member;
                                       Of_P := Best;
                                    else
                                       Rl := Ambiguous;
                                       Of_P := -1;
                                    end if;
                                    exit;
                                 end if;
                                 Rl := Unexplained;
                              end;
                           end loop;
                        end if;
                        Rep.Roles.Replace_Element (T, Rl);
                        Rep.Piece_Of.Replace_Element (T, Of_P);
                        if Of_P >= 0 then
                           New_Ids (Of_P).Append (T);
                        end if;
                     end;
                  end loop;
                  declare
                     Same_Ids : Boolean := True;
                  begin
                     for Pi in 0 .. Natural (Ps.Length) - 1 loop
                        Same_Ids := Same_Ids and then Geom.Nat_Vectors."=" (New_Ids (Pi), Ps (Pi).Ids);
                     end loop;
                     exit when Same_Ids;
                     exit when History.Contains (Code_Of);   --  回到了以前的某一轮:在几种归法之间来回,停在这一轮
                     History.Append (Code_Of);
                  end;
                  --  按新的成员重解;不够 3 条的块拿掉(它的成员下一轮重新归)
                  declare
                     Kept : WP_Vectors.Vector;
                  begin
                     for Pi in 0 .. Natural (Ps.Length) - 1 loop
                        if Natural (New_Ids (Pi).Length) >= Dim then
                           declare
                              W : Work_Piece := (Ids => New_Ids (Pi), P => Ps (Pi).P, C6 => Ps (Pi).C6, Cost => -1.0);
                              Cost : Long_Float;
                           begin
                              Refine (To_Arr (W.Ids), W.P, W.C6, Cost);
                              W.Cost := Cost;
                              Kept.Append (W);
                           end;
                        end if;
                     end loop;
                     Ps := Kept;
                  end;
               end;
            end loop;
         end Assign;

         --  量噪声倍数(归好块以后):每个成员每一笔的白化残差,按它的杠杆放大回去 —— 形状是从它自己这几笔解的
         --  (tr ((Σ Rᵀ C⁻¹ R)⁻¹ Rᵀ C⁻¹ R) / 3),这一帧的位姿是这一块在这一帧看见的 n 个成员一起解的(6 个数摊在 3n 个残差上;参照帧不解);
         --  残差天生比噪声小,放大回去才是噪声。三个分量的绝对值取中位 × Kinem.Mad_Sigma。杠杆到 1 的(只看见一帧)残差恒为 0,不进
         procedure Measure_Sig (Ps : WP_Vectors.Vector; S_Out, Dof_Out : out Long_Float) is
            Cnt : Natural := 0;
            Free_Sum : Long_Float := 0.0;   --  Σ (1 − 杠杆):这批残差的自由度
         begin
            for Pc of Ps loop
               for I of Pc.Ids loop
                  for F in 0 .. K - 1 loop
                     if G (I, F).Seen and then Pc.P (F).Ok then
                        Cnt := Cnt + Dim;
                     end if;
                  end loop;
               end loop;
            end loop;
            S_Out := Sig;
            Dof_Out := Sig_Dof;
            if Cnt = 0 then
               return;
            end if;
            declare
               Rs : Vec_Ptr := new Vec (0 .. Cnt - 1);
               J : Natural := 0;
               Md : Long_Float;
            begin
               for Pc of Ps loop
                  declare
                     Seen_N : Nat_Arr (0 .. K - 1) := [others => 0];   --  这一块每一帧看见几个成员
                  begin
                     for I of Pc.Ids loop
                        for F in 0 .. K - 1 loop
                           if G (I, F).Seen and then Pc.P (F).Ok then
                              Seen_N (F) := Seen_N (F) + 1;
                           end if;
                        end loop;
                     end loop;
                     for I of Pc.Ids loop
                        declare
                           S : V3;
                           Nf : Natural;
                           Info : M3;
                           Ok : Boolean;
                           Inv : M3;
                        begin
                           Solve_Shape (I, Pc.P, S, Nf, Info);
                           Inv := Inv3 (Info, Ok);
                           for F in 0 .. K - 1 loop
                              if Ok and then G (I, F).Seen and then Pc.P (F).Ok then
                                 declare
                                    E : constant V3 := Sub (G (I, F).X, Add (Geom.Ap (Pc.P (F).R, S), Pc.P (F).T));
                                    W : constant V3 := Geom.Ap (G (I, F).Wh, E);
                                    Rc : constant M3 := Geom.Mul (Geom.Tr (Pc.P (F).R), G (I, F).Ci);
                                    H : constant M3 := Geom.Mul (Inv, Geom.Mul (Rc, Pc.P (F).R));
                                    Lev : constant Long_Float := (H (0, 0) + H (1, 1) + H (2, 2)) / Long_Float (Dim)
                                                                 + (if F = Ref then 0.0 else Long_Float (Pose_N) / Long_Float (Dim * Seen_N (F)));
                                 begin
                                    if Lev < 1.0 then
                                       for C in V3'Range loop
                                          Rs (J + C) := W (C) / Sqrt (1.0 - Lev);
                                       end loop;
                                       J := J + Dim;
                                       Free_Sum := Free_Sum + Long_Float (Dim) * (1.0 - Lev);
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end;
                     end loop;
                  end;
               end loop;
               if J = 0 then
                  Free (Rs);
                  return;
               end if;
               Md := Median_Of (Rs (0 .. J - 1));
               Free (Rs);
               S_Out := Long_Float'Max (Kinem.Mad_Sigma * Md, Floor_W);
               Dof_Out := Mad_Eff * Free_Sum;
            end;
         end Measure_Sig;

         --  ── 两块之间的一根轴 ──
         --  参数化(一种写法同时装下转和走):B 上的一点 C 相对 A 走的路 = 一段圆弧,起点的方向 D(单位)、曲率向量 k(⊥ D,指向轴,
         --  |k| = 曲率 = 1 / C 离轴多远)、弧长 q(每一帧一个)。轴的方向 W = D × k / |k|,轴上离 C 最近的点 P = C + k / |k|²;
         --  第 f 帧 B 相对 A 的动法 J(x) = R x + (C − R C) + q sinc(θ) D + ½ q² sinc²(θ/2) k,R = 转动向量 q (D × k)(绕 W 转 θ = q |k|)
         --  —— 就是 Kinem.FK 的一根转轴(绕 W 过 P 转 θ);k = 0 时正好是沿 D 走 q(滑轴),而且 k = 0 是参数里普通的一点:
         --  转和走不用先挑一种。k 按垂直于 D 的平面里的两个分量算(不按"多弯 + 轴朝哪"两个数:弯得很少时轴朝哪跟着噪声走,
         --  "多弯"就成了噪声向量的长度,永远是正的、偏大 —— 09-30 离线:真曲率 0.5、噪声 2 的那一场,20 组里 18 组估大、平均大 1 倍噪声)。
         --  参数:A 每一帧的位姿、D(切平面里 2 个数)、k(2 个数)、每一帧的 q;B 每一帧的位姿 = A 的位姿接上 J。
         --  两块所有成员的观测一起按最大似然解(形状消掉),协方差 = 解处 (Jᵀ W J)⁻¹。判法(都是量的):
         --    一根轴比两块各走各的代价多出来的(卡方,自由度 = 参数差 5m − 4)过门 ⇒ Not_One_Axis;
         --    kᵀ Σk⁻¹ k > Gate (2)(C 的路显著地弯)⇒ 转轴;
         --    k 的置信椭圆整个在 |k| < 1 / Reach 里(|k| + √(Gate (2) λmax) < 1 / Reach:连"轴离 C 不比这件东西看得见的点更远"的转轴
         --    都排除了 —— 轴长在东西上,就一定是走)⇒ 滑轴;
         --    都不是 ⇒ Undecided:动得太少,转和走分不出(照实说要是转轴,它至少离 C 多远)
         procedure Fit_Joint (Pa, Pb : Work_Piece; Jt : in out Joint) is
            Fr : Nat_Arr (0 .. K - 1) := [others => 0];
            M : Natural := 0;
         begin
            for F in 0 .. K - 1 loop
               Jt.Q.Append (0.0);
               Jt.Q_Sd.Append ((if F = Ref then 0.0 else Long_Float'Last));
               if F /= Ref and then Pa.P (F).Ok and then Pb.P (F).Ok then
                  Fr (M) := F;
                  M := M + 1;
               end if;
            end loop;
            Jt.Common := M;
            if M = 0 then
               Jt.Status := No_Common_Frame;
               return;
            end if;
            declare
               A_Ids : constant Nat_Arr := To_Arr (Pa.Ids);
               B_Ids : constant Nat_Arr := To_Arr (Pb.Ids);
               Pa_P, Pb_P : Pose_Arr (0 .. K - 1) := [others => No_Pose];
               Ca6, Cb6 : Cov6_Arr (0 .. K - 1);
               Cost_A, Cost_B : Long_Float;
               Rm : array (0 .. M - 1) of M3;
               Tm : array (0 .. M - 1) of V3;
               C : V3 := [others => 0.0];
               D0, W0, U1, U2 : V3;
               Chord : Long_Float := 0.0;
               F_Star : Natural := 0;
               Kappa0 : Long_Float := 0.0;
               --  一个转动阵的转动向量(轴 × 角,角在 [0, π]):经四元数(Kinem.To_Pose,Shepperd 法),转到 π 附近也稳
               function Rot_Of (R : M3) return V3 is
                  Q : constant Plug.Arm_Pose := Kinem.To_Pose (R, [others => 0.0]);
                  Vn : constant Long_Float := Sqrt (Q (4) ** 2 + Q (5) ** 2 + Q (6) ** 2);
               begin
                  if not (Vn > 0.0) then
                     return [others => 0.0];
                  end if;
                  return Scl ([Q (4), Q (5), Q (6)], 2.0 * Arctan (Vn, Q (3)) / Vn);
               end Rot_Of;
            begin
               Pa_P (Ref) := Id_Pose;
               Pb_P (Ref) := Id_Pose;
               for J in 0 .. M - 1 loop
                  Pa_P (Fr (J)) := Pa.P (Fr (J));
                  Pb_P (Fr (J)) := Pb.P (Fr (J));
               end loop;
               Fit_Piece (A_Ids, Pa_P, Ca6, Cost_A);
               Fit_Piece (B_Ids, Pb_P, Cb6, Cost_B);
               for J in 0 .. M - 1 loop
                  Rm (J) := Geom.Mul (Geom.Tr (Pa_P (Fr (J)).R), Pb_P (Fr (J)).R);
                  Tm (J) := Geom.Ap (Geom.Tr (Pa_P (Fr (J)).R), Sub (Pb_P (Fr (J)).T, Pa_P (Fr (J)).T));
               end loop;
               --  C:B 的成员里相对 A 挪得最远的那一点(轴穿过 B 的形心时形心不挪,拿它量不出弧)
               for I of B_Ids loop
                  declare
                     S : V3;
                     Nf : Natural;
                     Info : M3;
                  begin
                     Solve_Shape (I, Pb_P, S, Nf, Info);
                     if Nf > 0 then
                        for J in 0 .. M - 1 loop
                           declare
                              Dl : constant V3 := Sub (Add (Geom.Ap (Rm (J), S), Tm (J)), S);
                           begin
                              if Geom.Norm (Dl) > Chord then
                                 Chord := Geom.Norm (Dl);
                                 C := S;
                                 D0 := Scl (Dl, 1.0 / Geom.Norm (Dl));
                                 F_Star := J;
                              end if;
                           end;
                        end loop;
                     end if;
                  end;
               end loop;
               Jt.C := C;
               if not (Chord > 0.0) then
                  Jt.Status := Undecided;
                  return;
               end if;
               --  起步:B 相对 A 在挪得最远那一帧的转动,扣掉沿 D0 的那一截 ⇒ 轴的方向 W0、曲率 κ0(弦长 = 2 sin(θ/2) / κ);
               --  W0 同时当垂直于 D 的那对轴的参照(D 变的时候按它连续地取)
               declare
                  Om : constant V3 := Rot_Of (Rm (F_Star));
                  Wr : constant V3 := Sub (Om, Scl (D0, Contact.Dot (Om, D0)));
               begin
                  if Geom.Norm (Wr) > 0.0 then
                     W0 := Scl (Wr, 1.0 / Geom.Norm (Wr));
                     Kappa0 := 2.0 * Sin (0.5 * Geom.Norm (Wr)) / Chord;
                  else
                     Perp (D0, W0, U1);
                  end if;
               end;
               Perp (D0, U1, U2);
               declare
                  Iq : constant Natural := Pose_N * M + Geo_N;   --  q 从第几个参数起
                  Ig : constant Natural := Pose_N * M;           --  轴的 4 个数从第几个起
                  Np : constant Natural := Iq + M;
                  X : Vec (0 .. Np - 1) := [others => 0.0];
                  St : Vec (0 .. Np - 1);
                  --  这组参数 ⇒ D(单位)、k(曲率向量)、W(轴的方向,单位;k = 0 时随便一个垂直于 D 的)
                  procedure Axis_Of (Xx : Vec; Dv, Kv, Wv : out V3) is
                     V1, V2 : V3;
                  begin
                     Dv := Unit3 (Add (D0, Add (Scl (U1, Xx (Ig)), Scl (U2, Xx (Ig + 1)))));
                     V1 := Unit3 (Sub (W0, Scl (Dv, Contact.Dot (W0, Dv))));
                     V2 := Contact.Cross (Dv, V1);
                     Kv := Add (Scl (V1, Xx (Ig + Ka)), Scl (V2, Xx (Ig + Ka + 1)));
                     Wv := (if Geom.Norm (Kv) > 0.0 then Unit3 (Contact.Cross (Dv, Kv)) else Contact.Cross (Dv, V1));
                  end Axis_Of;
                  procedure Poses_Of (Xx : Vec; Qa, Qb : out Pose_Arr) is
                     Dv, Kv, Wv : V3;
                  begin
                     Axis_Of (Xx, Dv, Kv, Wv);
                     Qa := Pa_P;
                     Qb := Pa_P;
                     for J in 0 .. M - 1 loop
                        declare
                           B : constant Natural := Pose_N * J;
                           F : constant Natural := Fr (J);
                           Q : constant Long_Float := Xx (Iq + J);
                           Th : constant Long_Float := Q * Geom.Norm (Kv);
                           Rj : constant M3 := Geom.Rodrigues (Scl (Contact.Cross (Dv, Kv), Q));
                           J0 : constant V3 := Add (Sub (C, Geom.Ap (Rj, C)),
                                                    Add (Scl (Dv, Q * Sinc (Th)), Scl (Kv, 0.5 * Q ** 2 * Sinc (0.5 * Th) ** 2)));
                        begin
                           Qa (F).R := Geom.Mul (Geom.Rodrigues (V3 (Xx (B .. B + Dim - 1))), Pa_P (F).R);
                           Qa (F).T := Add (Pa_P (F).T, V3 (Xx (B + Dim .. B + Pose_N - 1)));
                           Qb (F).R := Geom.Mul (Qa (F).R, Rj);
                           Qb (F).T := Add (Geom.Ap (Qa (F).R, J0), Qa (F).T);
                        end;
                     end loop;
                  end Poses_Of;
                  N_R : constant Natural := N_Res (A_Ids, Pa_P) + N_Res (B_Ids, Pa_P);
                  procedure Resid (Xx : Vec; R : out Vec) is
                     Qa, Qb : Pose_Arr (0 .. K - 1);
                     J : Natural := R'First;
                  begin
                     Poses_Of (Xx, Qa, Qb);
                     for I of A_Ids loop
                        Res_Track (I, Qa, R, J);
                     end loop;
                     for I of B_Ids loop
                        Res_Track (I, Qb, R, J);
                     end loop;
                  end Resid;
                  Done : Boolean;
                  R0 : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_R) - 1);
                  H : Mat (0 .. Np - 1, 0 .. Np - 1);
                  Undet : Bool_Arr (0 .. Np - 1) := [others => False];
                  Cost_J : Long_Float;
               begin
                  for J in 0 .. M - 1 loop
                     for Cc in V3'Range loop
                        St (Pose_N * J + Cc) := Hs;
                        St (Pose_N * J + Dim + Cc) := Hs * Scale;
                     end loop;
                     declare
                        Om : constant V3 := Rot_Of (Rm (J));
                     begin
                        X (Iq + J) := (if Kappa0 > 0.0 then Contact.Dot (Om, W0) / Kappa0
                                       else Contact.Dot (Sub (Add (Geom.Ap (Rm (J), C), Tm (J)), C), D0));
                     end;
                     St (Iq + J) := Hs * Scale;
                  end loop;
                  --  k0 = κ0 N0,N0 = W0 × D0 = −(D0 × W0):在 (V1, V2) = (W0, D0 × W0) 里是 (0, −κ0)
                  X (Ig + Ka + 1) := -Kappa0;
                  St (Ig) := Hs;
                  St (Ig + 1) := Hs;
                  St (Ig + Ka) := Hs / Scale;
                  St (Ig + Ka + 1) := Hs / Scale;
                  Kinem.Robust_LM (X, N_R, N_R, Positive'Last, St, Resid'Access, Done);
                  Resid (X, R0 (0 .. N_R - 1));
                  Cost_J := Sum_Sq (R0 (0 .. N_R - 1));
                  Jt.Dof := (Pose_N - 1) * M - Geo_N;
                  Jt.Chi := Long_Float'Max (0.0, Cost_J - Cost_A - Cost_B);
                  Jt.Chi_Gate := Gate_F (Positive (Jt.Dof));   --  M ≥ 1 ⇒ 5M − 4 ≥ 1
                  Param_Cov (X, St, N_R, Resid'Access, H, Undet);
                  --  ── 解出来的量 ──
                  declare
                     Dv, Kv, Wv : V3;
                     Kp : Long_Float;
                     Qa, Qb : Pose_Arr (0 .. K - 1);
                     S11 : constant Long_Float := H (Ig + Ka, Ig + Ka);
                     S12 : constant Long_Float := H (Ig + Ka, Ig + Ka + 1);
                     S22 : constant Long_Float := H (Ig + Ka + 1, Ig + Ka + 1);
                     K_Undet : constant Boolean := Undet (Ig + Ka) or else Undet (Ig + Ka + 1);
                     --  曲率向量 k 的协方差(2×2)最大的特征值、kᵀ Σk⁻¹ k
                     L_Max : constant Long_Float := 0.5 * (S11 + S22) + Sqrt ((0.5 * (S11 - S22)) ** 2 + S12 ** 2);
                     Det : constant Long_Float := S11 * S22 - S12 ** 2;
                     --  轴的几何(D、W、P)对那 4 个数的数值雅可比 ⇒ 它们的协方差(3×3)
                     function Cov_Of (Fn : not null access function (Xx : Vec) return V3; Upto : Natural) return M3 is
                        Base : constant V3 := Fn (X);
                        Jg : array (0 .. 2, 0 .. Geo_N - 1) of Long_Float := [others => [others => 0.0]];
                        Cv : M3 := [others => [others => 0.0]];
                     begin
                        for P in 0 .. Upto - 1 loop
                           declare
                              Xp : Vec := X;
                              V : V3;
                           begin
                              Xp (Ig + P) := Xp (Ig + P) + St (Ig + P);
                              V := Fn (Xp);
                              for R in V3'Range loop
                                 Jg (R, P) := (V (R) - Base (R)) / St (Ig + P);
                              end loop;
                           end;
                        end loop;
                        for R in V3'Range loop
                           for Cc in V3'Range loop
                              for P in 0 .. Upto - 1 loop
                                 for Q in 0 .. Upto - 1 loop
                                    Cv (R, Cc) := Cv (R, Cc) + Jg (R, P) * H (Ig + P, Ig + Q) * Jg (Cc, Q);
                                 end loop;
                              end loop;
                           end loop;
                        end loop;
                        return Cv;
                     end Cov_Of;
                     function D_Fn (Xx : Vec) return V3 is
                        D1, K1, W1 : V3;
                     begin
                        Axis_Of (Xx, D1, K1, W1);
                        return D1;
                     end D_Fn;
                     function W_Fn (Xx : Vec) return V3 is
                        D1, K1, W1 : V3;
                     begin
                        Axis_Of (Xx, D1, K1, W1);
                        return W1;
                     end W_Fn;
                     function P_Fn (Xx : Vec) return V3 is
                        D1, K1, W1 : V3;
                     begin
                        Axis_Of (Xx, D1, K1, W1);
                        return Add (C, Scl (K1, 1.0 / Contact.Dot (K1, K1)));
                     end P_Fn;
                     --  这几条轨迹按这组位姿的形状离 C 最远多远,并进 Jt.Reach
                     procedure Reach_Of (Ids : Nat_Arr; Q : Pose_Arr) is
                        S : V3;
                        Nf : Natural;
                        Info : M3;
                     begin
                        for I of Ids loop
                           Solve_Shape (I, Q, S, Nf, Info);
                           if Nf > 0 then
                              Jt.Reach := Long_Float'Max (Jt.Reach, Geom.Norm (Sub (S, C)));
                           end if;
                        end loop;
                     end Reach_Of;
                     function Any_Undet (From, Upto : Natural) return Boolean is
                     begin
                        for P in From .. Upto - 1 loop
                           if Undet (Ig + P) then
                              return True;
                           end if;
                        end loop;
                        return False;
                     end Any_Undet;
                     Bend, Far : Long_Float := Long_Float'Last;   --  kᵀ Σk⁻¹ k;|k| + √(Gate (2) λmax)(k 的置信椭圆离 0 最远多远)
                  begin
                     Axis_Of (X, Dv, Kv, Wv);
                     Poses_Of (X, Qa, Qb);
                     Kp := Geom.Norm (Kv);
                     Jt.Kappa := Kp;
                     if not K_Undet and then Det > 0.0 then
                        Bend := (S22 * X (Ig + Ka) ** 2 - 2.0 * S12 * X (Ig + Ka) * X (Ig + Ka + 1) + S11 * X (Ig + Ka + 1) ** 2) / Det;
                        Far := Kp + Sqrt (Gate_F (2) * Long_Float'Max (0.0, L_Max));
                        Jt.Kappa_Sd := Sqrt (Long_Float'Max (0.0, L_Max));
                        Jt.Min_Radius := (if Far > 0.0 then 1.0 / Far else 0.0);
                     else
                        Jt.Kappa_Sd := Long_Float'Last;
                        Jt.Min_Radius := 0.0;
                     end if;
                     Jt.Reach := 0.0;
                     Reach_Of (A_Ids, Qa);
                     Reach_Of (B_Ids, Qb);
                     if Jt.Chi > Jt.Chi_Gate then
                        Jt.Status := Not_One_Axis;
                     elsif Bend < Long_Float'Last and then Bend > Gate_F (2) then
                        Jt.Status := Found;
                        Jt.Ax := (W => Wv, P => Add (C, Scl (Kv, 1.0 / Kp ** 2)), Slide => False);
                        if not Any_Undet (0, Geo_N) then
                           Jt.W_Sd := Sqrt (Long_Float'Max (0.0, Max_Eig3 (Cov_Of (W_Fn'Access, Geo_N))));
                           declare
                              Cp : constant M3 := Cov_Of (P_Fn'Access, Geo_N);
                              Pj : M3;
                           begin
                              for R in V3'Range loop
                                 for Cc in V3'Range loop
                                    Pj (R, Cc) := (if R = Cc then 1.0 else 0.0) - Wv (R) * Wv (Cc);
                                 end loop;
                              end loop;
                              Jt.P_Sd := Sqrt (Long_Float'Max (0.0, Max_Eig3 (Geom.Mul (Pj, Geom.Mul (Cp, Pj)))));
                           end;
                        end if;
                     elsif Far < Long_Float'Last and then Far * Jt.Reach < 1.0 then
                        Jt.Status := Found;
                        Jt.Ax := (W => Dv, P => [others => 0.0], Slide => True);
                        if not Any_Undet (0, Ka) then
                           Jt.W_Sd := Sqrt (Long_Float'Max (0.0, Max_Eig3 (Cov_Of (D_Fn'Access, Ka))));
                        end if;
                     else
                        Jt.Status := Undecided;
                     end if;
                     --  每一帧的关节量:转 = θ = q |k|(弧度;按 (k 的两个分量, q) 的协方差传过去),别的 = q(C 走过的弧长)
                     for J in 0 .. M - 1 loop
                        declare
                           Q : constant Long_Float := X (Iq + J);
                           Rot_Ax : constant Boolean := Jt.Status = Found and then not Jt.Ax.Slide;
                           V : Long_Float;
                        begin
                           if Rot_Ax then
                              declare
                                 G1 : constant Long_Float := Q * X (Ig + Ka) / Kp;       --  ∂θ/∂k1
                                 G2 : constant Long_Float := Q * X (Ig + Ka + 1) / Kp;   --  ∂θ/∂k2
                                 G3 : constant Long_Float := Kp;                          --  ∂θ/∂q
                                 I1 : constant Natural := Ig + Ka;
                                 I2 : constant Natural := Ig + Ka + 1;
                                 I3 : constant Natural := Iq + J;
                              begin
                                 Jt.Q.Replace_Element (Fr (J), Kp * Q);
                                 V := G1 * G1 * H (I1, I1) + G2 * G2 * H (I2, I2) + G3 * G3 * H (I3, I3)
                                      + 2.0 * (G1 * G2 * H (I1, I2) + G1 * G3 * H (I1, I3) + G2 * G3 * H (I2, I3));
                              end;
                           else
                              Jt.Q.Replace_Element (Fr (J), Q);
                              V := H (Iq + J, Iq + J);
                           end if;
                           Jt.Q_Sd.Replace_Element (Fr (J), (if Undet (Iq + J) or else (Rot_Ax and then K_Undet) then Long_Float'Last
                                                             else Sqrt (Long_Float'Max (0.0, V))));
                        end;
                     end loop;
                  end;
                  Free (R0);
               end;
            end;
         end Fit_Joint;

         Ps : WP_Vectors.Vector;
         Rounds_Seen : State_Vectors.Vector;   --  每一轮(量噪声倍数 → 归块)归出来的样子
      begin
         --  观测:协方差按下三角做 Cholesky,不是正定的当没看见
         for I in 0 .. N - 1 loop
            for F in 0 .. Natural (Tracks (I).Length) - 1 loop
               declare
                  O : constant Obs := Tracks (I) (F);
                  Cc : M3 := O.Cov;
                  L : M3 := [others => [others => 0.0]];
                  W : M3 := [others => [others => 0.0]];
                  Ok : Boolean := O.Seen;
               begin
                  if Ok then
                     for R in 0 .. 2 loop
                        for Q in 0 .. 2 loop
                           Cc (R, Q) := 0.5 * (O.Cov (R, Q) + O.Cov (Q, R));
                        end loop;
                     end loop;
                     if Cc (0, 0) > 0.0 then
                        L (0, 0) := Sqrt (Cc (0, 0));
                        L (1, 0) := Cc (1, 0) / L (0, 0);
                        L (2, 0) := Cc (2, 0) / L (0, 0);
                        declare
                           D1 : constant Long_Float := Cc (1, 1) - L (1, 0) ** 2;
                        begin
                           if D1 > 0.0 then
                              L (1, 1) := Sqrt (D1);
                              L (2, 1) := (Cc (2, 1) - L (2, 0) * L (1, 0)) / L (1, 1);
                              declare
                                 D2 : constant Long_Float := Cc (2, 2) - L (2, 0) ** 2 - L (2, 1) ** 2;
                              begin
                                 if D2 > 0.0 then
                                    L (2, 2) := Sqrt (D2);
                                 else
                                    Ok := False;
                                 end if;
                              end;
                           else
                              Ok := False;
                           end if;
                        end;
                     else
                        Ok := False;
                     end if;
                     if Ok then
                        W (0, 0) := 1.0 / L (0, 0);
                        W (1, 1) := 1.0 / L (1, 1);
                        W (2, 2) := 1.0 / L (2, 2);
                        W (1, 0) := -L (1, 0) * W (0, 0) / L (1, 1);
                        W (2, 1) := -L (2, 1) * W (1, 1) / L (2, 2);
                        W (2, 0) := -(L (2, 0) * W (0, 0) + L (2, 1) * W (1, 0)) / L (2, 2);
                        G (I, F) := (Seen => True, X => O.X, Cv => Cc, Ci => Geom.Mul (Geom.Tr (W), W), Wh => W, Sd => Sqrt (Max_Eig3 (Cc)));
                     else
                        Rep.Bad_Cov := Rep.Bad_Cov + 1;
                     end if;
                  end if;
               end;
            end loop;
         end loop;
         --  参照帧 = 看见的点最多的那一帧;能用的轨迹 = 至少看见两帧;尺度 = 所有观测离形心的均方根
         declare
            Best : Natural := 0;
            Sum : V3 := [others => 0.0];
            Cnt : Natural := 0;
         begin
            for F in 0 .. K - 1 loop
               declare
                  C : Natural := 0;
               begin
                  for I in 0 .. N - 1 loop
                     if G (I, F).Seen then
                        C := C + 1;
                     end if;
                  end loop;
                  if C > Best then
                     Best := C;
                     Ref := F;
                  end if;
               end;
            end loop;
            for I in 0 .. N - 1 loop
               declare
                  C : Natural := 0;
               begin
                  for F in 0 .. K - 1 loop
                     if G (I, F).Seen then
                        C := C + 1;
                        Cnt := Cnt + 1;
                        Sum := Add (Sum, G (I, F).X);
                        Floor_W := Long_Float'Max (Floor_W, Geom.Norm (Geom.Ap (G (I, F).Wh, G (I, F).X)));
                     end if;
                  end loop;
                  Usable (I) := C >= 2;
               end;
            end loop;
            Floor_W := Long_Float'Model_Epsilon * Floor_W;
            if Cnt > 0 then
               Sum := Scl (Sum, 1.0 / Long_Float (Cnt));
               for I in 0 .. N - 1 loop
                  for F in 0 .. K - 1 loop
                     if G (I, F).Seen then
                        Scale := Scale + Contact.Dot (Sub (G (I, F).X, Sum), Sub (G (I, F).X, Sum));
                     end if;
                  end loop;
               end loop;
               Scale := Sqrt (Scale / Long_Float (Cnt));
            end if;
         end;
         Rep.Ref := Ref;
         Rep.Scale := Scale;
         if not (Scale > 0.0) then
            Free (G);
            return;
         end if;
         --  起步的噪声倍数(还没归块也量得出):同一块上两个点的距离不会变 ⇒ 每个点和它在参照帧最近的那个点,
         --  两点距离在别的帧比参照帧变了多少,全是噪声 —— 按给的协方差投到两点连线上白化(方差 = uᵀ (C_i + C_j) u,两帧各一份),
         --  绝对值取中位 × Mad_Sigma;超过 Z 倍的那几对(跨块的、跟错了的)去掉再量,做到留下的那一批不再变
         --  (截在 Z 倍的正态,中位只小 0.3%,不改)。最近的两点多半在同一块上。每个点只有一对、三帧时每对两个样本 ⇒ 不够准
         --  (有效样本几十个,中位法差 15% 上下),只当起步;归完块按成员的残差重量(Measure_Sig),做到归法不再变
         declare
            Cnt : Natural := 0;
         begin
            for I in 0 .. N - 1 loop
               if G (I, Ref).Seen then
                  Cnt := Cnt + K;
               end if;
            end loop;
            declare
               Zs : Vec_Ptr := new Vec (0 .. Natural'Max (1, Cnt) - 1);
               Nz : Natural := 0;
               function Var_Along (A, B : Cell; U : V3) return Long_Float is
                 (Contact.Dot (U, Geom.Ap (A.Cv, U)) + Contact.Dot (U, Geom.Ap (B.Cv, U)));
            begin
               for I in 0 .. N - 1 loop
                  if G (I, Ref).Seen then
                     declare
                        Nb : Integer := -1;
                        Best : Long_Float := Long_Float'Last;
                     begin
                        for J in 0 .. N - 1 loop
                           if J /= I and then G (J, Ref).Seen then
                              declare
                                 D : constant Long_Float := Geom.Norm (Sub (G (I, Ref).X, G (J, Ref).X));
                              begin
                                 if D < Best then
                                    Best := D;
                                    Nb := J;
                                 end if;
                              end;
                           end if;
                        end loop;
                        if Nb >= 0 and then Best > 0.0 then
                           for F in 0 .. K - 1 loop
                              if F /= Ref and then G (I, F).Seen and then G (Nb, F).Seen then
                                 declare
                                    Dr : constant V3 := Sub (G (I, Ref).X, G (Nb, Ref).X);
                                    Df : constant V3 := Sub (G (I, F).X, G (Nb, F).X);
                                 begin
                                    if Geom.Norm (Df) > 0.0 then
                                       declare
                                          V : constant Long_Float := Var_Along (G (I, Ref), G (Nb, Ref), Unit3 (Dr))
                                                                     + Var_Along (G (I, F), G (Nb, F), Unit3 (Df));
                                       begin
                                          if V > 0.0 then
                                             Zs (Nz) := abs (Geom.Norm (Df) - Geom.Norm (Dr)) / Sqrt (V);
                                             Nz := Nz + 1;
                                          end if;
                                       end;
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
               if Nz > 0 then
                  Sig := Long_Float'Max (Kinem.Mad_Sigma * Median_Of (Zs (0 .. Nz - 1)), Floor_W);
                  declare
                     Kept : Bool_Arr (0 .. Nz - 1) := [others => True];
                     Keep : Vec (0 .. Nz - 1);
                     Rounds : Natural := 0;
                  begin
                     loop
                        declare
                           Nk : Natural := 0;
                           Changed : Boolean := False;
                        begin
                           for Q in 0 .. Nz - 1 loop
                              if Kept (Q) /= (Zs (Q) <= Stats.Z * Sig) then
                                 Kept (Q) := not Kept (Q);
                                 Changed := True;
                              end if;
                              if Kept (Q) then
                                 Keep (Nk) := Zs (Q);
                                 Nk := Nk + 1;
                              end if;
                           end loop;
                           exit when (Rounds > 0 and then not Changed) or else Nk = 0;
                           Sig := Long_Float'Max (Kinem.Mad_Sigma * Median_Of (Keep (0 .. Nk - 1)), Floor_W);
                           Sig_Dof := Mad_Eff * Long_Float (Nk);
                        end;
                        Rounds := Rounds + 1;
                        if Rounds > Nz then
                           Rep.Settled := False;
                           exit;
                        end if;
                     end loop;
                  end;
               end if;
               Free (Zs);
            end;
         end;
         --  归块 ↔ 量噪声倍数,做到归法回到以前出现过的某一轮(包括没变)为止:归法是有限多种,一定会回到以前的某一种
         loop
            Segment (Ps);
            Assign (Ps);
            declare
               Code : Bytes.Ints;
            begin
               for T in 0 .. N - 1 loop
                  Code.Append (Role'Pos (Rep.Roles (T)) + (Role'Pos (Role'Last) + 1) * (Rep.Piece_Of (T) + 1));
               end loop;
               exit when Rounds_Seen.Contains (Code);
               Rounds_Seen.Append (Code);
            end;
            Measure_Sig (Ps, Sig, Sig_Dof);
         end loop;
         Rep.Sigma := Sig;
         Rep.Sigma_Dof := Sig_Dof;
         --  一块:它整块动过没有 = 它的位姿比"每一帧都不动"好得显著(卡方,自由度 6 × 解的帧数)
         if Natural (Ps.Length) = 1 then
            declare
               Ids : constant Nat_Arr := To_Arr (Ps (0).Ids);
               P : constant Pose_Arr := Ps (0).P;
               Still : Pose_Arr := P;
               Nfr : Natural := 0;
               N_R : constant Natural := N_Res (Ids, P);
               Rv : Vec_Ptr := new Vec (0 .. Natural'Max (1, N_R) - 1);
               J : Natural := 0;
               Cost_M, Cost_S : Long_Float;
            begin
               for F in 0 .. K - 1 loop
                  if P (F).Ok then
                     Still (F) := Id_Pose;
                     if F /= Ref then
                        Nfr := Nfr + 1;
                     end if;
                  end if;
               end loop;
               for I of Ids loop
                  Res_Track (I, P, Rv.all, J);
               end loop;
               Cost_M := Sum_Sq (Rv (0 .. N_R - 1));
               J := 0;
               for I of Ids loop
                  Res_Track (I, Still, Rv.all, J);
               end loop;
               Cost_S := Sum_Sq (Rv (0 .. N_R - 1));
               Rep.Moved := Nfr > 0 and then Cost_S - Cost_M > Gate_F (Pose_N * Nfr);
               Free (Rv);
            end;
         end if;
         for Pc of Ps loop
            declare
               Pv : Piece;
            begin
               Pv.Members := Pc.Ids;
               for F in 0 .. K - 1 loop
                  Pv.Poses.Append (Pc.P (F));
               end loop;
               Rep.Pieces.Append (Pv);
            end;
         end loop;
         for A in 0 .. Natural (Ps.Length) - 1 loop
            for B in A + 1 .. Natural (Ps.Length) - 1 loop
               declare
                  Jt : Joint;
               begin
                  Jt.A := A;
                  Jt.B := B;
                  Fit_Joint (Ps (A), Ps (B), Jt);
                  Rep.Joints.Append (Jt);
               end;
            end loop;
         end loop;
         Free (G);
      end;
   end Fit;

   function Say (Rep : Report) return String is
      use Codec;
      S : Unbounded_String;
      Amb, Unex, Unu : Natural := 0;
      Deg : constant Long_Float := 180.0 / Ada.Numerics.Pi;   --  弧度 → 度(日志)
      function Vs (X : V3) return String is ("(" & Fmt (X (0)) & "," & Fmt (X (1)) & "," & Fmt (X (2)) & ")");
   begin
      for R of Rep.Roles loop
         case R is
            when Ambiguous => Amb := Amb + 1;
            when Unexplained => Unex := Unex + 1;
            when Unused => Unu := Unu + 1;
            when Member => null;
         end case;
      end loop;
      Append (S, "部件:" & Img (Natural (Rep.Pieces.Length)) & " 块(");
      for I in 0 .. Natural (Rep.Pieces.Length) - 1 loop
         Append (S, (if I > 0 then " / " else "") & Img (Natural (Rep.Pieces (I).Members.Length)));
      end loop;
      Append (S, " 个点)· 两块都说得通 " & Img (Amb) & " · 哪块都说不通 " & Img (Unex) & " · 看见不到两帧 " & Img (Unu)
              & " · 噪声倍数 " & Fmt (Rep.Sigma, 2) & " · " & Img (Rep.Frames) & " 帧(参照第 " & Img (Rep.Ref) & " 帧)");
      if Natural (Rep.Pieces.Length) = 1 then
         Append (S, (if Rep.Moved then " · 只见过它整块动:当一整块" else " · 没见过它动:当一整块"));
      elsif Rep.Pieces.Is_Empty then
         Append (S, " · 凑不出一块(没有三个显著铺开、一起动的点)");
      end if;
      for J of Rep.Joints loop
         declare
            Mx : Long_Float := 0.0;
         begin
            for Q of J.Q loop
               Mx := Long_Float'Max (Mx, abs Q);
            end loop;
            Append (S, " · 块 " & Img (J.A + 1) & "–块 " & Img (J.B + 1) & ":");
            case J.Status is
               when Found =>
                  if J.Ax.Slide then
                     Append (S, "滑轴 方向 " & Vs (J.Ax.W) & " ± " & Fmt (Deg * J.W_Sd, 1) & "°,走了 " & Fmt (Mx) & " 单位");
                  else
                     Append (S, "转轴 方向 " & Vs (J.Ax.W) & " ± " & Fmt (Deg * J.W_Sd, 1) & "°,过 " & Vs (J.Ax.P) & " ± " & Fmt (J.P_Sd)
                             & " 单位,转了 " & Fmt (Deg * Mx, 1) & "°");
                  end if;
               when Undecided =>
                  Append (S, "定不下(动得太少,转还是走分不出:C 的路曲率 " & Fmt (J.Kappa, 3) & " ± " & Fmt (J.Kappa_Sd, 3)
                          & ",要是转轴,离 C 至少 " & Fmt (J.Min_Radius) & " 单位,这件东西看得见的只铺到 " & Fmt (J.Reach) & ")");
               when Not_One_Axis =>
                  Append (S, "一根轴说不清(比各走各的多出卡方 " & Fmt (J.Chi, 1) & ",门 " & Fmt (J.Chi_Gate, 1) & ")");
               when No_Common_Frame =>
                  Append (S, "除了参照帧没有一帧两块都定得住,量不了");
            end case;
         end;
      end loop;
      if not Rep.Settled then
         Append (S, " · 归法做到保险上限还在变");
      end if;
      return To_String (S);
   end Say;

   function Light_Len (Floor : Long_Float; Thing_Cov : M3) return Long_Float is
   begin
      return Long_Float'Max (Floor, (Sqrt (Gate (Dim)) + Stats.Z) * Sqrt (Long_Float'Max (0.0, Max_Eig3 (Thing_Cov))));
   end Light_Len;

   procedure Follow (Want : V3; Len : Long_Float; Budget : Natural;
                     Step : access procedure (Dir : V3; Len : Long_Float; R : out Step_Report);
                     Goal : access function return Boolean;
                     Rep : out Follow_Report) is
      function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
      function Neg (A : V3) return V3 is ([-A (0), -A (1), -A (2)]);
      Ok : Boolean;
      Wu : constant V3 := Contact.Unit (Want, Ok);
      E1, E2 : V3;
      Probe_N : constant := 5;   --  试的方向:Want、垂直于它的 ±E1、±E2
      Probes : array (0 .. Probe_N - 1) of V3;
      Used : array (0 .. Probe_N - 1) of Boolean := [others => False];
      Cur : Integer := 0;        --  此刻的方向是第几个试的方向(0 = Want 本身;-1 = 正顺着它上一步真挪的方向走)
      Dir : V3;
      --  正顺着它挪的方向走的这一段:它沿 Want 挪的累计、累计的方差(序贯地看顺不顺:显著地往回了就停)
      Run_A, Run_V : Long_Float := 0.0;
      --  沿 Want 走到过的最远处(累计挪的)、从那以后累计的方差:显著地走过了那一处才算新的进展 —— 那时试过的方向才清掉
      --  (试的方向把它往回挪一点、下一个又把它推回去,不算进展;不然到头以后来回打转)。累计按每一步量到的全加(过没过"挪了"的门都加,
      --  方差也每一步都加):只加过了门的,往回那一小步没过门、推回来那一步过了,就像走到了新地方(09-30 离线 20 组种子 2 次)
      Best_A, Best_V : Long_Float := 0.0;
   begin
      Rep := (others => <>);
      if not Ok or else not (Len > 0.0) or else Step = null then
         Rep.How := No_Direction;
         return;
      end if;
      Perp (Wu, E1, E2);
      Probes := [Wu, E1, Neg (E1), E2, Neg (E2)];
      Dir := Wu;
      while Rep.Steps < Budget loop
         declare
            R : Step_Report;
         begin
            Step (Dir, Len, R);
            Rep.Steps := Rep.Steps + 1;
            Rep.Dirs.Append (Dir);
            if not R.Ok then
               Rep.How := Body_Failed;
               return;
            end if;
            declare
               Small : constant Long_Float := Selfmap.Negligible * Len;
               T_Moved : constant Boolean := Geom.Norm (R.Thing) > Small and then Mahal (R.Thing, R.Thing_Cov) > Gate (Dim);
               H_Moved : constant Boolean := Geom.Norm (R.Hand) > Small and then Mahal (R.Hand, R.Hand_Cov) > Gate (Dim);
               Along : constant Long_Float := Contact.Dot (R.Thing, Wu);
               Along_V : constant Long_Float := Long_Float'Max (0.0, Contact.Dot (Wu, Geom.Ap (R.Thing_Cov, Wu)));
               Go_On : Boolean := False;
            begin
               Rep.Moved := Add (Rep.Moved, R.Thing);
               Rep.Along := Rep.Along + Along;
               Best_V := Best_V + Along_V;
               if T_Moved then
                  if Cur = 0 then
                     --  沿 Want 推的:不会自己动的约束只会让它往推的那一侧挪(沿 Want 的不为负),挪了就顺着它挪的方向走
                     Go_On := Along > -Stats.Z * Sqrt (Along_V);
                  elsif Cur > 0 then
                     --  垂直于 Want 试的:它挪的方向顺不顺 Want,要这一步就显著地看得出(垂直地让开不算顺)
                     Go_On := Along > Small and then Along > Stats.Z * Sqrt (Along_V);
                  else
                     Run_A := Run_A + Along;
                     Run_V := Run_V + Along_V;
                     Go_On := Run_A >= -Stats.Z * Sqrt (Run_V);
                  end if;
               end if;
               if Go_On then
                  if Cur >= 0 then
                     Run_A := Along;
                     Run_V := Along_V;
                     Used (Cur) := True;   --  这个试的方向用过了(让了也算;走到新的地方才清)
                  end if;
                  if Rep.Along - Best_A > Stats.Z * Sqrt (Best_V) then
                     --  沿 Want 显著地走过了以前到过的最远处:新的进展,先前试过的方向清掉(再被挡住时从头试)
                     Best_A := Rep.Along;
                     Best_V := 0.0;
                     Used := [others => False];
                     Rep.Tried.Clear;
                     Rep.Yields.Clear;
                  end if;
                  Dir := Contact.Unit (R.Thing, Ok);
                  Cur := -1;
                  if Goal /= null and then Goal.all then
                     Rep.How := Arrived;
                     return;
                  end if;
               else
                  if T_Moved then
                     Rep.Yields.Append (Contact.Unit (R.Thing, Ok));
                  elsif H_Moved and then not R.Blocked then
                     Rep.How := Left_Behind;
                     return;
                  end if;
                  Rep.Tried.Append (Dir);
                  if Cur >= 0 then
                     Used (Cur) := True;
                  end if;
                  Run_A := 0.0;
                  Run_V := 0.0;
                  Cur := -1;
                  for I in Probes'Range loop
                     if not Used (I) then
                        Cur := I;
                        exit;
                     end if;
                  end loop;
                  if Cur < 0 then
                     Rep.How := Stuck;
                     return;
                  end if;
                  Dir := Probes (Cur);
               end if;
            end;
         end;
      end loop;
      Rep.How := Out_Of_Steps;
   end Follow;
end Linkage;
