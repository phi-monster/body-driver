with Ada.Unchecked_Deallocation;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Numerics.Float_Random;
with Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Json;
with Codec;
package body Geom is

   function Quat_To_R (P : Plug.Arm_Pose) return M3 is
      --  读数不一定精确归一(四位小数就够让矩阵不正交):先归一
      Nq : constant Long_Float := Long_Float'Max (1.0e-12, Sqrt (P (3) ** 2 + P (4) ** 2 + P (5) ** 2 + P (6) ** 2));
      W : constant Long_Float := P (3) / Nq; X : constant Long_Float := P (4) / Nq;
      Y : constant Long_Float := P (5) / Nq; Z : constant Long_Float := P (6) / Nq;
   begin
      return [[1.0 - 2.0 * (Y * Y + Z * Z), 2.0 * (X * Y - Z * W), 2.0 * (X * Z + Y * W)],
              [2.0 * (X * Y + Z * W), 1.0 - 2.0 * (X * X + Z * Z), 2.0 * (Y * Z - X * W)],
              [2.0 * (X * Z - Y * W), 2.0 * (Y * Z + X * W), 1.0 - 2.0 * (X * X + Y * Y)]];
   end Quat_To_R;

   function Mul (A, B : M3) return M3 is
      R : M3 := [others => [others => 0.0]];
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            for K in 0 .. 2 loop
               R (I, J) := R (I, J) + A (I, K) * B (K, J);
            end loop;
         end loop;
      end loop;
      return R;
   end Mul;

   function Tr (A : M3) return M3 is
      R : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            R (I, J) := A (J, I);
         end loop;
      end loop;
      return R;
   end Tr;

   function Ap (A : M3; X : V3) return V3 is
      R : V3 := [others => 0.0];
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            R (I) := R (I) + A (I, J) * X (J);
         end loop;
      end loop;
      return R;
   end Ap;

   function Norm (X : V3) return Long_Float is (Sqrt (X (0) ** 2 + X (1) ** 2 + X (2) ** 2));

   function Rodrigues (R : V3) return M3 is
      Th : constant Long_Float := Norm (R);
      K : V3;
      Kx : M3;
      Res : M3 := Identity;
   begin
      if Th < 1.0e-12 then
         return Identity;
      end if;
      K := [R (0) / Th, R (1) / Th, R (2) / Th];
      Kx := [[0.0, -K (2), K (1)], [K (2), 0.0, -K (0)], [-K (1), K (0), 0.0]];
      declare
         Kx2 : constant M3 := Mul (Kx, Kx);
         S : constant Long_Float := Sin (Th);
         Cc : constant Long_Float := 1.0 - Cos (Th);
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Res (I, J) := Res (I, J) + S * Kx (I, J) + Cc * Kx2 (I, J);
            end loop;
         end loop;
      end;
      return Res;
   end Rodrigues;

   function Rot_Vec (A : M3) return V3 is
      C : Long_Float := (A (0, 0) + A (1, 1) + A (2, 2) - 1.0) / 2.0;
      Th : Long_Float;
      S : Long_Float;
   begin
      C := Long_Float'Max (-1.0, Long_Float'Min (1.0, C));
      Th := Arccos (C);
      if Th < 1.0e-9 then
         return [0.0, 0.0, 0.0];
      end if;
      S := 2.0 * Sin (Th);
      return [(A (2, 1) - A (1, 2)) / S * Th, (A (0, 2) - A (2, 0)) / S * Th, (A (1, 0) - A (0, 1)) / S * Th];
   end Rot_Vec;

   function Angle_Between (P, Q : Plug.Arm_Pose) return Long_Float is
      D : Long_Float := abs (P (3) * Q (3) + P (4) * Q (4) + P (5) * Q (5) + P (6) * Q (6));
   begin
      D := Long_Float'Min (1.0, D);
      return 2.0 * Arccos (D);
   end Angle_Between;

   function Cam_R (G : Cam_Geo; P : Plug.Arm_Pose) return M3 is (Mul (Quat_To_R (P), G.R_Ce));
   function Cam_Pos (G : Cam_Geo; P : Plug.Arm_Pose) return V3 is
      Ow : constant V3 := Ap (Quat_To_R (P), G.Off);
   begin
      return [P (0) + Ow (0), P (1) + Ow (1), P (2) + Ow (2)];
   end Cam_Pos;

   --  径向畸变(归一化平面,r² = x² + y²):畸变后 = 理想 × (1 + K1 r² + K2 r⁴)
   procedure Distort (G : Cam_Geo; X, Y : Long_Float; Xd, Yd : out Long_Float) is
      R2 : constant Long_Float := X * X + Y * Y;
      D : constant Long_Float := 1.0 + G.K1 * R2 + G.K2 * R2 * R2;
   begin
      Xd := X * D; Yd := Y * D;
   end Distort;

   --  反过来:从畸变后的点迭代回理想的点(不动点迭代 x = x畸 ÷ (1 + K1 r² + K2 r⁴);畸变不到三成时几步就收敛,50 次封顶)。
   --  迭代到来回差不到 1e-12(归一化坐标,纯数值精度)就停
   procedure Undistort (G : Cam_Geo; Xd, Yd : Long_Float; X, Y : out Long_Float) is
   begin
      X := Xd; Y := Yd;
      if G.K1 = 0.0 and then G.K2 = 0.0 then
         return;
      end if;
      for It in 1 .. 50 loop
         declare
            R2 : constant Long_Float := X * X + Y * Y;
            D : constant Long_Float := 1.0 + G.K1 * R2 + G.K2 * R2 * R2;
            Xn, Yn : Long_Float;
         begin
            exit when D <= 0.0;   --  这么远的地方畸变已经折回来了:停在上一步
            Xn := Xd / D; Yn := Yd / D;
            exit when abs (Xn - X) + abs (Yn - Y) < 1.0e-12;
            X := Xn; Y := Yn;
         end;
      end loop;
   end Undistort;

   function Cam_Dir (G : Cam_Geo; U, V : Long_Float) return V3 is
      X, Y : Long_Float;
   begin
      Undistort (G, (U - G.Cx) / G.F, -(V - G.Cy) / G.F, X, Y);
      declare
         N : constant Long_Float := Sqrt (X * X + Y * Y + 1.0);
      begin
         return [X / N, Y / N, -1.0 / N];
      end;
   end Cam_Dir;

   procedure Cam_Pixel (G : Cam_Geo; Pc : V3; U, V : out Long_Float; In_Front : out Boolean) is
      Z : constant Long_Float := -Pc (2);
      Xd, Yd : Long_Float;
   begin
      In_Front := Z > 1.0e-6;
      if not In_Front then
         U := 0.0; V := 0.0;
         return;
      end if;
      Distort (G, Pc (0) / Z, Pc (1) / Z, Xd, Yd);
      U := G.Cx + G.F * Xd;
      V := G.Cy - G.F * Yd;
   end Cam_Pixel;

   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3 is
      Dc : constant V3 := Cam_Dir (G, U, V);
      Dw : V3 := Ap (Cam_R (G, P), Dc);
      N : constant Long_Float := Norm (Dw);
   begin
      for I in 0 .. 2 loop
         Dw (I) := Dw (I) / N;
      end loop;
      return Dw;
   end Ray;

   --  3×3 线性方程组,列主元消元
   function Solve3 (A : M3; B : V3) return V3 is
      M : M3 := A;
      R : V3 := B;
   begin
      for Col in 0 .. 2 loop
         declare
            Piv : Natural := Col;
         begin
            for Rw in Col + 1 .. 2 loop
               if abs (M (Rw, Col)) > abs (M (Piv, Col)) then
                  Piv := Rw;
               end if;
            end loop;
            if Piv /= Col then
               for J in 0 .. 2 loop
                  declare
                     T : constant Long_Float := M (Col, J);
                  begin
                     M (Col, J) := M (Piv, J); M (Piv, J) := T;
                  end;
               end loop;
               declare
                  T : constant Long_Float := R (Col);
               begin
                  R (Col) := R (Piv); R (Piv) := T;
               end;
            end if;
            if abs (M (Col, Col)) < 1.0e-15 then
               return [0.0, 0.0, 0.0];
            end if;
            for Rw in 0 .. 2 loop
               if Rw /= Col then
                  declare
                     Fct : constant Long_Float := M (Rw, Col) / M (Col, Col);
                  begin
                     for J in 0 .. 2 loop
                        M (Rw, J) := M (Rw, J) - Fct * M (Col, J);
                     end loop;
                     R (Rw) := R (Rw) - Fct * R (Col);
                  end;
               end if;
            end loop;
         end;
      end loop;
      return [R (0) / M (0, 0), R (1) / M (1, 1), R (2) / M (2, 2)];
   end Solve3;

   function Triangulate (G : Cam_Geo; O : Obs_Vectors.Vector) return V3 is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [others => 0.0];
   begin
      for Ob of O loop
         declare
            D : constant V3 := Ray (G, Ob.Pose, Ob.U, Ob.V);
            T : constant V3 := [Ob.Pose (0), Ob.Pose (1), Ob.Pose (2)];
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  declare
                     Pm : constant Long_Float := (if I = J then 1.0 else 0.0) - D (I) * D (J);
                  begin
                     A (I, J) := A (I, J) + Pm;
                     B (I) := B (I) + Pm * T (J);
                  end;
               end loop;
            end loop;
         end;
      end loop;
      return Solve3 (A, B);
   end Triangulate;

   function To_Cam (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3) return V3 is
      Cp : constant V3 := Cam_Pos (G, P);
      D : constant V3 := [Pw (0) - Cp (0), Pw (1) - Cp (1), Pw (2) - Cp (2)];
   begin
      return Ap (Tr (Cam_R (G, P)), D);
   end To_Cam;

   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
   begin
      Cam_Pixel (G, To_Cam (G, P, Pw), U, V, In_Front);
   end Project;

   --  ── 量朝向 ──
   --  ── Levenberg–Marquardt 精修(数值雅可比、正规方程高斯消元)──:参数个数由调用方定(6 = 朝向 + 点/位置;7 = 再加焦距)。
   --  Resid 把每个观测的两个像素差填进 Fill;Steps 是各参数的差分步(弧度 / 米 / 像素,极小量)。
   --  阻尼升降的两个倍数不是门槛、不影响身体动不动,只管这次拟合怎么迭代
   type Param_Vec is array (Natural range <>) of Long_Float;
   --  雅可比和残差放在堆上:标定板一起解时 2 万多条残差 × 20 多个未知数,摆在 8 MB 的栈上不稳(一份雅可比就 4 MB)
   type Big_Mat is array (Natural range <>, Natural range <>) of Long_Float;
   type Big_Mat_Ptr is access Big_Mat;
   procedure Free_Mat is new Ada.Unchecked_Deallocation (Big_Mat, Big_Mat_Ptr);
   type Big_Vec_Ptr is access Param_Vec;
   procedure Free_Vec is new Ada.Unchecked_Deallocation (Param_Vec, Big_Vec_Ptr);
   function New_Vec (N : Natural) return Big_Vec_Ptr is
      V : constant Big_Vec_Ptr := new Param_Vec (0 .. Integer (N) - 1);
   begin
      for I in V'Range loop
         V (I) := 0.0;
      end loop;
      return V;
   end New_Vec;
   procedure LM_Refine (P : in out Param_Vec; N_Obs : Natural; Steps : Param_Vec; Iters : Positive;
                        Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                        Cur : in out Long_Float) is
      Lm_Loosen : constant := 3;
      Lm_Tighten : constant := 10;
      Np : constant Natural := P'Length;
      Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
      Rv : Big_Vec_Ptr := New_Vec (2 * N_Obs);
      Rp : Big_Vec_Ptr := New_Vec (2 * N_Obs);
      J : Big_Mat_Ptr := new Big_Mat (0 .. Integer (2 * N_Obs) - 1, 0 .. Integer (Np) - 1);
      procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
      begin
         Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
      end Fill_R;
   begin
      for It in 1 .. Iters loop
         declare
            R0 : Long_Float;
         begin
            Resid (P, R0, Fill_R'Access);
         end;
         for K in 0 .. Np - 1 loop
            declare
               Pp : Param_Vec := P;
               procedure Fill_P (I : Natural; Du, Dv : Long_Float) is
               begin
                  Rp (2 * I) := Du; Rp (2 * I + 1) := Dv;
               end Fill_P;
               Dummy : Long_Float;
               H : constant Long_Float := Steps (Steps'First + K);
            begin
               Pp (P'First + K) := Pp (P'First + K) + H;
               Resid (Pp, Dummy, Fill_P'Access);
               for I in 0 .. 2 * N_Obs - 1 loop
                  J (I, K) := (Rp (I) - Rv (I)) / H;
               end loop;
            end;
         end loop;
         declare
            A : array (0 .. Np - 1, 0 .. Np - 1) of Long_Float := [others => [others => 0.0]];
            B : array (0 .. Np - 1) of Long_Float := [others => 0.0];
            Dlt : array (0 .. Np - 1) of Long_Float := [others => 0.0];
         begin
            for K in 0 .. Np - 1 loop
               for M in 0 .. Np - 1 loop
                  for I in 0 .. 2 * N_Obs - 1 loop
                     A (K, M) := A (K, M) + J (I, K) * J (I, M);
                  end loop;
               end loop;
               for I in 0 .. 2 * N_Obs - 1 loop
                  B (K) := B (K) - J (I, K) * Rv (I);
               end loop;
            end loop;
            for K in 0 .. Np - 1 loop
               A (K, K) := A (K, K) * (1.0 + Lam) + 1.0e-12;
            end loop;
            for Col in 0 .. Np - 1 loop
               declare
                  Piv : Natural := Col;
               begin
                  for Rw in Col + 1 .. Np - 1 loop
                     if abs (A (Rw, Col)) > abs (A (Piv, Col)) then
                        Piv := Rw;
                     end if;
                  end loop;
                  if Piv /= Col then
                     for M in 0 .. Np - 1 loop
                        declare
                           T : constant Long_Float := A (Col, M);
                        begin
                           A (Col, M) := A (Piv, M); A (Piv, M) := T;
                        end;
                     end loop;
                     declare
                        T : constant Long_Float := B (Col);
                     begin
                        B (Col) := B (Piv); B (Piv) := T;
                     end;
                  end if;
                  if abs (A (Col, Col)) > 1.0e-18 then
                     for Rw in 0 .. Np - 1 loop
                        if Rw /= Col then
                           declare
                              Fct : constant Long_Float := A (Rw, Col) / A (Col, Col);
                           begin
                              for M in 0 .. Np - 1 loop
                                 A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                              end loop;
                              B (Rw) := B (Rw) - Fct * B (Col);
                           end;
                        end if;
                     end loop;
                  end if;
               end;
            end loop;
            for K in 0 .. Np - 1 loop
               Dlt (K) := (if abs (A (K, K)) > 1.0e-18 then B (K) / A (K, K) else 0.0);
            end loop;
            declare
               Pn : Param_Vec := P;
               Cn : Long_Float;
            begin
               for K in 0 .. Np - 1 loop
                  Pn (P'First + K) := Pn (P'First + K) + Dlt (K);
               end loop;
               Resid (Pn, Cn, null);
               if Cn < Cur then
                  P := Pn; Cur := Cn; Lam := Lam / Long_Float (Lm_Loosen);
               else
                  Lam := Lam * Long_Float (Lm_Tighten);
               end if;
            end;
         end;
         exit when Lam > 1.0e6;
      end loop;
      Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
   end LM_Refine;

   --  ── 解完之后每个参数的不确定度 ──:在解处再算一次数值雅可比 J,σ² = 残差平方和 ÷ (方程数 − 未知数),协方差 = σ² (JᵀJ)⁻¹,
   --  Sd = 对角线开方(和参数同单位)。"解不出"从此按它判:不确定度比量本身还大 = 方程分不开这个量,而不是拍一个阈值
   procedure Param_Sd (P : Param_Vec; N_Obs : Natural; Steps : Param_Vec;
                       Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                       Sd : out Param_Vec) is
      Np : constant Natural := P'Length;
      Rv : Big_Vec_Ptr := New_Vec (2 * N_Obs);
      Rp : Big_Vec_Ptr := New_Vec (2 * N_Obs);
      J : Big_Mat_Ptr := new Big_Mat (0 .. Integer (2 * N_Obs) - 1, 0 .. Integer (Np) - 1);
      A : array (0 .. Np - 1, 0 .. 2 * Np - 1) of Long_Float := [others => [others => 0.0]];   --  [JᵀJ | I],高斯-约当求逆
      procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
      begin
         Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
      end Fill_R;
      Sum : Long_Float := 0.0;
      Sigma2 : Long_Float;
      R0 : Long_Float;
      Undet : array (0 .. Np - 1) of Boolean := [others => False];   --  没有信息的参数
   begin
      Sd := [others => 0.0];
      if Np = 0 or else 2 * N_Obs <= Np then
         Sd := [others => Long_Float'Last];   --  方程比未知数还少:什么都定不了
         Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
         return;
      end if;
      Resid (P, R0, Fill_R'Access);
      for I in 0 .. 2 * N_Obs - 1 loop
         Sum := Sum + Rv (I) * Rv (I);
      end loop;
      Sigma2 := Sum / Long_Float (2 * N_Obs - Np);
      for K in 0 .. Np - 1 loop
         declare
            Pp : Param_Vec := P;
            procedure Fill_P (I : Natural; Du, Dv : Long_Float) is
            begin
               Rp (2 * I) := Du; Rp (2 * I + 1) := Dv;
            end Fill_P;
            Dummy : Long_Float;
            H : constant Long_Float := Steps (Steps'First + K);
         begin
            Pp (P'First + K) := Pp (P'First + K) + H;
            Resid (Pp, Dummy, Fill_P'Access);
            for I in 0 .. 2 * N_Obs - 1 loop
               J (I, K) := (Rp (I) - Rv (I)) / H;
            end loop;
         end;
      end loop;
      for K in 0 .. Np - 1 loop
         for M in 0 .. Np - 1 loop
            for I in 0 .. 2 * N_Obs - 1 loop
               A (K, M) := A (K, M) + J (I, K) * J (I, M);
            end loop;
         end loop;
         A (K, Np + K) := 1.0;
      end loop;
      for Col in 0 .. Np - 1 loop
         declare
            Piv : Natural := Col;
         begin
            for Rw in Col + 1 .. Np - 1 loop
               if abs (A (Rw, Col)) > abs (A (Piv, Col)) then
                  Piv := Rw;
               end if;
            end loop;
            if Piv /= Col then
               for M in 0 .. 2 * Np - 1 loop
                  declare
                     T : constant Long_Float := A (Col, M);
                  begin
                     A (Col, M) := A (Piv, M); A (Piv, M) := T;
                  end;
               end loop;
            end if;
            if abs (A (Col, Col)) <= 1.0e-18 then
               --  这一列没有信息(比如一个点的观测全被踢成离群,它的三列全零):这个参数不确定度无穷,别的参数照算
               --  (G1O 2026-09-24 右眼:残差 0.64 px 的好解被"± inf"整个否掉)
               Undet (Col) := True;
            else
               declare
                  D : constant Long_Float := A (Col, Col);
               begin
                  for M in 0 .. 2 * Np - 1 loop
                     A (Col, M) := A (Col, M) / D;
                  end loop;
               end;
               for Rw in 0 .. Np - 1 loop
                  if Rw /= Col and then A (Rw, Col) /= 0.0 then
                     declare
                        Fct : constant Long_Float := A (Rw, Col);
                     begin
                        for M in 0 .. 2 * Np - 1 loop
                           A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                        end loop;
                     end;
                  end if;
               end loop;
            end if;
         end;
      end loop;
      for K in 0 .. Np - 1 loop
         Sd (Sd'First + K) := (if Undet (K) then Long_Float'Last else Sqrt (Long_Float'Max (0.0, Sigma2 * A (K, Np + K))));
      end loop;
      Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
   end Param_Sd;

   procedure Fit (G : in out Cam_Geo; O : Obs_Vectors.Vector; Ok : out Boolean) is
      N : constant Natural := Natural (O.Length);
      Fit_F : constant Boolean := G.F <= 0.0;          --  焦距没给 ⇒ 一起解(官方 RoboDojo 观测就没有内参)
      Np : constant Natural := (if Fit_F then 7 else 6);   --  转向量 3 + 那东西的世界位置 3 (+ 焦距)
      --  仪器给了焦距先验(带不确定度)⇒ 残差多一条,和像素残差一起最小二乘
      Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;
      Nr : constant Natural := N + (if Use_Prior then 1 else 0);   --  残差槽数:每停一个 + 先验一个
      --  焦距的起点:仪器给了就从仪器的值起步;没给时按"画幅宽 = 焦距"起步(约 53° 视场,只是搜索起点,最小二乘会把它改掉)
      F0 : constant Long_Float := (if not Fit_F then G.F elsif Use_Prior then G.F_Prior else 2.0 * G.Cx);
      Gen : Ada.Numerics.Float_Random.Generator;
      Best_Cost : Long_Float := Long_Float'Last;
      Best_P : Param_Vec (0 .. Np - 1) := [others => 0.0];
      Have_Best : Boolean := False;
      function Rnd return Long_Float is (Long_Float (Ada.Numerics.Float_Random.Random (Gen)));
      --  残差:每个观测两个像素差;东西跑到相机后面就给一个大罚(1e3 像素,无量纲哨兵)
      procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
         Gt : Cam_Geo := G;
         Sum : Long_Float := 0.0;
      begin
         Gt.R_Ce := Rodrigues ([P (P'First), P (P'First + 1), P (P'First + 2)]);
         if P'Length > 6 then
            Gt.F := P (P'First + 6);
         end if;
         for I in 0 .. N - 1 loop
            declare
               U, V : Long_Float;
               Front : Boolean;
               Du, Dv : Long_Float;
            begin
               Project (Gt, O (I).Pose, [P (P'First + 3), P (P'First + 4), P (P'First + 5)], U, V, Front);
               if Front and then Gt.F > 0.0 then
                  Du := U - O (I).U; Dv := V - O (I).V;
               else
                  Du := 1.0e3; Dv := 1.0e3;   --  跑到相机后面:远大于画幅的罚(像素数,无量纲哨兵)
               end if;
               Sum := Sum + Du * Du + Dv * Dv;
               if Fill /= null then
                  Fill (I, Du, Dv);
               end if;
            end;
         end loop;
         if Use_Prior then
            declare
               Dp : constant Long_Float := (Gt.F - G.F_Prior) / G.F_Prior_Sd;   --  先验那一条残差(以不确定度为单位,无量纲;像素残差按 1 px 噪声计)
            begin
               Sum := Sum + Dp * Dp;
               if Fill /= null then
                  Fill (N, Dp, 0.0);
               end if;
            end;
         end if;
         R := Sqrt (Sum / Long_Float (Natural'Max (1, N)));
      end Resid;
      function Cost (P : Param_Vec) return Long_Float is
         R : Long_Float;
      begin
         Resid (P, R, null);
         return R;
      end Cost;
   begin
      Ok := False;
      if N < 4 or else F0 <= 0.0 then
         return;
      end if;
      Ada.Numerics.Float_Random.Reset (Gen, 7);
      --  盲搜:随机转向(四元数均匀采样)⇒ 视线交点 ⇒ 残差;东西必须在相机前面(镜像解排掉)
      for Trial in 1 .. 3000 loop
         declare
            Q : Plug.Arm_Pose := [others => 0.0];
            Nq : Long_Float := 0.0;
            Gt : Cam_Geo := G;
            Pw : V3;
            P : Param_Vec (0 .. Np - 1) := [others => 0.0];
            Front_All : Boolean := True;
         begin
            for K in 3 .. 6 loop
               --  Box–Muller 出正态,归一化后在球面上均匀
               declare
                  U1 : constant Long_Float := Long_Float'Max (1.0e-12, Rnd);
                  U2 : constant Long_Float := Rnd;
               begin
                  Q (K) := Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
               end;
               Nq := Nq + Q (K) ** 2;
            end loop;
            Nq := Sqrt (Nq);
            for K in 3 .. 6 loop
               Q (K) := Q (K) / Nq;
            end loop;
            Gt.R_Ce := Quat_To_R (Q);
            Gt.F := F0;
            Pw := Triangulate (Gt, O);
            for I in 0 .. N - 1 loop
               declare
                  Pc : constant V3 := To_Cam (Gt, O (I).Pose, Pw);
               begin
                  if -Pc (2) <= 0.0 then
                     Front_All := False;
                  end if;
               end;
            end loop;
            if Front_All then
               declare
                  Rv : constant V3 := Rot_Vec (Gt.R_Ce);
               begin
                  P (0) := Rv (0); P (1) := Rv (1); P (2) := Rv (2); P (3) := Pw (0); P (4) := Pw (1); P (5) := Pw (2);
                  if Fit_F then
                     P (6) := F0;
                  end if;
                  declare
                     C : constant Long_Float := Cost (P);
                  begin
                     if C < Best_Cost then
                        Best_Cost := C; Best_P := P; Have_Best := True;
                     end if;
                  end;
               end;
            end if;
         end;
      end loop;
      if not Have_Best then
         return;
      end if;
      declare
         P : Param_Vec (0 .. Np - 1) := Best_P;
         Cur : Long_Float := Best_Cost;
         Steps : constant Param_Vec (0 .. 6) := [1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0];   --  差分步(弧度 / 米 / 像素,极小量)
      begin
         LM_Refine (P, Nr, Steps (0 .. Np - 1), 40, Resid'Access, Cur);
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         if Fit_F then
            G.F := P (6);
         end if;
         G.F_Meas := (if Fit_F then P (6) else 0.0);
         G.Rms := Cur;
         G.Valid := True;
         Ok := True;
      end;
   end Fit;

   --  前 K 个数的中位数(拷一份排序;标定的观测最多几百笔)
   function Median (Xs : Param_Vec; K : Natural) return Long_Float is
      A : Param_Vec (0 .. Natural'Max (0, K - 1)) := [others => 0.0];
   begin
      if K = 0 then
         return 0.0;
      end if;
      for I in 0 .. K - 1 loop
         A (I) := Xs (Xs'First + I);
      end loop;
      for I in 1 .. K - 1 loop   --  插入排序
         declare
            X : constant Long_Float := A (I);
            J : Integer := I - 1;
         begin
            while J >= 0 and then A (J) > X loop
               A (J + 1) := A (J); J := J - 1;
            end loop;
            A (J + 1) := X;
         end;
      end loop;
      return A (K / 2);
   end Median;

   procedure Fit_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; N_Pts : Natural; Ok : out Boolean; Used : out Natural) is
      Fit_F : constant Boolean := G.F <= 0.0;
      Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;
      N : constant Natural := Natural (O.Length);
      Min_Seen : constant := 3;   --  一个点至少在几停里看见才进(次数)
      Cnt : array (0 .. N_Pts) of Natural := [others => 0];
      Keep : array (0 .. N_Pts) of Boolean := [others => False];
      Slot : array (0 .. N_Pts) of Integer := [others => -1];
      Pw0 : array (0 .. N_Pts) of V3 := [others => [others => 0.0]];
      Gi : Cam_Geo := G;
      Best_Pt : Integer := -1;
      Nk : Natural := 0;
   begin
      Ok := False; Used := 0;
      Why := Ada.Strings.Unbounded.To_Unbounded_String ("观测不到 4 笔");
      if N_Pts = 0 or else N < 4 then
         return;
      end if;
      for Ob of O loop
         if Ob.Pt < N_Pts then
            Cnt (Ob.Pt) := Cnt (Ob.Pt) + 1;
         end if;
      end loop;
      --  起点:看见最多次的那个点单独解一遍(盲搜 + 精修),给朝向和焦距一个像样的起点
      for K in 0 .. N_Pts - 1 loop
         if Cnt (K) >= 4 and then (Best_Pt < 0 or else Cnt (K) > Cnt (Best_Pt)) then
            Best_Pt := K;
         end if;
      end loop;
      if Best_Pt < 0 then
         Why := Ada.Strings.Unbounded.To_Unbounded_String ("没有一个点在 4 停以上都看见");
         return;
      end if;
      declare
         Sub : Obs_Vectors.Vector;
         Fok : Boolean;
      begin
         for Ob of O loop
            if Ob.Pt = Best_Pt then
               Sub.Append (Obs'(Pose => Ob.Pose, U => Ob.U, V => Ob.V));
            end if;
         end loop;
         Gi.Off := [others => 0.0];
         Fit (Gi, Sub, Fok);
         if not Fok then
            Why := Ada.Strings.Unbounded.To_Unbounded_String ("起点那一个点单独解不出(" & Codec.Img (Natural (Sub.Length)) & " 停)");
            return;
         end if;
      end;
      --  其余的点按这个起点三角化;在相机后面的不要
      for K in 0 .. N_Pts - 1 loop
         if Cnt (K) >= Min_Seen then
            declare
               Sub : Obs_Vectors.Vector;
               Front : Boolean := True;
            begin
               for Ob of O loop
                  if Ob.Pt = K then
                     Sub.Append (Obs'(Pose => Ob.Pose, U => Ob.U, V => Ob.V));
                  end if;
               end loop;
               Pw0 (K) := Triangulate (Gi, Sub);
               for Ob of Sub loop
                  declare
                     Pc : constant V3 := To_Cam (Gi, Ob.Pose, Pw0 (K));
                  begin
                     if -Pc (2) <= 0.0 then
                        Front := False;
                     end if;
                  end;
               end loop;
               if Front then
                  Keep (K) := True; Slot (K) := Integer (Nk); Nk := Nk + 1;
               end if;
            end;
         end if;
      end loop;
      if Nk = 0 then
         Why := Ada.Strings.Unbounded.To_Unbounded_String ("按起点三角化后没有一个点在相机前面");
         return;
      end if;
      declare
         Base : constant Natural := (if Fit_F then 7 else 6);   --  转向量 3 + 偏移 3 (+ 焦距)
         Np : constant Natural := Base + 3 * Nk;
         P : Param_Vec (0 .. Np - 1) := [others => 0.0];
         Steps : Param_Vec (0 .. Np - 1) := [others => 1.0e-4];   --  差分步(弧度 / 米,极小量)
         Rv : constant V3 := Rot_Vec (Gi.R_Ce);
         Nr : Natural := 0;
         Cur : Long_Float := 0.0;
         Behind : Natural := 0;   --  最近一次算残差时跑到相机后面的观测数
         Skip : array (0 .. N - 1) of Boolean := [others => False];   --  被判离群、不再进解的观测(按观测序号)
         procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
            Gt : Cam_Geo := G;
            Sum : Long_Float := 0.0;
            I : Natural := 0;
         begin
            Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
            Gt.Off := [P (3), P (4), P (5)];
            if Fit_F then
               Gt.F := P (6);
            end if;
            Behind := 0;
            for J in 0 .. N - 1 loop
               declare
                  Ob : constant Obs_Pt := O (J);
               begin
               if Ob.Pt < N_Pts and then Keep (Ob.Pt) and then not Skip (J) then
                  declare
                     B : constant Natural := Base + 3 * Natural (Slot (Ob.Pt));
                     U, V, Du, Dv : Long_Float;
                     Front : Boolean;
                  begin
                     Project (Gt, Ob.Pose, [P (B), P (B + 1), P (B + 2)], U, V, Front);
                     if Front and then Gt.F > 0.0 then
                        Du := U - Ob.U; Dv := V - Ob.V;
                     else
                        Du := 1.0e3; Dv := 1.0e3;   --  跑到相机后面:远大于画幅的罚(像素数,无量纲哨兵)
                        Behind := Behind + 1;
                     end if;
                     Sum := Sum + Du * Du + Dv * Dv;
                     if Fill /= null then
                        Fill (I, Du, Dv);
                     end if;
                     I := I + 1;
                  end;
               end if;
               end;
            end loop;
            if Use_Prior then
               declare
                  Dp : constant Long_Float := (Gt.F - G.F_Prior) / G.F_Prior_Sd;   --  先验那一条残差(以不确定度为单位,无量纲)
               begin
                  Sum := Sum + Dp * Dp;
                  if Fill /= null then
                     Fill (I, Dp, 0.0);
                  end if;
                  I := I + 1;
               end;
            end if;
            R := Sqrt (Sum / Long_Float (Natural'Max (1, I)));
         end Resid;
      begin
         P (0) := Rv (0); P (1) := Rv (1); P (2) := Rv (2);
         if Fit_F then
            P (6) := Gi.F; Steps (6) := 1.0;   --  焦距的差分步(像素,极小量)
         end if;
         for K in 0 .. N_Pts - 1 loop
            if Keep (K) then
               declare
                  B : constant Natural := Base + 3 * Natural (Slot (K));
               begin
                  P (B) := Pw0 (K) (0); P (B + 1) := Pw0 (K) (1); P (B + 2) := Pw0 (K) (2);
               end;
            end if;
         end loop;
         for Ob of O loop
            if Ob.Pt < N_Pts and then Keep (Ob.Pt) then
               Nr := Nr + 1;
            end if;
         end loop;
         if Use_Prior then
            Nr := Nr + 1;
         end if;
         Resid (P, Cur, null);
         LM_Refine (P, Nr, Steps, 40, Resid'Access, Cur);
         --  跟错的观测(仪器说看见、其实认错了)会把焦距带偏(V1F 右眼:残差 8.4 px、焦距 446 / 真 397):
         --  每笔观测的残差比中位数大 3 倍(比例,无量纲)的踢出去,再解一遍
         declare
            Rs : Param_Vec (0 .. Natural'Max (0, Nr - 1)) := [others => 0.0];
            procedure Grab (I : Natural; Du, Dv : Long_Float) is
            begin
               if I < Nr then
                  Rs (I) := Sqrt (Du * Du + Dv * Dv);
               end if;
            end Grab;
            Med : Long_Float := 0.0;
            Dropped : Natural := 0;
            Rtmp : Long_Float;
         begin
            Resid (P, Rtmp, Grab'Access);
            Med := Median (Rs, Nr - (if Use_Prior then 1 else 0));
            if Med > 0.0 then
               declare
                  I : Natural := 0;
               begin
                  for J in 0 .. N - 1 loop
                     if O (J).Pt < N_Pts and then Keep (O (J).Pt) and then not Skip (J) then
                        if Rs (I) > 3.0 * Med then
                           Skip (J) := True;
                           Dropped := Dropped + 1;
                        end if;
                        I := I + 1;
                     end if;
                  end loop;
               end;
               if Dropped > 0 and then Dropped * 4 < Nr then   --  踢掉的不到四分之一才算离群,再多就是整体不对(比例,无量纲)
                  Nr := Nr - Dropped;
                  Resid (P, Cur, null);
                  LM_Refine (P, Nr, Steps, 40, Resid'Access, Cur);
               else
                  for K in Skip'Range loop
                     Skip (K) := False;
                  end loop;
               end if;
               G.Dropped := Dropped;
            end if;
         end;
         Resid (P, Cur, null);
         if Behind > 0 then
            Why := Ada.Strings.Unbounded.To_Unbounded_String ("解出来还有 " & Codec.Img (Behind) & " 笔观测的点跑到相机后面(残差 " & Codec.Fmt (Cur, 2) & " px)");
            return;   --  解出来还有点跑到相机后面 ⇒ 不是解,不存
         end if;
         declare
            Sd : Param_Vec (0 .. Np - 1);
         begin
            Param_Sd (P, Nr, Steps, Resid'Access, Sd);
            G.Rot_Sd := Sqrt (Sd (0) ** 2 + Sd (1) ** 2 + Sd (2) ** 2);
            G.Off_Sd := Sqrt (Sd (3) ** 2 + Sd (4) ** 2 + Sd (5) ** 2);
            G.F_Sd := (if Fit_F then Sd (6) else 0.0);
            --  不确定度比量本身还大 = 方程分不开它(横着挪、不转:焦距和远近绑着)⇒ 不算解出来
            if (Fit_F and then G.F_Sd >= P (6)) or else G.Rot_Sd >= 1.0 then   --  朝向的不确定度 ≥ 1 弧度 = 根本没定(无量纲)
               Why := Ada.Strings.Unbounded.To_Unbounded_String ("不确定度比量本身还大:焦距 " & Codec.Fmt (P (Base - 1), 1) & " ± " & Codec.Fmt (G.F_Sd, 1)
                                                                & " px,朝向 ± " & Codec.Fmt (G.Rot_Sd, 3) & " rad,偏移 ± " & Codec.Fmt (G.Off_Sd, 3) & " 单位(残差 "
                                                                & Codec.Fmt (Cur, 2) & " px," & Codec.Img (Nk) & " 点," & Codec.Img (Nr) & " 笔)");
               return;
            end if;
            --  这套几何是无畸变针孔:焦距短到半幅宽 ÷ 焦距 > tan 60°(视场 > 120°)时针孔假设本身不成立,
            --  这样的"解"是拟合把错数据凑平的结果(G1M 2026-09-24 左眼:5 个点解出 47.8 px),不存
            if Fit_F and then G.Cx > 1.732 * P (6) then
               Why := Ada.Strings.Unbounded.To_Unbounded_String ("焦距解成 " & Codec.Fmt (P (6), 1) & " px,视场超过 120°,针孔假设不成立(残差 " & Codec.Fmt (Cur, 2) & " px)");
               return;
            end if;
         end;
         Why := Ada.Strings.Unbounded.Null_Unbounded_String;
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         G.Off := [P (3), P (4), P (5)];
         if Fit_F then
            G.F := P (6);
         end if;
         G.F_Meas := (if Fit_F then P (6) else 0.0);
         G.Rms := Cur;
         G.Valid := True;
         Ok := True;
         Used := Nk;
      end;
   end Fit_Rig;

   --  ── 不动的眼 ──
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float) return V3 is
      Dc : constant V3 := Cam_Dir (G, U, V);
      Dw : V3 := Ap (G.R_Ce, Dc);
      N : constant Long_Float := Norm (Dw);
   begin
      for I in 0 .. 2 loop
         Dw (I) := Dw (I) / N;
      end loop;
      return Dw;
   end Ray_Fixed;

   procedure Project_Fixed (G : Cam_Geo; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
   begin
      Cam_Pixel (G, Ap (Tr (G.R_Ce), [Pw (0) - G.Pos (0), Pw (1) - G.Pos (1), Pw (2) - G.Pos (2)]), U, V, In_Front);
   end Project_Fixed;

   procedure Fit_Tip_Scale (Fixed, Hand : Cam_Geo; Dir_C : V3; O : Obs_Vectors.Vector; S, Rms_Px : out Long_Float; Ok : out Boolean) is
      Num, Den, Sum : Long_Float := 0.0;
      N : Natural := 0;
      function Cross (A, B : V3) return V3 is
        ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
   begin
      S := 0.0; Rms_Px := 0.0; Ok := False;
      if not Fixed.Fixed or else Fixed.F <= 0.0 or else Norm (Dir_C) <= 0.0 then
         return;
      end if;
      for Ob of O loop
         declare
            Pos : constant V3 := [Ob.Pose (0), Ob.Pose (1), Ob.Pose (2)];
            W : constant V3 := Ap (Cam_R (Hand, Ob.Pose), Dir_C);        --  指尖那条视线在世界里的方向
            D : constant V3 := Ray_Fixed (Fixed, Ob.U, Ob.V);            --  不动的眼看指尖的那条视线
            A : constant V3 := Cross (W, D);
            B : constant V3 := Cross ([Fixed.Pos (0) - Pos (0), Fixed.Pos (1) - Pos (1), Fixed.Pos (2) - Pos (2)], D);
         begin
            --  (Pos + S·W − O) × D = 0  ⇒  S · (W × D) = (O − Pos) × D
            Num := Num + A (0) * B (0) + A (1) * B (1) + A (2) * B (2);
            Den := Den + A (0) * A (0) + A (1) * A (1) + A (2) * A (2);
            N := N + 1;
         end;
      end loop;
      if N = 0 or else Den <= 1.0e-12 then
         return;
      end if;
      S := Num / Den;
      if S <= 0.0 then
         return;   --  指尖跑到相机背后:两只眼的说法对不上
      end if;
      for Ob of O loop
         declare
            Pos : constant V3 := [Ob.Pose (0), Ob.Pose (1), Ob.Pose (2)];
            W : constant V3 := Ap (Cam_R (Hand, Ob.Pose), Dir_C);
            Pw : constant V3 := [Pos (0) + S * W (0), Pos (1) + S * W (1), Pos (2) + S * W (2)];
            U, V : Long_Float;
            Front : Boolean;
         begin
            Project_Fixed (Fixed, Pw, U, V, Front);
            if Front then
               Sum := Sum + (U - Ob.U) ** 2 + (V - Ob.V) ** 2;
            else
               return;
            end if;
         end;
      end loop;
      Rms_Px := Sqrt (Sum / Long_Float (N));
      Ok := True;
   end Fit_Tip_Scale;

   function Hit_Plane (Origin, Dir, P0, N : V3; Ok : out Boolean) return V3 is
      Den : constant Long_Float := Dir (0) * N (0) + Dir (1) * N (1) + Dir (2) * N (2);
      Num : constant Long_Float := (P0 (0) - Origin (0)) * N (0) + (P0 (1) - Origin (1)) * N (1) + (P0 (2) - Origin (2)) * N (2);
   begin
      Ok := False;
      if abs Den < 1.0e-12 then
         return Origin;
      end if;
      declare
         T : constant Long_Float := Num / Den;
      begin
         if T <= 0.0 then
            return Origin;
         end if;
         Ok := True;
         return [Origin (0) + T * Dir (0), Origin (1) + T * Dir (1), Origin (2) + T * Dir (2)];
      end;
   end Hit_Plane;

   --  给定朝向,相机位置有闭式最小二乘解:每条视线都该穿过它看见的那个点 ⇒ Σ(I − ddᵀ)(P − c) = 0
   function Pos_For (R : M3; G : Cam_Geo; O : Mark_Vectors.Vector) return V3 is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [others => 0.0];
      Gt : Cam_Geo := G;
   begin
      Gt.R_Ce := R;
      for Ob of O loop
         declare
            D : constant V3 := Ray_Fixed (Gt, Ob.U, Ob.V);
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  declare
                     Pm : constant Long_Float := (if I = J then 1.0 else 0.0) - D (I) * D (J);
                  begin
                     A (I, J) := A (I, J) + Pm;
                     B (I) := B (I) + Pm * Ob.Pw (J);
                  end;
               end loop;
            end loop;
         end;
      end loop;
      return Solve3 (A, B);
   end Pos_For;

   procedure Fit_Fixed (G : in out Cam_Geo; O : Mark_Vectors.Vector; Ok : out Boolean) is
      N : constant Natural := Natural (O.Length);
      Fit_F : constant Boolean := G.F <= 0.0;          --  焦距没给 ⇒ 一起解
      Np : constant Natural := (if Fit_F then 7 else 6);   --  转向量 3 + 相机位置 3 (+ 焦距)
      Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;   --  仪器的焦距先验,同 Fit
      Nr : constant Natural := N + (if Use_Prior then 1 else 0);   --  残差槽数:每停一个 + 先验一个
      F0 : constant Long_Float := (if not Fit_F then G.F elsif Use_Prior then G.F_Prior else 2.0 * G.Cx);
      Gen : Ada.Numerics.Float_Random.Generator;
      Best_Cost : Long_Float := Long_Float'Last;
      Best_P : Param_Vec (0 .. Np - 1) := [others => 0.0];
      Have_Best : Boolean := False;
      function Rnd return Long_Float is (Long_Float (Ada.Numerics.Float_Random.Random (Gen)));
      procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
         Gt : Cam_Geo := G;
         Sum : Long_Float := 0.0;
      begin
         Gt.R_Ce := Rodrigues ([P (P'First), P (P'First + 1), P (P'First + 2)]);
         Gt.Pos := [P (P'First + 3), P (P'First + 4), P (P'First + 5)];
         if P'Length > 6 then
            Gt.F := P (P'First + 6);
         end if;
         for I in 0 .. N - 1 loop
            declare
               U, V, Du, Dv : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (Gt, O (I).Pw, U, V, Front);
               if Front and then Gt.F > 0.0 then
                  Du := U - O (I).U; Dv := V - O (I).V;
               else
                  Du := 1.0e3; Dv := 1.0e3;   --  跑到相机后面:远大于画幅的罚(像素数,无量纲哨兵)
               end if;
               Sum := Sum + Du * Du + Dv * Dv;
               if Fill /= null then
                  Fill (I, Du, Dv);
               end if;
            end;
         end loop;
         if Use_Prior then
            declare
               Dp : constant Long_Float := (Gt.F - G.F_Prior) / G.F_Prior_Sd;   --  先验那一条残差(以不确定度为单位,无量纲;像素残差按 1 px 噪声计)
            begin
               Sum := Sum + Dp * Dp;
               if Fill /= null then
                  Fill (N, Dp, 0.0);
               end if;
            end;
         end if;
         R := Sqrt (Sum / Long_Float (Natural'Max (1, N)));
      end Resid;
      function Cost (P : Param_Vec) return Long_Float is
         R : Long_Float;
      begin
         Resid (P, R, null);
         return R;
      end Cost;
   begin
      Ok := False;
      if N < 4 or else F0 <= 0.0 then
         return;
      end if;
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      for Trial in 1 .. 3000 loop
         declare
            Q : Plug.Arm_Pose := [others => 0.0];
            Nq : Long_Float := 0.0;
            R : M3;
            Ps : V3;
            Gt : Cam_Geo := G;
            P : Param_Vec (0 .. Np - 1) := [others => 0.0];
         begin
            for K in 3 .. 6 loop
               declare
                  U1 : constant Long_Float := Long_Float'Max (1.0e-12, Rnd);
                  U2 : constant Long_Float := Rnd;
               begin
                  Q (K) := Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
               end;
               Nq := Nq + Q (K) ** 2;
            end loop;
            Nq := Sqrt (Nq);
            for K in 3 .. 6 loop
               Q (K) := Q (K) / Nq;
            end loop;
            R := Quat_To_R (Q);
            Gt.F := F0;
            Ps := Pos_For (R, Gt, O);
            declare
               Rv : constant V3 := Rot_Vec (R);
               C : Long_Float;
            begin
               P (0) := Rv (0); P (1) := Rv (1); P (2) := Rv (2); P (3) := Ps (0); P (4) := Ps (1); P (5) := Ps (2);
               if Fit_F then
                  P (6) := F0;
               end if;
               C := Cost (P);
               if C < Best_Cost then
                  Best_Cost := C; Best_P := P; Have_Best := True;
               end if;
            end;
         end;
      end loop;
      if not Have_Best then
         return;
      end if;
      declare
         P : Param_Vec (0 .. Np - 1) := Best_P;
         Cur : Long_Float := Best_Cost;
         Steps : constant Param_Vec (0 .. 6) := [1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4, 1.0];   --  差分步(弧度 / 米 / 像素,极小量)
      begin
         LM_Refine (P, Nr, Steps (0 .. Np - 1), 60, Resid'Access, Cur);
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         G.Pos := [P (3), P (4), P (5)];
         if Fit_F then
            G.F := P (6);
         end if;
         G.F_Meas := (if Fit_F then P (6) else 0.0);
         G.Rms := Cur;
         G.Fixed := True;
         G.Valid := True;
         Ok := True;
      end;
   end Fit_Fixed;

   --  板上一个点在不动的眼里的每轴像素方差:配点噪声² + 三角的协方差投进这只眼(雅可比 ∂(u,v)/∂Pw;两轴取平均,系数无量纲)
   function Scene_Var (G : Cam_Geo; S : Scene_Pt) return Long_Float is
      Rt : constant M3 := Tr (G.R_Ce);
      Pc : constant V3 := Ap (Rt, [S.Pw (0) - G.Pos (0), S.Pw (1) - G.Pos (1), S.Pw (2) - G.Pos (2)]);
      Z : constant Long_Float := -Pc (2);
      Ju, Jv : V3 := [others => 0.0];
      function Quad (X : V3) return Long_Float is
         Sum : Long_Float := 0.0;
      begin
         for I in 0 .. 2 loop
            for K in 0 .. 2 loop
               Sum := Sum + X (I) * S.Cov (I, K) * X (K);
            end loop;
         end loop;
         return Sum;
      end Quad;
   begin
      if Z <= 0.0 or else G.F <= 0.0 then
         return S.Sh * S.Sh;
      end if;
      declare
         Du : constant V3 := [G.F / Z, 0.0, G.F * Pc (0) / (Z * Z)];
         Dv : constant V3 := [0.0, -G.F / Z, -G.F * Pc (1) / (Z * Z)];
      begin
         for K in 0 .. 2 loop
            Ju (K) := Du (0) * Rt (0, K) + Du (1) * Rt (1, K) + Du (2) * Rt (2, K);
            Jv (K) := Dv (0) * Rt (0, K) + Dv (1) * Rt (1, K) + Dv (2) * Rt (2, K);
         end loop;
      end;
      return S.Sh * S.Sh + 0.5 * (Quad (Ju) + Quad (Jv));
   end Scene_Var;

   procedure Fit_Fixed_Board (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Rep : in out Fixed_Report; Ok : out Boolean; Start_Here : Boolean := False) is
      Fit_F : constant Boolean := G.F <= 0.0;
      Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;
      Ns : constant Natural := Natural (Scene.Length);
      Min_Pts : constant := 4;   --  单点法至少几个点(次数)
      Gi : Cam_Geo := G;
      Ws : array (0 .. Natural'Max (1, Ns) - 1) of Long_Float := [others => 1.0];   --  每个点的权 = 1 / 它在这只眼里的每轴像素噪声
      Skip : array (0 .. Natural'Max (1, Ns) - 1) of Boolean := [others => False];  --  被判离群、不再进解的点
      Behind : Natural := 0;   --  最近一次算残差时跑到相机后面的点数
   begin
      Ok := False;
      Rep.Scene_N := Ns; Rep.Scene_Used := 0; Rep.Scene_Rms := 0.0;
      if Ns < Min_Pts then
         Why := To_Unbounded_String ("标定板不到 4 个点(" & Codec.Img (Ns) & ")");
         return;
      end if;
      --  ① 起点:单点法(盲搜 + 精修),板上的点世界位置已知;Start_Here 就从 G 现在的位姿起步
      if Start_Here then
         if not (G.Valid and then G.Fixed) then
            Why := To_Unbounded_String ("要从现在的位姿起步,可它还没有位姿");
            return;
         end if;
      else
         declare
            Marks : Mark_Vectors.Vector;
            Fok : Boolean;
         begin
            for S of Scene loop
               Marks.Append (Mark'(Pw => S.Pw, U => S.U, V => S.V));
            end loop;
            Fit_Fixed (Gi, Marks, Fok);
            if not Fok then
               Why := To_Unbounded_String ("标定板的点单独解不出(" & Codec.Img (Ns) & " 个)");
               return;
            end if;
         end;
      end if;
      --  ② 权:按起点处的相机算每个点在这只眼里的每轴像素方差(配点噪声² + 三角的协方差投进来)
      for I in 0 .. Ns - 1 loop
         declare
            Var : constant Long_Float := Scene_Var (Gi, Scene (I));
         begin
            if Var > 0.0 then
               Ws (I) := 1.0 / Sqrt (Var);
            end if;
         end;
      end loop;
      --  ③ 加权精修:朝向 3 + 位置 3 (+ 焦距)
      declare
         Np : constant Natural := (if Fit_F then 7 else 6);
         P : Param_Vec (0 .. Np - 1) := [others => 0.0];
         Steps : Param_Vec (0 .. Np - 1) := [others => 1.0e-4];   --  差分步(弧度 / 米,极小量)
         Rv : constant V3 := Rot_Vec (Gi.R_Ce);
         Nr : Natural := 0;
         Cur : Long_Float := 0.0;
         function Cam_Of (P : Param_Vec) return Cam_Geo is
            Gt : Cam_Geo := G;
         begin
            Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
            Gt.Pos := [P (3), P (4), P (5)];
            if Fit_F then
               Gt.F := P (6);
            end if;
            return Gt;
         end Cam_Of;
         procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
            Gt : constant Cam_Geo := Cam_Of (P);
            Sum : Long_Float := 0.0;
            I : Natural := 0;
            procedure Put (Du, Dv : Long_Float) is
            begin
               Sum := Sum + Du * Du + Dv * Dv;
               if Fill /= null then
                  Fill (I, Du, Dv);
               end if;
               I := I + 1;
            end Put;
         begin
            Behind := 0;
            for S in 0 .. Ns - 1 loop
               if not Skip (S) then
                  declare
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                     if Front and then Gt.F > 0.0 then
                        Put (Ws (S) * (U - Scene (S).U), Ws (S) * (V - Scene (S).V));
                     else
                        Put (1.0e3, 1.0e3);   --  跑到相机后面:远大于任何一条加权残差的罚(无量纲哨兵)
                        Behind := Behind + 1;
                     end if;
                  end;
               end if;
            end loop;
            if Use_Prior then
               Put ((Gt.F - G.F_Prior) / G.F_Prior_Sd, 0.0);   --  先验那一条残差(以不确定度为单位,无量纲)
            end if;
            R := Sqrt (Sum / Long_Float (Natural'Max (1, I)));
         end Resid;
         --  进解的点按像素算的残差(记账、报原因用;权只管解,不管报数)
         procedure Px_Rms (P : Param_Vec; R : out Long_Float; K : out Natural) is
            Gt : constant Cam_Geo := Cam_Of (P);
            Sum : Long_Float := 0.0;
         begin
            K := 0;
            for S in 0 .. Ns - 1 loop
               if not Skip (S) then
                  declare
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                     if Front then
                        Sum := Sum + (U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2;
                        K := K + 1;
                     end if;
                  end;
               end if;
            end loop;
            R := (if K > 0 then Sqrt (Sum / Long_Float (K)) else 0.0);
         end Px_Rms;
         function Px_Note (P : Param_Vec) return String is
            R : Long_Float;
            K : Natural;
         begin
            Px_Rms (P, R, K);
            return "像素残差 " & Codec.Fmt (R, 2) & " px(" & Codec.Img (K) & " 个点)";
         end Px_Note;
      begin
         P (0) := Rv (0); P (1) := Rv (1); P (2) := Rv (2);
         P (3) := Gi.Pos (0); P (4) := Gi.Pos (1); P (5) := Gi.Pos (2);
         if Fit_F then
            P (6) := Gi.F; Steps (6) := 1.0;   --  焦距的差分步(像素,极小量)
         end if;
         Nr := Ns + (if Use_Prior then 1 else 0);
         if Start_Here then
            --  起点就在真值附近:门从粗到细 —— 先放画幅宽的 1/16(比例,无量纲;挡住的那片配成乱的,乱点散在几百像素里,门外),
            --  解一次,再收到"门内这些点自己的像素离散"的 3 倍(倍数无量纲),来回三遍(次数)。门的尺度是这一次配点自己量的:
            --  转过、挡过的画面配得比标定时粗得多,按标定时的噪声挑,好点也全挑没了(X5B 2026-09-25)
            declare
               Gate : Long_Float := 0.125 * G.Cx;   --  半幅宽的八分之一 = 画幅宽的 1/16(比例,无量纲)
            begin
               for Round in 1 .. 3 loop
                  declare
                     Gt : constant Cam_Geo := Cam_Of (P);
                     Dropped : Natural := 0;
                     Sum : Long_Float := 0.0;
                  begin
                     for S in 0 .. Ns - 1 loop
                        declare
                           U, V : Long_Float;
                           Front : Boolean;
                           E : Long_Float;
                        begin
                           Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                           E := (if Front then Sqrt ((U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2) else Long_Float'Last);
                           Skip (S) := E > Gate;
                           if Skip (S) then
                              Dropped := Dropped + 1;
                           else
                              Sum := Sum + E * E;
                           end if;
                        end;
                     end loop;
                     if Ns - Dropped < Min_Pts then
                        Why := To_Unbounded_String ("从现在的位姿起步,门 " & Codec.Fmt (Gate, 1) & " px 内的点不到 4 个(" & Codec.Img (Ns - Dropped) & "/" & Codec.Img (Ns) & ")");
                        return;
                     end if;
                     Nr := Ns - Dropped + (if Use_Prior then 1 else 0);
                     Resid (P, Cur, null);
                     LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
                     Gate := 3.0 * Sqrt (Sum / Long_Float (Ns - Dropped));
                  end;
               end loop;
            end;
         end if;
         Resid (P, Cur, null);
         LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
         --  离群的点踢掉再解,来回三遍(次数):加权残差比进解的那些的中位数大 3 倍(比例,无量纲)的不要,每遍都从全体重挑(先前踢错的能回来)。
         --  不设"踢的不到四分之一才算"那条(手上标记的规矩):板上天然混着一小撮远处、桌下配错的点,它们在各停里错得一样、交叉核不出来,
         --  一次最小二乘就被它们拽走(G2E 离线:327 个点里 14 个配错,不踢 ⇒ 焦距 −5%、位置差 3 cm;连踢三遍 ⇒ 14 个全踢掉)
         declare
            Rs : Param_Vec (0 .. Natural'Max (0, Ns - 1)) := [others => 0.0];
            Dropped : Natural := 0;
         begin
            for Round in 1 .. (if Start_Here then 0 else 3) loop   --  从现位姿起步的已经按门挑过了
               declare
                  Gt : constant Cam_Geo := Cam_Of (P);
                  Kept : Param_Vec (0 .. Natural'Max (0, Ns - 1)) := [others => 0.0];
                  Nk : Natural := 0;
                  Med : Long_Float;
               begin
                  for S in 0 .. Ns - 1 loop
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                        Rs (S) := (if Front then Ws (S) * Sqrt ((U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2) else Long_Float'Last);
                        if not Skip (S) then
                           Kept (Nk) := Rs (S); Nk := Nk + 1;
                        end if;
                     end;
                  end loop;
                  Med := Median (Kept, Nk);
                  exit when Med <= 0.0;
                  Dropped := 0;
                  for S in 0 .. Ns - 1 loop
                     Skip (S) := Rs (S) > 3.0 * Med;
                     if Skip (S) then
                        Dropped := Dropped + 1;
                     end if;
                  end loop;
                  exit when Ns - Dropped < Min_Pts;
                  Nr := Ns - Dropped + (if Use_Prior then 1 else 0);
                  Resid (P, Cur, null);
                  LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
               end;
            end loop;
            G.Dropped := Dropped;
         end;
         Resid (P, Cur, null);
         if Behind > 0 then
            Why := To_Unbounded_String ("解出来还有 " & Codec.Img (Behind) & " 个板上的点跑到相机后面(" & Px_Note (P) & ")");
            return;   --  不是解,不存
         end if;
         --  同 Fit_Rig 那一条:无畸变针孔的视场不超过 120°(半幅宽 ÷ 焦距 ≤ tan 60° = 1.732,无量纲)——焦距比这还短就不是针孔的解
         --  (X5A 2026-09-24:头顶眼粗解成焦距 140、残差 21 px,驱动照着它把手往错的方向送了 25 cm)
         if Fit_F and then G.Cx > 1.732 * P (6) then
            Why := To_Unbounded_String ("焦距解成 " & Codec.Fmt (P (6), 1) & " px,视场超过 120°,针孔假设不成立(" & Px_Note (P) & ")");
            return;
         end if;
         declare
            Sd : Param_Vec (0 .. Np - 1);
            Lo : V3 := [others => Long_Float'Last];
            Hi : V3 := [others => Long_Float'First];
            Span : Long_Float := 0.0;   --  板上的点在世界里铺开的量程(米)
         begin
            for S in 0 .. Ns - 1 loop
               if not Skip (S) then
                  for I in 0 .. 2 loop
                     Lo (I) := Long_Float'Min (Lo (I), Scene (S).Pw (I)); Hi (I) := Long_Float'Max (Hi (I), Scene (S).Pw (I));
                  end loop;
               end if;
            end loop;
            Span := Sqrt ((Hi (0) - Lo (0)) ** 2 + (Hi (1) - Lo (1)) ** 2 + (Hi (2) - Lo (2)) ** 2);
            Param_Sd (P, Nr, Steps, Resid'Access, Sd);
            G.Rot_Sd := Sqrt (Sd (0) ** 2 + Sd (1) ** 2 + Sd (2) ** 2);
            G.Pos_Sd := Sqrt (Sd (3) ** 2 + Sd (4) ** 2 + Sd (5) ** 2);
            G.F_Sd := (if Fit_F then Sd (6) else 0.0);
            --  位置的不确定度比板铺开的量程还大、或焦距的不确定度比焦距还大 = 方程分不开 ⇒ 不算解出来
            --  (V1I / G1K 2026-09-24:相机解到 2.8 m / 120 m 外、残差却只有零点几像素,就是这种"解")
            if G.Pos_Sd >= Span or else (Fit_F and then G.F_Sd >= P (6)) or else G.Rot_Sd >= 1.0 then
               Why := To_Unbounded_String ("不确定度比量本身还大:位置 ± " & Codec.Fmt (G.Pos_Sd, 3) & " 单位(板铺开 " & Codec.Fmt (Span, 3) & " 单位),焦距 "
                                           & Codec.Fmt (P (Np - 1), 1) & " ± " & Codec.Fmt (G.F_Sd, 1) & " px,朝向 ± " & Codec.Fmt (G.Rot_Sd, 3) & " rad(" & Px_Note (P) & ")");
               return;
            end if;
         end;
         Why := Null_Unbounded_String;
         Px_Rms (P, Rep.Scene_Rms, Rep.Scene_Used);
         G := Cam_Of (P);
         G.F_Meas := (if Fit_F then P (6) else 0.0);
         G.Rms := Rep.Scene_Rms;
         G.Fixed := True;
         G.Valid := True;
         Ok := True;
      end;
   end Fit_Fixed_Board;

   procedure Hand_Points (G : Cam_Geo; O : Obs_Pt_Vectors.Vector; Ray_O, Ray_D : V3_Vectors.Vector; Own_Kind : Nat_Vectors.Vector;
                          Tips : out Tip_Class_Vectors.Vector; Rep : in out Fixed_Report) is
      N : constant Natural := Natural (O.Length);
      Min_Marks : constant := 4;   --  一个点至少几笔才进(次数)
      All_Cls : Tip_Class_Vectors.Vector;   --  出现过的每个点(臂, 瓣数)和它的笔数
      Sum_All : Long_Float := 0.0;
      Cnt_All : Natural := 0;
      function Has_Ray (A : Natural) return Boolean is (A < Natural (Ray_D.Length) and then Norm (Ray_D (A)) > 0.0);
      function On_Ray_Of (A, K : Natural) return Boolean is (Has_Ray (A) and then A < Natural (Own_Kind.Length) and then Own_Kind (A) = K);
      function World_Of (Pose : Plug.Arm_Pose; T : V3) return V3 is
         Tw : constant V3 := Ap (Quat_To_R (Pose), T);
      begin
         return [Pose (0) + Tw (0), Pose (1) + Tw (1), Pose (2) + Tw (2)];
      end World_Of;
      function Px_Err (J : Natural; T : V3) return Long_Float is   --  这一笔:点投回不动的眼,离记下的像素多远(在眼后 = 无穷)
         U, V : Long_Float;
         Front : Boolean;
      begin
         Project_Fixed (G, World_Of (O (J).Pose, T), U, V, Front);
         return (if Front then Sqrt ((U - O (J).U) ** 2 + (V - O (J).V) ** 2) else Long_Float'Last);
      end Px_Err;
   begin
      Tips.Clear;
      Rep.Hand_N := N; Rep.Hand_Used := 0; Rep.Hand_Rms := 0.0;
      if not G.Valid or else not G.Fixed or else G.F <= 0.0 then
         return;
      end if;
      --  认点:(臂, 瓣数) 一样的是手上同一个点
      for J in 0 .. N - 1 loop
         declare
            Found : Integer := -1;
         begin
            for I in 0 .. Natural (All_Cls.Length) - 1 loop
               if All_Cls (I).Arm = O (J).Pt and then All_Cls (I).Kind = O (J).Kind then
                  Found := Integer (I);
               end if;
            end loop;
            if Found < 0 then
               All_Cls.Append (Tip_Class'(Arm => O (J).Pt, Kind => O (J).Kind, Tip => [0.0, 0.0, 0.0], N => 0, Rms => 0.0,
                                          On_Ray => On_Ray_Of (O (J).Pt, O (J).Kind)));
               Found := Integer (All_Cls.Length) - 1;
            end if;
            All_Cls (Natural (Found)).N := All_Cls (Natural (Found)).N + 1;
         end;
      end loop;
      for Cls of All_Cls loop
         if Cls.N >= Min_Marks then
            declare
               Cl : Tip_Class := Cls;
               Use_J : array (0 .. N - 1) of Boolean := [others => False];
               Solved : Boolean := False;
               --  这个点用 Use_J 的那几笔解:每笔 (I − d dᵀ)(p_j + R_j t − C) = 0,d = 不动的眼过 (u_j, v_j) 的单位视线;
               --  视线上的点 t = o + S·r 只解 S(一个数),自由的点解 t(3×3)
               procedure Solve is
                  A : M3 := [others => [others => 0.0]];
                  B : V3 := [others => 0.0];
                  Aa, Ab : Long_Float := 0.0;
                  K : Natural := 0;
               begin
                  Solved := False;
                  for J in 0 .. N - 1 loop
                     if Use_J (J) then
                        declare
                           D : constant V3 := Ray_Fixed (G, O (J).U, O (J).V);
                           Rj : constant M3 := Quat_To_R (O (J).Pose);
                           Pr : M3;
                           Q : M3;
                           Cp : constant V3 := [G.Pos (0) - O (J).Pose (0), G.Pos (1) - O (J).Pose (1), G.Pos (2) - O (J).Pose (2)];
                        begin
                           for R in 0 .. 2 loop
                              for Cc in 0 .. 2 loop
                                 Pr (R, Cc) := (if R = Cc then 1.0 else 0.0) - D (R) * D (Cc);   --  I − d dᵀ(纯数学)
                              end loop;
                           end loop;
                           Q := Mul (Pr, Rj);
                           if Cl.On_Ray then
                              declare
                                 Ro : constant V3 := Ray_O (Cl.Arm);
                                 Rd : constant V3 := Ray_D (Cl.Arm);
                                 Av : constant V3 := Ap (Q, Rd);
                                 Cv : constant V3 := Ap (Pr, [Cp (0) - Ap (Rj, Ro) (0), Cp (1) - Ap (Rj, Ro) (1), Cp (2) - Ap (Rj, Ro) (2)]);
                              begin
                                 Aa := Aa + Av (0) * Av (0) + Av (1) * Av (1) + Av (2) * Av (2);
                                 Ab := Ab + Av (0) * Cv (0) + Av (1) * Cv (1) + Av (2) * Cv (2);
                              end;
                           else
                              declare
                                 Bj : constant V3 := Ap (Pr, Cp);
                              begin
                                 for R in 0 .. 2 loop
                                    for Cc in 0 .. 2 loop
                                       for Kk in 0 .. 2 loop
                                          A (R, Cc) := A (R, Cc) + Q (Kk, R) * Q (Kk, Cc);
                                       end loop;
                                    end loop;
                                    for Kk in 0 .. 2 loop
                                       B (R) := B (R) + Q (Kk, R) * Bj (Kk);
                                    end loop;
                                 end loop;
                              end;
                           end if;
                           K := K + 1;
                        end;
                     end if;
                  end loop;
                  if K < Min_Marks then
                     return;
                  end if;
                  if Cl.On_Ray then
                     if Aa > 0.0 then
                        declare
                           S : constant Long_Float := Ab / Aa;
                           Ro : constant V3 := Ray_O (Cl.Arm);
                           Rd : constant V3 := Ray_D (Cl.Arm);
                        begin
                           Cl.Tip := [Ro (0) + S * Rd (0), Ro (1) + S * Rd (1), Ro (2) + S * Rd (2)];
                           Solved := True;
                        end;
                     end if;
                  else
                     declare
                        T : constant V3 := Solve3 (A, B);
                     begin
                        if Norm (T) > 0.0 and then abs T (0) <= Long_Float'Last and then abs T (1) <= Long_Float'Last and then abs T (2) <= Long_Float'Last then
                           Cl.Tip := T;
                           Solved := True;
                        end if;
                     end;
                  end if;
               end Solve;
            begin
               for J in 0 .. N - 1 loop
                  Use_J (J) := O (J).Pt = Cl.Arm and then O (J).Kind = Cl.Kind;
               end loop;
               Solve;
               --  像素残差超过这个点中位 3 倍的那几笔踢掉(倍数无量纲,同踢离群那一条),再解一次
               if Solved then
                  declare
                     Es : Param_Vec (0 .. N - 1) := [others => 0.0];
                     Ne : Natural := 0;
                  begin
                     for J in 0 .. N - 1 loop
                        if Use_J (J) then
                           Es (Ne) := Px_Err (J, Cl.Tip); Ne := Ne + 1;
                        end if;
                     end loop;
                     declare
                        Me : constant Long_Float := Median (Es, Ne);
                     begin
                        if Me > 0.0 then
                           for J in 0 .. N - 1 loop
                              if Use_J (J) and then Px_Err (J, Cl.Tip) > 3.0 * Me then
                                 Use_J (J) := False;
                              end if;
                           end loop;
                           Solve;
                        end if;
                     end;
                  end;
               end if;
               if Solved then
                  declare
                     Nearest : Long_Float := Long_Float'Last;
                     Sum : Long_Float := 0.0;
                     Cnt : Natural := 0;
                  begin
                     for J in 0 .. N - 1 loop
                        if Use_J (J) then
                           Nearest := Long_Float'Min (Nearest, Norm ([G.Pos (0) - O (J).Pose (0), G.Pos (1) - O (J).Pose (1), G.Pos (2) - O (J).Pose (2)]));
                           Sum := Sum + Px_Err (J, Cl.Tip) ** 2;
                           Cnt := Cnt + 1;
                        end if;
                     end loop;
                     --  点在手上 ⇒ 它离手腕原点不可能比手腕离这只眼还远(几何,不是常数)
                     if Norm (Cl.Tip) < Nearest and then Cnt > 0 then
                        Cl.N := Cnt;
                        Cl.Rms := Sqrt (Sum / Long_Float (Cnt));
                        Tips.Append (Cl);
                        Sum_All := Sum_All + Sum;
                        Cnt_All := Cnt_All + Cnt;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
      Rep.Hand_Used := Cnt_All;
      Rep.Hand_Rms := (if Cnt_All > 0 then Sqrt (Sum_All / Long_Float (Cnt_All)) else 0.0);
   end Hand_Points;

   procedure Fit_Fixed_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; Scene : Scene_Pt_Vectors.Vector; Ray_O, Ray_D : V3_Vectors.Vector;
                            Own_Kind : Nat_Vectors.Vector; Tips : out Tip_Class_Vectors.Vector; Rep : out Fixed_Report; Ok : out Boolean) is
   begin
      Rep := (Hand_N => Natural (O.Length), others => <>);
      Tips.Clear;
      Fit_Fixed_Board (G, Scene, Rep, Ok);
      if Ok then
         Hand_Points (G, O, Ray_O, Ray_D, Own_Kind, Tips, Rep);
      end if;
   end Fit_Fixed_Rig;

   procedure Build_Board (G : Cam_Geo; Cam : Natural; O : Board_Obs_Vectors.Vector; Scene : in out Scene_Pt_Vectors.Vector;
                          Tracks : in out Board_Track_Vectors.Vector; St : out Board_Stats) is
      N : constant Natural := Natural (O.Length);
      --  二维高斯噪声(每轴 σ)下径向误差的中位数 = σ·√(2 ln 2)(纯数学)⇒ 由中位数反推每轴 σ
      Rayleigh_Med : constant Long_Float := Sqrt (2.0 * Log (2.0));
      Max_Pt : Natural := 0;
      --  中位数:拷一份排序,偶数个取中间两个的平均
      function Med (Xs : Param_Vec) return Long_Float is
         A : Param_Vec := Xs;
         K : constant Natural := A'Length;
      begin
         if K = 0 then
            return 0.0;
         end if;
         for I in A'First + 1 .. A'Last loop   --  插入排序
            declare
               X : constant Long_Float := A (I);
               J : Integer := I - 1;
            begin
               while J >= A'First and then A (J) > X loop
                  A (J + 1) := A (J); J := J - 1;
               end loop;
               A (J + 1) := X;
            end;
         end loop;
         if K mod 2 = 1 then
            return A (A'First + K / 2);
         end if;
         return 0.5 * (A (A'First + K / 2 - 1) + A (A'First + K / 2));
      end Med;
   begin
      St := (others => <>);
      if N = 0 or else G.F <= 0.0 then
         return;
      end if;
      for Ob of O loop
         Max_Pt := Natural'Max (Max_Pt, Ob.Pt);
      end loop;
      declare
         Nt : constant Natural := Max_Pt + 1;
         First : array (0 .. Nt - 1) of Integer := [others => -1];   --  每个点的第一笔(链表头)
         Next : array (0 .. N - 1) of Integer := [others => -1];
         Keep : array (0 .. N - 1) of Boolean := [others => True];   --  这一笔进这个点的三角
         Err : array (0 .. N - 1) of Long_Float := [others => -1.0];  --  这一笔在它那个点三角位置处的重投误差(像素;负 = 没算 / 点在这一停的眼后)
         X : array (0 .. Nt - 1) of V3 := [others => [others => 0.0]];
         Cv : array (0 .. Nt - 1) of M3 := [others => [others => [others => 0.0]]];
         Good : array (0 .. Nt - 1) of Boolean := [others => False];
         Nv : array (0 .. Nt - 1) of Natural := [others => 0];
         Mu, Mv, Spread : array (0 .. Nt - 1) of Long_Float := [others => 0.0];
         Has_H : array (0 .. Nt - 1) of Boolean := [others => False];
         Gate : Long_Float := 0.0;
         --  Keep 的那几笔的视线求最小二乘交点(到各条视线的垂直距离平方和最小);至少两笔、解出来是有限数才算
         procedure Solve (T : Natural; Ok : out Boolean; A : out M3; K : out Natural) is
            B : V3 := [others => 0.0];
            J : Integer := First (T);
         begin
            A := [others => [others => 0.0]];
            K := 0;
            while J >= 0 loop
               if Keep (J) then
                  declare
                     C0 : constant V3 := Cam_Pos (G, O (J).Pose);
                     D : constant V3 := Ray (G, O (J).Pose, O (J).U, O (J).V);
                  begin
                     for R in 0 .. 2 loop
                        for Cc in 0 .. 2 loop
                           declare
                              Pm : constant Long_Float := (if R = Cc then 1.0 else 0.0) - D (R) * D (Cc);   --  I − d dᵀ(纯数学)
                           begin
                              A (R, Cc) := A (R, Cc) + Pm;
                              B (R) := B (R) + Pm * C0 (Cc);
                           end;
                        end loop;
                     end loop;
                     K := K + 1;
                  end;
               end if;
               J := Next (J);
            end loop;
            Ok := False;
            if K >= 2 then
               X (T) := Solve3 (A, B);
               --  Solve3 对奇异的方程组(各停视线全平行)回零向量:那不是一个交点
               Ok := abs X (T) (0) <= Long_Float'Last and then abs X (T) (1) <= Long_Float'Last and then abs X (T) (2) <= Long_Float'Last
                 and then Norm (X (T)) > 0.0;
            end if;
         end Solve;
         --  这条点的每一笔:三角位置投回那一停的眼,离配到的像素多远
         procedure Reproject (T : Natural) is
            J : Integer := First (T);
         begin
            while J >= 0 loop
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Project (G, O (J).Pose, X (T), U, V, Front);
                  Err (J) := (if Front then Sqrt ((U - O (J).U) ** 2 + (V - O (J).V) ** 2) else -1.0);
               end;
               J := Next (J);
            end loop;
         end Reproject;
      begin
         for J in reverse 0 .. N - 1 loop
            Next (J) := First (O (J).Pt);
            First (O (J).Pt) := J;
         end loop;
         --  ① 每个点先拿全部几笔三角一次,量出这只眼的配点噪声(全体重投误差的中位)
         declare
            Es : Param_Vec (0 .. N - 1) := [others => 0.0];
            Ne : Natural := 0;
         begin
            for T in 0 .. Nt - 1 loop
               if First (T) >= 0 then
                  St.Tracks := St.Tracks + 1;
                  declare
                     A : M3;
                     K : Natural;
                     Ok : Boolean;
                  begin
                     Solve (T, Ok, A, K);
                     if Ok then
                        Reproject (T);
                        declare
                           J : Integer := First (T);
                        begin
                           while J >= 0 loop
                              if Err (J) >= 0.0 then
                                 Es (Ne) := Err (J); Ne := Ne + 1;
                              end if;
                              J := Next (J);
                           end loop;
                        end;
                     end if;
                  end;
               end if;
            end loop;
            if Ne = 0 then
               return;
            end if;
            declare
               Me : constant Long_Float := Med (Es (0 .. Ne - 1));
            begin
               St.Sigma_W := Me / Rayleigh_Med;
               Gate := 3.0 * Me;   --  重投超过中位 3 倍的那一停是配错了(倍数无量纲,同踢离群那一条)
            end;
         end;
         --  ② 门来回量三遍(次数):出了画面、配错的那一停先把整条点的交点拽歪,同一条点的每一停误差都大,全体的中位被抬高(合成:量成真值的 3 倍);
         --  只留门内的几笔再三角、再对全部几笔重投、按留下的重量中位,门就收到只剩配点噪声
         for Round in 1 .. 3 loop
            declare
               Es : Param_Vec (0 .. N - 1) := [others => 0.0];
               Ne : Natural := 0;
            begin
               for T in 0 .. Nt - 1 loop
                  if First (T) >= 0 then
                     declare
                        J : Integer := First (T);
                        A : M3;
                        K : Natural;
                        Ok : Boolean;
                     begin
                        while J >= 0 loop
                           Keep (J) := Err (J) >= 0.0 and then Err (J) <= Gate;
                           J := Next (J);
                        end loop;
                        Solve (T, Ok, A, K);
                        if Ok then
                           Reproject (T);   --  对全部几笔重投:拽歪的交点放回去之后,好的那几停回到门内
                           J := First (T);
                           while J >= 0 loop
                              if Err (J) >= 0.0 and then Err (J) <= Gate then
                                 Es (Ne) := Err (J); Ne := Ne + 1;
                              end if;
                              J := Next (J);
                           end loop;
                        else
                           J := First (T);
                           while J >= 0 loop
                              Err (J) := -1.0;   --  这条点三角不了:它的每一笔都不算
                              J := Next (J);
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
               exit when Ne = 0;
               declare
                  Me : constant Long_Float := Med (Es (0 .. Ne - 1));
               begin
                  St.Sigma_W := Me / Rayleigh_Med;
                  Gate := 3.0 * Me;   --  倍数无量纲,同踢离群那一条
               end;
            end;
         end loop;
         --  ② 只留门内的几笔再三角;远近的不确定度比远近本身还小才算定住了
         --  (自己手上的点跟着眼走、各停视线平行,远近定不住;太远的点视差淹在噪声里,同样定不住)
         for T in 0 .. Nt - 1 loop
            if First (T) >= 0 then
               declare
                  J : Integer := First (T);
                  A : M3;
                  K : Natural;
                  Ok : Boolean;
                  In_Front : Boolean := True;
               begin
                  while J >= 0 loop
                     Keep (J) := Err (J) >= 0.0 and then Err (J) <= Gate;
                     J := Next (J);
                  end loop;
                  Solve (T, Ok, A, K);
                  if Ok then
                     Reproject (T);
                     J := First (T);
                     while J >= 0 loop
                        if Keep (J) and then Err (J) < 0.0 then
                           In_Front := False;
                        end if;
                        J := Next (J);
                     end loop;
                  end if;
                  if Ok and then In_Front then
                     declare
                        Sig : constant Long_Float := St.Sigma_W / G.F;   --  每条视线的角噪声(弧度)
                        S : M3 := [others => [others => 0.0]];
                        Dm : V3 := [others => 0.0];
                        Rsum : Long_Float := 0.0;
                        Ai, Tmp, C3 : M3;
                        Singular : Boolean := False;
                     begin
                        J := First (T);
                        while J >= 0 loop
                           if Keep (J) then
                              declare
                                 C0 : constant V3 := Cam_Pos (G, O (J).Pose);
                                 D : constant V3 := Ray (G, O (J).Pose, O (J).U, O (J).V);
                                 Rv : constant V3 := [X (T) (0) - C0 (0), X (T) (1) - C0 (1), X (T) (2) - C0 (2)];
                                 R : constant Long_Float := Norm (Rv);
                              begin
                                 --  这条视线在交点处横着的抖动:协方差 (σ·r)² (I − d dᵀ)
                                 for Ii in 0 .. 2 loop
                                    for Kk in 0 .. 2 loop
                                       S (Ii, Kk) := S (Ii, Kk) + (Sig * R) ** 2 * ((if Ii = Kk then 1.0 else 0.0) - D (Ii) * D (Kk));
                                    end loop;
                                 end loop;
                                 if R > 0.0 then
                                    for Ii in 0 .. 2 loop
                                       Dm (Ii) := Dm (Ii) + Rv (Ii) / R;
                                    end loop;
                                 end if;
                                 Rsum := Rsum + R;
                              end;
                           end if;
                           J := Next (J);
                        end loop;
                        --  交点 = A⁻¹ Σ (I − d dᵀ) c ⇒ 协方差 = A⁻¹ S A⁻¹(逆的某一列回零向量 = 方程组奇异,远近定不住)
                        for Col in 0 .. 2 loop
                           declare
                              E : V3 := [others => 0.0];
                              Xc : V3;
                           begin
                              E (Col) := 1.0;
                              Xc := Solve3 (A, E);
                              if Norm (Xc) = 0.0 then
                                 Singular := True;
                              end if;
                              for Rw in 0 .. 2 loop
                                 Ai (Rw, Col) := Xc (Rw);
                              end loop;
                           end;
                        end loop;
                        Tmp := Mul (Ai, S);
                        C3 := Mul (Tmp, Ai);
                        declare
                           Dn : constant Long_Float := Norm (Dm);
                           Rm : constant Long_Float := Rsum / Long_Float (K);
                           Var_D : Long_Float := 0.0;
                        begin
                           if Dn > 0.0 and then not Singular then
                              for Ii in 0 .. 2 loop
                                 for Kk in 0 .. 2 loop
                                    Var_D := Var_D + Dm (Ii) / Dn * C3 (Ii, Kk) * Dm (Kk) / Dn;
                                 end loop;
                              end loop;
                              --  远近定得住 = 远近的不确定度的 3 倍(倍数无量纲,同踢离群)还小于远近本身。以前是 1 倍:有不动的眼时各停交叉核对把漏进来的踢掉,
                              --  没有不动的眼的身体就靠这一道 —— 跟着眼走的夹爪上的点几停视线几乎平行,1 倍的门让它们三角到二三十米外还进了板(焊点实测)
                              if Var_D >= 0.0 and then 3.0 * Sqrt (Var_D) < Rm then
                                 Good (T) := True; Cv (T) := C3; Nv (T) := K;
                                 St.Tri_Ok := St.Tri_Ok + 1;
                              end if;
                           end if;
                        end;
                     end;
                  end if;
               end;
            end if;
         end loop;
         --  ③ 不动的眼里:各停(门内的那几笔)配到的像素取中位,离散 = 各停离中位的中位距离;至少两停配到才核得了
         declare
            Ss : Param_Vec (0 .. Nt - 1) := [others => 0.0];
            N_Ss : Natural := 0;
         begin
            for T in 0 .. Nt - 1 loop
               if Good (T) then
                  declare
                     Hu_A, Hv_A, Ds : Param_Vec (0 .. Nv (T) - 1) := [others => 0.0];
                     Nh : Natural := 0;
                     J : Integer := First (T);
                  begin
                     while J >= 0 loop
                        if Keep (J) and then O (J).Hu >= 0.0 and then O (J).Hv >= 0.0 and then Nh < Nv (T) then
                           Hu_A (Nh) := O (J).Hu; Hv_A (Nh) := O (J).Hv; Nh := Nh + 1;
                        end if;
                        J := Next (J);
                     end loop;
                     if Nh >= 2 then
                        Mu (T) := Med (Hu_A (0 .. Nh - 1));
                        Mv (T) := Med (Hv_A (0 .. Nh - 1));
                        for I in 0 .. Nh - 1 loop
                           Ds (I) := Sqrt ((Hu_A (I) - Mu (T)) ** 2 + (Hv_A (I) - Mv (T)) ** 2);
                        end loop;
                        Spread (T) := Med (Ds (0 .. Nh - 1));
                        Has_H (T) := True;
                        Ss (N_Ss) := Spread (T); N_Ss := N_Ss + 1;
                     end if;
                  end;
               end if;
            end loop;
            if N_Ss = 0 then
               --  一个点都没有不动的眼里的像素 = 这具身体没有不动的眼(2026-09-26,PLAN §2b):板只靠这只手上的眼几停三角,点照样进板
               --  (东西躺的面照样拟合得出来,碰桌面量指尖照样做得了);不动的眼里的像素记成 −1(没有)
               for T in 0 .. Nt - 1 loop
                  if Good (T) then
                     Scene.Append (Scene_Pt'(Pw => X (T), Cov => Cv (T), U => -1.0, V => -1.0, Sh => 0.0, Views => Nv (T)));
                     declare
                        Tr : Board_Track := (Cam => Cam, Hu => -1.0, Hv => -1.0, Sw => St.Sigma_W, Sh => 0.0, others => <>);
                        J : Integer := First (T);
                     begin
                        while J >= 0 loop
                           if Keep (J) then
                              Tr.Views.Append (Board_View'(Pose => O (J).Pose, U => O (J).U, V => O (J).V));
                           end if;
                           J := Next (J);
                        end loop;
                        Tracks.Append (Tr);
                     end;
                     St.Kept := St.Kept + 1;
                  end if;
               end loop;
               return;
            end if;
            declare
               Ms : constant Long_Float := Med (Ss (0 .. N_Ss - 1));
            begin
               St.Sigma_H := Ms / Rayleigh_Med;
               for T in 0 .. Nt - 1 loop
                  if Has_H (T) and then Spread (T) <= 3.0 * Ms then   --  各停配到的离散超过全体中位 3 倍 = 有一停配错了(倍数无量纲)
                     Scene.Append (Scene_Pt'(Pw => X (T), Cov => Cv (T), U => Mu (T), V => Mv (T), Sh => St.Sigma_H, Views => Nv (T)));
                     declare
                        Tr : Board_Track := (Cam => Cam, Hu => Mu (T), Hv => Mv (T), Sw => St.Sigma_W, Sh => St.Sigma_H, others => <>);
                        J : Integer := First (T);
                     begin
                        while J >= 0 loop
                           if Keep (J) then
                              Tr.Views.Append (Board_View'(Pose => O (J).Pose, U => O (J).U, V => O (J).V));
                           end if;
                           J := Next (J);
                        end loop;
                        Tracks.Append (Tr);
                     end;
                     St.Kept := St.Kept + 1;
                  end if;
               end loop;
            end;
         end;
      end;
   end Build_Board;

   --  一条点的几停视线求最小二乘交点(到各条视线垂直距离平方和最小)和它的协方差(每条视线角噪声 Sw / F,同 Build_Board)
   procedure Tri_Views (G : Cam_Geo; Views : Board_View_Vectors.Vector; Sw : Long_Float; X : out V3; Cov : out M3; Ok : out Boolean) is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [others => 0.0];
      S : M3 := [others => [others => 0.0]];
      Sig : constant Long_Float := (if G.F > 0.0 then Sw / G.F else 0.0);   --  每条视线的角噪声(弧度)
   begin
      X := [others => 0.0]; Cov := [others => [others => 0.0]]; Ok := False;
      if Natural (Views.Length) < 2 or else G.F <= 0.0 then
         return;
      end if;
      for Vw of Views loop
         declare
            C0 : constant V3 := Cam_Pos (G, Vw.Pose);
            D : constant V3 := Ray (G, Vw.Pose, Vw.U, Vw.V);
         begin
            for R in 0 .. 2 loop
               for Cc in 0 .. 2 loop
                  declare
                     Pm : constant Long_Float := (if R = Cc then 1.0 else 0.0) - D (R) * D (Cc);   --  I − d dᵀ(纯数学)
                  begin
                     A (R, Cc) := A (R, Cc) + Pm;
                     B (R) := B (R) + Pm * C0 (Cc);
                  end;
               end loop;
            end loop;
         end;
      end loop;
      X := Solve3 (A, B);
      if Norm (X) = 0.0 or else not (abs X (0) <= Long_Float'Last and then abs X (1) <= Long_Float'Last and then abs X (2) <= Long_Float'Last) then
         return;   --  奇异(视线全平行)或不是有限数
      end if;
      for Vw of Views loop
         declare
            C0 : constant V3 := Cam_Pos (G, Vw.Pose);
            D : constant V3 := Ray (G, Vw.Pose, Vw.U, Vw.V);
            R : constant Long_Float := Norm ([X (0) - C0 (0), X (1) - C0 (1), X (2) - C0 (2)]);
         begin
            for Ii in 0 .. 2 loop
               for Kk in 0 .. 2 loop
                  S (Ii, Kk) := S (Ii, Kk) + (Sig * R) ** 2 * ((if Ii = Kk then 1.0 else 0.0) - D (Ii) * D (Kk));
               end loop;
            end loop;
         end;
      end loop;
      declare
         Ai : M3;
      begin
         for Col in 0 .. 2 loop
            declare
               E : V3 := [others => 0.0];
               Xc : V3;
            begin
               E (Col) := 1.0;
               Xc := Solve3 (A, E);
               if Norm (Xc) = 0.0 then
                  return;
               end if;
               for Rw in 0 .. 2 loop
                  Ai (Rw, Col) := Xc (Rw);
               end loop;
            end;
         end loop;
         Cov := Mul (Mul (Ai, S), Ai);
      end;
      Ok := True;
   end Tri_Views;

   procedure Board_Points (Geos : Geo_Vectors.Vector; Tracks : Board_Track_Vectors.Vector; Scene : out Scene_Pt_Vectors.Vector) is
   begin
      Scene.Clear;
      for T of Tracks loop
         if T.Cam < Natural (Geos.Length) then
            declare
               X : V3;
               Cv : M3;
               Ok : Boolean;
            begin
               Tri_Views (Geos (T.Cam), T.Views, T.Sw, X, Cv, Ok);
               if Ok then
                  Scene.Append (Scene_Pt'(Pw => X, Cov => Cv, U => T.Hu, V => T.Hv, Sh => T.Sh, Views => Natural (T.Views.Length)));
               end if;
            end;
         end if;
      end loop;
   end Board_Points;

   procedure Refine_Board (Geos : in out Geo_Vectors.Vector; Head : in out Cam_Geo; Tracks : Board_Track_Vectors.Vector; Rep : out Refine_Report; Ok : out Boolean) is
      Nt : constant Natural := Natural (Tracks.Length);
      --  出现过的腕眼(相机号),各占参数里的 9 个:朝向改正 3(手系里左乘的小转动)、偏移改正 3(米)、焦距比例改正 1、镜头畸变改正 2(K1、K2)
      Cams : Nat_Vectors.Vector;
      function Slot_Of (Cam : Natural) return Natural is
      begin
         for I in 0 .. Natural (Cams.Length) - 1 loop
            if Cams (I) = Cam then
               return I;
            end if;
         end loop;
         return 0;
      end Slot_Of;
      Base : constant Geo_Vectors.Vector := Geos;
      Fit_Fh : constant Boolean := Head.F_Meas > 0.0;   --  不动的眼的焦距是解出来的(没给)才接着解
      --  没有不动的眼(2026-09-26):只解腕眼(朝向、偏移、焦距、畸变),板上的点照样每步按腕眼几停重新三角;不动的眼那几个参数不进解、Head 不动
      Has_Head : constant Boolean := Head.Valid and then Head.Fixed;
      Use_H : array (0 .. Natural'Max (1, Nt) - 1) of Boolean := [others => True];   --  这条点进不动的眼的残差
      N_Views : Natural := 0;
   begin
      Ok := False;
      Rep := (Tracks => Nt, others => <>);
      for K in 0 .. Nt - 1 loop
         Use_H (K) := Has_Head and then Tracks (K).Hu >= 0.0 and then Tracks (K).Hv >= 0.0;   --  不动的眼里没配到的点(−1)不进它的残差
      end loop;
      if Nt < 4 then   --  单点法的下限(次数)
         Why := To_Unbounded_String ("板上的点不到 4 条");
         return;
      end if;
      for T of Tracks loop
         if not Cams.Contains (T.Cam) then
            Cams.Append (T.Cam);
         end if;
         N_Views := N_Views + Natural (T.Views.Length);
      end loop;
      declare
         Nc : constant Natural := Natural (Cams.Length);
         Per : constant := 9;   --  每台眼的参数个数(格式)
         Hb : constant Natural := Per * Nc;   --  不动的眼的参数从这儿起:朝向 3、位置 3、焦距 1、畸变 2
         Np : constant Natural := Hb + (if Has_Head then Per else 0);
         P : Param_Vec (0 .. Np - 1) := [others => 0.0];
         Steps : Param_Vec (0 .. Np - 1) := [others => 1.0e-4];   --  差分步(弧度 / 米 / 比例,极小量)
         Cur : Long_Float := 0.0;
         Nr : Natural := 0;
         function Fit_F (C : Natural) return Boolean is (Base (Cams (C)).F_Meas > 0.0);   --  这台腕眼的焦距是解出来的(没给)
         function Wrist_Of (P : Param_Vec; C : Natural) return Cam_Geo is
            G : Cam_Geo := Base (Cams (C));
            I : constant Natural := Per * C;
         begin
            G.R_Ce := Mul (Rodrigues ([P (I), P (I + 1), P (I + 2)]), Base (Cams (C)).R_Ce);
            G.Off := [Base (Cams (C)).Off (0) + P (I + 3), Base (Cams (C)).Off (1) + P (I + 4), Base (Cams (C)).Off (2) + P (I + 5)];
            if Fit_F (C) then
               G.F := Base (Cams (C)).F * (1.0 + P (I + 6));
            end if;
            G.K1 := Base (Cams (C)).K1 + P (I + 7); G.K2 := Base (Cams (C)).K2 + P (I + 8);
            return G;
         end Wrist_Of;
         function Head_Of (P : Param_Vec) return Cam_Geo is
            G : Cam_Geo := Head;
         begin
            if not Has_Head then
               return G;
            end if;
            G.R_Ce := Rodrigues ([P (Hb), P (Hb + 1), P (Hb + 2)]);
            G.Pos := [P (Hb + 3), P (Hb + 4), P (Hb + 5)];
            if Fit_Fh then
               G.F := P (Hb + 6);
            end if;
            G.K1 := Head.K1 + P (Hb + 7); G.K2 := Head.K2 + P (Hb + 8);
            return G;
         end Head_Of;
         N_Prior : Natural := 0;   --  先验的残差条数(偏移三轴 + 焦距,按腕眼)
         procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
            Gh : constant Cam_Geo := Head_Of (P);
            Ws : array (0 .. Nc - 1) of Cam_Geo;
            Sum : Long_Float := 0.0;
            I : Natural := 0;
            procedure Put (Du, Dv : Long_Float) is
            begin
               Sum := Sum + Du * Du + Dv * Dv;
               if Fill /= null then
                  Fill (I, Du, Dv);
               end if;
               I := I + 1;
            end Put;
         begin
            for C in 0 .. Nc - 1 loop
               Ws (C) := Wrist_Of (P, C);
            end loop;
            for K in 0 .. Nt - 1 loop
               declare
                  T : constant Board_Track := Tracks (K);
                  Gw : Cam_Geo renames Ws (Slot_Of (T.Cam));
                  X : V3;
                  Cv : M3;
                  Tok : Boolean;
               begin
                  Tri_Views (Gw, T.Views, T.Sw, X, Cv, Tok);
                  for Vw of T.Views loop
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        if Tok then
                           Project (Gw, Vw.Pose, X, U, V, Front);
                        else
                           Front := False;
                        end if;
                        if Front then
                           Put ((U - Vw.U) / T.Sw, (V - Vw.V) / T.Sw);
                        else
                           Put (1.0e3, 1.0e3);   --  三角不了 / 在眼后:远大于任何一条加权残差的罚(无量纲哨兵)
                        end if;
                     end;
                  end loop;
                  if Use_H (K) then
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        if Tok then
                           Project_Fixed (Gh, X, U, V, Front);
                        else
                           Front := False;
                        end if;
                        if Front and then Gh.F > 0.0 then
                           Put ((U - T.Hu) / T.Sh, (V - T.Hv) / T.Sh);
                        else
                           Put (1.0e3, 1.0e3);   --  同上(无量纲哨兵)
                        end if;
                     end;
                  end if;
               end;
            end loop;
            --  腕眼标定量到的偏移、焦距当先验:改正量 ÷ 它们自己报的不确定度(偏移的 ± 是三轴合起来的,每轴分 √3;没报就不加)
            for C in 0 .. Nc - 1 loop
               declare
                  G0 : constant Cam_Geo := Base (Cams (C));
                  Ix : constant Natural := Per * C;
                  Axis_Sd : constant Long_Float := G0.Off_Sd / Sqrt (3.0);
               begin
                  if G0.Off_Sd > 0.0 then
                     Put (P (Ix + 3) / Axis_Sd, P (Ix + 4) / Axis_Sd);
                     Put (P (Ix + 5) / Axis_Sd, 0.0);
                  end if;
                  if Fit_F (C) and then G0.F_Sd > 0.0 then
                     Put (G0.F * P (Ix + 6) / G0.F_Sd, 0.0);
                  end if;
               end;
            end loop;
            R := Sqrt (Sum / Long_Float (Natural'Max (1, I)));
         end Resid;
         --  按像素的残差(报数用):腕眼各停、不动的眼
         procedure Px (P : Param_Vec; Wr, Hr : out Long_Float; Hn : out Natural) is
            Gh : constant Cam_Geo := Head_Of (P);
            Sw2, Sh2 : Long_Float := 0.0;
            Nw : Natural := 0;
         begin
            Hn := 0;
            for K in 0 .. Nt - 1 loop
               declare
                  T : constant Board_Track := Tracks (K);
                  Gw : constant Cam_Geo := Wrist_Of (P, Slot_Of (T.Cam));
                  X : V3;
                  Cv : M3;
                  Tok : Boolean;
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Tri_Views (Gw, T.Views, T.Sw, X, Cv, Tok);
                  if Tok then
                     for Vw of T.Views loop
                        Project (Gw, Vw.Pose, X, U, V, Front);
                        if Front then
                           Sw2 := Sw2 + (U - Vw.U) ** 2 + (V - Vw.V) ** 2; Nw := Nw + 1;
                        end if;
                     end loop;
                     if Use_H (K) then
                        Project_Fixed (Gh, X, U, V, Front);
                        if Front then
                           Sh2 := Sh2 + (U - T.Hu) ** 2 + (V - T.Hv) ** 2; Hn := Hn + 1;
                        end if;
                     end if;
                  end if;
               end;
            end loop;
            Wr := (if Nw > 0 then Sqrt (Sw2 / Long_Float (Nw)) else 0.0);
            Hr := (if Hn > 0 then Sqrt (Sh2 / Long_Float (Hn)) else 0.0);
         end Px;
      begin
         if Has_Head then
            declare
               Rv : constant V3 := Rot_Vec (Head.R_Ce);
            begin
               P (Hb) := Rv (0); P (Hb + 1) := Rv (1); P (Hb + 2) := Rv (2);
               P (Hb + 3) := Head.Pos (0); P (Hb + 4) := Head.Pos (1); P (Hb + 5) := Head.Pos (2);
               P (Hb + 6) := Head.F; Steps (Hb + 6) := 1.0;   --  焦距的差分步(像素,极小量)
            end;
         end if;
         for C in 0 .. Nc - 1 loop
            if Base (Cams (C)).Off_Sd > 0.0 then
               N_Prior := N_Prior + 2;
            end if;
            if Fit_F (C) and then Base (Cams (C)).F_Sd > 0.0 then
               N_Prior := N_Prior + 1;
            end if;
         end loop;
         --  两遍(次数):解 → 不动的眼里的点按 3 倍中位重挑(倍数无量纲,每遍从全体重挑)→ 再解
         for Round in 1 .. 2 loop
            declare
               N_H : Natural := 0;
            begin
               for K in 0 .. Nt - 1 loop
                  if Use_H (K) then
                     N_H := N_H + 1;
                  end if;
               end loop;
               Nr := N_Views + N_H + N_Prior;
               Resid (P, Cur, null);
               LM_Refine (P, Nr, Steps, 60, Resid'Access, Cur);   --  60 = 迭代次数上限(次数)
            end;
            declare
               Gh : constant Cam_Geo := Head_Of (P);
               Rs : Param_Vec (0 .. Nt - 1) := [others => Long_Float'Last];
               Kept : Param_Vec (0 .. Nt - 1) := [others => 0.0];
               Nk : Natural := 0;
               Med : Long_Float;
            begin
               for K in 0 .. Nt - 1 loop
                  declare
                     T : constant Board_Track := Tracks (K);
                     X : V3;
                     Cv : M3;
                     Tok : Boolean;
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Tri_Views (Wrist_Of (P, Slot_Of (T.Cam)), T.Views, T.Sw, X, Cv, Tok);
                     if Tok then
                        Project_Fixed (Gh, X, U, V, Front);
                        if Front then
                           Rs (K) := Sqrt ((U - T.Hu) ** 2 + (V - T.Hv) ** 2) / T.Sh;
                        end if;
                     end if;
                     if Use_H (K) then
                        Kept (Nk) := Rs (K); Nk := Nk + 1;
                     end if;
                  end;
               end loop;
               Med := Median (Kept, Nk);
               if Has_Head and then Med > 0.0 then
                  for K in 0 .. Nt - 1 loop
                     Use_H (K) := Rs (K) <= 3.0 * Med;
                  end loop;
               end if;
            end;
         end loop;
         declare
            N_H : Natural := 0;
         begin
            for K in 0 .. Nt - 1 loop
               if Use_H (K) then
                  N_H := N_H + 1;
               end if;
            end loop;
            Nr := N_Views + N_H + N_Prior;
            Resid (P, Cur, null);
            LM_Refine (P, Nr, Steps, 60, Resid'Access, Cur);   --  按最后挑的那批再解一次(次数)
         end;
         declare
            Sd : Param_Vec (0 .. Np - 1);
            Gh : constant Cam_Geo := Head_Of (P);
            Lo : V3 := [others => Long_Float'Last];
            Hi : V3 := [others => Long_Float'First];
            Span : Long_Float;
            Scene : Scene_Pt_Vectors.Vector;
            Wr, Hr : Long_Float;
            Hn : Natural;
         begin
            Param_Sd (P, Nr, Steps, Resid'Access, Sd);
            declare
               Gs : Geo_Vectors.Vector := Base;
            begin
               for C in 0 .. Nc - 1 loop
                  Gs.Replace_Element (Cams (C), Wrist_Of (P, C));
               end loop;
               Board_Points (Gs, Tracks, Scene);
            end;
            for S of Scene loop
               for I in 0 .. 2 loop
                  Lo (I) := Long_Float'Min (Lo (I), S.Pw (I)); Hi (I) := Long_Float'Max (Hi (I), S.Pw (I));
               end loop;
            end loop;
            Span := Sqrt ((Hi (0) - Lo (0)) ** 2 + (Hi (1) - Lo (1)) ** 2 + (Hi (2) - Lo (2)) ** 2);
            --  同 Fit_Fixed_Board:视场界、不确定度界;任何一台腕眼的焦距 ± 比焦距还大、偏移 ± 比板铺开的量程还大,也不算
            if Has_Head and then Fit_Fh and then Gh.Cx > 1.732 * Gh.F then   --  tan 60°(半幅宽 ÷ 焦距,无量纲)
               Why := To_Unbounded_String ("一起解之后不动的眼焦距 " & Codec.Fmt (Gh.F, 1) & " px,视场超过 120°");
               return;
            end if;
            declare
               Pos_Sd : constant Long_Float := (if Has_Head then Sqrt (Sd (Hb + 3) ** 2 + Sd (Hb + 4) ** 2 + Sd (Hb + 5) ** 2) else 0.0);
               Fh_Sd : constant Long_Float := (if Has_Head and then Fit_Fh then Sd (Hb + 6) else 0.0);
            begin
               if Has_Head and then (Pos_Sd >= Span or else Fh_Sd >= Gh.F) then
                  Why := To_Unbounded_String ("一起解之后不动的眼的不确定度比量本身还大:位置 ± " & Codec.Fmt (Pos_Sd, 3) & " 单位(板铺开 " & Codec.Fmt (Span, 3) & " 单位),焦距 ± "
                                              & Codec.Fmt (Fh_Sd, 1) & " px");
                  return;
               end if;
               for C in 0 .. Nc - 1 loop
                  declare
                     Gw : constant Cam_Geo := Wrist_Of (P, C);
                     I : constant Natural := Per * C;
                     Fw_Sd : constant Long_Float := (if Fit_F (C) then Sd (I + 6) * Base (Cams (C)).F else 0.0);
                     Off_Sd : constant Long_Float := Sqrt (Sd (I + 3) ** 2 + Sd (I + 4) ** 2 + Sd (I + 5) ** 2);
                  begin
                     if Fw_Sd >= Gw.F or else Off_Sd >= Span then
                        Why := To_Unbounded_String ("一起解之后第 " & Codec.Img (Cams (C)) & " 台腕眼的不确定度比量本身还大:焦距 ± " & Codec.Fmt (Fw_Sd, 1) & " px,偏移 ± "
                                                    & Codec.Fmt (Off_Sd, 3) & " 单位");
                        return;
                     end if;
                  end;
               end loop;
               --  收下:腕眼的朝向、偏移、焦距(和各自的 ±),不动的眼的朝向、位置、焦距
               for C in 0 .. Nc - 1 loop
                  declare
                     Gw : Cam_Geo := Wrist_Of (P, C);
                     I : constant Natural := Per * C;
                  begin
                     Gw.F_Sd := (if Fit_F (C) then Sd (I + 6) * Base (Cams (C)).F else Gw.F_Sd);
                     if Fit_F (C) then
                        Gw.F_Meas := Gw.F;
                     end if;
                     Gw.Off_Sd := Sqrt (Sd (I + 3) ** 2 + Sd (I + 4) ** 2 + Sd (I + 5) ** 2);
                     Gw.Rot_Sd := Sqrt (Sd (I) ** 2 + Sd (I + 1) ** 2 + Sd (I + 2) ** 2);
                     Gw.K1_Sd := Sd (I + 7);
                     Geos.Replace_Element (Cams (C), Gw);
                  end;
               end loop;
               Px (P, Wr, Hr, Hn);
               if Has_Head then
                  Head := Gh;
                  Head.Pos_Sd := Pos_Sd; Head.F_Sd := Fh_Sd;
                  Head.Rot_Sd := Sqrt (Sd (Hb) ** 2 + Sd (Hb + 1) ** 2 + Sd (Hb + 2) ** 2);
                  Head.K1_Sd := Sd (Hb + 7);
                  if Fit_Fh then
                     Head.F_Meas := Gh.F;
                  end if;
                  Head.Rms := Hr;
               end if;
               Rep.Head_Used := Hn; Rep.Head_Rms := Hr; Rep.Wrist_Rms := Wr;
               Why := Null_Unbounded_String;
               Ok := True;
            end;
         end;
      end;
   end Refine_Board;

   --  画面的几块:0 左半、1 右半、2 上半、3 下半、4 左上、5 右上、6 左下、7 右下(按像素;分界 = 主点,驱动的约定里主点就是画幅中心)
   function In_Region (R : Natural; U, V, Cx, Cy : Long_Float) return Boolean is
     (case R is
         when 0 => U < Cx,
         when 1 => U >= Cx,
         when 2 => V < Cy,
         when 3 => V >= Cy,
         when 4 => U < Cx and then V < Cy,
         when 5 => U >= Cx and then V < Cy,
         when 6 => U < Cx and then V >= Cy,
         when others => U >= Cx and then V >= Cy);
   function Region_Name (R : Natural) return String is
     (case R is
         when 0 => "左半边", when 1 => "右半边", when 2 => "上半边", when 3 => "下半边",
         when 4 => "左上那四分之一", when 5 => "右上那四分之一", when 6 => "左下那四分之一", when others => "右下那四分之一");
   --  点按位姿 Pg 投进画面(画幅 = 两倍主点):落在哪几块
   procedure Add_Regions (Pg : Cam_Geo; U, V : Long_Float; Reg : in out Region_Counts) is
   begin
      if U >= 0.0 and then V >= 0.0 and then U < 2.0 * Pg.Cx and then V < 2.0 * Pg.Cy then
         for R in Reg'Range loop
            if In_Region (R, U, V, Pg.Cx, Pg.Cy) then
               Reg (R) := Reg (R) + 1;
            end if;
         end loop;
      end if;
   end Add_Regions;
   function Seen_All (G : Cam_Geo; Scene : Scene_Pt_Vectors.Vector) return Fixed_Best is
      B : Fixed_Best;
   begin
      for P of Scene loop
         declare
            U, V : Long_Float;
            Front : Boolean;
         begin
            Project_Fixed (G, P.Pw, U, V, Front);
            if Front then
               B.All_N := B.All_N + 1;
               Add_Regions (G, U, V, B.Region);
            end if;
         end;
      end loop;
      return B;
   end Seen_All;

   procedure Check_Fixed (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Now : Scene_Pt_Vectors.Vector; Best : in out Fixed_Best; Rep : out Fixed_Check;
                          Turn_Sd : Long_Float := 0.0; Base_Now : Integer := -1) is
      Cur : Scene_Pt_Vectors.Vector;
      Gn : Cam_Geo := G;
      Fr : Fixed_Report;
      Ok : Boolean;
      Deg : constant Long_Float := 180.0 / Ada.Numerics.Pi;   --  弧度 → 度(换算,无量纲)
      Reg_Now : Region_Counts := [others => 0];               --  此刻每一块里和现在的位姿对得上几个
      --  和位姿 Pg 对得上的点(门 Gt 以内)数一遍,再按点在 Pg 下该落在哪块分着数
      procedure Tally (Pg : Cam_Geo; Gt : Long_Float; Total : out Natural; Reg : out Region_Counts) is
      begin
         Total := 0; Reg := [others => 0];
         for P of Cur loop
            declare
               U, V : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (Pg, P.Pw, U, V, Front);
               if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gt then
                  Total := Total + 1;
                  Add_Regions (Pg, U, V, Reg);
               end if;
            end;
         end loop;
      end Tally;
   begin
      Rep := (Asked => Natural (Scene.Length), others => <>);
      for I in 0 .. Natural'Min (Natural (Scene.Length), Natural (Now.Length)) - 1 loop
         if Now (I).U >= 0.0 and then Now (I).V >= 0.0 then
            declare
               P : Scene_Pt := Scene (I);
            begin
               P.U := Now (I).U; P.V := Now (I).V;
               Cur.Append (P);
            end;
         end if;
      end loop;
      Rep.Matched := Natural (Cur.Length);
      if Gn.F <= 0.0 then
         return;   --  焦距都没有:没法核
      end if;
      --  三个候选:原来的位姿本身、从原位姿起步重解的、从零盲搜的(焦距已知 ⇒ 只解位姿)。拿同一道门数每个候选解释得了几个点:
      --  门 = 3 倍(倍数无量纲)"原来那份标定自己的像素残差"(它是按多细的配点解出来的,就按多细判;数值精度兜底 1e-9 px)。
      --  不按新解自己的残差定门:挡住的那半边 RoMa 不是乱配,是顺着看得见的那半边"编"出一片平滑的配点,一个错的位姿能以 4 px 的残差把它们全吃下,
      --  门跟着放到 12 px,就把"挡住"判成"挪了 7.6°、10.6 cm"(X5C 2026-09-25)。按原来那份的精度判,编出来的那片只有粗解得了,细的门里没有它
      --  新位姿的门另按"仪器转着看时配得多细"放宽到 max(细门, 3 倍 Turn_Sd):RoMa 转 90° 配点噪声约 0.8 px,标定时残差只有 0.16 px 的眼
      --  真被转了,按细门只数得到三成的点,会被"至少四分之一"那条挡掉(X5B 的数)。原来的位姿仍按细门数:小挪也抓得到
      declare
         Gate : constant Long_Float := 3.0 * Long_Float'Max (1.0e-9, G.Rms);
         Gate_New : constant Long_Float := Long_Float'Max (Gate, 3.0 * Turn_Sd);
         function Count (Pg : Cam_Geo; Gt : Long_Float) return Natural is
            K : Natural := 0;
         begin
            for P of Cur loop
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Project_Fixed (Pg, P.Pw, U, V, Front);
                  if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gt then
                     K := K + 1;
                  end if;
               end;
            end loop;
            return K;
         end Count;
         Ga : Cam_Geo := G;
         Fa, Fb : Fixed_Report;
         Oka, Okb : Boolean;
         Na, Nb : Natural := 0;
      begin
         Tally (G, Gate, Rep.Consistent_Now, Reg_Now);
         Rep.Gate := Gate;
         Fit_Fixed_Board (Ga, Cur, Fa, Oka, Start_Here => True);
         Fit_Fixed_Board (Gn, Cur, Fb, Okb);
         if Oka then
            Na := Count (Ga, Gate_New);
         end if;
         if Okb then
            Nb := Count (Gn, Gate_New);
         end if;
         if Oka and then (not Okb or else Na >= Nb) then
            Gn := Ga; Fr := Fa; Ok := True; Rep.Consistent := Na;
         elsif Okb then
            Fr := Fb; Ok := True; Rep.Consistent := Nb;
         else
            Ok := False;
         end if;
      end;
      if not Ok then
         Rep.Covered := True;   --  配到的点和任何一个位姿都对不上:看不见了 / 挡住了
         return;
      end if;
      Rep.Rms := Fr.Scene_Rms;
      Rep.Turn_Deg := Norm (Rot_Vec (Mul (Tr (G.R_Ce), Gn.R_Ce))) * Deg;
      Rep.Move_M := Norm ([Gn.Pos (0) - G.Pos (0), Gn.Pos (1) - G.Pos (1), Gn.Pos (2) - G.Pos (2)]);
      --  新解把板上的点投到的地方比原位姿投到的挪了多少(报数用)
      declare
         Px : Param_Vec (0 .. Natural'Max (1, Natural (Cur.Length)) - 1) := [others => 0.0];
         N : Natural := 0;
      begin
         for P of Cur loop
            declare
               U0, V0, U1, V1 : Long_Float;
               F0, F1 : Boolean;
            begin
               Project_Fixed (G, P.Pw, U0, V0, F0);
               Project_Fixed (Gn, P.Pw, U1, V1, F1);
               if F0 and then F1 then
                  Px (N) := Sqrt ((U1 - U0) ** 2 + (V1 - V0) ** 2);
                  N := N + 1;
               end if;
            end;
         end loop;
         Rep.Shift_Px := Median (Px, N);
         Rep.Shift_Sd := (if G.Rms > 0.0 then Rep.Shift_Px / G.Rms else 0.0);
      end;
      --  挪过 = 三条都成立:原来的位姿解释得不到新解一半(比例);新解至少解释得了放好以来最多那次的四分之一(比例,同"挡住"那条的四分之三);
      --  新旧位姿投出来的板点差得比细门远(差不到门里 = 同一个位姿,只是这会儿配得糙)。
      --  看得见、配得上的不到四分之一时解出来的位姿不可信 —— X5C4 2026-09-26 转 90° 重标后再挡一半,仪器整幅配飞,此刻的位姿一个点都解释不了,
      --  一份错得离谱的位姿以 18.9 px 的残差在门里凑到 35/782 个,就被当成"挪了 0.84 m"换上了。三条不全 ⇒ 按挡没挡报,位姿不动
      if (if Base_Now >= 0 then 2 * Base_Now else 2 * Rep.Consistent_Now) < Rep.Consistent and then 4 * Rep.Consistent >= Best.All_N and then Rep.Shift_Px > Rep.Gate then
         Rep.Moved := True;
         Gn.F := G.F; Gn.F_Meas := G.F_Meas; Gn.F_Sd := G.F_Sd;   --  焦距照旧
         --  以后按新解配得多细来判:新位姿解释得了的那些点(新门内)像素误差的中位 × 1.2(换算,无量纲:二维高斯误差中位 ≈ 均方根 ÷ 1.2)。
         --  不拿解的时候那份没加权的均方根:加权挑点留下了三角得不准的点,它们的大误差把均方根抬到 1.31 px(标定时 0.16),门跟着放到 3.9 px(X5E2 2026-09-26)
         declare
            Es : Param_Vec (0 .. Natural'Max (1, Natural (Cur.Length)) - 1) := [others => 0.0];
            Ne : Natural := 0;
            Gate_New : constant Long_Float := Long_Float'Max (3.0 * Long_Float'Max (1.0e-9, G.Rms), 3.0 * Turn_Sd);
         begin
            for P of Cur loop
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Project_Fixed (Gn, P.Pw, U, V, Front);
                  if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gate_New then
                     Es (Ne) := Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2);
                     Ne := Ne + 1;
                  end if;
               end;
            end loop;
            Gn.Rms := (if Ne > 0 then 1.2 * Median (Es, Ne) else Fr.Scene_Rms);   --  误差中位 → 均方根(换算,无量纲)
            Rep.Rms := Gn.Rms;
         end;
         G := Gn;
         --  重新放好了:从这一刻起重记"看见过的最多"—— 按新位姿自己的门数(以后每轮按它数;09-27 以前记的是按放宽的新门数的那份,
         --  下一轮按自己的门数就少一截,V1B39 重标后 630 → 569,凭空离"挡住了"近了一截)
         declare
            N_Own : Natural;
            Reg_Own : Region_Counts;
         begin
            Tally (G, 3.0 * Long_Float'Max (1.0e-9, G.Rms), N_Own, Reg_Own);
            Best := (All_N => N_Own, Region => Reg_Own);
         end;
      else
         Best.All_N := Natural'Max (Best.All_N, Rep.Consistent_Now);
         for R in Reg_Now'Range loop
            Best.Region (R) := Natural'Max (Best.Region (R), Reg_Now (R));
         end loop;
         --  看不全 = 整幅比放好以来最多的少了四分之一以上(比例);或者画面的某一块(一半 / 四分之一;放好以来在那儿至少看见过 Min_Pts 个)
         --  少了四分之三以上 —— 那一块基本看不见了(比例)。09-27 V1B39:转过 90° 以后板上的点多在右边,挡住左半只挡掉整幅的 25%,整幅那条擦线没报;
         --  左半那一块其实一个都不剩
         Rep.Covered := 4 * Rep.Consistent_Now < 3 * Best.All_N;
         for R in Reg_Now'Range loop
            if Best.Region (R) >= Min_Pts and then 4 * Reg_Now (R) < Best.Region (R)
              and then (Rep.Dark < 0 or else Reg_Now (R) * Rep.Dark_Best < Rep.Dark_Now * Best.Region (R))   --  剩得比例最少的那块
            then
               Rep.Covered := True;
               Rep.Dark := R; Rep.Dark_Now := Reg_Now (R); Rep.Dark_Best := Best.Region (R);
            end if;
         end loop;
      end if;
   end Check_Fixed;

   function Tips_On_Plane (G : Cam_Geo; Views : Board_View_Vectors.Vector; P0, N : V3; Sd_Plane : Long_Float) return Plane_Tip_Vectors.Vector is
      R : Plane_Tip_Vectors.Vector;
   begin
      for Vw of Views loop
         declare
            O : constant V3 := Cam_Pos (G, Vw.Pose);
            D : constant V3 := Ray (G, Vw.Pose, Vw.U, Vw.V);
            Dn : constant Long_Float := D (0) * N (0) + D (1) * N (1) + D (2) * N (2);
            H : constant Long_Float := (P0 (0) - O (0)) * N (0) + (P0 (1) - O (1)) * N (1) + (P0 (2) - O (2)) * N (2);
            T : Plane_Tip;
         begin
            if Dn /= 0.0 and then H / Dn > 0.0 then   --  朝着面、交在眼前
               T.S := H / Dn;
               T.Sd := Sd_Plane / abs Dn;
               T.Pw := [O (0) + T.S * D (0), O (1) + T.S * D (1), O (2) + T.S * D (2)];
               T.Ok := True;
            end if;
            R.Append (T);
         end;
      end loop;
      return R;
   end Tips_On_Plane;

   function Tips_On_Rays (Fixed : Cam_Geo; O : Obs_Pt_Vectors.Vector; Ray_O : V3; Ray_D : V3_Vectors.Vector; Gate_Px : Long_Float) return Ray_Tip_Vectors.Vector is
      package LF_Vectors is new Ada.Containers.Vectors (Natural, Long_Float);
      Nr : constant Natural := Natural (Ray_D.Length);
      Ss : array (0 .. Natural'Max (1, Nr) - 1) of LF_Vectors.Vector;
      Res : Ray_Tip_Vectors.Vector;
      function Med (V : LF_Vectors.Vector) return Long_Float is   --  中位数(拷一份插入排序;几十个数)
         A : LF_Vectors.Vector := V;
         N : constant Natural := Natural (A.Length);
      begin
         if N = 0 then
            return 0.0;
         end if;
         for I in 1 .. N - 1 loop
            declare
               X : constant Long_Float := A (I);
               J : Integer := I - 1;
            begin
               while J >= 0 and then A (J) > X loop
                  A.Replace_Element (J + 1, A (J)); J := J - 1;
               end loop;
               A.Replace_Element (J + 1, X);
            end;
         end loop;
         return A (N / 2);
      end Med;
   begin
      if Fixed.F > 0.0 then
         for Ob of O loop
            declare
               Rh : constant M3 := Quat_To_R (Ob.Pose);
               Ow : constant V3 := Ap (Rh, Ray_O);
               A : constant V3 := [Ob.Pose (0) + Ow (0), Ob.Pose (1) + Ow (1), Ob.Pose (2) + Ow (2)];   --  这只手的眼在世界里
               E0 : constant V3 := Ap (Fixed.R_Ce, Cam_Dir (Fixed, Ob.U, Ob.V));
               En : constant Long_Float := Norm (E0);
               E : constant V3 := [E0 (0) / En, E0 (1) / En, E0 (2) / En];   --  不动的眼过这个尖的视线(单位)
               W0 : constant V3 := [A (0) - Fixed.Pos (0), A (1) - Fixed.Pos (1), A (2) - Fixed.Pos (2)];
               Best_K : Integer := -1;
               Best_Gap : Long_Float := Long_Float'Last;
               Best_S : Long_Float := 0.0;
            begin
               for K in 0 .. Nr - 1 loop
                  declare
                     B0 : constant V3 := Ap (Rh, Ray_D (K));
                     Bn : constant Long_Float := Norm (B0);
                  begin
                     if Bn > 0.0 then
                        declare
                           B : constant V3 := [B0 (0) / Bn, B0 (1) / Bn, B0 (2) / Bn];
                           Bb : constant Long_Float := B (0) * E (0) + B (1) * E (1) + B (2) * E (2);
                           Dd : constant Long_Float := B (0) * W0 (0) + B (1) * W0 (1) + B (2) * W0 (2);
                           Ee : constant Long_Float := E (0) * W0 (0) + E (1) * W0 (1) + E (2) * W0 (2);
                           Den : constant Long_Float := 1.0 - Bb * Bb;   --  两条单位方向的 1 − cos²(纯数学)
                        begin
                           if Den > 0.0 then
                              declare
                                 S : constant Long_Float := (Bb * Ee - Dd) / Den;   --  两条直线最近点在这条视线上的参数(纯数学)
                                 P : constant V3 := [A (0) + S * B (0), A (1) + S * B (1), A (2) + S * B (2)];
                                 U, V : Long_Float;
                                 Front : Boolean;
                              begin
                                 Project_Fixed (Fixed, P, U, V, Front);
                                 if Front and then S > 0.0 then
                                    declare
                                       Gap : constant Long_Float := Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2);
                                    begin
                                       if Gap < Best_Gap then
                                          Best_Gap := Gap; Best_K := Integer (K); Best_S := S;
                                       end if;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                     end if;
                  end;
               end loop;
               if Best_K >= 0 and then Best_Gap <= Gate_Px then
                  Ss (Natural (Best_K)).Append (Best_S);
               end if;
            end;
         end loop;
      end if;
      for K in 0 .. Nr - 1 loop
         declare
            R : Ray_Tip;
            Dev : LF_Vectors.Vector;
         begin
            R.N := Natural (Ss (K).Length);
            R.S := Med (Ss (K));
            for X of Ss (K) loop
               Dev.Append (abs (X - R.S));
            end loop;
            R.Spread := Med (Dev);
            Res.Append (R);
         end;
      end loop;
      return Res;
   end Tips_On_Rays;

   function Meet (Rays : Sight_Vectors.Vector; Ok : out Boolean; Spread : out Long_Float) return V3 is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [others => 0.0];
      P : V3 := [others => 0.0];
   begin
      Ok := False; Spread := 0.0;
      if Natural (Rays.Length) < 2 then
         return P;
      end if;
      for R of Rays loop
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               declare
                  Pm : constant Long_Float := (if I = J then 1.0 else 0.0) - R.D (I) * R.D (J);
               begin
                  A (I, J) := A (I, J) + Pm;
                  B (I) := B (I) + Pm * R.O (J);
               end;
            end loop;
         end loop;
      end loop;
      --  视线全平行时 A 退化(行列式为零),交点没有意义
      declare
         Det : constant Long_Float :=
           A (0, 0) * (A (1, 1) * A (2, 2) - A (1, 2) * A (2, 1))
           - A (0, 1) * (A (1, 0) * A (2, 2) - A (1, 2) * A (2, 0))
           + A (0, 2) * (A (1, 0) * A (2, 1) - A (1, 1) * A (2, 0));
      begin
         if abs Det < 1.0e-9 then
            return P;
         end if;
      end;
      P := Solve3 (A, B);
      for R of Rays loop
         declare
            W : constant V3 := [P (0) - R.O (0), P (1) - R.O (1), P (2) - R.O (2)];
            T : constant Long_Float := W (0) * R.D (0) + W (1) * R.D (1) + W (2) * R.D (2);
            Perp : constant V3 := [W (0) - T * R.D (0), W (1) - T * R.D (1), W (2) - T * R.D (2)];
         begin
            Spread := Long_Float'Max (Spread, Norm (Perp));
            if T <= 0.0 then
               return P;      --  交点在某只眼的背后 ⇒ 不是它,Ok 留 False
            end if;
         end;
      end loop;
      Ok := True;
      return P;
   end Meet;

   --  ── 存 / 读(自己的小文件,一行一台相机,坏一行不毁整份)──
   procedure Save (Path : String; Gs : Geo_Vectors.Vector) is
      Fh : Ada.Text_IO.File_Type;
      B : Unbounded_String;
   begin
      Append (B, "{""cams"":[");
      for K in 0 .. Natural (Gs.Length) - 1 loop
         declare
            G : constant Cam_Geo := Gs (K);
         begin
            if K > 0 then
               Append (B, ",");
            end if;
            Append (B, "{""cam"":" & Codec.Img (K) & ",""valid"":" & (if G.Valid then "true" else "false") &
                      ",""f"":" & Codec.Fmt (G.F, 4) & ",""cx"":" & Codec.Fmt (G.Cx, 3) & ",""cy"":" & Codec.Fmt (G.Cy, 3) & ",""k1"":" & Codec.Fmt (G.K1, 6)
                      & ",""k2"":" & Codec.Fmt (G.K2, 6) & ",""rms"":" & Codec.Fmt (G.Rms, 3) & ",""r_ce"":[");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Append (B, (if I + J > 0 then "," else "") & Codec.Fmt (G.R_Ce (I, J), 7));
               end loop;
            end loop;
            Append (B, "],""tip_valid"":" & (if G.Tip_Valid then "true" else "false") & ",""tip_touch"":" & (if G.Tip_Touch then "true" else "false") & ",""tip"":[" & Codec.Fmt (G.Tip (0), 5) & "," & Codec.Fmt (G.Tip (1), 5) & "," &
                      Codec.Fmt (G.Tip (2), 5) & "],""gap"":" & Codec.Fmt (G.Gap, 5) & ",""stride"":" & Codec.Fmt (G.Stride, 5) & ",""stride_rot"":" & Codec.Fmt (G.Stride_Rot, 5) &
                      ",""f_meas"":" & Codec.Fmt (G.F_Meas, 3) & ",""f_prior"":" & Codec.Fmt (G.F_Prior, 3) & ",""f_prior_sd"":" & Codec.Fmt (G.F_Prior_Sd, 3) &
                      ",""off"":[" & Codec.Fmt (G.Off (0), 5) & "," & Codec.Fmt (G.Off (1), 5) & "," & Codec.Fmt (G.Off (2), 5) & "]" &
                      ",""fixed"":" & (if G.Fixed then "true" else "false") & ",""pos"":[" & Codec.Fmt (G.Pos (0), 5) & "," & Codec.Fmt (G.Pos (1), 5) & "," & Codec.Fmt (G.Pos (2), 5) & "]}");
         end;
      end loop;
      Append (B, "]}");
      Ada.Text_IO.Create (Fh, Ada.Text_IO.Out_File, Path);
      Ada.Text_IO.Put_Line (Fh, To_String (B));
      Ada.Text_IO.Close (Fh);
   end Save;

   procedure Load (Path : String; Gs : in out Geo_Vectors.Vector; N_Cams : Natural; Note : out String) is
      Fh : Ada.Text_IO.File_Type;
      Src : Unbounded_String;
      D : Json.Doc;
      Err : Unbounded_String;
      procedure Put_Note (S : String) is
      begin
         Note := [others => ' '];
         Note (Note'First .. Note'First + Integer'Min (S'Length, Note'Length) - 1) := S (S'First .. S'First + Integer'Min (S'Length, Note'Length) - 1);
      end Put_Note;
   begin
      Gs.Clear;
      for K in 0 .. N_Cams - 1 loop
         Gs.Append (No_Geo);
      end loop;
      begin
         Ada.Text_IO.Open (Fh, Ada.Text_IO.In_File, Path);
      exception
         when others =>
            Put_Note ("没有几何文件,从零量");
            return;
      end;
      while not Ada.Text_IO.End_Of_File (Fh) loop
         Append (Src, Ada.Text_IO.Get_Line (Fh));
      end loop;
      Ada.Text_IO.Close (Fh);
      if not Json.Parse (To_String (Src), D, Err) then
         Put_Note ("几何文件读不回:" & To_String (Err));
         return;
      end if;
      declare
         Cams : constant Integer := Json.Get (D, 0, "cams");
         Loaded : Natural := 0;
      begin
         if Cams < 0 then
            Put_Note ("几何文件里没有 cams");
            return;
         end if;
         for I in 0 .. Json.Count (D, Cams) - 1 loop
            declare
               Nd : constant Integer := Json.Child (D, Cams, I);
               K : constant Integer := Integer (Json.Num (D, Json.Get (D, Nd, "cam")));
               G : Cam_Geo;
               Rn : constant Integer := Json.Get (D, Nd, "r_ce");
               Tn : constant Integer := Json.Get (D, Nd, "tip");
            begin
               if K >= 0 and then K < N_Cams then
                  G.Valid := Json.Bool (D, Json.Get (D, Nd, "valid"));
                  G.F := Json.Num (D, Json.Get (D, Nd, "f")); G.Cx := Json.Num (D, Json.Get (D, Nd, "cx")); G.Cy := Json.Num (D, Json.Get (D, Nd, "cy"));
                  G.K1 := Json.Num (D, Json.Get (D, Nd, "k1")); G.K2 := Json.Num (D, Json.Get (D, Nd, "k2"));   --  旧文件没有 ⇒ 0(理想针孔)
                  G.Rms := Json.Num (D, Json.Get (D, Nd, "rms"));
                  if Rn >= 0 and then Json.Count (D, Rn) = 9 then
                     for A in 0 .. 2 loop
                        for B in 0 .. 2 loop
                           G.R_Ce (A, B) := Json.Num (D, Json.Child (D, Rn, A * 3 + B));
                        end loop;
                     end loop;
                  else
                     G.Valid := False;
                  end if;
                  G.Tip_Valid := Json.Bool (D, Json.Get (D, Nd, "tip_valid"));
                  G.Tip_Touch := Json.Bool (D, Json.Get (D, Nd, "tip_touch"));   --  旧文件没有 ⇒ False(按头顶眼交的,不算)
                  if Tn >= 0 and then Json.Count (D, Tn) = 3 then
                     for A in 0 .. 2 loop
                        G.Tip (A) := Json.Num (D, Json.Child (D, Tn, A));
                     end loop;
                  else
                     G.Tip_Valid := False;
                  end if;
                  G.Gap := Json.Num (D, Json.Get (D, Nd, "gap"));
                  declare
                     Sn : constant Integer := Json.Get (D, Nd, "stride");   --  老文件没有这一项 ⇒ 0,开机再量
                     Sr : constant Integer := Json.Get (D, Nd, "stride_rot");
                     Fm : constant Integer := Json.Get (D, Nd, "f_meas");
                     Fp : constant Integer := Json.Get (D, Nd, "f_prior");
                     Fs : constant Integer := Json.Get (D, Nd, "f_prior_sd");
                  begin
                     if Sn >= 0 then
                        G.Stride := Json.Num (D, Sn);
                     end if;
                     if Sr >= 0 then
                        G.Stride_Rot := Json.Num (D, Sr);
                     end if;
                     if Fm >= 0 then
                        G.F_Meas := Json.Num (D, Fm);
                     end if;
                     if Fp >= 0 and then Fs >= 0 then
                        G.F_Prior := Json.Num (D, Fp); G.F_Prior_Sd := Json.Num (D, Fs);
                     end if;
                     declare
                        On : constant Integer := Json.Get (D, Nd, "off");   --  老文件没有 ⇒ 0
                     begin
                        if On >= 0 and then Json.Count (D, On) = 3 then
                           for A in 0 .. 2 loop
                              G.Off (A) := Json.Num (D, Json.Child (D, On, A));
                           end loop;
                        end if;
                     end;
                  end;
                  declare
                     Fx : constant Integer := Json.Get (D, Nd, "fixed");
                     Pn : constant Integer := Json.Get (D, Nd, "pos");
                  begin
                     G.Fixed := Fx >= 0 and then Json.Bool (D, Fx) and then Pn >= 0 and then Json.Count (D, Pn) = 3;
                     if G.Fixed then
                        for A in 0 .. 2 loop
                           G.Pos (A) := Json.Num (D, Json.Child (D, Pn, A));
                        end loop;
                     end if;
                  end;
                  Gs.Replace_Element (K, G);
                  Loaded := Loaded + 1;
               end if;
            end;
         end loop;
         Put_Note ("装回几何文件:" & Codec.Img (Loaded) & " 台相机");
      end;
   end Load;
end Geom;
