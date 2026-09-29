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
   function Round_Trip_Ok (Qu, Qv, Bu, Bv : Long_Float) return Boolean is
     (Bu >= 0.0 and then Bv >= 0.0 and then Sqrt ((Bu - Qu) ** 2 + (Bv - Qv) ** 2) < Trip_Px);
   function Board_Rms (G : Cam_Geo; Pts : Scene_Pt_Vectors.Vector; Gate : Long_Float) return Long_Float is
      Es : Param_Vec (0 .. Natural'Max (1, Natural (Pts.Length)) - 1) := [others => 0.0];
      Ne : Natural := 0;
   begin
      for P of Pts loop
         if P.U >= 0.0 and then P.V >= 0.0 then
            declare
               U, V : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (G, P.Pw, U, V, Front);
               if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gate then
                  Es (Ne) := Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2);
                  Ne := Ne + 1;
               end if;
            end;
         end if;
      end loop;
      return (if Ne > 0 then 1.2 * Median (Es, Ne) else 0.0);   --  误差中位 → 均方根(换算,无量纲)
   end Board_Rms;

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
            Gate_New : constant Long_Float := Long_Float'Max (3.0 * Long_Float'Max (1.0e-9, G.Rms), 3.0 * Turn_Sd);
            Br : constant Long_Float := Board_Rms (Gn, Cur, Gate_New);
         begin
            Gn.Rms := (if Br > 0.0 then Br else Fr.Scene_Rms);
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

   function Press_Of (G : Cam_Geo; P : Plug.Arm_Pose; P0, N : V3) return Press_Eq is
      R : constant M3 := Cam_R (G, P);
      T : constant V3 := Cam_Pos (G, P);
      E : Press_Eq;
   begin
      E.A := Ap (Tr (R), N);
      E.B := (P0 (0) - T (0)) * N (0) + (P0 (1) - T (1)) * N (1) + (P0 (2) - T (2)) * N (2);
      return E;
   end Press_Of;

   function Fit_Presses (Eqs : Press_Eq_Vectors.Vector; Gate : Long_Float) return Press_Fit is
      Ai : Nat_Vectors.Vector;   --  对准这一瓣的那几下
      Max_Enum : constant := 16;   --  枚举的上限(次数:2^16 组;调用方一瓣最多压 8 下)
      Min_Set : constant := 4;     --  3 个未知数 + 1 条自己核(次数)
      function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
      type Cand is record
         Fit : Press_Fit;
      end record;
      package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
      Cs : Cand_Vectors.Vector;
      Best_Size : Natural := 0;
      Res : Press_Fit;
      --  3×3 对称阵求逆(余子式);行列式相对 (迹/3)³ 太小 ⇒ 三个方向分不开(数值保护)
      procedure Inv3 (M : M3; Mi : out M3; Ok : out Boolean) is
         Det : Long_Float;
         Tr3 : constant Long_Float := (M (0, 0) + M (1, 1) + M (2, 2)) / 3.0;
      begin
         Mi (0, 0) := M (1, 1) * M (2, 2) - M (1, 2) * M (2, 1);
         Mi (0, 1) := M (0, 2) * M (2, 1) - M (0, 1) * M (2, 2);
         Mi (0, 2) := M (0, 1) * M (1, 2) - M (0, 2) * M (1, 1);
         Mi (1, 0) := M (1, 2) * M (2, 0) - M (1, 0) * M (2, 2);
         Mi (1, 1) := M (0, 0) * M (2, 2) - M (0, 2) * M (2, 0);
         Mi (1, 2) := M (0, 2) * M (1, 0) - M (0, 0) * M (1, 2);
         Mi (2, 0) := M (1, 0) * M (2, 1) - M (1, 1) * M (2, 0);
         Mi (2, 1) := M (0, 1) * M (2, 0) - M (0, 0) * M (2, 1);
         Mi (2, 2) := M (0, 0) * M (1, 1) - M (0, 1) * M (1, 0);
         Det := M (0, 0) * Mi (0, 0) + M (0, 1) * Mi (1, 0) + M (0, 2) * Mi (2, 0);
         Ok := Tr3 > 0.0 and then Det > 1.0e-12 * Tr3 ** 3;
         if Ok then
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Mi (I, J) := Mi (I, J) / Det;
               end loop;
            end loop;
         end if;
      end Inv3;
      function In_Mask (Mask, K : Natural) return Boolean is ((Mask / 2 ** K) mod 2 = 1);
      --  组里这几下(去掉 Skip 那一下;-1 = 不去)的最小二乘解
      procedure Solve (Mask : Natural; Skip : Integer; X : out V3; Mi : out M3; Ok : out Boolean) is
         M : M3 := [others => [others => 0.0]];
         V : V3 := [others => 0.0];
      begin
         for J in 0 .. Natural (Ai.Length) - 1 loop
            if In_Mask (Mask, J) and then J /= Skip then
               declare
                  E : constant Press_Eq := Eqs (Ai (J));
               begin
                  for R in 0 .. 2 loop
                     for S in 0 .. 2 loop
                        M (R, S) := M (R, S) + E.A (R) * E.A (S);
                     end loop;
                     V (R) := V (R) + E.A (R) * E.B;
                  end loop;
               end;
            end if;
         end loop;
         Inv3 (M, Mi, Ok);
         X := (if Ok then Ap (Mi, V) else [0.0, 0.0, 0.0]);
      end Solve;
   begin
      for I in 0 .. Natural (Eqs.Length) - 1 loop
         if Eqs (I).Aimed then
            Ai.Append (I);
         end if;
      end loop;
      if Natural (Ai.Length) < Min_Set or else Natural (Ai.Length) > Max_Enum then
         return Res;
      end if;
      for Mask in 1 .. 2 ** Natural (Ai.Length) - 1 loop
         declare
            K : Natural := 0;
         begin
            for J in 0 .. Natural (Ai.Length) - 1 loop
               if In_Mask (Mask, J) then
                  K := K + 1;
               end if;
            end loop;
            if K >= Min_Set and then K >= Best_Size then
               declare
                  Mi : M3;
                  Inv_Ok : Boolean;
                  F : Press_Fit;
                  Good : Boolean := True;
               begin
                  Solve (Mask, -1, F.X, Mi, Inv_Ok);
                  Good := Inv_Ok;
                  --  组里每一下:拿别的几下解、预测它
                  for J in 0 .. Natural (Ai.Length) - 1 loop
                     exit when not Good;
                     if In_Mask (Mask, J) then
                        declare
                           Xo : V3;
                           Mo : M3;
                           Oo : Boolean;
                        begin
                           Solve (Mask, J, Xo, Mo, Oo);
                           if not Oo then
                              Good := False;
                           else
                              F.Worst := Long_Float'Max (F.Worst, abs (Dot (Eqs (Ai (J)).A, Xo) - Eqs (Ai (J)).B));
                              Good := F.Worst <= Gate;
                           end if;
                        end;
                     end if;
                  end loop;
                  if Good then
                     declare
                        Ss : Long_Float := 0.0;
                        Low : Long_Float := Long_Float'Last;
                        Inside : Boolean;
                     begin
                        for I in 0 .. Natural (Eqs.Length) - 1 loop
                           declare
                              R : constant Long_Float := Dot (Eqs (I).A, F.X) - Eqs (I).B;
                           begin
                              Inside := False;
                              for J in 0 .. Natural (Ai.Length) - 1 loop
                                 if Ai (J) = I and then In_Mask (Mask, J) then
                                    Inside := True;
                                 end if;
                              end loop;
                              if Inside then
                                 F.Used.Append (I);
                                 Ss := Ss + R * R;
                              else
                                 Low := Long_Float'Min (Low, R);
                              end if;
                           end;
                        end loop;
                        F.Low := (if Low = Long_Float'Last then 0.0 else Low);
                        declare
                           Sig : constant Long_Float := Sqrt (Ss / Long_Float (K - 3));   --  自由度 = 下数 − 3 个未知数
                        begin
                           for R in 0 .. 2 loop
                              F.Sd (R) := Sig * Sqrt (Long_Float'Max (0.0, Mi (R, R)));
                           end loop;
                        end;
                        if F.Low >= -Gate then
                           F.Ok := True;
                           if K > Best_Size then
                              Cs.Clear;
                              Best_Size := K;
                           end if;
                           Cs.Append (Cand'(Fit => F));
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
      if Cs.Is_Empty then
         return Res;
      end if;
      --  一样大的组不止一组:两组的解对两组里的每一下差都在 Gate 以内 = 同一个解(取预测差得最少的那组);差过 Gate = 认不出哪一下是坏的
      declare
         Pick : Natural := 0;
      begin
         for I in 0 .. Natural (Cs.Length) - 1 loop
            for J in I + 1 .. Natural (Cs.Length) - 1 loop
               declare
                  Dx : constant V3 := [Cs (I).Fit.X (0) - Cs (J).Fit.X (0), Cs (I).Fit.X (1) - Cs (J).Fit.X (1), Cs (I).Fit.X (2) - Cs (J).Fit.X (2)];
               begin
                  for U of Cs (I).Fit.Used loop
                     if abs Dot (Eqs (U).A, Dx) > Gate then
                        Res.Ambiguous := True;
                     end if;
                  end loop;
                  for U of Cs (J).Fit.Used loop
                     if abs Dot (Eqs (U).A, Dx) > Gate then
                        Res.Ambiguous := True;
                     end if;
                  end loop;
               end;
            end loop;
            if Cs (I).Fit.Worst < Cs (Pick).Fit.Worst then
               Pick := I;
            end if;
         end loop;
         if Res.Ambiguous then
            return Res;
         end if;
         return Cs (Pick).Fit;
      end;
   end Fit_Presses;

   function Tilt_Dir (D : V3; Tilt, Azim : Long_Float) return V3 is
      function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
      X : constant V3 := [1.0, 0.0, 0.0];
      Y : constant V3 := [0.0, 1.0, 0.0];
      Xp : V3 := [X (0) - Dot (X, D) * D (0), X (1) - Dot (X, D) * D (1), X (2) - Dot (X, D) * D (2)];
   begin
      if Norm (Xp) < 1.0e-9 then   --  数值保护:视线正好沿着眼的 x 轴(画面里看不到)⇒ 从 y 轴起量
         Xp := [Y (0) - Dot (Y, D) * D (0), Y (1) - Dot (Y, D) * D (1), Y (2) - Dot (Y, D) * D (2)];
      end if;
      declare
         L : constant Long_Float := Norm (Xp);
         E1 : constant V3 := [Xp (0) / L, Xp (1) / L, Xp (2) / L];
         E2 : constant V3 := [D (1) * E1 (2) - D (2) * E1 (1), D (2) * E1 (0) - D (0) * E1 (2), D (0) * E1 (1) - D (1) * E1 (0)];
         C : constant Long_Float := Cos (Tilt);
         S : constant Long_Float := Sin (Tilt);
      begin
         return [C * D (0) + S * (Cos (Azim) * E1 (0) + Sin (Azim) * E2 (0)),
                 C * D (1) + S * (Cos (Azim) * E1 (1) + Sin (Azim) * E2 (1)),
                 C * D (2) + S * (Cos (Azim) * E1 (2) + Sin (Azim) * E2 (2))];
      end;
   end Tilt_Dir;

   function Tilt_Angle (D : V3_Vectors.Vector; K : Natural; Single : Long_Float) return Long_Float is
      Beta : Long_Float := Long_Float'Last;
   begin
      for J in 0 .. Natural (D.Length) - 1 loop
         if J /= K then
            Beta := Long_Float'Min (Beta, Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, D (K) (0) * D (J) (0) + D (K) (1) * D (J) (1) + D (K) (2) * D (J) (2)))));
         end if;
      end loop;
      return (if Beta = Long_Float'Last then Single else (1.0 / 3.0) * Beta);
   end Tilt_Angle;

   function Ray_Owner (X : V3; D : V3_Vectors.Vector) return Natural is
      Best : Natural := Natural'Last;
      Bd : Long_Float := Long_Float'Last;
   begin
      for J in 0 .. Natural (D.Length) - 1 loop
         declare
            T : constant Long_Float := X (0) * D (J) (0) + X (1) * D (J) (1) + X (2) * D (J) (2);
            Off : constant Long_Float := Norm ([X (0) - T * D (J) (0), X (1) - T * D (J) (1), X (2) - T * D (J) (2)]);
         begin
            if T > 0.0 and then Off < Bd then
               Bd := Off; Best := J;
            end if;
         end;
      end loop;
      return Best;
   end Ray_Owner;

   function Turn_To (Fwd, Down : V3) return V3 is
      Cr : constant V3 := [Fwd (1) * Down (2) - Fwd (2) * Down (1), Fwd (2) * Down (0) - Fwd (0) * Down (2), Fwd (0) * Down (1) - Fwd (1) * Down (0)];
      Sn : constant Long_Float := Norm (Cr);
      Ang : constant Long_Float := Arctan (Sn, Fwd (0) * Down (0) + Fwd (1) * Down (1) + Fwd (2) * Down (2));
      --  正好反向(叉积为零)时随便取一根和它垂直的轴(同 Geo_Turn)
      Az : constant V3 := [Fwd (1), -Fwd (0), 0.0];
      Ax : constant V3 := [0.0, Fwd (2), -Fwd (1)];
      Alt : constant V3 := (if Norm (Az) >= Norm (Ax) then Az else Ax);
      Aln : constant Long_Float := Norm (Alt);
      Axis : constant V3 := (if Sn > 1.0e-9 then [Cr (0) / Sn, Cr (1) / Sn, Cr (2) / Sn]
                             elsif Aln > 1.0e-9 then [Alt (0) / Aln, Alt (1) / Aln, Alt (2) / Aln] else [0.0, 0.0, 1.0]);
   begin
      return [Axis (0) * Ang, Axis (1) * Ang, Axis (2) * Ang];
   end Turn_To;

   function Meet_Sd (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P, U : V3) return Long_Float is
      A : M3 := [others => [others => 0.0]];
   begin
      if Natural (Sds.Length) /= Natural (Rays.Length) or else Natural (Rays.Length) < 2 then
         return Long_Float'Last;
      end if;
      for K in 0 .. Natural (Rays.Length) - 1 loop
         declare
            R : constant Sight := Rays (K);
            T : constant Long_Float := (P (0) - R.O (0)) * R.D (0) + (P (1) - R.O (1)) * R.D (1) + (P (2) - R.O (2)) * R.D (2);
            S : constant Long_Float := Sds (K) * T;   --  这条视线在交点处垂直方向的位置噪声
         begin
            if Sds (K) <= 0.0 or else T <= 0.0 then
               return Long_Float'Last;
            end if;
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  A (I, J) := A (I, J) + ((if I = J then 1.0 else 0.0) - R.D (I) * R.D (J)) / (S * S);
               end loop;
            end loop;
         end;
      end loop;
      declare
         --  协方差 = A⁻¹;沿 U 的方差 = Uᵀ A⁻¹ U = U · X,X 解 A X = U(奇异 ⇒ Solve3 交零向量 ⇒ 这个方向量不出)。
         --  先按 A 的迹缩到 1 附近再解(Solve3 的奇异门是绝对数),解完再缩回去
         Un : constant Long_Float := Norm (U);
         Uu : constant V3 := (if Un > 0.0 then [U (0) / Un, U (1) / Un, U (2) / Un] else U);
         Sc : constant Long_Float := (A (0, 0) + A (1, 1) + A (2, 2)) / 3.0;
         An : M3;
         X : V3;
         Var : Long_Float;
      begin
         if Sc <= 0.0 then
            return Long_Float'Last;
         end if;
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               An (I, J) := A (I, J) / Sc;
            end loop;
         end loop;
         X := Solve3 (An, Uu);
         Var := (Uu (0) * X (0) + Uu (1) * X (1) + Uu (2) * X (2)) / Sc;
         if Un <= 0.0 or else Var <= 0.0 then
            return Long_Float'Last;
         end if;
         return Sqrt (Var);
      end;
   end Meet_Sd;

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
                      ",""fixed"":" & (if G.Fixed then "true" else "false") & ",""pos"":[" & Codec.Fmt (G.Pos (0), 5) & "," & Codec.Fmt (G.Pos (1), 5) & "," & Codec.Fmt (G.Pos (2), 5) & "]"
                      & ",""tip_sd"":" & Codec.Fmt (G.Tip_Sd, 6) & ",""lobes"":[");
            for Li in 0 .. Natural (G.Lobes.Length) - 1 loop
               declare
                  Lg : constant Lobe_Geo := G.Lobes (Li);
               begin
                  Append (B, (if Li > 0 then "," else "") & "{""tip"":[" & Codec.Fmt (Lg.Tip (0), 6) & "," & Codec.Fmt (Lg.Tip (1), 6) & "," & Codec.Fmt (Lg.Tip (2), 6)
                          & "],""wide"":" & Codec.Fmt (Lg.Wide, 6) & ",""thin"":" & Codec.Fmt (Lg.Thin, 6) & "}");
               end;
            end loop;
            Append (B, "]}");
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
                  --  每一瓣的尖(09-29 起存;老文件没有 ⇒ 空,接触集照实说"没量每一瓣")
                  declare
                     Ln : constant Integer := Json.Get (D, Nd, "lobes");
                  begin
                     G.Tip_Sd := Json.Num (D, Json.Get (D, Nd, "tip_sd"));
                     if Ln >= 0 then
                        for Li in 0 .. Json.Count (D, Ln) - 1 loop
                           declare
                              Lnd : constant Integer := Json.Child (D, Ln, Li);
                              Lt : constant Integer := Json.Get (D, Lnd, "tip");
                              Lg : Lobe_Geo;
                           begin
                              if Lt >= 0 and then Json.Count (D, Lt) = 3 then
                                 for A in 0 .. 2 loop
                                    Lg.Tip (A) := Json.Num (D, Json.Child (D, Lt, A));
                                 end loop;
                                 Lg.Wide := Json.Num (D, Json.Get (D, Lnd, "wide"));
                                 Lg.Thin := Json.Num (D, Json.Get (D, Lnd, "thin"));
                                 G.Lobes.Append (Lg);
                              end if;
                           end;
                        end loop;
                     end if;
                  end;
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
