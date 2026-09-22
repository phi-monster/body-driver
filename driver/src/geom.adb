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

   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3 is
      Dc : constant V3 := [(U - G.Cx) / G.F, -(V - G.Cy) / G.F, -1.0];
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
      D : constant V3 := [Pw (0) - P (0), Pw (1) - P (1), Pw (2) - P (2)];
   begin
      return Ap (Tr (Cam_R (G, P)), D);
   end To_Cam;

   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
      Pc : constant V3 := To_Cam (G, P, Pw);
      Z : constant Long_Float := -Pc (2);
   begin
      In_Front := Z > 1.0e-6;
      if not In_Front then
         U := 0.0; V := 0.0;
         return;
      end if;
      U := G.Cx + G.F * Pc (0) / Z;
      V := G.Cy - G.F * Pc (1) / Z;
   end Project;

   --  ── 量朝向 ──
   procedure Fit (G : in out Cam_Geo; O : Obs_Vectors.Vector; Ok : out Boolean) is
      --  LM 阻尼的升降倍数(次数,无量纲;只管求解器迭代,不管身体)
      Lm_Loosen : constant := 3;
      Lm_Tighten : constant := 10;
      N : constant Natural := Natural (O.Length);
      type Params is array (0 .. 5) of Long_Float;   --  转向量 3 + 那东西的世界位置 3
      Gen : Ada.Numerics.Float_Random.Generator;
      Best_Cost : Long_Float := Long_Float'Last;
      Best_P : Params := [others => 0.0];
      Have_Best : Boolean := False;

      function Rnd return Long_Float is (Long_Float (Ada.Numerics.Float_Random.Random (Gen)));

      --  残差:每个观测两个像素差;东西跑到相机后面就给一个大罚(1e3 像素,无量纲哨兵)
      procedure Resid (P : Params; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
         Gt : Cam_Geo := G;
         Sum : Long_Float := 0.0;
      begin
         Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         for I in 0 .. N - 1 loop
            declare
               U, V : Long_Float;
               Front : Boolean;
               Du, Dv : Long_Float;
            begin
               Project (Gt, O (I).Pose, [P (3), P (4), P (5)], U, V, Front);
               if Front then
                  Du := U - O (I).U; Dv := V - O (I).V;
               else
                  --  跑到相机后面:给一个远大于画幅的罚(像素数,无量纲哨兵)
                  Du := 1.0e3; Dv := 1.0e3;
               end if;
               Sum := Sum + Du * Du + Dv * Dv;
               if Fill /= null then
                  Fill (I, Du, Dv);
               end if;
            end;
         end loop;
         R := Sqrt (Sum / Long_Float (Natural'Max (1, N)));
      end Resid;

      function Cost (P : Params) return Long_Float is
         R : Long_Float;
      begin
         Resid (P, R, null);
         return R;
      end Cost;
   begin
      Ok := False;
      if N < 4 or else G.F <= 0.0 then
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
            P : Params;
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
                  P := [Rv (0), Rv (1), Rv (2), Pw (0), Pw (1), Pw (2)];
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
      --  Levenberg–Marquardt 精修:数值雅可比,6 个参数
      declare
         P : Params := Best_P;
         Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
         Cur : Long_Float := Best_Cost;
         type Jac is array (0 .. 2 * N - 1, 0 .. 5) of Long_Float;
         J : Jac;
         Rv : array (0 .. 2 * N - 1) of Long_Float;
         procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
         begin
            Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
         end Fill_R;
      begin
         for It in 1 .. 40 loop
            declare
               R0 : Long_Float;
            begin
               Resid (P, R0, Fill_R'Access);
               for K in 0 .. 5 loop
                  declare
                     H : constant Long_Float := (if K < 3 then 1.0e-4 else 1.0e-4);   --  差分步(弧度 / 米,极小量)
                     Pp : Params := P;
                     Rp : array (0 .. 2 * N - 1) of Long_Float;
                     procedure Fill_P (I : Natural; Du, Dv : Long_Float) is
                     begin
                        Rp (2 * I) := Du; Rp (2 * I + 1) := Dv;
                     end Fill_P;
                     Dummy : Long_Float;
                  begin
                     Pp (K) := Pp (K) + H;
                     Resid (Pp, Dummy, Fill_P'Access);
                     for I in 0 .. 2 * N - 1 loop
                        J (I, K) := (Rp (I) - Rv (I)) / H;
                     end loop;
                  end;
               end loop;
               --  正规方程 (JᵀJ + λ diag) δ = -Jᵀr,6×6 高斯消元
               declare
                  A : array (0 .. 5, 0 .. 5) of Long_Float := [others => [others => 0.0]];
                  B : Params := [others => 0.0];
                  Dlt : Params := [others => 0.0];
               begin
                  for K in 0 .. 5 loop
                     for M in 0 .. 5 loop
                        for I in 0 .. 2 * N - 1 loop
                           A (K, M) := A (K, M) + J (I, K) * J (I, M);
                        end loop;
                     end loop;
                     for I in 0 .. 2 * N - 1 loop
                        B (K) := B (K) - J (I, K) * Rv (I);
                     end loop;
                  end loop;
                  for K in 0 .. 5 loop
                     A (K, K) := A (K, K) * (1.0 + Lam) + 1.0e-12;
                  end loop;
                  for Col in 0 .. 5 loop
                     declare
                        Piv : Natural := Col;
                     begin
                        for Rw in Col + 1 .. 5 loop
                           if abs (A (Rw, Col)) > abs (A (Piv, Col)) then
                              Piv := Rw;
                           end if;
                        end loop;
                        if Piv /= Col then
                           for M in 0 .. 5 loop
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
                           for Rw in 0 .. 5 loop
                              if Rw /= Col then
                                 declare
                                    Fct : constant Long_Float := A (Rw, Col) / A (Col, Col);
                                 begin
                                    for M in 0 .. 5 loop
                                       A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                                    end loop;
                                    B (Rw) := B (Rw) - Fct * B (Col);
                                 end;
                              end if;
                           end loop;
                        end if;
                     end;
                  end loop;
                  for K in 0 .. 5 loop
                     Dlt (K) := (if abs (A (K, K)) > 1.0e-18 then B (K) / A (K, K) else 0.0);
                  end loop;
                  declare
                     Pn : Params := P;
                     Cn : Long_Float;
                  begin
                     for K in 0 .. 5 loop
                        Pn (K) := Pn (K) + Dlt (K);
                     end loop;
                     Cn := Cost (Pn);
                     --  🔴 这两个数不是门槛,也不影响身体动不动:它们只管【这次拟合怎么迭代】——
                     --  Levenberg-Marquardt 的阻尼升降(这一步让残差变小就松一点,变大就收紧)。
                     --  写成具名常数,好让读的人和棘轮都看得出它不是一个可调的物理系数。
                     if Cn < Cur then
                        P := Pn; Cur := Cn; Lam := Lam / Long_Float (Lm_Loosen);
                     else
                        Lam := Lam * Long_Float (Lm_Tighten);
                     end if;
                  end;
               end;
            end;
            exit when Lam > 1.0e6;
         end loop;
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         G.Rms := Cur;
         G.Valid := True;
         Ok := True;
      end;
   end Fit;

   --  ── 不动的眼 ──
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float) return V3 is
      Dc : constant V3 := [(U - G.Cx) / G.F, -(V - G.Cy) / G.F, -1.0];
      Dw : V3 := Ap (G.R_Ce, Dc);
      N : constant Long_Float := Norm (Dw);
   begin
      for I in 0 .. 2 loop
         Dw (I) := Dw (I) / N;
      end loop;
      return Dw;
   end Ray_Fixed;

   procedure Project_Fixed (G : Cam_Geo; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
      Pc : constant V3 := Ap (Tr (G.R_Ce), [Pw (0) - G.Pos (0), Pw (1) - G.Pos (1), Pw (2) - G.Pos (2)]);
      Z : constant Long_Float := -Pc (2);
   begin
      In_Front := Z > 1.0e-6;
      if not In_Front then
         U := 0.0; V := 0.0;
         return;
      end if;
      U := G.Cx + G.F * Pc (0) / Z;
      V := G.Cy - G.F * Pc (1) / Z;
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
      Lm_Loosen : constant := 3;     --  LM 阻尼的升降倍数(次数,无量纲;只管求解器迭代,不管身体)
      Lm_Tighten : constant := 10;
      N : constant Natural := Natural (O.Length);
      type Params is array (0 .. 5) of Long_Float;   --  转向量 3 + 相机位置 3
      Gen : Ada.Numerics.Float_Random.Generator;
      Best_Cost : Long_Float := Long_Float'Last;
      Best_P : Params := [others => 0.0];
      Have_Best : Boolean := False;
      function Rnd return Long_Float is (Long_Float (Ada.Numerics.Float_Random.Random (Gen)));
      procedure Resid (P : Params; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
         Gt : Cam_Geo := G;
         Sum : Long_Float := 0.0;
      begin
         Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         Gt.Pos := [P (3), P (4), P (5)];
         for I in 0 .. N - 1 loop
            declare
               U, V, Du, Dv : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (Gt, O (I).Pw, U, V, Front);
               if Front then
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
         R := Sqrt (Sum / Long_Float (Natural'Max (1, N)));
      end Resid;
      function Cost (P : Params) return Long_Float is
         R : Long_Float;
      begin
         Resid (P, R, null);
         return R;
      end Cost;
   begin
      Ok := False;
      if N < 4 or else G.F <= 0.0 then
         return;
      end if;
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      for Trial in 1 .. 3000 loop
         declare
            Q : Plug.Arm_Pose := [others => 0.0];
            Nq : Long_Float := 0.0;
            R : M3;
            Ps : V3;
            P : Params;
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
            Ps := Pos_For (R, G, O);
            declare
               Rv : constant V3 := Rot_Vec (R);
               C : Long_Float;
            begin
               P := [Rv (0), Rv (1), Rv (2), Ps (0), Ps (1), Ps (2)];
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
         P : Params := Best_P;
         Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
         Cur : Long_Float := Best_Cost;
         type Jac is array (0 .. 2 * N - 1, 0 .. 5) of Long_Float;
         J : Jac;
         Rv : array (0 .. 2 * N - 1) of Long_Float;
         procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
         begin
            Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
         end Fill_R;
      begin
         for It in 1 .. 60 loop
            declare
               R0 : Long_Float;
            begin
               Resid (P, R0, Fill_R'Access);
               for K in 0 .. 5 loop
                  declare
                     H : constant Long_Float := 1.0e-4;   --  差分步(弧度 / 米,极小量)
                     Pp : Params := P;
                     Rp : array (0 .. 2 * N - 1) of Long_Float;
                     procedure Fill_P (I : Natural; Du, Dv : Long_Float) is
                     begin
                        Rp (2 * I) := Du; Rp (2 * I + 1) := Dv;
                     end Fill_P;
                     Dummy : Long_Float;
                  begin
                     Pp (K) := Pp (K) + H;
                     Resid (Pp, Dummy, Fill_P'Access);
                     for I in 0 .. 2 * N - 1 loop
                        J (I, K) := (Rp (I) - Rv (I)) / H;
                     end loop;
                  end;
               end loop;
               declare
                  A : array (0 .. 5, 0 .. 5) of Long_Float := [others => [others => 0.0]];
                  B : Params := [others => 0.0];
                  Dlt : Params := [others => 0.0];
               begin
                  for K in 0 .. 5 loop
                     for M in 0 .. 5 loop
                        for I in 0 .. 2 * N - 1 loop
                           A (K, M) := A (K, M) + J (I, K) * J (I, M);
                        end loop;
                     end loop;
                     for I in 0 .. 2 * N - 1 loop
                        B (K) := B (K) - J (I, K) * Rv (I);
                     end loop;
                  end loop;
                  for K in 0 .. 5 loop
                     A (K, K) := A (K, K) * (1.0 + Lam) + 1.0e-12;
                  end loop;
                  for Col in 0 .. 5 loop
                     declare
                        Piv : Natural := Col;
                     begin
                        for Rw in Col + 1 .. 5 loop
                           if abs (A (Rw, Col)) > abs (A (Piv, Col)) then
                              Piv := Rw;
                           end if;
                        end loop;
                        if Piv /= Col then
                           for M in 0 .. 5 loop
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
                           for Rw in 0 .. 5 loop
                              if Rw /= Col then
                                 declare
                                    Fct : constant Long_Float := A (Rw, Col) / A (Col, Col);
                                 begin
                                    for M in 0 .. 5 loop
                                       A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                                    end loop;
                                    B (Rw) := B (Rw) - Fct * B (Col);
                                 end;
                              end if;
                           end loop;
                        end if;
                     end;
                  end loop;
                  for K in 0 .. 5 loop
                     Dlt (K) := (if abs (A (K, K)) > 1.0e-18 then B (K) / A (K, K) else 0.0);
                  end loop;
                  declare
                     Pn : Params := P;
                     Cn : Long_Float;
                  begin
                     for K in 0 .. 5 loop
                        Pn (K) := Pn (K) + Dlt (K);
                     end loop;
                     Cn := Cost (Pn);
                     if Cn < Cur then
                        P := Pn; Cur := Cn; Lam := Lam / Long_Float (Lm_Loosen);
                     else
                        Lam := Lam * Long_Float (Lm_Tighten);
                     end if;
                  end;
               end;
            end;
            exit when Lam > 1.0e6;
         end loop;
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         G.Pos := [P (3), P (4), P (5)];
         G.Rms := Cur;
         G.Fixed := True;
         G.Valid := True;
         Ok := True;
      end;
   end Fit_Fixed;

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
                      ",""f"":" & Codec.Fmt (G.F, 4) & ",""cx"":" & Codec.Fmt (G.Cx, 3) & ",""cy"":" & Codec.Fmt (G.Cy, 3) & ",""rms"":" & Codec.Fmt (G.Rms, 3) & ",""r_ce"":[");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Append (B, (if I + J > 0 then "," else "") & Codec.Fmt (G.R_Ce (I, J), 7));
               end loop;
            end loop;
            Append (B, "],""tip_valid"":" & (if G.Tip_Valid then "true" else "false") & ",""tip"":[" & Codec.Fmt (G.Tip (0), 5) & "," & Codec.Fmt (G.Tip (1), 5) & "," &
                      Codec.Fmt (G.Tip (2), 5) & "],""gap"":" & Codec.Fmt (G.Gap, 5) &
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
                  if Tn >= 0 and then Json.Count (D, Tn) = 3 then
                     for A in 0 .. 2 loop
                        G.Tip (A) := Json.Num (D, Json.Child (D, Tn, A));
                     end loop;
                  else
                     G.Tip_Valid := False;
                  end if;
                  G.Gap := Json.Num (D, Json.Get (D, Nd, "gap"));
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
