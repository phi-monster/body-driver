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
      Cp : constant V3 := Cam_Pos (G, P);
      D : constant V3 := [Pw (0) - Cp (0), Pw (1) - Cp (1), Pw (2) - Cp (2)];
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
   --  ── Levenberg–Marquardt 精修(数值雅可比、正规方程高斯消元)──:参数个数由调用方定(6 = 朝向 + 点/位置;7 = 再加焦距)。
   --  Resid 把每个观测的两个像素差填进 Fill;Steps 是各参数的差分步(弧度 / 米 / 像素,极小量)。
   --  阻尼升降的两个倍数不是门槛、不影响身体动不动,只管这次拟合怎么迭代
   type Param_Vec is array (Natural range <>) of Long_Float;
   procedure LM_Refine (P : in out Param_Vec; N_Obs : Natural; Steps : Param_Vec; Iters : Positive;
                        Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                        Cur : in out Long_Float) is
      Lm_Loosen : constant := 3;
      Lm_Tighten : constant := 10;
      Np : constant Natural := P'Length;
      Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
      Rv : array (0 .. 2 * N_Obs - 1) of Long_Float := [others => 0.0];
      J : array (0 .. 2 * N_Obs - 1, 0 .. Np - 1) of Long_Float := [others => [others => 0.0]];
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
               Rp : array (0 .. 2 * N_Obs - 1) of Long_Float := [others => 0.0];
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
   end LM_Refine;

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
            return;   --  解出来还有点跑到相机后面 ⇒ 不是解,不存
         end if;
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

   procedure Fit_Fixed_Rig (G : in out Cam_Geo; O : Obs_Pt_Vectors.Vector; N_Arms : Natural; Tip_H : out V3_Vectors.Vector; Ok : out Boolean) is
      Fit_F : constant Boolean := G.F <= 0.0;
      Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;
      N : constant Natural := Natural (O.Length);
      Cnt : array (0 .. N_Arms) of Natural := [others => 0];
      Keep : array (0 .. N_Arms) of Boolean := [others => False];   --  看见 4 停以上的点才进(次数)
      Slot : array (0 .. N_Arms) of Integer := [others => -1];
      Nk : Natural := 0;
      N_Used : Natural := 0;
      Gi : Cam_Geo := G;
      Best_Arm : Integer := -1;
   begin
      Ok := False; Tip_H.Clear;
      if N_Arms = 0 or else N < 4 then
         return;
      end if;
      for Ob of O loop
         if Ob.Pt < N_Arms then
            Cnt (Ob.Pt) := Cnt (Ob.Pt) + 1;
         end if;
      end loop;
      for K in 0 .. N_Arms - 1 loop
         if Cnt (K) >= 4 then
            Keep (K) := True; Slot (K) := Integer (Nk); Nk := Nk + 1; N_Used := N_Used + Cnt (K);
            if Best_Arm < 0 or else Cnt (K) > Cnt (Best_Arm) then
               Best_Arm := K;
            end if;
         end if;
      end loop;
      --  方程数(每笔观测两条)不到未知数的两倍就是在猜(V1I 2026-09-24:10 笔观测解 13 个未知数,解出相机在 2.8 m 外、残差 0.27 px)
      if Best_Arm < 0 or else 2 * N_Used < 2 * ((if Fit_F then 7 else 6) + 3 * Nk) then
         return;
      end if;
      --  起点:观测最多的那条臂,先把指尖当成就在手的位姿点上(偏移 0),用老的单点法(盲搜 + 精修)给相机位姿和焦距一个像样的起点
      declare
         Marks : Mark_Vectors.Vector;
         Fok : Boolean;
      begin
         for Ob of O loop
            if Ob.Pt = Best_Arm then
               Marks.Append (Mark'(Pw => [Ob.Pose (0), Ob.Pose (1), Ob.Pose (2)], U => Ob.U, V => Ob.V));
            end if;
         end loop;
         Fit_Fixed (Gi, Marks, Fok);
         if not Fok then
            return;
         end if;
      end;
      declare
         Base : constant Natural := (if Fit_F then 7 else 6);   --  转向量 3 + 位置 3 (+ 焦距)
         Np : constant Natural := Base + 3 * Nk;
         P : Param_Vec (0 .. Np - 1) := [others => 0.0];
         Steps : Param_Vec (0 .. Np - 1) := [others => 1.0e-4];   --  差分步(弧度 / 米,极小量)
         Rv : constant V3 := Rot_Vec (Gi.R_Ce);
         Nr : Natural := 0;
         Cur : Long_Float := 0.0;
         Behind : Natural := 0;   --  最近一次算残差时跑到相机后面的观测数(解出来还有 ⇒ 不算解出来)
         Skip : array (0 .. N - 1) of Boolean := [others => False];   --  被判离群、不再进解的观测(按观测序号)
         procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
            Gt : Cam_Geo := G;
            Sum : Long_Float := 0.0;
            I : Natural := 0;
         begin
            Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
            Gt.Pos := [P (3), P (4), P (5)];
            if Fit_F then
               Gt.F := P (6);
            end if;
            Behind := 0;
            for J in 0 .. N - 1 loop
               declare
                  Ob : constant Obs_Pt := O (J);
               begin
               if Ob.Pt < N_Arms and then Keep (Ob.Pt) and then not Skip (J) then
                  declare
                     B : constant Natural := Base + 3 * Natural (Slot (Ob.Pt));
                     Tw : constant V3 := Ap (Quat_To_R (Ob.Pose), [P (B), P (B + 1), P (B + 2)]);   --  指尖偏移转到世界
                     Pw : constant V3 := [Ob.Pose (0) + Tw (0), Ob.Pose (1) + Tw (1), Ob.Pose (2) + Tw (2)];
                     U, V, Du, Dv : Long_Float;
                     Front : Boolean;
                  begin
                     Project_Fixed (Gt, Pw, U, V, Front);
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
         P (3) := Gi.Pos (0); P (4) := Gi.Pos (1); P (5) := Gi.Pos (2);
         if Fit_F then
            P (6) := Gi.F; Steps (6) := 1.0;   --  焦距的差分步(像素,极小量)
         end if;
         for Ob of O loop
            if Ob.Pt < N_Arms and then Keep (Ob.Pt) then
               Nr := Nr + 1;
            end if;
         end loop;
         if Use_Prior then
            Nr := Nr + 1;
         end if;
         Resid (P, Cur, null);
         LM_Refine (P, Nr, Steps, 60, Resid'Access, Cur);
         --  离群观测(指尖跟错了)踢掉再解:每笔残差比中位数大 3 倍(比例,无量纲)的不要;踢掉的不到四分之一才算离群
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
                     if O (J).Pt < N_Arms and then Keep (O (J).Pt) and then not Skip (J) then
                        if Rs (I) > 3.0 * Med then
                           Skip (J) := True;
                           Dropped := Dropped + 1;
                        end if;
                        I := I + 1;
                     end if;
                  end loop;
               end;
               if Dropped > 0 and then Dropped * 4 < Nr then
                  Nr := Nr - Dropped;
                  Resid (P, Cur, null);
                  LM_Refine (P, Nr, Steps, 60, Resid'Access, Cur);
               else
                  for K in Skip'Range loop
                     Skip (K) := False;
                  end loop;
                  Dropped := 0;
               end if;
               G.Dropped := Dropped;
            end if;
         end;
         Resid (P, Cur, null);
         if Behind > 0 then
            return;   --  解出来还有指尖跑到相机后面 ⇒ 不是解,不存(V1H 2026-09-24:11 笔观测解到残差 953 px)
         end if;
         G.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         G.Pos := [P (3), P (4), P (5)];
         if Fit_F then
            G.F := P (6);
         end if;
         G.F_Meas := (if Fit_F then P (6) else 0.0);
         G.Rms := Cur;
         G.Fixed := True;
         G.Valid := True;
         for K in 0 .. N_Arms - 1 loop
            if Keep (K) then
               declare
                  B : constant Natural := Base + 3 * Natural (Slot (K));
               begin
                  Tip_H.Append (V3'[P (B), P (B + 1), P (B + 2)]);
               end;
            else
               Tip_H.Append (V3'[0.0, 0.0, 0.0]);   --  没解的点:0 向量(调用方按范数 > 0 认)
            end if;
         end loop;
         Ok := True;
      end;
   end Fit_Fixed_Rig;

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
                      ",""f"":" & Codec.Fmt (G.F, 4) & ",""cx"":" & Codec.Fmt (G.Cx, 3) & ",""cy"":" & Codec.Fmt (G.Cy, 3) & ",""rms"":" & Codec.Fmt (G.Rms, 3) & ",""r_ce"":[");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Append (B, (if I + J > 0 then "," else "") & Codec.Fmt (G.R_Ce (I, J), 7));
               end loop;
            end loop;
            Append (B, "],""tip_valid"":" & (if G.Tip_Valid then "true" else "false") & ",""tip"":[" & Codec.Fmt (G.Tip (0), 5) & "," & Codec.Fmt (G.Tip (1), 5) & "," &
                      Codec.Fmt (G.Tip (2), 5) & "],""gap"":" & Codec.Fmt (G.Gap, 5) & ",""stride"":" & Codec.Fmt (G.Stride, 5) &
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
                     Sn : constant Integer := Json.Get (D, Nd, "stride");   --  老文件没有这一项 ⇒ 0,开机再量
                     Fm : constant Integer := Json.Get (D, Nd, "f_meas");
                     Fp : constant Integer := Json.Get (D, Nd, "f_prior");
                     Fs : constant Integer := Json.Get (D, Nd, "f_prior_sd");
                  begin
                     if Sn >= 0 then
                        G.Stride := Json.Num (D, Sn);
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
