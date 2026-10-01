with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Contact;
with Stats;
package body Held is
   Dim : constant := 3;        --  三维:一个点三个分量
   Pose_N : constant := 6;     --  一个位姿几个数:转 3 + 移 3

   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Add (A, B : M3) return M3 is
      C : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            C (I, J) := A (I, J) + B (I, J);
         end loop;
      end loop;
      return C;
   end Add;
   function Scl (A : M3; S : Long_Float) return M3 is
      C : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            C (I, J) := S * A (I, J);
         end loop;
      end loop;
      return C;
   end Scl;
   --  R A Rᵀ
   function Turned (R, A : M3) return M3 is (Geom.Mul (R, Geom.Mul (A, Geom.Tr (R))));

   --  位姿的小扰动(左乘的小转动 ω、平移 τ)对世界里一点 V = R s 那一处的雅可比:d = ω × V + τ = [−[V]×, I] (ω; τ);
   --  它带来的那一点的协方差 J C Jᵀ
   function Pose_Cov_At (V : V3; C : Cov6) return M3 is
      J : constant array (0 .. 2, 0 .. Pose_N - 1) of Long_Float :=
        [[0.0, V (2), -V (1), 1.0, 0.0, 0.0],
         [-V (2), 0.0, V (0), 0.0, 1.0, 0.0],
         [V (1), -V (0), 0.0, 0.0, 0.0, 1.0]];
      O : M3 := [others => [others => 0.0]];
   begin
      for R in 0 .. 2 loop
         for Cc in 0 .. 2 loop
            for A in 0 .. Pose_N - 1 loop
               for B in 0 .. Pose_N - 1 loop
                  O (R, Cc) := O (R, Cc) + J (R, A) * C (A, B) * J (Cc, B);
               end loop;
            end loop;
         end loop;
      end loop;
      return O;
   end Pose_Cov_At;

   --  一个位姿的不准换一个系看:diag (A, A) C diag (Aᵀ, Aᵀ)(转动、平移两块都按 A 转;A = Rᵀ ⇒ 世界的换到手的系,A = R ⇒ 反过来)
   function Mapped (A : M3; C : Cov6) return Cov6 is
      O : Cov6 := Zero6;
   begin
      for Bi in 0 .. 1 loop
         for Bj in 0 .. 1 loop
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  declare
                     Sm : Long_Float := 0.0;
                  begin
                     for Ka in 0 .. 2 loop
                        for Kb in 0 .. 2 loop
                           Sm := Sm + A (I, Ka) * C (Dim * Bi + Ka, Dim * Bj + Kb) * A (J, Kb);
                        end loop;
                     end loop;
                     O (Dim * Bi + I, Dim * Bj + J) := Sm;
                  end;
               end loop;
            end loop;
         end loop;
      end loop;
      return O;
   end Mapped;
   function Add (A, B : Cov6) return Cov6 is
      O : Cov6;
   begin
      for I in 0 .. Pose_N - 1 loop
         for J in 0 .. Pose_N - 1 loop
            O (I, J) := A (I, J) + B (I, J);
         end loop;
      end loop;
      return O;
   end Add;

   --  白化平方和 eᵀ C⁻¹ e(C 求不了逆 ⇒ e 不是 0 就当无穷远)
   function Mahal (E : V3; C : M3) return Long_Float renames Linkage.Mahal;

   procedure Take (Hands : Hand_Vectors.Vector; Tracks : Linkage.Track_Vectors.Vector; Arm : Natural;
                   Sigma : Long_Float; Sigma_Dof : Long_Float; P : out Part; Rep : out Take_Report) is
      S2 : constant Long_Float := Sigma ** 2;
   begin
      P := (Arm => Arm, Sigma => Sigma, Sigma_Dof => Sigma_Dof, others => <>);
      Rep := (others => <>);
      for I in 0 .. Natural (Tracks.Length) - 1 loop
         declare
            Tr : constant Linkage.Obs_Vectors.Vector := Tracks (I);
            K : constant Natural := Natural'Min (Natural (Tr.Length), Natural (Hands.Length));
            function Used (F : Natural) return Boolean is (Tr (F).Seen and then Hands (F).Ok);
            Nf : Natural := 0;
            Rl : Ride := Unused;
         begin
            for F in 0 .. K - 1 loop
               if Used (F) then
                  Nf := Nf + 1;
               end if;
            end loop;
            if Nf >= 2 then
               declare
                  --  "长在手上"那一说每一帧的协方差:眼的噪声 + 手此刻位姿的不准(按这一点离手的系原点多远)
                  function Ride_Cov (F : Natural; S : V3) return M3 is
                    (Add (Scl (Tr (F).Cov, S2), Pose_Cov_At (Geom.Ap (Hands (F).R, S), Hands (F).Cov)));
                  --  长在手上:s = (Σ Rᵀ Σ⁻¹ R)⁻¹ Σ Rᵀ Σ⁻¹ (x − T);Σ 跟 s 有关(杠杆)⇒ 从不算手的不准起,做到 s 不再变
                  S, S_New : V3 := [others => 0.0];
                  Info : M3;
                  Prev : Long_Float := Long_Float'Last;
                  procedure Solve_Ride (S_In : V3; With_Hand : Boolean; S_Out : out V3; Inf : out M3) is
                     B : V3 := [others => 0.0];
                  begin
                     Inf := [others => [others => 0.0]];
                     for F in 0 .. K - 1 loop
                        if Used (F) then
                           declare
                              Ok : Boolean;
                              Ci : constant M3 := Linkage.Inv3 ((if With_Hand then Ride_Cov (F, S_In) else Scl (Tr (F).Cov, S2)), Ok);
                              Rc : constant M3 := Geom.Mul (Geom.Tr (Hands (F).R), Ci);
                           begin
                              if Ok then
                                 Inf := Add (Inf, Geom.Mul (Rc, Hands (F).R));
                                 B := Add (B, Geom.Ap (Rc, Sub (Tr (F).X, Hands (F).T)));
                              end if;
                           end;
                        end if;
                     end loop;
                     S_Out := Geom.Solve3 (Inf, B);
                  end Solve_Ride;
                  W : V3 := [others => 0.0];
                  D_Ride, D_World, Sep : Long_Float := 0.0;
               begin
                  Solve_Ride (S_New, False, S, Info);
                  loop
                     Solve_Ride (S, True, S_New, Info);
                     declare
                        D : constant Long_Float := Geom.Norm (Sub (S_New, S));
                     begin
                        S := S_New;
                        exit when not (D > 0.0 and then D < Prev);
                        Prev := D;
                     end;
                  end loop;
                  --  留在世界里:w = 带权平均
                  declare
                     Inf_W : M3 := [others => [others => 0.0]];
                     B : V3 := [others => 0.0];
                  begin
                     for F in 0 .. K - 1 loop
                        if Used (F) then
                           declare
                              Ok : Boolean;
                              Ci : constant M3 := Linkage.Inv3 (Scl (Tr (F).Cov, S2), Ok);
                           begin
                              if Ok then
                                 Inf_W := Add (Inf_W, Ci);
                                 B := Add (B, Geom.Ap (Ci, Tr (F).X));
                              end if;
                           end;
                        end if;
                     end loop;
                     W := Geom.Solve3 (Inf_W, B);
                  end;
                  --  两种说法的残差;两种说法对它的预测隔多远:长在手上的预测 μ 当观测、按"留在世界里"解一个 w′,μ 离 w′ 的白化平方和
                  declare
                     Inf_M : M3 := [others => [others => 0.0]];
                     B : V3 := [others => 0.0];
                     Wm : V3;
                  begin
                     for F in 0 .. K - 1 loop
                        if Used (F) then
                           declare
                              Mu : constant V3 := Add (Geom.Ap (Hands (F).R, S), Hands (F).T);
                              Cr : constant M3 := Ride_Cov (F, S);
                              Ok : Boolean;
                              Ci : constant M3 := Linkage.Inv3 (Cr, Ok);
                           begin
                              D_Ride := D_Ride + Mahal (Sub (Tr (F).X, Mu), Cr);
                              D_World := D_World + Mahal (Sub (Tr (F).X, W), Scl (Tr (F).Cov, S2));
                              if Ok then
                                 Inf_M := Add (Inf_M, Ci);
                                 B := Add (B, Geom.Ap (Ci, Mu));
                              end if;
                           end;
                        end if;
                     end loop;
                     Wm := Geom.Solve3 (Inf_M, B);
                     for F in 0 .. K - 1 loop
                        if Used (F) then
                           Sep := Sep + Mahal (Sub (Add (Geom.Ap (Hands (F).R, S), Hands (F).T), Wm), Ride_Cov (F, S));
                        end if;
                     end loop;
                  end;
                  declare
                     G : constant Long_Float := Linkage.Gate_F (Dim * (Nf - 1), Sigma_Dof);
                  begin
                     if Sqrt (Sep) < 2.0 * Stats.Z then
                        Rl := Unknown;
                     elsif D_Ride <= D_World and then D_Ride <= G then
                        Rl := Rides;
                     elsif D_World < D_Ride and then D_World <= G then
                        Rl := World;
                     else
                        Rl := Neither;
                     end if;
                  end;
                  if Rl = Rides then
                     declare
                        Ok : Boolean;
                        S_Tmp : V3;
                        Info_Eye : M3;
                     begin
                        Solve_Ride (S, False, S_Tmp, Info_Eye);   --  只按眼的噪声:这一点自己的那一份(手的不准是所有点共有的,单记)
                        declare
                           Cs : constant M3 := Linkage.Inv3 (Info_Eye, Ok);
                        begin
                           if Ok then
                              P.Tracks.Append (I);
                              P.Pts.Append (S);
                              P.Covs.Append (Cs);
                           else
                              Rl := Unknown;
                           end if;
                        end;
                     end;
                  end if;
               end;
            end if;
            Rep.Roles.Append (Rl);
            case Rl is
               when Rides => Rep.N_Rides := Rep.N_Rides + 1;
               when World => Rep.N_World := Rep.N_World + 1;
               when Unknown => Rep.N_Unknown := Rep.N_Unknown + 1;
               when Neither => Rep.N_Neither := Rep.N_Neither + 1;
               when Unused => Rep.N_Unused := Rep.N_Unused + 1;
            end case;
         end;
      end loop;
      --  形状整体的不准:形状 ≈ 各帧换到手的系以后的平均 ⇒ 整体那一份 = 各帧手的不准(换到手的系)的平均:(1 / n²) Σ;
      --  n = 手在哪知道、又看见了跟着手走的点的那几帧
      declare
         Sum : Cov6 := Zero6;
         Nh : Natural := 0;
      begin
         for F in 0 .. Natural (Hands.Length) - 1 loop
            if Hands (F).Ok then
               declare
                  Any : Boolean := False;
               begin
                  for T of P.Tracks loop
                     Any := Any or else (F < Natural (Tracks (T).Length) and then Tracks (T) (F).Seen);
                  end loop;
                  if Any then
                     Sum := Add (Sum, Mapped (Geom.Tr (Hands (F).R), Hands (F).Cov));
                     Nh := Nh + 1;
                  end if;
               end;
            end if;
         end loop;
         if Nh > 0 then
            for I in 0 .. Pose_N - 1 loop
               for J in 0 .. Pose_N - 1 loop
                  P.Common (I, J) := Sum (I, J) / Long_Float (Nh) ** 2;
               end loop;
            end loop;
         end if;
      end;
   end Take;

   function Shape_Cov (P : Part; K : Natural) return M3 is (Add (P.Covs (K), Pose_Cov_At (P.Pts (K), P.Common)));

   function In_World (P : Part; H : Hand_Pose) return World_View is
      Wv : World_View;
   begin
      for K in 0 .. Natural (P.Pts.Length) - 1 loop
         declare
            V : constant V3 := Geom.Ap (H.R, P.Pts (K));
         begin
            Wv.Pts.Append (Add (V, H.T));
            Wv.Covs.Append (Add (Turned (H.R, P.Covs (K)), Pose_Cov_At (V, Add (H.Cov, Mapped (H.R, P.Common)))));
         end;
      end loop;
      return Wv;
   end In_World;

   procedure Check_Slip (P : Part; H : Hand_Pose; Now : Linkage.Obs_Vectors.Vector; Rep : out Slip_Report) is
      S2 : constant Long_Float := P.Sigma ** 2;
      N : constant Natural := Natural'Min (Natural (P.Pts.Length), Natural (Now.Length));
      Rt : constant M3 := Geom.Tr (H.R);
      --  这一眼每一点在手的系里在哪、协方差(眼的噪声转到手的系 + 形状的不准;手此刻的不准单独算,所有点共用)
      Ys : array (0 .. Natural'Max (1, N) - 1) of V3;
      Cs : array (0 .. Natural'Max (1, N) - 1) of M3;
      Use_K : array (0 .. Natural'Max (1, N) - 1) of Boolean := [others => False];
      Al, Be : V3 := [others => 0.0];   --  δ:转动向量、平移
      A : Linkage.Mat (0 .. Pose_N - 1, 0 .. Pose_N - 1);
      Bv : array (0 .. Pose_N - 1) of Long_Float;
      --  在 δ 处:信息阵 A = Σ Jᵀ C⁻¹ J、梯度 b = Σ Jᵀ C⁻¹ r、代价 Σ rᵀ C⁻¹ r(J = [−[q]×, I],q = 转过的形状点,r = y − q − β)
      procedure Normal (Cost : out Long_Float) is
         Rd : constant M3 := Geom.Rodrigues (Al);
      begin
         A := [others => [others => 0.0]];
         Bv := [others => 0.0];
         Cost := 0.0;
         for K in 0 .. N - 1 loop
            if Use_K (K) then
               declare
                  Q : constant V3 := Geom.Ap (Rd, P.Pts (K));
                  R : constant V3 := Sub (Ys (K), Add (Q, Be));
                  Ok : Boolean;
                  Ci : constant M3 := Linkage.Inv3 (Cs (K), Ok);
                  J : constant array (0 .. 2, 0 .. Pose_N - 1) of Long_Float :=
                    [[0.0, Q (2), -Q (1), 1.0, 0.0, 0.0],
                     [-Q (2), 0.0, Q (0), 0.0, 1.0, 0.0],
                     [Q (1), -Q (0), 0.0, 0.0, 0.0, 1.0]];
               begin
                  if Ok then
                     Cost := Cost + Contact.Dot (R, Geom.Ap (Ci, R));
                     for Ra in 0 .. Pose_N - 1 loop
                        for Rr in 0 .. 2 loop
                           for Cc in 0 .. 2 loop
                              Bv (Ra) := Bv (Ra) + J (Rr, Ra) * Ci (Rr, Cc) * R (Cc);
                              for Rb in 0 .. Pose_N - 1 loop
                                 A (Ra, Rb) := A (Ra, Rb) + J (Rr, Ra) * Ci (Rr, Cc) * J (Cc, Rb);
                              end loop;
                           end loop;
                        end loop;
                     end loop;
                  end if;
               end;
            end if;
         end loop;
      end Normal;
      --  对称阵 M(6 × 6)的特征分解;数值分辨率以上的那几个方向 = 定得住的
      Ev, V : Linkage.Mat (0 .. Pose_N - 1, 0 .. Pose_N - 1);
      Det : array (0 .. Pose_N - 1) of Boolean := [others => False];
      procedure Decompose is
         Mx : Long_Float := 0.0;
      begin
         Ev := A;
         Linkage.Eig_Sym (Ev, V);
         for D in 0 .. Pose_N - 1 loop
            Mx := Long_Float'Max (Mx, Ev (D, D));
         end loop;
         for D in 0 .. Pose_N - 1 loop
            Det (D) := Ev (D, D) > Long_Float (Pose_N) * Long_Float'Model_Epsilon * Mx and then Mx > 0.0;
         end loop;
      end Decompose;
   begin
      Rep := (others => <>);
      for K in 0 .. N - 1 loop
         if Now (K).Seen then
            Ys (K) := Geom.Ap (Rt, Sub (Now (K).X, H.T));
            Cs (K) := Add (Scl (Turned (Rt, Now (K).Cov), S2), P.Covs (K));
            Use_K (K) := True;
            Rep.Seen := Rep.Seen + 1;
         end if;
      end loop;
      if Rep.Seen = 0 or else not H.Ok then
         return;
      end if;
      --  高斯–牛顿:每一步只在定得住的方向上走(广义逆);做到代价不再降
      declare
         Cost, Prev : Long_Float;
      begin
         Normal (Prev);
         loop
            Decompose;
            declare
               Step : array (0 .. Pose_N - 1) of Long_Float := [others => 0.0];
               Al0 : constant V3 := Al;
               Be0 : constant V3 := Be;
            begin
               for D in 0 .. Pose_N - 1 loop
                  if Det (D) then
                     declare
                        Pr : Long_Float := 0.0;
                     begin
                        for Q in 0 .. Pose_N - 1 loop
                           Pr := Pr + V (Q, D) * Bv (Q);
                        end loop;
                        for Q in 0 .. Pose_N - 1 loop
                           Step (Q) := Step (Q) + V (Q, D) * Pr / Ev (D, D);
                        end loop;
                     end;
                  end if;
               end loop;
               Al := Geom.Rot_Vec (Geom.Mul (Geom.Rodrigues ([Step (0), Step (1), Step (2)]), Geom.Rodrigues (Al)));
               Be := Add (Be, [Step (3), Step (4), Step (5)]);
               Normal (Cost);
               if not (Cost < Prev) then
                  Al := Al0;
                  Be := Be0;
                  Normal (Cost);
                  exit;
               end if;
               Prev := Cost;
            end;
         end loop;
      end;
      Decompose;
      Rep.Turn := Al;
      Rep.Shift := Be;
      --  手此刻位姿的不准在手的系里凑出来的 δ:(Rᵀ ω, Rᵀ τ)⇒ 协方差 diag (Rᵀ, Rᵀ) C diag (R, R);再加上形状整体的那一份
      declare
         Ch : constant Cov6 := Add (Mapped (Rt, H.Cov), P.Common);
         Dv : constant array (0 .. Pose_N - 1) of Long_Float := [Al (0), Al (1), Al (2), Be (0), Be (1), Be (2)];
         R_Dof : Natural := 0;
      begin
         for D in 0 .. Pose_N - 1 loop
            if Det (D) then
               R_Dof := R_Dof + 1;
            end if;
         end loop;
         Rep.Dof := R_Dof;
         if R_Dof = 0 then
            return;
         end if;
         --  定得住的方向上:z = Vdᵀ δ,M = Λd⁻¹ + Vdᵀ Ch Vd,χ² = zᵀ M⁻¹ z
         declare
            Idx : array (0 .. R_Dof - 1) of Natural;
            Z : array (0 .. R_Dof - 1) of Long_Float := [others => 0.0];
            M, Mv : Linkage.Mat (0 .. R_Dof - 1, 0 .. R_Dof - 1);
            J : Natural := 0;
         begin
            for D in 0 .. Pose_N - 1 loop
               if Det (D) then
                  Idx (J) := D;
                  J := J + 1;
               end if;
            end loop;
            for A1 in 0 .. R_Dof - 1 loop
               for Q in 0 .. Pose_N - 1 loop
                  Z (A1) := Z (A1) + V (Q, Idx (A1)) * Dv (Q);
               end loop;
               for A2 in 0 .. R_Dof - 1 loop
                  declare
                     Sm : Long_Float := (if A1 = A2 then 1.0 / Ev (Idx (A1), Idx (A1)) else 0.0);
                  begin
                     for Q1 in 0 .. Pose_N - 1 loop
                        for Q2 in 0 .. Pose_N - 1 loop
                           Sm := Sm + V (Q1, Idx (A1)) * Ch (Q1, Q2) * V (Q2, Idx (A2));
                        end loop;
                     end loop;
                     M (A1, A2) := Sm;
                  end;
               end loop;
            end loop;
            Linkage.Eig_Sym (M, Mv);
            for D in 0 .. R_Dof - 1 loop
               declare
                  Pr : Long_Float := 0.0;
               begin
                  for Q in 0 .. R_Dof - 1 loop
                     Pr := Pr + Mv (Q, D) * Z (Q);
                  end loop;
                  if M (D, D) > 0.0 then
                     Rep.Chi := Rep.Chi + Pr ** 2 / M (D, D);
                  end if;
               end;
            end loop;
            Rep.Gate := Linkage.Gate_F (R_Dof, P.Sigma_Dof);
            Rep.Slipped := Rep.Chi > Rep.Gate;
         end;
      end;
   end Check_Slip;

   function Say (Rep : Take_Report) return String is
     ("拿着的东西:跟着手走 " & Codec.Img (Rep.N_Rides) & " · 留在世界里 " & Codec.Img (Rep.N_World) & " · 手动得不够、判不了 "
      & Codec.Img (Rep.N_Unknown) & " · 自己在动 / 跟错了 " & Codec.Img (Rep.N_Neither) & " · 没有可比的 " & Codec.Img (Rep.N_Unused));
   function Say (Rep : Slip_Report) return String is
     (if Rep.Dof = 0 then "滑没滑:这一眼看见的点什么都定不住,判不了(看见 " & Codec.Img (Rep.Seen) & " 个)"
      else "滑没滑:" & (if Rep.Slipped then "滑了" else "没滑") & "(看见 " & Codec.Img (Rep.Seen) & " 个,定得住 " & Codec.Img (Rep.Dof)
           & " 个方向,比手的不准多出 " & Codec.Fmt (Rep.Chi, 1) & ",门 " & Codec.Fmt (Rep.Gate, 1) & ")");
end Held;
