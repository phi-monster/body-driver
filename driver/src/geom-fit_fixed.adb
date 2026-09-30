separate (Geom)
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
