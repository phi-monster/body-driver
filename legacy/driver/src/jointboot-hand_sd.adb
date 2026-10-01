separate (Jointboot)
procedure Hand_Sd (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; P : in out Hand_Place) is
   use Geom;
   Nr : constant Natural := Natural (Obs.Length);
   Steps : constant Kinem.Vec (0 .. 6) := [others => 1.0e-7];   --  差分步(同 Place_Hand,极小量)
   function Sm_Of (G : Natural) return Long_Float is (if G < Natural (P.Sm.Length) then P.Sm (G) else 0.0);
   Use_Set : Bools;
   Nu : Natural := 0;
begin
   P.Inl := 0; P.Md := 0.0; P.Weak := [0.0, 0.0, 0.0]; P.Sd_Formal := 0.0; P.Sd_Sys := 0.0; P.Sd_Real := 0.0;
   --  门里的:在这个放法、这份噪声上(同 Place_Hand 的 Pick:混合模型的分界)
   declare
      W2, Dens, Ein : Floats;
      Gamma : Long_Float;
   begin
      for I in 0 .. Nr - 1 loop
         declare
            E1, E2, Area : Long_Float;
         begin
            Whiten (Hand_Part (Obs (I), P.S, P.R, P.T), Sm_Of (Obs (I).Grp), P.K, E1, E2, Area);
            W2.Append (E1 * E1 + E2 * E2); Dens.Append (Dens_Of (Area, Obs (I).W, Obs (I).H));
         end;
      end loop;
      Mix_Gate (W2, Dens, Use_Set, Gamma);
      for I in 0 .. Nr - 1 loop
         if Use_Set (I) then
            Nu := Nu + 1; Ein.Append (Sqrt (W2 (I)));
         end if;
      end loop;
      P.Inl := Nu;
      P.Md := Median_Of (Ein);
   end;
   if Nu = 0 then
      return;
   end if;
   declare
      N_Res : constant Natural := 2 * Nu + 3;   --  每条配点两条残差 + 桌面 3 条(结构)
      X0 : constant Kinem.Vec (0 .. 6) := Pack7 (P.S, P.R, P.T);
      R0, R1v : Kinem.Vec (0 .. N_Res - 1);
      J : Mat (0 .. N_Res - 1, 0 .. 6);
      H : Mat (0 .. 6, 0 .. 6) := [others => [others => 0.0]];
      Jp : Mat (0 .. 2, 0 .. 6);
      Wt : array (0 .. N_Res - 1) of Long_Float;
      --  门里每条配点:系统那一份(量到的远近不准超出自报的那一截,K² − 1 倍的远近方差)沿对极线的像素向量,白化后的两条
      --  系统那一份 = 这条配点整条远近误差:量到的 K 倍自报的、而且不小于它自报的那一份,按全相关(GUM:成片的误差线性相加)。
      --  不按"超出自报的那一截":成片的误差被拟合吃掉的那一截在残差里看不见(10-01 合成的远墙:远近顺着最不准的方向偏 1–2%、每点再随机 1%,
      --  只用远点放 ⇒ 按"超出的那一截"量 K = 0.51 < 1、系统那一份算成 0、报准,其实偏 0.25 单位)⇒ 看不见的只能按它自己说的有多准兜底
      Sw1, Sw2 : array (0 .. Natural'Max (1, Nu) - 1) of Long_Float := [others => 0.0];
      procedure Fill (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
         Sx : Long_Float;
         Rx : M3;
         Tx : V3;
         K : Natural := Rr'First;
      begin
         Unpack7 (Xx, Sx, Rx, Tx);
         for I in 0 .. Nr - 1 loop
            if Use_Set (I) then
               declare
                  Area : Long_Float;
               begin
                  Whiten (Hand_Part (Obs (I), Sx, Rx, Tx), Sm_Of (Obs (I).Grp), P.K, Rr (K), Rr (K + 1), Area);
               end;
               K := K + 2;
            end if;
         end loop;
         Tie_Res (Tie, Sx, Rx, Tx, Rr (K), Rr (K + 1), Rr (K + 2));
      end Fill;
      Wk0 : constant V3 := Work_Of (Tie, P.S, P.R, P.T);
      Okv : Boolean;
   begin
      Fill (X0, R0);
      for C in 0 .. 6 loop
         declare
            Xp : Kinem.Vec := X0;
            Sx : Long_Float;
            Rx : M3;
            Tx : V3;
         begin
            Xp (C) := Xp (C) + Steps (C);
            Fill (Xp, R1v);
            for I in 0 .. N_Res - 1 loop
               J (I, C) := (R1v (I) - R0 (I)) / Steps (C);
            end loop;
            Unpack7 (Xp, Sx, Rx, Tx);
            declare
               Wk : constant V3 := Work_Of (Tie, Sx, Rx, Tx);
            begin
               for D in 0 .. 2 loop
                  Jp (D, C) := (Wk (D) - Wk0 (D)) / Steps (C);
               end loop;
            end;
         end;
      end loop;
      for I in 0 .. N_Res - 1 loop
         Wt (I) := (if abs R0 (I) <= Kinem.Huber_K then 1.0 else Kinem.Huber_K / abs R0 (I));   --  Huber 的权(同 Robust_LM)
         for A in 0 .. 6 loop
            for B in 0 .. 6 loop
               H (A, B) := H (A, B) + Wt (I) * J (I, A) * J (I, B);
            end loop;
         end loop;
      end loop;
      Invert (H, Okv);
      if not Okv then
         return;
      end if;
      declare
         K : Natural := 0;
         Ex : constant Long_Float := Long_Float'Max (P.K, 1.0);   --  整条远近误差,不小于自报的那一份(1 倍自报)
      begin
         for I in 0 .. Nr - 1 loop
            if Use_Set (I) then
               declare
                  Pt : constant Ob_Part := Hand_Part (Obs (I), P.S, P.R, P.T);
                  Sd : constant Long_Float := Ex * Sqrt (Pt.Vd);
                  Area : Long_Float;
               begin
                  if Pt.Valid then
                     Whiten_Vec (Pt, Sm_Of (Obs (I).Grp), P.K, Sd * Pt.A0, Sd * Pt.A1, Sw1 (K), Sw2 (K), Area);
                  end if;
               end;
               K := K + 1;
            end if;
         end loop;
      end;
      declare
         Pw : M3 := [others => [others => 0.0]];
         Ev : V3;
         Vv : M3;
         Best : Long_Float := -1.0;
      begin
         for A in 0 .. 2 loop
            for B in 0 .. 2 loop
               for I in 0 .. 6 loop
                  for L in 0 .. 6 loop
                     Pw (A, B) := Pw (A, B) + Jp (A, I) * H (I, L) * Jp (B, L);
                  end loop;
               end loop;
            end loop;
         end loop;
         Eig3 (Pw, Ev, Vv);
         --  三个主方向各算一遍:形式的 = √特征值;系统的 = Σ 每条门里配点 |它对这个方向的增益 · 它系统那一份|(全相关:符号取最坏的,按 GUM 线性加)
         for D in 0 .. 2 loop
            declare
               U : constant V3 := [Vv (0, D), Vv (1, D), Vv (2, D)];
               Gp : array (0 .. 6) of Long_Float := [others => 0.0];   --  Uᵀ Jp C(1 × 7)
               Sf : constant Long_Float := Sqrt (Long_Float'Max (Ev (D), 0.0));
               Sys : Long_Float := 0.0;
            begin
               for L in 0 .. 6 loop
                  for I in 0 .. 6 loop
                     Gp (L) := Gp (L) + (U (0) * Jp (0, I) + U (1) * Jp (1, I) + U (2) * Jp (2, I)) * H (I, L);
                  end loop;
               end loop;
               for K in 0 .. Nu - 1 loop
                  declare
                     G1, G2 : Long_Float := 0.0;   --  这个方向对这条配点两条白化残差的增益:Uᵀ Jp C Jᵢᵀ wᵢ
                  begin
                     for L in 0 .. 6 loop
                        G1 := G1 + Gp (L) * J (2 * K, L) * Wt (2 * K);
                        G2 := G2 + Gp (L) * J (2 * K + 1, L) * Wt (2 * K + 1);
                     end loop;
                     Sys := Sys + abs (G1 * Sw1 (K) + G2 * Sw2 (K));
                  end;
               end loop;
               declare
                  Real : constant Long_Float := Sqrt (Sf * Sf + Sys * Sys);
               begin
                  if Real > Best then
                     Best := Real;
                     P.Weak := U; P.Sd_Formal := Sf; P.Sd_Sys := Sys; P.Sd_Real := Real;
                  end if;
               end;
            end;
         end loop;
      end;
   end;
end Hand_Sd;
