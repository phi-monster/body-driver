separate (Jointboot)
procedure Place_Hand (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; Sig0 : Long_Float; P : in out Hand_Place; Ok : out Boolean) is
   use Geom;
   Nr : constant Natural := Natural (Obs.Length);
   N_Grp : Natural := 0;
   --  此刻的解和它的噪声
   S : Long_Float := P.S;
   R : M3 := P.R;
   T : V3 := P.T;
   Sm : Floats;
   Kd : Long_Float := 1.0;
   Use_Set : Bools;
   Cost : Long_Float := 0.0;
   Steps : constant Kinem.Vec (0 .. 6) := [others => 1.0e-7];   --  差分步(弧度 / 模型单位 / 对数倍数,极小量,无量纲;同原来 Place_Arm)
   Lm_Capped : Boolean := False;

   procedure Res_Area (I : Natural; Sx : Long_Float; Rx : M3; Tx : V3; E1, E2, Area : out Long_Float) is
   begin
      Whiten (Hand_Part (Obs (I), Sx, Rx, Tx), Sm (Obs (I).Grp), Kd, E1, E2, Area);
   end Res_Area;
   procedure Res (I : Natural; Sx : Long_Float; Rx : M3; Tx : V3; E1, E2 : out Long_Float) is
      Area : Long_Float;
   begin
      Res_Area (I, Sx, Rx, Tx, E1, E2, Area);
   end Res;

   --  此刻的解上按 Over 那几条量方差分量(配点噪声每组一个、远近放大一个)
   procedure Measure (Over : Bools) is
      Ep, Vp, Ea, Va, Vd : Floats;
      Grp : Ints;
      Live : Bools;
   begin
      for I in 0 .. Nr - 1 loop
         declare
            Pt : constant Ob_Part := Hand_Part (Obs (I), S, R, T);
            A, B, C, D : Long_Float;
         begin
            Split (Pt, A, B, C, D);
            Ea.Append (A); Ep.Append (B); Va.Append (C); Vp.Append (D); Vd.Append (Pt.Vd); Grp.Append (Obs (I).Grp);
            Live.Append (Over (I) and then Pt.Valid);
         end;
      end loop;
      Noise_Of (Ep, Vp, Ea, Va, Vd, Grp, Live, N_Grp, Sig0, Sm, Kd);
   end Measure;

   --  按一组配点(Sel)+ 桌面 3 条,Huber 解(同原来 Place_Arm 的 Solve:残差全部按 Huber 加权)
   procedure Solve_On (Sel : Bools) is
      Nu : Natural := 0;
   begin
      for U of Sel loop
         if U then
            Nu := Nu + 1;
         end if;
      end loop;
      declare
         N_Res : constant Natural := 2 * Nu + 3;   --  每条配点两条残差 + 桌面 3 条(结构)
         X : Kinem.Vec (0 .. 6) := Pack7 (S, R, T);
         procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
            Sx : Long_Float;
            Rx : M3;
            Tx : V3;
            J : Natural := Rr'First;
         begin
            Unpack7 (Xx, Sx, Rx, Tx);
            for I in 0 .. Nr - 1 loop
               if Sel (I) then
                  Res (I, Sx, Rx, Tx, Rr (J), Rr (J + 1)); J := J + 2;
               end if;
            end loop;
            Tie_Res (Tie, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2));
         end Resid;
         Done : Boolean;
      begin
         Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access, Done);   --  100 = 保险次数(同原来 Place_Arm;没收住照实报)
         if not Done then
            Lm_Capped := True;
         end if;
         Unpack7 (X, S, R, T);
      end;
   end Solve_On;

   --  精修:每轮先在此刻的解上按上一轮门里的量方差分量,再挑门里的、解,做到门里的那一组不再变(Until_Settled)
   Prev : Bools;
   --  每条配点此刻的 |w|² 和乱配在白化单位里的密度(混合模型的门、代价都用)
   procedure W2_Dens (W2, Dens : out Floats) is
   begin
      W2.Clear; Dens.Clear;
      for I in 0 .. Nr - 1 loop
         declare
            E1, E2, Area : Long_Float;
         begin
            Res_Area (I, S, R, T, E1, E2, Area);
            W2.Append (E1 * E1 + E2 * E2); Dens.Append (Dens_Of (Area, Obs (I).W, Obs (I).H));
         end;
      end loop;
   end W2_Dens;
   procedure Pick (Use_R : out Bools; Enough : out Boolean) is
      W2, Dens : Floats;
      Gamma : Long_Float;
      Nu : Natural := 0;
   begin
      Measure (Prev);
      W2_Dens (W2, Dens);
      Mix_Gate (W2, Dens, Use_R, Gamma);
      for U of Use_R loop
         if U then
            Nu := Nu + 1;
         end if;
      end loop;
      Enough := Nu >= Min_Inl;
      Prev := Use_R;
   end Pick;
   procedure Refine is new Until_Settled (Pick, Solve_On);

   --  混合似然代价(每条配点:门里 = 二维单位正态、门外 = 均匀落在那只眼的画面上;门里的占比 γ 按 EM 解到不再变,同门的判法)+ 桌面 3 条(正态)。
   --  两坑比的时候各自按自己量的噪声算 —— 噪声也是要解的量,各自取它最可信的那一份(剖面似然)
   --  在像素里比(两坑各按自己量的噪声白化,白化单位不一样 ⇒ 每条配点的密度换回像素:÷ 它的像素面积 √det,负对数似然 + ln 面积)。
   --  (10-01 V1B68 拿掉头顶眼、世界换到第 2 只手:一个配点噪声量成 3.9 px、远近放大 58 倍的坏解,在白化单位里照样"对得上",出坑去了那儿)
   function Mix_Cost return Long_Float is
      W2, Dens : Floats;
      Gamma, C : Long_Float;
      R1, R2, R3 : Long_Float;
      Log_Area : Long_Float := 0.0;
   begin
      W2_Dens (W2, Dens);
      Mix_Em (W2, Dens, Gamma, C);
      for I in 0 .. Nr - 1 loop
         Log_Area := Log_Area + Log (Long_Float'Max (Dens (I) * Long_Float'Max (1.0, Long_Float (Obs (I).W) * Long_Float (Obs (I).H)), Long_Float'Model_Small));   --  面积 = 密度 × 画幅面积(同 Dens_Of 的 1 像素下限)
      end loop;
      Tie_Res (Tie, S, R, T, R1, R2, R3);
      return C + Log_Area + 0.5 * (R1 * R1 + R2 * R2 + R3 * R3);   --  正态的负对数似然(纯数学的 ½)
   end Mix_Cost;

   --  最不准的方向:按此刻门里的配点(Huber 权)+ 桌面 3 条的形式协方差 C,干活的地方的 3 × 3 协方差最大的那个方向 U;
   --  参数空间里沿它的方向 Dir = C Jpᵀ U / (Uᵀ P U)(让干活的地方沿 U 挪 1 单位、代价最小的那一个方向)。Okw = False:解不出协方差
   procedure Weak_Dir (Dir : out Kinem.Vec; Okw : out Boolean) is
      Nu : Natural := 0;
   begin
      Okw := False; Dir := [others => 0.0];
      for U of Use_Set loop
         if U then
            Nu := Nu + 1;
         end if;
      end loop;
      declare
         N_Res : constant Natural := 2 * Nu + 3;   --  结构(同 Solve_On)
         X0 : constant Kinem.Vec (0 .. 6) := Pack7 (S, R, T);
         R0, R1v : Kinem.Vec (0 .. N_Res - 1);
         J : Mat (0 .. N_Res - 1, 0 .. 6);
         H : Mat (0 .. 6, 0 .. 6) := [others => [others => 0.0]];
         Jp : Mat (0 .. 2, 0 .. 6);
         procedure Fill (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
            Sx : Long_Float;
            Rx : M3;
            Tx : V3;
            K : Natural := Rr'First;
         begin
            Unpack7 (Xx, Sx, Rx, Tx);
            for I in 0 .. Nr - 1 loop
               if Use_Set (I) then
                  Res (I, Sx, Rx, Tx, Rr (K), Rr (K + 1)); K := K + 2;
               end if;
            end loop;
            Tie_Res (Tie, Sx, Rx, Tx, Rr (K), Rr (K + 1), Rr (K + 2));
         end Fill;
         Wk0 : constant V3 := Work_Of (Tie, S, R, T);
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
            declare
               Wt : constant Long_Float := (if abs R0 (I) <= Kinem.Huber_K then 1.0 else Kinem.Huber_K / abs R0 (I));   --  Huber 的权(同 Robust_LM)
            begin
               for A in 0 .. 6 loop
                  for B in 0 .. 6 loop
                     H (A, B) := H (A, B) + Wt * J (I, A) * J (I, B);
                  end loop;
               end loop;
            end;
         end loop;
         Invert (H, Okv);
         if not Okv then
            return;
         end if;
         declare
            Pw : M3 := [others => [others => 0.0]];
            Ev : V3;
            Vv : M3;
            U : V3;
            Ct : array (0 .. 6) of Long_Float := [others => 0.0];   --  C Jpᵀ U
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
            if Ev (2) <= 0.0 then
               return;
            end if;
            U := [Vv (0, 2), Vv (1, 2), Vv (2, 2)];
            for I in 0 .. 6 loop
               for L in 0 .. 6 loop
                  Ct (I) := Ct (I) + H (I, L) * (Jp (0, L) * U (0) + Jp (1, L) * U (1) + Jp (2, L) * U (2));
               end loop;
               Dir (I) := Ct (I) / Ev (2);
            end loop;
            Okw := True;
         end;
      end;
   end Weak_Dir;

   Rounds : Natural;
   Verdict : Settle_Verdict;
begin
   Ok := False;
   P.Escapes := 0; P.Lm_Capped := False; P.Unsettled := False;
   if Nr < Min_Inl then
      return;
   end if;
   for O of Obs loop
      N_Grp := Natural'Max (N_Grp, O.Grp + 1);
   end loop;
   Sm := Filled (N_Grp, Sig0);
   Prev := Bool_Vectors.To_Vector (True, Ada.Containers.Count_Type (Nr));
   Refine (Rounds, Verdict);
   if Verdict = Too_Few then
      return;
   end if;
   P.Unsettled := Verdict = Cycled or else Verdict = Capped;
   Use_Set := Prev;
   Cost := Mix_Cost;
   --  出坑:到代价不再降为止(每换一次代价严格降;保险 = 条数 + 1 次)
   for Try in 0 .. Nr loop
      declare
         Dir : Kinem.Vec (0 .. 6);
         Okw : Boolean;
         Ts, Sts : Floats;
         Idx : Ints;
         X0 : constant Kinem.Vec (0 .. 6) := Pack7 (S, R, T);
         H_T : constant Long_Float := Steps (0);   --  沿方向的差分步(同 Steps,极小量)
      begin
         Weak_Dir (Dir, Okw);
         exit when not Okw;
         --  每条被门挡掉的配点:沿 Dir 挪 t 让它的白化残差最小(线性化),它的 t 的不确定度 = 1 / |∂w/∂t|
         declare
            Xh : Kinem.Vec (0 .. 6);
            Sh : Long_Float;
            Rh : M3;
            Th : V3;
         begin
            for C in 0 .. 6 loop
               Xh (C) := X0 (C) + H_T * Dir (C);
            end loop;
            Unpack7 (Xh, Sh, Rh, Th);
            for I in 0 .. Nr - 1 loop
               if not Use_Set (I) then
                  declare
                     A1, A2, B1, B2 : Long_Float;
                  begin
                     Res (I, S, R, T, A1, A2);
                     Res (I, Sh, Rh, Th, B1, B2);
                     declare
                        G1 : constant Long_Float := (B1 - A1) / H_T;
                        G2 : constant Long_Float := (B2 - A2) / H_T;
                        Gg : constant Long_Float := G1 * G1 + G2 * G2;
                     begin
                        if Gg > 0.0 and then abs A1 < Behind_W and then abs A2 < Behind_W then
                           Ts.Append (-(G1 * A1 + G2 * A2) / Gg); Sts.Append (1.0 / Sqrt (Gg)); Idx.Append (I);
                        end if;
                     end;
                  end;
               end if;
            end loop;
         end;
         exit when Ts.Is_Empty;
         declare
            Pk : constant Integer := Peak_Of (Ts, Sts);
            Tc : constant Long_Float := Ts (Natural (Pk));
            Cons : Bools := Use_Set;
            Keep_S : constant Long_Float := S;
            Keep_R : constant M3 := R;
            Keep_T : constant V3 := T;
            Keep_Use : constant Bools := Use_Set;
            Keep_Sm : constant Floats := Sm;
            Keep_K : constant Long_Float := Kd;
            Keep_Unsettled : constant Boolean := P.Unsettled;
            Xc : Kinem.Vec (0 .. 6);
            New_Cost : Long_Float := Long_Float'Last;
         begin
            --  这一坑的共识:原来门里的 + 提议落在峰上(Z 倍它自己的不确定度以内)的被挡点
            for J in 0 .. Natural (Ts.Length) - 1 loop
               if abs (Ts (J) - Tc) <= Stats.Z * Sts (J) then
                  Cons.Replace_Element (Natural (Idx (J)), True);
               end if;
            end loop;
            for C in 0 .. 6 loop
               Xc (C) := X0 (C) + Tc * Dir (C);
            end loop;
            Unpack7 (Xc, S, R, T);
            Solve_On (Cons);
            Prev := Cons;
            Refine (Rounds, Verdict);
            if Verdict /= Too_Few then
               Use_Set := Prev;
               New_Cost := Mix_Cost;
               P.Unsettled := Verdict = Cycled or else Verdict = Capped;
            end if;
            --  换坑 = 似然比检验过了:新坑的代价(负对数似然)要低过 Z² / 2(沿一个方向跳的那一个自由度,Z 倍;
            --  只是同一坑里重挑了几条、代价降一点点的,不算另一坑 —— 10-01 重放:不设这一道换了 2–4 次,后几次代价只降零点几)
            if New_Cost < Cost - 0.5 * Stats.Z ** 2 then
               Cost := New_Cost;
               P.Escapes := P.Escapes + 1;
            else
               S := Keep_S; R := Keep_R; T := Keep_T; Use_Set := Keep_Use; Sm := Keep_Sm; Kd := Keep_K; P.Unsettled := Keep_Unsettled;
               exit;
            end if;
         end;
      end;
   end loop;
   P.S := S; P.R := R; P.T := T; P.Sm := Sm; P.K := Kd; P.Cost := Cost; P.Lm_Capped := Lm_Capped;
   Hand_Sd (Obs, Tie, P);
   Ok := P.Inl >= Min_Inl;
end Place_Hand;
