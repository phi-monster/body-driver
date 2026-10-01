with Stats;
separate (Selfcheck)
procedure Welds_Path_3 is
   --  路 3 的焊点(大并行.md §5 路 3):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   use Geom;
   use Ada.Numerics.Long_Elementary_Functions;
   Gen : Ada.Numerics.Float_Random.Generator;
   function U01 return Long_Float is (Long_Float (Ada.Numerics.Float_Random.Random (Gen)));
   --  标准正态(Box-Muller)
   function Gauss return Long_Float is
      U1 : constant Long_Float := Long_Float'Max (U01, 1.0e-12);
      U2 : constant Long_Float := U01;
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gauss;
   function Rot (Ax : V3; Th : Long_Float) return M3 is
      N : constant Long_Float := Norm (Ax);
   begin
      return Rodrigues ([Ax (0) / N * Th, Ax (1) / N * Th, Ax (2) / N * Th]);
   end Rot;

   --  ── 合成的两只手(第 7 条后半那几条焊点共用):世界 = 第 0 只手参照眼系(-z 朝前、+y 朝上),第 2 只手参照眼真在 (11, 0, 0)、同朝向、倍数 1;
   --  桌面 y = -8。第 0 只手 5 格眼(焦距 400、640 × 480,朝右下)看第 2 只手三角出的点 ──
   F : constant := 400.0;
   Cx : constant := 320.0;
   Cy : constant := 240.0;
   Wd : constant := 640;
   Ht : constant := 480;
   N_Cam : constant := 5;
   type Cam_Arr is array (0 .. N_Cam - 1) of Cam_Geo;
   Cams : Cam_Arr;
   Offs : constant array (0 .. N_Cam - 1, 0 .. 1) of Long_Float := [[0.0, 0.0], [0.8, 0.3], [-0.8, 0.2], [0.4, -0.6], [-0.3, -0.5]];
   T_True : constant V3 := [11.0, 0.0, 0.0];
   procedure Proj (G : Cam_Geo; X : V3; U, V : out Long_Float; Ok : out Boolean) is
      Pc : constant V3 := Ap (Tr (G.R_Ce), [X (0) - G.Pos (0), X (1) - G.Pos (1), X (2) - G.Pos (2)]);
   begin
      Ok := Pc (2) < 0.0;
      U := 0.0; V := 0.0;
      if Ok then
         U := F * Pc (0) / (-Pc (2)) + Cx; V := -F * Pc (1) / (-Pc (2)) + Cy;
         Ok := U >= 0.0 and then U < Long_Float (Wd) and then V >= 0.0 and then V < Long_Float (Ht);
      end if;
   end Proj;
   --  这只手自报的协方差:横向 0.1 px 的视线误差,远近 Rel_Sd × 距离
   function Cov_Of (Xb : V3; Rel_Sd : Long_Float) return M3 is
      D : constant Long_Float := Norm (Xb);
      Dd : constant V3 := [Xb (0) / D, Xb (1) / D, Xb (2) / D];
      Sd : constant Long_Float := Rel_Sd * D;
      Sl : constant Long_Float := D * 0.1 / F;
      C : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            C (I, J) := Sl * Sl * ((if I = J then 1.0 else 0.0) - Dd (I) * Dd (J)) + Sd * Sd * Dd (I) * Dd (J);
         end loop;
      end loop;
      return C;
   end Cov_Of;
   --  一个点 Pw(世界里真在哪)、它在第 2 只手系里三角出的 Xb,配进 5 格眼(配点噪声 0.3 px)
   procedure Observe (Pw, Xb : V3; Cov : M3; Obs : in out Jointboot.Hand_Ob_Vectors.Vector) is
   begin
      for C in 0 .. N_Cam - 1 loop
         declare
            U, V : Long_Float;
            Ok : Boolean;
         begin
            Proj (Cams (C), Pw, U, V, Ok);
            if Ok then
               Obs.Append (Jointboot.Hand_Ob'(X => Xb, Cov => Cov, Cam => Cams (C), U => U + 0.3 * Gauss, V => V + 0.3 * Gauss,
                                              W => Wd, H => Ht, Grp => 0, Rd => [0.0, 0.0, -1.0]));
            end if;
         end;
      end loop;
   end Observe;
   --  几条乱配:随便一个点配到随便一格眼里的随便一处
   procedure Junk (Obs : in out Jointboot.Hand_Ob_Vectors.Vector; N : Natural) is
      Base : constant Natural := Natural (Obs.Length);
   begin
      for J in 1 .. N loop
         declare
            K : constant Natural := Natural'Min (Base - 1, Natural (U01 * Long_Float (Base)));
            C : constant Natural := Natural'Min (N_Cam - 1, Natural (U01 * Long_Float (N_Cam)));
            O : Jointboot.Hand_Ob := Obs (K);
         begin
            O.Cam := Cams (C); O.U := U01 * Long_Float (Wd); O.V := U01 * Long_Float (Ht);
            Obs.Append (O);
         end;
      end loop;
   end Junk;
   --  两手之间桌面上 15 个近点(第 2 只手三角的远近随机差 0.2%、自报 0.07%)
   procedure Near_Pts (Obs : in out Jointboot.Hand_Ob_Vectors.Vector; Cb : out V3) is
      S : V3 := [0.0, 0.0, 0.0];
   begin
      for I in 0 .. 14 loop
         declare
            Pw : constant V3 := [3.0 + 5.0 * U01, -8.0, -14.0 + 6.0 * U01];
            L : constant Long_Float := 1.0 + 0.002 * Gauss;
            Xb : constant V3 := [L * (Pw (0) - T_True (0)), L * (Pw (1) - T_True (1)), L * (Pw (2) - T_True (2))];
         begin
            Observe (Pw, Xb, Cov_Of (Xb, 0.0007), Obs);
            S := [S (0) + Xb (0), S (1) + Xb (1), S (2) + Xb (2)];
         end;
      end loop;
      Cb := [S (0) / 15.0, S (1) / 15.0, S (2) / 15.0];
   end Near_Pts;
   function Tie_At (Cb : V3) return Jointboot.Plane_Tie is
     (Cb => Cb, Nb => [0.0, 1.0, 0.0], Sn => 0.001, Sd => 0.005, P0 => [0.0, -8.0, 0.0], N0 => [0.0, 1.0, 0.0]);
   function Work (P : Jointboot.Hand_Place; Cb : V3) return V3 is
      Rc : constant V3 := Ap (P.R, Cb);
   begin
      return [P.S * Rc (0) + P.T (0), P.S * Rc (1) + P.T (1), P.S * Rc (2) + P.T (2)];
   end Work;
   function Err (P : Jointboot.Hand_Place; Cb : V3) return Long_Float is
      Wk : constant V3 := Work (P, Cb);
   begin
      return Norm ([Wk (0) - Cb (0) - T_True (0), Wk (1) - Cb (1) - T_True (1), Wk (2) - Cb (2) - T_True (2)]);
   end Err;
begin
   Ada.Numerics.Float_Random.Reset (Gen, 20261001);
   for C in 0 .. N_Cam - 1 loop
      Cams (C) := No_Geo;
      Cams (C).R_Ce := Mul (Rot ([0.0, 1.0, 0.0], -12.0 * Ada.Numerics.Pi / 180.0), Rot ([1.0, 0.0, 0.0], -14.0 * Ada.Numerics.Pi / 180.0));   --  朝右、朝下
      Cams (C).Pos := [Offs (C, 0), Offs (C, 1), 0.0];
      Cams (C).F := F; Cams (C).Cx := Cx; Cams (C).Cy := Cy; Cams (C).Valid := True; Cams (C).Fixed := True;
   end loop;

   --  🔴 ① 方差分量(Jointboot.Noise_Of,10-01 V1B68 / V1B69):第 2 只手三角点的远近,量到的不准是它自报的 2.6–3.8 倍(按真值 2.6–3.5 倍),
   --  配点噪声按往返差的中位只有 0.24 px、按垂直对极线那一份量是 0.19–0.42 px。错了会是什么病:远近按自报的信,远处那面墙几百个点的远近
   --  一起偏的那一片压过近处的几十个点,第 2 只手沿两手连线偏 38 mm。合成:两组配点(配点噪声 0.4 / 0.2 px),每条横向那份各不相同,
   --  远近那份自报 0.1–1 px、真的是它的 3 倍 ⇒ 两组的配点噪声、远近放大都要量回来(容差 = Z × 中位数量尺度的标准误差 1.166 / √条数)。
   --  牙:远近放大按自报的(K 取 1,原来的做法)⇒ K 差 67%,红
   declare
      N : constant := 3000;
      Sm_T : constant array (0 .. 1) of Long_Float := [0.4, 0.2];
      K_T : constant := 3.0;
      Ep, Vp, Ea, Va, Vd : Floats;
      Grp : Ints;
      Live : Bools;
      Sm : Floats;
      K : Long_Float;
      Tol : constant Long_Float := Stats.Z * 1.166 / Sqrt (Long_Float (N / 2));
   begin
      for I in 0 .. N - 1 loop
         declare
            G : constant Natural := I mod 2;
            Lp : constant Long_Float := (0.05 * U01) ** 2;
            La : constant Long_Float := (0.05 * U01) ** 2;
            D : constant Long_Float := (0.1 + 0.9 * U01) ** 2;
         begin
            Ep.Append (Gauss * Sqrt (Sm_T (G) ** 2 + Lp)); Vp.Append (Lp);
            Ea.Append (Gauss * Sqrt (Sm_T (G) ** 2 + La + K_T ** 2 * D)); Va.Append (La); Vd.Append (D);
            Grp.Append (G); Live.Append (True);
         end;
      end loop;
      Jointboot.Noise_Of (Ep, Vp, Ea, Va, Vd, Grp, Live, 2, 1.0, Sm, K);
      Check (abs (Sm (0) / Sm_T (0) - 1.0) < Tol and then abs (Sm (1) / Sm_T (1) - 1.0) < Tol and then abs (K / K_T - 1.0) < Tol,
             "方差分量:两组配点噪声 0.4 / 0.2 px、远近真的是自报的 3 倍 ⇒ 量回 " & Codec.Fmt (Sm (0), 3) & " / " & Codec.Fmt (Sm (1), 3) & " px、"
             & Codec.Fmt (K, 3) & " 倍(容差 " & Codec.Fmt (100.0 * Tol, 1) & "%)");
   end;

   --  🔴 ② 核密度峰(Jointboot.Peak_Of):被门挡掉的配点各报一个"沿最不准的方向挪多少我就对上了",一致的那一拨是另一坑的证据,乱配的散在各处。
   --  错了会是什么病:按提议的中位 / 平均挑,乱配一多就挑到乱配那一边,哪一坑都不是,出坑那一步落空。合成:25 个一致地报 0.6(各自不确定度 0.02)、
   --  40 个乱配均匀散在 -3…0(乱配不会正好对称地落在那一拨两边;第一回焊的时候撒在 ±3,中位数恰好落进 0.6 那一拨,牙咬不住)⇒ 峰在 0.6 ± Z × 0.02。
   --  牙:Peak_Of 改成取提议的中位 ⇒ 落在乱配那一边,红
   declare
      T, St : Floats;
   begin
      for I in 0 .. 24 loop
         T.Append (0.6 + 0.02 * Gauss); St.Append (0.02);
      end loop;
      for I in 0 .. 39 loop
         T.Append (-3.0 * U01); St.Append (0.02);
      end loop;
      declare
         Pk : constant Integer := Jointboot.Peak_Of (T, St);
         Tp : constant Long_Float := (if Pk >= 0 then T (Natural (Pk)) else Long_Float'Last);
      begin
         Check (abs (Tp - 0.6) <= Stats.Z * 0.02, "核密度峰:25 个一致地报 0.6、40 个乱配散在 -3…0 ⇒ 峰在 " & Codec.Fmt (Tp, 3));
      end;
   end;

   --  🔴 ③ 门里还是乱配(Jointboot.Mix_Gate,10-01 P3B):按混合模型自己的分界(像门里的二维正态比像均匀落在画面上的乱配似然大)。
   --  错了会是什么病:原来的门 = max(Z, Z × 中位) 假定所有配点同一个尺度,两只腕眼隔 0.6 m 看近处桌面的那几十条,沿对极线的真误差比
   --  "一只手一个远近放大"给它的大,在真值处就有好几条卡在 3.4–3.5σ,门 3.3σ ⇒ 砍掉,远点成片那一拉把第 2 只手拖偏 32 mm(P3B 在线)。
   --  合成:1400 条门里的(二维单位正态)+ 20 条门里的尾巴(4.5σ:远近那一份比量到的放大还不准一点的那几条)+ 100 条乱配(20–100σ),
   --  每条的像素面积 0.1 px²、画幅 640 × 480 ⇒ 尾巴全留(4.5σ 像门里的比像均匀乱配的大几百倍)、乱配全挑掉。
   --  牙:分界换回 max(Z, Z × 中位)(≈ 3.75σ)⇒ 尾巴 20 条全砍,红(第一回焊的时候尾巴放在 3.6σ,旧门也留着它,牙咬不住)
   declare
      W2, Dens : Floats;
      Inl : Bools;
      Gamma : Long_Float;
      N_Tail, N_Junk : Natural := 0;
   begin
      for I in 0 .. 1399 loop
         W2.Append (Gauss ** 2 + Gauss ** 2); Dens.Append (0.1 / (640.0 * 480.0));
      end loop;
      for I in 0 .. 19 loop
         W2.Append (4.5 ** 2); Dens.Append (0.1 / (640.0 * 480.0));
      end loop;
      for I in 0 .. 99 loop
         W2.Append ((20.0 + 80.0 * U01) ** 2); Dens.Append (0.1 / (640.0 * 480.0));
      end loop;
      Jointboot.Mix_Gate (W2, Dens, Inl, Gamma);
      for I in 1400 .. 1419 loop
         if Inl (I) then
            N_Tail := N_Tail + 1;
         end if;
      end loop;
      for I in 1420 .. 1519 loop
         if Inl (I) then
            N_Junk := N_Junk + 1;
         end if;
      end loop;
      Check (N_Tail = 20 and then N_Junk = 0,
             "门里还是乱配:4.5σ 的尾巴留下 " & Codec.Img (N_Tail) & " / 20、乱配进门 " & Codec.Img (N_Junk) & " / 100(门里的占比 " & Codec.Fmt (Gamma, 3) & ")");
   end;

   --  🔴 ④ 第 2 只手放进世界真的有多不准(Jointboot.Place_Hand / Hand_Sd;10-01 V1B68 / V1B69 拿掉头顶眼:形式上说沿连线准到 1.9 mm,实际偏 38 mm)。
   --  合成的远墙(z = -50,x -25…35、y -18…25)400 个点:第 2 只手三角的远近顺着"沿连线挪 0.7 + 反着转 0.7 / 50"成片偏
   --  (每个点沿它自己的视线挑远近,让那个错的放法投进 5 格眼最对得上;真数据量到的就是这样一片 1–2%),每个点再随机 1%,自报 0.35%;
   --  另有 25 条乱配。只用远点,从错的放法起步 ⇒ 形式上的不确定度说它准(偏差是它的 Z 倍以上),真的不准(系统那一份按全相关线性加)罩得住偏差,
   --  而且比它自己的眼在那儿量一个点(桌面上的点自报的远近 × 量到的放大)还不准 ⇒ 判不准。
   --  错了会是什么病:形式上的不确定度当真 ⇒ 偏着的解判成准,不去看共同的近处。牙:系统那一份不加(Hand_Sd 只报形式的)⇒ 罩不住,红
   declare
      Shift : constant := 0.7;
      T_Wrong : constant V3 := [11.0 + Shift, 0.0, 0.0];
      R_Wrong : constant M3 := Rot ([0.0, 1.0, 0.0], Shift / 50.0);
      Far_Obs, Dummy : Jointboot.Hand_Ob_Vectors.Vector;
      Cb : V3;
      Pf : Jointboot.Hand_Place;
      Okf : Boolean;
   begin
      for I in 0 .. 399 loop
         declare
            Pw : constant V3 := [-25.0 + 60.0 * U01, -18.0 + 43.0 * U01, -50.0];
            Xt : constant V3 := [Pw (0) - T_True (0), Pw (1) - T_True (1), Pw (2) - T_True (2)];
            function Cost (Lam : Long_Float) return Long_Float is
               C : Long_Float := 0.0;
               Xw : constant V3 := Ap (R_Wrong, [Lam * Xt (0), Lam * Xt (1), Lam * Xt (2)]);
            begin
               for K in 0 .. N_Cam - 1 loop
                  declare
                     Ua, Va, Ub, Vb : Long_Float;
                     Oa, Ob : Boolean;
                  begin
                     Proj (Cams (K), Pw, Ua, Va, Oa);
                     Proj (Cams (K), [Xw (0) + T_Wrong (0), Xw (1) + T_Wrong (1), Xw (2) + T_Wrong (2)], Ub, Vb, Ob);
                     if Oa and then Ob then
                        C := C + (Ua - Ub) ** 2 + (Va - Vb) ** 2;
                     end if;
                  end;
               end loop;
               return C;
            end Cost;
            Lo : Long_Float := 0.9;
            Hi : Long_Float := 1.1;
         begin
            for It in 1 .. 80 loop   --  三分搜索
               declare
                  M1 : constant Long_Float := Lo + (Hi - Lo) / 3.0;
                  M2 : constant Long_Float := Hi - (Hi - Lo) / 3.0;
               begin
                  if Cost (M1) < Cost (M2) then
                     Hi := M2;
                  else
                     Lo := M1;
                  end if;
               end;
            end loop;
            declare
               Lam : constant Long_Float := 0.5 * (Lo + Hi) * (1.0 + 0.01 * Gauss);
               Xb : constant V3 := [Lam * Xt (0), Lam * Xt (1), Lam * Xt (2)];
            begin
               Observe (Pw, Xb, Cov_Of (Xb, 0.0035), Far_Obs);
            end;
         end;
      end loop;
      Junk (Far_Obs, 25);
      Near_Pts (Dummy, Cb);   --  只要它们的中心(干活的地方)和它们离第 2 只手的眼多远;它们的配点不进这一条
      Pf.S := 1.0; Pf.R := R_Wrong; Pf.T := T_Wrong;
      Jointboot.Place_Hand (Far_Obs, Tie_At (Cb), 0.3, Pf, Okf);
      declare
         Eye : constant Long_Float := Pf.K * 0.0007 * Norm (Cb);   --  它自己的眼在那儿量一个点:桌面上的点自报的远近 × 量到的放大(同 Align 的 Eye_Sd_Of)
         E : constant Long_Float := Err (Pf, Cb);
      begin
         Check (Okf and then E > Stats.Z * Pf.Sd_Formal and then E <= Stats.Z * Pf.Sd_Real and then Pf.Sd_Real > Eye,
                "第 2 只手只用远点放:偏 " & Codec.Fmt (E, 4) & " 单位 —— 形式上说 " & Codec.Fmt (Pf.Sd_Formal, 4) & "(偏差是它的 "
                & Codec.Fmt (E / Long_Float'Max (Pf.Sd_Formal, 1.0e-12), 1) & " 倍,不诚实),真的不准 " & Codec.Fmt (Pf.Sd_Real, 4) & "(远近放大 "
                & Codec.Fmt (Pf.K, 2) & " 倍,罩得住),比它自己的眼在那儿量点的 " & Codec.Fmt (Eye, 4) & " 还大 ⇒ 判不准");
      end;
   end;

   --  🔴 ⑤ 出坑(Jointboot.Place_Hand;10-01 V1B68 / V1B69 / P3B:网格起步落在远点那一坑,近处桌面那几十个点在那儿差二三十像素全被门挑掉,精修锁死在偏的解上)。
   --  合成:远处一面墙(z = -200)上的点,第 2 只手三角出的位置和"沿连线挪 0.7 + 反着转 0.7 / 200"那个错的放法正好对得上(墙远,两个放法投进
   --  5 格眼差不到 0.2 px,远点分不出),远近随机 1%;两手之间桌面上 15 个近点按真的放法;25 条乱配。从错的放法起步 ⇒ 近点在那儿差 28 px,门外;
   --  远点一点都不拽 ⇒ 光精修不动。出坑:近点沿最不准的方向各报的挪法一致 ⇒ 去那一坑再解,混合似然低得多 ⇒ 换。
   --  错了会是什么病:锁死在偏 0.7 单位的解上,照样往下走。牙:不出坑(出坑那一段拿掉)⇒ 差真值 0.66 单位,红
   declare
      Shift : constant := 0.7;
      T_Wrong : constant V3 := [11.0 + Shift, 0.0, 0.0];
      R_Wrong : constant M3 := Rot ([0.0, 1.0, 0.0], Shift / 200.0);
      Obs : Jointboot.Hand_Ob_Vectors.Vector;
      Cb : V3;
      Pa : Jointboot.Hand_Place;
      Oka : Boolean;
      Tie : Jointboot.Plane_Tie;
   begin
      for I in 0 .. 399 loop
         declare
            Pw : constant V3 := [-60.0 + 160.0 * U01, -60.0 + 100.0 * U01, -200.0];
            Xw : constant V3 := Ap (Tr (R_Wrong), [Pw (0) - T_Wrong (0), Pw (1) - T_Wrong (1), Pw (2) - T_Wrong (2)]);   --  错的放法下正好对得上
            L : constant Long_Float := 1.0 + 0.01 * Gauss;
            Xb : constant V3 := [L * Xw (0), L * Xw (1), L * Xw (2)];
         begin
            Observe (Pw, Xb, Cov_Of (Xb, 0.0035), Obs);
         end;
      end loop;
      Near_Pts (Obs, Cb);
      Junk (Obs, 25);
      Tie := Tie_At (Cb);
      Pa.S := 1.0; Pa.R := R_Wrong; Pa.T := T_Wrong;
      Jointboot.Place_Hand (Obs, Tie, 0.3, Pa, Oka);
      Check (Oka and then Pa.Escapes >= 1 and then Err (Pa, Cb) <= Stats.Z * Pa.Sd_Real and then Err (Pa, Cb) < Shift / Stats.Z,
             "第 2 只手从远点那一坑起步:出坑 " & Codec.Img (Pa.Escapes) & " 次 ⇒ 干活的地方差真值 " & Codec.Fmt (Err (Pa, Cb), 4) & " 单位(起步偏 "
             & Codec.Fmt (Shift, 1) & "),自报真的不准 " & Codec.Fmt (Pa.Sd_Real, 4) & "、门里 " & Codec.Img (Pa.Inl) & " / " & Codec.Img (Natural (Obs.Length)));
   end;

   --  🔴 ⑥ 世界取能定世界的第一只手(Jointboot.World_Arm;旧不足 4:原来第一组读数写死当世界,第 0 只手扫坏了整个开机就退出,"定不了世界")。
   --  第 0 只手当不了(-1)、第 1、2 只当得了 ⇒ 第 1 只;只有第 0 只当不了 ⇒ 第 1 只;第 0 只当得了 ⇒ 第 0 只(和原来一样);都当不了 ⇒ -1。
   --  牙:World_Arm 写回"第 0 只" ⇒ 红
   declare
      A1, A2, A3, A4 : Floats;
   begin
      A1.Append (-1.0); A1.Append (0.002); A1.Append (0.001);
      A2.Append (-1.0); A2.Append (0.0013);
      A3.Append (-1.0); A3.Append (-1.0);
      A4.Append (0.003); A4.Append (0.001);
      Check (Jointboot.World_Arm (A1) = 1 and then Jointboot.World_Arm (A2) = 1 and then Jointboot.World_Arm (A3) = -1 and then Jointboot.World_Arm (A4) = 0,
             "世界取能定世界的第一只手:[当不了, 0.002, 0.001] ⇒ 第" & Integer'Image (Jointboot.World_Arm (A1)) & " 只;第 0 只扫坏了 ⇒ 第"
             & Integer'Image (Jointboot.World_Arm (A2)) & " 只;都当不了 ⇒" & Integer'Image (Jointboot.World_Arm (A3)) & ";第 0 只量成了 ⇒ 第"
             & Integer'Image (Jointboot.World_Arm (A4)) & " 只");
   end;

   --  🔴 ⑦ 眼长在谁身上(Jointboot.Eye_List,I3;10-01 P8A):开机认出第 1 只手 = 第 0 组读数、眼 = 第 1 台,第 2 只手 = 第 1 组、眼 = 第 2 台,
   --  第 0 台推哪组都不动;第 2 只手扫描撞上柜子把手、运动学没量成。错了会是什么病:往下只认量成的手上的眼,第 2 台就成了"不长在臂上的",
   --  告诉脑 "through the eye that does not move with me (camera index 2)"。这里:第 2 台照样长在第 1 组上(没装上、用不了,不是不动的眼),
   --  第 0 台不动、放进了世界,第 1 台在世界那只手上。反过来第 0 只手扫坏了、世界取第 2 只:第 1 台照样长在第 0 组上(没装上),第 2 台装上是第 1 只(下标 0)。
   --  牙:Eye_List 只收量成的手上的眼 ⇒ 第 2 台判成量不清 / 不动,红
   declare
      Eyes, Groups : Ints;
      Valid : Bools;
      L, L2 : Jointboot.Eye_Vectors.Vector;
      use type Jointboot.Eye_Carrier;
   begin
      Eyes.Append (1); Eyes.Append (2);
      Groups.Append (0); Groups.Append (1);
      Valid.Append (True); Valid.Append (False);
      L := Jointboot.Eye_List (3, Eyes, Groups, Valid, 0, 0, True);
      Valid.Replace_Element (0, False); Valid.Replace_Element (1, True);
      L2 := Jointboot.Eye_List (3, Eyes, Groups, Valid, 0, 1, True);
      Check (L (2).Kind = Jointboot.On_Group and then L (2).Group = 1 and then L (2).Arm = -1 and then L (0).Kind = Jointboot.Still and then L (0).Placed
             and then L (1).Kind = Jointboot.On_Group and then L (1).Arm = 0 and then L (1).World
             and then L2 (1).Kind = Jointboot.On_Group and then L2 (1).Group = 0 and then L2 (1).Arm = -1 and then L2 (2).Arm = 0 and then L2 (2).World,
             "眼长在谁身上(P8A):" & Jointboot.Eye_Say (L));
   end;
end Welds_Path_3;
