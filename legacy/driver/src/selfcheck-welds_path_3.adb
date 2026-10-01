with Stats;
with Things;
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
   --  ── I4 一件东西的估计(Things)──:合成的桌面 z = 0 上一块 4 × 3 × 2 的箱子(x ∈ ±2、y ∈ ±1.5、z ∈ 0 … 2);
   --  四只眼从四边、高 2.5、离 20 看,一只从正上方 22 看(焦距 400、640 × 480;掩膜 = 像素中心那条视线碰得到箱子)
   declare
      Bx_Lo : constant V3 := [-2.0, -1.5, 0.0];
      Bx_Hi : constant V3 := [2.0, 1.5, 2.0];
      Up_Z : constant V3 := [0.0, 0.0, 1.0];
      function Look_At (P, T, Up : V3) return Cam_Geo is
         G : Cam_Geo;
         Fw : V3 := [T (0) - P (0), T (1) - P (1), T (2) - P (2)];
         Nf : constant Long_Float := Norm (Fw);
         Xc, Yc : V3;
      begin
         Fw := [Fw (0) / Nf, Fw (1) / Nf, Fw (2) / Nf];
         Xc := [Fw (1) * Up (2) - Fw (2) * Up (1), Fw (2) * Up (0) - Fw (0) * Up (2), Fw (0) * Up (1) - Fw (1) * Up (0)];
         declare
            Nx : constant Long_Float := Norm (Xc);
         begin
            Xc := [Xc (0) / Nx, Xc (1) / Nx, Xc (2) / Nx];
         end;
         Yc := [Xc (1) * Fw (2) - Xc (2) * Fw (1), Xc (2) * Fw (0) - Xc (0) * Fw (2), Xc (0) * Fw (1) - Xc (1) * Fw (0)];
         for I in 0 .. 2 loop
            G.R_Ce (I, 0) := Xc (I); G.R_Ce (I, 1) := Yc (I); G.R_Ce (I, 2) := -Fw (I);
         end loop;
         G.Pos := P; G.F := F; G.Cx := Cx; G.Cy := Cy; G.Valid := True; G.Fixed := True;
         return G;
      end Look_At;
      --  一条视线碰不碰得到箱子(三对平面夹出来的那一段,纯几何)
      function Hits (O, D, Lo, Hi : V3) return Boolean is
         T0 : Long_Float := 0.0;
         T1 : Long_Float := Long_Float'Last;
      begin
         for I in 0 .. 2 loop
            if D (I) = 0.0 then
               if O (I) < Lo (I) or else O (I) > Hi (I) then
                  return False;
               end if;
            else
               declare
                  A : constant Long_Float := (Lo (I) - O (I)) / D (I);
                  B : constant Long_Float := (Hi (I) - O (I)) / D (I);
               begin
                  T0 := Long_Float'Max (T0, Long_Float'Min (A, B)); T1 := Long_Float'Min (T1, Long_Float'Max (A, B));
               end;
            end if;
         end loop;
         return T0 <= T1;
      end Hits;
      function Render (G : Cam_Geo; Lo, Hi : V3) return Bools is
         M : Bools;
      begin
         M.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               M (Y * Wd + X) := Hits (G.Pos, Ray_Fixed (G, Long_Float (X), Long_Float (Y)), Lo, Hi);
            end loop;
         end loop;
         return M;
      end Render;
      --  一眼:掩膜只留窗里的(分割只在窗里作数);窗 = 掩膜外接框四面各让一个像素(没给就这么取)
      function View_Of (G : Cam_Geo; Idx : Natural; M : Bools; X0, Y0, X1, Y1 : Integer := -1) return Things.View is
         V : Things.View;
         Ax0, Ay0 : Integer := Integer'Last;
         Ax1, Ay1 : Integer := -1;
      begin
         V.Cam := G; V.Cam_Index := Idx; V.W := Wd; V.H := Ht; V.Mask := M;
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               if M (Y * Wd + X) then
                  Ax0 := Integer'Min (Ax0, X); Ay0 := Integer'Min (Ay0, Y); Ax1 := Integer'Max (Ax1, X); Ay1 := Integer'Max (Ay1, Y);
               end if;
            end loop;
         end loop;
         if X0 >= 0 then
            V.X0 := X0; V.Y0 := Y0; V.X1 := X1; V.Y1 := Y1;
         else
            V.X0 := Integer'Max (0, Ax0 - 1); V.Y0 := Integer'Max (0, Ay0 - 1); V.X1 := Integer'Min (Wd - 1, Ax1 + 1); V.Y1 := Integer'Min (Ht - 1, Ay1 + 1);
         end if;
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               if X < V.X0 or else X > V.X1 or else Y < V.Y0 or else Y > V.Y1 then
                  V.Mask (Y * Wd + X) := False;
               end if;
            end loop;
         end loop;
         return V;
      end View_Of;
      Gs : array (0 .. 5) of Cam_Geo;
      --  箱子表面的真值采样:每面 9 × 9
      Truth : V3_Vectors.Vector;
      procedure Sample_Box (Lo, Hi : V3) is
      begin
         Truth.Clear;
         for Ax in 0 .. 2 loop
            for Sd in 0 .. 1 loop
               for I in 0 .. 8 loop
                  for J in 0 .. 8 loop
                     declare
                        P : V3;
                        A1 : constant Natural := (Ax + 1) mod 3;
                        A2 : constant Natural := (Ax + 2) mod 3;
                     begin
                        P (Ax) := (if Sd = 0 then Lo (Ax) else Hi (Ax));
                        P (A1) := Lo (A1) + (Hi (A1) - Lo (A1)) * Long_Float (I) / 8.0;
                        P (A2) := Lo (A2) + (Hi (A2) - Lo (A2)) * Long_Float (J) / 8.0;
                        Truth.Append (P);
                     end;
                  end loop;
               end loop;
            end loop;
         end loop;
      end Sample_Box;
      --  真值的点有几成落在外包里(外包 = 没被雕掉的格子)。格子的边是起步格一半一半分出来的,舍入差几个末位;底面那些真值点正好躺在
      --  面上(z = 0),格子的底边落在 0 的哪一侧只差末位 —— 判"在格子里"放宽格子自己的几个末位(数值)
      function Covered (E : Things.Estimate) return Long_Float is
         N : Natural := 0;
         function Within (A, C, H : Long_Float) return Boolean is
           (abs (A - C) <= H + Long_Float'Model_Epsilon * (abs C + H + abs A));
      begin
         for P of Truth loop
            for Cl of E.Solid loop
               if Within (P (0), Cl.C (0), Cl.H) and then Within (P (1), Cl.C (1), Cl.H) and then Within (P (2), Cl.C (2), Cl.H) then
                  N := N + 1;
                  exit;
               end if;
            end loop;
         end loop;
         return Long_Float (N) / Long_Float (Truth.Length);
      end Covered;
      --  独立的判法(按真值的箱子,不经 Things 的积分图、窗、八叉树):这一格的八个角投进第 K 只眼,外接框放宽 Z 倍像素量化、再多一个像素,
      --  框里每个像素中心的视线都碰不到箱子 ⇒ 这一格明明是空的。外包里留着这样的格子 = 该雕没雕
      function Clearly_Free (Cl : Things.Cell; Lo, Hi : V3) return Boolean is
         D : constant Long_Float := Stats.Z / Sqrt (12.0) + 1.0;
      begin
         for K in 0 .. 4 loop
            declare
               U0, V0 : Long_Float := Long_Float'Last;
               U1, V1 : Long_Float := Long_Float'First;
               All_Front : Boolean := True;
               Any_Hit : Boolean := False;
            begin
               for Sx in 0 .. 1 loop
                  for Sy in 0 .. 1 loop
                     for Sz in 0 .. 1 loop
                        declare
                           P : constant V3 := [Cl.C (0) + (if Sx = 0 then -Cl.H else Cl.H), Cl.C (1) + (if Sy = 0 then -Cl.H else Cl.H),
                                               Cl.C (2) + (if Sz = 0 then -Cl.H else Cl.H)];
                           U, V : Long_Float;
                           Ok : Boolean;
                        begin
                           Project_Fixed (Gs (K), P, U, V, Ok);
                           if Ok then
                              U0 := Long_Float'Min (U0, U); U1 := Long_Float'Max (U1, U); V0 := Long_Float'Min (V0, V); V1 := Long_Float'Max (V1, V);
                           else
                              All_Front := False;
                           end if;
                        end;
                     end loop;
                  end loop;
               end loop;
               if All_Front and then U1 - U0 < Long_Float (Wd) and then V1 - V0 < Long_Float (Ht) then
                  for Y in Integer (Long_Float'Ceiling (V0 - D)) .. Integer (Long_Float'Floor (V1 + D)) loop
                     for X in Integer (Long_Float'Ceiling (U0 - D)) .. Integer (Long_Float'Floor (U1 + D)) loop
                        if Hits (Gs (K).Pos, Ray_Fixed (Gs (K), Long_Float (X), Long_Float (Y)), Lo, Hi) then
                           Any_Hit := True;
                           exit;
                        end if;
                     end loop;
                     exit when Any_Hit;
                  end loop;
                  if not Any_Hit then
                     return True;
                  end if;
               end if;
            end;
         end loop;
         return False;
      end Clearly_Free;
      E0 : Things.Estimate;
      Ms : array (0 .. 4) of Bools;
      T_Start : Ada.Calendar.Time;
      Solve_S : Duration;
      use type Ada.Calendar.Time;
   begin
      Gs (0) := Look_At ([0.0, -20.0, 2.5], [0.0, 0.0, 1.0], Up_Z);
      Gs (1) := Look_At ([20.0, 0.0, 2.5], [0.0, 0.0, 1.0], Up_Z);
      Gs (2) := Look_At ([0.0, 20.0, 2.5], [0.0, 0.0, 1.0], Up_Z);
      Gs (3) := Look_At ([-20.0, 0.0, 2.5], [0.0, 0.0, 1.0], Up_Z);
      Gs (4) := Look_At ([0.0, 0.0, 22.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]);
      Gs (5) := Look_At ([14.0, -14.0, 10.0], [0.0, 0.0, 1.0], Up_Z);
      for K in 0 .. 4 loop
         Ms (K) := Render (Gs (K), Bx_Lo, Bx_Hi);
      end loop;
      Sample_Box (Bx_Lo, Bx_Hi);

      --  🔴 ⑧ 五只眼雕出来的外包(Things.Solve):真值的点全在外包里(雕多了 = 把真的边雕掉);外包里没有一格是"按真值的箱子明明是空的"
      --  (独立的判法,见 Clearly_Free;雕少了 = 该雕没雕)—— 外包本来就比箱子大一圈(四只侧眼从 2.5 高看,箱子底边外头那一圈从哪只眼看都挡在箱子前面,
      --  轮廓分不出它和箱子),这一圈不算错;底面那一面(外法向朝下)没有眼看过、顶面看过;同一只眼没挪的第二眼换掉上一眼(眼数不涨);
      --  只有一只眼(一处)⇒ 不雕,说沿视线多厚说不出(锥一直通到眼上)。
      --  错了会是什么病:放宽那一圈没有(像素量化 ½ 像素就把真的边雕掉)⇒ 接触集挑到它外头;窗外当"不是它"⇒ 截了一截。
      --  牙:Dilate 返回 0 ⇒ 真值落在外包外,红
      E0.Name := To_Unbounded_String ("box");
      Things.Set_Support (E0, [0.0, 0.0, 0.0], Up_Z);
      for K in 0 .. 4 loop
         Things.Add_View (E0, View_Of (Gs (K), K, Ms (K)));
      end loop;
      Things.Add_View (E0, View_Of (Gs (4), 4, Ms (4)));   --  同一只眼没挪:换掉上一眼
      T_Start := Ada.Calendar.Clock;
      Things.Solve (E0);
      Solve_S := Ada.Calendar.Clock - T_Start;
      declare
         Bottom_Seen, Top_Unseen, N_Bottom, N_Top, N_Wrong : Natural := 0;
         One : Things.Estimate;
         Cov : constant Long_Float := Covered (E0);
      begin
         for P of E0.Surface loop
            if P.N = [0.0, 0.0, -1.0] then
               N_Bottom := N_Bottom + 1;
               if P.Seen then
                  Bottom_Seen := Bottom_Seen + 1;
               end if;
            elsif P.N = [0.0, 0.0, 1.0] then
               N_Top := N_Top + 1;
               if not P.Seen then
                  Top_Unseen := Top_Unseen + 1;
               end if;
            end if;
         end loop;
         for Cl of E0.Solid loop
            if Clearly_Free (Cl, Bx_Lo, Bx_Hi) then
               N_Wrong := N_Wrong + 1;
            end if;
         end loop;
         One.Name := To_Unbounded_String ("box");
         Things.Set_Support (One, [0.0, 0.0, 0.0], Up_Z);
         Things.Add_View (One, View_Of (Gs (4), 4, Ms (4)));
         Things.Solve (One);
         Check (E0.Valid and then Cov = 1.0 and then N_Wrong = 0 and then Natural (E0.Views.Length) = 5
                and then N_Bottom > 0 and then Bottom_Seen = 0 and then N_Top > 0 and then Top_Unseen = 0
                and then not E0.Thick_Unknown and then not One.Valid and then One.Thick_Unknown,
                "一件东西的外包(五只眼,I4):真值的点 " & Codec.Img (Natural (Long_Float'Floor (100.0 * Cov))) & "% 在外包里;外包 "
                & Codec.Img (Natural (E0.Solid.Length)) & " 格里按真值明明是空的 " & Codec.Img (N_Wrong) & " 格;外接盒 x "
                & Codec.Fmt (E0.Lo (0), 3) & " … " & Codec.Fmt (E0.Hi (0), 3) & "、y " & Codec.Fmt (E0.Lo (1), 3) & " … " & Codec.Fmt (E0.Hi (1), 3)
                & "、z " & Codec.Fmt (E0.Lo (2), 3) & " … " & Codec.Fmt (E0.Hi (2), 3) & "(箱子 ±2 / ±1.5 / 0 … 2,间距 " & Codec.Fmt (E0.Pitch, 3) & ");"
                & Codec.Img (Natural (E0.Surface.Length)) & " 个表面点,底面 " & Codec.Img (Bottom_Seen) & " / " & Codec.Img (N_Bottom) & " 看过、顶面 "
                & Codec.Img (N_Top - Top_Unseen) & " / " & Codec.Img (N_Top) & " 看过;眼 " & Codec.Img (Natural (E0.Views.Length))
                & " 眼;只一只眼 ⇒ 不雕(" & Boolean'Image (One.Valid) & ")、多厚说不出 " & Boolean'Image (One.Thick_Unknown)
                & ";解了 " & Codec.Fmt (Long_Float (Solve_S), 3) & " s");
      end;

      --  🔴 ⑨ 窗切着它的那一眼(Whole = False):第 1 只眼的窗只盖住箱子在它眼里的左半边(掩膜顶着窗边)⇒ 窗外不算"不是它",外包照样兜住整个箱子;
      --  错了会是什么病:窗外当"不是它" ⇒ 窗外那一半被这一眼雕掉,接触集只在剩下那一截里挑(10-01 路 5:C1 第 102 拍 482 / 1704、S1A5 只剩转轴)。
      --  牙:Judge 不分 Whole(窗外一律当空)⇒ 真值落在外包外,红
      declare
         E : Things.Estimate;
         Vc : Things.View := View_Of (Gs (1), 1, Ms (1));
      begin
         Vc := View_Of (Gs (1), 1, Ms (1), Vc.X0, Vc.Y0, (Vc.X0 + Vc.X1) / 2, Vc.Y1);
         E.Name := To_Unbounded_String ("box");
         Things.Set_Support (E, [0.0, 0.0, 0.0], Up_Z);
         for K in 0 .. 4 loop
            Things.Add_View (E, (if K = 1 then Vc else View_Of (Gs (K), K, Ms (K))));
         end loop;
         Things.Solve (E);
         Check (E.Valid and then not E.Views (1).Whole and then E.Views (0).Whole and then Covered (E) = 1.0,
                "窗切着它的那一眼:第 1 只眼的窗只盖住左半边 ⇒ 判成没整个在窗里(" & Boolean'Image (E.Views (1).Whole) & "),真值的点 "
                & Codec.Img (Natural (Long_Float'Floor (100.0 * Covered (E)))) & "% 在外包里;判动过 " & Codec.Img (E.Moves) & " 次、眼 "
                & Codec.Img (Natural (E.Views.Length)) & " 眼(" & To_String (E.Note) & ")");
      end;

      --  🔴 ⑩ 被我自己挡着的像素(Occl,腕眼自己的手指):第 0 只眼右边三分之一被手指挡着,掩膜里没有那一截 ⇒ 挡着的地方不算"不是它",
      --  这一眼也不算整个在窗里;外包照样兜住。错了会是什么病:手指后面那一截被雕掉,抓的时候以为它比真的短。
      --  牙:Judge 不数挡着的像素、Whole 不看挨不挨着挡着的 ⇒ 真值落在外包外,红
      declare
         E : Things.Estimate;
         V0 : Things.View := View_Of (Gs (0), 0, Ms (0));
         Cut_X : constant Integer := V0.X0 + 2 * (V0.X1 - V0.X0) / 3;
      begin
         V0.Occl.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               V0.Occl (Y * Wd + X) := X >= Cut_X;
               if X >= Cut_X then
                  V0.Mask (Y * Wd + X) := False;
               end if;
            end loop;
         end loop;
         E.Name := To_Unbounded_String ("box");
         Things.Set_Support (E, [0.0, 0.0, 0.0], Up_Z);
         Things.Add_View (E, V0);
         for K in 1 .. 4 loop
            Things.Add_View (E, View_Of (Gs (K), K, Ms (K)));
         end loop;
         Things.Solve (E);
         Check (E.Valid and then not E.Views (0).Whole and then Covered (E) = 1.0,
                "被我自己挡着的像素:第 0 只眼右边三分之一被手指挡着 ⇒ 不算整个在窗里(" & Boolean'Image (E.Views (0).Whole) & "),真值的点 "
                & Codec.Img (Natural (Long_Float'Floor (100.0 * Covered (E)))) & "% 在外包里;判动过 " & Codec.Img (E.Moves) & " 次、眼 "
                & Codec.Img (Natural (E.Views.Length)) & " 眼");
      end;

      --  🔴 ⑪ 它动过了(Add_View 核新的一眼和外包对不对得上):五只眼雕好以后,从新的一处(14, −14, 10)看 —— 箱子没动 ⇒ 对得上,眼数 6;
      --  箱子挪了 x + 1.5 ⇒ 对不上 ⇒ 判它动过、以前的眼作废(只剩这一眼,Moves = 1)。
      --  错了会是什么病:挪过的东西前后两处的锥交在一起,外包只剩两处重叠的那一小块(或者整个雕没了)。牙:Off_Hull 永远说对得上 ⇒ 红
      declare
         Same, Shifted : Things.Estimate := E0;
         Ms5 : constant Bools := Render (Gs (5), Bx_Lo, Bx_Hi);
         Ms5b : constant Bools := Render (Gs (5), [Bx_Lo (0) + 1.5, Bx_Lo (1), Bx_Lo (2)], [Bx_Hi (0) + 1.5, Bx_Hi (1), Bx_Hi (2)]);
      begin
         Things.Add_View (Same, View_Of (Gs (5), 5, Ms5));
         Things.Add_View (Shifted, View_Of (Gs (5), 5, Ms5b));
         Check (Same.Moves = 0 and then Natural (Same.Views.Length) = 6 and then Shifted.Moves = 1 and then Natural (Shifted.Views.Length) = 1,
                "它动过了:没动 ⇒ 判动过 " & Codec.Img (Same.Moves) & " 次、眼 " & Codec.Img (Natural (Same.Views.Length)) & " 眼;挪了 1.5 ⇒ 判动过 "
                & Codec.Img (Shifted.Moves) & " 次、剩 " & Codec.Img (Natural (Shifted.Views.Length)) & " 眼");
      end;

      --  🔴 ⑬ 哪儿算"不是它"(Things.Reach,10-01 C1 重放):第 0 只眼里箱子右边挨着一块挡着的像素(从箱子一直连到画幅右边,像腕眼自己的手指),
      --  掩膜没顶着窗边 ⇒ 说不出的只有挨着它的那一块(Unknown)、它可能伸到画幅外(Beyond);窗外没被挡着的像素照样不是它。
      --  按三处像素各取一点(离眼 20):箱子上方窗外 ⇒ Free;挡着的那一块里 ⇒ No_Info;画幅外 ⇒ No_Info(它可能从挡着的那一块伸出去)。
      --  错了会是什么病:挨着手指 ⇒ 窗外全算说不出,腕眼只在窗里雕,外包顺着别的眼的锥一直伸到眼上(C1:伸到 10 单位高)。
      --  牙:挨着挡着的像素就当它顶着窗边(窗外全算说不出)⇒ 箱子上方那一点 No_Info,红
      declare
         V0 : Things.View := View_Of (Gs (0), 0, Ms (0));
         Cut_X : constant Integer := V0.X0 + 2 * (V0.X1 - V0.X0) / 3;
         function Pt_At (U, V : Long_Float) return V3 is
            D : constant V3 := Ray_Fixed (Gs (0), U, V);
         begin
            return [Gs (0).Pos (0) + 20.0 * D (0), Gs (0).Pos (1) + 20.0 * D (1), Gs (0).Pos (2) + 20.0 * D (2)];
         end Pt_At;
         use type Things.Verdict;
      begin
         V0.Occl.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               V0.Occl (Y * Wd + X) := X >= Cut_X and then Y >= V0.Y0 and then Y <= V0.Y1;
            end loop;
         end loop;
         Things.Reach (V0);
         declare
            Above : constant Things.Verdict := Things.Point_In (V0, Pt_At (Long_Float ((V0.X0 + V0.X1) / 2), Long_Float (V0.Y0 - 40)));
            Hid : constant Things.Verdict := Things.Point_In (V0, Pt_At (Long_Float (Cut_X + 5), Long_Float ((V0.Y0 + V0.Y1) / 2)));
            Off : constant Things.Verdict := Things.Point_In (V0, Pt_At (Long_Float (Wd + 60), Long_Float ((V0.Y0 + V0.Y1) / 2)));
         begin
            Check (V0.Beyond and then not V0.Whole and then Above = Things.Free and then Hid = Things.No_Info and then Off = Things.No_Info,
                   "哪儿算不是它:挨着一块连到画幅边的挡着的像素 ⇒ 伸到画幅外 " & Boolean'Image (V0.Beyond) & ";箱子上方窗外 " & Things.Verdict'Image (Above)
                   & "、挡着的那一块里 " & Things.Verdict'Image (Hid) & "、画幅外 " & Things.Verdict'Image (Off));
         end;
      end;

      --  🔴 ⑭ 两眼夹着的那一截(Core,10-01 C1 重放):正上方那只眼看全了箱子;第 0 只眼(侧上方)里箱子上方整片被挡着(挡着的那一片挨着箱子、连到画幅上边)
      --  ⇒ 箱子上方顺着正上方那只眼的视线一直到眼上,只有一眼说得上话(那儿有没有东西说不出):外包里有这一截,Core 里没有;
      --  Core 的形心落在箱子里(按格子的半边长 + 放宽那一圈的世界尺寸),外包的上沿伸到箱子顶以上;真值的点全在外包里。
      --  错了会是什么病:形心按整个外包算,被那一截拉到半空(C1:形心高 3.4 单位,东西本身 0.2),接触集、"走过去"都朝着半空。
      --  牙:形心按整个外包算 ⇒ 红
      declare
         E : Things.Estimate;
         V0 : Things.View := View_Of (Gs (0), 0, Ms (0));
         Tol : Long_Float;
      begin
         V0.Occl.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               V0.Occl (Y * Wd + X) := Y < V0.Y0 + 2;
            end loop;
         end loop;
         for Y in 0 .. Ht - 1 loop
            for X in 0 .. Wd - 1 loop
               if V0.Occl (Y * Wd + X) then
                  V0.Mask (Y * Wd + X) := False;
               end if;
            end loop;
         end loop;
         E.Name := To_Unbounded_String ("box");
         Things.Set_Support (E, [0.0, 0.0, 0.0], Up_Z);
         Things.Add_View (E, V0);
         Things.Add_View (E, View_Of (Gs (4), 4, Ms (4)));
         Things.Solve (E);
         Tol := E.Pitch + E.Center_Sd;
         Check (E.Valid and then E.One_Look_Frac > 0.0 and then E.Hull_Hi (2) > Bx_Hi (2) + Tol and then Covered (E) = 1.0
                and then E.Center (0) >= Bx_Lo (0) - Tol and then E.Center (0) <= Bx_Hi (0) + Tol
                and then E.Center (1) >= Bx_Lo (1) - Tol and then E.Center (1) <= Bx_Hi (1) + Tol
                and then E.Center (2) >= Bx_Lo (2) - Tol and then E.Center (2) <= Bx_Hi (2) + Tol,
                "两眼夹着的那一截:只一眼说得上话的占外包的 " & Codec.Fmt (E.One_Look_Frac, 3) & ";外包上沿 " & Codec.Fmt (E.Hull_Hi (2), 2)
                & "(箱子顶 2)· Core 的形心 (" & Codec.Fmt (E.Center (0), 3) & ", " & Codec.Fmt (E.Center (1), 3) & ", " & Codec.Fmt (E.Center (2), 3)
                & ")、上沿 " & Codec.Fmt (E.Hi (2), 3) & ";真值的点 " & Codec.Img (Natural (Long_Float'Floor (100.0 * Covered (E)))) & "% 在外包里");
      end;
   end;

   --  🔴 ⑫ 窗随它长(Picture.Grow_Window / Covers_Interior;10-01 路 5:下一帧的框 = 这一帧掩膜的外接框,分割又不出框 ⇒ 框只缩不长,
   --  C1 第 102 拍窗只盖住 28%,S1A5 只剩剪刀转轴)。量到的那一块顶着窗的右边(窗不在画幅边上)⇒ 往右长出它自己那么宽;
   --  顶着画幅左边 ⇒ 左边不长;哪边都没顶着 ⇒ 不长。新的掩膜盖住旧的里头 ⇒ 同一件;换成旁边另一块 ⇒ 不是。
   --  错了会是什么病:窗切着的东西永远只量到那一截(原来判的是"顶没顶到画幅边")。牙:Grow_Window 不长 ⇒ 红
   declare
      R1 : Picture.Region;
      X0, Y0, X1, Y1 : Natural;
      G1, G2, G3 : Boolean;
      Old_M, New_M, Other_M : Bools;
      Rx : constant Natural := 300;
   begin
      R1.X0 := 250; R1.X1 := Rx; R1.Y0 := 100; R1.Y1 := 140;
      X0 := 250; Y0 := 90; X1 := Rx; Y1 := 150;
      Picture.Grow_Window (R1, Wd, Ht, X0, Y0, X1, Y1, G1);
      declare
         Ok1 : constant Boolean := G1 and then X1 = Rx + (Rx - 250 + 1) and then X0 = 250 - (Rx - 250 + 1) and then Y0 = 90 and then Y1 = 150;
         Ax0, Ay0, Ax1, Ay1 : Natural;
      begin
         R1.X0 := 0; R1.X1 := 40; R1.Y0 := 100; R1.Y1 := 140;
         Ax0 := 0; Ay0 := 90; Ax1 := 60; Ay1 := 150;
         Picture.Grow_Window (R1, Wd, Ht, Ax0, Ay0, Ax1, Ay1, G2);
         R1.X0 := 10; R1.X1 := 40; R1.Y0 := 100; R1.Y1 := 140;
         Picture.Grow_Window (R1, Wd, Ht, Ax0, Ay0, Ax1, Ay1, G3);
         Old_M.Set_Length (Ada.Containers.Count_Type (Wd * Ht)); New_M.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         Other_M.Set_Length (Ada.Containers.Count_Type (Wd * Ht));
         for I in 0 .. Wd * Ht - 1 loop
            declare
               X : constant Natural := I mod Wd;
               Y : constant Natural := I / Wd;
            begin
               Old_M (I) := X in 250 .. Rx and then Y in 100 .. 140;
               New_M (I) := X in 250 .. 380 and then Y in 100 .. 140;
               Other_M (I) := X in 330 .. 380 and then Y in 100 .. 140;
            end;
         end loop;
         R1.X0 := 250; R1.X1 := Rx; R1.Y0 := 100; R1.Y1 := 140;
         Check (Ok1 and then not G2 and then not G3 and then Picture.Covers_Interior (New_M, Old_M, Wd, Ht, R1)
                and then not Picture.Covers_Interior (Other_M, Old_M, Wd, Ht, R1),
                "窗随它长:顶着窗边 ⇒ 窗 x " & Codec.Img (X0) & " … " & Codec.Img (X1) & "(长了 " & Boolean'Image (G1) & ");顶着画幅边 ⇒ 长了 "
                & Boolean'Image (G2) & ";没顶着 ⇒ 长了 " & Boolean'Image (G3) & ";长大的那一块盖住旧的里头、旁边另一块盖不住");
      end;
   end;
   --  🔴 ⑮ 不动的眼解镜头畸变(Geom.Fit_Fixed_Board,10-01 P8WD:三台眼都加了 k1 −0.15 / k2 0.03,驱动哪儿都不解畸变,头顶眼放不进世界):
   --  合成的头顶眼(焦距 288、640 × 480、斜着看 1.2 米外的桌面),板上的点按画面里铺满的格子打到桌面上(高低起伏几厘米),配点按 0.3 px 抖。
   --  带畸变的那只:畸变进了解(F 检验显著),K1 / K2 / 焦距 / 位置都在它自报的 Z 倍不确定度以内;不带的那只:畸变不进解(针孔),K1 = K2 = 0。
   --  错了会是什么病:针孔去拟合有畸变的镜头 ⇒ 焦距、位置整个歪掉(P8WD 腕眼焦距 645 / 真 397,两手对齐差 586 mm),头顶眼放不进世界。
   --  牙:K1 / K2 不放开(只解针孔)⇒ 带畸变的那只焦距 / 位置出了自报的不确定度(或解不出),红
   declare
      Gt0 : Geom.Cam_Geo;
      Seed : Long_Long_Integer := 29;
      function Jit return Long_Float is   --  确定性伪随机 ±1(测试数据自己的抖动)
      begin
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Integer ((Seed / 65536) mod 2001) - 1000) / 1000.0;
      end Jit;
      --  画面里 16 × 12 个格点各发一条视线打到桌面(z = 0.8 上下几厘米)上,再按真相机(带不带畸变)投回去、抖 Sh
      procedure Board (Gt : Geom.Cam_Geo; Sh : Long_Float; Sc : out Geom.Scene_Pt_Vectors.Vector) is
      begin
         Sc.Clear;
         for I in 0 .. 15 loop
            for J in 0 .. 11 loop
               declare
                  Uq : constant Long_Float := 10.0 + 620.0 * Long_Float (I) / 15.0;
                  Vq : constant Long_Float := 10.0 + 460.0 * Long_Float (J) / 11.0;
                  Ok, Okh : Boolean;
                  D : constant Geom.V3 := Geom.Ray_Fixed (Gt, Uq, Vq, Ok);
                  Z0 : constant Long_Float := 0.8 + 0.04 * Sin (Long_Float (I + 3 * J));
                  Pw : constant Geom.V3 := (if Ok then Geom.Hit_Plane (Gt.Pos, D, [0.0, 0.0, Z0], [0.0, 0.0, 1.0], Okh) else Gt.Pos);
                  U, V : Long_Float;
                  Fr : Boolean;
               begin
                  if Ok and then Okh then
                     Geom.Project_Fixed (Gt, Pw, U, V, Fr);
                     if Fr and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0 then
                        Sc.Append (Geom.Scene_Pt'(Pw => Pw, Cov => [others => [others => 0.0]], U => U + Sh * Jit, V => V + Sh * Jit, Sh => Sh, Views => 3));
                     end if;
                  end if;
               end;
            end loop;
         end loop;
      end Board;
      Sc_D, Sc_P : Geom.Scene_Pt_Vectors.Vector;
      Gd, Gp : Geom.Cam_Geo;
      Rd, Rp : Geom.Fixed_Report;
      Okd, Okp : Boolean;
      Gtd : Geom.Cam_Geo;
   begin
      Gt0.F := 288.0; Gt0.Cx := 320.0; Gt0.Cy := 240.0; Gt0.R_Ce := Geom.Rodrigues ([0.55, 0.12, 0.05]); Gt0.Pos := [0.05, -0.65, 1.75];
      Gt0.Fixed := True; Gt0.Valid := True;
      Gtd := Gt0; Gtd.K1 := -0.15; Gtd.K2 := 0.03;
      Board (Gtd, 0.3, Sc_D);
      Board (Gt0, 0.3, Sc_P);
      Gd.F := 0.0; Gd.Cx := 320.0; Gd.Cy := 240.0;
      Gp := Gd;
      Geom.Fit_Fixed_Board (Gd, Sc_D, Rd, Okd);
      Geom.Fit_Fixed_Board (Gp, Sc_P, Rp, Okp);
      declare
         Ed : constant Long_Float := Geom.Norm ([Gd.Pos (0) - Gtd.Pos (0), Gd.Pos (1) - Gtd.Pos (1), Gd.Pos (2) - Gtd.Pos (2)]);
         Ep : constant Long_Float := Geom.Norm ([Gp.Pos (0) - Gt0.Pos (0), Gp.Pos (1) - Gt0.Pos (1), Gp.Pos (2) - Gt0.Pos (2)]);
      begin
         Check (Okd and then Rd.K_Kept and then abs (Gd.K1 - Gtd.K1) <= Stats.Z * Gd.K1_Sd and then abs (Gd.K2 - Gtd.K2) <= Stats.Z * Rd.K2_Sd
                and then abs (Gd.F - Gtd.F) <= Stats.Z * Gd.F_Sd and then Ed <= Stats.Z * Gd.Pos_Sd
                and then Okp and then not Rp.K_Kept and then Gp.K1 = 0.0 and then Gp.K2 = 0.0 and then abs (Gp.F - Gt0.F) <= Stats.Z * Gp.F_Sd
                and then Ep <= Stats.Z * Gp.Pos_Sd,
                "不动的眼解镜头畸变:带畸变的板(" & Codec.Img (Natural (Sc_D.Length)) & " 点)⇒ 畸变进解 " & Boolean'Image (Rd.K_Kept) & "、K1 "
                & Codec.Fmt (Gd.K1, 4) & " ± " & Codec.Fmt (Gd.K1_Sd, 4) & "(真 −0.15)、K2 " & Codec.Fmt (Gd.K2, 4) & " ± " & Codec.Fmt (Rd.K2_Sd, 4)
                & "(真 0.03)、焦距 " & Codec.Fmt (Gd.F, 2) & " ± " & Codec.Fmt (Gd.F_Sd, 2) & "(真 288)、位置差 " & Codec.Fmt (1000.0 * Ed, 2) & " mm ± "
                & Codec.Fmt (1000.0 * Gd.Pos_Sd, 2) & ";不带畸变的板 ⇒ 畸变进解 " & Boolean'Image (Rp.K_Kept) & "、焦距 " & Codec.Fmt (Gp.F, 2) & " ± "
                & Codec.Fmt (Gp.F_Sd, 2) & "、位置差 " & Codec.Fmt (1000.0 * Ep, 2) & " mm");
      end;
   end;
end Welds_Path_3;
