separate (Jointboot)
procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                 Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                 Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "";
                 Pin_Fixed_F : Long_Float := 0.0;
                 Eyes : Ints := Int_Vectors.Empty_Vector; World_Cam : Integer := -1; N_Cams : Natural := 0) is
   use Geom;
   Max_Pts : constant := Gx * Gy;   --  每只手最多三角几个点(同扫描格点数,次数)
   T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   Na : constant Natural := Natural (Worlds.Length);
   Ps : array (0 .. Natural'Max (1, Na) - 1) of Tri_Vectors.Vector;
   Pls, Nrs : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
   Gates : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
   Sig_Px : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];   --  每只手自己配点的噪声(像素,Tri_Pts 量的;给它的三角点定不确定度)
   Placed : array (0 .. Natural'Max (1, Na) - 1) of Boolean := [others => False];
   Pl0, N0 : V3 := [0.0, 0.0, 1.0];  --  世界的桌面(世界那只手系里)
   Pl0_Md : Long_Float := 0.0;       --  世界那只手三角出、拟合进桌面的点离面的中位
   --  世界那只手(10-01:原来写死第 0 只,它扫坏了整个开机就退出 ⇒ 能定世界的第一只,World_Arm)
   Ref : Natural := 0;
   --  世界里的眼
   type World_View is record
      Id : Integer := -1;
      Cam : Cam_Geo;                   --  世界系里的相机(R_Ce = 相机 → 世界,Pos)
      Arm : Integer := -1;             --  哪只手的哪一格(-1 = 不长在手上的眼)
      Frame : Natural := 0;
      W, H : Natural := 0;
   end record;
   package World_View_Vectors is new Ada.Containers.Vectors (Natural, World_View);
   Wv : World_View_Vectors.Vector;
   --  不长在手上的眼(这具身体有几只就几只;V1b 这一版的扫描只存了一只):扫描起点那一刻它的画面,存在每只扫过的手的 Ds 里(Sweep_All),哪只都一样
   function Fx_Of return Natural is
   begin
      for A in 0 .. Natural (Ds.Length) - 1 loop
         if Ds (A).World_Id >= 0 then
            return A;
         end if;
      end loop;
      return 0;
   end Fx_Of;
   Fx_D : constant Natural := Fx_Of;
   Fx_Id : constant Integer := (if Natural (Ds.Length) > Fx_D then Ds (Fx_D).World_Id else -1);
   Fx_W : constant Natural := (if Natural (Ds.Length) > Fx_D then Ds (Fx_D).World_Img.W else 0);
   Fx_H : constant Natural := (if Natural (Ds.Length) > Fx_D then Ds (Fx_D).World_Img.H else 0);
   Fx_Placed : Boolean := False;
   G : Cam_Geo := No_Geo;             --  它(世界那只手系里)
   G_Rep : Fixed_Report;
   --  整体特征(按仪器编号)
   D_Ids : Ints;
   D_Vec : Instrument.Vec_Vectors.Vector;
   function Desc_Dot (Ia, Ib : Integer) return Long_Float is
      Ka : constant Integer := D_Ids.Find_Index (Ia);
      Kb : constant Integer := D_Ids.Find_Index (Ib);
      S : Long_Float := 0.0;
   begin
      if Ka < 0 or else Kb < 0 then
         return -1.0;
      end if;
      for K in 0 .. Natural'Min (Natural (D_Vec (Ka).Length), Natural (D_Vec (Kb).Length)) - 1 loop
         S := S + D_Vec (Ka) (K) * D_Vec (Kb) (K);
      end loop;
      return S;
   end Desc_Dot;
   --  试过的对(按仪器编号)
   type Pair is record
      A, B : Integer := -1;
   end record;
   package Pair_Vectors is new Ada.Containers.Vectors (Natural, Pair);
   Tried : Pair_Vectors.Vector;
   --  攒下来的共同看见的点
   type Cam_Ob is record
      Pa, Pk : Natural := 0;          --  哪只手的第几个三角点
      Wk : Natural := 0;              --  它是在世界里第几只眼(那只手的哪一格)里配过来的
      Uw, Vw : Long_Float := 0.0;     --  在那一格里的像素
      Xw : V3;
      Cw : M3;
      U, V, E : Long_Float := 0.0;
   end record;
   package Cam_Ob_Vectors is new Ada.Containers.Vectors (Natural, Cam_Ob);
   Cam_Obs : Cam_Ob_Vectors.Vector;
   type Arm_Ob is record
      K : Natural := 0;                       --  它的第几个三角点
      Bf : Natural := 0;                      --  它的第几格(配点那一格)
      Wk : Natural := 0;                      --  世界里的第几只眼(Wv 里的下标)
      Ro, Rd : V3;                            --  世界里那只眼过配上的那个像素的视线(起点、单位方向)
      U, V, Uw, Vw, E : Long_Float := 0.0;   --  世界里那只眼里的像素、这只手那一格里的像素、往返差
   end record;
   package Arm_Ob_Vectors is new Ada.Containers.Vectors (Natural, Arm_Ob);
   Arm_Obs : array (0 .. Natural'Max (1, Na) - 1) of Arm_Ob_Vectors.Vector;
   --  一只手和世界里的眼之间那批配点的噪声 = 它们往返差的中位(像素;同不动的眼那边)—— 不用它自己扫描配点的噪声:
   --  两只手之间视角差得远,配点真的误差是 1.5–3 px,自己扫描的只有 0.1 px(V1B12 回放:按 0.1 px 算,网格分不出好坏,起步落错)
   function Cross_Sig (B : Natural) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      Es : Floats;
   begin
      for X of Arm_Obs (B) loop
         Es.Append (X.E);
      end loop;
      if Es.Is_Empty then
         return 1.0;
      end if;
      Sorting.Sort (Es);
      return Long_Float'Max (Es (Natural (Es.Length) / 2), 1.0e-6);   --  数值保护(无量纲)
   end Cross_Sig;
   N_Pairs_Of : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
   --  这一轮给每只候选的手算出来的放法(真放进世界时直接用,证据没变,不再算一遍)
   Cand_S, Cand_Md : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];
   Cand_R : array (0 .. Natural'Max (1, Na) - 1) of M3 := [others => Identity];
   Cand_T : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 0.0]];
   Cand_Inl : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
   N_Pairs_Fx : Natural := 0;
   Round_Note : Unbounded_String;
   --  第 A 只手第 F 格里看得见的三角点:下标和像素 = 这只手的全部三角点按运动学投进这一格,在眼前面、落在画面里的
   --  (09-27 V1B15:原来只给"在这一格三角出来的"那 ≤ 40 个,第 2 只手 22 格去配头顶眼只有 1 格配上 18 个点,两只手对齐差 11 mm;
   --  离线拿整片 768 个格点去问,40 / 44 格每格往返 1 px 内配上 100–250 个 —— 缺的是问的点,不是配不上)
   package M3_Vectors is new Ada.Containers.Vectors (Natural, M3);
   Fk_R : array (0 .. Natural'Max (1, Na) - 1) of M3_Vectors.Vector;   --  每只手每一格的腕眼位姿(参照眼系里;按读数算一次存着)
   Fk_T : array (0 .. Natural'Max (1, Na) - 1) of V3_Vectors.Vector;
   procedure Pts_In (A, F : Natural; Idx : out Ints; Q : out Instrument.Match_Vectors.Vector) is
      M : Kinem.Model renames Worlds (A).Model;
   begin
      Idx.Clear; Q.Clear;
      if Fk_R (A).Is_Empty then
         for Fr of Ds (A).Frames loop
            declare
               Rr : M3;
               Tt : V3;
            begin
               Kinem.FK (M, Fr.Q, Rr, Tt);
               Fk_R (A).Append (Rr); Fk_T (A).Append (Tt);
            end;
         end loop;
      end if;
      if F >= Natural (Fk_R (A).Length) then
         return;
      end if;
      for K in 0 .. Natural (Ps (A).Length) - 1 loop
         declare
            X : constant V3 := Ps (A) (K).X;
            Xc : constant V3 := Ap (Tr (Fk_R (A) (F)), [X (0) - Fk_T (A) (F) (0), X (1) - Fk_T (A) (F) (1), X (2) - Fk_T (A) (F) (2)]);
         begin
            if Xc (2) < 0.0 then   --  在眼前面(-z 朝前,同 Dir_Of)
               declare
                  U : constant Long_Float := M.Cx + M.F * Xc (0) / (-Xc (2));
                  V : constant Long_Float := M.Cy - M.F * Xc (1) / (-Xc (2));
               begin
                  if U >= 0.0 and then U < Long_Float (Ds (A).W) and then V >= 0.0 and then V < Long_Float (Ds (A).H) then
                     Idx.Append (K); Q.Append (Instrument.Match_Pt'(U => U, V => V, others => <>));
                  end if;
               end;
            end if;
         end;
      end loop;
   end Pts_In;
   --  第 A 只手的格子里用得上的(有三角点的):起点 + 三角用到的每一格
   function Frames_Of (A : Natural) return Ints is
      Fr : Ints;
   begin
      Fr.Append (0);
      for Q of Ps (A) loop
         if not Fr.Contains (Q.Fr) then
            Fr.Append (Q.Fr);
         end if;
      end loop;
      return Fr;
   end Frames_Of;
   function Id_Of (A, F : Natural) return Integer is (if F < Natural (Ds (A).Ids.Length) then Ds (A).Ids (F) else -1);
   --  第 A 只手第 F 格的眼放进世界(A 已放进世界:X_世界 = S · Ra · X + Ta)
   function Cam_Of (A, F : Natural) return Cam_Geo is
      Cg : Cam_Geo := No_Geo;
      Rr : M3;
      Tt : V3;
      W : constant Arm_World := Worlds (A);
      Rt : V3;
   begin
      Kinem.FK (W.Model, Ds (A).Frames (F).Q, Rr, Tt);
      Rt := Ap (W.Ra, Tt);
      Cg.R_Ce := Mul (W.Ra, Rr);
      Cg.Pos := [W.S * Rt (0) + W.Ta (0), W.S * Rt (1) + W.Ta (1), W.S * Rt (2) + W.Ta (2)];
      Cg.F := W.Model.F; Cg.Cx := W.Model.Cx; Cg.Cy := W.Model.Cy;
      Cg.Fixed := True; Cg.Valid := True;
      return Cg;
   end Cam_Of;
   function To_World (A : Natural; X : V3) return V3 is
      W : constant Arm_World := Worlds (A);
      Rt : constant V3 := Ap (W.Ra, X);
   begin
      return [W.S * Rt (0) + W.Ta (0), W.S * Rt (1) + W.Ta (1), W.S * Rt (2) + W.Ta (2)];
   end To_World;
   function Cov_World (A : Natural; C : M3) return M3 is
      W : constant Arm_World := Worlds (A);
      Rc : constant M3 := Mul (Mul (W.Ra, C), Tr (W.Ra));
      Out_C : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Out_C (I, J) := W.S * W.S * Rc (I, J);
         end loop;
      end loop;
      return Out_C;
   end Cov_World;
   --  配一对(Src 里的点 Q ⇒ Dst 那张图),往返 1 px 内的留下:返回每个留下的点在 Q 里是第几个、在 Dst 里的像素、往返差
   T_Match : Duration := 0.0;   --  对齐里花在配点仪器上的时间(秒;只记账)
   N_Match : Natural := 0;
   --  Dst_Arm ≥ 0:Dst 是这只手某一格的腕眼 ⇒ 落在它长在眼上的像素那一格的不要(被自己的手挡着,不是这个点;Kinem.On_Eye_Grid)
   procedure Match_Pair (Src, Dst : Integer; Dw, Dh : Natural; Q : Instrument.Match_Vectors.Vector; Keep : out Ints; U, V, E : out Floats;
                         Dst_Arm : Integer := -1) is
      Err : Unbounded_String;
      Tm : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      Keep.Clear; U.Clear; V.Clear; E.Clear;
      Tried.Append (Pair'(A => Src, B => Dst));
      N_Match := N_Match + 1;
      if Q.Is_Empty or else Src < 0 or else Dst < 0 then
         return;
      end if;
      declare
         R : constant Instrument.Match_Vectors.Vector := Instrument.Match_Ids (Host, Port, Natural (Src), Natural (Dst), Q, Err, Coarse => True, Back => True);
      begin
         if Natural (R.Length) = Natural (Q.Length) then
            for I in 0 .. Natural (Q.Length) - 1 loop
               if R (I).Bu >= 0.0 and then R (I).U >= 0.0 and then R (I).U < Long_Float (Dw) and then R (I).V >= 0.0 and then R (I).V < Long_Float (Dh) then
                  declare
                     Ei : constant Long_Float := Norm ([R (I).Bu - Q (I).U, R (I).Bv - Q (I).V, 0.0]);
                  begin
                     if Ei < Trip_Px and then not (Dst_Arm >= 0 and then Dw > 0 and then Dh > 0
                                                   and then Kinem.On_Eye_Grid (Worlds (Natural (Dst_Arm)).Model.Eye, R (I).U, R (I).V, Dw, Dh))
                     then
                        Keep.Append (I); U.Append (R (I).U); V.Append (R (I).V); E.Append (Ei);
                     end if;
                  end;
               end if;
            end loop;
         end if;
      end;
      T_Match := T_Match + Ada.Calendar."-" (Ada.Calendar.Clock, Tm);
   end Match_Pair;
   function Was_Tried (A, B : Integer) return Boolean is (Tried.Contains (Pair'(A => A, B => B)));
   --  到这一步用了多少秒、其中配点仪器多少(只记账)
   function Lap return String is
     ("(到这儿 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒,其中配点 " & Codec.Img (N_Match) & " 对 "
      & Codec.Fmt (Long_Float (T_Match), 1) & " 秒)");
   function Dist (X, Pl, Nrm : V3) return Long_Float is ((X (0) - Pl (0)) * Nrm (0) + (X (1) - Pl (1)) * Nrm (1) + (X (2) - Pl (2)) * Nrm (2));
   function Cross (A, B : V3) return V3 is ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
   procedure Plane_Of (P : Tri_Vectors.Vector; Pl, Nrm : out V3; Gate : out Long_Float; Inl : out Natural; Md : out Long_Float) is
      X : Kinem.V3_Array (0 .. Natural'Max (1, Natural (P.Length)) - 1);
   begin
      for K in 0 .. Natural (P.Length) - 1 loop
         X (K) := P (K).X;
      end loop;
      Kinem.Robust_Plane (X (0 .. Natural (P.Length) - 1), Pl, Nrm, Inl, Md);
      Gate := 2.5 * 1.4826 * Md;   --  同 Robust_Plane 挑内点(统计常数,无量纲)
   end Plane_Of;
   --  把第 A 只手(已放进世界)有三角点的每一格加成世界里的眼
   procedure Add_Arm_Views (A : Natural) is
   begin
      for F of Frames_Of (A) loop
         if Id_Of (A, Natural (F)) >= 0 then
            Wv.Append (World_View'(Id => Id_Of (A, Natural (F)), Cam => Cam_Of (A, Natural (F)), Arm => A, Frame => Natural (F), W => Ds (A).W, H => Ds (A).H));
         end if;
      end loop;
   end Add_Arm_Views;
   --  按攒下的点试解:不动的眼(内点数)/ 第 B 只手(相似变换、内点数、残差中位)
   --  放这只眼只拿世界那只手(Ref)自己三角出的点:它们在世界里的位置就是它自己系里的位置,不带任何"放进世界"的误差。
   --  别的手的点在世界里的位置带着那只手放进世界时的误差,而这个误差不在它们的协方差里(Cov_World 只有三角的那一份)——
   --  拿它们单独定一只新眼,等于把一个没算进不确定度的错当成量准了的点。它们照样记进 Cam_Obs:一起精修里那只手的放法也是未知数,
   --  在那里它们既修这只眼、也修那只手。
   --  (V1B69 2026-09-30:第 2 只手先放进世界,沿两手连线偏了 0.7 单位 ≈ 3.9 cm —— 两只腕眼共同看见的 97% 是远处的墙,墙几乎是一个面,
   --  "沿连线挪一点、同时转一点"分不出来,只有桌面上那 17 个近点分得出,又被当野点挑掉了;V1B68 的扫描拿掉头顶眼重放一样偏 5.4%。
   --  第二轮把它的 1512 个点和第 0 只手的 1117 个一起拿来解头顶眼,两拨在头顶眼里差 14 px,门外的过半 ⇒ 照实拒绝 ⇒ 头顶眼没放进世界、
   --  没有板、没法碰桌面量指尖。只拿第 0 只手的 1117 个:进解 959 个、残差 0.27 px,第 2 只手的点再在一起精修里把它拉回来)
   procedure Fit_Cam (Gt : out Cam_Geo; Rp : out Fixed_Report; Inl : out Natural) is
      Scene : Scene_Pt_Vectors.Vector;
      Sh : Long_Float := 0.0;
      Okf : Boolean;
      N_Ref : Natural := 0;
   begin
      Gt := No_Geo; Rp := (others => <>); Inl := 0;
      for X of Cam_Obs loop
         if X.Pa = Ref then
            N_Ref := N_Ref + 1;
         end if;
      end loop;
      if N_Ref < Min_Inl then
         return;
      end if;
      declare
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : Floats;
      begin
         for X of Cam_Obs loop
            if X.Pa = Ref then
               Es.Append (X.E);
            end if;
         end loop;
         Sorting.Sort (Es);
         Sh := Es (Natural (Es.Length) / 2);   --  这只眼里配点的噪声 = 这一次往返差的中位(像素)
      end;
      for X of Cam_Obs loop
         if X.Pa = Ref then
            Scene.Append (Scene_Pt'(Pw => X.Xw, Cov => X.Cw, U => X.U, V => X.V, Sh => Sh, Views => 2));
         end if;
      end loop;
      Gt.Cx := Long_Float (Fx_W) / 2.0; Gt.Cy := Long_Float (Fx_H) / 2.0; Gt.F := Pin_Fixed_F;   --  焦距一起解(Pin_Fixed_F = 0;离线对照实验才钉)
      Fit_Fixed_Board (Gt, Scene, Rp, Okf);
      Inl := (if Okf then Rp.Scene_Used else 0);
   end Fit_Cam;
   --  每只手的桌面当一个约束用:法向(朝它的眼)、面上点的中心;不确定度 = 两张面各自拟合出来的标准误差按平方和 ——
   --  离面的散布(1.4826 × 中位,统计常数)÷ √面上的点数:高度直接是它,倾角再除以面铺开的大小(V1B12 回放:拿"算不算在面上的门"当不确定度,
   --  那是桌面的厚度不是它位置的精度,大了几十倍,长度倍数差 2.9%)。只算 3 条残差(两个倾角 + 一个高度)—— 不按桌面上每个点各算一条:
   --  那样几百条压过几十条像素残差,两只手各自量的桌面之间本来就有几毫米的差,解会被拽到"桌面对得最齐"的错解上
   --  (V1B11 回放 2026-09-26:按点算时倍数 0.98、转错 12°、平移错 583 mm)
   Pl_Cb, Pl_Nb : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
   Pl_Sn, Pl_Sd : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];   --  倾角(弧度)、高度(世界单位)的不确定度
   --  第 A 只手自己那张桌面上的点:中心、铺开的大小(各点离中心的均方根)、高度的标准误差(离面散布 ÷ √点数;Gates = 2.5 × 1.4826 × 中位
   --  ⇒ ÷ 2.5 = 离面散布,统计常数)、面上几个点
   procedure Plane_Stats (A : Natural; Pl, Nrm : V3; Ctr : out V3; Ext, Se : out Long_Float; N : out Natural) is
   begin
      Ctr := [0.0, 0.0, 0.0]; Ext := 0.0; Se := 0.0; N := 0;
      for Q of Ps (A) loop
         if abs Dist (Q.X, Pl, Nrm) <= Gates (A) then
            Ctr := [Ctr (0) + Q.X (0), Ctr (1) + Q.X (1), Ctr (2) + Q.X (2)]; N := N + 1;
         end if;
      end loop;
      if N > 0 then
         Ctr := [Ctr (0) / Long_Float (N), Ctr (1) / Long_Float (N), Ctr (2) / Long_Float (N)];
         for Q of Ps (A) loop
            if abs Dist (Q.X, Pl, Nrm) <= Gates (A) then
               Ext := Ext + Norm ([Q.X (0) - Ctr (0), Q.X (1) - Ctr (1), Q.X (2) - Ctr (2)]) ** 2;
            end if;
         end loop;
         Ext := Sqrt (Ext / Long_Float (N));
      end if;
      Se := Gates (A) / 2.5 / Sqrt (Long_Float (Natural'Max (1, N)));   --  统计常数(见上)
   end Plane_Stats;
   procedure Plane_Info (B : Natural) is
      Ext_B, Ext_0, Se_B, Se_0 : Long_Float;
      C0, Cb : V3;
      N_0, N_B : Natural;
   begin
      Plane_Stats (B, Pls (B), Nrs (B), Cb, Ext_B, Se_B, N_B);
      Plane_Stats (Ref, Pl0, N0, C0, Ext_0, Se_0, N_0);
      Pl_Cb (B) := Cb;
      Pl_Nb (B) := (if Dist ([0.0, 0.0, 0.0], Pls (B), Nrs (B)) < 0.0 then [-Nrs (B) (0), -Nrs (B) (1), -Nrs (B) (2)] else Nrs (B));
      Pl_Sn (B) := Sqrt ((Se_B / Long_Float'Max (Ext_B, 1.0e-12)) ** 2 + (Se_0 / Long_Float'Max (Ext_0, 1.0e-12)) ** 2);   --  数值保护(无量纲)
      Pl_Sd (B) := Sqrt (Se_0 ** 2 + Se_B ** 2);   --  第 B 只手那份按它自己的单位,和世界单位差一个倍数(约 1,放进世界之前不知道)
   end Plane_Info;
   --  那 3 条残差本身:包级的 Tie_Res(Place_Hand、一起精修、Hand_Sd 同一份)

   --  第 B 只手的配点(它的点、配上它的世界里那只眼此刻的位姿)和它的桌面,交给 Place_Hand / Hand_Sd
   function Hand_Obs_Of (B : Natural) return Hand_Ob_Vectors.Vector is
      Out_V : Hand_Ob_Vectors.Vector;
   begin
      for Ob of Arm_Obs (B) loop
         Out_V.Append (Hand_Ob'(X => Ps (B) (Ob.K).X, Cov => Ps (B) (Ob.K).Cov, Cam => Wv (Ob.Wk).Cam, U => Ob.U, V => Ob.V,
                                W => Wv (Ob.Wk).W, H => Wv (Ob.Wk).H, Grp => (if Wv (Ob.Wk).Arm < 0 then 1 else 0), Rd => Ob.Rd));
      end loop;
      return Out_V;
   end Hand_Obs_Of;
   function Tie_Of (B : Natural) return Plane_Tie is
     (Plane_Tie'(Cb => Pl_Cb (B), Nb => Pl_Nb (B), Sn => Pl_Sn (B), Sd => Pl_Sd (B), P0 => Pl0, N0 => N0));
   Cand_P : array (0 .. Natural'Max (1, Na) - 1) of Hand_Place;   --  每只手最后一次 Place_Hand 的账(噪声、出坑几次、真的有多不准)
   --  每组配点噪声印成 "a / b"(日志)
   function Sm_Img (Sm : Floats) return String is
      R : Unbounded_String;
   begin
      for I in 0 .. Natural (Sm.Length) - 1 loop
         Append (R, (if I > 0 then " / " else "") & Codec.Fmt (Sm (I), 3));
      end loop;
      return To_String (R);
   end Sm_Img;
   --  第 B 只手自己的眼在它干活的地方量一个点有多不准:它的桌面上那些三角点的远近标准差(自报的 × 量到的放大 K),换成世界单位(× 它的长度倍数 Sx),取中位
   function Eye_Sd_Of (B : Natural; K, Sx : Long_Float) return Long_Float is
      Ds_V : Floats;
   begin
      for Q of Ps (B) loop
         if abs Dist (Q.X, Pls (B), Nrs (B)) <= Gates (B) then
            declare
               Nx : constant Long_Float := Norm (Q.X);
               Sd2 : Long_Float := 0.0;
            begin
               if Nx > 0.0 then
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Sd2 := Sd2 + Q.X (I) * Q.Cov (I, J) * Q.X (J) / (Nx * Nx);
                     end loop;
                  end loop;
                  Ds_V.Append (Sx * K * Sqrt (Sd2));
               end if;
            end;
         end if;
      end loop;
      return Median_Of (Ds_V);
   end Eye_Sd_Of;

   --  第 B 只手放进世界:求 X_世界 = S · R · X + T,让 ① 它每个配上的点(它自己三角出的,桌面上的、东西上的都算)投回世界里配上它的那只眼,
   --  落在配上的那个像素上(像素残差);② 它的桌面和世界的桌面重合(上面那 3 条)。
   --  起步:两张桌面的法向对齐,剩下"绕法向转多少"× 长度倍数铺网格,每一格的平移按视线的线性最小二乘直接解,挑每组中位对数加起来最小的;
   --  再交给 Place_Hand:按量到的方差分量精修、出坑、算真的有多不准(10-01)。Inl = 门里的配点数,Md = 它们的白化残差中位
   N_Theta : constant := 360;   --  绕法向每 1° 一档(次数)
   N_Scale : constant := 41;    --  长度倍数 0.1–10 按对数分 41 档(每档约 12%;倍数范围是比例,无量纲)
   Coarse_T : constant := 5;    --  网格先粗找:转角 5 档并 1 档(5°)、倍数 2 档并 1 档(约 26%),再在最好那格周围按细档补(次数)
   Coarse_S : constant := 2;
   procedure Place_Arm (B : Natural; S : out Long_Float; R : out M3; T : out V3; Inl : out Natural; Md : out Long_Float) is
      Nr : constant Natural := Natural (Arm_Obs (B).Length);
      Sig_B : constant Long_Float := Cross_Sig (B);
      --  配点分两组(噪声各自量):0 = 配进手的格子,1 = 配进不长在手上的眼
      function Grp (Ob : Arm_Ob) return Natural is (if Wv (Ob.Wk).Arm < 0 then 1 else 0);
      R0 : M3 := Identity;
      function Rz (Th : Long_Float) return M3 is (Rodrigues ([N0 (0) * Th, N0 (1) * Th, N0 (2) * Th]));
      --  网格只给精修找起点 ⇒ 网格的分数每组按顺序等间隔最多取 Sub_Max 条(次数);每一格的平移用全部
      --  (平移按普通最小二乘解,子集里错配那组占的份量变大,09-27 V1B15 回放起点从倍数 1.0060 偏到 1.0394)。
      --  精修(Place_Hand)和一起精修(Joint_Refine)用全部 —— 近处那几十条只占 2%,抽子集就把它们抽没了(10-01 V1B68 / V1B69)
      Sub_Max : constant := 400;
      In_Sub : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Nr));
      --  给定倍数、转动 ⇒ 平移:沿世界法向那一分量由"它的桌面落在世界的桌面上"定死(桌面是几百个点拟合的,高度的标准误差很小;
      --  精修时再按标准误差放软,见 Tie_Res),平面里那两个分量按视线的线性最小二乘(点到视线的距离)。
      --  (09-27 V1B15 回放:配进头顶眼的点来自同一格、大多在桌面上,只按视线解时"放大再挪远"投回头顶眼一样,网格挑到倍数 1.2952)
      function Solve_T (Sx : Long_Float; Rx : M3) return V3 is
         A : M3 := [others => [others => 0.0]];
         Bb : V3 := [0.0, 0.0, 0.0];
         Rc : constant V3 := Ap (Rx, Pl_Cb (B));
         H : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2) - Sx * (Rc (0) * N0 (0) + Rc (1) * N0 (1) + Rc (2) * N0 (2));
         E1 : V3 := Cross (N0, (if abs N0 (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  垂直于世界法向的一对方向(纯数学,无量纲)
         E2 : V3;
      begin
         E1 := [E1 (0) / Norm (E1), E1 (1) / Norm (E1), E1 (2) / Norm (E1)];
         E2 := Cross (N0, E1);
         for Ob of Arm_Obs (B) loop
            declare
               D : constant V3 := Ob.Rd;
               Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
               Q : constant V3 := [Ob.Ro (0) - Sx * Rt (0), Ob.Ro (1) - Sx * Rt (1), Ob.Ro (2) - Sx * Rt (2)];
               Dq : constant Long_Float := D (0) * Q (0) + D (1) * Q (1) + D (2) * Q (2);
            begin
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     A (I, J) := A (I, J) + (if I = J then 1.0 else 0.0) - D (I) * D (J);
                  end loop;
                  Bb (I) := Bb (I) + Q (I) - D (I) * Dq;
               end loop;
            end;
         end loop;
         --  T = H · 法向 + a · E1 + b · E2:(a, b) 的 2×2 正规方程
         declare
            An : constant V3 := Ap (A, N0);
            Ae1 : constant V3 := Ap (A, E1);
            Ae2 : constant V3 := Ap (A, E2);
            R1 : constant Long_Float := (Bb (0) - H * An (0)) * E1 (0) + (Bb (1) - H * An (1)) * E1 (1) + (Bb (2) - H * An (2)) * E1 (2);
            R2 : constant Long_Float := (Bb (0) - H * An (0)) * E2 (0) + (Bb (1) - H * An (1)) * E2 (1) + (Bb (2) - H * An (2)) * E2 (2);
            M11 : constant Long_Float := E1 (0) * Ae1 (0) + E1 (1) * Ae1 (1) + E1 (2) * Ae1 (2);
            M12 : constant Long_Float := E1 (0) * Ae2 (0) + E1 (1) * Ae2 (1) + E1 (2) * Ae2 (2);
            M22 : constant Long_Float := E2 (0) * Ae2 (0) + E2 (1) * Ae2 (1) + E2 (2) * Ae2 (2);
            Dt : constant Long_Float := M11 * M22 - M12 * M12;
            Ya, Yb : Long_Float := 0.0;
         begin
            if abs Dt > 1.0e-18 then   --  数值保护(无量纲)
               Ya := (M22 * R1 - M12 * R2) / Dt; Yb := (M11 * R2 - M12 * R1) / Dt;
            end if;
            return [H * N0 (0) + Ya * E1 (0) + Yb * E2 (0), H * N0 (1) + Ya * E1 (1) + Yb * E2 (1), H * N0 (2) + Ya * E1 (2) + Yb * E2 (2)];
         end;
      end Solve_T;
      --  一条配点:它的点按(Sx, Rx, Tx)放进世界、投回世界里那只眼,白化残差两条(网格起步:配点噪声 = 往返差的中位,远近按它自报的)
      procedure Res2 (Ob : Arm_Ob; Cg : Cam_Geo; Sx : Long_Float; Rx : M3; Tx : V3; E1, E2 : out Long_Float) is
         Area : Long_Float;
      begin
         Whiten (Part_Of (Cg, Ps (B) (Ob.K).X, Ps (B) (Ob.K).Cov, Sx, Rx, Tx, Ob.U, Ob.V), Sig_B, 1.0, E1, E2, Area);
      end Res2;
      function Epi (Ob : Arm_Ob; Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
         E1, E2 : Long_Float;
      begin
         Res2 (Ob, Wv (Ob.Wk).Cam, Sx, Rx, Tx, E1, E2);
         return Sqrt (E1 * E1 + E2 * E2);
      end Epi;
      --  网格那一步的分数 = 每组(条数 × 这一组白化残差中位的对数)加起来 = 每组噪声各自不知道时的最大似然(同精修里按组重估噪声倍数:
      --  一组的噪声倍数按它自己的中位估,代回去似然只剩 −Σ 条数 · log 中位)。一组整体大小(噪声估大估小)只差一个常数,不影响排名;
      --  原来直接把各组中位相加(09-27 V1B15 回放:配进第 1 只手格子的 331 条 84% 是木纹上的错配,中位几百像素,随网格变一点就压过
      --  配进头顶眼那 192 条 0.4 px 的信号,放进世界时倍数解成 1.2957,靠一起精修才拉回 1.0066)
      function Score (Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : array (0 .. 1) of Floats;
         Sm : Long_Float := 0.0;
      begin
         for K in 0 .. Nr - 1 loop
            if In_Sub (K) then
               Es (Grp (Arm_Obs (B) (K))).Append (Epi (Arm_Obs (B) (K), Sx, Rx, Tx));
            end if;
         end loop;
         for G in Es'Range loop
            if not Es (G).Is_Empty then
               Sorting.Sort (Es (G));
               Sm := Sm + Long_Float (Es (G).Length) * Log (Long_Float'Max (Es (G) (Natural (Es (G).Length) / 2), 1.0e-9));   --  数值保护(无量纲)
            end if;
         end loop;
         return Sm;
      end Score;
   begin
      S := 1.0; R := Identity; T := [0.0, 0.0, 0.0]; Inl := 0; Md := 0.0;
      if Nr < Min_Inl then
         return;
      end if;
      --  它的桌面法向(朝它的眼)转到世界的法向上
      declare
         Nb : constant V3 := Pl_Nb (B);
         Ax : constant V3 := Cross (Nb, N0);
         Sn : constant Long_Float := Norm (Ax);
         Cs : constant Long_Float := Nb (0) * N0 (0) + Nb (1) * N0 (1) + Nb (2) * N0 (2);
      begin
         if Sn > 1.0e-12 then   --  数值保护(无量纲)
            R0 := Rodrigues ([Ax (0) / Sn * Arctan (Sn, Cs), Ax (1) / Sn * Arctan (Sn, Cs), Ax (2) / Sn * Arctan (Sn, Cs)]);
         elsif Cs < 0.0 then
            declare
               E1 : constant V3 := Cross (Nb, (if abs Nb (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  辅助方向(纯数学,无量纲)
               En : constant Long_Float := Norm (E1);
            begin
               R0 := Rodrigues ([E1 (0) / En * Ada.Numerics.Pi, E1 (1) / En * Ada.Numerics.Pi, E1 (2) / En * Ada.Numerics.Pi]);   --  反向:转半圈
            end;
         end if;
      end;
      --  网格起步
      declare
         N_Grp, Seen : array (0 .. 1) of Natural := [others => 0];
      begin
         for Ob of Arm_Obs (B) loop
            N_Grp (Grp (Ob)) := N_Grp (Grp (Ob)) + 1;
         end loop;
         for K in 0 .. Nr - 1 loop
            declare
               G : constant Natural := Grp (Arm_Obs (B) (K));
            begin
               In_Sub.Replace_Element (K, Seen (G) mod Natural'Max (1, (N_Grp (G) + Sub_Max - 1) / Sub_Max) = 0);
               Seen (G) := Seen (G) + 1;
            end;
         end loop;
      end;
      --  两级网格(09-27:一格要把几百条配点白化一遍,360 × 41 格放一次 8–20 秒):粗档挑出最好那格,再在它周围按细档补;
      --  最后落点的细度同原来(1°、约 12%)
      declare
         Best : Long_Float := Long_Float'Last;
         It_B, Ks_B : Integer := 0;
         procedure Try (It, Ks : Integer) is
            Itm : constant Integer := It mod N_Theta;
         begin
            if Ks < 0 or else Ks > N_Scale - 1 then
               return;
            end if;
            declare
               Rx : constant M3 := Mul (Rz (2.0 * Ada.Numerics.Pi * Long_Float (Itm) / Long_Float (N_Theta)), R0);
               Sx : constant Long_Float := 0.1 * Exp (Long_Float (Ks) / Long_Float (N_Scale - 1) * Log (100.0));   --  0.1–10(比例,无量纲)
               Tx : constant V3 := Solve_T (Sx, Rx);
               Sc : constant Long_Float := Score (Sx, Rx, Tx);
            begin
               if Sc < Best then
                  Best := Sc; S := Sx; R := Rx; T := Tx; It_B := Itm; Ks_B := Ks;
               end if;
            end;
         end Try;
      begin
         for It in 0 .. N_Theta / Coarse_T - 1 loop
            for Ks in 0 .. (N_Scale - 1) / Coarse_S loop
               Try (It * Coarse_T, Ks * Coarse_S);
            end loop;
         end loop;
         declare
            It_C : constant Integer := It_B;
            Ks_C : constant Integer := Ks_B;
         begin
            for Dt in -(Coarse_T - 1) .. Coarse_T - 1 loop
               for Dk in -(Coarse_S - 1) .. Coarse_S - 1 loop
                  if Dt /= 0 or else Dk /= 0 then
                     Try (It_C + Dt, Ks_C + Dk);
                  end if;
               end loop;
            end loop;
         end;
      end;
      --  精修、出坑、算真的有多不准(Place_Hand;全部配点)
      declare
         Obs : constant Hand_Ob_Vectors.Vector := Hand_Obs_Of (B);
         Pl : Hand_Place;
         Okp : Boolean;
      begin
         Pl.S := S; Pl.R := R; Pl.T := T;
         Place_Hand (Obs, Tie_Of (B), Sig_B, Pl, Okp);
         S := Pl.S; R := Pl.R; T := Pl.T; Inl := (if Okp then Pl.Inl else 0); Md := Pl.Md;
         Cand_P (B) := Pl;
         if Pl.Unsettled then
            Say ("  第" & Codec.Img (B + 1) & " 只手放进世界的精修:门里的那一组来回转 / 解满保险的遍数还在变 ⇒ 没定下来,交的是最后一遍的解");
         end if;
         if Pl.Lm_Capped then
            Say ("  第" & Codec.Img (B + 1) & " 只手放进世界的精修:有一遍的解做满保险的次数代价还在降 ⇒ 那一遍没解到底,照实交");
         end if;
      end;
   end Place_Arm;

   --  全部放完以后一起精修:每只放进世界的别的手(相似变换 7 个数)、不长在手上的那只眼(位姿 6 个 + 焦距)用全部配上的点一起按像素解
   --  (抗野点,门同上),加每只手"桌面重合"那 3 条。先放进世界的也被后放进的证据修正(按放进去的先后,前面的定下以后后面的就不再动它 ⇒ 错会被锁死)。
   --  噪声同 Place_Hand 按方差分量量(10-01):每组配点噪声按垂直对极线那一份、每只手三角点的远近放大按沿对极线那一份(那只手的点进的所有配点)
   --  —— 组:第 B 只手配进手的格子 = 2B、配进不长在手上的眼 = 2B + 1,不动的眼的配点 = 2 Na;一起精修完留在 Jr_Sm / Jr_K,放完算真的有多不准用
   Jr_Sm : Floats;
   Jr_K : Floats;
   procedure Joint_Refine is
      Arm_Slot : array (0 .. Natural'Max (1, Na) - 1) of Integer := [others => -1];
      Cam_Slot : Integer := -1;
      Np : Natural := 0;
      Nv : constant Natural := Natural (Wv.Length);
      Fr_R : array (0 .. Natural'Max (1, Nv) - 1) of M3 := [others => Identity];
      Fr_T : array (0 .. Natural'Max (1, Nv) - 1) of V3 := [others => [0.0, 0.0, 0.0]];
      procedure Map_Of (Xx : Kinem.Vec; A : Natural; Sx : out Long_Float; Rx : out M3; Tx : out V3) is
      begin
         if Arm_Slot (A) >= 0 then
            declare
               K : constant Natural := Xx'First + Natural (Arm_Slot (A));
            begin
               Rx := Rodrigues ([Xx (K), Xx (K + 1), Xx (K + 2)]); Tx := [Xx (K + 3), Xx (K + 4), Xx (K + 5)]; Sx := Exp (Xx (K + 6));
            end;
         else
            Sx := Worlds (A).S; Rx := Worlds (A).Ra; Tx := Worlds (A).Ta;
         end if;
      end Map_Of;
      function Cam_Now (Xx : Kinem.Vec; K : Natural) return Cam_Geo is
         Cg : Cam_Geo := Wv (K).Cam;
      begin
         if Wv (K).Arm >= 0 then
            declare
               Sx : Long_Float;
               Rx : M3;
               Tx, Rt : V3;
            begin
               Map_Of (Xx, Natural (Wv (K).Arm), Sx, Rx, Tx);
               Rt := Ap (Rx, Fr_T (K));
               Cg.R_Ce := Mul (Rx, Fr_R (K));
               Cg.Pos := [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)];
            end;
         elsif Cam_Slot >= 0 then
            declare
               J : constant Natural := Xx'First + Natural (Cam_Slot);
            begin
               Cg.R_Ce := Rodrigues ([Xx (J), Xx (J + 1), Xx (J + 2)]);
               Cg.Pos := [Xx (J + 3), Xx (J + 4), Xx (J + 5)];
               Cg.F := (if Pin_Fixed_F > 0.0 then Pin_Fixed_F else Exp (Xx (J + 6)));
            end;
         end if;
         return Cg;
      end Cam_Now;
      Fx_K : Integer := -1;   --  不动的眼在 Wv 里第几个
      Cam_Sig : Long_Float := 1.0e-6;   --  不动的眼里配点的噪声起步 = 这一批往返差的中位(像素;同解它时的 Sh)
      N_G : constant Natural := 2 * Na + 1;   --  配点的组数(结构:每只手两组 + 不动的眼一组)
      Sm_G : Floats := Filled (N_G, 1.0);     --  每组配点噪声(像素;第一轮之前按往返差的中位起步,见下)
      K_A : Floats := Filled (Natural'Max (1, Na), 1.0);   --  每只手三角点的远近放大(第一轮之前按自报的)
      function Arm_Grp (Ob : Arm_Ob; B : Natural) return Natural is (2 * B + (if Wv (Ob.Wk).Arm < 0 then 1 else 0));
      --  全部残差(门里的):手的配点(第 B 只手的点投回世界里那只眼)、不动的眼的配点(世界里的手的点投进它)、每只手桌面 3 条。
      --  Fill_E = 只量不填:Ea / Ec = 每条的 |w|²,Da / Dc = 乱配在白化单位里的密度(混合模型的门用),Ga = 每条手的配点在哪一组
      procedure All_Res (Xx : Kinem.Vec; Rr : out Kinem.Vec; Use_A : Bools; Use_C : Bools; Fill_E : Boolean; Ea, Ec : in out Floats; Ga : in out Ints;
                         Da, Dc : in out Floats) is
         J : Natural := Rr'First;
      begin
         Ea.Clear; Ec.Clear; Ga.Clear; Da.Clear; Dc.Clear;
         for B in 0 .. Na - 1 loop
            if Arm_Slot (B) >= 0 then
               declare
                  Sx : Long_Float;
                  Rx : M3;
                  Tx : V3;
               begin
                  Map_Of (Xx, B, Sx, Rx, Tx);
                  for K in 0 .. Natural (Arm_Obs (B).Length) - 1 loop
                     declare
                        Ob : constant Arm_Ob := Arm_Obs (B) (K);
                        E1, E2, Area : Long_Float;
                        Gi : constant Natural := Arm_Grp (Ob, B);
                     begin
                        Whiten (Part_Of (Cam_Now (Xx, Ob.Wk), Ps (B) (Ob.K).X, Ps (B) (Ob.K).Cov, Sx, Rx, Tx, Ob.U, Ob.V), Sm_G (Gi), K_A (B), E1, E2, Area);
                        if Fill_E then
                           Ga.Append (Gi);
                           Ea.Append (E1 * E1 + E2 * E2); Da.Append (Dens_Of (Area, Wv (Ob.Wk).W, Wv (Ob.Wk).H));
                        elsif Use_A (Natural (Ea.Length)) then
                           Rr (J) := E1; Rr (J + 1) := E2; J := J + 2;
                        end if;
                        if not Fill_E then
                           Ea.Append (0.0);
                        end if;
                     end;
                  end loop;
                  if not Fill_E then
                     Tie_Res (Tie_Of (B), Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2)); J := J + 3;
                  end if;
               end;
            end if;
         end loop;
         if Cam_Slot >= 0 and then Fx_K >= 0 then
            declare
               Cg : constant Cam_Geo := Cam_Now (Xx, Natural (Fx_K));
            begin
               for X of Cam_Obs loop
                  if X.Pa = Ref or else Arm_Slot (X.Pa) >= 0 then
                     declare
                        Sx : Long_Float;
                        Rx : M3;
                        Tx : V3;
                        E1, E2, Area : Long_Float;
                     begin
                        Map_Of (Xx, X.Pa, Sx, Rx, Tx);
                        Whiten (Part_Of (Cg, Ps (X.Pa) (X.Pk).X, Ps (X.Pa) (X.Pk).Cov, Sx, Rx, Tx, X.U, X.V), Sm_G (2 * Na), K_A (X.Pa), E1, E2, Area);
                        if Fill_E then
                           Ec.Append (E1 * E1 + E2 * E2); Dc.Append (Dens_Of (Area, Fx_W, Fx_H));
                        elsif Use_C (Natural (Ec.Length)) then
                           Rr (J) := E1; Rr (J + 1) := E2; J := J + 2;
                        end if;
                        if not Fill_E then
                           Ec.Append (0.0);
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end if;
      end All_Res;
      --  方差分量:在此刻的解上按上一轮门里的(第一轮全算)量 —— 每组的配点噪声按垂直对极线那一份,每只手的远近放大按它的点进的所有配点沿对极线那一份
      Prev_A, Prev_C : Bools;
      procedure Measure (Xx : Kinem.Vec) is
         Ep, Vp, Ea, Va, Vd : Floats;
         Gr, Ar : Ints;
         Us : Bools;
         Ia, Ic : Natural := 0;
         procedure Add (Pt : Ob_Part; G, A : Natural; U : Boolean) is
            E_A, E_P, V_A, V_P : Long_Float;
         begin
            Split (Pt, E_A, E_P, V_A, V_P);
            Ea.Append (E_A); Ep.Append (E_P); Va.Append (V_A); Vp.Append (V_P); Vd.Append (Pt.Vd);
            Gr.Append (G); Ar.Append (A); Us.Append (U and then Pt.Valid);
         end Add;
      begin
         for B in 0 .. Na - 1 loop
            if Arm_Slot (B) >= 0 then
               declare
                  Sx : Long_Float;
                  Rx : M3;
                  Tx : V3;
               begin
                  Map_Of (Xx, B, Sx, Rx, Tx);
                  for Ob of Arm_Obs (B) loop
                     Add (Part_Of (Cam_Now (Xx, Ob.Wk), Ps (B) (Ob.K).X, Ps (B) (Ob.K).Cov, Sx, Rx, Tx, Ob.U, Ob.V), Arm_Grp (Ob, B), B,
                          Ia >= Natural (Prev_A.Length) or else Prev_A (Ia));
                     Ia := Ia + 1;
                  end loop;
               end;
            end if;
         end loop;
         if Cam_Slot >= 0 and then Fx_K >= 0 then
            declare
               Cg : constant Cam_Geo := Cam_Now (Xx, Natural (Fx_K));
            begin
               for X of Cam_Obs loop
                  if X.Pa = Ref or else Arm_Slot (X.Pa) >= 0 then
                     declare
                        Sx : Long_Float;
                        Rx : M3;
                        Tx : V3;
                     begin
                        Map_Of (Xx, X.Pa, Sx, Rx, Tx);
                        Add (Part_Of (Cg, Ps (X.Pa) (X.Pk).X, Ps (X.Pa) (X.Pk).Cov, Sx, Rx, Tx, X.U, X.V), 2 * Na, X.Pa,
                             Ic >= Natural (Prev_C.Length) or else Prev_C (Ic));
                        Ic := Ic + 1;
                     end;
                  end if;
               end loop;
            end;
         end if;
         for G_I in 0 .. N_G - 1 loop
            declare
               R_V, V_V, One : Floats;
            begin
               for I in 0 .. Natural (Ep.Length) - 1 loop
                  if Us (I) and then Gr (I) = G_I then
                     R_V.Append (Ep (I)); V_V.Append (Vp (I)); One.Append (1.0);
                  end if;
               end loop;
               if not R_V.Is_Empty then
                  Sm_G.Replace_Element (G_I, Scale_Root (R_V, V_V, One));
               end if;
            end;
         end loop;
         for A in 0 .. Na - 1 loop
            declare
               R_V, V_V, D_V : Floats;
            begin
               for I in 0 .. Natural (Ea.Length) - 1 loop
                  if Us (I) and then Ar (I) = A and then Vd (I) > 0.0 then
                     R_V.Append (Ea (I)); V_V.Append (Sm_G (Natural (Gr (I))) ** 2 + Va (I)); D_V.Append (Vd (I));
                  end if;
               end loop;
               if not R_V.Is_Empty then
                  K_A.Replace_Element (A, Scale_Root (R_V, V_V, D_V));
               end if;
            end;
         end loop;
      end Measure;
   begin
      for B in 0 .. Na - 1 loop
         if B /= Ref then
            Sm_G.Replace_Element (2 * B, Cross_Sig (B)); Sm_G.Replace_Element (2 * B + 1, Cross_Sig (B));
         end if;
      end loop;
      declare
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : Floats;
      begin
         for X of Cam_Obs loop
            Es.Append (X.E);
         end loop;
         if not Es.Is_Empty then
            Sorting.Sort (Es);
            Cam_Sig := Long_Float'Max (Es (Natural (Es.Length) / 2), 1.0e-6);   --  数值保护(无量纲)
         end if;
         Sm_G.Replace_Element (2 * Na, Cam_Sig);
      end;
      for B in 0 .. Na - 1 loop
         if B /= Ref and then Placed (B) and then Worlds (B).Valid then
            Arm_Slot (B) := Np; Np := Np + 7;
         end if;
      end loop;
      for K in 0 .. Nv - 1 loop
         if Wv (K).Arm < 0 then
            Fx_K := K;
         else
            Kinem.FK (Worlds (Natural (Wv (K).Arm)).Model, Ds (Natural (Wv (K).Arm)).Frames (Wv (K).Frame).Q, Fr_R (K), Fr_T (K));
         end if;
      end loop;
      if Fx_Placed and then Fx_K >= 0 then
         Cam_Slot := Np; Np := Np + 7;
      end if;
      if Np = 0 then
         return;
      end if;
      declare
         X : Kinem.Vec (0 .. Np - 1);
         Steps : constant Kinem.Vec (0 .. Np - 1) := [others => 1.0e-7];   --  差分步(极小量,无量纲)
      begin
         for B in 0 .. Na - 1 loop
            if Arm_Slot (B) >= 0 then
               declare
                  K : constant Natural := Natural (Arm_Slot (B));
                  Rv : constant V3 := Rot_Vec (Worlds (B).Ra);
               begin
                  X (K .. K + 6) := [Rv (0), Rv (1), Rv (2), Worlds (B).Ta (0), Worlds (B).Ta (1), Worlds (B).Ta (2), Log (Worlds (B).S)];
               end;
            end if;
         end loop;
         if Cam_Slot >= 0 then
            declare
               K : constant Natural := Natural (Cam_Slot);
               Rv : constant V3 := Rot_Vec (G.R_Ce);
            begin
               X (K .. K + 6) := [Rv (0), Rv (1), Rv (2), G.Pos (0), G.Pos (1), G.Pos (2), Log (G.F)];
            end;
         end if;
         --  每轮先在此刻的解上量方差分量,再挑门里的、解,做到门里的那一组(手的配点 + 不动的眼的配点)不再变(Until_Settled)
         declare
            Ua, Uc : Bools;
            Nua, Nuc, N_Planes : Natural := 0;
            procedure Pick (Both : out Bools; Enough : out Boolean) is
               Ea, Ec, Da, Dc : Floats;
               Ga : Ints;
               Dummy : Kinem.Vec (0 .. 0);
            begin
               Measure (X);
               Ua.Clear; Uc.Clear; Nua := 0; Nuc := 0; N_Planes := 0;
               All_Res (X, Dummy, Bool_Vectors.Empty_Vector, Bool_Vectors.Empty_Vector, True, Ea, Ec, Ga, Da, Dc);
               declare
                  Ga_A, Ga_C : Long_Float;
               begin
                  Mix_Gate (Ea, Da, Ua, Ga_A);
                  Mix_Gate (Ec, Dc, Uc, Ga_C);
               end;
               for U of Ua loop
                  if U then
                     Nua := Nua + 1;
                  end if;
               end loop;
               for U of Uc loop
                  if U then
                     Nuc := Nuc + 1;
                  end if;
               end loop;
               for B in 0 .. Na - 1 loop
                  if Arm_Slot (B) >= 0 then
                     N_Planes := N_Planes + 1;
                  end if;
               end loop;
               Both := Bool_Vectors."&" (Ua, Uc);
               Prev_A := Ua; Prev_C := Uc;
               Enough := 2 * Nua + 2 * Nuc + 3 * N_Planes > Np;   --  残差条数(同 Solve 里的 N_Res)比参数多才解得了
            end Pick;
            Lm_Capped : Boolean := False;   --  哪一遍的 LM 做满保险的次数还在降
            procedure Solve (Both : Bools) is
               pragma Unreferenced (Both);   --  这一组 Pick 已经按手 / 不动的眼分开放在 Ua、Uc 里
            begin
               declare
                  N_Res : constant Natural := 2 * Nua + 2 * Nuc + 3 * N_Planes;
                  procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                     E1, E2, D1, D2 : Floats;
                     G2 : Ints;
                  begin
                     All_Res (Xx, Rr, Ua, Uc, False, E1, E2, G2, D1, D2);
                  end Resid;
                  Lm_Done : Boolean;   --  LM 收住了没有(False = 做满 100 次还在降)
               begin
                  if N_Res > Np then
                     Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access, Lm_Done);
                     if not Lm_Done then
                        Lm_Capped := True;
                     end if;
                  end if;
               end;
            end Solve;
            procedure Refine is new Until_Settled (Pick, Solve);
            Rounds : Natural;
            Verdict : Settle_Verdict;
         begin
            Refine (Rounds, Verdict);
            if Lm_Capped then
               Say ("  一起精修:有一遍的解做满保险的次数代价还在降 ⇒ 那一遍没解到底,照实交");
            end if;
            Say ("  一起精修:重挑重解了 " & Codec.Img (Rounds) & " 遍,门里的那一组"
                 & (case Verdict is
                       when Settled => "不再变了",
                       when Cycled => "又变回了前面某一遍的样子(来回转)⇒ 没定下来,交的是最后一遍的解",
                       when Capped => "解满保险的遍数还在变 ⇒ 没定下来,交的是最后一遍的解",
                       when Too_Few => "的残差条数不比要解的参数多 ⇒ 解不了"));
         end;
         Jr_Sm := Sm_G; Jr_K := K_A;
         --  写回去
         for B in 0 .. Na - 1 loop
            if Arm_Slot (B) >= 0 then
               declare
                  K : constant Natural := Natural (Arm_Slot (B));
                  Wb : Arm_World := Worlds (B);
               begin
                  Wb.Ra := Rodrigues ([X (K), X (K + 1), X (K + 2)]); Wb.Ta := [X (K + 3), X (K + 4), X (K + 5)]; Wb.S := Exp (X (K + 6));
                  Worlds.Replace_Element (B, Wb);
               end;
            end if;
         end loop;
         if Cam_Slot >= 0 then
            declare
               K : constant Natural := Natural (Cam_Slot);
            begin
               G.R_Ce := Rodrigues ([X (K), X (K + 1), X (K + 2)]); G.Pos := [X (K + 3), X (K + 4), X (K + 5)];
               G.F := (if Pin_Fixed_F > 0.0 then Pin_Fixed_F else Exp (X (K + 6)));
            end;
         end if;
         for K in 0 .. Nv - 1 loop
            declare
               Vw : World_View := Wv (K);
            begin
               Vw.Cam := Cam_Now (X, K);
               Wv.Replace_Element (K, Vw);
            end;
         end loop;
      end;
   end Joint_Refine;
begin
   Rw := Identity; O := [0.0, 0.0, 0.0]; Ok := False; Fixed_Eye := No_Geo;
   Board.Clear; Plane_Pt := [0.0, 0.0, 0.0]; Plane_N := [0.0, 0.0, 1.0]; Plane_Rms := 0.0;
   if Worlds.Is_Empty then
      return;
   end if;
   for A in 0 .. Na - 1 loop
      if Worlds (A).Valid then
         Tri_Pts (Worlds (A).Model, Ds (A), Css (A), Max_Pts, Ps (A), Sig_Px (A));
      end if;
   end loop;
   --  每只手自己的桌面;世界取能定世界的第一只(World_Arm;每只"上"量得多准照实说)
   declare
      Tilt : Floats;
      Inl_A : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
      Md_A : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
      Note : Unbounded_String;
   begin
      for A in 0 .. Na - 1 loop
         if Worlds (A).Valid and then Natural (Ps (A).Length) >= Min_Inl then
            declare
               Ctr : V3;
               Ext, Se : Long_Float;
               N_On : Natural;
            begin
               Plane_Of (Ps (A), Pls (A), Nrs (A), Gates (A), Inl_A (A), Md_A (A));
               Plane_Stats (A, Pls (A), Nrs (A), Ctr, Ext, Se, N_On);
               Tilt.Append (if Ext > 0.0 and then N_On >= Min_Inl then Se / Ext else -1.0);   --  -1 = 当不了世界(哨兵)
            end;
         else
            Tilt.Append (-1.0);   --  当不了世界(哨兵)
         end if;
         Append (Note, (if A > 0 then " · " else "") & "第" & Codec.Img (A + 1) & " 只手 "
                 & (if not Worlds (A).Valid then "运动学没量成"
                    elsif Natural (Ps (A).Length) < Min_Inl then "只三角出 " & Codec.Img (Natural (Ps (A).Length)) & " 个点"
                    elsif Tilt (A) < 0.0 then "桌面上的点不够"
                    else Codec.Img (Natural (Ps (A).Length)) & " 个点、桌面 " & Codec.Img (Inl_A (A)) & " 个、桌面法向准到 " & Codec.Fmt (Tilt (A), 5) & " 弧度"));
      end loop;
      declare
         W_A : constant Integer := World_Arm (Tilt);
      begin
         if W_A < 0 then
            Say ("世界:哪只手都定不了桌面(" & To_String (Note) & ")");
            return;
         end if;
         Ref := Natural (W_A);
         Say ("世界取能定世界的第一只手:第" & Codec.Img (Ref + 1) & " 只(" & To_String (Note) & ")");
      end;
      Pl0 := Pls (Ref); N0 := Nrs (Ref); Pl0_Md := Md_A (Ref);
      if Dist ([0.0, 0.0, 0.0], Pl0, N0) < 0.0 then
         N0 := [-N0 (0), -N0 (1), -N0 (2)];
      end if;
      Nrs (Ref) := N0;
      --  世界那只手的放法就是它自己(恒等)
      declare
         Wr : Arm_World := Worlds (Ref);
      begin
         Wr.S := 1.0; Wr.Ra := Identity; Wr.Ta := [0.0, 0.0, 0.0];
         Worlds.Replace_Element (Ref, Wr);
      end;
      --  世界的桌面:"上" = 桌面法向(朝世界那只手的眼),原点 = 那只眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上
      declare
         Dn : constant Long_Float := N0 (0);
         Xr0 : constant V3 := [1.0 - Dn * N0 (0), -Dn * N0 (1), -Dn * N0 (2)];
         Xr : constant V3 := [Xr0 (0) / Norm (Xr0), Xr0 (1) / Norm (Xr0), Xr0 (2) / Norm (Xr0)];
         Yr : constant V3 := [N0 (1) * Xr (2) - N0 (2) * Xr (1), N0 (2) * Xr (0) - N0 (0) * Xr (2), N0 (0) * Xr (1) - N0 (1) * Xr (0)];
         Ph : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2);
      begin
         Rw := [[Xr (0), Xr (1), Xr (2)], [Yr (0), Yr (1), Yr (2)], [N0 (0), N0 (1), N0 (2)]];
         O := [Ph * N0 (0), Ph * N0 (1), Ph * N0 (2)];
         Say ("世界:第" & Codec.Img (Ref + 1) & " 只手三角出 " & Codec.Img (Natural (Ps (Ref).Length)) & " 个点,桌面拟合了 " & Codec.Img (Inl_A (Ref)) & " 个(离面中位 "
              & Codec.Fmt (Md_A (Ref), 4) & " 单位)⇒ 上 = 桌面法向;眼离桌面 " & Codec.Fmt (abs Ph, 3) & " 单位(长度单位 = 第" & Codec.Img (Ref + 1) & " 只手的模型单位)");
      end;
      Ok := True;
      Placed (Ref) := True;
      for B in 0 .. Na - 1 loop
         if B /= Ref and then Worlds (B).Valid and then Natural (Ps (B).Length) >= Min_Inl then
            Plane_Info (B);
            Say ("世界:第" & Codec.Img (B + 1) & " 只手三角出 " & Codec.Img (Natural (Ps (B).Length)) & " 个点,它自己的桌面拟合了 " & Codec.Img (Inl_A (B)) & " 个(离面中位 "
                 & Codec.Fmt (Md_A (B), 4) & ",算在面上的门 " & Codec.Fmt (Gates (B), 4) & " 单位)");
         end if;
      end loop;
   end;
   --  整体特征:每只手有三角点的格子 + 不长在手上的眼
   for A in 0 .. Na - 1 loop
      if Worlds (A).Valid then
         for F of Frames_Of (A) loop
            if Id_Of (A, Natural (F)) >= 0 and then not D_Ids.Contains (Id_Of (A, Natural (F))) then
               D_Ids.Append (Id_Of (A, Natural (F)));
            end if;
         end loop;
      end if;
   end loop;
   if Fx_Id >= 0 then
      D_Ids.Append (Fx_Id);
   end if;
   declare
      Err : Unbounded_String;
   begin
      D_Vec := Instrument.Describe (Host, Port, D_Ids, Err);
      if Natural (D_Vec.Length) /= Natural (D_Ids.Length) then
         Say ("世界:配点仪器没给整体特征(" & To_String (Err) & ")⇒ 只有第" & Codec.Img (Ref + 1) & " 只手在世界里");
         D_Ids.Clear;
      end if;
   end;
   Add_Arm_Views (Ref);
   --  一轮一轮放
   loop
      declare
         Best_Inl : Natural := 0;
         Best : Integer := -2;   --  -1 = 不动的眼;≥ 1 = 第几只手;-2 = 这一轮谁都放不进
      begin
         --  不动的眼:世界里有点的眼(手的格子)按像它的程度挑没配过的
         if Fx_Id >= 0 and then not Fx_Placed and then not D_Ids.Is_Empty then
            declare
               type Cand is record
                  K : Natural := 0;
                  S : Long_Float := 0.0;
               end record;
               package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
               Cs : Cand_Vectors.Vector;
            begin
               for K in 0 .. Natural (Wv.Length) - 1 loop
                  if Wv (K).Arm >= 0 and then not Was_Tried (Wv (K).Id, Fx_Id) then
                     declare
                        Sc : constant Long_Float := Desc_Dot (Wv (K).Id, Fx_Id);
                        Pos : Natural := Natural (Cs.Length);
                     begin
                        for J in 0 .. Natural (Cs.Length) - 1 loop
                           if Sc > Cs (J).S then
                              Pos := J;
                              exit;
                           end if;
                        end loop;
                        Cs.Insert (Pos, Cand'(K => K, S => Sc));
                     end;
                  end if;
               end loop;
               for C of Cs loop
                  declare
                     Vw : constant World_View := Wv (C.K);
                     Idx, Keep : Ints;
                     Q : Instrument.Match_Vectors.Vector;
                     U, V, E : Floats;
                  begin
                     Pts_In (Natural (Vw.Arm), Vw.Frame, Idx, Q);
                     Match_Pair (Vw.Id, Fx_Id, Fx_W, Fx_H, Q, Keep, U, V, E);
                     N_Pairs_Fx := N_Pairs_Fx + 1;
                     for I in 0 .. Natural (Keep.Length) - 1 loop
                        declare
                           P : constant Tri_Pt := Ps (Natural (Vw.Arm)) (Natural (Idx (Natural (Keep (I)))));
                        begin
                           Cam_Obs.Append (Cam_Ob'(Pa => Natural (Vw.Arm), Pk => Natural (Idx (Natural (Keep (I)))), Wk => C.K, Uw => Q (Natural (Keep (I))).U,
                                                   Vw => Q (Natural (Keep (I))).V, Xw => To_World (Natural (Vw.Arm), P.X),
                                                   Cw => Cov_World (Natural (Vw.Arm), P.Cov), U => U (I), V => V (I), E => E (I)));
                        end;
                     end loop;
                  end;
               end loop;
               declare
                  Gt : Cam_Geo;
                  Rp : Fixed_Report;
                  Inl : Natural;
               begin
                  Fit_Cam (Gt, Rp, Inl);
                  if Inl >= Min_Inl and then Inl > Best_Inl then
                     Best_Inl := Inl; Best := -1;
                  end if;
               end;
            end;
         end if;
         --  别的手:它的格子 × 世界里的眼,按像不像挑没配过的
         for B in 0 .. Na - 1 loop
            if Worlds (B).Valid and then not Placed (B) and then Natural (Ps (B).Length) >= Min_Inl and then not D_Ids.Is_Empty then
               declare
                  type Cand is record
                     F, K : Natural := 0;
                     S : Long_Float := 0.0;
                  end record;
                  package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
                  Cs : Cand_Vectors.Vector;
               begin
                  --  世界里每一只还没和它配过的眼:挑它最像那只眼的一格去配一次
                  for K in 0 .. Natural (Wv.Length) - 1 loop
                     declare
                        Tried_K : Boolean := False;
                        Bf : Integer := -1;
                        Bs : Long_Float := Long_Float'First;
                     begin
                        for F of Frames_Of (B) loop
                           if Id_Of (B, Natural (F)) >= 0 then
                              Tried_K := Tried_K or else Was_Tried (Id_Of (B, Natural (F)), Wv (K).Id);
                              if Desc_Dot (Id_Of (B, Natural (F)), Wv (K).Id) > Bs then
                                 Bs := Desc_Dot (Id_Of (B, Natural (F)), Wv (K).Id); Bf := F;
                              end if;
                           end if;
                        end loop;
                        if not Tried_K and then Bf >= 0 then
                           declare
                              Pos : Natural := Natural (Cs.Length);
                           begin
                              for J in 0 .. Natural (Cs.Length) - 1 loop
                                 if Bs > Cs (J).S then
                                    Pos := J;
                                    exit;
                                 end if;
                              end loop;
                              Cs.Insert (Pos, Cand'(F => Natural (Bf), K => K, S => Bs));
                           end;
                        end if;
                     end;
                  end loop;
                  for C of Cs loop
                     declare
                        Vw : constant World_View := Wv (C.K);
                        Idx, Keep : Ints;
                        Q : Instrument.Match_Vectors.Vector;
                        U, V, E : Floats;
                     begin
                        Pts_In (B, C.F, Idx, Q);
                        Match_Pair (Id_Of (B, C.F), Vw.Id, Vw.W, Vw.H, Q, Keep, U, V, E, Dst_Arm => Vw.Arm);
                        N_Pairs_Of (B) := N_Pairs_Of (B) + 1;
                        declare
                           N_On : Natural := 0;
                        begin
                           for I in 0 .. Natural (Keep.Length) - 1 loop
                              if abs Dist (Ps (B) (Natural (Idx (Natural (Keep (I))))).X, Pls (B), Nrs (B)) <= Gates (B) then
                                 N_On := N_On + 1;
                              end if;
                           end loop;
                           Append (Round_Note, " " & Codec.Img (C.F) & "→" & (if Vw.Arm < 0 then "眼" else "手" & Codec.Img (Vw.Arm + 1) & "格" & Codec.Img (Vw.Frame))
                                   & ":" & Codec.Img (Natural (Q.Length)) & "/" & Codec.Img (Natural (Keep.Length)) & "/" & Codec.Img (N_On));
                        end;
                        for I in 0 .. Natural (Keep.Length) - 1 loop
                           if Ray_Usable (Vw.Cam, U (I), V (I)) then
                              Arm_Obs (B).Append (Arm_Ob'(K => Natural (Idx (Natural (Keep (I)))), Bf => C.F, Wk => C.K, Ro => Vw.Cam.Pos, Rd => Ray_Fixed (Vw.Cam, U (I), V (I)),
                                                          U => U (I), V => V (I), Uw => Q (Natural (Keep (I))).U, Vw => Q (Natural (Keep (I))).V, E => E (I)));
                           end if;
                        end loop;
                     end;
                  end loop;
                  declare
                     S, Md : Long_Float;
                     R : M3;
                     T : V3;
                     Inl : Natural;
                  begin
                     Place_Arm (B, S, R, T, Inl, Md);
                     Cand_S (B) := S; Cand_R (B) := R; Cand_T (B) := T; Cand_Inl (B) := Inl; Cand_Md (B) := Md;
                     Say ("  第" & Codec.Img (B + 1) & " 只手这一轮配了(它的第几格 → 世界里的哪只眼:问几个点 / 往返配上几个 / 其中在它桌面上的):" & To_String (Round_Note)
                          & " ⇒ 一共配上 " & Codec.Img (Natural (Arm_Obs (B).Length)) & " 次,放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次(残差中位 "
                          & Codec.Fmt (Md, 2) & " 倍标准差)· 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
                     Round_Note := Null_Unbounded_String;
                     if Inl >= Min_Inl and then Inl > Best_Inl then
                        Best_Inl := Inl; Best := B;
                     end if;
                  end;
               end;
            end if;
         end loop;
         exit when Best = -2;
         if Best = -1 then
            declare
               Inl : Natural;
            begin
               Fit_Cam (G, G_Rep, Inl);
               Fx_Placed := True;
               Wv.Append (World_View'(Id => Fx_Id, Cam => G, Arm => -1, Frame => 0, W => Fx_W, H => Fx_H));
               --  已经放进世界的别的手,也和这只新放进来的眼配一次(用它最像这只眼的一格;先后不同,证据不能不同)
               for P in 0 .. Na - 1 loop
                  if P /= Ref and then Placed (P) then
                     declare
                        Bf : Integer := -1;
                        Bs : Long_Float := Long_Float'First;
                        Idx, Keep : Ints;
                        Q : Instrument.Match_Vectors.Vector;
                        U, V, E : Floats;
                        K_New : constant Natural := Natural (Wv.Length) - 1;
                     begin
                        for F of Frames_Of (P) loop
                           if Id_Of (P, Natural (F)) >= 0 and then Desc_Dot (Id_Of (P, Natural (F)), Fx_Id) > Bs then
                              Bs := Desc_Dot (Id_Of (P, Natural (F)), Fx_Id); Bf := F;
                           end if;
                        end loop;
                        if Bf >= 0 then
                           Pts_In (P, Natural (Bf), Idx, Q);
                           Match_Pair (Id_Of (P, Natural (Bf)), Fx_Id, Fx_W, Fx_H, Q, Keep, U, V, E);
                           N_Pairs_Of (P) := N_Pairs_Of (P) + 1;
                           for I in 0 .. Natural (Keep.Length) - 1 loop
                              if Ray_Usable (G, U (I), V (I)) then
                                 Arm_Obs (P).Append (Arm_Ob'(K => Natural (Idx (Natural (Keep (I)))), Bf => Natural (Bf), Wk => K_New, Ro => G.Pos, Rd => Ray_Fixed (G, U (I), V (I)),
                                                             U => U (I), V => V (I), Uw => Q (Natural (Keep (I))).U, Vw => Q (Natural (Keep (I))).V, E => E (I)));
                              end if;
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
               Say ("放进世界:不长在手上的那只眼 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,往返 " & Codec.Fmt (Trip_Px) & " px 内共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length))
                    & " 个 ⇒ 焦距 " & Codec.Fmt (G.F, 1) & "、残差 " & Codec.Fmt (G_Rep.Scene_Rms, 2) & " px(进解 " & Codec.Img (G_Rep.Scene_Used) & " / "
                    & Codec.Img (G_Rep.Scene_N) & ")、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位 " & Lap);
            end;
         else
            declare
               S, Md : Long_Float;
               R : M3;
               T : V3;
               Inl : Natural;
               Wb : Arm_World := Worlds (Natural (Best));
               N_Fx, N_Arm : Natural := 0;
            begin
               S := Cand_S (Natural (Best)); R := Cand_R (Natural (Best)); T := Cand_T (Natural (Best));
               Inl := Cand_Inl (Natural (Best)); Md := Cand_Md (Natural (Best));
               Say ("  [计时] 放进世界 " & Lap);
               Wb.S := S; Wb.Ra := R; Wb.Ta := T;
               Worlds.Replace_Element (Natural (Best), Wb);
               Placed (Natural (Best)) := True;
               declare
                  K0 : constant Natural := Natural (Wv.Length);
               begin
                  Add_Arm_Views (Natural (Best));
                  --  已经放进世界的不长在手上的眼,也和这只手新带进来的每一格配一次(同它放进世界时的配法;先后不同,证据不能不同)
                  if Fx_Placed then
                     for K in K0 .. Natural (Wv.Length) - 1 loop
                        declare
                           Vw : constant World_View := Wv (K);
                           Idx, Keep : Ints;
                           Q : Instrument.Match_Vectors.Vector;
                           U, V, E : Floats;
                        begin
                           Pts_In (Natural (Vw.Arm), Vw.Frame, Idx, Q);
                           Match_Pair (Vw.Id, Fx_Id, Fx_W, Fx_H, Q, Keep, U, V, E);
                           N_Pairs_Fx := N_Pairs_Fx + 1;
                           for I in 0 .. Natural (Keep.Length) - 1 loop
                              declare
                                 Pt : constant Tri_Pt := Ps (Natural (Vw.Arm)) (Natural (Idx (Natural (Keep (I)))));
                              begin
                                 Cam_Obs.Append (Cam_Ob'(Pa => Natural (Vw.Arm), Pk => Natural (Idx (Natural (Keep (I)))), Wk => K, Uw => Q (Natural (Keep (I))).U,
                                                         Vw => Q (Natural (Keep (I))).V, Xw => To_World (Natural (Vw.Arm), Pt.X), Cw => Cov_World (Natural (Vw.Arm), Pt.Cov),
                                                         U => U (I), V => V (I), E => E (I)));
                              end;
                           end loop;
                        end;
                     end loop;
                  end if;
               end;
               --  按它全部的配点重放一遍:刚才加密配进不动的眼的那些格也当它的证据(同一批配点,不再配)。放进世界那一步手里只有
               --  "最像那只眼的一格"(09-27 V1B15 回放:配进头顶眼的只有一格 192 个点,网格挑到倍数 1.31,靠一起精修才拉回 1.0066)。
               --  只在重放时借用;放好就撤掉,一起精修里它们只在不动的眼那一组算一次
               if Fx_Placed then
                  declare
                     Bi : constant Natural := Natural (Best);
                     N_Before : constant Natural := Natural (Arm_Obs (Bi).Length);
                     Fk : Natural := 0;
                     Have : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural'Max (1, Natural (Ds (Bi).Frames.Length))));
                  begin
                     for K in 0 .. Natural (Wv.Length) - 1 loop
                        if Wv (K).Arm < 0 then
                           Fk := K;
                        end if;
                     end loop;
                     for Ob of Arm_Obs (Bi) loop
                        if Ob.Wk = Fk and then Ob.Bf < Natural (Have.Length) then
                           Have.Replace_Element (Ob.Bf, True);   --  这一格和不动的眼放进世界之前就配过了,已经在它的证据里
                        end if;
                     end loop;
                     for X of Cam_Obs loop
                        if X.Pa = Bi and then Wv (X.Wk).Arm = Integer (Bi) and then Wv (X.Wk).Frame < Natural (Have.Length) and then not Have (Wv (X.Wk).Frame)
                          and then Ray_Usable (G, X.U, X.V)
                        then
                           Arm_Obs (Bi).Append (Arm_Ob'(K => X.Pk, Bf => Wv (X.Wk).Frame, Wk => Fk, Ro => G.Pos, Rd => Ray_Fixed (G, X.U, X.V),
                                                       U => X.U, V => X.V, Uw => X.Uw, Vw => X.Vw, E => X.E));
                        end if;
                     end loop;
                     Say ("  [计时] 加密完 " & Lap);
                     if Natural (Arm_Obs (Bi).Length) > N_Before then
                        Place_Arm (Bi, S, R, T, Inl, Md);
                        Wb.S := S; Wb.Ra := R; Wb.Ta := T;
                        Worlds.Replace_Element (Bi, Wb);
                        Say ("  第" & Codec.Img (Bi + 1) & " 只手按全部配点重放(加上加密配进不长在手上的眼的 " & Codec.Img (Natural (Arm_Obs (Bi).Length) - N_Before)
                             & " 个):放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次 ⇒ 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
                     end if;
                     Arm_Obs (Bi).Set_Length (Ada.Containers.Count_Type (N_Before));
                  end;
               end if;
               for P of Tried loop
                  if Ds (Natural (Best)).Ids.Contains (P.A) then
                     if P.B = Fx_Id then
                        N_Fx := N_Fx + 1;
                     else
                        N_Arm := N_Arm + 1;
                     end if;
                  end if;
               end loop;
               Say ("放进世界:第" & Codec.Img (Natural (Best) + 1) & " 只手 —— 配了 " & Codec.Img (N_Pairs_Of (Natural (Best))) & " 对画面(对不长在手上的眼 " & Codec.Img (N_Fx)
                    & " 对、对别的手的格子 " & Codec.Img (N_Arm) & " 对),往返 " & Codec.Fmt (Trip_Px) & " px 内配上 " & Codec.Img (Natural (Arm_Obs (Natural (Best)).Length))
                    & " 次,放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次(残差中位 " & Codec.Fmt (Md, 2) & " 倍标准差)⇒ 长度倍数 " & Codec.Fmt (S, 4)
                    & ";精修按量到的噪声:配点 " & Sm_Img (Cand_P (Natural (Best)).Sm) & " px、远近是它自报的 " & Codec.Fmt (Cand_P (Natural (Best)).K, 2)
                    & " 倍;换到代价更低的一坑 " & Codec.Img (Cand_P (Natural (Best)).Escapes) & " 次 " & Lap);
            end;
         end if;
      end;
   end loop;
   Say ("  一起精修之前 " & Lap);
   Joint_Refine;
   for B in 0 .. Na - 1 loop
      if B /= Ref and then Placed (B) then
         Say ("一起精修以后:第" & Codec.Img (B + 1) & " 只手 长度倍数 " & Codec.Fmt (Worlds (B).S, 4) & "、平移 (" & Codec.Fmt (Worlds (B).Ta (0), 3) & ", "
              & Codec.Fmt (Worlds (B).Ta (1), 3) & ", " & Codec.Fmt (Worlds (B).Ta (2), 3) & ") 单位");
      end if;
   end loop;
   --  放完照实算:每只放进世界的手,它干活的地方沿最不准的方向真的有多不准(按一起精修以后的放法、一起精修量到的噪声、配上它的眼此刻的位姿),
   --  和它自己的眼在那儿量一个点有多不准比 —— 放法比它还不准 ⇒ 放法是它干活时最不准的那一环,该两只手一起去看共同的近处再放
   for B in 0 .. Na - 1 loop
      if B /= Ref and then Placed (B) and then Worlds (B).Valid then
         declare
            Pl : Hand_Place := Cand_P (B);
            Wb : Arm_World := Worlds (B);
            Eye : Long_Float;
         begin
            Pl.S := Wb.S; Pl.R := Wb.Ra; Pl.T := Wb.Ta;
            if Natural (Jr_Sm.Length) > 2 * B + 1 and then Natural (Jr_K.Length) > B then
               Pl.Sm := Filled (2, 0.0);   --  两组(结构:配进手的格子 / 配进不长在手上的眼)
               Pl.Sm.Replace_Element (0, Jr_Sm (2 * B)); Pl.Sm.Replace_Element (1, Jr_Sm (2 * B + 1));
               Pl.K := Jr_K (B);
            end if;
            Hand_Sd (Hand_Obs_Of (B), Tie_Of (B), Pl);
            Eye := Eye_Sd_Of (B, Pl.K, Wb.S);
            Wb.Place_Dir := Pl.Weak; Wb.Place_Sd := Pl.Sd_Real; Wb.Place_Eye_Sd := Eye; Wb.Place_Ok := Pl.Sd_Real <= Eye;
            Worlds.Replace_Element (B, Wb);
            Cand_P (B) := Pl;
            Say ("放进世界的第" & Codec.Img (B + 1) & " 只手:它干活的地方(它的桌面中心)沿 (" & Codec.Fmt (Pl.Weak (0), 2) & ", " & Codec.Fmt (Pl.Weak (1), 2) & ", "
                 & Codec.Fmt (Pl.Weak (2), 2) & ") 最不准 —— 真的不准 " & Codec.Fmt (Pl.Sd_Real, 4) & " 单位(形式 " & Codec.Fmt (Pl.Sd_Formal, 4) & "、系统 "
                 & Codec.Fmt (Pl.Sd_Sys, 4) & ":这只手三角点的远近真的不准是它自报的 " & Codec.Fmt (Pl.K, 2) & " 倍,整条远近误差(不小于自报的)按全相关线性加);它自己的眼在那儿量一个点不准 "
                 & Codec.Fmt (Eye, 4) & " 单位 ⇒ "
                 & (if Wb.Place_Ok then "放法不比它自己的眼差,准"
                    else "放法比它自己的眼还不准 —— 不准:该两只手一起去看共同的近处(两手之间的桌面、对方的手)再放;开机这一段还没有动手去看的那一步,照实用这一份"));
         end;
      end if;
   end loop;
   if Fx_Placed then
      Say ("一起精修以后:不长在手上的那只眼 焦距 " & Codec.Fmt (G.F, 1) & "、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位");
   end if;
   for B in 0 .. Na - 1 loop
      if Worlds (B).Valid and then not Placed (B) then
         declare
            Wb : Arm_World := Worlds (B);
         begin
            Wb.Valid := False;
            Worlds.Replace_Element (B, Wb);
            Say ("世界:第" & Codec.Img (B + 1) & " 只手放不进世界 —— 配了 " & Codec.Img (N_Pairs_Of (B)) & " 对画面,共同看见的点只有 " & Codec.Img (Natural (Arm_Obs (B).Length))
                 & " 个(它和世界里的眼没有共同看见的东西)⇒ 这只手先不用");
         end;
      end if;
   end loop;
   if Fx_Id >= 0 and then not Fx_Placed then
      Say ("世界:不长在手上的那只眼放不进世界 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length)) & " 个");
   end if;
   if Fx_Placed then
      Fixed_Eye := G;
      Fixed_Eye.R_Ce := Mul (Rw, G.R_Ce);
      Fixed_Eye.Pos := Ap (Rw, [G.Pos (0) - O (0), G.Pos (1) - O (1), G.Pos (2) - O (2)]);
   end if;
   --  交给开机后半段:世界系的桌面(原点在桌面上、法向 +z,见上面定世界那一段),离散 = 离面中位换标准差
   Plane_Pt := Ap (Rw, [Pl0 (0) - O (0), Pl0 (1) - O (1), Pl0 (2) - O (2)]);
   Plane_N := Ap (Rw, N0);
   Plane_Rms := 1.4826 * Pl0_Md;   --  正态下中位绝对偏差 → 标准差(统计常数,无量纲)
   --  板:配进不动的眼的每个点(同一个点从几格配进来的,留往返差最小的那一笔),按一起精修以后的放法搬进世界;
   --  每轴噪声 = 这一批往返差的中位 ÷ 1.1774(二维正态下距离的中位 = 1.1774 σ,统计常数)
   if Fx_Placed and then not Cam_Obs.Is_Empty then
      declare
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : Floats;
         Sh : Long_Float;
         package Key_Maps is new Ada.Containers.Vectors (Natural, Integer);
         Best : array (0 .. Natural'Max (1, Na) - 1) of Key_Maps.Vector;
      begin
         for X of Cam_Obs loop
            Es.Append (X.E);
         end loop;
         Sorting.Sort (Es);
         Sh := Long_Float'Max (Es (Natural (Es.Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
         for A in 0 .. Na - 1 loop
            Best (A) := Key_Maps.To_Vector (-1, Ada.Containers.Count_Type (Natural'Max (1, Natural (Ps (A).Length))));
         end loop;
         for I in 0 .. Natural (Cam_Obs.Length) - 1 loop
            declare
               X : constant Cam_Ob := Cam_Obs (I);
            begin
               if X.Pa < Na and then Placed (X.Pa) and then X.Pk < Natural (Best (X.Pa).Length)
                 and then (Best (X.Pa) (X.Pk) < 0 or else X.E < Cam_Obs (Natural (Best (X.Pa) (X.Pk))).E)
               then
                  Best (X.Pa).Replace_Element (X.Pk, Integer (I));
               end if;
            end;
         end loop;
         for A in 0 .. Na - 1 loop
            for K in 0 .. Natural (Best (A).Length) - 1 loop
               if Best (A) (K) >= 0 then
                  declare
                     X : constant Cam_Ob := Cam_Obs (Natural (Best (A) (K)));
                     Sa : constant Long_Float := (if A = Ref then 1.0 else Worlds (A).S);
                     Ra : constant M3 := (if A = Ref then Identity else Worlds (A).Ra);
                     Ta : constant V3 := (if A = Ref then [0.0, 0.0, 0.0] else Worlds (A).Ta);
                     Rx : constant V3 := Ap (Ra, Ps (A) (K).X);
                     X0 : constant V3 := [Sa * Rx (0) + Ta (0) - O (0), Sa * Rx (1) + Ta (1) - O (1), Sa * Rx (2) + Ta (2) - O (2)];
                     Rc : constant M3 := Mul (Mul (Mul (Rw, Ra), Ps (A) (K).Cov), Tr (Mul (Rw, Ra)));
                     Cw : M3;
                  begin
                     for I in 0 .. 2 loop
                        for J in 0 .. 2 loop
                           Cw (I, J) := Sa * Sa * Rc (I, J);
                        end loop;
                     end loop;
                     Board.Append (Scene_Pt'(Pw => Ap (Rw, X0), Cov => Cw, U => X.U, V => X.V, Sh => Sh, Views => 2));
                  end;
               end if;
            end loop;
         end loop;
         Say ("交给开机后半段:板 " & Codec.Img (Natural (Board.Length)) & " 个点(配进不动的眼的,每轴噪声 " & Codec.Fmt (Sh, 2) & " px)· 桌面离散 "
              & Codec.Fmt (Plane_Rms, 4) & " 单位");
      end;
   end if;
   if Dump /= "" then
      declare
         Fo : Ada.Text_IO.File_Type;
      begin
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/world.txt");
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Ada.Text_IO.Put (Fo, Codec.Fmt (Rw (I, J), 9) & " ");
            end loop;
         end loop;
         Ada.Text_IO.Put_Line (Fo, Codec.Fmt (O (0), 9) & " " & Codec.Fmt (O (1), 9) & " " & Codec.Fmt (O (2), 9));
         Ada.Text_IO.Close (Fo);
         if Fx_Placed then
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/fixed_eye.txt");
            Ada.Text_IO.Put (Fo, "f " & Codec.Fmt (G.F, 6) & " cx " & Codec.Fmt (G.Cx, 3) & " cy " & Codec.Fmt (G.Cy, 3) & " rms " & Codec.Fmt (G_Rep.Scene_Rms, 4)
                             & " used " & Codec.Img (G_Rep.Scene_Used) & " of " & Codec.Img (G_Rep.Scene_N) & " pos " & Codec.Fmt (G.Pos (0), 9) & " "
                             & Codec.Fmt (G.Pos (1), 9) & " " & Codec.Fmt (G.Pos (2), 9) & " R");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Ada.Text_IO.Put (Fo, " " & Codec.Fmt (G.R_Ce (I, J), 9));
               end loop;
            end loop;
            Ada.Text_IO.New_Line (Fo);
            Ada.Text_IO.Close (Fo);
         end if;
         --  配进不动的眼的每一笔(离线查它的位姿准不准用):哪只手、它的第几格、第几个三角点、在不动的眼里的像素、往返差、这个点在那只手自己系里的坐标和协方差、三角它的另一格
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/cam_obs.txt");
         for X of Cam_Obs loop
            declare
               P : constant Tri_Pt := Ps (X.Pa) (X.Pk);
            begin
               Ada.Text_IO.Put_Line (Fo, Codec.Img (X.Pa) & " " & Codec.Img (Wv (X.Wk).Frame) & " " & Codec.Img (X.Pk) & " " & Codec.Fmt (X.U, 3) & " " & Codec.Fmt (X.V, 3)
                                     & " " & Codec.Fmt (X.E, 3) & " " & Codec.Fmt (P.X (0), 6) & " " & Codec.Fmt (P.X (1), 6) & " " & Codec.Fmt (P.X (2), 6)
                                     & " " & Codec.Fmt (P.Cov (0, 0), 9) & " " & Codec.Fmt (P.Cov (0, 1), 9) & " " & Codec.Fmt (P.Cov (0, 2), 9)
                                     & " " & Codec.Fmt (P.Cov (1, 1), 9) & " " & Codec.Fmt (P.Cov (1, 2), 9) & " " & Codec.Fmt (P.Cov (2, 2), 9)
                                     & " " & Codec.Img (P.Fr));
            end;
         end loop;
         Ada.Text_IO.Close (Fo);
         --  每只手三角出的点(它自己系里,离线查放法用):第几个、坐标、协方差(上三角)、三角它的另一格、在起点那格里的像素;第一行 = 这只手配点的噪声(像素),
         --  第二行 = 它自己的桌面(面上一点、法向、算在面上的门;放进世界用的中心、朝眼的法向、倾角 / 高度的不确定度)
         for A in 0 .. Na - 1 loop
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/tri_arm" & Codec.Img (A) & ".txt");
            Ada.Text_IO.Put_Line (Fo, "sig " & Codec.Fmt (Sig_Px (A), 6));
            Ada.Text_IO.Put_Line (Fo, "plane " & Codec.Fmt (Pls (A) (0), 9) & " " & Codec.Fmt (Pls (A) (1), 9) & " " & Codec.Fmt (Pls (A) (2), 9)
                                  & " " & Codec.Fmt (Nrs (A) (0), 9) & " " & Codec.Fmt (Nrs (A) (1), 9) & " " & Codec.Fmt (Nrs (A) (2), 9) & " " & Codec.Fmt (Gates (A), 9)
                                  & " " & Codec.Fmt (Pl_Cb (A) (0), 9) & " " & Codec.Fmt (Pl_Cb (A) (1), 9) & " " & Codec.Fmt (Pl_Cb (A) (2), 9)
                                  & " " & Codec.Fmt (Pl_Nb (A) (0), 9) & " " & Codec.Fmt (Pl_Nb (A) (1), 9) & " " & Codec.Fmt (Pl_Nb (A) (2), 9)
                                  & " " & Codec.Fmt (Pl_Sn (A), 12) & " " & Codec.Fmt (Pl_Sd (A), 12));
            for K in 0 .. Natural (Ps (A).Length) - 1 loop
               declare
                  P : constant Tri_Pt := Ps (A) (K);
               begin
                  Ada.Text_IO.Put_Line (Fo, Codec.Img (K) & " " & Codec.Fmt (P.X (0), 6) & " " & Codec.Fmt (P.X (1), 6) & " " & Codec.Fmt (P.X (2), 6)
                                        & " " & Codec.Fmt (P.Cov (0, 0), 9) & " " & Codec.Fmt (P.Cov (0, 1), 9) & " " & Codec.Fmt (P.Cov (0, 2), 9)
                                        & " " & Codec.Fmt (P.Cov (1, 1), 9) & " " & Codec.Fmt (P.Cov (1, 2), 9) & " " & Codec.Fmt (P.Cov (2, 2), 9)
                                        & " " & Codec.Img (P.Fr) & " " & Codec.Fmt (P.U0, 2) & " " & Codec.Fmt (P.V0, 2));
               end;
            end loop;
            Ada.Text_IO.Close (Fo);
         end loop;
         --  世界那只手是第几只(第 0 只以外的,离线工具按它认世界;没有这个文件 = 第 0 只)
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/world_arm.txt");
         Ada.Text_IO.Put_Line (Fo, Codec.Img (Ref));
         Ada.Text_IO.Close (Fo);
         for B in 0 .. Na - 1 loop
            if B /= Ref and then Placed (B) then
               Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/align_arm" & Codec.Img (B) & ".txt");
               Ada.Text_IO.Put (Fo, "S " & Codec.Fmt (Worlds (B).S, 9) & " R");
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     Ada.Text_IO.Put (Fo, " " & Codec.Fmt (Worlds (B).Ra (I, J), 9));
                  end loop;
               end loop;
               --  第一行后面接放法的账(离线打分核"真的有多不准"准不准用):真的 / 形式 / 系统的标准差、它自己的眼量点的标准差(世界单位)、远近放大、最不准的方向、准不准
               Ada.Text_IO.Put_Line (Fo, " T " & Codec.Fmt (Worlds (B).Ta (0), 9) & " " & Codec.Fmt (Worlds (B).Ta (1), 9) & " " & Codec.Fmt (Worlds (B).Ta (2), 9)
                                     & " sd " & Codec.Fmt (Cand_P (B).Sd_Real, 6) & " formal " & Codec.Fmt (Cand_P (B).Sd_Formal, 6) & " sys " & Codec.Fmt (Cand_P (B).Sd_Sys, 6)
                                     & " eye " & Codec.Fmt (Worlds (B).Place_Eye_Sd, 6) & " k " & Codec.Fmt (Cand_P (B).K, 4)
                                     & " dir " & Codec.Fmt (Cand_P (B).Weak (0), 4) & " " & Codec.Fmt (Cand_P (B).Weak (1), 4) & " " & Codec.Fmt (Cand_P (B).Weak (2), 4)
                                     & " ok " & (if Worlds (B).Place_Ok then "1" else "0") & " escapes " & Codec.Img (Cand_P (B).Escapes));
               for X of Arm_Obs (B) loop
                  declare
                     Xb : constant V3 := Ps (B) (X.K).X;
                     Rt : constant V3 := Ap (Worlds (B).Ra, Xb);
                     Xw : constant V3 := [Worlds (B).S * Rt (0) + Worlds (B).Ta (0), Worlds (B).S * Rt (1) + Worlds (B).Ta (1), Worlds (B).S * Rt (2) + Worlds (B).Ta (2)];
                  begin
                     Ada.Text_IO.Put_Line (Fo, Codec.Img (Wv (X.Wk).Arm) & " " & Codec.Img (Wv (X.Wk).Frame) & " " & Codec.Img (X.K) & " "
                                           & Codec.Fmt (X.U, 3) & " " & Codec.Fmt (X.V, 3) & " " & Codec.Fmt (X.Uw, 3) & " " & Codec.Fmt (X.Vw, 3) & " " & Codec.Fmt (X.E, 3)
                                           & " " & Codec.Fmt (Xw (0), 6) & " " & Codec.Fmt (Xw (1), 6) & " " & Codec.Fmt (Xw (2), 6)
                                           & " " & Codec.Fmt (Xb (0), 6) & " " & Codec.Fmt (Xb (1), 6) & " " & Codec.Fmt (Xb (2), 6));
                  end;
               end loop;
               Ada.Text_IO.Close (Fo);
            end if;
         end loop;
      exception
         when others => null;
      end;
   end if;
   --  每只眼长在谁身上(I3):开机量的(Eyes / World_Cam),和这一回哪几只手量成、放进了世界合起来照实说 —— 扫坏的手的眼还是那只手上的
   if not Eyes.Is_Empty then
      declare
         Groups : Ints;
         Valid : Bools;
         Nc : Natural := N_Cams;
      begin
         for A in 0 .. Na - 1 loop
            Groups.Append (Worlds (A).Group); Valid.Append (Worlds (A).Valid);
         end loop;
         for E of Eyes loop
            Nc := Natural'Max (Nc, Natural (Integer'Max (0, E + 1)));
         end loop;
         Nc := Natural'Max (Nc, Natural (Integer'Max (0, World_Cam + 1)));
         Say ("每只眼长在谁身上(开机推每组读数量的;世界 = 第" & Codec.Img (Ref + 1) & " 只手的系):"
              & Eye_Say (Eye_List (Nc, Eyes, Groups, Valid, World_Cam, Integer (Ref), Fx_Placed)));
      end;
   end if;
   Say ("  对齐用了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒 " & Lap);
end Align;
