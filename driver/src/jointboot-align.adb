separate (Jointboot)
procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                 Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                 Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "";
                 Pin_Fixed_F : Long_Float := 0.0) is
   use Geom;
   Max_Pts : constant := Gx * Gy;   --  每只手最多三角几个点(同扫描格点数,次数)
   T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   Na : constant Natural := Natural (Worlds.Length);
   Ps : array (0 .. Natural'Max (1, Na) - 1) of Tri_Vectors.Vector;
   Pls, Nrs : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
   Gates : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
   Sig_Px : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];   --  每只手自己配点的噪声(像素,Tri_Pts 量的;给它的三角点定不确定度)
   Placed : array (0 .. Natural'Max (1, Na) - 1) of Boolean := [others => False];
   Pl0, N0 : V3 := [0.0, 0.0, 1.0];  --  世界的桌面(第一只手系里)
   Pl0_Md : Long_Float := 0.0;       --  第一只手三角出、拟合进桌面的点离面的中位
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
   --  不长在手上的眼(这具身体有几只就几只;V1b 这一版的扫描只存了一只)
   Fx_Id : constant Integer := Ds (0).World_Id;
   Fx_Placed : Boolean := False;
   G : Cam_Geo := No_Geo;             --  它(第一只手系里)
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
   --  一个点投进世界里一只眼的像素残差,按"配点噪声 ⊕ 这个点自己三角的不确定度投进这只眼"白化(2 条,以标准差为单位;同 Geom.Scene_Var 的做法):
   --  三角的远近误差从侧面看是几像素(V1B11:第 2 只手的点投进 60 cm 外第 1 只手的眼,不加权时按真值对齐也只有 54% 在 3 px 内),加权以后按它实际的精度算;
   --  不用两只眼之间的对极误差 —— 它只定得住两只眼往哪个方向隔开、定不住隔开多远(V1B12:朝向一样、横着隔开的那几对下,平移错 431 mm)
   procedure Proj_W (Cg : Cam_Geo; Xw : V3; Cw : M3; U, V, Sig_M : Long_Float; E1, E2 : out Long_Float) is
      Rt : constant M3 := Tr (Cg.R_Ce);
      Pc : constant V3 := Ap (Rt, [Xw (0) - Cg.Pos (0), Xw (1) - Cg.Pos (1), Xw (2) - Cg.Pos (2)]);
      Z : constant Long_Float := -Pc (2);
   begin
      if Z <= 0.0 or else Cg.F <= 0.0 then
         E1 := 1.0e3; E2 := 1.0e3;   --  在那只眼后面:远大于任何一条白化残差的罚(无量纲哨兵)
         return;
      end if;
      declare
         Du : constant V3 := [Cg.F / Z, 0.0, Cg.F * Pc (0) / (Z * Z)];
         Dv : constant V3 := [0.0, -Cg.F / Z, -Cg.F * Pc (1) / (Z * Z)];
         Ju, Jv : V3 := [0.0, 0.0, 0.0];
         A11, A12, A22 : Long_Float;
         Eu : constant Long_Float := Cg.F * Pc (0) / Z + Cg.Cx - U;
         Ev : constant Long_Float := -Cg.F * Pc (1) / Z + Cg.Cy - V;
         function Q (X, Y : V3) return Long_Float is
            Sm : Long_Float := 0.0;
         begin
            for I in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Sm := Sm + X (I) * Cw (I, K) * Y (K);
               end loop;
            end loop;
            return Sm;
         end Q;
      begin
         for K in 0 .. 2 loop
            Ju (K) := Du (0) * Rt (0, K) + Du (1) * Rt (1, K) + Du (2) * Rt (2, K);
            Jv (K) := Dv (0) * Rt (0, K) + Dv (1) * Rt (1, K) + Dv (2) * Rt (2, K);
         end loop;
         A11 := Sig_M * Sig_M + Q (Ju, Ju); A12 := Q (Ju, Jv); A22 := Sig_M * Sig_M + Q (Jv, Jv);
         declare
            L11 : constant Long_Float := Sqrt (Long_Float'Max (A11, 1.0e-18));   --  数值保护(无量纲)
            L21 : constant Long_Float := A12 / L11;
            L22 : constant Long_Float := Sqrt (Long_Float'Max (A22 - L21 * L21, 1.0e-18));   --  同上
         begin
            E1 := Eu / L11;
            E2 := (Ev - L21 * E1) / L22;
         end;
      end;
   end Proj_W;      procedure Plane_Of (P : Tri_Vectors.Vector; Pl, Nrm : out V3; Gate : out Long_Float; Inl : out Natural; Md : out Long_Float) is
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
   --  放这只眼只拿世界那只手(第 0 只)自己三角出的点:它们在世界里的位置就是它自己系里的位置,不带任何"放进世界"的误差。
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
         if X.Pa = 0 then
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
            if X.Pa = 0 then
               Es.Append (X.E);
            end if;
         end loop;
         Sorting.Sort (Es);
         Sh := Es (Natural (Es.Length) / 2);   --  这只眼里配点的噪声 = 这一次往返差的中位(像素)
      end;
      for X of Cam_Obs loop
         if X.Pa = 0 then
            Scene.Append (Scene_Pt'(Pw => X.Xw, Cov => X.Cw, U => X.U, V => X.V, Sh => Sh, Views => 2));
         end if;
      end loop;
      Gt.Cx := Long_Float (Ds (0).World_Img.W) / 2.0; Gt.Cy := Long_Float (Ds (0).World_Img.H) / 2.0; Gt.F := Pin_Fixed_F;   --  焦距一起解(Pin_Fixed_F = 0;离线对照实验才钉)
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
   procedure Plane_Info (B : Natural) is
      Ext_B, Ext_0 : Long_Float := 0.0;
      C0, Cb : V3 := [0.0, 0.0, 0.0];
      N_0, N_B : Natural := 0;
   begin
      for Q of Ps (B) loop
         if abs Dist (Q.X, Pls (B), Nrs (B)) <= Gates (B) then
            Cb := [Cb (0) + Q.X (0), Cb (1) + Q.X (1), Cb (2) + Q.X (2)]; N_B := N_B + 1;
         end if;
      end loop;
      if N_B > 0 then
         Cb := [Cb (0) / Long_Float (N_B), Cb (1) / Long_Float (N_B), Cb (2) / Long_Float (N_B)];
         for Q of Ps (B) loop
            if abs Dist (Q.X, Pls (B), Nrs (B)) <= Gates (B) then
               Ext_B := Ext_B + Norm ([Q.X (0) - Cb (0), Q.X (1) - Cb (1), Q.X (2) - Cb (2)]) ** 2;
            end if;
         end loop;
         Ext_B := Sqrt (Ext_B / Long_Float (N_B));
      end if;
      for Q of Ps (0) loop
         if abs Dist (Q.X, Pl0, N0) <= Gates (0) then
            C0 := [C0 (0) + Q.X (0), C0 (1) + Q.X (1), C0 (2) + Q.X (2)]; N_0 := N_0 + 1;
         end if;
      end loop;
      if N_0 > 0 then
         C0 := [C0 (0) / Long_Float (N_0), C0 (1) / Long_Float (N_0), C0 (2) / Long_Float (N_0)];
         for Q of Ps (0) loop
            if abs Dist (Q.X, Pl0, N0) <= Gates (0) then
               Ext_0 := Ext_0 + Norm ([Q.X (0) - C0 (0), Q.X (1) - C0 (1), Q.X (2) - C0 (2)]) ** 2;
            end if;
         end loop;
         Ext_0 := Sqrt (Ext_0 / Long_Float (N_0));
      end if;
      Pl_Cb (B) := Cb;
      Pl_Nb (B) := (if Dist ([0.0, 0.0, 0.0], Pls (B), Nrs (B)) < 0.0 then [-Nrs (B) (0), -Nrs (B) (1), -Nrs (B) (2)] else Nrs (B));
      declare
         --  两张面各自的高度标准误差(它自己的单位):离面散布 ÷ √点数
         Se_B : constant Long_Float := Gates (B) / 2.5 / Sqrt (Long_Float (Natural'Max (1, N_B)));   --  Gates = 2.5 × 1.4826 × 中位 ⇒ ÷ 2.5 = 离面散布(统计常数,无量纲)
         Se_0 : constant Long_Float := Gates (0) / 2.5 / Sqrt (Long_Float (Natural'Max (1, N_0)));
      begin
         Pl_Sn (B) := Sqrt ((Se_B / Long_Float'Max (Ext_B, 1.0e-12)) ** 2 + (Se_0 / Long_Float'Max (Ext_0, 1.0e-12)) ** 2);   --  数值保护(无量纲)
         Pl_Sd (B) := Sqrt (Se_0 ** 2 + Se_B ** 2);   --  第 B 只手那份按它自己的单位,和世界单位差一个倍数(约 1,放进世界之前不知道)
      end;
   end Plane_Info;
   procedure Plane_Res (B : Natural; Sx : Long_Float; Rx : M3; Tx : V3; R1, R2, R3 : out Long_Float) is
      Nw : constant V3 := Ap (Rx, Pl_Nb (B));
      Tilt : constant V3 := Cross (Nw, N0);
      E1, E2 : V3;
      Rt : constant V3 := Ap (Rx, Pl_Cb (B));
      Sig_P : constant Long_Float := Long_Float'Max (1.0e-12, Pl_Sd (B));   --  数值保护(无量纲)
      D0w : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2);
   begin
      E1 := Cross (N0, (if abs N0 (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  垂直于世界法向的一对方向(纯数学,无量纲)
      E1 := [E1 (0) / Norm (E1), E1 (1) / Norm (E1), E1 (2) / Norm (E1)];
      E2 := Cross (N0, E1);
      R1 := (Tilt (0) * E1 (0) + Tilt (1) * E1 (1) + Tilt (2) * E1 (2)) / Pl_Sn (B);
      R2 := (Tilt (0) * E2 (0) + Tilt (1) * E2 (1) + Tilt (2) * E2 (2)) / Pl_Sn (B);
      R3 := (N0 (0) * (Sx * Rt (0) + Tx (0)) + N0 (1) * (Sx * Rt (1) + Tx (1)) + N0 (2) * (Sx * Rt (2) + Tx (2)) - D0w) / Sig_P;
   end Plane_Res;
   --  一串残差的门:max(3 px, 3 × 中位)(协议,同运动学一起解)
   function Gate_Of (Es : Floats) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      Sorted : Floats := Es;
   begin
      if Sorted.Is_Empty then
         return 3.0;
      end if;
      Sorting.Sort (Sorted);
      return Long_Float'Max (3.0, 3.0 * Sorted (Natural (Sorted.Length) / 2));
   end Gate_Of;

   --  第 B 只手放进世界:求 X_世界 = S · R · X + T,让 ① 它每个配上的点(它自己三角出的,桌面上的、东西上的都算)投回世界里配上它的那只眼,
   --  落在配上的那个像素上(像素残差);② 它的桌面和世界的桌面重合(上面那 3 条)。
   --  起步:两张桌面的法向对齐,剩下"绕法向转多少"× 长度倍数铺网格,每一格的平移按视线的线性最小二乘直接解,挑截断(3 倍,同挑内点的门)平方和最小的;
   --  再全部一起抗野点精修两轮(门 max(3 px, 3 × 中位))。Inl = 投回去差不到 3 px 的配点数,Md = 它们的像素残差中位
   N_Theta : constant := 360;   --  绕法向每 1° 一档(次数)
   N_Scale : constant := 41;    --  长度倍数 0.1–10 按对数分 41 档(每档约 12%;倍数范围是比例,无量纲)
   Coarse_T : constant := 5;    --  网格先粗找:转角 5 档并 1 档(5°)、倍数 2 档并 1 档(约 26%),再在最好那格周围按细档补(次数)
   Coarse_S : constant := 2;
   procedure Place_Arm (B : Natural; S : out Long_Float; R : out M3; T : out V3; Inl : out Natural; Md : out Long_Float) is
      Nr : constant Natural := Natural (Arm_Obs (B).Length);
      Sig_B : constant Long_Float := Cross_Sig (B);
      --  两组配点各自的噪声倍数(方差分量:按这一组自己的残差估,见 Group_Scale):0 = 配进手的格子,1 = 配进不长在手上的眼
      Fac : array (0 .. 1) of Long_Float := [1.0, 1.0];
      function Grp (Ob : Arm_Ob) return Natural is (if Wv (Ob.Wk).Arm < 0 then 1 else 0);
      R0 : M3 := Identity;
      function Rz (Th : Long_Float) return M3 is (Rodrigues ([N0 (0) * Th, N0 (1) * Th, N0 (2) * Th]));
      --  放进世界只是给一起精修找起点 ⇒ 网格的分数和这里的精修每组按顺序等间隔最多取 Sub_Max 条(次数);每一格的平移用全部
      --  (平移按普通最小二乘解,子集里错配那组占的份量变大,09-27 V1B15 回放起点从倍数 1.0060 偏到 1.0394);数门里有几条用全部。
      --  一起精修(Joint_Refine)用全部
      Sub_Max : constant := 400;
      In_Sub : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Nr));
      --  给定倍数、转动 ⇒ 平移:沿世界法向那一分量由"它的桌面落在世界的桌面上"定死(桌面是几百个点拟合的,高度的标准误差很小;
      --  精修时再按标准误差放软,见 Plane_Res),平面里那两个分量按视线的线性最小二乘(点到视线的距离)。
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
      --  一条配点:它的点按(Sx, Rx, Tx)放进世界、投回世界里那只眼,白化残差两条
      procedure Res2 (Ob : Arm_Ob; Cg : Cam_Geo; Sx : Long_Float; Rx : M3; Tx : V3; E1, E2 : out Long_Float) is
         Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
         Xw : constant V3 := [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)];
         Rc : constant M3 := Mul (Mul (Rx, Ps (B) (Ob.K).Cov), Tr (Rx));
         Cw : M3;
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Cw (I, J) := Sx * Sx * Rc (I, J);
            end loop;
         end loop;
         Proj_W (Cg, Xw, Cw, Ob.U, Ob.V, Sig_B, E1, E2);
         E1 := E1 / Fac (Grp (Ob)); E2 := E2 / Fac (Grp (Ob));
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
      --  按这一组自己的残差把它的噪声倍数重估一遍:白化后二维残差的长度,单位方差时中位 = √(2 ln 2) ≈ 1.1774(瑞利分布的中位,统计常数,无量纲)
      procedure Group_Scale is
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : array (0 .. 1) of Floats;
      begin
         Fac := [1.0, 1.0];
         for Ob of Arm_Obs (B) loop
            Es (Grp (Ob)).Append (Epi (Ob, S, R, T));
         end loop;
         for G in Es'Range loop
            if not Es (G).Is_Empty then
               Sorting.Sort (Es (G));
               Fac (G) := Long_Float'Max (Es (G) (Natural (Es (G).Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
            end if;
         end loop;
      end Group_Scale;
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
      --  一起精修:每轮先按各组自己的残差重估噪声倍数,再挑门里的、解,做到门里的那一组不再变(Until_Settled;原来固定三轮)
      declare
         procedure Pick (Use_R : out Bools; Enough : out Boolean) is
            Es : Floats;
            Gate : Long_Float;
            Nu : Natural := 0;
         begin
            Group_Scale;
            Use_R := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Nr));
            for Ob of Arm_Obs (B) loop
               Es.Append (abs Epi (Ob, S, R, T));
            end loop;
            Gate := Gate_Of (Es);
            for K in 0 .. Nr - 1 loop
               if Es (K) < Gate and then In_Sub (K) then
                  Use_R.Replace_Element (K, True); Nu := Nu + 1;
               end if;
            end loop;
            Enough := Nu >= Min_Inl;
         end Pick;
         Lm_Capped : Boolean := False;   --  哪一遍的 LM 做满保险的次数还在降(09-30,B 组的 Robust_LM 交出"收住没有")
         procedure Solve (Use_R : Bools) is
            Nu : Natural := 0;
         begin
            for U of Use_R loop
               if U then
                  Nu := Nu + 1;
               end if;
            end loop;
            declare
               N_Res : constant Natural := 2 * Nu + 3;
               Rv : constant V3 := Rot_Vec (R);
               X : Kinem.Vec (0 .. 6) := [Rv (0), Rv (1), Rv (2), T (0), T (1), T (2), Log (S)];
               Steps : constant Kinem.Vec (0 .. 6) := [others => 1.0e-7];   --  差分步(弧度 / 模型单位 / 对数倍数,极小量,无量纲)
               procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                  Rx : constant M3 := Rodrigues ([Xx (Xx'First), Xx (Xx'First + 1), Xx (Xx'First + 2)]);
                  Tx : constant V3 := [Xx (Xx'First + 3), Xx (Xx'First + 4), Xx (Xx'First + 5)];
                  Sx : constant Long_Float := Exp (Xx (Xx'First + 6));
                  J : Natural := Rr'First;
               begin
                  for K in 0 .. Nr - 1 loop
                     if Use_R (K) then
                        Res2 (Arm_Obs (B) (K), Wv (Arm_Obs (B) (K).Wk).Cam, Sx, Rx, Tx, Rr (J), Rr (J + 1)); J := J + 2;
                     end if;
                  end loop;
                  Plane_Res (B, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2));
               end Resid;
               Lm_Done : Boolean;   --  LM 收住了没有(False = 做满 100 次还在降)
            begin
               Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access, Lm_Done);
               if not Lm_Done then
                  Lm_Capped := True;
               end if;
               R := Rodrigues ([X (0), X (1), X (2)]); T := [X (3), X (4), X (5)]; S := Exp (X (6));
            end;
         end Solve;
         procedure Refine is new Until_Settled (Pick, Solve);
         Rounds : Natural;
         Verdict : Settle_Verdict;
      begin
         Refine (Rounds, Verdict);
         if Verdict = Cycled or else Verdict = Capped then
            Say ("  第" & Codec.Img (B + 1) & " 只手放进世界的精修:重挑重解了 " & Codec.Img (Rounds) & " 遍,门里的那一组"
                 & (if Verdict = Cycled then "又变回了前面某一遍的样子(来回转)" else "解满保险的遍数还在变") & " ⇒ 没定下来,交的是最后一遍的解");
         end if;
         if Lm_Capped then
            Say ("  第" & Codec.Img (B + 1) & " 只手放进世界的精修:有一遍的解做满保险的次数代价还在降 ⇒ 那一遍没解到底,照实交");
         end if;
      end;
      declare
         All_E, Es : Floats;
         package Sorting is new F64_Vectors.Generic_Sorting;
         Gate : Long_Float;
      begin
         for Ob of Arm_Obs (B) loop
            All_E.Append (Epi (Ob, S, R, T));
         end loop;
         Gate := Gate_Of (All_E);
         for E of All_E loop
            if E < Gate then
               Es.Append (E);
            end if;
         end loop;
         Inl := Natural (Es.Length);
         if not Es.Is_Empty then
            Sorting.Sort (Es);
            Md := Es (Natural (Es.Length) / 2);
         end if;
      end;
   end Place_Arm;

   --  全部放完以后一起精修:每只放进世界的别的手(相似变换 7 个数)、不长在手上的那只眼(位姿 6 个 + 焦距)用全部配上的点一起按像素解
   --  (抗野点,两轮门同上),加每只手"桌面重合"那 3 条。先放进世界的也被后放进的证据修正(按放进去的先后,前面的定下以后后面的就不再动它 ⇒ 错会被锁死)
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
      Cam_Sig : Long_Float := 1.0e-6;   --  不动的眼里配点的噪声 = 这一批往返差的中位(像素;同解它时的 Sh)
      --  全部残差(门里的):手的配点(第 B 只手的点投回世界里那只眼)、不动的眼的配点(世界里的手的点投进它)、每只手桌面 3 条
      --  各组配点的噪声倍数(方差分量,按这一组自己的残差估):第 B 只手配进手的格子 = 2B,配进不长在手上的眼 = 2B + 1;不动的眼的配点 = 2 Na
      Jf : array (0 .. 2 * Na) of Long_Float := [others => 1.0];
      --  每只手配点的噪声(Cross_Sig:往返差的中位)在精修里不变 ⇒ 先算好(原来每条观测都把整批排一遍序,第 2 只手配点 1306 条时一起精修 274 秒)
      Sig_Of : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];
      procedure All_Res (Xx : Kinem.Vec; Rr : out Kinem.Vec; Use_A : Bools; Use_C : Bools; Fill_E : Boolean; Ea, Ec : in out Floats; Ga : in out Ints) is
         J : Natural := Rr'First;
      begin
         Ea.Clear; Ec.Clear; Ga.Clear;
         for B in 1 .. Na - 1 loop
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
                        E1, E2 : Long_Float;
                        Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
                        Rc : constant M3 := Mul (Mul (Rx, Ps (B) (Ob.K).Cov), Tr (Rx));
                        Cw : M3;
                     begin
                        for I in 0 .. 2 loop
                           for Jj in 0 .. 2 loop
                              Cw (I, Jj) := Sx * Sx * Rc (I, Jj);
                           end loop;
                        end loop;
                        Proj_W (Cam_Now (Xx, Ob.Wk), [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)], Cw, Ob.U, Ob.V, Sig_Of (B), E1, E2);
                        declare
                           Gi : constant Natural := 2 * B + (if Wv (Ob.Wk).Arm < 0 then 1 else 0);
                        begin
                           E1 := E1 / Jf (Gi); E2 := E2 / Jf (Gi);
                           if Fill_E then
                              Ga.Append (Gi);
                           end if;
                        end;
                        if Fill_E then
                           Ea.Append (Sqrt (E1 * E1 + E2 * E2));
                        elsif Use_A (Natural (Ea.Length)) then
                           Rr (J) := E1; Rr (J + 1) := E2; J := J + 2;
                        end if;
                        if not Fill_E then
                           Ea.Append (0.0);
                        end if;
                     end;
                  end loop;
                  if not Fill_E then
                     Plane_Res (B, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2)); J := J + 3;
                  end if;
               end;
            end if;
         end loop;
         if Cam_Slot >= 0 and then Fx_K >= 0 then
            declare
               Cg : constant Cam_Geo := Cam_Now (Xx, Natural (Fx_K));
            begin
               for X of Cam_Obs loop
                  if X.Pa = 0 or else Arm_Slot (X.Pa) >= 0 then
                     declare
                        Sx : Long_Float;
                        Rx : M3;
                        Tx : V3;
                        E1, E2 : Long_Float;
                     begin
                        Map_Of (Xx, X.Pa, Sx, Rx, Tx);
                        declare
                           Rt : constant V3 := Ap (Rx, Ps (X.Pa) (X.Pk).X);
                           Rc : constant M3 := Mul (Mul (Rx, Ps (X.Pa) (X.Pk).Cov), Tr (Rx));
                           Cw : M3;
                        begin
                           for I in 0 .. 2 loop
                              for Jj in 0 .. 2 loop
                                 Cw (I, Jj) := Sx * Sx * Rc (I, Jj);
                              end loop;
                           end loop;
                           Proj_W (Cg, [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)], Cw, X.U, X.V, Cam_Sig, E1, E2);
                           E1 := E1 / Jf (2 * Na); E2 := E2 / Jf (2 * Na);
                        end;
                        if Fill_E then
                           Ec.Append (Sqrt (E1 * E1 + E2 * E2));
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
   begin
      for B in 1 .. Na - 1 loop
         Sig_Of (B) := Cross_Sig (B);
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
      end;
      for B in 1 .. Na - 1 loop
         if Placed (B) and then Worlds (B).Valid then
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
         for B in 1 .. Na - 1 loop
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
         --  每轮先按各组自己的残差重估噪声倍数,再挑门里的、解,做到门里的那一组(手的配点 + 不动的眼的配点)不再变(Until_Settled;原来固定三轮)
         declare
            Ua, Uc : Bools;
            Nua, Nuc, N_Planes : Natural := 0;
            procedure Pick (Both : out Bools; Enough : out Boolean) is
               Ea, Ec : Floats;
               Ga : Ints;
               Dummy : Kinem.Vec (0 .. 0);
            begin
               Ua.Clear; Uc.Clear; Nua := 0; Nuc := 0; N_Planes := 0;
               Jf := [others => 1.0];
               All_Res (X, Dummy, Bool_Vectors.Empty_Vector, Bool_Vectors.Empty_Vector, True, Ea, Ec, Ga);
               --  各组的噪声倍数 = 白化后二维残差长度的中位 ÷ √(2 ln 2)(单位方差时瑞利分布的中位,统计常数,无量纲)
               declare
                  package Sorting is new F64_Vectors.Generic_Sorting;
                  Per : array (0 .. 2 * Na) of Floats;
               begin
                  for K in 0 .. Natural (Ea.Length) - 1 loop
                     Per (Natural (Ga (K))).Append (Ea (K));
                  end loop;
                  for E of Ec loop
                     Per (2 * Na).Append (E);
                  end loop;
                  for G in Per'Range loop
                     if not Per (G).Is_Empty then
                        Sorting.Sort (Per (G));
                        Jf (G) := Long_Float'Max (Per (G) (Natural (Per (G).Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
                     end if;
                  end loop;
                  for K in 0 .. Natural (Ea.Length) - 1 loop
                     Ea.Replace_Element (K, Ea (K) / Jf (Natural (Ga (K))));
                  end loop;
                  for K in 0 .. Natural (Ec.Length) - 1 loop
                     Ec.Replace_Element (K, Ec (K) / Jf (2 * Na));
                  end loop;
               end;
               declare
                  Gta : constant Long_Float := Gate_Of (Ea);
                  Gtc : constant Long_Float := Gate_Of (Ec);
               begin
                  for E of Ea loop
                     Ua.Append (E < Gta);
                     if E < Gta then
                        Nua := Nua + 1;
                     end if;
                  end loop;
                  for E of Ec loop
                     Uc.Append (E < Gtc);
                     if E < Gtc then
                        Nuc := Nuc + 1;
                     end if;
                  end loop;
               end;
               for B in 1 .. Na - 1 loop
                  if Arm_Slot (B) >= 0 then
                     N_Planes := N_Planes + 1;
                  end if;
               end loop;
               Both := Bool_Vectors."&" (Ua, Uc);
               Enough := 2 * Nua + 2 * Nuc + 3 * N_Planes > Np;   --  残差条数(同 Solve 里的 N_Res)比参数多才解得了
            end Pick;
            Lm_Capped : Boolean := False;   --  哪一遍的 LM 做满保险的次数还在降
            procedure Solve (Both : Bools) is
               pragma Unreferenced (Both);   --  这一组 Pick 已经按手 / 不动的眼分开放在 Ua、Uc 里
            begin
               declare
                  N_Res : constant Natural := 2 * Nua + 2 * Nuc + 3 * N_Planes;
                  procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                     E1, E2 : Floats;
                     G2 : Ints;
                  begin
                     All_Res (Xx, Rr, Ua, Uc, False, E1, E2, G2);
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
         --  写回去
         for B in 1 .. Na - 1 loop
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
   if Worlds.Is_Empty or else not Worlds (0).Valid then
      return;
   end if;
   for A in 0 .. Na - 1 loop
      if Worlds (A).Valid then
         Tri_Pts (Worlds (A).Model, Ds (A), Css (A), Max_Pts, Ps (A), Sig_Px (A));
      end if;
   end loop;
   if Natural (Ps (0).Length) < Min_Inl then
      Say ("世界:第一只手只三角出 " & Codec.Img (Natural (Ps (0).Length)) & " 个点 ⇒ 定不了桌面");
      return;
   end if;
   --  世界的桌面:"上" = 桌面法向(朝第一只手的眼),原点 = 那只眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上
   declare
      Inl : Natural;
      Md : Long_Float;
   begin
      Plane_Of (Ps (0), Pl0, N0, Gates (0), Inl, Md);
      Pl0_Md := Md;
      if Dist ([0.0, 0.0, 0.0], Pl0, N0) < 0.0 then
         N0 := [-N0 (0), -N0 (1), -N0 (2)];
      end if;
      Pls (0) := Pl0; Nrs (0) := N0;
      declare
         Dn : constant Long_Float := N0 (0);
         Xr0 : constant V3 := [1.0 - Dn * N0 (0), -Dn * N0 (1), -Dn * N0 (2)];
         Xr : constant V3 := [Xr0 (0) / Norm (Xr0), Xr0 (1) / Norm (Xr0), Xr0 (2) / Norm (Xr0)];
         Yr : constant V3 := [N0 (1) * Xr (2) - N0 (2) * Xr (1), N0 (2) * Xr (0) - N0 (0) * Xr (2), N0 (0) * Xr (1) - N0 (1) * Xr (0)];
         Ph : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2);
      begin
         Rw := [[Xr (0), Xr (1), Xr (2)], [Yr (0), Yr (1), Yr (2)], [N0 (0), N0 (1), N0 (2)]];
         O := [Ph * N0 (0), Ph * N0 (1), Ph * N0 (2)];
         Say ("世界:第一只手三角出 " & Codec.Img (Natural (Ps (0).Length)) & " 个点,桌面拟合了 " & Codec.Img (Inl) & " 个(离面中位 " & Codec.Fmt (Md, 4)
              & " 单位)⇒ 上 = 桌面法向;眼离桌面 " & Codec.Fmt (abs Ph, 3) & " 单位(长度单位 = 第一只手的模型单位)");
      end;
   end;
   Ok := True;
   Placed (0) := True;
   for B in 1 .. Na - 1 loop
      if Worlds (B).Valid and then not Ps (B).Is_Empty then
         declare
            Inl : Natural;
            Md : Long_Float;
         begin
            Plane_Of (Ps (B), Pls (B), Nrs (B), Gates (B), Inl, Md);
            Plane_Info (B);
            Say ("世界:第" & Codec.Img (B + 1) & " 只手三角出 " & Codec.Img (Natural (Ps (B).Length)) & " 个点,它自己的桌面拟合了 " & Codec.Img (Inl) & " 个(离面中位 "
                 & Codec.Fmt (Md, 4) & ",算在面上的门 " & Codec.Fmt (Gates (B), 4) & " 单位)");
         end;
      end if;
   end loop;
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
         Say ("世界:配点仪器没给整体特征(" & To_String (Err) & ")⇒ 只有第一只手在世界里");
         D_Ids.Clear;
      end if;
   end;
   Add_Arm_Views (0);
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
                     Match_Pair (Vw.Id, Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
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
         for B in 1 .. Na - 1 loop
            if Worlds (B).Valid and then not Placed (B) and then not Ps (B).Is_Empty and then not D_Ids.Is_Empty then
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
               Wv.Append (World_View'(Id => Fx_Id, Cam => G, Arm => -1, Frame => 0, W => Ds (0).World_Img.W, H => Ds (0).World_Img.H));
               --  已经放进世界的别的手,也和这只新放进来的眼配一次(用它最像这只眼的一格;先后不同,证据不能不同)
               for P in 1 .. Na - 1 loop
                  if Placed (P) then
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
                           Match_Pair (Id_Of (P, Natural (Bf)), Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
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
               Say ("放进世界:不长在手上的那只眼 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,往返 1 px 内共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length))
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
                           Match_Pair (Vw.Id, Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
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
                    & " 对、对别的手的格子 " & Codec.Img (N_Arm) & " 对),往返 1 px 内配上 " & Codec.Img (Natural (Arm_Obs (Natural (Best)).Length))
                    & " 次,放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次(残差中位 " & Codec.Fmt (Md, 2) & " 倍标准差)⇒ 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
            end;
         end if;
      end;
   end loop;
   Say ("  一起精修之前 " & Lap);
   Joint_Refine;
   for B in 1 .. Na - 1 loop
      if Placed (B) then
         Say ("一起精修以后:第" & Codec.Img (B + 1) & " 只手 长度倍数 " & Codec.Fmt (Worlds (B).S, 4) & "、平移 (" & Codec.Fmt (Worlds (B).Ta (0), 3) & ", "
              & Codec.Fmt (Worlds (B).Ta (1), 3) & ", " & Codec.Fmt (Worlds (B).Ta (2), 3) & ") 单位");
      end if;
   end loop;
   if Fx_Placed then
      Say ("一起精修以后:不长在手上的那只眼 焦距 " & Codec.Fmt (G.F, 1) & "、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位");
   end if;
   for B in 1 .. Na - 1 loop
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
               if X.Pa < Na and then (X.Pa = 0 or else Placed (X.Pa)) and then X.Pk < Natural (Best (X.Pa).Length)
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
                     Sa : constant Long_Float := (if A = 0 then 1.0 else Worlds (A).S);
                     Ra : constant M3 := (if A = 0 then Identity else Worlds (A).Ra);
                     Ta : constant V3 := (if A = 0 then [0.0, 0.0, 0.0] else Worlds (A).Ta);
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
         for B in 1 .. Na - 1 loop
            if Placed (B) then
               Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/align_arm" & Codec.Img (B) & ".txt");
               Ada.Text_IO.Put (Fo, "S " & Codec.Fmt (Worlds (B).S, 9) & " R");
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     Ada.Text_IO.Put (Fo, " " & Codec.Fmt (Worlds (B).Ra (I, J), 9));
                  end loop;
               end loop;
               Ada.Text_IO.Put_Line (Fo, " T " & Codec.Fmt (Worlds (B).Ta (0), 9) & " " & Codec.Fmt (Worlds (B).Ta (1), 9) & " " & Codec.Fmt (Worlds (B).Ta (2), 9));
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
   Say ("  对齐用了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒 " & Lap);
end Align;
