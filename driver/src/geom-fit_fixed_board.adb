separate (Geom)
procedure Fit_Fixed_Board (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Rep : in out Fixed_Report; Ok : out Boolean; Start_Here : Boolean := False) is
   Fit_F : constant Boolean := G.F <= 0.0;
   Use_Prior : constant Boolean := Fit_F and then G.F_Prior > 0.0 and then G.F_Prior_Sd > 0.0;
   Ns : constant Natural := Natural (Scene.Length);
   Min_Pts : constant := 4;   --  单点法至少几个点(次数)
   Gi : Cam_Geo := G;
   Ws : array (0 .. Natural'Max (1, Ns) - 1) of Long_Float := [others => 1.0];   --  每个点的权 = 1 / 它在这只眼里的每轴像素噪声
   Skip : Flags (0 .. Ns - 1) := [others => False];   --  被判离群、不再进解的点
   Behind : Natural := 0;   --  最近一次算残差时跑到相机后面的点数
   K_Kept : Boolean := False;   --  镜头畸变进了解(F 检验显著)
begin
   Ok := False;
   Rep.Scene_N := Ns; Rep.Scene_Used := 0; Rep.Scene_Rms := 0.0; Refits := 0;
   if Ns < Min_Pts then
      Why := To_Unbounded_String ("标定板不到 4 个点(" & Codec.Img (Ns) & ")");
      return;
   end if;
   --  ① 起点:单点法(盲搜 + 精修),板上的点世界位置已知;Start_Here 就从 G 现在的位姿起步
   if Start_Here then
      if not (G.Valid and then G.Fixed) then
         Why := To_Unbounded_String ("要从现在的位姿起步,可它还没有位姿");
         return;
      end if;
   else
      declare
         Marks : Mark_Vectors.Vector;
         Fok : Boolean;
      begin
         for S of Scene loop
            Marks.Append (Mark'(Pw => S.Pw, U => S.U, V => S.V));
         end loop;
         Fit_Fixed (Gi, Marks, Fok);
         if not Fok then
            Why := To_Unbounded_String ("标定板的点单独解不出(" & Codec.Img (Ns) & " 个)");
            return;
         end if;
      end;
   end if;
   --  ② 权:按起点处的相机算每个点在这只眼里的每轴像素方差(配点噪声² + 三角的协方差投进来)
   for I in 0 .. Ns - 1 loop
      declare
         Var : constant Long_Float := Scene_Var (Gi, Scene (I));
      begin
         if Var > 0.0 then
            Ws (I) := 1.0 / Sqrt (Var);
         end if;
      end;
   end loop;
   --  ③ 加权精修:朝向 3 + 位置 3 (+ 焦距 + 镜头畸变 K1 / K2:内参没给就一起解,和焦距同进同出 —— 给了焦距的(装回的、核对时)内参照旧不动)
   declare
      Np : constant Natural := (if Fit_F then 9 else 6);
      P : Param_Vec (0 .. Np - 1) := [others => 0.0];
      Steps : Param_Vec (0 .. Np - 1) := [others => 1.0e-4];   --  差分步(弧度 / 米,极小量)
      Rv : constant V3 := Rot_Vec (Gi.R_Ce);
      Nr : Natural := 0;
      Cur : Long_Float := 0.0;
      function Cam_Of (P : Param_Vec) return Cam_Geo is
         Gt : Cam_Geo := G;
      begin
         Gt.R_Ce := Rodrigues ([P (0), P (1), P (2)]);
         Gt.Pos := [P (3), P (4), P (5)];
         if Fit_F and then P'Last >= 6 then
            Gt.F := P (6);
         end if;
         if Fit_F and then P'Last >= 8 then
            Gt.K1 := P (7); Gt.K2 := P (8);   --  不带畸变那一份(P 只到 6)按 G 的,针孔
         end if;
         return Gt;
      end Cam_Of;
      procedure Resid (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float)) is
         Gt : constant Cam_Geo := Cam_Of (P);
         Sum : Long_Float := 0.0;
         I : Natural := 0;
         procedure Put (Du, Dv : Long_Float) is
         begin
            Sum := Sum + Du * Du + Dv * Dv;
            if Fill /= null then
               Fill (I, Du, Dv);
            end if;
            I := I + 1;
         end Put;
      begin
         Behind := 0;
         for S in 0 .. Ns - 1 loop
            if not Skip (S) then
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                  if Front and then Gt.F > 0.0 then
                     Put (Ws (S) * (U - Scene (S).U), Ws (S) * (V - Scene (S).V));
                  else
                     Put (1.0e3, 1.0e3);   --  跑到相机后面:远大于任何一条加权残差的罚(无量纲哨兵)
                     Behind := Behind + 1;
                  end if;
               end;
            end if;
         end loop;
         if Use_Prior then
            Put ((Gt.F - G.F_Prior) / G.F_Prior_Sd, 0.0);   --  先验那一条残差(以不确定度为单位,无量纲)
         end if;
         R := Sqrt (Sum / Long_Float (Natural'Max (1, I)));
      end Resid;
      --  进解的点按像素算的残差(记账、报原因用;权只管解,不管报数)
      procedure Px_Rms (P : Param_Vec; R : out Long_Float; K : out Natural) is
         Gt : constant Cam_Geo := Cam_Of (P);
         Sum : Long_Float := 0.0;
      begin
         K := 0;
         for S in 0 .. Ns - 1 loop
            if not Skip (S) then
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                  if Front then
                     Sum := Sum + (U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2;
                     K := K + 1;
                  end if;
               end;
            end if;
         end loop;
         R := (if K > 0 then Sqrt (Sum / Long_Float (K)) else 0.0);
      end Px_Rms;
      function Px_Note (P : Param_Vec) return String is
         R : Long_Float;
         K : Natural;
      begin
         Px_Rms (P, R, K);
         return "像素残差 " & Codec.Fmt (R, 2) & " px(" & Codec.Img (K) & " 个点)";
      end Px_Note;
   begin
      P (0) := Rv (0); P (1) := Rv (1); P (2) := Rv (2);
      P (3) := Gi.Pos (0); P (4) := Gi.Pos (1); P (5) := Gi.Pos (2);
      if Fit_F then
         P (6) := Gi.F; Steps (6) := 1.0;   --  焦距的差分步(像素,极小量)
         P (7) := Gi.K1; P (8) := Gi.K2;    --  畸变从单点法的起点起(针孔 = 0);差分步同朝向(无量纲,极小量)
      end if;
      Nr := Ns + (if Use_Prior then 1 else 0);
      if Start_Here then
         --  起点就在真值附近:门从粗到细 —— 先放画幅宽的 1/16(比例,无量纲;挡住的那片配成乱的,乱点散在几百像素里,门外),
         --  解一次,再收到"门内这些点自己的像素离散"的 3 倍(倍数无量纲),收到挑出来的那一批不再变为止(09-30 以前固定来回三遍,
         --  第三遍挑出来的还在变也照样交);遍数到了点数还在变 ⇒ 解不出,照实报。门的尺度是这一次配点自己量的:
         --  转过、挡过的画面配得比标定时粗得多,按标定时的噪声挑,好点也全挑没了(X5B 2026-09-25)
         declare
            Gate : Long_Float := 0.125 * G.Cx;   --  半幅宽的八分之一 = 画幅宽的 1/16(比例,无量纲)
         begin
            loop
               declare
                  Gt : constant Cam_Geo := Cam_Of (P);
                  Dropped : Natural := 0;
                  Sum : Long_Float := 0.0;
                  Changed : Boolean := False;
               begin
                  for S in 0 .. Ns - 1 loop
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                        E : Long_Float;
                     begin
                        Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                        E := (if Front then Sqrt ((U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2) else Long_Float'Last);
                        Changed := Changed or else (E > Gate) /= Skip (S);
                        Skip (S) := E > Gate;
                        if Skip (S) then
                           Dropped := Dropped + 1;
                        else
                           Sum := Sum + E * E;
                        end if;
                     end;
                  end loop;
                  if Ns - Dropped < Min_Pts then
                     Why := To_Unbounded_String ("从现在的位姿起步,门 " & Codec.Fmt (Gate, 1) & " px 内的点不到 4 个(" & Codec.Img (Ns - Dropped) & "/" & Codec.Img (Ns) & ")");
                     return;
                  end if;
                  exit when Refits > 0 and then not Changed;   --  收过的门挑出来的还是上一遍解的那一批 ⇒ 定了
                  if Refits >= Ns then
                     Why := To_Unbounded_String ("从现在的位姿起步,门从粗到细收了 " & Codec.Img (Refits)
                                                 & " 遍,挑出来的那一批还在变(遍数上限 = 点数,只当保险)");
                     return;
                  end if;
                  Nr := Ns - Dropped + (if Use_Prior then 1 else 0);
                  Resid (P, Cur, null);
                  LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
                  Gate := 3.0 * Sqrt (Sum / Long_Float (Ns - Dropped));
                  Refits := Refits + 1;
               end;
            end loop;
         end;
      end if;
      Resid (P, Cur, null);
      LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
      --  离群的点按 Reselect_Loop 踢(同 Fit_Rig 那一套;从现位姿起步的已经按门挑过了):加权残差比进解的那些的中位大 3 倍的不要,
      --  每遍都从全体重挑(先前踢错的能回来)、重解,踢到进解的那一批不再变为止(09-30 以前固定三遍)。
      --  不设"踢的不到四分之一才算":板上天然混着一小撮远处、桌下配错的点,它们在各停里错得一样、交叉核不出来,
      --  一次最小二乘就被它们拽走(G2E 离线:327 个点里 14 个配错,不踢 ⇒ 焦距 −5%、位置差 3 cm;连踢三遍 ⇒ 14 个全踢掉);
      --  进解的不到一半 ⇒ 是整个解不对(Broken),剩下不到 4 个点 ⇒ 解不出,都照实报
      if not Start_Here then
         declare
            Sk : Flags (Skip'Range) := Skip;
            Kept, Rounds : Natural;
            How : Reselect_End;
            procedure Errs (Rs : out Param_Vec) is
               Gt : constant Cam_Geo := Cam_Of (P);
            begin
               for S in Rs'Range loop
                  declare
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Project_Fixed (Gt, Scene (S).Pw, U, V, Front);
                     Rs (S) := (if Front then Ws (S) * Sqrt ((U - Scene (S).U) ** 2 + (V - Scene (S).V) ** 2) else Long_Float'Last);
                  end;
               end loop;
            end Errs;
            procedure Solve (S : Flags) is
            begin
               Skip := S;
               Nr := (if Use_Prior then 1 else 0);   --  先验那一槽
               for K in S'Range loop
                  if not S (K) then
                     Nr := Nr + 1;
                  end if;
               end loop;
               Resid (P, Cur, null);
               LM_Refine (P, Nr, Steps, 100, Resid'Access, Cur);   --  100 = 迭代次数上限(次数)
            end Solve;
         begin
            Reselect_Loop (Errs'Access, Solve'Access, Sk, Kept, Rounds, How);
            Refits := Rounds;
            if How = Broken then
               Why := To_Unbounded_String ("踢离群的点踢到只剩 " & Codec.Img (Kept) & " / " & Codec.Img (Ns) & " 个:门外的比一半还多,不是离群,是整个解不对");
               return;
            elsif How = Stuck then
               Why := To_Unbounded_String ("踢离群的点重解了 " & Codec.Img (Rounds) & " 遍,进解的那一批还在变(遍数上限 = 点数,只当保险)");
               return;
            elsif Kept < Min_Pts then
               Why := To_Unbounded_String ("踢离群的点踢到只剩 " & Codec.Img (Kept) & " 个,比单点法要的点数还少");
               return;
            end if;
            Skip := Sk;
         end;
      end if;
      G.Dropped := 0;
      for S of Skip loop
         if S then
            G.Dropped := G.Dropped + 1;
         end if;
      end loop;
      --  ④ 畸变要不要(内参一起解时):同一批进解的点,不带畸变(K1 = K2 = 0,针孔)再解一次,两份的残差平方和做嵌套模型的 F 检验
      --  (Two_More_Significant,多出的两个数)。不显著 ⇒ 用针孔那一份 —— 和以前不解畸变时解出来的一样;显著 ⇒ 带畸变那一份。
      --  K1 / K2 的不确定度两样都从带畸变那一份算(不显著时 K1 = K2 = 0 落在它们自己的不确定度以内)
      if Fit_F then
         declare
            P7 : Param_Vec (0 .. 6) := P (0 .. 6);
            Cur9, Cur7 : Long_Float;
            Kept_Slots : constant Natural := Nr;
            N_Eq : constant Natural := 2 * Kept_Slots - (if Use_Prior then 1 else 0);
            Sd9 : Param_Vec (0 .. Np - 1);
         begin
            Resid (P, Cur9, null);
            Param_Sd (P, Nr, Use_Prior, Steps, Resid'Access, Sd9);
            Resid (P7, Cur7, null);
            LM_Refine (P7, Nr, Steps (0 .. 6), 100, Resid'Access, Cur7);   --  100 = 迭代次数上限(次数)
            K_Kept := N_Eq > Np and then Two_More_Significant (Cur7 ** 2 * Long_Float (Kept_Slots), Cur9 ** 2 * Long_Float (Kept_Slots), N_Eq - Np);
            G.K1_Sd := Sd9 (7); Rep.K2_Sd := Sd9 (8); Rep.K_Kept := K_Kept;
            if not K_Kept then
               P (0 .. 6) := P7; P (7) := 0.0; P (8) := 0.0;
            end if;
         end;
      end if;
      Resid (P, Cur, null);
      if Behind > 0 then
         Why := To_Unbounded_String ("解出来还有 " & Codec.Img (Behind) & " 个板上的点跑到相机后面(" & Px_Note (P) & ")");
         return;   --  不是解,不存
      end if;
      --  同 Fit_Rig 那一条:无畸变针孔的视场不超过 120°(半幅宽 ÷ 焦距 ≤ tan 60° = 1.732,无量纲)——焦距比这还短就不是针孔的解
      --  (X5A 2026-09-24:头顶眼粗解成焦距 140、残差 21 px,驱动照着它把手往错的方向送了 25 cm)
      if Fit_F and then G.Cx > 1.732 * P (6) then
         Why := To_Unbounded_String ("焦距解成 " & Codec.Fmt (P (6), 1) & " px,视场超过 120°,针孔假设不成立(" & Px_Note (P) & ")");
         return;
      end if;
      declare
         Sd : Param_Vec (0 .. Np - 1);
         Lo : V3 := [others => Long_Float'Last];
         Hi : V3 := [others => Long_Float'First];
         Span : Long_Float := 0.0;   --  板上的点在世界里铺开的量程(米)
      begin
         for S in 0 .. Ns - 1 loop
            if not Skip (S) then
               for I in 0 .. 2 loop
                  Lo (I) := Long_Float'Min (Lo (I), Scene (S).Pw (I)); Hi (I) := Long_Float'Max (Hi (I), Scene (S).Pw (I));
               end loop;
            end if;
         end loop;
         Span := Sqrt ((Hi (0) - Lo (0)) ** 2 + (Hi (1) - Lo (1)) ** 2 + (Hi (2) - Lo (2)) ** 2);
         if Fit_F and then not K_Kept then
            declare
               Sd7 : Param_Vec (0 .. 6);
            begin
               Param_Sd (P (0 .. 6), Nr, Use_Prior, Steps (0 .. 6), Resid'Access, Sd7);   --  针孔那一份的不确定度(畸变没进解)
               Sd (0 .. 6) := Sd7; Sd (7) := 0.0; Sd (8) := 0.0;
            end;
         else
            Param_Sd (P, Nr, Use_Prior, Steps, Resid'Access, Sd);
         end if;
         G.Rot_Sd := Sqrt (Sd (0) ** 2 + Sd (1) ** 2 + Sd (2) ** 2);
         G.Pos_Sd := Sqrt (Sd (3) ** 2 + Sd (4) ** 2 + Sd (5) ** 2);
         G.F_Sd := (if Fit_F then Sd (6) else 0.0);
         --  位置的不确定度比板铺开的量程还大、或焦距的不确定度比焦距还大、或朝向定不住(Pointing_Lost,同 Fit_Rig)= 方程分不开 ⇒ 不算解出来
         --  (V1I / G1K 2026-09-24:相机解到 2.8 m / 120 m 外、残差却只有零点几像素,就是这种"解")。
         --  报原因时印这次用的焦距:解的带 ±,给的照实说"给的"(09-30 以前不解焦距时印的是 P (5) = 相机位置的 z)
         declare
            Fv : constant Long_Float := (if Fit_F then P (6) else G.F);
            F_Say : constant String := (if Fit_F then "焦距 " & Codec.Fmt (P (6), 1) & " ± " & Codec.Fmt (G.F_Sd, 1) & " px"
                                        else "焦距 " & Codec.Fmt (G.F, 1) & " px(给的,不解)");
         begin
            if G.Pos_Sd >= Span or else (Fit_F and then G.F_Sd >= P (6)) or else Pointing_Lost (G, Fv, G.Rot_Sd) then
               Why := To_Unbounded_String ("不确定度比量本身还大:位置 ± " & Codec.Fmt (G.Pos_Sd, 3) & " 单位(板铺开 " & Codec.Fmt (Span, 3) & " 单位)," & F_Say
                                           & ",朝向 ± " & Codec.Fmt (G.Rot_Sd, 3) & " rad(让投影挪 " & Codec.Fmt (Fv * G.Rot_Sd, 1) & " px,半幅对角线 "
                                           & Codec.Fmt (Sqrt (G.Cx ** 2 + G.Cy ** 2), 1) & " px)(" & Px_Note (P) & ")");
               return;
            end if;
         end;
      end;
      --  解出来的镜头模型在画幅里不许折回:画幅四个角的像素都得去得了畸变(Cam_Dir 的 Ok)—— 折回半径落在画幅里,
      --  外面那一圈的像素没有视线,这个"解"只是拿畸变去拟合别的错
      if Fit_F and then K_Kept then
         declare
            Gt : constant Cam_Geo := Cam_Of (P);
            Ok1, Ok2, Ok3, Ok4 : Boolean;
            D1 : constant V3 := Cam_Dir (Gt, 0.0, 0.0, Ok1);
            D2 : constant V3 := Cam_Dir (Gt, 2.0 * G.Cx, 0.0, Ok2);
            D3 : constant V3 := Cam_Dir (Gt, 0.0, 2.0 * G.Cy, Ok3);
            D4 : constant V3 := Cam_Dir (Gt, 2.0 * G.Cx, 2.0 * G.Cy, Ok4);
            pragma Unreferenced (D1, D2, D3, D4);
         begin
            if not (Ok1 and then Ok2 and then Ok3 and then Ok4) then
               Why := To_Unbounded_String ("解出来的镜头畸变 K1 " & Codec.Fmt (P (7), 4) & "、K2 " & Codec.Fmt (P (8), 4)
                                           & " 在画幅里就折回了(角上的像素去不了畸变)(" & Px_Note (P) & ")");
               return;
            end if;
         end;
      end if;
      Why := Null_Unbounded_String;
      Px_Rms (P, Rep.Scene_Rms, Rep.Scene_Used);
      G := Cam_Of (P);
      G.F_Meas := (if Fit_F then P (6) else 0.0);
      G.Rms := Rep.Scene_Rms;
      G.Fixed := True;
      G.Valid := True;
      Ok := True;
   end;
end Fit_Fixed_Board;
