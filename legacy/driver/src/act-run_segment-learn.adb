separate (Act.Run_Segment)
procedure Learn is
   All_Verified : Boolean := True;
   Any_Wrong : Boolean := False;
   --  🔴 这一步有没有哪个被跟的点是【跟丢的】(位置是按身体图猜的,不是看见的)
   Blind_Now : Boolean := False;
begin
   for P of Pts loop
      if P.Lost then
         Blind_Now := True;
      end if;
   end loop;
   Note.Err_Now := 0.0;
   Note.Raw_Now := 0.0;
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         P : constant Point := Pts (I);
         W0 : constant Point := Was (I);
         Dy : Table.Vec3;
         E : Table.Effect := Effs (I);
      begin
         if P.Lost then
            All_Verified := False;
            Any_Wrong := True;
         else
            --  修表只用眼睛量到的(光流核对值或抖认到的),按图猜的位置不喂回表
            Dy (0) := (if P.Has_Meas then P.Meas_U else P.Cu) - W0.Cu;
            Dy (1) := (if P.Has_Meas then P.Meas_V else P.Cv) - W0.Cv;
            Dy (2) := (if P.Has_Meas then (if P.Meas_Z > 0.0 and then W0.Z > 0.0 then P.Meas_Z - W0.Z else 0.0)
                       elsif P.Z > 0.0 and then W0.Z > 0.0 then P.Z - W0.Z else 0.0);
            --  🔴🔴 尺子:我这一步真挪了多少米(胳膊自己知道)+ 这一块游了多少画幅
            --  ⇒ 它有多近。不碰深度图。挪不够/游不够就不出数。
            declare
               Moved : Long_Float := 0.0;
               Ran : constant Long_Float := Sqrt (Dy (0) ** 2 + Dy (1) ** 2);
            begin
               for K in 0 .. 2 loop
                  Moved := Moved + (F.EE (Arm) (K) - Was_EE (K)) ** 2;
               end loop;
               Moved := Sqrt (Moved);
               declare
                  Nn : constant Long_Float :=
                    Near_From_Motion (Ran, Moved, Fl.Track, C.Map.EE_Noise);
                  Q : Point := Pts (I);
               begin
                  if Nn > 0.0 then
                     Q.Near := Nn;
                     Q.Near_N := Q.Near_N + 1;
                     Pts.Replace_Element (I, Q);
                  end if;
               end;
            end;
            --  同样的地板:没过就当没变(不然把量化噪声学进表里,符号可能是反的)
            Dy (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Size_Floor (Cw)
                       then P.Size - W0.Size else 0.0);
            Dy (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Ang_Floor (W0, Cw, Ch)
                       then Wrap (P.Ang - W0.Ang) else 0.0);
            Table.Update (E, Note.Got, Dy, Fl.Track * 2.0, Long_Float'Max (C.Map.EE_Noise, 0.5 * Note.Floor_Cmd));
            if Table.Blocked (E) then
               Note.Blocked := True;
            end if;
            if not (E.Null_Res > Fl.Track * 2.0 and then E.Free_Res < E.Null_Res) then
               All_Verified := False;
            end if;
            if E.Free_Res > Fl.Track * 2.0 and then E.Free_Res >= E.Null_Res then
               Any_Wrong := True;
            end if;
         end if;
         Effs (I) := E;
         Note.Err_Now := Note.Err_Now + P.Steps_Err;
         Note.Raw_Now := Note.Raw_Now + P.Raw_Err;
      end;
   end loop;
   --  整步没照做:各通道按自己的探针幅度归一后,实到与命令差过一半(逐个通道判会被同量级的小出入触发)
   if not Note.Halted then
      declare
         Dn, An, Gn : Long_Float := 0.0;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
            begin
               Dn := Dn + ((Note.Got (K) - Note.Cmd (K)) / Am) ** 2;
               An := An + (Note.Cmd (K) / Am) ** 2;
               Gn := Gn + (Note.Got (K) / Am) ** 2;
            end;
         end loop;
         Dn := Sqrt (Dn); An := Sqrt (An); Gn := Sqrt (Gn);
         --  🔴 撤回要放在【每一步都会走到】的地方。上一版我把它塞进了"没照做 ⇒ 步子太小"
         --  那个嵌套分支里 —— 那条路只在这一步没走成时才走到,于是一根【恢复正常、步步交付】的
         --  通道永远碰不到它,那句假话就一直留在自述里。判"它死了"看的是单独这一根,
         --  收回也该只看单独这一根:这一步它交付得动 ⇒ 那句话此刻是假的 ⇒ 撤掉。
         for K in 0 .. Chan.Per_Arm - 1 loop
            if abs Note.Got (K) > Long_Float (Fl.Delivery) then
               Cn_Recovered (C, Arm * Chan.Per_Arm + K);
            end if;
         end loop;
         --  🔴 以前这里写 An > 1.0 ⇒ 命令比一次探针幅度小就【一个字都不报】。
         --  GM 实测:连着 60 步命令 0.004、实到精确 0.0000,脑什么都没听到。任何非零命令都要判。
         if An > 0.0 and then Dn > 0.5 * An then
            Note.Not_Followed := True;
            --  🔴 "没照做"有两种完全相反的情形,以前一律【缩】步幅,于是越缩越动不了:
            --  缩到地板 1.0 时命令只剩 0.003 弧度,关节压根不转,而步幅只有"走成了才加倍"这一条回头路
            --  ⇒ 永久锁死(GM:三段命令三次一步 timeout,手一个像素没挪)。
            --  分开判,不用新系数:实到比"命令与实到之差"还小 = 几乎没动 ⇒ 步子太小,加倍;
            --  实到不小但对不上 = 动过头/动错了 ⇒ 缩。加倍这一条和开机探针是同一条规矩。
            if Gn < Dn then
               --  🔴🔴 加倍只治"步子太小"。治不了【顶死】—— 顶死的方向上,62 倍的零还是零。
               --  GN/GO/GP/GQ 四炮同一个终局:胳膊推进一个出不来的姿势,正反两个方向命令都交付 0,
               --  而步幅已经被加到 ×62。加力是错的解药,该做的是【换个走法】。
               --  这一具身体不知道自己的关节限位(零假设),它只能量:这一根被命令了、实到却落在
               --  自己的噪声地板里 ⇒ 此刻它推不动 ⇒ 这一步把它摘掉,让解算拿剩下的自由度绕过去。
               --  地板是量出来的(Fl.Delivery = 本体报的"实到"抖多少),不是人拍的。
               declare
                  Stuck : Natural := 0;
               begin
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if Note.Active (K)
                       and then abs Note.Cmd (K) > Long_Float (Fl.Delivery)
                       and then abs Note.Got (K) <= Long_Float (Fl.Delivery)
                     then
                        Note.Active (K) := False;   --  这一根此刻推不动,绕过它
                        Stuck := Stuck + 1;
                        --  🔴 自述那一条通道的"我变了":这一根以前听话、现在不听话,是关于【我自己】的变化,
                        --  不是这一段任务的事 ⇒ 它该被记住并讲出来,而不是修完这一步就忘。
                        if Cn_Changed (C, Arm * Chan.Per_Arm + K) then
                           Append (C.Changed_Say,
                                   "  I HAVE CHANGED: channel " & Codec.Img (Arm * Chan.Per_Arm + K)
                                   & " used to move when I commanded it and now it does not - "
                                   & "I commanded it and my body delivered nothing." & ASCII.LF);
                        end if;
                     elsif Note.Active (K) then
                        Reach (K) := Long_Float'Min (Reach (K) * 2.0,
                                                     Track_Win / Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
                     end if;
                  end loop;
                  if Stuck > 0 then
                     Put_Line ("[身]     顶死了:" & Codec.Img (Stuck) & " 根通道命令了而实到落在噪声里 ⇒ 这一步不用它们,换剩下的自由度绕过去");
                     C.Blind_Say := S ("some of the ways I can move are jammed right now - I commanded them and my body "
                                       & "did not move at all - so I dropped those and went around with the ways that still work");
                  else
                     Put_Line ("[身]     命令了几乎没动:实到只有命令的 " & Codec.Fmt (Gn / Long_Float'Max (1.0e-9, An) * 100.0, 0) & "% ⇒ 步幅加倍再试");
                     C.Blind_Say := S ("I commanded a push and my body barely moved at all, so I doubled the step and kept going");
                  end if;
               end;
            else
               Any_Wrong := True; All_Verified := False;
               for K in 0 .. Chan.Per_Arm - 1 loop
                  if Note.Active (K) then
                     --  🔴 油门只许往下踩,但【踩得死不了】:以前这里夹在 1.0,于是表被证明不准的时候身体一步也慢不下来。
                     --  老版"只会减速、没有底线"会一路减到零卡死 —— 那才是当初的 bug;现在底线在 Push_Cap 里
                     --  (身体噪声的两倍 / 这个通道自己量到的死区),所以减得下去、踩不死。
                     Reach (K) := Reach (K) * 0.5;
                  end if;
               end loop;
               Put_Line ("[身]     整步没照做:要走的和实际走的差了 " & Codec.Fmt (Dn / Long_Float'Max (1.0e-9, An) * 100.0, 0) & "% ⇒ 步幅缩回上一档");
            end if;
         end if;
      end;
   end if;
   --  🔴 步幅只认一件事:这一步【表说会挪多少】和【实际挪了多少】对不对得上。
   --  对得上 ⇒ 这几个用到的通道可以把步子放大一倍;差过一半 ⇒ 立刻缩回去。
   --  不管是平移还是转腕,都得先证明自己说话算数才有资格迈大步(FD/FE:转腕说了不算,一转球就更远)
   declare
      Pred_Ok : Boolean := True;
      Any_Meas : Boolean := False;
   begin
      for I in 0 .. Natural (Pts.Length) - 1 loop
         if not Pts (I).Lost then
            declare
               W0 : constant Point := Was (I);
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
               Act_U : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
               Act_V : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
               Pred : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
               Act : constant Long_Float := Sqrt (Act_U ** 2 + Act_V ** 2);
            begin
               if Pred > Fl.Track * 2.0 or else Act > Fl.Track * 2.0 then
                  Any_Meas := True;
                  --  差过预测的一半就算说了不算;再给一个和跟踪精度挂钩的绝对宽容(四分之一个跟踪窗),
                  --  否则步子越小相对误差越大,永远判"说了不算",步子就永远放不大(FF 实测每步都判不准)
                  if abs (Act - Pred) > 0.5 * Pred + Track_Win * 0.25 then
                     Pred_Ok := False;
                  end if;
               end if;
            end;
         end if;
      end loop;
      for K in 0 .. Chan.Per_Arm - 1 loop
         if Note.Active (K) and then abs Note.Cmd (K) > Long_Float'Max (Note.Floor_Cmd, C.Map.EE_Noise) then
            if Any_Meas and then Pred_Ok and then (not Note.Halted) and then not Note.Not_Followed then
               Reach (K) := Long_Float'Min (Reach (K) * 2.0, Track_Win / Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
            elsif Any_Meas and then not Pred_Ok then
               --  同上:油门踩得下去,底线在 Push_Cap 里
               Reach (K) := Reach (K) * 0.5;
            end if;
         end if;
      end loop;
      if Any_Meas and then not Pred_Ok then
         Put_Line ("[身]     表说了不算:预测挪的和实际挪的差过一半 ⇒ 用到的通道步子缩回去");
      end if;
      --  这张表这一步准到什么程度 ⇒ 下一步走它算出来的多大比例(准 = 走满,差一半 = 走一半)
      if Any_Meas then
         declare
            Worst_Rel : Long_Float := 0.0;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               if not Pts (I).Lost then
                  declare
                     W0 : constant Point := Was (I);
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
                     Au : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
                     Av : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
                     Pd : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
                     Ac : constant Long_Float := Sqrt (Au ** 2 + Av ** 2);
                  begin
                     Worst_Rel := Long_Float'Max (Worst_Rel, abs (Ac - Pd) / Long_Float'Max (Pd, Track_Win * 0.25));
                  end;
               end if;
            end loop;
            Trust := 0.5 * Trust + 0.5 / (1.0 + Worst_Rel);
         end;
      end if;
   end;
   for I in 0 .. Natural (Pts.Length) - 1 loop
      Store_Effect (C, Arm, Pts (I).Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trusts (I), Reach);
   end loop;
   --  碰到 = 我没在推的东西自己动了(跟着这只手动的相机里满画面都在动,分不出来 ⇒ 不下结论)
   --  🔴🔴 还要一条:【我看得见】才谈得上碰到(IG 2026-09-15 实测)。
   --  身体自己的原话:"I could not see 2 of 2 of the points I am tracking; I am going on where my
   --  body map says they are" —— 两个点全跟丢、位置全靠身体图猜,然后宣布"碰上了"。
   --  看图证实:机械手在画面右下角,球在桌心,中间隔着大半张桌子。
   --  瞎着的时候不许宣布碰到 —— 这不是保守,是"碰到"这个词在没有观测时根本没有内容。
   if not Own_Cam and then not Was_Regs.Is_Empty and then not Blind_Now then
      declare
         Now_Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
      begin
         for R of Now_Regs loop
            declare
               Mine : Boolean := False;
               Found_Prev : Boolean := False;
               Best : Long_Float := 0.0;
            begin
               for P of Pts loop
                  if Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Long_Float'Max (P.Box_W, P.Box_H) then
                     Mine := True;
                  end if;
               end loop;
               if not Mine then
                  for Q of Was_Regs loop
                     if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                        declare
                           D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                        begin
                           if not Found_Prev or else D < Best then
                              Best := D; Found_Prev := True;
                           end if;
                        end;
                     end if;
                  end loop;
                  --  🔴🔴 "碰到"有【两个入口】,今晚加的三条旁证只挡住了另一个
                  --  (有身份的那条:跟着的点动了)。这一条是重切斑点、按大小差不到三倍去配对,
                  --  **斑点没有身份**,分割抖一下就配出"挪了两个跟踪地板"。
                  --  JB 2026-09-15 实测:`until touched` 第 3 推成立,而画面里手整条收回自己底座、
                  --  球在桌心一动没动。⇒ 补上和另一条一样的两条旁证:
                  --    ① 我这一步【真送出去了一推】(实到超过本体交付噪声);
                  --    ② 它得【贴着我】—— 离我最近那一块不超过那一块自己的大小。
                  --  两个都是量出来的,零系数。
                  declare
                     Pushed : constant Boolean :=
                       Table.Norm (Note.Got, Chan.Per_Arm) > Long_Float (Fl.Delivery);
                     Near_Me : Boolean := False;
                  begin
                     for P of Pts loop
                        if P.Kind /= Thing_Pt
                          and then Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2)
                                   <= Long_Float'Max (Track_Win,
                                                      Long_Float'Max (P.Box_W, P.Box_H))
                        then
                           Near_Me := True;
                        end if;
                     end loop;
                     if Found_Prev and then Best > Fl.Track * 2.0
                       and then Pushed and then Near_Me
                     then
                        Note.Touched := True;
                     end if;
                  end;
               end if;
            end;
         end loop;
      end;
   end if;
   --  🔴 "碰到"也要从【被跟住的那几块】上判,不能只靠切块比对。
   --  不跟着这只手动的那台相机里常常一块都切不出来 ⇒ 那条判据永远不响 ⇒ 脑说的"走到碰到为止"
   --  变成一句空话,身体只会一路走到步数上限(IL/IM 实测:手压到球上、差距 0.003,
   --  60 步里一次 contact 都没响过)。而被脑点名跟住的那个东西【本来就在跟着】:
   --  在不跟着这只手动的相机里,它自己动了就只能是被碰了。
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         Not_Mine : constant Boolean := Cam_Arm (C, Pts (I).Cam) /= Integer (Arm);
      begin
         --  🔴 门槛用【这块东西自己有多大】,不是跟踪噪声地板。
         --  用地板当门槛 ⇒ 噪声天天越过它 ⇒ HN 实测:手离球 0.9 m,每一推都报"碰上了",
         --  于是 `until touched` 每段只走一推就结束,永远走不到球跟前。
         --  一个东西被撞得挪了【自己一个身位】,那才是真碰上了;零系数,尺寸是量出来的。
         --  🔴🔴 还要一条旁证:【我这一步真动过】。我一动不动就不可能碰到任何东西 ——
         --  东西在画面里跳了一大格,多半是我把它认成了另一块(切块每帧重切,块数忽多忽少)。
         --  IA 2026-09-15 实测:第二段第 1 推就报"碰到了",而同一行写着
         --  差距 2.830 → 2.830(一点没变)、还差二十步、"点在画面里没动过"——
         --  一个什么都没发生的步子宣布了接触,`until touched` 于是一推就结束。
         --  零系数:门槛是身体自己量到的交付噪声。
         --  🔴🔴 第三条旁证:【它得贴着我】。画面上离我自己那一块比一个我还远 ⇒
         --  隔着大半张桌子,不可能是我碰的(IG 2026-09-15 看图证实:手在右下角,球在桌心)。
         --  尺子是我自己那块有多大,量出来的,零系数。
         if Not_Mine and then Pts (I).Kind = Thing_Pt and then not Pts (I).Lost
           and then not Blind_Now and then I < Natural (Was.Length)
           and then Table.Norm (Note.Got, Chan.Per_Arm) > Long_Float (Fl.Delivery)
           and then Near_My_Piece (Pts, I)
           and then Sqrt ((Pts (I).Cu - Was (I).Cu) ** 2 + (Pts (I).Cv - Was (I).Cv) ** 2)
                    > Long_Float'Max (Pts (I).Box_W, Pts (I).Box_H)
         then
            Note.Touched := True;
         end if;
      end;
   end loop;
   if Note.Touched then
      Put_Line ("[身]     我没在推的东西也动了 ⇒ 碰到它了");
      --  🔴 观测,不改行为(JB 2026-09-15:第 3 推就报碰到,而画面里手整条收回自己底座、
      --  球在桌心一动没动 —— 身体报的球的位置 (0.89,0.85) 正压在它自己的爪子上)。
      --  今晚的做法定版:同一处连续猜错就停止改判据,改成【让身体把判据用到的量自己说出来】。
      --  这一行说四件:球在哪 · 这一步是看见的还是按身体图猜的 · 它离我最近那一块有几个"我"那么远
      --  · 它挪了多少 vs 我那一块挪了多少(真碰到时这两个应该同量级)。
      for I in 0 .. Natural (Pts.Length) - 1 loop
         if Pts (I).Kind = Thing_Pt and then I < Natural (Was.Length) then
            declare
               Me_D : Long_Float := -1.0;   --  离我最近那一块多远(以那一块自己的大小为尺)
               Me_M : Long_Float := 0.0;    --  我那一块这一步挪了多少
            begin
               for J in 0 .. Natural (Pts.Length) - 1 loop
                  if Pts (J).Kind /= Thing_Pt and then J < Natural (Was.Length) then
                     declare
                        Sz : constant Long_Float :=
                          Long_Float'Max (Track_Win,
                                          Long_Float'Max (Pts (J).Box_W, Pts (J).Box_H));
                        D : constant Long_Float :=
                          Sqrt ((Pts (I).Cu - Pts (J).Cu) ** 2 + (Pts (I).Cv - Pts (J).Cv) ** 2) / Sz;
                     begin
                        if Me_D < 0.0 or else D < Me_D then
                           Me_D := D;
                           Me_M := Sqrt ((Pts (J).Cu - Was (J).Cu) ** 2
                                         + (Pts (J).Cv - Was (J).Cv) ** 2);
                        end if;
                     end;
                  end if;
               end loop;
               Put_Line ("[身]     🔎 碰到谁:第" & Codec.Img (Pts (I).Item_No) & " 块在 ("
                         & Codec.Fmt (Pts (I).Cu, 3) & "," & Codec.Fmt (Pts (I).Cv, 3) & ")·"
                         & (if Pts (I).Lost then "这一步【没看见,按身体图猜的】" else "这一步真看见了")
                         & "·离我最近那一块 " & Codec.Fmt (Me_D, 2) & " 个我"
                         & "·它挪了 " & Codec.Fmt (Sqrt ((Pts (I).Cu - Was (I).Cu) ** 2
                                                      + (Pts (I).Cv - Was (I).Cv) ** 2), 4)
                         & " 而我那一块挪了 " & Codec.Fmt (Me_M, 4));
            end;
         end if;
      end loop;
   end if;
end Learn;
