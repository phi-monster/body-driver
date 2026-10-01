separate (Act.Run_Segment)
procedure Trim is
   Scale : Long_Float := 1.0;
begin
   --  求稳不求快:一步里任何被跟的点在画面里最多跑一个跟踪窗
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
         D : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
      begin
         if D > Track_Win then
            Scale := Long_Float'Min (Scale, Track_Win / D);
         end if;
      end;
   end loop;
   if Scale < 1.0e-3 then
      Note.Big_Step := True;   --  缩到千分之一还不够(比例,无量纲)= 表已经不可信
   end if;
   --  不许把被跟的东西推出视野;不许让【我身上任何一块】压到"不许碰"的框里(只查一个点等于没查)
   for Round in 1 .. 4 loop
      declare
         Hit : Boolean := False;
         function In_Avoid (X0, Y0, X1, Y1 : Long_Float) return Boolean is
         begin
            for Av of Avoid loop
               if Av.Located and then X0 <= Long_Float (Av.X1) and then X1 >= Long_Float (Av.X0)
                 and then Y0 <= Long_Float (Av.Y1) and then Y1 >= Long_Float (Av.Y0)
               then
                  return True;
               end if;
            end loop;
            return False;
         end In_Avoid;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
               Nu : constant Long_Float := Pts (I).Cu + Pr (0) * Scale;
               Nv : constant Long_Float := Pts (I).Cv + Pr (1) * Scale;
            begin
               if Nu < Track_Win or else Nu > 1.0 - Track_Win or else Nv < Track_Win or else Nv > 1.0 - Track_Win then
                  Hit := True;
               end if;
               if In_Avoid (Nu * Long_Float (Cw), Nv * Long_Float (Ch), Nu * Long_Float (Cw), Nv * Long_Float (Ch)) then
                  Hit := True;
               end if;
            end;
         end loop;
         --  这一段是"别撞到脑点名要躲的东西",躲的框是【脑看的那台相机】里的坐标,所以这里仍用 Cam
         if Cam_Arm (C, Cam) /= Integer (Arm) and then Track_Idx (C, Arm, Cam) < Natural (C.Zones.Length) then
            declare
               Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Cam));
            begin
               for K in 0 .. Chan.Per_Arm loop
                  if Tr.Pieces (K).Valid then
                     declare
                        Idx : constant Integer := Find_Effect (C, Arm, Cam, Piece_Pt, K, -1);
                        Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Note.Cmd) else Table.Zero3);
                        Du : constant Long_Float := Sh (0) * Scale * Long_Float (Cw);
                        Dv : constant Long_Float := Sh (1) * Scale * Long_Float (Ch);
                     begin
                        if In_Avoid (Long_Float (Tr.Pieces (K).X0) + Du, Long_Float (Tr.Pieces (K).Y0) + Dv,
                                     Long_Float (Tr.Pieces (K).X1) + Du, Long_Float (Tr.Pieces (K).Y1) + Dv)
                        then
                           Hit := True;
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end if;
         exit when not Hit;
         if Round = 4 then
            --  🔴 原来这里会停下,理由是"再走一步我就看不见它了"。那是【怕】,不是【做不到】。
            --  身体不许有意见:照走,把这件事说出来就行。
            C.Blind_Say := S ("I kept going even though the next step may take what I am tracking "
                              & "out of my sight");
            exit;
         end if;
         Scale := Scale * 0.5;
      end;
   end loop;
   for K in 0 .. Chan.Per_Arm - 1 loop
      Note.Cmd (K) := Note.Cmd (K) * Scale * Trust;   --  表有多准就走多少(不然每步走过头,下一步再拉回来,来回晃)
   end loop;
   declare
      N0 : constant Long_Float := Table.Norm (Note.Cmd, Chan.Per_Arm);
      --  🔴🔴 "我能走的最小一步"不能用【静止噪声】——这具仿真里静止两拍画面一模一样,
      --  量出来就是 0.00000,于是这条放大整条失效(G = Max(1, 0/N0) = 1)。
      --  HK 实测:连着五步 `命令 [-0.000 …] 实到 [0.0000 ×6]`,差距钉在 0.400、远近还差 0.198 m。
      --  改用【我确实动过的最小命令】:这个通道自己学到的死区;还没学到就用开机量到的那一档
      --  (0.0064/0.0032/0.0016 —— 正是 FO 抓球那一档的量级)。两个都是量出来的。
      Floor_Move : Long_Float := C.Map.EE_Noise;
   begin
      for K in 0 .. Chan.Per_Arm - 1 loop
         if Note.Active (K) then
            declare
               Cn : constant Natural := Arm * Chan.Per_Arm + K;
               D : constant Long_Float :=
                 (if Cn < Natural (C.Dead.Length) and then C.Dead.Element (Cn) > 0.0
                  then C.Dead.Element (Cn) else C.Map.Amp (Cn));
            begin
               Floor_Move := Long_Float'Max (Floor_Move, D);
            end;
         end if;
      end loop;
      if N0 <= Floor_Move then
         --  剩下要推的比我能动起来的最小一步还小,【不许】宣布"已经到了" ——
         --  那是身体在替脑判断。它是一个事实,说出来,继续走。
         if N0 > 0.0 then
            --  🔴 脑没让停 ⇒ 不许发一个身体根本走不动的命令。放大到我能走的最小一步,方向不变。
            --  (GK 实测:不放大的话每一步都是零命令,30 步全是空转。)
            declare
               G : constant Long_Float := Long_Float'Max (1.0, Floor_Move / N0);
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  Note.Cmd (K) := Note.Cmd (K) * G;
               end loop;
            end;
         end if;
      end if;
   end;
   --  上一步点在画面里没动过 ⇒ 这一步整体放大(方向不变)
   if Push_Mult > 1.0 then
      for K in 0 .. Chan.Per_Arm - 1 loop
         Note.Cmd (K) := Note.Cmd (K) * Push_Mult;
      end loop;
   end if;
   --  🔴🔴 解算【里面】按 Note.Cap 夹过了,可上面这两下放大都在【外面】,谁都不管:
   --    ① G = Floor_Move / N0 —— 表说"没有一根通道能改这个"时 N0≈0 ⇒ G 是天文数字;
   --    ② Push_Mult —— 画面没动就一直涨,于是上一步的天文数字再乘一遍。
   --  IZ 2026-09-15 实测三步:命令 [.. -2.2e8 -3.5e9 7.6e9 -1.9e9],实到全零,每步 ×85。
   --  身体送不出去 ⇒ 画面不动 ⇒ Push_Mult 再涨 ⇒ 自我放大,一段四步全废。
   --  🔴 一条量出来的天花板就够:**我发的这一下不许超过【眼睛跟得住的那一档】和
   --  【能让我动起来的最小一步】里大的那个**。两个都是身体自己量的:
   --    Note.Cap = 探针那一档 × 核实过的倍数,压在"眼睛一步跟得住"底下;
   --    Floor_Move = 这根通道自己学到的死区(没学到就用开机量到的那一档)。
   --  放大到死区那一档照样成立(GK 那条"不放大就 30 步空转"不受影响),
   --  再往上的一律不是推,是胡说 —— 夹回去并且说出来。
   declare
      Said : Boolean := False;
      Floor_Move : Long_Float := C.Map.EE_Noise;
   begin
      for K in 0 .. Chan.Per_Arm - 1 loop
         if Note.Active (K) then
            declare
               Cn : constant Natural := Arm * Chan.Per_Arm + K;
               D : constant Long_Float :=
                 (if Cn < Natural (C.Dead.Length) and then C.Dead.Element (Cn) > 0.0
                  then C.Dead.Element (Cn) else C.Map.Amp (Cn));
            begin
               Floor_Move := Long_Float'Max (Floor_Move, D);
            end;
         end if;
      end loop;
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Lim : constant Long_Float := Long_Float'Max (abs Note.Cap (K), Floor_Move);
         begin
            if Lim > 0.0 and then abs Note.Cmd (K) > Lim then
               if not Said then
                  Put_Line ("[身]     要发的这一下比我真推得动的那一下大 "
                            & Codec.Fmt (abs Note.Cmd (K) / Lim, 0) & " 倍 ⇒ 按我推得动的那一下走"
                            & "(表说没一根通道能改这个，放大它也没用)");
                  C.Blind_Say := S ("the push my own map asked for was far bigger than anything I have ever "
                                    & "actually managed to deliver, so I sent the biggest one I really can");
                  Said := True;
               end if;
               Note.Cmd (K) := (if Note.Cmd (K) > 0.0 then Lim else -Lim);
            end if;
         end;
      end loop;
   end;
end Trim;
