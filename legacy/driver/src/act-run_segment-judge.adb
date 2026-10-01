separate (Act.Run_Segment)
procedure Judge is
begin
   --  进度只看不随表变的那把尺(Raw):"还差几步"的刻度每步都在变,用它判进度会把靠近判成退步(ES 实测两步就报停滞)
   --  和"到目前为止最好的一次"比:和上一步比的话,一次噪声就被当成退步
   Monitor.Step (W, Monitor.Floor (Long_Float'Max (0.0, Note.Pic_Delta)), Monitor.Bounded (Best_Raw), Monitor.Bounded (Note.Raw_Now),
                 Monitor.Floor (Long_Float'Max (0.0, Table.Norm (Note.Got, Chan.Per_Arm))), Fl,
                 Seen => (for all P of Pts => not P.Lost));
   Put_Line ("[身]     步" & Natural'Image (Steps_Taken) & (if Note.Big_Step then "(大步)" else "") &
             ":差距 " & Codec.Fmt (Last_Raw, 3) & " → " & Codec.Fmt (Note.Raw_Now, 3)
             & (if Pts (0).No_Scale and then Note.Err_Now <= 0.0
                then " · 还差几步【我不知道】(这里推一下能改多少还没量出来,不是到了)"
                else " · 还差 " & Codec.Fmt (Note.Err_Now, 1) & " 步")
             & "(左右 " & Codec.Fmt (Pts (0).Err_U, 1) &
             " 上下 " & Codec.Fmt (Pts (0).Err_V, 1) & " 远近 " & Codec.Fmt (Pts (0).Err_Z, 1) &
             " 大小 " & Codec.Fmt (Pts (0).Err_S, 1) & " 朝向 " & Codec.Fmt (Pts (0).Err_A, 1) & ")· 拍 " & Codec.Img (Beats) &
             " · 信表 " & Codec.Fmt (Trust, 2) & " · 步幅 ×[" & Codec.Fmt (Reach (0), 0) & " " & Codec.Fmt (Reach (1), 0) & " " & Codec.Fmt (Reach (2), 0) & " " &
             Codec.Fmt (Reach (3), 0) & " " & Codec.Fmt (Reach (4), 0) & " " & Codec.Fmt (Reach (5), 0) &
             "] · 命令 [" & Codec.Fmt (Note.Cmd (0), 3) & " " & Codec.Fmt (Note.Cmd (1), 3) & " " & Codec.Fmt (Note.Cmd (2), 3) & " " &
             Codec.Fmt (Note.Cmd (3), 3) & " " & Codec.Fmt (Note.Cmd (4), 3) & " " & Codec.Fmt (Note.Cmd (5), 3) &
             "] · 实到 [" & Codec.Fmt (Note.Got (0), 4) & " " & Codec.Fmt (Note.Got (1), 4) & " " & Codec.Fmt (Note.Got (2), 4) & " " &
             Codec.Fmt (Note.Got (3), 3) & " " & Codec.Fmt (Note.Got (4), 3) & " " & Codec.Fmt (Note.Got (5), 3) &
             "] · 差 " & Cm_Gap (Effs (0), Pts (0)) &
             " · 点 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) &
             (if Note.Blocked then " · 零表更准(顶住?)" else ""));
   --  点这一步在画面里跑了多远?没跑过跟踪地板就把下一步的命令翻倍(见 Push_Mult 的说明)
   if Natural (Pts.Length) > 0 and then Natural (Was.Length) > 0 then
      declare
         D : constant Long_Float :=
           Sqrt ((Pts (0).Cu - Was (0).Cu) ** 2 + (Pts (0).Cv - Was (0).Cv) ** 2);
      begin
         if D <= Long_Float (Fl.Track) then
            --  🔴 放大多少不是人拍的:拿【这一点还差多远】当目标,不是拿跟踪地板。
            --  第一版用的是跟踪地板,而跟踪地板 ≈ 一个像素(实测 0.0016 vs 1/640 = 0.0015625)⇒
            --  比值恒等于 1.02 ⇒ 打印永远是 ×1.0,等于没放大。目标设成"一个像素"本来就够不着任何用。
            --  D 可能是 0,下限仍取这台相机的一个像素(它能分辨的最小位移,量出来的)。
            declare
               Need : constant Long_Float :=
                 Sqrt ((Pts (0).Tu - Pts (0).Cu) ** 2 + (Pts (0).Tv - Pts (0).Cv) ** 2);
               Floor_D : constant Long_Float := 1.0 / Long_Float'Max (1.0, Long_Float (Cw));
            begin
               if Need > Long_Float (Fl.Track) then
                  --  🔴 封顶:一步推出去,这一点在画面里跑的距离不许超过【眼睛跟得住的一个窗口】。
                  --  没有这一条,GT 实测一步就放到 ×236,点被甩到画面角落 (1.000,0.124) 深 13.9 m。
                  --  这条上界不是新拍的 —— Note.Cap 用的就是同一条(Track_Win / 这一点每单位跑多远)。
                  Push_Mult := Long_Float'Min (Push_Mult * (Need / Long_Float'Max (D, Floor_D)),
                                               Track_Win / Long_Float'Max (D, Floor_D));
               end if;
            end;
            Put_Line ("[身]     点在画面里没动过(" & Codec.Fmt (D, 4) & " ≤ 地板 " & Codec.Fmt (Long_Float (Fl.Track), 4)
                      & ")⇒ 下一步命令整体 ×" & Codec.Fmt (Push_Mult, 0));
         else
            Push_Mult := 1.0;
         end if;
      end;
   end if;
   Last_Err := Note.Err_Now;
   Last_Raw := Note.Raw_Now;
   if Best_Raw < 0.0 or else Note.Raw_Now < Best_Raw then
      Best_Raw := Note.Raw_Now;
   end if;
   if Codec.Env ("BL_STEPSHOT") /= "" then
      Dump_Picture ("step");   --  逐步落图:看被跟的那块在靠近时到底怎么变(BL_STEPSHOT 打开才存)
   end if;
   if Note.Blocked or else Monitor.Refusing (W) then
      Blocked_Out := True;
   end if;
   --  认不到:第一次落图给人看,连着两步才停(被挡住一团是常事)
   Lost_Run := (if Note.Lost_All then Lost_Run + 1 else 0);
   if Lost_Run = 1 then
      Dump_Picture ("lost");
   end if;
   if Lost_Run >= 2 then
      --  🔴 "我宁可停下也不瞎走"是意见。改成:瞎着也走,并且如实说我瞎着走了。
      C.Blind_Say := S ("for two steps in a row I could not find what I am tracking in this picture, "
                        & "so from here I am moving without seeing it");
   end if;
   --  🔴 认东西是脑的活:两块一样像的时候身体不许自己挑
   if Note.Unsure then
      Dump_Picture ("unsure");
      --  🔴 分不清也不许停:挑一个走,并如实说我分不清、我挑了哪个。
      C.Blind_Say := S ("two things here look equally like the one you named (same size, same distance); "
                        & "I could not tell them apart, so I picked one and kept going");
   end if;
   if Note.Not_Followed then
      Put_Line ("[身]     没照做这一步不算数,步幅已缩回;接着走");
   end if;
   --  没写步数就拿安全上限比,别拿 0 比(拿 0 比 = 第一步就"走完了")
   --  抓握读数先换成"离空手合那头往张开那头走了多远"再交给监视器(方向是量的;监视器按"读数 − 空手值 ≤ 抖动 = 滑掉了"判)
   if Monitor.Fired (Until_Kind, W, Effective_Cap (Step_Limit), Note.Blocked, Monitor.Bounded (if Selfmap.Has_Jaw (F, Arm) and then Hand_Of (C, Arm).Measured
                                                             then Past_Empty (Hand_Of (C, Arm), Selfmap.Jaw_Of (F, Arm))
                                                             else Monitor.Bounded'Last),   --  没读数 / 手没量过:这一拍没有"滑了"的证据
                     Monitor.Bounded (0.0),
                     Monitor.Floor (C.Map.Jaw_Noise), Note.Touched,
                     Lost => Pts (0).Lost,
                     Height_Now => Monitor.Bounded (Pts (0).Height),
                     Height_Then => Monitor.Bounded (H0),
                     Height_Noise => Monitor.Floor (Long_Float'Max (0.0, Pts (0).Z_Noise)))
   then
      Note.Say_Stop := (case Until_Kind is
                          when Monitor.U_Steps => S ("steps: I took the steps you asked for"),
                          when Monitor.U_Contact => S ("contact: something I was not pushing moved when I moved - I am touching it"),
                          when Monitor.U_Resist => S ("resist: I commanded a push and my body did not go"),
                          when Monitor.U_Slip => S ("slip: what I was holding has left my fingers"),
                          when Monitor.U_Settle => S ("settle: the picture stopped changing"),
                          when Monitor.U_Stall => S ("stall: I am still moving, but for several steps in a row "
                                                     & "the gap has stopped shrinking - you asked me to come back "
                                                     & "when that happened"),
                          when Monitor.U_Lost => S ("lost: I cannot see the thing I am tracking any more"),
                          when Monitor.U_Free => S ("free: it now stands higher off the surface than when I started - it has come free"));
      return;
   end if;
   --  🔴🔴 owner 2026-09-14 死命令:【"到了没到"只有脑能判】。身体这里原来自己算一个容差,
   --  容差之内就宣布"到了"并停下 —— 那是身体在替脑做判断,而且它判错过:
   --  HI 实测 `结局 = arrived` 只走了 1 推,而远近还差 0.200 m,合爪时球离两指之间 0.289 画幅、
   --  只有该有大小的 7.7%。容差换了一版还是错(容差本身就不该存在)。
   --  身体只许报它【量到的事件】:碰到、顶住、滑了、跟丢、离开了面、合拢停住。
   --  "我觉得够近了"不是事件,是意见。⇒ 整段删掉,不再有任何"到了"的自停。
   --  🔴 原来这里有一条"停止靠近了 —— 要么有东西挡着我,要么这条胳膊够不了更远"。
   --  那是身体自己发明的终止条件,而且那句解释还常常是假的(真实原因往往是这个视角看不出)。
   --  【删掉】。误差不缩小是一个【事实】,如实记下来给脑看,由脑决定还走不走。
   if Steps_Taken > 1 and then Monitor.Stalled (W) then
      C.Blind_Say := S ("for several steps in a row the gap stopped shrinking (about "
                        & Codec.Fmt (Note.Err_Now, 1) & " pushes still to go) - I kept going anyway");
   end if;
end Judge;
