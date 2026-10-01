separate (Act.Run_Segment)
procedure Budget (Terms : Table.Term_Vectors.Vector; Solved : out Boolean) is
   Damp : Table.Vec := Table.Zero_Vec;
begin
   for K in 0 .. Chan.Per_Arm - 1 loop
      declare
         Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
         Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Ch_No));
         All_Trust : Boolean := True;
         Px : Long_Float := 0.0;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            if not Trusts (I) (K) then
               All_Trust := False;
            end if;
            Px := Long_Float'Max (Px, Sqrt (Effs (I).B (K, 0) ** 2 + Effs (I).B (K, 1) ** 2));
         end loop;
         --  🔴🔴 走【米】的时候,能不能用一根关节看的是"它能把手挪动几米"(本体感觉),
         --  不是"它在画面里量准没量准"(IS 2026-09-15 实测)。
         --  IS:米数终于进了解算(远近 -40.0 步,前 24 炮全是 0.0),可六根关节里
         --  只有一根过得了画面那道门 ⇒ 每步只挪 2 毫米 ⇒ 0.73 m 要三百多步,
         --  而一段只有 40 步。差距九步 0.744 → 0.730,基本不动。
         --  ⚠️ 只在【这一段真的在用米】的时候放开(有点的远近是尺子量出来的),
         --  免得平时让没量准的通道去搅画面(FD 实测:0.6 rad 的腕一转就把距离搞坏)。
         declare
            Walks : constant Boolean :=
              (for some I in 0 .. Natural (Pts.Length) - 1 => In_Metres (I))
              and then Ch_No < Natural (C.Reach_M.Length)
              and then C.Reach_M.Element (Ch_No) > 0.0;
         begin
         if C.Map.Seen (Ch_No) and then (All_Trust or else Walks) then
            Note.Active (K) := True;
            --  🔴 只有【后果全量清楚了】的方向才准迈大步。某一格没量出来(探针时变化没过地板)会被留成 0,
            --  而 0 的意思是"没影响",解算就当它免费 —— 转腕对"远近/看着多大"正是这样,于是它拿转腕去修画面位置,
            --  一转就把距离搞坏(FD 实测:0.6 rad 的腕,球越走越远)。没量清楚的方向只给探针那一档。
            declare
               Known_All : Boolean := Px * Am > Fl.Track * 2.0;
            begin
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  for R in 0 .. Table.Rows - 1 loop
                     if Terms (I).W (R) > 0.0 and then abs (Effs (I).B (K, R)) <= 0.0 then
                        Known_All := False;
                     end if;
                  end loop;
               end loop;
               --  上限 = 自己那一档 × 核实过的倍数,再压在"眼睛跟得住"这个天花板下。
               --  倍数只有靠"表说会挪多少 vs 实际挪了多少"对上才涨(见 Learn),没证明过就不许迈大步
               --  🔴 天花板底下还要有【地板】:命令小到比身体自己的噪声还小 ⇒ 一步一动不动。
               --  这条 7d832e3 装过又被 09-13 那次整体回滚削掉:FO 每步命令 0.006(探针那一档的 1/4),
               --  一步推进 8 厘米、44 推【够到球并合爪咬在球上】;削掉之后同一通道每步走到 0.026,
               --  表当场不准、球被甩出视野。
               --  ⚠️ 措辞订正(2026-09-16):这里原写"44 推抓到球"不准。逐炮记录原文:
               --  "第一次真的够到球并【合上爪子】,但夹在球的【很偏上处】,一合就把球撞到画面角落;
               --   身体两次报「拿住了」都是假的",接近到 8.4 cm(指尖 6.8)。
               --  ⇒ 两指确实合在球上了,假在【没提起来】,不是没碰到。基线 = 合爪咬住球。
               --  地板 = 身体自己的噪声的两倍(量出来的,不是探针那一档)。
               --  地板顶穿天花板 = "能让我动起来的命令,我的眼睛一步跟不住" —— 这是身体量得出来的事实,
               --  照地板走并且说出来(动不了的命令严格无用,跟丢了还能重新认)。
               declare
                  Ceiling : constant Long_Float :=
                    Long_Float'Min (Am * Cap_Mult * Reach (K),
                                    (if Known_All then Track_Win / Px else Am * Cap_Mult * Reach (K))) * Amount;
                  Floor : constant Long_Float := Long_Float'Max (C.Map.EE_Noise + C.Map.EE_Noise, (if Ch_No < Natural (C.Dead.Length) then C.Dead.Element (Ch_No) else 0.0));
               begin
                  --  地板同理:静止噪声在这具仿真里量到 0,拿它当地板等于没有地板。
                  --  用【这个通道确实动过的最小命令】:学到的死区,没学到就用开机量到的那一档。
                  Note.Cap (K) := Push_Cap
                    (Ceiling, C.Map.EE_Noise,
                     (if Ch_No < Natural (C.Dead.Length) and then C.Dead.Element (Ch_No) > 0.0
                      then C.Dead.Element (Ch_No) else C.Map.Amp (Ch_No)));
                  if Floor > Ceiling and then Ceiling > 0.0 then
                     C.Blind_Say := S ("any push big enough for my body to actually move is bigger than my eye "
                                       & "can follow in one step here; I took the smaller-of-the-two that still moves me");
                  end if;
               end;
            end;
            Note.Floor_Cmd := (if Note.Floor_Cmd <= 0.0 then Am else Long_Float'Min (Note.Floor_Cmd, Am));
         end if;
         end;
         --  🔴 标价改成"这个动作把画面搅动多少":一单位命令让被跟的点在画面里跑几个跟踪窗,就付几分钱(无量纲)。
         --  以前按"自己那一档"计价,而转腕那一档(0.0256)比平移那一档(0.0064)大四倍 ⇒ 转腕在账本上便宜十六倍,
         --  于是它一直买转腕,而转腕不会让手靠近(FJ 实测:横挪 4 cm,球反而从 0.333 m 退到 0.360 m)
         --  🔴🔴 在【长在我身上的那只眼】里,"把手抬高"和"把镜头仰起来"在画面上一模一样:
         --  球都往下走。按上面那个标价,两者花的钱也一样(代价 = 画面改变量²,与用哪根通道无关),
         --  于是解算总买更省力的那个 —— 转腕。手一步没靠近,球却被仰出了视野。
         --  IW 2026-09-15 同一炮里【连着两次】:腕眼里压几步,画面就只剩窗户和天花板。
         --  这是 FJ 那条老账的同族(转腕不会让手靠近:横挪 4 cm,球反而从 0.333 m 退到 0.360 m)。
         --  身体现在分得出来:我在世界里【真走了多少米】是关节读数给的,
         --  仰镜头几乎不位移、抬手真位移 —— 画面里一样,本体感觉里天差地别。
         --  ⇒ 在这只眼里按【搅动画面 ÷ 真把我挪了多远】标价:
         --  只搅画面不挪我的通道,价钱按倍数涨上去,解算自己就不买了。零系数,两个都是量出来的。
         Damp (K) := (Px / Track_Win) ** 2;
         if Own_Cam and then Ch_No < Natural (C.Reach_M.Length) then
            declare
               Mine : constant Long_Float := C.Reach_M.Element (Ch_No);
               Best : Long_Float := 0.0;
            begin
               for J in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Jn : constant Natural := Arm * Chan.Per_Arm + J;
                  begin
                     if Jn < Natural (C.Reach_M.Length) then
                        Best := Long_Float'Max (Best, C.Reach_M.Element (Jn));
                     end if;
                  end;
               end loop;
               if Best > 0.0 then
                  if Mine > 0.0 then
                     Damp (K) := Damp (K) * (Best / Mine);
                  else
                     --  一点都不挪我 ⇒ 在这只眼里它对"靠近"毫无贡献,只会把镜头转开
                     --  一点都不挪我 ⇒ 在这只眼里它对"靠近"毫无贡献,只会把镜头转开。
                     --  价钱抬到【这一段最能挪我的那根】的整个量级之上,解算自然不买。
                     Damp (K) := Damp (K) + Damp (K) / Long_Float'Max (Best, Long_Float'Small);
                  end if;
               end if;
            end;
         end if;
      end;
   end loop;
   --  hold 的那几条进硬约束:先把它们解到位,软目标只能在剩下的自由度里做文章。
   --  平权解在挤不下的时候一定会牺牲朝向(自检里那条 5.500 就是),所以这里不能用平权。
   declare
      Hard, Soft : Table.Term_Vectors.Vector;
   begin
      for I in 0 .. Natural (Terms.Length) - 1 loop
         if I < Natural (Pts.Length) and then Pts (I).Hard then
            Hard.Append (Terms (I));
         else
            Soft.Append (Terms (I));
         end if;
      end loop;
      Table.Solve_Priority (Hard, Soft, Chan.Per_Arm, Note.Cap, Note.Active, Damp, Note.Cmd, Solved);
   end;
end Budget;
