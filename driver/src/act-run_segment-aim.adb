separate (Act.Run_Segment)
procedure Aim (Terms : out Table.Term_Vectors.Vector) is
begin
   In_Metres := [others => False];
   Terms.Clear;
   declare
      Big : Long_Float := 0.0;
   begin
      for P of Pts loop
         if P.Kind = Thing_Pt then
            Big := Long_Float'Max (Big, Long_Float'Max (P.Box_W, P.Box_H));
         end if;
      end loop;
      C.Want_Size := Big;
   end;
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         P : constant Point := Pts (I);
         T : Table.Term;
      begin
         T.E := Effs (I);
         --  🔴 离得越远,"落在两指正中间"越不该急着满足:那是到跟前才成立的几何。
         --  权重 = 指尖有多近 ÷ 它有多远(量出来的两个深度之比):远的时候远近压过画面 ⇒ 先走过去;
         --  越走近画面越重要 ⇒ 最后才精确对准。不这么定,它会在 25 cm 外用转手腕把画面对齐,手一步没靠近(FI 实测)
         declare
            Near : constant Long_Float :=
              (if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) and then P.Tz > 0.0
               then Long_Float'Min (1.0, P.Tz / P.Z) else 1.0);
         begin
            --  🔴🔴 两块东西一前一后时,【画面上重合 ≠ 真的在一起】。
            --  投影的规矩:同一段真实的横移,离相机越近在画面上跑得越多(跑的距离 ∝ 1/远近)。
            --  所以要对齐的不是 u,而是 u × 远近 —— 比出来的才是真实的横向差,而焦距在两边同样出现、自动约掉,
            --  一个标定参数都不需要。
            --  实测(FZ):头顶相机报"爪子离球只差 0.062 幅、几乎压上了",切到手腕相机一看球根本不在视野里 ——
            --  爪子在球【上方 30 厘米】,画面上却正好叠住。只比 u 就是在比影子。
            --  做法:把目标投影到【我这一点自己的那个远近平面】上再比 —— 远处的目标 u 按远近之比从画面中心
            --  往外放大,那才是"我要走到的那个 u"。误差仍然是 u 的单位(表/预测/走多远的检查全不变)。
            --  🔴 放大倍数用【目标的画面坐标是在哪个远近上量的】(Tuv_Z),不是目标本身的远近 Tz。
            --  `into` 的目标是"左右别动,只把远近走到它的腰上":Tu/Tv 抄的是我自己的位置,
            --  在【我的】远近上;拿 Tz/Z 去放大它,"别动"就变成了"一路往画面外走"
            --  (HP 实测:目标 (0.766,0.562) = 我自己的位置,放大后"该去 0.935",第二个点更是 1.014,
            --   已经在画面外;被跟的点整天往右沿飘到 u=1.000 就是这么来的)。
            if P.Z > 0.0 and then P.Tuv_Z > 0.0 then
               T.Err (0) := On_My_Plane (P.Tu, P.Tuv_Z, P.Z) - P.Cu;
               T.Err (1) := On_My_Plane (P.Tv, P.Tuv_Z, P.Z) - P.Cv;
            else
               T.Err (0) := P.Tu - P.Cu;
               T.Err (1) := P.Tv - P.Cv;
            end if;
            T.W (0) := Near;
            T.W (1) := Near;
         end;
         --  远近:画面位置和远近一起要,不许替它定"先对准再靠近"的顺序(那等于叫它先扭脖子)
         --  🔴🔴 尺子量出来的距离【优先】(2026-09-15):深度读数被我自己量出来放大了几十倍,
         --  而胳膊量出来的那个米数是真的。有米数就用米数,单位换算靠"一推走几米"(探针顺手量的)。
         --  IM 实测不接进来的后果:横向对到 2 毫米、前后还差 0.535 m,解算却说"还差 0.0 步" ——
         --  前后那一栏没有任何真东西在驱动它。
         if P.Kind = Thing_Pt and then P.Dist > 0.0 then
            T.Err (2) := -P.Dist;   --  还要往它那边走这么多米(负号 = 要靠近)
            T.W (2) := 1.0;
            In_Metres (I) := True;
         elsif P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) then
            T.Err (2) := P.Tz - P.Z; T.W (2) := 1.0;
         end if;
         --  看着多大:离得越近越大,这是最稳的远近信号(画面上量的)
         if P.Wsize > 0.0 and then P.Size > 0.0 and then P.Tsize > 0.0 then
            T.Err (3) := P.Tsize - P.Size; T.W (3) := 1.0;
         end if;
         --  朝向:差绕回 (-π, π];圆的东西这一行谁也改不动,归一时自动关掉
         if P.Wang > 0.0 then
            T.Err (4) := Wrap (P.Tang - P.Ang); T.W (4) := P.Wang;
         end if;
         --  🔴 五样单位不同,混着求和就是错的判据。不换算成米,改成【只比较】:
         --  每一样除以"推一步最多能把它改多少",都变成"还差几步"(无量纲),本来就可比。
         --  扭手腕改不了远近 ⇒ 它在那一栏拿不到分,偷不了便宜。
         No_Scale_Row := False;
         for R in 0 .. Table.Rows - 1 loop
            declare
               Per_Step : Long_Float := 0.0;
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
                     --  🔴🔴 "一推能改多少"必须用【这一段真发得出的那一推】,
                     --  不是开机量到的那一档(IX 2026-09-15 实测两边差四十倍):
                     --  开机那一档 0.0256 在腕眼里能扫 4 个画幅,而实际发出的命令是 0.003
                     --  (被"眼睛一步跟得住多少"的天花板压着),只扫 0.1 个画幅。
                     --  于是真实误差 0.26 画幅(四分之一张画面)被算成"不到一步",
                     --  身体认定自己差不到一根头发丝,只发极小命令一步步蹭,差距九步不动。
                     --  天花板是量出来的:跟踪窗 ÷ 这根通道每单位命令把画面搅动多少。
                     Px_K : constant Long_Float :=
                       Sqrt (T.E.B (K, 0) ** 2 + T.E.B (K, 1) ** 2);
                     Am : constant Long_Float :=
                       Long_Float'Min (Long_Float'Max (1.0e-9, C.Map.Amp (Ch_No)),
                                       (if Px_K > 0.0 then Track_Win / Px_K
                                        else Long_Float'Max (1.0e-9, C.Map.Amp (Ch_No))));
                  begin
                     --  🔴 这一行是【米】的时候,一步能改多少也得是米:一推手在世界里走几米。
                     --  拿画面单位的斜率去除米,等于把两把不同的尺子相除 —— 那才是真的乱来。
                     --  🔴🔴 而且米这一行【不看画面证没证过】(IR 2026-09-15 实测):
                     --  走几米是关节读数给的,是本体感觉,跟"这根通道在画面里量准没量准"毫无关系。
                     --  卡在 Trusts 上的后果:腕眼里所有通道都标"没证过" ⇒ 这一行永远没有换算
                     --  ⇒ `米那一行没换算` 连喊 8 次,而换算其实早就量到了。
                     if R = 2 and then In_Metres (I) then
                        if C.Map.Seen (Ch_No) then
                           Per_Step := Long_Float'Max
                             (Per_Step,
                              (if Ch_No < Natural (C.Reach_M.Length)
                               then C.Reach_M.Element (Ch_No) else 0.0) * Am);
                        end if;
                     elsif C.Map.Seen (Ch_No) and then Trusts (I) (K) then
                        Per_Step := Long_Float'Max (Per_Step, abs (T.E.B (K, R)) * Am);
                     end if;
                  end;
               end loop;
               --  🔴 米那一行没有换算(一推走几米还没量到)⇒ 这一行【静悄悄地失效】,
               --  解算照跑、日志全绿、差距一步不动(IO 实测五步 0.719→0.719)。喊出来。
               if R = 2 and then In_Metres (I) and then Per_Step <= 0.0 then
                  C.Blind_Say := S ("I measured how far that thing is with my own arm, but I have not yet "
                                    & "measured how far my hand travels per push, so I cannot turn those metres "
                                    & "into pushes - that row is doing nothing and I am telling you instead of "
                                    & "quietly going nowhere.");
                  Put_Line ("[身]     📏 米那一行没换算(还没量到一推走几米)⇒ 这一行是死的");
               end if;
               --  🔴🔴 只加观测,不改逻辑:把【真正送进解算的那个误差】和【分母】原样打出来。
               --  IY 2026-09-15:我压小了分母,`还差` 照旧全是 0.0 —— 五次落空之后不再猜第六个机制。
               --  同一份日志里 `我在 (0.872,0.489) · 目标 (0.918,0.750)` 明明差 0.26 画幅,
               --  而 `还差 上下 0.0` ⇒ **送进解算的误差和打印给脑看的目标不是同一个东西**。
               --  哪一半是 0,打出来就知道 —— `米那一行没换算` 那次正是这么抓到的。
               if I = 0 then
                  Put_Line ("[身]     🔎 第" & Codec.Img (R) & " 行:误差 "
                            & Codec.Fmt (T.Err (R), 4) & " · 权重 " & Codec.Fmt (T.W (R), 3)
                            & " · 一推能改 " & Codec.Fmt (Per_Step, 4)
                            & " ⇒ 还差 "
                            & Codec.Fmt ((if Per_Step > 0.0 then T.Err (R) / Per_Step else T.Err (R)), 2)
                            & " 步");
               end if;
               if Per_Step > 0.0 then
                  T.Err (R) := T.Err (R) / Per_Step;
                  --  🔴 "还差几步"不许超过"我这一节总共有几步"。
                  --  一行几乎推不动时,它的每步效果≈0,误差除下来是个天文数字(GL 实测:
                  --  "看着多大"这一行推每根通道都一动不动,却算出 12.9 步,把整个解算劫持了)。
                  --  超过这一节的步数预算,就说明这一行在这一节里【本来就修不完】,
                  --  不许它压过那些修得完的行。上限用的是【脑自己给的步数】,不是我拍的数。
                  declare
                     Budget : constant Long_Float :=
                       Long_Float (Effective_Cap (Step_Limit));
                  begin
                     T.Err (R) := Long_Float'Max (-Budget, Long_Float'Min (Budget, T.Err (R)));
                  end;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     T.E.B (K, R) := T.E.B (K, R) / Per_Step;
                  end loop;
               else
                  --  🔴 "算不出"要记下来,不许悄悄变成 0 ——
                  --  下面 Steps_Err 是按权重加起来的,权重清零 ⇒ 总和 0 ⇒ 对外就成了"还差 0.0 步"。
                  if T.W (R) > 0.0 then
                     No_Scale_Row := True;
                  end if;
                  T.W (R) := 0.0;   --  这一行一个通道都改不动 ⇒ 这一步没法管它(不是拦,是算不出)
               end if;
            end;
         end loop;
         declare
            Q : Point := P;
         begin
            Q.Steps_Err := 0.0;
            for R in 0 .. Table.Rows - 1 loop
               Q.Steps_Err := Q.Steps_Err + (T.Err (R) * T.W (R)) ** 2;
            end loop;
            Q.Steps_Err := Sqrt (Q.Steps_Err);
            Q.No_Scale := No_Scale_Row;
            Q.Err_U := T.Err (0) * T.W (0); Q.Err_V := T.Err (1) * T.W (1); Q.Err_Z := T.Err (2) * T.W (2);
            Q.Err_S := T.Err (3) * T.W (3); Q.Err_A := T.Err (4) * T.W (4);
            Q.Raw_Err := Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2
                               + (if P.Wz > 0.0 and then P.Z > 0.0 then ((P.Tz - P.Z) / P.Z) ** 2 else 0.0)
                               + (if P.Wsize > 0.0 and then P.Tsize > 0.0 then ((P.Tsize - P.Size) / P.Tsize) ** 2 else 0.0)
                               + (if P.Wang > 0.0 then (P.Wang * Wrap (P.Tang - P.Ang) / Ada.Numerics.Pi) ** 2 else 0.0));
            Pts.Replace_Element (I, Q);
         end;
         Terms.Append (T);
      end;
   end loop;
   if Last_Err < 0.0 then
      Last_Err := 0.0; Last_Raw := 0.0;
      for P of Pts loop
         Last_Err := Last_Err + P.Steps_Err;
         Last_Raw := Last_Raw + P.Raw_Err;
      end loop;
      Best_Raw := Last_Raw;
   end if;
end Aim;
