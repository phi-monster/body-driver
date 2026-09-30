separate (Act.Run_Segment)
procedure Range_Probe is
   Best_K : Integer := -1;
   Best_Amp : Long_Float := 0.0;
   EE0 : Plug.Arm_Pose;
   Moved, Turned, Slid : Long_Float := 0.0;
   --  🔴 一路拨大的过程里,最后一档【还跟得住】的读数。跟丢那一档不算数。
   Good_Amp, Good_Slid, Good_Moved, Good_Turn : Long_Float := 0.0;
   Have_Good : Boolean := False;
   Any_Lost : Boolean := False;
   Widest : Long_Float := 0.0;    --  被跟的那几块里最宽的那块,在画面里占多少
   Before : Buf_Vectors.Vector;
   Was_R : Point_Vectors.Vector;
   Got : Table.Vec;
   Ok_W : Boolean;
   Tries : Natural := 0;
   Swims : Boolean := False;
   Moved_Ref : Boolean := False;   --  这一次真出了米数 ⇒ 参照才换到这儿
   Reuse : Boolean := False;       --  这一段已经挑好拨法了 ⇒ 原样重用,不再加大
   Dir : Xyz := [others => 0.0];
   Dot : Long_Float := 0.0;
   Comparable : Boolean := False;
begin
   --  🔴🔴 ① 这只眼睛得【长在我正在动的这部分上】,世界才会在它里面滑。
   --  IF 2026-09-15 实测这一条写松了的后果:判据只问"我一动它变不变",
   --  而不动的那台眼睛里我自己的胳膊也占着画面 ⇒ 判据通过 ⇒ 身体在【不动的眼睛】里量远近。
   --  可是在不动的眼睛里,桌上的东西本来就【一动不动】—— 它要是动了,那只能是【我撞的】。
   --  当时身体把"球滑了 0.0300 幅"读成了视差,一路把拨动加到 0.3529 m,
   --  **那一甩直接把球撞到桌子最里面去了**(看图确认:球从桌心跑到最远沿)。
   --  判据改成量出来的比较:我一动,哪台眼睛变得最多,哪台才是长在我身上的。
   --  这一条对所有机体成立(无人机的眼睛长在自己身上;不动的那台只能量【我自己的零件】有多远)。
   declare
      Ix : constant Natural := Arm * C.Map.N_Cams + Cam;
      Mine : constant Long_Float :=
        (if Ix < Natural (C.Map.Cam_Frac.Length) then C.Map.Cam_Frac (Ix) else -1.0);
      Most : Long_Float := -1.0;
   begin
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         declare
            Jx : constant Natural := Arm * C.Map.N_Cams + Cm;
         begin
            if Jx < Natural (C.Map.Cam_Frac.Length) then
               Most := Long_Float'Max (Most, C.Map.Cam_Frac (Jx));
            end if;
         end;
      end loop;
      Swims := Mine > Long_Float (Fl.Track) and then Mine >= Most;
   end;
   if not Swims then
      Put_Line ("[身]   📏 这只眼睛量不了远近:它不是长在我正动的这部分上 ⇒ 桌上的东西在它里面本来就不滑;"
                & "在这只眼睛里东西要是动了,那是【我撞的】,不是远近");
      C.Blind_Say := S ("I cannot work out how far that thing is with this eye: this eye does not ride on the part "
                        & "I am moving, so the world does not slide across it at all. In this eye, a thing that "
                        & "moves while I move has been HIT by me, not measured. Ask again with the eye that rides "
                        & "on me if you want a distance.");
      return;
   end if;
   --  ② 拨哪一下:第一次量什么就一直用它,同一下拨两遍,横向那一份才会在相除时约掉
   --  🔴🔴 一段里【只挑一次拨法】,之后原样重用,不许再加大(IK 2026-09-15 的真凶)。
   --  IK 实测:同一段里参照那一拨挪 0.0033 m、后一拨只挪 0.0004 m —— 差八倍。
   --  差的来源不是物理,是我自己每次都重跑一遍"一路拨大"的escalation。
   --  同一下拨两遍,"同一下"首先得是【同一个命令】。
   if Probe_Have then
      Best_K := Probe_K; Best_Amp := Probe_Amp;
      Reuse := True;
   else
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
         begin
            if Ch_No < Natural (C.Map.Amp.Length) and then C.Map.Seen (Ch_No)
              and then C.Map.Amp (Ch_No) > Best_Amp
            then
               Best_Amp := C.Map.Amp (Ch_No); Best_K := K;
            end if;
         end;
      end loop;
   end if;
   if Best_K < 0 then
      Put_Line ("[身]   📏 量不了远近:我一根通道都没量过,不知道该拨哪一下");
      return;
   end if;
   for P of Pts loop
      Widest := Long_Float'Max (Widest, Long_Float'Max (P.Box_W, P.Box_H));
   end loop;
   Before := All_Gray (F);
   Was_R := Pts;
   EE0 := F.EE (Arm);
   --  🔴🔴 拨到【它真的滑得动】为止,不是拨到"我自己的位置读数动了"为止。
   --  IB 2026-09-15 实测第一炮就踩到:退出条件写的是"挪过本体读数抖动",
   --  而本体读数抖动只有半毫米 ⇒ 一拨 0.0005 m 就退出 ⇒ 东西只滑 0.0009 幅,
   --  还没过跟踪抖动 ⇒ 等于没量。三角形扁不扁,看的是【它在我眼里滑了多少】,
   --  不是我自己动了多少(记录 2026-08-16:挪 4 mm 而至少要 50 mm)。
   loop
      declare
         A : Table.Vec := Table.Zero_Vec;
      begin
         A (Natural (Best_K)) := Best_Amp;
         Step_Arm (L, C, F, Arm, A, Jaw, Got, Ok_W, C.Fast);
      end;
      Moved := 0.0;
      for K in 0 .. 2 loop
         Moved := Moved + (F.EE (Arm) (K) - EE0 (K)) ** 2;
      end loop;
      Moved := Sqrt (Moved);
      Turned := 0.0;
      for K in 3 .. 6 loop
         Turned := Turned + (F.EE (Arm) (K) - EE0 (K)) ** 2;
      end loop;
      Turned := Sqrt (Turned);
      --  这一拨,最能滑的那一块滑了多少
      Slid := 0.0;
      Any_Lost := False;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            Q : Point := Pts (I);
            W0 : constant Point := Was_R (I);
         begin
            if Natural (Q.Cam) < Natural (Before.Length) then
               Retrack (C, F, Q.Cam, Before (Natural (Q.Cam)), Q, W0.Cu, W0.Cv, True);
            end if;
            if Q.Lost or else Q.At_Edge then
               Any_Lost := True;
            end if;
            Slid := Long_Float'Max
              (Slid, Sqrt ((Q.Cu - W0.Cu) ** 2 + (Q.Cv - W0.Cv) ** 2));
            Pts.Replace_Element (I, Q);
         end;
      end loop;
      --  🔴 一推走几米:这一拨【只推了一根通道】,归因最干净 —— 当场记下来。
      --  放在循环里(而不是函数末尾),是因为函数有好几条提前返回的路,
      --  记在末尾就会整段漏掉 ⇒ 换算表永远是 0 ⇒ 米那一行永远是死的(IQ 实测喊了 8 次)。
      declare
         Ch_No : constant Natural := Arm * Chan.Per_Arm + Natural (Best_K);
      begin
         if Best_Amp > 0.0 and then Moved > 0.0
           and then Ch_No < Natural (C.Reach_M.Length)
         then
            C.Reach_M.Replace_Element (Ch_No, Moved / Best_Amp);
         end if;
      end;
      Tries := Tries + 1;
      exit when Reuse;   --  原样重用那一拨:量一次就走,不再加大
      --  🔴🔴 拨到【我还跟得住的最大那一档】,不是拨到"刚过地板"就停(ID 2026-09-15 实测)。
      --  刚过地板 = 滑动只有地板的两倍,而我要比的是【两次滑动之差】——
      --  差是滑动的一小部分,所以滑动必须【远大于】地板,差才有可能过噪声。
      --  ID 实测:0.5 mm 的拨动滑 0.003 幅(地板 0.0016),球从 0.40 m 走到 0.35 m
      --  滑动只变 0.0004 幅 —— 永远测不出来。记录 08-26 D6:步子太小信号就淹进噪声。
      --  所以按 LAB 那条老结论办:**推得够大**。跟丢了才停,那就是"我还跟得住"的边界本身。
      --  🔴 循环上限不是门槛,是【这台相机有几层金字塔】这条分辨率事实:
      --  `Levels_For (1.0)` 返回 1 ⇒ 拨一下就退出 ⇒ 上面那段"一路拨大"一次都没跑过
      --  (IE 2026-09-15 实测:每次都停在开机那一档 0.0256,滑动 0.0009 幅,永远太扁)。
      exit when Any_Lost or else Tries >= Levels_For (Long_Float (Cw)) or else not Ok_W;
      if Slid > Long_Float (Fl.Track) and then Moved > C.Map.EE_Noise then
         Good_Amp := Best_Amp; Good_Slid := Slid; Good_Moved := Moved;
         Good_Turn := Turned; Have_Good := True;
      end if;
      --  🔴 已经滑得比【那东西自己还宽】了就够了,再大就是白甩一路家具。
      --  尺寸是量出来的,不是我拍的门槛(和"碰到"用的是同一把尺)。
      exit when Have_Good and then Slid > Widest;
      --  还能拨得更大 ⇒ 先拨回去,再拨得更大(缩是修反的,记录 08-27 V2:缩了就等于把信号缩进噪声里)
      declare
         A : Table.Vec := Table.Zero_Vec;
      begin
         A (Natural (Best_K)) := -Best_Amp;
         Step_Arm (L, C, F, Arm, A, Jaw, Got, Ok_W, C.Fast);
      end;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Pts.Replace_Element (I, Was_R (I));
      end loop;
      Best_Amp := Best_Amp + Best_Amp;
   end loop;
   --  跟丢的那一档不作数,退回最后一档还跟得住的
   if Any_Lost and then Have_Good then
      declare
         A : Table.Vec := Table.Zero_Vec;
      begin
         A (Natural (Best_K)) := -Best_Amp;
         Step_Arm (L, C, F, Arm, A, Jaw, Got, Ok_W, C.Fast);
         A (Natural (Best_K)) := Good_Amp;
         Step_Arm (L, C, F, Arm, A, Jaw, Got, Ok_W, C.Fast);
      end;
      Best_Amp := Good_Amp; Slid := Good_Slid; Moved := Good_Moved; Turned := Good_Turn;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            Q : Point := Pts (I);
            W0 : constant Point := Was_R (I);
         begin
            if Natural (Q.Cam) < Natural (Before.Length) then
               Retrack (C, F, Q.Cam, Before (Natural (Q.Cam)), Q, W0.Cu, W0.Cv, True);
            end if;
            Pts.Replace_Element (I, Q);
         end;
      end loop;
      Put_Line ("[身]   📏 再大就跟丢了 ⇒ 退回还跟得住的最大一档:拨 "
                & Codec.Fmt (Good_Amp, 4) & " ⇒ 挪 " & Mm (Good_Moved)
                & " · 最能滑的滑了 " & Codec.Fmt (Good_Slid, 4) & " 幅");
   end if;
   if Slid <= Long_Float (Fl.Track) then
      Put_Line ("[身]   📏 量不了远近:拨到 " & Codec.Fmt (Best_Amp, 4)
                & " 了,最能滑的那一块也只滑了 " & Codec.Fmt (Slid, 4)
                & " 幅,没过跟踪抖动 " & Codec.Fmt (Long_Float (Fl.Track), 4)
                & " ⇒ 三角形太扁,这个数我不给");
      C.Blind_Say := S ("I tried to work out how far things are by nudging myself and watching them slide, but "
                        & "even at my biggest nudge nothing slid further than my own tracking jitter. A flat "
                        & "triangle gives a worthless distance, so I am giving you no number at all.");
      return;
   end if;
   if Moved <= C.Map.EE_Noise then
      Put_Line ("[身]   📏 量不了远近:这一拨我只挪了 " & Mm (Moved)
                & ",没过我自己的位置读数抖动 " & Mm (C.Map.EE_Noise));
      C.Blind_Say := S ("I tried to measure how far things are by nudging myself, but I only travelled "
                        & Len (C, Moved) & ", inside my own position-reading jitter. "
                        & "Too small a nudge makes the answer worthless, so I am giving you no number at all.");
      return;
   end if;
   --  这一拨在世界里往哪儿推了
   for K in 0 .. 2 loop
      Dir (K) := F.EE (Arm) (K) - EE0 (K);
   end loop;
   --  ③ 每一块滑了多远 ⇒ 这一次的"滑速";和上一次的滑速一比,就是米
   declare
      Said : Unbounded_String;
      Trav : Long_Float := 0.0;
      Got_One : Boolean := False;
   begin
      if Probe_Have then
         for K in 0 .. 2 loop
            Trav := Trav + (EE0 (K) - Probe_EE (K)) ** 2;
            Dot := Dot + Dir (K) * Probe_Dir (K);
         end loop;
         Trav := Sqrt (Trav);
         Comparable := Same_Nudge (Dot, Moved, Probe_Len, Slid, Long_Float (Fl.Track));
         if not Comparable then
            Put_Line ("[身]   📏 这两拨不是同一下:上次把手推向一个方向,这次推向另一个"
                      & "(方向一致度 " & Codec.Fmt ((if Moved * Probe_Len > 0.0
                                                    then Dot / (Moved * Probe_Len) else 0.0), 3)
                      & ")⇒ 滑速变了不代表我走近了 ⇒ 这次不出米数,把参照换成这一拨");
         end if;
      end if;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            Q : Point := Pts (I);
            W0 : constant Point := Was_R (I);
            Ran : Long_Float;
            S_Now : Long_Float;
         begin
            if Natural (Q.Cam) < Natural (Before.Length) then
               Retrack (C, F, Q.Cam, Before (Natural (Q.Cam)), Q, W0.Cu, W0.Cv, True);
            end if;
            Ran := Sqrt ((Q.Cu - W0.Cu) ** 2 + (Q.Cv - W0.Cv) ** 2);
            S_Now := Near_From_Motion (Ran, Moved, Fl.Track, C.Map.EE_Noise);
            Append (Said, (if Length (Said) > 0 then " · " else "")
                    & To_String (Q.Desc) & " 滑了 " & Codec.Fmt (Ran, 4) & " 幅");
            --  🔴🔴 走得还不到【我已经知道的那个下界】⇒ 这一段根本不可能分辨出它有多远,
            --  这一次只拿来量【滑速自己晃多少】,不出距离(IP 2026-09-15 实测:
            --  只走 2 mm 就敢报"离我 0.002 m",而球在三十厘米外)。
            --  尺度不是我拍的:上一次量出来的下界就是"至少要走这么远才谈得上分辨"。
            --  还没有下界时退回"拨一下挪多远",和以前一样。
            if Probe_Have and then Comparable and then Q.Near > 0.0 and then S_Now > 0.0
              and then Trav <= Long_Float'Max (Probe_Len, Q.Dist)
            then
               Q.Near_Jit := Long_Float'Max (Q.Near_Jit, abs (S_Now - Q.Near));
            elsif Probe_Have and then Comparable and then Q.Near > 0.0 and then S_Now > 0.0
              and then Trav > Long_Float'Max (C.Map.EE_Noise, Long_Float'Max (Probe_Len, Q.Dist))
            then
               declare
                  --  🔴 门槛的单位必须跟滑速一样是"幅每米":跟踪抖动(幅)÷ 这一拨挪了多少米。
                  --  直接拿"幅"当门槛,门槛就小了三个数量级,噪声会当场变成一个距离(IC 实测 0.014 m)。
                  --  🔴🔴 【第一次】只许给下界,不许给准数(IQ 2026-09-15 实测:
                  --  第一次量、走了 1 mm 就报"离我 0.001 m",而球在三十厘米外)。
                  --  道理:下界只要这一次的滑速就算得出来;准数要拿【两次】比,
                  --  而第一次根本没有可比的那一次 —— 手上没有尺度,就不许报尺度。
                  --  有过一次下界之后,那个下界本身就是"至少要走这么远才谈得上再量"的尺度。
                  Zd : constant Long_Float :=
                    (if Q.Dist <= 0.0 then 0.0
                     else Distance_Now (Trav, Q.Near, S_Now,
                                        Long_Float'Max (Long_Float (Fl.Track) / Moved, Q.Near_Jit)));
                  Lim : Long_Float;
               begin
                  Lim := Can_Tell_Upto (S_Now, Trav,
                                        Long_Float'Max (Long_Float (Fl.Track) / Moved, Q.Near_Jit));
                  if Zd > 0.0 and then Zd < Lim then
                     Q.Dist := Zd;
                     Append (Said, " ⇒ 离我 " & Mm (Zd));
                  elsif Lim > 0.0 then
                     --  🔴🔴 撤回(IT 2026-09-15 实测):这个数【不是"球有多远"】,是"我能分辨到多远" ——
                     --  它随着我走动一直变大(走得越远、分辨得越远)。我一度拿它当"还差多少米"去驱动
                     --  ⇒ 身体在追一个越走越远的目标。实测自相矛盾:球在画面里 78 px → 334 px(近了四倍多),
                     --  而这个数 0.203 → 0.748 m。⇒ 它只当【说明】给脑听,不当驱动量。
                     Q.Dist := 0.0;
                     Append (Said, " ⇒ 这一段我只走了 " & Mm (Trav)
                             & ",再远就分辨不出来了 —— " & Mm (Lim)
                             & " 以外我说不准(这是我看得多清楚,不是它有多远)");
                     Q.Dist := 0.0;
                     Append (Said, " ⇒ 说不准(这一段我没真的走近它)");
                  end if;
               end;
            end if;
            --  🔴🔴 参照那一次【出不了米数就不许换】(IC 实测):
            --  每拨一次就把参照换成这一次 ⇒ 两次之间永远只隔七八毫米 ⇒ 滑速差永远淹在噪声里 ⇒
            --  永远量不出米数。参照留着不动,我一路走下去,差值自己会长过噪声。
            if S_Now > 0.0 and then (Q.Near <= 0.0 or else Q.Dist > 0.0 or else not Comparable) then
               Q.Near := S_Now;
               Q.Near_N := Q.Near_N + 1;
               Got_One := True;
            end if;
            Q.Cu := W0.Cu; Q.Cv := W0.Cv; Q.Z := W0.Z;
            Pts.Replace_Element (I, Q);
         end;
      end loop;
      Put_Line ("[身]   📏 量远近:这一拨我挪了 " & Mm (Moved)
                & (if Probe_Have then " · 上次量到现在走了 " & Mm (Trav) else " · 这是第一次量,还没有走过的长度")
                & " ⇒ " & To_String (Said));
      if Turned > C.Map.Rot_Noise then
         Put_Line ("[身]   📏 ⚠️ 这一拨还转了 " & Codec.Fmt (Turned, 4)
                   & "(我的姿态读数抖动才 " & Codec.Fmt (C.Map.Rot_Noise, 4)
                   & ")⇒ 转出来的那一份和远近无关,这个数只是近似");
      end if;
      Moved_Ref := Got_One;
      if Probe_Have and then Trav > C.Map.EE_Noise then
         C.Blind_Say := S ("I worked out how far things are without any depth sensor: I nudge myself, watch how far "
                           & "a thing slides across my eye, travel, then give myself the same nudge again. A thing "
                           & "that slides more after I travelled is nearer, and how much more turns my own travel "
                           & "into its distance. One caveat I will not hide: I used how far I travelled in total, "
                           & "and the sum only equals how far I travelled TOWARD it if I went straight at it - if I "
                           & "also moved sideways, it is nearer than I just said.");
      end if;
   end;
   --  ④ 拨回去:量距离不该改变我要抓的姿势
   declare
      A : Table.Vec := Table.Zero_Vec;
   begin
      A (Natural (Best_K)) := -Best_Amp;
      Step_Arm (L, C, F, Arm, A, Jaw, Got, Ok_W, C.Fast);
   end;
   --  🔴🔴 顺手把【一推走几米】记下来:这一拨只推了一根通道,归因最干净。
   --  IO 2026-09-15 实测非记不可:这个换算原来只在探针里量,而身体一旦【装回存好的表】
   --  探针就不跑 ⇒ 换算表全是 0 ⇒ 尺子量出来的米进了解算也是死的,
   --  差距五步纹丝不动(0.719 / 0.731 / 0.729 / 0.724 / 0.719)而日志全绿。
   declare
      Ch_No : constant Natural := Arm * Chan.Per_Arm + Natural (Best_K);
   begin
      if Best_Amp > 0.0 and then Moved > 0.0
        and then Ch_No < Natural (C.Reach_M.Length)
      then
         C.Reach_M.Replace_Element (Ch_No, Moved / Best_Amp);
      end if;
   end;
   Probe_K := Best_K; Probe_Amp := Best_Amp;
   if not Probe_Have or else Moved_Ref or else not Comparable then
      Probe_EE := EE0;
      Probe_Dir := Dir;
      Probe_Len := Moved;
   end if;
   Probe_Have := True;
end Range_Probe;
