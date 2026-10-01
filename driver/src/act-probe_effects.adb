separate (Act)
procedure Probe_Effects (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                         Effs : in out Effect_Array; Trust : out Table.Mask; Ok : out Boolean;
                         Allow : Long_Float := 1.0) is
   --  Pts 空的时候 `Pts (0)` 当场越界,而它在声明区 ⇒ 异常记在【调用处】,
   --  栈里根本看不到这个子程序这一帧(实测查了半天)。空就当第 0 条胳膊,下面第一句直接回。
   Pts_Empty : constant Boolean := Natural (Pts.Length) = 0;
   Arm : constant Natural := (if Pts_Empty then 0 else Pts (0).Arm);
   P0 : constant Plug.Arm_Pose := F.EE (Arm);
   Jaw : Floats;
   --  (路 1,10-01)驱动不读身体给的深度(Plug.Frame_Of:Has_Depth 恒为 False),原来在这里量的四样地板 —— 跟踪 4 像素、
   --  远近、看着多大、朝向 —— 只在 Has_Depth 那一支里量,那一支从来没跑过:远近 / 看着多大 / 朝向三样地板恒为 0,
   --  而"这一推有没有一个点动过地板"写成 "挪过 4 像素 或者 远近变过远近的地板",后一半 0 ≥ 0 恒真 ⇒ 4 像素那一半从来不起作用。
   --  删掉以后行为逐位不变:动没动过只按命令实到超过读数噪声判(画面里挪多少不进这一判;要不要换成量出来的跟踪地板,报主代理了)
   --  这条臂的位姿通道(问身体图,大并行 I1):响应表第 K 列 = 第 K 个位姿通道,通道号 = Chs (K);不按"臂 × 每臂几个 + K"算
   Chs : constant Ints := Selfmap.Graph.Pose_Channels (C.Map, Arm);
   N_Ch : constant Natural := Natural (Chs.Length);
   --  一次推动只证明"它动过",证明不了"它稳"。同一个推法重复这么多次,量散布(次数,无量纲)。
   Reps_Wanted : constant := 3;
   N_Pts : constant Natural := Natural (Pts.Length);
   type Sum_Grid is array (0 .. N_Pts - 1, 0 .. N_Ch - 1, 0 .. Table.Rows - 1) of Long_Float;
   S1 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值之和
   S2 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值平方和
   --  🔴🔴 来回对表(2026-08-27 NV3 第一次上机就抓到一个符号错:分歧 2.539 / 共识 0.019):
   --  同一根通道 +δ 走一遍、−δ 走回来一遍,两遍各除以【自己那一遍的实到】⇒ 结果应当相等。
   --  不相等 = 这一列不是一个测量(跟丢了 / 符号错了 / 关节翻支了),而光看去程那一遍看不出来。
   --  判据零系数:两遍的【分歧】要小于两遍的【共识】。
   B1 : Sum_Grid := [others => [others => [others => 0.0]]];   --  去程那一遍的列
   B2 : Sum_Grid := [others => [others => [others => 0.0]]];   --  回程那一遍的列
   Nb : array (0 .. N_Ch - 1) of Natural := [others => 0];
   Agree_Out : Table.Vec := [others => -1.0];   --  每根通道:分歧 ÷ 共识(<1 才算稳)
   --  🔴 每(通道,行)自己的来回对账结果。对不上的【那一行】清零 = "这根通道对这一行没有意见"。
   --  一开始全是 True:没对过表的行照原样用(没量过不等于量出来是错的)。
   Row_Ok : array (0 .. N_Ch - 1, 0 .. Table.Rows - 1) of Boolean := [others => [others => True]];
   Said_Wide : array (0 .. N_Ch - 1) of Boolean := [others => False];
   Nrep : array (0 .. N_Ch - 1) of Natural := [others => 0];
   --  🔴 上一轮(幅度的一半)这一通道最多的那个点跑了多远。加倍之后【一点没多跑】⇒ 再加也没用,
   --  这一列就是零 —— 零本身是一次正确的测量("这个通道不动它")。
   --  HC 实测:腕转那几根一路加码到 0.8192 rad(47°,owner 看 JA 视频原话"机械臂全程在发癫"),
   --  每一档都是 0.0000 画幅,加了五档等于白甩五次。
   Last_Ran : array (0 .. N_Ch - 1) of Long_Float := [others => -1.0];
   --  重复够了(或者中途翻脸了)⇒ 把均值写进表,把散布÷|均值| 写进散布格
   procedure Finalise (K : Natural) is
   begin
      for I in 0 .. N_Pts - 1 loop
         declare
            Nk : constant Long_Float := Long_Float (Natural'Max (1, Nrep (K)));
            Mean, Sc : Table.Vec3;
         begin
            for R in 0 .. Table.Rows - 1 loop
               Mean (R) := S1 (I, K, R) / Nk;
               declare
                  Var : constant Long_Float := Long_Float'Max (0.0, S2 (I, K, R) / Nk - Mean (R) * Mean (R));
               begin
                  Sc (R) := (if abs Mean (R) > 0.0 then Sqrt (Var) / abs Mean (R) else 0.0);
               end;
            end loop;
            --  🔴 来回对不上的那几行,写进表里的是零 —— "这根通道对这一行没有意见"。
            --  留着它反而更坏:解算会拿一个假的斜率去修那一行,越修越远(HZ 实测深度行如此)。
            --  🔴🔴 但【画面那两行不许单独清】(IL 2026-09-15 实测):
            --  "这根通道能不能用"是拿画面两行【合起来】判的,合起来判过了、分开判却把两行都清零
            --  ⇒ 通道留着却什么都不贡献 ⇒ 身体又没了横向本钱,正是 HZ 那个瘫痪的翻版,
            --  而这一次是我自己的行清零造成的。IL 原话:通道 7 共识 141.6「对得上,信得过」,
            --  同一炮的表里却是 ch7(左右 0.000 没证过)。
            --  一对量、一个判决 ⇒ 要留一起留,要清一起清(由 Trust 决定),不许分开。
            for R in 2 .. Table.Rows - 1 loop
               if not Row_Ok (K, R) then
                  Mean (R) := 0.0;
                  Sc (R) := 0.0;
               end if;
            end loop;
            Table.Set_Col (Effs (I), K, Mean);
            Table.Set_Spread (Effs (I), K, Nrep (K), Sc);
         end;
      end loop;
   end Finalise;
begin
   if Pts_Empty then
      Trust := [others => False];
      Ok := False;
      return;
   end if;
   Trust := [others => False];
   Jaw := Selfmap.Jaw_All (F, Arm);
   for I in 0 .. Natural (Pts.Length) - 1 loop
      Table.Reset (Effs (I), N_Ch, 1.0);
      for K in 0 .. N_Ch - 1 loop
         declare
            Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Chs (K)));
         begin
            Table.Set_Prior (Effs (I), K, 100.0 / (Am * Am));   --  先验按探针幅度定(倍数,无量纲)
         end;
      end loop;
   end loop;
   Ok := True;
   Put_Line ("[身]   这些点还没有响应表 ⇒ 这条臂的 " & Codec.Img (N_Ch) & " 个位姿通道各推一下量列(幅度从开机看得见的那一档起;命令实到没超过读数噪声才翻倍)");
   --  这条臂的位姿通道一起解:转动不禁(owner 2026-09-07:禁了就永远和桌面平行,格斗全成直线)。让转动有对错的是"两根手指各自到位":
   --  转歪了必有一指不到位;让转动不比平移便宜的是按各自探针幅度计价。
   for K in 0 .. N_Ch - 1 loop
      declare
         Chn : constant Natural := Chs (K);
         Amp : Long_Float := C.Map.Amp (Chn);
         --  🔴🔴 "能看见它动的那一档"(C.Map.Amp)是【开机时在某一台相机里】量的,而它被所有相机通用。
         --  同样推一下关节,手在不长在这条胳膊上的相机里跑的画幅小得多 ⇒ 在那台相机里还没推到看得见,
         --  就先撞上限被扔掉 ⇒ 表只剩几列噪声 ⇒ 符号都能算反。
         --  GO 实测:头顶相机里推 0.0222 rad,点跑了 0.0000 画幅 ⇒ 整根通道被扔;
         --  拿这种表解出来的命令把胳膊一路推出画面右边缘,而"还差几步"一路从 29.8 "改善"到 20.0。
         --  放宽多少不用人拍:身体开机就量了 Cam_Frac(这条胳膊一动,每台相机的画面各变多少)——
         --  哪台看得小,上限就按【看得最大的那台 ÷ 这一台】的比例放大。全是量出来的。
         --  上限 = 脑让它动的那一档 × 这台相机看得出动过所需要的放宽;至少是自己那一档,否则一步都探不出来
         Cap_Amp : constant Long_Float :=
           C.Map.Amp (Chn) * Long_Float'Max (1.0, Allow) * Cam_Slack (C, Arm, Cam);
      begin
         if not C.Map.Seen (Chn) or else Amp <= 0.0 then
            Put_Line ("[身]     通道" & Natural'Image (Chn) & " 开机时没看见它动,这一列留零");
         else
            loop
               declare
                  A : Table.Vec := Table.Zero_Vec;
                  Before_All : constant Buf_Vectors.Vector := All_Gray (F);
                  Was : constant Point_Vectors.Vector := Pts;
                  Deliv, Back : Table.Vec;
                  Ok2 : Boolean;
                  Frames : Natural;
                  --  🔴 只要【有一个】被跟的点真的动过,这一列就算量到了。
                  --  以前是"任一个点没动 ⇒ 整条通道作废",于是两指里被挡住一根就扔掉一整个自由度:
                  --  FQ 实测 6 个通道扔掉 5 个,只剩 1 个还想管三个方向 ⇒ "还差几步"算出 5528 步、手来回摆。
                  Seen_Enough : Boolean := False;
                  N_Moved : Natural := 0;
                  Ran_Max : Long_Float := 0.0;
               begin
                  A (K) := Amp;
                  declare
                     Ee0 : constant Plug.Arm_Pose := F.EE (Arm);
                  begin
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok2);
                     if not Ok2 then
                        Ok := False;
                        return;
                     end if;
                     --  🔴 顺手量下这一推【手在世界里真走了几米】:尺子量出来的米要接进解算,
                     --  就靠这个换算(还差几米 ÷ 一推走几米 = 还差几步)。关节读数给的,不碰深度图。
                     declare
                        Dm : Long_Float := 0.0;
                        Ch_No : constant Natural := Chn;
                     begin
                        for Q in 0 .. 2 loop
                           Dm := Dm + (F.EE (Arm) (Q) - Ee0 (Q)) ** 2;
                        end loop;
                        Dm := Sqrt (Dm);
                        if Amp > 0.0 and then Ch_No < Natural (C.Reach_M.Length) then
                           C.Reach_M.Replace_Element (Ch_No, Dm / Amp);
                        end if;
                     end;
                  end;
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     declare
                        P : Point := Pts (I);
                        W0 : constant Point := Was (I);
                        Ran : Long_Float;
                     begin
                        Retrack (C, F, P.Cam, Before_All (P.Cam), P, W0.Cu, W0.Cv, True);
                        Ran := Sqrt ((P.Cu - W0.Cu) ** 2 + (P.Cv - W0.Cv) ** 2);
                        Ran_Max := Long_Float'Max (Ran_Max, Ran);
                        if abs Deliv (K) > C.Map.EE_Noise then
                           declare
                              Col : Table.Vec3;
                           begin
                              Col (0) := (P.Cu - W0.Cu) / Deliv (K);
                              Col (1) := (P.Cv - W0.Cv) / Deliv (K);
                              Col (2) := (if P.Z > 0.0 and then W0.Z > 0.0 then (P.Z - W0.Z) / Deliv (K) else 0.0);
                              --  推一下这块看着变大变小多少、转了多少(圆的东西转不出来 ⇒ 这一列恒零 ⇒ 自动不参与)
                              --  只有变化过了自己的噪声地板才敢写进表,否则这一格留零(留零 = 归一时这一行自动不参与)
                              Col (3) := (if P.Size > 0.0 and then W0.Size > 0.0 then (P.Size - W0.Size) / Deliv (K) else 0.0);
                              Col (4) := (if P.Size > 0.0 and then W0.Size > 0.0 then Wrap (P.Ang - W0.Ang) / Deliv (K) else 0.0);
                              for R in 0 .. Table.Rows - 1 loop
                                 S1 (I, K, R) := S1 (I, K, R) + Col (R);
                                 S2 (I, K, R) := S2 (I, K, R) + Col (R) * Col (R);
                                 B1 (I, K, R) := Col (R);   --  去程这一遍,留着和回程对
                              end loop;
                           end;
                           Seen_Enough := True;
                           N_Moved := N_Moved + 1;
                        end if;
                        --  没动过的点这一列【留零】,而留零本身就是一次正确的测量("这个通道不动它"),归一时它自动不参与
                        Pts.Replace_Element (I, P);
                     end;
                  end loop;
                  if Seen_Enough then
                     Nrep (K) := Nrep (K) + 1;
                     Put_Line ("[身]     通道" & Natural'Image (Chn) & " 第" & Natural'Image (Nrep (K)) & " 次:命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ " &
                               Natural'Image (N_Moved) & "/" & Natural'Image (Natural (Pts.Length)) & " 个点动了,最多的跑了 " &
                               Codec.Fmt (Ran_Max, 4) & " 画幅,深度变 " & Codec.Fmt ((if Pts (0).Z > 0.0 and then Was (0).Z > 0.0 then Pts (0).Z - Was (0).Z else 0.0), 4));
                  end if;
                  declare
                     Before2_All : constant Buf_Vectors.Vector := All_Gray (F);
                  begin
                     Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Back, Frames, Ok2);
                     if not Ok2 then
                        Ok := False;
                        return;
                     end if;
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        declare
                           P : Point := Pts (I);
                           Wb : constant Point := Pts (I);   --  回程之前(= 去程走完)那一刻
                        begin
                           Retrack (C, F, P.Cam, Before2_All (P.Cam), P, Was (I).Cu, Was (I).Cv, True);
                           --  🔴 回程也量一遍同一列:除以【回程自己的实到】(反号),两遍应当相等
                           if abs Back (K) > C.Map.EE_Noise and then not P.Lost then
                              B2 (I, K, 0) := (P.Cu - Wb.Cu) / Back (K);
                              B2 (I, K, 1) := (P.Cv - Wb.Cv) / Back (K);
                              B2 (I, K, 2) := (if P.Z > 0.0 and then Wb.Z > 0.0 then (P.Z - Wb.Z) / Back (K) else 0.0);
                              Nb (K) := Nb (K) + 1;
                           end if;
                           P.Cu := Was (I).Cu; P.Cv := Was (I).Cv; P.Z := Was (I).Z;   --  推回起点了:点回到原处(比光流往返的累积误差可信)
                           Pts.Replace_Element (I, P);
                        end;
                     end loop;
                  end;
                  --  🔴 一推跑得比眼睛一步跟得住的还远 ⇒ 【不是把推的幅度缩小】,而是【把搜索范围放宽】。
                  --  记录 2026-08-27 V2:窗口比真实位移小的时候,模板搜索会静默返回一个完全错误的位置;
                  --  记录 2026-08-26 D6:探针步子太小 ⇒ 信号和噪声一样大,一列只解释掉 42%。
                  --  所以两条合起来只有一个做法:**推得够大,搜得够宽**。我 09-15 一度改成缩幅度,是修反了,已撤。
                  --  这里只如实说出来,幅度不动。
                  if Ran_Max > Track_Win and then not Said_Wide (K) then
                     Said_Wide (K) := True;
                     Put_Line ("[身]     通道" & Natural'Image (Chn) & ":这一推让点跑了 " & Codec.Fmt (Ran_Max, 4)
                               & " 画幅,比眼睛一步跟得住的 " & Codec.Fmt (Track_Win, 4)
                               & " 还远 ⇒ 我不缩这一推,改成整幅画面都找(缩了就等于把信号缩进噪声里)");
                  end if;
                  if Seen_Enough then
                     --  🔴 来回对账:去程和回程量出来的同一列应当相等。
                     --  分歧 = 两遍之差的长度;共识 = 两遍之和的一半的长度。分歧 ≥ 共识 ⇒ 这一列不是测量。
                     --  🔴🔴 一行一判(HZ 2026-09-15 实测改):以前把五行【合成一个数】来判整根通道,
                     --  于是被放大了二三十倍、且一动不动也在乱跳的【深度那一行】,单独一行就能把一整根
                     --  【画面里量得准准的】通道否掉。HZ 实测:6 根判死 4 根,活下来的两根左右都是 0.000
                     --  ⇒ 解算连着 10 步命令全零、身体一动不动,而日志每一行都是绿的。
                     --  改成:画面那两行(左右/上下)说了算"这根通道能不能用";其余各行自己对自己负责,
                     --  哪一行来回对不上就把【那一行】清零 —— 清零的意思是"这根通道对这一行没有意见",
                     --  不是"它是零"。⚠️ 不许拿体检那个倍数去除深度:那是灵敏度不是绝对尺度错,
                     --  而且解算里误差和列都用同一套读数单位,倍数本来就会约掉(HY 实测除了就炸)。
                     if Nb (K) > 0 and then Agree_Out (K) < 0.0 then
                        declare
                           Dp, Cp : Long_Float := 0.0;
                           Dropped : Natural := 0;
                        begin
                           for R in 0 .. Table.Rows - 1 loop
                              declare
                                 Dr, Cr : Long_Float := 0.0;
                              begin
                                 for I in 0 .. Natural (Pts.Length) - 1 loop
                                    Dr := Dr + (B1 (I, K, R) - B2 (I, K, R)) ** 2;
                                    Cr := Cr + ((B1 (I, K, R) + B2 (I, K, R)) / 2.0) ** 2;
                                 end loop;
                                 Dr := Sqrt (Dr); Cr := Sqrt (Cr);
                                 Row_Ok (K, R) := Row_Is_Measurement (Dr, Cr);
                                 if R <= 1 then
                                    Dp := Dp + Dr * Dr;
                                    Cp := Cp + Cr * Cr;
                                 elsif not Row_Ok (K, R) then
                                    Dropped := Dropped + 1;
                                 end if;
                              end;
                           end loop;
                           Dp := Sqrt (Dp); Cp := Sqrt (Cp);
                           if Cp > 0.0 then
                              Agree_Out (K) := Dp / Cp;
                              Put_Line ("[身]     通道" & Natural'Image (Chn) & " 来回对表(画面那两行):分歧 "
                                        & Codec.Fmt (Dp, 4) & " · 共识 " & Codec.Fmt (Cp, 4)
                                        & " ⇒ " & (if Agree_Out (K) < 1.0 then "对得上,这根通道信得过"
                                                   else "🔴 对不上,这根通道不是测量(跟丢/符号反/关节翻支)"));
                           end if;
                           if Dropped > 0 then
                              Put_Line ("[身]     通道" & Natural'Image (Chn) & ":其中 " & Codec.Img (Dropped)
                                        & " 行(远近/看着多大/朝向)来回对不上 ⇒ 这几行清零,"
                                        & "这根通道对它们没意见;画面那两行照用");
                           end if;
                        end;
                     end if;
                     if Nrep (K) >= Reps_Wanted then
                        Finalise (K);
                        Trust (K) := Agree_Out (K) < 0.0 or else Agree_Out (K) < 1.0;
                        exit;
                     end if;
                     --  同一幅度再来一次(不翻倍):现在要证的是"它稳",不是"它动过"
                  else
                     if Nrep (K) > 0 then
                        --  前面动过,这一次同样的推法没动 ⇒ 这就是"不稳"本身,如实收档交给体检判
                        Finalise (K);
                        Trust (K) := True;
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":同一个推法第" & Natural'Image (Nrep (K) + 1) & " 次没动 ⇒ 不稳,如实记下");
                        exit;
                     end if;
                     if Amp * 2.0 > Cap_Amp then
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 命令实到都没超过读数噪声 " & Codec.Fmt (C.Map.EE_Noise, 4)
                                  & "(点最多跑了 " & Codec.Fmt (Ran_Max, 4) & " 画幅)⇒ 这一段不用它");
                        exit;
                     end if;
                     --  加倍了却一点没多跑 ⇒ 这一列是零,再加码只是空甩胳膊
                     if Last_Ran (K) >= 0.0 and then Ran_Max <= Last_Ran (K) then
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":加倍到 " & Codec.Fmt (Amp, 4) &
                                  " 之后点一点没多跑(" & Codec.Fmt (Last_Ran (K), 4) & " ⇒ " & Codec.Fmt (Ran_Max, 4) &
                                  " 画幅)⇒ 这一列就是零,不再加码空甩");
                        exit;
                     end if;
                     Last_Ran (K) := Ran_Max;
                     Amp := Amp * 2.0;
                  end if;
               end;
            end loop;
         end if;
      end;
   end loop;
   --  🔴 一根都没量到 ⇒ 这一段根本无从下手,必须当场说给脑听,并且说清【它能怎么办】。
   --  HH 实测:我给的程序里只有第一行写了 large,后两行默认只给一半幅度;而在【手自己那台相机】里
   --  按相机放宽的系数是 1 ⇒ "量自己"的上限正好等于起始档 ⇒ 一次都加不了 ⇒ 六根全被扔、
   --  表是空的、命令恒零,连着六步 `还差 0.0 步 · 命令 [0.000 ×6]`,读日志像"已经到位了"。
   --  身体当时每根都老实说了"这一段不用它",但没有一句话说"合起来 = 我这一段动不了"。
   if (for all K in 0 .. N_Ch - 1 => not Trust (K)) then
      Put_Line ("[身]   🔴 这一段一根通道都没量到 ⇒ 我没有任何一条能用的走法。"
                & "你给的幅度那一档不够我看清自己动了没有。");
      C.Blind_Say := S ("with the step size you gave me I could not see any of my channels move in this eye, "
                        & "so I have no usable way to move at all for this stretch - "
                        & "say a larger step, or judge this stretch with another eye");
   end if;
end Probe_Effects;
