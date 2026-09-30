separate (Act)
procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
   Cam : constant Natural := Natural'Min (C.Cam, Natural (F.Cams.Length) - 1);
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   RGB : Buf := F.Cams (Cam).RGB;
   Listing : Unbounded_String;
   Say : Brain.Say;
   Err : Unbounded_String;
   Report : Unbounded_String;
begin
   C.Round_N := C.Round_N + 1;
   C.Cam := Cam;
   Feel (C, F);   --  先感觉手在哪(按位姿查身体图),不看
   --  切块 → 世界槽
   World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
   Draw.Grid (RGB, Cw, Ch, C.Cols, C.Rows, C.Cells_U, C.Cells_V);
   Build_Listing (C, F, Cam, RGB, Listing);
   --  🔴 每一台相机都切块、都编号,号全局唯一。以前只有当前这一台有号,别的相机在条带里
   --  连个框都没有 ⇒ 脑看得见却点不了名(GM:我答"一个都不是",而头顶相机里球一直看得见)。
   --  先把位子占够:Shown 是空向量时 C.Shown (K) 会当场 CONSTRAINT_ERROR 把炮打死
   while Natural (C.Shown.Length) < Natural (F.Cams.Length) loop
      C.Shown.Append (Bytes.U8_Vectors.Empty_Vector);
   end loop;
   for K in 0 .. Natural (F.Cams.Length) - 1 loop
      if K /= Cam and then F.Cams (K).W > 0 then
         declare
            Kw : constant Natural := F.Cams (K).W;
            Kh : constant Natural := F.Cams (K).H;
            Sub : Unbounded_String;
         begin
            if Natural (F.Cams (K).RGB.Length) >= Kw * Kh * 3 then
               C.Shown (K) := F.Cams (K).RGB;
               World.Observe (C.Wld, K, Cut_Things (C, F, K), Kw, Kh);
               Build_Listing (C, F, K, C.Shown (K), Sub, Keep => True, Things_Only => True);
               Append (Listing, Sub);
            end if;
         end;
      end if;
   end loop;
   Put_Line ("[身] ── 第" & Natural'Image (C.Round_N) & " 轮(第" & Natural'Image (Cam) & " 台相机)── 这一集已用 " & Codec.Img (Plug.Steps (L)) & " 拍(开机量身体 " & Codec.Img (C.Boot_Steps) & " 拍)");
   Put (To_String (Listing));
   if C.Dump_Dir /= "" then
      Codec.Write_BMP (To_String (C.Dump_Dir) & "/grid_" & Codec.Pad6 (C.Round_N) & ".bmp", RGB, Cw, Ch);
   end if;
   declare
      --  拍数只进日志(我们自己记账),不进问脑的话:真实世界没有"步",脑只看画面。
      --  🔴 一段程序跑了好几节 ⇒ 把【每一节】的结果都给它,不是只给最后一节。
      Recent : constant String := Memory.Text (C.Mem)
        & (if Length (C.Prog_Log) > 0 then To_String (C.Prog_Log) else To_String (C.Recent));
   begin
      --  🔴 每一轮把【所有相机】一起给脑:编号那张在上面(格子和编号只管它),其余几只眼睛按半幅
      --  拼在下面一条。以前一轮只给一台,想看别的得先说"下一轮换一台" —— 那是【盲切】:说完看不到
      --  结果,切过去才发现手腕正对着墙(DS/EI/EA 各记过一次)。而且切一台就丢一轮,跨相机的判断
      --  根本做不了。下面那一条不画格子、不编号 —— 它只是"我另外几只眼睛现在看见什么"。
      declare
         Sh : constant Natural := Ch / 2;
         Sw : constant Natural := Cw / 2;
         Bh : constant Natural := Ch + (if C.Map.N_Cams > 1 then Sh else 0);
         Big : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Bh * 3));
         Slot : Natural := 0;
      begin
         for I in 0 .. Cw * Ch * 3 - 1 loop
            Big.Replace_Element (I, RGB.Element (I));
         end loop;
         if C.Map.N_Cams > 1 then
            for K in 0 .. Natural (F.Cams.Length) - 1 loop
               if K /= Cam and then Slot * Sw < Cw then
                  declare
                     Kw : constant Natural := F.Cams (K).W;
                     Kh : constant Natural := F.Cams (K).H;
                     Ox : constant Natural := Slot * Sw;
                  begin
                     if Natural (C.Shown (K).Length) >= Kw * Kh * 3 then
                        for Y in 0 .. Sh - 1 loop
                           for X in 0 .. Sw - 1 loop
                              declare
                                 Sx : constant Natural := Natural'Min (Kw - 1, X * Kw / Sw);
                                 Sy : constant Natural := Natural'Min (Kh - 1, Y * Kh / Sh);
                                 D : constant Natural := ((Ch + Y) * Cw + Ox + X) * 3;
                                 Sp : constant Natural := (Sy * Kw + Sx) * 3;
                              begin
                                 if Ox + X < Cw then
                                    Big.Replace_Element (D, C.Shown (K).Element (Sp));
                                    Big.Replace_Element (D + 1, C.Shown (K).Element (Sp + 1));
                                    Big.Replace_Element (D + 2, C.Shown (K).Element (Sp + 2));
                                 end if;
                              end;
                           end loop;
                        end loop;
                        Draw.Numbered_Box (Big, Cw, Bh, Ox, Ch, Natural'Min (Cw - 1, Ox + Sw - 1), Bh - 1,
                                           K + 1, Draw.White, 2);
                     end if;
                     Slot := Slot + 1;
                  end;
               end if;
            end loop;
         end if;
      --  🔴 脑交上来的是【一段程序】,不是一张表。收到之后:解析 → 对着体检判决编译 → 过了才存起来跑。
      --  退回是免费的:一根手指都不动,理由和一个能照抄的替代随下一轮一起给它。
      --  🔴 脑交上来的是【一段 Sinew 程序】。收到之后:解析 → 把名词落到具体的块上 →
      --  对着体检判决整段检查 → 过了才存起来跑。退回是免费的:一根手指都不动。
      if not C.Have_Prog then
         declare
            Text : Unbounded_String;
            --  键盘只给这具身体、这一版、此刻真按得动的键。三张表都当场从驱动自己的判定里生成:
            --  关系问 Plan.Usable_Rels(体检报告说哪几行量过了)· 结局问 Plan.Waitable_Outcomes
            --  (和拒绝语共用 Oc_Waitable)· 角色问 Role_Wants(绑不上的角色不给)。
            --  Any_Stands = 此刻有没有哪一块量得出它鼓出所靠的面多少 —— 没有就谈不上 onto/off/into/free。
            Rep0 : constant Exam.Report := Exam.Judge (C.Map, C.Tables);
            Any_Stands : Boolean := False;
            Roles : Unbounded_String;
            --  🔴 关系键盘按【此刻角色绑得上的那几个"我"】逐个问编译器那一套(Plan.Usable_Rels_Any),
            --  不再拿 -1 去扫"量过的每一块":响应表空的时候那样扫恒为空,键盘上一个移动词都没有,
            --  而表只有执行移动命令才会去量 ⇒ 死锁(SC4 实测,`0 张响应表` ⇒ 关系 [(一个都没有)])。
            All_Facts : constant Plan.Facts_Vectors.Vector := Build_Facts (C, F);
            Subjects : Plan.Facts_Vectors.Vector;
            Rels : Unbounded_String;
            Qtys : Unbounded_String;   --  这一轮能说的【东西的量】(身体列的;有 = 键盘上只给"量往哪变"那一句)
         begin
            for I in 0 .. Natural (C.Items.Length) - 1 loop
               if C.Items (I).Height > 0.0 then
                  Any_Stands := True;
               end if;
            end loop;
            --  🔴 CS2 实测:按 Role_Wants 这张【静态表】给键,pusher 就被给了出去,
            --  而这具身体这一帧根本没有能绑上的 Piece ⇒ 28 次绑定 28 次失败,
            --  程序卡在那儿走不到"送进两指之间",接触集一次都没跑到。
            --  ⇒ 键盘按【此刻真绑得上谁】给:清单里有没有这个角色要的那种块。
            for R in Sinew.Role loop
               declare
                  Any : Boolean := False;
               begin
                  for I in 0 .. Natural (C.Items.Length) - 1 loop
                     if C.Items (I).Located and then Role_Wants (R, C.Items (I).Kind) then
                        Any := True;
                        --  Build_Facts 的第 0 条是占位,第 I+1 条才是清单第 I 件
                        if I + 1 < Natural (All_Facts.Length) then
                           Subjects.Append (All_Facts (I + 1));
                        end if;
                     end if;
                  end loop;
                  if Any then
                     Append (Roles, (if Length (Roles) > 0 then " " else "") & Sinew.Role_Word (R));
                  end if;
               end;
            end loop;
            --  🔴 CS5 实测:整份日志里查不到【这一轮键盘上到底有哪些键】—— 语法一个字都没进日志。
            --  于是"零死键"这条前置条件在事后无法核对,任何"给了它键它不用"的判决都建立在没记录的假设上。
            --  ⇒ 把当场生成的三张表如实记一行。这行只写日志,不参与任何判定。
            Rels := To_Unbounded_String (Plan.Usable_Rels_Any (Rep0, Subjects, Any_Stands));
            --  语言的根(2026-09-23):清单里有点过名的东西、我又有能合拢的手 ⇒ 键盘上只给"它的量往哪变"这一句;
            --  量的名字是身体列的(现在只有 height:离它躺的面多高 —— 两眼视线交点对它躺的面量出来)
            --  东西由脑在句子里点名(名字是自由的,身体去认);所以只要我有一只能合拢的手,这一句就在键盘上
            Qtys := To_Unbounded_String (Qty_Words (C, To_String (Roles)));
            Put_Line ("[身] 🎹 这一轮键盘:" & (if Length (Qtys) > 0 then "量 [" & To_String (Qtys) & "] · " else "")
                      & "关系 [" & To_String (Rels)
                      & "] · 角色 [" & To_String (Roles)
                      & "] · 结局 [" & Plan.Waitable_Outcomes (Any_Stands)
                      & "] · 清单 " & Codec.Img (Natural (C.Items.Length)) & " 件");
            if not Brain.Ask (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                              Sinew.Grammar (To_String (Rels), To_String (Roles),
                                             Plan.Waitable_Outcomes (Any_Stands), To_String (Qtys)),
                              To_String (C.Refused),
                              To_String (Rels), To_String (Roles),
                              Plan.Waitable_Outcomes (Any_Stands),
                              C.Cols, C.Rows, Natural (C.Items.Length), C.Map.N_Cams, C.Map.Arms, Big, Cw, Bh, Text, Err,
                              Qtys_Usable => To_String (Qtys))
            then
               --  🔴 装不下是【量得到的事实】,不是猜:回包里就写着限额和用量。
               --  照着把清单上限减半再来,减到装得下为止;以前只会一股脑全给,
               --  实测 3589 轮里 3577 轮撞墙 —— 脑几乎从没真正看见过画面。
               if (for some I in 1 .. Length (Err) - 21 =>
                     Slice (Err, I, I + 21) = "maximum context length")
               then
                  declare
                     Now_N : constant Natural := World.Count (C.Wld, Cam);
                     Was : constant Natural := C.List_Cap;
                  begin
                     C.List_Cap := (if C.List_Cap = 0 then Natural'Max (1, Now_N / 2)
                                    else Natural'Max (1, C.List_Cap / 2));
                     Put_Line ("[身] 🧠 你读不下这么长:清单上限 "
                               & (if Was = 0 then "不限" else Codec.Img (Was)) & " ⇒ " & Codec.Img (C.List_Cap)
                               & " 件(只留最大的,漏掉的我会说出来)");
                  end;
               end if;
               Put_Line ("[身] 🧠 问不通(" & To_String (Err) & ")⇒ 这一拍不动,下一拍重问");
               return;
            end if;
            Put_Line ("[身] 🧠 它交上来一段程序:");
            Put_Line (To_String (Text));
            --  🔴 一字不差地重复了上一段,而上一段一根手指都没动过 ⇒ 如实说出来。
            --  不给窍门、不给例句,只报这一个事实 —— 它是真的,而且脑看不到就永远出不了这个圈。
            if Text = C.Last_Prog and then not C.Last_Moved then
               C.Recent := C.Recent
                 & " You just gave me the very same program again, word for word, and the one before it"
                 & " did not move any part of me: nothing in the picture changed because of it.";
               Put_Line ("[身] 🧠 这一段和上一段一字不差,而上一段没让我动过 ⇒ 已如实告诉它");
            end if;
            C.Last_Prog := Text;
            C.Last_Moved := False;   --  这一段走过步就会被置回 True
            declare
               Rep : constant Exam.Report := Exam.Judge (C.Map, C.Tables);
               P : constant Sinew.Program := Sinew.Parse (To_String (Text));
               --  🔴 脑点的眼睛要在【绑定和选眼之前】就位。第一版把 C.Eye_Want 放在执行指令时赋值,
               --  而顺序是 解析 → 绑定 → 选眼 → 执行 ⇒ 赋值永远晚一步,自动选眼照样把段拽走
               --  (GZ 实测:写了 with my still eye,日志里还是"这条胳膊一动,第2 只眼睛…换过去")。
               --  取第一条 do 的那个选择:一段程序里几节用不同眼睛是后话,现在按整段一个眼睛算。
               function First_Eye return Sinew.Eye_Pick is
               begin
                  for K in 0 .. Natural (P.Code.Length) - 1 loop
                     if P.Code (K).O = Sinew.Op_Interval
                       and then Sinew."/=" (P.Code (K).Eye, Sinew.Ey_None)
                     then
                        return P.Code (K).Eye;
                     end if;
                  end loop;
                  return Sinew.Ey_None;
               end First_Eye;
               Facts : Plan.Facts_Vectors.Vector := Build_Facts (C, F);
               Binds : Plan.Bind_Vectors.Vector;
               V : Plan.Verdict;

               --  角色靠【量出来的东西】绑定:grasper = 我量到能相向靠拢并夹住东西的那一组。
               --  🔴 挑哪一只手【不许】用"这张画面里离它最近":在一只长在【另一条胳膊】上的眼睛里,
               --  那条胳膊的手可以正好投影在球旁边而实际隔着半张桌子(GD 实测:在手2的眼睛里
               --  绑到了手1,于是两只眼睛之间来回弹,一步不走)。这和最初那个 bug 是同一个病:
               --  拿一个看不见这件事的视角去判空间关系。
               --  规矩改成:我现在这只眼睛长在哪条胳膊上,就用那条胳膊;这只眼睛不长在任何胳膊上
               --  (它看得见全场),才在这里比远近。
               function Bind_Role (R : Sinew.Role; Near_U, Near_V : Long_Float; Has_Near : Boolean) return Integer is
                  Own : constant Integer := Cam_Arm (C, C.Cam);
                  Best : Integer := -1;
                  Bd : Long_Float := 1.0e9;
               begin
                  --  🔴 哪只手:先看【哪只手的眼量得出东西有多远】(它自己那只眼的朝向和指尖都量过了),量得出的里面再挑离它近的。
                  --  H18 2026-09-22 实测:两只手离剪刀一样远,按像素挑到了左手,而左腕眼朝向还没量、又没有可盯的东西 ⇒
                  --  转不了眼、走不了路;右手的眼全量过了却没被选。这是量出来的事实(标定在不在),不是偏好。
                  declare
                     function Eye_Ready (Arm : Natural) return Boolean is
                        Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
                     begin
                        return Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length)
                          and then C.Geo (Natural (Hc)).Valid and then C.Geo (Natural (Hc)).Tip_Valid and then C.Geo (Natural (Hc)).F > 0.0;
                     end Eye_Ready;
                     Any_Ready : Boolean := False;
                  begin
                     for K in 0 .. Natural (C.Items.Length) - 1 loop
                        if Role_Wants (R, C.Items (K).Kind) and then C.Items (K).Located and then C.Items (K).Cam = C.Cam
                          and then (Own < 0 or else Integer (C.Items (K).Arm) = Own) and then Eye_Ready (C.Items (K).Arm)
                        then
                           Any_Ready := True;
                        end if;
                     end loop;
                     for K in 0 .. Natural (C.Items.Length) - 1 loop
                        declare
                           It : constant Item := C.Items (K);
                           Want : constant Boolean := Role_Wants (R, It.Kind);
                           D : constant Long_Float :=
                             (if Has_Near and then It.Located
                              then Sqrt ((It.Cu - Near_U) ** 2 + (It.Cv - Near_V) ** 2) else 0.0);
                        begin
                           --  🔴 清单跨相机之后必须加这一条:只认【这一台相机里】看见的那一块。
                           --  不加,grasper 可能绑到另一台相机里的那只爪上 —— 位置、响应表全对不上。
                           if Want and then It.Located and then It.Cam = C.Cam
                             and then (Own < 0 or else Integer (It.Arm) = Own)
                             and then (not Any_Ready or else Eye_Ready (It.Arm))
                             and then D < Bd
                           then
                              Bd := D; Best := Integer (K) + 1;
                           end if;
                        end;
                     end loop;
                     if Best > 0 and then Any_Ready and then R = Sinew.Rl_Grasper then
                        Put_Line ("[身] 🔎 grasper 挑第" & Codec.Img (C.Items (Natural (Best) - 1).Arm + 1)
                                  & " 只手:它自己那只眼的朝向和指尖都量过(量得出东西有多远),且离点名的东西最近");
                     end if;
                  end;
                  return Best;
               end Bind_Role;

               --  绑不上时,把身上量到的零件种类如实报出来,让脑知道该换成什么说法
               function Why_No_Role (Key : String) return String is
                  N_Grip, N_Piece, N_Finger : Natural := 0;
               begin
                  for K in 0 .. Natural (C.Items.Length) - 1 loop
                     if C.Items (K).Cam = C.Cam then   --  只数这一台相机里的,别把跨相机的重复计进来
                        case C.Items (K).Kind is
                           when Grip => N_Grip := N_Grip + 1;
                           when Piece => N_Piece := N_Piece + 1;
                           when Finger => N_Finger := N_Finger + 1;
                           when others => null;
                        end case;
                     end if;
                  end loop;
                  --  手指 / 爪心只数带手指的抓握通道(清单里没手指的通道根本不列,见 Has_Fingers)
                  if not Any_Fingers (C) and then (Key = "grasper" or else Key = "pusher") then
                     return "我没有手指:抓握通道推到头,哪台相机里都没有东西跟着动"
                       & (if Key = "pusher" then ";也没量到【推得动东西又合不拢】的零件" else "");
                  elsif Key = "pusher" then
                     return "我身上没量到【推得动东西又合不拢】的零件(合得拢的爪心"
                       & Codec.Img (N_Grip) & " 组不算);要用手,写 grasper";
                  elsif Key = "me" and then N_Finger + N_Grip = 0 then
                     --  没有手指的身体(无人机)"我"本该就是整个机身;这一版还没接上(PLAN V1b 无人机 (c)),照实说
                     return "me 是【整个我】:我身上没量出手指和爪心,本该就是它 —— 可这一版还不会按整个机身走(me 没接上)";
                  elsif Key = "me" then
                     return "me 是【整个我】,只有推一下整幅画面跟着变、身上又分不出零件的机体才有它;"
                       & "我身上量得出 " & Codec.Img (N_Finger) & " 瓣手指、" & Codec.Img (N_Grip)
                       & " 组爪心,所以要点名到零件:写 grasper";
                  end if;
                  return "这只眼睛里没有一块符合它";
               end Why_No_Role;

               --  🔴 名字怎么落到画面上(2026-09-21 改问法):脑说"它在这一框里",框里哪些像素是它由我自己量。
               --  以前是"画面切成带编号的块,让脑挑一个号"—— 没有深度时那一刀是按明暗切的,剪刀被切成四五个碎框、
               --  腕眼一帧 191–450 件,没有任何一个号【是】那把剪刀,脑挑哪个都不对。
               --  编号照旧从头到尾不进语言;量不出来 ⇒ 如实说,绝不瞎猜。
               function Bind_Name (W : String; Tried : out Unbounded_String) return Integer is
                  Cam : constant Natural := C.Cam;
                  Kw : constant Natural := F.Cams (Cam).W;
                  Kh : constant Natural := F.Cams (Cam).H;
                  Found, Got, Iso : Boolean := False;
                  X0, Y0, X1, Y1 : Natural := 0;
                  R : Picture.Region;
                  M0 : Bools;                --  脑指它那一帧它的像素(整幅掩膜)
                  E2 : Unbounded_String;

                  --  这件点过名的东西此刻在清单第几号(没有就是 0)
                  function Item_Of (Bx : Natural) return Natural is
                  begin
                     for K in 0 .. Natural (C.Items.Length) - 1 loop
                        if C.Items (K).Kind = Thing and then C.Items (K).Cam = Cam and then C.Items (K).Located
                          and then Boxed_Index (C, Cam, C.Items (K).Cu, C.Items (K).Cv) = Integer (Bx)
                        then
                           return K + 1;
                        end if;
                     end loop;
                     return 0;
                  end Item_Of;
               begin
                  Tried := Null_Unbounded_String;
                  --  ① 这个名字在这只眼里点过、这一帧也量到了 ⇒ 就是它,不必再问一遍
                  for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                     if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Seen and then To_String (C.Boxed (Bi).Name) = W
                       and then Item_Of (Bi) > 0
                     then
                        C.Name_Cam := Integer (Cam);
                        return Integer (Item_Of (Bi));
                     end if;
                  end loop;
                  --  ①b 脑这回写的名字里【含着】它以前起过的名字(H44 2026-09-23 实测:它写 "grip mintgreenscissors"、"reach cell mintgreenscissors",
                  --  前面挂个动词;H48 实测头顶眼里叫 "scissor"、腕眼里叫 "scissors")⇒ 按字面就是同一件东西。
                  --  同一件东西在每只眼里、在记忆里都得叫一个名(H47 实测:两眼名字不同,视线就对不上号,交点算不出来)
                  --  ⇒ 不管在哪只眼里起的,旧名全改成脑现在用的这个;然后这只眼里要是已经量到它就直接用,没有再去问。只按字面包含,不猜别的。
                  declare
                     function Longest_Word (S : String) return String is
                        Bs, Be : Natural := 0;
                        I : Natural := S'First;
                     begin
                        while I <= S'Last loop
                           declare
                              J : Natural := I;
                           begin
                              while J <= S'Last and then S (J) /= ' ' loop
                                 J := J + 1;
                              end loop;
                              if J - I > Be - Bs then
                                 Bs := I; Be := J;
                              end if;
                              I := J + 1;
                           end;
                        end loop;
                        return (if Be > Bs then S (Bs .. Be - 1) else "");
                     end Longest_Word;
                     Lw : constant String := Longest_Word (W);
                     Renamed : Boolean := False;
                  begin
                     for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                        declare
                           Old : constant String := To_String (C.Boxed (Bi).Name);
                        begin
                           if Old'Length > 0 and then Old /= W
                             and then (Ada.Strings.Fixed.Index (W, Old) > 0
                                       or else (Lw'Length > 0 and then Lw /= Old and then Ada.Strings.Fixed.Index (Old, Lw) > 0))
                           then
                              if not Renamed then
                                 Put_Line ("[身] 📦 你写的「" & W & "」和你起过的名字「" & Old & "」是同一件东西 ⇒ 以后都叫它「" & W & "」");
                              end if;
                              Renamed := True;
                              declare
                                 B2 : Boxed_Thing := C.Boxed (Bi);
                              begin
                                 B2.Name := To_Unbounded_String (W);
                                 C.Boxed.Replace_Element (Bi, B2);
                              end;
                              if To_String (C.Geo_Pw_Name) = Old then
                                 C.Geo_Pw_Name := To_Unbounded_String (W);
                              end if;
                              if To_String (C.Geo_Name) = Old then
                                 C.Geo_Name := To_Unbounded_String (W);
                              end if;
                              if To_String (C.Sil_Name) = Old then   --  它的顶面点也跟着改名(H53:换个叫法轮廓就不认了)
                                 C.Sil_Name := To_Unbounded_String (W);
                              end if;
                           end if;
                        end;
                     end loop;
                     if Renamed then
                        for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                           if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Seen and then To_String (C.Boxed (Bi).Name) = W
                             and then Item_Of (Bi) > 0
                           then
                              C.Name_Cam := Integer (Cam);
                              return Integer (Item_Of (Bi));
                           end if;
                        end loop;
                     end if;
                  end;
                  --  ② 问脑它在哪一框。给它【干净】的画面:我画上去的格子和编号框实测在伤它的视力
                  if not Brain.Locate (To_String (C.Eye_Host), C.Eye_Port, W, F.Cams (Cam).RGB, Kw, Kh,
                                       Found, X0, Y0, X1, Y1, E2)
                  then
                     Tried := To_Unbounded_String ("我问自己的眼睛时没问通(" & To_String (E2) & ")");
                     return -1;
                  end if;
                  if not Found then
                     --  🔴 这只眼里指不出它,但它在哪我上一段刚量过(视线交点 / 我自己挪过的几眼)⇒ 按记住的位置绑上,走路凭记住的位置走。
                     --  H30 2026-09-22 实测:手贴到剪刀 8 mm 时腕眼里它糊了、被切了,Qwen 指不出 ⇒ 名字绑不上、程序编不过、一步不走,
                     --  而它在哪我明明知道。这不是替脑认东西:名字是脑起的、位置是我量的,只是不再要脑在糊掉的图上再指一次。
                     if C.Geo_Pw_Valid and then To_String (C.Geo_Pw_Name) = W then
                        declare
                           Gc : constant Geom.Cam_Geo := Geo_Of (C, Cam);
                           A2 : constant Integer := Cam_Arm (C, Cam);
                           Pu : Long_Float := Long_Float (Kw / 2);   --  投不进这只眼时先记在画面中央(只是个占位,走路不用它)
                           Pv : Long_Float := Long_Float (Kh / 2);
                           Front : Boolean := False;
                           Bt : Boxed_Thing;
                           It : Item;
                           At_Bx : Integer := -1;
                        begin
                           if A2 < 0 and then Gc.Fixed then
                              Geom.Project_Fixed (Gc, C.Geo_Pw, Pu, Pv, Front);
                           elsif A2 >= 0 and then Gc.Valid and then Gc.F > 0.0 and then A2 < Integer (F.EE.Length) then
                              Geom.Project (Gc, F.EE (Natural (A2)), C.Geo_Pw, Pu, Pv, Front);
                           end if;
                           if not Front or else Pu < 0.0 or else Pv < 0.0 or else Pu >= Long_Float (Kw) or else Pv >= Long_Float (Kh) then
                              Pu := Long_Float (Kw / 2); Pv := Long_Float (Kh / 2);
                           end if;
                           Bt.Name := To_Unbounded_String (W); Bt.Cam := Cam;
                           Bt.Cu := Pu / Long_Float (Kw); Bt.Cv := Pv / Long_Float (Kh);
                           Bt.Seen := False; Bt.Blind := True;   --  这只眼里确实指不出它;走路按名字用记住的位置,不用这只眼的像素
                           for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                              if C.Boxed (Bi).Cam = Cam and then To_String (C.Boxed (Bi).Name) = W then
                                 At_Bx := Integer (Bi);
                              end if;
                           end loop;
                           if At_Bx >= 0 then
                              C.Boxed.Replace_Element (Natural (At_Bx), Bt);
                           else
                              C.Boxed.Append (Bt);
                           end if;
                           It.Kind := Thing_Remembered; It.Located := True; It.Cam := Cam;
                           It.Cu := Bt.Cu; It.Cv := Bt.Cv;
                           C.Items.Append (It);
                           Put_Line ("[身] 📦 " & W & ":这只眼里指不出它,可它在哪我上一段量过 (" & Mm (C.Geo_Pw (0)) & "," & Mm (C.Geo_Pw (1)) & ","
                                     & Mm (C.Geo_Pw (2)) & ") ⇒ 按记住的位置绑上(前提是它没动)");
                           C.Name_Cam := Integer (Cam);
                           return Integer (C.Items.Length);
                        end;
                     end if;
                     --  脑看着图说"这只眼里我指不出它" ⇒ 记下【这个名字在这只眼里】,选眼的时候跳过它(不记就来回弹)。
                     Mark_Blind (C, Cam, To_Unbounded_String (W));
                     Tried := To_Unbounded_String ("我在这只眼里指不出它在哪");
                     return -1;
                  end if;
                  --  ③ 框里哪一片是它,我自己量
                  Seg_In_Box (C, F, Cam, X0, Y0, X1, Y1, Got, Iso, R, M0);
                  Put_Line ("[身] 📦 " & W & ":脑给的框 [" & Codec.Img (X0) & " " & Codec.Img (Y0) & " " & Codec.Img (X1) & " " & Codec.Img (Y1)
                            & "](第" & Codec.Img (Cam) & " 台相机)⇒ "
                            & (if Got then "框里量到一整块 " & Codec.Img (R.Count) & " px · 形心 ("
                                 & Codec.Fmt (R.Cu * Long_Float (Kw), 1) & "," & Codec.Fmt (R.Cv * Long_Float (Kh), 1)
                                 & ") · 长宽比 " & Codec.Fmt (R.Elong, 1)
                                 & (if Iso then " · 是单独的一块" else " · 它顶到了框外那一圈(挨着别的东西或被画面切掉),形心和长轴在这只眼里不可信")
                               else "框里没有哪一片和周围分得开"));
                  if not Got then
                     Tried := To_Unbounded_String ("你指的那一片里,我量不出哪些像素和周围分得开");
                     return -1;
                  end if;
                  --  ④ 记下它:从这一帧起每帧在原地重量。先前同名同眼的那一条作废(脑重新指了一遍,以新的为准)
                  declare
                     Bt : Boxed_Thing;
                     At_Bx : Integer := -1;
                  begin
                     Bt.Name := To_Unbounded_String (W); Bt.Cam := Cam;
                     Bt.X0 := R.X0; Bt.Y0 := R.Y0; Bt.X1 := R.X1; Bt.Y1 := R.Y1;
                     Bt.Cu := R.Cu; Bt.Cv := R.Cv; Bt.Seen := True; Bt.Isolated := Iso;
                     Bt.Mask := M0; Bt.Count := R.Count;
                     On_Pixel (M0, Kw, Kh, R, Bt.Pu_On, Bt.Pv_On);
                     Blob_Levels (F.Cams (Cam).Gray, Kw, Kh, M0, R, Bt.Gray, Bt.Bg);   --  记下它多亮、周围多亮:以后每帧认它靠这个
                     for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                        if C.Boxed (Bi).Cam = Cam and then To_String (C.Boxed (Bi).Name) = W then
                           At_Bx := Integer (Bi);
                        end if;
                     end loop;
                     if At_Bx >= 0 then
                        C.Boxed.Replace_Element (Natural (At_Bx), Bt);
                     else
                        C.Boxed.Append (Bt);
                        At_Bx := Integer (C.Boxed.Length) - 1;
                     end if;
                     --  ④b 别的眼里已经起过名的那块和它是不是同一件:看视线交不交在一点(名字对不上也认得出)
                     Unify_By_Sight (C, F, Cam, W, R);
                     --  ⑤ 让它当场进槽、进清单:这一帧重切一次(这回带着它),再照常对号
                     C.Cut_Cam := -1;
                     World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Kw, Kh);
                     for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
                        declare
                           Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
                           It : Item;
                        begin
                           if Sl.Present and then Boxed_Index (C, Cam, Sl.R.Cu, Sl.R.Cv) = At_Bx then
                              It.Kind := Thing; It.Located := True; It.Slot := Si; It.Cam := Cam;
                              It.Cu := Sl.R.Cu; It.Cv := Sl.R.Cv; It.Count := Sl.R.Count;
                              It.X0 := Sl.R.X0; It.Y0 := Sl.R.Y0; It.X1 := Sl.R.X1; It.Y1 := Sl.R.Y1;
                              It.Au := Sl.R.Au; It.Av := Sl.R.Av; It.Elong := Sl.R.Elong;
                              It.Gray := Picture.Mean_Gray (F.Cams (Cam).Gray, Kw, Kh, Sl.R);
                              C.Items.Append (It);
                              --  (刚记进去的那一条 Blind = False:只在【就是这只被判过"没有它"的眼】里又认出来了才解除 —— JA 推演过,别改回去)
                              C.Name_Cam := Integer (Cam);
                              return Integer (C.Items.Length);
                           end if;
                        end;
                     end loop;
                  end;
                  Tried := To_Unbounded_String ("我在你指的那一片里量到了它,可重量一遍时它没再出来");
                  return -1;
               end Bind_Name;

               --  脑点了名的那件东西此刻在【这只眼】里的像素(没有 ⇒ False)
               procedure Named_Pixel (U, V : out Long_Float; Have : out Boolean) is
               begin
                  U := 0.0; V := 0.0; Have := False;
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     declare
                        Key : constant String := To_String (Binds (I2).Key);
                        N : constant Integer := Binds (I2).Item;
                     begin
                        if Key /= "me" and then Key /= "grasper" and then Key /= "pusher"
                          and then N >= 1 and then N <= Integer (C.Items.Length)
                          and then C.Items (Natural (N) - 1).Kind in Thing | Thing_Remembered
                          and then C.Items (Natural (N) - 1).Located and then C.Items (Natural (N) - 1).Cam = C.Cam
                        then
                           U := C.Items (Natural (N) - 1).Cu * Long_Float (F.Cams (C.Cam).W);
                           V := C.Items (Natural (N) - 1).Cv * Long_Float (F.Cams (C.Cam).H);
                           Have := True;
                           return;
                        end if;
                     end;
                  end loop;
               end Named_Pixel;

               --  先认外面的东西(名字),再绑角色 —— 角色要挑"离它最近的那一组",所以顺序不能反
               procedure Bind_All is
                  Nu, Nv : Long_Float := 0.0;
                  Has_Near : Boolean := False;
                  procedure One (N : Sinew.Noun) is
                     Key : constant String := Plan.Key_Of (N);
                     E : Plan.Bind_Entry;
                  begin
                     if N.K /= Sinew.Nk_Thing or else Key = "" then
                        return;
                     end if;
                     for I2 in 0 .. Natural (Binds.Length) - 1 loop
                        if To_String (Binds (I2).Key) = Key then
                           return;
                        end if;
                     end loop;
                     E.Key := To_Unbounded_String (Key);
                     E.Item := Bind_Name (Key, E.Tried);
                     Binds.Append (E);
                     --  "离它近"这个提示是拿画面坐标比的 ⇒ 必须同一台相机,别台的坐标没有可比性
                     if E.Item >= 1 and then E.Item <= Integer (C.Items.Length)
                       and then C.Items (Natural (E.Item) - 1).Located
                       and then C.Items (Natural (E.Item) - 1).Cam = C.Cam
                       and then not Has_Near
                     then
                        Nu := C.Items (Natural (E.Item) - 1).Cu;
                        Nv := C.Items (Natural (E.Item) - 1).Cv;
                        Has_Near := True;
                     end if;
                  end One;
               begin
                  for Ix in 0 .. Natural (P.Code.Length) - 1 loop
                     if P.Code (Ix).O = Sinew.Op_Interval then
                        for Ci in 0 .. Natural (P.Code (Ix).Cons.Length) - 1 loop
                           One (P.Code (Ix).Cons (Ci).Subj);
                           One (P.Code (Ix).Cons (Ci).Obj);
                        end loop;
                     end if;
                  end loop;
                  for R in Sinew.Role loop
                     if R /= Sinew.Rl_None then
                        declare
                           E : Plan.Bind_Entry;
                        begin
                           E.Key := To_Unbounded_String (Sinew.Role_Word (R));
                           E.Item := Bind_Role (R, Nu, Nv, Has_Near);
                           Binds.Append (E);
                        end;
                     end if;
                  end loop;
               end Bind_All;
            begin
               C.Eye_Want := First_Eye;
               if P.Ok then
                  Bind_All;
                  --  🔴 认名字这一步会把【刚在框里量出来的东西】当场加进清单(T1 2026-09-21 实测:
                  --  "item scissors ⇒ 第7 块" 绑上了,编译器却说"我认不出 item scissors" —— 它手里那份事实表
                  --  是绑定【之前】建的,里面没有第 7 块)。⇒ 绑完重建一次,编译器看到的和清单是同一份。
                  Facts := Build_Facts (C, F);
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     --  绑不上要说【为什么】:光一句"认不出"等于没说,脑没法据此改写程序
                     Put_Line ("[身] 🔎 " & To_String (Binds (I2).Key) & " ⇒ "
                               & (if Binds (I2).Item > 0 then "第" & Codec.Img (Natural (Binds (I2).Item)) & " 块"
                                  else "绑不上:" & Why_No_Role (To_String (Binds (I2).Key))));
                  end loop;
               end if;
               --  🔴🔴 先看一条更硬的:这一段能用的相机,必须是【看得见被点名那个东西】的相机。
               --  看不见目标的相机,把手看得再清楚也没用 —— GM 就死在这儿:球从手腕相机里消失了,
               --  而头顶相机里它一直在,身体却从没去那儿看过。
               --  脑点名的那一块自己带着"我在哪台相机里",直接跟过去。这一条不受"一段只换一次眼"限制:
               --  它不是偏好,是这一段能不能干活的前提。
               C.Tgt_Cam := -1;
               declare
                  Tgt_Cam : Integer := -1;
               begin
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     declare
                        Key : constant String := To_String (Binds (I2).Key);
                     begin
                        if Key /= "me" and then Key /= "grasper" and then Key /= "pusher"
                          and then Binds (I2).Item > 0
                          and then Binds (I2).Item <= Integer (C.Items.Length)
                        then
                           Tgt_Cam := Integer (C.Items (Natural (Binds (I2).Item) - 1).Cam);
                        end if;
                     end;
                  end loop;
                  if Tgt_Cam >= 0 and then Natural (Tgt_Cam) /= C.Cam then
                     Put_Line ("[身] 👁 你点名的那块在第" & Codec.Img (Natural (Tgt_Cam))
                               & " 只眼睛里,这只眼睛看不见它 ⇒ 换过去(看不见目标的眼睛干不了这一段)");
                     C.Cam := Natural (Tgt_Cam);
                     C.Recent := S ("the thing you named is in a different eye of mine and this one cannot see it, "
                                    & "so I moved to the eye that can. Nothing moved. Say the same thing again. "
                                    & Mode_Line (C, "moved to the eye that can see what you named"));
                     return;
                  end if;
                  C.Tgt_Cam := Tgt_Cam;   --  给下面的选眼用:脑点的眼睛不许把段挪离目标所在的那一台
               end;

               --  🔴 用哪只眼睛,身体自己选,脑不参与(语言里没有 look 这个词)。
               --  判据是量出来的:这条胳膊一动,哪只眼睛的画面变得最多 —— 变得最少的那只
               --  正好是"顺着我伸过去的方向看"的那只,真实偏差在它眼里是零(GC 实测:
               --  主相机说差 0.2 格,腕相机里一看,夹的是剪刀)。20 台位置毫无规律的相机也是这一招:
               --  20 个数取最大,位置朝向内参一个都不用知道。
               declare
                  Sub_Arm : Integer := -1;
                  Best_Cam : Natural := C.Cam;
                  Best_V : Long_Float := -1.0;
                  --  脑说过"这只眼里没有【这一段点了名的东西】"(按名字记:脑写个 it 绑不上,不等于剪刀在那只眼里看不见)
                  function Blind_Here (Cm : Natural) return Boolean is
                  begin
                     for I3 in 0 .. Natural (Binds.Length) - 1 loop
                        declare
                           Key : constant String := To_String (Binds (I3).Key);
                        begin
                           if Key /= "me" and then Key /= "grasper" and then Key /= "pusher"
                             and then Is_Blind (C, Integer (Cm), Binds (I3).Key)
                           then
                              return True;
                           end if;
                        end;
                     end loop;
                     return False;
                  end Blind_Here;
               begin
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     if To_String (Binds (I2).Key) = "grasper" and then Binds (I2).Item > 0
                       and then Binds (I2).Item <= Integer (C.Items.Length)
                     then
                        Sub_Arm := Integer (C.Items (Natural (Binds (I2).Item) - 1).Arm);
                     end if;
                  end loop;
                  if Sub_Arm >= 0 then
                     for Cm in 0 .. C.Map.N_Cams - 1 loop
                        declare
                           Ix : constant Natural := Natural (Sub_Arm) * C.Map.N_Cams + Cm;
                           Vv : constant Long_Float :=
                             (if Ix < Natural (C.Map.Cam_Frac.Length) then C.Map.Cam_Frac (Ix) else 0.0);
                        begin
                           if Vv > Best_V then
                              Best_V := Vv; Best_Cam := Cm;
                           end if;
                        end;
                     end loop;
                     --  🔴 脑点了名就照脑说的挑,而且不受"一集只换一次眼"限制:
                     --  那条限制防的是身体自己来回弹,不是防脑。判据仍然是量出来的 Cam_Frac:
                     --  still = 这条胳膊一动、画面变得【最少】的那只(不长在我身上 ⇒ 看得见我平移);
                     --  moving = 变得【最多】的那只。量不出来就照实说,不许瞎挑。
                     if Sinew."/=" (C.Eye_Want, Sinew.Ey_None) then
                        declare
                           use type Sinew.Eye_Pick;
                           Pick : Integer := -1;
                           Bv : Long_Float := (if C.Eye_Want = Sinew.Ey_Still then 1.0e9 else -1.0);
                           Any : Boolean := False;
                           --  🔴 "不动的眼"先挑【不长在任何一条胳膊上】的(开机量的 Cam_On_Arm)。H27 2026-09-22 实测:按"变得最少"挑,
                           --  挑中的是长在【另一条】胳膊上的左腕眼(这条胳膊一动它变 0.024 幅,比头顶眼的 0.039 还静),
                           --  而那只眼里没有剪刀、它自己一动画面又全变。一只眼都不长在胳膊上的才叫不动;没有这样的眼才退回"变得最少"。
                           Any_Free : Boolean := False;
                           Free_Exists : Boolean := False;   --  有不长在胳膊上的眼(不管它此刻看不看得见它)
                           Still_Dead : Boolean := False;    --  有这样的眼,可它们此刻都看不见它(脑指不出 / 我的手压着)⇒ 不换,说清楚
                        begin
                           for Cm in 0 .. C.Map.N_Cams - 1 loop
                              if Cam_Arm (C, Cm) < 0 and then Natural (Sub_Arm) * C.Map.N_Cams + Cm < Natural (C.Map.Cam_Frac.Length) then
                                 Free_Exists := True;
                                 if not Blind_Here (Cm) then
                                    Any_Free := True;
                                 end if;
                              end if;
                           end loop;
                           --  🔴 H37 2026-09-22 实测:头顶眼里它被我的手盖住了,"不动的眼"就退到了左腕眼,脑在那只眼里把 grasper 绑成了左手,
                           --  从此指挥的是另一条胳膊。不长在胳膊上的眼都看不见它时,没有可用的"不动的眼",就不换。
                           Still_Dead := C.Eye_Want = Sinew.Ey_Still and then Free_Exists and then not Any_Free;
                           for Cm in 0 .. C.Map.N_Cams - 1 loop
                              declare
                                 Ix : constant Natural := Natural (Sub_Arm) * C.Map.N_Cams + Cm;
                                 Vv : constant Long_Float :=
                                   --  "不动的眼"只认不长在任何一条胳膊上的(09-26 owner:一种量法;以前没有这样的眼时退回"变得最少",
                                   --  会把长在另一条胳膊上的眼当成不动的 —— H27 就栽在这上面)⇒ 没有就是没有
                                   (if Ix < Natural (C.Map.Cam_Frac.Length)
                                      and then not (C.Eye_Want = Sinew.Ey_Still and then Cam_Arm (C, Cm) >= 0)
                                    then C.Map.Cam_Frac (Ix) else -1.0);
                              begin
                                 --  🔴 "最静"只是一半 —— 另一半是【脑在那只眼里认得出这一段要做的事】。
                                 --  JA 2026-09-15 实测:我写 with my still eye,身体按 Cam_Frac 挑了第 1 只
                                 --  (这条胳膊一动它只变 0.024 幅,确实最静),而那只眼里是风扇和键盘,没有球
                                 --  ⇒ 编译期连拒三轮,一推没走。判据没错,是少了一半。
                                 --  ⚠️ 这不是 IH 撤回的那条("目标那只眼永远赢"—— 那条会把脑永远拽回 0 号眼,
                                 --  于是量远近永远被拒)。这里只跳过【脑自己刚说过"这儿没有"】的那只:
                                 --  是脑在决定,不是身体替它决定;脑看得见时照样答真编号,那只眼一次都不会被跳。
                                 if Vv >= 0.0 and then not Blind_Here (Cm) and then not Still_Dead then
                                    Any := True;
                                    if (C.Eye_Want = Sinew.Ey_Still and then Vv < Bv)
                                      or else (C.Eye_Want = Sinew.Ey_Moving and then Vv > Bv)
                                    then
                                       Bv := Vv; Pick := Cm;
                                    end if;
                                 end if;
                              end;
                           end loop;
                           --  🔴 "跟着我动的那只眼" = 【长在这条胳膊上的那只】,这是开机量过的事实(Cam_Arm),不是"变化第二大的那只"。
                           --  T9 2026-09-21 实测:Qwen 已经在左手自己的眼里,写了 with my moving eye;那只眼刚因为一个没认出的名字
                           --  被标成 Blind 而被跳过 ⇒ 挑了"这条胳膊一动只变 0.048 幅"的右腕眼(长在【另一条】胳膊上)⇒
                           --  那只眼里看不见这只手,角色表为空,白耗 8 轮。没认出某个名字,不改变"哪只眼长在我手上"。
                           if C.Eye_Want = Sinew.Ey_Moving then
                              for Cm in 0 .. C.Map.N_Cams - 1 loop
                                 if Cam_Arm (C, Cm) = Sub_Arm then
                                    Pick := Integer (Cm); Any := True;
                                    Bv := (if Natural (Sub_Arm) * C.Map.N_Cams + Cm < Natural (C.Map.Cam_Frac.Length)
                                           then C.Map.Cam_Frac (Natural (Sub_Arm) * C.Map.N_Cams + Cm) else 0.0);
                                 end if;
                              end loop;
                           end if;
                           --  🔴 "静"只是一半,另一半是【它得看得见目标】。
                           --  HA 实测:我写 with my still eye,它按判据挑了第 1 台(变 0.024 幅,确实最静)——
                           --  而第 1 台里是风扇和键盘,根本没有球。判据没错,是我少写了一半。
                           --  脑点了名的那一块自己带着相机号,那一台是唯一看得见它的;
                           --  两者冲突时【目标那一台赢】,并且照实说清楚为什么没听脑的。
                           --  🔴🔴 撤回"目标那一台赢"(IH 2026-09-15 实测)。
                           --  原意是对的:HA 那次身体【自己】挑了一只没有球的眼睛,于是规定"看得见的那台赢"。
                           --  但脑【明确点名】要换眼睛时,这条就变成了一道永远拉不开的闸:
                           --  球只在 0 号眼里被点过名 ⇒ 每次都被拽回 0 号眼 ⇒ 而 0 号眼不长在我身上
                           --  ⇒ 量远近永远被拒 ⇒ 前后那一栏永远是 0.0。IH 实测:横向已经对到 0.2 步,
                           --  而两段下来一个米数都没有,就卡在这儿。
                           --  改成:照脑说的换过去,并且【在新那只眼睛里重新点一次名】——
                           --  认不认得出由脑看着图说,不由我替它决定(脑答 0 = 这儿没有,我再回去)。
                           if C.Tgt_Cam >= 0 and then Pick >= 0 and then Pick /= C.Tgt_Cam then
                              Put_Line ("[身] 👁 你点名要的那只眼睛(第" & Codec.Img (Natural (Pick))
                                        & " 只)不是你上次点名那块所在的第" & Codec.Img (Natural (C.Tgt_Cam))
                                        & " 只 ⇒ 我照你说的换过去,到那边再问你一次它是哪一块");
                              C.Blind_Say := S ("the eye you asked for is not the one you last named that thing in. "
                                                & "I moved to the eye you asked for anyway, and I will ask you which "
                                                & "one it is in that picture. Answer 0 if it is not visible there and "
                                                & "I will go back to the eye that can see it.");
                           end if;
                           if Still_Dead then
                              Put_Line ("[身] 👁 你要不动的眼;不长在胳膊上的眼此刻都看不见它(指不出 / 被我的手盖着)⇒ 不换眼,留在这只");
                              C.Blind_Say := S ("you asked for my still eye, but right now none of my still eyes can see the thing "
                                                & "(it could not be pointed out there, or my hand is over it), so I stayed in the eye I am in");
                           elsif not Any or else Pick < 0 then
                              C.Blind_Say := S ("you asked me to judge this with one of my eyes picked by how much it "
                                                & "changes when I move, but I have not measured that for this part yet, "
                                                & "so I used the eye I am already in and said so");
                           elsif Natural (Pick) /= C.Cam then
                              Put_Line ("[身] 👁 你点名要"
                                        & (if C.Eye_Want = Sinew.Ey_Still then "不跟着我动" else "跟着我动")
                                        & "的那只眼睛 ⇒ 第" & Codec.Img (Natural (Pick)) & " 只(这条胳膊一动它变 "
                                        & Codec.Fmt (Bv, 3) & " 幅)⇒ 换过去");
                              declare
                                 Pu, Pv : Long_Float;
                                 Have : Boolean;
                                 Ev : Unbounded_String;
                                 Aok : Boolean := False;
                              begin
                                 Named_Pixel (Pu, Pv, Have);
                                 if Have and then C.Eye_Want = Sinew.Ey_Moving and then Sub_Arm >= 0
                                   and then C.Cam < Natural (C.Geo.Length) and then C.Geo (C.Cam).Fixed
                                 then
                                    Aim_Eye_At (L, C, F, Natural (Sub_Arm), C.Cam, Pu, Pv, 0.5, Ev, Aok);
                                    Put_Line ("[身] 👁 转眼:" & To_String (Ev));
                                    Clear_Blind (C, Natural (Pick));
                                 end if;
                                 C.Cam := Natural (Pick);
                                 C.Recent := S ("I moved to the eye you asked for. "
                                                & (if Have and then Aok then "I first turned it toward where my still eye says the thing is. "
                                                   elsif Have and then Length (Ev) > 0 then "I tried to turn it toward where my still eye says the thing is: " & To_String (Ev) & ". "
                                                   else "")
                                                & "Nothing else moved. Say the same thing again. " & Mode_Line (C, "moved to the eye you named"));
                              end;
                              return;
                           end if;
                        end;
                     end if;
                     --  🔴🔴 换过去的那只眼【必须看得见这只手】,否则换完角色就绑不上了:
                     --  SC2 实测,自动换眼把我挪到一只看不见爪子的眼里,那一轮键盘上
                     --  「角色」整个是空的 —— 连 grasper 都点不了名,脑写什么都被退回,
                     --  一步都走不了。理由本身(那只眼对这条胳膊的动作最敏感)没错,
                     --  但"最敏感"不等于"看得见我" —— 得两条都成立才值得换。
                     --  握区看不看得见是量出来的(每台相机各量一次),不是猜的。
                     --  🔴 脑没点眼、这一段又点了名要去够一件东西 ⇒ 用【量得出它有多远】的那只眼:长在这条胳膊上、几何常数齐的那只
                     --  (语言 §8.1:约束绑到量得出它的那只眼,写的人不选相机)。这一条不受"一集只换一次"限制 ——
                     --  那条防的是来回弹;这里不会弹:那只眼里指不出它时 Bind_Name 会把它记成 Blind_Cam,下面就不再去。
                     --  T9 实测:身体换过一次眼之后被拽回头顶眼,此后每一段都在头顶眼里一推一像素地磨(60 推 · 483 拍没到)。
                     declare
                        Hand_Eye : Integer := -1;
                        Names_A_Thing : Boolean := False;
                     begin
                        for Cm in 0 .. C.Map.N_Cams - 1 loop
                           if Cam_Arm (C, Cm) = Sub_Arm and then Cm < Natural (C.Geo.Length)
                             and then C.Geo (Cm).Tip_Valid and then C.Geo (Cm).F > 0.0
                           then
                              Hand_Eye := Integer (Cm);
                           end if;
                        end loop;
                        for I2 in 0 .. Natural (Binds.Length) - 1 loop
                           declare
                              Key : constant String := To_String (Binds (I2).Key);
                           begin
                              if Key /= "me" and then Key /= "grasper" and then Key /= "pusher" then
                                 Names_A_Thing := True;
                              end if;
                           end;
                        end loop;
                        --  🔴 脑说 with my still eye 也一样要换过去问一次名(H27 2026-09-22 实测:不换,走路就落进不动的眼、掉回老路)。
                        --  但只在【那只眼里还没点过它的名】时换:点过名的东西我每帧自己重量,走路时按名字在那只眼里跟,
                        --  脑照样看着它点的那只眼。这不是替脑选眼,是走路的手要在自己的眼里认一次它。
                        declare
                           Named_There : Boolean := False;
                           Pu0, Pv0 : Long_Float;
                           Have0 : Boolean := False;
                           Can_Aim : Boolean := False;   --  不动的眼看得见它、也量过自己在哪 ⇒ 能把这只手的眼转向它
                        begin
                           if Hand_Eye >= 0 then
                              for I2 in 0 .. Natural (Binds.Length) - 1 loop
                                 declare
                                    Key : constant String := To_String (Binds (I2).Key);
                                 begin
                                    if Key /= "me" and then Key /= "grasper" and then Key /= "pusher" and then Binds (I2).Item >= 1
                                      and then Boxed_By (C, Natural (Hand_Eye), Item_Name (C, Natural (Binds (I2).Item))) >= 0
                                      and then not Is_Blind (C, Hand_Eye, Item_Name (C, Natural (Binds (I2).Item)))
                                    then
                                       Named_There := True;
                                    end if;
                                 end;
                              end loop;
                              Named_Pixel (Pu0, Pv0, Have0);
                              Can_Aim := Have0 and then C.Cam < Natural (C.Geo.Length) and then C.Geo (C.Cam).Fixed;
                           end if;
                        --  🔴 脑说过"那只眼里没有它"只拦【不转眼就再问一遍】;能把那只眼转向它,以前说的"没有它"就不算数了
                        --  (H32 2026-09-22 实测:腕眼还没转过去时脑在里面指不出剪刀 ⇒ 记成没有 ⇒ 这一条从此不进 ⇒ 眼永远转不过去)。
                        if Hand_Eye >= 0 and then Names_A_Thing and then Natural (Hand_Eye) /= C.Cam
                          and then (Can_Aim or else not Blind_Here (Natural (Hand_Eye))) and then Sinew."/=" (C.Eye_Want, Sinew.Ey_Moving)
                          and then not Named_There
                        then
                           Put_Line ("[身] 👁 " & (if Sinew."=" (C.Eye_Want, Sinew.Ey_None) then "你没点眼;" else "你要用不动的眼判,可走路得用")
                                     & "这条胳膊自己的那只眼(第" & Codec.Img (Natural (Hand_Eye))
                                     & " 只)量得出东西有多远 ⇒ 换过去,在那儿再问你一次它在哪");
                           --  🔴 换过去之前先把那只眼【转向它】:现在这只是不动的眼,它量过自己在世界里的位置,
                           --  视线 ∩ 它躺着的面 = 它在哪。不转的话手眼常常根本看不见它(V3 2026-09-22:剪刀在两只手后方)。
                           declare
                              Pu, Pv : Long_Float;
                              Have : Boolean;
                              Ev : Unbounded_String;
                              Aok : Boolean := False;
                           begin
                              Named_Pixel (Pu, Pv, Have);
                              if Have and then C.Cam < Natural (C.Geo.Length) and then C.Geo (C.Cam).Fixed then
                                 Aim_Eye_At (L, C, F, Natural (Sub_Arm), C.Cam, Pu, Pv, 0.5, Ev, Aok);
                                 Put_Line ("[身] 👁 转眼:" & To_String (Ev));
                                 Clear_Blind (C, Natural (Hand_Eye));   --  那只眼现在看的是别处了,以前说的"没有它"不再算数
                              end if;
                              C.Cam := Natural (Hand_Eye);
                              C.Eye_Chosen := True;
                              --  🔴 这句话要和"没点眼"那句一个样子(H29 2026-09-22 实测:我多写了一句"把它在这张图里指出来",
                              --  Qwen 从此不再写 do,改写 remember/run/请告诉我怎么动手 —— 脑对话里多一句要求,它就以为该做别的事)。
                              C.Recent := S ((if Sinew."=" (C.Eye_Want, Sinew.Ey_None)
                                              then "you did not name an eye, so I moved to the eye that rides on the arm you are moving: "
                                              else "to walk up to a thing I use the eye that rides on the arm I am moving, so I moved to it: ")
                                             & "it is the one through which I can measure how far away a thing is. "
                                             & (if Have and then Aok then "I first turned that eye toward where my still eye says the thing is. "
                                                elsif Have then "I tried to turn that eye toward where my still eye says the thing is: " & To_String (Ev) & ". "
                                                else "")
                                             & "Nothing else moved. Say the same thing again. " & Mode_Line (C, "moved to the eye that can measure distance"));
                           end;
                           return;
                        end if;
                        end;
                     end;
                     --  🔴 撤掉"我自己换到变化最大的那只眼"(H32 2026-09-22 实测:脑第一轮只写了几句 say、没点名,这条就把脑换到腕眼,
                     --  而腕眼没转过去、里面根本没有剪刀 ⇒ 脑在那只眼里指不出它 ⇒ 记成"没有它" ⇒ 后面每一段都以此为由不走)。
                     --  它是逐像素推那条老路的需要(哪只眼看得见我差多少);走路的眼现在由要动的胳膊定(Hand_Eye_Of),
                     --  脑看哪只眼只为了点名,换眼只在"要把这只手的眼转向它、再问一次名"时发生(上面那一条)。
                     pragma Unreferenced (Best_Cam, Best_V);
                  end if;
               end;
               --  🔴 兑现身体自己印过的那句承诺:"Answer 0 if it is not visible there and I will go
               --  back to the eye that can see it"。JA 2026-09-15 实测:我答了 0,它【没回去】,
               --  原地又拒了三轮 —— 身体说了一句它不做的话,这是本仓最坏的一类。
               --  脑说"这只眼里没有它"⇒ 回到脑上一次真认出名字的那只眼,让它再说一遍同样的话。
               declare
                  Miss : Boolean := False;
               begin
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     declare
                        Key : constant String := To_String (Binds (I2).Key);
                     begin
                        if Key /= "me" and then Key /= "grasper" and then Key /= "pusher"
                          and then Binds (I2).Item <= 0
                        then
                           Miss := True;
                        end if;
                     end;
                  end loop;
                  if Miss and then C.Name_Cam >= 0 and then C.Name_Cam /= Integer (C.Cam) then
                     Put_Line ("[身] 👁 你说这只眼(第" & Codec.Img (C.Cam)
                               & " 只)里没有它 ⇒ 我回到你上次真认出它的第"
                               & Codec.Img (Natural (C.Name_Cam)) & " 只眼(这是我答应过的)");
                     C.Cam := Natural (C.Name_Cam);
                     C.Recent := S ("you told me that thing is not visible in the eye I had moved to, so I went "
                                    & "back to the eye you last recognised it in, as I said I would. Nothing "
                                    & "moved. Say the same thing again. "
                                    & Mode_Line (C, "went back to the eye that can see what you named"));
                     return;
                  end if;
               end;
               V := Plan.Check (P, Rep, Facts, Binds);
               if V.Ok then
                  --  第三道闸:整段在自己量出来的表上跑一遍,不通电
                  V := Plan.Dry_Run (P, Rep, Facts, Binds);
                  if V.Ok then
                     Put_Line ("[身] ⚖ 编译过了,空转也走得通");
                  else
                     Put_Line ("[身] ⚖ 空转就走不通:" & Plan.Say (V));
                  end if;
               else
                  Put_Line ("[身] ⚖ " & Plan.Say (V));
               end if;
               if not V.Ok then
                  C.Refused := S ("line " & Codec.Img (V.Err_Line) & ": " & To_String (V.Err)
                                  & (if Length (V.Instead) > 0 then "  -> " & To_String (V.Instead) else ""));
                  C.Recent := S ("I refused your program before anything moved. " & To_String (C.Refused)
                                 & " Nothing has moved. " & Mode_Line (C, "refused"));
                  return;
               end if;
               C.Refused := Null_Unbounded_String;
               C.Prog := P;
               C.Binds := Binds;
               C.M := (others => <>);
               C.Have_Prog := True;
               C.Prog_Log := Null_Unbounded_String;
            end;
         end;
      end if;
      --  往前走到下一条要真动身体的指令。说人话、记地方这些在这儿就地办掉。
      declare
         What : Runtime.Yield;
         Ins : Sinew.Instr;
         Guard : Natural := 0;
      begin
         loop
            Guard := Guard + 1;
            exit when Guard > 64;
            Runtime.Advance (C.Prog, C.M, What, Ins);
            case What is
               when Runtime.Y_Say =>
                  Put_Line ("[身] 🧠 它说:" & To_String (Ins.Text));
                  --  🔴🔴 每一轮的提示词末尾都印着「CAMERAS (say look = k to see through that camera
                  --  next turn)」,而这件事【从来没有实现过】:`Brain.Say.Look` 这个字段全项目没有
                  --  任何地方给它赋过值,恒为 0,于是 `if Say.Look >= 2` 是一段死代码。
                  --  HB6 实测:我说 `say look = 3`,下一轮日志仍是「第 2 轮(第 0 台相机)」。
                  --  ⇒ 这是"我答应给你的键,其实按不动"—— 和键盘上关系词恒空是同一类错。
                  --  ⇒ 就在 say 这一句里认这个词:第 k 个编号按提示词的排法(1 = 此刻这只眼,
                  --    2.. = 其余各只,顺序和印给脑的那一行完全一致,所以脑看到几就是几)。
                  declare
                     T : constant String := To_String (Ins.Text);
                     I : Natural := T'First;
                     K : Natural := 0;
                     Got : Boolean := False;
                  begin
                     while I + 3 <= T'Last loop
                        if T (I .. I + 3) = "look" then
                           declare
                              J : Natural := I + 4;
                           begin
                              while J <= T'Last and then (T (J) = ' ' or else T (J) = '=') loop
                                 J := J + 1;
                              end loop;
                              while J <= T'Last and then T (J) in '0' .. '9' loop
                                 K := K * 10 + (Character'Pos (T (J)) - Character'Pos ('0'));
                                 J := J + 1;
                                 Got := True;
                              end loop;
                           end;
                           exit;
                        end if;
                        I := I + 1;
                     end loop;
                     --  🔴 k = 1 是"就用我现在这只眼"。以前这一支什么都不做,于是脑【说不出
                     --  "别换,就这只"】—— 而那条"哪只眼变化最大"的启发式会在下一段把它抢走。
                     --  说了"就用这只"也是点过名,一样要挡住启发式。
                     if Got and then K = 1 then
                        Put_Line ("[身]    它说就用现在这只眼(第" & Natural'Image (C.Cam)
                                  & " 台)⇒ 这一段不再自己换");
                        C.Eye_Chosen := True;
                     end if;
                     if Got and then K >= 2 then
                        declare
                           N : Natural := 2;
                        begin
                           for Ci in 0 .. C.Map.N_Cams - 1 loop
                              if Ci /= C.Cam then
                                 if N = K then
                                    Put_Line ("[身]    它要换到第" & Natural'Image (Ci)
                                              & " 台相机 ⇒ 下一轮在那台里列块、问、执行");
                                    C.Cam := Ci;
                                    Clear_Blind (C, Ci);
                                    --  🔴 脑明确点了眼 ⇒ 这一段不许再被"哪只眼变化最大"那条启发式抢走。
                                    --  HB9 实测:那条启发式在【远距离伸手】时选反 —— 它挑腕眼,而贴近的目标里
                                    --  有一项是"看着多大要长到爪口那么大",腕眼里这要求手凑到极近,
                                    --  胳膊够不到就顶死:大小那一项从第 3 步到第 38 步一直是 40,
                                    --  左右/上下反被带偏,差距 0.869 → 1.136(越走越远)。
                                    --  头顶眼没有这个毛病(离得远),HB8 就是在头顶眼里走到 touched 的。
                                    --  ⇒ 人说的话盖过身体的偏好;身体的偏好只在脑没说话时才作数。
                                    C.Eye_Chosen := True;
                                    exit;
                                 end if;
                                 N := N + 1;
                              end if;
                           end loop;
                        end;
                     end if;
                  end;
               when Runtime.Y_Remember =>
                  --  记的是"这一刻它在我这只眼睛里的位置和远近" —— 身体自己找得回来的东西,不是坐标
                  declare
                     A : constant Integer := Plan.Look_Up (C.Binds, Ins.Subj);
                     Pl : Place;
                     Found : Boolean := False;
                  begin
                     if A > 0 and then A <= Integer (C.Items.Length)
                       and then C.Items (Natural (A) - 1).Located
                     then
                        Pl.Name := Ins.Name;
                        Pl.Cam := C.Items (Natural (A) - 1).Cam;   --  这个地方是在哪台相机里记下的
                        Pl.Cu := C.Items (Natural (A) - 1).Cu;
                        Pl.Cv := C.Items (Natural (A) - 1).Cv;
                        Pl.Z := C.Items (Natural (A) - 1).Depth;
                        for K in 0 .. Natural (C.Places.Length) - 1 loop
                           if C.Places (K).Name = Pl.Name then
                              C.Places.Replace_Element (K, Pl);
                              Found := True;
                           end if;
                        end loop;
                        if not Found then
                           C.Places.Append (Pl);
                        end if;
                        Put_Line ("[身] 📍 记住了「" & To_String (Pl.Name) & "」= 此刻 ("
                                  & Codec.Fmt (Pl.Cu, 3) & "," & Codec.Fmt (Pl.Cv, 3) & ") 深 "
                                  & Codec.Fmt (Pl.Z, 3));
                     else
                        --  🔴 记不住不是停下的理由(HZ 2026-09-15 实测:脑写了三行,
                        --  第二行是【记个名字】,没记成就把第三行那句"去球上方"整条扔了 ⇒ 一推没走)。
                        --  能停我的只有人的命令和脑写的 until。记不住就照说出来,后面的行照跑。
                        Append (C.Prog_Log,
                                (if Length (C.Prog_Log) > 0 then ASCII.LF & "" else "")
                                & "I could not see what you told me to remember, so I remembered nothing - "
                                & "I did not stop, and I ran the rest of your program anyway.");
                        Put_Line ("[身] 📍 记不住「" & To_String (Ins.Name)
                                  & "」—— 这一刻我看不见它;我不停,后面的行照跑");
                     end if;
                  end;
               when Runtime.Y_Done =>
                  Say.Done := True;
                  exit;
               when Runtime.Y_Finished =>
                  C.Have_Prog := False;
                  C.Recent := S (To_String (C.Prog_Log) & " That was the whole program. " & Mode_Line (C, "program finished"));
                  return;
               when Runtime.Y_Broken =>
                  C.Have_Prog := False;
                  C.Refused := S (Runtime.Broken_Why (C.M));
                  C.Recent := S (To_String (C.Prog_Log) & " " & Runtime.Broken_Why (C.M) & " " & Mode_Line (C, "program broke"));
                  return;
               when Runtime.Y_Interval =>
                  Fill_Say (C, Ins, Say);
                  Put_Line ("[身] ▶ " & Sinew.Unparse (Ins));
                  exit;
            end case;
         end loop;
      end;
      end;
   end;
   Put_Line ("[身] 🧠 它说:" & To_String (Say.Text) & " ‖ 看见=" & To_String (Say.See) & " · 动" & Natural'Image (Natural (Say.Moves.Length)) &
             " 条 · 抓握=" & To_String (Say.Grip) & (if Say.Grip_Arm > 0 then "(第" & Natural'Image (Say.Grip_Arm) & " 只手" & (if Say.Grip_On > 0 then ",在第" & Natural'Image (Say.Grip_On) & " 号上" else "") & ")" else "") &
             " · 到 " & To_String (Say.Until_Kind) & (if Say.Until_Kind = "steps" then Natural'Image (Say.Steps) & " 步" else "") & " 为止" &
             (if Say.Fast then " · 快" else "") & (if Say.Done then " · 它说已经做完了" else ""));
   --  它点名的世界块 ⇒ 记槽号
   for G of Say.Moves loop
      declare
         Ns : constant array (1 .. 2) of Natural := [G.Of_Item, G.Item];
      begin
      for N of Ns loop
         if N >= 1 and then N <= Natural (C.Items.Length) and then C.Items (N - 1).Kind in Thing | Thing_Remembered | Thing_Held then
            declare
               --  记到【那一块自己所在的】相机上:槽号跨相机不通用,记错台等于记了个别的东西
               Kc : constant Natural := C.Items (N - 1).Cam;
               Cs : World.Cam_State := C.Wld.Cams (Kc);
            begin
               Cs.Named := C.Items (N - 1).Slot;
               C.Wld.Cams.Replace_Element (Kc, Cs);
            end;
         end if;
      end loop;
      end;
   end loop;
   if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered then
      declare
         Cs : World.Cam_State := C.Wld.Cams (Cam);
      begin
         Cs.Named := C.Items (Say.Grip_On - 1).Slot;
         C.Wld.Cams.Replace_Element (Cam, Cs);
      end;
   end if;
   C.Fast := Say.Fast;
   if Say.Look >= 2 then
      declare
         K : Natural := 2;
      begin
         for Ci in 0 .. C.Map.N_Cams - 1 loop
            if Ci /= Cam then
               if K = Say.Look then
                  C.Cam := Ci;
                  Put_Line ("[身]    它要换到第" & Natural'Image (Ci) & " 台相机 ⇒ 下一轮在那台里列块、问、执行");
               end if;
               K := K + 1;
            end if;
         end loop;
      end;
   end if;
   if Say.Done then
      C.Recent := S ("you said it is already done. the body did nothing and is looking again. " & Mode_Line (C, "you said done"));
      return;
   end if;
   if C.Look_Only then
      C.Recent := S ("(look only) I did not move. " & Mode_Line (C, "look only"));
      return;
   end if;
   --  ── 执行 ──
   declare
      Until_K : constant Monitor.Until_Kind :=
        Kind_Of_Word (To_String (Say.Until_Kind));
      --  🔴 脑写的 "or N steps" 是【所有】until 的步数上限,不是只有 until steps 才读。
      --  以前只在 Until_Kind = "steps" 时才取 ⇒ 写 until arrived 时上限成了 0,而 arrived 又落进
      --  Until_K 的 else 分支变成 U_Steps,Fired 判 W.Steps >= 0 立刻成立 ⇒ 一段只走一步。
      --  (GM 实测:三段 until arrived 各只走 1 推 5 拍,身体却报"步子走完还没到")
      Step_Limit : constant Natural := Say.Steps;
      Avoid : Item_Vectors.Vector;
      Pts : Point_Vectors.Vector;
      Amount : Long_Float := 1.0;   --  没说 amount 时用满(比例);说了按它的
      Event : Unbounded_String;
      Steps_Taken : Natural := 0;
      Beats : Natural := 0;
      Blocked : Boolean;
      Desc : Unbounded_String;
      Grip_Arm : constant Integer := (if Say.Grip_Arm >= 1 and then Say.Grip_Arm <= C.Map.Arms then Integer (Say.Grip_Arm) - 1 else -1);
      Did_Grip : Unbounded_String;
      --  🔴 怎么走过去:拿【手自己报的位置】当尺子(几何走法)。GB5/GC2/GC4 抓起球的三炮走的就是它 ——
      --  GB5 日志原文:横挪 25.6 mm、它在画面里从 u=288.2 跳到 259.6 ⇒ "它在相机前 336.3 mm";
      --  下一步一口气挪 170 mm(实到差 2.6 mm);4 步从差 284 mm 走到差 8 mm。
      --  而 main 一直走的是"推一点、看画面变多少"那条:T3 2026-09-21 实测 60 推 · 704 拍,差距 0.434 → 0.433。
      --  09-20 把几何驾驶的过程体搬回了 main,但【开机装常数 / 内参进帧 / 这里的分派】三处接线一处没接,整条是死代码。
      Geo_Case : Natural := 0;            --  0 = 不是几何走法管的事;1 = 走到它跟前;2 = 拿着往回退(抬);3 = 合(合之前不再走)
      Geo_Slot_Now : Integer := -1;
      Geo_Above : Boolean := False;
      Geo_Desc : Unbounded_String;
      Own : Integer := -1;                --  这一段要动的胳膊
      Geo_Cam : Integer := -1;            --  它自己的眼(几何常数齐);-1 = 这条胳膊上没有这样的眼
      Geo_Name : Unbounded_String;        --  要去的那件东西叫什么(脑点的名;走路的眼里按名字跟)
      Grasp_Set : Contact.Set;            --  这一段合上之前算出来的接触集(合上拿住了就成为 Held_Set)
      Grasp_Valid : Boolean := False;
      Grasp_Mu_Nom, Grasp_Mu_Worst : Long_Float := 0.0;   --  这一组按量到的法向 / 法向取最坏要的摩擦(合完拿没拿住,按它记这件东西的摩擦)
   --  2a 把脑说的话变成要求:别动的,目标就是它现在的位置;要动的,目标是格子或与某号的关系;
   --  抓某号,目标是"和我张开的那片地方重合"(位置 / 远近 / 看着多大 / 朝向)
   --  把去哪翻成目标:格子 / 与某号的关系(碰到它 · 上下左右 · 前后 · 离远点)。全是量出来的位置,没有写死的距离
   procedure Set_Target (G : Brain.Goal; P : in out Point; Ok_Pt : in out Boolean) is
   begin
                        --  目标
                        if G.Cell >= 1 and then G.Cell <= Natural (C.Cells_U.Length) then
                           P.Tu := C.Cells_U (G.Cell - 1); P.Tv := C.Cells_V (G.Cell - 1); P.Tz := P.Z; P.Wz := 0.0;
                           P.Desc := S (Say_Item (C, G.Item) & " to cell " & Codec.Img (G.Cell));
                        elsif G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length) then
                           declare
                              O : constant Item := C.Items (G.Of_Item - 1);
                              Ow : constant Long_Float := Long_Float (O.X1 - O.X0) / Long_Float (Cw);
                              Oh : constant Long_Float := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                              Rl : constant String := To_String (G.Rel);
                           begin
                              if G.Has_Place then
                                 --  去一个【记住的地方】:目标就是那一刻记下的位置和远近
                                 P.Desc := S (Say_Item (C, G.Item) & " back to the place you had me remember");
                                 P.Tu := G.Pu; P.Tv := G.Pv;
                                 P.Tz := G.Pz;
                                 P.Wz := (if G.Pz > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                              elsif not O.Located then
                                 --  🔴 看不见它也不许停:用它上次被看见的地方当目标,照走,如实说。
                                 Report := Report & "I cannot see " & Say_Item (C, G.Of_Item)
                                           & " right now, so I aimed at where it was last seen. ";
                              else
                                 P.Desc := S (Say_Item (C, G.Item) & " " & Rl & " " & Say_Item (C, G.Of_Item));
                                 P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := P.Z; P.Wz := 0.0; P.Tuv_Z := O.Depth;
                                 if Rl = "at" then
                                    if P.To_Grip then
                                       --  它要来的地方 = 我两指之间:区心、区深、【到了跟前该有多大】
                                       declare
                                          Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, 0);
                                       begin
                                          --  🔴 目标取【合拢时扫过的那几瓣的共同中心】,不是整片扫过区的中心:
                                          --  手腕相机里手指离镜头很近,扫过的那一片几乎半个屏幕,它的中心没有意义。
                                          --  瓣是量出来的 —— 一瓣 = 吸盘,两瓣 = 两指,七瓣 = 七指,同一段代码,
                                          --  一个字没提手指几根。
                                          declare
                                             Lu : Long_Float := 0.0;
                                             Lv : Long_Float := 0.0;
                                             Ln : Long_Float := 0.0;
                                          begin
                                             if Z.A.Valid then
                                                Lu := Lu + Z.A.Cu; Lv := Lv + Z.A.Cv; Ln := Ln + 1.0;
                                             end if;
                                             if Z.B.Valid then
                                                Lu := Lu + Z.B.Cu; Lv := Lv + Z.B.Cv; Ln := Ln + 1.0;
                                             end if;
                                             P.Tu := (if Ln > 0.0 then Lu / Ln else Z.Cu);
                                             P.Tv := (if Ln > 0.0 then Lv / Ln else Z.Cv);
                                          end;
                                          P.Tz := Z.Depth;
                                          P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                                          --  🔴 "到了跟前该有多大"不能用瓣心距:在手【自己】的眼睛里,
                                          --  两根指头分别贴在画面最左最右,瓣心距 ≈ 整幅画(实测 1.001),
                                          --  拿它当目标等于要求球把整幅画填满(身体报"只有该有的 10.0% 那么大"),
                                          --  这一项就把解算整个带跑偏。
                                          --  正确的量:看着多大与距离成反比 —— 现在在 d、该到 d*,就该大 d/d* 倍。
                                          if not Picture.Is_Nan (Z.Depth) and then Z.Depth > 0.0
                                            and then P.Z > 0.0 and then P.Size > 0.0
                                          then
                                             P.Tsize := P.Size * (P.Z / Z.Depth);
                                             P.Wsize := 1.0;
                                          elsif Z.Span > 0.0 then
                                             --  🔴🔴 手【自己】那只眼睛里握区的远近常常读不到(NaN),
                                             --  而"看着多大"正是那只眼睛里【唯一】的前后信号 ——
                                             --  按上面那条算不出来就整个关掉,等于把前后这一维扔了
                                             --  (IU 2026-09-15 实测:大小 0.0、远近 0.0,九步差距纹丝不动)。
                                             --  退路不需要深度,而且是本仓 2026-09-08 定版的那条判据本身:
                                             --  **它在画面里长到和我张开的那片爪心一样宽,就是到了**。
                                             --  两个宽度都是身体自己量的,零系数、不碰深度图。
                                             P.Tsize := Z.Span;
                                             P.Wsize := 1.0;
                                          else
                                             --  🔴🔴 不许静悄悄地把前后这一维关掉:说清楚【为什么】关。
                                             --  一句"这一栏是 0"什么都不说明,"我为什么把它关了"才指得到地方
                                             --  (IV 2026-09-15:我补了退路,可 大小 还是 0.0,而日志一个字都没解释)。
                                             Put_Line ("[身]   📏 看着多大这一栏我关了 —— 握区的远近 "
                                                       & (if Picture.Is_Nan (Z.Depth) then "读不到(NaN)"
                                                          else Codec.Fmt (Z.Depth, 3))
                                                       & " · 握区张开 " & Codec.Fmt (Z.Span, 4)
                                                       & " 画幅 · 它现在看着 " & Codec.Fmt (P.Size, 4)
                                                       & " · 我自己的远近 " & Codec.Fmt (P.Z, 3)
                                                       & " ⇒ 这几样凑不出【它该多大】");
                                             C.Blind_Say := S ("I switched off the one thing that tells me how far away it is "
                                                              & "in this eye - how big it looks - because I could not work out "
                                                              & "how big it OUGHT to look: my grip's distance is unreadable here "
                                                              & "and I have no width measured for it either. Without that I am "
                                                              & "lining up the picture and nothing else.");
                                             P.Wsize := 0.0;   --  量不出该有多大就别用它,不许瞎给一个
                                          end if;
                                       end;
                                    elsif P.Kind = Thing_Pt and then O.Kind in Finger | Grip then
                                       --  X 装进握区:区心、区深、【到了跟前该有多大】
                                       --  🔴 最后这一项不能少:在手上这只眼睛里,握区的远近常常读不到(NaN),
                                       --  那时"看着多大"是【唯一】的距离信号。两个都没有的话,
                                       --  像素一对齐就会被判成"到了",而实际差着 20 厘米(GE 实测)。
                                       declare
                                          Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
                                       begin
                                          P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0); P.Tuv_Z := Z.Depth;
                                          P.Tsize := Long_Float'Max (1.0e-6, Z.Span);
                                          P.Wsize := (if Z.Span > 0.0 then 1.0 else 0.0);
                                       end;
                                    elsif P.Kind = Piece_Pt and then O.Kind in Thing | Thing_Remembered then
                                       --  "到它那儿" = 到它那一面,不替它挑高低(owner 2026-09-08:"半腰"假设了两指从侧面夹一个立在台面上的东西,
                                       --  吸盘、拆弹的线上不成立)。要更低就由脑说"下",词表里有
                                       P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 then 1.0 else 0.0);
                                    else
                                       P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                    end if;
                                 elsif Rl = "above" then
                                    P.Tv := O.Cv - Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                 elsif Rl = "below" then
                                    P.Tv := O.Cv + Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                 elsif Rl = "left" then
                                    P.Tu := O.Cu - Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                 elsif Rl = "right" then
                                    P.Tu := O.Cu + Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                 elsif Rl = "front" or else Rl = "back" then
                                    declare
                                       --  "一截" = 它自己在画面里的宽 × 它的深度(米,全是量的),没量到深度就用它鼓起的高度
                                       Sz : constant Long_Float := Long_Float'Max (O.Height, Long_Float'Max (P.Height, Ow * O.Depth));
                                    begin
                                       P.Tz := (if Rl = "front" then O.Depth - Sz else O.Depth + Sz);
                                       P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                    end;
                                 elsif Rl = "away" then
                                    declare
                                       Du : constant Long_Float := P.Cu - O.Cu;
                                       Dv : constant Long_Float := P.Cv - O.Cv;
                                       Ln : constant Long_Float := Long_Float'Max (1.0e-9, Sqrt (Du * Du + Dv * Dv));
                                       St : constant Long_Float := Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                    begin
                                       P.Tu := P.Cu + Du / Ln * St; P.Tv := P.Cv + Dv / Ln * St; P.Tuv_Z := P.Z;
                                    end;
                                 elsif Rl = "onto" or else Rl = "off" or else Rl = "press" or else Rl = "into" then
                                    --  它站的那个面在哪:它自己的深度 + 它鼓出多少(两个都是量出来的)。
                                    --  onto = 压到那个面那么深;off = 反过来离开那个面它自己那么高一截。
                                    --  press = 朝那个面【压过去一个到不了的深度】:走不到的那一截就是力。
                                    --  这具身体的观测里没有力那一路,所以"劲"只能是命令与实到之差 —— 那是任何身体都有的。
                                    declare
                                       Floor_Z : constant Long_Float := O.Depth + O.Height;
                                    begin
                                       if O.Depth > 0.0 and then P.Z > 0.0 and then O.Height > 0.0 then
                                          P.Tu := P.Cu; P.Tv := P.Cv; P.Tuv_Z := P.Z;
                                          --  into = 皮(这块的中位深度)和它站着的那个面(Floor_Z),正中间。
                                          --  写成两个量出来的深度取中点,不是"高度 × 一个我拍的数"。
                                          P.Tz := (if Rl = "off" then O.Depth - O.Height
                                                   elsif Rl = "onto" then Floor_Z
                                                   elsif Rl = "into" then Into_Depth (O.Depth, Floor_Z)
                                                   else Floor_Z + O.Height * Amount);
                                          P.Wz := 1.0;
                                       else
                                          --  🔴 量不出它鼓出多少也不许停:就朝它本身的远近走,如实说。
                                          P.Tu := P.Cu; P.Tv := P.Cv; P.Tz := O.Depth; P.Tuv_Z := P.Z;
                                          P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                          Report := Report & "I cannot measure how far " & Say_Item (C, G.Of_Item)
                                                    & " stands out of what it rests on, so I just went to its own distance. ";
                                       end if;
                                    end;
                                 elsif Rl = "face" then
                                    --  转到"从我这一点指向它那一点"的方向 = 我这一块的主轴方向。人不动,只转。
                                    --  存两倍角(和"朝哪"那一行同一个约定:主轴的正负两种写法算同一个)。
                                    declare
                                       Du : constant Long_Float := O.Cu - P.Cu;
                                       Dv : constant Long_Float := O.Cv - P.Cv;
                                    begin
                                       if abs Du > 0.0 or else abs Dv > 0.0 then
                                          P.Tu := P.Cu; P.Tv := P.Cv; P.Wz := 0.0; P.Tuv_Z := P.Z;
                                          P.Tang := Wrap (2.0 * Arctan (Dv, Du));
                                          P.Wang := 1.0;
                                       else
                                          --  🔴 没方向可转就不转,别的照走。
                                          Report := Report & Say_Item (C, G.Item) & " and item "
                                                    & Codec.Img (G.Of_Item) & " sit at the same spot, so I did not turn. ";
                                       end if;
                                    end;
                                 else
                                    --  🔴 认不得的关系【不许静悄悄地什么都不做】。词表长出一个新词而执行器还没实现它,
                                    --  静默 no-op 会让脑以为它说的话被执行了 —— 这正是整套设计要杀掉的那一类失败。
                                    --  🔴 运行期不许拦(认不得的词是编译器的活)。当作"贴上它"走。
                                    P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := O.Depth; P.Tuv_Z := O.Depth;
                                    P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                    Report := Report & "I do not know the relation " & Rl
                                              & ", so I went to it. ";
                                 end if;
                              end if;
                           end;
                        else
                           --  编译器已经拦掉"既没说格子也没说关系"的句子;真走到这儿说明编译器漏了。
                           --  不许静默丢掉这一条 —— 出声。
                           Report := Report & "this line named neither a place nor a relation, "
                                     & "so I had nothing to aim at (my compiler should have caught that). ";
                           Ok_Pt := False;
                        end if;
   end Set_Target;

   procedure Build_Goals is
   begin
         --  "保持不动" → 也变成一个点:它的目标就是它此刻在哪(误差为零)。这样解算必须选一条【不动它】的走法,
         --  而不是"没人管它" —— 一只手捏住不动、另一只手干活,全靠这一条(以前这种条目被直接跳过)
         for G of Say.Moves loop
            if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then G.Stay then
               declare
                  It : constant Item := C.Items (G.Item - 1);
                  P : Point;
               begin
                  if It.Located then
                     P.Item_No := G.Item;
                     if It.Kind in Finger | Grip | Piece | Thing_Held then
                        P.Arm := It.Arm; P.Kind := Piece_Pt;
                        P.Chan_K := (if It.Kind = Piece then It.Which else Chan.Per_Arm + It.Jaw_K);
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                        begin
                           P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                        end;
                     elsif Cam_Arm (C, Cam) >= 0 then
                        P.Arm := Natural (Cam_Arm (C, Cam)); P.Kind := Thing_Pt; P.Slot := It.Slot;
                        P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                        P.Elong := It.Elong; P.Gray := It.Gray;
                     end if;
                     P.Tu := P.Cu; P.Tv := P.Cv; P.Tz := P.Z; P.Tuv_Z := P.Z;
                     P.Wz := (if P.Z > 0.0 then 1.0 else 0.0);
                     P.Desc := S (Say_Item (C, G.Item) & " stays exactly where it is");
                     if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                        P.Cam := Cam;   --  这一点是哪台相机里的(此刻只有一台)
                  Pts.Append (P);
                     end if;
                  end if;
               end;
            end if;
            if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then not G.Stay then
               declare
                  It : constant Item := C.Items (G.Item - 1);
                  P : Point;
                  Own : constant Boolean := It.Kind in Finger | Grip | Thing_Held | Piece;
                  Cam_A : constant Integer := Cam_Arm (C, Cam);
                  Ok_Pt : Boolean := True;
               begin
                  Amount := Amount_Factor (G.Amount);
                  P.Item_No := G.Item;
                  P.Hard := G.Hard;   --  脑说的是 hold ⇒ 这一条进硬约束,解算时不许被牺牲
                  if not It.Located then
                     --  🔴 看不见自己那一块也不许停:按身体图算出来的位置当它此刻在哪,照走。
                     Report := Report & "I cannot see " & Say_Item (C, G.Item)
                               & " in this picture, so I used where my joints say it is. ";
                  elsif It.Kind = Piece then
                     --  我身上的一块零件:点 = 它此刻的形心,表按需量(六个通道各推一下)
                     P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := It.Which; P.Blob := -1;
                     P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth;
                     P.Known := C.Zones (Track_Idx (C, P.Arm, Cam)).Pieces_Known (It.Which);
                     P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                  elsif Own then
                     P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm + It.Jaw_K;   --  手指 = 那个抓握通道带的那块
                     declare
                        Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                     begin
                        P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known or else Cam_A = Integer (P.Arm);
                     end;
                     --  🔴🔴 这一条以前写成"只在【正在抓的那条胳膊自己的腕相机】里才算",
                     --  而接触集(挑"哪一段弦窄得塞得进钳口")只在 To_Grip 为真时才跑 ⇒
                     --  从头顶眼开车时它【一次都跑不到】。SC1 实测:整炮 contact set 出现 0 次。
                     --  这是个死结:头顶眼走得过去但没有接触集,腕眼有接触集但贴近走不过去
                     --  (腕眼里"看着多大"要长到爪口那么大,手够不到就顶死)。
                     --  而接触集本身不需要是腕眼 —— 它扫的是物体轮廓,任何一只【同时看得见
                     --  这只爪子和这件东西】的眼都算得出来。⇒ 闸改成问这件事,不问相机长在哪。
                     if G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length)
                       and then C.Items (G.Of_Item - 1).Kind = Thing
                       and then Zone_Of (C, P.Arm, Cam, 0).Valid
                     then
                        --  自己的手上相机里"我的手到 X" = 让 X 的像素来到握区:改跟 X。
                        --  🔴 换了跟的点,【目标也必须跟着换成握区】。以前只换了前者,于是目标是
                        --  它自己的位置,误差恒等于 0 ⇒ 每一次都 0 推就宣布"已经到了"(GG 实测)。
                        declare
                           O : constant Item := C.Items (G.Of_Item - 1);
                        begin
                           P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                           P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                           P.Size := Sqrt (Long_Float (O.Count) / Long_Float'Max (1.0, Long_Float (Cw * Ch)));
                           P.Elong := O.Elong; P.Gray := O.Gray;
                           P.To_Grip := True;
                        end;
                     end if;
                  elsif It.Kind = Thing and then Cam_A >= 0 then
                     P.Arm := Natural (Cam_A); P.Kind := Thing_Pt; P.Slot := It.Slot;
                     P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                     P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                  else
                     --  🔴 这是意见,不是无能(而且编译器已经查过"是不是我身上的")。照走。
                     Report := Report & Say_Item (C, G.Item) & " is not in my hand, I pushed toward it anyway. ";
                  end if;
                  if Ok_Pt then
                     Set_Target (G, P, Ok_Pt);
                  end if;
                  if Ok_Pt then
                     if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                        P.Cam := Cam;   --  这一点是哪台相机里的(此刻只有一台)
                  Pts.Append (P);
                     else
                        Report := Report & "goal for " & Say_Item (C, G.Item) & " needs a different arm than the first goal; I do one arm per segment. ";
                     end if;
                  end if;
               end;
            end if;
         end loop;
         --  抓握 close 在某块上:先把那块装进握区(手上相机里跟块;否则世界相机里握区去它的顶面),笼住了才合
         if Say.Grip = "close" and then Grip_Arm >= 0 and then Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
           and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered and then C.Items (Say.Grip_On - 1).Located
         then
            declare
               O : constant Item := C.Items (Say.Grip_On - 1);
               A : constant Natural := Natural (Grip_Arm);
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
               P : Point;
               Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
            begin
               --  🔴 这台相机里没量到握区 ⇒ 以前整块【静默跳过】,脑只看到一段什么都没干。
               --  新的扫描筛法(动过不止一步的才算手指)会让"看不见自己开合"的相机诚实地交白卷,
               --  所以这条路一定会被走到 —— 必须说出来,并且告诉脑它能怎么办。
               if Pts.Is_Empty and then not Z.Valid then
                  Report := Report & "I have no measured grip for arm " & Codec.Img (A + 1)
                            & " in the eye I am judging this stretch with: when I open and close that hand here, "
                            & "nothing on me sweeps a solid patch of pixels, so I refuse to invent a grip box. "
                            & "Judge this stretch with the eye that rides on that hand, or name the thing in that eye. ";
                  C.Blind_Say := S ("in this eye I cannot see my own hand open and close, so I have no grip box here");
                  Put_Line ("[身]   🔴 相机" & Codec.Img (Cam) & " 里量不到第" & Codec.Img (A + 1)
                            & " 只手的握区(开合扫不出一团像素)⇒ 不编握区,如实说");
               end if;
               if Pts.Is_Empty and then Z.Valid then
                  P.Arm := A; P.Item_No := Say.Grip_On;
                  if Own_Cam then
                     --  🔴 "抓住它" = 让它在画面里和我张开的那片地方重合:位置对上、远近对上、看着一样大、朝向一样。
                     --  这四件全是量出来的区域属性,一个字没提手指几根 —— 两指、七指、吸盘、软体臂同一句话。
                     --  只对中心那一版是错的:三个数管不住六个通道,剩下的自由度乱走(手腕拧、球转出画面)。
                     --  "看着一样大"顺带就是最稳的远近信号(离得越近越大),比一步只变 5 mm、自己抖 5 mm 的深度读数强。
                     --  圆的东西没有朝向 ⇒ 那一行谁也改不动 ⇒ 自动不参与,不需要写规则。
                     P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                     P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                     P.Size := Sqrt (Long_Float (O.Count) / Long_Float'Max (1.0, Long_Float (Cw * Ch)));
                     P.Ang := 2.0 * Arctan (O.Av, O.Au);
                     P.Elong := O.Elong; P.Gray := O.Gray;
                     P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0); P.Tuv_Z := Z.Depth;
                     --  "到了跟前该有多大":看着多大与距离成反比 —— 现在在 d、该到 d*,就该大 d/d* 倍。
                     --  (不能用瓣心距:手自己的眼睛里两指贴在画面两端,那个数 ≈ 整幅画)
                     P.Tsize := (if not Picture.Is_Nan (Z.Depth) and then Z.Depth > 0.0 and then P.Z > 0.0
                                 then P.Size * (P.Z / Z.Depth) else P.Size);
                     P.Tang := 2.0 * Arctan (Z.Av, Z.Au);
                     --  🔴 "看着多大"这一项 2026-09-08 曾被写死关掉(当时框随切块忽大忽小)。现在重新打开:
                     --  稳不稳【由体检量出来判】,不由我写死 —— 不稳的话点用之前那一关会把它摘掉。
                     --  而在手上这只眼睛里,握区的远近常常读不到(NaN),那时它是【唯一】的距离信号:
                     --  GE 实测,两个距离信号同时关着 ⇒ 像素一对齐就宣布"到了",实际差着 20 厘米。
                     P.Wsize := (if not Picture.Is_Nan (Z.Depth) and then Z.Depth > 0.0 and then P.Z > 0.0
                                 then 1.0 else 0.0);
                     --  朝向的分量 = 这块有多长条(圆的为零)
                     P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, O.Elong));
                     P.Desc := S (Say_Item (C, Say.Grip_On) & " to sit where my fingers close (same place, same distance, same apparent size, same lie)");
                  else
                     declare
                        Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                     begin
                        P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                        --  同上:合到它那一面,高低由脑说了算
                        P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 and then Tr.Z > 0.0 then 1.0 else 0.0); P.Tuv_Z := O.Depth;
                        P.Desc := S ("grip " & Codec.Img (A + 1) & " onto " & Say_Item (C, Say.Grip_On) & " (fingertips to its middle)");
                     end;
                  end if;
                  P.Cam := Cam;   --  这一点是哪台相机里的(此刻只有一台)
                  Pts.Append (P);
               end if;
            end;
         end if;
   end Build_Goals;

   --  接触集(09-29 重写,PLAN §2 ②):到它上方(两眼交点 + 指尖朝下,现成)→ 量出来的手在它的形状上挑一组(Plan_Contact)→
   --  眼转到那一组的朝向、到悬停点(下手处沿进场方向往回退一个张口)→ 每一块先合到离料还剩一点(Pre)→ 沿进场方向往下,
   --  碰到没有按 Selfmap.Blocked(同碰桌面量指尖)。下到下手那一处之前一步以上就被挡住 = 手指落在了东西上(它自己别处、旁边的东西)
   --  ⇒ 尖那一刻在哪记进 C.Bumps,抬回悬停点,重挑(试一下就知道);下到了 ⇒ 交给 Do_Grip 合、抬一点看它跟不跟手。
   --  往下伸看着走(09-30,owner 09-29"为啥会有 3mm 这种数字"):不再"每步 4 倍最小一档",步子全从量到的不准来 ——
   --  最先可能碰到它的那一层在哪,按它顶面的不准(轮廓横着的误差 Sil_Err、那张面高低的不准 Sil_H_Sd)、尖的不准(Tip_Sd)、
   --  手这一次到位差多少(Eye_To 量到的)合起来的 Stats.Z 倍算出一段"可能碰到"的带子;带子之外一条命令下去,
   --  带子里一步 = 带子宽 ÷ Blocked 要的空走步数(碰上之前要有这么多步空走当底),再细也不细过身体量得出的那一档
   procedure Contact_Onto (Amt : Long_Float) is
      Ev1 : Unbounded_String;
      St1, Bt1 : Natural;
      Arm : constant Natural := Natural (Own);
      Cam1 : constant Natural := Natural (Geo_Cam);
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam1);
      --  到没到的分辨率(都是量的):位置 = 读数噪声和这只眼配点噪声折到手上(这只手挪一档 = 自己那只眼里挪 1 像素,配点准到 G.Rms 像素)
      --  两样里大的那个的 Stats.Z 倍;转动 = 读数噪声和这只眼的角度噪声(G.Rms ÷ 焦距)里大的那个的 Stats.Z 倍
      Floor_P : constant Long_Float := Stats.Z * Long_Float'Max (C.Map.EE_Noise, Geo_Base (C, Arm) * G.Rms);
      Floor_R : constant Long_Float := Stats.Z * Long_Float'Max (C.Map.Rot_Noise, (if G.F > 0.0 then G.Rms / G.F else 0.0));
      Pick : Contact.Grasp.Candidate;
      Note : Unbounded_String;
      Pok : Boolean;
      --  眼走到 (R, T):一条命令 = 此刻还差的平移 + 转动(世界轴),走完看还差多少。差到分辨率以内就到;
      --  一条命令下去还差的没少过分辨率(身体到头了、被挡住、这就是它能到的最近)就停,照实说还差多少(Miss 交出去,往下伸时算进不准里)
      procedure Eye_To (R : Geom.M3; T : Geom.V3; Said : String; Arrived : out Boolean; Miss_Out : out Long_Float) is
         Prev_Miss, Prev_Rot : Long_Float := Long_Float'Last;
      begin
         Arrived := False; Miss_Out := Long_Float'Last;
         loop
            declare
               P : constant Plug.Arm_Pose := F.EE (Arm);
               Rc : constant Geom.M3 := Geom.Cam_R (G, P);
               Oc : constant Geom.V3 := Geom.Cam_Pos (G, P);
               Rv : constant Geom.V3 := Geom.Rot_Vec (Geom.Mul (R, Geom.Tr (Rc)));   --  世界轴:从此刻的朝向转到要的
               Miss : constant Long_Float := Geom.Norm ([T (0) - Oc (0), T (1) - Oc (1), T (2) - Oc (2)]);
               Mrot : constant Long_Float := Geom.Norm (Rv);
               --  手的位姿要到哪:手 → 世界 = R · R_ceᵀ;眼的中心 = 手的位置 + 手 → 世界 · Off
               Rp : constant Geom.M3 := Geom.Mul (R, Geom.Tr (G.R_Ce));
               Ow : constant Geom.V3 := Geom.Ap (Rp, G.Off);
               Av : Table.Vec := Table.Zero_Vec;
               Jaw : Floats;
               Del : Table.Vec;
               Mok : Boolean;
            begin
               Miss_Out := Miss;
               if Miss <= Floor_P and then Mrot <= Floor_R then
                  Arrived := True;
                  return;
               end if;
               if Miss > Prev_Miss - Floor_P and then Mrot > Prev_Rot - Floor_R then
                  --  上一条命令以后差的没少过分辨率:这就是这只手此刻能到的最近
                  Arrived := Miss <= Prev_Miss + Floor_P and then Mrot <= Prev_Rot + Floor_R;
                  Put_Line ("[身] ✋ " & Said & ":还差 " & Mm (Miss) & "、转 " & Codec.Fmt (Mrot, 3) & " rad,上一条以后没再变近 ⇒ 停在这儿");
                  return;
               end if;
               Prev_Miss := Miss; Prev_Rot := Mrot;
               for I in 0 .. 2 loop
                  Av (I) := T (I) - Ow (I) - P (I);
                  Av (3 + I) := Rv (I);
               end loop;
               Step_Arm (L, C, F, Arm, Av, Jaw, Del, Mok, Geo_Settle => True);
               Steps_Taken := Steps_Taken + 1;
               if Plug.Reset_Pending (L) then
                  return;
               end if;
            end;
         end loop;
      end Eye_To;
      --  沿 Dir 往下压,每步 Lstep,最多 Dist 那么深(还差不到半步就算到了:四舍五入),按 Selfmap.Blocked 认挡住(同碰桌面量指尖:
      --  比上一步空走多少走的量超过这一步的百分之一 / 3 倍读数噪声 / 3 倍前两步空走之差);Went = 实际往 Dir 走了多少
      --  Prev / Prev2 / N_Free 交回去:后面接着的那一条命令拿它们当底
      procedure Press_Along (Dir : Geom.V3; Dist, Lstep : Long_Float; Hit : out Boolean; Went : out Long_Float; Limit : out Boolean;
                             Prev, Prev2 : in out Long_Float; N_Free : in out Natural) is
      begin
         Hit := False; Went := 0.0; Limit := False;
         while Went + 0.5 * Lstep < Dist loop   --  还差不到半步就算到了(四舍五入,数学)
            declare
               Cur : constant Plug.Arm_Pose := F.EE (Arm);
               Av : Table.Vec := Table.Zero_Vec;
               Pe, Re : Long_Float;
               Rok, Mok : Boolean;
            begin
               for I in 0 .. 2 loop
                  Av (I) := Lstep * Dir (I);
               end loop;
               Plug.Reach (Arm, Chan.Compose (Cur, Av), Pe, Re, Rok);
               if Rok and then (Pe > Floor_P or else Re > Floor_R) then
                  Limit := True;   --  这一步在量到的关节范围里反解到不了(差得比分辨率还多)
                  return;
               end if;
               Geo_Move (L, C, F, Arm, [Av (0), Av (1), Av (2)], Mok, Press => True);
               Steps_Taken := Steps_Taken + 1;
               declare
                  Now : constant Plug.Arm_Pose := F.EE (Arm);
                  Moved : constant Long_Float := (Now (0) - Cur (0)) * Dir (0) + (Now (1) - Cur (1)) * Dir (1) + (Now (2) - Cur (2)) * Dir (2);
                  Short : constant Long_Float := Lstep - Moved;
               begin
                  Went := Went + Moved;
                  if Selfmap.Blocked (Short, Prev, Prev2, N_Free, Lstep, C.Map.EE_Noise) then
                     Hit := True;
                     return;
                  end if;
                  Prev2 := Prev; Prev := Short; N_Free := N_Free + 1;
               end;
            end;
         end loop;
      end Press_Along;
      --  每一个尖此刻在世界里在哪(挡住时记进 C.Bumps)
      procedure Note_Tips_As_Bumps is
         P : constant Plug.Arm_Pose := F.EE (Arm);
         O : constant Geom.V3 := Geom.Cam_Pos (G, P);
         Rc : constant Geom.M3 := Geom.Cam_R (G, P);
      begin
         for Lg of G.Lobes loop
            declare
               T : constant Geom.V3 := Geom.Ap (Rc, Lg.Tip);
            begin
               C.Bumps.Append (Geom.V3'[O (0) + T (0), O (1) + T (1), O (2) + T (2)]);
            end;
         end loop;
      end Note_Tips_As_Bumps;
      Tried : Contact.V3_Vectors.Vector;   --  挑过的下手处(眼的位置):又挑回同一处 = 候选里没有新的了
   begin
      Grasp_Valid := False;
      Geo_Approach (L, C, F, Cam1, Arm, Geo_Slot_Now, 0, Ev1, St1, Bt1, Above => True, Amt => Amt, Until_Touch => False, Name => Geo_Name);
      if Index (Ev1, "reset:") = 1 then
         Event := Ev1;
         return;
      end if;
      Put_Line ("[身] 📐 进场:先到它上方 ⇒ " & To_String (Ev1) & "(" & Codec.Img (St1) & " 推)");
      Steps_Taken := St1; Beats := Bt1;
      --  没到上方:被一个面顶住(contact)说明我已经贴着它躺的面了,接触集照样能从这儿把手抬到悬停点再下去;别的(看丢、顶死)就没法继续
      if Index (Ev1, "amount: arrived") = 0 and then Index (Ev1, "contact") = 0 then
         Event := S ("on the way to a point above it: ") & Ev1;
         return;
      end if;
      --  伸下去被挡住就记下来重挑:挡住的尖进 C.Bumps,候选只会越来越少;挑不出来了、或者又挑回挑过的那一处,就停
      loop
         Plan_Contact (C, F, Arm, Cam1, Geo_Name, Pick, Note, Pok);
         Put_Line ("[身] ✋ " & To_String (Note));
         Report := Report & To_String (Note) & ". ";
         if not Pok then
            Event := S ("lost: I could not lay out a hold on it - ") & Note;
            return;
         end if;
         declare
            Again : Boolean := False;
         begin
            for Q of Tried loop
               if Geom.Norm ([Q (0) - Pick.T (0), Q (1) - Pick.T (1), Q (2) - Pick.T (2)]) <= Floor_P then
                  Again := True;
               end if;
            end loop;
            if Again then
               Event := S ("lost: I was stopped above the hold and the next hold I can lay out is that same place again - something is in the way there");
               return;
            end if;
            Tried.Append (Pick.T);
         end;
         declare
            A : constant Geom.V3 := Pick.Approach;
            Stand : constant Long_Float := G.Gap;   --  悬停:沿进场方向往回退一个张口(同原来的 Standoff)
            Hover : constant Geom.V3 := [Pick.T (0) - Stand * A (0), Pick.T (1) - Stand * A (1), Pick.T (2) - Stand * A (2)];
            Arr, Hit, Lim : Boolean;
            Miss, Went : Long_Float;
         begin
            Eye_To (Pick.R, Hover, "转到这一组的朝向、到悬停点", Arr, Miss);
            if Plug.Reset_Pending (L) then
               Event := S (Reset_Event);
               return;
            end if;
            if not Arr then
               --  没到悬停点、这只手又到不了更近:往下伸,手指就不在它两边(09-30 审计:原来不看到没到照样压)
               Event := S ("stuck: I could not get my hand to the point above the hold (still ") & Len (C, Miss) & " away), so I did not go down";
               return;
            end if;
            --  每一块先合到离料还剩一点(Pre;张口和抓握读数按线性换算 —— 开机只量了张开、合空两头的读数,说出来)
            if Pick.Pre > 0.0 and then not Hand_Of (C, Arm, Say.Grip_K).Measured then
               Event := S ("refused: I never measured which readings open and close this grip, so I cannot pre-close it before going down");
               return;
            end if;
            if Pick.Pre > 0.0 then
               declare
                  Hk : constant Zone.Hand := Hand_Of (C, Arm, Say.Grip_K);
                  Half : constant Long_Float := 0.5 * (Geom.Norm ([G.Lobes (1).Tip (0) - G.Lobes (0).Tip (0), G.Lobes (1).Tip (1) - G.Lobes (0).Tip (1),
                                                                     G.Lobes (1).Tip (2) - G.Lobes (0).Tip (2)]) - Long_Float'Max (G.Lobes (0).Thin, G.Lobes (1).Thin));
                  Frac : constant Long_Float := (if Half > 0.0 then Long_Float'Min (1.0, Pick.Pre / Half) else 0.0);
                  Steps_J : Natural;
                  Reading : Long_Float;
               begin
                  Move_Jaw (L, C, F, Arm, Hk.Open_Reading + Frac * (Hk.Empty_Close - Hk.Open_Reading), Steps_J, Reading, Say.Grip_K);
                  Put_Line ("[身] ✋ 下去之前每一块先合 " & Mm (Pick.Pre) & "(行程的 " & Codec.Fmt (100.0 * Frac, 0) & "%;读数按张开、合空两头线性换算)⇒ 读数 " & Codec.Fmt (Reading, 3));
               end;
            end if;
            declare
               X_Tip : Long_Float := Long_Float'First;   --  下手那一刻最靠前(沿进场方向)的那个尖
               X_Top : Long_Float := Long_Float'Last;    --  它最先会被碰到的那一层(沿进场方向最靠后的表面点)
               An : constant Long_Float := abs (A (0) * C.Sil_N (0) + A (1) * C.Sil_N (1) + A (2) * C.Sil_N (2));
               Dp : Descent;
               Band, Lstep, Fast : Long_Float;
               Fast_Went : Long_Float := 0.0;
            begin
               for Lg of G.Lobes loop
                  declare
                     Tw : constant Geom.V3 := Geom.Ap (Pick.R, Lg.Tip);
                  begin
                     X_Tip := Long_Float'Max (X_Tip, (Pick.T (0) + Tw (0)) * A (0) + (Pick.T (1) + Tw (1)) * A (1) + (Pick.T (2) + Tw (2)) * A (2));
                  end;
               end loop;
               for Q of C.Sil_Pts loop
                  X_Top := Long_Float'Min (X_Top, Q (0) * A (0) + Q (1) * A (1) + Q (2) * A (2));
               end loop;
               Dp := Plan_Descent (Stand, (if C.Sil_Pts.Is_Empty then 0.0 else X_Tip - X_Top), C.Sil_Err, C.Sil_H_Sd, An, G.Tip_Sd, Miss, C.Map.EE_Noise, Floor_P);
               Band := Dp.Band; Lstep := Dp.Lstep; Fast := Dp.Fast;
               if Fast > Lstep then
                  declare
                     Cur : constant Plug.Arm_Pose := F.EE (Arm);
                     Mok : Boolean;
                  begin
                     Geo_Move (L, C, F, Arm, [Fast * A (0), Fast * A (1), Fast * A (2)], Mok, Press => True);
                     Steps_Taken := Steps_Taken + 1;
                     declare
                        Now : constant Plug.Arm_Pose := F.EE (Arm);
                     begin
                        Fast_Went := (Now (0) - Cur (0)) * A (0) + (Now (1) - Cur (1)) * A (1) + (Now (2) - Cur (2)) * A (2);
                     end;
                  end;
               end if;
               Put_Line ("[身] ✋ 往下伸:可能碰到它的那条带子半宽 " & (if Band < Long_Float'Last then Mm (Band) else "(它顶面高低量不出)")
                         & "(顶面横着 " & Mm (C.Sil_Err) & "、高低 " & (if C.Sil_H_Sd < Long_Float'Last then Mm (C.Sil_H_Sd) else "量不出") & "、尖 " & Mm (G.Tip_Sd) & "、到位 " & Mm (Miss)
                         & ",合起来的 " & Codec.Fmt (Stats.Z, 0) & " 倍)⇒ 先一条命令下 " & Mm (Fast_Went) & ",再每步 " & Mm (Lstep) & " 探到 " & Mm (Dp.Fine_End)
                         & "(尖过了它顶面那一层),剩下的一条命令");
               if Fast > Lstep and then Fast - Fast_Went > Floor_P + Lstep then
                  --  带子外那一条命令就没走完:在我以为碰不到它的地方就被挡住了(它顶面比量到的高、或旁边有东西)
                  Hit := True; Went := Fast_Went; Lim := False;
               else
                  declare
                     Prev, Prev2 : Long_Float := 0.0;
                     N_Free : Natural := 0;
                  begin
                     Press_Along (A, Dp.Fine_End - Fast_Went, Lstep, Hit, Went, Lim, Prev, Prev2, N_Free);
                     Went := Went + Fast_Went;
                     if not Hit and then not Lim and then Went + 0.5 * Lstep < Stand then
                        --  尖过了它顶面那一层、在它两边了:剩下到下手处一条命令;挡没挡,Blocked 拿前面小步空走的少走量当底
                        declare
                           Rest : constant Long_Float := Stand - Went;
                           Cur : constant Plug.Arm_Pose := F.EE (Arm);
                           Mok : Boolean;
                        begin
                           Geo_Move (L, C, F, Arm, [Rest * A (0), Rest * A (1), Rest * A (2)], Mok, Press => True);
                           Steps_Taken := Steps_Taken + 1;
                           declare
                              Now : constant Plug.Arm_Pose := F.EE (Arm);
                              Moved : constant Long_Float := (Now (0) - Cur (0)) * A (0) + (Now (1) - Cur (1)) * A (1) + (Now (2) - Cur (2)) * A (2);
                           begin
                              Went := Went + Moved;
                              Hit := Selfmap.Blocked (Rest - Moved, Prev, Prev2, N_Free, Rest, C.Map.EE_Noise);
                           end;
                        end;
                     end if;
                  end;
               end if;
               Put_Line ("[身] ✋ 沿进场方向往下 " & Mm (Went) & "(要 " & Mm (Stand) & ")" & (if Lim then ",再往下在量到的关节限位里解不出来"
                         elsif Hit then ",被挡住" else ",下到了"));
               if Hit and then Went < Stand - Lstep then
                  --  下到下手的高度之前一步以上就被挡住:手指落在了东西上 ⇒ 记下尖此刻在哪,抬回去重挑
                  Note_Tips_As_Bumps;
                  Report := Report & "my fingers were stopped " & Len (C, Stand - Went) & " above where they should go down to (something is under them there), so I marked that spot and laid the hold out again. ";
                  Eye_To (Pick.R, Hover, "被挡住,抬回悬停点", Arr, Miss);
               else
                  Grasp_Valid := not Lim;
                  exit;
               end if;
            end;
         end;
      end loop;
      --  合上以后交给 Do_Grip:接触集(接触点、朝里的法向、按摩擦锥)和这一组要的摩擦(拿住 / 没拿住都按它记)
      if Grasp_Valid then
         Grasp_Set := (Points => Contact.Point_Vectors.Empty_Vector, Motion => Contact.Still (Pick.T), Has_Approach => True, Approach => Pick.Approach);
         for T of Pick.Touches loop
            Grasp_Set.Points.Append (Contact.Point'(By => (Kind => Contact.Hand, Id => 0), Pos => T.P, Normal => [-T.N (0), -T.N (1), -T.N (2)],
                                                    Push => (Axis => T.N, Half_Angle => Arctan (Pick.Mu_Worst)), Pull => False, Torsion => T.Twist_R > 0.0,
                                                    Peel => False, Tol_M => Floor_P));
         end loop;
         Grasp_Mu_Nom := Pick.Mu_Nom; Grasp_Mu_Worst := Pick.Mu_Worst;
         Event := S ("contact: my fingers are down around it where the hold was laid out (turned to the hold, came to the hover point, then down along the approach)");
      elsif Event = Null_Unbounded_String then
         Event := S ("lost: every time I went down my fingers were stopped above the hold - something is in the way");
      end if;
   end Contact_Onto;

   --  拿着它抬:沿它躺的面的法向(碰过的面按量到的法向,没碰过按"上")走一个单位(4 倍探针幅度 × 脑的档位,同贴近时那把尺)。
   --  抬完看手指读数:掉回空手值 = 它掉了(slipped);命令了没走到一半 = 胳膊到头(resist);否则 settled,并说它还在手里。
   --  改手里东西的一个量:沿"让它变的方向"(Axis,单位向量)平移一个单位 —— 任何量同一条路;抬只是 height 这个量往上
   procedure Change_Held_Qty (Arm : Natural; Amt : Long_Float; Axis : Geom.V3; Qty : String) is
      Nn : constant Geom.V3 := Axis;
      Ln : constant Long_Float := Stride_Of (C, Arm) * Amt;   --  一个单位 = 量出来的最大一档 × 脑的档位
      Cur : constant Plug.Arm_Pose := F.EE (Arm);
      Dw : constant Geom.V3 := [Nn (0) * Ln, Nn (1) * Ln, Nn (2) * Ln];
      Mok : Boolean;
      --  抬了多高:沿它躺的面的法向量(不是位移的长度)
      function Rise (From, To : Plug.Arm_Pose) return Long_Float is
        ((To (0) - From (0)) * Nn (0) + (To (1) - From (1)) * Nn (1) + (To (2) - From (2)) * Nn (2));
      Went : Long_Float := 0.0;
      Note : Unbounded_String;
   begin
      if Ln <= 0.0 then
         Event := S ("refused: I have not measured how far one push moves this arm, so I cannot change its " & Qty & " by a known amount");
         return;
      end if;
      if Plug.Reset_Pending (L) then
         Event := S (Reset_Event);
         return;
      end if;
      --  接触集(PLAN 1.5):抬 = 手里那几个接触点沿它躺的面的法向平移一个单位(第③格是旋量);判据只说,不拦(身体不许因为"算出来做不到"而不动)
      if C.Held_Set_Valid then
         declare
            S2 : Contact.Set := C.Held_Set;
            St : Contact.Exec.Step_Vectors.Vector;
            Why : Contact.Exec.No_Plan;
            use type Contact.Exec.No_Plan_Kind;
         begin
            S2.Motion := Contact.Slide (Dw);
            Contact.Exec.Steps (S2, (Standoff_M => Geo_Base (C, Arm), Repeat_M => Geo_Base (C, Arm)), True, 1, St, Why);
            Geo_Say ("接触集:改它的量(" & Qty & ")= " & Codec.Img (Natural (S2.Points.Length)) & " 个接触点沿法向平移 " & Mm (Ln) & " ⇒ "
                     & (if Why.Kind = Contact.Exec.Fine then "航点 " & Codec.Img (Natural (St.Length)) & " 步" else "判据说 " & Contact.Exec.Img (Why) & ",照抬,抬完看手指读数"));
         end;
      end if;
      Geo_Move (L, C, F, Arm, Dw, Mok);
      Steps_Taken := Steps_Taken + 1;
      declare
         Now : constant Plug.Arm_Pose := F.EE (Arm);
         Got : constant Long_Float := Rise (Cur, Now);
      begin
         Went := Got;
         --  🔴 直着往上被顶住 = 胳膊伸到头了(H50 2026-09-23 实测:右臂横跨整桌夹住剪刀,抬到 8 cm 后"命令 0.051 只走 0.000",仿真要 10 cm)。
         --  顶住我的方向是量出来的(命令的位移减实到的位移);把"上"里顺着那个方向的那一份去掉,剩下的还走得了 ——
         --  和贴近时"被一个面顶着就沿着面走"是同一条:胳膊的边界也是一个面。
         if Got + Got < Ln then
            declare
               Miss : constant Geom.V3 := [Dw (0) - (Now (0) - Cur (0)), Dw (1) - (Now (1) - Cur (1)), Dw (2) - (Now (2) - Cur (2))];
               Ml : constant Long_Float := Geom.Norm (Miss);
            begin
               if Ml > 0.0 then
                  declare
                     Wall : constant Geom.V3 := [Miss (0) / Ml, Miss (1) / Ml, Miss (2) / Ml];
                     Into : constant Long_Float := Nn (0) * Wall (0) + Nn (1) * Wall (1) + Nn (2) * Wall (2);
                     D2 : constant Geom.V3 := [Nn (0) - Into * Wall (0), Nn (1) - Into * Wall (1), Nn (2) - Into * Wall (2)];
                     L2 : constant Long_Float := Geom.Norm (D2);
                  begin
                     if L2 > 0.0 then
                        declare
                           Dw2 : constant Geom.V3 := [D2 (0) / L2 * Ln, D2 (1) / L2 * Ln, D2 (2) / L2 * Ln];
                           Mid : constant Plug.Arm_Pose := F.EE (Arm);
                        begin
                           Geo_Say ("沿要的方向被顶住(方向 (" & Codec.Fmt (Wall (0), 2) & "," & Codec.Fmt (Wall (1), 2) & "," & Codec.Fmt (Wall (2), 2)
                                    & "),是胳膊的边界)⇒ 沿着边界再走一步");
                           Geo_Move (L, C, F, Arm, Dw2, Mok);
                           Steps_Taken := Steps_Taken + 1;
                           Went := Went + Rise (Mid, F.EE (Arm));
                           Note := S (" (that direction was blocked by my own reach, so I moved along the edge of it)");
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end;
      declare
         Jk : constant Natural := Natural (Integer'Max (0, C.Wld.Held_Jaw));
         Have_R : constant Boolean := Selfmap.Has_Jaw (F, Arm, Jk);
         R_Now : constant Long_Float := (if Have_R then Selfmap.Jaw_Of (F, Arm, Jk) else 0.0);   --  没读数时不用它(下面先问 Have_R)
         Emp : Long_Float := 0.0;
         Hf : Zone.Hand;
         Found : Boolean := False;
      begin
         for H of C.Hands loop
            if H.Arm = Arm and then H.K = Jk and then H.Measured then
               Emp := H.Empty_Close; Hf := H; Found := True;
            end if;
         end loop;
         if not Have_R then
            Event := S ("settled: I moved it " & Len (C, Went) & " along the direction that changes its " & Qty
                        & "; my fingers report no reading this beat, so I cannot tell whether it is still between them") & Note;
         elsif Found and then Past_Empty (Hf, R_Now) <= C.Map.Jaw_Noise then
            Event := S ("slipped: I moved my hand " & Len (C, Went) & " along that direction and my fingers closed to their empty reading - it is no longer between them");
            C.Wld.Holding := False;
         elsif Went + Went < Ln then   --  两下加起来还不到要的一半(纯数学的一半)
            Event := S ("resist: I commanded " & Len (C, Ln) & " along the direction that changes its " & Qty & " and moved only " & Len (C, Went) & " - my arm cannot go further that way from here; it is still between my fingers") & Note;
         else
            Event := S ("settled: I moved it " & Len (C, Went) & " along the direction that changes its " & Qty & "; it is still between my fingers (reading "
                        & Codec.Fmt (R_Now, 3) & ", empty would be " & Codec.Fmt (Emp, 3) & ")") & Note;
         end if;
      end;
   end Change_Held_Qty;

   --  拿没拿住,按这一组要的摩擦记这件东西和这只身体之间的摩擦(Grip_Mu):拿住 ⇒ 法向取最坏时要的那么多它给得起(下限往上走);
   --  没拿住 ⇒ 按量到的法向要的那么多它给不起(上限往下走)。下一次挑下手处按它们
   procedure Note_Grip_Mu (Name : Unbounded_String; Held : Boolean) is
      Found : Boolean := False;
   begin
      for I in 0 .. Natural (C.Grip_Mus.Length) - 1 loop
         if C.Grip_Mus (I).Name = Name then
            declare
               M : Grip_Mu := C.Grip_Mus (I);
            begin
               if Held then
                  M.Lb := Long_Float'Max (M.Lb, Grasp_Mu_Worst);
               else
                  M.Ub := Long_Float'Min (M.Ub, Grasp_Mu_Nom);
               end if;
               C.Grip_Mus.Replace_Element (I, M);
            end;
            Found := True;
         end if;
      end loop;
      if not Found then
         C.Grip_Mus.Append (Grip_Mu'(Name => Name, Lb => (if Held then Grasp_Mu_Worst else 0.0), Ub => (if Held then Long_Float'Last else Grasp_Mu_Nom)));
      end if;
      Put_Line ("[身] ✋ " & To_String (Name) & (if Held then " 拿住了 ⇒ 它和这只手之间的摩擦至少 " & Codec.Fmt (Grasp_Mu_Worst, 2)
                                                  else " 没拿住 ⇒ 这一组要的摩擦 " & Codec.Fmt (Grasp_Mu_Nom, 2) & " 它给不起"));
   end Note_Grip_Mu;

   procedure Do_Grip is
   begin
         --  ── 抓握 ──
         if Say.Grip = "close" and then Grip_Arm >= 0 then
            declare
               A : constant Natural := Natural (Grip_Arm);
               Caged : constant Boolean := True;   --  永远合:成没成由合完提一提来判,不由合之前的预测来判
               Cage_Note : Unbounded_String;
               Steps_J : Natural;
               Reading : Long_Float;
               Hz : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Say.Grip_K);
            begin
               --  笼判据:点名的那块的像素在握区框里(它的形心落在区框内),深度和手指对得上
               if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                  declare
                     Pin : Point;
                     Found : Boolean := False;
                     Nz : Long_Float := 0.0;
                  begin
                     for P of Pts loop
                        if P.Item_No = Say.Grip_On then
                           if P.Kind = Piece_Pt then
                              --  瓣点取均值 = 区心;深度取最近的那一瓣
                              if not Found then
                                 Pin := P; Pin.Cu := 0.0; Pin.Cv := 0.0; Pin.Z := 1.0e30;
                              end if;
                              Pin.Cu := Pin.Cu + P.Cu; Pin.Cv := Pin.Cv + P.Cv; Nz := Nz + 1.0;
                              if P.Z > 0.0 then
                                 Pin.Z := Long_Float'Min (Pin.Z, P.Z);
                              end if;
                           else
                              Pin := P;
                           end if;
                           Found := True;
                        end if;
                     end loop;
                     if Found and then Pin.Kind = Piece_Pt and then Nz > 0.0 then
                        Pin.Cu := Pin.Cu / Nz; Pin.Cv := Pin.Cv / Nz;
                        --  1e29 = "没读到"的哨兵(无量纲)
                        if Pin.Z >= 1.0e29 then
                           Pin.Z := 0.0;
                        end if;
                     end if;
                     if Found and then Pin.Kind = Thing_Pt and then Hz.Valid then
                        --  🔴 这里原来有一道"笼住了没有"的预测闸(位置/远近/看着多大三项容差),
                        --  它拦住过真实的合手动作,而三项容差全是"量出来的东西 × 人拍的系数":
                        --  横向 = 两指间距 × 0.25(在手自己的眼睛里 ≈ 四分之一张画)、
                        --  远近 = 球自己鼓出桌面那么高。GJ 实测:手离球十几厘米,三项全"过"。
                        --  【删掉】。预测不是判据:脑说合就合,成没成由【合完提一提】来判 —— 那是真实验。
                        declare
                           Dp : constant Long_Float := Sqrt ((Pin.Tu - Pin.Cu) ** 2 + (Pin.Tv - Pin.Cv) ** 2);
                        begin
                           Cage_Note := S ("before I closed, it was " & Codec.Fmt (Dp, 3)
                                           & " of a frame from where my fingers close, and looked "
                                           & Codec.Fmt (Pin.Size / Long_Float'Max (1.0e-9, Pin.Tsize) * 100.0, 0)
                                           & "% of the size it should - I closed anyway and let the lift decide");
                        end;
                     end if;
                  end;
               end if;
               --  🔴 脑说合就合。这里不再有任何"我觉得还不到时候"的判断。
               if Caged and then not Hand_Of (C, A, Say.Grip_K).Measured then
                  --  这个抓握通道开机没推到两头量过 ⇒ 不知道哪个读数是合(09-30:原来按缺省"合 0"= x5 的约定照发)
                  Did_Grip := S ("I did NOT close grip " & Codec.Img (A + 1) & ": I never measured which reading closes it (its two ends were not measured at boot)");
               elsif Caged then
                  --  合 = 发合拢那头的读数(开机两头推到头量的,V1b ②;原来写死 0.0 —— 读数在 0–1、0 = 合是 x5 的约定)
                  Move_Jaw (L, C, F, A, Hand_Of (C, A, Say.Grip_K).Empty_Close, Steps_J, Reading, Say.Grip_K);
                  declare
                     Empty : constant Long_Float := Hand_Of (C, A, Say.Grip_K).Empty_Close;
                     By_Reading : Boolean := Past_Empty (Hand_Of (C, A, Say.Grip_K), Reading) > C.Map.Jaw_Noise;
                     Sure_Held : Boolean := False;
                     Note : Unbounded_String;
                     Origin : Picture.Region;
                     Obj_Count : Natural := 0;
                  begin
                     --  Slot 的声明就是 `Integer := -1`("世界图里没有槽"的东西正是 -1),
                     --  只守 Grip_On 的范围挡不住 Natural(-1) ⇒ CONSTRAINT_ERROR 当场打死整炮。
                     if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
                       and then C.Items (Say.Grip_On - 1).Slot >= 0
                     then
                        Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                        Obj_Count := C.Items (Say.Grip_On - 1).Count;
                     end if;
                     Held_Test (L, C, F, A, Cam, Origin,
                                (if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
                                 then C.Items (Say.Grip_On - 1).Slot else -1),
                                Obj_Count, By_Reading, Sure_Held, Note);
                     if not Sure_Held then
                        By_Reading := False;   --  说不准 ⇒ 不许记成"手里有东西"(记错了下一步它就去"搬"而不是重抓)
                     end if;
                     --  🔴 抓没抓到,是这本经历账里最要紧的一条 —— 它以后说"上次我在这上面是怎么成的",
                     --  靠的就是这一行。判据用的是身体自己抬手量出来的那一条,不是我替它写的。
                     Codec.Append_Line (Life_Path,
                                        "beat " & Codec.Img (Plug.Steps (L))
                                        & " | eye " & Codec.Img (Cam)
                                        & " | CLOSED on " & Say_Item (C, Say.Grip_On)
                                        & " | " & (if not Sure_Held then "COULD NOT TELL"
                                                   elsif By_Reading then "HELD - it came with my hand"
                                                   else "NOT HELD - it did not come with my hand"));
                     Did_Grip := S ("I closed grip " & Codec.Img (A + 1) & " until the picture stopped changing (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) &
                                    ", empty-close reading " & Codec.Fmt (Empty, 3) & "); " & To_String (Note));
                     if By_Reading then
                        C.Wld.Holding := True; C.Wld.Held_Arm := Integer (A); C.Wld.Held_Jaw := Integer (Say.Grip_K); C.Wld.Held_Cam := Integer (Cam);
                        --  接触集:拿住了 ⇒ 这一把的摩擦够(这就是身体量 μ 的办法)⇒ 手里的接触集记下来,锥放开到半空间(抬/搬按它算)
                        if Grasp_Valid then
                           Note_Grip_Mu (Geo_Name, Held => True);
                           C.Held_Set := Grasp_Set;
                           for I in 0 .. Natural (C.Held_Set.Points.Length) - 1 loop
                              declare
                                 P : Contact.Point := C.Held_Set.Points (I);
                              begin
                                 P.Push.Half_Angle := 0.5 * Ada.Numerics.Pi;
                                 C.Held_Set.Points.Replace_Element (I, P);
                              end;
                           end loop;
                           C.Held_Set_Valid := True;
                        end if;
                        if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
                          and then C.Items (Say.Grip_On - 1).Slot >= 0
                        then
                           C.Wld.Held_Slot := C.Items (Say.Grip_On - 1).Slot;
                           C.Wld.Held_Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                        else
                           C.Wld.Held_Slot := -1;
                        end if;
                        Memory.Set (C.Mem, "holding", "arm " & Codec.Img (A + 1) & " closed on " & Say_Item (C, Say.Grip_On) & " at reading " & Codec.Fmt (Reading, 3));
                     else
                        C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Jaw := -1;
                        C.Held_Set_Valid := False;
                        if Grasp_Valid then
                           Note_Grip_Mu (Geo_Name, Held => False);   --  没拿住:这一组要的摩擦它给不起(不是"这一处拉黑")
                        end if;
                        Move_Jaw (L, C, F, A, Hand_Of (C, A, Say.Grip_K).Open_Reading, Steps_J, Reading, Say.Grip_K);   --  走到这里手一定量过(上面没量过就不合)
                        Append (Did_Grip, "; I opened it again");
                     end if;
                  end;
               else
                  Did_Grip := S ("I did NOT close grip " & Codec.Img (A + 1) & ": " & To_String (Cage_Note));
               end if;
               if Cage_Note /= "" then
                  Append (Did_Grip, " (" & To_String (Cage_Note) & ")");
               end if;
            end;
         elsif Say.Grip = "open" and then Grip_Arm >= 0 then
            declare
               A : constant Natural := Natural (Grip_Arm);
               Steps_J : Natural;
               Reading : Long_Float;
            begin
               if not Hand_Of (C, A, Say.Grip_K).Measured then
                  --  没量过哪个读数是张开 ⇒ 不张(09-30:原来按缺省"张 1"= x5 的约定照发)
                  Did_Grip := S ("I did NOT open grip " & Codec.Img (A + 1) & ": I never measured which reading opens it (its two ends were not measured at boot)");
               else
                  Move_Jaw (L, C, F, A, Hand_Of (C, A, Say.Grip_K).Open_Reading, Steps_J, Reading, Say.Grip_K);
                  C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Jaw := -1; C.Wld.Held_Slot := -1;
                  C.Held_Set_Valid := False;
                  Memory.Set (C.Mem, "holding", "");
                  Did_Grip := S ("I opened grip " & Codec.Img (A + 1) & " (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) & ")");
               end if;
            end;
         end if;
         if Did_Grip /= "" then
            Report := Report & To_String (Did_Grip) & ". ";
         end if;
   end Do_Grip;

   begin
      for N of Say.Avoid loop
         if N >= 1 and then N <= Natural (C.Items.Length) then
            Avoid.Append (C.Items (N - 1));
         end if;
      end loop;
      --  ── 几何走法(要动的那条胳膊上长着一只几何常数齐的眼时)──:贴近/瞄进 = 视线一交;合 = 刚算过的距离说了算;拿着离远 = 沿原路退
      --  🔴 走路用哪只眼,由【要动的那条胳膊】定(开机量的 Cam_On_Arm),不由脑此刻看着哪只眼定。
      --  H27 2026-09-22 实测:Qwen 每一段都写 with my still eye,脑于是留在头顶眼里;走路也跟着落到头顶眼 ⇒ 掉回逐通道推的老路,
      --  而那只眼量不了远近 ⇒ 691 拍一步没走、1200 拍超时。脑点的眼是它【看】的眼;量远近、贴上去,只有长在这条胳膊上的眼做得到。
      --  走路的眼未必是脑看着的眼 ⇒ 它里面未必有槽,东西按【名字】跟(脑在那只眼里点过一次名,之后每帧我自己重量)。
      if (Say.Grip = "none" or else Length (Say.Grip) = 0) and then Natural (Say.Moves.Length) = 1 then
         declare
            G0 : constant Brain.Goal := Say.Moves (0);
         begin
            if G0.Item >= 1 and then G0.Item <= Natural (C.Items.Length) and then C.Items (G0.Item - 1).Kind in Finger | Grip then
               Own := Integer (C.Items (G0.Item - 1).Arm);
            end if;
         end;
      elsif Say.Grip = "close" then
         Own := Grip_Arm;
      elsif Length (Say.Qty) > 0 then
         Own := Grip_Arm;   --  东西的量要变、我已经拿着它 ⇒ 动的是拿着它的那只手
      end if;
      Geo_Cam := Hand_Eye_Of (C, Own);
      declare
      begin
         if Own >= 0 and then Geo_Cam >= 0 then
            --  🔴 T8 2026-09-21 实测这一支没进来:程序一节翻成内部请求时,纯移动的 Grip 是【空串】(只有 close/open 才赋值),
            --  而 GB5 那个年代脑填表、填的是 "none"。两种都是"这一节不动爪子"。
            if (Say.Grip = "none" or else Length (Say.Grip) = 0) and then Natural (Say.Moves.Length) = 1 then
               declare
                  G0 : constant Brain.Goal := Say.Moves (0);
                  Rl : constant String := To_String (G0.Rel);
               begin
                  --  nearer(front)也是朝它走:同一条视线走法,走到脑说的步数/到位/碰到为止(H28 2026-09-22 实测:Qwen 写 nearer,
                  --  掉回老路,逐通道量表 60 拍一步没走)。farther(back)手里没东西时 = 沿来时的方向反着走(Geo_Away)。
                  if (Rl = "at" or else Rl = "into" or else Rl = "onto" or else Rl = "above" or else Rl = "front")
                    and then G0.Of_Item >= 1 and then G0.Of_Item <= Natural (C.Items.Length)
                    and then C.Items (G0.Of_Item - 1).Kind in Thing | Thing_Remembered and then C.Items (G0.Of_Item - 1).Located
                  then
                     --  🔴 above 在【长在手上的眼】里按"画面里的上方"没法执行也没有物理意义(那只眼跟着手转);
                     --  这里按重力的"上"走:它正上方、高出一个张口。T5–T7 实测 Qwen 的抓法每一段都从 above 起手。
                     --  ⚠️ 这是对语言 §4.1 的一处改义(只在手上的眼里),已记进 LAB,待 owner 认。
                     Geo_Above := Rl = "above";
                     Geo_Case := 1;
                     Geo_Slot_Now := (if Natural (Geo_Cam) = Cam then C.Items (G0.Of_Item - 1).Slot else -1);
                     Geo_Name := Item_Name (C, G0.Of_Item);
                     Geo_Desc := S (Say_Item (C, G0.Item) & " " & Rl & " " & Say_Item (C, G0.Of_Item) & " (by sightlines, in my own hand camera)");
                  elsif Rl = "back" and then C.Wld.Holding and then C.Wld.Held_Arm = Own then
                     Geo_Case := 2;
                     Geo_Desc := S (Say_Item (C, G0.Item) & " back the way it came, holding");
                  elsif Rl = "back" and then G0.Of_Item >= 1 and then G0.Of_Item <= Natural (C.Items.Length) then
                     Geo_Case := 4;
                     Geo_Desc := S (Say_Item (C, G0.Item) & " away from " & Say_Item (C, G0.Of_Item) & " along the line I came in on");
                  end if;
               end;
            elsif Say.Grip = "close" and then Grip_Arm = Own then
               --  🔴 "刚算的距离还作不作数"不按轮数判(H11 2026-09-22 实测:main 上每段程序跑完多一轮记账,close 落在两轮之后,
               --  条件 ≤ 1 轮不成立 ⇒ 掉回老路,手正压在剪刀上却开始逐通道推着量响应表)。
               --  该问的是:算完之后我的手挪开过没有。没挪开(不超过一个探针幅度 —— 身体量过的最小一档)就还作数。
               declare
                  Fresh : Boolean := False;
               begin
                  if C.Geo_Dist >= 0.0 and then C.Geo_At_Arm = Own then
                     declare
                        Now : constant Plug.Arm_Pose := F.EE (Natural (Own));
                        Moved : constant Long_Float :=
                          Geom.Norm ([Now (0) - C.Geo_At (0), Now (1) - C.Geo_At (1), Now (2) - C.Geo_At (2)]);
                     begin
                        --  一个量距单位之内(4 倍探针幅度,倍数,无量纲;最后那一步没走成的也在这之内);
                        --  而且那个距离得是到它身上的 —— 到它【上方】的距离再小也不算笼住(H35 2026-09-22 实测:above 到 0.007 m 就合,合的是 9 cm 空气)
                        Fresh := Moved <= 4.0 * Geo_Base (C, Natural (Own)) and then not C.Geo_At_Above;
                     end;
                  end if;
                  if Fresh and then Length (Say.Qty) = 0 then
                     Geo_Case := 3;   --  合:不再先走一段
                  elsif Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
                    and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered and then C.Items (Say.Grip_On - 1).Located
                  then
                     --  🔴 "合在 X 上"= 先把合拢点送到 X 身上(从它躺的面的上方进场),再合。H34 2026-09-22 实测:Qwen 的计划永远是
                     --  above → close,到了上方就合 ⇒ 合的是 9 cm 高的空气。八月 §1.2:抓 = 相对的两点向内使劲、物体跟着手走 ——
                     --  进场是抓的一部分,不是另一个词。脑说的是"合在它上",身体就把合拢点送到它上再合。
                     Geo_Case := 5;
                     Geo_Name := Item_Name (C, Say.Grip_On);
                     Geo_Slot_Now := (if Natural (Geo_Cam) = Cam then C.Items (Say.Grip_On - 1).Slot else -1);
                     Geo_Desc := S ("grip " & Codec.Img (Natural (Own) + 1) & " onto " & Say_Item (C, Say.Grip_On)
                                    & " before closing (by sightlines, in my own hand camera)");
                  end if;
               end;
            elsif Length (Say.Qty) > 0 and then C.Wld.Holding and then C.Wld.Held_Arm = Own then
               Geo_Case := 7;   --  拿着它了,这一段只改它的量(抬),不走
            end if;
         end if;
      end;
      --  这条臂的眼在手上怎么装的开机没量出 ⇒ 用不了这只眼走路、合手,照实说(一种量法:开机量;09-30 删了"第一次用它时当场挪四下量"那条后备)
      if Geo_Case in 1 | 5 and then not Geo_Of (C, Natural (Geo_Cam)).Valid then
         Report := S ("how my hand camera sits on my hand was not measured at boot, so I cannot walk or close by it. ");
         if Geo_Case = 5 then
            Say.Grip := Null_Unbounded_String;   --  眼都量不出,合拢点送不到它身上,合了也是空
         end if;
         Geo_Case := 0;
      end if;
      if Geo_Case = 1 then
         Put_Line ("[身] ⚙ 几何走法:" & To_String (Geo_Desc)
                   & (if Natural (Geo_Cam) /= Cam then "(你看着第" & Codec.Img (Cam) & " 只眼,走路用长在这条胳膊上的第" & Codec.Img (Natural (Geo_Cam)) & " 只)" else ""));
         if Geo_Above then
            Geo_Approach (L, C, F, Natural (Geo_Cam), Natural (Own), Geo_Slot_Now, Step_Limit, Event, Steps_Taken, Beats,
                          Above => True, Amt => Amount_Factor (Say.Moves (0).Amount),
                          Until_Touch => Until_K in Monitor.U_Contact | Monitor.U_Resist,
                          Name => Geo_Name);
         else
            --  走到它跟前(不是抓):视线走法直接往它身上走。抓的进场(上方 → 指尖朝下 → 贴上)归接触集那条路(Contact_Onto),不在这儿
            Geo_Approach (L, C, F, Natural (Geo_Cam), Natural (Own), Geo_Slot_Now, Step_Limit, Event, Steps_Taken, Beats,
                          Above => False, Amt => Amount_Factor (Say.Moves (0).Amount),
                          Until_Touch => Until_K in Monitor.U_Contact | Monitor.U_Resist,
                          Name => Geo_Name);
         end if;
         Feel (C, F);
         Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
         Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
         Codec.Append_Line (Life_Path, "beat " & Codec.Img (Plug.Steps (L)) & " | eye " & Codec.Img (Natural (Geo_Cam)) & " | " & To_String (Geo_Desc)
                            & " | " & Codec.Img (Steps_Taken) & " pushes | ended: " & To_String (Event));
      elsif Geo_Case = 2 then
         Put_Line ("[身] ⚙ 几何走法:" & To_String (Geo_Desc));
         Geo_Retreat (L, C, F, Natural (Own), Event, Steps_Taken, Beats);
         Feel (C, F);
         Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
         Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍");
         Codec.Append_Line (Life_Path, "beat " & Codec.Img (Plug.Steps (L)) & " | eye " & Codec.Img (Cam) & " | " & To_String (Geo_Desc)
                            & " | " & Codec.Img (Steps_Taken) & " pushes | ended: " & To_String (Event));
      elsif Geo_Case = 4 then
         Put_Line ("[身] ⚙ 几何走法:" & To_String (Geo_Desc));
         Geo_Away (L, C, F, Natural (Own), Step_Limit, Amount_Factor (Say.Moves (0).Amount), Event, Steps_Taken, Beats);
         Feel (C, F);
         Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
         Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍");
         Codec.Append_Line (Life_Path, "beat " & Codec.Img (Plug.Steps (L)) & " | eye " & Codec.Img (Natural (Geo_Cam)) & " | " & To_String (Geo_Desc)
                            & " | " & Codec.Img (Steps_Taken) & " pushes | ended: " & To_String (Event));
      elsif Geo_Case = 5 then
         Put_Line ("[身] ⚙ 几何走法:" & To_String (Geo_Desc)
                   & (if Natural (Geo_Cam) /= Cam then "(你看着第" & Codec.Img (Cam) & " 只眼,走路用长在这条胳膊上的第" & Codec.Img (Natural (Geo_Cam)) & " 只)" else ""));
         Contact_Onto (Amount_Factor (Null_Unbounded_String));
         Feel (C, F);
         Report := Report & "before closing I brought my fingers onto " & Say_Item (C, Say.Grip_On) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
         --  没到它身上(看丢 / 顶住 / 上方都没到)就不合:合上的是空气,白花四十拍,还要试抬一次(H48 2026-09-23 实测)。
         --  这不是我自己收工:脑要的是合在它上,我到不了它上,如实说,合这一下就没有意义。
         if Index (Event, "amount: arrived") = 0 and then Index (Event, "contact") = 0 then
            Say.Grip := Null_Unbounded_String;
            Report := Report & "I did not close: my fingers are not on it. ";
            Put_Line ("[身]   没到它身上 ⇒ 这回不合");
         end if;
         Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
         Codec.Append_Line (Life_Path, "beat " & Codec.Img (Plug.Steps (L)) & " | eye " & Codec.Img (Natural (Geo_Cam)) & " | " & To_String (Geo_Desc)
                            & " | " & Codec.Img (Steps_Taken) & " pushes | ended: " & To_String (Event));
      elsif Geo_Case = 3 then
         Put_Line ("[身] ⚙ 几何走法:合手前不再走,笼住与否由刚算的 " & Mm (C.Geo_Dist) & " 说");
      elsif Geo_Case = 7 then
         Put_Line ("[身] ⚙ 几何走法:它已经在我手里,这一段只改它的量");
      else
      Build_Goals;
      if not Pts.Is_Empty then
         Expand_Lobes (C, F, Cam, Pts);
         --  生地先看一眼:我的手/零件在这台相机里的位置若只是按关节推的(没真看过),先抖/推一下认清,再按认清的位置重算各团目标,再量表、再走
         --  (EI:按关节推的手指位置差 0.2 画幅,在错地方量表 ⇒ 六列全空 ⇒ 一步没走)
         declare
            Need_Look : Boolean := False;
         begin
            for P of Pts loop
               if P.Kind = Piece_Pt and then Cam_Arm (C, Cam) /= Integer (P.Arm) and then not P.Known then
                  Need_Look := True;
               end if;
               --  🔴🔴 还有一种"生地":位置【看着像知道】,但它自相矛盾。
               --  同一个爪的两瓣在画面里应该只隔【量到的钳口张幅】那么远;
               --  GW 实测 arm 2(右臂)的两瓣被放到画面左边、相隔四分之三个画面 —— 不可能都对。
               --  这时候位置是错的而 Known 却是真的 ⇒ 不会触发"先看一眼" ⇒ 拿错位置算误差 ⇒
               --  往错的方向推 ⇒ 十炮里七炮"靠近→停在错的稳定点→退开"。
               --  这不是给身体加闸(它照样动),是让它动之前先看清自己 —— 那条路本来就有。
               if P.Kind = Piece_Pt and then Cam_Arm (C, Cam) /= Integer (P.Arm) then
                  declare
                     Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
                  begin
                     if Z.Valid and then Z.A.Valid and then Z.B.Valid and then Z.Span > 0.0
                       and then Sqrt ((Z.A.Cu - Z.B.Cu) ** 2 + (Z.A.Cv - Z.B.Cv) ** 2) > Z.Span + Z.Span
                     then
                        Need_Look := True;
                     end if;
                  end;
               end if;
            end loop;
            if Need_Look then
               Put_Line ("[身] 生地:我在这台相机里对自己位置没把握(按关节推的,或者两瓣间距和量到的钳口张幅对不上)⇒ 先动一下认清自己再走");
               Refind_Pieces (L, C, F, Cam, Pts);
               Feel (C, F);
               declare
                  Su, Sv, N : Long_Float := 0.0;
               begin
                  for P of Pts loop
                     if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                        Su := Su + P.Cu; Sv := Sv + P.Cv; N := N + 1.0;
                     end if;
                  end loop;
                  if N > 0.0 then
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        declare
                           P : Point := Pts (I);
                        begin
                           if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                              P.Tu := P.Par_Tu + (P.Cu - Su / N); P.Tv := P.Par_Tv + (P.Cv - Sv / N);
                              Pts.Replace_Element (I, P);
                           end if;
                        end;
                     end loop;
                  end if;
               end;
               declare
                  Lost_N : Natural := 0;
               begin
                  for P of Pts loop
                     if P.Lost then
                        Lost_N := Lost_N + 1;
                     end if;
                  end loop;
                  --  🔴 认不出自己【不是停下的理由】(owner 死命令:能让身体停的只有人的命令和脑写的 until,
                  --  "我做不到"都不算)。以前这里 Pts.Clear ⇒ 整段一推不走:HC 实测【连着四段零推】,
                  --  45 推的那一段一个 步 都没有,三段全 timeout,而胳膊是自由的、球就在画面里。
                  --  而且判据是"任一个点跟丢"就全清 —— 和探针那条一票否决同一类错。
                  --  改成:位置用身体图按此刻关节推出来的那一份(Feel 刚算过),说出来,照走。
                  if Lost_N > 0 then
                     Report := Report & "I moved my own piece to find it in this picture and could not see "
                               & Codec.Img (Lost_N) & " of " & Codec.Img (Natural (Pts.Length))
                               & " of the points I am tracking; I am going on where my body map says they are, "
                               & "and I am telling you rather than holding still. ";
                     C.Blind_Say := S ("I could not see my own piece after moving it, so I am going on the guess "
                                       & "my body map gives for it - I am moving, not holding still");
                  end if;
               end;
            end if;
         end;
         for P of Pts loop
            if P.Desc /= "" then
               Append (Desc, (if Desc = "" then "" else " and ") & To_String (P.Desc));
            end if;
         end loop;
         if not Pts.Is_Empty then
            Put_Line ("[身] ⚙ 一起解" & Natural'Image (Natural (Pts.Length)) & " 条:" & To_String (Desc));
            --  🔴 把【目标在哪、我在哪、每根通道推正一点画面往哪跑】原样打出来。
            --  HO 实测:手一路往右飘到画面最右沿(u 0.926→0.998),而球在它左边 —— 不知道是
            --  目标算错了还是表的符号反了,光看"差 x m / 还差 N 步"分不出来。打出来就分得出。
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : constant Point := Pts (I);
                  Ix : constant Integer := Find_Effect (C, P.Arm, P.Cam, P.Kind, P.Chan_K, P.Blob);
                  Ln : Unbounded_String;
               begin
                  Ln := S ("[身]   点" & Codec.Img (I) & "(相机" & Codec.Img (P.Cam) & "):我在 ("
                           & Codec.Fmt (P.Cu, 3) & "," & Codec.Fmt (P.Cv, 3) & ") 深 " & Codec.Fmt (P.Z, 3)
                           & " · 目标 (" & Codec.Fmt (P.Tu, 3) & "," & Codec.Fmt (P.Tv, 3) & ") 深 "
                           & Codec.Fmt (P.Tz, 3)
                           & " · 目标画面坐标量在 " & Codec.Fmt (P.Tuv_Z, 3)
                           & " · 搬到我这个远近后该去 " & Codec.Fmt (On_My_Plane (P.Tu, P.Tuv_Z, P.Z), 3)
                           & " ⇒ 左右要走 " & Codec.Fmt (On_My_Plane (P.Tu, P.Tuv_Z, P.Z) - P.Cu, 3) & " 画幅");
                  Put_Line (To_String (Ln));
                  if Ix >= 0 then
                     Ln := S ("[身]   点" & Codec.Img (I) & " 表(推 +1 画面往哪跑,左右那一行):");
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        Append (Ln, " ch" & Codec.Img (P.Arm * Chan.Per_Arm + K) & "="
                                & Codec.Fmt (C.Tables (Natural (Ix)).E.B (K, 0), 3));
                     end loop;
                     Put_Line (To_String (Ln));
                     --  远近那一行:推 +1 我离相机远近变多少(正 = 变远)。方向对不对全看它的正负。
                     Ln := S ("[身]   点" & Codec.Img (I) & " 表(推 +1 远近变多少,正=变远):");
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        Append (Ln, " ch" & Codec.Img (P.Arm * Chan.Per_Arm + K) & "="
                                & Codec.Fmt (C.Tables (Natural (Ix)).E.B (K, 2), 3)
                                & (if C.Tables (Natural (Ix)).Trust (K) then "" else "(没证过)"));
                     end loop;
                     Put_Line (To_String (Ln));
                  end if;
               end;
            end loop;
            Run_Segment (L, C, F, Cam, Pts, Until_K, Step_Limit, Amount, Avoid, Event, Steps_Taken, Blocked, Beats);
            --  🔴 脑说过"这只眼的这几个框里没有它"⇒ 那只眼被跳过(见 Blind_Cam)。
            --  可那句话只对【当下这一帧的切块】成立:JF 2026-09-16 实测,腕眼里球
            --  【看得见但没被切成块】(被两根手指从中间劈开),我照实答 0,结果那只眼整集被判死,
            --  而尺子只有长在手上的眼能用 ⇒ 这一集再也量不了远近。
            --  "这几个框里没有它" ≠ "这只眼看不见它" —— 我把两件事混成了一件。
            --  🔴 清标记的条件不是"我动了",是"**那只眼跟着我动了**"(JG 2026-09-16 实测)。
            --  第一版写成"走过步就清",于是每跑完一段、我再要"不跟着我动的那只眼",
            --  它又被挑回第 1 只 —— 而第 1 只长在【另一条】胳膊上,我动这条它的画面一帧都不会变,
            --  切块一模一样,答案必然还是 0。白弹三个来回(每回两分半)。
            --  身体早就量过哪只眼长在哪条胳膊上(Cam_On_Arm)⇒ 直接用:
            --    长在我正动的这条胳膊上 ⇒ 画面确实变了 ⇒ 清掉重新问;
            --    长在别处 ⇒ 我动它不变 ⇒ 标记留着,别再弹过去。
            if Steps_Taken > 0 then
               C.Last_Moved := True;   --  这一段真的让身体走过步
            end if;
            if Steps_Taken > 0
              and then not Pts.Is_Empty
              and then Pts (0).Arm < Natural (C.Map.Cam_On_Arm.Length)
              and then C.Map.Cam_On_Arm (Pts (0).Arm) >= 0
            then
               Clear_Blind (C, Natural (C.Map.Cam_On_Arm (Pts (0).Arm)));
            end if;
            C.Last_Outcome := Classify (To_String (Event));
            Feel (C, F);
            Report := Report & "you asked " & Desc & ": " & Event & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
            --  🔴 一段【一推都没走】绝不许看起来正常:除非脑写的 until 在第 0 步就成立,否则这是身体自己没动。
            --  HC 实测连着四段零推、全部报 timeout,读日志像一切正常。喊出来,让脑看得见。
            if Steps_Taken = 0 then
               Report := Report & "*** I took ZERO pushes in that stretch - nothing on me moved at all. "
                         & "Unless your until was already true before I started, that is me failing to move, "
                         & "not the task being done. ";
               Put_Line ("[身]   🔴 这一段一推都没走 —— 除非脑的 until 在第 0 步就成立,这就是身体自己没动");
            end if;
            Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
            --  🔴 经历账:这一段我干了什么、成没成。跨炮留着 —— 这是"它记得自己昨天"的全部物质基础。
            --  一行一条纯文本:坏一行不毁整份(身体文件里一个 NaN 就整份读不回来,那个坑不许重犯)。
            Codec.Append_Line (Life_Path,
                               "beat " & Codec.Img (Plug.Steps (L))
                               & " | eye " & Codec.Img (Cam)
                               & " | " & To_String (Desc)
                               & " | " & Codec.Img (Steps_Taken) & " pushes"
                               & " | ended: " & To_String (Event));
            for P of Pts loop
               if P.Blob <= 0 then
                  Report := Report & Say_Item (C, P.Item_No) & (if P.Blob = 0 then " (finger A)" else "") & " now at (" & Codec.Fmt (P.Cu, 2) & "," & Codec.Fmt (P.Cv, 2) &
                            ") depth " & Codec.Fmt (P.Z, 2)
                            & (if P.No_Scale and then P.Steps_Err <= 0.0
                               then ", and I do not know how many pushes away it is: I have not yet measured "
                                    & "what one push changes here, so I cannot turn the gap into pushes - "
                                    & "this is NOT me saying I have arrived; "
                               else ", still " & Codec.Fmt (P.Steps_Err, 1) & " pushes away; ");
               end if;
            end loop;
         else
            Event := S ("lost: could not see my own piece after moving it");
         end if;
      elsif Say.Moves.Is_Empty and then (Say.Grip = "none" or else Length (Say.Grip) = 0) then
         Report := S ((if Say.See = "not_here" then "you said the thing is not in that picture; the body did not move. "
                       elsif Say.See = "unclear" then "you said you could not tell; the body did not move. "
                       else "you gave no move and no grip; the body did not move. "));
      end if;
      end if;   --  Geo_Case
      Do_Grip;
      --  语言的根:脑说的是"它的某个量往哪变"。合完(或本来就拿着)⇒ 沿让那个量变的方向走一个单位;没拿住就说没拿住,不走。哪个量都是这一条
      if Length (Say.Qty) > 0 and then Say.Qty_Dir /= 0 then
         declare
            Qn : constant String := To_String (Say.Qty);
            Ax : Geom.V3 := Qty_Axis (C, Qn);
            Dir_Word : constant String := (if Say.Qty_Dir > 0 then " up" else " down");
         begin
            if Geom.Norm (Ax) <= 0.0 then
               Event := S ("refused: " & Qn & " is not a quantity I measure on it");
               Report := Report & " I do not measure a quantity called " & Qn & ". ";
            elsif Own >= 0 and then C.Wld.Holding and then C.Wld.Held_Arm = Own then
               if Say.Qty_Dir < 0 then
                  Ax := [-Ax (0), -Ax (1), -Ax (2)];
               end if;
               Change_Held_Qty (Natural (Own), Amount_Factor (Null_Unbounded_String), Ax, Qn);
               Put_Line ("[身] ⚙ 改它的量(" & Qn & Dir_Word & "):" & To_String (Event));
               Report := Report & " Then, holding it, I changed its " & Qn & Dir_Word & ": " & To_String (Event) & ". ";
               Codec.Append_Line (Life_Path, "beat " & Codec.Img (Plug.Steps (L)) & " | " & Say_Item (C, Say.Qty_Of) & " " & Qn & Dir_Word & " | " & To_String (Event));
            else
               Event := S ("lost: I could not change the " & Qn & " of " & Say_Item (C, Say.Qty_Of) & " - it is not in my hand");
               Report := Report & " I did not change its " & Qn & ": it is not in my hand. ";
            end if;
         end;
      end if;
      Report := Report & Mode_Line (C, To_String (Event));
   end;
   --  这一节的结果攒进这一段程序的账上;跑完一整段才一次交给脑
   --  身体照走了但有话要说的,一并交给脑(不是停,是说)
   if Length (C.Blind_Say) > 0 then
      Report := Report & " " & To_String (C.Blind_Say);
      C.Blind_Say := Null_Unbounded_String;
   end if;
   --  把这一节的结局喂回执行器 —— 控制流只认这八个词
   if C.Have_Prog then
      declare
         O : constant Sinew.Outcome := C.Last_Outcome;
      begin
         Put_Line ("[身]   结局 = " & Sinew.Outcome_Word (O) & "(" & Sinew.Outcome_Cn (O) & ")");
         Runtime.Report (C.Prog, C.M, O);
      end;
   end if;
   Append (C.Prog_Log, (if Length (C.Prog_Log) > 0 then ASCII.LF & "" else "") & To_String (Report));
   C.Recent := Report;
   Put_Line ("[身]   ⇒ " & To_String (Report));
end Round;
