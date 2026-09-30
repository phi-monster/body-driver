separate (Act)
procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                       Until_Kind : Monitor.Until_Kind;
                       Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                       Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean; Beats : out Natural) is
   --  Pts 空的时候 `Pts (0)` 当场越界,而它在声明区 ⇒ 异常记在【调用处】,
   --  栈里根本看不到这个子程序这一帧(实测查了半天)。空就当第 0 条胳膊,下面第一句直接回。
   Pts_Empty : constant Boolean := Natural (Pts.Length) = 0;
   Arm : constant Natural := (if Pts_Empty then 0 else Pts (0).Arm);
   --  🔴 不是 constant:线一断重连,仿真那边的帧计数从头开始 ⇒ 现在的拍数会【小于】开工时的拍数。
   --  以前这里两个 Natural 直接相减,负数当场 CONSTRAINT_ERROR 把整炮打死
   --  (GM 崩在 act.adb:1518,崩之前日志里 [链] 线断了/重新接上了 刷了几十遍)。
   --  这一段开始时,被跟的那块鼓出背景多少米。"离开了原来靠着的面"就是拿它和此刻比
   --  🔴🔴 命令整体放大多少倍。上一步点在画面里没跑过跟踪地板 ⇒ 翻倍;跑过了 ⇒ 复位。
   --  这一条治的是 GN–GR 四炮的共同终局:表【高估】了"推一单位点跑多远" ⇒ 解算算出 0.002 rad
   --  就够了 ⇒ 推下去一动不动 ⇒ 点没动就没有新信息去纠正表 ⇒ 永远循环。
   --  已有的放大只把命令抬到【关节】的噪声地板(EE_Noise),抬不到"画面里看得出动过"。
   --  翻倍这条和开机探针"翻倍到点真的动过地板为止"是同一条规矩,不是新拍的系数。
   Push_Mult : Long_Float := 1.0;
   H0 : constant Long_Float := (if Natural (Pts.Length) > 0 then Pts (0).Height else 0.0);
   Beats0 : Natural := Plug.Steps (L);
   --  开工到现在过了几拍。倒退 = 对面重连过 ⇒ 把起点挪到现在,从这儿重新数,别炸
   function Since (Lk : Plug.Link; Start : in out Natural) return Natural is
      Now : constant Natural := Plug.Steps (Lk);
   begin
      if Now < Start then
         Start := Now;
         return 0;
      end if;
      return Now - Start;
   end Since;
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (Arm);
   Effs : Effect_Array (0 .. Natural (Pts.Length) - 1);
   Trusts : array (0 .. Natural (Pts.Length) - 1) of Table.Mask := [others => [others => True]];
   Reach : Table.Vec := Unit_Reach;   --  每通道核实过的步幅倍数(存表里,越用越强)
   Blocked_Run : Natural := 0;        --  连着几步"零表更准":身体自己说这张地图不如"什么都不会发生"准
   Fl : Monitor.Floors;
   W : Monitor.Watch;
   Ring : Backup.Ring;
   Jaw : Floats;
   Last_Err : Long_Float := -1.0;     --  -1 = 还没算过(哨兵,无量纲)
   Last_Raw : Long_Float := -1.0;     --  上一步不随表变的差距
   Best_Raw : Long_Float := -1.0;     --  到目前为止最好的一次(判"有没有在靠近"和它比,不和上一步比 —— 噪声一晃就成退步)
   Trust : Long_Float := 0.5;         --  这张表有多准,就走它算出来的多大比例(半开始;预测差一半就只走一半,准了再放开)
   Lost_Run : Natural := 0;           --  连着几步全部认不到

   --  这一步的账
   type Note_Rec is record
      Cmd, Got, Cap : Table.Vec := Table.Zero_Vec;   --  要走的 / 实际走的 / 各通道这一步的上限
      Active : Table.Mask := [others => False];
      Floor_Cmd : Long_Float := 0.0;                 --  最小探针幅度:比它一半还小的命令说明不了"顶住"
      Err_Now : Long_Float := 0.0;                   --  这一步之后还差几步(尺度会变,只用来说话)
      Raw_Now : Long_Float := 0.0;                   --  这一步之后不随表变的差距(判有没有在靠近就看它)
      Pic_Delta : Long_Float := 0.0;
      Big_Step : Boolean := False;                   --  大到光流跟不住 ⇒ 走完抖一下认自己
      Halted : Boolean := False;                     --  途中眼睛叫停
      Blocked : Boolean := False;                    --  顶住了(零表更准)
      Not_Followed : Boolean := False;               --  整步没照做
      Touched : Boolean := False;                    --  我没在推的东西也动了 = 碰到
      Lost_All : Boolean := False;                   --  被跟的全都认不到
      Unsure : Boolean := False;                     --  两块一样像,不许自己挑
      Say_Stop : Unbounded_String;                   --  非空 = 这一段到此为止,内容就是给脑的话
   end record;
   Note : Note_Rec;
   --  🔴 一段动作【同时用所有相机】(owner 死命令)。以前整段只在一台相机里解:
   --  "我的手在哪、目标在哪、差多少、推哪几根"全在单只眼里算 ⇒ 必须选一只眼 ⇒
   --  选中了唯一看不见自己手的那只(不动的那台,开合扫出来的是噪声,填充率 0.015)⇒ 手的位置是编的 ⇒ 全线中毒。
   --  FO 没这个病只是因为当年只有一只眼可选,而那只正好是看得见手的腕相机 —— 运气,不是设计。
   --  现在每个点带着自己那台相机(Point.Cam),走之前的灰度也每台各存一份。
   Before_All : Buf_Vectors.Vector;   --  走之前那一拍的灰度,每台相机各一份(光流用)
   Was : Point_Vectors.Vector;        --  走之前各点在哪
   --  🔴 走之前【我自己】在哪(米,关节读数给的)。尺子的另一半:
   --  没有"我真挪了多少米"就没有距离,只有一堆画幅。
   Was_EE : Plug.Arm_Pose := [others => 0.0];
   Was_Regs : Picture.Regions;        --  走之前世界里各块在哪

   --  走的途中每一拍看一眼:被跟的东西还找得到吗、离画面边够不够远(转一点点就该知道不对劲)
   function Watch_Things (Fr : Plug.Frame) return Boolean is separate;

   --  落一张图,把这一拍切出来的块和被跟的点画上去(出岔子时给人看)
   procedure Dump_Picture (Tag : String) is separate;

   --  段前:每个点的响应表 —— 装回(必须是同一位姿、同样拿着东西、至少有一列信得过)或当场重量
   procedure Ready_Tables (Ok_Out : out Boolean) is separate;

   --  ①a 定目标:每个点的五样差距,各自除以"推一步最多能改多少",变成"还差几步"
   --  🔴 这一段里,哪几个点的"远近"那一行是【米】(尺子量出来的),不是深度读数
   In_Metres : array (0 .. Natural'Max (0, Natural (Pts.Length) - 1)) of Boolean := [others => False];
   --  这一个点的五行里,有没有【这一段要管、却算不出"还差几步"】的行(见 Point.No_Scale)
   No_Scale_Row : Boolean := False;

   procedure Aim (Terms : out Table.Term_Vectors.Vector) is separate;

   --  ①b 定额度:各通道这一步最多走多少 —— 两条取小:①眼睛跟得住的那么多(表说走多少画面跑满一个跟踪窗)
   --  ②自己那一档 × 核实过的倍数。只用①,在画面里几乎不动的通道会拿到无限额度(甩腕 2 rad)
   procedure Budget (Terms : Table.Term_Vectors.Vector; Solved : out Boolean) is separate;

   --  ①c 修步子:缩到眼睛跟得住,且不把被跟的东西推出视野、不让我身上任何一块压到"不许碰"的框
   procedure Trim is separate;

   --  ① 打算怎么走 = 定目标 → 定额度 → 修步子
   procedure Plan is separate;

   --  ② 走:记下走之前的样子,发命令,途中盯着
   procedure Walk (Ok_Out : out Boolean) is separate;

   --  ③ 看:每个点现在在哪。我的零件(别人的相机里)先按位姿"感觉",熟地让眼睛核对,生地或大步就抖一下去看;
   --  世界里的块每步重切就近对上。全都认不到才叫看不见。
   procedure Look is separate;

   --  ④ 学:拿这一步的实际结果修表;判"整步没照做";按通道各自放宽/收紧步幅;看有没有碰到别的东西
   procedure Learn is separate;

   --  ⑤ 判:这一步之后接着走,还是到了 / 出事了 / 拿不准
   --  🔴 差多少,报【厘米】。没有相机内参,也不需要:响应表平移那三列存的就是
   --  "这条通道推一个单位,这一点在画面里跑多少" —— 而平移通道的单位就是米(位姿前三位是平移)。
   --  拿它当折算率,画幅 ÷ (画幅/米) = 米。全是量出来的,零假设。
   --  不加这个,读数只有归一化的"差距 0.262",没法和历史炮的厘米数摆在一起比(本仓规矩:相对量必须配绝对量)。
   function Cm_Gap (E : Table.Effect; P : Point) return String is
      Du : constant Table.Vec3 := Table.Col (E, 0);
      Dv : constant Table.Vec3 := Table.Col (E, 1);
      Dz : constant Table.Vec3 := Table.Col (E, 2);
      --  这三条平移通道各自"推一米画面跑多少":取它在自己主方向上的那一项
      Su : constant Long_Float := abs Du (0) + abs Dv (0) + abs Dz (0);
      Sv : constant Long_Float := abs Du (1) + abs Dv (1) + abs Dz (1);
      Eu : constant Long_Float := P.Tu - P.Cu;
      Ev : constant Long_Float := P.Tv - P.Cv;
      Ez : constant Long_Float := (if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz)
                                   then P.Tz - P.Z else 0.0);
      Mu : constant Long_Float := (if Su > 1.0e-9 then Eu / Su else 0.0);
      Mv : constant Long_Float := (if Sv > 1.0e-9 then Ev / Sv else 0.0);
   begin
      --  报世界单位(④ 09-27:原来印"m";只读关节以后世界单位是运动学的单位,不是米)
      if Su <= 1.0e-9 and then Sv <= 1.0e-9 then
         return "左右上下折不出长度(平移三列还没量到);远近 " & Mm (Ez);
      end if;
      return Mm (Sqrt (Mu * Mu + Mv * Mv + Ez * Ez))
        & "(左右 " & Codec.Fmt (Mu, 3) & " 上下 " & Codec.Fmt (Mv, 3)
        & " 远近 " & Codec.Fmt (Ez, 3) & ")";
   end Cm_Gap;

   procedure Judge is separate;

   Ok : Boolean;
   --  🔴 上一次量远近时的差距。只在【比上次量的时候更近了】才再量一遍 ——
   --  量一次要把另一条胳膊甩出去再收回来,不该每步都甩;而画面已经对齐之后,
   --  剩下的差距就只可能是前后,这时候才值得再量。零系数:两个都是量出来的差距。
   Ranged_Raw : Long_Float := Long_Float'Last;
   --  🔴 量距离用的那一下:第一次量什么,以后就一直拨同样的一下 ——
   --  两次相除时,横向那一份才会约掉(不同的拨法之间没法比)。
   Probe_K : Integer := -1;
   Probe_Amp : Long_Float := 0.0;
   Probe_EE : Plug.Arm_Pose := [others => 0.0];
   --  🔴 参照那一拨【在世界里往哪儿推了、推了多远】。下一拨方向不一样就不能比:
   --  滑速里含着"我横着挪了多少",方向一变它就变,而那跟远近无关(IC 实测 0.014 m 假距离的真凶)。
   Probe_Dir : Xyz := [others => 0.0];
   Probe_Len : Long_Float := 0.0;
   Probe_Have : Boolean := False;
   --  🔴🔴 量距离:轻轻拨一下,看它滑多远;走一段,再拨【同样的一下】(2026-09-15)。
   --  这是"拿自己的胳膊当尺子"真正该干的那一半 —— 以前只量出"我的距离感放大了三十二倍"
   --  然后把这句话打印出来,从来没拿它量过任何一个距离,于是"远近"那一栏常年 0.0,
   --  身体把左右上下对得极准而人和球之间一步没缩(FP–GS 十八炮、HC–IA 二十四炮 0 握,同一个死因)。
   --
   --  🔴 撤回(owner 当场指出):我第一版写的是"甩【另一条胳膊】,比谁滑得快"——
   --  那是把这台机器人当成了所有机体。一条胳膊的机器没有"另一条",无人机连胳膊都没有。
   --  量距离要的从来不是第二条胳膊,只有三件:**眼睛跟着我动 · 我知道自己动了多远 · 我能把同一下拨两遍**。
   --
   --  几何(零系数,焦距/基线/深度尺度全部约掉):
   --    同一下拨动,在离我 Z 的东西上滑过 S ∝ 1/Z
   --    走近一段 D 之后再拨同样一下,滑 S₂ ∝ 1/(Z−D)
   --    ⇒ 此刻它离我 = D × S₁ ÷ (S₂ − S₁)     ← 单位就是 D 的单位(米),不需要知道焦距
   --  ⚠️ D 是我【总共走了多远】,而公式要的是【朝它走了多远】。两者相等只有在我一直朝它走时成立;
   --     我要是还横着挪了,真实距离比算出来的更近 —— 这一句必须照实说出去,不许假装是纯几何。
   --  ⚠️ S₂ − S₁ 没过跟踪抖动 ⇒ 这一段我没有真的走近过 ⇒ **说我量不出来**,
   --     绝不给一个看起来正常的烂数(记录 2026-08-16:三角形太扁时误差 ∝ 距离²/基线,当场拒绝)。
   procedure Range_Probe is separate;
begin
   if Pts_Empty then
      Event := To_Unbounded_String ("没有点要跟:这一节没有可动的东西");
      Steps_Taken := 0; Blocked_Out := False; Beats := 0;
      return;
   end if;
   Event := S ("hit the safety cap on steps");
   Steps_Taken := 0;
   Beats := 0;
   Blocked_Out := False;
   Jaw := Selfmap.Jaw_All (F, Arm);
   Fl.Track := Long_Float'Max (1.0 / Long_Float (Cw), 0.0);
   Fl.Picture := Long_Float (Integer'(if Cam < Natural (C.Map.Pic_Floor.Length) then C.Map.Pic_Floor (Cam) else 0));
   Fl.Reading := C.Map.Jaw_Noise;
   Fl.Delivery := C.Map.EE_Noise;
   Backup.Clear (Ring);
   Ready_Tables (Ok);
   if not Ok then
      Event := S ("the body stopped answering while I measured my response table");
      return;
   end if;
   --  🔴 证明可以晚,但不许没有:到【要拿这一行去算动作】的这一刻,它必须已经被证明过。
   --  编译期这一块还没量过响应时放行了,现在量完了,当场补判;判不过就一步都不走,把原话退回给脑。
   declare
      Bad : Unbounded_String;
      Dropped : Unbounded_String;
      Notch : Table.Vec := Table.Zero_Vec;
   begin
      for K in 0 .. Chan.Per_Arm - 1 loop
         Notch (K) := C.Map.Amp (Arm * Chan.Per_Arm + K);
      end loop;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            P : constant Point := Pts (I);
            Q : Point := Pts (I);
            Live : Natural := 0;
            --  🔴 没证过的行【摘掉】,不是整节拒绝 —— "不许参与"的意思就是不参与解算。
            --  一行都不剩,才是真的做不到。
            procedure Want (R : Natural; Nm : String) is
            begin
               if Table.Row_Proven (Effs (I), Notch, R) then
                  Live := Live + 1;
               else
                  --  🔴 没证过【不等于】不用它。摘掉的后果:五行全摘 ⇒ 归一后误差恒为 0 ⇒
                  --  每一步发出的命令幅度都是 0,身体空转烧步数(GK 实测:差距 0.241 一动不动)。
                  --  按规矩:说出来,照用。
                  Append (Dropped, (if Length (Dropped) > 0 then "; " else "")
                          & "I used " & Nm & " for " & Say_Item (C, P.Item_No)
                          & " even though " & Table.Row_Why (Effs (I), Notch, R));
               end if;
            end Want;
         begin
            if abs (P.Tu - P.Cu) > 0.0 then
               Want (0, "sideways");
            end if;
            if abs (P.Tv - P.Cv) > 0.0 then
               Want (1, "up-down");
            end if;
            if P.Wz > 0.0 and then abs (P.Tz - P.Z) > 0.0 then
               Want (2, "nearness");
            end if;
            if P.Wsize > 0.0 then
               Want (3, "apparent size");
            end if;
            if P.Wang > 0.0 then
               Want (4, "facing");
            end if;
            --  🔴 判距离的行【一个都不剩】的时候,不许宣布"到了":
            --  那正是"看着对齐、实际差 20 厘米"那一类(GE 实测)。这是无能,如实说。
            if (P.Wz > 0.0 or else P.Wsize > 0.0) and then Q.Wz <= 0.0 and then Q.Wsize <= 0.0 then
               Append (Bad, (if Length (Bad) > 0 then "; " else "")
                       & Say_Item (C, P.Item_No)
                       & ": in this eye I have nothing left that tells me how far away it is "
                       & "(its distance reads as nothing here, and how big it looks is not steady), "
                       & "so lining up the picture would prove nothing");
            end if;
            if Live = 0 then
               Append (Bad, (if Length (Bad) > 0 then "; " else "")
                       & Say_Item (C, P.Item_No) & ": not one of the things this needs is proven");
            end if;
            Pts.Replace_Element (I, Q);
         end;
      end loop;
      if Length (Dropped) > 0 then
         Put_Line ("[身]   ⊘ " & To_String (Dropped));
      end if;
      --  🔴 这里原来会因为"没证过"而一步不走。身体不许自己停 —— 说出来,照走。
      if Length (Bad) > 0 then
         C.Blind_Say := S ("I went ahead even though " & To_String (Bad));
      end if;
   end;
   --  🔴 开工先拨一下量一次 —— 不量就等于闭着眼睛往前够。
   --  不用脑点名:任何机体只要"眼睛跟着我动"就量得了,量不了它自己会说。
   Range_Probe;
   --  🔴🔴 这里以前是 Natural'Min (Step_Cap, Effective_Cap (Step_Limit)) —— 身体把【脑写的步数】
   --  砍到自己那个 60,还把 60 当成"你的上限"报回去(JD 2026-09-15 实测:我写 or 400 steps,
   --  它走了 60 步就回 "hit the step cap (60)")。
   --  owner 死命令:**能让身体停下来的只有【人的命令】和【脑写的 until,含脑给的步数】**,
   --  60 两个都不是。Effective_Cap 本来就已经分好了:脑写了就用脑的,脑没写才用安全上限。
   --  取 min 等于把安全上限偷偷盖在脑的话上面 —— 删。
   for Step in 1 .. Effective_Cap (Step_Limit) loop
      Plan;
      if Note.Say_Stop /= "" then
         Event := Note.Say_Stop;
         return;
      end if;
      Walk (Ok);
      if not Ok then
         Event := S ("the body refused the command");
         return;
      end if;
      Look;
      Learn;
      --  🔴 再量一次的判据:**我从上次量到现在走了多远**,不是"差距有没有变小"。
      --  ID 2026-09-15 实测:判据写成"更近了"时,身体一路没改善 ⇒ 一次都不再量 ⇒
      --  米数永远停在"这是第一次量"。而米数要的正是【两次之间走了多远】,
      --  所以该由走了多远说了算。走够我上一拨自己挪的那么多,就再量一次(零系数,都是量出来的)。
      if Probe_Have then
         declare
            D : Long_Float := 0.0;
         begin
            for K in 0 .. 2 loop
               D := D + (F.EE (Arm) (K) - Probe_EE (K)) ** 2;
            end loop;
            if Sqrt (D) > Probe_Len then
               Range_Probe;
            end if;
         end;
      end if;
      Ranged_Raw := Long_Float'Min (Ranged_Raw, Note.Raw_Now);
      --  🔴 平时不重量表(每段重量 = 一推 13~21 拍的老账);但身体一旦【连着三步说"我的地图不如零假设准"】,
      --  就当场重量一遍 —— 拿着一张被证明错的表一路开,正是 GW 实测"手在动、球的距离一点不变"的直接原因。
      --  只在被证明错的时候才重量:既不回到每段重量,也不拿假表开车。
      --  🔴 第二条触发(IZ 2026-09-15):上面那条只认"零表【更准】",而表说"我一推也改不了这个"时,
      --  零表和这张表【预测一模一样】(都说不动)⇒ 零表永远不更准 ⇒ 重量这条路一次都走不到,
      --  身体就守着一张废表把命令放大到 7.6e9,一段四步全空转。
      --  这一条判的是另一件事:**差距还在,而我连一下【推得动的推】都没开出来**。
      --  两个量都是身体自己的:差距 = 这一步量到的;推得动 = 这根通道的死区 / 本体噪声。
      declare
         Nothing_Asked : Boolean := True;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            if Note.Active (K)
              and then abs Note.Cmd (K) > Long_Float'Max (Note.Floor_Cmd, C.Map.EE_Noise)
            then
               Nothing_Asked := False;
            end if;
         end loop;
         if Note.Blocked
           or else (Nothing_Asked and then Note.Raw_Now > Fl.Track)
         then
            Blocked_Run := Blocked_Run + 1;
         else
            Blocked_Run := 0;
         end if;
      end;
      if Blocked_Run >= 3 then
         Blocked_Run := 0;
         Put_Line ("[身]     连着三步这张表要么不如零表准、要么一推也改不了差距 ⇒ 当场重量一遍");
         C.Blind_Say := S ("for three steps in a row my own map of what my pushes do was worse than assuming "
                           & "nothing happens, so I stopped driving on it and measured it again on the spot");
         declare
            Trust2 : Table.Mask;
            Ok2 : Boolean;
         begin
            Probe_Effects (L, C, F, Cam, Pts, Effs, Trust2, Ok2, Amount * Cap_Mult);
            if Ok2 then
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  Trusts (I) := Trust2;
                  Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust2,
                                Unit_Reach, F.EE (Arm), True);
               end loop;
            end if;
         end;
      end if;
      Judge;
      --  🔴 没有距离这一路的时候要说出来。GV 实测:这一点的深度读成 -0.526(负数,物理上不可能),
      --  远近那一行的权重于是是 0 ⇒ 身体只在【画面上】对齐,完全没有距离信息 ——
      --  于是它停在"图上重合"的地方,报"差 0.005 m、还差 7.3 步",而画面里爪子离球还有 0.08 画幅。
      --  单目对齐的经典歧义:图上重合,实际可能差得远。身体知道自己没距离,必须说,别让脑以为到了。
      if Natural (Pts.Length) > 0 and then Pts (0).Wz <= 0.0 then
         C.Blind_Say := S ("I have no distance to this thing right now - my depth reading for it is not usable - "
                           & "so I am only lining it up in the picture. Lining up in the picture does not mean "
                           & "I am next to it: I could be well short of it or past it.");
      end if;
      if Note.Say_Stop /= "" then
         Event := Note.Say_Stop;
         Beats := Since (L, Beats0);
         return;
      end if;
   end loop;
   Event := S ("steps: hit the step cap (" & Codec.Img (Steps_Taken) & ")");
end Run_Segment;
