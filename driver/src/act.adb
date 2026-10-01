with Ada.Calendar; use type Ada.Calendar.Time;
with Ada.Strings.Fixed;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Limits;
with Codec;
with Draw;
with Flow;
with Monitor;
with Backup;
with Learned; use Learned;
with Sinew;
use type Sinew.Noun_Kind;
use type Sinew.Op;
use type Sinew.Role;
use type Sinew.Outcome;
with Runtime;
with Exam;
with Contact;
with Stats;
with Contact.Gen;
with Contact.Grasp;
with Kinem;
with Contact.Exec;
with Contact.Surface;
with Instrument;
with Lockstep;
with Ada.Exceptions;
with Selfmap.Graph;
package body Act is
   Sigma_Mult : constant Long_Float := 3.0;   --  鼓出来超过背景自己稳健 σ 的几倍才算一块(在真实深度图上验过:3 中,5 杀光);无量纲
   Track_Win : constant Long_Float := 0.10;   --  一步里任何被跟踪的点在画面里最多跑十分之一画幅(跟踪窗,比例,无量纲)
   Cap_Mult : constant Long_Float := 2.0;     --  一步命令上限 = 探针幅度(点在画面里跑过地板的那一档)的几倍(倍数,无量纲;EH:8 倍让阻尼当家,步子反而只剩探针的一倍)
   Step_Cap : constant := 60;                 --  一段最多几步(安全上限,不是策略)
   Unit_Reach : constant Table.Vec := [others => 1.0];

   function S (X : String) return Unbounded_String renames To_Unbounded_String;

   --  按(第几只手,第几个抓握通道)查握区。一条臂可以有好几个通道(五指手),所以不能拿臂号当下标。
   function Zone_Of (C : Context; Arm, Cam : Natural; K : Natural := 0) return Zone.Hand_Zone is
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = K
           and then Cam < Natural (C.Hands (I).Zones.Length)
         then
            return C.Hands (I).Zones (Cam);
         end if;
      end loop;
      return (others => <>);
   end Zone_Of;

   --  这条臂量到几个抓握通道就是几个;没有抓握通道(无人机、只有胳膊的身体)就是 0 —— 不按"至少一个"猜(09-30 原来 Max (1, …)、缺省 1)。
   --  问身体图(Selfmap.Graph,大并行 I1),不读字段
   function Jaws_Of (C : Context; Arm : Natural) return Natural is (Selfmap.Graph.Closing_Count (C.Map, Arm));

   --  这条臂第 K 个抓握通道带不带手指 = 开机把它推到头时,有没有哪台相机量出了握区(Zone.Measure:两次比较、不共用一帧都看见东西动了)。
   --  哪台都没有 ⇒ 这个通道什么都不带,不列手指 / 爪心、不数、不说"你的手指之间"(无人机 DR1 / DR2 2026-09-28:
   --  开机说了"握区量不了",干活时清单照样按抓握通道个数列了两瓣手指一组爪心,拒 me 时还说"我身上量得出 2 瓣手指、1 组爪心")
   function Has_Fingers (C : Context; Arm : Natural; K : Natural := 0) return Boolean is
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = K then
            for Z of C.Hands (I).Zones loop
               if Z.Valid then
                  return True;
               end if;
            end loop;
         end if;
      end loop;
      return False;
   end Has_Fingers;

   --  这具身体上有没有哪个抓握通道带手指
   function Any_Fingers (C : Context) return Boolean is
   begin
      for A in 0 .. Selfmap.Graph.Arm_Count (C.Map) - 1 loop
         for K in 0 .. Jaws_Of (C, A) - 1 loop
            if Has_Fingers (C, A, K) then
               return True;
            end if;
         end loop;
      end loop;
      return False;
   end Any_Fingers;

   --  这条臂上有没有哪个抓握通道带手指
   function Arm_Has_Fingers (C : Context; Arm : Natural) return Boolean is
   begin
      for K in 0 .. Jaws_Of (C, Arm) - 1 loop
         if Has_Fingers (C, Arm, K) then
            return True;
         end if;
      end loop;
      return False;
   end Arm_Has_Fingers;

   --  按(第几只手,第几个抓握通道)取那只"手"。一条臂可以有好几个,拿臂号当下标是错的。
   function Hand_Of (C : Context; Arm : Natural; K : Natural := 0) return Zone.Hand is
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = K then
            return C.Hands (I);
         end if;
      end loop;
      return (others => <>);
   end Hand_Of;

   --  读数 R 离"空手合"那头往张开那头走了多远(读数单位):两头是开机推到头量的(V1b ② 2026-09-27),方向按量的 —— 不假设"读数变小 = 合"
   function Past_Empty (H : Zone.Hand; R : Long_Float) return Long_Float is
   begin
      return (if H.Open_Reading >= H.Empty_Close then R - H.Empty_Close else H.Empty_Close - R);
   end Past_Empty;

   --  这个点属于哪个抓握通道(不是抓握通道带的就当 0 号)
   function Jaw_K_Of (Ck : Natural) return Natural is
     (if Ck >= Chan.Per_Arm then Ck - Chan.Per_Arm else 0);

   --  🔴 读手指深度用的窗口:取【这一瓣自己】最窄那一边的四分之一,落在手指身上。
   --  以前用的是整只手的张幅,把自己的白色大臂框了进去(大臂比手指更近)⇒ 靠近的那一档分位一路下滑,
   --  9 步从 1.07 m 滑到 0.50 m(LAB 09-09「爬深」)。这一条和"就地重读"是一对,一起被 a7ab7e9 退掉了。
   function Lobe_Win (Z : Zone.Hand_Zone; Cw, Ch : Natural) return Long_Float is
      W1 : constant Long_Float := Long_Float (Zone.Lobe_Of (Z, 0).X1 - Zone.Lobe_Of (Z, 0).X0 + 1) / Long_Float (Cw);
      H1 : constant Long_Float := Long_Float (Zone.Lobe_Of (Z, 0).Y1 - Zone.Lobe_Of (Z, 0).Y0 + 1) / Long_Float (Ch);
      W2 : constant Long_Float := (if Zone.Lobe_Of (Z, 1).Valid then Long_Float (Zone.Lobe_Of (Z, 1).X1 - Zone.Lobe_Of (Z, 1).X0 + 1) / Long_Float (Cw) else W1);
      H2 : constant Long_Float := (if Zone.Lobe_Of (Z, 1).Valid then Long_Float (Zone.Lobe_Of (Z, 1).Y1 - Zone.Lobe_Of (Z, 1).Y0 + 1) / Long_Float (Ch) else H1);
   begin
      --  还没量到手的时候退回一个百分之一画幅的小窗(比例,无量纲)
      if not Z.Valid or else not Zone.Lobe_Of (Z, 0).Valid then
         return 0.01;
      end if;
      --  取最窄那一边的四分之一;再小也留千分之四画幅,免得窗口小到一个像素(比例,无量纲)
      return Long_Float'Max (0.004, 0.25 * Long_Float'Min (Long_Float'Min (W1, H1), Long_Float'Min (W2, H2)));
   end Lobe_Win;

   function Track_Idx (C : Context; Arm, Cam : Natural) return Natural is (Arm * C.Map.N_Cams + Cam);

   --  一段动作同时用所有相机 ⇒ 每台相机的"走之前那一拍"都要留着(光流按点自己那台相机算)
   function All_Gray (F : Plug.Frame) return Buf_Vectors.Vector is
      V : Buf_Vectors.Vector;
   begin
      for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
         V.Append (F.Cams (Cm).Gray);
      end loop;
      return V;
   end All_Gray;

   function Cam_Arm (C : Context; Cam : Natural) return Integer is
   begin
      for A in 0 .. Natural (C.Map.Cam_On_Arm.Length) - 1 loop
         if C.Map.Cam_On_Arm (A) = Integer (Cam) then
            return A;
         end if;
      end loop;
      return -1;
   end Cam_Arm;

   procedure Init_Tracks (C : in out Context) is separate;

   function Cut_Window (C : Context; Cam : Natural; F : Plug.Frame) return Long_Float is separate;

   function Safety_Cap return Positive is (Step_Cap);
   function Effective_Cap (Say_Steps : Natural) return Positive is
     (if Say_Steps > 0 then Say_Steps else Safety_Cap);

   --  两瓣叠到一起(间距 0)也是炸 —— 两根手指不可能落在同一个像素上。
   --  这个洞是自检当场逮到的:|0 − 0.137| > 0.137 是【假】(不是严格大于),原判据放过了它。
   function Into_Depth (Skin, Surface : Long_Float) return Long_Float is
     ((Skin + Surface) / 2.0);

   --  🔴 经历账放哪:和身体文件同一个地方,跨炮留着。永远只追加,不许清。
   function Life_Path return String is
     (if Codec.Env ("BL_LIFE") /= "" then Codec.Env ("BL_LIFE") else "/root/经历.txt");

   --  同一根通道只记一次"我变了",免得同一句话刷满整段话
   function Cn_Changed (C : Context; Cn : Natural) return Boolean is
     (Index (C.Changed_Say, "channel " & Codec.Img (Cn) & " used to move") = 0);

   --  🔴🔴 「这根通道不听话了」以前【只进不出】:`Changed_Say` 从不清空,`Cn_Changed` 是一次性闩,
   --  于是某一步的一次顶死(在仿真里本体读数没有抖动,地板量出来恰好是 0,所以"实到恰好 0.0"
   --  一步就够)被讲成永久的"我变了",而且此后每一轮都讲一遍。CS5 实测:从第 113 轮起
   --  六条通道全被这么宣告死掉,同一份日志里身体却一直在交付(命令与实到平均差 0.0202 m)。
   --  脑读到的是"我整具身体都不听话了"。**那是一句假话,而假话比不说更坏。**
   --  ⇒ 这一根又交付得动了,就把那句话【撤掉】。量过期就收回,是这一层允许说的话之一。
   procedure Cn_Recovered (C : in out Context; Cn : Natural) is
      Tag : constant String := "  I HAVE CHANGED: channel " & Codec.Img (Cn) & " used to move";
      At_S : constant Natural := Index (C.Changed_Say, Tag);
   begin
      if At_S = 0 then
         return;
      end if;
      declare
         Rest : constant String := Slice (C.Changed_Say, At_S, Length (C.Changed_Say));
         Nl : constant Natural := Index (To_Unbounded_String (Rest), "" & ASCII.LF);
         Head : constant String := Slice (C.Changed_Say, 1, At_S - 1);
         Tail : constant String :=
           (if Nl = 0 then "" else Slice (C.Changed_Say, At_S + Nl, Length (C.Changed_Say)));
      begin
         C.Changed_Say := To_Unbounded_String (Head & Tail);
      end;
   end Cn_Recovered;

   --  层数 = 让最粗那一层的位移落到一个像素以内所需要的层数。
   --  上限 6 层:再粗下去图本身只剩几十个像素,已经没有内容可对(不是调参,是图没了)。
   function Levels_For (Px_Move : Long_Float) return Positive is
      N : Natural := Natural (Long_Float'Min (1.0e6, Long_Float'Max (0.0, abs Px_Move)));
      L : Positive := 1;
   begin
      --  整数写:每加一层分辨率【减半】是金字塔的定义本身,不是一个可调的门槛
      while N > 1 and then L < 6 loop
         N := N / 2;
         L := L + 1;
      end loop;
      return L;
   end Levels_For;

   --  1.0 不是系数:它是"沿着相机看的方向走一米,远近最多变一米"这条几何事实本身
   function Depth_Scale_Bad (Depth_Per_Metre : Long_Float) return Boolean is
     (abs Depth_Per_Metre > 1.0);

   --  一行一判:这一行去回两遍的【分歧】要小于两遍的【共识】。两遍都是零 ⇒ 没证据,照留。
   function Row_Is_Measurement (Dif, Con : Long_Float) return Boolean is
     (Con <= 0.0 or else Dif < Con);

   --  尺子:我真挪了多少米(胳膊自己知道)⇒ 这一块游了多少 ⇒ 它有多近。零系数,两个地板都是量出来的。
   function Near_From_Motion (Ran, Moved, Ran_Floor, Move_Floor : Long_Float) return Long_Float is
     (if Moved > Move_Floor and then Ran > Ran_Floor then Ran / Moved else 0.0);

   --  它比我远几倍 = 我游得多快 ÷ 它游得多快(同一台相机、同一段挪动 ⇒ 焦距和基线约掉)
   function Farther_By (Near_Me, Near_It : Long_Float) return Long_Float is
     (if Near_Me > 0.0 and then Near_It > 0.0 then Near_Me / Near_It else 0.0);

   type Xyz is array (0 .. 2) of Long_Float;

   --  两拨是不是【同一下】:世界里的方向要一样。容差 = 方向本身的不确定度(读数抖动 ÷ 挪了多远)。
   function Same_Nudge (Dot, Len_A, Len_B, Slid, Floor : Long_Float) return Boolean is
     (Len_A > 0.0 and then Len_B > 0.0 and then Slid > 0.0
      --  方向要一样。幅度不必一样 —— 除以各自的实到之后它本来就抵消了,
      --  硬拿方向那条容差(0.3%)去卡幅度 ⇒ 真实推送几个百分点的波动就被判成"不是同一下",
      --  这道闸当场变成永不放行(IN 2026-09-15 实测:方向一致度 0.999 也被拦)。
      and then Dot / (Len_A * Len_B) >= 1.0 - Floor / Slid);

   --  走近一段再拨同样的一下 ⇒ 米数。滑得没比上次多(没过跟踪抖动)= 这一段没走近 ⇒ 说不准。
   function Distance_Now (Travelled, Swim_Then, Swim_Now, Floor : Long_Float) return Long_Float is
     (if Travelled > 0.0 and then Swim_Then > 0.0 and then Swim_Now - Swim_Then > Floor
        and then Travelled * Swim_Then / (Swim_Now - Swim_Then) >= Travelled
      then Travelled * Swim_Then / (Swim_Now - Swim_Then) else 0.0);

   --  走这么远最远分辨得到多远:滑速变化要过跟踪抖动 ⇒ 走 D 能分辨到 滑速 × D ÷ 抖动
   function Can_Tell_Upto (Swim_Now, Travelled, Floor : Long_Float) return Long_Float is
     (if Floor > 0.0 then Swim_Now * Travelled / Floor else 0.0);

   function Depth_Ok (Zd, Old_Z, Pred_Z, Noise, Last_Rejected : Long_Float) return Boolean is
     (Old_Z <= 0.0
      --  连着两次被拒、而两次读数互相吻合 ⇒ 新值是可重复的,旧基准才是陈的 ⇒ 收
      or else (Last_Rejected > 0.0
               and then abs (Zd - Last_Rejected) <= Long_Float'Max (0.0, Noise))
      --  🔴 带子的【中心是上次真读到的那个数】,不是表预测的位置。半宽 = 表说这一步会变多少 + 自己的抖动。
      --  以前中心设在预测上:表一旦高估这一步的变化(说走 0.223,实际没走),
      --  一个离上次真读数只差 0.04 m 的【诚实读数】就会被判成离谱 ——
      --  HS 实测:读到 2.306 · 上次真读到 2.346 · 表说走 0.223 · 抖动 0.022
      --            |2.306-2.569| = 0.263 > 0.245 ⇒ 拒。于是深度连着几十推纹丝不动。
      --  改成以 Old_Z 为中心之后:|2.306-2.346| = 0.040 <= 0.245 ⇒ 收,而该挡的仍然挡得住。
      or else (if Pred_Z <= 0.0
               then abs (Zd - Old_Z) <= Long_Float'Max (0.0, Noise)
               else abs (Zd - Old_Z) <= abs (Pred_Z - Old_Z) + Long_Float'Max (0.0, Noise)));

   --  0.5 = 画面中心(比例,不是系数:u 是 0..1 的画幅比例,中心就在一半处)
   function On_My_Plane (T_Pic, T_Depth, My_Depth : Long_Float) return Long_Float is
     (if My_Depth > 0.0 and then T_Depth > 0.0
      then 0.5 + (T_Pic - 0.5) * (T_Depth / My_Depth)
      else T_Pic);

   function Push_Cap (Ceiling, Noise, Dead : Long_Float) return Long_Float is
     (Long_Float'Max (Long_Float'Max (Noise + Noise, Dead), Ceiling));

   function Extrapolation_Blew (Was, Now : Long_Float) return Boolean is
     (Was > 0.0 and then (Now <= 0.0 or else abs (Now - Was) > Was));

   --  差 + 差 <= 手挪的 —— 写成加法是为了不引入一个手挑的系数(棘轮认那个形状)
   function Came_With_Me (Obj_Du, Obj_Dv, Hand_Du, Hand_Dv : Long_Float) return Boolean is
      Hand_Len : constant Long_Float := Sqrt (Hand_Du ** 2 + Hand_Dv ** 2);
      Miss : constant Long_Float := Sqrt ((Obj_Du - Hand_Du) ** 2 + (Obj_Dv - Hand_Dv) ** 2);
   begin
      return Hand_Len > 0.0 and then Miss + Miss <= Hand_Len;
   end Came_With_Me;

   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean is
     (case R is
         --  grasper = 我量到能相向靠拢、中间扫出一片能装东西的那一组
         when Sinew.Rl_Grasper => K = Grip,
         --  pusher = 推得动东西、但【合不拢】的部件。合得拢的(Grip / Finger)一律不算 ——
         --  以前这里写 Grip | Piece,爪心同时满足两个角色,语言的角色区分等于没有。
         --  这具身体上量不到这样的零件时,绑不上就是对的,身体要说出来,不许拿爪心顶数。
         when Sinew.Rl_Pusher => K = Piece,
         --  me = 整个我。只有"推一下整幅画面跟着变、而且身上量不出可分的零件"的机体(无人机)才有它。
         --  这具身体量得出手指和爪心 ⇒ me 绑不上,而这是对的;身体要说清为什么,不许只回一句"认不出"。
         when others => False);

   function Rel_Cmd (R : Sinew.Rel) return String is
     (case R is
         when Sinew.Re_Touching => "at",   when Sinew.Re_Above => "above", when Sinew.Re_Below => "below",
         when Sinew.Re_Left => "left",     when Sinew.Re_Right => "right",
         when Sinew.Re_Nearer => "front",  when Sinew.Re_Farther => "back",
         when Sinew.Re_Onto => "onto",     when Sinew.Re_Off => "off", when Sinew.Re_Into => "into",
         when Sinew.Re_Facing => "face",
         when Sinew.Re_Press => "press",   when Sinew.Re_Still => "",
         when others => "?");   --  close / open / clear 各有各的分支,够得着这里的只有 Re_None
   function Rel_Has_Own_Branch (R : Sinew.Rel) return Boolean is
     (case R is when Sinew.Re_Close | Sinew.Re_Open | Sinew.Re_Clear | Sinew.Re_Still | Sinew.Re_Qty => True,
                when others => False);

   --  🔴 结局词 → 判法,唯一的一处(见 act.ads 的说明)
   function Until_Word (O : Sinew.Outcome) return String is
     (case O is
         when Sinew.Oc_Touched => "contact",
         when Sinew.Oc_Stuck   => "resist",
         when Sinew.Oc_Slipped => "slip",
         when Sinew.Oc_Free    => "free",      --  离开了原来靠着的面 = 被拿起来了
         when Sinew.Oc_Lost    => "lost",      --  看不见我正跟着的东西了
         when Sinew.Oc_Settled => "settle",
         when Sinew.Oc_Stalled => "stall",
         when Sinew.Oc_Arrived => "arrived",
         when others           => "steps");    --  timeout / none / refused(refused 编译期就被拒了)
   --  字符串那一跳的反向表。自检逐词钉死 Kind_Of_Word (Until_Word (O)) = Until_Of (O),
   --  任何一次"顺手并个词"都会当场红
   function Kind_Of_Word (W : String) return Monitor.Until_Kind is
     (if W = "contact" then Monitor.U_Contact
      elsif W = "stall" then Monitor.U_Stall
      elsif W = "resist" then Monitor.U_Resist
      elsif W = "slip" then Monitor.U_Slip
      elsif W = "settle" then Monitor.U_Settle
      elsif W = "lost" then Monitor.U_Lost
      elsif W = "free" then Monitor.U_Free
      else Monitor.U_Steps);
   function Wants_Arrive (O : Sinew.Outcome) return Boolean is
     (case O is when Sinew.Oc_Arrived => True, when others => False);
   function Until_Of (O : Sinew.Outcome) return Monitor.Until_Kind is
     (case O is
         when Sinew.Oc_Touched => Monitor.U_Contact,
         when Sinew.Oc_Stuck   => Monitor.U_Resist,
         when Sinew.Oc_Slipped => Monitor.U_Slip,
         when Sinew.Oc_Free    => Monitor.U_Free,
         when Sinew.Oc_Lost    => Monitor.U_Lost,
         when Sinew.Oc_Settled => Monitor.U_Settle,
         when Sinew.Oc_Stalled => Monitor.U_Stall,
         when others           => Monitor.U_Steps);   --  arrived / timeout 都走步数上限,靠 Wants_Arrive 区分

   --  这一台相机里,脑上次点名那一块有多宽(画幅)。切块的第二把尺子要按相机各算各的。
   function Named_Span (C : Context; Cam : Natural; Cw : Natural) return Long_Float is
      N : constant Integer := (if Cam < Natural (C.Wld.Cams.Length) then C.Wld.Cams (Cam).Named else -1);
   begin
      if N < 0 or else Natural (N) >= World.Count (C.Wld, Cam) then
         return 0.0;
      end if;
      declare
         Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (N));
         R : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
      begin
         if R.X1 <= R.X0 then
            return 0.0;
         end if;
         --  用它自己的框宽,除以这台相机的画幅宽 —— 两个都是量出来的
         return Long_Float (R.X1 - R.X0 + 1) / Long_Float'Max (1.0, Long_Float (Cw));
      end;
   end Named_Span;

   --  这一台相机比"看得最大的那一台"小多少倍:探针在这台里就要按这个倍数多推一点才看得见。
   --  1.0 = 它就是看得最大的那台;量不到就退回 1.0(不放宽,和以前一样)。
   function Cam_Slack (C : Context; Arm, Cam : Natural) return Long_Float is
      Best, Here : Long_Float := 0.0;
   begin
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         declare
            Ix : constant Natural := Arm * C.Map.N_Cams + Cm;
            V : constant Long_Float :=
              (if Ix < Natural (C.Map.Cam_Frac.Length) then C.Map.Cam_Frac (Ix) else 0.0);
         begin
            if V > Best then
               Best := V;
            end if;
            if Cm = Cam then
               Here := V;
            end if;
         end;
      end loop;
      if Here <= 0.0 or else Best <= Here then
         return 1.0;
      end if;
      return Best / Here;
   end Cam_Slack;

   --  ── 接触集(最小可落地版) ──────────────────────────────────────────────
   --  来自 09-06 删掉的那三个 crate,核心判据【一个都不需要 μ】(原文:
   --  "只给方向 —— 一个轴 + 一个半张角。没有牛顿" / "不需要 μ:μ 只决定'多大算过线',
   --   而排序只需要'谁更小'")。这里把它搬成画面里的版本,无深度也能算:
   --    硬过滤 within_jaw = 这一处要夹的【弦长】≤ 量出来的爪口(瓣心距)
   --    ① face_tilt  = 弦长随位置变化的【斜率】。斜率≈0 ⇒ 两个面平行相对 ⇒ 不会横着滑走。
   --       (原文:"沿指头宽方向,近面/远面的深度随位置变化的斜率取反正切" —— 换成轮廓即此)
   --    ② com_offset = 这个下手点离这块东西【重心】多远。管"提起来转出去"。
   --       (原文举的例子正是我们的场景:"抓在剪刀手柄圆环上,而重量全在刀刃那头")
   --  两项都无量纲/无常数地合并:各自排名相加取最小(只用序,不引入权重)。
   Last_Bright : Bools;
   Last_Bright_Cam : Integer := -1;
   --  这只眼这一帧自己算出来的明暗分界(0..255)。清单里报每一块的亮度时拿它当基准:
   --  「亮度 212,这只眼的分界是 149」比光说 212 多一件事 —— 它是我量的,不是写死的。
   Last_Split : Long_Float := -1.0;

   function Cut_Bright (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is separate;

   function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is separate;

   --  🔴 脑点过名的东西,每一帧在【上一帧量到它的地方】原样再量一遍,量到的那一整块顶替掉全图切块在它身上切出的碎片。
   --  为什么:没有深度时全图按明暗切,一把剪刀被切成四五个指甲盖大的碎框(09-21 头顶眼实测),腕眼一帧 191–450 件;
   --  跨帧"认号"是在这些碎片里找最近的一块,于是身份漂(HB1)、视差拿到两块不同的碎片(GC42)。
   --  在它自己的框里量,出来的是一整块、形心三个独立回合差 0.1 px。全图切块照旧留着(碰没碰到别的东西还靠它)。
   --  这一块的像素平均多亮、它框里剩下的背景平均多亮(掩膜 = 整幅;都在它的框里数)。数不到 ⇒ -1
   procedure Blob_Levels (G : Buf; W, H : Natural; Mask : Bools; R : Picture.Region; Thing, Back : out Long_Float) is
      St, Sb : Long_Float := 0.0;
      Nt, Nb : Natural := 0;
   begin
      Thing := -1.0; Back := -1.0;
      if Natural (G.Length) < W * H or else Natural (Mask.Length) < W * H then
         return;
      end if;
      for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
         for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
            if Mask (Y * W + X) then
               St := St + Long_Float (G.Element (Y * W + X)); Nt := Nt + 1;
            else
               Sb := Sb + Long_Float (G.Element (Y * W + X)); Nb := Nb + 1;
            end if;
         end loop;
      end loop;
      if Nt > 0 then
         Thing := St / Long_Float (Nt);
      end if;
      if Nb > 0 then
         Back := Sb / Long_Float (Nb);
      end if;
   end Blob_Levels;

   --  掩膜里离形心最近的那个像素(它身上的一点;剪刀这种中间空的东西形心本身不在它身上)。没有 ⇒ -1
   procedure On_Pixel (M : Bools; W, H : Natural; R : Picture.Region; U, V : out Long_Float) is
      Cx : constant Long_Float := R.Cu * Long_Float (W);
      Cy : constant Long_Float := R.Cv * Long_Float (H);
      Best : Long_Float := Long_Float'Last;
   begin
      U := -1.0; V := -1.0;
      if Natural (M.Length) /= W * H or else R.Count = 0 then
         return;
      end if;
      for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
         for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
            if M (Y * W + X) then
               declare
                  D2 : constant Long_Float := (Long_Float (X) - Cx) ** 2 + (Long_Float (Y) - Cy) ** 2;
               begin
                  if D2 < Best then
                     Best := D2; U := Long_Float (X); V := Long_Float (Y);
                  end if;
               end;
            end if;
         end loop;
      end loop;
   end On_Pixel;

   --  脑框出来的那件东西在这一帧里的像素(2026-09-26 owner 批准装 SAM):SAM 按框出整片像素 —— 这件量只有这一种量法。
   --  (驱动自己按明暗切,一把剪刀常常只切出一截、或连着别的东西;SHOT1 腕眼里剪刀被画面下边切着,两只眼的"中心"差 1.7 cm。
   --  以前没配仪器时退回按明暗量,09-26 owner:一个量只许一种量法 ⇒ 删了;没配仪器 / 仪器没回来 = 这一样量不出来,如实说)
   --  Iso = 这一片没顶到画面边(顶到 = 被画面切了一截,形心和长轴不可信)
   --  Pu_On/Pv_On ≥ 0:它身上的一点(给仪器当"就是这一点",和框一起给)
   Said_No_Seg : Boolean := False;
   --  不动的眼每轮核对要配点仪器(板上的点从参考图配到此刻的图)。没配 ⇒ 核不了,开机说一次(以前悄悄跳过 = 日志全绿而世界没发生)
   Said_No_Check_Done : Boolean := False;
   procedure Say_No_Check is
   begin
      if not Said_No_Check_Done then
         Said_No_Check_Done := True;
         Put_Line ("[身] 📐 没配配点仪器 ⇒ 不动的眼挪没挪、挡没挡,这次核不了(核对只有配点这一种量法)");
      end if;
   end Say_No_Check;
   procedure Seg_In_Box (C : Context; F : Plug.Frame; Cam : Natural; X0, Y0, X1, Y1 : Natural; Got, Iso : out Boolean; R : out Picture.Region; M : out Bools;
                         Pu_On, Pv_On : Long_Float := -1.0) is separate;

   procedure Remeasure_Boxed (C : in out Context; F : Plug.Frame; Cam : Natural; Regs : in out Picture.Regions) is separate;

   --  这一块是不是脑点过名的那几件之一(拿形心对;我量出来的那一块原样进了槽,所以对得上)。-1 = 不是
   function Boxed_Index (C : Context; Cam : Natural; Cu, Cv : Long_Float) return Integer is
   begin
      for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
         if C.Boxed (Bi).Cam = Cam
           and then abs (C.Boxed (Bi).Cu - Cu) < 1.0e-9 and then abs (C.Boxed (Bi).Cv - Cv) < 1.0e-9
         then
            return Integer (Bi);
         end if;
      end loop;
      return -1;
   end Boxed_Index;

   --  这只眼里叫这个名字的那一件(脑点过名的);没有 ⇒ -1
   function Boxed_By (C : Context; Cam : Natural; Name : Unbounded_String) return Integer is
   begin
      if Length (Name) = 0 then
         return -1;
      end if;
      for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
         if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Name = Name then
            return Integer (Bi);
         end if;
      end loop;
      return -1;
   end Boxed_By;

   --  清单第 N 件(1 起)叫什么:脑点过名才有名字,没点过 ⇒ 空
   function Item_Name (C : Context; N : Natural) return Unbounded_String is
   begin
      if N >= 1 and then N <= Natural (C.Items.Length) then
         declare
            It : constant Item := C.Items (N - 1);
            Bx : constant Integer := (if It.Kind in Thing | Thing_Remembered then Boxed_Index (C, It.Cam, It.Cu, It.Cv) else -1);
         begin
            if Bx >= 0 then
               return C.Boxed (Natural (Bx)).Name;
            end if;
         end;
      end if;
      return Null_Unbounded_String;
   end Item_Name;

   --  脑说过"这只眼里没有【它】":按名字 × 眼记(H29 2026-09-22 实测:只按眼记,脑写个 it 绑不上,整只腕眼就被判死,再也换不过去)
   function Is_Blind (C : Context; Cam : Integer; Name : Unbounded_String) return Boolean is
      Bx : constant Integer := (if Cam >= 0 then Boxed_By (C, Natural (Cam), Name) else -1);
   begin
      return Bx >= 0 and then C.Boxed (Natural (Bx)).Blind;
   end Is_Blind;

   procedure Mark_Blind (C : in out Context; Cam : Natural; Name : Unbounded_String) is
      Bx : constant Integer := Boxed_By (C, Cam, Name);
   begin
      if Bx >= 0 then
         declare
            B : Boxed_Thing := C.Boxed (Natural (Bx));
         begin
            B.Blind := True; B.Seen := False;
            C.Boxed.Replace_Element (Natural (Bx), B);
         end;
      elsif Length (Name) > 0 then
         declare
            B : Boxed_Thing;
         begin
            B.Name := Name; B.Cam := Cam; B.Blind := True;
            C.Boxed.Append (B);
         end;
      end if;
   end Mark_Blind;

   --  这只眼看的地方变了(转过了 / 脑明确换过来了)⇒ 以前说的"没有它"不再算数
   procedure Clear_Blind (C : in out Context; Cam : Natural) is
   begin
      for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
         if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Blind then
            declare
               B : Boxed_Thing := C.Boxed (Bi);
            begin
               B.Blind := False;
               C.Boxed.Replace_Element (Bi, B);
            end;
         end if;
      end loop;
   end Clear_Blind;

   --  一件东西【叫什么】:脑点过名的用脑起的名字,我身上的用它是哪一块。编号不进语言(LANGUAGE §3.1),
   --  也就不该进我说给脑听的话和经历账 —— T2 2026-09-21 实测:清单每行以 "item N:" 开头、经历账全是
   --  "item 6 above item 10",Qwen 于是把 item 当成了东西的名字(`do grasper touching item item`),
   --  身体去问"item 在哪",脑随手框了一个乐高小人。
   function Say_Item (C : Context; N : Natural) return String is
   begin
      if N < 1 or else N > Natural (C.Items.Length) then
         return "something I could not point at";
      end if;
      declare
         It : constant Item := C.Items (N - 1);
         Bx : constant Integer := (if It.Kind in Thing | Thing_Remembered then Boxed_Index (C, It.Cam, It.Cu, It.Cv) else -1);
      begin
         case It.Kind is
            when Grip => return "grip " & Codec.Img (It.Arm + 1);
            when Finger => return "a finger of arm " & Codec.Img (It.Arm + 1);
            when Piece => return "a part of arm " & Codec.Img (It.Arm + 1);
            when Thing_Held => return "the thing in my hand";
            when Thing | Thing_Remembered =>
               return (if Bx >= 0 then To_String (C.Boxed (Natural (Bx)).Name) else "a thing nobody has named");
         end case;
      end;
   end Say_Item;

   --  同一帧、同一台相机只切一次(颜色切块要扫全图两遍,一步里被问好几次)
   function Cut_Things (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Self : constant access Context := C'Unrestricted_Access;
   begin
      if C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam) then
         return C.Cut_Regs;
      end if;
      Self.Cut_Regs := Cut_Things_Raw (C, F, Cam);
      Remeasure_Boxed (Self.all, F, Cam, Self.Cut_Regs);
      Self.Cut_Seq := F.Seq;
      Self.Cut_Cam := Integer (Cam);
      return Self.Cut_Regs;
   end Cut_Things;

   function Cell_Of (C : Context; U, V : Long_Float) return Natural is
      Col : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Cols) - 1.0, Long_Float'Floor (U * Long_Float (C.Cols)))));
      Row : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Rows) - 1.0, Long_Float'Floor (V * Long_Float (C.Rows)))));
   begin
      return Row * C.Cols + Col + 1;
   end Cell_Of;

   --  ── 编号表:先我身上的,再世界里的 ──
   --  Keep = True:不清空清单,编号接着往下排 —— 这样【每一台相机】都能各切各的、各编各的号,
   --  而号是全局唯一的。以前只有当前那一台有号:GM 里我答"一个都不是",而头顶相机里球一直看得见,
   --  只是它没有号可点。
   --  日志里的长度(④,09-27):世界单位 = 第一只手运动学的单位(x5 约 52 mm;每具身体按身体文件固定),不是米 —— 印"单位"。
   --  原来这里印"X m",给脑的话里"jaw 1.752 m"其实是张口 91 mm;给脑的长度一律走 Len(按身体自己的尺子)
   function Mm (X : Long_Float) return String is (Codec.Fmt (X, 3) & " 单位");
   --  身体的尺子(④,09-27):第一只碰桌面量过指尖的手,眼到两瓣指尖中点的距离(世界单位;一只都没量过 = 0)
   function Hand_Len (C : Context) return Long_Float is
   begin
      for A in 0 .. C.Map.Arms - 1 loop
         if A < Natural (C.Map.Cam_On_Arm.Length) and then C.Map.Cam_On_Arm (A) >= 0 and then Natural (C.Map.Cam_On_Arm (A)) < Natural (C.Geo.Length) then
            declare
               G : constant Geom.Cam_Geo := C.Geo (Natural (C.Map.Cam_On_Arm (A)));
            begin
               if G.Tip_Valid and then G.Tip_Touch and then Geom.Norm (G.Tip) > 0.0 then
                  return Geom.Norm (G.Tip);
               end if;
            end;
         end if;
      end loop;
      return 0.0;
   end Hand_Len;
   --  给脑的长度:按指尖长说;没量过指尖就照实说是我自己的比例
   function Len (C : Context; X : Long_Float) return String is
      H : constant Long_Float := Hand_Len (C);
   begin
      return (if H > 0.0 then Codec.Fmt (X / H, 2) & " hand-lengths" else Codec.Fmt (X, 3) & " units of my own scale");
   end Len;

   procedure Build_Listing (C : in out Context; F : Plug.Frame; Cam : Natural; RGB : in out Buf; Text : out Unbounded_String;
                            Keep : Boolean := False; Things_Only : Boolean := False) is separate;

   --  ── 被跟踪的点 ──
   type Point is record
      Arm : Natural := 0;
      --  🔴 这一点是【哪台相机】里的。一段动作要同时用几台相机时,每个点各带各的;
      --  只有一台时全都等于那一台,行为和以前一模一样(这一步是纯搬家)。
      Cam : Natural := 0;
      Kind : Track_Kind := Piece_Pt;
      Slot : Integer := -1;
      Item_No : Natural := 0;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道 ⇒ 这块是手指)
      Blob : Integer := -1;      --  这块的第几团(手指两团时一团一个点:两指各自到位,歪了就有一团不到位 —— 倾斜自然被罚)
      Cu, Cv, Z : Long_Float := 0.0;
      Tu, Tv, Tz : Long_Float := 0.0;
      Wz : Long_Float := 0.0;
      Desc : Unbounded_String;
      Box_W, Box_H : Long_Float := 0.0;
      Count : Natural := 0;
      Height : Long_Float := 0.0;
      Z_Noise : Long_Float := 0.0;   --  这一点读深度抖多少(米):高度是深度之差,判"离开了面"用它当地板
      --  🔴 上一次【真从深度图上读到】的远近(0 = 还没读到过)。闸要盯着它,不能盯 Z ——
      --  Z 有可能是按位姿猜出来的、从没被眼睛校过,拿猜测当基准会把真读数全挡在外面
      --  (JE 实测:深度从此纹丝不动 2.422,而真读数在 0.45~0.61,和球的 0.64 同一个尺度)。
      Z_Seen : Long_Float := 0.0;
      --  🔴 上一次【被闸拒掉】的读数。闸拒了一次就永远拿旧值当基准 ⇒ 真实深度一旦变了就锁死:
      --  HE 实测手的深度连着 12 步一模一样 2.182 m,而它在画面里一直在动,差距全卡在"远近"上不动。
      --  这是"永不响的闸"那一类。escape:两次被拒的读数【互相吻合】(差在这一点自己的读深抖动之内)
      --  ⇒ 两次独立测量一致,胜过一个陈旧的基准 ⇒ 收下新值。零系数,用的是量出来的 Z_Noise。
      Z_Rej : Long_Float := 0.0;
      --  🔴 这一点是不是【被夹在画面边界上】。跟丢 ≠ 贴边:跟丢的时候读数仍可能是真的,
      --  而贴边意味着真值在画面外、读到的是那儿的墙(HF:点被夹到 (0,0),深度一路放到 5.109 m)。
      --  深度闸的"出路"只在【不贴边】时才给。
      At_Edge : Boolean := False;
      --  🔴🔴 目标的【画面坐标】是在哪个远近上量的。把目标搬到我这个远近平面上再比,
      --  倍数要用这一个,不是 Tz。HP 实测:`into` 的目标是"左右别动,只把远近走到球的腰上"
      --  ⇒ Tu/Tv 直接抄的我自己的位置(在【我的】远近上),而 Tz 是球的远近;
      --  拿 Tz/Z = 1.63 去放大"别动",就把"别动"变成了"一路往画面外走" ——
      --  被跟的点整天往右沿飘到 u=1.000,根子在这儿。0 = 就在我这个远近上(不放大)。
      Tuv_Z : Long_Float := 0.0;
      --  🔴🔴 尺子量出来的"它有多近"(画幅每米):我自己挪一米,它在画面里游几幅。
      --  这一格【不碰深度图】—— 深度读数被我自己量出来放大了二三十倍,而这个数是我的胳膊量的。
      --  0 = 这一段还没量过(没挪够 / 没游够),不许当真。
      Near : Long_Float := 0.0;
      Near_N : Natural := 0;      --  量过几次(一次不算稳,两次以上才谈得上重复)
      --  🔴 它离我多少米 —— 全靠我自己走出来的,不碰深度图。0 = 还没量出来。
      Dist : Long_Float := 0.0;
      --  🔴🔴 我【没走】的时候,同一下拨动量出来的滑速自己会晃多少(IJ 2026-09-15 量出来的真地板)。
      --  IJ 实测:两次只隔 2 mm,滑速却差了 40% —— 那不是"走近了",是这一拨落点不一样。
      --  拿跟踪抖动当地板挡不住它(小了两个数量级);真正的地板只能是【原地重量一遍它自己晃多少】。
      Near_Jit : Long_Float := 0.0;
      Err0 : Long_Float := 0.0;
      Lost : Boolean := False;   --  这一步没在画面里认出它,位置是按表猜的
      Has_Meas : Boolean := False;              --  眼睛(光流)另外量到的位置,只用来修表
      Meas_U, Meas_V, Meas_Z : Long_Float := 0.0;
      Known : Boolean := True;                  --  这个位置是真看过的/离真看过的样本不超过一步 ⇒ 不是,走之前先看一眼
      Elong : Long_Float := 1.0;                --  脑点名那一刻这块的胖瘦(长轴/短轴),用来和别的块区分
      Gray : Long_Float := -1.0;                --  脑点名那一刻这块的平均灰度(< 0 = 没量到)
      Unsure : Boolean := False;                --  重新认的时候有两块一样像 ⇒ 不许自己挑,回去问脑
      Size : Long_Float := 0.0;                 --  这一块在画面里看着多大(框的边长,画幅):离得越近越大
      Ang : Long_Float := 0.0;                  --  这一块的朝向(主轴角的两倍,弧度;两倍 = 让主轴的正负两种写法算同一个)
      Tsize, Tang : Long_Float := 0.0;          --  要它看着多大 / 转到多少
      Wsize, Wang : Long_Float := 0.0;          --  这两样这一段要不要(0 = 不管)
      Steps_Err : Long_Float := 0.0;            --  上一步算出来的"还差几步"(三样都除以推一步能改多少之后的总和)
      --  🔴 这一步里有【要管的行】因为"推一下能改多少"没量出来而算不出来。
      --  算不出 ≠ 到了:JH 2026-09-16 实测,两行都没量到 ⇒ 权重被清零 ⇒ 总和 0 ⇒ 打印成
      --  `还差 0.0 步`,而同一屏的诊断写着 `误差 0.4277`。这个数骗过我一次(IZ 的假 settled)。
      No_Scale : Boolean := False;
      Err_U, Err_V, Err_Z : Long_Float := 0.0;  --  拆开的五样(左右 / 上下 / 远近 / 大小 / 朝向),单位都是"还差几步"
      Err_S, Err_A : Long_Float := 0.0;
      Raw_Err : Long_Float := 0.0;              --  不随表变的差距(全是比例):画面距离 + 远近差几成 + 大小差几成 + 朝向差几成。
                                                --  判"有没有在靠近"只能用它 —— "还差几步"的刻度每步都在变,尺子一缩就看着像退步
      Par_Tu, Par_Tv : Long_Float := 0.0;       --  两团展开时,整块的目标(看清各团真实位置后按它重算各团目标)
      To_Grip : Boolean := False;               --  这个点是【被换成跟着那个东西】的:目标是我的握区,不是它自己
      Hard : Boolean := False;                  --  脑说的是 hold ⇒ 这一条整段不许被牺牲(解算时进硬约束,软目标只能在它的零空间里做文章)
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);

   --  🔴 这一块在画面上贴着我自己的某一块吗(隔得比我自己那块还宽 ⇒ 碰不到)
   function Near_My_Piece (Pts : Point_Vectors.Vector; I : Natural) return Boolean is
      Best : Long_Float := Long_Float'Last;
      Mine : Long_Float := 0.0;
   begin
      for J in 0 .. Natural (Pts.Length) - 1 loop
         if J /= I and then Pts (J).Kind /= Thing_Pt and then not Pts (J).Lost then
            declare
               D : constant Long_Float :=
                 Sqrt ((Pts (I).Cu - Pts (J).Cu) ** 2 + (Pts (I).Cv - Pts (J).Cv) ** 2);
               W : constant Long_Float :=
                 Long_Float'Max (Long_Float'Max (Pts (J).Box_W, Pts (J).Box_H),
                                 Long_Float'Max (Pts (I).Box_W, Pts (I).Box_H));
            begin
               if D < Best then
                  Best := D; Mine := W;
               end if;
            end;
         end if;
      end loop;
      --  身上一块都看不见 ⇒ 说不准它贴没贴着我 ⇒ 不许据此宣布碰到
      return Best < Long_Float'Last and then Best <= Mine;
   end Near_My_Piece;

   type Effect_Array is array (Natural range <>) of Table.Effect;
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector);

   --  还差多少:只算画面上的距离(画幅)。远近不混进来 —— 混着求和是错的判据(LAB 2026-08-17)
   --  角度差绕回 (-π, π](两倍角的世界里,这等于把主轴的正负两种写法当成同一个)
   function Wrap (X : Long_Float) return Long_Float is
      Two_Pi : constant Long_Float := 2.0 * Ada.Numerics.Pi;
      Y : Long_Float := X;
   begin
      while Y > Ada.Numerics.Pi loop
         Y := Y - Two_Pi;
      end loop;
      while Y <= -Ada.Numerics.Pi loop
         Y := Y + Two_Pi;
      end loop;
      return Y;
   end Wrap;

   --  "看着多大"最小分得清多少:框变一个像素(画幅的 1/宽)。仿真里静止两拍画面一模一样,拿"静止时抖多少"当地板会得到 0 = 没有地板
   function Size_Floor (Cw : Natural) return Long_Float is (1.0 / Long_Float (Natural'Max (1, Cw)));
   --  朝向最小分得清多少:这块最长的那边偏一个像素(两倍角 ⇒ ×4)
   function Ang_Floor (P : Point; Cw, Ch : Natural) return Long_Float is
     (4.0 / Long_Float'Max (4.0, Long_Float'Max (P.Box_W * Long_Float (Cw), P.Box_H * Long_Float (Ch))));

   --  到位了没:画面上进了跟踪噪声,且远近的差不超过这块东西自己的尺寸(全是量出来的,没有写死的容差)
   --  🔴 Reached 已删(owner 2026-09-14):"到了没到"只有脑能判,身体不许自己算一个容差说"够近了"。

   function Held_Now (C : Context; Arm : Natural) return Integer is
     (if C.Wld.Holding and then C.Wld.Held_Arm = Integer (Arm) then C.Wld.Held_Slot else -1);

   function Find_Effect (C : Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer := -1) return Integer is
   begin
      for I in 0 .. Natural (C.Tables.Length) - 1 loop
         if C.Tables (I).Arm = Arm and then C.Tables (I).Cam = Cam and then C.Tables (I).Kind = Kind and then C.Tables (I).Chan_K = Chan_K and then C.Tables (I).Blob = Blob
           and then C.Tables (I).Held = Held_Now (C, Arm) then
            return I;
         end if;
      end loop;
      return -1;
   end Find_Effect;

   procedure Store_Effect (C : in out Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer; E : Table.Effect; Trust : Table.Mask; Reach : Table.Vec := Unit_Reach;
                           Pose : Plug.Arm_Pose := [others => 0.0]; Has_Pose : Boolean := False;
                           Beat : Natural := 0; Agree : Table.Vec := [others => -1.0]) is
      I : constant Integer := Find_Effect (C, Arm, Cam, Kind, Chan_K, Blob);
      Old : constant Integer := I;
      Se : Stored_Effect := (Arm, Cam, Kind, Chan_K, Blob, E, Trust, Reach, Pose, Has_Pose, Held_Now (C, Arm),
                             0, Beat, Agree);
   begin
      --  🔴 自我:每学一次就加一次,并记下是第几拍学的。没有这两格,身体只能说"是这个数",
      --  说不出"我有多信、什么时候学的"。来回对表的分歧也跟着存,没对过的沿用旧的。
      if Old >= 0 then
         Se.N_Learned := C.Tables (Natural (Old)).N_Learned + 1;
         for K in 0 .. Chan.Per_Arm - 1 loop
            if Se.Agree (K) < 0.0 then
               Se.Agree (K) := C.Tables (Natural (Old)).Agree (K);
            end if;
         end loop;
         if Beat = 0 then
            Se.When_Beat := C.Tables (Natural (Old)).When_Beat;
         end if;
      else
         Se.N_Learned := 1;
      end if;
      if not Has_Pose and then Old >= 0 then
         Se.Pose := C.Tables (Natural (Old)).Pose;      --  没带位姿的更新:沿用这张表原来量的位姿
         Se.Has_Pose := C.Tables (Natural (Old)).Has_Pose;
      end if;
      if I >= 0 then
         C.Tables.Replace_Element (Natural (I), Se);
      else
         C.Tables.Append (Se);
      end if;
   end Store_Effect;

   --  感觉:按此刻的位姿,从身体图算出每只手在每台(不长在它上面的)相机里的两瓣位置 —— 不看画面。
   --  Known = 离最近的真看过的样本不超过一步核实过的步幅;超了就是生地,走过去要看一眼(抖手指)。
   procedure Feel (C : in out Context; F : Plug.Frame) is separate;

   --  重新定位一个点:握区靠光流平流(世界相机)/固定(自己的手上相机);世界块重切后就近对上
   procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is separate;

   --  发一步并等稳;返回实到(通道)
   --  Geo_Settle:按位姿走的几何动作(不看画面)⇒ 到了目标一步看得见的那一档以内连着两拍就算到,没到就按"每拍挪不到这条命令的百分之一"算停
   --  (Selfmap.Go 的 Tol / Tol_Rot = 这只手平移、转动各自一步看得见的那一档,量的);看着画面走的那几种照旧等读数完全不动
   procedure Step_Arm (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; A : Table.Vec; Jaw : Floats;
                       Delivered : out Table.Vec; Ok : out Boolean; Press : Boolean := False; Watch : Selfmap.Watcher := null;
                       Geo_Settle : Boolean := False) is
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Frames : Natural;
      K : constant Natural := Arm * C.Map.Per_Arm;
      Tp : constant Long_Float := (if Geo_Settle and then K + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (K) else 0.0);
      Tr : constant Long_Float := (if Geo_Settle and then K + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (K + 3) else 0.0);
   begin
      Selfmap.Go (L, C.Map, Arm, Chan.Compose (P0, A), Jaw, F, Delivered, Frames, Ok, Press, Watch, Tol => Tp, Tol_Rot => Tr);
   end Step_Arm;

   --  没有表的点(同一只手的几个点一起):每个通道推一下量一列。幅度从开机看得见的那一档起,翻倍到每个点在画面里
   --  跑过 4 个跟踪地板、或深度变过深度地板为止(倍数,无量纲;EF 实测:最小可见幅度量出的列全是噪声,解算据此拧手腕);
   --  深度地板 = 这一点连着两拍读深度抖多少的 4 倍,再小也有距离的百分之一(比例,无量纲);翻到上限还看不出动的通道,这一段不用它。推完推回起点。
   --  🔴 身体【为了量自己而动】的幅度,不许超过【脑让它动】的幅度(Allow = 脑说的 small/medium/large 那一档)。
   --  原来这里一路加码到自己那一档的十几倍,于是每次量身体、每次中途重量,关节都被推到 0.4 弧度去甩一下 ——
   --  owner 看 JA 视频的原话:"机械臂全程在发癫"。这不是给某个任务定的数,是一条规矩:
   --  擦桌子的时候你也不会想让它为了标定自己甩胳膊。量不出来就【老实说这个方向量不到】,
   --  要不要给更大的幅度去量,是脑的事。
   procedure Probe_Effects (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                            Effs : in out Effect_Array; Trust : out Table.Mask; Ok : out Boolean;
                            Allow : Long_Float := 1.0) is separate;

   function Amount_Factor (A : Unbounded_String) return Long_Float is
     (if A = "small" then 0.25 elsif A = "large" then 1.0 else 0.5);   --  探针上限的几分之几(比例,无量纲)

   --  握区的点展开成瓣点(两瓣时):每一瓣各自到位,目标 = 区目标 + 这一瓣相对区心的偏移(保持此刻的开合朝向);
   --  歪了就有一瓣不到位,倾斜不用规则自然被罚。自己的手上相机里握区是固定像素,不展开。
   procedure Expand_Lobes (C : Context; F : Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is separate;

   --  ── 一段 = 反复做五件事:①打算怎么走 ②走 ③看 ④学 ⑤判 ──
   --  每件事一个小过程;这一步发生了什么全记在 Note 里(字段名就是人话),五件事之间只靠它说话。
   --  🔴 "到了没到"只有脑能判(owner 2026-09-14)。身体只报量到的事件,不报"我觉得到了"。
   --  身体唯一能结束一节的理由,是脑写的那个 until(含它给的步数),外加"我物理上做不到"。
   procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                          Until_Kind : Monitor.Until_Kind;
                          Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean; Beats : out Natural) is separate;

   --  合/张:最多 Max_Iter 拍,或到画面不再变;Sweep_Cam >= 0 时把那台相机里动过的像素累进 Sweep(手指自己扫过的地方)
   procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm, K : Natural; Target : Long_Float; Max_Iter : Natural;
                        Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is separate;

   --  合/张到读数不再变
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float;
                       K : Natural := 0) is
      None : Bools;
   begin
      Jaw_Sweep (L, C, F, Arm, K, Target, 40, -1, None, Steps, Reading);
   end Move_Jaw;

   --  生地/大步之后在世界相机里重新看见自己:手指 = 抖一下手指(合几拍再张回来),零件 = 推一下它自己的通道再推回来;
   --  动过的像素就是它,每个点认离预测最近的那团。抖的幅度不是常数:手指合"量出来的稳定拍数"那么久;零件推开机看得见的那一档。认不到的留预测、记 Lost。
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is separate;

   --  握住了没:抬一小截,看东西跟不跟我走。手上相机里 = 它的块还在握区框里;世界相机里 = 它原来那块地方空了。读数不算数(回声)。
   --  Sure = 有没有【不跟着手动的相机】能核实。没有就只能说"我说不准",不许把状态记成"手里有东西"
   --  一台不长在这只手上的相机(优先不长在任何手上的那台):判"夹住没有"只能靠它
   function Still_Cam (C : Context; F : Plug.Frame; Arm : Natural) return Integer is
   begin
      for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
         if Cam_Arm (C, Cm) < 0 then
            return Integer (Cm);
         end if;
      end loop;
      for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
         if Cam_Arm (C, Cm) /= Integer (Arm) then
            return Integer (Cm);
         end if;
      end loop;
      return -1;
   end Still_Cam;

   procedure Held_Test (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Cam : Natural; Origin : Picture.Region;
                        Slot : Integer;
                        Obj_Count : Natural; Held : out Boolean; Sure : out Boolean; Note : out Unbounded_String) is separate;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

   --  编译器要知道的、关于每个名词的事实。编号和给脑看的清单一致(1 起),0 号空着。
   function Cw_Of (C : Context; F : Plug.Frame) return Natural is
     (if C.Cam < Natural (F.Cams.Length) then F.Cams (C.Cam).W else 1);
   function Ch_Of (C : Context; F : Plug.Frame) return Natural is
     (if C.Cam < Natural (F.Cams.Length) then F.Cams (C.Cam).H else 1);

   function Build_Facts (C : Context; F : Plug.Frame) return Plan.Facts_Vectors.Vector is separate;

   --  把 Sinew 的一段区间落成执行器内部那一小节。角色在这儿变成具体的那一块。
   --  离第 N 件东西(1 起)近的那条臂(0 起);说不出就 -1。它在哪只腕眼里被量到 ⇒ 那条臂;在不动的眼里 ⇒ 比它和各只手在那只眼里的画面距离
   --  (各只手在不动的眼里在哪,是开机合空时量出来的握区中心)。过渡规则:臂的活动范围量出来、候选按每条臂各排一遍之后,这一步整个删掉
   function Nearer_Arm (C : Context; N : Natural) return Integer is separate;

   procedure Fill_Say (C : in out Context; I : Sinew.Instr; Answer : out Brain.Say) is separate;

   --  身体报的那句事件,归到八个结局里的哪一个。控制流只认这八个。
   function Classify (Event : String) return Sinew.Outcome is
      use Sinew;
      function Has (P : String) return Boolean is
        (Event'Length >= P'Length and then Event (Event'First .. Event'First + P'Length - 1) = P);
   begin
      if Has ("amount: arrived") or else Has ("amount: already there") then
         return Oc_Arrived;
      elsif Has ("contact") then
         return Oc_Touched;
      elsif Has ("resist") or else Has ("amount: stopped getting closer") then
         return Oc_Stuck;
      elsif Has ("slip") then
         return Oc_Slipped;
      elsif Has ("settle") then
         return Oc_Settled;
      elsif Has ("lost") then
         return Oc_Lost;
      elsif Has ("free") then
         return Oc_Free;
      elsif Has ("steps") then
         return Oc_Timeout;
      end if;
      return Oc_Refused;
   end Classify;

   procedure Geo_Say (S : String) is
      H : constant Integer := Lockstep.Current_Hand;   --  几只手按拍对齐时:哪只手说的(PLAN ⑧ (g))
   begin
      Put_Line ("[身] 📐 " & (if H >= 0 then "〔手" & Codec.Img (Natural (H) + 1) & "〕" else "") & S);
   end Geo_Say;

   function Geo_Of (C : Context; Cam : Natural) return Geom.Cam_Geo is
     (if Cam < Natural (C.Geo.Length) then C.Geo (Cam) else Geom.No_Geo);

   --  这台相机的几何能不能开工:焦距 + 指尖都有(朝向没有可以现量)
   function Geo_Ready (C : Context; Cam : Natural) return Boolean is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   begin
      return G.Tip_Valid and then G.F > 0.0;
   end Geo_Ready;

   --  长在这条胳膊上、几何常数齐的那只眼(量得出东西有多远的那只);没有 ⇒ -1
   function Hand_Eye_Of (C : Context; Arm : Integer) return Integer is
   begin
      if Arm >= 0 then
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            if Cam_Arm (C, Cm) = Arm and then Geo_Ready (C, Cm) then
               return Integer (Cm);
            end if;
         end loop;
      end if;
      return -1;
   end Hand_Eye_Of;

   --  点名的那块此刻在这台相机里的像素(这一帧还没切过就切一遍、槽号对上)
   procedure Geo_Track (C : in out Context; F : Plug.Frame; Cam : Natural; Slot : Integer; U, V : out Long_Float; Seen : out Boolean;
                        Name : Unbounded_String := Null_Unbounded_String) is separate;

   --  这只眼转过/挪过之后,点过名的那件东西在画面里【该】到哪:把它的世界位置(或它所在的方向上的一点)投进此刻的位姿,
   --  把重量的窗挪过去(大小不变)。不挪的话窗还留在转眼前的地方,里面是别的东西
   --  (H33 2026-09-22 实测:转 0.4 rad 把它整个看进来,旧窗里剩下的是黑手指 ⇒ "明暗反了,不是它" ⇒ 看丢 ⇒ 十轮一步不走)。
   procedure Retarget_Box (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P : Geom.V3) is separate;

   --  只平移(世界系),不转
   --  🔴 这里的量全是【米】。09-20 搬回来时为了不碰棘轮把"×1000"删了,标签却还写着 mm ⇒ 横挪 25.6 毫米显示成 "0.0 mm",
   --  "它在相机前 -0.8 mm"其实是负 0.8 米(算到相机背后去了)—— T10 2026-09-21 差点被这个标签骗过去。量的是米,就按米说,三位小数到毫米。

   --  走一步(Selfmap.Step,I6):这只手的目标 = 此刻的读数平移 Dw,这一步走它的 Frac、最长 Track(Selfmap.Step 的上限),一条命令、等它停
   --  (没给 Watch ⇒ 到了一步看得见的那一档以内就算到,同 Step_Arm 的 Geo_Settle);Rep = 这一步的账(实到、到没到、挡没挡:Blocked_By 拿 Wk 里这一段空走的底)
   procedure Geo_Move (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Dw : Geom.V3; Ok : out Boolean;
                       Wk : in out Selfmap.Walk; Rep : out Selfmap.Leg_Step;
                       Watch : Selfmap.Watcher := null; Press : Boolean := False;
                       Frac : Long_Float := 1.0; Track : Long_Float := Long_Float'Last) is
      A : Table.Vec := Table.Zero_Vec;
      Seq0 : constant Natural := F.Seq;
      Legs : Selfmap.Leg_Vectors.Vector;
      Rs : Selfmap.Leg_Step_Vectors.Vector;
      Frames : Natural;
      Lim : Selfmap.Limits;
   begin
      Rep := (others => <>);
      if Arm >= Natural (F.EE.Length) then
         Ok := False;
         return;
      end if;
      A (0) := Dw (0); A (1) := Dw (1); A (2) := Dw (2);
      Legs.Append (Selfmap.Leg'(Arm => Arm, Goal => Chan.Compose (F.EE (Arm), A), Jaw => <>));
      Lim.Press := Press; Lim.Watch := Watch; Lim.Loose := Selfmap."=" (Watch, null); Lim.Frac := Frac; Lim.Track := Track;
      Selfmap.Step (L, C.Map, Legs, Lim, F, Wk, Rs, Frames, Ok);
      if not Rs.Is_Empty then
         Rep := Rs (0);
      end if;
      --  命令了多少、实到多少,每一步都说(GB5 那一版有这一行,搬回 main 时丢了;H6 2026-09-22 实测每步要 14 cm 而差距只缩 0–2 cm,
      --  没有这一行就分不清是身体没走成、还是我算错了)。拍号 = 这一下起止那两帧的帧号(同 poses.txt / fk_poses.txt / joints.txt 的第一列,
      --  离线按仿真真值给每一下压标"碰没碰到"用;09-29 V1B65 按位移反推拍号一半对不上)
      Geo_Say ("挪 (" & Mm (Rep.Cmd (0)) & "," & Mm (Rep.Cmd (1)) & "," & Mm (Rep.Cmd (2)) & ") ⇒ 实到 (" & Mm (Rep.Got (0)) & "," & Mm (Rep.Got (1)) & "," & Mm (Rep.Got (2)) &
               "),差 " & Mm (Geom.Norm ([Rep.Cmd (0) - Rep.Got (0), Rep.Cmd (1) - Rep.Got (1), Rep.Cmd (2) - Rep.Got (2)])) & (if Ok then "" else " · 身体说没走成")
               & (if Rep.Blocked_T then " · 被挡住(比这一段空走时少走得多)" else "")
               & " · 拍 " & Codec.Img (Seq0) & "→" & Codec.Img (F.Seq));
   end Geo_Move;

   procedure Geo_Move (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Dw : Geom.V3; Ok : out Boolean;
                       Watch : Selfmap.Watcher := null; Press : Boolean := False) is
      Wk : Selfmap.Walk;
      Rep : Selfmap.Leg_Step;
   begin
      Geo_Move (L, C, F, Arm, Dw, Ok, Wk, Rep, Watch, Press);
   end Geo_Move;

   --  "上"只写在这一处:位姿系的 +z 是协议约定的重力反方向(观测里没有重力读数的身体只能这么约;有加速度计的身体应把它换成量出来的)。
   --  碰过面之后"上"= 那张面的法向(量出来的)
   Protocol_Up : constant Geom.V3 := [0.0, 0.0, 1.0];
   function Up_Dir (C : Context) return Geom.V3 is (if C.Touch_Valid then C.Touch_N else Protocol_Up);

   --  ── 东西的量(登记表,大并行 §2 第 16 条)──:脑的句子:do <东西> <量> up|down until <结局>(两件东西那一句的关系词也按量算,见 Contact.Qty)。
   --  量的名字由身体列(键盘上"量 [...]"那一栏);每个量 = 一种量法(Contact.Qty.Kind),它往哪变 = 让它变得最快的那个刚体运动(一个旋量,从量到的几何算)
   --  ⇒ 同一条接触集 + 执行层。加一个量 = 这里加一行;句子、接触集、执行层都不动,不按任务分。
   --  一张表:名字、量法、给脑看的那句"我怎么量它"(Qty_Meaning;路 7 的 Qty_Gloss 从这儿取)
   function Qty_Kind (Name : String; K : out Contact.Qty.Kind) return Boolean is
   begin
      K := Contact.Qty.Height;
      if Name = "height" then
         K := Contact.Qty.Height;
      elsif Name = "heading" then
         K := Contact.Qty.Heading;
      elsif Name = "tilt" then
         K := Contact.Qty.Tilt;
      elsif Name = "away" then
         K := Contact.Qty.Away;
      else
         return False;
      end if;
      return True;
   end Qty_Kind;
   function Qty_Meaning (Name : String) return String is
     (if Name = "height" then "how far the thing is above the surface it lies on (I measure it with my own eyes; up = off that surface, down = back onto it)"
      elsif Name = "heading" then "which way the thing's long side points along the surface it lies on (up rotates it counterclockwise seen from above that surface, down clockwise)"
      elsif Name = "tilt" then "how far the thing leans from how it stands (up leans its top away from my still eye, down toward it)"
      elsif Name = "away" then "how far the thing is from my still eye, measured along the surface it lies on (up = farther, down = nearer)"
      else "a reading of it I can change");
   --  键盘上列哪几个:有能合拢的部件(grasper)才列。heading 要它的长轴(轮廓量得出);tilt / away 要一只"不跟着动它的那条臂走"的眼 ——
   --  按开机量的"每只眼长在哪条臂上"判(Cam_Arm):有手指的臂里有一条臂,有一只量过几何的眼不长在它上面,就列(用到哪条臂时缺了照实说)
   function Qty_Words (C : Context; Roles : String) return String is
      Still_Eye : Boolean := False;
   begin
      if Ada.Strings.Fixed.Index (Roles, "grasper") = 0 then
         return "";
      end if;
      for A in 0 .. C.Map.Arms - 1 loop
         if Arm_Has_Fingers (C, A) then
            for Cm in 0 .. C.Map.N_Cams - 1 loop
               if Cam_Arm (C, Cm) /= Integer (A) and then Cm < Natural (C.Geo.Length) and then (C.Geo (Cm).Fixed or else C.Geo (Cm).Valid) then
                  Still_Eye := True;
               end if;
            end loop;
         end if;
      end loop;
      return "height heading" & (if Still_Eye then " tilt away" else "");
   end Qty_Words;

   --  这一段用了几拍:对方在段中间复位(新的一集,步数从零起)时不许算成负数(S1 2026-09-23 实测:第二集开始时正在进场,减出负数把驱动崩了)
   function Beats_Since (L : Plug.Link; B0 : Natural) return Natural is
     (if Plug.Steps (L) >= B0 then Plug.Steps (L) - B0 else Plug.Steps (L));
   --  段中间对方复位了 ⇒ 这一段作废的那句话(所有走路的段共用)
   Reset_Event : constant String := "reset: the world was reset under me (a new episode began) - this segment is void";

   --  朝下被顶住的一点进地图。它躺的面 = 这一集里【最低】的那张(东西靠面抵住重力;比已知的面高的顶住是躺在面上的别的东西或它自己,
   --  H61 2026-09-23 实测:压到牛仔裤上把它当成剪刀躺的面,轮廓按高 4 cm 的面重投、落点挪了 4 cm、合空)。
   --  上一集留下的面第一次被顶住时直接换新(桌子可能换了);之后只让更低的换。"高出多少"按已知面的法向量,门槛 = 本体位置读数的抖动(量过的)
   procedure Note_Support (C : in out Context; P, N : Geom.V3; How : String) is separate;

   --  这条臂一条命令能走多远还走得到(米):开机按阶梯探出来的(存在它那只眼的几何记录里);0 = 没量 ⇒ 走路的段如实拒
   function Stride_Of (C : Context; Arm : Natural) return Long_Float is
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
   begin
      if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) then
         return C.Geo (Natural (Hc)).Stride;
      end if;
      return 0.0;
   end Stride_Of;

   --  这只手一步能走出来又看得见的那一档(开机量的,米)
   function Geo_Base (C : Context; Arm : Natural) return Long_Float is
      K : constant Natural := Arm * C.Map.Per_Arm;
   begin
      if K < Natural (C.Map.Amp.Length) and then C.Map.Amp (K) > 0.0 then
         return C.Map.Amp (K);
      end if;
      return C.Map.EE_Noise;
   end Geo_Base;

   --  指尖在相机里的位置:开机那一帧里两根手指(合空扫过的像素)各自最靠上的那一截 = 指尖;有深度那一帧读一次深度
   --  (真机:一台相机一辈子量一次,用尺子也行;之后再也不读深度)

   procedure Geo_Install (F : Plug.Frame; C : in out Context; Body_Path : String; Geo : Geom.Geo_Vectors.Vector; Board : Geom.Scene_Pt_Vectors.Vector;
                          Plane_Pt, Plane_N : Geom.V3; Plane_Rms : Long_Float; Ref : Plug.Cam; Keep_Tips : Boolean := False) is separate;

   --  几何逼近:让"指尖该到的那一点"(指尖中点再往手心里一点)和点名那块重合。每段走一截、停稳、再看一眼、再算。
   --  这一槽里的东西此刻看得【全不全】,以及它叫什么(点过名的才有名字)。看不全(顶到窗边/被画面切掉)的那一眼,形心不是同一个物理点。
   --  Whole = 这一眼量到的是一整块(没被【画面边】切掉;挨着邻居的已在框里量时裁掉,形心照用)。
   --  Edge = 它被画面边切掉了 ⇒ 转一下眼把它整个看进来,这一眼才算数。
   procedure Slot_Whole (C : Context; F : Plug.Frame; Cam : Natural; Slot : Integer; Whole, Edge : out Boolean; Name : out Unbounded_String;
                         Named : Unbounded_String := Null_Unbounded_String) is separate;

   --  指尖此刻在世界里的位置:手的位姿读数 + 量过的指尖偏置(在这只手自己那台相机的坐标里)转到世界
   function Tip_World (C : Context; Arm : Natural; P : Plug.Arm_Pose) return Geom.V3 is
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
   begin
      if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then C.Geo (Natural (Hc)).Tip_Valid then
         declare
            Gw : constant Geom.Cam_Geo := C.Geo (Natural (Hc));
            Tw : constant Geom.V3 := Geom.Ap (Geom.Cam_R (Gw, P), Gw.Tip);
            Cp : constant Geom.V3 := Geom.Cam_Pos (Gw, P);   --  指尖偏移是从相机中心量的
         begin
            return [Cp (0) + Tw (0), Cp (1) + Tw (1), Cp (2) + Tw (2)];
         end;
      end if;
      return [P (0), P (1), P (2)];
   end Tip_World;

   --  RGB 图顺时针转 90°(W×H → 宽 H、高 W):新图 (x', y') = 原图 (y', H − 1 − x');原图的 (u, v) 落到新图的 (H − 1 − v, u)
   function Turn_90 (Img : Buf; W, H : Natural) return Buf is
      R : Buf;
   begin
      for Y2 in 0 .. W - 1 loop
         for X2 in 0 .. H - 1 loop
            declare
               K : constant Natural := ((H - 1 - X2) * W + Y2) * 3;
            begin
               R.Append (Img (K)); R.Append (Img (K + 1)); R.Append (Img (K + 2));
            end;
         end loop;
      end loop;
      return R;
   end Turn_90;

   --  原图(宽 W、高 H)顺时针转了 Turns 个 90° 之后那张图里的 (U, V),换算回原图。坐标是连续的像素坐标 —— 配点仪器的约定:下标 i 的像素占 [i, i + 1)、
   --  中心在 i + 0.5(开机钉的主点 = 画幅中心 W / 2 就是这个约定)⇒ 每退一步,转之前高 h 的图里 (u, v) = (v', h − u')。
   --  09-27 查出:原来按下标写成 h − 1 − u',每退一步错 1 px —— 参考图配它自己转 90° 的那张,板上 714 个点按 h − u' 误差中位 0.245 px、按 h − 1 − u' 0.883 px;
   --  被转以后重标的那份整片偏 1 px 左右(按真值中位 1.15–1.39 px,开机那份 0.30);开机量的"转着看的配点噪声"也按它算,0.8 px 大半是这 1 px
   procedure Unturn (U, V : Long_Float; Turns, W, H : Natural; U0, V0 : out Long_Float) is
      Uc : Long_Float := U;
      Vc : Long_Float := V;
   begin
      for T in reverse 1 .. Turns loop
         declare
            H_Before : constant Natural := (if T mod 2 = 1 then H else W);   --  第 T 步转之前那张图的高:奇数步前是原图朝向
            U1 : constant Long_Float := Vc;
            V1 : constant Long_Float := Long_Float (H_Before) - Uc;
         begin
            Uc := U1; Vc := V1;
         end;
      end loop;
      U0 := Uc; V0 := Vc;
   end Unturn;

   procedure Board_Save (C : Context) is
      Base : constant String := To_String (C.Geo_Path);
      Fo : Ada.Text_IO.File_Type;
   begin
      if Base = "" or else C.Board.Is_Empty or else C.Fixed_Ref.Is_Empty then
         return;
      end if;
      Codec.Write_BMP (Base & ".board_ref.bmp", C.Fixed_Ref, C.Fixed_Ref_W, C.Fixed_Ref_H);
      Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Base & ".board.txt");
      Ada.Text_IO.Put_Line (Fo, "board " & Codec.Img (Natural (C.Board.Length)) & " " & Codec.Img (C.Fixed_Best.All_N) & " " & Codec.Fmt (C.Fixed_Turn_Sd, 4));
      --  点数、核对时对得上最多的那次(挡没挡按它比)、转着看的配点噪声(像素)
      for S of C.Board loop
         Ada.Text_IO.Put_Line (Fo, Codec.Fmt (S.Pw (0), 6) & " " & Codec.Fmt (S.Pw (1), 6) & " " & Codec.Fmt (S.Pw (2), 6) & " " & Codec.Fmt (S.U, 3) & " " & Codec.Fmt (S.V, 3)
                               & " " & Codec.Fmt (S.Sh, 4) & " " & Codec.Img (S.Views) & " " & Codec.Fmt (S.Cov (0, 0) * 1.0e6, 6) & " " & Codec.Fmt (S.Cov (0, 1) * 1.0e6, 6)
                               & " " & Codec.Fmt (S.Cov (0, 2) * 1.0e6, 6) & " " & Codec.Fmt (S.Cov (1, 1) * 1.0e6, 6) & " " & Codec.Fmt (S.Cov (1, 2) * 1.0e6, 6)
                               & " " & Codec.Fmt (S.Cov (2, 2) * 1.0e6, 6));   --  协方差按 mm² 存(单位换算)
      end loop;
      Ada.Text_IO.Close (Fo);
   exception
      when others => null;
   end Board_Save;

   procedure Check_Fixed_Eye (F : Plug.Frame; C : in out Context) is separate;

   --  板上的点在不动的眼此刻的画面里重找一遍(2026-09-28 V1B47):参考图往返配(同核对不动的眼,配过去再配回来 1 px 以内算找到),
   --  找不到的 = 那儿被挪来的东西盖住了、或者此刻被手挡着 ⇒ 挑空地时不算量过的桌面(Board_Free_Spots)。
   --  画面按核对定下的转法先转正再配(同核对)。只管"还找不找得到",不按位姿判:相机挪过时点照样找得到,板的世界位置不跟着变
   procedure Board_Recheck (F : Plug.Frame; C : in out Context; Found : out Natural; Said : out Unbounded_String) is separate;

   --  ── 接触集接线(PLAN.md 1.5)──:身体量的数全从这只眼和握区来,一个字面量都没有;哪儿夹得住由 Contact.Gen 从形状里算,不由我挑

   --  它躺的面过哪一点:碰过它躺的面之后,就是那个面上离 P 最近的点(当它厚度为零)。量到的位置 P 的高度是视线交出来/挪眼估出来的,
   --  两条视线都近乎竖直时深度病态(H53:桌面之下 9–28 cm),而腕眼近乎竖直向下看时,面的高度错 5 cm 就把整片轮廓横着投歪 5 cm
   --  (H58/H59 2026-09-23 实测:落点准到 1 mm 却合空;第三把又把估高了 3.3 cm 的面当成"量到的厚度")。零厚度只错它自己的厚度那么一点。没碰过面就只能信 P
   function Plane_Point (C : Context; P, N : Geom.V3; Say : Boolean) return Geom.V3 is
      H : Long_Float;
   begin
      if not C.Touch_Valid then
         return P;
      end if;
      H := (P (0) - C.Touch_Pt (0)) * N (0) + (P (1) - C.Touch_Pt (1)) * N (1) + (P (2) - C.Touch_Pt (2)) * N (2);
      if Say and then abs H > C.Map.EE_Noise then
         Geo_Say ("它量到的位置" & (if H < 0.0 then "在我碰过的面之下 " & Mm (-H) else "高出我碰过的面 " & Mm (H)) & " ⇒ 当它贴在那个面上(它躺在面上;厚度当零,只错它自己那么厚)");
      end if;
      return [P (0) - H * N (0), P (1) - H * N (1), P (2) - H * N (2)];
   end Plane_Point;

   --  这只眼这一帧看全了它、它的位置又是两眼交出来的 ⇒ 轮廓像素各发一条视线,落到它躺的面(过它的位置,法向 = 碰过的面的法向,没碰过按上)上,
   --  记成它顶面的点。每次看全都重记(最新的一份离得最近、最准)。像素多就隔几个取一个(采样密度是可观测性参数),最多约 Keep 个;
   --  我自己的手指像素(握区量过的)不算
   --  P0 = 它此刻的位置估计:两眼交点最好;只有一只眼时是"我自己挪过的几眼"算出来的(H55 2026-09-23 实测:头顶眼没认出它,整炮只有腕眼看见它,
   --  两眼交点一次都没有 ⇒ 只认交点就永远没有轮廓)。单眼估计只用在这一段里,取的点碰到面之后还会按真高度重投(视线存着)
   procedure Take_Silhouette (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P0 : Geom.V3; P0_Up_Sd : Long_Float) is separate;

   --  接触集(09-29 重写,PLAN §2 ②):量出来的手(每一瓣的尖和尖那一截的截面,碰桌面量的)在看到的形状上真合一次,
   --  挑按量得出的误差最坏时每单位重量要夹得最松的那一组(Contact.Grasp.Plan)。
   --  形状 = 记下的顶面点(Take_Silhouette;碰过它躺的面就按真高度重投)+ 从轮廓那一圈垂直补到它躺的面的侧壁(实心、竖壁的假设,说出来);
   --  旁边的东西 = 这一集里被顶住过、比面高、又不在它身上的点(C.Bumps:伸下去被挡住就记进来 —— 试一下就知道);
   --  够不够得着 = 眼在那个位姿时按量到的关节范围反解(Plug.Reach);摩擦 = 这件东西和这只身体以前量到的上下限(C.Grip_Mus)。
   --  Pick = 挑中的那一组:下手那一刻眼的朝向 R、位置 T、进场方向、每一块先合多少(Pre)、接触
   function Plan_Descent (Stand, Tip_Over, Sil_Err, H_Sd, An, Tip_Sd, Miss, Noise, Floor : Long_Float) return Descent is
      D : Descent;
      Hand : constant Long_Float := Stats.Z * Sqrt (Tip_Sd ** 2 + Miss ** 2 + Noise ** 2);   --  手自己的不准
      E_Top : constant Long_Float := Stand - Long_Float'Max (0.0, Tip_Over);   --  照量到的,走这么远尖碰到它顶面那一层
   begin
      D.Lstep := Long_Float'Max (Hand / Long_Float (Selfmap.Free_Base), Floor);
      if H_Sd = Long_Float'Last then
         --  它顶面高低量不出:没有"碰不到它"的那一段,全程小步探(照实说)
         D.Band := Long_Float'Last; D.Fast := 0.0; D.Fine_End := Stand;
         return D;
      end if;
      D.Band := Stats.Z * Sqrt (Sil_Err ** 2 + (An * H_Sd) ** 2 + Tip_Sd ** 2 + Miss ** 2 + Noise ** 2);
      --  带子之前碰不到它 ⇒ 一条命令下到带子前、再留出 Blocked 当底的那几步;小步探过带子为止
      D.Fast := E_Top - D.Band - Long_Float (Selfmap.Free_Base) * D.Lstep;
      D.Fine_End := Long_Float'Min (Stand, E_Top + D.Band);
      return D;
   end Plan_Descent;

   --  手拿着它绕 M 的那根轴(过 M.Pivot)转 Th 弧度,手的位姿要到哪:位置绕那一点转过去、朝向转同一个角(Chan.Compose 的转动按世界轴)。
   --  拿住了它就跟着手走 ⇒ 它身上每一点正好绕那根轴转了 Th(Want_Scene 按"手从合上那一刻起挪过的刚体运动"搬它,同一个变换)
   function Carry_Goal (Cur : Plug.Arm_Pose; M : Contact.Twist; Th : Long_Float) return Plug.Arm_Pose is
      Oa : Boolean;
      Ax : constant Geom.V3 := Contact.Unit (M.Ang, Oa);
      Rv : constant Geom.V3 := [Th * Ax (0), Th * Ax (1), Th * Ax (2)];
      Rq : constant Geom.V3 := Geom.Ap (Geom.Rodrigues (Rv), [Cur (0) - M.Pivot (0), Cur (1) - M.Pivot (1), Cur (2) - M.Pivot (2)]);
      A : Table.Vec := Table.Zero_Vec;
   begin
      for I in 0 .. 2 loop
         A (I) := M.Pivot (I) + Rq (I) - Cur (I);
         A (3 + I) := Rv (I);
      end loop;
      return Chan.Compose (Cur, A);
   end Carry_Goal;

   --  它躺的面(接触集和"要它怎么动"共用这一份):碰过的面 ⇒ 量到的;没碰过、标定板拟合出了面 ⇒ 板的;都没有 ⇒ 协议的"上"(过原点)
   function Lie_N (C : Context) return Geom.V3 is (if C.Touch_Valid then C.Touch_N elsif C.Board_Plane then C.Board_N else Protocol_Up);
   function Lie_P (C : Context) return Geom.V3 is (if C.Touch_Valid then C.Touch_Pt else C.Board_Pt);
   --  它的实心模型(接触集和"要它怎么动"共用这一份,一个量一种量法):记下的顶面轮廓点;碰过它躺的面 ⇒ 按真的面重投那些视线
   --  (一条都没落到面上 ⇒ 还用原来那份);再从轮廓那一圈往下补到它躺的面(实心、竖壁的假设,说出来)。没记下它的轮廓 / 没量过它躺的面 ⇒ 空
   procedure Solid_Of (C : Context; Name : Unbounded_String; Shape : out Contact.V3_Vectors.Vector; Reprojected : out Boolean) is separate;

   procedure Plan_Contact (C : in out Context; F : Plug.Frame; Arm, Cam : Natural; Name : Unbounded_String;
                           Pick : out Contact.Grasp.Candidate; Note : out Unbounded_String; Ok : out Boolean) is separate;

   --  转这只手,让它自己那只眼的正前方对准世界里的一个方向(Want,单位向量)。
   --  转最少的角度:转轴 = 现在的正前方 × 要的方向。指尖不许甩走(08-28 那次甩出 20 cm):每一步先按要转的角度算出
   --  指尖会挪到哪,再用平移把它补回原处 —— 指尖偏置是量过的。一条命令最多转多少 = 开机量出来的"一条命令转得到的最大一档"(G.Stride_Rot)× 脑的档位;
   --  转了没转到(不到一半)⇒ 如实说,不硬转。到位的判据 = 差不到一个转动探针幅度(身体量过的最小一档)。
   --  Along:我身上要拿去对准的那根方向(在这只眼的坐标里):默认是眼的正前方 [0,0,-1];要让【手指】指向某处就传指尖方向(G.Tip 归一化)。
   procedure Geo_Turn (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Want : Geom.V3;
                       Amt : Long_Float; Event : out Unbounded_String; Steps_Taken : out Natural;
                       Along : Geom.V3 := [0.0, 0.0, -1.0]) is separate;

   --  不动的眼看见了它,长在手上的眼还没看见 ⇒ 把那只眼转向它:不动的眼的视线 ∩ 它躺着的面 = 它在哪(面 = 我最后碰过的那个面;
   --  一次都没碰过就拿指尖此刻的高度当面,并说出来),再让手眼的正前方指向那一点。
   procedure Aim_Eye_At (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Fixed_Cam : Natural;
                         U, V : Long_Float; Amt : Long_Float; Event : out Unbounded_String; Ok : out Boolean) is separate;

   --  这只眼里我正走路的那只手压在它上面/挨着它 ⇒ 这只眼此刻"量到的它"多半是我的手和手的影子。
   --  (H36 2026-09-22 实测:手越靠近,头顶眼的框里越是手影,两眼交点从 z=0.661 一路掉到 0.503 —— 桌面之下 15 cm,偏差却只有 1 cm,看着很准)。
   --  手在那只眼里的位置是身体图按此刻位姿算的,不看画面。近到一个框之内就算压着
   function Hand_Covers (C : Context; F : Plug.Frame; Arm, Cm : Natural; B : Boxed_Thing) return Boolean is
      Ti : constant Natural := Track_Idx (C, Arm, Cm);
      Pu : constant Long_Float := B.Cu * Long_Float (F.Cams (Cm).W);
      Pv : constant Long_Float := B.Cv * Long_Float (F.Cams (Cm).H);
      Bw : constant Long_Float := Long_Float (B.X1 - B.X0 + 1);
      Bh : constant Long_Float := Long_Float (B.Y1 - B.Y0 + 1);
      function Within (Hu, Hv : Long_Float) return Boolean is
        (abs (Hu * Long_Float (F.Cams (Cm).W) - Pu) < Bw and then abs (Hv * Long_Float (F.Cams (Cm).H) - Pv) < Bh);
   begin
      if Ti < Natural (C.Zones.Length) and then C.Zones (Ti).Valid then
         declare
            Tr : constant Zone_Track := C.Zones (Ti);
         begin
            return Within (Tr.Cu, Tr.Cv) or else (Tr.Has_Lobes and then (Within (Tr.Au, Tr.Av) or else Within (Tr.Bu, Tr.Bv)));
         end;
      end if;
      return False;
   end Hand_Covers;

   --  同一件东西在两只眼里被脑起了不同的名字(H55/H58 2026-09-23:头顶眼里叫「the mint green」、腕眼里叫「scissors」)⇒ 两眼视线永远配不成对、没有交点,
   --  只剩单眼估计,窗口跟丢后落到别的亮东西上(H58 举起了机器人跟前的小白块,剪刀原封不动)。同一性是量得出来的:这只眼到这块的视线,和别的眼到它那块的
   --  视线交在一点(偏差不超过那块东西自己的一半大)⇒ 同一件,名字统一到脑现在的叫法 —— 和"按字面包含统一"是同一条规矩的另一半:那条看字,这条看视线
   procedure Unify_By_Sight (C : in out Context; F : Plug.Frame; Cam : Natural; W : String; R : Picture.Region) is separate;

   --  这只眼里没有它的窗(脑没在这只眼里点过它的名),可它顶面的点我量过 ⇒ 把那些点投进这只眼,外接框就是窗;窗里哪一片是它照常每帧重量。
   --  名字是脑起的、点是我量的:这不是替脑认东西,是把量到的东西送进另一只眼(换手之后左手的眼里本来什么都没有)
   procedure Window_From_Outline (C : in out Context; F : Plug.Frame; Cam : Natural; Name : Unbounded_String) is separate;

   --  ── 此刻每一只看得见它的眼给一条视线 ──(它叫 Its_Name,脑点过名的)
   --  眼可以是:正在走路的这只手自己的眼(Seen 且整块)、不动的眼(量过自己在哪)、另一只手的眼(朝向量过)。
   --  两条以上 ⇒ 交点就是它此刻的位置,它动不动都一样;这是抓会动的东西唯一诚实的量法(owner 09-22)。
   --  一只眼的视线有多不准(弧度,一倍标准差)= 它量朝向时的像素残差 ÷ 焦距;没量过 ⇒ 0(交点那一步就照实说量不出)
   function Ray_Sd (G : Geom.Cam_Geo) return Long_Float is (if G.F > 0.0 and then G.Rms > 0.0 then G.Rms / G.F else 0.0);

   function Sightlines_Now (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Its_Name : Unbounded_String;
                            Seen, Whole : Boolean; U, V : Long_Float; Who : out Unbounded_String; Sds : out Floats) return Geom.Sight_Vectors.Vector is separate;

   --  Above = True:不是走到它跟前,而是走到它【正上方、高出一个张口】(张口是身体量过的长度,不是拍的数)。
   --  "上" = 位姿读数系的 +z,和抬手那一条同一个约定(当它朝上;真机该由重力读数定)。
   procedure Geo_Approach (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Slot : Integer;
                           Step_Limit : Natural; Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural;
                           Above : Boolean := False; Amt : Long_Float := 0.5; Until_Touch : Boolean := False;
                           Name : Unbounded_String := Null_Unbounded_String) is separate;

   --  离远点(拿着东西):沿来的路退,退它来时那么远(全是量的,两段走)
   procedure Geo_Retreat (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
      Beats0 : constant Natural := Plug.Steps (L);
      Dist : constant Long_Float := C.Geo_Came;
      Mok : Boolean;
   begin
      Steps_Taken := 0; Beats := 0;
      if Dist <= 0.0 or else Geom.Norm (C.Geo_Dir) <= 0.0 then
         Event := S ("amount: stopped (I have no approach path to retrace)");
         return;
      end if;
      --  分几截退:除数【就是截数】,不是一个可调的系数 —— 走一截量一眼,免得一口气退过头。
      declare
         Legs : constant := 2;
         Leg_D : constant Long_Float := Dist / Long_Float (Legs);
      begin
      for Leg in 1 .. Legs loop
         Geo_Move (L, C, F, Arm, [-C.Geo_Dir (0) * Leg_D, -C.Geo_Dir (1) * Leg_D, -C.Geo_Dir (2) * Leg_D], Mok);
         Steps_Taken := Steps_Taken + 1;
      end loop;
      end;
      Event := S ("amount: arrived (I went back the way I came, " & Len (C, Dist) & ")");
      Beats := Beats_Since (L, Beats0);
   end Geo_Retreat;

   --  离远点(手里没东西):沿我来时走向它的方向【反着】走。一步多长 = 脑那一档的步子(和贴近时同一把尺);走几步 = 脑说的步数,没说就一步。
   --  没朝它走过就说不出哪边是"远" ⇒ 如实拒,不猜。
   procedure Geo_Away (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Step_Limit : Natural; Amt : Long_Float;
                       Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is separate;

   --  ── 一轮 ──
   --  量到的几何 → Contact.Qty.Scene。它:实心模型的形心、底离它躺的面多高、在面里的长轴(没有模型 ⇒ 两眼交点那个位置,没有长轴和底)。
   --  "上" = Up_Dir(和 09-23 起"沿面的法向走一个单位"那一条是同一个;高低从它躺的面 Lie_P 量);"我" = 不跟着这条臂走的那只眼(Still_Cam);
   --  "横" = 脑看着的那只眼的横轴;参照那一件 = 此刻看得见它的几只眼的视线交点(交点的高 = 它顶面的高:接触集的模型就是"顶面过它量到的位置")。
   --  不准:它轮廓横着的误差(Sil_Err)、它那张面高低的不准(Sil_H_Sd)、参照那一件交点沿"上"的不准(Meet_Sd)、位姿读数的抖动,合起来;
   --  长轴朝向的不准 = 轮廓点误差 × √(长轴方向的方差 / 点数) ÷ (长短两轴方差之差)(主轴的一阶扰动),Z 倍到不了直角 ⇒ 才算有长轴
   procedure Want_Scene (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; Sc : out Contact.Qty.Scene) is separate;
   --  这一个要 ⇒ 要它怎么动(一个旋量)。量的名字按登记表(Qty_Kind);两件东西那一句的关系词各是两件之间的一个量(Contact.Qty),方向由关系词定
   procedure Want_Twist (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; M : out Contact.Twist; Ok : out Boolean; Note : out Unbounded_String) is separate;

   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is separate;

   --  V1 口径"头顶眼按指尖算的残差"(2026-09-26):开机各停里不动的眼给这只手做的合空标记(每一瓣的尖 C.Lobe_Obs、各瓣的中点 C.Fixed_Obs),
   --  和"那一停的位姿 + 碰出来的指尖"投进它眼里的那一点比。只报数、不改任何量:两边都是量的(标记是分割出来的尖,指尖是碰出来的)
   procedure Head_Tip_Check (C : Context; A, Hc : Natural; Tips : Geom.V3_Vectors.Vector) is separate;

   --  板上空的面(开机碰桌面量指尖用):Lp = 这一下各瓣视线落在面上的点(第 0 个是朝下压的那一瓣);Tb = 每一瓣视线离朝下的角的一半的正切。
   --  整体平移 Delta 之后:压的那一瓣的落点那一圈(半径 R)整个在量过的桌面里(Inside),R 之内没有高出面的板点;
   --  别的每一瓣:它的手指沿自己的视线从眼往下伸,碰到面的那一刻压的那一瓣的尖在面上 —— 两根手指一样长时,离压的那一点水平 ρ 处
   --  这根手指离面至少 ρ·tan(β/2) 高(β = 它的视线离朝下的角;纯几何),所以它那条落点连线 R 之内、高出面超过这个高度的板点才挡它
   --  (V1B22 2026-09-27:原来连线旁边高出面一点点的板点都算挡,空的面挑到了 0.39 m 外)。
   --  躺在面上 / 高出面:离面在 / 超出 3 倍(倍数无量纲,同踢离群)"面内离散 ⊕ 这一点自己沿法向的不确定度"。
   --  量过的桌面 = 躺在面上、上回在不动的眼里重找时还找得到(C.Board_Seen;没重找过 = 按量的那一刻)的板点围成的那一片。
   --  09-28 V1B47:原来只要"落点 R 之内有一个躺在面上的板点",落在那片的边上也收 —— 边外是开机时手自己挡着、没量过的一块,
   --  那儿放着一台电子琴:手指压在琴上,还把琴推进了板上量过是桌面的那片,第 2 瓣接着压在琴上(按仿真真值这只手 14 下里 9 下碰的不是桌面)。
   --  高出面的板点重找时找没找到都照样挡(东西被挪走了也不知道挪到了哪)。压之前看见的高出面的点(C.Seen_Above)和它们一样挡。
   --  候选按先不挪、再按离压的那一点近排每个量过的桌面上的板点,返回全部空的(挑哪个由调用方按反解解不解得出来定)
   --  离面在 / 超出:Z 倍(Stats.Z,同踢离群)"面内离散 ⊕ 这一点自己沿法向的不确定度"(Sn = 沿法向的方差)。板点、压之前看见的点同一个门
   function Plane_Tol (C : Context; Sn : Long_Float) return Long_Float is (Stats.Z * Sqrt (C.Board_Rms ** 2 + Long_Float'Max (0.0, Sn)));

   procedure Board_Free_Spots (C : Context; Lp : Geom.V3_Vectors.Vector; Tb : Floats; R : Long_Float; Deltas : out Geom.V3_Vectors.Vector) is separate;

   function Look_Points (C : Context; G : Geom.Cam_Geo; P0 : Plug.Arm_Pose; W, H : Natural;
                         Spot : Geom.V3; Far_Ends : Geom.V3_Vectors.Vector; R, Step_Px : Long_Float) return Instrument.Match_Vectors.Vector is separate;

   --  压之前先看底下(09-30 V1B70 / V1B73):一对立体像里比面高出的点。配上 = 配到的落在画面里、配回来离问的那一点 Geom.Trip_Px 以内
   --  (同核对不动的眼、重找板点);配点噪声(每轴)= 这一批往返差的中位 ÷ 瑞利分布的中位(同板的);两条视线交出一点(Geom.Meet),
   --  按两个位姿投回两帧,四个像素残差合起来超过 Z 倍配点噪声 = 两条视线对不上(动着的东西、配错的)⇒ 不要。
   --  交成的点离面高出 Plane_Tol(同挑空地的"高出面";沿法向的方差按 Geom.Meet_Cov,每条视线的角度噪声 = 配点噪声 ÷ 焦距)⇒ Above。
   --  挨着眼平移方向的那一片视差小、远近定不住:它的方差大,门跟着宽,判不成高出面(不猜)
   procedure Seen_Above_Of (C : Context; G : Geom.Cam_Geo; P0, P1 : Plug.Arm_Pose; W, H : Natural; Qu, Qv, Mu, Mv, Bu, Bv : Floats;
                            Above : out Geom.Scene_Pt_Vectors.Vector; Matched, Tri : out Natural; Sig : out Long_Float) is separate;

   --  ③ 每只手:摸它下面的面,顺带量指尖(2026-09-26;09-28 改成换倾角碰,PLAN 开机后半段 ③)。
   --  标定板的点拟合过那张面(1 mm 级,Geo_Board)⇒ 指尖按碰量:每一瓣压 6 下,每一下让手上一个方向朝正下 —— 这一瓣指尖那条视线
   --  (Zone.Tip_Band:这一瓣自己那一块手指像素里离它贴画面边处最远的那一截)本身 1 下、朝方位各差 72° 的五个方向各斜 θ 1 下
   --  (θ = 这一瓣视线和最近的另一瓣视线夹角的三分之一:斜 θ 时另一根手指一直比它高,纯几何;只有一瓣 ⇒ 一条命令转得到的那一档,量过的),
   --  落到板上一块空的面(Board_Free_Spots)、转和挪一条命令、压到被顶住、不再往下顶让手歇下来 ⇒ 手上最低的那一点落在面上:
   --  关于它在手系(眼系)里位置 x 的一条线性方程(Geom.Press_Of)。几下一起解(Geom.Fit_Presses:每一下拿别的几下预测,差 ≤ 一小步;
   --  组外的只许是停早了;对不上补压两个方位中间的,最多 2 下)⇒ 这一瓣的尖 = x,不必在哪条像素视线上(人形大拇指的尖在画面外也一样量)。
   --  指尖 = 各瓣的尖的中点,张口 = 两瓣相距。碰的每一下爪子都按量过的张开那头发命令(09-28 V1B43:开机时对方复位一次,爪子的目标作废,
   --  命令跟着读数走,压的时候手指被桌面顶着越压越合)。以前按"尖就在那条视线上、视线交面 = 尖"量:V1B36 错 12.7 mm、V1B43 三处对不上。
   --  没有板的面、指尖量过(身体文件里的)⇒ 碰一下量面(顶住点 = 位姿 + 指尖:同一条"指尖碰在面上"反过来解);两样都没有 ⇒ 不碰,如实说。
   --  压的下数:有板的面 = 眼离面的高度 ÷ 一压 + 1(指尖在眼和面之间);没有 = 最多 8 下(次数)。完了回到原处

   procedure Geo_Boot_Support (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is separate;

   function Kin_Turn_Reach (Arm : Natural; P0 : Plug.Arm_Pose; Notch, Tol_P, Tol_R : Long_Float) return Long_Float is
      Best : Long_Float := 0.0;
      Ang : Long_Float := Notch;
      Pe, Re : Long_Float;
      Ok : Boolean;
      Av : Table.Vec := Table.Zero_Vec;
   begin
      if Notch <= 0.0 then
         return 0.0;
      end if;
      while Ang <= Ada.Numerics.Pi loop
         Av (3) := Ang;
         Plug.Reach (Arm, Chan.Compose (P0, Av), Pe, Re, Ok);
         exit when not Ok or else Pe > Tol_P or else Re > Tol_R;
         Best := Ang;
         Ang := Ang + Ang;
      end loop;
      return Best;
   end Kin_Turn_Reach;

   --  ④ 每条臂:一条命令能走多远还走得到 —— 从原处往上走 4、16、64 倍探针幅度(倍数,无量纲的阶梯),每档走完退回;
   --  实到不足命令一半(纯数学的一半)就是这条臂在这一档走不到(关节到头或控制器不跟),取走得到的最大一档。量一次存进几何文件。
   --  转动那一档不再推阶梯,每次开机按运动学算(Kin_Turn_Reach,09-28)
   procedure Geo_Boot_Stride (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is separate;

end Act;
