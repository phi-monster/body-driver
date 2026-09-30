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

   --  这条臂量到几个抓握通道就是几个;没有抓握通道(无人机、只有胳膊的身体)就是 0 —— 不按"至少一个"猜(09-30 原来 Max (1, …)、缺省 1)
   function Jaws_Of (C : Context; Arm : Natural) return Natural is
     (if Arm < Natural (C.Map.Jaws.Length) then C.Map.Jaws (Arm) else 0);

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
      for A in 0 .. C.Map.Arms - 1 loop
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
      W1 : constant Long_Float := Long_Float (Z.A.X1 - Z.A.X0 + 1) / Long_Float (Cw);
      H1 : constant Long_Float := Long_Float (Z.A.Y1 - Z.A.Y0 + 1) / Long_Float (Ch);
      W2 : constant Long_Float := (if Z.B.Valid then Long_Float (Z.B.X1 - Z.B.X0 + 1) / Long_Float (Cw) else W1);
      H2 : constant Long_Float := (if Z.B.Valid then Long_Float (Z.B.Y1 - Z.B.Y0 + 1) / Long_Float (Ch) else H1);
   begin
      --  还没量到手的时候退回一个百分之一画幅的小窗(比例,无量纲)
      if not Z.Valid or else not Z.A.Valid then
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

   procedure Init_Tracks (C : in out Context) is
   begin
      --  死区一开始当作零(还没证据说哪个通道推不动),边走边学
      C.Dead.Clear;
      C.Reach_M.Clear;
      for K in 0 .. C.Map.Arms * Chan.Per_Arm loop
         C.Dead.Append (0.0);
         C.Reach_M.Append (0.0);   --  0 = 还没量过这根通道一推走几米
      end loop;
      C.Zones.Clear;
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            declare
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cm);
               T : Zone_Track;
            begin
               T.Valid := Z.Valid;
               T.Cu := Z.Cu; T.Cv := Z.Cv; T.Z := Z.Depth;
               T.Au := Z.A.Cu; T.Av := Z.A.Cv; T.Bu := Z.B.Cu; T.Bv := Z.B.Cv;
               T.Has_Lobes := Z.Valid and then Z.N_Lobes >= 1;
               T.Known := Z.Valid;
               if Z.Valid then
                  T.Pieces (Chan.Per_Arm) := (True, Z.Cu, Z.Cv, (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth), Z.X0, Z.Y0, Z.X1, Z.Y1, Z.N_Lobes, Z.A.Cu, Z.A.Cv, Z.B.Cu, Z.B.Cv);
                  T.Pieces_Known (Chan.Per_Arm) := True;
               end if;
               --  开机每个通道推过一下:跟着动的那块 = 这个通道带的零件(不长在这只手上的相机里才算)
               if Cam_Arm (C, Cm) /= Integer (A) then
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     declare
                        Pi : constant Natural := (A * Chan.Per_Arm + K) * C.Map.N_Cams + Cm;
                     begin
                        if Pi < Natural (C.Map.Parts.Length) and then C.Map.Parts (Pi).Valid then
                           declare
                              P : constant Selfmap.Part := C.Map.Parts (Pi);
                           begin
                              T.Pieces (K) := (True, P.Cu, P.Cv, 0.0, P.X0, P.Y0, P.X1, P.Y1, 1, P.Cu, P.Cv, 0.0, 0.0);
                              T.Pieces_Known (K) := True;
                           end;
                        end if;
                     end;
                  end loop;
               end if;
               C.Zones.Append (T);
            end;
         end loop;
      end loop;
   end Init_Tracks;

   function Cut_Window (C : Context; Cam : Natural; F : Plug.Frame) return Long_Float is
      A : constant Integer := Cam_Arm (C, Cam);
   begin
      --  长在手上的相机斜看桌面:窗口 = 两指张幅按"指深 / 画面中位深"缩到画面深处("比这还大的不是能拿的东西");世界相机用画幅八分之一
      --  下面的 0.02 / 0.125 都是画幅的比例(无量纲):窗口的下限与上限
      if A >= 0 then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            Dp : Floats := F.Cams (Cam).Depth;
            Med : Long_Float;
         begin
            --  窗口至少要比正在跟的那块大半圈(倍数,无量纲),否则它一走近就被闭运算填平、只剩一圈边
            --  ⚠️ 试过"窗口跟着被跟的东西放大",错的:窗口一到画幅四分之一,桌面自己的起伏就盖过物体,
            --  深度那一路彻底切不出东西,颜色那一路接管、把墙缝和衣服切成上百块(EZ 逐步落图坐实)。窗口保持小。
            if Z.Valid and then Z.Span > 0.0 and then not Picture.Is_Nan (Z.Depth) and then F.Cams (Cam).Has_Depth then
               declare
                  Samp : Floats;
                  I : Natural := 0;
               begin
                  while I < Natural (Dp.Length) loop
                     if not Picture.Is_Nan (Dp (I)) and then Dp (I) > 0.0 then
                        Samp.Append (Dp (I));
                     end if;
                     I := I + 37;
                  end loop;
                  if Natural (Samp.Length) >= 16 then
                     Med := Picture.Quantile (Samp, 0.5);
                     if Med > 0.0 then
                        --  夹在画幅的 0.02 与 0.125 之间(比例,无量纲)
                        return Long_Float'Max (0.02, Long_Float'Min (0.125, Z.Span * (Z.Depth / Med) * 0.5));
                     end if;
                  end if;
               end;
            end if;
         end;
      end if;
      return 0.125;   --  世界相机:画幅八分之一(比例,无量纲)
   end Cut_Window;

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

   function Cut_Bright (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      G : constant Buf := F.Cams (Cam).Gray;
      Samp : Floats;
      T : Long_Float;
      T_First : Long_Float := 0.0;
      T_Low_Keep : Long_Float := 0.0;   --  暗刀的分界,中间那一段要用它当下沿
      Mask : Bools;
      Out_R : Picture.Regions;
      I : Natural := 0;
   begin
      if Natural (G.Length) < Cw * Ch then
         return Out_R;
      end if;
      --  分界按每一个像素算(09-30:原来每 7 个取一个 —— 新的 Split 要按置信界证出一道真谷,样本少了证不出,
      --  头顶眼里桌上的东西和桌面并成一块丢掉;Split 按直方图算,全像素也只多一遍直方图)
      while I < Cw * Ch loop
         Samp.Append (Long_Float (G.Element (I)));
         I := I + 1;
      end loop;
      T := Picture.Split (Samp);
      if Picture.Is_Nan (T) then
         return Out_R;   --  全一样 ⇒ 这只眼里按明暗切不出东西,如实交空
      end if;
      T_First := T;
      --  🔴 分两次:第一刀分的是"暗桌面 vs 亮的一切"(NJK 存图离线:分界 111,浅色木纹和白球并成一块);
      --  在亮的那一拨里再分一刀,才把最亮的一撮(白球、乐高的黄)从浅木纹里切出来。两刀的分界都是算出来的。
      declare
         Upper : Floats;
         T2 : Long_Float;
      begin
         for X of Samp loop
            if X > T then
               Upper.Append (X);
            end if;
         end loop;
         T2 := Picture.Split (Upper);
         if not Picture.Is_Nan (T2) then
            T := T2;
         end if;
      end;
      --  🔴 暗的东西也要认(黑键盘、红乐高、深色把手):在暗的那一拨里再分一刀,最暗的一撮单独成块;
      --  贴着画面边的暗块是我自己的胳膊/手指(它们从画面外伸进来),丢掉
      declare
         Lower : Floats;
         T_Low : Long_Float;
         Dark : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
      begin
         for X of Samp loop
            if X <= T_First then
               Lower.Append (X);
            end if;
         end loop;
         T_Low := Picture.Split (Lower);
         T_Low_Keep := (if Picture.Is_Nan (T_Low) then 0.0 else T_Low);
         if not Picture.Is_Nan (T_Low) then
            for J in 0 .. Cw * Ch - 1 loop
               if Long_Float (G.Element (J)) < T_Low then
                  Dark.Replace_Element (J, True);
               end if;
            end loop;
            for R of Picture.Components (Dark, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
               declare
                  Q : Picture.Region := R;
                  Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 + 1 >= Cw or else R.Y1 + 1 >= Ch;
               begin
                  if not Edge then
                     Q.Height := 0.0; Q.Depth := 0.0;   --  main 的 Region 没有 Top 这一位
                     Out_R.Append (Q);
                  end if;
               end;
            end loop;
         end if;
      end;
      --  🔴🔴 中间那一段以前整个当桌面扔了 —— 而"不亮不暗"的东西正好落在那儿。
      --  SC1 实测(头顶眼):剪刀那一片中位 95、桌面中位 135 —— 剪刀【比桌子暗】,
      --  却又没暗过暗刀的分界,于是亮刀和暗刀都不要它,画面上一个框都没有,
      --  脑连它的号都拿不到 ⇒ 点不了名 ⇒ 一步都动不了(验收线 1「剪刀」因此一次都没试成)。
      --  ⇒ 中间这一段再分一刀(还是 Otsu,和上下两刀同一个办法,不新加门槛):
      --    分完两拨里【少的那一拨】是东西,多的那一拨是桌面 —— 桌子总是占大头,这是数出来的,不是我拍的。
      declare
         Mid : Floats;
         T_Mid : Long_Float;
         N_Lo, N_Hi : Natural := 0;
         Thing_Is_Darker : Boolean;
         Band : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
      begin
         for X of Samp loop
            if X > T_Low_Keep and then X <= T then
               Mid.Append (X);
            end if;
         end loop;
         T_Mid := Picture.Split (Mid);
         if not Picture.Is_Nan (T_Mid) then
            for X of Mid loop
               if X <= T_Mid then
                  N_Lo := N_Lo + 1;
               else
                  N_Hi := N_Hi + 1;
               end if;
            end loop;
            Thing_Is_Darker := N_Lo < N_Hi;
            for J in 0 .. Cw * Ch - 1 loop
               declare
                  V : constant Long_Float := Long_Float (G.Element (J));
               begin
                  if V > T_Low_Keep and then V <= T
                    and then ((Thing_Is_Darker and then V <= T_Mid)
                              or else (not Thing_Is_Darker and then V > T_Mid))
                  then
                     Band.Replace_Element (J, True);
                  end if;
               end;
            end loop;
            for R of Picture.Components (Band, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
               declare
                  Q : Picture.Region := R;
                  Span_W : constant Boolean := R.X0 = 0 and then R.X1 + 1 >= Cw;
                  Span_H : constant Boolean := R.Y0 = 0 and then R.Y1 + 1 >= Ch;
               begin
                  if not Span_W and then not Span_H then
                     Q.Height := 0.0; Q.Depth := 0.0;
                     Out_R.Append (Q);
                  end if;
               end;
            end loop;
         end if;
      end;
      Mask := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
      for J in 0 .. Cw * Ch - 1 loop
         if Long_Float (G.Element (J)) > T then
            Mask.Replace_Element (J, True);
         end if;
      end loop;
      Last_Bright := Mask;   --  留给接触集扫轮廓用(同一张掩膜,不另切一遍)
      Last_Bright_Cam := Integer (Cam);
      Last_Split := T;
      for R of Picture.Components (Mask, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
         declare
            Q : Picture.Region := R;
            Span_W : constant Boolean := R.X0 = 0 and then R.X1 + 1 >= Cw;
            Span_H : constant Boolean := R.Y0 = 0 and then R.Y1 + 1 >= Ch;
         begin
            if not Span_W and then not Span_H then
               Q.Height := 0.0;   --  main 的 Region 没有 Top 这一位
               if F.Cams (Cam).Has_Depth then
                  declare
                     --  读深窗口 = 这块自己最窄边的四分之一,再小也有半个百分点的画幅(比例,无量纲);只当记录
                     Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                    Long_Float'Max (0.005, 0.25 * Long_Float'Min (Long_Float (R.X1 - R.X0 + 1) / Long_Float (Cw),
                                                                                                                 Long_Float (R.Y1 - R.Y0 + 1) / Long_Float (Ch))));
                  begin
                     Q.Depth := (if Picture.Is_Nan (Zd) then 0.0 else Zd);
                  end;
               end if;
               Out_R.Append (Q);
            end if;
         end;
      end loop;
      return Out_R;
   end Cut_Bright;

   function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Raw : Picture.Regions;
      Kept : Picture.Regions;
   begin
      --  🔴 M2 实测(850 轮 850 次撑爆、一段程序没问出来):没深度时这里原来直接返回空,
      --  THINGS OUT IN THE WORLD 整节是空的 —— 官方观测就是 3 路 RGB,这等于把眼睛关掉。
      --  而我第一次的修法(落到按颜色切)更糟:单帧清单涨到 267 条,把脑淹死
      --  —— 同一段代码上面那行警告早写着"在能看见东西的桌子上开着它,清单会从 7 条涨到 46 条"。
      --  ⇒ 正确的那条路是【按明暗切】(Cut_Bright):它自带门槛,并且在非自眼里把握区里的自己剔掉。
      --  长在手上的眼也走这条(手指在自己眼里是黑的);它一块都切不出来时才退回深度那一路。
      if Cam_Arm (C, Cam) >= 0 or else not F.Cams (Cam).Has_Depth then
         Raw := Cut_Bright (C, F, Cam);
         if not Raw.Is_Empty then
            for R of Raw loop
               declare
                  Mine : Boolean := False;
               begin
                  for A in 0 .. C.Map.Arms - 1 loop
                     if Cam_Arm (C, Cam) < 0 and then Zone.Is_Self (Zone_Of (C, A, Cam), R, Cw, Ch) then
                        Mine := True;
                     end if;
                  end loop;
                  if not Mine then
                     Kept.Append (R);
                  end if;
               end;
            end loop;
            return Kept;
         end if;
      end if;
      if not F.Cams (Cam).Has_Depth then
         return Kept;
      end if;
      --  🔴🔴 尺子不是选一把,是【每一把都看一遍,合起来】。
      --  "鼓出来"是相对周围说的:窗口比这块东西小的时候,这块东西自己就是周围 ⇒ 它鼓 0、整块消失。
      --  而窗口是按"两指在画面里张多开"缩放的 ⇒ 手越近、窗口越大、能被抹掉的东西越大 —— 方向正好反了。
      --  GM 实测:手一凑近,球(3027 px)和乐高(5098 px)双双从清单里消失,只剩 2 米外 10 px 的墙斑;
      --  而旧的"一块都切不出才换尺子"因为墙斑还在,根本不触发 ⇒ 最后一步反而瞎了 ⇒ 模板飘到墙上 ⇒
      --  手追着 2.13 m 外的墙把关节顶死。(同一条 GB 也实测过,修法 fe2ec8e 写过,被 a7ab7e9 回滚掉了。)
      --  合并规则:粗尺子切出来的块,只有当它的形心还没被任何已收的块盖住时才收(不重复列)。
      declare
         Win : constant Long_Float := Cut_Window (C, Cam, F);
         --  这台相机长在某条胳膊上吗:长着的话,要抓的东西贴到画面边是常态,不许因为贴边就丢
         Own_Cam_Here : constant Boolean := Cam_Arm (C, Cam) >= 0;
         --  🔴 第二把尺子 = 【脑点名那个东西自己有多大】(身体量的,不是我拍的系数)。
         --  闭运算填的是比窗口窄的东西 ⇒ 比窗口【宽】的东西自己就是背景,鼓 0、整块消失。
         --  所以会消失的恰恰是"比尺子宽"的那个,拿它自己的宽度当第二把尺子正好够着它。
         --  🔴🔴 而"多大"必须【按这一台相机算】:同一个球在头顶相机里 144 px、在腕相机里 96 px 宽却
         --  占 0.15 画幅。GP 实测:段跑在头顶相机 ⇒ Want_Size 是头顶那个小数 ⇒ 到腕相机不够大 ⇒
         --  球又被抹掉(腕相机只切出 3 块、2 块判成自己、只剩一块 516 px)。
         --  这一台相机自己记着"你上次点名那块在我这儿多大",拿它。取不到才退回 Want_Size。
         Wide : constant Long_Float := Long_Float'Max (Named_Span (C, Cam, Cw), C.Want_Size);
      begin
         Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Win, Sigma_Mult, Keep_Edge => Own_Cam_Here);
         if Wide > Win then
            declare
               More : constant Picture.Regions :=
                 Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Wide, Sigma_Mult, Keep_Edge => Own_Cam_Here);
            begin
               for R of More loop
                  declare
                     Covered : Boolean := False;
                  begin
                     for Q of Raw loop
                        if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                           Covered := True;
                        end if;
                     end loop;
                     if not Covered then
                        Raw.Append (R);
                     end if;
                  end;
               end loop;
            end;
         end if;
      end;
      --  🔴 只有【深度上一个都看不出来】的时候才按颜色切:桌面木纹、瓷砖缝的颜色台阶比线还明显,
      --  在能看见东西的桌子上开着它,清单会从 7 条涨到 46 条,脑子被淹掉(ES 实测)。
      --  线板那种场合深度切不出任何东西,颜色这一路才接手。
      if Raw.Is_Empty then
      --  再按颜色切一遍,把深度上鼓不出来的细东西(线、缝、刀口)补进来:
      --  门槛 = 这台相机静止时颜色抖多少(量出来的)的几倍;贴画面边的是桌面/墙,丢掉(本仓既有规矩);
      --  已经被深度块盖住的不重复列
      declare
         --  门槛:比相机噪声大,也要比这张画面自己的纹理粗(木纹、布纹都会被纹理这一项吃掉);倍数无量纲
         Noise_C : constant Natural := (if Cam < Natural (C.Map.Pic_Floor.Length) and then C.Map.Pic_Floor (Cam) > 0
                                        then Natural (C.Map.Pic_Floor (Cam)) else 0);
         Floor_C : constant Long_Float :=
           Long_Float'Max (Long_Float (Noise_C) * 2.0 + 1.0, Picture.Texture_Level (F.Cams (Cam).RGB, Cw, Ch) * 4.0);
         Thin : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C, Picture.Min_Pixels (Cw, Ch));
      begin
         for R of Thin loop
            declare
               Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 >= Cw - 1 or else R.Y1 >= Ch - 1;
               Covered : Boolean := False;
            begin
               for Q of Raw loop
                  if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                     Covered := True;
                  end if;
               end loop;
               if not Edge and then not Covered then
                  declare
                     Q : Picture.Region := R;
                  begin
                     --  颜色切出来的块也要有远近:在它自己的位置上读一小片深度(窗口 = 它自己框的四分之一,比例,无量纲)
                     if F.Cams (Cam).Has_Depth then
                        declare
                           Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                          Long_Float'Max (0.005, Long_Float (R.X1 - R.X0) / Long_Float (Cw) * 0.25));
                        begin
                           if not Picture.Is_Nan (Zd) then
                              Q.Depth := Zd;
                           end if;
                        end;
                     end if;
                     Raw.Append (Q);
                  end;
               end if;
            end;
         end loop;
      end;
      end if;
      declare
         Mine_N : Natural := 0;
         Big_W : Natural := 0;
      begin
      for R of Raw loop
         declare
            Mine : Boolean := False;
         begin
            for A in 0 .. C.Map.Arms - 1 loop
               if Zone.Is_Self (Zone_Of (C, A, Cam), R, Cw, Ch) then
                  Mine := True;
               end if;
            end loop;
            if not Mine then
               Kept.Append (R);
               Big_W := Natural'Max (Big_W, R.X1 - R.X0 + 1);
            else
               Mine_N := Mine_N + 1;
            end if;
         end;
      end loop;
      --  GM 里手一凑近,球(3027 px)和乐高人(5098 px)双双从清单里消失,只剩 2 米外 10 px 的墙斑。
      --  两个可疑处各印一个数,别再靠猜:闭运算窗口(比它窄的凸起会被当背景填平)· 被判成"我自己"而丢掉的块数。
      if Codec.Env ("BL_CUTLOG") /= "" then
         Put_Line ("[身]     切块(相机" & Natural'Image (Cam) & "):窗口 " & Codec.Fmt (Cut_Window (C, Cam, F), 3) & " 画幅 = "
                   & Codec.Img (Natural (Long_Float (Cw) * Cut_Window (C, Cam, F))) & " px · 切出 " & Codec.Img (Natural (Raw.Length))
                   & " 块,其中 " & Codec.Img (Mine_N) & " 块判成我自己丢掉 · 留下最宽的一块 " & Codec.Img (Big_W) & " px");
      end if;
      end;
      return Kept;
   end Cut_Things_Raw;

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
                         Pu_On, Pv_On : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      if Length (C.Inst_Host) = 0 then
         Got := False; Iso := False; R := (others => <>); M := Bool_Vectors.Empty_Vector;
         if not Said_No_Seg then
            Said_No_Seg := True;
            Put_Line ("[身] 📦 没配分割仪器 ⇒ 量不出脑框出来的东西是哪些像素(这一样只有仪器这一种量法)");
         end if;
         return;
      end if;
      declare
         Area : Natural;
         Score : Long_Float;
         Ok : Boolean;
         Err : Unbounded_String;
         On_Pts : Instrument.Seg_Pt_Vectors.Vector;
      begin
         if Pu_On >= 0.0 and then Pv_On >= 0.0 and then Pu_On < Long_Float (Cw) and then Pv_On < Long_Float (Ch) then
            On_Pts.Append (Instrument.Seg_Pt'(U => Pu_On, V => Pv_On, On => True));
         end if;
         Instrument.Segment (To_String (C.Inst_Host), C.Inst_Port, F.Cams (Cam).RGB, Cw, Ch, X0, Y0, X1, Y1, On_Pts, M, Area, Score, Ok, Err);
         if not Ok or else Area = 0 then
            Got := False; Iso := False; R := (others => <>);
            if not Ok then
               Put_Line ("[身] 📦 分割仪器没回来(" & To_String (Err) & ")");
            end if;
            return;
         end if;
         Picture.Region_Of_Mask (M, Cw, Ch, R, Got);
         Iso := Got and then R.X0 > 0 and then R.Y0 > 0 and then R.X1 + 1 < Cw and then R.Y1 + 1 < Ch;
      end;
   end Seg_In_Box;

   procedure Remeasure_Boxed (C : in out Context; F : Plug.Frame; Cam : Natural; Regs : in out Picture.Regions) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
         if C.Boxed (Bi).Cam = Cam and then not C.Boxed (Bi).Blind then   --  脑说"这只眼里没有它"的那一条没有框可量
            declare
               B : Boxed_Thing := C.Boxed (Bi);
               Found, Iso : Boolean;
               R : Picture.Region;
            begin
               Seg_In_Box (C, F, Cam, B.X0, B.Y0, B.X1, B.Y1, Found, Iso, R, B.Mask, B.Pu_On, B.Pv_On);
               --  我一动,长在我手上的眼里它会平移一截(GB5:横挪 25.6 mm,它从 u=288 跳到 260)。
               --  量到的那一块顶到了窗边 = 它有一部分在窗外 ⇒ 把窗挪到【量到的这一块】身上再量,直到整块落进窗里或不再变。
               --  还是同一个量法,只是跟着它走;最多跟 4 回(次数)。
               for Again in 1 .. 4 loop
                  exit when not Found or else Iso;
                  declare
                     F2, I2 : Boolean;
                     R2 : Picture.Region;
                     M2 : Bools;
                  begin
                     Seg_In_Box (C, F, Cam, R.X0, R.Y0, R.X1, R.Y1, F2, I2, R2, M2, B.Pu_On, B.Pv_On);
                     exit when not F2 or else (R2.X0 = R.X0 and then R2.Y0 = R.Y0 and then R2.X1 = R.X1 and then R2.Y1 = R.Y1);
                     R := R2; Iso := I2; B.Mask := M2;
                  end;
               end loop;
               --  🔴 量到的那一块还是不是它:拿它和周围的明暗【哪边亮】对。脑指它那一帧记下"它比周围亮还是暗";这一帧量到的块要是反过来了,
               --  那是别的东西(H31 2026-09-22 实测:预测窗漂到另一只手的黑爪子上,框里"最大的一块"就成了爪子,视线交点算到 14 cm 高的空中)。
               --  只比方向不比幅度:H32 实测手的影子一盖,剪刀从 216 暗到 165(背景 127),按幅度就把真剪刀判成了别的东西。认不出就老实说看不见,不许锁错。
               if Found and then B.Gray >= 0.0 and then B.Bg >= 0.0 then
                  declare
                     Tg, Bk : Long_Float;
                  begin
                     Blob_Levels (F.Cams (Cam).Gray, Cw, Ch, B.Mask, R, Tg, Bk);
                     --  09-13 总规矩:动起来之后身体不许有闸 ⇒ 说出来、照走(2026-09-26 以前这里判"不是它,算看不见")
                     if Tg >= 0.0 and then Bk >= 0.0 and then (Tg - Bk) * (B.Gray - B.Bg) <= 0.0 and then B.Seen then
                        Put_Line ("[身] 📦 " & To_String (B.Name) & "(第" & Codec.Img (Cam) & " 台):框里量到的那块平均亮 "
                                  & Codec.Fmt (Tg, 0) & "、周围 " & Codec.Fmt (Bk, 0) & ",它当初 " & Codec.Fmt (B.Gray, 0) & "、周围 " & Codec.Fmt (B.Bg, 0)
                                  & " ⇒ 明暗反了(可能不是它,也可能是影子盖住了);照这块跟");
                     end if;
                  end;
               end if;
               --  🔴 量到的那一块还是不是它,第二样:大小。窗挪到预测处时它该有多少像素是算过的(上一次的像素数 × 远近比例的平方);
               --  这一帧量到的是单独的一块、却不到预期的四分之一(线尺寸的一半,纯数学)⇒ 那是窗底下的别的东西,不是它
               --  (H61 2026-09-23 实测:交点算深了 14 cm,窗一路漂到桌面上,540 px 的一小块桌纹当成了 7000 px 的剪刀,合空)。
               --  顶着窗边的块不判(它可能只露了一截);更大也不判(它可能刚露全)
               if Found and then Iso and then B.Count > 0 and then R.Count * 4 < B.Count and then B.Seen then   --  说出来、照走(同上)
                  Put_Line ("[身] 📦 " & To_String (B.Name) & "(第" & Codec.Img (Cam) & " 台):框里量到的那块只有 " & Codec.Img (R.Count)
                            & " px,它该有约 " & Codec.Img (B.Count) & " px ⇒ 小得不像它(可能不是它);照这块跟");
               end if;
               B.Seen := Found;
               if Found then
                  B.X0 := R.X0; B.Y0 := R.Y0; B.X1 := R.X1; B.Y1 := R.Y1;
                  B.Cu := R.Cu; B.Cv := R.Cv; B.Isolated := Iso; B.Count := R.Count;
                  On_Pixel (B.Mask, Cw, Ch, R, B.Pu_On, B.Pv_On);
                  --  它身上的碎片:形心落在它框里的那些块,由这一整块顶替
                  for Ri in reverse 0 .. Natural (Regs.Length) - 1 loop
                     if Picture.Inside (R, Regs (Ri).Cu, Regs (Ri).Cv, Cw, Ch, 0.0) then
                        Regs.Delete (Ri);
                     end if;
                  end loop;
                  --  调用方都拿 (0) 当最大的一块 ⇒ 按像素数插回去,不许打乱从多到少的次序
                  declare
                     At_I : Natural := Natural (Regs.Length);
                  begin
                     for Ri in 0 .. Natural (Regs.Length) - 1 loop
                        if Regs (Ri).Count < R.Count then
                           At_I := Ri;
                           exit;
                        end if;
                     end loop;
                     if At_I >= Natural (Regs.Length) then
                        Regs.Append (R);
                     else
                        Regs.Insert (At_I, R);
                     end if;
                  end;
               end if;
               C.Boxed.Replace_Element (Bi, B);
            end;
         end if;
      end loop;
   end Remeasure_Boxed;

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
                            Keep : Boolean := False; Things_Only : Boolean := False) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      T : Unbounded_String;
      Named_U, Named_V : Long_Float := -1.0;
      Have_Named : Boolean := False;
      --  语言的根(2026-09-23):有能合拢的手 ⇒ 脑的句子只说【东西的量往哪变】,我身上的零件它点不到、也不该看见
      --  (H45 实测:纸上列着 "a finger of arm 1 … grip 2",9B 的脑就把 "reach arm leftwards"、"grip above …" 填进东西的位置)。
      --  零件照旧进清单(绑 grasper 要用),只是不写给脑、不画框。
      --  "有能合拢的手"是量出来的:哪个抓握通道推到头时有相机看见手指来去(Has_Fingers);原来按抓握通道的个数(至少 1),
      --  什么都不带的抓握通道(无人机)也算 ⇒ 脑只能说"东西的量往哪变",可身体根本没有能拿起东西的零件(DR2 2026-09-28)
      Qmode : constant Boolean := Any_Fingers (C);
      function Rel (U, V : Long_Float) return String is
         Half : constant String := (if U < 0.5 then "LEFT" else "RIGHT");
      begin
         if Have_Named then
            declare
               D : constant Long_Float := Sqrt (((U - Named_U) * Long_Float (C.Cols)) ** 2 + ((V - Named_V) * Long_Float (C.Rows)) ** 2);
            begin
               return ", " & Codec.Fmt (D, 1) & " cells from the thing you last named, in the " & Half & " half of the picture";
            end;
         end if;
         return ", in the " & Half & " half of the picture";
      end Rel;
      --  这个框此刻跟我身上哪一块的框重叠吗(身体零件在世界那一段之前就已经列进 C.Items 了)
      function Under_Me (X0, Y0, X1, Y1 : Natural) return Boolean is
      begin
         for I in 0 .. Natural (C.Items.Length) - 1 loop
            declare
               B : constant Item := C.Items (I);
            begin
               if B.Kind in Finger | Grip | Piece and then B.Located
                 and then B.X1 >= X0 and then X1 >= B.X0
                 and then B.Y1 >= Y0 and then Y1 >= B.Y0
               then
                  return True;
               end if;
            end;
         end loop;
         return False;
      end Under_Me;

      procedure Push (It_In : Item; Line : String; Col : Draw.Color; Thick : Natural) is
         It : Item := It_In;
      begin
         It.Cam := Cam;   --  这一块是在哪台相机里看见的:清单跨相机之后,这一位是它唯一的落脚点
         C.Items.Append (It);
         if Qmode and then It.Kind in Finger | Grip | Piece then
            return;   --  零件进清单不进纸(见 Qmode)
         end if;
         if It.Located then
            Draw.Numbered_Box (RGB, Cw, Ch, It.X0, It.Y0, It.X1, It.Y1, Natural (C.Items.Length), Col, Thick);
         end if;
         --  行首只放方括号里的号(对得上画面上那个框就够了),不再写 "item":T2 实测 Qwen 把这个记账词当成了东西的名字
         Append (T, "  [" & Codec.Img (Natural (C.Items.Length)) & "] " & Line & ASCII.LF);
      end Push;
   begin
      if not Keep then
         C.Items.Clear;
      end if;
      if C.Wld.Cams (Cam).Named >= 0 then
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (C.Wld.Cams (Cam).Named));
         begin
            if Sl.Seen then
               Have_Named := True;
               Named_U := (if Sl.Present then Sl.R.Cu else Sl.Shadow.Cu);
               Named_V := (if Sl.Present then Sl.R.Cv else Sl.Shadow.Cv);
            end if;
         end;
      end if;
      --  🔴🔴 自述那一条通道(owner 2026-09-15 定;架构 2026-08-18 已为它留位):
      --  命令那条通道一个字不提身体 ⇒ 换一具机器照样能用,那是命根子,不许碰。
      --  这一条【必然提到身体】—— "我这根通道""我的手""我上次" —— 所以单独一段、单独出现。
      --  它是身体唯一一处可以讲【自己】的地方:我知道什么 · 我不知道什么 · 我变了什么。
      --  每一句后面都带【我凭什么信它】(量过几次 / 上次第几拍 / 来回对不对得上),
      --  没有这个,"我很有把握"和"我瞎猜的"在脑那边长得一模一样。
      if not Things_Only then
         Append (T, ASCII.LF & "ABOUT MYSELF (measured by me, on this body, with how much I trust each line):" & ASCII.LF);
         --  尺子(④ 09-27):给脑的长度按指尖长说,在这儿说一次单位是什么
         Append (T, (if Hand_Len (C) > 0.0
                     then "  MY RULER: every length I tell you is in hand-lengths. One hand-length is the distance from my eye to my fingertips, "
                          & "which I measured myself by touching the table. I have no other ruler and I do not know centimetres."
                     elsif not Any_Fingers (C)
                     then "  MY RULER: I have no fingers, so I have no hand-length to measure; any length I tell you is only in my own scale, "
                          & "which means nothing outside this body."
                     else "  MY RULER: I have not measured my own size yet, so any length I tell you is only in my own scale, "
                          & "which means nothing outside this body.") & ASCII.LF);
         --  没有手指的臂照实说一句(PLAN V1b 无人机第 2 条:"我没有手指,长度只有我自己的比例")
         for A in 0 .. C.Map.Arms - 1 loop
            if not Arm_Has_Fingers (C, A) then
               Append (T, "  I HAVE NO FINGERS on arm " & Codec.Img (A + 1) & ": I pushed its grip channel from one end to the other and nothing in any of my pictures moved. "
                       & "I cannot hold, pinch or lift anything with it." & ASCII.LF);
            end if;
         end loop;
         declare
            Said : Natural := 0;
         begin
            for I in 0 .. Natural (C.Tables.Length) - 1 loop
               declare
                  Se : constant Stored_Effect := C.Tables (I);
               begin
                  if Se.Cam = C.Cam and then Se.Kind = Piece_Pt and then Said < 4 then
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        if Se.Trust (K) and then Said < 4 then
                           Append (T, "  I KNOW: push channel " & Codec.Img (Se.Arm * Chan.Per_Arm + K)
                                   & " by one unit and a piece of me travels "
                                   & Codec.Fmt (Sqrt (Se.E.B (K, 0) ** 2 + Se.E.B (K, 1) ** 2), 3)
                                   & " of a frame here - learned " & Codec.Img (Se.N_Learned)
                                   & " time(s), last confirmed at beat " & Codec.Img (Se.When_Beat)
                                   & (if Se.Agree (K) < 0.0 then ", never checked out-and-back"
                                      elsif Se.Agree (K) < 1.0 then ", and out-and-back agree so I am confident"
                                      else ", but out-and-back DISAGREE so I do not trust it")
                                   & ASCII.LF);
                           Said := Said + 1;
                        end if;
                     end loop;
                  end if;
               end;
            end loop;
            if Said = 0 then
               Append (T, "  I KNOW: nothing yet about how my own pushes move me in this eye - I have not measured it here." & ASCII.LF);
            end if;
         end;
         if C.Depth_Scale > 0.0 then
            Append (T, "  I DO NOT KNOW HOW FAR: one metre of my own real motion changes my depth reading by "
                    & Codec.Fmt (C.Depth_Scale, 1) & " metres, and one metre is the most that is physically possible."
                    & " So my sense of distance is inflated by at least that much. I use it for which way, never for how far."
                    & ASCII.LF);
         end if;
         for A in 0 .. C.Map.Arms - 1 loop
            for Cm in 0 .. C.Map.N_Cams - 1 loop
               if Cam_Arm (C, Cm) /= Integer (A) and then Track_Idx (C, A, Cm) < Natural (C.Zones.Length)
                 and then Cm = C.Cam
               then
                  declare
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cm));
                  begin
                     Append (T, "  I KNOW WHERE MY OWN HAND IS (arm " & Codec.Img (A + 1) & ") here only "
                             & (if not Tr.Valid then "not at all"
                                elsif Tr.Blew_Up then "as a guess that blew up - I refuse it"
                                elsif Tr.Known then "because I have actually looked at it from a pose close to this one"
                                else "by working it out from my joints; I have not looked at it from a pose like this")
                             & ASCII.LF);
                  end;
               end if;
            end loop;
         end loop;
         --  🔴 我【怎么量远近】每只眼不一样,而且这是量过的,以前却从不告诉脑:
         --  长在手上的眼 —— 我横挪一段自己报得出的距离,看它在画面里跳多少,两条视线一交就是它离我多远 ⇒ 几大步走到
         --  (GB5 实测 4 步从差 284 mm 到差 8 mm);不跟着我动的眼 —— 它不动,我看不出远近 ⇒ 只能推一点看一点
         --  (T3 2026-09-21 实测 60 推差距 0.434 → 0.433)。T5/T6 里 Qwen 每一段都写 with my still eye,它不知道这件事。
         --  量了不说 = 把量到的东西藏起来。这是读数,不是窍门:没有例句,没有流程。
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            declare
               A : constant Integer := Cam_Arm (C, Cm);
               Ready : constant Boolean := A >= 0 and then Cm < Natural (C.Geo.Length)
                                           and then C.Geo (Cm).Tip_Valid and then C.Geo (Cm).F > 0.0;
            begin
               if Ready then
                  Append (T, "  HOW FAR AWAY A THING IS - through the eye that rides on arm " & Codec.Img (Natural (A) + 1)
                          & " (camera index " & Codec.Img (Cm) & ") I CAN measure it: I step sideways a distance I know and watch how far the thing jumps. "
                          & "Through that eye, touching is carried out by walking arm " & Codec.Img (Natural (A) + 1)
                          & " up to the thing in a few large steps." & ASCII.LF);
               elsif A < 0 then
                  Append (T, "  HOW FAR AWAY A THING IS - through the eye that does not move with me (camera index " & Codec.Img (Cm)
                          & ") I CANNOT measure it. Through that eye I can only nudge and look again, about one pixel of progress per push." & ASCII.LF);
               end if;
            end;
         end loop;
         Append (T, To_String (C.Changed_Say));
         --  🔴 经历账读回来:它才说得出"上次我在这上面是怎么成的"。
         --  这一段【不是】命令通道的一部分 —— 它提到我自己干过什么,所以只出现在自述这一段里。
         declare
            Life : constant String := Codec.Tail_Lines (Life_Path, 6);
         begin
            if Life /= "" then
               Append (T, "  WHAT I HAVE DONE BEFORE (my own log, kept across every run I have ever had):" & ASCII.LF & Life);
            else
               Append (T, "  WHAT I HAVE DONE BEFORE: nothing - this is the first stretch I can remember." & ASCII.LF);
            end if;
         end;
      if not Qmode then
         Append (T, "PIECES OF YOURSELF (measured just now: you moved one channel at a time and watched which part of the picture followed; you closed each hand on nothing and watched which pixels swept). Each is boxed and NUMBERED on the picture in orange:" & ASCII.LF);
      end if;
      for A in 0 .. C.Map.Arms - 1 loop
       --  一条臂上量到几个抓握通道就列几组:两指手 1 组,五指手 5 组。代码里没有"一只手一个夹爪"这个假设。
       for Jk in 0 .. Jaws_Of (C, A) - 1 loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Jk);
            Tr : constant Zone_Track := (if Track_Idx (C, A, Cam) < Natural (C.Zones.Length) then C.Zones (Track_Idx (C, A, Cam)) else (others => <>));
            Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
            Du : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cu - Z.Cu);
            Dv : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cv - Z.Cv);
            procedure Finger (Lb : Zone.Lobe; Which : Natural) is
               It : Item;
               --  这一瓣挪了多少:身体图给了各瓣位置就按瓣,否则整区平移
               Lu : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Au else Tr.Bu) - Lb.Cu else Du);
               Lv : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Av else Tr.Bv) - Lb.Cv else Dv);
            begin
               It.Kind := Finger; It.Arm := A; It.Which := Which; It.Jaw_K := Jk;
               if Z.Valid and then Lb.Valid and then Tr.Valid then
                  It.Located := True;
                  It.Cu := Lb.Cu + Lu; It.Cv := Lb.Cv + Lv;
                  It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Lu * Long_Float (Cw)));
                  It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Lu * Long_Float (Cw))));
                  It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Lv * Long_Float (Ch)));
                  It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Lv * Long_Float (Ch))));
                  It.Depth := Tr.Z; It.Count := Lb.Count;
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when grip channel " & Codec.Img (Jk)
                        & " of that arm moves), now in cell " &
                        Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                        (if Own_Cam or else Tr.Known then "" else " (placed from my joints; I have not yet looked at my hand here)"), Draw.Orange, 2);
               else
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (grip channel " & Codec.Img (Jk)
                        & ") - NOT locatable in this picture right now, do not name it", Draw.Orange, 0);
               end if;
            end Finger;
            G : Item;
         begin
            --  这个抓握通道开机推到头时哪台相机里都没看见东西跟着动 ⇒ 它不带手指:不列手指 / 爪心(零件照列,见下)
            if Has_Fingers (C, A, Jk) then
               Finger (Z.A, 0);
               Finger (Z.B, 1);
               --  🔴🔴 同一个爪的两瓣,在画面里应该只隔【量到的钳口张幅】那么远。
               --  差得离谱 = 我按关节推出来的位置在这台相机里根本不对,而这条我自己量得出来。
               --  GW 实测:arm 2(右臂)的两根手指被放到画面【左】边的第 2 格和第 19 格,相隔四分之三个画面,
               --  而它自己标着"我还没在这儿看过我的手"。位置错 ⇒ 误差错 ⇒ 往错的方向推 ⇒
               --  十炮里七炮"靠近→停在错的稳定点→退开"。必须说出来,别让脑拿它当真。
               if Z.Valid and then Z.A.Valid and then Z.B.Valid and then Z.Span > 0.0 then
                  declare
                     Sep : constant Long_Float :=
                       Sqrt ((Z.A.Cu - Z.B.Cu) ** 2 + (Z.A.Cv - Z.B.Cv) ** 2);
                  begin
                     --  比的是两个量出来的量,没有人拍的系数:隔得比张幅还远 ⇒ 对不上
                     if Sep > Z.Span + Z.Span and then not Qmode then
                        Append (T, "  (careful: I placed the two jaws of arm " & Codec.Img (A + 1)
                                & " " & Codec.Fmt (Sep, 3) & " of the picture apart, but the jaw span I measured on myself is only "
                                & Codec.Fmt (Z.Span, 3) & " - they cannot both be right, so where I think my hand is in this"
                                & " picture is not to be trusted)" & ASCII.LF);
                     end if;
                  end;
               end if;
               G.Kind := Grip; G.Arm := A; G.Jaw_K := Jk;
               if Z.Valid and then Tr.Valid then
                  G.Located := True;
                  G.Cu := Tr.Cu; G.Cv := Tr.Cv; G.Depth := Tr.Z;
                  G.X0 := Natural (Long_Float'Max (0.0, Long_Float (Z.X0) + Du * Long_Float (Cw)));
                  G.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Z.X1) + Du * Long_Float (Cw))));
                  G.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Z.Y0) + Dv * Long_Float (Ch)));
                  G.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Z.Y1) + Dv * Long_Float (Ch))));
                  Push (G, "grip " & Codec.Img (A + 1) & " - the space between the fingers of arm " & Codec.Img (A + 1) &
                        " (closing = grip close with grip_arm " & Codec.Img (A + 1) & "; a thing must sit in this box to be held), now in cell " &
                        Codec.Img (Cell_Of (C, G.Cu, G.Cv)) & Rel (G.Cu, G.Cv), Draw.Pink, 2);
               else
                  Push (G, "grip " & Codec.Img (A + 1) & " (the space between the fingers of arm " & Codec.Img (A + 1) & ") - not locatable in this picture right now", Draw.Pink, 0);
               end if;
            end if;
            --  全身零件:每个通道带的那一块(从那个关节往外的全部),位置按此刻位姿从身体图来
            if not Own_Cam and then Jk = 0 then
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Pc : constant Schema.Part_Pos := Tr.Pieces (K);
                     It : Item;
                  begin
                     if Pc.Valid then
                        It.Kind := Piece; It.Arm := A; It.Which := K; It.Located := True;
                        It.Cu := Pc.Cu; It.Cv := Pc.Cv; It.Depth := Pc.Z;
                        It.X0 := Pc.X0; It.Y0 := Pc.Y0; It.X1 := Pc.X1; It.Y1 := Pc.Y1;
                        Push (It, "a piece of you: everything that swings when channel " & Codec.Img (K) & " of arm " & Codec.Img (A + 1) & " moves (measured), now in cell " &
                              Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                              (if Tr.Pieces_Known (K) then "" else " (placed from my joints; not yet looked at here)"), Draw.Orange, 1);
                     end if;
                  end;
               end loop;
            end if;
         end;
       end loop;
      end loop;
      --  一块都没列出来就照实说一句(DR4 / DR5 2026-09-28:无人机这一行底下什么都没有,"每一块都框了编号"对着一张空单子)
      if not Qmode and then (for all It of C.Items => It.Cam /= Cam or else It.Kind not in Finger | Grip | Piece) then
         Append (T, "  (none: I found no piece of myself in this picture)" & ASCII.LF);
      end if;
      end if;
      Append (T, (if Things_Only
                  then "THINGS IN MY EYE " & Codec.Img (Cam + 1) & " (same numbering - a number means the same thing everywhere I say it):"
                  else "THINGS YOU HAVE NAMED (you told me which part of the picture each one is in; inside that part I measured it myself, and I measure it again every frame). Each is boxed and NUMBERED on the picture in green:") & ASCII.LF);
      --  没点过名的块不列(见下),所以这里不再需要"装不下就只列最大的几件"那一套;
      --  但它们【有多少】要如实说 —— 脑得知道这只眼里还有别的东西,只是它还没给它们起名字。
      declare
         N_Unnamed : Natural := 0;
      begin
      for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
            It : Item;
         begin
            It.Slot := Si;
            --  🔴 世界里只列【脑点过名】的东西(和此刻攥在手里的)。没点过名的块不列、不画框:
            --  编号早就不进语言了(LANGUAGE §3.1),认名字那一问也不再问"第几号"(Brain.Locate 问"在哪一框"),
            --  所以几百行 "a thing, now in cell N" 对脑没有任何用处;09-21 实测画上去的编号框还在伤它的视力
            --  (同一帧:干净画面在场 16/16,画上格子和编号框后 2/4)。全图切块照旧在跑,碰没碰到别的东西还靠它。
            declare
               Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
               Held_Here : constant Boolean :=
                 C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam);
            begin
               if not Held_Here and then Boxed_Index (C, Cam, Ref.Cu, Ref.Cv) < 0 then
                  if Sl.Present then
                     N_Unnamed := N_Unnamed + 1;
                  end if;
                  goto Next_Slot;
               end if;
            end;
            if C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam) then
               declare
                  A : constant Natural := Natural (C.Wld.Held_Arm);
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                  --  拿着东西的是哪一个抓握通道:身体记着(C.Wld.Held_Jaw)
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Natural (Integer'Max (0, C.Wld.Held_Jaw)));
               begin
                  It.Kind := Thing_Held; It.Arm := A; It.Located := Tr.Valid;
                  It.Cu := Tr.Cu; It.Cv := Tr.Cv; It.Depth := Tr.Z;
                  It.X0 := Z.X0; It.Y0 := Z.Y0; It.X1 := Z.X1; It.Y1 := Z.Y1;
                  It.Count := Sl.Shadow.Count; It.Height := Sl.Shadow.Height;
                  Push (It, "the thing between the fingers of arm " & Codec.Img (A + 1) & " (it moves with that arm), now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)), Draw.Green, 2);
               end;
            elsif Sl.Present then
               It.Kind := Thing; It.Located := True;
               It.Cu := Sl.R.Cu; It.Cv := Sl.R.Cv; It.Depth := Sl.R.Depth; It.Height := Sl.R.Height; It.Count := Sl.R.Count;
               It.X0 := Sl.R.X0; It.Y0 := Sl.R.Y0; It.X1 := Sl.R.X1; It.Y1 := Sl.R.Y1;
               It.Au := Sl.R.Au; It.Av := Sl.R.Av; It.Elong := Sl.R.Elong;
               It.Gray := Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, Sl.R);
               --  🔴 这一块的【胖瘦】和【亮度】我本来就量了(Elong / Gray,量它们的注释写着"用来和别的块区分"),
               --  却只留给自己重新找目标用,从不说给脑听。结果是清单上两百条字面一模一样的
               --  "a thing, now in cell N (NN px, standing 0.000)" —— 没有深度时"standing"全是 0.000,
               --  于是每一行只剩一个格号和一个像素数,谁也点不出名。点不出名就写不出"走到它那儿去",
               --  脑只好退回 close/open/still 这三个不用点名的词。**量了不说 = 把眼睛量到的东西藏起来。**
               --  ⇒ 量到什么就说什么:长宽比、平均亮度、以及这只眼这一帧自己算出来的明暗分界(基准)。
               --  这不是给窍门 —— 没有例句、没有"哪一块是球",只是把尺子上的读数念出来。
               declare
                  Bx : constant Integer := Boxed_Index (C, Cam, Sl.R.Cu, Sl.R.Cv);
                  Nm : constant String := (if Bx >= 0 then To_String (C.Boxed (Natural (Bx)).Name) else "");
                  Alone : constant Boolean := Bx >= 0 and then C.Boxed (Natural (Bx)).Isolated;
               begin
                  Push (It, "what you called " & Nm & ", now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px, "
                        & Codec.Fmt (It.Elong, 1) & "x as long as it is wide"
                        & (if Alone then "" else "; in this eye it runs into something next to it or into the edge of the picture, so its middle and its long direction are not trustworthy here")
                        & ")" & Rel (It.Cu, It.Cv)
                        --  身体列它量得到的量(语言的根):脑要改的是这个量,不是我的手
                        & "; a quantity of it I measure and can change: height (how far it is above the surface it lies on)", Draw.Green, 2);
               end;
            elsif Sl.Seen then
               It.Kind := Thing_Remembered; It.Located := True;
               It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Count := Sl.Shadow.Count;
               It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
               It.Au := Sl.Shadow.Au; It.Av := Sl.Shadow.Av; It.Elong := Sl.Shadow.Elong;
               --  🔴 实测(另一棵树,同样这一行):凡"见过但现在看不见"的槽全列出来 ⇒ 涨到 item 317,
               --  提示词撑爆模型上下文 785 次,从第 189 轮起 560 轮一段程序都问不出来。
               --  ⇒ 只留【我确实自己挡住了它】的那些:它上次待的框跟我身上哪一块此刻的框重不重叠。
               --    重叠 = 伸手过去时那件东西消失的那一刻,留着有用;不重叠 = 它就是不见了,
               --    我说不出它在哪,也就不许拿它占脑的篇幅。
               if Under_Me (It.X0, It.Y0, It.X1, It.Y1) then
                  Push (It, "what you called " & To_String (C.Boxed (Natural (Boxed_Index (C, Cam, It.Cu, It.Cv))).Name)
                        & ", now hidden behind a part of me, last seen in cell "
                        & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px)", Draw.Dim_Green, 1);
               end if;
            else
               null;   --  空槽:里面此刻什么都没有,列出来只是占篇幅
            end if;
            <<Next_Slot>>
            null;
         end;
      end loop;
         if N_Unnamed > 0 then
            Append (T, "  (In this eye I can also make out " & Codec.Img (N_Unnamed)
                    & " other patches that you have not named. I do not list them: a name is how you point at a thing.)" & ASCII.LF);
         end if;
      end;
      --  相机表
      declare
         K : Natural := 2;
      begin
         Append (T, "CAMERAS (say look = k to see through that camera next turn): 1 = this picture (camera index " & Codec.Img (Cam) & ")");
         for Ci in 0 .. C.Map.N_Cams - 1 loop
            if Ci /= Cam then
               declare
                  A : constant Integer := Cam_Arm (C, Ci);
               begin
                  Append (T, "; " & Codec.Img (K) & " = camera index " & Codec.Img (Ci) & (if A >= 0 then " (rides on arm " & Codec.Img (Natural (A) + 1) & ": when that arm moves, that whole picture changes)" else ""));
                  K := K + 1;
               end;
            end if;
         end loop;
         Append (T, ASCII.LF);
      end;
      declare
         A : constant Integer := Cam_Arm (C, Cam);
      begin
         if A >= 0 then
            Append (T, "- this picture rides on arm " & Codec.Img (Natural (A) + 1)
                    & (if Arm_Has_Fingers (C, Natural (A)) then ": its fingers and grip stay put in this picture, the world moves when that arm moves"
                       else ": the world moves in this picture when that arm moves (arm " & Codec.Img (Natural (A) + 1) & " has no fingers)")
                    & ASCII.LF);
         end if;
      end;
      if Have_Named then
         for I in 0 .. Natural (C.Items.Length) - 1 loop
            --  槽号是【每台相机各一套】的,不许拿别台相机的槽号来和这台的"上次点名"比
            if C.Items (I).Kind in Thing | Thing_Remembered | Thing_Held
              and then C.Items (I).Cam = Cam
              and then C.Items (I).Slot = C.Wld.Cams (Cam).Named
            then
               Append (T, "- the thing you last named is " & Say_Item (C, I + 1) & ", now in cell " & Codec.Img (Cell_Of (C, Named_U, Named_V)) & ASCII.LF);
            end if;
         end loop;
      end if;
      --  没有手指的身体不说"你的手指之间"(DR2 2026-09-28:无人机每一轮都被告知 "there is NOTHING between your fingers right now")
      if Any_Fingers (C) then
         Append (T, "- there is " & (if C.Wld.Holding then "ALREADY something" else "NOTHING") & " between your fingers right now" & ASCII.LF);
      else
         Append (T, "- I have no fingers: when I pushed my grip channel from one end to the other, nothing in any of my pictures moved" & ASCII.LF);
      end if;
      Text := T;
   end Build_Listing;

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
   procedure Feel (C : in out Context; F : Plug.Frame) is
      function Clamp (X : Long_Float) return Long_Float is (Long_Float'Max (0.0, Long_Float'Min (1.0, X)));
   begin
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            if Cam_Arm (C, Cm) /= Integer (A) and then A < Natural (F.EE.Length) and then Track_Idx (C, A, Cm) < Natural (C.Zones.Length) then
               declare
                  Diff : Table.Vec;
                  Dist : Long_Float;
                  Si : constant Integer := Schema.Nearest (C.Sch, A, Cm, F.EE (A), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
               begin
                  if Si >= 0 then
                     declare
                        Sm : constant Schema.Sample := C.Sch.S (Natural (Si));
                        Tr : Zone_Track := C.Zones (Track_Idx (C, A, Cm));
                        Reach : Table.Vec := Unit_Reach;
                        Gp : constant Schema.Part_Pos := Sm.Parts (Chan.Per_Arm);   --  握合通道带的那块 = 手指
                        function Shift (Blob : Integer) return Table.Vec3 is
                           Idx : Integer := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, Blob);
                        begin
                           if Idx < 0 then
                              Idx := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, -1);
                           end if;
                           if Idx >= 0 then
                              for K in 0 .. Chan.Per_Arm - 1 loop
                                 Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                              end loop;
                              return Table.Predict (C.Tables (Natural (Idx)).E, Diff);
                           end if;
                           return Table.Zero3;
                        end Shift;
                        Sa : constant Table.Vec3 := Shift (0);
                        Sb : constant Table.Vec3 := Shift (1);
                     begin
                        if Gp.Valid then
                           Tr.Valid := True;
                           --  🔴🔴 外推炸了就不许用,退回样本里存的原样,并且不许再自称"我知道"。
                           --  箱上真数据(cal.json,臂1/相机0,64 个样本)存的是:两瓣 u = 0.8475 / 0.9847,
                           --  隔 0.137,都在画面右边 —— 样本是对的。而身体报给脑的是"左边,第 2 格和第 19 格,
                           --  隔四分之三个画面"。坏在这一行:胳膊离样本远时 Sa/Sb 这个外推量会炸,
                           --  Clamp 把它夹到 0 或 1 ⇒ 一瓣被夹到画面最左、另一瓣在别处。
                           --  而"我知不知道"只看【位姿差多大】,不看【算出来的结果合不合理】⇒
                           --  它一边给荒谬的位置一边说"我知道",十三炮的伺服全是拿这个位置算的。
                           --  判据零系数:算出来的两瓣间距,和【样本里那两瓣本来隔多远】比;
                           --  差得比它本身还大 ⇒ 这次外推不作数。
                           declare
                              Pu0 : constant Long_Float := Gp.B0u + Sa (0);
                              Pv0 : constant Long_Float := Gp.B0v + Sa (1);
                              Pu1 : constant Long_Float := Gp.B1u + Sb (0);
                              Pv1 : constant Long_Float := Gp.B1v + Sb (1);
                              Was : constant Long_Float :=
                                Sqrt ((Gp.B0u - Gp.B1u) ** 2 + (Gp.B0v - Gp.B1v) ** 2);
                              Now : constant Long_Float := Sqrt ((Pu0 - Pu1) ** 2 + (Pv0 - Pv1) ** 2);
                              Blew : constant Boolean :=
                                Gp.N_Blobs >= 2 and then Extrapolation_Blew (Was, Now);
                           begin
                              if Blew then
                                 --  外推不作数:用样本里的原样,并在下面把 Known 判掉(身体会因此先看一眼)
                                 Tr.Au := Gp.B0u; Tr.Av := Gp.B0v;
                                 Tr.Bu := Gp.B1u; Tr.Bv := Gp.B1v;
                              else
                                 Tr.Au := Clamp (Pu0); Tr.Av := Clamp (Pv0);
                                 Tr.Bu := Clamp (Pu1); Tr.Bv := Clamp (Pv1);
                              end if;
                              Tr.Blew_Up := Blew;
                           end;
                           Tr.Has_Lobes := Gp.N_Blobs >= 1;
                           if Gp.N_Blobs >= 2 then
                              Tr.Cu := (Tr.Au + Tr.Bu) / 2.0; Tr.Cv := (Tr.Av + Tr.Bv) / 2.0;
                           else
                              Tr.Cu := Tr.Au; Tr.Cv := Tr.Av;
                           end if;
                           if Gp.Z > 0.0 then
                              Tr.Z := Gp.Z + (if Gp.N_Blobs >= 2 then (Sa (2) + Sb (2)) / 2.0 else Sa (2));
                           end if;
                           Tr.Known := not Tr.Blew_Up;   --  外推炸过 ⇒ 这一处的位置不作数,别再自称知道
                           for K in 0 .. Chan.Per_Arm - 1 loop
                              if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                                 Tr.Known := False;
                              end if;
                           end loop;
                           Tr.Pieces (Chan.Per_Arm) := (True, Tr.Cu, Tr.Cv, Tr.Z, Gp.X0, Gp.Y0, Gp.X1, Gp.Y1, Gp.N_Blobs, Tr.Au, Tr.Av, Tr.Bu, Tr.Bv);
                           Tr.Pieces_Known (Chan.Per_Arm) := Tr.Known;
                        end if;
                        --  零件:样本里的位置 + 这个零件自己的响应表外推(没表 ⇒ 只在位姿几乎没差时算"知道")
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           if Sm.Parts (K).Valid then
                              declare
                                 Idx : constant Integer := Find_Effect (C, A, Cm, Piece_Pt, K, -1);
                                 Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Diff) else Table.Zero3);
                                 Pr : Schema.Part_Pos := Sm.Parts (K);
                                 Kn : Boolean := True;
                                 R2 : constant Table.Vec := (if Idx >= 0 then C.Tables (Natural (Idx)).Reach else Table.Zero_Vec);
                              begin
                                 Pr.Cu := Clamp (Sm.Parts (K).Cu + Sh (0)); Pr.Cv := Clamp (Sm.Parts (K).Cv + Sh (1));
                                 if Sm.Parts (K).Z > 0.0 then
                                    Pr.Z := Sm.Parts (K).Z + Sh (2);
                                 end if;
                                 --  框跟着形心平移
                                 Pr.X0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).X0) + Sh (0) * Long_Float (F.Cams (Cm).W)));
                                 Pr.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).W - 1), Long_Float (Sm.Parts (K).X1) + Sh (0) * Long_Float (F.Cams (Cm).W))));
                                 Pr.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).Y0) + Sh (1) * Long_Float (F.Cams (Cm).H)));
                                 Pr.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).H - 1), Long_Float (Sm.Parts (K).Y1) + Sh (1) * Long_Float (F.Cams (Cm).H))));
                                 for J in 0 .. Chan.Per_Arm - 1 loop
                                    if abs Diff (J) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + J)) * (if Idx >= 0 then Cap_Mult * Long_Float'Max (1.0, R2 (J)) else 1.0) then
                                       Kn := False;
                                    end if;
                                 end loop;
                                 Tr.Pieces (K) := Pr;
                                 Tr.Pieces_Known (K) := Kn;
                              end;
                           end if;
                        end loop;
                        C.Zones.Replace_Element (Track_Idx (C, A, Cm), Tr);
                     end;
                  end if;
               end;
            end if;
         end loop;
      end loop;
   end Feel;

   --  重新定位一个点:握区靠光流平流(世界相机)/固定(自己的手上相机);世界块重切后就近对上
   procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      --  这台相机这一拍、或者上一拍没有画面(插头留的空位,09-30):这一拍跟不了,照实记成跟丢,不拿空图去比
      if not Plug.Has_Picture (F.Cams (Cam)) or else Natural (Before.Length) /= Cw * Ch then
         P.Lost := True;
         return;
      end if;
      P.Lost := False;
      case P.Kind is
         when Piece_Pt =>
            if Cam_Arm (C, Cam) = Integer (P.Arm) then
               return;    --  自己的手上相机:握区是固定像素
            end if;
            declare
               --  半分辨率算光流(3 层 30 轮:次数),在这一点一小片取平均位移
               Hw : constant Natural := Cw / 2;
               Hh : constant Natural := Ch / 2;
               A, B : Buf;
               Fl : Flow.Field;
               Du, Dv : Long_Float;
               Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
               Old_Z : constant Long_Float := (if P.Z_Seen > 0.0 then P.Z_Seen else P.Z);
            begin
               A.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               B.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. Hw - 1 loop
                     A.Append (Before.Element ((2 * Y) * Cw + 2 * X));
                     B.Append (F.Cams (Cam).Gray.Element ((2 * Y) * Cw + 2 * X));
                  end loop;
               end loop;
               --  🔴 搜多宽由【这一步预计跑多远】定,不是写死 3 层。
               --  预计位移 = 从上一个位置到预测位置的距离(画幅)× 这半分辨率图的宽(像素)。
               --  预计跑得远 ⇒ 多加几层,最粗那层的位移落到一个像素以内,光流才找得准。
               Fl := Flow.Compute
                 (A, B, Hw, Hh,
                  Levels_For (Sqrt ((Pred_U - P.Cu) ** 2 + (Pred_V - P.Cv) ** 2) * Long_Float (Hw)),
                  30);
               --  取平均的那一片 = 张幅的四分之一(比例,无量纲),再小也有一个像素百分比
               Flow.Sample (Fl, P.Cu, P.Cv, Long_Float'Max (0.01, Z.Span * 0.25), Du, Dv);
               if Moved_Arm and then Sqrt (Du * Du + Dv * Dv) * Long_Float (Cw) < 0.5 then
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;      --  手臂动了,这儿画面却没流:跟丢的迹象,用预测
               else
                  P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu + Du));
                  P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv + Dv));
               end if;
               if F.Cams (Cam).Has_Depth then
                  declare
                     --  深度读在这一瓣自己的位置上(区心是两指之间的空,读到的是桌面);窗口 = 张幅的四分之一(比例,无量纲)
                     Win : constant Long_Float := Long_Float'Max (0.005, Z.Span * 0.25);
                     --  ⚠️ 撤回(HY 实测):曾经在这里除以"量出来的放大倍数"。**那是过头了** ——
                     --  那个倍数量的是"推一米读数变几米"(灵敏度),不是"读数的绝对尺度错几倍"。
                     --  拿灵敏度去除绝对值 ⇒ 1.4 m 被除成 0.003 m(手离镜头 3 毫米,物理上不可能),
                     --  差距当场从 1.366 炸到 2460。倍数只作【自知之明】用,不许改读数。
                     Zd : constant Long_Float :=
                       Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  begin
                     if not Picture.Is_Nan (Zd) then
                        --  一步之内深度跳了超过"预测的变化 + 这一点自己的读深抖动"⇒ 读到的不是我的手指,留预测。
                        --  🔴 没有预测值时这道闸以前【整条失效】(Pred_Z <= 0.0 直接短路成真),于是任何读数都收:
                        --  FS 实测手指的"离相机多远"一步从 0.454 m 跳到 0.010 m(离镜头一厘米,物理上不可能),
                        --  抓握的高低判据当场作废。没有预测就退回"一步最多变自己抖动那么多",而不是不管。
                        --  🔴 出路只给【我此刻真看得见自己】的时候用:墙是极其"可重复"的,
                        --  两次读到同一面墙也一致。HF 实测:点飘到画面角落之后,出路把 2.19 m 一路放到 5.109 m
                        --  (场景渲染出来的深度只到 4.359 m,物理上不可能)。跟丢的时候不许走出路。
                        if Depth_Ok (Zd, Old_Z, Pred_Z, P.Z_Noise, (if P.At_Edge then 0.0 else P.Z_Rej)) then
                           P.Z := Zd; P.Z_Seen := Zd; P.Z_Rej := 0.0;
                        else
                           P.Z_Rej := Zd;   --  记下这次被拒的:下一次要是又读到同一个数,就是它对、旧的陈了
                           if Pred_Z > 0.0 then
                              P.Z := Pred_Z;
                           else
                              P.Z := Old_Z;
                           end if;
                        end if;
                     end if;
                  end;
               end if;
            end;
         when Thing_Pt =>
            --  重新认脑点名的那一块:除了"位置近、大小差不多",还要"胖瘦像、颜色像"(ER:球 4703 px 和乐高 6334 px 大小分不开,跟错了)。
            --  🔴 认东西是脑的活:两块一样像的时候不许自己挑 —— 记 Unsure,回去问脑要号。
            declare
               Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
               Best, Second : Integer := -1;
               Bd, Sd : Long_Float := 1.0e9;
               Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;
               --  不像的程度:位置差几个跟踪窗 + 大小差几成 + 胖瘦差几成 + 灰度差几成(都是比例,无量纲)。
               --  🔴 大小只参与"像不像",不许当一票否决的硬门槛 —— 越走近它越大,硬门槛会在最该抓住的时候把它判丢(ET 实测)
               function Unlike (R : Picture.Region) return Long_Float is
                  D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2) / Long_Float'Max (Tol, 1.0e-9);
                  Sz : constant Long_Float := (if P.Count > 0 and then R.Count > 0 then
                                                  Long_Float (Integer'Max (R.Count, P.Count) - Integer'Min (R.Count, P.Count))
                                                  / Long_Float (Integer'Max (R.Count, P.Count))
                                               else 0.0);
                  E : constant Long_Float := abs (R.Elong - P.Elong) / Long_Float'Max (1.0, P.Elong);
                  G : constant Long_Float := (if P.Gray >= 0.0 then
                                                 abs (Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, R) - P.Gray) / 255.0
                                              else 0.0);
               begin
                  return D + Sz + E + G;
               end Unlike;
            begin
               --  🔴🔴 不许只在一个小窗里挑(2026-08-27 V2 实测):窗口比真实位移小的时候,
               --  它会在窗里挑一个【完全错误而读起来毫无异常】的位置,从不报错。
               --  改成【全画面都参与排序】—— 远的靠 Unlike 里那一项自己吃亏,但不再被一刀切掉。
               --  (我 2026-09-15 一度把探针幅度缩小来迁就小窗口,那是修反了:
               --   缩幅度等于把信号缩进噪声里,记录 D6 写着"探针步子太小 ⇒ 一列只解释掉 42%"。)
               for I in 0 .. Natural (Regs.Length) - 1 loop
                  declare
                     R : constant Picture.Region := Regs (I);
                     D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2);
                     U : constant Long_Float := Unlike (R);
                  begin
                     if True then
                        if U < Bd then
                           Sd := Bd; Second := Best;
                           Bd := U; Best := I;
                        elsif U < Sd then
                           Sd := U; Second := I;
                        end if;
                     end if;
                  end;
               end loop;
               if Best >= 0 then
                  declare
                     R : Picture.Region := Regs (Best);
                  begin
                     --  挨在一起的碎片算同一块:走近时物体会被切成几瓣(EV 落图:球裂成上沿+左右两条边)。
                     --  把外框挨着最像那块的碎片并进来,大小/形状按并集算
                     for Q of Regs loop
                        if Q.Count > 0 and then Q.X0 <= R.X1 and then Q.X1 >= R.X0 and then Q.Y0 <= R.Y1 and then Q.Y1 >= R.Y0 then
                           declare
                              Cx : constant Long_Float := (R.Cu * Long_Float (R.Count) + Q.Cu * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                              Cy : constant Long_Float := (R.Cv * Long_Float (R.Count) + Q.Cv * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                           begin
                              R.X0 := Natural'Min (R.X0, Q.X0); R.Y0 := Natural'Min (R.Y0, Q.Y0);
                              R.X1 := Natural'Max (R.X1, Q.X1); R.Y1 := Natural'Max (R.Y1, Q.Y1);
                              R.Cu := Cx; R.Cv := Cy;
                              R.Count := R.Count + Q.Count;
                              if Q.Depth > 0.0 and then (R.Depth <= 0.0 or else Q.Depth < R.Depth) then
                                 R.Depth := Q.Depth;   --  并起来之后取最近的那一片的远近
                              end if;
                           end;
                        end if;
                     end loop;
                     P.Z := R.Depth; P.Height := R.Height; P.Count := R.Count;
                     P.Box_W := Long_Float (R.X1 - R.X0) / Long_Float (Cw);
                     P.Box_H := Long_Float (R.Y1 - R.Y0) / Long_Float (Ch);
                     P.Cu := R.Cu; P.Cv := R.Cv;
                     --  🔴 按【面积】算,不按外接框:框被一颗杂散像素并进来就跳,面积几乎不动。
                     --  GF 实测:框版的"看着多大"散得比自己的均值还大 ⇒ 体检把它摘掉 ⇒ 腕相机里
                     --  一个能判距离的信号都不剩(2026-09-08 曾因此把这一项写死关掉,那是治标)。
                     P.Size := Sqrt (Long_Float (P.Count) / Long_Float'Max (1.0, Long_Float (Cw * Ch)));
                     P.Ang := 2.0 * Arctan (R.Av, R.Au);
                     P.Elong := R.Elong;
                     --  朝向算多少分,看这块有多"长条":圆的(长短轴一样)自动为零 —— 球没有朝向,给满分就是追噪声
                     if P.Wang > 0.0 then
                        P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, P.Elong));
                     end if;
                     --  只有真打平才叫分不开:第二像的和最像的差不到一成(比例,无量纲)。
                     --  松了会天天停下问(EU:球和它自己裂出来的小块也算"一样像")
                     P.Unsure := Second >= 0 and then Sd <= Bd * 1.1;
                  end;
               else
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
               end if;
            end;
      end case;
      --  🔴 统一收口:算出来的位置落在【画面边界上】就不是一次测量 —— 它是被夹回来的,
      --  真值在画面外。以前各处都写 Max(0.0, Min(1.0, …)) 把它夹回来却【不标跟丢】,
      --  于是解算一本正经地朝一个编出来的位置收敛,深度也跟着读到那儿的墙。
      --  HF 实测:点一路走到 (0.000,0.000) 画面左上角,深度读出 5.109 m ——
      --  而这个场景渲染出来的深度范围只有 0.646~4.359 m,物理上不可能;差距当场从 0.294 炸到 2.575。
      --  (这就是 5754c72 那条修法,a7ab7e9 回滚里丢掉的 13 条里我漏捞的那一条。)
      P.At_Edge := P.Cu <= 0.0 or else P.Cu >= 1.0 or else P.Cv <= 0.0 or else P.Cv >= 1.0;
      if P.At_Edge then
         P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu));
         P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv));
         P.Lost := True;
      end if;
   end Retrack;

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
                            Allow : Long_Float := 1.0) is
      --  Pts 空的时候 `Pts (0)` 当场越界,而它在声明区 ⇒ 异常记在【调用处】,
      --  栈里根本看不到这个子程序这一帧(实测查了半天)。空就当第 0 条胳膊,下面第一句直接回。
      Pts_Empty : constant Boolean := Natural (Pts.Length) = 0;
      Arm : constant Natural := (if Pts_Empty then 0 else Pts (0).Arm);
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Jaw : Floats;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Floor_Px : constant Long_Float := 4.0 / Long_Float (Cw);   --  跟踪地板:4 个像素(倍数,无量纲)
      Floor_Z : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      --  新加的两行也要有自己的噪声地板:探针那一点点幅度下,"看着多大/朝向"的变化可能比噪声还小,
      --  连符号都会是反的 ⇒ 身体照着反方向走(FB 实测:它一路往后退)。地板 = 静止两拍的抖动的 4 倍(倍数,无量纲)
      Floor_S : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Floor_A : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      --  一次推动只证明"它动过",证明不了"它稳"。同一个推法重复这么多次,量散布(次数,无量纲)。
      Reps_Wanted : constant := 3;
      N_Pts : constant Natural := Natural (Pts.Length);
      type Sum_Grid is array (0 .. N_Pts - 1, 0 .. Chan.Per_Arm - 1, 0 .. Table.Rows - 1) of Long_Float;
      S1 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值之和
      S2 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值平方和
      --  🔴🔴 来回对表(2026-08-27 NV3 第一次上机就抓到一个符号错:分歧 2.539 / 共识 0.019):
      --  同一根通道 +δ 走一遍、−δ 走回来一遍,两遍各除以【自己那一遍的实到】⇒ 结果应当相等。
      --  不相等 = 这一列不是一个测量(跟丢了 / 符号错了 / 关节翻支了),而光看去程那一遍看不出来。
      --  判据零系数:两遍的【分歧】要小于两遍的【共识】。
      B1 : Sum_Grid := [others => [others => [others => 0.0]]];   --  去程那一遍的列
      B2 : Sum_Grid := [others => [others => [others => 0.0]]];   --  回程那一遍的列
      Nb : array (0 .. Chan.Per_Arm - 1) of Natural := [others => 0];
      Agree_Out : Table.Vec := [others => -1.0];   --  每根通道:分歧 ÷ 共识(<1 才算稳)
      --  🔴 每(通道,行)自己的来回对账结果。对不上的【那一行】清零 = "这根通道对这一行没有意见"。
      --  一开始全是 True:没对过表的行照原样用(没量过不等于量出来是错的)。
      Row_Ok : array (0 .. Chan.Per_Arm - 1, 0 .. Table.Rows - 1) of Boolean := [others => [others => True]];
      Said_Wide : array (0 .. Chan.Per_Arm - 1) of Boolean := [others => False];
      Nrep : array (0 .. Chan.Per_Arm - 1) of Natural := [others => 0];
      --  🔴 上一轮(幅度的一半)这一通道最多的那个点跑了多远。加倍之后【一点没多跑】⇒ 再加也没用,
      --  这一列就是零 —— 零本身是一次正确的测量("这个通道不动它")。
      --  HC 实测:腕转那几根一路加码到 0.8192 rad(47°,owner 看 JA 视频原话"机械臂全程在发癫"),
      --  每一档都是 0.0000 画幅,加了五档等于白甩五次。
      Last_Ran : array (0 .. Chan.Per_Arm - 1) of Long_Float := [others => -1.0];
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
         Table.Reset (Effs (I), Chan.Per_Arm, 1.0);
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
            begin
               Table.Set_Prior (Effs (I), K, 100.0 / (Am * Am));   --  先验按探针幅度定(倍数,无量纲)
            end;
         end loop;
      end loop;
      --  深度读数地板:什么都不做,连着两拍在各点读深度
      if F.Cams (Cam).Has_Depth then
         declare
            Z1 : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => -1.0];
            Ok2 : Boolean;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
               Z1 (I) := Picture.Near_Depth (F.Cams (Pts (I).Cam).Depth, F.Cams (Pts (I).Cam).W, F.Cams (Pts (I).Cam).H,
                                             Pts (I).Cu, Pts (I).Cv, Long_Float'Max (0.005, Z.Span * 0.25));
            end loop;
            declare
               Was0 : constant Point_Vectors.Vector := Pts;
               Before0_All : Buf_Vectors.Vector := All_Gray (F);
            begin
               Selfmap.Idle (L, F, 1, Ok2);
               --  静止一拍,量"看着多大/朝向"自己抖多少
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     P2 : Point := Pts (I);
                  begin
                     Retrack (C, F, P2.Cam, Before0_All (P2.Cam), P2, Was0 (I).Cu, Was0 (I).Cv, False);
                     Floor_S (I) := Long_Float'Max (4.0 * abs (P2.Size - Was0 (I).Size), Size_Floor (Cw));
                     Floor_A (I) := Long_Float'Max (4.0 * abs (Wrap (P2.Ang - Was0 (I).Ang)), Ang_Floor (Was0 (I), Cw, Ch));
                  end;
               end loop;
            end;
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                  Z2 : constant Long_Float := Picture.Near_Depth (F.Cams (Pts (I).Cam).Depth, F.Cams (Pts (I).Cam).W, F.Cams (Pts (I).Cam).H,
                                                                 Pts (I).Cu, Pts (I).Cv, Long_Float'Max (0.005, Z.Span * 0.25));
                  Zr : constant Long_Float := (if Pts (I).Z > 0.0 then Pts (I).Z else 1.0);
               begin
                  --  地板 = 两拍读深抖动的 4 倍(倍数,无量纲),再小也有距离的百分之一(比例,无量纲)
                  if not Picture.Is_Nan (Z1 (I)) and then not Picture.Is_Nan (Z2) then
                     Floor_Z (I) := Long_Float'Max (4.0 * abs (Z1 (I) - Z2), 0.01 * Zr);
                  else
                     Floor_Z (I) := 0.01 * Zr;
                  end if;
                  --  同一个地板留在点上:高度是深度之差,判"离开了原来靠着的面"用的就是它
                  declare
                     P : Point := Pts (I);
                  begin
                     P.Z_Noise := Floor_Z (I);
                     Pts.Replace_Element (I, P);
                  end;
               end;
            end loop;
         end;
      end if;
      Ok := True;
      Put_Line ("[身]   这些点还没有响应表 ⇒ 六个通道各推一下量列(幅度从开机看得见的那一档起翻倍,到点真的动过地板为止)");
      --  六个通道一起解:转动不禁(owner 2026-09-07:禁了就永远和桌面平行,格斗全成直线)。让转动有对错的是"两根手指各自到位":
      --  转歪了必有一指不到位;让转动不比平移便宜的是按各自探针幅度计价。
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Chn : constant Natural := Arm * Chan.Per_Arm + K;
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
                           Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
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
                           Ran, Dz : Long_Float;
                        begin
                           Retrack (C, F, P.Cam, Before_All (P.Cam), P, W0.Cu, W0.Cv, True);
                           Ran := Sqrt ((P.Cu - W0.Cu) ** 2 + (P.Cv - W0.Cv) ** 2);
                           Dz := (if P.Z > 0.0 and then W0.Z > 0.0 then abs (P.Z - W0.Z) else 0.0);
                           Ran_Max := Long_Float'Max (Ran_Max, Ran);
                           if abs Deliv (K) > C.Map.EE_Noise and then (Ran >= Floor_Px or else Dz >= Floor_Z (I)) then
                              declare
                                 Col : Table.Vec3;
                              begin
                                 Col (0) := (P.Cu - W0.Cu) / Deliv (K);
                                 Col (1) := (P.Cv - W0.Cv) / Deliv (K);
                                 Col (2) := (if P.Z > 0.0 and then W0.Z > 0.0 then (P.Z - W0.Z) / Deliv (K) else 0.0);
                                 --  推一下这块看着变大变小多少、转了多少(圆的东西转不出来 ⇒ 这一列恒零 ⇒ 自动不参与)
                                 --  只有变化过了自己的噪声地板才敢写进表,否则这一格留零(留零 = 归一时这一行自动不参与)
                                 Col (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Floor_S (I)
                                             then (P.Size - W0.Size) / Deliv (K) else 0.0);
                                 Col (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Floor_A (I)
                                             then Wrap (P.Ang - W0.Ang) / Deliv (K) else 0.0);
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
                           Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 一个点也没动过地板(最多的跑了 " & Codec.Fmt (Ran_Max, 4) & " 画幅,地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
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
      if (for all K in 0 .. Chan.Per_Arm - 1 => not Trust (K)) then
         Put_Line ("[身]   🔴 这一段一根通道都没量到 ⇒ 我没有任何一条能用的走法。"
                   & "你给的幅度那一档不够我看清自己动了没有。");
         C.Blind_Say := S ("with the step size you gave me I could not see any of my channels move in this eye, "
                           & "so I have no usable way to move at all for this stretch - "
                           & "say a larger step, or judge this stretch with another eye");
      end if;
   end Probe_Effects;

   function Amount_Factor (A : Unbounded_String) return Long_Float is
     (if A = "small" then 0.25 elsif A = "large" then 1.0 else 0.5);   --  探针上限的几分之几(比例,无量纲)

   --  握区的点展开成瓣点(两瓣时):每一瓣各自到位,目标 = 区目标 + 这一瓣相对区心的偏移(保持此刻的开合朝向);
   --  歪了就有一瓣不到位,倾斜不用规则自然被罚。自己的手上相机里握区是固定像素,不展开。
   procedure Expand_Lobes (C : Context; F : Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      Out_P : Point_Vectors.Vector;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      for P of Pts loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam, Jaw_K_Of (P.Chan_K));
         begin
            --  🔴🔴 每一瓣一个接触点,瓣数【读身体量到的那个数】,不许写死。
            --  以前这里写 Z.N_Lobes = 2:7 指爪、软体臂、吸盘一律不展开 —— owner 揪出过一次,换了个地方又长出来。
            --  以前还写 Cam_Arm (C, Cam) /= P.Arm(手【自己】的相机里不展开):而 GM 全程跑在手腕相机里
            --  ⇒ 这段代码等于从没执行 ⇒ 全程只跟一个中心点 ⇒ 3 个自由度在零空间里乱走
            --  (6b3ad77 原话:手腕乱拧、球被转出画面、指尖落在球旁边 —— 一字不差就是 GM 的死法)。
            --  手自己的相机里瓣是固定像素,那正好:它们就是【要抓的东西的几侧必须去到的地方】,
            --  两点 ×(左右/上下/远近) = 6 个约束,正好按住 6 个通道。
            if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm and then Z.Valid and then Z.N_Lobes >= 2 then
               for Lb in 0 .. Z.N_Lobes - 1 loop
                  declare
                     Q : Point := P;
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                     --  瓣相对区心的偏移:身体图给了此刻各瓣位置就用它(转过的手瓣也跟着转),否则用开机量的
                     Lo : constant Zone.Lobe := Zone.Lobe_Of (Z, Lb);
                     Ou : constant Long_Float :=
                       (if Tr.Has_Lobes and then Lb <= 1 then (if Lb = 0 then Tr.Au else Tr.Bu) - Tr.Cu
                        else Lo.Cu - Z.Cu);
                     Ov : constant Long_Float :=
                       (if Tr.Has_Lobes and then Lb <= 1 then (if Lb = 0 then Tr.Av else Tr.Bv) - Tr.Cv
                        else Lo.Cv - Z.Cv);
                     Zd : Long_Float := P.Z;
                  begin
                     Q.Blob := Lb;
                     Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                     Q.Cu := P.Cu + Ou; Q.Cv := P.Cv + Ov;
                     Q.Tu := P.Tu + Ou; Q.Tv := P.Tv + Ov; Q.Tuv_Z := P.Tuv_Z;
                     if F.Cams (Cam).Has_Depth then
                        --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                        Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Q.Cu, Q.Cv, Long_Float'Max (0.005, Z.Span * 0.25));
                        if Picture.Is_Nan (Zd) then
                           Zd := P.Z;
                        end if;
                     end if;
                     Q.Z := Zd;
                     if Lb > 0 then
                        Q.Desc := Null_Unbounded_String;
                     end if;
                     Out_P.Append (Q);
                  end;
               end loop;
            elsif P.Kind = Thing_Pt and then Z.Valid and then Z.N_Lobes >= 2
              and then Zone.Lobe_Of (Z, 0).Valid and then Zone.Lobe_Of (Z, 1).Valid
            then
               --  🔴🔴 要抓的那一块,也沿【合拢方向】拆成和爪瓣一样多的点。
               --  只跟它的中心时,一个点只给 3 行(左右/上下/远近)去定 6 个通道 ⇒ 3 个自由度在零空间里
               --  自由乱走 —— 6b3ad77 原话:"手腕乱拧、球被转出画面、指尖落在球旁边",GU 实测:
               --  爪子到过距球 0.09 画幅,下一条命令又晃回去。两点 × 3 行 = 6 个约束,正好按住 6 个通道。
               --  拆几个不写死:爪有几瓣就几个(Z.N_Lobes,身体量的)。方向用爪的合拢方向(Z.Au,Z.Av,量的),
               --  半径用这一块自己的半宽(量的)。这是"跟这块沿合拢方向的两侧",讲的是【物体】的性质,
               --  和几根手指无关 —— owner 当年撤掉的是"写死两根手指"那一版实现,不是这个做法。
               begin
                  for Lb in 0 .. Z.N_Lobes - 1 loop
                     declare
                        Q : Point := P;
                        Lo : constant Zone.Lobe := Zone.Lobe_Of (Z, Lb);
                        --  🔴 拆开多远,不自己算半宽(那要写 ×0.5,是人拍的系数),
                        --  直接用【身体量到的瓣位相对区心的偏移】—— 球的接触点本来就该落在手指将来所在的地方。
                        Ou : constant Long_Float := Lo.Cu - Z.Cu;
                        Ov : constant Long_Float := Lo.Cv - Z.Cv;
                     begin
                        Q.Blob := Lb;
                        Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                        Q.Cu := P.Cu + Ou;
                        Q.Cv := P.Cv + Ov;
                        Q.Tu := P.Tu + Ou; Q.Tuv_Z := P.Tuv_Z;
                        Q.Tv := P.Tv + Ov;
                        if Lb > 0 then
                           Q.Desc := Null_Unbounded_String;
                        end if;
                        Out_P.Append (Q);
                     end;
                  end loop;
               end;
            else
               Out_P.Append (P);
            end if;
         end;
      end loop;
      Pts := Out_P;
   end Expand_Lobes;

   --  ── 一段 = 反复做五件事:①打算怎么走 ②走 ③看 ④学 ⑤判 ──
   --  每件事一个小过程;这一步发生了什么全记在 Note 里(字段名就是人话),五件事之间只靠它说话。
   --  🔴 "到了没到"只有脑能判(owner 2026-09-14)。身体只报量到的事件,不报"我觉得到了"。
   --  身体唯一能结束一节的理由,是脑写的那个 until(含它给的步数),外加"我物理上做不到"。
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
      function Watch_Things (Fr : Plug.Frame) return Boolean is
         Regs : Picture.Regions;
         Have : Boolean := False;
      begin
         for P of Pts loop
            if P.Kind = Thing_Pt then
               if not Have then
                  Regs := Cut_Things (C, Fr, Cam);
                  Have := True;
               end if;
               declare
                  Found : Boolean := False;
                  Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;   --  和 Retrack 同一个认领半径(比例,无量纲)
               begin
                  for R of Regs loop
                     if R.Count * 3 >= P.Count and then R.Count <= P.Count * 3 and then Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Tol then
                        Found := True;
                        if R.Cu < Track_Win or else R.Cu > 1.0 - Track_Win or else R.Cv < Track_Win or else R.Cv > 1.0 - Track_Win then
                           Note.Halted := True;
                           return True;
                        end if;
                     end if;
                  end loop;
                  if not Found then
                     Note.Halted := True;
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Watch_Things;

      --  落一张图,把这一拍切出来的块和被跟的点画上去(出岔子时给人看)
      procedure Dump_Picture (Tag : String) is
      begin
         if C.Dump_Dir = "" then
            return;
         end if;
         declare
            RGB : Buf := F.Cams (Cam).RGB;
            Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            N : Natural := 0;
         begin
            for R of Regs loop
               N := N + 1;
               Draw.Numbered_Box (RGB, Cw, Ch, R.X0, R.Y0, R.X1, R.Y1, N, Draw.Green, 2);
            end loop;
            for P of Pts loop
               declare
                  Hw : constant Long_Float := Long_Float'Max (P.Box_W, Track_Win) * 0.5;   --  框(没有就用一个跟踪窗;比例,无量纲)
                  Hh : constant Long_Float := Long_Float'Max (P.Box_H, Track_Win) * 0.5;
               begin
                  Draw.Numbered_Box (RGB, Cw, Ch,
                                     Natural (Long_Float'Max (0.0, (P.Cu - Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Max (0.0, (P.Cv - Hh) * Long_Float (Ch))),
                                     Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + Hh) * Long_Float (Ch))),
                                     0, Draw.Pink, 2);
               end;
            end loop;
            Codec.Write_BMP (To_String (C.Dump_Dir) & "/" & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp", RGB, Cw, Ch);
            Put_Line ("[身]     落图 " & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp(切出" & Natural'Image (N) & " 块)");
         end;
      end Dump_Picture;

      --  段前:每个点的响应表 —— 装回(必须是同一位姿、同样拿着东西、至少有一列信得过)或当场重量
      procedure Ready_Tables (Ok_Out : out Boolean) is
         Need : Boolean := False;
      begin
         Ok_Out := True;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Idx : constant Integer := Find_Effect (C, Arm, Pts (I).Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob);
            begin
               if Idx >= 0 and then (for some K in 0 .. Chan.Per_Arm - 1 => C.Tables (Natural (Idx)).Trust (K))
                 and then C.Tables (Natural (Idx)).Has_Pose
                 and then (for all K in 0 .. Chan.Per_Arm - 1 =>
                             abs Chan.Delivered (C.Tables (Natural (Idx)).Pose, F.EE (Arm)) (K)
                             <= Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K)) * Cap_Mult * C.Tables (Natural (Idx)).Reach (K))
               then
                  Effs (I) := C.Tables (Natural (Idx)).E;
                  Trusts (I) := C.Tables (Natural (Idx)).Trust;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                  end loop;
               else
                  Need := True;
               end if;
            end;
         end loop;
         if Need then
            declare
               Trust : Table.Mask;
               Ok : Boolean;
            begin
               Probe_Effects (L, C, F, Cam, Pts, Effs, Trust, Ok, Amount * Cap_Mult);
               if not Ok then
                  Ok_Out := False;
                  return;
               end if;
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  Trusts (I) := Trust;
                  Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust, Unit_Reach, F.EE (Arm), True);
               end loop;
               --  🔴 量完就把表打出来:每根通道推 +1,画面左右跑多少 / 离相机远近变多少(正=变远)。
               --  方向对不对全看正负。HU 实测:左右已经对到 0.008 m,而远近误差越走越大(1.357→1.711),
               --  手朝相机走而球在 3.5 m 外 —— 光看误差分不出是表的符号反了还是解算没得选。
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     Ln : Unbounded_String :=
                       S ("[身]   表 点" & Codec.Img (I) & ":");
                  begin
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        Append (Ln, " ch" & Codec.Img (Arm * Chan.Per_Arm + K) & "(左右"
                                & Codec.Fmt (Effs (I).B (K, 0), 3) & " 远近"
                                & Codec.Fmt (Effs (I).B (K, 2), 3)
                                & (if Trust (K) then "" else " 没证过") & ")");
                     end loop;
                     Put_Line (To_String (Ln));
                     --  🔴🔴 体检:平移通道推一米,远近最多变一米。绝对值 > 1 = 物理上不可能。
                     --  取所有平移通道里最大的那个,就是【我的深度读数被放大了几倍】的下界 ——
                     --  这就是拿自己的胳膊当尺子:我知道自己走了几米,也看得见深度读数变了多少。
                     --  🔴 分两句说:信得过的列里最坏多少 · 【已经作废的列里】最坏多少。
                     --  只统计信得过的列会把病情说小:HY 实测报"1.4 倍",而同一炮里有一根
                     --  命令 0.0032 实到 0.0020、深度却变 0.7605 ⇒ 380 倍 —— 那一根已被来回对表判死,
                     --  于是不参与,病情就被漏报了。作废的那些不参与修正,但必须说出来给脑看病。
                     declare
                        Worst : Long_Float := 0.0;
                        Worst_Dead : Long_Float := 0.0;
                     begin
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           if Depth_Scale_Bad (Effs (I).B (K, 2)) then
                              if Trust (K) then
                                 Worst := Long_Float'Max (Worst, abs Effs (I).B (K, 2));
                              else
                                 Worst_Dead := Long_Float'Max (Worst_Dead, abs Effs (I).B (K, 2));
                              end if;
                           end if;
                        end loop;
                        if Worst_Dead > 0.0 then
                           Put_Line ("[身]   🔴 体检(已作废的那些列里):最坏的一根是 我真走一米、深度读数变 "
                                     & Codec.Fmt (Worst_Dead, 1) & " 米 —— 它已经被来回对表判死了,不参与,"
                                     & "但这就是我的距离感到底有多烂。");
                        end if;
                        if Worst > 0.0 then
                           C.Depth_Scale := Long_Float'Max (C.Depth_Scale, Worst);
                           Put_Line ("[身]   🔴 体检:我真走一米,深度读数变了 " & Codec.Fmt (Worst, 1)
                                     & " 米 —— 物理上最多一米。我的深度读数被放大了至少 "
                                     & Codec.Fmt (Worst, 1) & " 倍,这一维我不当真的量看。");
                           C.Blind_Say := S ("I checked myself: one metre of my own real motion changes my depth reading by "
                                             & Codec.Fmt (Worst, 1) & " metres, and the most that is physically possible is one. "
                                             & "So my sense of distance is inflated by at least that much and I do not trust it "
                                             & "as a real measurement - I am using it only for direction, not for how far.");
                        end if;
                     end;
                  end;
               end loop;
            end;
         end if;
      end Ready_Tables;

      --  ①a 定目标:每个点的五样差距,各自除以"推一步最多能改多少",变成"还差几步"
      --  🔴 这一段里,哪几个点的"远近"那一行是【米】(尺子量出来的),不是深度读数
      In_Metres : array (0 .. Natural'Max (0, Natural (Pts.Length) - 1)) of Boolean := [others => False];
      --  这一个点的五行里,有没有【这一段要管、却算不出"还差几步"】的行(见 Point.No_Scale)
      No_Scale_Row : Boolean := False;

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

      --  ①b 定额度:各通道这一步最多走多少 —— 两条取小:①眼睛跟得住的那么多(表说走多少画面跑满一个跟踪窗)
      --  ②自己那一档 × 核实过的倍数。只用①,在画面里几乎不动的通道会拿到无限额度(甩腕 2 rad)
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

      --  ①c 修步子:缩到眼睛跟得住,且不把被跟的东西推出视野、不让我身上任何一块压到"不许碰"的框
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

      --  ① 打算怎么走 = 定目标 → 定额度 → 修步子
      procedure Plan is
         Terms : Table.Term_Vectors.Vector;
         Solved : Boolean;
      begin
         Note := (others => <>);
         Aim (Terms);
         Budget (Terms, Solved);
         --  🔴🔴 "算出来了,而算出来的是一动不动" 和 "算不出来" 是同一件事(HZ 2026-09-15 实测)。
         --  HZ:6 根通道被判死 4 根,活下来的两根左右都是 0.000 ⇒ 解算每一步都返回全零、还报成功,
         --  于是身体连着 10 步一根关节都没转,差距 0.479 → 0.565(还涨了),而日志每一行都绿。
         --  一步命令全零 = 这一步没走。不许把它当成"走过了"。
         if Solved and then (for all K in 0 .. Chan.Per_Arm - 1 => abs Note.Cmd (K) <= 0.0) then
            Solved := False;
         end if;
         if not Solved then
            --  🔴 解不出来也不许停:用手上最好的那个估计推一步,并说清楚这一步是硬凑的。
            --  🔴 挑哪一根:能用的里面【画面里动得最多】的那一根 —— 以前写死推 0 号,
            --  0 号不能用就等于什么都不推,"不许停"变成了一句空话(HZ 实测全零 10 步)。
            for K in 0 .. Chan.Per_Arm - 1 loop
               Note.Cmd (K) := 0.0;
            end loop;
            declare
               Best : Integer := -1;
               Best_Px : Long_Float := -1.0;
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  if Note.Active (K) then
                     declare
                        Px : Long_Float := 0.0;
                     begin
                        for T of Terms loop
                           Px := Long_Float'Max
                             (Px, Sqrt (T.E.B (K, 0) ** 2 + T.E.B (K, 1) ** 2));
                        end loop;
                        if Px > Best_Px then
                           Best_Px := Px;
                           Best := K;
                        end if;
                     end;
                  end if;
               end loop;
               --  一根能用的都没有 ⇒ 还是要动:推身上量到过幅度的第一根,并说清楚这是硬凑的。
               if Best < 0 then
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if C.Map.Seen (Arm * Chan.Per_Arm + K) then
                        Best := K;
                        exit;
                     end if;
                  end loop;
               end if;
               if Best >= 0 then
                  Note.Cmd (Natural (Best)) :=
                    C.Map.Amp (Arm * Chan.Per_Arm + Natural (Best)) * Amount;
                  Note.Active (Natural (Best)) := True;
               end if;
            end;
            C.Blind_Say := S ("I could not work out which channels to push, so this step was a guess");
            return;
         end if;
         Trim;
      end Plan;

      --  ② 走:记下走之前的样子,发命令,途中盯着
      procedure Walk (Ok_Out : out Boolean) is
      begin
         Before_All := All_Gray (F);
         Was := Pts;
         Was_EE := F.EE (Arm);
         Was_Regs := (if Cam_Arm (C, Cam) /= Integer (Arm) then Cut_Things (C, F, Cam) else Picture.Region_Vectors.Empty_Vector);
         Step_Arm (L, C, F, Arm, Note.Cmd, Jaw, Note.Got, Ok_Out, C.Fast, Watch_Things'Unrestricted_Access);
         Beats := Since (L, Beats0);
         if Note.Halted then
            Put_Line ("[身]     途中眼睛叫停:被跟的东西快出画面或看不见了,这一步没走完");
         end if;
         if not Ok_Out then
            return;
         end if;
         Steps_Taken := Steps_Taken + 1;
         declare
            Sv : Backup.Step_Vec := [others => 0.0];
         begin
            for K in 0 .. Chan.Per_Arm - 1 loop
               Sv (K) := Note.Got (K);
               --  🔴 学死区:命令发了而身体没动 ⇒ 这一档不够,抬上去;真动了 ⇒ 说明这一档够,压下来。
               --  抬到刚才那一档的一半再加一次(=1.5 倍,倍数无量纲),压到刚好走成的那一档。
               declare
                  Cn : constant Natural := Arm * Chan.Per_Arm + K;
                  Half : constant Long_Float := abs Note.Cmd (K) / 2.0;
               begin
                  if Cn < Natural (C.Dead.Length) and then abs Note.Cmd (K) > C.Map.EE_Noise then
                     if abs Note.Got (K) <= C.Map.EE_Noise then
                        C.Dead.Replace_Element
                          (Cn, Long_Float'Max (C.Dead.Element (Cn), abs Note.Cmd (K) + Half));
                     else
                        C.Dead.Replace_Element
                          (Cn, Long_Float'Min (C.Dead.Element (Cn), abs Note.Cmd (K)));
                     end if;
                  end if;
               end;
            end loop;
            Backup.Remember (Ring, Sv);
         end;
         Note.Pic_Delta := Long_Float (Picture.Max_Diff (Before_All (Cam), F.Cams (Cam).Gray));
      end Walk;

      --  ③ 看:每个点现在在哪。我的零件(别人的相机里)先按位姿"感觉",熟地让眼睛核对,生地或大步就抖一下去看;
      --  世界里的块每步重切就近对上。全都认不到才叫看不见。
      procedure Look is
         Need_Refind : Boolean := False;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : Point := Pts (I);
               W0 : constant Point := Was (I);
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
            begin
               P.Has_Meas := False;
               if P.Kind = Piece_Pt and then Cam_Arm (C, P.Cam) /= Integer (P.Arm) then
                  declare
                     Diff : Table.Vec;
                     Dist : Long_Float;
                     Si : constant Integer := Schema.Nearest (C.Sch, P.Arm, Cam, F.EE (P.Arm), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
                     Familiar : Boolean := False;
                     In_Map : Boolean := False;
                  begin
                     if Si >= 0 then
                        In_Map := P.Chan_K <= Chan.Per_Arm and then C.Sch.S (Natural (Si)).Parts (P.Chan_K).Valid
                                  and then (P.Blob < 0 or else C.Sch.S (Natural (Si)).Parts (P.Chan_K).N_Blobs > Natural (P.Blob));
                     end if;
                     if In_Map then
                        declare
                           Gp : constant Schema.Part_Pos := C.Sch.S (Natural (Si)).Parts (P.Chan_K);
                           Pm : constant Table.Vec3 := Table.Predict (Effs (I), Diff);
                           Su : constant Long_Float := (if P.Blob = 1 then Gp.B1u elsif P.Blob = 0 then Gp.B0u else Gp.Cu);
                           Sv : constant Long_Float := (if P.Blob = 1 then Gp.B1v elsif P.Blob = 0 then Gp.B0v else Gp.Cv);
                        begin
                           P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, Su + Pm (0)));
                           P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, Sv + Pm (1)));
                           --  🔴 距离不可能是负的(物理,不是人拍的门槛)。这里只检查了【旧】深度是正的,
                           --  没检查【算出来的新】深度 —— 画面坐标 u/v 都夹在 [0,1] 里,唯独深度一个夹子都没有。
                           --  GV 实测:预测把它推成 -0.526 ⇒ 远近整行作废 ⇒ 身体只在画面上对齐、
                           --  停在离球 0.08 画幅处还报"差 0.005 m"。(同一个坑 LAB 记过:ca3641b。)
                           if Gp.Z > 0.0 and then Gp.Z + Pm (2) > 0.0 then
                              P.Z := Gp.Z + Pm (2);
                           end if;
                           Familiar := True;
                           for K in 0 .. Chan.Per_Arm - 1 loop
                              if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (P.Arm * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                                 Familiar := False;
                              end if;
                           end loop;
                        end;
                     else
                        P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cu + Pr (0)));
                        P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cv + Pr (1)));
                        --  同上:算出来的新深度必须仍是正的,否则这一步的预测就是错的,宁可留着旧值
                        if W0.Z > 0.0 and then W0.Z + Pr (2) > 0.0 then
                           P.Z := W0.Z + Pr (2);
                        end if;
                     end if;
                     --  🔴🔴 位置从姿态表里查出来之后,【深度要在深度图上就地重读】。
                     --  以前这一路的 Z 全是姿态表里存的那个数加上表的预测 —— 也就是【猜】出来的,
                     --  从来没被眼睛校过。IY 实测(真深度也开着):球读 0.640 m,而挨着它的指尖读 2.40 m,
                     --  差了四倍;每一步 Z 平滑地变 0.007,像预测不像测量。于是"远近"那一行永远差着,
                     --  手在前后方向上要么不动要么一路顶,合手全是空的。
                     --  LAB 判定这就是 FO"夹太靠上、一合把球顶飞"的根子。修法(4c24742 + 闸 ad6d76e)
                     --  在 a7ab7e9 回滚里被一起退掉了,这里捞回来。
                     if F.Cams (P.Cam).Has_Depth then
                        declare
                           Zn : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam);
                           --  🔴 读"我离相机多远"要读在【我这一瓣自己身上】。
                           --  区心是【两指之间的空】,那儿什么都没有,读到的是它背后的东西 ——
                           --  HW 实测:爪子读 2.19 m 而球读 3.53 m,差了 1.34 m;桌面上不可能有这么大的高度差,
                           --  是读窗落在空处、读到了更靠近相机的自己的大臂。
                           --  (LAB 3b9d570 原话:"区心是两指之间的空,读到的是桌面"。)
                           --  瓣是量出来的:一瓣=吸盘,两瓣=两指,七瓣=七指,这里取第一瓣的位置,零身体假设。
                           Lb : constant Zone.Lobe := Zone.Lobe_Of (Zn, 0);
                           Ru : constant Long_Float := (if P.Kind = Piece_Pt and then P.Blob < 0
                                                        and then Zn.Valid and then Lb.Valid then Lb.Cu else P.Cu);
                           Rv : constant Long_Float := (if P.Kind = Piece_Pt and then P.Blob < 0
                                                        and then Zn.Valid and then Lb.Valid then Lb.Cv else P.Cv);
                           Zd : constant Long_Float :=
                             Picture.Near_Depth (F.Cams (P.Cam).Depth, F.Cams (P.Cam).W, F.Cams (P.Cam).H,
                                                 Ru, Rv, Lobe_Win (Zn, F.Cams (P.Cam).W, F.Cams (P.Cam).H));
                           --  🔴 闸盯【上一次真读到的】远近,不是 P.Z —— P.Z 可能是按位姿猜的、从没被眼睛校过
                           Old_Z : constant Long_Float := (if P.Z_Seen > 0.0 then P.Z_Seen else P.Z);
                        begin
                           --  🔴 收读数前先过闸:读窗里同时有指头和它【后面那个面】时,读数会在两者之间来回跳
                           --  (JD 实测:指尖深度在 0.61 和 0.45 之间几乎每步翻一次,差 16 cm,而它在画面里几乎没动
                           --   ⇒ 前后那一维的误差每步翻符号 ⇒ 手被拉过去又拉回来,视频里就是发癫)。
                           --  原版这道闸写的是"表预测的变化 + 距离的【一成】",那个一成是人拍的;
                           --  换成这一点自己量到的深度抖动地板(Z_Noise),零系数,而且比一成更对。
                           if not Picture.Is_Nan (Zd) and then Zd > 0.0 then
                              if Depth_Ok (Zd, Old_Z, Old_Z + Pr (2), P.Z_Noise,
                                           (if P.At_Edge then 0.0 else P.Z_Rej))
                              then
                                 P.Z := Zd; P.Z_Seen := Zd; P.Z_Rej := 0.0;
                              else
                                 --  🔴 打出来:读到多少、上次真读到多少、表预测这一步走多少、抖动多少、上次被拒的是多少。
                                 --  HR 实测:深度 8 推纹丝不动 2.062,而点在画面里确实在动 ⇒ 每一读都被挡,
                                 --  光看"差 1.498 m"看不出是挡的还是真没动。
                                 Put_Line ("[身]     深度被挡:读到 " & Codec.Fmt (Zd, 3)
                                           & " · 上次真读到 " & Codec.Fmt (Old_Z, 3)
                                           & " · 表说这一步走 " & Codec.Fmt (Pr (2), 3)
                                           & " · 这一点读深抖动 " & Codec.Fmt (P.Z_Noise, 3)
                                           & " · 上次被拒 " & Codec.Fmt (P.Z_Rej, 3)
                                           & (if P.At_Edge then " · 此刻贴在画面边上(出路不给)" else ""));
                                 P.Z_Rej := Zd;   --  记下被拒的那个数;连着两次一致就说明旧基准陈了
                                 P.Z := Old_Z;    --  这一帧读到的是别的面,留上一次真读到的
                              end if;
                           end if;
                        end;
                     end if;
                     if Note.Big_Step or else not Familiar then
                        P.Lost := True;
                        Need_Refind := True;
                     else
                        declare
                           Q : Point := W0;
                           Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam, Jaw_K_Of (P.Chan_K));
                        begin
                           Retrack (C, F, P.Cam, Before_All (P.Cam), Q, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
                           --  眼睛和图对不上(差过张幅的四分之一,比例,无量纲;再小也有两个跟踪地板)⇒ 去看
                           if Q.Lost or else Sqrt ((Q.Cu - P.Cu) ** 2 + (Q.Cv - P.Cv) ** 2) > Long_Float'Max (Z.Span * 0.25, Fl.Track * 2.0) then
                              P.Lost := True;
                              Need_Refind := True;
                           else
                              P.Lost := False;
                              P.Has_Meas := True; P.Meas_U := Q.Cu; P.Meas_V := Q.Cv; P.Meas_Z := Q.Z;
                           end if;
                        end;
                     end if;
                  end;
               else
                  Retrack (C, F, P.Cam, Before_All (P.Cam), P, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
               end if;
               --  🔴 认错了东西要说出来,不许悄悄换目标。
               --  GM 实测:被跟的那块从 (0.44,0.93) 深 0.81 m 一步跳到 (0.21,0.60) 深 2.13 m —— 那是球后面的墙。
               --  身体自己知道(信表从 0.94 掉到 0.31),脑一个字没听到,然后追着墙把关节顶死 60 步。
               --  判据不用新系数,用【物理上不可能】:这一步就算把额度用满,表说这块最多能跑多远?
               --  跑得比那还远 ⇒ 不是同一个东西。
               if not P.Lost and then P.Z > 0.0 and then W0.Z > 0.0 then
                  declare
                     Most : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cap);
                     Jump : constant Long_Float := abs (P.Z - W0.Z);
                  begin
                     if Jump > abs (Most (2)) + Long_Float'Max (0.0, P.Z_Noise) then
                        C.Blind_Say := S ("the thing I am tracking jumped further in one push than any push of mine could move it"
                                          & " - I have probably locked onto something else, and I kept going");
                        Put_Line ("[身]     认错了?这一步它跑了 " & Mm (Jump) & ",而用满额度最多也只跑得动 "
                                  & Mm (abs (Most (2))) & "(读深抖动 " & Mm (P.Z_Noise) & ")");
                     end if;
                  end;
               end if;
               Pts.Replace_Element (I, P);
            end;
         end loop;
         if Need_Refind then
            Refind_Pieces (L, C, F, Cam, Pts);
            Beats := Since (L, Beats0);
         end if;
         Note.Lost_All := True;
         for P of Pts loop
            if not P.Lost then
               Note.Lost_All := False;
            end if;
            if P.Unsure then
               Note.Unsure := True;
            end if;
         end loop;
      end Look;

      --  ④ 学:拿这一步的实际结果修表;判"整步没照做";按通道各自放宽/收紧步幅;看有没有碰到别的东西
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

   --  合/张:最多 Max_Iter 拍,或到画面不再变;Sweep_Cam >= 0 时把那台相机里动过的像素累进 Sweep(手指自己扫过的地方)
   procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm, K : Natural; Target : Long_Float; Max_Iter : Natural;
                        Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is
      Jaw : Floats;
      Prev : Long_Float := (if Selfmap.Has_Jaw (F, Arm, K) then Selfmap.Jaw_Of (F, Arm, K) else 0.0);
      Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
      Cm : Plug.Cmd;
      --  "停住"只看读数和【这条臂自己那只眼】(手指就在它里面);别的眼里别的东西在动跟合爪无关
      --  (S1 2026-09-23 实测:等三台相机全静止,一次合爪 35 拍,官方一集只有 200 拍)。没有自己的眼就看全部
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      function Own_Eye_Still return Boolean is
        (if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) and then Natural (Hc) < Natural (Prev_Cams.Length)
            and then Natural (Hc) < Natural (C.Map.Floors.Length)
         then Selfmap.Picture_Still (C.Map, Prev_Cams (Natural (Hc)), F.Cams (Natural (Hc)), Natural (Hc))
         else Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams));
   begin
      Steps := 0;
      Reading := Prev;
      if not Selfmap.Has_Jaw (F, Arm, K) then
         --  这一拍没有这个通道的读数:不推(不拿编的数当读数、也不拿它当别的通道的目标;09-30 原来补 1.0 = x5"1 = 张开")
         Put_Line ("[身] ✋ 第" & Codec.Img (Arm + 1) & " 只手第 " & Codec.Img (K + 1) & " 个抓握通道这一拍没有读数 ⇒ 不合不张");
         return;
      end if;
      --  只动点名的那一个抓握通道,其余保持它们此刻的读数(五指手:合一根不牵动另外四根)
      declare
         Rest : constant Floats := Selfmap.Jaw_All (F, Arm);
      begin
         for I in 0 .. Natural (Rest.Length) - 1 loop
            Jaw.Append (if I = K then Target else Rest (I));
         end loop;
      end;
      --  读数是命令的回声,"停住"只认画面:每台相机连着两拍不变
      for I in 1 .. Max_Iter loop
         Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
         exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
         Steps := I;
         if Selfmap.Has_Jaw (F, Arm, K) then
            Reading := Selfmap.Jaw_Of (F, Arm, K);
         else
            Still := 0;   --  这一拍没读数:算不上"停住"
         end if;
         if Sweep_Cam >= 0 and then Natural (Sweep_Cam) < Natural (F.Cams.Length) and then Natural (Sweep_Cam) < Natural (C.Map.Floors.Length) then
            Sweep := Picture.Either (Sweep, Picture.Moved (Prev_Cams (Natural (Sweep_Cam)).Gray, F.Cams (Natural (Sweep_Cam)).Gray, C.Map.Floors (Natural (Sweep_Cam))));
         end if;
         if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Own_Eye_Still then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev := Reading;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then I >= 3;
      end loop;
   end Jaw_Sweep;

   --  合/张到读数不再变
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float;
                       K : Natural := 0) is
      None : Bools;
   begin
      Jaw_Sweep (L, C, F, Arm, K, Target, 40, -1, None, Steps, Reading);
   end Move_Jaw;

   --  生地/大步之后在世界相机里重新看见自己:手指 = 抖一下手指(合几拍再张回来),零件 = 推一下它自己的通道再推回来;
   --  动过的像素就是它,每个点认离预测最近的那团。抖的幅度不是常数:手指合"量出来的稳定拍数"那么久;零件推开机看得见的那一档。认不到的留预测、记 Lost。
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      --  Pts 空的时候 `Pts (0)` 当场越界,而它在声明区 ⇒ 异常记在【调用处】,
      --  栈里根本看不到这个子程序这一帧(实测查了半天)。空就当第 0 条胳膊,下面第一句直接回。
      Pts_Empty : constant Boolean := Natural (Pts.Length) = 0;
      Arm : constant Natural := (if Pts_Empty then 0 else Pts (0).Arm);
      --  🔴🔴 抖之前先记下【我猜的】位置。抖完拿"我看到的"和它一比,就是这具身体
      --  唯一一次能自己验证"我的手在哪"的机会 —— 而它一直没比过。
      --  十三炮的终局全是"伺服往错的方向推",而错的源头就是这个猜出来的位置
      --  (GW 实测:右臂两根手指被放到画面左边、相隔四分之三个画面)。
      Guess_U : constant Long_Float := Pts (0).Cu;
      Guess_V : constant Long_Float := Pts (0).Cv;
      Guess_Z : constant Long_Float := Pts (0).Z;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      Steps_J : Natural;
      Reading : Long_Float;
      Any_Fingers : Boolean := False;
      Jaw : Floats;
      --  在累积的"动过"掩膜里给一个点认领最近的一团;Tol = 认领半径(画幅比例,无量纲)
      procedure Claim (P : in out Point; Tol, Win : Long_Float; Taken : in out Bools; Regs : Picture.Regions) is
         Best : Integer := -1;
         Bd : Long_Float := 1.0e9;
      begin
         for R in 0 .. Natural (Regs.Length) - 1 loop
            declare
               D : constant Long_Float := Sqrt ((Regs (R).Cu - P.Cu) ** 2 + (Regs (R).Cv - P.Cv) ** 2);
            begin
               if not Taken (R) and then D <= Tol and then D < Bd then
                  Bd := D; Best := R;
               end if;
            end;
         end loop;
         if Best >= 0 then
            Taken.Replace_Element (Natural (Best), True);
            declare
               Old_Z : constant Long_Float := P.Z;
               Zd : Long_Float;
            begin
               P.Cu := Regs (Best).Cu; P.Cv := Regs (Best).Cv; P.Lost := False;
               P.Box_W := Long_Float (Regs (Best).X1 - Regs (Best).X0) / Long_Float (Cw);
               P.Box_H := Long_Float (Regs (Best).Y1 - Regs (Best).Y0) / Long_Float (Ch);
               if F.Cams (Cam).Has_Depth then
                  --  深度一步跳过"距离的一成"(比例,无量纲)就是读到别的东西了
                  Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  if not Picture.Is_Nan (Zd) and then (Old_Z <= 0.0 or else abs (Zd - Old_Z) <= 0.1 * Old_Z) then
                     P.Z := Zd;
                  end if;
               end if;
            end;
         else
            P.Lost := True;
         end if;
      end Claim;
   begin
      if Pts_Empty then
         return;
      end if;
      --  这条臂此刻的抓握读数原样当目标(挪手的时候手指不动);这一拍没读数就不带(插头按"这一集给过的最后一个目标"保持)
      Jaw := Selfmap.Jaw_All (F, Arm);
      for P of Pts loop
         if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
            Any_Fingers := True;
         end if;
      end loop;
      if Any_Fingers then
         declare
            Sweep : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
            Regs : Picture.Regions;
            Taken : Bools;
         begin
            --  这些点分别属于哪几个抓握通道,就抖哪几个(五指手:只抖被跟着的那几根)
            for Kk in 0 .. Jaws_Of (C, Arm) - 1 loop
               declare
                  Wanted : Boolean := False;
               begin
                  for P of Pts loop
                     if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm + Kk then
                        Wanted := True;
                     end if;
                  end loop;
                  if Wanted then
                     declare
                        --  抖一下手指重新认它:合到这只手量过的"合空"那头、再回到抖之前的读数(09-30:原来合的目标写死 0.0 = x5"0 = 合",
                        --  回的目标又是合完以后才读的读数 ⇒ 手指张不回去)。拍数上限 = 开机量到的合一次要几拍(Close_Steps)
                        Hk : constant Zone.Hand := Hand_Of (C, Arm, Kk);
                     begin
                        if Selfmap.Has_Jaw (F, Arm, Kk) and then Hk.Measured and then Hk.Close_Steps > 0 then
                           declare
                              R0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm, Kk);
                           begin
                              Jaw_Sweep (L, C, F, Arm, Kk, Hk.Empty_Close, Hk.Close_Steps, Integer (Cam), Sweep, Steps_J, Reading);
                              Jaw_Sweep (L, C, F, Arm, Kk, R0, Hk.Close_Steps, Integer (Cam), Sweep, Steps_J, Reading);
                           end;
                        else
                           Put_Line ("[身] ✋ 第" & Codec.Img (Arm + 1) & " 只手第 " & Codec.Img (Kk + 1) & " 个抓握通道"
                                     & (if not Selfmap.Has_Jaw (F, Arm, Kk) then "这一拍没有读数" else "没量过合一次要几拍") & " ⇒ 不抖它来重认");
                        end if;
                     end;
                  end if;
               end;
            end loop;
            Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
            Taken := Bool_Vectors.To_Vector (False, Regs.Length);
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : Point := Pts (I);
               begin
                  if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
                     --  认领半径:一个张幅,再小也有一个跟踪窗;读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                     --  没真看过的位置(只是按关节推的)可能差得远 ⇒ 认领半径放到整幅画面(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Z.Span, Track_Win) else 1.0), Long_Float'Max (0.005, Z.Span * 0.25), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                     Pts.Replace_Element (I, P);
                  end if;
               end;
            end loop;
         end;
      end if;
      --  零件:各自推一下自己的通道(开机看得见的那一档)再推回来
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            P : Point := Pts (I);
         begin
            if P.Kind = Piece_Pt and then P.Chan_K < Chan.Per_Arm then
               declare
                  K : constant Natural := P.Chan_K;
                  Chn : constant Natural := Arm * Chan.Per_Arm + K;
                  A : Table.Vec := Table.Zero_Vec;
                  Deliv : Table.Vec;
                  Ok : Boolean;
                  B0 : constant Buf := F.Cams (Cam).Gray;
                  Sweep : Bools;
                  Regs : Picture.Regions;
                  Taken : Bools;
                  P0 : constant Plug.Arm_Pose := F.EE (Arm);
                  Frames : Natural;
               begin
                  if Chn < Natural (C.Map.Amp.Length) and then C.Map.Amp (Chn) > 0.0 and then Cam < Natural (C.Map.Floors.Length) then
                     A (K) := C.Map.Amp (Chn);
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
                     Sweep := Picture.Moved (B0, F.Cams (Cam).Gray, C.Map.Floors (Cam));
                     declare
                        B1 : constant Buf := F.Cams (Cam).Gray;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Deliv, Frames, Ok);
                        Sweep := Picture.Either (Sweep, Picture.Moved (B1, F.Cams (Cam).Gray, C.Map.Floors (Cam)));
                     end;
                     Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
                     Taken := Bool_Vectors.To_Vector (False, Regs.Length);
                     --  认领半径:这块自己的框那么大,再小也有一个跟踪窗;读深窗口 = 框的四分之一(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win) else 1.0), Long_Float'Max (0.005, Long_Float'Max (P.Box_W, P.Box_H) * 0.25), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                  else
                     P.Lost := True;
                  end if;
                  Pts.Replace_Element (I, P);
               end;
            end if;
         end;
      end loop;
      --  认到的记进身体图:这个位姿下,这只手的这些零件(手指也是零件)在这台相机里就在这儿(下次到这附近不用看)
      declare
         X : Schema.Sample;
         Gp : Schema.Part_Pos;   --  手指那块:各团合成
         All_Fingers : Boolean := True;
         N, Nz : Natural := 0;
         Zmin : Long_Float := 1.0e30;   --  哨兵(无量纲)
         Any_Part : Boolean := False;
         Zh : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      begin
         X.Arm := Arm; X.Cam := Cam; X.Pose := F.EE (Arm);
         for P of Pts loop
            if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
               if P.Lost then
                  All_Fingers := False;
               end if;
               N := N + 1;
               Gp.Cu := Gp.Cu + P.Cu; Gp.Cv := Gp.Cv + P.Cv;
               if P.Blob = 1 then
                  Gp.B1u := P.Cu; Gp.B1v := P.Cv;
               else
                  Gp.B0u := P.Cu; Gp.B0v := P.Cv;
               end if;
               if P.Z > 0.0 then
                  Zmin := Long_Float'Min (Zmin, P.Z); Nz := Nz + 1;
               end if;
            elsif P.Kind = Piece_Pt and then not P.Lost and then P.Chan_K < Chan.Per_Arm then
               X.Parts (P.Chan_K) := (True, P.Cu, P.Cv, P.Z,
                                      Natural (Long_Float'Max (0.0, (P.Cu - P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Max (0.0, (P.Cv - P.Box_H / 2.0) * Long_Float (Ch))),
                                      Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + P.Box_H / 2.0) * Long_Float (Ch))),
                                      1, P.Cu, P.Cv, 0.0, 0.0);
               Any_Part := True;
            end if;
         end loop;
         if All_Fingers and then N > 0 then
            Gp.Valid := True;
            Gp.Cu := Gp.Cu / Long_Float (N); Gp.Cv := Gp.Cv / Long_Float (N);
            Gp.N_Blobs := N;
            Gp.Z := (if Nz > 0 then Zmin else 0.0);
            Gp.X0 := Zh.X0; Gp.Y0 := Zh.Y0; Gp.X1 := Zh.X1; Gp.Y1 := Zh.Y1;   --  框先沿用开机量的(Feel 会按形心平移)
            X.Parts (Chan.Per_Arm) := Gp;
         end if;
         if (All_Fingers and then N > 0) or else Any_Part then
            Schema.Add (C.Sch, X, C.Map.EE_Noise, C.Map.Rot_Noise);
         end if;
      end;
      --  🔴 认不出自己的时候,光说"按图猜"不够 —— 要说清【猜到哪儿了】。
      --  GP 实测:猜到 (1.000,0.475),那是画面最右边一列;从画面外的位置算出来的误差全是垃圾,
      --  于是 60 步一个像素没动,而日志一路"绿"。猜到画面边上 = 这台相机判不了这一段,
      --  必须让脑知道,好换一只眼睛(和"看不见目标的眼睛干不了这一段"是同一条规矩,只是换到我自己这一半)。
      declare
         U : constant Long_Float := Pts (0).Cu;
         V : constant Long_Float := Pts (0).Cv;
         --  边不是人拍的:一个跟踪窗那么宽 —— 眼睛跟得住的最小尺度,比它还靠边就没法量位移了
         Edge : constant Long_Float := Track_Win;
         Off : constant Boolean := U <= Edge or else U >= 1.0 - Edge or else V <= Edge or else V >= 1.0 - Edge;
      begin
         --  猜的 vs 看到的:差了多少。比不出来就不说(没认到时看到的那一份不存在)
         if not Pts (0).Lost then
            declare
               D : constant Long_Float := Sqrt ((U - Guess_U) ** 2 + (V - Guess_V) ** 2);
            begin
               if D > Track_Win then
                  Put_Line ("[身]     🔴 我猜我在 (" & Codec.Fmt (Guess_U, 3) & "," & Codec.Fmt (Guess_V, 3)
                            & ") 深 " & Codec.Fmt (Guess_Z, 3) & ",一看其实在 (" & Codec.Fmt (U, 3) & ","
                            & Codec.Fmt (V, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) & " —— 差 "
                            & Codec.Fmt (D, 3) & " 画幅(眼睛能跟住的一个窗口才 " & Codec.Fmt (Track_Win, 4)
                            & ")。这一段之前算的误差全是拿猜的位置算的");
                  C.Blind_Say := S ("careful: where my body map said my hand was and where I just saw it are "
                                    & Codec.Fmt (D, 3) & " of the picture apart - everything I worked out before "
                                    & "this look was measured from the wrong place");
               end if;
            end;
         end if;
         Put_Line ("[身]     生地/大步之后看一眼自己(手指抖一下 / 零件推一下):" &
                   (if Pts (0).Lost then "没认到,按图猜" else "认到了") &
                   " (" & Codec.Fmt (U, 3) & "," & Codec.Fmt (V, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) &
                   (if Off then " 🔴 这个位置贴在画面边上(边宽 " & Codec.Fmt (Edge, 3)
                      & " 画幅)—— 从画面外算出来的误差是垃圾,这台相机判不了这一段" else "") &
                   " · 这台相机里这只手的身体图 " & Codec.Img (Schema.Count (C.Sch, Arm, Cam)) & " 个样本");
         if Off then
            C.Blind_Say := S ("I could not find my own part in this eye and the place my body map guesses for it "
                              & "is right at the edge of the picture, so anything I measure from it is rubbish - "
                              & "name what you want in one of my other eyes and I will work there");
         end if;
      end;
   end Refind_Pieces;

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
                        Obj_Count : Natural; Held : out Boolean; Sure : out Boolean; Note : out Unbounded_String) is
      A : Table.Vec := Table.Zero_Vec;
      Deliv : Table.Vec;
      Ok : Boolean;
      Grip_Says_Held : Boolean := False;   --  手指停在空手值之上 ⇒ 中间有东西(和画面完全独立的一条证据)
      Grip_Note : Unbounded_String;
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      Jaw : Floats;
      Seen_In_Hand : Boolean := False;
      Home : Picture.Region := Origin;
      Gone_From_Table : Boolean := False;
      Could_Judge : Boolean := False;
      --  合完之后要量出"到底发生了什么",不是只答"拿住了没":旁边有没有东西被我碰动、那一块是不是断成了两块
      --  🔴 判"到底夹住没有"要的是【一台不跟着这只手动的相机】—— 以前只在【当前这只眼睛】里找,
      --  而当前这只正好长在手上 ⇒ 找不到 ⇒ "我不确定" ⇒ 把到手的东西又松开(GI 实测:
      --  笼住三项全过、合到底了,就因为没人核实而松手)。外面那台一直在那儿,去用它。
      World_Cam : constant Integer :=
        (if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 then Integer (Cam)
         else Still_Cam (C, F, Arm));
      Before_Regs : Picture.Regions;
      Moved_Others : Natural := 0;
      Pieces_Now : Natural := 0;
      --  抬之前我的手在【那台不跟着我动的相机】里的哪儿(拿住的唯一硬证据是"它跟着我的手走了同样一段")
      Hand_U0, Hand_V0 : Long_Float := 0.0;
      Have_Hand0 : Boolean := False;
      Follows : Boolean := False;
      Found_After : Boolean := False;
      Follow_Note : Unbounded_String;
   begin
      if World_Cam >= 0 and then Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
         declare
            Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
         begin
            Hand_U0 := Tr.Cu; Hand_V0 := Tr.Cv;
            Have_Hand0 := Tr.Valid and then not Tr.Blew_Up;
         end;
      end if;
      --  它原来在【那台相机】里的哪儿:用那台相机自己记着的影子,不能拿当前这只眼睛里的位置去比
      if World_Cam >= 0 and then Slot >= 0
        and then Natural (World_Cam) /= Cam
        and then Natural (Slot) < World.Count (C.Wld, Natural (World_Cam))
      then
         Home := World.Get (C.Wld, Natural (World_Cam), Natural (Slot)).Shadow;
      end if;
      if World_Cam >= 0 then
         Before_Regs := Cut_Things (C, F, Natural (World_Cam));
      end if;
      Jaw := Selfmap.Jaw_All (F, Arm);
      A (2) := C.Map.Amp (Arm * Chan.Per_Arm + 2) * 4.0;   --  抬起 = 看得见的探针幅度的几倍(倍数,无量纲),不假设哪根轴朝上:2 号轴是身体报的第三个平移通道
      Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
      if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (Hc));
            Regs : constant Picture.Regions := Cut_Things (C, F, Natural (Hc));
            Cw : constant Natural := F.Cams (Natural (Hc)).W;
            Ch : constant Natural := F.Cams (Natural (Hc)).H;
         begin
            if Z.Valid then
               Could_Judge := True;
               for R of Regs loop
                  if R.Count * 3 >= Obj_Count and then R.Cu * Long_Float (Cw) >= Long_Float (Z.X0) and then R.Cu * Long_Float (Cw) <= Long_Float (Z.X1)
                    and then R.Cv * Long_Float (Ch) >= Long_Float (Z.Y0) and then R.Cv * Long_Float (Ch) <= Long_Float (Z.Y1)
                  then
                     Seen_In_Hand := True;
                  end if;
               end loop;
            end if;
         end;
      end if;
      if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 and then Origin.Count > 0 then
         Could_Judge := True;
         Gone_From_Table := World.Vanished (Cut_Things (C, F, Cam), Origin, F.Cams (Cam).W, F.Cams (Cam).H);
      end if;
      --  旁边动了几件 · 原地现在剩几块(断成两块的话会多出一块)
      if World_Cam >= 0 then
         declare
            After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
            Tol : constant Long_Float := 1.0 / Long_Float (F.Cams (Natural (World_Cam)).W);   --  一个像素(画幅比例,无量纲)
         begin
            for Q of Before_Regs loop
               declare
                  Best : Long_Float := 0.0;
                  Found : Boolean := False;
               begin
                  for R of After loop
                     if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                        declare
                           D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                        begin
                           if not Found or else D < Best then
                              Best := D; Found := True;
                           end if;
                        end;
                     end if;
                  end loop;
                  --  不是我夹的那件,却挪过了噪声地板 ⇒ 我碰动了它
                  if Found and then Best > Tol * 4.0 and then Home.Count > 0
                    and then Sqrt ((Q.Cu - Home.Cu) ** 2 + (Q.Cv - Home.Cv) ** 2) > Long_Float'Max (Home.Sig_U, Home.Sig_V) * 2.0
                  then
                     Moved_Others := Moved_Others + 1;
                  end if;
               end;
            end loop;
            for R of After loop
               if Home.Count > 0 and then Sqrt ((R.Cu - Home.Cu) ** 2 + (R.Cv - Home.Cv) ** 2) <= Long_Float'Max (Home.Sig_U, Home.Sig_V) * 3.0 then
                  Pieces_Now := Pieces_Now + 1;
               end if;
            end loop;
         end;
      end if;
      --  抬完之后:我的手挪了多远、它挪了多远,两段差多少
      --  🔴 判不了也要说【缺哪一样】。HC 实测这一整段没进来,而给脑的话只有"我判不出来"五个字,
      --  我是靠 grep 括号才发现判据根本没跑 —— 不说缺什么 = 让脑以为判据跑过了。
      if World_Cam < 0 then
         Follow_Note := S (" (I have no camera that stays put while this arm moves, so nothing could watch it travel)");
      elsif not Have_Hand0 then
         Follow_Note := S (" (I could not pin down where my own hand was in that still camera before the lift"
                           & ", so I had nothing to compare the thing's travel against)");
      elsif Home.Count <= 0 then
         Follow_Note := S (" (I had no picture of where the thing was sitting before the lift)");
      end if;
      if World_Cam >= 0 and then Have_Hand0 and then Home.Count > 0 then
         declare
            After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
            Best : Integer := -1;
            Bd : Long_Float := 0.0;
            Hand_Du, Hand_Dv : Long_Float := 0.0;
         begin
            Feel (C, F);
            if Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
               declare
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
               begin
                  if Tr.Valid and then not Tr.Blew_Up then
                     Hand_Du := Tr.Cu - Hand_U0; Hand_Dv := Tr.Cv - Hand_V0;
                  else
                     Have_Hand0 := False;   --  这一抬我把自己的手跟丢了 ⇒ 判不了,老实说
                  end if;
               end;
            end if;
            --  抬完之后最像它的那一块:大小相近的里面离原处最近的
            for I in 0 .. Natural (After.Length) - 1 loop
               if After (I).Count * 3 >= Home.Count and then Home.Count * 3 >= After (I).Count then
                  declare
                     D : constant Long_Float := Sqrt ((After (I).Cu - Home.Cu) ** 2 + (After (I).Cv - Home.Cv) ** 2);
                  begin
                     if Best < 0 or else D < Bd then
                        Bd := D; Best := I;
                     end if;
                  end;
               end if;
            end loop;
            Found_After := Best >= 0;
            if Found_After and then Have_Hand0 then
               declare
                  Ou : constant Long_Float := After (Best).Cu - Home.Cu;
                  Ov : constant Long_Float := After (Best).Cv - Home.Cv;
               begin
                  Follows := Came_With_Me (Ou, Ov, Hand_Du, Hand_Dv);
                  Follow_Note := S (" (my hand moved " & Codec.Fmt (Sqrt (Hand_Du ** 2 + Hand_Dv ** 2), 3) &
                                    " of a frame, it moved " & Codec.Fmt (Sqrt (Ou ** 2 + Ov ** 2), 3) &
                                    ", the two differ by " & Codec.Fmt (Sqrt ((Ou - Hand_Du) ** 2 + (Ov - Hand_Dv) ** 2), 3) & ")");
               end;
            elsif not Found_After then
               Follow_Note := S (" (after the lift I could not find it anywhere in the camera that does not move with me)");
            else
               Follow_Note := S (" (I lost track of my own hand during the lift, so I cannot tell)");
            end if;
         end;
      end if;
      --  🔴 "拿住了"唯一分得开的硬证据:抬手时它【跟着我的手走了同样一段】。
      --  "它原来待的地方空了"分不开【撞跑】—— 球被撞到画面角落,原地照样空了,身体照样报"拿住"(FO 实测)。
      --  手上相机里"还在握区框里"更不算数 —— 那个框在手上相机里几乎是半个屏幕(FM 实测)。
      --  判不了就老实说"我说不准",不许自称拿住。
      --  🔴🔴 上面那三条全是【画面】信号。还有第四条,和画面完全独立:**手指停在哪儿**。
      --  爪子合在空气上会停在一个固定读数(开机量的空手值);中间夹着东西就停得更早。
      --  这一条撞跑伪造不了 —— 球被撞飞,手指照样合到空手值。
      --  ⇒ 两条正面证据【谁也不许否决谁】:手指卡住 = 拿住;跟着手走 = 拿住;两条都没有才叫没拿住。
      --  代价照记:反过来(拿画面一票否决)已经被实测判死 —— 纸杯蛋糕抬完 45 mm 读数远在空手值之上,
      --  只因画面里它变了样就被判滑掉、随即张手扔了。
      declare
         Jk : constant Natural := Natural (Integer'Max (0, C.Wld.Held_Jaw));
         --  空手值本来就量过、也存在身体文件里(Zone.Hand.Empty_Close,开机合空那一下的读数);
         --  这里只是【第一次把它拿来判拿住】,不新量一个。
         Hi : Integer := -1;
         Emp : Long_Float := -1.0;
         Have_R : constant Boolean := Selfmap.Has_Jaw (F, Arm, Jk);
         R_Now : constant Long_Float := (if Have_R then Selfmap.Jaw_Of (F, Arm, Jk) else 0.0);   --  没读数时不用它(下面都先问 Have_R)
      begin
         for I in 0 .. Natural (C.Hands.Length) - 1 loop
            if C.Hands (I).Arm = Arm and then C.Hands (I).K = Jk and then C.Hands (I).Measured then
               Hi := Integer (I);
            end if;
         end loop;
         if Hi >= 0 then
            Emp := C.Hands (Natural (Hi)).Empty_Close;
         end if;
         --  量得出空手值、这一拍也有读数,才谈得上问手指;门槛是读数自己的抖动(量出来的),不是我拍的容差。
         Grip_Says_Held := Hi >= 0 and then Have_R and then Past_Empty (C.Hands (Natural (Hi)), R_Now) > C.Map.Jaw_Noise;
         if not Have_R then
            Grip_Note := S (" (my fingers report no reading this beat, so I cannot ask them)");
         elsif Hi >= 0 then
            Grip_Note := S (" (my fingers stopped at " & Codec.Fmt (R_Now, 3)
                            & ", empty they stop at " & Codec.Fmt (Emp, 3)
                            & (if Grip_Says_Held then " - so something is wedged between them" else " - so there is nothing between them") & ")");
         else
            Grip_Note := S (" (I have never measured where my fingers stop on empty air, so I cannot ask them)");
         end if;
      end;
      Held := Grip_Says_Held
              or else (if World_Cam >= 0 and then Have_Hand0 and then Found_After then Follows else Seen_In_Hand);
      Sure := Grip_Says_Held or else (World_Cam >= 0 and then Have_Hand0 and then Found_After);
      if Grip_Says_Held and then not (World_Cam >= 0 and then Have_Hand0 and then Found_After and then Follows) then
         --  手指说有、画面说不出 ⇒ 以手指为准,并且把两边都说出来(不许只报结论)
         Note := S ("after a small lift my fingers are still held apart ⇒ held") & Grip_Note & Follow_Note;
      elsif Sure and then Follows then
         Note := S ("after a small lift it came with my hand ⇒ held") & Grip_Note & Follow_Note
                 & (if Seen_In_Hand then ", and my hand camera still shows it between my fingers" else "");
      elsif Sure then
         Note := S ("after a small lift it did NOT come with my hand ⇒ not held") & Grip_Note
                 & (if Gone_From_Table then S (" - and its old place is empty, so I knocked it away rather than picked it up") else S (""))
                 & Follow_Note
                 & (if Seen_In_Hand then " (my hand camera still shows something between my fingers, which proves nothing)" else "");
      elsif World_Cam >= 0 then
         Note := S ("after a small lift I could not judge whether it came with me") & Follow_Note;
      elsif Seen_In_Hand then
         Note := S ("after a small lift the thing is still inside my grip box in my hand camera; no still camera could check, so I am not sure");
      elsif Could_Judge then
         Note := S ("after a small lift the thing did not come with me ⇒ not held");
      else
         Note := S ("I could not judge whether it is held (no camera could see it)");
      end if;
      if World_Cam >= 0 then
         Append (Note, ". While I closed and lifted, " & Codec.Img (Moved_Others) & " other thing(s) I was not pushing moved");
         if Pieces_Now >= 2 then
            Append (Note, ", and where it stood there are now " & Codec.Img (Pieces_Now) & " separate pieces");
         end if;
      end if;
   end Held_Test;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

   --  编译器要知道的、关于每个名词的事实。编号和给脑看的清单一致(1 起),0 号空着。
   function Cw_Of (C : Context; F : Plug.Frame) return Natural is
     (if C.Cam < Natural (F.Cams.Length) then F.Cams (C.Cam).W else 1);
   function Ch_Of (C : Context; F : Plug.Frame) return Natural is
     (if C.Cam < Natural (F.Cams.Length) then F.Cams (C.Cam).H else 1);

   function Build_Facts (C : Context; F : Plug.Frame) return Plan.Facts_Vectors.Vector is
      Fs : Plan.Facts_Vectors.Vector;
      Zero : Plan.Item_Facts;
   begin
      Fs.Append (Zero);
      for I in 0 .. Natural (C.Items.Length) - 1 loop
         declare
            It : constant Item := C.Items (I);
            Ft : Plan.Item_Facts;
            Kk : constant Natural := (if It.Kind in Finger | Grip then Chan.Per_Arm + It.Jaw_K else It.Which);
         begin
            Ft.Exists := It.Located or else It.Kind in Finger | Grip | Piece;
            Ft.Mine := It.Kind in Finger | Grip | Piece;
            Ft.Grasp := It.Kind = Grip;
            Ft.Arm := It.Arm;
            Ft.Thing_Idx := -1;
            --  量得出它鼓出它站的那个面多少 ⇒ 才有"那个面"可言。面不是全局开关,是每个东西自己的事。
            Ft.Stands := It.Height > 0.0;
            Ft.Jaw_K := It.Jaw_K;
            --  张得开多少 / 这一块多宽:空转拿它判"合下去是不是必然空的"
            Ft.Span := (if It.Kind = Grip then Zone_Of (C, It.Arm, C.Cam, It.Jaw_K).Span else 0.0);
            Ft.Size := Long_Float'Max (Long_Float (It.X1 - It.X0) / Long_Float (Natural'Max (1, Cw_Of (C, F))),
                                       Long_Float (It.Y1 - It.Y0) / Long_Float (Natural'Max (1, Ch_Of (C, F))));
            Ft.Label := To_Unbounded_String
              ((case It.Kind is
                   when Grip => "grasper(第" & Codec.Img (It.Arm + 1) & " 只手第" & Codec.Img (It.Jaw_K) & " 组)",
                   when Finger => "grasper 的一瓣",
                   when Piece => "第" & Codec.Img (It.Arm + 1) & " 只手" & Codec.Img (It.Which) & " 轴带的那一块",
                   when others => ""));
            if Ft.Mine then
               for T in 0 .. Natural (C.Tables.Length) - 1 loop
                  if C.Tables (T).Arm = It.Arm and then C.Tables (T).Cam = C.Cam
                    and then C.Tables (T).Chan_K = Kk
                  then
                     Ft.Thing_Idx := Integer (T);
                     exit;
                  end if;
               end loop;
            end if;
            Fs.Append (Ft);
         end;
      end loop;
      return Fs;
   end Build_Facts;

   --  把 Sinew 的一段区间落成执行器内部那一小节。角色在这儿变成具体的那一块。
   --  离第 N 件东西(1 起)近的那条臂(0 起);说不出就 -1。它在哪只腕眼里被量到 ⇒ 那条臂;在不动的眼里 ⇒ 比它和各只手在那只眼里的画面距离
   --  (各只手在不动的眼里在哪,是开机合空时量出来的握区中心)。过渡规则:臂的活动范围量出来、候选按每条臂各排一遍之后,这一步整个删掉
   function Nearer_Arm (C : Context; N : Natural) return Integer is
   begin
      if N < 1 or else N > Natural (C.Items.Length) then
         return -1;
      end if;
      declare
         It : constant Item := C.Items (N - 1);
         A2 : constant Integer := Cam_Arm (C, It.Cam);
         Best : Integer := -1;
         Best_D : Long_Float := 0.0;
      begin
         if It.Kind not in Thing | Thing_Remembered then
            return -1;
         end if;
         --  在哪只腕眼里被点了名就是那条臂 —— 除非这一集里那条臂已经试过"它身上一段都在够不着那侧"(H60 2026-09-23:右臂虽然横着到头,
         --  剪刀手柄那头仍在够得着这侧,正是它拿起来的;所以只在一段都够不着时才换另一条)
         if A2 >= 0 then
            if C.No_Reach_Arm = A2 and then C.Map.Arms = 2 then
               return 1 - A2;
            end if;
            return A2;
         end if;
         for A in 0 .. C.Map.Arms - 1 loop
            declare
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, It.Cam, 0);
               D : constant Long_Float := Sqrt ((Z.Cu - It.Cu) ** 2 + (Z.Cv - It.Cv) ** 2);
            begin
               if Z.Valid and then C.No_Reach_Arm /= Integer (A) and then (Best < 0 or else D < Best_D) then
                  Best := Integer (A);
                  Best_D := D;
               end if;
            end;
         end loop;
         return Best;
      end;
   end Nearer_Arm;

   procedure Fill_Say (C : in out Context; I : Sinew.Instr; Answer : out Brain.Say) is
      use Sinew;
      function Old_Rel (R : Rel) return String is (Rel_Cmd (R));   --  唯一那张表,不许在这儿再抄一份
      function Old_Until (O : Outcome) return String is (Until_Word (O));   --  唯一那张表,不许在这儿再抄一份
      function Old_Step (Sp : Step) return String is
        (case Sp is when Sp_Small => "small", when Sp_Medium => "medium",
            when Sp_Large => "large", when Sp_None => "medium");
      function Item_Of (N : Noun) return Natural is
         A : constant Integer := Plan.Look_Up (C.Binds, N);
      begin
         return (if A > 0 then Natural (A) else 0);
      end Item_Of;
      function Place_Of (N : Noun; Pl : out Place) return Boolean is
      begin
         Pl := (others => <>);
         if N.K /= Nk_Thing then
            return False;
         end if;
         for K in 0 .. Natural (C.Places.Length) - 1 loop
            if C.Places (K).Name = N.Word then
               Pl := C.Places (K);
               return True;
            end if;
         end loop;
         return False;
      end Place_Of;
   begin
      Answer := (others => <>);
      Answer.See := To_Unbounded_String ("target");
      Answer.Fast := True;
      Answer.Until_Kind := To_Unbounded_String (Old_Until (I.Until_Oc));
      Answer.Steps := (if I.Max_Steps > 0 then I.Max_Steps else 0);
      for K in 0 .. Natural (I.Cons.Length) - 1 loop
         declare
            Cn : constant Constraint := I.Cons (K);
            Sub : constant Natural := Item_Of (Cn.Subj);
            Obj : constant Natural := Item_Of (Cn.Obj);
         begin
            case Cn.R is
               when Re_Qty =>
                  --  语言的根:某件东西的某个量往哪变。手由身体选(grasper 那一绑);没拿着它 ⇒ 先合在它上(合的定义含从上方进场),再抬
                  declare
                     Gn : constant Noun := (K => Nk_Role, R => Rl_Grasper, Word => Null_Unbounded_String);
                     Gi : constant Integer := Plan.Look_Up (C.Binds, Gn);
                     Bound_Arm : constant Natural := (if Gi >= 1 and then Gi <= Integer (C.Items.Length) then C.Items (Natural (Gi) - 1).Arm + 1 else 1);
                     --  哪只手去:离它近的那只(PLAN 第 2 步)。它在哪只腕眼里被点了名就是那条臂;在不动的眼里就比"它在画面里的位置"和"两只手在那只眼里各在哪"(握区量过的)
                     --  (H50/H56 2026-09-23 实测:右臂横跨整桌去够,关节到头,三把都合空)
                     --  手里已经拿着它 ⇒ 改它的量的就是拿着它的那只手,不再按远近选(H58 2026-09-23 实测:右手举着剪刀,头顶眼里它离左手近,左手去量眼、没动,还报"不在我手里")
                     Near_Arm : constant Integer := (if C.Wld.Holding and then C.Wld.Held_Arm >= 0 then C.Wld.Held_Arm else Nearer_Arm (C, Sub));
                     Arm1 : constant Natural := (if Near_Arm >= 0 then Natural (Near_Arm) + 1 else Bound_Arm);
                     Jk : Natural := 0;
                  begin
                     for It of C.Items loop
                        if It.Kind = Grip and then It.Arm + 1 = Arm1 then
                           Jk := It.Jaw_K;
                        end if;
                     end loop;
                     if Near_Arm >= 0 and then Arm1 /= Bound_Arm then
                        Put_Line ("[身] ✋ 离它近的是第" & Codec.Img (Arm1) & " 只手(不是绑到的第" & Codec.Img (Bound_Arm) & " 只)⇒ 用它");
                     end if;
                     Answer.Qty := Cn.Obj.Word; Answer.Qty_Dir := Cn.Dir; Answer.Qty_Of := Sub;
                     Answer.Grip_Arm := Arm1;
                     Answer.Grip_K := Jk;
                     if not (C.Wld.Holding and then C.Wld.Held_Arm = Integer (Arm1) - 1) then
                        Answer.Grip := To_Unbounded_String ("close");
                        Answer.Grip_On := Sub;
                     end if;
                  end;
               when Re_Close =>
                  Answer.Grip := To_Unbounded_String ("close");
                  Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                      then C.Items (Sub - 1).Arm + 1 else 1);
                  Answer.Grip_K := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                    then C.Items (Sub - 1).Jaw_K else 0);
                  Answer.Grip_On := Obj;
               when Re_Open =>
                  Answer.Grip := To_Unbounded_String ("open");
                  Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                      then C.Items (Sub - 1).Arm + 1 else 1);
                  Answer.Grip_K := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                    then C.Items (Sub - 1).Jaw_K else 0);
               when Re_Clear =>
                  Answer.Avoid.Append (Integer (Obj));
               when Re_Still =>
                  Answer.Moves.Append (Brain.Goal'(Item => Sub, Cell => 0, Rel => Null_Unbounded_String,
                                                   Of_Item => 0, Amount => Null_Unbounded_String,
                                                   Stay => True, Hard => True,
                                                   Has_Place => False, Pu => 0.0, Pv => 0.0, Pz => 0.0));
               when others =>
                  declare
                     Pl : Place;
                     Is_Place : constant Boolean := Place_Of (Cn.Obj, Pl);
                  begin
                  Answer.Moves.Append
                    (Brain.Goal'(Item => Sub, Cell => 0,
                                 Rel => To_Unbounded_String (Old_Rel (Cn.R)),
                                 Of_Item => Obj,
                                 Amount => To_Unbounded_String
                                   (if Cn.R = Re_Press
                                    then (case Cn.Ef is
                                             when Ef_Light => "small", when Ef_Firm => "medium",
                                             when Ef_Hard => "large", when Ef_None => "small")
                                    else Old_Step (Cn.Sp)),
                                 Stay => False, Hard => Cn.Rk = Rk_Must,
                                 Has_Place => Is_Place, Pu => Pl.Cu, Pv => Pl.Cv, Pz => Pl.Z));
                  end;
            end case;
         end;
      end loop;
      --  🔴 anyway:身体的一切认知性谨慎全部作废 —— 瞎着也走、离得远也合、顶着也推。
      C.Reckless := I.Anyway;
      C.Eye_Want := I.Eye;
   end Fill_Say;

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
                        Name : Unbounded_String := Null_Unbounded_String) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Bx : Integer := -1;
   begin
      Seen := False; U := 0.0; V := 0.0;
      if not (C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam)) then
         World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
      end if;
      --  带着名字来的:这只眼里叫这个名字的那一件,这一帧量到了就是它(槽是脑看着的那只眼的记账,走路的眼未必有槽)
      if Length (Name) > 0 then
         Bx := Boxed_By (C, Cam, Name);
         if Bx >= 0 and then C.Boxed (Natural (Bx)).Seen then
            U := C.Boxed (Natural (Bx)).Cu * Long_Float (Cw); V := C.Boxed (Natural (Bx)).Cv * Long_Float (Ch); Seen := True;
         end if;
         return;
      end if;
      --  🔴 跟的是【脑点过名、我在框里重量出来的那一块】,不是槽。槽是全图切块的记账,认槽靠"就近",
      --  H23 2026-09-22 实测:框里明明量到了(离线复算 3760 px、形心 (244,381)),槽却没对上 ⇒ 报"看丢了"。
      --  点过名的东西按名字找;只有没点过名的才退回槽。
      if Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam) then
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (Slot));
            Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
         begin
            Bx := Boxed_Index (C, Cam, Ref.Cu, Ref.Cv);
            if Bx < 0 then
               --  槽已经被挪到预测处、和框里的读数对不上号 ⇒ 按名字找这只眼里点过名的那一件
               for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                  if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Seen then
                     Bx := Integer (Bi);
                  end if;
               end loop;
            end if;
            if Bx >= 0 and then C.Boxed (Natural (Bx)).Seen then
               U := C.Boxed (Natural (Bx)).Cu * Long_Float (Cw); V := C.Boxed (Natural (Bx)).Cv * Long_Float (Ch); Seen := True;
               return;
            end if;
            if Sl.Present and then Sl.Seen then
               U := Sl.R.Cu * Long_Float (Cw); V := Sl.R.Cv * Long_Float (Ch); Seen := True;
            end if;
         end;
      end if;
   end Geo_Track;

   --  这只眼转过/挪过之后,点过名的那件东西在画面里【该】到哪:把它的世界位置(或它所在的方向上的一点)投进此刻的位姿,
   --  把重量的窗挪过去(大小不变)。不挪的话窗还留在转眼前的地方,里面是别的东西
   --  (H33 2026-09-22 实测:转 0.4 rad 把它整个看进来,旧窗里剩下的是黑手指 ⇒ "明暗反了,不是它" ⇒ 看丢 ⇒ 十轮一步不走)。
   procedure Retarget_Box (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P : Geom.V3) is
      Bx : constant Integer := Boxed_By (C, Cam, Name);
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Pu, Pv : Long_Float;
      Front : Boolean;
   begin
      if Bx < 0 or else Cam >= Natural (F.Cams.Length) or else Arm >= Natural (F.EE.Length) then
         return;
      end if;
      Geom.Project (G, F.EE (Arm), P, Pu, Pv, Front);
      declare
         Cw : constant Natural := F.Cams (Cam).W;
         Ch : constant Natural := F.Cams (Cam).H;
         B : Boxed_Thing := C.Boxed (Natural (Bx));
         Hw : constant Integer := (Integer (B.X1) - Integer (B.X0)) / 2;   --  半宽(纯数学的一半)
         Hh : constant Integer := (Integer (B.Y1) - Integer (B.Y0)) / 2;
         function Px (V2 : Long_Float; Span : Natural) return Natural is
           (Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Span - 1), V2))));
      begin
         if Front and then Pu >= 0.0 and then Pv >= 0.0 and then Pu < Long_Float (Cw) and then Pv < Long_Float (Ch)
           and then Hw > 0 and then Hh > 0
         then
            if B.Pu_On >= 0.0 then   --  它身上那一点跟着框平移(框心从旧的挪到预测处)
               B.Pu_On := Pu + (B.Pu_On - 0.5 * Long_Float (B.X0 + B.X1));
               B.Pv_On := Pv + (B.Pv_On - 0.5 * Long_Float (B.Y0 + B.Y1));
            end if;
            B.X0 := Px (Pu - Long_Float (Hw), Cw); B.X1 := Px (Pu + Long_Float (Hw), Cw);
            B.Y0 := Px (Pv - Long_Float (Hh), Ch); B.Y1 := Px (Pv + Long_Float (Hh), Ch);
            B.Blind := False;   --  这只眼看的地方变了,以前"这儿没有它"不再算数
            C.Boxed.Replace_Element (Natural (Bx), B);
            C.Cut_Cam := -1;    --  这一帧按挪过的窗重量
            Geo_Say ("它该出现在 (" & Codec.Fmt (Pu, 1) & "," & Codec.Fmt (Pv, 1) & ")(按转过/挪过的眼算)⇒ 窗挪过去再量");
         end if;
      end;
   end Retarget_Box;

   --  只平移(世界系),不转
   --  🔴 这里的量全是【米】。09-20 搬回来时为了不碰棘轮把"×1000"删了,标签却还写着 mm ⇒ 横挪 25.6 毫米显示成 "0.0 mm",
   --  "它在相机前 -0.8 mm"其实是负 0.8 米(算到相机背后去了)—— T10 2026-09-21 差点被这个标签骗过去。量的是米,就按米说,三位小数到毫米。

   procedure Geo_Move (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Dw : Geom.V3; Ok : out Boolean;
                       Watch : Selfmap.Watcher := null; Press : Boolean := False) is
      A : Table.Vec := Table.Zero_Vec;
      Jaw : Floats;
      Del : Table.Vec;
      Seq0 : constant Natural := F.Seq;
   begin
      A (0) := Dw (0); A (1) := Dw (1); A (2) := Dw (2);
      Step_Arm (L, C, F, Arm, A, Jaw, Del, Ok, Press => Press, Watch => Watch, Geo_Settle => Selfmap."=" (Watch, null));
      --  命令了多少、实到多少,每一步都说(GB5 那一版有这一行,搬回 main 时丢了;H6 2026-09-22 实测每步要 14 cm 而差距只缩 0–2 cm,
      --  没有这一行就分不清是身体没走成、还是我算错了)。拍号 = 这一下起止那两帧的帧号(同 poses.txt / fk_poses.txt / joints.txt 的第一列,
      --  离线按仿真真值给每一下压标"碰没碰到"用;09-29 V1B65 按位移反推拍号一半对不上)
      Geo_Say ("挪 (" & Mm (Dw (0)) & "," & Mm (Dw (1)) & "," & Mm (Dw (2)) & ") ⇒ 实到 (" & Mm (Del (0)) & "," & Mm (Del (1)) & "," & Mm (Del (2)) &
               "),差 " & Mm (Geom.Norm ([Dw (0) - Del (0), Dw (1) - Del (1), Dw (2) - Del (2)])) & (if Ok then "" else " · 身体说没走成")
               & " · 拍 " & Codec.Img (Seq0) & "→" & Codec.Img (F.Seq));
   end Geo_Move;

   --  "上"只写在这一处:位姿系的 +z 是协议约定的重力反方向(观测里没有重力读数的身体只能这么约;有加速度计的身体应把它换成量出来的)。
   --  碰过面之后"上"= 那张面的法向(量出来的)
   Protocol_Up : constant Geom.V3 := [0.0, 0.0, 1.0];
   function Up_Dir (C : Context) return Geom.V3 is (if C.Touch_Valid then C.Touch_N else Protocol_Up);

   --  ── 东西的量(登记表)──:脑的句子只有一种:do <东西> <量> up|down until <结局>。量的名字由身体列(键盘上"量 [...]"那一栏),
   --  每个量有一个"让它变的方向"(世界系单位向量,从量出来的东西算);量变了 = 手里的接触点沿那个方向的旋量 ⇒ 同一条接触集 + 执行层。
   --  加一个量 = 这两处各加一行;句子、接触集、执行层都不动,不按任务分。现在身体量得出的只有一个:height = 离它躺的面多高,方向 = 那张面的法向
   function Qty_Words (C : Context; Roles : String) return String is
     (if Ada.Strings.Fixed.Index (Roles, "grasper") > 0 then "height" else "");
   function Qty_Axis (C : Context; Qty : String) return Geom.V3 is
     (if Qty = "height" then Up_Dir (C) else [0.0, 0.0, 0.0]);

   --  这一段用了几拍:对方在段中间复位(新的一集,步数从零起)时不许算成负数(S1 2026-09-23 实测:第二集开始时正在进场,减出负数把驱动崩了)
   function Beats_Since (L : Plug.Link; B0 : Natural) return Natural is
     (if Plug.Steps (L) >= B0 then Plug.Steps (L) - B0 else Plug.Steps (L));
   --  段中间对方复位了 ⇒ 这一段作废的那句话(所有走路的段共用)
   Reset_Event : constant String := "reset: the world was reset under me (a new episode began) - this segment is void";

   --  朝下被顶住的一点进地图。它躺的面 = 这一集里【最低】的那张(东西靠面抵住重力;比已知的面高的顶住是躺在面上的别的东西或它自己,
   --  H61 2026-09-23 实测:压到牛仔裤上把它当成剪刀躺的面,轮廓按高 4 cm 的面重投、落点挪了 4 cm、合空)。
   --  上一集留下的面第一次被顶住时直接换新(桌子可能换了);之后只让更低的换。"高出多少"按已知面的法向量,门槛 = 本体位置读数的抖动(量过的)
   procedure Note_Support (C : in out Context; P, N : Geom.V3; How : String) is
   begin
      --  标定板的点拟合过一张面(腕眼三角,1 mm 级,Geo_Board)⇒ 东西躺的面就是它;朝下顶住的点只和它对账、不换它:
      --  顶住的点 = 位姿读数 + 量过的指尖偏移,指尖错了它就错(X5B 2026-09-25:指尖错了的那只手顶住的点比板的面低 20.8 cm,"最低的赢"把它当成了桌面,板的面被顶掉)。
      --  门 = 3 倍(倍数无量纲,同踢离群)"板的面内离散 ⊕ 位姿读数的抖动",两样都是量的。高出门 ⇒ 躺在面上的东西;低过门 ⇒ 桌面压不下去,错的是我算的那一点
      if C.Board_Plane then
         declare
            Hb : constant Long_Float := (P (0) - C.Board_Pt (0)) * C.Board_N (0) + (P (1) - C.Board_Pt (1)) * C.Board_N (1) + (P (2) - C.Board_Pt (2)) * C.Board_N (2);
            Tol : constant Long_Float := 3.0 * Sqrt (C.Board_Rms ** 2 + C.Map.EE_Noise ** 2);
         begin
            C.Touch_Pt := C.Board_Pt; C.Touch_N := C.Board_N; C.Touch_Valid := True; C.Touch_Fresh := True;
            if Geom."=" (P, C.Board_Pt) then
               Geo_Say ("东西躺的面 = " & How);
            elsif Hb > Tol then
               C.Bumps.Append (P);
               Geo_Say ("有个东西顶着我(" & How & "),比标定板的面高 " & Mm (Hb) & " ⇒ 是躺在面上的东西;记成「这儿有东西」,面还是板的那张;沿着它接着走");
            elsif Hb < -Tol then
               Geo_Say ("对不上:顶住我的这一点(" & How & ")比标定板的面低 " & Mm (-Hb) & "(门 " & Mm (Tol) & "),桌面压不下去 ⇒ 错的是我算的这一点"
                        & "(指尖偏移或手上那只眼的几何);面还是板的那张");
            else
               Geo_Say ("对账:顶住我的这一点(" & How & ")就在标定板的面上(差 " & Mm (Hb) & ",门 " & Mm (Tol) & ")⇒ 对得上");
            end if;
         end;
         return;
      end if;
      if C.Touch_Valid and then C.Touch_Fresh then
         declare
            H : constant Long_Float := (P (0) - C.Touch_Pt (0)) * C.Touch_N (0) + (P (1) - C.Touch_Pt (1)) * C.Touch_N (1) + (P (2) - C.Touch_Pt (2)) * C.Touch_N (2);
         begin
            if H > C.Map.EE_Noise then
               C.Bumps.Append (P);
               Geo_Say ("有个东西顶着我(" & How & "),比它躺的面高 " & Mm (H) & " ⇒ 不是面,是躺在面上的东西;记成「这儿有东西」,面还是原来那张;沿着它接着走");
               return;
            end if;
         end;
      end if;
      C.Touch_Pt := P; C.Touch_N := N; C.Touch_Valid := True; C.Touch_Fresh := True;
      Geo_Say ("有个面顶着我(" & How & ")⇒ 是它躺的面,记进地图;沿着它接着走");
   end Note_Support;

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
                          Plane_Pt, Plane_N : Geom.V3; Plane_Rms : Long_Float; Ref : Plug.Cam; Keep_Tips : Boolean := False) is
      pragma Unreferenced (F);
   begin
      C.Geo_Path := S (Body_Path & ".geo.json");
      C.Geo := Geo;
      if Keep_Tips and then Body_Path /= "" then
         --  前半段装回的:存的指尖、张口(碰桌面量的)、步幅(一条命令走多远 / 转多远,开机按阶梯量的)并进来 —— 同一次从零量的结果、同一个世界单位。
         --  09-28 S1A1:原来只并指尖和张口,步幅在记分那一集里整套重量,吃掉 122 拍(一集 200 拍)
         declare
            Old : Geom.Geo_Vectors.Vector;
            Note : String (1 .. 160);
         begin
            Geom.Load (To_String (C.Geo_Path), Old, Natural (C.Geo.Length), Note);
            for Cm in 0 .. Natural'Min (Natural (Old.Length), Natural (C.Geo.Length)) - 1 loop
               if not C.Geo (Cm).Fixed then
                  declare
                     G : Geom.Cam_Geo := C.Geo (Cm);
                  begin
                     if Old (Cm).Tip_Valid and then Old (Cm).Tip_Touch then
                        G.Tip := Old (Cm).Tip; G.Gap := Old (Cm).Gap; G.Tip_Valid := True; G.Tip_Touch := True;
                        G.Lobes := Old (Cm).Lobes; G.Tip_Sd := Old (Cm).Tip_Sd;   --  每一瓣的尖和截面(接触集的手)跟着指尖一起并回来
                     end if;
                     if Old (Cm).Stride > 0.0 then
                        G.Stride := Old (Cm).Stride;
                     end if;
                     if Old (Cm).Stride_Rot > 0.0 then
                        G.Stride_Rot := Old (Cm).Stride_Rot;
                     end if;
                     C.Geo.Replace_Element (Cm, G);
                  end;
               end if;
            end loop;
         end;
      end if;
      C.Board := Board; C.Board_Seen.Clear; C.Seen_Above.Clear;   --  换了板:以前压之前看见的点按旧的面判的高低,作废
      C.Board_Pt := Plane_Pt; C.Board_N := Plane_N; C.Board_Rms := Plane_Rms;
      C.Board_Plane := not Board.Is_Empty;
      --  有板的面 ⇒ 东西躺的面就是它(同 Note_Support:有板时朝下顶住的点只和它对账、不换它),装上就登记;不等第一次朝下被顶住。
      --  09-28 S1A1:装回开机不碰桌面、碰指尖那几下又不记接触 ⇒ "碰过的面"一直空着,不动的眼看见剪刀时按指尖此刻的高度当面,
      --  剪刀被放到桌面上方 5 个单位,腕眼转了 1.7 弧度还没转到、顶到关节尽头。Touch_Fresh 不设:这一集里还没真碰过
      if C.Board_Plane then
         C.Touch_Pt := Plane_Pt; C.Touch_N := Plane_N; C.Touch_Valid := True;
      end if;
      C.Fixed_Ref := Ref.RGB; C.Fixed_Ref_W := Ref.W; C.Fixed_Ref_H := Ref.H;
      C.Fixed_Best := (others => <>);
      --  不动的眼核对用的细门按板定,和重标那一份同一个算法(Geom.Board_Rms):板上每个点(参考图里的像素)按标定的位姿投回去,门以内误差的中位 × 1.2
      for Cam in 0 .. Natural (C.Geo.Length) - 1 loop
         declare
            G : Geom.Cam_Geo := C.Geo (Cam);
         begin
            if G.Fixed and then G.Valid and then not C.Board.Is_Empty then
               declare
                  Br : constant Long_Float := Geom.Board_Rms (G, C.Board, 3.0 * Long_Float'Max (1.0e-9, G.Rms));   --  解的时候的门(3 倍,协议;同核对)
               begin
                  if Br > 0.0 then
                     Geo_Say ("不动的眼按板配得多细:板上 " & Codec.Img (Natural (C.Board.Length)) & " 个点投回去,误差中位 × 1.2 = " & Codec.Fmt (Br, 2)
                              & " px(解的时候的均方根 " & Codec.Fmt (G.Rms, 2) & " px)⇒ 核对的细门按它定");
                     G.Rms := Br;
                     C.Geo.Replace_Element (Cam, G);
                  end if;
               end;
            end if;
         end;
      end loop;
      for Cam in 0 .. Natural (C.Geo.Length) - 1 loop
         declare
            G : constant Geom.Cam_Geo := C.Geo (Cam);
            A : constant Integer := Cam_Arm (C, Cam);
         begin
            if G.Fixed then
               Geo_Say ("第" & Codec.Img (Cam) & " 台相机(不长在手上):焦距 " & Codec.Fmt (G.F, 1) & " px、在世界 (" & Codec.Fmt (G.Pos (0), 3) & ", " & Codec.Fmt (G.Pos (1), 3) & ", "
                        & Codec.Fmt (G.Pos (2), 3) & ") 单位(开机前半段对齐量的)");
            elsif A >= 0 and then G.Valid then
               Geo_Say ("第" & Codec.Img (Cam) & " 台相机(长在第" & Codec.Img (Natural (A) + 1) & " 只手上):焦距 " & Codec.Fmt (G.F, 1) & " px(运动学量的)· 手的位姿就是它的位姿 · 指尖 "
                        & (if G.Tip_Valid and then G.Tip_Touch then "存的(碰桌面量过,离眼 " & Mm (Geom.Norm (G.Tip)) & ")" else "待碰桌面量"));
            end if;
         end;
      end loop;
      Geo_Say ("标定板 " & Codec.Img (Natural (C.Board.Length)) & " 个点(开机前半段三角出、配进不动的眼的)· 桌面 = 世界 z = 0、离散 " & Codec.Fmt (C.Board_Rms, 4)
               & " 单位(长度单位 = 第一只手运动学的单位)");
      if not C.Geo.Is_Empty then
         Geom.Save (To_String (C.Geo_Path), C.Geo);
      end if;
   end Geo_Install;

   --  几何逼近:让"指尖该到的那一点"(指尖中点再往手心里一点)和点名那块重合。每段走一截、停稳、再看一眼、再算。
   --  这一槽里的东西此刻看得【全不全】,以及它叫什么(点过名的才有名字)。看不全(顶到窗边/被画面切掉)的那一眼,形心不是同一个物理点。
   --  Whole = 这一眼量到的是一整块(没被【画面边】切掉;挨着邻居的已在框里量时裁掉,形心照用)。
   --  Edge = 它被画面边切掉了 ⇒ 转一下眼把它整个看进来,这一眼才算数。
   procedure Slot_Whole (C : Context; F : Plug.Frame; Cam : Natural; Slot : Integer; Whole, Edge : out Boolean; Name : out Unbounded_String;
                         Named : Unbounded_String := Null_Unbounded_String) is
   begin
      Whole := True; Edge := False; Name := Null_Unbounded_String;
      if Length (Named) > 0 then
         Name := Named;
         declare
            Bx : constant Integer := Boxed_By (C, Cam, Named);
         begin
            if Bx >= 0 then
               declare
                  B : constant Boxed_Thing := C.Boxed (Natural (Bx));
               begin
                  Edge := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cam).W or else B.Y1 + 1 >= F.Cams (Cam).H;
                  Whole := not Edge;
               end;
            end if;
         end;
         return;
      end if;
      if Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam) then
         declare
            R : constant Picture.Region := World.Get (C.Wld, Cam, Natural (Slot)).R;
            Bx : constant Integer := Boxed_Index (C, Cam, R.Cu, R.Cv);
         begin
            if Bx >= 0 then
               declare
                  B : constant Boxed_Thing := C.Boxed (Natural (Bx));
               begin
                  Edge := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cam).W or else B.Y1 + 1 >= F.Cams (Cam).H;
                  Whole := not Edge;
                  Name := B.Name;
               end;
            end if;
         end;
      end if;
   end Slot_Whole;

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

   procedure Check_Fixed_Eye (F : Plug.Frame; C : in out Context) is
      Wc : constant Natural := C.Map.World_Cam;
      Deg_Say : constant String := "°";
   begin
      if Length (C.Inst_Host) = 0 then
         Say_No_Check;
         return;
      end if;
      if C.Board.Is_Empty or else C.Fixed_Ref.Is_Empty or else Wc >= Natural (C.Geo.Length) or else Wc >= Natural (F.Cams.Length)
        or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else F.Cams (Wc).W = 0
      then
         return;
      end if;
      declare
         Q : Instrument.Match_Vectors.Vector;
         Err : Unbounded_String;
         Now : Geom.Scene_Pt_Vectors.Vector;
         G : Geom.Cam_Geo := C.Geo (Wc);
         R : Geom.Fixed_Check;
         Ok : Boolean;
         Turned : Natural := 0;   --  此刻的图转回几个 90° 才配上的
         --  此刻的图先顺时针转 Turns 个 90°(Turn_90)再配;配到的像素一步步换算回没转的画面:转一步前高 h 的图里 (u, v) ← 转后的 (u', v') = (v', h − u')(连续坐标,见 Unturn)
         function Matched (Turns : Natural; Got : out Boolean) return Geom.Scene_Pt_Vectors.Vector is
            Img : Buf := F.Cams (Wc).RGB;
            W : Natural := F.Cams (Wc).W;
            H : Natural := F.Cams (Wc).H;
            M : Instrument.Match_Vectors.Vector;
            Res : Geom.Scene_Pt_Vectors.Vector;
         begin
            for T in 1 .. Turns loop
               Img := Turn_90 (Img, W, H);
               declare
                  W0 : constant Natural := W;
               begin
                  W := H; H := W0;
               end;
            end loop;
            --  往返配(同扫描、对齐):配过去再配回来 1 px 以内才算看见,挡住的那一块仪器编出来的点配不回来(见 Geom.Trip_Px)
            M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, C.Fixed_Ref, C.Fixed_Ref_W, C.Fixed_Ref_H, Img, W, H, Q, Err, Back => True);
            Got := Natural (M.Length) = Natural (Q.Length);
            if not Got then
               return Res;
            end if;
            for I in 0 .. Natural (M.Length) - 1 loop
               declare
                  P : Geom.Scene_Pt := C.Board (I);
                  In_Pic : constant Boolean := M (I).U >= 0.0 and then M (I).V >= 0.0 and then M (I).U < Long_Float (W) and then M (I).V < Long_Float (H)
                    and then Geom.Round_Trip_Ok (Q (I).U, Q (I).V, M (I).Bu, M (I).Bv);
                  U, V : Long_Float;
               begin
                  Unturn (M (I).U, M (I).V, Turns, F.Cams (Wc).W, F.Cams (Wc).H, U, V);
                  P.U := (if In_Pic then U else -1.0); P.V := (if In_Pic then V else -1.0);
                  Res.Append (P);
               end;
            end loop;
            return Res;
         end Matched;
      begin
         for S of C.Board loop
            Q.Append (Instrument.Match_Pt'(U => S.U, V => S.V, Cert => 0.0, others => <>));
         end loop;
         declare
            B0 : constant Geom.Fixed_Best := C.Fixed_Best;   --  这一轮之前的"放好以来最多"(别的转法各自从它起算)
         begin
            Now := Matched (C.Fixed_Turn, Ok);
            if not Ok then
               Geo_Say ("核对不动的眼:仪器没配成(" & To_String (Err) & ")⇒ 这一轮不核");
               return;
            end if;
            Geom.Check_Fixed (G, C.Board, Now, C.Fixed_Best, R, Turn_Sd => C.Fixed_Turn_Sd);
            Turned := C.Fixed_Turn;
            --  挪过,或看不全(看不全只在刚变的那一轮、之后隔 1、2、4、8……轮:次数翻倍,挡着的时候也可能被转,代价随挡的时长按对数涨):
            --  此刻的图按四个转法(转 0/90/180/270°)各配一次,挑新门里解释点最多的那个 —— 转正的那个配得最细、点最多
            --  (RoMa 转 90° 配上六成、配得糙,转 180° 一个都配不上;X5E2 2026-09-26 第一个过关的是转 180° 那个,相对还差 90°,采纳了一份差 2.9 cm、4.75 px 的位姿)。
            --  采纳之后每轮就按这个转法配(C.Fixed_Turn),配点一直是转正的精度,细门不被糙解抬高
            if R.Moved or else (R.Covered and then (not C.Fixed_Covered or else C.Round_N >= C.Fixed_Turn_Next)) then
               if R.Covered then
                  if not C.Fixed_Covered then
                     C.Fixed_Turn_Gap := 1;
                  else
                     C.Fixed_Turn_Gap := 2 * C.Fixed_Turn_Gap;
                  end if;
                  C.Fixed_Turn_Next := C.Round_N + C.Fixed_Turn_Gap;
               end if;
               declare
                  Base : constant Natural := R.Consistent_Now;
                  Have : Boolean := R.Moved;
                  Best_G : Geom.Cam_Geo := G;
                  Best_R : Geom.Fixed_Check := R;
                  Best_B : Geom.Fixed_Best := C.Fixed_Best;
                  Best_T : Natural := C.Fixed_Turn;
               begin
                  for T in 0 .. 3 loop
                     if T /= C.Fixed_Turn then
                        declare
                           Gt : Geom.Cam_Geo := C.Geo (Wc);
                           Bt : Geom.Fixed_Best := B0;
                           Rt : Geom.Fixed_Check;
                           Okt : Boolean;
                           Nt : constant Geom.Scene_Pt_Vectors.Vector := Matched (T, Okt);
                        begin
                           if Okt then
                              Geom.Check_Fixed (Gt, C.Board, Nt, Bt, Rt, Turn_Sd => C.Fixed_Turn_Sd, Base_Now => Base);
                              if Rt.Moved and then (not Have or else Rt.Consistent > Best_R.Consistent) then
                                 Best_G := Gt; Best_R := Rt; Best_B := Bt; Best_T := T; Have := True;
                              end if;
                           end if;
                        end;
                     end if;
                  end loop;
                  if Have then
                     G := Best_G; R := Best_R; C.Fixed_Best := Best_B; Turned := Best_T; C.Fixed_Turn := Best_T;
                  end if;
               end;
            end if;
         end;
         if R.Moved then
            C.Geo.Replace_Element (Wc, G);
            --  参考图和板上的点在参考图里的像素都不换(一直是标好那一刻的):换成此刻的,一挡住参考图就跟着坏,错一轮接一轮地叠(X5B 2026-09-25)。
            --  它按旧位姿做的轮廓作废
            if C.Sil_Valid and then C.Sil_Cam = Integer (Wc) then
               C.Sil_Valid := False;
            end if;
            Geom.Save (To_String (C.Geo_Path), C.Geo);
            Board_Save (C);
            Geo_Say ("核对不动的眼:它被挪过 —— 转了 " & Codec.Fmt (R.Turn_Deg, 1) & Deg_Say & "、挪了 " & Mm (R.Move_M) & ",板上的点在画面里挪了 " & Codec.Fmt (R.Shift_Px, 1)
                     & " px(" & Codec.Fmt (R.Shift_Sd, 1) & " 个配点噪声)"
                     & (if Turned > 0 then ",此刻的图顺时针转 " & Codec.Img (90 * Turned) & Deg_Say & " 配得最好(以后每轮都这么转了再配)" else "")
                     & " ⇒ 按板重新标好(" & Codec.Img (R.Consistent) & "/" & Codec.Img (R.Asked) & " 个点对得上,残差 " & Codec.Fmt (R.Rms, 2) & " px),接着干");
         elsif R.Covered then
            if not C.Fixed_Covered then
               Geo_Say ("核对不动的眼:板上 " & Codec.Img (R.Asked) & " 个点这会儿只有 " & Codec.Img (R.Consistent_Now) & " 个还对得上(放好以来最多 " & Codec.Img (C.Fixed_Best.All_N)
                        & (if R.Dark >= 0 then ";画面" & Geom.Region_Name (Natural (R.Dark)) & "放好以来看见过 " & Codec.Img (R.Dark_Best) & " 个,这会儿只剩 "
                           & Codec.Img (R.Dark_Now) & " 个" else "")
                        & " 个)⇒ 它被挡住了一大块(或看不见了);位姿照旧,它这会儿看见的东西先别全信");
            end if;
         elsif C.Fixed_Covered or else not C.Fixed_Said then
            Geo_Say ("核对不动的眼" & (if C.Fixed_Said then "" else "(这次开机第一次)") & ":板上 " & Codec.Img (R.Asked) & " 个点此刻 " & Codec.Img (R.Consistent_Now)
                     & " 个对得上(放好以来最多 " & Codec.Img (C.Fixed_Best.All_N) & " 个)⇒ " & (if C.Fixed_Covered then "又看全了" else "没挪、没挡"));
         end if;
         --  挡没挡只在变的那一轮说(X5C 每轮报一遍"挡住了")
         C.Fixed_Covered := R.Covered and then not R.Moved;
         C.Fixed_Said := True;
         --  每一轮核对的数落盘(BL_DUMP/check.txt):轮、配到、原位姿对得上、新解对得上、放好以来最多、挪没挪、挡没挡、转回几个 90°、细门(像素)
         if Length (C.Dump_Dir) > 0 then
            declare
               Fo : Ada.Text_IO.File_Type;
               Path : constant String := To_String (C.Dump_Dir) & "/check.txt";
            begin
               begin
                  Ada.Text_IO.Open (Fo, Ada.Text_IO.Append_File, Path);
               exception
                  when others => Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Path);
               end;
               Ada.Text_IO.Put_Line (Fo, Codec.Img (C.Round_N) & " " & Codec.Img (R.Matched) & " " & Codec.Img (R.Consistent_Now) & " " & Codec.Img (R.Consistent) & " "
                                     & Codec.Img (C.Fixed_Best.All_N) & " " & (if R.Moved then "1" else "0") & " " & (if R.Covered then "1" else "0") & " " & Codec.Img (Turned)
                                     & " " & Codec.Fmt (R.Gate, 3) & " " & Integer'Image (R.Dark) & " " & Codec.Img (R.Dark_Now) & " " & Codec.Img (R.Dark_Best));
               --  列:轮、配到、原位姿对得上、新解对得上、放好以来最多、挪没挪、挡没挡、转回几个 90°、细门(像素)、看不见的那一块(-1 = 没有)、它此刻 / 放好以来对得上几个
               Ada.Text_IO.Close (Fo);
            exception
               when others => null;
            end;
         end if;
      end;
   end Check_Fixed_Eye;

   --  板上的点在不动的眼此刻的画面里重找一遍(2026-09-28 V1B47):参考图往返配(同核对不动的眼,配过去再配回来 1 px 以内算找到),
   --  找不到的 = 那儿被挪来的东西盖住了、或者此刻被手挡着 ⇒ 挑空地时不算量过的桌面(Board_Free_Spots)。
   --  画面按核对定下的转法先转正再配(同核对)。只管"还找不找得到",不按位姿判:相机挪过时点照样找得到,板的世界位置不跟着变
   procedure Board_Recheck (F : Plug.Frame; C : in out Context; Found : out Natural; Said : out Unbounded_String) is
      Wc : constant Natural := C.Map.World_Cam;
   begin
      Found := 0; Said := Null_Unbounded_String;
      if C.Board.Is_Empty then
         Said := To_Unbounded_String ("板上没有点");
         return;
      end if;
      if Length (C.Inst_Host) = 0 then
         Said := To_Unbounded_String ("没配配点仪器");
         return;
      end if;
      if C.Fixed_Ref.Is_Empty or else Wc >= Natural (C.Geo.Length) or else Wc >= Natural (F.Cams.Length)
        or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else F.Cams (Wc).W = 0
      then
         Said := To_Unbounded_String ("没有不动的眼(或者它这会儿没有画面)");
         return;
      end if;
      declare
         Q : Instrument.Match_Vectors.Vector;
         Err : Unbounded_String;
         Img : Buf := F.Cams (Wc).RGB;
         W : Natural := F.Cams (Wc).W;
         H : Natural := F.Cams (Wc).H;
         M : Instrument.Match_Vectors.Vector;
         Seen : Bools;
      begin
         for T in 1 .. C.Fixed_Turn loop
            Img := Turn_90 (Img, W, H);
            declare
               W0 : constant Natural := W;
            begin
               W := H; H := W0;
            end;
         end loop;
         for S of C.Board loop
            Q.Append (Instrument.Match_Pt'(U => S.U, V => S.V, Cert => 0.0, others => <>));
         end loop;
         M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, C.Fixed_Ref, C.Fixed_Ref_W, C.Fixed_Ref_H, Img, W, H, Q, Err, Back => True);
         if Natural (M.Length) /= Natural (Q.Length) then
            Said := To_Unbounded_String ("仪器没配成(" & To_String (Err) & ")");
            return;
         end if;
         for I in 0 .. Natural (M.Length) - 1 loop
            declare
               Ok : constant Boolean := M (I).U >= 0.0 and then M (I).V >= 0.0 and then M (I).U < Long_Float (W) and then M (I).V < Long_Float (H)
                 and then Geom.Round_Trip_Ok (Q (I).U, Q (I).V, M (I).Bu, M (I).Bv);
            begin
               Seen.Append (Ok);
               if Ok then
                  Found := Found + 1;
               end if;
            end;
         end loop;
         C.Board_Seen := Seen;
      end;
   end Board_Recheck;

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
   procedure Take_Silhouette (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P0 : Geom.V3; P0_Up_Sd : Long_Float) is
      Bx : constant Integer := Boxed_By (C, Cam, Name);
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Keep : constant := 2000;   --  最多留这么多个点(点数)
      A2 : constant Integer := Cam_Arm (C, Cam);
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam, 0);
      N : constant Geom.V3 := Up_Dir (C);
      Rays : Geom.Sight_Vectors.Vector;
      Pts : Contact.V3_Vectors.Vector;
      Dropped : Natural;
      K : Natural := 0;
      Stride : Positive := 1;
   begin
      if Bx < 0 or else (A2 < 0 and then not G.Fixed) then
         return;
      end if;
      declare
         B : constant Boxed_Thing := C.Boxed (Natural (Bx));
         --  我自己的手指像素只在"这只眼长在正走路的这条胳膊上"时才剔:别的眼里我的手随位姿到处走,握区那张手指图不是此刻的
         Self_Px : constant Boolean := A2 = Integer (Arm) and then Z.Valid and then Natural (Z.Fingers.Length) = Cw * Ch;
      begin
         if not B.Seen or else Natural (B.Mask.Length) /= Cw * Ch then
            return;
         end if;
         for I in B.Y0 .. B.Y1 loop
            for J in B.X0 .. B.X1 loop
               if B.Mask (I * Cw + J) then
                  K := K + 1;
               end if;
            end loop;
         end loop;
         if K = 0 then
            return;
         end if;
         Stride := Positive'Max (1, Positive (Long_Float'Ceiling (Sqrt (Long_Float (K) / Long_Float (Keep)))));
         for I in B.Y0 .. B.Y1 loop
            if I mod Stride = 0 then
               for J in B.X0 .. B.X1 loop
                  if J mod Stride = 0 and then B.Mask (I * Cw + J) and then not (Self_Px and then Z.Fingers (I * Cw + J)) then
                     declare
                        U : constant Long_Float := Long_Float (J);
                        V : constant Long_Float := Long_Float (I);
                     begin
                        if A2 >= 0 then
                           declare
                              P : constant Plug.Arm_Pose := F.EE (Natural (A2));
                           begin
                              Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P), D => Geom.Ray (G, P, U, V)));
                           end;
                        else
                           Rays.Append (Geom.Sight'(O => G.Pos, D => Geom.Ray_Fixed (G, U, V)));
                        end if;
                     end;
                  end if;
               end loop;
            end if;
         end loop;
      end;
      --  面过哪一点:它量到的位置(两眼交点)。可它躺在我碰过的那个面上:交点不可能在那个面之下,也不可能比我的张口还高出面(那样我也夹不住它)
      --  —— 两条视线都近乎竖直时交点的深度是病态的(H53 2026-09-23 实测:交点在桌面之下 9–28 cm)。出了这个范围就把面贴回碰过的那一点
      --  (当它厚度为零),并说出来;范围之内照用(那一截就是它的厚度)
      Contact.Surface.On_Plane (Rays, Plane_Point (C, P0, N, Say => True), N, Pts, Dropped);
      if Natural (Pts.Length) < 8 then   --  点数
         return;
      end if;
      declare
         Pitch : constant Long_Float := Contact.Gen.Sampling_Gap (Pts);
         --  这份点的预期误差:那只眼量朝向时的像素残差 ÷ 焦距 × 眼到面的距离(全是量过的数)。H57 2026-09-23 实测:头顶眼残差 11 px、离桌 1 m ⇒ 4 cm,
         --  腕眼 0.3 px、离桌 0.3 m ⇒ 0.2 mm;"留最细的一份"留下了头顶眼那份(3 mm 间距但整片偏了几厘米),三把合空 ⇒ 留【误差最小】的那份
         Eye_O : constant Geom.V3 := (if A2 >= 0 then [F.EE (Natural (A2)) (0), F.EE (Natural (A2)) (1), F.EE (Natural (A2)) (2)] else G.Pos);
         Sp0 : constant Geom.V3 := Pts.First_Element;
         Dist : constant Long_Float := Geom.Norm ([Sp0 (0) - Eye_O (0), Sp0 (1) - Eye_O (1), Sp0 (2) - Eye_O (2)]);
         Err : constant Long_Float := (if G.F > 0.0 then G.Rms * Dist / G.F else Dist);
         --  已有的那份(同一件)只让误差更小(相同则更细)的盖它。面的高度变了不算数:视线存着,碰到面后会按真高度重投
         --  (H60 2026-09-23 实测:交点高度一抖,头顶眼那份 2.5 cm 误差的把腕眼 0.2 mm 的盖掉了)
         Fresh : constant Boolean := C.Sil_Valid and then C.Sil_Name = Name;
      begin
         if Pitch <= 0.0 or else (Fresh and then (Err > C.Sil_Err or else (Err = C.Sil_Err and then Pitch > C.Sil_Pitch))) then
            return;
         end if;
         C.Sil_Pts := Pts; C.Sil_Valid := True; C.Sil_Name := Name; C.Sil_Cam := Integer (Cam); C.Sil_N := N; C.Sil_Pitch := Pitch; C.Sil_Err := Err;
         C.Sil_H_Sd := P0_Up_Sd;
         C.Sil_P0 := Sp0;   --  面过的点:就取这份点里的一个(它们全在那张面上)
         C.Sil_Rays := Rays;
         Geo_Say ("第" & Codec.Img (Cam) & " 台眼看全了它 ⇒ 记下它顶面的 " & Codec.Img (Natural (Pts.Length)) & " 个点(轮廓像素隔 " & Codec.Img (Stride)
                  & " 个取一个,落到它躺的面上,采样间距 " & Mm (Pitch) & ",预期误差 " & Mm (Err) & ";" & Codec.Img (Dropped) & " 条视线落不到面上)");
      end;
   end Take_Silhouette;

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

   procedure Plan_Contact (C : in out Context; F : Plug.Frame; Arm, Cam : Natural; Name : Unbounded_String;
                           Pick : out Contact.Grasp.Candidate; Note : out Unbounded_String; Ok : out Boolean) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Have_Plane : constant Boolean := C.Touch_Valid or else C.Board_Plane;
      Up : constant Geom.V3 := (if C.Touch_Valid then C.Touch_N elsif C.Board_Plane then C.Board_N else Protocol_Up);
      Sp : constant Geom.V3 := (if C.Touch_Valid then C.Touch_Pt else C.Board_Pt);
      Tol_P : constant Long_Float := Geo_Base (C, Arm);
      Tol_R : constant Long_Float := (if Arm * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (Arm * Chan.Per_Arm + 3) else 0.0);
      Mu_Lb : Long_Float := 0.0;
      Mu_Ub : Long_Float := Long_Float'Last;
      H : Contact.Grasp.Hand_Model;
      Top : Contact.V3_Vectors.Vector := C.Sil_Pts;
      Surf, Around : Contact.V3_Vectors.Vector;
      Found : Contact.Grasp.Cand_Vectors.Vector;
      St : Contact.Grasp.Plan_Stats;
      Reprojected : Boolean := False;
      --  眼在 (R, T) 时手的位姿:眼 → 世界 = 手 → 世界 · 相机 → 手 ⇒ 手 → 世界 = R · R_ceᵀ;眼的中心 = 手的位置 + 手 → 世界 · Off
      function Reach (R : Geom.M3; T : Geom.V3) return Boolean is
         Rp : constant Geom.M3 := Geom.Mul (R, Geom.Tr (G.R_Ce));
         Ow : constant Geom.V3 := Geom.Ap (Rp, G.Off);
         Pe, Re : Long_Float;
         Rok : Boolean;
      begin
         Plug.Reach (Arm, Kinem.To_Pose (Rp, [T (0) - Ow (0), T (1) - Ow (1), T (2) - Ow (2)]), Pe, Re, Rok);
         return not Rok or else (Pe <= Tol_P and then (Tol_R <= 0.0 or else Re <= Tol_R));
      end Reach;
   begin
      Ok := False;
      Pick := (others => <>);
      if not C.Sil_Valid or else C.Sil_Name /= Name then
         Note := S ("I have no measured outline of " & To_String (Name) & " (no eye saw it whole while I knew where it was)");
         return;
      end if;
      if not Have_Plane then
         Note := S ("I have not measured the surface it lies on, so I cannot tell how tall it is or where my fingers can go down beside it");
         return;
      end if;
      if Natural (G.Lobes.Length) /= 2 then
         Note := S ("my fingers in this eye are " & Codec.Img (Natural (G.Lobes.Length)) & " measured pads (I lay out holds for two pads closing on each other"
                    & (if G.Lobes.Is_Empty then "; my body file has no per-finger tips, it has to be measured once from scratch" else "") & ")");
         return;
      end if;
      H := Contact.Grasp.Two_Pads (G.Lobes (0).Tip, G.Lobes (1).Tip, Long_Float'Min (G.Lobes (0).Wide, G.Lobes (1).Wide),
                                   Long_Float'Max (G.Lobes (0).Thin, G.Lobes (1).Thin), Long_Float'Max (G.Tip_Sd, Tol_P));
      if not H.Valid then
         Note := S ("my measured fingers do not make a hand I can lay a hold out with: " & To_String (H.Why));
         return;
      end if;
      for Gm of C.Grip_Mus loop
         if Gm.Name = Name then
            Mu_Lb := Gm.Lb; Mu_Ub := Gm.Ub;
         end if;
      end loop;
      --  取轮廓时面的高度可能只是交点估的;碰过它躺的面 ⇒ 按真的面重投那些视线
      if C.Touch_Valid and then not C.Sil_Rays.Is_Empty then
         declare
            P0 : constant Geom.V3 := Plane_Point (C, C.Sil_P0, C.Sil_N, Say => False);
            Dropped : Natural;
            Again : Contact.V3_Vectors.Vector;
         begin
            if Geom.Norm ([P0 (0) - C.Sil_P0 (0), P0 (1) - C.Sil_P0 (1), P0 (2) - C.Sil_P0 (2)]) > C.Sil_Pitch then
               Contact.Surface.On_Plane (C.Sil_Rays, P0, C.Sil_N, Again, Dropped);
               if Natural (Again.Length) >= 8 then   --  点数
                  Top := Again;
                  Reprojected := True;
               end if;
            end if;
         end;
      end if;
      Contact.Surface.Walls_To_Support (Top, Up, Sp, C.Sil_Pitch, Surf);
      --  旁边的东西:被顶住过、比面高、离它自己的表面点超过两个采样间距的(贴着它的那些是它自己被顶住)
      for B of C.Bumps loop
         declare
            Near : Boolean := False;
         begin
            for Q of Surf loop
               if Geom.Norm ([B (0) - Q (0), B (1) - Q (1), B (2) - Q (2)]) <= 2.0 * C.Sil_Pitch then   --  两个采样间距(纯几何:对角邻居在 √2 个以内)
                  Near := True;
                  exit;
               end if;
            end loop;
            if not Near then
               Around.Append (B);
            end if;
         end;
      end loop;
      Contact.Grasp.Plan (Surf, Around, C.Sil_Pitch, C.Sil_Err, Up, Sp, H, Mu_Lb, G.Gap, Reach'Access, 8, Found, St);   --  留前 8 组(个数)
      --  量到的摩擦上限:这件东西以前没拿住过的那一组要的摩擦它给不起 ⇒ 要得比它还多的不要
      declare
         Kept : Contact.Grasp.Cand_Vectors.Vector;
      begin
         for Cd of Found loop
            if Cd.Mu_Nom < Mu_Ub then
               Kept.Append (Cd);
            end if;
         end loop;
         if Kept.Is_Empty then
            Note := S ("from " & Codec.Img (Natural (Surf.Length)) & " surface points of " & To_String (Name) & " (top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam)))
                       & " pulled straight down to the surface it lies on, which assumes solid upright sides) I tried " & Codec.Img (St.Poses) & " placements of my hand: "
                       & Codec.Img (St.Air) & " close on nothing, " & Codec.Img (St.Landed_On) & " put a finger down on it, " & Codec.Img (St.Blocked) & " hit something beside it, "
                       & Codec.Img (St.Palm_Hit) & " push it into my palm, " & Codec.Img (St.No_Hold) & " cannot hold it up, " & Codec.Img (St.Unreachable) & " out of my reach"
                       & (if Natural (Found.Length) > 0 then ", and every hold left needs more friction than " & To_String (Name) & " gave me before" else ""));
            return;
         end if;
         Pick := Kept (0);
         Note := S ("hold on " & To_String (Name) & ": " & Codec.Img (Natural (Kept.Length)) & " holds kept of " & Codec.Img (St.Poses) & " hand placements (from "
                    & Codec.Img (Natural (Surf.Length)) & " surface points, top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam))) & " at " & Len (C, C.Sil_Pitch)
                    & " pitch, expected error " & Len (C, C.Sil_Err) & (if Reprojected then ", re-laid on the surface I touched" else "")
                    & ", sides assumed solid and upright down to the surface); best: fingers " & Len (C, Pick.Width) & " apart, coming in "
                    & Codec.Fmt (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, -(Pick.Approach (0) * Up (0) + Pick.Approach (1) * Up (1) + Pick.Approach (2) * Up (2))))), 2)
                    & " rad from straight down"
                    & ", closing " & Len (C, Pick.Pre) & " before going down, needs friction at least " & Codec.Fmt (Pick.Mu_Worst, 2) & " in the worst case"
                    & (if Mu_Lb > 0.0 then " (" & To_String (Name) & " has held at " & Codec.Fmt (Mu_Lb, 2) & ")" else " (friction on it not measured yet)"));
         Ok := True;
      end;
   end Plan_Contact;

   --  转这只手,让它自己那只眼的正前方对准世界里的一个方向(Want,单位向量)。
   --  转最少的角度:转轴 = 现在的正前方 × 要的方向。指尖不许甩走(08-28 那次甩出 20 cm):每一步先按要转的角度算出
   --  指尖会挪到哪,再用平移把它补回原处 —— 指尖偏置是量过的。一条命令最多转多少 = 开机量出来的"一条命令转得到的最大一档"(G.Stride_Rot)× 脑的档位;
   --  转了没转到(不到一半)⇒ 如实说,不硬转。到位的判据 = 差不到一个转动探针幅度(身体量过的最小一档)。
   --  Along:我身上要拿去对准的那根方向(在这只眼的坐标里):默认是眼的正前方 [0,0,-1];要让【手指】指向某处就传指尖方向(G.Tip 归一化)。
   procedure Geo_Turn (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Want : Geom.V3;
                       Amt : Long_Float; Event : out Unbounded_String; Steps_Taken : out Natural;
                       Along : Geom.V3 := [0.0, 0.0, -1.0]) is
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      Notch : constant Long_Float := (if Arm * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (Arm * Chan.Per_Arm + 3) else 0.0);
      --  一条命令最多转多少 = 开机按运动学量出来的"一条命令转得到的最大一档"(看着走,09-28 定:步子是身体的事,不再乘脑的档位;
      --  C1 09-29:乘了一半,转 0.4 rad 花 5 条命令)。Amt 不再用来定步子
      pragma Unreferenced (Amt);
      Cap : constant Long_Float := (if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) then C.Geo (Natural (Hc)).Stride_Rot else 0.0);
   begin
      Event := Null_Unbounded_String; Steps_Taken := 0;
      if Hc < 0 or else Natural (Hc) >= Natural (C.Geo.Length) or else Notch <= 0.0 or else Cap <= 0.0 then
         Event := S ("refused: I cannot turn this eye - it does not ride on this arm, or my turning stride has not been measured");
         return;
      end if;
      --  这只眼在手上怎么装的只有一种量法:开机量(运动学 + 对齐);开机没量出 ⇒ 照实说(09-30 删了"干活时盯着一块挪几下现量"那条后备)
      if not C.Geo (Natural (Hc)).Valid then
         Event := S ("refused: I cannot turn this eye - how it sits on my hand was not measured at boot");
         return;
      end if;
      declare
         G : constant Geom.Cam_Geo := C.Geo (Natural (Hc));
      begin
         for Step_No in 1 .. 40 loop
            if Plug.Reset_Pending (L) then
               Event := S (Reset_Event);
               return;
            end if;
            declare
               P : constant Plug.Arm_Pose := F.EE (Arm);
               Rc : constant Geom.M3 := Geom.Cam_R (G, P);
               Al : constant Long_Float := Geom.Norm (Along);
               Fwd : constant Geom.V3 := Geom.Ap (Rc, (if Al > 0.0 then [Along (0) / Al, Along (1) / Al, Along (2) / Al] else [0.0, 0.0, -1.0]));
               Cr : constant Geom.V3 := [Fwd (1) * Want (2) - Fwd (2) * Want (1), Fwd (2) * Want (0) - Fwd (0) * Want (2), Fwd (0) * Want (1) - Fwd (1) * Want (0)];
               Sn : constant Long_Float := Geom.Norm (Cr);
               Cs : constant Long_Float := Fwd (0) * Want (0) + Fwd (1) * Want (1) + Fwd (2) * Want (2);
               Ang : constant Long_Float := Arctan (Sn, Cs);
            begin
               if Ang <= Notch then
                  Event := S ("amount: arrived (my eye now points there, off by " & Codec.Fmt (Ang, 3) & " rad)");
                  return;
               end if;
               declare
                  --  正前方正好背对着要的方向时叉积为零:随便取一根和正前方垂直的轴(和世界 z、世界 x 各叉一次,取长的那根)
                  Az : constant Geom.V3 := [Fwd (1), -Fwd (0), 0.0];          --  Fwd × z
                  Ax : constant Geom.V3 := [0.0, Fwd (2), -Fwd (1)];          --  Fwd × x
                  Alt : constant Geom.V3 := (if Geom.Norm (Az) >= Geom.Norm (Ax) then Az else Ax);
                  Aln : constant Long_Float := Geom.Norm (Alt);
                  Axis : constant Geom.V3 := (if Sn > 1.0e-9 then [Cr (0) / Sn, Cr (1) / Sn, Cr (2) / Sn]
                                              elsif Aln > 1.0e-9 then [Alt (0) / Aln, Alt (1) / Aln, Alt (2) / Aln] else [0.0, 0.0, 1.0]);
                  Stp : constant Long_Float := Long_Float'Min (Ang, Cap);
                  Rv : constant Geom.V3 := [Axis (0) * Stp, Axis (1) * Stp, Axis (2) * Stp];
                  Rn : constant Geom.M3 := Geom.Mul (Geom.Rodrigues (Rv), Geom.Quat_To_R (P));
                  Tip0 : constant Geom.V3 := Geom.Ap (Rc, G.Tip);
                  Tip1 : constant Geom.V3 := Geom.Ap (Geom.Mul (Rn, G.R_Ce), G.Tip);
                  A : Table.Vec := Table.Zero_Vec;
                  Jaw : Floats;
                  Del : Table.Vec;
                  Ok : Boolean;
               begin
                  A (0) := Tip0 (0) - Tip1 (0); A (1) := Tip0 (1) - Tip1 (1); A (2) := Tip0 (2) - Tip1 (2);
                  A (3) := Rv (0); A (4) := Rv (1); A (5) := Rv (2);
                  Step_Arm (L, C, F, Arm, A, Jaw, Del, Ok, Geo_Settle => True);
                  Steps_Taken := Steps_Taken + 1;
                  declare
                     Got : constant Long_Float := Del (3) * Axis (0) + Del (4) * Axis (1) + Del (5) * Axis (2);
                  begin
                     Geo_Say ("转 " & Codec.Fmt (Stp, 3) & " rad ⇒ 实到 " & Codec.Fmt (Got, 3) & " rad(还差 " & Codec.Fmt (Ang, 3) & ")"
                              & (if Ok then "" else " · 身体说没走成"));
                     if Got + Got < Stp then
                        Event := S ("resist: I commanded a turn of " & Codec.Fmt (Stp, 3) & " rad and my hand only turned " & Codec.Fmt (Got, 3)
                                    & " (still " & Codec.Fmt (Ang, 3) & " rad from pointing there) - a joint is at its end or the pose is not reachable");
                        return;
                     end if;
                  end;
               end;
            end;
         end loop;
         Event := S ("steps: I took 40 turning steps and my eye is still not pointing there");
      end;
   end Geo_Turn;

   --  不动的眼看见了它,长在手上的眼还没看见 ⇒ 把那只眼转向它:不动的眼的视线 ∩ 它躺着的面 = 它在哪(面 = 我最后碰过的那个面;
   --  一次都没碰过就拿指尖此刻的高度当面,并说出来),再让手眼的正前方指向那一点。
   procedure Aim_Eye_At (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Fixed_Cam : Natural;
                         U, V : Long_Float; Amt : Long_Float; Event : out Unbounded_String; Ok : out Boolean) is
      G0 : constant Geom.Cam_Geo := Geo_Of (C, Fixed_Cam);
      Ray : constant Geom.V3 := Geom.Ray_Fixed (G0, U, V);
      P0 : constant Geom.V3 := (if C.Touch_Valid then C.Touch_Pt else Tip_World (C, Arm, F.EE (Arm)));
      Nn : constant Geom.V3 := Up_Dir (C);
      Hok : Boolean;
      P : constant Geom.V3 := Geom.Hit_Plane (G0.Pos, Ray, P0, Nn, Hok);
      Steps : Natural;
   begin
      Ok := False;
      Event := Null_Unbounded_String;
      if not G0.Fixed then
         Event := S ("refused: the eye that sees it does not know where it sits in the world - I have not measured that yet");
         return;
      end if;
      if not Hok then
         Event := S ("lost: the still eye's line of sight to it does not meet the surface I know");
         return;
      end if;
      declare
         Hp : constant Plug.Arm_Pose := F.EE (Arm);
         D : Geom.V3 := [P (0) - Hp (0), P (1) - Hp (1), P (2) - Hp (2)];
         Ln : constant Long_Float := Geom.Norm (D);
      begin
         Geo_Say ("不动的眼说它在 (" & Mm (P (0)) & "," & Mm (P (1)) & "," & Mm (P (2)) & ")"
                  & (if C.Touch_Valid then "(视线落到我碰过的那个面上)" else "(还没碰过任何面,先按指尖此刻的高度算)")
                  & ",离第" & Codec.Img (Arm + 1) & " 只手 " & Mm (Ln) & " ⇒ 把这只手的眼转向它");
         if Ln <= 0.0 then
            return;
         end if;
         D := [D (0) / Ln, D (1) / Ln, D (2) / Ln];
         Geo_Turn (L, C, F, Arm, D, Amt, Event, Steps);
         Ok := Index (Event, "amount: arrived") > 0;
      end;
   end Aim_Eye_At;

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
   procedure Unify_By_Sight (C : in out Context; F : Plug.Frame; Cam : Natural; W : String; R : Picture.Region) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      A1 : constant Integer := Cam_Arm (C, Cam);
      Kw : constant Natural := F.Cams (Cam).W;
      Kh : constant Natural := F.Cams (Cam).H;
      Ok1 : Boolean := False;
      S1 : Geom.Sight;
   begin
      if A1 < 0 and then G.Fixed then
         S1 := (O => G.Pos, D => Geom.Ray_Fixed (G, R.Cu * Long_Float (Kw), R.Cv * Long_Float (Kh)));
         Ok1 := True;
      elsif A1 >= 0 and then G.Valid and then G.F > 0.0 and then A1 < Integer (F.EE.Length) then
         declare
            P : constant Plug.Arm_Pose := F.EE (Natural (A1));
         begin
            S1 := (O => Geom.Cam_Pos (G, P), D => Geom.Ray (G, P, R.Cu * Long_Float (Kw), R.Cv * Long_Float (Kh)));
            Ok1 := True;
         end;
      end if;
      if not Ok1 then
         return;
      end if;
      for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
         declare
            B : constant Boxed_Thing := C.Boxed (Bi);
         begin
            if B.Cam /= Cam and then B.Seen and then To_String (B.Name) /= W and then B.Cam < Natural (C.Geo.Length) and then B.Cam < Natural (F.Cams.Length) then
               declare
                  Gm : constant Geom.Cam_Geo := C.Geo (B.Cam);
                  A2 : constant Integer := Cam_Arm (C, B.Cam);
                  W2 : constant Natural := F.Cams (B.Cam).W;
                  H2 : constant Natural := F.Cams (B.Cam).H;
                  S2 : Geom.Sight;
                  Ok2 : Boolean := False;
               begin
                  if A2 < 0 and then Gm.Fixed then
                     S2 := (O => Gm.Pos, D => Geom.Ray_Fixed (Gm, B.Cu * Long_Float (W2), B.Cv * Long_Float (H2)));
                     Ok2 := True;
                  elsif A2 >= 0 and then Gm.Valid and then Gm.F > 0.0 and then A2 < Integer (F.EE.Length) then
                     declare
                        P2 : constant Plug.Arm_Pose := F.EE (Natural (A2));
                     begin
                        S2 := (O => Geom.Cam_Pos (Gm, P2), D => Geom.Ray (Gm, P2, B.Cu * Long_Float (W2), B.Cv * Long_Float (H2)));
                        Ok2 := True;
                     end;
                  end if;
                  if Ok2 then
                     declare
                        Rays : Geom.Sight_Vectors.Vector;
                        Mok : Boolean;
                        Spread : Long_Float;
                        Pm : Geom.V3;
                     begin
                        Rays.Append (S1);
                        Rays.Append (S2);
                        Pm := Geom.Meet (Rays, Mok, Spread);
                        if Mok then
                           declare
                              --  那块东西自己有多大(米):它在那只眼里框的对角线 × 那只眼到交点的距离 ÷ 焦距(全是量出来的);两条视线差得不超过它的一半(纯比例)就是同一件
                              D2 : constant Long_Float := Geom.Norm ([Pm (0) - S2.O (0), Pm (1) - S2.O (1), Pm (2) - S2.O (2)]);
                              Diag : constant Long_Float := Sqrt (Long_Float (B.X1 - B.X0 + 1) ** 2 + Long_Float (B.Y1 - B.Y0 + 1) ** 2);
                              Size : constant Long_Float := (if Gm.F > 0.0 then Diag * D2 / Gm.F else 0.0);
                           begin
                              if Spread <= 0.5 * Size then
                                 declare
                                    Old : constant String := To_String (B.Name);
                                    B2 : Boxed_Thing := B;
                                 begin
                                    Put_Line ("[身] 📦 第" & Codec.Img (B.Cam) & " 台里你叫「" & Old & "」的和这只眼里你叫「" & W & "」的,视线交在一点(偏差 "
                                              & Mm (Spread) & ",它本身约 " & Mm (Size) & ")⇒ 同一件,以后都叫它「" & W & "」");
                                    B2.Name := To_Unbounded_String (W);
                                    C.Boxed.Replace_Element (Bi, B2);
                                    if To_String (C.Geo_Pw_Name) = Old then
                                       C.Geo_Pw_Name := To_Unbounded_String (W);
                                    end if;
                                    if To_String (C.Geo_Name) = Old then
                                       C.Geo_Name := To_Unbounded_String (W);
                                    end if;
                                    if To_String (C.Sil_Name) = Old then
                                       C.Sil_Name := To_Unbounded_String (W);
                                    end if;
                                 end;
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Unify_By_Sight;

   --  这只眼里没有它的窗(脑没在这只眼里点过它的名),可它顶面的点我量过 ⇒ 把那些点投进这只眼,外接框就是窗;窗里哪一片是它照常每帧重量。
   --  名字是脑起的、点是我量的:这不是替脑认东西,是把量到的东西送进另一只眼(换手之后左手的眼里本来什么都没有)
   procedure Window_From_Outline (C : in out Context; F : Plug.Frame; Cam : Natural; Name : Unbounded_String) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      A2 : constant Integer := Cam_Arm (C, Cam);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      X0 : Integer := Integer'Last;
      Y0 : Integer := Integer'Last;
      X1 : Integer := -1;
      Y1 : Integer := -1;
      N : Natural := 0;
      Bt : Boxed_Thing;
   begin
      if Boxed_By (C, Cam, Name) >= 0 or else not C.Sil_Valid or else C.Sil_Name /= Name then
         return;
      end if;
      if not ((A2 < 0 and then G.Fixed) or else (A2 >= 0 and then G.Valid and then G.F > 0.0 and then A2 < Integer (F.EE.Length))) then
         return;
      end if;
      for P of C.Sil_Pts loop
         declare
            U, V : Long_Float;
            Front : Boolean;
         begin
            if A2 < 0 then
               Geom.Project_Fixed (G, P, U, V, Front);
            else
               Geom.Project (G, F.EE (Natural (A2)), P, U, V, Front);
            end if;
            if Front and then U >= 0.0 and then V >= 0.0 and then U < Long_Float (Cw) and then V < Long_Float (Ch) then
               X0 := Integer'Min (X0, Integer (U));
               X1 := Integer'Max (X1, Integer (U));
               Y0 := Integer'Min (Y0, Integer (V));
               Y1 := Integer'Max (Y1, Integer (V));
               N := N + 1;
            end if;
         end;
      end loop;
      if N < 8 or else X1 <= X0 or else Y1 <= Y0 then   --  点数
         return;
      end if;
      Bt.Name := Name;
      Bt.Cam := Cam;
      Bt.X0 := X0; Bt.Y0 := Y0; Bt.X1 := X1; Bt.Y1 := Y1;
      Bt.Cu := Long_Float (X0 + X1) / 2.0 / Long_Float (Cw);
      Bt.Cv := Long_Float (Y0 + Y1) / 2.0 / Long_Float (Ch);
      Bt.Seen := False;
      C.Boxed.Append (Bt);
      C.Cut_Cam := -1;
      Geo_Say ("第" & Codec.Img (Cam) & " 台眼里没有它的窗 ⇒ 把它顶面的点投进这只眼:窗 [" & Codec.Img (X0) & " " & Codec.Img (Y0) & " " & Codec.Img (X1) & " " & Codec.Img (Y1)
               & "](" & Codec.Img (N) & " 个点落在画面里),窗里哪一片是它照常重量");
   end Window_From_Outline;

   --  ── 此刻每一只看得见它的眼给一条视线 ──(它叫 Its_Name,脑点过名的)
   --  眼可以是:正在走路的这只手自己的眼(Seen 且整块)、不动的眼(量过自己在哪)、另一只手的眼(朝向量过)。
   --  两条以上 ⇒ 交点就是它此刻的位置,它动不动都一样;这是抓会动的东西唯一诚实的量法(owner 09-22)。
   --  一只眼的视线有多不准(弧度,一倍标准差)= 它量朝向时的像素残差 ÷ 焦距;没量过 ⇒ 0(交点那一步就照实说量不出)
   function Ray_Sd (G : Geom.Cam_Geo) return Long_Float is (if G.F > 0.0 and then G.Rms > 0.0 then G.Rms / G.F else 0.0);

   function Sightlines_Now (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Its_Name : Unbounded_String;
                            Seen, Whole : Boolean; U, V : Long_Float; Who : out Unbounded_String; Sds : out Floats) return Geom.Sight_Vectors.Vector is
      Rays : Geom.Sight_Vectors.Vector;
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   begin
      Who := Null_Unbounded_String;
      Sds.Clear;
      if Seen and then Whole then
         declare
            P : constant Plug.Arm_Pose := F.EE (Arm);
            Rok : Boolean;
            D : constant Geom.V3 := Geom.Ray (G, P, U, V, Rok);   --  去不了畸变(像素在镜头模型够不到的地方)⇒ 这只眼不给视线
         begin
            if Rok then
               Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P), D => D));
               Sds.Append (Ray_Sd (G));
            end if;
            Append (Who, "第" & Codec.Img (Cam) & " 台");
         end;
      end if;
      if Length (Its_Name) = 0 then
         return Rays;
      end if;
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         if Cm /= Cam and then Cm < Natural (C.Geo.Length) and then Cm < Natural (F.Cams.Length) then
            declare
               Gm : constant Geom.Cam_Geo := C.Geo (Cm);
               A2 : constant Integer := Cam_Arm (C, Cm);
               Usable : constant Boolean := (A2 < 0 and then Gm.Fixed) or else (A2 >= 0 and then Gm.Valid and then Gm.F > 0.0 and then A2 < Integer (F.EE.Length));
            begin
               if Usable then
                  --  这只眼这一帧再量一遍它(脑点过名 ⇒ 在上一帧量到它的地方原样重量)
                  World.Observe (C.Wld, Cm, Cut_Things (C, F, Cm), F.Cams (Cm).W, F.Cams (Cm).H);
                  for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                     declare
                        B : constant Boxed_Thing := C.Boxed (Bi);
                        Edge : constant Boolean := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cm).W or else B.Y1 + 1 >= F.Cams (Cm).H;
                     begin
                        if B.Cam = Cm and then B.Seen and then not Edge and then B.Name = Its_Name then
                           declare
                              Pu : constant Long_Float := B.Cu * Long_Float (F.Cams (Cm).W);
                              Pv : constant Long_Float := B.Cv * Long_Float (F.Cams (Cm).H);
                              Hand_On_It : constant Boolean := Hand_Covers (C, F, Arm, Cm, B);
                           begin
                              if Hand_On_It then
                                 null;   --  这一眼不给视线
                              elsif A2 < 0 then
                                 declare
                                    Rok : Boolean;
                                    D : constant Geom.V3 := Geom.Ray_Fixed (Gm, Pu, Pv, Rok);
                                 begin
                                    if Rok then
                                       Rays.Append (Geom.Sight'(O => Gm.Pos, D => D));
                                       Sds.Append (Ray_Sd (Gm));
                                    end if;
                                 end;
                              else
                                 declare
                                    P2 : constant Plug.Arm_Pose := F.EE (Natural (A2));
                                    Rok : Boolean;
                                    D : constant Geom.V3 := Geom.Ray (Gm, P2, Pu, Pv, Rok);
                                 begin
                                    if Rok then
                                       Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (Gm, P2), D => D));
                                       Sds.Append (Ray_Sd (Gm));
                                    end if;
                                 end;
                              end if;
                              Append (Who, (if Length (Who) > 0 then "+" else "") & "第" & Codec.Img (Cm) & " 台");
                           end;
                        end if;
                     end;
                  end loop;
               end if;
            end;
         end if;
      end loop;
      return Rays;
   end Sightlines_Now;

   --  Above = True:不是走到它跟前,而是走到它【正上方、高出一个张口】(张口是身体量过的长度,不是拍的数)。
   --  "上" = 位姿读数系的 +z,和抬手那一条同一个约定(当它朝上;真机该由重力读数定)。
   procedure Geo_Approach (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Slot : Integer;
                           Step_Limit : Natural; Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural;
                           Above : Boolean := False; Amt : Long_Float := 0.5; Until_Touch : Boolean := False;
                           Name : Unbounded_String := Null_Unbounded_String) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Beats0 : constant Natural := Plug.Steps (L);
      --  🔴 没说步数 ⇒ 我不设上限:走到到位 / 碰到 / 被顶住 / 看丢为止(身体不许自己收工)。H28 2026-09-22 实测:我自设的 12 推上限
      --  (其中 5 推是转眼)让 above 停在离合拢点 0.111 m 处,还报"你要的步数走完了"—— 脑根本没要过步数,接着就合了个空。
      Limit : constant Natural := (if Step_Limit > 0 then Step_Limit else Natural'Last);
      Tol : constant Long_Float := 0.1 * G.Gap;      --  到位容差 = 张口的一成(比例,无量纲)
      Inward : constant Long_Float := 0.15 * G.Gap;  --  指尖中点再往手心里一点 = 张口的 15%(比例,无量纲):别咬在皮上
      Want : Geom.V3 := G.Tip;
      U, V : Long_Float;
      Seen, Mok : Boolean;
      --  🔴 一条命令最多走多远,由【脑说的步子档位】定(small / medium / large,语言 §4.3:按身体自己量出的幅度计价),不由我自己调。
      --  单位 = 测距那一下横挪的大小(4 倍探针幅度,每一段开头它都刚被证明走得到);small = 1 个单位,medium = 2,large = 4。
      --  H8 2026-09-22 实测为什么要有上限:横挪 0.026 m 实到 0.024 m;之后每步命令 0.14 m(其中往下 0.097 m)实到 ≈ 0,连着 5 步 ——
      --  仿真日志 65 行 "continuous ik did not converge … falling back to global IK":大步先被连续逆解拒掉,退回全局逆解
      --  又因为目标在桌面高度而无解 ⇒ 静默不动。GB5 的球心离桌面 3.4 cm,一步 170 mm 过得去;平躺的剪刀过不去。
      --  ⚠️ 我先写过一版"走成了加倍、没走成减半",被自由棘轮拦下(owner 09-03:驱动不许自己调步子)—— 已撤。
      --  命令了没走到 ⇒ 我不自己换打法,如实说"没走成"交回脑(它可以说 small,也可以说合手)。
      --  一条命令最多走多远:看着走(09-28 定)—— 不再乘脑的档位;远的时候走还差的六成再看一眼(下面的 Frac),
      --  一条命令走不到的那一截由"到过的范围 + 往外一步"拆开、手一动就跟着往前重发(Selfmap.Go);量出来的最大一档只用来判"量过没有"
      Step_Cap : constant Long_Float := (if Stride_Of (C, Arm) > 0.0 then Long_Float'Last else 0.0);
      --  🔴 被一个面顶住之后:顶住的只是【那个方向】(命令了没走到的那个方向,量出来的),剩下的误差里沿着面的那一部分照样走得了。
      --  H12 2026-09-22 实测:垂直下探碰到桌面即停,此刻剪刀在两指正前方 0.021 m(沿桌面);整段就此停下 ⇒ 合手合了个空(读数 0.000 = 空手值)。
      --  "touching" 要的是合拢点到它身上;桌面不让我再往下,不等于不让我往前。这是在量到的接触下继续解同一个约束,不是换打法。
      --  🔴 身体不许自己收工(总规矩 09-13 / 分叉最后一个提交 09-18):脑写的是 until touched,那就一直往它身上走到【真的碰到】为止;
      --  我自己估出来的"到位了"只能说出来,不能当停的理由。H13 2026-09-22 实测:估计差 0.005 m 就停了,指尖还悬在剪刀上方,
      --  合手合到 0.000(空手值)。平躺在桌上的东西,只有往下走到被桌面顶住,指尖才真的在它两侧。
      --  🔴 不可信的观测不进解算。近处它有一截出了画面,"看到的那一块"的形心不再是同一个物理点(H14 2026-09-22 实测:
      --  下探到近处,估计位置乱跳,手往上往后走了两步,然后"看丢了")。远处那几眼看到的是完整的一块,交出来的位置是准的,
      --  而手的位姿读数每步只差 1 mm ⇒ 看不全了就不再更新它的位置,凭已知位置 + 位姿读数走完(LAB D2:不看也在)。
      Whole, Edge : Boolean;
      Its_Name : Unbounded_String;
      Said_Blind : Boolean := False;
      Known : Boolean := False;             --  此刻没眼看得清它,但它在哪我量过(C.Geo_Pw)
      Said_Known : Boolean := False;
      Said_Cut : Boolean := False;          --  说过一次"这只眼里它顶着画面边,轮廓不记"
      Said_Clamp : Boolean := False;        --  说过一次"交点出了它躺的面的范围,贴回面上"
      Who : Unbounded_String;
      Pressing : Boolean := False;          --  估计已到位,正沿原方向接着往它身上走
      Press_Dir : Geom.V3 := [0.0, 0.0, 0.0];
      Held_Back : Boolean := False;
      Wall : Geom.V3 := [0.0, 0.0, 0.0];   --  顶住我的那个方向(世界系单位向量,指向面里)
   begin
      Event := Null_Unbounded_String; Steps_Taken := 0; Beats := 0;
      Want (2) := Want (2) + Inward;   --  相机 -z 朝前 ⇒ 往手心方向 = +z
      --  🔴 每一段从头量:上一段留下的那几眼(转过手、离得远)和这一段近处的眼搅在一起,交点会飞
      --  (H25 2026-09-22 实测:悬停 12 cm 处重新指了它,交点却算到 0.7 m 外、偏 54 cm,手往反方向走)。
      --  两只眼同时看见就一帧出数;只有一只眼就横挪一步当基线 —— 这一段自己的眼。
      C.Geo_Obs.Clear;
      --  上一段末尾转过手/挪过手(指尖朝下那一转尤其大)⇒ 它在这只眼里早不在旧窗那儿了;它在哪我量过 ⇒ 先把窗投到它该在的地方
      if Length (Name) > 0 and then C.Geo_Pw_Valid and then C.Geo_Pw_Name = Name then
         Retarget_Box (C, F, Cam, Arm, Name, C.Geo_Pw);
      end if;
      if Length (Name) > 0 then
         Window_From_Outline (C, F, Cam, Name);
      end if;
      Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
      Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
      --  🔴 看着它走:它被画面边切掉时形心不是同一个物理点,H21 2026-09-22 实测整段路只有开头两眼算数,
      --  一条 26 mm 的基线量 0.42 m 外的东西,落点偏了 5–8 cm。⇒ 被画面边切到就先转眼把它整个看进来。
      if Above then
         C.Fingers_Aimed := False;   --  又要去它上方 ⇒ 到了再重新指
      end if;
      if Seen and then Edge and then not (C.Fingers_Aimed and then not Above) then
         declare
            Ev : Unbounded_String;
            St : Natural;
            --  🔴 先算好再传:从 F 算出的东西不许直接当实参交给会改 F 的调用(GC12 / H24 2026-09-22 同一处崩:
            --  Plug.Sense 里帧的 finalize 报 PROGRAM_ERROR)
            Ray_Ok : Boolean;
            Want : constant Geom.V3 := Geom.Ray (G, F.EE (Arm), U, V, Ray_Ok);
         begin
            if Ray_Ok then
               Geo_Turn (L, C, F, Arm, Want, Amt, Ev, St);
            else
               Ev := S ("its pixel is outside what my lens model covers, so I cannot tell which way to turn");
               St := 0;
            end if;
            Steps_Taken := Steps_Taken + St;
            Geo_Say ("它被画面边切着 ⇒ 转眼看着它(" & To_String (Ev) & ")");
            --  转过之后它的方向没变(世界系那条视线),把窗投到这条视线在新位姿画面里的落点
            declare
               Pn : constant Plug.Arm_Pose := F.EE (Arm);
            begin
               Retarget_Box (C, F, Cam, Arm, Name,
                             (if C.Geo_Pw_Valid and then C.Geo_Pw_Name = Name then C.Geo_Pw
                              else [Pn (0) + Want (0), Pn (1) + Want (1), Pn (2) + Want (2)]));
            end;
            Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
            Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
         end;
      end if;
      if (Length (Its_Name) = 0 and then C.Geo_Slot /= Slot) or else (Length (Its_Name) > 0 and then C.Geo_Name /= Its_Name) then
         C.Geo_Obs.Clear; C.Geo_Came := 0.0;
      end if;
      C.Geo_Slot := Slot;
      if Length (Its_Name) > 0 then
         C.Geo_Name := Its_Name;
      end if;
      --  🔴 此刻没有一只眼看得清它,但它在哪我上一段刚量过 ⇒ 凭记住的位置走,并如实说前提是它没动。
      --  H30 2026-09-22 实测:手贴到剪刀 8 mm 时腕眼里它糊了、被切了,脑指不出 ⇒ 我报"看不见"、一步不走,而它在哪我明明知道。
      Known := Length (Its_Name) > 0 and then C.Geo_Pw_Valid and then C.Geo_Pw_Name = Its_Name;
      --  它在哪只有一种量法:两只眼同一刻的视线交点(09-22 owner 定的地基;09-26 owner:一个量只许一种量法 ⇒
      --  以前的"一条视线落到它躺的面上""我自己横挪几眼算视差(前提是它没动)"都删了)。此刻交不上 ⇒ 用上一次交出来的位置并说出来
      if not Seen and then not Known then
         Event := S ("lost: I cannot see the thing you named in this eye right now");
         return;
      end if;
      if Step_Cap <= 0.0 then
         Event := S ("refused: I have not measured how far one command moves this arm, so I cannot walk toward it");
         return;
      end if;
      loop
         if Plug.Reset_Pending (L) then
            Event := S (Reset_Event);
            exit;
         end if;         declare
            Cur : constant Plug.Arm_Pose := F.EE (Arm);
            Pw, Pc, D : Geom.V3;
            Pw_Up_Sd : Long_Float := Long_Float'Last;   --  它的位置沿"上"有多不准(交点的几何算出来的;量不出 = 最大)
            Dist : Long_Float;
         begin
            --  🔴 它此刻在哪:问【此刻】每一只看得见它的眼 —— 两条以上视线一交就是它,它动不动都一样。这是唯一的量法;
            --  交不上 ⇒ 用上一次两眼交出来的位置并说出来(09-26 删了"我自己挪过的那几眼"和"一条视线落到它躺的面上"两种)。
            declare
               Sds : Floats;
               Rays : constant Geom.Sight_Vectors.Vector := Sightlines_Now (C, F, Cam, Arm, Its_Name, Seen, Whole, U, V, Who, Sds);
               Mok : Boolean;
               Spread : Long_Float;
               Pm : Geom.V3;
            begin
               Pm := Geom.Meet (Rays, Mok, Spread);
               --  🔴 交点可信的条件:几条视线离交点的最大偏差不超过【眼自己量朝向时的像素残差】换算到那个距离上的米数(量过的数,不是拍的)。
               --  H42 2026-09-22 实测:一个偏差 0.079 m 的交点被当真记住,后面每一段都往 8 cm 高的空中走。
               if Mok then
                  declare
                     Gw : constant Geom.Cam_Geo := Geo_Of (C, C.Map.World_Cam);
                     Hp : constant Plug.Arm_Pose := F.EE (Arm);
                     Tol_Hand : constant Long_Float := (if G.F > 0.0 then G.Rms * Geom.Norm ([Pm (0) - Hp (0), Pm (1) - Hp (1), Pm (2) - Hp (2)]) / G.F else 0.0);
                     Tol_Still : constant Long_Float := (if Gw.Fixed and then Gw.F > 0.0 then Gw.Rms * Geom.Norm ([Pm (0) - Gw.Pos (0), Pm (1) - Gw.Pos (1), Pm (2) - Gw.Pos (2)]) / Gw.F else 0.0);
                     Tol : constant Long_Float := Long_Float'Max (Tol_Hand, Tol_Still);
                  begin
                     --  09-13 总规矩:动起来之后身体不许有闸 ⇒ 交点照用,偏差说出来(它就是这个位置有多不准)。
                     --  以前偏差超过眼的误差就扔掉交点(H42 之后加的):标定准到 1 mm 之后,长条的东西被画面边切着、两只眼的"中心"不是同一点,
                     --  1.7 cm 的偏差回回被扔,SHOT1 一集扔了 32 次、一次都没走到它身上
                     if Spread > Tol then
                        Geo_Say ("此刻 " & To_String (Who) & " 相机的视线交在 (" & Mm (Pm (0)) & "," & Mm (Pm (1)) & "," & Mm (Pm (2)) & "),视线间偏差 "
                                 & Mm (Spread) & ",比眼自己的误差(" & Mm (Tol) & ")大 —— 两只眼看到的中心可能不是同一点;照这个交点走,它的位置按差 " & Mm (Spread) & " 算");
                     end if;
                  end;
               end if;
               if Mok then
                  Pw := Pm;
                  Pw_Up_Sd := Geom.Meet_Sd (Rays, Sds, Pm, Up_Dir (C));
                  Geo_Say ("此刻 " & To_String (Who) & " 相机同时看见它 ⇒ 视线交在 (" & Mm (Pw (0)) & "," & Mm (Pw (1)) & "," & Mm (Pw (2))
                           & "),视线间最大偏差 " & Mm (Spread) & ";按两只眼各自的误差和视线夹角,高低上不准 "
                           & (if Pw_Up_Sd < Long_Float'Last then Mm (Pw_Up_Sd) else "(量不出)"));
               elsif Known then
                  Pw := C.Geo_Pw;
                  Pw_Up_Sd := C.Geo_Pw_Up_Sd;
                  if not Said_Known then
                     Said_Known := True;
                     Geo_Say ("此刻没有两只眼同时看见它 ⇒ 按上一次两眼交出来的位置 (" & Mm (Pw (0)) & "," & Mm (Pw (1)) & "," & Mm (Pw (2))
                              & ") 走(前提是它没动)");
                  end if;
               else
                  Event := S ("lost: two of my eyes have not seen it at the same moment, so I cannot tell where it is");
                  exit;
               end if;
               if Mok and then Length (Its_Name) > 0 then
                  --  记住它在哪:下一段看不清时凭这个走。两眼交点(偏差毫米级)比单眼挪出来的准得多(H35 2026-09-22 实测:交点 z=0.628,
                  --  之后手指朝下近处单眼挪出来的 z=0.745 把它盖掉了,下一段就按 12 cm 高的空中走)⇒ 这一段里有过交点就不让单眼盖
                  --  H47 2026-09-23 实测:单眼挪出来的一个坏位置 (0.43, −0.21, 0.90) 被记住,之后十几段全按它走、一步没走。
                  --  ⇒ 只记两眼交出来的(09-26 起也只有这一种估计)
                  if Mok then
                     C.Geo_Pw := Pw; C.Geo_Pw_Valid := True; C.Geo_Pw_Name := Its_Name; C.Geo_Pw_Met := True; C.Geo_Pw_Up_Sd := Pw_Up_Sd;
                  end if;
               end if;
            end;
            --  它躺在我碰过的面上 ⇒ 交点在面之下 / 比张口还高出面的,贴回面上再当目标(H53:交点在桌面之下 12 cm,"上方一个张口"就成了桌面之下,一路顶着桌子)
            Pw := Plane_Point (C, Pw, Up_Dir (C), Say => not Said_Clamp);
            if C.Touch_Valid and then not Said_Clamp then
               Said_Clamp := True;
            end if;
            Pc := Geom.To_Cam (G, Cur, Pw);
            --  它的位置是两眼交出来的 ⇒ 每只看全了它的眼都记一份它顶面的点(留最细的);哪儿夹得住由接触集从这上面算(PLAN 1.5),不再在像素上扫弦。
            --  腕眼里它常常顶着画面边(H48 的框就贴着 y=479)⇒ 那一眼的轮廓不完整、不记;不动的眼/另一只手的眼看全了它、我的手又没压在它上面 ⇒ 记
            if Length (Its_Name) > 0 then
               if Seen and then Whole then
                  Take_Silhouette (C, F, Cam, Arm, Its_Name, Pw, Pw_Up_Sd);
               elsif Seen and then not Said_Cut then
                  Said_Cut := True;
                  Geo_Say ("这只眼里它顶着画面边,轮廓不完整 ⇒ 这一眼不记它的顶面点,看别的眼");
               end if;
               for Cm in 0 .. C.Map.N_Cams - 1 loop
                  if Cm /= Cam and then Cm < Natural (C.Geo.Length) and then Cm < Natural (F.Cams.Length) then
                     declare
                        Bx2 : constant Integer := Boxed_By (C, Cm, Its_Name);
                     begin
                        if Bx2 >= 0 then
                           declare
                              B2 : constant Boxed_Thing := C.Boxed (Natural (Bx2));
                              Edge2 : constant Boolean := B2.X0 = 0 or else B2.Y0 = 0 or else B2.X1 + 1 >= F.Cams (Cm).W or else B2.Y1 + 1 >= F.Cams (Cm).H;
                           begin
                              if B2.Seen and then not Edge2 and then not Hand_Covers (C, F, Arm, Cm, B2) then
                                 Take_Silhouette (C, F, Cm, Arm, Its_Name, Pw, Pw_Up_Sd);
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
            end if;
            declare
               --  到它上方 ⇒ 它该落在"指尖合拢那一点"正下方一个张口处:把世界系的"往下一个张口"转进相机系,加到目标上。
               --  "上方" = 它躺的那个面的自由一侧:碰过的面按量到的法向,没碰过按重力的上(和转指尖那一条同一个约定)
               Nn_Up : constant Geom.V3 := Up_Dir (C);
               Down_C : constant Geom.V3 :=
                 (if Above then Geom.Ap (Geom.Tr (Geom.Cam_R (G, Cur)), [-G.Gap * Nn_Up (0), -G.Gap * Nn_Up (1), -G.Gap * Nn_Up (2)]) else [0.0, 0.0, 0.0]);
            begin
               D := [Pc (0) - Want (0) - Down_C (0), Pc (1) - Want (1) - Down_C (1), Pc (2) - Want (2) - Down_C (2)];
            end;
            Dist := Geom.Norm (D);
            C.Geo_Dist := Dist; C.Geo_Round := C.Round_N; C.Geo_At := Cur; C.Geo_At_Arm := Integer (Arm); C.Geo_At_Above := Above;
            Geo_Say ("它在相机前 " & Mm (-Pc (2)) & "(左右 " & Mm (Pc (0)) & " 上下 " & Mm (Pc (1)) & "),离指尖该到的那点还差 " & Mm (Dist) &
                     "(左右 " & Mm (D (0)) & " 上下 " & Mm (D (1)) & " 前后 " & Mm (D (2)) & ")");
            if -Pc (2) <= 0.0 then
               Event := S ("lost: my sightlines do not meet in front of me (the thing may have moved)");
               exit;
            end if;
            if Held_Back then
               declare
                  --  剩余误差搬到世界系,去掉指向面里的那一份;剩下的长度才是"还走得了的差距"
                  Rcw : constant Geom.M3 := Geom.Cam_R (G, Cur);
                  Dwf : Geom.V3 := Geom.Ap (Rcw, D);
                  Into : constant Long_Float := Dwf (0) * Wall (0) + Dwf (1) * Wall (1) + Dwf (2) * Wall (2);
               begin
                  if Into > 0.0 then
                     Dwf := [Dwf (0) - Into * Wall (0), Dwf (1) - Into * Wall (1), Dwf (2) - Into * Wall (2)];
                  end if;
                  D := Geom.Ap (Geom.Tr (Rcw), Dwf);
                  Dist := Geom.Norm (D);
                  Geo_Say ("被一个面顶着:沿着面还差 " & Mm (Dist) & "(往面里那一份 " & Mm (Long_Float'Max (0.0, Into)) & " 走不了,不算)");
               end;
            end if;
            if (Pressing or else Dist <= Tol) and then Until_Touch and then (not Above or else Pressing) and then not Held_Back
              and then (Geom.Norm (C.Geo_Dir) > 0.0 or else Pressing) and then Steps_Taken < Limit
            then
               if not Pressing then
                  Pressing := True; Press_Dir := C.Geo_Dir;
                  Geo_Say ("我估着到位了(差 " & Mm (Dist) & "),可你说的是碰到为止 ⇒ 沿来的方向接着往它身上走,到真被顶住");
               end if;
               declare
                  Ln : constant Long_Float := 4.0 * Geo_Base (C, Arm);     --  一个量距单位(刚被证明走得到的那一档)
                  Dw : constant Geom.V3 := [Press_Dir (0) * Ln, Press_Dir (1) * Ln, Press_Dir (2) * Ln];
               begin
                  Geo_Move (L, C, F, Arm, Dw, Mok);
                  Steps_Taken := Steps_Taken + 1;
                  declare
                     Now : constant Plug.Arm_Pose := F.EE (Arm);
                     Got : constant Long_Float := ((Now (0) - Cur (0)) * Dw (0) + (Now (1) - Cur (1)) * Dw (1) + (Now (2) - Cur (2)) * Dw (2)) / Ln;
                  begin
                     if Got + Got < Ln then
                        Event := S ("contact: I kept going toward it as you asked and something stopped my hand (I commanded " & Len (C, Ln)
                                    & " and went " & Len (C, Got) & "); by my own estimate the thing sits at where my fingers close");
                        C.Geo_At := Now; C.Geo_At_Arm := Integer (Arm); C.Geo_At_Above := False;   --  压到它身上了:接下来合手不用再下去
                        exit;
                     end if;
                  end;
               end;
            elsif Dist <= Tol then
               Event := S ((if Held_Back
                            then "contact: I am against a surface and as close as it lets me (the thing sits " & Len (C, Dist) & " from where my fingers close, measured along that surface)"
                            elsif Above
                            then "amount: arrived above it (it sits one hand-opening, " & Len (C, G.Gap) & ", straight below where my fingers close, within " & Len (C, Dist) & ")"
                            else "amount: arrived (the thing sits " & Len (C, Dist) & " from where my fingers close)"));
               --  🔴 到了它上方,顺手把【手指】指向它躺着的那个面(面 = 我碰过的那个面的法向;没碰过就按"上"的反向)。
               --  这不是抓剪刀的规矩,是"在它上方"对一副夹爪的含义:两指要能落到它两侧,指尖得朝着它来。
               --  H15/H19 2026-09-22 实测:手指斜着伸,合拢点到了它身上 3–5 mm 内,指尖却还悬在它上方 ⇒ 合空。
               --  转的是量过的方向(指尖方向 = 量过的指尖偏置),转多少由几何定,指尖位置边转边补;转不动就如实说。
               if Above and then G.Tip_Valid then
                  declare
                     Nn : constant Geom.V3 := Up_Dir (C);
                     Ev : Unbounded_String;
                     St : Natural;
                  begin
                     Geo_Turn (L, C, F, Arm, [-Nn (0), -Nn (1), -Nn (2)], Amt, Ev, St, Along => G.Tip);
                     Steps_Taken := Steps_Taken + St;
                     Retarget_Box (C, F, Cam, Arm, Name, Pw);   --  指尖朝下这一转很大:把它的窗投进转过的眼,下一段才认得出它
                     C.Fingers_Aimed := Index (Ev, "amount: arrived") > 0;
                     Append (Event, (if C.Fingers_Aimed then "; my fingers now point down at it"
                                     else "; I tried to point my fingers down at it: " & To_String (Ev)));
                  end;
               end if;
               --  🔴 "到它上方、碰到为止":到了上方,脑说的是碰到为止 ⇒ 顺着它躺的面的法向往它身上压,到真被顶住(和 touching 的"接着往它身上走"同一条)。
               --  H37 2026-09-22 实测:Qwen 十有八九写 `above X until touched`;上方永远碰不到,那一行就原地重跑十遍,然后它写 farther 走了。
               --  "until touched" 是脑明说的:没碰到就接着走。
               if Above and then Until_Touch and then not Held_Back and then Steps_Taken < Limit then
                  declare
                     Nn_P : constant Geom.V3 := Up_Dir (C);
                  begin
                     Pressing := True; Press_Dir := [-Nn_P (0), -Nn_P (1), -Nn_P (2)];
                     Geo_Say ("到了它上方,可你说的是碰到为止 ⇒ 顺着法向往它身上压,到真被顶住");
                  end;
               else
                  exit;
               end if;
            end if;
            if Steps_Taken >= Limit then
               Event := S ("steps: I took the steps you asked for (still " & Len (C, Dist) & " from where my fingers close)");
               exit;
            end if;
            if not Pressing then
            declare
               Frac : constant Long_Float := (if Dist > G.Gap then 0.6 else 1.0);   --  远时走六成再看一眼(比例,无量纲);近了一步到
               Want_Ln : constant Long_Float := Dist * Frac;
               Cut : constant Long_Float := (if Want_Ln > Step_Cap and then Want_Ln > 0.0 then Step_Cap / Want_Ln else 1.0);   --  超过脑给的那一档就按比例缩
               Step : constant Geom.V3 := [D (0) * Frac * Cut, D (1) * Frac * Cut, D (2) * Frac * Cut];
               Rc : constant Geom.M3 := Geom.Cam_R (G, Cur);
               Dw : constant Geom.V3 := Geom.Ap (Rc, Step);
               Ln : constant Long_Float := Geom.Norm (Dw);
            begin
               Geo_Move (L, C, F, Arm, Dw, Mok);
               Steps_Taken := Steps_Taken + 1;
               declare
                  Now : constant Plug.Arm_Pose := F.EE (Arm);
                  --  实到 = 沿【命令的方向】真走了多少(不是位移的长度:H10 2026-09-22 实测,命令往下 0.041 m 只下去 0.006 m,
                  --  手却横着滑了 0.025 m —— 按长度比就被当成"走成了",接着在一个撞着桌面的姿势上继续算、算飞)
                  Got : constant Long_Float :=
                    (if Ln > 0.0 then ((Now (0) - Cur (0)) * Dw (0) + (Now (1) - Cur (1)) * Dw (1) + (Now (2) - Cur (2)) * Dw (2)) / Ln else 0.0);
               begin
                  if Got + Got < Ln and then not Held_Back then
                     --  第一次被顶住:记下顶住我的方向 = 命令的位移减去实到的位移(量出来的),之后只走沿着面的那一部分
                     declare
                        Miss : constant Geom.V3 := [Dw (0) - (Now (0) - Cur (0)), Dw (1) - (Now (1) - Cur (1)), Dw (2) - (Now (2) - Cur (2))];
                        Ml : constant Long_Float := Geom.Norm (Miss);
                     begin
                        if Ml > 0.0 then
                           Wall := [Miss (0) / Ml, Miss (1) / Ml, Miss (2) / Ml];
                           Held_Back := True;
                           --  碰过的点进地图:面上的一点 = 此刻指尖的世界位置,法向 = 顶住我的方向反过来。
                           --  🔴 只有【朝下】被顶住的才是它躺的面(东西靠着它抵住重力);顶住我的方向横着的,是墙、或是我自己够不着了
                           --  (H31 2026-09-22 实测:右臂横跨整张桌去够,在 (-0.26,-0.97,0) 方向被自己的关节顶住,我把它记成了"面",
                           --  于是"上方"和"指尖朝下"都朝了横向,后面全乱)。朝下 = 竖直分量比水平分量大(纯比较);"下"= 位姿系 -z,同抬手那条约定。
                           if abs (Wall (2)) > Sqrt (Wall (0) ** 2 + Wall (1) ** 2) then
                              declare
                                 Tw : constant Geom.V3 := Geom.Ap (Geom.Cam_R (G, Now), G.Tip);
                              begin
                                 Note_Support (C, [Now (0) + Tw (0), Now (1) + Tw (1), Now (2) + Tw (2)], [-Wall (0), -Wall (1), -Wall (2)],
                                               "这一步要 " & Mm (Ln) & " 只到 " & Mm (Got) & ",方向 ("
                                               & Codec.Fmt (Wall (0), 2) & "," & Codec.Fmt (Wall (1), 2) & "," & Codec.Fmt (Wall (2), 2) & ")");
                              end;
                           else
                              C.Walls.Append (Wall_Mark'(Arm => Arm, P => Tip_World (C, Arm, Now), W => Wall));
                              Geo_Say ("这一步要 " & Mm (Ln) & " 只到 " & Mm (Got) & " ⇒ 被横着顶住了,方向 ("
                                       & Codec.Fmt (Wall (0), 2) & "," & Codec.Fmt (Wall (1), 2) & "," & Codec.Fmt (Wall (2), 2)
                                       & "):不是它躺的面(是墙,或我自己的关节到头了),不记成面;记下「这条臂到这儿为止」,沿着它接着走");
                           end if;
                        end if;
                     end;
                  elsif Got + Got < Ln then              --  沿命令方向实到不到要的一半(纯数学的一半)= 命令了,身体没走
                     Event := S ("resist: I commanded a step of " & Len (C, Ln) & " toward it and my hand only went " & Len (C, Got)
                                 & " (" & Len (C, Dist) & " from where my fingers close) - either something is holding my hand there, "
                                 & "or that step was more than I can do in one command from this pose");
                     exit;
                  end if;
               end;
               C.Geo_Came := C.Geo_Came + Ln;
               if Ln > 0.0 then
                  C.Geo_Dir := [Dw (0) / Ln, Dw (1) / Ln, Dw (2) / Ln];
               end if;
               --  🔴 迈了一大步之后,它在我这只眼里的位置和大小都变了(H5 2026-09-22 实测:走到差 0.071 m 时跟丢 ——
               --  点过名的东西是"在上一帧量到它的地方原样再量",一步 14 cm 之后它早不在那儿了)。
               --  可我【知道】它该在哪:它的位置是我刚用两条视线交出来的(Pw),我挪了多少是位姿读数说的 ⇒
               --  把它投到新位姿的画面里,就是它这一帧该出现的像素;离我近了几成,它就大了几成。先把重量的窗挪过去、放大,再量。
               declare
                  Have_Slot : constant Boolean := Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam);
               begin
               if Length (Name) > 0 or else Have_Slot then
                  declare
                     Bx : constant Integer :=
                       (if Length (Name) > 0 then Boxed_By (C, Cam, Name)
                        else Boxed_Index (C, Cam, World.Get (C.Wld, Cam, Natural (Slot)).R.Cu, World.Get (C.Wld, Cam, Natural (Slot)).R.Cv));
                     Pu, Pv : Long_Float;
                     Front : Boolean;
                     Z_Was : constant Long_Float := -Pc (2);
                     Z_Now : constant Long_Float := -Geom.To_Cam (G, F.EE (Arm), Pw) (2);
                     Cw : constant Natural := F.Cams (Cam).W;
                     Ch : constant Natural := F.Cams (Cam).H;
                  begin
                     Geom.Project (G, F.EE (Arm), Pw, Pu, Pv, Front);
                     if Bx >= 0 and then Front and then Z_Was > 0.0 and then Z_Now > 0.0
                       and then Pu >= 0.0 and then Pv >= 0.0 and then Pu < Long_Float (Cw) and then Pv < Long_Float (Ch)
                     then
                        declare
                           B : Boxed_Thing := C.Boxed (Natural (Bx));
                           Grow : constant Long_Float := Z_Was / Z_Now;
                           Hw : constant Long_Float := 0.5 * (Long_Float (B.X1 - B.X0) * Grow);   --  半宽(纯数学的一半)
                           Hh : constant Long_Float := 0.5 * (Long_Float (B.Y1 - B.Y0) * Grow);
                           function Px (V2 : Long_Float; Span : Natural) return Natural is
                             (Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Span - 1), V2))));
                        begin
                           if B.Pu_On >= 0.0 then   --  它身上那一点跟着框平移、按远近缩放
                              B.Pu_On := Pu + Grow * (B.Pu_On - 0.5 * Long_Float (B.X0 + B.X1));
                              B.Pv_On := Pv + Grow * (B.Pv_On - 0.5 * Long_Float (B.Y0 + B.Y1));
                           end if;
                           B.X0 := Px (Pu - Hw, Cw); B.X1 := Px (Pu + Hw, Cw);
                           B.Y0 := Px (Pv - Hh, Ch); B.Y1 := Px (Pv + Hh, Ch);
                           B.Count := Natural (Long_Float (B.Count) * Grow * Grow);   --  它该有的像素数跟着远近变(面积 = 线尺寸的平方,纯数学)
                           if B.X1 > B.X0 and then B.Y1 > B.Y0 then
                              C.Boxed.Replace_Element (Natural (Bx), B);
                              --  槽里记的那一块也挪到预测处,好让这一帧量到的新块对得上同一个槽(World.Observe 按形心就近认槽)
                              if Have_Slot then
                                 World.Shift_Slot (C.Wld, Cam, Natural (Slot), Pu / Long_Float (Cw), Pv / Long_Float (Ch), Grow);
                              end if;
                              Geo_Say ("它该出现在 (" & Codec.Fmt (Pu, 1) & "," & Codec.Fmt (Pv, 1) & "),大了 " & Codec.Fmt (Grow, 2) & " 倍 ⇒ 到那儿去量");
                           end if;
                        end;
                     end if;
                  end;
               end if;
               end;
               Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
               Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
               --  手指已经指着它躺的面时,最后贴上去这一段不再为了看它而转手(转了指尖就不朝下了);看不全就按位姿读数走
               if Seen and then Edge and then not Pressing and then not (C.Fingers_Aimed and then not Above) then
                  declare
                     Ev : Unbounded_String;
                     St : Natural;
                     Ray_Ok : Boolean;
                     Want : constant Geom.V3 := Geom.Ray (G, F.EE (Arm), U, V, Ray_Ok);   --  先算好再传(见上)
                  begin
                     if Ray_Ok then
                        Geo_Turn (L, C, F, Arm, Want, Amt, Ev, St);
                     else
                        Ev := S ("its pixel is outside what my lens model covers, so I cannot tell which way to turn");
                        St := 0;
                     end if;
                     Steps_Taken := Steps_Taken + St;
                     Geo_Say ("它被画面边切着 ⇒ 转眼看着它(" & To_String (Ev) & ")");
                     Retarget_Box (C, F, Cam, Arm, Name, Pw);   --  它在哪这一步刚算过(Pw)⇒ 投进转过的眼
                     Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
                     Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
                  end;
               end if;
               if (Known or else C.Geo_Pw_Valid) and then Seen and then not Whole then
                  if not Said_Blind then
                     Said_Blind := True;
                     Geo_Say ("它有一截出了画面/被挡住,这一眼不可信 ⇒ 不再更新它的位置;凭上一次两眼交出来的位置走完");
                  end if;
               elsif not Seen then
                  --  最后一步它进了指缝、被手指挡住也正常:上一眼已经在两倍容差内(倍数,无量纲)
                  if Dist <= 2.0 * Tol then
                     Event := S ("amount: arrived (I lost sight of it on the last step; it was " & Len (C, Dist) & " from where my fingers close)");
                     exit;
                  elsif Known or else C.Geo_Pw_Valid then
                     --  看不见了,可它在哪我这一段(或上一段)量过 ⇒ 凭记住的位置走完
                     Known := True;
                     if not Said_Known then
                        Said_Known := True;
                        Geo_Say ("这一步之后看不见它了 ⇒ 按我量到的位置走完(前提是它没动)");
                     end if;
                  else
                     Event := S ("lost: I lost sight of it after that step (it was " & Len (C, Dist) & " away)");
                     exit;
                  end if;
               end if;
            end;
            end if;   --  not Pressing
         end;
      end loop;
      Beats := Beats_Since (L, Beats0);
   end Geo_Approach;

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
                       Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
      Beats0 : constant Natural := Plug.Steps (L);
      Ln : constant Long_Float := Stride_Of (C, Arm) * Amt;   --  一步 = 量出来的最大一档 × 脑的档位
      N : constant Natural := (if Step_Limit > 0 then Step_Limit else 1);
      Mok : Boolean;
      Went : Long_Float := 0.0;
   begin
      Steps_Taken := 0; Beats := 0;
      if Geom.Norm (C.Geo_Dir) <= 0.0 or else Ln <= 0.0 then
         Event := S ("refused: I have not walked toward it yet, so I do not know which way is away from it");
         return;
      end if;
      for K in 1 .. N loop
         declare
            Cur : constant Plug.Arm_Pose := F.EE (Arm);
            Dw : constant Geom.V3 := [-C.Geo_Dir (0) * Ln, -C.Geo_Dir (1) * Ln, -C.Geo_Dir (2) * Ln];
         begin
            Geo_Move (L, C, F, Arm, Dw, Mok);
            Steps_Taken := Steps_Taken + 1;
            declare
               Now : constant Plug.Arm_Pose := F.EE (Arm);
               Got : constant Long_Float := ((Now (0) - Cur (0)) * Dw (0) + (Now (1) - Cur (1)) * Dw (1) + (Now (2) - Cur (2)) * Dw (2)) / Ln;
            begin
               Went := Went + Got;
               C.Geo_Dist := C.Geo_Dist + Got; C.Geo_At := Now;   --  离它远了这么多;刚算的"笼住"距离跟着变
               if Got + Got < Ln then   --  实到不到要的一半(纯数学的一半)= 被顶住了
                  Event := S ("resist: I commanded a step of " & Len (C, Ln) & " away from it and my hand only went " & Len (C, Got));
                  Beats := Beats_Since (L, Beats0);
                  return;
               end if;
            end;
         end;
      end loop;
      Event := S ("amount: arrived (I moved " & Len (C, Went) & " away from it, along the line I had come in on)");
      Beats := Beats_Since (L, Beats0);
   end Geo_Away;

   --  ── 一轮 ──
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

   --  V1 口径"头顶眼按指尖算的残差"(2026-09-26):开机各停里不动的眼给这只手做的合空标记(每一瓣的尖 C.Lobe_Obs、各瓣的中点 C.Fixed_Obs),
   --  和"那一停的位姿 + 碰出来的指尖"投进它眼里的那一点比。只报数、不改任何量:两边都是量的(标记是分割出来的尖,指尖是碰出来的)
   procedure Head_Tip_Check (C : Context; A, Hc : Natural; Tips : Geom.V3_Vectors.Vector) is
      Wc : constant Natural := C.Map.World_Cam;
      package Sorting is new F64_Vectors.Generic_Sorting;
      procedure Report (Name : String; E : in out Floats) is
         Within : Natural := 0;
         V1_Line : constant Long_Float := 2.0;   --  V1 验收线 2 px(PLAN.md §1 协议里的判据;只数一数,不当门)
      begin
         if E.Is_Empty then
            Geo_Say ("  对账(头顶眼按指尖,V1 口径):" & Name & " 一笔都没有");
            return;
         end if;
         Sorting.Sort (E);
         for X of E loop
            if X <= V1_Line then
               Within := Within + 1;
            end if;
         end loop;
         Geo_Say ("  对账(头顶眼按指尖,V1 口径):它开机时标的" & Name & " " & Codec.Img (Natural (E.Length)) & " 笔,离碰出来的指尖投进它眼里的那一点 中位 "
                  & Codec.Fmt (E (Natural (E.Length) / 2), 2) & " px、最大 " & Codec.Fmt (E (Natural (E.Length) - 1), 1) & " px,2 px 内 " & Codec.Img (Within) & " 笔");
      end Report;
   begin
      if Wc >= Natural (C.Geo.Length) or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else Hc >= Natural (C.Geo.Length) then
         return;
      end if;
      declare
         Gw : constant Geom.Cam_Geo := C.Geo (Wc);
         G : constant Geom.Cam_Geo := C.Geo (Hc);
         El, Em, Eo : Floats;
         function Tip_At (P : Plug.Arm_Pose; K : Natural) return Geom.V3 is
            O : constant Geom.V3 := Geom.Cam_Pos (G, P);
            T : constant Geom.V3 := Geom.Ap (Geom.Cam_R (G, P), Tips (K));
         begin
            return [O (0) + T (0), O (1) + T (1), O (2) + T (2)];
         end Tip_At;
      begin
         for Ob of C.Lobe_Obs loop
            if Ob.Pt = A then
               declare
                  Best : Long_Float := Long_Float'Last;
               begin
                  for K in 0 .. Natural (Tips.Length) - 1 loop
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        Geom.Project_Fixed (Gw, Tip_At (Ob.Pose, K), U, V, Front);
                        if Front then
                           Best := Long_Float'Min (Best, Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2));
                        end if;
                     end;
                  end loop;
                  if Best < Long_Float'Last then
                     El.Append (Best);
                  end if;
               end;
            end if;
         end loop;
         for I in 0 .. Natural (C.Fixed_Obs.Length) - 1 loop
            declare
               Ob : constant Geom.Obs_Pt := C.Fixed_Obs (I);
            begin
               if Ob.Pt = A then
                  declare
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Geom.Project_Fixed (Gw, Tip_World (C, A, Ob.Pose), U, V, Front);
                     if Front then
                        Em.Append (Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2));
                     end if;
                  end;
                  --  碰出来的每一瓣指尖投进它眼里,离那一笔里它看见的手指像素最近多远(落在手指上 = 0):不比分割出来的"尖"那一点(远处小夹爪的尖一会儿一个样)
                  if I < Natural (C.Mark_Px.Length) and then not C.Mark_Px (I).Is_Empty then
                     for K in 0 .. Natural (Tips.Length) - 1 loop
                        declare
                           U, V : Long_Float;
                           Front : Boolean;
                           Best : Long_Float := Long_Float'Last;
                        begin
                           Geom.Project_Fixed (Gw, Tip_At (Ob.Pose, K), U, V, Front);
                           if Front then
                              for P of C.Mark_Px (I) loop
                                 Best := Long_Float'Min (Best, Sqrt ((Long_Float (P.U) - U) ** 2 + (Long_Float (P.V) - V) ** 2));
                              end loop;
                              if Best < Long_Float'Last then
                                 Eo.Append (Best);
                              end if;
                           end if;
                        end;
                     end loop;
                  end if;
               end if;
            end;
         end loop;
         Report ("每一瓣的尖", El);
         Report ("各瓣的中点", Em);
         Report ("手指像素(碰出来的每一瓣指尖离它最近多远)", Eo);
      end;
   end Head_Tip_Check;

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

   procedure Board_Free_Spots (C : Context; Lp : Geom.V3_Vectors.Vector; Tb : Floats; R : Long_Float; Deltas : out Geom.V3_Vectors.Vector) is
      N : constant Geom.V3 := C.Board_N;
      Nb : constant Natural := Natural (C.Board.Length);
      Nt : constant Natural := Nb + Natural (C.Seen_Above.Length);   --  板点在前,压之前看见的高出面的点在后(只挡,不当量过的桌面)
      Fresh : constant Boolean := Natural (C.Board_Seen.Length) = Nb;   --  重找过(和板一一对应)
      On, Above, Tried : Bools;
      Hgt : Floats;
      Nn : Floats;                   --  每个量过的桌面上的板点离最近一个同类的多远(面内;别的点 = 0)
      Pp : Geom.V3_Vectors.Vector;   --  板点投到面上
      E1, E2 : Geom.V3 := [others => 0.0];   --  面内两根正交的轴(排方位用)
      package Sorting is new F64_Vectors.Generic_Sorting;
      function Gap (A, B : Geom.V3) return Long_Float is (Geom.Norm ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]));
      --  落点 A0 那一圈(半径 R)整个在量过的桌面里:圈里每一处(按半个板点间距铺的格,采样,无量纲),离它两个板点间距以内那些量过的
      --  桌面上的板点把它围住 —— 按方位排开,最大的空档 < 180°(在那片里面);在那片的边上、边外、里面一块没点的洞里 = 空档 ≥ 180°。
      --  板点间距 = 离落点最近的 3 个(次数)量过的桌面上的板点各自离最近一个的中位(板自己量的;圈比间距小时圈里可能一个点都没有);
      --  两个间距 = 缺一个点照样围得住(倍数,无量纲);量过的桌面上不到 3 个点 ⇒ 量不出间距,不算
      function Inside (A0 : Geom.V3) return Boolean is
         Kn : constant := 3;
         Near : array (0 .. Kn - 1) of Long_Float := [others => Long_Float'Last];   --  最近几个的距离(从近到远)
         Near_S : array (0 .. Kn - 1) of Long_Float := [others => 0.0];            --  它们各自离最近一个同类的距离
         S : Long_Float;
         Pool : Geom.Nat_Vectors.Vector;   --  离落点 R + 两个间距以内的那些量过的桌面上的板点
      begin
         for I in 0 .. Nb - 1 loop
            if On (I) and then Nn (I) > 0.0 then
               declare
                  D : constant Long_Float := Gap (Pp (I), A0);
                  K : Integer := Kn - 1;
               begin
                  if D < Near (Kn - 1) then
                     while K > 0 and then Near (K - 1) > D loop
                        Near (K) := Near (K - 1); Near_S (K) := Near_S (K - 1);
                        K := K - 1;
                     end loop;
                     Near (K) := D; Near_S (K) := Nn (I);
                  end if;
               end;
            end if;
         end loop;
         if Near (Kn - 1) = Long_Float'Last then
            return False;
         end if;
         --  三个的中位:排一下取中间那个
         declare
            Ns : Floats;
         begin
            for V of Near_S loop
               Ns.Append (V);
            end loop;
            Sorting.Sort (Ns);
            S := Ns (Kn / 2);
         end;
         for I in 0 .. Nb - 1 loop
            if On (I) and then Gap (Pp (I), A0) <= R + 2.0 * S then
               Pool.Append (I);
            end if;
         end loop;
         declare
            Kmax : constant := 64;   --  每条半径上最多铺这么多格(算力的上限,次数;板里两点几乎重合时间距会很小)
            Pitch : constant Long_Float := Long_Float'Max (0.5 * S, R / Long_Float (Kmax));
            M : constant Integer := Integer (Long_Float'Floor (R / Pitch));
         begin
            for Ia in -M .. M loop
               for Ib in -M .. M loop
                  declare
                     Xa : constant Long_Float := Long_Float (Ia) * Pitch;
                     Xb : constant Long_Float := Long_Float (Ib) * Pitch;
                     X : constant Geom.V3 := [A0 (0) + Xa * E1 (0) + Xb * E2 (0), A0 (1) + Xa * E1 (1) + Xb * E2 (1), A0 (2) + Xa * E1 (2) + Xb * E2 (2)];
                     Angs : Floats;
                     Widest : Long_Float := 0.0;
                  begin
                     if Xa * Xa + Xb * Xb <= R * R then
                        for I of Pool loop
                           declare
                              Q : constant Geom.V3 := [Pp (I) (0) - X (0), Pp (I) (1) - X (1), Pp (I) (2) - X (2)];
                              Qa : constant Long_Float := Q (0) * E1 (0) + Q (1) * E1 (1) + Q (2) * E1 (2);
                              Qb : constant Long_Float := Q (0) * E2 (0) + Q (1) * E2 (1) + Q (2) * E2 (2);
                              D2 : constant Long_Float := Qa * Qa + Qb * Qb;
                           begin
                              if D2 > 0.0 and then D2 <= 4.0 * S * S then
                                 Angs.Append (Arctan (Qb, Qa));
                              end if;
                           end;
                        end loop;
                        if Angs.Is_Empty then
                           return False;
                        end if;
                        Sorting.Sort (Angs);
                        for K in 1 .. Natural (Angs.Length) - 1 loop
                           Widest := Long_Float'Max (Widest, Angs (K) - Angs (K - 1));
                        end loop;
                        Widest := Long_Float'Max (Widest, Angs (0) + 2.0 * Ada.Numerics.Pi - Angs (Natural (Angs.Length) - 1));
                        if Widest >= Ada.Numerics.Pi - 1.0e-9 then   --  正好 180°(点在两个板点连线上 = 那片的边)不算里面:数值上不许靠舍入定
                           return False;
                        end if;
                     end if;
                  end;
               end loop;
            end loop;
         end;
         return True;
      end Inside;
      function Clear (Dl : Geom.V3) return Boolean is
         A0 : constant Geom.V3 := [Lp (0) (0) + Dl (0), Lp (0) (1) + Dl (1), Lp (0) (2) + Dl (2)];
      begin
         for I in 0 .. Nt - 1 loop
            if Above (I) then
               if Geom.Norm ([Pp (I) (0) - A0 (0), Pp (I) (1) - A0 (1), Pp (I) (2) - A0 (2)]) <= R then
                  return False;
               end if;
               for J in 1 .. Natural (Lp.Length) - 1 loop
                  declare
                     Aj : constant Geom.V3 := [Lp (J) (0) + Dl (0) - A0 (0), Lp (J) (1) + Dl (1) - A0 (1), Lp (J) (2) + Dl (2) - A0 (2)];
                     Ln : constant Long_Float := Geom.Norm (Aj);
                     Q : constant Geom.V3 := [Pp (I) (0) - A0 (0), Pp (I) (1) - A0 (1), Pp (I) (2) - A0 (2)];
                     Rho : constant Long_Float := (if Ln > 0.0 then Long_Float'Max (0.0, Long_Float'Min (Ln, (Q (0) * Aj (0) + Q (1) * Aj (1) + Q (2) * Aj (2)) / Ln)) else 0.0);
                     Side : constant Long_Float := (if Ln > 0.0 then Geom.Norm ([Q (0) - Rho * Aj (0) / Ln, Q (1) - Rho * Aj (1) / Ln, Q (2) - Rho * Aj (2) / Ln]) else Geom.Norm (Q));
                  begin
                     if Side <= R and then Hgt (I) >= Rho * Tb (J) then
                        return False;
                     end if;
                  end;
               end loop;
            end if;
         end loop;
         return Inside (A0);
      end Clear;
   begin
      Deltas := Geom.V3_Vectors.Empty_Vector;
      if Lp.Is_Empty or else Nb = 0 or else Natural (Tb.Length) < Natural (Lp.Length) then
         return;
      end if;
      for I in 0 .. Nt - 1 loop
         declare
            S : constant Geom.Scene_Pt := (if I < Nb then C.Board (I) else C.Seen_Above (I - Nb));
            H : constant Long_Float := (S.Pw (0) - C.Board_Pt (0)) * N (0) + (S.Pw (1) - C.Board_Pt (1)) * N (1) + (S.Pw (2) - C.Board_Pt (2)) * N (2);
            Cn : constant Geom.V3 := Geom.Ap (S.Cov, N);
            Tol : constant Long_Float := Plane_Tol (C, Cn (0) * N (0) + Cn (1) * N (1) + Cn (2) * N (2));
         begin
            On.Append (I < Nb and then abs H <= Tol and then (not Fresh or else C.Board_Seen (I)));
            Above.Append (H > Tol);
            Hgt.Append (H);
            Tried.Append (False);
            Pp.Append (Geom.V3'[S.Pw (0) - H * N (0), S.Pw (1) - H * N (1), S.Pw (2) - H * N (2)]);
         end;
      end loop;
      --  面内两根轴:法向叉上和它最不平行的那根坐标轴
      declare
         Ax : constant Geom.V3 := (if abs N (0) <= abs N (1) and then abs N (0) <= abs N (2) then [1.0, 0.0, 0.0]
                                   elsif abs N (1) <= abs N (2) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
         Cx : constant Geom.V3 := [N (1) * Ax (2) - N (2) * Ax (1), N (2) * Ax (0) - N (0) * Ax (2), N (0) * Ax (1) - N (1) * Ax (0)];
         Cl : constant Long_Float := Geom.Norm (Cx);
      begin
         if Cl <= 0.0 then
            return;
         end if;
         E1 := [Cx (0) / Cl, Cx (1) / Cl, Cx (2) / Cl];
         E2 := [N (1) * E1 (2) - N (2) * E1 (1), N (2) * E1 (0) - N (0) * E1 (2), N (0) * E1 (1) - N (1) * E1 (0)];
      end;
      for I in 0 .. Nb - 1 loop
         declare
            Best : Long_Float := 0.0;
         begin
            if On (I) then
               Best := Long_Float'Last;
               for J in 0 .. Nb - 1 loop
                  if J /= I and then On (J) then
                     Best := Long_Float'Min (Best, Gap (Pp (I), Pp (J)));
                  end if;
               end loop;
               if Best = Long_Float'Last then
                  Best := 0.0;
               end if;
            end if;
            Nn.Append (Best);
         end;
      end loop;
      if Clear ([0.0, 0.0, 0.0]) then
         Deltas.Append (Geom.V3'[0.0, 0.0, 0.0]);
      end if;
      loop
         declare
            Best : Integer := -1;
            Bd : Long_Float := Long_Float'Last;
         begin
            for I in 0 .. Nb - 1 loop
               if On (I) and then not Tried (I) then
                  declare
                     D : constant Long_Float := Geom.Norm ([Pp (I) (0) - Lp (0) (0), Pp (I) (1) - Lp (0) (1), Pp (I) (2) - Lp (0) (2)]);
                  begin
                     if D < Bd then
                        Bd := D; Best := I;
                     end if;
                  end;
               end if;
            end loop;
            exit when Best < 0;
            Tried.Replace_Element (Natural (Best), True);
            declare
               Dl : constant Geom.V3 := [Pp (Natural (Best)) (0) - Lp (0) (0), Pp (Natural (Best)) (1) - Lp (0) (1), Pp (Natural (Best)) (2) - Lp (0) (2)];
            begin
               if Clear (Dl) then
                  Deltas.Append (Dl);
               end if;
            end;
         end;
      end loop;
   end Board_Free_Spots;

   function Look_Points (C : Context; G : Geom.Cam_Geo; P0 : Plug.Arm_Pose; W, H : Natural;
                         Spot : Geom.V3; Far_Ends : Geom.V3_Vectors.Vector; R, Step_Px : Long_Float) return Instrument.Match_Vectors.Vector is
      Q : Instrument.Match_Vectors.Vector;
      Nb : constant Geom.V3 := C.Board_N;
      function Dot (P, Q : Geom.V3) return Long_Float is (P (0) * Q (0) + P (1) * Q (1) + P (2) * Q (2));
      function Free_Px (U, V : Long_Float) return Boolean is
        (U >= 0.0 and then V >= 0.0 and then U < Long_Float (W) and then V < Long_Float (H));
      O0 : constant Geom.V3 := Geom.Cam_Pos (G, P0);
      H0 : constant Long_Float := Dot ([O0 (0) - C.Board_Pt (0), O0 (1) - C.Board_Pt (1), O0 (2) - C.Board_Pt (2)], Nb);
      Sw : constant Long_Float := (if G.F > 0.0 then Long_Float'Max (1.0, Step_Px) * Long_Float'Max (0.0, H0) / G.F else 0.0);   --  铺点的间距(世界单位)
      procedure Ask (X : Geom.V3) is
         U, V : Long_Float;
         Front : Boolean;
      begin
         Geom.Project (G, P0, X, U, V, Front);
         if Front and then Free_Px (U, V) then
            Q.Append (Instrument.Match_Pt'(U => U, V => V, others => <>));
         end if;
      end Ask;
      --  从 Pa 到 Pb、两边各 R 的那一片(Pa = Pb ⇒ 以它为心、半径 R 的一圈)
      procedure Strip (Pa, Pb : Geom.V3) is
         Dv : constant Geom.V3 := [Pb (0) - Pa (0), Pb (1) - Pa (1), Pb (2) - Pa (2)];
         Hz : constant Long_Float := Dot (Dv, Nb);
         Hv : constant Geom.V3 := [Dv (0) - Hz * Nb (0), Dv (1) - Hz * Nb (1), Dv (2) - Hz * Nb (2)];
         Ln_S : constant Long_Float := Geom.Norm (Hv);
         --  面内两根轴:沿带子;没有长度 ⇒ 法向叉上和它最不平行的那根坐标轴
         Ax : constant Geom.V3 := (if abs Nb (0) <= abs Nb (1) and then abs Nb (0) <= abs Nb (2) then [1.0, 0.0, 0.0]
                                   elsif abs Nb (1) <= abs Nb (2) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
         Cx : constant Geom.V3 := [Nb (1) * Ax (2) - Nb (2) * Ax (1), Nb (2) * Ax (0) - Nb (0) * Ax (2), Nb (0) * Ax (1) - Nb (1) * Ax (0)];
         T : constant Geom.V3 := (if Ln_S > 0.0 then [Hv (0) / Ln_S, Hv (1) / Ln_S, Hv (2) / Ln_S]
                                  else [Cx (0) / Geom.Norm (Cx), Cx (1) / Geom.Norm (Cx), Cx (2) / Geom.Norm (Cx)]);
         Wv : constant Geom.V3 := [Nb (1) * T (2) - Nb (2) * T (1), Nb (2) * T (0) - Nb (0) * T (2), Nb (0) * T (1) - Nb (1) * T (0)];
         Na : constant Natural := Natural (Long_Float'Ceiling (Ln_S / Sw));
         Nr : constant Natural := Natural (Long_Float'Ceiling (R / Sw));
      begin
         for I in -Integer (Nr) .. Integer (Na + Nr) loop
            for J in -Integer (Nr) .. Integer (Nr) loop
               declare
                  Al : constant Long_Float := Long_Float (I) * Sw;   --  沿带子离 Pa 多远
                  Ac : constant Long_Float := Long_Float (J) * Sw;   --  离带子中线多远
                  Along : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Ln_S, Al));   --  离线段 Pa–Pb 多远(两头按圆)
               begin
                  if (Al - Along) ** 2 + Ac ** 2 <= R * R then
                     Ask ([Pa (0) + Al * T (0) + Ac * Wv (0), Pa (1) + Al * T (1) + Ac * Wv (1), Pa (2) + Al * T (2) + Ac * Wv (2)]);
                  end if;
               end;
            end loop;
         end loop;
      end Strip;
   begin
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            if Free_Px (Kinem.Grid_U (Gxx, W), Kinem.Grid_V (Gyy, H)) then
               Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, W), V => Kinem.Grid_V (Gyy, H), others => <>));
            end if;
         end loop;
      end loop;
      if Sw > 0.0 and then R > 0.0 then
         Strip (Spot, Spot);
         for Pb of Far_Ends loop
            Strip (Spot, Pb);
         end loop;
      end if;
      return Q;
   end Look_Points;

   --  压之前先看底下(09-30 V1B70 / V1B73):一对立体像里比面高出的点。配上 = 配到的落在画面里、配回来离问的那一点 Geom.Trip_Px 以内
   --  (同核对不动的眼、重找板点);配点噪声(每轴)= 这一批往返差的中位 ÷ 瑞利分布的中位(同板的);两条视线交出一点(Geom.Meet),
   --  按两个位姿投回两帧,四个像素残差合起来超过 Z 倍配点噪声 = 两条视线对不上(动着的东西、配错的)⇒ 不要。
   --  交成的点离面高出 Plane_Tol(同挑空地的"高出面";沿法向的方差按 Geom.Meet_Cov,每条视线的角度噪声 = 配点噪声 ÷ 焦距)⇒ Above。
   --  挨着眼平移方向的那一片视差小、远近定不住:它的方差大,门跟着宽,判不成高出面(不猜)
   procedure Seen_Above_Of (C : Context; G : Geom.Cam_Geo; P0, P1 : Plug.Arm_Pose; W, H : Natural; Qu, Qv, Mu, Mv, Bu, Bv : Floats;
                            Above : out Geom.Scene_Pt_Vectors.Vector; Matched, Tri : out Natural; Sig : out Long_Float) is
      package Sorting is new F64_Vectors.Generic_Sorting;
      N : constant Geom.V3 := C.Board_N;
      O0 : constant Geom.V3 := Geom.Cam_Pos (G, P0);
      O1 : constant Geom.V3 := Geom.Cam_Pos (G, P1);
      Nq : constant Natural := Natural'Min (Natural'Min (Natural (Qu.Length), Natural (Qv.Length)),
                                            Natural'Min (Natural'Min (Natural (Mu.Length), Natural (Mv.Length)), Natural'Min (Natural (Bu.Length), Natural (Bv.Length))));
      Ok_M : Bools;
      Es : Floats;
   begin
      Above := Geom.Scene_Pt_Vectors.Empty_Vector; Matched := 0; Tri := 0; Sig := 0.0;
      for I in 0 .. Nq - 1 loop
         declare
            Ok : constant Boolean := Mu (I) >= 0.0 and then Mv (I) >= 0.0 and then Mu (I) < Long_Float (W) and then Mv (I) < Long_Float (H)
              and then Geom.Round_Trip_Ok (Qu (I), Qv (I), Bu (I), Bv (I));
         begin
            Ok_M.Append (Ok);
            if Ok then
               Matched := Matched + 1;
               Es.Append (Sqrt ((Bu (I) - Qu (I)) ** 2 + (Bv (I) - Qv (I)) ** 2));
            end if;
         end;
      end loop;
      if Es.Is_Empty or else not (G.F > 0.0) then
         return;
      end if;
      Sorting.Sort (Es);
      Sig := Es (Natural (Es.Length) / 2) / Stats.Rayleigh_Median;
      if not (Sig > 0.0) then
         return;   --  往返分毫不差:量不出配点噪声 ⇒ 远近的不确定度也量不出,不判
      end if;
      for I in 0 .. Nq - 1 loop
         if Ok_M (I) then
            declare
               Ok0, Ok1, Okm, Front0, Front1, Okc : Boolean;
               D0 : constant Geom.V3 := Geom.Ray (G, P0, Qu (I), Qv (I), Ok0);
               D1 : constant Geom.V3 := Geom.Ray (G, P1, Mu (I), Mv (I), Ok1);
               Rays : Geom.Sight_Vectors.Vector;
               Sds : Floats;
               Spread : Long_Float;
               X : Geom.V3;
               U0, V0, U1, V1 : Long_Float;
               Cv : Geom.M3;
            begin
               if Ok0 and then Ok1 then
                  Rays.Append (Geom.Sight'(O => O0, D => D0));
                  Rays.Append (Geom.Sight'(O => O1, D => D1));
                  X := Geom.Meet (Rays, Okm, Spread);
                  if Okm then
                     Geom.Project (G, P0, X, U0, V0, Front0);
                     Geom.Project (G, P1, X, U1, V1, Front1);
                     if Front0 and then Front1
                       and then Sqrt ((U0 - Qu (I)) ** 2 + (V0 - Qv (I)) ** 2 + (U1 - Mu (I)) ** 2 + (V1 - Mv (I)) ** 2) <= Stats.Z * Sig
                     then
                        Tri := Tri + 1;
                        Sds.Append (Sig / G.F, Count => Rays.Length);   --  两条视线同一只眼、同一批配点
                        Cv := Geom.Meet_Cov (Rays, Sds, X, Okc);
                        if Okc then
                           declare
                              Hh : constant Long_Float := (X (0) - C.Board_Pt (0)) * N (0) + (X (1) - C.Board_Pt (1)) * N (1) + (X (2) - C.Board_Pt (2)) * N (2);
                              Cn : constant Geom.V3 := Geom.Ap (Cv, N);
                           begin
                              if Hh > Plane_Tol (C, Cn (0) * N (0) + Cn (1) * N (1) + Cn (2) * N (2)) then
                                 Above.Append (Geom.Scene_Pt'(Pw => X, Cov => Cv, Sh => Sig, Views => Natural (Rays.Length), others => <>));
                              end if;
                           end;
                        end if;
                     end if;
                  end if;
               end if;
            end;
         end if;
      end loop;
   end Seen_Above_Of;

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

   procedure Geo_Boot_Support (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
      Down : constant Geom.V3 := [-Protocol_Up (0), -Protocol_Up (1), -Protocol_Up (2)];
      S_Known : Long_Float := 0.0;   --  这次开机头一瓣朝下那一下视线交面离眼多远(世界单位):别的瓣、别的手先一条命令下到按它算的高度

      procedure Go_Back (A : Natural; To : Plug.Arm_Pose) is
         Cur : constant Plug.Arm_Pose := F.EE (A);
         Mok : Boolean;
      begin
         Geo_Move (L, C, F, A, [To (0) - Cur (0), To (1) - Cur (1), To (2) - Cur (2)], Mok);
      end Go_Back;

      --  有板的面:每一瓣换倾角碰几下,量指尖
      procedure Touch_Tips (A, Hc : Natural) is
         Z : constant Zone.Hand_Zone := Zone_Of (C, A, Hc);
         Cw : constant Natural := F.Cams (Hc).W;
         Ch : constant Natural := F.Cams (Hc).H;
         G0 : constant Geom.Cam_Geo := C.Geo (Hc);   --  碰之前那份(身体文件里的指尖,没核过)
         Home : constant Plug.Arm_Pose := F.EE (A);
         --  量到的那张面的法向(朝眼那边):往面里压 = 逆着它,抬 = 顺着它。不按协议的"上"(Protocol_Up):面是墙、是地面一样压得了
         Nb0 : constant Geom.V3 := C.Board_N;
         Into : constant Geom.V3 := [-Nb0 (0), -Nb0 (1), -Nb0 (2)];
         --  这只手以前轻碰时确定空走的那几档各少走多少(核下一回轻碰的第一档是不是已经压着;09-29 V1B66 第 2 只手头一下:粗找压深了,
         --  抬 4.4 mm 手指只离开 0.7 mm,轻碰当"空走的底"的是压着的那一档,11 档都没认出)
         Notch_Pool : aliased Floats;
         --  中位(拷一份插入排序;几十个数)
         function Median_Of (V : Floats) return Long_Float is
            W : Floats := V;
         begin
            for I in 1 .. Natural (W.Length) - 1 loop
               declare
                  X : constant Long_Float := W (I);
                  J : Integer := I - 1;
               begin
                  while J >= 0 and then W (J) > X loop
                     W.Replace_Element (J + 1, W (J));
                     J := J - 1;
                  end loop;
                  W.Replace_Element (J + 1, X);
               end;
            end loop;
            return (if W.Is_Empty then 0.0 else W (Natural (W.Length) / 2));
         end Median_Of;
         --  离中位的散布:中位绝对偏差 × 1.4826(正态下等于标准差;统计换算常数,无量纲)
         function Spread_Of (V : Floats; Med : Long_Float) return Long_Float is
            Dv : Floats;
         begin
            for X of V loop
               Dv.Append (abs (X - Med));
            end loop;
            return 1.4826 * Median_Of (Dv);
         end Spread_Of;
         Tu, Tv, Nw, Nt : Floats;   --  每一瓣指尖的像素、指尖那一小截的像素跨度(宽的那个 / 窄的那个)
         D : Geom.V3_Vectors.Vector;   --  每一瓣指尖的相机系单位视线
         Lobe_At : Geom.Nat_Vectors.Vector;   --  和 D 一一对应:它是握区里的第几瓣(认不出尖的那一瓣不进 D,下标会错开)
         Nl : Natural := 0;
         Who : constant String := "第" & Codec.Img (A + 1) & " 只手";
         Top_H : Long_Float := Long_Float'First;  --  压之前到过的最高处(眼沿面的法向有多高)
         Limit : Boolean := False;   --  上一次 Press_At:压到一半这一压在量到的关节限位里解不出来(停下不是碰到)
         Eqs : Geom.Press_Eq_Vectors.Vector;   --  这只手压过的每一下(顶住那一刻的方程)
         Eq_Lobe : Geom.Nat_Vectors.Vector;    --  每一下对准的是第几瓣
         Est : Geom.V3_Vectors.Vector;         --  每一瓣的尖此刻的估计(相机系;Has_Est 为假 ⇒ 没有,按它的视线交面)
         Has_Est : Bools;
         Gate : constant Long_Float := 4.0 * Geo_Base (C, A);   --  一小步 = 4 倍最小一档(同 Geo_Go 默认的一压、同原来两处对不对得上的门)
         Small : constant Long_Float := 4.0 * Geo_Base (C, A);  --  同上(压的时候的一小步)
         Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算,无量纲)
         Az_Step : constant Long_Float := 72.0 * Deg;   --  五个方位各差 72°(360° ÷ 5,纯几何:没有正对着的两个方位)
         N_Tilt : constant := 5;      --  斜着压的下数(次数)
         N_Extra : constant := 2;     --  对不上时补压的下数(次数;方位取前两个方位中间的)
         function Dot (P, Q : Geom.V3) return Long_Float is (P (0) * Q (0) + P (1) * Q (1) + P (2) * Q (2));
         --  世界里一点沿面的法向落到面上
         function Proj (Q : Geom.V3) return Geom.V3 is
            Hq : constant Long_Float := Dot ([Q (0) - C.Board_Pt (0), Q (1) - C.Board_Pt (1), Q (2) - C.Board_Pt (2)], C.Board_N);
         begin
            return [Q (0) - Hq * C.Board_N (0), Q (1) - Hq * C.Board_N (1), Q (2) - Hq * C.Board_N (2)];
         end Proj;
         --  爪子张开那头的命令(每个抓握通道一个;开机推到头量的读数,Zone.Measure 停在那一头时发的就是它)。量过的通道按它,没量过的保持此刻的读数
         function Open_Cmd return Floats is
            Cur : constant Floats := Selfmap.Jaw_All (F, A);
            R : Floats;
         begin
            for K in 0 .. Natural (Cur.Length) - 1 loop
               declare
                  V : Long_Float := Cur (K);
               begin
                  for Hh of C.Hands loop
                     if Hh.Arm = A and then Hh.K = K and then Hh.Measured then
                        V := Hh.Open_Reading;
                     end if;
                  end loop;
                  R.Append (V);
               end;
            end loop;
            return R;
         end Open_Cmd;
         Jaw_Open : constant Floats := Open_Cmd;
         --  在板上一块空的面上压一下:让手上 Tilt_Dir (这一瓣的视线, Tilt, Azim) 那个方向朝正下(绕眼转,转最少),
         --  按转完以后这一瓣的尖(估计)落在面上的点、别的瓣的视线落在面上的点(再平移 Shift)找一块空的面,
         --  转和挪一条命令走完(反解在量到的关节限位里解;V1B21 2026-09-27:原来按"一条命令转得到的最大一档"一步一步转完再挪,一瓣要 7–8 条命令);
         --  挪完核这一瓣的尖落在哪:离挑好的那块超过指尖那一小截的宽 ⇒ 没转到或没挪到,这一下不压(V1B21:挪 0.22 m 没走到、朝向也被带歪约 30°)。
         --  压到被顶住以后不再往下顶、让手歇下来再读位姿(V1B21 仿真真值:顶着的时候手指压进桌面 6.6 mm,命令一换成停在此刻两拍后回到 2.5 mm)。
         --  Got = 真顶住了(那一刻的方程记进 Eqs、对准第 K 瓣);S_Ray = 此刻这一瓣的视线交面离眼多远(朝下那一下给后面几下当"尖大概在哪"的起点)
         --  Lifted / H_First / Prev_Off:转的时候手指撑在面上、抬起来再转的那几回(09-28 H4,见下面"挪到了没有"),已经抬了多少、
         --  第一回那一刻眼离面多高、上一回挪完落点差多少(这一回没比上一回近 = 抬了没用,挡住它的不是面)。
         --  Seen:压之前刚在此刻这个位姿看过底下、看见挡着原来挑的那一处(见下面"压之前先看底下"),这一回是从这儿重挑的
         procedure Press_At (K : Natural; Tilt, Azim : Long_Float; Shift : Geom.V3; Got : out Boolean; S_Ray : out Long_Float;
                             Lifted : Long_Float := 0.0; H_First : Long_Float := -1.0; Prev_Off : Long_Float := Long_Float'Last;
                             Seen : Boolean := False) is
            P : constant Plug.Arm_Pose := F.EE (A);
            Gk : constant Geom.Cam_Geo := C.Geo (Hc);
            O : constant Geom.V3 := Geom.Cam_Pos (Gk, P);
            Rp : constant Geom.M3 := Geom.Cam_R (Gk, P);
            Nb : constant Geom.V3 := C.Board_N;
            H : constant Long_Float := Dot ([O (0) - C.Board_Pt (0), O (1) - C.Board_Pt (1), O (2) - C.Board_Pt (2)], Nb);
            Rv : constant Geom.V3 := Geom.Turn_To (Geom.Ap (Rp, Geom.Tilt_Dir (D (K), Tilt, Azim)), Into);
            Rr : constant Geom.M3 := Geom.Rodrigues (Rv);
            Ang : constant Long_Float := Geom.Norm (Rv);
            Dk : constant Geom.V3 := Geom.Ap (Rr, Geom.Ray (Gk, P, Tu (K), Tv (K)));   --  转完以后这一瓣的视线(绕眼转,眼不动)
            Ck : constant Long_Float := -Dot (Nb, Dk);   --  它和朝下的夹角的余弦
            --  转完以后这一瓣的尖(相对眼,世界系):有估计按估计;没有 ⇒ 按它的视线交面(视线朝正下时就是正下方那一点)
            Tk : constant Geom.V3 := (if Has_Est (K) then Geom.Ap (Rr, Geom.Ap (Rp, Est (K)))
                                      elsif Ck > 1.0e-9 then [H / Ck * Dk (0), H / Ck * Dk (1), H / Ck * Dk (2)] else [0.0, 0.0, 0.0]);
            A0 : constant Geom.V3 := Proj ([O (0) + Tk (0), O (1) + Tk (1), O (2) + Tk (2)]);   --  这一瓣的尖碰到面时落在哪
            Lp : Geom.V3_Vectors.Vector;
            Tb : Floats;   --  别的瓣:从压的这一瓣的尖到它的尖那条连线的坡度(两根手指一样长时,Board_Free_Spots 算它的手指离面多高)
            Dl : Geom.V3 := [0.0, 0.0, 0.0];
            All_Hit : Boolean := Ck > 1.0e-9;
            --  指尖那一小截的宽(像素)落到尖那么远:有估的尖按它离眼多远(量的);没有 ⇒ 按眼离面多高(尖在眼和面之间,上限)。
            --  09-30 V1B77:原来一直按眼离面多高,压不成、抬高以后圈跟着变大(0.798 单位),那一带一处空地都挑不出
            R : constant Long_Float := Nw (K) * (if Has_Est (K) then Long_Float'Min (Long_Float'Max (0.0, H), Geom.Norm (Est (K))) else Long_Float'Max (0.0, H)) / Gk.F;
            Spot : Geom.V3;
            Aim_O : Geom.V3 := [0.0, 0.0, 0.0];
            Moved : Boolean := True;   --  这一回转和挪那条命令真要动(超过这只手平移、转动各自一步看得见的那一档)
         begin
            Got := False; S_Ray := 0.0; Limit := False;
            Lp.Append (Geom.V3'[A0 (0) + Shift (0), A0 (1) + Shift (1), A0 (2) + Shift (2)]);
            Tb.Append (0.0);
            for J in 0 .. Nl - 1 loop
               if J /= K then
                  declare
                     Dj : constant Geom.V3 := Geom.Ap (Rr, Geom.Ray (Gk, P, Tu (J), Tv (J)));
                     Cj : constant Long_Float := -Dot (Nb, Dj);
                     Vt : constant Geom.V3 := [Dj (0) - Dk (0), Dj (1) - Dk (1), Dj (2) - Dk (2)];
                     Hz : constant Long_Float := Dot (Nb, Vt);
                     Hv : constant Geom.V3 := [Vt (0) - Hz * Nb (0), Vt (1) - Hz * Nb (1), Vt (2) - Hz * Nb (2)];
                     Lh : constant Long_Float := Geom.Norm (Hv);
                     --  沿它的视线从眼到面那么远(朝正下压时这条连线的另一头就是它的视线交面那一点,同原来)
                     Sj : constant Long_Float := (if Cj > 1.0e-9 then H / Cj else 0.0);
                  begin
                     All_Hit := All_Hit and then Cj > 1.0e-9;
                     Lp.Append (Geom.V3'[A0 (0) + Sj * Hv (0) + Shift (0), A0 (1) + Sj * Hv (1) + Shift (1), A0 (2) + Sj * Hv (2) + Shift (2)]);
                     Tb.Append ((if Lh > 0.0 then Long_Float'Max (0.0, Hz) / Lh else 0.0));
                  end;
               end if;
            end loop;
            if H <= 0.0 or else not All_Hit then
               Geo_Say ("  眼在板的面之下、或手指的视线落不到面上(眼离面 " & Mm (H) & ")⇒ 这一下压不成");
               return;
            end if;
            --  挑落点:空的面按离得近排,一处一处先问反解(转和挪到那儿在量到的关节限位里解不解得出来,还差一步看得见的那一档以内才去);
            --  有尖的估计时,压到"尖在面以下一小步"那么低也要解得出来(V1B31 2026-09-27:往下走到指尖离桌面 5 mm 时到了关节限位、手停住被当成碰到)
            declare
               Ds : Geom.V3_Vectors.Vector;
               Found : Boolean := False;
               Asked : Natural := 0;
               Tol_P : constant Long_Float := Geo_Base (C, A);
               Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
            begin
               Board_Free_Spots (C, Lp, Tb, R, Ds);
               for D1 of Ds loop
                  declare
                     Av : Table.Vec := Table.Zero_Vec;
                     Pe, Re : Long_Float;
                     Rok : Boolean;
                  begin
                     for I in 0 .. 2 loop
                        Av (I) := Shift (I) + D1 (I);
                        Av (3 + I) := Rv (I);
                     end loop;
                     Plug.Reach (A, Chan.Compose (P, Av), Pe, Re, Rok);
                     Asked := Asked + 1;
                     if Rok and then Pe <= Tol_P and then Re <= Tol_R and then Has_Est (K) then
                        declare
                           Dz : constant Long_Float := Long_Float'Max (0.0, H - (-Dot (Nb, Tk) - Small));
                           Pe2, Re2 : Long_Float;
                           Rok2 : Boolean;
                           Av2 : Table.Vec := Av;
                        begin
                           for I in 0 .. 2 loop
                              Av2 (I) := Av (I) + Dz * Into (I);
                           end loop;
                           Plug.Reach (A, Chan.Compose (P, Av2), Pe2, Re2, Rok2);
                           Pe := Long_Float'Max (Pe, Pe2); Re := Long_Float'Max (Re, Re2);
                        end;
                     end if;
                     if not Rok or else (Pe <= Tol_P and then Re <= Tol_R) then
                        Dl := D1; Found := True;
                        exit;
                     end if;
                  end;
               end loop;
               if not Found then
                  if Ds.Is_Empty then
                     Geo_Say ("  板上量过、此刻还找得到的桌面里没有一处落点圈(半径 " & Mm (R) & ")整个在里面、又躲得开高出面的点 ⇒ 这一下压不成");
                  else
                     Geo_Say ("  板上空的面 " & Codec.Img (Natural (Ds.Length)) & " 处(指尖那一小截宽的上限 " & Mm (R) & "),问了 " & Codec.Img (Asked)
                              & " 处,转和挪到那儿在量到的关节限位里都解不出来 ⇒ 这一下压不成");
                  end if;
                  return;
               end if;
               if Asked > 1 then
                  Geo_Say ("  空的面按远近问了 " & Codec.Img (Asked) & " 处,前 " & Codec.Img (Asked - 1) & " 处在量到的关节限位里解不出来,去第 " & Codec.Img (Asked) & " 处");
               end if;
            end;
            Spot := [Lp (0) (0) + Dl (0), Lp (0) (1) + Dl (1), Lp (0) (2) + Dl (2)];
            declare
               Av : Table.Vec := Table.Zero_Vec;
               Del : Table.Vec;
               Mok : Boolean;
            begin
               for I in 0 .. 2 loop
                  Av (I) := Shift (I) + Dl (I);
                  Av (3 + I) := Rv (I);
               end loop;
               Aim_O := Geom.Cam_Pos (Gk, Chan.Compose (P, Av));   --  这条命令要眼到的地方(下面核它有没有被顶高)
               declare
                  Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
               begin
                  Moved := Geom.Norm ([Av (0), Av (1), Av (2)]) > Geo_Base (C, A) or else Ang > Tol_R;
               end;
               --  爪子这一条命令里按张开那头发(复位以后爪子的目标作废,不发就跟着读数走)
               Step_Arm (L, C, F, A, Av, Jaw_Open, Del, Mok, Geo_Settle => True);
               Geo_Say ("  让手上" & (if Tilt > 0.0 then "这一瓣的视线朝方位 " & Codec.Fmt (Azim / Deg, 0) & "° 斜 " & Codec.Fmt (Tilt / Deg, 1) & "° 那个方向" else "这一瓣的视线")
                        & "朝下:转 " & Codec.Fmt (Ang, 3) & " rad、挪 (" & Mm (Av (0)) & "," & Mm (Av (1)) & "," & Mm (Av (2))
                        & ") 到板上空的那块,一条命令 ⇒ 实到转 " & Codec.Fmt (Sqrt (Del (3) ** 2 + Del (4) ** 2 + Del (5) ** 2), 3) & " rad、挪 ("
                        & Mm (Del (0)) & "," & Mm (Del (1)) & "," & Mm (Del (2)) & ")(指尖那一小截宽的上限 " & Mm (R) & ",眼离面 " & Mm (H) & ")"
                        & (if Mok then "" else " · 身体说没走成"));
            end;
            --  挪到了没有:按此刻的位姿重算这一瓣的尖落在面上哪儿,离挑好的那块超过手指宽上限 ⇒ 没挪到(够不着那么远),不在没核过的地方压
            declare
               P2 : constant Plug.Arm_Pose := F.EE (A);
               O2 : constant Geom.V3 := Geom.Cam_Pos (Gk, P2);
               Hok : Boolean := True;
               Q2 : constant Geom.V3 := (if Has_Est (K)
                                         then Proj (Geom.V3'[O2 (0) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (0), O2 (1) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (1),
                                                             O2 (2) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (2)])
                                         else Geom.Hit_Plane (O2, Geom.Ray (Gk, P2, Tu (K), Tv (K)), C.Board_Pt, Nb, Hok));
               Off_By : constant Long_Float := Geom.Norm ([Q2 (0) - Spot (0), Q2 (1) - Spot (1), Q2 (2) - Spot (2)]);
               --  眼比命令的高多少(沿面的法向):手指一转就撑在面上、把手顶起来了(09-28 H4:人形手指约和眼离桌一样长,原地转向下,
               --  腕俯仰要到 1.60 只到 1.17、眼被顶高约 5 cm)
               Up_By : constant Long_Float := Dot ([O2 (0) - Aim_O (0), O2 (1) - Aim_O (1), O2 (2) - Aim_O (2)], Nb);
               H0 : constant Long_Float := (if H_First > 0.0 then H_First else H);
               Ln : constant Long_Float := Stride_Of (C, A);
            begin
               if not Hok or else Off_By > R then
                  --  抬了没用(这一回落点没比上一回近)⇒ 挡住它的不是面:关节到了真尽头、或撞在别处(09-28 H5:第一只手腕俯仰顶在仿真的真尽头 1.609,
                  --  反解不知道,照样往 1.7 以上解,手停在偏高的位姿,被当成撑在面上连抬 4 回、眼抬到离面 45 cm)
                  if Up_By > Small and then Ln > 0.0 and then Lifted + Ln <= H0 and then Off_By < Prev_Off then
                     --  被面顶起来了 ⇒ 沿法向抬一大步(步幅),从那儿把这一下重算一遍再转(抬的总量不超过第一回眼离面的高度:
                     --  手指比那还长的身体这样碰不出来,下面照实说)
                     Geo_Say ("  转的时候手指撑在面上了(眼比命令的高 " & Mm (Up_By) & ")⇒ 抬一大步(" & Mm (Ln) & ")再转");
                     declare
                        Mok : Boolean;
                     begin
                        Geo_Move (L, C, F, A, [Ln * Nb (0), Ln * Nb (1), Ln * Nb (2)], Mok);
                     end;
                     Press_At (K, Tilt, Azim, Shift, Got, S_Ray, Lifted + Ln, H0, Off_By);
                     return;
                  end if;
                  Geo_Say ("  没挪到那块空的面(落点差 " & Mm (Off_By) & ",手指宽上限 " & Mm (R)
                           & (if Up_By > Small and then Lifted > 0.0 and then Off_By >= Prev_Off
                              then ",眼比命令的高 " & Mm (Up_By) & ",可抬了一大步落点没比上一回(差 " & Mm (Prev_Off) & ")近 ⇒ 挡住它的不是面"
                              elsif Up_By > Small then ",眼比命令的高 " & Mm (Up_By) & ",已经抬过 " & Mm (Lifted) & "、再抬就超过眼离面的高度 " & Mm (H0)
                              else "") & ")⇒ 这一下不压");
                  return;
               end if;
            end;
            --  往下压(沿量到的那张面的法向压进去),两段:粗找(找到面在哪)+ 轻碰(在那儿读位姿)。
            --  每一步都等胳膊沿压的方向真停下来再读(Selfmap.Go 的 Press:动起来以后连着两拍挪不到这一步的百分之一 = 停了)。
            --  碰到没有 = Selfmap.Blocked:比上一步空走时多少走的量超过"这一步的百分之一 / 3 倍读数噪声 / 3 倍前两步空走之差"里最大的那样
            --  (09-29 台架:x5 两步空走少走的量前后只差约 1e-5 单位,碰上的第一步多少走至少 0.0014;原来的门 = 第一步 + 3 × 静止噪声,
            --  仿真读数不抖 ⇒ 门 = 第一步,差一丝就认成碰到:V1B60 虚认 37 次、V1B65 18 次)。
            --  粗找:有尖的估计时先一条命令下到"按估的尖算,离面三小步",再一小步一小步往下;这一下就被顶住了(比估的长)⇒ 抬两大步,按头一回的走法;
            --  小步下到"按估的尖算的桌面以下一大步"还没碰到(比估的短)⇒ 从那儿接着按大步压。头一回(没有估计):一大步一大步往下,碰到了
            --  ⇒ 退回碰到的那一大步开始的地方,等手指回过来(自己那只眼里画面停下;09-29 V1B66:大步压下去 21 mm,手指被顶开,退回后回弹约 5 拍,
            --  这期间第一小步碰上了却一点没少走),再一小步一小步找。
            --  轻碰:抬一小步 + 两档,等手指回过来,再一档一档往下;第一档比这只手以前确定空走的一档多少走得多 ⇒ 手指还压着(粗找压深了)
            --  ⇒ 再抬一小步重来;碰到 ⇒ 就在那一刻读位姿(不歇:位置控制下停在此刻卸不掉压着的那一点)
            declare
               Start : constant Plug.Arm_Pose := F.EE (A);
               Ln : constant Long_Float := Stride_Of (C, A);   --  一压 = 步幅
               Cap : constant Natural := (if Ln > 0.0 then Natural (Long_Float'Ceiling (H / Ln)) + 1 else 0);
               Notch : constant Long_Float := Geo_Base (C, A);
               Direct : Boolean := False;
               Coarse : Boolean := False;    --  粗找找到了面
               Touched : Boolean := False;   --  轻碰碰到了
               --  沿 Into 走一步 Lstep:先问反解(同 Geo_Go:位置还差超过这一步的一半、或朝向差超过转动一步看得见的那一档 = 到了量到的关节限位,不走)
               procedure Step_Down (Lstep : Long_Float; Short : out Long_Float; At_Limit : out Boolean) is
                  Cur : constant Plug.Arm_Pose := F.EE (A);
                  Av : Table.Vec := Table.Zero_Vec;
                  Pe, Re : Long_Float;
                  Rok, Mok : Boolean;
                  Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
               begin
                  Short := 0.0; At_Limit := False;
                  for I in 0 .. 2 loop
                     Av (I) := Lstep * Into (I);
                  end loop;
                  Plug.Reach (A, Chan.Compose (Cur, Av), Pe, Re, Rok);
                  if Rok and then (Pe + Pe > Lstep or else (Tol_R > 0.0 and then Re > Tol_R)) then
                     At_Limit := True;
                     return;
                  end if;
                  --  等胳膊沿压的方向停下来再读(Press):被顶住的软手指那点转动蠕动不等;伸远了还在漂就接着等,不到固定拍数就读
                  Geo_Move (L, C, F, A, [Av (0), Av (1), Av (2)], Mok, Press => True);
                  declare
                     Now : constant Plug.Arm_Pose := F.EE (A);
                  begin
                     Short := Lstep - ((Now (0) - Cur (0)) * Into (0) + (Now (1) - Cur (1)) * Into (1) + (Now (2) - Cur (2)) * Into (2));
                  end;
               end Step_Down;
               --  等这只手自己那只眼里的画面停下来(被顶开的手指回过来),最多同 Go 的上限;用了几拍照说
               procedure Settle (Why : String) is
                  Prev : Plug.Cam := F.Cams (Hc);
                  Still : Natural := 0;
                  Used : Natural := 0;
               begin
                  for I in 1 .. 12 + C.Map.Settle loop
                     exit when not Plug.Sense (L, F);
                     Used := I;
                     Still := (if Selfmap.Picture_Still (C.Map, Prev, F.Cams (Hc), Hc) then Still + 1 else 0);
                     Prev := F.Cams (Hc);
                     exit when Still >= 2;
                  end loop;
                  Geo_Say ("  " & Why & ":等自己那只眼里画面停下(手指回过来)用了 " & Codec.Img (Used) & " 拍" & (if Still >= 2 then "" else ",到上限还在动"));
               end Settle;
               --  一步一步往下(每步 Lstep,最多 Steps 步),按 Selfmap.Blocked 认碰到。Base0 给了 = 已经走过的一步空走的少走量(当第一个底);
               --  Free 给了 = 判成空走的每一步各少走多少
               procedure Descend (Lstep : Long_Float; Steps : Natural; Got_It : out Boolean; From : out Plug.Arm_Pose; Said : String;
                                  Base0 : Long_Float := Long_Float'First; Free : access Floats := null) is
                  Prev, Prev2, Sh : Long_Float := 0.0;
                  N_Free : Natural := 0;
                  Lim : Boolean;
               begin
                  Got_It := False;
                  From := F.EE (A);
                  if Base0 /= Long_Float'First then
                     Prev := Base0; N_Free := 1;
                     if Free /= null then
                        Free.Append (Base0);
                     end if;
                  end if;
                  for I in 1 .. Steps loop
                     From := F.EE (A);   --  这一步开始的地方(碰到的那一步开始时手指还没碰到:上一步是空走的)
                     Step_Down (Lstep, Sh, Lim);
                     if Lim then
                        Limit := True;
                        Geo_Say ("  " & Said & ":再往下一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
                        return;
                     end if;
                     if Selfmap.Blocked (Sh, Prev, Prev2, N_Free, Lstep, C.Map.EE_Noise) then
                        Got_It := True;
                        Geo_Say ("  " & Said & ":第 " & Codec.Img (I) & " 步(一步 " & Mm (Lstep) & ")少走 " & Mm (Sh) & ",空走时少走 " & Mm (Prev)
                                 & "(门 " & Mm (Prev + Long_Float'Max (Selfmap.Negligible * Lstep,
                                                                      3.0 * Long_Float'Max (C.Map.EE_Noise, (if N_Free >= 2 then abs (Prev - Prev2) else 0.0))))
                                 & ")⇒ 碰到");
                        return;
                     end if;
                     Prev2 := Prev; Prev := Sh; N_Free := N_Free + 1;
                     if Free /= null then
                        Free.Append (Sh);
                     end if;
                  end loop;
                  Geo_Say ("  " & Said & ":往下 " & Codec.Img (Steps) & " 步(一步 " & Mm (Lstep) & ")都没认出碰到");
               end Descend;
               --  大步找:一大步一大步(一步 = 步幅)往下;碰到的那一大步开始的地方手指还没碰到 ⇒ 退回那儿、等手指回过来,
               --  再一小步一小步找(最多一大步那么深再多两步,次数)。小步往下一大步那么深都没碰着 ⇒ 大步那一下是虚的 ⇒ 从这儿接着大步往下,
               --  直到小步真碰着、到了量到的关节限位、或者眼走到面那么低(纯几何)。Base0 = 刚走过的一大步空走的少走量(压之前看底下那一步;
               --  给了 ⇒ 第一大步就有得比,见 Descend)
               procedure Big_Press (Base0 : Long_Float := Long_Float'First) is
                  Fr : Plug.Arm_Pose;
                  Hit : Boolean;
                  B0 : Long_Float := Base0;
               begin
                  loop
                     Descend (Ln, Cap, Hit, Fr, "一大步一大步找", Base0 => B0);
                     B0 := Long_Float'First;
                     exit when not Hit;
                     declare
                        Now : constant Plug.Arm_Pose := F.EE (A);
                        Mok : Boolean;
                     begin
                        Geo_Move (L, C, F, A, [Fr (0) - Now (0), Fr (1) - Now (1), Fr (2) - Now (2)], Mok);
                        Settle ("退回碰到的那一大步开始的地方");
                        Descend (Small, Natural (Long_Float'Ceiling (Ln / Small)) + 2, Coarse, Fr, "退回碰到的那一大步开始的地方、一小步一小步找");
                     end;
                     exit when Coarse or else Limit;
                     declare
                        On : constant Geom.V3 := Geom.Cam_Pos (Gk, F.EE (A));
                        Hn : constant Long_Float := Dot ([On (0) - C.Board_Pt (0), On (1) - C.Board_Pt (1), On (2) - C.Board_Pt (2)], Nb);
                     begin
                        exit when Hn <= 0.0;
                        Geo_Say ("  小步往下一大步那么深都没碰着 ⇒ 大步那一下是虚的 ⇒ 接着按大步压(眼离面 " & Mm (Hn) & ")");
                     end;
                  end loop;
               end Big_Press;
               --  有尖的估计时一条命令下多少,到"按估的尖算,离面三小步"(按此刻的位姿)。三小步(次数:估的尖差一两小步时第一小步照样是空走的)。
               --  09-28 V1B51 试过两小步:第 1 只手第 2 瓣斜 216° 那一下第一小步就碰着了(当底的那一步坏了)⇒ 这一瓣差到 4.3 mm ⇒ 三小步
               function Drop_To_Est return Long_Float is
                  P3 : constant Plug.Arm_Pose := F.EE (A);
                  O3 : constant Geom.V3 := Geom.Cam_Pos (Gk, P3);
                  H3 : constant Long_Float := Dot ([O3 (0) - C.Board_Pt (0), O3 (1) - C.Board_Pt (1), O3 (2) - C.Board_Pt (2)], Nb);
               begin
                  return H3 - (-Dot (Nb, Geom.Ap (Geom.Cam_R (Gk, P3), Est (K))) + 3.0 * Small);
               end Drop_To_Est;
               --  压之前先看底下(09-30 V1B70 / V1B73):开机量的板只有不动的眼看得见、腕眼三角得出的那片,手自己挡着的那块没有板点 ——
               --  V1B70 / V1B73 第 1 只手底下那块是一台电子琴,挑空地只拿板点挡,另一瓣(V1B73 连压的那一瓣)压在琴上查不出。
               --  往下压的第一步本身就是一对立体像:Im0 / P0 = 走之前那一帧和位姿,此刻 = 走之后;两帧之间眼只平移(位姿读数量的)。
               --  问的点见 Look_Points(手指像素照样问)。比面高出的(Seen_Above_Of)进 C.Seen_Above;
               --  Blocked = 按新看见的点,挑好的这一处(Dl)不再是空的
               procedure Look_Below (Im0 : Plug.Cam; P0 : Plug.Arm_Pose; Blocked : out Boolean) is
                  Q, M : Instrument.Match_Vectors.Vector;
                  Err : Unbounded_String;
                  Qu, Qv, Mu, Mv, Bu, Bv : Floats;
                  New_Pts : Geom.Scene_Pt_Vectors.Vector;
                  Matched, Tri : Natural;
                  Sig : Long_Float;
                  P1 : constant Plug.Arm_Pose := F.EE (A);
                  Went : constant Long_Float := Geom.Norm ([Geom.Cam_Pos (Gk, P1) (0) - Geom.Cam_Pos (Gk, P0) (0), Geom.Cam_Pos (Gk, P1) (1) - Geom.Cam_Pos (Gk, P0) (1),
                                                            Geom.Cam_Pos (Gk, P1) (2) - Geom.Cam_Pos (Gk, P0) (2)]);
               begin
                  Blocked := False;
                  declare
                     Ends : Geom.V3_Vectors.Vector;
                  begin
                     for J in 1 .. Natural (Lp.Length) - 1 loop
                        Ends.Append (Geom.V3'[Lp (J) (0) + Dl (0), Lp (J) (1) + Dl (1), Lp (J) (2) + Dl (2)]);
                     end loop;
                     Q := Look_Points (C, Gk, P0, Cw, Ch, Spot, Ends, R, Nt (K));
                  end;
                  M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, Im0.RGB, Cw, Ch, F.Cams (Hc).RGB, Cw, Ch, Q, Err, Back => True);
                  if Natural (M.Length) /= Natural (Q.Length) then
                     Geo_Say ("  压之前看底下:仪器没配成(" & To_String (Err) & ")⇒ 这一处按原来知道的那份压");
                     return;
                  end if;
                  for I in 0 .. Natural (Q.Length) - 1 loop
                     Qu.Append (Q (I).U); Qv.Append (Q (I).V); Mu.Append (M (I).U); Mv.Append (M (I).V); Bu.Append (M (I).Bu); Bv.Append (M (I).Bv);
                  end loop;
                  Seen_Above_Of (C, Gk, P0, P1, Cw, Ch, Qu, Qv, Mu, Mv, Bu, Bv, New_Pts, Matched, Tri, Sig);
                  for Pt of New_Pts loop
                     C.Seen_Above.Append (Pt);
                  end loop;
                  if not New_Pts.Is_Empty then
                     declare
                        Ds2 : Geom.V3_Vectors.Vector;
                     begin
                        Board_Free_Spots (C, Lp, Tb, R, Ds2);
                        Blocked := not (for some D2 of Ds2 => Geom."=" (D2, Dl));   --  候选是同一批板点算的:没被挡就原样还在
                     end;
                  end if;
                  Geo_Say ("  压之前看底下(往下第一步前后两帧,眼挪了 " & Mm (Went) & "):问 " & Codec.Img (Natural (Q.Length)) & " 个点(格点 + 落点圈和带子里密铺的,手指像素不问)、配上 "
                           & Codec.Img (Matched) & " 个、交成且两帧对得上 " & Codec.Img (Tri) & " 个(配点噪声 " & Codec.Fmt (Sig, 2) & " px)、比面高出的 "
                           & Codec.Img (Natural (New_Pts.Length)) & " 个(看见的一共 " & Codec.Img (Natural (C.Seen_Above.Length)) & ")⇒ "
                           & (if Blocked then "挑好的这一处被挡了 ⇒ 退回去,从这儿重挑" else "这一处照样空"));
               end Look_Below;
               Look_Free : Long_Float := Long_Float'First;   --  压之前看底下那一步空走的少走量(大步找的第一大步拿它当底)
            begin
               Top_H := Long_Float'Max (Top_H, Start (0) * Nb0 (0) + Start (1) * Nb0 (1) + Start (2) * Nb0 (2));
               --  压之前先看底下(见 Look_Below):往下压的第一步 —— 大步找的第一大步;有尖的估计时是"下到尖离面约三小步"那一条命令的头一大步 ——
               --  走完了看。看见挡着挑好的这一处 ⇒ 退回去,从这儿重挑(Press_At 再来一遍;挡的点只多不少 ⇒ 这一处不会再被挑中,空地只会少,挑不到照实说)。
               --  刚看过、重挑挑中的又是原处(这一回没挪)⇒ 不再看。没配配点仪器 ⇒ 看不了,照原来知道的那份压
               if not (Seen and then not Moved) and then Length (C.Inst_Host) > 0 and then Ln > 0.0 and then Cw > 0 then
                  declare
                     Im0 : constant Plug.Cam := F.Cams (Hc);
                     P0 : constant Plug.Arm_Pose := F.EE (A);
                     First : constant Long_Float := (if Has_Est (K) and then Small > 0.0 then Long_Float'Min (Ln, Drop_To_Est) else Ln);
                     Sh : Long_Float;
                     Lim, Blocked, Mok : Boolean;
                  begin
                     if First > 0.0 then
                        Step_Down (First, Sh, Lim);
                        if Lim then
                           Limit := True;
                           Geo_Say ("  压之前看底下:往下第一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
                        else
                           Look_Below (Im0, P0, Blocked);
                           if Blocked then
                              declare
                                 Now : constant Plug.Arm_Pose := F.EE (A);
                              begin
                                 Geo_Move (L, C, F, A, [P0 (0) - Now (0), P0 (1) - Now (1), P0 (2) - Now (2)], Mok);
                              end;
                              Press_At (K, Tilt, Azim, [0.0, 0.0, 0.0], Got, S_Ray, Seen => True);
                              return;
                           end if;
                           if First = Ln then
                              Look_Free := Sh;
                           end if;
                        end if;
                     end if;
                  end;
               end if;
               if not Limit and then Has_Est (K) and then Small > 0.0 and then Ln > 0.0 then
                  declare
                     P3 : constant Plug.Arm_Pose := F.EE (A);
                     O3 : constant Geom.V3 := Geom.Cam_Pos (Gk, P3);
                     H3 : constant Long_Float := Dot ([O3 (0) - C.Board_Pt (0), O3 (1) - C.Board_Pt (1), O3 (2) - C.Board_Pt (2)], Nb);
                     Depth3 : constant Long_Float := -Dot (Nb, Geom.Ap (Geom.Cam_R (Gk, P3), Est (K)));   --  估的尖此刻在眼下多深
                     Dn : constant Long_Float := Drop_To_Est;
                     Mok : Boolean;
                  begin
                     if Dn > 0.0 then
                        declare
                           Jaw : Floats;
                           Del : Table.Vec;
                           Av : Table.Vec := Table.Zero_Vec;
                        begin
                           for I in 0 .. 2 loop
                              Av (I) := Dn * Into (I);
                           end loop;
                           Step_Arm (L, C, F, A, Av, Jaw, Del, Mok, Geo_Settle => True);
                           Look_Free := Long_Float'First;   --  手挪过了:看底下那一步的少走量不再是大步找的底
                           declare
                              Went : constant Long_Float := Del (0) * Into (0) + Del (1) * Into (1) + Del (2) * Into (2);
                           begin
                              Direct := Went + Geo_Base (C, A) >= Dn;
                              Geo_Say ("  按估的尖(离眼 " & Mm (Geom.Norm (Est (K))) & "、此刻在眼下 " & Mm (Depth3) & ")一条命令下 " & Mm (Dn) & " 到尖离面约三小步 ⇒ 实到 " & Mm (Went)
                                       & (if Direct then ",一小步一小步找" else ",这一下就被顶住了(比估的长)⇒ 抬两大步,按头一回的走法"));
                              if not Direct then
                                 --  抬两大步(两 = 次数:被顶住时尖在面上或更低,抬一大步第一大步未必是空走的)
                                 Geo_Move (L, C, F, A, [2.0 * Ln * Nb0 (0), 2.0 * Ln * Nb0 (1), 2.0 * Ln * Nb0 (2)], Mok);
                                 Settle ("被顶住、抬两大步以后");
                              end if;
                           end;
                        end;
                     else
                        Direct := True;
                     end if;
                  end;
               end if;
               if Limit then
                  null;   --  看底下那一步就到了量到的关节限位:这一下不算(上面说过了),下面只抬起来
               elsif Direct then
                  declare
                     Fr : Plug.Arm_Pose;
                  begin
                     Descend (Small, Natural (Long_Float'Ceiling ((3.0 * Small + Ln) / Small)) + 1, Coarse, Fr, "一小步一小步找");   --  三小步 + 一大步那么深(次数)
                  end;
                  if not Coarse and then not Limit then
                     Geo_Say ("  下到按估的尖算的桌面以下一大步还没碰到(比估的短)⇒ 接着按大步压");
                     Big_Press;
                  end if;
               else
                  Big_Press (Look_Free);
               end if;
               --  粗找认成碰到、轻碰往下一小步那么深都没碰着 ⇒ 粗找那一下是虚的 ⇒ 细的(轻碰)否掉粗的,从这儿接着一小步一小步往下找
               --  (还没碰到 ⇒ 大步),直到轻碰真碰着、到了量到的关节限位、或者眼走到面那么低(眼到不了面以下,纯几何)
               loop
                  exit when not Coarse;
                  --  轻碰:抬一小步 + 两档(粗找多压不到一小步,纯几何),等手指回过来,再一档一档往下(最多抬的那么多再加一小步)
                  declare
                     Mok, Lim : Boolean;
                     Up : constant Long_Float := Small + 2.0 * Notch;
                     Lifted : Long_Float := Up;
                     Fr : Plug.Arm_Pose;
                     S1 : Long_Float := 0.0;
                     Free_Now : aliased Floats;
                  begin
                     Geo_Move (L, C, F, A, [Up * Nb0 (0), Up * Nb0 (1), Up * Nb0 (2)], Mok);
                     Settle ("抬起来以后");
                     --  第一档:比这只手以前确定空走的一档多少走得多(同 Blocked 的门)⇒ 手指还压着 ⇒ 再抬一小步重来(最多多抬一大步)
                     loop
                        Step_Down (Notch, S1, Lim);
                        exit when Lim;
                        declare
                           N_P : constant Natural := Natural (Notch_Pool.Length);
                           Med : constant Long_Float := (if N_P >= 3 then Median_Of (Notch_Pool) else 0.0);
                           Spr : constant Long_Float := (if N_P >= 3 then Spread_Of (Notch_Pool, Med) else 0.0);
                           Pressed : constant Boolean := N_P >= 3 and then Selfmap.Blocked (S1, Med, Med - Spr, 2, Notch, C.Map.EE_Noise);
                        begin
                           exit when not Pressed or else Lifted >= Up + Ln;
                           Geo_Say ("  轻碰第一档就少走 " & Mm (S1) & "(这只手确定空走的一档中位 " & Mm (Med) & ")⇒ 手指还压着(粗找压深了)⇒ 再抬一小步");
                           Geo_Move (L, C, F, A, [(Small + Notch) * Nb0 (0), (Small + Notch) * Nb0 (1), (Small + Notch) * Nb0 (2)], Mok);
                           Lifted := Lifted + Small;
                           Settle ("再抬一小步以后");
                        end;
                     end loop;
                     if Lim then
                        Limit := True;
                        Geo_Say ("  轻碰(一档一档):再往下一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
                     else
                        Descend (Notch, Natural (Long_Float'Ceiling ((Lifted + Small) / Notch)), Touched, Fr, "轻碰(一档一档)",
                                 Base0 => S1, Free => Free_Now'Access);
                        if Touched then
                           --  确定空走的那几档进这只手的底子(碰到的前一档可能已经擦着,不要)
                           for I in 0 .. Natural (Free_Now.Length) - 2 loop
                              Notch_Pool.Append (Free_Now (I));
                           end loop;
                        end if;
                     end if;
                  end;
                  exit when Touched or else Limit;
                  declare
                     On : constant Geom.V3 := Geom.Cam_Pos (Gk, F.EE (A));
                     Hn : constant Long_Float := Dot ([On (0) - C.Board_Pt (0), On (1) - C.Board_Pt (1), On (2) - C.Board_Pt (2)], Nb);
                     Fr : Plug.Arm_Pose;
                  begin
                     exit when Hn <= 0.0;
                     Geo_Say ("  轻碰往下一小步那么深都没碰着 ⇒ 粗找那一下是虚的 ⇒ 从这儿接着一小步一小步找(眼离面 " & Mm (Hn) & ")");
                     Descend (Small, Natural (Long_Float'Ceiling (Ln / Small)) + 2, Coarse, Fr, "接着一小步一小步找");   --  一大步那么深再多两步(次数)
                  end;
                  if not Coarse and then not Limit then
                     Geo_Say ("  一大步那么深还没碰到 ⇒ 接着按大步压");
                     Big_Press;
                  end if;
               end loop;
               if Touched then
                  --  碰到那一刻手上最低的那一点在面上 ⇒ 一条方程;碰到的是不是这一瓣的尖、是不是桌面,由几下对不对得上管(Fit_Presses)
                  declare
                     Pc : constant Plug.Arm_Pose := F.EE (A);
                     Vs : Geom.Board_View_Vectors.Vector;
                     Row : Geom.Plane_Tip_Vectors.Vector;
                     Jr : constant Floats := Selfmap.Jaw_All (F, A);
                     Jd : Long_Float := 0.0;   --  爪子读数离张开那头最多差多少(只记账:仿真里读数是上一拍命令的回声,手指被顶开 20% 行程以上它才跟着变)
                  begin
                     for I in 0 .. Natural'Min (Natural (Jr.Length), Natural (Jaw_Open.Length)) - 1 loop
                        Jd := Long_Float'Max (Jd, abs (Jr (I) - Jaw_Open (I)));
                     end loop;
                     Eqs.Append (Geom.Press_Of (C.Geo (Hc), Pc, C.Board_Pt, Nb));
                     Eq_Lobe.Append (K);
                     Vs.Append (Geom.Board_View'(Pose => Pc, U => Tu (K), V => Tv (K)));
                     Row := Geom.Tips_On_Plane (C.Geo (Hc), Vs, C.Board_Pt, Nb, C.Board_Rms);
                     if Row (0).Ok then
                        S_Ray := Row (0).S;
                     end if;
                     Got := True;
                     Geo_Say ("  碰到:眼离面 " & Mm (-Eqs.Last_Element.B) & "、这一瓣的视线交面离眼 " & (if Row (0).Ok then Mm (S_Ray) else "交不到")
                              & " · 爪子读数离张开那头 " & Codec.Fmt (Jd, 4));
                  end;
               end if;
               --  压完抬两压(次数)就走,不回压之前的高处:下一处从这个高度横挪过去(⑧ 的 (a));最高到过哪儿照样记着,最后回原处上方
               declare
                  Mok : Boolean;
               begin
                  Geo_Move (L, C, F, A, [2.0 * Ln * Nb0 (0), 2.0 * Ln * Nb0 (1), 2.0 * Ln * Nb0 (2)], Mok);
               end;
            end;
         end Press_At;
         --  压一下;压到一半在量到的关节限位里解不出来(不是没挑到空地)⇒ 沿面挪开 4 倍一压换两边各试一处(同原来第一处的挪法)
         Far0 : constant Long_Float := 4.0 * Small;   --  4 倍一压(倍数,无量纲)
         function Along (Dd : Long_Float) return Geom.V3 is
            Nb : constant Geom.V3 := C.Board_N;
            Ax : constant Geom.V3 := (if abs (Nb (0)) < abs (Nb (1)) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
            T0 : constant Geom.V3 := [Nb (1) * Ax (2) - Nb (2) * Ax (1), Nb (2) * Ax (0) - Nb (0) * Ax (2), Nb (0) * Ax (1) - Nb (1) * Ax (0)];
            Tn : constant Long_Float := Geom.Norm (T0);
         begin
            return (if Tn > 0.0 then [Dd * T0 (0) / Tn, Dd * T0 (1) / Tn, Dd * T0 (2) / Tn] else [0.0, 0.0, 0.0]);
         end Along;
         procedure Press_Try (K : Natural; Tilt, Azim : Long_Float; Got : out Boolean; S_Ray : out Long_Float) is
         begin
            Press_At (K, Tilt, Azim, [0.0, 0.0, 0.0], Got, S_Ray);
            for Try in 1 .. 2 loop   --  两边(次数)
               exit when Got or else not Limit;
               Geo_Say ("  压到一半在量到的关节限位里解不出来 ⇒ 挪开 " & Mm (Far0) & " 换一处再碰");
               Press_At (K, Tilt, Azim, Along ((if Try = 1 then Far0 else -2.0 * Far0)), Got, S_Ray);   --  第二次挪到另一边(从第一次那儿挪两倍,纯几何)
            end loop;
         end Press_Try;
         --  第 K 瓣按压过的几下解(对准它的几下进解,别的瓣的几下只当"它不许在面之下"核);解出来的尖要落在这一瓣看得见的手指上
         --  (Geom.Finger_View;这一瓣穿过画面、尖在画面外 ⇒ 不核)
         function Fit_Of (K : Natural) return Geom.Press_Fit is
            E : Geom.Press_Eq_Vectors.Vector;
            Through : Boolean;
            Mask : constant Bools := Zone.Lobe_Mask (Z, Zone.Lobe_Of (Z, Lobe_At (K)), Cw, Ch, Through);
         begin
            for I in 0 .. Natural (Eqs.Length) - 1 loop
               E.Append (Geom.Press_Eq'(A => Eqs (I).A, B => Eqs (I).B, Aimed => Eq_Lobe (I) = K));
            end loop;
            return Geom.Fit_Presses (E, Gate, (if Through then Geom.No_View else Geom.Finger_View'(G => G0, W => Cw, H => Ch, Mask => Mask)));
         end Fit_Of;
         function Aimed_At (K : Natural) return Natural is
            N : Natural := 0;
         begin
            for Q of Eq_Lobe loop
               if Q = K then
                  N := N + 1;
               end if;
            end loop;
            return N;
         end Aimed_At;
      begin
         for K in 0 .. Z.N_Lobes - 1 loop
            declare
               Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, K);
               U, V, Wd, Wt : Long_Float;
               Ok : Boolean;
            begin
               Zone.Tip_Section (Z, Lb, Cw, Ch, U, V, Wd, Wt, Ok);
               if Ok and then Z.Valid then
                  declare
                     Dir_Ok : Boolean;
                     Dc : Geom.V3 := Geom.Cam_Dir (G0, U, V, Dir_Ok);   --  相机系单位视线(去掉镜头畸变;去不了 ⇒ 这一瓣不要)
                     Nn : constant Long_Float := Geom.Norm (Dc);
                  begin
                     if Dir_Ok and then Nn > 0.0 then
                        for I in 0 .. 2 loop
                           Dc (I) := Dc (I) / Nn;
                        end loop;
                        D.Append (Dc);
                        Lobe_At.Append (K);
                        Tu.Append (U); Tv.Append (V);
                        Nw.Append (Wd);   --  指尖那一小截的像素跨度(不是整瓣:V1B21 整瓣 124 px 落到面上 90 mm,空的面挑到了半米外)
                        Nt.Append (Wt);
                     else
                        Geo_Say (Who & ":第 " & Codec.Img (K + 1) & " 瓣的尖落在镜头模型够不到的地方 ⇒ 这一瓣不量");
                     end if;
                  end;
               end if;
            end;
         end loop;
         Nl := Natural (D.Length);
         if Nl = 0 then
            Geo_Say (Who & ":它自己眼里没量到手指的尖 ⇒ 指尖量不了(东西躺的面用标定板的)");
            return;
         end if;
         for K in 0 .. Nl - 1 loop
            Est.Append (Geom.V3'[S_Known * D (K) (0), S_Known * D (K) (1), S_Known * D (K) (2)]);
            Has_Est.Append (S_Known > 0.0);
         end loop;
         declare
            Tips : Geom.V3_Vectors.Vector;
            Fits : array (0 .. Nl - 1) of Geom.Press_Fit;
            Failed : Boolean := False;
         begin
            --  没核过的指尖不拿来补转手时的平移:支点一直是眼本身(转的时候每个指尖都在离眼 S 的球面上,不会比眼低 S 以上)
            declare
               G : Geom.Cam_Geo := C.Geo (Hc);
            begin
               G.Tip := [0.0, 0.0, 0.0]; G.Tip_Valid := False;
               C.Geo.Replace_Element (Hc, G);
            end;
            for K in 0 .. Nl - 1 loop
               exit when Failed;
               declare
                  --  斜多少:这一瓣视线和最近的另一瓣视线夹角的三分之一;只有一瓣 ⇒ 这只手一条命令转得到的那一档(开机量的)
                  Theta : constant Long_Float := Geom.Tilt_Angle (D, K, C.Geo (Hc).Stride_Rot);
                  Got : Boolean;
                  S1 : Long_Float;
                  --  尖大概在哪:压到的几下里这一瓣的视线交面离眼最近的那一下(后面几下挑落点、快下多深都按它;解出来以后换成解的)。
                  --  别的东西先顶住只会让手停得更高、交面显得更远,不会更近(Press_Eq 的"只错一边")⇒ 来了近一小步以上的就换成它。
                  --  09-30 V1B75 第 1 只手第 2 瓣:朝下那一下压在约 7.6 cm 高的东西上(交面 3.432 单位,真的约 1.87),原来只信这一下,
                  --  后面几下按它挑落点、在关节限位里解不出,6 下只压成 4 下(其中 3 下交面 1.856–1.889)⇒ 这一瓣量不成
                  procedure Note_Ray (S : Long_Float) is
                  begin
                     if Got and then S > 0.0 and then (not Has_Est (K) or else S + Gate < Geom.Norm (Est (K))) then
                        if Has_Est (K) then
                           Geo_Say ("  这一下视线交面离眼 " & Mm (S) & ",比估的尖(" & Mm (Geom.Norm (Est (K))) & ")近一小步以上 ⇒ 估的尖换成它(停早了只会显得更远)");
                        end if;
                        Est.Replace_Element (K, Geom.V3'[S * D (K) (0), S * D (K) (1), S * D (K) (2)]);
                        Has_Est.Replace_Element (K, True);
                     end if;
                  end Note_Ray;
               begin
                  if Theta <= 0.0 then
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:只有这一瓣、一条命令转得到的那一档也没量 ⇒ 斜不了,这一瓣量不成");
                     Failed := True;
                  else
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:让它指尖的视线朝下压 1 下、再朝五个方位(各差 72°)各斜 " & Codec.Fmt (Theta / Deg, 1)
                              & "° 压 1 下(" & (if Nl >= 2 then "它和最近的另一瓣视线夹角 " & Codec.Fmt (3.0 * Theta / Deg, 1) & "° 的三分之一" else "一条命令转得到的那一档")
                              & ")⇒ 每一下手上最低那一点落在面上,几下一起解它在手系里在哪");
                     --  压之前板上的点在不动的眼里重找一遍:这一瓣只在此刻还找得到的那片桌面上挑落点(09-28 V1B47:手把电子琴推进了板量过的那片)
                     declare
                        Found : Natural;
                        Why : Unbounded_String;
                     begin
                        Board_Recheck (F, C, Found, Why);
                        if Length (Why) = 0 then
                           Geo_Say ("  板上 " & Codec.Img (Natural (C.Board.Length)) & " 个点此刻在不动的眼里还找得到 " & Codec.Img (Found)
                                    & " 个(找不到的当没量过:被挪来的东西盖住了、或者此刻被手挡着)");
                        else
                           Geo_Say ("  板这会儿没法在不动的眼里重找(" & To_String (Why) & ")⇒ 按上一回知道的那份挑落点");
                        end if;
                     end;
                     Press_Try (K, 0.0, 0.0, Got, S1);
                     Note_Ray (S1);
                     if Got and then S1 > 0.0 and then S_Known <= 0.0 then
                        S_Known := S1;
                     end if;
                     for I in 0 .. N_Tilt - 1 loop
                        Press_Try (K, Theta, Long_Float (I) * Az_Step, Got, S1);
                        Note_Ray (S1);
                     end loop;
                     Fits (K) := Fit_Of (K);
                     for E in 0 .. N_Extra - 1 loop
                        exit when Fits (K).Ok;
                        Geo_Say ("  第 " & Codec.Img (K + 1) & " 瓣压了 " & Codec.Img (Aimed_At (K)) & " 下:"
                                 & (if Fits (K).Ambiguous then "有两组一样多、互相对不上(认不出哪一下是坏的)" else "找不到 4 下以上互相对得上的(3 个未知数 + 1 条自己核)")
                                 & " ⇒ 补压一下(方位 " & Codec.Fmt ((0.5 + Long_Float (E)) * Az_Step / Deg, 0) & "°)");
                        Press_Try (K, Theta, (0.5 + Long_Float (E)) * Az_Step, Got, S1);   --  两个方位中间(一半,纯数学)
                        Note_Ray (S1);
                        Fits (K) := Fit_Of (K);
                     end loop;
                     if not Fits (K).Ok then
                        Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:压了 " & Codec.Img (Aimed_At (K)) & " 下,"
                                 & (if Fits (K).Ambiguous then "认不出哪几下是坏的" else "找不到 4 下以上互相对得上的") & " ⇒ 这一瓣量不成");
                        Failed := True;
                     else
                        declare
                           X : constant Geom.V3 := Fits (K).X;
                           Along_Ray : constant Long_Float := Dot (X, D (K));
                           Off_Ray : constant Long_Float := Geom.Norm ([X (0) - Along_Ray * D (K) (0), X (1) - Along_Ray * D (K) (1), X (2) - Along_Ray * D (K) (2)]);
                        begin
                           Est.Replace_Element (K, X);
                           Has_Est.Replace_Element (K, True);
                           Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:压了 " & Codec.Img (Aimed_At (K)) & " 下、" & Codec.Img (Natural (Fits (K).Used.Length))
                                    & " 下互相对得上(别的几下预测每一下最多差 " & Mm (Fits (K).Worst) & ",门 " & Mm (Gate) & ")⇒ 尖离眼 " & Mm (Geom.Norm (X))
                                    & ",离它指尖那条视线 " & Mm (Off_Ray) & " · 不确定度 (" & Mm (Fits (K).Sd (0)) & "," & Mm (Fits (K).Sd (1)) & "," & Mm (Fits (K).Sd (2)) & ")");
                        end;
                     end if;
                  end if;
               end;
            end loop;
            --  每一瓣再按这只手压过的全部几下核一遍:别的瓣压的那几下它也不许在面之下(Fit_Presses 的组外核);对不上 ⇒ 如实说、不收
            if not Failed then
               for K in 0 .. Nl - 1 loop
                  Fits (K) := Fit_Of (K);
                  if not Fits (K).Ok then
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:按全部 " & Codec.Img (Natural (Eqs.Length)) & " 下再核 ⇒ 别的瓣压的时候它落到了面之下(或认不出坏的那一下)⇒ 指尖这回量不成");
                     Failed := True;
                  elsif Geom.Ray_Owner (Fits (K).X, D) /= K then
                     --  解出来的点离别的瓣的视线比离它自己的还近 ⇒ 几下碰着的是那一瓣(它长得多,斜着压时一直是它先碰到)
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:解出来的尖离" & (if Geom.Ray_Owner (Fits (K).X, D) < Nl then "第 " & Codec.Img (Geom.Ray_Owner (Fits (K).X, D) + 1) & " 瓣" else "哪一瓣")
                              & "的视线比离它自己的还近 ⇒ 碰着的不是这一瓣;指尖这回量不成");
                     Failed := True;
                  else
                     Tips.Append (Fits (K).X);
                  end if;
               end loop;
            end if;
            if Failed then
               C.Geo.Replace_Element (Hc, G0);
               Geo_Say (Who & ":指尖这回没量成 ⇒ " & (if G0.Tip_Valid then "身体文件里那份照旧(没核过)" else "没有指尖"));
            else
               declare
                  G : Geom.Cam_Geo := C.Geo (Hc);
                  Tip : Geom.V3 := [0.0, 0.0, 0.0];
               begin
                  for K in 0 .. Nl - 1 loop
                     for I in 0 .. 2 loop
                        Tip (I) := Tip (I) + Tips (K) (I) / Long_Float (Nl);
                     end loop;
                  end loop;
                  G.Tip := Tip; G.Tip_Valid := True; G.Tip_Touch := True;
                  --  每一瓣的尖和尖那一截的截面(像素跨度 × 这一瓣的尖有多深 ÷ 焦距)进身体文件:接触集的手按它们来
                  G.Lobes.Clear; G.Tip_Sd := 0.0;
                  for K in 0 .. Nl - 1 loop
                     declare
                        Dk : constant Long_Float := Long_Float'Max (0.0, -Tips (K) (2));
                     begin
                        G.Lobes.Append (Geom.Lobe_Geo'(Tip => Tips (K), Wide => Nw (K) * Dk / G.F, Thin => Nt (K) * Dk / G.F));
                        for I in 0 .. 2 loop
                           G.Tip_Sd := Long_Float'Max (G.Tip_Sd, Fits (K).Sd (I));
                        end loop;
                     end;
                  end loop;
                  if Nl = 2 then
                     G.Gap := Geom.Norm ([Tips (0) (0) - Tips (1) (0), Tips (0) (1) - Tips (1) (1), Tips (0) (2) - Tips (1) (2)]);
                  end if;
                  C.Geo.Replace_Element (Hc, G);
                  Geom.Save (To_String (C.Geo_Path), C.Geo);
                  declare
                     Say : Unbounded_String := To_Unbounded_String (Who & ":指尖碰桌面量好(换倾角碰,一共压了 " & Codec.Img (Natural (Eqs.Length)) & " 下)—— 每瓣离眼");
                  begin
                     for K in 0 .. Nl - 1 loop
                        Append (Say, " " & Mm (Geom.Norm (Tips (K))));
                     end loop;
                     Append (Say, " · 指尖中点离眼 " & Mm (Geom.Norm (Tip)) & (if Nl = 2 then " · 两指尖相距 " & Mm (G.Gap) else ""));
                     if G0.Tip_Valid then
                        Append (Say, "(身体文件里那份:离眼 " & Mm (Geom.Norm (G0.Tip)) & "、张口 " & Mm (G0.Gap) & ",作废)");
                     end if;
                     Geo_Say (To_String (Say));
                  end;
                  --  每一瓣的尖(眼系,世界单位)落一行:打分脚本按它逐瓣和网格比
                  for K in 0 .. Nl - 1 loop
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣的尖在眼系 (" & Codec.Fmt (Tips (K) (0), 5) & ", " & Codec.Fmt (Tips (K) (1), 5) & ", "
                              & Codec.Fmt (Tips (K) (2), 5) & ")");
                  end loop;
                  Head_Tip_Check (C, A, Hc, Tips);
               end;
            end if;
         end;
         --  回到原处上方(手指这会儿朝下:按原处的高度回去,手指会戳进面里;回到压之前最高的那一处的高度)
         declare
            Hh : constant Long_Float := Home (0) * Nb0 (0) + Home (1) * Nb0 (1) + Home (2) * Nb0 (2);
            Up_By : constant Long_Float := Long_Float'Max (0.0, Top_H - Hh);
         begin
            Go_Back (A, [Home (0) + Up_By * Nb0 (0), Home (1) + Up_By * Nb0 (1), Home (2) + Up_By * Nb0 (2), Home (3), Home (4), Home (5), Home (6)]);
         end;
      end Touch_Tips;

      --  几只手同时碰(2026-09-28 PLAN ⑧ (g)):每只手一个任务照原样做它那一段(Touch_Tips),按拍对齐(Lockstep:同一时刻只有一个线程在跑);
      --  主线程每拍把几只手的目标合成一条关节命令发出去(Plug.Lock_Beat)。V1B50 两只手一只一只碰用了 975 拍
      procedure Touch_Tips_Together (Arms, Cams : Geom.Nat_Vectors.Vector) is
         N : constant Natural := Natural (Arms.Length);
         Seq0 : constant Natural := L.Seq;
         T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
         task type Hand_Task (H, Hc : Natural) with Storage_Size => 64 * 1024 * 1024;
         task body Hand_Task is
         begin
            Lockstep.Begin_Hand (H);
            begin
               Touch_Tips (H, Hc);
            exception
               when E : others =>
                  Put_Line ("[身] 📐 〔手" & Codec.Img (H + 1) & "〕碰指尖这一段出错 ⇒ 这只手这回没有指尖:" & Ada.Exceptions.Exception_Information (E));
            end;
            Lockstep.Done;
         end Hand_Task;
         type Hand_Ref is access Hand_Task;
         Hands : array (0 .. N - 1) of Hand_Ref;
         Ok : Boolean := True;
      begin
         Geo_Say (Codec.Img (N) & " 只手同时碰桌面量指尖:每一拍一条关节命令带几只手的目标,各按各的步子走、各自判碰到(日志里〔手K〕是哪只手说的)");
         Lockstep.Clear;
         Plug.Lock_Begin;
         for I in 0 .. N - 1 loop
            Hands (I) := new Hand_Task (Arms (I), Cams (I));
            Lockstep.Start (Arms (I), Hands (I).all'Identity);
         end loop;
         loop
            declare
               All_Done : Boolean := True;
            begin
               for I in 0 .. N - 1 loop
                  Lockstep.Run (Arms (I));
                  if not Lockstep.Finished (Arms (I)) then
                     All_Done := False;
                  end if;
               end loop;
               exit when All_Done;
            end;
            Plug.Lock_Beat (L, F, Ok);
         end loop;
         Plug.Lock_End;
         Lockstep.Clear;
         Geo_Say (Codec.Img (N) & " 只手同时碰完:" & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒");
      end Touch_Tips_Together;
      Tip_Arms, Tip_Cams : Geom.Nat_Vectors.Vector;   --  要碰桌面量指尖的手、它们各自的眼
   begin
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
            Have : constant Boolean := Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then Natural (Hc) < Natural (F.Cams.Length) and then A < Natural (F.EE.Length)
              and then C.Geo (Natural (Hc)).Valid and then C.Geo (Natural (Hc)).F > 0.0;
         begin
            if Plug.Reset_Pending (L) and then Plug.Take_Reset (L) then
               Geo_Say ("对方复位(新的一集)⇒ 手回了原处,接着摸面");
            end if;
            if not Have then
               Geo_Say ("第" & Codec.Img (A + 1) & " 只手:眼的朝向没量 ⇒ 这只手先不去摸它下面的面");
            elsif C.Board_Plane and then C.Geo (Natural (Hc)).Tip_Valid and then C.Geo (Natural (Hc)).Tip_Touch then
               --  缺什么才量什么:指尖是碰桌面量过的(几何文件里存着)、东西躺的面是板的(随板装回)⇒ 这回不碰
               --  (X5C4 2026-09-26:装回身体干活,开机每瓣碰一次用掉 900 多拍,官方一集只有 200 步)
               Geo_Say ("第" & Codec.Img (A + 1) & " 只手:指尖是碰桌面量过的(离眼 " & Mm (Geom.Norm (C.Geo (Natural (Hc)).Tip)) & "、张口 " & Mm (C.Geo (Natural (Hc)).Gap)
                        & "),桌面是板的 ⇒ 这回不碰");
            elsif C.Board_Plane then
               if A < Lockstep.Max_Hands then
                  Tip_Arms.Append (A); Tip_Cams.Append (Natural (Hc));
               else
                  Touch_Tips (A, Natural (Hc));
               end if;
            else
               --  东西躺的面只有一种量法:开机前半段按板量(09-30 删了"没有板就碰一下、顶住点 = 面"那条后备 —— 一个量两种量法);
               --  没纹理的世界配不出板的点,照实说(PLAN ⑦:改成碰出来的点连面带指尖一起解,那时候就是唯一的量法)
               Geo_Say ("第" & Codec.Img (A + 1) & " 只手:没有标定板量出的面 ⇒ 量不了它下面的面,也就没法碰桌面量指尖(世界里配不出板的点?)");
            end if;
         end;
      end loop;
      if Natural (Tip_Arms.Length) = 1 then
         Touch_Tips (Tip_Arms (0), Tip_Cams (0));
      elsif Natural (Tip_Arms.Length) > 1 then
         Touch_Tips_Together (Tip_Arms, Tip_Cams);
      end if;
   end Geo_Boot_Support;

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
   procedure Geo_Boot_Stride (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
      Rungs : constant array (1 .. 3) of Long_Float := [4.0, 16.0, 64.0];
   begin
      --  转动那一档:每次开机按运动学算(09-28 S1A2:阶梯只推到第三档 0.161 弧度就停,那是阶梯的顶,不是身体的顶 —— 碰指尖时一条命令
      --  转 1.36 弧度;转眼看剪刀要 17 步、67 拍,一集 200 拍)。不动胳膊、不占拍数;平移那一档照旧按阶梯量(脑的"一个单位"按它)
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
            Notch_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
         begin
            if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then A < Natural (F.EE.Length) then
               declare
                  G : Geom.Cam_Geo := C.Geo (Natural (Hc));
                  P0 : constant Plug.Arm_Pose := F.EE (A);
               begin
                  G.Stride_Rot := Kin_Turn_Reach (A, P0, Notch_R, Geo_Base (C, A), Notch_R);
                  C.Geo.Replace_Element (Natural (Hc), G);
                  Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令转得到的最大一档 = " & Codec.Fmt (G.Stride_Rot, 3)
                           & " 弧度(按运动学在量到的关节限位里问反解,不动胳膊;一档转动 " & Codec.Fmt (Notch_R, 4) & " 弧度起翻倍)");
               end;
            end if;
         end;
      end loop;
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
            Amp : constant Long_Float := Geo_Base (C, A);
         begin
            if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then A < Natural (F.EE.Length) and then Amp > 0.0
              and then C.Geo (Natural (Hc)).Stride <= 0.0
            then
               declare
                  G : Geom.Cam_Geo := C.Geo (Natural (Hc));
                  Best : Long_Float := 0.0;
                  Tried : Natural := 0;   --  真试过几档(对方复位打断时一档没试就不许下"走不了路"的结论:V1C/V1E 右臂就是这么被冤枉的)
                  --  先往上探(离桌面远,安全);第一档往上就走不到(手在上限)⇒ 往下探。哪个方向走得到就记哪个
                  Dirs : constant array (1 .. 2) of Long_Float := [1.0, -1.0];
               begin
                  --  (09-27 起不再每停给不动的眼打指尖标记:不动的眼由开机前半段对齐量了,那些标记只给旧的标法用;一笔要合一次爪、约 11 拍)
                  for Dir of Dirs loop
                     exit when Best > 0.0;
                     for R of Rungs loop
                        if Plug.Reset_Pending (L) and then Plug.Take_Reset (L) then
                           Geo_Say ("对方复位(新的一集)⇒ 手回了原处,步幅接着量");
                        end if;
                        declare
                           Ln : constant Long_Float := R * Amp;
                           Av : Table.Vec := Table.Zero_Vec;
                           Jaw : Floats;
                           Del : Table.Vec;
                           Ok : Boolean;
                           Got : Long_Float;
                        begin
                           Av (2) := Dir * Ln;
                           Step_Arm (L, C, F, A, Av, Jaw, Del, Ok, Geo_Settle => True);
                           Got := Dir * Del (2);
                           Tried := Tried + 1;
                           Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令往" & (if Dir > 0.0 then "上 " else "下 ") & Mm (Ln) & " ⇒ 实到 " & Mm (Got));
                           declare
                              Back : Table.Vec := Table.Zero_Vec;
                           begin
                              Back (0) := -Del (0); Back (1) := -Del (1); Back (2) := -Del (2);
                              Step_Arm (L, C, F, A, Back, Jaw, Del, Ok, Geo_Settle => True);
                           end;
                           exit when Got + Got < Ln;
                           Best := Ln;
                        end;
                     end loop;
                  end loop;

                  if Tried = 0 then
                     Geo_Say ("第" & Codec.Img (A + 1) & " 只手:步幅没量成(对方复位打断,一档都没试)⇒ 下次开机再量");
                  else
                     G.Stride := Best;
                     C.Geo.Replace_Element (Natural (Hc), G);
                     Geom.Save (To_String (C.Geo_Path), C.Geo);
                     Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令走得到的最大一档 = " & Mm (Best)
                              & (if Best <= 0.0 then "(上下都走不到 ⇒ 这条臂走不了路)" else "") & ",存进几何文件");
                  end if;
               end;
            end if;
         end;
      end loop;
   end Geo_Boot_Stride;

end Act;
