--  开机量身体:每个通道推一下再推回来,看每台相机里哪片画面跟着动(部件图)、哪台相机长在哪只手上、
--  一条命令实际交付多少、本体读数抖多少、画面抖多少。探针幅度从极小起翻倍,直到走得出来又看得见。
with Bytes; use Bytes;
with Plug;
with Picture;
with Table;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Selfmap is
   type Part is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
      Frac : Long_Float := 0.0;
   end record;
   package Part_Vectors is new Ada.Containers.Vectors (Natural, Part);
   package Floor_Vectors is new Ada.Containers.Vectors (Natural, Picture.Floor_Map, Picture."=");
   type Body_Map is record
      Arms : Natural := 0;
      N_Cams : Natural := 0;
      Per_Arm : Natural := 6;
      Channels : Natural := 0;             --  Arms × Per_Arm
      Parts : Part_Vectors.Vector;         --  (通道 × N_Cams + 相机)
      Cam_Frac : Floats;                   --  (臂 × N_Cams + 相机):这只手一动,那台相机变了多少画面
      Cam_On_Arm : Ints;                   --  每只手:长在它上面的相机号,-1 = 没有
      World_Cam : Natural := 0;            --  变得最少的那台
      Amp : Floats;                        --  每通道:走得出来又看得见的探针幅度
      Delivered : Floats;                  --  每通道:那一幅度实际交付了多少
      Seen : Bools;                        --  每通道:有没有一台相机看见它动
      EE_Noise : Long_Float := 0.0;        --  本体位置读数抖多少(米)
      Rot_Noise : Long_Float := 0.0;       --  本体姿态读数抖多少(弧度)
      Jaw_Noise : Long_Float := 0.0;
      Joint_Noise : Long_Float := 0.0;     --  关节读数不动时抖多少(读数的单位;V1b 2026-09-26:按关节目标挪手时"停稳"看它)

      Floors : Floor_Vectors.Vector;       --  每台相机的静止噪声地板
      Pic_Floor : Ints;                    --  每台相机:整幅画静止时最大灰度差
      Jaws : Ints;                         --  每条臂量到几个抓握通道(五指手 5,两指手 1)
      --  一条命令发出后读数稳下来要几拍(量出来的:Settle_Since,每一次探针各量一回、取最多的;0 = 还没量到。
      --  09-30:原来缺省 2、开机前半段从来不量,后半段又夹在 6 拍以内)
      Settle : Natural := 0;
      --  越用越强:历次量到的幅度/实到(现值取中位数),量过几次
      Amp_Hist : Plug.Floats_Vectors.Vector;
      Deliv_Hist : Plug.Floats_Vectors.Vector;
      Measured_Times : Natural := 0;
   end record;
   --  快速核对:每只手推一个通道(存的幅度),实到和存的差一半以内且画面里看得见 ⇒ 身体没变
   type String_Note is record
      Text : Ada.Strings.Unbounded.Unbounded_String;
   end record;
   procedure Verify (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Ok_Body, Ok_Link : out Boolean; Note : out String_Note);

   --  一步里可以忽略的那一丝 = 这一步的百分之一(比例):Go 判"停了"(一拍挪不到这条命令的百分之一)和"碰到没有"(Blocked)用同一个
   Negligible : constant := 0.01;
   --  往前压的这一步有没有被挡住(纯函数,导出给自检):Short = 这一步沿命令方向少走了多少;Prev / Prev2 = 这一段里前两步空走的少走量,
   --  N_Free = 前面有几步空走的(0 ⇒ 这一步是第一步,没有可比的,判不了 ⇒ False);Lstep = 这一步多大;Noise = 静止读数噪声。
   --  被挡住 = 比上一步空走时多少走的量超过"这一步的百分之一、3 倍读数噪声、3 倍前两步空走之差"三样里最大的那样(倍数无量纲,同踢离群)。
   --  09-29 台架(V1B66 满精度):x5 同样大小的两步空走少走的量前后只差约 1e-5 单位;碰上的第一步多少走至少 0.0014(一档的一成)。
   --  原来的门 = 第一步 + 3 × 静止噪声,仿真读数不抖 ⇒ 门 = 第一步,差一丝就认成碰到(V1B60 虚认 37 次、V1B65 18 次)
   function Blocked (Short, Prev, Prev2 : Long_Float; N_Free : Natural; Lstep, Noise : Long_Float) return Boolean;
   --  发一条位姿命令并等它稳:返回实际交付(按通道)与用掉的拍数。F 更新到最后一帧。
   --  Press:往前压、碰到为止的那一步 —— 停没停只看沿命令平移方向的挪动:动起来以后连着两拍挪不到这一步的百分之一(Negligible)= 停了,
   --  就读;被顶住的软手指那点转动蠕动不算(原来等它慢下来要 10–11 拍);胳膊还在沿这个方向挪(伸远了跟不上、还在往回收)就接着等,
   --  不到固定拍数就读(09-29 V1B65:原来的"快读"到了 5 拍就读,胳膊伸远了还在漂,读到的少走量随时刻乱跳,被当成碰到 18 次)
   --  Watch:走的途中每一拍问一句"出事了没"(被跟的东西快出画面 / 看不见了);说出事就当拍把目标改成"停在这儿",不走完这一步
   --  Group >= 0:这一条发的不是位姿,是第 Group 组关节读数的目标 Joints(开机一个关节一个关节扫,V1b 2026-09-26);
   --  Groups / Qs 非空:同一条命令给几组读数各自的目标(几只手一起扫)。同一条发命令的路(Target / Jaw 这时不用)。
   --  关节目标的"停稳":读数到了目标 Tol 以内再有一拍不动就算到(Tol = 0 不这样判),否则连着两拍不动
   --  位姿目标给了 Tol(平移)/ Tol_Rot(转动)⇒ 位姿到了目标这么近连着两拍就算到;没到 ⇒ 连着两拍每拍挪不到这一档就算停(被顶住 / 到头);
   --  Tol = 0 照旧:连着两拍挪不到读数噪声才算停(V1B21 2026-09-27:位姿读数按关节算,停下以后还有十几微米的蠕动,空中一步要等 13 拍、压到桌面那一步 24 拍)
   type Watcher is access function (F : Plug.Frame) return Boolean;
   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Press : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0; Tol_Rot : Long_Float := 0.0;
                 Tols : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector);
   --  Tols(和 Qs 同形:每组每个关节一道门)给了 ⇒ "到了" = 每个关节差不到它自己那道门(关节目标);"停了"的门照旧按 Tol。
   --  开机扫描用:扫的那根差不到这一格的三分之一,别的关节差不到每根轴单独起步收格子的门(Kinem.Clean_Tol;H1 2026-09-28)
   --  一组关节这一拍"到了没有"(Go 里用的就是它;纯函数,导出给自检):Tols 这一位 > 0 ⇒ 这个关节按它自己的门,否则按 Tol;门 ≤ 0 的关节永远不算到
   function Joints_Arrived (Now, Target, Tols : Floats; Tol : Long_Float) return Boolean;
   --  一条命令从发出到读数停住用了几拍(纯函数,导出给自检):Moves (I) = 发出后第 I + 1 拍读数挪了多少(那一拍挪得最多的那个关节)。
   --  停住 = 动起来以后(挪过超过 Noise 的一拍),第一次"这一拍挪动不超过 Noise、而且不比上一拍小"(不再变小)的那一拍;返回它是第几拍。
   --  一直没动起来(探针小得读数跟不上)/ 看到的那几拍里还在变小(还没停住)⇒ 0:这一条量不出,不算(不拿没停住的拍数顶)
   function Settle_Beats (Moves : Floats; Noise : Long_Float) return Natural;
   --  量 Settle 的唯一办法:帧号 From_Seq(发命令之前那一拍,L.Seq)以后 Plug 逐拍记下的读数(Beats.Q_Chg,每一拍取各组里挪得最多的)⇒ Settle_Beats。
   --  开机前半段认手的每一次探针、后半段逐通道推的每一次(推过去、推回来各一条)都这样量,M.Settle 取量到的最多的
   function Settle_Since (L : Plug.Link; From_Seq : Natural; Noise : Long_Float) return Natural;
   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean);   --  不下命令空等 N 拍
   --  什么都不做时读数抖多少、画面抖多少(静止对,4 拍):位姿 / 姿态 / 抓握 / 关节读数的噪声 + 每台相机的灰度地板。
   --  地板用这几拍里最后一对"两帧都收到了画面"的静止对;一对都没有的那台 ⇒ 地板记成 U8'Last(量不到它的噪声 ⇒ 它的画面里什么都不算动了,
   --  不拿空画面当静止对、不编一个 0 的地板)。
   --  Measure 开头用它;只报关节的身体开机前半段(还没有位姿)也用它(同一种量法)。M.Arms 条臂的位姿噪声(没有位姿 = 0)
   procedure Measure_Idle (L : in out Plug.Link; F : in out Plug.Frame; M : in out Body_Map; Ok : out Boolean);
   --  等到画面连着两拍都不再变(各自的灰度地板以内),最多 Max 拍;返回用了几拍。
   --  Ok = 停稳了。等满 Max 拍还在变 ⇒ Ok = False、Used = Max(照实说没停稳 —— 09-30:原来超时照样 Ok = True,握区在还在动的画面上量);
   --  线断了 ⇒ Ok = False、Used < Max
   --  Prev_Pic 给了 ⇒ 等完时里面是最后一帧之前那一帧(两帧都是画面停下以后的:抓握通道推到头时"看没看见动了"两次比较、不共用一帧用,Picture.Seen_Twice)
   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean;
                         Prev_Pic : access Plug.Cam_Vectors.Vector := null);
   --  每台判得了的相机(有地板、两帧都收到了画面)都静止,而且至少有一台判得了;一台都判不了 ⇒ False(没有证据不说静止)
   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean;
   --  只看第 Cam 台:两帧之间超过噪声地板的像素凑不成一团。两帧里有一帧没收到这台的画面(占位)⇒ False(判不了,不说静止)
   function Picture_Still (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean;
   --  Eyes / World:开机前半段只用关节命令已经认出了"哪台相机长在哪只手上、哪台是世界相机"(Jointboot.Find_Arms)⇒ 照用,
   --  这里不再按位姿探针另认一遍(一个量一种量法);空 = 这里认
   --  Step_Px:每只手"一步看得见"的幅度 = 在它自己那只眼里画面挪 1 像素(V1b 09-27;第 A 个 = [平移, 转动]:平移 = 眼离桌面的距离 ÷ 焦距,
   --  转动 = 1 ÷ 焦距 弧度,开机前半段量的)。每个通道按它推一次再推回来:量走没走到(Delivered)、哪块跟着动(零件)。
   --  原来从极小起翻倍、推过去推回来两张图都变了一块就算看见 —— V1B17 仿真渲染噪声下第 1 只手推 0.0003 单位(约 0.016 mm)就被噪声凑成"看见了"。
   --  没有这一项的手(没量成运动学)⇒ 它的通道量不了
   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean;
                      Step_Px : Plug.Floats_Vectors.Vector;
                      Eyes : Ints := Int_Vectors.Empty_Vector; World : Integer := -1);
   function Jaw_Count (F : Plug.Frame; Arm : Natural) return Natural;   --  这条臂量到几个抓握通道
   --  这一帧里有没有这条臂第 K 个抓握通道的读数
   function Has_Jaw (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Boolean is (K < Jaw_Count (F, Arm));
   --  这条臂第 K 个抓握通道此刻的读数。没读数就不许问(09-30:原来没读数返回 1.0 = x5 夹爪"1 = 张开"的约定,被当成读数、又被当成目标发出去):
   --  先问 Has_Jaw,没有就照实说这一拍没读数
   function Jaw_Of (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Long_Float
     with Pre => Has_Jaw (F, Arm, K);
   function Jaw_All (F : Plug.Frame; Arm : Natural) return Floats;      --  这条臂全部抓握通道此刻的读数
   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural;
end Selfmap;
