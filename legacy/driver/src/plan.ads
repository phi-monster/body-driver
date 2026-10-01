--  编译:把脑交的一段 Sinew 程序,对着【这具身体此刻的体检判决】整段检查一遍,过了才准跑。
--  编译器只问一个问题:这句话要靠的那几个量,身体证明过它们能用吗?没证过 ⇒ 一根手指都不许动。
--  报错的读者是模型,所以每一条错必须是人话 + 一个【能直接照抄的替代】。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Bytes;
with Sinew;
with Exam;
package Plan is

   type Need is array (Exam.Row_Id) of Boolean;
   function Rows_Needed (R : Sinew.Rel) return Need;

   --  执行层告诉编译器的、关于每个名词的事实
   type Item_Facts is record
      Exists : Boolean := False;
      Mine : Boolean := False;      --  我身上的部件(只有我自己的部件能被命令去动)
      Grasp : Boolean := False;     --  这一块能合拢并夹住东西(grasper 角色绑在它身上)
      Arm : Natural := 0;
      Jaw_K : Natural := 0;
      Thing_Idx : Integer := -1;    --  它对应体检报告里的第几条(-1 = 这一块还没量过响应)
      Stands : Boolean := False;    --  量得出它鼓出它靠着的那个面多少 ⇒ 才谈得上 onto/off/free
      Span : Long_Float := 0.0;     --  这一组能张多开(画幅);grasper 才有
      Size : Long_Float := 0.0;     --  这一块在画面里多大(画幅)
      Label : Unbounded_String;
   end record;
   package Facts_Vectors is new Ada.Containers.Vectors (Natural, Item_Facts);

   --  一个名词落到了哪一块。角色靠量绑定,名字靠身体去认;认不出就是 -1,附"我看过哪些"。
   type Bind_Entry is record
      Key : Unbounded_String;
      Item : Integer := -1;
      Tried : Unbounded_String;
   end record;
   package Bind_Vectors is new Ada.Containers.Vectors (Natural, Bind_Entry);
   function Key_Of (N : Sinew.Noun) return String;
   --  K 是不是这门语言的一个角色词(me / grasper / pusher …,照 Sinew 的角色表,不另抄一份)
   function Is_Role (K : String) return Boolean;
   function Look_Up (B : Bind_Vectors.Vector; N : Sinew.Noun) return Integer;

   --  ── 名字落到哪一件(大并行 §2 第 15 条,2026-10-01)──────────────────────────────
   --  act-round 的 Bind_Name 和自检里对 S1A1–S1A5 落盘轮次的重放用的是下面这同一段。
   --  名字是脑的话,落到哪一件只有两种证据:
   --    · 量:脑说"它在这一框里",框里那一片由我量 —— 和我已经量到的某一件是同一片像素 ⇒ 就是那一件;
   --    · 字:脑这回写的和它以前写过的某个名字是【同一串字母】(粘在一起、拆开、大小写都不算不同),
   --      或者多出来的字母全是键盘不许名字里单独出现的那几个词(Same_Core)⇒ 是那一件;对得上的不止一件 ⇒ 不猜。
   --  别的一概不算:共用一个词(the red ball / the red cup)、差几个字母、让脑自己说"是哪一件"都量过,都会认错
   --  (10-01 用 S1A1–S1A5 落盘的名字问真 Qwen3.5-9B"它是你起过名的哪一件",能判对错的 33 问错 15:the red ball 认成 the red cup、
   --  cupboard 认成 cup,lift scissors 却认不出是 reach left …withscissors)。差几个字母的由眼来认:同一个模型看着画面,
   --  scisors / scissers / scis sors 都框在剪刀上。
   --  字母 = 名字里的 a–z(大写折成小写);空格、数字、标点不算。
   function Letters (W : String) return String;
   function Same_Name (A, B : String) return Boolean;           --  字母一样,而且不是空的
   --  名字里多出字母的来路(10-01):键盘不许名字里单独出现语言自己的词和 item(Sinew.Name_Forbidden)——
   --  脑想写 pick up the mint green scissors,up 单独打不出,只能粘到旁边成了 upmint。这是今天唯一的来路;
   --  09-30 以前还有两条:一个名字最多三个词、一个词最多 24 个字母(S1A1–S1A5 就是那时跑的:the mint green scissors
   --  四个词写不下,截成了 the mint green;树莓派上那句 pick upthe pinktissueby 也正好是三个词)。
   --  所以只按今天这一条来路认:多出来的每个字母都得能切成那几个词,别的(board、cil、the、pick)一律不算 ——
   --  cupboard 不是 cup,pencil、open 不是 pen,red cupboard 不是 red cup,tissue box 不是 tissue。
   --  A、B 各自去掉头尾粘着的语言词(Forbidden 里的词,空格隔开)以后剩下的字母一样(剩下的不空);
   --  整个名字都切得成语言词的(untildone)没有芯,和谁都不一样
   function Same_Core (A, B : String; Forbidden : String) return Boolean;
   --  同一片像素:两块各自身上的那一点(离形心最近的它自己的像素)都落在对方的像素里。
   --  不设重叠比例,两个方向都要成立;哪一点没有(< 0)或者掩膜不是整幅 ⇒ 不算同一片
   function Same_Pixels (Ma : Bytes.Bools; Ua, Va : Long_Float; Mb : Bytes.Bools; Ub, Vb : Long_Float; W, H : Natural) return Boolean;
   --  这一片是不是我自己(大并行 §2 第 3 条:眼框出来的那一片可能是我自己的胳膊 —— S1A1 R1 眼把我的右臂框成了「arm reach ight」,
   --  从此它是清单上的一件"东西"):它自己身上那一点(离形心最近的它自己的像素,同 Same_Pixels)落在 Self 里 ⇒ 是我。
   --  Self = 这只眼此刻哪些像素是身体自己(路 1 的 Links.Self_Mask_Now:每一节量过的表面点按此刻的读数投进这只眼)。
   --  只问新的一片:和我已经量到的某一件同一片像素的,先认成那一件(拿在手里的东西,它身上那一点也可能落在手指那几个圆里)
   function On_Me (U, V : Long_Float; Self : Bytes.Bools; W, H : Natural) return Boolean;

   --  一件点过名的东西在一只眼里的记录(和 Act.Boxed_Thing 同一个下标,只取判名字要的几样)
   type Named_Record is record
      Name : Unbounded_String;
      Eye : Natural := 0;
      Boxed : Boolean := False;   --  这只眼里有过它的一片框(不是只记了一句"脑说这只眼里指不出它")
      Seen : Boolean := False;    --  这一帧在这只眼里量到了
      Blind : Boolean := False;   --  脑说过"这只眼里指不出它"
   end record;
   package Named_Vectors is new Ada.Containers.Vectors (Natural, Named_Record);
   --  眼转过了(或者脑明确换到这只眼):以前"脑说这只眼里指不出它"的那一条怎么办 ——
   --  这只眼里有过它的框 ⇒ 解除(框还在,下一帧按框重量);从来没有过框 ⇒ 删掉:解除了它就是一条框为 (0,0,0,0) 的"东西",
   --  下一帧在画面左上角量出一块来当它(S1A4 2026-09-27,见 Act.Clear_Blind)
   function Forget_When_Eye_Moves (R : Named_Record) return Boolean is (R.Blind and then not R.Boxed);

   type Name_Verdict_Kind is
     (Nv_Ask_Eye,     --  这只眼此刻没量到叫这个名字的 ⇒ 问这只眼它在哪一框(调用方去问)
      Nv_This,        --  就是 Index 那一条,它在这只眼里、这一帧量到了
      Nv_Elsewhere,   --  是 Index 那一条那件东西,它这一帧在【别的】眼里量到了
      Nv_Not_Seen,    --  是以前那一件(Name),可这一帧哪只眼都没量到它
      Nv_Ambiguous,   --  字对得上的不止一件(Name 里列着)⇒ 不猜
      Nv_Unknown);    --  字对不上以前说过的任何一件
   type Name_Verdict is record
      Kind : Name_Verdict_Kind := Nv_Unknown;
      Index : Integer := -1;     --  Records 里的哪一条(Nv_This / Nv_Elsewhere / Nv_Not_Seen)
      Name : Unbounded_String;   --  那件东西现在叫什么(Nv_Ambiguous:对得上的那几个,「」隔开)
      Known : Unbounded_String;  --  以前说过的名字都有哪些(照实告诉脑用;「」隔开)
   end record;
   --  这只眼给不出它的一片 ⇒ 还问哪几只眼、按什么次序(Bind_Name ②b):别的每一只有画面的眼,按相机的次序。
   --  问眼那句话里说的是"看不见就照实说,我换一只眼看,不猜";以前换眼要等下一轮(这一轮整段退回,回到上次认出名字的那只眼,
   --  脑把同一段话再说一遍),S1A4 在看不见剪刀的腕眼里这样耗了 21 次
   type Eye_Flags is array (Natural range <>) of Boolean;
   function Other_Eyes (Cam : Natural; Has_Picture : Eye_Flags) return Bytes.Ints;
   --  问眼之前:这只眼这一帧量到的东西里,有没有和 W 同一串字母的 ⇒ Nv_This,否则 Nv_Ask_Eye
   function Before_Eye (W : String; Eye : Natural; Records : Named_Vectors.Vector) return Name_Verdict;
   --  眼说这只眼里指不出它(或者没问眼):只按字找它是不是以前说过的哪一件。
   --  同一串字母优先;没有才看 Same_Core(Forbidden = 这一轮键盘的 Sinew.Name_Forbidden);
   --  只认有过框的那几条(只记了"指不出"的不算一件东西)
   function Without_Eye (W : String; Eye : Natural; Records : Named_Vectors.Vector; Forbidden : String) return Name_Verdict;
   --  同一段程序里,一个名字绑没绑上不许取决于它写在第几行:头一遍认的时候后面几行的东西还没进清单。
   --  整段认完一遍以后,头一遍没绑上的东西名字(不是角色)按字再找一次 —— 规则就是 Without_Eye(不再问眼);
   --  找到的换成它此刻在清单上的号(Item_Of:Records 的第几条 → 清单第几号,0 = 不在清单上)。Got = 这一遍绑上了几个
   procedure Rebind_Missing (Binds : in out Bind_Vectors.Vector; Records : Named_Vectors.Vector; Eye : Natural;
                             Forbidden : String;
                             Item_Of : not null access function (Bx : Natural) return Natural;
                             Got : out Natural);
   --  这段程序里用 remember … as <名字> 起的地名(跑到那一行才记下位置):编译期它还不是一处地方,可它是脑自己起的地名,
   --  不是画面里的东西 —— 不许拿去问眼(问了也指不出,整段退回:以前"先 remember 再回来"的写法从来编不过)。
   --  返回起这个名字的那一条 remember 在 P.Code 里的下标(按字母比,同 Same_Name);不是 ⇒ -1
   function Remembered_Here (P : Sinew.Program; W : String) return Integer;

   --  一段收尾时执行器说的那句话 → 结局词(控制流唯一能读的东西)。和 Act.Until_Word 是同一张表的两个方向:
   --  那边是"脑等的是哪个词 ⇒ 执行器听哪句话",这边是"执行器说了哪句话 ⇒ 是哪个词"(自检逐词核对两边对得上)。
   --  以前这张表在 Act.Classify 里,缺了 stall 那一行:以「差距连着几步不缩」收尾的一段被读成 refused ——
   --  if stalled 永远不成立,repeat until stalled 永远出不去,在 try 里还被当成"没成"
   function Outcome_Of_Event (Event : String) return Sinew.Outcome;

   type Verdict is record
      Ok : Boolean := True;
      Err : Unbounded_String;
      Instead : Unbounded_String;
      Err_Line : Natural := 0;
   end record;

   --  整段检查:每一条 Op_Interval 里的每一条约束都过一遍。
   --  Qtys_Usable = 这一轮键盘上列的量词(Round 给的,和键盘同一份):量的那一句只认它们(空 = 这一轮一个量都量不出)
   function Check (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                   B : Bind_Vectors.Vector; Qtys_Usable : String) return Verdict;

   --  🔴 第三道闸:整段程序在【自己量出来的表】上跑一遍,不通电。
   --  它复用同一台执行器,只是喂【预测的结局】而不是真结局 —— 所以循环、分支、try 全都照走。
   --  这一关抓的是真跑才会暴露、而跑一次要几分钟的那些错:
   --    · 循环等一个这段程序里永远不会发生的结局 ⇒ 它会一直转下去
   --    · 一节里两条 must 抢同一行 ⇒ 一定得牺牲一条
   --    · 张不到那么开却要去合它 ⇒ 合了也是空的
   function Dry_Run (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                     B : Bind_Vectors.Vector) return Verdict;

   --  这具身体此刻【说得出口】的关系有哪些(报错时列给模型抄)
   --  写在 until 后面【等得到】的那些结局。和下面的拒绝语共用同一个判定,不许各写一份
   --  (fo 那棵树上就是各写了一份,键盘给了 until lost 而驱动当场退回,27 次退回里 13 次撞这条)。
   function Oc_Waitable (O : Sinew.Outcome; Surface : Boolean) return Boolean;
   function Waitable_Outcomes (Surface : Boolean) return String;

   function Usable_Rels (R : Exam.Report; Thing_Idx : Integer; Surface : Boolean) return String;
   --  键盘用:此刻脑【绑得上】的每一个"我"(Subjects:执行层挑好的那几条 Facts)各按编译器那一套判一遍,取并集。
   --  量过的按体检逐行判;还没量过的 —— 编译器本来就放行(执行器当场量)⇒ 键盘也得给。说明见 plan.adb。
   function Usable_Rels_Any (R : Exam.Report; Subjects : Facts_Vectors.Vector; Surface : Boolean) return String;
   function Say (V : Verdict) return String;
end Plan;
