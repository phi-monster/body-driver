--  Sinew:身体语言(第二版)。设计见 driver/LANGUAGE.md。
--  一段程序 = 若干【区间】。一段区间 = 同时成立的若干约束 + 一个结局。
--  控制流只能读【八个结局词】,传感器的数一个都不许进来 —— 这是它换机体还能跑的原因。
--  语法上不存在:关节号 · 坐标 · 米 · 牛顿 · 秒 · 相机号 · 物体编号 · 手指数目 · 任何机体名字。
--
--  这里只管【把话变成一串指令】。能不能跑是体检和编译层的事。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Sinew is

   --  ── 名词 ──
   --  角色是【量出来绑定】的,不是名字。grasper = 任意一组"推它们会相向靠拢、
   --  中间扫出一片能装东西的区域"的部件 —— 没有手的机体,肘和躯干合拢就是它的 grasper。
   type Role is (Rl_None, Rl_Me, Rl_Grasper, Rl_Pusher);
   type Noun_Kind is (Nk_None, Nk_Role, Nk_Thing, Nk_Place);
   type Noun is record
      K : Noun_Kind := Nk_None;
      R : Role := Rl_None;
      Word : Unbounded_String;      --  外面的东西:一句名字,身体自己去认。绝不是编号。
   end record;

   --  ── 关系 ──
   --  Re_Into = 瞄【进它身子里】:它自己的皮(这块的中位深度)和它站着的那个面,正中间。
   --  两个数都是这块自己量出来的,一个字不提它是什么东西、也不提手上有几根手指
   --  (吸盘瞄进去照样先碰到皮就停)。owner 2026-09-08 判过"身体不许自己挑高低",
   --  所以这件事必须由【脑说得出口】—— 而 touching(皮) / onto(桌面) / press(穿过桌面)
   --  三个词都不是那一层,球就卡在这儿:FO 瞄了皮,夹在球的很偏上处,一合把球撞飞。
   type Rel is (Re_None, Re_Touching, Re_Above, Re_Below, Re_Left, Re_Right,
                Re_Nearer, Re_Farther, Re_Onto, Re_Off, Re_Into, Re_Facing, Re_Clear,
                Re_Still, Re_Press, Re_Close, Re_Open);

   --  ── 用哪只眼睛判这一段 ──
   --  🔴 十二炮里每一炮开头我都在【用手工做这件事】:先发一条只说话的命令把"一集只换一次眼"
   --  烧掉,再靠"在哪台相机里点名"这个副作用把段挪过去 —— 一炮浪费两条命令,而且脆。
   --  真正想说的就一句:「用不长在我这一块上的那只眼睛判这一段」。语言里没这个词(look 被删后没补)。
   --  不用编号(脑说话不说数字);判据是量出来的、任何机体都成立:
   --    Ey_Still  = 我这一块一动,画面变得【最少】的那只 —— 它不长在我身上,所以能看见我在平移
   --    Ey_Moving = 变得【最多】的那只 —— 它长在我这一块上,离得近、看得清,但看不见自己平移
   --  只有一只眼、身上又分不出零件的机体(无人机):Ey_Still 不存在,身体照实说。
   --  🔴 撤回(2026-09-15,owner 当场指出):我一度加过第三只眼 Ey_Ranging,
   --  定义是"长在我【另一条胳膊】上的那只" —— 那是把【这台机器人】当成了所有机体:
   --  一条胳膊的机器没有"另一条",无人机连胳膊都没有。量距离要的从来不是第二条胳膊,
   --  只是【眼睛跟着我动,而且我知道自己动了多远】。所以这个词删掉,测距改成对所有机体都成立的做法。
   type Eye_Pick is (Ey_None, Ey_Still, Ey_Moving);

   type Step is (Sp_None, Sp_Small, Sp_Medium, Sp_Large);
   type Effort is (Ef_None, Ef_Light, Ef_Firm, Ef_Hard);
   type Rank is (Rk_Prefer, Rk_Must);   --  must 不许被牺牲(走零空间),prefer 可以

   --  ── 结局:控制流唯一能读的东西 ──
   --  🔴 Oc_Stalled(JE 2026-09-15 补):身体每次卡住都在说
   --  "for several steps in a row the gap stopped shrinking - I kept going anyway",
   --  而那句话旁边的注释写着"由脑决定还走不走"—— 可脑【没有这个词能问它】,
   --  只能等整段跑完才知道。差距不缩是身体量得出来的事实(No_Progress),
   --  给它一个结局词,脑才真的能决定。停身体的仍然只有脑写的 until。
   type Outcome is (Oc_None, Oc_Arrived, Oc_Touched, Oc_Stuck, Oc_Slipped,
                    Oc_Lost, Oc_Free, Oc_Settled, Oc_Stalled, Oc_Timeout, Oc_Refused);

   type Constraint is record
      Subj, Obj : Noun;
      R : Rel := Re_None;
      Sp : Step := Sp_None;
      Ef : Effort := Ef_None;
      Rk : Rank := Rk_Prefer;
   end record;
   package Constraint_Vectors is new Ada.Containers.Vectors (Natural, Constraint);

   --  ── 指令(把块结构压平成带跳转的一串,好解释、好空转) ──
   type Op is (Op_Interval, Op_Jump, Op_If, Op_Loop, Op_Next, Op_Call, Op_Ret,
               Op_Remember, Op_Say, Op_Done, Op_Try, Op_Endtry);

   type Instr is record
      O : Op := Op_Interval;
      --  Op_Interval
      Cons : Constraint_Vectors.Vector;
      Until_Oc : Outcome := Oc_None;
      Max_Steps : Natural := 0;
      Anyway : Boolean := False;      --  作废身体的一切认知性谨慎:瞎着也走、远也合、顶着也推
      Eye : Eye_Pick := Ey_None;      --  这一段用哪只眼睛判(没写 = 身体自己按量到的挑)
      --  控制
      Cond : Outcome := Oc_None;
      Target : Integer := -1;
      Count : Natural := 0;
      --  Op_Remember / Op_Call
      Name : Unbounded_String;
      Subj : Noun;
      --  Op_Say
      Text : Unbounded_String;
      --  出处
      Line : Natural := 0;
      Src : Unbounded_String;
   end record;
   package Instr_Vectors is new Ada.Containers.Vectors (Natural, Instr);

   type Def is record
      Name : Unbounded_String;
      At_Addr : Natural := 0;
   end record;
   package Def_Vectors is new Ada.Containers.Vectors (Natural, Def);

   type Program is record
      Code : Instr_Vectors.Vector;
      Defs : Def_Vectors.Vector;
      Ok : Boolean := True;
      Err : Unbounded_String;        --  人话,而且必须带"那你可以怎么说"
      Err_Line : Natural := 0;
   end record;

   function Parse (Src : String) return Program;

   --  给脑的文法原文(BNF)。文档和运行时是同一份,不会漂。
   function Grammar return String;
   --  同一份语法的机器可读版(GBNF):交给推理引擎做受限解码,不合语法的词根本采样不到。
   --  内容与 Grammar 逐字对应 —— 词表都从同两个枚举来,改一个必须改另一个(自检看着)。
   function EBNF (Rels_Usable : String) return String;

   function Role_Word (R : Role) return String;
   function Rel_Word (R : Rel) return String;
   function Step_Word (S : Step) return String;
   function Effort_Word (E : Effort) return String;
   function Outcome_Word (O : Outcome) return String;
   function Outcome_Cn (O : Outcome) return String;
   function Rel_Cn (R : Rel) return String;
   function All_Rels return String;
   function All_Outcomes return String;
   function Unparse (I : Instr) return String;
   --  这个结局算不算"这一段没成"(try 用它决定跳不跳)
   function Is_Failure (O : Outcome) return Boolean is
     (O in Oc_Stuck | Oc_Slipped | Oc_Lost | Oc_Stalled | Oc_Timeout | Oc_Refused);
end Sinew;
