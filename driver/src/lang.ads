--  身体语言:脑能说的全部句子。除了这些句子,没有第二条通道能让身体动。
--  没有坐标、没有关节号、没有米、没有动作名 —— 那些词【在语法上不存在】,想说也说不出口。
--  五个动词、十种关系、五个停机事件、三档步子。词表小是故意的:要让一个小模型读一遍就会。
--  这里只管【把话变成结构】。一句话能不能成立,是体检和编译层的事。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Lang is

   --  hold  这一条整段保持,不许为了别的被牺牲(硬约束)
   --  reach 朝这个关系走(软目标)
   --  close/open  手
   --  never 不许进入(不等式)
   --  say/look/onfail/done  不动身体的四句
   --  press:在【接触方向】上只说"用多大劲",不说"走到哪"。任务坐标系那条老规矩:
   --  同一根轴上要么说怎么动,要么说多用力,二选一 —— 这里语法上就只给你后者。
   --  这具身体的观测里没有力/电流那一路,所以"劲"= 命令出去而没走到的那一部分(实到差),
   --  它是任何身体都有的量:朝它压过去一个到不了的目标,压不动的那一截就是力。
   type Verb is (V_Hold, V_Reach, V_Press, V_Close, V_Open, V_Never, V_Say, V_Look, V_Onfail, V_Done, V_Bad);
   type Effort is (F_None, F_Light, F_Firm, F_Hard);

   --  关系只描述【画面里和远近上的相对状态】,不描述怎么走
   type Rel is (R_None, R_At, R_Above, R_Below, R_Left, R_Right, R_Nearer, R_Farther, R_Onto, R_Off, R_Facing);

   type Amount is (A_None, A_Small, A_Medium, A_Large);

   --  走到什么为止:走够步数 / 碰到 / 顶住 / 画面不再变 / 它离开原来站的面
   type Event is (E_None, E_Steps, E_Touch, E_Resist, E_Settle, E_Free);

   type Fail_Act is (F_None, F_Retry, F_Stop);

   --  一个名词:要么是画面上的编号,要么是一个名字(名字要身体自己去认;认不出就是编译错)
   type Name is record
      Given : Boolean := False;
      By_Number : Boolean := False;
      Number : Natural := 0;
      Word : Unbounded_String;
   end record;

   type Stmt is record
      V : Verb := V_Bad;
      Subject : Name;              --  谁动
      R : Rel := R_None;
      Object : Name;               --  相对谁
      Amt : Amount := A_None;
      Ev : Event := E_None;
      Steps : Natural := 0;
      Eye : Natural := 0;
      Text : Unbounded_String;     --  say 的内容
      Fa : Fail_Act := F_None;
      Ef : Effort := F_None;
      Together : Boolean := False;  --  这一行前面写了 while ⇒ 和上一条动作【同一节里一起解】,不是排在它后面
      Line : Natural := 0;         --  原文第几行(报错要指得准)
      Src : Unbounded_String;      --  原文这一行
   end record;
   package Stmt_Vectors is new Ada.Containers.Vectors (Natural, Stmt);

   type Program is record
      Stmts : Stmt_Vectors.Vector;
      On_Fail : Fail_Act := F_None;
      Ok : Boolean := True;
      Err : Unbounded_String;      --  人话,而且必须带"那你可以怎么说"
      Err_Line : Natural := 0;
   end record;

   function Parse (Src : String) return Program;

   --  给脑的文法本身(BNF)。论文结论:模型预训练里不可能见全一门 DSL,给【语法】比给散文有效。
   --  这里只许出现语法,不许出现任何"要做某件事就这么说"的示范 —— 那是在替模型决定动作。
   function Grammar return String;

   function Verb_Word (V : Verb) return String;
   function Rel_Word (R : Rel) return String;
   function Amount_Word (A : Amount) return String;
   function Event_Word (E : Event) return String;
   function Effort_Word (E : Effort) return String;
   function Rel_Cn (R : Rel) return String;      --  给日志和报错用的人话
   function Verb_Cn (V : Verb) return String;
   function Event_Cn (E : Event) return String;
   function Unparse (S : Stmt) return String;
   function Moves_Body (V : Verb) return Boolean is (V in V_Hold | V_Reach | V_Close | V_Open | V_Never);
end Lang;
