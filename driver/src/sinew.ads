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
   type Rel is (Re_None, Re_Touching, Re_Above, Re_Below, Re_Left, Re_Right,
                Re_Nearer, Re_Farther, Re_Onto, Re_Off, Re_Facing, Re_Clear,
                Re_Still, Re_Press, Re_Close, Re_Open);

   type Step is (Sp_None, Sp_Small, Sp_Medium, Sp_Large);
   type Effort is (Ef_None, Ef_Light, Ef_Firm, Ef_Hard);
   type Rank is (Rk_Prefer, Rk_Must);   --  must 不许被牺牲(走零空间),prefer 可以

   --  ── 结局:控制流唯一能读的东西 ──
   type Outcome is (Oc_None, Oc_Arrived, Oc_Touched, Oc_Stuck, Oc_Slipped,
                    Oc_Lost, Oc_Free, Oc_Timeout, Oc_Refused);

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
     (O in Oc_Stuck | Oc_Slipped | Oc_Lost | Oc_Timeout | Oc_Refused);
end Sinew;
