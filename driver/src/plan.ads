--  编译:把脑写的一段程序,对着【这具身体此刻的体检判决】检查一遍,过了才给执行层。
--  编译器只问一个问题:这句话要靠的那几个量,身体证明过它们能用吗?没证过 ⇒ 不许跑。
--  报错的读者是模型,不是人 ⇒ 每一条错必须是人话,而且必须附一个【能直接照抄的替代】,
--  否则一个小模型只会把同一版再交一遍。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Lang;
with Exam;
package Plan is

   --  一句话要动到画面里的哪几行
   type Need is array (Exam.Row_Id) of Boolean;
   function Rows_Needed (R : Lang.Rel) return Need;

   --  编译器需要知道的、关于每个名词的事实(谁给它,取决于执行层此刻看见什么)
   type Item_Facts is record
      Exists : Boolean := False;
      Mine : Boolean := False;      --  我身上的零件(只有我自己的零件能被命令去动)
      Grip : Boolean := False;      --  是一只手的握区(只有它能合、能张)
      Arm : Natural := 0;
      Thing_Idx : Integer := -1;    --  它对应体检报告里的第几条(-1 = 这一块还没有量过响应)
      Stands : Boolean := False;
      Jaw_K : Natural := 0;         --  它是这条臂的第几个抓握通道(五指手:哪一根)    --  量得出它"鼓出它站的那个面多少" ⇒ 才谈得上朝那个面压 / 离开那个面
      Label : Unbounded_String;
   end record;
   package Facts_Vectors is new Ada.Containers.Vectors (Natural, Item_Facts);

   type Goal is record
      Line : Natural := 0;
      V : Lang.Verb := Lang.V_Reach;
      R : Lang.Rel := Lang.R_None;
      Subject, Object : Natural := 0;
      Subject_Arm : Natural := 0;
      Subject_Jaw : Natural := 0;   --  合/张点名的是第几个抓握通道   --  谁动的那一块长在第几只手上(合手要知道是哪只手)
      Rows : Need := [others => False];
      Hard : Boolean := False;      --  hold ⇒ 整段保持,不许被牺牲
      Forbid : Boolean := False;    --  never ⇒ 不许进入
      Amt : Lang.Amount := Lang.A_None;
      Ef : Lang.Effort := Lang.F_None;
      Together : Boolean := False;   --  和上一条动作同一节里一起解
      Ev : Lang.Event := Lang.E_None;
      Steps : Natural := 0;
      Needs_Proof : Boolean := False;   --  这一块还没量过响应 ⇒ 编译期无从判,执行器量完必须【当场再判一次】,
                                        --  判不过就中止这一节并把同一句话退回给脑。证明可以晚,但不许没有。
      Src : Unbounded_String;
   end record;
   package Goal_Vectors is new Ada.Containers.Vectors (Natural, Goal);

   type Compiled is record
      Goals : Goal_Vectors.Vector;
      On_Fail : Lang.Fail_Act := Lang.F_None;
      Done : Boolean := False;
      Look : Integer := -1;
      Says : Unbounded_String;
      Ok : Boolean := True;
      Err : Unbounded_String;
      Instead : Unbounded_String;   --  能照抄的替代;空 = 这一条真的没有替代
      Err_Line : Natural := 0;
   end record;

   --  面不再是一个全局开关:能不能说 onto/off/free,取决于【那个东西】量不量得出它鼓出多少。
   function Compile (P : Lang.Program; R : Exam.Report; Facts : Facts_Vectors.Vector) return Compiled;

   --  这具身体此刻【说得出口】的关系有哪些(报错时列给模型抄)
   function Usable_Rels (R : Exam.Report; Thing_Idx : Integer; Surface : Boolean) return String;
   function Report_Text (C : Compiled) return String;
end Plan;
