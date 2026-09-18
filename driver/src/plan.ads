--  编译:把脑交的一段 Sinew 程序,对着【这具身体此刻的体检判决】整段检查一遍,过了才准跑。
--  编译器只问一个问题:这句话要靠的那几个量,身体证明过它们能用吗?没证过 ⇒ 一根手指都不许动。
--  报错的读者是模型,所以每一条错必须是人话 + 一个【能直接照抄的替代】。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
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
   function Look_Up (B : Bind_Vectors.Vector; N : Sinew.Noun) return Integer;

   type Verdict is record
      Ok : Boolean := True;
      Err : Unbounded_String;
      Instead : Unbounded_String;
      Err_Line : Natural := 0;
   end record;

   --  整段检查:每一条 Op_Interval 里的每一条约束都过一遍。
   function Check (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                   B : Bind_Vectors.Vector) return Verdict;

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
   function Say (V : Verdict) return String;
end Plan;
