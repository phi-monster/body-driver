--  编译:把脑交的一段 Sinew 程序,对着【这具身体此刻列出来的东西】整段检查一遍,过了才准跑。
--  这一版接的是 FO 那代执行核,能说的话就是它能做的事;说不了的当场退回,附一句人话 + 一个能照抄的替代。
--  退回是免费的:一根手指都不动。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Sinew;
package Plan is

   --  执行层告诉编译器的、关于每个名词的事实(编号和给脑看的清单一致,1 起;0 号空着)
   type Item_Facts is record
      Exists : Boolean := False;
      Mine : Boolean := False;      --  我身上的部件(只有我自己的部件能被命令去动)
      Grasp : Boolean := False;     --  这一块能合拢并夹住东西(grasper 角色绑在它身上)
      Arm : Natural := 0;
      Stands : Boolean := False;    --  量得出它鼓出它靠着的那个面多少 ⇒ 才谈得上 onto/off/into
      Span : Long_Float := 0.0;     --  这一组能张多开(画幅);grasper 才有
      Size : Long_Float := 0.0;     --  这一块在画面里多大(画幅)
      Label : Unbounded_String;
   end record;
   package Facts_Vectors is new Ada.Containers.Vectors (Natural, Item_Facts);

   --  一个名词落到了哪一块。角色靠量绑定,名字靠身体去认;认不出就是 -1,附"我试过哪些"。
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

   --  整段检查。Own_Eye = 这一段在长在 grasper 那条胳膊上的眼睛里跑:手在那只眼里不动,能说的关系少。
   function Check (P : Sinew.Program; Facts : Facts_Vectors.Vector; B : Bind_Vectors.Vector; Own_Eye : Boolean) return Verdict;
   --  空转:整段程序在自己的状态机上跑一遍,不通电;喂预测的结局,抓停不下来的循环、没 to 过的名字、张不开却要合的。
   function Dry_Run (P : Sinew.Program; Facts : Facts_Vectors.Vector; B : Bind_Vectors.Vector) return Verdict;
   --  这版在这只眼里说得出口的关系(退回时列给脑抄)
   --  写在 until 后面等得到的那些结局(After_Close = 这一节里有没有合手)
   function Waitable_Outcomes (After_Close : Boolean) return String;
   function Usable_Rels (Own_Eye : Boolean) return String;
   function Say (V : Verdict) return String;
end Plan;
