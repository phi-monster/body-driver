--  脑(模型)拿着循环:它看画面,写一段【身体语言】的程序交上来。
--  这里只管把话送过去、把程序原文读回来。能不能跑不归这里管 —— 那是体检和编译层的事,
--  而且退回来是免费的:一根手指都不动,退回的理由和一个能照抄的替代会随下一轮一起给它。
--  提示词里只讲文法,不讲任何"要做什么就这么说"的示范。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Brain is
   --  下面这两个是【执行器内部】的形状:编译过的程序,一次落一小节到这里。不再是线上格式。
   type Goal is record
      Item : Natural := 0;
      Cell : Natural := 0;
      Rel : Unbounded_String;
      Of_Item : Natural := 0;
      Amount : Unbounded_String;
      Stay : Boolean := False;
      Hard : Boolean := False;     --  hold:这一条整段不许被牺牲(解算时进硬约束)
      Has_Place : Boolean := False;   --  去的是一个【记住的地方】,不是某一块东西
      Pu, Pv, Pz : Long_Float := 0.0;
   end record;
   package Goal_Vectors is new Ada.Containers.Vectors (Natural, Goal);
   type Say is record
      Text, See : Unbounded_String;
      Look : Natural := 0;
      Moves : Goal_Vectors.Vector;
      Grip : Unbounded_String;
      Grip_Arm, Grip_On : Natural := 0;
      Grip_K : Natural := 0;       --  合/张的是这条臂的第几个抓握通道(五指手:点名哪一根)
      Until_Kind : Unbounded_String;
      Steps : Natural := 0;
      Fast, Done : Boolean := False;
      Avoid : Ints;
      --  语言的根(2026-09-23):某件东西(清单第 Qty_Of 件)的量 Qty 往 Qty_Dir(+1 上 / -1 下)变
      Qty : Unbounded_String;
      Qty_Dir : Integer := 0;
      Qty_Of : Natural := 0;
   end record;

   --  🔴 认名字,改问法(2026-09-21):问脑"它在【哪一框】里",不再问"第几号"。
   --  文档早有定案(LAB 08-28):眼给【点】0/10,眼给【框】10 次成 9、23/34,"病因是问法,不是模型弱"。
   --  09-21 离线复量(同一个模型、四炮的原始灰度帧、不画任何编号):在场的东西框准 19/20,剪刀 5/5;
   --  而画上格子和编号框之后同一帧只剩 2/4 —— 我画上去的标记本身在伤它的视力 ⇒ 这一问给【干净】的画面。
   --  分工:脑只说【哪一片】;那一片里哪些像素是它、形心、长轴,由身体自己量(Picture.Measure_In_Box)。
   --  框用千分比(0..1000,这个模型自己的坐标习惯),与画幅大小无关。
   --  Found = False 是正常回答:"这只眼里我指不出它"。实测:自由回答时它对【不在场】的东西只有 9/16 肯说没有;
   --  换成现在这份请求(回答里有一格 found + 那句"说没有是正常回答")同一批帧 14/14 肯说没有、在场的 15/16 指对 0 指错。
   --  仍然只把 Found = False 当"这只眼里没指出来,换只眼再问",不当"世界上没有"(N 小,别当定律)。
   function Locate (Host : String; Port : Natural; Word : String; RGB : Buf; W, H : Natural;
                    Found : out Boolean; X0, Y0, X1, Y1 : out Natural; Err : out Unbounded_String) return Boolean;

   --  Grammar:这门语言的全部文法(BNF)。Refused:上一轮的程序被退回的话,原因 + 能照抄的替代。
   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused : String;
                 Rels_Usable, Roles_Usable, Outs_Usable : String;
                 Cols, Rows, N_Items, N_Cams, N_Arms : Natural; RGB : Buf; W, H : Natural;
                 Program : out Unbounded_String; Err : out Unbounded_String;
                 Qtys_Usable : String := "") return Boolean;
end Brain;
