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
   end record;

   --  🔴 认名字:身体自己把画面切成【带编号的块】,只让模型在这些块里【挑一个】。
   --  这是选择题,不是叫它报坐标 —— 精度来自身体的切块,名字来自模型,各干各擅长的。
   --  挑不出来就回 0,那时候身体如实说"我认不出这个名字",绝不瞎猜一个。
   function Find (Host : String; Port : Natural; Word, Body_Text : String; N_Items : Natural;
                  RGB : Buf; W, H : Natural; Which : out Natural; Err : out Unbounded_String) return Boolean;

   --  Grammar:这门语言的全部文法(BNF)。Refused:上一轮的程序被退回的话,原因 + 能照抄的替代。
   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused : String;
                 Cols, Rows, N_Items, N_Cams, N_Arms : Natural; RGB : Buf; W, H : Natural;
                 Program : out Unbounded_String; Err : out Unbounded_String) return Boolean;
end Brain;
