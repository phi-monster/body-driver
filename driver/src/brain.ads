--  脑(模型)拿着循环:它看带编号的彩图,说"第几号 → 去哪 → 直到什么事件为止",抓握是独立的一个词。
--  这里只管把话送过去、把答案读回来;提示词里只讲格式,不教任何动作。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Brain is
   type Goal is record
      Item : Natural := 0;
      Cell : Natural := 0;
      Rel : Unbounded_String;      --  "" / at / above / below / left / right / front / back / away
      Of_Item : Natural := 0;
      Amount : Unbounded_String;   --  small / medium / large
      Stay : Boolean := False;
   end record;
   package Goal_Vectors is new Ada.Containers.Vectors (Natural, Goal);
   type Say is record
      Text, See : Unbounded_String;
      Look : Natural := 0;
      Moves : Goal_Vectors.Vector;
      Grip : Unbounded_String;     --  none / close / open
      Grip_Arm, Grip_On : Natural := 0;
      Until_Kind : Unbounded_String;   --  steps / contact / resist / slip / settle
      Steps : Natural := 0;
      Fast, Done : Boolean := False;
      Avoid : Ints;
   end record;
   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent : String;
                 Cols, Rows, N_Items, N_Cams, N_Arms : Natural; RGB : Buf; W, H : Natural;
                 Answer : out Say; Err : out Unbounded_String) return Boolean;

   --  🔴 认名字:身体自己把画面切成【带编号的块】,只让模型在这些块里【挑一个】。
   --  这是选择题,不是叫它报坐标 —— 精度来自身体的切块,名字来自模型,各干各擅长的。
   --  挑不出来就回 0,那时候身体如实说"我认不出这个名字",绝不瞎猜一个。
   function Find (Host : String; Port : Natural; Word, Body_Text : String; N_Items : Natural;
                  RGB : Buf; W, H : Natural; Which : out Natural; Err : out Unbounded_String) return Boolean;

   --  脑交一段【身体语言】的程序(Sinew)。Grammar:全部文法(BNF);Refused:上一轮被退回的话(原因 + 能照抄的替代)。
   --  提示词里只讲文法,不讲任何"要做什么就这么说"的示范。
   function Ask_Prog (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused, Rels_Usable, Roles_Usable, Outs_Usable, Outs_After_Close : String;
                      Cols, Rows : Natural; RGB : Buf; W, H : Natural;
                      Program : out Unbounded_String; Err : out Unbounded_String) return Boolean;
end Brain;
