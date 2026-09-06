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
end Brain;
