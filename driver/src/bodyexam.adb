--  离线出判决书:拿一份存下来的身体文件,不连仿真、不动电机,直接问
--  "这具身体上,哪些量允许被程序引用"。用法:bodyexam <身体文件路径>
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Json;
with Bodyfile;
with Selfmap;
with Zone;
with Learned;
with Schema;
with Exam;
procedure Bodyexam is
   Text : Unbounded_String;
   D : Json.Doc;
   Err, Note : Unbounded_String;
   M : Selfmap.Body_Map;
   Hands : Zone.Hand_Vectors.Vector;
   Tables : Learned.Effect_Vectors.Vector;
   Sch : Schema.Map;
begin
   if Argument_Count < 1 then
      Put_Line ("用法:bodyexam <身体文件路径>");
      Set_Exit_Status (Failure);
      return;
   end if;
   declare
      F : File_Type;
   begin
      Open (F, In_File, Argument (1));
      while not End_Of_File (F) loop
         Append (Text, Get_Line (F));
      end loop;
      Close (F);
   exception
      when others =>
         Put_Line ("读不出 " & Argument (1));
         Set_Exit_Status (Failure);
         return;
   end;
   if not Json.Parse (To_String (Text), D, Err) then
      Put_Line ("身体文件不是合法 JSON:" & To_String (Err));
      Set_Exit_Status (Failure);
      return;
   end if;
   declare
      Kn : constant Integer := Json.Get (D, 0, "key");
      Key : constant String := (if Kn >= 0 then Json.Text (D, Kn) else "");
   begin
      Put_Line ("身体:" & Key);
      if not Bodyfile.Load (Argument (1), Key, M, Hands, Tables, Sch, Note, With_Tables => True) then
         Put_Line ("装不回来:" & To_String (Note));
         Set_Exit_Status (Failure);
         return;
      end if;
   end;
   Put_Line ("装回来了:" & Natural'Image (M.Arms) & " 条臂 ·" & Natural'Image (M.N_Cams)
     & " 台相机 ·" & Natural'Image (M.Channels) & " 个通道 ·"
     & Natural'Image (Natural (Tables.Length)) & " 张响应表");
   Exam.Say (Exam.Judge (M, Tables));
end Bodyexam;
