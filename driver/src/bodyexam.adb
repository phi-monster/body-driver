--  离线出判决书:拿一份存下来的身体文件,不连仿真、不动电机,直接问
--  "这具身体上,哪些量允许被程序引用";再给它一段程序,看编译器收不收。
--  用法:bodyexam <身体文件路径> [程序文件]
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
with Lang;
with Plan;
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
   declare
      R : constant Exam.Report := Exam.Judge (M, Tables);
   begin
      Exam.Say (R);
      if Argument_Count >= 2 then
         declare
            Src : Unbounded_String;
            F : File_Type;
            Facts : Plan.Facts_Vectors.Vector;
         begin
            Open (F, In_File, Argument (2));
            while not End_Of_File (F) loop
               Append (Src, Get_Line (F) & ASCII.LF);
            end loop;
            Close (F);
            --  这份档案里量过响应的每一块 = 我身上的一个名词;再加一个"外面的东西"当靶子
            for I in 0 .. Natural (R.Things.Length) - 1 loop
               declare
                  Ft : Plan.Item_Facts;
               begin
                  Ft.Exists := True; Ft.Mine := True; Ft.Grip := True;
                  Ft.Arm := R.Things (I).Arm; Ft.Thing_Idx := Integer (I);
                  Ft.Label := To_Unbounded_String ("我身上量过响应的第 " & Natural'Image (I) & " 块");
                  Facts.Append (Ft);
               end;
            end loop;
            declare
               Ft : Plan.Item_Facts;
            begin
               Ft.Exists := True; Ft.Mine := False; Ft.Thing_Idx := -1; Ft.Stands := True;
               Ft.Label := To_Unbounded_String ("外面的一个东西");
               Facts.Append (Ft);
            end;
            Put_Line ("══ 交上来的程序 ══");
            Put_Line (To_String (Src));
            Put_Line ("══ 编译器 ══");
            Put_Line (Plan.Report_Text (Plan.Compile (Lang.Parse (To_String (Src)), R, Facts)));
            Put_Line ("");
         end;
      end if;
   end;
end Bodyexam;
