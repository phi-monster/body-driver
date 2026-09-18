with Ada.Strings.Fixed;
with Ada.Environment_Variables;
with Ada.Directories;
with Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Sinew;
with Codec;
with Json;
with Http_Client;
package body Brain is

   --  ── 人当脑 ────────────────────────────────────────────────────────────
   --  BL_BRAIN=<目录> ⇒ 不走 HTTP:把问题(和这一帧的画面)写进那个目录,
   --  然后等一个回答文件出现。脑机那头输出的也是文字,所以这条通道就是产品本身的形状:
   --  一个没有身体、只会说话的脑,靠同一套键盘开这具身体。
   function Brain_Dir return String is
     (if Ada.Environment_Variables.Exists ("BL_BRAIN")
      then Ada.Environment_Variables.Value ("BL_BRAIN") else "");

   procedure Write_Text (Path, Text : String) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (F, Ada.Text_IO.Out_File, Path);
      Ada.Text_IO.Put (F, Text);
      Ada.Text_IO.Close (F);
   end Write_Text;

   procedure Write_Bmp (Path : String; RGB : Buf; W, H : Natural) is
      use Ada.Streams.Stream_IO;
      B : constant Buf := Codec.BMP24 (RGB, W, H);
      F : File_Type;
   begin
      Create (F, Out_File, Path);
      for X of B loop
         Character'Write (Stream (F), Character'Val (Natural (X)));
      end loop;
      Close (F);
   end Write_Bmp;

   function Read_All (Path : String) return String is
      F : Ada.Text_IO.File_Type;
      R : Unbounded_String;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Append (R, Ada.Text_IO.Get_Line (F));
         if not Ada.Text_IO.End_Of_File (F) then
            Append (R, ASCII.LF);
         end if;
      end loop;
      Ada.Text_IO.Close (F);
      return To_String (R);
   end Read_All;

   --  问一句,等回答。回答文件出现即读走(读完删掉,免得下一轮拿到旧的)
   function Ask_Human (Dir, Stem, Prompt : String; RGB : Buf; W, H : Natural;
                       Answer : out Unbounded_String) return Boolean is
      Q : constant String := Dir & "/" & Stem & ".txt";
      Pic : constant String := Dir & "/" & Stem & ".bmp";
      A : constant String := Dir & "/" & Stem & "_answer.txt";
   begin
      Answer := Null_Unbounded_String;
      if Ada.Directories.Exists (A) then
         Ada.Directories.Delete_File (A);
      end if;
      Write_Bmp (Pic, RGB, W, H);
      Write_Text (Q, "[写回答请用:写临时文件再 mv 成 " & Stem & "_answer.txt —— 改名是原子的]" & ASCII.LF & ASCII.LF & Prompt);
      loop
         delay 1.0;
         exit when Ada.Directories.Exists (A);
      end loop;
      --  回答一律【写临时文件再改名】(改名是原子的)⇒ 不需要"等它写完"这种睡眠,
      --  也就不需要一个拍出来的秒数。问题文件头一行就把这条写给回答的人。
      Answer := To_Unbounded_String (Read_All (A));
      Ada.Directories.Delete_File (A);
      return Length (Answer) > 0;
   end Ask_Human;

   function Extract_Content (Raw : String) return String is
      Key_S : constant String := """content"":""";
      P : constant Natural := Ada.Strings.Fixed.Index (Raw, Key_S);
      R : Unbounded_String;
      I : Natural;
   begin
      if P = 0 then
         return "";
      end if;
      I := P + Key_S'Length;
      while I <= Raw'Last loop
         case Raw (I) is
            when '"' => return To_String (R);
            when '\' =>
               if I < Raw'Last then
                  I := I + 1;
                  case Raw (I) is
                     when 'n' => Append (R, ASCII.LF);
                     when 't' => Append (R, ASCII.HT);
                     when 'r' => null;
                     when '"' => Append (R, '"');
                     when '\' => Append (R, '\');
                     when others => Append (R, Raw (I));
                  end case;
               end if;
            when others => Append (R, Raw (I));
         end case;
         I := I + 1;
      end loop;
      return To_String (R);
   end Extract_Content;

   function Find (Host : String; Port : Natural; Word, Body_Text : String; N_Items : Natural;
                  RGB : Buf; W, H : Natural; Which : out Natural; Err : out Unbounded_String) return Boolean is
      NL : constant String := "" & ASCII.LF;
      Prompt : constant String :=
        "Everything I can see right now is already cut out and NUMBERED for you, boxed on the picture:" & NL &
        Body_Text & NL & NL &
        "Which one of those numbers is what someone would call: " & Word & NL &
        "Answer with that number. Answer 0 if none of them is that thing, or if two of them look equally like it - " &
        "0 is a normal answer and I will say so plainly rather than guess." & NL &
        "Do not give me coordinates. Only one of the numbers that are already on the picture.";
      Schema : constant String :=
        "{""type"":""json_schema"",""json_schema"":{""name"":""which_one"",""strict"":true,""schema"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""which""],""properties"":{""which"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Natural'Max (1, N_Items)) & "}}}}}";
      B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
      Body_Json : constant String :=
        "{""model"":""eye"",""max_tokens"":80,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false},""response_format"":" & Schema &
        ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & B64 &
        """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      Reply : Unbounded_String;
   begin
      Which := 0;
      Err := Null_Unbounded_String;
      if Brain_Dir /= "" then
         declare
            A : Unbounded_String;
            N : Integer := 0;
         begin
            if not Ask_Human (Brain_Dir, "which", Prompt, RGB, W, H, A) then
               Err := To_Unbounded_String ("人没回哪一块");
               return False;
            end if;
            begin
               N := Integer'Value (Ada.Strings.Fixed.Trim (To_String (A), Ada.Strings.Both));
            exception
               when others => N := 0;
            end;
            Which := Natural (Integer'Max (0, N));
            return True;
         end;
      end if;
      if Natural (RGB.Length) < W * H * 3 then
         Err := To_Unbounded_String ("画面短了");
         return False;
      end if;
      if not Http_Client.Post (Host, Port, "/v1/chat/completions", Body_Json, Reply) then
         Err := To_Unbounded_String ("连不上脑 " & Host & ":" & Codec.Img (Port));
         return False;
      end if;
      declare
         Inner : constant String := Extract_Content (To_String (Reply));
         D : Json.Doc;
         Perr : Unbounded_String;
      begin
         if Inner = "" or else not Json.Parse (Inner, D, Perr) then
            Err := To_Unbounded_String ("认名字的回包读不出来");
            return False;
         end if;
         Which := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "which"))));
         return True;
      end;
   end Find;

   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused : String;
                 Rels_Usable, Roles_Usable, Outs_Usable : String;
                 Cols, Rows, N_Items, N_Cams, N_Arms : Natural; RGB : Buf; W, H : Natural;
                 Program : out Unbounded_String; Err : out Unbounded_String) return Boolean is
      Cells : constant Natural := Cols * Rows;
      NL : constant String := "" & ASCII.LF;
      Prompt : constant String :=
        "You are not a model looking at a picture. You ARE this robot, and you run the loop: nothing moves unless you say so, and you are called back whenever you ask to be. " &
        "This colour image is what you see right now through the camera you chose, with a numbered grid drawn over it: " & Codec.Img (Cols) & " columns x " & Codec.Img (Rows) &
        " rows, numbered 1.." & Codec.Img (Cells) & " left to right then top to bottom, so cell n-" & Codec.Img (Cols) & " is DIRECTLY ABOVE cell n and cell n+1 is directly to its right." & NL & NL &
        "YOUR BODY (measured by yourself: you moved one channel at a time and watched which part of the picture followed):" & NL & Body_Text & NL & NL &
        "WHAT YOU JUST DID AND WHAT HAPPENED:" & NL & Recent & NL & NL &
        "WHAT YOU ARE TRYING TO DO: " & Task_Text & NL & NL &
        (if Refused = "" then ""
         else "I REFUSED YOUR LAST PROGRAM BEFORE ANYTHING MOVED:" & NL & Refused & NL & NL) &
        "ANSWER WITH ONE PROGRAM in my language. This is the whole grammar - there is nothing else I understand:" & NL &
        Grammar & NL & NL &
        "How this works: I read every line of your program and check it against what I have actually measured about myself BEFORE anything moves. " &
        "If a line asks for something I cannot measure or cannot do, I run none of it, and I tell you which line, why, and what I can do instead. " &
        "That refusal is free: no motor turns, nothing gets knocked over, and you may answer again. " &
        "A program can hold several things at once and run several lines in order, so you do not have to be called back after every single push." & NL &
        "hold means that line must not be given up while the rest runs. reach means go that way. never means do not enter that. " &
        "until says when to call me back. Lines run in the order you write them." & NL & NL &
        "Do NOT give distances, angles, speeds or any numbers other than item numbers, camera numbers and step counts - I measure those myself. " &
        "If there is a strip of smaller pictures under the numbered one, those are my OTHER eyes right now, each boxed with its camera number in white. " &
        "Every eye is numbered: an item number means the same thing wherever I say it, and a thing only one eye can see still has a number you can point at. " &
        "The grid of cells belongs to the BIG picture on top only.";
      --  🔴 原来这里是 json_schema + "program": string —— 自由字符串,脑想写什么写什么。
      --  实测 GC9:587 段里 0 段合语法。换成【受限解码】:把驱动当场生成的文法交给解码器,
      --  说不出口的话在 token 层面就打不出来。回包的 content 本身就是程序,不再包一层 JSON。
      B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
      Body_Json : constant String :=
        "{""model"":""eye"",""max_tokens"":700,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false}," &
        """structured_outputs"":{""grammar"":""" & Json.Escape (Sinew.EBNF (Rels_Usable, Roles_Usable, Outs_Usable)) & """}" &
        ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & B64 &
        """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      Reply : Unbounded_String;
   begin
      Program := Null_Unbounded_String;
      Err := Null_Unbounded_String;
      if Brain_Dir /= "" then
         if not Ask_Human (Brain_Dir, "prog", Prompt, RGB, W, H, Program) then
            Err := To_Unbounded_String ("人交上来一段空程序");
            return False;
         end if;
         return True;
      end if;
      pragma Unreferenced (N_Items, N_Cams, N_Arms);
      if Natural (RGB.Length) < W * H * 3 then
         Err := To_Unbounded_String ("画面短了");
         return False;
      end if;
      if not Http_Client.Post (Host, Port, "/v1/chat/completions", Body_Json, Reply) then
         Err := To_Unbounded_String ("连不上脑 " & Host & ":" & Codec.Img (Port));
         return False;
      end if;
      declare
         Inner : constant String := Extract_Content (To_String (Reply));
         D : Json.Doc;
         Perr : Unbounded_String;
      begin
         if Inner = "" then
            Err := To_Unbounded_String ("回包里没有 content(前 200 字:" & Ada.Strings.Fixed.Head (To_String (Reply), 200) & ")");
            return False;
         end if;
         --  受限解码之后 content 不是 JSON 了,这里原来那道 Json.Parse 会把【每一段】程序都毙掉。
         --  受限解码之后 content 本身就是程序(不再包一层 JSON)
         Program := To_Unbounded_String (Inner);
         if Length (Program) = 0 then
            Err := To_Unbounded_String ("脑交上来一段空程序");
            return False;
         end if;
         return True;
      end;
   end Ask;
end Brain;
