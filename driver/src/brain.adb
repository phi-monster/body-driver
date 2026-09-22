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

   function Locate (Host : String; Port : Natural; Word : String; RGB : Buf; W, H : Natural;
                    Found : out Boolean; X0, Y0, X1, Y1 : out Natural; Err : out Unbounded_String) return Boolean is
      NL : constant String := "" & ASCII.LF;
      Prompt : constant String :=
        "Locate what someone would call: " & Word & NL &
        "If you can see it in this picture, answer with the box around it. If you cannot see it here, say so - " &
        "that is a normal answer and I will look with another eye rather than guess.";
      --  键名 bbox_2d 是这个模型自己给框时用的那个名字;四个数 = 左、上、右、下,千分比
      Schema : constant String :=
        "{""type"":""json_schema"",""json_schema"":{""name"":""where_is_it"",""strict"":true,""schema"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""found"",""bbox_2d""],""properties"":{""found"":{""type"":""boolean""},""bbox_2d"":{""type"":""array"",""minItems"":4,""maxItems"":4," &
        """items"":{""type"":""integer"",""minimum"":0,""maximum"":1000}}}}}}";
      Reply : Unbounded_String;

      --  千分比(画幅的比例,无量纲)→ 像素(闭区间),并保证左<右、上<下
      procedure To_Pixels (A, B, C2, D2 : Long_Float) is
         function Px (V : Long_Float; Span : Natural) return Natural is
           (Natural'Min (Span - 1, Natural (Long_Float'Floor (Long_Float'Max (0.0, Long_Float'Min (1000.0, V)) / 1000.0 * Long_Float (Span)))));
      begin
         X0 := Px (Long_Float'Min (A, C2), W); X1 := Px (Long_Float'Max (A, C2), W);
         Y0 := Px (Long_Float'Min (B, D2), H); Y1 := Px (Long_Float'Max (B, D2), H);
      end To_Pixels;
   begin
      Found := False;
      X0 := 0; Y0 := 0; X1 := 0; Y1 := 0;
      Err := Null_Unbounded_String;
      if W = 0 or else H = 0 or else Natural (RGB.Length) < W * H * 3 then
         Err := To_Unbounded_String ("画面短了");
         return False;
      end if;
      if Brain_Dir /= "" then
         declare
            A : Unbounded_String;
            D : Json.Doc;
            Perr : Unbounded_String;
         begin
            if not Ask_Human (Brain_Dir, "where", Prompt & NL & NL &
                              "Answer with JSON: {""found"": true, ""bbox_2d"": [left, top, right, bottom]} in thousandths of the picture (0..1000), " &
                              "or {""found"": false, ""bbox_2d"": [0,0,0,0]}.", RGB, W, H, A)
            then
               Err := To_Unbounded_String ("人没回它在哪");
               return False;
            end if;
            if not Json.Parse (To_String (A), D, Perr) then
               Err := To_Unbounded_String ("回的不是 JSON");
               return False;
            end if;
            declare
               Fn : constant Integer := Json.Get (D, 0, "found");
               Bn : constant Integer := Json.Get (D, 0, "bbox_2d");
            begin
               if Fn >= 0 and then Json.Bool (D, Fn) and then Bn >= 0 and then Json.Count (D, Bn) = 4 then
                  To_Pixels (Json.Num (D, Json.Child (D, Bn, 0)), Json.Num (D, Json.Child (D, Bn, 1)),
                             Json.Num (D, Json.Child (D, Bn, 2)), Json.Num (D, Json.Child (D, Bn, 3)));
                  Found := X1 > X0 and then Y1 > Y0;
               end if;
               return True;
            end;
         end;
      end if;
      declare
         B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
         Body_Json : constant String :=
           "{""model"":""eye"",""max_tokens"":80,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false},""response_format"":" & Schema &
           ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & B64 &
           """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      begin
         if not Http_Client.Post (Host, Port, "/v1/chat/completions", Body_Json, Reply) then
            Err := To_Unbounded_String ("连不上脑 " & Host & ":" & Codec.Img (Port));
            return False;
         end if;
      end;
      declare
         Inner : constant String := Extract_Content (To_String (Reply));
         D : Json.Doc;
         Perr : Unbounded_String;
      begin
         if Inner = "" or else not Json.Parse (Inner, D, Perr) then
            Err := To_Unbounded_String ("问它在哪的回包读不出来");
            return False;
         end if;
         declare
            Fn : constant Integer := Json.Get (D, 0, "found");
            Bn : constant Integer := Json.Get (D, 0, "bbox_2d");
         begin
            if Fn >= 0 and then Json.Bool (D, Fn) and then Bn >= 0 and then Json.Count (D, Bn) = 4 then
               To_Pixels (Json.Num (D, Json.Child (D, Bn, 0)), Json.Num (D, Json.Child (D, Bn, 1)),
                          Json.Num (D, Json.Child (D, Bn, 2)), Json.Num (D, Json.Child (D, Bn, 3)));
               Found := X1 > X0 and then Y1 > Y0;
            end if;
         end;
         return True;
      end;
   end Locate;

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
        "Do NOT give distances, angles or speeds - I measure those myself. The only numbers I understand are camera numbers and step counts. " &
        "Things out in the world have no numbers: you point at a thing by calling it what it is, in your own plain words, and I then ask you where in the picture it is. " &
        "If there is a strip of smaller pictures under the numbered one, those are my OTHER eyes right now, each boxed with its camera number in white. " &
        "The bracketed numbers on the pictures are only my own labels for the list above, so that you can tell which box is which. " &
        "The grid of cells belongs to the BIG picture on top only.";
      --  🔴 原来这里是 json_schema + "program": string —— 自由字符串,脑想写什么写什么。
      --  实测 GC9:587 段里 0 段合语法。换成【受限解码】:把驱动当场生成的文法交给解码器,
      --  说不出口的话在 token 层面就打不出来。回包的 content 本身就是程序,不再包一层 JSON。
      B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
      Body_Json : constant String :=
        --  🔴 写程序这一问【不能用温度 0】。CS3 实测:45 段里 43 段第一句一字不差,
        --  全炮只有 3 种开头 —— 那不是 45 个样本,是 1 个样本的 43 份复印件。
        --  它写了一句不动身体的话 ⇒ 世界没变 ⇒ 提示词没变 ⇒ 温度 0 ⇒ 又写同一句,闭环。
        --  温度是【解码器设置】,不是给它的暗示:要判"它会不会想",至少得是独立抽样。
        --  代价照记:同一炮不再逐字可复现(认名字那一问仍然温度 0,那是要稳)。
        "{""model"":""eye"",""max_tokens"":700,""temperature"":0.7,""chat_template_kwargs"":{""enable_thinking"":false}," &
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
