with Ada.Strings.Fixed;
with Codec;
with Json;
with Http_Client;
with Sinew;
package body Brain is
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

   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent : String;
                 Cols, Rows, N_Items, N_Cams, N_Arms : Natural; RGB : Buf; W, H : Natural;
                 Answer : out Say; Err : out Unbounded_String) return Boolean is
      Cells : constant Natural := Cols * Rows;
      Items : constant Natural := Natural'Max (1, N_Items);
      NL : constant String := "" & ASCII.LF;
      Prompt : constant String :=
        "You are not a model looking at a picture. You ARE this robot, and you run the loop: nothing moves unless you say so, and you are called back whenever you ask to be. " &
        "This colour image is what you see right now through the camera you chose, with a numbered grid drawn over it: " & Codec.Img (Cols) & " columns x " & Codec.Img (Rows) &
        " rows, numbered 1.." & Codec.Img (Cells) & " left to right then top to bottom, so cell n-" & Codec.Img (Cols) & " is DIRECTLY ABOVE cell n and cell n+1 is directly to its right." & NL & NL &
        "YOUR BODY (measured by yourself: you moved one channel at a time and watched which part of the picture followed):" & NL & Body_Text & NL & NL &
        "WHAT YOU JUST DID AND WHAT HAPPENED:" & NL & Recent & NL & NL &
        "WHAT YOU ARE TRYING TO DO: " & Task_Text & NL & NL &
        "Answer with these fields." & NL &
        "- say: one sentence in your own words: what you see and what you decide." & NL &
        "- see: target = the thing the task refers to is in THIS picture; not_here = it is not in this picture; unclear = you cannot tell. not_here and unclear are normal answers: nothing moves, and you may ask for another camera." & NL &
        "- look: 0 = keep answering about this camera; k = show me camera k next time (cameras are listed under YOUR BODY)." & NL &
        "- moves: 0 to 4 entries. Each names WHICH NUMBERED ITEM moves and WHERE: a numbered CELL, or a RELATION to another numbered item (rel = at: touching it / above / below / left / right / front: nearer the camera / back: farther / away: farther from it than now, with of = that item's number). " &
        "amount = small / medium / large: how far to push this time, as a fraction of what the body measured it can reach. stay_put = true only for an item that must not move (then give it no cell and no rel). " &
        "The body solves all entries together and works out which channels to push from what it measured. An empty list = do not move. The grid lies flat over the picture: nearer/farther from the camera does not change the cell - say front/back for that." & NL &
        "- grip: close / open / none, with grip_arm = which arm (1.." & Codec.Img (N_Arms) & "), and grip_on = the numbered thing to close on (0 = just close or open where the fingers are). " &
        "Closing on a thing means the body itself works out where on that thing to hold it and from which free side, brings that arm's fingers there, closes, and checks whether it is held - you do not describe those steps. This is its own word; a move never implies it." & NL &
        "- until: WHEN to call you back, an EVENT the body measures: steps (after the number in steps, 1..50) / contact (something is touched) / resist (it will not move any further) / slip (the thing stops following me) / settle (the picture stops changing)." & NL &
        "- If there is a strip of smaller pictures under the numbered one: those are my OTHER eyes right now, each boxed with its camera number in white. They carry no grid and no item numbers - the numbered grid and every item number belong to the BIG picture on top only. Use the strip to see what my other eyes see (for example whether one of them is facing a wall) and say look = k if you want that one to become the big numbered picture next turn." & NL &
        "- fast: full steps without pausing. avoid_items: numbered items that must not be touched (may be empty). done: true only when the thing has ALREADY ended up where the task wants it." & NL & NL &
        "Do NOT give distances, angles, speeds or any numbers other than item, cell, camera and step counts - the body measures them. Keep say to ONE short sentence.";
      Schema : constant String :=
        "{""type"":""json_schema"",""json_schema"":{""name"":""what_i_do_now"",""strict"":true,""schema"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""say"",""see"",""look"",""moves"",""grip"",""grip_arm"",""grip_on"",""until"",""steps"",""fast"",""avoid_items"",""done""]," &
        """properties"":{""say"":{""type"":""string""},""see"":{""type"":""string"",""enum"":[""target"",""not_here"",""unclear""]}," &
        """look"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Natural'Max (1, N_Cams)) & "}," &
        """moves"":{""type"":""array"",""minItems"":0,""maxItems"":4,""items"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""item"",""cell"",""rel"",""of"",""amount"",""stay_put""],""properties"":{" &
        """item"":{""type"":""integer"",""minimum"":1,""maximum"":" & Codec.Img (Items) & "}," &
        """cell"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Cells) & "}," &
        """rel"":{""type"":""string"",""enum"":[""none"",""at"",""above"",""below"",""left"",""right"",""front"",""back"",""away""]}," &
        """of"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Items) & "}," &
        """amount"":{""type"":""string"",""enum"":[""small"",""medium"",""large""]},""stay_put"":{""type"":""boolean""}}}}," &
        """grip"":{""type"":""string"",""enum"":[""none"",""close"",""open""]}," &
        """grip_arm"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Natural'Max (1, N_Arms)) & "}," &
        """grip_on"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Items) & "}," &
        """until"":{""type"":""string"",""enum"":[""steps"",""contact"",""resist"",""slip"",""settle""]}," &
        """steps"":{""type"":""integer"",""minimum"":0,""maximum"":50},""fast"":{""type"":""boolean""}," &
        """avoid_items"":{""type"":""array"",""maxItems"":4,""items"":{""type"":""integer"",""minimum"":1,""maximum"":" & Codec.Img (Items) & "}}," &
        """done"":{""type"":""boolean""}}}}}";
      B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
      Body_Json : constant String :=
        "{""model"":""eye"",""max_tokens"":700,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false},""response_format"":" & Schema &
        ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & B64 &
        """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      Reply : Unbounded_String;
   begin
      Answer := (others => <>);
      Err := Null_Unbounded_String;
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
         if not Json.Parse (Inner, D, Perr) then
            Err := To_Unbounded_String ("脑给的不是 JSON:" & To_String (Perr) & " ‖ " & Ada.Strings.Fixed.Head (Inner, 200));
            return False;
         end if;
         Answer.Text := To_Unbounded_String (Json.Text (D, Json.Get (D, 0, "say")));
         Answer.See := To_Unbounded_String (Json.Text (D, Json.Get (D, 0, "see")));
         if Answer.See = "" then
            Answer.See := To_Unbounded_String ("target");
         end if;
         Answer.Look := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "look"))));
         declare
            Mv : constant Integer := Json.Get (D, 0, "moves");
         begin
            for I in 0 .. Natural'Min (4, Json.Count (D, Mv)) - 1 loop
               declare
                  G : constant Integer := Json.Child (D, Mv, I);
                  Gl : Goal;
                  Rel : constant String := Json.Text (D, Json.Get (D, G, "rel"));
               begin
                  Gl.Item := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, G, "item"))));
                  Gl.Cell := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, G, "cell"))));
                  Gl.Of_Item := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, G, "of"))));
                  Gl.Rel := To_Unbounded_String (if Rel = "none" then "" else Rel);
                  Gl.Amount := To_Unbounded_String (Json.Text (D, Json.Get (D, G, "amount")));
                  --  说了地方就是要动;"别动"只在没说任何地方时才算数
                  Gl.Stay := not (Gl.Cell > 0 or else (Gl.Rel /= "" and then Gl.Of_Item > 0))
                             and then Json.Bool (D, Json.Get (D, G, "stay_put"));
                  if Gl.Item = 0 then
                     Err := To_Unbounded_String ("第" & Natural'Image (I + 1) & " 条移动没说动第几号");
                     return False;
                  end if;
                  if not Gl.Stay and then Gl.Cell = 0 and then (Gl.Rel = "" or else Gl.Of_Item = 0) then
                     Err := To_Unbounded_String ("第" & Natural'Image (I + 1) & " 条移动既没说格子、也没说相对哪一号、也没说别动");
                     return False;
                  end if;
                  Answer.Moves.Append (Gl);
               end;
            end loop;
         end;
         declare
            G : constant String := Json.Text (D, Json.Get (D, 0, "grip"));
         begin
            Answer.Grip := To_Unbounded_String (if G = "" then "none" else G);
         end;
         Answer.Grip_Arm := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "grip_arm"))));
         Answer.Grip_On := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "grip_on"))));
         Answer.Until_Kind := To_Unbounded_String (Json.Text (D, Json.Get (D, 0, "until")));
         if Answer.Until_Kind = "" then
            Err := To_Unbounded_String ("脑没给 until");
            return False;
         end if;
         Answer.Steps := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "steps"))));
         Answer.Fast := Json.Bool (D, Json.Get (D, 0, "fast"));
         Answer.Done := Json.Bool (D, Json.Get (D, 0, "done"));
         declare
            Av : constant Integer := Json.Get (D, 0, "avoid_items");
         begin
            for I in 0 .. Json.Count (D, Av) - 1 loop
               declare
                  V : constant Long_Float := Json.Num (D, Json.Child (D, Av, I));
               begin
                  if V >= 1.0 then
                     Answer.Avoid.Append (Integer (V));
                  end if;
               end;
            end loop;
         end;
         return True;
      end;
   end Ask;

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

   function Ask_Prog (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused, Rels_Usable, Roles_Usable, Outs_Usable, Outs_After_Close : String;
                      Cols, Rows : Natural; RGB : Buf; W, H : Natural;
                      Program : out Unbounded_String; Err : out Unbounded_String) return Boolean is
      Cells : constant Natural := Cols * Rows;
      NL : constant String := "" & ASCII.LF;
      Prompt : constant String :=
        "You are not a model looking at a picture. You ARE this robot, and you run the loop: nothing moves unless you say so, and you are called back whenever you ask to be. " &
        "This colour image is what you see right now through the eye I am judging with, with a numbered grid drawn over it: " & Codec.Img (Cols) & " columns x " & Codec.Img (Rows) &
        " rows, numbered 1.." & Codec.Img (Cells) & " left to right then top to bottom." & NL & NL &
        "YOUR BODY (measured by yourself: you moved one channel at a time and watched which part of the picture followed):" & NL & Body_Text & NL & NL &
        "WHAT YOU JUST DID AND WHAT HAPPENED:" & NL & Recent & NL & NL &
        "WHAT YOU ARE TRYING TO DO: " & Task_Text & NL & NL &
        (if Refused = "" then ""
         else "I REFUSED YOUR LAST PROGRAM BEFORE ANYTHING MOVED:" & NL & Refused & NL & NL) &
        "ANSWER WITH ONE PROGRAM in my language. This is the whole grammar - there is nothing else I understand:" & NL &
        Grammar & NL & NL &
        "How this works: I read every line of your program and check it against what I have measured about myself BEFORE anything moves. " &
        "If a line asks for something I cannot do, I run none of it, and I tell you which line, why, and what I can say instead. " &
        "That refusal is free: no motor turns, and you may answer again. " &
        "A program runs one do-line at a time; each do-line ends on its outcome, and try/repeat/if read that outcome, so you are not called back after every push." & NL &
        "Name things in your own words (the ball, the small figure): I will ask you which numbered box that is. " &
        "Do NOT give distances, angles, speeds or any numbers other than step counts - I measure those myself. " &
        "If there is a strip of smaller pictures under the numbered one, those are my OTHER eyes right now, each boxed with its eye number in white; " &
        "they are not numbered inside - to act in one of them say with my still eye / with my moving eye and I will move there and ask you again.";
      --  GC9:这一问原来的约束只有 `{"program": 字符串}` —— 字符串里写什么完全不限,
      --  于是 9B 自己发明动词和格子号,587 段 0 段合语法(而"哪一块"那一问带整数约束,它答得出)。
      --  ⇒ 把同一份语法交给推理引擎做受限解码:不合语法的词根本采样不到。提示词里一个字的教程都不加。
      Body_Json : constant String :=
        "{""model"":""eye"",""max_tokens"":700,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false}," &
        """structured_outputs"":{""grammar"":""" & Json.Escape (Sinew.EBNF (Rels_Usable, Roles_Usable, Outs_Usable, Outs_After_Close)) & """}" &
        ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & Codec.Base64 (Codec.BMP24 (RGB, W, H)) &
        """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      Reply : Unbounded_String;
   begin
      Program := Null_Unbounded_String;
      Err := Null_Unbounded_String;
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
         --  受限解码之后 content 本身就是程序(不再包一层 JSON)
         Program := To_Unbounded_String (Inner);
         if Length (Program) = 0 then
            Err := To_Unbounded_String ("脑交上来一段空程序");
            return False;
         end if;
         return True;
      end;
   end Ask_Prog;
end Brain;
