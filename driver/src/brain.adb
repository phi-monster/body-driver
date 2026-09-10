with Ada.Strings.Fixed;
with Codec;
with Json;
with Http_Client;
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
      --  指点用的细格子:比画出来的粗格子每边细四倍。两倍不够 —— 远处的东西在广角相机里
      --  只有二十几个像素宽,半格就有五十多,一格的中心根本落不到东西上(IJ 实测:头顶相机里
      --  没有任何一个细格中心落在球上)。倍数无量纲。
      Fine_C : constant Natural := Cols * 4;
      Fine_R : constant Natural := Rows * 4;
      Fine_N : constant Natural := Fine_C * Fine_R;
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
        "- moves: 0 to 4 entries. Each names WHICH NUMBERED ITEM moves and WHERE: a numbered CELL, or a RELATION to another numbered item (rel = at: touching it / above / below / left / right / front: nearer the camera / back: farther / away: farther from it than now / down: toward the surface things here are standing on / up: away from that surface, with of = that item's number). " &
        "amount = small / medium / large: how far to push this time, as a fraction of what the body measured it can reach. stay_put = true only for an item that must not move (then give it no cell and no rel). " &
        "The body solves all entries together and works out which channels to push from what it measured. An empty list = do not move. The grid lies flat over the picture: nearer/farther from the camera does not change the cell - say front/back for that." & NL &
        "- grip: close / open / none, with grip_arm = which arm (1.." & Codec.Img (N_Arms) & "), and grip_on = the numbered thing to close on (0 = just close or open where the fingers are). " &
        "Closing on a thing means the body itself works out where on that thing to hold it and from which free side, brings that arm's fingers there, closes, and checks whether it is held - you do not describe those steps. This is its own word; a move never implies it." & NL &
        "- until: WHEN to call you back, an EVENT the body measures: steps (after the number in steps, 1..50) / contact (something is touched) / resist (it will not move any further) / slip (the thing stops following me) / settle (the picture stops changing) / free (the thing you named is no longer touching what it was standing on - that is what lifted means)." & NL &
        "- point_at: 0, or a FINE cell number 1.." & Codec.Img (Fine_N) & ". For this one field the picture is divided " & Codec.Img (Fine_C) & " columns by " & Codec.Img (Fine_R) & " rows (finer than the drawn grid), numbered 1.." & Codec.Img (Fine_N) & " left to right then top to bottom, so column c row r is (r-1)*" & Codec.Img (Fine_C) & "+c. Use it when the thing you mean is not in my numbered list: I will take whatever is in that fine cell as a thing, give it a number, and keep following it from then on. Naming what a thing is, is your job; following it and measuring it is mine." & NL &
        "- fast: full steps without pausing. avoid_items: numbered items that must not be touched (may be empty). done: true only when the thing has ALREADY ended up where the task wants it." & NL & NL &
        "Do NOT give distances, angles, speeds or any numbers other than item, cell, camera and step counts - the body measures them. Keep say to ONE short sentence.";
      Schema : constant String :=
        "{""type"":""json_schema"",""json_schema"":{""name"":""what_i_do_now"",""strict"":true,""schema"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""say"",""see"",""look"",""moves"",""grip"",""grip_arm"",""grip_on"",""until"",""steps"",""fast"",""avoid_items"",""done"",""point_at""]," &
        """properties"":{""say"":{""type"":""string""},""see"":{""type"":""string"",""enum"":[""target"",""not_here"",""unclear""]}," &
        """look"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Natural'Max (1, N_Cams)) & "}," &
        """moves"":{""type"":""array"",""minItems"":0,""maxItems"":4,""items"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""item"",""cell"",""rel"",""of"",""amount"",""stay_put""],""properties"":{" &
        """item"":{""type"":""integer"",""minimum"":1,""maximum"":" & Codec.Img (Items) & "}," &
        """cell"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Cells) & "}," &
        """rel"":{""type"":""string"",""enum"":[""none"",""at"",""above"",""below"",""left"",""right"",""front"",""back"",""away"",""down"",""up""]}," &
        """of"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Items) & "}," &
        """amount"":{""type"":""string"",""enum"":[""small"",""medium"",""large""]},""stay_put"":{""type"":""boolean""}}}}," &
        """grip"":{""type"":""string"",""enum"":[""none"",""close"",""open""]}," &
        """grip_arm"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Natural'Max (1, N_Arms)) & "}," &
        """grip_on"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Items) & "}," &
        """point_at"":{""type"":""integer"",""minimum"":0,""maximum"":" & Codec.Img (Fine_N) & "}," &
        """until"":{""type"":""string"",""enum"":[""steps"",""contact"",""resist"",""slip"",""settle"",""free""]}," &
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
         Answer.Point_At := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, 0, "point_at"))));
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
end Brain;
