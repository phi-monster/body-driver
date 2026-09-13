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

   function Ask (Host : String; Port : Natural; Task_Text, Body_Text, Recent, Grammar, Refused : String;
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
        "If there is a strip of smaller pictures under the numbered one, those are my OTHER eyes right now, each boxed with its camera number in white; " &
        "they carry no grid and no item numbers - the numbered grid and every item number belong to the BIG picture on top only.";
      Schema : constant String :=
        "{""type"":""json_schema"",""json_schema"":{""name"":""my_program"",""strict"":true,""schema"":{""type"":""object"",""additionalProperties"":false," &
        """required"":[""program""],""properties"":{""program"":{""type"":""string""}}}}}";
      B64 : constant String := Codec.Base64 (Codec.BMP24 (RGB, W, H));
      Body_Json : constant String :=
        "{""model"":""eye"",""max_tokens"":700,""temperature"":0,""chat_template_kwargs"":{""enable_thinking"":false},""response_format"":" & Schema &
        ",""messages"":[{""role"":""user"",""content"":[{""type"":""image_url"",""image_url"":{""url"":""data:image/bmp;base64," & B64 &
        """}},{""type"":""text"",""text"":""" & Json.Escape (Prompt) & """}]}]}";
      Reply : Unbounded_String;
   begin
      Program := Null_Unbounded_String;
      Err := Null_Unbounded_String;
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
         if not Json.Parse (Inner, D, Perr) then
            Err := To_Unbounded_String ("脑给的不是 JSON:" & To_String (Perr) & " ‖ " & Ada.Strings.Fixed.Head (Inner, 200));
            return False;
         end if;
         Program := To_Unbounded_String (Json.Text (D, Json.Get (D, 0, "program")));
         if Length (Program) = 0 then
            Err := To_Unbounded_String ("脑交上来一段空程序");
            return False;
         end if;
         return True;
      end;
   end Ask;
end Brain;
