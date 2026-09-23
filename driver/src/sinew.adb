with Ada.Characters.Handling; use Ada.Characters.Handling;
package body Sinew is

   function Role_Word (R : Role) return String is
     (case R is when Rl_None => "", when Rl_Me => "me",
         when Rl_Grasper => "grasper", when Rl_Pusher => "pusher");

   function Rel_Word (R : Rel) return String is
     (case R is
         when Re_None => "", when Re_Touching => "touching", when Re_Above => "above",
         when Re_Below => "below", when Re_Left => "left", when Re_Right => "right",
         when Re_Nearer => "nearer", when Re_Farther => "farther", when Re_Onto => "onto", when Re_Into => "into",
         when Re_Off => "off", when Re_Facing => "facing", when Re_Clear => "clear",
         when Re_Still => "still", when Re_Press => "press", when Re_Close => "close",
         when Re_Open => "open", when Re_Qty => "qty");

   function Rel_Cn (R : Rel) return String is
     (case R is
         when Re_None => "(没说关系)", when Re_Touching => "贴上它", when Re_Above => "在它上面",
         when Re_Below => "在它下面", when Re_Left => "在它左边", when Re_Right => "在它右边",
         when Re_Nearer => "比它更靠近看得最清的那只眼睛", when Re_Farther => "比它更远离那只眼睛",
         when Re_Onto => "朝它靠着的那个面压过去", when Re_Into => "瞄进它身子里(皮和它站的面正中间)", when Re_Off => "离开那个面",
         when Re_Facing => "转到我这一块指着它", when Re_Clear => "不许靠得比这更近",
         when Re_Still => "这一段不许动", when Re_Press => "朝它压,只说劲不说位置",
         when Re_Close => "合拢", when Re_Open => "张开", when Re_Qty => "让它的这个量变");

   --  把"这只眼里说得出口的关系"逐个配上它的含义,一行一个
   function Rel_Gloss (Rels_Usable : String) return String is
      R : Unbounded_String;
      I : Natural := Rels_Usable'First;
      J : Natural;
   begin
      while I <= Rels_Usable'Last loop
         J := I;
         while J <= Rels_Usable'Last and then Rels_Usable (J) /= ' ' loop
            J := J + 1;
         end loop;
         if J > I then
            declare
               W : constant String := Rels_Usable (I .. J - 1);
            begin
               for Rl in Rel loop
                  if Rel_Word (Rl) = W and then Rel_En (Rl) /= "" then
                     Append (R, "              " & W & " = " & Rel_En (Rl) & ASCII.LF);
                  end if;
               end loop;
            end;
         end if;
         I := J + 1;
      end loop;
      return To_String (R);
   end Rel_Gloss;

   function Rel_En (R : Rel) return String is
     (case R is
         when Re_None => "",
         when Re_Touching => "move me until I am against it",
         when Re_Above => "move me until I am above it in the picture",
         when Re_Below => "move me until I am below it in the picture",
         when Re_Left => "move me until I am to the left of it in the picture",
         when Re_Right => "move me until I am to the right of it in the picture",
         when Re_Nearer => "move me nearer to the eye that sees it best",
         when Re_Farther => "move me away from the eye that sees it best (this is how I lift)",
         when Re_Onto => "press me down onto the surface it is resting on",
         when Re_Into => "aim me inside it, halfway between its skin and the surface it stands on",
         when Re_Off => "take me off that surface",
         when Re_Facing => "turn this part of me until it points at it",
         when Re_Clear => "never come closer to it than I am now",
         when Re_Still => "do not move at all during this stretch",
         when Re_Press => "push against it, saying only how hard, not where to go",
         when Re_Close => "close my fingers",
         when Re_Open => "open my fingers",
         when Re_Qty => "change a quantity of that thing (I work out how)");

   function Step_Word (S : Step) return String is
     (case S is when Sp_None => "", when Sp_Small => "small",
         when Sp_Medium => "medium", when Sp_Large => "large");

   function Effort_Word (E : Effort) return String is
     (case E is when Ef_None => "", when Ef_Light => "light",
         when Ef_Firm => "firm", when Ef_Hard => "hard");

   function Outcome_Word (O : Outcome) return String is
     (case O is
         when Oc_None => "", when Oc_Arrived => "arrived", when Oc_Touched => "touched",
         when Oc_Stuck => "stuck", when Oc_Slipped => "slipped", when Oc_Lost => "lost",
         when Oc_Free => "free", when Oc_Settled => "settled",
         when Oc_Stalled => "stalled",
         when Oc_Timeout => "timeout", when Oc_Refused => "refused");

   function Outcome_Cn (O : Outcome) return String is
     (case O is
         when Oc_None => "(没说到什么为止)", when Oc_Arrived => "到了没到只有你能判,这个词我说不出口",
         when Oc_Touched => "碰上了", when Oc_Stuck => "命令了但身体没走",
         when Oc_Slipped => "手里的东西掉了", when Oc_Lost => "看不见我正跟着的东西了",
         when Oc_Free => "它离开了原来靠着的面", when Oc_Settled => "画面不再变了",
         when Oc_Stalled => "我还在动,可差距连着几步不缩了",
         when Oc_Timeout => "步子走完还没到",
         when Oc_Refused => "我做不到");

   function All_Rels return String is
      S : Unbounded_String;
   begin
      for R in Rel loop
         if R /= Re_None then
            Append (S, (if Length (S) > 0 then " " else "") & Rel_Word (R));
         end if;
      end loop;
      return To_String (S);
   end All_Rels;

   function All_Outcomes return String is
      S : Unbounded_String;
   begin
      --  🔴 arrived / refused 不列进语法(owner 2026-09-14):
      --  arrived = "到了没到",只有脑能判;refused = 我做不到时回给脑的话,等不来。
      --  两个都在编译期当场退回并说明,不许悄悄换成步数上限。
      for O in Outcome loop
         if O /= Oc_None and then O /= Oc_Arrived and then O /= Oc_Refused then
            Append (S, (if Length (S) > 0 then " " else "") & Outcome_Word (O));
         end if;
      end loop;
      return To_String (S);
   end All_Outcomes;

   function Lower (S : String) return String is
      R : String := S;
   begin
      for I in R'Range loop
         R (I) := To_Lower (R (I));
      end loop;
      return R;
   end Lower;

   function Is_Digits (S : String) return Boolean is
   begin
      if S'Length = 0 then
         return False;
      end if;
      for C of S loop
         if not Is_Digit (C) then
            return False;
         end if;
      end loop;
      return True;
   end Is_Digits;

   function To_Role (W : String) return Role is
   begin
      for R in Role loop
         if R /= Rl_None and then Role_Word (R) = W then
            return R;
         end if;
      end loop;
      return Rl_None;
   end To_Role;

   function To_Rel (W : String) return Rel is
   begin
      for R in Rel loop
         if R /= Re_None and then Rel_Word (R) = W then
            return R;
         end if;
      end loop;
      return Re_None;
   end To_Rel;

   function To_Step (W : String) return Step is
   begin
      for S in Step loop
         if S /= Sp_None and then Step_Word (S) = W then
            return S;
         end if;
      end loop;
      return Sp_None;
   end To_Step;

   function To_Effort (W : String) return Effort is
   begin
      for E in Effort loop
         if E /= Ef_None and then Effort_Word (E) = W then
            return E;
         end if;
      end loop;
      return Ef_None;
   end To_Effort;

   function To_Outcome (W : String) return Outcome is
   begin
      for O in Outcome loop
         if O /= Oc_None and then Outcome_Word (O) = W then
            return O;
         end if;
      end loop;
      return Oc_None;
   end To_Outcome;

   function Unparse (I : Instr) return String is
      function N (X : Noun) return String is
        (case X.K is when Nk_Role => Role_Word (X.R), when Nk_None => "",
            when others => To_String (X.Word));
      S : Unbounded_String;
   begin
      case I.O is
         when Op_Interval =>
            Append (S, "do");
            for K in 0 .. Natural (I.Cons.Length) - 1 loop
               declare
                  C : constant Constraint := I.Cons (K);
               begin
                  Append (S, (if K > 0 then " and " else " ") & N (C.Subj) & " " & Rel_Word (C.R));
                  if C.Obj.K /= Nk_None then
                     Append (S, " " & N (C.Obj));
                  end if;
                  if C.Ef /= Ef_None then
                     Append (S, " " & Effort_Word (C.Ef));
                  end if;
                  if C.Sp /= Sp_None then
                     Append (S, " " & Step_Word (C.Sp));
                  end if;
                  if C.Rk = Rk_Must then
                     Append (S, " must");
                  end if;
               end;
            end loop;
            if I.Until_Oc /= Oc_None then
               Append (S, " until " & Outcome_Word (I.Until_Oc));
            end if;
            if I.Max_Steps > 0 then
               Append (S, " or" & Natural'Image (I.Max_Steps) & " steps");
            end if;
            if I.Anyway then
               Append (S, " anyway");
            end if;
         when Op_Say => Append (S, "say " & To_String (I.Text));
         when Op_Done => Append (S, "done");
         when Op_Call => Append (S, "run " & To_String (I.Name));
         when Op_Remember => Append (S, "remember where " & N (I.Subj) & " is as " & To_String (I.Name));
         when Op_Jump => Append (S, "<jump" & Integer'Image (I.Target) & ">");
         when Op_If => Append (S, "<if " & Outcome_Word (I.Cond) & " else" & Integer'Image (I.Target) & ">");
         when Op_Loop => Append (S, "<loop" & Natural'Image (I.Count) & " " & Outcome_Word (I.Cond)
                                 & " out" & Integer'Image (I.Target) & ">");
         when Op_Next => Append (S, "<next" & Integer'Image (I.Target) & ">");
         when Op_Try => Append (S, "<try or" & Integer'Image (I.Target) & ">");
         when Op_Endtry => Append (S, "<endtry>");
         when Op_Ret => Append (S, "<return>");
      end case;
      return To_String (S);
   end Unparse;

   function Quoted_List (S : String) return String is
      R : Unbounded_String;
      I : Natural := S'First;
      J : Natural;
   begin
      while I <= S'Last loop
         J := I;
         while J <= S'Last and then S (J) /= ' ' loop
            J := J + 1;
         end loop;
         if J > I then
            if Length (R) > 0 then
               Append (R, " | ");
            end if;
            Append (R, '"' & S (I .. J - 1) & '"');
         end if;
         I := J + 1;
      end loop;
      return To_String (R);
   end Quoted_List;

   function Has_Word (S, W : String) return Boolean is
      I : Natural := S'First;
      J : Natural;
   begin
      while I <= S'Last loop
         J := I;
         while J <= S'Last and then S (J) /= ' ' loop
            J := J + 1;
         end loop;
         if J > I and then S (I .. J - 1) = W then
            return True;
         end if;
         I := J + 1;
      end loop;
      return False;
   end Has_Word;

   function Sufs (Words : String; C : Character) return String is
      S : Unbounded_String;
      I : Natural := Words'First;
      J : Natural;
   begin
      while I <= Words'Last loop
         J := I;
         while J <= Words'Last and then Words (J) /= ' ' loop
            J := J + 1;
         end loop;
         if J > I and then Words (I) = C then
            Append (S, (if Length (S) > 0 then " " else "")
                       & (if J - I = 1 then "." else Words (I + 1 .. J - 1)));
         end if;
         I := J + 1;
      end loop;
      return To_String (S);
   end Sufs;

   function Complement (Words : String; Top : Boolean) return String is
      Free : Unbounded_String;
      Alts : Unbounded_String;
      procedure Add (S : String) is
      begin
         if Length (Alts) > 0 then
            Append (Alts, " | ");
         end if;
         Append (Alts, S);
      end Add;
   begin
      for C in Character range 'a' .. 'z' loop
         if Sufs (Words, C) = "" then
            Append (Free, C);
         end if;
      end loop;
      if Length (Free) > 0 then
         Add ("[" & To_String (Free) & "] ([a-z])*");
      end if;
      for C in Character range 'a' .. 'z' loop
         declare
            Su : constant String := Sufs (Words, C);
         begin
            if Su /= "" then
               Add ("""" & C & """ " & Complement (Su, False));
            end if;
         end;
      end loop;
      return "(" & To_String (Alts) & ")"
             & (if not Top and then not Has_Word (Words, ".") then "?" else "");
   end Complement;

   function Literal_Words (G : String) return String is
      S : Unbounded_String;
      I : Natural := G'First;
   begin
      while I <= G'Last loop
         if G (I) = '"' then
            declare
               J : Natural := I + 1;
               K : Natural;
            begin
               while J <= G'Last and then G (J) /= '"' loop
                  if G (J) in 'a' .. 'z' then
                     K := J;
                     while K <= G'Last and then G (K) in 'a' .. 'z' loop
                        K := K + 1;
                     end loop;
                     if not Has_Word (To_String (S), G (J .. K - 1)) then
                        Append (S, (if Length (S) > 0 then " " else "") & G (J .. K - 1));
                     end if;
                     J := K;
                  else
                     J := J + 1;
                  end if;
               end loop;
               I := J + 1;
            end;
         else
            I := I + 1;
         end if;
      end loop;
      return To_String (S);
   end Literal_Words;

   --  🔴 键盘只给这具身体、这一版真有的键;而且【告诉它有哪些键的那张纸】必须是同一张。
   --  名字用前缀树补集挡掉语言自己的词(GBNF 没有负向断言)——
   --  不挡的话 `do grasper close until touched or and ...` 里「until touched or」会被整个吞成一个名字,
   --  每个 token 都合语法,而真解析器读成完全另一句。
   --  这张表里有没有哪怕一个真的键(小写词)。"(一个都没有)"和空串都算没有。
   function Has_Key (S : String) return Boolean is
   begin
      for Ch of S loop
         if Ch in 'a' .. 'z' then
            return True;
         end if;
      end loop;
      return False;
   end Has_Key;

   function EBNF (Rels_Usable, Roles_Usable, Outs_Usable : String; Qtys_Usable : String := "") return String is
      --  🔴 一条没有任何候选的规则(`who ::= ` 后面是空的)不是"窄的键盘",是【坏掉的语法】:
      --  T1 2026-09-21 实测,脑点了 `with my moving eye`,身体照办换到一只绑不上任何"我"的眼 ⇒ 角色表为空
      --  ⇒ vLLM 原话 "Invalid grammar specification … Expected name at line 8 'who ::= '" ⇒ 这一轮问不了脑,
      --  下一轮还是这只眼、还是问不了 ⇒ 脑再也没机会把眼换回来(第 9 轮起每一轮都是"问不通")。
      --  ⇒ 哪张表空了,就把用到它的产生式整条拿掉,语法永远是合法的:
      --     角色空 ⇒ 这一轮只剩 say / done(说话、换眼都走 say)—— 脑仍然问得到、仍然能说 look = k;
      --     关系空、角色不空 ⇒ 只是没有"<who> <relation> <what>"那一支,close / open / still 还在。
      Has_Who : constant Boolean := Has_Key (Roles_Usable);
      Has_Rel : constant Boolean := Has_Key (Rels_Usable);
      --  🔴 say 后面那一句必须打得出数字和等号:提示词每一轮都印着 "say look = k",而以前这里只许字母、逗号、句号
      --  ⇒ 受限解码下这个键【按不动】(人当脑时不走掩膜所以一直没暴露)。纸上有的键,键盘上必须有。
      --  🔴 自由填的槽要有长度上限(H44 2026-09-23 实测:名字槽里生成了 "untiltimeoutuntiltimeout…" 一整行,say 也会无限重复)。
      --  GBNF 没有 {n,m},用嵌套的可选项写出"最多几个字符":一个名字的词最多 12 个字母,一句话最多 80 个字符。
      function Sent_Tail (N : Natural) return String is
        (if N = 0 then "" else "([a-zA-Z0-9 ,.=\'] " & Sent_Tail (N - 1) & ")?");
      Sent_Rule : constant String := "sent ::= [a-zA-Z] " & Sent_Tail (80);
      function Body_Text (W_Rule : String) return String is
      begin
         if not Has_Who then
            return
              "root ::= line (line)? (line)? (line)?" & ASCII.LF &
              "line ::= word ""\n""" & ASCII.LF &
              "word ::= ""say "" sent | ""done""" & ASCII.LF &
              Sent_Rule;
         end if;
         return
           "root ::= line (line)? (line)? (line)?" & ASCII.LF &
           "line ::= (interval | control | decl | word) ""\n""" & ASCII.LF &
           "simple ::= (interval | decl1 | word) ""\n""" & ASCII.LF &
           "interval ::= ""do "" cons ("" and "" cons)? "" until "" outc ("" or "" num "" steps"")? (eye)?" & ASCII.LF &
           "eye ::= "" with my still eye"" | "" with my moving eye""" & ASCII.LF &
           "cons ::= " & (if Has_Rel then "who "" "" rel "" "" name (step)? | " else "")
                       & "who "" close "" name | who "" open"" | who "" still""" & ASCII.LF &
           "who ::= " & Quoted_List (Roles_Usable) & ASCII.LF &
           (if Has_Rel then "rel ::= " & Quoted_List (Rels_Usable) & ASCII.LF else "") &
           "outcome ::= " & Quoted_List (All_Outcomes) & ASCII.LF &
           "outc ::= " & Quoted_List (Outs_Usable) & ASCII.LF &
           "step ::= "" small"" | "" medium"" | "" large""" & ASCII.LF &
           "num ::= [1-9] ([0-9])?" & ASCII.LF &
           "name ::= w ("" "" w)? ("" "" w)?" & ASCII.LF &
           "w ::= " & W_Rule & ASCII.LF &
           "control ::= ""repeat "" num "" times:\n"" simple (simple)? ""end"" | ""if "" outcome "":\n"" simple (simple)? (""else:\n"" simple (simple)?)? ""end"" | ""try:\n"" simple (simple)? ""or:\n"" simple (simple)? ""end""" & ASCII.LF &
           "decl ::= ""to "" name "":\n"" simple (simple)? ""end"" | decl1" & ASCII.LF &
           "decl1 ::= ""run "" name | ""remember where "" who "" is as "" name" & ASCII.LF &
           "word ::= ""say "" sent | ""done""" & ASCII.LF &
           Sent_Rule;
      end Body_Text;
      Draft : constant String := Body_Text ("[a-z] ([a-z])*");
      --  语言的根(2026-09-23):有可用的量时,键盘上只有这一句 —— <东西> <量> up|down until <结局>,外加 say / done。
      --  手的关系词、眼、步子、控制块全不在键盘上:它们是 9B 的脑乱按的地方(09-22 十七炮里四炮乱码),不是地基。
      function Qty_Text (W_Rule : String) return String is
        ("root ::= line (line)? (line)? (line)?" & ASCII.LF &
         "line ::= (interval | word) ""\n""" & ASCII.LF &
         "interval ::= ""do "" name "" "" qty "" "" dir "" until "" outc" & ASCII.LF &
         "qty ::= " & Quoted_List (Qtys_Usable) & ASCII.LF &
         "dir ::= ""up"" | ""down""" & ASCII.LF &
         "outc ::= " & Quoted_List (Outs_Usable) & ASCII.LF &
         "name ::= w ("" "" w)? ("" "" w)?" & ASCII.LF &
         "w ::= " & W_Rule & ASCII.LF &
         "word ::= ""say "" sent | ""done""" & ASCII.LF &
         Sent_Rule);
      --  把词尾的 "([a-z])*" 换成最多 11 个字母的嵌套可选项(见上)
      function Word_Tail (N : Natural) return String is
        (if N = 0 then "" else "([a-z] " & Word_Tail (N - 1) & ")?");
      function Bound_Tails (G : String) return String is
         Pat : constant String := "([a-z])*";
         R : Unbounded_String;
         I : Natural := G'First;
      begin
         while I <= G'Last loop
            if I + Pat'Length - 1 <= G'Last and then G (I .. I + Pat'Length - 1) = Pat then
               Append (R, Word_Tail (23));   --  一个词最多 24 个字母("mintgreenscissors" 17 个;H45 实测 12 个把它自己的名字截断了)
               I := I + Pat'Length;
            else
               Append (R, G (I));
               I := I + 1;
            end if;
         end loop;
         return To_String (R);
      end Bound_Tails;
   begin
      if Has_Key (Qtys_Usable) then
         declare
            Dq : constant String := Qty_Text ("[a-z] ([a-z])*");
         begin
            return Bound_Tails (Qty_Text (Complement (Literal_Words (Dq) & " item", True)));
         end;
      end if;
      --  名字里也打不出 item:那是我清单上的记账词,不是任何东西的名字(T2 实测 Qwen 拿它当名字用)
      return Bound_Tails (Body_Text (Complement (Literal_Words (Draft) & " item", True)));
   end EBNF;

   --  每个量配一句它是什么(含义来自身体怎么量它,不是说明书)
   function Qty_Gloss (Qtys_Usable : String) return String is
      R : Unbounded_String;
      I : Natural := Qtys_Usable'First;
      J : Natural;
   begin
      while I <= Qtys_Usable'Last loop
         J := I;
         while J <= Qtys_Usable'Last and then Qtys_Usable (J) /= ' ' loop
            J := J + 1;
         end loop;
         if J > I then
            declare
               Wd : constant String := Qtys_Usable (I .. J - 1);
            begin
               Append (R, "              " & Wd & " = "
                       & (if Wd = "height" then "how far the thing is above the surface it lies on (I measure it with my own eyes; up means lift it off that surface)"
                          else "a reading of it I can change")
                       & ASCII.LF);
            end;
         end if;
         I := J + 1;
      end loop;
      return To_String (R);
   end Qty_Gloss;

   function Grammar (Rels_Usable, Roles_Usable, Outs_Usable : String; Qtys_Usable : String := "") return String is
      function Bar (S : String) return String is
         R : Unbounded_String;
         I : Natural := S'First;
         J : Natural;
      begin
         while I <= S'Last loop
            J := I;
            while J <= S'Last and then S (J) /= ' ' loop
               J := J + 1;
            end loop;
            if J > I then
               Append (R, (if Length (R) > 0 then " | " else "") & S (I .. J - 1));
            end if;
            I := J + 1;
         end loop;
         return To_String (R);
      end Bar;
   begin
      --  语言的根(2026-09-23):有可用的量 ⇒ 纸上只印这一句(和 EBNF 同一张纸)
      if Has_Key (Qtys_Usable) then
         return
           "<program>   ::= <line> (up to four lines)" & ASCII.LF &
           "<line>      ::= <interval> | <word>" & ASCII.LF &
           "<interval>  ::= do <what> <quantity> <direction> until <outcome>" & ASCII.LF &
           "<what>      ::= <the thing's name only> (one to three plain words: the name you use when you point the thing out; not an action, not a part of me; it may NOT be any of the words in this grammar, nor the word item)" & ASCII.LF &
           "<quantity>  ::= " & Bar (Qtys_Usable) & "   (a quantity of that thing that I measure myself and can change)" & ASCII.LF &
           Qty_Gloss (Qtys_Usable) &
           "<direction> ::= up | down" & ASCII.LF &
           "<outcome>   ::= " & Bar (Outs_Usable) & ASCII.LF &
           "<word>      ::= say <one sentence in your own words> | done" & ASCII.LF &
           "(You never say where my hand should go or when to close it: given the thing and the quantity, I work out from its shape where to take hold of it, come in from the free side, close, and move it.)";
      end if;
      --  和 EBNF 同一张纸:角色表空了,这一轮能按的键就只有 say / done,纸上也只印这两个,并照实说为什么。
      if not Has_Key (Roles_Usable) then
         return
           "<program>   ::= <line> (up to four lines)" & ASCII.LF &
           "<line>      ::= <word>" & ASCII.LF &
           "<word>      ::= say <one sentence in your own words> | done" & ASCII.LF &
           "(In the eye I am looking through right now I cannot find any part of me that I can command, " &
           "so this turn there is nothing I could be told to move. Speaking still works, and so does changing eyes.)";
      end if;
      return
        "<program>   ::= <line> (up to four lines)" & ASCII.LF &
        "<line>      ::= <interval> | <control> | <decl> | <word>" & ASCII.LF &
        "<interval>  ::= do <constraint> (and <constraint>)? until <outcome> [or <n> steps] [<eye>]" & ASCII.LF &
        "<eye>       ::= with my still eye | with my moving eye" & ASCII.LF &
        (if Has_Key (Rels_Usable)
         then "<constraint>::= <who> <relation> <what> [<step>]" & ASCII.LF &
              "              | <who> close <what> | <who> open | <who> still" & ASCII.LF
         else "<constraint>::= <who> close <what> | <who> open | <who> still" & ASCII.LF) &
        "<who>       ::= " & Bar (Roles_Usable) & "   (roles; I bind them by measuring myself)" & ASCII.LF &
        "<what>      ::= <a name in your words> | <a name you told me to remember> | <who>" & ASCII.LF &
        "              (a name is one to three plain words; it may NOT be any of the words in this grammar, nor the word item - that is only my label for list entries, not a name of anything)" & ASCII.LF &
        --  每个键标上它是干什么的(含义来自驱动自己那张表,不是我写的说明书)
        (if Has_Key (Rels_Usable) then "<relation>  ::= " & Bar (Rels_Usable) & ASCII.LF & Rel_Gloss (Rels_Usable) else "") &
        "<step>      ::= small | medium | large" & ASCII.LF &
        "<outcome>   ::= " & Bar (Outs_Usable) & ASCII.LF &
        "<control>   ::= repeat <n> times: <line> [<line>] end" & ASCII.LF &
        "              | if <outcome>: <line> [<line>] [else: <line> [<line>]] end" & ASCII.LF &
        "              | try: <line> [<line>] or: <line> [<line>] end" & ASCII.LF &
        "<decl>      ::= to <name>: <line> [<line>] end | run <name>" & ASCII.LF &
        "              | remember where <who> is as <name>" & ASCII.LF &
        "<word>      ::= say <one sentence in your own words> | done";
   end Grammar;

   Max_Words : constant := 64;
   type Word_Array is array (1 .. Max_Words) of Unbounded_String;

   function Parse (Src : String) return Program is
      P : Program;
      Line_No : Natural := 0;

      --  开着的块:记住它在哪一条,收到 end / else / or 时回填跳转
      type Block_Kind is (B_Repeat, B_If, B_Else, B_Try, B_Or, B_Def);
      type Open_Block is record
         K : Block_Kind;
         At_Addr : Natural;      --  这个块的头指令
         Patch : Integer := -1;  --  待回填的那一条
      end record;
      Stack : array (1 .. 32) of Open_Block;
      Depth : Natural := 0;

      procedure Fail (Msg : String; Ln : Natural) is
      begin
         if P.Ok then
            P.Ok := False;
            P.Err := To_Unbounded_String (Msg);
            P.Err_Line := Ln;
         end if;
      end Fail;

      function Here return Natural is (Natural (P.Code.Length));

      procedure Emit (I : Instr) is
         X : Instr := I;
      begin
         X.Line := Line_No;
         P.Code.Append (X);
      end Emit;

      procedure Patch_To (Addr : Integer; Target : Natural) is
         X : Instr;
      begin
         if Addr >= 0 and then Addr < Integer (P.Code.Length) then
            X := P.Code (Natural (Addr));
            X.Target := Integer (Target);
            P.Code.Replace_Element (Natural (Addr), X);
         end if;
      end Patch_To;

      procedure Do_Line (Text : String) is
         W : Word_Array;
         N : Natural := 0;
         J : Natural := Text'First;
         Opens_Block : Boolean := False;

         function Lw (K : Natural) return String is
           (if K <= N then Lower (To_String (W (K))) else "");

         --  把 From..To 这几个词拼成一个名字
         function Phrase (From, To : Natural) return Unbounded_String is
            S : Unbounded_String;
         begin
            for K in From .. To loop
               Append (S, (if Length (S) > 0 then " " else "") & To_String (W (K)));
            end loop;
            return S;
         end Phrase;

         function Make_Noun (From, To : Natural) return Noun is
            X : Noun;
            R : constant Role := (if From = To then To_Role (Lw (From)) else Rl_None);
         begin
            if R /= Rl_None then
               X.K := Nk_Role; X.R := R;
            else
               X.K := Nk_Thing; X.Word := Phrase (From, To);
            end if;
            return X;
         end Make_Noun;
      begin
         --  切词;行尾的 ":" 单独算一个词
         while J <= Text'Last loop
            while J <= Text'Last and then (Text (J) = ' ' or else Text (J) = ASCII.HT) loop
               J := J + 1;
            end loop;
            exit when J > Text'Last;
            declare
               K : Natural := J;
            begin
               while K <= Text'Last and then Text (K) /= ' ' and then Text (K) /= ASCII.HT loop
                  K := K + 1;
               end loop;
               if N < Max_Words then
                  N := N + 1;
                  declare
                     Tk : constant String := Text (J .. K - 1);
                  begin
                     if Tk'Length > 1 and then Tk (Tk'Last) = ':' then
                        W (N) := To_Unbounded_String (Tk (Tk'First .. Tk'Last - 1));
                        if N < Max_Words then
                           N := N + 1;
                           W (N) := To_Unbounded_String (":");
                        end if;
                     else
                        W (N) := To_Unbounded_String (Tk);
                     end if;
                  end;
               end if;
               J := K;
            end;
         end loop;
         if N = 0 or else Lw (1) (Lw (1)'First) = '#' then
            return;
         end if;
         Opens_Block := Lw (N) = ":";
         if Opens_Block then
            N := N - 1;
         end if;

         --  ── end / else / or ──
         if Lw (1) = "end" then
            if Depth = 0 then
               Fail ("这里多了一个 end", Line_No);
               return;
            end if;
            declare
               B : constant Open_Block := Stack (Depth);
            begin
               Depth := Depth - 1;
               case B.K is
                  when B_Repeat =>
                     declare
                        I : Instr;
                     begin
                        I.O := Op_Next; I.Target := Integer (B.At_Addr);
                        Emit (I);
                     end;
                     Patch_To (Integer (B.At_Addr), Here);
                  when B_If | B_Else =>
                     Patch_To (B.Patch, Here);
                  when B_Try =>
                     --  try 没有 or 分支:走完就摘掉这一层
                     declare
                        I : Instr;
                     begin
                        I.O := Op_Endtry;
                        Emit (I);
                     end;
                     Patch_To (B.Patch, Here);
                  when B_Or =>
                     Patch_To (B.Patch, Here);
                  when B_Def =>
                     declare
                        I : Instr;
                     begin
                        I.O := Op_Ret;
                        Emit (I);
                     end;
                     Patch_To (B.Patch, Here);
               end case;
            end;
            return;
         end if;

         if Lw (1) = "else" then
            if Depth = 0 or else Stack (Depth).K /= B_If then
               Fail ("else 前面没有一个 if", Line_No);
               return;
            end if;
            declare
               I : Instr;
               B : constant Open_Block := Stack (Depth);
            begin
               I.O := Op_Jump;
               Emit (I);                       --  真分支走完跳过 else
               Patch_To (B.Patch, Here);       --  if 不成立跳到这里
               Stack (Depth) := (K => B_Else, At_Addr => B.At_Addr, Patch => Integer (Here) - 1);
            end;
            return;
         end if;

         if Lw (1) = "or" and then Opens_Block then
            if Depth = 0 or else Stack (Depth).K /= B_Try then
               Fail ("or: 前面没有一个 try:", Line_No);
               return;
            end if;
            declare
               I : Instr;
               B : constant Open_Block := Stack (Depth);
            begin
               I.O := Op_Endtry;
               Emit (I);                       --  try 体顺利走完:先把这一层 try 摘掉
               I := (others => <>);
               I.O := Op_Jump;
               Emit (I);                       --  再跳过 or 那一段
               Patch_To (B.Patch, Here);       --  try 里失败跳到这里(运行时顺手出栈)
               Stack (Depth) := (K => B_Or, At_Addr => B.At_Addr, Patch => Integer (Here) - 1);
            end;
            return;
         end if;

         --  ── 开块的几种 ──
         if Lw (1) = "repeat" then
            declare
               I : Instr;
            begin
               I.O := Op_Loop;
               if N >= 3 and then Is_Digits (To_String (W (2))) and then Lw (3) = "times" then
                  I.Count := Natural'Value (To_String (W (2)));
               elsif N >= 3 and then Lw (2) = "until" then
                  I.Cond := To_Outcome (Lw (3));
                  if I.Cond = Oc_None then
                     Fail ("repeat until 后面「" & Lw (3) & "」不是结局。结局只有:" & All_Outcomes, Line_No);
                     return;
                  end if;
               else
                  Fail ("repeat 后面要么是「<几> times」,要么是「until <结局>」", Line_No);
                  return;
               end if;
               if not Opens_Block then
                  Fail ("repeat 这一行结尾要有冒号", Line_No);
                  return;
               end if;
               Emit (I);
               Depth := Depth + 1;
               Stack (Depth) := (K => B_Repeat, At_Addr => Here - 1, Patch => -1);
            end;
            return;
         end if;

         if Lw (1) = "if" then
            declare
               I : Instr;
            begin
               if N < 2 or else not Opens_Block then
                  Fail ("if 后面要跟一个结局,这一行结尾要有冒号。结局只有:" & All_Outcomes, Line_No);
                  return;
               end if;
               I.O := Op_If; I.Cond := To_Outcome (Lw (2));
               if I.Cond = Oc_None then
                  Fail ("if 后面「" & Lw (2) & "」不是结局。结局只有:" & All_Outcomes, Line_No);
                  return;
               end if;
               Emit (I);
               Depth := Depth + 1;
               Stack (Depth) := (K => B_If, At_Addr => Here - 1, Patch => Integer (Here) - 1);
            end;
            return;
         end if;

         if Lw (1) = "try" then
            declare
               I : Instr;
            begin
               if not Opens_Block then
                  Fail ("try 这一行结尾要有冒号", Line_No);
                  return;
               end if;
               I.O := Op_Try;
               Emit (I);
               Depth := Depth + 1;
               Stack (Depth) := (K => B_Try, At_Addr => Here - 1, Patch => Integer (Here) - 1);
            end;
            return;
         end if;

         if Lw (1) = "to" then
            declare
               I : Instr;
               D : Def;
            begin
               if N < 2 or else not Opens_Block then
                  Fail ("to 后面要跟这段行为的名字,这一行结尾要有冒号", Line_No);
                  return;
               end if;
               I.O := Op_Jump;         --  正常流跳过定义体
               Emit (I);
               D.Name := Phrase (2, N);
               D.At_Addr := Here;
               P.Defs.Append (D);
               Depth := Depth + 1;
               Stack (Depth) := (K => B_Def, At_Addr => Here, Patch => Integer (Here) - 1);
            end;
            return;
         end if;

         --  ── 不开块的几种 ──
         if Lw (1) = "say" then
            declare
               I : Instr;
            begin
               I.O := Op_Say; I.Text := Phrase (2, N);
               Emit (I);
            end;
            return;
         end if;

         if Lw (1) = "done" then
            declare
               I : Instr;
            begin
               I.O := Op_Done;
               Emit (I);
            end;
            return;
         end if;

         if Lw (1) = "run" then
            declare
               I : Instr;
            begin
               if N < 2 then
                  Fail ("run 后面要跟一段行为的名字", Line_No);
                  return;
               end if;
               I.O := Op_Call; I.Name := Phrase (2, N);
               Emit (I);
            end;
            return;
         end if;

         if Lw (1) = "remember" then
            declare
               I : Instr;
               As_At : Natural := 0;
            begin
               for K in 2 .. N loop
                  if Lw (K) = "as" then
                     As_At := K;
                  end if;
               end loop;
               if N < 5 or else Lw (2) /= "where" or else As_At = 0 or else As_At + 1 > N then
                  Fail ("记一个地方要写成:remember where <谁> is as <名字>", Line_No);
                  return;
               end if;
               I.O := Op_Remember;
               I.Subj := Make_Noun (3, (if Lw (As_At - 1) = "is" then As_At - 2 else As_At - 1));
               I.Name := Phrase (As_At + 1, N);
               Emit (I);
            end;
            return;
         end if;

         --  ── 一段区间 ──
         if Lw (1) /= "do" then
            Fail ("第一个词「" & To_String (W (1)) & "」我不认得。一行的开头只能是:"
                  & "do repeat if try to run remember say done end else or", Line_No);
            return;
         end if;
         declare
            I : Instr;
            K : Natural := 2;
         begin
            I.O := Op_Interval;
            loop
               --  一条约束:<谁> <关系> [<什么>] [劲] [步子] [must]
               declare
                  C : Constraint;
                  Rel_At : Natural := 0;
                  Stop : Natural := N;
               begin
                  --  找这一条约束的结束位置(and / until / anyway)
                  for M in K .. N loop
                     if Lw (M) = "and" or else Lw (M) = "until" or else Lw (M) = "anyway" then
                        Stop := M - 1;
                        exit;
                     end if;
                  end loop;
                  --  ── 语言的根(2026-09-23):<东西> <量> up|down ──  一条约束只说某件东西的某个量往哪变
                  if Stop >= K + 2 and then (Lw (Stop) = "up" or else Lw (Stop) = "down") then
                     C.R := Re_Qty;
                     C.Dir := (if Lw (Stop) = "up" then 1 else -1);
                     C.Obj.K := Nk_None; C.Obj.Word := To_Unbounded_String (Lw (Stop - 1));   --  量的名字(身体列的,不绑成东西)
                     C.Subj := Make_Noun (K, Stop - 2);
                     if C.Subj.K /= Nk_Thing then
                        Fail ("「" & Lw (Stop - 1) & " " & Lw (Stop) & "」前面要说的是【哪件东西】(你给它起的名字)", Line_No);
                        return;
                     end if;
                     I.Cons.Append (C);
                     K := Stop + 1;
                     goto Next_Cons;
                  end if;
                  for M in K .. Stop loop
                     if To_Rel (Lw (M)) /= Re_None then
                        Rel_At := M;
                        exit;
                     end if;
                  end loop;
                  if Rel_At = 0 then
                     Fail ("这一条没说关系。关系只有:" & All_Rels, Line_No);
                     return;
                  end if;
                  if Rel_At = K then
                     Fail ("「" & Lw (Rel_At) & "」前面要先说【谁】(me / grasper / pusher,"
                           & "或者一个我记过的地方)", Line_No);
                     return;
                  end if;
                  C.Subj := Make_Noun (K, Rel_At - 1);
                  C.R := To_Rel (Lw (Rel_At));
                  --  关系之后:先剥掉尾巴上的 must / 步子 / 劲
                  declare
                     Tail : Natural := Stop;
                  begin
                     if Tail >= Rel_At + 1 and then Lw (Tail) = "must" then
                        C.Rk := Rk_Must; Tail := Tail - 1;
                     end if;
                     if Tail >= Rel_At + 1 and then To_Step (Lw (Tail)) /= Sp_None then
                        C.Sp := To_Step (Lw (Tail)); Tail := Tail - 1;
                     end if;
                     if Tail >= Rel_At + 1 and then To_Effort (Lw (Tail)) /= Ef_None then
                        C.Ef := To_Effort (Lw (Tail)); Tail := Tail - 1;
                     end if;
                     --  "on" 只是个连接词
                     declare
                        From : Natural := Rel_At + 1;
                     begin
                        if From <= Tail and then Lw (From) = "on" then
                           From := From + 1;
                        end if;
                        if From <= Tail then
                           C.Obj := Make_Noun (From, Tail);
                        end if;
                     end;
                  end;
                  if C.R = Re_Press and then C.Ef = Ef_None then
                     Fail ("press 要说多大劲:light firm hard。"
                           & "压的那一路上不许再说走到哪 —— 一根轴上要么说怎么动,要么说多用力", Line_No);
                     return;
                  end if;
                  if C.R = Re_Press and then C.Sp /= Sp_None then
                     Fail ("press 说了劲就不许再说步子 —— 同一根轴上位置和力只能二选一", Line_No);
                     return;
                  end if;
                  --  close 可以不带宾语:"就在这儿合上"。GH 实测:球被我自己的手挡住、点不了名的时候,
                  --  没有任何一句话能让它合手 —— 那是语言缺一个原语,不是身体做不到。
                  if C.R not in Re_Still | Re_Open | Re_Close and then C.Obj.K = Nk_None then
                     Fail ("「" & Rel_Word (C.R) & "」后面要跟一个东西 —— 这个词说的是【和谁的关系】", Line_No);
                     return;
                  end if;
                  I.Cons.Append (C);
                  K := Stop + 1;
                  <<Next_Cons>>
                  null;
               end;
               exit when K > N or else Lw (K) /= "and";
               K := K + 1;
            end loop;
            --  until / or n steps / anyway
            while K <= N loop
               if Lw (K) = "until" then
                  if K + 1 > N then
                     Fail ("until 后面要跟一个结局。结局只有:" & All_Outcomes, Line_No);
                     return;
                  end if;
                  I.Until_Oc := To_Outcome (Lw (K + 1));
                  if I.Until_Oc = Oc_None then
                     Fail ("until 后面「" & Lw (K + 1) & "」不是结局。结局只有:" & All_Outcomes, Line_No);
                     return;
                  end if;
                  K := K + 2;
               elsif Lw (K) = "or" then
                  if K + 2 > N or else not Is_Digits (To_String (W (K + 1))) or else Lw (K + 2) /= "steps" then
                     Fail ("写成「or <几> steps」", Line_No);
                     return;
                  end if;
                  I.Max_Steps := Natural'Value (To_String (W (K + 1)));
                  K := K + 3;
               elsif Lw (K) = "anyway" then
                  I.Anyway := True;
                  K := K + 1;
               --  🔴 用哪只眼睛判这一段:「with my still eye」/「with my moving eye」。不用编号。
               --  still = 我这一块一动、画面变得最少的那只(不长在我身上 ⇒ 看得见我在平移);
               --  moving = 变得最多的那只(长在我这一块上 ⇒ 离得近看得清,但看不见自己平移)。
               elsif Lw (K) = "with" then
                  if K + 3 <= N and then Lw (K + 1) = "my" and then Lw (K + 3) = "eye"
                    and then (Lw (K + 2) = "still" or else Lw (K + 2) = "moving")
                  then
                     I.Eye := (if Lw (K + 2) = "still" then Ey_Still else Ey_Moving);
                     K := K + 4;
                  else
                     Fail ("写成「with my still eye」或「with my moving eye」", Line_No);
                     return;
                  end if;
               else
                  Fail ("「" & To_String (W (K)) & "」我不认得。这一位上能放的是:until / or <几> steps / anyway / with my still eye / with my moving eye", Line_No);
                  return;
               end if;
            end loop;
            if I.Until_Oc = Oc_None then
               Fail ("每一段都要说【到什么为止】。结局只有:" & All_Outcomes, Line_No);
               return;
            end if;
            I.Src := To_Unbounded_String (Text);
            Emit (I);
         end;
      end Do_Line;

      I : Natural := Src'First;
   begin
      while I <= Src'Last loop
         declare
            K : Natural := I;
         begin
            while K <= Src'Last and then Src (K) /= ASCII.LF loop
               K := K + 1;
            end loop;
            Line_No := Line_No + 1;
            declare
               Raw : constant String := Src (I .. K - 1);
               Cut : Natural := Raw'Last;
            begin
               while Cut >= Raw'First and then Raw (Cut) = ASCII.CR loop
                  Cut := Cut - 1;
               end loop;
               Do_Line (Raw (Raw'First .. Cut));
            end;
            I := K + 1;
         end;
      end loop;
      if P.Ok and then Depth > 0 then
         Fail ("有" & Natural'Image (Depth) & " 个块没有 end", Line_No);
      end if;
      return P;
   end Parse;

end Sinew;
