with Ada.Characters.Handling; use Ada.Characters.Handling;
package body Sinew is

   function Role_Word (R : Role) return String is
     (case R is when Rl_None => "", when Rl_Me => "me",
         when Rl_Grasper => "grasper", when Rl_Pusher => "pusher");

   function Rel_Word (R : Rel) return String is
     (case R is
         when Re_None => "", when Re_Touching => "touching", when Re_Above => "above",
         when Re_Below => "below", when Re_Left => "left", when Re_Right => "right",
         when Re_Nearer => "nearer", when Re_Farther => "farther", when Re_Onto => "onto",
         when Re_Off => "off", when Re_Facing => "facing", when Re_Clear => "clear",
         when Re_Still => "still", when Re_Press => "press", when Re_Close => "close",
         when Re_Open => "open");

   function Rel_Cn (R : Rel) return String is
     (case R is
         when Re_None => "(没说关系)", when Re_Touching => "贴上它", when Re_Above => "在它上面",
         when Re_Below => "在它下面", when Re_Left => "在它左边", when Re_Right => "在它右边",
         when Re_Nearer => "比它更靠近看得最清的那只眼睛", when Re_Farther => "比它更远离那只眼睛",
         when Re_Onto => "朝它靠着的那个面压过去", when Re_Off => "离开那个面",
         when Re_Facing => "转到我这一块指着它", when Re_Clear => "不许靠得比这更近",
         when Re_Still => "这一段不许动", when Re_Press => "朝它压,只说劲不说位置",
         when Re_Close => "合拢", when Re_Open => "张开");

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
         when Oc_Free => "free", when Oc_Timeout => "timeout", when Oc_Refused => "refused");

   function Outcome_Cn (O : Outcome) return String is
     (case O is
         when Oc_None => "(没说到什么为止)", when Oc_Arrived => "约束满足了",
         when Oc_Touched => "碰上了", when Oc_Stuck => "命令了但身体没走",
         when Oc_Slipped => "手里的东西掉了", when Oc_Lost => "看不见我正跟着的东西了",
         when Oc_Free => "它离开了原来靠着的面", when Oc_Timeout => "步子走完还没到",
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
      for O in Outcome loop
         if O /= Oc_None then
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

   function Grammar return String is
   begin
      return
        "<program>   ::= <line>+" & ASCII.LF &
        "<line>      ::= <interval> | <control> | <decl> | <word>" & ASCII.LF &
        "<interval>  ::= do <constraint> (and <constraint>)* until <outcome> [or <n> steps] [anyway]" & ASCII.LF &
        "<constraint>::= <who> <relation> <what> [<step>] [must]" & ASCII.LF &
        "              | <who> press <what> <effort> [must]      (that axis says effort, NOT where to go)" & ASCII.LF &
        "              | <who> close <what> | <who> open | <who> still" & ASCII.LF &
        "<who>       ::= me | grasper | pusher                   (roles; I bind them by measuring myself)" & ASCII.LF &
        "<what>      ::= <a name in your words> | <a name you told me to remember> | <who>" & ASCII.LF &
        "<relation>  ::= " & All_Rels & ASCII.LF &
        "<step>      ::= small | medium | large" & ASCII.LF &
        "<effort>    ::= light | firm | hard" & ASCII.LF &
        "<outcome>   ::= " & All_Outcomes & ASCII.LF &
        "<control>   ::= repeat <n> times: <line>+ end" & ASCII.LF &
        "              | repeat until <outcome>: <line>+ end" & ASCII.LF &
        "              | if <outcome>: <line>+ [else: <line>+] end" & ASCII.LF &
        "              | try: <line>+ or: <line>+ end" & ASCII.LF &
        "<decl>      ::= to <name>: <line>+ end | run <name>" & ASCII.LF &
        "              | remember where <who> is as <name>" & ASCII.LF &
        "<word>      ::= say <one sentence in your own words> | done" & ASCII.LF &
        "anyway = drop every caution of mine: go blind, close far, keep pushing. I will not argue.";
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
                  if C.R not in Re_Still | Re_Open and then C.Obj.K = Nk_None then
                     Fail ("「" & Rel_Word (C.R) & "」后面要跟一个东西 —— 这个词说的是【和谁的关系】", Line_No);
                     return;
                  end if;
                  I.Cons.Append (C);
                  K := Stop + 1;
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
               else
                  Fail ("「" & To_String (W (K)) & "」我不认得。这一位上能放的是:until / or <几> steps / anyway", Line_No);
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
