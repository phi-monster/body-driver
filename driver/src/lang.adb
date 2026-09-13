with Ada.Characters.Handling; use Ada.Characters.Handling;
package body Lang is

   function Verb_Word (V : Verb) return String is
     (case V is
         when V_Hold => "hold", when V_Reach => "reach", when V_Press => "press", when V_Close => "close",
         when V_Open => "open", when V_Never => "never", when V_Say => "say",
         when V_Look => "look", when V_Onfail => "onfail", when V_Done => "done",
         when V_Bad => "?");

   function Verb_Cn (V : Verb) return String is
     (case V is
         when V_Hold => "一直保持", when V_Reach => "朝这个关系走", when V_Press => "朝它压,只说劲不说位置", when V_Close => "合手",
         when V_Open => "张手", when V_Never => "不许进入", when V_Say => "说一句",
         when V_Look => "换主画面", when V_Onfail => "失手了怎么办", when V_Done => "做完了",
         when V_Bad => "?");

   function Rel_Word (R : Rel) return String is
     (case R is
         when R_None => "", when R_At => "at", when R_Above => "above", when R_Below => "below",
         when R_Left => "left", when R_Right => "right", when R_Nearer => "nearer",
         when R_Farther => "farther", when R_Onto => "onto", when R_Off => "off",
         when R_Facing => "facing");

   function Rel_Cn (R : Rel) return String is
     (case R is
         when R_None => "(没说关系)", when R_At => "贴着它", when R_Above => "在它上面",
         when R_Below => "在它下面", when R_Left => "在它左边", when R_Right => "在它右边",
         when R_Nearer => "比它更靠近我这只眼睛", when R_Farther => "比它更远离我这只眼睛",
         when R_Onto => "朝它站的那个面压下去", when R_Off => "离开它站的那个面",
         when R_Facing => "转到我这一块指着它");

   function Amount_Word (A : Amount) return String is
     (case A is when A_None => "", when A_Small => "small", when A_Medium => "medium", when A_Large => "large");

   function Event_Word (E : Event) return String is
     (case E is when E_None => "", when E_Steps => "steps", when E_Touch => "touch",
         when E_Resist => "resist", when E_Settle => "settle", when E_Free => "free");

   function Event_Cn (E : Event) return String is
     (case E is when E_None => "(没说到什么为止)", when E_Steps => "走够步数",
         when E_Touch => "碰到", when E_Resist => "顶住推不动", when E_Settle => "画面不再变",
         when E_Free => "它离开了原来站的那个面");

   function Effort_Word (E : Effort) return String is
     (case E is when F_None => "", when F_Light => "light", when F_Firm => "firm", when F_Hard => "hard");

   function To_Effort (W : String) return Effort is
   begin
      for E in Effort loop
         if E /= F_None and then Effort_Word (E) = W then
            return E;
         end if;
      end loop;
      return F_None;
   end To_Effort;

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

   function To_Verb (W : String) return Verb is
   begin
      for V in Verb loop
         if V /= V_Bad and then Verb_Word (V) = W then
            return V;
         end if;
      end loop;
      return V_Bad;
   end To_Verb;

   function To_Rel (W : String) return Rel is
   begin
      for R in Rel loop
         if R /= R_None and then Rel_Word (R) = W then
            return R;
         end if;
      end loop;
      return R_None;
   end To_Rel;

   function To_Amount (W : String) return Amount is
   begin
      for A in Amount loop
         if A /= A_None and then Amount_Word (A) = W then
            return A;
         end if;
      end loop;
      return A_None;
   end To_Amount;

   function To_Event (W : String) return Event is
   begin
      for E in Event loop
         if E /= E_None and then Event_Word (E) = W then
            return E;
         end if;
      end loop;
      return E_None;
   end To_Event;

   function All_Rels return String is
      R : Unbounded_String;
   begin
      for X in Rel loop
         if X /= R_None then
            Append (R, (if Length (R) > 0 then " " else "") & Rel_Word (X));
         end if;
      end loop;
      return To_String (R);
   end All_Rels;

   function Unparse (S : Stmt) return String is
      function N (X : Name) return String is
        (if not X.Given then "" elsif X.By_Number then Natural'Image (X.Number) else " " & To_String (X.Word));
   begin
      return (if S.Together then "while " else "") & Verb_Word (S.V) & N (S.Subject)
        & (if S.R /= R_None then " " & Rel_Word (S.R) else "")
        & N (S.Object)
        & (if S.Amt /= A_None then " " & Amount_Word (S.Amt) else "")
        & (if S.Ef /= F_None then " " & Effort_Word (S.Ef) else "")
        & (if S.Ev /= E_None then " until " & Event_Word (S.Ev)
             & (if S.Ev = E_Steps then Natural'Image (S.Steps) else "") else "");
   end Unparse;

   function Parse (Src : String) return Program is
      P : Program;
      Line_No : Natural := 0;
      I : Natural := Src'First;

      procedure Fail (Msg : String; Ln : Natural) is
      begin
         if P.Ok then
            P.Ok := False;
            P.Err := To_Unbounded_String (Msg);
            P.Err_Line := Ln;
         end if;
      end Fail;

      procedure Do_Line (Text : String) is
         Words : array (1 .. 32) of Unbounded_String;
         N : Natural := 0;
         J : Natural := Text'First;
         S : Stmt;
      begin
         --  切词
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
               if N < Words'Last then
                  N := N + 1;
                  Words (N) := To_Unbounded_String (Text (J .. K - 1));
               end if;
               J := K;
            end;
         end loop;
         if N = 0 then
            return;
         end if;
         --  行首的 while:和上一条动作同一节里一起解
         if Lower (To_String (Words (1))) = "while" and then N >= 2 then
            S.Together := True;
            for K in 1 .. N - 1 loop
               Words (K) := Words (K + 1);
            end loop;
            N := N - 1;
         end if;
         declare
            W1 : constant String := Lower (To_String (Words (1)));
         begin
            --  注释行
            if W1'Length >= 1 and then W1 (W1'First) = '#' then
               return;
            end if;
            S.V := To_Verb (W1);
            S.Line := Line_No;
            S.Src := To_Unbounded_String (Text);
            if S.V = V_Bad then
               Fail ("第一个词「" & To_String (Words (1)) & "」不是动词。动词只有这几个:"
                     & "hold reach close open never say look onfail done", Line_No);
               return;
            end if;
         end;

         case S.V is
            when V_Say =>
               declare
                  T : Unbounded_String;
               begin
                  for K in 2 .. N loop
                     Append (T, (if Length (T) > 0 then " " else "") & To_String (Words (K)));
                  end loop;
                  S.Text := T;
               end;
            when V_Done =>
               null;
            when V_Look =>
               if N < 2 or else not Is_Digits (To_String (Words (2))) then
                  Fail ("look 后面要跟一只眼睛的编号", Line_No);
                  return;
               end if;
               S.Eye := Natural'Value (To_String (Words (2)));
            when V_Onfail =>
               if N < 2 then
                  Fail ("onfail 后面要跟 retry 或 stop", Line_No);
                  return;
               end if;
               declare
                  W : constant String := Lower (To_String (Words (2)));
               begin
                  if W = "retry" then
                     S.Fa := F_Retry;
                  elsif W = "stop" then
                     S.Fa := F_Stop;
                  else
                     Fail ("onfail 后面只能是 retry 或 stop,你写的是「" & W & "」", Line_No);
                     return;
                  end if;
               end;
               P.On_Fail := S.Fa;
            when others =>
               --  <动词> <谁> [<关系> <相对谁>] [<步子>] [until <事件> [<步数>]]
               if N < 2 then
                  Fail (Verb_Word (S.V) & " 后面要先说【谁动】(画面上的编号,或者一个名字)", Line_No);
                  return;
               end if;
               declare
                  W2 : constant String := To_String (Words (2));
               begin
                  S.Subject.Given := True;
                  if Is_Digits (W2) then
                     S.Subject.By_Number := True;
                     S.Subject.Number := Natural'Value (W2);
                  else
                     S.Subject.Word := Words (2);
                  end if;
               end;
               declare
                  K : Natural := 3;
               begin
                  while K <= N loop
                     declare
                        W : constant String := Lower (To_String (Words (K)));
                        Rr : constant Rel := To_Rel (W);
                        Aa : constant Amount := To_Amount (W);
                     begin
                        if S.V = V_Press and then S.R = R_None and then not S.Object.Given
                          and then Rr = R_None and then Aa = A_None and then To_Effort (W) = F_None
                          and then W /= "until"
                        then
                           S.Object.Given := True;
                           if Is_Digits (To_String (Words (K))) then
                              S.Object.By_Number := True;
                              S.Object.Number := Natural'Value (To_String (Words (K)));
                           else
                              S.Object.Word := Words (K);
                           end if;
                        elsif W = "on" then
                           --  close 的 "on X" 就是 at X
                           if K + 1 > N then
                              Fail ("on 后面要跟一个东西", Line_No);
                              return;
                           end if;
                           S.R := R_At;
                           K := K + 1;
                           S.Object.Given := True;
                           if Is_Digits (To_String (Words (K))) then
                              S.Object.By_Number := True;
                              S.Object.Number := Natural'Value (To_String (Words (K)));
                           else
                              S.Object.Word := Words (K);
                           end if;
                        elsif Rr /= R_None then
                           S.R := Rr;
                           if K + 1 > N then
                              Fail ("「" & W & "」后面要跟一个东西(编号或名字)——"
                                    & "这个词说的是【和谁的关系】,不说谁就不成立", Line_No);
                              return;
                           end if;
                           K := K + 1;
                           S.Object.Given := True;
                           if Is_Digits (To_String (Words (K))) then
                              S.Object.By_Number := True;
                              S.Object.Number := Natural'Value (To_String (Words (K)));
                           else
                              S.Object.Word := Words (K);
                           end if;
                        elsif Aa /= A_None then
                           S.Amt := Aa;
                        elsif To_Effort (W) /= F_None then
                           S.Ef := To_Effort (W);
                        elsif W = "until" then
                           if K + 1 > N then
                              Fail ("until 后面要跟一个事件:steps touch resist settle free", Line_No);
                              return;
                           end if;
                           K := K + 1;
                           declare
                              Ew : constant String := Lower (To_String (Words (K)));
                           begin
                              S.Ev := To_Event (Ew);
                              if S.Ev = E_None then
                                 Fail ("until 后面「" & Ew & "」不是事件。事件只有:"
                                       & "steps touch resist settle free", Line_No);
                                 return;
                              end if;
                              if S.Ev = E_Steps then
                                 if K + 1 <= N and then Is_Digits (To_String (Words (K + 1))) then
                                    K := K + 1;
                                    S.Steps := Natural'Value (To_String (Words (K)));
                                 else
                                    Fail ("until steps 后面要跟走几步", Line_No);
                                    return;
                                 end if;
                              end if;
                           end;
                        else
                           Fail ("「" & To_String (Words (K)) & "」这个词我不认得。"
                                 & "这一位上能放的是:关系(" & All_Rels & ")、步子(small medium large)、"
                                 & "或者 until", Line_No);
                           return;
                        end if;
                        K := K + 1;
                     end;
                  end loop;
               end;
               if S.V in V_Hold | V_Reach | V_Never and then S.R = R_None then
                  Fail (Verb_Word (S.V) & " 要说清楚【和谁的什么关系】。关系只有这几个:" & All_Rels, Line_No);
                  return;
               end if;
               if S.V = V_Press then
                  if not S.Object.Given then
                     Fail ("press 要说【朝谁压】", Line_No);
                     return;
                  end if;
                  if S.Ef = F_None then
                     Fail ("press 要说多大劲:light firm hard。"
                           & "压这一路上不许再说走到哪 —— 一根轴上要么说怎么动,要么说多用力,二选一", Line_No);
                     return;
                  end if;
                  if S.Amt /= A_None then
                     Fail ("press 说了劲就不许再说步子 —— 同一根轴上位置和力只能二选一", Line_No);
                     return;
                  end if;
                  if S.Ev = E_None then
                     S.Ev := E_Resist;   --  压的默认停机条件就是"顶住了"
                  end if;
               end if;
         end case;
         P.Stmts.Append (S);
      end Do_Line;

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
               --  去掉行尾回车
               while Cut >= Raw'First and then (Raw (Cut) = ASCII.CR) loop
                  Cut := Cut - 1;
               end loop;
               Do_Line (Raw (Raw'First .. Cut));
            end;
            I := K + 1;
         end;
      end loop;
      return P;
   end Parse;

   function Grammar return String is
   begin
      return
        "<program> ::= <line>+" & ASCII.LF &
        "<line>    ::= [while] <action>            (while = run it together with the line above, not after it)" & ASCII.LF &
        "            | look <eye> | say <one sentence in your own words>" & ASCII.LF &
        "            | onfail retry | onfail stop | done" & ASCII.LF &
        "<action>  ::= hold <mine> <rel> <thing>" & ASCII.LF &
        "            | reach <mine> <rel> <thing> [<step>] [until <event>]" & ASCII.LF &
        "            | press <mine> <thing> <effort> [until <event>]" & ASCII.LF &
        "            | never <mine> <rel> <thing>" & ASCII.LF &
        "            | close <mine> [on <thing>] [until <event>]" & ASCII.LF &
        "            | open  <mine>" & ASCII.LF &
        "<rel>     ::= " & All_Rels & ASCII.LF &
        "<step>    ::= small | medium | large" & ASCII.LF &
        "<effort>  ::= light | firm | hard        (on the axis you press, you say effort and NOT where to go)" & ASCII.LF &
        "<event>   ::= steps <n> | touch | resist | settle | free" & ASCII.LF &
        "<mine> <thing> ::= <number on the picture> | <a name I must find myself>" & ASCII.LF &
        "<eye>     ::= <number of one of my eyes>";
   end Grammar;

end Lang;
