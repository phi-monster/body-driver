with Codec;
package body Plan is

   use type Lang.Rel;
   use type Lang.Verb;
   use type Lang.Event;
   use type Exam.Verdict;

   function Rows_Needed (R : Lang.Rel) return Need is
      N : Need := [others => False];
   begin
      case R is
         when Lang.R_At =>
            N (Exam.Sideways) := True; N (Exam.Updown) := True; N (Exam.Nearness) := True;
         when Lang.R_Above | Lang.R_Below =>
            N (Exam.Updown) := True;
         when Lang.R_Left | Lang.R_Right =>
            N (Exam.Sideways) := True;
         when Lang.R_Nearer | Lang.R_Farther | Lang.R_Onto | Lang.R_Off =>
            N (Exam.Nearness) := True;
         when Lang.R_Facing =>
            N (Exam.Facing) := True;
         when Lang.R_None =>
            null;
      end case;
      return N;
   end Rows_Needed;

   function Row_Ok (R : Exam.Report; Thing_Idx : Integer; Row : Exam.Row_Id) return Boolean is
   begin
      if Thing_Idx < 0 or else Thing_Idx >= Integer (R.Things.Length) then
         return False;
      end if;
      return Exam.Allowed (R.Things (Natural (Thing_Idx)).Rows (Row));
   end Row_Ok;

   function Rel_Ok (R : Exam.Report; Thing_Idx : Integer; Rl : Lang.Rel; Surface : Boolean) return Boolean is
      N : constant Need := Rows_Needed (Rl);
   begin
      if (Rl = Lang.R_Onto or else Rl = Lang.R_Off) and then not Surface then
         return False;
      end if;
      for Row in Exam.Row_Id loop
         if N (Row) and then not Row_Ok (R, Thing_Idx, Row) then
            return False;
         end if;
      end loop;
      return True;
   end Rel_Ok;

   function Usable_Rels (R : Exam.Report; Thing_Idx : Integer; Surface : Boolean) return String is
      S : Unbounded_String;
   begin
      for Rl in Lang.Rel loop
         if Rl /= Lang.R_None and then Rel_Ok (R, Thing_Idx, Rl, Surface) then
            Append (S, (if Length (S) > 0 then " " else "") & Lang.Rel_Word (Rl));
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Usable_Rels;

   function Why_Row (R : Exam.Report; Thing_Idx : Integer; Row : Exam.Row_Id) return String is
   begin
      if Thing_Idx < 0 or else Thing_Idx >= Integer (R.Things.Length) then
         return "这一块我还没量过响应 —— 我不知道推哪根通道会让它动";
      end if;
      return To_String (R.Things (Natural (Thing_Idx)).Rows (Row).Why);
   end Why_Row;

   function Name_Of (F : Facts_Vectors.Vector; N : Natural) return String is
   begin
      if N < Natural (F.Length) and then Length (F (N).Label) > 0 then
         return Codec.Img (N) & " 号(" & To_String (F (N).Label) & ")";
      end if;
      return Codec.Img (N) & " 号";
   end Name_Of;

   function Mine_List (F : Facts_Vectors.Vector) return String is
      S : Unbounded_String;
   begin
      for I in 0 .. Natural (F.Length) - 1 loop
         if F (I).Exists and then F (I).Mine then
            Append (S, (if Length (S) > 0 then " " else "") & Codec.Img (I));
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Mine_List;

   function Compile (P : Lang.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                     Surface : Boolean) return Compiled is
      C : Compiled;
      Hard_Rows : Need := [others => False];

      procedure Reject (Line : Natural; Msg : String; Alt : String := "") is
      begin
         if C.Ok then
            C.Ok := False;
            C.Err := To_Unbounded_String (Msg);
            C.Instead := To_Unbounded_String (Alt);
            C.Err_Line := Line;
         end if;
      end Reject;

      --  一个名词落地成编号:名字要身体自己认,现在认不出 ⇒ 直说,并把能用的编号列出来
      function Resolve (X : Lang.Name; Line : Natural; Who : String; Num : out Natural) return Boolean is
      begin
         Num := 0;
         if not X.Given then
            Reject (Line, "这一句没说" & Who);
            return False;
         end if;
         if not X.By_Number then
            Reject (Line, "我认不出「" & To_String (X.Word) & "」这个名字 —— 我看得见东西,但不知道它们叫什么",
                    "改成用画面上的编号。我身上能动的是:" & Mine_List (Facts));
            return False;
         end if;
         Num := X.Number;
         if Num >= Natural (Facts.Length) or else not Facts (Num).Exists then
            Reject (Line, Codec.Img (Num) & " 号我现在看不到",
                    "换一个我现在看得到的。我身上能动的是:" & Mine_List (Facts));
            return False;
         end if;
         return True;
      end Resolve;
   begin
      if not P.Ok then
         C.Ok := False;
         C.Err := P.Err;
         C.Err_Line := P.Err_Line;
         C.Instead := To_Unbounded_String ("先把这一行写成合法的句子,文法我每一轮都给你");
         return C;
      end if;
      C.On_Fail := P.On_Fail;
      for I in 0 .. Natural (P.Stmts.Length) - 1 loop
         declare
            S : constant Lang.Stmt := P.Stmts (I);
            G : Goal;
            Sub, Obj : Natural := 0;
         begin
            exit when not C.Ok;
            case S.V is
               when Lang.V_Say =>
                  Append (C.Says, (if Length (C.Says) > 0 then " " else "") & To_String (S.Text));
               when Lang.V_Done =>
                  C.Done := True;
               when Lang.V_Look =>
                  if S.Eye >= Natural (R.Eyes.Length) then
                     Reject (S.Line, "我没有第" & Codec.Img (S.Eye) & " 只眼睛",
                             "我一共有 " & Codec.Img (Natural (R.Eyes.Length)) & " 只,编号从 0 起");
                  else
                     C.Look := Integer (S.Eye);
                  end if;
               when Lang.V_Onfail =>
                  null;
               when others =>
                  if not Resolve (S.Subject, S.Line, "【谁动】", Sub) then
                     exit;
                  end if;
                  if not Facts (Sub).Mine then
                     Reject (S.Line, "第" & Name_Of (Facts, Sub) & "不是我身上的东西 —— 我只推得动我自己的零件",
                             "把【谁动】换成我身上的一块。我身上能动的是:" & Mine_List (Facts));
                     exit;
                  end if;
                  if S.V in Lang.V_Close | Lang.V_Open and then not Facts (Sub).Grip then
                     Reject (S.Line, "第" & Name_Of (Facts, Sub) & "不是一只手 —— 它合不上也张不开",
                             "合手要对着一只手说");
                     exit;
                  end if;
                  if S.R /= Lang.R_None then
                     if not Resolve (S.Object, S.Line, "【相对谁】", Obj) then
                        exit;
                     end if;
                     declare
                        Ti : constant Integer := Facts (Sub).Thing_Idx;
                        N : constant Need := Rows_Needed (S.R);
                     begin
                        for Row in Exam.Row_Id loop
                           if N (Row) and then not Row_Ok (R, Ti, Row) then
                              Reject (S.Line,
                                "「" & Lang.Rel_Word (S.R) & "」(" & Lang.Rel_Cn (S.R) & ")要靠「"
                                & Exam.Row_Name (Row) & "」这一行,而这一行在我身上不能用:"
                                & Why_Row (R, Ti, Row),
                                "我现在说得出口的关系只有:" & Usable_Rels (R, Ti, Surface));
                              exit;
                           end if;
                        end loop;
                        exit when not C.Ok;
                        if (S.R = Lang.R_Onto or else S.R = Lang.R_Off) and then not Surface then
                           Reject (S.Line, "「" & Lang.Rel_Word (S.R) & "」要知道它站在哪个面上,而我现在拟不出那个面",
                                   "我现在说得出口的关系只有:" & Usable_Rels (R, Ti, Surface));
                           exit;
                        end if;
                        G.Rows := N;
                     end;
                  end if;
                  if S.Ev = Lang.E_Free and then not Surface then
                     Reject (S.Line, "「until free」要知道它原来站在哪个面上,而我现在拟不出那个面",
                             "换成 until touch 或 until resist —— 这两个不用面");
                     exit;
                  end if;
                  --  两条硬约束抢同一行 ⇒ 后一条必然要牺牲前一条,不许跑
                  if S.V = Lang.V_Hold then
                     for Row in Exam.Row_Id loop
                        if G.Rows (Row) and then Hard_Rows (Row) then
                           Reject (S.Line, "已经有一条 hold 在管「" & Exam.Row_Name (Row)
                                   & "」这一行了,两条都说"& "必须保持,我一定得牺牲一条",
                                   "把其中一条改成 reach(可以被牺牲),或者换一个不抢这一行的关系");
                           exit;
                        end if;
                     end loop;
                     exit when not C.Ok;
                     for Row in Exam.Row_Id loop
                        if G.Rows (Row) then
                           Hard_Rows (Row) := True;
                        end if;
                     end loop;
                  end if;
                  G.Line := S.Line; G.V := S.V; G.R := S.R;
                  G.Subject := Sub; G.Object := Obj;
                  G.Hard := S.V = Lang.V_Hold;
                  G.Forbid := S.V = Lang.V_Never;
                  G.Amt := S.Amt; G.Ev := S.Ev; G.Steps := S.Steps;
                  G.Src := S.Src;
                  C.Goals.Append (G);
            end case;
         end;
      end loop;
      return C;
   end Compile;

   function Report_Text (C : Compiled) return String is
      S : Unbounded_String;
   begin
      if C.Ok then
         Append (S, "编译过了:" & Codec.Img (Natural (C.Goals.Length)) & " 条约束");
         for I in 0 .. Natural (C.Goals.Length) - 1 loop
            Append (S, ASCII.LF & "   " & (if C.Goals (I).Hard then "[必须保持] " else "")
                    & (if C.Goals (I).Forbid then "[不许进入] " else "") & To_String (C.Goals (I).Src));
         end loop;
      else
         Append (S, "第 " & Codec.Img (C.Err_Line) & " 行退回来了:" & To_String (C.Err));
         if Length (C.Instead) > 0 then
            Append (S, ASCII.LF & "   ⇒ " & To_String (C.Instead));
         end if;
      end if;
      return To_String (S);
   end Report_Text;

end Plan;
