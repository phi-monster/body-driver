with Codec;
package body Plan is

   use type Sinew.Rel;
   use type Sinew.Noun_Kind;
   use type Sinew.Op;
   use type Sinew.Outcome;
   use type Exam.Verdict;

   function Rows_Needed (R : Sinew.Rel) return Need is
      N : Need := [others => False];
   begin
      case R is
         when Sinew.Re_Touching =>
            N (Exam.Sideways) := True; N (Exam.Updown) := True; N (Exam.Nearness) := True;
         when Sinew.Re_Above | Sinew.Re_Below =>
            N (Exam.Updown) := True;
         when Sinew.Re_Left | Sinew.Re_Right =>
            N (Exam.Sideways) := True;
         when Sinew.Re_Nearer | Sinew.Re_Farther | Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Press =>
            N (Exam.Nearness) := True;
         when Sinew.Re_Facing =>
            N (Exam.Facing) := True;
         when Sinew.Re_Clear =>
            N (Exam.Sideways) := True; N (Exam.Updown) := True;
         when Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open | Sinew.Re_None =>
            null;
      end case;
      return N;
   end Rows_Needed;

   function Key_Of (N : Sinew.Noun) return String is
     (case N.K is
         when Sinew.Nk_Role => Sinew.Role_Word (N.R),
         when Sinew.Nk_None => "",
         when others => To_String (N.Word));

   function Look_Up (B : Bind_Vectors.Vector; N : Sinew.Noun) return Integer is
      K : constant String := Key_Of (N);
   begin
      if K = "" then
         return -1;
      end if;
      for I in 0 .. Natural (B.Length) - 1 loop
         if To_String (B (I).Key) = K then
            return B (I).Item;
         end if;
      end loop;
      return -1;
   end Look_Up;

   function Tried_Of (B : Bind_Vectors.Vector; N : Sinew.Noun) return String is
      K : constant String := Key_Of (N);
   begin
      for I in 0 .. Natural (B.Length) - 1 loop
         if To_String (B (I).Key) = K then
            return To_String (B (I).Tried);
         end if;
      end loop;
      return "";
   end Tried_Of;

   function Row_Ok (R : Exam.Report; Thing_Idx : Integer; Row : Exam.Row_Id) return Boolean is
   begin
      if Thing_Idx < 0 or else Thing_Idx >= Integer (R.Things.Length) then
         return False;
      end if;
      return Exam.Allowed (R.Things (Natural (Thing_Idx)).Rows (Row));
   end Row_Ok;

   function Rel_Ok (R : Exam.Report; Thing_Idx : Integer; Rl : Sinew.Rel; Surface : Boolean) return Boolean is
      N : constant Need := Rows_Needed (Rl);
   begin
      if Rl in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Press and then not Surface then
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
      for Rl in Sinew.Rel loop
         if Rl not in Sinew.Re_None | Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open
           and then Rel_Ok (R, Thing_Idx, Rl, Surface)
         then
            Append (S, (if Length (S) > 0 then " " else "") & Sinew.Rel_Word (Rl));
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

   function Mine_List (F : Facts_Vectors.Vector) return String is
      S : Unbounded_String;
   begin
      for I in 0 .. Natural (F.Length) - 1 loop
         if F (I).Exists and then F (I).Mine and then Length (F (I).Label) > 0 then
            Append (S, (if Length (S) > 0 then " · " else "") & To_String (F (I).Label));
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Mine_List;

   function Check (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                   B : Bind_Vectors.Vector) return Verdict is
      V : Verdict;

      procedure Reject (Line : Natural; Msg : String; Alt : String := "") is
      begin
         if V.Ok then
            V.Ok := False;
            V.Err := To_Unbounded_String (Msg);
            V.Instead := To_Unbounded_String (Alt);
            V.Err_Line := Line;
         end if;
      end Reject;

      function Resolve (N : Sinew.Noun; Line : Natural; Who : String; Num : out Natural) return Boolean is
         A : constant Integer := Look_Up (B, N);
         T : constant String := Tried_Of (B, N);
      begin
         Num := 0;
         if N.K = Sinew.Nk_None then
            Reject (Line, "这一句没说" & Who);
            return False;
         end if;
         if A < 0 or else A >= Integer (Facts.Length) or else not Facts (Natural (A)).Exists then
            if N.K = Sinew.Nk_Role then
               Reject (Line, "我身上没有「" & Key_Of (N) & "」这个东西",
                       "我量出来身上有的是:" & Mine_List (Facts));
            else
               Reject (Line, "我认不出「" & Key_Of (N) & "」"
                       & (if T = "" then "" else " —— " & T),
                       "换一个说法,或者先让我看清它");
            end if;
            return False;
         end if;
         Num := Natural (A);
         return True;
      end Resolve;
   begin
      if not P.Ok then
         V.Ok := False;
         V.Err := P.Err;
         V.Err_Line := P.Err_Line;
         V.Instead := To_Unbounded_String ("文法我每一轮都给你,照着改一行就行");
         return V;
      end if;
      for Ix in 0 .. Natural (P.Code.Length) - 1 loop
         exit when not V.Ok;
         declare
            I : constant Sinew.Instr := P.Code (Ix);
         begin
            if I.O = Sinew.Op_Interval then
               for Ci in 0 .. Natural (I.Cons.Length) - 1 loop
                  exit when not V.Ok;
                  declare
                     C : constant Sinew.Constraint := I.Cons (Ci);
                     Sub, Obj : Natural := 0;
                  begin
                     if not Resolve (C.Subj, I.Line, "【谁】", Sub) then
                        exit;
                     end if;
                     if not Facts (Sub).Mine then
                        Reject (I.Line, "「" & Key_Of (C.Subj) & "」不是我身上的东西 —— 我只推得动我自己",
                                "我量出来身上有的是:" & Mine_List (Facts));
                        exit;
                     end if;
                     if C.R in Sinew.Re_Close | Sinew.Re_Open and then not Facts (Sub).Grasp then
                        Reject (I.Line, "「" & Key_Of (C.Subj) & "」合不拢 —— 我没量到它有两块能相向靠拢的部件",
                                "我身上能合拢的是:" & Mine_List (Facts));
                        exit;
                     end if;
                     if C.Obj.K /= Sinew.Nk_None then
                        if not Resolve (C.Obj, I.Line, "【和谁】", Obj) then
                           exit;
                        end if;
                     end if;
                     declare
                        Ti : constant Integer := Facts (Sub).Thing_Idx;
                        Nd : constant Need := Rows_Needed (C.R);
                        Surface : constant Boolean :=
                          C.Obj.K = Sinew.Nk_None or else Facts (Obj).Stands;
                        Unknown : constant Boolean := Ti < 0;   --  还没量过 ⇒ 执行器量完当场补判
                     begin
                        if C.R in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Press and then not Surface then
                           Reject (I.Line, "我量不出「" & Key_Of (C.Obj) & "」鼓出它靠着的那个面多少,"
                                   & "所以我不知道哪个方向才算朝它压",
                                   "我现在说得出口的关系:" & Usable_Rels (R, Ti, False));
                           exit;
                        end if;
                        if I.Until_Oc = Sinew.Oc_Free and then not Surface then
                           Reject (I.Line, "「until free」要知道它原来靠着哪个面,我量不出来",
                                   "换成 until touched 或 until stuck —— 这两个不用面");
                           exit;
                        end if;
                        if not Unknown then
                           for Row in Exam.Row_Id loop
                              if Nd (Row) and then not Row_Ok (R, Ti, Row) then
                                 Reject (I.Line,
                                   "「" & Sinew.Rel_Word (C.R) & "」(" & Sinew.Rel_Cn (C.R) & ")要靠「"
                                   & Exam.Row_Name (Row) & "」这一行,而这一行在我身上不能用:"
                                   & Why_Row (R, Ti, Row),
                                   "我现在说得出口的关系:" & Usable_Rels (R, Ti, Surface));
                                 exit;
                              end if;
                           end loop;
                        end if;
                     end;
                  end;
               end loop;
            end if;
         end;
      end loop;
      return V;
   end Check;

   function Say (V : Verdict) return String is
   begin
      if V.Ok then
         return "编译过了";
      end if;
      return "第 " & Codec.Img (V.Err_Line) & " 行退回来了:" & To_String (V.Err)
        & (if Length (V.Instead) > 0 then ASCII.LF & "   ⇒ " & To_String (V.Instead) else "");
   end Say;

end Plan;
