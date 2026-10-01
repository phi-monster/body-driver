--  numbers check | numbers gen | numbers teeth
--
--  Every numeric literal in the driver must have a stated origin, and none
--  may be a tuning number.
--
--  Each literal in driver/src (strings included, self-test packages
--  excluded) is either structural by rule (zero, powers, small component
--  indexes, the 1 in X + 1, X - 1 and 1 .. N, 1.0 in 1.0 - x and in unit
--  axis vectors) or listed in driver/numbers.tsv, one row per occurrence,
--  with a category: structure, math, numerics, statistics or format. A
--  number that describes a body, a scene or a behavior is a tuning number
--  and is not allowed. Unlisted literals, unclassified rows and tuning rows
--  make the check fail. gen appends unlisted literals as undecided rows and
--  drops rows whose literal is gone.
--
--  The rules were hardened by two audits of the previous driver; Teeth holds
--  the holes those audits found, and check refuses to run if any reopens.
--  Run it from the repository root.

with Ada.Characters.Handling;
with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Ordered_Sets;
with Ada.Containers.Indefinite_Vectors;
with Ada.Directories;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;

procedure Numbers is

   use Ada.Characters.Handling;
   use Ada.Strings.Unbounded;
   use Ada.Text_IO;

   package String_Sets is new Ada.Containers.Indefinite_Ordered_Sets (String);
   package String_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

   Registry_Path : constant String := "driver/numbers.tsv";
   Header : constant String :=
     "# file" & ASCII.HT & "line (comments and strings removed, blanks normalized)" & ASCII.HT
     & "literal (a "" prefix: inside a string)" & ASCII.HT & "category" & ASCII.HT & "why";
   Undecided_Why : constant String :=
     "undecided: read the context and classify as structure / math / numerics / statistics / format";

   function Category_Ok (C : String) return Boolean is
     (C = "structure" or else C = "math" or else C = "numerics" or else C = "statistics" or else C = "format");

   Keywords : String_Sets.Set;
   Standard_Callables : String_Sets.Set;
   Packages : String_Sets.Set;

   procedure Add_Words (S : in out String_Sets.Set; Words : String) is
      First : Natural := Words'First;
   begin
      for I in Words'Range loop
         if Words (I) = ' ' then
            if I > First then
               S.Include (Words (First .. I - 1));
            end if;
            First := I + 1;
         end if;
      end loop;
      if First <= Words'Last then
         S.Include (Words (First .. Words'Last));
      end if;
   end Add_Words;

   procedure Initialize_Words is
   begin
      Add_Words (Keywords, "abort abs abstract accept access aliased all and array at begin body case constant "
                 & "declare delay delta digits do else elsif end entry exception exit for function generic goto if "
                 & "in interface is limited loop mod new not null of or others out overriding package pragma private "
                 & "procedure protected raise range record rem renames requeue return reverse select separate some "
                 & "subtype synchronized tagged task terminate then type until use when while with xor");
      Add_Words (Standard_Callables, "Long_Float Float Integer Natural Positive Boolean Character String Duration "
                 & "Long_Integer Long_Long_Integer Short_Integer Unsigned_8 Unsigned_16 Unsigned_32 Unsigned_64 "
                 & "Integer_8 Integer_16 Integer_32 Integer_64 Stream_Element Stream_Element_Offset "
                 & "Stream_Element_Array Sqrt Sin Cos Tan Cot Arctan Arcsin Arccos Arccot Exp Log Sinh Cosh Tanh Put "
                 & "Put_Line Get Get_Line Append Prepend Insert Delete Replace_Element Element To_Vector To_String "
                 & "To_Unbounded_String Set_Length Reserve_Capacity Slice Head Tail Index Trim Shift_Left Shift_Right "
                 & "Rotate_Left Rotate_Right Unchecked_Conversion Unchecked_Deallocation Clock Seconds Milliseconds "
                 & "Microseconds Argument Value Image Floor Ceiling Rounding Truncation");
      Add_Words (Packages, "Ada Interfaces GNAT System");
   end Initialize_Words;

   function Is_Letter (C : Character) return Boolean is (C in 'A' .. 'Z' | 'a' .. 'z' | '_');
   function Is_Word (C : Character) return Boolean is (Is_Letter (C) or else C in '0' .. '9');
   function Is_Digit (C : Character) return Boolean is (C in '0' .. '9');

   function Is_Keyword (W : String) return Boolean is (Keywords.Contains (To_Lower (W)));

   --  A line split into its code (strings emptied, comment removed) and the
   --  contents of its strings.
   procedure Split_Code (Raw : String; Code : out Unbounded_String; Strings : out String_Vectors.Vector) is
      I : Natural := Raw'First;
   begin
      Code := Null_Unbounded_String;
      Strings.Clear;
      while I <= Raw'Last loop
         declare
            C : constant Character := Raw (I);
         begin
            if C = '"' then
               declare
                  J   : Natural := I + 1;
                  Cur : Unbounded_String;
               begin
                  while J <= Raw'Last loop
                     if Raw (J) = '"' then
                        if J < Raw'Last and then Raw (J + 1) = '"' then
                           Append (Cur, '"');
                           J := J + 2;
                        else
                           exit;
                        end if;
                     else
                        Append (Cur, Raw (J));
                        J := J + 1;
                     end if;
                  end loop;
                  Strings.Append (To_String (Cur));
                  Append (Code, """""");
                  I := J + 1;
               end;
            elsif C = ''' and then I + 2 <= Raw'Last and then Raw (I + 2) = '''
              and then not (I > Raw'First and then (Is_Word (Raw (I - 1)) or else Raw (I - 1) = ')'))
            then
               Append (Code, "''");
               I := I + 3;
            elsif C = '-' and then I < Raw'Last and then Raw (I + 1) = '-' then
               exit;
            else
               Append (Code, C);
               I := I + 1;
            end if;
         end;
      end loop;
   end Split_Code;

   function Normalized (Code : String) return String is
      R : Unbounded_String;
      Blank : Boolean := True;
   begin
      for C of Code loop
         if C in ' ' | ASCII.HT then
            Blank := True;
         else
            if Blank and then Length (R) > 0 then
               Append (R, ' ');
            end if;
            Append (R, C);
            Blank := False;
         end if;
      end loop;
      return To_String (R);
   end Normalized;

   --  Per-file scope: callables and objects declared in a file, its spec and
   --  the bodies separate from it; public: those of every spec.
   type Scope is record
      Callables, Objects : String_Sets.Set;
   end record;

   package Scope_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Scope);
   File_Scopes : Scope_Maps.Map;
   Public : Scope;

   procedure Scan_Names (Path : String; S : in out Scope) is
      F : File_Type;
      Code : Unbounded_String;
      Strs : String_Vectors.Vector;
   begin
      Open (F, In_File, Path);
      while not End_Of_File (F) loop
         Split_Code (Get_Line (F), Code, Strs);
         declare
            C : constant String := To_String (Code);
            I : Natural := C'First;

            function Word_At (P : Natural; Last : out Natural) return String is
               Q : Natural := P;
            begin
               while Q <= C'Last and then Is_Word (C (Q)) loop
                  Q := Q + 1;
               end loop;
               Last := Q - 1;
               return C (P .. Q - 1);
            end Word_At;

            function Skip_Blanks (P : Natural) return Natural is
               Q : Natural := P;
            begin
               while Q <= C'Last and then C (Q) in ' ' | ASCII.HT loop
                  Q := Q + 1;
               end loop;
               return Q;
            end Skip_Blanks;

            Can_Start  : Boolean := True;   --  at the start of the line, after ';' or '('
            After_Name : Boolean := False;  --  the last token was a name of the group
            Group : String_Vectors.Vector;  --  names that a ':' would declare
         begin
            while I <= C'Last loop
               if Is_Letter (C (I)) and then (I = C'First or else not Is_Word (C (I - 1))) then
                  declare
                     Last : Natural;
                     W    : constant String := Word_At (I, Last);
                     L    : constant String := To_Lower (W);
                  begin
                     if L = "with" then
                        --  with A.B, C;  the first segment of each name is a package prefix
                        declare
                           P : Natural := Skip_Blanks (Last + 1);
                        begin
                           while P <= C'Last and then Is_Letter (C (P)) loop
                              declare
                                 E : Natural;
                                 N : constant String := Word_At (P, E);
                              begin
                                 if not Is_Keyword (N) then
                                    Packages.Include (N);
                                 end if;
                                 P := E + 1;
                                 while P <= C'Last and then (Is_Word (C (P)) or else C (P) = '.') loop
                                    P := P + 1;
                                 end loop;
                                 P := Skip_Blanks (P);
                                 exit when P > C'Last or else C (P) /= ',';
                                 P := Skip_Blanks (P + 1);
                              end;
                           end loop;
                        end;
                     elsif L in "function" | "procedure" | "entry" | "type" | "subtype" | "package" | "task"
                       | "protected"
                     then
                        declare
                           P : Natural := Skip_Blanks (Last + 1);
                           E : Natural;
                        begin
                           if P <= C'Last and then Is_Letter (C (P)) then
                              declare
                                 N : constant String := Word_At (P, E);
                              begin
                                 if To_Lower (N) = "body" then
                                    P := Skip_Blanks (E + 1);
                                 end if;
                              end;
                           end if;
                           if P <= C'Last and then Is_Letter (C (P)) then
                              declare
                                 N : constant String := Word_At (P, E);
                              begin
                                 S.Callables.Include (N);
                                 if L = "package" then
                                    --  package A.B.C: every segment names a package
                                    declare
                                       Q : Natural := P;
                                    begin
                                       loop
                                          declare
                                             E2  : Natural;
                                             Seg : constant String := Word_At (Q, E2);
                                          begin
                                             Packages.Include (Seg);
                                             exit when E2 + 1 > C'Last or else C (E2 + 1) /= '.';
                                             Q := E2 + 2;
                                             exit when Q > C'Last or else not Is_Letter (C (Q));
                                          end;
                                       end loop;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                     elsif L = "for" then
                        declare
                           P : constant Natural := Skip_Blanks (Last + 1);
                           E : Natural;
                        begin
                           if P <= C'Last and then Is_Letter (C (P)) then
                              declare
                                 N  : constant String := Word_At (P, E);
                                 Q  : constant Natural := Skip_Blanks (E + 1);
                                 E3 : Natural;
                              begin
                                 if Q <= C'Last and then Is_Letter (C (Q)) then
                                    declare
                                       K : constant String := To_Lower (Word_At (Q, E3));
                                    begin
                                       if K in "in" | "of" then
                                          S.Objects.Include (N);
                                       end if;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                     end if;
                     if Can_Start or else (not Group.Is_Empty and then not After_Name) then
                        Group.Append (W);
                        After_Name := True;
                     else
                        Group.Clear;
                        After_Name := False;
                     end if;
                     Can_Start := False;
                     I := Last + 1;
                  end;
               else
                  case C (I) is
                     when ',' =>
                        if After_Name then
                           After_Name := False;
                        else
                           Group.Clear;
                        end if;
                     when ':' =>
                        if After_Name and then not (I < C'Last and then C (I + 1) = '=') then
                           for N of Group loop
                              S.Objects.Include (N);
                           end loop;
                        end if;
                        Group.Clear;
                        After_Name := False;
                        Can_Start := False;
                     when ';' | '(' =>
                        Group.Clear;
                        After_Name := False;
                        Can_Start := True;
                     when ' ' | ASCII.HT =>
                        null;
                     when others =>
                        Group.Clear;
                        After_Name := False;
                        Can_Start := False;
                  end case;
                  I := I + 1;
               end if;
            end loop;
         end;
      end loop;
      Close (F);
   end Scan_Names;

   --  Driver sources: everything under driver/src except the self tests.
   procedure Collect (Root : String; Files : in out String_Vectors.Vector) is
      use Ada.Directories;
      Search : Search_Type;
      Item   : Directory_Entry_Type;
   begin
      Start_Search (Search, Root, "", [Ordinary_File => True, Ada.Directories.Directory => True, others => False]);
      while More_Entries (Search) loop
         Get_Next_Entry (Search, Item);
         declare
            Name : constant String := Simple_Name (Item);
         begin
            if Kind (Item) = Ada.Directories.Directory then
               if Name /= "." and then Name /= ".." then
                  Collect (Full_Name (Item), Files);
               end if;
            elsif (Extension (Name) = "ads" or else Extension (Name) = "adb")
              and then Ada.Strings.Fixed.Index (Name, "-tests.") = 0
            then
              Files.Append (Full_Name (Item));
            end if;
         end;
      end loop;
      End_Search (Search);
   end Collect;

   function Base (Path : String) return String is (Ada.Directories.Simple_Name (Path));

   --  The current file and lines, for the rules that look at context.
   Current : Unbounded_String;
   Lines   : String_Vectors.Vector;
   Line_Index : Natural := 0;

   --  The bracket that opens the expression containing position Pos.
   procedure Opener (C : String; Pos : Natural; Bracket : out Character; At_Index : out Natural) is
      Depth : Natural := 0;
   begin
      for I in reverse C'First .. Pos - 1 loop
         if C (I) in ')' | ']' then
            Depth := Depth + 1;
         elsif C (I) in '(' | '[' then
            if Depth = 0 then
               Bracket := C (I);
               At_Index := I;
               return;
            end if;
            Depth := Depth - 1;
         end if;
      end loop;
      Bracket := ' ';
      At_Index := 0;
   end Opener;

   function Closer (C : String; I : Natural) return Natural is
      Depth : Natural := 0;
   begin
      for J in I .. C'Last loop
         if C (J) in '(' | '[' then
            Depth := Depth + 1;
         elsif C (J) in ')' | ']' then
            Depth := Depth - 1;
            if Depth = 0 then
               return J;
            end if;
         end if;
      end loop;
      return C'Last + 1;
   end Closer;

   function Trim (S : String) return String is (Ada.Strings.Fixed.Trim (S, Ada.Strings.Both));

   --  The comma-separated parts of S at depth 0.
   function Parts (S : String) return String_Vectors.Vector is
      R : String_Vectors.Vector;
      Depth : Natural := 0;
      First : Natural := S'First;
   begin
      for I in S'Range loop
         if S (I) in '(' | '[' then
            Depth := Depth + 1;
         elsif S (I) in ')' | ']' then
            Depth := Depth - 1;
         elsif S (I) = ',' and then Depth = 0 then
            R.Append (Trim (S (First .. I - 1)));
            First := I + 1;
         end if;
      end loop;
      R.Append (Trim (S (First .. S'Last)));
      return R;
   end Parts;

   type Head_Kind is (Index_Head, Call_Head, Attribute_Head, No_Head);

   --  What precedes the bracket at I: an indexed object, a call, an attribute, or nothing callable.
   procedure Head_Of (C : String; I : Natural; Kind : out Head_Kind; Attribute : out Unbounded_String) is
      B : Natural := I - 1;
   begin
      Attribute := Null_Unbounded_String;
      while B >= C'First and then C (B) in ' ' | ASCII.HT loop
         B := B - 1;
      end loop;
      if B < C'First then
         Kind := No_Head;
         return;
      end if;
      if C (B) = ')' then
         Kind := Index_Head;
         return;
      end if;
      if not Is_Word (C (B)) then
         Kind := No_Head;
         return;
      end if;
      declare
         E : constant Natural := B;
         S : Natural := B;
      begin
         while S > C'First and then Is_Word (C (S - 1)) loop
            S := S - 1;
         end loop;
         declare
            Name : constant String := C (S .. E);
            P    : Natural := S - 1;
         begin
            while P >= C'First and then C (P) in ' ' | ASCII.HT loop
               P := P - 1;
            end loop;
            if P >= C'First and then C (P) = ''' then
               Kind := Attribute_Head;
               Attribute := To_Unbounded_String (Name);
               return;
            end if;
            if not Is_Letter (Name (Name'First)) or else Is_Keyword (Name) then
               Kind := No_Head;
               return;
            end if;
            --  The prefix chain A.B. before the name: a package prefix makes it a call.
            declare
               Q : Natural := P;
               First_Prefix : Unbounded_String;
            begin
               while Q >= C'First and then C (Q) = '.' loop
                  declare
                     R : Natural := Q - 1;
                     T : Natural;
                  begin
                     while R >= C'First and then C (R) in ' ' | ASCII.HT loop
                        R := R - 1;
                     end loop;
                     exit when R < C'First or else not Is_Word (C (R));
                     T := R;
                     while T > C'First and then Is_Word (C (T - 1)) loop
                        T := T - 1;
                     end loop;
                     First_Prefix := To_Unbounded_String (C (T .. R));
                     Q := T - 1;
                     while Q >= C'First and then C (Q) in ' ' | ASCII.HT loop
                        Q := Q - 1;
                     end loop;
                  end;
               end loop;
               if Length (First_Prefix) > 0 and then Packages.Contains (To_String (First_Prefix)) then
                  Kind := Call_Head;
                  return;
               end if;
            end;
            declare
               Local : constant Scope := (if File_Scopes.Contains (To_String (Current))
                                          then File_Scopes (To_String (Current)) else (others => <>));
            begin
               if Local.Callables.Contains (Name) then
                  Kind := Call_Head;
               elsif Local.Objects.Contains (Name) then
                  Kind := Index_Head;
               elsif Public.Callables.Contains (Name) or else Standard_Callables.Contains (Name) then
                  Kind := Call_Head;
               elsif Public.Objects.Contains (Name) then
                  Kind := Index_Head;
               else
                  Kind := Call_Head;
               end if;
            end;
         end;
      end;
   end Head_Of;

   function Is_Integer_Literal (V : String) return Boolean is
     (for all C of V => Is_Digit (C) or else C = '_');

   function Integer_Value (V : String) return Natural is
      N : Natural := 0;
   begin
      for C of V loop
         if Is_Digit (C) then
            N := N * 10 + (Character'Pos (C) - Character'Pos ('0'));
         end if;
      end loop;
      return N;
   end Integer_Value;

   function Is_Zero (V : String) return Boolean is
      Mantissa_End : Natural := V'Last;
   begin
      for I in V'Range loop
         if V (I) in 'e' | 'E' then
            Mantissa_End := I - 1;
            exit;
         end if;
      end loop;
      return (for all C of V (V'First .. Mantissa_End) => C in '0' | '_' | '.') and then V (V'First) = '0';
   end Is_Zero;

   function Is_One_Real (V : String) return Boolean is
     (V'Length >= 3 and then V (V'First .. V'First + 1) = "1." and then
        (for all C of V (V'First + 2 .. V'Last) => C = '0'));

   function Ends_With_Pattern (Before : String; Tail : String) return Boolean is
      T : constant String := Trim (Before);
   begin
      return T'Length >= Tail'Length and then T (T'Last - Tail'Length + 1 .. T'Last) = Tail;
   end Ends_With_Pattern;

   --  Whether the loop variable is used as an index inside the loop that starts at the current line.
   function Loop_Indexes (Var : String) return Boolean is
      Depth : Integer := 0;
   begin
      for J in Line_Index .. Natural'Min (Natural (Lines.Length), Line_Index + 400) loop
         declare
            Code : Unbounded_String;
            Strs : String_Vectors.Vector;
         begin
            Split_Code (Lines (J), Code, Strs);
            declare
               C : constant String := To_String (Code);
               Lower : constant String := To_Lower (C);
               P : Natural := 0;
            begin
               --  loop depth: every "loop" opens, every "end loop" closes (it also counts as one "loop")
               declare
                  K : Natural := Lower'First;
               begin
                  loop
                     K := Ada.Strings.Fixed.Index (Lower (K .. Lower'Last), "loop");
                     exit when K = 0;
                     if (K = Lower'First or else not Is_Word (Lower (K - 1)))
                       and then (K + 4 > Lower'Last or else not Is_Word (Lower (K + 4)))
                     then
                        Depth := Depth + 1;
                        declare
                           B : Integer := K - 1;
                        begin
                           while B >= Lower'First and then Lower (B) = ' ' loop
                              B := B - 1;
                           end loop;
                           if B >= Lower'First + 2 and then Lower (B - 2 .. B) = "end" then
                              Depth := Depth - 2;
                           end if;
                        end;
                     end if;
                     K := K + 4;
                     exit when K > Lower'Last;
                  end loop;
               end;
               --  Var used as an index: "(... Var ," or "( Var )" after an indexed head
               loop
                  P := Ada.Strings.Fixed.Index (C (P + 1 .. C'Last), Var);
                  exit when P = 0;
                  if (P = C'First or else not Is_Word (C (P - 1)))
                    and then (P + Var'Length > C'Last or else not Is_Word (C (P + Var'Length)))
                  then
                     declare
                        After : Natural := P + Var'Length;
                        Bracket : Character;
                        Open_At : Natural;
                        Kind : Head_Kind;
                        Attr : Unbounded_String;
                     begin
                        while After <= C'Last and then C (After) = ' ' loop
                           After := After + 1;
                        end loop;
                        if After <= C'Last and then C (After) in ',' | ')' then
                           Opener (C, P, Bracket, Open_At);
                           if Bracket = '(' then
                              Head_Of (C, Open_At, Kind, Attr);
                              if Kind = Index_Head then
                                 return True;
                              end if;
                           end if;
                        end if;
                     end;
                  end if;
                  exit when P + Var'Length > C'Last;
                  P := P + Var'Length - 1;
               end loop;
               if J > Line_Index and then Depth <= 0 then
                  return False;
               end if;
            end;
         end;
      end loop;
      return False;
   end Loop_Indexes;

   --  The last tokens of S: names, numbers, "..", ":=", or single symbols, in order.
   function Tail_Tokens (S : String; Count : Positive) return String_Vectors.Vector is
      Reversed : String_Vectors.Vector;
      I : Integer := S'Last;
   begin
      while I >= S'First and then Natural (Reversed.Length) < Count loop
         if S (I) in ' ' | ASCII.HT then
            I := I - 1;
         elsif Is_Word (S (I)) then
            declare
               J : Integer := I;
            begin
               while J > S'First and then Is_Word (S (J - 1)) loop
                  J := J - 1;
               end loop;
               Reversed.Append (S (J .. I));
               I := J - 1;
            end;
         elsif I > S'First and then S (I - 1 .. I) in ".." | ":=" | "**" | "=>" then
            Reversed.Append (S (I - 1 .. I));
            I := I - 2;
         else
            Reversed.Append (S (I .. I));
            I := I - 1;
         end if;
      end loop;
      return R : String_Vectors.Vector do
         for K in reverse 1 .. Natural (Reversed.Length) loop
            R.Append (Reversed (K));
         end loop;
      end return;
   end Tail_Tokens;

   --  The rules by which a literal needs no registry row.
   function Structural (V : String; C : String; S, E : Natural) return Boolean is
      Before : constant String := C (C'First .. S - 1);
      After  : constant String := C (E + 1 .. C'Last);
      Bracket : Character;
      Open_At : Natural;
      Kind    : Head_Kind := No_Head;
      Attr    : Unbounded_String;
      Inner   : Unbounded_String;

      function Before_Ends_With_Open_Or_Comma return Boolean is
        (Trim (Before)'Length > 0 and then Trim (Before) (Trim (Before)'Last) in '(' | ',');
      function After_Starts_With_Close_Or_Comma return Boolean is
        (Trim (After)'Length > 0 and then Trim (After) (Trim (After)'First) in ')' | ',');
      function Trimmed_After return String is (Trim (After));
      function Trimmed_Before return String is (Trim (Before));
   begin
      if Is_Zero (V) then
         Opener (C, S, Bracket, Open_At);
         if Bracket = '(' then
            Head_Of (C, Open_At, Kind, Attr);
            if Kind = Call_Head and then Before_Ends_With_Open_Or_Comma and then After_Starts_With_Close_Or_Comma
              and then Ada.Strings.Fixed.Index (V, ".") > 0
            then
               return False;   --  F (X, 0.0): a zero sent as a reading or target may be a body convention
            end if;
         end if;
         declare
            T : constant String := Trimmed_Before;
            Lower : constant String := To_Lower (T);
            K : constant Natural := Ada.Strings.Fixed.Index (Lower, "constant");
         begin
            if K > 0 and then Ada.Strings.Fixed.Index (T (K .. T'Last), ";") = 0
              and then (Ends_With_Pattern (T, ":=") or else Ends_With_Pattern (T, ":= -")
                        or else Ends_With_Pattern (T, ":=-"))
            then
               return False;   --  X : constant := 0.0 names a setting
            end if;
         end;
         return True;
      end if;
      if Ends_With_Pattern (Before, "**") then
         return True;
      end if;
      Opener (C, S, Bracket, Open_At);
      if Bracket = '(' then
         Head_Of (C, Open_At, Kind, Attr);
      else
         Kind := No_Head;
      end if;
      if Bracket /= ' ' then
         Inner := To_Unbounded_String (C (Open_At + 1 .. Natural'Min (C'Last, Closer (C, Open_At) - 1)));
      end if;
      if Is_Integer_Literal (V) then
         declare
            N : constant Natural := Integer_Value (V);
            Direct : constant Boolean := Before_Ends_With_Open_Or_Comma and then After_Starts_With_Close_Or_Comma;
         begin
            if Kind = Attribute_Head
              and then (To_String (Attr) in "First" | "Last" | "Range" | "Length")
              and then Trim (To_String (Inner)) = V and then N <= 3
            then
               return True;   --  A'Range (2): which dimension
            end if;
            if Kind = Index_Head then
               if Direct then
                  return N <= 8;   --  X (2), M (I, 2): a component
               end if;
               if Ends_With_Pattern (Before, "..") or else
                 (Trimmed_After'Length >= 2 and then Trimmed_After (Trimmed_After'First .. Trimmed_After'First + 1) = "..")
               then
                  return N <= 2;   --  slices: only component ranges such as 0 .. 2
               end if;
               return N = 1;       --  X (I + 1): the neighbour
            end if;
            if N = 1 then
               declare
                  T : constant String := Trimmed_Before;
               begin
                  if T'Length >= 1 and then T (T'Last) in '+' | '-' then
                     declare
                        U : constant String := Trim (T (T'First .. T'Last - 1));
                     begin
                        if U'Length > 0 and then (Is_Word (U (U'Last)) or else U (U'Last) in ')' | ']' | '.') then
                           --  X + 1 / X - 1, but not "in -1", "range -1" or ":= -1"
                           declare
                              W : Natural := U'Last;
                           begin
                              while W > U'First and then Is_Word (U (W - 1)) loop
                                 W := W - 1;
                              end loop;
                              if U (U'Last) in ')' | ']' or else not Is_Keyword (U (W .. U'Last)) then
                                 return True;
                              end if;
                           end;
                        end if;
                     end;
                  end if;
                  declare
                     A : constant String := Trimmed_After;
                  begin
                     if A'Length >= 1 and then A (A'First) in '+' | '-'
                       and then not (Trim (A (A'First + 1 .. A'Last))'Length > 0
                                     and then Is_Digit (Trim (A (A'First + 1 .. A'Last)) (1)))
                       and then not (T'Length >= 1 and then T (T'Last) in '+' | '-' | '*' | '/')
                     then
                        return True;   --  1 + X
                     end if;
                     if A'Length >= 2 and then A (A'First .. A'First + 1) = ".."
                       and then not (T'Length >= 1 and then T (T'Last) = '-')
                     then
                        return True;   --  for I in 1 .. N: counting from one
                     end if;
                  end;
               end;
               return False;   --  comparisons, assignments, arguments, defaults: possibly "at least one"
            end if;
            --  0 .. 2 in a type, or a loop variable used as an index: three components
            declare
               Tail : constant String_Vectors.Vector := Tail_Tokens (Trimmed_Before, 6);
               L    : constant Natural := Natural (Tail.Length);

               function Tok (K : Natural) return String is
                 (if K >= 1 and then K <= L then To_Lower (Tail (L - K + 1)) else "");
               --  Tok (1) is the token just before the literal.
            begin
               if N <= 2 and then Tok (1) = ".." and then Tok (2) = "0" then
                  declare
                     After_In : constant Natural := (if Tok (3) = "reverse" then 4 else 3);
                  begin
                     if Tok (After_In) = "in" and then Tok (After_In + 2) = "for"
                       and then Tok (After_In + 1) /= "" and then Is_Letter (Tok (After_In + 1) (1))
                     then
                        return Loop_Indexes (Tail (L - After_In));
                     end if;
                     return (Bracket = '(' and then Kind = No_Head) or else Tok (3) = "range";
                  end;
               end if;
            end;
            return False;
         end;
      end if;
      if Is_One_Real (V) then
         declare
            A : constant String := Trimmed_After;
            T : constant String := Trimmed_Before;
         begin
            if A'Length >= 1 and then A (A'First) = '-' and then not (A'Length >= 2 and then A (A'First + 1) = '-')
              and then not (T'Length >= 1 and then T (T'Last) in '*' | '/')
            then
               return True;   --  1.0 - x
            end if;
            if (Bracket = '[' or else (Bracket = '(' and then Kind = No_Head))
              and then Ada.Strings.Fixed.Index (To_Lower (C), "constant") = 0
            then
               declare
                  Ps : constant String_Vectors.Vector := Parts (To_String (Inner));
                  All_Axis : Boolean := Natural (Ps.Length) >= 2;
               begin
                  for P of Ps loop
                     declare
                        Q : constant String := Ada.Strings.Fixed.Trim (P, Ada.Strings.Both);
                        R : constant String := (if Q'Length > 0 and then Q (Q'First) = '-'
                                                then Trim (Q (Q'First + 1 .. Q'Last)) else Q);
                     begin
                        if not (R = "0" or else R = "1" or else Is_Zero (R) or else Is_One_Real (R)) then
                           All_Axis := False;
                        end if;
                     end;
                  end loop;
                  if All_Axis then
                     return True;   --  axis vectors and unit quaternions made of 0 and +-1
                  end if;
               end;
            end if;
         end;
      end if;
      return False;
   end Structural;

   --  Numeric literals of a code line: decimal or based, with fraction and exponent.
   type Occurrence is record
      First, Last : Natural;
   end record;

   package Occurrence_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Occurrence);

   function Literals (C : String) return Occurrence_Vectors.Vector is
      R : Occurrence_Vectors.Vector;
      I : Natural := C'First;
   begin
      while I <= C'Last loop
         if Is_Digit (C (I))
           and then not (I > C'First and then (Is_Letter (C (I - 1)) or else Is_Digit (C (I - 1)) or else C (I - 1) = '#'))
           and then not (I > C'First + 1 and then C (I - 1) = '.' and then Is_Digit (C (I - 2)))
         then
            declare
               J : Natural := I;
            begin
               while J <= C'Last and then (Is_Digit (C (J)) or else C (J) = '_') loop
                  J := J + 1;
               end loop;
               if J <= C'Last and then C (J) = '#' then
                  J := J + 1;
                  while J <= C'Last and then (Is_Word (C (J)) or else C (J) = '.') and then C (J) /= '#' loop
                     J := J + 1;
                  end loop;
                  if J <= C'Last and then C (J) = '#' then
                     J := J + 1;
                  end if;
               elsif J < C'Last and then C (J) = '.' and then Is_Digit (C (J + 1)) then
                  J := J + 1;
                  while J <= C'Last and then (Is_Digit (C (J)) or else C (J) = '_') loop
                     J := J + 1;
                  end loop;
               end if;
               if J < C'Last and then C (J) in 'e' | 'E'
                 and then (Is_Digit (C (J + 1)) or else (J + 1 < C'Last and then C (J + 1) in '+' | '-'
                                                          and then Is_Digit (C (J + 2))))
               then
                  J := J + 2;
                  while J <= C'Last and then (Is_Digit (C (J)) or else C (J) = '_') loop
                     J := J + 1;
                  end loop;
               end if;
               if not (J <= C'Last and then (Is_Letter (C (J)) or else C (J) = '#')) then
                  R.Append (Occurrence'(First => I, Last => J - 1));
               end if;
               I := J;
            end;
         else
            I := I + 1;
         end if;
      end loop;
      return R;
   end Literals;

   --  Numbers inside a string: plain decimal digits with an optional fraction and exponent.
   function String_Numbers (S : String) return String_Vectors.Vector is
      R : String_Vectors.Vector;
      I : Natural := S'First;
   begin
      while I <= S'Last loop
         if Is_Digit (S (I)) and then not (I > S'First and then Is_Word (S (I - 1)))
           and then not (I > S'First + 1 and then S (I - 1) = '.' and then Is_Digit (S (I - 2)))
         then
            declare
               J : Natural := I;
            begin
               while J <= S'Last and then Is_Digit (S (J)) loop
                  J := J + 1;
               end loop;
               if J < S'Last and then S (J) = '.' and then Is_Digit (S (J + 1)) then
                  J := J + 1;
                  while J <= S'Last and then Is_Digit (S (J)) loop
                     J := J + 1;
                  end loop;
               end if;
               R.Append (S (I .. J - 1));
               I := J;
            end;
         else
            I := I + 1;
         end if;
      end loop;
      return R;
   end String_Numbers;

   --  X + X written for 2 x X: the same name (optionally indexed once) on both sides.
   function Twice (C : String) return String_Vectors.Vector is
      R : String_Vectors.Vector;
   begin
      for P in C'Range loop
         if C (P) = '+' then
            declare
               L : Natural := P - 1;
               Q : Natural := P + 1;
            begin
               while L >= C'First and then C (L) = ' ' loop
                  L := L - 1;
               end loop;
               while Q <= C'Last and then C (Q) = ' ' loop
                  Q := Q + 1;
               end loop;
               if L >= C'First and then Q <= C'Last then
                  --  left operand: a dotted name, optionally followed by one parenthesized group
                  declare
                     Le : constant Natural := L;
                     Ls : Natural := L;
                  begin
                     if C (Ls) = ')' then
                        declare
                           Depth : Natural := 0;
                        begin
                           while Ls >= C'First loop
                              if C (Ls) = ')' then
                                 Depth := Depth + 1;
                              elsif C (Ls) = '(' then
                                 Depth := Depth - 1;
                                 exit when Depth = 0;
                              end if;
                              Ls := Ls - 1;
                           end loop;
                           Ls := Ls - 1;
                           while Ls >= C'First and then C (Ls) = ' ' loop
                              Ls := Ls - 1;
                           end loop;
                        end;
                     end if;
                     while Ls >= C'First and then (Is_Word (C (Ls)) or else C (Ls) = '.') loop
                        Ls := Ls - 1;
                     end loop;
                     Ls := Ls + 1;
                     if Ls <= Le and then Is_Letter (C (Ls)) and then not (Ls > C'First and then C (Ls - 1) = ''') then
                        declare
                           Left : constant String := Normalized (C (Ls .. Le));
                        begin
                           if Q + Left'Length - 1 <= C'Last then
                              declare
                                 Right_End : Natural := Q;
                              begin
                                 while Right_End <= C'Last and then (Is_Word (C (Right_End)) or else C (Right_End) = '.'
                                                                     or else C (Right_End) = ' ' or else C (Right_End) = '('
                                                                     or else C (Right_End) = ')' or else C (Right_End) = ',')
                                 loop
                                    exit when Normalized (C (Q .. Right_End)) = Left;
                                    Right_End := Right_End + 1;
                                 end loop;
                                 if Right_End <= C'Last and then Normalized (C (Q .. Right_End)) = Left
                                   and then (Right_End = C'Last or else not (Is_Word (C (Right_End + 1))
                                                                             or else C (Right_End + 1) in '.' | '('))
                                 then
                                    R.Append ("2x" & Left);
                                 end if;
                              end;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
      return R;
   end Twice;

   --  Whether the literal at S .. E is the last argument of Image (X, N): digits printed.
   function Digits_Argument (C : String; S, E : Natural) return Boolean is
      Bracket : Character;
      Open_At : Natural;
   begin
      Opener (C, S, Bracket, Open_At);
      if Bracket /= '(' then
         return False;
      end if;
      declare
         B : constant String := Trim (C (C'First .. Open_At - 1));
      begin
         return B'Length >= 5 and then B (B'Last - 4 .. B'Last) = "Image"
           and then (B'Length = 5 or else not Is_Word (B (B'Last - 5)))
           and then Ada.Strings.Fixed.Trim (C (C'First .. S - 1), Ada.Strings.Right)'Length > 0
           and then Trim (C (C'First .. S - 1)) (Trim (C (C'First .. S - 1))'Last) = ','
           and then Trim (C (E + 1 .. C'Last))'Length > 0 and then Trim (C (E + 1 .. C'Last)) (1) = ')';
      end;
   end Digits_Argument;

   --  Registry rows and occurrences, keyed by file, normalized line and literal.
   type Row is record
      Category, Why : Unbounded_String;
   end record;

   package Row_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Row);
   package Row_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Row_Vectors.Vector, "<", Row_Vectors."=");
   package Count_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Natural);
   package Line_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Positive);

   Sep : constant Character := ASCII.HT;

   Found, Formats : Count_Maps.Map;
   Where : Line_Maps.Map;

   procedure Count_Occurrence (File, Key, Literal : String; Line_Number : Positive; Is_Format : Boolean) is
      K : constant String := File & Sep & Key & Sep & Literal;
   begin
      if Found.Contains (K) then
         Found.Replace (K, Found (K) + 1);
      else
         Found.Insert (K, 1);
         Where.Insert (K, Line_Number);
      end if;
      if Is_Format then
         if Formats.Contains (K) then
            Formats.Replace (K, Formats (K) + 1);
         else
            Formats.Insert (K, 1);
         end if;
      end if;
   end Count_Occurrence;

   procedure Scan_File (Path : String) is
      F : File_Type;
      Header_End : Integer := -1;
   begin
      Current := To_Unbounded_String (Base (Path));
      Lines.Clear;
      Open (F, In_File, Path);
      while not End_Of_File (F) loop
         Lines.Append (Get_Line (F));
      end loop;
      Close (F);
      --  A separate body repeats its parent's parameter list; its numbers count once, in the parent.
      for I in 1 .. Natural (Lines.Length) loop
         declare
            Code : Unbounded_String;
            Strs : String_Vectors.Vector;
         begin
            Split_Code (Lines (I), Code, Strs);
            if Trim (To_String (Code)) /= "" then
               if Ada.Strings.Fixed.Index (To_Lower (Trim (To_String (Code))), "separate") = 1 then
                  declare
                     Depth : Integer := 0;
                     Started : Boolean := False;
                  begin
                     for J in I + 1 .. Natural (Lines.Length) loop
                        Split_Code (Lines (J), Code, Strs);
                        declare
                           C : constant String := To_Lower (To_String (Code));
                        begin
                           if not Started and then (Ada.Strings.Fixed.Index (C, "procedure") > 0
                                                    or else Ada.Strings.Fixed.Index (C, "function") > 0)
                           then
                              Started := True;
                           end if;
                           if Started then
                              for K in C'Range loop
                                 if C (K) = '(' then
                                    Depth := Depth + 1;
                                 elsif C (K) = ')' then
                                    Depth := Depth - 1;
                                 elsif Depth = 0 and then K + 1 <= C'Last and then C (K .. K + 1) = "is"
                                   and then (K = C'First or else not Is_Word (C (K - 1)))
                                   and then (K + 2 > C'Last or else not Is_Word (C (K + 2)))
                                 then
                                    Header_End := J;
                                    exit;
                                 end if;
                              end loop;
                              exit when Header_End >= 0;
                           end if;
                        end;
                     end loop;
                  end;
               end if;
               exit;
            end if;
         end;
      end loop;
      for I in 1 .. Natural (Lines.Length) loop
         Line_Index := I;
         if I > Header_End then
            declare
               Code : Unbounded_String;
               Strs : String_Vectors.Vector;
            begin
               Split_Code (Lines (I), Code, Strs);
               declare
                  C   : constant String := To_String (Code);
                  Key : constant String := Normalized (C);
               begin
                  if Trim (C) /= "" or else not Strs.Is_Empty then
                     for O of Literals (C) loop
                        if not Structural (C (O.First .. O.Last), C, O.First, O.Last) then
                           Count_Occurrence (To_String (Current), Key, C (O.First .. O.Last), I,
                                             Digits_Argument (C, O.First, O.Last));
                        end if;
                     end loop;
                     for T of Twice (C) loop
                        Count_Occurrence (To_String (Current), Key, T, I, False);
                     end loop;
                     for S of Strs loop
                        for N of String_Numbers (S) loop
                           Count_Occurrence (To_String (Current), Key, """" & N, I, False);
                        end loop;
                     end loop;
                  end if;
               end;
            end;
         end if;
      end loop;
   end Scan_File;

   Registry : Row_Maps.Map;

   procedure Load_Registry is
      F : File_Type;
   begin
      if not Ada.Directories.Exists (Registry_Path) then
         return;
      end if;
      Open (F, In_File, Registry_Path);
      while not End_Of_File (F) loop
         declare
            L : constant String := Get_Line (F);
            Fields : String_Vectors.Vector;
            First : Natural := L'First;
         begin
            if L'Length > 0 and then L (L'First) /= '#' then
               for I in L'Range loop
                  if L (I) = Sep then
                     Fields.Append (L (First .. I - 1));
                     First := I + 1;
                  end if;
               end loop;
               Fields.Append (L (First .. L'Last));
               if Natural (Fields.Length) >= 5 then
                  declare
                     K : constant String := Fields (1) & Sep & Fields (2) & Sep & Fields (3);
                     V : Row_Vectors.Vector;
                  begin
                     if Registry.Contains (K) then
                        V := Registry (K);
                     end if;
                     V.Append (Row'(Category => To_Unbounded_String (Fields (4)),
                                    Why => To_Unbounded_String (Fields (5))));
                     Registry.Include (K, V);
                  end;
               end if;
            end if;
         end;
      end loop;
      Close (F);
   end Load_Registry;

   --  Teeth: each line is a hole an audit found; the rules must still close it.
   type Tooth is record
      Line : Unbounded_String;
      Literal : Unbounded_String;
      Listed : Boolean;   --  must the literal need a registry row?
   end record;

   function T (L, V : String; Listed : Boolean) return Tooth is
     ((Line => To_Unbounded_String (L), Literal => To_Unbounded_String (V), Listed => Listed));

   Teeth : constant array (Positive range <>) of Tooth :=
     [T ("Small := 4.0 * Geo_Base (C, Arm);", "4.0", True),
      T ("Ns := Natural'Max (4, Natural (Pix.Length) / 8);", "4", True),
      T ("S := Natural'Min (Frames, 6);", "6", True),
      T ("if Selfmap.Blocked (S1, Med, 2, Notch) then", "2", True),
      T ("Px := 1.0 / Long_Float (W);", "1.0", True),
      T ("if C.Map.N_Cams > 1 then", "1", True),
      T ("if Natural (L.Jaw.Length) = 1 then", "1", True),
      T ("Put_Array (S, 1);", "1", True),
      T ("Trip_Px : constant := 1.0;", "1.0", True),
      T ("Retries : Natural := 1;", "1", True),
      T ("Up : constant V3 := [0.0, 0.0, 1.0];", "1.0", True),
      T ("Zero_Grip : constant Long_Float := 0.0;", "0.0", True),
      T ("Send_Jaw (C, 0.0);", "0.0", True),
      T ("N := 1_000;", "1_000", True),
      T ("Eps := 1.0E-6;", "1.0E-6", True),
      T ("Es : array (0 .. 1) of Floats;", "1", True),
      T ("P := Pts (I + 2);", "2", True),
      T ("if abs X > 7.0 then", "7.0", True),
      T ("if abs R <= 1.0 then", "1.0", True),
      T ("Y := X (2) + Z;", "2", False),
      T ("M (I, 2) := 0.0;", "2", False),
      T ("N := N + 1;", "1", False),
      T ("A := [1.0, 0.0, 0.0];", "1.0", False),
      T ("R := 1.0 - F;", "1.0", False),
      T ("S := X ** 2;", "2", False),
      T ("Z := 0.0;", "0.0", False),
      T ("for K in 1 .. N loop", "1", False),
      T ("for K in -1 .. N loop", "1", True),
      T ("Arm : Integer := -1;", "1", True),
      T ("Y := N - 1;", "1", False),
      T ("Z := Pts (K) (J + 1);", "1", False)];

   function Check_Teeth return Natural is
      Bad : Natural := 0;
      Test_Scope : Scope;
   begin
      Add_Words (Test_Scope.Callables, "Put_Array Send_Jaw Fmt");
      Add_Words (Test_Scope.Objects, "X M Pts V S N Z Y R A P Px Es Up Small Ns Retries Eps Blocked");
      File_Scopes.Include ("teeth.adb", Test_Scope);
      Current := To_Unbounded_String ("teeth.adb");
      Packages.Include ("Selfmap");
      Packages.Include ("Plug");
      for Th of Teeth loop
         Lines.Clear;
         Lines.Append (To_String (Th.Line));
         Line_Index := 1;
         declare
            Code : Unbounded_String;
            Strs : String_Vectors.Vector;
         begin
            Split_Code (To_String (Th.Line), Code, Strs);
            declare
               C : constant String := To_String (Code);
               Any, Listed : Boolean := False;
            begin
               for O of Literals (C) loop
                  if C (O.First .. O.Last) = To_String (Th.Literal) then
                     Any := True;
                     if not Structural (C (O.First .. O.Last), C, O.First, O.Last) then
                        Listed := True;
                     end if;
                  end if;
               end loop;
               if not Any then
                  Put_Line ("  tooth failed: literal not found: " & To_String (Th.Literal) & "  " & To_String (Th.Line));
                  Bad := Bad + 1;
               elsif Listed /= Th.Listed then
                  Put_Line ("  tooth failed: " & (if Th.Listed then "should be listed but is not" else
                              "listed though structural") & ": " & To_String (Th.Literal) & "  " & To_String (Th.Line));
                  Bad := Bad + 1;
               end if;
            end;
         end;
      end loop;
      --  A counted loop is not three components; an index loop is.
      declare
         procedure Loop_Case (First_Line, Body_Line : String; Listed : Boolean) is
            Code : Unbounded_String;
            Strs : String_Vectors.Vector;
         begin
            Lines.Clear;
            Lines.Append (First_Line);
            Lines.Append (Body_Line);
            Lines.Append ("end loop;");
            Line_Index := 1;
            Split_Code (First_Line, Code, Strs);
            declare
               C : constant String := To_String (Code);
            begin
               for O of Literals (C) loop
                  if C (O.First .. O.Last) = "2"
                    and then (not Structural ("2", C, O.First, O.Last)) /= Listed
                  then
                     Put_Line ("  tooth failed: loop: " & First_Line);
                     Bad := Bad + 1;
                  end if;
               end loop;
            end;
         end Loop_Case;
      begin
         Loop_Case ("for Try in 0 .. 2 loop", "   Press (C, Arm);", True);
         Loop_Case ("for I in 0 .. 2 loop", "   V (I) := 0.0;", False);
      end;
      if Twice ("if Got + Got < Ln then").Is_Empty or else not Twice ("if Got + Gotten < Ln then").Is_Empty then
         Put_Line ("  tooth failed: X + X not read as 2 x X (or misread)");
         Bad := Bad + 1;
      end if;
      declare
         S : constant String_Vectors.Vector := String_Numbers ("in thousandths of the picture (0..1000), max_tokens:700");
      begin
         if Natural (S.Length) /= 3 or else S (1) /= "0" or else S (2) /= "1000" or else S (3) /= "700" then
            Put_Line ("  tooth failed: numbers inside strings not all found");
            Bad := Bad + 1;
         end if;
      end;
      File_Scopes.Delete ("teeth.adb");
      if Bad = 0 then
         Put_Line ("  checker teeth: all" & Natural'Image (Teeth'Length + 4) & " hold");
      end if;
      return Bad;
   end Check_Teeth;

   Mode : constant String := (if Ada.Command_Line.Argument_Count > 0 then Ada.Command_Line.Argument (1) else "check");
   Files : String_Vectors.Vector;

begin
   Initialize_Words;
   if Check_Teeth > 0 then
      Put_Line ("FAIL: the checker's own rules have loosened");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   if Mode = "teeth" then
      return;
   end if;
   Collect ("driver/src", Files);
   for F of Files loop
      declare
         S : Scope;
      begin
         Scan_Names (F, S);
         File_Scopes.Include (Base (F), S);
         if Ada.Directories.Extension (F) = "ads" then
            Public.Callables.Union (S.Callables);
            Public.Objects.Union (S.Objects);
         end if;
      end;
   end loop;
   --  A body sees its own spec's names.
   for F of Files loop
      if Ada.Directories.Extension (F) = "adb" then
         declare
            Spec : constant String := Base (F) (Base (F)'First .. Base (F)'Last - 1) & "s";
         begin
            if File_Scopes.Contains (Spec) then
               declare
                  S : Scope := File_Scopes (Base (F));
               begin
                  S.Callables.Union (File_Scopes (Spec).Callables);
                  S.Objects.Union (File_Scopes (Spec).Objects);
                  File_Scopes.Replace (Base (F), S);
               end;
            end if;
         end;
      end if;
   end loop;
   for F of Files loop
      Scan_File (F);
   end loop;
   Load_Registry;

   declare
      Missing, Undecided, Tuning, Stale : Natural := 0;
   begin
      if Mode = "gen" then
         declare
            Registry_Out : File_Type;
            Kept : String_Vectors.Vector;
            Appended, Dropped : Natural := 0;
         begin
            for C in Registry.Iterate loop
               declare
                  K : constant String := Row_Maps.Key (C);
                  N : constant Natural := (if Found.Contains (K) then Found (K) else 0);
                  V : constant Row_Vectors.Vector := Row_Maps.Element (C);
               begin
                  for I in 1 .. Natural (V.Length) loop
                     if I <= N then
                        Kept.Append (K & Sep & To_String (V (I).Category) & Sep & To_String (V (I).Why));
                     else
                        Dropped := Dropped + 1;
                     end if;
                  end loop;
               end;
            end loop;
            for C in Found.Iterate loop
               declare
                  K : constant String := Count_Maps.Key (C);
                  Have : constant Natural := (if Registry.Contains (K) then Natural (Registry (K).Length) else 0);
                  Format_Count : constant Natural := (if Formats.Contains (K) then Formats (K) else 0);
                  Format_Have : Natural := 0;
               begin
                  if Registry.Contains (K) then
                     for R of Registry (K) loop
                        if To_String (R.Category) = "format" then
                           Format_Have := Format_Have + 1;
                        end if;
                     end loop;
                  end if;
                  for I in Have + 1 .. Count_Maps.Element (C) loop
                     Appended := Appended + 1;
                     if I - Have <= Integer'Max (0, Format_Count - Format_Have) then
                        Kept.Append (K & Sep & "format" & Sep & "digits printed in a log line or a message");
                     else
                        Kept.Append (K & Sep & "undecided" & Sep & Undecided_Why);
                     end if;
                  end loop;
               end;
            end loop;
            --  Sorted by file, line and literal, as the rows are compared by key.
            declare
               package Sorting is new String_Vectors.Generic_Sorting;
            begin
               Sorting.Sort (Kept);
            end;
            Create (Registry_Out, Ada.Text_IO.Out_File, Registry_Path);
            Put_Line (Registry_Out, Header);
            for L of Kept loop
               Put_Line (Registry_Out, L);
            end loop;
            Close (Registry_Out);
            Put_Line ("appended" & Appended'Image & " rows, dropped" & Dropped'Image & " rows whose literal is gone");
         end;
         return;
      end if;
      for C in Found.Iterate loop
         declare
            K : constant String := Count_Maps.Key (C);
            Have : constant Natural := (if Registry.Contains (K) then Natural (Registry (K).Length) else 0);
         begin
            if Count_Maps.Element (C) > Have then
               Missing := Missing + Count_Maps.Element (C) - Have;
               if Missing <= 40 then
                  Put_Line ("  unlisted: " & K (K'First .. Ada.Strings.Fixed.Index (K, "" & Sep) - 1) & ":"
                            & Ada.Strings.Fixed.Trim (Positive'Image (Where (K)), Ada.Strings.Both) & "  "
                            & K (Ada.Strings.Fixed.Index (K, "" & Sep, Ada.Strings.Backward) + 1 .. K'Last));
               end if;
            end if;
         end;
      end loop;
      for C in Registry.Iterate loop
         declare
            K : constant String := Row_Maps.Key (C);
            N : constant Natural := (if Found.Contains (K) then Found (K) else 0);
            V : constant Row_Vectors.Vector := Row_Maps.Element (C);
         begin
            if Natural (V.Length) > N then
               Stale := Stale + Natural (V.Length) - N;
            end if;
            if N > 0 then
               for R of V loop
                  if To_String (R.Category) = "tuning" then
                     Tuning := Tuning + 1;
                     Put_Line ("  tuning: " & K);
                  elsif not Category_Ok (To_String (R.Category)) then
                     Undecided := Undecided + 1;
                     if Undecided <= 20 then
                        Put_Line ("  unclassified: " & K);
                     end if;
                  end if;
               end loop;
            end if;
         end;
      end loop;
      Put_Line ("== numeric literals:" & Missing'Image & " unlisted," & Undecided'Image & " unclassified,"
                & Tuning'Image & " tuning," & Stale'Image & " listed but gone ==");
      if Missing > 0 then
         Put_Line ("FAIL: every literal needs its origin in driver/numbers.tsv (numbers gen, then classify)");
      end if;
      if Undecided > 0 then
         Put_Line ("FAIL: classify every row as structure / math / numerics / statistics / format");
      end if;
      if Tuning > 0 then
         Put_Line ("FAIL: tuning numbers are not allowed; derive the quantity from a measurement");
      end if;
      if Stale > 0 then
         Put_Line ("  (" & Ada.Strings.Fixed.Trim (Stale'Image, Ada.Strings.Both)
                   & " listed rows no longer occur; numbers gen drops them)");
      end if;
      if Missing + Undecided + Tuning = 0 then
         Put_Line ("PASS: no tuning numbers");
      else
         Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      end if;
   end;
end Numbers;
