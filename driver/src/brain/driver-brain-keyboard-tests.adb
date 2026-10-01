with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Vectors;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Driver.Action;
with Driver.Tests;

package body Driver.Brain.Keyboard.Tests is

   use Ada.Strings.Unbounded;
   use Driver.Action;
   use Driver.Tests;

   --  A GBNF reader for the subset the driver writes: rules "name ::= alt",
   --  alternatives with |, sequences, groups, * and ?, "literals" with \n,
   --  [classes] with ranges and escapes, and rule names.

   type Node_Kind is (Literal, Class, Sequence, Choice, Star, Optional, Reference);
   type Char_Set is array (Character) of Boolean;
   type Node_Id is new Positive;
   package Id_Vectors is new Ada.Containers.Vectors (Positive, Node_Id);

   type Node is record
      Kind  : Node_Kind := Literal;
      Text  : Unbounded_String;          --  the literal, or the referenced rule's name
      Set   : Char_Set := [others => False];
      Kids  : Id_Vectors.Vector;
   end record;

   package Node_Vectors is new Ada.Containers.Vectors (Node_Id, Node);
   package Rule_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Node_Id);

   type Parsed is record
      Nodes : Node_Vectors.Vector;
      Rules : Rule_Maps.Map;
      Good  : Boolean := True;
   end record;

   function Read_Grammar (G : String) return Parsed is
      P : Parsed;

      procedure Read_Rule (Line : String) is
         Sep : constant Natural := Ada.Strings.Fixed.Index (Line, " ::= ");
         I   : Natural;

         function Add (N : Node) return Node_Id is
         begin
            P.Nodes.Append (N);
            return P.Nodes.Last_Index;
         end Add;

         procedure Skip is
         begin
            while I <= Line'Last and then Line (I) = ' ' loop
               I := I + 1;
            end loop;
         end Skip;

         function Alternatives return Node_Id;

         function Atom return Node_Id is
            N : Node;
         begin
            Skip;
            if I > Line'Last then
               P.Good := False;
               return Add (N);
            end if;
            case Line (I) is
               when '"' =>
                  I := I + 1;
                  while I <= Line'Last and then Line (I) /= '"' loop
                     if Line (I) = '\' and then I < Line'Last then
                        Append (N.Text, (if Line (I + 1) = 'n' then ASCII.LF else Line (I + 1)));
                        I := I + 2;
                     else
                        Append (N.Text, Line (I));
                        I := I + 1;
                     end if;
                  end loop;
                  I := I + 1;
                  N.Kind := Literal;
               when '[' =>
                  I := I + 1;
                  N.Kind := Class;
                  while I <= Line'Last and then Line (I) /= ']' loop
                     declare
                        C : Character := Line (I);
                     begin
                        if C = '\' then
                           I := I + 1;
                           C := Line (I);
                        end if;
                        if I + 2 <= Line'Last and then Line (I + 1) = '-' and then Line (I + 2) /= ']' then
                           for X in C .. Line (I + 2) loop
                              N.Set (X) := True;
                           end loop;
                           I := I + 3;
                        else
                           N.Set (C) := True;
                           I := I + 1;
                        end if;
                     end;
                  end loop;
                  I := I + 1;
               when '(' =>
                  I := I + 1;
                  declare
                     Inner : constant Node_Id := Alternatives;
                  begin
                     Skip;
                     if I > Line'Last or else Line (I) /= ')' then
                        P.Good := False;
                     end if;
                     I := I + 1;
                     return Inner;
                  end;
               when 'a' .. 'z' | 'A' .. 'Z' =>
                  N.Kind := Reference;
                  while I <= Line'Last and then Line (I) in 'a' .. 'z' | 'A' .. 'Z' | '0' .. '9' | '_' loop
                     Append (N.Text, Line (I));
                     I := I + 1;
                  end loop;
               when others =>
                  P.Good := False;
                  I := I + 1;
            end case;
            return Add (N);
         end Atom;

         function Item return Node_Id is
            A : constant Node_Id := Atom;
         begin
            if I <= Line'Last and then Line (I) in '*' | '?' then
               declare
                  N : Node;
               begin
                  N.Kind := (if Line (I) = '*' then Star else Optional);
                  N.Kids.Append (A);
                  I := I + 1;
                  return Add (N);
               end;
            end if;
            return A;
         end Item;

         function Sequence_Of return Node_Id is
            N : Node;
         begin
            N.Kind := Sequence;
            loop
               Skip;
               exit when I > Line'Last or else Line (I) in '|' | ')';
               N.Kids.Append (Item);
            end loop;
            return Add (N);
         end Sequence_Of;

         function Alternatives return Node_Id is
            N : Node;
         begin
            N.Kind := Choice;
            N.Kids.Append (Sequence_Of);
            loop
               Skip;
               exit when I > Line'Last or else Line (I) /= '|';
               I := I + 1;
               N.Kids.Append (Sequence_Of);
            end loop;
            return Add (N);
         end Alternatives;
      begin
         if Sep = 0 then
            P.Good := False;
            return;
         end if;
         I := Sep + 5;
         declare
            Root : constant Node_Id := Alternatives;
         begin
            if I <= Line'Last then
               P.Good := False;
            end if;
            P.Rules.Include (Line (Line'First .. Sep - 1), Root);
         end;
      end Read_Rule;

      First : Positive := G'First;
   begin
      for J in G'Range loop
         if G (J) = ASCII.LF then
            if J > First then
               Read_Rule (G (First .. J - 1));
            end if;
            First := J + 1;
         end if;
      end loop;
      if First <= G'Last then
         Read_Rule (G (First .. G'Last));
      end if;
      for N of P.Nodes loop
         if N.Kind = Reference and then not P.Rules.Contains (To_String (N.Text)) then
            P.Good := False;
         end if;
      end loop;
      return P;
   end Read_Grammar;

   --  Matching computes, for a node and a start, every position where the
   --  node can end; references are memoized by rule and start.
   function Accepts (Grammar, Text : String) return Boolean is
      P : constant Parsed := Read_Grammar (Grammar);
      L : constant Natural := Text'Length;
      T : constant String (1 .. L) := Text;
      type Position_Set is array (0 .. L) of Boolean;
      None : constant Position_Set := [others => False];
      package Memo_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Position_Set);
      Memo : Memo_Maps.Map;

      function Ends (N : Node_Id; From : Natural) return Position_Set is
         X : constant Node := P.Nodes (N);
         R : Position_Set := None;
      begin
         case X.Kind is
            when Literal =>
               declare
                  S : constant String := To_String (X.Text);
               begin
                  if From + S'Length <= L and then T (From + 1 .. From + S'Length) = S then
                     R (From + S'Length) := True;
                  end if;
               end;
            when Class =>
               if From < L and then X.Set (T (From + 1)) then
                  R (From + 1) := True;
               end if;
            when Sequence =>
               R (From) := True;
               for K of X.Kids loop
                  declare
                     Next : Position_Set := None;
                  begin
                     for Q in R'Range loop
                        if R (Q) then
                           declare
                              E : constant Position_Set := Ends (K, Q);
                           begin
                              for Z in E'Range loop
                                 Next (Z) := Next (Z) or else E (Z);
                              end loop;
                           end;
                        end if;
                     end loop;
                     R := Next;
                  end;
               end loop;
            when Choice =>
               for K of X.Kids loop
                  declare
                     E : constant Position_Set := Ends (K, From);
                  begin
                     for Z in E'Range loop
                        R (Z) := R (Z) or else E (Z);
                     end loop;
                  end;
               end loop;
            when Optional | Star =>
               R (From) := True;
               declare
                  Frontier : Position_Set := R;
               begin
                  loop
                     declare
                        Next : Position_Set := None;
                        Grew : Boolean := False;
                     begin
                        for Q in Frontier'Range loop
                           if Frontier (Q) then
                              declare
                                 E : constant Position_Set := Ends (X.Kids.First_Element, Q);
                              begin
                                 for Z in E'Range loop
                                    if E (Z) and then not R (Z) then
                                       R (Z) := True;
                                       Next (Z) := True;
                                       Grew := True;
                                    end if;
                                 end loop;
                              end;
                           end if;
                        end loop;
                        exit when not Grew or else X.Kind = Optional;
                        Frontier := Next;
                     end;
                  end loop;
               end;
            when Reference =>
               declare
                  Key : constant String := To_String (X.Text) & Natural'Image (From);
               begin
                  if Memo.Contains (Key) then
                     return Memo.Element (Key);
                  end if;
                  R := Ends (P.Rules.Element (To_String (X.Text)), From);
                  Memo.Include (Key, R);
               end;
         end case;
         return R;
      end Ends;
   begin
      return P.Good and then P.Rules.Contains ("root") and then Ends (P.Rules.Element ("root"), 0) (L);
   end Accepts;

   function Well_Formed (Grammar : String) return Boolean is (Read_Grammar (Grammar).Good);

   --  Keyboards used below.

   LF : constant String := [ASCII.LF];

   function Words (A, B : String := "") return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      if A'Length > 0 then
         R.Append (A);
      end if;
      if B'Length > 0 then
         R.Append (B);
      end if;
      return R;
   end Words;

   No_Relation : constant Relation_Set := [others => False];
   All_Relations : constant Relation_Set := [others => True];

   function Eyes_Up_To (Last : Natural) return Eye_Vectors.Vector is
      R : Eye_Vectors.Vector;
   begin
      for E in 1 .. Last loop
         R.Append (Driver.Observations.Camera_Id (E));
      end loop;
      return R;
   end Eyes_Up_To;

   function Hand_Body
     (Surface : Boolean := False; Two_Things : Relation_Set := No_Relation; Eyes : Natural := 0) return Keyboard is
     (Choose (Quantities => Words ("height"), Meanings => Words ("how high it is above what it rests on"),
              Roles => [Grasper => True, others => False], Relations => All_Relations,
              Surface_Measured => Surface, Two_Things => Two_Things, Eyes => Eyes_Up_To (Eyes)));

   function Handless_Body (Eyes : Natural := 0) return Keyboard is
     (Choose (Quantities => Words ("height"), Meanings => Words ("x"),
              Roles => [Me => True, others => False], Relations => All_Relations,
              Surface_Measured => False, Two_Things => No_Relation, Eyes => Eyes_Up_To (Eyes)));

   procedure Choice is
      Speech : constant Keyboard :=
        Choose (Words, Words, [others => False], All_Relations, False, No_Relation, Eyes_Up_To (2));
   begin
      Check (Hand_Body.Keys = Quantity_Keys, "a bound grasper and a quantity give the quantity keyboard");
      Check (Handless_Body.Keys = Full_Keys, "without a grasper the full keyboard");
      Check (Speech.Keys = Speech_Only, "no role bound: only say and done");
      Check (not Hand_Body.Endings (Free) and then Hand_Body (Surface => True).Endings (Free),
             "free is offered only when a surface is measured");
      Check (not Hand_Body.Endings (Arrived) and then not Hand_Body.Endings (Refused)
             and then Hand_Body.Endings (Settled), "arrived and refused are never waited for");
      Check (Accepts (Grammar (Speech), "say I cannot move" & LF & "done" & LF)
             and then not Accepts (Grammar (Speech), "do me above table until settled" & LF),
             "the speech-only keyboard types only say and done");
      Check (Accepts (Grammar (Speech), "say look = 2" & LF), "the speech-only keyboard can switch eyes");
   end Choice;

   procedure Quantity_Grammar is
      G : constant String := Grammar (Hand_Body);
   begin
      Check (Well_Formed (G), "the grammar reads back");
      Check (Accepts (G, "do scissors height up until settled" & LF & "say I am lifting it" & LF)
             and then Accepts (G, "say it is up" & LF & "done" & LF) and then Accepts (G, "done" & LF),
             "the quantity sentence, say, and done after lines that change nothing");
      Check (not Accepts (G, "do scissors height up until settled" & LF & "done" & LF)
             and then not Accepts (G, "do scissors height up until settled" & LF & "say up" & LF & "done" & LF),
             "done cannot claim the task finished after a change whose ending nobody has seen");
      Check (not Accepts (G, "say look = 2" & LF), "no look key without eyes to switch to");
      Check (Accepts (G, "do upmint green scissors height up until touched" & LF),
             "a language word glued to a name word is a name word");
      Check (not Accepts (G, "do up height up until settled" & LF), "a language word alone is not a name word");
      Check (not Accepts (G, "do item height up until settled" & LF), "item is not a name");
      Check (not Accepts (G, "do scissors height up until arrived" & LF), "arrived cannot be typed");
      Check (not Accepts (G, "do scissors height up until free" & LF)
             and then Accepts (Grammar (Hand_Body (Surface => True)), "do scissors height up until free" & LF),
             "free only with a measured surface");
      Check (not Accepts (G, "do scissors above cup until settled" & LF),
             "no sentence about two things unless it is offered");
      Check (Accepts (Grammar (Hand_Body (Two_Things => [Above => True, others => False])),
                      "do the ball above the blue jeans until settled" & LF),
             "the sentence about two things when it is offered");
   end Quantity_Grammar;

   procedure Full_Grammar is
      G : constant String := Grammar (Handless_Body);
      Program : constant String :=
        "to look around:" & LF
        & "remember where me is as start" & LF
        & "repeat 3 times:" & LF
        & "do me above table small until touched or 20 steps with my still eye" & LF
        & "if touched:" & LF
        & "do me farther table until timeout or 5 steps" & LF
        & "else:" & LF
        & "done" & LF
        & "end" & LF
        & "end" & LF
        & "say I looked three times" & LF
        & "end" & LF
        & "run look around" & LF;
   begin
      Check (Well_Formed (G), "the grammar reads back");
      Check (Accepts (G, Program), "a definition with a loop holding an if, as on the sheet");
      Check (Accepts (G, "do me press table firm until stuck" & LF), "press with its effort");
      Check (not Accepts (G, "do me press table until stuck" & LF), "press without an effort cannot be typed");
      Check (Accepts (G, "do me still until settled or 20 steps" & LF), "waiting");
      Check (Accepts (G, "do me touching start until touched" & LF), "a remembered place is a name");
      Check (not Accepts (G, "do me still until settled" & LF & "done" & LF)
             and then not Accepts (G, "repeat 2 times:" & LF & "done" & LF & "end" & LF)
             and then not Accepts (G, "if settled:" & LF & "do me still until settled" & LF & "done" & LF & "end" & LF),
             "done cannot follow a stretch whose ending nobody tested");
      Check (Accepts (G, "do me still until settled" & LF & "if settled:" & LF & "say it rests" & LF & "done" & LF
                      & "end" & LF)
             and then Accepts (G, "try:" & LF & "do me above table until touched" & LF & "done" & LF & "or:" & LF
                               & "say I could not" & LF & "end" & LF)
             and then Accepts (G, "remember where me is as start" & LF & "done" & LF),
             "done where every stretch before it was tested, or nothing moved");
      Check (not Accepts (G, "do grasper touching start until touched" & LF), "a role not bound cannot be typed");
   end Full_Grammar;

   type Slot_Pair is record
      Slot, Rule : Unbounded_String;   --  a slot on the sheet and the grammar rule that types it
   end record;

   type Slot_Pairs is array (Positive range <>) of Slot_Pair;

   function "+" (S : String) return Unbounded_String renames To_Unbounded_String;

   --  The words a sheet line lists after "::=" up to its parenthesis, or the
   --  quoted literals of a grammar rule, sorted.
   function Sorted (V : Word_Vectors.Vector) return Word_Vectors.Vector is
      R : Word_Vectors.Vector := V;
      package Sorting is new Word_Vectors.Generic_Sorting;
   begin
      Sorting.Sort (R);
      return R;
   end Sorted;

   function Line_Starting (Text, Head : String) return String is
      First : Positive := Text'First;
   begin
      for I in Text'Range loop
         if Text (I) = ASCII.LF then
            if I - First >= Head'Length and then Text (First .. First + Head'Length - 1) = Head then
               return Text (First .. I - 1);
            end if;
            First := I + 1;
         end if;
      end loop;
      return "";
   end Line_Starting;

   function Sheet_Words (Line : String) return Word_Vectors.Vector is
      Marker : constant String := "::=";
      R      : Word_Vectors.Vector;
      From   : constant Natural := Ada.Strings.Fixed.Index (Line, Marker);
      Stop   : Natural := Ada.Strings.Fixed.Index (Line, "(");
      W      : Unbounded_String;
   begin
      if From = 0 then
         return R;
      end if;
      if Stop = 0 then
         Stop := Line'Last + 1;
      end if;
      declare
         Listed : constant String := Line (From + Marker'Length .. Stop - 1) & " ";
      begin
         for C of Listed loop
            if C in 'a' .. 'z' | '0' .. '9' then
               Append (W, C);
            elsif Length (W) > 0 then
               R.Append (To_String (W));
               W := Null_Unbounded_String;
            end if;
         end loop;
      end;
      return Sorted (R);
   end Sheet_Words;

   function Rule_Words (Line : String) return Word_Vectors.Vector is
      R         : Word_Vectors.Vector;
      In_Quotes : Boolean := False;
      W         : Unbounded_String;
   begin
      for C of Line loop
         if C = '"' then
            if In_Quotes and then Length (W) > 0 then
               R.Append (Ada.Strings.Fixed.Trim (To_String (W), Ada.Strings.Both));
               W := Null_Unbounded_String;
            end if;
            In_Quotes := not In_Quotes;
         elsif In_Quotes then
            Append (W, C);
         end if;
      end loop;
      return Sorted (R);
   end Rule_Words;

   function Same_Slots (K : Keyboard; Pairs : Slot_Pairs) return Boolean is
      S : constant String := Sheet (K);
      G : constant String := Grammar (K);
   begin
      for P of Pairs loop
         declare
            On_Sheet : constant Word_Vectors.Vector := Sheet_Words (Line_Starting (S, To_String (P.Slot)));
            In_Rule  : constant Word_Vectors.Vector := Rule_Words (Line_Starting (G, To_String (P.Rule) & " ::= "));
         begin
            if On_Sheet.Is_Empty or else Word_Vectors."/=" (On_Sheet, In_Rule) then
               return False;
            end if;
         end;
      end loop;
      return True;
   end Same_Slots;

   procedure Same_Source is
      K     : constant Keyboard := Hand_Body (Two_Things => [Above | Touching => True, others => False]);
      Names : constant Word_Vectors.Vector := Name_Words (K);
      S     : constant String := Sheet (K);
   begin
      for W of Words ("height", "settled") loop
         Check (Names.Contains (W) and then Ada.Strings.Fixed.Index (S, W) > 0,
                W & " is a key on the sheet and a word no name may be");
      end loop;
      Check (Names.Contains ("above") and then Names.Contains ("until") and then Names.Contains ("item")
             and then not Names.Contains ("n"), "the grammar's words, item, and no letter of an escape");
      Check (Ada.Strings.Fixed.Index (S, "height = how high it is") > 0,
             "the sheet glosses a quantity in the words of the side that measures it");
      Check (Ada.Strings.Fixed.Index (Sheet (Hand_Body), "<relation>") = 0,
             "no relation on the sheet when the keyboard has none");
      Check (Same_Slots (Hand_Body (Surface => True, Two_Things => [Above | Left => True, others => False]),
                         [(+"<quantity>", +"qty"), (+"<ending>", +"outc"), (+"<relation>", +"rel")]),
             "quantity keyboard: each slot on the sheet lists exactly the words its grammar rule offers");
      Check (Same_Slots (Handless_Body, [(+"<who>", +"who"), (+"<relation>", +"rel"), (+"<ending>", +"outc"),
                                         (+"<outcome>", +"outcome"), (+"<effort>", +"effort")]),
             "full keyboard: each slot on the sheet lists exactly the words its grammar rule offers");
   end Same_Source;

   procedure Looking is
      Three : constant String := Grammar (Hand_Body (Eyes => 3));
      One   : constant String := Grammar (Hand_Body (Eyes => 1));
   begin
      Check (Accepts (Three, "say look = 2" & LF) and then Accepts (Three, "say look = 3" & LF),
             "the look key types the number of an eye that sees");
      Check (not Accepts (Three, "say look = 4" & LF) and then not Accepts (Three, "say look = 90" & LF),
             "the look key cannot name an eye that does not exist");
      Check (not Accepts (One, "say look = 1" & LF), "a body with one eye has no look key");
      Check (not Accepts (Three, "say the cup is at x=70" & LF) and then Accepts (Three, "say the cup is at x 70" & LF),
             "a free sentence cannot hold =, so it cannot type an eye number of its own");
      Check (Accepts (Grammar (Handless_Body (Eyes => 2)), "say look = 2" & LF & "do me still until settled" & LF),
             "the full keyboard has the look key too");
      Check (Name_Words (Hand_Body (Eyes => 3)).Contains ("look") and then not Name_Words (Hand_Body).Contains ("look"),
             "look is a word of the language exactly when its key is offered");
      Check (Same_Slots (Hand_Body (Eyes => 3), [1 => (+"<eye number>", +"eyeno")]),
             "the sheet lists exactly the eye numbers the grammar offers");
   end Looking;

   procedure Register is
   begin
      Register ("brain.keyboard.choice", "a body gets keys it cannot use, or loses keys it can", Choice'Access);
      Register ("brain.keyboard.look", "the brain can type an eye that does not exist, or cannot switch to one that"
                & " does", Looking'Access);
      Register ("brain.keyboard.quantity", "the quantity keyboard lets through what it must not, or blocks what it"
                & " offers", Quantity_Grammar'Access);
      Register ("brain.keyboard.full", "the full keyboard cannot type a program its sheet describes",
                Full_Grammar'Access);
      Register ("brain.keyboard.same_source", "the sheet, the grammar and the name rules disagree",
                Same_Source'Access);
   end Register;

end Driver.Brain.Keyboard.Tests;
