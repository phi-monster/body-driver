with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Driver.Brain.Words;

package body Driver.Brain.Keyboard is

   use Ada.Strings.Unbounded;
   use Driver.Action;
   use Driver.Brain.Words;

   NL : constant String := [ASCII.LF];

   --  The relations a constraint can carry with a thing after it, in the
   --  generic form: still, open, close and press have forms of their own.
   function Generic_Relation (R : Relation) return Boolean is (R not in Still | Open | Close | Press);

   function Choose
     (Quantities       : Word_Vectors.Vector;
      Meanings         : Word_Vectors.Vector;
      Roles            : Role_Set;
      Relations        : Relation_Set;
      Surface_Measured : Boolean;
      Two_Things       : Relation_Set;
      Eyes             : Eye_Vectors.Vector) return Keyboard
   is
      K : Keyboard;
   begin
      for E in Ending loop
         K.Endings (E) := Can_Wait_For (E) and then (E /= Free or else Surface_Measured);
      end loop;
      if Roles (Grasper) and then not Quantities.Is_Empty then
         K.Keys := Quantity_Keys;
         K.Quantities := Quantities;
         K.Meanings := Meanings;
         K.Relations := [for R in Relation => Two_Things (R) and then Generic_Relation (R)];
      elsif (for some R in Role => Roles (R)) then
         K.Keys := Full_Keys;
         K.Roles := Roles;
         K.Relations := Relations;
      end if;
      if Eyes.First_Index < Eyes.Last_Index then
         K.Eyes := Eyes;
      end if;
      return K;
   end Choose;

   --  Rendering helpers.

   function Has_Any (S : Relation_Set) return Boolean is (for some R in Relation => S (R));

   function Eye_Number (E : Driver.Observations.Camera_Id) return String is
     (Ada.Strings.Fixed.Trim (Driver.Observations.Camera_Id'Image (E), Ada.Strings.Both));

   function Eye_Numbers (K : Keyboard) return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for E of K.Eyes loop
         R.Append (Eye_Number (E));
      end loop;
      return R;
   end Eye_Numbers;

   function Has_Generic (S : Relation_Set) return Boolean is
     (for some R in Relation => S (R) and then Generic_Relation (R));

   function Quoted (S : String) return String is ("""" & S & """");

   function Or_List (Items : Word_Vectors.Vector; Quote : Boolean; Sep : String) return String is
      R : Unbounded_String;
   begin
      for W of Items loop
         Append (R, (if Length (R) > 0 then Sep else "") & (if Quote then Quoted (W) else W));
      end loop;
      return To_String (R);
   end Or_List;

   function Ending_Words (K : Keyboard; Only_Waitable : Boolean) return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for E in Ending loop
         if K.Endings (E) or else (not Only_Waitable and then (E /= Free or else K.Endings (Free))) then
            R.Append (Word (E));
         end if;
      end loop;
      return R;
   end Ending_Words;

   function Relation_Words (S : Relation_Set; Generic_Only : Boolean) return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for X in Relation loop
         if S (X) and then (not Generic_Only or else Generic_Relation (X)) then
            R.Append (Word (X));
         end if;
      end loop;
      return R;
   end Relation_Words;

   function Role_Words (S : Role_Set) return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for X in Role loop
         if S (X) then
            R.Append (Word (X));
         end if;
      end loop;
      return R;
   end Role_Words;

   function Step_Words return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for S in Small .. Large loop
         R.Append (Word (S));
      end loop;
      return R;
   end Step_Words;

   function Effort_Words return Word_Vectors.Vector is
      R : Word_Vectors.Vector;
   begin
      for E in Light .. Hard loop
         R.Append (Word (E));
      end loop;
      return R;
   end Effort_Words;

   --  GBNF. Literal text is plain ASCII here, so quoting needs no escapes but
   --  the line break, written \n.

   Line_Break : constant String := "\n";

   Sentence_Rule : constant String := "sent ::= [a-zA-Z] ([a-zA-Z0-9 ,.\'])*";
   --  No "=" in a free sentence: the look key is the only way to type one.

   function Word_Rules (K : Keyboard) return String is
     ("word ::= " & Quoted (Say_Word & " ") & " sent"
      & (if K.Eyes.Is_Empty then "" else " | " & Quoted (Say_Word & " " & Look_Sign & " ") & " eyeno")
      & " | " & Quoted (Done_Word) & NL
      & Sentence_Rule & NL
      & (if K.Eyes.Is_Empty then "" else "eyeno ::= " & Or_List (Eye_Numbers (K), True, " | ") & NL));

   Placeholder_Name_Word : constant String := "[a-z] ([a-z])*";

   function Render (K : Keyboard; Name_Word_Rule : String) return String is
      G : Unbounded_String;

      procedure Rule (S : String) is
      begin
         Append (G, S & NL);
      end Rule;

      function Spaced (Items : Word_Vectors.Vector) return String is
         R : Unbounded_String;
      begin
         for W of Items loop
            Append (R, (if Length (R) > 0 then " | " else "") & Quoted (" " & W));
         end loop;
         return To_String (R);
      end Spaced;
   begin
      case K.Keys is
         when Speech_Only =>
            Rule ("root ::= line (line)*");
            Rule ("line ::= word " & Quoted (Line_Break));
            Append (G, Word_Rules (K));

         when Quantity_Keys =>
            Rule ("root ::= line (line)*");
            Rule ("line ::= (change" & (if Has_Any (K.Relations) then " | placing" else "") & " | word) "
                  & Quoted (Line_Break));
            Rule ("change ::= " & Quoted (Do_Word & " ") & " name " & Quoted (" ") & " qty " & Quoted (" ")
                  & " dir " & Quoted (" " & Until_Word & " ") & " outc");
            if Has_Any (K.Relations) then
               Rule ("placing ::= " & Quoted (Do_Word & " ") & " name " & Quoted (" ") & " rel " & Quoted (" ")
                     & " name " & Quoted (" " & Until_Word & " ") & " outc");
               Rule ("rel ::= " & Or_List (Relation_Words (K.Relations, Generic_Only => False), True, " | "));
            end if;
            Rule ("qty ::= " & Or_List (K.Quantities, True, " | "));
            Rule ("dir ::= " & Quoted (Up_Word) & " | " & Quoted (Down_Word));
            Rule ("outc ::= " & Or_List (Ending_Words (K, Only_Waitable => True), True, " | "));
            Rule ("name ::= w (" & Quoted (" ") & " w)*");
            Rule ("w ::= " & Name_Word_Rule);
            Append (G, Word_Rules (K));

         when Full_Keys =>
            declare
               Generic_Form : constant Boolean := Has_Generic (K.Relations);
               Who          : constant String := "who";
               Forms        : Unbounded_String;
            begin
               if Generic_Form then
                  Append (Forms, Who & " " & Quoted (" ") & " rel " & Quoted (" ") & " what (step)?");
               end if;
               if K.Relations (Press) then
                  Append (Forms, (if Length (Forms) > 0 then " | " else "") & Who & " "
                          & Quoted (" " & Word (Press) & " ") & " what " & Quoted (" ") & " effort");
               end if;
               Append (Forms, (if Length (Forms) > 0 then " | " else "") & Who & " "
                       & Quoted (" " & Word (Close) & " ") & " what | " & Who & " " & Quoted (" " & Word (Open))
                       & " | " & Who & " " & Quoted (" " & Word (Still)));
               Rule ("root ::= line (line)*");
               Rule ("line ::= (interval | control | decl | word) " & Quoted (Line_Break));
               Rule ("interval ::= " & Quoted (Do_Word & " ") & " cons (" & Quoted (" " & And_Word & " ")
                     & " cons)* " & Quoted (" " & Until_Word & " ") & " outc ("
                     & Quoted (" " & Or_Word & " ") & " num " & Quoted (" " & Steps_Word) & ")? (eye)?");
               Rule ("eye ::= " & Quoted (" " & With_Word & " " & My_Word & " " & Still_Word & " " & Eye_Word)
                     & " | " & Quoted (" " & With_Word & " " & My_Word & " " & Moving_Word & " " & Eye_Word));
               Rule ("cons ::= " & To_String (Forms));
               Rule ("who ::= " & Or_List (Role_Words (K.Roles), True, " | "));
               Rule ("what ::= name | who");
               if Generic_Form then
                  Rule ("rel ::= " & Or_List (Relation_Words (K.Relations, Generic_Only => True), True, " | "));
                  Rule ("step ::= " & Spaced (Step_Words));
               end if;
               if K.Relations (Press) then
                  Rule ("effort ::= " & Or_List (Effort_Words, True, " | "));
               end if;
               Rule ("outc ::= " & Or_List (Ending_Words (K, Only_Waitable => True), True, " | "));
               Rule ("outcome ::= " & Or_List (Ending_Words (K, Only_Waitable => False), True, " | "));
               Rule ("num ::= [1-9] ([0-9])*");
               Rule ("name ::= w (" & Quoted (" ") & " w)*");
               Rule ("w ::= " & Name_Word_Rule);
               Rule ("control ::= " & Quoted (Repeat_Word & " ") & " num " & Quoted (" " & Times_Word & ":" & Line_Break)
                     & " line (line)* " & Quoted (End_Word)
                     & " | " & Quoted (If_Word & " ") & " outcome " & Quoted (":" & Line_Break)
                     & " line (line)* (" & Quoted (Else_Word & ":" & Line_Break) & " line (line)*)? "
                     & Quoted (End_Word)
                     & " | " & Quoted (Try_Word & ":" & Line_Break) & " line (line)* "
                     & Quoted (Or_Word & ":" & Line_Break) & " line (line)* " & Quoted (End_Word));
               Rule ("decl ::= " & Quoted (To_Word & " ") & " name " & Quoted (":" & Line_Break)
                     & " line (line)* " & Quoted (End_Word) & " | " & Quoted (Run_Word & " ") & " name | "
                     & Quoted (Remember_Word & " " & Where_Word & " ") & " what "
                     & Quoted (" " & Is_Word & " " & As_Word & " ") & " name");
               Append (G, Word_Rules (K));
            end;
      end case;
      return To_String (G);
   end Render;

   --  Every maximal run of letters inside the quoted literals of a grammar,
   --  each once. An escape such as \n is not a word.
   function Literal_Words (G : String) return Word_Vectors.Vector is
      R         : Word_Vectors.Vector;
      I         : Natural := G'First;
      In_Quotes : Boolean := False;
      Escaped   : Boolean := False;   --  the previous character was a backslash
   begin
      while I <= G'Last loop
         if Escaped then
            Escaped := False;
            I := I + 1;
         elsif G (I) = '"' then
            In_Quotes := not In_Quotes;
            I := I + 1;
         elsif In_Quotes and then G (I) = '\' then
            Escaped := True;
            I := I + 1;
         elsif In_Quotes and then G (I) in 'a' .. 'z' then
            declare
               J : Natural := I;
            begin
               while J <= G'Last and then G (J) in 'a' .. 'z' loop
                  J := J + 1;
               end loop;
               if not R.Contains (G (I .. J - 1)) then
                  R.Append (G (I .. J - 1));
               end if;
               I := J;
            end;
         else
            I := I + 1;
         end if;
      end loop;
      return R;
   end Literal_Words;

   function Name_Words (K : Keyboard) return Word_Vectors.Vector is
      R : Word_Vectors.Vector := Literal_Words (Render (K, Placeholder_Name_Word));
   begin
      if not R.Contains (Item_Word) then
         R.Append (Item_Word);
      end if;
      return R;
   end Name_Words;

   --  Every non-empty string of lower-case letters except the words in
   --  Forbidden, as a GBNF expression: a trie of the forbidden words where
   --  each letter that leaves the trie may continue freely, and a string may
   --  stop at a node only if that node is not the end of a forbidden word.
   function Complement (Forbidden : Word_Vectors.Vector) return String is

      function Node (Prefix : String) return String is
         Alternatives : Unbounded_String;
         Free_Letters : Unbounded_String;
         Is_Word_End  : constant Boolean := Forbidden.Contains (Prefix);

         function Continues (C : Character) return Boolean is
           (for some W of Forbidden => W'Length > Prefix'Length
              and then W (W'First .. W'First + Prefix'Length - 1) = Prefix
              and then W (W'First + Prefix'Length) = C);
      begin
         for C in Character range 'a' .. 'z' loop
            if Continues (C) then
               Append (Alternatives, (if Length (Alternatives) > 0 then " | " else "")
                       & Quoted ([C]) & " " & Node (Prefix & C));
            else
               Append (Free_Letters, C);
            end if;
         end loop;
         if Length (Free_Letters) > 0 then
            Alternatives := "[" & Free_Letters & "] ([a-z])*"
              & (if Length (Alternatives) > 0 then " | " & Alternatives else Null_Unbounded_String);
         end if;
         --  A string may end here unless it would be a forbidden word, and
         --  it may not be empty.
         return "(" & To_String (Alternatives) & ")"
           & (if Prefix'Length > 0 and then not Is_Word_End then "?" else "");
      end Node;

   begin
      return Node ("");
   end Complement;

   function Grammar (K : Keyboard) return String is (Render (K, Complement (Name_Words (K))));

   --  The sheet.

   function Meaning (E : Ending) return String is
     (case E is
         when Arrived  => "what you asked for holds",
         when Touched  => "I touched something",
         when Stuck    => "I was commanded and did not move",
         when Slipped  => "what I was holding left my hand",
         when Lost     => "I can no longer see what I was following",
         when Free     => "the thing left the surface it was resting on",
         when Settled  => "the picture stopped changing",
         when Stalled  => "I keep moving but the gap stopped shrinking",
         when Timeout  => "I took every step the stretch allowed",
         when Refused  => "I cannot do it, and I say what I tried");

   function Meaning (R : Relation) return String is
     (case R is
         when Touching => "against it",
         when Above    => "above it",
         when Below    => "below it",
         when Left     => "to its left in the picture",
         when Right    => "to its right in the picture",
         when Nearer   => "nearer to the eye that sees it best",
         when Farther  => "farther from the eye that sees it best",
         when Onto     => "pressed onto the surface it rests on",
         when Off      => "off that surface",
         when Into     => "inside it, halfway between its skin and the surface it stands on",
         when Facing   => "turned until this part points at it",
         when Clear    => "never closer to it than now",
         when Still    => "not moving during this stretch",
         when Press    => "pushing against it, saying only how hard",
         when Close    => "closing on it, or where it is when nothing is named",
         when Open     => "opening");

   --  The same words in the sentence about two things, where both sides are
   --  things and the first one is the one that moves.
   function Placing_Meaning (R : Relation) return String is
     (case R is
         when Touching => "the first thing ends up against the second",
         when Above    => "the first thing ends up above the second",
         when Below    => "the first thing ends up below the second",
         when Left     => "the first thing ends up to the left of the second, as the large picture shows them",
         when Right    => "the first thing ends up to the right of the second, as the large picture shows them",
         when Nearer   => "the first thing ends up nearer than the second to the eye that sees them best",
         when Farther  => "the first thing ends up farther than the second from the eye that sees them best",
         when Onto     => "the first thing ends up pressed onto the second",
         when Off      => "the first thing ends up off the second",
         when Into     => "the first thing ends up inside the second",
         when Facing   => "the first thing ends up turned to point at the second",
         when Clear    => "the first thing never comes closer to the second than now",
         when Still | Press | Close | Open => Meaning (R));

   function Glosses (Items : Word_Vectors.Vector; Meanings : Word_Vectors.Vector; Indent : String) return String is
      R : Unbounded_String;
   begin
      for I in Items.First_Index .. Items.Last_Index loop
         declare
            M : constant String := Meanings (I);
         begin
            if M'Length > 0 then
               Append (R, Indent & Items (I) & " = " & M & NL);
            end if;
         end;
      end loop;
      return To_String (R);
   end Glosses;

   function Ending_Glosses (K : Keyboard; Indent : String) return String is
      R : Unbounded_String;
   begin
      for E in Ending loop
         if K.Endings (E) then
            Append (R, Indent & Word (E) & " = " & Meaning (E) & NL);
         end if;
      end loop;
      return To_String (R);
   end Ending_Glosses;

   function Relation_Glosses (S : Relation_Set; Two_Things : Boolean; Indent : String) return String is
      R : Unbounded_String;
   begin
      for X in Relation loop
         if S (X) and then (Two_Things or else Generic_Relation (X)) then
            Append (R, Indent & Word (X) & " = " & (if Two_Things then Placing_Meaning (X) else Meaning (X)) & NL);
         end if;
      end loop;
      return To_String (R);
   end Relation_Glosses;

   Name_Rule_Text : constant String :=
     "plain words, as many as the name needs; none of the words of this grammar, and not the word " & Item_Word;

   Gloss_Indent : constant String := "      ";

   --  The speech keys and what done does on this keyboard. Word_Head: how
   --  this sheet writes the head of the <word> line, aligned with its other
   --  lines.
   function Speech_Sheet (K : Keyboard; Word_Head : String; Done_Means : String) return String is
      Eye_Slot : constant String := "<eye number>";
   begin
      return Word_Head & Say_Word & " <one sentence in your own words>"
        & (if K.Eyes.Is_Empty then "" else " | " & Say_Word & " " & Look_Sign & " " & Eye_Slot)
        & " | " & Done_Word & NL
        & (if K.Eyes.Is_Empty then ""
           else Eye_Slot & " ::= " & Or_List (Eye_Numbers (K), False, " | ")
                & "   (from the next round on, the large picture is what that eye sees)" & NL)
        & Gloss_Indent & Done_Word & " = the program ends here; " & Done_Means
        & ", it also tells me the task is finished, and I ask you nothing more for this task" & NL;
   end Speech_Sheet;

   function Sheet (K : Keyboard) return String is
      S : Unbounded_String;

      procedure Put (Line : String) is
      begin
         Append (S, Line & NL);
      end Put;

      No_Change_Before : constant String := "if no change ran before it";
   begin
      case K.Keys is
         when Speech_Only =>
            Put ("<program>  ::= <line> (<line>)*");
            Put ("<line>     ::= <word>");
            Append (S, Speech_Sheet (K, "<word>     ::= ", No_Change_Before));
            Put ("No part of me can be commanded this round, so a program only speaks.");

         when Quantity_Keys =>
            Put ("<program>  ::= <line> (<line>)*");
            Put ("<line>     ::= <change>" & (if Has_Any (K.Relations) then " | <placing>" else "") & " | <word>");
            Put ("<change>   ::= do <thing> <quantity> <direction> until <ending>");
            if Has_Any (K.Relations) then
               Put ("<placing>  ::= do <thing> <relation> <thing> until <ending>");
            end if;
            Put ("<thing>    ::= the name of a thing you see (" & Name_Rule_Text & ")");
            Put ("<quantity> ::= " & Or_List (K.Quantities, False, " | ")
                 & "   (a quantity of that thing that I measure and can change)");
            Append (S, Glosses (K.Quantities, K.Meanings, Gloss_Indent));
            Put ("<direction> ::= " & Up_Word & " | " & Down_Word);
            if Has_Any (K.Relations) then
               Put ("<relation> ::= " & Or_List (Relation_Words (K.Relations, Generic_Only => False), False, " | ")
                    & "   (where the first thing ends up, relative to the second)");
               Append (S, Relation_Glosses (K.Relations, Two_Things => True, Indent => Gloss_Indent));
            end if;
            Put ("<ending>   ::= " & Or_List (Ending_Words (K, Only_Waitable => True), False, " | "));
            Append (S, Ending_Glosses (K, Gloss_Indent));
            Append (S, Speech_Sheet (K, "<word>     ::= ", No_Change_Before));
            Put ("A change names what should happen to the thing; I choose where to hold it and how to move it.");

         when Full_Keys =>
            Put ("<program>    ::= <line> (<line>)*");
            Put ("<line>       ::= <interval> | <control> | <decl> | <word>");
            Put ("<interval>   ::= do <constraint> (and <constraint>)* until <ending> [or <n> steps] [<eye>]");
            Put ("<eye>        ::= with my still eye | with my moving eye");
            declare
               Forms : Unbounded_String;
            begin
               if Has_Generic (K.Relations) then
                  Append (Forms, "<who> <relation> <what> [<step>]");
               end if;
               if K.Relations (Press) then
                  Append (Forms, (if Length (Forms) > 0 then " | " else "") & "<who> press <what> <effort>");
               end if;
               Append (Forms, (if Length (Forms) > 0 then " | " else "")
                       & "<who> close <what> | <who> open | <who> still");
               Put ("<constraint> ::= " & To_String (Forms));
            end;
            Put ("<who>        ::= " & Or_List (Role_Words (K.Roles), False, " | ")
                 & "   (parts of me, bound by what I measured)");
            Put ("<what>       ::= <a name> | <who>");
            Put ("<name>       ::= " & Name_Rule_Text);
            if Has_Generic (K.Relations) then
               Put ("<relation>   ::= " & Or_List (Relation_Words (K.Relations, Generic_Only => True), False, " | "));
               Append (S, Relation_Glosses (K.Relations, Two_Things => False, Indent => Gloss_Indent));
               Put ("<step>       ::= " & Or_List (Step_Words, False, " | ")
                    & "   (the most one step may move)");
            end if;
            if K.Relations (Press) then
               Put ("<effort>     ::= " & Or_List (Effort_Words, False, " | "));
            end if;
            Put ("<ending>     ::= " & Or_List (Ending_Words (K, Only_Waitable => True), False, " | ")
                 & "   (what a stretch waits for)");
            Append (S, Ending_Glosses (K, Gloss_Indent));
            Put ("<outcome>    ::= " & Or_List (Ending_Words (K, Only_Waitable => False), False, " | ")
                 & "   (how the last stretch ended)");
            for E in Ending loop
               if not K.Endings (E) and then (E /= Free or else K.Endings (Free)) then
                  Put (Gloss_Indent & Word (E) & " = " & Meaning (E));
               end if;
            end loop;
            Put ("<control>    ::= repeat <n> times: <line> (<line>)* end");
            Put ("               | if <outcome>: <line> (<line>)* [else: <line> (<line>)*] end");
            Put ("               | try: <line> (<line>)* or: <line> (<line>)* end");
            Put ("<decl>       ::= to <name>: <line> (<line>)* end | run <name> | remember where <what> is as <name>");
            Append (S, Speech_Sheet (K, "<word>       ::= ",
                                     "if no stretch ran before it, or each one ran inside a try or was followed by an if"));
      end case;
      return To_String (S);
   end Sheet;

end Driver.Brain.Keyboard;
