with Ada.Characters.Handling;
with Ada.Containers.Vectors;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Driver.Action;
with Driver.Brain.Words;

package body Driver.Brain.Parser is

   use Ada.Strings.Unbounded;
   use Driver.Brain.Programs;
   use Driver.Brain.Words;

   subtype Role is Driver.Action.Role;
   subtype Relation is Driver.Action.Relation;
   subtype Ending is Driver.Action.Ending;
   use all type Driver.Action.Role;
   use all type Driver.Action.Relation;
   use all type Driver.Action.Size;
   use all type Driver.Action.Effort;
   use all type Driver.Action.Ending;
   use all type Driver.Action.Eye_Choice;

   type Token is record
      Text : Unbounded_String;   --  as written
      Low  : Unbounded_String;   --  in lower case, for matching words
   end record;

   package Token_Vectors is new Ada.Containers.Vectors (Positive, Token);

   Colon : constant String := ":";

   function Make (S : String) return Token is
     ((Text => To_Unbounded_String (S), Low => To_Unbounded_String (Ada.Characters.Handling.To_Lower (S))));

   function Is_Blank (C : Character) return Boolean is (C = ' ' or else C = ASCII.HT);

   --  The words of a line; a colon ending the last word is a word of its own,
   --  since it marks a block header.
   function Split (Line : String) return Token_Vectors.Vector is
      Result : Token_Vectors.Vector;
      I      : Natural := Line'First;
   begin
      while I <= Line'Last loop
         if Is_Blank (Line (I)) then
            I := I + 1;
         else
            declare
               J : Natural := I;
            begin
               while J <= Line'Last and then not Is_Blank (Line (J)) loop
                  J := J + 1;
               end loop;
               Result.Append (Make (Line (I .. J - 1)));
               I := J;
            end;
         end if;
      end loop;
      if not Result.Is_Empty then
         declare
            Last : constant String := To_String (Result.Last_Element.Text);
         begin
            if Last'Length > Colon'Length and then Last (Last'Last - Colon'Length + 1 .. Last'Last) = Colon then
               Result.Replace_Element (Result.Last_Index, Make (Last (Last'First .. Last'Last - Colon'Length)));
               Result.Append (Make (Colon));
            end if;
         end;
      end if;
      return Result;
   end Split;

   function Is_Digits (S : String) return Boolean is
     (S'Length > 0 and then (for all C of S => C in '0' .. '9'));

   function Waitable_List return String is
      R : Unbounded_String;
   begin
      for E in Ending loop
         if Can_Wait_For (E) then
            Append (R, (if Length (R) > 0 then " " else "") & Word (E));
         end if;
      end loop;
      return To_String (R);
   end Waitable_List;

   function Relation_List return String is
      R : Unbounded_String;
   begin
      for X in Relation loop
         Append (R, (if Length (R) > 0 then " " else "") & Word (X));
      end loop;
      return To_String (R);
   end Relation_List;

   Line_Starts : constant String :=
     Do_Word & ", " & Repeat_Word & ", " & If_Word & ", " & Try_Word & ", " & To_Word & ", " & Run_Word & ", "
     & Remember_Word & ", " & Say_Word & ", " & Done_Word & ", " & End_Word & ", " & Else_Word & ", " & Or_Word;

   procedure Parse
     (Text   : String;
      Result : out Program;
      Ok     : out Boolean;
      Why    : out Refusal)
   is
      type Open_Kind is (Repeat_Open, If_Then, If_Else, Try_Attempt, Try_Alternative, Define_Open);

      type Open_Block is record
         Kind    : Open_Kind;
         Header  : Statement_Id;
         Filling : Block_Index;
      end record;

      package Open_Vectors is new Ada.Containers.Vectors (Positive, Open_Block);

      Stack   : Open_Vectors.Vector;
      Line_No : Natural := 0;
      Failed  : Boolean := False;

      procedure Fail (Reason, Instead : String) is
      begin
         if not Failed then
            Failed := True;
            Why := Refused (Line_No, Reason, Instead);
         end if;
      end Fail;

      function Current return Block_Index is (if Stack.Is_Empty then Top else Stack.Last_Element.Filling);

      function New_Block return Block_Index is
      begin
         Result.Blocks.Append (Id_Vectors.Empty_Vector);
         return Result.Blocks.Last_Index;
      end New_Block;

      procedure Add (S : Statement) is
      begin
         Result.Statements.Append (S);
         Result.Blocks (Current).Append (Result.Statements.Last_Index);
      end Add;

      procedure Open (S : Statement; Kind : Open_Kind; Filling : Block_Index) is
      begin
         Add (S);
         Stack.Append (Open_Block'(Kind => Kind, Header => Result.Statements.Last_Index, Filling => Filling));
      end Open;

      procedure Switch (Kind : Open_Kind; Filling : Block_Index) is
         Was : constant Open_Block := Stack.Last_Element;
      begin
         Stack.Replace_Element (Stack.Last_Index, Open_Block'(Kind => Kind, Header => Was.Header, Filling => Filling));
      end Switch;

      procedure Read_Line (Raw : String) is
         T     : constant Token_Vectors.Vector := Split (Raw);
         Opens : constant Boolean := not T.Is_Empty and then To_String (T.Last_Element.Low) = Colon;
         Last  : constant Natural := (if Opens then T.Last_Index - 1 else T.Last_Index);
         --  The words of the line, the header colon left out; At is the next
         --  word to read.
         At_Word : Positive := T.First_Index;

         function Low (K : Positive) return String is (To_String (T (K).Low));
         function More return Boolean is (At_Word <= Last);
         function Next return String is (if More then Low (At_Word) else "");

         procedure Skip is
         begin
            At_Word := At_Word + 1;
         end Skip;

         function Take (W : String) return Boolean is
         begin
            if Next = W then
               Skip;
               return True;
            end if;
            return False;
         end Take;

         function Phrase (From, To : Natural) return Unbounded_String is
            R : Unbounded_String;
         begin
            for K in From .. To loop
               Append (R, (if Length (R) > 0 then " " else "") & To_String (T (K).Text));
            end loop;
            return R;
         end Phrase;

         function Rest return Unbounded_String is (Phrase (At_Word, Last));

         function Noun_Of (From, To : Positive) return Noun is
            Found : Boolean;
            R     : Role;
         begin
            if From = To then
               Find (Low (From), Found, R);
               if Found then
                  return (Kind => Role_Noun, Role => R, Name => Null_Unbounded_String);
               end if;
            end if;
            return (Kind => Name_Noun, Role => Me, Name => Phrase (From, To));
         end Noun_Of;

         function Header (Kind : Statement_Kind) return Statement is
            S : Statement (Kind);
         begin
            S.Line := Line_No;
            S.Text := To_Unbounded_String (Raw);
            return S;
         end Header;

         procedure Take_Count (Value : out Natural) is
         begin
            Value := Natural'Value (Next);
            Skip;
         exception
            when Constraint_Error =>
               Value := 0;
               Fail ("the number " & Next & " is larger than I can count", "write a smaller number");
         end Take_Count;

         procedure Take_Ending (E : out Ending) is
            Found : Boolean;
         begin
            Find (Next, Found, E);
            if Found then
               Skip;
            elsif More then
               Fail ("""" & Next & """ is not an ending; the endings are: " & Waitable_List, "");
            else
               Fail ("an ending is missing here; the endings are: " & Waitable_List, "");
            end if;
         end Take_Ending;

         procedure Expect_End_Of_Line (Form : String) is
         begin
            if More and then not Failed then
               Fail ("""" & To_String (T (At_Word).Text) & """ cannot stand here; the line is written " & Form, "");
            end if;
         end Expect_End_Of_Line;

         procedure Read_Constraint (From, To : Positive; Into : in out Constraint_Vectors.Vector) is
            C      : Programs.Constraint;
            Rel_At : Natural := 0;
            Found  : Boolean;
            R      : Relation;
            Tail   : Natural := To;
         begin
            for K in From .. To loop
               Find (Low (K), Found, R);
               if Found then
                  Rel_At := K;
                  exit;
               end if;
            end loop;
            if Rel_At = 0 then
               Fail ("""" & To_String (Phrase (From, To)) & """ says no relation; the relations are: " & Relation_List,
                     "");
               return;
            elsif Rel_At = From then
               Fail ("say who comes before """ & Low (Rel_At) & """: me, grasper, pusher or the name of a thing", "");
               return;
            end if;
            C.Subject := Noun_Of (From, Rel_At - 1);
            C.Relation := R;
            if Tail > Rel_At and then Low (Tail) = Must_Word then
               C.Must := True;
               Tail := Tail - 1;
            end if;
            if Tail > Rel_At then
               Find (Low (Tail), Found, C.Step);
               if Found then
                  Tail := Tail - 1;
               end if;
            end if;
            if Tail > Rel_At then
               Find (Low (Tail), Found, C.Strength);
               if Found then
                  Tail := Tail - 1;
               end if;
            end if;
            declare
               Object_From : Positive := Rel_At + 1;
            begin
               if Object_From <= Tail and then Low (Object_From) = On_Word then
                  Object_From := Object_From + 1;
               end if;
               if Object_From <= Tail then
                  C.Object := Noun_Of (Object_From, Tail);
               end if;
            end;
            if R = Press then
               if C.Strength = Unspecified then
                  Fail ("press says how hard: light, firm or hard", "");
               elsif C.Step /= Unspecified then
                  Fail ("press says how hard, so it cannot also say how far a step goes", "");
               end if;
            elsif C.Strength /= Unspecified then
               Fail ("light, firm and hard go only with press", "");
            end if;
            if R in Still | Open | Close and then C.Step /= Unspecified then
               Fail (Word (R) & " takes no step size", "");
            end if;
            if R in Still | Open and then C.Object.Kind /= Nothing then
               Fail (Word (R) & " takes nothing after it", "");
            elsif R not in Still | Open | Close and then C.Object.Kind = Nothing then
               Fail (Word (R) & " says how something stands to another thing: write that thing after it", "");
            end if;
            Into.Append (C);
         end Read_Constraint;

         --  The constraints of an interval are the words From .. To.
         procedure Read_Constraints (From, To : Positive; Into : in out Constraint_Vectors.Vector) is
            Direction   : constant String := Low (To);
            Quantity_At : constant Natural := To - 1;
         begin
            if (Direction = Up_Word or else Direction = Down_Word) and then Quantity_At > From then
               Into.Append (Programs.Constraint'(Kind     => Quantity_Constraint,
                                                 Subject  => Noun_Of (From, Quantity_At - 1),
                                                 Quantity => T (Quantity_At).Low,
                                                 Increase => Direction = Up_Word,
                                                 others   => <>));
               return;
            end if;
            declare
               Part : Positive := From;
            begin
               for J in From .. To loop
                  exit when Failed;
                  if Low (J) = And_Word then
                     if J = Part then
                        Fail ("there is nothing to do before this " & And_Word, "");
                     else
                        Read_Constraint (Part, J - 1, Into);
                        Part := J + 1;
                     end if;
                  end if;
               end loop;
               if not Failed then
                  if Part > To then
                     Fail ("there is nothing to do after the last " & And_Word, "");
                  else
                     Read_Constraint (Part, To, Into);
                  end if;
               end if;
            end;
         end Read_Constraints;

         procedure Read_Interval is
            S        : Statement := Header (Interval);
            Until_At : Natural := 0;
            E        : Ending;
         begin
            for J in At_Word .. Last loop
               if Low (J) = Until_Word then
                  Until_At := J;
                  exit;
               end if;
            end loop;
            if Until_At = 0 then
               Fail ("this stretch never says when it ends", Raw & " " & Until_Word & " " & Word (Settled));
               return;
            elsif Until_At = At_Word then
               Fail ("this stretch says nothing before until", "");
               return;
            end if;
            Read_Constraints (At_Word, Until_At - 1, S.Constraints);
            At_Word := Until_At + 1;
            Take_Ending (E);
            if Failed then
               return;
            elsif E = Arrived then
               Fail ("I cannot wait until arrived: only you can judge whether I have arrived, I can only notice events",
                     "end it with or <n> steps to have me come back after n steps, until touched to stop on contact,"
                     & " or until stuck to stop when I cannot go on");
               return;
            elsif E = Refused then
               Fail ("I cannot wait until refused: refused is what I answer when I cannot do something,"
                     & " not an event I can wait for",
                     "end it with until stuck to push until I cannot move, or with or <n> steps");
               return;
            end if;
            S.Until_Ending := E;
            while More and then not Failed loop
               if Take (Or_Word) then
                  if Is_Digits (Next) then
                     Take_Count (S.Max_Steps);
                     if not Take (Steps_Word) then
                        Fail ("a step limit is written or <n> steps", "");
                     end if;
                  else
                     Fail ("a step limit is written or <n> steps", "");
                  end if;
               elsif Take (With_Word) then
                  if Take (My_Word) and then (Next = Still_Word or else Next = Moving_Word) then
                     S.Eye := (if Next = Still_Word then Still_Eye else Moving_Eye);
                     Skip;
                     if not Take (Eye_Word) then
                        Fail ("an eye is written with my still eye or with my moving eye", "");
                     end if;
                  else
                     Fail ("an eye is written with my still eye or with my moving eye", "");
                  end if;
               elsif Take (Anyway_Word) then
                  S.Anyway := True;
               else
                  Fail ("""" & To_String (T (At_Word).Text) & """ cannot stand here; after the ending come only"
                        & " or <n> steps, with my still eye, with my moving eye and anyway", "");
               end if;
            end loop;
            if not Failed then
               Add (S);
            end if;
         end Read_Interval;

         First_Word : constant String := Next;
      begin
         if T.Is_Empty or else First_Word (First_Word'First) = '#' then
            return;
         end if;
         Skip;

         if First_Word = Say_Word then
            --  The sentence is kept character for character after the word say.
            declare
               At_Say : Positive := Raw'First;
               S      : Statement := Header (Say);
            begin
               while Is_Blank (Raw (At_Say)) loop
                  At_Say := At_Say + 1;
               end loop;
               S.Sentence := To_Unbounded_String
                 (Ada.Strings.Fixed.Trim (Raw (At_Say + Say_Word'Length .. Raw'Last), Ada.Strings.Both));
               Add (S);
            end;

         elsif First_Word = Colon then
            Fail ("a colon alone is not a line", "");

         elsif First_Word = End_Word then
            if More or else Opens then
               Fail ("end stands alone on its line", End_Word);
            elsif Stack.Is_Empty then
               Fail ("there is an end here but no block is open", "");
            else
               Stack.Delete_Last;
            end if;

         elsif First_Word = Else_Word then
            if More or else not Opens then
               Fail ("else is written alone with a colon", Else_Word & Colon);
            elsif Stack.Is_Empty or else Stack.Last_Element.Kind /= If_Then then
               Fail ("this else belongs to no if", "");
            else
               declare
                  B : constant Block_Index := New_Block;
                  H : Statement := Result.Statements (Stack.Last_Element.Header);
               begin
                  H.Else_Block := B;
                  Result.Statements.Replace_Element (Stack.Last_Element.Header, H);
                  Switch (If_Else, B);
               end;
            end if;

         elsif First_Word = Or_Word and then Opens and then not More then
            if Stack.Is_Empty or else Stack.Last_Element.Kind /= Try_Attempt then
               Fail ("this or: belongs to no try", "");
            else
               declare
                  B : constant Block_Index := New_Block;
                  H : Statement := Result.Statements (Stack.Last_Element.Header);
               begin
                  H.Alternative := B;
                  Result.Statements.Replace_Element (Stack.Last_Element.Header, H);
                  Switch (Try_Alternative, B);
               end;
            end if;

         elsif First_Word = Repeat_Word then
            if not Opens then
               Fail ("a repeat line ends with a colon", Raw & Colon);
            elsif Is_Digits (Next) then
               declare
                  S : Statement := Header (Repeat_Times);
               begin
                  Take_Count (S.Count);
                  if not Take (Times_Word) then
                     Fail ("repeat is followed by <n> times or by until <ending>", "");
                  end if;
                  Expect_End_Of_Line ("repeat <n> times:");
                  if not Failed then
                     S.Times_Body := New_Block;
                     Open (S, Repeat_Open, S.Times_Body);
                  end if;
               end;
            elsif Take (Until_Word) then
               declare
                  S : Statement := Header (Repeat_Until);
               begin
                  Take_Ending (S.Exit_Ending);
                  Expect_End_Of_Line ("repeat until <ending>:");
                  if not Failed then
                     S.Until_Body := New_Block;
                     Open (S, Repeat_Open, S.Until_Body);
                  end if;
               end;
            else
               Fail ("repeat is followed by <n> times or by until <ending>", "");
            end if;

         elsif First_Word = If_Word then
            if not Opens then
               Fail ("an if line ends with a colon", Raw & Colon);
            else
               declare
                  S : Statement := Header (If_Ending);
               begin
                  Take_Ending (S.Test);
                  Expect_End_Of_Line ("if <ending>:");
                  if not Failed then
                     S.Then_Block := New_Block;
                     Open (S, If_Then, S.Then_Block);
                  end if;
               end;
            end if;

         elsif First_Word = Try_Word then
            if More or else not Opens then
               Fail ("try is written alone with a colon", Try_Word & Colon);
            else
               declare
                  S : Statement := Header (Try_Or);
               begin
                  S.Attempt := New_Block;
                  Open (S, Try_Attempt, S.Attempt);
               end;
            end if;

         elsif First_Word = To_Word then
            if not More or else not Opens then
               Fail ("to is followed by the behaviour's name and a colon", "");
            else
               declare
                  S : Statement := Header (Define);
               begin
                  S.Behaviour := Rest;
                  S.Definition := New_Block;
                  Open (S, Define_Open, S.Definition);
               end;
            end if;

         elsif Opens then
            Fail ("only repeat, if, try, to, else and or lines end with a colon", "");

         elsif First_Word = Run_Word then
            if not More then
               Fail ("run is followed by the name of a behaviour you defined with to", "");
            else
               declare
                  S : Statement := Header (Run);
               begin
                  S.Called := Rest;
                  Add (S);
               end;
            end if;

         elsif First_Word = Remember_Word then
            declare
               Is_At : Natural := 0;
            begin
               if Take (Where_Word) then
                  for J in At_Word + 1 .. Last - 1 loop
                     if Low (J) = Is_Word and then Low (J + 1) = As_Word then
                        Is_At := J;
                        exit;
                     end if;
                  end loop;
               end if;
               if Is_At = 0 or else Is_At + 1 = Last then
                  Fail ("a place is remembered as: remember where <who> is as <name>", "");
               else
                  declare
                     S     : Statement := Header (Remember);
                     As_At : constant Positive := Is_At + 1;
                  begin
                     S.Who := Noun_Of (At_Word, Is_At - 1);
                     S.Place := Phrase (As_At + 1, Last);
                     Add (S);
                  end;
               end if;
            end;

         elsif First_Word = Done_Word then
            if More then
               Fail ("done stands alone on its line", Done_Word);
            else
               Add (Header (Done));
            end if;

         elsif First_Word = Do_Word then
            Read_Interval;

         else
            Fail ("I do not know the first word """ & To_String (T.First_Element.Text) & """; a line starts with "
                  & Line_Starts, "");
         end if;
      end Read_Line;

      I : Natural := Text'First;
   begin
      Result := (others => <>);
      Result.Blocks.Append (Id_Vectors.Empty_Vector);
      Why := (others => <>);
      while I <= Text'Last and then not Failed loop
         declare
            J : Natural := I;
         begin
            while J <= Text'Last and then Text (J) /= ASCII.LF loop
               J := J + 1;
            end loop;
            Line_No := Line_No + 1;
            declare
               Raw  : constant String := Text (I .. J - 1);
               Last : Natural := Raw'Last;
            begin
               while Last >= Raw'First and then Raw (Last) = ASCII.CR loop
                  Last := Last - 1;
               end loop;
               Read_Line (Raw (Raw'First .. Last));
            end;
            I := J + 1;
         end;
      end loop;
      if not Failed and then not Stack.Is_Empty then
         Fail ("the program ends while " & Natural'Image (Natural (Stack.Length)) & " block(s) are still open",
               End_Word);
      end if;
      Ok := not Failed;
   end Parse;

end Driver.Brain.Parser;
