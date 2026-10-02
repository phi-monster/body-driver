with Ada.Command_Line;
with Ada.Directories;
with Ada.Strings;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Action;
with Driver.Brain.Names;
with Driver.Brain.Programs;
with Driver.Brain.Termination;
with Driver.Tests;

package body Driver.Brain.Parser.Tests is

   use Ada.Strings.Unbounded;
   use Driver.Action;
   use Driver.Brain.Programs;
   use Driver.Tests;

   LF : constant String := [ASCII.LF];

   procedure Read (Text : String; P : out Program; What : String) is
      Ok  : Boolean;
      Why : Refusal;
   begin
      Parse (Text, P, Ok, Why);
      Check (Ok, What & ": refused (" & To_String (Why.Why) & ")");
   end Read;

   function Refusal_Of (Text : String) return Refusal is
      P   : Program;
      Ok  : Boolean;
      Why : Refusal;
   begin
      Parse (Text, P, Ok, Why);
      Check (not Ok, "accepted: " & Text);
      return Why;
   end Refusal_Of;

   function First (P : Program) return Statement is (P.Statements (P.Blocks (Top).First_Element));

   function First_Constraint (P : Program) return Programs.Constraint is
      S : constant Statement := First (P);
   begin
      return S.Constraints.First_Element;
   end First_Constraint;

   procedure Quantity_Sentence is
      P : Program;
   begin
      Read ("do scissors height up until settled", P, "the quantity sentence");
      declare
         S : constant Statement := First (P);
      begin
         Check (S.Kind = Interval and then Natural (S.Constraints.Length) = 1, "one stretch, one constraint");
         Check (S.Constraints (1).Kind = Quantity_Constraint and then S.Constraints (1).Subject.Kind = Name_Noun
                and then To_String (S.Constraints (1).Subject.Name) = "scissors"
                and then To_String (S.Constraints (1).Quantity) = "height" and then S.Constraints (1).Increase
                and then S.Until_Ending = Settled, "thing, quantity, direction and ending");
      end;
      Read ("do salt and pepper above the tray height down until touched", P, "a name with and in it");
      declare
         S : constant Statement := First (P);
      begin
         Check (Natural (S.Constraints.Length) = 1
                and then To_String (S.Constraints (1).Subject.Name) = "salt and pepper above the tray"
                and then not S.Constraints (1).Increase, "and and relation words stay inside the name");
      end;
   end Quantity_Sentence;

   procedure Constraints is
      P : Program;
   begin
      Read ("do grasper touching the red ball small must and pusher clear cup until touched or 20 steps"
            & " with my still eye anyway", P, "two constraints with every option");
      declare
         S : constant Statement := First (P);
         A : constant Programs.Constraint := S.Constraints (1);
         B : constant Programs.Constraint := S.Constraints (2);
      begin
         Check (A.Subject.Kind = Role_Noun and then A.Subject.Role = Grasper and then A.Relation = Touching
                and then To_String (A.Object.Name) = "the red ball" and then A.Step = Small and then A.Must,
                "first constraint");
         Check (B.Subject.Role = Pusher and then B.Relation = Clear and then To_String (B.Object.Name) = "cup"
                and then not B.Must, "second constraint");
         Check (S.Max_Steps = 20 and then S.Eye = Still_Eye and then S.Anyway and then S.Until_Ending = Touched,
                "step limit, eye and anyway");
      end;
      Read ("do grasper close on ball until stuck", P, "close on");
      declare
         C : constant Programs.Constraint := First_Constraint (P);
      begin
         Check (C.Relation = Close and then To_String (C.Object.Name) = "ball", "on after a relation is skipped");
      end;
      Read ("do grasper close until stuck", P, "close here");
      Check (First_Constraint (P).Object.Kind = Nothing, "close without a thing closes where it is");
      Read ("do grasper press table firm until stuck", P, "press");
      Check (First_Constraint (P).Strength = Firm, "press keeps its effort");
   end Constraints;

   procedure Refusals is
      W : Refusal;
   begin
      W := Refusal_Of ("do grasper press table until stuck");
      Check (Index (W.Why, "how hard") > 0, "press without an effort");
      W := Refusal_Of ("do grasper press table firm small until stuck");
      Check (Index (W.Why, "how far") > 0, "press with a step");
      W := Refusal_Of ("do grasper touching ball firm until touched");
      Check (Index (W.Why, "only with press") > 0, "an effort without press");
      W := Refusal_Of ("do grasper open ball until settled");
      Check (Index (W.Why, "nothing after it") > 0, "open takes no thing");
      W := Refusal_Of ("do grasper touching until touched");
      Check (Index (W.Why, "write that thing") > 0, "touching needs a thing");
      W := Refusal_Of ("do grasper touching ball");
      Check (W.Line = 1 and then To_String (W.Instead) = "do grasper touching ball until settled",
             "a missing ending comes back with a line to copy");
      W := Refusal_Of ("do grasper touching ball until arrived");
      Check (Index (W.Why, "only you can judge") > 0, "until arrived is refused");
      W := Refusal_Of ("do grasper touching ball until refused");
      Check (Index (W.Why, "not an event") > 0, "until refused is refused");
      W := Refusal_Of ("do grasper touching ball until later");
      Check (Index (W.Why, "not an ending") > 0, "an unknown ending");
      W := Refusal_Of ("say hi" & LF & "end");
      Check (W.Line = 2 and then Index (W.Why, "no block is open") > 0, "an end with nothing open");
      W := Refusal_Of ("repeat 2 times:" & LF & "say hi");
      Check (Index (W.Why, "still open") > 0, "a block never closed");
      W := Refusal_Of ("else:");
      Check (Index (W.Why, "no if") > 0, "else without if");
      W := Refusal_Of ("or:");
      Check (Index (W.Why, "no try") > 0, "or: without try");
      W := Refusal_Of ("grab the ball");
      Check (Index (W.Why, "first word") > 0, "an unknown first word");
      W := Refusal_Of ("repeat 99999999999999999999 times:" & LF & "say hi" & LF & "end");
      Check (Index (W.Why, "larger than I can count") > 0, "a count too large for the driver is refused, not cut");
      W := Refusal_Of ("do grasper touching ball until touched or many steps");
      Check (Index (W.Why, "or <n> steps") > 0, "a step limit that is not a number");
   end Refusals;

   procedure Blocks is
      P    : Program;
      Text : constant String :=
        "to pick up ball:" & LF
        & "  remember where grasper is as start" & LF
        & "  repeat 3 times:" & LF
        & "    do grasper touching ball until touched or 20 steps" & LF
        & "    do grasper close ball until stuck" & LF
        & "    do grasper farther ball until slipped or 10 steps" & LF
        & "    if slipped:" & LF
        & "      do grasper open until settled" & LF
        & "      do grasper touching start until touched or 20 steps" & LF
        & "    else:" & LF
        & "      done" & LF
        & "    end" & LF
        & "  end" & LF
        & "  say I tried three times and it is not in my hand" & LF
        & "end" & LF
        & "run pick up ball";
   begin
      Read (Text, P, "the remedy program of LANGUAGE.md 17.8");
      declare
         Top_Lines : constant Id_Vectors.Vector := P.Blocks (Top);
         D         : constant Statement := P.Statements (Top_Lines (1));
         R         : constant Statement := P.Statements (Top_Lines (2));
      begin
         Check (Natural (Top_Lines.Length) = 2 and then D.Kind = Define and then R.Kind = Run
                and then To_String (D.Behaviour) = "pick up ball" and then To_String (R.Called) = "pick up ball",
                "a definition and a call at the top");
         declare
            Def_Lines : constant Id_Vectors.Vector := P.Blocks (D.Definition);
            Rep       : constant Statement := P.Statements (Def_Lines (2));
            Memo      : constant Statement := P.Statements (Def_Lines (1));
         begin
            Check (Natural (Def_Lines.Length) = 3 and then Memo.Kind = Remember
                   and then Rep.Kind = Repeat_Times and then Rep.Count = 3
                   and then P.Statements (Def_Lines (3)).Kind = Say, "remember, repeat, say");
            Check (To_String (Memo.Place) = "start" and then Memo.Who.Role = Grasper, "the remembered place");
            declare
               Rep_Lines : constant Id_Vectors.Vector := P.Blocks (Rep.Times_Body);
               Iff       : constant Statement := P.Statements (Rep_Lines (4));
            begin
               Check (Natural (Rep_Lines.Length) = 4 and then Iff.Kind = If_Ending and then Iff.Test = Slipped,
                      "three stretches and an if");
               Check (Natural (P.Blocks (Iff.Then_Block).Length) = 2 and then Iff.Else_Block /= No_Block
                      and then P.Statements (P.Blocks (Iff.Else_Block).First_Element).Kind = Done,
                      "then and else branches");
            end;
         end;
      end;
      Read ("try:" & LF & "do grasper touching ball until touched" & LF & "or:" & LF & "say no" & LF & "end", P,
            "try with or");
      Check (First (P).Kind = Try_Or and then First (P).Alternative /= No_Block, "try keeps its or branch");
      Read ("repeat until touched:" & LF & "do grasper nearer ball until timeout or 5 steps" & LF & "end", P,
            "repeat until");
      Check (First (P).Kind = Repeat_Until and then First (P).Exit_Ending = Touched,
             "repeat until, which the parser reads though no keyboard offers it");
   end Blocks;

   procedure Lines is
      P : Program;
   begin
      Read ("# a comment" & LF & LF & "say look = 2" & ASCII.CR & LF & "say I am Lifting: it", P,
            "comments, blank lines and CR LF");
      Check (Natural (P.Blocks (Top).Length) = 2
             and then To_String (P.Statements (P.Blocks (Top) (1)).Sentence) = "look = 2"
             and then To_String (P.Statements (P.Blocks (Top) (2)).Sentence) = "I am Lifting: it",
             "a sentence is kept as written, colon and capitals included");
   end Lines;

   --  docs/language.md, found from where the self test runs (driver/bin).
   function Reference return String is
      Exe  : constant String := Ada.Directories.Full_Name (Ada.Command_Line.Command_Name);
      Root : constant String := Ada.Directories.Containing_Directory
        (Ada.Directories.Containing_Directory (Ada.Directories.Containing_Directory (Exe)));
      Path : constant String := Ada.Directories.Compose (Ada.Directories.Compose (Root, "docs"), "language.md");
      F    : Ada.Text_IO.File_Type;
      R    : Unbounded_String;
   begin
      if not Ada.Directories.Exists (Path) then
         return "";
      end if;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Append (R, Ada.Text_IO.Get_Line (F) & LF);
      end loop;
      Ada.Text_IO.Close (F);
      return To_String (R);
   end Reference;

   --  Every block marked program in the reference is read and can end;
   --  every block marked refused is refused before anything moves.
   procedure Reference_Examples is
      Text      : constant String := Reference;
      Fence     : constant String := "```";
      Kind      : Unbounded_String;   --  the open block's mark, "" outside a block
      Block     : Unbounded_String;
      First     : Positive := Text'First;
      Programs_Read, Refusals_Read : Natural := 0;

      procedure Judge is
         P   : Program;
         Ok  : Boolean;
         Why : Refusal;
      begin
         Parse (To_String (Block), P, Ok, Why);
         if Ok then
            Why := Driver.Brain.Termination.Check (P, Driver.Brain.Termination.No_Stretch_Yet,
                                                   Driver.Brain.Names.Same_Name'Access);
            Ok := Why.Line = 0;
         end if;
         if To_String (Kind) = "program" then
            Programs_Read := Programs_Read + 1;
            Check (Ok, "docs/language.md: a program the reference shows is refused (" & To_String (Why.Why) & "):" & LF
                   & To_String (Block));
         elsif To_String (Kind) = "refused" then
            Refusals_Read := Refusals_Read + 1;
            Check (not Ok, "docs/language.md: a program the reference shows refused is accepted:" & LF
                   & To_String (Block));
         end if;
      end Judge;
   begin
      Check (Text'Length > 0, "docs/language.md is not beside the driver");
      for I in Text'Range loop
         if Text (I) = ASCII.LF then
            declare
               Line : constant String := Text (First .. I - 1);
            begin
               if Line'Length >= Fence'Length and then Line (Line'First .. Line'First + Fence'Length - 1) = Fence then
                  if Length (Kind) = 0 then
                     Kind := To_Unbounded_String (Line (Line'First + Fence'Length .. Line'Last) & " ");
                     Kind := Ada.Strings.Unbounded.Trim (Kind, Ada.Strings.Both);
                     if Length (Kind) = 0 then
                        Kind := To_Unbounded_String (Fence);   --  an unmarked block: not judged
                     end if;
                     Block := Null_Unbounded_String;
                  else
                     Judge;
                     Kind := Null_Unbounded_String;
                  end if;
               elsif Length (Kind) > 0 then
                  Append (Block, Line & LF);
               end if;
            end;
            First := I + 1;
         end if;
      end loop;
      Check (Programs_Read > 0 and then Refusals_Read > 0,
             "the reference shows" & Programs_Read'Image & " programs and" & Refusals_Read'Image & " refusals");
   end Reference_Examples;

   procedure Register is
   begin
      Register ("brain.parser.reference", "a program docs/language.md shows is not what the driver reads",
                Reference_Examples'Access);
      Register ("brain.parser.quantity", "a quantity sentence whose name holds and or a relation word is cut apart",
                Quantity_Sentence'Access);
      Register ("brain.parser.constraints", "a constraint loses its step, rank, eye, step limit or anyway",
                Constraints'Access);
      Register ("brain.parser.refusals", "a line the language does not allow is read as something else",
                Refusals'Access);
      Register ("brain.parser.blocks", "blocks nest wrongly or lose their else, or or body", Blocks'Access);
      Register ("brain.parser.lines", "a sentence is altered or a comment is read as a line", Lines'Access);
   end Register;

end Driver.Brain.Parser.Tests;
