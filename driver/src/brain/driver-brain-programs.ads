--  A program in the body language, as a tree of statements (LANGUAGE.md 11
--  and 17): what the parser produces and the interpreter runs.
--
--  Names stay exactly as the brain wrote them; binding them to things is a
--  separate step (Driver.Brain.Names), so one program can be checked against
--  the body without touching the world. Blocks are numbered and statements
--  refer to their inner blocks by number, so the tree needs no pointers.

with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Action;

package Driver.Brain.Programs is

   use Ada.Strings.Unbounded;

   type Noun_Kind is (Nothing, Role_Noun, Name_Noun);

   type Noun is record
      Kind : Noun_Kind := Nothing;
      Role : Driver.Action.Role := Driver.Action.Me;
      Name : Unbounded_String;     --  for Name_Noun: the words as written
   end record;

   type Constraint_Kind is (Relation_Constraint, Quantity_Constraint);

   type Constraint is record
      Kind     : Constraint_Kind := Relation_Constraint;
      Subject  : Noun;
      Relation : Driver.Action.Relation := Driver.Action.Touching;
      Object   : Noun;               --  Nothing for close here, open and still
      Step     : Driver.Action.Size := Driver.Action.Unspecified;
      Strength : Driver.Action.Effort := Driver.Action.Unspecified;
      Must     : Boolean := False;
      Quantity : Unbounded_String;   --  for Quantity_Constraint: the quantity's word
      Increase : Boolean := True;    --  for Quantity_Constraint: up rather than down
   end record;

   package Constraint_Vectors is new Ada.Containers.Vectors (Positive, Constraint);

   type Block_Id is new Natural;
   subtype Block_Index is Block_Id range 1 .. Block_Id'Last;
   No_Block : constant Block_Id := Block_Id'First;
   Top      : constant Block_Index := Block_Index'First;   --  the program's own lines

   type Statement_Kind is
     (Interval, Repeat_Times, Repeat_Until, If_Ending, Try_Or, Define, Run, Remember, Say, Done);

   type Statement (Kind : Statement_Kind := Done) is record
      Line : Positive := Positive'First;   --  where it was written
      Text : Unbounded_String;        --  that line as written
      case Kind is
         when Interval =>
            Constraints : Constraint_Vectors.Vector;
            Until_Ending : Driver.Action.Ending := Driver.Action.Settled;
            Max_Steps    : Natural := 0;   --  0: no step limit written
            Eye          : Driver.Action.Eye_Choice := Driver.Action.Any_Eye;
            Anyway       : Boolean := False;
         when Repeat_Times =>
            Count      : Natural := 0;
            Times_Body : Block_Id := No_Block;
         when Repeat_Until =>
            Exit_Ending : Driver.Action.Ending := Driver.Action.Settled;
            Until_Body  : Block_Id := No_Block;
         when If_Ending =>
            Test        : Driver.Action.Ending := Driver.Action.Settled;
            Then_Block  : Block_Id := No_Block;
            Else_Block  : Block_Id := No_Block;   --  No_Block when there is no else
         when Try_Or =>
            Attempt     : Block_Id := No_Block;
            Alternative : Block_Id := No_Block;
         when Define =>
            Behaviour   : Unbounded_String;
            Definition  : Block_Id := No_Block;
         when Run =>
            Called      : Unbounded_String;
         when Remember =>
            Who         : Noun;
            Place       : Unbounded_String;
         when Say =>
            Sentence    : Unbounded_String;
         when Done =>
            null;
      end case;
   end record;

   type Statement_Id is new Positive;

   package Statement_Vectors is new Ada.Containers.Indefinite_Vectors (Statement_Id, Statement);
   package Id_Vectors is new Ada.Containers.Vectors (Positive, Statement_Id);
   package Block_Vectors is new Ada.Containers.Vectors (Block_Index, Id_Vectors.Vector, Id_Vectors."=");

   type Program is record
      Statements : Statement_Vectors.Vector;
      Blocks     : Block_Vectors.Vector;   --  Blocks (Top) is the program's own lines
   end record;

   function Lines_Of (P : Program; B : Block_Index) return Id_Vectors.Vector is (P.Blocks (B));

   function Is_Empty (P : Program) return Boolean is (P.Blocks.Is_Empty or else P.Blocks (Top).Is_Empty);

   type Refusal is record
      Line    : Natural := 0;          --  0 when no single line is at fault
      Why     : Unbounded_String;      --  plain words for the brain
      Instead : Unbounded_String;      --  a line it can write instead; may be empty
   end record;
   --  Every check of a program before anything moves ends either in no
   --  refusal or in one of these, which the next round shows the brain.

   function Refused (Line : Natural; Why, Instead : String) return Refusal is
     ((Line => Line, Why => To_Unbounded_String (Why), Instead => To_Unbounded_String (Instead)));

end Driver.Brain.Programs;
