with Ada.Containers.Vectors;
with Ada.Strings.Fixed;
with Driver.Action;
with Driver.Tests;

package body Driver.Brain.Names.Tests is

   use Driver.Tests;
   use type Driver.World.Thing_Id;
   use type Driver.World.Place_Id;
   use type Driver.Observations.Camera_Id;

   --  The words a name may not be on the quantity keyboard of the recorded
   --  rounds: do, until, up, down, height, say, done, the endings and item.
   function Glue return Word_List is
      Q, M : Driver.Brain.Keyboard.Word_Vectors.Vector;
   begin
      Q.Append ("height");
      M.Append ("");
      return Driver.Brain.Keyboard.Name_Words
        (Driver.Brain.Keyboard.Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True],
                                       False, [others => False], Driver.Brain.Keyboard.Eye_Vectors.Empty_Vector));
   end Glue;

   procedure Letter_Rules is
   begin
      Check (Same_Name ("mintgreenscissors", "mint green scissors") and then Same_Name ("MintGreen Scissors", "mint"
             & " green scissors") and then Same_Name ("scis sors", "scissors"),
             "glued, split and capitalised words are one name");
      Check (not Same_Name ("scisors", "scissors") and then not Same_Name ("cap", "cup")
             and then not Same_Name ("", ""), "a letter more or less is another name, and no letters is no name");
   end Letter_Rules;

   procedure Glue_Rules is
      G : constant Word_List := Glue;
      procedure Same (A, B, Why : String) is
      begin
         Check (Same_Core (A, B, G) and then Same_Core (B, A, G), A & " is " & B & ": " & Why);
      end Same;
      procedure Other (A, B, Why : String) is
      begin
         Check (not Same_Core (A, B, G) and then not Same_Core (B, A, G), A & " is not " & B & ": " & Why);
      end Other;
   begin
      Same ("upmint green scissors", "mint green scissors", "up could only be typed glued to mint");
      Same ("scissors upuntil toucheduntil", "scissors", "up until touched until are all language words");
      Same ("mint green scissorsuntilstuck", "upmint green scissors", "glue on either end");
      Other ("cupboard", "cup", "board is not a language word");
      Other ("pencil", "pen", "cil is not one");
      Other ("open", "pen", "o is not one");
      Other ("red cupboard", "red cup", "board is not one");
      Other ("tissue box", "tissue", "box is not one: a tissue box is not a tissue");
      Other ("untildone", "saydone", "made only of language words, neither has a core");
      Other ("pick upthe pinktissueby", "pink tissue", "pick, the and by are not language words");
      Other ("the red ball", "the red cup", "sharing words is not being the same");
   end Glue_Rules;

   --  A fake body: each eye answers a name from a table; the patch it boxes is
   --  a given thing, part of the body, or nothing.

   type Reply is record
      Eye    : Eye_Id;
      Name   : Unbounded_String;
      Answer : Pointing;
      Found  : Patch;
      Thing  : Thing_Id;
   end record;

   package Reply_Vectors is new Ada.Containers.Vectors (Positive, Reply);

   type Sight is record
      Thing : Thing_Id;
      Eye   : Eye_Id;
   end record;

   package Sight_Vectors is new Ada.Containers.Vectors (Positive, Sight);

   type Fake is new Senses with record
      All_Eyes : Eye_Vectors.Vector;
      Replies  : Reply_Vectors.Vector;
      Seen     : Sight_Vectors.Vector;
      Asked    : Unbounded_String;    --  "e:name;" for every question, in order
      Last     : Natural := 0;        --  the reply to the latest question
   end record;

   overriding function Eyes (S : Fake) return Eye_Vectors.Vector;
   overriding function Sees (S : Fake; T : Thing_Id; E : Eye_Id) return Boolean;
   overriding procedure Ask_Where
     (S      : in out Fake;
      E      : Eye_Id;
      Name   : String;
      Answer : out Pointing;
      Where  : out Box;
      Why    : out Unbounded_String);
   overriding procedure Identify
     (S     : in out Fake;
      E     : Eye_Id;
      Where : Box;
      Found : out Patch;
      T     : out Thing_Id;
      Why   : out Unbounded_String);

   overriding function Eyes (S : Fake) return Eye_Vectors.Vector is (S.All_Eyes);

   overriding function Sees (S : Fake; T : Thing_Id; E : Eye_Id) return Boolean is
     (for some X of S.Seen => X.Thing = T and then X.Eye = E);

   overriding procedure Ask_Where
     (S      : in out Fake;
      E      : Eye_Id;
      Name   : String;
      Answer : out Pointing;
      Where  : out Box;
      Why    : out Unbounded_String)
   is
   begin
      Append (S.Asked, Ada.Strings.Fixed.Trim (E'Image, Ada.Strings.Both) & ":" & Name & ";");
      Answer := Not_Here;
      Where := (others => <>);
      Why := Null_Unbounded_String;
      S.Last := 0;
      for I in S.Replies.First_Index .. S.Replies.Last_Index loop
         if S.Replies (I).Eye = E and then To_String (S.Replies (I).Name) = Name then
            Answer := S.Replies (I).Answer;
            S.Last := I;
         end if;
      end loop;
   end Ask_Where;

   overriding procedure Identify
     (S     : in out Fake;
      E     : Eye_Id;
      Where : Box;
      Found : out Patch;
      T     : out Thing_Id;
      Why   : out Unbounded_String)
   is
      pragma Unreferenced (E, Where);
   begin
      Found := S.Replies (S.Last).Found;
      T := S.Replies (S.Last).Thing;
      Why := Null_Unbounded_String;
   end Identify;

   function Three_Eyes return Fake is
      F : Fake;
   begin
      for E in Eye_Id range 1 .. 3 loop
         F.All_Eyes.Append (E);
      end loop;
      return F;
   end Three_Eyes;

   function R (E : Eye_Id; Name : String; A : Pointing; P : Patch := A_Thing; T : Thing_Id := 1) return Reply is
     ((Eye => E, Name => To_Unbounded_String (Name), Answer => A, Found => P, Thing => T));

   procedure Order is
      N : Table;
      F : Fake := Three_Eyes;
      B : Binding;
   begin
      N.Name_Place ("home", 4);
      Bind (N, F, 1, "Home", Glue, B);
      Check (B.Kind = To_Place and then B.Place = 4 and then Length (F.Asked) = 0,
             "step 0: a remembered place, no eye asked");
      F.Replies.Append (R (2, "mint green scissors", Boxed, A_Thing, 7));
      Bind (N, F, 2, "mint green scissors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 7 and then To_String (F.Asked) = "2:mint green scissors;",
             "step 2: the eye boxes it, the brain's words verbatim");
      F.Seen.Append (Sight'(Thing => 7, Eye => 2));
      F.Asked := Null_Unbounded_String;
      Bind (N, F, 2, "mintgreenscissors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 7 and then Length (F.Asked) = 0,
             "step 1: this eye sees a thing with the same letters, nobody is asked again");
      F.Replies.Append (R (2, "scisors", Boxed, A_Thing, 7));
      Bind (N, F, 2, "scisors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 7 and then N.Name_Of (7) = "scisors",
             "a letter wrong is for the eye: the same pixels are the same thing, which now goes by the new words");
   end Order;

   procedure Other_Eyes is
      N : Table;
      F : Fake := Three_Eyes;
      B : Binding;
   begin
      F.Replies.Append (R (3, "scissors", Boxed, A_Thing, 5));
      Bind (N, F, 2, "scissors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 5 and then To_String (F.Asked) = "2:scissors;1:scissors;3:scissors;",
             "an eye that cannot point it out passes the question on, in camera order");
      F := Three_Eyes;
      F.Replies.Append (R (1, "arm reach ight", Boxed, Part_Of_Me));
      F.Replies.Append (R (2, "arm reach ight", Boxed, A_Thing, 6));
      Bind (N, F, 1, "arm reach ight", Glue, B);
      Check (B.Kind = Unbound and then Ada.Strings.Fixed.Index (To_String (B.Account), "part of me") > 0
             and then To_String (F.Asked) = "1:arm reach ight;",
             "a patch that is part of the body is not a thing, and no other eye's guess replaces it");
      declare
         Held : Table;
      begin
         F := Three_Eyes;
         F.Replies.Append (R (1, "cup", Boxed, A_Thing, 8));
         Bind (Held, F, 1, "cup", Glue, B);
         F.Replies.Append (R (2, "the cup", Boxed, Part_Of_Me));
         Bind (Held, F, 2, "the cup", Glue, B);
         Check (B.Kind = Unbound, "the cup is not the letters of the cup: the box on the body binds nothing");
         F.Replies.Append (R (2, "cup", Boxed, Part_Of_Me));
         Bind (Held, F, 2, "cup", Glue, B);
         Check (B.Kind = To_Thing and then B.Thing = 8,
                "a thing named before and boxed where my hand is: its letters still bind it (it is in my hand)");
      end;
      F := Three_Eyes;
      F.Replies.Append (R (1, "ball", No_Answer));
      Bind (N, F, 1, "ball", Glue, B);
      Check (B.Kind = Unbound and then To_String (F.Asked) = "1:ball;",
             "when the brain service fails no other eye is asked");
      F := Three_Eyes;
      Bind (N, F, 1, "untildone", Glue, B);
      Check (B.Kind = Unbound and then Length (F.Asked) = 0, "a name made only of language words is not asked about");
   end Other_Eyes;

   procedure Letters_Last is
      N : Table;
      F : Fake := Three_Eyes;
      B : Binding;
   begin
      F.Replies.Append (R (1, "mint green scissors", Boxed, A_Thing, 7));
      Bind (N, F, 1, "mint green scissors", Glue, B);
      F.Asked := Null_Unbounded_String;
      Bind (N, F, 2, "upmint green scissors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 7 and then Length (F.Asked) > 0,
             "no eye points it out: by its letters, after glue is shed, it is the scissors");
      F.Replies.Append (R (1, "cup", Boxed, A_Thing, 8));
      Bind (N, F, 1, "cup", Glue, B);
      Bind (N, F, 2, "cupboard", Glue, B);
      Check (B.Kind = Unbound and then Ada.Strings.Fixed.Index (To_String (B.Account), "none of the things") > 0,
             "a cupboard is not the cup");
      F.Replies.Append (R (1, "ball", Boxed, A_Thing, 9));
      F.Replies.Append (R (1, "balldo", Boxed, A_Thing, 10));
      Bind (N, F, 1, "ball", Glue, B);
      Bind (N, F, 1, "balldo", Glue, B);
      Bind (N, F, 3, "ballup", Glue, B);
      Check (B.Kind = Unbound and then Ada.Strings.Fixed.Index (To_String (B.Account), "do not guess") > 0,
             "two things answer to the letters: no guess");
   end Letters_Last;

   procedure Second_Pass is
      N : Table;
      F : Fake := Three_Eyes;
      B, Late : Binding;
   begin
      Bind (N, F, 2, "lift the mintgreenscissors", Glue, B);
      F.Replies.Append (R (1, "the mintgreenscissors", Boxed, A_Thing, 7));
      Bind (N, F, 1, "the mintgreenscissors", Glue, Late);
      Check (B.Kind = Unbound, "the first line's name binds to nothing on the first pass");
      Bind_Again (N, Glue, "lift the mintgreenscissors", B);
      Check (B.Kind = Unbound, "lift is not a language word, so the letters do not make it the scissors");
      B := (others => <>);
      Bind (N, F, 2, "upuntil the mintgreenscissors", Glue, B);
      Check (B.Kind = To_Thing and then B.Thing = 7, "glue alone in front: the letters find it");
      N := (others => <>);
      F := Three_Eyes;
      Bind (N, F, 2, "upthe mintgreenscissors", Glue, B);
      F.Replies.Append (R (1, "the mintgreenscissors", Boxed, A_Thing, 7));
      Bind (N, F, 1, "the mintgreenscissors", Glue, Late);
      Bind_Again (N, Glue, "upthe mintgreenscissors", B);
      Check (B.Kind = To_Thing and then B.Thing = 7,
             "a name unbound on the first pass binds once a later line named the thing");
   end Second_Pass;

   procedure Register is
   begin
      Register ("brain.names.letters", "glued, split or capitalised words are taken for another name",
                Letter_Rules'Access);
      Register ("brain.names.glue", "letters other than glued language words make two names one", Glue_Rules'Access);
      Register ("brain.names.order", "the binding order of LANGUAGE.md 17.7 is broken", Order'Access);
      Register ("brain.names.eyes", "an eye that cannot point a name out ends the search, or the body binds itself",
                Other_Eyes'Access);
      Register ("brain.names.letters_last", "the letters bind a different thing, or guess between two",
                Letters_Last'Access);
      Register ("brain.names.second_pass", "whether a name binds depends on the line it is written on",
                Second_Pass'Access);
   end Register;

end Driver.Brain.Names.Tests;
