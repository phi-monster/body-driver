--  The words of the body language (LANGUAGE.md 17): roles, relations, step
--  sizes, efforts and endings, one to one with the types of Driver.Action,
--  and the structural words that frame a line.
--
--  The parser, the keyboard and the name rules all spell a language word
--  through this one table, so they cannot disagree about what is a word of
--  the language and what is part of a name.

with Driver.Action;

package Driver.Brain.Words is

   use Driver.Action;

   function Word (R : Role) return String;
   function Word (R : Relation) return String;
   function Word (S : Size) return String
     with Pre => S /= Unspecified;
   function Word (E : Effort) return String
     with Pre => E /= Unspecified;
   function Word (E : Ending) return String;

   procedure Find (W : String; Found : out Boolean; R : out Role);
   procedure Find (W : String; Found : out Boolean; R : out Relation);
   procedure Find (W : String; Found : out Boolean; S : out Size);
   procedure Find (W : String; Found : out Boolean; E : out Effort);
   procedure Find (W : String; Found : out Boolean; E : out Ending);
   --  W must already be in lower case; Found is False when W is not that
   --  kind of word.

   --  Structural words, spelled once.
   Do_Word       : constant String := "do";
   And_Word      : constant String := "and";
   Until_Word    : constant String := "until";
   Or_Word       : constant String := "or";
   Steps_Word    : constant String := "steps";
   With_Word     : constant String := "with";
   My_Word       : constant String := "my";
   Still_Word    : constant String := "still";
   Moving_Word   : constant String := "moving";
   Eye_Word      : constant String := "eye";
   Anyway_Word   : constant String := "anyway";
   Must_Word     : constant String := "must";
   On_Word       : constant String := "on";
   Up_Word       : constant String := "up";
   Down_Word     : constant String := "down";
   Repeat_Word   : constant String := "repeat";
   Times_Word    : constant String := "times";
   If_Word       : constant String := "if";
   Else_Word     : constant String := "else";
   Try_Word      : constant String := "try";
   End_Word      : constant String := "end";
   To_Word       : constant String := "to";
   Run_Word      : constant String := "run";
   Remember_Word : constant String := "remember";
   Where_Word    : constant String := "where";
   Is_Word       : constant String := "is";
   As_Word       : constant String := "as";
   Say_Word      : constant String := "say";
   Done_Word     : constant String := "done";

   Look_Sign : constant String := "look =";
   --  What a say sentence starts with when it switches eyes (LANGUAGE.md
   --  17.5); the eye's number follows.

   Item_Word : constant String := "item";
   --  The body's own bookkeeping word for entries of a list; never a name
   --  (LANGUAGE.md 17.7).

   function Is_Failure (E : Ending) return Boolean is
     (E in Stuck | Slipped | Lost | Stalled | Timeout | Refused);
   --  A stretch inside try that ends with one of these did not succeed and
   --  control moves to the or branch (LANGUAGE.md 17.3).

   function Can_Wait_For (E : Ending) return Boolean is (E not in Arrived | Refused);
   --  Only the brain can judge arrival, and refused is the body's answer, not
   --  an event it can wait for (LANGUAGE.md 17.3).

end Driver.Brain.Words;
