--  The keyboard of one round (LANGUAGE.md 17.1): the keys this body can use
--  now, built from what it measured.
--
--  One description is rendered three ways: the sheet printed for the brain,
--  the grammar sent for constrained decoding, and the words a name may not
--  be. They come from the same record, so what the brain reads, what the
--  decoder lets it type and what the name rules expect cannot disagree.
--
--  With a grasper bound and a quantity to change, the keyboard is the
--  quantity sentence (do <thing> <quantity> up|down until <ending>), say and
--  done. Without a grasper but with some role, it is the full keyboard of
--  constraints and control. With no role at all, only say and done.

with Ada.Containers.Indefinite_Vectors;
with Driver.Action;

package Driver.Brain.Keyboard is

   type Role_Set is array (Driver.Action.Role) of Boolean;
   type Relation_Set is array (Driver.Action.Relation) of Boolean;
   subtype Ending_Set is Driver.Action.Ending_Set;

   package Word_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

   type Layout is (Speech_Only, Quantity_Keys, Full_Keys);

   function Image (L : Layout) return String is
     (case L is when Speech_Only => "speech-only", when Quantity_Keys => "quantity", when Full_Keys => "full");

   type Keyboard is record
      Keys       : Layout := Speech_Only;
      Quantities : Word_Vectors.Vector;    --  the quantities' words
      Meanings   : Word_Vectors.Vector;    --  one per quantity, as the side that measures it says; may be empty
      Roles      : Role_Set := [others => False];
      Relations  : Relation_Set := [others => False];
      --  Full_Keys: the relations of constraints. Quantity_Keys: the
      --  relations of the sentence about two things, when it is offered.
      Endings    : Ending_Set := [others => False];   --  what a stretch can wait for
   end record;

   function Choose
     (Quantities       : Word_Vectors.Vector;
      Meanings         : Word_Vectors.Vector;
      Roles            : Role_Set;
      Relations        : Relation_Set;
      Surface_Measured : Boolean;
      Two_Things       : Relation_Set) return Keyboard
     with Pre => Natural (Meanings.Length) = Natural (Quantities.Length);
   --  Relations: what the full keyboard may offer. Two_Things: the relations
   --  of the sentence about two things on the quantity keyboard (none: the
   --  sentence is not offered). Free is offered only when the surface a
   --  thing rests on is measured.

   function Sheet (K : Keyboard) return String;
   --  The grammar as the brain reads it, every key with its meaning.

   function Grammar (K : Keyboard) return String;
   --  The same grammar in GBNF, for constrained decoding.

   function Name_Words (K : Keyboard) return Word_Vectors.Vector;
   --  The words a name may not be this round: every word of the grammar,
   --  and item. The decoder cannot type them inside a name, so when the brain
   --  wanted one there it can only glue it to a neighbouring word.

   function Waits_For (K : Keyboard; E : Driver.Action.Ending) return Boolean is (K.Endings (E));

end Driver.Brain.Keyboard;
