--  From a written stretch to what layer 4 executes: one Driver.Action.Want,
--  its names replaced by the things and places they were bound to.
--
--  A stretch that names a place the same program remembers further on can
--  only be built when it runs, after that remember: it is checked then,
--  just before it would move (LANGUAGE.md 8, proof may come late but never
--  not at all).

with Ada.Containers.Indefinite_Ordered_Maps;
with Driver.Action;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Brain.Programs;

package Driver.Brain.Wants is

   package Binding_Maps is new Ada.Containers.Indefinite_Ordered_Maps
     (String, Driver.Brain.Names.Binding, "<", Driver.Brain.Names."=");
   --  The binding of every name of one program, by the words as written.

   function Find (Bound : Binding_Maps.Map; Name : String) return Driver.Brain.Names.Binding;
   --  The binding of Name: under the same words, else under the same
   --  letters; Unbound, with no account, when there is none.

   type Build_Result is (Built, Later, Refused);

   procedure Build
     (S          : Driver.Brain.Programs.Statement;
      Bound      : Binding_Maps.Map;
      Quantities : Driver.Brain.Keyboard.Word_Vectors.Vector;
      W          : out Driver.Action.Want;
      Result     : out Build_Result;
      Why        : out Driver.Brain.Programs.Refusal)
     with Pre => Driver.Brain.Programs."=" (S.Kind, Driver.Brain.Programs.Interval);
   --  Quantities: this round's quantity words, in the order of
   --  Driver.Action.Quantities. Later: a name is a place not remembered yet.

   function Names_Of (P : Driver.Brain.Programs.Program) return Driver.Brain.Keyboard.Word_Vectors.Vector;
   --  Every name the program uses for a thing or a place, once each, in the
   --  order written; the names it remembers places under are left out.

   function Remembers (P : Driver.Brain.Programs.Program; Name : String) return Boolean;
   --  The program has a remember ... as Name (compared by letters).

end Driver.Brain.Wants;
