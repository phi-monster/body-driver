with Ada.Characters.Handling;

package body Driver.Brain.Words is

   --  The enumerations of Driver.Action are named exactly as the language's
   --  words, so a word is the lower-case image of its literal.

   function Lower (S : String) return String renames Ada.Characters.Handling.To_Lower;

   function Word (R : Role) return String is (Lower (Role'Image (R)));
   function Word (R : Relation) return String is (Lower (Relation'Image (R)));
   function Word (S : Size) return String is (Lower (Size'Image (S)));
   function Word (E : Effort) return String is (Lower (Effort'Image (E)));
   function Word (E : Ending) return String is (Lower (Ending'Image (E)));

   procedure Find (W : String; Found : out Boolean; R : out Role) is
   begin
      for X in Role loop
         if Word (X) = W then
            Found := True;
            R := X;
            return;
         end if;
      end loop;
      Found := False;
      R := Role'First;
   end Find;

   procedure Find (W : String; Found : out Boolean; R : out Relation) is
   begin
      for X in Relation loop
         if Word (X) = W then
            Found := True;
            R := X;
            return;
         end if;
      end loop;
      Found := False;
      R := Relation'First;
   end Find;

   procedure Find (W : String; Found : out Boolean; S : out Size) is
   begin
      for X in Small .. Large loop
         if Word (X) = W then
            Found := True;
            S := X;
            return;
         end if;
      end loop;
      Found := False;
      S := Unspecified;
   end Find;

   procedure Find (W : String; Found : out Boolean; E : out Effort) is
   begin
      for X in Light .. Hard loop
         if Word (X) = W then
            Found := True;
            E := X;
            return;
         end if;
      end loop;
      Found := False;
      E := Unspecified;
   end Find;

   procedure Find (W : String; Found : out Boolean; E : out Ending) is
   begin
      for X in Ending loop
         if Word (X) = W then
            Found := True;
            E := X;
            return;
         end if;
      end loop;
      Found := False;
      E := Ending'First;
   end Find;

end Driver.Brain.Words;
