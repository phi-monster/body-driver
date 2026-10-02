--  Binding the brain's names to things and places (LANGUAGE.md 17.7).
--
--  A name is the brain's words; the body finds what they point at, in this
--  order, and never guesses:
--
--    0  a place the brain had me remember under the same letters;
--    1  a thing this eye sees now that already has the same letters;
--    2  the eye is asked where the name is, with the brain's words as they
--       are; the box it gives is segmented, and the patch is the thing that
--       already occupies those pixels, or a new thing. An eye that cannot
--       point it out passes the question to the other eyes, in camera order.
--       A patch that is part of the body itself ends the asking: the eye did
--       point it out, and parts of the body go by their role, never by a
--       name, so only the letters of step 3 may still bind it (a thing named
--       before, now in the hand);
--    3  when no eye points it out, the letters alone: the same letters, or
--       letters whose only additions are this round's language words glued
--       to its ends (the only way the decoder adds letters to a name), and
--       only one earlier thing answers to them;
--    4  after every name of a program had its turn, the names still unbound
--       try step 3 again, so a name binds the same wherever it is written.
--
--  Letters are a to z, case folded; blanks and everything else do not
--  count, so "mintgreenscissors" and "Mint Green scissors" are one name.
--  An eye that identifies an earlier thing under new words renames it: from
--  then on it goes by the brain's latest words.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Brain.Keyboard;
with Driver.Images;
with Driver.Observations;
with Driver.World;

package Driver.Brain.Names is

   use Ada.Strings.Unbounded;

   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Thing_Id is Driver.World.Thing_Id;
   subtype Place_Id is Driver.World.Place_Id;
   subtype Word_List is Driver.Brain.Keyboard.Word_Vectors.Vector;

   function Letters (W : String) return String;
   --  The letters a to z of W, in lower case.

   function Same_Name (A, B : String) return Boolean;
   --  Equal letters, and at least one.

   function Same_Core (A, B : String; Glue : Word_List) return Boolean;
   --  Equal once each has shed the language words glued to its ends. A name
   --  made only of language words has no core and matches nothing.

   --  What the body can ask while binding. The live body asks the brain
   --  service, the instrument, the world and the robot; tests and replays
   --  answer from fixtures.

   type Pointing is (Boxed, Not_Here, No_Answer);
   --  The eye boxed it, said it is not in this picture, or could not be
   --  asked (the brain service failed).

   type Box is record
      Top_Left, Bottom_Right : Driver.Images.Pixel;
   end record;
   --  In the pixel coordinates of Driver.Images.

   type Patch is (A_Thing, Part_Of_Me, No_Patch);
   --  What the boxed pixels turned out to be: a thing (adopted or found),
   --  the body itself, or nothing that stands apart from its surroundings.

   package Eye_Vectors renames Driver.Brain.Keyboard.Eye_Vectors;

   type Senses is limited interface;

   function Eyes (S : Senses) return Eye_Vectors.Vector is abstract;
   --  Every eye with a picture now, in camera order.

   function Sees (S : Senses; T : Thing_Id; E : Eye_Id) return Boolean is abstract;

   procedure Ask_Where
     (S      : in out Senses;
      E      : Eye_Id;
      Name   : String;
      Answer : out Pointing;
      Where  : out Box;
      Why    : out Unbounded_String) is abstract;

   procedure Identify
     (S     : in out Senses;
      E     : Eye_Id;
      Where : Box;
      Found : out Patch;
      T     : out Thing_Id;
      Why   : out Unbounded_String) is abstract;

   --  The names given in one episode.

   type Table is tagged private;

   procedure Name_Place (N : in out Table; Name : String; P : Place_Id);
   --  remember ... as Name: from now on Name is that place.

   function Name_Of (N : Table; T : Thing_Id) return String;
   --  The latest words the brain used for a thing, or "" if it has none.

   function Named_Count (N : Table) return Natural;
   function Named_Thing (N : Table; Index : Positive) return Thing_Id
     with Pre => Index <= Named_Count (N);

   type Binding_Kind is (To_Thing, To_Place, Unbound);

   type Binding is record
      Kind    : Binding_Kind := Unbound;
      Thing   : Thing_Id := Thing_Id'First;
      Place   : Place_Id := Place_Id'First;
      Account : Unbounded_String;   --  how it was bound, or every reason it was not
   end record;

   procedure Bind
     (N      : in out Table;
      S      : in out Senses'Class;
      View   : Eye_Id;
      Name   : String;
      Glue   : Word_List;
      Result : out Binding);
   --  Steps 0 to 3 for one name. View is the eye the brain looks through
   --  this round; Glue the words a name may not be this round.

   procedure Bind_Again (N : Table; Glue : Word_List; Name : String; Result : in out Binding);
   --  Step 4: an unbound name tries the letters again, against every name
   --  given so far.

private

   type Named is record
      Name  : Unbounded_String;
      Thing : Thing_Id;
   end record;

   package Named_Vectors is new Ada.Containers.Vectors (Positive, Named);

   type Place_Name is record
      Name  : Unbounded_String;
      Place : Place_Id;
   end record;

   package Place_Vectors is new Ada.Containers.Vectors (Positive, Place_Name);

   type Table is tagged record
      Things : Named_Vectors.Vector;
      Places : Place_Vectors.Vector;
   end record;

end Driver.Brain.Names;
