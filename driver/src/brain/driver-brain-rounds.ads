--  The rounds of one episode: look, ask the brain for a program, read it,
--  bind its names, check it, run it, tell the brain what happened, until
--  the brain says done or the episode ends.
--
--  Everything outside the brain layer is reached through four interfaces:
--  what the body shows this beat (Surroundings), the brain service
--  (Thinker), the eyes for binding names (Driver.Brain.Names.Senses) and the
--  body that checks and runs stretches (Driver.Brain.Execution.Performer).
--  The live body implements them in Driver.Brain.Live; tests and the
--  measurement tools implement them from fixtures.

with Ada.Strings.Unbounded;
with Driver.Brain.Execution;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Brain.Round;
with Driver.Brain.Service;
with Driver.Images;
with Driver.Observations;

package Driver.Brain.Rounds is

   use Ada.Strings.Unbounded;

   type Snapshot is record
      Images      : Driver.Observations.Image_Vectors.Vector;   --  one per eye, No_Image when missing this beat
      Eyes        : Driver.Brain.Round.Eye_Fact_Vectors.Vector;
      Things      : Driver.Brain.Round.Thing_Fact_Vectors.Vector;   --  the named things, as measured now
      Keys        : Driver.Brain.Keyboard.Keyboard;
      Instruction : Unbounded_String;   --  the person's latest words
   end record;

   type Surroundings is limited interface;

   procedure Look (S : in out Surroundings; Named : Driver.Brain.Names.Table; Now : out Snapshot) is abstract;
   --  What the body shows at one beat, the named things included.

   function Episode_Over (S : Surroundings) return Boolean is abstract;

   type Thinker is limited interface;

   function Write_Program
     (T       : in out Thinker;
      Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Driver.Brain.Service.Answer is abstract;

   procedure Run
     (Instruction : String;
      Around      : in out Surroundings'Class;
      Brain       : in out Thinker'Class;
      Eyes        : in out Driver.Brain.Names.Senses'Class;
      Doer        : in out Driver.Brain.Execution.Performer'Class);

end Driver.Brain.Rounds;
