--  The live body behind the interfaces of a round: the robot, its hands and
--  the scene through Driver.Action, Driver.Robot and Driver.World; the brain
--  and the instrument through Driver.Services.
--
--  The models may be read only inside a beat's window (Driver.Beats): every
--  reading here happens inside one, and every call to a service, which takes
--  seconds, happens between windows while the main loop keeps the body
--  holding still.

with Ada.Strings.Unbounded;
with Driver.Action;
with Driver.Brain.Execution;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Brain.Rounds;
with Driver.Brain.Service;
with Driver.Images;
with Driver.Observations;
with Driver.World;

package Driver.Brain.Live is

   use Ada.Strings.Unbounded;

   type Body_Link (C : not null access Driver.Action.Context) is limited new
     Driver.Brain.Rounds.Surroundings and Driver.Brain.Names.Senses and Driver.Brain.Execution.Performer
   with private;

   procedure Start (B : in out Body_Link);
   --  Serve the episode that is current now.

   overriding procedure Look
     (B : in out Body_Link; Named : Driver.Brain.Names.Table; Now : out Driver.Brain.Rounds.Snapshot);
   overriding function Episode_Over (B : Body_Link) return Boolean;

   overriding function Eyes (B : Body_Link) return Driver.Brain.Names.Eye_Vectors.Vector;
   overriding function Sees (B : Body_Link; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean;
   overriding procedure Ask_Where
     (B      : in out Body_Link;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String);
   overriding procedure Identify
     (B     : in out Body_Link;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String);

   overriding function Quantities (B : Body_Link) return Driver.Brain.Keyboard.Word_Vectors.Vector;
   overriding function Check (B : Body_Link; W : Driver.Action.Want) return Driver.Action.Verdict;
   overriding procedure Execute (B : in out Body_Link; W : Driver.Action.Want; R : out Driver.Action.Result);
   overriding procedure Place_Of
     (B     : in out Body_Link;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String);
   overriding procedure Say (B : in out Body_Link; Sentence : String);
   overriding function Interrupted (B : Body_Link) return Boolean;

   type Brain_Link is limited new Driver.Brain.Rounds.Thinker with null record;

   overriding function Write_Program
     (T       : in out Brain_Link;
      Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Driver.Brain.Service.Answer;

   function Mostly (Part, Whole : Driver.Images.Mask) return Boolean;
   --  More than half of Whole's pixels lie in Part. A patch is what most of
   --  its pixels are, never what one point of it is: a finger over the middle
   --  of a thing does not make the thing part of the body.

private

   type Body_Link (C : not null access Driver.Action.Context) is limited new
     Driver.Brain.Rounds.Surroundings and Driver.Brain.Names.Senses and Driver.Brain.Execution.Performer
   with record
      Episode : Natural := 0;                         --  the episode this link serves
      Heard   : Natural := 0;                         --  the person's words as of the last look
      Now     : Driver.Observations.Observation;      --  the beat last looked at
   end record;

end Driver.Brain.Live;
