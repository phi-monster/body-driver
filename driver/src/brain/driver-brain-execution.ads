--  Running a checked program (LANGUAGE.md 6, 17.3 and 17.8).
--
--  Statements run in order; each stretch goes to the body as one
--  Driver.Action.Want and ends with one ending, the only thing control flow
--  reads:
--
--    if E          reads how the last stretch ended; that carries over from
--                  the programs before it in the same episode
--    repeat n      runs its lines n times
--    repeat until  runs its lines, and again until a pass ends with E
--    try ... or    a stretch inside the attempt that ends stuck, slipped,
--                  lost, stalled, timeout or refused abandons it for the or
--                  lines, from however deep it was called
--    to, run       a behaviour is defined anywhere in the program and run by
--                  its letters
--    remember      the body records where something is, as a place
--    say           the sentence goes to the log and to the next round
--    done          the task is finished: nothing more runs this episode
--
--  The interpreter keeps its own stack, so a behaviour that calls itself
--  deeply costs memory, not the decider's stack. Before every statement it
--  asks the body whether the run was interrupted (a new episode, or new
--  words from the person), and stops if so.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Action;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Brain.Programs;
with Driver.Brain.Termination;
with Driver.Brain.Wants;
with Driver.World;

package Driver.Brain.Execution is

   use Ada.Strings.Unbounded;

   type Performer is limited interface;

   function Quantities (P : Performer) return Driver.Brain.Keyboard.Word_Vectors.Vector is abstract;
   --  This round's quantity words, in the order of Driver.Action.Quantities.

   function Check (P : Performer; W : Driver.Action.Want) return Driver.Action.Verdict is abstract;

   procedure Execute (P : in out Performer; W : Driver.Action.Want; R : out Driver.Action.Result) is abstract;

   procedure Place_Of
     (P     : in out Performer;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String) is abstract;
   --  remember where Who is: a place the body can find again.

   procedure Say (P : in out Performer; Sentence : String) is abstract;

   function Interrupted (P : Performer) return Boolean is abstract;

   type Event_Kind is (Stretch, Spoken, Remembered, Not_Remembered);

   type Event is record
      Kind    : Event_Kind := Stretch;
      Line    : Positive := Positive'First;
      Text    : Unbounded_String;                         --  the line as written
      Ending  : Driver.Action.Ending := Driver.Action.Refused;   --  for Stretch
      Account : Unbounded_String;   --  what moved, what was tried, or why nothing could be done
   end record;

   package Event_Vectors is new Ada.Containers.Vectors (Positive, Event);

   type Run_End is (Finished, Said_Done, Stopped);
   --  Finished: the last line ran. Said_Done: a done ran. Stopped: interrupted.

   type Run_Report is record
      How    : Run_End := Finished;
      Events : Event_Vectors.Vector;
      Moved  : Boolean := False;   --  some stretch reached the body
   end record;

   procedure Run
     (P      : Driver.Brain.Programs.Program;
      Bound  : in out Driver.Brain.Wants.Binding_Maps.Map;
      Names  : in out Driver.Brain.Names.Table;
      Doer   : in out Performer'Class;
      Last   : in out Driver.Brain.Termination.Last_Endings;
      Report : out Run_Report);
   --  Bound: the program's bindings; remember adds the places it records.
   --  Last: how the last stretch of the episode ended, before and after.

end Driver.Brain.Execution;
