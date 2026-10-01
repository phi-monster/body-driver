--  When to stop reading a streamed answer.
--
--  The grammar bounds no slot and no line count, so an answer can go on
--  until the service's context is full. The driver stops reading when the
--  answer is complete or has run away, and keeps the lines finished before
--  that point:
--
--    complete  a done outside every block: nothing after it can ever run.
--    runaway   the start of a copy loop: a run of whole words inside one
--              sentence comes again at once, or a run of whole lines does,
--              or inside one name a run of units: the language words glued
--              into its words, the letters between them, and words without
--              any. Only finished lines and words are compared, and of a
--              word still being written only the units no later letter can
--              change, so the verdict depends on the text alone, never on
--              how the stream was cut into pieces.

with Ada.Strings.Unbounded;
with Driver.Brain.Keyboard;

package Driver.Brain.Runaway is

   use Ada.Strings.Unbounded;

   type Verdict is record
      Complete : Boolean := False;   --  a done outside every block was read
      Fired    : Boolean := False;   --  the answer ran away
      Keep     : Natural := 0;       --  the lines to keep, from the first
      Why      : Unbounded_String;   --  for the log: which rule, which words or lines
   end record;

   function Judge
     (Text       : String;
      Name_Words : Driver.Brain.Keyboard.Word_Vectors.Vector;
      Final      : Boolean) return Verdict;
   --  Text: the answer received so far. Name_Words: the words a name may
   --  not be this round; they separate the name slots of a line. Final: the
   --  answer has ended, so its last line and last word are finished.

   function Stop_Reading (V : Verdict) return Boolean is (V.Complete or else V.Fired);

   function First_Lines (Text : String; Count : Natural) return String;
   --  The first Count lines of Text, each ending in a line break.

end Driver.Brain.Runaway;
