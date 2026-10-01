--  What the brain is told each round, besides the picture.
--
--  The text says what the picture shows, which things the brain has named
--  and what the body measured about them, what happened since the last
--  question, the task, and this round's keyboard. It explains the format
--  only and never how to act; a gate (tools/check.sh, prompt) rejects
--  tutorial sentences in this layer.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Brain.Execution;
with Driver.Brain.Names;
with Driver.Brain.Programs;
with Driver.Robot;
with Driver.Uncertain;

package Driver.Brain.Round is

   use Ada.Strings.Unbounded;

   type Eye_Facts is record
      Eye   : Driver.Brain.Names.Eye_Id;
      Mount : Driver.Robot.Mount;
   end record;

   package Eye_Fact_Vectors is new Ada.Containers.Vectors (Positive, Eye_Facts);

   type Thing_Facts is record
      Name    : Unbounded_String;
      Seen_By : Driver.Brain.Names.Eye_Vectors.Vector;   --  eyes that see it now
      Held    : Boolean := False;
      Height  : Driver.Uncertain.Estimate;              --  above what it rests on; unknown when nothing is measured
   end record;

   package Thing_Fact_Vectors is new Ada.Containers.Vectors (Positive, Thing_Facts);

   type Facts is record
      View        : Driver.Brain.Names.Eye_Id;
      Strip       : Driver.Brain.Names.Eye_Vectors.Vector;   --  the other eyes in the picture, left to right
      Eyes        : Eye_Fact_Vectors.Vector;                 --  every eye of the body
      Things      : Thing_Fact_Vectors.Vector;
      Happened    : Unbounded_String;
      Instruction : Unbounded_String;
      Sheet       : Unbounded_String;
   end record;

   function Prompt (F : Facts) return String;

   First_Round : constant String := "Nothing yet: this is the first round of the task.";

   function Happened_Text (Report : Driver.Brain.Execution.Run_Report; Cut : String) return String;
   --  What the last program did, line by line. Cut: why the answer was cut
   --  short, or "" when it was read whole.

   function Refusal_Text (R : Driver.Brain.Programs.Refusal) return String;
   --  A program refused before anything moved.

   function Look_At (Sentence : String) return Natural;
   --  The eye a sentence of the form "look = <n>" asks for, or 0.

end Driver.Brain.Round;
