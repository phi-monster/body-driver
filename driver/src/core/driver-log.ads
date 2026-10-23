--  Log lines: one line per event, "[topic] text", written to standard output
--  and flushed, so a log read live is never behind. The meaning of every line
--  a user may see is listed in docs/log-lines.md.

package Driver.Log is

   type Topic is (Core, Robot, World, Action, Brain);

   procedure Line (T : Topic; Text : String);

   procedure Stamp (Beat : Natural);
   --  From then on each line begins with "@", the beat given last and a blank:
   --  the replay says so at which beat of a recording each line was written.

   function Image (X : Real; Digits_After_Point : Natural := 3) return String;
   --  A fixed-point rendering without the leading blank of Real'Image.

   function Image (N : Integer) return String;

   procedure Check_Silence;
   --  Called by the decider as it takes each beat (Driver.Beats.Next): when the deciding tasks have written no robot,
   --  action or brain line for twice the longest such silence before this one, a core line says how long and where the calling
   --  task waits (its call chain, the functions and lines), and again at twice that, until a line is written. A wait
   --  that tells nothing of itself was found only by reading the code: k204sd (10-23) held still and silent from beat
   --  3287 to the run's end, 20 000 beats, after a press.

end Driver.Log;
