--  Log lines: one line per event, "[topic] text", written to standard output
--  and flushed, so a log read live is never behind. The meaning of every line
--  a user may see is listed in docs/log-lines.md.

package Driver.Log is

   type Topic is (Core, Robot, World, Action, Brain);

   procedure Line (T : Topic; Text : String);

   function Image (X : Real; Digits_After_Point : Natural := 3) return String;
   --  A fixed-point rendering without the leading blank of Real'Image.

   function Image (N : Integer) return String;

end Driver.Log;
