--  Whether a program can end (LANGUAGE.md 9, the dry run's loops).
--
--  Control flow reads only how the last stretch ended, and any stretch may
--  end with any ending: the body, not the program, decides that. So the
--  question is exact without running anything: follow every path the
--  program can take, with the set of endings the last stretch may have had,
--  and look for a loop or a behaviour that no path can ever leave. Such a
--  loop would hold the body still forever, because nothing inside it can
--  change what it waits for; it is refused before anything moves.
--
--  No step is counted and no bound is chosen: the sets are finite, so every
--  loop of the analysis stops when its set stops growing.

with Driver.Action;
with Driver.Brain.Programs;

package Driver.Brain.Termination is

   type Last_Endings is record
      Endings : Driver.Action.Ending_Set := [others => False];
      None    : Boolean := False;   --  no stretch has run yet this episode
   end record;
   --  The endings the last stretch may have had.

   function Exactly (E : Driver.Action.Ending) return Last_Endings;
   No_Stretch_Yet : constant Last_Endings;

   function Check
     (P       : Driver.Brain.Programs.Program;
      Before  : Last_Endings;
      Same    : not null access function (A, B : String) return Boolean) return Driver.Brain.Programs.Refusal;
   --  A refusal with Line 0 and no reason when every loop and call can end.
   --  Before is how the last stretch before the program ended; Same says
   --  whether two behaviour names are one name.

private

   No_Stretch_Yet : constant Last_Endings := (Endings => [others => False], None => True);

end Driver.Brain.Termination;
