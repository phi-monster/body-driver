--  Boot: the decider that measures the body.
--
--  From zero it recognizes the groups, measures response, noise and limits,
--  fits the kinematics, the lenses and where every eye is mounted, measures
--  the link shapes, and calls Driver.Robot.Hand.Measure. With a body file
--  from an earlier boot it reloads every quantity whose measuring method has
--  not changed since, checks the reloaded body against what it sees now, and
--  re-measures only what is stale or contradicted.

with Driver.Robot.Hand;

package Driver.Robot.Boot is

   procedure Run (M : in out Model; H : in out Driver.Robot.Hand.Hands; Body_File : String; Ok : out Boolean);
   --  Body_File may be empty (measure from zero, store nothing) or name a
   --  file to reload from and store into. Ok is False when the body breaks
   --  the porting contract (docs/body-protocol.md); the log says which clause.

   function Grew (Before, After, Cells : Integer) return Boolean;
   --  Whether After of the Cells of an eye respond to a group where Before did
   --  a round of pushes earlier: more than the false alarms among the cells
   --  that did not respond could make (the exact binomial tail of the new
   --  responders among them, at the per-cell false-alarm rate, below Z's
   --  tail). The boot pushes a group again, at twice the amounts, while an
   --  eye that is undecided about it or shows a patch of it grew by this.

   function Pause_After_A_Push (Every_Lag_Known : Boolean; Longest_Lag : Natural; Guess : Natural) return Natural is
     (if Every_Lag_Known then Longest_Lag else Guess);
   --  How many beats a recognition round holds after each push before the
   --  pictures are asked to settle. The hold is for an eye whose picture
   --  trails the readings: settled the beat after they stop, before it has
   --  moved. Once every eye's lag is measured, the hold is the longest of
   --  them, and nothing more; until then it is Guess, the longest lag the
   --  stream could tell when the rounds began, doubled with the rounds (the
   --  doubling convention, as the amounts are). B0_1 (x5, 10-12) doubled it
   --  with the lags measured at 1 beat from beat 405 on: 12, 24, 48 ... 384
   --  beats a hold, 3456 of a round's 3516 beats spent holding at 32 times
   --  the amounts, and the groups were never recognized in 180 minutes.

   procedure Save (M : Model; H : Driver.Robot.Hand.Hands; Body_File : String);
   --  Writes everything measured so far, with uncertainties and method
   --  versions, to the body file. Boot calls it as it measures; the replay
   --  tool calls it at the end so the estimates can be scored against truth.

end Driver.Robot.Boot;
