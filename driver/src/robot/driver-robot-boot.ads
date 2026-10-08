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

   procedure Save (M : Model; H : Driver.Robot.Hand.Hands; Body_File : String);
   --  Writes everything measured so far, with uncertainties and method
   --  versions, to the body file. Boot calls it as it measures; the replay
   --  tool calls it at the end so the estimates can be scored against truth.

end Driver.Robot.Boot;
