--  The beat channel between the main loop and the decider task.
--
--  The main loop owns the wire: every beat it reads one observation, runs
--  every estimator on it, then offers the beat to the decider. A decider is
--  written as sequential code (boot, then rounds with the brain) and moves
--  the robot by calling Next and Send:
--
--     loop
--        Next (Beat);              --  blocks until the next beat is offered
--        ...read the models, choose...
--        Send (Command);           --  the reply for this beat
--     end loop;
--
--  Ownership is what keeps this safe without locks on every model: the
--  decider may read and change the models only between Next returning and
--  Send; the main loop touches them only outside that window. A decider that
--  is busy elsewhere (waiting for the brain, say) is simply not waiting in
--  Next, and the main loop then replies Hold on its own and moves on.

with Driver.Clock;
with Driver.Commands;

package Driver.Beats is

   procedure Next (Beat : out Driver.Clock.Beat);
   --  Decider side: wait for the next offered beat.

   procedure Send (C : Driver.Commands.Command);
   --  Decider side: the command for the beat Next returned.

   procedure Offer (Beat : Driver.Clock.Beat; Decider_Took : out Boolean);
   --  Main side: offer a beat; Decider_Took is False when no decider was
   --  waiting in Next, and the beat then gets Hold.

   procedure Await (C : out Driver.Commands.Command);
   --  Main side: after Offer took the beat, wait for the decider's Send.

   procedure New_Episode;
   --  Main side: the robot announced a new episode (a reset message).

   function Episode return Natural;
   --  How many new episodes have been announced; a decider compares it
   --  across beats to notice that the episode it was working on has ended.

end Driver.Beats;
