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
with Driver.Observations;

package Driver.Beats is

   procedure Next (Beat : out Driver.Clock.Beat);
   --  Decider side: wait for the next offered beat.

   type Observation_View is access constant Driver.Observations.Observation;

   function Latest return Observation_View;
   --  Decider side: the observation of the beat Next returned. The view is
   --  valid only until Send; keep a copy for anything needed longer.

   function Last_Sent return Driver.Commands.Command;
   --  Decider side: what was sent to the robot for the previous beat, holds
   --  included (the command in effect while Latest was captured).

   procedure Send (C : Driver.Commands.Command);
   --  Decider side: the command for the beat Next returned.

   procedure Release;
   --  Decider side, after a failure: if the decider holds a beat (Next
   --  returned, Send not yet called), it is answered with hold, so the main
   --  loop never waits on a decider that has stopped.

   function Held return Boolean;
   --  A beat is held now: Next has returned and Send has not been called yet,
   --  the only window in which a decider may read and change the models.
   --  Code that must run only there, or never there, asks this.

   procedure Within_A_Beat (During : not null access procedure);
   --  Decider side: takes the next beat, runs During in its window (the
   --  models hold still until it returns) and answers the beat with a hold,
   --  so a decider that only looks never moves the robot. A failure in
   --  During still answers the beat before it propagates.

   procedure Offer
     (Beat         : Driver.Clock.Beat;
      O            : Driver.Observations.Observation;
      Sent_Before  : Driver.Commands.Command;
      Decider_Took : out Boolean);
   --  Main side: offer a beat with its observation and the command sent for
   --  the previous beat; Decider_Took is False when no decider was waiting
   --  in Next, and the beat then gets Hold.

   procedure Await (C : out Driver.Commands.Command);
   --  Main side: after Offer took the beat, wait for the decider's Send.

   procedure New_Episode;
   --  Main side: the robot announced a new episode (a reset message).

   function Episode return Natural;
   --  How many new episodes have been announced; a decider compares it
   --  across beats to notice that the episode it was working on has ended.

   procedure Hear (Words : String);
   --  Main side: what the person said, as the latest observation carries it
   --  (an observation without words leaves the last ones standing).

   function Latest_Words return String;
   --  Decider side, at any time: the person's latest words.

   function Words_Heard return Natural;
   --  How many times the person's words have changed: a decider compares it
   --  over time to notice that something new was said.

end Driver.Beats;
