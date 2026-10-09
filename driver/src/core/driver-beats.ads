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

   generic
      with procedure Work (Lane : Positive);
   procedure At_Once (Lanes : Positive);
   --  Decider side, outside a held beat: runs Work (1) .. Work (Lanes) at once,
   --  each a decider of its own (a lane) on a task of its own, and returns
   --  when every one has ended. Every beat goes to the lanes waiting in Next,
   --  one after another: a lane's window (from its Next to its Send) is never
   --  open while another's is, so the models are read and changed by one lane
   --  at a time as by one decider, and what each lane sends is merged into
   --  the one reply of the beat (Driver.Commands.Merge). Each lane moves its
   --  own groups: two lanes targeting one group in a beat is an error that
   --  ends the second. A lane busy elsewhere misses the beat, and its groups
   --  hold. Next, Send, Held, Release, Within_A_Beat and Wait_For_Estimates
   --  act for the lane of the task that calls them. An exception a lane
   --  raised is raised again by At_Once once every lane has ended; a lane
   --  that ends while it holds a beat answers it with a hold.

   function Lane return Natural;
   --  The lane of the calling task: 0 for the decider itself, 1 and up for
   --  the lanes of At_Once.

   procedure Within_A_Beat (During : not null access procedure);
   --  Decider side: takes the next beat, runs During in its window (the
   --  models hold still until it returns) and answers the beat with a hold,
   --  so a decider that only looks never moves the robot. A failure in
   --  During still answers the beat before it propagates.

   procedure Wait_For_Estimates;
   --  Decider side, in a held beat, when the estimates it asked for are
   --  computed apart from the main loop (Driver.Robot.Estimate_Now): answers
   --  the beat with a hold, waits until the main loop has adopted them, then
   --  takes the next beat, so the caller goes on inside a held beat. The robot
   --  is answered every beat meanwhile.

   procedure Estimates_Adopted;
   --  Main side: the estimates computed apart are in the models; a decider
   --  waiting for them goes on. With nobody waiting it does nothing.

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
