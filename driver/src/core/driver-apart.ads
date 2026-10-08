--  The heavier estimates computed apart from the main loop.
--
--  The robot is answered every beat. A recompute of the robot's heavier
--  estimates (Driver.Robot.Compute_Estimates) grows with the evidence behind
--  it, to minutes after a boot's worth of beats (A27: 294 s at 8642 beats);
--  inside the main loop it left the robot unanswered as long, longer than the
--  stock RoboDojo client waits (120 s) and than any real robot can be left
--  without a command. So when the estimates are due, the main loop gives the
--  models to a task of their own and goes on answering every message with
--  Hold, keeping each one back; that task computes the estimates, then gives
--  the models the messages kept back, in order, as the main loop would have
--  given them; once none is left, the main loop takes the models back and
--  offers beats to the decider again.
--
--  Each message is given to the models in two parts, the robot's (Robot_Part)
--  and the rest (Rest), because the robot's part is what makes the estimates
--  due, and when they were computed in place, within the robot's part, the
--  rest came after them. Every model therefore goes through the same steps,
--  in the same order, as when the estimates were computed in place.
--
--  While the estimator gives the models what was kept back, the main loop
--  answers a message only once fewer messages are kept back than when it
--  answered the one before (or none), so a robot that sends faster than the
--  models take its messages in cannot keep them apart for good: each answer
--  then waits for at most two messages to be taken in. While the estimator
--  computes, every message is answered at once.
--
--  Replay. The recording (Driver.Recording) holds where the models went apart
--  (kind K), each part of a message the estimator gave them (kind A) and where
--  the main loop took them back (kind B), among the robot's messages and the
--  services' replies as they happened; the Replay_ procedures give a replay's
--  models the same parts in the same order relative to those, computing the
--  estimates in place. A recording made while the estimates were computed in
--  place has no such records, and replays as it ran.

with Ada.Exceptions;

generic
   type Message is private;
   --  What one robot message gives the models.
   with procedure Robot_Part (M : Message);
   --  The robot model's part of a message (and what comes before it).
   with procedure Rest (M : Message);
   --  Everything else the message gives the models.
   with function Due return Boolean;
   --  The robot's heavier estimates are due (Driver.Robot.Estimates_Due).
   with procedure Compute;
   --  Computes them (Driver.Robot.Compute_Estimates).
   with procedure Failed (E : Ada.Exceptions.Exception_Occurrence);
   --  The estimator task failed; the models are in an unknown state.
package Driver.Apart is

   --  Main loop, for every robot message, in this order: Arrive; the message
   --  into the recording; Hold_Back if Arrive says it is held, else Take_In;
   --  the reply; After_Reply if the message was taken in whole.

   procedure Arrive (Held : out Boolean);
   --  Held: the models are apart and the message must be kept back. When the
   --  estimator has given them everything kept back and waits, the models
   --  come back here (kind B), a decider waiting for the estimates goes on
   --  (Driver.Beats.Estimates_Adopted), and Held is False.

   procedure Hold_Back (M : Message);
   --  Keeps the message back for the estimator and returns when the main loop
   --  may answer it.

   procedure Take_In (M : Message; Went_Apart : out Boolean);
   --  Gives the models the message's robot part, then the rest unless the
   --  robot part made the estimates due; then the models go apart (kind K)
   --  with the rest of this message first, and Went_Apart is True: the
   --  message is not offered to the decider.

   procedure After_Reply;
   --  When a decider asked for the estimates in the beat just answered
   --  (Driver.Robot.Estimate_Now), the models go apart (kind K).

   function Is_Apart return Boolean;

   --  Replay, record by record, in the recording's order.

   procedure Replay_Message (M : Message);
   --  A robot message (kind R): given to the models, or kept back while they
   --  are apart.

   procedure Replay_Decider;
   --  Before a record a decider wrote in its beat (kinds E, F, W): a recording
   --  made with the estimates computed in place gave the models the rest of
   --  that beat's message before the decider ran.

   procedure Replay_Apart;
   --  Kind K.

   procedure Replay_Taken_In (Ok : out Boolean);
   --  Kind A. Ok is False when nothing was kept back to take in: the
   --  recording and this code disagree.

   procedure Replay_Back (Ok : out Boolean);
   --  Kind B. Ok is False when something kept back was never taken in.

   procedure Replay_End;
   --  At the end of a recording made with the estimates computed in place:
   --  the rest of the last message.

end Driver.Apart;
