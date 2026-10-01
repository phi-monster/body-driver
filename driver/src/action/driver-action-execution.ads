--  A want carried out on a plant, one step at a time, to one ending.
--
--  The thing a want is about is first brought under control: if no hand
--  holds it, the contact search (Driver.Action.Contact.Search) picks where
--  the body meets it for the wanted motion, and the body goes there by the
--  lowest way around what is in between, opens its closers as far as the
--  way in needs, comes in along the last straight stretch and closes until
--  the closers stop on it. Then every step moves it a part of the way, as
--  far as three measured bounds allow: the arm reaches there, an eye that
--  stays still keeps seeing it, and nothing it could meet is nearer than its
--  own uncertainty (from there on it creeps by the arm's smallest step);
--  a wanted ending that the motion is about to bring about bounds the step
--  too, so the step does not overshoot it. After every step the monitor
--  (Driver.Action.Monitor) reads the measured facts; the interval ends with
--  the ending it names. A thing brought down onto a surface is let go only
--  when it rests there. What failed is handed back to the plant (a friction
--  that let the thing slip), so the next choice differs. Nothing here knows
--  which body or which task it serves.

with Driver.Action.Plants;

package Driver.Action.Execution is

   procedure Execute (P : in out Driver.Action.Plants.Plant'Class; W : Want; R : out Result);

end Driver.Action.Execution;
