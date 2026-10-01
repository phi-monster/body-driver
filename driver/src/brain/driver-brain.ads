--  Layer 5: rounds with the brain.
--
--  Each round shows the brain what the body sees and what it can do now (the
--  keyboard, built from what the body measured), reads back a program in the
--  body language, binds the names in it to things, and runs it through
--  Driver.Action, reporting every interval's ending in the next round. The
--  brain service is reached only through Driver.Services; its sampling
--  settings come from the deployment, never from the driver.
--
--  Ownership: path D.

with Driver.Action;

package Driver.Brain is

   procedure Run_Episode (C : in out Driver.Action.Context; Instruction : String);
   --  Decider: rounds until the brain says done or a new episode begins.

end Driver.Brain;
