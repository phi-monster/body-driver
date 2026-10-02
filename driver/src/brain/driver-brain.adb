with Ada.Exceptions;
with Driver.Brain.Live;
with Driver.Brain.Rounds;
with Driver.Brain.Service;
with Driver.Log;

package body Driver.Brain is

   procedure Run_Episode (C : in out Driver.Action.Context; Instruction : String) is
      Here  : aliased Driver.Action.Context (C.Robot, C.Hands, C.Scene);
      Link  : Driver.Brain.Live.Body_Link (Here'Access);
      Brain : Driver.Brain.Live.Brain_Link;
   begin
      Link.Start;
      Driver.Brain.Service.New_Episode;
      Driver.Log.Line (Driver.Log.Brain, "a new episode: " & Instruction);
      Driver.Brain.Rounds.Run (Instruction, Link, Brain, Link, Link);
   exception
      when E : others =>
         --  The decider is a task: an exception would end it without a word,
         --  and the robot would hold for good. Say so, and wait for the next episode.
         Driver.Log.Line (Driver.Log.Brain, "the rounds stopped on " & Ada.Exceptions.Exception_Name (E) & ": "
                          & Ada.Exceptions.Exception_Message (E) & "; holding still until the next episode");
   end Run_Episode;

end Driver.Brain;
