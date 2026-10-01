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
      when E : Program_Error =>
         --  The decider task must not die silently; the robot keeps holding.
         Driver.Log.Line (Driver.Log.Brain, "the rounds stopped: " & Ada.Exceptions.Exception_Message (E));
   end Run_Episode;

end Driver.Brain;
