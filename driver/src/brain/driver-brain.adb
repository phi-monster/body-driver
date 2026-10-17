with Ada.Exceptions;
with Driver.Brain.Live;
with Driver.Brain.Rounds;
with Driver.Brain.Service;
with Driver.Beats;
with Driver.Log;
with Driver.Robot.Look;
with Driver.World;

package body Driver.Brain is

   procedure Run_Episode (C : in out Driver.Action.Context; Instruction : String) is
      Here  : aliased Driver.Action.Context (C.Robot, C.Hands, C.Scene);
      Link  : Driver.Brain.Live.Body_Link (Here'Access);
      Brain : Driver.Brain.Live.Brain_Link;
   begin
      Link.Start;
      Driver.Brain.Service.New_Episode;
      Driver.Log.Line (Driver.Log.Brain, "a new episode: " & Instruction);
      --  Before the brain looks, every eye an arm carries looks at the scene as it now is from places apart, so that
      --  what the brain points at in it can be placed (Driver.Robot.Look), and the scene is measured from them.
      declare
         Places : Natural := 0;
         procedure Anew is
         begin
            Driver.World.Seen_Anew (C.Scene.all);
         end Anew;
      begin
         for A in 1 .. Driver.Robot.Arm_Count (C.Robot.all) loop
            declare
               Taken : Natural;
            begin
               Driver.Robot.Look.Around (C.Robot.all, Driver.Robot.Arm_Id (A), Taken);
               Places := Places + Taken;
            end;
         end loop;
         if Places > 0 then
            Driver.Beats.Within_A_Beat (Anew'Access);
         end if;
      end;
      Driver.Brain.Rounds.Run (Instruction, Link, Brain, Link, Link);
   exception
      when E : others =>
         --  The decider is a task: an exception would end it without a word,
         --  and the robot would hold for good. Say so, and wait for the next episode.
         Driver.Log.Line (Driver.Log.Brain, "the rounds stopped on " & Ada.Exceptions.Exception_Name (E) & ": "
                          & Ada.Exceptions.Exception_Message (E) & "; holding still until the next episode");
   end Run_Episode;

end Driver.Brain;
