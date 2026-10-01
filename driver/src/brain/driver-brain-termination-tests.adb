with Ada.Strings.Unbounded;
with Driver.Brain.Names;
with Driver.Brain.Parser;
with Driver.Tests;

package body Driver.Brain.Termination.Tests is

   use Ada.Strings.Unbounded;
   use Driver.Brain.Programs;
   use Driver.Tests;

   LF : constant String := [ASCII.LF];

   function Same (A, B : String) return Boolean is (Driver.Brain.Names.Same_Name (A, B));

   --  The line refused, 0 when the program can end.
   function Refused_Line (Text : String; Before : Last_Endings := No_Stretch_Yet) return Natural is
      P   : Program;
      Ok  : Boolean;
      Why : Refusal;
   begin
      Driver.Brain.Parser.Parse (Text, P, Ok, Why);
      Check (Ok, "the test program does not parse: " & To_String (Why.Why));
      return Check (P, Before, Same'Access).Line;
   end Refused_Line;

   procedure Loops is
      Waits_On_Talk : constant String := "repeat until touched:" & LF & "say I wait" & LF & "end";
      Moves         : constant String :=
        "repeat until touched:" & LF & "do grasper nearer ball until timeout or 5 steps" & LF & "end";
      Gated         : constant String :=
        "repeat until touched:" & LF & "if stuck:" & LF & "do grasper open until settled" & LF & "end" & LF & "end";
   begin
      Check (Refused_Line (Waits_On_Talk) = 1, "a loop whose passes run no stretch never sees its ending");
      Check (Refused_Line (Moves) = 0, "a stretch inside the loop may end with anything, so the loop may end");
      Check (Refused_Line (Gated, Exactly (Driver.Action.Settled)) = 1,
             "entered after settled, the only stretch is skipped on every pass");
      Check (Refused_Line (Gated, Exactly (Driver.Action.Stuck)) = 0,
             "entered after stuck, the first pass runs the stretch");
      Check (Refused_Line ("repeat until touched:" & LF & "done" & LF & "end") = 0, "done ends a loop");
      Check (Refused_Line ("repeat 99999999 times:" & LF & "say hi" & LF & "end") = 0,
             "a counted loop ends whatever its count, and nothing is counted out");
      Check (Refused_Line ("try:" & LF & "repeat until stuck:" & LF & "say x" & LF & "end" & LF & "or:" & LF & "say y"
             & LF & "end") = 2, "an endless loop inside a try is still endless");
      Check (Refused_Line ("try:" & LF & "repeat until stuck:" & LF & "do me left table until settled" & LF & "end" & LF
             & "or:" & LF & "say y" & LF & "end") = 0,
             "inside a try a stuck stretch leaves for the alternative, which ends the loop");
   end Loops;

   procedure Calls is
      Endless : constant String :=
        "to dance:" & LF & "do me left table until settled" & LF & "run dance" & LF & "end" & LF & "run dance";
      Way_Out : constant String :=
        "to search:" & LF & "try:" & LF & "do me left table until stuck" & LF & "or:" & LF & "run search" & LF & "end"
        & LF & "end" & LF & "run search";
   begin
      Check (Refused_Line (Endless) = 3, "a behaviour that calls itself on every path never ends");
      Check (Refused_Line (Way_Out) = 0, "a behaviour that calls itself only when a stretch fails may end");
      Check (Refused_Line ("run nothing") = 1, "a call to a behaviour never defined");
      Check (Refused_Line ("to a:" & LF & "say x" & LF & "end" & LF & "to A:" & LF & "say y" & LF & "end") = 1,
             "two behaviours under one name");
      Check (Refused_Line ("to look:" & LF & "say x" & LF & "end" & LF & "run Look") = 0,
             "a behaviour is called by its letters");
   end Calls;

   procedure Register is
   begin
      Register ("brain.termination.loops", "a loop that can never end is run, or one that can end is refused",
                Loops'Access);
      Register ("brain.termination.calls", "a behaviour that never returns is run, or a call is left unchecked",
                Calls'Access);
   end Register;

end Driver.Brain.Termination.Tests;
