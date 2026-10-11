with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Observations;
with Driver.Tests;

package body Driver.Replies.Tests is

   use Ada.Strings.Unbounded;
   use Driver.Tests;

   function One_Group return Driver.Observations.Layout is
      L : Driver.Observations.Layout;
   begin
      L.Groups.Append
        (Driver.Observations.Group_Info'(Path        => To_Unbounded_String ("a/k"),
                                         Keys        => To_Unbounded_String ("a" & Driver.Observations.Key_Separator & "k"),
                                         Size        => 1,
                                         Command_Key => To_Unbounded_String ("k"),
                                         Echo_Path   => Null_Unbounded_String,
                                         Echo_Keys   => Null_Unbounded_String));
      return L;
   end One_Group;

   function Reading_Of (Value : Real) return Driver.Observations.Observation is
      O : Driver.Observations.Observation;
   begin
      O.Readings.Append (Real_Array'[1 => Value]);
      O.Echoes.Append (Real_Array'[1 .. 0 => 0.0]);
      O.Turned.Append (False);
      return O;
   end Reading_Of;

   function Held_Value (S : in out State; L : Driver.Observations.Layout; Reading : Real;
                        C : Driver.Commands.Command := Driver.Commands.Hold) return Real is
      B    : Driver.Bytes.Buffer;
      Sent : Driver.Commands.Command;
   begin
      Write_Action (S, L, Reading_Of (Reading), C, B, Sent);
      return Driver.Commands.Target (Sent, 1) (1);
   end Held_Value;

   --  A group never commanded this episode, held beat after beat while its
   --  reading creeps (a body giving under its load): the action carries the
   --  reading of the beat it was first held at, every beat, not the creeping
   --  reading. Sent the reading of every beat, the robot was told each beat to
   --  stand where it had sagged to and crept without end (fake body B0F1,
   --  10-13: arm 2's fingers rose 72 cm over 8000 beats of holds). A target
   --  once sent is held afterwards; a new episode holds at its own first
   --  reading.
   procedure A_Hold_Keeps_Its_First_Reading is
      S : State;
      L : constant Driver.Observations.Layout := One_Group;
      First, Second, Third, After_Target, Next_Episode : Real;
      Target : Driver.Commands.Command := Driver.Commands.Hold;
   begin
      New_Episode (S);
      First := Held_Value (S, L, 10.0);
      Second := Held_Value (S, L, 10.1);
      Third := Held_Value (S, L, 10.2);
      Check (First = 10.0, "the first hold carries the first reading, got" & First'Image);
      Check (Second = 10.0 and then Third = 10.0, "later holds carry the creeping reading (" & Second'Image
             & "," & Third'Image & ") instead of the first, 10.0");
      Driver.Commands.Set_Target (Target, 1, [1 => 12.0]);
      After_Target := Held_Value (S, L, 10.3, Target);
      Check (After_Target = 12.0, "a target is sent as given, got" & After_Target'Image);
      After_Target := Held_Value (S, L, 11.9);
      Check (After_Target = 12.0, "after a target the hold carries that target, got" & After_Target'Image);
      New_Episode (S);
      Next_Episode := Held_Value (S, L, 20.0);
      Check (Next_Episode = 20.0, "a new episode holds at its own first reading, got" & Next_Episode'Image);
      Next_Episode := Held_Value (S, L, 20.5);
      Check (Next_Episode = 20.0, "the new episode's hold crept to" & Next_Episode'Image);
   end A_Hold_Keeps_Its_First_Reading;

   procedure Register is
   begin
      Driver.Tests.Register ("replies.held", "a group never commanded is sent its reading of every beat while held, "
                             & "so a body that gives under its load is told each beat to stand where it sagged to "
                             & "and creeps without end (B0F1)", A_Hold_Keeps_Its_First_Reading'Access);
   end Register;

end Driver.Replies.Tests;
