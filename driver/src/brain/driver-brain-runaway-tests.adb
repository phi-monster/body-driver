with Driver.Action;
with Driver.Brain.Keyboard;
with Driver.Tests;

package body Driver.Brain.Runaway.Tests is

   use Driver.Brain.Keyboard;
   use Driver.Tests;

   LF : constant String := [ASCII.LF];

   --  The quantity keyboard the recorded answers below were written on.
   function Names return Word_Vectors.Vector is
      Q, M : Word_Vectors.Vector;
   begin
      Q.Append ("height");
      M.Append ("");
      return Name_Words (Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True], True,
                                 [others => False], Eye_Vectors.Empty_Vector));
   end Names;

   function J (Text : String; Final : Boolean := True) return Verdict is (Judge (Text, Names, Final));

   procedure Complete_At_Done is
      Repeated_Done : constant String :=
        "do scissors height down until touched" & LF & "do scissors height up until settled" & LF & "say done" & LF
        & "done" & LF & "done" & LF & "done" & LF;
      V : constant Verdict := J (Repeated_Done);
      Nested : constant Verdict :=
        J ("if slipped:" & LF & "done" & LF & "end" & LF & "do scissors height up until settled" & LF);
   begin
      Check (V.Complete and then not V.Fired and then V.Keep = 4
             and then First_Lines (Repeated_Done, V.Keep) = "do scissors height down until touched" & LF
                        & "do scissors height up until settled" & LF & "say done" & LF & "done" & LF,
             "a done outside every block completes the answer; what follows is never read");
      Check (not Nested.Complete and then not Nested.Fired and then Nested.Keep = 4,
             "a done inside a block does not end the answer");
   end Complete_At_Done;

   procedure Copy_Loops is
      Name_Loop : constant String :=
        "do pick upthe red cupheight ight untilstuck untiltimeout doneuntilfree untiltimeout doneuntilfree untiltimeout";
      Sentence_Loop : constant String :=
        "do the scissors height down until touched" & LF & "say i am grabbing the mint green scissors" & LF
        & "do the scissors height up until settled" & LF & "say done. done. done." & LF;
      Line_Pair_Loop : constant String :=
        "do the cup height up until settled" & LF & "say i lift it" & LF & "do the cup height up until settled" & LF
        & "say i lift it" & LF & "do the cup";
      A : constant Verdict := J (Name_Loop, Final => False);
      B : constant Verdict := J (Sentence_Loop);
      C : constant Verdict := J (Line_Pair_Loop, Final => False);
   begin
      Check (A.Fired and then A.Keep = 0, "a run of words inside a name that comes again at once: nothing is kept");
      Check (B.Fired and then B.Keep = 3, "a word inside a sentence that comes again at once: the lines before stay");
      Check (C.Fired and then C.Keep = 2, "two lines that come again at once: the first copy stays");
   end Copy_Loops;

   procedure Right_Answers is
      One : constant String := "do the baseball height up until settled" & LF & "say I lifted it" & LF;
      Two : constant String := "do the lego man height up until settled" & LF & "do the lego man height down until"
        & " touched" & LF & "say done" & LF & "done" & LF;
   begin
      Check (not J (One).Fired and then J (One).Keep = 2, "an answer that ended by itself is kept whole");
      Check (J (Two).Complete and then J (Two).Keep = 4 and then not J (Two).Fired, "a right answer ending in done");
   end Right_Answers;

   procedure Unfinished_Words is
   begin
      Check (not J ("do the mint green scissors under cell elli elli", Final => False).Fired,
             "a word still being written is not compared: elli may yet become ellipse");
      Check (J ("do the mint green scissors under cell elli elli ", Final => False).Fired,
             "once a blank follows it, it is");
   end Unfinished_Words;

   --  Fed one character at a time, the first verdict that stops reading keeps
   --  the same lines as the verdict on the whole text.
   procedure Pieces_Do_Not_Matter is
      Text : constant String :=
        "say i will lift the cup" & LF & "do the cup height up until settled" & LF & "do the cup height up until"
        & " settled" & LF & "say more text that would follow" & LF;
      Whole : constant Verdict := J (Text);
   begin
      for Last in Text'Range loop
         declare
            V : constant Verdict := J (Text (Text'First .. Last), Final => False);
         begin
            if Stop_Reading (V) then
               Check (V.Keep = Whole.Keep and then V.Fired = Whole.Fired,
                      "stopped at character" & Last'Image & " keeping" & V.Keep'Image & " lines, the whole text keeps"
                      & Whole.Keep'Image);
               return;
            end if;
         end;
      end loop;
      Check (False, "the repeated line was never noticed while streaming");
   end Pieces_Do_Not_Matter;

   --  A line still being written may equal the line before it and then go on.
   procedure Unfinished_Lines is
      Text : constant String :=
        "do me above table until touched" & LF & "do me above table until touched or 5 steps" & LF & "done" & LF;
      Names_Of_Full : Word_Vectors.Vector;
   begin
      Names_Of_Full.Append ("until");
      for Last in Text'Range loop
         Check (not Judge (Text (Text'First .. Last), Names_Of_Full, Final => False).Fired,
                "fired at character" & Last'Image & " on a line that was not finished");
      end loop;
   end Unfinished_Lines;

   procedure Register is
   begin
      Register ("brain.runaway.done", "an answer is read past the done that ends it, or cut at a done inside a block",
                Complete_At_Done'Access);
      Register ("brain.runaway.loops", "a copy loop in a name, a sentence or a run of lines is read to the end",
                Copy_Loops'Access);
      Register ("brain.runaway.right", "an answer that ended by itself is cut", Right_Answers'Access);
      Register ("brain.runaway.unfinished", "a half-written word is compared as if finished",
                Unfinished_Words'Access);
      Register ("brain.runaway.pieces", "where the stream is cut into pieces changes what is kept",
                Pieces_Do_Not_Matter'Access);
      Register ("brain.runaway.unfinished_line", "a half-written line is compared as if finished",
                Unfinished_Lines'Access);
   end Register;

end Driver.Brain.Runaway.Tests;
