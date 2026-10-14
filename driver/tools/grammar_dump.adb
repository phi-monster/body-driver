--  grammar_dump DIR
--
--  Writes the GBNF of the brain's three keyboards, as the driver hands them to a brain service for constrained
--  decoding (Driver.Brain.Keyboard.Grammar), to DIR/speech.gbnf, DIR/quantity.gbnf and DIR/full.gbnf: every key
--  each layout can offer, three eyes, so that a service's grammar parser can be asked about every rule the
--  driver writes before a run depends on it (f15b on 10-15: llama.cpp refused every grammar the driver wrote
--  with status 400, for one escape).

with Ada.Command_Line;
with Ada.Text_IO;
with Driver.Action;
with Driver.Brain.Keyboard;

procedure Grammar_Dump is
   use Driver.Brain.Keyboard;
   Dir  : constant String := (if Ada.Command_Line.Argument_Count >= 1 then Ada.Command_Line.Argument (1) else ".");
   Eyes : Eye_Vectors.Vector;
   Q, M : Word_Vectors.Vector;

   procedure Write (Name : String; K : Keyboard) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (F, Ada.Text_IO.Out_File, Dir & "/" & Name & ".gbnf");
      Ada.Text_IO.Put (F, Grammar (K));
      Ada.Text_IO.Close (F);
   end Write;
begin
   Eyes.Append (1);
   Eyes.Append (2);
   Eyes.Append (3);
   Q.Append ("height");
   Q.Append ("distance");
   Q.Append ("angle");
   M.Append ("how high it is above the surface it rests on");
   M.Append ("how far it is from the other");
   M.Append ("how far it is turned");
   Write ("speech", Choose (Word_Vectors.Empty_Vector, Word_Vectors.Empty_Vector, [others => False], [others => True],
                            True, [others => True], Eyes));
   Write ("quantity", Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True], True,
                              [others => True], Eyes));
   Write ("full", Choose (Word_Vectors.Empty_Vector, Word_Vectors.Empty_Vector,
                          [Driver.Action.Grasper => False, others => True], [others => True], True, [others => True],
                          Eyes));
end Grammar_Dump;
