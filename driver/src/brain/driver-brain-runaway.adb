with Ada.Characters.Handling;
with Ada.Containers.Indefinite_Vectors;
with Ada.Strings.Fixed;
with Driver.Brain.Words;

package body Driver.Brain.Runaway is

   use Driver.Brain.Words;

   package Text_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

   function Lines_Of (Text : String) return Text_Vectors.Vector is
      R     : Text_Vectors.Vector;
      First : Positive := Text'First;
   begin
      for I in Text'Range loop
         if Text (I) = ASCII.LF then
            R.Append (Text (First .. I - 1));
            First := I + 1;
         end if;
      end loop;
      if First <= Text'Last then
         R.Append (Text (First .. Text'Last));
      end if;
      return R;
   end Lines_Of;

   --  The words of a line; the last one counts only when the line is finished
   --  or a blank follows it.
   function Words_Of (Line : String; Finished : Boolean) return Text_Vectors.Vector is
      R : Text_Vectors.Vector;
      I : Natural := Line'First;
   begin
      while I <= Line'Last loop
         if Line (I) = ' ' then
            I := I + 1;
         else
            declare
               J : Natural := I;
            begin
               while J <= Line'Last and then Line (J) /= ' ' loop
                  J := J + 1;
               end loop;
               exit when J > Line'Last and then not Finished;
               R.Append (Line (I .. J - 1));
               I := J;
            end;
         end if;
      end loop;
      return R;
   end Words_Of;

   function Lower (S : String) return String renames Ada.Characters.Handling.To_Lower;

   function Same_Run (W : Text_Vectors.Vector; Last, Length : Positive) return Boolean is
     (for all M in 0 .. Length - 1 => W (Last - M) = W (Last - Length - M));
   --  Whether the Length items ending at Last equal the Length items before them.

   --  A run of whole words that comes again at once inside one name or one
   --  sentence of the line; "" if none. A name is a run of words that are
   --  not language words; after say, the whole sentence is one run.
   function Repeated_Words
     (Line : String; Finished : Boolean; Name_Words : Driver.Brain.Keyboard.Word_Vectors.Vector) return String
   is
      W : constant Text_Vectors.Vector := Words_Of (Line, Finished);
   begin
      if W.Is_Empty then
         return "";
      end if;
      declare
         Sentence   : constant Boolean := Lower (W.First_Element) = Say_Word;
         Slot_First : Positive := W.First_Index + 1;   --  the line's first word is never in a slot
      begin
         for N in W.First_Index + 1 .. W.Last_Index loop
            if not Sentence and then Name_Words.Contains (Lower (W (N))) then
               Slot_First := N + 1;
            else
               for Length in 1 .. (N - Slot_First + 1) / 2 loop
                  if Same_Run (W, N, Length) then
                     declare
                        Run : Unbounded_String;
                     begin
                        for M in N - Length + 1 .. N loop
                           Append (Run, (if M > N - Length + 1 then " " else "") & W (M));
                        end loop;
                        return To_String (Run);
                     end;
                  end if;
               end loop;
            end if;
         end loop;
      end;
      return "";
   end Repeated_Words;

   function Opens_Block (Line : String) return Boolean is
      T : constant String := Ada.Strings.Fixed.Trim (Lower (Line), Ada.Strings.Both);
      W : constant Text_Vectors.Vector := Words_Of (T, Finished => True);
   begin
      return T'Length > 0 and then T (T'Last) = ':' and then not W.Is_Empty
        and then (W.First_Element = Repeat_Word or else W.First_Element = If_Word
                  or else W.First_Element = Try_Word or else W.First_Element = Try_Word & ":"
                  or else W.First_Element = To_Word);
   end Opens_Block;

   function Judge
     (Text       : String;
      Name_Words : Driver.Brain.Keyboard.Word_Vectors.Vector;
      Final      : Boolean) return Verdict
   is
      Lines    : constant Text_Vectors.Vector := Lines_Of (Text);
      Finished : constant Natural :=
        (if Final or else Lines.Is_Empty or else Text (Text'Last) = ASCII.LF
         then Natural (Lines.Length) else Natural (Lines.Length) - 1);
      Depth    : Natural := 0;
      V        : Verdict;
   begin
      for L in Lines.First_Index .. Lines.Last_Index loop
         declare
            Line : constant String := Lines (L);
            Done_With_Line : constant Boolean := L <= Finished;
            Run  : constant String := Repeated_Words (Line, Done_With_Line, Name_Words);
         begin
            if Run'Length > 0 then
               V.Fired := True;
               V.Keep := L - 1;
               V.Why := To_Unbounded_String ("line" & L'Image & ": """ & Run & """ came again at once");
               return V;
            end if;
            exit when not Done_With_Line;
            for Length in 1 .. L / 2 loop
               if Same_Run (Lines, L, Length) then
                  V.Fired := True;
                  V.Keep := L - Length;
                  V.Why := To_Unbounded_String
                    ("from line" & Natural'Image (L - Length + 1) & " the" & Length'Image
                     & " line(s) before came again at once");
                  return V;
               end if;
            end loop;
            declare
               T : constant String := Ada.Strings.Fixed.Trim (Lower (Line), Ada.Strings.Both);
            begin
               if T = Done_Word and then Depth = 0 then
                  V.Complete := True;
                  V.Keep := L;
                  V.Why := To_Unbounded_String ("line" & L'Image & " is done outside every block");
                  return V;
               elsif T = End_Word then
                  Depth := (if Depth > 0 then Depth - 1 else 0);
               elsif Opens_Block (Line) then
                  Depth := Depth + 1;
               end if;
            end;
         end;
      end loop;
      V.Keep := Finished;
      return V;
   end Judge;

   function First_Lines (Text : String; Count : Natural) return String is
      Lines : constant Text_Vectors.Vector := Lines_Of (Text);
      R     : Unbounded_String;
   begin
      for L in Lines.First_Index .. Natural'Min (Count, Lines.Last_Index) loop
         Append (R, Lines (L) & ASCII.LF);
      end loop;
      return To_String (R);
   end First_Lines;

end Driver.Brain.Runaway;
