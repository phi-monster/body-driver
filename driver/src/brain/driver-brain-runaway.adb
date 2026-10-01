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
   --  or a blank follows it, unless Partial asks for it as well.
   function Words_Of (Line : String; Finished : Boolean; Partial : Boolean := False) return Text_Vectors.Vector is
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
               exit when J > Line'Last and then not Finished and then not Partial;
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

   --  The units of a word in a name slot: every language word glued into it,
   --  taken longest first from the left, and every run of other letters
   --  between them. Glue is the only way the decoder adds letters to a name
   --  (LANGUAGE.md 17.7), so a name word that never ends is language words
   --  glued over and over. Growing: the word may still go on, so only the
   --  units no later letter can change are given, those that end at least
   --  a longest language word before its last letter.
   function Units_Of
     (Word : String; Name_Words : Driver.Brain.Keyboard.Word_Vectors.Vector; Growing : Boolean)
      return Text_Vectors.Vector
   is
      W       : constant String := Lower (Word);
      Longest : Natural := 0;
      R       : Text_Vectors.Vector;
      Other   : Natural := W'First;   --  where the run of other letters being read began
      I       : Natural := W'First;
      Stable  : Integer;
   begin
      for N of Name_Words loop
         Longest := Natural'Max (Longest, N'Length);
      end loop;
      Stable := (if Growing then W'Last - Longest + 1 else W'Last);
      while I <= W'Last loop
         declare
            Match : Natural := 0;
         begin
            for N of Name_Words loop
               if N'Length > Match and then I + N'Length - 1 <= W'Last and then W (I .. I + N'Length - 1) = N then
                  Match := N'Length;
               end if;
            end loop;
            if Match > 0 then
               if Other < I then
                  exit when I - 1 > Stable;
                  R.Append (W (Other .. I - 1));
               end if;
               exit when I + Match - 1 > Stable;
               R.Append (W (I .. I + Match - 1));
               I := I + Match;
               Other := I;
            else
               I := I + 1;
            end if;
         end;
      end loop;
      if I > W'Last and then Other <= W'Last and then not Growing then
         R.Append (W (Other .. W'Last));
      end if;
      return R;
   end Units_Of;

   function Joined (U : Text_Vectors.Vector; Last, Length : Positive) return String is
      R : Unbounded_String;
   begin
      for M in Last - Length + 1 .. Last loop
         Append (R, (if M > Last - Length + 1 then " " else "") & U (M));
      end loop;
      return To_String (R);
   end Joined;

   --  A run that comes again at once inside one name or one sentence of the
   --  line; "" if none. After say, the sentence's words are the run's items;
   --  elsewhere a name is what lies between language words, and its items
   --  are the units of its words, the word still being written included.
   function Repeated_Words
     (Line : String; Finished : Boolean; Name_Words : Driver.Brain.Keyboard.Word_Vectors.Vector) return String
   is
      W : constant Text_Vectors.Vector := Words_Of (Line, Finished, Partial => True);
      Last_Growing : constant Boolean :=
        not Finished and then Line'Length > 0 and then Line (Line'Last) /= ' ';
   begin
      if W.Is_Empty then
         return "";
      end if;
      if Lower (W.First_Element) = Say_Word then
         declare
            S : constant Text_Vectors.Vector := Words_Of (Line, Finished);
         begin
            for N in S.First_Index + 1 .. S.Last_Index loop
               for Length in 1 .. (N - S.First_Index) / 2 loop
                  if Same_Run (S, N, Length) then
                     return Joined (S, N, Length);
                  end if;
               end loop;
            end loop;
         end;
         return "";
      end if;
      declare
         Slot : Text_Vectors.Vector;   --  the units of the name being read
      begin
         for N in W.First_Index + 1 .. W.Last_Index loop
            declare
               Growing : constant Boolean := N = W.Last_Index and then Last_Growing;
            begin
               if not Growing and then Name_Words.Contains (Lower (W (N))) then
                  Slot.Clear;
               else
                  for U of Units_Of (W (N), Name_Words, Growing) loop
                     Slot.Append (U);
                     for Length in 1 .. Natural (Slot.Length) / 2 loop
                        if Same_Run (Slot, Slot.Last_Index, Length) then
                           return Joined (Slot, Slot.Last_Index, Length);
                        end if;
                     end loop;
                  end loop;
               end if;
            end;
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
