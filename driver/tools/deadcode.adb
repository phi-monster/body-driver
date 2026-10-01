--  deadcode [--check]
--
--  Subprograms of the driver that body_driver can never reach. The call
--  graph comes from the compiler's cross-reference (driver/obj/*.ali), not
--  from names. The other mains (self test, offline tools) and the self-test
--  packages do not make code live: they are not the driver. There is no list
--  of exemptions; code only a test calls is dead and goes, with the test.
--  Run it from the repository root after a build, so the cross-reference is
--  current. --check fails when anything is dead.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Ordered_Sets;
with Ada.Containers.Indefinite_Vectors;
with Ada.Directories;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;

procedure Deadcode is

   use Ada.Strings.Unbounded;
   use Ada.Text_IO;

   package String_Sets is new Ada.Containers.Indefinite_Ordered_Sets (String);
   package String_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   package Set_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String_Sets.Set, "<", String_Sets."=");

   Ours    : String_Sets.Set;   --  base names of the driver's sources
   Offline : String_Sets.Set;   --  base names whose references do not make code live

   function Image (N : Natural) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Both));

   procedure Collect (Root : String) is
      use Ada.Directories;
      Search : Search_Type;
      Item   : Directory_Entry_Type;
   begin
      Start_Search (Search, Root, "", [Ordinary_File => True, Ada.Directories.Directory => True, others => False]);
      while More_Entries (Search) loop
         Get_Next_Entry (Search, Item);
         declare
            Name : constant String := Simple_Name (Item);
         begin
            if Kind (Item) = Ada.Directories.Directory then
               if Name /= "." and then Name /= ".." then
                  Collect (Full_Name (Item));
               end if;
            elsif Extension (Name) = "ads" or else Extension (Name) = "adb" then
               Ours.Include (Name);
               if Ada.Strings.Fixed.Index (Name, "-tests.") > 0 then
                  Offline.Include (Name);
               end if;
            end if;
         end;
      end loop;
      End_Search (Search);
   end Collect;

   --  Every main of driver/driver.gpr except body_driver.adb is offline.
   procedure Read_Mains is
      F : File_Type;
      All_Text : Unbounded_String;
   begin
      Open (F, In_File, "driver/driver.gpr");
      while not End_Of_File (F) loop
         Append (All_Text, Get_Line (F) & " ");
      end loop;
      Close (F);
      declare
         T : constant String := To_String (All_Text);
         P : constant Natural := Ada.Strings.Fixed.Index (T, "for Main use");
         Q : Natural;
      begin
         if P = 0 then
            return;
         end if;
         Q := P;
         loop
            declare
               A : constant Natural := Ada.Strings.Fixed.Index (T (Q .. T'Last), """");
            begin
               exit when A = 0 or else Ada.Strings.Fixed.Index (T (P .. A), ";") > 0;
               declare
                  B : constant Natural := Ada.Strings.Fixed.Index (T (A + 1 .. T'Last), """");
               begin
                  exit when B = 0;
                  if T (A + 1 .. B - 1) /= "body_driver.adb" then
                     Offline.Include (T (A + 1 .. B - 1));
                  end if;
                  Q := B + 1;
               end;
            end;
         end loop;
      end;
   end Read_Mains;

   --  A subprogram declared in our sources, keyed "file:line:name".
   type Subprogram is record
      Body_File   : Unbounded_String;
      First, Last : Natural := 0;   --  body lines; Last = 0 when no body was seen
   end record;

   package Subprogram_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Subprogram);

   type Reference is record
      File   : Unbounded_String;
      Line   : Natural;
      Target : Unbounded_String;
   end record;

   package Reference_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Reference);

   Subs : Subprogram_Maps.Map;
   Refs : Reference_Vectors.Vector;

   --  One .ali file: its D lines number the files; X sections hold the
   --  entities of a file with their references.
   procedure Parse_Ali (Path : String) is
      F : File_Type;
      Files : String_Vectors.Vector;
      Section_File : Natural := 0;
      Have_Entity : Boolean := False;
      Entity_Key : Unbounded_String;
      Entity_Kind : Character := ' ';
      Ref_File : Natural := 0;
      Bodies, Ends : String_Vectors.Vector;   --  "file|line" of b and t references of the entity

      function File_Name (Index : Natural) return String is
        (if Index >= 1 and then Index <= Natural (Files.Length) then Files (Index) else "?");

      procedure Finish_Entity is
      begin
         if Have_Entity and then Entity_Kind in 'U' | 'V' | 'y' and then not Bodies.Is_Empty and then not Ends.Is_Empty
         then
            declare
               B : constant String := Bodies (1);
               E : constant String := Ends (Natural (Ends.Length));
               Bb : constant Natural := Ada.Strings.Fixed.Index (B, "|");
               Eb : constant Natural := Ada.Strings.Fixed.Index (E, "|");
            begin
               if B (B'First .. Bb - 1) = E (E'First .. Eb - 1) then
                  declare
                     S : Subprogram := (if Subs.Contains (To_String (Entity_Key)) then Subs (To_String (Entity_Key))
                                        else (others => <>));
                  begin
                     S := (Body_File => To_Unbounded_String (B (B'First .. Bb - 1)),
                           First => Natural'Value (B (Bb + 1 .. B'Last)),
                           Last => Natural'Value (E (Eb + 1 .. E'Last)));
                     Subs.Include (To_String (Entity_Key), S);
                  end;
               end if;
            end;
         elsif Have_Entity and then Entity_Kind in 'U' | 'V' | 'y'
           and then not Subs.Contains (To_String (Entity_Key))
         then
            Subs.Include (To_String (Entity_Key), (others => <>));
         end if;
         Bodies.Clear;
         Ends.Clear;
      end Finish_Entity;

      --  The references of an entity line, after its declaration part.
      procedure Read_References (Text : String) is
         Clean : String := Text;
         I : Natural;
      begin
         --  Blank out {...}, <...>, (...) and [...] annotations, each up to its
         --  first closer; an opener with no closer later (as in 115<53, a
         --  reference of type '<') is kept.
         declare
            K : Natural := Clean'First;
         begin
            while K <= Clean'Last loop
               if Clean (K) in '{' | '<' | '(' | '[' then
                  declare
                     Closer : constant Character :=
                       (case Clean (K) is when '{' => '}', when '<' => '>', when '(' => ')', when others => ']');
                     E : constant Natural := Ada.Strings.Fixed.Index (Clean (K + 1 .. Clean'Last), "" & Closer);
                  begin
                     if E > 0 then
                        Clean (K .. E) := [others => ' '];
                        K := E + 1;
                     else
                        K := K + 1;
                     end if;
                  end;
               else
                  K := K + 1;
               end if;
            end loop;
         end;
         I := Clean'First;
         while I <= Clean'Last loop
            if Clean (I) in '0' .. '9' then
               declare
                  J : Natural := I;
                  A, B : Natural := 0;
                  Ref_Type : Character;
               begin
                  while J <= Clean'Last and then Clean (J) in '0' .. '9' loop
                     A := A * 10 + Character'Pos (Clean (J)) - Character'Pos ('0');
                     J := J + 1;
                  end loop;
                  if J <= Clean'Last and then Clean (J) = '|' then
                     Ref_File := A;
                     J := J + 1;
                     A := 0;
                     while J <= Clean'Last and then Clean (J) in '0' .. '9' loop
                        A := A * 10 + Character'Pos (Clean (J)) - Character'Pos ('0');
                        J := J + 1;
                     end loop;
                  end if;
                  if J <= Clean'Last and then Clean (J) in 'a' .. 'z' | 'A' .. 'Z' | '<' | '>' | '=' | '^' | '*' then
                     Ref_Type := Clean (J);
                     J := J + 1;
                     while J <= Clean'Last and then Clean (J) in '0' .. '9' loop
                        B := B * 10 + Character'Pos (Clean (J)) - Character'Pos ('0');
                        J := J + 1;
                     end loop;
                     if Have_Entity then
                        if Ref_Type = 'b' then
                           Bodies.Append (File_Name (Ref_File) & "|" & Image (A));
                        elsif Ref_Type = 't' then
                           Ends.Append (File_Name (Ref_File) & "|" & Image (A));
                        elsif Ref_Type in 'r' | 's' | 'R' | 'm' | 'i' and then Entity_Kind in 'U' | 'V' | 'y' then
                           Refs.Append (Reference'(File => To_Unbounded_String (File_Name (Ref_File)), Line => A,
                                         Target => Entity_Key));
                        end if;
                     end if;
                     pragma Unreferenced (B);
                  end if;
                  I := J;
               end;
            else
               I := I + 1;
            end if;
         end loop;
      end Read_References;
   begin
      Open (F, In_File, Path);
      while not End_Of_File (F) loop
         declare
            L : constant String := Get_Line (F);
         begin
            if L'Length >= 2 and then L (L'First .. L'First + 1) = "D " then
               --  D name, then a tab-separated time stamp and checksum
               declare
                  Rest : constant String := L (L'First + 2 .. L'Last);
                  Last : Natural := Rest'First - 1;
               begin
                  while Last < Rest'Last and then Rest (Last + 1) not in ' ' | ASCII.HT loop
                     Last := Last + 1;
                  end loop;
                  Files.Append (Rest (Rest'First .. Last));
               end;
            elsif L'Length >= 2 and then L (L'First .. L'First + 1) = "X " then
               Finish_Entity;
               Have_Entity := False;
               declare
                  Rest : constant String := L (L'First + 2 .. L'Last);
                  Sp : constant Natural := Ada.Strings.Fixed.Index (Rest, " ");
               begin
                  Section_File := Natural'Value (if Sp = 0 then Rest else Rest (Rest'First .. Sp - 1));
               end;
            elsif Section_File > 0 and then L'Length > 0 and then L (L'First) in '0' .. '9' then
               Finish_Entity;
               --  line kind col [* ] name ...
               declare
                  P : Natural := L'First;
                  Line_No : Natural := 0;
               begin
                  while P <= L'Last and then L (P) in '0' .. '9' loop
                     Line_No := Line_No * 10 + Character'Pos (L (P)) - Character'Pos ('0');
                     P := P + 1;
                  end loop;
                  Have_Entity := False;
                  if P <= L'Last then
                     Entity_Kind := L (P);
                     P := P + 1;
                     while P <= L'Last and then L (P) in '0' .. '9' loop
                        P := P + 1;
                     end loop;
                     if P <= L'Last and then L (P) in '*' | ' ' then
                        P := P + 1;
                     end if;
                     declare
                        Q : Natural := P;
                     begin
                        while Q <= L'Last and then L (Q) not in '{' | '<' | '(' | '=' | '[' | ' ' loop
                           Q := Q + 1;
                        end loop;
                        if Ours.Contains (File_Name (Section_File)) then
                           Entity_Key := To_Unbounded_String (File_Name (Section_File) & ":" & Image (Line_No) & ":"
                                                              & L (P .. Q - 1));
                           Have_Entity := True;
                           Ref_File := Section_File;
                           if Q <= L'Last then
                              Read_References (L (Q .. L'Last));
                           end if;
                        end if;
                     end;
                  end if;
               end;
            elsif Section_File > 0 and then L'Length > 0 and then L (L'First) = '.' then
               if Have_Entity then
                  Read_References (L (L'First + 1 .. L'Last));
               end if;
            elsif L'Length > 0 and then L (L'First) in 'A' .. 'Z' then
               Finish_Entity;
               Have_Entity := False;
               Section_File := 0;
            end if;
         end;
      end loop;
      Finish_Entity;
      Close (F);
   end Parse_Ali;

   procedure Parse_All is
      use Ada.Directories;
      Search : Search_Type;
      Item   : Directory_Entry_Type;
   begin
      Start_Search (Search, "driver/obj", "*.ali", [Ordinary_File => True, others => False]);
      while More_Entries (Search) loop
         Get_Next_Entry (Search, Item);
         Parse_Ali (Full_Name (Item));
      end loop;
      End_Search (Search);
   end Parse_All;

   Edges : Set_Maps.Map;

   function Owner (File : String; Line : Natural) return String is
      Best : Unbounded_String;
      Best_Span : Natural := Natural'Last;
   begin
      for C in Subs.Iterate loop
         declare
            S : constant Subprogram := Subprogram_Maps.Element (C);
         begin
            if S.Last > 0 and then To_String (S.Body_File) = File and then S.First <= Line and then Line <= S.Last
              and then S.Last - S.First < Best_Span
            then
               Best_Span := S.Last - S.First;
               Best := To_Unbounded_String (Subprogram_Maps.Key (C));
            end if;
         end;
      end loop;
      return (if Length (Best) = 0 then "<elaboration>:" & File else To_String (Best));
   end Owner;

   Live : String_Sets.Set;

begin
   Collect ("driver/src");
   Read_Mains;
   Parse_All;
   for R of Refs loop
      if not Offline.Contains (To_String (R.File)) then
         declare
            O : constant String := Owner (To_String (R.File), R.Line);
            S : String_Sets.Set;
         begin
            if Edges.Contains (O) then
               S := Edges (O);
            end if;
            S.Include (To_String (R.Target));
            Edges.Include (O, S);
         end;
      end if;
   end loop;
   declare
      Stack : String_Vectors.Vector;
   begin
      for C in Subs.Iterate loop
         declare
            K : constant String := Subprogram_Maps.Key (C);
         begin
            if K'Length > 16 and then K (K'First .. K'First + 15) = "body_driver.adb:"
              and then Ada.Strings.Fixed.Index (K, ":Body_Driver") > 0
            then
               Stack.Append (K);
            end if;
         end;
      end loop;
      for C in Edges.Iterate loop
         declare
            K : constant String := Set_Maps.Key (C);
         begin
            if K'Length > 14 and then K (K'First .. K'First + 13) = "<elaboration>:"
              and then not Offline.Contains (K (K'First + 14 .. K'Last))
            then
               Stack.Append (K);
            end if;
         end;
      end loop;
      while not Stack.Is_Empty loop
         declare
            K : constant String := Stack.Last_Element;
         begin
            Stack.Delete_Last;
            if not Live.Contains (K) then
               Live.Include (K);
               if Edges.Contains (K) then
                  for T of Edges (K) loop
                     Stack.Append (T);
                  end loop;
               end if;
            end if;
         end;
      end loop;
   end;
   --  --why NAME: which live owners reference a subprogram of that name.
   if Ada.Command_Line.Argument_Count >= 2 and then Ada.Command_Line.Argument (1) = "--why" then
      for C in Edges.Iterate loop
         for T of Set_Maps.Element (C) loop
            if Ada.Strings.Fixed.Index (T, ":" & Ada.Command_Line.Argument (2)) = T'Last - Ada.Command_Line.Argument (2)'Length
            then
               Put_Line (Set_Maps.Key (C) & (if Live.Contains (Set_Maps.Key (C)) then " (live)" else "") & " -> " & T);
            end if;
         end loop;
      end loop;
      return;
   end if;
   declare
      Dead_Count, Dead_Lines : Natural := 0;
      Report : String_Vectors.Vector;
   begin
      for C in Subs.Iterate loop
         declare
            K : constant String := Subprogram_Maps.Key (C);
            S : constant Subprogram := Subprogram_Maps.Element (C);
            Nested : Boolean := False;
         begin
            if S.Last > 0 and then not Live.Contains (K) and then not Offline.Contains (To_String (S.Body_File)) then
               --  Only the outermost dead body is listed.
               for D in Subs.Iterate loop
                  declare
                     O : constant Subprogram := Subprogram_Maps.Element (D);
                  begin
                     if Subprogram_Maps.Key (D) /= K and then O.Last > 0 and then not Live.Contains (Subprogram_Maps.Key (D))
                       and then O.Body_File = S.Body_File and then O.First <= S.First and then S.Last <= O.Last
                       and then (O.First /= S.First or else O.Last /= S.Last)
                     then
                        Nested := True;
                     end if;
                  end;
               end loop;
               if not Nested then
                  Dead_Count := Dead_Count + 1;
                  Dead_Lines := Dead_Lines + S.Last - S.First + 1;
                  Report.Append ("  " & To_String (S.Body_File) & ":" & Image (S.First) & "-" & Image (S.Last) & "  "
                                 & K (Ada.Strings.Fixed.Index (K, ":", Ada.Strings.Backward) + 1 .. K'Last));
               end if;
            end if;
         end;
      end loop;
      Put_Line ("== subprograms body_driver cannot reach:" & Dead_Count'Image & "," & Dead_Lines'Image & " lines ==");
      for L of Report loop
         Put_Line (L);
      end loop;
      if Ada.Command_Line.Argument_Count > 0 and then Ada.Command_Line.Argument (1) = "--check" and then Dead_Count > 0
      then
         Put_Line ("FAIL: unreachable code in the driver; delete it together with the tests written only for it");
         Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      end if;
   end;
end Deadcode;
