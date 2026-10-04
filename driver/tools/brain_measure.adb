--  brain_measure: measurements of the brain layer against a live brain
--  service, through the driver's own round, keyboard and client, so every
--  request is the one the driver sends.
--
--    brain_measure keyboard HOST:PORT QUESTIONS.json REPEATS OUT.jsonl
--       Asks every question REPEATS times on every keyboard it names, through
--       the driver's client, and writes one line per answer: how reading
--       ended, the kept program, how many stretches it asks for, and whether
--       its first stretch says what the task asks. REPEATS 0 prints each
--       prompt instead of asking.
--    brain_measure truncation HOST:PORT QUESTIONS.json REPEATS OUT.jsonl
--       The same questions, each answer read to its end (the service's own
--       end, or its token limit), with the driver's verdict evaluated on
--       every event: where the driver would have stopped reading and what it
--       would have kept, against where and how the answer really ended.
--    brain_measure stream HOST:PORT QUESTIONS.json INDEX KEYBOARD LIMIT
--       One answer printed as it streams, with the driver's verdict.
--    brain_measure where HOST:PORT IMAGE.ppm NAME...
--       Asks where each NAME is in one picture, as the binder does.
--    brain_measure where-request IMAGE.ppm NAME
--       Prints the where-is-it request the driver would send.
--    brain_measure bindings HOST:PORT SCENES.json QUESTIONS.json ANSWERS.jsonl REPEATS OUT.jsonl
--       Binds every name the answers of a keyboard run wrote, as a first round
--       does, and scores it against the thing its question meant.
--    brain_measure names HOST:PORT REPLAY.json REPEATS OUT.jsonl
--       Binds the names of recorded programs with the driver's binder, the
--       eyes asked live, and scores each binding against what the name meant.
--    brain_measure where-all HOST:PORT QUESTIONS.json ANSWERS.jsonl OUT.jsonl
--    brain_measure patches HOST:PORT QUESTIONS.json WHERE.jsonl OUT.jsonl
--    brain_measure bind-score QUESTIONS.json ANSWERS.jsonl WHERE.jsonl PATCHES.jsonl MEANT.json OUT.jsonl
--       Binding on scenes with truth (driver/tools/brain_scene), in three
--       steps: every eye asked where every name of a keyboard run's programs
--       is; every box segmented by the instrument (HOST:PORT) as the live
--       Identify asks it; the binder replayed on those answers and patches,
--       each patch taken as the object most of its pixels show, and scored.
--
--  QUESTIONS.json is an array of objects:
--    id, task, kind (lift | turn | push | next_to | on), thing, other (the
--    second thing's key word, or ""), eyes (PPM files, in camera order),
--    mounts ("fixed" or "arm <n>", one per eye), keyboards (names below),
--    and optionally view (the eye whose picture is the large one, 1 when
--    absent) and happened (what the round says happened before; a first
--    round's words when absent). Sampling comes from BL_BRAIN_SAMPLING, as
--    in the driver.
--
--  Keyboards: Q (height), QH (height, heading), and the same with the
--  sentence about two things added: Q2 and QH2 (touching, above, below,
--  left, right), Q2d and QH2d (touching, above), Q2o and QH2o (touching,
--  onto); a name ending in s writes that sentence as one key per relation.
--  Every quantity keyboard allows one stretch per program.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Directories;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Action;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Brain.Parser;
with Driver.Brain.Pictures;
with Driver.Brain.Programs;
with Driver.Brain.Round;
with Driver.Brain.Runaway;
with Driver.Brain.Service;
with Driver.Brain.Wants;
with Driver.Bytes;
with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Json;
with Driver.Log;
with Driver.Observations;
with Driver.Robot;
with Driver.Services;

procedure Brain_Measure is

   use Ada.Strings.Unbounded;
   use Ada.Command_Line;
   use type Driver.Brain.Names.Pointing;
   use type Driver.Brain.Names.Binding_Kind;
   use type Driver.Brain.Names.Patch;
   use type Driver.Brain.Programs.Statement_Kind;
   use type Driver.Brain.Programs.Constraint_Kind;
   use type Driver.Action.Relation;


   function Slurp (Path : String) return String is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      S : String (1 .. Natural (Ada.Directories.Size (Path)));
   begin
      Open (F, In_File, Path);
      String'Read (Stream (F), S);
      Close (F);
      return S;
   end Slurp;

   --  A binary PPM (P6, 8 bits): "P6", width, height, maximum, one blank, pixels.
   function Read_Ppm (Path : String) return Driver.Images.Image is
      S : constant String := Slurp (Path);
      I : Positive := S'First;

      function Field return Natural is
         First : Positive;
      begin
         while S (I) in ' ' | ASCII.LF | ASCII.CR | ASCII.HT loop
            I := I + 1;
         end loop;
         First := I;
         while S (I) in '0' .. '9' loop
            I := I + 1;
         end loop;
         return Natural'Value (S (First .. I - 1));
      end Field;
   begin
      if S (I .. I + 1) /= "P6" then
         raise Constraint_Error with Path & " is not a binary PPM";
      end if;
      I := I + 2;
      declare
         W : constant Positive := Field;
         H : constant Positive := Field;
         M : constant Natural := Field;
         pragma Unreferenced (M);
         Data : constant Driver.Bytes.Byte_Array := Driver.Bytes.To_Bytes (S (I + 1 .. I + 3 * W * H));
      begin
         return Driver.Images.Create (W, H, Data);
      end;
   end Read_Ppm;

   function Keys (Name : String; Eye_Count : Natural) return Driver.Brain.Keyboard.Keyboard is
      Q, M : Driver.Brain.Keyboard.Word_Vectors.Vector;
      Two  : Driver.Brain.Keyboard.Relation_Set := [others => False];
      Eyes : Driver.Brain.Keyboard.Eye_Vectors.Vector;
   begin
      Q.Append ("height");
      M.Append (Driver.Action.Meaning ("height"));
      if Ada.Strings.Fixed.Index (Name, "QH") = Name'First then
         Q.Append ("heading");
         --  A name with g in it keeps an earlier wording of heading, to
         --  compare the action layer's with it.
         M.Append (if Ada.Strings.Fixed.Index (Name, "g") > 0
                   then "which way it points on the surface it rests on; up and down turn it one way or the other"
                   else Driver.Action.Meaning ("heading"));
      end if;
      for E in 1 .. Eye_Count loop
         Eyes.Append (Driver.Observations.Camera_Id (E));
      end loop;
      if Ada.Strings.Fixed.Index (Name, "2d") > 0 then
         Two := [Driver.Action.Touching | Driver.Action.Above => True, others => False];
      elsif Ada.Strings.Fixed.Index (Name, "2o") > 0 then
         Two := [Driver.Action.Touching | Driver.Action.Onto => True, others => False];
      elsif Ada.Strings.Fixed.Index (Name, "2") > 0 then
         Two := [Driver.Action.Touching | Driver.Action.Above | Driver.Action.Below | Driver.Action.Left
                 | Driver.Action.Right => True, others => False];
      end if;
      return Driver.Brain.Keyboard.Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True],
                                           True, Two, Eyes);
   end Keys;

   --  The sheet as the brain reads it. A keyboard whose name ends in s
   --  writes the sentence about two things as one key per relation, each with
   --  its meaning on its own line: the same grammar, another sheet.
   function Sheet_Of (K : Driver.Brain.Keyboard.Keyboard; Name : String) return String is
      S     : constant String := Driver.Brain.Keyboard.Sheet (K);
      R     : Unbounded_String;
      First : Positive := S'First;
      Skip  : Boolean := False;   --  inside the relation slot's glosses

      type Key is record
         Word, Meaning : Unbounded_String;
      end record;

      function "+" (W : String) return Unbounded_String renames To_Unbounded_String;

      Per_Relation : constant array (Positive range <>) of Key :=
        [(+"touching", +"the first thing ends up against the second"),
         (+"above", +"the first thing ends up above the second"),
         (+"below", +"the first thing ends up below the second"),
         (+"left", +"the first thing ends up to the left of the second, as the large picture shows them"),
         (+"right", +"the first thing ends up to the right of the second, as the large picture shows them")];

      function Starts (Line, Head : String) return Boolean is
        (Line'Length >= Head'Length and then Line (Line'First .. Line'First + Head'Length - 1) = Head);
   begin
      if Name (Name'Last) /= 's' then
         return S;
      end if;
      for I in S'Range loop
         if S (I) = ASCII.LF then
            declare
               Line : constant String := S (First .. I - 1);
            begin
               if Skip and then Starts (Line, "      ") then
                  null;
               else
                  Skip := False;
                  if Starts (Line, "<line>") then
                     declare
                        Keys_Line : Unbounded_String := To_Unbounded_String (Line);
                        At_P      : constant Natural := Ada.Strings.Fixed.Index (Line, "<placing>");
                        Listed    : Unbounded_String;
                     begin
                        for K of Per_Relation loop
                           Append (Listed, (if Length (Listed) > 0 then " | " else "") & "<" & K.Word & ">");
                        end loop;
                        if At_P > 0 then
                           Replace_Slice (Keys_Line, At_P - Line'First + 1, At_P - Line'First + 9, To_String (Listed));
                        end if;
                        Append (R, Keys_Line & ASCII.LF);
                     end;
                  elsif Starts (Line, "<placing>") then
                     for K of Per_Relation loop
                        Append (R, "<" & K.Word & "> ::= do <thing> " & K.Word & " <thing> until <ending>   ("
                                & K.Meaning & ")" & ASCII.LF);
                     end loop;
                  elsif Starts (Line, "<relation>") then
                     Skip := True;
                  else
                     Append (R, Line & ASCII.LF);
                  end if;
               end if;
            end;
            First := I + 1;
         end if;
      end loop;
      return To_String (R);
   end Sheet_Of;

   procedure Configure (Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      Driver.Services.Configure (Driver.Services.Brain, Address (Address'First .. Colon - 1),
                                 Natural'Value (Address (Colon + 1 .. Address'Last)));
   end Configure;

   function Has_Letters (Name, Key : String) return Boolean is
     (Key'Length > 0 and then Ada.Strings.Fixed.Index (Driver.Brain.Names.Letters (Name),
                                                       Driver.Brain.Names.Letters (Key)) > 0);

   --  The stretches a program asks for, as written.
   function Stretches (Program : String) return Natural is
      P   : Driver.Brain.Programs.Program;
      Ok  : Boolean;
      Why : Driver.Brain.Programs.Refusal;
      N   : Natural := 0;
   begin
      Driver.Brain.Parser.Parse (Program, P, Ok, Why);
      if Ok then
         for S of P.Statements loop
            if S.Kind = Driver.Brain.Programs.Interval then
               N := N + 1;
            end if;
         end loop;
      end if;
      return N;
   end Stretches;

   --  Whether the first stretch of the program says what the task asks.
   function Right (Kind, Thing, Other, Program : String) return Boolean is
      P   : Driver.Brain.Programs.Program;
      Ok  : Boolean;
      Why : Driver.Brain.Programs.Refusal;
   begin
      Driver.Brain.Parser.Parse (Program, P, Ok, Why);
      if not Ok then
         return False;
      end if;
      for S of P.Statements loop
         if S.Kind = Driver.Brain.Programs.Interval then
            declare
               C : constant Driver.Brain.Programs.Constraint := S.Constraints.First_Element;
               Subject : constant String := To_String (C.Subject.Name);
               Object  : constant String := To_String (C.Object.Name);
            begin
               if Kind = "lift" then
                  return C.Kind = Driver.Brain.Programs.Quantity_Constraint and then To_String (C.Quantity) = "height"
                    and then C.Increase and then Has_Letters (Subject, Thing);
               elsif Kind = "turn" then
                  return C.Kind = Driver.Brain.Programs.Quantity_Constraint and then To_String (C.Quantity) = "heading"
                    and then Has_Letters (Subject, Thing);
               elsif Kind = "push" then
                  return C.Kind = Driver.Brain.Programs.Relation_Constraint and then Has_Letters (Subject, Thing)
                    and then C.Relation in Driver.Action.Left | Driver.Action.Right;
               elsif Kind = "next_to" then
                  return C.Kind = Driver.Brain.Programs.Relation_Constraint and then Has_Letters (Subject, Thing)
                    and then Has_Letters (Object, Other)
                    and then C.Relation in Driver.Action.Touching | Driver.Action.Nearer | Driver.Action.Left
                                           | Driver.Action.Right;
               elsif Kind = "on" then
                  return C.Kind = Driver.Brain.Programs.Relation_Constraint and then Has_Letters (Subject, Thing)
                    and then Has_Letters (Object, Other)
                    and then C.Relation in Driver.Action.Above | Driver.Action.Onto | Driver.Action.Touching;
               end if;
               return False;
            end;
         end if;
      end loop;
      return False;
   end Right;

   --  One question on one keyboard: the prompt and the picture as the round
   --  composes them.
   type Setting is record
      Keys    : Driver.Brain.Keyboard.Keyboard;
      Prompt  : Unbounded_String;
      Picture : Driver.Images.Image;
   end record;

   --  The eye a question looks through: its "view", else the first.
   function View_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Driver.Observations.Camera_Id is
     (if Driver.Json."/=" (Driver.Json.Lookup (Doc, N, "view"), Driver.Json.No_Node)
      then Driver.Observations.Camera_Id (Natural (Driver.Json.Number (Doc, Driver.Json.Lookup (Doc, N, "view"))))
      else 1);

   function Setting_Of (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String) return Setting is
      Eyes     : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "eyes");
      Mounts   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "mounts");
      Happened : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "happened");
      View     : constant Driver.Observations.Camera_Id := View_Of (Doc, N);
      Images   : Driver.Observations.Image_Vectors.Vector;
      Facts    : Driver.Brain.Round.Facts;
      K        : constant Driver.Brain.Keyboard.Keyboard := Keys (Board, Driver.Json.Count (Doc, Eyes));
   begin
      for E in 1 .. Driver.Json.Count (Doc, Eyes) loop
         Images.Append (Read_Ppm (Driver.Json.Text (Doc, Driver.Json.Element (Doc, Eyes, E))));
         declare
            M : constant String := Driver.Json.Text (Doc, Driver.Json.Element (Doc, Mounts, E));
         begin
            Facts.Eyes.Append
              (Driver.Brain.Round.Eye_Facts'
                 (Eye   => Driver.Observations.Camera_Id (E),
                  Mount => (if M = "fixed" then (Kind => Driver.Robot.World_Fixed)
                            else (Kind => Driver.Robot.Arm_Carried,
                                  Arm  => Driver.Robot.Arm_Id'Value (M (M'First + 4 .. M'Last))))));
         end;
         --  The other eyes below the view, in camera order, as a round shows them.
         if Driver.Observations."/=" (Driver.Observations.Camera_Id (E), View) then
            Facts.Strip.Append (Driver.Observations.Camera_Id (E));
         end if;
      end loop;
      Facts.View := View;
      Facts.Happened := To_Unbounded_String
        (if Driver.Json."/=" (Happened, Driver.Json.No_Node) then Driver.Json.Text (Doc, Happened)
         else Driver.Brain.Round.First_Round);
      Facts.Instruction := To_Unbounded_String (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "task")));
      Facts.Sheet := To_Unbounded_String (Sheet_Of (K, Board));
      declare
         function Image_Of (E : Driver.Brain.Names.Eye_Id) return Driver.Images.Image is (Images (E));
      begin
         return (Keys    => K,
                 Prompt  => To_Unbounded_String (Driver.Brain.Round.Prompt (Facts)),
                 Picture => Driver.Brain.Pictures.Compose (Images (View), Facts.Strip, Image_Of'Access));
      end;
   end Setting_Of;

   --  Interleaved: each repeat asks every question once on every keyboard it
   --  names, the keyboards one after another in turn and in the other order
   --  on the next repeat, so keyboards compared with each other share the
   --  service's state and its drift over the hours a series takes.
   procedure Each_Answer
     (Path    : String;
      Repeats : Natural;
      Visit   : not null access procedure
        (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive; S : Setting))
   is
      Doc : Driver.Json.Document;
      Ok  : Boolean;
      Why : Unbounded_String;

      type Asked is record
         N     : Driver.Json.Node;
         Board : Unbounded_String;
         S     : Setting;
      end record;

      package Asked_Vectors is new Ada.Containers.Vectors (Positive, Asked);

      All_Asked : Asked_Vectors.Vector;
   begin
      Driver.Json.Parse (Slurp (Path), Doc, Ok, Why);
      if not Ok then
         Ada.Text_IO.Put_Line ("questions: " & To_String (Why));
         return;
      end if;
      for Q in 1 .. Driver.Json.Count (Doc, Driver.Json.Root (Doc)) loop
         declare
            N      : constant Driver.Json.Node := Driver.Json.Element (Doc, Driver.Json.Root (Doc), Q);
            Boards : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "keyboards");
         begin
            for B in 1 .. Driver.Json.Count (Doc, Boards) loop
               declare
                  Board : constant String := Driver.Json.Text (Doc, Driver.Json.Element (Doc, Boards, B));
                  S     : constant Setting := Setting_Of (Doc, N, Board);
               begin
                  if Repeats = 0 then
                     Ada.Text_IO.Put_Line (To_String (S.Prompt));
                  end if;
                  All_Asked.Append (Asked'(N => N, Board => To_Unbounded_String (Board), S => S));
               end;
            end loop;
         end;
      end loop;
      for Rep in 1 .. Repeats loop
         declare
            Q_First : Positive := All_Asked.First_Index;
         begin
            --  The keyboards of one question are consecutive in All_Asked.
            while Q_First <= All_Asked.Last_Index loop
               declare
                  Q_Last : Positive := Q_First;
               begin
                  while Q_Last < All_Asked.Last_Index
                    and then Driver.Json."=" (All_Asked (Q_Last + 1).N, All_Asked (Q_First).N)
                  loop
                     Q_Last := Q_Last + 1;
                  end loop;
                  if Rep mod 2 = 1 then
                     for K in Q_First .. Q_Last loop
                        Visit (Doc, All_Asked (K).N, To_String (All_Asked (K).Board), Rep, All_Asked (K).S);
                     end loop;
                  else
                     for K in reverse Q_First .. Q_Last loop
                        Visit (Doc, All_Asked (K).N, To_String (All_Asked (K).Board), Rep, All_Asked (K).S);
                     end loop;
                  end if;
                  Q_First := Q_Last + 1;
               end;
            end loop;
         end;
      end loop;
   end Each_Answer;

   function Head_Of (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive) return String is
     ("{""id"":" & Driver.Json.Quote (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "id")))
      & ",""kind"":" & Driver.Json.Quote (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "kind")))
      & ",""keyboard"":" & Driver.Json.Quote (Board) & ",""rep"":" & Driver.Log.Image (Rep));

   function Is_Right (Doc : Driver.Json.Document; N : Driver.Json.Node; Program : String) return Boolean is
     (Right (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "kind")),
             Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "thing")),
             Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "other")), Program));

   Out_File : Ada.Text_IO.File_Type;

   procedure Ask_Keyboard
     (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive; S : Setting)
   is
      A    : constant Driver.Brain.Service.Answer :=
        Driver.Brain.Service.Write_Program (S.Picture, To_String (S.Prompt), S.Keys);
      Good : constant Boolean := Is_Right (Doc, N, To_String (A.Program));
   begin
      Ada.Text_IO.Put_Line
        (Out_File, Head_Of (Doc, N, Board, Rep)
         & ",""how"":" & Driver.Json.Quote (Driver.Brain.Service.Image (A.How))
         & ",""why"":" & Driver.Json.Quote (To_String (A.Why))
         & ",""seconds"":" & Driver.Log.Image (Driver.Real (A.Seconds), 1)
         & ",""right"":" & (if Good then "true" else "false")
         & ",""stretches"":" & Driver.Log.Image (Stretches (To_String (A.Program)))
         & ",""program"":" & Driver.Json.Quote (To_String (A.Program)) & "}");
      Ada.Text_IO.Flush (Out_File);
      Ada.Text_IO.Put_Line (Board & Rep'Image & " " & Driver.Brain.Service.Image (A.How) & " "
                            & (if Good then "RIGHT" else "-") & " | "
                            & Ada.Strings.Fixed.Head (To_String (A.Program), 160));
   end Ask_Keyboard;

   --  The answer read to its end, with the verdict the driver would have
   --  reached after every event.
   procedure Ask_Truncation
     (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive; S : Setting)
   is
      Text, Finish : Unbounded_String;
      Started      : constant Duration := Driver.Clock.Seconds;
      Stopped      : Boolean := False;
      Cut          : Driver.Brain.Runaway.Verdict;
      Cut_At       : Duration := 0.0;
      Cut_Chars    : Natural := 0;
      Names        : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Keyboard.Name_Words (S.Keys);

      procedure On_Text (Chunk : String; Stop : out Boolean) is
      begin
         Stop := False;
         Driver.Brain.Service.Read_Event (Chunk, Text, Finish);
         if not Stopped then
            declare
               V : constant Driver.Brain.Runaway.Verdict := Driver.Brain.Runaway.Judge (To_String (Text), Names, False);
            begin
               if Driver.Brain.Runaway.Stop_Reading (V) then
                  Stopped := True;
                  Cut := V;
                  Cut_At := Driver.Clock.Seconds - Started;
                  Cut_Chars := Length (Text);
               end if;
            end;
         end if;
      end On_Text;

      R : constant Driver.Services.Reply := Driver.Services.Call_Streaming
        (Driver.Services.Brain, "/v1/chat/completions",
         Driver.Brain.Service.Program_Request (Driver.Brain.Pictures.Data_Url (S.Picture), To_String (S.Prompt),
                                               Driver.Brain.Keyboard.Grammar (S.Keys)),
         On_Text'Access);
      Ended_At : constant Duration := Driver.Clock.Seconds - Started;
      Whole    : constant Driver.Brain.Runaway.Verdict := Driver.Brain.Runaway.Judge (To_String (Text), Names, True);
      Kept     : constant String :=
        Driver.Brain.Runaway.First_Lines (To_String (Text), (if Stopped then Cut.Keep else Whole.Keep));
      Full     : constant String := Driver.Brain.Runaway.First_Lines (To_String (Text), Whole.Keep);
   begin
      Ada.Text_IO.Put_Line
        (Out_File, Head_Of (Doc, N, Board, Rep)
         & ",""ok"":" & (if R.Ok then "true" else "false")
         & ",""cut"":" & Driver.Json.Quote (if not Stopped then "none" elsif Cut.Complete then "complete" else "ran away")
         & ",""why"":" & Driver.Json.Quote (To_String (Cut.Why))
         & ",""cut_seconds"":" & Driver.Log.Image (Driver.Real (Cut_At), 2)
         & ",""cut_chars"":" & Driver.Log.Image (Cut_Chars)
         & ",""end_seconds"":" & Driver.Log.Image (Driver.Real (Ended_At), 2)
         & ",""end_chars"":" & Driver.Log.Image (Length (Text))
         & ",""finish"":" & Driver.Json.Quote (To_String (Finish))
         & ",""kept_right"":" & (if Is_Right (Doc, N, Kept) then "true" else "false")
         & ",""kept_stretches"":" & Driver.Log.Image (Stretches (Kept))
         & ",""full_stretches"":" & Driver.Log.Image (Stretches (Full))
         & ",""kept"":" & Driver.Json.Quote (Kept)
         & ",""text"":" & Driver.Json.Quote (To_String (Text)) & "}");
      Ada.Text_IO.Flush (Out_File);
      Ada.Text_IO.Put_Line (Board & Rep'Image & " cut "
                            & (if not Stopped then "none" elsif Cut.Complete then "complete" else "ran away")
                            & " at " & Driver.Log.Image (Driver.Real (Cut_At), 1) & " s, ended "
                            & Driver.Log.Image (Driver.Real (Ended_At), 1) & " s (" & To_String (Finish) & ")");
   end Ask_Truncation;

   procedure Over_Questions (Visit : not null access procedure
                               (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive;
                                S : Setting)) is
   begin
      Configure (Argument (2));
      Ada.Text_IO.Create (Out_File, Ada.Text_IO.Append_File, Argument (5));
      Each_Answer (Argument (3), Natural'Value (Argument (4)), Visit);
      Ada.Text_IO.Close (Out_File);
   end Over_Questions;

   --  One question, one keyboard: the answer printed as it streams, with the
   --  driver's verdict on it, until the verdict stops reading or LIMIT
   --  characters arrived (the limit is the measurement's, never the driver's).
   procedure Stream_Run is
      Doc   : Driver.Json.Document;
      Ok    : Boolean;
      Why   : Unbounded_String;
      Limit : constant Positive := Positive'Value (Argument (6));
   begin
      Configure (Argument (2));
      Driver.Json.Parse (Slurp (Argument (3)), Doc, Ok, Why);
      declare
         S : constant Setting :=
           Setting_Of (Doc, Driver.Json.Element (Doc, Driver.Json.Root (Doc), Positive'Value (Argument (4))),
                       Argument (5));
         Text, Finish : Unbounded_String;
         Started : constant Duration := Driver.Clock.Seconds;

         procedure On_Text (Chunk : String; Stop : out Boolean) is
            V : Driver.Brain.Runaway.Verdict;
         begin
            Driver.Brain.Service.Read_Event (Chunk, Text, Finish);
            V := Driver.Brain.Runaway.Judge (To_String (Text), Driver.Brain.Keyboard.Name_Words (S.Keys), False);
            Stop := Driver.Brain.Runaway.Stop_Reading (V) or else Length (Text) >= Limit;
            if Stop then
               Ada.Text_IO.Put_Line (To_String (Text));
               Ada.Text_IO.Put_Line ("-- " & Driver.Log.Image (Driver.Real (Driver.Clock.Seconds - Started), 1)
                                     & " s, " & Driver.Log.Image (Length (Text)) & " characters: "
                                     & (if V.Complete then "complete" elsif V.Fired then "ran away"
                                        else "limit of the measurement") & " " & To_String (V.Why));
            end if;
         end On_Text;

         R : constant Driver.Services.Reply := Driver.Services.Call_Streaming
           (Driver.Services.Brain, "/v1/chat/completions",
            Driver.Brain.Service.Program_Request (Driver.Brain.Pictures.Data_Url (S.Picture), To_String (S.Prompt),
                                                  Driver.Brain.Keyboard.Grammar (S.Keys)),
            On_Text'Access);
      begin
         if not R.Ok then
            Ada.Text_IO.Put_Line ("failed: " & To_String (R.Why));
         elsif Length (Finish) > 0 then
            Ada.Text_IO.Put_Line (To_String (Text));
            Ada.Text_IO.Put_Line ("-- " & Driver.Log.Image (Driver.Real (Driver.Clock.Seconds - Started), 1)
                                  & " s, " & Driver.Log.Image (Length (Text)) & " characters: the service ended it ("
                                  & To_String (Finish) & ")");
         end if;
      end;
   end Stream_Run;

   --  The name replay: recorded programs bound by the driver's binder, the
   --  eyes asked live through the driver's client, and the patch a box picks
   --  out read off regions marked by hand on the same pictures (the box's
   --  centre: a thing's region first, then the body's).

   type Region is record
      Eye            : Driver.Observations.Camera_Id;
      X0, Y0, X1, Y1 : Driver.Real;
   end record;

   package Region_Vectors is new Ada.Containers.Vectors (Positive, Region);

   type Marked is record
      Name    : Unbounded_String;
      Regions : Region_Vectors.Vector;
   end record;

   package Marked_Vectors is new Ada.Containers.Vectors (Positive, Marked);

   type Replay_Eyes is new Driver.Brain.Names.Senses with record
      Images : Driver.Observations.Image_Vectors.Vector;
      Things : Marked_Vectors.Vector;   --  thing K is Driver.World.Thing_Id (K)
      Self   : Region_Vectors.Vector;
   end record;

   overriding function Eyes (S : Replay_Eyes) return Driver.Brain.Names.Eye_Vectors.Vector;
   overriding function Sees (S : Replay_Eyes; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean;
   overriding procedure Ask_Where
     (S      : in out Replay_Eyes;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String);
   overriding procedure Identify
     (S     : in out Replay_Eyes;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String);

   overriding function Eyes (S : Replay_Eyes) return Driver.Brain.Names.Eye_Vectors.Vector is
      R : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      for E in S.Images.First_Index .. S.Images.Last_Index loop
         R.Append (E);
      end loop;
      return R;
   end Eyes;

   function Inside (R : Region; E : Driver.Observations.Camera_Id; U, V : Driver.Real) return Boolean is
     (Driver.Observations."=" (R.Eye, E) and then U in R.X0 .. R.X1 and then V in R.Y0 .. R.Y1);

   overriding function Sees (S : Replay_Eyes; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean is
     (Natural (T) in S.Things.First_Index .. S.Things.Last_Index
      and then (for some R of S.Things (Natural (T)).Regions => Driver.Observations."=" (R.Eye, E)));

   overriding procedure Ask_Where
     (S      : in out Replay_Eyes;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String) is
   begin
      Driver.Brain.Service.Ask_Where (S.Images (E), Name, Answer, Where, Why);
   end Ask_Where;

   overriding procedure Identify
     (S     : in out Replay_Eyes;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String)
   is
      U : constant Driver.Real := (Where.Top_Left.U + Where.Bottom_Right.U) / 2.0;
      V : constant Driver.Real := (Where.Top_Left.V + Where.Bottom_Right.V) / 2.0;
   begin
      T := Driver.Brain.Names.Thing_Id'First;
      Why := Null_Unbounded_String;
      for K in S.Things.First_Index .. S.Things.Last_Index loop
         if (for some R of S.Things (K).Regions => Inside (R, E, U, V)) then
            Found := Driver.Brain.Names.A_Thing;
            T := Driver.Brain.Names.Thing_Id (K);
            return;
         end if;
      end loop;
      Found := (if (for some R of S.Self => Inside (R, E, U, V)) then Driver.Brain.Names.Part_Of_Me
                else Driver.Brain.Names.No_Patch);
   end Identify;

   procedure Names_Run is
      Doc : Driver.Json.Document;
      Ok  : Boolean;
      Why : Unbounded_String;
      Repeats : constant Positive := Positive'Value (Argument (4));
      S       : Replay_Eyes;
      Right, Asked : Natural := 0;

      function Real_Of (N : Driver.Json.Node; K : Positive) return Driver.Real is
        (Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, K)));

      function Region_Of (N : Driver.Json.Node) return Region is
        ((Eye => Driver.Observations.Camera_Id (Natural (Real_Of (N, 1))),
          X0 => Real_Of (N, 2), Y0 => Real_Of (N, 3), X1 => Real_Of (N, 4), Y1 => Real_Of (N, 5)));
   begin
      Configure (Argument (2));
      Driver.Json.Parse (Slurp (Argument (3)), Doc, Ok, Why);
      if not Ok then
         Ada.Text_IO.Put_Line ("replay: " & To_String (Why));
         return;
      end if;
      Ada.Text_IO.Create (Out_File, Ada.Text_IO.Append_File, Argument (5));
      declare
         Root   : constant Driver.Json.Node := Driver.Json.Root (Doc);
         Eyes   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "eyes");
         Things : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "things");
         Self   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "self");
         Intent : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "intent");
         Runs   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "runs");
         Glue   : constant Driver.Brain.Keyboard.Word_Vectors.Vector :=
           Driver.Brain.Keyboard.Name_Words (Keys ("Q", Driver.Json.Count (Doc, Eyes)));
      begin
         for E in 1 .. Driver.Json.Count (Doc, Eyes) loop
            S.Images.Append (Read_Ppm (Driver.Json.Text (Doc, Driver.Json.Element (Doc, Eyes, E))));
         end loop;
         for K in 1 .. Driver.Json.Count (Doc, Things) loop
            declare
               N : constant Driver.Json.Node := Driver.Json.Element (Doc, Things, K);
               M : Marked := (Name => To_Unbounded_String (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "name"))),
                              Regions => <>);
               Rs : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "regions");
            begin
               for J in 1 .. Driver.Json.Count (Doc, Rs) loop
                  M.Regions.Append (Region_Of (Driver.Json.Element (Doc, Rs, J)));
               end loop;
               S.Things.Append (M);
            end;
         end loop;
         for J in 1 .. Driver.Json.Count (Doc, Self) loop
            S.Self.Append (Region_Of (Driver.Json.Element (Doc, Self, J)));
         end loop;
         for Rep in 1 .. Repeats loop
            for R in 1 .. Driver.Json.Count (Doc, Runs) loop
               declare
                  Run    : constant Driver.Json.Node := Driver.Json.Element (Doc, Runs, R);
                  Id     : constant String := Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Run, "id"));
                  Rounds : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Run, "rounds");
                  Table  : Driver.Brain.Names.Table;
               begin
                  for Q in 1 .. Driver.Json.Count (Doc, Rounds) loop
                     declare
                        Round : constant Driver.Json.Node := Driver.Json.Element (Doc, Rounds, Q);
                        View  : constant Driver.Brain.Names.Eye_Id :=
                          Driver.Brain.Names.Eye_Id (Natural (Real_Of (Round, 1)));
                        P     : Driver.Brain.Programs.Program;
                        Bad   : Driver.Brain.Programs.Refusal;
                        Bound : Driver.Brain.Wants.Binding_Maps.Map;
                     begin
                        Driver.Brain.Parser.Parse (Driver.Json.Text (Doc, Driver.Json.Element (Doc, Round, 2)), P, Ok, Bad);
                        for N of Driver.Brain.Wants.Names_Of (P) loop
                           declare
                              B : Driver.Brain.Names.Binding;
                           begin
                              Driver.Brain.Names.Bind (Table, S, View, N, Glue, B);
                              Bound.Include (N, B);
                           end;
                        end loop;
                        for N of Driver.Brain.Wants.Names_Of (P) loop
                           declare
                              B      : Driver.Brain.Names.Binding := Bound.Element (N);
                              Meant  : constant String := Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Intent, N));
                              Got    : Unbounded_String;
                              Good   : Boolean;
                           begin
                              Driver.Brain.Names.Bind_Again (Table, Glue, N, B);
                              Got := To_Unbounded_String
                                (case B.Kind is
                                    when Driver.Brain.Names.To_Thing => To_String (S.Things (Natural (B.Thing)).Name),
                                    when Driver.Brain.Names.To_Place => "a place",
                                    when Driver.Brain.Names.Unbound  => "none");
                              Good := (if Meant = "either" then To_String (Got) in "none" | "scissors"
                                       else To_String (Got) = Meant);
                              Asked := Asked + 1;
                              Right := Right + (if Good then 1 else 0);
                              Ada.Text_IO.Put_Line
                                (Out_File, "{""rep"":" & Driver.Log.Image (Rep) & ",""run"":" & Driver.Json.Quote (Id)
                                 & ",""round"":" & Driver.Log.Image (Q) & ",""view"":" & Driver.Log.Image (Natural (View))
                                 & ",""name"":" & Driver.Json.Quote (N) & ",""meant"":" & Driver.Json.Quote (Meant)
                                 & ",""bound"":" & Driver.Json.Quote (To_String (Got))
                                 & ",""right"":" & (if Good then "true" else "false")
                                 & ",""account"":" & Driver.Json.Quote (To_String (B.Account)) & "}");
                              Ada.Text_IO.Put_Line (Id & " round" & Q'Image & " eye" & View'Image & " | " & N & " -> "
                                                    & To_String (Got) & (if Good then "" else "   WRONG (meant "
                                                                         & Meant & ")"));
                           end;
                        end loop;
                     end;
                  end loop;
               end;
            end loop;
         end loop;
      end;
      Ada.Text_IO.Close (Out_File);
      Ada.Text_IO.Put_Line ("right" & Right'Image & " of" & Asked'Image);
   end Names_Run;

   --  Bindings of the names answers wrote: every distinct name of a scene
   --  bound as a first round binds it (no name given before, the eye fixed in
   --  the scene first), the eyes asked live, each binding scored against the
   --  thing the question meant by it (the name holds the letters of the
   --  question's thing or of its second thing; "?" when it holds neither).
   procedure Bindings_Run is
      Scene_Doc, Q_Doc, A_Doc : Driver.Json.Document;
      Ok      : Boolean;
      Why     : Unbounded_String;
      Repeats : constant Positive := Positive'Value (Argument (6));

      type Scene_Eyes_Access is access Replay_Eyes;
      package Scene_Vectors is new Ada.Containers.Vectors (Positive, Scene_Eyes_Access);
      package Text_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

      Scenes     : Scene_Vectors.Vector;
      First_Eyes : Text_Vectors.Vector;   --  each scene's first picture, which questions name

      type Name_Use is record
         Scene : Positive;
         Name  : Unbounded_String;
         Meant : Unbounded_String;
         Count : Natural := 0;
      end record;

      package Use_Vectors is new Ada.Containers.Vectors (Positive, Name_Use);
      Uses : Use_Vectors.Vector;

      function Real_Of (D : Driver.Json.Document; N : Driver.Json.Node; K : Positive) return Driver.Real is
        (Driver.Json.Number (D, Driver.Json.Element (D, N, K)));

      function Region_Of (D : Driver.Json.Document; N : Driver.Json.Node) return Region is
        ((Eye => Driver.Observations.Camera_Id (Natural (Real_Of (D, N, 1))),
          X0 => Real_Of (D, N, 2), Y0 => Real_Of (D, N, 3), X1 => Real_Of (D, N, 4), Y1 => Real_Of (D, N, 5)));

      --  The scene's thing whose name holds the letters of Key, or "".
      function Thing_Named (S : Positive; Key : String) return String is
      begin
         if Key'Length > 0 then
            for T of Scenes (S).Things loop
               if Has_Letters (To_String (T.Name), Key) then
                  return To_String (T.Name);
               end if;
            end loop;
         end if;
         return "";
      end Thing_Named;

      Out_F : Ada.Text_IO.File_Type;
      In_F  : Ada.Text_IO.File_Type;
   begin
      Configure (Argument (2));
      Driver.Json.Parse (Slurp (Argument (3)), Scene_Doc, Ok, Why);
      if Ok then
         Driver.Json.Parse (Slurp (Argument (4)), Q_Doc, Ok, Why);
      end if;
      if not Ok then
         Ada.Text_IO.Put_Line ("bindings: " & To_String (Why));
         return;
      end if;
      for K in 1 .. Driver.Json.Count (Scene_Doc, Driver.Json.Root (Scene_Doc)) loop
         declare
            N      : constant Driver.Json.Node := Driver.Json.Element (Scene_Doc, Driver.Json.Root (Scene_Doc), K);
            Eyes   : constant Driver.Json.Node := Driver.Json.Lookup (Scene_Doc, N, "eyes");
            Things : constant Driver.Json.Node := Driver.Json.Lookup (Scene_Doc, N, "things");
            Self   : constant Driver.Json.Node := Driver.Json.Lookup (Scene_Doc, N, "self");
            S      : constant Scene_Eyes_Access := new Replay_Eyes;
         begin
            for E in 1 .. Driver.Json.Count (Scene_Doc, Eyes) loop
               S.Images.Append (Read_Ppm (Driver.Json.Text (Scene_Doc, Driver.Json.Element (Scene_Doc, Eyes, E))));
            end loop;
            First_Eyes.Append (Driver.Json.Text (Scene_Doc, Driver.Json.Element (Scene_Doc, Eyes, 1)));
            for T in 1 .. Driver.Json.Count (Scene_Doc, Things) loop
               declare
                  TN : constant Driver.Json.Node := Driver.Json.Element (Scene_Doc, Things, T);
                  Rs : constant Driver.Json.Node := Driver.Json.Lookup (Scene_Doc, TN, "regions");
                  M  : Marked := (Name    => To_Unbounded_String
                                    (Driver.Json.Text (Scene_Doc, Driver.Json.Lookup (Scene_Doc, TN, "name"))),
                                  Regions => <>);
               begin
                  for J in 1 .. Driver.Json.Count (Scene_Doc, Rs) loop
                     M.Regions.Append (Region_Of (Scene_Doc, Driver.Json.Element (Scene_Doc, Rs, J)));
                  end loop;
                  S.Things.Append (M);
               end;
            end loop;
            for J in 1 .. Driver.Json.Count (Scene_Doc, Self) loop
               S.Self.Append (Region_Of (Scene_Doc, Driver.Json.Element (Scene_Doc, Self, J)));
            end loop;
            Scenes.Append (S);
         end;
      end loop;
      --  Every name of every answer, with the scene of its question.
      Ada.Text_IO.Open (In_F, Ada.Text_IO.In_File, Argument (5));
      while not Ada.Text_IO.End_Of_File (In_F) loop
         declare
            Line : constant String := Ada.Text_IO.Get_Line (In_F);
         begin
            Driver.Json.Parse (Line, A_Doc, Ok, Why);
            if Ok then
               declare
                  Id      : constant String := Driver.Json.Text (A_Doc, Driver.Json.Lookup (A_Doc, Driver.Json.Root (A_Doc), "id"));
                  Program : constant String :=
                    Driver.Json.Text (A_Doc, Driver.Json.Lookup (A_Doc, Driver.Json.Root (A_Doc), "program"));
                  P       : Driver.Brain.Programs.Program;
                  Bad     : Driver.Brain.Programs.Refusal;
               begin
                  Driver.Brain.Parser.Parse (Program, P, Ok, Bad);
                  for Q in 1 .. Driver.Json.Count (Q_Doc, Driver.Json.Root (Q_Doc)) loop
                     declare
                        QN : constant Driver.Json.Node := Driver.Json.Element (Q_Doc, Driver.Json.Root (Q_Doc), Q);
                        function Get (K : String) return String is (Driver.Json.Text (Q_Doc, Driver.Json.Lookup (Q_Doc, QN, K)));
                        First : constant String :=
                          Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Driver.Json.Lookup (Q_Doc, QN, "eyes"), 1));
                     begin
                        if Ok and then Get ("id") = Id then
                           for S in First_Eyes.First_Index .. First_Eyes.Last_Index loop
                              if First_Eyes (S) = First then
                                 for N of Driver.Brain.Wants.Names_Of (P) loop
                                    declare
                                       Meant : constant String :=
                                         (if Has_Letters (N, Get ("thing")) then Thing_Named (S, Get ("thing"))
                                          elsif Has_Letters (N, Get ("other")) then Thing_Named (S, Get ("other"))
                                          else "?");
                                       Found : Boolean := False;
                                    begin
                                       for U of Uses loop
                                          if U.Scene = S and then To_String (U.Name) = N then
                                             U.Count := U.Count + 1;
                                             Found := True;
                                          end if;
                                       end loop;
                                       if not Found then
                                          Uses.Append (Name_Use'(Scene => S, Name => To_Unbounded_String (N),
                                                        Meant => To_Unbounded_String (Meant), Count => 1));
                                       end if;
                                    end;
                                 end loop;
                              end if;
                           end loop;
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end;
      end loop;
      Ada.Text_IO.Close (In_F);
      Ada.Text_IO.Put_Line (Uses.Length'Image & " distinct names");
      Ada.Text_IO.Create (Out_F, Ada.Text_IO.Append_File, Argument (7));
      declare
         Glue : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Keyboard.Name_Words (Keys ("QH2o", 3));
         Right, Weight : Natural := 0;
      begin
         for U of Uses loop
            for Rep in 1 .. Repeats loop
               declare
                  Table : Driver.Brain.Names.Table;
                  B     : Driver.Brain.Names.Binding;
                  Got   : Unbounded_String;
               begin
                  Driver.Brain.Names.Bind (Table, Scenes (U.Scene).all, 1, To_String (U.Name), Glue, B);
                  Driver.Brain.Names.Bind_Again (Table, Glue, To_String (U.Name), B);
                  Got := To_Unbounded_String
                    (case B.Kind is
                        when Driver.Brain.Names.To_Thing =>
                           To_String (Scenes (U.Scene).Things (Natural (B.Thing)).Name),
                        when Driver.Brain.Names.To_Place => "a place",
                        when Driver.Brain.Names.Unbound  => "none");
                  Weight := Weight + U.Count;
                  if Got = U.Meant then
                     Right := Right + U.Count;
                  end if;
                  Ada.Text_IO.Put_Line
                    (Out_F, "{""scene"":" & Driver.Log.Image (U.Scene) & ",""rep"":" & Driver.Log.Image (Rep)
                     & ",""name"":" & Driver.Json.Quote (To_String (U.Name)) & ",""count"":" & Driver.Log.Image (U.Count)
                     & ",""meant"":" & Driver.Json.Quote (To_String (U.Meant)) & ",""bound"":"
                     & Driver.Json.Quote (To_String (Got)) & ",""right"":" & (if Got = U.Meant then "true" else "false")
                     & ",""account"":" & Driver.Json.Quote (To_String (B.Account)) & "}");
                  Ada.Text_IO.Flush (Out_F);
               end;
            end loop;
         end loop;
         Ada.Text_IO.Put_Line ("right, weighted by how often each name was written:" & Right'Image & " of"
                               & Weight'Image);
      end;
      Ada.Text_IO.Close (Out_F);
   end Bindings_Run;

   ---------------------------------------------------------------------------
   --  Binding measured on scenes with truth (driver/tools/brain_scene), in
   --  three steps so the instrument, which other runs share, is held only
   --  while it segments: every eye asked where every name of a keyboard run's
   --  programs is (where-all); every box an eye gave segmented as the live
   --  Identify asks it (patches); the binder replayed on those answers, its
   --  patches told apart by the truth's labels of their pixels, and scored
   --  (bind-score). A question of such a run also holds "labels" (the PGM of
   --  each eye), "objects" (the objects in label order) and "target" (the
   --  object its task names, or "").

   Robot_Label : constant := 255;   --  brain_scene's label for the robot's own pixels

   type Text_Access is access String;

   type Label_Image is record
      Width, Height : Natural := 0;
      Data          : Text_Access;   --  one character per pixel, row by row
   end record;

   function Read_Pgm (Path : String) return Label_Image is
      S : constant String := Slurp (Path);
      I : Positive := S'First;

      function Field return Natural is
         First : Positive;
      begin
         while S (I) in ' ' | ASCII.LF | ASCII.CR | ASCII.HT loop
            I := I + 1;
         end loop;
         First := I;
         while S (I) in '0' .. '9' loop
            I := I + 1;
         end loop;
         return Natural'Value (S (First .. I - 1));
      end Field;
   begin
      if S (I .. I + 1) /= "P5" then
         raise Constraint_Error with Path & " is not a binary PGM";
      end if;
      I := I + 2;
      declare
         W : constant Positive := Field;
         H : constant Positive := Field;
         M : constant Natural := Field;
         pragma Unreferenced (M);
      begin
         return (Width => W, Height => H, Data => new String'(S (I + 1 .. I + W * H)));
      end;
   end Read_Pgm;

   function Label_At (L : Label_Image; Column, Row : Natural) return Natural is
     (Character'Pos (L.Data (L.Data'First + Row * L.Width + Column)));

   type Count_Array is array (0 .. Robot_Label) of Natural;

   --  The labels of the pixels whose centres lie in the box.
   function Counts_In (L : Label_Image; B : Driver.Brain.Names.Box) return Count_Array is
      Half : constant := 0.5;   --  the centre of a pixel (Driver.Images)
      R    : Count_Array := [others => 0];
      C0   : constant Integer := Integer'Max (0, Integer (Driver.Real'Ceiling (B.Top_Left.U - Half)));
      C1   : constant Integer := Integer'Min (L.Width - 1, Integer (Driver.Real'Floor (B.Bottom_Right.U - Half)));
      R0   : constant Integer := Integer'Max (0, Integer (Driver.Real'Ceiling (B.Top_Left.V - Half)));
      R1   : constant Integer := Integer'Min (L.Height - 1, Integer (Driver.Real'Floor (B.Bottom_Right.V - Half)));
   begin
      for Row in R0 .. R1 loop
         for Column in C0 .. C1 loop
            R (Label_At (L, Column, Row)) := R (Label_At (L, Column, Row)) + 1;
         end loop;
      end loop;
      return R;
   end Counts_In;

   function All_Counts (L : Label_Image) return Count_Array is
     (Counts_In (L, (Top_Left => (U => 0.0, V => 0.0),
                     Bottom_Right => (U => Driver.Real (L.Width), V => Driver.Real (L.Height)))));

   --  The object most of the counted pixels show, robot and nothing left out; 0 when none.
   function Most_Object (C : Count_Array) return Natural is
      Best : Natural := 0;
   begin
      for L in 1 .. Robot_Label - 1 loop
         if C (L) > 0 and then (Best = 0 or else C (L) > C (Best)) then
            Best := L;
         end if;
      end loop;
      return Best;
   end Most_Object;

   function Counts_Text (C : Count_Array) return String is
      R : Unbounded_String;
   begin
      for L in C'Range loop
         if C (L) > 0 then
            Append (R, (if Length (R) > 0 then "," else "") & "[" & Driver.Log.Image (L) & ","
                    & Driver.Log.Image (C (L)) & "]");
         end if;
      end loop;
      return "[" & To_String (R) & "]";
   end Counts_Text;

   function Box_Text (B : Driver.Brain.Names.Box) return String is
     (Driver.Log.Image (B.Top_Left.U, 2) & "," & Driver.Log.Image (B.Top_Left.V, 2) & ","
      & Driver.Log.Image (B.Bottom_Right.U, 2) & "," & Driver.Log.Image (B.Bottom_Right.V, 2));

   function Pointing_Text (P : Driver.Brain.Names.Pointing) return String is
     (case P is
         when Driver.Brain.Names.Boxed     => "boxed",
         when Driver.Brain.Names.Not_Here  => "not_here",
         when Driver.Brain.Names.No_Answer => "no_answer");

   function Question_Node (Doc : Driver.Json.Document; Id : String) return Driver.Json.Node is
   begin
      for Q in 1 .. Driver.Json.Count (Doc, Driver.Json.Root (Doc)) loop
         declare
            N : constant Driver.Json.Node := Driver.Json.Element (Doc, Driver.Json.Root (Doc), Q);
         begin
            if Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "id")) = Id then
               return N;
            end if;
         end;
      end loop;
      return Driver.Json.No_Node;
   end Question_Node;

   function Eye_File (Doc : Driver.Json.Document; N : Driver.Json.Node; Field : String;
                      E : Driver.Observations.Camera_Id) return String is
     (Driver.Json.Text (Doc, Driver.Json.Element (Doc, Driver.Json.Lookup (Doc, N, Field), Positive (E))));

   --  The eyes in the order the binder asks them: the view, then the others in camera order.
   function Asking_Order (View : Driver.Observations.Camera_Id; Count : Natural)
                          return Driver.Brain.Names.Eye_Vectors.Vector is
      R : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      R.Append (View);
      for E in 1 .. Count loop
         if Driver.Observations."/=" (Driver.Observations.Camera_Id (E), View) then
            R.Append (Driver.Observations.Camera_Id (E));
         end if;
      end loop;
      return R;
   end Asking_Order;

   --  Calls Visit for every answer of a keyboard run whose question is in the
   --  file and whose program reads.
   procedure Over_Answers
     (Q_Doc : Driver.Json.Document;
      Path  : String;
      Visit : not null access procedure (Q : Driver.Json.Node; Id : String; Rep : Natural; Board : String;
                                         P : Driver.Brain.Programs.Program))
   is
      F     : Ada.Text_IO.File_Type;
      A_Doc : Driver.Json.Document;
      Ok    : Boolean;
      Why   : Unbounded_String;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Driver.Json.Parse (Ada.Text_IO.Get_Line (F), A_Doc, Ok, Why);
         if Ok then
            declare
               Root : constant Driver.Json.Node := Driver.Json.Root (A_Doc);
               Id   : constant String := Driver.Json.Text (A_Doc, Driver.Json.Lookup (A_Doc, Root, "id"));
               QN   : constant Driver.Json.Node := Question_Node (Q_Doc, Id);
               P    : Driver.Brain.Programs.Program;
               Bad  : Driver.Brain.Programs.Refusal;
            begin
               Driver.Brain.Parser.Parse (Driver.Json.Text (A_Doc, Driver.Json.Lookup (A_Doc, Root, "program")), P, Ok, Bad);
               if Ok and then Driver.Json."/=" (QN, Driver.Json.No_Node) then
                  Visit (QN, Id, Natural (Driver.Json.Number (A_Doc, Driver.Json.Lookup (A_Doc, Root, "rep"))),
                         Driver.Json.Text (A_Doc, Driver.Json.Lookup (A_Doc, Root, "keyboard")), P);
               end if;
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (F);
   end Over_Answers;

   --  brain_measure where-all HOST:PORT QUESTIONS.json ANSWERS.jsonl OUT.jsonl
   procedure Where_All_Run is
      Q_Doc : Driver.Json.Document;
      Ok    : Boolean;
      Why   : Unbounded_String;
      Out_F : Ada.Text_IO.File_Type;

      procedure Ask (Q : Driver.Json.Node; Id : String; Rep : Natural; Board : String;
                     P : Driver.Brain.Programs.Program) is
         pragma Unreferenced (Board);
         Count  : constant Natural := Driver.Json.Count (Q_Doc, Driver.Json.Lookup (Q_Doc, Q, "eyes"));
         Images : Driver.Observations.Image_Vectors.Vector;
      begin
         for E in 1 .. Count loop
            Images.Append (Read_Ppm (Eye_File (Q_Doc, Q, "eyes", Driver.Observations.Camera_Id (E))));
         end loop;
         for N of Driver.Brain.Wants.Names_Of (P) loop
            for E of Asking_Order (View_Of (Q_Doc, Q), Count) loop
               declare
                  Found : Driver.Brain.Names.Pointing;
                  Where : Driver.Brain.Names.Box;
                  Said  : Unbounded_String;
               begin
                  Driver.Brain.Service.Ask_Where (Images (E), N, Found, Where, Said);
                  Ada.Text_IO.Put_Line
                    (Out_F, "{""id"":" & Driver.Json.Quote (Id) & ",""rep"":" & Driver.Log.Image (Rep)
                     & ",""name"":" & Driver.Json.Quote (N) & ",""eye"":" & Driver.Log.Image (Natural (E))
                     & ",""answer"":" & Driver.Json.Quote (Pointing_Text (Found)) & ",""box"":[" & Box_Text (Where)
                     & "],""why"":" & Driver.Json.Quote (To_String (Said)) & "}");
                  Ada.Text_IO.Flush (Out_F);
                  Ada.Text_IO.Put_Line (Id & Rep'Image & " | " & N & " | eye" & E'Image & ": " & Pointing_Text (Found));
               end;
            end loop;
         end loop;
      end Ask;
   begin
      Configure (Argument (2));
      Driver.Json.Parse (Slurp (Argument (3)), Q_Doc, Ok, Why);
      if not Ok then
         Ada.Text_IO.Put_Line ("questions: " & To_String (Why));
         return;
      end if;
      Ada.Text_IO.Create (Out_F, Ada.Text_IO.Append_File, Argument (5));
      Over_Answers (Q_Doc, Argument (4), Ask'Access);
      Ada.Text_IO.Close (Out_F);
   end Where_All_Run;

   function Box_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Driver.Brain.Names.Box is
      function At_K (K : Positive) return Driver.Real is (Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, K)));
   begin
      return (Top_Left => (U => At_K (1), V => At_K (2)), Bottom_Right => (U => At_K (3), V => At_K (4)));
   end Box_Of;

   package Label_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Label_Image);
   Label_Cache : Label_Maps.Map;

   function Labels_Of (Path : String) return Label_Image is
   begin
      if not Label_Cache.Contains (Path) then
         Label_Cache.Include (Path, Read_Pgm (Path));
      end if;
      return Label_Cache (Path);
   end Labels_Of;

   --  brain_measure patches HOST:PORT QUESTIONS.json WHERE.jsonl OUT.jsonl (HOST:PORT: the instrument)
   procedure Patches_Run is
      Q_Doc, W_Doc : Driver.Json.Document;
      Ok           : Boolean;
      Why          : Unbounded_String;
      In_F, Out_F  : Ada.Text_IO.File_Type;
      Done         : Label_Maps.Map;   --  the keys segmented already; the value is unused
      No_Points    : constant Driver.Instrument.Prompt_Array := [];
      Colon        : constant Natural := Ada.Strings.Fixed.Index (Argument (2), ":", Ada.Strings.Backward);
   begin
      Driver.Services.Configure (Driver.Services.Instrument, Argument (2) (Argument (2)'First .. Colon - 1),
                                 Natural'Value (Argument (2) (Colon + 1 .. Argument (2)'Last)));
      Driver.Json.Parse (Slurp (Argument (3)), Q_Doc, Ok, Why);
      if not Ok then
         Ada.Text_IO.Put_Line ("questions: " & To_String (Why));
         return;
      end if;
      Ada.Text_IO.Open (In_F, Ada.Text_IO.In_File, Argument (4));
      Ada.Text_IO.Create (Out_F, Ada.Text_IO.Append_File, Argument (5));
      while not Ada.Text_IO.End_Of_File (In_F) loop
         Driver.Json.Parse (Ada.Text_IO.Get_Line (In_F), W_Doc, Ok, Why);
         if Ok and then Driver.Json.Text (W_Doc, Driver.Json.Lookup (W_Doc, Driver.Json.Root (W_Doc), "answer")) = "boxed"
         then
            declare
               Root  : constant Driver.Json.Node := Driver.Json.Root (W_Doc);
               QN    : constant Driver.Json.Node :=
                 Question_Node (Q_Doc, Driver.Json.Text (W_Doc, Driver.Json.Lookup (W_Doc, Root, "id")));
               E     : constant Driver.Observations.Camera_Id :=
                 Driver.Observations.Camera_Id (Natural (Driver.Json.Number (W_Doc, Driver.Json.Lookup (W_Doc, Root, "eye"))));
               B     : constant Driver.Brain.Names.Box := Box_Of (W_Doc, Driver.Json.Lookup (W_Doc, Root, "box"));
            begin
               if Driver.Json."/=" (QN, Driver.Json.No_Node) then
                  declare
                     Picture : constant String := Eye_File (Q_Doc, QN, "eyes", E);
                     Key     : constant String := Picture & " " & Box_Text (B);
                  begin
                     if not Done.Contains (Key) then
                        Done.Include (Key, (others => <>));
                        declare
                           L      : constant Label_Image := Labels_Of (Eye_File (Q_Doc, QN, "labels", E));
                           Region : Driver.Images.Mask;
                           Score  : Driver.Real;
                           Seg_Ok : Boolean;
                           Said   : Unbounded_String;
                           Counts : Count_Array := [others => 0];
                           Inside : Natural := 0;   --  the patch's pixels whose centres lie in the box
                           Half   : constant := 0.5;   --  the centre of a pixel (Driver.Images)
                        begin
                           Driver.Instrument.Segment
                             (Read_Ppm (Picture), Has_Box => True,
                              Around => (X0 => B.Top_Left.U, Y0 => B.Top_Left.V, X1 => B.Bottom_Right.U,
                                         Y1 => B.Bottom_Right.V),
                              Points => No_Points, Region => Region, Score => Score, Ok => Seg_Ok, Why => Said);
                           if Seg_Ok then
                              for Row in 0 .. Driver.Images.Height (Region) - 1 loop
                                 for Column in 0 .. Driver.Images.Width (Region) - 1 loop
                                    if Driver.Images.Contains (Region, Column, Row) then
                                       Counts (Label_At (L, Column, Row)) := Counts (Label_At (L, Column, Row)) + 1;
                                       if Driver.Real (Column) + Half in B.Top_Left.U .. B.Bottom_Right.U
                                         and then Driver.Real (Row) + Half in B.Top_Left.V .. B.Bottom_Right.V
                                       then
                                          Inside := Inside + 1;
                                       end if;
                                    end if;
                                 end loop;
                              end loop;
                           end if;
                           Ada.Text_IO.Put_Line
                             (Out_F, "{""key"":" & Driver.Json.Quote (Key) & ",""ok"":" & (if Seg_Ok then "true" else "false")
                              & ",""why"":" & Driver.Json.Quote (To_String (Said))
                              & ",""score"":" & Driver.Log.Image ((if Seg_Ok then Score else 0.0), 3)
                              & ",""pixels"":" & Driver.Log.Image (if Seg_Ok then Driver.Images.Count (Region) else 0)
                              & ",""inside"":" & Driver.Log.Image (Inside)
                              & ",""labels"":" & Counts_Text (Counts) & "}");
                           Ada.Text_IO.Flush (Out_F);
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (In_F);
      Ada.Text_IO.Close (Out_F);
      Ada.Text_IO.Put_Line (Done.Length'Image & " boxes segmented");
   end Patches_Run;

   --  What the replayed binder asked and was told, step by step.
   type Step is record
      Eye         : Driver.Observations.Camera_Id := 1;
      Answer      : Driver.Brain.Names.Pointing := Driver.Brain.Names.No_Answer;
      In_Box      : Count_Array := [others => 0];
      Identified  : Boolean := False;
      Found       : Driver.Brain.Names.Patch := Driver.Brain.Names.No_Patch;
      Patch_Label : Natural := 0;    --  the label most of the patch's pixels off the robot show
      On_Me       : Natural := 0;    --  the patch's pixels on the robot
      Pixels      : Natural := 0;
   end record;

   package Step_Vectors is new Ada.Containers.Vectors (Positive, Step);

   type Where_Entry is record
      Answer : Driver.Brain.Names.Pointing := Driver.Brain.Names.No_Answer;
      Where  : Driver.Brain.Names.Box;
   end record;

   package Where_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Where_Entry);

   type Patch_Entry is record
      Ok     : Boolean := False;
      Pixels : Natural := 0;
      Counts : Count_Array := [others => 0];
   end record;

   package Patch_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Patch_Entry);
   package Path_Vectors is new Ada.Containers.Indefinite_Vectors (Driver.Observations.Camera_Id, String);
   package Labels_Vectors is new Ada.Containers.Vectors (Driver.Observations.Camera_Id, Label_Image);
   package Counts_Vectors is new Ada.Containers.Vectors (Driver.Observations.Camera_Id, Count_Array);

   Wheres  : Where_Maps.Map;   --  by id|rep|name|eye
   Patches : Patch_Maps.Map;   --  by picture and box

   type Cached_Eyes is new Driver.Brain.Names.Senses with record
      Prefix     : Unbounded_String;   --  id|rep|
      Pictures   : Path_Vectors.Vector;
      Labels     : Labels_Vectors.Vector;
      Shown      : Counts_Vectors.Vector;   --  each eye's count of every label
      Objects    : Natural := 0;
      Steps      : Step_Vectors.Vector;
      Background : Natural := 0;   --  patches on no object so far, each a thing of its own
   end record;

   overriding function Eyes (S : Cached_Eyes) return Driver.Brain.Names.Eye_Vectors.Vector;
   overriding function Sees (S : Cached_Eyes; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean;
   overriding procedure Ask_Where
     (S      : in out Cached_Eyes;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String);
   overriding procedure Identify
     (S     : in out Cached_Eyes;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String);

   overriding function Eyes (S : Cached_Eyes) return Driver.Brain.Names.Eye_Vectors.Vector is
      R : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      for E in S.Pictures.First_Index .. S.Pictures.Last_Index loop
         R.Append (E);
      end loop;
      return R;
   end Eyes;

   overriding function Sees (S : Cached_Eyes; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean is
     (Natural (T) <= S.Objects and then S.Shown (E) (Natural (T)) > 0);

   overriding procedure Ask_Where
     (S      : in out Cached_Eyes;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String)
   is
      Key : constant String := To_String (S.Prefix) & Name & "|" & Driver.Log.Image (Natural (E));
      St  : Step;
   begin
      Why := Null_Unbounded_String;
      if Wheres.Contains (Key) then
         Answer := Wheres (Key).Answer;
         Where := Wheres (Key).Where;
      else
         Answer := Driver.Brain.Names.No_Answer;
         Where := (others => <>);
         Why := To_Unbounded_String ("this eye was not asked in the run");
      end if;
      St.Eye := E;
      St.Answer := Answer;
      if Answer = Driver.Brain.Names.Boxed then
         St.In_Box := Counts_In (S.Labels (E), Where);
      end if;
      S.Steps.Append (St);
   end Ask_Where;

   overriding procedure Identify
     (S     : in out Cached_Eyes;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String)
   is
      Key : constant String := S.Pictures (E) & " " & Box_Text (Where);
      St  : Step := S.Steps.Last_Element;
   begin
      Why := Null_Unbounded_String;
      T := Driver.Brain.Names.Thing_Id'First;
      Found := Driver.Brain.Names.No_Patch;
      if Patches.Contains (Key) and then Patches (Key).Ok and then Patches (Key).Pixels > 0 then
         declare
            P    : constant Patch_Entry := Patches (Key);
            Most : Natural := 0;   --  the label most pixels off the robot show, nothing included
         begin
            for L in 1 .. Robot_Label - 1 loop
               if P.Counts (L) > P.Counts (Most) then
                  Most := L;
               end if;
            end loop;
            St.Pixels := P.Pixels;
            St.On_Me := P.Counts (Robot_Label);
            St.Patch_Label := Most;
            --  As the live Identify (Driver.Brain.Live.Mostly): a patch most of
            --  whose pixels are on the body is part of me; any other is the
            --  thing on those pixels. The truth gives each pixel one label, so
            --  no pixel of the body is also a thing's.
            if 2 * P.Counts (Robot_Label) > P.Pixels then
               Found := Driver.Brain.Names.Part_Of_Me;
               Why := To_Unbounded_String ("most of its pixels lie on my own body in that eye");
            elsif Most > 0 then
               Found := Driver.Brain.Names.A_Thing;
               T := Driver.Brain.Names.Thing_Id (Most);
            else
               S.Background := S.Background + 1;
               Found := Driver.Brain.Names.A_Thing;
               T := Driver.Brain.Names.Thing_Id (S.Objects + S.Background);
            end if;
         end;
      elsif not Patches.Contains (Key) then
         Why := To_Unbounded_String ("the box was not segmented in the run");
      end if;
      St.Identified := True;
      St.Found := Found;
      S.Steps.Replace_Element (S.Steps.Last_Index, St);
   end Identify;

   --  brain_measure bind-score QUESTIONS.json ANSWERS.jsonl WHERE.jsonl PATCHES.jsonl MEANT.json OUT.jsonl
   --
   --  MEANT.json says what each name meant, for scoring only: {"names":
   --  {name: object | "?" | "none"}, "words": {object: [word, ...]}}. A name
   --  it does not list is taken to mean the one object whose words its
   --  letters hold, marked "auto", or "?" when they hold none or several;
   --  "?" is a name that points at no one object, "none" one that points at
   --  nothing in the scene.
   procedure Bind_Score_Run is
      Q_Doc, M_Doc, L_Doc : Driver.Json.Document;
      Ok    : Boolean;
      Why   : Unbounded_String;
      In_F  : Ada.Text_IO.File_Type;
      Out_F : Ada.Text_IO.File_Type;
      Names_Node, Words_Node : Driver.Json.Node;
      Total, Right_Count : Natural := 0;

      function Meant_Of (Name : String; Objects : Driver.Json.Node; How : out Unbounded_String) return String is
         Listed : constant Driver.Json.Node := Driver.Json.Lookup (M_Doc, Names_Node, Name);
         Pick   : Unbounded_String;
         Picks  : Natural := 0;
      begin
         if Driver.Json."/=" (Listed, Driver.Json.No_Node) then
            How := To_Unbounded_String ("listed");
            return Driver.Json.Text (M_Doc, Listed);
         end if;
         How := To_Unbounded_String ("auto");
         for K in 1 .. Driver.Json.Count (Q_Doc, Objects) loop
            declare
               Object : constant String := Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Objects, K));
               Words  : constant Driver.Json.Node := Driver.Json.Lookup (M_Doc, Words_Node, Object);
               Holds  : Boolean := False;
            begin
               if Driver.Json."/=" (Words, Driver.Json.No_Node) then
                  for W in 1 .. Driver.Json.Count (M_Doc, Words) loop
                     Holds := Holds
                       or else Has_Letters (Name, Driver.Json.Text (M_Doc, Driver.Json.Element (M_Doc, Words, W)));
                  end loop;
               end if;
               if Holds then
                  Picks := Picks + 1;
                  Pick := To_Unbounded_String (Object);
               end if;
            end;
         end loop;
         return (if Picks = 1 then To_String (Pick) else "?");
      end Meant_Of;

      procedure Score (Q : Driver.Json.Node; Id : String; Rep : Natural; Board : String;
                       P : Driver.Brain.Programs.Program) is
         Objects : constant Driver.Json.Node := Driver.Json.Lookup (Q_Doc, Q, "objects");
         Count   : constant Natural := Driver.Json.Count (Q_Doc, Driver.Json.Lookup (Q_Doc, Q, "eyes"));
         View    : constant Driver.Observations.Camera_Id := View_Of (Q_Doc, Q);
         Target  : constant String := Driver.Json.Text (Q_Doc, Driver.Json.Lookup (Q_Doc, Q, "target"));
         Scene   : constant String := Driver.Json.Text (Q_Doc, Driver.Json.Lookup (Q_Doc, Q, "scene"));
         S       : Cached_Eyes;
         Table   : Driver.Brain.Names.Table;
         Glue    : constant Driver.Brain.Keyboard.Word_Vectors.Vector :=
           Driver.Brain.Keyboard.Name_Words (Keys (Board, Count));
         Names   : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Wants.Names_Of (P);
         Bound   : Driver.Brain.Wants.Binding_Maps.Map;

         package Step_Lists is new Ada.Containers.Indefinite_Ordered_Maps (String, Step_Vectors.Vector,
                                                                          "<", Step_Vectors."=");
         Steps_Of : Step_Lists.Map;

         function Object_Index (Object : String) return Natural is
         begin
            for K in 1 .. Driver.Json.Count (Q_Doc, Objects) loop
               if Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Objects, K)) = Object then
                  return K;
               end if;
            end loop;
            return 0;
         end Object_Index;
      begin
         S.Prefix := To_Unbounded_String (Id & "|" & Driver.Log.Image (Rep) & "|");
         S.Objects := Driver.Json.Count (Q_Doc, Objects);
         for E in 1 .. Count loop
            declare
               Eye : constant Driver.Observations.Camera_Id := Driver.Observations.Camera_Id (E);
            begin
               S.Pictures.Append (Eye_File (Q_Doc, Q, "eyes", Eye));
               S.Labels.Append (Labels_Of (Eye_File (Q_Doc, Q, "labels", Eye)));
               S.Shown.Append (All_Counts (S.Labels.Last_Element));
            end;
         end loop;
         for N of Names loop
            declare
               B : Driver.Brain.Names.Binding;
            begin
               S.Steps.Clear;
               Driver.Brain.Names.Bind (Table, S, View, N, Glue, B);
               Bound.Include (N, B);
               Steps_Of.Include (N, S.Steps);
            end;
         end loop;
         for N of Names loop
            declare
               B       : Driver.Brain.Names.Binding := Bound.Element (N);
               How     : Unbounded_String;
               Meant   : constant String := Meant_Of (N, Objects, How);
               K       : constant Natural := Object_Index (Meant);
               Got     : Natural := 0;   --  the object bound, if any
               Bound_T : Unbounded_String;
               Visible : Boolean := False;   --  some eye shows the meant object
               In_View : Boolean := False;
               Right   : Boolean;
               Cause   : Unbounded_String;
               Trace   : Unbounded_String;
               Steps   : constant Step_Vectors.Vector := Steps_Of (N);
            begin
               Driver.Brain.Names.Bind_Again (Table, Glue, N, B);
               if B.Kind = Driver.Brain.Names.To_Thing and then Natural (B.Thing) <= S.Objects then
                  Got := Natural (B.Thing);
                  Bound_T := To_Unbounded_String (Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Objects, Got)));
               elsif B.Kind = Driver.Brain.Names.To_Thing then
                  Bound_T := To_Unbounded_String ("nothing in particular");
               else
                  Bound_T := To_Unbounded_String ("unbound");
               end if;
               if K > 0 then
                  for E in 1 .. Count loop
                     Visible := Visible or else S.Shown (Driver.Observations.Camera_Id (E)) (K) > 0;
                  end loop;
                  In_View := S.Shown (View) (K) > 0;
               end if;
               Right := (if K > 0 then (if Visible then Got = K else B.Kind = Driver.Brain.Names.Unbound)
                         elsif Meant = "none" then B.Kind = Driver.Brain.Names.Unbound
                         else False);
               --  Why it went wrong: the name, the eye's box, the segmenting, or the binder's own rule.
               if not Right then
                  if Meant = "?" then
                     Cause := To_Unbounded_String ("wording: the name points at no one object");
                  elsif Meant = "none" then
                     Cause := To_Unbounded_String ("wording: the name points at nothing in the scene");
                  elsif K = 0 then
                     Cause := To_Unbounded_String ("scoring: unknown object " & Meant);
                  elsif Steps.Is_Empty then
                     Cause := To_Unbounded_String ("binder: no eye was asked (" & To_String (B.Account) & ")");
                  else
                     --  The binder ends at its last step: a patch it took as a
                     --  thing or as me, or the last eye that could not point it out.
                     declare
                        Last     : constant Step := Steps.Last_Element;
                        Boxed_On : constant Natural := Most_Object (Last.In_Box);
                     begin
                        if (for some St of Steps => St.Answer = Driver.Brain.Names.No_Answer) then
                           Cause := To_Unbounded_String ("service: an eye could not be asked");
                        elsif Last.Identified and then Last.Found /= Driver.Brain.Names.No_Patch then
                           if Boxed_On /= K then
                              Cause := To_Unbounded_String
                                ("box: eye" & Last.Eye'Image & " boxed "
                                 & (if Boxed_On = 0 then "no object"
                                    else Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Objects, Boxed_On))));
                           elsif Last.Found = Driver.Brain.Names.Part_Of_Me then
                              Cause := To_Unbounded_String ("segmentation: the patch is mostly me");
                           elsif Got = 0 then
                              Cause := To_Unbounded_String ("segmentation: the patch lies on no object");
                           else
                              Cause := To_Unbounded_String ("segmentation: the patch lies on another thing");
                           end if;
                        elsif (for some St of Steps => St.Identified and then Most_Object (St.In_Box) = K) then
                           Cause := To_Unbounded_String ("segmentation: no patch where an eye boxed it");
                        else
                           Cause := To_Unbounded_String ("box: no eye pointed it out");
                        end if;
                     end;
                  end if;
               end if;
               for St of Steps loop
                  Append (Trace, (if Length (Trace) > 0 then "," else "")
                          & "{""eye"":" & Driver.Log.Image (Natural (St.Eye)) & ",""answer"":"
                          & Driver.Json.Quote (Pointing_Text (St.Answer))
                          & ",""in_box"":" & Counts_Text (St.In_Box)
                          & (if St.Identified
                             then ",""found"":" & Driver.Json.Quote
                                    (case St.Found is
                                        when Driver.Brain.Names.A_Thing    => "a thing",
                                        when Driver.Brain.Names.Part_Of_Me => "part of me",
                                        when Driver.Brain.Names.No_Patch   => "no patch")
                                  & ",""pixels"":" & Driver.Log.Image (St.Pixels)
                                  & ",""patch_label"":" & Driver.Log.Image (St.Patch_Label)
                                  & ",""on_me"":" & Driver.Log.Image (St.On_Me)
                             else "")
                          & "}");
               end loop;
               --  Every eye's own answer, asked by the binder or not: how many
               --  pixels of the meant object it shows, and what its box is on.
               declare
                  Every : Unbounded_String;
               begin
                  for E in 1 .. Count loop
                     declare
                        Eye : constant Driver.Observations.Camera_Id := Driver.Observations.Camera_Id (E);
                        Key : constant String := To_String (S.Prefix) & N & "|" & Driver.Log.Image (E);
                        Has : constant Boolean := Wheres.Contains (Key);
                        A   : constant Driver.Brain.Names.Pointing :=
                          (if Has then Wheres (Key).Answer else Driver.Brain.Names.No_Answer);
                        On  : constant Natural :=
                          (if A = Driver.Brain.Names.Boxed then Most_Object (Counts_In (S.Labels (Eye), Wheres (Key).Where))
                           else 0);
                     begin
                        Append (Every, (if E > 1 then "," else "") & "{""eye"":" & Driver.Log.Image (E)
                                & ",""answer"":" & Driver.Json.Quote (Pointing_Text (A))
                                & ",""shows"":" & Driver.Log.Image (if K > 0 then S.Shown (Eye) (K) else 0)
                                & ",""box_on"":" & Driver.Json.Quote
                                  (if On = 0 then "" else Driver.Json.Text (Q_Doc, Driver.Json.Element (Q_Doc, Objects, On)))
                                & "}");
                     end;
                  end loop;
                  Append (Trace, "],""eyes"":[" & To_String (Every));
               end;
               Total := Total + 1;
               Right_Count := Right_Count + (if Right then 1 else 0);
               Ada.Text_IO.Put_Line
                 (Out_F, "{""id"":" & Driver.Json.Quote (Id) & ",""scene"":" & Driver.Json.Quote (Scene)
                  & ",""rep"":" & Driver.Log.Image (Rep) & ",""view"":" & Driver.Log.Image (Natural (View))
                  & ",""target"":" & Driver.Json.Quote (Target) & ",""name"":" & Driver.Json.Quote (N)
                  & ",""meant"":" & Driver.Json.Quote (Meant) & ",""meant_how"":" & Driver.Json.Quote (To_String (How))
                  & ",""visible"":" & (if Visible then "true" else "false")
                  & ",""in_view"":" & (if In_View then "true" else "false")
                  & ",""bound"":" & Driver.Json.Quote (To_String (Bound_T))
                  & ",""right"":" & (if Right then "true" else "false")
                  & ",""cause"":" & Driver.Json.Quote (To_String (Cause))
                  & ",""account"":" & Driver.Json.Quote (To_String (B.Account))
                  & ",""steps"":[" & To_String (Trace) & "]}");
            end;
         end loop;
      end Score;
   begin
      Driver.Json.Parse (Slurp (Argument (2)), Q_Doc, Ok, Why);
      if Ok then
         Driver.Json.Parse (Slurp (Argument (6)), M_Doc, Ok, Why);
      end if;
      if not Ok then
         Ada.Text_IO.Put_Line ("bind-score: " & To_String (Why));
         return;
      end if;
      Names_Node := Driver.Json.Lookup (M_Doc, Driver.Json.Root (M_Doc), "names");
      Words_Node := Driver.Json.Lookup (M_Doc, Driver.Json.Root (M_Doc), "words");
      --  The eyes' answers and the patches, as the run left them.
      Ada.Text_IO.Open (In_F, Ada.Text_IO.In_File, Argument (4));
      while not Ada.Text_IO.End_Of_File (In_F) loop
         Driver.Json.Parse (Ada.Text_IO.Get_Line (In_F), L_Doc, Ok, Why);
         if Ok then
            declare
               Root : constant Driver.Json.Node := Driver.Json.Root (L_Doc);
               A    : constant String := Driver.Json.Text (L_Doc, Driver.Json.Lookup (L_Doc, Root, "answer"));
            begin
               Wheres.Include
                 (Driver.Json.Text (L_Doc, Driver.Json.Lookup (L_Doc, Root, "id")) & "|"
                  & Driver.Log.Image (Natural (Driver.Json.Number (L_Doc, Driver.Json.Lookup (L_Doc, Root, "rep")))) & "|"
                  & Driver.Json.Text (L_Doc, Driver.Json.Lookup (L_Doc, Root, "name")) & "|"
                  & Driver.Log.Image (Natural (Driver.Json.Number (L_Doc, Driver.Json.Lookup (L_Doc, Root, "eye")))),
                  (Answer => (if A = "boxed" then Driver.Brain.Names.Boxed
                              elsif A = "not_here" then Driver.Brain.Names.Not_Here
                              else Driver.Brain.Names.No_Answer),
                   Where  => Box_Of (L_Doc, Driver.Json.Lookup (L_Doc, Root, "box"))));
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (In_F);
      Ada.Text_IO.Open (In_F, Ada.Text_IO.In_File, Argument (5));
      while not Ada.Text_IO.End_Of_File (In_F) loop
         Driver.Json.Parse (Ada.Text_IO.Get_Line (In_F), L_Doc, Ok, Why);
         if Ok then
            declare
               Root   : constant Driver.Json.Node := Driver.Json.Root (L_Doc);
               Labels : constant Driver.Json.Node := Driver.Json.Lookup (L_Doc, Root, "labels");
               E      : Patch_Entry;
            begin
               E.Ok := Driver.Json.Is_True (L_Doc, Driver.Json.Lookup (L_Doc, Root, "ok"));
               E.Pixels := Natural (Driver.Json.Number (L_Doc, Driver.Json.Lookup (L_Doc, Root, "pixels")));
               for J in 1 .. Driver.Json.Count (L_Doc, Labels) loop
                  declare
                     Pair : constant Driver.Json.Node := Driver.Json.Element (L_Doc, Labels, J);
                  begin
                     E.Counts (Natural (Driver.Json.Number (L_Doc, Driver.Json.Element (L_Doc, Pair, 1)))) :=
                       Natural (Driver.Json.Number (L_Doc, Driver.Json.Element (L_Doc, Pair, 2)));
                  end;
               end loop;
               Patches.Include (Driver.Json.Text (L_Doc, Driver.Json.Lookup (L_Doc, Root, "key")), E);
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (In_F);
      Ada.Text_IO.Create (Out_F, Ada.Text_IO.Out_File, Argument (7));
      Over_Answers (Q_Doc, Argument (3), Score'Access);
      Ada.Text_IO.Close (Out_F);
      Ada.Text_IO.Put_Line ("bound right:" & Right_Count'Image & " of" & Total'Image & " names");
   end Bind_Score_Run;

   procedure Where_Run is
      Picture : constant Driver.Images.Image := Read_Ppm (Argument (3));
   begin
      Configure (Argument (2));
      for A in 4 .. Argument_Count loop
         declare
            Found : Driver.Brain.Names.Pointing;
            Where : Driver.Brain.Names.Box;
            Why   : Unbounded_String;
         begin
            Driver.Brain.Service.Ask_Where (Picture, Argument (A), Found, Where, Why);
            Ada.Text_IO.Put_Line
              (Argument (A) & " | "
               & (if Found = Driver.Brain.Names.Boxed
                  then "box " & Driver.Log.Image (Where.Top_Left.U, 0) & " " & Driver.Log.Image (Where.Top_Left.V, 0)
                       & " " & Driver.Log.Image (Where.Bottom_Right.U, 0) & " "
                       & Driver.Log.Image (Where.Bottom_Right.V, 0)
                  else Driver.Brain.Names.Pointing'Image (Found) & " " & To_String (Why)));
         end;
      end loop;
   end Where_Run;

begin
   if Argument_Count >= 5 and then Argument (1) = "keyboard" then
      Over_Questions (Ask_Keyboard'Access);
   elsif Argument_Count >= 5 and then Argument (1) = "truncation" then
      Over_Questions (Ask_Truncation'Access);
   elsif Argument_Count >= 7 and then Argument (1) = "bindings" then
      Bindings_Run;
   elsif Argument_Count = 5 and then Argument (1) = "where-all" then
      Where_All_Run;
   elsif Argument_Count = 5 and then Argument (1) = "patches" then
      Patches_Run;
   elsif Argument_Count = 7 and then Argument (1) = "bind-score" then
      Bind_Score_Run;
   elsif Argument_Count >= 5 and then Argument (1) = "names" then
      Names_Run;
   elsif Argument_Count >= 6 and then Argument (1) = "stream" then
      Stream_Run;
   elsif Argument_Count >= 4 and then Argument (1) = "where" then
      Where_Run;
   elsif Argument_Count = 3 and then Argument (1) = "where-request" then
      Ada.Text_IO.Put_Line (Driver.Brain.Service.Where_Request
                              (Driver.Brain.Pictures.Data_Url (Read_Ppm (Argument (2))), Argument (3)));
   else
      Ada.Text_IO.Put_Line ("usage: brain_measure keyboard|truncation HOST:PORT QUESTIONS.json REPEATS OUT.jsonl"
                            & " | stream HOST:PORT QUESTIONS.json INDEX KEYBOARD LIMIT | where HOST:PORT IMAGE.ppm NAME...");
      Set_Exit_Status (Failure);
   end if;
end Brain_Measure;
