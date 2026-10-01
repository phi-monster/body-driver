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
--
--  QUESTIONS.json is an array of objects:
--    id, task, kind (lift | turn | push | next_to | on), thing, other (the
--    second thing's key word, or ""), eyes (PPM files, the first is the large
--    picture), mounts ("fixed" or "arm <n>", one per eye), keyboards (names
--    below). Sampling comes from BL_BRAIN_SAMPLING, as in the driver.
--
--  Keyboards: Q (height), QH (height, heading), and the same with the
--  sentence about two things added: Q2 and QH2 (touching, above, below,
--  left, right).

with Ada.Command_Line;
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
with Driver.Bytes;
with Driver.Clock;
with Driver.Images;
with Driver.Json;
with Driver.Log;
with Driver.Observations;
with Driver.Robot;
with Driver.Services;

procedure Brain_Measure is

   use Ada.Strings.Unbounded;
   use Ada.Command_Line;
   use type Driver.Brain.Names.Pointing;
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
      M.Append ("how high it is above the surface it rests on; up lifts it off that surface");
      if Ada.Strings.Fixed.Index (Name, "QH") = Name'First then
         Q.Append ("heading");
         M.Append ("which way it points on the surface it rests on; up and down turn it one way or the other");
      end if;
      for E in 1 .. Eye_Count loop
         Eyes.Append (Driver.Observations.Camera_Id (E));
      end loop;
      if Name (Name'Last) = '2' then
         Two := [Driver.Action.Touching | Driver.Action.Above | Driver.Action.Below | Driver.Action.Left
                 | Driver.Action.Right => True, others => False];
      end if;
      return Driver.Brain.Keyboard.Choose (Q, M, [Driver.Action.Grasper => True, others => False],
                                           [others => True], True, Two, Eyes);
   end Keys;

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

   function Setting_Of (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String) return Setting is
      Eyes   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "eyes");
      Mounts : constant Driver.Json.Node := Driver.Json.Lookup (Doc, N, "mounts");
      Images : Driver.Observations.Image_Vectors.Vector;
      Facts  : Driver.Brain.Round.Facts;
      K      : constant Driver.Brain.Keyboard.Keyboard := Keys (Board, Driver.Json.Count (Doc, Eyes));
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
         if E > 1 then
            Facts.Strip.Append (Driver.Observations.Camera_Id (E));
         end if;
      end loop;
      Facts.View := 1;
      Facts.Happened := To_Unbounded_String (Driver.Brain.Round.First_Round);
      Facts.Instruction := To_Unbounded_String (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, N, "task")));
      Facts.Sheet := To_Unbounded_String (Driver.Brain.Keyboard.Sheet (K));
      declare
         function Image_Of (E : Driver.Brain.Names.Eye_Id) return Driver.Images.Image is (Images (E));
      begin
         return (Keys    => K,
                 Prompt  => To_Unbounded_String (Driver.Brain.Round.Prompt (Facts)),
                 Picture => Driver.Brain.Pictures.Compose (Images (1), Facts.Strip, Image_Of'Access));
      end;
   end Setting_Of;

   --  Every question of the file on every keyboard it names, Repeats times.
   procedure Each_Answer
     (Path    : String;
      Repeats : Natural;
      Visit   : not null access procedure
        (Doc : Driver.Json.Document; N : Driver.Json.Node; Board : String; Rep : Positive; S : Setting))
   is
      Doc : Driver.Json.Document;
      Ok  : Boolean;
      Why : Unbounded_String;
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
                  for Rep in 1 .. Repeats loop
                     Visit (Doc, N, Board, Rep, S);
                  end loop;
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
   elsif Argument_Count >= 6 and then Argument (1) = "stream" then
      Stream_Run;
   elsif Argument_Count >= 4 and then Argument (1) = "where" then
      Where_Run;
   else
      Ada.Text_IO.Put_Line ("usage: brain_measure keyboard|truncation HOST:PORT QUESTIONS.json REPEATS OUT.jsonl"
                            & " | stream HOST:PORT QUESTIONS.json INDEX KEYBOARD LIMIT | where HOST:PORT IMAGE.ppm NAME...");
      Set_Exit_Status (Failure);
   end if;
end Brain_Measure;
