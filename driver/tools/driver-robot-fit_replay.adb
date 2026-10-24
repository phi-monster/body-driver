--  driver-robot-fit_replay DUMP
--
--  The fit of an arm (Driver.Robot.Kinematics.Fit.Fit) made again from a dump of its inputs, which the driver writes
--  with BD_DUMP_FITS naming a folder: its stages' lines as the driver logs them, then what it found, the time it took.
--  A fit studied so takes the seconds it takes, not the replay of a whole recording up to it.

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Long_Float_Text_IO;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;

procedure Driver.Robot.Fit_Replay is
   use Ada.Text_IO;
   use Ada.Strings.Unbounded;
   package Fit renames Driver.Robot.Kinematics.Fit;
   package Token_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   File     : File_Type;
   Tokens   : Token_Vectors.Vector;
   At_Token : Positive := 1;
   Who      : Unbounded_String;

   function Next return String is
   begin
      At_Token := At_Token + 1;
      return Tokens (At_Token - 1);
   end Next;
   function Next_Real return Real is (Real'Value (Next));
   function Next_Natural return Natural is (Natural'Value (Next));
   function Next_Flag return Boolean is (Next_Natural = 1);
   procedure Expect (Word : String) is
   begin
      if Next /= Word then
         raise Constraint_Error with "the dump has no " & Word & " where it should";
      end if;
   end Expect;
begin
   if Ada.Command_Line.Argument_Count not in 1 | 2 then
      Put_Line (Standard_Error, "usage: fit_replay DUMP [SLACK]  (SLACK: a file of two numbers a joint, the slack to begin from)");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Open (File, In_File, Ada.Command_Line.Argument (1));
   while not End_Of_File (File) loop
      declare
         Line : constant String := Get_Line (File);
         I    : Natural := Line'First;
      begin
         if Line'Length >= 4 and then Line (Line'First .. Line'First + 3) = "who " then
            Who := To_Unbounded_String (Line (Line'First + 4 .. Line'Last));
         else
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
                     Tokens.Append (Line (I .. J - 1));
                     I := J;
                  end;
               end if;
            end loop;
         end if;
      end;
   end loop;
   Close (File);
   Expect ("sizes");
   declare
      Frames      : constant Natural := Next_Natural;
      N           : constant Natural := Next_Natural;
      S           : constant Natural := Next_Natural;
      Width       : constant Positive := Next_Natural;
      Height      : constant Positive := Next_Natural;
      Unit_Frames : constant Natural := Next_Natural;
      type Sightings_Access is access Fit.Sighting_Array;
      Changes     : Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
      Asked       : Fit.Flag_Matrix (1 .. Frames, 1 .. N);
      type Play_Access is access Fit.Play_Paths;
      Play        : Play_Access := null;   --  the readings' way between the keyframes, when dumped
      type Real_Array_Access is access Real_Array;
      Dumped_Slack : Real_Array_Access := null;   --  the slack the driver began from, when dumped
      Visible     : Real_Array (1 .. N);
      Sightings   : constant Sightings_Access := new Fit.Sighting_Array (1 .. S);
   begin
      Expect ("changes");
      for I in 1 .. Frames loop
         for J in 1 .. N loop
            Changes (I, J) := Next_Real;
         end loop;
      end loop;
      Expect ("asked");
      for I in 1 .. Frames loop
         for J in 1 .. N loop
            Asked (I, J) := Next_Flag;
         end loop;
      end loop;
      Expect ("visible");
      for J in 1 .. N loop
         Visible (J) := Next_Real;
      end loop;
      Expect ("sightings");
      for K in 1 .. S loop
         declare
            X : Fit.Sighting renames Sightings (K);
         begin
            X.Frame := Next_Natural;
            X.Track := Next_Natural;
            X.U0 := Next_Real;
            X.V0 := Next_Real;
            X.U := Next_Real;
            X.V := Next_Real;
            X.Own := Next_Real;
            X.Own_Freedom := Next_Natural;
            X.Off_U := Next_Real;
            X.Off_V := Next_Real;
            X.Off_Told := Next_Flag;
         end;
      end loop;
      Expect ("from");
      declare
         M      : constant Natural := Next_Natural;
         From   : Fit.Fit_Start (M);
         Joints : Fit.Joint_Array (1 .. N);
         Chain  : Fit.Chain_Order (1 .. N);
         L      : Fit.Lens;
         Report : Fit.Fit_Report;
         Began  : Ada.Calendar.Time;
      begin
         From.Valid := Next_Flag;
         From.Whole := Next_Flag;
         From.Order_Told := Next_Flag;
         From.Profiled_Over := Next_Natural;
         From.Focal_Within := Next_Real;
         From.Searched_Least := Next_Real;
         From.Searched_Most := Next_Real;
         From.L.Fx := Next_Real;
         From.L.Fy := Next_Real;
         From.L.Cx := Next_Real;
         From.L.Cy := Next_Real;
         From.L.K1 := Next_Real;
         From.L.K2 := Next_Real;
         for J in 1 .. M loop
            for A in From.Joints (J).W'Range loop
               From.Joints (J).W (A) := Next_Real;
            end loop;
            for A in From.Joints (J).P'Range loop
               From.Joints (J).P (A) := Next_Real;
            end loop;
            From.Joints (J).C := Next_Real;
            From.Joints (J).Slide := Next_Flag;
         end loop;
         for J in 1 .. M loop
            From.Chain (J) := Next_Natural;
         end loop;
         for J in 1 .. M loop
            for A in From.Slide_Turns (J)'Range loop
               From.Slide_Turns (J) (A) := Next_Real;
            end loop;
         end loop;
         for J in 1 .. M loop
            From.Turns_Told (J) := Next_Flag;
         end loop;
         --  The readings' way between the keyframes, when the dump has it (Fit's Play): its stretches' count, the
         --  stretches a row each, each keyframe's count of them a row each.
         if At_Token <= Tokens.Last_Index and then Tokens (At_Token) = "play" then
            Expect ("play");
            declare
               Stretches : constant Natural := Next_Natural;
            begin
               Play := new Fit.Play_Paths (Stretches => Stretches, Frames => Frames, Joints => N);
               for K in 1 .. Stretches loop
                  for J in 1 .. N loop
                     Play.Path (K, J) := Next_Real;
                  end loop;
               end loop;
               for F in 1 .. Frames loop
                  for J in 1 .. N loop
                     Play.Upto (F, J) := Next_Real;
                  end loop;
               end loop;
            end;
         end if;
         --  The slack the driver began from, when the dump has it (its count, two numbers a joint): an arm with a fit
         --  of its own reference begins from that fit's slack, not from one measured again.
         if At_Token <= Tokens.Last_Index and then Tokens (At_Token) = "slack" then
            Expect ("slack");
            declare
               Count : constant Natural := Next_Natural;
            begin
               Dumped_Slack := new Real_Array (1 .. Count);
               for K in 1 .. Count loop
                  Dumped_Slack (K) := Next_Real;
               end loop;
            end;
         end if;
         Began := Ada.Calendar.Clock;
         if Play = null then
            Fit.Fit (Changes, Asked, Visible, Sightings.all, Width, Height, Unit_Frames, Joints, Chain, L, Report,
                     Who => To_String (Who), From => From);
         else
            declare
               --  The slack the driver began from: the one a file names (two numbers a joint), else the dump's, else
               --  measured from the keyframes alone, as the driver measures it where the arm has no fit of its own.
               function Slack_Given return Real_Array is
                  F : File_Type;
                  R : Real_Array (1 .. 2 * N);
               begin
                  Open (F, In_File, Ada.Command_Line.Argument (2));
                  for K in R'Range loop
                     declare
                        V : Long_Float;
                     begin
                        Ada.Long_Float_Text_IO.Get (F, V);
                        R (K) := Real (V);
                     end;
                  end loop;
                  Close (F);
                  return R;
               end Slack_Given;
               Measured : constant Real_Array :=
                 (if Ada.Command_Line.Argument_Count = 2 then Slack_Given
                  elsif Dumped_Slack /= null then Dumped_Slack.all
                  else Fit.Measure_Slack (Changes, Sightings.all, Play.all));
            begin
               Put ((if Ada.Command_Line.Argument_Count = 2 then "slack given:"
                     elsif Dumped_Slack /= null then "slack dumped:" else "slack measured:"));
               for H of Measured loop
                  Put (H'Image);
               end loop;
               New_Line;
               Fit.Fit (Changes, Asked, Visible, Sightings.all, Width, Height, Unit_Frames, Joints, Chain, L, Report,
                        Who => To_String (Who), From => From, Play => Play.all, Slack_Start => Measured);
            end;
         end if;
         Put ("half slacks:");
         for H of Report.Slacks loop
            Put (H'Image);
         end loop;
         New_Line;
         Put ("slacks at the reference:");
         for H of Report.Slack_Places loop
            Put (H'Image);
         end loop;
         New_Line;
         Put_Line ("fitted " & Report.Fitted'Image & ", determined " & Report.Determined'Image & ", stage"
                   & Report.Stage'Image & ", order told " & Report.Order_Told'Image & ", flipped "
                   & Report.Flipped'Image);
         Put_Line ("why: " & To_String (Report.Why));
         Put ("chain:");
         for C of Chain loop
            Put (C'Image);
         end loop;
         New_Line;
         Put_Line ("lens:" & L.Fx'Image & L.Fy'Image & L.Cx'Image & L.Cy'Image & L.K1'Image & L.K2'Image);
         Put_Line ("focal sigma" & Report.Focal_Sigma'Image & " px; every sighting's median" & Report.All_Median_Px'Image
                   & " px over" & Report.All_Count'Image);
         Put_Line ("median" & Report.Median_Px'Image & " px over" & Report.Used'Image & " sightings, noise"
                   & Report.Sigma_Px'Image);
         Put ("stages, s:");
         for X of Report.Seconds loop
            Put (X'Image);
         end loop;
         New_Line;
         Put_Line ("in" & Duration'Image (Ada.Calendar."-" (Ada.Calendar.Clock, Began)) & " s");
      end;
   end;
end Driver.Robot.Fit_Replay;
