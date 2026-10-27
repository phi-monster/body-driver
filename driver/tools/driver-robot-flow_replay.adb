--  driver-robot-flow_replay FLOW [TRUTH]
--
--  A fixed eye's flow fit (Driver.Robot.Kinematics.Flow_Eye, step 3) made again from the dump the driver writes with
--  BD_DUMP_FITS naming a folder: FLOW is the dump's folder (flow_EYE_BEAT: index.txt and every pose's lumas). Every
--  pose's evidence is taken from its pictures as the driver takes it (Flow_Eye.Evidence, the background the arm's
--  stills' median), and the eye fitted from it (Fit_Eye): the report's verdict, its lens, its camera in each arm's
--  reference frame (its turn as a rotation matrix, row by row, and its centre), how far it leaves the arms' points and
--  how widely the residuals scatter. TRUTH, where given, is a file of the eye as a simulator's truth has it, a line a
--  term: "lens FX FY CX CY", then for each arm "arm U" with its "rotation" (nine reals, row by row) and "centre"
--  (three, in the arm's unit); the model is judged there (Flow_Eye.Judge_At) beside the fit's own eye. A flow fit
--  studied so takes seconds, not a boot.

with Ada.Command_Line;
with Ada.Environment_Variables;
with Ada.Containers.Indefinite_Vectors;
with Ada.Long_Float_Text_IO;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Flow_Eye;

procedure Driver.Robot.Flow_Replay is
   use Ada.Text_IO;
   package Flow renames Driver.Robot.Kinematics.Flow_Eye;
   package KFit renames Driver.Robot.Kinematics.Fit;
   package Token_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   function Trim (S : String) return String is (Ada.Strings.Fixed.Trim (S, Ada.Strings.Both));

   Tokens   : Token_Vectors.Vector;
   At_Token : Positive := 1;
   function Next return String is
   begin
      At_Token := At_Token + 1;
      return Tokens (At_Token - 1);
   end Next;
   function Next_Real return Real is (Real'Value (Next));
   function Next_Natural return Natural is (Natural'Value (Next));
   procedure Expect (Word : String) is
   begin
      if Next /= Word then
         raise Constraint_Error with "the dump has no " & Word & " where it should";
      end if;
   end Expect;
   procedure Read_Tokens (Name : String) is
      File : File_Type;
   begin
      Tokens.Clear;
      At_Token := 1;
      Open (File, In_File, Name);
      while not End_Of_File (File) loop
         declare
            Line : constant String := Get_Line (File);
            From : Natural := Line'First;
         begin
            while From <= Line'Last loop
               if Line (From) = ' ' then
                  From := From + 1;
               else
                  declare
                     To : Natural := From;
                  begin
                     while To < Line'Last and then Line (To + 1) /= ' ' loop
                        To := To + 1;
                     end loop;
                     Tokens.Append (Line (From .. To));
                     From := To + 1;
                  end;
               end if;
            end loop;
         end;
      end loop;
      Close (File);
   end Read_Tokens;
   procedure Put_Real (X : Real) is
   begin
      Put (" ");
      Ada.Long_Float_Text_IO.Put (X, Fore => 1, Aft => 6, Exp => 0);
   end Put_Real;
   procedure Put_Pose (P : Driver.Numerics.Rigid) is
   begin
      Put ("rotation");
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            Put_Real (P.Rotation (I, J));
         end loop;
      end loop;
      Put (" centre");
      for I in 1 .. 3 loop
         Put_Real (P.Translation (I));
      end loop;
      New_Line;
   end Put_Pose;
begin
   if Ada.Command_Line.Argument_Count not in 1 .. 2 then
      Put_Line (Standard_Error, "usage: flow_replay FLOW [TRUTH]  (FLOW: the dump's folder flow_EYE_BEAT; TRUTH: the"
                & " eye"
                & " as the truth has it, lens and per arm rotation and centre)");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   declare
      Folder : constant String := Ada.Command_Line.Argument (1);
   begin
      Read_Tokens (Folder & "/index.txt");
      Expect ("flow");
      Expect ("eye");
      declare
         Eye    : constant Natural := Next_Natural;
         Beat   : Natural;
         Width, Height, Arms : Positive;
      begin
         Expect ("beat");
         Beat := Next_Natural;
         Expect ("width");
         Width := Next_Natural;
         Expect ("height");
         Height := Next_Natural;
         Expect ("arms");
         Arms := Next_Natural;
         Put_Line ("flow eye" & Eye'Image & " at beat" & Beat'Image & ":" & Width'Image & " by" & Height'Image
                   & " px," & Arms'Image & " arms");
         declare
            type Joints_Access is access KFit.Joint_Array;
            type Sigmas_Access is access Real_Array;
            type Poses_Access is access Flow.Pose_Array;
            Joints  : array (1 .. Arms) of Joints_Access;
            Sigmas  : array (1 .. Arms) of Sigmas_Access;
            Views   : array (1 .. Arms) of Poses_Access;
            In_Arms : Flow.Arm_Flow_Array (1 .. Arms);
            Result  : Flow.Report (Arms => Arms, Count => Flow.Terms_For (Arms));
            function Read_Luma (Name : String) return Flow.Luma_Access is
               F : Ada.Streams.Stream_IO.File_Type;
               L : constant Flow.Luma_Access := new Real_Array (1 .. Width * Height);
            begin
               Ada.Streams.Stream_IO.Open (F, Ada.Streams.Stream_IO.In_File, Folder & "/" & Name);
               Real_Array'Read (Ada.Streams.Stream_IO.Stream (F), L.all);
               Ada.Streams.Stream_IO.Close (F);
               return L;
            end Read_Luma;
            --  Every pixel's residuals at an eye, beside what the pixel is (its place, its gradient, its link, each
            --  joint's rate and sigma), into FLOW/NAME: which pixels the model does not hold there. The residuals as
            --  the fit lays them out (Flow_Eye.Lay_Out): a row for each joint up to the pixel's link, for pixels two
            --  joints move or more, none for the others (nan).
            procedure Write_Residuals (U : Positive; Name : String; Pose : Driver.Numerics.Rigid; Lens : KFit.Lens) is
               R : constant Real_Array :=
                 Flow.Residuals_At (Views (U).all, Joints (U).all, Width, Height, Pose, Lens, 0.0);
               F : File_Type;
               K : Natural := R'First;
            begin
               Create (F, Out_File, Folder & "/" & Name);
               for V in Views (U)'Range loop
                  declare
                     P : Flow.Pose_View renames Views (U) (V).all;
                  begin
                     for X in 1 .. P.Pixels loop
                        Put (F, V'Image & " " & Trim (P.U (X)'Image) & " " & Trim (P.V (X)'Image) & " "
                             & Trim (P.Gu (X)'Image) & " " & Trim (P.Gv (X)'Image) & " "
                             & Trim (P.Link (X)'Image) & " " & Trim (P.Back (X)'Image));
                        for J in 1 .. P.Joints loop
                           Put (F, " " & Trim (P.Rate (X, J)'Image) & " " & Trim (P.Sigma (X, J)'Image));
                           if P.Link (X) >= 2 and then J <= P.Link (X) then
                              Put (F, " " & Trim (R (K)'Image));
                              K := K + 1;
                           else
                              Put (F, " nan");
                           end if;
                        end loop;
                        New_Line (F);
                     end loop;
                  end;
               end loop;
               Close (F);
            end Write_Residuals;
         begin
            for U in 1 .. Arms loop
               Expect ("arm");
               declare
                  Id : constant Natural := Next_Natural;
                  N, P : Natural;
               begin
                  Expect ("joints");
                  N := Next_Natural;
                  Expect ("poses");
                  P := Next_Natural;
                  Joints (U) := new KFit.Joint_Array (1 .. N);
                  Sigmas (U) := new Real_Array (1 .. N);
                  for J in 1 .. N loop
                     Expect ("joint");
                     for I in 1 .. 3 loop
                        Joints (U) (J).W (I) := Next_Real;
                     end loop;
                     for I in 1 .. 3 loop
                        Joints (U) (J).P (I) := Next_Real;
                     end loop;
                     Joints (U) (J).C := Next_Real;
                     Joints (U) (J).Slide := Next_Natural = 1;
                     Sigmas (U) (J) := Next_Real;
                  end loop;
                  declare
                     type Pose_Data is record
                        Readings, Steps : Real_Array (1 .. N);
                        Noise           : Real;
                        Still           : Flow.Luma_Access;
                        Minus, Plus     : Flow.Luma_Array (1 .. N);
                     end record;
                     Data : array (1 .. P) of Pose_Data;
                     Background : Real_Array (1 .. Width * Height);
                     Column     : Real_Array (1 .. P);
                  begin
                     for V in 1 .. P loop
                        Expect ("pose");
                        if Next_Natural /= V then
                           raise Constraint_Error with "the poses are out of order";
                        end if;
                        Expect ("noise");
                        Data (V).Noise := Next_Real;
                        Expect ("readings");
                        for J in 1 .. N loop
                           Data (V).Readings (J) := Next_Real;
                        end loop;
                        Expect ("steps");
                        for J in 1 .. N loop
                           Data (V).Steps (J) := Next_Real;
                        end loop;
                        declare
                           Stem : constant String := "arm_" & Trim (U'Image) & "_pose_" & Trim (V'Image);
                        begin
                           Data (V).Still := Read_Luma (Stem & "_still.bin");
                           for J in 1 .. N loop
                              Data (V).Minus (J) := Read_Luma (Stem & "_minus_" & Trim (J'Image) & ".bin");
                              Data (V).Plus (J) := Read_Luma (Stem & "_plus_" & Trim (J'Image) & ".bin");
                           end loop;
                        end;
                     end loop;
                     --  The background as the driver takes it: the poses' stills' median at every pixel.
                     for K in Background'Range loop
                        for V in 1 .. P loop
                           Column (V) := Data (V).Still (K);
                        end loop;
                        for I in 2 .. P loop
                           for D in reverse 2 .. I loop
                              exit when Column (D - 1) <= Column (D);
                              declare
                                 Swap : constant Real := Column (D);
                              begin
                                 Column (D) := Column (D - 1);
                                 Column (D - 1) := Swap;
                              end;
                           end loop;
                        end loop;
                        Background (K) := (if P mod 2 = 1 then Column ((P + 1) / 2)
                                           else (Column (P / 2) + Column (P / 2 + 1)) / 2.0);
                     end loop;
                     Views (U) := new Flow.Pose_Array (1 .. P);
                     for V in 1 .. P loop
                        Views (U) (V) := Flow.Evidence (Data (V).Readings, Data (V).Still.all, Background,
                                                        Data (V).Noise, Data (V).Minus, Data (V).Plus, Data (V).Steps,
                                                        Width, Height);
                     end loop;
                  end;
                  --  BD_PROFILED: each pixel's link the model's (as the driver fits poses taken at turns a fit sized).
                  In_Arms (U) := (Poses => Views (U).all'Unchecked_Access, Joints => Joints (U).all'Unchecked_Access,
                                  Unit_Sigma => Sigmas (U).all'Unchecked_Access,
                                  Profiled => Ada.Environment_Variables.Exists ("BD_PROFILED"));
                  Put ("arm" & Id'Image & ":" & N'Image & " joints," & P'Image & " poses, pixels a pose:");
                  for V of Views (U).all loop
                     Put (V.Pixels'Image);
                  end loop;
                  New_Line;
               end;
            end loop;
            Flow.Fit_Eye (In_Arms, Width, Height, Result);
            Put_Line ("fit: " & (if Result.Determined then "told" else "untold") & ", used" & Result.Used'Image
                      & " of" & Result.Offered'Image & " pixels, scatter " & Trim (Result.Scatter'Image) & ", spread "
                      & Trim (Result.Spread'Image) & " px, widened " & Trim (Result.Widened'Image) & ": "
                      & Ada.Strings.Unbounded.To_String (Result.Why));
            Put ("lens:");
            Put_Real (Result.Lens.Fx);
            Put_Real (Result.Lens.Fy);
            Put_Real (Result.Lens.Cx);
            Put_Real (Result.Lens.Cy);
            New_Line;
            for U in 1 .. Arms loop
               Put ("pose" & U'Image & ": ");
               Put_Pose (Result.Poses (U));
               declare
                  Scatter  : Real;
                  In_Front : Natural;
               begin
                  Flow.Judge_At (Views (U).all, Joints (U).all, Width, Height, Result.Poses (U), Result.Lens, Scatter,
                                 In_Front);
                  Put_Line ("at the fit's eye, arm" & U'Image & ": scatter " & Trim (Scatter'Image) & ","
                            & In_Front'Image & " pixels in front");
               end;
               Write_Residuals (U, "residuals_fit_" & Trim (U'Image) & ".txt", Result.Poses (U), Result.Lens);
            end loop;
            if Ada.Command_Line.Argument_Count = 2 then
               Read_Tokens (Ada.Command_Line.Argument (2));
               Expect ("lens");
               declare
                  Lens  : KFit.Lens;
                  Truth : Flow.Rigid_Array (1 .. Arms);
               begin
                  Lens.Fx := Next_Real;
                  Lens.Fy := Next_Real;
                  Lens.Cx := Next_Real;
                  Lens.Cy := Next_Real;
                  for U in 1 .. Arms loop
                     Expect ("arm");
                     if Next_Natural /= U then
                        raise Constraint_Error with "the truth's arms are out of order";
                     end if;
                     Expect ("rotation");
                     for I in 1 .. 3 loop
                        for J in 1 .. 3 loop
                           Truth (U).Rotation (I, J) := Next_Real;
                        end loop;
                     end loop;
                     Expect ("centre");
                     for I in 1 .. 3 loop
                        Truth (U).Translation (I) := Next_Real;
                     end loop;
                     declare
                        Scatter  : Real;
                        In_Front : Natural;
                     begin
                        Flow.Judge_At
                          (Views (U).all, Joints (U).all, Width, Height, Truth (U), Lens, Scatter, In_Front);
                        Put_Line ("at the truth's eye, arm" & U'Image & ": scatter " & Trim (Scatter'Image) & ","
                                  & In_Front'Image & " pixels in front");
                     end;
                     Write_Residuals (U, "residuals_" & Trim (U'Image) & ".txt", Truth (U), Lens);
                  end loop;
                  --  The model's own rates at the truth's eye, with the noise the pictures state (Flow_Eye.Synthesize),
                  --  fitted again: where the fit finds the truth from them and not from the pictures, the pictures
                  --  depart from the model; where it does not find it from them either, the search does not.
                  declare
                     State : Long_Long_Integer := 1;
                     function Deviate return Real is
                        function Uniform return Real is
                        begin
                           State := (State * 48_271) mod 2_147_483_647;
                           return Real (State) / 2_147_483_647.0;
                        end Uniform;
                        U1 : constant Real := Real'Max (Uniform, 1.0e-12);
                        U2 : constant Real := Uniform;
                     begin
                        return Ada.Numerics.Long_Elementary_Functions.Sqrt
                                 (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (U1))
                          * Ada.Numerics.Long_Elementary_Functions.Cos (2.0 * Ada.Numerics.Pi * U2);
                     end Deviate;
                     Synthetic : Flow.Report (Arms => Arms, Count => Flow.Terms_For (Arms));
                  begin
                     for U in 1 .. Arms loop
                        Flow.Synthesize (Views (U).all, Joints (U).all, Width, Height, Truth (U), Lens, Deviate'Access);
                     end loop;
                     Flow.Fit_Eye (In_Arms, Width, Height, Synthetic);
                     Put_Line ("synthetic at the truth: " & (if Synthetic.Determined then "told" else "untold")
                               & ", scatter " & Trim (Synthetic.Scatter'Image) & ", spread "
                               & Trim (Synthetic.Spread'Image)
                               & " px: " & Ada.Strings.Unbounded.To_String (Synthetic.Why));
                     Put ("synthetic lens:");
                     Put_Real (Synthetic.Lens.Fx);
                     Put_Real (Synthetic.Lens.Fy);
                     Put_Real (Synthetic.Lens.Cx);
                     Put_Real (Synthetic.Lens.Cy);
                     New_Line;
                     for U in 1 .. Arms loop
                        Put ("synthetic pose" & U'Image & ": ");
                        Put_Pose (Synthetic.Poses (U));
                     end loop;
                  end;
               end;
            end if;
         end;
      end;
   end;
end Driver.Robot.Flow_Replay;
