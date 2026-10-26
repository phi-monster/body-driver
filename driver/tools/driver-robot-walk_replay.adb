--  driver-robot-walk_replay WALK [FOCAL [MATCHES]]
--
--  A walk of the body under an arm (Driver.Robot.Kinematics.Base) aligned and adjusted again from the dump the driver
--  writes with BD_DUMP_FITS naming a folder: WALK is the dump's name without its ending (walk_ARM_BEAT), its pictures
--  beside it. FOCAL, where given, is the arm eye's focal length to begin from (the arm's fit's), at the picture's
--  middle; none, the stills' own search. The stills' lines as the driver logs them, then the calibration: the body
--  eye's frame on the arm's eye (its turn as a rotation matrix, row by row, and its place), both focal lengths, how it
--  fits, and the sigmas of its terms; then every point found at every still. MATCHES, where given, is a file of such
--  lines taken in place of the aligner's (a study's truth put there). A walk studied so takes seconds, not a boot.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Long_Float_Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Driver.Images;
with Driver.Images.Files;
with Driver.Log;
with Driver.Rig;
with Driver.Robot.Kinematics.Base;

procedure Driver.Robot.Walk_Replay is
   use Ada.Text_IO;
   package Base renames Driver.Robot.Kinematics.Base;
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
   procedure Put_Real (X : Real) is
   begin
      Put (" ");
      Ada.Long_Float_Text_IO.Put (X, Fore => 1, Aft => 6, Exp => 0);
   end Put_Real;
begin
   if Ada.Command_Line.Argument_Count not in 1 .. 3 then
      Put_Line (Standard_Error, "usage: walk_replay WALK [FOCAL [MATCHES]]  (WALK: the dump's name without .txt;"
                & " FOCAL 0: none; MATCHES: a file of match lines in place of the aligner's)");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   declare
      Name : constant String := Ada.Command_Line.Argument (1);
      File : File_Type;
   begin
      Open (File, In_File, Name & ".txt");
      Put_Line (Get_Line (File));   --  who
      while not End_Of_File (File) loop
         declare
            Line : constant String := Get_Line (File);
            I    : Natural := Line'First;
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
                     Tokens.Append (Line (I .. J - 1));
                     I := J;
                  end;
               end if;
            end loop;
         end;
      end loop;
      Close (File);
      Expect ("sizes");
      declare
         Count  : constant Natural := Next_Natural;
         Arm_N  : constant Natural := Next_Natural;
         Body_N : constant Natural := Next_Natural;
         Centre : Driver.Rig.Place;
         Arm_U, Arm_V, Body_U, Body_V : Real_Vectors.Vector;
         Still, Riding : Flag_Vectors.Vector;
         Stills : Body_Still_Vectors.Vector;
         Matches, Body_Matches : Match_Set_Vectors.Vector;
         Arm_Reference, Body_Reference : Driver.Images.Image;
         C : Driver.Rig.Calibration (Count, Arm_N, Body_N);
         K : Natural;
      begin
         Expect ("centre");
         Centre.U := Next_Real;
         Centre.V := Next_Real;
         Expect ("arm");
         for I in 1 .. Arm_N loop
            Arm_U.Append (Next_Real);
            Arm_V.Append (Next_Real);
            Still.Append (Next_Natural = 1);
            Riding.Append (Next_Natural = 1);
         end loop;
         Expect ("body");
         for J in 1 .. Body_N loop
            Body_U.Append (Next_Real);
            Body_V.Append (Next_Real);
         end loop;
         Expect ("stills");
         Arm_Reference := Driver.Images.Files.Read_PPM (Name & "_ref_arm.ppm");
         Body_Reference := Driver.Images.Files.Read_PPM (Name & "_ref_body.ppm");
         for S in 1 .. Count loop
            declare
               Prefix : constant String := Name & "_still_" & Trim (S'Image);
            begin
               Stills.Append
                 (Body_Still'(Beat       => Next_Natural,
                              Image      => Driver.Images.Files.Read_PPM (Prefix & "_arm.ppm"),
                              Body_Image => Driver.Images.Files.Read_PPM (Prefix & "_body.ppm")));
            end;
         end loop;
         if Ada.Command_Line.Argument_Count = 3 then
            --  The matches from a file in place of the aligner's: lines "match EYE STILL POINT U V SIGMA" as this tool
            --  prints them (a study's truth put in their place).
            for S in 1 .. Count loop
               declare
                  Empty : Match_Set := (Frame => S, others => <>);
                  Empty_Body : Match_Set := (Frame => S, others => <>);
               begin
                  for I in 1 .. Arm_N loop
                     Empty.Found.Append (False);
                     Empty.To_U.Append (0.0);
                     Empty.To_V.Append (0.0);
                     Empty.Sigma.Append (0.0);
                  end loop;
                  for J in 1 .. Body_N loop
                     Empty_Body.Found.Append (False);
                     Empty_Body.To_U.Append (0.0);
                     Empty_Body.To_V.Append (0.0);
                     Empty_Body.Sigma.Append (0.0);
                  end loop;
                  Matches.Append (Empty);
                  Body_Matches.Append (Empty_Body);
               end;
            end loop;
            declare
               Given : File_Type;
            begin
               Open (Given, In_File, Ada.Command_Line.Argument (3));
               while not End_Of_File (Given) loop
                  declare
                     Line  : constant String := Get_Line (Given);
                     Words : Token_Vectors.Vector;
                     I     : Natural := Line'First;
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
                              Words.Append (Line (I .. J - 1));
                              I := J;
                           end;
                        end if;
                     end loop;
                     if Natural (Words.Length) = 7 and then Words (1) = "match" then
                        declare
                           Eye   : constant Positive := Positive'Value (Words (2));
                           S     : constant Positive := Positive'Value (Words (3));
                           Point : constant Natural := Positive'Value (Words (4)) - 1;
                           procedure Put_In (Set : in out Match_Set) is
                           begin
                              Set.Found.Replace_Element (Point, True);
                              Set.To_U.Replace_Element (Point, Real'Value (Words (5)));
                              Set.To_V.Replace_Element (Point, Real'Value (Words (6)));
                              Set.Sigma.Replace_Element (Point, Real'Value (Words (7)));
                           end Put_In;
                        begin
                           if Eye = 1 then
                              Matches.Update_Element (S, Put_In'Access);
                           else
                              Body_Matches.Update_Element (S, Put_In'Access);
                           end if;
                        end;
                     end if;
                  end;
               end loop;
               Close (Given);
            end;
         else
            Base.Align_Stills ("the walk", Arm_Reference, Body_Reference, Arm_U, Arm_V, Body_U, Body_V, Still, Riding,
                               Stills, Matches, Body_Matches);
         end if;
         declare
            W : constant Real := Real (Driver.Images.Width (Arm_Reference));
            H : constant Real := Real (Driver.Images.Height (Arm_Reference));
            Start : constant Driver.Rig.Lens :=
              (Focal => (if Ada.Command_Line.Argument_Count >= 2 then Real'Value (Ada.Command_Line.Argument (2))
                         else 0.0),
               Cx => W / 2.0, Cy => H / 2.0);
         begin
            Base.Calibrate_Stills (Arm_U, Arm_V, Body_U, Body_V, Matches, Body_Matches, Start, Centre, C, K);
         end;
         Put_Line ((if C.Ok then "calibrated" else "NOT calibrated") & " from" & K'Image & " sightings of" & Count'Image
                   & " stills: focal lengths " & Driver.Log.Image (C.First_Lens.Focal, 2) & " and "
                   & Driver.Log.Image (C.Second_Lens.Focal, 2) & " px, misfit " & Driver.Log.Image (C.Misfit, 2)
                   & " sigmas at the residuals' scale " & Driver.Log.Image (C.Scale, 2) & "," & C.Kept'Image & " of"
                   & C.Used'Image & " within Z of it");
         Put ("frame");
         for A in 1 .. 3 loop
            for B in 1 .. 3 loop
               Put_Real (C.Second_On_First.Rotation (A, B));
            end loop;
         end loop;
         for A in 1 .. 3 loop
            Put_Real (C.Second_On_First.Translation (A));
         end loop;
         New_Line;
         Put ("lenses");
         Put_Real (C.First_Lens.Focal);
         Put_Real (C.Second_Lens.Focal);
         New_Line;
         Put ("sigmas");
         for T in 1 .. 8 loop
            Put_Real (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (0.0, C.Covariance (T, T))));
         end loop;
         New_Line;
         --  Every point found at every still, for a study of the matches against a truth: eye, still, point (from
         --  1), where it was found, the sigma stated.
         for Eye in 1 .. 2 loop
            for S in 1 .. Count loop
               declare
                  Set : constant Match_Set := (if Eye = 1 then Matches (S) else Body_Matches (S));
               begin
                  for I in 0 .. Natural (Set.Found.Length) - 1 loop
                     if Set.Found (I) then
                        Put ("match" & Eye'Image & S'Image & Natural'Image (I + 1));
                        Put_Real (Set.To_U (I));
                        Put_Real (Set.To_V (I));
                        Put_Real (Set.Sigma (I));
                        New_Line;
                     end if;
                  end loop;
               end;
            end loop;
         end loop;
      end;
   end;
end Driver.Robot.Walk_Replay;
