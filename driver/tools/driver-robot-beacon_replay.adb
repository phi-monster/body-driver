with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Log;
with Driver.Numerics;
with Driver.Robot.Kinematics.Beacon;

procedure Driver.Robot.Beacon_Replay is
   use Ada.Text_IO;
   package Beacon renames Driver.Robot.Kinematics.Beacon;
   package Token_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   File     : File_Type;
   Tokens   : Token_Vectors.Vector;
   At_Token : Positive := 1;

   function Next return String is
   begin
      At_Token := At_Token + 1;
      return Tokens (At_Token - 1);
   end Next;
   function Next_Real return Real is (Real'Value (Next));
   function Next_Natural return Natural is (Natural'Value (Next));
begin
   if Ada.Command_Line.Argument_Count /= 1 and then Ada.Command_Line.Argument_Count /= 5 then
      Put_Line (Standard_Error, "usage: beacon_replay DUMP [FX FY CX CY]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Open (File, In_File, Ada.Command_Line.Argument (1));
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
   if Next /= "sizes" then
      Put_Line (Standard_Error, "the dump has no sizes line");
      return;
   end if;
   declare
      Count  : constant Natural := Next_Natural;
      Width  : constant Positive := Next_Natural;
      Height : constant Positive := Next_Natural;
      Seen   : Beacon.Sighting_Array (1 .. Count);
      True_Lens : constant Boolean := Ada.Command_Line.Argument_Count = 5;
      Fx, Fy, Cx, Cy : Real := 0.0;
      Last_K : Natural := 0;

      procedure Fit (K : Positive) is
         Result : Beacon.Report;
         function Image (X : Real; Aft : Natural := 1) return String is (Driver.Log.Image (X, Aft));
      begin
         Beacon.Fit_Eye (Seen (1 .. K), Width, Height, Result);
         Put (Natural'Image (K) & " flicks: "
              & (if Result.Determined then "TOLD" else "untold")
              & ", used" & Result.Used'Image & ", focal " & Image (Result.Lens.Fx) & " x " & Image (Result.Lens.Fy)
              & ", centre " & Image (Result.Lens.Cx) & ", " & Image (Result.Lens.Cy)
              & ", scatter " & Image (Result.Scatter, 2));
         if True_Lens and then Result.Lens.Fx > 0.0 then
            Put (", off the truth: focal " & Image (100.0 * (Result.Lens.Fx / Fx - 1.0)) & " % x "
                 & Image (100.0 * (Result.Lens.Fy / Fy - 1.0)) & " %, centre " & Image (Result.Lens.Cx - Cx) & ", "
                 & Image (Result.Lens.Cy - Cy) & " px");
         end if;
         if not Result.Determined then
            Put (" (" & Ada.Strings.Unbounded.To_String (Result.Why) & ")");
         end if;
         New_Line;
         Last_K := K;
      end Fit;
   begin
      for K in 1 .. Count loop
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Seen (K).Carrier.Rotation (I, J) := Next_Real;
            end loop;
         end loop;
         for I in 1 .. 3 loop
            Seen (K).Carrier.Translation (I) := Next_Real;
         end loop;
         Seen (K).U := Next_Real;
         Seen (K).V := Next_Real;
         Seen (K).Sigma := Next_Real;
      end loop;
      if True_Lens then
         Fx := Real'Value (Ada.Command_Line.Argument (2));
         Fy := Real'Value (Ada.Command_Line.Argument (3));
         Cx := Real'Value (Ada.Command_Line.Argument (4));
         Cy := Real'Value (Ada.Command_Line.Argument (5));
      end if;
      Put_Line ("beacon dump of" & Count'Image & " flicks in a picture of" & Width'Image & " by" & Height'Image);
      --  The first sightings that make a fit at all (six) and on, every second one and the last.
      for K in 6 .. Count loop
         if K mod 2 = 0 or else K = Count then
            Fit (K);
         end if;
      end loop;
   end;
end Driver.Robot.Beacon_Replay;
