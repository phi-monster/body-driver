with Ada.Containers.Indefinite_Vectors;
with Ada.Environment_Variables;
with Ada.IO_Exceptions;
with Ada.Long_Float_Text_IO;
with Ada.Strings.Fixed;
with Ada.Text_IO;

package body Driver.Robot.Kinematics.Hand_Eye.Dumps is

   protected Counter is
      procedure Next (Number : out Positive);
   private
      Count : Natural := 0;
   end Counter;

   protected body Counter is
      procedure Next (Number : out Positive) is
      begin
         Count := Count + 1;
         Number := Count;
      end Next;
   end Counter;

   procedure Dump (Arms : Watched_Array; Width, Height : Positive; Name : String) is
      use Ada.Text_IO;
      File   : File_Type;
      Number : Positive;
      procedure Put_Real (X : Real) is
      begin
         Put (File, " ");
         Ada.Long_Float_Text_IO.Put (File, X, Fore => 1, Aft => Real'Digits + 1, Exp => 1);
      end Put_Real;
   begin
      if not Ada.Environment_Variables.Exists ("BD_DUMP_FITS") then
         return;
      end if;
      Counter.Next (Number);
      Create (File, Out_File, Ada.Environment_Variables.Value ("BD_DUMP_FITS") & "/handeye_" & Name & "_"
              & Ada.Strings.Fixed.Trim (Number'Image, Ada.Strings.Left) & ".txt");
      Put_Line (File, "hand_eye" & Natural'Image (Arms'Length) & Width'Image & Height'Image);
      for A of Arms loop
         Put_Line (File, "arm" & A.Joints'Image & A.Frames'Image & A.Tracks'Image & A.Sightings'Image
                   & Natural'Image (Natural (A.Covariance.Length)));
         for J of A.Joint loop
            Put (File, "joint");
            for I in 1 .. 3 loop
               Put_Real (J.W (I));
            end loop;
            for I in 1 .. 3 loop
               Put_Real (J.P (I));
            end loop;
            Put_Real (J.C);
            Put_Line (File, Natural'Image (Boolean'Pos (J.Slide)));
         end loop;
         for F in 1 .. A.Frames loop
            Put (File, "change");
            for J in 1 .. A.Joints loop
               Put_Real (A.Change (F, J));
            end loop;
            New_Line (File);
         end loop;
         Put (File, "visible");
         for X of A.Visible loop
            Put_Real (X);
         end loop;
         New_Line (File);
         Put (File, "cov");
         for X of A.Covariance loop
            Put_Real (X);
         end loop;
         New_Line (File);
         for T of A.Track loop
            Put (File, "track");
            Put_Real (T.U0);
            Put_Real (T.V0);
            Put_Line (File, T.Frame'Image);
         end loop;
         for S of A.Seen loop
            Put (File, "seen" & S.Track'Image & S.Frame'Image);
            Put_Real (S.U);
            Put_Real (S.V);
            Put_Real (S.Own);
            Put_Line (File, S.Own_Freedom'Image);
         end loop;
      end loop;
      Close (File);
   end Dump;

   function Load (Path : String) return Loaded is
      package Token_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
      use Ada.Text_IO;
      File   : File_Type;
      Tokens : Token_Vectors.Vector;
      At_Token : Positive := 1;
      function Next return String is
      begin
         if At_Token > Tokens.Last_Index then
            raise Ada.IO_Exceptions.Data_Error with "the dump ends early";
         end if;
         At_Token := At_Token + 1;
         return Tokens (At_Token - 1);
      end Next;
      procedure Expect (Word : String) is
      begin
         if Next /= Word then
            raise Ada.IO_Exceptions.Data_Error with "the dump has no " & Word & " where one stands";
         end if;
      end Expect;
      function Next_Real return Real is (Real'Value (Next));
      function Next_Natural return Natural is (Natural'Value (Next));
   begin
      Open (File, In_File, Path);
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
      Expect ("hand_eye");
      declare
         Count  : constant Natural := Next_Natural;
         Result : Loaded (Count);
      begin
         Result.Width := Next_Natural;
         Result.Height := Next_Natural;
         for K in 1 .. Count loop
            Expect ("arm");
            declare
               Joints    : constant Natural := Next_Natural;
               Frames    : constant Natural := Next_Natural;
               Tracks    : constant Natural := Next_Natural;
               Sightings : constant Natural := Next_Natural;
               Cov       : constant Natural := Next_Natural;
               A         : constant Watched_Access := new Watched (Joints, Frames, Tracks, Sightings);
            begin
               for J in 1 .. Joints loop
                  Expect ("joint");
                  for I in 1 .. 3 loop
                     A.Joint (J).W (I) := Next_Real;
                  end loop;
                  for I in 1 .. 3 loop
                     A.Joint (J).P (I) := Next_Real;
                  end loop;
                  A.Joint (J).C := Next_Real;
                  A.Joint (J).Slide := Next_Natural = 1;
               end loop;
               for F in 1 .. Frames loop
                  Expect ("change");
                  for J in 1 .. Joints loop
                     A.Change (F, J) := Next_Real;
                  end loop;
               end loop;
               Expect ("visible");
               for J in 1 .. Joints loop
                  A.Visible (J) := Next_Real;
               end loop;
               Expect ("cov");
               for I in 1 .. Cov loop
                  A.Covariance.Append (Next_Real);
               end loop;
               for T in 1 .. Tracks loop
                  Expect ("track");
                  A.Track (T).U0 := Next_Real;
                  A.Track (T).V0 := Next_Real;
                  A.Track (T).Frame := Next_Natural;
               end loop;
               for S in 1 .. Sightings loop
                  Expect ("seen");
                  A.Seen (S).Track := Next_Natural;
                  A.Seen (S).Frame := Next_Natural;
                  A.Seen (S).U := Next_Real;
                  A.Seen (S).V := Next_Real;
                  A.Seen (S).Own := Next_Real;
                  A.Seen (S).Own_Freedom := Next_Natural;
               end loop;
               Result.Arms (K) := A;
            end;
         end loop;
         return Result;
      end;
   end Load;

end Driver.Robot.Kinematics.Hand_Eye.Dumps;
