with Ada.Text_IO;

package body Driver.Memory is

   --  The number after Key in the file, in kilobytes; Absent when the file or the key is not there.
   function Kilobytes_Of (File_Name, Key : String; Absent : Natural) return Natural is
      use Ada.Text_IO;
      Status : File_Type;
   begin
      Open (Status, In_File, File_Name);
      while not End_Of_File (Status) loop
         declare
            Line : constant String := Get_Line (Status);
         begin
            if Line'Length > Key'Length and then Line (Line'First .. Line'First + Key'Length - 1) = Key then
               --  "VmRSS:    123456 kB": the digits between the key and the unit.
               declare
                  Value : Natural := 0;
                  Seen  : Boolean := False;
               begin
                  for C of Line (Line'First + Key'Length .. Line'Last) loop
                     if C in '0' .. '9' then
                        Value := Value * 10 + (Character'Pos (C) - Character'Pos ('0'));
                        Seen := True;
                     elsif Seen then
                        exit;
                     end if;
                  end loop;
                  Close (Status);
                  return Value;
               end;
            end if;
         end;
      end loop;
      Close (Status);
      return Absent;
   exception
      when others =>
         if Is_Open (Status) then
            Close (Status);
         end if;
         return Absent;
   end Kilobytes_Of;

   function Resident_Kilobytes return Natural is (Kilobytes_Of ("/proc/self/status", "VmRSS:", 0));

   function Available_Kilobytes return Natural is (Kilobytes_Of ("/proc/meminfo", "MemAvailable:", Natural'Last));

end Driver.Memory;
