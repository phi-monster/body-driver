with Ada.Text_IO;

package body Driver.Memory is

   function Resident_Kilobytes return Natural is
      use Ada.Text_IO;
      Status : File_Type;
      Key    : constant String := "VmRSS:";
   begin
      Open (Status, In_File, "/proc/self/status");
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
      return 0;
   exception
      when others =>
         if Is_Open (Status) then
            Close (Status);
         end if;
         return 0;
   end Resident_Kilobytes;

end Driver.Memory;
