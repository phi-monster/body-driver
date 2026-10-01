package body Driver.Base64 is

   use Driver.Bytes;
   use type Driver.Bytes.Offset;

   Alphabet : constant String := "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

   Sextet : constant := 2 ** 6;
   --  Each output character carries six bits; three bytes make four characters.

   function Encode (Data : Byte_Array) return String is
      Result : String (1 .. Encoded_Length (Data'Length));
      Out_At : Positive := 1;
      I      : Offset := Data'First;
   begin
      while I <= Data'Last loop
         declare
            Have  : constant Natural := Natural (Offset'Min (3, Data'Last - I + 1));
            B0    : constant Natural := Natural (Data (I));
            B1    : constant Natural := (if Have > 1 then Natural (Data (I + 1)) else 0);
            B2    : constant Natural := (if Have > 2 then Natural (Data (I + 2)) else 0);
            Group : constant Natural := (B0 * 2 ** 8 + B1) * 2 ** 8 + B2;
         begin
            Result (Out_At) := Alphabet (Group / Sextet ** 3 + 1);
            Result (Out_At + 1) := Alphabet ((Group / Sextet ** 2) mod Sextet + 1);
            Result (Out_At + 2) := (if Have > 1 then Alphabet ((Group / Sextet) mod Sextet + 1) else '=');
            Result (Out_At + 3) := (if Have > 2 then Alphabet (Group mod Sextet + 1) else '=');
         end;
         Out_At := Out_At + 4;
         I := I + 3;
      end loop;
      return Result;
   end Encode;

end Driver.Base64;
