package body Bytes is
   function To_String (B : Buf; First, Len : Natural) return String is
      S : String (1 .. Len);
   begin
      for I in 0 .. Len - 1 loop
         if First + I < Natural (B.Length) then
            S (I + 1) := Character'Val (B (First + I));
         else
            S (I + 1) := ' ';
         end if;
      end loop;
      return S;
   end To_String;

   function From_String (S : String) return Buf is
      B : Buf;
   begin
      B.Reserve_Capacity (Ada.Containers.Count_Type (S'Length));
      for C of S loop
         B.Append (U8 (Character'Pos (C)));
      end loop;
      return B;
   end From_String;

   procedure Append (B : in out Buf; S : String) is
   begin
      for C of S loop
         B.Append (U8 (Character'Pos (C)));
      end loop;
   end Append;

   function Zeros (N : Natural) return Floats is (Filled (N, 0.0));

   function Filled (N : Natural; V : Long_Float) return Floats is
      F : Floats;
   begin
      for I in 1 .. N loop
         F.Append (V);
      end loop;
      return F;
   end Filled;
end Bytes;
