with Ada.Streams.Stream_IO;
with Ada.Containers;
with Ada.Streams;
with Ada.Directories;
with Ada.Environment_Variables;
with Ada.Strings.Fixed;
with Ada.Long_Float_Text_IO;
with Interfaces;
package body Codec is
   Alphabet : constant String :=
     "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

   function Base64 (B : Buf) return String is
      N : constant Natural := Natural (B.Length);
      Out_Len : constant Natural := ((N + 2) / 3) * 4;
      S : String (1 .. Out_Len);
      P : Natural := 1;
      I : Natural := 0;
      V : Natural;
      B0, B1, B2 : Natural;
   begin
      while I < N loop
         B0 := Natural (B.Element (I));
         B1 := (if I + 1 < N then Natural (B.Element (I + 1)) else 0);
         B2 := (if I + 2 < N then Natural (B.Element (I + 2)) else 0);
         V := B0 * 65536 + B1 * 256 + B2;
         S (P) := Alphabet (V / 262144 mod 64 + 1);
         S (P + 1) := Alphabet (V / 4096 mod 64 + 1);
         S (P + 2) := (if I + 1 < N then Alphabet (V / 64 mod 64 + 1) else '=');
         S (P + 3) := (if I + 2 < N then Alphabet (V mod 64 + 1) else '=');
         P := P + 4;
         I := I + 3;
      end loop;
      return S;
   end Base64;

   function Base64_Of_String (S : String) return String is (Base64 (From_String (S)));

   function Hex_To_Bytes (H : String) return Buf is
      B : Buf;
      function Nib (C : Character) return Natural is
        (case C is
            when '0' .. '9' => Character'Pos (C) - Character'Pos ('0'),
            when 'a' .. 'f' => Character'Pos (C) - Character'Pos ('a') + 10,
            when 'A' .. 'F' => Character'Pos (C) - Character'Pos ('A') + 10,
            when others => 0);
      I : Natural := H'First;
   begin
      while I + 1 <= H'Last loop
         B.Append (U8 (Nib (H (I)) * 16 + Nib (H (I + 1))));
         I := I + 2;
      end loop;
      return B;
   end Hex_To_Bytes;

   procedure Put_U32 (B : in out Buf; V : Interfaces.Unsigned_32) is
      use Interfaces;
   begin
      B.Append (U8 (V and 16#FF#));
      B.Append (U8 (Shift_Right (V, 8) and 16#FF#));
      B.Append (U8 (Shift_Right (V, 16) and 16#FF#));
      B.Append (U8 (Shift_Right (V, 24) and 16#FF#));
   end Put_U32;

   procedure Put_U16 (B : in out Buf; V : Natural) is
   begin
      B.Append (U8 (V mod 256));
      B.Append (U8 (V / 256 mod 256));
   end Put_U16;

   function BMP24 (RGB : Buf; W, H : Natural) return Buf is
      Row : constant Natural := W * 3;
      Pad : constant Natural := (4 - Row mod 4) mod 4;
      Data : constant Natural := (Row + Pad) * H;
      B : Buf;
      Neg_H : constant Interfaces.Unsigned_32 := Interfaces.Unsigned_32'Mod (-Integer (H));   --  二补码的负高度
   begin
      B.Reserve_Capacity (Ada.Containers.Count_Type (54 + Data));
      Append (B, "BM");
      Put_U32 (B, Interfaces.Unsigned_32 (54 + Data));
      Put_U32 (B, 0);
      Put_U32 (B, 54);
      Put_U32 (B, 40);
      Put_U32 (B, Interfaces.Unsigned_32 (W));
      Put_U32 (B, Neg_H);
      Put_U16 (B, 1);
      Put_U16 (B, 24);
      Put_U32 (B, 0);
      Put_U32 (B, Interfaces.Unsigned_32 (Data));
      for K in 1 .. 4 loop
         Put_U32 (B, 0);
      end loop;
      for Y in 0 .. H - 1 loop
         for X in 0 .. W - 1 loop
            declare
               P : constant Natural := (Y * W + X) * 3;
            begin
               if P + 2 < Natural (RGB.Length) then
                  B.Append (RGB.Element (P + 2));
                  B.Append (RGB.Element (P + 1));
                  B.Append (RGB.Element (P));
               else
                  B.Append (0); B.Append (0); B.Append (0);
               end if;
            end;
         end loop;
         for K in 1 .. Pad loop
            B.Append (0);
         end loop;
      end loop;
      return B;
   end BMP24;

   procedure Write_File (Path : String; B : Buf) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      N : constant Natural := Natural (B.Length);
   begin
      Create (F, Out_File, Path);
      declare
         A : Ada.Streams.Stream_Element_Array (1 .. Ada.Streams.Stream_Element_Offset (N));
      begin
         for I in 0 .. N - 1 loop
            A (Ada.Streams.Stream_Element_Offset (I + 1)) := Ada.Streams.Stream_Element (B.Element (I));
         end loop;
         Write (F, A);
      end;
      Close (F);
   exception
      when others =>
         if Is_Open (F) then
            Close (F);
         end if;
   end Write_File;

   procedure Write_PGM (Path : String; Gray : Buf; W, H : Natural) is
      B : Buf;
   begin
      Append (B, "P5" & ASCII.LF & Img (W) & " " & Img (H) & ASCII.LF & "255" & ASCII.LF);
      for I in 0 .. W * H - 1 loop
         B.Append (if I < Natural (Gray.Length) then Gray.Element (I) else 0);
      end loop;
      Write_File (Path, B);
   end Write_PGM;

   procedure Write_BMP (Path : String; RGB : Buf; W, H : Natural) is
   begin
      Write_File (Path, BMP24 (RGB, W, H));
   end Write_BMP;

   procedure Make_Dir (Path : String) is
   begin
      if Path /= "" and then not Ada.Directories.Exists (Path) then
         Ada.Directories.Create_Path (Path);
      end if;
   exception
      when others => null;
   end Make_Dir;

   function Fmt (X : Long_Float; Aft : Natural := 3) return String is
      S : String (1 .. 40);
   begin
      if X /= X then
         return "nan";
      end if;
      --  排版协议:太大的数印成 inf(无量纲)
      if abs X > 1.0e15 then
         return (if X > 0.0 then "inf" else "-inf");
      end if;
      Ada.Long_Float_Text_IO.Put (S, X, Aft => Aft, Exp => 0);
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Fmt;

   function Img (N : Integer) return String is
     (Ada.Strings.Fixed.Trim (Integer'Image (N), Ada.Strings.Both));

   function Pad6 (N : Natural) return String is
      S : constant String := Img (N);
   begin
      if S'Length >= 6 then
         return S;
      end if;
      return [1 .. 6 - S'Length => '0'] & S;
   end Pad6;

   function Env (Name : String) return String is
   begin
      if Ada.Environment_Variables.Exists (Name) then
         return Ada.Environment_Variables.Value (Name);
      end if;
      return "";
   end Env;

   function Env_Nat (Name : String; Default : Natural) return Natural is
      V : constant String := Env (Name);
   begin
      if V = "" then
         return Default;
      end if;
      return Natural'Value (V);
   exception
      when others => return Default;
   end Env_Nat;
end Codec;
