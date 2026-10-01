with Ada.Strings.Unbounded;
with Driver.Bytes;

package body Driver.Brain.Pictures is

   use Driver.Images;
   use type Driver.Bytes.Offset;

   Channels : constant := 3;   --  bytes per RGB pixel

   function Compose
     (Main   : Image;
      Rest : Driver.Brain.Names.Eye_Vectors.Vector;
      Images : not null access function (E : Driver.Brain.Names.Eye_Id) return Image)
      return Image
   is
      W      : constant Positive := Width (Main);
      Slot   : constant Natural := (if Rest.Is_Empty then 0 else W / Natural (Rest.Length));
      Strip  : Natural := 0;

      function Slot_Height (I : Image) return Natural is
        (if Is_Empty (I) or else Slot = 0 then 0 else Height (I) * Slot / Width (I));
   begin
      for E of Rest loop
         Strip := Natural'Max (Strip, Slot_Height (Images (E)));
      end loop;
      declare
         H   : constant Positive := Height (Main) + Strip;
         RGB : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (Channels * W * H)) := [others => 0];

         procedure Put (Column, Row : Natural; From : Image; C, R : Natural) is
            At_Byte : constant Driver.Bytes.Offset := Driver.Bytes.Offset (Channels * (Row * W + Column));
         begin
            RGB (At_Byte + 1 .. At_Byte + Channels) :=
              [Driver.Bytes.Byte (Red (From, C, R)), Driver.Bytes.Byte (Green (From, C, R)),
               Driver.Bytes.Byte (Blue (From, C, R))];
         end Put;
      begin
         for R in 0 .. Height (Main) - 1 loop
            for C in 0 .. W - 1 loop
               Put (C, R, Main, C, R);
            end loop;
         end loop;
         for K in Rest.First_Index .. Rest.Last_Index loop
            declare
               I  : constant Image := Images (Rest (K));
               Sh : constant Natural := Slot_Height (I);
               X0 : constant Natural := (K - Rest.First_Index) * Slot;
            begin
               for R in 0 .. Sh - 1 loop
                  for C in 0 .. Slot - 1 loop
                     Put (X0 + C, Height (Main) + R, I, C * Width (I) / Slot, R * Height (I) / Sh);
                  end loop;
               end loop;
            end;
         end loop;
         return Create (W, H, RGB);
      end;
   end Compose;

   --  BMP: a 14-byte file header and a 40-byte information header, then rows
   --  of blue, green, red, each padded to a multiple of four bytes. A
   --  negative height means the first row is the top one.

   File_Header : constant := 14;
   Info_Header : constant := 40;
   Row_Align   : constant := 4;
   Bits        : constant := 24;
   Byte_Base   : constant := 256;

   function Le (Value : Long_Long_Integer; Size : Positive) return String is
      R : String (1 .. Size);
      V : Long_Long_Integer := (if Value < 0 then Value + Long_Long_Integer (Byte_Base) ** Size else Value);
   begin
      for I in R'Range loop
         R (I) := Character'Val (V mod Byte_Base);
         V := V / Byte_Base;
      end loop;
      return R;
   end Le;

   function Bmp (I : Image) return String is
      W       : constant Positive := Width (I);
      H       : constant Positive := Height (I);
      Row     : constant Natural := Channels * W;
      Padded  : constant Natural := (Row + Row_Align - 1) / Row_Align * Row_Align;
      Pixels  : constant Natural := Padded * H;
      Offset  : constant Natural := File_Header + Info_Header;
      Two     : constant := 2;   --  the size of a 16-bit field
      Four    : constant := 4;   --  the size of a 32-bit field
      R       : String (1 .. Offset + Pixels) := [others => Character'Val (0)];
   begin
      R (1 .. Offset) :=
        "BM" & Le (Long_Long_Integer (Offset + Pixels), Four) & Le (0, Four) & Le (Long_Long_Integer (Offset), Four)
        & Le (Info_Header, Four) & Le (Long_Long_Integer (W), Four) & Le (-Long_Long_Integer (H), Four)
        & Le (1, Two) & Le (Bits, Two) & Le (0, Four) & Le (Long_Long_Integer (Pixels), Four)
        & Le (0, Four) & Le (0, Four) & Le (0, Four) & Le (0, Four);
      for Y in 0 .. H - 1 loop
         for X in 0 .. W - 1 loop
            declare
               At_Char : constant Positive := Offset + Y * Padded + Channels * X + 1;
            begin
               R (At_Char .. At_Char + Channels - 1) :=
                 Character'Val (Blue (I, X, Y)) & Character'Val (Green (I, X, Y)) & Character'Val (Red (I, X, Y));
            end;
         end loop;
      end loop;
      return R;
   end Bmp;

   Alphabet : constant String := "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

   function Base64 (Data : String) return String is
      use Ada.Strings.Unbounded;
      Group  : constant := 3;    --  three bytes become four characters of six bits
      Sextet : constant := 64;
      R      : Unbounded_String;
      I      : Natural := Data'First;
   begin
      while I <= Data'Last loop
         declare
            Have : constant Positive := Natural'Min (Group, Data'Last - I + 1);
            N    : Natural := 0;
         begin
            for K in 0 .. Group - 1 loop
               N := N * Byte_Base + (if K < Have then Character'Pos (Data (I + K)) else 0);
            end loop;
            for K in reverse 0 .. Group loop
               declare
                  Six : constant Natural := N / Sextet ** K mod Sextet;
               begin
                  Append (R, (if Group - K <= Have then Alphabet (Alphabet'First + Six) else '='));
               end;
            end loop;
            I := I + Group;
         end;
      end loop;
      return To_String (R);
   end Base64;

end Driver.Brain.Pictures;
