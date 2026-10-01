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

end Driver.Brain.Pictures;
