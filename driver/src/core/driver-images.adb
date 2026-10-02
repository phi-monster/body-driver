with Ada.Unchecked_Deallocation;

package body Driver.Images is

   use Driver.Bytes;
   use type Offset;

   function Create (Width, Height : Positive; RGB : Byte_Array) return Image is
     (Width => Width, Height => Height, Data => Holders.To_Holder (RGB));

   function Is_Empty (I : Image) return Boolean is (I.Width = 0 or else I.Height = 0);
   function Width (I : Image) return Natural is (I.Width);
   function Height (I : Image) return Natural is (I.Height);

   function Channel (I : Image; Column, Row, Index : Natural) return Natural is
      Ref  : constant Holders.Constant_Reference_Type := I.Data.Constant_Reference;
      Data : Byte_Array renames Ref.Element.all;
   begin
      return Natural (Data (Data'First + Offset (3 * (Row * I.Width + Column) + Index)));
   end Channel;

   function Red (I : Image; Column, Row : Natural) return Natural is (Channel (I, Column, Row, 0));
   function Green (I : Image; Column, Row : Natural) return Natural is (Channel (I, Column, Row, 1));
   function Blue (I : Image; Column, Row : Natural) return Natural is (Channel (I, Column, Row, 2));

   Red_Weight   : constant := 0.299;
   Green_Weight : constant := 0.587;
   Blue_Weight  : constant := 0.114;
   --  ITU-R BT.601.

   function Luma (I : Image; Column, Row : Natural) return Real is
     (Red_Weight * Real (Red (I, Column, Row)) + Green_Weight * Real (Green (I, Column, Row))
      + Blue_Weight * Real (Blue (I, Column, Row)));

   procedure Luma (I : Image; Into : out Real_Array) is
      procedure Fill (RGB : Byte_Array) is
      begin
         for K in 0 .. Into'Length - 1 loop
            declare
               B : constant Offset := RGB'First + Offset (3 * K);
            begin
               Into (Into'First + K) := Red_Weight * Real (RGB (B)) + Green_Weight * Real (RGB (B + 1))
                                        + Blue_Weight * Real (RGB (B + 2));
            end;
         end loop;
      end Fill;
   begin
      Query (I, Fill'Access);
   end Luma;

   procedure Query (I : Image; Process : not null access procedure (RGB : Byte_Array)) is
   begin
      if I.Data.Is_Empty then
         Process (Byte_Array'(1 .. 0 => 0));
      else
         Process (I.Data.Constant_Reference.Element.all);
      end if;
   end Query;

   type Bit_Array_Access is access Bit_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Bit_Array, Bit_Array_Access);

   function Create (Width, Height : Natural) return Mask is
      --  Cleared on the heap, as Pixels.Empty is: an aggregate of a frame's
      --  bits is a stack temporary, a megabyte for a 4K frame.
      Bits : Bit_Array_Access := new Bit_Array (0 .. Width * Height - 1);
   begin
      for B of Bits.all loop
         B := False;
      end loop;
      return Result : constant Mask := (Width => Width, Height => Height, Bits => Bit_Holders.To_Holder (Bits.all)) do
         Free (Bits);
      end return;
   end Create;

   function Width (M : Mask) return Natural is (M.Width);
   function Height (M : Mask) return Natural is (M.Height);

   function Contains (M : Mask; Column, Row : Natural) return Boolean is
     (M.Bits.Constant_Reference.Element (Row * M.Width + Column));

   procedure Include (M : in out Mask; Column, Row : Natural; Value : Boolean := True) is
   begin
      M.Bits.Reference.Element (Row * M.Width + Column) := Value;
   end Include;

   function Count (M : Mask) return Natural is
      Ref : constant Bit_Holders.Constant_Reference_Type := M.Bits.Constant_Reference;
      N   : Natural := 0;
   begin
      for B of Ref.Element.all loop
         if B then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Count;

end Driver.Images;
