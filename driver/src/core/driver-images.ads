--  Camera images, pixel coordinates and pixel masks.
--
--  Pixel coordinates are continuous: U grows to the right, V grows down, and
--  (0, 0) is the top-left corner of the top-left pixel, so the pixel in
--  column C and row R covers [C, C + 1) x [R, R + 1) and has its centre at
--  (C + 0.5, R + 0.5). Columns and rows are numbered from 0.

with Ada.Containers.Indefinite_Holders;
with Driver.Bytes;

package Driver.Images is

   type Pixel is record
      U, V : Real := 0.0;
   end record;

   type Image is private;
   --  An immutable 8-bit RGB image; copies share nothing and cost a copy.

   No_Image : constant Image;

   function Create (Width, Height : Positive; RGB : Driver.Bytes.Byte_Array) return Image
     with Pre => RGB'Length = 3 * Width * Height;
   --  RGB holds rows top to bottom, each row left to right, three bytes per pixel.

   function Is_Empty (I : Image) return Boolean;
   function Width (I : Image) return Natural;
   function Height (I : Image) return Natural;

   function Red (I : Image; Column, Row : Natural) return Natural
     with Pre => Column < Width (I) and then Row < Height (I);
   function Green (I : Image; Column, Row : Natural) return Natural
     with Pre => Column < Width (I) and then Row < Height (I);
   function Blue (I : Image; Column, Row : Natural) return Natural
     with Pre => Column < Width (I) and then Row < Height (I);

   function Luma (I : Image; Column, Row : Natural) return Real
     with Pre => Column < Width (I) and then Row < Height (I);
   --  ITU-R BT.601 luma in [0, 255].

   procedure Luma (I : Image; Into : out Real_Array)
     with Pre => Into'Length = Width (I) * Height (I);
   --  Every pixel's luma, row after row (pixel Column, Row at index
   --  Row * Width + Column from Into'First), in one pass over the bytes: a
   --  frame costs a fraction of reading its pixels one at a time. The caller
   --  owns Into, so a frame-sized array need never sit on a task's stack.

   procedure Query (I : Image; Process : not null access procedure (RGB : Driver.Bytes.Byte_Array));
   --  Reads the raw RGB bytes in place.

   type Mask is private;
   --  A set of pixels of one image size.

   function Create (Width, Height : Natural) return Mask;
   function Width (M : Mask) return Natural;
   function Height (M : Mask) return Natural;
   function Contains (M : Mask; Column, Row : Natural) return Boolean
     with Pre => Column < Width (M) and then Row < Height (M);
   procedure Include (M : in out Mask; Column, Row : Natural; Value : Boolean := True)
     with Pre => Column < Width (M) and then Row < Height (M);
   function Count (M : Mask) return Natural;

private

   type Image is record
      Width, Height : Natural := 0;
      Data          : Driver.Bytes.Holders.Holder;
   end record;

   No_Image : constant Image := (Width => 0, Height => 0, Data => Driver.Bytes.Holders.Empty_Holder);

   type Bit_Array is array (Natural range <>) of Boolean with Pack;

   package Bit_Holders is new Ada.Containers.Indefinite_Holders (Bit_Array);

   type Mask is record
      Width, Height : Natural := 0;
      Bits          : Bit_Holders.Holder;
   end record;

end Driver.Images;
