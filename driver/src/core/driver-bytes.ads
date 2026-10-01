--  Byte storage: a growable buffer for assembling and receiving messages, and
--  a holder for immutable byte arrays such as image data.

with Ada.Containers.Indefinite_Holders;
with Ada.Finalization;
with Ada.Streams;

package Driver.Bytes is

   subtype Byte is Ada.Streams.Stream_Element;
   subtype Byte_Array is Ada.Streams.Stream_Element_Array;
   subtype Offset is Ada.Streams.Stream_Element_Offset;

   package Holders is new Ada.Containers.Indefinite_Holders (Byte_Array, Ada.Streams."=");
   --  An immutable heap copy of a byte array; read it in place through
   --  Holders.Constant_Reference to avoid copying large payloads.

   type Buffer is tagged private;
   --  Grows by doubling; copying a Buffer copies its contents.

   function Length (B : Buffer) return Natural;
   procedure Clear (B : in out Buffer);
   procedure Append (B : in out Buffer; Data : Byte_Array);
   procedure Append (B : in out Buffer; Item : Byte);
   procedure Append (B : in out Buffer; Text : String);

   function Element (B : Buffer; Index : Positive) return Byte
     with Inline, Pre => Index <= Length (B);

   procedure Query (B : Buffer; Process : not null access procedure (Data : Byte_Array));
   --  Calls Process with the contents (indexed from 1) without copying them.

   function To_Array (B : Buffer) return Byte_Array;
   function To_String (Data : Byte_Array) return String;
   function To_Bytes (Text : String) return Byte_Array;

private

   type Byte_Array_Access is access Byte_Array;

   type Buffer is new Ada.Finalization.Controlled with record
      Data : Byte_Array_Access;
      Last : Offset := 0;
   end record;

   overriding procedure Adjust (B : in out Buffer);
   overriding procedure Finalize (B : in out Buffer);

end Driver.Bytes;
