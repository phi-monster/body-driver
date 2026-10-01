with Ada.Unchecked_Deallocation;

package body Driver.Bytes is

   use type Offset;

   procedure Free is new Ada.Unchecked_Deallocation (Byte_Array, Byte_Array_Access);

   function Length (B : Buffer) return Natural is (Natural (B.Last));

   procedure Clear (B : in out Buffer) is
   begin
      B.Last := 0;
   end Clear;

   procedure Reserve (B : in out Buffer; Needed : Offset) is
      Capacity : Offset := (if B.Data = null then 0 else B.Data'Length);
   begin
      if Needed <= Capacity then
         return;
      end if;
      if Capacity = 0 then
         Capacity := 1;
      end if;
      while Capacity < Needed loop
         Capacity := 2 * Capacity;
      end loop;
      declare
         Grown : constant Byte_Array_Access := new Byte_Array (1 .. Capacity);
      begin
         if B.Data /= null then
            Grown (1 .. B.Last) := B.Data (1 .. B.Last);
            Free (B.Data);
         end if;
         B.Data := Grown;
      end;
   end Reserve;

   procedure Append (B : in out Buffer; Data : Byte_Array) is
   begin
      if Data'Length = 0 then
         return;   --  an empty buffer may have no storage yet
      end if;
      Reserve (B, B.Last + Data'Length);
      B.Data (B.Last + 1 .. B.Last + Data'Length) := Data;
      B.Last := B.Last + Data'Length;
   end Append;

   procedure Append (B : in out Buffer; Item : Byte) is
   begin
      Reserve (B, B.Last + 1);
      B.Last := B.Last + 1;
      B.Data (B.Last) := Item;
   end Append;

   procedure Append (B : in out Buffer; Text : String) is
   begin
      Append (B, To_Bytes (Text));
   end Append;

   function Element (B : Buffer; Index : Positive) return Byte is (B.Data (Offset (Index)));

   procedure Query (B : Buffer; Process : not null access procedure (Data : Byte_Array)) is
   begin
      if B.Data = null then
         Process (Byte_Array'(1 .. 0 => 0));
      else
         Process (B.Data (1 .. B.Last));
      end if;
   end Query;

   function To_Array (B : Buffer) return Byte_Array is
     (if B.Data = null then Byte_Array'(1 .. 0 => 0) else B.Data (1 .. B.Last));

   function To_String (Data : Byte_Array) return String is
      Result : String (1 .. Data'Length);
   begin
      for I in Result'Range loop
         Result (I) := Character'Val (Data (Data'First + Offset (I) - 1));
      end loop;
      return Result;
   end To_String;

   function To_Bytes (Text : String) return Byte_Array is
      Result : Byte_Array (1 .. Text'Length);
   begin
      for I in Result'Range loop
         Result (I) := Character'Pos (Text (Text'First + Natural (I) - 1));
      end loop;
      return Result;
   end To_Bytes;

   overriding procedure Adjust (B : in out Buffer) is
   begin
      if B.Data /= null then
         B.Data := new Byte_Array'(B.Data (1 .. B.Last));
      end if;
   end Adjust;

   overriding procedure Finalize (B : in out Buffer) is
   begin
      Free (B.Data);
      B.Last := 0;
   end Finalize;

end Driver.Bytes;
