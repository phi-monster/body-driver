with Ada.Unchecked_Deallocation;
with Interfaces;
with Driver.Clock;

package body Driver.Recording is

   use Ada.Streams.Stream_IO;
   use Driver.Bytes;
   use Interfaces;
   use type Offset;

   Header : constant String := "BDWIRE1" & ASCII.LF;

   Codes : constant array (Record_Kind) of Character :=
     [Robot_Message => 'R', Driver_Message => 'D', Robot_Text => 'r', Driver_Text => 'd',
      Connection => 'C', Service_Request => 'S', Service_Reply => 'T'];

   type Byte_Array_Access is access Byte_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Byte_Array, Byte_Array_Access);

   procedure Read_Exactly (F : File_Type; Item : out Byte_Array; Ok : out Boolean) is
      Last : Offset;
   begin
      Read (F, Item, Last);
      Ok := Last = Item'Last;
   end Read_Exactly;

   function Little_Endian (Data : Byte_Array) return Unsigned_64 is
      V : Unsigned_64 := 0;
   begin
      for I in reverse Data'Range loop
         V := Shift_Left (V, 8) or Unsigned_64 (Data (I));
      end loop;
      return V;
   end Little_Endian;

   procedure Open (R : in out Reader; Path : String; Ok : out Boolean) is
      Head : Byte_Array (1 .. Header'Length);
   begin
      Open (R.File, In_File, Path);
      Read_Exactly (R.File, Head, Ok);
      Ok := Ok and then To_String (Head) = Header;
      if not Ok then
         Close (R.File);
      end if;
   exception
      when Name_Error | Use_Error =>
         Ok := False;
   end Open;

   procedure Next
     (R           : in out Reader;
      Kind        : out Record_Kind;
      Nanoseconds : out Long_Long_Integer;
      Payload     : in out Buffer;
      Ok          : out Boolean)
   is
      Head : Byte_Array (1 .. 13);
   begin
      Payload.Clear;
      Kind := Connection;
      Nanoseconds := 0;
      if End_Of_File (R.File) then
         Ok := False;
         return;
      end if;
      Read_Exactly (R.File, Head, Ok);
      if not Ok then
         return;
      end if;
      Ok := False;
      for K in Record_Kind loop
         if Character'Val (Head (1)) = Codes (K) then
            Kind := K;
            Ok := True;
         end if;
      end loop;
      if not Ok then
         return;
      end if;
      Nanoseconds := Long_Long_Integer (Little_Endian (Head (2 .. 9)));
      declare
         Size : constant Offset := Offset (Little_Endian (Head (10 .. 13)));
         Data : Byte_Array_Access := new Byte_Array (1 .. Size);
      begin
         Read_Exactly (R.File, Data.all, Ok);
         if Ok then
            Payload.Append (Data.all);
         end if;
         Free (Data);
      end;
   end Next;

   procedure Close (R : in out Reader) is
   begin
      if Is_Open (R.File) then
         Close (R.File);
      end if;
   end Close;

   procedure Create (W : in out Writer; Path : String) is
   begin
      Create (W.File, Out_File, Path);
      Write (W.File, To_Bytes (Header));
   end Create;

   procedure Write (W : in out Writer; Kind : Record_Kind; Payload : Byte_Array) is
      Head : Byte_Array (1 .. 13);
      Ns   : constant Unsigned_64 := Unsigned_64 (Driver.Clock.Seconds * 1_000_000_000);
      Size : constant Unsigned_64 := Unsigned_64 (Payload'Length);
   begin
      Head (1) := Character'Pos (Codes (Kind));
      for I in 0 .. 7 loop
         Head (Offset (2 + I)) := Byte (Shift_Right (Ns, 8 * I) and 16#FF#);
      end loop;
      for I in 0 .. 3 loop
         Head (Offset (10 + I)) := Byte (Shift_Right (Size, 8 * I) and 16#FF#);
      end loop;
      Write (W.File, Head);
      Write (W.File, Payload);
   end Write;

   procedure Close (W : in out Writer) is
   begin
      if Is_Open (W.File) then
         Close (W.File);
      end if;
   end Close;

   function Is_Open (W : Writer) return Boolean is (Is_Open (W.File));

   --  File output is potentially blocking, so it happens outside the
   --  protected action: the protected object is only the lock.
   protected Lock is
      entry Seize;
      procedure Release;
   private
      Busy : Boolean := False;
   end Lock;

   protected body Lock is
      entry Seize when not Busy is
      begin
         Busy := True;
      end Seize;

      procedure Release is
      begin
         Busy := False;
      end Release;
   end Lock;

   Shared : Writer;

   procedure Start_Shared (Path : String) is
   begin
      Lock.Seize;
      Create (Shared, Path);
      Lock.Release;
   exception
      when others =>
         Lock.Release;
         raise;
   end Start_Shared;

   procedure Write_Shared (Kind : Record_Kind; Payload : Byte_Array) is
   begin
      Lock.Seize;
      if Is_Open (Shared) then
         Write (Shared, Kind, Payload);
      end if;
      Lock.Release;
   exception
      when others =>
         Lock.Release;
         raise;
   end Write_Shared;

   procedure Stop_Shared is
   begin
      Lock.Seize;
      Close (Shared);
      Lock.Release;
   end Stop_Shared;

end Driver.Recording;
