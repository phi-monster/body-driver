with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;

package body Driver.Images.Files is

   procedure Write_PPM (Name : String; I : Image) is
      F    : Ada.Streams.Stream_IO.File_Type;
      Head : constant String :=
        "P6" & ASCII.LF & Ada.Strings.Fixed.Trim (Natural'Image (Width (I)), Ada.Strings.Left) & " "
        & Ada.Strings.Fixed.Trim (Natural'Image (Height (I)), Ada.Strings.Left) & ASCII.LF & "255" & ASCII.LF;
      Bytes : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (Head'Length));

      procedure Put (RGB : Driver.Bytes.Byte_Array) is
      begin
         Ada.Streams.Stream_IO.Write (F, RGB);
      end Put;
   begin
      for K in Head'Range loop
         Bytes (Ada.Streams.Stream_Element_Offset (K - Head'First + 1)) := Driver.Bytes.Byte (Character'Pos (Head (K)));
      end loop;
      Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Name);
      Ada.Streams.Stream_IO.Write (F, Bytes);
      Query (I, Put'Access);
      Ada.Streams.Stream_IO.Close (F);
   end Write_PPM;

   function Read_PPM (Name : String) return Image is
      use type Ada.Streams.Stream_Element_Offset;
      F    : Ada.Streams.Stream_IO.File_Type;
      Size : Ada.Streams.Stream_Element_Offset;
   begin
      Ada.Streams.Stream_IO.Open (F, Ada.Streams.Stream_IO.In_File, Name);
      Size := Ada.Streams.Stream_Element_Offset (Ada.Streams.Stream_IO.Size (F));
      declare
         All_Bytes : Driver.Bytes.Byte_Array (1 .. Size);
         Last      : Ada.Streams.Stream_Element_Offset;
         At_Byte   : Ada.Streams.Stream_Element_Offset := 1;
         --  The header's next field: its whitespace skipped, the digits or letters up to the next.
         function Field return String is
            From : Ada.Streams.Stream_Element_Offset;
         begin
            while At_Byte <= Size
              and then Character'Val (All_Bytes (At_Byte)) in ' ' | ASCII.LF | ASCII.CR | ASCII.HT
            loop
               At_Byte := At_Byte + 1;
            end loop;
            From := At_Byte;
            while At_Byte <= Size
              and then Character'Val (All_Bytes (At_Byte)) not in ' ' | ASCII.LF | ASCII.CR | ASCII.HT
            loop
               At_Byte := At_Byte + 1;
            end loop;
            return Result : String (1 .. Natural (At_Byte - From)) do
               for K in Result'Range loop
                  Result (K) := Character'Val (All_Bytes (From + Ada.Streams.Stream_Element_Offset (K - 1)));
               end loop;
            end return;
         end Field;
      begin
         Ada.Streams.Stream_IO.Read (F, All_Bytes, Last);
         Ada.Streams.Stream_IO.Close (F);
         if Field /= "P6" then
            raise Constraint_Error with Name & " is no binary PPM";
         end if;
         declare
            W     : constant Positive := Positive'Value (Field);
            H     : constant Positive := Positive'Value (Field);
            Depth : constant Positive := Positive'Value (Field);
            First : constant Ada.Streams.Stream_Element_Offset := At_Byte + 1;   --  past the one whitespace byte
            Count : constant Ada.Streams.Stream_Element_Offset := Ada.Streams.Stream_Element_Offset (3 * W * H);
         begin
            if Depth /= 255 or else First + Count - 1 > Last then
               raise Constraint_Error with Name & " holds no eight-bit picture of its size";
            end if;
            return Create (W, H, All_Bytes (First .. First + Count - 1));
         end;
      end;
   end Read_PPM;

end Driver.Images.Files;
