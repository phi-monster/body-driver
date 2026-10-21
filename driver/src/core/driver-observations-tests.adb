with Ada.Unchecked_Conversion;
with Interfaces;
with Driver.Bytes;
with Driver.Msgpack;
with Driver.Tests;

package body Driver.Observations.Tests is

   use Driver.Tests;
   use Driver.Msgpack;
   use type Interfaces.Unsigned_32;

   function To_Bits is new Ada.Unchecked_Conversion (Float, Interfaces.Unsigned_32);

   --  A 32-bit float as msgpack writes it (Put_Float writes 64 bits).
   procedure Put_Single (B : in out Driver.Bytes.Buffer; V : Float) is
      W : constant Interfaces.Unsigned_32 := To_Bits (V);
   begin
      B.Append (Driver.Bytes.Byte'(16#CA#));
      for K in reverse 0 .. 3 loop
         B.Append (Driver.Bytes.Byte (Interfaces.Shift_Right (W, 8 * K) and 16#FF#));
      end loop;
   end Put_Single;

   --  A msgpack-numpy array of three values.
   procedure Put_Nd (B : in out Driver.Bytes.Buffer; Dtype : String; Data : Driver.Bytes.Byte_Array) is
   begin
      Put_Map_Header (B, 4);
      Put_String (B, "nd"); Put_Boolean (B, True);
      Put_String (B, "type"); Put_String (B, Dtype);
      Put_String (B, "shape"); Put_Array_Header (B, 1); Put_Integer (B, 3);
      Put_String (B, "data"); Put_Binary (B, Data);
   end Put_Nd;

   --  The same three readings, 2048, 3000 and 10, in every form a body sends them.
   procedure Ints (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Array_Header (B, 3); Put_Integer (B, 2048); Put_Integer (B, 3000); Put_Integer (B, 10);
   end Ints;

   procedure Doubles (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Array_Header (B, 3); Put_Float (B, 2048.0); Put_Float (B, 3000.0); Put_Float (B, 10.0);
   end Doubles;

   procedure Singles (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Array_Header (B, 3); Put_Single (B, 2048.0); Put_Single (B, 3000.0); Put_Single (B, 10.0);
   end Singles;

   procedure Mixed (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Array_Header (B, 3); Put_Integer (B, 2048); Put_Float (B, 3000.0); Put_Single (B, 10.0);
   end Mixed;

   procedure Nd_Shorts (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Nd (B, "<i2", [0, 8, 184, 11, 10, 0]);
   end Nd_Shorts;

   procedure Nd_Singles (B : in out Driver.Bytes.Buffer) is
   begin
      Put_Nd (B, "<f4", [0, 0, 0, 69, 0, 128, 59, 69, 0, 0, 32, 65]);
   end Nd_Singles;

   --  The precision the layout and the parser give the one group of an
   --  observation of one camera (2 x 2 pixels) and the readings Put_Readings writes.
   function Precision_After (Put_Readings : not null access procedure (B : in out Driver.Bytes.Buffer))
                             return Natural
   is
      B      : Driver.Bytes.Buffer;
      Pixels : constant Driver.Bytes.Byte_Array (1 .. 12) := [others => 0];
      Doc    : Document;
      Ok     : Boolean;
      L      : Layout;
      O      : Observation;
   begin
      Put_Map_Header (B, 2);
      Put_String (B, "state");
      Put_Map_Header (B, 1);
      Put_String (B, "arm");
      Put_Readings (B);
      Put_String (B, "vision");
      Put_Map_Header (B, 1);
      Put_String (B, "cam");
      Put_Map_Header (B, 4);
      Put_String (B, "nd"); Put_Boolean (B, True);
      Put_String (B, "type"); Put_String (B, "|u1");
      Put_String (B, "shape"); Put_Array_Header (B, 3); Put_Integer (B, 2); Put_Integer (B, 2); Put_Integer (B, 3);
      Put_String (B, "data"); Put_Binary (B, Pixels);
      Decode (Driver.Bytes.To_Array (B), Doc, Ok);
      Check (Ok, "the observation decodes");
      Recognize (Doc, Root (Doc), L, Ok);
      Check (Ok, "the layout is recognized");
      Check (Natural (L.Groups.Length) = 1, "one group, got" & Natural'Image (Natural (L.Groups.Length)));
      Parse (Doc, Root (Doc), L, 0, O);
      Check (Natural (O.Precisions.Length) = 1, "one precision per group, got"
             & Natural'Image (Natural (O.Precisions.Length)));
      declare
         R : constant Real_Array := O.Readings.Element (1);
      begin
         Check (R'Length = 3 and then R (R'First) = 2048.0, "the readings are read whatever their form");
      end;
      return Precision_Of (O, 1);
   end Precision_After;

   --  Readings keep the format they came in: integers (a servo's ticks: the
   --  SO-101 pair of R0, whose bridge sent them as floats and cost every joint
   --  some forty doublings of a push below one tick) are exact, floats have
   --  the bits of their width, one integer among floats makes the group exact,
   --  and a rig's Reals have the Reals' bits.
   procedure Readings_Keep_Their_Wire_Format is
      Rig : Observation;
   begin
      Check (Precision_After (Ints'Access) = 0,
             "integers are exact, got" & Natural'Image (Precision_After (Ints'Access)) & " bits");
      Check (Precision_After (Doubles'Access) = Real'Machine_Mantissa,
             "64-bit floats have the Reals' bits, got" & Natural'Image (Precision_After (Doubles'Access)));
      Check (Precision_After (Singles'Access) = Float'Machine_Mantissa,
             "32-bit floats have 24 bits, got" & Natural'Image (Precision_After (Singles'Access)));
      Check (Precision_After (Mixed'Access) = 0,
             "an integer among floats makes the group exact, got" & Natural'Image (Precision_After (Mixed'Access)));
      Check (Precision_After (Nd_Shorts'Access) = 0,
             "an integer ndarray is exact, got" & Natural'Image (Precision_After (Nd_Shorts'Access)));
      Check (Precision_After (Nd_Singles'Access) = Float'Machine_Mantissa,
             "a 32-bit float ndarray has 24 bits, got" & Natural'Image (Precision_After (Nd_Singles'Access)));
      Rig.Readings.Append (Real_Array'[1 => 1.0]);
      Check (Precision_Of (Rig, 1) = Real'Machine_Mantissa, "a rig's Reals have the Reals' bits");
   end Readings_Keep_Their_Wire_Format;

   procedure Register is
   begin
      Driver.Tests.Register ("observations.precision", "a reading's wire format is lost: a servo's integer ticks read "
                             & "as floats start the recognition's pushes at the float's resolution, some forty "
                             & "doublings below one tick (R0, 10-11)", Readings_Keep_Their_Wire_Format'Access);
   end Register;

end Driver.Observations.Tests;
