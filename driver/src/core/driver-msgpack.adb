with Ada.Unchecked_Conversion;
with Interfaces;

package body Driver.Msgpack is

   use Driver.Bytes;
   use Interfaces;
   use type Offset;
   use type Byte;

   function U32_To_F32 is new Ada.Unchecked_Conversion (Unsigned_32, Float);
   function U64_To_F64 is new Ada.Unchecked_Conversion (Unsigned_64, Long_Float);
   function F64_To_U64 is new Ada.Unchecked_Conversion (Long_Float, Unsigned_64);
   function To_I8 is new Ada.Unchecked_Conversion (Unsigned_8, Integer_8);
   function To_I16 is new Ada.Unchecked_Conversion (Unsigned_16, Integer_16);
   function To_I32 is new Ada.Unchecked_Conversion (Unsigned_32, Integer_32);
   function To_I64 is new Ada.Unchecked_Conversion (Unsigned_64, Integer_64);
   function To_U64 is new Ada.Unchecked_Conversion (Integer_64, Unsigned_64);

   function Signed (V : Unsigned_64; Width : Natural) return Long_Long_Integer is
     (case Width is
         when 1      => Long_Long_Integer (To_I8 (Unsigned_8 (V))),
         when 2      => Long_Long_Integer (To_I16 (Unsigned_16 (V))),
         when 4      => Long_Long_Integer (To_I32 (Unsigned_32 (V))),
         when others => Long_Long_Integer (To_I64 (V)));
   --  Reinterprets the two's complement bits; a value conversion would raise
   --  Constraint_Error on every negative number.

   --  IEEE 754 binary16 (numpy f2): sign, 5 exponent bits biased by 15, 10 fraction bits.
   Half_Fraction_Bits : constant := 10;
   Half_Exponent_Bits : constant := 5;
   Half_Exponent_All  : constant := 2 ** Half_Exponent_Bits - 1;
   Half_Bias          : constant := 2 ** (Half_Exponent_Bits - 1) - 1;

   function Half (H : Unsigned_64) return Real is
      Negative : constant Boolean := (H and 2 ** (Half_Fraction_Bits + Half_Exponent_Bits)) /= 0;
      E : constant Integer := Integer (Shift_Right (H, Half_Fraction_Bits) and Half_Exponent_All);
      M : constant Integer := Integer (H and (2 ** Half_Fraction_Bits - 1));
      V : Real;
   begin
      if E = Half_Exponent_All then
         V := U64_To_F64 (if M = 0 then 16#7FF0_0000_0000_0000# else 16#7FF8_0000_0000_0000#);
      elsif E = 0 then
         V := Real'Scaling (Real (M), 1 - Half_Bias - Half_Fraction_Bits);
      else
         V := Real'Scaling (Real (2 ** Half_Fraction_Bits + M), E - Half_Bias - Half_Fraction_Bits);
      end if;
      return (if Negative then -V else V);
   end Half;

   procedure Decode (Data : Byte_Array; Doc : out Document; Ok : out Boolean) is
      Pos : Offset := Data'First;
      Bad : Boolean := False;

      function Take return Unsigned_8 is
      begin
         if Pos > Data'Last then
            Bad := True;
            return 0;
         end if;
         Pos := Pos + 1;
         return Unsigned_8 (Data (Pos - 1));
      end Take;

      function Take_Unsigned (Width : Natural) return Unsigned_64 is
         V : Unsigned_64 := 0;
      begin
         for K in 1 .. Width loop
            V := Shift_Left (V, 8) or Unsigned_64 (Take);
         end loop;
         return V;
      end Take_Unsigned;

      procedure Skip (Length : Natural; R : in out Node_Record) is
      begin
         R.First := Natural (Pos - Data'First);
         R.Size := Length;
         if Offset (Length) > Data'Last - Pos + 1 then
            Bad := True;
         else
            Pos := Pos + Offset (Length);
         end if;
      end Skip;

      function Parse return Node;

      procedure Children (R : in out Node_Record; Length : Natural) is
      begin
         R.Kids := Natural (Doc.Children.Length) + 1;
         R.Kids_Count := Length;
         Doc.Children.Append (No_Node, Ada.Containers.Count_Type (Length));
         for K in 1 .. Length loop
            exit when Bad;
            Doc.Children.Replace_Element (R.Kids + K - 1, Parse);
         end loop;
      end Children;

      function Parse return Node is
         Me : Node;
         R  : Node_Record;
         B  : Unsigned_8;
      begin
         Doc.Nodes.Append (R);
         Me := Node (Doc.Nodes.Length);
         B := Take;
         if Bad then
            return Me;
         end if;
         case B is
            when 16#00# .. 16#7F# =>
               R.Of_Kind := Integer_Value; R.Int := Long_Long_Integer (B);
            when 16#E0# .. 16#FF# =>
               R.Of_Kind := Integer_Value; R.Int := Long_Long_Integer (B) - 256;
            when 16#A0# .. 16#BF# =>
               R.Of_Kind := String_Value; Skip (Natural (B and 16#1F#), R);
            when 16#90# .. 16#9F# =>
               R.Of_Kind := Array_Value; Children (R, Natural (B and 16#0F#));
            when 16#80# .. 16#8F# =>
               R.Of_Kind := Map_Value; Children (R, 2 * Natural (B and 16#0F#));
            when 16#C0# =>
               R.Of_Kind := Nil_Value;
            when 16#C2# | 16#C3# =>
               R.Of_Kind := Boolean_Value; R.Flag := B = 16#C3#;
            when 16#C4# | 16#C5# | 16#C6# =>
               R.Of_Kind := Binary_Value;
               Skip (Natural (Take_Unsigned (if B = 16#C4# then 1 elsif B = 16#C5# then 2 else 4)), R);
            when 16#C7# | 16#C8# | 16#C9# =>
               declare
                  Length : constant Natural :=
                    Natural (Take_Unsigned (if B = 16#C7# then 1 elsif B = 16#C8# then 2 else 4));
                  Ext_Type : constant Unsigned_8 := Take;
                  pragma Unreferenced (Ext_Type);
               begin
                  R.Of_Kind := Extension_Value; Skip (Length, R);
               end;
            when 16#CA# =>
               R.Of_Kind := Float_Value; R.Flt := Real (U32_To_F32 (Unsigned_32 (Take_Unsigned (4))));
            when 16#CB# =>
               R.Of_Kind := Float_Value; R.Flt := U64_To_F64 (Take_Unsigned (8));
            when 16#CC# | 16#CD# | 16#CE# =>
               R.Of_Kind := Integer_Value;
               R.Int := Long_Long_Integer (Take_Unsigned (if B = 16#CC# then 1 elsif B = 16#CD# then 2 else 4));
            when 16#CF# =>
               declare
                  V : constant Unsigned_64 := Take_Unsigned (8);
               begin
                  --  Above the signed range the value is kept as a float, not clipped.
                  if V <= Unsigned_64 (Long_Long_Integer'Last) then
                     R.Of_Kind := Integer_Value; R.Int := Long_Long_Integer (V);
                  else
                     R.Of_Kind := Float_Value; R.Flt := Real (V);
                  end if;
               end;
            when 16#D0# | 16#D1# | 16#D2# | 16#D3# =>
               declare
                  Width : constant Natural :=
                    (case B is when 16#D0# => 1, when 16#D1# => 2, when 16#D2# => 4, when others => 8);
               begin
                  R.Of_Kind := Integer_Value; R.Int := Signed (Take_Unsigned (Width), Width);
               end;
            when 16#D4# .. 16#D8# =>
               declare
                  Length : constant Natural :=
                    (case B is when 16#D4# => 1, when 16#D5# => 2, when 16#D6# => 4, when 16#D7# => 8,
                               when others => 16);
                  Ext_Type : constant Unsigned_8 := Take;
                  pragma Unreferenced (Ext_Type);
               begin
                  R.Of_Kind := Extension_Value; Skip (Length, R);
               end;
            when 16#D9# | 16#DA# | 16#DB# =>
               R.Of_Kind := String_Value;
               Skip (Natural (Take_Unsigned (if B = 16#D9# then 1 elsif B = 16#DA# then 2 else 4)), R);
            when 16#DC# | 16#DD# =>
               R.Of_Kind := Array_Value; Children (R, Natural (Take_Unsigned (if B = 16#DC# then 2 else 4)));
            when 16#DE# | 16#DF# =>
               R.Of_Kind := Map_Value; Children (R, 2 * Natural (Take_Unsigned (if B = 16#DE# then 2 else 4)));
            when 16#C1# =>
               Bad := True;
         end case;
         Doc.Nodes.Replace_Element (Positive (Me), R);
         return Me;
      end Parse;

      Top : Node;
   begin
      Doc := (Raw => Holders.To_Holder (Data), Nodes => <>, Children => <>);
      Top := Parse;
      Ok := not Bad and then Top = 1 and then Pos = Data'Last + 1;
      if not Ok then
         Doc := (Raw => Holders.Empty_Holder, Nodes => <>, Children => <>);
      end if;
   end Decode;

   function Valid (Doc : Document; N : Node) return Boolean is
     (N /= No_Node and then Natural (N) <= Natural (Doc.Nodes.Length));

   function Root (Doc : Document) return Node is (if Doc.Nodes.Is_Empty then No_Node else 1);

   function Kind_Of (Doc : Document; N : Node) return Kind is
     (if Valid (Doc, N) then Doc.Nodes (Positive (N)).Of_Kind else Nil_Value);

   function Count (Doc : Document; N : Node) return Natural is
   begin
      if not Valid (Doc, N) then
         return 0;
      end if;
      declare
         R : constant Node_Record := Doc.Nodes (Positive (N));
      begin
         return (case R.Of_Kind is
                    when Array_Value => R.Kids_Count,
                    when Map_Value   => R.Kids_Count / 2,
                    when others      => 0);
      end;
   end Count;

   function Child (Doc : Document; N : Node; Index : Positive) return Node is
      R : constant Node_Record := Doc.Nodes (Positive (N));
   begin
      return (if Index <= R.Kids_Count then Doc.Children (R.Kids + Index - 1) else No_Node);
   end Child;

   function Element (Doc : Document; N : Node; Index : Positive) return Node is
     (if Kind_Of (Doc, N) = Array_Value then Child (Doc, N, Index) else No_Node);

   function Key (Doc : Document; N : Node; Index : Positive) return Node is
     (if Kind_Of (Doc, N) = Map_Value then Child (Doc, N, 2 * Index - 1) else No_Node);

   function Value (Doc : Document; N : Node; Index : Positive) return Node is
     (if Kind_Of (Doc, N) = Map_Value then Child (Doc, N, 2 * Index) else No_Node);

   function Text (Doc : Document; N : Node) return String is
   begin
      if Kind_Of (Doc, N) not in String_Value | Binary_Value then
         return "";
      end if;
      declare
         R   : constant Node_Record := Doc.Nodes (Positive (N));
         Ref : constant Holders.Constant_Reference_Type := Doc.Raw.Constant_Reference;
         Raw : Byte_Array renames Ref.Element.all;
         First : constant Offset := Raw'First + Offset (R.First);
      begin
         return To_String (Raw (First .. First + Offset (R.Size) - 1));
      end;
   end Text;

   function Lookup (Doc : Document; Map : Node; Name : String) return Node is
   begin
      for I in 1 .. Count (Doc, Map) loop
         if Kind_Of (Doc, Map) = Map_Value and then Text (Doc, Key (Doc, Map, I)) = Name then
            return Value (Doc, Map, I);
         end if;
      end loop;
      return No_Node;
   end Lookup;

   function Is_Number (Doc : Document; N : Node) return Boolean is
     (Kind_Of (Doc, N) in Integer_Value | Float_Value | Boolean_Value);

   function Number (Doc : Document; N : Node) return Real is
      R : constant Node_Record := Doc.Nodes (Positive (N));
   begin
      return (case R.Of_Kind is
                 when Integer_Value => Real (R.Int),
                 when Float_Value   => R.Flt,
                 when Boolean_Value => (if R.Flag then 1.0 else 0.0),
                 when others        => 0.0);
   end Number;

   function Is_True (Doc : Document; N : Node) return Boolean is
     (Kind_Of (Doc, N) = Boolean_Value and then Doc.Nodes (Positive (N)).Flag);

   function Is_Ndarray (Doc : Document; N : Node) return Boolean is
     (Kind_Of (Doc, N) = Map_Value and then Is_True (Doc, Lookup (Doc, N, "nd")));

   function Dtype (Doc : Document; N : Node) return String is (Text (Doc, Lookup (Doc, N, "type")));

   --  A numpy dtype string: optional byte order (< > | =), kind (f i u b), width in bytes.
   type Dtype_Info is record
      Valid  : Boolean := False;
      Little : Boolean := True;
      Of_Kind : Character := ' ';
      Width  : Natural := 0;
   end record;

   function Parse_Dtype (T : String) return Dtype_Info is
      D : Dtype_Info;
      P : Natural := T'First;
   begin
      if P <= T'Last and then T (P) in '<' | '>' | '|' | '=' then
         D.Little := T (P) /= '>';
         P := P + 1;
      end if;
      if P + 1 = T'Last and then T (T'Last) in '1' .. '9' then
         D.Of_Kind := T (P);
         D.Width := Character'Pos (T (T'Last)) - Character'Pos ('0');
         D.Valid := (case D.Of_Kind is
                        when 'f'       => D.Width in 2 | 4 | 8,
                        when 'i' | 'u' => D.Width in 1 | 2 | 4 | 8,
                        when 'b'       => D.Width = 1,
                        when others    => False);
      end if;
      return D;
   end Parse_Dtype;

   function Data_Size (Doc : Document; N : Node) return Natural is
      Data : constant Node := Lookup (Doc, N, "data");
   begin
      return (if Kind_Of (Doc, Data) = Binary_Value then Doc.Nodes (Positive (Data)).Size else 0);
   end Data_Size;

   function Nd_Shape (Doc : Document; N : Node) return Natural_Array is
      S : constant Node := Lookup (Doc, N, "shape");
      R : Natural_Array (1 .. Count (Doc, S));
   begin
      for I in R'Range loop
         R (I) := Natural (Real'Max (0.0, Number (Doc, Element (Doc, S, I))));
      end loop;
      return R;
   end Nd_Shape;

   function Nd_Is_Numeric (Doc : Document; N : Node) return Boolean is
      D : constant Dtype_Info := Parse_Dtype (Dtype (Doc, N));
      Total : Natural := 1;
   begin
      if not D.Valid then
         return False;
      end if;
      for Extent of Nd_Shape (Doc, N) loop
         Total := Total * Extent;
      end loop;
      return Data_Size (Doc, N) = Total * D.Width;
   end Nd_Is_Numeric;

   function Plain_Shape (Doc : Document; N : Node; Rectangular : out Boolean) return Natural_Array is
   begin
      Rectangular := True;
      if Is_Number (Doc, N) then
         return [1 .. 0 => 0];
      elsif Kind_Of (Doc, N) /= Array_Value then
         Rectangular := False;
         return [1 .. 0 => 0];
      elsif Count (Doc, N) = 0 then
         return [1 => 0];
      end if;
      declare
         Inner : constant Natural_Array := Plain_Shape (Doc, Element (Doc, N, 1), Rectangular);
      begin
         for I in 2 .. Count (Doc, N) loop
            exit when not Rectangular;
            declare
               Other_Ok : Boolean;
               Other    : constant Natural_Array := Plain_Shape (Doc, Element (Doc, N, I), Other_Ok);
            begin
               Rectangular := Other_Ok and then Other = Inner;
            end;
         end loop;
         return Count (Doc, N) & Inner;
      end;
   end Plain_Shape;

   function Shape (Doc : Document; N : Node) return Natural_Array is
      Rectangular : Boolean;
   begin
      if Is_Ndarray (Doc, N) then
         return Nd_Shape (Doc, N);
      end if;
      declare
         S : constant Natural_Array := Plain_Shape (Doc, N, Rectangular);
      begin
         return (if Rectangular then S else [1 .. 0 => 0]);
      end;
   end Shape;

   function Is_Numeric (Doc : Document; N : Node) return Boolean is
      Rectangular : Boolean;
   begin
      if Is_Ndarray (Doc, N) then
         return Nd_Is_Numeric (Doc, N);
      end if;
      declare
         S : constant Natural_Array := Plain_Shape (Doc, N, Rectangular);
         pragma Unreferenced (S);
      begin
         return Rectangular;
      end;
   end Is_Numeric;

   function Is_Byte_Image (Doc : Document; N : Node) return Boolean is
   begin
      if not Is_Ndarray (Doc, N) or else not Nd_Is_Numeric (Doc, N) then
         return False;
      end if;
      declare
         D : constant Dtype_Info := Parse_Dtype (Dtype (Doc, N));
      begin
         return D.Width = 1 and then D.Of_Kind in 'u' | 'i';
      end;
   end Is_Byte_Image;

   procedure Read_Binary
     (Doc     : Document;
      N       : Node;
      Process : not null access procedure (Data : Byte_Array))
   is
      Target : constant Node := (if Is_Ndarray (Doc, N) then Lookup (Doc, N, "data") else N);
   begin
      if Kind_Of (Doc, Target) not in Binary_Value | Extension_Value then
         Process (Byte_Array'(1 .. 0 => 0));
         return;
      end if;
      declare
         R   : constant Node_Record := Doc.Nodes (Positive (Target));
         Ref : constant Holders.Constant_Reference_Type := Doc.Raw.Constant_Reference;
         Raw : Byte_Array renames Ref.Element.all;
         First : constant Offset := Raw'First + Offset (R.First);
      begin
         Process (Raw (First .. First + Offset (R.Size) - 1));
      end;
   end Read_Binary;

   function Leaf_Count (Doc : Document; N : Node) return Natural is
      Total : Natural := 0;
   begin
      if Is_Number (Doc, N) then
         return 1;
      end if;
      for I in 1 .. Count (Doc, N) loop
         Total := Total + Leaf_Count (Doc, Element (Doc, N, I));
      end loop;
      return Total;
   end Leaf_Count;

   function Numbers (Doc : Document; N : Node) return Real_Array is
   begin
      if Is_Number (Doc, N) then
         return [1 => Number (Doc, N)];
      elsif Kind_Of (Doc, N) = Array_Value then
         declare
            Values : Real_Array (1 .. Leaf_Count (Doc, N));
            Next   : Positive := 1;

            procedure Fill (M : Node) is
            begin
               if Is_Number (Doc, M) then
                  Values (Next) := Number (Doc, M);
                  Next := Next + 1;
               else
                  for I in 1 .. Count (Doc, M) loop
                     Fill (Element (Doc, M, I));
                  end loop;
               end if;
            end Fill;
         begin
            Fill (N);
            return Values;
         end;
      end if;
      declare
         D      : constant Dtype_Info := Parse_Dtype (Dtype (Doc, N));
         Values : Real_Array (1 .. Data_Size (Doc, N) / D.Width);

         procedure Convert (Data : Byte_Array) is
         begin
            for I in Values'Range loop
               declare
                  Base : constant Offset := Data'First + Offset ((I - 1) * D.Width);
                  W    : Unsigned_64 := 0;
               begin
                  for K in 0 .. D.Width - 1 loop
                     W := Shift_Left (W, 8)
                       or Unsigned_64 (Data (Base + Offset (if D.Little then D.Width - 1 - K else K)));
                  end loop;
                  Values (I) :=
                    (case D.Of_Kind is
                        when 'f' => (case D.Width is
                                        when 2      => Half (W),
                                        when 4      => Real (U32_To_F32 (Unsigned_32 (W))),
                                        when others => U64_To_F64 (W)),
                        when 'i' => Real (Signed (W, D.Width)),
                        when 'b' => (if W = 0 then 0.0 else 1.0),
                        when others => Real (W));
               end;
            end loop;
         end Convert;
      begin
         Read_Binary (Doc, N, Convert'Access);
         return Values;
      end;
   end Numbers;

   procedure Put_Unsigned (B : in out Buffer; V : Unsigned_64; Width : Natural) is
   begin
      for K in reverse 0 .. Width - 1 loop
         B.Append (Byte (Shift_Right (V, 8 * K) and 16#FF#));
      end loop;
   end Put_Unsigned;

   procedure Put_Nil (B : in out Buffer) is
   begin
      B.Append (Byte'(16#C0#));
   end Put_Nil;

   procedure Put_Boolean (B : in out Buffer; V : Boolean) is
   begin
      B.Append (Byte'(if V then 16#C3# else 16#C2#));
   end Put_Boolean;

   procedure Put_Integer (B : in out Buffer; V : Long_Long_Integer) is
   begin
      if V in 0 .. 16#7F# then
         B.Append (Byte (V));
      elsif V in -32 .. -1 then
         B.Append (Byte (256 + V));
      elsif V in 0 .. 16#FF# then
         B.Append (Byte'(16#CC#)); B.Append (Byte (V));
      elsif V in 0 .. 16#FFFF# then
         B.Append (Byte'(16#CD#)); Put_Unsigned (B, Unsigned_64 (V), 2);
      elsif V in 0 .. 16#FFFF_FFFF# then
         B.Append (Byte'(16#CE#)); Put_Unsigned (B, Unsigned_64 (V), 4);
      elsif V >= 0 then
         B.Append (Byte'(16#CF#)); Put_Unsigned (B, Unsigned_64 (V), 8);
      else
         B.Append (Byte'(16#D3#)); Put_Unsigned (B, To_U64 (Integer_64 (V)), 8);
      end if;
   end Put_Integer;

   procedure Put_Float (B : in out Buffer; V : Real) is
   begin
      B.Append (Byte'(16#CB#));
      Put_Unsigned (B, F64_To_U64 (V), 8);
   end Put_Float;

   procedure Put_Length (B : in out Buffer; Length : Natural; Short_Base, Code8, Code16, Code32 : Byte;
                         Short_Limit : Natural) is
   begin
      if Length <= Short_Limit then
         B.Append (Short_Base + Byte (Length));
      elsif Code8 /= 0 and then Length <= 16#FF# then
         B.Append (Code8); B.Append (Byte (Length));
      elsif Length <= 16#FFFF# then
         B.Append (Code16); Put_Unsigned (B, Unsigned_64 (Length), 2);
      else
         B.Append (Code32); Put_Unsigned (B, Unsigned_64 (Length), 4);
      end if;
   end Put_Length;

   procedure Put_String (B : in out Buffer; V : String) is
   begin
      Put_Length (B, V'Length, 16#A0#, 16#D9#, 16#DA#, 16#DB#, 31);
      B.Append (V);
   end Put_String;

   procedure Put_Binary (B : in out Buffer; V : Byte_Array) is
   begin
      if V'Length <= 16#FF# then
         B.Append (Byte'(16#C4#)); B.Append (Byte (V'Length));
      elsif V'Length <= 16#FFFF# then
         B.Append (Byte'(16#C5#)); Put_Unsigned (B, Unsigned_64 (V'Length), 2);
      else
         B.Append (Byte'(16#C6#)); Put_Unsigned (B, Unsigned_64 (V'Length), 4);
      end if;
      B.Append (V);
   end Put_Binary;

   procedure Put_Array_Header (B : in out Buffer; Length : Natural) is
   begin
      Put_Length (B, Length, 16#90#, 0, 16#DC#, 16#DD#, 15);
   end Put_Array_Header;

   procedure Put_Map_Header (B : in out Buffer; Pairs : Natural) is
   begin
      Put_Length (B, Pairs, 16#80#, 0, 16#DE#, 16#DF#, 15);
   end Put_Map_Header;

   procedure Put_Node (B : in out Buffer; Doc : Document; N : Node) is
   begin
      if not Valid (Doc, N) then
         Put_Nil (B);
         return;
      end if;
      declare
         R : constant Node_Record := Doc.Nodes (Positive (N));

         procedure Copy (Data : Byte_Array) is
         begin
            Put_Binary (B, Data);
         end Copy;
      begin
         case R.Of_Kind is
            when Nil_Value       => Put_Nil (B);
            when Boolean_Value   => Put_Boolean (B, R.Flag);
            when Integer_Value   => Put_Integer (B, R.Int);
            when Float_Value     => Put_Float (B, R.Flt);
            when String_Value    => Put_String (B, Text (Doc, N));
            when Binary_Value | Extension_Value => Read_Binary (Doc, N, Copy'Access);
            when Array_Value =>
               Put_Array_Header (B, R.Kids_Count);
               for I in 1 .. R.Kids_Count loop
                  Put_Node (B, Doc, Child (Doc, N, I));
               end loop;
            when Map_Value =>
               Put_Map_Header (B, R.Kids_Count / 2);
               for I in 1 .. R.Kids_Count loop
                  Put_Node (B, Doc, Child (Doc, N, I));
               end loop;
         end case;
      end;
   end Put_Node;

end Driver.Msgpack;
