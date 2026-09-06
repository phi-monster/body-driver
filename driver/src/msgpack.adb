with Interfaces; use Interfaces;
with Ada.Unchecked_Conversion;
package body Msgpack is
   function U32_To_F32 is new Ada.Unchecked_Conversion (Unsigned_32, Float);
   function U64_To_F64 is new Ada.Unchecked_Conversion (Unsigned_64, Long_Float);
   function F64_To_U64 is new Ada.Unchecked_Conversion (Long_Float, Unsigned_64);

   function Decode (Data : Buf; D : out Doc) return Boolean is
      Pos : Natural := 0;
      Len : constant Natural := Natural (Data.Length);
      Bad : Boolean := False;

      function Take return Unsigned_8 is
         V : Unsigned_8;
      begin
         if Pos >= Len then
            Bad := True;
            return 0;
         end if;
         V := Data.Element (Pos);
         Pos := Pos + 1;
         return V;
      end Take;

      function Take_U (N : Natural) return Unsigned_64 is
         V : Unsigned_64 := 0;
      begin
         for K in 1 .. N loop
            V := Shift_Left (V, 8) or Unsigned_64 (Take);
         end loop;
         return V;
      end Take_U;

      function Parse return Integer;

      function Parse return Integer is
         Me : constant Integer := Integer (D.Nodes.Length);
         Nd : Node;
         B : Unsigned_8;
         L : Natural;
      begin
         D.Nodes.Append (Nd);
         B := Take;
         if Bad then
            return -1;
         end if;
         if B <= 16#7F# then
            Nd.K := Int; Nd.I := Long_Long_Integer (B);
         elsif B >= 16#E0# then
            Nd.K := Int; Nd.I := Long_Long_Integer (B) - 256;
         elsif B >= 16#A0# and then B <= 16#BF# then
            L := Natural (B and 16#1F#);
            Nd.K := Str; Nd.S := To_Unbounded_String (To_String (Data, Pos, L)); Pos := Pos + L;
         elsif B >= 16#90# and then B <= 16#9F# then
            L := Natural (B and 16#0F#);
            Nd.K := Arr;
            for K in 1 .. L loop
               Nd.Kids.Append (Parse);
            end loop;
         elsif B >= 16#80# and then B <= 16#8F# then
            L := Natural (B and 16#0F#);
            Nd.K := Map;
            for K in 1 .. 2 * L loop
               Nd.Kids.Append (Parse);
            end loop;
         else
            case B is
               when 16#C0# => Nd.K := Nil;
               when 16#C2# => Nd.K := Bool; Nd.B := False;
               when 16#C3# => Nd.K := Bool; Nd.B := True;
               when 16#C4# | 16#C5# | 16#C6# =>
                  L := Natural (Take_U (if B = 16#C4# then 1 elsif B = 16#C5# then 2 else 4));
                  Nd.K := Bin; Nd.Bin_First := Pos; Nd.Bin_Len := L; Pos := Pos + L;
               when 16#C7# | 16#C8# | 16#C9# =>
                  L := Natural (Take_U (if B = 16#C7# then 1 elsif B = 16#C8# then 2 else 4));
                  B := Take;   --  ext type
                  Nd.K := Ext; Nd.Bin_First := Pos; Nd.Bin_Len := L; Pos := Pos + L;
               when 16#CA# => Nd.K := Flt; Nd.F := Long_Float (U32_To_F32 (Unsigned_32 (Take_U (4))));
               when 16#CB# => Nd.K := Flt; Nd.F := U64_To_F64 (Take_U (8));
               when 16#CC# => Nd.K := Int; Nd.I := Long_Long_Integer (Take_U (1));
               when 16#CD# => Nd.K := Int; Nd.I := Long_Long_Integer (Take_U (2));
               when 16#CE# => Nd.K := Int; Nd.I := Long_Long_Integer (Take_U (4));
               when 16#CF# =>
                  declare
                     V : constant Unsigned_64 := Take_U (8);
                  begin
                     Nd.K := Int;
                     Nd.I := (if V <= Unsigned_64 (Long_Long_Integer'Last) then Long_Long_Integer (V) else Long_Long_Integer'Last);
                  end;
               when 16#D0# => Nd.K := Int; Nd.I := Long_Long_Integer (Integer_8 (Unsigned_8 (Take_U (1))));
               when 16#D1# => Nd.K := Int; Nd.I := Long_Long_Integer (Integer_16 (Unsigned_16 (Take_U (2))));
               when 16#D2# => Nd.K := Int; Nd.I := Long_Long_Integer (Integer_32 (Unsigned_32 (Take_U (4))));
               when 16#D3# => Nd.K := Int; Nd.I := Long_Long_Integer (Integer_64 (Take_U (8)));
               when 16#D4# .. 16#D8# =>
                  L := (case B is when 16#D4# => 1, when 16#D5# => 2, when 16#D6# => 4, when 16#D7# => 8, when others => 16);
                  B := Take;
                  Nd.K := Ext; Nd.Bin_First := Pos; Nd.Bin_Len := L; Pos := Pos + L;
               when 16#D9# | 16#DA# | 16#DB# =>
                  L := Natural (Take_U (if B = 16#D9# then 1 elsif B = 16#DA# then 2 else 4));
                  Nd.K := Str; Nd.S := To_Unbounded_String (To_String (Data, Pos, L)); Pos := Pos + L;
               when 16#DC# | 16#DD# =>
                  L := Natural (Take_U (if B = 16#DC# then 2 else 4));
                  Nd.K := Arr;
                  for K in 1 .. L loop
                     Nd.Kids.Append (Parse);
                  end loop;
               when 16#DE# | 16#DF# =>
                  L := Natural (Take_U (if B = 16#DE# then 2 else 4));
                  Nd.K := Map;
                  for K in 1 .. 2 * L loop
                     Nd.Kids.Append (Parse);
                  end loop;
               when others => Bad := True;
            end case;
         end if;
         if Pos > Len then
            Bad := True;
         end if;
         D.Nodes.Replace_Element (Me, Nd);
         return Me;
      end Parse;
   begin
      D.Nodes.Clear;
      D.Raw := Data;
      declare
         R : constant Integer := Parse;
      begin
         return R = 0 and then not Bad;
      end;
   end Decode;

   function Ok (D : Doc; N : Integer) return Boolean is (N >= 0 and then N < Integer (D.Nodes.Length));

   function Kind_Of (D : Doc; N : Integer) return Kind is
     (if Ok (D, N) then D.Nodes (N).K else Nil);

   function Text (D : Doc; N : Integer) return String is
   begin
      if not Ok (D, N) then
         return "";
      end if;
      case D.Nodes (N).K is
         when Str => return To_String (D.Nodes (N).S);
         when Bin => return To_String (D.Raw, D.Nodes (N).Bin_First, D.Nodes (N).Bin_Len);
         when others => return "";
      end case;
   end Text;

   function Is_Text (D : Doc; N : Integer) return Boolean is
     (Ok (D, N) and then D.Nodes (N).K in Str | Bin);

   function Is_Num (D : Doc; N : Integer) return Boolean is
     (Ok (D, N) and then D.Nodes (N).K in Int | Flt);

   function Num (D : Doc; N : Integer) return Long_Float is
   begin
      if not Ok (D, N) then
         return 0.0;
      end if;
      case D.Nodes (N).K is
         when Int => return Long_Float (D.Nodes (N).I);
         when Flt => return D.Nodes (N).F;
         when Bool => return (if D.Nodes (N).B then 1.0 else 0.0);
         when others => return 0.0;
      end case;
   end Num;

   function Count (D : Doc; N : Integer) return Natural is
   begin
      if not Ok (D, N) then
         return 0;
      end if;
      case D.Nodes (N).K is
         when Arr => return Natural (D.Nodes (N).Kids.Length);
         when Map => return Natural (D.Nodes (N).Kids.Length) / 2;
         when others => return 0;
      end case;
   end Count;

   function Child (D : Doc; N : Integer; I : Natural) return Integer is
   begin
      if Ok (D, N) and then D.Nodes (N).K = Arr and then I < Natural (D.Nodes (N).Kids.Length) then
         return D.Nodes (N).Kids (I);
      end if;
      return -1;
   end Child;

   function Map_Key (D : Doc; N : Integer; I : Natural) return Integer is
   begin
      if Ok (D, N) and then D.Nodes (N).K = Map and then 2 * I + 1 < Natural (D.Nodes (N).Kids.Length) then
         return D.Nodes (N).Kids (2 * I);
      end if;
      return -1;
   end Map_Key;

   function Map_Val (D : Doc; N : Integer; I : Natural) return Integer is
   begin
      if Ok (D, N) and then D.Nodes (N).K = Map and then 2 * I + 1 < Natural (D.Nodes (N).Kids.Length) then
         return D.Nodes (N).Kids (2 * I + 1);
      end if;
      return -1;
   end Map_Val;

   function Key (D : Doc; Map_Node : Integer; Name : String) return Integer is
   begin
      for I in 0 .. Count (D, Map_Node) - 1 loop
         if Text (D, Map_Key (D, Map_Node, I)) = Name then
            return Map_Val (D, Map_Node, I);
         end if;
      end loop;
      return -1;
   end Key;

   function Is_Nd (D : Doc; N : Integer) return Boolean is
      K : constant Integer := Key (D, N, "nd");
   begin
      return Kind_Of (D, N) = Map and then K >= 0 and then D.Nodes (K).K = Bool and then D.Nodes (K).B;
   end Is_Nd;

   function Nd_Type (D : Doc; N : Integer) return String is (Text (D, Key (D, N, "type")));

   function Nd_Shape (D : Doc; N : Integer) return Ints is
      Sh : constant Integer := Key (D, N, "shape");
      R : Ints;
   begin
      for I in 0 .. Count (D, Sh) - 1 loop
         R.Append (Integer (Num (D, Child (D, Sh, I))));
      end loop;
      return R;
   end Nd_Shape;

   procedure Nd_Data (D : Doc; N : Integer; First, Len : out Natural) is
      K : constant Integer := Key (D, N, "data");
   begin
      First := 0; Len := 0;
      if Ok (D, K) and then D.Nodes (K).K = Bin then
         First := D.Nodes (K).Bin_First;
         Len := D.Nodes (K).Bin_Len;
      end if;
   end Nd_Data;

   function Numbers (D : Doc; N : Integer) return Floats is
      R : Floats;
   begin
      if Kind_Of (D, N) = Arr then
         for I in 0 .. Count (D, N) - 1 loop
            R.Append (Num (D, Child (D, N, I)));
         end loop;
         return R;
      end if;
      if Is_Num (D, N) then
         R.Append (Num (D, N));
         return R;
      end if;
      if Is_Nd (D, N) then
         declare
            T : constant String := Nd_Type (D, N);
            First, Len : Natural;
            Little : Boolean := True;
            function Word (Off, Bytes_N : Natural) return Unsigned_64 is
               V : Unsigned_64 := 0;
            begin
               for K in 0 .. Bytes_N - 1 loop
                  declare
                     Idx : constant Natural := (if Little then Off + Bytes_N - 1 - K else Off + K);
                  begin
                     V := Shift_Left (V, 8) or Unsigned_64 (D.Raw.Element (Idx));
                  end;
               end loop;
               return V;
            end Word;
            Code : String (1 .. 2) := "  ";
         begin
            Nd_Data (D, N, First, Len);
            if T'Length >= 2 then
               Little := T (T'First) /= '>';
               Code := T (T'Last - 1 .. T'Last);
            end if;
            if Code = "f4" then
               for I in 0 .. Len / 4 - 1 loop
                  R.Append (Long_Float (U32_To_F32 (Unsigned_32 (Word (First + 4 * I, 4)))));
               end loop;
            elsif Code = "f8" then
               for I in 0 .. Len / 8 - 1 loop
                  R.Append (U64_To_F64 (Word (First + 8 * I, 8)));
               end loop;
            elsif Code = "i4" then
               for I in 0 .. Len / 4 - 1 loop
                  R.Append (Long_Float (Integer_32 (Unsigned_32 (Word (First + 4 * I, 4)))));
               end loop;
            elsif Code = "i8" then
               for I in 0 .. Len / 8 - 1 loop
                  R.Append (Long_Float (Integer_64 (Word (First + 8 * I, 8))));
               end loop;
            elsif Code = "u1" then
               for I in 0 .. Len - 1 loop
                  R.Append (Long_Float (D.Raw.Element (First + I)));
               end loop;
            end if;
         end;
      end if;
      return R;
   end Numbers;

   --  ── 写 ──
   procedure Put_U (S : in out Buf; V : Unsigned_64; N : Natural) is
   begin
      for K in reverse 0 .. N - 1 loop
         S.Append (Unsigned_8 (Shift_Right (V, 8 * K) and 16#FF#));
      end loop;
   end Put_U;

   procedure Put_Nil (S : in out Buf) is
   begin
      S.Append (16#C0#);
   end Put_Nil;

   procedure Put_Bool (S : in out Buf; V : Boolean) is
   begin
      S.Append (if V then 16#C3# else 16#C2#);
   end Put_Bool;

   procedure Put_Int (S : in out Buf; V : Long_Long_Integer) is
   begin
      if V >= 0 and then V <= 127 then
         S.Append (Unsigned_8 (V));
      elsif V < 0 and then V >= -32 then
         S.Append (Unsigned_8 (256 + V));
      elsif V >= 0 and then V <= 255 then
         S.Append (16#CC#); S.Append (Unsigned_8 (V));
      elsif V >= 0 and then V <= 65535 then
         S.Append (16#CD#); Put_U (S, Unsigned_64 (V), 2);
      elsif V >= 0 and then V <= 4294967295 then
         S.Append (16#CE#); Put_U (S, Unsigned_64 (V), 4);
      elsif V >= 0 then
         S.Append (16#CF#); Put_U (S, Unsigned_64 (V), 8);
      elsif V >= -128 then
         S.Append (16#D0#); S.Append (Unsigned_8 (256 + V));
      elsif V >= -32768 then
         S.Append (16#D1#); Put_U (S, Unsigned_64 (Unsigned_16 (65536 + V)), 2);
      elsif V >= -2147483648 then
         S.Append (16#D2#); Put_U (S, Unsigned_64 (Unsigned_32 (4294967296 + V)), 4);
      else
         S.Append (16#D3#); Put_U (S, Unsigned_64 (Integer_64 (V)), 8);
      end if;
   end Put_Int;

   procedure Put_Float (S : in out Buf; V : Long_Float) is
   begin
      S.Append (16#CB#);
      Put_U (S, F64_To_U64 (V), 8);
   end Put_Float;

   procedure Put_Str (S : in out Buf; V : String) is
      L : constant Natural := V'Length;
   begin
      if L <= 31 then
         S.Append (16#A0# or Unsigned_8 (L));
      elsif L <= 255 then
         S.Append (16#D9#); S.Append (Unsigned_8 (L));
      elsif L <= 65535 then
         S.Append (16#DA#); Put_U (S, Unsigned_64 (L), 2);
      else
         S.Append (16#DB#); Put_U (S, Unsigned_64 (L), 4);
      end if;
      Append (S, V);
   end Put_Str;

   procedure Put_Bin (S : in out Buf; V : Buf; First, Len : Natural) is
   begin
      if Len <= 255 then
         S.Append (16#C4#); S.Append (Unsigned_8 (Len));
      elsif Len <= 65535 then
         S.Append (16#C5#); Put_U (S, Unsigned_64 (Len), 2);
      else
         S.Append (16#C6#); Put_U (S, Unsigned_64 (Len), 4);
      end if;
      for I in 0 .. Len - 1 loop
         S.Append (V.Element (First + I));
      end loop;
   end Put_Bin;

   procedure Put_Array (S : in out Buf; N : Natural) is
   begin
      if N <= 15 then
         S.Append (16#90# or Unsigned_8 (N));
      elsif N <= 65535 then
         S.Append (16#DC#); Put_U (S, Unsigned_64 (N), 2);
      else
         S.Append (16#DD#); Put_U (S, Unsigned_64 (N), 4);
      end if;
   end Put_Array;

   procedure Put_Map (S : in out Buf; N : Natural) is
   begin
      if N <= 15 then
         S.Append (16#80# or Unsigned_8 (N));
      elsif N <= 65535 then
         S.Append (16#DE#); Put_U (S, Unsigned_64 (N), 2);
      else
         S.Append (16#DF#); Put_U (S, Unsigned_64 (N), 4);
      end if;
   end Put_Map;

   procedure Put_Node (S : in out Buf; D : Doc; N : Integer) is
   begin
      if not Ok (D, N) then
         Put_Nil (S);
         return;
      end if;
      declare
         Nd : constant Node := D.Nodes (N);
      begin
         case Nd.K is
            when Nil => Put_Nil (S);
            when Bool => Put_Bool (S, Nd.B);
            when Int => Put_Int (S, Nd.I);
            when Flt => Put_Float (S, Nd.F);
            when Str => Put_Str (S, To_String (Nd.S));
            when Bin | Ext => Put_Bin (S, D.Raw, Nd.Bin_First, Nd.Bin_Len);
            when Arr =>
               Put_Array (S, Natural (Nd.Kids.Length));
               for K of Nd.Kids loop
                  Put_Node (S, D, K);
               end loop;
            when Map =>
               Put_Map (S, Natural (Nd.Kids.Length) / 2);
               for K of Nd.Kids loop
                  Put_Node (S, D, K);
               end loop;
         end case;
      end;
   end Put_Node;
end Msgpack;
