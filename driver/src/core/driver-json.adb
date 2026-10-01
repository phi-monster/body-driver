with Ada.Long_Float_Text_IO;
with Ada.Strings.Fixed;
with Ada.Unchecked_Conversion;
with Interfaces;

package body Driver.Json is

   use Ada.Strings.Unbounded;
   use Interfaces;

   function To_Real is new Ada.Unchecked_Conversion (Unsigned_64, Real);
   Not_A_Number : constant Real := To_Real (16#7FF8_0000_0000_0000#);

   procedure Append_Code_Point (S : in out Unbounded_String; C : Natural) is
   begin
      --  UTF-8 encoding of one code point.
      if C < 16#80# then
         Append (S, Character'Val (C));
      elsif C < 16#800# then
         Append (S, Character'Val (16#C0# + C / 16#40#));
         Append (S, Character'Val (16#80# + C mod 16#40#));
      elsif C < 16#1_0000# then
         Append (S, Character'Val (16#E0# + C / 16#1000#));
         Append (S, Character'Val (16#80# + (C / 16#40#) mod 16#40#));
         Append (S, Character'Val (16#80# + C mod 16#40#));
      else
         Append (S, Character'Val (16#F0# + C / 16#4_0000#));
         Append (S, Character'Val (16#80# + (C / 16#1000#) mod 16#40#));
         Append (S, Character'Val (16#80# + (C / 16#40#) mod 16#40#));
         Append (S, Character'Val (16#80# + C mod 16#40#));
      end if;
   end Append_Code_Point;

   procedure Parse (Text : String; Doc : out Document; Ok : out Boolean; Why : out Unbounded_String) is
      Pos : Natural := Text'First;
      Bad : Boolean := False;

      procedure Fail (What : String) is
      begin
         if not Bad then
            Why := To_Unbounded_String (What & " at character" & Natural'Image (Pos - Text'First + 1));
         end if;
         Bad := True;
      end Fail;

      procedure Skip_Blanks is
      begin
         while Pos <= Text'Last and then Text (Pos) in ' ' | ASCII.HT | ASCII.LF | ASCII.CR loop
            Pos := Pos + 1;
         end loop;
      end Skip_Blanks;

      function Hex4 return Natural is
         V : Natural := 0;
      begin
         if Pos + 3 > Text'Last then
            Fail ("truncated \u escape");
            return 0;
         end if;
         for K in 0 .. 3 loop
            declare
               C : constant Character := Text (Pos + K);
            begin
               V := 16 * V + (case C is
                                 when '0' .. '9' => Character'Pos (C) - Character'Pos ('0'),
                                 when 'a' .. 'f' => Character'Pos (C) - Character'Pos ('a') + 10,
                                 when 'A' .. 'F' => Character'Pos (C) - Character'Pos ('A') + 10,
                                 when others     => 0);
               if C not in '0' .. '9' | 'a' .. 'f' | 'A' .. 'F' then
                  Fail ("bad \u escape");
               end if;
            end;
         end loop;
         Pos := Pos + 4;
         return V;
      end Hex4;

      function Parse_String return Unbounded_String is
         S : Unbounded_String;
      begin
         Pos := Pos + 1;
         while Pos <= Text'Last loop
            declare
               C : constant Character := Text (Pos);
            begin
               if C = '"' then
                  Pos := Pos + 1;
                  return S;
               elsif C = '\' and then Pos < Text'Last then
                  Pos := Pos + 1;
                  case Text (Pos) is
                     when 'n' => Append (S, ASCII.LF); Pos := Pos + 1;
                     when 't' => Append (S, ASCII.HT); Pos := Pos + 1;
                     when 'r' => Append (S, ASCII.CR); Pos := Pos + 1;
                     when 'b' => Append (S, ASCII.BS); Pos := Pos + 1;
                     when 'f' => Append (S, ASCII.FF); Pos := Pos + 1;
                     when 'u' =>
                        Pos := Pos + 1;
                        declare
                           High : constant Natural := Hex4;
                        begin
                           if High in 16#D800# .. 16#DBFF# and then Pos + 1 <= Text'Last
                             and then Text (Pos .. Pos + 1) = "\u"
                           then
                              Pos := Pos + 2;
                              declare
                                 Low : constant Natural := Hex4;
                              begin
                                 Append_Code_Point
                                   (S, 16#1_0000# + (High - 16#D800#) * 16#400# + (Low - 16#DC00#));
                              end;
                           else
                              Append_Code_Point (S, High);
                           end if;
                        end;
                     when others => Append (S, Text (Pos)); Pos := Pos + 1;
                  end case;
               else
                  Append (S, C);
                  Pos := Pos + 1;
               end if;
            end;
         end loop;
         Fail ("unterminated string");
         return S;
      end Parse_String;

      function Parse_Number return Real is
         Start : constant Natural := Pos;
         Point, Exponent : Natural := 0;
      begin
         while Pos <= Text'Last and then Text (Pos) in '0' .. '9' | '-' | '+' | '.' | 'e' | 'E' loop
            if Text (Pos) = '.' then
               Point := Pos;
            elsif Text (Pos) in 'e' | 'E' then
               Exponent := Pos;
            end if;
            Pos := Pos + 1;
         end loop;
         if Pos = Start then
            Fail ("expected a value");
            return 0.0;
         end if;
         declare
            Literal : constant String := Text (Start .. Pos - 1);
            Mantissa_End : constant Natural := (if Exponent > 0 then Exponent - 1 else Pos - 1);
            --  Ada needs a point in a real literal: 1e5 becomes 1.0e5.
            Normal : constant String :=
              (if Point = 0 then Text (Start .. Mantissa_End) & ".0" & Text (Mantissa_End + 1 .. Pos - 1)
               else Literal);
         begin
            return Real'Value (Normal);
         exception
            when Constraint_Error =>
               Fail ("bad number " & Literal);
               return 0.0;
         end;
      end Parse_Number;

      function Parse_Value return Node;

      function Add (R : Node_Record) return Node is
      begin
         Doc.Nodes.Append (R);
         return Node (Doc.Nodes.Length);
      end Add;

      procedure Attach (R : in out Node_Record; Kids : Child_Vectors.Vector) is
      begin
         R.Kids := Natural (Doc.Children.Length) + 1;
         R.Kids_Count := Natural (Kids.Length);
         Doc.Children.Append (Kids);
      end Attach;

      function Parse_Value return Node is
         R : Node_Record;
      begin
         Skip_Blanks;
         if Pos > Text'Last then
            Fail ("expected a value");
            return Add (R);
         end if;
         case Text (Pos) is
            when '{' =>
               declare
                  Kids : Child_Vectors.Vector;
               begin
                  R.Of_Kind := Object_Value;
                  Pos := Pos + 1;
                  Skip_Blanks;
                  if Pos <= Text'Last and then Text (Pos) = '}' then
                     Pos := Pos + 1;
                  else
                     loop
                        Skip_Blanks;
                        if Pos > Text'Last or else Text (Pos) /= '"' then
                           Fail ("expected a member name");
                           exit;
                        end if;
                        Kids.Append (Add ((Of_Kind => String_Value, Str => Parse_String, others => <>)));
                        Skip_Blanks;
                        if Pos > Text'Last or else Text (Pos) /= ':' then
                           Fail ("expected ':'");
                           exit;
                        end if;
                        Pos := Pos + 1;
                        Kids.Append (Parse_Value);
                        exit when Bad;
                        Skip_Blanks;
                        if Pos <= Text'Last and then Text (Pos) = ',' then
                           Pos := Pos + 1;
                        elsif Pos <= Text'Last and then Text (Pos) = '}' then
                           Pos := Pos + 1;
                           exit;
                        else
                           Fail ("expected ',' or '}'");
                           exit;
                        end if;
                     end loop;
                  end if;
                  Attach (R, Kids);
               end;
            when '[' =>
               declare
                  Kids : Child_Vectors.Vector;
               begin
                  R.Of_Kind := Array_Value;
                  Pos := Pos + 1;
                  Skip_Blanks;
                  if Pos <= Text'Last and then Text (Pos) = ']' then
                     Pos := Pos + 1;
                  else
                     loop
                        Kids.Append (Parse_Value);
                        exit when Bad;
                        Skip_Blanks;
                        if Pos <= Text'Last and then Text (Pos) = ',' then
                           Pos := Pos + 1;
                        elsif Pos <= Text'Last and then Text (Pos) = ']' then
                           Pos := Pos + 1;
                           exit;
                        else
                           Fail ("expected ',' or ']'");
                           exit;
                        end if;
                     end loop;
                  end if;
                  Attach (R, Kids);
               end;
            when '"' =>
               R.Of_Kind := String_Value;
               R.Str := Parse_String;
            when 't' | 'f' | 'n' =>
               declare
                  Word : constant String :=
                    (case Text (Pos) is when 't' => "true", when 'f' => "false", when others => "null");
               begin
                  if Pos + Word'Length - 1 <= Text'Last and then Text (Pos .. Pos + Word'Length - 1) = Word then
                     R.Of_Kind := (if Word = "null" then Null_Value else Boolean_Value);
                     R.Flag := Word = "true";
                     Pos := Pos + Word'Length;
                  else
                     Fail ("unknown word");
                  end if;
               end;
            when others =>
               R.Of_Kind := Number_Value;
               R.Value := Parse_Number;
         end case;
         return Add (R);
      end Parse_Value;

      Top : Node;
   begin
      Doc := (others => <>);
      Why := Null_Unbounded_String;
      Top := Parse_Value;
      Skip_Blanks;
      if not Bad and then Pos <= Text'Last then
         Fail ("text after the value");
      end if;
      --  Children are added before their container, so the top value is the
      --  last node, which is where Root looks.
      Ok := not Bad and then Natural (Top) = Natural (Doc.Nodes.Length);
      if not Ok then
         Doc := (others => <>);
      end if;
   end Parse;

   function Valid (Doc : Document; N : Node) return Boolean is
     (N /= No_Node and then Natural (N) <= Natural (Doc.Nodes.Length));

   function Root (Doc : Document) return Node is
     (if Doc.Nodes.Is_Empty then No_Node else Node (Doc.Nodes.Length));

   function Kind_Of (Doc : Document; N : Node) return Kind is
     (if Valid (Doc, N) then Doc.Nodes (Positive (N)).Of_Kind else Null_Value);

   function Count (Doc : Document; N : Node) return Natural is
     (if not Valid (Doc, N) then 0
      elsif Doc.Nodes (Positive (N)).Of_Kind = Array_Value then Doc.Nodes (Positive (N)).Kids_Count
      elsif Doc.Nodes (Positive (N)).Of_Kind = Object_Value then Doc.Nodes (Positive (N)).Kids_Count / 2
      else 0);

   function Child (Doc : Document; N : Node; Index : Positive) return Node is
      R : constant Node_Record := Doc.Nodes (Positive (N));
   begin
      return (if Index <= R.Kids_Count then Doc.Children (R.Kids + Index - 1) else No_Node);
   end Child;

   function Element (Doc : Document; N : Node; Index : Positive) return Node is
     (if Kind_Of (Doc, N) = Array_Value then Child (Doc, N, Index) else No_Node);

   function Member_Name (Doc : Document; N : Node; Index : Positive) return String is
     (if Kind_Of (Doc, N) = Object_Value then Text (Doc, Child (Doc, N, 2 * Index - 1)) else "");

   function Member_Value (Doc : Document; N : Node; Index : Positive) return Node is
     (if Kind_Of (Doc, N) = Object_Value then Child (Doc, N, 2 * Index) else No_Node);

   function Lookup (Doc : Document; Object : Node; Name : String) return Node is
   begin
      for I in 1 .. Count (Doc, Object) loop
         if Kind_Of (Doc, Object) = Object_Value and then Member_Name (Doc, Object, I) = Name then
            return Member_Value (Doc, Object, I);
         end if;
      end loop;
      return No_Node;
   end Lookup;

   function Text (Doc : Document; N : Node) return String is
     (if Kind_Of (Doc, N) = String_Value then To_String (Doc.Nodes (Positive (N)).Str) else "");

   function Number (Doc : Document; N : Node) return Real is
     (case Kind_Of (Doc, N) is
         when Number_Value  => Doc.Nodes (Positive (N)).Value,
         when Boolean_Value => (if Doc.Nodes (Positive (N)).Flag then 1.0 else 0.0),
         when Null_Value    => Not_A_Number,
         when others        => Not_A_Number);

   function Is_True (Doc : Document; N : Node) return Boolean is
     (Kind_Of (Doc, N) = Boolean_Value and then Doc.Nodes (Positive (N)).Flag);

   function Quote (S : String) return String is
      R : Unbounded_String := To_Unbounded_String ("""");
      Hex : constant String := "0123456789abcdef";
   begin
      for C of S loop
         case C is
            when '"'      => Append (R, "\""");
            when '\'      => Append (R, "\\");
            when ASCII.LF => Append (R, "\n");
            when ASCII.CR => Append (R, "\r");
            when ASCII.HT => Append (R, "\t");
            when others =>
               if Character'Pos (C) < 16#20# then
                  Append (R, "\u00" & Hex (Character'Pos (C) / 16 + 1) & Hex (Character'Pos (C) mod 16 + 1));
               else
                  Append (R, C);   --  UTF-8 bytes pass through unchanged
               end if;
         end case;
      end loop;
      Append (R, '"');
      return To_String (R);
   end Quote;

   function Number_Image (X : Real) return String is
      Buffer : String (1 .. Real'Width + Real'Digits + 8);
   begin
      if X /= X or else abs X > Real'Last then
         return "null";
      end if;
      --  Fewest significant digits that read back to the same double; 17 always do.
      for Digits_Shown in Real'Digits .. Real'Digits + 2 loop
         Ada.Long_Float_Text_IO.Put (Buffer, X, Aft => Digits_Shown - 1, Exp => 1);
         if Real'Value (Buffer) = X or else Digits_Shown = Real'Digits + 2 then
            return Ada.Strings.Fixed.Trim (Buffer, Ada.Strings.Both);
         end if;
      end loop;
      return Ada.Strings.Fixed.Trim (Buffer, Ada.Strings.Both);
   end Number_Image;

end Driver.Json;
