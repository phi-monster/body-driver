package body Json is
   function Parse (Src : String; D : out Doc; Err : out Unbounded_String) return Boolean is
      Pos : Natural := Src'First;
      Bad : Boolean := False;

      procedure Skip_Ws is
      begin
         while Pos <= Src'Last and then Src (Pos) in ' ' | ASCII.HT | ASCII.LF | ASCII.CR loop
            Pos := Pos + 1;
         end loop;
      end Skip_Ws;

      function Parse_Value return Integer;

      function Parse_String return Unbounded_String is
         R : Unbounded_String;
      begin
         Pos := Pos + 1;   --  开引号
         while Pos <= Src'Last loop
            if Src (Pos) = '"' then
               Pos := Pos + 1;
               return R;
            elsif Src (Pos) = '\' and then Pos < Src'Last then
               Pos := Pos + 1;
               case Src (Pos) is
                  when 'n' => Append (R, ASCII.LF);
                  when 't' => Append (R, ASCII.HT);
                  when 'r' => Append (R, ASCII.CR);
                  when 'b' => Append (R, ASCII.BS);
                  when 'f' => Append (R, ASCII.FF);
                  when 'u' =>
                     --  \uXXXX:只保留 ASCII 范围,其它写成 ?(提示词里不会出现非 ASCII 的转义)
                     if Pos + 4 <= Src'Last then
                        declare
                           V : Natural := 0;
                        begin
                           for K in 1 .. 4 loop
                              declare
                                 C : constant Character := Src (Pos + K);
                                 Nib : constant Natural :=
                                   (case C is
                                       when '0' .. '9' => Character'Pos (C) - 48,
                                       when 'a' .. 'f' => Character'Pos (C) - 87,
                                       when 'A' .. 'F' => Character'Pos (C) - 55,
                                       when others => 0);
                              begin
                                 V := V * 16 + Nib;
                              end;
                           end loop;
                           Append (R, (if V < 128 then Character'Val (V) else '?'));
                           Pos := Pos + 4;
                        end;
                     end if;
                  when others => Append (R, Src (Pos));
               end case;
               Pos := Pos + 1;
            else
               Append (R, Src (Pos));
               Pos := Pos + 1;
            end if;
         end loop;
         Bad := True;
         return R;
      end Parse_String;

      function Parse_Value return Integer is
         Me : constant Integer := Integer (D.Nodes.Length);
         Nd : Node;
      begin
         D.Nodes.Append (Nd);
         Skip_Ws;
         if Pos > Src'Last then
            Bad := True;
            return Me;
         end if;
         case Src (Pos) is
            when '{' =>
               Nd.K := J_Obj;
               Pos := Pos + 1;
               loop
                  Skip_Ws;
                  exit when Pos <= Src'Last and then Src (Pos) = '}';
                  if Pos > Src'Last or else Src (Pos) /= '"' then
                     Bad := True;
                     exit;
                  end if;
                  declare
                     Kn : Node;
                     Ki : constant Integer := Integer (D.Nodes.Length);
                  begin
                     Kn.K := J_Str;
                     Kn.S := Parse_String;
                     D.Nodes.Append (Kn);
                     Nd.Kids.Append (Ki);
                  end;
                  Skip_Ws;
                  if Pos > Src'Last or else Src (Pos) /= ':' then
                     Bad := True;
                     exit;
                  end if;
                  Pos := Pos + 1;
                  Nd.Kids.Append (Parse_Value);
                  Skip_Ws;
                  if Pos <= Src'Last and then Src (Pos) = ',' then
                     Pos := Pos + 1;
                  end if;
                  exit when Bad;
               end loop;
               if Pos <= Src'Last then
                  Pos := Pos + 1;
               end if;
            when '[' =>
               Nd.K := J_Arr;
               Pos := Pos + 1;
               loop
                  Skip_Ws;
                  exit when Pos <= Src'Last and then Src (Pos) = ']';
                  if Pos > Src'Last then
                     Bad := True;
                     exit;
                  end if;
                  Nd.Kids.Append (Parse_Value);
                  Skip_Ws;
                  if Pos <= Src'Last and then Src (Pos) = ',' then
                     Pos := Pos + 1;
                  end if;
                  exit when Bad;
               end loop;
               if Pos <= Src'Last then
                  Pos := Pos + 1;
               end if;
            when '"' =>
               Nd.K := J_Str;
               Nd.S := Parse_String;
            when 't' =>
               Nd.K := J_Bool; Nd.B := True; Pos := Pos + 4;
            when 'f' =>
               Nd.K := J_Bool; Nd.B := False; Pos := Pos + 5;
            when 'n' =>
               Nd.K := J_Null; Pos := Pos + 4;
            when others =>
               declare
                  St : constant Natural := Pos;
               begin
                  while Pos <= Src'Last and then Src (Pos) in '0' .. '9' | '-' | '+' | '.' | 'e' | 'E' loop
                     Pos := Pos + 1;
                  end loop;
                  if Pos = St then
                     Bad := True;
                  else
                     Nd.K := J_Num;
                     begin
                        Nd.F := Long_Float'Value (Src (St .. Pos - 1));
                     exception
                        when others => Bad := True;
                     end;
                  end if;
               end;
         end case;
         D.Nodes.Replace_Element (Me, Nd);
         return Me;
      end Parse_Value;
   begin
      D.Nodes.Clear;
      Err := Null_Unbounded_String;
      declare
         R : constant Integer := Parse_Value;
      begin
         if Bad then
            Err := To_Unbounded_String ("解析失败,位置 " & Natural'Image (Pos));
         end if;
         return R = 0 and then not Bad;
      end;
   end Parse;

   function Ok (D : Doc; N : Integer) return Boolean is (N >= 0 and then N < Integer (D.Nodes.Length));
   function Kind_Of (D : Doc; N : Integer) return Kind is (if Ok (D, N) then D.Nodes (N).K else J_Null);

   function Get (D : Doc; Obj : Integer; Name : String) return Integer is
   begin
      if not Ok (D, Obj) or else D.Nodes (Obj).K /= J_Obj then
         return -1;
      end if;
      declare
         Kids : constant Ints := D.Nodes (Obj).Kids;
         I : Natural := 0;
      begin
         while I + 1 < Natural (Kids.Length) loop
            if To_String (D.Nodes (Kids (I)).S) = Name then
               return Kids (I + 1);
            end if;
            I := I + 2;
         end loop;
      end;
      return -1;
   end Get;

   function Text (D : Doc; N : Integer) return String is
     (if Ok (D, N) and then D.Nodes (N).K = J_Str then To_String (D.Nodes (N).S) else "");
   function Num (D : Doc; N : Integer) return Long_Float is
     (if Ok (D, N) and then D.Nodes (N).K = J_Num then D.Nodes (N).F
      elsif Ok (D, N) and then D.Nodes (N).K = J_Bool then (if D.Nodes (N).B then 1.0 else 0.0) else 0.0);
   function Is_Num (D : Doc; N : Integer) return Boolean is (Ok (D, N) and then D.Nodes (N).K = J_Num);
   function Bool (D : Doc; N : Integer) return Boolean is
     (Ok (D, N) and then ((D.Nodes (N).K = J_Bool and then D.Nodes (N).B) or else (D.Nodes (N).K = J_Num and then D.Nodes (N).F /= 0.0)));
   function Count (D : Doc; N : Integer) return Natural is
     (if Ok (D, N) and then D.Nodes (N).K = J_Arr then Natural (D.Nodes (N).Kids.Length) else 0);
   function Child (D : Doc; N : Integer; I : Natural) return Integer is
     (if Ok (D, N) and then D.Nodes (N).K = J_Arr and then I < Natural (D.Nodes (N).Kids.Length) then D.Nodes (N).Kids (I) else -1);

   function Escape (S : String) return String is
      R : Unbounded_String;
   begin
      for C of S loop
         case C is
            when '"' => Append (R, "\""");
            when '\' => Append (R, "\\");
            when ASCII.LF => Append (R, "\n");
            when ASCII.CR => null;
            when ASCII.HT => Append (R, "\t");
            when others =>
               if Character'Pos (C) < 32 then
                  Append (R, ' ');
               else
                  Append (R, C);
               end if;
         end case;
      end loop;
      return To_String (R);
   end Escape;
end Json;
