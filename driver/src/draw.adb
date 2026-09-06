package body Draw is
   Font : constant array (0 .. 9) of String (1 .. 15) :=
     ["111101101101111", "010110010010111", "111001111100111", "111001111001111", "101101111001001",
      "111100111001111", "111100111101111", "111001001001001", "111101111101111", "111101111001111"];

   procedure Put (RGB : in out Buf; W, H : Natural; X, Y : Integer; C : Color) is
   begin
      if X >= 0 and then Y >= 0 and then X < W and then Y < H then
         declare
            I : constant Natural := (Y * W + X) * 3;
         begin
            if I + 2 < Natural (RGB.Length) then
               RGB.Replace_Element (I, C.R); RGB.Replace_Element (I + 1, C.G); RGB.Replace_Element (I + 2, C.B);
            end if;
         end;
      end if;
   end Put;

   procedure Rect (RGB : in out Buf; W, H, X0, Y0, X1, Y1 : Natural; C : Color; Thick : Natural) is
   begin
      for T in 0 .. Natural'Max (0, Thick - 1) loop
         for X in X0 .. X1 loop
            Put (RGB, W, H, X, Y0 + T, C);
            Put (RGB, W, H, X, Integer (Y1) - T, C);
         end loop;
         for Y in Y0 .. Y1 loop
            Put (RGB, W, H, X0 + T, Y, C);
            Put (RGB, W, H, Integer (X1) - T, Y, C);
         end loop;
      end loop;
   end Rect;

   procedure Digit (RGB : in out Buf; W, H, X, Y, D : Natural; C : Color; Scale : Natural) is
      Bits : constant String := Font (D);
   begin
      for R in 0 .. 4 loop
         for Cc in 0 .. 2 loop
            if Bits (R * 3 + Cc + 1) = '1' then
               for Sy in 0 .. Scale - 1 loop
                  for Sx in 0 .. Scale - 1 loop
                     Put (RGB, W, H, X + Cc * Scale + Sx, Y + R * Scale + Sy, C);
                  end loop;
               end loop;
            end if;
         end loop;
      end loop;
   end Digit;

   procedure Number (RGB : in out Buf; W, H, X, Y, N : Natural; C : Color; Scale : Natural) is
      S : constant String := Natural'Image (N);
      Cx : Natural := X;
      Dark : constant Color := (0, 0, 0);
   begin
      --  先铺一块黑底,数字在任何背景上都读得出
      for Yy in Y - 1 .. Y + 5 * Scale loop
         for Xx in X - 1 .. X + (S'Length - 1) * 4 * Scale loop
            Put (RGB, W, H, Xx, Yy, Dark);
         end loop;
      end loop;
      for I in S'First + 1 .. S'Last loop
         if S (I) in '0' .. '9' then
            Digit (RGB, W, H, Cx, Y, Character'Pos (S (I)) - Character'Pos ('0'), C, Scale);
            Cx := Cx + 4 * Scale;
         end if;
      end loop;
   end Number;

   procedure Numbered_Box (RGB : in out Buf; W, H, X0, Y0, X1, Y1, N : Natural; C : Color; Thick : Natural) is
   begin
      Rect (RGB, W, H, X0, Y0, X1, Y1, C, Thick);
      Number (RGB, W, H, X0 + 2, (if Y0 >= 12 then Y0 - 12 else Y0 + 2), N, C, 2);
   end Numbered_Box;

   procedure Dot (RGB : in out Buf; W, H, X, Y, Radius : Natural; C : Color) is
   begin
      for Dy in -Integer (Radius) .. Integer (Radius) loop
         for Dx in -Integer (Radius) .. Integer (Radius) loop
            Put (RGB, W, H, Integer (X) + Dx, Integer (Y) + Dy, C);
         end loop;
      end loop;
   end Dot;

   procedure Grid (RGB : in out Buf; W, H, Cols, Rows : Natural; Centers_U, Centers_V : out Floats) is
      Line : constant Color := (200, 200, 200);
      Label : constant Color := (255, 255, 0);
   begin
      Centers_U.Clear; Centers_V.Clear;
      for C in 1 .. Cols - 1 loop
         declare
            X : constant Natural := C * W / Cols;
         begin
            for Y in 0 .. H - 1 loop
               Put (RGB, W, H, X, Y, Line);
            end loop;
         end;
      end loop;
      for R in 1 .. Rows - 1 loop
         declare
            Y : constant Natural := R * H / Rows;
         begin
            for X in 0 .. W - 1 loop
               Put (RGB, W, H, X, Y, Line);
            end loop;
         end;
      end loop;
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Cols - 1 loop
            declare
               N : constant Natural := R * Cols + C + 1;
               X0 : constant Natural := C * W / Cols;
               Y0 : constant Natural := R * H / Rows;
            begin
               Number (RGB, W, H, X0 + 3, Y0 + 3, N, Label, 2);
               Centers_U.Append ((Long_Float (C) + 0.5) / Long_Float (Cols));
               Centers_V.Append ((Long_Float (R) + 0.5) / Long_Float (Rows));
            end;
         end loop;
      end loop;
   end Grid;
end Draw;
