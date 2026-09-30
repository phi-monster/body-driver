separate (Plug)
function Image_Lag (L : Link; Cam, Group, From_Seq : Natural; Corr : out Floats) return Integer is
   Best : Integer := 0;
   Best_C : Long_Float := Long_Float'First;
begin
   Corr := Zeros (2 * Max_Lag + 1);
   for Lag in -Max_Lag .. Max_Lag loop
      declare
         Sx, Sy, Sxx, Syy, Sxy : Long_Float := 0.0;
         N : Natural := 0;
      begin
         for K in 0 .. Natural (L.Beats.Length) - 1 loop
            declare
               Kq : constant Integer := K - Lag;
            begin
               --  这台相机那一拍没量成(那一拍或上一拍没收到它的画面)⇒ 不参与:不当成"画面没变"
               if Kq >= 0 and then Kq < Natural (L.Beats.Length) and then L.Beats (K).Seq >= From_Seq and then L.Beats (Kq).Seq >= From_Seq
                 and then Cam < Natural (L.Beats (K).Img_Chg.Length) and then Cam < Natural (L.Beats (K).Img_Ok.Length)
                 and then L.Beats (K).Img_Ok (Cam) and then Group < Natural (L.Beats (Kq).Q_Chg.Length)
               then
                  declare
                     X : constant Long_Float := L.Beats (K).Img_Chg (Cam);
                     Y : constant Long_Float := L.Beats (Kq).Q_Chg (Group);
                  begin
                     Sx := Sx + X; Sy := Sy + Y; Sxx := Sxx + X * X; Syy := Syy + Y * Y; Sxy := Sxy + X * Y;
                     N := N + 1;
                  end;
               end if;
            end;
         end loop;
         if N >= 2 then
            declare
               Nn : constant Long_Float := Long_Float (N);
               Vx : constant Long_Float := Sxx - Sx * Sx / Nn;
               Vy : constant Long_Float := Syy - Sy * Sy / Nn;
            begin
               if Vx > 0.0 and then Vy > 0.0 then
                  Corr.Replace_Element (Lag + Max_Lag, (Sxy - Sx * Sy / Nn) / Sqrt (Vx * Vy));
                  if Corr (Lag + Max_Lag) > Best_C then
                     Best_C := Corr (Lag + Max_Lag); Best := Lag;
                  end if;
               end if;
            end;
         end if;
      end;
   end loop;
   return Best;
end Image_Lag;
