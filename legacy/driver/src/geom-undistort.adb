separate (Geom)
procedure Undistort (G : Cam_Geo; Xd, Yd : Long_Float; X, Y : out Long_Float; Ok : out Boolean) is
   Rd : constant Long_Float := Sqrt (Xd * Xd + Yd * Yd);
   function Excess (R : Long_Float) return Long_Float is (R * (1.0 + G.K1 * R * R + G.K2 * R ** 4) - Rd);   --  畸变后的半径比 r畸 多多少
   function Slope (R : Long_Float) return Long_Float is (1.0 + 3.0 * G.K1 * R * R + 5.0 * G.K2 * R ** 4);   --  d(畸变后的半径) / dr
   --  导数的零点:5K2 s² + 3K1 s + 1 = 0(s = r²)最小的正根 = 2 ÷ (−3K1 + √(9K1² − 20K2))(常数项是 1 的求根式,K2 = 0 也成立);分母 ≤ 0 = 没有正根
   Disc : constant Long_Float := 9.0 * G.K1 * G.K1 - 20.0 * G.K2;
   Den : constant Long_Float := (if Disc >= 0.0 then Sqrt (Disc) - 3.0 * G.K1 else 0.0);
   Cap : constant Natural := Long_Float'Machine_Mantissa + Long_Float'Machine_Emax - Long_Float'Machine_Emin;
   Lo : Long_Float := 0.0;
   Hi, R, Rn : Long_Float;
   Steps : Natural := 0;
begin
   X := Xd; Y := Yd; Ok := True;
   if (G.K1 = 0.0 and then G.K2 = 0.0) or else Rd = 0.0 then
      return;
   end if;
   if Den > 0.0 then
      Hi := Sqrt (2.0 / Den);   --  折回半径 r*
      if Excess (Hi) < 0.0 then
         Ok := False;           --  r畸 比镜头模型能到的最大半径还大
         return;
      end if;
   else
      --  一直单调:导数的最小值 m(K1 < 0 时在 s = −3K1 ÷ (10 K2) 处 = −(9K1² − 20K2) ÷ (20 K2);否则在 r = 0 处 = 1)
      --  ⇒ 畸变后的半径 ≥ m·r ⇒ 根 ≤ r畸 ÷ m
      Hi := (if G.K1 < 0.0 then Rd * (20.0 * G.K2) / (-Disc) else Rd);
   end if;
   R := Long_Float'Min (Rd, Hi);
   loop
      declare
         E : constant Long_Float := Excess (R);
         S : constant Long_Float := Slope (R);
         Newton : Boolean := S > 0.0;
      begin
         exit when E = 0.0;
         if E < 0.0 then
            Lo := R;
         else
            Hi := R;
         end if;
         if Newton then
            Rn := R - E / S;
            exit when Rn = R;   --  牛顿一步不再改变 r:到了浮点数的分辨率
            Newton := Rn > Lo and then Rn < Hi;
         end if;
         if not Newton then
            Rn := Lo + (Hi - Lo) / 2.0;   --  走出夹住的区间(或在 r* 上导数为 0)⇒ 对半分
            exit when not (Rn > Lo and then Rn < Hi);   --  区间只剩相邻两个浮点数
         end if;
         R := Rn;
      end;
      Steps := Steps + 1;
      if Steps >= Cap then
         Ok := False;   --  保险:到了上限还在变,不交半路的数
         return;
      end if;
   end loop;
   X := Xd * (R / Rd); Y := Yd * (R / Rd);
end Undistort;
