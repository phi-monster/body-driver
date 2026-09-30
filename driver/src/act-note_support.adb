separate (Act)
procedure Note_Support (C : in out Context; P, N : Geom.V3; How : String) is
begin
   --  标定板的点拟合过一张面(腕眼三角,1 mm 级,Geo_Board)⇒ 东西躺的面就是它;朝下顶住的点只和它对账、不换它:
   --  顶住的点 = 位姿读数 + 量过的指尖偏移,指尖错了它就错(X5B 2026-09-25:指尖错了的那只手顶住的点比板的面低 20.8 cm,"最低的赢"把它当成了桌面,板的面被顶掉)。
   --  门 = 3 倍(倍数无量纲,同踢离群)"板的面内离散 ⊕ 位姿读数的抖动",两样都是量的。高出门 ⇒ 躺在面上的东西;低过门 ⇒ 桌面压不下去,错的是我算的那一点
   if C.Board_Plane then
      declare
         Hb : constant Long_Float := (P (0) - C.Board_Pt (0)) * C.Board_N (0) + (P (1) - C.Board_Pt (1)) * C.Board_N (1) + (P (2) - C.Board_Pt (2)) * C.Board_N (2);
         Tol : constant Long_Float := 3.0 * Sqrt (C.Board_Rms ** 2 + C.Map.EE_Noise ** 2);
      begin
         C.Touch_Pt := C.Board_Pt; C.Touch_N := C.Board_N; C.Touch_Valid := True; C.Touch_Fresh := True;
         if Geom."=" (P, C.Board_Pt) then
            Geo_Say ("东西躺的面 = " & How);
         elsif Hb > Tol then
            C.Bumps.Append (P);
            Geo_Say ("有个东西顶着我(" & How & "),比标定板的面高 " & Mm (Hb) & " ⇒ 是躺在面上的东西;记成「这儿有东西」,面还是板的那张;沿着它接着走");
         elsif Hb < -Tol then
            Geo_Say ("对不上:顶住我的这一点(" & How & ")比标定板的面低 " & Mm (-Hb) & "(门 " & Mm (Tol) & "),桌面压不下去 ⇒ 错的是我算的这一点"
                     & "(指尖偏移或手上那只眼的几何);面还是板的那张");
         else
            Geo_Say ("对账:顶住我的这一点(" & How & ")就在标定板的面上(差 " & Mm (Hb) & ",门 " & Mm (Tol) & ")⇒ 对得上");
         end if;
      end;
      return;
   end if;
   if C.Touch_Valid and then C.Touch_Fresh then
      declare
         H : constant Long_Float := (P (0) - C.Touch_Pt (0)) * C.Touch_N (0) + (P (1) - C.Touch_Pt (1)) * C.Touch_N (1) + (P (2) - C.Touch_Pt (2)) * C.Touch_N (2);
      begin
         if H > C.Map.EE_Noise then
            C.Bumps.Append (P);
            Geo_Say ("有个东西顶着我(" & How & "),比它躺的面高 " & Mm (H) & " ⇒ 不是面,是躺在面上的东西;记成「这儿有东西」,面还是原来那张;沿着它接着走");
            return;
         end if;
      end;
   end if;
   C.Touch_Pt := P; C.Touch_N := N; C.Touch_Valid := True; C.Touch_Fresh := True;
   Geo_Say ("有个面顶着我(" & How & ")⇒ 是它躺的面,记进地图;沿着它接着走");
end Note_Support;
