with Ada.Exceptions;
with Touchdown;
separate (Selfcheck)
procedure Welds_Path_2 is
   --  路 2 的焊点(大并行.md §5 路 2):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   use Ada.Numerics.Long_Elementary_Functions;
   function Same_Box (P, Q : Zone.Lobe) return Boolean is
     (P.Valid = Q.Valid and then P.X0 = Q.X0 and then P.Y0 = Q.Y0 and then P.X1 = Q.X1 and then P.Y1 = Q.Y1);
   function Same_Lobe (P, Q : Zone.Lobe) return Boolean is
     (Same_Box (P, Q) and then P.Cu = Q.Cu and then P.Cv = Q.Cv and then P.Count = Q.Count);
begin
   --  🔴 ① I2 瓣改成一串(大并行 §2 第 4 条"手 = N 瓣",§4 I2):量到几块就是几瓣,一段代码,不看瓣数。
   --  病:原来握区只有 A / B 两格、Assemble 只留最大的两块 ⇒ 五指手只认出两根手指、三指爪第三根手指的尖永远量不到;
   --  区心 / 主轴按"两瓣 / 一瓣"两套写法;Is_Self 只查 A / B 两格(第三根手指框里的块不算"我");补全把第 0 瓣以外一律写进 B(第三瓣盖掉第二瓣)。
   --  合成:160 × 60 的画面、背景 100;张开时 N 根黑手指(20,宽 10、高 h)从下边伸进来,合上时并成上方正中一块 8 × 10(x 76–83、y 5–14)。
   --  要:N 瓣、每一瓣的框 / 像素数对上一根手指(一根不少、不重)、第 N 瓣 = 没有;旧的两格 A / B = 第 0 / 1 瓣;每一瓣的尖在它那根手指顶上正中、
   --  尖那一截宽 10;区心 = 手指会合到的那一点(有两根以上 ⇒ 各手指形心的平均;只有一根 ⇒ 合上那一块的形心,上方正中 (79.5, 9.5),
   --  和那根手指的形心隔得远 —— 两根以上时合上那一块也不在手指形心的平均上(y 9.5 对 49.5),它不许把区心拉过去);
   --  主轴 = 各瓣形心排开的方向(一瓣 = 那根手指自己的方向:竖着);张幅 = 合上那一块沿主轴伸多长;
   --  区框 = 合上那一块;每一根手指上的块都是"我"、空地上的不是;瓣的像素一根不少;一串瓣存进 JSON、读回来一个比特不差、还算这一版量的
   declare
      W : constant := 160;
      H : constant := 60;
      type Finger is record
         X0, Hgt : Natural;
      end record;
      type Finger_Set is array (Positive range <>) of Finger;
      procedure Case_N (Fs : Finger_Set; Name : String) is
         N : constant Natural := Fs'Length;
         Open_G, Closed_G : Buf := U8_Vectors.To_Vector (100, Ada.Containers.Count_Type (W * H));
         Z : Zone.Hand_Zone;
         Matched : array (Fs'Range) of Natural := [others => 0];
         Boxes_Ok, Tips_Ok, Self_Ok : Boolean := True;
         Why : Unbounded_String;
         Mu, Mv, Sxx, Syy, Sxy : Long_Float := 0.0;
         Mu_Want, Mv_Want : Long_Float := 0.0;   --  区心该在哪(像素)
         Eu, Ev : Long_Float := 0.0;
         Px_Want : Natural := 0;
         Px_Got : Natural := 0;
      begin
         for F of Fs loop
            for Y in H - F.Hgt .. H - 1 loop
               for X in F.X0 .. F.X0 + 9 loop
                  Open_G.Replace_Element (Y * W + X, 20);
               end loop;
            end loop;
            Px_Want := Px_Want + 10 * F.Hgt;
         end loop;
         for Y in 5 .. 14 loop
            for X in 76 .. 83 loop
               Closed_G.Replace_Element (Y * W + X, 20);
            end loop;
         end loop;
         Z := Zone.From_Frames (Open_G, Closed_G, W, H);
         --  每一瓣对上哪一根手指(按框;同样大的几块谁先谁后不管)
         for K in 0 .. Z.N_Lobes - 1 loop
            declare
               Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, K);
               Hit : Boolean := False;
            begin
               for I in Fs'Range loop
                  if Lb.Valid and then Lb.X0 = Fs (I).X0 and then Lb.X1 = Fs (I).X0 + 9 and then Lb.Y0 = H - Fs (I).Hgt and then Lb.Y1 = H - 1
                    and then Lb.Count = 10 * Fs (I).Hgt
                  then
                     Matched (I) := Matched (I) + 1; Hit := True;
                     declare
                        U, V, Wd, Th : Long_Float;
                        Ok : Boolean;
                     begin
                        Zone.Tip_Section (Z, Lb, W, H, U, V, Wd, Th, Ok);
                        if not (Ok and then abs (U - (Long_Float (Fs (I).X0) + 4.5)) < 1.0e-9 and then V = Long_Float (H - Fs (I).Hgt) and then Wd = 10.0) then
                           Tips_Ok := False;
                           Append (Why, " · 第" & Natural'Image (K) & " 瓣的尖 (" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ") 宽 " & Codec.Fmt (Wd, 0));
                        end if;
                     end;
                     declare
                        R : Picture.Region;
                     begin
                        R.Cu := (Long_Float (Fs (I).X0) + 4.5) / Long_Float (W); R.Cv := (Long_Float (H) - 5.0) / Long_Float (H);
                        Self_Ok := Self_Ok and then Zone.Is_Self (Z, R, W, H);
                     end;
                  end if;
               end loop;
               Boxes_Ok := Boxes_Ok and then Hit;
            end;
         end loop;
         for I in Fs'Range loop
            Boxes_Ok := Boxes_Ok and then Matched (I) = 1;
         end loop;
         declare
            R : Picture.Region;
         begin
            R.Cu := 150.0 / Long_Float (W); R.Cv := 30.0 / Long_Float (H);   --  空地上(没有手指)
            Self_Ok := Self_Ok and then not Zone.Is_Self (Z, R, W, H);
         end;
         for B of Zone.Lobe_Pixels (Z, W, H) loop
            if B then
               Px_Got := Px_Got + 1;
            end if;
         end loop;
         --  区心、主轴:另起一套独立的算法对(形心平均;形心散布按 ½·atan2 求主方向;一瓣的方向 = 手指竖着)
         for F of Fs loop
            Mu := Mu + (Long_Float (F.X0) + 4.5) / Long_Float (N);
            Mv := Mv + (Long_Float (H - 1) - Long_Float (F.Hgt - 1) / 2.0) / Long_Float (N);
         end loop;
         declare
            Cmu : constant Long_Float := (if N = 1 then 79.5 else Mu);   --  一根手指:合上那一块(x 76–83、y 5–14)的形心
            Cmv : constant Long_Float := (if N = 1 then 9.5 else Mv);
         begin
            Mu_Want := Cmu; Mv_Want := Cmv;
         end;
         for F of Fs loop
            declare
               Dx : constant Long_Float := Long_Float (F.X0) + 4.5 - Mu;
               Dy : constant Long_Float := Long_Float (H - 1) - Long_Float (F.Hgt - 1) / 2.0 - Mv;
            begin
               Sxx := Sxx + Dx * Dx; Syy := Syy + Dy * Dy; Sxy := Sxy + Dx * Dy;
            end;
         end loop;
         if N = 1 then
            Eu := 0.0; Ev := 1.0;
         else
            declare
               Th : constant Long_Float := 0.5 * Arctan (2.0 * Sxy, Sxx - Syy);
            begin
               Eu := Cos (Th); Ev := Sin (Th);
               if Eu < 0.0 then
                  Eu := -Eu; Ev := -Ev;
               end if;
            end;
         end if;
         declare
            Du0 : constant Long_Float := Eu / Long_Float (W);
            Dv0 : constant Long_Float := Ev / Long_Float (H);
            Dn : constant Long_Float := Sqrt (Du0 * Du0 + Dv0 * Dv0);
            function Pr (X, Y : Long_Float) return Long_Float is ((X / Long_Float (W)) * Du0 / Dn + (Y / Long_Float (H)) * Dv0 / Dn);
            Span_Want : constant Long_Float :=
              Long_Float'Max (Long_Float'Max (Pr (76.0, 5.0), Pr (83.0, 5.0)), Long_Float'Max (Pr (76.0, 14.0), Pr (83.0, 14.0)))
              - Long_Float'Min (Long_Float'Min (Pr (76.0, 5.0), Pr (83.0, 5.0)), Long_Float'Min (Pr (76.0, 14.0), Pr (83.0, 14.0)));
            --  哪一类是张开时的手指:两根以上按散得开定得下来;一根(两类各一块、又没给不动的部分)定不下来 —— 照样拼出一份,但说明白不许照用。
            --  合上那一块浮在画面中间、不贴画面边 ⇒ 看不出手从哪伸进来,合空那一截没有
            Count_Ok : constant Boolean := Z.Valid and then Z.N_Lobes = N and then not Zone.Lobe_Of (Z, N).Valid and then Z.Open_Known = (N >= 2)
              and then not Z.Shut.Ok;
            Mirror_Ok : constant Boolean := Same_Lobe (Z.A, Zone.Lobe_Of (Z, 0)) and then Same_Lobe (Z.B, Zone.Lobe_Of (Z, 1));
            Center_Ok : constant Boolean := abs (Z.Cu - Mu_Want / Long_Float (W)) < 1.0e-12 and then abs (Z.Cv - Mv_Want / Long_Float (H)) < 1.0e-12;
            Axis_Ok : constant Boolean := abs (Z.Au - Eu) < 1.0e-9 and then abs (Z.Av - Ev) < 1.0e-9;
            Span_Ok : constant Boolean := abs (Z.Span - Span_Want) < 1.0e-12;
            Frame_Ok : constant Boolean := Z.X0 = 76 and then Z.Y0 = 5 and then Z.X1 = 83 and then Z.Y1 = 14;
            Z2 : Zone.Hand_Zone;
            D : Json.Doc;
            E : Unbounded_String;
            Json_Ok : Boolean := False;
         begin
            if Json.Parse ("{""z"":{""lobes"":" & Zone.Lobes_Json (Z) & "}}", D, E) then
               Z2.Valid := True;   --  装回的人先当它有效(同 bodyfile-load 的 Read_Zone),版本对不上 Lobes_From_Json 才把它置成没量过
               Zone.Lobes_From_Json (D, Json.Get (D, 0, "z"), Z2);
               Json_Ok := Z2.Valid and then Z2.N_Lobes = Z.N_Lobes and then (for all K in 0 .. Z.N_Lobes - 1 => Same_Lobe (Zone.Lobe_Of (Z2, K), Zone.Lobe_Of (Z, K)));
            end if;
            Check (Count_Ok and then Boxes_Ok and then Mirror_Ok and then Tips_Ok and then Center_Ok and then Axis_Ok and then Span_Ok and then Frame_Ok
                   and then Self_Ok and then Px_Got = Px_Want and then Json_Ok,
                   "I2 瓣改成一串 · " & Name & ":认出" & Natural'Image (Z.N_Lobes) & " 瓣(该" & Natural'Image (N) & ")"
                   & (if Boxes_Ok then "、一瓣一根手指" else "、瓣和手指对不上") & (if Mirror_Ok then "、A / B = 第 0 / 1 瓣" else "、A / B 没照旧填")
                   & (if Tips_Ok then "、每瓣的尖在手指顶上正中" else "、尖不对" & To_String (Why))
                   & " · 区心 (" & Codec.Fmt (Z.Cu, 4) & "," & Codec.Fmt (Z.Cv, 4) & ")(该 (" & Codec.Fmt (Mu_Want / Long_Float (W), 4) & "," & Codec.Fmt (Mv_Want / Long_Float (H), 4) & "))"
                   & " · 主轴 (" & Codec.Fmt (Z.Au, 4) & "," & Codec.Fmt (Z.Av, 4) & ")(该 (" & Codec.Fmt (Eu, 4) & "," & Codec.Fmt (Ev, 4) & "))"
                   & " · 张幅 " & Codec.Fmt (Z.Span, 4) & "(该 " & Codec.Fmt (Span_Want, 4) & ")" & (if Frame_Ok then "" else " · 区框不对")
                   & (if Self_Ok then " · 每根手指上的块都是我" else " · 有手指上的块不算我") & " · 瓣的像素 " & Codec.Img (Px_Got) & "(该 " & Codec.Img (Px_Want) & ")"
                   & (if Json_Ok then " · 存取一个比特不差" else " · 存取对不上"));
         end;
      end Case_N;
   begin
      Case_N ([1 => (20, 20)], "一瓣(吸盘 / 并着看不开的几根)");
      Case_N ([(10, 20), (140, 20)], "两瓣(两指)");
      Case_N ([(10, 20), (70, 30), (140, 25)], "三瓣(高低不齐:主轴是斜的)");
      Case_N ([(4, 20), (36, 20), (68, 20), (100, 20), (132, 20)], "五瓣(五指)");
   end;

   --  🔴 ② 一串瓣的存取(I2):这一版写的 "lobes" = {"rule": 这一版的号, "list": [...]} ⇒ 读回、照用;
   --  别的版本存的 —— I2 以前(两格 "a" / "b",瓣数在 "n_lobes",第二格瓣数不到也照样写了)、I2 第一版("lobes" 是一串)—— 瓣照样读回来,
   --  可区心 / 主轴是旧算法算的(I2 以前两瓣的主轴按归一化画幅、和东西的主轴差 0.56° 一类的角;I2 第一版一瓣的区心按那一瓣自己),
   --  文件里没有能重算的画面 ⇒ 这一格判成没量过(开机合空重量),照实说;新旧键都有时按 "lobes"。
   --  旧写法写的握区(只填 A / B、一串空着)Lobe_Of 照旧读它的两格,存出去是这一版的那两格,换一瓣时先转成一串。
   --  病:读不回旧身体文件 ⇒ 装回身体时手指全丢;读成三瓣 / 把没写的第二格当一瓣 ⇒ 多出一根不存在的手指;
   --  旧算法算的区心 / 主轴装回来照用 ⇒ 悄悄差着用(主代理 10-01:"不许悄悄差着用")
   declare
      D : Json.Doc;
      E : Unbounded_String;
      Z1, Z2, Z3, Z4, Zl : Zone.Hand_Zone;
      La : constant Zone.Lobe := (Valid => True, X0 => 1, Y0 => 2, X1 => 3, Y1 => 4, Cu => 0.5, Cv => 0.25, Count => 6);
      Lb : constant Zone.Lobe := (Valid => True, X0 => 5, Y0 => 6, X1 => 7, Y1 => 8, Cu => 0.75, Cv => 0.125, Count => 9);
      Lc : constant Zone.Lobe := (Valid => True, X0 => 10, Y0 => 11, X1 => 12, Y1 => 13, Cu => 1.0 / 3.0, Cv => 2.0 / 3.0, Count => 14);
      Ok1, Ok2, Ok3, Ok4, Okl : Boolean := False;
      --  装回一格:先当它有效(同 bodyfile-load 的 Read_Zone),再让 Lobes_From_Json 读瓣、判版本
      procedure Load (Text : String; Z : out Zone.Hand_Zone; Ok : out Boolean) is
      begin
         Z := (others => <>);
         Ok := Json.Parse (Text, D, E);
         if Ok then
            Z.Valid := True;
            Zone.Lobes_From_Json (D, Json.Get (D, 0, "z"), Z);
         end if;
      end Load;
      Parsed : Boolean;
   begin
      Load ("{""z"":{""n_lobes"":1,""a"":[1,2,3,4,0.5,0.25,6],""b"":[0,0,0,0,0,0,0]}}", Z1, Parsed);
      Ok1 := Parsed and then Z1.N_Lobes = 1 and then Same_Lobe (Zone.Lobe_Of (Z1, 0), La) and then not Zone.Lobe_Of (Z1, 1).Valid and then not Z1.Valid;
      Load ("{""z"":{""n_lobes"":2,""a"":[1,2,3,4,0.5,0.25,6],""b"":[5,6,7,8,0.75,0.125,9]}}", Z2, Parsed);
      Ok2 := Parsed and then Z2.N_Lobes = 2 and then Same_Lobe (Zone.Lobe_Of (Z2, 0), La) and then Same_Lobe (Zone.Lobe_Of (Z2, 1), Lb) and then Same_Lobe (Z2.B, Lb)
        and then not Z2.Valid;
      Z3.Valid := True;
      Zone.Set_Lobes (Z3, Zone.Lobe_Vectors.Vector'[La, Lb, Lc]);
      Load ("{""z"":{""n_lobes"":2,""a"":[1,2,3,4,0.5,0.25,6],""b"":[5,6,7,8,0.75,0.125,9],""lobes"":" & Zone.Lobes_Json (Z3) & "}}", Z1, Parsed);
      Ok3 := Parsed and then Z1.N_Lobes = 3 and then Same_Lobe (Zone.Lobe_Of (Z1, 2), Lc) and then Same_Lobe (Zone.Lobe_Of (Z1, 0), La) and then Z1.Valid;
      Load ("{""z"":{""lobes"":[[1,2,3,4,0.5,0.25,6],[5,6,7,8,0.75,0.125,9]]}}", Z4, Parsed);   --  I2 第一版:一串
      Ok4 := Parsed and then Z4.N_Lobes = 2 and then Same_Lobe (Zone.Lobe_Of (Z4, 1), Lb) and then not Z4.Valid;
      --  第 2 版(10-01 上午:还不存合空时手指到的那一截)⇒ 瓣照样读回、判成要重量(碰桌面量合空时的尖要那一截)
      Load ("{""z"":{""lobes"":{""rule"":2,""list"":[[1,2,3,4,0.5,0.25,6],[5,6,7,8,0.75,0.125,9]]}}}", Z2, Parsed);
      Ok4 := Ok4 and then Parsed and then Z2.N_Lobes = 2 and then not Z2.Valid and then not Z2.Shut.Ok;
      Zl.Valid := True; Zl.N_Lobes := 2; Zl.A := La; Zl.B := Lb;   --  旧写法写的握区
      if Same_Lobe (Zone.Lobe_Of (Zl, 0), La) and then Same_Lobe (Zone.Lobe_Of (Zl, 1), Lb) and then not Zone.Lobe_Of (Zl, 2).Valid then
         Load ("{""z"":{""lobes"":" & Zone.Lobes_Json (Zl) & "}}", Z2, Parsed);
         Zone.Set_Lobe (Zl, 1, Lc);
         Okl := Parsed and then Z2.Valid and then Z2.N_Lobes = 2 and then Same_Lobe (Zone.Lobe_Of (Z2, 1), Lb)
           and then Zl.N_Lobes = 2 and then Same_Lobe (Zone.Lobe_Of (Zl, 0), La) and then Same_Lobe (Zone.Lobe_Of (Zl, 1), Lc) and then Same_Lobe (Zl.B, Lc);
      end if;
      Check (Ok1 and then Ok2 and then Ok3 and then Ok4 and then Okl,
             "I2 一串瓣的存取:I2 以前的文件 n_lobes 1(第二格照样写了)⇒ " & (if Ok1 then "读回 1 瓣、判成要重量" else "读错")
             & " · n_lobes 2 ⇒ " & (if Ok2 then "读回 2 瓣、判成要重量" else "读错")
             & " · I2 第一版(一串)、第 2 版(不存合空那一截)⇒ " & (if Ok4 then "读回 2 瓣、判成要重量" else "读错")
             & " · 这一版、新旧键都有 ⇒ " & (if Ok3 then "按 lobes 读回 3 瓣、照用" else "读错")
             & " · 旧写法写的握区(只填 A / B)⇒ " & (if Okl then "照旧读、存成这一版、换一瓣先转成一串" else "读错"));
   end;

   --  🔴 ③ 三瓣的补全(Zone.Apply_Refine,瓣按"长在眼上"补全,09-30 V1B69 那一段):每一瓣补进来的像素扩它自己的框。
   --  病:原来 Kl = 0 写 A、别的一律写 B ⇒ 第三瓣补全时把第二瓣的框换成第三瓣的,第三瓣自己一个像素都没扩(尖还认低)。
   --  合成的眼 640 × 480(焦距 400、主点正中),三根手指从画面下边伸进来:x 20–99 / 200–279 / 380–459,变化掩码里只有 y ≥ 300 那一截,
   --  真的手指 y ≥ 240(上面 60 px 缺了,同 V1B69);合到的区在右边 x 500–600、y 400–479(离三根手指都远)。眼绕竖直轴转 0.16 弧度(画面往右挪 ~64 px,
   --  手指和它四周的背景转完都还在画幅里);手指像素配到原处,别的按这个转动挪。要:三瓣的框都长到 y = 240、各自的横向范围不变,
   --  每一瓣的尖到手指顶上(尖那一截 = 离画面边最远的 480 / 80 = 6 px 那几行 240–246 ⇒ v = 243)
   declare
      Wd : constant := 640;
      Ht : constant := 480;
      Z : Zone.Hand_Zone;
      Eye : Geom.Cam_Geo := Geom.No_Geo;
      Rot : constant Geom.V3 := [0.0, -0.16, 0.0];
      Gr : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Kinem.Gx * Kinem.Gy));
      Ps : Zone.Probe_Vectors.Vector;
      Mu, Mv : Bytes.Floats;
      Added : Natural;
      Xs : constant array (0 .. 2) of Natural := [20, 200, 380];
      function True_Finger (X, Y : Long_Float) return Boolean is
        (Y >= 240.0 and then ((X >= 20.0 and then X < 100.0) or else (X >= 200.0 and then X < 280.0) or else (X >= 380.0 and then X < 460.0)));
      Ls : Zone.Lobe_Vectors.Vector;
      Ok : Boolean := True;
      Note : Unbounded_String;
   begin
      Eye.F := 400.0; Eye.Cx := 320.0; Eye.Cy := 240.0; Eye.Valid := True;
      for K in Xs'Range loop
         Ls.Append (Zone.Lobe'(Valid => True, X0 => Xs (K), Y0 => 300, X1 => Xs (K) + 79, Y1 => 479,
                               Cu => (Long_Float (Xs (K)) + 39.5) / Long_Float (Wd), Cv => 389.5 / Long_Float (Ht), Count => 80 * 180));
      end loop;
      Z.Valid := True;
      Zone.Set_Lobes (Z, Ls);
      Z.X0 := 500; Z.Y0 := 400; Z.X1 := 600; Z.Y1 := 479;
      Z.Fingers := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Wd * Ht));
      for Y in 0 .. Ht - 1 loop
         for X in 0 .. Wd - 1 loop
            if (Y >= 300 and then (for some K in Xs'Range => X in Xs (K) .. Xs (K) + 79)) or else (X in 500 .. 600 and then Y >= 400) then
               Z.Fingers.Replace_Element (Y * Wd + X, True);
            end if;
         end loop;
      end loop;
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            Gr.Replace_Element (Gyy * Kinem.Gx + Gxx, True_Finger (Kinem.Grid_U (Gxx, Wd), Kinem.Grid_V (Gyy, Ht)));
         end loop;
      end loop;
      Ps := Zone.Refine_Probes (Z, Wd, Ht, Gr);
      declare
         Rm : constant Geom.M3 := Geom.Rodrigues (Rot);
      begin
         for P of Ps loop
            if True_Finger (P.U, P.V) then
               Mu.Append (P.U); Mv.Append (P.V);
            else
               declare
                  Okd : Boolean;
                  Dd : constant Geom.V3 := Geom.Cam_Dir (Eye, P.U, P.V, Okd);
                  U1, V1 : Long_Float;
                  Front : Boolean;
               begin
                  Geom.Cam_Pixel (Eye, Geom.Ap (Rm, Dd), U1, V1, Front);
                  Mu.Append (U1); Mv.Append (V1);
               end;
            end if;
         end loop;
      end;
      Zone.Apply_Refine (Z, Wd, Ht, Ps, Mu, Mv, Eye, Rot, 0.3, Added);
      for K in Xs'Range loop
         declare
            Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, K);
            U, V, Wt, Th : Long_Float;
            Okt : Boolean;
         begin
            Zone.Tip_Section (Z, Lb, Wd, Ht, U, V, Wt, Th, Okt);
            Append (Note, " · 第" & Natural'Image (K) & " 瓣框 x " & Codec.Img (Lb.X0) & "–" & Codec.Img (Lb.X1) & " y " & Codec.Img (Lb.Y0) & "–" & Codec.Img (Lb.Y1)
                    & "、尖 v " & (if Okt then Codec.Fmt (V, 1) else "-"));
            Ok := Ok and then Lb.X0 = Xs (K) and then Lb.X1 = Xs (K) + 79 and then Lb.Y0 = 240 and then Okt and then V = 243.0;
         end;
      end loop;
      Check (Ok and then Z.N_Lobes = 3 and then Same_Lobe (Z.B, Zone.Lobe_Of (Z, 1)),
             "I2 三瓣的补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素、补进 " & Codec.Img (Added) & " 个" & To_String (Note) & "(要三瓣都长到 y 240、横向不变)");
   end;

   --  🔴 ④ 压之前看底下看见、躺在面上的点也当量过的桌面(大并行 §2 第 5 条"看底下看见的点也能当'量过的桌面'"):
   --  Act.Board_Free_Spots 里 C.Seen_On 的点和板点一样能围住落点圈、能当落点。
   --  病:原来量过的桌面只有板点 —— 板只是开机不动的眼和腕眼都配得上的那一片,手自己挡着的那块、东西旁边那一圈没有板点;
   --  V1B82 第 1 只手那一瓣朝下压时离它近的板上一处都解不出来(空地全在 5.5 单位以外、够不着),看底下明明看见了脚下一片躺在面上的桌面也不能用。
   --  合成:板 21 × 21 个点 2 cm 一格铺在 0.765 m 的面上,正中 7 × 7 格(±6 cm)一个板点都没有(手挡着的那块);落点 (0,0)、另一瓣 (0.05,0)、
   --  手指宽上限 1 cm。没有看见的点 ⇒ 原处不空(落点圈周围没有量过的桌面),要挪;看底下看见了洞里那 49 个、躺在面上 ⇒ 原处就空,不用挪;
   --  看见的那 49 个里正中那一个其实高 5 mm(东西)⇒ 它不算桌面、照样挡,原处不空
   declare
      Cell : constant Long_Float := 0.02;
      Rw : constant Long_Float := 0.01;
      function Pt (I, J : Integer; Z : Long_Float) return Geom.Scene_Pt is
        (Geom.Scene_Pt'(Pw => [Cell * Long_Float (I), Cell * Long_Float (J), Z], U => 0.0, V => 0.0, Sh => 0.0, Views => 2,
                        Cov => [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]]));
      Cb : Act.Context;
      Lp : Geom.V3_Vectors.Vector;
      Tb : Bytes.Floats;
      D0, D1, D2 : Geom.V3_Vectors.Vector;
   begin
      Cb.Board_Plane := True; Cb.Board_Pt := [0.0, 0.0, 0.765]; Cb.Board_N := [0.0, 0.0, 1.0]; Cb.Board_Rms := 0.001;
      for I in -10 .. 10 loop
         for J in -10 .. 10 loop
            if abs I > 3 or else abs J > 3 then
               Cb.Board.Append (Pt (I, J, 0.765));
            end if;
         end loop;
      end loop;
      Lp.Append (Geom.V3'[0.0, 0.0, 0.765]);
      Lp.Append (Geom.V3'[0.05, 0.0, 0.765]);
      Tb.Append (0.0); Tb.Append (1.0);
      Act.Board_Free_Spots (Cb, Lp, Tb, Rw, D0);
      declare
         Cs : Act.Context := Cb;
         Ct : Act.Context := Cb;
      begin
         for I in -3 .. 3 loop
            for J in -3 .. 3 loop
               Cs.Seen_On.Append (Pt (I, J, 0.765));
               Ct.Seen_On.Append (Pt (I, J, (if I = 0 and then J = 0 then 0.770 else 0.765)));
            end loop;
         end loop;
         Act.Board_Free_Spots (Cs, Lp, Tb, Rw, D1);
         Act.Board_Free_Spots (Ct, Lp, Tb, Rw, D2);
      end;
      Check ((D0.Is_Empty or else Geom.Norm (D0 (0)) > 0.0) and then not D1.Is_Empty and then Geom.Norm (D1 (0)) = 0.0
             and then (D2.Is_Empty or else Geom.Norm (D2 (0)) > 0.0),
             "看底下看见的躺在面上的点当量过的桌面:板中间 ±6 cm 没有板点 ⇒ "
             & (if D0.Is_Empty then "一处都没有" elsif Geom.Norm (D0 (0)) > 0.0 then "原处不空、挪 " & Codec.Fmt (Geom.Norm (D0 (0)), 3) & " m" else "原处就空(不该)")
             & " · 看见了洞里那 49 个 ⇒ " & (if not D1.Is_Empty and then Geom.Norm (D1 (0)) = 0.0 then "原处就空" else "还是不空(看见的点没当桌面)")
             & " · 其中正中那个高 5 mm ⇒ " & (if D2.Is_Empty or else Geom.Norm (D2 (0)) > 0.0 then "照样挡" else "没挡(高出面的当成了桌面)"));
   end;

   --  🔴 ⑤ 一边不动、一边动的夹爪(10-01 路 5 要的"合空时的尖";大并行 §2 第 4 条"合拢那一路:张到头、合空两头都量"):
   --  只有一根在动 ⇒ 张开、合上两头各一块,只看两类各自散得多开分不出哪一类是张开的;不动的那根两头都长在眼上(开机转一下眼判的)⇒
   --  张开那头离它远、合上那头贴着它 ⇒ 定得下来。合空时手指到的那一截 = 动的那根合到不动的那根旁边、它的尖。
   --  病:原来两类一样开时按像素多的那一类当张开的 —— 一根手指两头一样大 ⇒ 把合上那头当成张开(瓣在不动的那根旁边、"张开"的读数其实是合上的),
   --  握区、张开 / 合空的读数整个反过来;合空时的尖没有,路 5 只能假设各瓣对称相向合、各走一半,这种夹爪算错。
   --  合成:160 × 60、背景 100;不动的那根 x 70–79、动的那根张开时 x 130–139、合上时 x 80–89(贴着不动的那根),都从下边伸进来、高 30(到 y 30)、灰度 20。
   --  要:给了不动的那根 ⇒ 定得下来,一瓣 = 张开那头(x 130–139),它的尖 (134.5, 30);合空那一截 = 合上那头的尖 (84.5, 30)、宽 10,
   --  从 (84.5, 59) 伸进来;区心 = 合上那头的形心;存进 JSON、读回来一个比特不差。没给 ⇒ 照样拼出一份,但说明白定不下来(不许照用)
   declare
      W : constant := 160;
      H : constant := 60;
      Open_G, Closed_G : Buf := U8_Vectors.To_Vector (100, Ada.Containers.Count_Type (W * H));
      Static : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      Z, Zn, Z2 : Zone.Hand_Zone;
      U, V, Wd, Th : Long_Float := 0.0;
      Tip_Ok : Boolean := False;
      D : Json.Doc;
      E : Unbounded_String;
      Json_Ok : Boolean := False;
   begin
      for Y in 30 .. H - 1 loop
         for X in 70 .. 79 loop
            Open_G.Replace_Element (Y * W + X, 20); Closed_G.Replace_Element (Y * W + X, 20);
            Static.Replace_Element (Y * W + X, True);
         end loop;
         for X in 130 .. 139 loop
            Open_G.Replace_Element (Y * W + X, 20);
         end loop;
         for X in 80 .. 89 loop
            Closed_G.Replace_Element (Y * W + X, 20);
         end loop;
      end loop;
      Z := Zone.From_Frames (Open_G, Closed_G, W, H, Static);
      Zn := Zone.From_Frames (Open_G, Closed_G, W, H);
      Zone.Tip_Section (Z, Zone.Lobe_Of (Z, 0), W, H, U, V, Wd, Th, Tip_Ok);
      if Json.Parse ("{""z"":{""lobes"":" & Zone.Lobes_Json (Z) & "}}", D, E) then
         Z2.Valid := True;
         Zone.Lobes_From_Json (D, Json.Get (D, 0, "z"), Z2);
         Json_Ok := Z2.Valid and then Z2.N_Lobes = 1 and then Same_Lobe (Zone.Lobe_Of (Z2, 0), Zone.Lobe_Of (Z, 0)) and then Zone."=" (Z2.Shut, Z.Shut);
      end if;
      declare
         Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, 0);
         Lobe_Ok : constant Boolean := Z.Valid and then Z.Open_Known and then Z.N_Lobes = 1 and then Lb.X0 = 130 and then Lb.X1 = 139 and then Lb.Y0 = 30
           and then Lb.Y1 = H - 1 and then Tip_Ok and then U = 134.5 and then V = 30.0;
         Shut_Ok : constant Boolean := Z.Shut.Ok and then Z.Shut.U = 84.5 and then Z.Shut.V = 30.0 and then Z.Shut.Wide = 10.0
           and then Z.Shut.Eu = 84.5 and then Z.Shut.Ev = Long_Float (H - 1);
         Center_Ok : constant Boolean := abs (Z.Cu - 84.5 / Long_Float (W)) < 1.0e-12 and then abs (Z.Cv - 44.5 / Long_Float (H)) < 1.0e-12;
      begin
         Check (Lobe_Ok and then Shut_Ok and then Center_Ok and then Json_Ok and then Zn.Valid and then not Zn.Open_Known,
                "一边不动一边动的夹爪:给了不动的那根 ⇒ " & (if Z.Open_Known then "定得下来" else "还是定不下来")
                & "、瓣 x " & Codec.Img (Lb.X0) & "–" & Codec.Img (Lb.X1) & "(该 130–139)、尖 (" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ")(该 (134.5,30.0))"
                & " · 合空那一截 " & (if Z.Shut.Ok then "(" & Codec.Fmt (Z.Shut.U, 1) & "," & Codec.Fmt (Z.Shut.V, 1) & ") 宽 " & Codec.Fmt (Z.Shut.Wide, 0)
                                      & "、从 (" & Codec.Fmt (Z.Shut.Eu, 1) & "," & Codec.Fmt (Z.Shut.Ev, 1) & ") 伸进来" else "没有")
                & "(该 (84.5,30.0) 宽 10、从 (84.5,59.0))"
                & (if Center_Ok then " · 区心在合上那头" else " · 区心不对") & (if Json_Ok then " · 存取一个比特不差" else " · 存取对不上")
                & " · 没给不动的那根 ⇒ " & (if Zn.Open_Known then "说定下了(不该:一根两头一样大是猜的)" else "说明白定不下来"));
      end;
   end;

   --  🔴 ⑥ 沿一条视线解一点(Geom.Fit_On_Ray,合空时的尖):那一点在眼系里的视线 D 已知、只差多远 λ。
   --  每一下压一条方程(A·x = B,同 Fit_Presses);对得上的最大一组至少 2 下(1 个未知数 + 1 条自己核)。
   --  病:合空时手上最低的那一点不一定在那一截的视线上(五指手只合一根、别的还伸着先碰到;两根手指合空不齐)——
   --  只压朝下那一下、按视线交面当尖,碰着的是别处也照收;两下不核也照收。
   --  合成:真点 λ* = 1.9 沿 D = (0, −0.3, −1)/|·|;每一下眼的朝向不同(朝下、斜 9.2° 朝两个方位),方程按"真点碰到桌面"给;
   --  坏的一下按离视线 0.5 的另一点碰到给。要:两下好的 ⇒ 解到真点(差 < 1e-9)、预测差 ≈ 0;一好一坏 ⇒ 不收;两好一坏 ⇒ 挑出两下好的、解到真点;
   --  一下都没有 / 只有一下 ⇒ 不收
   declare
      Dn : constant Geom.V3 := [0.0, -0.3, -1.0];
      Dl : constant Long_Float := Geom.Norm (Dn);
      Dd : constant Geom.V3 := [Dn (0) / Dl, Dn (1) / Dl, Dn (2) / Dl];
      Xs : constant Geom.V3 := [1.9 * Dd (0), 1.9 * Dd (1), 1.9 * Dd (2)];
      Xb : constant Geom.V3 := [Xs (0) + 0.5, Xs (1), Xs (2)];   --  离视线 0.5 的另一点(别的手指)
      Up : constant Geom.V3 := [0.0, 0.0, 1.0];
      Gate : constant Long_Float := 0.056;
      --  眼的朝向:先把 D 转到朝正下,再绕世界的轴 Ax 转 Th(斜着压)
      function Eq_For (Ax : Geom.V3; Th : Long_Float; Contact : Geom.V3) return Geom.Press_Eq is
         R0 : constant Geom.M3 := Geom.Rodrigues (Geom.Turn_To (Dd, [0.0, 0.0, -1.0]));
         R : constant Geom.M3 := Geom.Mul (Geom.Rodrigues ([Th * Ax (0), Th * Ax (1), Th * Ax (2)]), R0);
         A : constant Geom.V3 := Geom.Ap (Geom.Tr (R), Up);
      begin
         return Geom.Press_Eq'(A => A, B => A (0) * Contact (0) + A (1) * Contact (1) + A (2) * Contact (2), Aimed => True);
      end Eq_For;
      Tilt : constant Long_Float := 0.161;
      Good0 : constant Geom.Press_Eq := Eq_For ([1.0, 0.0, 0.0], 0.0, Xs);
      Good1 : constant Geom.Press_Eq := Eq_For ([1.0, 0.0, 0.0], Tilt, Xs);
      Good2 : constant Geom.Press_Eq := Eq_For ([0.0, 1.0, 0.0], Tilt, Xs);
      Bad1 : constant Geom.Press_Eq := Eq_For ([0.0, 1.0, 0.0], Tilt, Xb);
      E2, Eb, E3, E1 : Geom.Press_Eq_Vectors.Vector;
      F2, Fb, F3, F1 : Geom.Press_Fit;
      function Off (F : Geom.Press_Fit) return Long_Float is (Geom.Norm ([F.X (0) - Xs (0), F.X (1) - Xs (1), F.X (2) - Xs (2)]));
   begin
      E2.Append (Good0); E2.Append (Good1);
      Eb.Append (Good0); Eb.Append (Bad1);
      E3.Append (Good0); E3.Append (Bad1); E3.Append (Good2);
      E1.Append (Good0);
      F2 := Geom.Fit_On_Ray (E2, Dd, Gate);
      Fb := Geom.Fit_On_Ray (Eb, Dd, Gate);
      F3 := Geom.Fit_On_Ray (E3, Dd, Gate);
      F1 := Geom.Fit_On_Ray (E1, Dd, Gate);
      Check (F2.Ok and then Off (F2) < 1.0e-9 and then F2.Worst < 1.0e-9 and then not Fb.Ok
             and then F3.Ok and then Off (F3) < 1.0e-9 and then Natural (F3.Used.Length) = 2 and then F3.Used (0) = 0 and then F3.Used (1) = 2
             and then not F1.Ok,
             "沿一条视线解一点:两下好的 ⇒ " & (if F2.Ok then "离真点 " & Codec.Fmt (Off (F2), 12) else "没收")
             & " · 一好一坏(另一点离视线 0.5)⇒ " & (if Fb.Ok then "收了(不该),离真点 " & Codec.Fmt (Off (Fb), 4) else "不收")
             & " · 两好一坏 ⇒ " & (if F3.Ok then "挑出 " & Codec.Img (Natural (F3.Used.Length)) & " 下、离真点 " & Codec.Fmt (Off (F3), 12) else "没收")
             & " · 只有一下 ⇒ " & (if F1.Ok then "收了(不该)" else "不收"));
   end;

   --  🔴 ⑦ 合空时的尖进几何文件、读回来(Geom.Save / Geom.Load 的 "shut"):量过的一瓣存三维位置、读回 Shut_Ok;没量的那一瓣不写、读回没量。
   --  病:存不住 ⇒ 下回开机装回身体时合空时的尖丢了,路 5 又只能假设对称;没量的读成量过(全零)⇒ 合拢行程算成到眼上
   declare
      Path : constant String := "/tmp/bd_selfcheck_p2_shut.geo.json";
      Gs, Back : Geom.Geo_Vectors.Vector;
      G : Geom.Cam_Geo;
      Note : String (1 .. 200);
      Ok : Boolean := False;
   begin
      G.Valid := True; G.F := 397.0; G.Cx := 320.0; G.Cy := 240.0;
      G.Tip := [0.0, -0.23, -1.65]; G.Tip_Valid := True; G.Tip_Touch := True; G.Gap := 1.65;
      G.Lobes.Append (Geom.Lobe_Geo'(Tip => [0.83, -0.23, -1.65], Wide => 0.19, Thin => 0.03, Shut => [0.012, -0.231, -1.649], Shut_Ok => True));
      G.Lobes.Append (Geom.Lobe_Geo'(Tip => [-0.82, -0.23, -1.65], Wide => 0.19, Thin => 0.03, others => <>));
      Gs.Append (G);
      Geom.Save (Path, Gs);
      Geom.Load (Path, Back, 1, Note);
      if Natural (Back.Length) = 1 and then Natural (Back (0).Lobes.Length) = 2 then
         declare
            L0 : constant Geom.Lobe_Geo := Back (0).Lobes (0);
            L1 : constant Geom.Lobe_Geo := Back (0).Lobes (1);
         begin
            Ok := L0.Shut_Ok and then abs (L0.Shut (0) - 0.012) < 1.0e-6 and then abs (L0.Shut (1) + 0.231) < 1.0e-6 and then abs (L0.Shut (2) + 1.649) < 1.0e-6
              and then not L1.Shut_Ok;
         end;
      end if;
      Check (Ok, "合空时的尖进几何文件:量过的那一瓣存、读回来是它" & (if Ok then "" else "(不对)") & " · 没量的那一瓣读回来还是没量");
   end;
   --  🔴 ⑧ 压到碰到为止(Touchdown.Descend,碰指尖、合空时的尖都走它;10-01 主代理转来路 8 查的 P8N):
   --  每一步走 Selfmap.Step(压的那种步),碰到 = 它判的"挡住了"(这一步少走的比这一段空走时多出门;底 = 开机探针那几步 + 这一段判成空走的)。
   --  病:原来拿"这一段前两步空走"当底 —— 手底下有东西、每一步只走一半(P8N 人形右手底下的午餐肉罐,手指被顶弯):
   --  头一步没得比、照收成空走的底,之后每一步一样只走一半、一步都不比底多 ⇒ 认不出碰到,大步 / 小步来回找了 250 多拍。
   --  假身体(位姿空间,一条臂;命令隔 1 拍起效、之后每拍走还差的 89%,同 x5 量的;读数加 ±1e-6 的抖动):开机按驱动自己的量法量它
   --  (Selfmap.Measure:探针、起效拍数、静止噪声)。手底下一块东西(顶面 z = Floor):一条命令要往它下面去 ⇒ 这一条只走一半。
   --  要:(a) 从顶面上方 1.5 步起往下 ⇒ 第 2 步碰到;(b) 一开始就压在顶面上 ⇒ 第 1 步碰到;(c) 没有东西 ⇒ 12 步都不认成碰到。
   --  牙:(b) 那一串每一步的账按老底("这一段前两步空走",Selfmap.Blocked)重判一遍 ⇒ 12 步都认不出碰到
   declare
      Lstep : constant Long_Float := 0.036;   --  一小步(合成;同 P8N 人形的小步)
      Tn : constant Long_Float := 0.005;      --  一步看得见的那一档(平移)
      Tr : constant Long_Float := 0.0025;     --  转动那一档
      Noise : constant Long_Float := 1.0e-6;
      Alpha : constant Long_Float := 0.89;
      Start : constant Plug.Arm_Pose := [0.3, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0];
      Into : constant Geom.V3 := [0.0, 0.0, -1.0];
      X, Y, Last_T : Plug.Arm_Pose := Start;
      type Pend is record
         At_Beat : Natural := 0;
         T : Plug.Arm_Pose := [others => 0.0];
      end record;
      package Pend_Vectors is new Ada.Containers.Vectors (Natural, Pend);
      Q : Pend_Vectors.Vector;
      Floor_On : Boolean := False;
      Floor : Long_Float := 0.0;
      Beat : Natural := 0;
      Lk : Plug.Link;
      Pic : Plug.Cam;
      M : Selfmap.Body_Map;
      Measure_Ok : Boolean := False;
      type Job_Kind is (Do_Measure, Do_Descend, Do_Steps);
      Job : Job_Kind := Do_Measure;
      N_Max : constant Natural := 12;
      Got, At_Lim : Boolean := False;
      Used : Natural := 0;
      From : Plug.Arm_Pose;
      Sh : Long_Float;
      Shorts : Bytes.Floats;   --  Do_Steps:每一步沿 Into 少走了多少
      function To_Q (P : Plug.Arm_Pose) return Floats is
         R : Floats;
      begin
         for V of P loop
            R.Append (V);
         end loop;
         return R;
      end To_Q;
      function To_Pose (V : Floats) return Plug.Arm_Pose is
         P : Plug.Arm_Pose := [others => 0.0];
      begin
         for I in P'Range loop
            if I < Natural (V.Length) then
               P (I) := V (I);
            end if;
         end loop;
         return P;
      end To_Pose;
      procedure Fake_Cmd (C : in out Plug.Cmd; Ok : out Boolean) is
      begin
         C.Kind := Plug.Joint; C.Group := Integer (C.Arm); C.Q := To_Q (C.Pose);
         Ok := True;
      end Fake_Cmd;
      function Frame_Now return Plug.Frame is
         Ff : Plug.Frame;
         N : constant Long_Float := Noise * Long_Float ((Beat * 7) mod 5 - 2) / 2.0;
         V : Table.Vec := Table.Zero_Vec;
         P : Plug.Arm_Pose;
      begin
         V (0) := N; V (1) := -N; V (2) := N; V (5) := N;
         P := Chan.Compose (X, V);
         Ff.EE.Append (P);
         Ff.Joints.Append (To_Q (P));
         Ff.Cams.Append (Pic);
         Ff.Seq := Beat;
         return Ff;
      end Frame_Now;
      procedure Advance is
         Mg : constant Plug.Cmd := Plug.Lock_Merged;
      begin
         for K in 0 .. Natural (Mg.Groups.Length) - 1 loop
            if Mg.Groups (K) = 0 then
               declare
                  T : constant Plug.Arm_Pose := To_Pose (Mg.Qs (K));
               begin
                  if (for some I in T'Range => abs (T (I) - Last_T (I)) > 1.0e-9) then
                     Q.Append (Pend'(At_Beat => Beat + 1, T => T));
                     Last_T := T;
                  end if;
               end;
            end if;
         end loop;
         while not Q.Is_Empty and then Q.First_Element.At_Beat <= Beat loop
            declare
               T : constant Plug.Arm_Pose := Q.First_Element.T;
            begin
               --  手底下那块东西:一条命令要往顶面下面去 ⇒ 这一条只走一半(手指被顶弯 / 东西被压下去一点)
               if Floor_On and then T (2) < Floor then
                  declare
                     Half : Table.Vec := Table.Zero_Vec;
                  begin
                     for I in 0 .. 2 loop
                        Half (I) := 0.5 * (T (I) - X (I));
                     end loop;
                     Y := Chan.Compose (X, Half);
                  end;
               else
                  Y := T;
               end if;
            end;
            Q.Delete_First;
         end loop;
         declare
            Go : Table.Vec := Table.Zero_Vec;
         begin
            for I in 0 .. 2 loop
               Go (I) := Alpha * (Y (I) - X (I));
            end loop;
            X := Chan.Compose (X, Go);
         end;
      end Advance;
      procedure Run_Hand is
         task type Hand;
         task body Hand is
            Fr : Plug.Frame := Frame_Now;
         begin
            Lockstep.Begin_Hand (0);
            begin
               case Job is
                  when Do_Measure =>
                     declare
                        Step_Px : Plug.Floats_Vectors.Vector;
                        St : Floats;
                        Eyes : Ints;
                     begin
                        St.Append (Tn); St.Append (Tr);
                        Step_Px.Append (St); Eyes.Append (0);
                        Selfmap.Measure (Lk, Fr, M, Measure_Ok, Step_Px, Eyes => Eyes, World => 0);
                     end;
                  when Do_Descend =>
                     declare
                        W : Selfmap.Walk;
                     begin
                        Touchdown.Descend (Lk, M, Fr, 0, Into, Lstep, N_Max, W, Got, At_Lim, From, Used, Sh);
                     end;
                  when Do_Steps =>
                     declare
                        W : Selfmap.Walk;
                        Hit : Boolean;
                        Rep : Selfmap.Leg_Step;
                     begin
                        for I in 1 .. N_Max loop
                           Touchdown.Step_Down (Lk, M, Fr, 0, Into, Lstep, W, Sh, At_Lim, Hit, Rep);
                           Shorts.Append (Sh);
                        end loop;
                     end;
               end case;
            exception
               when E : others =>
                  Put_Line ("  🔴 路 2 假身体的手出错:" & Ada.Exceptions.Exception_Information (E));
                  Fails := Fails + 1;
            end;
            Lockstep.Done;
         end Hand;
      begin
         Lockstep.Clear;
         Plug.Lock_Begin;
         declare
            H : Hand;
         begin
            Lockstep.Start (0, H'Identity);
            loop
               Lockstep.Run (0);
               exit when Lockstep.Finished (0);
               Beat := Beat + 1;
               begin
                  Advance;
                  declare
                     Ff : constant Plug.Frame := Frame_Now;
                  begin
                     Lk.Seq := Beat;
                     Plug.Note_Beat (Lk, Ff);
                     Plug.Lock_Feed (Ff);
                  end;
               exception
                  when E : others =>
                     --  假身体自己出错:手的任务还停在等这一拍,不收掉它整个自检就卡死
                     Put_Line ("  🔴 路 2 假身体出错:" & Ada.Exceptions.Exception_Information (E));
                     Fails := Fails + 1;
                     abort H;
                     exit;
               end;
            end loop;
         end;
         Plug.Lock_End;
         Lockstep.Clear;
      end Run_Hand;
      procedure Reset (Z0 : Long_Float) is
      begin
         X := Start; X (2) := Z0; Y := X; Last_T := X; Q.Clear;
         Lk.Beats.Clear;
      end Reset;
      Got_A, Got_B, Got_C : Boolean := False;
      Used_A, Used_B, Used_C : Natural := 0;
      Old_Hit : Boolean := False;
   begin
      Pic.W := 8; Pic.H := 8;
      for I in 0 .. Pic.W * Pic.H - 1 loop
         Pic.Gray.Append (U8 (100));
      end loop;
      Plug.Set_Hooks (null, Fake_Cmd'Unrestricted_Access);
      Plug.Set_Reach (null); Plug.Set_Limit (null);
      Reset (Start (2));
      Job := Do_Measure;
      Run_Hand;
      --  (a) 顶面在起点下方 1.5 步
      Reset (Start (2)); Floor_On := True; Floor := Start (2) - 1.5 * Lstep;
      Job := Do_Descend; Run_Hand; Got_A := Got; Used_A := Used;
      --  (b) 一开始就压在顶面上
      Reset (Start (2)); Floor := Start (2);
      Job := Do_Descend; Run_Hand; Got_B := Got; Used_B := Used;
      --  (b) 那一串每一步的账,按老底重判(牙)
      Reset (Start (2)); Floor := Start (2); Shorts.Clear;
      Job := Do_Steps; Run_Hand;
      declare
         P1, P2 : Long_Float := 0.0;
         N_Free : Natural := 0;
      begin
         for S of Shorts loop
            if Selfmap.Blocked (S, P1, P2, N_Free, Lstep, M.EE_Noise) then
               Old_Hit := True;
            end if;
            P2 := P1; P1 := S; N_Free := N_Free + 1;
         end loop;
      end;
      --  (c) 没有东西
      Reset (Start (2)); Floor_On := False;
      Job := Do_Descend; Run_Hand; Got_C := Got; Used_C := Used;
      Plug.Set_Hooks (null, null);
      Check (Measure_Ok and then Got_A and then Used_A = 2 and then Got_B and then Used_B = 1 and then not Got_C and then Used_C = N_Max and then not Old_Hit,
             "压到碰到为止 = Selfmap.Step 判的挡住了:顶面在 1.5 步下 ⇒ " & (if Got_A then "第 " & Codec.Img (Used_A) & " 步碰到" else "没认出碰到") & "(该第 2 步)"
             & " · 一开始就压着 ⇒ " & (if Got_B then "第 " & Codec.Img (Used_B) & " 步碰到" else "没认出碰到") & "(该第 1 步)"
             & " · 没有东西 ⇒ " & (if Got_C then "第 " & Codec.Img (Used_C) & " 步认成碰到(不该)" else Codec.Img (Used_C) & " 步都没认成碰到")
             & " · 牙:压着的那一串按老底(这一段前两步空走)重判 ⇒ " & (if Old_Hit then "认出了(牙没咬住)" else "12 步都认不出")
             & (if Measure_Ok then "" else " · 假身体开机没量成"));
   end;
   --  🔴 ⑨ 画面带 σ 6 的噪声,二指夹爪认成两瓣(10-01 主代理转来路 8 的 P8WN:第 2 只手认出"第 3 瓣")。
   --  病:合上那根手指被照亮的那一侧比身后的地板亮 —— 按"合上以后变亮 / 变暗"分两类时,它和张开时的手指落进同一类;
   --  没噪声时它只有最大块的一成(V1B82 第 1 只手 1854 / 17875 像素),噪声把它的边连大到三成(5247 / 17851),过了"不比最大块小四倍"⇒ 多一瓣。
   --  它在张开那头是背景(地板):张开那头转一下眼,它里面的格点不长在眼上 ⇒ 不是瓣,挪进合到的那一类(From_Frames 的 Open_Ride / Open_Judged)。
   --  合成:200 × 80、背景 100,每个像素加 σ 6 的噪声(两帧各自的,确定的伪随机);张开时两根手指 x 10–29、x 170–189(y 30–79,灰度 20);
   --  合上时两根并在正中 x 70–129,左边那根的左半截 x 70–89 被照亮(180,比背景亮)、其余 20。要:
   --  (a) 只有噪声(没有照亮的那一截)⇒ 两瓣;(b) 有照亮的那一截 ⇒ 只按两张图拼是三瓣;张开那头的格点(手指上的长在眼上、别处是背景)一核 ⇒
   --  那一块不长在眼上、挪走 ⇒ 两瓣,框是两根张开的手指。牙:不核(不给格点)⇒ 三瓣
   declare
      W : constant := 200;
      H : constant := 80;
      Ng : constant Natural := Kinem.Gx * Kinem.Gy;
      Seed : Long_Long_Integer := 12345;
      --  确定的伪随机(线性同余),12 个均匀数之和减 6 ≈ 标准正态(中心极限)
      function Gauss return Long_Float is
         S : Long_Float := 0.0;
      begin
         for I in 1 .. 12 loop
            Seed := (Seed * 1103515245 + 12345) mod 2 ** 31;
            S := S + Long_Float (Seed) / 2.0 ** 31;
         end loop;
         return S - 6.0;
      end Gauss;
      function Noisy (V : Long_Float) return U8 is (U8 (Long_Float'Max (0.0, Long_Float'Min (255.0, Long_Float'Rounding (V + 6.0 * Gauss)))));
      procedure Frames (Lit : Boolean; Open_G, Closed_G : out Buf) is
      begin
         Open_G.Clear; Closed_G.Clear;
         for Y in 0 .. H - 1 loop
            for X in 0 .. W - 1 loop
               Open_G.Append (Noisy (if Y >= 30 and then (X in 10 .. 29 or else X in 170 .. 189) then 20.0 else 100.0));
               Closed_G.Append (Noisy (if Y >= 30 and then X in 70 .. 89 then (if Lit then 180.0 else 20.0)
                                       elsif Y >= 30 and then X in 90 .. 129 then 20.0 else 100.0));
            end loop;
         end loop;
      end Frames;
      Oa, Ca, Ob, Cb : Buf;
      Za, Zb, Zc : Zone.Hand_Zone;
      Ride, Judged : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Ng));
      Dropped : Natural := 0;
      function Finger_Boxes (Z : Zone.Hand_Zone) return Boolean is
        (Z.N_Lobes = 2 and then (for all K in 0 .. 1 => Zone.Lobe_Of (Z, K).Y0 >= 28 and then
           ((Zone.Lobe_Of (Z, K).X0 in 8 .. 12 and then Zone.Lobe_Of (Z, K).X1 in 27 .. 31)
            or else (Zone.Lobe_Of (Z, K).X0 in 168 .. 172 and then Zone.Lobe_Of (Z, K).X1 in 187 .. 191))));
   begin
      Frames (False, Oa, Ca);
      Frames (True, Ob, Cb);
      Za := Zone.From_Frames (Oa, Ca, W, H);
      Zb := Zone.From_Frames (Ob, Cb, W, H);
      --  张开那头转一下眼判的格点:张开的手指上的长在眼上,别处(背景)判得了、不长在眼上
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            declare
               U : constant Long_Float := Kinem.Grid_U (Gxx, W);
               V : constant Long_Float := Kinem.Grid_V (Gyy, H);
            begin
               Judged.Replace_Element (Gyy * Kinem.Gx + Gxx, True);
               Ride.Replace_Element (Gyy * Kinem.Gx + Gxx, V >= 30.0 and then ((U >= 10.0 and then U < 30.0) or else (U >= 170.0 and then U < 190.0)));
            end;
         end loop;
      end loop;
      Zc := Zone.From_Frames (Ob, Cb, W, H, Open_Class => (if Zb.Lobes_Darker then 1 else -1), Open_Ride => Ride, Open_Judged => Judged);
      Dropped := Zc.Moved_Out;
      Check (Za.Valid and then Finger_Boxes (Za) and then Zb.Valid and then Zb.N_Lobes = 3 and then Dropped = 1 and then Zc.Valid and then Finger_Boxes (Zc),
             "画面带 σ 6 噪声的二指夹爪:只有噪声 ⇒ " & Codec.Img (Za.N_Lobes) & " 瓣" & (if Finger_Boxes (Za) then "(两根张开的手指)" else "(不对)")
             & " · 合上那根有一截被照亮(比背景亮)⇒ 只按两张图拼 " & Codec.Img (Zb.N_Lobes) & " 瓣(牙:不核就是它)"
             & " · 按张开那头的格点核 ⇒ 挪走 " & Codec.Img (Dropped) & " 块、" & Codec.Img (Zc.N_Lobes) & " 瓣" & (if Finger_Boxes (Zc) then "(两根张开的手指)" else "(不对)"));
   end;
end Welds_Path_2;
