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
            Count_Ok : constant Boolean := Z.Valid and then Z.N_Lobes = N and then not Zone.Lobe_Of (Z, N).Valid;
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
             & " · I2 第一版(一串)⇒ " & (if Ok4 then "读回 2 瓣、判成要重量" else "读错")
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
end Welds_Path_2;
