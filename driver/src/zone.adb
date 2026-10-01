with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Ada.Containers;
with Ada.Unchecked_Deallocation;
with Chan;
with Kinem;
with Instrument;
with Stats;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Zone is
   --  绕世界竖直轴(z)转的那个通道:每条臂的通道前三个平移、后三个绕世界 x / y / z 小转(Chan.Compose)
   Turn_Ch : constant Natural := Chan.Pos_Channels + 2;

   function Turn_Step (M : Selfmap.Body_Map; Arm : Natural) return Table.Vec is
      Av : Table.Vec := Table.Zero_Vec;
      Ch : constant Natural := Arm * M.Per_Arm + Turn_Ch;
   begin
      if Ch < Natural (M.Amp.Length) then
         --  绕世界竖直轴转 64 倍转动探针(倍数,无量纲;同步幅阶梯的顶档,约 0.16 弧度):背景挪几十像素、手指跟着眼不动,眼不挪位置、碰不着东西
         --  (V1B28 2026-09-27:平移 4 倍探针 = 2.9 mm,木纹只挪约 4 像素,后半段灰度地板 26 ⇒ 合到的区里 89% 的像素也"没变",分不开)。
         --  步子按推的这个通道自己的探针幅度定(09-30:原来按第 3 个通道 —— 绕 x 转 —— 的幅度定步子,推的却是第 5 个)
         Av (Turn_Ch) := 64.0 * M.Amp (Ch);
      end if;
      return Av;
   end Turn_Step;

   --  ── 一串瓣(I2)──
   function Lobe_Of (Z : Hand_Zone; I : Natural) return Lobe is
   begin
      if Z.Lobes.Is_Empty then
         --  旧写法写的握区(只填了 A / B 两格):照第 0 步那一版读它的两格。合并时主代理把旧写法换成 Set_Lobes / Lobes_From_Json,连这一段一起删
         return (if I = 0 then Z.A elsif I = 1 then Z.B else No_Lobe);
      end if;
      return (if I < Natural (Z.Lobes.Length) then Z.Lobes (I) else No_Lobe);
   end Lobe_Of;

   procedure Set_Lobes (Z : in out Hand_Zone; Ls : Lobe_Vectors.Vector) is
      function Nth (I : Natural) return Lobe is (if I < Natural (Ls.Length) then Ls (I) else No_Lobe);
   begin
      Z.Lobes := Ls;
      Z.N_Lobes := Natural (Ls.Length);
      --  旧的两格照今天的身体照旧填(= 第 0 / 1 瓣,没有就是 No_Lobe):还直接读 A / B 的地方(selfcheck.adb 几条焊点)不坏
      Z.A := Nth (0);
      Z.B := Nth (1);
   end Set_Lobes;

   procedure Set_Lobe (Z : in out Hand_Zone; I : Natural; Lb : Lobe) is
      Ls : Lobe_Vectors.Vector := Z.Lobes;
   begin
      if Ls.Is_Empty then
         --  旧写法写的握区:它的瓣 = Lobe_Of 读出来的那几格
         for K in 0 .. Z.N_Lobes - 1 loop
            Ls.Append (Lobe_Of (Z, K));
         end loop;
      end if;
      if I < Natural (Ls.Length) then
         Ls.Replace_Element (I, Lb);
      end if;
      Set_Lobes (Z, Ls);
   end Set_Lobe;

   --  身体文件里一瓣几个数(x0, y0, x1, y1, cu, cv, 像素数;和旧文件的 "a" / "b" 同一个排法)
   Lobe_Fields : constant := 7;
   --  合空时手指到的那一截几个数(u, v, 宽, 窄, 伸进来的 u, 伸进来的 v;Section 的排法)
   Shut_Fields : constant := 6;
   --  这一份握区是按哪一版的算法量的(区心、主轴怎么从瓣算;存在 "lobes" 里,协议):
   --  0 = I2 以前("a" / "b" 两格:两瓣的主轴按归一化画幅算,和东西的主轴(像素系)差 0.56° 一类的角)、
   --  1 = I2 第一版(49dfbe1:一瓣的区心按那一瓣自己的形心)、2 = 10-01 起(主轴按像素系;区心按会合的那一点,一瓣时 = 合到的那片)、
   --  3 = 10-01 起存合空时手指到的那一截("shut";碰桌面量合空时的尖要它,老文件里没有 ⇒ 合空重量)、
   --  哪一类是张开时的手指也算离不动的部分多远(一边不动的夹爪;原来两类一样开时按像素多的那一类猜)
   Zone_Rule : constant := 3;

   function Lobes_Json (Z : Hand_Zone) return String is
      R : Unbounded_String := To_Unbounded_String ("{""rule"":" & Codec.Img (Zone_Rule) & ",""list"":[");
   begin
      for K in 0 .. Z.N_Lobes - 1 loop
         declare
            Lb : constant Lobe := Lobe_Of (Z, K);
         begin
            Append (R, (if K > 0 then "," else "") & "[" & Codec.Img (Lb.X0) & "," & Codec.Img (Lb.Y0) & "," & Codec.Img (Lb.X1) & "," & Codec.Img (Lb.Y1)
                    & "," & Json.Number (Lb.Cu) & "," & Json.Number (Lb.Cv) & "," & Codec.Img (Lb.Count) & "]");
         end;
      end loop;
      Append (R, "],""shut"":");
      if Z.Shut.Ok then
         Append (R, "[" & Json.Number (Z.Shut.U) & "," & Json.Number (Z.Shut.V) & "," & Json.Number (Z.Shut.Wide) & "," & Json.Number (Z.Shut.Thin)
                 & "," & Json.Number (Z.Shut.Eu) & "," & Json.Number (Z.Shut.Ev) & "]");
      else
         Append (R, "null");
      end if;
      Append (R, "}");
      return To_String (R);
   end Lobes_Json;

   procedure Lobes_From_Json (D : Json.Doc; Zn : Integer; Z : in out Hand_Zone) is
      use type Json.Kind;
      Ls : Lobe_Vectors.Vector;
      --  一瓣:正好 Lobe_Fields 个数、每个都是有限数才收(写的时候不是有限数的写成 null,读回来是 NaN,不拿它当像素号)
      procedure Take (N : Integer) is
         function Fld (I : Natural) return Long_Float is (Json.Real (D, Json.Child (D, N, I)));
      begin
         if N < 0 or else Json.Count (D, N) /= Lobe_Fields then
            return;
         end if;
         for I in 0 .. Lobe_Fields - 1 loop
            if not Json.Finite (Fld (I)) then
               return;
            end if;
         end loop;
         Ls.Append (Lobe'(Valid => True, X0 => Natural (Fld (0)), Y0 => Natural (Fld (1)), X1 => Natural (Fld (2)), Y1 => Natural (Fld (3)),
                          Cu => Fld (4), Cv => Fld (5), Count => Natural (Fld (6))));
      end Take;
      procedure Take_All (Arr : Integer) is
      begin
         for J in 0 .. Json.Count (D, Arr) - 1 loop
            Take (Json.Child (D, Arr, J));
         end loop;
      end Take_All;
      L : constant Integer := Json.Get (D, Zn, "lobes");
      Rule : Integer := 0;   --  这一份按第几版存的(见 Zone_Rule)
   begin
      if L >= 0 and then Json.Kind_Of (D, L) = Json.J_Obj then
         declare
            Rn : constant Integer := Json.Get (D, L, "rule");
            Sn : constant Integer := Json.Get (D, L, "shut");
         begin
            Rule := (if Rn >= 0 and then Json.Finite (Json.Real (D, Rn)) then Integer (Json.Real (D, Rn)) else 0);
            Take_All (Json.Get (D, L, "list"));
            --  合空时手指到的那一截:正好 Shut_Fields 个有限数才收(null / 没有 ⇒ 没有这一截)
            Z.Shut := (others => <>);
            if Sn >= 0 and then Json.Kind_Of (D, Sn) = Json.J_Arr and then Json.Count (D, Sn) = Shut_Fields
              and then (for all I in 0 .. Shut_Fields - 1 => Json.Finite (Json.Real (D, Json.Child (D, Sn, I))))
            then
               declare
                  function Fld (I : Natural) return Long_Float is (Json.Real (D, Json.Child (D, Sn, I)));
               begin
                  Z.Shut := (Ok => True, U => Fld (0), V => Fld (1), Wide => Fld (2), Thin => Fld (3), Eu => Fld (4), Ev => Fld (5));
               end;
            end if;
         end;
      elsif L >= 0 then
         Rule := 1;   --  I2 第一版:"lobes" 是一串
         Take_All (L);
      else
         --  I2 以前的文件:两格 "a" / "b"(第二格瓣数不到也照样写了),瓣数在 "n_lobes" ⇒ 按它取前几格
         declare
            Old_Keys : constant String := "ab";
            Nk : constant Integer := Json.Get (D, Zn, "n_lobes");
            Want : constant Natural := (if Nk >= 0 and then Json.Finite (Json.Real (D, Nk)) then Natural (Long_Float'Max (0.0, Json.Real (D, Nk))) else 0);
         begin
            for Key of Old_Keys loop
               exit when Natural (Ls.Length) >= Want;
               Take (Json.Get (D, Zn, [Key]));
            end loop;
         end;
      end if;
      Set_Lobes (Z, Ls);
      if Rule /= Zone_Rule then
         --  别的版本的算法存的握区:瓣照样读回来,区心 / 主轴不是这一版的算法算的(差多少要按存它的那一刻的画面重算,文件里没有)⇒
         --  不悄悄差着用:这一格判成没量过,开机合空一次重量(body_driver 里"存的握区里没有它自己那只眼那一格 ⇒ 合空一次补量"那一条)
         declare
            Cn : constant Integer := Json.Get (D, Zn, "cam");
         begin
            Put_Line ("[装] " & (if Cn >= 0 then "第" & Codec.Img (Natural (Long_Float'Max (0.0, Json.Num (D, Cn)))) & " 台相机里" else "")
                      & "存的握区是第 " & Codec.Img (Natural'Max (0, Rule)) & " 版的算法量的(这一版是第 " & Codec.Img (Zone_Rule)
                      & " 版:区心按会合的那一点、主轴按像素系、存合空时手指到的那一截)⇒ 不照用,要重量");
         end;
         Z.Valid := False;
      end if;
   end Lobes_From_Json;

   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean is
      --  只按瓣自己的框判(不外扩):EE2 实测外扩半个框把紧挨着右爪的剪刀当成了"我"
      function In_Box (Lb : Lobe) return Boolean is
         Cx : constant Natural := Natural (R.Cu * Long_Float (W));
         Cy : constant Natural := Natural (R.Cv * Long_Float (Hh));
      begin
         return Lb.Valid and then Cx >= Lb.X0 and then Cx <= Lb.X1 and then Cy >= Lb.Y0 and then Cy <= Lb.Y1;
      end In_Box;
   begin
      if not Z.Valid then
         return False;
      end if;
      --  每一瓣的框都查(原来只查 A / B 两格:第三根手指框里的块不算"我")
      for K in 0 .. Z.N_Lobes - 1 loop
         if In_Box (Lobe_Of (Z, K)) then
            return True;
         end if;
      end loop;
      return False;
   end Is_Self;

   --  一个 2×2 对称矩阵(像素系)的主方向。两个特征值一样大(差不过舍入:各向同性,连全零)⇒ Ok = False(定不出方向)。
   --  特征向量取 (A − L1)v = 0 两行里不相消的那一行((L1 − Vyy, Vxy) 或 (Vxy, L1 − Vxx),哪根轴大取哪一行);
   --  正负号同 Picture.Fill_Shape:第一个分量不为负(为零时第二个为正)
   procedure Principal (Vxx, Vyy, Vxy : Long_Float; Au, Av : out Long_Float; Ok : out Boolean) is
      Half : constant Long_Float := (Vxx - Vyy) / 2.0;   --  两轴之差的一半(纯数学)
      Gap : constant Long_Float := Sqrt (Half * Half + Vxy * Vxy);   --  = (L1 − L2) / 2
      L1 : constant Long_Float := (Vxx + Vyy) / 2.0 + Gap;
      Ax : Long_Float := (if Vxx >= Vyy then L1 - Vyy else Vxy);
      Ay : Long_Float := (if Vxx >= Vyy then Vxy else L1 - Vxx);
   begin
      Au := 0.0; Av := 0.0;
      Ok := Gap > Long_Float'Epsilon * (abs Vxx + abs Vyy);
      if not Ok then
         return;
      end if;
      if Ax < 0.0 or else (Ax = 0.0 and then Ay < 0.0) then
         Ax := -Ax; Ay := -Ay;
      end if;
      Au := Ax / Sqrt (Ax * Ax + Ay * Ay);
      Av := Ay / Sqrt (Ax * Ax + Ay * Ay);
   end Principal;

   --  瓣的主轴(像素系,和 Picture.Region 的主轴同一个系、同一个正负号约定;给脑看的朝向和东西的朝向按它比):
   --  瓣排开的方向 = 各瓣形心散布的主方向(一样的权:排法和每一瓣看着多大无关)—— 两瓣就是瓣心到瓣心那条线;
   --  排法定不出方向(一瓣,或者几瓣均匀排一圈)⇒ 各瓣自己形状(协方差,按 Picture 存的主轴、伸长、各向 1σ 还原)之和的主方向 ——
   --  一瓣就是它自己的主轴。一套算法,不看瓣数
   procedure Lobe_Axis (Rs : Picture.Regions; W, Hh : Natural; Au, Av : out Long_Float) is
      N : constant Long_Float := Long_Float (Rs.Length);
      Mu, Mv, Sxx, Syy, Sxy : Long_Float := 0.0;
      Ok : Boolean;
   begin
      Au := 0.0; Av := 0.0;
      if Rs.Is_Empty then
         return;
      end if;
      --  形心先在归一化画幅里减平均再换成像素:一瓣时差正好是 0。先乘成像素再减,编译器会把乘和减并成一次(FMA),
      --  剩下乘法的舍入(1e-15 像素)被当成一个方向(自检里一根竖着的手指主轴成了 (0.55, −0.83))
      for R of Rs loop
         Mu := Mu + R.Cu; Mv := Mv + R.Cv;
      end loop;
      Mu := Mu / N; Mv := Mv / N;
      for R of Rs loop
         declare
            Dx : constant Long_Float := (R.Cu - Mu) * Long_Float (W);
            Dy : constant Long_Float := (R.Cv - Mv) * Long_Float (Hh);
         begin
            Sxx := Sxx + Dx * Dx; Syy := Syy + Dy * Dy; Sxy := Sxy + Dx * Dy;
         end;
      end loop;
      Principal (Sxx, Syy, Sxy, Au, Av, Ok);
      if Ok then
         return;
      end if;
      Sxx := 0.0; Syy := 0.0; Sxy := 0.0;
      for R of Rs loop
         declare
            --  Fill_Shape 存的:Sig_U / Sig_V = 两轴方向的 1σ(归一化)、主轴 (Au, Av)、伸长 = √(L1 / L2) ⇒ L1 + L2 = Vxx + Vyy、Vxy = (L1 − L2)·Au·Av
            Vxx : constant Long_Float := (R.Sig_U * Long_Float (W)) ** 2;
            Vyy : constant Long_Float := (R.Sig_V * Long_Float (Hh)) ** 2;
            L2 : constant Long_Float := (Vxx + Vyy) / (1.0 + R.Elong ** 2);
            L1 : constant Long_Float := Vxx + Vyy - L2;
         begin
            Sxx := Sxx + Vxx; Syy := Syy + Vyy; Sxy := Sxy + (L1 - L2) * R.Au * R.Av;
         end;
      end loop;
      Principal (Sxx, Syy, Sxy, Au, Av, Ok);
   end Lobe_Axis;

   --  贴画面边(像素下标 P,W × Hh 的画幅)
   function On_Edge (P : Integer; W, Hh : Natural) return Boolean is
     (P mod W = 0 or else P mod W = W - 1 or else P / W = 0 or else P / W = Hh - 1);

   --  一块手指像素 Pix 里离 Edge(那一整块贴画面边的像素:手从画面外伸进来的地方)最远的那一小截:中心、两个跨度(那一截外接框的长边 / 短边 + 1)。
   --  Edge 空 ⇒ 看不出哪头是从画面外伸进来的,Ok = False(不猜)。尖那一截(Tip_Section)和合空时手指到的那一截(Shut_Of)同一个认法
   procedure Far_Band (Pix, Edge : Ints; W, Hh : Natural; U, V, Wide, Thin : out Long_Float; Ok : out Boolean) is
      Band : constant Long_Float := Long_Float (Hh) / 80.0;   --  最远的那一小截有多厚(比例,无量纲)
      Ne : constant Natural := Natural (Edge.Length);
      Dmax : Long_Float := 0.0;
      Su, Sv : Long_Float := 0.0;
      Cnt : Natural := 0;
      Bx0, By0 : Natural := Natural'Last;
      Bx1, By1 : Natural := 0;
      Ds : Floats;
   begin
      U := 0.0; V := 0.0; Wide := 0.0; Thin := 0.0; Ok := False;
      if Ne = 0 or else Pix.Is_Empty or else W = 0 then
         return;
      end if;
      declare
         Ex, Ey : array (0 .. Ne - 1) of Long_Float;
      begin
         for I in 0 .. Ne - 1 loop
            Ex (I) := Long_Float (Edge (I) mod W); Ey (I) := Long_Float (Edge (I) / W);
         end loop;
         for P of Pix loop
            declare
               X : constant Long_Float := Long_Float (P mod W);
               Y : constant Long_Float := Long_Float (P / W);
               D2 : Long_Float := Long_Float'Last;
            begin
               for I in 0 .. Ne - 1 loop
                  D2 := Long_Float'Min (D2, (X - Ex (I)) ** 2 + (Y - Ey (I)) ** 2);
               end loop;
               Ds.Append (Sqrt (D2));
               Dmax := Long_Float'Max (Dmax, Ds.Last_Element);
            end;
         end loop;
      end;
      for I in 0 .. Natural (Pix.Length) - 1 loop
         if Ds (I) >= Dmax - Band then
            declare
               P : constant Natural := Natural (Pix (I));
            begin
               Su := Su + Long_Float (P mod W); Sv := Sv + Long_Float (P / W); Cnt := Cnt + 1;
               Bx0 := Natural'Min (Bx0, P mod W); Bx1 := Natural'Max (Bx1, P mod W);
               By0 := Natural'Min (By0, P / W); By1 := Natural'Max (By1, P / W);
            end;
         end if;
      end loop;
      if Cnt > 0 then
         U := Su / Long_Float (Cnt); V := Sv / Long_Float (Cnt);
         Wide := Long_Float (Natural'Max (Bx1 - Bx0, By1 - By0) + 1);
         Thin := Long_Float (Natural'Min (Bx1 - Bx0, By1 - By0) + 1);
         Ok := True;
      end if;
   end Far_Band;

   --  合空时手指到的那一截:手指像素(Fingers)里 8 邻连成的那几整块里含合到的那片(Gap)的像素的 = 手指合到的地方;它们贴画面边的那些像素 =
   --  手伸进来的地方;合到的那片的像素里离那儿最远的那一小截 = 合空时的尖(Far_Band)。合上的手指和身后的背景一样亮的那一截变化量过不了分界,
   --  合到的那片会被切成几块(V1B82 第 2 只手:合上的两根手指只认出右边那根的下半截、左边一截、尖那一小块),几块一起算 ——
   --  原来只拿含得最多的那一块,尖认到了右边那根手指的边上(392, 328),真的尖在两指会合处。
   --  伸进来的地方(Eu, Ev)= 贴画面边那些像素的中点(同 Lobe_Entry)
   function Shut_Of (Gap, Fingers : Bools; W, Hh : Natural) return Section is
      N : constant Natural := W * Hh;
      type Flag_Array is array (Natural range <>) of Boolean;
      type Flag_Access is access Flag_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);
      Seen : Flag_Access;
      All_Gap, All_Edge, Cur_Gap, Cur_Edge, Stack : Ints;
      S : Section;
   begin
      if N = 0 or else Natural (Gap.Length) < N or else Natural (Fingers.Length) < N then
         return S;
      end if;
      Seen := new Flag_Array'(0 .. N - 1 => False);
      for P0 in 0 .. N - 1 loop
         if Fingers.Element (P0) and then Gap.Element (P0) and then not Seen (P0) then
            Cur_Gap.Clear; Cur_Edge.Clear; Stack.Clear;
            Seen (P0) := True; Stack.Append (P0);
            while not Stack.Is_Empty loop
               declare
                  P : constant Natural := Natural (Stack.Last_Element);
                  Px : constant Integer := P mod W;
                  Py : constant Integer := P / W;
               begin
                  Stack.Delete_Last;
                  if Gap.Element (P) then
                     Cur_Gap.Append (P);
                  end if;
                  if On_Edge (P, W, Hh) then
                     Cur_Edge.Append (P);
                  end if;
                  for Dy in -1 .. 1 loop
                     for Dx in -1 .. 1 loop
                        if (Dx /= 0 or else Dy /= 0) and then Px + Dx in 0 .. W - 1 and then Py + Dy in 0 .. Hh - 1 then
                           declare
                              Q : constant Natural := (Py + Dy) * W + (Px + Dx);
                           begin
                              if not Seen (Q) and then Fingers.Element (Q) then
                                 Seen (Q) := True; Stack.Append (Q);
                              end if;
                           end;
                        end if;
                     end loop;
                  end loop;
               end;
            end loop;
            --  含合到的那片的每一整块都算(种子就是合到的那片的像素,这一块一定含它)
            All_Gap.Append_Vector (Cur_Gap);
            All_Edge.Append_Vector (Cur_Edge);
         end if;
      end loop;
      Free (Seen);
      Far_Band (All_Gap, All_Edge, W, Hh, S.U, S.V, S.Wide, S.Thin, S.Ok);
      if S.Ok then
         declare
            Su, Sv : Long_Float := 0.0;
         begin
            for P of All_Edge loop
               Su := Su + Long_Float (P mod W); Sv := Sv + Long_Float (P / W);
            end loop;
            S.Eu := Su / Long_Float (All_Edge.Length); S.Ev := Sv / Long_Float (All_Edge.Length);
         end;
      end if;
      return S;
   end Shut_Of;

   --  从三张掩膜拼出握区:瓣 = "张开时是手指"那一类里的每一块(大小的门见下),区 = 扫过而张开时不是手指的那片
   procedure Assemble (Z : in out Hand_Zone; Left, Gap : Bools; W, Hh : Natural;
                       Has_Depth : Boolean; Depth_Open, Depth_Closed : Floats; Clean : Bools) is
   begin
      declare
         Lobes : constant Picture.Regions := Picture.Components (Left, W, Hh, Picture.Min_Pixels (W, Hh));
         Gaps : constant Picture.Regions := Picture.Components (Gap, W, Hh, Picture.Min_Pixels (W, Hh));
         Ls : Lobe_Vectors.Vector;
         Rs : Picture.Regions;   --  和 Ls 一一对应的那几块(主轴按它们的形状)
      begin
         if Lobes.Is_Empty then
            return;
         end if;
         --  每一块不比最大块小四倍的都是一瓣(原来只看第二块、最多留两块:五指手只认出两根、三指爪第三根手指的尖永远量不到)。
         --  比它小的是手指边上零碎的几小块(同一根手指被匀色背景切开的边角,V1B78 腕眼里 28–520 像素,最多是最大块的 3%)
         for R of Lobes loop
            if R.Count * 4 >= Lobes (0).Count then
               Ls.Append (Lobe'(Valid => True, X0 => R.X0, Y0 => R.Y0, X1 => R.X1, Y1 => R.Y1, Cu => R.Cu, Cv => R.Cv, Count => R.Count));
               Rs.Append (R);
            end if;
         end loop;
         Set_Lobes (Z, Ls);
         --  区心 = 这几瓣合拢时会合到的那一点("东西会被夹在哪")。一条规则:最小二乘 ——
         --  每一对瓣合拢时在它们的连线中点会合 ⇒ 每一对一条约束"区心 = 这一对的中点";所有对一起的解 = 各瓣形心的平均。
         --  这些约束定不下来的方向(一对都没有 = 这只眼里只有一瓣:另一根手指在画面外,或者本来就只有一瓣)由"手指合拢时到的那片"
         --  (合到的区里最大的一块)的形心定:取最小二乘解里离那一片最近的那个(伪逆解:有一对以上 ⇒ 它不动结果;一对都没有 ⇒ 结果就是它)。
         --  两瓣时 = 两瓣心的中点(EE3 实测"合到处"的形心在手上相机里落到扫过带的上沿 —— 合上的手指和后面的桌面差得不够,那片只认出一半;
         --  10-01 按仿真真值:V1B78–V1B82 腕眼两瓣中点沿合拢方向离指尖真中点 1–2 px,合到的那片 77–96 px)。
         --  一瓣时 = 合到的那片(10-01 按仿真真值:V1B78–V1B82 头顶眼第 1 只手,指尖真中点离它 32–38 px、沿合拢方向 2–3 px;
         --  离那一瓣自己的形心 64–65 px、沿合拢方向 41 px —— I2 那一版改成了后者,改回)
         declare
            N : constant Long_Float := Long_Float (Ls.Length);
            Pairs : constant Long_Float := N * (N - 1.0) / 2.0;   --  几对瓣
            --  伪逆:Pairs⁺ · Pairs(Pairs = 0 ⇒ 0,否则 1;分母只防 0 / 0)
            Gain : constant Long_Float := Pairs / Long_Float'Max (Pairs, Long_Float'Model_Small);
            Mu, Mv : Long_Float := 0.0;
            Ru, Rv : Long_Float;   --  合到的那片的形心(那一片没有 ⇒ 各瓣的平均:没有别的证据)
         begin
            for Lb of Ls loop
               Mu := Mu + Lb.Cu; Mv := Mv + Lb.Cv;
            end loop;
            Mu := Mu / N; Mv := Mv / N;
            Ru := (if Gaps.Is_Empty then Mu else Gaps (0).Cu);
            Rv := (if Gaps.Is_Empty then Mv else Gaps (0).Cv);
            Z.Cu := Gain * Mu + (1.0 - Gain) * Ru;
            Z.Cv := Gain * Mv + (1.0 - Gain) * Rv;
         end;
         Lobe_Axis (Rs, W, Hh, Z.Au, Z.Av);
         --  区框 = 扫过而张开时不是手指的那片(Gap 里不比最大块小十倍的块);张幅 = 它沿主轴伸多长(在归一化画幅里量:主轴折成归一化画幅里的方向)。
         --  那片没有 ⇒ 区框 = 各瓣的框并起来,张幅 = 各瓣的框沿主轴伸多长
         declare
            Du0 : constant Long_Float := Z.Au / Long_Float (W);
            Dv0 : constant Long_Float := Z.Av / Long_Float (Hh);
            Dn : constant Long_Float := Sqrt (Du0 * Du0 + Dv0 * Dv0);
            Du : constant Long_Float := (if Dn > 0.0 then Du0 / Dn else 0.0);
            Dv : constant Long_Float := (if Dn > 0.0 then Dv0 / Dn else 0.0);
            Lo : Long_Float := Long_Float'Last;
            Hi : Long_Float := Long_Float'First;
            X0 : Natural := W; Y0 : Natural := Hh; X1 : Natural := 0; Y1 : Natural := 0;
            procedure Reach (X, Y : Natural) is
               P : constant Long_Float := (Long_Float (X) / Long_Float (W)) * Du + (Long_Float (Y) / Long_Float (Hh)) * Dv;
            begin
               Lo := Long_Float'Min (Lo, P);
               Hi := Long_Float'Max (Hi, P);
            end Reach;
         begin
            if not Gaps.Is_Empty then
               for K in 0 .. Natural (Gaps.Length) - 1 loop
                  if Gaps (K).Count * 10 >= Gaps (0).Count then
                     declare
                        G : constant Picture.Region := Gaps (K);
                     begin
                        X0 := Natural'Min (X0, G.X0); Y0 := Natural'Min (Y0, G.Y0);
                        X1 := Natural'Max (X1, G.X1); Y1 := Natural'Max (Y1, G.Y1);
                        for Y in G.Y0 .. G.Y1 loop
                           for X in G.X0 .. G.X1 loop
                              if Gap.Element (Y * W + X) then
                                 Reach (X, Y);
                              end if;
                           end loop;
                        end loop;
                     end;
                  end if;
               end loop;
            else
               for Lb of Ls loop
                  X0 := Natural'Min (X0, Lb.X0); Y0 := Natural'Min (Y0, Lb.Y0);
                  X1 := Natural'Max (X1, Lb.X1); Y1 := Natural'Max (Y1, Lb.Y1);
                  Reach (Lb.X0, Lb.Y0); Reach (Lb.X1, Lb.Y0); Reach (Lb.X0, Lb.Y1); Reach (Lb.X1, Lb.Y1);
               end loop;
            end if;
            Z.X0 := X0; Z.Y0 := Y0; Z.X1 := X1; Z.Y1 := Y1;
            Z.Span := (if Hi >= Lo then Hi - Lo else 0.0);
         end;
         if Has_Depth then
            Z.Depth := Picture.Region_Depth (Depth_Open, W, Hh, Left, 0.5);
            if Picture.Is_Nan (Z.Depth) then
               Z.Depth := Picture.Region_Depth (Depth_Closed, W, Hh, Clean, 0.25);
            end if;
         else
            Z.Depth := 0.0;
         end if;
         Z.Shut := Shut_Of (Gap, Clean, W, Hh);
         Z.Valid := True;
      end;
   end Assemble;

   --  这一瓣自己那一块:框里每一块手指像素(8 邻连通)各数一数有几个像素落在框里,取最多的那块(像素下标;尖那一截、瓣的像素、
   --  这一瓣的剪影都按它)
   function Lobe_Component (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural) return Ints is
      N : constant Natural := W * Hh;
      type Flag_Array is array (Natural range <>) of Boolean;
      type Flag_Access is access Flag_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);
      Seen : Flag_Access;
      Best, Cur, Stack : Ints;
      Best_In : Natural := 0;
      function In_Box (P : Natural) return Boolean is
        (P mod W in Lb.X0 .. Lb.X1 and then P / W in Lb.Y0 .. Lb.Y1);
   begin
      if not Lb.Valid or else N = 0 or else Natural (Z.Fingers.Length) < N then
         return Best;
      end if;
      Seen := new Flag_Array'(0 .. N - 1 => False);
      for Y in Lb.Y0 .. Natural'Min (Lb.Y1, Hh - 1) loop
         for X in Lb.X0 .. Natural'Min (Lb.X1, W - 1) loop
            declare
               P0 : constant Natural := Y * W + X;
               In_Cnt : Natural := 0;
            begin
               if Z.Fingers.Element (P0) and then not Seen (P0) then
                  Cur.Clear; Stack.Clear;
                  Seen (P0) := True; Stack.Append (P0);
                  while not Stack.Is_Empty loop
                     declare
                        P : constant Natural := Natural (Stack.Last_Element);
                        Px : constant Integer := P mod W;
                        Py : constant Integer := P / W;
                     begin
                        Stack.Delete_Last;
                        Cur.Append (P);
                        if In_Box (P) then
                           In_Cnt := In_Cnt + 1;
                        end if;
                        for Dy in -1 .. 1 loop
                           for Dx in -1 .. 1 loop
                              if (Dx /= 0 or else Dy /= 0) and then Px + Dx in 0 .. W - 1 and then Py + Dy in 0 .. Hh - 1 then
                                 declare
                                    Q : constant Natural := (Py + Dy) * W + (Px + Dx);
                                 begin
                                    if not Seen (Q) and then Z.Fingers.Element (Q) then
                                       Seen (Q) := True; Stack.Append (Q);
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end loop;
                     end;
                  end loop;
                  if In_Cnt > Best_In then
                     Best_In := In_Cnt; Best := Cur;
                  end if;
               end if;
            end;
         end loop;
      end loop;
      Free (Seen);
      return Best;
   end Lobe_Component;

   procedure Tip_Section (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V, Wide, Thin : out Long_Float; Ok : out Boolean) is
      N : constant Natural := W * Hh;
      Best, Edge : Ints;
   begin
      U := 0.0; V := 0.0; Wide := 0.0; Thin := 0.0; Ok := False;
      if not Lb.Valid or else N = 0 or else Natural (Z.Fingers.Length) < N then
         return;
      end if;
      Best := Lobe_Component (Z, Lb, W, Hh);
      for P of Best loop
         if On_Edge (P, W, Hh) then
            Edge.Append (P);
         end if;
      end loop;
      --  这一块一个像素都不贴画面边 ⇒ 看不出哪头是从画面外伸进来的,不猜(Far_Band 给 Ok = False)
      Far_Band (Best, Edge, W, Hh, U, V, Wide, Thin, Ok);
   end Tip_Section;

   function Lobe_Pixels (Z : Hand_Zone; W, Hh : Natural) return Bools is
      N : constant Natural := W * Hh;
      R : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
   begin
      if Natural (Z.Fingers.Length) < N then
         return R;
      end if;
      for I in 0 .. Z.N_Lobes - 1 loop
         for P of Lobe_Component (Z, Lobe_Of (Z, I), W, Hh) loop
            R.Replace_Element (P, True);
         end loop;
      end loop;
      return R;
   end Lobe_Pixels;

   procedure Lobe_Entry (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; Eu, Ev : out Long_Float; Ok : out Boolean) is
      Su, Sv : Long_Float := 0.0;
      Nb : Natural := 0;
   begin
      Eu := 0.0; Ev := 0.0; Ok := False;
      for P of Lobe_Component (Z, Lb, W, Hh) loop
         if P mod W = 0 or else P mod W = W - 1 or else P / W = 0 or else P / W = Hh - 1 then
            Su := Su + Long_Float (P mod W); Sv := Sv + Long_Float (P / W); Nb := Nb + 1;
         end if;
      end loop;
      if Nb > 0 then
         Eu := Su / Long_Float (Nb); Ev := Sv / Long_Float (Nb); Ok := True;
      end if;
   end Lobe_Entry;

   function Lobe_Mask (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; Through : out Boolean) return Bools is
      N : constant Natural := W * Hh;
      R : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Runs : Natural := 0;
   begin
      Through := False;
      if Natural (Z.Fingers.Length) < N or else W = 0 or else Hh = 0 then
         return R;
      end if;
      for P of Lobe_Component (Z, Lb, W, Hh) loop
         R.Replace_Element (P, True);
      end loop;
      --  沿画面四条边绕一圈(顺时针),数这一块贴画面边的有几段:一段 = 从画面外伸进来、尖在画面里;两段以上 = 穿过画面
      declare
         Perim : constant Natural := 2 * (W + Hh) - 4;
         function At_Perim (K : Natural) return Natural is
           (if K < W then K                                          --  上边,从左往右
            elsif K < W + Hh - 1 then (K - W + 1) * W + (W - 1)      --  右边,从上往下
            elsif K < 2 * W + Hh - 2 then (Hh - 1) * W + (2 * W + Hh - 3 - K)   --  下边,从右往左
            else (2 * (W + Hh) - 4 - K) * W);                        --  左边,从下往上
         First_On : constant Boolean := R.Element (At_Perim (0));
         Prev : Boolean := R.Element (At_Perim (Perim - 1));
      begin
         for K in 0 .. Perim - 1 loop
            declare
               Cur : constant Boolean := R.Element (At_Perim (K));
            begin
               if Cur and then not Prev then
                  Runs := Runs + 1;
               end if;
               Prev := Cur;
            end;
         end loop;
         if Runs = 0 and then First_On then
            Runs := 1;   --  一整圈都是它(整幅都是手指)
         end if;
      end;
      Through := Runs > 1;
      return R;
   end Lobe_Mask;

   function From_Frames (Open_G, Closed_G : Buf; W, Hh : Natural; Static : Bools := Bool_Vectors.Empty_Vector;
                         Open_Class : Integer := 0) return Hand_Zone is
      Z : Hand_Zone;
      N : constant Natural := W * Hh;
      Changed : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Clean : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Darker : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));    --  合上后变暗的
      Lighter : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));   --  合上后变亮的
      Ds : Floats;
      T : Long_Float;
      I : Natural := 0;
      Su, Sv : Floats;   --  不动的部分(Static)每个像素的位置(归一化画幅,同 Picture 的形心)
      --  一类有多开(归一化画幅):这一类里几块(不比最大块小四倍的,倍数无量纲)两两形心之间多远;给了不动的部分 ⇒ 还有这几块里的像素
      --  离最近一个不动的像素最远多远(手指伸出去、离手掌和不动的那根有多远),两样取大的那样。
      --  张开时的手指彼此分得开、伸得离手上不动的部分远;合上时挤在一起、蜷在手掌边上 / 贴着不动的那根。
      --  (10-01 按 H4 那几帧:原来按形心离最近一个不动的格点多远 —— 伸直的四指形心离指根、握拳形心离手掌都只有几十像素,第 1 只手照样认反)
      function Spread (Mask : Bools) return Long_Float is
         Comps : constant Picture.Regions := Picture.Components (Mask, W, Hh, Picture.Min_Pixels (W, Hh));
         Best : Long_Float := 0.0;
         function Kept (I : Natural) return Boolean is (Comps (I).Count * 4 >= Comps (0).Count);   --  不比最大块小四倍
      begin
         for A in 0 .. Natural (Comps.Length) - 1 loop
            for B in A + 1 .. Natural (Comps.Length) - 1 loop
               if Kept (A) and then Kept (B) then
                  Best := Long_Float'Max (Best, Sqrt ((Comps (A).Cu - Comps (B).Cu) ** 2 + (Comps (A).Cv - Comps (B).Cv) ** 2));
               end if;
            end loop;
            if Kept (A) and then not Su.Is_Empty then
               for Y in Comps (A).Y0 .. Comps (A).Y1 loop
                  for X in Comps (A).X0 .. Comps (A).X1 loop
                     if Mask.Element (Y * W + X) then
                        declare
                           Pu : constant Long_Float := Long_Float (X) / Long_Float (W);
                           Pv : constant Long_Float := Long_Float (Y) / Long_Float (Hh);
                           Near : Long_Float := Long_Float'Last;
                        begin
                           for S in 0 .. Natural (Su.Length) - 1 loop
                              Near := Long_Float'Min (Near, (Pu - Su (S)) ** 2 + (Pv - Sv (S)) ** 2);
                           end loop;
                           Best := Long_Float'Max (Best, Sqrt (Near));
                        end;
                     end if;
                  end loop;
               end loop;
            end if;
         end loop;
         return Best;
      end Spread;
      function Count_Of (Mask : Bools) return Natural is
         C : Natural := 0;
      begin
         for B of Mask loop
            if B then
               C := C + 1;
            end if;
         end loop;
         return C;
      end Count_Of;
   begin
      if Natural (Open_G.Length) < N or else Natural (Closed_G.Length) < N or else N = 0 then
         return Z;
      end if;
      --  变化量分两拨,每一个像素都进(09-30:原来每 7 个取一个,小相机、手指只占一小块时新的 Split 证不出谷)
      while I < N loop
         Ds.Append (abs (Long_Float (Open_G.Element (I)) - Long_Float (Closed_G.Element (I))));
         I := I + 1;
      end loop;
      T := Picture.Split (Ds);
      if Picture.Is_Nan (T) then
         return Z;   --  两张画面分不出"变了很多"的一拨 ⇒ 这只眼里看不见这只手合拢
      end if;
      for K in 0 .. N - 1 loop
         --  Split 交的分界落在两级正中(k + 0.5):按实数比,取整会进位、漏掉一级
         Changed.Replace_Element (K, Long_Float (abs (Integer (Open_G.Element (K)) - Integer (Closed_G.Element (K)))) > T);
      end loop;
      --  散点扫掉:只留像素数不少于最大块十分之一的连通块(比例,无量纲;同 From_Sweep)
      declare
         Comps : constant Picture.Regions := Picture.Components (Changed, W, Hh, Picture.Min_Pixels (W, Hh));
      begin
         if Comps.Is_Empty then
            return Z;
         end if;
         for K in 0 .. Natural (Comps.Length) - 1 loop
            if Comps (K).Count * 10 >= Comps (0).Count then
               declare
                  R : constant Picture.Region := Comps (K);
               begin
                  for Y in R.Y0 .. R.Y1 loop
                     for X in R.X0 .. R.X1 loop
                        if Changed.Element (Y * W + X) then
                           Clean.Replace_Element (Y * W + X, True);
                        end if;
                     end loop;
                  end loop;
               end;
            end if;
         end loop;
      end;
      for K in 0 .. N - 1 loop
         if Clean.Element (K) then
            if Integer (Closed_G.Element (K)) < Integer (Open_G.Element (K)) then
               Darker.Replace_Element (K, True);
            else
               Lighter.Replace_Element (K, True);
            end if;
         end if;
      end loop;
      --  不动的部分:给了、和画幅一样大,里面不在变了的像素里的那些(变了的就不是"没跟着动")
      if Natural (Static.Length) = N then
         for K in 0 .. N - 1 loop
            if Static.Element (K) and then not Clean.Element (K) then
               Su.Append (Long_Float (K mod W) / Long_Float (W)); Sv.Append (Long_Float (K / W) / Long_Float (Hh));
            end if;
         end loop;
      end if;
      declare
         Sd : constant Long_Float := Spread (Darker);
         Sl : constant Long_Float := Spread (Lighter);
         --  两类一样开 ⇒ 定不下来:照原来按像素多的那一类排一个出来(Open_Known = False,调用方不许照用)
         Dark_Is_Open : constant Boolean :=
           (if Open_Class /= 0 then Open_Class > 0 elsif Sd /= Sl then Sd > Sl else Count_Of (Darker) >= Count_Of (Lighter));
         None : constant Floats := F64_Vectors.Empty_Vector;
      begin
         if Dark_Is_Open then
            Assemble (Z, Darker, Lighter, W, Hh, False, None, None, Clean);
         else
            Assemble (Z, Lighter, Darker, W, Hh, False, None, None, Clean);
         end if;
         Z.Open_Known := Open_Class /= 0 or else Sd /= Sl;
         Z.Lobes_Darker := Dark_Is_Open;
      end;
      Z.Fingers := Clean;
      return Z;
   end From_Frames;

   --  格子 (Gxx, Gyy) 的像素范围:x ∈ [⌊Gxx·W/Gx⌋, ⌊(Gxx+1)·W/Gx⌋ − 1](同 Kinem.Grid_U 的格子)
   function Cell_X0 (Gxx : Natural; W : Natural) return Natural is ((Gxx * W) / Kinem.Gx);
   function Cell_Y0 (Gyy : Natural; Hh : Natural) return Natural is ((Gyy * Hh) / Kinem.Gy);

   function Refine_Probes (Z : Hand_Zone; W, Hh : Natural; Grid_Ride : Bools) return Probe_Vectors.Vector is
      Ng : constant Natural := Kinem.Gx * Kinem.Gy;
      None : constant Integer := -1;
      Owner : array (0 .. Ng - 1) of Integer := [others => None];   --  长在眼上、连到的那一格归哪一瓣
      Queue : Ints;
      Head : Natural := 0;
      R : Probe_Vectors.Vector;
      function Idx (Gxx, Gyy : Natural) return Natural is (Gyy * Kinem.Gx + Gxx);
      function Overlaps (Lb : Lobe; Gxx, Gyy : Natural) return Boolean is
        (Lb.Valid and then Cell_X0 (Gxx, W) <= Lb.X1 and then Cell_X0 (Gxx + 1, W) > Lb.X0
         and then Cell_Y0 (Gyy, Hh) <= Lb.Y1 and then Cell_Y0 (Gyy + 1, Hh) > Lb.Y0);
      function Ride (I : Natural) return Boolean is (I < Natural (Grid_Ride.Length) and then Grid_Ride.Element (I));
   begin
      if W = 0 or else Hh = 0 or else Natural (Grid_Ride.Length) /= Ng then
         return R;
      end if;
      --  起点:和每一瓣的框重叠、判成长在眼上的格子(按瓣的先后;一格同时和两瓣的框重叠就归前一瓣)
      for K in 0 .. Z.N_Lobes - 1 loop
         for Gyy in 0 .. Kinem.Gy - 1 loop
            for Gxx in 0 .. Kinem.Gx - 1 loop
               if Ride (Idx (Gxx, Gyy)) and then Owner (Idx (Gxx, Gyy)) = None and then Overlaps (Lobe_Of (Z, K), Gxx, Gyy) then
                  Owner (Idx (Gxx, Gyy)) := K;
                  Queue.Append (Idx (Gxx, Gyy));
               end if;
            end loop;
         end loop;
      end loop;
      --  沿长在眼上的格子往外连(一层一层,先到先得)
      while Head < Natural (Queue.Length) loop
         declare
            C : constant Natural := Natural (Queue (Head));
            Cx : constant Integer := C mod Kinem.Gx;
            Cy : constant Integer := C / Kinem.Gx;
         begin
            Head := Head + 1;
            for Dy in -1 .. 1 loop
               for Dx in -1 .. 1 loop
                  if Cx + Dx in 0 .. Kinem.Gx - 1 and then Cy + Dy in 0 .. Kinem.Gy - 1 then
                     declare
                        Q : constant Natural := Idx (Cx + Dx, Cy + Dy);
                     begin
                        if Ride (Q) and then Owner (Q) = None then
                           Owner (Q) := Owner (C);
                           Queue.Append (Q);
                        end if;
                     end;
                  end if;
               end loop;
            end loop;
         end;
      end loop;
      --  要问的格子:连到的格子 + 它们四周那一圈;一格旁边(连它自己)有两瓣的格子 ⇒ 不问
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            declare
               Who : Integer := None;
               Clash : Boolean := False;
            begin
               for Dy in -1 .. 1 loop
                  for Dx in -1 .. 1 loop
                     if Gxx + Dx in 0 .. Kinem.Gx - 1 and then Gyy + Dy in 0 .. Kinem.Gy - 1 then
                        declare
                           O : constant Integer := Owner (Idx (Gxx + Dx, Gyy + Dy));
                        begin
                           if O /= None then
                              if Who = None then
                                 Who := O;
                              elsif Who /= O then
                                 Clash := True;
                              end if;
                           end if;
                        end;
                     end if;
                  end loop;
               end loop;
               if Who /= None and then not Clash then
                  for Y in Cell_Y0 (Gyy, Hh) .. Cell_Y0 (Gyy + 1, Hh) - 1 loop
                     for X in Cell_X0 (Gxx, W) .. Cell_X0 (Gxx + 1, W) - 1 loop
                        R.Append (Probe'(U => Long_Float (X) + 0.5, V => Long_Float (Y) + 0.5, Lobe => Natural (Who)));   --  像素中心(半个像素,纯几何)
                     end loop;
                  end loop;
               end if;
            end;
         end loop;
      end loop;
      return R;
   end Refine_Probes;

   procedure Apply_Refine (Z : in out Hand_Zone; W, Hh : Natural; Ps : Probe_Vectors.Vector; Mu, Mv : Bytes.Floats; G : Geom.Cam_Geo; Rot : Geom.V3;
                           Sig : Long_Float; Added : out Natural) is
      use type Kinem.Ride;
      Nv : Natural := 0;
      In_Lobes : constant Bools := Lobe_Pixels (Z, W, Hh);
      --  合到的区(手指在另一头时待的地方;张开这一头那儿是背景):区框里、不在任何一瓣那一块里的手指像素。补进来的像素贴着它一个都不收 ——
      --  不然瓣和合到的区连成一块,瓣的尖(离画面边最远那一截)跑到区的另一角去(离线 V1B69 两只手右边那一瓣:尖 520,281 → 327,288)。
      --  只看区框里的:区框外不属于哪一瓣的是手指边上零碎的几小块,拿它们挡,左边那根手指上半截整片补不进来(同一批画面:尖停在 108,335)
      function In_Zone (Q : Natural) return Boolean is
        (Z.Fingers.Element (Q) and then not In_Lobes.Element (Q) and then Q mod W in Z.X0 .. Z.X1 and then Q / W in Z.Y0 .. Z.Y1);
      function Near_Zone (X, Y : Natural) return Boolean is
      begin
         for Dy in -1 .. 1 loop
            for Dx in -1 .. 1 loop
               if X + Dx in 0 .. W - 1 and then Y + Dy in 0 .. Hh - 1 then
                  if In_Zone ((Y + Dy) * W + (X + Dx)) then
                     return True;
                  end if;
               end if;
            end loop;
         end loop;
         return False;
      end Near_Zone;
   begin
      Added := 0;
      if Natural (Z.Fingers.Length) /= W * Hh then
         return;
      end if;
      for I in 0 .. Natural (Ps.Length) - 1 loop
         if Mu (I) >= 0.0 and then Mv (I) >= 0.0 then
            Nv := Nv + 1;
         end if;
      end loop;
      declare
         Pu, Pv, Bu, Bv : Kinem.Vec (0 .. Nv - 1);
         Which : array (0 .. Natural'Max (1, Nv) - 1) of Natural := [others => 0];
         Rd : Kinem.Ride_Vec (0 .. Nv - 1);
         J : Natural := 0;
      begin
         for I in 0 .. Natural (Ps.Length) - 1 loop
            if Mu (I) >= 0.0 and then Mv (I) >= 0.0 then
               Pu (J) := Ps (I).U; Pv (J) := Ps (I).V; Bu (J) := Mu (I); Bv (J) := Mv (I); Which (J) := Ps (I).Lobe;
               J := J + 1;
            end if;
         end loop;
         Kinem.Classify_Rides (G, Rot, Sig, W, Hh, Pu, Pv, Bu, Bv, Rd);
         --  要补进来的(长在眼上、原来不是手指像素、不挨着合到的区)先记下,再做一遍开运算:只留能整块盖住一个格子(W/Gx × Hh/Gy)的 ——
         --  补全的证据是那张格点(一格一个点),比一格还薄的判不了。09-30 V1B74 第 1 只手右边那一瓣:手指边上一圈 3–6 px、尖旁边一块
         --  10×8 px、一列点子判成长在眼上(挨着手指的一片匀色白墙,眼转以后颜色没变,配点仪器在遮挡边上把"不动"往外带了几个像素:
         --  它们离"没动"中位 1.23 px,真手指 1.31 px,按配点分不开),尖取到那一块上、只有 8 px 宽 ⇒ 落点圈 0.113 单位,手指压在琴边上。
         --  V1B69 那种手指上半截整块缺的(约 50 px 宽)开运算后照样补上
         declare
            N : constant Natural := W * Hh;
            Wc : constant Natural := W / Kinem.Gx;
            Hc : constant Natural := Hh / Kinem.Gy;
            type Nat_Array is array (Natural range <>) of Natural;
            type Nat_Access is access Nat_Array;
            procedure Free is new Ada.Unchecked_Deallocation (Nat_Array, Nat_Access);
            --  Want = 这一像素要补、是第几瓣的(0 = 不补,k + 1 = 第 k 瓣);Sa / Sf = 求和表(右下角含自己,(W+1) × (Hh+1))
            Want : Nat_Access := new Nat_Array'(0 .. N - 1 => 0);
            Sa : Nat_Access := new Nat_Array'(0 .. (W + 1) * (Hh + 1) - 1 => 0);
            Sf : Nat_Access := new Nat_Array'(0 .. (W + 1) * (Hh + 1) - 1 => 0);
            function Sum (T : Nat_Access; X0, Y0, X1, Y1 : Integer) return Integer is   --  [X0, X1) × [Y0, Y1) 里的和
              (Integer (T ((Y1) * (W + 1) + X1)) - Integer (T ((Y0) * (W + 1) + X1)) - Integer (T ((Y1) * (W + 1) + X0)) + Integer (T ((Y0) * (W + 1) + X0)));
         begin
            for I in Rd'Range loop
               if Rd (I) = Kinem.Rides then
                  declare
                     X : constant Natural := Natural (Long_Float'Floor (Pu (I)));
                     Y : constant Natural := Natural (Long_Float'Floor (Pv (I)));
                  begin
                     if X < W and then Y < Hh and then not Z.Fingers.Element (Y * W + X) and then not In_Zone (Y * W + X) and then not Near_Zone (X, Y) then
                        Want (Y * W + X) := Which (I) + 1;
                     end if;
                  end;
               end if;
            end loop;
            if Wc > 0 and then Hc > 0 then
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. W - 1 loop
                     Sa ((Y + 1) * (W + 1) + X + 1) := Sa (Y * (W + 1) + X + 1) + Sa ((Y + 1) * (W + 1) + X) - Sa (Y * (W + 1) + X)
                       + (if Want (Y * W + X) > 0 then 1 else 0);
                  end loop;
               end loop;
               --  Sf:左上角在这一像素、一整格都要补的那些格子的起点
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. W - 1 loop
                     Sf ((Y + 1) * (W + 1) + X + 1) := Sf (Y * (W + 1) + X + 1) + Sf ((Y + 1) * (W + 1) + X) - Sf (Y * (W + 1) + X)
                       + (if X + Wc <= W and then Y + Hc <= Hh and then Sum (Sa, X, Y, X + Wc, Y + Hc) = Wc * Hc then 1 else 0);
                  end loop;
               end loop;
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. W - 1 loop
                     if Want (Y * W + X) > 0
                       and then Sum (Sf, Integer'Max (0, X - Wc + 1), Integer'Max (0, Y - Hc + 1), X + 1, Y + 1) > 0
                     then
                        Z.Fingers.Replace_Element (Y * W + X, True);
                        Added := Added + 1;
                        declare
                           Kl : constant Natural := Want (Y * W + X) - 1;
                           Lb : Lobe := Lobe_Of (Z, Kl);
                        begin
                           if Lb.Valid then
                              Lb.X0 := Natural'Min (Lb.X0, X); Lb.X1 := Natural'Max (Lb.X1, X);
                              Lb.Y0 := Natural'Min (Lb.Y0, Y); Lb.Y1 := Natural'Max (Lb.Y1, Y);
                              Set_Lobe (Z, Kl, Lb);   --  原来第 0 瓣以外一律写进 B:第三瓣补全会盖掉第二瓣
                           end if;
                        end;
                     end if;
                  end loop;
               end loop;
            end if;
            Free (Want); Free (Sa); Free (Sf);
         end;
      end;
   end Apply_Refine;

   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm, K : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean;
                      Host : String; Port : Natural; Eyes : Geom.Geo_Vectors.Vector) is
      --  抓握通道当关节量(V1b ②,2026-09-27):从此刻的读数起往读数变小那边推到头、再往另一边推到头(同关节扫描一个办法:
      --  头一步 = 读数量级那么大,推动了下一步 ×4,挪不到命令的一半、或者手指动过以后哪台相机里都没有一块像素跟着动 = 到头);两头停住的图比出握区(瓣 = 分得开的那一类,
      --  和两张图谁先谁后无关);哪头张开:到最后那一头时胳膊挪一下再挪回来,它自己那只眼里没跟着变的手指像素(长在手上的)落在瓣里多 ⇒ 这一头张开。
      --  原来是"命令 0 = 合空、再张回开机那个读数"—— 读数在 0–1、0 = 合是 x5 的约定。最后停在张开那头(碰桌面量指尖要手指在瓣那儿)
      N_Cams : constant Natural := Natural (F.Cams.Length);
      --  这个通道此刻有没有读数:没有就量不了(不拿编的数当读数推;09-30 原来没读数时 Selfmap.Jaw_Of 给 1.0 = x5"1 = 张开")
      Has_J0 : constant Boolean := Selfmap.Has_Jaw (F, Arm, K);
      Rest : constant Floats := Selfmap.Jaw_All (F, Arm);   --  其余通道保持它们此刻的读数
      function J0 return Long_Float is (Rest (K));   --  开始时的读数(只在 Has_J0 时问)
      Ramp : constant := 4.0;   --  推动了下一步放大几倍(次数,同关节扫描)
      --  推的时候手要停着的位姿 = 等画面静止【之后】的读数(不是刚进来时的):手还在慢慢挪时进来,按进来那一刻的位姿发"停住"命令会把手拽回去,
      --  合爪那几拍整条胳膊跟着动,扫出来的"手指"连着胳膊贴到画面边,记下的位姿也不是画面里那一刻的(G2A 2026-09-24:人形每挪一下要 ~40 拍才停稳,
      --  头顶眼 16 笔里 7 笔因手指贴画面边被拒)
      Pose : Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Target : Floats := Rest;
      Lo_Frame, Hi_Frame : Plug.Cam_Vectors.Vector;
      --  每一头停稳时最后一帧之前那一帧(看没看见动了:两次比较、不共用一帧,Picture.Seen_Twice)
      Lo_Prev, Hi_Prev : Plug.Cam_Vectors.Vector;
      Last_Prev : Plug.Cam_Vectors.Vector;   --  最近一次读画面之前那一帧(Go_Jaw、Wait_Still 走完时 = 停稳的倒数第二帧)
      S_Prev : aliased Plug.Cam_Vectors.Vector;
      Lo_R, Hi_R : Long_Float;   --  两头停住时的读数(Sweep 推到头才给)
      Lo_Steps, Hi_Steps : Natural := 0;
      Moved_Px : array (0 .. Natural'Max (1, N_Cams) - 1) of Natural := [others => 0];   --  整个行程里每台相机跟着动的像素(两头的图比)
      Okg : Boolean;
      Still_Wait : constant := 30;   --  等画面静止最多几拍(次数;量握区之前、转出去以后同一个等法)
      --  发一条抓握命令(其余通道照旧、胳膊停在 Pose),等读数和画面都静止(同原来合空的等法:读数连着两拍不动、每台相机连着两拍不变,至少 3 拍),回读数。
      --  Good = 停住了;等满了还在变 / 这一拍没收到这个通道的读数 ⇒ Good = False,照实说(09-30:原来等满 40 拍照样 Good = True,
      --  还在走的读数被当成"推到这儿停住了",慢的手被 Sweep 误判到头)。R 只在 Good 时有意义
      procedure Go_Jaw (V : Long_Float; R : out Long_Float; Steps : in out Natural; Good : out Boolean) is
         Prev_J : Floats := Selfmap.Jaw_All (F, Arm);   --  上一拍这条臂的抓握读数(没收到 = 空 ⇒ 这一拍判不了停没停)
         Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
         Still : Natural := 0;
      begin
         Target.Replace_Element (K, V);
         Good := False; R := V;
         for Step in 1 .. 40 loop   --  最多等 40 拍(次数)
            declare
               C : Plug.Cmd;
            begin
               C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
               Last_Prev := F.Cams;
               if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
                  return;
               end if;
            end;
            Steps := Steps + 1;
            declare
               J : constant Floats := Selfmap.Jaw_All (F, Arm);
            begin
               if K < Natural (J.Length) and then K < Natural (Prev_J.Length) and then abs (J (K) - Prev_J (K)) <= M.Jaw_Noise
                 and then Selfmap.Pictures_Still (M, Prev_Cams, F.Cams)
               then
                  Still := Still + 1;
               else
                  Still := 0;
               end if;
               Prev_J := J;
               Prev_Cams := F.Cams;
            end;
            Good := Still >= 2 and then Step >= 3;   --  连着两拍读数、画面都不动才算停住(这一拍有读数:Still 是这一拍才数上去的)
            exit when Good;
         end loop;
         if Good then
            R := Selfmap.Jaw_Of (F, Arm, K);
         else
            Put_Line ("[身]   抓握通道推到 " & Codec.Fmt (V, 3) & ":等满了读数 / 画面还在变(或这几拍没收到这个通道的读数)⇒ 这一下没停住,不当到了");
         end if;
      end Go_Jaw;
      --  这一步有没有哪台相机看见一块像素跟着动:推之前停稳的两帧 A1 / A2、推完停稳的两帧 B1 / B2,两次比较、不共用一帧(Picture.Seen_Twice,
      --  同开机认手、逐通道推;原来只数一次比较里超过地板的像素够不够一块,DR1 2026-09-28 头顶眼的渲染闪烁就够)
      function Any_Moved (A1, B1, A2, B2 : Plug.Cam_Vectors.Vector) return Boolean is
      begin
         for C in 0 .. N_Cams - 1 loop
            if not Picture.Seen_Twice (A1 (C).Gray, B1 (C).Gray, A2 (C).Gray, B2 (C).Gray, M.Floors (C), F.Cams (C).W, F.Cams (C).H).Is_Empty then
               return True;
            end if;
         end loop;
         return False;
      end Any_Moved;
      --  往 Dir 那边推到头。Good = 真推到头了(读数挪不到命令的一半 / 手指动过以后画面不再跟着动);推满 12 下还没到头、有一下没停住、
      --  起点这一拍没收到读数 ⇒ Good = False,照实说。R_End 只在 Good 时有意义
      procedure Sweep (Dir : Long_Float; R_End : out Long_Float; Fr, Fr_Prev : out Plug.Cam_Vectors.Vector; Steps : out Natural; Good : out Boolean) is
         R, S, Rn : Long_Float;
         Seen_Move : Boolean := False;   --  这一趟里画面已经跟着动过
         At_End : Boolean := False;
      begin
         Steps := 0; Good := False;
         Fr := F.Cams; Fr_Prev := Last_Prev;
         if not Selfmap.Has_Jaw (F, Arm, K) then
            Put_Line ("[身]   这一拍没收到这个抓握通道的读数 ⇒ 推不了(不拿编的数当起点)");
            return;
         end if;
         R := Selfmap.Jaw_Of (F, Arm, K); R_End := R;
         --  头一步 = 读数量级那么大(max(1, |读数|),同关节扫描的量级取法),推动了下一步 ×4:这里只找两头,不像关节扫描要细采样给运动学
         --  (V1B30 2026-09-27:头一步 3% 时每只手要推七八下、每下都等停稳,一只手 80 多拍;x5 现在往每边两下)
         S := Long_Float'Max (1.0, abs R);
         for Pushes in 1 .. 12 loop   --  最多推 12 下(次数;×4 放大,12 下远超任何读数量级)
            declare
               Before : constant Plug.Cam_Vectors.Vector := F.Cams;
               Before_Prev : constant Plug.Cam_Vectors.Vector := Last_Prev;
               Moved : Boolean;
               Pushed : Boolean;
            begin
               Go_Jaw (R + Dir * S, Rn, Steps, Pushed);
               if not Pushed then
                  return;   --  有一下没停住:这一头量不出(Good = False)
               end if;
               Moved := Any_Moved (Before_Prev, Last_Prev, Before, F.Cams);
               --  到头 = 读数挪不到命令的一半(纯数学的一半),或者这一趟里手指已经在画面里动过、这一下画面什么都没跟着动(读数只是命令的回声的身体)。
               --  动之前画面不动不算到头:x5 合到底以后命令 0–0.185 这一段手指不动、读数照样跟着命令走(V1B28 2026-09-27,往回推两步就被当成了到头)
               At_End := Dir * (Rn - R) < 0.5 * S or else (Seen_Move and then not Moved);
               R := Rn;
               exit when At_End;
               Seen_Move := Seen_Move or else Moved;
               S := S * Ramp;
            end;
         end loop;
         R_End := R;
         Fr := F.Cams;
         Fr_Prev := Last_Prev;
         Good := At_End;
         if not At_End then
            Put_Line ("[身]   推满了还没到头(读数一直跟着命令走、画面也没停过)⇒ 这一头量不出,不当到了");
         end if;
      end Sweep;
   begin
      H := (others => <>);
      H.Arm := Arm;
      H.K := K;
      if Has_J0 then
         H.Open_Reading := J0;
      end if;
      H.Pose := Pose;
      Ok := False;
      if N_Cams = 0 or else Arm >= Natural (F.EE.Length) then
         return;
      end if;
      if not Has_J0 then
         Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道这一拍没有读数 ⇒ 握区量不了(不拿编的数当读数推)");
         return;
      end if;
      declare
         Used : Natural;
         Ok2 : Boolean;
      begin
         Selfmap.Wait_Still (L, M, F, Still_Wait, Used, Ok2, Prev_Pic => S_Prev'Access);
         if not Ok2 then
            --  等满了画面还在变 ⇒ 不在还在动的画面上量握区(09-30:原来超时照样往下量);Used 没到上限 = 线断了
            Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道:先等画面静止,等了" & Natural'Image (Used)
                      & " 拍" & (if Used < Still_Wait then "线断了" else "画面还在变") & " ⇒ 握区这回不量");
            return;
         end if;
         Last_Prev := S_Prev;
         Pose := F.EE (Arm);
         H.Pose := Pose;
         Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道往两头各推到头(先等画面静止:" & Natural'Image (Used) & " 拍;读数从 " & Codec.Fmt (J0, 3) & " 起)…");
      end;
      Sweep (-1.0, Lo_R, Lo_Frame, Lo_Prev, Lo_Steps, Okg);
      if not Okg then
         return;
      end if;
      Sweep (1.0, Hi_R, Hi_Frame, Hi_Prev, Hi_Steps, Okg);
      if not Okg then
         return;
      end if;
      Put_Line ("[身]   往读数变小那边推到头 ⇒ 读数 " & Codec.Fmt (Lo_R, 3) & "(" & Natural'Image (Lo_Steps) & " 拍)· 往另一边推到头 ⇒ 读数 " & Codec.Fmt (Hi_R, 3)
                & "(" & Natural'Image (Hi_Steps) & " 拍)· 行程 " & Codec.Fmt (abs (Hi_R - Lo_R), 3));
      --  每台相机:两头的图 → 握区
      for C in 0 .. N_Cams - 1 loop
         declare
            Cw : constant Natural := F.Cams (C).W;
            Ch : constant Natural := F.Cams (C).H;
            Z : Hand_Zone;
            Mv : constant Bools := Picture.Moved (Lo_Frame (C).Gray, Hi_Frame (C).Gray, M.Floors (C));
            N_Mv : Natural := 0;
            --  两头各停稳两帧:两次比较、不共用一帧都变了的像素连成的块(Picture.Seen_Twice)
            Seen : constant Picture.Regions :=
              Picture.Seen_Twice (Lo_Prev (C).Gray, Hi_Prev (C).Gray, Lo_Frame (C).Gray, Hi_Frame (C).Gray, M.Floors (C), Cw, Ch);
            N_Seen : Natural := 0;
         begin
            for B of Mv loop
               if B then
                  N_Mv := N_Mv + 1;
               end if;
            end loop;
            for Rg of Seen loop
               N_Seen := N_Seen + Rg.Count;
            end loop;
            Moved_Px (C) := N_Seen;
            if Codec.Env ("BL_DUMP") /= "" then
               Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_lo.pgm", Lo_Frame (C).Gray, Cw, Ch);
               Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_hi.pgm", Hi_Frame (C).Gray, Cw, Ch);
            end if;
            --  两头之间这只眼里得真看见东西动过才算看见手指来去(一动没动时"变化量分两拨"分的是噪声,会把一撮噪声点当成一瓣;
            --  G1S 2026-09-24:手抬出画面、缩回身前时,头顶眼各记了一笔落在空桌面上的"指尖")。
            --  看见 = 两次比较、不共用一帧都变了的像素连成块。原来的门是"一次比较里超过地板的像素够一块最小连通块":
            --  DR1 2026-09-28 无人机的抓握通道什么都不带,头顶眼里渲染闪的 0.04% 画面(123 个散点)过了这道门,被当成两瓣手指存进身体图
            if Seen.Is_Empty then
               Z := (others => <>);
            else
               Z := From_Frames (Lo_Frame (C).Gray, Hi_Frame (C).Gray, Cw, Ch);
            end if;
            if Z.Valid then
               if Codec.Env ("BL_DUMP") /= "" then
                  declare
                     Mk : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Ch));
                  begin
                     for Y in Z.Y0 .. Z.Y1 loop
                        for X in Z.X0 .. Z.X1 loop
                           Mk.Replace_Element (Y * Cw + X, 128);
                        end loop;
                     end loop;
                     for Lb of Z.Lobes loop   --  每一瓣都画(原来只画 A / B 两格)
                        if Lb.Valid then
                           for Y in Lb.Y0 .. Lb.Y1 loop
                              for X in Lb.X0 .. Lb.X1 loop
                                 Mk.Replace_Element (Y * Cw + X, 255);
                              end loop;
                           end loop;
                        end if;
                     end loop;
                     Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_zone.pgm", Mk, Cw, Ch);
                  end;
               end if;
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里:" & Natural'Image (Z.N_Lobes) & " 瓣 · 区心 (" &
                         Codec.Fmt (Z.Cu, 3) & "," & Codec.Fmt (Z.Cv, 3) & ") · 区框 " & Codec.Img (Z.X0) & "," & Codec.Img (Z.Y0) & "-" & Codec.Img (Z.X1) & "," & Codec.Img (Z.Y1) &
                         " · 张幅 " & Codec.Fmt (Z.Span, 3) & " 画幅 · 两头之间动过 " & Codec.Fmt (100.0 * Long_Float (N_Mv) / Long_Float (Natural'Max (1, Cw * Ch)), 2) & "% 画面"
                         & " · 两次比较都动的 " & Codec.Img (N_Seen) & " 像素");
            else
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里看不见这只手合拢(两头之间超过地板的 " & Codec.Img (N_Mv) & " 个像素,"
                         & (if Seen.Is_Empty then "两次比较、不共用一帧都变的连不成一块)" else "变的那几块分不出手指)"));
            end if;
            H.Zones.Append (Z);
         end;
      end loop;
      --  这个通道长在哪只手上:两头之间跟着动得最多的那台相机长在哪只手上(量的,不按读数组的下标)
      declare
         Best_C : Natural := 0;
      begin
         for C in 1 .. N_Cams - 1 loop
            if Moved_Px (C) > Moved_Px (Best_C) then
               Best_C := C;
            end if;
         end loop;
         for A in 0 .. Natural (M.Cam_On_Arm.Length) - 1 loop
            if M.Cam_On_Arm (A) = Integer (Best_C) and then A /= Arm then
               Put_Line ("[身]   这个通道推的时候跟着动得最多的是第" & Natural'Image (Best_C) & " 台相机,它长在第" & Natural'Image (A + 1)
                         & " 只手上 ⇒ 这个通道是第" & Natural'Image (A + 1) & " 只手的(读数组的顺序不算数)");
               H.Arm := A;
            end if;
         end loop;
      end;
      --  哪头张开:此刻停在读数大的那头。胳膊绕世界竖直轴转出去一下(Turn_Step)再转回来;转之前、转出去停稳以后,这只手自己那只眼各一帧,
      --  问配点仪器那张格点(Kinem.Gx × Gy,同开机扫描)配到哪;按格点拟合眼转了多少(Kinem.Fit_Eye_Turn),判每个格点长在眼上 / 是世界 / 两种说法分不开。
      --  手指此刻在的那一类里长在眼上的格点多:在瓣(分得开的那一类)里 ⇒ 这一头张开,在合到的区里 ⇒ 另一头张开。
      --  两类各自长在眼上的比例之差不过 Z 倍它自己的标准差(两个比例之差,二项)⇒ 看不出;分不开的格点不算。
      --  原来比灰度(没跟着变、在地板以下的手指像素落在瓣里多 ⇒ 张开)。手指跟着眼不挪,可手一转光照的角度就变:V1B69 2026-09-30 第 1 只手
      --  两根手指整片暗了 13–20 级(平均灰度 32 → 14、40 → 27),这只眼的地板 10,瓣里 68% 的手指像素"变了"、合到的区里 59%,判不出;
      --  V1B65–68 过了只是因为那几炮的地板量在上一步还没停的尾巴上(81 级),把光照的变化盖住了。挪没挪是位置的事,按配点量
      --  (同一批画面离线:瓣里 72 / 73 个格点长在眼上、合到的区里 1 / 59;第 2 只手 74 / 75 对 0 / 65;没转的两帧、转回原处的两帧全是分不开)。
      --  判完停到张开那头,在那一头转一下眼(张开的就是转过的这一头 ⇒ 用同一对画面),瓣按"长在眼上"补全(Refine_Probes / Apply_Refine)
      declare
         Hc : constant Integer := (if H.Arm < Natural (M.Cam_On_Arm.Length) then M.Cam_On_Arm (H.Arm) else -1);
         Hi_Open : Boolean := True;
         Known : Boolean := False;
         Why : Unbounded_String;   --  判不了的时候为什么(空 = 判得了)
      begin
         if Hc >= 0 and then Natural (Hc) < Natural (H.Zones.Length) and then H.Zones (Natural (Hc)).Valid
           and then H.Arm * M.Per_Arm + Turn_Ch < Natural (M.Amp.Length)
         then
            declare
               use type Kinem.Ride;
               Cw : constant Natural := F.Cams (Natural (Hc)).W;
               Ch : constant Natural := F.Cams (Natural (Hc)).H;
               Turn : constant Table.Vec := Turn_Step (M, H.Arm);   --  绕世界竖直轴转出去的那一下(见 Turn_Step)
               Step : constant Long_Float := Turn (Turn_Ch);
               Eye_G : constant Geom.Cam_Geo := (if Natural (Hc) < Natural (Eyes.Length) then Eyes (Natural (Hc)) else Geom.No_Geo);
               Ng : constant Natural := Kinem.Gx * Kinem.Gy;
               type Verdicts is array (0 .. Ng - 1) of Kinem.Ride;
               Vd : Verdicts := [others => Kinem.Unknown];   --  每个格点(按格子号)判成什么
               Rot : Geom.V3 := [0.0, 0.0, 0.0];
               Sig_Px : Long_Float := 0.0;
               Settled : Boolean := True;
               Before, After : Buf;   --  转之前 / 转出去停稳的那一帧(这只手自己那只眼,彩图)
               Before_All, After_All : Plug.Cam_Vectors.Vector;   --  同一对,每台相机(别的眼里手跟着转动挪了 ⇒ 那儿就是此刻手指在的那一类)
               --  转出去一下再转回来,中间那一对画面配格点、拟合、判每个格点;Why 不空 = 这一转用不了
               procedure Turn_Fit (Why_T : in out Unbounded_String) is
                  Used : Natural;
                  Ok2 : Boolean;
                  Pre_Ok : constant Boolean := Plug.Has_Picture (F.Cams (Natural (Hc))) and then F.Cams (Natural (Hc)).W = Cw and then F.Cams (Natural (Hc)).H = Ch
                                               and then Cw * Ch > 0;
                  Pose_Now : constant Plug.Arm_Pose := (if H.Arm < Natural (F.EE.Length) then F.EE (H.Arm) else Pose);
               begin
                  Vd := [others => Kinem.Unknown];
                  Before := F.Cams (Natural (Hc)).RGB;
                  Before_All := F.Cams;
                  for Dir in 0 .. 1 loop
                     declare
                        C : Plug.Cmd;
                     begin
                        C.Kind := Plug.Ee; C.Arm := H.Arm; C.Pose := Pose_Now; C.Jaw := Target;
                        if Dir = 0 then
                           C.Pose := Chan.Compose (Pose_Now, Turn);
                        end if;
                        if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
                           Why_T := To_Unbounded_String ("线断了");
                           return;
                        end if;
                        Selfmap.Wait_Still (L, M, F, Still_Wait, Used, Ok2);
                        if Dir = 0 and then Why_T = Null_Unbounded_String then
                           if not Ok2 then
                              --  转出去以后画面没停住:不拿还在动的画面比(照实说看不出)
                              Why_T := To_Unbounded_String ("转出去以后等了" & Natural'Image (Used) & " 拍画面还没停住");
                           elsif not (Pre_Ok and then Plug.Has_Picture (F.Cams (Natural (Hc))) and then F.Cams (Natural (Hc)).W = Cw
                                      and then F.Cams (Natural (Hc)).H = Ch)
                           then
                              Why_T := To_Unbounded_String ("转之前 / 转之后这只眼有一帧没收到画面");   --  占位的帧比不了
                           elsif not (Eye_G.F > 0.0) then
                              Why_T := To_Unbounded_String ("这只眼的焦距没量过(转了多少投不回画面)");
                           else
                              After := F.Cams (Natural (Hc)).RGB;
                              After_All := F.Cams;
                              declare
                                 Q : Instrument.Match_Vectors.Vector;
                                 Err : Unbounded_String;
                              begin
                                 for Gyy in 0 .. Kinem.Gy - 1 loop
                                    for Gxx in 0 .. Kinem.Gx - 1 loop
                                       Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, Cw), V => Kinem.Grid_V (Gyy, Ch), others => <>));
                                    end loop;
                                 end loop;
                                 declare
                                    --  粗配(同开机扫描问格点)
                                    Mt : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Before, Cw, Ch, After, Cw, Ch, Q, Err, Coarse => True);
                                    Nv : Natural := 0;
                                 begin
                                    if Natural (Mt.Length) /= Natural (Q.Length) then
                                       Why_T := "配点仪器没配成(" & Err & ")";
                                    else
                                       for G of Mt loop
                                          if G.U >= 0.0 and then G.V >= 0.0 then   --  < 0 = 仪器配不出这一点(非有限数记成 -1)
                                             Nv := Nv + 1;
                                          end if;
                                       end loop;
                                       declare
                                          Pu, Pv, Bu, Bv : Kinem.Vec (0 .. Nv - 1);
                                          Rd : Kinem.Ride_Vec (0 .. Nv - 1);
                                          Gi : array (0 .. Natural'Max (1, Nv) - 1) of Natural := [others => 0];
                                          J : Natural := 0;
                                          Fitted : Boolean;
                                       begin
                                          for G in 0 .. Natural (Q.Length) - 1 loop
                                             if Mt (G).U >= 0.0 and then Mt (G).V >= 0.0 then
                                                Pu (J) := Q (G).U; Pv (J) := Q (G).V; Bu (J) := Mt (G).U; Bv (J) := Mt (G).V; Gi (J) := G;
                                                J := J + 1;
                                             end if;
                                          end loop;
                                          Kinem.Fit_Eye_Turn (Eye_G, Cw, Ch, Pu, Pv, Bu, Bv, Rot, Sig_Px, Settled, Fitted);
                                          if not Fitted then
                                             Why_T := To_Unbounded_String ("格点配上的太少,拟合不出眼转了多少");
                                          else
                                             Kinem.Classify_Rides (Eye_G, Rot, Sig_Px, Cw, Ch, Pu, Pv, Bu, Bv, Rd);
                                             for I in Rd'Range loop
                                                Vd (Gi (I)) := Rd (I);
                                             end loop;
                                          end if;
                                       end;
                                    end if;
                                 end;
                              end;
                           end if;
                        end if;
                     end;
                  end loop;
               end Turn_Fit;
               --  按这一转(Before / After / Rot / Sig_Px / Vd)把张开那头的每一瓣补全
               procedure Refine is
                  Z2 : Hand_Zone := H.Zones (Natural (Hc));
                  Gr : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Ng));
                  Ps : Probe_Vectors.Vector;
                  Added : Natural := 0;
                  Tip0 : Floats;   --  补之前每一瓣的尖(v;只进日志)
                  Tip0_Ok : Bools;   --  那一瓣补之前算得出尖
               begin
                  for I in 0 .. Ng - 1 loop
                     Gr.Replace_Element (I, Vd (I) = Kinem.Rides);
                  end loop;
                  for Kl in 0 .. Z2.N_Lobes - 1 loop
                     declare
                        U, V, Wd, Th : Long_Float;
                        Okt : Boolean;
                     begin
                        Tip_Section (Z2, Lobe_Of (Z2, Kl), Cw, Ch, U, V, Wd, Th, Okt);
                        Tip0.Append (V);
                        Tip0_Ok.Append (Okt);
                     end;
                  end loop;
                  Ps := Refine_Probes (Z2, Cw, Ch, Gr);
                  if Ps.Is_Empty then
                     Put_Line ("[身]   瓣按长在眼上补全:长在眼上的格子一格都连不到瓣上 ⇒ 不补");
                     return;
                  end if;
                  declare
                     Q : Instrument.Match_Vectors.Vector;
                     Err : Unbounded_String;
                     Mu, Mv : Floats;
                  begin
                     for P of Ps loop
                        Q.Append (Instrument.Match_Pt'(U => P.U, V => P.V, others => <>));
                     end loop;
                     declare
                        --  细配:手指的边要到像素(格点那一步粗配就够)
                        Mt : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Before, Cw, Ch, After, Cw, Ch, Q, Err, Coarse => False);
                     begin
                        if Natural (Mt.Length) /= Natural (Q.Length) then
                           Put_Line ("[身]   瓣按长在眼上补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素,配点仪器没配成(" & To_String (Err) & ")⇒ 不补");
                           return;
                        end if;
                        for G of Mt loop
                           Mu.Append (G.U); Mv.Append (G.V);
                        end loop;
                     end;
                     Apply_Refine (Z2, Cw, Ch, Ps, Mu, Mv, Eye_G, Rot, Sig_Px, Added);
                  end;
                  H.Zones.Replace_Element (Natural (Hc), Z2);
                  declare
                     Note : Unbounded_String;
                  begin
                     for Kl in 0 .. Z2.N_Lobes - 1 loop
                        declare
                           U, V, Wd, Th : Long_Float;
                           Okt : Boolean;
                        begin
                           Tip_Section (Z2, Lobe_Of (Z2, Kl), Cw, Ch, U, V, Wd, Th, Okt);
                           Append (Note, " · 第" & Natural'Image (Kl + 1) & " 瓣的尖 v "
                                   & (if Kl < Natural (Tip0.Length) and then Tip0_Ok (Kl) then Codec.Fmt (Tip0 (Kl), 1) else "-") & " → "
                                   & (if Okt then Codec.Fmt (V, 1) else "-"));
                        end;
                     end loop;
                     Put_Line ("[身]   瓣按长在眼上补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素,并进手指 " & Codec.Img (Added) & " 个" & To_String (Note));
                  end;
               end Refine;
               --  按这一转(在合空那头转的)把合空时手指到的那一截补全(同张开那头的瓣):合上的手指和身后一样亮的那一截变化量认不出 ——
               --  V1B82 第 2 只手合上的两根手指只认出右边那根的下半截、左边一截、尖那一小块,合空那一截认到右边那根手指的边上(392, 328),
               --  离两尖合空时的真中点(317, 293)83 px。只拿合到的那片(手指像素里不是瓣的那些)当一块、它的框当框,沿长在眼上的格子往外连、
               --  逐像素问(Refine_Probes / Apply_Refine,同瓣),补全以后它那一块离伸进来的地方最远的那一小截 = 合空那一截(Tip_Section / Lobe_Entry 同一认法)
               procedure Refine_Shut is
                  Z2 : Hand_Zone := H.Zones (Natural (Hc));
                  Zt : Hand_Zone := Z2;
                  Gr : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Ng));
                  Ps : Probe_Vectors.Vector;
                  Added : Natural := 0;
                  Il : constant Bools := Lobe_Pixels (Z2, Cw, Ch);
                  U, V, Wd, Th, Eu, Ev : Long_Float;
                  Okt, Oke : Boolean;
               begin
                  if Natural (Z2.Fingers.Length) /= Cw * Ch then
                     return;
                  end if;
                  for I in 0 .. Ng - 1 loop
                     Gr.Replace_Element (I, Vd (I) = Kinem.Rides);
                  end loop;
                  for P in 0 .. Cw * Ch - 1 loop
                     if Il.Element (P) then
                        Zt.Fingers.Replace_Element (P, False);   --  只留合到的那片
                     end if;
                  end loop;
                  Set_Lobes (Zt, Lobe_Vectors.To_Vector (Lobe'(Valid => True, X0 => Z2.X0, Y0 => Z2.Y0, X1 => Z2.X1, Y1 => Z2.Y1, Cu => Z2.Cu, Cv => Z2.Cv, Count => 0), 1));
                  Ps := Refine_Probes (Zt, Cw, Ch, Gr);
                  if not Ps.Is_Empty then
                     declare
                        Q : Instrument.Match_Vectors.Vector;
                        Err : Unbounded_String;
                        Mu, Mv : Floats;
                     begin
                        for P of Ps loop
                           Q.Append (Instrument.Match_Pt'(U => P.U, V => P.V, others => <>));
                        end loop;
                        declare
                           Mt : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Before, Cw, Ch, After, Cw, Ch, Q, Err, Coarse => False);
                        begin
                           if Natural (Mt.Length) = Natural (Q.Length) then
                              for G of Mt loop
                                 Mu.Append (G.U); Mv.Append (G.V);
                              end loop;
                              Apply_Refine (Zt, Cw, Ch, Ps, Mu, Mv, Eye_G, Rot, Sig_Px, Added);
                           else
                              Put_Line ("[身]   合空那一截按长在眼上补全:配点仪器没配成(" & To_String (Err) & ")⇒ 按变化量认的那一截");
                           end if;
                        end;
                     end;
                  end if;
                  Tip_Section (Zt, Lobe_Of (Zt, 0), Cw, Ch, U, V, Wd, Th, Okt);
                  Lobe_Entry (Zt, Lobe_Of (Zt, 0), Cw, Ch, Eu, Ev, Oke);
                  Put_Line ("[身]   合空那一截按长在眼上补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素、并进 " & Codec.Img (Added) & " 个 · 按变化量认的 "
                            & (if Z2.Shut.Ok then "(" & Codec.Fmt (Z2.Shut.U, 1) & "," & Codec.Fmt (Z2.Shut.V, 1) & ")" else "没有")
                            & " → " & (if Okt and then Oke then "(" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ") 宽 " & Codec.Fmt (Wd, 0) & " 窄 " & Codec.Fmt (Th, 0)
                                       else "补完认不出(那一块不贴画面边)"));
                  if Okt and then Oke then
                     Z2.Shut := (Ok => True, U => U, V => V, Wide => Wd, Thin => Th, Eu => Eu, Ev => Ev);
                     H.Zones.Replace_Element (Natural (Hc), Z2);
                  end if;
               end Refine_Shut;
               Z : Hand_Zone := H.Zones (Natural (Hc));
               In_Lobe : Bools;
               Ride_L, Ride_A, N_L, N_A, N_Unk : Natural := 0;
            begin
               if Natural (Z.Fingers.Length) /= Cw * Ch then
                  Why := To_Unbounded_String ("这只眼的握区和画幅对不上");
               else
                  Turn_Fit (Why);
               end if;
               if Why = Null_Unbounded_String then
                  --  哪一类是张开时的手指,也按两头都长在眼上、合拢时没跟着动的那些(手掌、不动的那根手指)算(From_Frames 的 Static):
                  --  转之前 / 转出去这一对里判成长在眼上、又不在手指像素里的格点。按它们重拼这只眼的握区 ——
                  --  一边不动的夹爪只有一块在动,只看两类各自散得多开分不出哪一类是张开的(原来按像素多的那一类猜)
                  declare
                     St : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
                     N_St : Natural := 0;
                  begin
                     for Gyy in 0 .. Kinem.Gy - 1 loop
                        for Gxx in 0 .. Kinem.Gx - 1 loop
                           declare
                              Px : constant Natural := Natural (Long_Float'Floor (Kinem.Grid_V (Gyy, Ch))) * Cw + Natural (Long_Float'Floor (Kinem.Grid_U (Gxx, Cw)));
                           begin
                              if Vd (Gyy * Kinem.Gx + Gxx) = Kinem.Rides and then not Z.Fingers.Element (Px) then
                                 St.Replace_Element (Px, True);
                                 N_St := N_St + 1;
                              end if;
                           end;
                        end loop;
                     end loop;
                     declare
                        Z2 : constant Hand_Zone := From_Frames (Lo_Frame (Natural (Hc)).Gray, Hi_Frame (Natural (Hc)).Gray, Cw, Ch, St);
                        Swapped : constant Boolean := Z2.Valid and then Z2.Lobes_Darker /= Z.Lobes_Darker;
                     begin
                        if Z2.Valid then
                           Z := Z2;
                           H.Zones.Replace_Element (Natural (Hc), Z);
                        end if;
                        Put_Line ("[身]   哪一类是张开时的手指:两头都长在眼上、合拢时没跟着动的格点 " & Codec.Img (N_St) & " 个一起算 ⇒ "
                                  & (if not Z.Open_Known then "还是分不出(两类一样开)"
                                     elsif Swapped then "换成另一类(只看两类各自散得多开时排的是另一类)" else "和只看散得多开时一样"));
                        if not Z.Open_Known then
                           Why := To_Unbounded_String ("这只眼里分不出哪一类是张开时的手指(两类一样开,也没有不动的部分可比)");
                        end if;
                     end;
                  end;
               end if;
               if Why = Null_Unbounded_String then
                  In_Lobe := Lobe_Pixels (Z, Cw, Ch);
                  for Gyy in 0 .. Kinem.Gy - 1 loop
                     for Gxx in 0 .. Kinem.Gx - 1 loop
                        declare
                           Px : constant Natural := Natural (Long_Float'Floor (Kinem.Grid_V (Gyy, Ch))) * Cw + Natural (Long_Float'Floor (Kinem.Grid_U (Gxx, Cw)));
                           Rv : constant Kinem.Ride := Vd (Gyy * Kinem.Gx + Gxx);
                        begin
                           if Z.Fingers.Element (Px) then
                              if Rv = Kinem.Unknown then
                                 N_Unk := N_Unk + 1;
                              elsif In_Lobe.Element (Px) then
                                 N_L := N_L + 1;
                                 if Rv = Kinem.Rides then
                                    Ride_L := Ride_L + 1;
                                 end if;
                              else
                                 N_A := N_A + 1;
                                 if Rv = Kinem.Rides then
                                    Ride_A := Ride_A + 1;
                                 end if;
                              end if;
                           end if;
                        end;
                     end loop;
                  end loop;
                  if N_L = 0 or else N_A = 0 then
                     Why := To_Unbounded_String ("瓣里 / 合到的区里有一类没有判得了的格点");
                  end if;
               end if;
               if Why = Null_Unbounded_String then
                  declare
                     Pl : constant Long_Float := Long_Float (Ride_L) / Long_Float (N_L);
                     Pa : constant Long_Float := Long_Float (Ride_A) / Long_Float (N_A);
                     P : constant Long_Float := Long_Float (Ride_L + Ride_A) / Long_Float (N_L + N_A);
                     --  两个比例之差的标准差(合起来的比例 P 算;1/N_L + 1/N_A 写成 (N_L + N_A) / (N_L·N_A))
                     Sd : constant Long_Float := Sqrt (P * (1.0 - P) * Long_Float (N_L + N_A) / (Long_Float (N_L) * Long_Float (N_A)));
                  begin
                     Known := abs (Pl - Pa) > Stats.Z * Sd;
                     Hi_Open := Pl > Pa;
                     if not Known then
                        Why := To_Unbounded_String ("两类长在眼上的比例 " & Codec.Fmt (Pl, 2) & " / " & Codec.Fmt (Pa, 2) & " 之差不过 " & Codec.Img (Natural (Stats.Z))
                                                    & " 倍它的标准差 " & Codec.Fmt (Sd, 3));
                     end if;
                  end;
               end if;
               Put_Line ("[身]   手绕眼转 " & Codec.Fmt (Step, 3) & " 弧度:它自己那只眼里的格点配到转出去那一帧(配点噪声 " & Codec.Fmt (Sig_Px, 2)
                         & " px" & (if Settled then "" else ",抗野点拟合换了点数那么多轮还在变") & "),长在眼上的 —— 瓣里 " & Codec.Img (Ride_L) & " / " & Codec.Img (N_L)
                         & "、合到的区里 " & Codec.Img (Ride_A) & " / " & Codec.Img (N_A) & "(两种说法分不开的 " & Codec.Img (N_Unk) & " 个不算)"
                         & (if Known then " ⇒ 读数 " & Codec.Fmt ((if Hi_Open then Hi_R else Lo_R), 3) & " 那头张开"
                            else " ⇒ 看不出哪头张开:" & To_String (Why)));
               if Known then
                  --  别的眼里哪一类是张开时的手指定不下来的(只有一块在动,那只眼里又看不出手上哪儿不动):转的那一下手在那只眼里跟着挪,
                  --  挪了的像素落得多的那一类 = 此刻(读数大的那头)手指在的那一类(两类各自挪了的比例之差过 Z 倍它的标准差,同上面长在眼上的判法);
                  --  读数大的那头张开 ⇒ 它就是张开时的手指,否则是另一类。判不出 ⇒ 那只眼的握区不要,照实说
                  for Cc in 0 .. N_Cams - 1 loop
                     if Cc /= Natural (Hc) and then H.Zones (Cc).Valid and then not H.Zones (Cc).Open_Known then
                        declare
                           Zc : constant Hand_Zone := H.Zones (Cc);
                           Wc : constant Natural := F.Cams (Cc).W;
                           Hgc : constant Natural := F.Cams (Cc).H;
                           Nc : constant Natural := Wc * Hgc;
                           M_L, M_A, T_L, T_A : Natural := 0;
                           Done : Boolean := False;
                        begin
                           if Cc < Natural (Before_All.Length) and then Cc < Natural (After_All.Length) and then Cc < Natural (M.Floors.Length)
                             and then Nc > 0 and then Natural (Zc.Fingers.Length) = Nc
                             and then Natural (Before_All (Cc).Gray.Length) = Nc and then Natural (After_All (Cc).Gray.Length) = Nc
                           then
                              declare
                                 Il : constant Bools := Lobe_Pixels (Zc, Wc, Hgc);
                                 Mv : constant Bools := Picture.Moved (Before_All (Cc).Gray, After_All (Cc).Gray, M.Floors (Cc));
                              begin
                                 for P in 0 .. Nc - 1 loop
                                    if Zc.Fingers.Element (P) then
                                       if Il.Element (P) then
                                          T_L := T_L + 1;
                                          if Mv.Element (P) then
                                             M_L := M_L + 1;
                                          end if;
                                       else
                                          T_A := T_A + 1;
                                          if Mv.Element (P) then
                                             M_A := M_A + 1;
                                          end if;
                                       end if;
                                    end if;
                                 end loop;
                              end;
                              if T_L > 0 and then T_A > 0 then
                                 declare
                                    Pl : constant Long_Float := Long_Float (M_L) / Long_Float (T_L);
                                    Pa : constant Long_Float := Long_Float (M_A) / Long_Float (T_A);
                                    P : constant Long_Float := Long_Float (M_L + M_A) / Long_Float (T_L + T_A);
                                    Sd : constant Long_Float := Sqrt (P * (1.0 - P) * Long_Float (T_L + T_A) / (Long_Float (T_L) * Long_Float (T_A)));
                                 begin
                                    if abs (Pl - Pa) > Stats.Z * Sd then
                                       declare
                                          Keep : constant Boolean := (Pl > Pa) = Hi_Open;   --  瓣那一类是不是张开那头手指在的那一类
                                          Z2 : Hand_Zone := (if Keep then Zc
                                                             else From_Frames (Lo_Frame (Cc).Gray, Hi_Frame (Cc).Gray, Wc, Hgc,
                                                                               Open_Class => (if Zc.Lobes_Darker then -1 else 1)));
                                       begin
                                          Z2.Open_Known := True;
                                          H.Zones.Replace_Element (Cc, Z2);
                                          Done := True;
                                          Put_Line ("[身]   第" & Natural'Image (Cc) & " 台相机里哪一类是张开时的手指(只有一块在动、散得多开分不出):转的那一下挪了的像素 —— 瓣里 "
                                                    & Codec.Img (M_L) & " / " & Codec.Img (T_L) & "、合到的区里 " & Codec.Img (M_A) & " / " & Codec.Img (T_A)
                                                    & " ⇒ " & (if Keep then "照原来排的" else "换成另一类"));
                                       end;
                                    end if;
                                 end;
                              end if;
                           end if;
                           if not Done then
                              Put_Line ("[身]   第" & Natural'Image (Cc) & " 台相机里分不出哪一类是张开时的手指(只有一块在动;转的那一下挪了的像素两类分不开:瓣里 "
                                        & Codec.Img (M_L) & " / " & Codec.Img (T_L) & "、合到的区里 " & Codec.Img (M_A) & " / " & Codec.Img (T_A) & ")⇒ 这只眼里的握区不要");
                              H.Zones.Replace_Element (Cc, Hand_Zone'(others => <>));
                           end if;
                        end;
                     end if;
                  end loop;
                  H.Open_Reading := (if Hi_Open then Hi_R else Lo_R);
                  H.Empty_Close := (if Hi_Open then Lo_R else Hi_R);
                  H.Close_Steps := (if Hi_Open then Lo_Steps else Hi_Steps);
                  --  两头各按长在眼上补全:刚转过的这一头(读数大的那头)先补 —— 张开 ⇒ 瓣,合空 ⇒ 合空那一截;再停到另一头转一下补另一样。
                  --  最后停在张开那头,发的就是那一头的读数(不留推到头时 ×4 放出去的那个命令当爪子的目标)
                  declare
                     Rr : Long_Float;
                     St : Natural := 0;
                     Why2 : Unbounded_String;
                  begin
                     if Hi_Open then
                        Refine;
                        Go_Jaw (H.Empty_Close, Rr, St, Okg);
                        if Okg then
                           Turn_Fit (Why2);
                           if Why2 = Null_Unbounded_String then
                              Refine_Shut;
                           else
                              Put_Line ("[身]   合空那一截按长在眼上补全:在合空那头转一下没转成(" & To_String (Why2) & ")⇒ 按变化量认的那一截");
                           end if;
                        else
                           Put_Line ("[身]   合空那一截按长在眼上补全:停到合空那头(读数 " & Codec.Fmt (H.Empty_Close, 3) & ")时没等到停住 ⇒ 按变化量认的那一截");
                        end if;
                        Go_Jaw (H.Open_Reading, Rr, St, Okg);
                     else
                        Refine_Shut;
                        Go_Jaw (H.Open_Reading, Rr, St, Okg);
                        if Okg then
                           Turn_Fit (Why2);
                           if Why2 = Null_Unbounded_String then
                              Refine;
                           else
                              Put_Line ("[身]   瓣按长在眼上补全:在张开那头再转一下没转成(" & To_String (Why2) & ")⇒ 不补");
                           end if;
                        end if;
                     end if;
                     if not Okg then
                        --  两头、握区都量完了;只是停回张开那头时没等到停住 ⇒ 照实说(后面碰桌面前会再等它停)
                        Put_Line ("[身]   两头量完了,停回张开那头(读数 " & Codec.Fmt (H.Open_Reading, 3) & ")时没等到停住");
                     end if;
                  end;
               end if;
            end;
         else
            Why := To_Unbounded_String ("它自己那只眼里没量出握区(或这条臂没有绕竖直轴转的那个通道)");
         end if;
         if not Known then
            Put_Line ("[身]   第" & Natural'Image (H.Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道哪头张开量不出来:" & To_String (Why));
            return;
         end if;
      end;
      H.Measured := True;
      Ok := True;
   end Measure;
end Zone;
