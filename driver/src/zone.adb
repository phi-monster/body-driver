with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Ada.Containers;
with Ada.Unchecked_Deallocation;
with Chan;
with Table;
package body Zone is
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean is
      --  只按瓣自己的框判(不外扩):EE2 实测外扩半个框把紧挨着右爪的剪刀当成了"我"
      function In_Box (X0, Y0, X1, Y1 : Natural) return Boolean is
         Cx : constant Natural := Natural (R.Cu * Long_Float (W));
         Cy : constant Natural := Natural (R.Cv * Long_Float (Hh));
      begin
         return Cx >= X0 and then Cx <= X1 and then Cy >= Y0 and then Cy <= Y1;
      end In_Box;
   begin
      if not Z.Valid then
         return False;
      end if;
      if Z.A.Valid and then In_Box (Z.A.X0, Z.A.Y0, Z.A.X1, Z.A.Y1) then
         return True;
      end if;
      if Z.B.Valid and then In_Box (Z.B.X0, Z.B.Y0, Z.B.X1, Z.B.Y1) then
         return True;
      end if;
      return False;
   end Is_Self;

   --  从三张掩膜拼出握区:瓣 = "张开时是手指"的连通块(最大的一两块),区 = 手指合到的地方 / 扫过而张开时不是手指的那片
   procedure Assemble (Z : in out Hand_Zone; Left, Arrived, Gap : Bools; W, Hh : Natural;
                       Has_Depth : Boolean; Depth_Open, Depth_Closed : Floats; Clean : Bools) is
      procedure Fill (Lb : in out Lobe; R : Picture.Region) is
      begin
         Lb.Valid := True; Lb.X0 := R.X0; Lb.Y0 := R.Y0; Lb.X1 := R.X1; Lb.Y1 := R.Y1;
         Lb.Cu := R.Cu; Lb.Cv := R.Cv; Lb.Count := R.Count;
      end Fill;
   begin
      declare
         Lobes : constant Picture.Regions := Picture.Components (Left, W, Hh, Picture.Min_Pixels (W, Hh));
         Arr : constant Picture.Regions := Picture.Components (Arrived, W, Hh, Picture.Min_Pixels (W, Hh));
         Gaps : constant Picture.Regions := Picture.Components (Gap, W, Hh, Picture.Min_Pixels (W, Hh));
      begin
         if Lobes.Is_Empty then
            return;
         end if;
         Fill (Z.A, Lobes (0));
         Z.N_Lobes := 1;
         if Natural (Lobes.Length) >= 2 and then Lobes (1).Count * 4 >= Lobes (0).Count then
            Fill (Z.B, Lobes (1));
            Z.N_Lobes := 2;
         end if;
         if Z.N_Lobes = 2 then
            declare
               Du : constant Long_Float := Z.B.Cu - Z.A.Cu;
               Dv : constant Long_Float := Z.B.Cv - Z.A.Cv;
               Ln : constant Long_Float := Sqrt (Du * Du + Dv * Dv);
            begin
               if Ln > 1.0e-9 then
                  Z.Au := Du / Ln; Z.Av := Dv / Ln;
               end if;
            end;
         else
            Z.Au := Lobes (0).Au; Z.Av := Lobes (0).Av;
         end if;
         --  区心:两瓣时 = 两瓣心的中点(EE3 实测"合到处"的形心在手上相机里落到扫过带的上沿,不可靠);
         --  一瓣时 = 手指合到的地方的形心(没有就用扫过区的形心);区框 = 扫过而张开时不是手指的那片;张幅 = 它沿瓣到瓣方向的伸展
         if Z.N_Lobes = 2 then
            Z.Cu := 0.5 * (Z.A.Cu + Z.B.Cu); Z.Cv := 0.5 * (Z.A.Cv + Z.B.Cv);
         elsif not Arr.Is_Empty then
            Z.Cu := Arr (0).Cu; Z.Cv := Arr (0).Cv;
         elsif not Gaps.Is_Empty then
            Z.Cu := Gaps (0).Cu; Z.Cv := Gaps (0).Cv;
         else
            Z.Cu := Z.A.Cu; Z.Cv := Z.A.Cv;
         end if;
         if not Gaps.Is_Empty then
            declare
               Lo : Long_Float := 1.0e30;
               Hi : Long_Float := -1.0e30;
               X0 : Natural := W; Y0 : Natural := Hh; X1 : Natural := 0; Y1 : Natural := 0;
            begin
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
                                 declare
                                    P : constant Long_Float := (Long_Float (X) / Long_Float (W)) * Z.Au + (Long_Float (Y) / Long_Float (Hh)) * Z.Av;
                                 begin
                                    Lo := Long_Float'Min (Lo, P);
                                    Hi := Long_Float'Max (Hi, P);
                                 end;
                              end if;
                           end loop;
                        end loop;
                     end;
                  end if;
               end loop;
               Z.X0 := X0; Z.Y0 := Y0; Z.X1 := X1; Z.Y1 := Y1;
               Z.Span := Long_Float'Max (0.0, Hi - Lo);
            end;
         else
            Z.X0 := Z.A.X0; Z.Y0 := Z.A.Y0; Z.X1 := Z.A.X1; Z.Y1 := Z.A.Y1;
            Z.Span := Long_Float (Z.A.X1 - Z.A.X0) / Long_Float (W);
         end if;
         if Has_Depth then
            Z.Depth := Picture.Region_Depth (Depth_Open, W, Hh, Left, 0.5);
            if Picture.Is_Nan (Z.Depth) then
               Z.Depth := Picture.Region_Depth (Depth_Closed, W, Hh, Clean, 0.25);
            end if;
         else
            Z.Depth := 0.0;
         end if;
         Z.Valid := True;
      end;
   end Assemble;

   function From_Sweep (Swept : Bools; Depth_Open, Depth_Closed : Floats; Has_Depth : Boolean; W, Hh : Natural) return Hand_Zone is
      Z : Hand_Zone;
      N : constant Natural := W * Hh;
      Clean : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Left : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));      --  张开时是手指、合上后不是:瓣
      Arrived : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));   --  合上后是手指、张开时不是:手指合到的地方
      Gap : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));       --  扫过但张开时不是手指:能装东西的区
      Tol : Long_Float := 0.0;
   begin
      Z.Fingers := Swept;
      if Natural (Swept.Length) < N then
         return Z;
      end if;
      --  先把散点扫掉:只留像素数不少于最大块十分之一的连通块(比例,无量纲;渲染器逐帧去噪会撒一地单像素)
      declare
         Comps : constant Picture.Regions := Picture.Components (Swept, W, Hh, Picture.Min_Pixels (W, Hh));
         Keep_Ids : Ints := Int_Vectors.To_Vector (-1, Ada.Containers.Count_Type (N));
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
                        if Swept.Element (Y * W + X) then
                           Keep_Ids.Replace_Element (Y * W + X, K);
                        end if;
                     end loop;
                  end loop;
               end;
            end if;
         end loop;
         for I in 0 .. N - 1 loop
            Clean.Replace_Element (I, Keep_Ids.Element (I) >= 0);
         end loop;
      end;
      Z.Fingers := Clean;
      if Has_Depth and then Natural (Depth_Open.Length) >= N and then Natural (Depth_Closed.Length) >= N then
         --  深度差的噪声地板:没扫过的像素两帧之间抖多少(取 90 分位,比它大的才算"真的换了东西")
         declare
            Ds : Floats;
            I : Natural := 0;
         begin
            while I < N loop
               if not Clean.Element (I) then
                  declare
                     A : constant Long_Float := Depth_Open.Element (I);
                     B : constant Long_Float := Depth_Closed.Element (I);
                  begin
                     if not Picture.Is_Nan (A) and then not Picture.Is_Nan (B) and then A > 0.0 and then B > 0.0 then
                        Ds.Append (abs (A - B));
                     end if;
                  end;
               end if;
               I := I + 7;
            end loop;
            --  90 分位(排序位置,无量纲)
            Tol := (if Natural (Ds.Length) >= 16 then Picture.Quantile (Ds, 0.9) else 0.0);
         end;
         for I in 0 .. N - 1 loop
            if Clean.Element (I) then
               declare
                  A : constant Long_Float := Depth_Open.Element (I);
                  B : constant Long_Float := Depth_Closed.Element (I);
               begin
                  if not Picture.Is_Nan (A) and then not Picture.Is_Nan (B) and then A > 0.0 and then B > 0.0 then
                     if B - A > Tol then
                        Left.Replace_Element (I, True);
                     elsif A - B > Tol then
                        Arrived.Replace_Element (I, True);
                        Gap.Replace_Element (I, True);
                     else
                        Gap.Replace_Element (I, True);    --  两帧都近(手指一直在)或都远(扫过一下又露出来)
                     end if;
                  end if;
               end;
            end if;
         end loop;
      else
         Left := Clean;
         Gap := Clean;
      end if;
      Assemble (Z, Left, Arrived, Gap, W, Hh, Has_Depth, Depth_Open, Depth_Closed, Clean);
      return Z;
   end From_Sweep;

   procedure Tip_Band (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V, Width : out Long_Float; Ok : out Boolean) is
      N : constant Natural := W * Hh;
      Band : constant Long_Float := Long_Float (Hh) / 80.0;   --  最远的那一小截有多厚(比例,无量纲)
      type Flag_Array is array (Natural range <>) of Boolean;
      type Flag_Access is access Flag_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);
      Seen : Flag_Access;
      Best, Cur, Stack : Ints;
      Best_In : Natural := 0;
      function In_Box (P : Natural) return Boolean is
        (P mod W in Lb.X0 .. Lb.X1 and then P / W in Lb.Y0 .. Lb.Y1);
      function On_Edge (P : Natural) return Boolean is
        (P mod W = 0 or else P mod W = W - 1 or else P / W = 0 or else P / W = Hh - 1);
   begin
      U := 0.0; V := 0.0; Width := 0.0; Ok := False;
      if not Lb.Valid or else N = 0 or else Natural (Z.Fingers.Length) < N then
         return;
      end if;
      --  这一瓣自己那一块:框里每一块手指像素(8 邻连通)各数一数有几个像素落在框里,取最多的那块
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
      declare
         Nb : Natural := 0;
      begin
         for P of Best loop
            if On_Edge (P) then
               Nb := Nb + 1;
            end if;
         end loop;
         if Nb = 0 then
            return;   --  这一块一个像素都不贴画面边 ⇒ 看不出哪头是从画面外伸进来的,不猜
         end if;
         declare
            Ex, Ey : array (1 .. Nb) of Long_Float;
            K : Natural := 0;
            Dmax : Long_Float := 0.0;
            Su, Sv : Long_Float := 0.0;
            Cnt : Natural := 0;
            Bx0, By0 : Natural := Natural'Last;
            Bx1, By1 : Natural := 0;
            function Dist (P : Natural) return Long_Float is
               X : constant Long_Float := Long_Float (P mod W);
               Y : constant Long_Float := Long_Float (P / W);
               D2 : Long_Float := Long_Float'Last;
            begin
               for I in 1 .. Nb loop
                  D2 := Long_Float'Min (D2, (X - Ex (I)) ** 2 + (Y - Ey (I)) ** 2);
               end loop;
               return Sqrt (D2);
            end Dist;
         begin
            for P of Best loop
               if On_Edge (P) then
                  K := K + 1;
                  Ex (K) := Long_Float (P mod W); Ey (K) := Long_Float (P / W);
               end if;
            end loop;
            for P of Best loop
               Dmax := Long_Float'Max (Dmax, Dist (P));
            end loop;
            for P of Best loop
               if Dist (P) >= Dmax - Band then
                  Su := Su + Long_Float (P mod W); Sv := Sv + Long_Float (P / W); Cnt := Cnt + 1;
                  Bx0 := Natural'Min (Bx0, P mod W); Bx1 := Natural'Max (Bx1, P mod W);
                  By0 := Natural'Min (By0, P / W); By1 := Natural'Max (By1, P / W);
               end if;
            end loop;
            if Cnt > 0 then
               U := Su / Long_Float (Cnt); V := Sv / Long_Float (Cnt);
               Width := Long_Float (Natural'Max (Bx1 - Bx0, By1 - By0) + 1);
               Ok := True;
            end if;
         end;
      end;
   end Tip_Band;

   function Lobe_Pixels (Z : Hand_Zone; W, Hh : Natural) return Bools is
      N : constant Natural := W * Hh;
      R : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
   begin
      if Natural (Z.Fingers.Length) < N then
         return R;
      end if;
      for I in 0 .. Z.N_Lobes - 1 loop
         declare
            Lb : constant Lobe := Lobe_Of (Z, I);
            Seen : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
            Best, Cur, Stack : Ints;
            Best_In : Natural := 0;
         begin
            if Lb.Valid then
               for Y in Lb.Y0 .. Natural'Min (Lb.Y1, Hh - 1) loop
                  for X in Lb.X0 .. Natural'Min (Lb.X1, W - 1) loop
                     declare
                        P0 : constant Natural := Y * W + X;
                        In_Cnt : Natural := 0;
                     begin
                        if Z.Fingers.Element (P0) and then not Seen.Element (P0) then
                           Cur.Clear; Stack.Clear;
                           Seen.Replace_Element (P0, True); Stack.Append (P0);
                           while not Stack.Is_Empty loop
                              declare
                                 P : constant Natural := Natural (Stack.Last_Element);
                                 Px : constant Integer := P mod W;
                                 Py : constant Integer := P / W;
                              begin
                                 Stack.Delete_Last;
                                 Cur.Append (P);
                                 if Px in Lb.X0 .. Lb.X1 and then Py in Lb.Y0 .. Lb.Y1 then
                                    In_Cnt := In_Cnt + 1;
                                 end if;
                                 for Dy in -1 .. 1 loop
                                    for Dx in -1 .. 1 loop
                                       if (Dx /= 0 or else Dy /= 0) and then Px + Dx in 0 .. W - 1 and then Py + Dy in 0 .. Hh - 1 then
                                          declare
                                             Q : constant Natural := (Py + Dy) * W + (Px + Dx);
                                          begin
                                             if not Seen.Element (Q) and then Z.Fingers.Element (Q) then
                                                Seen.Replace_Element (Q, True); Stack.Append (Q);
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
               for P of Best loop
                  R.Replace_Element (P, True);
               end loop;
            end if;
         end;
      end loop;
      return R;
   end Lobe_Pixels;

   procedure Tip_Px (Z : Hand_Zone; Lb : Lobe; W, Hh : Natural; U, V : out Long_Float; Ok : out Boolean) is
      Wd : Long_Float;
   begin
      Tip_Band (Z, Lb, W, Hh, U, V, Wd, Ok);
   end Tip_Px;

   function From_Frames (Open_G, Closed_G : Buf; W, Hh : Natural) return Hand_Zone is
      Z : Hand_Zone;
      N : constant Natural := W * Hh;
      Changed : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Clean : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Darker : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));    --  合上后变暗的
      Lighter : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));   --  合上后变亮的
      Ds : Floats;
      T : Long_Float;
      I : Natural := 0;
      --  一类里几块(不比最大块小四倍的,倍数无量纲)的形心散得多开:最远的两个形心之间的距离(归一化画幅)
      function Spread (Mask : Bools) return Long_Float is
         Comps : constant Picture.Regions := Picture.Components (Mask, W, Hh, Picture.Min_Pixels (W, Hh));
         Best : Long_Float := 0.0;
      begin
         for A in 0 .. Natural (Comps.Length) - 1 loop
            for B in A + 1 .. Natural (Comps.Length) - 1 loop
               if Comps (A).Count * 4 >= Comps (0).Count and then Comps (B).Count * 4 >= Comps (0).Count then
                  Best := Long_Float'Max (Best, Sqrt ((Comps (A).Cu - Comps (B).Cu) ** 2 + (Comps (A).Cv - Comps (B).Cv) ** 2));
               end if;
            end loop;
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
      --  变化量分两拨(抽样每 7 个像素取一个:次数,无量纲,只为省时间)
      while I < N loop
         Ds.Append (abs (Long_Float (Open_G.Element (I)) - Long_Float (Closed_G.Element (I))));
         I := I + 7;
      end loop;
      T := Picture.Split (Ds);
      if Picture.Is_Nan (T) then
         return Z;   --  两张画面分不出"变了很多"的一拨 ⇒ 这只眼里看不见这只手合拢
      end if;
      for K in 0 .. N - 1 loop
         Changed.Replace_Element (K, abs (Integer (Open_G.Element (K)) - Integer (Closed_G.Element (K))) > Integer (T));
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
      declare
         Sd : constant Long_Float := Spread (Darker);
         Sl : constant Long_Float := Spread (Lighter);
         Dark_Is_Open : constant Boolean := (if Sd /= Sl then Sd > Sl else Count_Of (Darker) >= Count_Of (Lighter));
         None : constant Floats := F64_Vectors.Empty_Vector;
      begin
         if Dark_Is_Open then
            Assemble (Z, Darker, Lighter, Lighter, W, Hh, False, None, None, Clean);
         else
            Assemble (Z, Lighter, Darker, Darker, W, Hh, False, None, None, Clean);
         end if;
      end;
      Z.Fingers := Clean;
      return Z;
   end From_Frames;

   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm, K : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean) is
      --  抓握通道当关节量(V1b ②,2026-09-27):从此刻的读数起往读数变小那边推到头、再往另一边推到头(同关节扫描一个办法:
      --  一步 = 读数量级的 3%,推动了下一步 ×4,挪不到命令的一半、或者哪台相机里都没有一块像素跟着动 = 到头);两头停住的图比出握区(瓣 = 分得开的那一类,
      --  和两张图谁先谁后无关);哪头张开:到最后那一头时胳膊挪一下再挪回来,它自己那只眼里没跟着变的手指像素(长在手上的)落在瓣里多 ⇒ 这一头张开。
      --  原来是"命令 0 = 合空、再张回开机那个读数"—— 读数在 0–1、0 = 合是 x5 的约定。最后停在张开那头(碰桌面量指尖要手指在瓣那儿)
      N_Cams : constant Natural := Natural (F.Cams.Length);
      J0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm, K);
      Rest : constant Floats := Selfmap.Jaw_All (F, Arm);   --  其余通道保持它们此刻的读数
      Ramp : constant := 4.0;   --  推动了下一步放大几倍(次数,同关节扫描)
      --  推的时候手要停着的位姿 = 等画面静止【之后】的读数(不是刚进来时的):手还在慢慢挪时进来,按进来那一刻的位姿发"停住"命令会把手拽回去,
      --  合爪那几拍整条胳膊跟着动,扫出来的"手指"连着胳膊贴到画面边,记下的位姿也不是画面里那一刻的(G2A 2026-09-24:人形每挪一下要 ~40 拍才停稳,
      --  头顶眼 16 笔里 7 笔因手指贴画面边被拒)
      Pose : Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Target : Floats;
      Lo_Frame, Hi_Frame : Plug.Cam_Vectors.Vector;
      Lo_R, Hi_R : Long_Float := J0;
      Lo_Steps, Hi_Steps : Natural := 0;
      Moved_Px : array (0 .. Natural'Max (1, N_Cams) - 1) of Natural := [others => 0];   --  整个行程里每台相机跟着动的像素(两头的图比)
      Okg : Boolean;
      --  发一条抓握命令(其余通道照旧、胳膊停在 Pose),等读数和画面都静止(同原来合空的等法:读数连着两拍不动、每台相机连着两拍不变,至少 3 拍),回读数
      procedure Go_Jaw (V : Long_Float; R : out Long_Float; Steps : in out Natural; Good : out Boolean) is
         Prev_J : Long_Float := Selfmap.Jaw_Of (F, Arm, K);
         Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
         Still : Natural := 0;
      begin
         Target.Replace_Element (K, V);
         Good := False; R := Prev_J;
         for Step in 1 .. 40 loop   --  最多等 40 拍(次数)
            declare
               C : Plug.Cmd;
            begin
               C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
               if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
                  return;
               end if;
            end;
            Steps := Steps + 1;
            declare
               J : constant Long_Float := Selfmap.Jaw_Of (F, Arm, K);
            begin
               if abs (J - Prev_J) <= M.Jaw_Noise and then Selfmap.Pictures_Still (M, Prev_Cams, F.Cams) then
                  Still := Still + 1;
               else
                  Still := 0;
               end if;
               Prev_J := J;
               Prev_Cams := F.Cams;
            end;
            exit when Still >= 2 and then Step >= 3;
         end loop;
         R := Selfmap.Jaw_Of (F, Arm, K);
         Good := True;
      end Go_Jaw;
      --  这一步有没有哪台相机里一块像素跟着动(超过它自己量的静止噪声地板、不少于一块最小连通块 —— 同 Components 的下限)
      function Any_Moved (A, B : Plug.Cam_Vectors.Vector) return Boolean is
      begin
         for C in 0 .. N_Cams - 1 loop
            declare
               Mv : constant Bools := Picture.Moved (A (C).Gray, B (C).Gray, M.Floors (C));
               N_Mv : Natural := 0;
            begin
               for Bb of Mv loop
                  if Bb then
                     N_Mv := N_Mv + 1;
                  end if;
               end loop;
               if N_Mv >= Picture.Min_Pixels (F.Cams (C).W, F.Cams (C).H) then
                  return True;
               end if;
            end;
         end loop;
         return False;
      end Any_Moved;
      --  往 Dir 那边推到头
      procedure Sweep (Dir : Long_Float; R_End : out Long_Float; Fr : out Plug.Cam_Vectors.Vector; Steps : out Natural; Good : out Boolean) is
         R : Long_Float := Selfmap.Jaw_Of (F, Arm, K);
         S : Long_Float := 0.03 * Long_Float'Max (1.0, abs R);   --  头一步 = 读数量级的 3%(比例,同关节扫描)
         Rn : Long_Float;
         Seen_Move : Boolean := False;   --  这一趟里画面已经跟着动过
      begin
         Steps := 0; Good := False; R_End := R;
         for Pushes in 1 .. 12 loop   --  最多推 12 下(次数;×4 放大,12 下远超任何读数量级)
            declare
               Before : constant Plug.Cam_Vectors.Vector := F.Cams;
               Moved : Boolean;
            begin
               Go_Jaw (R + Dir * S, Rn, Steps, Good);
               exit when not Good;
               Moved := Any_Moved (Before, F.Cams);
               --  到头 = 读数挪不到命令的一半(纯数学的一半),或者这一趟里手指已经在画面里动过、这一下画面什么都没跟着动(读数只是命令的回声的身体)。
               --  动之前画面不动不算到头:x5 合到底以后命令 0–0.185 这一段手指不动、读数照样跟着命令走(V1B28 2026-09-27,往回推两步就被当成了到头)
               exit when Dir * (Rn - R) < 0.5 * S or else (Seen_Move and then not Moved);
               Seen_Move := Seen_Move or else Moved;
               R := Rn;
               S := S * Ramp;
            end;
         end loop;
         R_End := Selfmap.Jaw_Of (F, Arm, K);
         Fr := F.Cams;
      end Sweep;
   begin
      H := (others => <>);
      H.Arm := Arm;
      H.K := K;
      H.Open_Reading := J0;
      H.Pose := Pose;
      Ok := False;
      if N_Cams = 0 or else Arm >= Natural (F.EE.Length) then
         return;
      end if;
      for I in 0 .. Natural'Max (1, Natural (Rest.Length)) - 1 loop
         Target.Append (if I < Natural (Rest.Length) then Rest (I) else J0);
      end loop;
      declare
         Used : Natural;
         Ok2 : Boolean;
      begin
         Selfmap.Wait_Still (L, M, F, 30, Used, Ok2);
         if not Ok2 then
            return;
         end if;
         Pose := F.EE (Arm);
         H.Pose := Pose;
         Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道往两头各推到头(先等画面静止:" & Natural'Image (Used) & " 拍;读数从 " & Codec.Fmt (J0, 3) & " 起)…");
      end;
      Sweep (-1.0, Lo_R, Lo_Frame, Lo_Steps, Okg);
      if not Okg then
         return;
      end if;
      Sweep (1.0, Hi_R, Hi_Frame, Hi_Steps, Okg);
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
         begin
            for B of Mv loop
               if B then
                  N_Mv := N_Mv + 1;
               end if;
            end loop;
            Moved_Px (C) := N_Mv;
            if Codec.Env ("BL_DUMP") /= "" then
               Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_lo.pgm", Lo_Frame (C).Gray, Cw, Ch);
               Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_hi.pgm", Hi_Frame (C).Gray, Cw, Ch);
            end if;
            --  两头之间这只眼里得真有像素动过才算看见手指来去(一动没动时"变化量分两拨"分的是噪声,会把一撮噪声点当成一瓣;
            --  G1S 2026-09-24:手抬出画面、缩回身前时,头顶眼各记了一笔落在空桌面上的"指尖")
            if N_Mv < Picture.Min_Pixels (Cw, Ch) then
               Z := (others => <>);
            else
               Z := From_Frames (Lo_Frame (C).Gray, Hi_Frame (C).Gray, Cw, Ch);
            end if;
            if Z.Valid then
               if Codec.Env ("BL_DUMP") /= "" then
                  declare
                     Mk : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Ch));
                     Lobes_Of_Z : constant array (1 .. 2) of Lobe := [Z.A, Z.B];
                  begin
                     for Y in Z.Y0 .. Z.Y1 loop
                        for X in Z.X0 .. Z.X1 loop
                           Mk.Replace_Element (Y * Cw + X, 128);
                        end loop;
                     end loop;
                     for Lb of Lobes_Of_Z loop
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
                         " · 张幅 " & Codec.Fmt (Z.Span, 3) & " 画幅 · 两头之间动过 " & Codec.Fmt (100.0 * Long_Float (N_Mv) / Long_Float (Natural'Max (1, Cw * Ch)), 2) & "% 画面");
            else
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里看不见这只手合拢");
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
      --  哪头张开:此刻停在读数大的那头。胳膊按平移探针幅度挪一下再挪回来,它自己那只眼里没跟着变的手指像素 = 长在手上、此刻手指在的地方;
      --  落在瓣里(瓣自己那一块)比落在合到的区里多 ⇒ 这一头张开
      declare
         Hc : constant Integer := (if H.Arm < Natural (M.Cam_On_Arm.Length) then M.Cam_On_Arm (H.Arm) else -1);
         Hi_Open : Boolean := True;
         Known : Boolean := False;
      begin
         if Hc >= 0 and then Natural (Hc) < Natural (H.Zones.Length) and then H.Zones (Natural (Hc)).Valid
           and then H.Arm * M.Per_Arm < Natural (M.Amp.Length)
         then
            declare
               Z : constant Hand_Zone := H.Zones (Natural (Hc));
               Cw : constant Natural := F.Cams (Natural (Hc)).W;
               Ch : constant Natural := F.Cams (Natural (Hc)).H;
               Pre : constant Buf := F.Cams (Natural (Hc)).Gray;
               --  绕世界竖直轴转 64 倍转动探针(倍数,无量纲;同步幅阶梯的顶档,约 0.16 弧度):背景挪几十像素、手指跟着眼不动,眼不挪位置、碰不着东西
               --  (V1B28 2026-09-27:平移 4 倍探针 = 2.9 mm,木纹只挪约 4 像素,后半段灰度地板 26 ⇒ 合到的区里 89% 的像素也"没变",分不开)
               Step : constant Long_Float := 64.0 * M.Amp (H.Arm * M.Per_Arm + 3);
               In_Lobe : constant Bools := Lobe_Pixels (Z, Cw, Ch);
               Stat_L, Stat_A, N_L, N_A : Natural := 0;
               Used : Natural;
               Ok2 : Boolean;
            begin
               for Dir in 0 .. 1 loop
                  declare
                     C : Plug.Cmd;
                  begin
                     C.Kind := Plug.Ee; C.Arm := H.Arm; C.Pose := Pose; C.Jaw := Target;
                     if Dir = 0 then
                        declare
                           Av : Table.Vec := Table.Zero_Vec;
                        begin
                           Av (5) := Step;
                           C.Pose := Chan.Compose (Pose, Av);
                        end;
                     end if;
                     if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
                        return;
                     end if;
                     Selfmap.Wait_Still (L, M, F, 30, Used, Ok2);
                     if Dir = 0 then
                        declare
                           Mv : constant Bools := Picture.Moved (Pre, F.Cams (Natural (Hc)).Gray, M.Floors (Natural (Hc)));
                        begin
                           for I in 0 .. Cw * Ch - 1 loop
                              if I < Natural (Z.Fingers.Length) and then Z.Fingers.Element (I) then
                                 if In_Lobe.Element (I) then
                                    N_L := N_L + 1;
                                    if not Mv.Element (I) then
                                       Stat_L := Stat_L + 1;
                                    end if;
                                 else
                                    N_A := N_A + 1;
                                    if not Mv.Element (I) then
                                       Stat_A := Stat_A + 1;
                                    end if;
                                 end if;
                              end if;
                           end loop;
                        end;
                     end if;
                  end;
               end loop;
               --  比跟着变了的比例(瓣里 / 合到的区里各自占多少):手指那块跟着眼走、几乎不变,背景在动;差不到两倍(倍数,无量纲)⇒ 看不出。
               --  比"变了的"不比"没变的":木纹对比低、后半段灰度地板高,背景也有一半像素算不上变,没变的比例都挤在 1 附近
               --  (V1B29 2026-09-27:没变的 95% 对 54%,变了的 5% 对 46%)
               declare
                  Ml : constant Long_Float := 1.0 - Long_Float (Stat_L) / Long_Float (Natural'Max (1, N_L));
                  Ma : constant Long_Float := 1.0 - Long_Float (Stat_A) / Long_Float (Natural'Max (1, N_A));
               begin
                  Known := N_L > 0 and then N_A > 0 and then (Ma > 2.0 * Ml or else Ml > 2.0 * Ma);
                  Hi_Open := Ma > Ml;
                  Put_Line ("[身]   手绕眼转 " & Codec.Fmt (Step, 3) & " 弧度再转回来:它自己那只眼里没跟着变的手指像素 在瓣里 " & Codec.Img (Stat_L) & " / " & Codec.Img (N_L)
                            & "、在合到的区里 " & Codec.Img (Stat_A) & " / " & Codec.Img (N_A)
                            & (if Known then " ⇒ 读数 " & Codec.Fmt ((if Hi_Open then Hi_R else Lo_R), 3) & " 那头张开" else " ⇒ 看不出哪头张开(差不到两倍)"));
               end;
            end;
         end if;
         if not Known then
            Put_Line ("[身]   第" & Natural'Image (H.Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道哪头张开量不出来(它自己那只眼里看不见两头的手指)");
            return;
         end if;
         H.Open_Reading := (if Hi_Open then Hi_R else Lo_R);
         H.Empty_Close := (if Hi_Open then Lo_R else Hi_R);
         H.Close_Steps := (if Hi_Open then Lo_Steps else Hi_Steps);
         if not Hi_Open then
            declare
               Rr : Long_Float;
               St : Natural := 0;
            begin
               Go_Jaw (Lo_R, Rr, St, Okg);   --  停在张开那头
            end;
         end if;
      end;
      Ok := True;
   end Measure;
end Zone;
