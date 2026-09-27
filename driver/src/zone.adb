with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Ada.Containers;
with Ada.Unchecked_Deallocation;
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
      N_Cams : constant Natural := Natural (F.Cams.Length);
      F0 : Plug.Cam_Vectors.Vector;
      Swept : array (0 .. Natural'Max (0, N_Cams - 1)) of Bools;
      --  🔴 扫过的那片以前是【40 步的并集】(每步和第一帧比,超过灰度地板就算动过)——
      --  渲染噪声有 40 次机会点亮每一个像素,于是并集铺满全画面。
      --  HC 实测:不动的那只眼里两只手都只"扫出"4100 个散点,外接框 616x430 ≈ 整幅画,填充率 0.015;
      --  由此硬编出来的握区被钉在画面最右边缘(u≈0.898),下游全线中毒 ——
      --  身体图垃圾 ⇒ 外推炸 ⇒ 手的位置不可信 ⇒ 拿住判据整段进不去、伺服也粗。
      --  对照:自己的腕相机里同一段代码扫出 13 万像素、填充率 0.44,是实打实的一团手指。
      --  改法零系数、零额外拍数:手指像素会【连着好几步】都和第一帧不同(手指离开了那儿),
      --  噪声只闪一步。只收【不止一步动过】的像素。
      Once : array (0 .. Natural'Max (0, N_Cams - 1)) of Bools;
      J0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm, K);
      Rest : constant Floats := Selfmap.Jaw_All (F, Arm);   --  其余通道保持它们此刻的读数
      --  合空时手要停着的位姿 = 等画面静止【之后】的读数(不是刚进来时的):手还在慢慢挪时进来,按进来那一刻的位姿发"停住"命令会把手拽回去,
      --  合爪那几拍整条胳膊跟着动,扫出来的"手指"连着胳膊贴到画面边,记下的位姿也不是画面里那一刻的(G2A 2026-09-24:人形每挪一下要 ~40 拍才停稳,
      --  头顶眼 16 笔里 7 笔因手指贴画面边被拒)
      Pose : Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Prev_J : Long_Float := J0;
      Prev_Cams : Plug.Cam_Vectors.Vector;
      Still : Natural := 0;
      Target : Floats;
      Closed_Frame : Plug.Cam_Vectors.Vector;
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
      for C in 0 .. N_Cams - 1 loop
         Once (C) := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (F.Cams (C).W * F.Cams (C).H));
         Swept (C) := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (F.Cams (C).W * F.Cams (C).H));
      end loop;
      for I in 0 .. Natural'Max (1, Natural (Rest.Length)) - 1 loop
         Target.Append (if I = K then 0.0 elsif I < Natural (Rest.Length) then Rest (I) else 1.0);
      end loop;
      --  抓握读数只是命令的回声(这台机器如此;真机也未必是关节)⇒ "合到停住"只认画面:每台相机连着两拍不变
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
         Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手第" & Natural'Image (K) & " 号抓握通道合空一次(先等画面静止:" & Natural'Image (Used) & " 拍;读数从 " & Codec.Fmt (J0, 3) & " 起)…");
      end;
      F0 := F.Cams;
      Prev_Cams := F.Cams;
      for Step in 1 .. 40 loop
         declare
            C : Plug.Cmd;
         begin
            C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
            if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
               return;
            end if;
         end;
         H.Close_Steps := Step;
         for C in 0 .. N_Cams - 1 loop
            declare
               Mv : constant Bools := Picture.Moved (F0 (C).Gray, F.Cams (C).Gray, M.Floors (C));
            begin
               --  动过不止一步的才算数(见上面那段):第二次再动到同一个像素时才收进来
               Swept (C) := Picture.Either (Swept (C), Picture.Both (Once (C), Mv));
               Once (C) := Picture.Either (Once (C), Mv);
            end;
         end loop;
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
      H.Empty_Close := Selfmap.Jaw_Of (F, Arm, K);
      Closed_Frame := F.Cams;
      Put_Line ("[身]   合到停住:读数 " & Codec.Fmt (H.Empty_Close, 3) & "(" & Natural'Image (H.Close_Steps) & " 拍)");
      --  张回去
      Target.Replace_Element (K, J0);
      Still := 0;
      Prev_Cams := F.Cams;
      for Step in 1 .. 40 loop
         declare
            C : Plug.Cmd;
         begin
            C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
            if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
               return;
            end if;
         end;
         if abs (Selfmap.Jaw_Of (F, Arm, K) - J0) <= Long_Float'Max (M.Jaw_Noise, 1.0e-3) and then Selfmap.Pictures_Still (M, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then Step >= 3;
      end loop;
      --  每台相机:扫过的像素 + 张开/合上的深度 → 握区
      for C in 0 .. N_Cams - 1 loop
         declare
            Cw : constant Natural := F.Cams (C).W;
            Ch : constant Natural := F.Cams (C).H;
            Z : Hand_Zone;
         begin
            if Codec.Env ("BL_DUMP") /= "" then
               declare
                  Mk : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Ch));
               begin
                  for I in 0 .. Cw * Ch - 1 loop
                     if Swept (C).Element (I) then
                        Mk.Replace_Element (I, 255);
                     end if;
                  end loop;
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_swept.pgm", Mk, Cw, Ch);
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_closed.pgm", Closed_Frame (C).Gray, Cw, Ch);
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_open.pgm", F0 (C).Gray, Cw, Ch);
               end;
            end if;
            --  合上前后这只眼里得真有像素动过(超过它自己量的静止噪声地板,像素数不少于一块最小连通块 —— 同 Components 的下限)才算看见手指来去。
            --  一动没动时"变化量分两拨"分的是噪声,会把一撮噪声点当成一瓣(G1S 2026-09-24:手抬出画面、缩回身前时,头顶眼各记了一笔落在空桌面上的"指尖")
            declare
               Mv : constant Bools := Picture.Moved (F0 (C).Gray, Closed_Frame (C).Gray, M.Floors (C));
               N_Mv : Natural := 0;
            begin
               for B of Mv loop
                  if B then
                     N_Mv := N_Mv + 1;
                  end if;
               end loop;
               if N_Mv < Picture.Min_Pixels (Cw, Ch) then
                  Z := (others => <>);   --  这只眼里没看见手指来去
               elsif F0 (C).Has_Depth and then Closed_Frame (C).Has_Depth then
                  Z := From_Sweep (Swept (C), F0 (C).Depth, Closed_Frame (C).Depth, True, Cw, Ch);
               else
                  Z := From_Frames (F0 (C).Gray, Closed_Frame (C).Gray, Cw, Ch);
               end if;
            end;
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
                         " · 张幅 " & Codec.Fmt (Z.Span, 3) & " 画幅 · 手指深 " & Codec.Fmt (Z.Depth, 3) & " m · 扫过 " & Codec.Fmt (Picture.Fraction (Swept (C)) * 100.0, 2) & "% 画面");
            else
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里看不见这只手合拢");
            end if;
            H.Zones.Append (Z);
         end;
      end loop;
      Ok := True;
   end Measure;
end Zone;
