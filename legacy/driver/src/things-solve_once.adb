separate (Things)
procedure Solve_Once (E : in out Estimate; Need : Long_Float) is
   Nv : constant Natural := Natural (E.Views.Length);
   Ims, Ios : Integral_Ptr;   --  每一眼:窗里它的像素、整幅里它可能在可又没量到的像素(积分图)
   type M3_Arr is array (Natural range <>) of M3;
   Rts : M3_Arr (0 .. Natural'Max (1, Nv) - 1);   --  每一眼:世界 → 这只眼
   Work, Leaves, Inner : Cell_Vectors.Vector;
   Root_H : Long_Float := 0.0;
   H_Floor : Long_Float := 0.0;
   function Height (P : V3) return Long_Float is
     (Dot ([P (0) - E.Support_P (0), P (1) - E.Support_P (1), P (2) - E.Support_P (2)], E.Support_N));
   --  这一格里有碰到过它、或者量到过它表面的点(放宽那一点自己的 Z 倍不准)
   function Hit (P : V3; Sd : Long_Float; Cl : Cell) return Boolean is
     (abs (P (0) - Cl.C (0)) <= Cl.H + Stats.Z * Sd and then abs (P (1) - Cl.C (1)) <= Cl.H + Stats.Z * Sd and then abs (P (2) - Cl.C (2)) <= Cl.H + Stats.Z * Sd);
   function Touched_In (Cl : Cell) return Boolean is
   begin
      for T of E.Touches loop
         if Hit (T.P, T.Sd, Cl) then
            return True;
         end if;
      end loop;
      for T of E.Points loop
         if Hit (T.P, T.Sd, Cl) then
            return True;
         end if;
      end loop;
      return False;
   end Touched_In;
   --  一格按全部的眼:空 / 实 / 压着轮廓 / 没眼说过话。Resolvable = 有一只说得上话的眼,它投进去比放宽那一圈的两倍(一圈两边,纯几何)大。
   --  整格在一只眼身后、它又伸不出那只眼的画幅 ⇒ 空(它整个在那只眼前面)
   type Cell_State is (C_Free, C_Inner, C_Edge, C_Unseen);
   procedure Classify (Cl : Cell; St : out Cell_State; Resolvable : out Boolean; Looks : out Natural) is
      Cs : constant Pt_Arr := Corners (Cl);
      N_In, N_Mix : Natural := 0;
      Straddle : Boolean := False;
   begin
      Resolvable := False; Looks := 0;
      declare
         Below, Above : Natural := 0;
      begin
         for P of Cs loop
            if Height (P) < 0.0 then
               Below := Below + 1;
            else
               Above := Above + 1;
            end if;
         end loop;
         if Above = 0 and then not Touched_In (Cl) then
            St := C_Free;   --  整格在它躺的面下面
            return;
         end if;
         Straddle := Below > 0;
      end;
      for K in 0 .. Nv - 1 loop
         declare
            V : View renames E.Views (K);
            U0, U1, V0, V1, Dp : Long_Float;
            Fr : Boolean;
         begin
            Footprint (V, Rts (K), Cs, U0, U1, V0, V1, Dp, Fr);
            if Fr then
               declare
                  D : constant Long_Float := Dilate (V, Dp);
                  Vd : constant Verdict := Judge (V, Ims (K), Ios (K), U0, U1, V0, V1, D);
               begin
                  case Vd is
                     when Free =>
                        if not Touched_In (Cl) then
                           St := C_Free;
                           return;
                        end if;
                        N_Mix := N_Mix + 1;
                     when Inside =>
                        N_In := N_In + 1;
                     when Mixed =>
                        N_Mix := N_Mix + 1;
                     when No_Info =>
                        null;
                  end case;
                  if Vd /= No_Info then
                     Looks := Looks + 1;
                     if Long_Float'Max (U1 - U0, V1 - V0) > 2.0 * D then
                        Resolvable := True;
                     end if;
                  end if;
               end;
            elsif not V.Beyond and then not Touched_In (Cl) then
               St := C_Free;   --  整格在这只眼身后
               return;
            end if;
         end;
      end loop;
      if Touched_In (Cl) then
         Looks := Looks + 1;   --  碰到过 / 量到过表面点:又一处说得上话的
      end if;
      if N_In + N_Mix = 0 then
         St := (if Touched_In (Cl) then C_Edge else C_Unseen);
      elsif N_Mix = 0 and then not Straddle then
         St := C_Inner;
      else
         St := C_Edge;
      end if;
   end Classify;
   --  一点按全部的眼(和它躺的面):空吗
   function Free_At (P : V3) return Boolean is
      One : constant Pt_Arr := [0 => P];
   begin
      if Height (P) < 0.0 then
         return True;
      end if;
      for K in 0 .. Nv - 1 loop
         declare
            V : View renames E.Views (K);
            U0, U1, V0, V1, Dp : Long_Float;
            Fr : Boolean;
         begin
            Footprint (V, Rts (K), One, U0, U1, V0, V1, Dp, Fr);
            if (if Fr then Judge (V, Ims (K), Ios (K), U0, U1, V0, V1, Dilate (V, Dp)) = Free else not V.Beyond) then
               return True;   --  这只眼说那儿是空的(或者那儿在一只它伸不出画幅的眼身后)
            end if;
         end;
      end loop;
      return False;
   end Free_At;
   --  起步的那一格。一眼的锥 = 它可能在的那些像素(掩膜 + Unknown)的边界像素各发一条视线,从眼出发、落到它躺的面上为止;
   --  这一圈落点 + 眼的位置的外接盒兜住整个锥(面上的落点是像素的射影变换,边界的像兜住里头的像;有一条落不到面上(朝上、和面平行)⇒
   --  这只眼的锥兜不住)。它伸不出画幅的眼(not Beyond):它整个在这只眼的锥里 ⇒ 起步格取这些眼外接盒的交;
   --  没有 ⇒ 取能兜住的那几眼外接盒的并(它被看见的那一截都在里头);碰到的点、量到的表面点并进去
   Box_Lo : V3 := [Long_Float'Last, Long_Float'Last, Long_Float'Last];
   Box_Hi : V3 := [Long_Float'First, Long_Float'First, Long_Float'First];
   Note : Unbounded_String;
   Deg_Per_Rad : constant Long_Float := 180.0 / Pi;   --  弧度换成度(给人看的话)
   Two_Looks : constant := 2;   --  沿一眼的视线伸多远,得有从别处看的第二眼夹着(两条视线才交得出一处,纯几何)
   procedure Free_All is
   begin
      for K in 0 .. Nv - 1 loop
         Free_Integral (Ims (K)); Free_Integral (Ios (K));
      end loop;
      Free_Ints (Ims); Free_Ints (Ios);
   end Free_All;
   --  这一眼掩膜形心那条视线落到它躺的面上有多远(落不到 ⇒ 0)
   function Plane_Depth (V : View) return Long_Float is
      Su, Sv : Long_Float := 0.0;
      N : Natural := 0;
   begin
      for Y in Integer'Max (0, V.Y0) .. Integer'Min (V.H - 1, V.Y1) loop
         for X in Integer'Max (0, V.X0) .. Integer'Min (V.W - 1, V.X1) loop
            if V.Mask (Natural (Y) * V.W + Natural (X)) then
               Su := Su + Long_Float (X); Sv := Sv + Long_Float (Y); N := N + 1;
            end if;
         end loop;
      end loop;
      if N = 0 then
         return 0.0;
      end if;
      declare
         Ok, Okh : Boolean;
         D : constant V3 := Ray_Fixed (V.Cam, Su / Long_Float (N), Sv / Long_Float (N), Ok);
         P : constant V3 := (if Ok then Hit_Plane (V.Cam.Pos, D, E.Support_P, E.Support_N, Okh) else V.Cam.Pos);
      begin
         return (if Ok and then Okh then Dist (P, V.Cam.Pos) else 0.0);
      end;
   end Plane_Depth;
begin
   E.Valid := False; E.Surface.Clear; E.Solid.Clear; E.Unseen.Clear; E.N_Eyes := 0; E.Thick_Unknown := False; E.Spread := 0.0; E.Inconsistent := False;
   for V of E.Views loop
      E.Solved_Seq := Natural'Max (E.Solved_Seq, V.Seq);
   end loop;
   if Nv = 0 and then E.Touches.Is_Empty and then E.Points.Is_Empty then
      E.Note := To_Unbounded_String ("no eye has outlined it yet");
      return;
   end if;
   if not E.Has_Support then
      E.Note := To_Unbounded_String ("I do not know the surface it lies on, so its outline cones are not closed below");
      return;
   end if;
   --  它躺的面朝哪一侧:看得见它的眼在哪一侧,它就在哪一侧(隔着面看不见它);眼分在两侧时按多的那一侧
   declare
      Pos_Side, Neg_Side : Natural := 0;
   begin
      for V of E.Views loop
         if Height (V.Cam.Pos) >= 0.0 then
            Pos_Side := Pos_Side + 1;
         else
            Neg_Side := Neg_Side + 1;
         end if;
      end loop;
      if Neg_Side > Pos_Side then
         E.Support_N := [-E.Support_N (0), -E.Support_N (1), -E.Support_N (2)];
      end if;
   end;
   Ims := new Integral_Arr (0 .. Natural'Max (1, Nv) - 1);
   Ios := new Integral_Arr (0 .. Natural'Max (1, Nv) - 1);
   for K in 0 .. Nv - 1 loop
      Ims (K) := Build (E.Views (K)); Ios (K) := Build (E.Views (K), Of_Unknown => True);
      Rts (K) := Tr (E.Views (K).Cam.R_Ce);
   end loop;
   declare
      Any_Whole, Any_Bounded : Boolean := False;
      Wlo : V3 := [Long_Float'First, Long_Float'First, Long_Float'First];
      Whi : V3 := [Long_Float'Last, Long_Float'Last, Long_Float'Last];
   begin
      for K in 0 .. Nv - 1 loop
         declare
            V : View renames E.Views (K);
            Lo : V3 := V.Cam.Pos;
            Hi : V3 := V.Cam.Pos;
            Bounded : Boolean := True;
         begin
            for Y in 0 .. V.H - 1 loop
               for X in 0 .. V.W - 1 loop
                  declare
                     I : constant Natural := Natural (Y) * V.W + Natural (X);
                     function In_It (J : Natural) return Boolean is (V.Mask (J) or else (Natural (V.Unknown.Length) = V.W * V.H and then V.Unknown (J)));
                  begin
                     if In_It (I) and then (X = 0 or else Y = 0 or else X = V.W - 1 or else Y = V.H - 1
                                            or else not In_It (I - 1) or else not In_It (I + 1) or else not In_It (I - V.W) or else not In_It (I + V.W))
                     then
                        declare
                           Ok, Okh : Boolean;
                           D : constant V3 := Ray_Fixed (V.Cam, Long_Float (X), Long_Float (Y), Ok);
                           P : constant V3 := (if Ok then Hit_Plane (V.Cam.Pos, D, E.Support_P, E.Support_N, Okh) else V.Cam.Pos);
                        begin
                           if Ok and then Okh then
                              for J in 0 .. 2 loop
                                 Lo (J) := Long_Float'Min (Lo (J), P (J)); Hi (J) := Long_Float'Max (Hi (J), P (J));
                              end loop;
                           else
                              Bounded := False;
                           end if;
                        end;
                     end if;
                  end;
               end loop;
            end loop;
            if Bounded then
               Any_Bounded := True;
               for J in 0 .. 2 loop
                  Box_Lo (J) := Long_Float'Min (Box_Lo (J), Lo (J)); Box_Hi (J) := Long_Float'Max (Box_Hi (J), Hi (J));
               end loop;
               if not V.Beyond then
                  Any_Whole := True;
                  for J in 0 .. 2 loop
                     Wlo (J) := Long_Float'Max (Wlo (J), Lo (J)); Whi (J) := Long_Float'Min (Whi (J), Hi (J));
                  end loop;
               end if;
            end if;
         end;
      end loop;
      if Any_Whole then
         Box_Lo := Wlo; Box_Hi := Whi;
      end if;
      for T of E.Touches loop
         for J in 0 .. 2 loop
            Box_Lo (J) := Long_Float'Min (Box_Lo (J), T.P (J) - Stats.Z * T.Sd); Box_Hi (J) := Long_Float'Max (Box_Hi (J), T.P (J) + Stats.Z * T.Sd);
         end loop;
      end loop;
      for T of E.Points loop
         for J in 0 .. 2 loop
            Box_Lo (J) := Long_Float'Min (Box_Lo (J), T.P (J) - Stats.Z * T.Sd); Box_Hi (J) := Long_Float'Max (Box_Hi (J), T.P (J) + Stats.Z * T.Sd);
         end loop;
      end loop;
      if not Any_Bounded and then E.Touches.Is_Empty and then E.Points.Is_Empty then
         Free_All;
         E.Note := To_Unbounded_String ("every outline has a sight line that never reaches the surface it lies on, so I cannot bound where it is");
         return;
      end if;
      if Box_Hi (0) < Box_Lo (0) or else Box_Hi (1) < Box_Lo (1) or else Box_Hi (2) < Box_Lo (2) then
         Free_All;
         E.Inconsistent := True;
         E.Note := To_Unbounded_String ("the outline cones of " & Codec.Img (Nv) & " looks do not meet (the looks disagree: it may have moved between them)");
         return;
      end if;
   end;
   --  所有的眼都从同一处看它(眼心两两挪的都不出放宽那一圈,远近按起步格的中心算)且没碰过、没量到表面点 ⇒ 沿视线多厚只靠面和眼夹着:
   --  锥一直通到眼上,雕出来的不是它的样子。不雕,照实说(Lo / Hi = 那几只眼的锥落在面上那一圈的外接盒)
   declare
      Cc : constant V3 := [0.5 * (Box_Lo (0) + Box_Hi (0)), 0.5 * (Box_Lo (1) + Box_Hi (1)), 0.5 * (Box_Lo (2) + Box_Hi (2))];
      Same_Place : Boolean := True;
   begin
      for A in 0 .. Nv - 1 loop
         for B in A + 1 .. Nv - 1 loop
            declare
               Lb : constant Long_Float := Dist (E.Views (B).Cam.Pos, Cc);
            begin
               if Shift_Px (E.Views (A), E.Views (B), Lb) >= Dilate (E.Views (B), Lb) then
                  Same_Place := False;
               end if;
            end;
         end loop;
      end loop;
      if Same_Place and then E.Touches.Is_Empty and then E.Points.Is_Empty then
         Free_All;
         E.Thick_Unknown := True;
         E.Lo := Box_Lo; E.Hi := Box_Hi;
         declare
            Cams : Ints;
         begin
            for V of E.Views loop
               if not Cams.Contains (V.Cam_Index) then
                  Cams.Append (V.Cam_Index);
               end if;
            end loop;
            E.N_Eyes := Natural (Cams.Length);
         end;
         E.Note := To_Unbounded_String ("every look came from the same place: it is somewhere inside that eye's outline cone, standing on the surface, "
                                        & "but how far it reaches toward the eye is unknown until an eye looks from elsewhere or I touch it");
         return;
      end if;
   end;
   Root_H := 0.5 * Long_Float'Max (Long_Float'Max (Box_Hi (0) - Box_Lo (0), Box_Hi (1) - Box_Lo (1)), Box_Hi (2) - Box_Lo (2));
   --  最细到哪儿:最准那一眼在它那儿分得出的格子的一半 —— 比这更细,哪只眼在它那儿也分不出。"它那儿"= 那只眼掩膜形心的视线落到面上的远近
   --  (它在眼和面之间,这是它离那只眼最远能有多远);落不到面上 ⇒ 到起步格中心的远近。离眼近的地方锥的尖上可以细到没有底
   --  (锥一直通到眼上,位置不准为 0 的眼放宽那一圈不随远近变大),那儿不是它 —— 不给底,八叉树在眼尖上拆不完(10-01 拔牙实测:放宽为 0 时自检卡死)。
   --  它被拿在离眼很近的地方时这个底偏粗(按面上的远近算的),表面点的间距跟着粗,照实印在 Pitch 里
   declare
      Best : Long_Float := Long_Float'Last;
      Cc : constant V3 := [0.5 * (Box_Lo (0) + Box_Hi (0)), 0.5 * (Box_Lo (1) + Box_Hi (1)), 0.5 * (Box_Lo (2) + Box_Hi (2))];
   begin
      for V of E.Views loop
         declare
            Dp : constant Long_Float := Plane_Depth (V);
            Dk : constant Long_Float := (if Dp > 0.0 then Dp else Dist (V.Cam.Pos, Cc));
         begin
            if Dk > 0.0 and then V.Cam.F > 0.0 then
               Best := Long_Float'Min (Best, Dilate (V, Dk) * Dk / V.Cam.F);
            end if;
         end;
      end loop;
      H_Floor := Long_Float'Max ((if Best < Long_Float'Last then 0.5 * Best else Long_Float'Model_Epsilon * Root_H), 0.5 * Need);
   end;
   Work.Append (Cell'(C => [0.5 * (Box_Lo (0) + Box_Hi (0)), 0.5 * (Box_Lo (1) + Box_Hi (1)), 0.5 * (Box_Lo (2) + Box_Hi (2))], H => Root_H, Looks => 0));
   --  八叉树:拆到再细哪只眼也分不出为止。只一眼说得上话的格子不拆:拆细了也只是那一眼的锥面更细,它沿那一眼的视线伸多远照样说不出
   --  (10-01 箱上 C1 重放:头顶眼的锥顺着视线一直通到头顶眼,腕眼投不进画幅、在它身后,拆到底 90 万格、每一眼交进来重解 3–13 秒)
   while not Work.Is_Empty loop
      declare
         Cl : Cell := Work.Last_Element;
         St : Cell_State;
         Res : Boolean;
         Lk : Natural;
      begin
         Work.Delete_Last;
         Classify (Cl, St, Res, Lk);
         Cl.Looks := Lk;
         case St is
            when C_Free =>
               null;
            when C_Unseen =>
               E.Unseen.Append (Cl);   --  没眼说过话:不算它,可也不能说不是它(核"动没动"时算它可能在的地方)
            when C_Inner =>
               Inner.Append (Cl);
            when C_Edge =>
               if Res and then Lk >= Two_Looks and then Cl.H > H_Floor then
                  for Sx in 0 .. 1 loop
                     for Sy in 0 .. 1 loop
                        for Sz in 0 .. 1 loop
                           Work.Append (Cell'(C => [Cl.C (0) + (if Sx = 0 then -0.5 else 0.5) * Cl.H, Cl.C (1) + (if Sy = 0 then -0.5 else 0.5) * Cl.H,
                                                    Cl.C (2) + (if Sz = 0 then -0.5 else 0.5) * Cl.H], H => 0.5 * Cl.H, Looks => 0));
                        end loop;
                     end loop;
                  end loop;
               else
                  Leaves.Append (Cl);
               end if;
         end case;
      end;
   end loop;
   --  表面:压着轮廓的最细格子里,六个邻格方向上有空的那些(外头那一层);外法向 = 朝空的那几个方向之和
   declare
      Min_H : Long_Float := Long_Float'Last;
      Lo : V3 := [Long_Float'Last, Long_Float'Last, Long_Float'Last];
      Hi : V3 := [Long_Float'First, Long_Float'First, Long_Float'First];
      Vol, Core_Vol : Long_Float := 0.0;
      Mom : V3 := [0.0, 0.0, 0.0];
      Hlo : V3 := [Long_Float'Last, Long_Float'Last, Long_Float'Last];
      Hhi : V3 := [Long_Float'First, Long_Float'First, Long_Float'First];
      N_Seen : Natural := 0;
      Cams : Ints;
      procedure Take (Cl : Cell) is
         V3c : constant Long_Float := (2.0 * Cl.H) ** 3;
      begin
         E.Solid.Append (Cl);
         Vol := Vol + V3c;
         for I in 0 .. 2 loop
            Hlo (I) := Long_Float'Min (Hlo (I), Cl.C (I) - Cl.H); Hhi (I) := Long_Float'Max (Hhi (I), Cl.C (I) + Cl.H);
         end loop;
         if Cl.Looks >= Two_Looks then
            Core_Vol := Core_Vol + V3c;
            for I in 0 .. 2 loop
               Lo (I) := Long_Float'Min (Lo (I), Cl.C (I) - Cl.H); Hi (I) := Long_Float'Max (Hi (I), Cl.C (I) + Cl.H);
               Mom (I) := Mom (I) + V3c * Cl.C (I);
            end loop;
         end if;
      end Take;
   begin
      for Cl of Inner loop
         Take (Cl);
      end loop;
      for Cl of Leaves loop
         Take (Cl);
         Min_H := Long_Float'Min (Min_H, Cl.H);
         declare
            Nrm : V3 := [0.0, 0.0, 0.0];
            Any : Boolean := False;
         begin
            for Ax in 0 .. 2 loop
               for Sg in 0 .. 1 loop
                  declare
                     S : constant Long_Float := (if Sg = 0 then -1.0 else 1.0);
                     Q : V3 := Cl.C;
                  begin
                     Q (Ax) := Q (Ax) + S * (Cl.H + Cl.H);   --  邻格的中心(一格边长 = 两个半边长)
                     if Free_At (Q) then
                        Nrm (Ax) := Nrm (Ax) + S; Any := True;
                     end if;
                  end;
               end loop;
            end loop;
            if Any and then Norm (Nrm) > 0.0 then
               declare
                  Nn : constant Long_Float := Norm (Nrm);
                  Pt : Surf_Pt;
                  Best_Sd : Long_Float := Long_Float'Last;
                  One : constant Pt_Arr := [0 => Cl.C];
               begin
                  Pt.P := Cl.C; Pt.N := [Nrm (0) / Nn, Nrm (1) / Nn, Nrm (2) / Nn];
                  for K in 0 .. Nv - 1 loop
                     declare
                        V : View renames E.Views (K);
                        U0, U1, V0, V1, Dp : Long_Float;
                        Fr : Boolean;
                     begin
                        Footprint (V, Rts (K), One, U0, U1, V0, V1, Dp, Fr);
                        if Fr and then Judge (V, Ims (K), Ios (K), U0, U1, V0, V1, Dilate (V, Dp)) /= No_Info then
                           Pt.Looks := Pt.Looks + 1;
                           Best_Sd := Long_Float'Min (Best_Sd, Sd_World (V, Dp));
                           if Dot ([V.Cam.Pos (0) - Cl.C (0), V.Cam.Pos (1) - Cl.C (1), V.Cam.Pos (2) - Cl.C (2)], Pt.N) > 0.0 then
                              Pt.Seen := True;
                           end if;
                        end if;
                     end;
                  end loop;
                  Pt.Sd := Sqrt (Cl.H ** 2 + (if Best_Sd < Long_Float'Last then Best_Sd ** 2 else 0.0));
                  Pt.Touched := Touched_In (Cl);
                  E.Surface.Append (Pt);
                  if Pt.Seen then
                     N_Seen := N_Seen + 1;
                  end if;
               end;
            end if;
         end;
      end loop;
      for T of E.Touches loop
         E.Surface.Append (Surf_Pt'(P => T.P, N => [0.0, 0.0, 0.0], Sd => T.Sd, Seen => True, Touched => True, Looks => 0));
         N_Seen := N_Seen + 1;
      end loop;
      for T of E.Points loop
         E.Surface.Append (Surf_Pt'(P => T.P, N => [0.0, 0.0, 0.0], Sd => T.Sd, Seen => True, Touched => False, Looks => 1));
         N_Seen := N_Seen + 1;
      end loop;
      Free_All;
      if E.Surface.Is_Empty or else Vol <= 0.0 then
         E.Inconsistent := Nv > 1;
         E.Note := To_Unbounded_String ("the outlines from " & Codec.Img (Nv) & " looks carve it away completely (the looks disagree: it may have moved between them)");
         return;
      end if;
      for V of E.Views loop
         if not Cams.Contains (V.Cam_Index) then
            Cams.Append (V.Cam_Index);
         end if;
      end loop;
      E.N_Eyes := Natural (Cams.Length);
      E.Pitch := (if Min_H < Long_Float'Last then 2.0 * Min_H else 0.0);
      E.Hull_Lo := Hlo; E.Hull_Hi := Hhi;
      E.One_Look_Frac := (Vol - Core_Vol) / Vol;
      if Core_Vol > 0.0 then
         E.Lo := Lo; E.Hi := Hi;
         E.Center := [Mom (0) / Core_Vol, Mom (1) / Core_Vol, Mom (2) / Core_Vol];
      else
         --  哪一格都只有一眼说得上话:没有 Core,形心 / 外接盒按整个外包给(照实在 Note 里说)
         E.Lo := Hlo; E.Hi := Hhi;
         declare
            M2 : V3 := [0.0, 0.0, 0.0];
         begin
            for Cl of E.Solid loop
               for I in 0 .. 2 loop
                  M2 (I) := M2 (I) + (2.0 * Cl.H) ** 3 * Cl.C (I);
               end loop;
            end loop;
            E.Center := [M2 (0) / Vol, M2 (1) / Vol, M2 (2) / Vol];
         end;
      end if;
      E.Seen_Frac := Long_Float (N_Seen) / Long_Float (E.Surface.Length);
      --  中心的不准:各眼位姿不准里最大的(它们对整团是同向的,不随点数变小)⊕ 半个间距
      declare
         Ps : Long_Float := 0.0;
      begin
         for V of E.Views loop
            Ps := Long_Float'Max (Ps, V.Pos_Sd);
         end loop;
         E.Center_Sd := Sqrt (Ps ** 2 + (if Min_H < Long_Float'Last then Min_H ** 2 else 0.0));
      end;
      --  看它的方向:两两夹角最大的那一对(Spread)
      for A in 0 .. Nv - 1 loop
         for B in A + 1 .. Nv - 1 loop
            declare
               Va : View renames E.Views (A);
               Vb : View renames E.Views (B);
               Da : constant V3 := [Va.Cam.Pos (0) - E.Center (0), Va.Cam.Pos (1) - E.Center (1), Va.Cam.Pos (2) - E.Center (2)];
               Db : constant V3 := [Vb.Cam.Pos (0) - E.Center (0), Vb.Cam.Pos (1) - E.Center (1), Vb.Cam.Pos (2) - E.Center (2)];
               La : constant Long_Float := Norm (Da);
               Lb : constant Long_Float := Norm (Db);
            begin
               if La > 0.0 and then Lb > 0.0 then
                  E.Spread := Long_Float'Max (E.Spread, Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Dot (Da, Db) / (La * Lb)))));
               end if;
            end;
         end loop;
      end loop;
      Append (Note, "outline-carved from " & Codec.Img (Nv) & " look" & (if Nv = 1 then "" else "s") & " by " & Codec.Img (E.N_Eyes) & " eye" & (if E.N_Eyes = 1 then "" else "s")
              & (if E.Touches.Is_Empty then "" else " and " & Codec.Img (Natural (E.Touches.Length)) & " touches")
              & (if E.Points.Is_Empty then "" else " and " & Codec.Img (Natural (E.Points.Length)) & " measured surface points")
              & ", looked at from directions up to " & Codec.Img (Natural (Long_Float'Floor (E.Spread * Deg_Per_Rad))) & " degrees apart"
              & ": " & Codec.Img (Natural (E.Surface.Length)) & " surface points, " & Codec.Img (Natural (Long_Float'Floor (100.0 * E.Seen_Frac))) & "% on sides an eye faced"
              & "; the side on the surface it lies on is unseen; this is its outer hull (hollows an outline cannot show are filled)"
              & (if E.One_Look_Frac > 0.0
                 then "; " & Codec.Img (Natural (Long_Float'Floor (100.0 * E.One_Look_Frac))) & "% of the hull is seen by only one look, so how far it reaches along that look's sight lines there is unknown"
                 else ""));
      E.Note := Note;
      E.Valid := True;
      declare
         Last_Seq : Natural := 0;
      begin
         for V of E.Views loop
            Last_Seq := Natural'Max (Last_Seq, V.Seq);
         end loop;
         E.Track.Append (Track_Pt'(Seq => Last_Seq, C => E.Center, Sd => E.Center_Sd));
      end;
   end;
end Solve_Once;
