with Ada.Text_IO;
with Ada.Containers;
with Ada.Calendar;
with Codec;
with Picture;
with Table;
with Instrument;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Jointboot is

   procedure Say (S : String) is
   begin
      Ada.Text_IO.Put_Line ("[身] 📐 " & S);
   end Say;

   Start_Amp : constant Long_Float := 1.0e-4;   --  探针协议的起点(同 Selfmap:极小,翻倍到走得出来又看得见为止;无量纲协议)
   Max_Doublings : constant := 12;              --  次数
   Grow : constant := 2.0;                      --  探针每次翻一倍(次数:同 Selfmap 的探针协议)

   --  一组关节读数一起挪到 Q(关节目标走唯一那条挪手的路 Selfmap.Go,停稳看这组读数)
   procedure Go_Group (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; A, G : Natural; Q : Floats; Ok : out Boolean) is
      Dl : Table.Vec;
      Fr : Natural;
   begin
      Selfmap.Go (L, M, A, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Ok, Joints => Q, Group => G);
   end Go_Group;

   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Natural; Ok : out Boolean) is
      Ng : constant Natural := Natural (F.Joints.Length);
      Nc : constant Natural := Natural (F.Cams.Length);
      Echo : array (0 .. Natural'Max (1, Ng) - 1) of Boolean := [others => False];
   begin
      Arms.Clear;
      World_Cam := 0;
      Ok := False;
      if Ng = 0 or else Nc = 0 then
         Say ("身体不报关节读数或没有相机 ⇒ 认不了身体");
         return;
      end if;
      for G in 0 .. Ng - 1 loop
         if not Echo (G) and then not F.Joints (G).Is_Empty then
            declare
               Q0 : constant Floats := F.Joints (G);
               Amp : Long_Float := Long_Float'Max (Start_Amp, 4.0 * M.Joint_Noise);
               Info : Arm_Info;
               Found : Boolean := False;
            begin
               Info.Group := G;
               for Try in 0 .. Max_Doublings loop
                  declare
                     Tgt : Floats := Q0;
                     F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
                     J0 : constant Plug.Floats_Vectors.Vector := F.Joints;
                     F1 : Plug.Cam_Vectors.Vector;
                     J1 : Plug.Floats_Vectors.Vector;
                     Okg : Boolean;
                     Got : Long_Float := Long_Float'Last;
                     Visible : Boolean := False;
                     Fr : Floats;
                  begin
                     for K in 0 .. Natural (Tgt.Length) - 1 loop
                        Tgt.Replace_Element (K, Q0 (K) + Amp);
                     end loop;
                     Go_Group (L, F, M, Natural (Arms.Length), G, Tgt, Okg);
                     exit when not Okg;
                     F1 := F.Cams; J1 := F.Joints;
                     Go_Group (L, F, M, Natural (Arms.Length), G, Q0, Okg);
                     exit when not Okg;
                     for K in 0 .. Natural'Min (Natural (J1 (G).Length), Natural (J0 (G).Length)) - 1 loop
                        Got := Long_Float'Min (Got, J1 (G) (K) - J0 (G) (K));   --  这组读数里跟得最少的那个关节
                     end loop;
                     for C in 0 .. Nc - 1 loop
                        declare
                           Fl : Picture.Floor_Map renames M.Floors (C);
                           M1 : constant Bools := Picture.Moved (F0 (C).Gray, F1 (C).Gray, Fl);
                           M2 : constant Bools := Picture.Moved (F1 (C).Gray, F.Cams (C).Gray, Fl);
                           Comps : constant Picture.Regions :=
                             Picture.Components (Picture.Both (M1, M2), F.Cams (C).W, F.Cams (C).H, Picture.Min_Pixels (F.Cams (C).W, F.Cams (C).H));
                        begin
                           Fr.Append (Picture.Fraction (Picture.Either (M1, M2)));
                           if not Comps.Is_Empty then
                              Visible := True;
                           end if;
                        end;
                     end loop;
                     if Got >= 0.5 * Amp and then Visible then   --  读数跟上了命令的一半(比例,同 Selfmap)、有相机看见了
                        Found := True;
                        Info.Frac := Fr;
                        Info.Probe := Amp;
                        --  别的组跟着变了同样多 ⇒ 这只手的回声组
                        for G2 in 0 .. Ng - 1 loop
                           if G2 /= G and then G2 < Natural (J1.Length) and then G2 < Natural (J0.Length)
                             and then Natural (J1 (G2).Length) = Natural (J0 (G2).Length) and then not J1 (G2).Is_Empty
                           then
                              declare
                                 Mn : Long_Float := Long_Float'Last;
                              begin
                                 for K in 0 .. Natural (J1 (G2).Length) - 1 loop
                                    Mn := Long_Float'Min (Mn, J1 (G2) (K) - J0 (G2) (K));
                                 end loop;
                                 if Mn >= 0.5 * Amp then
                                    Echo (G2) := True;
                                    Info.Echoes.Append (G2);
                                 end if;
                              end;
                           end if;
                        end loop;
                        exit;
                     end if;
                     Amp := Amp * Grow;
                  end;
               end loop;
               if Found then
                  --  哪台相机长在这只手上:整幅都变,而且比第二名多一倍(倍数,无量纲;同 Selfmap)
                  declare
                     Best : Integer := -1;
                     Bv, Second : Long_Float := 0.0;
                  begin
                     for C in 0 .. Nc - 1 loop
                        if Info.Frac (C) > Bv then
                           Second := Bv; Bv := Info.Frac (C); Best := C;
                        elsif Info.Frac (C) > Second then
                           Second := Info.Frac (C);
                        end if;
                     end loop;
                     if Best >= 0 and then Bv > 0.0 and then Bv >= 2.0 * Second then
                        Info.Eye := Best;
                     end if;
                     Say ("第" & Codec.Img (Natural (Arms.Length) + 1) & " 只手 = 第" & Codec.Img (G) & " 组关节读数(" & Codec.Img (Natural (Q0.Length))
                          & " 个,每个一起转 " & Codec.Fmt (Amp, 4) & " 就看得见)"
                          & (if Info.Eye >= 0 then ";第" & Codec.Img (Natural (Info.Eye)) & " 台相机变了 " & Codec.Fmt (100.0 * Bv, 0) & "% 的画面 ⇒ 它长在这只手上"
                             else ";哪台相机都不比第二名多一倍 ⇒ 这只手上没有眼")
                          & (if Info.Echoes.Is_Empty then "" else ";第" & Codec.Img (Natural (Info.Echoes (0))) & " 组跟着一起变 = 它的回声,不单算"));
                     Arms.Append (Info);
                  end;
               else
                  Say ("第" & Codec.Img (G) & " 组关节读数:探到 " & Codec.Fmt (Amp, 4) & " 读数还跟不上或哪台相机都没看见 ⇒ 不算一只手");
               end if;
            end;
         end if;
      end loop;
      --  世界相机 = 所有手动时变得最少的那台
      declare
         Bv : Long_Float := Long_Float'Last;
      begin
         for C in 0 .. Nc - 1 loop
            declare
               Mx : Long_Float := 0.0;
            begin
               for A of Arms loop
                  if C < Natural (A.Frac.Length) then
                     Mx := Long_Float'Max (Mx, A.Frac (C));
                  end if;
               end loop;
               if Mx < Bv then
                  Bv := Mx; World_Cam := C;
               end if;
            end;
         end loop;
      end;
      Ok := not Arms.Is_Empty;
   end Find_Arms;

   procedure Sweep_Arm (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; A : Natural; Info : Arm_Info;
                        Host : String; Port : Natural; Dump : String; N_Img : in out Natural; D : out Sweep_Data) is
      G : constant Natural := Info.Group;
      Cam : constant Natural := Natural (Info.Eye);
      Q0 : constant Floats := F.Joints (G);
      W : constant Natural := F.Cams (Cam).W;
      H : constant Natural := F.Cams (Cam).H;
      Gw : constant Long_Float := Long_Float (W) / 16.0;   --  每格画面挪画幅宽的 1/16(比例,无量纲)
      procedure Keep (J : Natural; Dd : Integer; K : Natural) is
         Fo : Ada.Text_IO.File_Type;
      begin
         D.Frames.Append (Kinem.Frame_Info'(Q => F.Joints (G), Joint => (if K = 0 then -1 else Integer (J))));
         D.Imgs.Append (F.Cams (Cam));
         D.Runs.Append (if K = 0 then 0 else 1 + 2 * Integer (J) + (if Dd > 0 then 1 else 0));
         if Dump = "" then
            return;
         end if;
         declare
            Nm : constant String := "sweep_" & Codec.Img (N_Img) & ".bmp";
         begin
            Codec.Write_BMP (Dump & "/" & Nm, F.Cams (Cam).RGB, W, H);
            N_Img := N_Img + 1;
            begin
               Ada.Text_IO.Open (Fo, Ada.Text_IO.Append_File, Dump & "/sweep.txt");
            exception
               when others => Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/sweep.txt");
            end;
            Ada.Text_IO.Put (Fo, Nm & " " & Codec.Img (A) & " " & Codec.Img (J) & " " & Codec.Img (Dd) & " " & Codec.Img (K) & " " & Codec.Img (Plug.Steps (L)));
            for Qg of F.Joints loop
               Ada.Text_IO.Put (Fo, " |");
               for X of Qg loop
                  Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
               end loop;
            end loop;
            Ada.Text_IO.Put (Fo, " ||");
            if A < Natural (F.Reported_EE.Length) then
               for I in 0 .. 6 loop
                  Ada.Text_IO.Put (Fo, " " & Codec.Fmt (F.Reported_EE (A) (I), 7));   --  身体报的手的位姿:只给离线打分,驱动不读
               end loop;
            end if;
            Ada.Text_IO.New_Line (Fo);
            Ada.Text_IO.Close (Fo);
         exception
            when others => null;
         end;
      end Keep;
      --  两张图之间画面挪了多少(像素):上一格那张图上铺 16 × 12 的格点,仪器配到这一张,取挪动的中位数
      function Flow_Px (Prev, Now : Plug.Cam) return Long_Float is
         Pts : Instrument.Match_Vectors.Vector;
         Err : Unbounded_String;
         Ds : Floats;
         Gx : constant Natural := 16;   --  格点数(采样密度,次数)
         Gy : constant Natural := 12;
         package Sorting is new F64_Vectors.Generic_Sorting;
      begin
         if Host = "" then
            return -1.0;
         end if;
         for Iy in 0 .. Gy - 1 loop
            for Ix in 0 .. Gx - 1 loop
               Pts.Append (Instrument.Match_Pt'(U => (Long_Float (Ix) + 0.5) * Long_Float (W) / Long_Float (Gx),
                                                V => (Long_Float (Iy) + 0.5) * Long_Float (H) / Long_Float (Gy), Cert => 0.0));
            end loop;
         end loop;
         declare
            R : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Prev.RGB, W, H, Now.RGB, W, H, Pts, Err);
         begin
            if Natural (R.Length) /= Natural (Pts.Length) then
               return -1.0;
            end if;
            for I in 0 .. Natural (Pts.Length) - 1 loop
               Ds.Append (Geom.Norm ([R (I).U - Pts (I).U, R (I).V - Pts (I).V, 0.0]));
            end loop;
         end;
         Sorting.Sort (Ds);
         return Ds (Natural (Ds.Length) / 2);
      end Flow_Px;
      Okc : Boolean;
   begin
      D := (others => <>);
      D.W := W; D.H := H;
      Say ("关节扫描 · 第" & Codec.Img (A + 1) & " 只手(第" & Codec.Img (G) & " 组读数," & Codec.Img (Natural (Q0.Length)) & " 个关节,眼 = 第"
           & Codec.Img (Cam) & " 台):每个关节两个方向一格一格转");
      Keep (0, 0, 0);
      for J in 0 .. Natural (Q0.Length) - 1 loop
         for Dd in -1 .. 1 loop
            if Dd /= 0 then
               declare
                  Step : Long_Float := 0.01 * Long_Float'Max (1.0, abs Q0 (J));   --  起步 = 读数量级的百分之一(比例,无量纲),按画面挪动放大
                  Off : Long_Float := 0.0;
                  Prev : Plug.Cam := F.Cams (Cam);
                  Q_Prev : Long_Float := F.Joints (G) (J);
                  K : Natural := 0;
                  Tgt : Floats := Q0;
                  Why : Unbounded_String := To_Unbounded_String ("走满 12 格");
               begin
                  while K < 12 loop   --  最多 12 格(次数)
                     K := K + 1;
                     Off := Off + Step;
                     Tgt.Replace_Element (J, Q0 (J) + Long_Float (Dd) * Off);
                     Go_Group (L, F, M, A, G, Tgt, Okc);
                     if not Okc then
                        Why := To_Unbounded_String ("插头发不出关节命令");
                        exit;
                     end if;
                     declare
                        Got : constant Long_Float := abs (F.Joints (G) (J) - Q_Prev);
                        Fl : constant Long_Float := Flow_Px (Prev, F.Cams (Cam));
                        --  别的关节离它们的目标(起点读数)最远多少:碰上东西时被扫的关节还在转、别的关节被顶偏
                        Pushed : Long_Float := 0.0;
                        Kp : Natural := 0;
                     begin
                        for X in 0 .. Natural (Q0.Length) - 1 loop
                           if X /= J and then abs (F.Joints (G) (X) - Q0 (X)) > Pushed then
                              Pushed := abs (F.Joints (G) (X) - Q0 (X)); Kp := X;
                           end if;
                        end loop;
                        Keep (J, Dd, K);
                        if 3.0 * Got < Step then   --  没转到命令的三分之一(比例):到头 / 被顶住
                           Why := To_Unbounded_String ("关节到头或被顶住(命令 " & Codec.Fmt (Step, 4) & ",实到 " & Codec.Fmt (Got, 4) & ")");
                           exit;
                        end if;
                        if 3.0 * Pushed > Step then   --  别的关节被顶偏超过这一格的三分之一(比例,同上一条):碰上东西了,不再往里压
                           Why := To_Unbounded_String ("碰上东西了:第" & Codec.Img (Kp) & " 个关节被顶偏 " & Codec.Fmt (Pushed, 4) & "(这一格命令 " & Codec.Fmt (Step, 4) & ")");
                           exit;
                        end if;
                        if Fl > 0.0 then
                           --  下一格按这一格的画面挪动放大 / 缩小,一次最多两倍(倍数,无量纲)
                           Step := Step * Long_Float'Max (0.5, Long_Float'Min (2.0, Gw / Fl));
                        end if;
                        Q_Prev := F.Joints (G) (J);
                        Prev := F.Cams (Cam);
                     end;
                  end loop;
                  Say ("  第" & Codec.Img (J) & " 个关节往" & (if Dd > 0 then "正" else "负") & "转了 " & Codec.Img (K) & " 格(累计 " & Codec.Fmt (Off, 3) & ")⇒ 停:"
                       & To_String (Why));
                  Go_Group (L, F, M, A, G, Q0, Okc);   --  转回起点
               end;
            end if;
         end loop;
      end loop;
      --  ③ 手指遮罩:整段扫描里画面一次都没变过的像素(每一格和起点比,超过这台相机的噪声地板就算变过)
      declare
         Moved_Any : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
         Fl : Picture.Floor_Map renames M.Floors (Cam);
         Cnt : Natural := 0;
      begin
         for K in 1 .. Natural (D.Imgs.Length) - 1 loop
            Moved_Any := Picture.Either (Moved_Any, Picture.Moved (D.Imgs (0).Gray, D.Imgs (K).Gray, Fl));
         end loop;
         D.Mask := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
         for P in 0 .. W * H - 1 loop
            if not Moved_Any (P) then
               D.Mask.Replace_Element (P, True);
               Cnt := Cnt + 1;
            end if;
         end loop;
         Say ("  手指遮罩:整段扫描里一次都没变过的像素占画面 " & Codec.Fmt (100.0 * Long_Float (Cnt) / Long_Float (Natural'Max (1, W * H)), 1)
              & "%(跟着眼一起动的自己 / 什么都没有的空白),配点不要它们");
      end;
   end Sweep_Arm;

   --  相机系里的单位视线(同 Geom 的约定:-z 朝前、+y 朝上)
   function Dir_Of (M : Kinem.Model; U, V : Long_Float) return Geom.V3 is
      X : constant Long_Float := (U - M.Cx) / M.F;
      Y : constant Long_Float := -(V - M.Cy) / M.F;
      N : constant Long_Float := Geom.Norm ([X, Y, -1.0]);
   begin
      return [X / N, Y / N, -1.0 / N];
   end Dir_Of;

   --  一对图:格点 Pts(A 里)在 B 里在哪,再从 B 配回 A;返回每个点的 B 像素和往返差(配不上 = 负的往返差)
   procedure Round_Trip (Host : String; Port : Natural; A_Img, B_Img : Plug.Cam; Pts : Instrument.Match_Vectors.Vector;
                         B_Mask : Bools; Ub, Vb, E : out Floats) is
      Err : Unbounded_String;
      R : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, A_Img.RGB, A_Img.W, A_Img.H, B_Img.RGB, B_Img.W, B_Img.H, Pts, Err);
      Back_Q : Instrument.Match_Vectors.Vector;
      Idx : Ints;
   begin
      Ub.Clear; Vb.Clear; E.Clear;
      for P in 0 .. Natural (Pts.Length) - 1 loop
         Ub.Append (-1.0); Vb.Append (-1.0); E.Append (-1.0);
      end loop;
      if Natural (R.Length) /= Natural (Pts.Length) then
         return;
      end if;
      for P in 0 .. Natural (Pts.Length) - 1 loop
         declare
            Iu : constant Integer := Integer (Long_Float'Floor (R (P).U));
            Iv : constant Integer := Integer (Long_Float'Floor (R (P).V));
         begin
            if Iu >= 0 and then Iu < B_Img.W and then Iv >= 0 and then Iv < B_Img.H
              and then (B_Mask.Is_Empty or else not B_Mask (Natural (Iv) * B_Img.W + Natural (Iu)))
            then
               Back_Q.Append (Instrument.Match_Pt'(U => R (P).U, V => R (P).V, Cert => 0.0));
               Idx.Append (P);
               Ub.Replace_Element (P, R (P).U); Vb.Replace_Element (P, R (P).V);
            end if;
         end;
      end loop;
      if Back_Q.Is_Empty then
         return;
      end if;
      declare
         Bk : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, B_Img.RGB, B_Img.W, B_Img.H, A_Img.RGB, A_Img.W, A_Img.H, Back_Q, Err);
      begin
         if Natural (Bk.Length) = Natural (Back_Q.Length) then
            for K in 0 .. Natural (Idx.Length) - 1 loop
               E.Replace_Element (Natural (Idx (K)), Geom.Norm ([Bk (K).U - Pts (Natural (Idx (K))).U, Bk (K).V - Pts (Natural (Idx (K))).V, 0.0]));
            end loop;
         end if;
      end;
   end Round_Trip;

   --  一组往返差 ⇒ 门(中位数的 3 倍:这一次量出来的配点噪声;比例)
   function Trip_Gate (E : Floats) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      V : Floats;
   begin
      for X of E loop
         if X >= 0.0 then
            V.Append (X);
         end if;
      end loop;
      if V.Is_Empty then
         return 0.0;
      end if;
      Sorting.Sort (V);
      return 3.0 * V (Natural (V.Length) / 2);
   end Trip_Gate;

   Gx : constant := 32;   --  配点格点 32 × 24(整幅,遮罩里的不要;采样密度,次数)
   Gy : constant := 24;
   function Grid (W, H : Natural; Mask : Bools) return Instrument.Match_Vectors.Vector is
      Pts : Instrument.Match_Vectors.Vector;
   begin
      for Iy in 0 .. Gy - 1 loop
         for Ix in 0 .. Gx - 1 loop
            declare
               U : constant Long_Float := (Long_Float (Ix) + 0.5) * Long_Float (W) / Long_Float (Gx);
               V : constant Long_Float := (Long_Float (Iy) + 0.5) * Long_Float (H) / Long_Float (Gy);
            begin
               if Mask.Is_Empty or else not Mask (Natural (Long_Float'Floor (V)) * W + Natural (Long_Float'Floor (U))) then
                  Pts.Append (Instrument.Match_Pt'(U => U, V => V, Cert => 0.0));
               end if;
            end;
         end loop;
      end loop;
      return Pts;
   end Grid;

   procedure Fit_Arm (A : Natural; D : Sweep_Data; Host : String; Port : Natural; Dump : String;
                      M : out Kinem.Model; Cs : out Kinem.Corr_Vectors.Vector; Ok : out Boolean) is
      Nf : constant Natural := Natural (D.Frames.Length);
      Pts : constant Instrument.Match_Vectors.Vector := Grid (D.W, D.H, D.Mask);
      Done : array (0 .. Natural'Max (1, Nf) - 1, 0 .. Natural'Max (1, Nf) - 1) of Boolean := [others => [others => False]];
      type Raw is record
         C : Kinem.Corr;
         E : Long_Float := -1.0;
      end record;
      package Raw_Vectors is new Ada.Containers.Vectors (Natural, Raw);
      Raws : Raw_Vectors.Vector;
      All_E : Floats;
      N_Pairs : Natural := 0;
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      procedure Pair (I0, J0 : Natural) is
         I : constant Natural := Natural'Min (I0, J0);
         J : constant Natural := Natural'Max (I0, J0);
         Ub, Vb, E : Floats;
      begin
         if I = J or else Done (I, J) then
            return;
         end if;
         Done (I, J) := True;
         N_Pairs := N_Pairs + 1;
         Round_Trip (Host, Port, D.Imgs (I), D.Imgs (J), Pts, D.Mask, Ub, Vb, E);
         for P in 0 .. Natural (Pts.Length) - 1 loop
            if E (P) >= 0.0 then
               Raws.Append (Raw'(C => (I => I, J => J, Ua => Pts (P).U, Va => Pts (P).V, Ub => Ub (P), Vb => Vb (P)), E => E (P)));
               All_E.Append (E (P));
            end if;
         end loop;
      end Pair;
      Rep : Kinem.Fit_Report;
   begin
      Cs.Clear; Ok := False; M := (others => <>);
      if Nf < 3 or else Host = "" or else Pts.Is_Empty then
         Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:扫描格子太少 / 没配配点仪器 / 遮罩外没有格点 ⇒ 量不了");
         return;
      end if;
      for K in 1 .. Nf - 1 loop
         Pair (0, K);
         if D.Runs (K) = D.Runs (K - 1) then
            Pair (K - 1, K);
         end if;
      end loop;
      for K in 0 .. Nf - 1 loop
         declare
            type Dk is record
               Dd : Long_Float := Long_Float'Last;
               J : Natural := 0;
            end record;
            Best : array (0 .. 3) of Dk;   --  关节读数上最近的 4 格(次数)
         begin
            for J in 0 .. Nf - 1 loop
               if J /= K then
                  declare
                     Dm : Long_Float := 0.0;
                  begin
                     for X in 0 .. Natural'Min (Natural (D.Frames (K).Q.Length), Natural (D.Frames (J).Q.Length)) - 1 loop
                        Dm := Long_Float'Max (Dm, abs (D.Frames (K).Q (X) - D.Frames (J).Q (X)));
                     end loop;
                     for B in Best'Range loop
                        if Dm < Best (B).Dd then
                           for Cc in reverse B + 1 .. Best'Last loop
                              Best (Cc) := Best (Cc - 1);
                           end loop;
                           Best (B) := (Dm, J);
                           exit;
                        end if;
                     end loop;
                  end;
               end if;
            end loop;
            for B of Best loop
               if B.Dd < Long_Float'Last then
                  Pair (K, B.J);
               end if;
            end loop;
         end;
      end loop;
      declare
         Gate : constant Long_Float := Trip_Gate (All_E);
      begin
         for R of Raws loop
            if R.E <= Gate then
               Cs.Append (R.C);
            end if;
         end loop;
         Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:" & Codec.Img (Nf) & " 格两两配了 " & Codec.Img (N_Pairs) & " 对,格点 " & Codec.Img (Natural (Pts.Length))
              & " 个;配回去核:往返差的门 " & Codec.Fmt (Gate, 2) & " px,留 " & Codec.Img (Natural (Cs.Length)) & " / " & Codec.Img (Natural (Raws.Length))
              & "(" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 0) & " 秒)⇒ 解");
      end;
      Kinem.Fit (D.Frames, 0, Cs, Long_Float (D.W) / 2.0, Long_Float (D.H) / 2.0, Long_Float (D.W), M, Rep, Ok);
      declare
         T : Unbounded_String;
      begin
         for J in 0 .. Natural (Rep.Joint_Med.Length) - 1 loop
            Append (T, " " & Codec.Fmt (Rep.Joint_Med (J), 3) & "(" & Codec.Img (Rep.Joint_Frames (J)) & " 格)");
         end loop;
         Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:" & (if Ok then "量成" else "没量成") & " · 每根轴单独的残差中位(像素):" & To_String (T));
         T := Null_Unbounded_String;
         for X of Rep.Rho loop
            Append (T, " " & Codec.Fmt (X, 3));
         end loop;
         Append (T, " · 各步秒数");
         for X of Rep.Secs loop
            Append (T, " " & Codec.Fmt (X, 1));
         end loop;
         Say ("    焦距 起步 " & Codec.Fmt (Rep.F_Start, 1) & " → " & Codec.Fmt (Rep.F, 1) & " · 一起解的残差中位 " & Codec.Fmt (Rep.Med_Px, 3) & " px、九成 "
              & Codec.Fmt (Rep.P90_Px, 3) & " px · 内点 " & Codec.Img (Rep.N_Used) & " / " & Codec.Img (Rep.N_Corr) & " · 各轴远近比例(以第"
              & Codec.Img (Rep.Ref_Joint) & " 根为 1):" & To_String (T) & (if Rep.Flipped then " · 平移反过一次号" else ""));
      end;
      if Dump /= "" and then Ok then
         declare
            Fo : Ada.Text_IO.File_Type;
         begin
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/kinem_arm" & Codec.Img (A) & ".txt");
            Ada.Text_IO.Put_Line (Fo, "arm " & Codec.Img (A) & " n " & Codec.Img (M.N) & " f " & Codec.Fmt (M.F, 6) & " cx " & Codec.Fmt (M.Cx, 3) & " cy " & Codec.Fmt (M.Cy, 3));
            Ada.Text_IO.Put (Fo, "q0");
            for X of M.Q0 loop
               Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 9));
            end loop;
            Ada.Text_IO.New_Line (Fo);
            for J in 0 .. M.N - 1 loop
               Ada.Text_IO.Put_Line (Fo, "axis " & Codec.Img (J) & " " & Codec.Fmt (M.Ax (J).W (0), 9) & " " & Codec.Fmt (M.Ax (J).W (1), 9) & " "
                                     & Codec.Fmt (M.Ax (J).W (2), 9) & " " & Codec.Fmt (M.Ax (J).P (0), 9) & " " & Codec.Fmt (M.Ax (J).P (1), 9) & " "
                                     & Codec.Fmt (M.Ax (J).P (2), 9));
            end loop;
            Ada.Text_IO.Close (Fo);
         exception
            when others => null;
         end;
      end if;
   end Fit_Arm;

   --  这只手起点那一格的格点三角:同一个格点在别的格子里配到的像素 + 各格的位姿(按运动学)⇒ 多条视线交一点(参照眼系)
   procedure Tri_Start (M : Kinem.Model; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Us, Vs : out Floats; X : out Kinem.V3_Array; N : out Natural) is
      Np : constant Natural := Gx * Gy;
      type Ray_List is record
         O, Dd : Kinem.V3_Array (0 .. 15);   --  每个格点最多 16 条视线(次数)
         K : Natural := 0;
         U, V : Long_Float := 0.0;
      end record;
      type Ray_Lists is array (0 .. Np - 1) of Ray_List;
      type Ray_Lists_Ptr is access Ray_Lists;
      Rl : constant Ray_Lists_Ptr := new Ray_Lists;
      Poses_R : array (0 .. Natural (D.Frames.Length) - 1) of Geom.M3;
      Poses_T : array (0 .. Natural (D.Frames.Length) - 1) of Geom.V3;
   begin
      Us.Clear; Vs.Clear; N := 0;
      for Fr in Poses_R'Range loop
         Kinem.FK (M, D.Frames (Fr).Q, Poses_R (Fr), Poses_T (Fr));
      end loop;
      for C of Cs loop
         if C.I = 0 and then C.J > 0 and then abs Kinem.Residual (M, D.Frames, C) < 3.0 then   --  这个配点在模型下对得上(3 px,协议:配点残差按像素记)
            declare
               Ix : constant Natural := Natural'Min (Gx - 1, Natural (Long_Float'Floor (C.Ua / (Long_Float (D.W) / Long_Float (Gx)))));
               Iy : constant Natural := Natural'Min (Gy - 1, Natural (Long_Float'Floor (C.Va / (Long_Float (D.H) / Long_Float (Gy)))));
               P : constant Natural := Iy * Gx + Ix;
            begin
               if Rl (P).K = 0 then
                  Rl (P).O (0) := [0.0, 0.0, 0.0]; Rl (P).Dd (0) := Dir_Of (M, C.Ua, C.Va); Rl (P).K := 1; Rl (P).U := C.Ua; Rl (P).V := C.Va;
               end if;
               if Rl (P).K < 16 then
                  Rl (P).O (Rl (P).K) := Poses_T (C.J);
                  Rl (P).Dd (Rl (P).K) := Geom.Ap (Poses_R (C.J), Dir_Of (M, C.Ub, C.Vb));
                  Rl (P).K := Rl (P).K + 1;
               end if;
            end;
         end if;
      end loop;
      for P in 0 .. Np - 1 loop
         if Rl (P).K >= 3 then   --  至少三条视线(次数)
            declare
               Xp : Geom.V3;
               Okm : Boolean;
            begin
               Kinem.Meet_Rays (Rl (P).O (0 .. Rl (P).K - 1), Rl (P).Dd (0 .. Rl (P).K - 1), Xp, Okm);
               if Okm and then Xp (2) < 0.0 then   --  在参照眼前面(-z 朝前)
                  X (X'First + N) := Xp; Us.Append (Rl (P).U); Vs.Append (Rl (P).V);
                  N := N + 1;
               end if;
            end;
         end if;
      end loop;
   end Tri_Start;

   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean) is
      use Geom;
      X0 : Kinem.V3_Array (0 .. Gx * Gy - 1);
      U0, V0 : Floats;
      N0 : Natural;
   begin
      Rw := Identity; O := [0.0, 0.0, 0.0]; Ok := False;
      if Worlds.Is_Empty or else not Worlds (0).Valid then
         return;
      end if;
      Tri_Start (Worlds (0).Model, Ds (0), Css (0), U0, V0, X0, N0);
      if N0 < 10 then   --  至少 10 个点(次数)
         Say ("世界:第一只手起点那一格只三角出 " & Codec.Img (N0) & " 个点 ⇒ 定不了桌面");
         return;
      end if;
      --  "上" = 桌面法向(朝第一只手的眼),原点 = 那只眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上
      declare
         P0, Nrm : V3;
         Inl : Natural;
         Md : Long_Float;
      begin
         Kinem.Robust_Plane (X0 (0 .. N0 - 1), P0, Nrm, Inl, Md);
         if Nrm (0) * (-P0 (0)) + Nrm (1) * (-P0 (1)) + Nrm (2) * (-P0 (2)) < 0.0 then
            Nrm := [-Nrm (0), -Nrm (1), -Nrm (2)];
         end if;
         declare
            Dn : constant Long_Float := Nrm (0);
            Xr0 : constant V3 := [1.0 - Dn * Nrm (0), -Dn * Nrm (1), -Dn * Nrm (2)];
            Xr : constant V3 := [Xr0 (0) / Norm (Xr0), Xr0 (1) / Norm (Xr0), Xr0 (2) / Norm (Xr0)];
            Yr : constant V3 := [Nrm (1) * Xr (2) - Nrm (2) * Xr (1), Nrm (2) * Xr (0) - Nrm (0) * Xr (2), Nrm (0) * Xr (1) - Nrm (1) * Xr (0)];
            Ph : constant Long_Float := P0 (0) * Nrm (0) + P0 (1) * Nrm (1) + P0 (2) * Nrm (2);
         begin
            Rw := [[Xr (0), Xr (1), Xr (2)], [Yr (0), Yr (1), Yr (2)], [Nrm (0), Nrm (1), Nrm (2)]];
            O := [Ph * Nrm (0), Ph * Nrm (1), Ph * Nrm (2)];
            Say ("世界:第一只手起点那一格三角出 " & Codec.Img (N0) & " 个点,桌面拟合了 " & Codec.Img (Inl) & " 个(离面中位 " & Codec.Fmt (Md, 4)
                 & " 单位)⇒ 上 = 桌面法向;眼离桌面 " & Codec.Fmt (abs Ph, 3) & " 单位(长度单位 = 第一只手的模型单位)");
         end;
      end;
      Ok := True;
      --  别的手:第一只手起点那一格的点在它起点那一格里配到哪、再在它自己的几格里三角 ⇒ 两团点的相似变换
      for B in 1 .. Natural (Worlds.Length) - 1 loop
         if Worlds (B).Valid then
            declare
               Db : constant Sweep_Data := Ds (B);
               Mb : constant Kinem.Model := Worlds (B).Model;
               Q_Pts : Instrument.Match_Vectors.Vector;
               Ub, Vb, E : Floats;
               Pa, Pb : Kinem.V3_Array (0 .. Gx * Gy - 1);
               Npair : Natural := 0;
            begin
               for I in 0 .. N0 - 1 loop
                  Q_Pts.Append (Instrument.Match_Pt'(U => U0 (I), V => V0 (I), Cert => 0.0));
               end loop;
               Round_Trip (Host, Port, Ds (0).Imgs (0), Db.Imgs (0), Q_Pts, Db.Mask, Ub, Vb, E);
               declare
                  Gate : constant Long_Float := Trip_Gate (E);
                  --  它自己的几格:和起点配点最多、按运动学离起点最远的 6 格(次数)
                  Cnt : array (0 .. Natural (Db.Frames.Length) - 1) of Natural := [others => 0];
                  Pick : Ints;
                  Bq : Instrument.Match_Vectors.Vector;
                  Bi : Ints;
               begin
                  for C of Css (B) loop
                     if C.I = 0 then
                        Cnt (C.J) := Cnt (C.J) + 1;
                     end if;
                  end loop;
                  for Round in 1 .. 6 loop
                     declare
                        Best : Integer := -1;
                        Bd : Long_Float := 0.0;
                     begin
                        for K in 1 .. Natural (Db.Frames.Length) - 1 loop
                           if Cnt (K) >= Natural (Q_Pts.Length) / 4 and then not Pick.Contains (K) then   --  和起点有四分之一以上的格点配得上(比例)
                              declare
                                 Rr : M3;
                                 Tt : V3;
                              begin
                                 Kinem.FK (Mb, Db.Frames (K).Q, Rr, Tt);
                                 if Norm (Tt) > Bd then
                                    Bd := Norm (Tt); Best := K;
                                 end if;
                              end;
                           end if;
                        end loop;
                        exit when Best < 0;
                        Pick.Append (Best);
                     end;
                  end loop;
                  for I in 0 .. N0 - 1 loop
                     if E (I) >= 0.0 and then E (I) <= Gate then
                        Bq.Append (Instrument.Match_Pt'(U => Ub (I), V => Vb (I), Cert => 0.0));
                        Bi.Append (I);
                     end if;
                  end loop;
                  declare
                     type Rays is record
                        Oo, Dd : Kinem.V3_Array (0 .. 7);
                        K : Natural := 0;
                     end record;
                     Rs : array (0 .. Natural'Max (1, Natural (Bq.Length)) - 1) of Rays;
                  begin
                     for I in 0 .. Natural (Bq.Length) - 1 loop
                        Rs (I).Oo (0) := [0.0, 0.0, 0.0]; Rs (I).Dd (0) := Dir_Of (Mb, Bq (I).U, Bq (I).V); Rs (I).K := 1;
                     end loop;
                     for K of Pick loop
                        declare
                           U2, V2, E2 : Floats;
                           Rr : M3;
                           Tt : V3;
                        begin
                           Kinem.FK (Mb, Db.Frames (Natural (K)).Q, Rr, Tt);
                           Round_Trip (Host, Port, Db.Imgs (0), Db.Imgs (Natural (K)), Bq, Db.Mask, U2, V2, E2);
                           declare
                              G2 : constant Long_Float := Trip_Gate (E2);
                           begin
                              for I in 0 .. Natural (Bq.Length) - 1 loop
                                 if E2 (I) >= 0.0 and then E2 (I) <= G2 and then Rs (I).K < 8 then
                                    Rs (I).Oo (Rs (I).K) := Tt; Rs (I).Dd (Rs (I).K) := Ap (Rr, Dir_Of (Mb, U2 (I), V2 (I)));
                                    Rs (I).K := Rs (I).K + 1;
                                 end if;
                              end loop;
                           end;
                        end;
                     end loop;
                     for I in 0 .. Natural (Bq.Length) - 1 loop
                        if Rs (I).K >= 3 then   --  至少三条视线(次数)
                           declare
                              Xb : V3;
                              Okm : Boolean;
                           begin
                              Kinem.Meet_Rays (Rs (I).Oo (0 .. Rs (I).K - 1), Rs (I).Dd (0 .. Rs (I).K - 1), Xb, Okm);
                              if Okm and then Xb (2) < 0.0 then
                                 Pa (Npair) := Xb; Pb (Npair) := X0 (Natural (Bi (I))); Npair := Npair + 1;
                              end if;
                           end;
                        end if;
                     end loop;
                  end;
               end;
               if Npair >= 10 then   --  至少 10 对(次数)
                  declare
                     S : Long_Float;
                     R : M3;
                     T : V3;
                     Inl : Natural;
                     Md : Long_Float;
                     Wb : Arm_World := Worlds (B);
                  begin
                     Kinem.Robust_Similarity (Pa (0 .. Npair - 1), Pb (0 .. Npair - 1), S, R, T, Inl, Md);
                     Wb.S := S; Wb.Ra := R; Wb.Ta := T;
                     Worlds.Replace_Element (B, Wb);
                     Say ("世界:第" & Codec.Img (B + 1) & " 只手对到第一只手的系:两只眼都三角出来的点 " & Codec.Img (Npair) & " 对(内点 " & Codec.Img (Inl)
                          & ",残差中位 " & Codec.Fmt (Md, 4) & " 单位)· 长度倍数 " & Codec.Fmt (S, 4));
                  end;
               else
                  declare
                     Wb : Arm_World := Worlds (B);
                  begin
                     Wb.Valid := False;
                     Worlds.Replace_Element (B, Wb);
                     Say ("世界:第" & Codec.Img (B + 1) & " 只手和第一只手看得见的桌面点只对上 " & Codec.Img (Npair) & " 对 ⇒ 对不到一个系里,这只手先不用");
                  end;
               end if;
            end;
         end if;
      end loop;
   end Align;

   --  ── 装上以后插头用的状态 ──
   St_Worlds : Arm_World_Vectors.Vector;
   St_Rw : Geom.M3 := Geom.Identity;
   St_O : Geom.V3 := [0.0, 0.0, 0.0];
   St_Last : Plug.Floats_Vectors.Vector;   --  每只手最近一帧的关节读数(反解从这儿起)

   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3) is
   begin
      St_Worlds.Clear;
      St_Last.Clear;
      for W of Worlds loop
         if W.Valid then
            St_Worlds.Append (W);
            St_Last.Append (W.Model.Q0);
         end if;
      end loop;
      St_Rw := Rw; St_O := O;
      Plug.Set_Hooks (Pose_Hook'Access, Cmd_Hook'Access);
      Say ("装上:从此每一帧手的位姿 = 按关节读数算出的腕眼位姿(" & Codec.Img (Natural (St_Worlds.Length)) & " 只手),位姿命令 = 在扫描量过的范围里解关节目标");
   end Install;

   procedure Pose_Hook (F : in out Plug.Frame) is
      use Geom;
   begin
      F.EE.Clear;
      for A in 0 .. Natural (St_Worlds.Length) - 1 loop
         declare
            W : Arm_World renames St_Worlds (A);
            Rr : M3;
            Tt : V3;
         begin
            if W.Group < Natural (F.Joints.Length) then
               St_Last.Replace_Element (A, F.Joints (W.Group));
               Kinem.FK (W.Model, F.Joints (W.Group), Rr, Tt);
               declare
                  R0 : constant M3 := Mul (W.Ra, Rr);
                  Rt : constant V3 := Ap (W.Ra, Tt);
                  T0 : constant V3 := [W.S * Rt (0) + W.Ta (0) - St_O (0), W.S * Rt (1) + W.Ta (1) - St_O (1), W.S * Rt (2) + W.Ta (2) - St_O (2)];
               begin
                  F.EE.Append (Kinem.To_Pose (Mul (St_Rw, R0), Ap (St_Rw, T0)));
               end;
            else
               F.EE.Append (Plug.Arm_Pose'[others => 0.0]);
            end if;
         end;
      end loop;
   end Pose_Hook;

   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean) is
      use Geom;
   begin
      Ok := False;
      if C.Arm >= Natural (St_Worlds.Length) then
         return;
      end if;
      declare
         W : Arm_World renames St_Worlds (C.Arm);
         Rt_W : constant M3 := Quat_To_R (C.Pose);
         Tt_W : constant V3 := [C.Pose (0), C.Pose (1), C.Pose (2)];
         --  世界 → 第一只手参照眼系 → 这只手参照眼系
         R0 : constant M3 := Mul (Tr (St_Rw), Rt_W);
         T0w : constant V3 := Ap (Tr (St_Rw), Tt_W);
         T0 : constant V3 := [T0w (0) + St_O (0), T0w (1) + St_O (1), T0w (2) + St_O (2)];
         Ra_T : constant M3 := Tr (W.Ra);
         Ra_Arm : constant M3 := Mul (Ra_T, R0);
         Ta_D : constant V3 := Ap (Ra_T, [T0 (0) - W.Ta (0), T0 (1) - W.Ta (1), T0 (2) - W.Ta (2)]);
         Ta_Arm : constant V3 := [Ta_D (0) / W.S, Ta_D (1) / W.S, Ta_D (2) / W.S];
         Q : Floats;
         Pe, Re : Long_Float;
      begin
         Kinem.IK (W.Model, Ra_Arm, Ta_Arm, St_Last (C.Arm), W.Lo, W.Hi, Q, Pe, Re);
         C.Kind := Plug.Joint;
         C.Group := W.Group;
         C.Q := Q;
         Ok := True;
      end;
   end Cmd_Hook;
end Jointboot;
