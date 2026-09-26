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
   Third : constant := 1.0 / 3.0;               --  三分之一(比例:"没转到命令的三分之一 = 被顶住",到没到目标用同一个比例)

   --  一组关节读数一起挪到 Q(关节目标走唯一那条挪手的路 Selfmap.Go,停稳看这组读数)
   procedure Go_Group (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; A, G : Natural; Q : Floats; Tol : Long_Float; Ok : out Boolean) is
      Dl : Table.Vec;
      Fr : Natural;
   begin
      Selfmap.Go (L, M, A, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Ok, Joints => Q, Group => G, Tol => Tol);
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
                     Go_Group (L, F, M, Natural (Arms.Length), G, Tgt, Amp * Third, Okg);
                     exit when not Okg;
                     F1 := F.Cams; J1 := F.Joints;
                     Go_Group (L, F, M, Natural (Arms.Length), G, Q0, Amp * Third, Okg);
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

   Gx : constant := 32;   --  格点 32 × 24(整幅,只铺在认出来那一下动过的像素里;采样密度,次数)
   Gy : constant := 24;

   procedure Sweep_All (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Arms : Arm_Vectors.Vector;
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector) is
      Na : constant Natural := Natural (Arms.Length);
      N_Img : Natural := 0;
      --  ── 后台配点:扫描的时候仿真每走一步要等 0.37 秒,GPU 空着 ⇒ 一个线程边扫边让配点仪器配(一对 0.85 秒)──
      type Job is record
         A, I, J : Natural := 0;
         Ia, Ib : Natural := 0;       --  两帧在仪器那边的编号
      end record;
      package Job_Vectors is new Ada.Containers.Vectors (Natural, Job);
      protected type Queue is
         procedure Put (X : Job);
         procedure Close;
         entry Get (X : out Job; Done : out Boolean);
      private
         Q : Job_Vectors.Vector;
         Head : Natural := 0;
         Closed : Boolean := False;
      end Queue;
      protected body Queue is
         procedure Put (X : Job) is
         begin
            Q.Append (X);
         end Put;
         procedure Close is
         begin
            Closed := True;
         end Close;
         entry Get (X : out Job; Done : out Boolean) when Head < Natural (Q.Length) or else Closed is
         begin
            if Head < Natural (Q.Length) then
               X := Q (Head); Head := Head + 1; Done := False;
            else
               X := (others => <>); Done := True;
            end if;
         end Get;
      end Queue;
      Jobs : Queue;
      Res : array (0 .. Natural'Max (1, Na) - 1) of Kinem.Corr_Vectors.Vector;   --  只有配点线程写;它结束以后才读
      N_Jobs, N_Empty : Natural := 0;
      Per_Pair : constant := 2500;   --  每一对让仪器抽 2500 对(次数;同 V1B2 离线过线的那一版)
      task Matcher with Storage_Size => 16 * 1024 * 1024;
      task body Matcher is
         X : Job;
         Done : Boolean;
         Err : Unbounded_String;
      begin
         loop
            Jobs.Get (X, Done);
            exit when Done;
            declare
               --  粗配:一对 0.39 秒对 0.83 秒,和完整配点只差中位 0.07–0.13 px、九成 0.2–0.6 px(trackexam 2026-09-26,V1B4 两段扫描)
               P : constant Instrument.Pair_Vectors.Vector := Instrument.Sample_Ids (Host, Port, X.Ia, X.Ib, Per_Pair, Err, Coarse => True);
            begin
               if P.Is_Empty then
                  N_Empty := N_Empty + 1;
               end if;
               for Pp of P loop
                  Res (X.A).Append (Kinem.Corr'(I => X.I, J => X.J, Ua => Pp.Ua, Va => Pp.Va, Ub => Pp.Ub, Vb => Pp.Vb));
               end loop;
            end;
         end loop;
      end Matcher;
      --  一段的头一格(给相邻关节之间配点用)
      type Head is record
         Frame, Joint : Natural := 0;
      end record;
      package Head_Vectors is new Ada.Containers.Vectors (Natural, Head);
      type Arm_State is record
         Live : Boolean := False;
         G, Cam : Natural := 0;
         Q0 : Floats;
         W, H : Natural := 0;
         Pts : Instrument.Track_Vectors.Vector;   --  跟点的格点(只拿来估每格画面挪多少、认哪些点跟着眼一起动)
         Max_Disp : Floats;                       --  每个格点整段扫描里离它起点最远挪过多少(像素)
         Tid : Integer := -1;
         Step, Off, Q_Prev : Long_Float := 0.0;
         Tgt : Floats;
         K : Natural := 0;
         Done : Boolean := True;
         Why : Unbounded_String;
         Prev_Pos : Instrument.Track_Vectors.Vector;
         Ids : Ints;                              --  每一格在仪器那边的编号
         Heads : Head_Vectors.Vector;
      end record;
      St : array (0 .. Natural'Max (1, Na) - 1) of Arm_State;
      --  这一格在仪器那边存成了没有
      function Sa_Id_Ok (A, Fr : Natural) return Boolean is (Fr < Natural (St (A).Ids.Length) and then St (A).Ids (Fr) >= 0);
      Gw : Long_Float := 0.0;
      Nj : Natural := 0;
      Okc : Boolean;
      Err : Unbounded_String;
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      S0 : constant Natural := Plug.Steps (L);
      procedure Keep (A, J : Natural; Dd : Integer; K : Natural) is
         Fo : Ada.Text_IO.File_Type;
         D : Sweep_Data := Ds (A);
         Sa : Arm_State renames St (A);
         Id : Integer;
      begin
         D.Frames.Append (Kinem.Frame_Info'(Q => F.Joints (Sa.G), Joint => (if K = 0 then -1 else Integer (J))));
         D.Imgs.Append (F.Cams (Sa.Cam));
         D.Runs.Append (if K = 0 then 0 else 1 + 2 * Integer (J) + (if Dd > 0 then 1 else 0));
         Ds.Replace_Element (A, D);
         --  存到仪器那边,起点 ↔ 这一格交给后台配
         Instrument.Frame_Put (Host, Port, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Id, Err);
         Sa.Ids.Append (Id);
         if K > 0 and then Id >= 0 and then Sa.Ids (0) >= 0 then
            Jobs.Put ((A => A, I => 0, J => Natural (D.Frames.Length) - 1, Ia => Natural (Sa.Ids (0)), Ib => Natural (Id)));
            N_Jobs := N_Jobs + 1;
         end if;
         --  每段头三格之间也配(转角小的对:每根轴单独起步时网格只用转角 ≤ 16° 的对;V1B5 只配起点 ↔ 每一格,两根轴没有够用的小转角对)
         if K >= 2 and then K <= 3 and then Id >= 0 and then Natural (Sa.Ids.Length) >= 2 and then Sa.Ids (Natural (Sa.Ids.Length) - 2) >= 0 then
            Jobs.Put ((A => A, I => Natural (D.Frames.Length) - 2, J => Natural (D.Frames.Length) - 1,
                       Ia => Natural (Sa.Ids (Natural (Sa.Ids.Length) - 2)), Ib => Natural (Id)));
            N_Jobs := N_Jobs + 1;
         end if;
         if Dump = "" then
            return;
         end if;
         declare
            Nm : constant String := "sweep_" & Codec.Img (N_Img) & ".bmp";
         begin
            Codec.Write_BMP (Dump & "/" & Nm, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H);
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
      --  一格画面挪了多少(像素):这一格和上一格都看得见的格点挪动的中位数
      function Flow (Pa, Pb : Instrument.Track_Vectors.Vector) return Long_Float is
         package Sorting is new F64_Vectors.Generic_Sorting;
         Dv : Floats;
      begin
         for P in 0 .. Natural'Min (Natural (Pa.Length), Natural (Pb.Length)) - 1 loop
            if Pa (P).Seen and then Pb (P).Seen then
               Dv.Append (Geom.Norm ([Pb (P).U - Pa (P).U, Pb (P).V - Pa (P).V, 0.0]));
            end if;
         end loop;
         if Dv.Is_Empty then
            return -1.0;
         end if;
         Sorting.Sort (Dv);
         return Dv (Natural (Dv.Length) / 2);
      end Flow;
      procedure Move_All (Tol : Long_Float) is
         Gs : Ints;
         Qs : Plug.Floats_Vectors.Vector;
         Dl : Table.Vec;
         Fr : Natural;
      begin
         for A in 0 .. Na - 1 loop
            if St (A).Live then
               Gs.Append (St (A).G); Qs.Append (St (A).Tgt);
            end if;
         end loop;
         Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okc, Groups => Gs, Qs => Qs, Tol => Tol);
      end Move_All;
   begin
      Ds.Clear; Css.Clear;
      for A in 0 .. Na - 1 loop
         Ds.Append (Sweep_Data'(others => <>));
         Css.Append (Kinem.Corr_Vectors.Empty_Vector);
         if Arms (A).Eye >= 0 and then Natural (Arms (A).Eye) < Natural (F.Cams.Length) and then Host /= "" then
            declare
               Sa : Arm_State renames St (A);
               D : Sweep_Data;
            begin
               Sa.Live := True;
               Sa.G := Arms (A).Group; Sa.Cam := Natural (Arms (A).Eye);
               Sa.Q0 := F.Joints (Sa.G); Sa.Tgt := Sa.Q0;
               Sa.W := F.Cams (Sa.Cam).W; Sa.H := F.Cams (Sa.Cam).H;
               D.W := Sa.W; D.H := Sa.H;
               Ds.Replace_Element (A, D);
               Nj := Natural'Max (Nj, Natural (Sa.Q0.Length));
               Gw := Long_Float (Sa.W) / 16.0;   --  每格画面挪画幅宽的 1/16(比例,无量纲)
               for Iy in 0 .. Gy - 1 loop
                  for Ix in 0 .. Gx - 1 loop
                     Sa.Pts.Append (Instrument.Track_Pt'(U => (Long_Float (Ix) + 0.5) * Long_Float (Sa.W) / Long_Float (Gx),
                                                         V => (Long_Float (Iy) + 0.5) * Long_Float (Sa.H) / Long_Float (Gy), Seen => True, Conf => 1.0));
                     Sa.Max_Disp.Append (0.0);
                  end loop;
               end loop;
               Say ("关节扫描 · 第" & Codec.Img (A + 1) & " 只手(第" & Codec.Img (Sa.G) & " 组读数," & Codec.Img (Natural (Sa.Q0.Length)) & " 个关节,眼 = 第"
                    & Codec.Img (Sa.Cam) & " 台)");
            end;
            Keep (A, 0, 0, 0);
         end if;
      end loop;
      for J in 0 .. Nj - 1 loop
         for Dd in -1 .. 1 loop
            if Dd /= 0 then
               for A in 0 .. Na - 1 loop
                  declare
                     Sa : Arm_State renames St (A);
                  begin
                     Sa.Done := not Sa.Live or else J >= Natural (Sa.Q0.Length);
                     if not Sa.Done then
                        Sa.Step := 0.02 * Long_Float'Max (1.0, abs Sa.Q0 (J));   --  起步 = 读数量级的 2%(比例,无量纲),按画面挪动放大
                        Sa.Off := 0.0; Sa.K := 0; Sa.Tgt := Sa.Q0; Sa.Q_Prev := F.Joints (Sa.G) (J);
                        Sa.Why := To_Unbounded_String ("走满 8 格");
                        Sa.Prev_Pos := Instrument.Track_Start (Host, Port, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Sa.Pts, Sa.Tid, Err);
                     end if;
                  end;
               end loop;
               loop
                  declare
                     Any : Boolean := False;
                     Tol : Long_Float := Long_Float'Last;
                  begin
                     for A in 0 .. Na - 1 loop
                        declare
                           Sa : Arm_State renames St (A);
                        begin
                           if not Sa.Done then
                              Any := True;
                              Sa.K := Sa.K + 1;
                              Sa.Off := Sa.Off + Sa.Step;
                              Sa.Tgt.Replace_Element (J, Sa.Q0 (J) + Long_Float (Dd) * Sa.Off);
                              Tol := Long_Float'Min (Tol, Sa.Step * Third);   --  到了 = 差不到这一格的三分之一(比例,同"被顶住")
                           end if;
                        end;
                     end loop;
                     exit when not Any;
                     Move_All (Tol);
                     exit when not Okc;
                     for A in 0 .. Na - 1 loop
                        declare
                           Sa : Arm_State renames St (A);
                        begin
                           if not Sa.Done then
                              declare
                                 Got : constant Long_Float := abs (F.Joints (Sa.G) (J) - Sa.Q_Prev);
                                 Pushed : Long_Float := 0.0;
                                 Kp : Natural := 0;
                                 Fl : Long_Float := -1.0;
                              begin
                                 for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
                                    if X /= J and then abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)) > Pushed then
                                       Pushed := abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)); Kp := X;
                                    end if;
                                 end loop;
                                 Keep (A, J, Dd, Sa.K);
                                 if Sa.K = 1 then
                                    Sa.Heads.Append (Head'(Frame => Natural (Ds (A).Frames.Length) - 1, Joint => J));
                                 end if;
                                 if Sa.Tid >= 0 then
                                    declare
                                       Pos : constant Instrument.Track_Vectors.Vector := Instrument.Track_Step (Host, Port, Sa.Tid, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Err);
                                    begin
                                       if Natural (Pos.Length) = Natural (Sa.Pts.Length) then
                                          for P in 0 .. Natural (Pos.Length) - 1 loop
                                             if Pos (P).Seen then
                                                Sa.Max_Disp.Replace_Element (P, Long_Float'Max (Sa.Max_Disp (P),
                                                  Geom.Norm ([Pos (P).U - Sa.Pts (P).U, Pos (P).V - Sa.Pts (P).V, 0.0])));
                                             end if;
                                          end loop;
                                          Fl := Flow (Sa.Prev_Pos, Pos);
                                          Sa.Prev_Pos := Pos;
                                       end if;
                                    end;
                                 end if;
                                 if 3.0 * Got < Sa.Step then   --  没转到命令的三分之一(比例):到头 / 被顶住
                                    Sa.Why := To_Unbounded_String ("关节到头或被顶住(命令 " & Codec.Fmt (Sa.Step, 4) & ",实到 " & Codec.Fmt (Got, 4) & ")");
                                    Sa.Done := True;
                                 elsif 3.0 * Pushed > Sa.Step then   --  别的关节被顶偏超过这一格的三分之一(比例,同上一条):碰上东西了,不再往里压
                                    Sa.Why := To_Unbounded_String ("碰上东西了:第" & Codec.Img (Kp) & " 个关节被顶偏 " & Codec.Fmt (Pushed, 4) & "(这一格命令 " & Codec.Fmt (Sa.Step, 4) & ")");
                                    Sa.Done := True;
                                 elsif Sa.K >= 8 then   --  最多 8 格(次数)
                                    Sa.Done := True;
                                 else
                                    if Fl > 0.0 then
                                       --  下一格按这一格的画面挪动放大 / 缩小,一次最多两倍(倍数,无量纲)
                                       Sa.Step := Sa.Step * Long_Float'Max (0.5, Long_Float'Min (Grow, Gw / Fl));
                                    end if;
                                    Sa.Q_Prev := F.Joints (Sa.G) (J);
                                 end if;
                                 if Sa.Done then
                                    Say ("  第" & Codec.Img (A + 1) & " 只手第" & Codec.Img (J) & " 个关节往" & (if Dd > 0 then "正" else "负") & "转了 " & Codec.Img (Sa.K)
                                         & " 格(累计 " & Codec.Fmt (Sa.Off, 3) & ")⇒ 停:" & To_String (Sa.Why));
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                  end;
               end loop;
               --  转回起点、收掉跟点(到没到按最后一格步子的三分之一判,同上)
               declare
                  Tol_Back : Long_Float := Long_Float'Last;
               begin
                  for A in 0 .. Na - 1 loop
                     if St (A).Live then
                        St (A).Tgt := St (A).Q0;
                        if St (A).Step > 0.0 then
                           Tol_Back := Long_Float'Min (Tol_Back, St (A).Step * Third);
                        end if;
                        if St (A).Tid >= 0 then
                           Instrument.Track_End (Host, Port, St (A).Tid);
                           St (A).Tid := -1;
                        end if;
                     end if;
                  end loop;
                  Move_All ((if Tol_Back < Long_Float'Last then Tol_Back else 0.0));
               end;
            end if;
         end loop;
      end loop;
      --  相邻关节头一格之间也配(各轴离眼远近的比例要一根接一根连起来)
      for A in 0 .. Na - 1 loop
         if St (A).Live then
            for H1 of St (A).Heads loop
               for H2 of St (A).Heads loop
                  if H2.Joint = H1.Joint + 1 and then Sa_Id_Ok (A, H1.Frame) and then Sa_Id_Ok (A, H2.Frame) then
                     Jobs.Put ((A => A, I => H1.Frame, J => H2.Frame, Ia => Natural (St (A).Ids (H1.Frame)), Ib => Natural (St (A).Ids (H2.Frame))));
                     N_Jobs := N_Jobs + 1;
                  end if;
               end loop;
            end loop;
         end if;
      end loop;
      declare
         T1 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         Say ("关节扫描走完:" & Codec.Img (Plug.Steps (L) - S0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (T1, T0)), 0) & " 秒;配点 " & Codec.Img (N_Jobs)
              & " 对在后台配,等它配完");
         Jobs.Close;
         while not Matcher'Terminated loop
            delay 0.2;   --  等后台配完(秒,协议:只是轮询间隔)
         end loop;
         Say ("  配点配完:再等了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T1)), 0) & " 秒(" & Codec.Img (N_Empty) & " 对配不上)");
      end;
      --  ③ 手指遮罩:跟点那批格点里,整段扫描离起点挪不到画幅宽 1/64 的 = 跟着眼一起动的自己(比例);落在这些格子里的配点不要
      for A in 0 .. Na - 1 loop
         if St (A).Live then
            declare
               D : Sweep_Data := Ds (A);
               W : constant Natural := D.W;
               Hh : constant Natural := D.H;
               Cw : constant Long_Float := Long_Float (W) / Long_Float (Gx);
               Ch : constant Long_Float := Long_Float (Hh) / Long_Float (Gy);
               Self : array (0 .. Gx * Gy - 1) of Boolean := [others => False];
               N_Self : Natural := 0;
               Kept : Kinem.Corr_Vectors.Vector;
               function Masked (U, V : Long_Float) return Boolean is
                  Ix : constant Integer := Integer (Long_Float'Floor (U / Cw));
                  Iy : constant Integer := Integer (Long_Float'Floor (V / Ch));
               begin
                  return Ix < 0 or else Ix >= Gx or else Iy < 0 or else Iy >= Gy or else Self (Natural (Iy) * Gx + Natural (Ix));
               end Masked;
            begin
               for P in Self'Range loop
                  if P < Natural (St (A).Max_Disp.Length) and then St (A).Max_Disp (P) < Long_Float (W) / 64.0 then   --  画幅宽 1/64(比例)
                     Self (P) := True; N_Self := N_Self + 1;
                  end if;
               end loop;
               D.Mask := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * Hh));
               for P in 0 .. W * Hh - 1 loop
                  if Masked (Long_Float (P mod W) + 0.5, Long_Float (P / W) + 0.5) then
                     D.Mask.Replace_Element (P, True);
                  end if;
               end loop;
               for C of Res (A) loop
                  if not Masked (C.Ua, C.Va) and then not Masked (C.Ub, C.Vb) then
                     Kept.Append (C);
                  end if;
               end loop;
               Say ("  第" & Codec.Img (A + 1) & " 只手:扫了 " & Codec.Img (Natural (D.Frames.Length)) & " 格;跟着眼一起动的格子(手指)" & Codec.Img (N_Self) & " / "
                    & Codec.Img (Gx * Gy) & ";配点 " & Codec.Img (Natural (Kept.Length)) & " / " & Codec.Img (Natural (Res (A).Length)) & " 个留下");
               Ds.Replace_Element (A, D);
               Css.Replace_Element (A, Kept);
            end;
         end if;
      end loop;
      Say ("关节扫描完:" & Codec.Img (Plug.Steps (L) - S0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 0) & " 秒");
   end Sweep_All;

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


   procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                      Note : out Unbounded_String) is
      Rep : Kinem.Fit_Report;
      procedure Say (S : String) is
      begin
         Append (Note, "[身] 📐 " & S & ASCII.LF);
      end Say;
   begin
      M := (others => <>); Ok := False; Note := Null_Unbounded_String;
      if Natural (D.Frames.Length) < 3 or else Cs.Is_Empty then
         Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:扫描格子太少 / 没有配点 ⇒ 量不了");
         return;
      end if;
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
