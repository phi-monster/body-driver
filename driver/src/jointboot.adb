with Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
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
   Ramp : constant := 3.0;                      --  扫描下一格最多放大三倍(次数)

   --  一组关节读数一起挪到 Q(关节目标走唯一那条挪手的路 Selfmap.Go,停稳看这组读数)
   procedure Go_Group (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; A, G : Natural; Q : Floats; Tol : Long_Float; Ok : out Boolean) is
      Dl : Table.Vec;
      Fr : Natural;
   begin
      Selfmap.Go (L, M, A, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Ok, Joints => Q, Group => G, Tol => Tol);
   end Go_Group;

   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean) is
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
      --  世界相机 = 不长在哪只手上的那台里,所有手动时变得最少的(每台相机都长在某只手上 ⇒ 没有,-1;DIY 身体可以没有不动的眼)
      declare
         Bv : Long_Float := Long_Float'Last;
      begin
         World_Cam := -1;
         for C in 0 .. Nc - 1 loop
            declare
               Mx : Long_Float := 0.0;
               On_Arm : Boolean := False;
            begin
               for A of Arms loop
                  if C < Natural (A.Frac.Length) then
                     Mx := Long_Float'Max (Mx, A.Frac (C));
                  end if;
                  On_Arm := On_Arm or else A.Eye = Integer (C);
               end loop;
               if not On_Arm and then Mx < Bv then
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
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector;
                        World_Cam : Integer := -1) is
      Na : constant Natural := Natural (Arms.Length);
      N_Img : Natural := 0;
      --  ── 配点:扫描时把要配的对攒着,扫完再让配点仪器一对一对配(粗配一对 0.39 秒)。
      --  边扫边配试过(V1B6 2026-09-26):仿真和配点仪器在同一块 GPU 上抢,一拍从 0.67 秒变 1.17 秒,比扫完再配还慢 ──
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
         entry Get (X : out Job; Done : out Boolean) when Closed is   --  扫完(Close)才开始配
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
      Multi_J : constant := 99;   --  落盘时"几个关节一起动"的格子记成第 99 个关节(协议:只是个记号)
      --  存下的每一格:哪只手、第几格、存它那一拍的帧号、落盘那一行的开头(读数和真值等量出"画面晚几拍"再按那一拍写)
      type Kept_Rec is record
         A, Frame, Seq : Natural := 0;
         Head : Unbounded_String;
      end record;
      package Kept_Vectors is new Ada.Containers.Vectors (Natural, Kept_Rec);
      Kept : Kept_Vectors.Vector;
      Seq0 : constant Natural := L.Seq;
      procedure Keep (A, J : Natural; Dd : Integer; K : Natural; Multi : Boolean := False) is
         Sa : Arm_State renames St (A);
         Id : Integer;
         Nm : constant String := "sweep_" & Codec.Img (N_Img) & ".bmp";
      begin
         --  就地加(不把整只手的格子连画面拷一遍再放回去:一格一张 640 × 480 的图,拷来拷去比扫描本身还费)
         Ds (A).Frames.Append (Kinem.Frame_Info'(Q => F.Joints (Sa.G), Joint => (if K = 0 or else Multi then -1 else Integer (J))));
         Ds (A).Imgs.Append (F.Cams (Sa.Cam));
         Ds (A).Runs.Append (if Multi then 2 * Nj + 1 else (if K = 0 then 0 else 1 + 2 * Integer (J) + (if Dd > 0 then 1 else 0)));
         Kept.Append (Kept_Rec'(A => A, Frame => Natural (Ds (A).Frames.Length) - 1, Seq => F.Seq,
                       Head => To_Unbounded_String (Nm & " " & Codec.Img (A) & " " & Codec.Img (if Multi then Multi_J else J) & " " & Codec.Img (Dd) & " "
                                                    & Codec.Img (K) & " " & Codec.Img (Plug.Steps (L)))));
         --  存到仪器那边,起点 ↔ 这一格交给后台配
         Instrument.Frame_Put (Host, Port, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Id, Err);
         Sa.Ids.Append (Id);
         if K > 0 and then Id >= 0 and then Sa.Ids (0) >= 0 then
            Jobs.Put ((A => A, I => 0, J => Natural (Ds (A).Frames.Length) - 1, Ia => Natural (Sa.Ids (0)), Ib => Natural (Id)));
            N_Jobs := N_Jobs + 1;
         end if;
         --  每段头两格之间也配(转角小的对:每根轴单独起步时网格只用转角 ≤ 16° 的对;V1B5 只配起点 ↔ 每一格,两根轴没有够用的小转角对);
         --  几个关节一起动的格子:相邻两格也配
         if (K = 2 or else (Multi and then K >= 2)) and then Id >= 0 and then Natural (Sa.Ids.Length) >= 2 and then Sa.Ids (Natural (Sa.Ids.Length) - 2) >= 0 then
            Jobs.Put ((A => A, I => Natural (Ds (A).Frames.Length) - 2, J => Natural (Ds (A).Frames.Length) - 1,
                       Ia => Natural (Sa.Ids (Natural (Sa.Ids.Length) - 2)), Ib => Natural (Id)));
            N_Jobs := N_Jobs + 1;
         end if;
         if Dump /= "" then
            Codec.Write_BMP (Dump & "/" & Nm, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H);
            N_Img := N_Img + 1;
         end if;
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
               Gw := Long_Float (Sa.W) / 10.0;   --  每格画面挪画幅宽的 1/10(比例,无量纲)
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
      --  不动的眼起点那一刻的画面(对齐几只手时当桥;见 Align)
      if World_Cam >= 0 and then Natural (World_Cam) < Natural (F.Cams.Length) and then Host /= "" then
         declare
            Wi : constant Plug.Cam := F.Cams (Natural (World_Cam));
            Id : Integer;
         begin
            Instrument.Frame_Put (Host, Port, Wi.RGB, Wi.W, Wi.H, Id, Err);
            for A in 0 .. Na - 1 loop
               if St (A).Live then
                  Ds (A).World_Img := Wi;
                  Ds (A).World_Id := Id;
               end if;
            end loop;
            if Dump /= "" then
               Codec.Write_BMP (Dump & "/world_cam.bmp", Wi.RGB, Wi.W, Wi.H);
            end if;
         end;
      end if;
      for J in 0 .. Nj - 1 loop
         for Dd in -1 .. 1 loop
            if Dd /= 0 then
               for A in 0 .. Na - 1 loop
                  declare
                     Sa : Arm_State renames St (A);
                  begin
                     Sa.Done := not Sa.Live or else J >= Natural (Sa.Q0.Length);
                     if not Sa.Done then
                        Sa.Step := 0.03 * Long_Float'Max (1.0, abs Sa.Q0 (J));   --  起步 = 读数量级的 3%(比例,无量纲),按画面挪动放大
                        Sa.Off := 0.0; Sa.K := 0; Sa.Tgt := Sa.Q0; Sa.Q_Prev := F.Joints (Sa.G) (J);
                        Sa.Why := To_Unbounded_String ("走满 5 格");
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
                                 elsif Sa.K >= 5 then   --  最多 5 格(次数;5 分钟一炮)
                                    Sa.Done := True;
                                 else
                                    if Fl > 0.0 then
                                       --  下一格按这一格的画面挪动放大 / 缩小,一次最多三倍(倍数,无量纲)
                                       Sa.Step := Sa.Step * Long_Float'Max (0.5, Long_Float'Min (Ramp, Gw / Fl));
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
      --  ② 几个关节一起动的 8 格(次数):每个关节转到它这次扫到过的那一头的一半(正负按格子号排开),每格和起点、和上一格配。
      --  只一个关节一个关节扫,各轴离眼远近的比例只靠相邻关节头一格那几对连,约束太弱:V1B6 / V1B8 驱动自己解的运动学在扫描格上
      --  中位 4–8 mm、最大 26–116 mm(像素残差却只有 0.2 px)。V1B2 离线能过线,靠的是板停 —— 几个关节一起动的姿势把各轴的比例绑在一起
      declare
         Lo, Hi : array (0 .. Natural'Max (1, Na) - 1) of Floats;
      begin
         for A in 0 .. Na - 1 loop
            if St (A).Live then
               for Jx in 0 .. Natural (St (A).Q0.Length) - 1 loop
                  Lo (A).Append (St (A).Q0 (Jx)); Hi (A).Append (St (A).Q0 (Jx));
               end loop;
               for Fr of Ds (A).Frames loop
                  for Jx in 0 .. Natural'Min (Natural (Fr.Q.Length), Natural (Lo (A).Length)) - 1 loop
                     Lo (A).Replace_Element (Jx, Long_Float'Min (Lo (A) (Jx), Fr.Q (Jx)));
                     Hi (A).Replace_Element (Jx, Long_Float'Max (Hi (A) (Jx), Fr.Q (Jx)));
                  end loop;
               end loop;
            end if;
         end loop;
         for Cb in 1 .. 8 loop
            declare
               Tol : Long_Float := Long_Float'Last;
            begin
               for A in 0 .. Na - 1 loop
                  if St (A).Live then
                     for Jx in 0 .. Natural (St (A).Q0.Length) - 1 loop
                        declare
                           --  正负按格子号和关节号排开(确定的,不随机):(格子号 × 37 + 关节号 × 11) 除以 16 的余数前一半为正(次数,只是排列)
                           Up : constant Boolean := ((Cb * 37 + Jx * 11) mod 16) < 8;
                           Tq : constant Long_Float :=
                             (if Up then St (A).Q0 (Jx) + 0.5 * (Hi (A) (Jx) - St (A).Q0 (Jx)) else St (A).Q0 (Jx) - 0.5 * (St (A).Q0 (Jx) - Lo (A) (Jx)));
                           Dq_Now : constant Long_Float := abs (Tq - St (A).Tgt (Jx));
                        begin
                           St (A).Tgt.Replace_Element (Jx, Tq);
                           if Dq_Now > 0.0 then
                              Tol := Long_Float'Min (Tol, Dq_Now * Third);
                           end if;
                        end;
                     end loop;
                  end if;
               end loop;
               Move_All ((if Tol < Long_Float'Last then Tol else 0.0));
               exit when not Okc;
               for A in 0 .. Na - 1 loop
                  if St (A).Live then
                     declare
                        Miss : Long_Float := 0.0;
                        Span : Long_Float := 0.0;
                     begin
                        for Jx in 0 .. Natural (St (A).Q0.Length) - 1 loop
                           Miss := Long_Float'Max (Miss, abs (F.Joints (St (A).G) (Jx) - St (A).Tgt (Jx)));
                           Span := Long_Float'Max (Span, abs (St (A).Tgt (Jx) - St (A).Q0 (Jx)));
                        end loop;
                        --  有关节没跟上(差超过它这次要走的三分之一 = 碰上东西 / 到头,比例同上)⇒ 这一格不要
                        if 3.0 * Miss <= Span then
                           Keep (A, 0, 0, Cb, Multi => True);   --  起点 ↔ 这一格、上一格 ↔ 这一格都在 Keep 里交给配点
                        else
                           Say ("  第" & Codec.Img (A + 1) & " 只手几个关节一起动的第" & Codec.Img (Cb) & " 格:有关节没跟上(差 " & Codec.Fmt (Miss, 4) & ")⇒ 不要这一格");
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end loop;
         for A in 0 .. Na - 1 loop
            if St (A).Live then
               St (A).Tgt := St (A).Q0;
            end if;
         end loop;
         Move_All (0.0);
      end;
      --  ④ 画面比读数晚几拍(见 Plug.Beat):这一段扫描里,每只手的眼每拍画面变了多少 和 几拍之前它那组读数变了多少 的相关,几只手加起来取最大的那个;
      --  每一格改配"画面那一刻"的读数(存格那一拍的帧号 − 晚的拍数)。V1B10 离线回放:配晚一拍的读数,扫描格上考试中位 0.71 → 0.11 mm,焦距 391.9 → 396.7(真 397)
      declare
         Sum : array (-Plug.Max_Lag .. Plug.Max_Lag) of Long_Float := [others => 0.0];
         Lag : Integer := 0;
         N_Live : Natural := 0;
         Txt : Unbounded_String;
      begin
         for A in 0 .. Na - 1 loop
            if St (A).Live then
               declare
                  Cr : Floats;
                  Lg : constant Integer := Plug.Image_Lag (L, St (A).Cam, St (A).G, Seq0, Cr);
               begin
                  N_Live := N_Live + 1;
                  for K in Sum'Range loop
                     Sum (K) := Sum (K) + Cr (K + Plug.Max_Lag);
                  end loop;
                  Append (Txt, (if N_Live > 1 then ";" else "") & "第" & Codec.Img (A + 1) & " 只手 " & Codec.Img (Lg) & " 拍");
               end;
            end if;
         end loop;
         for K in Sum'Range loop
            if Sum (K) > Sum (Lag) then
               Lag := K;
            end if;
         end loop;
         Append (Txt, " · 相关(几只手平均):");
         for K in Sum'Range loop
            Append (Txt, " " & Codec.Img (K) & " 拍 " & Codec.Fmt (Sum (K) / Long_Float (Natural'Max (1, N_Live)), 2));
         end loop;
         Say ("画面比关节读数晚 " & Codec.Img (Lag) & " 拍(" & To_String (Txt) & ")⇒ 每一格配晚这么多拍之前的读数");
         for Kp of Kept loop
            declare
               Qa : constant Plug.Floats_Vectors.Vector := Plug.Joints_At (L, Natural (Integer'Max (0, Integer (Kp.Seq) - Lag)));
            begin
               if St (Kp.A).G < Natural (Qa.Length) then
                  Ds (Kp.A).Frames (Kp.Frame).Q := Qa (St (Kp.A).G);
               end if;
            end;
         end loop;
         if Dump /= "" then
            declare
               Fo : Ada.Text_IO.File_Type;
            begin
               Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/sweep.txt");
               for Kp of Kept loop
                  declare
                     Sq : constant Natural := Natural (Integer'Max (0, Integer (Kp.Seq) - Lag));
                     Qa : constant Plug.Floats_Vectors.Vector := Plug.Joints_At (L, Sq);
                     Ea : constant Plug.Pose_Vectors.Vector := Plug.Reported_At (L, Sq);
                  begin
                     Ada.Text_IO.Put (Fo, To_String (Kp.Head));
                     for Qg of Qa loop
                        Ada.Text_IO.Put (Fo, " |");
                        for X of Qg loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                        end loop;
                     end loop;
                     Ada.Text_IO.Put (Fo, " ||");
                     if Kp.A < Natural (Ea.Length) then
                        for I in 0 .. 6 loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (Ea (Kp.A) (I), 7));   --  身体报的手的位姿(同一拍):只给离线打分,驱动不读
                        end loop;
                     end if;
                     Ada.Text_IO.New_Line (Fo);
                  end;
               end loop;
               Ada.Text_IO.Close (Fo);
            exception
               when others => null;
            end;
         end if;
      end;
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
               Kept_C : Kinem.Corr_Vectors.Vector;
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
                     Kept_C.Append (C);
                  end if;
               end loop;
               if Dump /= "" then
                  --  手指遮罩落盘(mask_arm<k>.bmp:白 = 跟着眼一起动的格子):离线回放对齐用
                  declare
                     Img : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (3 * W * Hh));
                  begin
                     for P in 0 .. W * Hh - 1 loop
                        if D.Mask (P) then
                           Img.Replace_Element (3 * P, 255); Img.Replace_Element (3 * P + 1, 255); Img.Replace_Element (3 * P + 2, 255);
                        end if;
                     end loop;
                     Codec.Write_BMP (Dump & "/mask_arm" & Codec.Img (A) & ".bmp", Img, W, Hh);
                  end;
               end if;
               if Dump /= "" then
                  --  配点落盘(corrs_arm<k>.txt:每行 I J Ua Va Ub Vb,帧号同 sweep.txt 里这只手的格子顺序):离线回放解法用
                  declare
                     Fo : Ada.Text_IO.File_Type;
                  begin
                     Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/corrs_arm" & Codec.Img (A) & ".txt");
                     for C of Kept_C loop
                        Ada.Text_IO.Put_Line (Fo, Codec.Img (C.I) & " " & Codec.Img (C.J) & " " & Codec.Fmt (C.Ua, 3) & " " & Codec.Fmt (C.Va, 3) & " "
                                              & Codec.Fmt (C.Ub, 3) & " " & Codec.Fmt (C.Vb, 3));
                     end loop;
                     Ada.Text_IO.Close (Fo);
                  exception
                     when others => null;
                  end;
               end if;
               Say ("  第" & Codec.Img (A + 1) & " 只手:扫了 " & Codec.Img (Natural (D.Frames.Length)) & " 格;跟着眼一起动的格子(手指)" & Codec.Img (N_Self) & " / "
                    & Codec.Img (Gx * Gy) & ";配点 " & Codec.Img (Natural (Kept_C.Length)) & " / " & Codec.Img (Natural (Res (A).Length)) & " 个留下");
               D.Ids := St (A).Ids;
               Ds.Replace_Element (A, D);
               Css.Replace_Element (A, Kept_C);
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
         Append (T, " · 定比例用了 " & Codec.Img (Rep.Rho_Pairs) & " 对(三对起步 " & Codec.Fmt (Rep.Rho_Start_Px, 3) & " px → 全部重解中位 " & Codec.Fmt (Rep.Rho_Px, 3) & " px)");
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

   --  这只手扫描里三角出来的点(参照眼系,模型单位):"起点 ↔ 某一格"的每个配点,两条视线(起点那只眼、那一格的眼,位姿按运动学)直接交一点。
   --  配点是仪器抽样出来的(每一对是不同的点),不能按格子把几对的配点当同一个点(V1B6 2026-09-26:那样三角出来桌面点离面中位 0.98 单位)。
   --  只收:两条视线夹角 ≥ 20 / 焦距 弧度(配点差 1 像素时远近误差不到二十分之一,比例)、在两只眼前面、模型下对得上(< 3 px);
   --  先用离起点挪得最远的格子,每一对最多 40 个(次数),一共最多 Max_Pts 个。
   --  每个点记它在起点那格、在另一格里的像素(拿去和别的眼配),和三角的协方差:配点噪声 σ = 这些配点在运动学下 Sampson 残差的中位 × 1.4826
   --  (正态下中位换标准差,统计常数);垂直视线 r·σ/f,沿视线再除以两条视线夹角的正弦
   type Tri_Pt is record
      X : Geom.V3 := [0.0, 0.0, 0.0];
      Cov : Geom.M3 := [others => [others => 0.0]];
      Fr : Natural := 0;                    --  另一格(三角它的那一对 = 起点 ↔ 这一格)
      U0, V0, Uk, Vk : Long_Float := 0.0;   --  在起点那格、在另一格里的像素
   end record;
   package Tri_Vectors is new Ada.Containers.Vectors (Natural, Tri_Pt);
   procedure Tri_Pts (M : Kinem.Model; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Max_Pts : Natural; P : out Tri_Vectors.Vector) is
      use Geom;
      Nf : constant Natural := Natural (D.Frames.Length);
      Pr : array (0 .. Natural'Max (1, Nf) - 1) of M3;
      Pt : array (0 .. Natural'Max (1, Nf) - 1) of V3;
      Used : array (0 .. Natural'Max (1, Nf) - 1) of Natural := [others => 0];
      Order : Ints;
      Min_Par : constant Long_Float := 20.0 / M.F;   --  视线夹角下限(弧度):20 像素 / 焦距(比例,见上)
      Res, Par : Floats;
      package Idx_Vectors is new Ada.Containers.Vectors (Natural, Ints, Int_Vectors."=");
      Bucket : Idx_Vectors.Vector;
   begin
      P.Clear;
      for Fr in 0 .. Nf - 1 loop
         Kinem.FK (M, D.Frames (Fr).Q, Pr (Fr), Pt (Fr));
         Bucket.Append (Int_Vectors.Empty_Vector);
      end loop;
      --  格子按离起点多远排(远的先用)
      for Fr in 1 .. Nf - 1 loop
         declare
            Pos : Natural := Natural (Order.Length);
         begin
            for K in 0 .. Natural (Order.Length) - 1 loop
               if Norm (Pt (Fr)) > Norm (Pt (Natural (Order (K)))) then
                  Pos := K;
                  exit;
               end if;
            end loop;
            Order.Insert (Pos, Fr);
         end;
      end loop;
      --  起点那一格的配点按另一格分桶(只扫一遍)
      for K in 0 .. Natural (Cs.Length) - 1 loop
         if Cs (K).I = 0 and then Cs (K).J < Nf then
            Bucket (Cs (K).J).Append (K);
         end if;
      end loop;
      for Fr of Order loop
         exit when Natural (P.Length) >= Max_Pts;
         for Ki of Bucket (Natural (Fr)) loop
            exit when Natural (P.Length) >= Max_Pts;
            declare
               C : constant Kinem.Corr := Cs (Natural (Ki));
            begin
               if Used (C.J) < 40 then
                  declare
                     R : constant Long_Float := Kinem.Residual (M, D.Frames, C);
                     D0 : constant V3 := Dir_Of (M, C.Ua, C.Va);
                     Dk : constant V3 := Ap (Pr (C.J), Dir_Of (M, C.Ub, C.Vb));
                     Cosang : constant Long_Float := D0 (0) * Dk (0) + D0 (1) * Dk (1) + D0 (2) * Dk (2);
                  begin
                     if abs R < 3.0 and then Cosang < Cos (Min_Par) then   --  3 px(协议)
                        declare
                           Xp : V3;
                           Okm : Boolean;
                        begin
                           Kinem.Meet_Rays ([[0.0, 0.0, 0.0], Pt (C.J)], [D0, Dk], Xp, Okm);
                           --  在两只眼前面(-z 朝前):参照眼系里 z < 0;那一格的眼系里 z < 0
                           if Okm and then Xp (2) < 0.0 and then Ap (Tr (Pr (C.J)), [Xp (0) - Pt (C.J) (0), Xp (1) - Pt (C.J) (1), Xp (2) - Pt (C.J) (2)]) (2) < 0.0 then
                              P.Append (Tri_Pt'(X => Xp, Cov => [others => [others => 0.0]], Fr => C.J, U0 => C.Ua, V0 => C.Va, Uk => C.Ub, Vk => C.Vb));
                              Res.Append (abs R); Par.Append (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Cosang))));
                              Used (C.J) := Used (C.J) + 1;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end;
         end loop;
      end loop;
      if P.Is_Empty then
         return;
      end if;
      declare
         package Sorting is new F64_Vectors.Generic_Sorting;
         Rs : Floats := Res;
      begin
         Sorting.Sort (Rs);
         declare
            Sig : constant Long_Float := 1.4826 * Rs (Natural (Rs.Length) / 2) / M.F;   --  配点噪声换成弧度(1.4826 = 正态下中位换标准差,统计常数,无量纲)
         begin
            for K in 0 .. Natural (P.Length) - 1 loop
               declare
                  X : constant V3 := P (K).X;
                  Rr : constant Long_Float := Norm (X);
                  Dd : constant V3 := [X (0) / Rr, X (1) / Rr, X (2) / Rr];
                  Sp : constant Long_Float := Rr * Sig;                                   --  垂直视线
                  Sd : constant Long_Float := Sp / Long_Float'Max (Sin (Par (K)), Min_Par);   --  沿视线
                  Cv : M3;
               begin
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Cv (I, J) := Sp * Sp * ((if I = J then 1.0 else 0.0) - Dd (I) * Dd (J)) + Sd * Sd * Dd (I) * Dd (J);
                     end loop;
                  end loop;
                  P (K).Cov := Cv;
               end;
            end loop;
         end;
      end;
   end Tri_Pts;

   --  ── 把几只手、几只不长在手上的眼放进同一个世界(一种办法,所有身体一样;代码里不问"有没有头顶眼")──
   --  世界 = 第一只手的参照眼系;它扫描的每一格(有三角点的)都是世界里的一只眼,位姿按它的运动学。
   --  还没放进世界的:每一只不长在手上的眼(解它的位姿 + 焦距)、每一只别的手(解它整个系的位置、朝向、长度倍数)。
   --  每一轮,每个没放进的都和世界里每一只还没配过的眼配一次(用它最像那只眼的一格,按整体特征 Instrument.Describe 挑),配点 + 往返 1 px 核对 ⇒ 共同看见的点:
   --    · 一只眼:世界点(世界里的手三角出的点)在它里面的像素 ⇒ 按板解它(Geom.Fit_Fixed_Board);
   --    · 一只手:它自己三角出、落在它自己桌面上的点,在世界里那只眼里的像素 ⇒ 那只眼的视线交世界的桌面 ⇒ 同一个点在两个系里 ⇒ 相似变换(抗野点)。
   --  这一轮内点最多的那个放进世界(它的格子 / 它这只眼随即也是世界里的眼),再下一轮;谁都放不进 ⇒ 停,放不进的如实说。
   --  全部放完以后所有放进世界的一起精修一遍(Joint_Refine):先放进去的也被后面的证据修正。
   --  (V1B11 2026-09-26 量:两只腕眼起点那格一点不重叠,可按整体特征挑的两手之间最像的 12 对往返配上 34–63%、随机 12 对平均 7%;
   --  头顶眼和第一只手最像的 8 格里 6 格配得上 29–36%)
   Trip_Px : constant := 1.0;   --  往返 1 px 内的才算(像素,协议:配点残差按像素记;绝对门 —— 按中位数倍数定的门在乱配占多数时跟着放宽,V1B11)
   Min_Inl : constant := 10;    --  至少 10 个内点才放进世界(次数)

   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo; Dump : String := "") is
      use Geom;
      Max_Pts : constant := Gx * Gy;   --  每只手最多三角几个点(同扫描格点数,次数)
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      Na : constant Natural := Natural (Worlds.Length);
      Ps : array (0 .. Natural'Max (1, Na) - 1) of Tri_Vectors.Vector;
      Pls, Nrs : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Gates : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
      Placed : array (0 .. Natural'Max (1, Na) - 1) of Boolean := [others => False];
      Pl0, N0 : V3 := [0.0, 0.0, 1.0];  --  世界的桌面(第一只手系里)
      --  世界里的眼
      type World_View is record
         Id : Integer := -1;
         Cam : Cam_Geo;                   --  世界系里的相机(R_Ce = 相机 → 世界,Pos)
         Arm : Integer := -1;             --  哪只手的哪一格(-1 = 不长在手上的眼)
         Frame : Natural := 0;
         W, H : Natural := 0;
      end record;
      package World_View_Vectors is new Ada.Containers.Vectors (Natural, World_View);
      Wv : World_View_Vectors.Vector;
      --  不长在手上的眼(这具身体有几只就几只;V1b 这一版的扫描只存了一只)
      Fx_Id : constant Integer := Ds (0).World_Id;
      Fx_Placed : Boolean := False;
      G : Cam_Geo := No_Geo;             --  它(第一只手系里)
      G_Rep : Fixed_Report;
      --  整体特征(按仪器编号)
      D_Ids : Ints;
      D_Vec : Instrument.Vec_Vectors.Vector;
      function Desc_Dot (Ia, Ib : Integer) return Long_Float is
         Ka : constant Integer := D_Ids.Find_Index (Ia);
         Kb : constant Integer := D_Ids.Find_Index (Ib);
         S : Long_Float := 0.0;
      begin
         if Ka < 0 or else Kb < 0 then
            return -1.0;
         end if;
         for K in 0 .. Natural'Min (Natural (D_Vec (Ka).Length), Natural (D_Vec (Kb).Length)) - 1 loop
            S := S + D_Vec (Ka) (K) * D_Vec (Kb) (K);
         end loop;
         return S;
      end Desc_Dot;
      --  试过的对(按仪器编号)
      type Pair is record
         A, B : Integer := -1;
      end record;
      package Pair_Vectors is new Ada.Containers.Vectors (Natural, Pair);
      Tried : Pair_Vectors.Vector;
      --  攒下来的共同看见的点
      type Cam_Ob is record
         Pa, Pk : Natural := 0;          --  哪只手的第几个三角点
         Wk : Natural := 0;              --  它是在世界里第几只眼(那只手的哪一格)里配过来的
         Uw, Vw : Long_Float := 0.0;     --  在那一格里的像素
         Xw : V3;
         Cw : M3;
         U, V, E : Long_Float := 0.0;
      end record;
      package Cam_Ob_Vectors is new Ada.Containers.Vectors (Natural, Cam_Ob);
      Cam_Obs : Cam_Ob_Vectors.Vector;
      type Arm_Ob is record
         K : Natural := 0;                       --  它的第几个三角点
         Bf : Natural := 0;                      --  它的第几格(配点那一格)
         Wk : Natural := 0;                      --  世界里的第几只眼(Wv 里的下标)
         Ro, Rd : V3;                            --  世界里那只眼过配上的那个像素的视线(起点、单位方向)
         U, V, Uw, Vw, E : Long_Float := 0.0;   --  世界里那只眼里的像素、这只手那一格里的像素、往返差
      end record;
      package Arm_Ob_Vectors is new Ada.Containers.Vectors (Natural, Arm_Ob);
      Arm_Obs : array (0 .. Natural'Max (1, Na) - 1) of Arm_Ob_Vectors.Vector;
      N_Pairs_Of : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
      N_Pairs_Fx : Natural := 0;
      Round_Note : Unbounded_String;
      --  第 A 只手第 F 格里看得见的三角点:下标和像素
      procedure Pts_In (A, F : Natural; Idx : out Ints; Q : out Instrument.Match_Vectors.Vector) is
      begin
         Idx.Clear; Q.Clear;
         for K in 0 .. Natural (Ps (A).Length) - 1 loop
            if F = 0 then
               Idx.Append (K); Q.Append (Instrument.Match_Pt'(U => Ps (A) (K).U0, V => Ps (A) (K).V0, others => <>));
            elsif Ps (A) (K).Fr = F then
               Idx.Append (K); Q.Append (Instrument.Match_Pt'(U => Ps (A) (K).Uk, V => Ps (A) (K).Vk, others => <>));
            end if;
         end loop;
      end Pts_In;
      --  第 A 只手的格子里用得上的(有三角点的):起点 + 三角用到的每一格
      function Frames_Of (A : Natural) return Ints is
         Fr : Ints;
      begin
         Fr.Append (0);
         for Q of Ps (A) loop
            if not Fr.Contains (Q.Fr) then
               Fr.Append (Q.Fr);
            end if;
         end loop;
         return Fr;
      end Frames_Of;
      function Id_Of (A, F : Natural) return Integer is (if F < Natural (Ds (A).Ids.Length) then Ds (A).Ids (F) else -1);
      --  第 A 只手第 F 格的眼放进世界(A 已放进世界:X_世界 = S · Ra · X + Ta)
      function Cam_Of (A, F : Natural) return Cam_Geo is
         Cg : Cam_Geo := No_Geo;
         Rr : M3;
         Tt : V3;
         W : constant Arm_World := Worlds (A);
         Rt : V3;
      begin
         Kinem.FK (W.Model, Ds (A).Frames (F).Q, Rr, Tt);
         Rt := Ap (W.Ra, Tt);
         Cg.R_Ce := Mul (W.Ra, Rr);
         Cg.Pos := [W.S * Rt (0) + W.Ta (0), W.S * Rt (1) + W.Ta (1), W.S * Rt (2) + W.Ta (2)];
         Cg.F := W.Model.F; Cg.Cx := W.Model.Cx; Cg.Cy := W.Model.Cy;
         Cg.Fixed := True; Cg.Valid := True;
         return Cg;
      end Cam_Of;
      function To_World (A : Natural; X : V3) return V3 is
         W : constant Arm_World := Worlds (A);
         Rt : constant V3 := Ap (W.Ra, X);
      begin
         return [W.S * Rt (0) + W.Ta (0), W.S * Rt (1) + W.Ta (1), W.S * Rt (2) + W.Ta (2)];
      end To_World;
      function Cov_World (A : Natural; C : M3) return M3 is
         W : constant Arm_World := Worlds (A);
         Rc : constant M3 := Mul (Mul (W.Ra, C), Tr (W.Ra));
         Out_C : M3;
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Out_C (I, J) := W.S * W.S * Rc (I, J);
            end loop;
         end loop;
         return Out_C;
      end Cov_World;
      --  配一对(Src 里的点 Q ⇒ Dst 那张图),往返 1 px 内的留下:返回每个留下的点在 Q 里是第几个、在 Dst 里的像素、往返差
      procedure Match_Pair (Src, Dst : Integer; Dw, Dh : Natural; Q : Instrument.Match_Vectors.Vector; Keep : out Ints; U, V, E : out Floats) is
         Err : Unbounded_String;
      begin
         Keep.Clear; U.Clear; V.Clear; E.Clear;
         Tried.Append (Pair'(A => Src, B => Dst));
         if Q.Is_Empty or else Src < 0 or else Dst < 0 then
            return;
         end if;
         declare
            R : constant Instrument.Match_Vectors.Vector := Instrument.Match_Ids (Host, Port, Natural (Src), Natural (Dst), Q, Err, Coarse => True, Back => True);
         begin
            if Natural (R.Length) = Natural (Q.Length) then
               for I in 0 .. Natural (Q.Length) - 1 loop
                  if R (I).Bu >= 0.0 and then R (I).U >= 0.0 and then R (I).U < Long_Float (Dw) and then R (I).V >= 0.0 and then R (I).V < Long_Float (Dh) then
                     declare
                        Ei : constant Long_Float := Norm ([R (I).Bu - Q (I).U, R (I).Bv - Q (I).V, 0.0]);
                     begin
                        if Ei < Trip_Px then
                           Keep.Append (I); U.Append (R (I).U); V.Append (R (I).V); E.Append (Ei);
                        end if;
                     end;
                  end if;
               end loop;
            end if;
         end;
      end Match_Pair;
      function Was_Tried (A, B : Integer) return Boolean is (Tried.Contains (Pair'(A => A, B => B)));
      function Dist (X, Pl, Nrm : V3) return Long_Float is ((X (0) - Pl (0)) * Nrm (0) + (X (1) - Pl (1)) * Nrm (1) + (X (2) - Pl (2)) * Nrm (2));
      function Cross (A, B : V3) return V3 is ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
      --  两只眼(世界系里的相机,各自的焦距、主点)之间一对配点的 Sampson 残差(像素):两条视线共不共面 —— 不经过哪只手三角出的点,不带它远近的误差
      --  (V1B11 回放 2026-09-26:拿第 2 只手自己三角出的点投进 60 cm 外第 1 只手的眼,按真值对齐也只有 54% 在 3 px 内;它俩的视线夹角最小只要 2.9°)
      function Samp2 (Ca : Cam_Geo; Ua, Va : Long_Float; Cb : Cam_Geo; Ub, Vb : Long_Float) return Long_Float is
         Rab : constant M3 := Mul (Tr (Cb.R_Ce), Ca.R_Ce);                                           --  X_b = Rab · X_a + tab(相机系)
         Tab : constant V3 := Ap (Tr (Cb.R_Ce), [Ca.Pos (0) - Cb.Pos (0), Ca.Pos (1) - Cb.Pos (1), Ca.Pos (2) - Cb.Pos (2)]);
         H1 : constant V3 := [(Ua - Ca.Cx) / Ca.F, -(Va - Ca.Cy) / Ca.F, -1.0];
         H2 : constant V3 := [(Ub - Cb.Cx) / Cb.F, -(Vb - Cb.Cy) / Cb.F, -1.0];
         Y : constant V3 := Ap (Rab, H1);
         Ex1 : constant V3 := Cross (Tab, Y);
         Etx2 : constant V3 := Ap (Tr (Rab), Cross (H2, Tab));
         Den : constant Long_Float := Sqrt ((Ex1 (0) / Cb.F) ** 2 + (Ex1 (1) / Cb.F) ** 2 + (Etx2 (0) / Ca.F) ** 2 + (Etx2 (1) / Ca.F) ** 2) + 1.0e-18;   --  数值保护(无量纲)
      begin
         return (H2 (0) * Ex1 (0) + H2 (1) * Ex1 (1) + H2 (2) * Ex1 (2)) / Den;
      end Samp2;
      --  第 B 只手第 F 格的眼,按放进世界的(S, R, T)
      function Cam_B (B, F : Natural; Sx : Long_Float; Rx : M3; Tx : V3) return Cam_Geo is
         Cg : Cam_Geo := No_Geo;
         Rr : M3;
         Tt : V3;
         Rt : V3;
      begin
         Kinem.FK (Worlds (B).Model, Ds (B).Frames (F).Q, Rr, Tt);
         Rt := Ap (Rx, Tt);
         Cg.R_Ce := Mul (Rx, Rr);
         Cg.Pos := [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)];
         Cg.F := Worlds (B).Model.F; Cg.Cx := Worlds (B).Model.Cx; Cg.Cy := Worlds (B).Model.Cy;
         Cg.Fixed := True; Cg.Valid := True;
         return Cg;
      end Cam_B;
      procedure Plane_Of (P : Tri_Vectors.Vector; Pl, Nrm : out V3; Gate : out Long_Float; Inl : out Natural; Md : out Long_Float) is
         X : Kinem.V3_Array (0 .. Natural'Max (1, Natural (P.Length)) - 1);
      begin
         for K in 0 .. Natural (P.Length) - 1 loop
            X (K) := P (K).X;
         end loop;
         Kinem.Robust_Plane (X (0 .. Natural (P.Length) - 1), Pl, Nrm, Inl, Md);
         Gate := 2.5 * 1.4826 * Md;   --  同 Robust_Plane 挑内点(统计常数,无量纲)
      end Plane_Of;
      --  把第 A 只手(已放进世界)有三角点的每一格加成世界里的眼
      procedure Add_Arm_Views (A : Natural) is
      begin
         for F of Frames_Of (A) loop
            if Id_Of (A, Natural (F)) >= 0 then
               Wv.Append (World_View'(Id => Id_Of (A, Natural (F)), Cam => Cam_Of (A, Natural (F)), Arm => A, Frame => Natural (F), W => Ds (A).W, H => Ds (A).H));
            end if;
         end loop;
      end Add_Arm_Views;
      --  按攒下的点试解:不动的眼(内点数)/ 第 B 只手(相似变换、内点数、残差中位)
      procedure Fit_Cam (Gt : out Cam_Geo; Rp : out Fixed_Report; Inl : out Natural) is
         Scene : Scene_Pt_Vectors.Vector;
         Sh : Long_Float := 0.0;
         Okf : Boolean;
      begin
         Gt := No_Geo; Rp := (others => <>); Inl := 0;
         if Natural (Cam_Obs.Length) < Min_Inl then
            return;
         end if;
         declare
            package Sorting is new F64_Vectors.Generic_Sorting;
            Es : Floats;
         begin
            for X of Cam_Obs loop
               Es.Append (X.E);
            end loop;
            Sorting.Sort (Es);
            Sh := Es (Natural (Es.Length) / 2);   --  这只眼里配点的噪声 = 这一次往返差的中位(像素)
         end;
         for X of Cam_Obs loop
            Scene.Append (Scene_Pt'(Pw => X.Xw, Cov => X.Cw, U => X.U, V => X.V, Sh => Sh, Views => 2));
         end loop;
         Gt.Cx := Long_Float (Ds (0).World_Img.W) / 2.0; Gt.Cy := Long_Float (Ds (0).World_Img.H) / 2.0; Gt.F := 0.0;   --  焦距一起解
         Fit_Fixed_Board (Gt, Scene, Rp, Okf);
         Inl := (if Okf then Rp.Scene_Used else 0);
      end Fit_Cam;
      --  每只手的桌面当一个约束用:法向(朝它的眼)、面上点的中心;倾角的不确定度 = 两张面各自"算在面上的门 ÷ 铺开的大小"按平方和,
      --  高度的不确定度 = 世界桌面算在面上的门(量出来的厚度)。只算 3 条残差(两个倾角 + 一个高度)—— 不按桌面上每个点各算一条:
      --  那样几百条压过几十条像素残差,两只手各自量的桌面之间本来就有几毫米的差,解会被拽到"桌面对得最齐"的错解上
      --  (V1B11 回放 2026-09-26:按点算时倍数 0.98、转错 12°、平移错 583 mm)
      Pl_Cb, Pl_Nb : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Pl_Sn : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];
      procedure Plane_Info (B : Natural) is
         Ext_B, Ext_0 : Long_Float := 0.0;
         C0, Cb : V3 := [0.0, 0.0, 0.0];
         N_0, N_B : Natural := 0;
      begin
         for Q of Ps (B) loop
            if abs Dist (Q.X, Pls (B), Nrs (B)) <= Gates (B) then
               Cb := [Cb (0) + Q.X (0), Cb (1) + Q.X (1), Cb (2) + Q.X (2)]; N_B := N_B + 1;
            end if;
         end loop;
         if N_B > 0 then
            Cb := [Cb (0) / Long_Float (N_B), Cb (1) / Long_Float (N_B), Cb (2) / Long_Float (N_B)];
            for Q of Ps (B) loop
               if abs Dist (Q.X, Pls (B), Nrs (B)) <= Gates (B) then
                  Ext_B := Ext_B + Norm ([Q.X (0) - Cb (0), Q.X (1) - Cb (1), Q.X (2) - Cb (2)]) ** 2;
               end if;
            end loop;
            Ext_B := Sqrt (Ext_B / Long_Float (N_B));
         end if;
         for Q of Ps (0) loop
            if abs Dist (Q.X, Pl0, N0) <= Gates (0) then
               C0 := [C0 (0) + Q.X (0), C0 (1) + Q.X (1), C0 (2) + Q.X (2)]; N_0 := N_0 + 1;
            end if;
         end loop;
         if N_0 > 0 then
            C0 := [C0 (0) / Long_Float (N_0), C0 (1) / Long_Float (N_0), C0 (2) / Long_Float (N_0)];
            for Q of Ps (0) loop
               if abs Dist (Q.X, Pl0, N0) <= Gates (0) then
                  Ext_0 := Ext_0 + Norm ([Q.X (0) - C0 (0), Q.X (1) - C0 (1), Q.X (2) - C0 (2)]) ** 2;
               end if;
            end loop;
            Ext_0 := Sqrt (Ext_0 / Long_Float (N_0));
         end if;
         Pl_Cb (B) := Cb;
         Pl_Nb (B) := (if Dist ([0.0, 0.0, 0.0], Pls (B), Nrs (B)) < 0.0 then [-Nrs (B) (0), -Nrs (B) (1), -Nrs (B) (2)] else Nrs (B));
         Pl_Sn (B) := Sqrt ((Gates (B) / Long_Float'Max (Ext_B, 1.0e-12)) ** 2 + (Gates (0) / Long_Float'Max (Ext_0, 1.0e-12)) ** 2);   --  数值保护(无量纲)
      end Plane_Info;
      procedure Plane_Res (B : Natural; Sx : Long_Float; Rx : M3; Tx : V3; R1, R2, R3 : out Long_Float) is
         Nw : constant V3 := Ap (Rx, Pl_Nb (B));
         Tilt : constant V3 := Cross (Nw, N0);
         E1, E2 : V3;
         Rt : constant V3 := Ap (Rx, Pl_Cb (B));
         Sig_P : constant Long_Float := Long_Float'Max (1.0e-12, Gates (0));   --  数值保护(无量纲)
         D0w : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2);
      begin
         E1 := Cross (N0, (if abs N0 (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  垂直于世界法向的一对方向(纯数学,无量纲)
         E1 := [E1 (0) / Norm (E1), E1 (1) / Norm (E1), E1 (2) / Norm (E1)];
         E2 := Cross (N0, E1);
         R1 := (Tilt (0) * E1 (0) + Tilt (1) * E1 (1) + Tilt (2) * E1 (2)) / Pl_Sn (B);
         R2 := (Tilt (0) * E2 (0) + Tilt (1) * E2 (1) + Tilt (2) * E2 (2)) / Pl_Sn (B);
         R3 := (N0 (0) * (Sx * Rt (0) + Tx (0)) + N0 (1) * (Sx * Rt (1) + Tx (1)) + N0 (2) * (Sx * Rt (2) + Tx (2)) - D0w) / Sig_P;
      end Plane_Res;
      --  一串残差的门:max(3 px, 3 × 中位)(协议,同运动学一起解)
      function Gate_Of (Es : Floats) return Long_Float is
         package Sorting is new F64_Vectors.Generic_Sorting;
         Sorted : Floats := Es;
      begin
         if Sorted.Is_Empty then
            return 3.0;
         end if;
         Sorting.Sort (Sorted);
         return Long_Float'Max (3.0, 3.0 * Sorted (Natural (Sorted.Length) / 2));
      end Gate_Of;

      --  第 B 只手放进世界:求 X_世界 = S · R · X + T,让 ① 它每个配上的点(它自己三角出的,桌面上的、东西上的都算)投回世界里配上它的那只眼,
      --  落在配上的那个像素上(像素残差);② 它的桌面和世界的桌面重合(上面那 3 条)。
      --  起步:两张桌面的法向对齐,剩下"绕法向转多少"× 长度倍数铺网格,每一格的平移按视线的线性最小二乘直接解,挑截断(3 倍,同挑内点的门)平方和最小的;
      --  再全部一起抗野点精修两轮(门 max(3 px, 3 × 中位))。Inl = 投回去差不到 3 px 的配点数,Md = 它们的像素残差中位
      N_Theta : constant := 360;   --  绕法向每 1° 一档(次数)
      N_Scale : constant := 41;    --  长度倍数 0.1–10 按对数分 41 档(每档约 12%;倍数范围是比例,无量纲)
      procedure Place_Arm (B : Natural; S : out Long_Float; R : out M3; T : out V3; Inl : out Natural; Md : out Long_Float) is
         Nr : constant Natural := Natural (Arm_Obs (B).Length);
         R0 : M3 := Identity;
         function Rz (Th : Long_Float) return M3 is (Rodrigues ([N0 (0) * Th, N0 (1) * Th, N0 (2) * Th]));
         --  给定倍数、转动 ⇒ 平移(视线的线性最小二乘:点到视线的距离)
         function Solve_T (Sx : Long_Float; Rx : M3) return V3 is
            A : M3 := [others => [others => 0.0]];
            Bb : V3 := [0.0, 0.0, 0.0];
         begin
            for Ob of Arm_Obs (B) loop
               declare
                  D : constant V3 := Ob.Rd;
                  Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
                  Q : constant V3 := [Ob.Ro (0) - Sx * Rt (0), Ob.Ro (1) - Sx * Rt (1), Ob.Ro (2) - Sx * Rt (2)];
                  Dq : constant Long_Float := D (0) * Q (0) + D (1) * Q (1) + D (2) * Q (2);
               begin
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        A (I, J) := A (I, J) + (if I = J then 1.0 else 0.0) - D (I) * D (J);
                     end loop;
                     Bb (I) := Bb (I) + Q (I) - D (I) * Dq;
                  end loop;
               end;
            end loop;
            return Solve3 (A, Bb);
         end Solve_T;
         function Epi (Ob : Arm_Ob; Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
           (Samp2 (Cam_B (B, Ob.Bf, Sx, Rx, Tx), Ob.Uw, Ob.Vw, Wv (Ob.Wk).Cam, Ob.U, Ob.V));
         function Score (Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
            Sm : Long_Float := 0.0;
            P1, P2, P3 : Long_Float;
         begin
            for Ob of Arm_Obs (B) loop
               Sm := Sm + Long_Float'Min (Epi (Ob, Sx, Rx, Tx) ** 2, 9.0);   --  截断在 3 px(协议,同挑内点的门)
            end loop;
            Plane_Res (B, Sx, Rx, Tx, P1, P2, P3);
            return Sm + Long_Float'Min (P1 * P1, 9.0) + Long_Float'Min (P2 * P2, 9.0) + Long_Float'Min (P3 * P3, 9.0);   --  截断在 3 倍不确定度(同上)
         end Score;
      begin
         S := 1.0; R := Identity; T := [0.0, 0.0, 0.0]; Inl := 0; Md := 0.0;
         if Nr < Min_Inl then
            return;
         end if;
         --  它的桌面法向(朝它的眼)转到世界的法向上
         declare
            Nb : constant V3 := Pl_Nb (B);
            Ax : constant V3 := Cross (Nb, N0);
            Sn : constant Long_Float := Norm (Ax);
            Cs : constant Long_Float := Nb (0) * N0 (0) + Nb (1) * N0 (1) + Nb (2) * N0 (2);
         begin
            if Sn > 1.0e-12 then   --  数值保护(无量纲)
               R0 := Rodrigues ([Ax (0) / Sn * Arctan (Sn, Cs), Ax (1) / Sn * Arctan (Sn, Cs), Ax (2) / Sn * Arctan (Sn, Cs)]);
            elsif Cs < 0.0 then
               declare
                  E1 : constant V3 := Cross (Nb, (if abs Nb (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  辅助方向(纯数学,无量纲)
                  En : constant Long_Float := Norm (E1);
               begin
                  R0 := Rodrigues ([E1 (0) / En * Ada.Numerics.Pi, E1 (1) / En * Ada.Numerics.Pi, E1 (2) / En * Ada.Numerics.Pi]);   --  反向:转半圈
               end;
            end if;
         end;
         --  网格起步
         declare
            Best : Long_Float := Long_Float'Last;
         begin
            for It in 0 .. N_Theta - 1 loop
               declare
                  Rx : constant M3 := Mul (Rz (2.0 * Ada.Numerics.Pi * Long_Float (It) / Long_Float (N_Theta)), R0);
               begin
                  for Ks in 0 .. N_Scale - 1 loop
                     declare
                        Sx : constant Long_Float := 0.1 * Exp (Long_Float (Ks) / Long_Float (N_Scale - 1) * Log (100.0));   --  0.1–10(比例,无量纲)
                        Tx : constant V3 := Solve_T (Sx, Rx);
                        Sc : constant Long_Float := Score (Sx, Rx, Tx);
                     begin
                        if Sc < Best then
                           Best := Sc; S := Sx; R := Rx; T := Tx;
                        end if;
                     end;
                  end loop;
               end;
            end loop;
         end;
         --  一起精修两轮
         for Round in 1 .. 2 loop
            declare
               Es : Floats;
               Gate : Long_Float;
               Use_R : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Nr));
               Nu : Natural := 0;
            begin
               for Ob of Arm_Obs (B) loop
                  Es.Append (abs Epi (Ob, S, R, T));
               end loop;
               Gate := Gate_Of (Es);
               for K in 0 .. Nr - 1 loop
                  if Es (K) < Gate then
                     Use_R.Replace_Element (K, True); Nu := Nu + 1;
                  end if;
               end loop;
               exit when Nu < Min_Inl;
               declare
                  N_Res : constant Natural := Nu + 3;
                  Rv : constant V3 := Rot_Vec (R);
                  X : Kinem.Vec (0 .. 6) := [Rv (0), Rv (1), Rv (2), T (0), T (1), T (2), Log (S)];
                  Steps : constant Kinem.Vec (0 .. 6) := [others => 1.0e-7];   --  差分步(弧度 / 模型单位 / 对数倍数,极小量,无量纲)
                  procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                     Rx : constant M3 := Rodrigues ([Xx (Xx'First), Xx (Xx'First + 1), Xx (Xx'First + 2)]);
                     Tx : constant V3 := [Xx (Xx'First + 3), Xx (Xx'First + 4), Xx (Xx'First + 5)];
                     Sx : constant Long_Float := Exp (Xx (Xx'First + 6));
                     J : Natural := Rr'First;
                  begin
                     for K in 0 .. Nr - 1 loop
                        if Use_R (K) then
                           Rr (J) := Epi (Arm_Obs (B) (K), Sx, Rx, Tx); J := J + 1;
                        end if;
                     end loop;
                     Plane_Res (B, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2));
                  end Resid;
               begin
                  Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access);
                  R := Rodrigues ([X (0), X (1), X (2)]); T := [X (3), X (4), X (5)]; S := Exp (X (6));
               end;
            end;
         end loop;
         declare
            Es : Floats;
            package Sorting is new F64_Vectors.Generic_Sorting;
         begin
            for Ob of Arm_Obs (B) loop
               if abs Epi (Ob, S, R, T) < 3.0 then   --  3 px(协议)
                  Es.Append (abs Epi (Ob, S, R, T));
               end if;
            end loop;
            Inl := Natural (Es.Length);
            if not Es.Is_Empty then
               Sorting.Sort (Es);
               Md := Es (Natural (Es.Length) / 2);
            end if;
         end;
      end Place_Arm;

      --  全部放完以后一起精修:每只放进世界的别的手(相似变换 7 个数)、不长在手上的那只眼(位姿 6 个 + 焦距)用全部配上的点一起按像素解
      --  (抗野点,两轮门同上),加每只手"桌面重合"那 3 条。先放进世界的也被后放进的证据修正(按放进去的先后,前面的定下以后后面的就不再动它 ⇒ 错会被锁死)
      procedure Joint_Refine is
         Arm_Slot : array (0 .. Natural'Max (1, Na) - 1) of Integer := [others => -1];
         Cam_Slot : Integer := -1;
         Np : Natural := 0;
         Nv : constant Natural := Natural (Wv.Length);
         Fr_R : array (0 .. Natural'Max (1, Nv) - 1) of M3 := [others => Identity];
         Fr_T : array (0 .. Natural'Max (1, Nv) - 1) of V3 := [others => [0.0, 0.0, 0.0]];
         procedure Map_Of (Xx : Kinem.Vec; A : Natural; Sx : out Long_Float; Rx : out M3; Tx : out V3) is
         begin
            if Arm_Slot (A) >= 0 then
               declare
                  K : constant Natural := Xx'First + Natural (Arm_Slot (A));
               begin
                  Rx := Rodrigues ([Xx (K), Xx (K + 1), Xx (K + 2)]); Tx := [Xx (K + 3), Xx (K + 4), Xx (K + 5)]; Sx := Exp (Xx (K + 6));
               end;
            else
               Sx := Worlds (A).S; Rx := Worlds (A).Ra; Tx := Worlds (A).Ta;
            end if;
         end Map_Of;
         function Cam_Now (Xx : Kinem.Vec; K : Natural) return Cam_Geo is
            Cg : Cam_Geo := Wv (K).Cam;
         begin
            if Wv (K).Arm >= 0 then
               declare
                  Sx : Long_Float;
                  Rx : M3;
                  Tx, Rt : V3;
               begin
                  Map_Of (Xx, Natural (Wv (K).Arm), Sx, Rx, Tx);
                  Rt := Ap (Rx, Fr_T (K));
                  Cg.R_Ce := Mul (Rx, Fr_R (K));
                  Cg.Pos := [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)];
               end;
            elsif Cam_Slot >= 0 then
               declare
                  J : constant Natural := Xx'First + Natural (Cam_Slot);
               begin
                  Cg.R_Ce := Rodrigues ([Xx (J), Xx (J + 1), Xx (J + 2)]);
                  Cg.Pos := [Xx (J + 3), Xx (J + 4), Xx (J + 5)];
                  Cg.F := Exp (Xx (J + 6));
               end;
            end if;
            return Cg;
         end Cam_Now;
         Fx_K : Integer := -1;   --  不动的眼在 Wv 里第几个
         --  全部残差(门里的):手的配点(第 B 只手的点投回世界里那只眼)、不动的眼的配点(世界里的手的点投进它)、每只手桌面 3 条
         procedure All_Res (Xx : Kinem.Vec; Rr : out Kinem.Vec; Use_A : Bools; Use_C : Bools; Fill_E : Boolean; Ea, Ec : in out Floats) is
            J : Natural := Rr'First;
         begin
            Ea.Clear; Ec.Clear;
            for B in 1 .. Na - 1 loop
               if Arm_Slot (B) >= 0 then
                  declare
                     Sx : Long_Float;
                     Rx : M3;
                     Tx : V3;
                  begin
                     Map_Of (Xx, B, Sx, Rx, Tx);
                     for K in 0 .. Natural (Arm_Obs (B).Length) - 1 loop
                        declare
                           Ob : constant Arm_Ob := Arm_Obs (B) (K);
                           E : constant Long_Float := Samp2 (Cam_B (B, Ob.Bf, Sx, Rx, Tx), Ob.Uw, Ob.Vw, Cam_Now (Xx, Ob.Wk), Ob.U, Ob.V);
                        begin
                           if Fill_E then
                              Ea.Append (abs E);
                           elsif Use_A (Natural (Ea.Length)) then
                              Rr (J) := E; J := J + 1;
                           end if;
                           if not Fill_E then
                              Ea.Append (0.0);
                           end if;
                        end;
                     end loop;
                     if not Fill_E then
                        Plane_Res (B, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2)); J := J + 3;
                     end if;
                  end;
               end if;
            end loop;
            if Cam_Slot >= 0 and then Fx_K >= 0 then
               declare
                  Cg : constant Cam_Geo := Cam_Now (Xx, Natural (Fx_K));
               begin
                  for X of Cam_Obs loop
                     if X.Pa = 0 or else Arm_Slot (X.Pa) >= 0 then
                        declare
                           E : constant Long_Float := Samp2 (Cam_Now (Xx, X.Wk), X.Uw, X.Vw, Cg, X.U, X.V);
                        begin
                           if Fill_E then
                              Ec.Append (abs E);
                           elsif Use_C (Natural (Ec.Length)) then
                              Rr (J) := E; J := J + 1;
                           end if;
                           if not Fill_E then
                              Ec.Append (0.0);
                           end if;
                        end;
                     end if;
                  end loop;
               end;
            end if;
         end All_Res;
      begin
         for B in 1 .. Na - 1 loop
            if Placed (B) and then Worlds (B).Valid then
               Arm_Slot (B) := Np; Np := Np + 7;
            end if;
         end loop;
         for K in 0 .. Nv - 1 loop
            if Wv (K).Arm < 0 then
               Fx_K := K;
            else
               Kinem.FK (Worlds (Natural (Wv (K).Arm)).Model, Ds (Natural (Wv (K).Arm)).Frames (Wv (K).Frame).Q, Fr_R (K), Fr_T (K));
            end if;
         end loop;
         if Fx_Placed and then Fx_K >= 0 then
            Cam_Slot := Np; Np := Np + 7;
         end if;
         if Np = 0 then
            return;
         end if;
         declare
            X : Kinem.Vec (0 .. Np - 1);
            Steps : constant Kinem.Vec (0 .. Np - 1) := [others => 1.0e-7];   --  差分步(极小量,无量纲)
         begin
            for B in 1 .. Na - 1 loop
               if Arm_Slot (B) >= 0 then
                  declare
                     K : constant Natural := Natural (Arm_Slot (B));
                     Rv : constant V3 := Rot_Vec (Worlds (B).Ra);
                  begin
                     X (K .. K + 6) := [Rv (0), Rv (1), Rv (2), Worlds (B).Ta (0), Worlds (B).Ta (1), Worlds (B).Ta (2), Log (Worlds (B).S)];
                  end;
               end if;
            end loop;
            if Cam_Slot >= 0 then
               declare
                  K : constant Natural := Natural (Cam_Slot);
                  Rv : constant V3 := Rot_Vec (G.R_Ce);
               begin
                  X (K .. K + 6) := [Rv (0), Rv (1), Rv (2), G.Pos (0), G.Pos (1), G.Pos (2), Log (G.F)];
               end;
            end if;
            for Round in 1 .. 2 loop
               declare
                  Ea, Ec : Floats;
                  Dummy : Kinem.Vec (0 .. 0);
                  Ua, Uc : Bools;
                  Nua, Nuc, N_Planes : Natural := 0;
               begin
                  All_Res (X, Dummy, Bool_Vectors.Empty_Vector, Bool_Vectors.Empty_Vector, True, Ea, Ec);
                  declare
                     Ga : constant Long_Float := Gate_Of (Ea);
                     Gc : constant Long_Float := Gate_Of (Ec);
                  begin
                     for E of Ea loop
                        Ua.Append (E < Ga);
                        if E < Ga then
                           Nua := Nua + 1;
                        end if;
                     end loop;
                     for E of Ec loop
                        Uc.Append (E < Gc);
                        if E < Gc then
                           Nuc := Nuc + 1;
                        end if;
                     end loop;
                  end;
                  for B in 1 .. Na - 1 loop
                     if Arm_Slot (B) >= 0 then
                        N_Planes := N_Planes + 1;
                     end if;
                  end loop;
                  declare
                     N_Res : constant Natural := Nua + Nuc + 3 * N_Planes;
                     procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                        E1, E2 : Floats;
                     begin
                        All_Res (Xx, Rr, Ua, Uc, False, E1, E2);
                     end Resid;
                  begin
                     if N_Res > Np then
                        Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access);
                     end if;
                  end;
               end;
            end loop;
            --  写回去
            for B in 1 .. Na - 1 loop
               if Arm_Slot (B) >= 0 then
                  declare
                     K : constant Natural := Natural (Arm_Slot (B));
                     Wb : Arm_World := Worlds (B);
                  begin
                     Wb.Ra := Rodrigues ([X (K), X (K + 1), X (K + 2)]); Wb.Ta := [X (K + 3), X (K + 4), X (K + 5)]; Wb.S := Exp (X (K + 6));
                     Worlds.Replace_Element (B, Wb);
                  end;
               end if;
            end loop;
            if Cam_Slot >= 0 then
               declare
                  K : constant Natural := Natural (Cam_Slot);
               begin
                  G.R_Ce := Rodrigues ([X (K), X (K + 1), X (K + 2)]); G.Pos := [X (K + 3), X (K + 4), X (K + 5)]; G.F := Exp (X (K + 6));
               end;
            end if;
            for K in 0 .. Nv - 1 loop
               declare
                  Vw : World_View := Wv (K);
               begin
                  Vw.Cam := Cam_Now (X, K);
                  Wv.Replace_Element (K, Vw);
               end;
            end loop;
         end;
      end Joint_Refine;
   begin
      Rw := Identity; O := [0.0, 0.0, 0.0]; Ok := False; Fixed_Eye := No_Geo;
      if Worlds.Is_Empty or else not Worlds (0).Valid then
         return;
      end if;
      for A in 0 .. Na - 1 loop
         if Worlds (A).Valid then
            Tri_Pts (Worlds (A).Model, Ds (A), Css (A), Max_Pts, Ps (A));
         end if;
      end loop;
      if Natural (Ps (0).Length) < Min_Inl then
         Say ("世界:第一只手只三角出 " & Codec.Img (Natural (Ps (0).Length)) & " 个点 ⇒ 定不了桌面");
         return;
      end if;
      --  世界的桌面:"上" = 桌面法向(朝第一只手的眼),原点 = 那只眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上
      declare
         Inl : Natural;
         Md : Long_Float;
      begin
         Plane_Of (Ps (0), Pl0, N0, Gates (0), Inl, Md);
         if Dist ([0.0, 0.0, 0.0], Pl0, N0) < 0.0 then
            N0 := [-N0 (0), -N0 (1), -N0 (2)];
         end if;
         Pls (0) := Pl0; Nrs (0) := N0;
         declare
            Dn : constant Long_Float := N0 (0);
            Xr0 : constant V3 := [1.0 - Dn * N0 (0), -Dn * N0 (1), -Dn * N0 (2)];
            Xr : constant V3 := [Xr0 (0) / Norm (Xr0), Xr0 (1) / Norm (Xr0), Xr0 (2) / Norm (Xr0)];
            Yr : constant V3 := [N0 (1) * Xr (2) - N0 (2) * Xr (1), N0 (2) * Xr (0) - N0 (0) * Xr (2), N0 (0) * Xr (1) - N0 (1) * Xr (0)];
            Ph : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2);
         begin
            Rw := [[Xr (0), Xr (1), Xr (2)], [Yr (0), Yr (1), Yr (2)], [N0 (0), N0 (1), N0 (2)]];
            O := [Ph * N0 (0), Ph * N0 (1), Ph * N0 (2)];
            Say ("世界:第一只手三角出 " & Codec.Img (Natural (Ps (0).Length)) & " 个点,桌面拟合了 " & Codec.Img (Inl) & " 个(离面中位 " & Codec.Fmt (Md, 4)
                 & " 单位)⇒ 上 = 桌面法向;眼离桌面 " & Codec.Fmt (abs Ph, 3) & " 单位(长度单位 = 第一只手的模型单位)");
         end;
      end;
      Ok := True;
      Placed (0) := True;
      for B in 1 .. Na - 1 loop
         if Worlds (B).Valid and then not Ps (B).Is_Empty then
            declare
               Inl : Natural;
               Md : Long_Float;
            begin
               Plane_Of (Ps (B), Pls (B), Nrs (B), Gates (B), Inl, Md);
               Plane_Info (B);
               Say ("世界:第" & Codec.Img (B + 1) & " 只手三角出 " & Codec.Img (Natural (Ps (B).Length)) & " 个点,它自己的桌面拟合了 " & Codec.Img (Inl) & " 个(离面中位 "
                    & Codec.Fmt (Md, 4) & ",算在面上的门 " & Codec.Fmt (Gates (B), 4) & " 单位)");
            end;
         end if;
      end loop;
      --  整体特征:每只手有三角点的格子 + 不长在手上的眼
      for A in 0 .. Na - 1 loop
         if Worlds (A).Valid then
            for F of Frames_Of (A) loop
               if Id_Of (A, Natural (F)) >= 0 and then not D_Ids.Contains (Id_Of (A, Natural (F))) then
                  D_Ids.Append (Id_Of (A, Natural (F)));
               end if;
            end loop;
         end if;
      end loop;
      if Fx_Id >= 0 then
         D_Ids.Append (Fx_Id);
      end if;
      declare
         Err : Unbounded_String;
      begin
         D_Vec := Instrument.Describe (Host, Port, D_Ids, Err);
         if Natural (D_Vec.Length) /= Natural (D_Ids.Length) then
            Say ("世界:配点仪器没给整体特征(" & To_String (Err) & ")⇒ 只有第一只手在世界里");
            D_Ids.Clear;
         end if;
      end;
      Add_Arm_Views (0);
      --  一轮一轮放
      loop
         declare
            Best_Inl : Natural := 0;
            Best : Integer := -2;   --  -1 = 不动的眼;≥ 1 = 第几只手;-2 = 这一轮谁都放不进
         begin
            --  不动的眼:世界里有点的眼(手的格子)按像它的程度挑没配过的
            if Fx_Id >= 0 and then not Fx_Placed and then not D_Ids.Is_Empty then
               declare
                  type Cand is record
                     K : Natural := 0;
                     S : Long_Float := 0.0;
                  end record;
                  package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
                  Cs : Cand_Vectors.Vector;
               begin
                  for K in 0 .. Natural (Wv.Length) - 1 loop
                     if Wv (K).Arm >= 0 and then not Was_Tried (Wv (K).Id, Fx_Id) then
                        declare
                           Sc : constant Long_Float := Desc_Dot (Wv (K).Id, Fx_Id);
                           Pos : Natural := Natural (Cs.Length);
                        begin
                           for J in 0 .. Natural (Cs.Length) - 1 loop
                              if Sc > Cs (J).S then
                                 Pos := J;
                                 exit;
                              end if;
                           end loop;
                           Cs.Insert (Pos, Cand'(K => K, S => Sc));
                        end;
                     end if;
                  end loop;
                  for C of Cs loop
                     declare
                        Vw : constant World_View := Wv (C.K);
                        Idx, Keep : Ints;
                        Q : Instrument.Match_Vectors.Vector;
                        U, V, E : Floats;
                     begin
                        Pts_In (Natural (Vw.Arm), Vw.Frame, Idx, Q);
                        Match_Pair (Vw.Id, Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
                        N_Pairs_Fx := N_Pairs_Fx + 1;
                        for I in 0 .. Natural (Keep.Length) - 1 loop
                           declare
                              P : constant Tri_Pt := Ps (Natural (Vw.Arm)) (Natural (Idx (Natural (Keep (I)))));
                           begin
                              Cam_Obs.Append (Cam_Ob'(Pa => Natural (Vw.Arm), Pk => Natural (Idx (Natural (Keep (I)))), Wk => C.K, Uw => Q (Natural (Keep (I))).U,
                                                      Vw => Q (Natural (Keep (I))).V, Xw => To_World (Natural (Vw.Arm), P.X),
                                                      Cw => Cov_World (Natural (Vw.Arm), P.Cov), U => U (I), V => V (I), E => E (I)));
                           end;
                        end loop;
                     end;
                  end loop;
                  declare
                     Gt : Cam_Geo;
                     Rp : Fixed_Report;
                     Inl : Natural;
                  begin
                     Fit_Cam (Gt, Rp, Inl);
                     if Inl >= Min_Inl and then Inl > Best_Inl then
                        Best_Inl := Inl; Best := -1;
                     end if;
                  end;
               end;
            end if;
            --  别的手:它的格子 × 世界里的眼,按像不像挑没配过的
            for B in 1 .. Na - 1 loop
               if Worlds (B).Valid and then not Placed (B) and then not Ps (B).Is_Empty and then not D_Ids.Is_Empty then
                  declare
                     type Cand is record
                        F, K : Natural := 0;
                        S : Long_Float := 0.0;
                     end record;
                     package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
                     Cs : Cand_Vectors.Vector;
                  begin
                     --  世界里每一只还没和它配过的眼:挑它最像那只眼的一格去配一次
                     for K in 0 .. Natural (Wv.Length) - 1 loop
                        declare
                           Tried_K : Boolean := False;
                           Bf : Integer := -1;
                           Bs : Long_Float := Long_Float'First;
                        begin
                           for F of Frames_Of (B) loop
                              if Id_Of (B, Natural (F)) >= 0 then
                                 Tried_K := Tried_K or else Was_Tried (Id_Of (B, Natural (F)), Wv (K).Id);
                                 if Desc_Dot (Id_Of (B, Natural (F)), Wv (K).Id) > Bs then
                                    Bs := Desc_Dot (Id_Of (B, Natural (F)), Wv (K).Id); Bf := F;
                                 end if;
                              end if;
                           end loop;
                           if not Tried_K and then Bf >= 0 then
                              declare
                                 Pos : Natural := Natural (Cs.Length);
                              begin
                                 for J in 0 .. Natural (Cs.Length) - 1 loop
                                    if Bs > Cs (J).S then
                                       Pos := J;
                                       exit;
                                    end if;
                                 end loop;
                                 Cs.Insert (Pos, Cand'(F => Natural (Bf), K => K, S => Bs));
                              end;
                           end if;
                        end;
                     end loop;
                     for C of Cs loop
                        declare
                           Vw : constant World_View := Wv (C.K);
                           Idx, Keep : Ints;
                           Q : Instrument.Match_Vectors.Vector;
                           U, V, E : Floats;
                        begin
                           Pts_In (B, C.F, Idx, Q);
                           Match_Pair (Id_Of (B, C.F), Vw.Id, Vw.W, Vw.H, Q, Keep, U, V, E);
                           N_Pairs_Of (B) := N_Pairs_Of (B) + 1;
                           declare
                              N_On : Natural := 0;
                           begin
                              for I in 0 .. Natural (Keep.Length) - 1 loop
                                 if abs Dist (Ps (B) (Natural (Idx (Natural (Keep (I))))).X, Pls (B), Nrs (B)) <= Gates (B) then
                                    N_On := N_On + 1;
                                 end if;
                              end loop;
                              Append (Round_Note, " " & Codec.Img (C.F) & "→" & (if Vw.Arm < 0 then "眼" else "手" & Codec.Img (Vw.Arm + 1) & "格" & Codec.Img (Vw.Frame))
                                      & ":" & Codec.Img (Natural (Q.Length)) & "/" & Codec.Img (Natural (Keep.Length)) & "/" & Codec.Img (N_On));
                           end;
                           for I in 0 .. Natural (Keep.Length) - 1 loop
                              Arm_Obs (B).Append (Arm_Ob'(K => Natural (Idx (Natural (Keep (I)))), Bf => C.F, Wk => C.K, Ro => Vw.Cam.Pos, Rd => Ray_Fixed (Vw.Cam, U (I), V (I)),
                                                          U => U (I), V => V (I), Uw => Q (Natural (Keep (I))).U, Vw => Q (Natural (Keep (I))).V, E => E (I)));
                           end loop;
                        end;
                     end loop;
                     declare
                        S, Md : Long_Float;
                        R : M3;
                        T : V3;
                        Inl : Natural;
                     begin
                        Place_Arm (B, S, R, T, Inl, Md);
                        Say ("  第" & Codec.Img (B + 1) & " 只手这一轮配了(它的第几格 → 世界里的哪只眼:问几个点 / 往返配上几个 / 其中在它桌面上的):" & To_String (Round_Note)
                             & " ⇒ 一共配上 " & Codec.Img (Natural (Arm_Obs (B).Length)) & " 次,放进世界后投回去差不到 3 px 的 " & Codec.Img (Inl) & " 次(残差中位 "
                             & Codec.Fmt (Md, 2) & " px)· 长度倍数 " & Codec.Fmt (S, 4));
                        Round_Note := Null_Unbounded_String;
                        if Inl >= Min_Inl and then Inl > Best_Inl then
                           Best_Inl := Inl; Best := B;
                        end if;
                     end;
                  end;
               end if;
            end loop;
            exit when Best = -2;
            if Best = -1 then
               declare
                  Inl : Natural;
               begin
                  Fit_Cam (G, G_Rep, Inl);
                  Fx_Placed := True;
                  Wv.Append (World_View'(Id => Fx_Id, Cam => G, Arm => -1, Frame => 0, W => Ds (0).World_Img.W, H => Ds (0).World_Img.H));
                  Say ("放进世界:不长在手上的那只眼 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,往返 1 px 内共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length))
                       & " 个 ⇒ 焦距 " & Codec.Fmt (G.F, 1) & "、残差 " & Codec.Fmt (G_Rep.Scene_Rms, 2) & " px(进解 " & Codec.Img (G_Rep.Scene_Used) & " / "
                       & Codec.Img (G_Rep.Scene_N) & ")、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位");
               end;
            else
               declare
                  S, Md : Long_Float;
                  R : M3;
                  T : V3;
                  Inl : Natural;
                  Wb : Arm_World := Worlds (Natural (Best));
                  N_Fx, N_Arm : Natural := 0;
               begin
                  Place_Arm (Natural (Best), S, R, T, Inl, Md);
                  Wb.S := S; Wb.Ra := R; Wb.Ta := T;
                  Worlds.Replace_Element (Natural (Best), Wb);
                  Placed (Natural (Best)) := True;
                  Add_Arm_Views (Natural (Best));
                  for P of Tried loop
                     if Ds (Natural (Best)).Ids.Contains (P.A) then
                        if P.B = Fx_Id then
                           N_Fx := N_Fx + 1;
                        else
                           N_Arm := N_Arm + 1;
                        end if;
                     end if;
                  end loop;
                  Say ("放进世界:第" & Codec.Img (Natural (Best) + 1) & " 只手 —— 配了 " & Codec.Img (N_Pairs_Of (Natural (Best))) & " 对画面(对不长在手上的眼 " & Codec.Img (N_Fx)
                       & " 对、对别的手的格子 " & Codec.Img (N_Arm) & " 对),往返 1 px 内配上 " & Codec.Img (Natural (Arm_Obs (Natural (Best)).Length))
                       & " 次,放进世界后投回去差不到 3 px 的 " & Codec.Img (Inl) & " 次(残差中位 " & Codec.Fmt (Md, 2) & " px)⇒ 长度倍数 " & Codec.Fmt (S, 4));
               end;
            end if;
         end;
      end loop;
      Joint_Refine;
      for B in 1 .. Na - 1 loop
         if Placed (B) then
            Say ("一起精修以后:第" & Codec.Img (B + 1) & " 只手 长度倍数 " & Codec.Fmt (Worlds (B).S, 4) & "、平移 (" & Codec.Fmt (Worlds (B).Ta (0), 3) & ", "
                 & Codec.Fmt (Worlds (B).Ta (1), 3) & ", " & Codec.Fmt (Worlds (B).Ta (2), 3) & ") 单位");
         end if;
      end loop;
      if Fx_Placed then
         Say ("一起精修以后:不长在手上的那只眼 焦距 " & Codec.Fmt (G.F, 1) & "、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位");
      end if;
      for B in 1 .. Na - 1 loop
         if Worlds (B).Valid and then not Placed (B) then
            declare
               Wb : Arm_World := Worlds (B);
            begin
               Wb.Valid := False;
               Worlds.Replace_Element (B, Wb);
               Say ("世界:第" & Codec.Img (B + 1) & " 只手放不进世界 —— 配了 " & Codec.Img (N_Pairs_Of (B)) & " 对画面,共同看见的点只有 " & Codec.Img (Natural (Arm_Obs (B).Length))
                    & " 个(它和世界里的眼没有共同看见的东西)⇒ 这只手先不用");
            end;
         end if;
      end loop;
      if Fx_Id >= 0 and then not Fx_Placed then
         Say ("世界:不长在手上的那只眼放不进世界 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length)) & " 个");
      end if;
      if Fx_Placed then
         Fixed_Eye := G;
         Fixed_Eye.R_Ce := Mul (Rw, G.R_Ce);
         Fixed_Eye.Pos := Ap (Rw, [G.Pos (0) - O (0), G.Pos (1) - O (1), G.Pos (2) - O (2)]);
      end if;
      if Dump /= "" then
         declare
            Fo : Ada.Text_IO.File_Type;
         begin
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/world.txt");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Ada.Text_IO.Put (Fo, Codec.Fmt (Rw (I, J), 9) & " ");
               end loop;
            end loop;
            Ada.Text_IO.Put_Line (Fo, Codec.Fmt (O (0), 9) & " " & Codec.Fmt (O (1), 9) & " " & Codec.Fmt (O (2), 9));
            Ada.Text_IO.Close (Fo);
            if Fx_Placed then
               Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/fixed_eye.txt");
               Ada.Text_IO.Put (Fo, "f " & Codec.Fmt (G.F, 6) & " cx " & Codec.Fmt (G.Cx, 3) & " cy " & Codec.Fmt (G.Cy, 3) & " rms " & Codec.Fmt (G_Rep.Scene_Rms, 4)
                                & " used " & Codec.Img (G_Rep.Scene_Used) & " of " & Codec.Img (G_Rep.Scene_N) & " pos " & Codec.Fmt (G.Pos (0), 9) & " "
                                & Codec.Fmt (G.Pos (1), 9) & " " & Codec.Fmt (G.Pos (2), 9) & " R");
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     Ada.Text_IO.Put (Fo, " " & Codec.Fmt (G.R_Ce (I, J), 9));
                  end loop;
               end loop;
               Ada.Text_IO.New_Line (Fo);
               Ada.Text_IO.Close (Fo);
            end if;
            for B in 1 .. Na - 1 loop
               if Placed (B) then
                  Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/align_arm" & Codec.Img (B) & ".txt");
                  Ada.Text_IO.Put (Fo, "S " & Codec.Fmt (Worlds (B).S, 9) & " R");
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Ada.Text_IO.Put (Fo, " " & Codec.Fmt (Worlds (B).Ra (I, J), 9));
                     end loop;
                  end loop;
                  Ada.Text_IO.Put_Line (Fo, " T " & Codec.Fmt (Worlds (B).Ta (0), 9) & " " & Codec.Fmt (Worlds (B).Ta (1), 9) & " " & Codec.Fmt (Worlds (B).Ta (2), 9));
                  for X of Arm_Obs (B) loop
                     declare
                        Xb : constant V3 := Ps (B) (X.K).X;
                        Rt : constant V3 := Ap (Worlds (B).Ra, Xb);
                        Xw : constant V3 := [Worlds (B).S * Rt (0) + Worlds (B).Ta (0), Worlds (B).S * Rt (1) + Worlds (B).Ta (1), Worlds (B).S * Rt (2) + Worlds (B).Ta (2)];
                     begin
                        Ada.Text_IO.Put_Line (Fo, Codec.Img (Wv (X.Wk).Arm) & " " & Codec.Img (Wv (X.Wk).Frame) & " " & Codec.Img (X.K) & " "
                                              & Codec.Fmt (X.U, 3) & " " & Codec.Fmt (X.V, 3) & " " & Codec.Fmt (X.Uw, 3) & " " & Codec.Fmt (X.Vw, 3) & " " & Codec.Fmt (X.E, 3)
                                              & " " & Codec.Fmt (Xw (0), 6) & " " & Codec.Fmt (Xw (1), 6) & " " & Codec.Fmt (Xw (2), 6)
                                              & " " & Codec.Fmt (Xb (0), 6) & " " & Codec.Fmt (Xb (1), 6) & " " & Codec.Fmt (Xb (2), 6));
                     end;
                  end loop;
                  Ada.Text_IO.Close (Fo);
               end if;
            end loop;
         exception
            when others => null;
         end;
      end if;
      Say ("  对齐用了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒");
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
