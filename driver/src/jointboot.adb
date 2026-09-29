with Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers;
with Ada.Calendar;
with Codec;
with Picture;
with Table;
with Instrument;
with Layout;
with Ada.Directories;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Jointboot is

   procedure Say (S : String) is
   begin
      Ada.Text_IO.Put_Line ("[身] 📐 " & S);
   end Say;

   --  落盘时一根轴是转还是走(格式里的一个词)
   function Kind_Word (A : Kinem.Axis) return String is (if A.Slide then "slide" else "turn");

   Start_Amp : constant Long_Float := 1.0e-4;   --  探针协议的起点(同 Selfmap:极小,翻倍到走得出来又看得见为止;无量纲协议)
   Max_Doublings : constant := 12;              --  次数
   Grow : constant := 2.0;                      --  探针每次翻一倍(次数:同 Selfmap 的探针协议)
   Third : constant := 1.0 / 3.0;               --  三分之一(比例:"没转到命令的三分之一 = 被顶住",到没到目标用同一个比例)
   Ramp : constant := 4.0;                      --  扫描下一格最多放大四倍(次数;每个方向只停 3 格,头一格之后两格就要转开)

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
                     F1, F1b : Plug.Cam_Vectors.Vector;   --  转到那头:走完那一帧、再读的一帧
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
                     --  转到那头再读一帧:画面比读数晚一拍(开机量的),"读数到了"那一刻的前一帧画面还没到那头(V1B58 2026-09-28:拿它当第二帧,
                     --  x5 第 1 只手 0.0001 / 0.0002 两档都判成没看见、第 2 只手认不出眼);走完那一帧和再读的这一帧都是到了以后的画面
                     exit when not Plug.Sense (L, F);
                     F1b := F.Cams;
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
                           --  看没看见动了:两次比较、不共用一帧(转之前 → 走完那一帧;再读的那一帧 → 转回来;同逐通道推、抓握通道推到头:Picture.Seen_Twice)
                           Comps : constant Picture.Regions :=
                             Picture.Seen_Twice (F0 (C).Gray, F1 (C).Gray, F1b (C).Gray, F.Cams (C).Gray, Fl, F.Cams (C).W, F.Cams (C).H);
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
      --  ── 配点:扫描时把要配的对攒着,扫完再让配点仪器一对一对配(粗配,一对约 0.33 秒)。
      --  边扫边配试过(V1B6 2026-09-26):仿真和配点仪器在同一块 GPU 上抢,一拍从 0.67 秒变 1.17 秒,比扫完再配还慢。
      --  只有"起点 ↔ 每段头一格"那一对在扫描时当场配(仿真等着,不抢):下一格的步子要按它定;配出来的点留给运动学,不再配一遍 ──
      type Job is record
         A, I, J : Natural := 0;
         Ia, Ib : Natural := 0;       --  两帧在仪器那边的编号
         W, H : Natural := 0;         --  画幅(第 I 帧的格点按它铺)
         Serial : Natural := 0;       --  第几对(轨迹号按它分开:起点那帧以外的对各自编号)
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
      Res : array (0 .. Natural'Max (1, Na) - 1) of Kinem.Corr_Vectors.Vector;   --  扫描时只有扫描自己写(配点线程扫完才开始);之后只有配点线程写,它结束以后才读
      N_Jobs, N_Empty, N_Now : Natural := 0;
      T_Now : Duration := 0.0;   --  扫描时当场配花的时间(秒)
      --  ── 问格点(09-27 V1B32 改):第 I 帧上铺 Gx × Gy 的格点,仪器配进第 J 帧,往返 1 px 内的留下。格点号就是轨迹号:
      --  起点那帧(I = 0)的对全问同一张格点 ⇒ 同一个格点跨很多帧 = 一条轨迹(运动学最后按重投影一起解要它:两两对极管不住轴离眼多远);
      --  别的对(相邻格、交叉格)各自编号,成两帧的轨迹。原来让仪器每对随机抽 2500 个,同一个点不跨格
      Trip_Sweep : constant := Geom.Trip_Px;   --  往返 1 px 内的才算(同对齐、核对不动的眼)
      Ng : constant := Gx * Gy;
      procedure Grid_Match (I, J, Ia, Ib, W, H, Serial : Natural; Into : in out Kinem.Corr_Vectors.Vector; Disp : in out Floats; Got : out Boolean) is
         Q : Instrument.Match_Vectors.Vector;
         Err : Unbounded_String;
      begin
         for Gyy in 0 .. Gy - 1 loop
            for Gxx in 0 .. Gx - 1 loop
               Q.Append (Instrument.Match_Pt'(U => (Long_Float (Gxx) + 0.5) * Long_Float (W) / Long_Float (Gx),
                                              V => (Long_Float (Gyy) + 0.5) * Long_Float (H) / Long_Float (Gy), others => <>));
            end loop;
         end loop;
         declare
            --  粗配:一对 0.39 秒对 0.83 秒,和完整配点只差中位 0.07–0.13 px、九成 0.2–0.6 px(trackexam 2026-09-26,V1B4 两段扫描)
            R : constant Instrument.Match_Vectors.Vector := Instrument.Match_Ids (Host, Port, Ia, Ib, Q, Err, Coarse => True, Back => True);
         begin
            Got := Natural (R.Length) = Natural (Q.Length);
            if Got then
               for G in 0 .. Natural (Q.Length) - 1 loop
                  if R (G).Bu >= 0.0 and then R (G).U >= 0.0 and then R (G).U < Long_Float (W) and then R (G).V >= 0.0 and then R (G).V < Long_Float (H)
                    and then Geom.Norm ([R (G).Bu - Q (G).U, R (G).Bv - Q (G).V, 0.0]) < Trip_Sweep
                  then
                     Into.Append (Kinem.Corr'(I => I, J => J, Ua => Q (G).U, Va => Q (G).V, Ub => R (G).U, Vb => R (G).V,
                                              Pt => (if I = 0 then G else Ng * (1 + Serial) + G)));
                     Disp.Append (Geom.Norm ([R (G).U - Q (G).U, R (G).V - Q (G).V, 0.0]));
                  end if;
               end loop;
            end if;
         end;
      end Grid_Match;
      task Matcher with Storage_Size => 16 * 1024 * 1024;
      task body Matcher is
         X : Job;
         Done : Boolean;
         Got : Boolean;
         Dv : Floats;
      begin
         loop
            Jobs.Get (X, Done);
            exit when Done;
            Grid_Match (X.I, X.J, X.Ia, X.Ib, X.W, X.H, X.Serial, Res (X.A), Dv, Got);
            Dv.Clear;
            if not Got then
               N_Empty := N_Empty + 1;
            end if;
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
         Step, Off, Q_Prev : Long_Float := 0.0;
         Px_Per : Long_Float := 0.0;              --  这个关节每转一个读数单位画面挪几像素(这一段头一格和起点那一对配点量的;0 = 没量到)
         Tgt : Floats;
         K : Natural := 0;
         Done : Boolean := True;
         Why : Unbounded_String;
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
         --  存到仪器那边;起点 ↔ 这一格:一段的头一格当场配(下一格的步子按它定),别的交给后台配
         Instrument.Frame_Put (Host, Port, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Id, Err);
         Sa.Ids.Append (Id);
         if K = 1 and then not Multi and then Id >= 0 and then Sa.Ids (0) >= 0 then
            declare
               package Sorting is new F64_Vectors.Generic_Sorting;
               Tn : constant Ada.Calendar.Time := Ada.Calendar.Clock;
               Fr : constant Natural := Natural (Ds (A).Frames.Length) - 1;
               Dv : Floats;
               Dq : constant Long_Float := abs (F.Joints (Sa.G) (J) - Sa.Q0 (J));
               Got : Boolean;
            begin
               Grid_Match (0, Fr, Natural (Sa.Ids (0)), Natural (Id), Sa.W, Sa.H, 0, Res (A), Dv, Got);
               N_Now := N_Now + 1;
               if not Got then
                  N_Empty := N_Empty + 1;
               end if;
               --  这一段画面挪了多少 = 这一对配点挪动的中位数;÷ 实到的转角 = 每个读数单位挪几像素
               Sa.Px_Per := 0.0;
               if not Dv.Is_Empty and then Dq > 0.0 then
                  Sorting.Sort (Dv);
                  Sa.Px_Per := Dv (Natural (Dv.Length) / 2) / Dq;
               end if;
               T_Now := T_Now + Ada.Calendar."-" (Ada.Calendar.Clock, Tn);
            end;
         elsif K > 0 and then Id >= 0 and then Sa.Ids (0) >= 0 then
            Jobs.Put ((A => A, I => 0, J => Natural (Ds (A).Frames.Length) - 1, Ia => Natural (Sa.Ids (0)), Ib => Natural (Id), W => Sa.W, H => Sa.H, Serial => N_Jobs));
            N_Jobs := N_Jobs + 1;
         end if;
         --  每段头两格之间也配(转角小的对:每根轴单独起步时网格只用转角 ≤ 16° 的对;V1B5 只配起点 ↔ 每一格,两根轴没有够用的小转角对);
         --  几个关节一起动的格子:相邻两格也配
         if (K = 2 or else (Multi and then K >= 2)) and then Id >= 0 and then Natural (Sa.Ids.Length) >= 2 and then Sa.Ids (Natural (Sa.Ids.Length) - 2) >= 0 then
            Jobs.Put ((A => A, I => Natural (Ds (A).Frames.Length) - 2, J => Natural (Ds (A).Frames.Length) - 1,
                       Ia => Natural (Sa.Ids (Natural (Sa.Ids.Length) - 2)), Ib => Natural (Id), W => Sa.W, H => Sa.H, Serial => N_Jobs));
            N_Jobs := N_Jobs + 1;
         end if;
         if Dump /= "" then
            Codec.Write_BMP (Dump & "/" & Nm, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H);
            N_Img := N_Img + 1;
         end if;
      end Keep;
      --  Swept ≥ 0 = 单关节扫描这一格:扫的那根到了按 Tol(这一格的三分之一);别的关节要差不到每根轴单独起步收格子的门(Kinem.Clean_Tol)才算到 ——
      --  读的这一帧才用得上(H1 2026-09-28:人形别的关节还偏 0.001–0.009 就读了,格子全不干净,两只手运动学没量成;x5 每只手 35 格里 7 格同样不干净)
      procedure Move_All (Tol : Long_Float; Swept : Integer := -1) is
         Gs : Ints;
         Qs, Ts : Plug.Floats_Vectors.Vector;
         Dl : Table.Vec;
         Fr : Natural;
      begin
         for A in 0 .. Na - 1 loop
            if St (A).Live then
               Gs.Append (St (A).G); Qs.Append (St (A).Tgt);
               declare
                  Tj : Floats;
               begin
                  for Jx in 0 .. Natural (St (A).Tgt.Length) - 1 loop
                     Tj.Append (if Jx = Swept then Tol else Kinem.Clean_Tol (Long_Float (St (A).W)));
                  end loop;
                  Ts.Append (Tj);
               end;
            end if;
         end loop;
         Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okc, Groups => Gs, Qs => Qs, Tol => Tol,
                     Tols => (if Swept >= 0 then Ts else Plug.Floats_Vectors.Empty_Vector));
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
               D.Has_Lo := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
               D.Has_Hi := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
               D.Step_Lo := F64_Vectors.To_Vector (0.0, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
               D.Step_Hi := F64_Vectors.To_Vector (0.0, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
               Ds.Replace_Element (A, D);
               Nj := Natural'Max (Nj, Natural (Sa.Q0.Length));
               --  每格画面挪画幅宽的 1/5(比例,无量纲;每个方向停 3 格,转开的总量同原来 5 格 × 1/10:V1B14 挑格回放,停 3 格最大 0.73 mm)
               Gw := Long_Float (Sa.W) / 5.0;
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
                        --  起步 = 读数量级的 3%(比例,无量纲),按画面挪动放大。目标里别的关节都回起点:上一段转完不单独回起点,
                        --  回去和这一段的头一格是同一个动作(到没到按这一格的三分之一判,所有关节一起看)
                        Sa.Step := 0.03 * Long_Float'Max (1.0, abs Sa.Q0 (J));
                        Sa.Off := 0.0; Sa.K := 0; Sa.Tgt := Sa.Q0; Sa.Q_Prev := F.Joints (Sa.G) (J); Sa.Px_Per := 0.0;
                        Sa.Why := To_Unbounded_String ("走满 3 格");
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
                     Move_All (Tol, Swept => Integer (J));
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
                                 if Sweep_Stops (Got, Pushed, Sa.Step) then
                                    --  没转到命令的三分之一(比例):到头;别的关节被顶偏超过这一格的三分之一(比例,同上):碰上东西了 —— 都不再往里压。
                                    --  只有"这个关节自己停住、别的关节没被顶偏"才是它这一边的界(反解不过这儿)。碰上东西了 = 手压在桌子 / 东西上,
                                    --  换个姿势这个关节照样转得过去,不记界(owner 09-28 到过的范围:"被桌子挡住这种'转不过去'本来就不是关节尽头";
                                    --  H4:人形第 0、3 关节往正碰桌记成了界,第二只手手指朝下再往前伸 15 cm 按这两道界解不出来,按仿真的真尽头解得出)
                                    Sa.Why := To_Unbounded_String (if not Sweep_Stop_Is_End (Got, Pushed, Sa.Step)
                                                                   then "碰上东西了:第" & Codec.Img (Kp) & " 个关节被顶偏 " & Codec.Fmt (Pushed, 4) & "(这一格命令 " & Codec.Fmt (Sa.Step, 4) & ";不是关节到头,不记界)"
                                                                   else "关节到头(命令 " & Codec.Fmt (Sa.Step, 4) & ",实到 " & Codec.Fmt (Got, 4) & ";这一边记界)");
                                    Sa.Done := True;
                                    if Sweep_Stop_Is_End (Got, Pushed, Sa.Step) and then J < Natural (Ds (A).Has_Lo.Length) then
                                       if Dd < 0 then
                                          Ds (A).Has_Lo.Replace_Element (J, True);
                                       else
                                          Ds (A).Has_Hi.Replace_Element (J, True);
                                       end if;
                                    end if;
                                 elsif Sa.K >= 3 then   --  最多 3 格(次数;5 分钟一炮)
                                    Sa.Done := True;
                                 else
                                    if Sa.Px_Per > 0.0 then
                                       --  下一格按这一格画面挪的(头一格那一对配点量的"每个读数单位挪几像素" × 这一格的步子)放大 / 缩小,
                                       --  一次最多四倍、最少减半(倍数,无量纲);没量到就不改
                                       Sa.Step := Sa.Step * Long_Float'Max (0.5, Long_Float'Min (Ramp, Gw / (Sa.Step * Sa.Px_Per)));
                                    end if;
                                    Sa.Q_Prev := F.Joints (Sa.G) (J);
                                 end if;
                                 if Sa.Done then
                                    Say ("  第" & Codec.Img (A + 1) & " 只手第" & Codec.Img (J) & " 个关节往" & (if Dd > 0 then "正" else "负") & "转了 " & Codec.Img (Sa.K)
                                         & " 格(累计 " & Codec.Fmt (Sa.Off, 3) & ")⇒ 停:" & To_String (Sa.Why));
                                    --  这一边最后一格命令的步子 = 往到过的范围外最多走的那一步(到过的范围:身体开机时一条命令走过的量)
                                    if J < Natural (Ds (A).Step_Lo.Length) then
                                       if Dd < 0 then
                                          Ds (A).Step_Lo.Replace_Element (J, Sa.Step);
                                       else
                                          Ds (A).Step_Hi.Replace_Element (J, Sa.Step);
                                       end if;
                                    end if;
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                  end;
               end loop;
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
                     Jobs.Put ((A => A, I => H1.Frame, J => H2.Frame, Ia => Natural (St (A).Ids (H1.Frame)), Ib => Natural (St (A).Ids (H2.Frame)),
                                W => St (A).W, H => St (A).H, Serial => N_Jobs));
                     N_Jobs := N_Jobs + 1;
                  end if;
               end loop;
            end loop;
         end if;
      end loop;
      declare
         T1 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         Say ("关节扫描走完:" & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (T1, T0)), 0) & " 秒(其中当场配 " & Codec.Img (N_Now)
              & " 对 " & Codec.Fmt (Long_Float (T_Now), 1) & " 秒);配点 " & Codec.Img (N_Jobs) & " 对在后台配,等它配完");
         Jobs.Close;
         while not Matcher'Terminated loop
            delay 0.2;   --  等后台配完(秒,协议:只是轮询间隔)
         end loop;
         Say ("  配点配完:再等了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T1)), 0) & " 秒(" & Codec.Img (N_Empty) & " 对配不上)");
      end;
      --  ③ 配点进运动学,这里只去掉落在画面外的点。长在眼上的像素(自己的手、夹爪)由 Kinem.Fit 按这批配点自己认(Eye_Pixels:
      --  两个以上关节各自单独转时各有一格没挪的格点),不再另跟一遍:09-27 以前按跟点仪器认手指,扫描时每格跟一次(V1B14 约 40 秒),
      --  x5 上遮了反而更差(最大 0.41 / 0.62 mm 对不遮 0.30 / 0.43 mm)就撤了 —— x5 的手指只占画面边上一小块,解法自己当野点去掉;
      --  09-28 人形 H2 腕眼三分之一是自己的手,解法去不掉,两只手错 16–20 mm ⇒ 按配点认、不多花时间(LAB H2)
      for A in 0 .. Na - 1 loop
         if St (A).Live then
            declare
               D : Sweep_Data := Ds (A);
               W : constant Long_Float := Long_Float (D.W);
               Hh : constant Long_Float := Long_Float (D.H);
               Kept_C : Kinem.Corr_Vectors.Vector;
               function Inside (U, V : Long_Float) return Boolean is (U >= 0.0 and then U < W and then V >= 0.0 and then V < Hh);
            begin
               for C of Res (A) loop
                  if Inside (C.Ua, C.Va) and then Inside (C.Ub, C.Vb) then
                     Kept_C.Append (C);
                  end if;
               end loop;
               if Dump /= "" then
                  --  配点落盘(corrs_arm<k>.txt:每行 I J Ua Va Ub Vb 轨迹号,帧号同 sweep.txt 里这只手的格子顺序):离线回放解法用
                  declare
                     Fo : Ada.Text_IO.File_Type;
                  begin
                     Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/corrs_arm" & Codec.Img (A) & ".txt");
                     for C of Kept_C loop
                        Ada.Text_IO.Put_Line (Fo, Codec.Img (C.I) & " " & Codec.Img (C.J) & " " & Codec.Fmt (C.Ua, 3) & " " & Codec.Fmt (C.Va, 3) & " "
                                              & Codec.Fmt (C.Ub, 3) & " " & Codec.Fmt (C.Vb, 3) & " " & Integer'Image (C.Pt));
                     end loop;
                     Ada.Text_IO.Close (Fo);
                  exception
                     when others => null;
                  end;
               end if;
               Say ("  第" & Codec.Img (A + 1) & " 只手:扫了 " & Codec.Img (Natural (D.Frames.Length)) & " 格;配点 " & Codec.Img (Natural (Kept_C.Length)) & " / "
                    & Codec.Img (Natural (Res (A).Length)) & " 个留下(落在画面外的不要)");
               D.Ids := St (A).Ids;
               Ds.Replace_Element (A, D);
               Css.Replace_Element (A, Kept_C);
            end;
         end if;
      end loop;
      Say ("关节扫描完:" & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 0) & " 秒");
   end Sweep_All;

   --  相机系里的单位视线(同 Geom 的约定:-z 朝前、+y 朝上)



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
            declare
               function Px (X : Long_Float) return String is (if X < 0.0 then "试不了" else Codec.Fmt (X, 3));
               Sl : constant Boolean := J < Natural (Rep.Slide.Length) and then Rep.Slide (J);
            begin
               Append (T, " " & (if Rep.Joint_Med (J) < 0.0 then "量不了" elsif Sl then "走 " else "转 ") & Px (Rep.Joint_Med (J))
                       & (if J < Natural (Rep.Joint_Med_Turn.Length) then "[" & (if Sl then "按转 " & Px (Rep.Joint_Med_Turn (J)) else "按走 " & Px (Rep.Joint_Med_Slide (J))) & "]" else "")
                       & "(" & Codec.Img (Rep.Joint_Frames (J)) & " 格)");
            end;
         end loop;
         Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:" & (if Ok then "量成" else "没量成") & " · 长在眼上的像素 " & Codec.Img (Rep.Eye_Px)
              & " 个(两个以上关节单独转时各有一格没挪 = 自己身上的),从它们出发的配点 " & Codec.Img (Rep.Eye_Corrs) & " / " & Codec.Img (Rep.N_Corr)
              & " 笔不进解 · 每根轴单独(转 / 走两样各解一次,残差小的那样)的残差中位(像素):" & To_String (T));
         T := Null_Unbounded_String;
         for X of Rep.Rho loop
            Append (T, " " & Codec.Fmt (X, 3));
         end loop;
         Append (T, " · 定比例用了 " & Codec.Img (Rep.Rho_Pairs) & " 对(三对起步 " & Codec.Fmt (Rep.Rho_Start_Px, 3) & " px → 全部重解中位 " & Codec.Fmt (Rep.Rho_Px, 3) & " px)");
         Append (T, " · 各步秒数");
         for X of Rep.Secs loop
            Append (T, " " & Codec.Fmt (X, 1));
         end loop;
         Say ("    焦距 起步 " & Codec.Fmt (Rep.F_Start, 1) & " → " & Codec.Fmt (Rep.F_Axes, 1) & " → " & Codec.Fmt (Rep.F, 1) & " · 一起解的残差中位 " & Codec.Fmt (Rep.Med_Px, 3) & " px、九成 "
              & Codec.Fmt (Rep.P90_Px, 3) & " px · 内点 " & Codec.Img (Rep.N_Used) & " / " & Codec.Img (Rep.N_Corr) & " · 各轴远近比例(以第"
              & Codec.Img (Rep.Ref_Joint) & " 根为 1):" & To_String (T) & (if Rep.Flipped then " · 平移反过一次号" else ""));
         Say ("    多视图一起解(起点那格的格点配进各格 = 轨迹,按重投影):" & Codec.Img (Rep.Mv_Tracks) & " 条轨迹 " & Codec.Img (Rep.Mv_Obs) & " 笔 · 重投影中位 "
              & Codec.Fmt (Rep.Mv_Start_Px, 3) & " → " & Codec.Fmt (Rep.Mv_Px, 3) & " px、九成 " & Codec.Fmt (Rep.Mv_P90_Px, 3) & " px · " & Codec.Img (Rep.Mv_Iters) & " 轮 · 焦距 "
              & Codec.Fmt (Rep.F, 1));
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
                                     & Codec.Fmt (M.Ax (J).P (2), 9) & " " & Kind_Word (M.Ax (J)));
            end loop;
            for P of M.Eye loop   --  长在眼上的像素(离线回放 alignexam 读回,三角时同样不用)
               Ada.Text_IO.Put_Line (Fo, "eye " & Codec.Fmt (P.U, 3) & " " & Codec.Fmt (P.V, 3));
            end loop;
            Ada.Text_IO.Close (Fo);
         exception
            when others => null;
         end;
      end if;
   end Fit_Arm;

   --  这只手扫描里三角出来的点(参照眼系,模型单位):起点那一格上的格点配进很多格(轨迹),按运动学在每一格里的像素一起解它的远近(多视图三角,
   --  Kinem.Track_Points;09-27 V1B32 改:原来按"起点 ↔ 某一格"一对三角,每一对自己的位姿误差让整片点成块偏 1–3.6 mm,不动的眼按它们定位偏 4 mm)。
   --  只收:至少 3 格看见(起点 + 另外两格)、重投影残差中位 < 3 px(协议)、配点差 1 像素时远近误差不到眼到它距离的二十分之一(比例,同原来"两条视线夹角 ≥ 20 / 焦距")。
   --  协方差:垂直视线 r·σ/f,沿视线 = 那一维解的方差(σ = 这些轨迹重投影残差的中位 × 1.4826)
   type Tri_Pt is record
      X : Geom.V3 := [0.0, 0.0, 0.0];
      Cov : Geom.M3 := [others => [others => 0.0]];
      Fr : Natural := 0;                    --  看见它的格子里离起点那格的眼最远的一格(对齐拿它当"世界里的一只眼")
      U0, V0 : Long_Float := 0.0;           --  在起点那格里的像素
   end record;
   package Tri_Vectors is new Ada.Containers.Vectors (Natural, Tri_Pt);
   procedure Tri_Pts (M : Kinem.Model; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Max_Pts : Natural; P : out Tri_Vectors.Vector; Sig_Px : out Long_Float) is
      use Geom;
      Tp : Kinem.Track_Pt_Vectors.Vector;
   begin
      P.Clear; Sig_Px := 0.0;
      Kinem.Track_Points (M, D.Frames, Cs, Only_I => 0, Min_Views => 3, Tracks => Tp, Sig_Px => Sig_Px);   --  起点那一格出发、至少 3 格(次数)
      if Sig_Px <= 0.0 then
         return;
      end if;
      for T of Tp loop
         exit when Natural (P.Length) >= Max_Pts;
         declare
            X : constant V3 := T.X;
            Rr : constant Long_Float := Norm (X);
            Sd : constant Long_Float := Sqrt (T.Var_Along);   --  沿视线(模型单位)
         begin
            if T.Med_Px < 3.0 and then Rr > 0.0 and then Sd / Sig_Px < 0.05 * Rr and then X (2) < 0.0 then   --  3 px(协议);二十分之一(比例,见上);在参照眼前面(-z 朝前)
               declare
                  Dd : constant V3 := [X (0) / Rr, X (1) / Rr, X (2) / Rr];
                  Sp : constant Long_Float := Rr * Sig_Px / M.F;   --  垂直视线
                  Cv : M3;
               begin
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Cv (I, J) := Sp * Sp * ((if I = J then 1.0 else 0.0) - Dd (I) * Dd (J)) + Sd * Sd * Dd (I) * Dd (J);
                     end loop;
                  end loop;
                  P.Append (Tri_Pt'(X => X, Cov => Cv, Fr => T.Far, U0 => T.U, V0 => T.V));
               end;
            end if;
         end;
      end loop;
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
   Trip_Px : constant := Geom.Trip_Px;   --  往返 1 px 内的才算(绝对门 —— 按中位数倍数定的门在乱配占多数时跟着放宽,V1B11)
   Min_Inl : constant := 10;    --  至少 10 个内点才放进世界(次数)

   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                    Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "";
                    Pin_Fixed_F : Long_Float := 0.0) is
      use Geom;
      Max_Pts : constant := Gx * Gy;   --  每只手最多三角几个点(同扫描格点数,次数)
      T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      Na : constant Natural := Natural (Worlds.Length);
      Ps : array (0 .. Natural'Max (1, Na) - 1) of Tri_Vectors.Vector;
      Pls, Nrs : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Gates : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
      Sig_Px : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];   --  每只手自己配点的噪声(像素,Tri_Pts 量的;给它的三角点定不确定度)
      Placed : array (0 .. Natural'Max (1, Na) - 1) of Boolean := [others => False];
      Pl0, N0 : V3 := [0.0, 0.0, 1.0];  --  世界的桌面(第一只手系里)
      Pl0_Md : Long_Float := 0.0;       --  第一只手三角出、拟合进桌面的点离面的中位
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
      --  一只手和世界里的眼之间那批配点的噪声 = 它们往返差的中位(像素;同不动的眼那边)—— 不用它自己扫描配点的噪声:
      --  两只手之间视角差得远,配点真的误差是 1.5–3 px,自己扫描的只有 0.1 px(V1B12 回放:按 0.1 px 算,网格分不出好坏,起步落错)
      function Cross_Sig (B : Natural) return Long_Float is
         package Sorting is new F64_Vectors.Generic_Sorting;
         Es : Floats;
      begin
         for X of Arm_Obs (B) loop
            Es.Append (X.E);
         end loop;
         if Es.Is_Empty then
            return 1.0;
         end if;
         Sorting.Sort (Es);
         return Long_Float'Max (Es (Natural (Es.Length) / 2), 1.0e-6);   --  数值保护(无量纲)
      end Cross_Sig;
      N_Pairs_Of : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
      --  这一轮给每只候选的手算出来的放法(真放进世界时直接用,证据没变,不再算一遍)
      Cand_S, Cand_Md : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];
      Cand_R : array (0 .. Natural'Max (1, Na) - 1) of M3 := [others => Identity];
      Cand_T : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 0.0]];
      Cand_Inl : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
      N_Pairs_Fx : Natural := 0;
      Round_Note : Unbounded_String;
      --  第 A 只手第 F 格里看得见的三角点:下标和像素 = 这只手的全部三角点按运动学投进这一格,在眼前面、落在画面里的
      --  (09-27 V1B15:原来只给"在这一格三角出来的"那 ≤ 40 个,第 2 只手 22 格去配头顶眼只有 1 格配上 18 个点,两只手对齐差 11 mm;
      --  离线拿整片 768 个格点去问,40 / 44 格每格往返 1 px 内配上 100–250 个 —— 缺的是问的点,不是配不上)
      package M3_Vectors is new Ada.Containers.Vectors (Natural, M3);
      Fk_R : array (0 .. Natural'Max (1, Na) - 1) of M3_Vectors.Vector;   --  每只手每一格的腕眼位姿(参照眼系里;按读数算一次存着)
      Fk_T : array (0 .. Natural'Max (1, Na) - 1) of V3_Vectors.Vector;
      procedure Pts_In (A, F : Natural; Idx : out Ints; Q : out Instrument.Match_Vectors.Vector) is
         M : Kinem.Model renames Worlds (A).Model;
      begin
         Idx.Clear; Q.Clear;
         if Fk_R (A).Is_Empty then
            for Fr of Ds (A).Frames loop
               declare
                  Rr : M3;
                  Tt : V3;
               begin
                  Kinem.FK (M, Fr.Q, Rr, Tt);
                  Fk_R (A).Append (Rr); Fk_T (A).Append (Tt);
               end;
            end loop;
         end if;
         if F >= Natural (Fk_R (A).Length) then
            return;
         end if;
         for K in 0 .. Natural (Ps (A).Length) - 1 loop
            declare
               X : constant V3 := Ps (A) (K).X;
               Xc : constant V3 := Ap (Tr (Fk_R (A) (F)), [X (0) - Fk_T (A) (F) (0), X (1) - Fk_T (A) (F) (1), X (2) - Fk_T (A) (F) (2)]);
            begin
               if Xc (2) < 0.0 then   --  在眼前面(-z 朝前,同 Dir_Of)
                  declare
                     U : constant Long_Float := M.Cx + M.F * Xc (0) / (-Xc (2));
                     V : constant Long_Float := M.Cy - M.F * Xc (1) / (-Xc (2));
                  begin
                     if U >= 0.0 and then U < Long_Float (Ds (A).W) and then V >= 0.0 and then V < Long_Float (Ds (A).H) then
                        Idx.Append (K); Q.Append (Instrument.Match_Pt'(U => U, V => V, others => <>));
                     end if;
                  end;
               end if;
            end;
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
      T_Match : Duration := 0.0;   --  对齐里花在配点仪器上的时间(秒;只记账)
      N_Match : Natural := 0;
      --  Dst_Arm ≥ 0:Dst 是这只手某一格的腕眼 ⇒ 落在它长在眼上的像素那一格的不要(被自己的手挡着,不是这个点;Kinem.On_Eye_Grid)
      procedure Match_Pair (Src, Dst : Integer; Dw, Dh : Natural; Q : Instrument.Match_Vectors.Vector; Keep : out Ints; U, V, E : out Floats;
                            Dst_Arm : Integer := -1) is
         Err : Unbounded_String;
         Tm : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         Keep.Clear; U.Clear; V.Clear; E.Clear;
         Tried.Append (Pair'(A => Src, B => Dst));
         N_Match := N_Match + 1;
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
                        if Ei < Trip_Px and then not (Dst_Arm >= 0 and then Dw > 0 and then Dh > 0
                                                      and then Kinem.On_Eye_Grid (Worlds (Natural (Dst_Arm)).Model.Eye, R (I).U, R (I).V, Dw, Dh, Gx, Gy))
                        then
                           Keep.Append (I); U.Append (R (I).U); V.Append (R (I).V); E.Append (Ei);
                        end if;
                     end;
                  end if;
               end loop;
            end if;
         end;
         T_Match := T_Match + Ada.Calendar."-" (Ada.Calendar.Clock, Tm);
      end Match_Pair;
      function Was_Tried (A, B : Integer) return Boolean is (Tried.Contains (Pair'(A => A, B => B)));
      --  到这一步用了多少秒、其中配点仪器多少(只记账)
      function Lap return String is
        ("(到这儿 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒,其中配点 " & Codec.Img (N_Match) & " 对 "
         & Codec.Fmt (Long_Float (T_Match), 1) & " 秒)");
      function Dist (X, Pl, Nrm : V3) return Long_Float is ((X (0) - Pl (0)) * Nrm (0) + (X (1) - Pl (1)) * Nrm (1) + (X (2) - Pl (2)) * Nrm (2));
      function Cross (A, B : V3) return V3 is ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
      --  一个点投进世界里一只眼的像素残差,按"配点噪声 ⊕ 这个点自己三角的不确定度投进这只眼"白化(2 条,以标准差为单位;同 Geom.Scene_Var 的做法):
      --  三角的远近误差从侧面看是几像素(V1B11:第 2 只手的点投进 60 cm 外第 1 只手的眼,不加权时按真值对齐也只有 54% 在 3 px 内),加权以后按它实际的精度算;
      --  不用两只眼之间的对极误差 —— 它只定得住两只眼往哪个方向隔开、定不住隔开多远(V1B12:朝向一样、横着隔开的那几对下,平移错 431 mm)
      procedure Proj_W (Cg : Cam_Geo; Xw : V3; Cw : M3; U, V, Sig_M : Long_Float; E1, E2 : out Long_Float) is
         Rt : constant M3 := Tr (Cg.R_Ce);
         Pc : constant V3 := Ap (Rt, [Xw (0) - Cg.Pos (0), Xw (1) - Cg.Pos (1), Xw (2) - Cg.Pos (2)]);
         Z : constant Long_Float := -Pc (2);
      begin
         if Z <= 0.0 or else Cg.F <= 0.0 then
            E1 := 1.0e3; E2 := 1.0e3;   --  在那只眼后面:远大于任何一条白化残差的罚(无量纲哨兵)
            return;
         end if;
         declare
            Du : constant V3 := [Cg.F / Z, 0.0, Cg.F * Pc (0) / (Z * Z)];
            Dv : constant V3 := [0.0, -Cg.F / Z, -Cg.F * Pc (1) / (Z * Z)];
            Ju, Jv : V3 := [0.0, 0.0, 0.0];
            A11, A12, A22 : Long_Float;
            Eu : constant Long_Float := Cg.F * Pc (0) / Z + Cg.Cx - U;
            Ev : constant Long_Float := -Cg.F * Pc (1) / Z + Cg.Cy - V;
            function Q (X, Y : V3) return Long_Float is
               Sm : Long_Float := 0.0;
            begin
               for I in 0 .. 2 loop
                  for K in 0 .. 2 loop
                     Sm := Sm + X (I) * Cw (I, K) * Y (K);
                  end loop;
               end loop;
               return Sm;
            end Q;
         begin
            for K in 0 .. 2 loop
               Ju (K) := Du (0) * Rt (0, K) + Du (1) * Rt (1, K) + Du (2) * Rt (2, K);
               Jv (K) := Dv (0) * Rt (0, K) + Dv (1) * Rt (1, K) + Dv (2) * Rt (2, K);
            end loop;
            A11 := Sig_M * Sig_M + Q (Ju, Ju); A12 := Q (Ju, Jv); A22 := Sig_M * Sig_M + Q (Jv, Jv);
            declare
               L11 : constant Long_Float := Sqrt (Long_Float'Max (A11, 1.0e-18));   --  数值保护(无量纲)
               L21 : constant Long_Float := A12 / L11;
               L22 : constant Long_Float := Sqrt (Long_Float'Max (A22 - L21 * L21, 1.0e-18));   --  同上
            begin
               E1 := Eu / L11;
               E2 := (Ev - L21 * E1) / L22;
            end;
         end;
      end Proj_W;      procedure Plane_Of (P : Tri_Vectors.Vector; Pl, Nrm : out V3; Gate : out Long_Float; Inl : out Natural; Md : out Long_Float) is
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
         Gt.Cx := Long_Float (Ds (0).World_Img.W) / 2.0; Gt.Cy := Long_Float (Ds (0).World_Img.H) / 2.0; Gt.F := Pin_Fixed_F;   --  焦距一起解(Pin_Fixed_F = 0;离线对照实验才钉)
         Fit_Fixed_Board (Gt, Scene, Rp, Okf);
         Inl := (if Okf then Rp.Scene_Used else 0);
      end Fit_Cam;
      --  每只手的桌面当一个约束用:法向(朝它的眼)、面上点的中心;不确定度 = 两张面各自拟合出来的标准误差按平方和 ——
      --  离面的散布(1.4826 × 中位,统计常数)÷ √面上的点数:高度直接是它,倾角再除以面铺开的大小(V1B12 回放:拿"算不算在面上的门"当不确定度,
      --  那是桌面的厚度不是它位置的精度,大了几十倍,长度倍数差 2.9%)。只算 3 条残差(两个倾角 + 一个高度)—— 不按桌面上每个点各算一条:
      --  那样几百条压过几十条像素残差,两只手各自量的桌面之间本来就有几毫米的差,解会被拽到"桌面对得最齐"的错解上
      --  (V1B11 回放 2026-09-26:按点算时倍数 0.98、转错 12°、平移错 583 mm)
      Pl_Cb, Pl_Nb : array (0 .. Natural'Max (1, Na) - 1) of V3 := [others => [0.0, 0.0, 1.0]];
      Pl_Sn, Pl_Sd : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];   --  倾角(弧度)、高度(世界单位)的不确定度
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
         declare
            --  两张面各自的高度标准误差(它自己的单位):离面散布 ÷ √点数
            Se_B : constant Long_Float := Gates (B) / 2.5 / Sqrt (Long_Float (Natural'Max (1, N_B)));   --  Gates = 2.5 × 1.4826 × 中位 ⇒ ÷ 2.5 = 离面散布(统计常数,无量纲)
            Se_0 : constant Long_Float := Gates (0) / 2.5 / Sqrt (Long_Float (Natural'Max (1, N_0)));
         begin
            Pl_Sn (B) := Sqrt ((Se_B / Long_Float'Max (Ext_B, 1.0e-12)) ** 2 + (Se_0 / Long_Float'Max (Ext_0, 1.0e-12)) ** 2);   --  数值保护(无量纲)
            Pl_Sd (B) := Sqrt (Se_0 ** 2 + Se_B ** 2);   --  第 B 只手那份按它自己的单位,和世界单位差一个倍数(约 1,放进世界之前不知道)
         end;
      end Plane_Info;
      procedure Plane_Res (B : Natural; Sx : Long_Float; Rx : M3; Tx : V3; R1, R2, R3 : out Long_Float) is
         Nw : constant V3 := Ap (Rx, Pl_Nb (B));
         Tilt : constant V3 := Cross (Nw, N0);
         E1, E2 : V3;
         Rt : constant V3 := Ap (Rx, Pl_Cb (B));
         Sig_P : constant Long_Float := Long_Float'Max (1.0e-12, Pl_Sd (B));   --  数值保护(无量纲)
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
      Coarse_T : constant := 5;    --  网格先粗找:转角 5 档并 1 档(5°)、倍数 2 档并 1 档(约 26%),再在最好那格周围按细档补(次数)
      Coarse_S : constant := 2;
      procedure Place_Arm (B : Natural; S : out Long_Float; R : out M3; T : out V3; Inl : out Natural; Md : out Long_Float) is
         Nr : constant Natural := Natural (Arm_Obs (B).Length);
         Sig_B : constant Long_Float := Cross_Sig (B);
         --  两组配点各自的噪声倍数(方差分量:按这一组自己的残差估,见 Group_Scale):0 = 配进手的格子,1 = 配进不长在手上的眼
         Fac : array (0 .. 1) of Long_Float := [1.0, 1.0];
         function Grp (Ob : Arm_Ob) return Natural is (if Wv (Ob.Wk).Arm < 0 then 1 else 0);
         R0 : M3 := Identity;
         function Rz (Th : Long_Float) return M3 is (Rodrigues ([N0 (0) * Th, N0 (1) * Th, N0 (2) * Th]));
         --  放进世界只是给一起精修找起点 ⇒ 网格的分数和这里的精修每组按顺序等间隔最多取 Sub_Max 条(次数);每一格的平移用全部
         --  (平移按普通最小二乘解,子集里错配那组占的份量变大,09-27 V1B15 回放起点从倍数 1.0060 偏到 1.0394);数门里有几条用全部。
         --  一起精修(Joint_Refine)用全部
         Sub_Max : constant := 400;
         In_Sub : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Nr));
         --  给定倍数、转动 ⇒ 平移:沿世界法向那一分量由"它的桌面落在世界的桌面上"定死(桌面是几百个点拟合的,高度的标准误差很小;
         --  精修时再按标准误差放软,见 Plane_Res),平面里那两个分量按视线的线性最小二乘(点到视线的距离)。
         --  (09-27 V1B15 回放:配进头顶眼的点来自同一格、大多在桌面上,只按视线解时"放大再挪远"投回头顶眼一样,网格挑到倍数 1.2952)
         function Solve_T (Sx : Long_Float; Rx : M3) return V3 is
            A : M3 := [others => [others => 0.0]];
            Bb : V3 := [0.0, 0.0, 0.0];
            Rc : constant V3 := Ap (Rx, Pl_Cb (B));
            H : constant Long_Float := Pl0 (0) * N0 (0) + Pl0 (1) * N0 (1) + Pl0 (2) * N0 (2) - Sx * (Rc (0) * N0 (0) + Rc (1) * N0 (1) + Rc (2) * N0 (2));
            E1 : V3 := Cross (N0, (if abs N0 (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  垂直于世界法向的一对方向(纯数学,无量纲)
            E2 : V3;
         begin
            E1 := [E1 (0) / Norm (E1), E1 (1) / Norm (E1), E1 (2) / Norm (E1)];
            E2 := Cross (N0, E1);
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
            --  T = H · 法向 + a · E1 + b · E2:(a, b) 的 2×2 正规方程
            declare
               An : constant V3 := Ap (A, N0);
               Ae1 : constant V3 := Ap (A, E1);
               Ae2 : constant V3 := Ap (A, E2);
               R1 : constant Long_Float := (Bb (0) - H * An (0)) * E1 (0) + (Bb (1) - H * An (1)) * E1 (1) + (Bb (2) - H * An (2)) * E1 (2);
               R2 : constant Long_Float := (Bb (0) - H * An (0)) * E2 (0) + (Bb (1) - H * An (1)) * E2 (1) + (Bb (2) - H * An (2)) * E2 (2);
               M11 : constant Long_Float := E1 (0) * Ae1 (0) + E1 (1) * Ae1 (1) + E1 (2) * Ae1 (2);
               M12 : constant Long_Float := E1 (0) * Ae2 (0) + E1 (1) * Ae2 (1) + E1 (2) * Ae2 (2);
               M22 : constant Long_Float := E2 (0) * Ae2 (0) + E2 (1) * Ae2 (1) + E2 (2) * Ae2 (2);
               Dt : constant Long_Float := M11 * M22 - M12 * M12;
               Ya, Yb : Long_Float := 0.0;
            begin
               if abs Dt > 1.0e-18 then   --  数值保护(无量纲)
                  Ya := (M22 * R1 - M12 * R2) / Dt; Yb := (M11 * R2 - M12 * R1) / Dt;
               end if;
               return [H * N0 (0) + Ya * E1 (0) + Yb * E2 (0), H * N0 (1) + Ya * E1 (1) + Yb * E2 (1), H * N0 (2) + Ya * E1 (2) + Yb * E2 (2)];
            end;
         end Solve_T;
         --  一条配点:它的点按(Sx, Rx, Tx)放进世界、投回世界里那只眼,白化残差两条
         procedure Res2 (Ob : Arm_Ob; Cg : Cam_Geo; Sx : Long_Float; Rx : M3; Tx : V3; E1, E2 : out Long_Float) is
            Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
            Xw : constant V3 := [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)];
            Rc : constant M3 := Mul (Mul (Rx, Ps (B) (Ob.K).Cov), Tr (Rx));
            Cw : M3;
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Cw (I, J) := Sx * Sx * Rc (I, J);
               end loop;
            end loop;
            Proj_W (Cg, Xw, Cw, Ob.U, Ob.V, Sig_B, E1, E2);
            E1 := E1 / Fac (Grp (Ob)); E2 := E2 / Fac (Grp (Ob));
         end Res2;
         function Epi (Ob : Arm_Ob; Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
            E1, E2 : Long_Float;
         begin
            Res2 (Ob, Wv (Ob.Wk).Cam, Sx, Rx, Tx, E1, E2);
            return Sqrt (E1 * E1 + E2 * E2);
         end Epi;
         --  网格那一步的分数 = 每组(条数 × 这一组白化残差中位的对数)加起来 = 每组噪声各自不知道时的最大似然(同精修里按组重估噪声倍数:
         --  一组的噪声倍数按它自己的中位估,代回去似然只剩 −Σ 条数 · log 中位)。一组整体大小(噪声估大估小)只差一个常数,不影响排名;
         --  原来直接把各组中位相加(09-27 V1B15 回放:配进第 1 只手格子的 331 条 84% 是木纹上的错配,中位几百像素,随网格变一点就压过
         --  配进头顶眼那 192 条 0.4 px 的信号,放进世界时倍数解成 1.2957,靠一起精修才拉回 1.0066)
         function Score (Sx : Long_Float; Rx : M3; Tx : V3) return Long_Float is
            package Sorting is new F64_Vectors.Generic_Sorting;
            Es : array (0 .. 1) of Floats;
            Sm : Long_Float := 0.0;
         begin
            for K in 0 .. Nr - 1 loop
               if In_Sub (K) then
                  Es (Grp (Arm_Obs (B) (K))).Append (Epi (Arm_Obs (B) (K), Sx, Rx, Tx));
               end if;
            end loop;
            for G in Es'Range loop
               if not Es (G).Is_Empty then
                  Sorting.Sort (Es (G));
                  Sm := Sm + Long_Float (Es (G).Length) * Log (Long_Float'Max (Es (G) (Natural (Es (G).Length) / 2), 1.0e-9));   --  数值保护(无量纲)
               end if;
            end loop;
            return Sm;
         end Score;
         --  按这一组自己的残差把它的噪声倍数重估一遍:白化后二维残差的长度,单位方差时中位 = √(2 ln 2) ≈ 1.1774(瑞利分布的中位,统计常数,无量纲)
         procedure Group_Scale is
            package Sorting is new F64_Vectors.Generic_Sorting;
            Es : array (0 .. 1) of Floats;
         begin
            Fac := [1.0, 1.0];
            for Ob of Arm_Obs (B) loop
               Es (Grp (Ob)).Append (Epi (Ob, S, R, T));
            end loop;
            for G in Es'Range loop
               if not Es (G).Is_Empty then
                  Sorting.Sort (Es (G));
                  Fac (G) := Long_Float'Max (Es (G) (Natural (Es (G).Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
               end if;
            end loop;
         end Group_Scale;
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
            N_Grp, Seen : array (0 .. 1) of Natural := [others => 0];
         begin
            for Ob of Arm_Obs (B) loop
               N_Grp (Grp (Ob)) := N_Grp (Grp (Ob)) + 1;
            end loop;
            for K in 0 .. Nr - 1 loop
               declare
                  G : constant Natural := Grp (Arm_Obs (B) (K));
               begin
                  In_Sub.Replace_Element (K, Seen (G) mod Natural'Max (1, (N_Grp (G) + Sub_Max - 1) / Sub_Max) = 0);
                  Seen (G) := Seen (G) + 1;
               end;
            end loop;
         end;
         --  两级网格(09-27:一格要把几百条配点白化一遍,360 × 41 格放一次 8–20 秒):粗档挑出最好那格,再在它周围按细档补;
         --  最后落点的细度同原来(1°、约 12%)
         declare
            Best : Long_Float := Long_Float'Last;
            It_B, Ks_B : Integer := 0;
            procedure Try (It, Ks : Integer) is
               Itm : constant Integer := It mod N_Theta;
            begin
               if Ks < 0 or else Ks > N_Scale - 1 then
                  return;
               end if;
               declare
                  Rx : constant M3 := Mul (Rz (2.0 * Ada.Numerics.Pi * Long_Float (Itm) / Long_Float (N_Theta)), R0);
                  Sx : constant Long_Float := 0.1 * Exp (Long_Float (Ks) / Long_Float (N_Scale - 1) * Log (100.0));   --  0.1–10(比例,无量纲)
                  Tx : constant V3 := Solve_T (Sx, Rx);
                  Sc : constant Long_Float := Score (Sx, Rx, Tx);
               begin
                  if Sc < Best then
                     Best := Sc; S := Sx; R := Rx; T := Tx; It_B := Itm; Ks_B := Ks;
                  end if;
               end;
            end Try;
         begin
            for It in 0 .. N_Theta / Coarse_T - 1 loop
               for Ks in 0 .. (N_Scale - 1) / Coarse_S loop
                  Try (It * Coarse_T, Ks * Coarse_S);
               end loop;
            end loop;
            declare
               It_C : constant Integer := It_B;
               Ks_C : constant Integer := Ks_B;
            begin
               for Dt in -(Coarse_T - 1) .. Coarse_T - 1 loop
                  for Dk in -(Coarse_S - 1) .. Coarse_S - 1 loop
                     if Dt /= 0 or else Dk /= 0 then
                        Try (It_C + Dt, Ks_C + Dk);
                     end if;
                  end loop;
               end loop;
            end;
         end;
         --  一起精修三轮(每轮先按各组自己的残差重估噪声倍数,再挑门里的、解)
         for Round in 1 .. 3 loop
            Group_Scale;
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
                  if Es (K) < Gate and then In_Sub (K) then
                     Use_R.Replace_Element (K, True); Nu := Nu + 1;
                  end if;
               end loop;
               exit when Nu < Min_Inl;
               declare
                  N_Res : constant Natural := 2 * Nu + 3;
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
                           Res2 (Arm_Obs (B) (K), Wv (Arm_Obs (B) (K).Wk).Cam, Sx, Rx, Tx, Rr (J), Rr (J + 1)); J := J + 2;
                        end if;
                     end loop;
                     Plane_Res (B, Sx, Rx, Tx, Rr (J), Rr (J + 1), Rr (J + 2));
                  end Resid;
                  Lm_Done : Boolean;   --  LM 收住了没有(False = 做满 100 次还在降)
               begin
                  Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access, Lm_Done);
                  R := Rodrigues ([X (0), X (1), X (2)]); T := [X (3), X (4), X (5)]; S := Exp (X (6));
               end;
            end;
         end loop;
         declare
            All_E, Es : Floats;
            package Sorting is new F64_Vectors.Generic_Sorting;
            Gate : Long_Float;
         begin
            for Ob of Arm_Obs (B) loop
               All_E.Append (Epi (Ob, S, R, T));
            end loop;
            Gate := Gate_Of (All_E);
            for E of All_E loop
               if E < Gate then
                  Es.Append (E);
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
                  Cg.F := (if Pin_Fixed_F > 0.0 then Pin_Fixed_F else Exp (Xx (J + 6)));
               end;
            end if;
            return Cg;
         end Cam_Now;
         Fx_K : Integer := -1;   --  不动的眼在 Wv 里第几个
         Cam_Sig : Long_Float := 1.0e-6;   --  不动的眼里配点的噪声 = 这一批往返差的中位(像素;同解它时的 Sh)
         --  全部残差(门里的):手的配点(第 B 只手的点投回世界里那只眼)、不动的眼的配点(世界里的手的点投进它)、每只手桌面 3 条
         --  各组配点的噪声倍数(方差分量,按这一组自己的残差估):第 B 只手配进手的格子 = 2B,配进不长在手上的眼 = 2B + 1;不动的眼的配点 = 2 Na
         Jf : array (0 .. 2 * Na) of Long_Float := [others => 1.0];
         --  每只手配点的噪声(Cross_Sig:往返差的中位)在精修里不变 ⇒ 先算好(原来每条观测都把整批排一遍序,第 2 只手配点 1306 条时一起精修 274 秒)
         Sig_Of : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 1.0];
         procedure All_Res (Xx : Kinem.Vec; Rr : out Kinem.Vec; Use_A : Bools; Use_C : Bools; Fill_E : Boolean; Ea, Ec : in out Floats; Ga : in out Ints) is
            J : Natural := Rr'First;
         begin
            Ea.Clear; Ec.Clear; Ga.Clear;
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
                           E1, E2 : Long_Float;
                           Rt : constant V3 := Ap (Rx, Ps (B) (Ob.K).X);
                           Rc : constant M3 := Mul (Mul (Rx, Ps (B) (Ob.K).Cov), Tr (Rx));
                           Cw : M3;
                        begin
                           for I in 0 .. 2 loop
                              for Jj in 0 .. 2 loop
                                 Cw (I, Jj) := Sx * Sx * Rc (I, Jj);
                              end loop;
                           end loop;
                           Proj_W (Cam_Now (Xx, Ob.Wk), [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)], Cw, Ob.U, Ob.V, Sig_Of (B), E1, E2);
                           declare
                              Gi : constant Natural := 2 * B + (if Wv (Ob.Wk).Arm < 0 then 1 else 0);
                           begin
                              E1 := E1 / Jf (Gi); E2 := E2 / Jf (Gi);
                              if Fill_E then
                                 Ga.Append (Gi);
                              end if;
                           end;
                           if Fill_E then
                              Ea.Append (Sqrt (E1 * E1 + E2 * E2));
                           elsif Use_A (Natural (Ea.Length)) then
                              Rr (J) := E1; Rr (J + 1) := E2; J := J + 2;
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
                           Sx : Long_Float;
                           Rx : M3;
                           Tx : V3;
                           E1, E2 : Long_Float;
                        begin
                           Map_Of (Xx, X.Pa, Sx, Rx, Tx);
                           declare
                              Rt : constant V3 := Ap (Rx, Ps (X.Pa) (X.Pk).X);
                              Rc : constant M3 := Mul (Mul (Rx, Ps (X.Pa) (X.Pk).Cov), Tr (Rx));
                              Cw : M3;
                           begin
                              for I in 0 .. 2 loop
                                 for Jj in 0 .. 2 loop
                                    Cw (I, Jj) := Sx * Sx * Rc (I, Jj);
                                 end loop;
                              end loop;
                              Proj_W (Cg, [Sx * Rt (0) + Tx (0), Sx * Rt (1) + Tx (1), Sx * Rt (2) + Tx (2)], Cw, X.U, X.V, Cam_Sig, E1, E2);
                              E1 := E1 / Jf (2 * Na); E2 := E2 / Jf (2 * Na);
                           end;
                           if Fill_E then
                              Ec.Append (Sqrt (E1 * E1 + E2 * E2));
                           elsif Use_C (Natural (Ec.Length)) then
                              Rr (J) := E1; Rr (J + 1) := E2; J := J + 2;
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
            Sig_Of (B) := Cross_Sig (B);
         end loop;
         declare
            package Sorting is new F64_Vectors.Generic_Sorting;
            Es : Floats;
         begin
            for X of Cam_Obs loop
               Es.Append (X.E);
            end loop;
            if not Es.Is_Empty then
               Sorting.Sort (Es);
               Cam_Sig := Long_Float'Max (Es (Natural (Es.Length) / 2), 1.0e-6);   --  数值保护(无量纲)
            end if;
         end;
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
            for Round in 1 .. 3 loop   --  三轮:每轮先按各组自己的残差重估噪声倍数,再挑门里的、解(次数)
               declare
                  Ea, Ec : Floats;
                  Ga : Ints;
                  Dummy : Kinem.Vec (0 .. 0);
                  Ua, Uc : Bools;
                  Nua, Nuc, N_Planes : Natural := 0;
               begin
                  Jf := [others => 1.0];
                  All_Res (X, Dummy, Bool_Vectors.Empty_Vector, Bool_Vectors.Empty_Vector, True, Ea, Ec, Ga);
                  --  各组的噪声倍数 = 白化后二维残差长度的中位 ÷ √(2 ln 2)(单位方差时瑞利分布的中位,统计常数,无量纲)
                  declare
                     package Sorting is new F64_Vectors.Generic_Sorting;
                     Per : array (0 .. 2 * Na) of Floats;
                  begin
                     for K in 0 .. Natural (Ea.Length) - 1 loop
                        Per (Natural (Ga (K))).Append (Ea (K));
                     end loop;
                     for E of Ec loop
                        Per (2 * Na).Append (E);
                     end loop;
                     for G in Per'Range loop
                        if not Per (G).Is_Empty then
                           Sorting.Sort (Per (G));
                           Jf (G) := Long_Float'Max (Per (G) (Natural (Per (G).Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
                        end if;
                     end loop;
                     for K in 0 .. Natural (Ea.Length) - 1 loop
                        Ea.Replace_Element (K, Ea (K) / Jf (Natural (Ga (K))));
                     end loop;
                     for K in 0 .. Natural (Ec.Length) - 1 loop
                        Ec.Replace_Element (K, Ec (K) / Jf (2 * Na));
                     end loop;
                  end;
                  declare
                     Gta : constant Long_Float := Gate_Of (Ea);
                     Gtc : constant Long_Float := Gate_Of (Ec);
                  begin
                     for E of Ea loop
                        Ua.Append (E < Gta);
                        if E < Gta then
                           Nua := Nua + 1;
                        end if;
                     end loop;
                     for E of Ec loop
                        Uc.Append (E < Gtc);
                        if E < Gtc then
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
                     N_Res : constant Natural := 2 * Nua + 2 * Nuc + 3 * N_Planes;
                     procedure Resid (Xx : Kinem.Vec; Rr : out Kinem.Vec) is
                        E1, E2 : Floats;
                        G2 : Ints;
                     begin
                        All_Res (Xx, Rr, Ua, Uc, False, E1, E2, G2);
                     end Resid;
                     Lm_Done : Boolean;   --  LM 收住了没有(False = 做满 100 次还在降)
                  begin
                     if N_Res > Np then
                        Kinem.Robust_LM (X, N_Res, N_Res, 100, Steps, Resid'Access, Lm_Done);
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
                  G.R_Ce := Rodrigues ([X (K), X (K + 1), X (K + 2)]); G.Pos := [X (K + 3), X (K + 4), X (K + 5)];
                  G.F := (if Pin_Fixed_F > 0.0 then Pin_Fixed_F else Exp (X (K + 6)));
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
      Board.Clear; Plane_Pt := [0.0, 0.0, 0.0]; Plane_N := [0.0, 0.0, 1.0]; Plane_Rms := 0.0;
      if Worlds.Is_Empty or else not Worlds (0).Valid then
         return;
      end if;
      for A in 0 .. Na - 1 loop
         if Worlds (A).Valid then
            Tri_Pts (Worlds (A).Model, Ds (A), Css (A), Max_Pts, Ps (A), Sig_Px (A));
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
         Pl0_Md := Md;
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
                           Match_Pair (Id_Of (B, C.F), Vw.Id, Vw.W, Vw.H, Q, Keep, U, V, E, Dst_Arm => Vw.Arm);
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
                        Cand_S (B) := S; Cand_R (B) := R; Cand_T (B) := T; Cand_Inl (B) := Inl; Cand_Md (B) := Md;
                        Say ("  第" & Codec.Img (B + 1) & " 只手这一轮配了(它的第几格 → 世界里的哪只眼:问几个点 / 往返配上几个 / 其中在它桌面上的):" & To_String (Round_Note)
                             & " ⇒ 一共配上 " & Codec.Img (Natural (Arm_Obs (B).Length)) & " 次,放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次(残差中位 "
                             & Codec.Fmt (Md, 2) & " 倍标准差)· 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
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
                  --  已经放进世界的别的手,也和这只新放进来的眼配一次(用它最像这只眼的一格;先后不同,证据不能不同)
                  for P in 1 .. Na - 1 loop
                     if Placed (P) then
                        declare
                           Bf : Integer := -1;
                           Bs : Long_Float := Long_Float'First;
                           Idx, Keep : Ints;
                           Q : Instrument.Match_Vectors.Vector;
                           U, V, E : Floats;
                           K_New : constant Natural := Natural (Wv.Length) - 1;
                        begin
                           for F of Frames_Of (P) loop
                              if Id_Of (P, Natural (F)) >= 0 and then Desc_Dot (Id_Of (P, Natural (F)), Fx_Id) > Bs then
                                 Bs := Desc_Dot (Id_Of (P, Natural (F)), Fx_Id); Bf := F;
                              end if;
                           end loop;
                           if Bf >= 0 then
                              Pts_In (P, Natural (Bf), Idx, Q);
                              Match_Pair (Id_Of (P, Natural (Bf)), Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
                              N_Pairs_Of (P) := N_Pairs_Of (P) + 1;
                              for I in 0 .. Natural (Keep.Length) - 1 loop
                                 Arm_Obs (P).Append (Arm_Ob'(K => Natural (Idx (Natural (Keep (I)))), Bf => Natural (Bf), Wk => K_New, Ro => G.Pos, Rd => Ray_Fixed (G, U (I), V (I)),
                                                             U => U (I), V => V (I), Uw => Q (Natural (Keep (I))).U, Vw => Q (Natural (Keep (I))).V, E => E (I)));
                              end loop;
                           end if;
                        end;
                     end if;
                  end loop;
                  Say ("放进世界:不长在手上的那只眼 —— 配了 " & Codec.Img (N_Pairs_Fx) & " 对画面,往返 1 px 内共同看见的点 " & Codec.Img (Natural (Cam_Obs.Length))
                       & " 个 ⇒ 焦距 " & Codec.Fmt (G.F, 1) & "、残差 " & Codec.Fmt (G_Rep.Scene_Rms, 2) & " px(进解 " & Codec.Img (G_Rep.Scene_Used) & " / "
                       & Codec.Img (G_Rep.Scene_N) & ")、离桌面 " & Codec.Fmt (abs Dist (G.Pos, Pl0, N0), 3) & " 单位 " & Lap);
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
                  S := Cand_S (Natural (Best)); R := Cand_R (Natural (Best)); T := Cand_T (Natural (Best));
                  Inl := Cand_Inl (Natural (Best)); Md := Cand_Md (Natural (Best));
                  Say ("  [计时] 放进世界 " & Lap);
                  Wb.S := S; Wb.Ra := R; Wb.Ta := T;
                  Worlds.Replace_Element (Natural (Best), Wb);
                  Placed (Natural (Best)) := True;
                  declare
                     K0 : constant Natural := Natural (Wv.Length);
                  begin
                     Add_Arm_Views (Natural (Best));
                     --  已经放进世界的不长在手上的眼,也和这只手新带进来的每一格配一次(同它放进世界时的配法;先后不同,证据不能不同)
                     if Fx_Placed then
                        for K in K0 .. Natural (Wv.Length) - 1 loop
                           declare
                              Vw : constant World_View := Wv (K);
                              Idx, Keep : Ints;
                              Q : Instrument.Match_Vectors.Vector;
                              U, V, E : Floats;
                           begin
                              Pts_In (Natural (Vw.Arm), Vw.Frame, Idx, Q);
                              Match_Pair (Vw.Id, Fx_Id, Ds (0).World_Img.W, Ds (0).World_Img.H, Q, Keep, U, V, E);
                              N_Pairs_Fx := N_Pairs_Fx + 1;
                              for I in 0 .. Natural (Keep.Length) - 1 loop
                                 declare
                                    Pt : constant Tri_Pt := Ps (Natural (Vw.Arm)) (Natural (Idx (Natural (Keep (I)))));
                                 begin
                                    Cam_Obs.Append (Cam_Ob'(Pa => Natural (Vw.Arm), Pk => Natural (Idx (Natural (Keep (I)))), Wk => K, Uw => Q (Natural (Keep (I))).U,
                                                            Vw => Q (Natural (Keep (I))).V, Xw => To_World (Natural (Vw.Arm), Pt.X), Cw => Cov_World (Natural (Vw.Arm), Pt.Cov),
                                                            U => U (I), V => V (I), E => E (I)));
                                 end;
                              end loop;
                           end;
                        end loop;
                     end if;
                  end;
                  --  按它全部的配点重放一遍:刚才加密配进不动的眼的那些格也当它的证据(同一批配点,不再配)。放进世界那一步手里只有
                  --  "最像那只眼的一格"(09-27 V1B15 回放:配进头顶眼的只有一格 192 个点,网格挑到倍数 1.31,靠一起精修才拉回 1.0066)。
                  --  只在重放时借用;放好就撤掉,一起精修里它们只在不动的眼那一组算一次
                  if Fx_Placed then
                     declare
                        Bi : constant Natural := Natural (Best);
                        N_Before : constant Natural := Natural (Arm_Obs (Bi).Length);
                        Fk : Natural := 0;
                        Have : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural'Max (1, Natural (Ds (Bi).Frames.Length))));
                     begin
                        for K in 0 .. Natural (Wv.Length) - 1 loop
                           if Wv (K).Arm < 0 then
                              Fk := K;
                           end if;
                        end loop;
                        for Ob of Arm_Obs (Bi) loop
                           if Ob.Wk = Fk and then Ob.Bf < Natural (Have.Length) then
                              Have.Replace_Element (Ob.Bf, True);   --  这一格和不动的眼放进世界之前就配过了,已经在它的证据里
                           end if;
                        end loop;
                        for X of Cam_Obs loop
                           if X.Pa = Bi and then Wv (X.Wk).Arm = Integer (Bi) and then Wv (X.Wk).Frame < Natural (Have.Length) and then not Have (Wv (X.Wk).Frame) then
                              Arm_Obs (Bi).Append (Arm_Ob'(K => X.Pk, Bf => Wv (X.Wk).Frame, Wk => Fk, Ro => G.Pos, Rd => Ray_Fixed (G, X.U, X.V),
                                                          U => X.U, V => X.V, Uw => X.Uw, Vw => X.Vw, E => X.E));
                           end if;
                        end loop;
                        Say ("  [计时] 加密完 " & Lap);
                        if Natural (Arm_Obs (Bi).Length) > N_Before then
                           Place_Arm (Bi, S, R, T, Inl, Md);
                           Wb.S := S; Wb.Ra := R; Wb.Ta := T;
                           Worlds.Replace_Element (Bi, Wb);
                           Say ("  第" & Codec.Img (Bi + 1) & " 只手按全部配点重放(加上加密配进不长在手上的眼的 " & Codec.Img (Natural (Arm_Obs (Bi).Length) - N_Before)
                                & " 个):放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次 ⇒ 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
                        end if;
                        Arm_Obs (Bi).Set_Length (Ada.Containers.Count_Type (N_Before));
                     end;
                  end if;
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
                       & " 次,放进世界后投回去在门里的 " & Codec.Img (Inl) & " 次(残差中位 " & Codec.Fmt (Md, 2) & " 倍标准差)⇒ 长度倍数 " & Codec.Fmt (S, 4) & " " & Lap);
               end;
            end if;
         end;
      end loop;
      Say ("  一起精修之前 " & Lap);
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
      --  交给开机后半段:世界系的桌面(原点在桌面上、法向 +z,见上面定世界那一段),离散 = 离面中位换标准差
      Plane_Pt := Ap (Rw, [Pl0 (0) - O (0), Pl0 (1) - O (1), Pl0 (2) - O (2)]);
      Plane_N := Ap (Rw, N0);
      Plane_Rms := 1.4826 * Pl0_Md;   --  正态下中位绝对偏差 → 标准差(统计常数,无量纲)
      --  板:配进不动的眼的每个点(同一个点从几格配进来的,留往返差最小的那一笔),按一起精修以后的放法搬进世界;
      --  每轴噪声 = 这一批往返差的中位 ÷ 1.1774(二维正态下距离的中位 = 1.1774 σ,统计常数)
      if Fx_Placed and then not Cam_Obs.Is_Empty then
         declare
            package Sorting is new F64_Vectors.Generic_Sorting;
            Es : Floats;
            Sh : Long_Float;
            package Key_Maps is new Ada.Containers.Vectors (Natural, Integer);
            Best : array (0 .. Natural'Max (1, Na) - 1) of Key_Maps.Vector;
         begin
            for X of Cam_Obs loop
               Es.Append (X.E);
            end loop;
            Sorting.Sort (Es);
            Sh := Long_Float'Max (Es (Natural (Es.Length) / 2) / 1.1774, 1.0e-6);   --  数值保护(无量纲)
            for A in 0 .. Na - 1 loop
               Best (A) := Key_Maps.To_Vector (-1, Ada.Containers.Count_Type (Natural'Max (1, Natural (Ps (A).Length))));
            end loop;
            for I in 0 .. Natural (Cam_Obs.Length) - 1 loop
               declare
                  X : constant Cam_Ob := Cam_Obs (I);
               begin
                  if X.Pa < Na and then (X.Pa = 0 or else Placed (X.Pa)) and then X.Pk < Natural (Best (X.Pa).Length)
                    and then (Best (X.Pa) (X.Pk) < 0 or else X.E < Cam_Obs (Natural (Best (X.Pa) (X.Pk))).E)
                  then
                     Best (X.Pa).Replace_Element (X.Pk, Integer (I));
                  end if;
               end;
            end loop;
            for A in 0 .. Na - 1 loop
               for K in 0 .. Natural (Best (A).Length) - 1 loop
                  if Best (A) (K) >= 0 then
                     declare
                        X : constant Cam_Ob := Cam_Obs (Natural (Best (A) (K)));
                        Sa : constant Long_Float := (if A = 0 then 1.0 else Worlds (A).S);
                        Ra : constant M3 := (if A = 0 then Identity else Worlds (A).Ra);
                        Ta : constant V3 := (if A = 0 then [0.0, 0.0, 0.0] else Worlds (A).Ta);
                        Rx : constant V3 := Ap (Ra, Ps (A) (K).X);
                        X0 : constant V3 := [Sa * Rx (0) + Ta (0) - O (0), Sa * Rx (1) + Ta (1) - O (1), Sa * Rx (2) + Ta (2) - O (2)];
                        Rc : constant M3 := Mul (Mul (Mul (Rw, Ra), Ps (A) (K).Cov), Tr (Mul (Rw, Ra)));
                        Cw : M3;
                     begin
                        for I in 0 .. 2 loop
                           for J in 0 .. 2 loop
                              Cw (I, J) := Sa * Sa * Rc (I, J);
                           end loop;
                        end loop;
                        Board.Append (Scene_Pt'(Pw => Ap (Rw, X0), Cov => Cw, U => X.U, V => X.V, Sh => Sh, Views => 2));
                     end;
                  end if;
               end loop;
            end loop;
            Say ("交给开机后半段:板 " & Codec.Img (Natural (Board.Length)) & " 个点(配进不动的眼的,每轴噪声 " & Codec.Fmt (Sh, 2) & " px)· 桌面离散 "
                 & Codec.Fmt (Plane_Rms, 4) & " 单位");
         end;
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
            --  配进不动的眼的每一笔(离线查它的位姿准不准用):哪只手、它的第几格、第几个三角点、在不动的眼里的像素、往返差、这个点在那只手自己系里的坐标和协方差、三角它的另一格
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/cam_obs.txt");
            for X of Cam_Obs loop
               declare
                  P : constant Tri_Pt := Ps (X.Pa) (X.Pk);
               begin
                  Ada.Text_IO.Put_Line (Fo, Codec.Img (X.Pa) & " " & Codec.Img (Wv (X.Wk).Frame) & " " & Codec.Img (X.Pk) & " " & Codec.Fmt (X.U, 3) & " " & Codec.Fmt (X.V, 3)
                                        & " " & Codec.Fmt (X.E, 3) & " " & Codec.Fmt (P.X (0), 6) & " " & Codec.Fmt (P.X (1), 6) & " " & Codec.Fmt (P.X (2), 6)
                                        & " " & Codec.Fmt (P.Cov (0, 0), 9) & " " & Codec.Fmt (P.Cov (0, 1), 9) & " " & Codec.Fmt (P.Cov (0, 2), 9)
                                        & " " & Codec.Fmt (P.Cov (1, 1), 9) & " " & Codec.Fmt (P.Cov (1, 2), 9) & " " & Codec.Fmt (P.Cov (2, 2), 9)
                                        & " " & Codec.Img (P.Fr));
               end;
            end loop;
            Ada.Text_IO.Close (Fo);
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
      Say ("  对齐用了 " & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒 " & Lap);
   end Align;

   --  ── 到过的范围(09-29,owner:"已知范围,越用越大"):到过的范围、往外一步、尽头 ──

   procedure Set_Ranges (D : Sweep_Data; W : in out Arm_World) is
   begin
      W.Lo.Clear; W.Hi.Clear; W.Got_Lo.Clear; W.Got_Hi.Clear; W.Step_Lo.Clear; W.Step_Hi.Clear;
      W.Eye_W := D.W;
      if D.Frames.Is_Empty then
         return;
      end if;
      for J in 0 .. Natural (D.Frames (0).Q.Length) - 1 loop
         declare
            Lo : Long_Float := Long_Float'Last;
            Hi : Long_Float := Long_Float'First;
            Lim_Lo : constant Boolean := J < Natural (D.Has_Lo.Length) and then D.Has_Lo (J);
            Lim_Hi : constant Boolean := J < Natural (D.Has_Hi.Length) and then D.Has_Hi (J);
         begin
            for Fr of D.Frames loop
               if J < Natural (Fr.Q.Length) then
                  Lo := Long_Float'Min (Lo, Fr.Q (J)); Hi := Long_Float'Max (Hi, Fr.Q (J));
               end if;
            end loop;
            --  记下的尽头:扫描时这一边是"关节到头"停的,以扫到的最远那一格为界;走满格数 / 碰上东西了停的这一边没量到头,不设界
            --  (V1B18 2026-09-27:问"够不够得着"时只按扫到过的范围解,碰桌面前转手转到 0.32 弧度就解不出更远的了 ⇒ 问够不够得着只按尽头)
            W.Lo.Append (if Lim_Lo then Lo else Long_Float'First);
            W.Hi.Append (if Lim_Hi then Hi else Long_Float'Last);
            W.Got_Lo.Append (Lo); W.Got_Hi.Append (Hi);
            W.Step_Lo.Append (if J < Natural (D.Step_Lo.Length) then D.Step_Lo (J) else 0.0);
            W.Step_Hi.Append (if J < Natural (D.Step_Hi.Length) then D.Step_Hi (J) else 0.0);
         end;
      end loop;
   end Set_Ranges;

   procedure Cmd_Bounds (W : Arm_World; Lo, Hi : out Floats) is
      N : constant Natural := Natural'Min (Natural (W.Lo.Length), Natural (W.Hi.Length));
   begin
      Lo.Clear; Hi.Clear;
      for J in 0 .. N - 1 loop
         declare
            Has : constant Boolean := J < Natural (W.Got_Lo.Length) and then J < Natural (W.Got_Hi.Length)
              and then J < Natural (W.Step_Lo.Length) and then J < Natural (W.Step_Hi.Length);
         begin
            Lo.Append (if Has then Long_Float'Max (W.Lo (J), W.Got_Lo (J) - W.Step_Lo (J)) else W.Lo (J));
            Hi.Append (if Has then Long_Float'Min (W.Hi (J), W.Got_Hi (J) + W.Step_Hi (J)) else W.Hi (J));
         end;
      end loop;
   end Cmd_Bounds;

   function Judge_End (Q_Cmd, Q_At, Q_Now, Got_Lo, Got_Hi, Step_Lo, Step_Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return End_Verdict is
      N : constant Natural := Natural'Min (Natural'Min (Natural (Q_Cmd.Length), Natural (Q_At.Length)),
                                           Natural'Min (Natural (Q_Now.Length), Natural'Min (Natural (Got_Lo.Length), Natural (Got_Hi.Length))));
      Shorts : Natural := 0;
      Blocked_Any : Boolean := False;
      --  到了 = 差不到 Tol,或者差不到它这一下要走的三分之一(比例,同扫描"到了")
      function Arrived (K : Natural) return Boolean is
        (abs (Q_Now (K) - Q_Cmd (K)) <= Long_Float'Max (Tol, abs (Q_Cmd (K) - Q_At (K)) * Third));
   begin
      J := -1; Hi_Side := False;
      for K in 0 .. N - 1 loop
         declare
            --  要往范围外走的那一截不到半步(这个关节那一边量过的步子;没量过 = 0)⇒ 这个关节不核:走没走到都判不准,
            --  和手爬到位的误差一个量级(V1B64:只多要 0.0057 弧度、停在一半不到 ⇒ 记成尽头,第 2 只手接着 11 回"在量到的关节限位里解不出来",
            --  读数越过它才删掉)。每拍跟着重发时,真碰到尽头的那一条多要的正好一整步(范围不长了、界还在一步开外)⇒ 照样核得到
            Half_Hi : constant Long_Float := (if K < Natural (Step_Hi.Length) then 0.5 * Step_Hi (K) else 0.0);
            Half_Lo : constant Long_Float := (if K < Natural (Step_Lo.Length) then 0.5 * Step_Lo (K) else 0.0);
            Up : constant Boolean := Q_Cmd (K) > Got_Hi (K) + Long_Float'Max (Tol, Half_Hi);
            Down : constant Boolean := Q_Cmd (K) < Got_Lo (K) - Long_Float'Max (Tol, Half_Lo);
            Small : constant Boolean := not Up and then not Down and then (Q_Cmd (K) > Got_Hi (K) + Tol or else Q_Cmd (K) < Got_Lo (K) - Tol);
         begin
            if Small then
               null;
            elsif Up or else Down then
               declare
                  B : constant Long_Float := (if Up then Got_Hi (K) else Got_Lo (K));
                  Ext : constant Long_Float := abs (Q_Cmd (K) - B);
                  Prog : constant Long_Float := (if Up then Q_Now (K) - B else B - Q_Now (K));
               begin
                  if Prog + Prog < Ext then   --  走到的不到要往外走的那一截的一半(纯数学的一半,同扫描)
                     Shorts := Shorts + 1;
                     J := K; Hi_Side := Up;
                  elsif not Arrived (K) then
                     Blocked_Any := True;
                  end if;
               end;
            elsif not Arrived (K) then
               Blocked_Any := True;   --  范围里的关节没到 / 被顶偏
            end if;
         end;
      end loop;
      if Blocked_Any then
         if Shorts /= 1 then
            J := -1;
         end if;
         return Blocked;
      elsif Shorts = 1 then
         return End_Hit;
      elsif Shorts > 1 then
         J := -1;
         return Ambiguous;
      end if;
      return Reached;
   end Judge_End;

   function End_Passed (Q, Lo, Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return Boolean is
   begin
      J := -1; Hi_Side := False;
      for K in 0 .. Natural'Min (Natural (Q.Length), Natural'Min (Natural (Lo.Length), Natural (Hi.Length))) - 1 loop
         if Hi (K) /= Long_Float'Last and then Q (K) > Hi (K) + Tol then
            J := K; Hi_Side := True;
            return True;
         elsif Lo (K) /= Long_Float'First and then Q (K) < Lo (K) - Tol then
            J := K; Hi_Side := False;
            return True;
         end if;
      end loop;
      return False;
   end End_Passed;

   --  判"到了 / 停在界上"的最小一档:这只手那只眼里画面挪不到 1 像素的转角(Kinem.Clean_Tol,同扫描收格子);画幅不知道 ⇒ 0(只认正好相等,宁可不记)
   function Tol_Of (W : Arm_World) return Long_Float is (if W.Eye_W > 0 then Kinem.Clean_Tol (Long_Float (W.Eye_W)) else 0.0);

   --  ── 装上以后插头用的状态 ──
   St_Worlds : Arm_World_Vectors.Vector;
   St_Rw : Geom.M3 := Geom.Identity;
   St_O : Geom.V3 := [0.0, 0.0, 0.0];
   St_Last : Plug.Floats_Vectors.Vector;   --  每只手最近一帧的关节读数(反解从这儿起)
   St_Noise : Long_Float := 0.0;           --  开机量的关节读数噪声(判"停下了"的下限)
   Still_Frac : constant := 0.01;          --  停下了 = 一拍挪的不到这条命令的百分之一(比例;同 Selfmap.Go 判关节目标停了)
   --  每只手上一条位姿命令:要不要等它停下核有没有碰到尽头、反解是不是被"往外一步"卡住了
   type Pend_State is record
      Live : Boolean := False;             --  有一条要到范围外的命令还没核
      Q_Cmd, Q_At, Glo, Ghi : Floats;      --  它的关节目标、发命令时的读数、发命令时到过的范围
      Still : Natural := 0;                --  停下了几拍
      Held_Back : Boolean := False;        --  上一条命令有关节被夹到"到过的范围 + 往外一步"(没发到解出来的那一处)
      Cmd_Glo, Cmd_Ghi : Floats;           --  发上一条命令时到过的范围(之后长了,重解才会往前走)
   end record;
   package Pend_Vectors is new Ada.Containers.Vectors (Natural, Pend_State);
   St_Pend : Pend_Vectors.Vector;
   --  身体文件(Remember_Kin):干活时范围长了 / 尽头变了写回
   St_Kin_Path : Unbounded_String;
   St_Kin : Kin_Store;
   St_Kin_Idx : Ints;                      --  St_Worlds 第 i 只手是 St_Kin.Worlds 的第几只
   St_Saved_Glo, St_Saved_Ghi : Plug.Floats_Vectors.Vector;   --  上回写文件时每只手到过的范围

   procedure Remember_Kin (Path : String; K : Kin_Store) is
   begin
      St_Kin_Path := To_Unbounded_String (Path);
      St_Kin := K;
   end Remember_Kin;

   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3; Joint_Noise : Long_Float := 0.0) is
   begin
      St_Worlds.Clear; St_Last.Clear; St_Pend.Clear; St_Kin_Idx.Clear; St_Saved_Glo.Clear; St_Saved_Ghi.Clear;
      for I in 0 .. Natural (Worlds.Length) - 1 loop
         if Worlds (I).Valid then
            St_Worlds.Append (Worlds (I));
            St_Last.Append (Worlds (I).Model.Q0);
            St_Pend.Append (Pend_State'(others => <>));
            St_Kin_Idx.Append (I);
            St_Saved_Glo.Append (Worlds (I).Got_Lo); St_Saved_Ghi.Append (Worlds (I).Got_Hi);
         end if;
      end loop;
      St_Rw := Rw; St_O := O; St_Noise := Joint_Noise;
      Plug.Set_Hooks (Pose_Hook'Access, Cmd_Hook'Access);
      Plug.Set_Reach (Reach_Hook'Access);
      Plug.Set_Limit (Held_Back'Access);
      Say ("装上:从此每一帧手的位姿 = 按关节读数算出的腕眼位姿(" & Codec.Img (Natural (St_Worlds.Length)) & " 只手),位姿命令 = 按记下的尽头解出关节目标、"
           & "每个关节夹到到过的范围往外一步里发(到过的范围;问够不够得着只按尽头)");
   end Install;

   --  上一条被截住了没有;截住了,从那一条以来到过的范围长了没有(不止一档)—— 手还没动(命令要隔一两拍才起效)⇒ Held,截住的状态留着;
   --  手一动、范围一长 ⇒ Held_Grown,重解重发。手停在真的尽头 / 碰上东西 ⇒ 范围不再长 ⇒ 一直 Held,照常停下、核尽头
   function Held_Back (Arm : Natural) return Plug.Limit_State is
   begin
      if Arm >= Natural (St_Pend.Length) or else Arm >= Natural (St_Worlds.Length) or else not St_Pend (Arm).Held_Back then
         return Plug.Free;
      end if;
      declare
         P : constant Pend_State := St_Pend (Arm);
         W : constant Arm_World := St_Worlds (Arm);
         Tol : constant Long_Float := Tol_Of (W);
      begin
         for J in 0 .. Natural'Min (Natural (W.Got_Hi.Length), Natural'Min (Natural (P.Cmd_Ghi.Length), Natural (P.Cmd_Glo.Length))) - 1 loop
            if W.Got_Hi (J) > P.Cmd_Ghi (J) + Tol or else W.Got_Lo (J) < P.Cmd_Glo (J) - Tol then
               return Plug.Held_Grown;
            end if;
         end loop;
         return Plug.Held;
      end;
   end Held_Back;

   --  到过的范围长了一步以上、或者尽头变了 ⇒ 写回身体文件(只写字)
   procedure Save_If_Grown (Changed_End : Boolean) is
      Grown : Boolean := False;
   begin
      if Length (St_Kin_Path) = 0 then
         return;
      end if;
      for A in 0 .. Natural (St_Worlds.Length) - 1 loop
         declare
            W : Arm_World renames St_Worlds (A);
         begin
            for J in 0 .. Natural'Min (Natural (W.Got_Hi.Length), Natural (St_Saved_Ghi (A).Length)) - 1 loop
               if (W.Step_Hi (J) > 0.0 and then W.Got_Hi (J) - St_Saved_Ghi (A) (J) >= W.Step_Hi (J))
                 or else (W.Step_Lo (J) > 0.0 and then St_Saved_Glo (A) (J) - W.Got_Lo (J) >= W.Step_Lo (J))
               then
                  Grown := True;
               end if;
            end loop;
         end;
      end loop;
      if not (Changed_End or else Grown) then
         return;
      end if;
      for I in 0 .. Natural (St_Worlds.Length) - 1 loop
         if St_Kin_Idx (I) < Natural (St_Kin.Worlds.Length) then
            declare
               Kw : Arm_World := St_Kin.Worlds (St_Kin_Idx (I));
            begin
               Kw.Lo := St_Worlds (I).Lo; Kw.Hi := St_Worlds (I).Hi;
               Kw.Got_Lo := St_Worlds (I).Got_Lo; Kw.Got_Hi := St_Worlds (I).Got_Hi;
               St_Kin.Worlds.Replace_Element (St_Kin_Idx (I), Kw);
            end;
         end if;
         St_Saved_Glo.Replace_Element (I, St_Worlds (I).Got_Lo); St_Saved_Ghi.Replace_Element (I, St_Worlds (I).Got_Hi);
      end loop;
      begin
         Save_Kin (To_String (St_Kin_Path), St_Kin, Images => False);
      exception
         when others =>
            Say ("到过的关节范围 / 尽头写不回 " & To_String (St_Kin_Path));
      end;
   end Save_If_Grown;

   --  这一帧第 A 只手的读数:到过的范围并进来;越过记下的尽头 ⇒ 删掉那个尽头;有一条要到范围外的命令 ⇒ 等它停下(连着两拍一拍挪不到这条命令的百分之一)
   --  核有没有碰到尽头(Judge_End):正好一个关节没走到一半、别的都到了 ⇒ 这一头到了,记下;别的关节没到 / 被顶偏 ⇒ 碰上东西了,不记
   procedure Track (A : Natural; Prev, Qn : Floats) is
      W : Arm_World := St_Worlds (A);
      P : Pend_State := St_Pend (A);
      Tol : constant Long_Float := Tol_Of (W);
      Changed_End : Boolean := False;
      Jx : Integer;
      Hs : Boolean;
      Who : constant String := "第" & Codec.Img (A + 1) & " 只手";
   begin
      for J in 0 .. Natural'Min (Natural (Qn.Length), Natural'Min (Natural (W.Got_Lo.Length), Natural (W.Got_Hi.Length))) - 1 loop
         W.Got_Lo.Replace_Element (J, Long_Float'Min (W.Got_Lo (J), Qn (J)));
         W.Got_Hi.Replace_Element (J, Long_Float'Max (W.Got_Hi (J), Qn (J)));
      end loop;
      while End_Passed (Qn, W.Lo, W.Hi, Tol, Jx, Hs) loop
         Say (Who & "第" & Codec.Img (Jx) & " 个关节读数 " & Codec.Fmt (Qn (Jx), 4) & " 越过了记下的" & (if Hs then "正" else "负") & "那一头 "
              & Codec.Fmt ((if Hs then W.Hi (Jx) else W.Lo (Jx)), 4) & " ⇒ 那个尽头记错了,删掉");
         if Hs then
            W.Hi.Replace_Element (Jx, Long_Float'Last);
         else
            W.Lo.Replace_Element (Jx, Long_Float'First);
         end if;
         Changed_End := True;
      end loop;
      if P.Live then
         declare
            Mv, Big : Long_Float := 0.0;
         begin
            for J in 0 .. Natural'Min (Natural (Qn.Length), Natural (Prev.Length)) - 1 loop
               Mv := Long_Float'Max (Mv, abs (Qn (J) - Prev (J)));
            end loop;
            for J in 0 .. Natural'Min (Natural (P.Q_Cmd.Length), Natural (P.Q_At.Length)) - 1 loop
               Big := Long_Float'Max (Big, abs (P.Q_Cmd (J) - P.Q_At (J)));
            end loop;
            P.Still := (if Mv <= Long_Float'Max (St_Noise, Still_Frac * Big) then P.Still + 1 else 0);
            if P.Still >= 2 then
               P.Live := False;
               case Judge_End (P.Q_Cmd, P.Q_At, Qn, P.Glo, P.Ghi, W.Step_Lo, W.Step_Hi, Tol, Jx, Hs) is
                  when End_Hit =>
                     Say (Who & "第" & Codec.Img (Jx) & " 个关节往" & (if Hs then "正" else "负") & "走:要到 " & Codec.Fmt (P.Q_Cmd (Jx), 4) & "(到过的范围只到 "
                          & Codec.Fmt ((if Hs then P.Ghi (Jx) else P.Glo (Jx)), 4) & "),停在 " & Codec.Fmt (Qn (Jx), 4) & ",往外走的不到一半,别的关节都到了"
                          & " ⇒ 这一头到了,记下(以后反解不往那边算;哪天真走过去了再删)");
                     if Hs then
                        W.Hi.Replace_Element (Jx, Qn (Jx));
                     else
                        W.Lo.Replace_Element (Jx, Qn (Jx));
                     end if;
                     Changed_End := True;
                  when Blocked =>
                     if Jx >= 0 then
                        Say (Who & "第" & Codec.Img (Jx) & " 个关节往范围外走不到一半,可别的关节也没到 / 被顶偏了 ⇒ 是手碰上东西了,不是关节到头,不记");
                     end if;
                  when Ambiguous =>
                     Say (Who & "好几个关节往范围外都走不到一半 ⇒ 分不清是哪一个到头了,不记");
                  when Reached =>
                     null;
               end case;
            end if;
         end;
      end if;
      St_Worlds.Replace_Element (A, W);
      St_Pend.Replace_Element (A, P);
      Save_If_Grown (Changed_End);
   end Track;

   procedure Pose_Hook (F : in out Plug.Frame) is
      use Geom;
   begin
      F.EE.Clear;
      for A in 0 .. Natural (St_Worlds.Length) - 1 loop
         if St_Worlds (A).Group < Natural (F.Joints.Length) then
            Track (A, St_Last (A), F.Joints (St_Worlds (A).Group));
         end if;
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

   --  世界里的一个腕眼位姿 ⇒ 第 A 只手的关节目标(位姿命令、开机自检、问够不够得着都走这一条)。
   --  每个关节夹到"到过的范围 + 往外一步"里(Cmd_Bounds);Clamped = 有关节被夹住了
   procedure Clamp_Cmd (W : Arm_World; Q : in out Floats; Clamped : out Boolean) is
      Lo, Hi : Floats;
   begin
      Clamped := False;
      Cmd_Bounds (W, Lo, Hi);
      for J in 0 .. Natural'Min (Natural (Q.Length), Natural'Min (Natural (Lo.Length), Natural (Hi.Length))) - 1 loop
         if Q (J) > Hi (J) then
            Q.Replace_Element (J, Hi (J)); Clamped := True;
         elsif Q (J) < Lo (J) then
            Q.Replace_Element (J, Lo (J)); Clamped := True;
         end if;
      end loop;
   end Clamp_Cmd;

   --  先只按记下的尽头解出这个位姿要的关节(Pe / Re = 解完还差多少:位置按第一只手的模型单位 = 世界的单位,朝向按弧度)。
   --  For_Command = 真要发出去(到过的范围):每个关节再夹到"到过的范围 + 往外一步"里 —— 每个关节直接朝那个解走、一条命令最多走出到过的地方一步;
   --  Clamped = 有关节被夹住了(这一条到不了那个解,手走过去、范围长了再往前)。
   --  (09-29 离线接真 Go 查出来的:原来直接在"到过的范围 + 一步"里反解,被夹住的那个关节差的那点由别的关节凑 ——
   --  只转第 4 个关节到 0.8,第 0、2 个关节中途被拉出去 0.23 弧度再转回来;到不了时停在一个扭着的姿势)
   procedure Pose_To_Q (A : Natural; Pose : Plug.Arm_Pose; For_Command : Boolean; Q : out Floats; Pe, Re : out Long_Float; Clamped : out Boolean) is
      use Geom;
      W : Arm_World renames St_Worlds (A);
      Rt_W : constant M3 := Quat_To_R (Pose);
      Tt_W : constant V3 := [Pose (0), Pose (1), Pose (2)];
      --  世界 → 第一只手参照眼系 → 这只手参照眼系
      R0 : constant M3 := Mul (Tr (St_Rw), Rt_W);
      T0w : constant V3 := Ap (Tr (St_Rw), Tt_W);
      T0 : constant V3 := [T0w (0) + St_O (0), T0w (1) + St_O (1), T0w (2) + St_O (2)];
      Ra_T : constant M3 := Tr (W.Ra);
      Ra_Arm : constant M3 := Mul (Ra_T, R0);
      Ta_D : constant V3 := Ap (Ra_T, [T0 (0) - W.Ta (0), T0 (1) - W.Ta (1), T0 (2) - W.Ta (2)]);
      Ta_Arm : constant V3 := [Ta_D (0) / W.S, Ta_D (1) / W.S, Ta_D (2) / W.S];
   begin
      Clamped := False;
      Kinem.IK (W.Model, Ra_Arm, Ta_Arm, St_Last (A), W.Lo, W.Hi, Q, Pe, Re);
      Pe := Pe * W.S;   --  换成第一只手的模型单位(= 世界的单位)
      if For_Command then
         Clamp_Cmd (W, Q, Clamped);
      end if;
   end Pose_To_Q;

   procedure Reach_Hook (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
      Q : Floats;
      Cl : Boolean;
   begin
      Pos_Err := Long_Float'Last; Rot_Err := Long_Float'Last;
      if Arm < Natural (St_Worlds.Length) then
         Pose_To_Q (Arm, Pose, False, Q, Pos_Err, Rot_Err, Cl);
      end if;
   end Reach_Hook;

   --  一条要发出去的位姿命令解成了 Q(Full = 只按记下的尽头解出来的那一个,Q = 夹到"到过的范围 + 往外一步"里以后真发的):
   --  记下要不要等它停下核尽头(有关节的目标出了到过的范围)、有没有被夹住(重不重发看之后范围长没长,见 Held_Back)
   procedure Note_Command (A : Natural; Q, Full : Floats; Clamped : Boolean) is
      W : constant Arm_World := St_Worlds (A);
      P : Pend_State := St_Pend (A);
      Tol : constant Long_Float := Tol_Of (W);
      Beyond : Boolean := False;
   begin
      for J in 0 .. Natural'Min (Natural (Q.Length), Natural (W.Got_Hi.Length)) - 1 loop
         if Q (J) > W.Got_Hi (J) + Tol or else Q (J) < W.Got_Lo (J) - Tol then
            Beyond := True;
         end if;
      end loop;
      --  只在"开始被夹住"那一刻说一句(之后跟着往前重发的都还是它,不重复说):夹得最多的那个关节要到哪、这一条只到哪
      if Clamped and then not P.Held_Back then
         declare
            Jm : Natural := 0;
            Dm : Long_Float := -1.0;
         begin
            for J in 0 .. Natural'Min (Natural (Q.Length), Natural (Full.Length)) - 1 loop
               if abs (Full (J) - Q (J)) > Dm then
                  Dm := abs (Full (J) - Q (J)); Jm := J;
               end if;
            end loop;
            Say ("第" & Codec.Img (A + 1) & " 只手:第 " & Codec.Img (Jm) & " 个关节要到 " & Codec.Fmt (Full (Jm), 4) & ",这一条只发到 " & Codec.Fmt (Q (Jm), 4)
                 & "(到过的范围 + 往外一步)⇒ 手走过去、范围长了就跟着往前重发(到过的范围)");
         end;
      end if;
      P.Held_Back := Clamped;
      P.Cmd_Glo := W.Got_Lo; P.Cmd_Ghi := W.Got_Hi;
      --  上一条还没核就来了新的 = 被打断了,不核(到过的范围原话)
      P.Live := Beyond;
      if Beyond then
         P.Q_Cmd := Q; P.Q_At := St_Last (A); P.Glo := W.Got_Lo; P.Ghi := W.Got_Hi; P.Still := 0;
      end if;
      St_Pend.Replace_Element (A, P);
   end Note_Command;

   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean) is
      Q, Full : Floats;
      Pe, Re : Long_Float;
      Cl : Boolean;
   begin
      Ok := False;
      if C.Arm >= Natural (St_Worlds.Length) then
         return;
      end if;
      Pose_To_Q (C.Arm, C.Pose, False, Full, Pe, Re, Cl);
      Q := Full;
      Clamp_Cmd (St_Worlds (C.Arm), Q, Cl);
      Note_Command (C.Arm, Q, Full, Cl);
      C.Kind := Plug.Joint;
      C.Group := St_Worlds (C.Arm).Group;
      C.Q := Q;
      Ok := True;
   end Cmd_Hook;
   N_Check : constant := 3;   --  每只手走几处(次数)
   procedure Self_Check (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Ds : Sweep_Vectors.Vector; Dump : String) is
      use Geom;
      Fo : Ada.Text_IO.File_Type;
      Have_Fo : Boolean := False;
      Na : constant Natural := Natural (St_Worlds.Length);
      --  每只手的目标:两格"几个关节一起动"的读数正中(扫描时没去过的地方)按运动学算到的腕眼位姿,搬到世界里
      Cmb : array (0 .. Natural'Max (1, Na) - 1) of Ints;
      N_Done : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
      Sum_P, Max_P, Max_Pe : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
      T0c : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      Seq0 : constant Natural := L.Seq;
   begin
      if Dump /= "" then
         begin
            Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/ik_check.txt");
            Have_Fo := True;
         exception
            when others => null;
         end;
      end if;
      for A in 0 .. Na - 1 loop
         for K in 1 .. Natural (Ds (St_Worlds (A).Sweep).Frames.Length) - 1 loop
            if Ds (St_Worlds (A).Sweep).Frames (K).Joint < 0 then
               Cmb (A).Append (K);
            end if;
         end loop;
      end loop;
      --  几只手同时走:每一处一条关节命令带几只手的目标(每只手的目标按位姿命令同一条路 Pose_To_Q 解)
      for I in 0 .. N_Check - 1 loop
         declare
            Targets : Plug.Pose_Vectors.Vector;
            Pes, Rrs : Floats;
            Gs : Ints;
            Qs : Plug.Floats_Vectors.Vector;
            Tol : Long_Float := Long_Float'Last;
            Dl : Table.Vec;
            Fr : Natural;
            Okg : Boolean;
         begin
            for A in 0 .. Na - 1 loop
               declare
                  W : constant Arm_World := St_Worlds (A);
                  Target : Plug.Arm_Pose := [others => 0.0];
                  Q : Floats;
                  Pe, Re : Long_Float := 0.0;
               begin
                  if I + 1 < Natural (Cmb (A).Length) then
                     declare
                        Qa : constant Floats := Ds (W.Sweep).Frames (Natural (Cmb (A) (I))).Q;
                        Qb : constant Floats := Ds (W.Sweep).Frames (Natural (Cmb (A) (I + 1))).Q;
                        Qm : Floats;
                        Rr : M3;
                        Tt : V3;
                     begin
                        for J in 0 .. Natural'Min (Natural (Qa.Length), Natural (Qb.Length)) - 1 loop
                           Qm.Append (0.5 * (Qa (J) + Qb (J)));
                        end loop;
                        Kinem.FK (W.Model, Qm, Rr, Tt);
                        declare
                           R0 : constant M3 := Mul (W.Ra, Rr);
                           Rt : constant V3 := Ap (W.Ra, Tt);
                           T0 : constant V3 := [W.S * Rt (0) + W.Ta (0) - St_O (0), W.S * Rt (1) + W.Ta (1) - St_O (1), W.S * Rt (2) + W.Ta (2) - St_O (2)];
                        begin
                           Target := Kinem.To_Pose (Mul (St_Rw, R0), Ap (St_Rw, T0));
                        end;
                        declare
                           Cl : Boolean;
                        begin
                           Pose_To_Q (A, Target, True, Q, Pe, Re, Cl);
                        end;
                        Gs.Append (W.Group); Qs.Append (Q);
                        --  到了 = 每个关节差不到它这一下要走的三分之一(比例,同扫描)
                        if W.Group < Natural (F.Joints.Length) then
                           for J in 0 .. Natural'Min (Natural (Q.Length), Natural (F.Joints (W.Group).Length)) - 1 loop
                              if abs (Q (J) - F.Joints (W.Group) (J)) > 0.0 then
                                 Tol := Long_Float'Min (Tol, Third * abs (Q (J) - F.Joints (W.Group) (J)));
                              end if;
                           end loop;
                        end if;
                     end;
                  end if;
                  Targets.Append (Target); Pes.Append (Pe); Rrs.Append (Re);
               end;
            end loop;
            exit when Gs.Is_Empty;
            Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okg, Groups => Gs, Qs => Qs,
                        Tol => (if Tol < Long_Float'Last then Tol else 0.0));
            exit when not Okg;
            for A in 0 .. Na - 1 loop
               if I + 1 < Natural (Cmb (A).Length) and then A < Natural (F.EE.Length) then
                  declare
                     W : constant Arm_World := St_Worlds (A);
                     Target : constant Plug.Arm_Pose := Targets (A);
                     Got : constant Plug.Arm_Pose := F.EE (A);
                     Dp : constant Long_Float := Norm ([Got (0) - Target (0), Got (1) - Target (1), Got (2) - Target (2)]);
                  begin
                     N_Done (A) := N_Done (A) + 1;
                     Sum_P (A) := Sum_P (A) + Dp; Max_P (A) := Long_Float'Max (Max_P (A), Dp); Max_Pe (A) := Long_Float'Max (Max_Pe (A), Pes (A));
                     if Have_Fo then
                        Ada.Text_IO.Put (Fo, Codec.Img (A) & " " & Codec.Img (W.Sweep) & " " & Codec.Img (Fr) & " |");
                        for X of Target loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                        end loop;
                        Ada.Text_IO.Put (Fo, " |");
                        for X of Got loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                        end loop;
                        Ada.Text_IO.Put (Fo, " | " & Codec.Fmt (Pes (A), 7) & " " & Codec.Fmt (Rrs (A), 7) & " |");
                        if W.Group < Natural (F.Joints.Length) then
                           for X of F.Joints (W.Group) loop
                              Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                           end loop;
                        end if;
                        Ada.Text_IO.Put (Fo, " ||");
                        if W.Sweep < Natural (F.Reported_EE.Length) then
                           for X of F.Reported_EE (W.Sweep) loop
                              Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));   --  身体报的手的位姿:只给离线打分,驱动不读
                           end loop;
                        end if;
                        Ada.Text_IO.New_Line (Fo);
                     end if;
                  end;
               end if;
            end loop;
         end;
      end loop;
      for A in 0 .. Na - 1 loop
         Say ("开机自检(V1b ②):第" & Codec.Img (A + 1) & " 只手走到 " & Codec.Img (N_Done (A)) & " 处扫描时没去过的地方(两格""几个关节一起动""的读数正中)⇒ "
              & "按关节读数算到的离目标 平均 " & Codec.Fmt ((if N_Done (A) > 0 then Sum_P (A) / Long_Float (N_Done (A)) else 0.0), 4) & "、最大 " & Codec.Fmt (Max_P (A), 4)
              & " 单位;反解最多还差 " & Codec.Fmt (Max_Pe (A), 4) & " 单位(真值只落盘打分)");
      end loop;
      Say ("  自检 " & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0c)), 1) & " 秒(几只手同时走)");
      if Have_Fo then
         Ada.Text_IO.Close (Fo);
      end if;
   end Self_Check;

   --  ── ⑦ 存 / 装回 ──
   function Kin_Key (L : Plug.Link; F : Plug.Frame) return String is
      R : Unbounded_String;
   begin
      Append (R, "cams=");
      for C of F.Cams loop
         Append (R, Codec.Img (C.W) & "x" & Codec.Img (C.H) & ",");
      end loop;
      Append (R, ";groups=");
      for G of F.Joints loop
         Append (R, Codec.Img (Natural (G.Length)) & ",");
      end loop;
      Append (R, ";jaws=" & Codec.Img (Natural (F.Jaw.Length)) & ";joints=");
      for P of L.Lay.Joints loop
         Append (R, Layout.Last_Seg (P) & ",");
      end loop;
      declare
         S : String := To_String (R);
      begin
         for I in S'Range loop
            if S (I) = ' ' then
               S (I) := '_';   --  钥匙在文件里是一行里的一个词
            end if;
         end loop;
         return S;
      end;
   end Kin_Key;

   function F9 (X : Long_Float) return String is (Codec.Fmt (X, 9));
   function Lim (X : Long_Float) return String is
     (if X = Long_Float'First or else X = Long_Float'Last then "none" else F9 (X));   --  没量到头的界记成 none

   procedure Save_Kin (Path : String; K : Kin_Store; Images : Boolean := True) is
      use Ada.Text_IO;
      Fo : File_Type;
      procedure Put_M3 (M : Geom.M3) is
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Put (Fo, " " & F9 (M (I, J)));
            end loop;
         end loop;
      end Put_M3;
      procedure Put_V3 (V : Geom.V3) is
      begin
         Put (Fo, " " & F9 (V (0)) & " " & F9 (V (1)) & " " & F9 (V (2)));
      end Put_V3;
   begin
      Create (Fo, Out_File, Path);
      Put_Line (Fo, "kin 4");   --  格式版本:4 = 每个关节到过的范围和往外一步(到过的范围,09-29);3 = 每根轴记着是转还是走(09-27 无人机);2 = 不动的眼整份相机几何;更旧的读到 ⇒ 从零量
      Put_Line (Fo, "key " & To_String (K.Key));
      Put_Line (Fo, "world_cam" & Integer'Image (K.World_Cam));
      Put (Fo, "rw"); Put_M3 (K.Rw); New_Line (Fo);
      Put (Fo, "o"); Put_V3 (K.O); New_Line (Fo);
      Put (Fo, "plane"); Put_V3 (K.Plane_Pt); Put_V3 (K.Plane_N); Put_Line (Fo, " " & F9 (K.Plane_Rms));
      --  不动的眼:整份相机几何按记录的次序一个字段不落(读回是不带 others 的整份聚合,记录加了字段那边编译不过)。
      --  09-27 V1B35 / V1B38:原来只存焦距、主点、位置、朝向,没存像素残差 ⇒ 装回后每轮核对的门 = 3 × 0 px,板上一个点都对不上,核对瞎了还报"没挪、没挡"
      declare
         G : Geom.Cam_Geo renames K.Fixed_Eye;
         function B (X : Boolean) return String is (if X then "1" else "0");
      begin
         Put (Fo, "fixed " & B (G.Valid) & " " & F9 (G.F) & " " & F9 (G.Cx) & " " & F9 (G.Cy) & " " & F9 (G.K1) & " " & F9 (G.K2) & " " & F9 (G.K1_Sd)
              & " " & F9 (G.F_Meas) & " " & F9 (G.F_Prior) & " " & F9 (G.F_Prior_Sd));
         Put_M3 (G.R_Ce); Put_V3 (G.Off);
         Put (Fo, " " & F9 (G.Rms) & " " & F9 (G.F_Sd) & " " & F9 (G.Rot_Sd) & " " & F9 (G.Off_Sd) & " " & F9 (G.Pos_Sd) & " " & Codec.Img (G.Dropped)
              & " " & B (G.Tip_Valid) & " " & B (G.Tip_Touch));
         Put_V3 (G.Tip);
         Put (Fo, " " & F9 (G.Gap) & " " & F9 (G.Stride) & " " & F9 (G.Stride_Rot) & " " & B (G.Fixed));
         Put_V3 (G.Pos); New_Line (Fo);
      end;
      for A in 0 .. Natural (K.Worlds.Length) - 1 loop
         declare
            W : constant Arm_World := K.Worlds (A);
            D : constant Sweep_Data := K.Ds (A);
         begin
            Put_Line (Fo, "arm " & Codec.Img (A) & " " & Codec.Img (W.Group) & " " & Integer'Image (K.Eyes (A)) & " " & (if W.Valid then "1" else "0") & " "
                      & Codec.Img (W.Model.N) & " " & F9 (W.Model.F) & " " & F9 (W.Model.Cx) & " " & F9 (W.Model.Cy) & " " & F9 (W.S) & " "
                      & Codec.Img (D.W) & " " & Codec.Img (D.H));
            Put (Fo, "q0 " & Codec.Img (A));
            for X of W.Model.Q0 loop
               Put (Fo, " " & F9 (X));
            end loop;
            New_Line (Fo);
            for J in 0 .. W.Model.N - 1 loop
               Put (Fo, "axis " & Codec.Img (A) & " " & Codec.Img (J)); Put_V3 (W.Model.Ax (J).W); Put_V3 (W.Model.Ax (J).P);
               Put_Line (Fo, " " & Kind_Word (W.Model.Ax (J)));
            end loop;
            Put (Fo, "ra " & Codec.Img (A)); Put_M3 (W.Ra); New_Line (Fo);
            Put (Fo, "ta " & Codec.Img (A)); Put_V3 (W.Ta); New_Line (Fo);
            Put (Fo, "lo " & Codec.Img (A));
            for X of W.Lo loop
               Put (Fo, " " & Lim (X));
            end loop;
            New_Line (Fo);
            Put (Fo, "hi " & Codec.Img (A));
            for X of W.Hi loop
               Put (Fo, " " & Lim (X));
            end loop;
            New_Line (Fo);
            --  到过的范围:到过的范围(两头)、往外一步(两边)
            declare
               procedure Row (Tag : String; V : Floats) is
               begin
                  Put (Fo, Tag & " " & Codec.Img (A));
                  for X of V loop
                     Put (Fo, " " & F9 (X));
                  end loop;
                  New_Line (Fo);
               end Row;
            begin
               Row ("glo", W.Got_Lo); Row ("ghi", W.Got_Hi); Row ("slo", W.Step_Lo); Row ("shi", W.Step_Hi);
            end;
            for Fr of D.Frames loop
               Put (Fo, "frame " & Codec.Img (A) & " " & Integer'Image (Fr.Joint));
               for X of Fr.Q loop
                  Put (Fo, " " & F9 (X));
               end loop;
               New_Line (Fo);
            end loop;
            if Images and then not D.Imgs.Is_Empty then
               Codec.Write_BMP (Path & "_arm" & Codec.Img (A) & ".bmp", D.Imgs (0).RGB, D.Imgs (0).W, D.Imgs (0).H);
            end if;
         end;
      end loop;
      for P of K.Board loop
         Put (Fo, "board"); Put_V3 (P.Pw); Put_M3 (P.Cov);
         Put_Line (Fo, " " & F9 (P.U) & " " & F9 (P.V) & " " & F9 (P.Sh) & " " & Codec.Img (P.Views));
      end loop;
      Close (Fo);
      if Images and then K.World_Cam >= 0 and then not K.Ds.Is_Empty and then K.Ds (0).World_Img.W > 0 then
         Codec.Write_BMP (Path & "_world.bmp", K.Ds (0).World_Img.RGB, K.Ds (0).World_Img.W, K.Ds (0).World_Img.H);
      end if;
   end Save_Kin;

   procedure Load_Kin (Path : String; K : out Kin_Store; Ok : out Boolean; Note : out Unbounded_String) is
      use Ada.Text_IO;
      Fi : File_Type;
      function Fields (S : String) return Strs is
         R : Strs;
         I : Natural := S'First;
      begin
         while I <= S'Last loop
            while I <= S'Last and then S (I) = ' ' loop
               I := I + 1;
            end loop;
            exit when I > S'Last;
            declare
               J : Natural := I;
            begin
               while J <= S'Last and then S (J) /= ' ' loop
                  J := J + 1;
               end loop;
               R.Append (S (I .. J - 1));
               I := J;
            end;
         end loop;
         return R;
      end Fields;
      function V (T : Strs; I : Natural) return Long_Float is (Long_Float'Value (T (I)));
      function M3_At (T : Strs; I : Natural) return Geom.M3 is
        ([[V (T, I), V (T, I + 1), V (T, I + 2)], [V (T, I + 3), V (T, I + 4), V (T, I + 5)], [V (T, I + 6), V (T, I + 7), V (T, I + 8)]]);
      function V3_At (T : Strs; I : Natural) return Geom.V3 is ([V (T, I), V (T, I + 1), V (T, I + 2)]);
      function Arm_Of (T : Strs) return Natural is (Natural'Value (T (1)));
      Version_Ok : Boolean := False;
   begin
      K := (others => <>); Ok := False; Note := Null_Unbounded_String;
      Open (Fi, In_File, Path);
      while not End_Of_File (Fi) loop
         declare
            T : constant Strs := Fields (Get_Line (Fi));
            Tag : constant String := (if T.Is_Empty then "" else T (0));
         begin
            if Tag = "kin" then
               Version_Ok := Natural (T.Length) >= 2 and then T (1) = "4";
            elsif Tag = "key" and then Natural (T.Length) >= 2 then
               K.Key := To_Unbounded_String (T (1));
            elsif Tag = "world_cam" then
               K.World_Cam := Integer'Value (T (1));
            elsif Tag = "rw" then
               K.Rw := M3_At (T, 1);
            elsif Tag = "o" then
               K.O := V3_At (T, 1);
            elsif Tag = "plane" then
               K.Plane_Pt := V3_At (T, 1); K.Plane_N := V3_At (T, 4); K.Plane_Rms := V (T, 7);
            elsif Tag = "fixed" and then Natural (T.Length) < 41 then   --  整份相机几何 = 标签 + 40 个字段(格式);少了 = 不是这一版
               Version_Ok := False;
            elsif Tag = "fixed" then
               K.Fixed_Eye := Geom.Cam_Geo'(Valid => T (1) = "1", F => V (T, 2), Cx => V (T, 3), Cy => V (T, 4), K1 => V (T, 5), K2 => V (T, 6), K1_Sd => V (T, 7),
                                           F_Meas => V (T, 8), F_Prior => V (T, 9), F_Prior_Sd => V (T, 10), R_Ce => M3_At (T, 11), Off => V3_At (T, 20),
                                           Rms => V (T, 23), F_Sd => V (T, 24), Rot_Sd => V (T, 25), Off_Sd => V (T, 26), Pos_Sd => V (T, 27),
                                           Dropped => Natural'Value (T (28)), Tip_Valid => T (29) = "1", Tip_Touch => T (30) = "1", Tip => V3_At (T, 31),
                                           Gap => V (T, 34), Stride => V (T, 35), Stride_Rot => V (T, 36), Fixed => T (37) = "1", Pos => V3_At (T, 38),
                                           Lobes => Geom.Lobe_Geo_Vectors.Empty_Vector, Tip_Sd => 0.0);   --  不动的眼没有手指:这两样恒为空
            elsif Tag = "arm" then
               declare
                  A : constant Natural := Arm_Of (T);
                  W : Arm_World;
                  D : Sweep_Data;
               begin
                  while Natural (K.Worlds.Length) <= A loop
                     K.Worlds.Append (Arm_World'(others => <>)); K.Ds.Append (Sweep_Data'(others => <>)); K.Eyes.Append (-1);
                  end loop;
                  W := K.Worlds (A); D := K.Ds (A);
                  W.Group := Natural'Value (T (2));
                  K.Eyes.Replace_Element (A, Integer'Value (T (3)));
                  W.Valid := T (4) = "1";
                  W.Model.N := Natural'Value (T (5));
                  W.Model.F := V (T, 6); W.Model.Cx := V (T, 7); W.Model.Cy := V (T, 8); W.S := V (T, 9);
                  W.Model.Valid := W.Valid; W.Sweep := A;
                  D.W := Natural'Value (T (10)); D.H := Natural'Value (T (11));
                  W.Eye_W := D.W;
                  K.Worlds.Replace_Element (A, W); K.Ds.Replace_Element (A, D);
               end;
            elsif Tag = "q0" or else Tag = "lo" or else Tag = "hi" or else Tag = "axis" or else Tag = "ra" or else Tag = "ta" or else Tag = "frame"
              or else Tag = "glo" or else Tag = "ghi" or else Tag = "slo" or else Tag = "shi"
            then
               declare
                  A : constant Natural := Arm_Of (T);
                  W : Arm_World := K.Worlds (A);
                  D : Sweep_Data := K.Ds (A);
               begin
                  if Tag = "q0" then
                     W.Model.Q0.Clear;
                     for I in 2 .. Natural (T.Length) - 1 loop
                        W.Model.Q0.Append (V (T, I));
                     end loop;
                  elsif Tag = "glo" or else Tag = "ghi" or else Tag = "slo" or else Tag = "shi" then
                     declare
                        Lst : Floats;
                     begin
                        for I in 2 .. Natural (T.Length) - 1 loop
                           Lst.Append (V (T, I));
                        end loop;
                        if Tag = "glo" then
                           W.Got_Lo := Lst;
                        elsif Tag = "ghi" then
                           W.Got_Hi := Lst;
                        elsif Tag = "slo" then
                           W.Step_Lo := Lst;
                        else
                           W.Step_Hi := Lst;
                        end if;
                     end;
                  elsif Tag = "lo" or else Tag = "hi" then
                     declare
                        Lst : Floats;
                     begin
                        for I in 2 .. Natural (T.Length) - 1 loop
                           Lst.Append (if T (I) = "none" then (if Tag = "lo" then Long_Float'First else Long_Float'Last) else V (T, I));
                        end loop;
                        if Tag = "lo" then
                           W.Lo := Lst;
                        else
                           W.Hi := Lst;
                        end if;
                     end;
                  elsif Tag = "axis" then
                     declare
                        J : constant Natural := Natural'Value (T (2));
                     begin
                        W.Model.Ax (J).W := V3_At (T, 3); W.Model.Ax (J).P := V3_At (T, 6);
                        if Natural (T.Length) < 10 or else (T (9) /= "turn" and then T (9) /= "slide") then   --  axis 臂 轴 W P 转/走(格式)
                           Version_Ok := False;
                        else
                           W.Model.Ax (J).Slide := T (9) = "slide";
                        end if;
                     end;
                  elsif Tag = "ra" then
                     W.Ra := M3_At (T, 2);
                  elsif Tag = "ta" then
                     W.Ta := V3_At (T, 2);
                  else
                     declare
                        Fr : Kinem.Frame_Info;
                     begin
                        Fr.Joint := Integer'Value (T (2));
                        for I in 3 .. Natural (T.Length) - 1 loop
                           Fr.Q.Append (V (T, I));
                        end loop;
                        D.Frames.Append (Fr);
                     end;
                  end if;
                  K.Worlds.Replace_Element (A, W); K.Ds.Replace_Element (A, D);
               end;
            elsif Tag = "board" then
               K.Board.Append (Geom.Scene_Pt'(Pw => V3_At (T, 1), Cov => M3_At (T, 4), U => V (T, 13), V => V (T, 14), Sh => V (T, 15),
                                              Views => Natural'Value (T (16))));
            end if;
         end;
      end loop;
      Close (Fi);
      if not Version_Ok then
         Note := To_Unbounded_String ("格式是旧版(" & Path & ";存的量不全:kin 1 没存不动的眼的像素残差,kin 2 没存每根轴是转是走,kin 3 没存关节到过的范围)");
         return;
      end if;
      if K.Worlds.Is_Empty or else Length (K.Key) = 0 then
         Note := To_Unbounded_String ("文件不全(" & Path & ")");
         return;
      end if;
      --  核对用的图
      for A in 0 .. Natural (K.Worlds.Length) - 1 loop
         declare
            D : Sweep_Data := K.Ds (A);
            Im : Plug.Cam;
            Okb : Boolean := False;
         begin
            if Ada.Directories.Exists (Path & "_arm" & Codec.Img (A) & ".bmp") then
               Codec.Read_BMP (Path & "_arm" & Codec.Img (A) & ".bmp", Im.RGB, Im.W, Im.H, Okb);
            end if;
            if not Okb then
               Note := To_Unbounded_String ("第" & Codec.Img (A + 1) & " 只手核对用的图读不了");
               return;
            end if;
            D.Imgs.Append (Im);
            K.Ds.Replace_Element (A, D);
         end;
      end loop;
      if K.World_Cam >= 0 then
         declare
            D : Sweep_Data := K.Ds (0);
            Okb : Boolean := False;
         begin
            if Ada.Directories.Exists (Path & "_world.bmp") then
               Codec.Read_BMP (Path & "_world.bmp", D.World_Img.RGB, D.World_Img.W, D.World_Img.H, Okb);
            end if;
            if not Okb then
               Note := To_Unbounded_String ("不动的眼核对用的图读不了");
               return;
            end if;
            K.Ds.Replace_Element (0, D);
         end;
      end if;
      Ok := True;
      Note := To_Unbounded_String (Codec.Img (Natural (K.Worlds.Length)) & " 只手的运动学和世界、不动的眼(第" & Integer'Image (K.World_Cam) & " 台)、板 "
                                   & Codec.Img (Natural (K.Board.Length)) & " 个点");
   exception
      when others =>
         if Is_Open (Fi) then
            Close (Fi);
         end if;
         Ok := False;
         Note := To_Unbounded_String ("文件读不了(" & Path & ")");
   end Load_Kin;

   function Same_View (Disp : Floats) return Boolean is
      package Sorting is new F64_Vectors.Generic_Sorting;
      D : Floats := Disp;
   begin
      if Natural (D.Length) < Min_Inl then
         return False;
      end if;
      Sorting.Sort (D);
      return D (Natural (D.Length) / 2) < Trip_Px;
   end Same_View;

   --  一对图(存的 → 此刻)问格点、往返 1 px 内的留下 ⇒ 每个留下的点挪了多少像素
   procedure View_Shift (Host : String; Port : Natural; A_Img, B_Img : Plug.Cam; Disp : out Floats; Err : out Unbounded_String) is
      Ia, Ib : Integer;
      Q : Instrument.Match_Vectors.Vector;
   begin
      Disp.Clear;
      Instrument.Frame_Put (Host, Port, A_Img.RGB, A_Img.W, A_Img.H, Ia, Err);
      Instrument.Frame_Put (Host, Port, B_Img.RGB, B_Img.W, B_Img.H, Ib, Err);
      if Ia < 0 or else Ib < 0 then
         return;
      end if;
      for Gyy in 0 .. Gy - 1 loop
         for Gxx in 0 .. Gx - 1 loop
            Q.Append (Instrument.Match_Pt'(U => (Long_Float (Gxx) + 0.5) * Long_Float (A_Img.W) / Long_Float (Gx),
                                           V => (Long_Float (Gyy) + 0.5) * Long_Float (A_Img.H) / Long_Float (Gy), others => <>));
         end loop;
      end loop;
      declare
         R : constant Instrument.Match_Vectors.Vector := Instrument.Match_Ids (Host, Port, Natural (Ia), Natural (Ib), Q, Err, Coarse => True, Back => True);
      begin
         if Natural (R.Length) = Natural (Q.Length) then
            for G in 0 .. Natural (Q.Length) - 1 loop
               if R (G).Bu >= 0.0 and then R (G).U >= 0.0 and then Geom.Norm ([R (G).Bu - Q (G).U, R (G).Bv - Q (G).V, 0.0]) < Trip_Px then
                  Disp.Append (Geom.Norm ([R (G).U - Q (G).U, R (G).V - Q (G).V, 0.0]));
               end if;
            end loop;
         end if;
      end;
   end View_Shift;

   procedure Check_Kin (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; K : Kin_Store; Host : String; Port : Natural;
                        Ok : out Boolean; Note : out Unbounded_String) is
      Gs : Ints;
      Qs : Plug.Floats_Vectors.Vector;
      Tol : Long_Float := Long_Float'Last;
      Moved : Long_Float := 0.0;
      function Med (D : Floats) return Long_Float is
         package Sorting is new F64_Vectors.Generic_Sorting;
         X : Floats := D;
      begin
         if X.Is_Empty then
            return -1.0;
         end if;
         Sorting.Sort (X);
         return X (Natural (X.Length) / 2);
      end Med;
   begin
      Ok := False; Note := Null_Unbounded_String;
      --  存的每只手:读数组、眼都得在这具身体上
      for A in 0 .. Natural (K.Worlds.Length) - 1 loop
         if K.Worlds (A).Valid then
            if K.Worlds (A).Group >= Natural (F.Joints.Length) or else K.Eyes (A) < 0 or else Natural (K.Eyes (A)) >= Natural (F.Cams.Length) then
               Note := To_Unbounded_String ("第" & Codec.Img (A + 1) & " 只手的读数组 / 眼这具身体上没有");
               return;
            end if;
            declare
               G : constant Natural := K.Worlds (A).Group;
               Q0 : constant Floats := K.Worlds (A).Model.Q0;
            begin
               for J in 0 .. Natural'Min (Natural (Q0.Length), Natural (F.Joints (G).Length)) - 1 loop
                  declare
                     Dq : constant Long_Float := abs (Q0 (J) - F.Joints (G) (J));
                  begin
                     Moved := Long_Float'Max (Moved, Dq);
                     if Dq > 0.0 then
                        Tol := Long_Float'Min (Tol, Third * Dq);   --  到了 = 差不到这一下要走的三分之一(比例,同开机自检)
                     end if;
                  end;
               end loop;
               Gs.Append (G); Qs.Append (Q0);
            end;
         end if;
      end loop;
      --  回到存的参照读数(已经在那儿 = 差不过关节读数的静止噪声,不动)
      if Moved > 3.0 * M.Joint_Noise then   --  3 倍静止噪声(统计常数)
         declare
            Dl : Table.Vec;
            Fr : Natural;
            Okg : Boolean;
         begin
            Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okg, Groups => Gs, Qs => Qs, Tol => (if Tol < Long_Float'Last then Tol else 0.0));
            if not Okg then
               Note := To_Unbounded_String ("回存的参照读数时线断了 / 走不到");
               return;
            end if;
         end;
      end if;
      declare
         Ok2 : Boolean;
      begin
         Selfmap.Idle (L, F, 2, Ok2);   --  画面比读数晚 1 拍:停两拍再拍(次数)
         if not Ok2 then
            Note := To_Unbounded_String ("停稳时线断了");
            return;
         end if;
      end;
      Ok := True;
      Append (Note, "回到存的参照读数(最多差 " & Codec.Fmt (Moved, 6) & ")");
      for A in 0 .. Natural (K.Worlds.Length) - 1 loop
         if K.Worlds (A).Valid then
            declare
               Disp : Floats;
               Err : Unbounded_String;
               E : constant Natural := Natural (K.Eyes (A));
            begin
               View_Shift (Host, Port, K.Ds (A).Imgs (0), F.Cams (E), Disp, Err);
               Append (Note, " · 第" & Codec.Img (A + 1) & " 只手的眼(第" & Codec.Img (E) & " 台)和存的图配上 " & Codec.Img (Natural (Disp.Length)) & " 个点、位移中位 "
                       & Codec.Fmt (Med (Disp), 2) & " px ⇒ " & (if Same_View (Disp) then "没动" else "动了"));
               Ok := Ok and then Same_View (Disp);
            end;
         end if;
      end loop;
      if K.World_Cam >= 0 then
         if Natural (K.World_Cam) >= Natural (F.Cams.Length) then
            Note := Note & " · 存的不动的眼这具身体上没有";
            Ok := False;
         else
            declare
               Disp : Floats;
               Err : Unbounded_String;
            begin
               View_Shift (Host, Port, K.Ds (0).World_Img, F.Cams (Natural (K.World_Cam)), Disp, Err);
               Append (Note, " · 不动的眼(第" & Integer'Image (K.World_Cam) & " 台)和存的图配上 " & Codec.Img (Natural (Disp.Length)) & " 个点、位移中位 "
                       & Codec.Fmt (Med (Disp), 2) & " px ⇒ " & (if Same_View (Disp) then "没挪" else "挪了"));
               Ok := Ok and then Same_View (Disp);
            end;
         end if;
      end if;
   end Check_Kin;

   procedure Dump_Kin (Dump : String; K : Kin_Store) is
      use Ada.Text_IO;
      Fo : File_Type;
   begin
      if Dump = "" then
         return;
      end if;
      for A in 0 .. Natural (K.Worlds.Length) - 1 loop
         declare
            W : constant Arm_World := K.Worlds (A);
         begin
            Create (Fo, Out_File, Dump & "/kinem_arm" & Codec.Img (A) & ".txt");
            Put_Line (Fo, "arm " & Codec.Img (A) & " n " & Codec.Img (W.Model.N) & " f " & Codec.Fmt (W.Model.F, 6) & " cx " & Codec.Fmt (W.Model.Cx, 3) & " cy " & Codec.Fmt (W.Model.Cy, 3));
            Put (Fo, "q0");
            for X of W.Model.Q0 loop
               Put (Fo, " " & F9 (X));
            end loop;
            New_Line (Fo);
            for J in 0 .. W.Model.N - 1 loop
               Put_Line (Fo, "axis " & Codec.Img (J) & " " & F9 (W.Model.Ax (J).W (0)) & " " & F9 (W.Model.Ax (J).W (1)) & " " & F9 (W.Model.Ax (J).W (2)) & " "
                         & F9 (W.Model.Ax (J).P (0)) & " " & F9 (W.Model.Ax (J).P (1)) & " " & F9 (W.Model.Ax (J).P (2)) & " " & Kind_Word (W.Model.Ax (J)));
            end loop;
            for P of W.Model.Eye loop   --  长在眼上的像素(身体文件不存 ⇒ 读回来的没有)
               Put_Line (Fo, "eye " & Codec.Fmt (P.U, 3) & " " & Codec.Fmt (P.V, 3));
            end loop;
            Close (Fo);
            if A > 0 then
               Create (Fo, Out_File, Dump & "/align_arm" & Codec.Img (A) & ".txt");
               Put (Fo, "S " & F9 (W.S) & " R");
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     Put (Fo, " " & F9 (W.Ra (I, J)));
                  end loop;
               end loop;
               Put_Line (Fo, " T " & F9 (W.Ta (0)) & " " & F9 (W.Ta (1)) & " " & F9 (W.Ta (2)));
               Close (Fo);
            end if;
         end;
      end loop;
      Create (Fo, Out_File, Dump & "/world.txt");
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Put (Fo, F9 (K.Rw (I, J)) & " ");
         end loop;
      end loop;
      Put_Line (Fo, F9 (K.O (0)) & " " & F9 (K.O (1)) & " " & F9 (K.O (2)));
      Close (Fo);
      if K.Fixed_Eye.Valid then
         --  存的不动的眼是世界系的(对齐交出来时已按 Rw、O 换过);落盘同对齐那份 = 第一只手的系:X_手 = Rwᵀ X_世界 + O
         declare
            Pos : constant Geom.V3 := Geom.Ap (Geom.Tr (K.Rw), K.Fixed_Eye.Pos);
            Rc : constant Geom.M3 := Geom.Mul (Geom.Tr (K.Rw), K.Fixed_Eye.R_Ce);
         begin
            Create (Fo, Out_File, Dump & "/fixed_eye.txt");
            Put (Fo, "f " & Codec.Fmt (K.Fixed_Eye.F, 6) & " cx " & Codec.Fmt (K.Fixed_Eye.Cx, 3) & " cy " & Codec.Fmt (K.Fixed_Eye.Cy, 3) & " rms " & Codec.Fmt (K.Fixed_Eye.Rms, 4) & " used 0 of 0 pos "
                 & F9 (Pos (0) + K.O (0)) & " " & F9 (Pos (1) + K.O (1)) & " " & F9 (Pos (2) + K.O (2)) & " R");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Put (Fo, " " & F9 (Rc (I, J)));
               end loop;
            end loop;
            New_Line (Fo);
            Close (Fo);
         end;
      end if;
   exception
      when others =>
         if Is_Open (Fo) then
            Close (Fo);
         end if;
   end Dump_Kin;
end Jointboot;
