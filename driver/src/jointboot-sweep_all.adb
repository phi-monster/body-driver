separate (Jointboot)
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
            Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, W), V => Kinem.Grid_V (Gyy, H), others => <>));
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
      Px_Lo, Px_Hi : Floats;                   --  每个关节往负 / 往正那一段量的 Px_Per(几个关节一起动的格子按它定每个关节走多远;0 = 没量到)
      Tgt : Floats;
      K : Natural := 0;
      Done : Boolean := True;
      Why : Unbounded_String;
      Ids : Ints;                              --  每一格在仪器那边的编号
      Heads : Head_Vectors.Vector;
      --  ── 扫描时身前不一定是空的(路 1,10-01 P8A:右臂扫第 2 个关节时撞上柜子把手,第 2 个关节一格就被带偏 1.24 rad、卡在那儿,
      --  后面每一段都从卡住的地方出发,22 格里一大半是卡着的,运动学没量成)──
      Last_Ok : Floats;                        --  这一段里最后一格干净的目标(段头 = 起点):碰上了就先退回这儿
      Collided : Boolean := False;             --  碰上过东西、还没确认退回了起点:下一段 / 下一格之前先退回去
      Stuck : Boolean := False;                --  退不回起点(一根一根往回挪也挪不动了):这只手不再扫,照实说
      Retry : Boolean := False;                --  这一段头一格就碰上了 ⇒ 退回去,按小一半的步子重来这一段
      Min_Step : Long_Float := 0.0;            --  步子最小缩到多少:认手时这组读数一起转多少就看得见(Arm_Info.Probe,量的)
      Kept_Lo, Kept_Hi : Ints;                 --  每个关节往负 / 往正留下了几格干净的
      Hit_Lo, Hit_Hi : Bools;                  --  每个关节往负 / 往正最后是碰上东西停的(不是走满、不是关节到头)
   end record;
   St : array (0 .. Natural'Max (1, Na) - 1) of Arm_State;
   --  这一格在仪器那边存成了没有
   function Sa_Id_Ok (A, Fr : Natural) return Boolean is (Fr < Natural (St (A).Ids.Length) and then St (A).Ids (Fr) >= 0);
   --  每格画面挪第 A 只手那只眼画幅宽的 1/5(比例,无量纲;每个方向停 3 格,转开的总量同原来 5 格 × 1/10:V1B14 挑格回放,停 3 格最大 0.73 mm)。
   --  按每只手自己的画幅(原来一个数给所有手、取的是最后一只手的画幅:几只手的眼画幅不一样时别的手每格挪错)
   function Gw_Of (A : Natural) return Long_Float is (Long_Float (St (A).W) / 5.0);
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
      --  存到仪器那边;起点 ↔ 这一格:一段的头一格当场配(下一格的步子按它定),别的交给后台配。
      --  这只手的眼这一拍没有画面(插头留的空位,09-30):这一格照样占位(读数、帧号都留着,和别的表对得齐),只是没有图 ⇒ 编号记 −1,
      --  配点、拟合都按"这一格没图"跳过它(同上传失败),照实记一笔
      if Plug.Has_Picture (F.Cams (Sa.Cam)) then
         Instrument.Frame_Put (Host, Port, F.Cams (Sa.Cam).RGB, Sa.W, Sa.H, Id, Err);
      else
         Id := -1;
         Say ("第" & Codec.Img (A + 1) & " 只手这一格(关节 " & Codec.Img (J) & ")它的眼这一拍没有画面 ⇒ 这一格没图,配点时跳过");
      end if;
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
            if J < Natural (Sa.Px_Lo.Length) then
               if Dd < 0 then
                  Sa.Px_Lo.Replace_Element (J, Sa.Px_Per);
               else
                  Sa.Px_Hi.Replace_Element (J, Sa.Px_Per);
               end if;
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
      if Dump /= "" and then Id >= 0 then
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
   --  这只手此刻每个关节离起点都不到"画面挪不到 1 像素"那一档(Kinem.Clean_Tol,同收格子)
   function At_Start (A : Natural) return Boolean is
      Sa : Arm_State renames St (A);
      Ct : constant Long_Float := Kinem.Clean_Tol (Long_Float (Sa.W));
   begin
      for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
         if X >= Natural (F.Joints (Sa.G).Length) or else abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)) > Ct then
            return False;
         end if;
      end loop;
      return True;
   end At_Start;
   --  碰上过东西的手先退回起点(别的手停在各自此刻的目标上,不动):整组回起点;回不去(挡着的东西卡在原路上)⇒ 一根一根往回挪,
   --  离起点最远的先挪;一整轮挪下来"在起点上的关节"一根都没多 ⇒ 不再挪 = 卡住了:这只手不再扫(Stuck),照实说卡在哪。
   --  (在起点上的关节每一轮只许多、不许持平,根数有限 ⇒ 最多挪关节数那么多轮)
   procedure Recover (A : Natural) is
      Sa : Arm_State renames St (A);
      Ct : constant Long_Float := Kinem.Clean_Tol (Long_Float (Sa.W));
      function Off_Of (X : Natural) return Long_Float is (abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)));
      function Home_Count return Natural is
         N : Natural := 0;
      begin
         for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
            if Off_Of (X) <= Ct then
               N := N + 1;
            end if;
         end loop;
         return N;
      end Home_Count;
   begin
      if not Sa.Live or else not Sa.Collided or else Sa.Stuck then
         return;
      end if;
      Sa.Collided := False;
      Sa.Tgt := Sa.Q0;
      Move_All (Ct);
      while Okc and then not At_Start (A) loop
         declare
            Before : constant Natural := Home_Count;
            Done_X : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
         begin
            loop
               declare
                  Far : Integer := -1;
                  Far_Off : Long_Float := Ct;
               begin
                  for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
                     if not Done_X (X) and then Off_Of (X) > Far_Off then
                        Far := X; Far_Off := Off_Of (X);
                     end if;
                  end loop;
                  exit when Far < 0;
                  Done_X.Replace_Element (Natural (Far), True);
                  Sa.Tgt := F.Joints (Sa.G);                       --  别的关节就停在此刻的读数上
                  Sa.Tgt.Replace_Element (Natural (Far), Sa.Q0 (Natural (Far)));
                  Move_All (Ct);
                  exit when not Okc;
               end;
            end loop;
            exit when Home_Count <= Before;
         end;
      end loop;
      if Okc and then not At_Start (A) then
         Sa.Stuck := True;
         Sa.Tgt := F.Joints (Sa.G);   --  卡住了就停在此刻的读数上,不再往挡着的东西里压
         declare
            T : Unbounded_String;
         begin
            for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
               if Off_Of (X) > Ct then
                  Append (T, " 第" & Codec.Img (X) & " 个关节停在 " & Codec.Fmt (F.Joints (Sa.G) (X), 4) & "(起点 " & Codec.Fmt (Sa.Q0 (X), 4) & ")");
               end if;
            end loop;
            Say ("  第" & Codec.Img (A + 1) & " 只手碰上东西以后退不回起点(整组退、一根一根退都挪不动了):" & To_String (T) & " ⇒ 这只手不再扫,后面的格子它都没有");
         end;
      elsif Okc then
         Sa.Tgt := Sa.Q0;
         Say ("  第" & Codec.Img (A + 1) & " 只手碰上东西以后退回了起点(每个关节离起点都不到 " & Codec.Fmt (Ct, 5) & ")");
      end if;
   end Recover;
   --  这一段从起点重新开始(步子按调用方给的;碰上过东西、头一格就碰上的那一段按小一半的步子重来时步子不重置)
   procedure Start_Segment (A, J : Natural) is
      Sa : Arm_State renames St (A);
   begin
      Sa.Off := 0.0; Sa.K := 0; Sa.Tgt := Sa.Q0; Sa.Q_Prev := F.Joints (Sa.G) (J); Sa.Px_Per := 0.0;
      Sa.Why := To_Unbounded_String ("走满 3 格");
      Sa.Last_Ok := Sa.Q0;
   end Start_Segment;
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
            Sa.Px_Lo := Zeros (Natural (Sa.Q0.Length)); Sa.Px_Hi := Zeros (Natural (Sa.Q0.Length));
            Sa.W := F.Cams (Sa.Cam).W; Sa.H := F.Cams (Sa.Cam).H;
            D.W := Sa.W; D.H := Sa.H;
            D.Has_Lo := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
            D.Has_Hi := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
            D.Step_Lo := F64_Vectors.To_Vector (0.0, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
            D.Step_Hi := F64_Vectors.To_Vector (0.0, Ada.Containers.Count_Type (Natural (Sa.Q0.Length)));
            Ds.Replace_Element (A, D);
            Nj := Natural'Max (Nj, Natural (Sa.Q0.Length));
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
   for A in 0 .. Na - 1 loop
      if St (A).Live then
         declare
            Sa : Arm_State renames St (A);
            Nq : constant Ada.Containers.Count_Type := Ada.Containers.Count_Type (Natural (Sa.Q0.Length));
         begin
            Sa.Min_Step := Arms (A).Probe;
            Sa.Kept_Lo := Int_Vectors.To_Vector (0, Nq); Sa.Kept_Hi := Int_Vectors.To_Vector (0, Nq);
            Sa.Hit_Lo := Bool_Vectors.To_Vector (False, Nq); Sa.Hit_Hi := Bool_Vectors.To_Vector (False, Nq);
         end;
      end if;
   end loop;
   for J in 0 .. Nj - 1 loop
      for Dd in -1 .. 1 loop
         if Dd /= 0 then
            for A in 0 .. Na - 1 loop
               declare
                  Sa : Arm_State renames St (A);
               begin
                  Recover (A);   --  上一段碰上过东西:先退回起点(退不回 ⇒ Stuck,不再扫)
                  Sa.Done := not Sa.Live or else Sa.Stuck or else J >= Natural (Sa.Q0.Length);
                  if not Sa.Done then
                     --  起步 = 读数量级的 3%(比例,无量纲),按画面挪动放大。目标里别的关节都回起点:上一段转完不单独回起点,
                     --  回去和这一段的头一格是同一个动作(到没到按这一格的三分之一判,所有关节一起看)
                     Sa.Step := 0.03 * Long_Float'Max (1.0, abs Sa.Q0 (J));
                     Start_Segment (A, J);
                  end if;
               end;
            end loop;
            loop   --  一段走完;头一格就碰上东西的手退回去、步子小一半,这一段只为它们再走一遍,直到没有要重来的
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
                                 --  扫的这根冲过了这一格的目标多少(沿命令的方向;负 = 没到):被别的东西带着走的那一种"没对上"
                                 Over : constant Long_Float := Long_Float (Dd) * (F.Joints (Sa.G) (J) - Sa.Tgt (J));
                                 Hit : Boolean;
                              begin
                                 for X in 0 .. Natural (Sa.Q0.Length) - 1 loop
                                    if X /= J and then abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)) > Pushed then
                                       Pushed := abs (F.Joints (Sa.G) (X) - Sa.Q0 (X)); Kp := X;
                                    end if;
                                 end loop;
                                 --  碰上东西 = 命令和读数对不上、又不是"这根自己到头":别的关节被顶偏超过这一格的三分之一(同 Sweep_Stops),
                                 --  或者扫的这根冲过了目标、超过同一个三分之一(被挡着的东西带着走:P8A 第 2 只手命令 −0.15、一格冲到 −1.237)
                                 Hit := (Sweep_Stops (Got, Pushed, Sa.Step) and then not Sweep_Stop_Is_End (Got, Pushed, Sa.Step))
                                        or else Over > Sa.Step * Third;
                                 if Hit then
                                    --  这一格不进运动学(读数是被挡着的东西顶出来的、画面可能正被它挡着);目标立刻改回这一段最后一格干净的地方 ——
                                    --  别的手接着扫的每一条命令都把它往回带;下一段之前再核一遍退没退到起点(Recover)
                                    Sa.Why := To_Unbounded_String ("碰上东西了:" & (if Over > Sa.Step * Third
                                                                   then "扫的这根冲过了目标 " & Codec.Fmt (Over, 4)
                                                                   else "第" & Codec.Img (Kp) & " 个关节被顶偏 " & Codec.Fmt (Pushed, 4))
                                                                   & "(这一格命令 " & Codec.Fmt (Sa.Step, 4) & ";这一格不进运动学,不记界)");
                                    Sa.Done := True;
                                    Sa.Collided := True;
                                    Sa.Tgt := Sa.Last_Ok;
                                    if Dd < 0 then
                                       Sa.Hit_Lo.Replace_Element (J, True);
                                    else
                                       Sa.Hit_Hi.Replace_Element (J, True);
                                    end if;
                                    --  这一边一格干净的都还没留下 ⇒ 按小一半的步子再来(不小过认手时看得见的那一推:再小画面里分不出动没动)
                                    if Sa.K = 1 and then Sa.Step / Grow >= Sa.Min_Step then
                                       Sa.Retry := True;
                                    end if;
                                 else
                                    Keep (A, J, Dd, Sa.K);
                                    Sa.Last_Ok := Sa.Tgt;
                                    if Dd < 0 then
                                       Sa.Kept_Lo.Replace_Element (J, Sa.Kept_Lo (J) + 1);
                                    else
                                       Sa.Kept_Hi.Replace_Element (J, Sa.Kept_Hi (J) + 1);
                                    end if;
                                    if Sa.K = 1 then
                                       Sa.Heads.Append (Head'(Frame => Natural (Ds (A).Frames.Length) - 1, Joint => J));
                                    end if;
                                    if Sweep_Stops (Got, Pushed, Sa.Step) then
                                       --  只剩"这个关节自己停住、别的关节没被顶偏"= 它这一边的界(反解不过这儿);碰上东西的那一种在上面
                                       --  (owner 09-28 到过的范围:"被桌子挡住这种'转不过去'本来就不是关节尽头")
                                       Sa.Why := To_Unbounded_String ("关节到头(命令 " & Codec.Fmt (Sa.Step, 4) & ",实到 " & Codec.Fmt (Got, 4) & ";这一边记界)");
                                       Sa.Done := True;
                                       if J < Natural (Ds (A).Has_Lo.Length) then
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
                                          Sa.Step := Sa.Step * Long_Float'Max (0.5, Long_Float'Min (Ramp, Gw_Of (A) / (Sa.Step * Sa.Px_Per)));
                                       end if;
                                       Sa.Q_Prev := F.Joints (Sa.G) (J);
                                    end if;
                                 end if;
                                 if Sa.Done then
                                    Say ("  第" & Codec.Img (A + 1) & " 只手第" & Codec.Img (J) & " 个关节往" & (if Dd > 0 then "正" else "负") & "转了 " & Codec.Img (Sa.K)
                                         & " 格(累计 " & Codec.Fmt (Sa.Off, 3) & ")⇒ 停:" & To_String (Sa.Why)
                                         & (if Sa.Retry then " ⇒ 退回去,步子小一半再来" else ""));
                                    --  这一边最后一格命令的步子 = 往到过的范围外最多走的那一步(到过的范围:身体开机时一条命令走过的量)
                                    if J < Natural (Ds (A).Step_Lo.Length) and then not Hit then
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
               exit when not Okc;
               declare
                  Again : Boolean := False;
               begin
                  for A in 0 .. Na - 1 loop
                     declare
                        Sa : Arm_State renames St (A);
                     begin
                        if Sa.Retry then
                           Sa.Retry := False;
                           Recover (A);
                           if not Sa.Stuck then
                              Sa.Step := Sa.Step / Grow;
                              Start_Segment (A, J);
                              Sa.Done := False;
                              Again := True;
                           end if;
                        else
                           Sa.Done := True;   --  别的手这一段已经走完,不再跟着走
                        end if;
                     end;
                  end loop;
                  exit when not Again;
               end;
            end loop;
         end if;
      end loop;
   end loop;
   --  ② 几个关节一起动的格子:最多那只手的关节数 + 1 格(Multi_Cells),第几格每个关节往哪边 = Hadamard 矩阵那一行(Multi_Up),
   --  走多远 = 预计画面挪一格(Gw ÷ 它那一边头一格量的 Px_Per,不越过它这次扫到过的那一头;Multi_Offset),每格和起点、和上一格配。
   --  只一个关节一个关节扫,各轴离眼远近的比例只靠相邻关节头一格那几对连,约束太弱:V1B6 / V1B8 驱动自己解的运动学在扫描格上
   --  中位 4–8 mm、最大 26–116 mm(像素残差却只有 0.2 px)。V1B2 离线能过线,靠的是板停 —— 几个关节一起动的姿势把各轴的比例绑在一起。
   --  关节比最多那只少的手多走几格(Hadamard 往下几行,照样分得开)
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
      for Cb in 1 .. Multi_Cells (Nj) loop
         declare
            Tol : Long_Float := Long_Float'Last;
         begin
            for A in 0 .. Na - 1 loop
               Recover (A);   --  上一格碰上东西的手先退回起点(退不回 ⇒ Stuck:不再走这些格)
            end loop;
            for A in 0 .. Na - 1 loop
               if St (A).Live and then not St (A).Stuck then
                  for Jx in 0 .. Natural (St (A).Q0.Length) - 1 loop
                     declare
                        Sa : Arm_State renames St (A);
                        Tq : constant Long_Float :=
                          (if Multi_Up (Cb - 1, Jx) then Sa.Q0 (Jx) + Multi_Offset (Gw_Of (A), Sa.Px_Hi (Jx), Hi (A) (Jx) - Sa.Q0 (Jx))
                           else Sa.Q0 (Jx) - Multi_Offset (Gw_Of (A), Sa.Px_Lo (Jx), Sa.Q0 (Jx) - Lo (A) (Jx)));
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
               if St (A).Live and then not St (A).Stuck then
                  declare
                     Miss : Long_Float := 0.0;
                     Span : Long_Float := 0.0;
                  begin
                     for Jx in 0 .. Natural (St (A).Q0.Length) - 1 loop
                        Miss := Long_Float'Max (Miss, abs (F.Joints (St (A).G) (Jx) - St (A).Tgt (Jx)));
                        Span := Long_Float'Max (Span, abs (St (A).Tgt (Jx) - St (A).Q0 (Jx)));
                     end loop;
                     --  有关节没跟上(差超过它这次要走的三分之一 = 碰上东西 / 到头,同单关节那一段的 Third)⇒ 这一格不要;
                     --  目标改回起点(别的手走下一格的命令把它往回带),下一格之前再核一遍退没退到(Recover)
                     if Miss <= Span * Third then
                        Keep (A, 0, 0, Cb, Multi => True);   --  起点 ↔ 这一格、上一格 ↔ 这一格都在 Keep 里交给配点
                     else
                        Say ("  第" & Codec.Img (A + 1) & " 只手几个关节一起动的第" & Codec.Img (Cb) & " 格:有关节没跟上(差 " & Codec.Fmt (Miss, 4)
                             & ")⇒ 不要这一格,先退回起点再走下一格");
                        St (A).Collided := True;
                        St (A).Tgt := St (A).Q0;
                     end if;
                  end;
               end if;
            end loop;
         end;
      end loop;
      for A in 0 .. Na - 1 loop
         if St (A).Live and then not St (A).Stuck then
            St (A).Tgt := St (A).Q0;
         end if;
      end loop;
      Move_All (0.0);
      --  扫完回起点:回不去的手照样按碰上东西退一遍(退不回就照实说卡在哪)
      for A in 0 .. Na - 1 loop
         if St (A).Live and then not St (A).Stuck and then not At_Start (A) then
            St (A).Collided := True;
            Recover (A);
         end if;
      end loop;
   end;
   --  扫不全的照实说:哪只手、哪根轴、哪一边一格干净的都没留下(碰上东西 / 卡住了);两边都没有 = 这根轴没扫成
   for A in 0 .. Na - 1 loop
      if St (A).Live then
         declare
            Sa : Arm_State renames St (A);
            T : Unbounded_String;
            Short : Boolean := False;
         begin
            for J in 0 .. Natural (Sa.Q0.Length) - 1 loop
               declare
                  Lo_N : constant Integer := Sa.Kept_Lo (J);
                  Hi_N : constant Integer := Sa.Kept_Hi (J);
               begin
                  if Lo_N = 0 or else Hi_N = 0 or else Sa.Hit_Lo (J) or else Sa.Hit_Hi (J) then
                     Short := Short or else Lo_N = 0 or else Hi_N = 0;
                     Append (T, " · 第" & Codec.Img (J) & " 个关节 往负 " & Codec.Img (Lo_N) & " 格" & (if Sa.Hit_Lo (J) then "(碰上东西)" else "")
                             & "、往正 " & Codec.Img (Hi_N) & " 格" & (if Sa.Hit_Hi (J) then "(碰上东西)" else "")
                             & (if Lo_N = 0 and then Hi_N = 0 then " ⇒ 这根轴没扫成" elsif Lo_N = 0 or else Hi_N = 0 then " ⇒ 只扫了一边" else ""));
                  end if;
               end;
            end loop;
            if Sa.Stuck then
               Say ("  第" & Codec.Img (A + 1) & " 只手扫描没扫全:碰上东西卡住了,卡住以后的格子都没有" & To_String (T));
            elsif Short then
               Say ("  第" & Codec.Img (A + 1) & " 只手扫描没扫全(碰上东西的那一边一格干净的都没留下)" & To_String (T));
            elsif Length (T) > 0 then
               Say ("  第" & Codec.Img (A + 1) & " 只手扫描里碰上过东西(碰上的那几格不进运动学,别的照扫完了)" & To_String (T));
            end if;
         end;
      end if;
   end loop;
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
