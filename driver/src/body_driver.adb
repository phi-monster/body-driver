--  body_driver --listen <口> [--eye host:port]:装上之后跑的唯一一条命令。
--  开机:认布局 → 量身体(逐通道推、合空)→ 循环:看 → 列块 → 问脑 → 执行 → 报。不读不写任何标定文件。
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Calendar;
with Ada.Directories;
with Ada.Command_Line;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Codec;
with Plug;
with Selfmap;
with Zone;
with World;
with Memory;
with Act;
with Bodyfile;
with Bytes;
with Picture;
with Schema;
with Chan;
with Table;
with Jointboot;
with Kinem;
with Geom;
procedure Body_Driver is
   Port : Natural := 0;
   Body_Path : Unbounded_String;   --  身体文件(--in/--out;同一具身体越用越强)
   --  脑的默认端点(接线协议,不是身体量)
   Eye : Unbounded_String := To_Unbounded_String ("127.0.0.1:8079");
   Inst : Unbounded_String := To_Unbounded_String (Codec.Env ("BL_INST"));   --  仪器进程 host:port(空 = 没配)
   L : Plug.Link;
   F : Plug.Frame;
   C : Act.Context;
   Ok : Boolean;
   Kin_Eyes : Bytes.Ints;          --  开机前半段认出来的:每只(量成了运动学的)手上的眼
   Kin_World_Cam : Integer := -1;  --  开机前半段认出来的世界相机(不长在手上的;没有 = -1)
   Kin_Fixed : Geom.Cam_Geo;       --  开机前半段按第一只手的桌面点解出的不动的眼(世界系;Valid = False 就是没解成)
   --  开机前半段交给后半段的几何(V1b 09-27):每台相机一份(手上那只眼 = 运动学的焦距、主点;不动的眼 = 对齐量的)、标定板、世界系的桌面、
   --  不动的眼那一刻的画面(板上的点在它里面的像素就是在这张图里配的)
   Kin_Geo : Geom.Geo_Vectors.Vector;
   Kin_Board : Geom.Scene_Pt_Vectors.Vector;
   Kin_Plane_Pt, Kin_Plane_N : Geom.V3 := [0.0, 0.0, 0.0];
   Kin_Plane_Rms : Long_Float := 0.0;
   Kin_Ref : Plug.Cam;
   Front_Reloaded : Boolean := False;   --  开机前半段是按身体文件旁边存的装回的(核对过):后面按同一个世界单位记的量(身体图、握区、指尖)才照用
   I : Natural := 1;
   Order : constant String := Codec.Env ("BL_ORDER");
begin
   while I <= Ada.Command_Line.Argument_Count loop
      declare
         A : constant String := Ada.Command_Line.Argument (I);
      begin
         if A = "--listen" and then I < Ada.Command_Line.Argument_Count then
            Port := Natural'Value (Ada.Command_Line.Argument (I + 1)); I := I + 1;
         elsif A = "--eye" and then I < Ada.Command_Line.Argument_Count then
            Eye := To_Unbounded_String (Ada.Command_Line.Argument (I + 1)); I := I + 1;
         elsif A = "--inst" and then I < Ada.Command_Line.Argument_Count then
            Inst := To_Unbounded_String (Ada.Command_Line.Argument (I + 1)); I := I + 1;
         elsif (A = "--in" or else A = "--out") and then I < Ada.Command_Line.Argument_Count then
            Body_Path := To_Unbounded_String (Ada.Command_Line.Argument (I + 1)); I := I + 1;   --  身体文件:装回、核对、合成、写回
         else
            Put_Line ("不认识的开关:" & A);
         end if;
      end;
      I := I + 1;
   end loop;
   if Codec.Env ("BL_EYE") /= "" then
      Eye := To_Unbounded_String (Codec.Env ("BL_EYE"));
   end if;
   if Port = 0 then
      Put_Line ("用法:body_driver --listen <端口> [--eye host:port] [--inst host:port]");
      Ada.Command_Line.Set_Exit_Status (2);
      return;
   end if;
   declare
      E : constant String := To_String (Eye);
      P : constant Natural := Ada.Strings.Fixed.Index (E, ":");
   begin
      if P > 0 then
         C.Eye_Host := To_Unbounded_String (E (E'First .. P - 1));
         C.Eye_Port := Natural'Value (E (P + 1 .. E'Last));
      else
         C.Eye_Host := Eye;
      end if;
   end;
   declare
      E : constant String := To_String (Inst);
      P : constant Natural := Ada.Strings.Fixed.Index (E, ":");
   begin
      if P > 0 then
         C.Inst_Host := To_Unbounded_String (E (E'First .. P - 1));
         C.Inst_Port := Natural'Value (E (P + 1 .. E'Last));
      else
         C.Inst_Host := Inst;
      end if;
   end;
   C.Look_Only := Codec.Env ("BL_LOOK") /= "";
   C.Dump_Dir := To_Unbounded_String (Codec.Env ("BL_DUMP"));
   if C.Dump_Dir /= "" then
      Codec.Make_Dir (To_String (C.Dump_Dir));
   end if;
   Plug.Boot (Port, L, Ok);
   if not Ok then
      Put_Line ("[装] 没接上,退出");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;
   if not Plug.Sense (L, F) then
      Put_Line ("[装] 取不到第一帧,退出");
      return;
   end if;
   Put_Line ("[装] 第一帧:" & Natural'Image (Natural (F.EE.Length)) & " 条臂 ·" & Natural'Image (Natural (F.Jaw.Length)) & " 个抓握通道 ·" &
             Natural'Image (Natural (F.Cams.Length)) & " 台相机" & (if F.Cams.Is_Empty then "" else "(" & Codec.Img (F.Cams (0).W) & "x" & Codec.Img (F.Cams (0).H) & (if F.Cams (0).Has_Depth then ",带深度" else ",无深度") & ")"));
   --  ── 开机前半段(V1b 第三步,2026-09-26):只用关节命令。身体报的"手在哪"驱动不读 ——
   --  认手认眼(每组关节一起转一小格)→ 每只有眼的手扫关节、两两配点、量运动学 → 两只手对到一个世界("上" = 桌面法向)→ 装上:
   --  从此每一帧手的位姿 = 按关节读数算出的腕眼位姿,位姿命令 = 在量到的关节限位里解关节目标。后面量身体的每一步都在这个世界里 ──
   declare
      M0 : Selfmap.Body_Map;
      Found : Jointboot.Arm_Vectors.Vector;
      Okj : Boolean;
      Ds : Jointboot.Sweep_Vectors.Vector;
      Worlds : Jointboot.Arm_World_Vectors.Vector;
      Css : Jointboot.Corr_Set_Vectors.Vector;
      Rw : Geom.M3;
      O : Geom.V3;
      Host : constant String := To_String (C.Inst_Host);
      Dump : constant String := To_String (C.Dump_Dir);
      Kin_Path : constant String := (if Body_Path /= "" then To_String (Body_Path) & ".kin.txt" else "");
      K : Jointboot.Kin_Store;
      Eyes_Of : Bytes.Ints;   --  每只手:长在它上面的相机
   begin
      M0.N_Cams := Natural (F.Cams.Length);
      Selfmap.Measure_Idle (L, F, M0, Okj);
      if not Okj then
         Put_Line ("[链] 量静止噪声时线断了,退出");
         return;
      end if;
      Put_Line ("[身] 静止噪声(开机前半段):关节读数 " & Codec.Fmt (M0.Joint_Noise, 6) & " · 各相机灰度地板 " & (if M0.Pic_Floor.Is_Empty then "-" else Codec.Img (M0.Pic_Floor (0))));
      --  ⑤ 装回:身体文件旁边存着前半段、钥匙对得上 ⇒ 每只手回到存的参照读数、拍一张和存的比;都没动 ⇒ 不扫描、不解。一项不过 ⇒ 从零量(不修补)
      if Kin_Path /= "" and then Ada.Directories.Exists (Kin_Path) then
         declare
            Loaded, Checked : Boolean := False;
            Note : Unbounded_String;
         begin
            Jointboot.Load_Kin (Kin_Path, K, Loaded, Note);
            if Loaded and then To_String (K.Key) /= Jointboot.Kin_Key (L, F) then
               Loaded := False;
               Note := To_Unbounded_String ("钥匙对不上(存的 " & To_String (K.Key) & " / 这具身体 " & Jointboot.Kin_Key (L, F) & ")");
            end if;
            Put_Line ("[装] 前半段存的(" & Kin_Path & "):" & To_String (Note));
            if Loaded then
               Jointboot.Check_Kin (L, F, M0, K, Host, C.Inst_Port, Checked, Note);
               Put_Line ("[装] 核对:" & To_String (Note) & (if Checked then " ⇒ 装回,不扫描" else " ⇒ 对不上,从零量"));
            end if;
            Front_Reloaded := Checked;
         end;
      end if;
      if Front_Reloaded then
         Worlds := K.Worlds; Ds := K.Ds; Rw := K.Rw; O := K.O; Kin_World_Cam := K.World_Cam; Eyes_Of := K.Eyes;
         Kin_Fixed := K.Fixed_Eye; Kin_Board := K.Board; Kin_Plane_Pt := K.Plane_Pt; Kin_Plane_N := K.Plane_N; Kin_Plane_Rms := K.Plane_Rms;
         Jointboot.Dump_Kin (Dump, K);
         Jointboot.Remember_Kin (Kin_Path, K);
      else
      Jointboot.Find_Arms (L, F, M0, Found, Kin_World_Cam, Okj);
      if not Okj then
         Put_Line ("[身] 只用关节命令认不出一只手,量不了身体,退出");
         return;
      end if;
      --  有眼的几只手同时扫(一条命令带几组目标),扫的时候跟点仪器一路跟
      Jointboot.Sweep_All (L, F, M0, Found, Host, C.Inst_Port, Dump, Ds, Css, World_Cam => Kin_World_Cam);
      --  每只手各自解运动学(两只手的解互不相干 ⇒ 一只手一个线程)
      declare
         Ms : array (0 .. Natural (Found.Length) - 1) of Kinem.Model;
         Oks : array (0 .. Natural (Found.Length) - 1) of Boolean := [others => False];
         Notes : array (0 .. Natural (Found.Length) - 1) of Unbounded_String;
         --  解一只手的运动学要几百 KB 的栈(各轴的候选表)⇒ 每个线程给 64 MB(次数)
         task type Fit_Task with Storage_Size => 64 * 1024 * 1024 is
            entry Start (A : Natural);
         end Fit_Task;
         task body Fit_Task is
            Aa : Natural := 0;
         begin
            accept Start (A : Natural) do
               Aa := A;
            end Start;
            if Found (Aa).Eye >= 0 and then not Ds (Aa).Frames.Is_Empty then
               Jointboot.Fit_Arm (Aa, Ds (Aa), Css (Aa), Dump, Ms (Aa), Oks (Aa), Notes (Aa));
            end if;
         end Fit_Task;
         T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      begin
         declare
            Workers : array (0 .. Natural (Found.Length) - 1) of Fit_Task;
         begin
            for A in Workers'Range loop
               Workers (A).Start (A);
            end loop;
         end;
         for A in Notes'Range loop
            Put (To_String (Notes (A)));
         end loop;
         Put_Line ("[身] 📐 运动学解完(" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 0) & " 秒)");
         for A in 0 .. Natural (Found.Length) - 1 loop
            declare
               W : Jointboot.Arm_World;
            begin
               W.Group := Found (A).Group;
               W.Sweep := A;
               W.Model := Ms (A);
               W.Valid := Oks (A);
               if Found (A).Eye < 0 then
                  Put_Line ("[身] 📐 第" & Codec.Img (A + 1) & " 只手上没有眼 ⇒ 这一版量不了它的运动学(要一只看得见它的眼),先不用");
               elsif not Ds (A).Frames.Is_Empty then
                  --  记下的尽头、到过的范围、往外一步(到过的范围,09-29):发命令时反解只在到过的范围往外一步里解,问够不够得着只按尽头
                  Jointboot.Set_Ranges (Ds (A), W);
               end if;
               Worlds.Append (W);
            end;
         end loop;
      end;
      Jointboot.Align (Ds, Worlds, Css, Host, C.Inst_Port, Rw, O, Okj, Kin_Fixed, Kin_Board, Kin_Plane_Pt, Kin_Plane_N, Kin_Plane_Rms, Dump => Dump);
      if not Okj then
         Put_Line ("[身] 定不了世界(第一只手的眼没三角出桌面),量不了身体,退出");
         return;
      end if;
      for A in 0 .. Natural (Found.Length) - 1 loop
         Eyes_Of.Append (Found (A).Eye);
      end loop;
      if Kin_Path /= "" then
         K := (Key => To_Unbounded_String (Jointboot.Kin_Key (L, F)), Worlds => Worlds, Eyes => Eyes_Of, Ds => Ds, Rw => Rw, O => O, World_Cam => Kin_World_Cam,
               Fixed_Eye => Kin_Fixed, Board => Kin_Board, Plane_Pt => Kin_Plane_Pt, Plane_N => Kin_Plane_N, Plane_Rms => Kin_Plane_Rms);
         begin
            Jointboot.Save_Kin (Kin_Path, K);
            Jointboot.Remember_Kin (Kin_Path, K);
            Put_Line ("[装] 前半段存进 " & Kin_Path & "(下回核对过就不再扫;干活时关节到过的范围长了 / 记下尽头就写回)");
         exception
            when others =>
               Put_Line ("[装] 前半段存不进 " & Kin_Path);
         end;
      end if;
      end if;
      for A in 0 .. Natural (Worlds.Length) - 1 loop
         if Worlds (A).Valid then
            Kin_Eyes.Append (Eyes_Of (A));
         end if;
      end loop;
      --  每台相机的几何:手上那只眼 = 运动学量的焦距、主点;插头给的手的位姿就是这只眼的位姿 ⇒ 眼在手上不转、不偏;
      --  不动的眼 = 对齐量的(世界系);别的相机只有画幅中心当主点(量不了)
      for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
         declare
            G : Geom.Cam_Geo := Geom.No_Geo;
         begin
            G.Cx := 0.5 * Long_Float (F.Cams (Cm).W); G.Cy := 0.5 * Long_Float (F.Cams (Cm).H);   --  画幅中心(纯几何的一半)
            for A in 0 .. Natural (Worlds.Length) - 1 loop
               if Worlds (A).Valid and then Eyes_Of (A) = Integer (Cm) then
                  G.F := Worlds (A).Model.F; G.F_Meas := G.F; G.Cx := Worlds (A).Model.Cx; G.Cy := Worlds (A).Model.Cy;
                  G.R_Ce := Geom.Identity; G.Off := [0.0, 0.0, 0.0]; G.Valid := True;
               end if;
            end loop;
            if Kin_World_Cam = Integer (Cm) and then Kin_Fixed.F > 0.0 then
               G := Kin_Fixed; G.Valid := True; G.Fixed := True;
            end if;
            Kin_Geo.Append (G);
         end;
      end loop;
      if not Ds.Is_Empty then
         Kin_Ref := Ds (0).World_Img;
      end if;
      Jointboot.Install (Worlds, Rw, O, Joint_Noise => M0.Joint_Noise);
      if not Plug.Sense (L, F) then
         Put_Line ("[链] 装上以后取不到画面,退出");
         return;
      end if;
      Jointboot.Self_Check (L, F, M0, Ds, Dump);
      Put_Line ("[装] 开机前半段完:" & Codec.Img (Natural (F.EE.Length)) & " 只手的位姿按关节读数算(用了 " & Codec.Img (Plug.Steps (L)) & " 拍)");
   end;
   --  ── 量身体:先装回身体文件(钥匙 = 这具身体报的形状),推一下核对;对不上或没有 ⇒ 从零量;量到的合进历史再写回 ──
   declare
      Key : constant String := Bodyfile.Fingerprint (L, F);
      Stored : Selfmap.Body_Map;
      Stored_Hands : Zone.Hand_Vectors.Vector;
      Stored_Tables : Act.Effect_Vectors.Vector;
      Stored_Sch : Schema.Map;
      Note : Unbounded_String;
      Loaded : Boolean := False;
      Use_Stored : Boolean := False;
   begin
      if Body_Path /= "" and then Front_Reloaded then
         Loaded := Bodyfile.Load (To_String (Body_Path), Key, Stored, Stored_Hands, Stored_Tables, Stored_Sch, Note);
         Put_Line ("[装] " & To_String (Note));
      elsif Body_Path /= "" and then Ada.Directories.Exists (To_String (Body_Path)) then
         --  前半段从零量了 ⇒ 世界单位换了(运动学的单位每回不一样),身体文件里按旧单位记的量(通道步子、握区时手的位姿)不装回
         Put_Line ("[装] 前半段是从零量的(世界单位换了)⇒ 身体文件里按旧单位记的量不装回,从零量");
      end if;
      if Loaded and then not Bodyfile.Jaws_Recorded (Stored) then
         --  旧的身体文件没记每条臂几个抓握通道:不猜(原来一律当 1 个 ⇒ 五指手第 1 号往后的握区全丢,存盘又把少了的写回去)
         --  ⇒ 重量;存的历次读数照样合进来
         Put_Line ("[装] 这份身体文件没记每条臂几个抓握通道 ⇒ 要重量(存的历次读数照样合进来)");
      elsif Loaded then
         declare
            Ok_Body, Ok_Link : Boolean;
            Vn : Selfmap.String_Note;
         begin
            --  存的没有噪声地板图(那是当场的相机),先量一遍静止对再核 —— 和开机前半段同一种量法(Selfmap.Measure_Idle:
            --  只用两帧都收到了画面的静止对;09-30 原来这里自己拿一对帧算,某台相机那一拍没画面时越界,也是第二种量法)
            declare
               Tmp : Selfmap.Body_Map;
               Ok2 : Boolean;
            begin
               Selfmap.Measure_Idle (L, F, Tmp, Ok2);
               Stored.Floors := Tmp.Floors; Stored.Pic_Floor := Tmp.Pic_Floor;
            end;
            Selfmap.Verify (L, Stored, F, Ok_Body, Ok_Link, Vn);
            Put_Line ("[装] 核对身体:" & To_String (Vn.Text) & (if Ok_Body then " ⇒ 同一具身体,直接用" else " ⇒ 重量"));
            if not Ok_Link then
               Put_Line ("[装] 核对时线断了,退出");
               return;
            end if;
            Use_Stored := Ok_Body;
         end;
      end if;
      if Use_Stored then
         C.Map := Stored;
         C.Tables := Stored_Tables;
         C.Sch := Stored_Sch;   --  身体没变 ⇒ 身体图照用(位姿 → 手指在画面哪儿)
      else
         --  每只手"一步看得见" = 在它自己那只眼里画面挪 1 像素:平移 = 眼离桌面的高度(世界 z,桌面 z = 0)÷ 焦距,转动 = 1 ÷ 焦距 弧度
         declare
            Step_Px : Plug.Floats_Vectors.Vector;
         begin
            for A in 0 .. Natural (F.EE.Length) - 1 loop
               declare
                  Fa : constant Long_Float := (if A < Natural (Kin_Eyes.Length) and then Kin_Eyes (A) >= 0 and then Natural (Kin_Eyes (A)) < Natural (Kin_Geo.Length)
                                               then Kin_Geo (Natural (Kin_Eyes (A))).F else 0.0);
                  St : Bytes.Floats;
               begin
                  if Fa > 0.0 and then F.EE (A) (2) > 0.0 then
                     St.Append (F.EE (A) (2) / Fa); St.Append (1.0 / Fa);
                  end if;
                  Step_Px.Append (St);
               end;
            end loop;
            Selfmap.Measure (L, F, C.Map, Ok, Step_Px, Eyes => Kin_Eyes, World => Kin_World_Cam);
         end;
         if not Ok then
            Put_Line ("[身] 身体量不了,退出");
            return;
         end if;
         if Loaded then
            declare
               Merged : Selfmap.Body_Map;
               Replaced, Kept : Natural;
            begin
               Bodyfile.Merge (Stored, C.Map, Merged, Replaced, Kept);
               C.Map := Merged;
               Put_Line ("[装] 越用越强:和存的合成 ⇒ 换掉 " & Codec.Img (Replaced) & " 格,持平 " & Codec.Img (Kept) & " 格(历次取中位数,噪声地板只放大)");
            end;
         else
            C.Map.Measured_Times := 1;
            for Ch in 0 .. C.Map.Channels - 1 loop
               declare
                  Ha, Hd : Bytes.Floats;
               begin
                  Ha.Append (C.Map.Amp (Ch)); Hd.Append (C.Map.Delivered (Ch));
                  C.Map.Amp_Hist.Append (Ha); C.Map.Deliv_Hist.Append (Hd);
               end;
            end loop;
         end if;
      end if;
      --  握区:存的这只手若是在【同一个位姿】下合空量的(每通道差不过一个探针幅度),身体又核对没变 ⇒ 照用,不再合空;否则合空一次
      --  一条臂上有几个抓握通道是【量出来的】:两指手 1 个,五指手 5 个。每一个各合空一次,各成一个名词。
      --  C.Map.Jaws 这时一条臂一个数(从零量的是 Selfmap.Measure 数的;照用存的,前面已经核过身体文件记了这一项)⇒ 照它合空,不另设"至少一个"、
      --  不在缺了的时候当 1 个(原来是 `else 1`:身体文件不存这一项,装回以后五指手只剩第 0 号)
      for A in 0 .. C.Map.Arms - 1 loop
       for Jk in 0 .. C.Map.Jaws (A) - 1 loop
         declare
            H : Zone.Hand;
            Reuse : Boolean := False;
            Old : Integer := -1;   --  存的手里,哪一个是这条臂的这个通道
         begin
            for I in 0 .. Natural (Stored_Hands.Length) - 1 loop
               if Stored_Hands (I).Arm = A and then Stored_Hands (I).K = Jk then
                  Old := Integer (I);
               end if;
            end loop;
            if Use_Stored and then Old >= 0 and then A < Natural (F.EE.Length) then
               declare
                  Dv : constant Table.Vec := Chan.Delivered (Stored_Hands (Natural (Old)).Pose, F.EE (A));
                  Any_Zone : Boolean := False;
               begin
                  Reuse := True;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if abs Dv (K) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + K)) then
                        Reuse := False;
                     end if;
                  end loop;
                  for Zc of Stored_Hands (Natural (Old)).Zones loop
                     if Zc.Valid then
                        Any_Zone := True;
                     end if;
                  end loop;
                  Reuse := Reuse and then Any_Zone;
                  --  存的握区少了哪台相机的(H53 2026-09-23 实测:第 2 只手只存了 0、1 两台,它自己那只眼(第 2 台)没有 ⇒ 合爪方向、指头都量不出)
                  --  ⇒ 不照用,合空一次把每台相机的都量上
                  --  (装回时握区表按相机数补齐了空位,所以要看的是它自己那只眼的那一格量过没有,不是表有多长 —— H54 实测表长 3、第 2 台那格是空的)
                  declare
                     Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
                  begin
                     if Hc >= 0 and then (Natural (Hc) >= Natural (Stored_Hands (Natural (Old)).Zones.Length)
                                          or else not Stored_Hands (Natural (Old)).Zones (Natural (Hc)).Valid)
                     then
                        Reuse := False;
                        Put_Line ("[装] 第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (Jk) & " 号抓握通道:存的握区里没有它自己那只眼(第"
                                  & Codec.Img (Natural (Hc)) & " 台)的那一格 ⇒ 合空一次补量");
                     elsif Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length)
                       and then Natural (Stored_Hands (Natural (Old)).Zones (Natural (Hc)).Fingers.Length) /= F.Cams (Natural (Hc)).W * F.Cams (Natural (Hc)).H
                     then
                        --  旧的身体文件不存手指像素 ⇒ 指尖认不出(Zone.Tip_Px 要按手指像素找;X5C3 2026-09-26)⇒ 合空一次补量
                        Reuse := False;
                        Put_Line ("[装] 第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (Jk) & " 号抓握通道:存的握区里没有手指像素(旧的身体文件不存)⇒ 合空一次补量");
                     end if;
                  end;
               end;
            end if;
            if Reuse then
               H := Stored_Hands (Natural (Old));
               Put_Line ("[装] 第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (Jk) & " 号抓握通道:位姿和存的一样 ⇒ 握区照用,不合空");
            else
               Zone.Measure (L, C.Map, A, Jk, F, H, Ok, To_String (C.Inst_Host), C.Inst_Port, Kin_Geo);
               if not Ok then
                  Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (Jk) & " 号抓握通道的握区量不了");
               end if;
            end if;
            if (not Reuse) and then Use_Stored and then Old >= 0 then
               declare
                  Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
               begin
                  if Hc >= 0 and then Natural (Hc) < Natural (H.Zones.Length) and then Natural (Hc) < Natural (Stored_Hands (Natural (Old)).Zones.Length)
                    and then H.Zones (Natural (Hc)).Valid and then Stored_Hands (Natural (Old)).Zones (Natural (Hc)).Valid
                  then
                     declare
                        Zn : constant Zone.Hand_Zone := H.Zones (Natural (Hc));
                        Zo : constant Zone.Hand_Zone := Stored_Hands (Natural (Old)).Zones (Natural (Hc));
                     begin
                        Put_Line ("[装]   第" & Natural'Image (A + 1) & " 只手上相机里的握区:这次 (" & Codec.Fmt (Zn.Cu, 3) & "," & Codec.Fmt (Zn.Cv, 3) & ") 深 " & Codec.Fmt (Zn.Depth, 3) &
                                  " · 存的 (" & Codec.Fmt (Zo.Cu, 3) & "," & Codec.Fmt (Zo.Cv, 3) & ") 深 " & Codec.Fmt (Zo.Depth, 3));
                     end;
                  end if;
               end;
            end if;
            --  合空那一下真看见了手指 ⇒ 记进身体图:这个位姿下,这只手在每台(不长在它上面的)相机里在哪
            for Cm in 0 .. Natural (H.Zones.Length) - 1 loop
               if A < Natural (C.Map.Cam_On_Arm.Length) and then C.Map.Cam_On_Arm (A) /= Integer (Cm) and then H.Zones (Cm).Valid and then A < Natural (F.EE.Length) then
                  declare
                     Z : constant Zone.Hand_Zone := H.Zones (Cm);
                     X : Schema.Sample;
                  begin
                     X.Arm := A; X.Cam := Cm; X.Pose := F.EE (A);
                     --  握合通道带的那块 = 手指:合空时看见的两团 + 区心 + 手指深
                     X.Parts (Chan.Per_Arm + Jk) := (True, Z.Cu, Z.Cv, (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth), Z.X0, Z.Y0, Z.X1, Z.Y1, Z.N_Lobes, Z.A.Cu, Z.A.Cv, Z.B.Cu, Z.B.Cv);
                     --  别的通道带的零件:开机每个通道推过一下,跟着动的那块(从零量的这次才有;装回的身体图里已经带着)
                     begin
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           declare
                              Pi : constant Natural := (A * Chan.Per_Arm + K) * C.Map.N_Cams + Cm;
                           begin
                              if Pi < Natural (C.Map.Parts.Length) and then C.Map.Parts (Pi).Valid then
                                 declare
                                    P : constant Selfmap.Part := C.Map.Parts (Pi);
                                    Zp : Long_Float := 0.0;
                                 begin
                                    if F.Cams (Cm).Has_Depth then
                                       --  读深窗口 = 框的四分之一(比例,无量纲)
                                       Zp := Picture.Near_Depth (F.Cams (Cm).Depth, F.Cams (Cm).W, F.Cams (Cm).H, P.Cu, P.Cv,
                                                                 Long_Float'Max (0.005, Long_Float (P.X1 - P.X0) / Long_Float (F.Cams (Cm).W) * 0.25));
                                       if Picture.Is_Nan (Zp) then
                                          Zp := 0.0;
                                       end if;
                                    end if;
                                    X.Parts (K) := (True, P.Cu, P.Cv, Zp, P.X0, P.Y0, P.X1, P.Y1, 1, P.Cu, P.Cv, 0.0, 0.0);
                                 end;
                              end if;
                           end;
                        end loop;
                     end;
                     Schema.Add (C.Sch, X, C.Map.EE_Noise, C.Map.Rot_Noise);
                  end;
               end if;
            end loop;
            C.Hands.Append (H);
         end;
       end loop;
      end loop;
      Put_Line ("[装] 身体图:" & Codec.Img (Natural (C.Sch.S.Length)) & " 个样本(位姿 → 手指在画面哪儿;只存真看见过的)");
      if Body_Path /= "" then
         Bodyfile.Save (To_String (Body_Path), Key, C.Map, C.Hands, C.Tables, C.Sch);
         Put_Line ("[装] 身体写进 " & To_String (Body_Path) & "(量过 " & Codec.Img (C.Map.Measured_Times) & " 次)");
      end if;
   end;
   C.Boot_Steps := Plug.Steps (L);
   Put_Line ("[装] 开机量身体一共用了 " & Codec.Img (C.Boot_Steps) & " 拍(一拍 = 对方走一步;只记账)");
   Act.Init_Tracks (C);
   World.Init (C.Wld, C.Map.N_Cams);
   for A in 0 .. C.Map.Arms - 1 loop
      Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手上的相机:" & Integer'Image (C.Map.Cam_On_Arm (A)) & " · 各相机变化比例:" &
                Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams), 3) & " " & (if C.Map.N_Cams > 1 then Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams + 1), 3) else "") & " " &
                (if C.Map.N_Cams > 2 then Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams + 2), 3) else ""));
   end loop;
   C.Cam := C.Map.World_Cam;
   --  🔴 抓起过球的那三炮(GB5/GC2/GC4)开机都有这一行;09-20 把几何驾驶搬回 main 时漏了它,
   --  于是几何常数从不装回、Geo_Ready 恒假、整条几何走法是死代码。
   Act.Geo_Install (F, C, To_String (Body_Path), Kin_Geo, Kin_Board, Kin_Plane_Pt, Kin_Plane_N, Kin_Plane_Rms, Kin_Ref, Keep_Tips => Front_Reloaded);
   --  对方在我连上时复位过一次(第一集开始):这个标记在这儿清掉,不然开机量身体的那几段会把它当成"段中间复位"当场收段(S2 2026-09-23 实测:左眼一停没挪就退了)
   if Plug.Take_Reset (L) then
      Put_Line ("[身] 对方在开机前复位过一次(第一集开始)⇒ 清掉标记,接着量身体");
   end if;
   --  步幅先量(每条臂只要几拍):标定变长后开机会顶到一集的步数上限,对方复位打断的应该是后面能"量到几停算几停"的段,不是步幅
   --  (V1F/V1G 2026-09-24:两条臂都"复位打断,一档没试")
   --  腕眼的焦距、朝向和不动的眼前半段已经量了(Geo_Install);这里只量前半段没量的:每条臂一条命令能走多远(步幅)、碰桌面量指尖
   Act.Geo_Boot_Stride (L, F, C);
   Act.Geo_Boot_Support (L, F, C);
   Put_Line ("[身] 身体量完 ⇒ 开始干活(脑在 " & To_String (C.Eye_Host) & ":" & Codec.Img (C.Eye_Port) & (if C.Look_Only then ",只看不动" else "") & ")");
   --  ── 干活循环 ──
   loop
      if not Plug.Sense (L, F) then
         Put_Line ("[链] 取不到画面 ⇒ 退出");
         exit;
      end if;
      if Plug.Take_Reset (L) then
         Put_Line ("[身] 对方复位(新的一集)⇒ 世界记忆清空,身体留着");
         World.Reset_All (C.Wld);
         Memory.Clear (C.Mem);
         C.Recent := Null_Unbounded_String;
         C.Cam := C.Map.World_Cam;
         --  脑起的名字、脑说过"这只眼里没有它"、手指指向 —— 都是上一集的世界,一起清;量过的身体留着。
         --  碰过的面留着但标成"上一集的":桌子一般不动,第一句话就有高度可用;新一集第一次朝下被顶住就换成新量的
         C.Boxed.Clear;
         C.Touch_Fresh := False; C.Bumps.Clear; C.Fingers_Aimed := False; C.Geo_Pw_Valid := False; C.Geo_Pw_Met := False; C.Geo_At_Above := False;
         C.Sil_Valid := False; C.Held_Set_Valid := False; C.Walls.Clear; C.No_Reach_Arm := -1;   --  每件东西量到的摩擦(C.Grip_Mus)留着:越用越准
         Act.Init_Tracks (C);
      end if;
      if Order /= "" then
         C.Task_Text := To_Unbounded_String (Order);
      elsif F.Instruction /= "" then
         C.Task_Text := F.Instruction;
      end if;
      if C.Task_Text = "" then
         C.Task_Text := To_Unbounded_String ("(no instruction was given)");
      end if;
      Memory.Set (C.Mem, "task", To_String (C.Task_Text));
      if F.Cams.Is_Empty then
         Put_Line ("[身] 这一帧没有相机画面");
      else
         Act.Check_Fixed_Eye (F, C);   --  不动的眼挪没挪、挡没挡(V1),每轮核一次
         Act.Round (L, F, C);
         if Body_Path /= "" then
            Bodyfile.Save (To_String (Body_Path), Bodyfile.Fingerprint (L, F), C.Map, C.Hands, C.Tables, C.Sch);
         end if;
      end if;
   end loop;
end Body_Driver;
