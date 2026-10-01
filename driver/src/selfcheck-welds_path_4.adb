with Ada.Exceptions;
with Selfmap.Graph;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
separate (Selfcheck)
procedure Welds_Path_4 is
   --  路 4 的焊点(大并行.md §5 路 4):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)

   --  ── 假身体(位姿空间,一组读数 = 一条臂的位姿):主线程当身体,手的任务里跑驱动真的那几段(Selfmap.Measure / Step / Go),
   --  开机也按驱动自己的量法量它(Measure:探针、起效拍数 Settle、静止噪声)。命令隔 Dead 拍才起效(真机晚 1–2 拍);
   --  交付不满两种(真机每步 70–85%):① 每条命令只走到这一条还差的 R 就停,同一个目标再发一遍不再走;② 每拍只走还差的 70–85%、命令一直有效。
   --  之外每拍走还差的 Alpha(x5 量的:命令发出下一拍走 89%,V1B78 延迟基线);读数(位姿、关节)加 ±Noise 的抖动;可选一堵墙:平移 x 不许超过 Wall_X ──
   Max_Arms : constant := 2;
   type Pend is record
      At_Beat : Natural := 0;
      T : Plug.Arm_Pose := [others => 0.0];
   end record;
   package Pend_Vectors is new Ada.Containers.Vectors (Natural, Pend);
   type Fake_Arm is record
      X, Y, Last_T : Plug.Arm_Pose := [others => 0.0];
      Q : Pend_Vectors.Vector;
      K : Natural := 0;              --  起效过几条命令(取交付比例用)
   end record;
   type Fake_Arms is array (0 .. Max_Arms - 1) of Fake_Arm;
   Bs : Fake_Arms;
   N_Arms : Positive := 1;
   Dead : Natural := 0;
   Alpha_X5 : constant Long_Float := 0.89;
   --  两种交付不满的身体(真机每步 70–85%):Rate_Mode = 每拍只走还差的 70–85%(命令一直有效,慢慢走到);
   --  否则 = 每条命令只走到七八成就停(同一个目标再发一遍不再走:目标差不到 1e-9 就算同一条)
   Rate_Mode : Boolean := False;
   --  第三种(真舵机常见):每条命令少走一截死区 —— 少走的长度不随步长变(平移 Band_T、转动 Band_R)
   Band_Mode : Boolean := False;
   Noise : constant Long_Float := 1.0e-6;
   Tn : constant Long_Float := 0.005;     --  这具假身体一步看得见的那一档(平移,开机"一步 = 自己那只眼里挪 1 像素"的那一档)
   Tr : constant Long_Float := 0.0025;    --  转动那一档
   --  死区那种身体:每条命令平移少走这么长、转动少走这么多 —— 一格长的探针只走到六成(过了开机"走到一半以上"那道,探针记得上),
   --  一大步几乎走满
   Band_T : constant Long_Float := 0.4 * Tn;
   Band_R : constant Long_Float := 0.4 * Tr;
   Wall_On : Boolean := False;
   Wall_X : Long_Float := 0.0;
   --  每条命令交付几成:Rand ⇒ 70–85% 之间按表轮着取(表里第 12–14 条是 0.85、0.84、0.70:两条挨着的差一丝、第三条少一截 ——
   --  拿"最近两步"当底会被认成挡住的那种);否则每条都交付 R_Fix
   Rand : Boolean := True;
   R_Fix : Long_Float := 1.0;
   R_Min : constant Long_Float := 0.70;
   type R_Table is array (0 .. 15) of Long_Float;
   R_Tab : constant R_Table := [0.80, 0.79, 0.72, 0.81, 0.84, 0.75, 0.71, 0.83, 0.78, 0.74, 0.85, 0.77, 0.85, 0.84, 0.70, 0.76];
   function R_Of (Arm, K : Natural) return Long_Float is
     (if Rate_Mode then 1.0 elsif Rand then R_Tab ((K + 5 * Arm) mod R_Tab'Length) else R_Fix);
   function Alpha_Of (B : Natural) return Long_Float is (if Rate_Mode then R_Tab (B mod R_Tab'Length) else Alpha_X5);
   Beat : Natural := 0;
   Lk : Plug.Link;
   Pic : Plug.Cam;
   Start : constant Plug.Arm_Pose := [0.3, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0];
   Start2 : constant Plug.Arm_Pose := [-0.3, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0];
   --  不来回晃:每一拍每只手离它的目标多远(真的,不带读数抖动),比上一拍变远最多多少
   Goals : array (0 .. Max_Arms - 1) of Plug.Arm_Pose := [others => Start];
   Watch_Goal : Boolean := False;
   Back_T, Back_R : Long_Float := 0.0;
   Prev_T, Prev_R : array (0 .. Max_Arms - 1) of Long_Float := [others => Long_Float'Last];
   Both_Moved : Natural := 0;   --  几组一起走:两只手在同一拍都动了的拍数

   function To_Pose (Q : Floats) return Plug.Arm_Pose is
      P : Plug.Arm_Pose := [others => 0.0];
   begin
      for I in P'Range loop
         if I < Natural (Q.Length) then
            P (I) := Q (I);
         end if;
      end loop;
      return P;
   end To_Pose;
   function To_Q (P : Plug.Arm_Pose) return Floats is
      Q : Floats;
   begin
      for X of P loop
         Q.Append (X);
      end loop;
      return Q;
   end To_Q;
   function Same (A, B : Plug.Arm_Pose) return Boolean is (for all I in A'Range => abs (A (I) - B (I)) <= 1.0e-9);
   function Rot_Len (D : Table.Vec) return Long_Float is (Sqrt (D (3) ** 2 + D (4) ** 2 + D (5) ** 2));
   function Scaled (D : Table.Vec; S : Long_Float) return Table.Vec is
      V : Table.Vec := Table.Zero_Vec;
   begin
      for I in 0 .. Chan.Per_Arm - 1 loop
         V (I) := S * D (I);
      end loop;
      return V;
   end Scaled;
   function Offset (P : Plug.Arm_Pose; Dx, Dy, Dz, Rx, Ry, Rz : Long_Float) return Plug.Arm_Pose is
      V : Table.Vec := Table.Zero_Vec;
   begin
      V (0) := Dx; V (1) := Dy; V (2) := Dz; V (3) := Rx; V (4) := Ry; V (5) := Rz;
      return Chan.Compose (P, V);
   end Offset;
   --  位姿命令 ⇒ "关节"目标(这具假身体的一组读数就是它的位姿):Plug 在按拍对齐时只收关节目标
   procedure Fake_Cmd (C : in out Plug.Cmd; Ok : out Boolean) is
   begin
      C.Kind := Plug.Joint; C.Group := Integer (C.Arm); C.Q := To_Q (C.Pose);
      Ok := True;
   end Fake_Cmd;
   --  这一拍的读数:真位姿加抖动(平移三轴、绕 z 一丝转动;抖动 ±Noise,确定的样子)
   function Frame_Now return Plug.Frame is
      Ff : Plug.Frame;
   begin
      for A in 0 .. N_Arms - 1 loop
         declare
            N : constant Long_Float := Noise * Long_Float ((Beat * 7 + A * 3) mod 5 - 2) / 2.0;
            V : Table.Vec := Table.Zero_Vec;
            P : Plug.Arm_Pose;
         begin
            V (0) := N; V (1) := -N; V (2) := N; V (5) := N;
            P := Chan.Compose (Bs (A).X, V);
            Ff.EE.Append (P);
            Ff.Joints.Append (To_Q (P));
         end;
      end loop;
      Ff.Cams.Append (Pic);
      Ff.Seq := Beat;
      return Ff;
   end Frame_Now;
   --  身体走一拍:合成的那一条命令里每组的目标和上一回收到的不一样 ⇒ 排进队、Dead 拍以后起效;起效时交付还差的 R;每拍走还差的 Alpha
   procedure Advance is
      Moved : array (0 .. Max_Arms - 1) of Boolean := [others => False];
   begin
      declare
         Mg : constant Plug.Cmd := Plug.Lock_Merged;
      begin
         for K in 0 .. Natural (Mg.Groups.Length) - 1 loop
            declare
               G : constant Integer := Mg.Groups (K);
               T : constant Plug.Arm_Pose := To_Pose (Mg.Qs (K));
            begin
               if G >= 0 and then G < N_Arms and then not Same (T, Bs (G).Last_T) then
                  Bs (G).Q.Append (Pend'(At_Beat => Beat + Dead, T => T));
                  Bs (G).Last_T := T;
               end if;
            end;
         end loop;
      end;
      for A in 0 .. N_Arms - 1 loop
         while not Bs (A).Q.Is_Empty and then Bs (A).Q.First_Element.At_Beat <= Beat loop
            declare
               D : constant Table.Vec := Chan.Delivered (Bs (A).X, Bs (A).Q.First_Element.T);
               Lt : constant Long_Float := Table.Norm (D, Chan.Pos_Channels);
               Lr : constant Long_Float := Rot_Len (D);
               V : Table.Vec := Table.Zero_Vec;
            begin
               if Band_Mode then
                  for I in 0 .. Chan.Per_Arm - 1 loop
                     V (I) := D (I) * (if I < Chan.Pos_Channels then (if Lt > 0.0 then Long_Float'Max (0.0, Lt - Band_T) / Lt else 0.0)
                                       else (if Lr > 0.0 then Long_Float'Max (0.0, Lr - Band_R) / Lr else 0.0));
                  end loop;
               else
                  V := Scaled (D, R_Of (A, Bs (A).K));
               end if;
               Bs (A).Y := Chan.Compose (Bs (A).X, V);
            end;
            Bs (A).K := Bs (A).K + 1;
            Bs (A).Q.Delete_First;
         end loop;
         declare
            X0 : constant Plug.Arm_Pose := Bs (A).X;
         begin
            Bs (A).X := Chan.Compose (Bs (A).X, Scaled (Chan.Delivered (Bs (A).X, Bs (A).Y), Alpha_Of (Beat)));
            if Wall_On and then Bs (A).X (0) > Wall_X then
               Bs (A).X (0) := Wall_X;
            end if;
            --  这一拍真在走(挪过这一档的百分之一;走完以后几何级数收尾的那一丝不算)
            Moved (A) := Table.Norm (Chan.Delivered (X0, Bs (A).X), Chan.Pos_Channels) > Selfmap.Negligible * Tn
              or else Rot_Len (Chan.Delivered (X0, Bs (A).X)) > Selfmap.Negligible * Tr;
         end;
         if Watch_Goal then
            declare
               D : constant Table.Vec := Chan.Delivered (Bs (A).X, Goals (A));
               Dt : constant Long_Float := Table.Norm (D, Chan.Pos_Channels);
               Dr : constant Long_Float := Rot_Len (D);
            begin
               if Prev_T (A) < Long_Float'Last then
                  Back_T := Long_Float'Max (Back_T, Dt - Prev_T (A));
                  Back_R := Long_Float'Max (Back_R, Dr - Prev_R (A));
               end if;
               Prev_T (A) := Dt; Prev_R (A) := Dr;
            end;
         end if;
      end loop;
      if N_Arms >= 2 and then Moved (0) and then Moved (1) then
         Both_Moved := Both_Moved + 1;
      end if;
   end Advance;
   procedure Reset_Body (D : Natural; Random : Boolean; R : Long_Float; Arms : Positive) is
   begin
      Dead := D; Rand := Random; R_Fix := R; N_Arms := Arms; Rate_Mode := False; Band_Mode := False;
      Wall_On := False;
      for A in 0 .. Max_Arms - 1 loop
         Bs (A) := (X | Y | Last_T => (if A = 0 then Start else Start2), others => <>);
      end loop;
      Watch_Goal := False; Back_T := 0.0; Back_R := 0.0; Prev_T := [others => Long_Float'Last]; Prev_R := [others => Long_Float'Last];
      Both_Moved := 0;
      Lk.Beats.Clear;
   end Reset_Body;

   --  ── 手的任务里做的那一件(Job),主线程当身体 ──
   type Job_Kind is (Do_Measure, Do_Steps, Do_Joint_Go);
   Job : Job_Kind := Do_Measure;
   M : Selfmap.Body_Map;
   Measure_Ok : Boolean := False;
   Legs : Selfmap.Leg_Vectors.Vector;
   Lim : Selfmap.Limits;
   Wk : Selfmap.Walk;
   Frames : Natural := 0;
   --  Do_Steps:一步一步走(调用方走到一个目标就是这样:同一个 Walk、每步从此刻的读数起走还差的),每一步的账都留下(判挡没挡的牙要看每一步);
   --  Until_There ⇒ 每组都差不到一档(平移 Tn、转动 Tr)或者有一组被挡住就停;Steps_Done = 走了几步
   Until_There : Boolean := False;
   Steps_Done : Natural := 0;

   Reps : Selfmap.Leg_Step_Vectors.Vector;
   N_Steps : Natural := 1;
   Step_Ok : Boolean := True;
   --  Do_Joint_Go:关节目标的 Go(开机前半段扫关节走的那一条)
   Joint_Target : Floats;
   Joint_Frames : Natural := 0;
   Joint_Ok : Boolean := False;
   M0 : Selfmap.Body_Map;

   procedure Run_Hand is
      task type Hand;
      task body Hand is
         Fr : Plug.Frame := Frame_Now;
      begin
         Lockstep.Begin_Hand (0);
         begin
            case Job is
               when Do_Measure =>
                  declare
                     Step_Px : Plug.Floats_Vectors.Vector;
                     Eyes : Ints;
                     St : Floats;
                  begin
                     St.Append (Tn); St.Append (Tr);
                     for A in 0 .. N_Arms - 1 loop
                        Step_Px.Append (St); Eyes.Append (0);
                     end loop;
                     Selfmap.Measure (Lk, Fr, M, Measure_Ok, Step_Px, Eyes => Eyes, World => 0);
                  end;
               when Do_Steps =>
                  for I in 1 .. N_Steps loop
                     declare
                        Rs : Selfmap.Leg_Step_Vectors.Vector;
                        Fs : Natural;
                        There : Boolean := True;
                        Hit : Boolean := False;
                     begin
                        Selfmap.Step (Lk, M, Legs, Lim, Fr, Wk, Rs, Fs, Step_Ok);
                        Frames := Frames + Fs; Steps_Done := Steps_Done + 1;
                        for S of Rs loop
                           Reps.Append (S);
                           There := There and then S.Left <= Tn and then S.Left_Rot <= Tr;
                           Hit := Hit or else S.Blocked_T or else S.Blocked_R;
                        end loop;
                        exit when Until_There and then (There or else Hit);
                     end;
                  end loop;
               when Do_Joint_Go =>
                  declare
                     Dl : Table.Vec;
                  begin
                     Selfmap.Go (Lk, M0, 0, Start, F64_Vectors.Empty_Vector, Fr, Dl, Joint_Frames, Joint_Ok, Joints => Joint_Target, Group => 0,
                                 Tol => Tn);
                  end;
            end case;
         exception
            when E : others =>
               Put_Line ("  🔴 路 4 假身体的手出错:" & Ada.Exceptions.Exception_Information (E));
               Fails := Fails + 1;
         end;
         Lockstep.Done;
      end Hand;
   begin
      Lockstep.Clear;
      Plug.Lock_Begin;
      declare
         H : Hand;
      begin
         Lockstep.Start (0, H'Identity);
         loop
            Lockstep.Run (0);
            exit when Lockstep.Finished (0);
            Beat := Beat + 1;
            Advance;
            declare
               Ff : constant Plug.Frame := Frame_Now;
            begin
               Lk.Seq := Beat;
               Plug.Note_Beat (Lk, Ff);
               Plug.Lock_Feed (Ff);
            end;
         end loop;
      end;
      Plug.Lock_End;
      Lockstep.Clear;
   end Run_Hand;
   procedure Boot_Measure is
   begin
      Job := Do_Measure;
      Run_Hand;
   end Boot_Measure;
   function Probe_Short (Ch : Natural) return Long_Float is
     (if Ch < Natural (M.Amp.Length) and then M.Amp (Ch) > 0.0 then 1.0 - M.Delivered (Ch) / M.Amp (Ch) else Long_Float'Last);
   --  每条臂的位姿通道都量成了(问身体图,不按下标算)
   function All_Seen return Boolean is
   begin
      if Selfmap.Graph.Arm_Count (M) /= N_Arms then
         return False;
      end if;
      for A in 0 .. Selfmap.Graph.Arm_Count (M) - 1 loop
         for Ch of Selfmap.Graph.Pose_Channels (M, A) loop
            if Ch >= Natural (M.Seen.Length) or else not M.Seen (Ch) then
               return False;
            end if;
         end loop;
      end loop;
      return True;
   end All_Seen;
begin
   Pic.W := 8; Pic.H := 8;
   for I in 0 .. Pic.W * Pic.H - 1 loop
      Pic.Gray.Append (U8 (100));
   end loop;
   Plug.Set_Hooks (null, Fake_Cmd'Unrestricted_Access);
   Plug.Set_Reach (null); Plug.Set_Limit (null);

   --  ① 开机量一具晚两拍才起效的身体(Selfmap.Measure 的探针,Go 等停):Settle 还没量过(0)时,读数动起来以前不算"停了"。
   --  病:发出去两拍读数不动就当成停了 ⇒ 第一下探针一步都没走就收、实到 0 ⇒ 通道量不成(晚两拍的身体开不了机)。
   --  牙(10-01 离线拆过跑过):把 Judge 和关节那一支里的"(M.Settle > 0 or else Started)"拿掉 ⇒ ①② 都红(探针没走到、关节目标 2 拍 0 挪就收)
   declare
      Settle_Ok, Deliv_Ok : Boolean := True;
   begin
      Reset_Body (2, False, 1.0, 1);
      Boot_Measure;
      for Ch of Selfmap.Graph.Pose_Channels (M, 0) loop
         Deliv_Ok := Deliv_Ok and then Probe_Short (Ch) < Selfmap.Negligible;   --  交付满的身体:探针停下再读,少走的不到这一步的百分之一
      end loop;
      Settle_Ok := M.Settle >= Dead + 1;
      Check (Measure_Ok and then All_Seen and then Settle_Ok and then Deliv_Ok,
             "走一步·晚两拍起效的身体照样开得了机:六个通道的探针都走到了(每个少走的不到一档)、"
             & "量出来的起效拍数 Settle = " & Codec.Img (M.Settle) & "(要 ≥ 晚的拍数 + 1 = " & Codec.Img (Dead + 1) & ")");
   end;
   --  ② 关节目标的 Go(开机前半段扫关节那一条),Settle 没量过、身体晚两拍:等读数动起来,走到目标附近才收。
   --  病:发出去两拍读数不动就收 ⇒ 这一格当成"关节没动"(开机前半段关节扫描的头一格就量不出来)。牙:同 ①
   declare
      Q0 : constant Floats := To_Q (Start);
      Got : Long_Float;
   begin
      Reset_Body (2, False, 1.0, 1);
      M0 := (others => <>);
      M0.Joint_Noise := Noise;
      Joint_Target := Q0;
      Joint_Target.Replace_Element (0, Q0 (0) + 20.0 * Tn);
      Job := Do_Joint_Go;
      Run_Hand;
      Got := Bs (0).X (0) - Start (0);
      Check (Joint_Ok and then abs (Got - 20.0 * Tn) < Tn and then Joint_Frames > Dead + 1,
             "走一步·关节目标、Settle 没量过、身体晚两拍:Go 等到读数动起来才判停,走了 " & Codec.Fmt (Got, 4) & "(要 " & Codec.Fmt (20.0 * Tn, 4)
             & ")、" & Codec.Img (Joint_Frames) & " 拍");
   end;
   --  ③ 核心(大并行 §5 路 4 第一条焊点):假身体晚 1–2 拍起效、每一步只交付 70–85%(三种:每条命令只走到七八成就停 / 每拍只走还差的
   --  七八成 / 每条命令少走一截死区)、读数有抖动 —— 先按开机的量法量它(Measure),再一步一步走(Step,每步走还差的一整份)到一个平移
   --  9.9 单位、转 0.2 弧度的目标:要收敛(每组都差不到一档就算到,步数不超过"交付至少 70% 时几何级数要几步"再多一步)、每一拍离目标
   --  只近不远(真位姿;看不出的一丝 —— 不到一档 —— 不算晃)、一步都不判成挡住、第一步一丝不放大(交付满的 x5 行为不变)。
   --  病:(a) 每条命令只走到七八成就停的身体,下一步还按"走还差的一整份"发 ⇒ 发的就是上一条那个目标,身体不再走 ⇒ 停在离目标还差一两成的
   --      地方、被判成挡住(今天 Geo_Approach 近了一步走完的那一段就是这样,会报一个假的 resist)⇒ 按量到的交付上界把这一步放大(Gain_Hi);
   --  (b) 按平均交付放大 ⇒ 交付得多的那一步走过头、回头 ⇒ 来回晃 ⇒ 用上界(平均少走 − Z 倍散布);
   --  (c) 拿一格长的探针量的交付比例去放大一大步 ⇒ 有死区的身体(少走的长度不随步长变)一步冲过头 ⇒ 只在量过的长度以内放大,
   --      第一大步不放大、走完它就是证据(几种长短的样本散布开,上界自己收回 1);
   --  (d) 拿"最近两步空走"当 Blocked 的底 ⇒ 交付 0.85、0.84 两步挨得近,第三步 0.70 就被认成挡住 ⇒ 半路收工。
   --  牙(10-01 离线各拆一处跑过):(a) Gain_Hi 恒为 1 ⇒ 每条命令 70–85% 的四炮都在第 2 步判成挡住、离目标 1.5–3.0 单位,两只手那条也红;
   --  (b) 按平均放大 ⇒ 离目标一拍变远 0.017 单位(三档多),红;(c) 比量过的还长的一步也放大 ⇒ 每一炮第一步都放大了,交付满的那炮
   --  一拍变远 0.0013,死区那炮冲过头,红;(d) 下面按每一步的账用老底重判一遍 ⇒ 交付 70–85% 轮着的那两炮有一步被认成挡住(当场算)
   declare
      type Case_Rec is record
         D : Natural;
         Rnd : Boolean;
         R : Long_Float;
         Rate : Boolean;
         Band : Boolean;
      end record;
      type Case_Array is array (Positive range <>) of Case_Rec;
      Cases : constant Case_Array := [(1, True, 0.0, False, False), (2, True, 0.0, False, False), (1, False, 0.70, False, False),
                                      (2, False, 0.85, False, False), (1, True, 0.0, True, False), (2, True, 0.0, True, False),
                                      (1, False, 1.0, False, True), (0, False, 1.0, False, False)];
      Goal0 : constant Plug.Arm_Pose := Offset (Start, 8.0, -5.0, 3.0, 0.1, -0.08, 0.15);
      Dt0 : constant Long_Float := Table.Norm (Chan.Delivered (Start, Goal0), Chan.Pos_Channels);
      Dr0 : constant Long_Float := Rot_Len (Chan.Delivered (Start, Goal0));
   begin
      for Cs of Cases loop
         Reset_Body (Cs.D, Cs.Rnd, Cs.R, 1);
         Rate_Mode := Cs.Rate; Band_Mode := Cs.Band;
         Boot_Measure;
         declare
            Rm : constant Long_Float := (if Cs.Rate then 1.0 elsif Cs.Rnd then R_Min else Cs.R);
            --  交付至少 Rm ⇒ 每一步还差的至少缩到 1 − Rm;差到一档以内要几步(几何级数),再多一步:Go 按"停了"收时身体离它要去的那一处还差一丝
            Bound : constant Natural :=
              (if Rm >= 1.0 then 1
               else Natural (Long_Float'Ceiling (Long_Float'Min (Log (Tn / Dt0), Log (Tr / Dr0)) / Log (1.0 - Rm)))) + 1;
            Any_Blocked : Boolean := False;
            Old_Blocked : Boolean := False;   --  牙:拿"最近两步空走"当底会不会有一步被认成挡住
         begin
            Legs.Clear;
            Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Goal0, Jaw => <>));
            Goals (0) := Goal0; Watch_Goal := True;
            Lim := (others => <>);
            Wk := (others => <>);
            Reps.Clear; Frames := 0; N_Steps := Bound + 2;
            Job := Do_Steps;
            Run_Hand;
            Watch_Goal := False;
            --  每一步的账:按新判法有没有一步挡住;按老的"最近两步"当底重判一遍(底从这一段第一步起数,同碰指尖那一段)
            declare
               P1, P2 : Long_Float := 0.0;
               N_Free : Natural := 0;
            begin
               for S of Reps loop
                  Any_Blocked := Any_Blocked or else S.Blocked_T or else S.Blocked_R;
                  if not S.Arrived and then S.Len > 0.0 then
                     if Selfmap.Blocked ((S.Len - S.Went), P1 * S.Len, P2 * S.Len, N_Free, S.Len, M.EE_Noise) then
                        Old_Blocked := True;
                     end if;
                     P2 := P1; P1 := (S.Len - S.Went) / S.Len; N_Free := N_Free + 1;
                  end if;
               end loop;
            end;
            declare
               Left_T : constant Long_Float := Table.Norm (Chan.Delivered (Bs (0).X, Goal0), Chan.Pos_Channels);
               Left_R : constant Long_Float := Rot_Len (Chan.Delivered (Bs (0).X, Goal0));
               Arrived_At : Natural := 0;
               --  第一步发的就是整份还差的,一丝都不放大:它比开机探针(一格)长,少走的是比例还是死区还分不出(交付满的 x5 ⇒ 行为不变)
               Unscaled : constant Boolean :=
                 abs (Reps (0).Len - Table.Norm (Chan.Delivered (Reps (0).From, Goal0), Chan.Pos_Channels)) < 1.0e-9
                 and then abs (Reps (0).Ang - Rot_Len (Chan.Delivered (Reps (0).From, Goal0))) < 1.0e-9;
            begin
               for I in 0 .. Natural (Reps.Length) - 1 loop
                  if Arrived_At = 0 and then Reps (I).Left <= Tn and then Reps (I).Left_Rot <= Tr then
                     Arrived_At := I + 1;
                  end if;
               end loop;
               Check (Arrived_At > 0 and then Arrived_At <= Bound and then Left_T <= Tn and then Left_R <= Tr and then not Any_Blocked
                      and then Back_T <= Tn and then Back_R <= Tr
                      and then (if Cs.Rnd and then not Cs.Rate then Old_Blocked else True)
                      and then Unscaled,
                      "走一步·假身体晚 " & Codec.Img (Cs.D) & " 拍起效、" & (if Cs.Rate then "每拍只走还差的 70–85%" elsif Cs.Band then "每条命令少走一截死区(0.4 档)"
                        elsif Cs.Rnd then "每条命令交付 70–85% 轮着"
                        else "每条命令交付 " & Codec.Fmt (100.0 * Cs.R, 0) & "%")
                      & ":第 " & Codec.Img (Arrived_At) & " 步到(要 ≤ " & Codec.Img (Bound) & ")、剩 " & Codec.Fmt (Left_T, 4) & " / " & Codec.Fmt (Left_R, 4) & " rad、"
                      & Codec.Img (Frames) & " 拍 · 离目标每拍最多变远 " & Codec.Fmt (Back_T, 7) & " / " & Codec.Fmt (Back_R, 7)
                      & "(要 ≤ 一档:看不出的那一丝不算晃)· 一步都没判成挡住" & (if Any_Blocked then "(错:有一步判成挡住)" else "")
                      & (if Unscaled then " · 第一步一丝没放大" else " · 第一步放大了(错:拿一格长的探针去放大一大步)")
                      & (if Cs.Rnd and then not Cs.Rate then " · 牙:按最近两步空走当底 ⇒ " & (if Old_Blocked then "有一步被认成挡住" else "(牙没咬住)") else ""));
            end;
         end;
      end loop;
   end;
   --  ④ 碰到没有 = Blocked(同碰指尖那一判)代替"沿命令方向实到不到一半":x5 那种身体(不晚、交付满、每拍走 89%),一堵墙挡在这一步的六成处。
   --  病:"走到一半就算没挡住" ⇒ 走了六成被墙挡住照样当空走,下一步才知道(接触集往下伸、走过去贴上时多压一步)。
   --  牙:按老判法(实到 + 实到 < 要走的)⇒ 没挡住(下面同一步的账重算)
   declare
      Rep_Hit, Rep_Small : Selfmap.Leg_Step;
      Old_Hit : Boolean;
   begin
      Reset_Body (0, False, 1.0, 1);
      Boot_Measure;
      Wall_On := True; Wall_X := Start (0) + 0.6 * 20.0 * Tn;
      Legs.Clear;
      Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Offset (Start, 20.0 * Tn, 0.0, 0.0, 0.0, 0.0, 0.0), Jaw => <>));
      Lim := (others => <>); Wk := (others => <>); Reps.Clear; Frames := 0; N_Steps := 1;
      Job := Do_Steps;
      Run_Hand;
      Rep_Hit := Reps (0);
      Old_Hit := Rep_Hit.Went + Rep_Hit.Went < Rep_Hit.Len;
      --  同一具身体一小步(四档)、没有墙:Go 按"到了那一档以内"收,读数只走到 98.8% —— 不判(不是它自己停下的)
      Wall_On := False;
      Reset_Body (0, False, 1.0, 1);
      Boot_Measure;
      Legs.Clear;
      Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Offset (Start, 4.0 * Tn, 0.0, 0.0, 0.0, 0.0, 0.0), Jaw => <>));
      Wk := (others => <>); Reps.Clear; Frames := 0; N_Steps := 1;
      Job := Do_Steps;
      Run_Hand;
      Rep_Small := Reps (0);
      declare
         --  牙:到了的那一步也按少走的比例判 ⇒ 这一步被认成挡住(底 = 开机探针那几步,停下再读、交付满)
         Wk_Probe : Selfmap.Walk;
         Exempt_Tooth : Boolean;
      begin
         Selfmap.Seed (Wk_Probe, M, 0);
         Exempt_Tooth := Selfmap.Blocked_By (Wk_Probe.Legs (0).Tr, Rep_Small.Len - Rep_Small.Went, Rep_Small.Len, M.EE_Noise);
         Check (Rep_Hit.Blocked_T and then not Rep_Hit.Arrived and then abs (Rep_Hit.Went - 0.6 * Rep_Hit.Len) < Tn and then not Old_Hit
                and then Rep_Small.Arrived and then not Rep_Small.Blocked_T and then Rep_Small.Went < Rep_Small.Len and then Exempt_Tooth,
                "走一步·碰到没有 = Blocked:墙挡在六成处 ⇒ 实到 " & Codec.Fmt (Rep_Hit.Went, 4) & " / 要 " & Codec.Fmt (Rep_Hit.Len, 4) & " ⇒ "
                & (if Rep_Hit.Blocked_T then "挡住" else "没挡住(错)") & " · 牙:老判法(实到不到一半才算挡)⇒ " & (if Old_Hit then "挡住(牙没咬住)" else "没挡住")
                & " · 一小步到了那一档以内(实到 " & Codec.Fmt (Rep_Small.Went / Rep_Small.Len * 100.0, 1) & "%)⇒ 不判挡"
                & (if Rep_Small.Blocked_T then "(错:判成挡住)" else "")
                & " · 牙:到了的也判 ⇒ " & (if Exempt_Tooth then "被认成挡住" else "(牙没咬住)"));
      end;
   end;
   --  ⑤ 交付 70–85% 的身体,墙挡在到目标的三成处 ⇒ 这一步就判成挡住;挡在七成五 ⇒ 这一步交付的比例在空走的散布里,判不出(照实:分辨不了),
   --  下一步(几乎走不动)判成挡住。病:散布量小了(只拿两步)⇒ 空走的也判成挡;散布量大了 ⇒ 真挡住的永远判不出
   declare
      Hit_30, Hit_75a, Hit_75b : Boolean;
   begin
      Hit_30 := False; Hit_75a := False; Hit_75b := False;
      Reset_Body (1, True, 0.0, 1);
      Boot_Measure;
      Wall_On := True; Wall_X := Start (0) + 0.3 * 40.0 * Tn;
      Legs.Clear;
      Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Offset (Start, 40.0 * Tn, 0.0, 0.0, 0.0, 0.0, 0.0), Jaw => <>));
      Lim := (others => <>); Wk := (others => <>); Reps.Clear; Frames := 0; N_Steps := 1;
      Job := Do_Steps;
      Run_Hand;
      Hit_30 := Reps (0).Blocked_T;
      Reset_Body (1, True, 0.0, 1);
      Boot_Measure;
      Wall_On := True; Wall_X := Start (0) + 0.75 * 40.0 * Tn;
      Legs.Clear;
      Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Offset (Start, 40.0 * Tn, 0.0, 0.0, 0.0, 0.0, 0.0), Jaw => <>));
      Wk := (others => <>); Reps.Clear; Frames := 0; N_Steps := 2;
      Job := Do_Steps;
      Run_Hand;
      Hit_75a := Reps (0).Blocked_T; Hit_75b := Reps (1).Blocked_T;
      Wall_On := False;
      Check (Hit_30 and then not Hit_75a and then Hit_75b,
             "走一步·交付 70–85% 的身体撞墙:挡在三成处 ⇒ " & (if Hit_30 then "这一步判成挡住" else "没判出(错)")
             & " · 挡在七成五(这一步放大过,交付的比例在空走的散布里)⇒ 这一步" & (if Hit_75a then "判成挡住(错:空走也会这样)" else "判不出")
             & "、下一步" & (if Hit_75b then "判成挡住" else "还没判出(错)"));
   end;
   --  ⑥ 几组一起走:两只手同一个 Walk 一步一步走(每一步一条 Step 带两组的目标、每一拍一条命令合成两组),都到;
   --  有好些拍两只手同时在动,总拍数比两只手先后各走一遍少 ——
   --  不是一只走完另一只再走。病:一次只动一只(交接力棒,§2 第 18 条)⇒ 拍数是两只各走的和。
   --  牙:两只各走一遍(先后)⇒ 拍数多出一截、同一拍都在动的拍数 0
   declare
      Goal_A : constant Plug.Arm_Pose := Offset (Start, 0.6, 0.2, -0.1, 0.0, 0.05, 0.1);
      Goal_B : constant Plug.Arm_Pose := Offset (Start2, -0.4, 0.3, 0.2, 0.08, 0.0, -0.06);
      Par_Frames, Par_Both, Seq_Frames, Seq_Both, Par_Steps : Natural := 0;
      Par_Left : Long_Float;
   begin
      Reset_Body (1, True, 0.0, 2);
      Boot_Measure;
      Both_Moved := 0;
      Legs.Clear;
      Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Goal_A, Jaw => <>));
      Legs.Append (Selfmap.Leg'(Arm => 1, Goal => Goal_B, Jaw => <>));
      Lim := (others => <>); Wk := (others => <>); Reps.Clear; Frames := 0; Steps_Done := 0; N_Steps := 30; Until_There := True;
      Job := Do_Steps;
      Run_Hand;
      Par_Frames := Frames; Par_Both := Both_Moved; Par_Steps := Steps_Done;
      Par_Left := Long_Float'Max (Table.Norm (Chan.Delivered (Bs (0).X, Goal_A), Chan.Pos_Channels),
                                  Table.Norm (Chan.Delivered (Bs (1).X, Goal_B), Chan.Pos_Channels));
      --  牙:同一具身体、同样两个目标,两只手先后各走一遍
      Reset_Body (1, True, 0.0, 2);
      Boot_Measure;
      Seq_Frames := 0; Both_Moved := 0;
      for A in 0 .. 1 loop
         Legs.Clear;
         Legs.Append (Selfmap.Leg'(Arm => A, Goal => (if A = 0 then Goal_A else Goal_B), Jaw => <>));
         Wk := (others => <>); Reps.Clear; Frames := 0; Steps_Done := 0;
         Job := Do_Steps;
         Run_Hand;
         Seq_Frames := Seq_Frames + Frames;
      end loop;
      Seq_Both := Both_Moved;
      Until_There := False;
      Check (Par_Steps < 30 and then Par_Left <= Tn and then Par_Frames < Seq_Frames and then Par_Both > 0 and then Seq_Both = 0,
             "走一步·两只手一起走:" & Codec.Img (Par_Steps) & " 步都到(差不到一档)、" & Codec.Img (Par_Frames) & " 拍,两只手同一拍在动 "
             & Codec.Img (Par_Both) & " 拍 · 牙:先后各走一遍 ⇒ " & Codec.Img (Seq_Frames) & " 拍、同一拍在动 " & Codec.Img (Seq_Both) & " 拍");
   end;
   --  ⑦ 三道上限:眼跟得住(平移 / 转动各一道)、离可能碰到的地方(Clear)、反解够得到(Plug.Reach 解不到就二分缩到解得到,细到一档)。
   --  病:上限不管 ⇒ 一步把跟着的东西甩出眼、冲进可能碰到的那条带子、发一条反解解不出的命令
   --  (S1A4:一条大命令转 1.4 弧度,身体在真尽头停下、腕眼看丢)。
   --  牙:不问反解 ⇒ 这一步照要的整步发(下面同一步不带 Reach 再走一次)
   declare
      Far : constant Plug.Arm_Pose := Offset (Start, 40.0 * Tn, 0.0, 0.0, 0.0, 0.0, 0.0);
      Turn : constant Plug.Arm_Pose := Offset (Start, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2);
      Reach_X : constant Long_Float := Start (0) + 14.0 * Tn;   --  够得到的:x 不超过这儿
      procedure Fake_Reach (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
         pragma Unreferenced (Arm);
      begin
         Pos_Err := Long_Float'Max (0.0, Pose (0) - Reach_X); Rot_Err := 0.0;
      end Fake_Reach;
      R_Track, R_Clear, R_Rot, R_Reach, R_Free : Selfmap.Leg_Step;
      Reach_Frames : Natural;
      procedure One (Goal : Plug.Arm_Pose; L_In : Selfmap.Limits; Out_R : out Selfmap.Leg_Step) is
      begin
         Reset_Body (0, False, 1.0, 1);
         Legs.Clear;
         Legs.Append (Selfmap.Leg'(Arm => 0, Goal => Goal, Jaw => <>));
         Lim := L_In; Wk := (others => <>); Reps.Clear; Frames := 0; N_Steps := 1;
         Job := Do_Steps;
         Run_Hand;
         Out_R := Reps (0);
      end One;
   begin
      Reset_Body (0, False, 1.0, 1);
      Boot_Measure;
      One (Far, (Track => 6.0 * Tn, others => <>), R_Track);
      One (Far, (Clear => 9.0 * Tn, others => <>), R_Clear);
      One (Turn, (Track_Rot => 20.0 * Tr, others => <>), R_Rot);
      Plug.Set_Reach (Fake_Reach'Unrestricted_Access);
      One (Far, (Reach => True, others => <>), R_Reach);
      Reach_Frames := Frames;
      One (Far, (others => <>), R_Free);
      Plug.Set_Reach (null);
      Check (abs (R_Track.Len - 6.0 * Tn) < 1.0e-9 and then abs (R_Clear.Len - 9.0 * Tn) < 1.0e-9 and then abs (R_Rot.Ang - 20.0 * Tr) < 1.0e-9
             and then R_Reach.Reach_Cut and then R_Reach.Aim (0) <= Reach_X + Tn and then R_Reach.Aim (0) >= Reach_X - Tn and then Reach_Frames > 0
             and then abs (R_Free.Len - Table.Norm (Chan.Delivered (R_Free.From, Far), Chan.Pos_Channels)) < 1.0e-9,
             "走一步·三道上限:眼跟得住 ⇒ 平移这一步 " & Codec.Fmt (R_Track.Len, 4) & "(上限 " & Codec.Fmt (6.0 * Tn, 4) & ")、转 " & Codec.Fmt (R_Rot.Ang, 4)
             & "(上限 " & Codec.Fmt (20.0 * Tr, 4) & ")· 离带子 " & Codec.Fmt (9.0 * Tn, 4) & " ⇒ " & Codec.Fmt (R_Clear.Len, 4)
             & " · 反解够到 x ≤ " & Codec.Fmt (Reach_X - Start (0), 4) & " ⇒ 这一步走到 " & Codec.Fmt (R_Reach.Aim (0) - Start (0), 4) & "(细到一档 " & Codec.Fmt (Tn, 4) & ")"
             & " · 牙:不问反解 ⇒ 整步 " & Codec.Fmt (R_Free.Len, 4));
   end;
   Plug.Set_Hooks (null, null);
end Welds_Path_4;
