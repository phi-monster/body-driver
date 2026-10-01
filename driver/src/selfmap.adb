with Ada.Text_IO; use Ada.Text_IO;
with Codec;
with Chan;
with Lockstep;
with Stats;
with Selfmap.Graph;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Selfmap is

   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural is
     (if Natural (F.Jaw.Length) > Arm then Arm else 0);

   function Jaw_Count (F : Plug.Frame; Arm : Natural) return Natural is
     (if F.Jaw.Is_Empty then 0 else Natural (F.Jaw (Jaw_Index (F, Arm)).Length));

   function Jaw_All (F : Plug.Frame; Arm : Natural) return Floats is
     (if F.Jaw.Is_Empty then F64_Vectors.Empty_Vector else F.Jaw (Jaw_Index (F, Arm)));

   function Jaw_Of (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Long_Float is (F.Jaw (Jaw_Index (F, Arm)) (K));

   function Settle_Beats (Moves : Floats; Noise : Long_Float) return Natural is
      Started : Boolean := False;
   begin
      for T in 0 .. Natural (Moves.Length) - 1 loop
         if Started and then Moves (T) <= Noise and then Moves (T) >= Moves (T - 1) then
            return T + 1;
         end if;
         Started := Started or else Moves (T) > Noise;
      end loop;
      return 0;
   end Settle_Beats;

   function Settle_Since (L : Plug.Link; From_Seq : Natural; Noise : Long_Float) return Natural is
      Moves : Floats;
   begin
      for B of L.Beats loop
         if B.Seq > From_Seq then
            declare
               Mx : Long_Float := 0.0;
            begin
               for X of B.Q_Chg loop
                  Mx := Long_Float'Max (Mx, X);
               end loop;
               Moves.Append (Mx);
            end;
         end if;
      end loop;
      return Settle_Beats (Moves, Noise);
   end Settle_Since;

   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean) is
   begin
      Ok := True;
      for I in 1 .. N loop
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
      end loop;
   end Idle;

   --  这一对帧能不能判第 Cam 台:有地板、两帧都收到了这台的画面、一样大
   function Judgeable (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean is
     (Cam < Natural (M.Floors.Length) and then Plug.Has_Picture (Before) and then Plug.Has_Picture (After)
      and then Before.W = After.W and then Before.H = After.H);

   function Picture_Still (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean is
      Cnt : Natural := 0;
   begin
      if not Judgeable (M, Before, After, Cam) then
         return False;   --  有一帧没收到这台的画面:判不了,不说静止(原来空画面比出来一个动的像素都没有 ⇒ 当成静止)
      end if;
      --  静止 = 超过各自噪声地板的像素凑不成一团(最少像素数的几倍,倍数无量纲;去噪闪烁是撒开的单点)
      for B of Picture.Moved (Before.Gray, After.Gray, M.Floors (Cam)) loop
         if B then
            Cnt := Cnt + 1;
         end if;
      end loop;
      return Cnt <= 4 * Picture.Min_Pixels (Before.W, Before.H);
   end Picture_Still;

   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean is
      Judged : Boolean := False;
   begin
      for C in 0 .. Natural'Min (Natural (Before.Length), Natural (After.Length)) - 1 loop
         if Judgeable (M, Before (C), After (C), C) then
            if not Picture_Still (M, Before (C), After (C), C) then
               return False;
            end if;
            Judged := True;
         end if;
      end loop;
      return Judged;
   end Pictures_Still;

   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean;
                         Prev_Pic : access Plug.Cam_Vectors.Vector := null) is
      Last : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
   begin
      Used := 0;
      Ok := False;   --  等满了还在变 ⇒ 照实说没停稳(调用方别拿这时的画面当停住的量)
      for I in 1 .. Max loop
         if Prev_Pic /= null then
            Prev_Pic.all := F.Cams;
         end if;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Used := I;
         if Pictures_Still (M, Last, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Last := F.Cams;
         Ok := Still >= 2;
         exit when Ok;
      end loop;
   end Wait_Still;

   function Joints_Arrived (Now, Target, Tols : Floats; Tol : Long_Float) return Boolean is
   begin
      for K in 0 .. Natural'Min (Natural (Now.Length), Natural (Target.Length)) - 1 loop
         declare
            Gate : constant Long_Float := (if K < Natural (Tols.Length) and then Tols (K) > 0.0 then Tols (K) else Tol);
         begin
            if Gate <= 0.0 or else abs (Now (K) - Target (K)) > Gate then
               return False;
            end if;
         end;
      end loop;
      return True;
   end Joints_Arrived;

   function Blocked_Stats (Short, Mean, Sd : Long_Float; N_Free : Natural; Lstep, Noise : Long_Float) return Boolean is
      Jit : constant Long_Float := (if N_Free >= 2 then Sd else 0.0);
   begin
      if N_Free = 0 then
         return False;
      end if;
      return Short > Mean + Long_Float'Max (Negligible * Lstep, Stats.Z * Long_Float'Max (Noise, Jit));
   end Blocked_Stats;

   function Blocked (Short, Prev, Prev2 : Long_Float; N_Free : Natural; Lstep, Noise : Long_Float) return Boolean is
     (Blocked_Stats (Short, Prev, abs (Prev - Prev2), N_Free, Lstep, Noise));

   --  一组位姿目标在 Go 里的那一段账:一只手的 Go 和几组一起的 Go 用同一段判停(Judge),一拍一判。
   --  Why:这一组为什么收了 —— Reached = 到了目标 Tol 以内连着两拍;Pressed = 压的那种步沿命令方向停下;Stopped = 连着两拍不动(没到);
   --  Waited = 等满了一条命令最多等的拍数(还在动,或者一次都没动起来);Capped = 总拍数上限;No_Pose = 这一拍没有这条臂的位姿读数
   type Stop_Why is (Going, Reached, Pressed, Stopped, Waited, Capped, No_Pose);
   type Run is record
      Arm : Natural := 0;
      C : Plug.Cmd;
      P0, Prev : Plug.Arm_Pose := [others => 0.0];
      Tol, Tol_Rot : Long_Float := 0.0;
      Still, Arrived, Sub_Frames, Press_Still : Natural := 0;
      Press_Moved : Boolean := False;   --  Press:这一步沿命令方向动起来过
      Halted : Boolean := False;        --  途中 Watch 叫停过
      Started : Boolean := False;       --  这一条发出去以后读数动起来过(挪过"停了"的那道门)
      Send : Boolean := True;
      Why : Stop_Why := Going;
   end record;
   package Run_Vectors is new Ada.Containers.Vectors (Natural, Run);

   function New_Run (Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats; F : Plug.Frame; Tol, Tol_Rot : Long_Float) return Run is
      R : Run;
   begin
      R.Arm := Arm; R.Tol := Tol; R.Tol_Rot := Tol_Rot;
      R.C.Kind := Plug.Ee; R.C.Arm := Arm; R.C.Pose := Target; R.C.Jaw := Jaw;
      R.P0 := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      R.Prev := R.P0;
      return R;
   end New_Run;

   --  这一拍对这一组:看它停没停、到没到、要不要重发(Frames = 这一条 Go 一共走了几拍)
   procedure Judge (M : Body_Map; F : Plug.Frame; Frames : Natural; Press : Boolean; Watch : Watcher; R : in out Run) is
      Arm : constant Natural := R.Arm;
      Still : Natural renames R.Still;
      Arrived : Natural renames R.Arrived;
      Sub_Frames : Natural renames R.Sub_Frames;
      Press_Moved : Boolean renames R.Press_Moved;
      Press_Still : Natural renames R.Press_Still;
      Started : Boolean renames R.Started;
      Tol : constant Long_Float := R.Tol;
      Tol_Rot : constant Long_Float := R.Tol_Rot;
      Still_Frac : constant := Negligible;   --  百分之一(比例,见下)
   begin
      Sub_Frames := Sub_Frames + 1;
      if Arm >= Natural (F.EE.Length) then
         R.Why := No_Pose;
         return;
      end if;
      --  途中每一拍看一眼:出事就把目标改成"停在此刻的位姿",同一条发命令的路再发一次
      if Watch /= null and then not R.Halted and then Watch (F) then
         R.Halted := True;
         R.C.Pose := F.EE (Arm);
         R.Send := True;
         Still := 0;
      end if;
      declare
         D : constant Table.Vec := Chan.Delivered (R.Prev, F.EE (Arm));
         Moved_P : constant Long_Float := Table.Norm (D, 3);
         Rv : constant Long_Float := D (3) ** 2 + D (4) ** 2 + D (5) ** 2;
         --  给了这一档(Tol、Tol_Rot > 0):平移、转动都折成"一步看得见的那一档"的个数,这一拍挪的档数不到这条命令档数的百分之一就算"停了"
         --  (比例;慢的身体还在一拍半毫米地挪时不算停,G2D 2026-09-24 人形返回时还在往下挪被误判成顶住;
         --  V1B23 2026-09-27:只往下压、不转的命令,转动那一项的门原来退成读数噪声 2e-5 弧度,被东西挡住时手一晃就不算停,
         --  顶满 17 拍、一滑把手指推进桌面 19 mm);
         --  没给:挪不到读数噪声才算
         Cmd : constant Table.Vec := Chan.Delivered (R.P0, R.C.Pose);
         Geo : constant Boolean := Tol > 0.0 and then Tol_Rot > 0.0;
         N_Cmd : constant Long_Float := (if Geo then Table.Norm (Cmd, 3) / Tol + Sqrt (Cmd (3) ** 2 + Cmd (4) ** 2 + Cmd (5) ** 2) / Tol_Rot else 0.0);
         N_Beat : constant Long_Float := (if Geo then Moved_P / Tol + Sqrt (Rv) / Tol_Rot else 0.0);
         N_Noise : constant Long_Float := (if Geo then M.EE_Noise / Tol + M.Rot_Noise / Tol_Rot else 0.0);
         Miss : constant Table.Vec := Chan.Delivered (F.EE (Arm), R.C.Pose);
      begin
         if (if Geo then N_Beat <= Long_Float'Max (N_Noise, Still_Frac * N_Cmd)
             else Moved_P <= M.EE_Noise and then Rv <= M.Rot_Noise * M.Rot_Noise)
         then
            Still := Still + 1;
         else
            Still := 0;
            Started := True;
         end if;
         Arrived := (if Tol > 0.0 and then Table.Norm (Miss, 3) <= Tol
                       and then Miss (3) ** 2 + Miss (4) ** 2 + Miss (5) ** 2 <= Tol_Rot * Tol_Rot then Arrived + 1 else 0);
         if Press then
            declare
               Lc : constant Long_Float := Table.Norm (Cmd, 3);
               Along : constant Long_Float := (if Lc > 0.0 then (D (0) * Cmd (0) + D (1) * Cmd (1) + D (2) * Cmd (2)) / Lc else 0.0);
            begin
               if abs Along > Long_Float'Max (M.EE_Noise, Still_Frac * Lc) then
                  Press_Moved := True; Press_Still := 0;
               elsif Press_Moved then
                  Press_Still := Press_Still + 1;
               end if;
            end;
         end if;
         R.Prev := F.EE (Arm);
      end;
      --  到过的范围(09-29):反解被"到过的范围 + 往外一步"截住了(Plug.Held_Back)⇒ 不等停稳:手一动、到过的范围一长(Held_Grown),
      --  这一拍就按此刻的读数重解、重发 —— 目标跟着手往前一步,大转一条 Go 里连着走完(V1B63:等停稳再发,碰指尖 520 → 1012 拍;
      --  快步不重发,一大步只走三成、被认成碰到)。截住了但手还没动起来(Held:命令隔一两拍才起效)⇒ 等,快步也不许先收;
      --  手停在真的尽头 / 碰上东西 ⇒ 范围不再长、一直 Held ⇒ 照常等停下(尽头由 Jointboot 核)。每重发一次拍数重新数;另有一道总拍数上限防万一。
      --  "停了"要等过量出来的起效拍数(M.Settle);还没量过(0)⇒ 读数动起来以后才算得上停(10-01 路 4:晚两拍起效的身体,
      --  开机第一下探针发出去两拍读数不动就被当成"停了",一步都没走就收、探针量成 0,Settle 也就一直量不出)
      declare
         use type Plug.Limit_State;
         Ls : constant Plug.Limit_State := (if Arrived < 2 then Plug.Held_Back (Arm) else Plug.Free);
         Settled : constant Boolean := Still >= 2 and then Sub_Frames >= M.Settle and then (M.Settle > 0 or else Started);
         Cap : constant Natural := 20 * (12 + M.Settle);   --  总拍数上限(次数:防万一,正常的大转十几拍走完)
      begin
         if Ls = Plug.Held_Grown and then Frames < Cap then
            R.Send := True; Still := 0; Sub_Frames := 0; Started := False;
         elsif Arrived >= 2 then
            R.Why := Reached;
         elsif Press and then Ls = Plug.Free and then Press_Still >= 2 then
            R.Why := Pressed;
         elsif Settled then
            R.Why := Stopped;
         elsif Sub_Frames >= 12 + M.Settle then
            R.Why := Waited;
         elsif Frames >= Cap then
            R.Why := Capped;
         end if;
      end;
   end Judge;

   --  运动命令只从这一处发出(自由棘轮:位姿、关节目标都经 Go 走到这里;一组、几组、关节那一支同一个口)
   function Issue (L : in out Plug.Link; C : Plug.Cmd) return Boolean is (Plug.Act (L, C));

   --  几组位姿目标一起走(一拍一条命令带几组的目标):每组一份账、各自判停,都收了才完。
   --  每一拍把要发的那几组各发一回:在按拍对齐的手的任务里 Plug.Act 只记下那一组的目标,主线程把几组合成一条发出去
   procedure Go_Runs (L : in out Plug.Link; M : Body_Map; Rs : in out Run_Vectors.Vector; F : in out Plug.Frame; Frames : out Natural;
                      Ok : out Boolean; Press : Boolean; Watch : Watcher) is
   begin
      Frames := 0;
      Ok := True;
      loop
         for I in 0 .. Natural (Rs.Length) - 1 loop
            if Rs (I).Why = Going and then Rs (I).Send then
               Ok := Issue (L, Rs (I).C);
               if not Ok then
                  return;
               end if;
               declare
                  R : Run := Rs (I);
               begin
                  R.Send := False;
                  Rs.Replace_Element (I, R);
               end;
            end if;
         end loop;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Frames := Frames + 1;
         declare
            All_Done : Boolean := True;
         begin
            for I in 0 .. Natural (Rs.Length) - 1 loop
               if Rs (I).Why = Going then
                  declare
                     R : Run := Rs (I);
                  begin
                     Judge (M, F, Frames, Press, Watch, R);
                     Rs.Replace_Element (I, R);
                     if R.Why = Going then
                        All_Done := False;
                     end if;
                  end;
               end if;
            end loop;
            exit when All_Done;
         end;
      end loop;
   end Go_Runs;

   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Press : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0; Tol_Rot : Long_Float := 0.0;
                 Tols : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector) is
      C : Plug.Cmd;
      Still : Natural := 0;
      Send : Boolean := True;
      --  关节目标:看哪几组读数、各自的目标
      W_G : Ints;
      W_Q : Plug.Floats_Vectors.Vector;
      Prev_All : Plug.Floats_Vectors.Vector;
      Is_Joint : constant Boolean := Group >= 0 or else not Groups.Is_Empty;
      Arrived : Natural := 0;
      Started : Boolean := False;      --  这一条发出去以后读数动起来过
      Still_Frac : constant := Negligible;   --  百分之一(比例,见下)
   begin
      Delivered := Table.Zero_Vec;
      Frames := 0;
      if not Is_Joint then
         --  位姿目标:一组的账、同几组一起走的那一段判停(Judge)
         declare
            Rs : Run_Vectors.Vector;
         begin
            Rs.Append (New_Run (Arm, Target, Jaw, F, Tol, Tol_Rot));
            Go_Runs (L, M, Rs, F, Frames, Ok, Press, Watch);
            if Arm < Natural (F.EE.Length) then
               Delivered := Chan.Delivered (Rs (0).P0, F.EE (Arm));
            end if;
         end;
         return;
      end if;
      C.Kind := Plug.Joint; C.Arm := Arm; C.Pose := Target; C.Jaw := Jaw;
      if not Groups.Is_Empty then
         C.Groups := Groups; C.Qs := Qs;
         W_G := Groups; W_Q := Qs;
      else
         C.Q := Joints; C.Group := Group;
         W_G.Append (Group); W_Q.Append (Joints);
      end if;
      for G of W_G loop
         Prev_All.Append (if G >= 0 and then G < Natural (F.Joints.Length) then F.Joints (Natural (G)) else F64_Vectors.Empty_Vector);
      end loop;
      loop
         if Send then
            Ok := Issue (L, C);
            if not Ok then
               return;
            end if;
            Send := False;
         end if;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Frames := Frames + 1;
         --  关节目标:"停稳"看这几组关节读数(不看位姿:只报关节的身体没有位姿读数)。
         --  到了目标附近(差 ≤ Tol,调用方按这一格的步子定)再有一拍不动 ⇒ 到了;没到目标就等连着两拍不动(被顶住 / 到头)
         --  (5 分钟一炮,2026-09-26:原来每格都等"连着两拍不动 + 量出来的稳定拍数",V1B3 扫描一格 9 拍)。
         --  "停了"要等过量出来的起效拍数;还没量过(Settle = 0)⇒ 读数动起来以后才算得上停(同 Judge)——
         --  这一拍一组读数都读不到 ⇒ 动没动起来看不见,照原来连着两拍就收(没有证据,不白等)
         declare
            Moved : Long_Float := 0.0;
            Arr : Boolean := True;          --  这一拍每一组读得到的关节都到了(Joints_Arrived)
            Any_Read : Boolean := False;    --  至少有一组读得到(读不到的组不算,同原来)
            --  一拍挪不到"到了"那个范围的百分之一 = 停了(比例;动作做完以后读数还会有极小的抖动,空闲时量的噪声是 0 ⇒ 不能拿它当"不动"的门,
            --  V1B4 2026-09-26:每格都等满 14 拍)
            Still_Gate : constant Long_Float := Long_Float'Max (M.Joint_Noise, Tol * Still_Frac);
         begin
            for Gi in 0 .. Natural (W_G.Length) - 1 loop
               declare
                  G : constant Integer := W_G (Gi);
               begin
                  if G >= 0 and then G < Natural (F.Joints.Length) and then Natural (Prev_All (Gi).Length) = Natural (F.Joints (Natural (G)).Length) then
                     for K in 0 .. Natural (Prev_All (Gi).Length) - 1 loop
                        Moved := Long_Float'Max (Moved, abs (F.Joints (Natural (G)) (K) - Prev_All (Gi) (K)));
                     end loop;
                     Any_Read := True;
                     Arr := Arr and then Joints_Arrived (F.Joints (Natural (G)), W_Q (Gi),
                                                         (if Gi < Natural (Tols.Length) then Tols (Gi) else F64_Vectors.Empty_Vector), Tol);
                     Prev_All.Replace_Element (Gi, F.Joints (Natural (G)));
                  end if;
               end;
            end loop;
            Still := (if Moved <= Still_Gate then Still + 1 else 0);
            Started := Started or else Moved > Still_Gate;
            Arrived := (if Any_Read and then Arr then Arrived + 1 else 0);
            --  连着两拍都到了目标附近 = 到了;没到目标就等连着两拍不动(被顶住 / 到头)
            exit when Arrived >= 2 or else (Still >= 2 and then Frames >= M.Settle and then (M.Settle > 0 or else Started or else not Any_Read))
              or else Frames >= 12 + M.Settle;
         end;
      end loop;
   end Go;

   procedure Verify (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Ok_Body, Ok_Link : out Boolean; Note : out String_Note) is
      use Ada.Strings.Unbounded;
      T : Unbounded_String;
   begin
      Ok_Body := True; Ok_Link := True;
      for A in 0 .. M.Arms - 1 loop
         declare
            K : constant Natural := 0;          --  第一个平移通道
            Ch : constant Natural := A * Chan.Per_Arm + K;
            P0 : constant Plug.Arm_Pose := F.EE (A);
            F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
            A_Cmd : Table.Vec := Table.Zero_Vec;
            Deliv, Back : Table.Vec;
            Frames : Natural;
            Ok2 : Boolean;
            --  抓握通道不给目标 = 保持(插头按这一集给过的目标 / 此刻的读数 / 上一回发出去的保持,Plug.Jaw_Values)。
            --  09-30:原来把"此刻第 0 个抓握读数"当目标发,没读数时那个数是编的 1.0(x5"1 = 张开")
            Jaw0 : constant Floats := F64_Vectors.Empty_Vector;
            Visible : Boolean := False;
         begin
            if Ch >= Natural (M.Amp.Length) or else not M.Seen (Ch) then
               Append (T, "第" & Natural'Image (A + 1) & " 只手没有可核的通道;");
               Ok_Body := False;
            else
               A_Cmd (K) := M.Amp (Ch);
               Go (L, M, A, Chan.Compose (P0, A_Cmd), Jaw0, F, Deliv, Frames, Ok2);
               if not Ok2 then
                  Ok_Link := False;
                  return;
               end if;
               for C in 0 .. Natural (F.Cams.Length) - 1 loop
                  if C < Natural (M.Floors.Length) then
                     declare
                        Mv : constant Bools := Picture.Moved (F0 (C).Gray, F.Cams (C).Gray, M.Floors (C));
                        Comps : constant Picture.Regions := Picture.Components (Mv, F.Cams (C).W, F.Cams (C).H, Picture.Min_Pixels (F.Cams (C).W, F.Cams (C).H));
                     begin
                        if not Comps.Is_Empty then
                           Visible := True;
                        end if;
                     end;
                  end if;
               end loop;
               Go (L, M, A, P0, Jaw0, F, Back, Frames, Ok2);
               if not Ok2 then
                  Ok_Link := False;
                  return;
               end if;
               declare
                  Expect : constant Long_Float := M.Delivered (Ch);
                  Got : constant Long_Float := Deliv (K);
               begin
                  --  差一半以内(比例,无量纲)算同一具身体
                  if abs (Got - Expect) <= 0.5 * abs Expect + M.EE_Noise and then Visible then
                     Append (T, "第" & Natural'Image (A + 1) & " 只手:命令 " & Codec.Fmt (M.Amp (Ch), 4) & " 实到 " & Codec.Fmt (Got, 4) & "(存的 " & Codec.Fmt (Expect, 4) & ")对得上;");
                  else
                     Append (T, "第" & Natural'Image (A + 1) & " 只手:实到 " & Codec.Fmt (Got, 4) & " 和存的 " & Codec.Fmt (Expect, 4) & " 对不上" & (if Visible then "" else "(画面里也没看见)") & ";");
                     Ok_Body := False;
                  end if;
               end;
            end if;
         end;
      end loop;
      Note.Text := T;
   end Verify;

   function Rot_Len (A : Table.Vec) return Long_Float is (Sqrt (A (3) ** 2 + A (4) ** 2 + A (5) ** 2));

   --  ── 上一个动作的尾巴收住了没有(Measure_Idle 先等它收住再量)──
   --  一帧到下一帧每一组读数挪了多少:每条臂的平移、转动(位姿),每组关节、每组抓握(组里取挪得最多的那个数)。
   --  组的排法按进门那一帧定,之后每一帧同一个排法;这一帧没有这一组的读数 ⇒ Have = False
   type Group_Move is record
      Moves : Floats;
      Have : Bools;
   end record;
   function Group_Moves (A, B : Plug.Frame; N_Ee, N_Q, N_Jaw : Natural) return Group_Move is
      G : Group_Move;
      procedure Put (Ok : Boolean; X : Long_Float) is
      begin
         G.Have.Append (Ok);
         G.Moves.Append (if Ok then X else 0.0);
      end Put;
      function Max_Diff (X, Y : Floats) return Long_Float is
         Mx : Long_Float := 0.0;
      begin
         for K in 0 .. Natural'Min (Natural (X.Length), Natural (Y.Length)) - 1 loop
            Mx := Long_Float'Max (Mx, abs (Y (K) - X (K)));
         end loop;
         return Mx;
      end Max_Diff;
   begin
      for I in 0 .. N_Ee - 1 loop
         declare
            Ok : constant Boolean := I < Natural (A.EE.Length) and then I < Natural (B.EE.Length);
            D : constant Table.Vec := (if Ok then Chan.Delivered (A.EE (I), B.EE (I)) else Table.Zero_Vec);
         begin
            Put (Ok, Table.Norm (D, Chan.Pos_Channels));
            Put (Ok, Rot_Len (D));
         end;
      end loop;
      for Q in 0 .. N_Q - 1 loop
         Put (Q < Natural (A.Joints.Length) and then Q < Natural (B.Joints.Length),
              (if Q < Natural (A.Joints.Length) and then Q < Natural (B.Joints.Length) then Max_Diff (A.Joints (Q), B.Joints (Q)) else 0.0));
      end loop;
      for J in 0 .. N_Jaw - 1 loop
         Put (J < Natural (A.Jaw.Length) and then J < Natural (B.Jaw.Length),
              (if J < Natural (A.Jaw.Length) and then J < Natural (B.Jaw.Length) then Max_Diff (A.Jaw (J), B.Jaw (J)) else 0.0));
      end loop;
      return G;
   end Group_Moves;

   --  不下命令,等上一个动作的尾巴收住:尾巴是一拍比一拍挪得少;一组读数这一拍挪的比上一拍少不到上一拍的百分之一(Negligible,
   --  同 Go 判"停了")= 这一组收到底了(到了噪声、读数的分辨率,或者身体自己在漂 —— 都不再变小),收住一次就算(噪声有大有小,
   --  不回头再看)。每一组都收住了才回。出口只有量到的"不再变小",不设拍数:H4(人形,09-28)第 490 拍起每拍只挪上一拍的 0.64,
   --  收到读数分辨率要二十来拍;原来接着就量,量成 0.0121 单位的"静止噪声"(真的不抖)。一拍只比上一拍少不到百分之一的慢慢挪
   --  = 身体自己在漂(等不完),它就是不下命令时读数一拍变多少,照实当地板。这一帧没有某一组的读数 ⇒ 那一组不等(没有证据就不等它)
   procedure Wait_Tail (L : in out Plug.Link; F : in out Plug.Frame; Used : out Natural; Ok : out Boolean) is
      N_Ee : constant Natural := Natural (F.EE.Length);
      N_Q : constant Natural := Natural (F.Joints.Length);
      N_Jaw : constant Natural := Natural (F.Jaw.Length);
      Prev_F : Plug.Frame := F;
      Last : Floats;
      Have_Last, Done : Bools;
   begin
      Used := 0;
      Ok := True;
      declare
         G0 : constant Group_Move := Group_Moves (F, F, N_Ee, N_Q, N_Jaw);   --  只取组数
      begin
         for S in 0 .. Natural (G0.Moves.Length) - 1 loop
            Last.Append (0.0); Have_Last.Append (False); Done.Append (False);
         end loop;
      end;
      loop
         exit when (for all D of Done => D);
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Used := Used + 1;
         declare
            G : constant Group_Move := Group_Moves (Prev_F, F, N_Ee, N_Q, N_Jaw);
         begin
            for S in 0 .. Natural (G.Moves.Length) - 1 loop
               if not G.Have (S) then
                  Done.Replace_Element (S, True);
               else
                  if Have_Last (S) and then Last (S) - G.Moves (S) <= Negligible * Last (S) then
                     Done.Replace_Element (S, True);
                  end if;
                  Last.Replace_Element (S, G.Moves (S));
                  Have_Last.Replace_Element (S, True);
               end if;
            end loop;
         end;
         Prev_F := F;
      end loop;
   end Wait_Tail;

   procedure Measure_Idle (L : in out Plug.Link; F : in out Plug.Frame; M : in out Body_Map; Ok : out Boolean) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      Arms : constant Natural := Natural'Min (Natural (F.EE.Length), M.Arms);
   begin
      Ok := True;
      --  ① 什么都不做时读数抖多少、画面抖多少(静止对)—— 先等上一个动作的尾巴收住(Wait_Tail),收住以后再量:
      --  慢的身体接着上一个动作就读,读到的是还在收的尾巴(H4 人形 0.0121 单位,仿真读数本身不抖)。
      --  09-28 头一回改成"等收住再量"把 x5 碰桌面粗找的门带坏过(V1B60:门 = 底 + 3 × 静止噪声,余量碰巧就是 x5 那 0.00004 的尾巴);
      --  09-29 起碰到没有只有 Selfmap.Blocked 一个判法、门里有"这一步的百分之一",x5 从 V1B69 起量到的静止噪声本来就是 0(V1B78 全过)
      declare
         Waited : Natural;
         Ok_W : Boolean;
      begin
         Wait_Tail (L, F, Waited, Ok_W);
         if not Ok_W then
            Ok := False;
            return;
         end if;
         Put_Line ("[身] 静止噪声:先等上一个动作收住 —— 每组读数一拍挪的不再比上一拍少,等了 " & Codec.Img (Waited) & " 拍");
      end;
      declare
         Prev_EE : Plug.Pose_Vectors.Vector := F.EE;
         Prev_Jaw : Plug.Floats_Vectors.Vector := F.Jaw;
         Prev_Q : Plug.Floats_Vectors.Vector := F.Joints;
         Prev_Pic : Plug.Cam_Vectors.Vector := F.Cams;
         --  每台相机最后一对"两帧都收到了画面"的静止对(占位的那一拍不进地板;没有 ⇒ 这台的地板量不到)
         Pair_A, Pair_B : Plug.Cam_Vectors.Vector := Plug.Cam_Vectors.To_Vector (Plug.Cam'(others => <>), Ada.Containers.Count_Type (N_Cams));
      begin
         for K in 1 .. 4 loop
            if not Plug.Sense (L, F) then
               Ok := False;
               return;
            end if;
            for C in 0 .. Natural'Min (N_Cams, Natural'Min (Natural (Prev_Pic.Length), Natural (F.Cams.Length))) - 1 loop
               if Plug.Has_Picture (Prev_Pic (C)) and then Plug.Has_Picture (F.Cams (C))
                 and then Prev_Pic (C).W = F.Cams (C).W and then Prev_Pic (C).H = F.Cams (C).H
               then
                  Pair_A.Replace_Element (C, Prev_Pic (C)); Pair_B.Replace_Element (C, F.Cams (C));
               end if;
            end loop;
            Prev_Pic := F.Cams;
            for A in 0 .. Arms - 1 loop
               declare
                  D : constant Table.Vec := Chan.Delivered (Prev_EE (A), F.EE (A));
               begin
                  M.EE_Noise := Long_Float'Max (M.EE_Noise, Table.Norm (D, 3));
                  M.Rot_Noise := Long_Float'Max (M.Rot_Noise, Sqrt (D (3) ** 2 + D (4) ** 2 + D (5) ** 2));
               end;
            end loop;
            for J in 0 .. Natural (F.Jaw.Length) - 1 loop
               if J < Natural (Prev_Jaw.Length) then
                  for K in 0 .. Natural'Min (Natural (F.Jaw (J).Length), Natural (Prev_Jaw (J).Length)) - 1 loop
                     M.Jaw_Noise := Long_Float'Max (M.Jaw_Noise, abs (F.Jaw (J) (K) - Prev_Jaw (J) (K)));
                  end loop;
               end if;
            end loop;
            for G in 0 .. Natural'Min (Natural (F.Joints.Length), Natural (Prev_Q.Length)) - 1 loop
               for K2 in 0 .. Natural'Min (Natural (F.Joints (G).Length), Natural (Prev_Q (G).Length)) - 1 loop
                  M.Joint_Noise := Long_Float'Max (M.Joint_Noise, abs (F.Joints (G) (K2) - Prev_Q (G) (K2)));
               end loop;
            end loop;
            Prev_EE := F.EE; Prev_Jaw := F.Jaw; Prev_Q := F.Joints;
         end loop;
         M.Floors.Clear; M.Pic_Floor.Clear;
         for C in 0 .. N_Cams - 1 loop
            declare
               Cw : constant Natural := Pair_B (C).W;
               Ch : constant Natural := Pair_B (C).H;
            begin
               if Plug.Has_Picture (Pair_B (C)) then
                  M.Floors.Append (Picture.Null_Floor (Pair_A (C).Gray, Pair_B (C).Gray, Cw, Ch, Picture.Min_Pixels (Cw, Ch)));
                  M.Pic_Floor.Append (Picture.Max_Diff (Pair_A (C).Gray, Pair_B (C).Gray));
               else
                  --  这几拍里这台一对静止对都没收全 ⇒ 量不到它的噪声:地板顶满(它的画面里什么都不算动),不编一个 0 的地板
                  --  (空画面算出来的地板是 0 ⇒ 下一拍收到画面时每个像素的渲染抖动都算"动了")
                  M.Floors.Append (Picture.Floor_Map'(Per_Pixel => U8_Vectors.Empty_Vector, Global => U8'Last, W => 0, H => 0));
                  M.Pic_Floor.Append (Integer (U8'Last));
               end if;
            end;
         end loop;
      end;
   end Measure_Idle;

   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean;
                      Step_Px : Plug.Floats_Vectors.Vector;
                      Eyes : Ints := Int_Vectors.Empty_Vector; World : Integer := -1) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      Arms : constant Natural := Natural (F.EE.Length);
   begin
      M := (others => <>);
      M.Arms := Arms; M.N_Cams := N_Cams; M.Per_Arm := Chan.Per_Arm; M.Channels := Arms * Chan.Per_Arm;
      M.Jaws.Clear;
      for A in 0 .. Arms - 1 loop
         M.Jaws.Append (Integer (Jaw_Count (F, A)));   --  量到几个就是几个,可以 0 个(没有抓握的身体照样开机;路 1 10-01 改这一行,原来 Max (1, …))
      end loop;
      Ok := False;
      if Arms = 0 or else N_Cams = 0 then
         Put_Line ("[身] 没有末端位姿或没有相机,量不了身体");
         return;
      end if;
      declare
         Ok2 : Boolean;
      begin
         Measure_Idle (L, F, M, Ok2);
         if not Ok2 then
            return;
         end if;
      end;
      Put_Line ("[身] 静止噪声:本体位置 " & Codec.Fmt (M.EE_Noise, 5) & " 单位 · 姿态 " & Codec.Fmt (M.Rot_Noise, 5) &
                " rad · 抓握读数 " & Codec.Fmt (M.Jaw_Noise, 4) & " · 各相机灰度地板 " &
                (if M.Pic_Floor.Is_Empty then "-" else Codec.Img (M.Pic_Floor (0))));
      --  ② 逐通道推一下再推回来
      M.Parts := Part_Vectors.To_Vector ((others => <>), Ada.Containers.Count_Type (M.Channels * N_Cams));
      M.Cam_Frac := Zeros (Arms * N_Cams);
      M.Amp := Zeros (M.Channels); M.Delivered := Zeros (M.Channels);
      M.Seen := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (M.Channels));
      M.Cam_On_Arm := Int_Vectors.To_Vector (-1, Ada.Containers.Count_Type (Arms));
      begin
      for A in 0 .. Arms - 1 loop
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Ch : constant Natural := A * Chan.Per_Arm + K;
               P0 : constant Plug.Arm_Pose := F.EE (A);
               F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
               Has_Step : constant Boolean := A < Natural (Step_Px.Length) and then Natural (Step_Px (A).Length) >= 2;
               Amp : constant Long_Float := (if Has_Step then Step_Px (A) (if K < 3 then 0 else 1) else 0.0);
               Accepted : Boolean := False;
               Jaw0 : constant Floats := F64_Vectors.Empty_Vector;   --  抓握通道保持(同 Verify:不拿此刻的读数当目标发,没读数更不编)
            begin
               for Try in 1 .. (if Has_Step and then Amp > 0.0 then 1 else 0) loop
                  declare
                     A_Cmd : Table.Vec := Table.Zero_Vec;
                     Deliv, Back : Table.Vec;
                     Frames : Natural;
                     Ok2 : Boolean;
                     Frames_Back : Natural;
                     Got : Long_Float;
                     F1, F1b : Plug.Cam_Vectors.Vector;   --  推到那头:走完那一帧、再读的一帧
                     Visible : Boolean := False;
                     S0 : Natural := L.Seq;               --  发命令之前那一拍的帧号(Settle 从它往后数)
                  begin
                     A_Cmd (K) := Amp;
                     Go (L, M, A, Chan.Compose (P0, A_Cmd), Jaw0, F, Deliv, Frames, Ok2);
                     if not Ok2 then
                        return;
                     end if;
                     Got := Deliv (K);
                     F1 := F.Cams;
                     --  推到那头再读一帧(画面比读数晚一拍;同开机前半段认手)
                     if not Plug.Sense (L, F) then
                        Ok := False;
                        return;
                     end if;
                     F1b := F.Cams;
                     --  推过去这一条从发出到读数停住用了几拍(连再读的那一拍一起看;09-30:原来取 Go 用的拍数、夹在 6 拍以内);推回来那一条另量
                     M.Settle := Natural'Max (M.Settle, Settle_Since (L, S0, M.Joint_Noise));
                     S0 := L.Seq;
                     Go (L, M, A, P0, Jaw0, F, Back, Frames_Back, Ok2);
                     if not Ok2 then
                        return;
                     end if;
                     M.Settle := Natural'Max (M.Settle, Settle_Since (L, S0, M.Joint_Noise));
                     for C in 0 .. N_Cams - 1 loop
                        declare
                           Fl : Picture.Floor_Map renames M.Floors (C);
                           M1 : constant Bools := Picture.Moved (F0 (C).Gray, F1 (C).Gray, Fl);
                           M2 : constant Bools := Picture.Moved (F1 (C).Gray, F.Cams (C).Gray, Fl);
                           Cw : constant Natural := F.Cams (C).W;
                           Ch2 : constant Natural := F.Cams (C).H;
                           --  跟着动的一块 = 两次比较、不共用一帧都变了的(推之前 → 走完那一帧;再读的那一帧 → 推回来):Picture.Seen_Twice
                           Comps : constant Picture.Regions :=
                             Picture.Seen_Twice (F0 (C).Gray, F1 (C).Gray, F1b (C).Gray, F.Cams (C).Gray, Fl, Cw, Ch2);
                           Fr : constant Long_Float := Picture.Fraction (Picture.Either (M1, M2));
                        begin
                           if not Comps.Is_Empty then
                              Visible := True;
                              declare
                                 Pt : Part;
                                 Big : constant Picture.Region := Comps (0);
                              begin
                                 Pt.Valid := True;
                                 Pt.X0 := Big.X0; Pt.Y0 := Big.Y0; Pt.X1 := Big.X1; Pt.Y1 := Big.Y1;
                                 Pt.Cu := Big.Cu; Pt.Cv := Big.Cv; Pt.Count := Big.Count;
                                 Pt.Frac := Long_Float (Big.Count) / Long_Float (Cw * Ch2);
                                 if abs Got >= Amp * 0.5 then
                                    M.Parts.Replace_Element (Ch * N_Cams + C, Pt);
                                 end if;
                              end;
                           end if;
                           if abs Got >= Amp * 0.5 then
                              M.Cam_Frac.Replace_Element (A * N_Cams + C, Long_Float'Max (M.Cam_Frac (A * N_Cams + C), Fr));
                           end if;
                        end;
                     end loop;
                     M.Amp.Replace_Element (Ch, Amp);
                     M.Delivered.Replace_Element (Ch, Got);
                     if abs Got >= Amp * 0.5 then   --  走到一半以上(纯数学的一半)
                        Accepted := True;
                        M.Seen.Replace_Element (Ch, True);
                        Put_Line ("[身]   通道" & Natural'Image (Ch) & "(第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (K) &
                                  " 轴):一步 = 它自己那只眼里画面挪 1 像素 = " & Codec.Fmt (Amp, 4) & ",实到 " & Codec.Fmt (Got, 4) & " · " & Natural'Image (Frames) & " 拍稳"
                                  & (if Visible then " · 画面里看见了跟着动的一块" else ""));
                     end if;
                  end;
               end loop;
               if not Accepted then
                  Put_Line ("[身]   通道" & Natural'Image (Ch) & (if Has_Step then ":推 " & Codec.Fmt (Amp, 4) & " 走不到一半(实到 " & Codec.Fmt (M.Delivered (Ch), 4) & ")"
                            else ":这只手没量成运动学 ⇒ 量不了"));
               end if;
            end;
         end loop;
      end loop;
      end;
      Put_Line ("[身] 一条命令从发出到读数停住(每拍挪动不超过静止噪声 " & Codec.Fmt (M.Joint_Noise, 6) & "、而且不再变小):"
                & (if M.Settle > 0 then "量到的最多 " & Codec.Img (M.Settle) & " 拍" else "每一次都还没停住就收了 / 读数没动起来 ⇒ 量不出(记 0,不编)"));
      --  ③ 哪台相机长在哪只手上:这只手一动它整幅都变,而且比第二名多一倍(倍数,无量纲);世界相机 = 变得最少的。
      --  开机前半段已经认过(Eyes 不空)⇒ 照用
      if not Eyes.Is_Empty then
         for A in 0 .. Natural'Min (Arms, Natural (Eyes.Length)) - 1 loop
            M.Cam_On_Arm.Replace_Element (A, Eyes (A));
         end loop;
         M.World_Cam := (if World >= 0 then Natural (World) else 0);
         Ok := True;
         return;
      end if;
      for A in 0 .. Arms - 1 loop
         declare
            Best : Integer := -1;
            Bv, Second : Long_Float := 0.0;
         begin
            for C in 0 .. N_Cams - 1 loop
               declare
                  V : constant Long_Float := M.Cam_Frac (A * N_Cams + C);
               begin
                  if V > Bv then
                     Second := Bv; Bv := V; Best := C;
                  elsif V > Second then
                     Second := V;
                  end if;
               end;
            end loop;
            if Best >= 0 and then Bv > 0.0 and then Bv >= 2.0 * Second then
               M.Cam_On_Arm.Replace_Element (A, Best);
               Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手一动,第" & Integer'Image (Best) & " 台相机变了 " &
                         Codec.Fmt (Bv * 100.0, 0) & "% 的画面 ⇒ 它长在这只手上");
            end if;
         end;
      end loop;
      declare
         Best : Natural := 0;
         Bv : Long_Float := 1.0e9;
      begin
         for C in 0 .. N_Cams - 1 loop
            declare
               Mx : Long_Float := 0.0;
            begin
               for A in 0 .. Arms - 1 loop
                  Mx := Long_Float'Max (Mx, M.Cam_Frac (A * N_Cams + C));
               end loop;
               if Mx < Bv then
                  Bv := Mx; Best := C;
               end if;
            end;
         end loop;
         M.World_Cam := Best;
         Put_Line ("[身] 世界相机 = 第" & Natural'Image (Best) & " 台(手动时它变得最少)");
      end;
      Ok := True;
   end Measure;

   --  ── 走一步(I6)──
   procedure Note_Free (P : in out Free_Part; Short, Len : Long_Float) is
      D : constant Long_Float := Short - P.Mean;
   begin
      P.Len_Hi := Long_Float'Max (P.Len_Hi, Len);
      P.N := P.N + 1;
      P.Mean := P.Mean + D / Long_Float (P.N);
      P.M2 := P.M2 + D * (Short - P.Mean);
   end Note_Free;

   --  样本标准差(n − 1:平均是从同一批样本估的);一个样本没有散布
   function Free_Sd (P : Free_Part) return Long_Float is
     (if P.N >= 2 then Sqrt (Long_Float'Max (0.0, P.M2) / Long_Float (P.N - 1)) else 0.0);

   function Blocked_By (P : Free_Part; Short, Len, Noise : Long_Float) return Boolean is
     (Blocked_Stats (Short, P.Mean * Len, Free_Sd (P) * Len, P.N, Len, Noise));

   procedure Seed (W : in out Walk; M : Body_Map; Arm : Natural) is
      Fl : Free_Leg;
      Chs : constant Ints := Selfmap.Graph.Pose_Channels (M, Arm);   --  这条臂的位姿通道(问身体图,不按下标算)
   begin
      for Lg of W.Legs loop
         if Lg.Arm = Arm then
            return;
         end if;
      end loop;
      Fl.Arm := Arm;
      --  开机探针(Measure):每个通道推一步、停下再读(Tol = 0)⇒ 实到 ÷ 推的 = 这具身体空走一步交付几成;平移、转动各一份
      for K in 0 .. Natural (Chs.Length) - 1 loop
         declare
            Ch : constant Natural := Natural (Chs (K));
         begin
            if Ch < Natural (M.Amp.Length) and then Ch < Natural (M.Delivered.Length) and then Ch < Natural (M.Seen.Length)
              and then M.Seen (Ch) and then M.Amp (Ch) > 0.0
            then
               if K < Chan.Pos_Channels then
                  Note_Free (Fl.Tr, 1.0 - M.Delivered (Ch) / M.Amp (Ch), M.Amp (Ch));
               else
                  Note_Free (Fl.Rot, 1.0 - M.Delivered (Ch) / M.Amp (Ch), M.Amp (Ch));
               end if;
            end if;
         end;
      end loop;
      W.Legs.Append (Fl);
   end Seed;

   function Leg_Of (W : Walk; Arm : Natural) return Natural is
   begin
      for I in 0 .. Natural (W.Legs.Length) - 1 loop
         if W.Legs (I).Arm = Arm then
            return I;
         end if;
      end loop;
      return Natural (W.Legs.Length);
   end Leg_Of;

   --  这具身体空走一步(长 Len)最多交付它自己的几成(上界):平均少走的减 Stats.Z 倍散布(同 Blocked 的门,另一侧)。拿上界不拿平均:
   --  交付得最多的那一步放大以后也不走过头(不来回晃)。不到两步(量不出散布)/ 上界是一整份(交付满,x5 的探针停下再读就是)⇒ 1,不放大。
   --  比量过的最长那一步还长 ⇒ 1:少走的是一个比例(每条命令只走到七八成)还是一截死区(少走的长度不随步长变),只有不同长的几步才分得出;
   --  一格长的探针少走四成、按比例放大到一大步 ⇒ 有死区的身体一步冲过头(焊点:0.4 档死区、10 单位一步冲过 1.6 单位)。
   --  这一步不放大走了,它就是下一步的证据(几种长短的样本散布开,上界自己收回 1)
   function Gain_Hi (P : Free_Part; Len : Long_Float) return Long_Float is
      G : constant Long_Float := 1.0 - Long_Float'Max (0.0, P.Mean - Stats.Z * Free_Sd (P));
   begin
      return (if P.N >= 2 and then G > 0.0 and then Len <= P.Len_Hi then G else 1.0);
   end Gain_Hi;

   procedure Step (L : in out Plug.Link; M : Body_Map; Legs : Leg_Vectors.Vector; Lim : Limits; F : in out Plug.Frame;
                   W : in out Walk; Rep : out Leg_Step_Vectors.Vector; Frames : out Natural; Ok : out Boolean) is
      Rs : Run_Vectors.Vector;
      Run_Of : Ints;   --  每一组:它的那份账在 Rs 里第几个(-1 = 这一步没发)
   begin
      Rep.Clear; Frames := 0; Ok := True;
      for Lg of Legs loop
         if Lg.Arm >= Natural (F.EE.Length) then
            Ok := False;   --  这一拍没有这条臂的位姿读数:从哪儿起走都不知道,一组都不发
            return;
         end if;
      end loop;
      for Lg of Legs loop
         declare
            S : Leg_Step;
            Chs : constant Ints := Selfmap.Graph.Pose_Channels (M, Lg.Arm);   --  这条臂的位姿通道(问身体图,不按下标算)
            Has_Notch : constant Boolean := Natural (Chs.Length) > Chan.Pos_Channels
              and then Natural (Chs (0)) < Natural (M.Amp.Length) and then Natural (Chs (Chan.Pos_Channels)) < Natural (M.Amp.Length);
            --  这只手一步看得见的那一档(开机量的:平移、转动各一档)—— Loose 的"到了"、反解"够得到"的细度都按它
            Tp : constant Long_Float := (if Has_Notch then M.Amp (Natural (Chs (0))) else 0.0);
            Tr : constant Long_Float := (if Has_Notch then M.Amp (Natural (Chs (Chan.Pos_Channels))) else 0.0);
            P0 : constant Plug.Arm_Pose := F.EE (Lg.Arm);
            D : constant Table.Vec := Chan.Delivered (P0, Lg.Goal);
            A : Table.Vec := Table.Zero_Vec;
            --  上限:这一条发出去的整步按一个比例缩到 Most 以内(Len = 这一步此刻的平移长 / 转角)
            procedure Cap_To (Len, Most : Long_Float) is
            begin
               if Len > Most then
                  for I in 0 .. Chan.Per_Arm - 1 loop
                     A (I) := A (I) * Long_Float'Max (0.0, Most) / Len;
                  end loop;
               end if;
            end Cap_To;
         begin
            S.Arm := Lg.Arm; S.From := P0;
            Seed (W, M, Lg.Arm);
            --  走还差的 Frac;交付不满的身体按它空走一步最多交付几成(上界,Gain_Hi)把这一步放大,平移、转动各按各的 ——
            --  每条命令只走到它的七八成就停的身体(真机 70–85%),把同一个目标再发一遍不会再走,不放大就停在离目标还差两三成的地方
            declare
               Fl : constant Free_Leg := W.Legs (Leg_Of (W, Lg.Arm));
               G_T : constant Long_Float := Gain_Hi (Fl.Tr, abs Lim.Frac * Table.Norm (D, Chan.Pos_Channels));
               G_R : constant Long_Float := Gain_Hi (Fl.Rot, abs Lim.Frac * Rot_Len (D));
            begin
               for I in 0 .. Chan.Per_Arm - 1 loop
                  A (I) := Lim.Frac * D (I) / (if I < Chan.Pos_Channels then G_T else G_R);
               end loop;
            end;
            --  上限:眼跟得住、离可能碰到的地方远(量的是发出去的这一条;整步按一个比例缩)
            Cap_To (Table.Norm (A, Chan.Pos_Channels), Lim.Track);
            Cap_To (Table.Norm (A, Chan.Pos_Channels), Lim.Clear);
            Cap_To (Rot_Len (A), Lim.Track_Rot);
            --  上限:反解够得到。解不到 ⇒ 沿这一步二分到解得到的那一截,细到这只手一步看得见的那一档(平移、转动都细过它才停)
            if Lim.Reach and then Tp > 0.0 and then Tr > 0.0 and then (Table.Norm (A, Chan.Pos_Channels) > 0.0 or else Rot_Len (A) > 0.0) then
               declare
                  Pe, Re : Long_Float;
                  Rok : Boolean;
                  At_Full : constant Table.Vec := A;
                  function Part (T : Long_Float) return Table.Vec is
                     V : Table.Vec := Table.Zero_Vec;
                  begin
                     for I in 0 .. Chan.Per_Arm - 1 loop
                        V (I) := T * At_Full (I);
                     end loop;
                     return V;
                  end Part;
               begin
                  Plug.Reach (Lg.Arm, Chan.Compose (P0, A), Pe, Re, Rok);
                  if Rok and then (Pe > Tp or else Re > Tr) then
                     declare
                        Lo : Long_Float := 0.0;
                        Hi : Long_Float := 1.0;   --  整步(沿这一步的比例)
                        Lt : constant Long_Float := Table.Norm (At_Full, Chan.Pos_Channels);
                        Lr : constant Long_Float := Rot_Len (At_Full);
                     begin
                        while (Hi - Lo) * Lt > Tp or else (Hi - Lo) * Lr > Tr loop
                           declare
                              Mid : constant Long_Float := (Lo + Hi) / 2.0;   --  二分
                           begin
                              Plug.Reach (Lg.Arm, Chan.Compose (P0, Part (Mid)), Pe, Re, Rok);
                              if Rok and then Pe <= Tp and then Re <= Tr then
                                 Lo := Mid;
                              else
                                 Hi := Mid;
                              end if;
                           end;
                        end loop;
                        A := Part (Lo);
                        S.Reach_Cut := True;
                     end;
                  end if;
               end;
            end if;
            S.Cmd := A;
            S.Len := Table.Norm (A, Chan.Pos_Channels);
            S.Ang := Rot_Len (A);
            S.Aim := Chan.Compose (P0, A);
            --  反解一截都够不到 ⇒ 这一组不发;别的照发(一步是零也发:同 Go,这只手保持在此刻、等它停)
            if not S.Reach_Cut or else S.Len > 0.0 or else S.Ang > 0.0 then
               Rs.Append (New_Run (Lg.Arm, S.Aim, Lg.Jaw, F, (if Lim.Loose then Tp else 0.0), (if Lim.Loose then Tr else 0.0)));
               Run_Of.Append (Integer (Rs.Length) - 1);
            else
               Run_Of.Append (-1);
            end if;
            Rep.Append (S);
         end;
      end loop;
      --  发:一组 ⇒ 同一只手的 Go;几组 ⇒ 在按拍对齐的手的任务里直接走(每一拍 Plug 把几组的目标合成一条),
      --  在主线程里 ⇒ 开一只手的任务按拍对齐(同几只手一起碰桌面那一段)
      if not Rs.Is_Empty and then (Rs.First_Index = Rs.Last_Index or else Lockstep.Current_Hand >= 0) then
         Go_Runs (L, M, Rs, F, Frames, Ok, Lim.Press, Lim.Watch);
      elsif not Rs.Is_Empty then
         declare
            Hand_Ok : Boolean := True;
            Hand_Frames : Natural := 0;
            Beat_Ok : Boolean := True;
            task Leg_Hand;
            task body Leg_Hand is
               Fr : Plug.Frame := F;
            begin
               Lockstep.Begin_Hand (0);
               begin
                  Go_Runs (L, M, Rs, Fr, Hand_Frames, Hand_Ok, Lim.Press, Lim.Watch);
               exception
                  when others =>
                     Hand_Ok := False;
               end;
               Lockstep.Done;
            end Leg_Hand;
         begin
            Lockstep.Clear;
            Plug.Lock_Begin;
            Lockstep.Start (0, Leg_Hand'Identity);
            loop
               Lockstep.Run (0);
               exit when Lockstep.Finished (0);
               Plug.Lock_Beat (L, F, Beat_Ok);
            end loop;
            Plug.Lock_End;
            Lockstep.Clear;
            Frames := Hand_Frames; Ok := Hand_Ok and then Beat_Ok;
         end;
      end if;
      --  量:每组实到多少、到没到、挡没挡(Blocked_By,拿这一段空走的底);空走的记进底
      for I in 0 .. Natural (Rep.Length) - 1 loop
         declare
            S : Leg_Step := Rep (I);
            Wi : constant Natural := Leg_Of (W, S.Arm);
            Fl : Free_Leg := W.Legs (Wi);
            Goal : constant Plug.Arm_Pose := Legs (I).Goal;
         begin
            if S.Arm < Natural (F.EE.Length) then
               declare
                  Now : constant Plug.Arm_Pose := F.EE (S.Arm);
                  Dl : constant Table.Vec := Chan.Delivered (Now, Goal);
               begin
                  S.Left := Table.Norm (Dl, Chan.Pos_Channels);
                  S.Left_Rot := Rot_Len (Dl);
                  if Run_Of (I) >= 0 then
                     declare
                        R : constant Run := Rs (Natural (Run_Of (I)));
                        Miss : constant Table.Vec := Chan.Delivered (Now, S.Aim);
                     begin
                        S.Got := Chan.Delivered (S.From, Now);
                        S.Went := (if S.Len > 0.0 then (S.Got (0) * S.Cmd (0) + S.Got (1) * S.Cmd (1) + S.Got (2) * S.Cmd (2)) / S.Len else 0.0);
                        S.Turned := (if S.Ang > 0.0 then (S.Got (3) * S.Cmd (3) + S.Got (4) * S.Cmd (4) + S.Got (5) * S.Cmd (5)) / S.Ang else 0.0);
                        S.Halted := R.Halted;
                        S.Moving := (R.Why in Waited | Capped) and then R.Still = 0;
                        S.Arrived := R.Tol > 0.0 and then Table.Norm (Miss, Chan.Pos_Channels) <= R.Tol and then Rot_Len (Miss) <= R.Tol_Rot;
                        --  判挡没挡:它自己停下来了、又没到 ⇒ 少走的比这一段空走时多出门没有。走到了(那一档以内,少走多少是 Go 在哪一刻收的)、
                        --  被 Watch 叫停、等满了还在动 ⇒ 不判,也不进底
                        if not (S.Arrived or else S.Halted or else S.Moving) and then R.Why /= No_Pose then
                           S.Blocked_T := S.Len > 0.0 and then Blocked_By (Fl.Tr, S.Len - S.Went, S.Len, M.EE_Noise);
                           S.Blocked_R := S.Ang > 0.0 and then Blocked_By (Fl.Rot, S.Ang - S.Turned, S.Ang, M.Rot_Noise);
                           if not (S.Blocked_T or else S.Blocked_R) then
                              if S.Len > 0.0 then
                                 Note_Free (Fl.Tr, (S.Len - S.Went) / S.Len, S.Len);
                              end if;
                              if S.Ang > 0.0 then
                                 Note_Free (Fl.Rot, (S.Ang - S.Turned) / S.Ang, S.Ang);
                              end if;
                              W.Legs.Replace_Element (Wi, Fl);
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end if;
            Rep.Replace_Element (I, S);
         end;
      end loop;
   end Step;

end Selfmap;
