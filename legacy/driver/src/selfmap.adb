with Ada.Text_IO; use Ada.Text_IO;
with Ada.Calendar;
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

   function Rot_Len (A : Table.Vec) return Long_Float is (Sqrt (A (3) ** 2 + A (4) ** 2 + A (5) ** 2));

   procedure Watch_Reset (W : out Settle_Watch; N : Natural) is
   begin
      W.Last.Clear; W.Peak.Clear; W.Have_Last.Clear; W.Done.Clear; W.Ruled.Clear;
      for I in 1 .. N loop
         W.Last.Append (0.0); W.Peak.Append (0.0); W.Have_Last.Append (False); W.Done.Append (False); W.Ruled.Append (False);
      end loop;
   end Watch_Reset;

   procedure Watch_Peak (W : in out Settle_Watch; S : Natural; V : Long_Float) is
   begin
      if S < Natural (W.Peak.Length) then
         W.Ruled.Replace_Element (S, True);
         if V > W.Peak (S) then
            W.Peak.Replace_Element (S, V);
         end if;
      end if;
   end Watch_Peak;

   procedure Watch_Feed (W : in out Settle_Watch; Moves : Floats; Have : Bools) is
   begin
      for S in 0 .. Natural'Min (Natural (W.Done.Length), Natural'Min (Natural (Moves.Length), Natural (Have.Length))) - 1 loop
         if not Have (S) then
            W.Done.Replace_Element (S, True);    --  这一拍没有它的数:不等它
         else
            if W.Ruled (S) and then Moves (S) > W.Peak (S) then
               W.Peak.Replace_Element (S, Moves (S));
            end if;
            if W.Have_Last (S) and then (Stopped_Shrinking (W.Last (S), Moves (S)) or else (W.Ruled (S) and then Moves (S) <= Negligible * W.Peak (S))) then
               W.Done.Replace_Element (S, True);
            end if;
            W.Last.Replace_Element (S, Moves (S));
            W.Have_Last.Replace_Element (S, True);
         end if;
      end loop;
   end Watch_Feed;

   function Watch_All_Done (W : Settle_Watch) return Boolean is (for all D of W.Done => D);

   procedure Picture_Change (Before, After : Plug.Cam; Change : out Long_Float; Ok : out Boolean) is
      Sum : Long_Float := 0.0;
   begin
      Change := 0.0;
      Ok := Plug.Has_Picture (Before) and then Plug.Has_Picture (After) and then Before.W = After.W and then Before.H = After.H;
      if not Ok then
         return;
      end if;
      for I in 0 .. Natural (Before.Gray.Length) - 1 loop
         Sum := Sum + abs (Long_Float (After.Gray (I)) - Long_Float (Before.Gray (I)));
      end loop;
      Change := Sum / Long_Float (Before.Gray.Length);
   end Picture_Change;

   procedure Cam_Feed (W : in out Cam_Watch; Prev2, Prev, Now : Plug.Cam) is
      C1, C2 : Long_Float;
      Ok1, Ok2 : Boolean;
   begin
      Picture_Change (Prev, Now, C1, Ok1);
      if not Ok1 then
         return;
      end if;
      Picture_Change (Prev2, Now, C2, Ok2);
      W.Seen := True;
      W.Peak := Long_Float'Max (W.Peak, C1);
      if W.Have and then (Stopped_Shrinking (W.C1_Last, C1) or else C1 <= Negligible * W.Peak)
        and then (not Ok2 or else C2 - C1 <= Stats.Z * abs (C1 - W.C1_Last))
      then
         W.Done := True;
      end if;
      W.C1_Last := C1; W.Have := True;
   end Cam_Feed;

   function Settle_Beats (Moves : Floats; Noise : Long_Float) return Natural is
      Started : Boolean := False;
      Wt : Settle_Watch;
      Mv : Floats;
      Hv : Bools;
   begin
      Watch_Reset (Wt, 1);
      Mv.Append (0.0); Hv.Append (True);
      for T in 0 .. Natural (Moves.Length) - 1 loop
         Started := Started or else Moves (T) > Noise;
         if Started then
            --  尺子 = 这一条挪得最多的那一拍(同 Go 判"走完了":掉到它的百分之一以下)
            Watch_Peak (Wt, 0, Moves (T));
            Mv.Replace_Element (0, Moves (T));
            Watch_Feed (Wt, Mv, Hv);
            if Watch_All_Done (Wt) then
               return T + 1;
            end if;
         end if;
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
      pragma Unreferenced (M);
      N : constant Natural := Natural (F.Cams.Length);
      Last, Last2 : Plug.Cam_Vectors.Vector := F.Cams;   --  上一拍、上上拍(第一拍之前都 = 进门那一帧)
      Ws : array (0 .. N) of Cam_Watch;
   begin
      Used := 0;
      Ok := False;   --  等满了还在变 ⇒ 照实说没停稳(调用方别拿这时的画面当停住的量)
      loop
         exit when Max > 0 and then Used >= Max;
         if Prev_Pic /= null then
            Prev_Pic.all := F.Cams;
         end if;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Used := Used + 1;
         for C in 0 .. N - 1 loop
            if C < Natural (Last.Length) and then C < Natural (Last2.Length) and then C < Natural (F.Cams.Length) then
               Cam_Feed (Ws (C), (if Used >= 2 then Last2 (C) else Plug.Cam'(others => <>)), Last (C), F.Cams (C));
            end if;
         end loop;
         Last2 := Last; Last := F.Cams;
         --  停稳 = 给过画面的每一台都停了(一台都没给过 ⇒ 没有证据,不说停了)
         declare
            Any_Seen : Boolean := False;
            All_Done : Boolean := True;
         begin
            for C in 0 .. N - 1 loop
               if Ws (C).Seen then
                  Any_Seen := True;
                  All_Done := All_Done and then Ws (C).Done;
               end if;
            end loop;
            Ok := Any_Seen and then All_Done;
         end;
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
   --  Why:这一组为什么收了 —— Reached = 差不到 Tol(给了才判)而且没在往远走;Pressed / Stopped = 动起来以后离目标还差的不再变小
   --  (压的那种步 / 别的;Stopped_Shrinking);Waited = 一点都没动起来,等满了量过的起效拍数(Start_Cap);No_Pose = 这一拍没有这条臂的位姿读数
   type Stop_Why is (Going, Reached, Pressed, Stopped, Waited, No_Pose);
   package Cam_Watch_Vectors is new Ada.Containers.Vectors (Natural, Cam_Watch);
   type Run is record
      Arm : Natural := 0;
      C : Plug.Cmd;
      P0, Prev : Plug.Arm_Pose := [others => 0.0];
      Tol, Tol_Rot : Long_Float := 0.0;
      Sub_Frames : Natural := 0;
      Arrived : Boolean := False;
      Prev_Rem_T : Long_Float := Long_Float'Last;   --  上一拍离目标还差几档(平移、转动各按各自那一档折了加起来)
      Wt : Settle_Watch;                --  停没停(离目标还差的;"停在这儿"那种命令看每拍挪了多少)
      Eyes : Ints;                      --  长在这条臂上的眼(Selfmap.Graph.Eyes_On)
      Eyes_Set : Boolean := False;
      Img_W : Cam_Watch_Vectors.Vector; --  读数停了以后,这几只眼的画面停没停(Cam_Feed;画面比读数晚的那一拍在这儿等出来)
      Rd_Done : Boolean := False;       --  读数停了(到了 / 离目标还差的不再变小)
      Halted : Boolean := False;        --  途中 Watch 叫停过
      Started : Boolean := False;       --  这一条发出去以后读数动起来过(离目标少了超过这一条的百分之一)
      Send : Boolean := True;
      Why : Stop_Why := Going;
      Track_T, Track_R : Floats;        --  (V5)发出后逐拍走到这一条的几成(平移 / 转动;这一条没有那一样 = 空)
      Busy : Long_Float := 0.0;         --  (V5)一拍最多花了几秒(判停 + 发命令)
   end record;
   package Run_Vectors is new Ada.Containers.Vectors (Natural, Run);

   function New_Run (Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats; F : Plug.Frame; Tol, Tol_Rot : Long_Float) return Run is
      R : Run;
   begin
      R.Arm := Arm; R.Tol := Tol; R.Tol_Rot := Tol_Rot;
      R.C.Kind := Plug.Ee; R.C.Arm := Arm; R.C.Pose := Target; R.C.Jaw := Jaw;
      R.P0 := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      R.Prev := R.P0;
      Watch_Reset (R.Wt, Chan.Per_Arm / Chan.Pos_Channels);   --  两样:平移、转动
      declare
         Cmd : constant Table.Vec := Chan.Delivered (R.P0, Target);
      begin
         Watch_Peak (R.Wt, 0, Table.Norm (Cmd, Chan.Pos_Channels));
         Watch_Peak (R.Wt, 1, Rot_Len (Cmd));
      end;
      return R;
   end New_Run;

   --  一点都没动起来的那一条最多等几拍:量过的起效拍数(Settle);开机还一个都没量过 ⇒ Unmeasured_Start
   --  (那时身体多久起效还不知道:晚两拍的身体照样要等它动起来,一组推不动的读数也不能一直等下去)
   Unmeasured_Start : constant := 12;
   function Start_Cap (M : Body_Map) return Natural is (if M.Settle > 0 then M.Settle else Unmeasured_Start);

   --  这一拍对这一组:看它停没停、到没到、要不要重发(Prev_Cams = 上一拍的画面)
   procedure Judge (M : Body_Map; F : Plug.Frame; Prev_Cams, Prev2_Cams : Plug.Cam_Vectors.Vector; Press : Boolean; Watch : Watcher; R : in out Run) is
      Arm : constant Natural := R.Arm;
      procedure Restart_Watches is
      begin
         Watch_Reset (R.Wt, Natural (R.Wt.Done.Length));
         R.Rd_Done := False;
      end Restart_Watches;
   begin
      if not R.Eyes_Set then
         R.Eyes := Selfmap.Graph.Eyes_On (M, Arm);
         R.Eyes_Set := True;
      end if;
      R.Sub_Frames := R.Sub_Frames + 1;
      if Arm >= Natural (F.EE.Length) then
         R.Why := No_Pose;
         return;
      end if;
      --  途中每一拍看一眼:出事就把目标改成"停在此刻的位姿",同一条发命令的路再发一次(从这一刻起重新判停)
      if Watch /= null and then not R.Halted and then Watch (F) then
         R.Halted := True;
         R.C.Pose := F.EE (Arm);
         R.Send := True;
         R.P0 := F.EE (Arm); R.Prev := R.P0; R.Started := False; R.Sub_Frames := 0;
         R.Prev_Rem_T := Long_Float'Last;
         Restart_Watches;
      end if;
      declare
         Cmd : constant Table.Vec := Chan.Delivered (R.P0, R.C.Pose);
         Cmd_T : constant Long_Float := Table.Norm (Cmd, Chan.Pos_Channels);
         Cmd_R : constant Long_Float := Rot_Len (Cmd);
         Hold : constant Boolean := Cmd_T <= 0.0 and then Cmd_R <= 0.0;   --  这一条是"停在这儿"
         Miss : constant Table.Vec := Chan.Delivered (F.EE (Arm), R.C.Pose);
         Rem_T : constant Long_Float := Table.Norm (Miss, Chan.Pos_Channels);
         Rem_R : constant Long_Float := Rot_Len (Miss);
         D : constant Table.Vec := Chan.Delivered (R.Prev, F.EE (Arm));     --  这一拍挪了多少
         Dc : constant Table.Vec := Chan.Delivered (R.P0, F.EE (Arm));
         Ct2 : constant Long_Float := Cmd (0) ** 2 + Cmd (1) ** 2 + Cmd (2) ** 2;
         Cr2 : constant Long_Float := Cmd (3) ** 2 + Cmd (4) ** 2 + Cmd (5) ** 2;
         Moves : Floats;
         Have : Bools;
      begin
         --  (V5)这一拍走到这一条的几成:从起点挪的沿命令方向的那一份 ÷ 命令的长(平移、转动各算各的)
         if Ct2 > 0.0 then
            R.Track_T.Append ((Dc (0) * Cmd (0) + Dc (1) * Cmd (1) + Dc (2) * Cmd (2)) / Ct2);
         end if;
         if Cr2 > 0.0 then
            R.Track_R.Append ((Dc (3) * Cmd (3) + Dc (4) * Cmd (4) + Dc (5) * Cmd (5)) / Cr2);
         end if;
         --  动起来了 = 离目标少了超过这一条的百分之一(晚几拍起效的身体:起效之前读数不动,不算停)
         R.Started := R.Started or else Hold
           or else (Cmd_T > 0.0 and then Cmd_T - Rem_T > Negligible * Cmd_T)
           or else (Cmd_R > 0.0 and then Cmd_R - Rem_R > Negligible * Cmd_R);
         --  看哪一样停没停:往一个目标走 ⇒ 离目标还差多少(匀速走着的那一段每拍都在变小,不会被当成停了;被挡住 / 到了 ⇒ 不再变小);
         --  停在这儿 ⇒ 每拍挪了多少。这一条不动的那一样(只平移的命令的转动)不等
         if Hold then
            Moves.Append (Table.Norm (D, Chan.Pos_Channels)); Moves.Append (Rot_Len (D));
            Have.Append (True); Have.Append (True);
         else
            Moves.Append (Rem_T); Moves.Append (Rem_R);
            Have.Append (Cmd_T > 0.0); Have.Append (Cmd_R > 0.0);
         end if;
         if R.Started then
            Watch_Feed (R.Wt, Moves, Have);
         end if;
         --  到了 = 差不到这一档(给了才判),而且没在往远走(冲过头往回摆的那一拍不算;平移、转动折成各自那一档的个数加起来看,
         --  这一条没要转的那一点转动读数的起伏不算往远走)
         declare
            N_Now : constant Long_Float := (if R.Tol > 0.0 then Rem_T / R.Tol else 0.0) + (if R.Tol_Rot > 0.0 then Rem_R / R.Tol_Rot else 0.0);
         begin
            R.Arrived := R.Tol > 0.0 and then Rem_T <= R.Tol and then Rem_R <= R.Tol_Rot and then N_Now <= R.Prev_Rem_T;
            R.Prev_Rem_T := N_Now;
         end;
         R.Prev := F.EE (Arm);
         --  画面:读数停了(到了 / 不再变小)以后,等这条臂自己那几只眼的画面也停了(Cam_Feed,同一种判法)——
         --  画面比读数晚一拍的身体,读数停了那一拍的画面还是上一拍的
         if not R.Rd_Done and then (R.Arrived or else (R.Started and then Watch_All_Done (R.Wt))) then
            R.Rd_Done := True;
            R.Img_W.Clear;
            for E of R.Eyes loop
               R.Img_W.Append (Cam_Watch'(others => <>));
            end loop;
         end if;
         if R.Rd_Done then
            for I in 0 .. Natural (R.Eyes.Length) - 1 loop
               declare
                  E : constant Natural := Natural (R.Eyes (I));
                  Cw : Cam_Watch := R.Img_W (I);
               begin
                  if E < Natural (Prev_Cams.Length) and then E < Natural (Prev2_Cams.Length) and then E < Natural (F.Cams.Length) then
                     Cam_Feed (Cw, Prev2_Cams (E), Prev_Cams (E), F.Cams (E));
                     R.Img_W.Replace_Element (I, Cw);
                  end if;
               end;
            end loop;
         end if;
      end;
      --  到过的范围(09-29):反解被"到过的范围 + 往外一步"截住了(Plug.Held_Back)⇒ 不等停稳:手一动、到过的范围一长(Held_Grown),
      --  这一拍就按此刻的读数重解、重发 —— 目标跟着手往前一步,大转一条 Go 里连着走完(V1B63:等停稳再发,碰指尖 520 → 1012 拍;
      --  快步不重发,一大步只走三成、被认成碰到)。手停在真的尽头 / 碰上东西 ⇒ 范围不再长 ⇒ 照常等停下(尽头由 Jointboot 核)。
      --  每重发一次从头判停(范围只会长到量过的尽头,重发的次数有底;原来另有一道"总拍数 20 × (12 + Settle)"的上限,删了)
      declare
         use type Plug.Limit_State;
         Ls : constant Plug.Limit_State := (if R.Arrived then Plug.Free else Plug.Held_Back (Arm));
      begin
         if Ls = Plug.Held_Grown then
            R.Send := True; R.Sub_Frames := 0; R.Started := False;
            Restart_Watches;
         elsif R.Rd_Done and then (for all Cw of R.Img_W => Cw.Done or else not Cw.Seen) then
            R.Why := (if R.Arrived then Reached elsif Press then Pressed else Stopped);
         elsif not R.Started and then R.Sub_Frames >= Start_Cap (M) then
            R.Why := Waited;
         end if;
      end;
   end Judge;

   --  运动命令只从这一处发出(自由棘轮:位姿、关节目标都经 Go 走到这里;一组、几组、关节那一支同一个口)
   function Issue (L : in out Plug.Link; C : Plug.Cmd) return Boolean is (Plug.Act (L, C));

   --  几组位姿目标一起走(一拍一条命令带几组的目标):每组一份账、各自判停,都收了才完。
   --  每一拍把要发的那几组各发一回:在按拍对齐的手的任务里 Plug.Act 只记下那一组的目标,主线程把几组合成一条发出去
   procedure Go_Runs (L : in out Plug.Link; M : Body_Map; Rs : in out Run_Vectors.Vector; F : in out Plug.Frame; Frames : out Natural;
                      Ok : out Boolean; Press : Boolean; Watch : Watcher) is
      use type Ada.Calendar.Time;
      T_Got : Ada.Calendar.Time := Ada.Calendar.Clock;   --  这一帧收到的时刻(第一拍之前 = 进门)
      Prev_Cams, Prev2_Cams : Plug.Cam_Vectors.Vector := F.Cams;   --  上一拍、上上拍的画面(判画面停没停)
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
         declare
            Busy : constant Long_Float := Long_Float (Ada.Calendar.Clock - T_Got);
         begin
            for I in 0 .. Natural (Rs.Length) - 1 loop
               if Rs (I).Busy < Busy then
                  declare
                     R : Run := Rs (I);
                  begin
                     R.Busy := Busy;
                     Rs.Replace_Element (I, R);
                  end;
               end if;
            end loop;
         end;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         T_Got := Ada.Calendar.Clock;
         Frames := Frames + 1;
         declare
            All_Done : Boolean := True;
         begin
            for I in 0 .. Natural (Rs.Length) - 1 loop
               if Rs (I).Why = Going then
                  declare
                     R : Run := Rs (I);
                  begin
                     Judge (M, F, Prev_Cams, Prev2_Cams, Press, Watch, R);
                     Rs.Replace_Element (I, R);
                     if R.Why = Going then
                        All_Done := False;
                     end if;
                  end;
               end if;
            end loop;
            Prev2_Cams := Prev_Cams; Prev_Cams := F.Cams;
            exit when All_Done;
         end;
      end loop;
   end Go_Runs;

   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Press : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0; Tol_Rot : Long_Float := 0.0;
                 Tols : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Track_T, Track_R : access Floats := null) is
      C : Plug.Cmd;
      Send : Boolean := True;
      --  关节目标:看哪几组读数、各自的目标
      W_G : Ints;
      W_Q : Plug.Floats_Vectors.Vector;
      Prev_All : Plug.Floats_Vectors.Vector;
      Rem0, Prev_Rem : Floats;           --  每组发命令时、上一拍离目标还差多少(组里差得最多的那个关节)
      Is_Joint : constant Boolean := Group >= 0 or else not Groups.Is_Empty;
      Started : Boolean := False;        --  这一条发出去以后读数动起来过(有一组离目标少了超过它的百分之一)
      Wt : Settle_Watch;
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
            if Track_T /= null then
               Track_T.all := Rs (0).Track_T;
            end if;
            if Track_R /= null then
               Track_R.all := Rs (0).Track_R;
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
      --  一组离目标还差多少:组里差得最多的那个关节
      declare
         function Gap (Now, Q : Floats) return Long_Float is
            Mx : Long_Float := 0.0;
         begin
            for K in 0 .. Natural'Min (Natural (Now.Length), Natural (Q.Length)) - 1 loop
               Mx := Long_Float'Max (Mx, abs (Now (K) - Q (K)));
            end loop;
            return Mx;
         end Gap;
      begin
         for Gi in 0 .. Natural (W_G.Length) - 1 loop
            declare
               G : constant Integer := W_G (Gi);
               Now : constant Floats := (if G >= 0 and then G < Natural (F.Joints.Length) then F.Joints (Natural (G)) else F64_Vectors.Empty_Vector);
            begin
               Prev_All.Append (Now);
               Rem0.Append (Gap (Now, W_Q (Gi)));
               Prev_Rem.Append (Long_Float'Last);
            end;
         end loop;
      end;
      Watch_Reset (Wt, Natural (W_G.Length));
      for Gi in 0 .. Natural (W_G.Length) - 1 loop
         Watch_Peak (Wt, Gi, Rem0 (Gi));
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
         --  关节目标:"停稳"看这几组关节读数(不看位姿:只报关节的身体没有位姿读数),同位姿那一支一种判法 ——
         --  动起来以后每组离目标还差的不再变小(Stopped_Shrinking)= 停了;"停在这儿"那一组(发命令时就在目标上)看每拍挪了多少;
         --  到了 = 每一组读得到的关节都差不到它的门(Joints_Arrived),而且没在往远走;一点都没动起来 ⇒ 最多等量过的起效拍数(Start_Cap)。
         --  这一拍一组读数都读不到 ⇒ 动没动起来看不见,不白等(同原来:最多等起效拍数)
         declare
            Arr : Boolean := True;          --  这一拍每一组读得到的关节都到了(Joints_Arrived)
            Any_Read : Boolean := False;    --  至少有一组读得到(读不到的组不算,同原来)
            Away : Boolean := False;        --  有一组离目标比上一拍远了
            Moves : Floats;
            Have : Bools;
         begin
            for Gi in 0 .. Natural (W_G.Length) - 1 loop
               declare
                  G : constant Integer := W_G (Gi);
               begin
                  if G >= 0 and then G < Natural (F.Joints.Length) and then Natural (Prev_All (Gi).Length) = Natural (F.Joints (Natural (G)).Length) then
                     declare
                        Now : constant Floats := F.Joints (Natural (G));
                        Gap_Now, Moved : Long_Float := 0.0;
                     begin
                        for K in 0 .. Natural (Now.Length) - 1 loop
                           Moved := Long_Float'Max (Moved, abs (Now (K) - Prev_All (Gi) (K)));
                           if K < Natural (W_Q (Gi).Length) then
                              Gap_Now := Long_Float'Max (Gap_Now, abs (Now (K) - W_Q (Gi) (K)));
                           end if;
                        end loop;
                        Any_Read := True;
                        Arr := Arr and then Joints_Arrived (Now, W_Q (Gi), (if Gi < Natural (Tols.Length) then Tols (Gi) else F64_Vectors.Empty_Vector), Tol);
                        Away := Away or else Gap_Now > Prev_Rem (Gi);
                        Started := Started or else Rem0 (Gi) <= 0.0 or else Rem0 (Gi) - Gap_Now > Negligible * Rem0 (Gi);
                        Moves.Append (if Rem0 (Gi) <= 0.0 then Moved else Gap_Now);   --  "停在这儿"那一组看每拍挪了多少
                        Have.Append (True);
                        Prev_Rem.Replace_Element (Gi, Gap_Now);
                        Prev_All.Replace_Element (Gi, Now);
                     end;
                  else
                     --  读不到这一组:当成这一拍什么都没变(两拍不变就停 —— 没有证据,不白等;同原来)
                     Moves.Append (0.0); Have.Append (True);
                  end if;
               end;
            end loop;
            Started := Started or else not Any_Read;
            if Started then
               Watch_Feed (Wt, Moves, Have);
            end if;
            exit when (Any_Read and then Arr and then not Away)
              or else (Started and then Watch_All_Done (Wt))
              or else (not Started and then Frames >= Start_Cap (M));
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
      Wt : Settle_Watch;
   begin
      Used := 0;
      Ok := True;
      Watch_Reset (Wt, Natural (Group_Moves (F, F, N_Ee, N_Q, N_Jaw).Moves.Length));   --  只取组数
      loop
         exit when Watch_All_Done (Wt);
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Used := Used + 1;
         declare
            G : constant Group_Move := Group_Moves (Prev_F, F, N_Ee, N_Q, N_Jaw);
         begin
            Watch_Feed (Wt, G.Moves, G.Have);
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
      M.Resp := Response_Vectors.To_Vector ((others => <>), Ada.Containers.Count_Type (Arms));
      begin
      for A in 0 .. Arms - 1 loop
         declare
            Tracks : Plug.Floats_Vectors.Vector;   --  (V5)这条臂每一条探针命令逐拍走到它自己的几成
         begin
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
                     Tr_T, Tr_R : aliased Floats;         --  (V5)这一条逐拍走到它自己的几成(平移 / 转动那一份)
                  begin
                     A_Cmd (K) := Amp;
                     Go (L, M, A, Chan.Compose (P0, A_Cmd), Jaw0, F, Deliv, Frames, Ok2, Track_T => Tr_T'Access, Track_R => Tr_R'Access);
                     if not Ok2 then
                        return;
                     end if;
                     Tracks.Append (if K < Chan.Pos_Channels then Tr_T else Tr_R);   --  推的是平移通道 ⇒ 平移那一份,转动通道 ⇒ 转动那一份
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
                     Go (L, M, A, P0, Jaw0, F, Back, Frames_Back, Ok2, Track_T => Tr_T'Access, Track_R => Tr_R'Access);
                     if not Ok2 then
                        return;
                     end if;
                     Tracks.Append (if K < Chan.Pos_Channels then Tr_T else Tr_R);
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
         declare
            Rp : constant Response := Fit_Response (Tracks);
         begin
            M.Resp.Replace_Element (A, Rp);
            Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手的阶跃响应(V5):"
                      & (if Rp.Alpha > 0.0
                         then "命令发出后头 " & Codec.Img (Rp.Dead) & " 拍不动,之后每拍走还差的 " & Codec.Fmt (Rp.Alpha, 3)
                              & "(" & Codec.Img (Rp.N) & " 条探针;拿它重放这几条,逐拍最多差一条的 " & Codec.Fmt (Rp.Err, 3) & ")"
                         else "探针一条都没动起来 ⇒ 量不出(记没量)"));
         end;
         end;
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
                        S.Fracs_T := R.Track_T; S.Fracs_R := R.Track_R; S.Busy := R.Busy;
                        S.Moving := False;   --  (10-01)没有"最多等几拍"了:没动起来的那一条(Waited)就是没动,照常判挡没挡
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

   --  ── 命令 → 动作的阶跃响应(V5)──
   function Predict (R : Response; Beat : Positive) return Long_Float is
     (if R.Alpha <= 0.0 or else Beat <= R.Dead then 0.0 else 1.0 - (1.0 - R.Alpha) ** (Beat - R.Dead));

   function Moved_At (Track : Floats) return Natural is   --  第几拍动起来(走过这一条的百分之一;一直没动 ⇒ 0)
   begin
      for I in 0 .. Natural (Track.Length) - 1 loop
         if Track (I) > Negligible then
            return I + 1;
         end if;
      end loop;
      return 0;
   end Moved_At;

   function Fit_Response (Tracks : Plug.Floats_Vectors.Vector) return Response is
      R : Response;
      Ds, As : Floats;
   begin
      for Tk of Tracks loop
         declare
            B : constant Natural := Moved_At (Tk);
         begin
            if B > 0 then
               Ds.Append (Long_Float (B - 1));
               As.Append (Tk (B - 1));
            end if;
         end;
      end loop;
      if Ds.Is_Empty then
         return R;
      end if;
      R.N := Natural (Ds.Length);
      R.Dead := Natural (Long_Float'Floor (Picture.Quantile (Ds, 0.5)));   --  中位(0.5 分位)
      R.Alpha := Long_Float'Min (1.0, Picture.Quantile (As, 0.5));
      for Tk of Tracks loop
         if Moved_At (Tk) > 0 then
            R.Err := Long_Float'Max (R.Err, Response_Err (R, Tk));
         end if;
      end loop;
      return R;
   end Fit_Response;

   function Response_Err (R : Response; Track : Floats) return Long_Float is
      E : Long_Float := 0.0;
   begin
      for I in 0 .. Natural (Track.Length) - 1 loop
         E := Long_Float'Max (E, abs (Track (I) - Predict (R, I + 1)));
      end loop;
      return E;
   end Response_Err;

   procedure Step_Track (M : Body_Map; S : Leg_Step; Track : out Floats; Notches : out Long_Float) is
      Chs : constant Ints := Selfmap.Graph.Pose_Channels (M, S.Arm);
      Tp : constant Long_Float := (if Natural (Chs.Length) > Chan.Pos_Channels and then Natural (Chs (0)) < Natural (M.Amp.Length)
                                   then M.Amp (Natural (Chs (0))) else 0.0);
      Tr : constant Long_Float := (if Natural (Chs.Length) > Chan.Pos_Channels and then Natural (Chs (Chan.Pos_Channels)) < Natural (M.Amp.Length)
                                   then M.Amp (Natural (Chs (Chan.Pos_Channels))) else 0.0);
      Nt : constant Long_Float := (if Tp > 0.0 then S.Len / Tp else 0.0);
      Nr : constant Long_Float := (if Tr > 0.0 then S.Ang / Tr else 0.0);
   begin
      if Nt >= Nr then
         Track := S.Fracs_T; Notches := Nt;
      else
         Track := S.Fracs_R; Notches := Nr;
      end if;
   end Step_Track;

   function Effect_Miss (R : Response; Track : Floats) return Natural is
      B : constant Natural := Moved_At (Track);
   begin
      return (if B = 0 then 0 elsif B > R.Dead + 1 then B - (R.Dead + 1) else (R.Dead + 1) - B);
   end Effect_Miss;

   --  ── 走近一件东西每一步多大 ──
   function Floor_Step (Noise, Notch, Eye_Rms : Long_Float) return Long_Float is
     (Long_Float'Max (Notch, Stats.Z * Long_Float'Max (Noise, Notch * Eye_Rms)));

   function Careful_Step (Tip_Sd, Miss, Noise, Notch, Eye_Rms : Long_Float) return Long_Float is
      Hand : constant Long_Float := Stats.Z * Sqrt (Tip_Sd ** 2 + Miss ** 2 + Noise ** 2);
   begin
      return Long_Float'Max (Hand / Long_Float (Free_Base), Floor_Step (Noise, Notch, Eye_Rms));
   end Careful_Step;

   function Plan_Approach (Dist, R_Obj, Sd_Target, Tip_Sd, Miss, Noise, Notch, Eye_Rms : Long_Float) return Approach_Plan is
      P : Approach_Plan;
      Sd_Hand : constant Long_Float := Sqrt (Tip_Sd ** 2 + Miss ** 2 + Noise ** 2);
   begin
      P.Lstep := Careful_Step (Tip_Sd, Miss, Noise, Notch, Eye_Rms);
      if Sd_Target = Long_Float'Last or else R_Obj = Long_Float'Last then
         --  它在哪 / 它多大量不出:没有"碰不到它"的那一段,全程小步(照实);到没到只按我自己看得出的那一步判
         P.Res := Floor_Step (Noise, Notch, Eye_Rms); P.Band := Long_Float'Last; P.Clear := Long_Float'First;
         return P;
      end if;
      P.Res := Long_Float'Max (Stats.Z * Sqrt (Sd_Target ** 2 + Sd_Hand ** 2), Floor_Step (Noise, Notch, Eye_Rms));
      P.Band := R_Obj + P.Res;
      P.Clear := Dist - P.Band - Long_Float (Free_Base) * P.Lstep;
      return P;
   end Plan_Approach;

   function Gear_Bound (Gear : String; Small, Large : Long_Float) return Long_Float is
     (if Gear = "small" then Small
      elsif Gear = "large" then Long_Float'Max (Small, Large)
      elsif Gear = "medium" then Sqrt (Small * Long_Float'Max (Small, Large))
      else Long_Float'Last);

   procedure Walk_To (L : in out Plug.Link; M : Body_Map; G : Leg; Lim : Limits; Res, Res_Rot : Long_Float; Max_Steps : Natural;
                      F : in out Plug.Frame; W : in out Walk; Went, Turned : out Long_Float; Steps : out Natural; Why : out Walk_End) is
      Prev_T, Prev_R : Long_Float := Long_Float'Last;
      Legs : Leg_Vectors.Vector;
   begin
      Went := 0.0; Turned := 0.0; Steps := 0; Why := Lost_Link;
      Legs.Append (G);
      loop
         if G.Arm >= Natural (F.EE.Length) then
            Why := Lost_Link;
            return;
         end if;
         declare
            D : constant Table.Vec := Chan.Delivered (F.EE (G.Arm), G.Goal);
            Left_T : constant Long_Float := Table.Norm (D, Chan.Pos_Channels);
            Left_R : constant Long_Float := Rot_Len (D);
            Rep : Leg_Step_Vectors.Vector;
            Fr : Natural;
            Ok : Boolean;
         begin
            if Left_T <= Res and then Left_R <= Res_Rot then
               Why := Arrived;
               return;
            end if;
            if not Gained (Prev_T, Left_T, Res) and then not Gained (Prev_R, Left_R, Res_Rot) then
               Why := No_Gain;
               return;
            end if;
            if Max_Steps > 0 and then Steps >= Max_Steps then
               Why := Max_Steps_Done;
               return;
            end if;
            Prev_T := Left_T; Prev_R := Left_R;
            Step (L, M, Legs, Lim, F, W, Rep, Fr, Ok);
            if not Ok or else Rep.Is_Empty then
               Why := Lost_Link;
               return;
            end if;
            Steps := Steps + 1;
            Went := Went + Rep (0).Went; Turned := Turned + Rep (0).Turned;
            if Rep (0).Blocked_T or else Rep (0).Blocked_R then
               Why := Was_Blocked;
               return;
            end if;
         end;
      end loop;
   end Walk_To;

end Selfmap;
