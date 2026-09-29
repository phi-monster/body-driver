with Ada.Text_IO; use Ada.Text_IO;
with Ada.Calendar;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Interfaces; use Interfaces;
with Codec;
with Lockstep;
package body Plug is
   use Msgpack;
   function U32_To_F32 is new Ada.Unchecked_Conversion (Unsigned_32, Float);
   use type Websocket.Op;

   Hook_P : Pose_Hook := null;
   Hook_C : Cmd_Hook := null;
   procedure Set_Hooks (P : Pose_Hook; Q : Cmd_Hook) is
   begin
      Hook_P := P; Hook_C := Q;
   end Set_Hooks;
   Hook_L : Limit_Hook := null;
   procedure Set_Limit (H : Limit_Hook) is
   begin
      Hook_L := H;
   end Set_Limit;
   function Held_Back (Arm : Natural) return Limit_State is (if Hook_L = null then Free else Hook_L (Arm));
   Hook_R : Reach_Hook := null;
   procedure Set_Reach (R : Reach_Hook) is
   begin
      Hook_R := R;
   end Set_Reach;
   procedure Reach (Arm : Natural; Pose : Arm_Pose; Pos_Err, Rot_Err : out Long_Float; Ok : out Boolean) is
   begin
      Pos_Err := 0.0; Rot_Err := 0.0; Ok := Hook_R /= null;
      if Ok then
         Hook_R (Arm, Pose, Pos_Err, Rot_Err);
      end if;
   end Reach;

   --  按拍对齐时记下的目标(见 spec 的 Lock_Begin):每组关节读数一个目标(空 = 没给)、每只手的爪子一个目标(空 = 没给),
   --  这一拍有没有变;主线程收的那一帧(手的任务醒来拿它)
   Lock_Q : Floats_Vectors.Vector;
   Lock_Jaw : Floats_Vectors.Vector;
   Lock_Changed : Boolean := False;
   Lock_F : Frame;
   Lock_Ok : Boolean := True;

   function Arms (L : Link) return Natural is
   begin
      if not L.Lay.EE.Is_Empty then
         return Natural'Min (Natural (L.Lay.EE.Length), Natural'Max (1, Natural (L.Lay.Jaw.Length)));
      end if;
      return Natural (L.Lay.Joints.Length);
   end Arms;

   function Joint_Mode (L : Link) return Boolean is (L.Lay.EE.Is_Empty);

   function Steps (L : Link) return Natural is (if L.Seq >= L.Ep_Seq0 then L.Seq - L.Ep_Seq0 else L.Seq);

   function Beat_Index (L : Link; Seq : Natural) return Integer is
   begin
      if L.Beats.Is_Empty or else Seq < L.Beats.First_Element.Seq or else Seq > L.Beats.Last_Element.Seq then
         return -1;
      end if;
      return Seq - L.Beats.First_Element.Seq;   --  帧号连着(每收一帧记一拍)
   end Beat_Index;
   function Joints_At (L : Link; Seq : Natural) return Floats_Vectors.Vector is
      K : constant Integer := Beat_Index (L, Seq);
   begin
      return (if K >= 0 then L.Beats (K).Joints else Floats_Vectors.Empty_Vector);
   end Joints_At;
   function Reported_At (L : Link; Seq : Natural) return Pose_Vectors.Vector is
      K : constant Integer := Beat_Index (L, Seq);
   begin
      return (if K >= 0 then L.Beats (K).Reported_EE else Pose_Vectors.Empty_Vector);
   end Reported_At;
   function Image_Lag (L : Link; Cam, Group, From_Seq : Natural; Corr : out Floats) return Integer is
      Best : Integer := 0;
      Best_C : Long_Float := Long_Float'First;
   begin
      Corr := Zeros (2 * Max_Lag + 1);
      for Lag in -Max_Lag .. Max_Lag loop
         declare
            Sx, Sy, Sxx, Syy, Sxy : Long_Float := 0.0;
            N : Natural := 0;
         begin
            for K in 0 .. Natural (L.Beats.Length) - 1 loop
               declare
                  Kq : constant Integer := K - Lag;
               begin
                  --  这台相机那一拍没量成(那一拍或上一拍没收到它的画面)⇒ 不参与:不当成"画面没变"
                  if Kq >= 0 and then Kq < Natural (L.Beats.Length) and then L.Beats (K).Seq >= From_Seq and then L.Beats (Kq).Seq >= From_Seq
                    and then Cam < Natural (L.Beats (K).Img_Chg.Length) and then Cam < Natural (L.Beats (K).Img_Ok.Length)
                    and then L.Beats (K).Img_Ok (Cam) and then Group < Natural (L.Beats (Kq).Q_Chg.Length)
                  then
                     declare
                        X : constant Long_Float := L.Beats (K).Img_Chg (Cam);
                        Y : constant Long_Float := L.Beats (Kq).Q_Chg (Group);
                     begin
                        Sx := Sx + X; Sy := Sy + Y; Sxx := Sxx + X * X; Syy := Syy + Y * Y; Sxy := Sxy + X * Y;
                        N := N + 1;
                     end;
                  end if;
               end;
            end loop;
            if N >= 2 then
               declare
                  Nn : constant Long_Float := Long_Float (N);
                  Vx : constant Long_Float := Sxx - Sx * Sx / Nn;
                  Vy : constant Long_Float := Syy - Sy * Sy / Nn;
               begin
                  if Vx > 0.0 and then Vy > 0.0 then
                     Corr.Replace_Element (Lag + Max_Lag, (Sxy - Sx * Sy / Nn) / Sqrt (Vx * Vy));
                     if Corr (Lag + Max_Lag) > Best_C then
                        Best_C := Corr (Lag + Max_Lag); Best := Lag;
                     end if;
                  end if;
               end;
            end if;
         end;
      end loop;
      return Best;
   end Image_Lag;

   function Reset_Pending (L : Link) return Boolean is (L.Reset_Flag);

   function Take_Reset (L : in out Link) return Boolean is
      R : constant Boolean := L.Reset_Flag;
   begin
      L.Reset_Flag := False;
      return R;
   end Take_Reset;

   function Nums_At (L : Link; P : Layout.Path) return Floats is
   begin
      if L.Last_Obs < 0 then
         return F64_Vectors.Empty_Vector;
      end if;
      return Numbers (L.Last, Layout.Find (L.Last, L.Last_Obs, P));
   end Nums_At;

   --  一条动作里的一个键:名字 + 那一串数。先凑齐再数有几个键(读数空、这回不发的键不占 Put_Map 的个数)
   procedure Put_Keys (S : in out Buf; Keys : Strs; Vals : Floats_Vectors.Vector) is
   begin
      Put_Map (S, Natural (Keys.Length));
      for K in 0 .. Natural (Keys.Length) - 1 loop
         Put_Str (S, Keys (K));
         Put_Array (S, Natural (Vals (K).Length));
         for X of Vals (K) loop
            Put_Float (S, X);
         end loop;
      end loop;
   end Put_Keys;

   --  「照现在这样保持」:把此刻报的位姿 / 关节原样回声,每组抓握照这一组读数的个数发(Jaw_Values,没有新命令)。零假设,一次回声。
   --  读数空的键不发:不知道对方要几个数,也不编(09-30:原来抓握一栏每只手只发 1 个数、没读数就发 1.0 —— x5"1 = 张开"的约定)
   function Hold_Action (L : in out Link) return Buf is
      S : Buf;
      N : constant Natural := Arms (L);
      Keys : Strs;
      Vals : Floats_Vectors.Vector;
      None : Cmd;
      procedure Add (Name : String; V : Floats) is
      begin
         if not V.Is_Empty then
            Keys.Append (Name); Vals.Append (V);
         end if;
      end Add;
   begin
      if N = 0 or else L.Last_Obs < 0 then
         return S;
      end if;
      for I in 0 .. N - 1 loop
         if Joint_Mode (L) then
            Add (Layout.Last_Seg (L.Lay.Joints (I)), Nums_At (L, L.Lay.Joints (I)));
         else
            Add (Layout.Last_Seg (L.Lay.EE (I)), Nums_At (L, L.Lay.EE (I)));
         end if;
         if not L.Lay.Jaw.Is_Empty then
            declare
               Ji : constant Natural := Natural'Min (I, Natural (L.Lay.Jaw.Length) - 1);
            begin
               Add (Layout.Last_Seg (L.Lay.Jaw (Ji)), Jaw_Values (L, Ji, False, None, Nums_At (L, L.Lay.Jaw (Ji))));
            end;
         end if;
      end loop;
      Put_Keys (S, Keys, Vals);
      return S;
   end Hold_Action;

   procedure Reply (L : in out Link; Req : Doc; Kind : String; Payload : Buf) is
      S : Buf;
      N : Natural := 4;   --  message_type, message_id, step, payload
      Extras : Strs;
      Ok : Boolean;
   begin
      Extras.Append ("evaluation_id"); Extras.Append ("action_case_id"); Extras.Append ("trial_id");
      Extras.Append ("repeat_index"); Extras.Append ("sent_at");
      for E of Extras loop
         if Key (Req, 0, E) >= 0 then
            N := N + 1;
         end if;
      end loop;
      Put_Map (S, N);
      Put_Str (S, "message_type"); Put_Str (S, Kind);
      Put_Str (S, "message_id"); Put_Node (S, Req, Key (Req, 0, "message_id"));
      for E of Extras loop
         declare
            K : constant Integer := Key (Req, 0, E);
         begin
            if K >= 0 then
               Put_Str (S, E); Put_Node (S, Req, K);
            end if;
         end;
      end loop;
      Put_Str (S, "step");
      declare
         K : constant Integer := Key (Req, 0, "step");
      begin
         if K >= 0 then
            Put_Node (S, Req, K);
         else
            Put_Int (S, 0);
         end if;
      end;
      Put_Str (S, "payload");
      for B of Payload loop
         S.Append (B);
      end loop;
      Websocket.Send_Binary (L.Conn, S, Ok);
   end Reply;

   --  抽这条连接直到拿到一帧新观测。握手照回,要动作就交出攥着的那条。
   function Pump (L : in out Link) return Boolean is
      Kind : Websocket.Op;
      Data : Buf;
      Ok : Boolean;
      Idle : Natural := 0;
   begin
      loop
         Websocket.Read_Message (L.Conn, Kind, Data, Ok);
         if not Ok or else Kind = Websocket.Op_Close then
            Put_Line ("[链] 线断了 ⇒ 在同一个口上等对方重新接上(身体量到的东西都留着)…");
            Websocket.Accept_Client (L.Conn, Ok);
            if not Ok then
               Put_Line ("[链] 没等到 ⇒ 取不到画面");
               return False;
            end if;
            --  攥着的命令和上一条命令都留着:断线重连不改变世界。上一条也不能丢 —— 丢了就退回"照现在报的位姿保持",而报的位姿落后一步,
            --  手臂正在走时等于把它拽回起点(EK:重连后第一步实到 0.000,就是这么被拽回去的)
            Put_Line ("[链] 重新接上了(不当作新的一集:对方明说 reset 才算;攥着的命令照发)");
         elsif Kind = Websocket.Op_Binary then
            declare
               D : Doc;
            begin
               if Decode (Data, D) then
                  declare
                     MT : constant String := Text (D, Key (D, 0, "message_type"));
                     P : constant Integer := Key (D, 0, "payload");
                     Fn : constant String := Text (D, Key (D, P, "func_name"));
                     Obs : Integer := Key (D, P, "obs");
                     Ack : constant String :=
                       (if MT = "hello" then "hello_ack"
                        elsif MT = "prepare_case" then "prepare_case_ack"
                        elsif MT = "reset" then "reset_result"
                        elsif MT = "call" then "call_result"
                        elsif MT = "infer" then "infer_result"
                        elsif MT = "trial_end" then "trial_end_ack"
                        elsif MT = "heartbeat" then "heartbeat_ack"
                        else "");
                     New_Frame : Boolean := False;
                     Payload : Buf;
                  begin
                     Idle := Idle + 1;
                     if Idle mod 1000 = 0 then
                        Put_Line ("[链] 对方连发" & Natural'Image (Idle) & " 条没带画面的消息,线还通,继续等");
                     end if;
                     if MT = "reset" then
                        L.Reset_Flag := True;
                        L.Ep_Seq0 := L.Seq;   --  新的一集从零数拍
                        L.Jaw_Set.Clear;      --  新的一集爪子回到对方的初始状态,上一集给过的目标作废
                        L.Jaw_Sent.Clear;     --  上一集发出去的那一串也作废(没读数的那一拍不许把上一集的数发进新的一集)
                     end if;
                     if Ack /= "" then
                        if Obs < 0 then
                           Obs := Key (D, P, "observation");
                        end if;
                        if Obs >= 0 then
                           L.Last := D;
                           L.Last_Obs := Obs;
                           New_Frame := True;
                        end if;
                        if Fn = "get_action" then
                           declare
                              Action : Buf;
                           begin
                              if L.Has_Pending then
                                 Action := L.Pending;
                                 L.Last_Sent := L.Pending;
                                 L.Has_Last := True;
                                 L.Has_Pending := False;
                              elsif L.Has_Last then
                                 Action := L.Last_Sent;
                              else
                                 Action := Hold_Action (L);
                              end if;
                              Put_Map (Payload, 1);
                              Put_Str (Payload, "result");
                              if Action.Is_Empty then
                                 Put_Array (Payload, 0);
                              else
                                 Put_Array (Payload, 1);
                                 for B of Action loop
                                    Payload.Append (B);
                                 end loop;
                              end if;
                           end;
                        elsif Ack = "hello_ack" then
                           Put_Map (Payload, 3);
                           Put_Str (Payload, "ok"); Put_Bool (Payload, True);
                           Put_Str (Payload, "server"); Put_Str (Payload, "xpolicylab_policy_server");
                           Put_Str (Payload, "server_instance_id"); Put_Str (Payload, "body-driver");
                        else
                           Put_Map (Payload, 1);
                           Put_Str (Payload, "ok"); Put_Bool (Payload, True);
                        end if;
                        Reply (L, D, Ack, Payload);
                        if New_Frame then
                           return True;
                        end if;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
   end Pump;

   procedure Boot (Port : Natural; L : in out Link; Ok : out Boolean) is
      Tries : Natural := 0;
   begin
      Ok := False;
      Put_Line ("[装] 在" & Natural'Image (Port) & " 上等这台机器人连过来…");
      Websocket.Listen (Port, L.Conn, Ok);
      if not Ok then
         return;
      end if;
      Put_Line ("[装] 接上了。先听一帧,认这台机器人报的东西长什么样。");
      loop
         if not Pump (L) then
            Ok := False;
            return;
         end if;
         Layout.Recognise (L.Last, L.Last_Obs, L.Lay);
         declare
            M : constant String := Layout.Missing (L.Lay);
         begin
            if M = "" then
               L.Have_Layout := True;
               Layout.Say (L.Lay);
               Ok := True;
               return;
            end if;
            Tries := Tries + 1;
            if Tries <= 3 then
               Put_Line ("[装] 拿到观测了,但认不出来 ⇒ " & M);
               Layout.Say (L.Lay);
            end if;
            if Tries > 4000 then
               Ok := False;
               return;
            end if;
         end;
      end loop;
   end Boot;

   procedure Frame_Of (L : Link; F : in out Frame) is
   begin
      F.Seq := L.Seq;
      for P of L.Lay.Joints loop
         F.Joints.Append (Nums_At (L, P));
      end loop;
      for P of L.Lay.EE loop
         declare
            A : constant Floats := Nums_At (L, P);
            Pose : Arm_Pose := [others => 0.0];
         begin
            if Natural (A.Length) = 7 then
               for I in 0 .. 6 loop
                  Pose (I) := A (I);
               end loop;
            end if;
            F.Reported_EE.Append (Pose);   --  V1b 3c:身体报的位姿驱动不读(F.EE 由运动学按关节读数算,Pose_Hook 填)
         end;
      end loop;
      for P of L.Lay.Jaw loop
         declare
            A : constant Floats := Nums_At (L, P);
         begin
            F.Jaw.Append (A);   --  整组留下,不再只取 A (0)
         end;
      end loop;
      declare
         Ins : constant Integer := Key (L.Last, L.Last_Obs, "instruction");
      begin
         if Ins >= 0 then
            F.Instruction := To_Unbounded_String (Text (L.Last, Ins));
         end if;
      end;
      for Ci in 0 .. Natural (L.Lay.Cams.Length) - 1 loop
         declare
            N : constant Integer := Layout.Find (L.Last, L.Last_Obs, L.Lay.Cams (Ci));
            W, H : Natural;
            C : Cam;   --  没收到这台的画面 ⇒ 这一格就是占位(W = H = 0、缓冲空),下标照样占住
            First, Len : Natural;
         begin
            if Layout.Is_Image (L.Last, N, W, H) then
               Nd_Data (L.Last, N, First, Len);
               if Len >= W * H * 3 then
                  C.W := W; C.H := H;
                  --  预分配后按下标写(比逐字节 Append 快好几倍:一帧三台相机近两百万个像素)
                  C.RGB := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H * 3));
                  C.Gray := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H));
                  for I in 0 .. W * H - 1 loop
                     declare
                        R : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I));
                        G : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I + 1));
                        B : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I + 2));
                     begin
                        C.RGB.Replace_Element (3 * I, U8 (R)); C.RGB.Replace_Element (3 * I + 1, U8 (G)); C.RGB.Replace_Element (3 * I + 2, U8 (B));
                        C.Gray.Replace_Element (I, U8 ((R * 299 + G * 587 + B * 114) / 1000));
                     end;
                  end loop;
                  --  身体另外给的深度图、相机内参:认得出(Layout 按形状认,免得当成别的读数),但驱动不读(铁律 1,2026-09-26 owner:
                  --  每个量只有一种量法 —— 远近、焦距都由身体自己量;以前"给了就用、没给就量"是两种量法)。Has_Depth / Has_K 永远是 False
               end if;
            end if;
            F.Cams.Append (C);
         end;
      end loop;
   end Frame_Of;

   procedure Note_Beat (L : in out Link; F : Frame) is
      B : Beat;
   begin
      B.Seq := F.Seq; B.Joints := F.Joints; B.Reported_EE := F.Reported_EE;
      for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
         declare
            W : constant Natural := F.Cams (Ci).W;
            H : constant Natural := F.Cams (Ci).H;
            Sum : Long_Float := 0.0;
            Cnt : Natural := 0;
         begin
            --  这一拍、上一拍都有这台的画面(而且一样大)才量;占位的那一拍、占位之后的那一拍这台都记"没量"(Img_Ok = False)
            if Has_Picture (F.Cams (Ci)) and then Ci < Natural (L.Prev_Gray.Length) and then Natural (L.Prev_Gray (Ci).Length) = W * H then
               declare
                  G0 : Buf renames L.Prev_Gray (Ci);
                  G1 : Buf renames F.Cams (Ci).Gray;
               begin
                  for Y in 0 .. (H - 1) / Img_Stride loop
                     for X in 0 .. (W - 1) / Img_Stride loop
                        declare
                           I : constant Natural := Y * Img_Stride * W + X * Img_Stride;
                        begin
                           Sum := Sum + abs (Long_Float (G1 (I)) - Long_Float (G0 (I)));
                           Cnt := Cnt + 1;
                        end;
                     end loop;
                  end loop;
               end;
            end if;
            B.Img_Chg.Append (if Cnt > 0 then Sum / Long_Float (Cnt) else 0.0);
            B.Img_Ok.Append (Cnt > 0);
         end;
      end loop;
      for Gi in 0 .. Natural (F.Joints.Length) - 1 loop
         declare
            Mx : Long_Float := 0.0;
         begin
            if not L.Beats.Is_Empty and then Gi < Natural (L.Beats.Last_Element.Joints.Length) then
               declare
                  Q0 : constant Floats := L.Beats.Last_Element.Joints (Gi);
               begin
                  for K in 0 .. Natural'Min (Natural (Q0.Length), Natural (F.Joints (Gi).Length)) - 1 loop
                     Mx := Long_Float'Max (Mx, abs (F.Joints (Gi) (K) - Q0 (K)));
                  end loop;
               end;
            end if;
            B.Q_Chg.Append (Mx);
         end;
      end loop;
      L.Beats.Append (B);
      if Natural (L.Beats.Length) > Keep_Beats then
         L.Beats.Delete_First (Ada.Containers.Count_Type (Natural (L.Beats.Length) - Keep_Beats));
      end if;
      L.Prev_Gray.Clear;
      for C of F.Cams loop
         L.Prev_Gray.Append (C.Gray);   --  占位的那台存空的 ⇒ 下一拍这台也不量(没有"上一拍")
      end loop;
   end Note_Beat;

   function Sense (L : in out Link; F : out Frame) return Boolean is
      use Ada.Calendar;
      T0 : constant Time := Clock;
      T1 : Time;
   begin
      if Lockstep.Current_Hand >= 0 then
         --  手的任务:把棒交还主线程,这一拍由主线程收(Lock_Beat),醒来拿那一帧
         Lockstep.Yield;
         F := Lock_F;
         return Lock_Ok;
      end if;
      F := (others => <>);
      if not Pump (L) then
         return False;
      end if;
      T1 := Clock;
      L.Seq := L.Seq + 1;
      Frame_Of (L, F);
      Note_Beat (L, F);   --  这一拍记下来(量画面比读数晚几拍用;见 Beat)
      --  录像:BL_VID 全分辨率灰度(开头密、后面疏,编号连续 ⇒ mkvid 能拼);BL_FILM 半分辨率抽帧。
      declare
         Vid : constant String := Codec.Env ("BL_VID");
         Film : constant String := Codec.Env ("BL_FILM");
      begin
         if Vid /= "" then
            --  每一帧的位姿读数都落盘(poses.txt:帧号、这一帧存下的画面编号或 -1、每条臂 xyz + wxyz),离线能核"画面和位姿是不是同一刻"
            --  (2026-09-24:人形腕眼焦距几炮都偏低 1–6%,x5 上在 1% 以内;要量的是画面是不是比位姿晚)
            declare
               Fo : File_Type;
               Pth : constant String := Vid & "/poses.txt";
               Saved : constant Boolean := L.Seq <= 2000 or else L.Seq mod 20 = 0;
            begin
               Codec.Make_Dir (Vid);
               begin
                  Open (Fo, Append_File, Pth);
               exception
                  when others => Create (Fo, Out_File, Pth);
               end;
               Put (Fo, Codec.Img (L.Seq) & " " & (if Saved then Codec.Img (L.Vid_N) else "-1"));
               for P of F.Reported_EE loop   --  身体自己报的(只给离线打分)
                  for I in P'Range loop
                     Put (Fo, " " & Codec.Fmt (P (I), 6));
                  end loop;
               end loop;
               New_Line (Fo);
               Close (Fo);
               --  同一帧的关节读数(joints.txt:帧号、画面编号或 -1、每组关节 "| v…";组的顺序同开机 [认] 关节角那一行)。
               --  2026-09-26 V1b:离线量"关节转多少、手到哪" —— 只记录,不改行为
               begin
                  Open (Fo, Append_File, Vid & "/joints.txt");
               exception
                  when others => Create (Fo, Out_File, Vid & "/joints.txt");
               end;
               Put (Fo, Codec.Img (L.Seq) & " " & (if Saved then Codec.Img (L.Vid_N) else "-1"));
               for Q of F.Joints loop
                  Put (Fo, " |");
                  for X of Q loop
                     Put (Fo, " " & Codec.Fmt (X, 6));
                  end loop;
               end loop;
               New_Line (Fo);
               Close (Fo);
            exception
               when others => null;
            end;
            --  2000 / 20 是帧计数(无量纲),只管落图密度
            if L.Seq <= 2000 or else L.Seq mod 20 = 0 then
               Codec.Make_Dir (Vid);
               for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
                  Codec.Write_PGM (Vid & "/f" & Codec.Pad6 (L.Vid_N) & "_c" & Codec.Img (Ci) & ".pgm",
                                   F.Cams (Ci).Gray, F.Cams (Ci).W, F.Cams (Ci).H);
               end loop;
               L.Vid_N := L.Vid_N + 1;
            end if;
         end if;
         if Film /= "" then
            declare
               Stride : constant Natural := Natural'Max (1, Codec.Env_Nat ("BL_FILM_STRIDE", 6));
               Max : constant Natural := Codec.Env_Nat ("BL_FILM_MAX", 6000);
            begin
               if L.Seq mod Stride = 0 and then L.Film_N < Max then
                  Codec.Make_Dir (Film);
                  for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
                     declare
                        C : Cam renames F.Cams (Ci);
                        Hw : constant Natural := C.W / 2;
                        Hh : constant Natural := C.H / 2;
                        G : Buf;
                     begin
                        for Y in 0 .. Hh - 1 loop
                           for X in 0 .. Hw - 1 loop
                              G.Append (C.Gray.Element ((Y * 2) * C.W + X * 2));
                           end loop;
                        end loop;
                        Codec.Write_PGM (Film & "/c" & Codec.Img (Ci) & "_" & Codec.Pad6 (L.Film_N) & ".pgm", G, Hw, Hh);
                     end;
                  end loop;
                  L.Film_N := L.Film_N + 1;
               end if;
            end;
         end if;
      end;
      L.Wait_Us := L.Wait_Us + Long_Float (T1 - T0) * 1.0e6;
      L.Parse_Us := L.Parse_Us + Long_Float (Clock - T1) * 1.0e6;
      if L.Seq mod 50 = 0 then
         --  50 帧(次数)× 1e6 微秒
         L.Frame_S := (L.Wait_Us + L.Parse_Us) / 50.0e6;
         Put_Line ("      [计时] 近 50 帧:等帧 " & Codec.Fmt (L.Wait_Us / 50000.0, 1) & " ms/帧 · 解图 " &
                   Codec.Fmt (L.Parse_Us / 50000.0, 1) & " ms/帧 · 一拍 " & Codec.Fmt (L.Frame_S, 3) &
                   " s · 相机" & Natural'Image (Natural (F.Cams.Length)) & " 台");
         L.Wait_Us := 0.0; L.Parse_Us := 0.0;
      end if;
      if Hook_P /= null then
         Hook_P (F);
         --  按关节读数算出来的手的位姿也落盘(fk_poses.txt,格式同 poses.txt):离线和身体报的比,驱动不读这个文件
         declare
            Vid : constant String := Codec.Env ("BL_VID");
            Fo : File_Type;
         begin
            if Vid /= "" then
               begin
                  Open (Fo, Append_File, Vid & "/fk_poses.txt");
               exception
                  when others => Create (Fo, Out_File, Vid & "/fk_poses.txt");
               end;
               Put (Fo, Codec.Img (L.Seq) & " -1");
               for P of F.EE loop
                  for I in P'Range loop
                     Put (Fo, " " & Codec.Fmt (P (I), 6));
                  end loop;
               end loop;
               New_Line (Fo);
               Close (Fo);
            end if;
         exception
            when others => null;
         end;
      end if;
      return True;
   end Sense;

   function Act_Raw (L : in out Link; C : Cmd) return Boolean;
   --  手的任务里的 Act:只记下这只手的目标(位姿命令先解成关节),这一拍由主线程合起来发
   function Lock_Act (C : Cmd) return Boolean is
      Cj : Cmd := C;
      Ok : Boolean := True;
      procedure Put_Q (G : Natural; Q : Floats) is
      begin
         while Natural (Lock_Q.Length) <= G loop
            Lock_Q.Append (F64_Vectors.Empty_Vector);
         end loop;
         Lock_Q.Replace_Element (G, Q);
      end Put_Q;
   begin
      if C.Kind = Hold then
         return True;
      end if;
      if C.Kind = Ee then
         if Hook_C = null then
            return False;   --  没有运动学:位姿命令解不成关节(按拍对齐只合关节动作)
         end if;
         Hook_C (Cj, Ok);
         if not Ok then
            return False;
         end if;
      end if;
      if Cj.Kind /= Joint then
         return False;
      end if;
      if not Cj.Groups.Is_Empty then
         for K in 0 .. Natural'Min (Natural (Cj.Groups.Length), Natural (Cj.Qs.Length)) - 1 loop
            if Cj.Groups (K) >= 0 then
               Put_Q (Natural (Cj.Groups (K)), Cj.Qs (K));
            end if;
         end loop;
      elsif Cj.Group >= 0 then
         Put_Q (Natural (Cj.Group), Cj.Q);
      else
         Put_Q (Cj.Arm, Cj.Q);   --  只报关节的身体:按臂
      end if;
      if not C.Jaw.Is_Empty then
         while Natural (Lock_Jaw.Length) <= C.Arm loop
            Lock_Jaw.Append (F64_Vectors.Empty_Vector);
         end loop;
         Lock_Jaw.Replace_Element (C.Arm, C.Jaw);
      end if;
      Lock_Changed := True;
      return True;
   end Lock_Act;
   function Act (L : in out Link; C : Cmd) return Boolean is
   begin
      if Lockstep.Current_Hand >= 0 then
         return Lock_Act (C);
      end if;
      if C.Kind = Ee and then Hook_C /= null then
         declare
            Cj : Cmd := C;
            Ok : Boolean;
         begin
            Hook_C (Cj, Ok);
            return Ok and then Act_Raw (L, Cj);
         end;
      end if;
      return Act_Raw (L, C);
   end Act;

   --  第 Ji 个抓握读数组这回发哪一串(见 spec):形状 = 这一拍的读数(没收到 ⇒ 上一回发出去的那一串);给了 ⇒ 给的,没给 ⇒ 给过的最后一个目标,
   --  一次没给过 ⇒ 此刻的读数,这一拍没读数 ⇒ 上一回发出去的那个数。一个都凑不出 ⇒ 空(这一组这回不发)
   function Jaw_Values (L : in out Link; Ji : Natural; Mine : Boolean; C : Cmd; Cur : Floats) return Floats is
      V : Floats;
   begin
      while Natural (L.Jaw_Set.Length) <= Ji loop
         L.Jaw_Set.Append (F64_Vectors.Empty_Vector);
      end loop;
      while Natural (L.Jaw_Sent.Length) <= Ji loop
         L.Jaw_Sent.Append (F64_Vectors.Empty_Vector);
      end loop;
      if Mine then
         --  这条命令给了这一组前几个通道的目标:整段记下(连着一段、没有空位 ⇒ 不用编数去填前面没给的)
         declare
            S : Floats := L.Jaw_Set (Ji);
         begin
            for K in 0 .. Natural (C.Jaw.Length) - 1 loop
               if K < Natural (S.Length) then
                  S.Replace_Element (K, C.Jaw (K));
               else
                  S.Append (C.Jaw (K));
               end if;
            end loop;
            L.Jaw_Set.Replace_Element (Ji, S);
         end;
      end if;
      declare
         Given : constant Floats := L.Jaw_Set (Ji);
         Last : constant Floats := L.Jaw_Sent (Ji);
         N : constant Natural := (if Cur.Is_Empty then Natural (Last.Length) else Natural (Cur.Length));
      begin
         for K in 0 .. N - 1 loop
            V.Append (if K < Natural (Given.Length) then Given (K) elsif K < Natural (Cur.Length) then Cur (K) else Last (K));
         end loop;
      end;
      if not V.Is_Empty then
         L.Jaw_Sent.Replace_Element (Ji, V);
      end if;
      return V;
   end Jaw_Values;

   function Act_Raw (L : in out Link; C : Cmd) return Boolean is
      S : Buf;
      N : constant Natural := Arms (L);
   begin
      if C.Kind = Hold then
         return True;
      end if;
      if L.Last_Obs < 0 or else N = 0 then
         return False;
      end if;
      if C.Kind = Base then
         if L.Lay.Base.Is_Empty then
            return False;
         end if;
         Put_Map (S, Natural (L.Lay.Base.Length));
         for I in 0 .. Natural (L.Lay.Base.Length) - 1 loop
            Put_Str (S, Layout.Last_Seg (L.Lay.Base (I)));
            if Natural (L.Lay.Base.Length) = 1 then
               Put_Array (S, Natural (C.V.Length));
               for X of C.V loop
                  Put_Float (S, X);
               end loop;
            else
               Put_Float (S, (if I < Natural (C.V.Length) then C.V (I) else 0.0));
            end if;
         end loop;
         L.Pending := S; L.Has_Pending := True;
         return True;
      end if;
      --  身体也报位姿,但开机量胳膊要一个关节一个关节地转(V1b,2026-09-26):发一条【只有关节】的动作。
      --  每个不同名字的关节组发一份(名字相同的读数组 / 命令回声组只发一次,取第一个):C.Group 那一组的名字发 C.Q,其余照此刻的读数保持;
      --  抓握通道同样按名字去重、照此刻的读数保持。一条动作里只有关节这一类,不混位姿(对方按键名认动作类型)
      if C.Kind = Joint and then not Joint_Mode (L) and then (C.Group >= 0 or else not C.Groups.Is_Empty) then
         if C.Group >= Natural (L.Lay.Joints.Length) then
            return False;
         end if;
         declare
            Target : constant String := (if C.Group >= 0 then Layout.Last_Seg (L.Lay.Joints (C.Group)) else "");
            --  这个名字的关节组在 C.Groups 里排第几(-1 = 不在)
            function In_Groups (Nm : String) return Integer is
            begin
               for K in 0 .. Natural (C.Groups.Length) - 1 loop
                  if C.Groups (K) >= 0 and then C.Groups (K) < Natural (L.Lay.Joints.Length)
                    and then Layout.Last_Seg (L.Lay.Joints (Natural (C.Groups (K)))) = Nm and then K < Natural (C.Qs.Length)
                  then
                     return Integer (K);
                  end if;
               end loop;
               return -1;
            end In_Groups;
            J_Names, W_Names : Strs;
            J_First, W_First : Ints;
            W_Vals : Floats_Vectors.Vector;   --  每个抓握键这回发的那一串(Jaw_Values;空 = 这回不发)
            N_Keys : Natural;
            function Has (V : Strs; X : String) return Boolean is
            begin
               for Y of V loop
                  if Y = X then
                     return True;
                  end if;
               end loop;
               return False;
            end Has;
         begin
            for I in 0 .. Natural (L.Lay.Joints.Length) - 1 loop
               if not Has (J_Names, Layout.Last_Seg (L.Lay.Joints (I))) then
                  J_Names.Append (Layout.Last_Seg (L.Lay.Joints (I))); J_First.Append (I);
               end if;
            end loop;
            for I in 0 .. Natural (L.Lay.Jaw.Length) - 1 loop
               if not Has (W_Names, Layout.Last_Seg (L.Lay.Jaw (I))) then
                  W_Names.Append (Layout.Last_Seg (L.Lay.Jaw (I))); W_First.Append (I);
               end if;
            end loop;
            N_Keys := Natural (J_Names.Length);
            for K in 0 .. Natural (W_Names.Length) - 1 loop
               declare
                  J : constant Floats := Nums_At (L, L.Lay.Jaw (W_First (K)));
                  --  这条臂自己的抓握通道给了目标就发目标(位姿命令解成关节目标时带着,V1b 3c),别的发这一集给过的最后一个目标(见 Jaw_Set)
                  Mine : constant Boolean := W_First (K) = C.Arm and then not C.Jaw.Is_Empty;
               begin
                  --  不截:读数范围是开机两头推到头量的(V1b ②;原来截在 [0, 1],x5 的约定)
                  W_Vals.Append (Jaw_Values (L, W_First (K), Mine, C, J));
                  if not W_Vals.Last_Element.Is_Empty then
                     N_Keys := N_Keys + 1;
                  end if;
               end;
            end loop;
            Put_Map (S, N_Keys);
            for K in 0 .. Natural (J_Names.Length) - 1 loop
               Put_Str (S, J_Names (K));
               declare
                  Kg : constant Integer := In_Groups (J_Names (K));
                  Q : constant Floats := (if Kg >= 0 then C.Qs (Natural (Kg)) elsif J_Names (K) = Target then C.Q else Nums_At (L, L.Lay.Joints (J_First (K))));
               begin
                  Put_Array (S, Natural (Q.Length));
                  for X of Q loop
                     Put_Float (S, X);
                  end loop;
               end;
            end loop;
            for K in 0 .. Natural (W_Names.Length) - 1 loop
               if not W_Vals (K).Is_Empty then
                  Put_Str (S, W_Names (K));
                  Put_Array (S, Natural (W_Vals (K).Length));
                  for X of W_Vals (K) loop
                     Put_Float (S, X);
                  end loop;
               end if;
            end loop;
         end;
         L.Pending := S; L.Has_Pending := True;
         return True;
      end if;
      if (C.Kind = Joint) /= Joint_Mode (L) then
         return False;    --  关节命令只在关节模式,位姿命令只在位姿模式:一条动作里不许混两种类型
      end if;
      declare
         --  每只手的抓握键先凑好这回发的那一串(Jaw_Values;空 = 这回不发:这一拍没读数、这一集也没发过),再数一条动作里有几个键
         Jv : Floats_Vectors.Vector;
         Ji_Of : Ints;
         N_Keys : Natural := N;
      begin
         for I in 0 .. N - 1 loop
            if L.Lay.Jaw.Is_Empty then
               Jv.Append (F64_Vectors.Empty_Vector); Ji_Of.Append (0);
            else
               declare
                  Ji : constant Natural := Natural'Min (I, Natural (L.Lay.Jaw.Length) - 1);
                  Mine : constant Boolean := (I = C.Arm or else Natural (L.Lay.Jaw.Length) = 1) and then not C.Jaw.Is_Empty;
               begin
                  --  没给命令的通道发这一集给过它的最后一个目标(一次只动脑点名的那一根手指;见 Jaw_Set);不截(同上)
                  Jv.Append (Jaw_Values (L, Ji, Mine, C, Nums_At (L, L.Lay.Jaw (Ji))));
                  Ji_Of.Append (Ji);
               end;
            end if;
            if not Jv.Last_Element.Is_Empty then
               N_Keys := N_Keys + 1;
            end if;
         end loop;
         Put_Map (S, N_Keys);
         for I in 0 .. N - 1 loop
            if Joint_Mode (L) then
               Put_Str (S, Layout.Last_Seg (L.Lay.Joints (I)));
               declare
                  --  发给第几组:给了 Group 就按它(开机前半段按读数组认手,V1b 3c),没给按臂
                  Tg : constant Natural := (if C.Group >= 0 then Natural (C.Group) else C.Arm);
                  Kg : Integer := -1;
               begin
                  for K in 0 .. Natural'Min (Natural (C.Groups.Length), Natural (C.Qs.Length)) - 1 loop
                     if C.Groups (K) = I then
                        Kg := Integer (K);
                     end if;
                  end loop;
                  declare
                     Q : constant Floats := (if Kg >= 0 then C.Qs (Natural (Kg)) elsif C.Groups.Is_Empty and then I = Tg then C.Q else Nums_At (L, L.Lay.Joints (I)));
                  begin
                     Put_Array (S, Natural (Q.Length));
                     for X of Q loop
                        Put_Float (S, X);
                     end loop;
                  end;
               end;
            else
               Put_Str (S, Layout.Last_Seg (L.Lay.EE (I)));
               if I = C.Arm then
                  Put_Array (S, 7);
                  for K in 0 .. 6 loop
                     Put_Float (S, C.Pose (K));
                  end loop;
               else
                  declare
                     P : constant Floats := Nums_At (L, L.Lay.EE (I));
                  begin
                     Put_Array (S, Natural (P.Length));
                     for X of P loop
                        Put_Float (S, X);
                     end loop;
                  end;
               end if;
            end if;
            if not Jv (I).Is_Empty then
               Put_Str (S, Layout.Last_Seg (L.Lay.Jaw (Natural (Ji_Of (I)))));
               Put_Array (S, Natural (Jv (I).Length));
               for X of Jv (I) loop
                  Put_Float (S, X);
               end loop;
            end if;
         end loop;
      end;
      L.Pending := S; L.Has_Pending := True;
      return True;
   end Act_Raw;

   procedure Lock_Begin is
   begin
      Lock_Q.Clear; Lock_Jaw.Clear; Lock_Changed := False;
   end Lock_Begin;

   procedure Lock_Feed (F : Frame; Ok : Boolean := True) is
   begin
      Lock_F := F;
      Lock_Ok := Ok;
      Lock_Changed := False;
   end Lock_Feed;

   procedure Lock_End is
   begin
      Lock_Q.Clear; Lock_Jaw.Clear; Lock_Changed := False;
   end Lock_End;

   function Lock_Merged return Cmd is
      Cm : Cmd;
   begin
      Cm.Kind := Joint;
      for G in 0 .. Natural (Lock_Q.Length) - 1 loop
         if not Lock_Q (G).Is_Empty then
            Cm.Groups.Append (G);
            Cm.Qs.Append (Lock_Q (G));
         end if;
      end loop;
      return Cm;
   end Lock_Merged;

   --  主线程走一拍:这一拍有手发了新目标 ⇒ 几只手的关节目标合成一条动作(每只给过爪子目标的手各发一遍,把它的爪子目标登记进 Jaw_Set;
   --  最后那一遍的动作里关节是全部手的、爪子是各自最后一个目标),再收一帧,留给手的任务醒来拿
   procedure Lock_Beat (L : in out Link; F : in out Frame; Ok : out Boolean) is
   begin
      if Lock_Changed then
         declare
            Cm : Cmd := Lock_Merged;
            Any_Jaw : Boolean := False;
            Sent : Boolean;
         begin
            if not Cm.Groups.Is_Empty then
               for A in 0 .. Natural (Lock_Jaw.Length) - 1 loop
                  if not Lock_Jaw (A).Is_Empty then
                     Cm.Arm := A; Cm.Jaw := Lock_Jaw (A);
                     Sent := Act_Raw (L, Cm);
                     Any_Jaw := True;
                  end if;
               end loop;
               if not Any_Jaw then
                  Cm.Arm := 0; Cm.Jaw.Clear;
                  Sent := Act_Raw (L, Cm);
               end if;
               if not Sent then
                  Put_Line ("[链] 几只手合成的那条关节动作没发成(这一拍照旧重发上一条)");
               end if;
            end if;
            Lock_Changed := False;
         end;
      end if;
      Ok := Sense (L, F);
      Lock_F := F;
      Lock_Ok := Ok;
   end Lock_Beat;
end Plug;
