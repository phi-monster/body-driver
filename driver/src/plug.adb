with Ada.Text_IO; use Ada.Text_IO;
with Ada.Calendar;
with Ada.Unchecked_Conversion;
with Interfaces; use Interfaces;
with Codec;
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

   function Arms (L : Link) return Natural is
   begin
      if not L.Lay.EE.Is_Empty then
         return Natural'Min (Natural (L.Lay.EE.Length), Natural'Max (1, Natural (L.Lay.Jaw.Length)));
      end if;
      return Natural (L.Lay.Joints.Length);
   end Arms;

   function Joint_Mode (L : Link) return Boolean is (L.Lay.EE.Is_Empty);

   function Steps (L : Link) return Natural is (if L.Seq >= L.Ep_Seq0 then L.Seq - L.Ep_Seq0 else L.Seq);

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

   --  「照现在这样保持」:把此刻报的位姿/关节/开合原样编成一条动作。零假设,一次回声。
   function Hold_Action (L : Link) return Buf is
      S : Buf;
      N : constant Natural := Arms (L);
   begin
      if N = 0 or else L.Last_Obs < 0 then
         return S;
      end if;
      Put_Map (S, 2 * N);
      for I in 0 .. N - 1 loop
         if Joint_Mode (L) then
            Put_Str (S, Layout.Last_Seg (L.Lay.Joints (I)));
            declare
               Q : constant Floats := Nums_At (L, L.Lay.Joints (I));
            begin
               Put_Array (S, Natural (Q.Length));
               for X of Q loop
                  Put_Float (S, X);
               end loop;
            end;
         else
            Put_Str (S, Layout.Last_Seg (L.Lay.EE (I)));
            declare
               P : constant Floats := Nums_At (L, L.Lay.EE (I));
            begin
               Put_Array (S, Natural (P.Length));
               for X of P loop
                  Put_Float (S, X);
               end loop;
            end;
         end if;
         declare
            Ji : constant Natural := Natural'Min (I, Natural (L.Lay.Jaw.Length) - 1);
            J : constant Floats := Nums_At (L, L.Lay.Jaw (Ji));
         begin
            Put_Str (S, Layout.Last_Seg (L.Lay.Jaw (Ji)));
            Put_Array (S, 1);
            Put_Float (S, (if J.Is_Empty then 1.0 else J (0)));
         end;
      end loop;
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

   function Sense (L : in out Link; F : out Frame) return Boolean is
      use Ada.Calendar;
      T0 : constant Time := Clock;
      T1 : Time;
   begin
      F := (others => <>);
      if not Pump (L) then
         return False;
      end if;
      T1 := Clock;
      L.Seq := L.Seq + 1;
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
            C : Cam;
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
                  F.Cams.Append (C);
               end if;
            end if;
         end;
      end loop;
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
   function Act (L : in out Link; C : Cmd) return Boolean is
   begin
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
      if C.Kind = Joint and then not Joint_Mode (L) and then C.Group >= 0 then
         if C.Group >= Natural (L.Lay.Joints.Length) then
            return False;
         end if;
         declare
            Target : constant String := Layout.Last_Seg (L.Lay.Joints (C.Group));
            J_Names, W_Names : Strs;
            J_First, W_First : Ints;
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
            Put_Map (S, Natural (J_Names.Length) + Natural (W_Names.Length));
            for K in 0 .. Natural (J_Names.Length) - 1 loop
               Put_Str (S, J_Names (K));
               declare
                  Q : constant Floats := (if J_Names (K) = Target then C.Q else Nums_At (L, L.Lay.Joints (J_First (K))));
               begin
                  Put_Array (S, Natural (Q.Length));
                  for X of Q loop
                     Put_Float (S, X);
                  end loop;
               end;
            end loop;
            for K in 0 .. Natural (W_Names.Length) - 1 loop
               Put_Str (S, W_Names (K));
               declare
                  J : constant Floats := Nums_At (L, L.Lay.Jaw (W_First (K)));
                  --  这条臂自己的抓握通道给了目标就发目标(位姿命令解成关节目标时带着,V1b 3c),别的保持此刻的读数
                  Mine : constant Boolean := W_First (K) = C.Arm and then not C.Jaw.Is_Empty;
               begin
                  Put_Array (S, Natural'Max (1, Natural (J.Length)));
                  if J.Is_Empty then
                     Put_Float (S, (if Mine then Long_Float'Max (0.0, Long_Float'Min (1.0, C.Jaw (0))) else 1.0));
                  else
                     for X in 0 .. Natural (J.Length) - 1 loop
                        Put_Float (S, Long_Float'Max (0.0, Long_Float'Min (1.0, (if Mine and then X < Natural (C.Jaw.Length) then C.Jaw (X) else J (X)))));
                     end loop;
                  end if;
               end;
            end loop;
         end;
         L.Pending := S; L.Has_Pending := True;
         return True;
      end if;
      if (C.Kind = Joint) /= Joint_Mode (L) then
         return False;    --  关节命令只在关节模式,位姿命令只在位姿模式:一条动作里不许混两种类型
      end if;
      Put_Map (S, 2 * N);
      for I in 0 .. N - 1 loop
         if Joint_Mode (L) then
            Put_Str (S, Layout.Last_Seg (L.Lay.Joints (I)));
            declare
               --  发给第几组:给了 Group 就按它(开机前半段按读数组认手,V1b 3c),没给按臂
               Tg : constant Natural := (if C.Group >= 0 then Natural (C.Group) else C.Arm);
               Q : constant Floats := (if I = Tg then C.Q else Nums_At (L, L.Lay.Joints (I)));
            begin
               Put_Array (S, Natural (Q.Length));
               for X of Q loop
                  Put_Float (S, X);
               end loop;
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
         declare
            Ji : constant Natural := Natural'Min (I, Natural (L.Lay.Jaw.Length) - 1);
            Cur : constant Floats := Nums_At (L, L.Lay.Jaw (Ji));
            Nj : constant Natural := Natural'Max (1, Natural (Cur.Length));
            Mine : constant Boolean := (I = C.Arm or else Natural (L.Lay.Jaw.Length) = 1) and then not C.Jaw.Is_Empty;
         begin
            Put_Str (S, Layout.Last_Seg (L.Lay.Jaw (Ji)));
            Put_Array (S, Nj);
            for K in 0 .. Nj - 1 loop
               declare
                  --  没给命令的通道保持它此刻的读数 —— 一次只动脑点名的那一根手指
                  V : constant Long_Float :=
                    (if Mine and then K < Natural (C.Jaw.Length) then C.Jaw (K)
                     elsif K < Natural (Cur.Length) then Cur (K) else 1.0);
               begin
                  Put_Float (S, Long_Float'Max (0.0, Long_Float'Min (1.0, V)));
               end;
            end loop;
         end;
      end loop;
      L.Pending := S; L.Has_Pending := True;
      return True;
   end Act_Raw;
end Plug;
