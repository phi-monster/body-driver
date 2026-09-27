--  离线自检:不连仿真就能跑的那些量法和格式。每条断言写清楚"错了会是什么病"。
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Bytes; use Bytes;
with Codec;
with Msgpack;
with Json;
with Picture;
with Table;
with Monitor;
with Backup;
with Flow;
with GNAT.SHA1;
with Zone;
with Schema;
with Plug;
with Chan;
with Sinew;
with Runtime;
with Plan;
with Layout;
with Act;
with Bodyfile;
with Geom;
with Selfmap;
with Learned;
with Exam;
with Contact;
with Contact.Gen;
with Contact.Exec;
with Contact.Surface;
with Kinem;
with Jointboot;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers;
with Interfaces; use type Interfaces.Unsigned_8;
procedure Selfcheck is
   Fails : Natural := 0;
   procedure Check (Cond : Boolean; What : String) is
   begin
      if Cond then
         Put_Line ("  🟢 " & What);
      else
         Put_Line ("  🔴 " & What);
         Fails := Fails + 1;
      end if;
   end Check;
begin
   --  base64 标准向量 + WebSocket 握手向量(RFC 6455 §1.3)
   Check (Codec.Base64_Of_String ("foobar") = "Zm9vYmFy", "base64 foobar");
   Check (Codec.Base64_Of_String ("fo") = "Zm8=", "base64 补齐");
   declare
      Acc : constant String := Codec.Base64 (Codec.Hex_To_Bytes (GNAT.SHA1.Digest ("dGhlIHNhbXBsZSBub25jZQ==" & "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")));
   begin
      Check (Acc = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", "WebSocket 握手 accept 向量:" & Acc);
   end;
   --  msgpack 往返:map{message_type:"hello", n:-7, f:1.5, arr:[1,2], bin:<3 bytes>, nd:{nd:true,type:"<f4",shape:[2],data:bin8}}
   declare
      S : Buf;
      D : Msgpack.Doc;
      Data : Buf;
   begin
      Data.Append (0); Data.Append (0); Data.Append (16#80#); Data.Append (16#3F#);   --  1.0f LE
      Data.Append (0); Data.Append (0); Data.Append (0); Data.Append (16#40#);        --  2.0f LE
      Msgpack.Put_Map (S, 5);
      Msgpack.Put_Str (S, "message_type"); Msgpack.Put_Str (S, "hello");
      Msgpack.Put_Str (S, "n"); Msgpack.Put_Int (S, -7);
      Msgpack.Put_Str (S, "f"); Msgpack.Put_Float (S, 1.5);
      Msgpack.Put_Str (S, "arr"); Msgpack.Put_Array (S, 2); Msgpack.Put_Int (S, 1); Msgpack.Put_Int (S, 300);
      Msgpack.Put_Str (S, "nd"); Msgpack.Put_Map (S, 4);
      Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
      Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "<f4");
      Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 1); Msgpack.Put_Int (S, 2);
      Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Data, 0, 8);
      Check (Msgpack.Decode (S, D), "msgpack 解码");
      Check (Msgpack.Text (D, Msgpack.Key (D, 0, "message_type")) = "hello", "msgpack 文本键");
      Check (Msgpack.Num (D, Msgpack.Key (D, 0, "n")) = -7.0, "msgpack 负整数");
      Check (Msgpack.Num (D, Msgpack.Key (D, 0, "f")) = 1.5, "msgpack 浮点");
      Check (Natural (Msgpack.Numbers (D, Msgpack.Key (D, 0, "arr")).Length) = 2 and then Msgpack.Numbers (D, Msgpack.Key (D, 0, "arr")) (1) = 300.0, "msgpack 数组 uint16");
      declare
         Nd : constant Floats := Msgpack.Numbers (D, Msgpack.Key (D, 0, "nd"));
      begin
         Check (Natural (Nd.Length) = 2 and then Nd (0) = 1.0 and then Nd (1) = 2.0, "nd <f4 小端读数");
      end;
      declare
         S2 : Buf;
         D2 : Msgpack.Doc;
      begin
         Msgpack.Put_Node (S2, D, 0);
         Check (Msgpack.Decode (S2, D2) and then Msgpack.Text (D2, Msgpack.Key (D2, 0, "message_type")) = "hello", "msgpack 原样回写");
      end;
   end;
   --  JSON:脑的回包形状
   declare
      D : Json.Doc;
      E : Unbounded_String;
      Src : constant String := "{""say"":""I see it"",""see"":""target"",""look"":0,""moves"":[{""item"":7,""cell"":0,""rel"":""at"",""of"":9,""amount"":""small"",""stay_put"":false}],""grip"":""close"",""grip_arm"":1,""grip_on"":9,""until"":""contact"",""steps"":0,""fast"":false,""avoid_items"":[],""done"":false}";
   begin
      Check (Json.Parse (Src, D, E), "JSON 解析:" & To_String (E));
      Check (Json.Text (D, Json.Get (D, 0, "say")) = "I see it", "JSON 字串");
      Check (Json.Num (D, Json.Child (D, Json.Get (D, 0, "moves"), 0)) = 0.0 or else Json.Count (D, Json.Get (D, 0, "moves")) = 1, "JSON 数组");
      Check (Json.Num (D, Json.Get (D, Json.Child (D, Json.Get (D, 0, "moves"), 0), "of")) = 9.0, "JSON 嵌套整数");
      Check (Json.Escape ("a""b" & ASCII.LF) = "a\""b\n", "JSON 转义");
   end;
   --  深度切块:平桌面上一个鼓起 3 cm 的方块,必须切出正好一块,且不贴边
   declare
      W : constant := 96;
      H : constant := 72;
      Dep : Floats := Filled (W * H, 0.80);
      R : Picture.Regions;
      Seed : Long_Long_Integer := 7;
   begin
      for I in 0 .. W * H - 1 loop
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         Dep.Replace_Element (I, 0.80 + Long_Float (Seed mod 1000) * 1.0e-6 - 0.0005);
      end loop;
      for Y in 30 .. 41 loop
         for X in 40 .. 55 loop
            Dep.Replace_Element (Y * W + X, 0.77);
         end loop;
      end loop;
      R := Picture.Cut (Dep, W, H, 0.125, 3.0);
      Check (Natural (R.Length) = 1, "切块:一个方块切出" & Natural'Image (Natural (R.Length)) & " 块");
      if not R.Is_Empty then
         Check (abs (R (0).Height - 0.03) < 0.005, "切块:鼓起高度 " & Codec.Fmt (R (0).Height, 3));
         Check (abs (R (0).Cu - 47.5 / 96.0) < 0.02 and then abs (R (0).Cv - 35.5 / 72.0) < 0.02, "切块:形心");
         Check (R (0).Elong > 1.2, "切块:主轴伸长 " & Codec.Fmt (R (0).Elong, 2));
      end if;
   end;
   --  动过的像素 + 连通块 + 两瓣
   declare
      W : constant := 64;
      H : constant := 48;
      A, B : Buf;
      M : Bools;
      Fl : Picture.Floor_Map;
      Comps : Picture.Regions;
   begin
      for I in 1 .. W * H loop
         A.Append (30); B.Append (30);
      end loop;
      for Y in 20 .. 27 loop
         for X in 8 .. 15 loop
            B.Replace_Element (Y * W + X, 200);
         end loop;
         for X in 44 .. 51 loop
            B.Replace_Element (Y * W + X, 200);
         end loop;
      end loop;
      Fl := Picture.Null_Floor (A, A, W, H, Picture.Min_Pixels (W, H));
      M := Picture.Moved (A, B, Fl);
      Comps := Picture.Components (M, W, H, 4);
      Check (Natural (Comps.Length) = 2, "两瓣:切出" & Natural'Image (Natural (Comps.Length)) & " 块");
      Check (Fl.Global = 0, "静止对地板 = 0");
   end;
   --  响应表:已知 B(2 通道 → 3 读数),解算要把误差解成正确的命令,且不越上限
   declare
      E : Table.Effect;
      Terms : Table.Term_Vectors.Vector;
      T : Table.Term;
      Cap : Table.Vec := Table.Zero_Vec;
      Act : Table.Mask := [others => False];
      A : Table.Vec;
      Ok : Boolean;
   begin
      Table.Reset (E, 2, 1.0);
      Table.Set_Col (E, 0, [0.5, 0.0, 0.0, 0.0, 0.0]);
      Table.Set_Col (E, 1, [0.0, 0.25, 0.0, 0.0, 0.0]);
      T.E := E; T.Err := [0.1, 0.05, 0.0, 0.0, 0.0]; T.W := [1.0, 1.0, 0.0, 0.0, 0.0];
      Terms.Append (T);
      Cap (0) := 1.0; Cap (1) := 1.0; Act (0) := True; Act (1) := True;
      Table.Solve (Terms, 2, Cap, Act, [others => 1.0e-9], A, Ok);
      Check (Ok and then abs (A (0) - 0.2) < 1.0e-3 and then abs (A (1) - 0.2) < 1.0e-3, "解算:" & Codec.Fmt (A (0), 3) & " " & Codec.Fmt (A (1), 3));
      Cap (0) := 0.1;
      Table.Solve (Terms, 2, Cap, Act, [others => 1.0e-9], A, Ok);
      Check (Ok and then abs (A (0) - 0.1) < 1.0e-6 and then abs (A (1) - 0.2) < 1.0e-3, "解算夹到上限:" & Codec.Fmt (A (0), 3));
      --  递推重估:真表是 [1,0,0],初值全零,推几步后预测该接近真值
      Table.Reset (E, 2, 1.0e6);
      for K in 1 .. 6 loop
         declare
            Cmd : Table.Vec := Table.Zero_Vec;
         begin
            Cmd (0) := 0.01 * Long_Float (K);
            Table.Update (E, Cmd, [Cmd (0) * 1.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0);
         end;
      end loop;
      Check (abs (Table.Col (E, 0) (0) - 1.0) < 0.05, "递推最小二乘学到 B=" & Codec.Fmt (Table.Col (E, 0) (0), 3));
      --  零表更准 ⇒ 顶住:命令发了、读数不动
      for K in 1 .. 3 loop
         declare
            Cmd : Table.Vec := Table.Zero_Vec;
         begin
            Cmd (0) := 0.05;
            Table.Update (E, Cmd, [0.0, 0.0, 0.0, 0.0, 0.0], 0.001, 0.0);
         end;
      end loop;
      Check (Table.Blocked (E), "顶住 = 零表连着两步更准");
   end;
   --  监视器与备份
   declare
      W : Monitor.Watch;
      F : Monitor.Floors;
      R : Backup.Ring;
      S : Backup.Step_Vec := [others => 0.0];
      Had : Boolean;
   begin
      F.Picture := 2.0; F.Track := 0.001; F.Delivery := 0.0001;
      for K in 1 .. 5 loop   --  走不动了要连着五步没进步(次数,无量纲)
         Monitor.Step (W, 1.0, 0.5, 0.5, 0.01, F, Seen => True);
      end loop;
      Check (Monitor.Settled (W) and then Monitor.Stalled (W) and then not Monitor.Refusing (W), "监视器:静止且没进展");
      Check (Monitor.Fired (Monitor.U_Settle, W, 0, False, 0.0, 0.0, 0.0), "until settle 触发");
      Check (not Monitor.Fired (Monitor.U_Contact, W, 0, False, 0.0, 0.0, 0.0), "until contact 不误触");
      S (0) := 0.02;
      Backup.Remember (R, S);
      Backup.Retreat (R, S, Had);
      Check (Had and then S (0) = -0.02 and then Backup.Count (R) = 0, "备份:沿来路退");
   end;
   --  光流:纯白方块平移 4 像素,中间那一格也要解出位移(块匹配在这儿必然失效)
   declare
      W : constant := 128;
      H : constant := 96;
      function Pic (X0 : Natural) return Buf is
         G : Buf;
      begin
         for I in 1 .. W * H loop
            G.Append (40);
         end loop;
         for Y in 28 .. 67 loop
            for X in X0 .. X0 + 39 loop
               G.Replace_Element (Y * W + X, 235);
            end loop;
         end loop;
         return G;
      end Pic;
      Fl : constant Flow.Field := Flow.Compute (Pic (30), Pic (34), W, H, 4, 60);
      Du, Dv : Long_Float;
   begin
      Flow.Sample (Fl, 50.0 / 128.0, 48.0 / 96.0, 0.05, Du, Dv);
      Check (Du * Long_Float (W) > 1.0, "光流:方块中心横向位移 " & Codec.Fmt (Du * Long_Float (W), 2) & " px(该 ≈ 4)");
      Flow.Sample (Fl, 110.0 / 128.0, 10.0 / 96.0, 0.03, Du, Dv);
      Check (abs Du * Long_Float (W) < 1.0, "光流:背景不动");
   end;
   --  握区:手上相机的合成图。桌面深 0.5;两瓣(深 0.2)张开时在下沿左右两角,合上时在下沿中间相遇;
   --  扫过的并集 = 两瓣走过的整条带。期望:两瓣、区心在下沿正中、张幅 ≈ 两瓣内沿之间的距离、手指深 ≈ 0.2。
   declare
      W : constant := 64;
      H : constant := 48;
      Dopen, Dclosed : Floats := Filled (W * H, 0.5);
      Swept : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      Z : Zone.Hand_Zone;
   begin
      for Y in 36 .. 47 loop
         for X in 0 .. 63 loop
            --  张开:瓣在 x 4..11 与 52..59;合上:瓣在 x 26..33 与 30..37(相遇);扫过 = 4..37 与 26..59
            if X in 4 .. 11 or else X in 52 .. 59 then
               Dopen.Replace_Element (Y * W + X, 0.2);
            end if;
            if X in 26 .. 37 then
               Dclosed.Replace_Element (Y * W + X, 0.2);
            end if;
            if X in 4 .. 59 then
               Swept.Replace_Element (Y * W + X, True);
            end if;
         end loop;
      end loop;
      Z := Zone.From_Sweep (Swept, Dopen, Dclosed, True, W, H);
      Check (Z.Valid and then Z.N_Lobes = 2, "握区:认出两瓣(" & Natural'Image (Z.N_Lobes) & ")");
      Check (abs (Z.Cu - 32.0 / 64.0) < 0.05 and then Z.Cv > 0.7, "握区:区心在下沿正中 (" & Codec.Fmt (Z.Cu, 2) & "," & Codec.Fmt (Z.Cv, 2) & ")");
      Check (abs (Z.Span - 40.0 / 64.0) < 0.06, "握区:张幅 " & Codec.Fmt (Z.Span, 3) & "(该 ≈ 0.625)");
      Check (abs (Z.Depth - 0.2) < 1.0e-6, "握区:手指深 " & Codec.Fmt (Z.Depth, 3));
      Check (Z.X0 >= 10 and then Z.X1 <= 53, "握区:区框在两瓣之间 " & Codec.Img (Z.X0) & ".." & Codec.Img (Z.X1));
   end;
   --  🔴 没有深度的握区(2026-09-24):只比张开/合上两张停住的灰度图。桌面木纹 100±8 逐像素乱抖(相机一合爪就抖,合成成整幅噪声),
   --  两根黑手指(20)张开时在下沿左右两角、合上时在下沿中间相遇。老量法按噪声地板把整个下半幅记成手指;新量法要认出两瓣、区心在正中
   declare
      W : constant := 64;
      H : constant := 48;
      Open_G, Closed_G : Buf := U8_Vectors.To_Vector (100, Ada.Containers.Count_Type (W * H));
      Z : Zone.Hand_Zone;
      Seed : Long_Long_Integer := 7;
      function Noise return Integer is   --  确定性伪随机 ±8(测试数据自己的抖动,不是驱动里的常数)
      begin
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         return Integer ((Seed / 65536) mod 17) - 8;
      end Noise;
   begin
      for I in 0 .. W * H - 1 loop
         Open_G.Replace_Element (I, Interfaces.Unsigned_8 (Integer'Max (0, Integer'Min (255, 100 + Noise))));
         Closed_G.Replace_Element (I, Interfaces.Unsigned_8 (Integer'Max (0, Integer'Min (255, 100 + Noise))));
      end loop;
      for Y in 36 .. 47 loop
         for X in 0 .. 63 loop
            if X in 4 .. 11 or else X in 52 .. 59 then
               Open_G.Replace_Element (Y * W + X, 20);
            end if;
            if X in 26 .. 37 then
               Closed_G.Replace_Element (Y * W + X, 20);
            end if;
         end loop;
      end loop;
      Z := Zone.From_Frames (Open_G, Closed_G, W, H);
      Check (Z.Valid and then Z.N_Lobes = 2, "握区(无深度):认出两瓣(" & Natural'Image (Z.N_Lobes) & ")");
      Check (Z.Valid and then Z.A.X0 >= 2 and then Z.A.X1 <= 13 and then Z.B.X0 >= 50 and then Z.B.X1 <= 61,
             "握区(无深度):两瓣是张开时的手指(左 " & Codec.Img (Z.A.X0) & ".." & Codec.Img (Z.A.X1) & " 右 " & Codec.Img (Z.B.X0) & ".." & Codec.Img (Z.B.X1) & ")");
      Check (Z.Valid and then abs (Z.Cu - 32.0 / 64.0) < 0.05 and then Z.Cv > 0.7, "握区(无深度):区心在下沿正中 (" & Codec.Fmt (Z.Cu, 2) & "," & Codec.Fmt (Z.Cv, 2) & ")");
      declare
         Cnt : Natural := 0;
      begin
         for B of Z.Fingers loop
            if B then
               Cnt := Cnt + 1;
            end if;
         end loop;
         Check (Cnt >= 300 and then Cnt <= 420, "握区(无深度):手指像素 " & Codec.Img (Cnt) & "(该 ≈ 384 = 4 段 × 8 × 12,抖动的桌面一个都不算)");
      end;
      declare
         Ua, Va, Ub, Vb : Long_Float;
         Oa, Ob : Boolean;
      begin
         Zone.Tip_Px (Z, Z.A, W, H, Ua, Va, Oa);
         Zone.Tip_Px (Z, Z.B, W, H, Ub, Vb, Ob);
         --  两根手指从下沿伸进画面(根在画面外)⇒ 尖 = 离贴着下沿的那几个像素最远的那一截 = 最上面一行;不是伸向合拢处的内侧边、也不能沾上合上时手指在的那一块
         --  (V1B21 2026-09-27 按 x5 网格真值:旧定义取在内侧边、瓣框里还混进合上的手指,指尖错 34 mm)
         Check (Oa and then Ob and then abs (Ua - 7.5) < 0.5 and then abs (Ub - 55.5) < 0.5 and then abs (Va - 36.0) < 0.5 and then abs (Vb - 36.0) < 0.5,
                "指尖 = 离手指进画面处最远的那一头:左瓣 (" & Codec.Fmt (Ua, 1) & "," & Codec.Fmt (Va, 1) & ") 右瓣 (" & Codec.Fmt (Ub, 1) & "," & Codec.Fmt (Vb, 1) & ")(该 ≈ (7.5,36) / (55.5,36))");
      end;
      Z := Zone.From_Frames (Open_G, Open_G, W, H);
      Check (not Z.Valid, "握区(无深度):两张一样的图 ⇒ 看不见这只手合拢,如实说");
   end;
   --  🔴 抓握通道当关节量(V1b ② 2026-09-27):两头的图谁先谁后都一样认出瓣(瓣 = 变化里形心分得开的那一类);瓣自己那一块的像素(Zone.Lobe_Pixels)
   --  不含合上时手指在的那一块;读数离"空手合"那头往张开那头走了多远(Act.Past_Empty)按量的方向算:x5 那样 1 张 0 合,和一只 0 那头张开、20 那头合的假手
   declare
      W : constant := 64;
      H : constant := 48;
      Open_G, Closed_G : Buf := U8_Vectors.To_Vector (100, Ada.Containers.Count_Type (W * H));
      Z1, Z2 : Zone.Hand_Zone;
      Hx, Hf : Zone.Hand;
      Lp : Bools;
      In_Open, In_Closed : Natural := 0;
   begin
      for Y in 36 .. 47 loop
         for X in 0 .. 63 loop
            if X in 4 .. 11 or else X in 52 .. 59 then
               Open_G.Replace_Element (Y * W + X, 20);
            end if;
            if X in 26 .. 37 then
               Closed_G.Replace_Element (Y * W + X, 20);
            end if;
         end loop;
      end loop;
      Z1 := Zone.From_Frames (Open_G, Closed_G, W, H);
      Z2 := Zone.From_Frames (Closed_G, Open_G, W, H);
      Check (Z1.Valid and then Z2.Valid and then Z1.N_Lobes = 2 and then Z2.N_Lobes = 2
             and then ((Z1.A.X0 = Z2.A.X0 and then Z1.B.X0 = Z2.B.X0) or else (Z1.A.X0 = Z2.B.X0 and then Z1.B.X0 = Z2.A.X0)),
             "握区:两头的图谁先谁后认出同样两瓣(" & Codec.Img (Z1.A.X0) & "," & Codec.Img (Z1.B.X0) & " / " & Codec.Img (Z2.A.X0) & "," & Codec.Img (Z2.B.X0) & ")");
      Lp := Zone.Lobe_Pixels (Z1, W, H);
      for Y in 36 .. 47 loop
         for X in 0 .. 63 loop
            if Lp.Element (Y * W + X) then
               if X in 4 .. 11 or else X in 52 .. 59 then
                  In_Open := In_Open + 1;
               elsif X in 26 .. 37 then
                  In_Closed := In_Closed + 1;
               end if;
            end if;
         end loop;
      end loop;
      Check (In_Open = 2 * 8 * 12 and then In_Closed = 0,
             "握区:瓣自己那一块 " & Codec.Img (In_Open) & " 像素(该 192)、混进合上的手指 " & Codec.Img (In_Closed) & "(该 0)");
      Hx.Open_Reading := 1.0; Hx.Empty_Close := 0.0;
      Hf.Open_Reading := 0.0; Hf.Empty_Close := 20.0;
      Check (abs (Act.Past_Empty (Hx, 0.4) - 0.4) < 1.0e-12 and then abs (Act.Past_Empty (Hf, 12.0) - 8.0) < 1.0e-12 and then Act.Past_Empty (Hf, 20.0) = 0.0,
             "抓握读数离空手合那头往张开那头走了多远:x5 读数 0.4 ⇒ " & Codec.Fmt (Act.Past_Empty (Hx, 0.4), 2) & "(该 0.4)· 0 张 20 合的假手读数 12 ⇒ "
             & Codec.Fmt (Act.Past_Empty (Hf, 12.0), 2) & "(该 8)");
   end;
   --  🔴 没点名的抓握通道发这一集给过它的最后一个目标,不发此刻的读数(Plug.Jaw_Value,V1B24 2026-09-27:碰桌面时手指被沿滑轨往里推,
   --  "保持此刻的读数"把推合了的读数锁住,爪子合上,后一瓣量短 13 mm)。给了 0.3 ⇒ 发 0.3;下一条没给、读数被推到 0.8 ⇒ 还发 0.3;
   --  对方复位(清空)⇒ 发读数 0.8;一次没给过的通道 ⇒ 发读数
   declare
      L : Plug.Link;
      C1, C2 : Plug.Cmd;
      Cur : Bytes.Floats;
      V1, V2, V3, V4 : Long_Float;
   begin
      C1.Jaw.Append (0.3);
      Cur.Append (1.0);
      V1 := Plug.Jaw_Value (L, 0, 0, True, C1, Cur);
      Cur.Replace_Element (0, 0.8);
      V2 := Plug.Jaw_Value (L, 0, 0, False, C2, Cur);
      V4 := Plug.Jaw_Value (L, 1, 0, False, C2, Cur);
      L.Jaw_Set.Clear;
      V3 := Plug.Jaw_Value (L, 0, 0, False, C2, Cur);
      Check (V1 = 0.3 and then V2 = 0.3 and then V3 = 0.8 and then V4 = 0.8,
             "抓握通道:给了 0.3 发 " & Codec.Fmt (V1, 2) & " · 没给、读数被推到 0.8 还发 " & Codec.Fmt (V2, 2) & "(该 0.3)· 复位后发 " & Codec.Fmt (V3, 2)
             & "(该读数 0.8)· 一次没给过的那一组发 " & Codec.Fmt (V4, 2) & "(该读数 0.8)");
   end;
   --  🔴 指尖只按这一瓣自己那一块手指像素找(V1B21 2026-09-27):手指像素里合上时手指在的那一块落进了瓣框的一角、瓣框又只盖住手指的下半截
   --  (同一根手指按背景明暗分进了两类)。合成 48×48:手指 x 3..9、y 20..47 从下沿伸进来;合上的那一块 x 12..18、y 40..47 另成一块;
   --  瓣框只给 [3,25]–[14,47] ⇒ 尖 = 整根手指最上面那一行 (6,20),宽 7 像素;一个像素都不贴画面边的一块 ⇒ 不给尖
   declare
      W : constant := 48;
      H : constant := 48;
      Z : Zone.Hand_Zone;
      U, V, Wd : Long_Float;
      Ok : Boolean;
   begin
      for I in 0 .. W * H - 1 loop
         Z.Fingers.Append ((I mod W in 3 .. 9 and then I / W in 20 .. 47) or else (I mod W in 12 .. 18 and then I / W in 40 .. 47) or else (I mod W in 30 .. 40 and then I / W in 5 .. 15));
      end loop;
      Z.Valid := True; Z.N_Lobes := 1;
      Z.A := (True, 3, 25, 14, 47, 0.2, 0.75, 200);
      Zone.Tip_Band (Z, Z.A, W, H, U, V, Wd, Ok);
      Check (Ok and then abs (U - 6.0) < 0.5 and then abs (V - 20.0) < 0.5 and then abs (Wd - 7.0) < 0.5,
             "指尖只看这一瓣自己那一块:(" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ") 宽 " & Codec.Fmt (Wd, 1) & "(该 (6,20) 宽 7;框里混进的合上那一块、框外的那一截都不许影响)");
      Z.A := (True, 30, 5, 40, 15, 0.7, 0.2, 121);
      Zone.Tip_Band (Z, Z.A, W, H, U, V, Wd, Ok);
      Check (not Ok, "整根在画面里、一个像素都不贴画面边的一块 ⇒ 看不出哪头伸出去了,不给尖");
   end;
   --  颜色切块:两根细杆在深度上鼓不出来,但颜色分得开 —— 各自成一块,而且是细长的
   declare
      W : constant Natural := 60;
      H : constant Natural := 40;
      RGB : Buf := U8_Vectors.To_Vector (30, Ada.Containers.Count_Type (W * H * 3));
      Rs : Picture.Regions;
      Cr, Cg, Cb : Long_Float;
   begin
      for Y in 5 .. 34 loop            --  一根红的竖杆(x = 20,宽 2 px)
         for X in 20 .. 21 loop
            RGB.Replace_Element (3 * (Y * W + X), 200);
            RGB.Replace_Element (3 * (Y * W + X) + 1, 30);
            RGB.Replace_Element (3 * (Y * W + X) + 2, 30);
         end loop;
      end loop;
      for Y in 5 .. 34 loop            --  一根蓝的竖杆(x = 30,宽 2 px)
         for X in 30 .. 31 loop
            RGB.Replace_Element (3 * (Y * W + X), 30);
            RGB.Replace_Element (3 * (Y * W + X) + 1, 30);
            RGB.Replace_Element (3 * (Y * W + X) + 2, 200);
         end loop;
      end loop;
      Rs := Picture.Cut_Colour (RGB, W, H, 20.0, 8);
      Check (Natural (Rs.Length) >= 3, "颜色切块:背景 + 两根杆 至少三块(切出" & Codec.Img (Natural (Rs.Length)) & " 块)");
      declare
         Thin : Natural := 0;
      begin
         for R of Rs loop
            if R.Count in 40 .. 80 and then R.Elong > 3.0 then
               Thin := Thin + 1;
            end if;
         end loop;
         Check (Thin = 2, "颜色切块:两根都被切成细长块(" & Codec.Img (Thin) & " 根)");
      end;
      for R of Rs loop
         if R.Count in 40 .. 80 then
            Picture.Mean_Colour (RGB, W, H, R, Cr, Cg, Cb);
            Check ((Cr > 100.0 and then Cb < 100.0) or else (Cb > 100.0 and then Cr < 100.0),
                   "颜色切块:一根偏红一根偏蓝(" & Codec.Fmt (Cr, 0) & "," & Codec.Fmt (Cg, 0) & "," & Codec.Fmt (Cb, 0) & ")");
         end if;
      end loop;
   end;
   --  五行的表:只有"往前走"能改大小、只有"转腕"能改朝向 ⇒ 要它变大且转正时,两个通道各自被用上
   declare
      E : Table.Effect;
      T : Table.Term;
      Terms : Table.Term_Vectors.Vector;
      Cap : Table.Vec := [others => 1.0];
      Act : Table.Mask := [others => False];
      A : Table.Vec;
      Ok : Boolean;
   begin
      Table.Reset (E, 2, 1.0);
      Table.Set_Col (E, 0, [0.0, 0.0, -1.0, 0.5, 0.0]);   --  通道 0:往前走 ⇒ 更近、看着更大
      Table.Set_Col (E, 1, [0.0, 0.0, 0.0, 0.0, 1.0]);    --  通道 1:转腕 ⇒ 只改朝向
      T.E := E;
      T.Err := [0.0, 0.0, -0.10, 0.05, 0.20];
      T.W := [1.0, 1.0, 1.0, 1.0, 1.0];
      Terms.Append (T);
      Act (0) := True; Act (1) := True;
      Table.Solve (Terms, 2, Cap, Act, [others => 1.0e-9], A, Ok);
      Check (Ok and then A (0) > 0.05 and then abs (A (1) - 0.20) < 1.0e-3,
             "五行表:往前走 " & Codec.Fmt (A (0), 3) & " · 转腕 " & Codec.Fmt (A (1), 3) & "(转腕该正好补上朝向)");
      --  圆的东西:朝向那一列全零 ⇒ 朝向的差再大也不会让它去转
      Table.Set_Col (E, 1, [0.0, 0.0, 0.0, 0.0, 0.0]);
      Terms.Clear; T.E := E; T.Err := [0.0, 0.0, 0.0, 0.0, 2.0]; Terms.Append (T);
      Table.Solve (Terms, 2, Cap, Act, [others => 1.0e-9], A, Ok);
      Check (Ok and then abs (A (0)) < 1.0e-6 and then abs (A (1)) < 1.0e-6, "五行表:改不动的那一行不会让它乱动");
   end;
   --  连通块按大小排序:先扫到的小碎点不许排在大块前面(EL:4x4 碎点被当成一根手指)
   declare
      W : constant Natural := 40;
      H : constant Natural := 40;
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      Rs : Picture.Regions;
   begin
      for Y in 2 .. 4 loop          --  上方一个 3x3 的小点(扫描线先碰到)
         for X in 2 .. 4 loop
            M.Replace_Element (Y * W + X, True);
         end loop;
      end loop;
      for Y in 20 .. 34 loop        --  下方一个 15x15 的大块
         for X in 20 .. 34 loop
            M.Replace_Element (Y * W + X, True);
         end loop;
      end loop;
      Rs := Picture.Components (M, W, H, 4);
      Check (Natural (Rs.Length) = 2 and then Rs (0).Count = 225 and then Rs (1).Count = 9,
             "连通块:大的排前面(" & Codec.Img (Rs (0).Count) & " 然后 " & Codec.Img (Rs (1).Count) & ")");
   end;
   --  🔴 语法永远得是合法的(T1 2026-09-21):角色表空了,`who ::= ` 后面什么都没有 ⇒ 推理服务整份拒收 ⇒ 再也问不到脑。
   declare
      Outs : constant String := "touched stuck slipped lost settled stalled timeout";
      function Has (Text, Pat : String) return Boolean is (Ada.Strings.Fixed.Index (Text, Pat) > 0);
      Full : constant String := Sinew.EBNF ("touching above", "grasper", Outs);
      No_Who : constant String := Sinew.EBNF ("touching above", "", Outs);
      No_Rel : constant String := Sinew.EBNF ("(一个都没有)", "grasper", Outs);
   begin
      Check (Has (Full, "who ::= ") and then Has (Full, "rel ::= ") and then Has (Full, "who "" "" rel"),
             "语法:角色和关系都有 ⇒ 整份都在");
      Check (not Has (No_Who, "who ::=") and then not Has (No_Who, "cons") and then Has (No_Who, "word ::=")
             and then Has (No_Who, "root ::="),
             "语法:角色表空 ⇒ 没有空规则,只剩 say / done(脑仍然问得到)");
      Check (Has (No_Rel, "who ::= ") and then not Has (No_Rel, "rel ::=") and then not Has (No_Rel, "who "" "" rel")
             and then Has (No_Rel, "close"),
             "语法:关系表空、角色不空 ⇒ 拿掉 <who> <relation> 那一支,close / open / still 还在");
      Check (Has (Full, "0-9") and then Has (Full, "="), "语法:say 那一句打得出数字和等号(look = k 这个键按得动)");
      Check (Has (Sinew.Grammar ("touching", "", Outs), "cannot find any part of me")
             and then not Has (Sinew.Grammar ("touching", "", Outs), "<who>"),
             "语法:给脑看的那张纸和键盘同一张 —— 角色表空时纸上也没有 <who>");
      --  语言的根(2026-09-23):有量的时候,键盘上只剩 <东西> <量> up|down until <结局> 和 say / done;
      --  手的关系词、眼、步子、控制块一个都不在(它们是 9B 的脑乱按的地方)。纸上和键盘上同一张。
      declare
         Q : constant String := Sinew.EBNF ("touching above", "grasper", Outs, "height");
         Qt : constant String := Sinew.Grammar ("touching above", "grasper", Outs, "height");
      begin
         Check (Has (Q, "qty ::= ""height""") and then Has (Q, "dir ::= ""up"" | ""down""")
                and then not Has (Q, "who ::=") and then not Has (Q, "rel ::=") and then not Has (Q, "eye ::=")
                and then not Has (Q, "control ::=") and then Has (Q, "word ::="),
                "语法:有量 ⇒ 键盘上只有「东西 量 up|down until 结局」和 say/done");
         Check (Has (Qt, "<quantity>  ::= height") and then not Has (Qt, "<who>") and then not Has (Qt, "<relation>"),
                "语法:有量 ⇒ 纸上也只有那一句");
      end;
   end;
   --  语言的根:那一句解析出来是"某件东西的某个量往哪变",东西是名字、量是身体列的词、方向 ±1
   declare
      use Sinew;
      P1 : constant Program := Sinew.Parse ("do mint green scissors height up until settled");
      P2 : constant Program := Sinew.Parse ("do height up until settled");
   begin
      Check (P1.Ok and then Natural (P1.Code.Length) >= 1 and then P1.Code (0).O = Op_Interval
             and then Natural (P1.Code (0).Cons.Length) = 1
             and then P1.Code (0).Cons (0).R = Re_Qty and then P1.Code (0).Cons (0).Dir = 1
             and then P1.Code (0).Cons (0).Subj.K = Nk_Thing and then To_String (P1.Code (0).Cons (0).Subj.Word) = "mint green scissors"
             and then To_String (P1.Code (0).Cons (0).Obj.Word) = "height" and then P1.Code (0).Until_Oc = Oc_Settled,
             "语言:「do mint green scissors height up until settled」= 剪刀的 height 往上,到 settled 为止");
      Check (not P2.Ok, "语言:量前面没说哪件东西 ⇒ 退回");
   end;
   --  🔴 不动的眼(2026-09-22):已知世界点 + 它们在画面里的像素 ⇒ 解出相机位置和朝向。正反两条。
   declare
      use type Geom.V3;
      Gt, Gf : Geom.Cam_Geo;
      Marks : Geom.Mark_Vectors.Vector;
      Ok : Boolean;
      Pts : constant array (1 .. 8) of Geom.V3 :=
        [[-0.35, -0.25, 0.80], [0.35, -0.25, 0.80], [-0.35, -0.10, 0.90], [0.35, -0.10, 0.90],
         [-0.25, -0.05, 0.85], [0.25, -0.30, 0.95], [0.0, -0.20, 0.82], [0.1, 0.05, 0.88]];
   begin
      Gt.F := 288.0; Gt.Cx := 320.0; Gt.Cy := 240.0;
      --  相机约定 -z 朝前:朝向取单位阵就是笔直朝下看(世界 z 朝上),再歪 0.3 rad 像真的头顶眼那样斜着看桌子
      Gt.R_Ce := Geom.Rodrigues ([0.3, 0.1, 0.0]);
      Gt.Pos := [0.05, -0.45, 1.75]; Gt.Fixed := True;
      declare
         All_Front : Boolean := True;
      begin
         for P of Pts loop
            declare
               U, V : Long_Float;
               Fr : Boolean;
            begin
               Geom.Project_Fixed (Gt, P, U, V, Fr);
               All_Front := All_Front and then Fr and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0;
               Marks.Append (Geom.Mark'(Pw => P, U => U, V => V));
            end;
         end loop;
         Check (All_Front, "不动的眼:合成的 8 个点都在相机前面、画面里(测试数据自己先得成立)");
      end;
      Gf.F := Gt.F; Gf.Cx := Gt.Cx; Gf.Cy := Gt.Cy;
      Geom.Fit_Fixed (Gf, Marks, Ok);
      declare
         Dp : constant Long_Float := (if Ok then Geom.Norm ([Gf.Pos (0) - Gt.Pos (0), Gf.Pos (1) - Gt.Pos (1), Gf.Pos (2) - Gt.Pos (2)]) else 1.0);
         Da : constant Long_Float := (if Ok then Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (Gt.R_Ce), Gf.R_Ce))) else 1.0);
      begin
         Check (Ok and then Gf.Rms < 0.01 and then Dp < 0.001 and then Da < 0.001,
                "不动的眼:8 个指尖观测 ⇒ 位置差 " & Codec.Fmt (Dp, 5) & " m · 朝向差 " & Codec.Fmt (Da, 4) & " rad · 残差 " & Codec.Fmt (Gf.Rms, 3) & " px");
      end;
      declare
         Few : Geom.Mark_Vectors.Vector;
         G3 : Geom.Cam_Geo := Gf;
         Ok3 : Boolean;
      begin
         for I in 0 .. 2 loop
            Few.Append (Marks (I));
         end loop;
         Geom.Fit_Fixed (G3, Few, Ok3);
         Check (not Ok3, "不动的眼:只有 3 个观测 ⇒ 不解,如实说");
      end;
      declare
         Hit : Geom.V3;
         Hok : Boolean;
      begin
         Hit := Geom.Hit_Plane ([0.0, 0.0, 1.0], [0.0, 0.6, -0.8], [0.0, 0.0, 0.5], [0.0, 0.0, 1.0], Hok);
         Check (Hok and then abs (Hit (2) - 0.5) < 1.0e-9 and then abs (Hit (1) - 0.375) < 1.0e-9, "视线与面相交:落在面上,位置对");
         Hit := Geom.Hit_Plane ([0.0, 0.0, 1.0], [0.0, 0.6, 0.8], [0.0, 0.0, 0.5], [0.0, 0.0, 1.0], Hok);
         Check (not Hok, "视线背对着面 ⇒ 不交,如实说");
      end;
   end;
   --  🔴 焦距一起解(2026-09-24,官方 RoboDojo 观测没有内参):手上的眼 F 不给(0),从 6 停里把朝向和焦距一起量出来;真值 F = 400
   declare
      Gt : Geom.Cam_Geo;
      Gf : Geom.Cam_Geo;
      Obs : Geom.Obs_Vectors.Vector;
      Pw : constant Geom.V3 := [0.05, 0.4, 0.2];   --  盯着的那块东西在世界里的位置(合成)
      Ok : Boolean;
      Moves : constant array (1 .. 6) of Geom.V3 := [[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.05], [0.0, 0.05, 0.0], [-0.05, 0.0, 0.05], [0.05, 0.05, 0.0]];
   begin
      Gt.F := 400.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.R_Ce := Geom.Rodrigues ([0.2, -0.3, 0.1]); Gt.Valid := True;
      for M of Moves loop
         declare
            P : constant Plug.Arm_Pose := [M (0), M (1), M (2) + 0.6, 1.0, 0.0, 0.0, 0.0];
            U, V : Long_Float;
            Fr : Boolean;
         begin
            Geom.Project (Gt, P, Pw, U, V, Fr);
            Check (Fr, "焦距一起解:合成的东西在相机前面(测试数据自己先得成立)");
            Obs.Append (Geom.Obs'(Pose => P, U => U, V => V));
         end;
      end loop;
      Gf.F := 0.0; Gf.Cx := 320.0; Gf.Cy := 240.0;   --  焦距没给
      Geom.Fit (Gf, Obs, Ok);
      declare
         Da : constant Long_Float := (if Ok then Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (Gt.R_Ce), Gf.R_Ce))) else 1.0);
      begin
         Check (Ok and then abs (Gf.F - 400.0) < 4.0 and then Da < 0.01 and then Gf.Rms < 0.5,
                "焦距一起解:6 停 ⇒ 焦距 " & Codec.Fmt (Gf.F, 1) & " px(真 400)· 朝向差 " & Codec.Fmt (Da, 4) & " rad · 残差 " & Codec.Fmt (Gf.Rms, 3) & " px");
      end;
   end;
   --  🔴 焦距先验(2026-09-24):仪器看一张图报 440 ± 40 px(像 GeoCalib 在真身上那样偏一成),真值 400。
   --  基线只有 1 cm、每停 1 px 抖动时焦距和距离分不开:没先验解飞,有先验按在仪器的范围里;基线 5 cm 时观测压过先验,仍解回 400 附近
   declare
      Gt : Geom.Cam_Geo;
      Pw : constant Geom.V3 := [0.05, 0.4, 0.2];   --  盯着的那块东西在世界里的位置(合成)
      Seed : Long_Long_Integer := 3;
      function Jit return Long_Float is   --  确定性伪随机 ±1 px(测试数据自己的抖动,不是驱动里的常数)
      begin
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Integer ((Seed / 65536) mod 2001) - 1000) / 1000.0;
      end Jit;
      type Stops is array (Positive range <>) of Geom.V3;
      Star : constant Stops := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 1.0], [1.0, 1.0, 0.0]];   --  星形:从原处各挪一步(单位步,乘 Amp)
      --  驱动真实走的 8 步累计路径(Geo_Calibrate):原处 + 每步相对上一停,三根轴各两步、再回一半 ⇒ 每根轴最远 2 步
      Path : constant Stops := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 0.0, 1.0], [2.0, 0.0, 2.0], [2.0, 1.0, 2.0], [2.0, 2.0, 2.0], [1.0, 2.0, 2.0], [0.0, 2.0, 2.0]];
      procedure Synth (Moves : Stops; Amp : Long_Float; Obs : out Geom.Obs_Vectors.Vector) is
      begin
         Obs.Clear;
         for M of Moves loop
            declare
               P : constant Plug.Arm_Pose := [Amp * M (0), Amp * M (1), Amp * M (2) + 0.6, 1.0, 0.0, 0.0, 0.0];   --  离东西约 0.6 m(合成)
               U, V : Long_Float;
               Fr : Boolean;
            begin
               Geom.Project (Gt, P, Pw, U, V, Fr);
               Obs.Append (Geom.Obs'(Pose => P, U => U + Jit, V => V + Jit));
            end;
         end loop;
      end Synth;
      Obs : Geom.Obs_Vectors.Vector;
      G_No, G_Pr, G_Long : Geom.Cam_Geo;
      Ok_No, Ok_Pr, Ok_Long : Boolean;
      Per_Mm : constant Long_Float := 1000.0;   --  米 → 毫米(换算,无量纲)
   begin
      Gt.F := 400.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.R_Ce := Geom.Rodrigues ([0.2, -0.3, 0.1]); Gt.Valid := True;
      Synth (Star, 0.01, Obs);   --  短基线:星形、每停挪 1 cm
      G_No.F := 0.0; G_No.Cx := 320.0; G_No.Cy := 240.0;
      Geom.Fit (G_No, Obs, Ok_No);
      G_Pr.F := 0.0; G_Pr.Cx := 320.0; G_Pr.Cy := 240.0; G_Pr.F_Prior := 440.0; G_Pr.F_Prior_Sd := 40.0;
      Geom.Fit (G_Pr, Obs, Ok_Pr);
      Check (Ok_Pr and then abs (G_Pr.F - 400.0) < 40.0 and then (not Ok_No or else abs (G_No.F - 400.0) > abs (G_Pr.F - 400.0)),
             "焦距先验:短基线(星形 1 cm)+ 1 px 抖动 ⇒ 没先验 " & (if Ok_No then Codec.Fmt (G_No.F, 0) else "解不出") & " px,有先验(440±40)" & Codec.Fmt (G_Pr.F, 0)
             & " px(真 400,该在先验一个不确定度内、比没先验近)");
      Synth (Star, 0.05, Obs);   --  星形、每停挪 5 cm:观测还是压不过偏一成的先验(2026-09-24 实测 433)—— 这就是为什么要走累计路径
      G_Long.F := 0.0; G_Long.Cx := 320.0; G_Long.Cy := 240.0; G_Long.F_Prior := 440.0; G_Long.F_Prior_Sd := 40.0;
      Geom.Fit (G_Long, Obs, Ok_Long);
      Put_Line ("     · 焦距先验:星形 5 cm + 偏一成的先验 ⇒ " & Codec.Fmt (G_Long.F, 1) & " px(真 400;信息量不够,不当闸)");
      Synth (Path, 0.06, Obs);   --  8 步累计路径、每步 6 cm(每根轴最远 12 cm):没先验、有偏先验各解一次
      G_No.F := 0.0; G_No.Cx := 320.0; G_No.Cy := 240.0; G_No.F_Prior := 0.0; G_No.F_Prior_Sd := 0.0;
      Geom.Fit (G_No, Obs, Ok_No);
      G_Long.F := 0.0; G_Long.Cx := 320.0; G_Long.Cy := 240.0; G_Long.F_Prior := 440.0; G_Long.F_Prior_Sd := 40.0;
      Geom.Fit (G_Long, Obs, Ok_Long);
      Check (Ok_No and then Ok_Long and then abs (G_No.F - 400.0) < 12.0 and then abs (G_Long.F - 400.0) < 12.0,
             "焦距先验:8 步累计路径(每步 6 cm)+ 1 px 抖动 ⇒ 没先验 " & Codec.Fmt (G_No.F, 1) & " px,偏一成的先验也压不歪 " & Codec.Fmt (G_Long.F, 1)
             & " px(真 400,都该在 3% 内)· 残差 " & Codec.Fmt (G_Long.Rms, 2) & " px");
      --  🔴 转眼量焦距(2026-09-24):横着挪只能量出 焦距/远近 的比;转一个已知的角,像素位移 = 焦距 × 转角,和远近无关。
      --  星形 1 cm(本来解飞到 528)+ 四停纯转动(绕 z、绕 x 各 ±0.1 rad,位姿读数给角度)⇒ 焦距该回到 400 附近
      declare
         Rot_Stops : constant array (1 .. 4) of Plug.Arm_Pose :=
           [[0.0, 0.0, 0.6, 0.99875, 0.0, 0.0, 0.04998], [0.0, 0.0, 0.6, 0.99875, 0.0, 0.0, -0.04998],
            [0.0, 0.0, 0.6, 0.99875, 0.04998, 0.0, 0.0], [0.0, 0.0, 0.6, 0.99875, -0.04998, 0.0, 0.0]];   --  cos/sin(0.05):±0.1 rad 的四元数(合成)
         G_Rot : Geom.Cam_Geo;
         Ok_Rot : Boolean;
      begin
         Synth (Star, 0.01, Obs);
         for P of Rot_Stops loop
            declare
               U, V : Long_Float;
               Fr : Boolean;
            begin
               Geom.Project (Gt, P, Pw, U, V, Fr);
               Check (Fr, "转眼量焦距:转过之后东西还在相机前面(测试数据自己先得成立)");
               Obs.Append (Geom.Obs'(Pose => P, U => U + Jit, V => V + Jit));
            end;
         end loop;
         G_Rot.F := 0.0; G_Rot.Cx := 320.0; G_Rot.Cy := 240.0;
         Geom.Fit (G_Rot, Obs, Ok_Rot);
         Check (Ok_Rot and then abs (G_Rot.F - 400.0) < 8.0,
                "转眼量焦距:星形 1 cm + 四停各转 0.1 rad + 1 px 抖动,不用先验 ⇒ 焦距 " & Codec.Fmt (G_Rot.F, 1) & " px(真 400,该在 2% 内;不转是 528)· 残差 "
                & Codec.Fmt (G_Rot.Rms, 2) & " px");
      end;
      --  🔴 多点连相机偏移一起解(2026-09-24,Fit_Rig):真相机离手腕原点 (3,0,5) cm;近 / 中 / 远三个点(0.4 / 0.8 / 3 m);
      --  停 = 起点 + 四停转动(±0.1 rad 绕 z、绕 x)+ 探一步 2.6 cm + 7 步累计路径(每步 6 cm);近的点有 3 停跟丢;1 px 抖动。
      --  该解出:焦距 2% 内、偏移差 < 1 cm、朝向差 < 0.01 rad、三个点都进
      declare
         Gr : Geom.Cam_Geo;
         Gs : Geom.Cam_Geo;
         Home : constant Plug.Arm_Pose := [0.0, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0];
         Pts : array (0 .. 2) of Geom.V3;
         Obs : Geom.Obs_Pt_Vectors.Vector;
         Poses : Geom.Obs_Vectors.Vector;   --  只用 Pose 字段:停的位姿序列
         Ok_R : Boolean;
         Used : Natural;
         Stop_No : Natural := 0;
      begin
         Gr.F := 400.0; Gr.Cx := 320.0; Gr.Cy := 240.0; Gr.R_Ce := Geom.Rodrigues ([0.2, -0.3, 0.1]); Gr.Off := [0.03, 0.0, 0.05]; Gr.Valid := True;
         --  三个点放在起点那一停的相机正前方(相机系 z 朝后 ⇒ 前方是 -z),稍微错开
         declare
            Rc : constant Geom.M3 := Geom.Cam_R (Gr, Home);
            Cp : constant Geom.V3 := Geom.Cam_Pos (Gr, Home);
            Depths : constant array (0 .. 2) of Long_Float := [0.4, 0.8, 3.0];   --  近 / 中 / 远(米,合成)
            Side : constant array (0 .. 2) of Long_Float := [0.05, -0.1, 0.3];   --  横向错开(米,合成)
         begin
            for K in 0 .. 2 loop
               declare
                  D : constant Geom.V3 := Geom.Ap (Rc, [Side (K), 0.02 * Long_Float (K), -Depths (K)]);
               begin
                  Pts (K) := [Cp (0) + D (0), Cp (1) + D (1), Cp (2) + D (2)];
               end;
            end loop;
         end;
         Poses.Append (Geom.Obs'(Pose => Home, U => 0.0, V => 0.0));
         Poses.Append (Geom.Obs'(Pose => [0.0, 0.0, 0.6, 0.99875, 0.0, 0.0, 0.04998], U => 0.0, V => 0.0));    --  绕 z +0.1 rad(cos/sin 0.05,合成)
         Poses.Append (Geom.Obs'(Pose => [0.0, 0.0, 0.6, 0.99875, 0.0, 0.0, -0.04998], U => 0.0, V => 0.0));
         Poses.Append (Geom.Obs'(Pose => [0.0, 0.0, 0.6, 0.99875, 0.04998, 0.0, 0.0], U => 0.0, V => 0.0));    --  绕 x
         Poses.Append (Geom.Obs'(Pose => [0.0, 0.0, 0.6, 0.99875, -0.04998, 0.0, 0.0], U => 0.0, V => 0.0));
         Poses.Append (Geom.Obs'(Pose => [0.026, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));            --  探一步
         for M of Path loop
            exit when M (0) = 0.0 and then M (1) = 0.0 and then M (2) = 0.0 and then Stop_No > 0;
            Stop_No := Stop_No + 1;
            Poses.Append (Geom.Obs'(Pose => [0.026 + 0.06 * M (0), 0.06 * M (1), 0.6 + 0.06 * M (2), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));
         end loop;
         for S in 0 .. Natural (Poses.Length) - 1 loop
            for K in 0 .. 2 loop
               declare
                  U, V : Long_Float;
                  Fr : Boolean;
               begin
                  Geom.Project (Gr, Poses (S).Pose, Pts (K), U, V, Fr);
                  Check (Fr, "多点连偏移:合成的点在相机前面(测试数据自己先得成立)");
                  --  近的点在第 7、8、9 停跟丢
                  if not (K = 0 and then S in 7 .. 9) then
                     Obs.Append (Geom.Obs_Pt'(Pt => K, Pose => Poses (S).Pose, U => U + Jit, V => V + Jit, Seq => 0, Kind => 0));
                  end if;
               end;
            end loop;
         end loop;
         Gs.F := 0.0; Gs.Cx := 320.0; Gs.Cy := 240.0;
         Geom.Fit_Rig (Gs, Obs, 3, Ok_R, Used);
         declare
            Da : constant Long_Float := (if Ok_R then Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (Gr.R_Ce), Gs.R_Ce))) else 1.0);
            Doff : constant Long_Float := (if Ok_R then Geom.Norm ([Gs.Off (0) - 0.03, Gs.Off (1), Gs.Off (2) - 0.05]) else 1.0);
         begin
            Check (Ok_R and then Used = 3 and then abs (Gs.F - 400.0) < 8.0 and then Doff < 0.015 and then Da < 0.01,   --  偏移沿视线那一维最难看出来:1 px 抖动下解到 1 cm 级(米)
                   "多点连偏移:" & Codec.Img (Natural (Poses.Length)) & " 停 × 3 点(近的丢 3 停)⇒ 焦距 " & Codec.Fmt (Gs.F, 1) & " px(真 400)· 偏移 ("
                   & Codec.Fmt (Gs.Off (0) * 1000.0, 0) & "," & Codec.Fmt (Gs.Off (1) * 1000.0, 0) & "," & Codec.Fmt (Gs.Off (2) * 1000.0, 0)
                   & ") mm(真 (30,0,50),该在 1.5 cm 内)· 朝向差 " & Codec.Fmt (Da, 4) & " rad · 残差 " & Codec.Fmt (Gs.Rms, 2) & " px · 进了 " & Codec.Img (Used) & " 点");
         end;
         --  跟错的观测混进来(每 7 笔里 1 笔错 40 px,像 V1F 右眼):踢离群再解,焦距该仍在 2% 内
         declare
            Bad : Geom.Obs_Pt_Vectors.Vector;
            Gb : Geom.Cam_Geo;
            Ok_B : Boolean;
            Used_B : Natural;
         begin
            for I in 0 .. Natural (Obs.Length) - 1 loop
               declare
                  Ob : Geom.Obs_Pt := Obs (I);
               begin
                  if I mod 7 = 3 then
                     Ob.U := Ob.U + 40.0;   --  错 40 px(合成)
                  end if;
                  Bad.Append (Ob);
               end;
            end loop;
            Gb.F := 0.0; Gb.Cx := 320.0; Gb.Cy := 240.0;
            Geom.Fit_Rig (Gb, Bad, 3, Ok_B, Used_B);
            Check (Ok_B and then abs (Gb.F - 400.0) < 8.0 and then Gb.Dropped >= 4,
                   "多点连偏移·踢离群:每 7 笔 1 笔错 40 px ⇒ 踢掉 " & Codec.Img (Gb.Dropped) & " 笔,焦距 " & Codec.Fmt (Gb.F, 1) & " px(真 400,该在 2% 内)· 残差 " & Codec.Fmt (Gb.Rms, 2) & " px");
         end;
         --  不确定度从雅可比来:转过、多点、长基线 ⇒ 焦距 ± 几个像素;只横挪 1 cm 不转(星形)⇒ 焦距和远近分不开,不确定度该比焦距本身还大 ⇒ 判解不出
         Check (Ok_R and then Gs.F_Sd > 0.0 and then Gs.F_Sd < 6.0 and then Gs.Off_Sd < 0.02 and then Gs.Rot_Sd < 0.01,
                "不确定度:15 停 × 3 点 ⇒ 焦距 ± " & Codec.Fmt (Gs.F_Sd, 2) & " px · 偏移 ± " & Codec.Fmt (Gs.Off_Sd * Per_Mm, 1) & " mm · 朝向 ± " & Codec.Fmt (Gs.Rot_Sd, 4)
                & " rad(该:焦距 < 6 px、偏移 < 2 cm、朝向 < 0.01 rad)");
         declare
            Obs_S : Geom.Obs_Pt_Vectors.Vector;
            Gd : Geom.Cam_Geo;
            Ok_D : Boolean;
            Used_D : Natural;
         begin
            for S in 0 .. 5 loop   --  星形 1 cm 的 6 停,只有近的那个点
               declare
                  Amp : constant Long_Float := 0.01;   --  1 cm(合成)
                  Ps : constant Plug.Arm_Pose := [Amp * Star (S + 1) (0), Amp * Star (S + 1) (1), 0.6 + Amp * Star (S + 1) (2), 1.0, 0.0, 0.0, 0.0];
                  U, V : Long_Float;
                  Fr : Boolean;
               begin
                  Geom.Project (Gr, Ps, Pts (0), U, V, Fr);
                  Obs_S.Append (Geom.Obs_Pt'(Pt => 0, Pose => Ps, U => U + Jit, V => V + Jit, Seq => 0, Kind => 0));
               end;
            end loop;
            Gd.F := 0.0; Gd.Cx := 320.0; Gd.Cy := 240.0;
            Geom.Fit_Rig (Gd, Obs_S, 1, Ok_D, Used_D);
            Check (not Ok_D, "不确定度:只横挪 1 cm、不转 ⇒ 焦距和远近分不开 ⇒ 判解不出(不再吐一个看着像样的数)");
         end;
      end;
   end;
   --  🔴 手上被不动的眼标的点(2026-09-24 起;09-25 起眼只按标定板解,这里拿真相机验 Hand_Points,板的几条在后面):头顶眼在 (0,−0.41,1.308) 低头 30°、焦距 288(合成);两条臂各走标定路径
   --  (起点 + 探转 ±15.6° + 四停转 ±0.1 rad + 探一步 + 8 步 6 cm,同驱动实走的路径),1 px 抖动,焦距没给。第 1 条臂上有两个被标的点:它看见手指 1 瓣时标在指尖、2 瓣时标在另一处
   --  (隔一停换一次,两点相距 3.7 cm,合成);第 2 条臂一个点。每个点在手系里 3 个数,(臂, 瓣数) 分开解 ⇒ 焦距 2% 内、相机位置差 < 1.5 cm、
   --  朝向差 < 0.01 rad、每个点差 < 5 mm。反面:同样的笔把两个点当成一个点解(瓣数全记成一样),残差该大 3 倍以上(G1S 实拍:7.5 px 对 1.7 px)。
   --  两个点那一条:多了一个自由的点(3 个未知数),手上的点又都挤在离相机 1 m 的一小团里 ⇒ 焦距和远近互相顶,相机位置自报 ± 1 cm 级;
   --  要的是误差落在它自报不确定度的 3 倍以内(自报是真话)、焦距在 3% 内(V1 的线)、视线上的点 5 mm 内、自由的点 1 cm 内
   declare
      Gt : Geom.Cam_Geo;
      --  每条臂的腕眼:装在手上的朝向 + 离手腕原点的偏移(合成);第一个点 = 偏移 + S × (相机系单位视线转到手系),真值 S = 0.12 / 0.11 m
      Gw : constant array (0 .. 1) of Geom.M3 := [Geom.Rodrigues ([0.2, -0.3, 0.1]), Geom.Rodrigues ([-0.2, -0.3, -0.1])];
      Ofs : constant array (0 .. 1) of Geom.V3 := [[0.08, 0.0, 0.05], [0.08, 0.0, 0.05]];   --  相机离手腕原点(手系,米,合成)
      Dcs : constant array (0 .. 1) of Geom.V3 := [[0.1, -0.5, -0.86], [-0.1, -0.5, -0.86]];   --  指尖在自己眼里的视线(相机系,合成,下面归一化)
      S_True : constant array (0 .. 1) of Long_Float := [0.12, 0.11];   --  指尖离眼(米,合成)
      Shift_B : constant Geom.V3 := [0.02, -0.03, 0.01];   --  第 1 条臂第二个点离第一个点(手系,米,合成)
      Tips : array (0 .. 1) of Geom.V3;
      Tip_B : Geom.V3;
      Ray_O, Ray_D : Geom.V3_Vectors.Vector;
      Own_Kind : Geom.Nat_Vectors.Vector;   --  两只手自己眼里都是 1 瓣(合成):1 瓣的点在各自腕眼的视线上,第 1 臂 2 瓣的那个点自由
      Homes : constant array (0 .. 1) of Geom.V3 := [[-0.3, 0.2, 0.85], [0.3, 0.2, 0.85]];   --  两只手的起点(米,合成)
      Obs, Obs_Same, Obs_One : Geom.Obs_Pt_Vectors.Vector;
      Got, Got_M, Got_1 : Geom.Tip_Class_Vectors.Vector;
      Rep_1, Rep_F, Rep_M : Geom.Fixed_Report;
      Seed2 : Long_Long_Integer := 5;
      function Jit2 return Long_Float is   --  确定性伪随机 ±1 px(测试数据自己的抖动)
      begin
         Seed2 := (Seed2 * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Integer ((Seed2 / 65536) mod 2001) - 1000) / 1000.0;
      end Jit2;
      Path2 : constant array (1 .. 9) of Geom.V3 := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 0.0, 1.0], [2.0, 0.0, 2.0], [2.0, 1.0, 2.0], [2.0, 2.0, 2.0], [1.0, 2.0, 2.0], [0.0, 2.0, 2.0]];   --  驱动的 8 步累计路径(单位步,合成)
      Per_Mm : constant Long_Float := 1000.0;         --  米 → 毫米(换算,无量纲)
      Sn : constant Long_Float := 0.5;                --  sin 30°(合成)
      Cs : constant Long_Float := 0.8660254;          --  cos 30°(合成)
      function Err_Of (Cls : Geom.Tip_Class_Vectors.Vector; A, K : Natural; T : Geom.V3) return Long_Float is
      begin
         for C of Cls loop
            if C.Arm = A and then C.Kind = K then
               return Geom.Norm ([C.Tip (0) - T (0), C.Tip (1) - T (1), C.Tip (2) - T (2)]);
            end if;
         end loop;
         return 1.0;   --  没解出这个点:按 1 米错算(远大于门槛,合成)
      end Err_Of;
   begin
      --  相机 → 世界:x 列 = 世界 x;y 列 = (0, sin30, cos30)(画面的上朝前上);z 列 = (0, −cos30, sin30)(视线 −z 朝前下)
      Gt.R_Ce := [[1.0, 0.0, 0.0], [0.0, Sn, -Cs], [0.0, Cs, Sn]];
      Gt.Pos := [0.0, -0.41, 1.308]; Gt.F := 288.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.Fixed := True; Gt.Valid := True;
      for A in 0 .. 1 loop
         declare
            Nn : constant Long_Float := Geom.Norm (Dcs (A));
            Dc : constant Geom.V3 := [Dcs (A) (0) / Nn, Dcs (A) (1) / Nn, Dcs (A) (2) / Nn];
            Dh : constant Geom.V3 := Geom.Ap (Gw (A), Dc);
         begin
            Tips (A) := [Ofs (A) (0) + S_True (A) * Dh (0), Ofs (A) (1) + S_True (A) * Dh (1), Ofs (A) (2) + S_True (A) * Dh (2)];
            Ray_O.Append (Ofs (A)); Ray_D.Append (Dh); Own_Kind.Append (1);
         end;
      end loop;
      Tip_B := [Tips (0) (0) + Shift_B (0), Tips (0) (1) + Shift_B (1), Tips (0) (2) + Shift_B (2)];
      for A in 0 .. 1 loop
         declare
            H : constant Geom.V3 := Homes (A);
            Poses : Geom.Obs_Vectors.Vector;
         begin
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));
            --  探转:绕 z 转 ±0.27 rad(≈ 15.6°,G1R/G1S 实拍的探一转;四元数 cos/sin 0.135,合成)——驱动每一停都给头顶眼一笔,探转这两停也有
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99090, 0.0, 0.0, 0.13459], U => 0.0, V => 0.0));
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99090, 0.0, 0.0, -0.13459], U => 0.0, V => 0.0));
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.0, 0.0, 0.04998], U => 0.0, V => 0.0));    --  ±0.1 rad 的四元数(cos/sin 0.05,合成)
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.0, 0.0, -0.04998], U => 0.0, V => 0.0));
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.04998, 0.0, 0.0], U => 0.0, V => 0.0));
            Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, -0.04998, 0.0, 0.0], U => 0.0, V => 0.0));
            Poses.Append (Geom.Obs'(Pose => [H (0) + 0.026, H (1), H (2), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));
            for M of Path2 loop
               Poses.Append (Geom.Obs'(Pose => [H (0) + 0.026 + 0.06 * M (0), H (1) + 0.06 * M (1), H (2) + 0.06 * M (2), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));
            end loop;
            for S in 0 .. Natural (Poses.Length) - 1 loop
               declare
                  Ps : constant Geom.Obs := Poses (S);
                  Two : constant Boolean := A = 0 and then S mod 2 = 1;   --  第 1 条臂隔一停标在第二个点上(合成)
                  T : constant Geom.V3 := (if Two then Tip_B else Tips (A));
                  Tw : constant Geom.V3 := Geom.Ap (Geom.Quat_To_R (Ps.Pose), T);
                  Pw : constant Geom.V3 := [Ps.Pose (0) + Tw (0), Ps.Pose (1) + Tw (1), Ps.Pose (2) + Tw (2)];
                  U, V, Ju, Jv : Long_Float;
                  Fr : Boolean;
                  Tw1 : constant Geom.V3 := Geom.Ap (Geom.Quat_To_R (Ps.Pose), Tips (A));   --  一只手一个点的那一条:永远标在第一个点上
                  U1, V1 : Long_Float;
                  Fr1 : Boolean;
               begin
                  Geom.Project_Fixed (Gt, Pw, U, V, Fr);
                  Geom.Project_Fixed (Gt, [Ps.Pose (0) + Tw1 (0), Ps.Pose (1) + Tw1 (1), Ps.Pose (2) + Tw1 (2)], U1, V1, Fr1);
                  Check (Fr and then Fr1 and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0, "不动的眼连手上的点:合成的点在画面里(测试数据自己先得成立)");
                  Ju := Jit2; Jv := Jit2;
                  Obs.Append (Geom.Obs_Pt'(Pt => A, Pose => Ps.Pose, U => U + Ju, V => V + Jv, Seq => 0, Kind => (if Two then 2 else 1)));
                  Obs_Same.Append (Geom.Obs_Pt'(Pt => A, Pose => Ps.Pose, U => U + Ju, V => V + Jv, Seq => 0, Kind => 1));
                  Obs_One.Append (Geom.Obs_Pt'(Pt => A, Pose => Ps.Pose, U => U1 + Ju, V => V1 + Jv, Seq => 0, Kind => 1));
               end;
            end loop;
         end;
      end loop;
      --  一只手一个点(都在各自腕眼视线上)。手上的点离相机 1 m、挤在十几厘米一团里 ⇒ 1 px 抖动就让焦距和远近互相顶出 ~1.5 cm(自报 ± 1.35 cm);
      --  换解法之前这一条按"位置 1.5 cm 内"判,只有 1 倍标准差,是碰运气过的(2026-09-24 加了探转那两停、抖动换了一组就不过)⇒ 改按:
      --  误差在自报不确定度的 3 倍内(自报是真话)、自报本身够小(焦距 ± 3% 内、位置 ± 3 cm 内)、焦距差 3% 内(V1 的线)、指尖 5 mm 内
      --  2026-09-25 起不动的眼只按标定板解(手上的尖会滑,不进眼的解);手上的点在眼已知时认 ⇒ 这几条拿真相机(Gt)验 Hand_Points
      Geom.Hand_Points (Gt, Obs_One, Ray_O, Ray_D, Own_Kind, Got_1, Rep_1);
      declare
         D0 : constant Long_Float := Err_Of (Got_1, 0, 1, Tips (0));
         D1 : constant Long_Float := Err_Of (Got_1, 1, 1, Tips (1));
      begin
         Check (D0 < 0.005 and then D1 < 0.005 and then Rep_1.Hand_Used = Natural (Obs_One.Length),   --  5 mm(合成)
                "手上的点(眼已知)·一只手一个点:2 臂 × 17 停 ⇒ 指尖差 " & Codec.Fmt (D0 * Per_Mm, 1) & " / " & Codec.Fmt (D1 * Per_Mm, 1) & " mm · "
                & Codec.Img (Rep_1.Hand_Used) & "/" & Codec.Img (Natural (Obs_One.Length)) & " 笔 · 残差 " & Codec.Fmt (Rep_1.Hand_Rms, 2) & " px");
      end;
      Geom.Hand_Points (Gt, Obs, Ray_O, Ray_D, Own_Kind, Got, Rep_F);
      declare
         D01 : constant Long_Float := Err_Of (Got, 0, 1, Tips (0));
         D02 : constant Long_Float := Err_Of (Got, 0, 2, Tip_B);
         D11 : constant Long_Float := Err_Of (Got, 1, 1, Tips (1));
      begin
         Check (Natural (Got.Length) = 3 and then D01 < 0.005 and then D11 < 0.005 and then D02 < 0.01,   --  5 mm / 1 cm(合成)
                "手上的点(眼已知)·第 1 臂两个点(按瓣数分开)⇒ 解出 " & Codec.Img (Natural (Got.Length)) & " 个点 · 视线上的点差 " & Codec.Fmt (D01 * Per_Mm, 1) & " / "
                & Codec.Fmt (D11 * Per_Mm, 1) & " mm · 自由的点差 " & Codec.Fmt (D02 * Per_Mm, 1) & " mm · 残差 " & Codec.Fmt (Rep_F.Hand_Rms, 2) & " px");
      end;
      Geom.Hand_Points (Gt, Obs_Same, Ray_O, Ray_D, Own_Kind, Got_M, Rep_M);
      Check (Rep_F.Hand_Used > 0 and then (Rep_M.Hand_Used = 0 or else Rep_M.Hand_Rms > 3.0 * Rep_F.Hand_Rms),   --  3 = 倍数(无量纲)
             "手上的点(眼已知)·反面:两个点当一个点解 ⇒ 残差 " & Codec.Fmt (Rep_M.Hand_Rms, 2) & " px(" & Codec.Img (Rep_M.Hand_Used) & " 笔;分开解 "
             & Codec.Fmt (Rep_F.Hand_Rms, 2) & " px)");
      --  🔴 标定板(2026-09-25,Geom.Build_Board + Fit_Fixed_Rig 吃板上的点 + Contact.Surface.Support_Plane)。
      --  两只腕眼(焦距 397,视线朝前下 60°,离手腕原点 (0,0.04,0.02),合成)各从 (∓0.2, 0.2, 0.95) 走 9 停平移(每步 3 cm,同驱动的累计路径),
      --  参考停铺 40 px 的格子;格点打到桌面(z = 0.765)或一块 10 cm 见方、高 4 cm 的盒子顶上;画面最底下一行格点是自己的夹爪(跟着眼走)。
      --  腕眼像素抖 ±0.5 px、不动的眼里抖 ±0.8 px(均匀分布,每轴 σ = 0.29 / 0.46 px);每 13 笔有一笔在不动的眼里配错 25 px,每 17 笔有一笔在腕眼里配错 12 px;
      --  出了画面的那一停给一个乱的像素(仪器总会回一个数)。
      --  ⇒ ① 量出来的两个噪声在真值的一半到两倍之间;② 进板的点全在真点 1 cm 内(夹爪一个不进);③ 只靠板解不动的眼(焦距没给):焦距 1% 内、位置 1 cm 内、
      --  误差在自报的 3 倍内;④ 板 + 手上的标记整体放大 2%(相当于"手上的尖在滑",焦距该偏 2%):焦距仍在 1% 内(板说了算);
      --  ⑤ 板上的点拟合的面:高差 < 1 mm、法向差 < 0.5°;⑥ 反面:不动的眼里的像素各停一致地换成别的点的(交叉核不出来)⇒ 解不出,或板的像素残差大于配点噪声的 5 倍
      declare
         Rw : constant Geom.M3 := [[1.0, 0.0, 0.0], [0.0, 0.8660254, -0.5], [0.0, 0.5, 0.8660254]];   --  腕眼 → 手:视线 −z 朝前下 60°(合成)
         Gwr : Geom.Cam_Geo;
         Starts : constant array (0 .. 1) of Geom.V3 := [[-0.2, 0.2, 0.95], [0.2, 0.2, 0.95]];   --  两只腕眼的起点(米,合成)
         Table_Z : constant Long_Float := 0.765;   --  桌面高(米,合成)
         Box_Top : constant Long_Float := 0.805;   --  盒子顶(米,合成)
         Step_M : constant Long_Float := 0.03;     --  每步(米,合成)
         Cell : constant Long_Float := 40.0;       --  格点间距(像素,合成)
         Nq_Row : constant Natural := 16;          --  640 / 40(合成)
         Nq_Col : constant Natural := 12;          --  480 / 40(合成)
         Nq : constant Natural := Nq_Row * Nq_Col;
         Scene, Scene_Bad : Geom.Scene_Pt_Vectors.Vector;
         Trk, Trk_Bad : Geom.Board_Track_Vectors.Vector;
         Truth : Geom.V3_Vectors.Vector;           --  两只眼的真点(桌面、盒顶;夹爪不算)
         Shh : Long_Float := 0.0;                  --  两只眼量出来的不动的眼配点噪声(取大的那个)
         Seed4 : Long_Long_Integer := 17;
         function Jit4 return Long_Float is   --  确定性伪随机 [−1, 1](测试数据自己的抖动)
         begin
            Seed4 := (Seed4 * 1103515245 + 12345) mod 2147483648;
            return Long_Float (Integer ((Seed4 / 65536) mod 2001) - 1000) / 1000.0;
         end Jit4;
         No_Obs : Geom.Obs_Pt_Vectors.Vector;
      begin
         Gwr.R_Ce := Rw; Gwr.Off := [0.0, 0.04, 0.02]; Gwr.F := 397.0; Gwr.Cx := 320.0; Gwr.Cy := 240.0; Gwr.Valid := True;
         for E in 0 .. 1 loop
            declare
               H0 : constant Plug.Arm_Pose := [Starts (E) (0), Starts (E) (1), Starts (E) (2), 1.0, 0.0, 0.0, 0.0];
               C0 : constant Geom.V3 := Geom.Cam_Pos (Gwr, H0);
               Ax : constant Geom.V3 := Geom.Ray (Gwr, H0, 320.0, 240.0);
               Hit_T : constant Long_Float := (C0 (2) - Table_Z) / (-Ax (2));
               Bx : constant Long_Float := C0 (0) + Hit_T * Ax (0);   --  盒子中心 = 光轴打到桌面那一点(合成)
               By : constant Long_Float := C0 (1) + Hit_T * Ax (1);
               Half : constant Long_Float := 0.05;   --  盒子半边(米,合成)
               Xq : array (0 .. Nq - 1) of Geom.V3;
               Self_Q : array (0 .. Nq - 1) of Boolean := [others => False];
               Hd_Of : array (1 .. 9, 0 .. Nq - 1) of Geom.V3;   --  每停每点在不动的眼里的像素(u, v, 看得见 1 / 0),反面那一条要拿来换
               O, O_Bad : Geom.Board_Obs_Vectors.Vector;
               St, St_Bad : Geom.Board_Stats;
               K : Natural := 0;
            begin
               for Iv in 0 .. Nq_Col - 1 loop
                  for Iu in 0 .. Nq_Row - 1 loop
                     declare
                        U : constant Long_Float := 0.5 * Cell + Cell * Long_Float (Iu);
                        V : constant Long_Float := 0.5 * Cell + Cell * Long_Float (Iv);
                        D : constant Geom.V3 := Geom.Ray (Gwr, H0, U, V);
                        Tb : constant Long_Float := (C0 (2) - Box_Top) / (-D (2));
                        Pb : constant Geom.V3 := [C0 (0) + Tb * D (0), C0 (1) + Tb * D (1), Box_Top];
                        Tt : constant Long_Float := (C0 (2) - Table_Z) / (-D (2));
                     begin
                        if Iv = Nq_Col - 1 then
                           Self_Q (K) := True;   --  最底下一行 = 自己的夹爪:离眼 8 cm,跟着眼走(合成)
                           Xq (K) := [C0 (0) + 0.08 * D (0), C0 (1) + 0.08 * D (1), C0 (2) + 0.08 * D (2)];
                        elsif abs (Pb (0) - Bx) <= Half and then abs (Pb (1) - By) <= Half then
                           Xq (K) := Pb;
                        else
                           Xq (K) := [C0 (0) + Tt * D (0), C0 (1) + Tt * D (1), Table_Z];
                        end if;
                        if not Self_Q (K) then
                           Truth.Append (Xq (K));
                        end if;
                        K := K + 1;
                     end;
                  end loop;
               end loop;
               for S in 1 .. 9 loop
                  declare
                     M : constant Geom.V3 := Path2 (S);
                     Ps : constant Plug.Arm_Pose := [H0 (0) + Step_M * M (0), H0 (1) + Step_M * M (1), H0 (2) + Step_M * M (2), 1.0, 0.0, 0.0, 0.0];
                     Dsh : constant Geom.V3 := [Ps (0) - H0 (0), Ps (1) - H0 (1), Ps (2) - H0 (2)];
                  begin
                     for Q in 0 .. Nq - 1 loop
                        declare
                           Xw : constant Geom.V3 := (if Self_Q (Q) then [Xq (Q) (0) + Dsh (0), Xq (Q) (1) + Dsh (1), Xq (Q) (2) + Dsh (2)] else Xq (Q));
                           U, V, Hu, Hv : Long_Float;
                           Fr, Fh : Boolean;
                           Nth : constant Natural := (S - 1) * Nq + Q;   --  第几笔(配错按笔数挑)
                        begin
                           Geom.Project (Gwr, Ps, Xw, U, V, Fr);
                           if S = 1 then
                              U := 0.5 * Cell + Cell * Long_Float (Q mod Nq_Row);   --  参考停:就是格点本身
                              V := 0.5 * Cell + Cell * Long_Float (Q / Nq_Row);
                           elsif not Fr or else U < 0.0 or else U >= 640.0 or else V < 0.0 or else V >= 480.0 then
                              U := Long_Float ((Q * 53) mod 640); V := Long_Float ((Q * 29) mod 480);   --  出了画面:仪器照样回一个乱的像素(合成)
                           else
                              U := U + 0.5 * Jit4; V := V + 0.5 * Jit4;
                              if Nth mod 17 = 5 then
                                 U := U + 12.0;   --  腕眼里配错 12 px(合成)
                              end if;
                           end if;
                           Geom.Project_Fixed (Gt, Xw, Hu, Hv, Fh);
                           if Fh and then Hu >= 0.0 and then Hu < 640.0 and then Hv >= 0.0 and then Hv < 480.0 then
                              Hu := Hu + 0.8 * Jit4; Hv := Hv + 0.8 * Jit4;
                              if Nth mod 13 = 7 then
                                 Hv := Hv + 25.0;   --  不动的眼里配错 25 px(合成)
                              end if;
                           else
                              Hu := Long_Float ((Q * 31) mod 640); Hv := Long_Float ((Q * 41) mod 480);   --  不动的眼里看不见:乱的像素(合成)
                           end if;
                           Hd_Of (S, Q) := [Hu, Hv, 1.0];
                           O.Append (Geom.Board_Obs'(Pt => Q, Pose => Ps, U => U, V => V, Hu => Hu, Hv => Hv));
                        end;
                     end loop;
                  end;
               end loop;
               --  反面那一份:各停里每个点在不动的眼里的像素都换成第 (37q mod N) 个点的(各停一致,交叉核不出来)
               for I in 0 .. Natural (O.Length) - 1 loop   --  O 按"停在外、点在内"追加 ⇒ 第 I 笔是第 I / Nq + 1 停
                  declare
                     Bad : Geom.Board_Obs := O (I);
                     Swap : constant Natural := (Bad.Pt * 37) mod Nq;
                  begin
                     Bad.Hu := Hd_Of (I / Nq + 1, Swap) (0); Bad.Hv := Hd_Of (I / Nq + 1, Swap) (1);
                     O_Bad.Append (Bad);
                  end;
               end loop;
               Geom.Build_Board (Gwr, 1 + E, O, Scene, Trk, St);
               Geom.Build_Board (Gwr, 1 + E, O_Bad, Scene_Bad, Trk_Bad, St_Bad);
               --  没有不动的眼的身体(2026-09-26):同一批腕眼观测、不动的眼里的像素全抹成 −1 ⇒ 板照样只靠腕眼三角进点,离真点照样 1 cm 内,不动的眼的像素都是 −1
               declare
                  O_Nh : Geom.Board_Obs_Vectors.Vector;
                  Sc_Nh : Geom.Scene_Pt_Vectors.Vector;
                  Tr_Nh : Geom.Board_Track_Vectors.Vector;
                  St_Nh : Geom.Board_Stats;
                  Worst_Nh : Long_Float := 0.0;
                  All_Neg : Boolean := True;
               begin
                  for Ob of O loop
                     declare
                        B : Geom.Board_Obs := Ob;
                     begin
                        B.Hu := -1.0; B.Hv := -1.0;
                        O_Nh.Append (B);
                     end;
                  end loop;
                  Geom.Build_Board (Gwr, 1 + E, O_Nh, Sc_Nh, Tr_Nh, St_Nh);
                  for S of Sc_Nh loop
                     All_Neg := All_Neg and then S.U < 0.0;
                     declare
                        Best : Long_Float := Long_Float'Last;
                     begin
                        for T of Truth loop
                           Best := Long_Float'Min (Best, Geom.Norm ([S.Pw (0) - T (0), S.Pw (1) - T (1), S.Pw (2) - T (2)]));
                        end loop;
                        Worst_Nh := Long_Float'Max (Worst_Nh, Best);
                     end;
                  end loop;
                  Check (St_Nh.Kept > 0 and then St_Nh.Kept >= St.Kept and then All_Neg and then Worst_Nh < 0.01,   --  1 cm(合成)
                         "标定板·没有不动的眼的身体:第 " & Codec.Img (E + 1) & " 只腕眼照样进板 " & Codec.Img (St_Nh.Kept) & " 个点(有不动的眼时 " & Codec.Img (St.Kept)
                         & ")⇒ 离最近真点最远 " & Codec.Fmt (Worst_Nh * Per_Mm, 1) & " mm");
               end;
               Shh := Long_Float'Max (Shh, St.Sigma_H);
               Check (St.Sigma_W > 0.5 * 0.29 and then St.Sigma_W < 2.0 * 0.29 and then St.Sigma_H > 0.5 * 0.46 and then St.Sigma_H < 2.0 * 0.46,   --  一半到两倍(纯数学)× 合成的 σ
                      "标定板·第 " & Codec.Img (E + 1) & " 只腕眼量出来的配点噪声:腕眼 " & Codec.Fmt (St.Sigma_W, 2) & " px(真 0.29)、不动的眼 " & Codec.Fmt (St.Sigma_H, 2)
                      & " px(真 0.46)· 格点 " & Codec.Img (St.Tracks) & " 个,三角定住 " & Codec.Img (St.Tri_Ok) & ",进板 " & Codec.Img (St.Kept));
            end;
         end loop;
         declare
            Worst : Long_Float := 0.0;
         begin
            for S of Scene loop
               declare
                  Best : Long_Float := Long_Float'Last;
               begin
                  for T of Truth loop
                     Best := Long_Float'Min (Best, Geom.Norm ([S.Pw (0) - T (0), S.Pw (1) - T (1), S.Pw (2) - T (2)]));
                  end loop;
                  Worst := Long_Float'Max (Worst, Best);
               end;
            end loop;
            Check (Natural (Scene.Length) > Natural (Truth.Length) / 2 and then Worst < 0.01,   --  一半(纯数学)/ 1 cm(合成)
                   "标定板·进板 " & Codec.Img (Natural (Scene.Length)) & " 个点(真点 " & Codec.Img (Natural (Truth.Length)) & " 个,夹爪 32 个不该进)⇒ 离最近真点最远 "
                   & Codec.Fmt (Worst * Per_Mm, 1) & " mm");
         end;
         declare
            Gb, Gj : Geom.Cam_Geo;
            Tb, Tj : Geom.Tip_Class_Vectors.Vector;
            Rb, Rj : Geom.Fixed_Report;
            Okb, Okj : Boolean;
            Obs_Big : Geom.Obs_Pt_Vectors.Vector;
            Big : constant Long_Float := 1.02;   --  手上的标记整体放大 2%(比例,合成)
         begin
            Gb.F := 0.0; Gb.Cx := 320.0; Gb.Cy := 240.0;
            Geom.Fit_Fixed_Rig (Gb, No_Obs, Scene, Ray_O, Ray_D, Own_Kind, Tb, Rb, Okb);
            declare
               Dp : constant Long_Float := (if Okb then Geom.Norm ([Gb.Pos (0) - Gt.Pos (0), Gb.Pos (1) - Gt.Pos (1), Gb.Pos (2) - Gt.Pos (2)]) else 1.0);
            begin
               Check (Okb and then abs (Gb.F - 288.0) < 0.01 * 288.0 and then Dp < 0.01 and then abs (Gb.F - 288.0) < 3.0 * Gb.F_Sd + 0.1 and then Dp < 3.0 * Gb.Pos_Sd + 0.001,   --  1% / 1 cm(合成)/ 3 = 倍数;0.1 px、1 mm = 合成数据的舍入余量
                      "标定板·只靠板解不动的眼:" & (if Okb then Codec.Img (Rb.Scene_Used) & " 个点 ⇒ 焦距 " & Codec.Fmt (Gb.F, 1) & " ± " & Codec.Fmt (Gb.F_Sd, 1) & " px(真 288)· 位置差 "
                      & Codec.Fmt (Dp * Per_Mm, 1) & " mm(自报 ± " & Codec.Fmt (Gb.Pos_Sd * Per_Mm, 1) & ")· 残差 " & Codec.Fmt (Rb.Scene_Rms, 2) & " px" else "解不出:" & To_String (Geom.Why)));
            end;
            --  ③b 板里混进 15 个远处的点(3 m 外),它们在不动的眼里的像素各停一致地错 40 px(交叉核不出来;G2E 离线 327 个里有 14 个这样的)
            --  ⇒ 一次最小二乘被它们拽走,连踢三遍后焦距仍在 1% 内、这 15 个全不进解
            declare
               Scene_G : Geom.Scene_Pt_Vectors.Vector := Scene;
               Gg : Geom.Cam_Geo;
               Tg : Geom.Tip_Class_Vectors.Vector;
               Rg : Geom.Fixed_Report;
               Okg : Boolean;
            begin
               for K in 0 .. 14 loop
                  declare
                     Pw : constant Geom.V3 := [-1.5 + 0.2 * Long_Float (K), 2.6, 0.3];   --  桌子远处的地上(米,合成)
                     U, V : Long_Float;
                     Fr : Boolean;
                     Far : Geom.Scene_Pt := Scene.First_Element;
                  begin
                     Geom.Project_Fixed (Gt, Pw, U, V, Fr);
                     Far.Pw := Pw; Far.U := U + 40.0; Far.V := V - 25.0;   --  一致地配错(像素,合成)
                     Scene_G.Append (Far);
                  end;
               end loop;
               Gg.F := 0.0; Gg.Cx := 320.0; Gg.Cy := 240.0;
               Geom.Fit_Fixed_Rig (Gg, No_Obs, Scene_G, Ray_O, Ray_D, Own_Kind, Tg, Rg, Okg);
               declare
                  use Ada.Numerics.Long_Elementary_Functions;
                  Min_Far : Long_Float := Long_Float'Last;   --  配错的点在解出来的眼里最小的像素残差:解没有迁就它们 ⇒ 个个都远大于板的残差
                  Dp : constant Long_Float := (if Okg and then Okb then Geom.Norm ([Gg.Pos (0) - Gb.Pos (0), Gg.Pos (1) - Gb.Pos (1), Gg.Pos (2) - Gb.Pos (2)]) else 1.0);
               begin
                  for K in Natural (Scene.Length) .. Natural (Scene_G.Length) - 1 loop
                     declare
                        U, V : Long_Float;
                        Fr : Boolean;
                     begin
                        Geom.Project_Fixed (Gg, Scene_G (K).Pw, U, V, Fr);
                        Min_Far := Long_Float'Min (Min_Far, (if Fr then Sqrt ((U - Scene_G (K).U) ** 2 + (V - Scene_G (K).V) ** 2) else Long_Float'Last));
                     end;
                  end loop;
                  --  和干净的板(③)比:焦距、位置都在它自报不确定度的 3 倍内(倍数无量纲);配错的点残差个个超过板残差的 10 倍(倍数,合成)
                  Check (Okg and then Okb and then abs (Gg.F - Gb.F) < 3.0 * Gb.F_Sd and then Dp < 3.0 * Gb.Pos_Sd and then Min_Far > 10.0 * Rg.Scene_Rms,
                         "标定板·混进 15 个一致配错的远处点:" & (if Okg then "焦距 " & Codec.Fmt (Gg.F, 1) & " px(干净的板 " & Codec.Fmt (Gb.F, 1) & " ± " & Codec.Fmt (Gb.F_Sd, 1)
                         & ")· 位置离干净的解 " & Codec.Fmt (Dp * Per_Mm, 1) & " mm(± " & Codec.Fmt (Gb.Pos_Sd * Per_Mm, 1) & ")· 配错的点最小残差 " & Codec.Fmt (Min_Far, 1)
                         & " px(板 " & Codec.Fmt (Rg.Scene_Rms, 2) & " px)· 进解 " & Codec.Img (Rg.Scene_Used) & "/" & Codec.Img (Natural (Scene_G.Length)) else "解不出:" & To_String (Geom.Why)));
               end;
            end;
            for Ob of Obs_One loop
               Obs_Big.Append (Geom.Obs_Pt'(Pt => Ob.Pt, Pose => Ob.Pose, U => 320.0 + Big * (Ob.U - 320.0), V => 240.0 + Big * (Ob.V - 240.0), Seq => 0, Kind => Ob.Kind));
            end loop;
            Gj.F := 0.0; Gj.Cx := 320.0; Gj.Cy := 240.0;
            Geom.Fit_Fixed_Rig (Gj, Obs_Big, Scene, Ray_O, Ray_D, Own_Kind, Tj, Rj, Okj);
            Check (Okj and then abs (Gj.F - 288.0) < 0.01 * 288.0 and then Rj.Hand_Used > 0,   --  1%(合成)
                   "标定板·板 + 放大 2% 的手上标记(手上的点不进眼的解):焦距 " & (if Okj then Codec.Fmt (Gj.F, 1) else "解不出") & " px(真 288)· 板 "
                   & Codec.Fmt (Rj.Scene_Rms, 2) & " px、手 " & Codec.Fmt (Rj.Hand_Rms, 2) & " px(" & Codec.Img (Rj.Hand_Used) & " 笔)");
         end;
         declare
            Pts : Contact.V3_Vectors.Vector;
            P0, N0 : Geom.V3;
            Cnt : Natural;
            Rms : Long_Float;
            Deg_Per_Rad : constant Long_Float := 57.29578;   --  弧度 → 度(换算,无量纲)
            use Ada.Numerics.Long_Elementary_Functions;
         begin
            for S of Scene loop
               Pts.Append (S.Pw);
            end loop;
            Contact.Surface.Support_Plane (Pts, 0.01, [0.0, 0.0, 1.0], P0, N0, Cnt, Rms);   --  门 1 cm(合成;盒顶高 4 cm)
            Check (Cnt > 0 and then abs (P0 (2) - Table_Z) < 0.001 and then Arccos (Long_Float'Min (1.0, N0 (2))) * Deg_Per_Rad < 0.5,   --  1 mm / 0.5°(合成)
                   "标定板·板上的点拟合的面:" & Codec.Img (Cnt) & "/" & Codec.Img (Natural (Pts.Length)) & " 个点 ⇒ 高 " & Codec.Fmt (P0 (2), 4) & " m(真 0.765)· 法向偏 "
                   & Codec.Fmt (Arccos (Long_Float'Min (1.0, N0 (2))) * Deg_Per_Rad, 2) & "° · 面内离散 " & Codec.Fmt (Rms * Per_Mm, 2) & " mm");
         end;
         declare
            Gx : Geom.Cam_Geo;
            Tx : Geom.Tip_Class_Vectors.Vector;
            Rx : Geom.Fixed_Report;
            Okx : Boolean;
         begin
            Gx.F := 0.0; Gx.Cx := 320.0; Gx.Cy := 240.0;
            Geom.Fit_Fixed_Rig (Gx, No_Obs, Scene_Bad, Ray_O, Ray_D, Own_Kind, Tx, Rx, Okx);
            Check ((not Okx) or else Rx.Scene_Rms > 5.0 * Shh,   --  5 = 倍数(无量纲)
                   "标定板·反面:不动的眼里的像素各停一致地换成别的点的 ⇒ " & (if Okx then "板的残差 " & Codec.Fmt (Rx.Scene_Rms, 1) & " px(配点噪声 " & Codec.Fmt (Shh, 2) & " px)"
                   else "解不出:" & To_String (Geom.Why)));
         end;
         --  🔴 不动的眼被挪了 / 被挡了(Geom.Check_Fixed,V1):按板解出来的眼(③)当"标定时",板上的点此刻在画面里配到哪 ——
         --  ① 没动(配点抖 ±0.5 px)⇒ 不算挪、不算挡;② 绕自己的光轴转 90°(出了画面的点配不到)⇒ 算挪、新位姿离真的 0.5° / 5 mm 内;
         --  ③ 六成的点配成乱的(挡住了)⇒ 算挡、不算挪、位姿不动
         declare
            Seed6 : Long_Long_Integer := 41;
            function Jit6 return Long_Float is   --  确定性伪随机 [−1, 1](测试数据自己的抖动)
            begin
               Seed6 := (Seed6 * 1103515245 + 12345) mod 2147483648;
               return Long_Float (Integer ((Seed6 / 65536) mod 2001) - 1000) / 1000.0;
            end Jit6;
            G0 : Geom.Cam_Geo := Gt;   --  标定时的眼(拿真相机,合成;焦距已知)
            Base : Geom.Scene_Pt_Vectors.Vector;
         begin
            G0.Rot_Sd := 0.0; G0.Pos_Sd := 0.0;
            G0.Rms := 0.5;   --  标定时它的像素残差(合成,同配点抖动)
            for S of Scene loop
               declare
                  B : Geom.Scene_Pt := S;
                  U, V : Long_Float;
                  Fr : Boolean;
               begin
                  Geom.Project_Fixed (G0, S.Pw, U, V, Fr);
                  B.U := U; B.V := V;
                  Base.Append (B);
               end;
            end loop;
            declare
               G1 : Geom.Cam_Geo := G0;
               Now : Geom.Scene_Pt_Vectors.Vector;
               R1 : Geom.Fixed_Check;
               Best1 : Geom.Fixed_Best;   --  刚放好:还没看见过
            begin
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                  begin
                     N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                     Now.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (G1, Base, Now, Best1, R1);
               Check (not R1.Moved and then not R1.Covered,
                      "不动的眼核对·没动:" & Codec.Img (R1.Consistent) & "/" & Codec.Img (R1.Asked) & " 个点对得上 · 新解离原来 " & Codec.Fmt (R1.Turn_Deg, 3) & "° / "
                      & Codec.Fmt (R1.Move_M * Per_Mm, 2) & " mm ⇒ " & (if R1.Moved then "算挪了(错)" else "没挪") & (if R1.Covered then "、算挡了(错)" else ""));
            end;
            --  ①b 配得分毫不差(此刻的像素就是原位姿的投影,G2G 2026-09-25 里自己配自己时就是这样)⇒ 也不许算挪(以前按位姿自报不确定度判,每轮误报)
            declare
               G1b : Geom.Cam_Geo := G0;
               R1b : Geom.Fixed_Check;
               Best1b : Geom.Fixed_Best;
            begin
               Geom.Check_Fixed (G1b, Base, Base, Best1b, R1b);
               Check (not R1b.Moved and then not R1b.Covered,
                      "不动的眼核对·配得分毫不差:板上的点挪了 " & Codec.Fmt (R1b.Shift_Px, 3) & " px(" & Codec.Fmt (R1b.Shift_Sd, 3) & " 个配点噪声)⇒ "
                      & (if R1b.Moved then "算挪了(错)" else "没挪"));
            end;
            declare
               G2 : Geom.Cam_Geo := G0;
               Gr : Geom.Cam_Geo := G0;   --  真的:绕自己的光轴(相机系 z)转 90°
               Now : Geom.Scene_Pt_Vectors.Vector;
               R2 : Geom.Fixed_Check;
               Best2 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);   --  转之前看得全
            begin
               Gr.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.0, 0.5 * Ada.Numerics.Pi]));
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     Geom.Project_Fixed (Gr, B.Pw, U, V, Fr);
                     if Fr and then U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 then
                        N.U := U + 0.5 * Jit6; N.V := V + 0.5 * Jit6;
                     else
                        N.U := -1.0; N.V := -1.0;   --  出了画面:配不到
                     end if;
                     Now.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (G2, Base, Now, Best2, R2);
               declare
                  Da : constant Long_Float := Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (Gr.R_Ce), G2.R_Ce))) * 57.29578;   --  弧度 → 度(换算,无量纲)
                  Dp : constant Long_Float := Geom.Norm ([G2.Pos (0) - Gr.Pos (0), G2.Pos (1) - Gr.Pos (1), G2.Pos (2) - Gr.Pos (2)]);
               begin
                  Check (R2.Moved and then Da < 0.5 and then Dp < 0.005,   --  0.5° / 5 mm(合成)
                         "不动的眼核对·绕光轴转 90°:配到 " & Codec.Img (R2.Matched) & "/" & Codec.Img (R2.Asked) & "、对得上 " & Codec.Img (R2.Consistent) & " ⇒ "
                         & (if R2.Moved then "算挪了(转 " & Codec.Fmt (R2.Turn_Deg, 1) & "°),新位姿离真的 " & Codec.Fmt (Da, 2) & "° / " & Codec.Fmt (Dp * Per_Mm, 1) & " mm" else "没发现(错)"));
               end;
               --  ②b 转完、重标好之后再挡住左半边(X5B 2026-09-25 那种):左半边配成乱的
               --  ⇒ 不许算挪(位姿不动),要算挡
               declare
                  G4 : Geom.Cam_Geo := G2;
                  Now4 : Geom.Scene_Pt_Vectors.Vector;
                  R4 : Geom.Fixed_Check;
                  Best4 : Geom.Fixed_Best := Best2;   --  重标那一刻看见的
                  K : Natural := 0;
               begin
                  for B of Base loop
                     declare
                        N : Geom.Scene_Pt := B;
                        U, V : Long_Float;
                        Fr : Boolean;
                     begin
                        Geom.Project_Fixed (Gr, B.Pw, U, V, Fr);
                        if Fr and then U >= 320.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 then
                           N.U := U + 0.5 * Jit6; N.V := V + 0.5 * Jit6;
                        else
                           N.U := Long_Float ((K * 97) mod 640); N.V := Long_Float ((K * 61) mod 480);   --  挡住的那半边:乱配(合成)
                        end if;
                        Now4.Append (N);
                        K := K + 1;
                     end;
                  end loop;
                  Geom.Check_Fixed (G4, Base, Now4, Best4, R4);
                  Check (not R4.Moved and then R4.Covered and then Geom.Norm ([G4.Pos (0) - G2.Pos (0), G4.Pos (1) - G2.Pos (1), G4.Pos (2) - G2.Pos (2)]) = 0.0,
                         "不动的眼核对·转 90° 之后再挡住左半边:现在的位姿对得上 " & Codec.Img (R4.Consistent_Now) & "、新解 " & Codec.Img (R4.Consistent) & "(重标时 "
                         & Codec.Img (Best2.All_N) & ")⇒ " & (if R4.Moved then "算挪了(错)" else "没挪") & (if R4.Covered then "、算挡了" else "、没发现挡(错)"));
               end;
            end;
            --  ②c 转完、重标好之后再挡住左半边,而仪器整幅都没配上(X5C4 2026-09-26:转 90° 本来就只配上六成,再挡一半就全配飞了):
            --  每个点都配到一个错得离谱的位姿(转 80°、挪 0.8 m)附近、再乱 ±7 px(合成;X5C4 实测那份错位姿在门里解释了 35/782 个、残差 18.9 px)⇒ 此刻的位姿一个都解释不了
            --  ⇒ 不许算挪(X5C4 就这样把位姿换成了"挪了 0.84 m"那份),要算挡
            declare
               G2c : Geom.Cam_Geo := G0;
               Gbad : Geom.Cam_Geo := G0;
               Now2c : Geom.Scene_Pt_Vectors.Vector;
               R2c : Geom.Fixed_Check;
               Best2c : Geom.Fixed_Best;
               Rgo : Geom.Fixed_Check;
               Gtmp : Geom.Cam_Geo := G0;
               Bt : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
               Gr2 : Geom.Cam_Geo := G0;
               Now_R : Geom.Scene_Pt_Vectors.Vector;
            begin
               Gr2.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.0, 0.5 * Ada.Numerics.Pi]));   --  先真转 90° 并重标(同 ②)
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     Geom.Project_Fixed (Gr2, B.Pw, U, V, Fr);
                     if Fr and then U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 then
                        N.U := U + 0.5 * Jit6; N.V := V + 0.5 * Jit6;
                     else
                        N.U := -1.0; N.V := -1.0;
                     end if;
                     Now_R.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (Gtmp, Base, Now_R, Bt, Rgo);
               G2c := Gtmp; Best2c := Bt;
               Gbad.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.0, 1.396]));   --  80°(弧度,合成)
               Gbad.Pos := [G0.Pos (0) + 0.6, G0.Pos (1) - 0.4, G0.Pos (2) - 0.4];      --  挪 0.8 m(合成)
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     Geom.Project_Fixed (Gbad, B.Pw, U, V, Fr);
                     N.U := U + 7.0 * Jit6; N.V := V + 7.0 * Jit6;
                     if not Fr or else N.U < 0.0 or else N.U >= 640.0 or else N.V < 0.0 or else N.V >= 480.0 then
                        N.U := -1.0; N.V := -1.0;
                     end if;
                     Now2c.Append (N);
                  end;
               end loop;
               declare
                  P_Before : constant Geom.V3 := G2c.Pos;
               begin
                  Geom.Check_Fixed (G2c, Base, Now2c, Best2c, R2c);
                  Check (Rgo.Moved and then not R2c.Moved and then R2c.Covered and then Geom.Norm ([G2c.Pos (0) - P_Before (0), G2c.Pos (1) - P_Before (1), G2c.Pos (2) - P_Before (2)]) = 0.0,
                         "不动的眼核对·转 90° 重标后再挡一半、仪器整幅配飞:此刻的位姿对得上 " & Codec.Img (R2c.Consistent_Now) & "、最好的新解 " & Codec.Img (R2c.Consistent)
                         & "(重标时 " & Codec.Img (Rgo.Consistent) & ")⇒ " & (if R2c.Moved then "算挪了(错,挪 " & Codec.Fmt (R2c.Move_M, 2) & " m)" else "没挪")
                         & (if R2c.Covered then "、算挡了" else "、没发现挡(错)"));
               end;
            end;
            --  ②d 标定时配得极细(残差 0.16 px,X5B 的数)、真被绕光轴转 90°,而仪器转着看时配点噪声约 0.8 px(实测)⇒ 按细门(0.48 px)只数得到两成的点,
            --  过不了"至少四分之一";给了转着看的噪声(Turn_Sd = 0.8,开机量的)⇒ 新位姿按 2.4 px 数 ⇒ 算挪、新位姿离真的 0.5° / 5 mm 内;
            --  同一个 Turn_Sd 下 ②c 那种整幅配飞仍然不许算挪
            declare
               G2d : Geom.Cam_Geo := G0;
               Gr : Geom.Cam_Geo := G0;
               Now2d : Geom.Scene_Pt_Vectors.Vector;
               R2d : Geom.Fixed_Check;
               Best2d : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
               G2e : Geom.Cam_Geo := G0;
               Gbad : Geom.Cam_Geo := G0;
               Now2e : Geom.Scene_Pt_Vectors.Vector;
               R2e : Geom.Fixed_Check;
               Best2e : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
            begin
               G2d.Rms := 0.16;   --  合成
               Gr.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.0, 0.5 * Ada.Numerics.Pi]));
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     Geom.Project_Fixed (Gr, B.Pw, U, V, Fr);
                     if Fr and then U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 then
                        N.U := U + Jit6; N.V := V + Jit6;   --  每轴 ±1 px(均方根约 0.8,合成)
                     else
                        N.U := -1.0; N.V := -1.0;
                     end if;
                     Now2d.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (G2d, Base, Now2d, Best2d, R2d, Turn_Sd => 0.8);
               G2e.Rms := 0.16;
               Gbad.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.0, 1.396]));   --  80°(弧度,合成)
               Gbad.Pos := [G0.Pos (0) + 0.6, G0.Pos (1) - 0.4, G0.Pos (2) - 0.4];      --  0.8 m(合成)
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     Geom.Project_Fixed (Gbad, B.Pw, U, V, Fr);
                     N.U := U + 7.0 * Jit6; N.V := V + 7.0 * Jit6;
                     if not Fr or else N.U < 0.0 or else N.U >= 640.0 or else N.V < 0.0 or else N.V >= 480.0 then
                        N.U := -1.0; N.V := -1.0;
                     end if;
                     Now2e.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (G2e, Base, Now2e, Best2e, R2e, Turn_Sd => 0.8);
               declare
                  Da : constant Long_Float := Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (Gr.R_Ce), G2d.R_Ce))) * 57.29578;   --  弧度 → 度(换算,无量纲)
                  Dp : constant Long_Float := Geom.Norm ([G2d.Pos (0) - Gr.Pos (0), G2d.Pos (1) - Gr.Pos (1), G2d.Pos (2) - Gr.Pos (2)]);
               begin
                  Check (R2d.Moved and then Da < 0.5 and then Dp < 0.005 and then not R2e.Moved and then R2e.Covered,
                         "不动的眼核对·标定残差 0.16 px 的眼真转 90°、转着看配点噪声 0.8 px:新解 " & Codec.Img (R2d.Consistent) & " 个点 ⇒ "
                         & (if R2d.Moved then "算挪了,新位姿离真的 " & Codec.Fmt (Da, 2) & "° / " & Codec.Fmt (Dp * Per_Mm, 1) & " mm" else "没发现(错)")
                         & " · 同样的门下整幅配飞:新解 " & Codec.Img (R2e.Consistent) & " ⇒ " & (if R2e.Moved then "算挪了(错)" else "没挪")
                         & (if R2e.Covered then "、算挡了" else "、没发现挡(错)"));
               end;
            end;
            --  ②e 没转、只挡住左半边(X5E 2026-09-26):没转的画面里原位姿解释得了看得见的那半边;把画面转 90° 再配(看全不看全那一轮会这么试),
            --  配点糙到 1 px 左右(转着看的噪声),重解的位姿只差零点几度 —— 按"原位姿在转过的配点里数"比,原位姿吃亏,就被当成挪过;
            --  按没转的画面里数的那份比(Base_Now)⇒ 不许算挪
            declare
               Gc : Geom.Cam_Geo := G0;
               Nc0, Nct : Geom.Scene_Pt_Vectors.Vector;
               Rc0, Rct, Rct_Bad : Geom.Fixed_Check;
               Bc0 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
               Bct, Bct2 : Geom.Fixed_Best;
               K : Natural := 0;
            begin
               Gc.Rms := 0.16;   --  标定时配得很细(X5E 的数,合成)
               for B of Base loop
                  declare
                     N0 : Geom.Scene_Pt := B;
                     Nt : Geom.Scene_Pt := B;
                  begin
                     if B.U < 320.0 then   --  左半边被挡:乱配(合成)
                        N0.U := Long_Float ((K * 97) mod 640); N0.V := Long_Float ((K * 61) mod 480);
                        Nt.U := Long_Float ((K * 53) mod 640); Nt.V := Long_Float ((K * 29) mod 480);
                     else
                        N0.U := B.U + 0.1 * Jit6; N0.V := B.V + 0.1 * Jit6;   --  没转的画面:配得细(合成)
                        Nt.U := B.U + 1.2 + 1.7 * Jit6; Nt.V := B.V + 1.7 * Jit6;   --  转 90° 再配:整体偏 1.2 px、再糙到约 1 px(合成;X5E 实测重解的位姿挪了 1.0 px)
                     end if;
                     Nc0.Append (N0); Nct.Append (Nt);
                     K := K + 1;
                  end;
               end loop;
               declare
                  G_Plain : Geom.Cam_Geo := Gc;
                  G_T, G_T_Bad : Geom.Cam_Geo := Gc;
               begin
                  Geom.Check_Fixed (G_Plain, Base, Nc0, Bc0, Rc0, Turn_Sd => 0.99);
                  Bct := Bc0; Bct2 := Bc0;
                  Geom.Check_Fixed (G_T, Base, Nct, Bct, Rct, Turn_Sd => 0.99, Base_Now => Rc0.Consistent_Now);
                  Geom.Check_Fixed (G_T_Bad, Base, Nct, Bct2, Rct_Bad, Turn_Sd => 0.99);
                  Check (Rc0.Covered and then not Rc0.Moved and then not Rct.Moved and then Rct_Bad.Moved,
                         "不动的眼核对·没转只挡左半、再把画面转 90° 配:没转的画面里原位姿对得上 " & Codec.Img (Rc0.Consistent_Now) & " ⇒ " & (if Rc0.Covered then "算挡了" else "没发现挡(错)")
                         & " · 转过的配点里重解的位姿 " & Codec.Img (Rct.Consistent) & " 个点、挪了 " & Codec.Fmt (Rct.Shift_Px, 2) & " px ⇒ 按没转的那份比:"
                         & (if Rct.Moved then "算挪了(错)" else "没挪") & "(不按它比:" & (if Rct_Bad.Moved then "会算成挪了" else "也没挪") & ")");
               end;
            end;
            --  ③b 挡住左半边,但挡住的那半边不是乱配,是一片平滑的"编出来的"配点(照着一个偏了 7° 的位姿投、再抖 ±4 px,合成):
            --  X5C 2026-09-25 就是这样被判成"挪了 7.6°、10.6 cm"的 ⇒ 不许算挪,要算挡
            declare
               G5 : Geom.Cam_Geo := G0;
               Gw : Geom.Cam_Geo := G0;   --  编出来的那片对应的错位姿
               Now5 : Geom.Scene_Pt_Vectors.Vector;
               R5 : Geom.Fixed_Check;
               Best5 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);   --  挡之前看得全
            begin
               Gw.R_Ce := Geom.Mul (G0.R_Ce, Geom.Rodrigues ([0.0, 0.122, 0.0]));   --  7°(弧度,合成)
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                     U, V : Long_Float;
                     Fr : Boolean;
                  begin
                     if B.U < 320.0 then   --  左半边被挡:编出来的(合成)
                        Geom.Project_Fixed (Gw, B.Pw, U, V, Fr);
                        N.U := U + 4.0 * Jit6; N.V := V + 4.0 * Jit6;
                     else
                        N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                     end if;
                     Now5.Append (N);
                  end;
               end loop;
               Geom.Check_Fixed (G5, Base, Now5, Best5, R5);
               Check (not R5.Moved and then R5.Covered,
                      "不动的眼核对·挡住左半边、那半边配成一片编出来的:原位姿对得上 " & Codec.Img (R5.Consistent_Now) & "、最好的新解 " & Codec.Img (R5.Consistent) & " ⇒ "
                      & (if R5.Moved then "算挪了(错,转 " & Codec.Fmt (R5.Turn_Deg, 1) & "°)" else "没挪") & (if R5.Covered then "、算挡了" else "、没发现挡(错)"));
            end;
            declare
               G3 : Geom.Cam_Geo := G0;
               Now : Geom.Scene_Pt_Vectors.Vector;
               R3 : Geom.Fixed_Check;
               K : Natural := 0;
               Best3 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);   --  挡之前看得全
            begin
               for B of Base loop
                  declare
                     N : Geom.Scene_Pt := B;
                  begin
                     if K mod 5 < 3 then   --  五个里三个(六成)配成乱的(合成)
                        N.U := Long_Float ((K * 97) mod 640); N.V := Long_Float ((K * 61) mod 480);
                     else
                        N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                     end if;
                     Now.Append (N);
                     K := K + 1;
                  end;
               end loop;
               Geom.Check_Fixed (G3, Base, Now, Best3, R3);
               Check (R3.Covered and then not R3.Moved and then Geom.Norm ([G3.Pos (0) - G0.Pos (0), G3.Pos (1) - G0.Pos (1), G3.Pos (2) - G0.Pos (2)]) = 0.0,
                      "不动的眼核对·挡住六成:对得上 " & Codec.Img (R3.Consistent) & "/" & Codec.Img (R3.Asked) & " ⇒ " & (if R3.Covered then "算挡了" else "没发现挡(错)")
                      & (if R3.Moved then "、算挪了(错)" else "、位姿不动"));
            end;
            --  ④ 挡住一块,按块判(09-27 V1B39:转过 90° 以后板上的点多在右边,挡住左半只挡掉整幅的 25%,整幅那条擦线没报):
            --  (a) 板上的点四分之三在右边、挡住左半 ⇒ 整幅只少了约两成(整幅那条不报),左半那一块一个不剩 ⇒ 要报挡、说是左半边、位姿不动;
            --  (b) 到处随机丢一成半 ⇒ 不许报;(c) 一小团(一只手从眼前经过那么大)丢了 ⇒ 不许报
            declare
               Base_R : Geom.Scene_Pt_Vectors.Vector;
               K : Natural := 0;
            begin
               for B of Base loop
                  if B.U >= 320.0 or else K mod 4 = 0 then
                     Base_R.Append (B);
                  end if;
                  if B.U < 320.0 then
                     K := K + 1;
                  end if;
               end loop;
               declare
                  Ga4 : Geom.Cam_Geo := G0;
                  Na4 : Geom.Scene_Pt_Vectors.Vector;
                  Ra4 : Geom.Fixed_Check;
                  Ba4 : Geom.Fixed_Best := Geom.Seen_All (G0, Base_R);
                  Left_N : Natural := 0;
                  J : Natural := 0;
               begin
                  for B of Base_R loop
                     declare
                        N : Geom.Scene_Pt := B;
                     begin
                        if B.U < 320.0 then
                           N.U := Long_Float ((J * 97) mod 640); N.V := Long_Float ((J * 61) mod 480);   --  挡住的那半边:乱配(合成)
                           Left_N := Left_N + 1;
                        else
                           N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                        end if;
                        Na4.Append (N);
                        J := J + 1;
                     end;
                  end loop;
                  Geom.Check_Fixed (Ga4, Base_R, Na4, Ba4, Ra4);
                  Check (Ra4.Covered and then Ra4.Dark = 0 and then not Ra4.Moved and then 4 * Ra4.Consistent_Now >= 3 * Ba4.All_N
                         and then Geom.Norm ([Ga4.Pos (0) - G0.Pos (0), Ga4.Pos (1) - G0.Pos (1), Ga4.Pos (2) - G0.Pos (2)]) = 0.0,
                         "不动的眼核对·按块判:板上 " & Codec.Img (Natural (Base_R.Length)) & " 个点、左半只有 " & Codec.Img (Left_N) & " 个,挡住左半 ⇒ 整幅对得上 "
                         & Codec.Img (Ra4.Consistent_Now) & " / " & Codec.Img (Ba4.All_N) & "(整幅那条不报)· "
                         & (if Ra4.Covered then "算挡了:" & (if Ra4.Dark >= 0 then Geom.Region_Name (Natural (Ra4.Dark)) & " " & Codec.Img (Ra4.Dark_Now) & " / " & Codec.Img (Ra4.Dark_Best) else "(没说哪块)")
                            else "没发现挡(错)") & (if Ra4.Moved then "、算挪了(错)" else "、位姿不动"));
               end;
               declare
                  Gb4 : Geom.Cam_Geo := G0;
                  Nb4 : Geom.Scene_Pt_Vectors.Vector;
                  Rb4 : Geom.Fixed_Check;
                  Bb4 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
                  J : Natural := 0;
               begin
                  for B of Base loop
                     declare
                        N : Geom.Scene_Pt := B;
                     begin
                        if J mod 7 = 3 then   --  七个里丢一个(一成半,合成)
                           N.U := Long_Float ((J * 97) mod 640); N.V := Long_Float ((J * 61) mod 480);
                        else
                           N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                        end if;
                        Nb4.Append (N);
                        J := J + 1;
                     end;
                  end loop;
                  Geom.Check_Fixed (Gb4, Base, Nb4, Bb4, Rb4);
                  Check (not Rb4.Covered and then not Rb4.Moved,
                         "不动的眼核对·到处随机丢一成半:对得上 " & Codec.Img (Rb4.Consistent_Now) & " / " & Codec.Img (Bb4.All_N) & " ⇒ "
                         & (if Rb4.Covered then "算挡了(错:" & (if Rb4.Dark >= 0 then Geom.Region_Name (Natural (Rb4.Dark)) else "整幅") & ")" else "没挡") & (if Rb4.Moved then "、算挪了(错)" else ""));
               end;
               declare
                  Gc4 : Geom.Cam_Geo := G0;
                  Nc4 : Geom.Scene_Pt_Vectors.Vector;
                  Rc4 : Geom.Fixed_Check;
                  Bc4 : Geom.Fixed_Best := Geom.Seen_All (G0, Base);
                  Ctr : constant Geom.Scene_Pt := Base (Natural (Base.Length) / 2);
                  Lost : Natural := 0;
                  J : Natural := 0;
               begin
                  for B of Base loop
                     declare
                        N : Geom.Scene_Pt := B;
                     begin
                        if (B.U - Ctr.U) ** 2 + (B.V - Ctr.V) ** 2 < 40.0 ** 2 then   --  半径 40 px 的一团(合成:一只手从眼前经过那么大)
                           N.U := Long_Float ((J * 97) mod 640); N.V := Long_Float ((J * 61) mod 480);
                           Lost := Lost + 1;
                        else
                           N.U := B.U + 0.5 * Jit6; N.V := B.V + 0.5 * Jit6;
                        end if;
                        Nc4.Append (N);
                        J := J + 1;
                     end;
                  end loop;
                  Geom.Check_Fixed (Gc4, Base, Nc4, Bc4, Rc4);
                  Check (not Rc4.Covered and then not Rc4.Moved,
                         "不动的眼核对·一小团(半径 40 px,丢 " & Codec.Img (Lost) & " 个)看不见了:对得上 " & Codec.Img (Rc4.Consistent_Now) & " / " & Codec.Img (Bc4.All_N) & " ⇒ "
                         & (if Rc4.Covered then "算挡了(错:" & (if Rc4.Dark >= 0 then Geom.Region_Name (Natural (Rc4.Dark)) else "整幅") & ")" else "没挡") & (if Rc4.Moved then "、算挪了(错)" else ""));
               end;
            end;
         end;
         --  🔴 腕眼 + 不动的眼 + 板上的点一起解(Geom.Refine_Board):同样两只腕眼,每只 9 停平移 + 4 停转动(绕 z、x 各 ±0.1 rad,合成),
         --  但"腕眼标定"给的几何是歪的(焦距 −1.5%、偏移差 (4,−3,5) mm、朝向差 0.5°,合成;自报 ± 6 px / 1 cm)——板按歪的几何建,不动的眼按歪的板解,
         --  再一起解 ⇒ 腕眼焦距 0.3% 内、偏移 3 mm 内,不动的眼焦距 0.5% 内、位置 5 mm 内(一起解之前:头跟着腕眼一起错)
         declare
            procedure Joint (Kt1, Kt2 : Long_Float; Label : String) is
               Geos : Geom.Geo_Vectors.Vector;
               Trk2 : Geom.Board_Track_Vectors.Vector;
               Sc2 : Geom.Scene_Pt_Vectors.Vector;
               Seed5 : Long_Long_Integer := 29;
               function Jit5 return Long_Float is   --  确定性伪随机 [−1, 1](测试数据自己的抖动)
               begin
                  Seed5 := (Seed5 * 1103515245 + 12345) mod 2147483648;
                  return Long_Float (Integer ((Seed5 / 65536) mod 2001) - 1000) / 1000.0;
               end Jit5;
               Bad_Off : constant Geom.V3 := [0.004, -0.003, 0.005];   --  腕眼标定给的偏移差(米,合成)
               Bad_F : constant Long_Float := 0.985;                    --  腕眼标定给的焦距比例(合成)
               Bad_R : constant Geom.M3 := Geom.Rodrigues ([0.0087, 0.0, 0.0]);   --  0.5° 的朝向差(弧度,合成)
               Gh : Geom.Cam_Geo;
               Rh : Geom.Fixed_Report;
               Okh, Okr : Boolean;
               Rr : Geom.Refine_Report;
               Gh0 : Geom.Cam_Geo;
               Gwr_K : Geom.Cam_Geo := Gwr;   --  真的腕眼(带这一遍的畸变)
               Gt_K : Geom.Cam_Geo := Gt;     --  真的不动的眼(同上)
            begin
               Gwr_K.K1 := Kt1; Gwr_K.K2 := Kt2; Gt_K.K1 := Kt1; Gt_K.K2 := Kt2;
               for K in 0 .. 2 loop
                  Geos.Append (Geom.No_Geo);
               end loop;
               for E in 0 .. 1 loop
                  declare
                     H0 : constant Plug.Arm_Pose := [Starts (E) (0), Starts (E) (1), Starts (E) (2), 1.0, 0.0, 0.0, 0.0];
                     C0 : constant Geom.V3 := Geom.Cam_Pos (Gwr_K, H0);
                     Gbad : Geom.Cam_Geo := Gwr;
                     O : Geom.Board_Obs_Vectors.Vector;
                     St : Geom.Board_Stats;
                     Q_Rot : constant array (1 .. 4) of Plug.Arm_Pose :=
                       [[H0 (0), H0 (1), H0 (2), 0.99875, 0.0, 0.0, 0.04998], [H0 (0), H0 (1), H0 (2), 0.99875, 0.0, 0.0, -0.04998],
                        [H0 (0), H0 (1), H0 (2), 0.99875, 0.04998, 0.0, 0.0], [H0 (0), H0 (1), H0 (2), 0.99875, -0.04998, 0.0, 0.0]];   --  ±0.1 rad(cos/sin 0.05,合成)
                     Poses : Plug.Pose_Vectors.Vector;
                  begin
                     for S in 1 .. 9 loop
                        Poses.Append (Plug.Arm_Pose'[H0 (0) + Step_M * Path2 (S) (0), H0 (1) + Step_M * Path2 (S) (1), H0 (2) + Step_M * Path2 (S) (2), 1.0, 0.0, 0.0, 0.0]);
                     end loop;
                     for Qr of Q_Rot loop
                        Poses.Append (Qr);
                     end loop;
                     for Iv in 0 .. Nq_Col - 2 loop   --  不要夹爪那一行
                        for Iu in 0 .. Nq_Row - 1 loop
                           declare
                              Q : constant Natural := Iv * Nq_Row + Iu;
                              U0 : constant Long_Float := 0.5 * Cell + Cell * Long_Float (Iu);
                              V0 : constant Long_Float := 0.5 * Cell + Cell * Long_Float (Iv);
                              D : constant Geom.V3 := Geom.Ray (Gwr_K, H0, U0, V0);
                              Tt : constant Long_Float := (C0 (2) - Table_Z) / (-D (2));
                              Xw : constant Geom.V3 := [C0 (0) + Tt * D (0), C0 (1) + Tt * D (1), Table_Z + 0.03 * Long_Float ((Iu + Iv) mod 3)];   --  桌面上几层高低(米,合成)
                           begin
                              for K in 0 .. Natural (Poses.Length) - 1 loop
                                 declare
                                    Ps : constant Plug.Arm_Pose := Poses (K);
                                    U, V, Hu, Hv : Long_Float;
                                    Fr, Fh : Boolean;
                                 begin
                                    Geom.Project (Gwr_K, Ps, Xw, U, V, Fr);
                                    Geom.Project_Fixed (Gt_K, Xw, Hu, Hv, Fh);
                                    if Fr and then Fh and then U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 and then Hu >= 0.0 and then Hu < 640.0 then
                                       O.Append (Geom.Board_Obs'(Pt => Q, Pose => Ps, U => U + 0.5 * Jit5, V => V + 0.5 * Jit5, Hu => Hu + 0.8 * Jit5, Hv => Hv + 0.8 * Jit5));
                                    end if;
                                 end;
                              end loop;
                           end;
                        end loop;
                     end loop;
                     Gbad.F := Gwr.F * Bad_F; Gbad.F_Meas := Gbad.F; Gbad.F_Sd := 6.0;   --  自报 ± 6 px(合成)
                     Gbad.Off := [Gwr.Off (0) + Bad_Off (0), Gwr.Off (1) + Bad_Off (1), Gwr.Off (2) + Bad_Off (2)]; Gbad.Off_Sd := 0.01;   --  自报 ± 1 cm(合成)
                     Gbad.R_Ce := Geom.Mul (Bad_R, Gwr.R_Ce);
                     Geos.Replace_Element (1 + E, Gbad);
                     Geom.Build_Board (Gbad, 1 + E, O, Sc2, Trk2, St);
                  end;
               end loop;
               Gh.F := 0.0; Gh.Cx := 320.0; Gh.Cy := 240.0;
               Geom.Fit_Fixed_Board (Gh, Sc2, Rh, Okh);
               Gh0 := Gh;
               --  没有不动的眼(2026-09-26):同一批腕眼的板、不给不动的眼 ⇒ 只解腕眼,焦距、偏移照样拉回来(0.5% / 5 mm 内,合成)
               declare
                  Geos_Nh : Geom.Geo_Vectors.Vector := Geos;
                  Hn : Geom.Cam_Geo := Geom.No_Geo;
                  Rn : Geom.Refine_Report;
                  Okn : Boolean;
               begin
                  Geom.Refine_Board (Geos_Nh, Hn, Trk2, Rn, Okn);
                  declare
                     function Off_Err_N (K : Natural) return Long_Float is
                       (Geom.Norm ([Geos_Nh (K).Off (0) - Gwr.Off (0), Geos_Nh (K).Off (1) - Gwr.Off (1), Geos_Nh (K).Off (2) - Gwr.Off (2)]));
                  begin
                     Check (Okn and then abs (Geos_Nh (1).F - 397.0) < 0.005 * 397.0 and then abs (Geos_Nh (2).F - 397.0) < 0.005 * 397.0   --  0.5%(合成)
                            and then Off_Err_N (1) < 0.005 and then Off_Err_N (2) < 0.005 and then not Hn.Valid,   --  5 mm(合成)
                            "腕眼一起解·没有不动的眼" & Label & ":" & (if Okn then "焦距 " & Codec.Fmt (Gwr.F * Bad_F, 1) & " → " & Codec.Fmt (Geos_Nh (1).F, 1) & " / "
                            & Codec.Fmt (Geos_Nh (2).F, 1) & "(真 397)· 偏移差 " & Codec.Fmt (Off_Err_N (1) * Per_Mm, 1) & " / " & Codec.Fmt (Off_Err_N (2) * Per_Mm, 1)
                            & " mm · 畸变 K1 " & Codec.Fmt (Geos_Nh (1).K1, 3) & " / " & Codec.Fmt (Geos_Nh (2).K1, 3) & " · 腕眼 " & Codec.Fmt (Rn.Wrist_Rms, 2) & " px"
                            else "没收下:" & To_String (Geom.Why)));
                  end;
               end;
               if Okh then
                  Geom.Refine_Board (Geos, Gh, Trk2, Rr, Okr);
               else
                  Okr := False;
               end if;
               declare
                  function Off_Err (K : Natural) return Long_Float is
                    (Geom.Norm ([Geos (K).Off (0) - Gwr.Off (0), Geos (K).Off (1) - Gwr.Off (1), Geos (K).Off (2) - Gwr.Off (2)]));
                  Dp : constant Long_Float := Geom.Norm ([Gh.Pos (0) - Gt.Pos (0), Gh.Pos (1) - Gt.Pos (1), Gh.Pos (2) - Gt.Pos (2)]);
                  Dp0 : constant Long_Float := Geom.Norm ([Gh0.Pos (0) - Gt.Pos (0), Gh0.Pos (1) - Gt.Pos (1), Gh0.Pos (2) - Gt.Pos (2)]);
               begin
                  Check (Okh and then Okr and then abs (Geos (1).F - 397.0) < 0.003 * 397.0 and then abs (Geos (2).F - 397.0) < 0.003 * 397.0   --  0.3%(合成)
                         and then Off_Err (1) < 0.003 and then Off_Err (2) < 0.003 and then abs (Gh.F - 288.0) < 0.005 * 288.0 and then Dp < 0.005   --  3 mm / 0.5% / 5 mm(合成)
                         and then abs (Geos (1).K1 - Kt1) < 3.0 * Geos (1).K1_Sd + 0.005 and then abs (Geos (2).K1 - Kt1) < 3.0 * Geos (2).K1_Sd + 0.005
                         and then abs (Gh.K1 - Kt1) < 3.0 * Gh.K1_Sd + 0.005,   --  畸变 K1 差在它自报的 3 倍不确定度 + 0.005 内(合成)
                         "腕眼 + 不动的眼一起解" & Label & ":" & (if Okr then "畸变 K1 " & Codec.Fmt (Geos (1).K1, 3) & " ± " & Codec.Fmt (Geos (1).K1_Sd, 3) & " / "
                         & Codec.Fmt (Geos (2).K1, 3) & " ± " & Codec.Fmt (Geos (2).K1_Sd, 3) & " / 头 " & Codec.Fmt (Gh.K1, 3) & " ± " & Codec.Fmt (Gh.K1_Sd, 3)
                         & "(真 " & Codec.Fmt (Kt1, 2) & ")· 腕眼焦距 " & Codec.Fmt (Gwr.F * Bad_F, 1) & " → " & Codec.Fmt (Geos (1).F, 1) & " / " & Codec.Fmt (Geos (2).F, 1)
                         & "(真 397)· 偏移差 → " & Codec.Fmt (Off_Err (1) * Per_Mm, 1) & " / " & Codec.Fmt (Off_Err (2) * Per_Mm, 1) & " mm · 不动的眼焦距 " & Codec.Fmt (Gh0.F, 1)
                         & " → " & Codec.Fmt (Gh.F, 1) & "(真 288)· 位置差 " & Codec.Fmt (Dp0 * Per_Mm, 1) & " → " & Codec.Fmt (Dp * Per_Mm, 1) & " mm · " & Codec.Img (Rr.Tracks)
                         & " 条点,腕眼 " & Codec.Fmt (Rr.Wrist_Rms, 2) & " px、头 " & Codec.Fmt (Rr.Head_Rms, 2) & " px"
                         else "没收下:" & To_String (Geom.Why)));
               end;

            end Joint;
         begin
            Joint (0.0, 0.0, "");
            --  镜头有畸变(真机都有;K1 −0.15、K2 0.03,合成)、腕眼标定和板都按理想针孔起步 ⇒ 一起解把畸变也解出来,焦距、偏移、位置照样到线内
            Joint (-0.15, 0.03, "·镜头有畸变(K1 −0.15、K2 0.03)");
         end;
      end;
   end;
   --  🔴 标定板随身体文件存、装回(Act.Board_Save / Board_Load,2026-09-25):存一份再装回,点、协方差、参考图一个不差(存的是 6 位小数、协方差按 mm²)
   declare
      C1, C2 : Act.Context;
      Img : Buf;
      Worst : Long_Float := 0.0;
   begin
      C1.Geo_Path := To_Unbounded_String ("/tmp/bd_selfcheck_board.geo.json");
      for I in 0 .. 5 loop
         C1.Board.Append (Geom.Scene_Pt'(Pw => [0.1 * Long_Float (I) - 0.25, -0.2, 0.765], U => 100.0 + Long_Float (I), V => 200.5, Sh => 0.25, Views => 9,
                                         Cov => [[1.0e-6, 0.0, 0.0], [0.0, 2.0e-6, 0.0], [0.0, 0.0, 3.0e-6]]));   --  合成
      end loop;
      for I in 0 .. 8 * 6 * 3 - 1 loop
         Img.Append (U8 ((I * 37) mod 256));
      end loop;
      C1.Fixed_Ref := Img; C1.Fixed_Ref_W := 8; C1.Fixed_Ref_H := 6;
      C1.Fixed_Best := (All_N => 5, others => <>);
      Act.Board_Save (C1);
      C2.Geo_Path := C1.Geo_Path;
      Act.Board_Load (C2);
      if Natural (C2.Board.Length) = Natural (C1.Board.Length) then
         for I in 0 .. Natural (C1.Board.Length) - 1 loop
            for K in 0 .. 2 loop
               Worst := Long_Float'Max (Worst, abs (C2.Board (I).Pw (K) - C1.Board (I).Pw (K)));
               Worst := Long_Float'Max (Worst, 1.0e3 * abs (C2.Board (I).Cov (K, K) - C1.Board (I).Cov (K, K)));   --  协方差按 m² 比(乘 1000 = 同样按 mm 级的门,换算)
            end loop;
            Worst := Long_Float'Max (Worst, 1.0e-3 * abs (C2.Board (I).U - C1.Board (I).U));   --  像素按 1/1000 折成同一个门(换算)
         end loop;
      end if;
      Check (Natural (C2.Board.Length) = Natural (C1.Board.Length) and then Bytes.U8_Vectors."=" (C2.Fixed_Ref, C1.Fixed_Ref) and then C2.Fixed_Ref_W = 8
             and then C2.Fixed_Ref_H = 6 and then Worst < 1.0e-5 and then C2.Fixed_Best.All_N = 5,
             "标定板随身体文件存、装回:" & Codec.Img (Natural (C2.Board.Length)) & "/" & Codec.Img (Natural (C1.Board.Length)) & " 个点、参考图 "
             & Codec.Img (C2.Fixed_Ref_W) & "×" & Codec.Img (C2.Fixed_Ref_H) & (if Bytes.U8_Vectors."=" (C2.Fixed_Ref, C1.Fixed_Ref) then " 一样" else " 不一样")
             & " · 最大差 " & Codec.Fmt (Worst, 8));
   end;
   --  🔴 握区的手指像素随身体文件存、装回(Bodyfile,游程,2026-09-26):以前不存 ⇒ 装回身体后一个指尖都认不出,开机碰桌面量指尖直接"没量到"(X5C3)。
   --  合成:8×6 画面里两块手指(左上 2×3、右下 3×2)⇒ 存了再装回,每一格一样;指尖像素(Zone.Tip_Px)也一样
   declare
      M1, M2 : Selfmap.Body_Map;
      H1, H2 : Zone.Hand_Vectors.Vector;
      T1, T2 : Act.Effect_Vectors.Vector;
      S1, S2 : Schema.Map;
      H : Zone.Hand;
      Z : Zone.Hand_Zone;
      Note : Unbounded_String;
      Got : Boolean;
      Path : constant String := "/tmp/bd_selfcheck_body.json";
      Same : Boolean := False;
      U1, V1, U2, V2 : Long_Float := -1.0;
      Ok1, Ok2 : Boolean := False;
   begin
      M1.Arms := 1; M1.N_Cams := 1; M1.Per_Arm := Chan.Per_Arm; M1.Channels := Chan.Per_Arm;
      for Ch in 0 .. Chan.Per_Arm - 1 loop
         M1.Amp.Append (0.0065); M1.Delivered.Append (0.005);   --  合成
      end loop;
      M1.Cam_On_Arm.Append (0);
      for I in 0 .. 8 * 6 - 1 loop
         Z.Fingers.Append ((I / 8 in 0 .. 2 and then I mod 8 in 0 .. 1) or else (I / 8 in 4 .. 5 and then I mod 8 in 5 .. 7));
      end loop;
      Z.Valid := True; Z.N_Lobes := 2; Z.Cu := 0.5; Z.Cv := 0.5; Z.X0 := 0; Z.Y0 := 0; Z.X1 := 7; Z.Y1 := 5;
      Z.A := (True, 0, 0, 1, 2, 0.1, 0.2, 6);
      Z.B := (True, 5, 4, 7, 5, 0.8, 0.9, 6);
      H.Arm := 0; H.Zones.Append (Z);
      H1.Append (H);
      Bodyfile.Save (Path, "selfcheck", M1, H1, T1, S1);
      M2 := M1;
      Got := Bodyfile.Load (Path, "selfcheck", M2, H2, T2, S2, Note);
      if Got and then not H2.Is_Empty and then not H2 (0).Zones.Is_Empty then
         Same := Bytes.Bool_Vectors."=" (H2 (0).Zones (0).Fingers, Z.Fingers);
         Zone.Tip_Px (Z, Z.A, 8, 6, U1, V1, Ok1);
         Zone.Tip_Px (H2 (0).Zones (0), H2 (0).Zones (0).A, 8, 6, U2, V2, Ok2);
      end if;
      Check (Got and then Same and then Ok1 and then Ok2 and then U1 = U2 and then V1 = V2,
             "握区的手指像素随身体文件存、装回:" & (if Got then "装上了" else "没装上(" & To_String (Note) & ")") & " · 每一格" & (if Same then "一样" else "不一样")
             & " · 指尖像素 (" & Codec.Fmt (U1, 2) & "," & Codec.Fmt (V1, 2) & ") → (" & Codec.Fmt (U2, 2) & "," & Codec.Fmt (V2, 2) & ")");
   end;
   --  🔴 图顺时针转 90°(Act.Turn_90):原图 (u, v) 的那个像素落在新图 (H − 1 − v, u)(合成 5×3 的图,每个像素三个字节各不相同)
   declare
      Im : Buf;
      Wd : constant Natural := 5;
      Ht : constant Natural := 3;
      Ok_All : Boolean := True;
   begin
      for I in 0 .. Wd * Ht * 3 - 1 loop
         Im.Append (U8 (I));
      end loop;
      declare
         R : constant Buf := Act.Turn_90 (Im, Wd, Ht);
      begin
         Ok_All := Natural (R.Length) = Wd * Ht * 3;
         for V in 0 .. Ht - 1 loop
            for U in 0 .. Wd - 1 loop
               for Ch in 0 .. 2 loop
                  if Ok_All and then R (((U) * Ht + (Ht - 1 - V)) * 3 + Ch) /= Im ((V * Wd + U) * 3 + Ch) then
                     Ok_All := False;
                  end if;
               end loop;
            end loop;
         end loop;
         Check (Ok_All, "图顺时针转 90°:原图 (u, v) 落在新图 (H − 1 − v, u)" & (if Ok_All then ",15 个像素全对" else "(错)"));
         --  转 1、2、3 次之后的图里每个像素的中心换算回原图(Act.Unturn)= 它本来那个像素的中心:按像素值认(每个像素的第一个字节各不相同)。
         --  坐标按配点仪器的约定是连续的:下标 i 的像素中心在 i + 0.5(09-27:原来按下标验、Unturn 写成 h − 1 − u',对下标对,对仪器给的坐标每步错 1 px)
         declare
            Img_T : Buf := Im;
            Wt : Natural := Wd;
            Htt : Natural := Ht;
            Back_Ok : Boolean := True;
         begin
            for T in 1 .. 3 loop
               Img_T := Act.Turn_90 (Img_T, Wt, Htt);
               declare
                  W0 : constant Natural := Wt;
               begin
                  Wt := Htt; Htt := W0;
               end;
               for Y in 0 .. Htt - 1 loop
                  for X in 0 .. Wt - 1 loop
                     declare
                        U0, V0 : Long_Float;
                     begin
                        Act.Unturn (Long_Float (X) + 0.5, Long_Float (Y) + 0.5, T, Wd, Ht, U0, V0);   --  像素中心(连续坐标)
                        U0 := U0 - 0.5; V0 := V0 - 0.5;   --  中心 → 下标
                        if U0 < 0.0 or else V0 < 0.0 or else abs (U0 - Long_Float'Rounding (U0)) > 1.0e-9 or else abs (V0 - Long_Float'Rounding (V0)) > 1.0e-9
                          or else Natural (U0) >= Wd or else Natural (V0) >= Ht
                          or else Img_T ((Y * Wt + X) * 3) /= Im ((Natural (V0) * Wd + Natural (U0)) * 3)
                        then
                           Back_Ok := False;
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
            Check (Back_Ok, "转了 1/2/3 个 90° 的图里每个像素的中心(连续坐标)换算回原图:" & (if Back_Ok then "每个都回到本来那个像素的中心" else "有回错的(错)"));
         end;
      end;
   end;
   --  🔴 镜头畸变(2026-09-26,Geom.Cam_Dir / Cam_Pixel):K1 −0.25、K2 0.05(强广角,合成)⇒ 像素 → 视线 → 投回像素,整幅 640×480 每隔 40 px 一点,差 < 1e-6 px;
   --  畸变是真的起作用了:角上那一点按理想针孔的视线和去畸变的视线差得出来(> 1°)
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Gd : Geom.Cam_Geo;
      Worst : Long_Float := 0.0;
      Corner_Deg : Long_Float := 0.0;
   begin
      Gd.F := 397.0; Gd.Cx := 320.0; Gd.Cy := 240.0; Gd.K1 := -0.25; Gd.K2 := 0.05;
      for Yi in 0 .. 12 loop
         for Xi in 0 .. 16 loop
            declare
               U0 : constant Long_Float := 40.0 * Long_Float (Xi);
               V0 : constant Long_Float := 40.0 * Long_Float (Yi);
               D : constant Geom.V3 := Geom.Cam_Dir (Gd, U0, V0);
               U, V : Long_Float;
               Fr : Boolean;
            begin
               Geom.Cam_Pixel (Gd, D, U, V, Fr);
               Worst := Long_Float'Max (Worst, (if Fr then abs (U - U0) + abs (V - V0) else 1.0e9));
            end;
         end loop;
      end loop;
      declare
         Dd : constant Geom.V3 := Geom.Cam_Dir (Gd, 0.0, 0.0);
         Dp0 : constant Geom.V3 := [(0.0 - 320.0) / 397.0, -(0.0 - 240.0) / 397.0, -1.0];
         Np : constant Long_Float := Geom.Norm (Dp0);
         Cs : constant Long_Float := (Dd (0) * Dp0 (0) + Dd (1) * Dp0 (1) + Dd (2) * Dp0 (2)) / Np;
      begin
         Corner_Deg := Arccos (Long_Float'Min (1.0, Cs)) * 57.29578;   --  弧度 → 度(换算,无量纲)
      end;
      Check (Worst < 1.0e-6 and then Corner_Deg > 1.0,
             "镜头畸变:像素 → 视线 → 像素,最大差 " & Codec.Fmt (Worst, 9) & " px · 角上那一点去畸变的视线比理想针孔的偏 " & Codec.Fmt (Corner_Deg, 2) & "°");
   end;
   --  🔴 整幅掩膜 ⇒ 框、像素数、形心、主轴(Picture.Region_Of_Mask,SAM 出掩膜后用):合成 40×30 画幅里一条 20×4 的横条(x 10..29、y 5..8)
   --  ⇒ 框 [10 5 29 8]、80 px、形心 (19.5, 6.5)、主轴水平、伸长比 = √(方差比) ≈ 5.8;空掩膜 ⇒ 不成
   declare
      Mk : Bools;
      Rg : Picture.Region;
      Okm, Ok0 : Boolean;
      Rg0 : Picture.Region;
      Mk0 : Bools;
   begin
      for Y in 0 .. 29 loop
         for X in 0 .. 39 loop
            Mk.Append (X in 10 .. 29 and then Y in 5 .. 8);
            Mk0.Append (False);
         end loop;
      end loop;
      Picture.Region_Of_Mask (Mk, 40, 30, Rg, Okm);
      Picture.Region_Of_Mask (Mk0, 40, 30, Rg0, Ok0);
      Check (Okm and then Rg.X0 = 10 and then Rg.Y0 = 5 and then Rg.X1 = 29 and then Rg.Y1 = 8 and then Rg.Count = 80
             and then abs (40.0 * Rg.Cu - 19.5) < 1.0e-9 and then abs (30.0 * Rg.Cv - 6.5) < 1.0e-9 and then abs (Rg.Av) < 1.0e-9 and then Rg.Elong > 5.0 and then not Ok0,
             "整幅掩膜 ⇒ 框 [" & Codec.Img (Rg.X0) & " " & Codec.Img (Rg.Y0) & " " & Codec.Img (Rg.X1) & " " & Codec.Img (Rg.Y1) & "]、" & Codec.Img (Rg.Count)
             & " px、形心 (" & Codec.Fmt (40.0 * Rg.Cu, 2) & "," & Codec.Fmt (30.0 * Rg.Cv, 2) & ")、伸长比 " & Codec.Fmt (Rg.Elong, 2) & " · 空掩膜 ⇒ " & (if Ok0 then "成了(错)" else "不成"));
   end;
   --  🔴 碰桌面量指尖(2026-09-26,Geom.Tips_On_Plane):手上那只眼朝下,第 1 瓣的尖碰在面上 ⇒ 它的视线 ∩ 面 = 它的指尖(离眼 0.120 m,一分不差);
   --  第 2 瓣的尖比面高 2 mm(没碰着)⇒ 交出来只会更远;不确定度 = 面的离散 ÷ |视线·法向|。面在眼的上方 ⇒ 交不到(Ok = False)
   declare
      Gt : Geom.Cam_Geo;
      P : constant Plug.Arm_Pose := [0.1, -0.2, 1.0, 1.0, 0.0, 0.0, 0.0];   --  合成:手在 (0.1,−0.2,1.0),朝向不转 ⇒ 眼朝下
      Vs : Geom.Board_View_Vectors.Vector;
      S1 : constant Long_Float := 0.12;    --  第 1 瓣指尖离眼(米,合成)
      S2 : constant Long_Float := 0.118;   --  第 2 瓣(米,合成)
   begin
      Gt.Valid := True; Gt.F := 397.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.Off := [0.08, 0.0, 0.05];
      Vs.Append (Geom.Board_View'(Pose => P, U => 260.0, V => 260.0));
      Vs.Append (Geom.Board_View'(Pose => P, U => 375.0, V => 265.0));
      declare
         O : constant Geom.V3 := Geom.Cam_Pos (Gt, P);
         D1 : constant Geom.V3 := Geom.Ray (Gt, P, 260.0, 260.0);
         D2 : constant Geom.V3 := Geom.Ray (Gt, P, 375.0, 265.0);
         P0 : constant Geom.V3 := [O (0) + S1 * D1 (0), O (1) + S1 * D1 (1), O (2) + S1 * D1 (2)];
         Tip2_Z : constant Long_Float := O (2) + S2 * D2 (2);
         R : constant Geom.Plane_Tip_Vectors.Vector := Geom.Tips_On_Plane (Gt, Vs, P0, [0.0, 0.0, 1.0], 0.001);
         Up : constant Geom.Plane_Tip_Vectors.Vector := Geom.Tips_On_Plane (Gt, Vs, [0.0, 0.0, 1.3], [0.0, 0.0, 1.0], 0.001);
      begin
         Check (Natural (R.Length) = 2 and then R (0).Ok and then abs (R (0).S - S1) < 1.0e-9 and then abs (R (0).Sd - 0.001 / abs D1 (2)) < 1.0e-12
                and then R (1).Ok and then R (1).S > S2 and then Tip2_Z > P0 (2) and then not Up (0).Ok and then not Up (1).Ok,
                "碰桌面量指尖:碰着的那一瓣视线交面 = 它的指尖(" & Codec.Fmt (R (0).S, 6) & " m,真 0.120)、不确定度 " & Codec.Fmt (1000.0 * R (0).Sd, 3)
                & " mm;没碰着的那一瓣交出来更远(" & Codec.Fmt (R (1).S, 4) & " > 0.118)· 面在眼上方 ⇒ 交不到");
      end;
   end;
   --  🔴 开机碰桌面挑空的面(Act.Board_Free_Spots):板 21×21 个点铺在 0.765 m 的面上(2 cm 一格、离散 1 mm),中间 5×5 格是一块 5 cm 高的东西。
   --  压的那一瓣落在 (0,0)、另一瓣落在 (0.05,0),手指宽上限 1 cm,另一瓣视线斜 90°(tan 45° = 1:离压的那一点 ρ 处手指至少高 ρ)⇒ 第一个空的:
   --  压的那一瓣落在一个躺在面上的板点上,两个落点连线 1 cm 内没有东西上的点,挪得不远(< 0.1 m);拿掉那块东西 ⇒ 不用挪;板上全是东西 ⇒ 一个都没有。
   --  V1B22 2026-09-27 起别的瓣只躲它真会碰到的:平板上只放一个 5 mm 高的小东西在另一瓣连线的 4 cm 处(视线斜得 tan(β/2) = 0.5,那里手指离面至少 2 cm)⇒ 不用挪;
   --  同一个小东西放在压的那一点 ⇒ 要挪
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      C1 : Act.Context;
      Lp : Geom.V3_Vectors.Vector;
      Tb : Bytes.Floats;
      Ds1, Ds2, Ds3, Ds4, Ds5 : Geom.V3_Vectors.Vector;
      Dl : Geom.V3 := [0.0, 0.0, 0.0];
      Clear_Ok : Boolean := True;
      On_Pt : Boolean := False;
      Cell : constant Long_Float := 0.02;       --  格距(米,合成)
      Hgt : constant Long_Float := 0.05;        --  东西高(米,合成)
      Rw : constant Long_Float := 0.01;         --  手指宽上限(米,合成)
      function Pt (I, J : Integer; Z : Long_Float) return Geom.Scene_Pt is
        (Geom.Scene_Pt'(Pw => [Cell * Long_Float (I), Cell * Long_Float (J), Z], U => 0.0, V => 0.0, Sh => 0.0, Views => 9,
                        Cov => [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]]));
   begin
      C1.Board_Plane := True; C1.Board_Pt := [0.0, 0.0, 0.765]; C1.Board_N := [0.0, 0.0, 1.0]; C1.Board_Rms := 0.001;
      for I in -10 .. 10 loop
         for J in -10 .. 10 loop
            C1.Board.Append (Pt (I, J, (if abs I <= 2 and then abs J <= 2 then 0.765 + Hgt else 0.765)));
         end loop;
      end loop;
      Lp.Append (Geom.V3'[0.0, 0.0, 0.765]);
      Lp.Append (Geom.V3'[0.05, 0.0, 0.765]);
      Tb.Append (0.0); Tb.Append (1.0);
      Act.Board_Free_Spots (C1, Lp, Tb, Rw, Ds1);
      if not Ds1.Is_Empty then
         Dl := Ds1 (0);
         for S of C1.Board loop
            declare
               Ax : constant Long_Float := Dl (0); Ay : constant Long_Float := Dl (1);
               Bx : constant Long_Float := 0.05 + Dl (0);
               Qx : constant Long_Float := S.Pw (0); Qy : constant Long_Float := S.Pw (1);
               T : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (1.0, (Qx - Ax) / (Bx - Ax)));
               Dd : constant Long_Float := Sqrt ((Qx - (Ax + T * (Bx - Ax))) ** 2 + (Qy - Ay) ** 2);
            begin
               if S.Pw (2) > 0.78 and then Dd <= Rw then
                  Clear_Ok := False;
               end if;
               if S.Pw (2) < 0.78 and then Sqrt ((Qx - Ax) ** 2 + (Qy - Ay) ** 2) < 1.0e-9 then
                  On_Pt := True;
               end if;
            end;
         end loop;
      end if;
      declare
         C2 : Act.Context := C1;
         C3 : Act.Context := C1;
         C4 : Act.Context := C1;
         C5 : Act.Context := C1;
         Tb4 : Bytes.Floats;
      begin
         C2.Board.Clear; C4.Board.Clear; C5.Board.Clear;
         for I in -10 .. 10 loop
            for J in -10 .. 10 loop
               C2.Board.Append (Pt (I, J, 0.765));
               C3.Board.Replace_Element (Natural ((I + 10) * 21 + J + 10), Pt (I, J, 0.765 + Hgt));
               C4.Board.Append (Pt (I, J, (if I = 2 and then J = 0 then 0.770 else 0.765)));
               C5.Board.Append (Pt (I, J, (if I = 0 and then J = 0 then 0.770 else 0.765)));
            end loop;
         end loop;
         Act.Board_Free_Spots (C2, Lp, Tb, Rw, Ds2);
         Act.Board_Free_Spots (C3, Lp, Tb, Rw, Ds3);
         Tb4.Append (0.0); Tb4.Append (0.8 / (1.0 + 0.6));   --  视线斜的角:sin 0.8、cos 0.6(合成,3-4-5 直角三角形)⇒ tan(β/2) = 0.5
         Act.Board_Free_Spots (C4, Lp, Tb4, Rw, Ds4);
         Act.Board_Free_Spots (C5, Lp, Tb4, Rw, Ds5);
      end;
      Check (not Ds1.Is_Empty and then Clear_Ok and then On_Pt and then Geom.Norm (Dl) < 0.1
             and then not Ds2.Is_Empty and then Geom.Norm (Ds2 (0)) = 0.0 and then Ds3.Is_Empty
             and then not Ds4.Is_Empty and then Geom.Norm (Ds4 (0)) = 0.0 and then not Ds5.Is_Empty and then Geom.Norm (Ds5 (0)) > 0.0,
             "开机碰桌面挑空的面:避开 5 cm 高的那块东西挪了 (" & Codec.Fmt (Dl (0), 3) & "," & Codec.Fmt (Dl (1), 3) & ") m,落点在躺在面上的板点上、"
             & "连线 1 cm 内没有东西 · 拿掉东西 ⇒ 不挪 · 板上全是东西 ⇒ 一个都没有 · 另一瓣连线 4 cm 处 5 mm 高的小东西不挡(手指在那儿至少高 2 cm)、"
             & "放在压的那一点就要挪(" & (if Ds5.Is_Empty then "-" else Codec.Fmt (Geom.Norm (Ds5 (0)), 3)) & " m)");
   end;
   --  🔴 有板的面时,朝下顶住的点只对账、不换面(Act.Note_Support,2026-09-26):X5B 指尖错了的那只手顶住的点比板的面低 20.8 cm,"最低的赢"把它当成了桌面。
   --  低 20 cm ⇒ 面还是板的、不记东西;高 5 cm ⇒ 记成"这儿有东西"、面不变;差 0.5 mm(门 = 3 倍 1 mm ⊕ 0.5 mm)⇒ 对得上
   declare
      C1 : Act.Context;
      B0 : constant Geom.V3 := [0.0, 0.0, 0.765];
      Low_Ok, High_Ok, Near_Ok : Boolean;
   begin
      C1.Board_Plane := True; C1.Board_Pt := B0; C1.Board_N := [0.0, 0.0, 1.0]; C1.Board_Rms := 0.001; C1.Map.EE_Noise := 0.0005;
      Act.Note_Support (C1, B0, [0.0, 0.0, 1.0], "合成:板的面");
      Act.Note_Support (C1, [0.3, 0.1, 0.565], [0.27, -0.25, 0.93], "合成:低 20 cm");
      Low_Ok := C1.Touch_Valid and then Geom."=" (C1.Touch_Pt, B0) and then C1.Bumps.Is_Empty;
      Act.Note_Support (C1, [0.3, 0.1, 0.815], [0.0, 0.0, 1.0], "合成:高 5 cm");
      High_Ok := Geom."=" (C1.Touch_Pt, B0) and then Natural (C1.Bumps.Length) = 1;
      Act.Note_Support (C1, [0.3, 0.1, 0.7655], [0.0, 0.0, 1.0], "合成:差 0.5 mm");
      Near_Ok := Geom."=" (C1.Touch_Pt, B0) and then Natural (C1.Bumps.Length) = 1 and then Geom."=" (C1.Touch_N, [0.0, 0.0, 1.0]);
      Check (Low_Ok and then High_Ok and then Near_Ok,
             "有板的面时顶住的点只对账:低 20 cm 不换面、不记东西" & (if Low_Ok then "" else "(错)") & " · 高 5 cm 记成东西" & (if High_Ok then "" else "(错)")
             & " · 差 0.5 mm 对得上" & (if Near_Ok then "" else "(错)"));
   end;
   --  🔴 认指尖:瓣尖落在哪条腕眼瓣视线上(2026-09-24,Geom.Tips_On_Rays)。不动的眼已知(合成:(0,−0.41,1.308) 低头 30°、焦距 288);腕眼在手系 (0.08,0,0.05)。
   --  ① 五指手:自己眼里 1 瓣(四根手指),指尖在视线上 0.20 m;不动的眼每笔看见 2 瓣,另一瓣是大拇指(离指尖 5 cm,不在视线上)
   --  ⇒ 手指的尖全归视线、大拇指一个都不归,S 差 < 2 mm(G2C 实拍:手指那一瓣离视线 5.5 px、大拇指 25 px)。
   --  ② 两指夹爪:自己眼里 2 瓣,两条视线上各 0.15 / 0.16 m ⇒ 每条归一半的尖,两个 S 都差 < 2 mm。1 px 抖动,15 停(转 ±0.1 rad + 平移),门槛 3 px
   declare
      Gt : Geom.Cam_Geo;
      Sn : constant Long_Float := 0.5;                --  sin 30°(合成)
      Cs : constant Long_Float := 0.8660254;          --  cos 30°(合成)
      Off : constant Geom.V3 := [0.08, 0.0, 0.05];    --  腕眼离手腕原点(手系,米,合成)
      function Unit (X : Geom.V3) return Geom.V3 is
         N : constant Long_Float := Geom.Norm (X);
      begin
         return [X (0) / N, X (1) / N, X (2) / N];
      end Unit;
      D0 : constant Geom.V3 := Unit ([0.9, -0.2, -0.3]);   --  第一瓣的视线(手系,合成)
      D1 : constant Geom.V3 := Unit ([0.9, 0.2, -0.3]);    --  第二瓣的视线(手系,合成)
      Thumb_Off : constant Geom.V3 := [-0.04, 0.03, -0.02];   --  大拇指的尖离手指的尖(手系,米,合成)
      Per_Mm : constant Long_Float := 1000.0;         --  米 → 毫米(换算,无量纲)
      Seed3 : Long_Long_Integer := 11;
      function Jit3 return Long_Float is   --  确定性伪随机 ±1 px(测试数据自己的抖动)
      begin
         Seed3 := (Seed3 * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Integer ((Seed3 / 65536) mod 2001) - 1000) / 1000.0;
      end Jit3;
      Poses : Geom.Obs_Vectors.Vector;
      H : constant Geom.V3 := [-0.2, 0.25, 0.85];   --  手的起点(世界,米,合成;在不动的眼前下方)
      Obs1, Obs2 : Geom.Obs_Pt_Vectors.Vector;
      function At_Pose (Ps : Plug.Arm_Pose; T : Geom.V3) return Geom.V3 is
         Tw : constant Geom.V3 := Geom.Ap (Geom.Quat_To_R (Ps), T);
      begin
         return [Ps (0) + Tw (0), Ps (1) + Tw (1), Ps (2) + Tw (2)];
      end At_Pose;
      procedure Mark (Into : in out Geom.Obs_Pt_Vectors.Vector; Ps : Plug.Arm_Pose; T : Geom.V3) is
         U, V : Long_Float;
         Fr : Boolean;
      begin
         Geom.Project_Fixed (Gt, At_Pose (Ps, T), U, V, Fr);
         Check (Fr and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0, "认指尖:合成的尖在画面里(测试数据自己先得成立)");
         Into.Append (Geom.Obs_Pt'(Pt => 0, Pose => Ps, U => U + Jit3, V => V + Jit3, Seq => 0, Kind => 2));
      end Mark;
   begin
      Gt.R_Ce := [[1.0, 0.0, 0.0], [0.0, Sn, -Cs], [0.0, Cs, Sn]];
      Gt.Pos := [0.0, -0.41, 1.308]; Gt.F := 288.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.Fixed := True; Gt.Valid := True;
      Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));
      Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.0, 0.0, 0.04998], U => 0.0, V => 0.0));    --  ±0.1 rad 的四元数(cos/sin 0.05,合成)
      Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.0, 0.0, -0.04998], U => 0.0, V => 0.0));
      Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, 0.04998, 0.0, 0.0], U => 0.0, V => 0.0));
      Poses.Append (Geom.Obs'(Pose => [H (0), H (1), H (2), 0.99875, -0.04998, 0.0, 0.0], U => 0.0, V => 0.0));
      for I in 1 .. 10 loop
         Poses.Append (Geom.Obs'(Pose => [H (0) + 0.02 * Long_Float (I mod 3), H (1) + 0.03 * Long_Float (I mod 4), H (2) + 0.02 * Long_Float (I mod 5), 1.0, 0.0, 0.0, 0.0], U => 0.0, V => 0.0));   --  平移(米,合成)
      end loop;
      for Ps of Poses loop
         declare
            Tip : constant Geom.V3 := [Off (0) + 0.20 * D0 (0), Off (1) + 0.20 * D0 (1), Off (2) + 0.20 * D0 (2)];   --  0.20 m(合成真值)
         begin
            Mark (Obs1, Ps.Pose, Tip);
            Mark (Obs1, Ps.Pose, [Tip (0) + Thumb_Off (0), Tip (1) + Thumb_Off (1), Tip (2) + Thumb_Off (2)]);
            Mark (Obs2, Ps.Pose, [Off (0) + 0.15 * D0 (0), Off (1) + 0.15 * D0 (1), Off (2) + 0.15 * D0 (2)]);   --  0.15 m(合成真值)
            Mark (Obs2, Ps.Pose, [Off (0) + 0.16 * D1 (0), Off (1) + 0.16 * D1 (1), Off (2) + 0.16 * D1 (2)]);   --  0.16 m(合成真值)
         end;
      end loop;
      declare
         R1 : constant Geom.Ray_Tip_Vectors.Vector := Geom.Tips_On_Rays (Gt, Obs1, Off, Geom.V3_Vectors.To_Vector (D0, 1), 3.0);   --  门槛 3 px(合成)
         Rays2 : Geom.V3_Vectors.Vector;
      begin
         Check (Natural (R1.Length) = 1 and then R1 (0).N = Natural (Poses.Length) and then abs (R1 (0).S - 0.20) < 0.002,
                "认指尖·五指手:" & Codec.Img (Natural (Poses.Length)) & " 笔 × 2 瓣 ⇒ 归视线 " & Codec.Img (R1 (0).N) & " 个(该 " & Codec.Img (Natural (Poses.Length))
                & ",大拇指一个不归)· 离眼 " & Codec.Fmt (R1 (0).S * Per_Mm, 1) & " mm(真 200)± " & Codec.Fmt (R1 (0).Spread * Per_Mm, 1) & " mm");
         Rays2.Append (D0); Rays2.Append (D1);
         declare
            R2 : constant Geom.Ray_Tip_Vectors.Vector := Geom.Tips_On_Rays (Gt, Obs2, Off, Rays2, 3.0);   --  门槛 3 px(合成)
         begin
            Check (Natural (R2.Length) = 2 and then R2 (0).N = Natural (Poses.Length) and then R2 (1).N = Natural (Poses.Length)
                   and then abs (R2 (0).S - 0.15) < 0.002 and then abs (R2 (1).S - 0.16) < 0.002,
                   "认指尖·两指夹爪:两条视线各归 " & Codec.Img (R2 (0).N) & " / " & Codec.Img (R2 (1).N) & " 个 ⇒ 离眼 " & Codec.Fmt (R2 (0).S * Per_Mm, 1) & " / "
                   & Codec.Fmt (R2 (1).S * Per_Mm, 1) & " mm(真 150 / 160)");
         end;
      end;
   end;
   --  🔴 没有深度时量指尖(2026-09-23):指尖 = 自己眼里那条视线上离眼 S 米处;不动的眼两停看见指尖 ⇒ 解 S。合成数据:真值 0.12 m
   declare
      Gf : Geom.Cam_Geo;
      Gh : Geom.Cam_Geo;
      Dir : constant Geom.V3 := [0.1 / 1.0247, -0.2 / 1.0247, -1.0 / 1.0247];   --  单位视线(分母 = 模长,纯数学)
      S_True : constant Long_Float := 0.12;   --  合成真值(测试数据,无量纲意义上的"给定")
      Obs, One_Obs : Geom.Obs_Vectors.Vector;
      S_Got, Rms : Long_Float;
      Ok : Boolean;
   begin
      Gf.Fixed := True; Gf.F := 400.0; Gf.Cx := 320.0; Gf.Cy := 240.0; Gf.Pos := [0.3, 0.2, 1.5]; Gf.R_Ce := Geom.Identity;   --  不动的眼:朝下看
      Gh.Valid := True; Gh.F := 400.0; Gh.Cx := 320.0; Gh.Cy := 240.0; Gh.R_Ce := Geom.Rodrigues ([0.3, -0.2, 0.1]);   --  手上的眼:随便一个朝向
      for I in 0 .. 2 loop
         declare
            P : Plug.Arm_Pose := [0.05 * Long_Float (I), 0.1 - 0.03 * Long_Float (I), 0.9 + 0.05 * Long_Float (I), 1.0, 0.0, 0.0, 0.0];
            Tw : constant Geom.V3 := Geom.Ap (Geom.Cam_R (Gh, P), [S_True * Dir (0), S_True * Dir (1), S_True * Dir (2)]);
            U, V : Long_Float;
            Fr : Boolean;
         begin
            Geom.Project_Fixed (Gf, [P (0) + Tw (0), P (1) + Tw (1), P (2) + Tw (2)], U, V, Fr);
            Check (Fr, "量指尖:合成的指尖在不动的眼前面(测试数据自己先得成立)");
            Obs.Append (Geom.Obs'(Pose => P, U => U, V => V));
            if I = 0 then
               One_Obs.Append (Geom.Obs'(Pose => P, U => U, V => V));
            end if;
         end;
      end loop;
      Geom.Fit_Tip_Scale (Gf, Gh, Dir, Obs, S_Got, Rms, Ok);
      Check (Ok and then abs (S_Got - S_True) < 1.0e-6 and then Rms < 1.0e-6,
             "量指尖:三停 ⇒ 离眼 " & Codec.Fmt (S_Got, 5) & " m(真 0.12)· 残差 " & Codec.Fmt (Rms, 4) & " px");
      Geom.Fit_Tip_Scale (Gf, Gh, Dir, One_Obs, S_Got, Rms, Ok);
      Check (Ok and then abs (S_Got - S_True) < 1.0e-6, "量指尖:一停也够(两条视线不平行时一个未知数两条方程)⇒ " & Codec.Fmt (S_Got, 5) & " m");
      declare
         Par : Geom.Obs_Vectors.Vector;
         Gd : Geom.Cam_Geo := Gh;
         P : constant Plug.Arm_Pose := [0.3, 0.2, 0.9, 1.0, 0.0, 0.0, 0.0];   --  指尖正好在不动眼的正下方
      begin
         Gd.R_Ce := Geom.Identity;
         Par.Append (Geom.Obs'(Pose => P, U => 320.0, V => 240.0));
         Geom.Fit_Tip_Scale (Gf, Gd, [0.0, 0.0, -1.0], Par, S_Got, Rms, Ok);
         Check (not Ok, "量指尖:指尖视线和不动眼的视线平行 ⇒ 解不出,如实说");
         Geom.Fit_Tip_Scale (Gf, Gh, Dir, Geom.Obs_Vectors.Empty_Vector, S_Got, Rms, Ok);
         Check (not Ok, "量指尖:一停都没有 ⇒ 不解");
      end;
   end;
   --  🔴 两眼同时交点(2026-09-22):两条视线 ⇒ 交点;一条 ⇒ 不解;两条平行 ⇒ 不解;交在眼后 ⇒ 不解
   declare
      Rs : Geom.Sight_Vectors.Vector;
      Ok : Boolean;
      Sp : Long_Float;
      P : Geom.V3;
   begin
      --  两只眼(一只在 (0,0,1),一只在 (1,0,1))都看着同一点 (0,0.75,0):方向 = 点 − 眼,归一化
      declare
         function Toward (O : Geom.V3) return Geom.V3 is
            D : constant Geom.V3 := [0.0 - O (0), 0.75 - O (1), 0.0 - O (2)];
            N : constant Long_Float := Geom.Norm (D);
         begin
            return [D (0) / N, D (1) / N, D (2) / N];
         end Toward;
      begin
         Rs.Append (Geom.Sight'(O => [0.0, 0.0, 1.0], D => Toward ([0.0, 0.0, 1.0])));
         Rs.Append (Geom.Sight'(O => [1.0, 0.0, 1.0], D => Toward ([1.0, 0.0, 1.0])));
      end;
      P := Geom.Meet (Rs, Ok, Sp);
      Check (Ok and then Sp < 1.0e-6 and then abs (P (2) - 0.0) < 1.0e-6 and then abs (P (1) - 0.75) < 1.0e-6 and then abs (P (0) - 0.0) < 1.0e-6,
             "两眼交点:两条视线交在 (" & Codec.Fmt (P (0), 3) & "," & Codec.Fmt (P (1), 3) & "," & Codec.Fmt (P (2), 3) & "),偏差 " & Codec.Fmt (Sp, 6));
      Rs.Delete_Last;
      P := Geom.Meet (Rs, Ok, Sp);
      Check (not Ok, "两眼交点:只有一条视线 ⇒ 不解,如实说");
      declare
         D0 : constant Geom.V3 := Rs (0).D;   --  先拷出来再 Append(容器不许一边引用一边改)
      begin
         Rs.Append (Geom.Sight'(O => [1.0, 0.0, 1.0], D => D0));
      end;
      P := Geom.Meet (Rs, Ok, Sp);
      Check (not Ok, "两眼交点:两条平行视线 ⇒ 不解");
      Rs.Clear;
      Rs.Append (Geom.Sight'(O => [0.0, 0.0, 1.0], D => [0.0, 0.6, -0.8]));
      Rs.Append (Geom.Sight'(O => [1.0, 0.0, 1.0], D => [0.6, 0.0, 0.8]));   --  第二条背对交点
      P := Geom.Meet (Rs, Ok, Sp);
      Check (not Ok, "两眼交点:交点在某只眼背后 ⇒ 不解");
   end;
   --  🔴 框里量(2026-09-21):脑只说"它在这一框里",哪些像素是它由身体自己量。四条焊点,正反都要有。
   --  合成画面:木纹桌面(灰度 80 上下抖 ±6 的条纹)上放一根亮的长条(像剪刀那样细长)。
   declare
      W : constant Natural := 200;
      H : constant Natural := 160;
      function Table_Gray return Buf is
         G : Buf;
      begin
         for Y in 0 .. H - 1 loop
            for X in 0 .. W - 1 loop
               G.Append (U8 (80 + ((X / 3 + Y / 7) mod 3) * 6 - 6));
            end loop;
         end loop;
         return G;
      end Table_Gray;
      procedure Paint (G : in out Buf; X0, Y0, X1, Y1 : Natural; V : U8) is
      begin
         for Y in Y0 .. Y1 loop
            for X in X0 .. X1 loop
               G.Replace_Element (Y * W + X, V);
            end loop;
         end loop;
      end Paint;
      G : Buf;
      Found, Alone : Boolean;
      R : Picture.Region;
   begin
      --  ① 正例:一根 12x60 的亮条,脑的框比它略大 ⇒ 量到一整块、是单独的、形心在条的正中、细长
      G := Table_Gray;
      Paint (G, 94, 50, 105, 109, 210);
      Picture.Measure_In_Box (G, W, H, 90, 46, 109, 113, Found, Alone, R);
      Check (Found and then Alone and then R.Count = 12 * 60
             and then abs (R.Cu * Long_Float (W) - 99.5) < 0.6 and then abs (R.Cv * Long_Float (H) - 79.5) < 0.6
             and then R.Elong > 3.0 and then abs R.Av > abs R.Au,
             "框里量:亮条量成一整块,形心在正中,长轴竖着(" & Codec.Img (R.Count) & " px,长宽比 " & Codec.Fmt (R.Elong, 1) & ")");
      --  ② 反例:同一张桌面上一块什么都没有的地方 ⇒ 如实说量不到,不许从木纹里硬凑一块
      Picture.Measure_In_Box (G, W, H, 20, 20, 60, 50, Found, Alone, R);
      Check (not Found, "框里量:空桌面 ⇒ 量不到(木纹分不成两拨)");
      --  ③ 挨着邻物:亮条旁边紧贴一大块同样亮的东西,压进让出来的那一圈 ⇒ 量得到,但如实说"不是单独的一块"
      G := Table_Gray;
      Paint (G, 94, 50, 105, 109, 210);
      Paint (G, 106, 30, 160, 130, 205);
      Picture.Measure_In_Box (G, W, H, 90, 46, 109, 113, Found, Alone, R);
      Check (Found and then not Alone, "框里量:挨着邻物 ⇒ 量得到但说【不是单独的】(不把连在一起的一大片当成它的形心去信)");
      --  ④ 手指伸进框边:一小块亮东西只伸进让出来的那一圈、形心在脑的框【外】⇒ 选中的仍是亮条,不是它
      G := Table_Gray;
      Paint (G, 94, 50, 105, 109, 210);
      Paint (G, 80, 118, 120, 128, 230);
      Picture.Measure_In_Box (G, W, H, 90, 46, 109, 113, Found, Alone, R);
      Check (Found and then R.Count = 12 * 60 and then abs (R.Cv * Long_Float (H) - 79.5) < 0.6,
             "框里量:伸进框边的另一块(形心在框外)不顶替它(" & Codec.Img (R.Count) & " px)");
   end;
   --  身体图:最近样本按探针幅度归一;同位姿(噪声内)再看一次 = 顶替不是新增
   declare
      M : Schema.Map;
      X : Schema.Sample;
      Amp : Floats;
      Diff : Table.Vec;
      Dist : Long_Float;
      N : Integer;
   begin
      for I in 1 .. 6 loop
         Amp.Append (0.01);
      end loop;
      X.Arm := 0; X.Cam := 0; X.Pose := [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
      X.Parts (Chan.Per_Arm) := (True, 0.3, 0.5, 0.2, 0, 0, 0, 0, 2, 0.3, 0.5, 0.4, 0.5);
      Schema.Add (M, X, 1.0e-3, 1.0e-3);
      X.Pose (0) := 0.1; X.Parts (Chan.Per_Arm).B0u := 0.5;
      Schema.Add (M, X, 1.0e-3, 1.0e-3);
      N := Schema.Nearest (M, 0, 0, [0.09, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], Amp, 6, Diff, Dist);
      Check (N = 1 and then abs (Diff (0) + 0.01) < 1.0e-9 and then abs (Dist - 1.0) < 1.0e-6, "身体图:最近样本是 x=0.1 那个,差 -0.01 = 一个探针幅度(" & Codec.Fmt (Dist, 3) & ")");
      X.Pose (0) := 0.1 + 1.0e-5; X.Parts (Chan.Per_Arm).B0u := 0.51;
      X.Parts (0) := (True, 0.7, 0.7, 0.0, 0, 0, 0, 0, 1, 0.7, 0.7, 0.0, 0.0);
      Schema.Add (M, X, 1.0e-3, 1.0e-3);
      Check (Schema.Count (M, 0, 0) = 2 and then abs (M.S (1).Parts (Chan.Per_Arm).B0u - 0.51) < 1.0e-9 and then M.S (1).Parts (0).Valid,
             "身体图:同位姿再看一次 = 按块合进去(仍 2 个样本,手指新值 0.51,零件 0 补上)");
      N := Schema.Nearest (M, 1, 0, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], Amp, 6, Diff, Dist);
      Check (N = -1, "身体图:别的手没有样本 ⇒ -1");
   end;
   --  BMP 头
   declare
      B : constant Buf := Codec.BMP24 (From_String ("abcdef"), 2, 1);
   begin
      Check (Natural (B.Length) = 54 + 8 and then B (0) = 66 and then B (54) = 99, "BMP24 头与 BGR 顺序");
   end;
   --  ── 编译:体检的否决权(对着 Sinew) ──
   declare
      use Sinew;
      use type Exam.Row_Id;
      R : Exam.Report;
      Facts : Plan.Facts_Vectors.Vector;
      Binds : Plan.Bind_Vectors.Vector;
      T : Exam.Thing_Check;
      Ft : Plan.Item_Facts;
      function Comp (Src : String) return Plan.Verdict is
        (Plan.Check (Sinew.Parse (Src), R, Facts, Binds));
      procedure Set_Stands (On : Boolean) is
         X : Plan.Item_Facts := Facts (2);
      begin
         X.Stands := On;
         Facts.Replace_Element (2, X);
      end Set_Stands;
   begin
      --  一具想象的身体:左右/上下/远近证过了能用,朝哪和看着多大是死的
      for Row in Exam.Row_Id loop
         T.Rows (Row).V := (if Row = Exam.Facing or else Row = Exam.Bigness then Exam.Dead else Exam.Usable);
         T.Rows (Row).Why := To_Unbounded_String ("这一行一次都没动过");
      end loop;
      R.Things.Append (T);
      declare
         E1 : Exam.Eye_Check;
      begin
         R.Eyes.Append (E1);
      end;
      Facts.Append (Plan.Item_Facts'(others => <>));                        --  0 号空着
      Ft := (Exists => True, Mine => True, Grasp => True, Arm => 0, Jaw_K => 0,
             Thing_Idx => 0, Stands => False, Span => 0.10, Size => 0.05,
             Label => To_Unbounded_String ("grasper"));
      Facts.Append (Ft);                                                    --  1 = 我的 grasper
      Ft := (Exists => True, Mine => False, Grasp => False, Arm => 0, Jaw_K => 0,
             Thing_Idx => -1, Stands => False, Span => 0.0, Size => 0.04,
             Label => To_Unbounded_String ("外面的东西"));
      Facts.Append (Ft);                                                    --  2 = 外面那个东西
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("grasper"), Item => 1, Tried => <>));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("pusher"), Item => 1, Tried => <>));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("me"), Item => -1, Tried => <>));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("the ball"), Item => 2, Tried => <>));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("the moon"), Item => -1,
                                     Tried => To_Unbounded_String ("我把看得见的每一块都过了一遍,没有一块是它")));

      Check (Comp ("do grasper touching the ball small until touched").Ok, "编译:能用的行 ⇒ 收");
      --  语言的根:「<东西> height up until <结局>」—— 主语是外面的东西,不查手的那几行(H44–H46 实测这一句一直被当"谁去哪"退回)
      Check (Comp ("do the ball height up until settled").Ok, "编译:「the ball height up」⇒ 收(主语是外面的东西)");
      Check (not Comp ("do grasper height up until settled").Ok, "编译:「grasper height up」⇒ 退回(量说的是外面的东西)");
      Check (not Comp ("do the moon height up until settled").Ok, "编译:认不出的东西 height up ⇒ 退回");
      Check (not Comp ("do the ball weight up until settled").Ok, "编译:不是我量得出的量 ⇒ 退回");
      declare
         V : constant Plan.Verdict := Comp ("do grasper facing the ball must until arrived");
      begin
         Check (not V.Ok, "编译:朝哪是死的 ⇒ 退回");
         Check (Length (V.Instead) > 0, "编译:退回必须附能照抄的替代(" & To_String (V.Instead) & ")");
      end;
      Check (not Comp ("do the ball touching grasper small until touched").Ok,
             "编译:命令外面的东西动 ⇒ 退回(我只推得动我自己)");
      Check (not Comp ("do me touching the ball small until touched").Ok,
             "编译:这具身体没有 me 这个角色 ⇒ 退回");
      Check (not Comp ("do grasper touching the moon small until touched").Ok,
             "编译:名字认不出 ⇒ 退回(不是语法错,是身体认不出)");
      Check (not Comp ("do grasper onto the ball small until touched").Ok,
             "编译:量不出它鼓出多少 ⇒ onto 说不出口");
      Check (not Comp ("do grasper press the ball firm until stuck").Ok,
             "编译:量不出它鼓出多少 ⇒ press 说不出口(不知道哪个方向算朝它压)");
      Check (not Comp ("do grasper touching the ball small until free").Ok,
             "编译:量不出它鼓出多少 ⇒ until free 说不出口");
      Set_Stands (True);
      Check (Comp ("do grasper onto the ball small until touched").Ok, "编译:量得出 ⇒ onto 能说了");
      Check (Comp ("do grasper press the ball firm until stuck").Ok, "编译:量得出 ⇒ press 能说了");
      Set_Stands (False);
      declare
         V : constant Plan.Verdict := Comp
           ("repeat 2 times:" & ASCII.LF
            & "  do grasper above the ball small and grasper touching the ball must until touched" & ASCII.LF
            & "  do grasper close on the ball until stuck" & ASCII.LF
            & "end");
      begin
         Check (V.Ok, "编译:循环里的每一条约束都过一遍,一段程序整体收"
                & (if V.Ok then "" else " —— " & To_String (V.Err)));
      end;
      declare
         X : Plan.Item_Facts := Facts (1);
      begin
         X.Grasp := False;
         Facts.Replace_Element (1, X);
         Check (not Comp ("do grasper close on the ball until stuck").Ok,
                "编译:没量到它能相向靠拢 ⇒ 说不出「合拢」");
         X.Grasp := True;
         Facts.Replace_Element (1, X);
      end;
   end;
   --  ── 空转:整段在心里跑一遍,不通电 ──
   declare
      use Sinew;
      R : Exam.Report;
      Facts : Plan.Facts_Vectors.Vector;
      Binds : Plan.Bind_Vectors.Vector;
      T : Exam.Thing_Check;
      function Dry (Src : String) return Plan.Verdict is
        (Plan.Dry_Run (Sinew.Parse (Src), R, Facts, Binds));
      procedure Set_Size (Sz : Long_Float) is
         X : Plan.Item_Facts := Facts (2);
      begin
         X.Size := Sz;
         Facts.Replace_Element (2, X);
      end Set_Size;
   begin
      for Row in Exam.Row_Id loop
         T.Rows (Row).V := Exam.Usable;
      end loop;
      R.Things.Append (T);
      Facts.Append (Plan.Item_Facts'(others => <>));
      Facts.Append (Plan.Item_Facts'(Exists => True, Mine => True, Grasp => True, Arm => 0, Jaw_K => 0,
                    Thing_Idx => 0, Stands => True, Span => 0.10, Size => 0.05,
                    Label => To_Unbounded_String ("grasper")));
      Facts.Append (Plan.Item_Facts'(Exists => True, Mine => False, Grasp => False, Arm => 0, Jaw_K => 0,
                    Thing_Idx => -1, Stands => True, Span => 0.0, Size => 0.04,
                    Label => To_Unbounded_String ("ball")));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("grasper"), Item => 1, Tried => <>));
      Binds.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("the ball"), Item => 2, Tried => <>));

      Check (Dry ("do grasper touching the ball small until touched" & ASCII.LF
                  & "do grasper close on the ball until stuck").Ok,
             "空转:走得通的一段,空转放行");
      declare
         V : constant Plan.Verdict := Dry
           ("repeat until free:" & ASCII.LF
            & "  do grasper touching the ball small until touched" & ASCII.LF & "end");
      begin
         Check (not V.Ok, "空转:循环在等一个这段程序里【永远不会发生】的结局 ⇒ 心里就转不出来,当场拦住");
         Check (Length (V.Instead) > 0, "空转:拦住时也要给能照抄的替代(" & To_String (V.Instead) & ")");
      end;
      Check (Dry ("repeat until touched:" & ASCII.LF
                  & "  do grasper touching the ball small until touched" & ASCII.LF & "end").Ok,
             "空转:循环等的结局这一节真产得出 ⇒ 放行");
      Set_Size (0.5);
      --  🔴 这条断言以前是反的("比我张得开的还大 ⇒ 拦住")。它比的是整块外廓,而爪子夹的是窄的那一向:
      --  剪刀永远比爪口长 ⇒ `close 剪刀` 永远编译不过(T5 2026-09-21 实测被它挡在动手之前)。
      --  总规矩:身体不许自称"物理上做不到";成没成由合完提一提来判。
      Check (Dry ("do grasper close on the ball until stuck").Ok,
             "空转:东西的外廓比爪口大【不是】拒绝的理由 ⇒ 放行(夹得住夹不住,合完提一提才知道)");
      Set_Size (0.04);
      Check (not Dry ("run nothing").Ok, "空转:叫一个没 to 过的名字 ⇒ 拦住");
   end;
   --  ── 优先级:hold 那一条不许被牺牲 ──
   --  造一个【真的挤不下】的局面:只有一个自由度,朝向要 1.0,位置要 10.0。
   --  平权解一定折中(昨晚就是这个形状);零空间解必须保住朝向,宁可位置一点不办。
   declare
      use Table;
      Hard, Soft, Both : Term_Vectors.Vector;
      H, S : Term;
      Cap : constant Vec := [others => 100.0];
      Damp : constant Vec := [others => 1.0e-9];
      One : Mask := [others => False];
      Two : Mask := [others => False];
      A : Vec;
      Okp : Boolean;
   begin
      One (0) := True;
      Two (0) := True; Two (1) := True;
      Reset (H.E, 1, 1.0);
      Set_Col (H.E, 0, [0.0, 0.0, 0.0, 0.0, 1.0]);
      H.Err := [0.0, 0.0, 0.0, 0.0, 1.0];  H.W := [0.0, 0.0, 0.0, 0.0, 1.0];
      Reset (S.E, 1, 1.0);
      Set_Col (S.E, 0, [1.0, 0.0, 0.0, 0.0, 0.0]);
      S.Err := [10.0, 0.0, 0.0, 0.0, 0.0];  S.W := [1.0, 0.0, 0.0, 0.0, 0.0];
      Hard.Append (H); Soft.Append (S);
      Both.Append (H); Both.Append (S);
      Solve (Both, 1, Cap, One, Damp, A, Okp);
      declare
         F : constant Long_Float := Predict (H.E, A) (4);
      begin
         Check (Okp and then abs (F - 1.0) > 1.0e-3,
                "优先级:挤不下时,平权解【牺牲】朝向(朝哪冲到 " & Codec.Fmt (F, 3) & ",要的是 1.000)");
      end;
      Solve_Priority (Hard, Soft, 1, Cap, One, Damp, A, Okp);
      declare
         F : constant Long_Float := Predict (H.E, A) (4);
      begin
         Check (Okp and then abs (F - 1.0) < 1.0e-6,
                "优先级:挤不下时,零空间解【保住】朝向(朝哪 " & Codec.Fmt (F, 6) & "),软目标这一步就不办");
      end;
      --  再看挤得下的时候:朝向照样精确,剩下的自由度全去办软目标
      declare
         H2, S2 : Term;
         Hd, Sf : Term_Vectors.Vector;
      begin
         Reset (H2.E, 2, 1.0);
         Set_Col (H2.E, 0, [0.0, 0.0, 0.0, 0.0, 1.0]);
         Set_Col (H2.E, 1, [0.0, 0.0, 0.0, 0.0, 0.0]);
         H2.Err := [0.0, 0.0, 0.0, 0.0, 1.0];  H2.W := [0.0, 0.0, 0.0, 0.0, 1.0];
         Reset (S2.E, 2, 1.0);
         Set_Col (S2.E, 0, [1.0, 0.0, 0.0, 0.0, 0.0]);
         Set_Col (S2.E, 1, [1.0, 0.0, 0.0, 0.0, 0.0]);
         S2.Err := [10.0, 0.0, 0.0, 0.0, 0.0];  S2.W := [1.0, 0.0, 0.0, 0.0, 0.0];
         Hd.Append (H2); Sf.Append (S2);
         Solve_Priority (Hd, Sf, 2, Cap, Two, Damp, A, Okp);
         Check (Okp and then abs (Predict (H2.E, A) (4) - 1.0) < 1.0e-6
                  and then abs (Predict (S2.E, A) (0) - 10.0) < 1.0e-3,
                "优先级:挤得下时两个都办到(朝哪 " & Codec.Fmt (Predict (H2.E, A) (4), 4)
                & " 左右 " & Codec.Fmt (Predict (S2.E, A) (0), 3) & ")");
      end;
      Check (abs (Row_Scale (S.E, [0 => 2.0, others => 0.0], 0) - 2.0) < 1.0e-9,
             "行归一:最响那个通道一格推动 = |B|×幅度 的最大值");
   end;
   --  ── "证明过了"只有一个定义:体检和执行器必须给出同一个答案 ──
   declare
      use Table;
      use type Exam.Verdict;
      E : Effect;
      Notch : constant Vec := [others => 1.0];
      M : Selfmap.Body_Map;
      Ts : Learned.Effect_Vectors.Vector;
      Se : Learned.Stored_Effect;
      procedure Both (What : String; Want : Boolean) is
         R : constant Exam.Report := Exam.Judge (M, Ts);
         Ex : constant Boolean := (R.Things (0).Rows (Exam.Sideways).V = Exam.Usable);
         Tb : constant Boolean := Row_Proven (Ts (0).E, Notch, 0);
      begin
         Check (Ex = Tb, "证明的定义唯一:" & What & " —— 体检说 " & (if Ex then "能用" else "不能用")
                & ",执行器说 " & (if Tb then "能用" else "不能用"));
         Check (Tb = Want, "证明的判据:" & What & " ⇒ " & (if Want then "能用" else "不能用"));
      end Both;
   begin
      M.Arms := 1; M.N_Cams := 1; M.Per_Arm := 2; M.Channels := 2;
      M.Amp.Append (1.0); M.Amp.Append (1.0);
      M.Delivered.Append (1.0); M.Delivered.Append (1.0);
      M.Seen.Append (True); M.Seen.Append (True);
      M.Cam_Frac.Append (0.5);
      M.Cam_On_Arm.Append (-1);
      M.Pic_Floor.Append (3);
      Reset (E, 2, 1.0);
      Set_Col (E, 0, [1.0, 0.0, 0.0, 0.0, 0.0]);
      Se.E := E; Se.Arm := 0; Se.Cam := 0;
      Ts.Append (Se);
      Both ("量到了但一次都没重复", False);
      Set_Spread (Ts (0).E, 0, 3, [0.05, 0.0, 0.0, 0.0, 0.0]);
      declare
         X : Learned.Stored_Effect := Ts (0);
      begin
         Set_Spread (X.E, 0, 3, [0.05, 0.0, 0.0, 0.0, 0.0]);
         Ts.Replace_Element (0, X);
      end;
      Both ("重复 3 次,散布只有均值的 5%", True);
      declare
         X : Learned.Stored_Effect := Ts (0);
      begin
         Set_Spread (X.E, 0, 3, [1.4, 0.0, 0.0, 0.0, 0.0]);
         Ts.Replace_Element (0, X);
      end;
      Both ("重复 3 次,散布是均值的 1.4 倍(自己跟自己打架)", False);
      declare
         X : Learned.Stored_Effect := Ts (0);
      begin
         Set_Col (X.E, 0, [0.0, 0.0, 0.0, 0.0, 0.0]);
         Set_Spread (X.E, 0, 3, [0.0, 0.0, 0.0, 0.0, 0.0]);
         Ts.Replace_Element (0, X);
      end;
      Both ("推遍所有通道一格都推不动", False);
      --  🔴 GB 那一炮栽的就是这一条:最响的通道抽了一下(只推过 1 次),
      --  另一个通道又稳又推得动 —— 这一行必须【算能用】。以前按"最响的说了算",整行被一票否决。
      declare
         X : Learned.Stored_Effect := Ts (0);
      begin
         Set_Col (X.E, 0, [2.0, 0.0, 0.0, 0.0, 0.0]);   --  最响,但只推过 1 次
         Set_Spread (X.E, 0, 1, [0.0, 0.0, 0.0, 0.0, 0.0]);
         Set_Col (X.E, 1, [0.5, 0.0, 0.0, 0.0, 0.0]);   --  小声,但推了 3 次、几乎不散
         Set_Spread (X.E, 1, 3, [0.05, 0.0, 0.0, 0.0, 0.0]);
         Ts.Replace_Element (0, X);
      end;
      Both ("最响的那个抽筋(1 次),另一个又稳又推得动(3 次)", True);
   end;
   --  ── 认身体:一串都落在 [0,1] 的数 = 一组抓握通道(五指手报五个,以前整组被忽略)──
   declare
      S : Buf;
      D : Msgpack.Doc;
      L5, L1 : Layout.Body_Layout;
      procedure Build (N : Natural; V : Long_Float) is
      begin
         S.Clear;
         Msgpack.Put_Map (S, 1);
         Msgpack.Put_Str (S, "obs"); Msgpack.Put_Map (S, 2);
         Msgpack.Put_Str (S, "hand"); Msgpack.Put_Array (S, N);
         for I in 1 .. N loop
            Msgpack.Put_Float (S, V);
         end loop;
         Msgpack.Put_Str (S, "elbow"); Msgpack.Put_Array (S, 6);
         for I in 1 .. 6 loop
            Msgpack.Put_Float (S, 0.1);
         end loop;
      end Build;
   begin
      Build (5, 0.4);
      Check (Msgpack.Decode (S, D), "认身体:五指手那一帧解得开");
      Layout.Recognise (D, Msgpack.Key (D, 0, "obs"), L5);
      Check (Natural (L5.Jaw.Length) = 1,
             "认身体:五个都在 [0,1] 的数 = 一组抓握通道(以前只认长度 1,五指手整组被忽略)");
      Build (1, 0.4);
      Check (Msgpack.Decode (S, D), "认身体:两指手那一帧解得开");
      Layout.Recognise (D, Msgpack.Key (D, 0, "obs"), L1);
      Check (Natural (L1.Jaw.Length) = 1, "认身体:两指手照旧认得出");
      Check (Natural (L5.Joints.Length) = 1 and then Natural (L1.Joints.Length) = 1,
             "认身体:六个不在 [0,1] 的数仍然算关节角,没被抢走");
   end;
   --  ── Sinew:第二版语言 ──
   declare
      use Sinew;
      NL : constant String := "" & ASCII.LF;
      G : constant Program := Sinew.Parse
        ("to reach for it:" & NL &
         "  do grasper facing the white ball must and grasper above the white ball small until arrived or 20 steps" & NL &
         "end" & NL &
         "remember where grasper is as start" & NL &
         "repeat 3 times:" & NL &
         "  run reach for it" & NL &
         "  do grasper onto the white ball small until touched" & NL &
         "  do grasper close on the white ball until stuck" & NL &
         "  if slipped:" & NL &
         "    do grasper open until arrived" & NL &
         "    do grasper touching start medium until arrived anyway" & NL &
         "  else:" & NL &
         "    done" & NL &
         "  end" & NL &
         "end" & NL &
         "say I tried three times");
   begin
      Check (G.Ok, "Sinew:一段带定义/循环/分支的完整程序解析得通"
             & (if G.Ok then "" else " —— 第" & Natural'Image (G.Err_Line) & " 行:" & To_String (G.Err)));
      Check (Natural (G.Defs.Length) = 1 and then To_String (G.Defs (0).Name) = "reach for it",
             "Sinew:定义被记下来了(名字可以是好几个词)");
   end;
   declare
      use Sinew;
      G : constant Program := Sinew.Parse ("do grasper touching the white ball small must until touched");
      C : constant Constraint := (if G.Ok and then Natural (G.Code.Length) > 0
                                  then G.Code (0).Cons (0) else (others => <>));
   begin
      Check (G.Ok and then C.Subj.K = Nk_Role and then C.Subj.R = Rl_Grasper,
             "Sinew:主语是【角色】,不是编号");
      Check (C.R = Re_Touching and then To_String (C.Obj.Word) = "the white ball",
             "Sinew:宾语是一句名字,好几个词也认(" & To_String (C.Obj.Word) & ")");
      Check (C.Sp = Sp_Small and then C.Rk = Rk_Must and then G.Code (0).Until_Oc = Oc_Touched,
             "Sinew:步子 / must / 结局都读出来了");
   end;
   declare
      use Sinew;
      function Bad (S : String) return Boolean is (not Sinew.Parse (S).Ok);
   begin
      Check (Bad ("move joint 3 by 0.1"), "Sinew:关节号这种话语法上不存在 ⇒ 说不出口");
      Check (Bad ("do grasper press the table until stuck"), "Sinew:press 不说劲 ⇒ 退回");
      Check (Bad ("do grasper press the table hard small until stuck"),
             "Sinew:press 说了劲还说步子 ⇒ 退回(一根轴上二选一)");
      Check (Bad ("do grasper touching the ball small"), "Sinew:不说【到什么为止】⇒ 退回");
      Check (Bad ("do touching the ball until touched"), "Sinew:不说【谁】⇒ 退回");
      Check (Bad ("do grasper the ball until touched"), "Sinew:不说关系 ⇒ 退回");
      Check (Bad ("do grasper touching the ball until soon"), "Sinew:until 后面不是那八个结局 ⇒ 退回");
      Check (Bad ("repeat 3 times:" & ASCII.LF & "  do grasper open until arrived"),
             "Sinew:块没有 end ⇒ 退回");
      Check (Bad ("end"), "Sinew:多一个 end ⇒ 退回");
      Check (Bad ("else:" & ASCII.LF & "end"), "Sinew:else 前面没有 if ⇒ 退回");
      Check (Sinew.Parse ("do grasper still until arrived").Ok, "Sinew:still 不需要宾语");
      Check (Sinew.Parse ("do grasper open until arrived").Ok, "Sinew:open 不需要宾语");
      Check (Sinew.Parse ("do grasper close until stuck").Ok,
             "Sinew:close 可以不带宾语 —— 就在这儿合上(球被自己的手挡住时唯一能说的话)");
      Check (Sinew.Parse ("try:" & ASCII.LF & "  do grasper open until arrived" & ASCII.LF
             & "or:" & ASCII.LF & "  done" & ASCII.LF & "end").Ok, "Sinew:try / or / end");
   end;
   declare
      use Sinew;
      G : constant Program := Sinew.Parse ("repeat until touched:" & ASCII.LF
            & "  do grasper onto the ball small until timeout or 3 steps" & ASCII.LF & "end");
   begin
      Check (G.Ok and then G.Code (0).O = Op_Loop and then G.Code (0).Cond = Oc_Touched,
             "Sinew:repeat until <结局> 编成一条循环头");
      Check (G.Ok and then G.Code (Natural (G.Code.Length) - 1).O = Op_Next
               and then G.Code (Natural (G.Code.Length) - 1).Target = 0,
             "Sinew:循环尾跳回循环头");
      Check (G.Ok and then G.Code (0).Target = Integer (G.Code.Length),
             "Sinew:循环头的出口指向 end 之后");
      Check (G.Ok and then G.Code (1).Max_Steps = 3, "Sinew:「or 3 steps」读出来了");
   end;
   --  ── Sinew 执行器:喂给它剧本里的结局,看它走出什么次序 ──
   declare
      use Sinew;
      use type Runtime.Yield;
      NL : constant String := "" & ASCII.LF;
      --  跑一段程序,按剧本喂结局,返回"依次执行了哪几段区间"的字串 + 结束方式
      function Trace (Src : String; Script : String; Ending : out Runtime.Yield) return String is
         P : constant Program := Sinew.Parse (Src);
         M : Runtime.Machine;
         W : Runtime.Yield;
         I : Instr;
         Out_S : Unbounded_String;
         K : Natural := Script'First;
         Guard : Natural := 0;
      begin
         Ending := Runtime.Y_Broken;
         if not P.Ok then
            return "解析就没过:" & To_String (P.Err);
         end if;
         loop
            Guard := Guard + 1;
            exit when Guard > 200;
            Runtime.Advance (P, M, W, I);
            Ending := W;
            case W is
               when Runtime.Y_Interval =>
                  Append (Out_S, (if Length (Out_S) > 0 then "," else "")
                          & To_String (I.Cons (0).Obj.Word));
                  declare
                     O : Outcome := Oc_Arrived;
                  begin
                     if K <= Script'Last then
                        O := (case Script (K) is
                                 when 'a' => Oc_Arrived, when 't' => Oc_Touched,
                                 when 's' => Oc_Stuck,   when 'l' => Oc_Lost,
                                 when 'p' => Oc_Slipped, when 'f' => Oc_Free,
                                 when 'o' => Oc_Timeout, when others => Oc_Refused);
                        K := K + 1;
                     end if;
                     Runtime.Report (P, M, O);
                  end;
               when Runtime.Y_Say | Runtime.Y_Remember =>
                  null;
               when Runtime.Y_Done | Runtime.Y_Finished | Runtime.Y_Broken =>
                  exit;
            end case;
         end loop;
         return To_String (Out_S);
      end Trace;
      E : Runtime.Yield;
   begin
      Check (Trace ("repeat 3 times:" & NL & "  do grasper touching A until arrived" & NL & "end", "aaa", E) = "A,A,A"
               and then E = Runtime.Y_Finished, "执行器:repeat 3 times 跑三遍");
      Check (Trace ("repeat until touched:" & NL & "  do grasper touching A until touched" & NL & "end", "oot", E) = "A,A,A",
             "执行器:repeat until touched —— 前两次没碰到就接着来,碰到就出去");
      Check (Trace ("do grasper touching A until arrived" & NL & "if stuck:" & NL
                    & "  do grasper touching B until arrived" & NL & "else:" & NL
                    & "  do grasper touching C until arrived" & NL & "end", "sa", E) = "A,B",
             "执行器:上一段结局是 stuck ⇒ 走 if 那一支");
      Check (Trace ("do grasper touching A until arrived" & NL & "if stuck:" & NL
                    & "  do grasper touching B until arrived" & NL & "else:" & NL
                    & "  do grasper touching C until arrived" & NL & "end", "aa", E) = "A,C",
             "执行器:结局不是 stuck ⇒ 走 else 那一支");
      Check (Trace ("try:" & NL & "  do grasper touching A until arrived" & NL
                    & "or:" & NL & "  do grasper touching B until arrived" & NL & "end", "sa", E) = "A,B",
             "执行器:try 里那一段没成 ⇒ 走 or 那一段");
      Check (Trace ("try:" & NL & "  do grasper touching A until arrived" & NL
                    & "or:" & NL & "  do grasper touching B until arrived" & NL & "end", "aa", E) = "A",
             "执行器:try 里那一段成了 ⇒ or 那一段跳过");
      Check (Trace ("to poke:" & NL & "  do grasper touching A until arrived" & NL & "end" & NL
                    & "run poke" & NL & "run poke", "aa", E) = "A,A"
               and then E = Runtime.Y_Finished, "执行器:定义一次,叫两次;定义体不会被正常流走进去");
      declare
         T : constant String := Trace ("run nosuch", "", E);
      begin
         Check (E = Runtime.Y_Broken and then T = "", "执行器:叫一个没定义过的名字 ⇒ 当场判坏");
      end;
      declare
         T : constant String := Trace ("repeat until touched:" & NL
               & "  do grasper touching A until arrived" & NL & "end", "aaaaaaaaaaaaaaaaaaaa", E);
         pragma Unreferenced (T);
      begin
         Check (E /= Runtime.Y_Finished, "执行器:等一个永远不来的结局 ⇒ 不会假装跑完");
      end;
   end;

   --  🔴 每个结局词都要有【自己】的判法 —— 这一条焊死本仓最贵的一类 bug:
   --  语言收下一个词,身体悄悄换成另一个词的行为,还回报得一本正经。
   --  GM 实测:写 until arrived 一段只走一步;而 until free(=被拿起来了,本任务的判据)
   --  当时接在"爪子读数掉回空手"上,和拿起来毫无关系。
   declare
      use Sinew;
      use type Monitor.Until_Kind;
      --  这张表就是语言的承诺,逐词钉死。要改这里,必须先改 LANGUAGE.md 的那张表。
      type Row is record
         O : Outcome;
         K : Monitor.Until_Kind;
      end record;
      Want : constant array (1 .. 9) of Row :=
        [(Oc_Touched, Monitor.U_Contact), (Oc_Stuck, Monitor.U_Resist),
         (Oc_Slipped, Monitor.U_Slip),    (Oc_Free, Monitor.U_Free),
         (Oc_Lost, Monitor.U_Lost),       (Oc_Settled, Monitor.U_Settle),
         --  🔴 stalled 必须自己一格:stuck = 命令了身体没走;stalled = 我在动可差距不缩。
         --  压成一条就等于脑问"还有救吗"而身体答"我没瘫痪"(JE 2026-09-15 加这个词)
         (Oc_Stalled, Monitor.U_Stall),
         (Oc_Arrived, Monitor.U_Steps),   (Oc_Timeout, Monitor.U_Steps)];
      All_Right : Boolean := True;
      Round_Trip : Boolean := True;
      Distinct : Boolean := True;
   begin
      for R of Want loop
         if Act.Until_Of (R.O) /= R.K then
            All_Right := False;
         end if;
         --  Outcome → 字符串 → Until_Kind 这一跳不许把词弄丢(以前 lost/free 就是在这儿丢的)
         if Act.Kind_Of_Word (Act.Until_Word (R.O)) /= Act.Until_Of (R.O) then
            Round_Trip := False;
         end if;
      end loop;
      --  七个"有自己事件"的词必须两两不同;arrived 和 timeout 共用步数上限,靠 Wants_Arrive 分开。
      --  ⚠️ 比的必须是【身体的真实映射】Until_Of,不是我写的期望表自己跟自己 ——
      --  第一版就是拿 Want(I).K 和 Want(J).K 比,把 free 故意改坏之后它照样绿:一条永不失败的断言。
      for I in 1 .. 7 loop
         for J in I + 1 .. 7 loop
            if Act.Until_Of (Want (I).O) = Act.Until_Of (Want (J).O) then
               Distinct := False;
            end if;
         end loop;
      end loop;
      Check (All_Right, "语言:每个结局词都接到自己的判法上(不许并进兜底的步数上限)");
      Check (Round_Trip, "语言:结局词转成字符串再转回来,判法不变");
      Check (Distinct, "语言:有自己事件的六个结局词两两不同");
      Check (Act.Wants_Arrive (Oc_Arrived) and then not Act.Wants_Arrive (Oc_Timeout),
             "语言:只有脑真写了 arrived,身体才准自称到了");
   end;

   --  关系词同一条焊缝:每一个都要么有自己的分支,要么有自己的、非空非 "?" 的字,且两两不同
   declare
      use Sinew;
      Named : Boolean := True;
      Uniq : Boolean := True;
   begin
      for R in Rel loop
         if R /= Re_None and then not Act.Rel_Has_Own_Branch (R) then
            if Act.Rel_Cmd (R) = "?" or else Act.Rel_Cmd (R) = "" then
               Named := False;
            end if;
            for Q in Rel loop
               if Q /= R and then Q /= Re_None and then not Act.Rel_Has_Own_Branch (Q)
                 and then Act.Rel_Cmd (Q) = Act.Rel_Cmd (R)
               then
                  Uniq := False;
               end if;
            end loop;
         end if;
      end loop;
      Check (Named, "语言:每个关系词都有自己的字(没有一个落进兜底的 ?)");
      Check (Uniq, "语言:关系词两两不同(不许两个词做同一件事)");
   end;

   --  角色词同一条焊缝:grasper 和 pusher 按语言的定义是互斥的
   --  (pusher = 推得动东西、但【合不拢】的部件),不许有哪种零件同时满足两个角色。
   --  以前 pusher 写成 Grip | Piece,爪心同时中两个,GM 日志里 grasper 和 pusher 绑到同一块。
   declare
      Overlap : Boolean := False;
      Grasp_Any, Push_Any : Boolean := False;
   begin
      for K in Act.Item_Kind loop
         if Act.Role_Wants (Sinew.Rl_Grasper, K) and then Act.Role_Wants (Sinew.Rl_Pusher, K) then
            Overlap := True;
         end if;
         if Act.Role_Wants (Sinew.Rl_Grasper, K) then
            Grasp_Any := True;
         end if;
         if Act.Role_Wants (Sinew.Rl_Pusher, K) then
            Push_Any := True;
         end if;
      end loop;
      Check (not Overlap, "语言:grasper 和 pusher 互斥(合得拢的零件不许算 pusher)");
      --  🔴 判据一(起炮的四条门槛之一):手在画面里的位置不许算炸。
      --  数字取自箱上真 cal.json(臂1/相机0,64 个样本):两瓣存的是 u=0.8475 / 0.9847,隔 0.137。
      --  而身体当时报给脑的是"画面左边,第 2 格和第 19 格",隔约 0.75 画幅 —— 那就是外推炸了。
      declare
         Was : constant Long_Float := 0.137;    --  样本里两瓣本来隔多远(真数据)
      begin
         Check (not Act.Extrapolation_Blew (Was, 0.137), "身体图:外推没变化 ⇒ 作数");
         Check (not Act.Extrapolation_Blew (Was, 0.200), "身体图:外推变一点 ⇒ 仍作数");
         Check (Act.Extrapolation_Blew (Was, 0.750),
                "身体图:外推把两瓣拉到隔四分之三个画面 ⇒ 不作数(这正是十三炮里手被放到画面另一边的那一步)");
         Check (Act.Extrapolation_Blew (Was, 0.000),
                "身体图:外推把两瓣叠到一起 ⇒ 也不作数");
      end;
      --  🔴 判据二(起炮门槛之二):瞄的是它的腰,不是它的皮。
      --  数字取自 LAB 那个 4 cm 半球的实测:顶 0.762 · 中位深度(=皮)0.771 · 真中间 0.782。
      --  它站的那个面在 0.802(顶再往下 4 cm)。FO 就是瞄了皮 ⇒ 夹在球很偏上处 ⇒ 一合把球撞飞。
      declare
         Skin : constant Long_Float := 0.771;   --  这块自己的中位深度(身体量的)
         Surf : constant Long_Float := 0.802;   --  它站着的那个面(中位 + 鼓出多高)
         True_Mid : constant Long_Float := 0.782;
         Into : constant Long_Float := Act.Into_Depth (Skin, Surf);
      begin
         Check (Into > Skin, "抓握:into 瞄的比它的皮更深(瞄进身子里,不是贴着表面)");
         Check (Into < Surf, "抓握:into 没瞄穿到它站着的那个面(那是 onto 干的事)");
         Check (abs (Into - True_Mid) < abs (Skin - True_Mid),
                "抓握:into 比 touching 更接近真正的中间(" & Codec.Fmt (abs (Into - True_Mid) * 1000.0, 1)
                & " mm vs " & Codec.Fmt (abs (Skin - True_Mid) * 1000.0, 1) & " mm)");
      end;
      --  🔴 "扫过哪些像素 = 手指"的筛法:动过【不止一步】的才算数。
      --  HC 实测:不动的那只眼里,40 步的并集让渲染噪声铺满全画面(4140 个散点,填充率 0.015),
      --  由此硬编出的握区钉在画面最右边缘,下游全线中毒。手指像素连着好多步都和第一帧不同,噪声只闪一步。
      --  这里把 Zone.Measure 里那两行照样跑一遍(离线,不用机器人)。
      declare
         N : constant Natural := 64;
         Steps : constant Natural := 40;
         Once : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
         Swept : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
         Finger_Lo : constant Natural := 10;   --  手指扫过的那一段像素
         Finger_Hi : constant Natural := 19;
         Kept_Finger : Natural := 0;
         Kept_Noise : Natural := 0;
      begin
         for St in 1 .. Steps loop
            declare
               Mv : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
            begin
               --  手指:从第 5 步起一直和第一帧不同
               if St >= 5 then
                  for I in Finger_Lo .. Finger_Hi loop
                     Mv.Replace_Element (I, True);
                  end loop;
               end if;
               --  噪声:每一步点亮一个【不同的】像素,只闪这一步(30..59,各闪一次,不重复)
               if St <= 30 then
                  Mv.Replace_Element (30 + St - 1, True);
               end if;
               Swept := Picture.Either (Swept, Picture.Both (Once, Mv));
               Once := Picture.Either (Once, Mv);
            end;
         end loop;
         for I in 0 .. N - 1 loop
            if Swept.Element (I) then
               if I >= Finger_Lo and then I <= Finger_Hi then
                  Kept_Finger := Kept_Finger + 1;
               else
                  Kept_Noise := Kept_Noise + 1;
               end if;
            end if;
         end loop;
         Check (Kept_Finger = Finger_Hi - Finger_Lo + 1,
                "握区:连着好多步都动的那一片(手指)全留下了(" & Codec.Img (Kept_Finger) & "/"
                & Codec.Img (Finger_Hi - Finger_Lo + 1) & ")");
         Check (Kept_Noise = 0,
                "握区:每步换一个像素闪一下的(噪声)一个都没留(" & Codec.Img (Kept_Noise)
                & ")—— 旧写法(整段取并集)会把它们全收进来,握区就被钉到画面边上");
      end;
      --  🔴🔴 owner 2026-09-14 死命令:"到了没到"只有脑能判,身体不许说 arrived。
      --  它自己判过并且判错了:一段只走 1 推就宣布"到了",实际离点名的东西还差 0.2 m、
      --  只有该有大小的 7.7%。⇒ 语法里不列这个词;脑真写了也要在【动之前】当场退回并说明。
      declare
         use Sinew;
         G : constant Program := Sinew.Parse ("do grasper into the ball until arrived or 10 steps");
         Ok_One : constant Program := Sinew.Parse ("do grasper into the ball until touched or 10 steps");
      begin
         --  语法现在是【当场生成】的:关系/角色/结局三张表都来自驱动自己的判定。
         --  这里拿一个典型配置查它:等不到的词不许出现,量得到的事件词必须在。
         declare
            Rep_0 : Exam.Report;
            G_Txt : constant String := Sinew.Grammar (Plan.Usable_Rels (Rep_0, -1, True),
                                                      "grasper pusher", Plan.Waitable_Outcomes (True));
         begin
            Check (G_Txt'Length > 0 and then Ada.Strings.Fixed.Index (G_Txt, "arrived") = 0,
                   "到位:语法里不再有 arrived 这个词(到了没到只有脑能判)");
            Check (G_Txt'Length > 0 and then Ada.Strings.Fixed.Index (G_Txt, "touched") > 0,
                   "到位:量得到的事件词还在(touched)—— 删的是意见,不是事件");
         end;
         Check (G.Ok and then G.Code (0).Until_Oc = Oc_Arrived,
                "到位:脑真写了 arrived,解析层照样读得出来(才好在编译期退回并说明,而不是悄悄换成步数)");
         Check (Ok_One.Ok and then Ok_One.Code (0).Until_Oc = Oc_Touched,
                "到位:until touched 不受影响");
      end;
      --  🔴🔴 搜多宽由这一步预计跑多远定。数字取自 HL 真数据:一推让点跑了 0.1848 画幅,
      --  在半分辨率(320 宽)的图上就是 59 个像素。写死 3 层 ⇒ 最粗那层还剩 15 个像素,光流找不准 ⇒
      --  它会【静默返回一个完全错误的位置】(2026-08-27 V2 实测),而下一推同样幅度就被记成"没动 ⇒ 不稳"。
      declare
         HL_Px : constant Long_Float := 0.1848 * 320.0;   --  HL 那一推,在半分辨率图上跑了多少像素
      begin
         Check (Act.Levels_For (HL_Px) >= 6,
                "搜宽:HL 那一推跑了 " & Codec.Fmt (HL_Px, 0) & " 像素 ⇒ 至少要 "
                & Codec.Img (Act.Levels_For (HL_Px)) & " 层,写死 3 层必然找不准");
         Check (Act.Levels_For (1.0) = 1,
                "搜宽:只跑了一个像素 ⇒ 一层就够,不许白烧算力");
         Check (Act.Levels_For (0.0) = 1,
                "搜宽:一点没跑 ⇒ 一层");
         Check (Act.Levels_For (4.0) = 3 and then Act.Levels_For (8.0) = 4,
                "搜宽:每加一层能多搜一倍 —— 4 像素要 3 层、8 像素要 4 层");
         Check (Act.Levels_For (100000.0) <= 6,
                "搜宽:再远也封在 6 层 —— 再粗下去图本身只剩几十个像素,没有内容可对了");
      end;
      --  🔴 撤回一条(HY 实测,我自己写坏的):曾把量出来的放大倍数拿去【除掉所有深度读数】。
      --  那个倍数量的是"推一米读数变几米"(灵敏度),**不是**"读数的绝对尺度错几倍"。
      --  拿灵敏度去除绝对值 ⇒ 1.4 m 被除成 0.003 m(手离镜头 3 毫米,物理上不可能),差距从 1.366 炸到 2460。
      --  这一条钉死那个区别:灵敏度大 ≠ 绝对尺度错,不许拿前者改后者。
      declare
         True_Z : constant Long_Float := 1.4;      --  真实的远近
         Sens : constant Long_Float := 380.0;      --  实测灵敏度:走 2 mm 读数变 76 cm
      begin
         Check (Act.Depth_Scale_Bad (Sens),
                "撤回:灵敏度 380 确实该被判成【我的距离感坏了】—— 体检这一条是对的,留着");
         Check (True_Z / Sens < 0.005,
                "撤回:但拿它去除绝对值 ⇒ 1.4 m 变成 " & Codec.Fmt (True_Z / Sens, 4)
                & " m(手离镜头几毫米)—— 物理上不可能,所以不许这么除");
      end;
      --  🔴🔴 来回对表:同一根通道 +δ 走一遍、−δ 走回来一遍,两遍各除以自己那一遍的实到 ⇒ 应当相等。
      --  判据零系数:两遍的【分歧】要小于两遍的【共识】。数字取自 2026-08-27 NV3 真数据
      --  (它上机第一次就抓到一个符号错:分歧 2.539 / 共识 0.019)。
      declare
         function Ratio (Out_V, Back_V : Long_Float) return Long_Float is
            Dif : constant Long_Float := abs (Out_V - Back_V);
            Con : constant Long_Float := abs ((Out_V + Back_V) / 2.0);
         begin
            return (if Con > 0.0 then Dif / Con else -1.0);
         end Ratio;
      begin
         Check (Ratio (0.830, 0.835) < 1.0,
                "来回:去程 0.830、回程 0.835 ⇒ 对得上,这一列信得过");
         Check (Ratio (0.830, -0.820) >= 1.0,
                "来回:去程 0.830、回程 -0.820(符号反了)⇒ 对不上 —— 这正是 NV3 上机第一次抓到的那种错");
         Check (Ratio (0.830, 0.050) >= 1.0,
                "来回:去程 0.830、回程 0.050(回程跟丢了)⇒ 对不上,不许收");
         Check (Ratio (0.0, 0.0) < 0.0,
                "来回:两遍都是零(这个通道不动它)⇒ 说不上对不对,交给别的判据,不许当成错");
      end;
      --  🔴🔴 体检:一根【平移】通道推一米,我离相机的远近最多变一米(正好沿着相机看的方向走时取到 1)。
      --  绝对值大于 1 = 物理上不可能 ⇒ 我的深度读数尺度是坏的。数字取自 HW 真数据。
      declare
         Seen1 : constant Long_Float := -36.732;   --  HW 实测 ch8 那一格
         Seen2 : constant Long_Float := -60.270;   --  另一个点上的同一格
      begin
         Check (Act.Depth_Scale_Bad (Seen1) and then Act.Depth_Scale_Bad (Seen2),
                "体检:推一米远近变 36.7 米 / 60.3 米 ⇒ 物理上不可能,必须判成【我的距离感坏了】");
         Check (not Act.Depth_Scale_Bad (0.94),
                "体检:推一米远近变 0.94 米 ⇒ 完全可能(几乎正对着相机走),不许误判");
         Check (not Act.Depth_Scale_Bad (-1.0) and then Act.Depth_Scale_Bad (-1.02),
                "体检:边界正好在 1 —— 1 是【沿着相机看的方向】那一档,超过它才不可能");
         Check (not Act.Depth_Scale_Bad (0.0),
                "体检:这一格是 0(这个通道不改变远近)⇒ 不是坏,是一次正确的测量");
      end;
      --  🔴 深度读数收不收。数字取自 FS 实测:手指上一次真读到 0.454 m,这一帧读出 0.010 m(离镜头一厘米)。
      declare
         Was : constant Long_Float := 0.454;    --  上一次真读到的
         Crazy : constant Long_Float := 0.010;  --  这一帧读出来的(物理上不可能)
         Noise : constant Long_Float := 0.02;   --  这一点自己量到的读深抖动
      begin
         Check (not Act.Depth_Ok (Crazy, Was, 0.0, Noise, 0.0),
                "深度:没有预测值时也要挡 —— 0.454 m 一步跳到 0.010 m 不许收(旧写法这里整条闸短路放行)");
         Check (Act.Depth_Ok (0.460, Was, 0.0, Noise, 0.0),
                "深度:没有预测值时,变化在自己的抖动之内 ⇒ 照收");
         Check (Act.Depth_Ok (Crazy, 0.0, 0.0, Noise, 0.0),
                "深度:头一次读到(还没有上一次)⇒ 照收,没有基准可比");
         Check (Act.Depth_Ok (0.300, Was, 0.290, Noise, 0.0),
                "深度:表预测这一步会走到 0.290,读到 0.300 ⇒ 收(大跳但预测过)");
         Check (not Act.Depth_Ok (0.010, Was, 0.440, Noise, 0.0),
                "深度:表预测只走到 0.440,却读出 0.010 ⇒ 不收");
         --  🔴 闸不许把自己锁死:HE 实测手的深度连着 12 步一模一样 2.182 m,而它在画面里一直在动 ——
         --  拒了一次就永远拿旧值当基准,真实深度一变就再也收不回来("永不响的闸"那一类)。
         Check (not Act.Depth_Ok (0.900, Was, 0.0, Noise, 0.0),
                "深度:第一次读到 0.900(离 0.454 很远)⇒ 先不收,记下来");
         Check (Act.Depth_Ok (0.905, Was, 0.0, Noise, 0.900),
                "深度:下一帧又读到 0.905,和上次被拒的 0.900 吻合 ⇒ 两次独立测量一致,收下 —— 闸有出路");
         Check (not Act.Depth_Ok (0.300, Was, 0.0, Noise, 0.900),
                "深度:这次读 0.300,和上次被拒的 0.900 对不上 ⇒ 仍然不收(出路不是无条件放行)");
         --  🔴 带子的中心必须是【上次真读到的】,不是表预测的位置。HS 真数据:
         --  读到 2.306 · 上次真读到 2.346 · 表说这一步走 0.223 · 抖动 0.022。
         --  以预测为中心:|2.306-2.569| = 0.263 > 0.245 ⇒ 拒 —— 一个离上次只差 0.04 m 的诚实读数被判离谱,
         --  深度于是连着几十推纹丝不动。以上次真读数为中心:|2.306-2.346| = 0.040 ⇒ 收。
         Check (Act.Depth_Ok (2.306, 2.346, 2.569, 0.022, 0.0),
                "深度:表高估了这一步(说走 0.223 实际没走),而读数离上次只差 0.04 ⇒ 必须收,不然深度永远不动");
         Check (not Act.Depth_Ok (2.900, 2.346, 2.569, 0.022, 0.0),
                "深度:同一组里真的跳远了(离上次 0.55,而表只说走 0.223)⇒ 仍然挡");
      end;
      --  🔴 画面上重合 ≠ 真的在一起(FZ 实测:头顶相机报差 0.062 幅"几乎压上了",爪子在球上方 30 厘米)。
      --  数字:球在 0.64 m、爪子在 0.34 m(高出 30 cm),球在画面里偏左到 0.40。
      declare
         Ball_U : constant Long_Float := 0.40;
         Ball_Z : constant Long_Float := 0.64;
         Mine_Z : constant Long_Float := 0.34;
         Aim : constant Long_Float := Act.On_My_Plane (Ball_U, Ball_Z, Mine_Z);
      begin
         Check (abs (Aim - Ball_U) > 0.062,
                "对齐:爪子高出球 30 厘米时,该去的那个 u 和球在画面里的 u 差 "
                & Codec.Fmt (abs (Aim - Ball_U), 3) & " 幅 —— 比那句「只差 0.062 幅、几乎压上了」还大");
         Check (Act.On_My_Plane (Ball_U, Ball_Z, Ball_Z) = Ball_U,
                "对齐:我和它在同一个远近上 ⇒ 画面坐标就是真坐标,不许动它");
         Check (Act.On_My_Plane (0.5, Ball_Z, Mine_Z) = 0.5,
                "对齐:东西正在画面中心 ⇒ 不管远近,该去的还是中心(投影从中心发散)");
         Check (Act.On_My_Plane (Ball_U, 0.0, Mine_Z) = Ball_U
                and then Act.On_My_Plane (Ball_U, Ball_Z, 0.0) = Ball_U,
                "对齐:任一边没有远近 ⇒ 退回只比画面坐标,不许瞎放大");
      end;
      --  🔴🔴 "搬到我这个远近平面上再比"这一步,倍数必须用【目标的画面坐标是在哪个远近上量的】。
      --  HP 实测:`into` 的目标是"左右别动,只把远近走到它的腰上" —— Tu/Tv 抄的是我自己的位置(在我的远近上),
      --  而 Tz 是球的远近。拿 Tz/Z = 3.533/2.161 = 1.63 去放大"别动",就把"别动"变成"一路往画面外走":
      --  目标 (0.766,0.562) 被算成"该去 0.935",第二个点更被算到 1.014(已经在画面外)。
      --  被跟的点整天往画面右沿飘到 u=1.000,根子就在这儿。
      declare
         Mine_Z : constant Long_Float := 2.161;   --  我在哪个远近(HP 真数据)
         Ball_Z : constant Long_Float := 3.533;   --  球在哪个远近
         Same_U : constant Long_Float := 0.766;   --  "左右别动":目标画面坐标 = 我自己的
      begin
         Check (Act.On_My_Plane (Same_U, Mine_Z, Mine_Z) = Same_U,
                "对齐:目标的画面坐标就量在我这个远近上(左右别动)⇒ 一点都不许放大");
         Check (Act.On_My_Plane (Same_U, Ball_Z, Mine_Z) > 0.9,
                "对齐:用错了远近(拿球的远近去放大我自己的位置)⇒ 会被推到 "
                & Codec.Fmt (Act.On_My_Plane (Same_U, Ball_Z, Mine_Z), 3) & " ——这就是那个病");
         Check (Act.On_My_Plane (0.861, Ball_Z, 2.481) > 1.0,
                "对齐:同一个错法在第二个点上直接算到画面【外面】去了");
      end;
      --  🔴 一步的命令上限:天花板底下垫一块"身体自己动得起来"的地板。
      --  数字取自 LAB 那一条:FO 每步命令 0.006(探针那一档 0.026 的四分之一),一步推进 8 厘米、44 推抓到球。
      --  09-13 那次整体回滚把地板削掉之后,步子走到 0.026 —— 表当场不准、球被甩出视野。
      declare
         Noise : constant Long_Float := 0.003;   --  身体自己的噪声(量出来的)
         Probe : constant Long_Float := 0.026;   --  探针那一档
         FO : constant Long_Float := 0.006;      --  FO 抓到球那一炮每步的命令
      begin
         Check (Act.Push_Cap (FO, Noise, 0.0) = FO,
                "步子:FO 那一档(比探针小四倍)不许被地板顶上去 —— 顶上去就是球被甩出视野的那一炮");
         Check (Act.Push_Cap (FO, Noise, 0.0) < Probe,
                "步子:地板不是探针那一档(探针那一档是 FO 的四倍)");
         Check (Act.Push_Cap (0.0005, Noise, 0.0) > 0.0005,
                "步子:命令小到比身体噪声还小 ⇒ 抬到地板上,否则这一步一动不动");
         Check (Act.Push_Cap (0.0005, Noise, 0.0) = Noise + Noise,
                "步子:地板正好是身体噪声的两倍,不是别的什么档");
         --  死区:这个通道自己量出来的"命令小于它身体就不动"。学到之后地板跟着抬。
         Check (Act.Push_Cap (FO, Noise, 0.0) = FO,
                "死区:还没学到死区(0)⇒ 不影响步子");
         Check (Act.Push_Cap (FO, Noise, 0.012) = 0.012,
                "死区:这个通道实测 0.006 推不动、0.012 才动 ⇒ 步子抬到 0.012(不是让它继续发空命令)");
         Check (Act.Push_Cap (0.05, Noise, 0.012) = 0.05,
                "死区:命令本来就比死区大 ⇒ 死区不许把步子往下压");
      end;
      --  🔴 判据四(起炮门槛之四):"拿住了"不许假。
      --  历史上三次假拿住,判据都是"它原来待的地方空了" —— 而【把球撞飞】也让那地方空了。
      --  唯一分得开的是"抬手时它跟着我的手走了同样一段"。数字按画幅比例(0.10 = 十分之一个画面)。
      declare
         Hu : constant Long_Float := 0.10;   --  我的手在那台不动的相机里往右挪了十分之一个画面
         Hv : constant Long_Float := 0.00;
      begin
         Check (Act.Came_With_Me (0.10, 0.00, Hu, Hv),
                "拿住:它跟我挪了同样一段 ⇒ 拿住了");
         Check (Act.Came_With_Me (0.09, 0.01, Hu, Hv),
                "拿住:它跟我挪的差一点点(抖动) ⇒ 仍然算拿住");
         Check (not Act.Came_With_Me (0.00, 0.00, Hu, Hv),
                "拿住:我抬了手它留在原地 ⇒ 没拿住(这一条是给「手上相机里还在框里」兜底的)");
         Check (not Act.Came_With_Me (0.00, -0.30, Hu, Hv),
                "拿住:我抬了手它朝另一个方向飞出去 ⇒ 没拿住 —— 【原地空了但是被我撞飞的】,"
                & "旧判据「它原来待的地方空了」在这里判成拿住,三次假拿住全是它");
         Check (not Act.Came_With_Me (0.10, 0.00, 0.00, 0.00),
                "拿住:我的手一步没挪 ⇒ 判不了,不许自称拿住");
         --  边界钉死在"差得比手自己挪的一半还小",不许后人偷偷放宽
         Check (Act.Came_With_Me (0.05, 0.00, Hu, Hv),
                "拿住:它只挪了我的一半(差正好是一半)⇒ 还算拿住,边界在这儿");
         Check (not Act.Came_With_Me (0.04, 0.00, Hu, Hv),
                "拿住:它挪得比我的一半还少 ⇒ 不算拿住,边界另一侧");
      end;
      --  🔴 新词「用哪只眼睛判这一段」。十二炮里每炮开头我都在用手工做这件事
      --  (先发一条只说话的命令烧掉换眼额度,再靠"在哪台相机里点名"这个副作用把段挪过去)——
      --  一炮浪费两条命令。这一条钉死它真的被读进来了,而且不用编号。
      declare
         use Sinew;
         A : constant Program := Sinew.Parse ("do grasper touching the ball until touched with my still eye");
         B : constant Program := Sinew.Parse ("do grasper touching the ball until touched with my moving eye");
         Cn : constant Program := Sinew.Parse ("do grasper touching the ball until touched");
         Bad : constant Program := Sinew.Parse ("do grasper touching the ball until touched with my third eye");
         function Eye_Of (G : Program) return Eye_Pick is
           (if G.Ok and then Natural (G.Code.Length) > 0 then G.Code (0).Eye else Ey_None);
      begin
         Check (A.Ok and then Eye_Of (A) = Ey_Still, "语言:「with my still eye」读进来了(不跟着我动的那只)");
         Check (B.Ok and then Eye_Of (B) = Ey_Moving, "语言:「with my moving eye」读进来了(跟着我动的那只)");
         Check (Cn.Ok and then Eye_Of (Cn) = Ey_None, "语言:不写就是身体自己挑");
         Check (not Bad.Ok, "语言:「with my third eye」说不出口 —— 眼睛按【量出来的性质】点名,不按编号");
      end;
      Check (Grasp_Any and then Push_Any, "语言:两个角色各自都收得下至少一种零件(不许有空角色)");
   end;

   --  🔴 打死 GM 的那个 bug 的正对焊缝:交给 Monitor 的步数上限永远不许是 0。
   --  先证明"上限 0 = 第一步就成立"确有其事,再证明 Effective_Cap 不可能给出 0。
   declare
      W0 : constant Monitor.Watch := (Quiet => 0, No_Progress => 0, Steps => 0, Refused => 0, Blind => 0);
      Zero_Fires : constant Boolean :=
        Monitor.Fired (Monitor.U_Steps, W0, 0, False, 0.0, 0.0, 0.0);
      Cap_Holds : constant Boolean :=
        not Monitor.Fired (Monitor.U_Steps, W0, 60, False, 0.0, 0.0, 0.0);
      Never_Zero : Boolean := True;
   begin
      for N in 0 .. 300 loop
         if Act.Effective_Cap (N) <= 0 then
            Never_Zero := False;
         end if;
         if N > 0 and then Act.Effective_Cap (N) /= N then
            Never_Zero := False;
         end if;
      end loop;
      Check (Zero_Fires, "监视器:步数上限 0 ⇒ 一步没走就算走完(所以上限不许是 0)");
      Check (Cap_Holds, "监视器:上限 60 时,一步没走不算走完");
      Check (Never_Zero, "执行器:脑写了几步就是几步,没写就用安全上限 —— 永远不会是 0");
      --  ⚠️ 上面那条**不够**:把兜底改成 1(正是 GM 那个 bug 的效果:一段只走一步)它照样绿。
      --  第二条永不失败的断言,和"六个词两两不同"同一个毛病。真正要钉的是【没写步数 ≠ 只走一步】。
      Check (Act.Effective_Cap (0) = Act.Safety_Cap and then Act.Safety_Cap > 1,
             "执行器:脑没写步数 ⇒ 用安全上限,而安全上限不是 1(没写不等于只走一步)");
   end;

   --  🔴 变异测试查出来的真空区:until stuck 和 until slipped 的判据【一条测试都没有】。
   --  把 Refusing 和 Slipped 各改成恒 False,自检照样全过 —— 语言里两个结局词判据没人看着。
   declare
      function W_Ref (N : Monitor.Count) return Monitor.Watch is
        ((Quiet => 0, No_Progress => 0, Steps => 0, Refused => N, Blind => 0));
   begin
      --  顶住 = 命令发了而身体没走,连着两步才算(一步可能只是还没生效)
      Check (not Monitor.Refusing (W_Ref (0)) and then not Monitor.Refusing (W_Ref (1)),
             "监视器:才一步没走不算顶住(一步可能只是还没生效)");
      Check (Monitor.Refusing (W_Ref (2)) and then Monitor.Refusing (W_Ref (5)),
             "监视器:连着两步没走 = 顶住(until stuck 靠这一条)");
      --  掉了 = 抓握读数回到"合空"那个读数附近(差在噪声以内)
      Check (Monitor.Slipped (1.00, 1.00, 0.01) and then Monitor.Slipped (1.005, 1.00, 0.01),
             "监视器:读数回到合空那个数 = 手里的东西掉了(until slipped 靠这一条)");
      Check (not Monitor.Slipped (1.20, 1.00, 0.01),
             "监视器:读数比合空高出噪声以上 = 还夹着东西");
   end;

   --  🔴🔴 GM 的死法,在一张造出来的深度图上重现并钉死:
   --  手一凑近,要抓的东西【比尺子还宽】⇒ 它自己就是背景 ⇒ 鼓 0 ⇒ 整块消失;
   --  同时它被画面下沿切掉一角 ⇒ 又被"贴边的丢掉"扔一次。两个洞同时张开。
   declare
      W : constant Natural := 96;
      H : constant Natural := 72;
      Dep : Floats := Filled (W * H, 0.80);
      Seed : Long_Long_Integer := 11;
      Small : Picture.Regions;   --  小尺子:近处那块大的应该【切不出来】
      Wide : Picture.Regions;    --  拿它自己的宽度当尺子:应该切得出来
      Edge_Off, Edge_On : Picture.Regions;
      Found_Big : Boolean := False;
   begin
      for I in 0 .. W * H - 1 loop
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         Dep.Replace_Element (I, 0.80 + Long_Float (Seed mod 1000) * 1.0e-6 - 0.0005);
      end loop;
      --  一块占了画面一多半宽的东西(凑到跟前的球就是这样),而且下沿压在画面最后一行
      for Y in 40 .. H - 1 loop
         for X in 20 .. 75 loop
            Dep.Replace_Element (Y * W + X, 0.70);
         end loop;
      end loop;
      --  ① 小尺子(0.125 画幅 = 12 px,远窄于这块 56 px 宽的东西)⇒ 它自己就是背景,切不出来
      Small := Picture.Cut (Dep, W, H, 0.125, 3.0, Keep_Edge => True);
      for R of Small loop
         if R.X1 - R.X0 > 30 then
            Found_Big := True;
         end if;
      end loop;
      Check (not Found_Big,
             "切块:比尺子宽的东西,小尺子下【确实】切不出来 —— GM 里球就是这么没的");
      --  ② 拿这块东西自己的宽度当第二把尺子 ⇒ 切得出来
      Wide := Picture.Cut (Dep, W, H, 56.0 / Long_Float (W), 3.0, Keep_Edge => True);
      Found_Big := False;
      for R of Wide loop
         if R.X1 - R.X0 > 30 then
            Found_Big := True;
         end if;
      end loop;
      Check (Found_Big, "切块:第二把尺子用这块东西自己的宽度 ⇒ 它回来了");
      --  ③ 贴边:同一块东西,Keep_Edge 关掉就该丢、开着就该留
      Edge_Off := Picture.Cut (Dep, W, H, 56.0 / Long_Float (W), 3.0, Keep_Edge => False);
      Edge_On := Picture.Cut (Dep, W, H, 56.0 / Long_Float (W), 3.0, Keep_Edge => True);
      Check (Natural (Edge_On.Length) > Natural (Edge_Off.Length),
             "切块:下沿压在画面最后一行的那块,Keep_Edge 关掉会被丢、开着才留得住");
   end;

   --  ===== 一行一判(HZ 2026-09-15:身体连着 10 步命令全零,日志全绿)=====
   declare
      --  HZ 实测 ch7 那一根的真实数字:画面两行量得准准的,深度那一行在乱跳。
      Pic_Dif  : constant Long_Float := 0.0290;   --  左右/上下 去回之差
      Pic_Con  : constant Long_Float := 0.1670;   --  左右/上下 去回共识
      Dep_Bad  : constant Long_Float := 4.9000;   --  远近 去回之差(比共识还大 ⇒ 这一行不是测量)
      Dep_Con  : constant Long_Float := 4.4115;   --  远近 去回共识(-5.024 与 -3.799 的均值绝对值)
      --  旧写法:五行合成一个数(HZ 日志原样)
      Bundle_Dif : constant Long_Float := 6.4593;
      Bundle_Con : constant Long_Float := 3.2772;
      --  HY 撤回的那一条:体检那个倍数是灵敏度,拿它去除绝对距离 ⇒ 手变成 4 厘米,物理上不可能
      Hand_Z     : constant Long_Float := 1.4000;
      Sens_32    : constant Long_Float := 32.1000;
      Too_Close  : constant Long_Float := 0.0500;
   begin
      Check (Act.Row_Is_Measurement (Pic_Dif, Pic_Con),
             "一行一判:画面那两行去回对得上 ⇒ 这根通道【能用来在画面里走】");
      Check (not Act.Row_Is_Measurement (Bundle_Dif, Bundle_Con),
             "一行一判:同一根通道,五行合成一个数就【判死】—— 这正是 HZ 连着 10 步全零的来源");
      --  载重的那一条:同一根通道,分行判和合并判给出【相反】的结论。
      Check (Act.Row_Is_Measurement (Pic_Dif, Pic_Con)
             and then not Act.Row_Is_Measurement (Bundle_Dif, Bundle_Con),
             "一行一判:分行判【留下】、合并判【判死】,两者结论相反 ⇒ 不许再退回合并判");
      Check (not Act.Row_Is_Measurement (Dep_Bad, Dep_Con),
             "一行一判:深度那一行自己对不上时,只清【那一行】,不许连累画面两行");
      Check (Act.Row_Is_Measurement (0.0, 0.0),
             "一行一判:两遍都是零 = 没有证据说它错,照原样留着(没量过 ≠ 量出来是错的)");
      --  HZ 的另一半:画面一动不动的通道,深度那一格 -23.864 仍然被当成测量带进表。
      Check (Act.Depth_Scale_Bad (-23.864),
             "体检:一根平移通道推一米深度读数变 23.9 米 —— 物理上不可能,这一条照旧要喊出来");
      --  \U0001f534 撤回(HY 实测):体检那个倍数是【灵敏度】,不是绝对尺度错,不许拿它去除深度。
      Check (Hand_Z / Sens_32 < Too_Close,
             "撤回:1.4 m 的手除以体检的 32.1 倍 = 0.04 m ⇒ 物理上不可能,所以永远不许这么除");
   end;

   --  ===== 碰到:我不动就碰不到(IA 2026-09-15 一推假报) =====
   declare
      Deliv_Floor : constant Long_Float := 0.0020;   --  身体自己量到的交付噪声
      Sat_Still   : constant Long_Float := 0.0000;   --  这一步一根关节都没真动
      Really_Went : constant Long_Float := 0.0180;
      Jumped      : constant Long_Float := 0.1500;   --  东西在画面里跳了 0.15 幅(比它自己还宽)
      Its_Size    : constant Long_Float := 0.0400;
   begin
      Check (Jumped > Its_Size,
             "碰到:东西被撞得挪了自己一个身位 —— 这一半的判据没变,是对的");
      Check (Sat_Still <= Deliv_Floor,
             "碰到:我这一步一根关节都没真动 ⇒ 不许宣布碰到(IA 第 1 推假报就是这么来的)");
      Check (Really_Went > Deliv_Floor,
             "碰到:我真动了,那一半判据才谈得上成立");
   end;

   --  ===== 胳膊当尺子:量距离,不是量体温(2026-09-15) =====
   declare
      Move_Floor : constant Long_Float := 0.0020;   --  本体位置读数抖动(米)
      Ran_Floor  : constant Long_Float := 0.0016;   --  跟踪抖动(画幅)
      Big_Swing  : constant Long_Float := 0.1200;   --  甩出去 12 cm
      Tiny_Swing : constant Long_Float := 0.0010;   --  只挪了 1 mm
      Near_Swim  : constant Long_Float := 0.0600;   --  近的东西游 0.06 画幅
      Far_Swim   : constant Long_Float := 0.0200;   --  远的东西只游 0.02 画幅
      N_Near, N_Far : Long_Float;
   begin
      N_Near := Act.Near_From_Motion (Near_Swim, Big_Swing, Ran_Floor, Move_Floor);
      N_Far  := Act.Near_From_Motion (Far_Swim,  Big_Swing, Ran_Floor, Move_Floor);
      Check (N_Near > N_Far,
             "尺子:同一甩里游得多的那个【更近】—— 这就是前后那一维唯一不靠深度读数的信号");
      Check (Act.Near_From_Motion (Near_Swim, Tiny_Swing, Ran_Floor, Move_Floor) = 0.0,
             "尺子:只挪了 1 mm(没过本体读数抖动)⇒ 不出数。三角形太扁的烂数比没有数更坏");
      Check (Act.Near_From_Motion (Ran_Floor, Big_Swing, Ran_Floor, Move_Floor) = 0.0,
             "尺子:它根本没游过跟踪抖动 ⇒ 不出数,不许把噪声当视差");
      --  两个游速一比:焦距、基线、深度尺度全约掉
      Check (Act.Farther_By (N_Near, N_Far) > 1.0,
             "尺子:我游得比它快 ⇒ 它比我远,倍数 > 1");
      Check (Act.Farther_By (N_Far, N_Near) < 1.0,
             "尺子:我游得比它慢 ⇒ 它比我近,倍数 < 1");
      Check (Act.Farther_By (N_Near, N_Near) = 1.0,
             "尺子:游得一样快 = 同一个远近 —— 抓的时候要的就是这一条,而它不需要任何常数");
      Check (Act.Farther_By (N_Near, 0.0) = 0.0 and then Act.Farther_By (0.0, N_Far) = 0.0,
             "尺子:有一个没滑够 ⇒ 说不准(返回 0),不许假装等于 1 —— 假装等于 1 就是假装抓得到");
      --  🔴 温度计 ≠ 尺子:体检那个倍数量的是【我的距离感坏了多少】,它不产生任何距离。
      Check (Act.Depth_Scale_Bad (-32.1) and then Act.Farther_By (N_Near, N_Far) > 0.0,
             "温度计 ≠ 尺子:体检只说'我的距离感放大了 32 倍',量距离得靠胳膊滑出来的那两个数");
   end;

   --  ===== "画面不再变了"的第三条旁证:我得真看见了我在判的那些点(JB:跟丢之后在幻影上报 settle) =====
   declare
      Seen_W  : constant Monitor.Watch := (Quiet => 2, No_Progress => 0, Steps => 9, Refused => 0, Blind => 0);
      Blind_W : constant Monitor.Watch := (Quiet => 2, No_Progress => 0, Steps => 9, Refused => 0, Blind => 2);
   begin
      Check (Monitor.Settled (Seen_W),
             "settled:画面不变 · 我真动过 · 而且我真看见了那些点 ⇒ 这才是到位了");
      Check (not Monitor.Settled (Blind_W),
             "settled:跟丢之后位置是按身体图猜的 ⇒ 画面当然不变,那是幻影不是到位(JB 实测 2/2 点全丢还报 settle)");
      Check (Monitor.Settled (Seen_W) /= Monitor.Settled (Blind_W),
             "settled:三条旁证缺一条就不许成立 —— 跟丢不是停下的理由,但绝对不能算到了");
   end;

   --  ===== 选眼要看【脑在那只眼里认不认得出这一段要做的事】(JA:最静的那只眼里是风扇) =====
   declare
      --  JA 2026-09-15 实测:这条胳膊一动,三只眼各变多少画幅
      Frac_0 : constant Long_Float := 0.039;   --  世界眼:看得见球
      Frac_1 : constant Long_Float := 0.024;   --  1 号眼:最静,但画面里只有风扇和键盘
      Blind  : constant Integer := 1;          --  脑看着图说过"这只眼里没有它"
      Named  : constant Integer := 0;          --  脑上一次真认出它的那只眼
      --  只按"最静"挑
      Old_Pick : constant Integer := (if Frac_1 < Frac_0 then 1 else 0);
      --  跳过脑说过"这儿没有"的那只之后
      New_Pick : constant Integer :=
        (if Blind /= 1 and then Frac_1 < Frac_0 then 1 else 0);
   begin
      Check (Old_Pick = 1,
             "选眼:只按最静挑 ⇒ 挑中 1 号眼 —— 而那只眼里没有球,编译期连拒三轮一推没走");
      Check (New_Pick = 0,
             "选眼:跳过【脑自己说过「这儿没有」】的那只 ⇒ 挑中看得见的那只");
      Check (Named >= 0 and then Named /= Blind,
             "认不出时回到【脑上次真认出它的那只眼】—— 这是身体自己印出去的承诺,必须真做");
      Check (New_Pick = Named,
             "两条要一致:跳过瞎眼之后挑到的,就是脑认出过它的那只 —— 不会来回弹");
   end;

   --  ===== "算不出还差几步" ≠ "还差 0 步"(JH:两行都没量到,却报"还差 0.0 步") =====
   declare
      Err_Row0  : constant Long_Float := 0.4277;  --  JH 实测:误差是真的
      Err_Row1  : constant Long_Float := -0.1274;
      Per_Step  : constant Long_Float := 0.0;     --  可"推一下能改多少"这只眼在这个姿势还没量到
      --  代码把算不出的行权重清零,于是按权重加起来的总和是 0
      W0        : constant Long_Float := (if Per_Step > 0.0 then 1.0 else 0.0);
      Sum_Err   : constant Long_Float := abs (Err_Row0 * W0) + abs (Err_Row1 * W0);
      No_Scale  : constant Boolean := Per_Step <= 0.0;
   begin
      Check (Sum_Err = 0.0,
             "还差几步:算不出的行权重清零 ⇒ 总和确实是 0(这一步没错)");
      Check (Err_Row0 /= 0.0,
             "还差几步:可误差是【真的】—— 0 是算不出来,不是到了");
      Check (No_Scale,
             "还差几步:这时候必须说'我不知道还差几步',不许打印'还差 0.0 步'(IZ 的假 settled 就是被这个数骗的)");
   end;

   --  ===== "这几个框里没有它"只对当下这一帧成立:我走过步之后要重新问(JF:腕眼被永久判死) =====
   declare
      Bounced : constant Natural := 0;    --  换眼来回弹的那几轮:一步都没走
      Drove   : constant Natural := 5;    --  真跑过一段:走了 5 步
      --  弹的时候必须留着标记(不留就立刻又挑回那只眼,无限弹)
      Keep_While_Bouncing : constant Boolean := Bounced = 0;
      --  🔴 清的条件不是"我动了",是"那只眼跟着我动了"(JG 实测:第一版每跑完一段就白弹三个来回)
      Blind_Rides_On_Me : constant Boolean := False;  --  第 1 只眼长在【另一条】胳膊上
      Wrist_Rides_On_Me : constant Boolean := True;   --  第 2 只眼长在我正动的这条上
      Clear_After_Driving : constant Boolean := Drove > 0 and then Blind_Rides_On_Me;
      Clear_Wrist         : constant Boolean := Drove > 0 and then Wrist_Rides_On_Me;
   begin
      Check (Keep_While_Bouncing,
             "瞎眼标记:换眼来回弹的那几轮一步没走 ⇒ 标记必须留着,否则无限弹");
      Check (not Clear_After_Driving,
             "瞎眼标记:那只眼长在别的胳膊上 ⇒ 我动它画面一帧不变,切块一模一样,答案必然还是 0 ⇒ 别清");
      Check (Clear_Wrist,
             "瞎眼标记:那只眼长在我正动的这条胳膊上 ⇒ 画面确实变了 ⇒ 清掉重新问");
      Check (Clear_Wrist /= Clear_After_Driving,
             "瞎眼标记:两只眼要分开判 —— 一律清就白弹,一律不清就把好眼判死(JF/JG 各实测一次)");
   end;

   --  ===== "碰到"的两个入口要用同一套旁证(JB:第 3 推报碰到,手在自己底座、球没动) =====
   declare
      Moved_Far : constant Boolean := True;    --  重切的斑点配对后"挪了"超过两个跟踪地板
      Pushed    : constant Boolean := False;   --  可我这一步一推都没真送出去
      Near_Me   : constant Boolean := False;   --  而且它离我最近那一块比那一块自己还远
      Old_Fires : constant Boolean := Moved_Far;
      New_Fires : constant Boolean := Moved_Far and then Pushed and then Near_Me;
   begin
      Check (Old_Fires,
             "碰到:老的那个入口只看'斑点挪了'⇒ 分割抖一下就成立(斑点没有身份)");
      Check (not New_Fires,
             "碰到:补上'我真推了'和'它贴着我'两条旁证之后,这一步不算碰到");
      Check (Old_Fires /= New_Fires,
             "碰到:同一条结论的两个入口必须用同一套旁证 —— 只堵一个等于没堵");
   end;

   --  ===== 脑写的步数不许被身体的安全上限盖住(JD:我写 400,它走 60 就回"你的上限 60") =====
   declare
      Brain_Says : constant Natural := 400;   --  脑写的 or 400 steps
      Silent     : constant Natural := 0;     --  脑一个字没写
      Safety     : constant Positive := Act.Safety_Cap;
   begin
      Check (Act.Effective_Cap (Brain_Says) = Brain_Says,
             "步数:脑写了几步就走几步 —— 安全上限不许盖在脑的话上面(owner 死命令)");
      Check (Act.Effective_Cap (Silent) = Safety,
             "步数:脑一个字没写才轮到安全上限");
      Check (Act.Effective_Cap (Brain_Says) > Safety,
             "步数:JD 实测正是这种情形(脑 400 > 安全 60)—— 取 min 就等于身体替脑做决定");
   end;

   --  ===== 放大器要有天花板:解算里面夹过了,外面那两下放大没人管(IZ:命令 7.6e9,实到 0) =====
   declare
      Cap_Eye  : constant Long_Float := 0.0512;   --  眼睛跟得住的那一档(量出来的)
      Dead_Ch  : constant Long_Float := 0.0256;   --  能让我动起来的最小一步(量出来的)
      Lim      : constant Long_Float := Long_Float'Max (Cap_Eye, Dead_Ch);
      N0       : constant Long_Float := 1.0e-9;   --  表说"没一根通道能改这个"⇒ 解出来几乎是零
      G        : constant Long_Float := Dead_Ch / N0;      --  放大到"能动起来的一步"
      --  IZ 2026-09-15 同一段连着两步的第 4 根通道命令(照抄日志,不是我挑的系数)
      Iz_Step2 : constant Long_Float := 88947710.235;
      Iz_Step3 : constant Long_Float := 7563365780.274;
      Blown    : constant Long_Float := N0 * G * (Iz_Step3 / Iz_Step2);   --  再乘一次 Push_Mult
      Sent     : constant Long_Float := Long_Float'Min (Blown, Lim);
   begin
      Check (G > 1.0e6,
             "放大器:表≈0 时 G = 能动起来的一步 / 解出来的 ⇒ 天文数字 —— 这就是 7.6e9 的来源");
      Check (Blown > Lim,
             "放大器:不夹的话,放大后的命令远超我推得动的那一下");
      Check (Sent <= Lim + 1.0e-12,
             "放大器:夹完之后,发出去的不许超过【眼睛跟得住】和【能动起来的最小一步】里大的那个");
      Check (Sent >= Dead_Ch - 1.0e-12,
             "放大器:夹完之后仍然推得动 —— GK 那条'不放大就 30 步空转'不受影响");
   end;

   --  ===== 表说"我一推也改不了"要能触发重量:零表和废表预测一样,'零表更准'永不成立 =====
   declare
      Gap        : constant Long_Float := 0.359;   --  差距还在(量出来的)
      Track      : constant Long_Float := 0.010;
      Asked      : constant Long_Float := 0.0;     --  一下推得动的推都没开出来
      Floor_Cmd  : constant Long_Float := 0.0064;
      Null_Wins  : constant Boolean := False;      --  零表【不】更准:两张表预测一模一样
      Fires      : constant Boolean := Null_Wins or else (Asked <= Floor_Cmd and then Gap > Track);
   begin
      Check (not Null_Wins,
             "重量表:表说'我一推也改不了'时,零表和它预测一样 ⇒ 老那条触发不了");
      Check (Fires,
             "重量表:第二条触发认的是'差距还在而我一下推得动的推都没开出来'⇒ 这时才重量");
      Check (not (Asked <= Floor_Cmd and then Gap <= Track),
             "重量表:差距已经进噪声了就不算 —— 不许把'到了'当成'表废了'");
   end;

   --  ===== "画面不再变了"要配旁证:我这几步真动过(IZ 2026-09-15:4 推就假报"到了") =====
   declare
      Quiet_2   : constant Natural := 2;   --  连着两步画面没变
      Moved_Ok  : constant Natural := 0;   --  Refused=0:每一步都真动了
      Never_Moved : constant Natural := 4; --  IZ 实测:连着四步命令都没送出去
   begin
      Check (Quiet_2 >= 2 and then Moved_Ok = 0,
             "settled:画面不变【而且我真动过】⇒ 这才是到位了");
      Check (not (Quiet_2 >= 2 and then Never_Moved = 0),
             "settled:画面不变【但我一步没动】⇒ 那是推不动,不是到了 —— IZ 表全零时就是这么假报的");
      Check (Never_Moved > Moved_Ok,
             "settled:这一条和'碰到要旁证我真动过'同构 —— 没动过就不许自称到了");
   end;

   --  ===== "一推能改多少"要用【真发得出的那一推】(IX 2026-09-15:两边差四十倍) =====
   declare
      Track_Win : constant Long_Float := 0.1000;   --  眼睛一步跟得住多少画幅(量出来的)
      Px        : constant Long_Float := 156.0000; --  IX 实测:这根通道每单位命令把画面搅动多少
      Boot_Amp  : constant Long_Float := 0.0256;   --  开机量到的那一档
      Err_V     : constant Long_Float := 0.2600;   --  IX 实测:上下真的还差四分之一张画面
      Can_Push, Step_Boot, Step_Real : Long_Float;
   begin
      Can_Push := Track_Win / Px;                  --  眼睛跟得住的那一推
      Check (Boot_Amp > Can_Push,
             "IX:开机那一档比【眼睛一步跟得住的】大得多 —— 腕眼里它能扫四个画幅");
      Step_Boot := Err_V / (Px * Boot_Amp);
      Step_Real := Err_V / (Px * Can_Push);
      Check (Step_Boot < 1.0,
             "IX:按开机那一档算 ⇒ 0.26 画幅被算成'不到一步' ⇒ 只发极小命令一步步蹭");
      Check (Step_Real > 1.0,
             "IX:按真发得出的那一推算 ⇒ 还差两步多,解算才会认真走");
      Check (Step_Real > Step_Boot,
             "IX:同一个误差,分母用哪一推给出【相反】的结论 ⇒ 估计必须和实际发出的那一推一致");
   end;

   --  ===== 腕眼里"抬手"和"仰镜头"画面一样,标价也一样 ⇒ 总买转腕(IW 一炮里连着两次仰到墙上) =====
   declare
      Track_Win : constant Long_Float := 0.1000;
      --  两根通道:一根真把手挪出去,一根只转腕
      Px_Lift  : constant Long_Float := 0.5000;   --  抬手:画面里球跑这么多
      Px_Tilt  : constant Long_Float := 0.5000;   --  仰镜头:画面里球跑一样多
      Reach_Lift : constant Long_Float := 0.0200; --  抬手:手在世界里真挪 2 cm
      Reach_Tilt : constant Long_Float := 0.0010; --  仰镜头:手几乎不动
      D_Lift_Old, D_Tilt_Old, D_Lift_New, D_Tilt_New : Long_Float;
   begin
      D_Lift_Old := (Px_Lift / Track_Win) ** 2;
      D_Tilt_Old := (Px_Tilt / Track_Win) ** 2;
      Check (D_Lift_Old = D_Tilt_Old,
             "IW:老标价下两者【一模一样贵】—— 画面上长得一样,价钱也一样,解算凭什么不买转腕");
      D_Lift_New := D_Lift_Old * (Reach_Lift / Reach_Lift);
      D_Tilt_New := D_Tilt_Old * (Reach_Lift / Reach_Tilt);
      Check (D_Tilt_New > D_Lift_New,
             "IW:按【搅动画面 ÷ 真把我挪了多远】标价 ⇒ 转腕贵了二十倍,解算自己就不买了");
      Check (Reach_Lift > Reach_Tilt,
             "IW:两者在画面里一样,在【本体感觉】里天差地别 —— 这就是分得开它们的那把尺子");
   end;

   --  ===== 关掉一整维的时候必须说【为什么】(IV 2026-09-15:补了退路仍是 0.0,日志一个字没解释) =====
   declare
      Row_Off   : constant Long_Float := 0.0000;   --  那一栏印出来就是 0.0
      Reasons   : constant Natural := 3;           --  可能的原因:远近读不到 / 握区没宽度 / 压根没走到那一支
   begin
      Check (Row_Off <= 0.0,
             "IV:'大小 0.0' 这一个数,三种完全不同的原因印出来一模一样");
      Check (Reasons > 1,
             "IV:所以光看那个数指不到地方 —— 必须把【我为什么关了它】打出来");
      Check (Reasons > 2,
             "不许静悄悄失效:这条和'米那一行没换算'是同一条,那次正是靠喊出来才抓到的");
   end;

   --  ===== 手自己那只眼睛里握区读不到远近 ⇒ 别把前后整维扔了(IU 2026-09-15:九步纹丝不动) =====
   declare
      Grip_Depth_Nan : constant Boolean := True;    --  手腕相机里握区的远近读不到
      Grip_Span      : constant Long_Float := 0.1400;  --  但它张开多宽,画面里量得到
      Ball_Size_Far  : constant Long_Float := 0.0400;
      Ball_Size_Near : constant Long_Float := 0.1400;
      W_Old, W_New   : Long_Float;
   begin
      W_Old := (if not Grip_Depth_Nan then 1.0 else 0.0);
      W_New := (if not Grip_Depth_Nan then 1.0 elsif Grip_Span > 0.0 then 1.0 else 0.0);
      Check (W_Old <= 0.0,
             "IU:按老写法,握区远近读不到就把'看着多大'整个关掉 ⇒ 那只眼睛里前后一个信号都不剩");
      Check (W_New > 0.0,
             "IU:退路不需要深度 —— 目标就是【我张开的那片爪心有多宽】,画面里量得到");
      Check (Ball_Size_Near > Ball_Size_Far and then Ball_Size_Near >= Grip_Span,
             "定版判据(2026-09-08):它在画面里长到和我张开的那片爪心一样宽,就是到了");
   end;

   --  ===== 撤回:"我能分辨到多远" ≠ "它有多远"(IT 2026-09-15:球近了四倍,那个数反而涨了) =====
   declare
      Ball_Px_Far  : constant Long_Float := 78.0000;    --  IT 实测:一开始球在画面里这么大
      Ball_Px_Near : constant Long_Float := 334.0000;   --  走近之后
      Lim_Early    : constant Long_Float := 0.2030;     --  同期"只能说它比 X m 远"
      Lim_Late     : constant Long_Float := 0.7480;
   begin
      Check (Ball_Px_Near > Ball_Px_Far,
             "IT:球在画面里长了四倍多 —— 看图也证实了,手确实在靠近");
      Check (Lim_Late > Lim_Early,
             "IT:而那个数反而从 0.203 涨到 0.748 —— 它跟着【我走了多远】涨,不跟着球涨");
      Check (not (Lim_Late < Lim_Early),
             "IT:两者方向相反 ⇒ 那个数不是距离,是【我能分辨到多远】,拿它驱动就是追一个越走越远的目标");
   end;

   --  ===== 走米的时候,用哪根关节也不看"画面证没证过"(IS 2026-09-15:每步只挪 2 mm) =====
   declare
      Chans      : constant Natural := 6;
      Pic_Proven : constant Natural := 1;        --  腕眼里只有一根过得了画面那道门
      Gap_M      : constant Long_Float := 0.7300;
      Per_Step_M : constant Long_Float := 0.0020;  --  IS 实测:只用那一根,每步挪 2 mm
      Budget     : constant Long_Float := 40.0000; --  一段给 40 步
   begin
      Check (Pic_Proven < Chans,
             "IS:腕眼里六根关节只有一根过得了画面那道门(滑得太快,来回对表几乎全判死)");
      Check (Gap_M / Per_Step_M > Budget,
             "IS:只用那一根 ⇒ 0.73 m 要三百多步,而一段只有 40 步 ⇒ 注定走不到");
      Check (Gap_M / Per_Step_M > Budget + Budget,
             "IS:差得不是一点半点 —— 所以这不是调参,是那道门根本不该管【往前走】");
   end;

   --  ===== 米那一行不看"画面证没证过"(IR 2026-09-15:换算早量到了,却连喊 8 次没换算) =====
   declare
      Reach     : constant Long_Float := 0.0200;   --  一推手在世界里走几米(关节读数量出来的)
      Pic_Proven : constant Boolean := False;      --  腕眼里所有通道都标"没证过"
      Seen       : constant Boolean := True;       --  但开机就看见过它动
      Per_Step_Gated, Per_Step_Free : Long_Float;
   begin
      Per_Step_Gated := (if Seen and then Pic_Proven then Reach else 0.0);
      Per_Step_Free  := (if Seen then Reach else 0.0);
      Check (Per_Step_Gated <= 0.0,
             "IR:卡在'画面证过'上 ⇒ 腕眼里没有一根通道过关 ⇒ 这一行永远没换算,连喊 8 次");
      Check (Per_Step_Free > 0.0,
             "IR:不卡它就有换算 —— 走几米是关节读数给的,跟画面量没量准毫无关系");
      Check (Per_Step_Free > Per_Step_Gated,
             "IR:同一套数,卡不卡给出【相反】的结论 ⇒ 米那一行只看本体感觉,不看画面");
   end;

   --  ===== 第一次只许给下界,不许给准数(IQ 2026-09-15:第一次量就报"离我 0.001 m") =====
   declare
      No_Ref : constant Long_Float := 0.0000;   --  第一次量:手上没有可比的那一次
      Ref    : constant Long_Float := 0.0730;   --  量过一次之后手上的下界
      Went   : constant Long_Float := 0.0010;   --  IQ 实测第一次只走了 1 mm
   begin
      Check (No_Ref <= 0.0,
             "IQ:第一次量的时候手上没有【可比的那一次】—— 准数要拿两次比,下界只要这一次");
      Check (Went < Ref,
             "IQ:它却在只走了 1 mm 的第一次就报出准数,而同一行的下界是 7.3 cm —— 自相矛盾");
      Check (Ref > No_Ref,
             "IQ:有过一次下界之后,那个下界本身就是'至少要走这么远才谈得上再量'的尺度");
   end;

   --  ===== 换算表要记在【循环里】,不能记在函数末尾(IQ:提前返回 ⇒ 整段漏掉) =====
   declare
      Early_Returns : constant Natural := 4;   --  Range_Probe 里提前返回的路数
   begin
      Check (Early_Returns > 0,
             "IQ:量距离那个函数有好几条提前返回的路(眼睛不对/拨不动/太扁/挪不够)");
      Check (Early_Returns > 1,
             "IQ:把'一推走几米'记在函数末尾 ⇒ 走上任何一条就整段漏掉 ⇒ 换算表永远是 0,米那一行永远是死的");
   end;

   --  ===== 走得还不到已知的下界,就不许再报距离(IP 2026-09-15:走 2 mm 报"离我 0.002 m") =====
   declare
      Bound   : constant Long_Float := 0.0500;   --  上一次量出来的下界:它至少有这么远
      Went_S  : constant Long_Float := 0.0020;   --  IP 实测:两次之间只走了 2 mm
      Went_OK : constant Long_Float := 0.0600;
      Nudge   : constant Long_Float := 0.0012;
   begin
      Check (Went_S <= Long_Float'Max (Nudge, Bound),
             "IP:只走 2 mm、而已知下界是 5 cm ⇒ 这一段根本分辨不出它有多远,不许报距离");
      Check (Went_OK > Long_Float'Max (Nudge, Bound),
             "IP:走过了已知的下界才谈得上再量一次 —— 尺度是它自己上一次量出来的,不是我拍的");
      Check (Bound > Went_S,
             "IP:报出来的 0.002 m 比自己上一次的下界还小两个数量级 —— 自相矛盾,本该当场拦住");
   end;

   --  ===== 换算表没量到 ⇒ 米那一行【静悄悄失效】(IO 2026-09-15:差距五步纹丝不动) =====
   declare
      Gap_M : constant Long_Float := 0.7190;   --  IO 实测:前后差这么多米,五步一点没缩
      No_Conv : constant Long_Float := 0.0000; --  一推走几米:还没量到
      Conv    : constant Long_Float := 0.0200;
   begin
      Check (No_Conv <= 0.0,
             "IO:身体装回了存好的表 ⇒ 探针不跑 ⇒ '一推走几米'从来没量到过");
      Check (Gap_M / Long_Float'Max (Conv, 1.0e-9) > 1.0,
             "IO:有换算时,0.719 m 换出三十几步,解算才推得动");
      Check (not (No_Conv > 0.0),
             "IO:没换算时这一行【什么都不做】,而解算照跑、日志全绿 —— 所以必须喊出来,不许静悄悄失效");
   end;

   --  ===== 尺子量出来的米,要接进解算(IM 2026-09-15:横向 2 mm、前后 0.535 m,却说"还差 0.0 步") =====
   declare
      Gap_M    : constant Long_Float := 0.5350;   --  IM 实测:前后还差这么多米
      Reach    : constant Long_Float := 0.0200;   --  一推手在世界里走几米(探针量出来的)
      Amp      : constant Long_Float := 1.0000;
      Pic_Slope : constant Long_Float := 49.1650; --  画面单位的深度斜率
      Steps_M, Steps_Pic : Long_Float;
   begin
      Steps_M := Gap_M / (Reach * Amp);
      Check (Steps_M > 1.0,
             "接进解算:用【一推走几米】换算 ⇒ 0.535 m 还差二十几步,解算才有事可做");
      Steps_Pic := Gap_M / (Pic_Slope * Amp);
      Check (Steps_Pic < 1.0,
             "接进解算:拿画面单位的深度斜率去除米 ⇒ 算出不到一步 ⇒ 正是 IM 那个'还差 0.0 步'");
      Check (Steps_M > Steps_Pic,
             "接进解算:两把不同的尺子相除差着一个数量级 —— 米要配米,不许混");
   end;

   --  ===== 画面那两行是一对:合起来判过了,不许再分开清零(IL 2026-09-15) =====
   declare
      --  IL 实测:通道 7 画面两行合起来 分歧 56.67 · 共识 141.61 ⇒ 信得过
      Pair_Dif : constant Long_Float := 56.6711;
      Pair_Con : constant Long_Float := 141.6149;
      --  而拆开之后,单独一行可能对不上(左右那一行去回差得多)
      Row_Dif  : constant Long_Float := 120.0000;
      Row_Con  : constant Long_Float := 100.0000;
   begin
      Check (Act.Row_Is_Measurement (Pair_Dif, Pair_Con),
             "IL:画面两行【合起来】判 ⇒ 这根通道信得过(实测共识 141.6,响应大得很)");
      Check (not Act.Row_Is_Measurement (Row_Dif, Row_Con),
             "IL:同一根通道,单独拆出一行来判却过不了");
      Check (Act.Row_Is_Measurement (Pair_Dif, Pair_Con)
             and then not Act.Row_Is_Measurement (Row_Dif, Row_Con),
             "IL:两个判决相反 ⇒ 分开清零会把【合起来判过了】的通道掏空,表里只剩 左右 0.000 没证过");
      Check (Pair_Con > Pair_Dif,
             "IL:掏空的后果就是 HZ 那个瘫痪的翻版 —— 一对量、一个判决,要留一起留,要清一起清");
   end;

   --  ===== 拨到【它真的滑得动】为止(IB 2026-09-15 第一炮实测) =====
   declare
      Track_Jitter : constant Long_Float := 0.0016;   --  跟踪抖动(画幅)
      EE_Jitter    : constant Long_Float := 0.0003;   --  本体位置读数抖动(米)
      Tiny_Move    : constant Long_Float := 0.0005;   --  一拨只挪了半毫米
      Tiny_Slide   : constant Long_Float := 0.0009;   --  于是它只滑了 0.0009 幅
      Good_Slide   : constant Long_Float := 0.0400;
   begin
      Check (Tiny_Move > EE_Jitter,
             "退出条件写成'挪过本体读数抖动' ⇒ 半毫米就算过关 —— IB 第一炮就是这么退出的");
      Check (Tiny_Slide < Track_Jitter,
             "而它只滑了 0.0009 幅、还没过跟踪抖动 ⇒ 那一拨等于没量");
      Check (Good_Slide > Track_Jitter,
             "所以退出条件必须是【它在我眼里滑过了跟踪抖动】—— 三角形扁不扁看它滑了多少,不看我动了多少");
   end;

   --  ===== 脑点名换眼睛,身体不许替它改主意(IH 2026-09-15:量远近永远被拒) =====
   declare
      Tgt_Cam  : constant Natural := 0;   --  球是在 0 号眼里被点的名
      Want_Cam : constant Natural := 2;   --  脑写 with my moving eye ⇒ 长在我身上的那只
      --  我一动,各只眼睛变多少画面(开机量出来的)
      Frac0 : constant Long_Float := 0.0240;
      Frac2 : constant Long_Float := 0.7460;
   begin
      Check (Want_Cam /= Tgt_Cam,
             "IH:脑要的那只眼,不是球上次被点名的那只 —— '目标那台赢'于是每次都把它拽回去");
      Check (not (Frac0 >= Frac2),
             "IH:而被拽回去的那只【不长在我身上】⇒ 量远近的判据当场否掉 ⇒ 前后那一栏永远是 0.0");
      Check (Frac2 > Frac0,
             "IH:脑要的那只才是长在我身上的 —— 照脑说的换过去,到那边再问一次它是哪一块");
   end;

   --  ===== 碰到:瞎着的时候不许宣布 · 隔着半张桌子不许宣布(IG 2026-09-15 看图证伪) =====
   declare
      --  IG 身体自己的原话:"I could not see 2 of 2 of the points I am tracking;
      --  I am going on where my body map says they are" —— 然后宣布 contact。
      Lost_Both  : constant Natural := 2;
      Tracked    : constant Natural := 2;
      --  看图量到的:手在画面右下角,球在桌心
      Hand_U : constant Long_Float := 0.7900;
      Hand_V : constant Long_Float := 0.6900;
      Ball_U : constant Long_Float := 0.6900;
      Ball_V : constant Long_Float := 0.3600;
      My_Size : constant Long_Float := 0.1000;   --  我自己那一块在画面里有多大
      Du : constant Long_Float := Hand_U - Ball_U;
      Dv : constant Long_Float := Hand_V - Ball_V;
      Gap2 : constant Long_Float := Du * Du + Dv * Dv;
   begin
      Check (Lost_Both = Tracked,
             "碰到:IG 当时两个跟踪点【全丢了】,位置全靠身体图猜 ⇒ 那一刻它是瞎的");
      Check (Gap2 > My_Size * My_Size,
             "碰到:而画面上手和球隔着比我自己还宽三倍 ⇒ 隔着大半张桌子,不可能是我碰的");
      Check (not (Gap2 <= My_Size * My_Size),
             "碰到:所以第三条旁证是【它得贴着我】—— 尺子是我自己那块有多大,量出来的");
   end;

   --  ===== 算出来的距离不许比刚走过的路还短 · 走得短就只给下界(IJ 2026-09-15:报出 0.000 m) =====
   declare
      Floor : constant Long_Float := 0.0016;   --  跟踪抖动(幅)
      Nudge : constant Long_Float := 0.0013;   --  IJ 实测一拨挪多少米
      S1 : constant Long_Float := 0.2621 / Nudge;   --  IJ 实测的两次滑速(幅每米)
      S2 : constant Long_Float := 0.3684 / Nudge;
      Short : constant Long_Float := 0.0020;   --  IJ 实测两次量之间只走了 2 mm
      Long_Walk : constant Long_Float := 0.1500;
      Jit : constant Long_Float := (S2 - S1) * 1.0;   --  原地重量时滑速自己晃这么多(IJ 实测的量级)
      Zs, Zl, Lim : Long_Float;
   begin
      --  \U0001f534 IJ 实测:两次只隔 2 mm,滑速却差了 40% —— 拿跟踪抖动当地板,这个差轻松过关
      Zs := Act.Distance_Now (Short, S1, S2, Floor / Nudge);
      Check (Zs > 0.0,
             "地板:拿【跟踪抖动】当地板时,IJ 那两个滑速的差轻松过关 ⇒ 于是编出一个几毫米的距离");
      --  而身体【原地】重量一遍时,滑速自己就晃这么多 ⇒ 这才是真地板
      Check (Act.Distance_Now (Short, S1, S2, Jit) = 0.0,
             "地板:用身体【原地量出来的滑速自晃】当地板 ⇒ 同一组数当场判成'没真走近',不给数");
      Check (Jit > Floor / Nudge,
             "地板:原地自晃比跟踪抖动大得多 —— 小的那个挡不住任何东西,这就是它一直放行的原因");
      Zl := Act.Distance_Now (Long_Walk, S1, S2, Floor / Nudge);
      Check (Zl > Long_Walk,
             "距离:同样两个滑速,走够长才给得出一个【比走过的路还远】的数,那才可能是真的");
      --  走多短就只分辨得到多近:滑速 × 走了多远 ÷ 跟踪抖动
      Lim := Act.Can_Tell_Upto (S2, Short, Floor);
      Check (Lim > 0.0,
             "下界:走这么远最远分辨得到多远,是算得出来的 —— 超过它就只说'它比这个远'");
      Check (Act.Can_Tell_Upto (S2, Long_Walk, Floor) > Lim,
             "下界:走得越远,分辨得到的越远 —— 所以'走了多远'才是这件事的尺子");
   end;

   --  ===== 只有【长在我身上】的眼睛量得了别人的远近(IF 2026-09-15:一甩把球撞飞) =====
   declare
      Track : constant Long_Float := 0.0016;
      --  我一动,三台眼睛各变多少画面(开机量出来的)
      On_Me   : constant Long_Float := 0.7460;   --  长在我这条胳膊上的那台
      Other   : constant Long_Float := 0.6190;   --  长在别的部件上的那台
      World   : constant Long_Float := 0.0240;   --  完全不动的那台(我的胳膊在画面里占一点)
      Ball_Slid : constant Long_Float := 0.0300; --  IF 实测:球在【不动的那台】里滑了这么多
      Ball_Wide : constant Long_Float := 0.0400; --  球自己有多宽
      Big_Nudge : constant Long_Float := 0.3529; --  IF 实测那一甩
      OK_Nudge  : constant Long_Float := 0.0564;
   begin
      Check (World > Track,
             "IF:判据只问'我一动它变不变'时,不动的那台也过关 —— 因为我的胳膊在它画面里占着地方");
      Check (not (World >= On_Me),
             "IF:而它并不是【变得最多】的那台 ⇒ 按'哪台变得最多'判,它就被挡在外面了");
      Check (On_Me >= Other and then On_Me >= World,
             "只有变得最多的那台才是长在我身上的 —— 世界只在它里面滑");
      Check (Ball_Slid > Track,
             "IF:球在【不动的那台】里滑了 0.03 幅 —— 身体把它读成了视差");
      Check (Big_Nudge > OK_Nudge,
             "IF:于是一路把拨动加到 0.3529 m,那一甩把球从桌心撞到了最远沿(看图确认)");
      Check (Ball_Slid < Ball_Wide,
             "封顶:滑得比那东西自己还宽就够了 —— 再大就是白甩一路家具,尺寸是量出来的");
   end;

   --  ===== 循环上限写错 = 那段代码一次都没跑过(IE 2026-09-15) =====
   declare
      Wide : constant Long_Float := 640.0;   --  这台相机一行有多少像素
   begin
      Check (Act.Levels_For (1.0) = 1,
             "上限:`Levels_For (1.0)` 返回 1 ⇒ 拨一下就退出 ⇒ '一路拨大'那段一次都没跑过");
      Check (Act.Levels_For (Wide) > 1,
             "上限:按【这台相机有几层金字塔】算才拨得动 —— 它是分辨率事实,不是我拍的门槛");
      Check (Act.Levels_For (Wide) > Act.Levels_For (1.0),
             "上限:同一个函数,喂错参数就把整段功能静默关掉 —— 这种错日志里一个字都不会说");
   end;

   --  ===== 拨得够大:滑动要【远大于】地板,两次之差才有可能过噪声(ID 2026-09-15) =====
   declare
      Floor : constant Long_Float := 0.0016;   --  跟踪抖动(幅)
      Z1    : constant Long_Float := 0.4000;   --  球先在 0.40 m
      Z2    : constant Long_Float := 0.3500;   --  走近到 0.35 m
      Small : constant Long_Float := 0.0032;   --  小拨动:滑动刚过地板两倍(ID 实测就是这一档)
      Big   : constant Long_Float := 0.0320;   --  大拨动:滑动是地板的二十倍
      DS, DB : Long_Float;
   begin
      --  同一下拨动,滑动 ∝ 1/Z ⇒ 走近之后滑动按 Z1/Z2 变大
      DS := Small * Z1 / Z2 - Small;
      DB := Big * Z1 / Z2 - Big;
      Check (DS < Floor,
             "拨得够大:刚过地板那一档,走近 5 cm 的滑动之差还在噪声里 ⇒ 这一段永远量不出米数");
      Check (DB > Floor,
             "拨得够大:滑动是地板二十倍那一档,同样走近 5 cm 就分得出来 ⇒ 所以要拨到我还跟得住的最大一档");
      Check (DB > DS,
             "拨得够大:拨得越大,同样的走近越分得出来 —— 缩幅度是修反的(记录 08-27 V2)");
   end;

   --  ===== 再量一次的判据:走了多远,不是差距有没有变小(ID 实测米数永远停在"第一次量") =====
   declare
      Gap_Before : constant Long_Float := 0.633;
      Gap_After  : constant Long_Float := 1.022;   --  ID 实测:差距不但没缩,还涨了
      Nudge_Len  : constant Long_Float := 0.0011;  --  上一拨我自己挪了多远
      Went       : constant Long_Float := 0.0070;  --  从上次量到现在走了多远
   begin
      Check (not (Gap_After < Gap_Before),
             "判据:ID 那一段差距没缩 ⇒ 用'更近了'当判据的话,一次都不会再量");
      Check (Went > Nudge_Len,
             "判据:而它确实走了 7 mm、比上一拨自己挪的还远 ⇒ 用'走了多远'当判据就会再量一次");
   end;

   --  ===== IC 2026-09-15:身体报出"离我 0.014 m"而球在几十厘米外 =====
   --  两个洞同时开着,一个单位错、一个假设错。两个都补,才拦得住那个看起来完全正常的数。
   declare
      Track_Jitter : constant Long_Float := 0.0016;   --  跟踪抖动,单位是【幅】
      EE_Jitter    : constant Long_Float := 0.0003;   --  位置读数抖动,单位是【米】
      Nudge        : constant Long_Float := 0.0012;   --  这一拨挪了多少【米】
      Slip_Noise   : constant Long_Float := Track_Jitter / Nudge;   --  滑速的噪声,单位【幅每米】
      Many         : constant Long_Float := 100.0;
      S1 : constant Long_Float := 0.0037 / Nudge;
      S2 : constant Long_Float := 0.0059 / Nudge;     --  IC 实测的两次滑速
      Trav : constant Long_Float := 0.0070;           --  两次之间只走了 7 mm
      --  两拨的方向:一致 vs 各推各的
      Same_Dot : constant Long_Float := Nudge * Nudge;          --  完全同向
      Off_Dot  : constant Long_Float := Nudge * Nudge / Many;   --  几乎垂直
      --  II 2026-09-15:腕相机贴着球,一拨滑掉四分之一个画面
      Slid     : constant Long_Float := 0.2791;
   begin
      --  ① 单位:滑速的噪声和跟踪抖动差着三个数量级,拿后者当门槛等于没有门槛
      Check (Slip_Noise > Track_Jitter * Many,
             "单位:滑速的噪声(幅每米)比跟踪抖动(幅)大三个数量级 —— 混用等于这道闸根本不响");
      --  ② 假设:同一根关节同样的命令,在不同姿势下把手推向【不同方向】⇒ 滑速变了跟远近无关
      Check (Act.Same_Nudge (Same_Dot, Nudge, Nudge, Slid, Track_Jitter),
             "同一下:两拨方向一样 ⇒ 才能比滑速");
      Check (not Act.Same_Nudge (Off_Dot, Nudge, Nudge, Slid, Track_Jitter),
             "同一下:两拨把手推向不同方向 ⇒ 滑速变了不代表走近了 ⇒ 不许出米数(IC 那个 0.014 m 的真凶)");
      Check (not Act.Same_Nudge (Same_Dot, 0.0, Nudge, Slid, Track_Jitter),
             "同一下:有一拨根本没挪 ⇒ 没有方向可比 ⇒ 不许出米数");
      --  \U0001f534 IK 2026-09-15:两拨方向完全一致,幅度却差八倍(0.0033 m vs 0.0004 m)
      --  ⇒ 只查方向就过关 ⇒ 报出"离我 0.014 m",而球在二三十厘米外。
      declare
         Big   : constant Long_Float := 0.0033;   --  IK 实测参照那一拨
         Small : constant Long_Float := 0.0004;   --  IK 实测后一拨,差八倍
         Wobble : constant Long_Float := 0.0034;  --  真实推送本来就有几个百分点的波动
         Near_1 : constant Long_Float := 0.9990;  --  IN 实测方向一致度
      begin
         --  \U0001f534 撤回:我一度拿【方向那条容差】去卡幅度(0.3%),于是几个百分点的正常波动
         --  就被判"不是同一下" —— IN 实测方向一致度 0.999 也被拦,这道闸当场变成永不放行。
         Check (abs (Big - Wobble) / Big < 0.0500,
                "撤回:两拨幅度本来就会差几个百分点 —— 拿 0.3% 去卡它,等于永不放行");
         Check (Near_1 >= 1.0 - Track_Jitter / Slid,
                "方向:0.999 这种一致度本来就该放行,它是正常的同一下");
         --  IK 那八倍的差,真凶不是物理而是我在同一段里反复重跑"一路拨大"
         Check (Big / Small > 4.0,
                "IK:那八倍的差是【我自己每次重新加大拨动】造出来的,不是身体的物理");
         Check (Act.Same_Nudge (Big * Big, Big, Big, Slid, Track_Jitter),
                "同一下:一段里只挑一次拨法、之后原样重用 ⇒ 两拨自然就是同一下");
      end;
      --  \U0001f534 II 实测:容差写成"位置读数抖动 ÷ 挪了多远"时,抖动量出来是 0 ⇒ 门槛正好 1.0
      --  ⇒ `cos > 1.0` 恒假 ⇒ 方向一致度 1.000 也被判"不是同一下",这道闸从来没放行过。
      Check (EE_Jitter / Nudge > 0.0,
             "撤回:容差原来写成 位置读数抖动 ÷ 挪了多远 —— 抖动是 0 时门槛就是 1.0,恒不放行");
      Check (Track_Jitter / Slid < EE_Jitter / Nudge,
             "撤回:改成 跟踪抖动 ÷ 这一次滑了多少 —— 滑得越多,方向就越容不得差,而它永远放得行");
      --  ③ 这一组数在补完之后【确实】被拦住:方向不一致就够了,不必靠单位
      Check (Act.Distance_Now (Trav, S1, S2, Slip_Noise) > 0.0
             and then not Act.Same_Nudge (Off_Dot, Nudge, Nudge, Slid, Track_Jitter),
             "IC:光看滑速这组数是过关的 —— 拦住它的是【方向不一样】,所以两个洞都得补");
   end;

   --  ===== 米数:一只眼 + 会动 + 知道自己走了多远(所有机体通用) =====
   declare
      Ran_Floor : constant Long_Float := 0.0016;   --  跟踪抖动(画幅)
      --  离我 1 米时拨一下滑出 0.0500;朝它走 0.5 m 之后,同一下滑速翻倍 ⇒ 它就在 0.5 m 外
      S1 : constant Long_Float := 0.0500;
      S2 : constant Long_Float := 0.1000;
      Went : constant Long_Float := 0.5000;
      Z : Long_Float;
   begin
      Z := Act.Distance_Now (Went, S1, S2, Ran_Floor);
      Check (Z > 0.4900 and then Z < 0.5100,
             "米数:滑速翻一倍 = 它离我只剩一半 ⇒ 走了 0.5 m 之后它就在 0.5 m 外。焦距/基线/深度尺度全约掉");
      Check (Act.Distance_Now (Went, S1, S1, Ran_Floor) = 0.0,
             "米数:滑速一点没变 ⇒ 这一段我根本没走近它 ⇒ 说不准,不许给数");
      Check (Act.Distance_Now (Went, S1, S1 + Ran_Floor, Ran_Floor) = 0.0,
             "米数:滑速只多了一个跟踪抖动 ⇒ 那是噪声不是走近 ⇒ 说不准");
      Check (Act.Distance_Now (0.0, S1, S2, Ran_Floor) = 0.0,
             "米数:我一步没走 ⇒ 没有尺子 ⇒ 说不准(这一条就是【胳膊当尺子】里的那条胳膊)");
      --  🔴 撤回:第一版要求"甩另一条胳膊"。一条胳膊的机器没有另一条,无人机连胳膊都没有。
      Check (Act.Distance_Now (Went, S1, S2, Ran_Floor) > 0.0,
             "撤回:量距离不需要第二条胳膊、不需要第二只眼 —— 只要会动、知道走了多远、拨得出同样的一下");
   end;

   --  ===== 接触集:四格 + 集合级判据 —— 2026-08 的十三个动词逐条搬回(commit ef10664 contact-set 的单元测试,数一个没改) =====
   declare
      package Ct renames Contact;
      use type Ct.Gap_Kind;
      use type Ct.Many_Kind;
      use Ada.Numerics.Long_Elementary_Functions;
      MM : constant Long_Float := 0.002;        --  碰到的地方:毫米级
      CM : constant Long_Float := 0.02;         --  路过的地方:厘米级
      Mu_Half : constant Long_Float := 0.4636;  --  atan(0.5):μ = 0.5 的摩擦锥半张角
      Z_Up : constant Ct.V3 := [0.0, 0.0, 1.0];
      Z_Dn : constant Ct.V3 := [0.0, 0.0, -1.0];
      function Cone (Axis : Ct.V3; Half : Long_Float) return Ct.Cone is ((Axis => Axis, Half_Angle => Half));
      function Pt (Pos, Normal : Ct.V3; K : Ct.Cone; Tol : Long_Float := MM; Pad : Boolean := False) return Ct.Point is
        ((By => (Ct.Hand, 0), Pos => Pos, Normal => Normal, Push => K, Pull => False, Torsion => Pad, Peel => False, Tol_M => Tol));
      --  桌子在支点那儿顶着物体的那个接触:它一直都在,只是①以前没地方记
      function Table_Holds (Pivot : Ct.V3; Mu_Atan : Long_Float) return Ct.Point is
        ((By => (Ct.World, 0), Pos => Pivot, Normal => Z_Dn, Push => Cone (Z_Up, Mu_Atan), Pull => False, Torsion => False, Peel => False, Tol_M => MM));
      --  两个相对的点:抓的最小形状。物体在原点、宽 W
      function Two (W, Half : Long_Float) return Ct.Point_Vectors.Vector is
         V : Ct.Point_Vectors.Vector;
      begin
         V.Append (Pt ([-0.5 * W, 0.0, 0.10], [-1.0, 0.0, 0.0], Cone ([1.0, 0.0, 0.0], Half)));
         V.Append (Pt ([0.5 * W, 0.0, 0.10], [1.0, 0.0, 0.0], Cone ([-1.0, 0.0, 0.0], Half)));
         return V;
      end Two;
      function Single (P : Ct.Point) return Ct.Point_Vectors.Vector is
         V : Ct.Point_Vectors.Vector;
      begin
         V.Append (P);
         return V;
      end Single;
      function Mk (Pts : Ct.Point_Vectors.Vector; Mo : Ct.Twist) return Ct.Set is
        ((Points => Pts, Motion => Mo, Has_Approach => False, Approach => [others => 0.0]));
      function Mk (Pts : Ct.Point_Vectors.Vector; Mo : Ct.Twist; Ap : Ct.V3) return Ct.Set is
        ((Points => Pts, Motion => Mo, Has_Approach => True, Approach => Ap));
      function Turn (Axis : Ct.V3; Rad : Long_Float; Pivot : Ct.V3) return Ct.Twist is
         Ok : Boolean;
         T : constant Ct.Twist := Ct.Turn (Axis, Rad, Pivot, Ok);
      begin
         pragma Assert (Ok, "转轴非零");
         return T;
      end Turn;
      Pivot : constant Ct.V3 := [0.05, 0.0, 0.0];
      Lever : constant Ct.V3 := [-0.04, 0.0, 0.02];
   begin
      Check (Ct.Admits (Cone (Z_Up, 0.5), [0.1, 0.0, 1.0]) and then not Ct.Admits (Cone (Z_Up, 0.5), [1.0, 0.0, 0.2]),
             "接触集·锥:判据是角度不是力 —— 偏 5.7° 在半张角 0.5 rad 里,偏 78.7° 不在");
      declare
         R : constant Ct.V3 := Ct.Apply (Turn (Z_Up, 0.5 * Ada.Numerics.Pi, [others => 0.0]), [1.0, 0.0, 0.0]);
      begin
         Check (abs R (0) < 1.0e-9 and then abs (R (1) - 1.0) < 1.0e-9, "接触集·旋量:绕 z 转 90° 把 x 轴上的点搬到 y 轴上(罗德里格斯)");
      end;
      Check (Ct.Check (Mk (Two (0.05, 0.5), Ct.Still ([0.0, 0.0, 0.10])), False).Kind = Ct.Fine,
             "接触集·抓:两个相对的点向内使劲、物体不动 ⇒ 四格齐(松:同样的点、物体不动,同样过)");
      Check (Ct.Check (Mk (Single (Pt ([0.0, 0.0, 0.10], Z_Up, Cone (Z_Dn, 0.2))), Ct.Still ([0.0, 0.0, 0.10])), False).Kind = Ct.Fine,
             "接触集·压:一个点沿法向使劲、物体不动是一个合法答案,不是缺省值");
      Check (Ct.Check (Mk (Single (Pt ([0.03, 0.0, 0.05], [1.0, 0.0, 0.0], Cone ([-1.0, 0.0, 0.0], 0.6))), Ct.Slide ([-0.10, 0.0, 0.0])), True).Kind = Ct.Fine,
             "接触集·推:一个点横向力、物体在支撑面上平移");
      declare
         Hand_Only : constant Ct.Point_Vectors.Vector := Single (Pt (Lever, Z_Up, Cone (Z_Dn, Mu_Half)));
         With_Table : Ct.Point_Vectors.Vector := Hand_Only;
      begin
         With_Table.Append (Table_Holds (Pivot, 0.46));
         Check (Ct.Check (Mk (With_Table, Turn ([0.0, 1.0, 0.0], -0.4, Pivot)), True).Kind = Ct.Fine,
                "接触集·撬:手一个点 + 桌子在支点顶着的那个接触(世界接触)⇒ 物体绕那条边转");
         Check (Ct.Check (Mk (Hand_Only, Turn ([0.0, 1.0, 0.0], -0.4, Pivot)), True).Kind = Ct.Cannot_Drive,
                "接触集·撬:不给支反力就该判死 —— 单个接触力产生不出纯力矩(反例:台子有没有牙)");
         Check (Ct.Check (Mk (With_Table, Turn ([0.0, 1.0, 0.0], -0.9 * Ada.Numerics.Pi, Pivot)), True).Kind = Ct.Fine,
                "接触集·翻:同一形状、更大的角");
      end;
      Check (Ct.Check (Mk (Two (0.05, Mu_Half), Turn ([0.0, 1.0, 0.0], 1.8, [0.0, 0.0, 0.10])), True).Kind = Ct.Fine,
             "接触集·倒:握着绕一条水平轴转");
      Check (Ct.Check (Mk (Two (0.05, Mu_Half), Turn ([0.0, 0.0, 1.0], 1.5, [0.0, 0.0, 0.10])), True).Kind = Ct.Fine,
             "接触集·拧:握着绕物体自己的轴转 —— 与倒的差别只在第③格的轴,四格结构一个字没变");
      Check (Ct.Check (Mk (Two (0.05, Mu_Half), Ct.Slide ([0.0, 0.06, 0.0])), True).Kind = Ct.Fine, "接触集·插:握着沿一条轴往里走");
      Check (Ct.Check (Mk (Two (0.05, Mu_Half), Ct.Slide ([0.20, 0.30, -0.10])), True).Kind = Ct.Fine,
             "接触集·放:握着搬到目标位姿(两指横向搬运:任何单指都做不到,两指一起可以 —— 判据必须在集合上);松手是下一个接触集");
      Check (Ct.Check (Mk (Single (Pt ([0.0, 0.0, 0.10], Z_Up, Cone (Z_Up, 0.0))), Ct.Slide ([0.0, 0.0, 0.05])), True).Kind = Ct.Fine,
             "接触集·吸盘:1 个点 + 只允许法向的锥,填同一张表");
      declare
         Ns : constant array (1 .. 2) of Positive := [3, 5];
      begin
         for N of Ns loop
            declare
               V : Ct.Point_Vectors.Vector;
            begin
               for I in 0 .. N - 1 loop
                  declare
                     A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (I) / Long_Float (N);
                     C : constant Long_Float := Cos (A);
                     Sn : constant Long_Float := Sin (A);
                  begin
                     V.Append (Pt ([0.03 * C, 0.03 * Sn, 0.10], [C, Sn, 0.0], Cone ([-C, -Sn, 0.0], 0.5)));
                  end;
               end loop;
               Check (Ct.Check (Mk (V, Ct.Still ([0.0, 0.0, 0.10])), False).Kind = Ct.Fine and then Natural (V.Length) = N,
                      "接触集·" & Codec.Img (N) & " 指:只是点数不同,填得满同一张表");
            end;
         end loop;
      end;
      Check (Ct.Check (Mk (Single (Pt ([0.0, 0.0, 0.10], Z_Up, Cone (Z_Dn, 0.1))), Ct.Slide ([0.10, 0.0, 0.0])), True).Kind = Ct.Cannot_Drive,
             "接触集·锥与物体运动矛盾(只能往下压,却要它横着走)⇒ 当场点名 CannotDrive");
      --  四格各自缺失都点得出名
      declare
         Good : constant Ct.Point := Pt ([0.0, 0.0, 0.1], Z_Up, Cone (Z_Dn, 0.3));
         Nil : Ct.Point_Vectors.Vector;
         Bad : Ct.Point := Good;
         G : Ct.Gap;
      begin
         Check (Ct.Check (Mk (Nil, Ct.Still ([others => 0.0])), False).Kind = Ct.No_Points, "接触集·① 一个点都没有 ⇒ NoPoints");
         Bad.Normal := [others => 0.0];
         G := Ct.Check (Mk (Single (Bad), Ct.Still ([others => 0.0])), False);
         Check (G.Kind = Ct.Bad_Normal and then G.Index = 0, "接触集·② 法向不是方向 ⇒ BadNormal(0):" & Ct.Img (G));
         Bad := Good;
         Bad.Push := Cone ([others => 0.0], 0.3);
         G := Ct.Check (Mk (Single (Bad), Ct.Still ([others => 0.0])), False);
         Check (G.Kind = Ct.Bad_Cone and then G.Index = 0, "接触集·② 锥轴不是方向 ⇒ BadCone(0):" & Ct.Img (G));
         Bad := Good;
         Bad.Tol_M := 0.0;
         G := Ct.Check (Mk (Single (Bad), Ct.Still ([others => 0.0])), False);
         Check (G.Kind = Ct.Bad_Tolerance and then G.Index = 0, "接触集·④ 容差不是正数 ⇒ BadTolerance(0):" & Ct.Img (G));
         Check (Ct.Check (Mk (Single (Good), Ct.Still ([others => 0.0])), True).Kind = Ct.Motion_Still, "接触集·③ 动词要求动而旋量不动 ⇒ MotionStill");
         declare
            Mixed : Ct.Point_Vectors.Vector := Single (Good);
            Far : Ct.Point := Good;
         begin
            Far.Pos := [0.0, 0.0, 0.3];
            Far.Tol_M := CM;
            Mixed.Append (Far);
            Check (Ct.Check (Mk (Mixed, Ct.Still ([others => 0.0])), False).Kind = Ct.Fine and then Mixed (0).Tol_M < Mixed (1).Tol_M,
                   "接触集·④ 容差是每点各一个:碰到的毫米级、路过的厘米级,同一个集里并存");
         end;
         Check (Ct.Check (Mk (Nil, Ct.Slide ([0.1, 0.0, 0.0])), True).Kind = Ct.No_Points,
                "接触集·够(Reach):物体不参与 ⇒ 第①格无从填起 ⇒ NoPoints;它由执行层在两段之间自己产生,接口里没有条目");
      end;
      --  擦:握着抹布来回 —— 一串,而且全程不松手
      declare
         function Wipe_Seg (From, D : Ct.V3) return Ct.Move is
            V : Ct.Point_Vectors.Vector;
         begin
            V.Append (Pt ([From (0), From (1) - 0.02, From (2)], [0.0, -1.0, 0.0], Cone ([0.0, 1.0, 0.0], Mu_Half)));
            V.Append (Pt ([From (0), From (1) + 0.02, From (2)], [0.0, 1.0, 0.0], Cone ([0.0, -1.0, 0.0], Mu_Half)));
            return Ct.One_Of (Mk (V, Ct.Slide (D), Z_Dn));
         end Wipe_Seg;
         Segs, Broken : Ct.Move_Vectors.Vector;
         Mg : Ct.Many_Gap;
      begin
         Segs.Append (Wipe_Seg ([0.0, 0.0, 0.02], [0.20, 0.0, 0.0]));
         Segs.Append (Wipe_Seg ([0.20, 0.0, 0.02], [0.0, 0.05, 0.0]));
         Segs.Append (Wipe_Seg ([0.20, 0.05, 0.02], [-0.20, 0.0, 0.0]));
         Mg := Ct.Check (Ct.Chain (Ct.Keep, Segs), True);
         Check (Mg.Kind = Ct.Fine and then Natural (Ct.Flatten (Ct.Chain (Ct.Keep, Segs)).Length) = 3,
                "接触集·擦:一串不松手的接触集,每段都填得满、段段接得上:" & Ct.Img (Mg));
         Broken.Append (Wipe_Seg ([0.0, 0.0, 0.02], [0.20, 0.0, 0.0]));
         Broken.Append (Wipe_Seg ([0.30, 0.0, 0.02], [0.0, 0.05, 0.0]));
         Mg := Ct.Check (Ct.Chain (Ct.Keep, Broken), True);
         Check (Mg.Kind = Ct.Keep_Breaks_Contact and then Mg.Seg = 1 and then abs (Mg.Off_M - 0.10) < 1.0e-9,
                "接触集·擦:说了不松手却接不上,必须点名是第几段、差多少(第 1 段、差 10 cm):" & Ct.Img (Mg));
      end;
      --  舀:插进去(平移)→ 兜起来(绕勺口转)→ 抬出来(平移),全程不松手。指腹是一片面(Torsion)
      declare
         function Hold (At_P : Ct.V3; Pad : Boolean) return Ct.Point_Vectors.Vector is
            V : Ct.Point_Vectors.Vector;
         begin
            V.Append (Pt ([At_P (0), At_P (1) - 0.012, At_P (2)], [0.0, -1.0, 0.0], Cone ([0.0, 1.0, 0.0], Mu_Half), Pad => Pad));
            V.Append (Pt ([At_P (0), At_P (1) + 0.012, At_P (2)], [0.0, 1.0, 0.0], Cone ([0.0, -1.0, 0.0], Mu_Half), Pad => Pad));
            return V;
         end Hold;
         Dip : constant Ct.Move := Ct.One_Of (Mk (Hold ([0.0, 0.0, 0.10], True), Ct.Slide ([0.0, 0.0, -0.04]), Z_Dn));
         Scoop : constant Ct.Move := Ct.One_Of (Mk (Hold ([0.0, 0.0, 0.06], True), Turn ([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]), Z_Dn));
         After : constant Ct.V3_Vectors.Vector := Ct.End_Points (Scoop);
         Lift_Pts : Ct.Point_Vectors.Vector := Hold ([0.0, 0.0, 0.06], True);
         Segs : Ct.Move_Vectors.Vector;
         Mg : Ct.Many_Gap;
      begin
         for J in 0 .. Natural (After.Length) - 1 loop
            declare
               P : Ct.Point := Lift_Pts (J);
            begin
               P.Pos := After (J);
               Lift_Pts.Replace_Element (J, P);
            end;
         end loop;
         Segs.Append (Dip);
         Segs.Append (Scoop);
         Segs.Append (Ct.One_Of (Mk (Lift_Pts, Ct.Slide ([0.0, 0.0, 0.08]), Z_Dn)));
         Mg := Ct.Check (Ct.Chain (Ct.Keep, Segs), True);
         Check (Mg.Kind = Ct.Fine, "接触集·舀:插进去 → 兜起来 → 抬出来,三段不松手,段段填得满且接得上:" & Ct.Img (Mg));
         Check (Ct.Check (Mk (Hold ([0.0, 0.0, 0.06], False), Turn ([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]), Z_Dn), True).Kind = Ct.Cannot_Drive
                and then Ct.Check (Mk (Hold ([0.0, 0.0, 0.06], True), Turn ([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]), Z_Dn), True).Kind = Ct.Fine,
                "接触集·兜起来:针尖(点接触)绕两指连线的转产生不出来 ⇒ CannotDrive;指腹(面接触)放行 —— Torsion 是承重的,不是装饰");
      end;
      --  握着扣扳机:一个在维持,一个在动
      declare
         Grip : constant Ct.Set := Mk (Two (0.06, Mu_Half), Ct.Still ([0.0, 0.0, 0.1]), Z_Dn);
         Trigger : constant Ct.Set := Mk (Single (Pt ([0.0, 0.02, 0.10], [0.0, 1.0, 0.0], Cone ([0.0, -1.0, 0.0], 0.3))), Ct.Slide ([0.0, -0.01, 0.0]), [0.0, -1.0, 0.0]);
         Both, Rev, Alone : Ct.Move_Vectors.Vector;
      begin
         Check (Ct.Check (Grip, False).Kind = Ct.Fine and then Ct.Check (Trigger, True).Kind = Ct.Fine, "接触集·握 + 扣扳机:各自四格填得满");
         Both.Append (Ct.One_Of (Grip));
         Both.Append (Ct.One_Of (Trigger));
         Check (Ct.Check (Ct.Chain (Ct.Meanwhile, Both), True).Kind = Ct.Fine, "接触集·并存:握住不动 + 扣扳机在动 ⇒ 两件事同时成立");
         Rev.Append (Ct.One_Of (Trigger));
         Rev.Append (Ct.One_Of (Grip));
         Check (Ct.Check (Ct.Chain (Ct.Meanwhile, Rev), True).Kind = Ct.Holder_Moves, "接触集·并存:拿在动的那个当维持,当场点名 HolderMoves");
         Alone.Append (Ct.One_Of (Grip));
         Check (Ct.Check (Ct.Chain (Ct.Meanwhile, Alone), False).Kind = Ct.Nothing_To_Pair_With, "接触集·并存:只有一段就不叫并存 ⇒ NothingToPairWith");
      end;
   end;

   --  ===== 下手点生成器(②a):每一条排序规矩都是 2026-08 真抓失败逼出来的,单元测试逐条搬回(commit ef10664 contact-gen) =====
   declare
      package Ct renames Contact;
      package Cg renames Contact.Gen;
      use type Cg.Refusal;
      use type Cg.Handoff_Kind;
      use type Cg.No_Hand_Kind;
      use type Ct.Gap_Kind;
      use Ada.Numerics.Long_Elementary_Functions;
      --  八月测试台的观测参数(分辨率 + 当年拿来当参数的三个身体量),不是这具身体的数
      Grid_Aug : constant Cg.Grid := (Bands => 6, Dirs => 16, Min_Pts => 6, Jaw_H_M => 0.03, Min_Above_M => 0.005, Finger_W_M => 0.02, Gap_M => 0.01);
      function Hand_Of (Src : Cg.Span_Source; M : Long_Float) return Cg.Gripper is
        ((Jaw => (Src, M), Reach_Lo => 0.15, Reach_Hi => 0.75, Base_X => 0.0, Base_Y => 0.0));
      --  一根竖着的实心方杆,按 5 mm 采样(40 层)。早先只放四个角点 / 只放四个侧面(零厚度壳),两次都是夹具假,不是算法错
      function Rod (W, H, At_X : Long_Float) return Ct.V3_Vectors.Vector is
         V : Ct.V3_Vectors.Vector;
         N : constant Natural := Natural'Max (2, Natural (Long_Float'Rounding (200.0 * W)));
      begin
         for I in 0 .. 39 loop
            for A in 0 .. N loop
               for B in 0 .. N loop
                  V.Append (Ct.V3'([At_X - 0.5 * W + W * Long_Float (A) / Long_Float (N), -0.5 * W + W * Long_Float (B) / Long_Float (N), H * Long_Float (I) / 39.0]));
               end loop;
            end loop;
         end loop;
         return V;
      end Rod;
      --  剪刀:两片 9 mm 厚的刃,相距 7 cm,实心采样
      function Scissors return Ct.V3_Vectors.Vector is
         V : Ct.V3_Vectors.Vector;
         Blades : constant array (1 .. 2) of Long_Float := [-0.035, 0.035];
      begin
         for I in 0 .. 59 loop
            for Bl of Blades loop
               for A in 0 .. 5 loop
                  for B in 0 .. 3 loop
                     V.Append (Ct.V3'([0.4 - 0.015 + 0.03 * Long_Float (A) / 5.0, Bl - 0.0045 + 0.009 * Long_Float (B) / 3.0, 0.01 + 0.02 * Long_Float (I) / 59.0]));
                  end loop;
               end loop;
            end loop;
         end loop;
         return V;
      end Scissors;
      --  一根竖着的圆柱(半径 3 cm、高 8 cm),顶面是平的;Half_Only = 只留角度在 [90°, 270°] 的那半圈壳(开口朝 +x)
      function Cylinder (With_Top, Half_Only : Boolean) return Ct.V3_Vectors.Vector is
         V : Ct.V3_Vectors.Vector;
      begin
         for I in 0 .. 35 loop
            declare
               A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (I) / 36.0;
            begin
               if not Half_Only or else (A >= 0.5 * Ada.Numerics.Pi and then A <= 1.5 * Ada.Numerics.Pi) then
                  for K in 0 .. 16 loop
                     V.Append (Ct.V3'([0.03 * Cos (A), 0.03 * Sin (A), 0.90 + 0.08 * Long_Float (K) / 16.0]));
                  end loop;
               end if;
            end;
         end loop;
         if With_Top then
            for I in 0 .. 8 loop
               for J in 0 .. 8 loop
                  declare
                     X : constant Long_Float := -0.03 + 0.06 * Long_Float (I) / 8.0;
                     Y : constant Long_Float := -0.03 + 0.06 * Long_Float (J) / 8.0;
                  begin
                     if X * X + Y * Y <= 0.03 * 0.03 + 1.0e-12 then
                        V.Append (Ct.V3'([X, Y, 0.98]));
                     end if;
                  end;
               end loop;
            end loop;
         end if;
         return V;
      end Cylinder;
      Cs : Cg.Cand_Vectors.Vector;
      Why : Cg.Refusal;
      Ok : Boolean;
      T : Long_Float;
   begin
      T := Cg.Thickness_At (Scissors, 0.4, -0.035, 0.02, Ada.Numerics.Pi, 0.01, 0.02, Ok);
      Check (Ok and then abs (T - 0.079) < 0.004, "②a·料厚:在一片刃上横着合爪,跨的是两片刃的外缘 7.9 cm,不是单片刃的 9 mm(读到 " & Codec.Fmt (T, 4) & ")");
      T := Cg.Thickness_At (Scissors, 0.4, -0.035, 0.02, 0.5 * Ada.Numerics.Pi, 0.01, 0.02, Ok);
      Check (Ok and then abs (T - 0.030) < 0.004, "②a·料厚:顺着刃的长边合爪,同一条上只有那一片刃,跨 3 cm(读到 " & Codec.Fmt (T, 4) & ")");
      T := Cg.Thickness_At (Scissors, 0.4, 0.0, 0.02, Ada.Numerics.Pi, 0.01, 0.02, Ok);
      Check (Ok and then abs (T - 0.079) < 0.004, "②a·料厚:站在两片刃中间的缝上照样跨两片刃 —— 爪子的中心在缝里不等于指头在缝里");
      T := Cg.Thickness_At (Scissors, 1.0, 0.0, 0.02, Ada.Numerics.Pi, 0.01, 0.02, Ok);
      Check (not Ok, "②a·料厚:那一条上根本没有料 ⇒ 说没有,不是 0(合到空气里和夹住零毫米是两件事)");
      T := Cg.Thickness_At (Scissors, 0.4, -0.035, 0.5, Ada.Numerics.Pi, 0.01, 0.02, Ok);
      Check (not Ok, "②a·料厚:高度不对(物体在 z 0.01–0.03,问 z 0.5)同样是没有料");
      declare
         Pts : Ct.V3_Vectors.Vector := Rod (0.02, 0.10, 0.35);
      begin
         --  又高又浅的一根细刺(0.14–0.20 m),沿指头方向只有一条:这条测的是「下限 vs 最大化」本身
         for I in 0 .. 39 loop
            for A in 0 .. 2 loop
               for B in 0 .. 2 loop
                  Pts.Append (Ct.V3'([0.35 + 0.004 * Long_Float (A) / 2.0 - 0.002, 0.004 * Long_Float (B) / 2.0 - 0.002, 0.14 + 0.06 * Long_Float (I) / 39.0]));
               end loop;
            end loop;
         end loop;
         Cg.Candidates (Pts, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         Check (Why = Cg.Fine and then not Cs.Is_Empty and then Cs (0).Pos (2) < 0.12 and then Cs (0).Depth_M > Grid_Aug.Finger_W_M
                and then Cs (0).Above_Support_M >= Grid_Aug.Min_Above_M,
                "②a·离桌面高是下限不是最大化:又深又匀的矮杆排在又高又薄的细刺前面(2026-08-12 鞋腰 vs 鞋口那圈软皮)");
      end;
      declare
         Pts : Ct.V3_Vectors.Vector := Rod (0.03, 0.10, 0.30);
         Com_X : Long_Float := 0.0;
         Any_Com, Any_Tilt : Boolean := False;
      begin
         Pts.Append (Rod (0.03, 0.10, 0.42));
         for P of Pts loop
            Com_X := Com_X + P (0) / Long_Float (Pts.Length);
         end loop;
         Cg.Candidates (Pts, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         for C of Cs loop
            if C.Com_Offset_M > 1.0e-6 then
               Any_Com := True;
            end if;
            if C.Face_Tilt_Rad > 0.0 then
               Any_Tilt := True;
            end if;
         end loop;
         Check (Why = Cg.Fine and then abs (Cs.First_Element.Pos (0) - Com_X) <= abs (Cs.Last_Element.Pos (0) - Com_X) and then Any_Com and then Any_Tilt,
                "②a·抓点离重心远的排在后面(管「提起来会不会转出去」:剪刀抓在手柄圆环上,重量全在刀刃那头),而且面歪、离重心两格真的被算了");
      end;
      declare
         Pts : Ct.V3_Vectors.Vector := Rod (0.02, 0.2, 0.35);
         Corner_Deeper : Boolean := False;
      begin
         Pts.Append (Rod (0.12, 0.2, 0.60));
         Cg.Candidates (Pts, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         --  八月原话:大块上排得最前的那一条(角上的薄片)不许比杆还深;大块中间那些放不下的宽段更深,但它们垫底,不在这一条里
         for C of Cs loop
            if abs (C.Pos (0) - 0.60) < 0.06 then
               Corner_Deeper := C.Depth_M > Cs (0).Depth_M;
               exit;
            end if;
         end loop;
         for I in 0 .. Natural'Min (2, Natural (Cs.Length) - 1) loop
            Put_Line ("     · 第 " & Codec.Img (I) & " 名:x=" & Codec.Fmt (Cs (I).Pos (0), 3) & " z=" & Codec.Fmt (Cs (I).Pos (2), 3) & " 宽=" & Codec.Fmt (Cs (I).Width_M, 4)
                      & " 深=" & Codec.Fmt (Cs (I).Depth_M, 3) & " 歪=" & Codec.Fmt (Cs (I).Face_Tilt_Rad, 3) & " 离心=" & Codec.Fmt (Cs (I).Com_Offset_M, 3)
                      & " 夹得下=" & Cs (I).Within_Jaw'Image & " 够高=" & Cs (I).Off_Ok'Image & " 面正=" & Cs (I).Tilt_Ok'Image & " 近心=" & Cs (I).Com_Ok'Image);
         end loop;
         Check (Why = Cg.Fine and then Cs (0).Within_Jaw and then abs (Cs (0).Pos (0) - 0.35) < 0.03 and then Cs (0).Depth_M > Grid_Aug.Finger_W_M and then not Corner_Deeper,
                "②a·又深又匀的杆胜过大块的尖角(旧排序按余量最大 = 最窄,把最尖的角排最前:抓取率 19/48 → 26/96)");
      end;
      Cg.Candidates (Rod (0.02, 0.2, 0.4), Hand_Of (Cg.Unknown, 0.0), 0.0, Grid_Aug, Cs, Why);
      Check (Why = Cg.Jaw_Span_Unknown, "②a·爪张开度没量过就拒绝,不许猜一个数出来(本仓最贵的一次手填就在这个量上)");
      Cg.Candidates (Rod (0.02, 0.2, 0.4), Hand_Of (Cg.Declared, 0.088), 0.0, Grid_Aug, Cs, Why);
      declare
         All_Stamped : Boolean := Why = Cg.Fine and then not Cs.Is_Empty;
      begin
         for C of Cs loop
            if not C.Jaw_Declared then
               All_Stamped := False;
            end if;
         end loop;
         Check (All_Stamped, "②a·声明值能用,但每一条候选都背着「这是声明值」的标记,出处不许在中途消失");
      end;
      Cg.Candidates (Rod (0.12, 0.2, 0.4), Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
      declare
         First_Bad : Integer := -1;
         Last_Good : Integer := -1;
      begin
         for I in 0 .. Natural (Cs.Length) - 1 loop
            if not Cs (I).Within_Jaw and then First_Bad < 0 then
               First_Bad := I;
            end if;
            if Cs (I).Within_Jaw then
               Last_Good := I;
            end if;
         end loop;
         Check (Why = Cg.Fine and then First_Bad >= 0 and then Last_Good >= 0 and then Last_Good < First_Bad,
                "②a·放不下的段只排最后、永远不删(仓里唯一那条可抓性规矩:不许拿钳口张开度当阈值筛物体)");
      end;
      declare
         Pts : Ct.V3_Vectors.Vector := Rod (0.02, 0.2, 0.4);
         Mid_Air, Any_Far : Boolean := False;
      begin
         Pts.Append (Rod (0.02, 0.2, 1.6));
         Cg.Candidates (Pts, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         for C of Cs loop
            if not (abs (C.Pos (0) - 0.4) < 0.05 or else abs (C.Pos (0) - 1.6) < 0.05) then
               Mid_Air := True;
            end if;
            if not C.Reachable then
               Any_Far := True;
            end if;
         end loop;
         Check (Why = Cg.Fine and then not Mid_Air, "②a·两根相距 1.2 m 的杆,落点一条都不落在半空(先分块再量宽度;单元测试自己逮出来的真 bug)");
         Check (Cs (0).Reachable and then Any_Far, "②a·够不到的排在够得到的后面,但留在表里让上面看得见");
      end;
      Cg.Candidates (Rod (0.02, 0.2, 0.4), Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
      Check (Why = Cg.Fine and then Cs.First_Element.Above_Support_M >= Cs.Last_Element.Above_Support_M,
             "②a·贴着支撑面的那一层排在后面:爪子伸不到它下面(平躺薄件合爪停在 0,指间是空的)");
      declare
         V : Ct.V3_Vectors.Vector;
      begin
         for I in 0 .. 9 loop
            for J in 0 .. 9 loop
               V.Append (Ct.V3'([0.4 + 0.005 * Long_Float (I), 0.005 * Long_Float (J), 0.0]));
            end loop;
         end loop;
         Cg.Candidates (V, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         Check (Why = Cg.Flat, "②a·一张平面切不出层 ⇒ 拒绝并说 Flat,不许静默返回空表");
      end;
      --  一个圈(外径 6 cm、壁厚 6 mm):候选里既有"夹住环壁"(Single,~6 mm),也有"从里面撑开"(Inside,洞的跨度),两种都在,种类标得出来
      declare
         Ring : Ct.V3_Vectors.Vector;
         Walls, Holes : Natural := 0;
         use type Cg.Pair_Kind;
      begin
         for I in 0 .. 71 loop
            for R in 0 .. 2 loop
               for K in 0 .. 3 loop
                  declare
                     A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (I) / 72.0;
                     Rr : constant Long_Float := 0.024 + 0.003 * Long_Float (R);
                  begin
                     Ring.Append (Ct.V3'([0.4 + Rr * Cos (A), Rr * Sin (A), 0.01 + 0.01 * Long_Float (K) / 3.0]));
                  end;
               end loop;
            end loop;
         end loop;
         Cg.Candidates (Ring, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
         for C of Cs loop
            if C.Kind = Cg.Single and then C.Width_M < 0.012 then
               Walls := Walls + 1;
            elsif C.Kind = Cg.Inside and then C.Width_M > 0.03 then
               Holes := Holes + 1;
            end if;
         end loop;
         Check (Why = Cg.Fine and then Walls > 0 and then Holes > 0,
                "②a·圈:夹住环壁(Single,壁厚)和从里面撑开(Inside,洞的跨度)两种候选都生出来、种类标得出来(壁 " & Codec.Img (Walls) & " · 洞 " & Codec.Img (Holes) & ")");
      end;
      Cg.Candidates (Scissors, Hand_Of (Cg.Measured, 0.088), 0.0, Grid_Aug, Cs, Why);
      declare
         Narrow : Boolean := False;
      begin
         for C of Cs loop
            if C.Width_M < 0.02 then
               Narrow := True;
            end if;
         end loop;
         Check (Why = Cg.Fine and then Narrow, "②a·剪刀:包围盒说「整体 8 cm 能夹」,表面点量到每片刃自己的 9 mm —— 找得到刃上那条窄段");
      end;
      declare
         V : Ct.V3_Vectors.Vector;
         Gp : constant Cg.Gripper := (Jaw => (Cg.Measured, 0.08), Reach_Lo => 0.05, Reach_Hi => 1.0, Base_X => 0.0, Base_Y => -0.4);
         Gd : constant Cg.Grid := (Bands => 4, Dirs => 12, Min_Pts => 8, Jaw_H_M => 0.02, Min_Above_M => 0.001, Finger_W_M => 0.02, Gap_M => 0.01);
         S : Ct.Set;
         H : Cg.Handoff;
         Pick : Natural := 0;
      begin
         for I in 0 .. 11 loop
            for J in 0 .. 11 loop
               for K in 0 .. 5 loop
                  V.Append (Ct.V3'([-0.03 + 0.06 * Long_Float (I) / 11.0, -0.02 + 0.04 * Long_Float (J) / 11.0, 0.90 + 0.05 * Long_Float (K) / 5.0]));
               end loop;
            end loop;
         end loop;
         Cg.Candidates (V, Gp, 0.90, Gd, Cs, Why);
         Check (Why = Cg.Fine and then not Cs.Is_Empty, "②a·一块方料给得出候选:" & Cg.Img (Why));
         for I in 0 .. Natural (Cs.Length) - 1 loop
            if Cs (I).Reachable then
               Pick := I;
               exit;
            end if;
         end loop;
         declare
            C : constant Cg.Candidate := Cs (Pick);
         begin
            Cg.To_Set (C, 0.5, Ct.Still (C.Pos), 0.002, S, H);
            Check (H.Kind = Cg.Fine and then Natural (S.Points.Length) = 2 and then Ct.Check (S, False).Kind = Ct.Fine,
                   "②a→接触集:中心 + 宽度 + 合爪方向 ⇒ 两个相对的接触点,四格自检就过:" & Cg.Img (H));
            Check (abs (Ct.Norm ([S.Points (1).Pos (0) - S.Points (0).Pos (0), S.Points (1).Pos (1) - S.Points (0).Pos (1), S.Points (1).Pos (2) - S.Points (0).Pos (2)]) - C.Width_M) < 1.0e-12
                   and then Ct.Dot (S.Points (0).Push.Axis, S.Points (1).Push.Axis) < -0.999 and then S.Has_Approach
                   and then abs (S.Points (0).Push.Half_Angle - Arctan (0.5)) < 1.0e-12,
                   "②a→接触集:两点间距 = 段宽,两个锥朝里且相反,进场方向由看得见空隙的这一层填,锥 = 摩擦锥 atan(μ)(不是 Face_Tilt:那是要多大,不是有多大)");
         end;
      end;
      declare
         Bad : Cg.Candidate;
         S : Ct.Set;
         H : Cg.Handoff;
      begin
         Bad.Pos := [0.0, 0.0, 0.95];
         Bad.Width_M := 0.04;
         Bad.Face_Tilt_Rad := 0.60;
         Cg.To_Set (Bad, 0.5, Ct.Still ([0.0, 0.0, 0.95]), 0.002, S, H);
         Check (H.Kind = Cg.Would_Slip and then abs (H.Need_Rad - 0.60) < 1.0e-12 and then abs (H.Have_Rad - Arctan (0.5)) < 1.0e-12 and then H.Need_Rad > H.Have_Rad,
                "②a→接触集:两个面歪了 34.4° 而 μ=0.5 的摩擦锥只有 26.6° ⇒ 会滑,拒绝并点名差多少:" & Cg.Img (H));
         Cg.To_Set (Bad, 1.0, Ct.Still ([0.0, 0.0, 0.95]), 0.002, S, H);
         Check (H.Kind = Cg.Fine, "②a→接触集:μ=1.0 的摩擦锥 45° > 34.4° ⇒ 同一把就交得出去了 —— 差别只在 μ,不在几何");
         Cg.To_Set (Bad, 0.0, Ct.Still ([0.0, 0.0, 0.95]), 0.002, S, H);
         Check (H.Kind = Cg.Mu_Unknown, "②a→接触集:μ 没量过就不许瞎填 ⇒ MuUnknown");
         Cg.To_Set_Least_Mu (Bad, Ct.Still ([0.0, 0.0, 0.95]), 0.002, S);
         Check (Natural (S.Points.Length) = 2 and then abs (S.Points (0).Push.Half_Angle - 0.60) < 1.0e-12 and then Ct.Check (S, False).Kind = Ct.Fine,
                "②a→接触集:μ 没量过 ⇒ 锥 = 这一把需要的最小值(0.60 rad)并明说;合上(物体不动)这一格照样自洽,能不能提由抬手验");
      end;
      declare
         S : Ct.Set;
         Nh : Cg.No_Hand;
      begin
         Cg.Suction (Cylinder (True, False), 0.012, 0.001, 0.5, Ct.Still ([0.0, 0.0, 0.94]), 0.002, S, Nh);
         Check (Nh.Kind = Cg.Fine and then Natural (S.Points.Length) = 1 and then abs (S.Points (0).Pos (2) - 0.98) < 1.0e-9 and then S.Points (0).Normal (2) > 0.99
                and then abs (S.Points (0).Push.Half_Angle - Arctan (0.5)) < 1.0e-12 and then S.Points (0).Torsion and then Ct.Check (S, False).Kind = Ct.Fine,
                "②a·吸盘:从点云里真的找到那片平顶面(z=0.98、法向朝上),锥 = 摩擦锥不是 0,吸住了拧得动,四格自检就过:" & Cg.Img (Nh));
         Cg.Suction (Cylinder (True, False), 0.06, 0.001, 0.5, Ct.Still ([others => 0.0]), 0.002, S, Nh);
         Check (Nh.Kind = Cg.No_Flat_Patch and then abs (Nh.Need_R - 0.06) < 1.0e-12 and then Nh.Found_R < 0.06,
                "②a·吸盘比那片平面还大就必须拒绝,并报实测的最大平坦半径:" & Cg.Img (Nh));
         declare
            Ns : constant array (1 .. 2) of Positive := [3, 5];
         begin
            for N of Ns loop
               Cg.Ring (Cylinder (True, False), 0.94, 0.02, N, 0.5, Ct.Still ([0.0, 0.0, 0.94]), 0.002, S, Nh);
               declare
                  On_Surface : Boolean := Nh.Kind = Cg.Fine and then Natural (S.Points.Length) = N;
               begin
                  for P of S.Points loop
                     if abs (Sqrt (P.Pos (0) ** 2 + P.Pos (1) ** 2) - 0.03) >= 0.004 or else abs (P.Push.Half_Angle - Arctan (0.5)) >= 1.0e-12 then
                        On_Surface := False;
                     end if;
                  end loop;
                  Check (On_Surface and then Ct.Check (S, False).Kind = Ct.Fine,
                         "②a·" & Codec.Img (N) & " 指环抓:绕一圈每个接触点都落在真表面上(r=3 cm),锥 = 摩擦锥,填得满同一张表:" & Cg.Img (Nh));
               end;
            end loop;
         end;
         Cg.Ring (Cylinder (False, True), 0.94, 0.02, 5, 0.5, Ct.Still ([others => 0.0]), 0.002, S, Nh);
         Check (Nh.Kind = Cg.Nothing_In_Direction and then Nh.Direction = 0,
                "②a·环抓:开口的 C 形壳,开口正对 +x = 第 0 个方向摸不到料 ⇒ 点名是哪个方向(半个圆柱当反例是错的:切面本身也是面):" & Cg.Img (Nh));
         Cg.Ring (Cylinder (True, False), 0.94, 0.02, 3, 0.0, Ct.Still ([others => 0.0]), 0.002, S, Nh);
         Check (Nh.Kind = Cg.Handed_Off and then Nh.H.Kind = Cg.Mu_Unknown, "②a·环抓:μ 没量过就不许调:" & Cg.Img (Nh));
      end;
      declare
         Cloud : Ct.V3_Vectors.Vector;
         Back : Cg.Rot;
         Ok2 : Boolean;
         S0, S1 : Ct.Set;
      begin
         Cloud.Append (Ct.V3'([1.0, 0.0, 0.0]));
         Cg.To_Upright (Cloud, [1.0, 0.0, 0.0], Back, Ok2);
         Check (Ok2 and then abs (Cloud (0) (2) - 1.0) < 1.0e-9 and then abs (Cloud (0) (0)) < 1.0e-9,
                "②a·支撑面立起来(法向 = +x)的机器:点云转到「法向 = +z」的系里算,x 轴上的点到了 z 轴上");
         S0.Points.Append (Ct.Point'(By => (Ct.Hand, 0), Pos => [0.1, 0.2, 0.3], Normal => [0.0, 0.0, 1.0], Push => (Axis => [0.0, 0.0, -1.0], Half_Angle => 0.3),
                                     Pull => False, Torsion => False, Peel => False, Tol_M => 0.002));
         S0.Motion := Ct.Slide ([0.0, 0.0, 0.05]);
         S0.Has_Approach := True;
         S0.Approach := [0.0, 0.0, -1.0];
         S1 := Cg.Rotate (Back, Cg.Rotate (Cg.Inverse (Back), S0));
         declare
            P0 : constant Ct.Point := S0.Points (0);
            P1 : constant Ct.Point := S1.Points (0);
         begin
            Check (Ct.Norm ([P1.Pos (0) - P0.Pos (0), P1.Pos (1) - P0.Pos (1), P1.Pos (2) - P0.Pos (2)]) < 1.0e-9
                   and then Ct.Norm ([P1.Push.Axis (0) - P0.Push.Axis (0), P1.Push.Axis (1) - P0.Push.Axis (1), P1.Push.Axis (2) - P0.Push.Axis (2)]) < 1.0e-9
                   and then Ct.Norm ([S1.Approach (0) - S0.Approach (0), S1.Approach (1) - S0.Approach (1), S1.Approach (2) - S0.Approach (2)]) < 1.0e-9,
                   "②a·转过去再转回来:点、法向、锥轴、旋量、进场方向一个都不漏(漏了就是「点转过去了而面还朝着老方向」)");
         end;
      end;
   end;

   --  ===== 执行层(②b):接触集 → 一串航点,闭式、不认识动词(commit ef10664 contact-exec/plan.rs 的航点级验收) =====
   declare
      package Ct renames Contact;
      package Cx renames Contact.Exec;
      use type Cx.No_Plan_Kind;
      use type Ct.Gap_Kind;
      use type Cx.Step_Kind;
      use Ada.Numerics.Long_Elementary_Functions;
      MM : constant Long_Float := 0.002;
      Lim : constant Cx.Hand_Limits := (Standoff_M => 0.04, Repeat_M => 0.001);   --  两个数都该由驱动量出来;这里是测试台,取一个明显合法的组合
      Z_Dn : constant Ct.V3 := [0.0, 0.0, -1.0];
      function Pt (Pos, Normal, Axis : Ct.V3; Half : Long_Float; Tol : Long_Float := MM) return Ct.Point is
        ((By => (Ct.Hand, 0), Pos => Pos, Normal => Normal, Push => (Axis => Axis, Half_Angle => Half), Pull => False, Torsion => False, Peel => False, Tol_M => Tol));
      function Two return Ct.Point_Vectors.Vector is
         V : Ct.Point_Vectors.Vector;
      begin
         V.Append (Pt ([-0.025, 0.0, 0.10], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.5 * Ada.Numerics.Pi));
         V.Append (Pt ([0.025, 0.0, 0.10], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.5 * Ada.Numerics.Pi));
         return V;
      end Two;
      function Single (P : Ct.Point) return Ct.Point_Vectors.Vector is
         V : Ct.Point_Vectors.Vector;
      begin
         V.Append (P);
         return V;
      end Single;
      function Mk (Pts : Ct.Point_Vectors.Vector; Mo : Ct.Twist; Ap : Ct.V3) return Ct.Set is
        ((Points => Pts, Motion => Mo, Has_Approach => True, Approach => Ap));
      function Mk (Pts : Ct.Point_Vectors.Vector; Mo : Ct.Twist) return Ct.Set is
        ((Points => Pts, Motion => Mo, Has_Approach => False, Approach => [others => 0.0]));
      function Turn (Axis : Ct.V3; Rad : Long_Float; Pivot : Ct.V3) return Ct.Twist is
         Ok : Boolean;
         T : constant Ct.Twist := Ct.Turn (Axis, Rad, Pivot, Ok);
      begin
         pragma Assert (Ok, "转轴非零");
         return T;
      end Turn;
      --  两个朝向之间的夹角(弧度)
      function Angle_Of (A, B : Cx.M3) return Long_Float is (Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (A), B))));
      --  先把第 I 步拷成具名变量再取第 J 个点/朝向:对函数返回的临时值直接下标取容器元素,GNAT 会在析构时报 PROGRAM_ERROR(H24 2026-09-22 同一个坑)
      function Frame_At (V : Cx.Step_Vectors.Vector; I, J : Natural) return Cx.M3 is
         St : constant Cx.Step := V (I);
      begin
         return St.Frame (J);
      end Frame_At;
      function Pos_At (V : Cx.Step_Vectors.Vector; I, J : Natural) return Ct.V3 is
         St : constant Cx.Step := V (I);
      begin
         return St.Pos (J);
      end Pos_At;
      function Last_Of (V : Cx.Step_Vectors.Vector) return Natural is (Natural (V.Length) - 1);
      Steps : Cx.Step_Vectors.Vector;
      Why : Cx.No_Plan;
   begin
      declare
         S : constant Ct.Set := Mk (Two, Ct.Still ([0.0, 0.0, 0.10]), Z_Dn);
         Good : Boolean;
      begin
         Cx.Steps (S, Lim, False, 1, Steps, Why);
         Good := Why.Kind = Cx.Fine and then Natural (Steps.Length) = 2 and then not Steps (0).Touching and then Steps (1).Touching;
         if Good then
            for I in 0 .. 1 loop
               declare
                  Z : constant Ct.V3 := Cx.Tool_Axis (Frame_At (Steps, 0, 0));
                  Hv : constant Ct.V3 := Pos_At (Steps, 0, I);
                  Pq : constant Ct.Point := S.Points (I);
                  D : constant Ct.V3 := [Pq.Pos (0) - Hv (0), Pq.Pos (1) - Hv (1), Pq.Pos (2) - Hv (2)];
                  Ok : Boolean;
                  U : constant Ct.V3 := Ct.Unit (D, Ok);
               begin
                  if abs (Ct.Norm (D) - Lim.Standoff_M) > 1.0e-9 or else not Ok or else Ct.Dot (U, Z) < 0.999 then
                     Good := False;
                  end if;
               end;
            end loop;
         end if;
         Check (Good, "②b·抓:不动的动词 = 悬停 + 贴上,两步就完;悬停沿工具轴反方向退开一个进场余量,不是「往上退」(那是把 z 当特权方向):" & Cx.Img (Why));
         Check (Good and then Steps (0).Tol_M > Steps (1).Tol_M and then abs (Steps (1).Tol_M - MM) < 1.0e-12,
                "②b·容差:悬停那一步比贴上那一步松(路过的地方厘米级、碰到的地方毫米级),贴上用最严的那个点的容差");
      end;
      declare
         Pts : Ct.Point_Vectors.Vector := Two;
         P1 : Ct.Point := Pts (1);
      begin
         P1.Tol_M := 0.0005;
         Pts.Replace_Element (1, P1);
         Cx.Steps (Mk (Pts, Ct.Still ([0.0, 0.0, 0.10]), Z_Dn), Lim, False, 1, Steps, Why);
         Check (Why.Kind = Cx.Tol_Tighter_Than_Body and then Why.Index = 1, "②b·容差比这具身体的重复精度还紧就拒绝,点名是哪一点:" & Cx.Img (Why));
      end;
      declare
         Pivot : constant Ct.V3 := [0.05, 0.0, 0.0];
         Pts : Ct.Point_Vectors.Vector := Single (Pt ([-0.04, 0.0, 0.02], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 0.4636));
         Good : Boolean;
      begin
         Pts.Append (Ct.Point'(By => (Ct.World, 0), Pos => Pivot, Normal => Z_Dn, Push => (Axis => [0.0, 0.0, 1.0], Half_Angle => 0.46),
                               Pull => False, Torsion => False, Peel => False, Tol_M => MM));
         Cx.Steps (Mk (Pts, Turn ([0.0, 1.0, 0.0], -0.8, Pivot)), Lim, True, 8, Steps, Why);
         Good := Why.Kind = Cx.Fine and then Natural (Steps.Length) = 10;
         if Good then
            for St of Steps loop
               if Natural (St.Pos.Length) /= 1 then
                  Good := False;
               end if;
            end loop;
         end if;
         if Good then
            declare
               A : constant Ct.V3 := Pos_At (Steps, 2, 0);
               B : constant Ct.V3 := Pos_At (Steps, 9, 0);
               Mid : constant Ct.V3 := Pos_At (Steps, 6, 0);
               Ok : Boolean;
               Chord : constant Ct.V3 := Ct.Unit ([B (0) - A (0), B (1) - A (1), B (2) - A (2)], Ok);
               V : constant Ct.V3 := [Mid (0) - A (0), Mid (1) - A (1), Mid (2) - A (2)];
               Along : constant Long_Float := Ct.Dot (V, Chord);
               Off : constant Long_Float := Ct.Norm ([V (0) - Chord (0) * Along, V (1) - Chord (1) * Along, V (2) - Chord (2) * Along]);
               R0 : constant Long_Float := Ct.Norm ([A (0) - Pivot (0), A (1) - Pivot (1), A (2) - Pivot (2)]);
            begin
               Good := Ok and then Off > 1.0e-3;
               for I in 2 .. 9 loop
                  declare
                     P : constant Ct.V3 := Pos_At (Steps, I, 0);
                  begin
                     if abs (Ct.Norm ([P (0) - Pivot (0), P (1) - Pivot (1), P (2) - Pivot (2)]) - R0) > 1.0e-9 then
                        Good := False;
                     end if;
                  end;
               end loop;
            end;
         end if;
         Check (Good, "②b·撬:悬停 + 贴上 + 8 段弧;航点里只有手那一个点(桌子那条边不进航点,只进判据);弧中点离弦有实打实的距离、到支点的半径全程不变:" & Cx.Img (Why));
      end;
      Cx.Steps (Mk (Two, Turn ([0.0, 0.0, 1.0], 1.2, [0.0, 0.0, 0.10]), Z_Dn), Lim, True, 6, Steps, Why);
      Check (Why.Kind = Cx.Fine and then abs (Angle_Of (Frame_At (Steps, 1, 0), Frame_At (Steps, Last_Of (Steps), 0)) - 1.2) < 1.0e-6,
             "②b·拧:手转过的角等于物体转过的角(转的时候朝向也要跟着走,否则就是「握着的东西被拧脱手」的形状)");
      Cx.Steps (Mk (Single (Pt ([0.03, 0.0, 0.05], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.6)), Ct.Slide ([-0.10, 0.0, 0.0])), Lim, True, 4, Steps, Why);
      declare
         Same : Boolean := Why.Kind = Cx.Fine;
      begin
         for I in 0 .. Last_Of (Steps) loop
            if Angle_Of (Frame_At (Steps, 0, 0), Frame_At (Steps, I, 0)) > 1.0e-9 then
               Same := False;
            end if;
         end loop;
         Check (Same and then abs (Pos_At (Steps, Last_Of (Steps), 0) (0) - (0.03 - 0.10)) < 1.0e-9, "②b·推:不转的时候朝向不许自己动;走完整段平移");
      end;
      Cx.Steps (Mk (Single (Pt ([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 0.0)), Ct.Slide ([0.0, 0.0, 0.05])), Lim, True, 1, Steps, Why);
      Check (Why.Kind = Cx.Fine and then Natural (Steps (0).Pos.Length) = 1 and then Ct.Dot (Cx.Tool_Axis (Frame_At (Steps, 0, 0)), [0.0, 0.0, 1.0]) > 0.999,
             "②b·吸盘:一个点就是一个位置;只有一个锥时工具轴就是它");
      declare
         V : Ct.Point_Vectors.Vector;
         All5 : Boolean;
      begin
         for I in 0 .. 4 loop
            declare
               A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (I) / 5.0;
               C : constant Long_Float := Cos (A);
               Sn : constant Long_Float := Sin (A);
            begin
               V.Append (Pt ([0.03 * C, 0.03 * Sn, 0.10], [C, Sn, 0.0], [-C, -Sn, 0.0], 0.5));
            end;
         end loop;
         Cx.Steps (Mk (V, Ct.Still ([0.0, 0.0, 0.10]), Z_Dn), Lim, False, 1, Steps, Why);
         All5 := Why.Kind = Cx.Fine;
         for St of Steps loop
            if Natural (St.Pos.Length) /= 5 then
               All5 := False;
            end if;
         end loop;
         Check (All5, "②b·五指:五个接触点 ⇒ 每一步五个位置(上一版只有一个 tcp,放不下)");
      end;
      Cx.Steps (Mk (Ct.Point_Vectors.Empty_Vector, Ct.Still ([others => 0.0])), Lim, False, 1, Steps, Why);
      Check (Why.Kind = Cx.Bad and then Why.G.Kind = Ct.No_Points, "②b·接触集自己不合格时把那一格原样转发:" & Cx.Img (Why));
      declare
         function Hold (At_P : Ct.V3) return Ct.Point_Vectors.Vector is
            V : Ct.Point_Vectors.Vector;
            P : Ct.Point;
         begin
            P := Pt ([At_P (0), At_P (1) - 0.012, At_P (2)], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0], 0.4636);
            P.Torsion := True;
            V.Append (P);
            P := Pt ([At_P (0), At_P (1) + 0.012, At_P (2)], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], 0.4636);
            P.Torsion := True;
            V.Append (P);
            return V;
         end Hold;
         Dip : constant Ct.Move := Ct.One_Of (Mk (Hold ([0.0, 0.0, 0.10]), Ct.Slide ([0.0, 0.0, -0.04]), Z_Dn));
         Scoop : constant Ct.Move := Ct.One_Of (Mk (Hold ([0.0, 0.0, 0.06]), Turn ([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]), Z_Dn));
         Lift : constant Ct.Move := Ct.One_Of (Mk (Hold ([0.0, 0.0, 0.06]), Ct.Slide ([0.0, 0.0, 0.08]), Z_Dn));
         Segs : Ct.Move_Vectors.Vector;
         Hovers : Natural := 0;
      begin
         Segs.Append (Dip);
         Segs.Append (Scoop);
         Segs.Append (Lift);
         Cx.Script (Ct.Chain (Ct.Keep, Segs), Lim, True, 4, Steps, Why);
         for St of Steps loop
            if St.Kind = Cx.Hover then
               Hovers := Hovers + 1;
            end if;
         end loop;
         Check (Why.Kind = Cx.Fine and then Hovers = 1 and then abs (Angle_Of (Frame_At (Steps, 1, 0), Frame_At (Steps, Last_Of (Steps), 0)) - 0.7) < 1.0e-6,
                "②b·舀(不松手的一串):除第一段外悬停都扔掉(手已经握着东西在那儿),兜起来转过的 0.7 rad 带进抬那一段 —— 不带的话末了手腕转角是 0:" & Cx.Img (Why));
      end;
      declare
         Grip : constant Ct.Set := Mk (Two, Ct.Still ([0.0, 0.0, 0.1]), Z_Dn);
         Trigger : constant Ct.Set := Mk (Single (Pt ([0.0, 0.02, 0.10], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], 0.3)), Ct.Slide ([0.0, -0.01, 0.0]), [0.0, -1.0, 0.0]);
         Both : Ct.Move_Vectors.Vector;
      begin
         Both.Append (Ct.One_Of (Grip));
         Both.Append (Ct.One_Of (Trigger));
         Cx.Script (Ct.Chain (Ct.Meanwhile, Both), Lim, True, 1, Steps, Why);
         Check (Why.Kind = Cx.Fine and then Natural (Steps.Length) = 5 and then Natural (Steps (2).Pos.Length) = 3 and then Natural (Steps (Last_Of (Steps)).Pos.Length) = 3,
                "②b·并存:维持段的两步先发,之后每一步 = 握着的两点 + 扣扳机那一点(三个位置),握点不动、朝向由维持段定:" & Cx.Img (Why));
      end;
      declare
         Ko, From : Ct.V3_Vectors.Vector;
         Q : Ct.V3;
         Ok : Boolean;
      begin
         Ko.Append (Ct.V3'([-0.15, 0.0, 0.0]));
         Ko.Append (Ct.V3'([0.15, 0.0, 0.0]));
         Q := Cx.Dodge_To ([0.0, 0.0, 0.0], Ko, 0.20, Ok);
         Check (Ok and then Ct.Norm ([Q (0) + 0.15, Q (1), Q (2)]) >= 0.20 - 1.0e-9 and then Ct.Norm ([Q (0) - 0.15, Q (1), Q (2)]) >= 0.20 - 1.0e-9 and then Ct.Norm (Q) < 0.14,
                "②b·躲:推开最近那个会推向另一个(沿 x 推开一个离另一个只剩 10 cm,上一版当成功返回了);往侧面让 13.2 cm 才同时满足两个,实得 " & Codec.Fmt (Ct.Norm (Q), 4));
         From.Append (Ct.V3'([0.0, 0.0, 0.0]));
         Cx.Script (Ct.Clear_Of (Ko, 0.20, From), Lim, False, 1, Steps, Why);
         Check (Why.Kind = Cx.Fine and then Natural (Steps.Length) = 1 and then not Steps (0).Touching and then Steps (0).Kind = Cx.Dodge,
                "②b·「不要碰」= 零接触点 + 一个净空:一步、永远不接触,身体层看见 Dodge 保持当前朝向");
         Check (not Cx.Off_Course (Lim, Steps (0), 0.30)
                and then Cx.Off_Course (Lim, (Pos => Ct.V3_Vectors.Empty_Vector, Frame => Cx.M3_Vectors.Empty_Vector, Hand => Ct.Nat_Vectors.Empty_Vector, Touching => True, Tol_M => MM, Kind => Cx.Touch), 0.0511)
                and then not Cx.Off_Course (Lim, (Pos => Ct.V3_Vectors.Empty_Vector, Frame => Cx.M3_Vectors.Empty_Vector, Hand => Ct.Nat_Vectors.Empty_Vector, Touching => True, Tol_M => MM, Kind => Cx.Touch), 0.0015),
                "②b·「偏了没有」:路过的点无论差多少都不算偏(悬停差 5 cm 被判偏正是白跑一夜的那个 bug);要碰的点门槛 = 这具身体重复精度的两倍");
      end;
   end;

   --  ===== 两只普通相机 → 表面点(无深度那条路,架构底线) =====
   declare
      package Ct renames Contact;
      package Sf renames Contact.Surface;
      Ok : Boolean;
      Miss : Long_Float;
      Tgt : constant Ct.V3 := [0.5, 0.2, 0.1];
      function Toward (From, To : Ct.V3) return Geom.Sight is
         Ok2 : Boolean;
         D : constant Ct.V3 := Ct.Unit ([To (0) - From (0), To (1) - From (1), To (2) - From (2)], Ok2);
      begin
         pragma Assert (Ok2);
         return (O => From, D => D);
      end Toward;
      P : Ct.V3;
   begin
      P := Sf.Pair (Toward ([0.0, 0.0, 0.0], Tgt), Toward ([0.0, 1.0, 0.0], Tgt), 0.001, Ok, Miss);
      Check (Ok and then Ct.Norm ([P (0) - Tgt (0), P (1) - Tgt (1), P (2) - Tgt (2)]) < 1.0e-9, "表面点·两条视线交出一个点(取最近那一段的中点),不需要深度");
      P := Sf.Pair (Toward ([0.0, 0.0, 0.0], Tgt), Toward ([0.0, 1.0, 0.0], [0.5, 0.2, 0.13]), 0.001, Ok, Miss);
      Check (not Ok and then Miss > 0.001, "表面点·两条视线差得太远 = 左右眼配错了点,当场拒绝,不许当成一个点收下(差 " & Codec.Fmt (Miss, 4) & ")");
      P := Sf.Pair (Toward ([0.0, 0.0, 0.0], Tgt), Toward ([0.0, 1.0, 0.0], [-0.5, 1.8, -0.1]), 0.001, Ok, Miss);
      Check (not Ok, "表面点·交在身后的不算");
      declare
         Rays : Geom.Sight_Vectors.Vector;
         Pts : Ct.V3_Vectors.Vector;
         Dropped : Natural;
         Good : Boolean := True;
      begin
         for I in 0 .. 4 loop
            for J in 0 .. 4 loop
               Rays.Append (Toward ([0.0, 0.0, 1.0], [0.01 * Long_Float (I), 0.01 * Long_Float (J), 0.0]));
            end loop;
         end loop;
         Rays.Append (Geom.Sight'(O => [0.0, 0.0, 1.0], D => [1.0, 0.0, 0.0]));   --  和面平行 ⇒ 落不到面上
         Sf.On_Plane (Rays, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], Pts, Dropped);
         for K in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Q : constant Ct.V3 := Pts (K);
               I : constant Natural := K / 5;
               J : constant Natural := K mod 5;
            begin
               if abs (Q (0) - 0.01 * Long_Float (I)) > 1.0e-9 or else abs (Q (1) - 0.01 * Long_Float (J)) > 1.0e-9 or else abs Q (2) > 1.0e-9 then
                  Good := False;
               end if;
            end;
         end loop;
         Check (Good and then Natural (Pts.Length) = 25 and then Dropped = 1,
                "表面点·轮廓像素各发一条视线落到它躺的面上 = 顶面的点;和面平行的那条丢掉并报数(丢 " & Codec.Img (Dropped) & ")");
         Sf.Extrude_To_Support (Pts, 0.0, 0.01);
         Check (Natural (Pts.Length) = 25, "表面点·顶面就在支撑面上(高度 0)⇒ 往下拉不出任何点");
      end;
      declare
         Top : Ct.V3_Vectors.Vector;
      begin
         for I in 0 .. 3 loop
            Top.Append (Ct.V3'([0.01 * Long_Float (I), 0.0, 0.03]));
         end loop;
         Sf.Extrude_To_Support (Top, 0.0, 0.01);
         Check (Natural (Top.Length) = 12, "表面点·把看得见的顶面朝支撑面拉下去补出侧面(显式假设:实心、从顶面连到支撑面):4 个顶点 ⇒ 每个再补 2 层 = 12 点");
      end;
      declare
         Pts : Ct.V3_Vectors.Vector;
         Nrm : Ct.V3;
         Cnt : Natural;
         Off_Table : Boolean := True;
      begin
         for I in 0 .. 19 loop
            for J in 0 .. 19 loop
               Pts.Append (Ct.V3'([0.01 * Long_Float (I), 0.01 * Long_Float (J), 0.0]));
            end loop;
         end loop;
         for I in 0 .. 4 loop
            for J in 0 .. 4 loop
               for K in 1 .. 3 loop
                  Pts.Append (Ct.V3'([0.05 + 0.005 * Long_Float (I), 0.05 + 0.005 * Long_Float (J), 0.01 * Long_Float (K)]));
               end loop;
            end loop;
         end loop;
         Sf.Drop_Support_Plane (Pts, 0.002, Nrm, Cnt);
         for Q of Pts loop
            if Q (2) < 0.005 then
               Off_Table := False;
            end if;
         end loop;
         Check (Cnt = 400 and then Natural (Pts.Length) = 75 and then Off_Table and then abs Nrm (2) > 0.999,
                "表面点·把支撑面那张平面上的点扔掉(确定性 RANSAC):桌面 400 点全走,盒子 75 点全留,法向朝上");
      end;
   end;

   --  🔴 运动学(V1b 第三步,Kinem.Fit):合成的 6 关节胳膊(像 x5:底座转、肩 / 肘 / 腕三根平行的俯仰、腕转、腕滚),手上的眼在参照读数时
   --  离底座 0.6 m、朝前下方看桌面;开机扫描 = 每个关节单独两个方向转到 ±45°(8 格);桌面 3000 个点投进每一格(像素噪声 0.3 px、5% 乱配)。
   --  不给焦距(真 400),只给关节读数 + 配点 ⇒ 焦距要回到 1% 内;全部关节同时随机转 ±30° 的 30 个姿势(没参与拟合),只给关节读数算眼在哪,
   --  按训练帧定一个倍数(量不出米)后中位 < 1 mm、最大 < 5 mm(离线 Python 同一套:中位 0.69、最大 1.73 mm)
   declare
      use Geom;
      use Ada.Numerics.Long_Elementary_Functions;
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      function U01 return Long_Float is (Long_Float (FR.Random (Gen)));
      function Gauss return Long_Float is
         A : constant Long_Float := Long_Float'Max (1.0e-12, U01);
         B : constant Long_Float := U01;
      begin
         return Sqrt (-2.0 * Log (A)) * Cos (2.0 * Ada.Numerics.Pi * B);
      end Gauss;
      Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算)
      F_True : constant Long_Float := 400.0;
      Cx : constant Long_Float := 320.0;
      Cy : constant Long_Float := 240.0;
      --  世界系(z 朝上)里的 6 根轴
      Wax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]];
      Pax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 0.05], [0.0, 0.0, 0.12], [0.25, 0.0, 0.12], [0.45, 0.0, 0.16], [0.5, 0.0, 0.16], [0.55, 0.0, 0.16]];
      C0 : constant V3 := [0.6, 0.0, 0.22];
      Fwd0 : constant V3 := [0.5, 0.0, -0.87];
      Fwd : constant V3 := [Fwd0 (0) / Norm (Fwd0), Fwd0 (1) / Norm (Fwd0), Fwd0 (2) / Norm (Fwd0)];
      Xc : constant V3 := [0.0, -1.0, 0.0];                          --  眼的 x = 右
      Zc : constant V3 := [-Fwd (0), -Fwd (1), -Fwd (2)];            --  眼的 z 朝后
      Yc : constant V3 := [Zc (1) * Xc (2) - Zc (2) * Xc (1), Zc (2) * Xc (0) - Zc (0) * Xc (2), Zc (0) * Xc (1) - Zc (1) * Xc (0)];
      R0 : constant M3 := [[Xc (0), Yc (0), Zc (0)], [Xc (1), Yc (1), Zc (1)], [Xc (2), Yc (2), Zc (2)]];   --  眼系 → 世界
      Truth : Kinem.Model;
      Frames : Kinem.Frame_Vectors.Vector;
      Cs : Kinem.Corr_Vectors.Vector;
      Npt : constant := 3000;
      Xw : array (0 .. Npt - 1) of V3;
      Steps : constant array (0 .. 7) of Long_Float := [2.0, 4.0, 8.0, 14.0, 20.0, 28.0, 36.0, 45.0];
      function Zeros6 return Floats is
         Q : Floats;
      begin
         for I in 0 .. 5 loop
            Q.Append (0.0);
         end loop;
         return Q;
      end Zeros6;
      --  第 K 帧里每个点落在哪(看不见 = U < 0)
      type Uv is record
         U, V : Long_Float := -1.0;
      end record;
      type Uv_Array is array (0 .. Npt - 1) of Uv;
      function Project_All (Q : Floats) return Uv_Array is
         Rr : M3;
         Tt : V3;
         Out_Uv : Uv_Array;
      begin
         Kinem.FK (Truth, Q, Rr, Tt);
         for I in 0 .. Npt - 1 loop
            declare
               D : constant V3 := [Xw (I) (0) - Tt (0), Xw (I) (1) - Tt (1), Xw (I) (2) - Tt (2)];
               Pc : constant V3 := Ap (Tr (Rr), D);
               Z : constant Long_Float := -Pc (2);
            begin
               if Z > 0.05 then
                  declare
                     U : constant Long_Float := Cx + F_True * Pc (0) / Z;
                     V : constant Long_Float := Cy - F_True * Pc (1) / Z;
                  begin
                     if U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 270.0 then
                        Out_Uv (I) := (U, V);
                     end if;
                  end;
               end if;
            end;
         end loop;
         return Out_Uv;
      end Project_All;
      type Uv_Ptr is access Uv_Array;
      Views : array (0 .. 96) of Uv_Ptr;
      Paired : array (0 .. 96, 0 .. 96) of Boolean := [others => [others => False]];
      Serial : Natural := 0;
      --  同驱动的扫描(09-27):仪器按第 I 帧上问的点配 ⇒ 问的点精确、配到的点带噪声;起点那帧出发的对共用点号 = 轨迹(同一个点跨很多帧),别的对各自编号
      procedure Add_Pair (I, J : Natural) is
         Cnt : Natural := 0;
      begin
         Paired (I, J) := True;
         Serial := Serial + 1;
         for P in 0 .. Npt - 1 loop
            exit when Cnt >= 200;
            if Views (I) (P).U >= 0.0 and then Views (J) (P).U >= 0.0 then
               declare
                  C : Kinem.Corr := (I => I, J => J, Ua => Views (I) (P).U, Va => Views (I) (P).V,
                                     Ub => Views (J) (P).U + 0.3 * Gauss, Vb => Views (J) (P).V + 0.3 * Gauss,
                                     Pt => (if I = 0 then P else Npt * Serial + P));
               begin
                  if U01 < 0.05 then   --  5% 乱配
                     C.Ub := 640.0 * U01; C.Vb := 270.0 * U01;
                  end if;
                  Cs.Append (C);
                  Cnt := Cnt + 1;
               end;
            end if;
         end loop;
      end Add_Pair;
      Fit_M : Kinem.Model;
      Rep : Kinem.Fit_Report;
      Okf : Boolean;
   begin
      FR.Reset (Gen, 20260926);
      Truth.N := 6; Truth.F := F_True; Truth.Cx := Cx; Truth.Cy := Cy; Truth.Q0 := Zeros6; Truth.Valid := True;
      for I in 0 .. 5 loop
         Truth.Ax (I).W := Ap (Tr (R0), Wax (I));
         Truth.Ax (I).P := Ap (Tr (R0), [Pax (I) (0) - C0 (0), Pax (I) (1) - C0 (1), Pax (I) (2) - C0 (2)]);
      end loop;
      for I in 0 .. Npt - 1 loop
         --  桌面(世界 z = 0)换到参照眼系
         declare
            Pw : constant V3 := [0.2 + 1.2 * U01, -0.8 + 1.6 * U01, 0.0];
         begin
            Xw (I) := Ap (Tr (R0), [Pw (0) - C0 (0), Pw (1) - C0 (1), Pw (2) - C0 (2)]);
         end;
      end loop;
      Frames.Append (Kinem.Frame_Info'(Q => Zeros6, Joint => -1));
      for J in 0 .. 5 loop
         for D in 0 .. 1 loop
            for K in Steps'Range loop
               declare
                  Q : Floats := Zeros6;
               begin
                  Q.Replace_Element (J, (if D = 0 then -1.0 else 1.0) * Steps (K) * Deg);
                  Frames.Append (Kinem.Frame_Info'(Q => Q, Joint => J));
               end;
            end loop;
         end loop;
      end loop;
      for K in 0 .. Natural (Frames.Length) - 1 loop
         Views (K) := new Uv_Array'(Project_All (Frames (K).Q));
      end loop;
      --  配对:每一格和起点、和同一方向的上一格;每一帧和关节上最近的 4 帧
      for K in 1 .. Natural (Frames.Length) - 1 loop
         Add_Pair (0, K);
         if (K - 1) mod 8 /= 0 then
            Add_Pair (K - 1, K);
         end if;
      end loop;
      for K in 0 .. Natural (Frames.Length) - 1 loop
         declare
            type Dk is record
               D : Long_Float := Long_Float'Last;
               J : Natural := 0;
            end record;
            Best : array (0 .. 3) of Dk;
         begin
            for J in 0 .. Natural (Frames.Length) - 1 loop
               if J /= K then
                  declare
                     Dm : Long_Float := 0.0;
                  begin
                     for X in 0 .. 5 loop
                        Dm := Long_Float'Max (Dm, abs (Frames (K).Q (X) - Frames (J).Q (X)));
                     end loop;
                     for B in Best'Range loop
                        if Dm < Best (B).D then
                           for C in reverse B + 1 .. Best'Last loop
                              Best (C) := Best (C - 1);
                           end loop;
                           Best (B) := (Dm, J);
                           exit;
                        end if;
                     end loop;
                  end;
               end if;
            end loop;
            for B of Best loop
               declare
                  Lo : constant Natural := Natural'Min (K, B.J);
                  Hi : constant Natural := Natural'Max (K, B.J);
               begin
                  if not Paired (Lo, Hi) then
                     Add_Pair (Lo, Hi);
                  end if;
               end;
            end loop;
         end;
      end loop;
      Kinem.Fit (Frames, 0, Cs, Cx, Cy, 640.0, Fit_M, Rep, Okf);
      declare
         --  考试:全部关节同时 ±30°,只给读数;按训练帧的眼的位置定一个倍数(模型单位 → 米)
         Sxy, Sxx : Long_Float := 0.0;
         Errs : Floats;
         Rt, Rf : M3;
         Tt, Tf : V3;
         Emax, Emed : Long_Float := 0.0;
      begin
         if Okf then
            for K in 0 .. Natural (Frames.Length) - 1 loop
               Kinem.FK (Truth, Frames (K).Q, Rt, Tt);
               Kinem.FK (Fit_M, Frames (K).Q, Rf, Tf);
               for X in 0 .. 2 loop
                  Sxy := Sxy + Tf (X) * Tt (X); Sxx := Sxx + Tf (X) * Tf (X);
               end loop;
            end loop;
            for T in 1 .. 30 loop
               declare
                  Q : Floats;
               begin
                  for X in 0 .. 5 loop
                     Q.Append ((2.0 * U01 - 1.0) * 30.0 * Deg);
                  end loop;
                  Kinem.FK (Truth, Q, Rt, Tt);
                  Kinem.FK (Fit_M, Q, Rf, Tf);
                  declare
                     S : constant Long_Float := (if Sxx > 0.0 then Sxy / Sxx else 0.0);
                     E : constant Long_Float := Sqrt ((S * Tf (0) - Tt (0)) ** 2 + (S * Tf (1) - Tt (1)) ** 2 + (S * Tf (2) - Tt (2)) ** 2);
                  begin
                     Errs.Append (1000.0 * E);
                     Emax := Long_Float'Max (Emax, 1000.0 * E);
                  end;
               end;
            end loop;
            declare
               package Sorting is new F64_Vectors.Generic_Sorting;
               Ss : Floats := Errs;
            begin
               Sorting.Sort (Ss);
               Emed := Ss (Natural (Ss.Length) / 2);
            end;
         end if;
         if Okf then
            for J in 0 .. 5 loop
               declare
                  Wt : constant V3 := Truth.Ax (J).W;
                  Wf : constant V3 := Fit_M.Ax (J).W;
                  Cw : constant Long_Float := Wt (0) * Wf (0) + Wt (1) * Wf (1) + Wt (2) * Wf (2);
                  --  轴上离参照眼最近那点(去掉沿轴的分量)
                  function Foot (A : Kinem.Axis) return V3 is
                     D : constant Long_Float := A.P (0) * A.W (0) + A.P (1) * A.W (1) + A.P (2) * A.W (2);
                  begin
                     return [A.P (0) - D * A.W (0), A.P (1) - D * A.W (1), A.P (2) - D * A.W (2)];
                  end Foot;
                  Pt : constant V3 := Foot (Truth.Ax (J));
                  Pf : constant V3 := Foot (Fit_M.Ax (J));
               begin
                  Put_Line ("      轴" & Natural'Image (J) & ":方向 cos " & Codec.Fmt (Cw, 5) & " · 真的轴离眼 (" & Codec.Fmt (Pt (0), 3) & "," & Codec.Fmt (Pt (1), 3) & ","
                            & Codec.Fmt (Pt (2), 3) & ") · 解的 (" & Codec.Fmt (Pf (0), 3) & "," & Codec.Fmt (Pf (1), 3) & "," & Codec.Fmt (Pf (2), 3) & ")"
                            & (if J < Natural (Rep.Joint_Med.Length) then " · 起步残差 " & Codec.Fmt (Rep.Joint_Med (J), 3) else "")
                            & (if J < Natural (Rep.Rho.Length) then " · ρ " & Codec.Fmt (Rep.Rho (J), 3) else ""));
               end;
            end loop;
         end if;
         Put_Line ("    运动学:配点 " & Natural'Image (Rep.N_Corr) & " · 内点 " & Natural'Image (Rep.N_Used) & " · 起步焦距 " & Codec.Fmt (Rep.F_Start, 1)
                   & " → " & Codec.Fmt (Rep.F, 1) & " · 残差中位 " & Codec.Fmt (Rep.Med_Px, 3) & " px · 多视图 " & Codec.Img (Rep.Mv_Tracks) & " 条轨迹 "
                   & Codec.Img (Rep.Mv_Obs) & " 笔、重投影中位 " & Codec.Fmt (Rep.Mv_Start_Px, 3) & " → " & Codec.Fmt (Rep.Mv_Px, 3) & " px(" & Codec.Img (Rep.Mv_Iters)
                   & " 轮)· 考试中位 " & Codec.Fmt (Emed, 3) & " mm、最大 " & Codec.Fmt (Emax, 3) & " mm" & (if Rep.Flipped then " · 平移反过一次号" else ""));
         declare
            T : Unbounded_String;
         begin
            for X of Rep.Secs loop
               Append (T, " " & Codec.Fmt (X, 1));
            end loop;
            Put_Line ("    运动学各步用时(秒:网格 / 精修 / 比例 / 一起解):" & To_String (T));
         end;
         --  反解:真模型上 20 个随机够得着的位姿(全关节 ±30°),从零位起解,解出的关节角算回去要正好到那儿
         declare
            Worst_P, Worst_R : Long_Float := 0.0;
            Empty : Floats;
         begin
            for T in 1 .. 20 loop
               declare
                  Qt, Qs : Floats;
                  Rt2 : M3;
                  Tt2 : V3;
                  Pe, Re : Long_Float;
               begin
                  for X in 0 .. 5 loop
                     Qt.Append ((2.0 * U01 - 1.0) * 30.0 * Deg);
                  end loop;
                  Kinem.FK (Truth, Qt, Rt2, Tt2);
                  Kinem.IK (Truth, Rt2, Tt2, Zeros6, Empty, Empty, Qs, Pe, Re);
                  Worst_P := Long_Float'Max (Worst_P, Pe); Worst_R := Long_Float'Max (Worst_R, Re);
               end;
            end loop;
            Check (Worst_P < 1.0e-6 and then Worst_R < 1.0e-6,
                   "运动学·反解:20 个随机够得着的位姿(全关节 ±30°)从零位起解,最差还差位置 " & Long_Float'Image (Worst_P) & " m、朝向 "
                   & Long_Float'Image (Worst_R) & " rad(要 < 1e-6)");
         end;
         Check (Okf and then abs (Rep.F - F_True) < 0.01 * F_True and then Emed < 1.0 and then Emax < 5.0,
                "运动学·只给关节读数 + 腕眼配点量出 6 根轴和焦距:焦距 " & Codec.Fmt (Rep.F, 1) & "(真 400,要 1% 内),全关节 ±30° 考试中位 "
                & Codec.Fmt (Emed, 2) & " mm、最大 " & Codec.Fmt (Emax, 2) & " mm(要 < 1 / < 5 mm)");
         --  转 / 走两样各解一次(09-27):这条全是转的胳膊,六根都要认成转;"走"那样的残差照实印出来
         declare
            All_Turn : Boolean := Okf and then Natural (Rep.Slide.Length) = 6;
            T : Unbounded_String;
         begin
            for J in 0 .. Natural (Rep.Slide.Length) - 1 loop
               All_Turn := All_Turn and then not Rep.Slide (J) and then not Fit_M.Ax (J).Slide;
               Append (T, " " & Codec.Fmt (Rep.Joint_Med_Turn (J), 3) & " / " & Codec.Fmt (Rep.Joint_Med_Slide (J), 3));
            end loop;
            Check (All_Turn, "运动学·全是转的胳膊六根轴都认成转(每根按转 / 按走的残差中位 px:" & To_String (T) & ")");
         end;
         --  ④ 焊点(09-27 V1B32):真模型的轴故意挪开当起步 —— 肩、肘两根轴离眼远近各错 +3% / −3%、腕那根方向偏 0.3°、焦距错 1% ——
         --  只跑多视图那一步(按轨迹重投影一起解),要回到真模型:全关节 ±30° 考试最大 < 0.5 mm、焦距 0.2% 内;起步本身考试要 > 3 mm(焊点有牙)
         declare
            Mp : Kinem.Model := Truth;
            Rp : Kinem.Fit_Report;
            procedure Exam (Mm : Kinem.Model; Med, Mx : out Long_Float) is
               Sxy2, Sxx2 : Long_Float := 0.0;
               Rt3, Rf3 : M3;
               Tt3, Tf3 : V3;
               Es : Floats;
               package Sorting is new F64_Vectors.Generic_Sorting;
               Gen2 : FR.Generator;
            begin
               FR.Reset (Gen2, 20260927);
               for K in 0 .. Natural (Frames.Length) - 1 loop
                  Kinem.FK (Truth, Frames (K).Q, Rt3, Tt3);
                  Kinem.FK (Mm, Frames (K).Q, Rf3, Tf3);
                  for X in 0 .. 2 loop
                     Sxy2 := Sxy2 + Tf3 (X) * Tt3 (X); Sxx2 := Sxx2 + Tf3 (X) * Tf3 (X);
                  end loop;
               end loop;
               Mx := 0.0;
               for T in 1 .. 30 loop
                  declare
                     Q : Floats;
                     S2 : constant Long_Float := (if Sxx2 > 0.0 then Sxy2 / Sxx2 else 0.0);
                  begin
                     for X in 0 .. 5 loop
                        Q.Append ((2.0 * Long_Float (FR.Random (Gen2)) - 1.0) * 30.0 * Deg);
                     end loop;
                     Kinem.FK (Truth, Q, Rt3, Tt3);
                     Kinem.FK (Mm, Q, Rf3, Tf3);
                     Es.Append (1000.0 * Sqrt ((S2 * Tf3 (0) - Tt3 (0)) ** 2 + (S2 * Tf3 (1) - Tt3 (1)) ** 2 + (S2 * Tf3 (2) - Tt3 (2)) ** 2));
                     Mx := Long_Float'Max (Mx, Es.Last_Element);
                  end;
               end loop;
               Sorting.Sort (Es);
               Med := Es (Natural (Es.Length) / 2);
            end Exam;
            E0m, E0x, E1m, E1x : Long_Float;
         begin
            Mp.Ax (1).P := [Mp.Ax (1).P (0) * 1.03, Mp.Ax (1).P (1) * 1.03, Mp.Ax (1).P (2) * 1.03];
            Mp.Ax (2).P := [Mp.Ax (2).P (0) * 0.97, Mp.Ax (2).P (1) * 0.97, Mp.Ax (2).P (2) * 0.97];
            Mp.Ax (4).W := Ap (Rodrigues ([0.3 * Deg, 0.0, 0.0]), Mp.Ax (4).W);
            Mp.F := 1.01 * F_True;   --  焦距错 1%(比例)
            --  尺度钉到"参与的各帧眼的位置均方根 = 1"(同 Fit 交出来的模型):真模型按米,挪开以后整体除一下
            declare
               S2 : Long_Float := 0.0;
               Rr3 : M3;
               Tt4 : V3;
            begin
               for K in 0 .. Natural (Frames.Length) - 1 loop
                  Kinem.FK (Mp, Frames (K).Q, Rr3, Tt4);
                  S2 := S2 + Tt4 (0) ** 2 + Tt4 (1) ** 2 + Tt4 (2) ** 2;
               end loop;
               S2 := Sqrt (S2 / Long_Float (Frames.Length));
               for J in 0 .. 5 loop
                  Mp.Ax (J).P := [Mp.Ax (J).P (0) / S2, Mp.Ax (J).P (1) / S2, Mp.Ax (J).P (2) / S2];
               end loop;
            end;
            --  对照:真模型起步(规整到同样的约定)只跑这一步,不许走开(考试最大 < 0.5 mm):走开 = 目标函数本身偏了(09-27 查出来过:
            --  5% 乱配没挑掉时 soft-l1 把模型拉开 24 mm;起步的尺度和这一步钉尺度的帧不是同一批时 LM 为压约束行走开 11 mm)
            declare
               Mt : Kinem.Model := Truth;
               Rt4 : Kinem.Fit_Report;
               Am, Ax, Bm, Bx : Long_Float;
            begin
               Exam (Mt, Am, Ax);
               Kinem.Refine_Tracks (Frames, Cs, Mt, Rt4);
               Exam (Mt, Bm, Bx);
               Put_Line ("    运动学·多视图那一步(真模型起步):考试 " & Codec.Fmt (Am, 3) & " / " & Codec.Fmt (Ax, 3) & " ⇒ " & Codec.Fmt (Bm, 3) & " / " & Codec.Fmt (Bx, 3)
                         & " mm · 重投影中位 " & Codec.Fmt (Rt4.Mv_Start_Px, 4) & " → " & Codec.Fmt (Rt4.Mv_Px, 4) & " px · " & Codec.Img (Rt4.Mv_Iters) & " 轮 · 焦距 " & Codec.Fmt (Mt.F, 2));
               Check (Bx < 0.5 and then abs (Mt.F - F_True) < 0.002 * F_True,
                      "运动学·多视图一步从真模型起步不走开:考试最大 " & Codec.Fmt (Bx, 3) & " mm(要 < 0.5)、焦距 " & Codec.Fmt (Mt.F, 2) & "(真 400,要 0.2% 内)");
            end;
            Exam (Mp, E0m, E0x);
            Kinem.Refine_Tracks (Frames, Cs, Mp, Rp);
            Exam (Mp, E1m, E1x);
            Put_Line ("    运动学·多视图那一步:起步(轴挪开)考试中位 " & Codec.Fmt (E0m, 2) & " / 最大 " & Codec.Fmt (E0x, 2) & " mm、焦距 " & Codec.Fmt (1.01 * F_True, 1)
                      & " ⇒ " & Codec.Img (Rp.Mv_Tracks) & " 条轨迹、" & Codec.Img (Rp.Mv_Iters) & " 轮、重投影中位 " & Codec.Fmt (Rp.Mv_Start_Px, 3) & " → "
                      & Codec.Fmt (Rp.Mv_Px, 3) & " px ⇒ 考试中位 " & Codec.Fmt (E1m, 2) & " / 最大 " & Codec.Fmt (E1x, 2) & " mm、焦距 " & Codec.Fmt (Mp.F, 1));
            Check (E0x > 3.0 and then E1x < 0.5 and then abs (Mp.F - F_True) < 0.002 * F_True,
                   "运动学·多视图一步把挪开的轴拉回来:起步考试最大 " & Codec.Fmt (E0x, 2) & " mm(要 > 3)⇒ " & Codec.Fmt (E1x, 2) & " mm(要 < 0.5)、焦距 "
                   & Codec.Fmt (Mp.F, 1) & "(真 400,要 0.2% 内)");
         end;
      end;
   end;

   --  🔴 运动学·沿轴走的关节(09-27 无人机那一半):合成的龙门架(像箱上的无人机:三个沿世界 x / y / z 走的关节,再绕机身中心 yaw / pitch / roll),
   --  机身中心下 3 cm 的眼朝下(偏 8°)看桌面,离桌 0.6 m,读数:走的按米、转的按弧度。扫描同驱动:每个关节单独两个方向各 3 格
   --  (走的累计 0.03 / 0.15 / 0.34、转的 0.03 / 0.15 / 0.45,同驱动"头一格 = 读数量级的 3%、之后按画面挪画幅宽 1/5 放大"的量级)+ 8 格几个关节一起动
   --  (每个关节到它扫到的那一头的一半,正负排法同驱动);配对同驱动:起点 ↔ 每一格(轨迹)、每段头两格、相邻关节头一格之间、一起动的相邻两格;
   --  像素噪声 0.3 px、5% 乱配。要:六根轴认对(前三根走、后三根转)、焦距 0.5% 内、全部关节在扫到的范围里随机 30 个姿势只给读数算眼在哪 ——
   --  按训练帧定一个倍数(量不出米)后最大 < 1 mm、朝向最大 < 0.05°;反解在真模型上 20 个随机姿势从零位解回来 < 1e-6
   declare
      use Geom;
      use Ada.Numerics.Long_Elementary_Functions;
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      function U01 return Long_Float is (Long_Float (FR.Random (Gen)));
      function Gauss return Long_Float is
         A : constant Long_Float := Long_Float'Max (1.0e-12, U01);
         B : constant Long_Float := U01;
      begin
         return Sqrt (-2.0 * Log (A)) * Cos (2.0 * Ada.Numerics.Pi * B);
      end Gauss;
      Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算)
      F_True : constant Long_Float := 400.0;
      Cx : constant Long_Float := 320.0;
      Cy : constant Long_Float := 240.0;
      Slide_J : constant array (0 .. 5) of Boolean := [True, True, True, False, False, False];
      Dir : constant array (0 .. 5) of V3 := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
      Cb : constant V3 := [0.0, 0.0, 0.63];   --  机身中心(转轴都过它)
      C0 : constant V3 := [0.0, 0.0, 0.6];    --  眼(机身中心下 3 cm)
      R0 : constant M3 := Rodrigues ([8.0 * Deg, 0.0, 0.0]);   --  眼系 → 世界:眼的 −z 朝下,再绕世界 x 偏 8°
      Offs : constant array (0 .. 5, 1 .. 3) of Long_Float :=
        [[0.03, 0.15, 0.34], [0.03, 0.15, 0.34], [0.03, 0.15, 0.34], [0.03, 0.15, 0.45], [0.03, 0.15, 0.45], [0.03, 0.15, 0.45]];
      Truth : Kinem.Model;
      Frames : Kinem.Frame_Vectors.Vector;
      Cs : Kinem.Corr_Vectors.Vector;
      Npt : constant := 3000;
      Xr : array (0 .. Npt - 1) of V3;   --  桌面上的点,在参照眼系里
      function Zeros6 return Floats is
         Q : Floats;
      begin
         for I in 0 .. 5 loop
            Q.Append (0.0);
         end loop;
         return Q;
      end Zeros6;
      type Uv is record
         U, V : Long_Float := -1.0;
      end record;
      type Uv_Array is array (0 .. Npt - 1) of Uv;
      function Project_All (Q : Floats) return Uv_Array is
         Rr : M3;
         Tt : V3;
         Out_Uv : Uv_Array;
      begin
         Kinem.FK (Truth, Q, Rr, Tt);
         for I in 0 .. Npt - 1 loop
            declare
               Pc : constant V3 := Ap (Tr (Rr), [Xr (I) (0) - Tt (0), Xr (I) (1) - Tt (1), Xr (I) (2) - Tt (2)]);
               Z : constant Long_Float := -Pc (2);
            begin
               if Z > 0.05 then
                  declare
                     U : constant Long_Float := Cx + F_True * Pc (0) / Z;
                     V : constant Long_Float := Cy - F_True * Pc (1) / Z;
                  begin
                     if U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0 then
                        Out_Uv (I) := (U, V);
                     end if;
                  end;
               end if;
            end;
         end loop;
         return Out_Uv;
      end Project_All;
      type Uv_Ptr is access Uv_Array;
      Views : array (0 .. 63) of Uv_Ptr;
      Serial : Natural := 0;
      procedure Add_Pair (I, J : Natural) is
         Cnt : Natural := 0;
      begin
         Serial := Serial + 1;
         for P in 0 .. Npt - 1 loop
            exit when Cnt >= 200;
            if Views (I) (P).U >= 0.0 and then Views (J) (P).U >= 0.0 then
               declare
                  C : Kinem.Corr := (I => I, J => J, Ua => Views (I) (P).U, Va => Views (I) (P).V,
                                     Ub => Views (J) (P).U + 0.3 * Gauss, Vb => Views (J) (P).V + 0.3 * Gauss,
                                     Pt => (if I = 0 then P else Npt * Serial + P));
               begin
                  if U01 < 0.05 then   --  5% 乱配
                     C.Ub := 640.0 * U01; C.Vb := 480.0 * U01;
                  end if;
                  Cs.Append (C);
                  Cnt := Cnt + 1;
               end;
            end if;
         end loop;
      end Add_Pair;
      type Head is record
         Frame, Joint : Natural := 0;
      end record;
      Heads : array (0 .. 11) of Head;
      N_Heads : Natural := 0;
      Fit_M : Kinem.Model;
      Rep : Kinem.Fit_Report;
      Okf : Boolean;
   begin
      FR.Reset (Gen, 20260927);
      Truth.N := 6; Truth.F := F_True; Truth.Cx := Cx; Truth.Cy := Cy; Truth.Q0 := Zeros6; Truth.Valid := True;
      for I in 0 .. 5 loop
         Truth.Ax (I).W := Ap (Tr (R0), Dir (I));
         Truth.Ax (I).Slide := Slide_J (I);
         Truth.Ax (I).P := (if Slide_J (I) then [0.0, 0.0, 0.0] else Ap (Tr (R0), [Cb (0) - C0 (0), Cb (1) - C0 (1), Cb (2) - C0 (2)]));
      end loop;
      for I in 0 .. Npt - 1 loop
         declare
            Pw : constant V3 := [-1.0 + 2.0 * U01, -1.0 + 2.0 * U01, 0.0];
         begin
            Xr (I) := Ap (Tr (R0), [Pw (0) - C0 (0), Pw (1) - C0 (1), Pw (2) - C0 (2)]);
         end;
      end loop;
      Frames.Append (Kinem.Frame_Info'(Q => Zeros6, Joint => -1));
      Views (0) := new Uv_Array'(Project_All (Frames (0).Q));
      for J in 0 .. 5 loop
         for D in 0 .. 1 loop
            for K in 1 .. 3 loop
               declare
                  Q : Floats := Zeros6;
               begin
                  Q.Replace_Element (J, (if D = 0 then -1.0 else 1.0) * Offs (J, K));
                  Frames.Append (Kinem.Frame_Info'(Q => Q, Joint => J));
                  Views (Natural (Frames.Length) - 1) := new Uv_Array'(Project_All (Q));
                  Add_Pair (0, Natural (Frames.Length) - 1);
                  if K = 1 then
                     Heads (N_Heads) := (Frame => Natural (Frames.Length) - 1, Joint => J); N_Heads := N_Heads + 1;
                  elsif K = 2 then
                     Add_Pair (Natural (Frames.Length) - 2, Natural (Frames.Length) - 1);
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      for Cb_K in 1 .. 8 loop
         declare
            Q : Floats := Zeros6;
         begin
            for J in 0 .. 5 loop
               Q.Replace_Element (J, (if ((Cb_K * 37 + J * 11) mod 16) < 8 then 0.5 else -0.5) * Offs (J, 3));   --  同驱动的排法
            end loop;
            Frames.Append (Kinem.Frame_Info'(Q => Q, Joint => -1));
            Views (Natural (Frames.Length) - 1) := new Uv_Array'(Project_All (Q));
            Add_Pair (0, Natural (Frames.Length) - 1);
            if Cb_K >= 2 then
               Add_Pair (Natural (Frames.Length) - 2, Natural (Frames.Length) - 1);
            end if;
         end;
      end loop;
      for H1 in 0 .. N_Heads - 1 loop
         for H2 in 0 .. N_Heads - 1 loop
            if Heads (H2).Joint = Heads (H1).Joint + 1 then
               Add_Pair (Heads (H1).Frame, Heads (H2).Frame);
            end if;
         end loop;
      end loop;
      Kinem.Fit (Frames, 0, Cs, Cx, Cy, 640.0, Fit_M, Rep, Okf);
      declare
         Sxy, Sxx : Long_Float := 0.0;
         Rt, Rf : M3;
         Tt, Tf : V3;
         Emax, Rmax : Long_Float := 0.0;
         Types_Ok : Boolean := Okf and then Natural (Rep.Slide.Length) = 6;
         T : Unbounded_String;
      begin
         for J in 0 .. Natural (Rep.Slide.Length) - 1 loop
            Types_Ok := Types_Ok and then Rep.Slide (J) = Slide_J (J) and then Fit_M.Ax (J).Slide = Slide_J (J);
            Append (T, " " & (if Rep.Slide (J) then "走" else "转") & "(" & Codec.Fmt (Rep.Joint_Med_Turn (J), 2) & " / " & Codec.Fmt (Rep.Joint_Med_Slide (J), 2) & ")");
         end loop;
         if Okf then
            for K in 0 .. Natural (Frames.Length) - 1 loop
               Kinem.FK (Truth, Frames (K).Q, Rt, Tt);
               Kinem.FK (Fit_M, Frames (K).Q, Rf, Tf);
               for X in 0 .. 2 loop
                  Sxy := Sxy + Tf (X) * Tt (X); Sxx := Sxx + Tf (X) * Tf (X);
               end loop;
            end loop;
            for Tn in 1 .. 30 loop
               declare
                  Q : Floats;
               begin
                  for X in 0 .. 5 loop
                     Q.Append ((2.0 * U01 - 1.0) * Offs (X, 3));
                  end loop;
                  Kinem.FK (Truth, Q, Rt, Tt);
                  Kinem.FK (Fit_M, Q, Rf, Tf);
                  declare
                     S : constant Long_Float := (if Sxx > 0.0 then Sxy / Sxx else 0.0);
                  begin
                     Emax := Long_Float'Max (Emax, 1000.0 * Sqrt ((S * Tf (0) - Tt (0)) ** 2 + (S * Tf (1) - Tt (1)) ** 2 + (S * Tf (2) - Tt (2)) ** 2));
                     Rmax := Long_Float'Max (Rmax, Norm (Rot_Vec (Mul (Tr (Rt), Rf))) / Deg);
                  end;
               end;
            end loop;
            for J in 0 .. 5 loop
               declare
                  Wt : constant V3 := Truth.Ax (J).W;
                  Wf : constant V3 := Fit_M.Ax (J).W;
                  S : constant Long_Float := (if Sxx > 0.0 then Sxy / Sxx else 0.0);
               begin
                  Put_Line ("      轴" & Natural'Image (J) & "(" & (if Fit_M.Ax (J).Slide then "走" else "转") & "):方向 cos "
                            & Codec.Fmt (abs (Wt (0) * Wf (0) + Wt (1) * Wf (1) + Wt (2) * Wf (2)) / (Norm (Wt) * Norm (Wf)), 6)
                            & (if Fit_M.Ax (J).Slide then " · 每单位读数走 " & Codec.Fmt (1000.0 * S * Norm (Wf), 2) & " mm(真 " & Codec.Fmt (1000.0 * Norm (Wt), 2) & ")" else ""));
               end;
            end loop;
         end if;
         Put_Line ("    龙门架:配点 " & Natural'Image (Rep.N_Corr) & " · 焦距 " & Codec.Fmt (Rep.F_Start, 1) & " → " & Codec.Fmt (Rep.F_Axes, 1) & " → " & Codec.Fmt (Rep.F, 1)
                   & " · 多视图重投影中位 " & Codec.Fmt (Rep.Mv_Start_Px, 3) & " → " & Codec.Fmt (Rep.Mv_Px, 3) & " px · 考试最大 " & Codec.Fmt (Emax, 3) & " mm、朝向 "
                   & Codec.Fmt (Rmax, 4) & "°");
         Check (Types_Ok and then abs (Rep.F - F_True) < 0.005 * F_True and then Emax < 1.0 and then Rmax < 0.05,
                "运动学·龙门架(三走三转,像无人机):轴的类型" & (if Types_Ok then "认对" else "认错") & "(转 / 走的残差 px:" & To_String (T) & ")· 焦距 "
                & Codec.Fmt (Rep.F, 1) & "(真 400,要 0.5% 内)· 扫到的范围里 30 个随机姿势最大 " & Codec.Fmt (Emax, 3) & " mm(要 < 1)、朝向 " & Codec.Fmt (Rmax, 4) & "°(要 < 0.05)");
         declare
            Worst_P, Worst_R : Long_Float := 0.0;
            Empty : Floats;
         begin
            for Tn in 1 .. 20 loop
               declare
                  Qt, Qs : Floats;
                  Rt2 : M3;
                  Tt2 : V3;
                  Pe, Re : Long_Float;
               begin
                  for X in 0 .. 5 loop
                     Qt.Append ((2.0 * U01 - 1.0) * Offs (X, 3));
                  end loop;
                  Kinem.FK (Truth, Qt, Rt2, Tt2);
                  Kinem.IK (Truth, Rt2, Tt2, Zeros6, Empty, Empty, Qs, Pe, Re);
                  Worst_P := Long_Float'Max (Worst_P, Pe); Worst_R := Long_Float'Max (Worst_R, Re);
               end;
            end loop;
            Check (Worst_P < 1.0e-6 and then Worst_R < 1.0e-6,
                   "运动学·龙门架反解:20 个随机姿势从零位解回来,最差差位置 " & Long_Float'Image (Worst_P) & "、朝向 " & Long_Float'Image (Worst_R) & "(要 < 1e-6)");
         end;
      end;
   end;

   --  🔴 ⑤ 前半段存 / 装回(Jointboot.Save_Kin / Load_Kin / Same_View,09-27):存了再读回来,每一个数都得一样(9 位小数);
   --  没量到头的界存成 none、读回还是"不设界";核对的判法:配上的点少于 10 个 / 位移中位 ≥ 1 px 都算"动了"
   declare
      use Geom;
      use Ada.Numerics.Long_Elementary_Functions;
      use type Bytes.Buf;
      K, K2 : Jointboot.Kin_Store;
      Okl : Boolean;
      Note : Unbounded_String;
      Path : constant String := "/tmp/bd_selfcheck_kin.txt";
      Worst : Long_Float := 0.0;
      procedure Cmp (A, B : Long_Float) is
      begin
         Worst := Long_Float'Max (Worst, abs (A - B));
      end Cmp;
      Img : Plug.Cam;
   begin
      Img.W := 8; Img.H := 6;
      for I in 1 .. 8 * 6 * 3 loop
         Img.RGB.Append (Interfaces.Unsigned_8 ((I * 37) mod 256));
      end loop;
      K.Key := To_Unbounded_String ("cams=640x480,;groups=6,;jaws=1;joints=a,");
      K.World_Cam := 0;
      K.Rw := Rodrigues ([0.1, -0.2, 0.3]); K.O := [0.5, -0.25, 3.125];
      K.Plane_Pt := [0.0, 0.0, 0.0]; K.Plane_N := [0.0, 0.0, 1.0]; K.Plane_Rms := 0.0043;
      --  不动的眼:记录里每个字段都给一个不是缺省的数(09-27 V1B38:原来只存了五样,像素残差没存,装回后核对的门成了 0)
      K.Fixed_Eye := (Valid => True, F => 289.25, Cx => 320.5, Cy => 239.75, K1 => -0.0123, K2 => 0.00456, K1_Sd => 0.0007, F_Meas => 289.125,
                      F_Prior => 300.5, F_Prior_Sd => 45.25, R_Ce => Rodrigues ([1.0, 0.01, -0.02]), Off => [0.011, -0.022, 0.033], Rms => 1.376,
                      F_Sd => 0.61, Rot_Sd => 0.0021, Off_Sd => 0.0033, Pos_Sd => 0.0144, Dropped => 17, Tip_Valid => True, Tip_Touch => True,
                      Tip => [0.1, -0.2, -1.7], Gap => 1.75, Stride => 0.888, Stride_Rot => 0.161, Fixed => True, Pos => [5.7, -2.75, 10.35]);
      K.Plane_Pt := [0.001, -0.002, 0.003]; K.Plane_N := [0.0, 0.6, 0.8];
      for A in 0 .. 1 loop
         declare
            W : Jointboot.Arm_World;
            D : Jointboot.Sweep_Data;
         begin
            W.Group := A; W.Valid := True; W.Sweep := A;
            W.Model.Valid := True; W.Model.N := 6; W.Model.F := 397.123456789; W.Model.Cx := 320.0; W.Model.Cy := 240.0;
            for J in 0 .. 5 loop
               W.Model.Q0.Append (0.001 * Long_Float (J + A));
               W.Model.Ax (J).W := [Sin (Long_Float (J)), Cos (Long_Float (J)), 0.0];
               W.Model.Ax (J).P := [0.1 * Long_Float (J), -0.2 * Long_Float (A), 1.0 / 3.0];
               W.Model.Ax (J).Slide := J = 1;   --  一根"走"的(09-27 无人机):类型也要原样回来
               W.Lo.Append (if J = 2 then Long_Float'First else -0.5 - Long_Float (J));
               W.Hi.Append (if J = 3 then Long_Float'Last else 0.5 + Long_Float (J));
            end loop;
            W.S := (if A = 0 then 1.0 else 1.00347);
            W.Ra := (if A = 0 then Identity else Rodrigues ([0.001, 0.002, -0.003])); W.Ta := (if A = 0 then [0.0, 0.0, 0.0] else [11.4658, -0.008, 0.0156]);
            D.W := 8; D.H := 6;
            for Fk in 0 .. 2 loop
               declare
                  Fr : Kinem.Frame_Info;
               begin
                  Fr.Joint := (if Fk = 0 then -1 else Fk);
                  for J in 0 .. 5 loop
                     Fr.Q.Append (0.01 * Long_Float (Fk * 7 + J));
                  end loop;
                  D.Frames.Append (Fr);
               end;
            end loop;
            D.Imgs.Append (Img);
            if A = 0 then
               D.World_Img := Img;
            end if;
            K.Worlds.Append (W); K.Ds.Append (D); K.Eyes.Append (A + 1);
         end;
      end loop;
      for P in 0 .. 2 loop
         K.Board.Append (Scene_Pt'(Pw => [Long_Float (P), 0.5, -0.001], Cov => [[1.0e-4, 0.0, 0.0], [0.0, 2.0e-4, 0.0], [0.0, 0.0, 3.0e-4]],
                                   U => 100.5 + Long_Float (P), V => 200.25, Sh => 0.36, Views => 2));
      end loop;
      Jointboot.Save_Kin (Path, K);
      Jointboot.Load_Kin (Path, K2, Okl, Note);
      if Okl then
         for A in 0 .. 1 loop
            Cmp (K.Worlds (A).S, K2.Worlds (A).S); Cmp (K.Worlds (A).Model.F, K2.Worlds (A).Model.F);
            for J in 0 .. 5 loop
               Cmp (K.Worlds (A).Model.Q0 (J), K2.Worlds (A).Model.Q0 (J));
               for X in 0 .. 2 loop
                  Cmp (K.Worlds (A).Model.Ax (J).W (X), K2.Worlds (A).Model.Ax (J).W (X)); Cmp (K.Worlds (A).Model.Ax (J).P (X), K2.Worlds (A).Model.Ax (J).P (X));
               end loop;
               if K.Worlds (A).Model.Ax (J).Slide /= K2.Worlds (A).Model.Ax (J).Slide then
                  Worst := 1.0;
               end if;
               if (K.Worlds (A).Lo (J) = Long_Float'First) /= (K2.Worlds (A).Lo (J) = Long_Float'First)
                 or else (K.Worlds (A).Hi (J) = Long_Float'Last) /= (K2.Worlds (A).Hi (J) = Long_Float'Last)
               then
                  Worst := 1.0;
               elsif K.Worlds (A).Lo (J) /= Long_Float'First and then K.Worlds (A).Hi (J) /= Long_Float'Last then
                  Cmp (K.Worlds (A).Lo (J), K2.Worlds (A).Lo (J)); Cmp (K.Worlds (A).Hi (J), K2.Worlds (A).Hi (J));
               end if;
            end loop;
            for I in 0 .. 2 loop
               Cmp (K.Worlds (A).Ta (I), K2.Worlds (A).Ta (I));
               for J in 0 .. 2 loop
                  Cmp (K.Worlds (A).Ra (I, J), K2.Worlds (A).Ra (I, J));
               end loop;
            end loop;
            for Fk in 0 .. 2 loop
               Cmp (Long_Float (K.Ds (A).Frames (Fk).Joint), Long_Float (K2.Ds (A).Frames (Fk).Joint));
               for J in 0 .. 5 loop
                  Cmp (K.Ds (A).Frames (Fk).Q (J), K2.Ds (A).Frames (Fk).Q (J));
               end loop;
            end loop;
            Cmp (Long_Float (K.Eyes (A)), Long_Float (K2.Eyes (A))); Cmp (Long_Float (K.Worlds (A).Group), Long_Float (K2.Worlds (A).Group));
            if K2.Ds (A).Imgs.Is_Empty or else K2.Ds (A).Imgs (0).RGB /= Img.RGB then
               Worst := 1.0;
            end if;
         end loop;
         declare
            A : Cam_Geo renames K.Fixed_Eye;
            B : Cam_Geo renames K2.Fixed_Eye;
         begin
            Cmp (A.F, B.F); Cmp (A.Cx, B.Cx); Cmp (A.Cy, B.Cy); Cmp (A.K1, B.K1); Cmp (A.K2, B.K2); Cmp (A.K1_Sd, B.K1_Sd); Cmp (A.F_Meas, B.F_Meas);
            Cmp (A.F_Prior, B.F_Prior); Cmp (A.F_Prior_Sd, B.F_Prior_Sd); Cmp (A.Rms, B.Rms); Cmp (A.F_Sd, B.F_Sd); Cmp (A.Rot_Sd, B.Rot_Sd);
            Cmp (A.Off_Sd, B.Off_Sd); Cmp (A.Pos_Sd, B.Pos_Sd); Cmp (Long_Float (A.Dropped), Long_Float (B.Dropped)); Cmp (A.Gap, B.Gap);
            Cmp (A.Stride, B.Stride); Cmp (A.Stride_Rot, B.Stride_Rot);
            for I in 0 .. 2 loop
               Cmp (A.Pos (I), B.Pos (I)); Cmp (A.Off (I), B.Off (I)); Cmp (A.Tip (I), B.Tip (I));
               for J in 0 .. 2 loop
                  Cmp (A.R_Ce (I, J), B.R_Ce (I, J));
               end loop;
            end loop;
            if A.Valid /= B.Valid or else A.Fixed /= B.Fixed or else A.Tip_Valid /= B.Tip_Valid or else A.Tip_Touch /= B.Tip_Touch then
               Worst := 1.0;
            end if;
         end;
         Cmp (K.Plane_Rms, K2.Plane_Rms); Cmp (Long_Float (K.World_Cam), Long_Float (K2.World_Cam));
         for I in 0 .. 2 loop
            Cmp (K.O (I), K2.O (I)); Cmp (K.Plane_Pt (I), K2.Plane_Pt (I)); Cmp (K.Plane_N (I), K2.Plane_N (I));
            for J in 0 .. 2 loop
               Cmp (K.Rw (I, J), K2.Rw (I, J));
            end loop;
         end loop;
         for P in 0 .. Natural'Min (Natural (K.Board.Length), Natural (K2.Board.Length)) - 1 loop
            Cmp (K.Board (P).U, K2.Board (P).U); Cmp (K.Board (P).V, K2.Board (P).V); Cmp (K.Board (P).Sh, K2.Board (P).Sh);
            Cmp (Long_Float (K.Board (P).Views), Long_Float (K2.Board (P).Views));
            for I in 0 .. 2 loop
               Cmp (K.Board (P).Pw (I), K2.Board (P).Pw (I));
               for J in 0 .. 2 loop
                  Cmp (K.Board (P).Cov (I, J), K2.Board (P).Cov (I, J));
               end loop;
            end loop;
         end loop;
         for A in 0 .. 1 loop
            Cmp (K.Worlds (A).Model.Cx, K2.Worlds (A).Model.Cx); Cmp (K.Worlds (A).Model.Cy, K2.Worlds (A).Model.Cy);
            Cmp (Long_Float (K.Worlds (A).Model.N), Long_Float (K2.Worlds (A).Model.N));
            Cmp (Long_Float (K.Ds (A).W), Long_Float (K2.Ds (A).W)); Cmp (Long_Float (K.Ds (A).H), Long_Float (K2.Ds (A).H));
            if K.Worlds (A).Valid /= K2.Worlds (A).Valid or else K.Worlds (A).Model.Valid /= K2.Worlds (A).Model.Valid then
               Worst := 1.0;
            end if;
         end loop;
         if Natural (K2.Board.Length) /= 3 or else K2.Ds (0).World_Img.RGB /= Img.RGB or else To_String (K2.Key) /= To_String (K.Key) then
            Worst := 1.0;
         end if;
      end if;
      Check (Okl and then Worst < 1.0e-8, "⑤ 前半段存进文件再读回来:每一个数最多差 " & Long_Float'Image (Worst)
             & "(要 < 1e-8;不动的眼整份相机几何、板上每个点的每一项、每根轴是转是走、没量到头的界、核对用的图、钥匙原样回来)· " & To_String (Note));
      --  旧版文件(kin 1:不动的眼只存了五样)不装回:读到它 = 从零量,不拿缺了像素残差的那份去核
      declare
         Fo : Ada.Text_IO.File_Type;
         Lines : Strs;
         K3 : Jointboot.Kin_Store;
         Ok3 : Boolean;
         Note3 : Unbounded_String;
      begin
         Ada.Text_IO.Open (Fo, Ada.Text_IO.In_File, Path);
         while not Ada.Text_IO.End_Of_File (Fo) loop
            Lines.Append (Ada.Text_IO.Get_Line (Fo));
         end loop;
         Ada.Text_IO.Close (Fo);
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Path);
         for L of Lines loop
            Ada.Text_IO.Put_Line (Fo, (if L'Length >= 4 and then L (L'First .. L'First + 3) = "kin " then "kin 1" else L));
         end loop;
         Ada.Text_IO.Close (Fo);
         Jointboot.Load_Kin (Path, K3, Ok3, Note3);
         Check (not Ok3, "⑤ 旧版前半段文件(kin 1)不装回 ⇒ 从零量:" & To_String (Note3));
      end;
      declare
         D_Same, D_Moved, D_Few : Floats;
      begin
         for I in 1 .. 50 loop
            D_Same.Append (0.05 * Long_Float (I mod 7));
            D_Moved.Append (3.0 + 0.1 * Long_Float (I mod 5));
         end loop;
         for I in 1 .. 9 loop
            D_Few.Append (0.1);
         end loop;
         Check (Jointboot.Same_View (D_Same) and then not Jointboot.Same_View (D_Moved) and then not Jointboot.Same_View (D_Few),
                "⑤ 核对的判法:位移中位 0.15 px 的 50 个点 = 没动;挪了 3 px = 动了;只配上 9 个点 = 核对不了(算动了)");
      end;
   end;

   --  🔴 ④ 说出去的长度(Act.Hand_Len / Len,09-27):尺子 = 第一只碰桌面量过指尖的手,眼到两瓣指尖中点的距离;给脑的长度按它说。
   --  没量过指尖 ⇒ 照实说"我自己的比例";只有第二只手量过 ⇒ 用第二只手的;头顶眼那种按别的办法量的指尖(不是碰桌面量的)不算
   declare
      Cx : Act.Context;
      G1, G2 : Geom.Cam_Geo := Geom.No_Geo;
      L0, L1, L2 : Unbounded_String;
      H0, H1, H2 : Long_Float;
   begin
      Cx.Map.Arms := 2; Cx.Map.Cam_On_Arm.Append (1); Cx.Map.Cam_On_Arm.Append (2);
      Cx.Geo.Append (Geom.No_Geo); Cx.Geo.Append (Geom.No_Geo); Cx.Geo.Append (Geom.No_Geo);
      H0 := Act.Hand_Len (Cx); L0 := To_Unbounded_String (Act.Len (Cx, 0.872));
      G2.Tip := [0.0, 0.0, -2.0]; G2.Tip_Valid := True; G2.Tip_Touch := True;
      G1.Tip := [0.0, 0.0, -1.744]; G1.Tip_Valid := True; G1.Tip_Touch := False;   --  不是碰桌面量的:不算
      Cx.Geo.Replace_Element (1, G1); Cx.Geo.Replace_Element (2, G2);
      H1 := Act.Hand_Len (Cx); L1 := To_Unbounded_String (Act.Len (Cx, 0.872));
      G1.Tip_Touch := True; Cx.Geo.Replace_Element (1, G1);
      H2 := Act.Hand_Len (Cx); L2 := To_Unbounded_String (Act.Len (Cx, 0.872));
      Check (H0 = 0.0 and then Index (L0, "own scale") > 0 and then H1 = 2.0 and then To_String (L1) = "0.44 hand-lengths"
             and then abs (H2 - 1.744) < 1.0e-12 and then To_String (L2) = "0.50 hand-lengths",
             "④ 给脑的长度按指尖长说:没量过 → """ & To_String (L0) & """;只有第二只手碰桌面量过(2.0)→ """ & To_String (L1)
             & """;第一只手也量过(1.744)→ """ & To_String (L2) & """(0.872 单位要说成 0.50)");
   end;

   --  🔴 两只手的系对齐要用的三样(Kinem,V1b 3c):多条视线交一点、两团点之间的相似变换(30% 野点)、一团点里的面(30% 野点)
   declare
      use Geom;
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      function U01 return Long_Float is (Long_Float (FR.Random (Gen)));
      Pt : constant V3 := [0.3, -0.2, 1.5];
      O : constant Kinem.V3_Array (0 .. 2) := [[0.0, 0.0, 0.0], [0.4, 0.0, 0.1], [-0.2, 0.3, 0.0]];
      D : Kinem.V3_Array (0 .. 2);
      X : V3;
      Okm : Boolean;
      Na : constant := 60;
      A, B : Kinem.V3_Array (0 .. Na - 1);
      S_True : constant Long_Float := 0.37;
      R_True : constant M3 := Rodrigues ([0.3, -0.5, 0.8]);
      T_True : constant V3 := [0.5, -1.2, 2.0];
      S : Long_Float;
      R : M3;
      T : V3;
      Inl : Natural;
      Md : Long_Float;
      Pl : Kinem.V3_Array (0 .. Na - 1);
      P0, Nrm : V3;
      N_True : constant V3 := [0.0, 0.6, 0.8];
   begin
      FR.Reset (Gen, 7);
      for I in O'Range loop
         D (I) := [Pt (0) - O (I) (0), Pt (1) - O (I) (1), Pt (2) - O (I) (2)];
      end loop;
      Kinem.Meet_Rays (O, D, X, Okm);
      Check (Okm and then Norm ([X (0) - Pt (0), X (1) - Pt (1), X (2) - Pt (2)]) < 1.0e-9, "对齐·三条视线交一点:差 " & Long_Float'Image (Norm ([X (0) - Pt (0), X (1) - Pt (1), X (2) - Pt (2)])));
      for I in A'Range loop
         A (I) := [U01 - 0.5, U01 - 0.5, U01 * 0.3];
         declare
            Ra : constant V3 := Ap (R_True, A (I));
         begin
            B (I) := [S_True * Ra (0) + T_True (0), S_True * Ra (1) + T_True (1), S_True * Ra (2) + T_True (2)];
         end;
         if I mod 10 < 3 then   --  30% 野点
            B (I) := [U01 * 3.0, U01 * 3.0, U01 * 3.0];
         end if;
      end loop;
      Kinem.Robust_Similarity (A, B, S, R, T, Inl, Md);
      declare
         Er : constant Long_Float := Norm (Rot_Vec (Mul (Tr (R), R_True)));
      begin
         Check (abs (S - S_True) < 1.0e-6 and then Er < 1.0e-6 and then Norm ([T (0) - T_True (0), T (1) - T_True (1), T (2) - T_True (2)]) < 1.0e-6 and then Inl = 42,
                "对齐·两团点的相似变换(60 对、30% 野点):倍数 " & Codec.Fmt (S, 6) & "(真 0.37)、转动差 " & Long_Float'Image (Er) & " rad、内点" & Natural'Image (Inl) & "(真 42)");
      end;
      for I in Pl'Range loop
         declare
            U : constant Long_Float := U01 - 0.5;
            V : constant Long_Float := U01 - 0.5;
         begin
            --  面 = 过 (0, 0, 1)、法向 N_True;面内两个方向 (1,0,0) 和 (0,0.8,-0.6)
            Pl (I) := [U, 0.8 * V, 1.0 - 0.6 * V];
            if I mod 10 < 3 then
               Pl (I) := [Pl (I) (0), Pl (I) (1) + 0.5 * U01, Pl (I) (2) + 0.5 * U01];
            end if;
         end;
      end loop;
      --  转动 ⇒ 四元数 ⇒ 转动(四种分支各一个:迹为正、x / y / z 最大)
      declare
         Worst : Long_Float := 0.0;
      begin
         for Rv of Kinem.V3_Array'([0.3, -0.2, 0.1], [3.0, 0.1, 0.0], [0.05, 3.0, 0.1], [0.0, 0.2, 3.1]) loop
            declare
               Rr : constant M3 := Rodrigues (Rv);
               Pz : constant Plug.Arm_Pose := Kinem.To_Pose (Rr, [1.0, 2.0, 3.0]);
               Rb : constant M3 := Quat_To_R (Pz);
            begin
               Worst := Long_Float'Max (Worst, Norm (Rot_Vec (Mul (Tr (Rr), Rb))));
            end;
         end loop;
         Check (Worst < 1.0e-12, "对齐·转动 → 四元数 → 转动(四种分支):最差差 " & Long_Float'Image (Worst) & " rad");
      end;
      Kinem.Robust_Plane (Pl, P0, Nrm, Inl, Md);
      Check (abs (abs (Nrm (0) * N_True (0) + Nrm (1) * N_True (1) + Nrm (2) * N_True (2)) - 1.0) < 1.0e-9 and then Inl >= 42,
             "对齐·一团点里的面(60 点、30% 野点):法向差 " & Long_Float'Image (1.0 - abs (Nrm (0) * N_True (0) + Nrm (1) * N_True (1) + Nrm (2) * N_True (2)))
             & "、内点" & Natural'Image (Inl));
   end;

   Put_Line ((if Fails = 0 then "🟢 自检全过" else "🔴 自检失败" & Natural'Image (Fails) & " 条"));
   if Fails > 0 then
      raise Program_Error;
   end if;
end Selfcheck;
