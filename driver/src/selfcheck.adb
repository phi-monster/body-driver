--  离线自检:不连仿真就能跑的那些量法和格式。每条断言写清楚"错了会是什么病"。
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
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
with Lang;
with Sinew;
with Runtime;
with Plan;
with Layout;
with Selfmap;
with Learned;
with Exam;
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
         Monitor.Step (W, 1.0, 0.5, 0.5, 0.01, F);
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
   --  ── 身体语言:解析 ──
   declare
      use Lang;
      use type Exam.Row_Id;
      function P1 (Src : String) return Program is (Lang.Parse (Src));
      G : constant Program := P1 ("hold 6 facing 7" & ASCII.LF &
                                  "reach 6 at 7 medium until touch" & ASCII.LF &
                                  "close 6 on 7 until resist" & ASCII.LF &
                                  "never 6 nearer 9" & ASCII.LF &
                                  "# 这一行是注释" & ASCII.LF &
                                  "say I can see it" & ASCII.LF &
                                  "onfail retry" & ASCII.LF &
                                  "done");
   begin
      Check (G.Ok, "语言:一段完整程序解析得通" & (if G.Ok then "" else " —— " & To_String (G.Err)));
      Check (Natural (G.Stmts.Length) = 7, "语言:注释不算一句,共 7 句(实" & Natural'Image (Natural (G.Stmts.Length)) & ")");
      Check (G.Stmts (0).V = V_Hold and then G.Stmts (0).R = R_Facing and then G.Stmts (0).Object.Number = 7,
             "语言:hold 6 facing 7");
      Check (G.Stmts (1).V = V_Reach and then G.Stmts (1).R = R_At and then G.Stmts (1).Amt = A_Medium
               and then G.Stmts (1).Ev = E_Touch, "语言:reach 带步子带停机事件");
      Check (G.Stmts (2).V = V_Close and then G.Stmts (2).R = R_At and then G.Stmts (2).Ev = E_Resist,
             "语言:close X on Y 的 on 等于 at");
      Check (G.Stmts (3).V = V_Never and then G.Stmts (3).R = R_Nearer, "语言:never = 不等式");
      Check (G.On_Fail = F_Retry, "语言:onfail retry 记在程序上");
      Check (G.Stmts (6).V = V_Done, "语言:done");
   end;
   declare
      use Lang;
      use type Exam.Row_Id;
      A : constant Program := Lang.Parse ("wiggle 6 at 7");
      B : constant Program := Lang.Parse ("reach 6 at");
      C : constant Program := Lang.Parse ("reach 6 medium until touch");
      D : constant Program := Lang.Parse ("reach 6 at 7 until steps");
      E : constant Program := Lang.Parse ("move_joint 3 0.1");
      F : constant Program := Lang.Parse ("reach hand at baseball small until touch");
   begin
      Check (not A.Ok and then A.Err_Line = 1, "语言:不是动词的第一个词 ⇒ 编译错,而且指出第几行");
      Check (not B.Ok, "语言:关系后面不跟东西 ⇒ 编译错");
      Check (not C.Ok, "语言:reach 不说关系 ⇒ 编译错");
      Check (not D.Ok, "语言:until steps 不给步数 ⇒ 编译错");
      Check (not E.Ok, "语言:关节号这种话【语法上就不存在】⇒ 说不出口");
      Check (F.Ok and then not F.Stmts (0).Subject.By_Number
               and then To_String (F.Stmts (0).Object.Word) = "baseball",
             "语言:名词可以是名字(能不能认出来是身体的事,不是语法的事)");
      Check (Lang.Unparse (F.Stmts (0)) = "reach hand at baseball small until touch",
             "语言:解析回写一模一样(" & Lang.Unparse (F.Stmts (0)) & ")");
   end;
   --  ── 编译:体检的否决权 ──
   declare
      use Lang;
      use type Exam.Row_Id;
      R : Exam.Report;
      Facts : Plan.Facts_Vectors.Vector;
      T : Exam.Thing_Check;
      Ft : Plan.Item_Facts;
      function Comp (Src : String) return Plan.Compiled is
        (Plan.Compile (Lang.Parse (Src), R, Facts));
      procedure Set_Stands (On : Boolean) is
         Ft : Plan.Item_Facts := Facts (1);
      begin
         Ft.Stands := On;
         Facts.Replace_Element (1, Ft);
      end Set_Stands;
   begin
      --  一具想象的身体:左右/上下/远近证过了能用,朝哪是死的
      for Row in Exam.Row_Id loop
         T.Rows (Row).V := (if Row = Exam.Facing or else Row = Exam.Bigness then Exam.Dead else Exam.Usable);
         T.Rows (Row).Why := To_Unbounded_String ("这一行一次都没动过");
      end loop;
      R.Things.Append (T);
      declare
         E1 : Exam.Eye_Check;
      begin
         R.Eyes.Append (E1);      --  这具想象的身体只有一只眼睛
      end;
      Ft.Exists := True; Ft.Mine := True; Ft.Grip := True; Ft.Thing_Idx := 0;
      Facts.Append (Ft);                                   --  0 号 = 我的手
      Ft := (Exists => True, Mine => False, Grip => False, Arm => 0, Thing_Idx => -1, Stands => False, Jaw_K => 0, Label => <>);
      Facts.Append (Ft);                                   --  1 号 = 外面的东西
      Check (Comp ("reach 0 at 1 small until touch").Ok, "编译:能用的行 ⇒ 收");
      declare
         C : constant Plan.Compiled := Comp ("hold 0 facing 1");
      begin
         Check (not C.Ok and then C.Err_Line = 1, "编译:朝哪是死的 ⇒ 退回,并指出第几行");
         Check (Length (C.Instead) > 0, "编译:退回必须附一个能照抄的替代(" & To_String (C.Instead) & ")");
      end;
      Check (not Comp ("reach 1 at 0 small").Ok, "编译:命令别人动 ⇒ 退回(我只推得动我自己)");
      Check (not Comp ("close 1 on 0").Ok, "编译:对着不是手的东西说合手 ⇒ 退回");
      Check (not Comp ("reach 0 onto 1 small until touch").Ok, "编译:量不出它鼓出多少时 onto 说不出口");
      Check (not Comp ("reach 0 at 1 small until free").Ok, "编译:量不出它鼓出多少时 until free 说不出口");
      Check (not Comp ("press 0 1 hard").Ok, "编译:量不出它鼓出多少时 press 说不出口(不知道哪个方向算压向它)");
      Set_Stands (True);
      Check (Comp ("reach 0 onto 1 small until touch").Ok, "编译:量得出它鼓出多少 ⇒ onto 就能说了");
      Check (Comp ("press 0 1 hard").Ok, "编译:量得出 ⇒ press 能说了");
      Check (not Comp ("press 0 1 hard small").Ok, "编译:press 说了劲就不许再说步子(一根轴上二选一)");
      Check (not Comp ("press 0 1").Ok, "编译:press 不说劲 ⇒ 退回");
      declare
         C6 : constant Plan.Compiled := Comp ("hold 0 above 1" & ASCII.LF & "reach 0 at 1 small until touch"
              & ASCII.LF & "while reach 0 left 1 small" & ASCII.LF & "press 0 1 firm"
              & ASCII.LF & "close 0 on 1 until resist" & ASCII.LF & "open 0" & ASCII.LF & "never 0 nearer 1");
      begin
         Check (C6.Ok and then Natural (C6.Goals.Length) = 7,
                "编译:一段程序里 7 条约束照收,没有"
                & "「一次最多四条」这种上限了(实" & Natural'Image (Natural (C6.Goals.Length)) & ")");
         Check (C6.Ok and then C6.Goals (2).Together, "编译:while 那一条标成【和上一条同一节里一起解】");
      end;
      Set_Stands (False);
      Check (not Comp ("hold 0 at 1" & ASCII.LF & "hold 0 above 1").Ok,
             "编译:两条 hold 抢同一行 ⇒ 退回(一定得牺牲一条,不许跑)");
      Check (Comp ("hold 0 above 1" & ASCII.LF & "reach 0 at 1 small until touch").Ok,
             "编译:一条 hold 一条 reach 不冲突 ⇒ 收");
      Check (not Comp ("reach 0 at baseball small").Ok, "编译:名字认不出 ⇒ 退回(不是语法错,是身体认不出)");
      --  眼睛从 1 编号(提示词里就是这么给的);以前按 0 起算 ⇒ 最后一只眼睛永远"不存在"
      Check (Comp ("look 1").Ok, "编译:第 1 只眼睛认");
      Check (not Comp ("look 0").Ok, "编译:没有第 0 只眼睛");
      Check (not Comp ("look 2").Ok, "编译:这具身体只有 1 只眼睛,说 2 就退回");
      Check (not Comp ("reach 0 at 9 small").Ok, "编译:点名一个看不到的东西 ⇒ 退回");
      declare
         C : constant Plan.Compiled := Comp ("say hi" & ASCII.LF & "reach 0 at 1 large until resist" & ASCII.LF & "done");
      begin
         Check (C.Ok and then Natural (C.Goals.Length) = 1 and then C.Done
                  and then To_String (C.Says) = "hi", "编译:say/done 不产生动作,reach 产生一条约束");
      end;
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
   Put_Line ((if Fails = 0 then "🟢 自检全过" else "🔴 自检失败" & Natural'Image (Fails) & " 条"));
   if Fails > 0 then
      raise Program_Error;
   end if;
end Selfcheck;
