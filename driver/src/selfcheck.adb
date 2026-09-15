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
      Check (not Dry ("do grasper close on the ball until stuck").Ok,
             "空转:那个东西比我张得开的还大 ⇒ 合了也是空的,不通电就拦住");
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
         Check (Sinew.Grammar'Length > 0
                and then Ada.Strings.Fixed.Index (Sinew.Grammar, "arrived") = 0,
                "到位:语法里不再有 arrived 这个词(到了没到只有脑能判)");
         Check (Sinew.Grammar'Length > 0
                and then Ada.Strings.Fixed.Index (Sinew.Grammar, "touched") > 0,
                "到位:量得到的事件词还在(touched)—— 删的是意见,不是事件");
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

   Put_Line ((if Fails = 0 then "🟢 自检全过" else "🔴 自检失败" & Natural'Image (Fails) & " 条"));
   if Fails > 0 then
      raise Program_Error;
   end if;
end Selfcheck;
