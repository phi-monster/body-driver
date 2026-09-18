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
with Ada.Containers;
with Interfaces; use type Interfaces.Unsigned_8;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Sinew;
with Plan;
with Runtime;
with Act;
with Geom;
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

   --  ── 顶面 / 贴边 / 横跨整幅:抓握要靠"顶面到桌面的一半",而近处的东西必然贴画面边 ──
   declare
      W : constant := 96;
      H : constant := 72;
      Dep : Floats := Filled (W * H, 0.80);
      R : Picture.Regions;
      Rad : constant Long_Float := 12.0;
      Seed : Long_Long_Integer := 11;
   begin
      for I in 0 .. W * H - 1 loop
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         Dep.Replace_Element (I, 0.80 + Long_Float (Seed mod 1000) * 1.0e-6 - 0.0005);
      end loop;
      --  一个半球:中心最高,边缘贴桌面 ⇒ 顶面必须比中位深度更近
      for Y in 21 .. 44 loop
         for X in 36 .. 59 loop
            declare
               Dx : constant Long_Float := Long_Float (X - 47);
               Dy : constant Long_Float := Long_Float (Y - 32);
               T : constant Long_Float := Rad * Rad - Dx * Dx - Dy * Dy;
            begin
               if T > 0.0 then
                  Dep.Replace_Element (Y * W + X, 0.80 - 0.04 * Sqrt (T) / Rad);
               end if;
            end;
         end loop;
      end loop;
      R := Picture.Cut (Dep, W, H, 0.125, 3.0);
      Check (Natural (R.Length) = 1, "顶面:半球切出" & Natural'Image (Natural (R.Length)) & " 块");
      if not R.Is_Empty then
         Check (R (0).Top < R (0).Depth - 0.005, "顶面:顶 " & Codec.Fmt (R (0).Top, 3) & " 比中位 " & Codec.Fmt (R (0).Depth, 3) & " 更近");
         Check (abs (R (0).Top + R (0).Height - 0.80) < 0.012, "顶面:顶 + 鼓高 = 桌面 " & Codec.Fmt (R (0).Top + R (0).Height, 3));
         declare
            It : Act.Item;
         begin
            It.Top := R (0).Top; It.Height := R (0).Height; It.Depth := R (0).Depth;
            Check (Act.Grab_Depth (It) > It.Top and then Act.Grab_Depth (It) > It.Depth and then Act.Grab_Depth (It) < It.Top + It.Height,
                   "抓握:瞄的高度在顶面和桌面之间、比中位深度(=皮)更深(" & Codec.Fmt (Act.Grab_Depth (It), 3) & ")");
            It.Top := 0.0;
            Check (abs (Act.Grab_Depth (It) - It.Depth) < 1.0e-12, "抓握:没量到顶面就退回中位深度");
         end;
      end if;
   end;
   declare
      W : constant := 96;
      H : constant := 72;
      Dep : Floats := Filled (W * H, 0.80);
      R : Picture.Regions;
      Seed : Long_Long_Integer := 13;
   begin
      for I in 0 .. W * H - 1 loop
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         Dep.Replace_Element (I, 0.80 + Long_Float (Seed mod 1000) * 1.0e-6 - 0.0005);
      end loop;
      --  贴右边的一块:凑近了要抓的东西就长这样,不许整块丢掉
      for Y in 26 .. 45 loop
         for X in 76 .. 95 loop
            Dep.Replace_Element (Y * W + X, 0.77);
         end loop;
      end loop;
      R := Picture.Cut (Dep, W, H, 0.125, 3.0);
      Check (R.Is_Empty, "贴边:严格规则下贴右边的块被丢掉(" & Natural'Image (Natural (R.Length)) & " 块)");
      R := Picture.Cut (Dep, W, H, 0.125, 3.0, Keep_Edge => True);
      Check (Natural (R.Length) = 1, "贴边:放宽之后它留下了(" & Natural'Image (Natural (R.Length)) & " 块)");
   end;
   declare
      W : constant := 96;
      H : constant := 72;
      Dep : Floats := Filled (W * H, 0.80);
      R : Picture.Regions;
      Seed : Long_Long_Integer := 17;
   begin
      for I in 0 .. W * H - 1 loop
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         Dep.Replace_Element (I, 0.80 + Long_Float (Seed mod 1000) * 1.0e-6 - 0.0005);
      end loop;
      --  横贯整幅的一条带 = 背景,必须丢
      for Y in 26 .. 45 loop
         for X in 0 .. 95 loop
            Dep.Replace_Element (Y * W + X, 0.77);
         end loop;
      end loop;
      R := Picture.Cut (Dep, W, H, 0.125, 3.0, Keep_Edge => True);
      Check (R.Is_Empty, "贴边:横跨整幅的带就算放宽也丢掉(" & Natural'Image (Natural (R.Length)) & " 块)");
   end;

   --  ── 明暗分两拨(Otsu):腕眼里按明暗切东西靠它;两拨分得开时分界落在两拨之间,单峰时说分不开 ──
   declare
      F : Floats;
      T : Long_Float;
   begin
      for I in 1 .. 200 loop
         F.Append (40.0 + Long_Float (I mod 7));      --  暗的一拨(桌面)
      end loop;
      for I in 1 .. 60 loop
         F.Append (210.0 + Long_Float (I mod 5));     --  亮的一拨(白球)
      end loop;
      T := Picture.Split (F);
      Check (not Picture.Is_Nan (T) and then T > 46.0 and then T < 210.0, "明暗:两拨分得开,分界 " & Codec.Fmt (T, 1) & " 落在两拨之间(暗拨 40–46,亮拨 210–214)");
      --  ⚠️ Split 是 Otsu:单峰的一堆数它照样给一个分界(它判不了"分不开");全一样的数才回 NaN。
      --  所以 Cut_Bright 在没有亮东西的画面里会把纹理切成碎块 —— 靠最少像素数和"横跨整幅就丢"兜住,认东西靠脑。
      F.Clear;
      for I in 1 .. 40 loop
         F.Append (100.0);
      end loop;
      Check (Picture.Is_Nan (Picture.Split (F)), "明暗:全一样 ⇒ 分不开(NaN)");
   end;

   --  ── 拿住了没:唯一分得开的那一条 ──
   declare
      Hu : constant Long_Float := 0.10;   --  我的手在那台不动的相机里往右挪了十分之一个画面(画幅比例)
      Hv : constant Long_Float := 0.00;
   begin
      Check (Act.Came_With_Me (0.10, 0.00, Hu, Hv), "拿住:它跟我挪了同样一段 ⇒ 拿住了");
      Check (Act.Came_With_Me (0.09, 0.01, Hu, Hv), "拿住:它跟我挪的差一点点(抖动)⇒ 仍然算拿住");
      Check (not Act.Came_With_Me (0.00, 0.00, Hu, Hv), "拿住:我抬了手它留在原地 ⇒ 没拿住");
      Check (not Act.Came_With_Me (0.00, -0.30, Hu, Hv),
             "拿住:我抬了手它朝另一个方向飞出去 ⇒ 没拿住(原地空了但是被撞飞的 —— 旧判据在这里判成拿住,三次假拿住全是它)");
      Check (not Act.Came_With_Me (0.10, 0.00, 0.00, 0.00), "拿住:我的手一步没挪 ⇒ 判不了,不许自称拿住");
      Check (Act.Came_With_Me (0.05, 0.00, Hu, Hv), "拿住:它只挪了我的一半(差正好是一半)⇒ 还算拿住,边界在这儿");
      Check (not Act.Came_With_Me (0.04, 0.00, Hu, Hv), "拿住:它挪得比我的一半还少 ⇒ 不算拿住,边界另一侧");
   end;

   --  ── Sinew:脑的嘴 ──
   declare
      use Sinew;
      G : constant Program := Sinew.Parse
        ("to pick up:" & ASCII.LF &
         "  repeat 3 times:" & ASCII.LF &
         "    do grasper above the ball must until touched or 20 steps" & ASCII.LF &
         "    try:" & ASCII.LF &
         "      do grasper close the ball until free" & ASCII.LF &
         "    or:" & ASCII.LF &
         "      do grasper open until settled" & ASCII.LF &
         "    end" & ASCII.LF &
         "  end" & ASCII.LF &
         "end" & ASCII.LF &
         "run pick up" & ASCII.LF &
         "say done my best");
   begin
      Check (G.Ok, "Sinew:一段带定义/循环/try 的完整程序解析得通" & (if G.Ok then "" else "(" & To_String (G.Err) & ")"));
      Check (Natural (G.Defs.Length) = 1 and then To_String (G.Defs (0).Name) = "pick up", "Sinew:定义被记下来了(名字可以是好几个词)");
   end;
   declare
      use Sinew;
      G : constant Program := Sinew.Parse ("do grasper touching the white ball small must until touched or 7 steps with my still eye");
      C : Constraint;
   begin
      Check (G.Ok and then Natural (G.Code.Length) = 1 and then G.Code (0).O = Op_Interval, "Sinew:一行 do = 一段区间");
      if G.Ok and then not G.Code.Is_Empty and then not G.Code (0).Cons.Is_Empty then
         C := G.Code (0).Cons (0);
         Check (C.Subj.K = Nk_Role and then C.Subj.R = Rl_Grasper, "Sinew:主语是【角色】,不是编号");
         Check (C.Obj.K = Nk_Thing and then To_String (C.Obj.Word) = "the white ball", "Sinew:宾语是一句名字,好几个词也认");
         Check (C.Sp = Sp_Small and then C.Rk = Rk_Must and then G.Code (0).Until_Oc = Oc_Touched and then G.Code (0).Max_Steps = 7,
                "Sinew:步子 / must / 结局 / or N steps 都读出来了");
         Check (G.Code (0).Eye = Ey_Still, "Sinew:「with my still eye」读出来了");
      end if;
   end;
   declare
      function Bad (S : String) return Boolean is (not Sinew.Parse (S).Ok);
   begin
      Check (Bad ("move joint 3 by 0.1"), "Sinew:关节号这种话语法上不存在 ⇒ 说不出口");
      Check (Bad ("do grasper touching the ball small"), "Sinew:不说【到什么为止】⇒ 退回");
      Check (Bad ("do touching the ball until touched"), "Sinew:不说【谁】⇒ 退回");
      Check (Bad ("do grasper the ball until touched"), "Sinew:不说关系 ⇒ 退回");
      Check (Bad ("do grasper touching the ball until soon"), "Sinew:until 后面不是结局词 ⇒ 退回");
      Check (Bad ("repeat 3 times:" & ASCII.LF & "do grasper open until settled"), "Sinew:块没有 end ⇒ 退回");
      Check (Bad ("end"), "Sinew:多一个 end ⇒ 退回");
      Check (Sinew.Parse ("do grasper close until stuck").Ok, "Sinew:close 可以不带宾语 —— 就在这儿合上");
   end;
   --  done 写在循环里 = 整段程序到此为止,不许说完"做完了"又循环回去
   declare
      use Sinew;
      use type Runtime.Yield;
      G : constant Program := Sinew.Parse
        ("repeat 3 times:" & ASCII.LF & "  do grasper open until settled" & ASCII.LF & "  done" & ASCII.LF & "end" & ASCII.LF & "say never here");
      M : Runtime.Machine;
      W : Runtime.Yield;
      I : Sinew.Instr;
      Seen_Done, Then_Finished : Boolean := False;
   begin
      Runtime.Advance (G, M, W, I);                 --  第一段 open
      if W = Runtime.Y_Interval then
         Runtime.Report (G, M, Oc_Settled);
         Runtime.Advance (G, M, W, I);              --  done
         Seen_Done := W = Runtime.Y_Done;
         Runtime.Advance (G, M, W, I);              --  之后必须是 Finished,不许回到循环头、也不许走到 say
         Then_Finished := W = Runtime.Y_Finished;
      end if;
      Check (Seen_Done and then Then_Finished, "Sinew:循环里的 done 之后程序结束(不再循环、不走后面的行)");
   end;

   --  🔴 每个结局词都要有【自己】的判法 —— 语言收下一个词,身体悄悄换成另一个词的行为,是本仓最贵的一类 bug。
   --  这张表就是语言的承诺,逐词钉死;比的是【身体的真实映射】,不是期望表自己跟自己。
   declare
      use Sinew;
      use type Monitor.Until_Kind;
      type Row is record
         O : Outcome;
         K : Monitor.Until_Kind;
      end record;
      Want : constant array (1 .. 7) of Row :=
        [(Oc_Touched, Monitor.U_Contact), (Oc_Stuck, Monitor.U_Resist),
         (Oc_Slipped, Monitor.U_Slip),    (Oc_Settled, Monitor.U_Settle),
         (Oc_Stalled, Monitor.U_Stall),
         (Oc_Timeout, Monitor.U_Steps),   (Oc_Free, Monitor.U_Steps)];
      All_Right : Boolean := True;
      Round_Trip : Boolean := True;
      Distinct : Boolean := True;
   begin
      for R of Want loop
         if Act.Until_Of (R.O) /= R.K then
            All_Right := False;
         end if;
         if Act.Kind_Of_Word (Act.Until_Word (R.O)) /= Act.Until_Of (R.O) then
            Round_Trip := False;
         end if;
      end loop;
      --  五个"有自己事件"的词必须两两不同(前五行);timeout / free 走步数上限是设计(free 由合手那一节判)
      for I in 1 .. 5 loop
         for J in I + 1 .. 5 loop
            if Act.Until_Of (Want (I).O) = Act.Until_Of (Want (J).O) then
               Distinct := False;
            end if;
         end loop;
      end loop;
      Check (All_Right, "语言:每个结局词都接到自己的判法上(不许并进兜底的步数上限)");
      Check (Round_Trip, "语言:结局词转成字符串再转回来,判法不变");
      Check (Distinct, "语言:有自己事件的五个结局词两两不同");
      --  身体段末那句话 → 结局词:每一句都要落到它自己的词上(顺序错一处就会串词,这里逐句钉死)
      Check (Act.Classify ("contact: something I was not pushing moved when I moved - I am touching it", "") = Oc_Touched, "语言:碰到 → touched");
      Check (Act.Classify ("resist: I commanded a push and my body did not go", "") = Oc_Stuck, "语言:顶住 → stuck");
      Check (Act.Classify ("slip: what I was holding has left my fingers", "") = Oc_Slipped, "语言:滑了 → slipped");
      Check (Act.Classify ("settle: the picture stopped changing", "") = Oc_Settled, "语言:画面不变 → settled(含 stopped 一词也不许串成 stalled)");
      Check (Act.Classify ("amount: stopped getting closer (still about 3.0 pushes away) - either something holds me or this arm cannot reach farther from here", "") = Oc_Stalled,
             "语言:差距不缩 → stalled");
      Check (Act.Classify ("steps: hit the step cap (12)", "") = Oc_Timeout, "语言:步数用完 → timeout");
      Check (Act.Classify ("lost sight: two steps in a row I could not find what I am tracking in this picture; I stopped rather than move blind", "") = Oc_Lost,
             "语言:跟丢 → lost(含 steps 一词也不许串成 timeout)");
      Check (Act.Classify ("amount: arrived (in the picture and at the same distance as my fingers)", "") = Oc_Arrived, "语言:到位 → arrived");
      Check (Act.Classify ("steps: I took the steps you asked for", "I closed grip 2 until the picture stopped changing (9 steps, reading 0.000, empty-close reading 0.000); after a small lift it came with my hand ⇒ held (my hand moved 0.020 of a frame, it moved 0.019, the two differ by 0.001)") = Oc_Free,
             "语言:合完抬一截它跟着我走 → free");
      Check (Act.Classify ("steps: I took the steps you asked for", "I closed grip 2 until the picture stopped changing (9 steps, reading 0.000, empty-close reading 0.000); after a small lift it did NOT come with my hand ⇒ not held - and its old place is empty, so I knocked it away rather than picked it up; I opened it again") = Oc_Slipped,
             "语言:合了没拿住(撞飞)→ slipped");
      Check (Act.Classify ("steps: I took the steps you asked for", "I did NOT close grip 2: cage check in this camera: my grip centre is 0.120 of a frame from the thing (allowed 0.050)") = Oc_Stalled,
             "语言:没笼住不敢合 → stalled");
      Check (Act.Classify ("", "I opened grip 2 (4 steps, reading 1.000)") = Oc_Arrived, "语言:只张开、没有要走的段 → arrived");
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
               if Q /= R and then Q /= Re_None and then not Act.Rel_Has_Own_Branch (Q) and then Act.Rel_Cmd (Q) = Act.Rel_Cmd (R) then
                  Uniq := False;
               end if;
            end loop;
         end if;
      end loop;
      Check (Named, "语言:每个关系词都有自己的字(没有一个落进兜底的 ?)");
      Check (Uniq, "语言:关系词两两不同(不许两个词做同一件事)");
      Check (not (Act.Role_Wants (Rl_Grasper, Act.Grip) and then Act.Role_Wants (Rl_Pusher, Act.Grip)), "语言:grasper 和 pusher 互斥(合得拢的零件不许算 pusher)");
   end;

   --  ── 编译器:说不了的当场退回,说得了的放行;空转抓停不下来的循环 ──
   declare
      use Sinew;
      Facts : Plan.Facts_Vectors.Vector;
      B : Plan.Bind_Vectors.Vector;
      Zero : Plan.Item_Facts;
      Gr, Th : Plan.Item_Facts;
      function Verdict_Of (Src : String; Own : Boolean := False) return Plan.Verdict is
        (Plan.Check (Sinew.Parse (Src), Facts, B, Own));
      function Dry (Src : String) return Plan.Verdict is
        (Plan.Dry_Run (Sinew.Parse (Src), Facts, B));
   begin
      Facts.Append (Zero);
      Gr.Exists := True; Gr.Mine := True; Gr.Grasp := True; Gr.Span := 0.13; Gr.Label := To_Unbounded_String ("grasper(第1 只手)");
      Facts.Append (Gr);                                     --  1 号 = 爪心
      Th.Exists := True; Th.Stands := True; Th.Size := 0.10;
      Facts.Append (Th);                                     --  2 号 = 球
      B.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("grasper"), Item => 1, Tried => Null_Unbounded_String));
      B.Append (Plan.Bind_Entry'(Key => To_Unbounded_String ("the ball"), Item => 2, Tried => Null_Unbounded_String));
      Check (Verdict_Of ("do grasper into the ball until touched or 30 steps").Ok, "编译:说得出口的一句放行");
      Check (not Verdict_Of ("do grasper touching the ball until arrived").Ok, "编译:until arrived 退回(到没到只有脑能判)");
      Check (not Verdict_Of ("do grasper press the ball firm until stuck").Ok, "编译:press 这版做不了 ⇒ 退回");
      Check (not Verdict_Of ("do grasper touching the ball until free").Ok, "编译:until free 不跟 close 一起 ⇒ 退回");
      Check (Verdict_Of ("do grasper close the ball until free").Ok, "编译:close … until free 放行");
      Check (not Verdict_Of ("do grasper onto the ball until stuck or 5 steps", Own => True).Ok, "编译:跟着我动的眼里 onto 判不了 ⇒ 退回");
      Check (Verdict_Of ("do grasper farther the ball until stuck or 5 steps", Own => True).Ok, "编译:跟着我动的眼里 farther 放行(= 让它的远近读数变一截)");
      Check (Verdict_Of ("do grasper nearer the ball until touched or 5 steps", Own => False).Ok, "编译:不动的眼里 nearer 放行");
      Check (not Verdict_Of ("do the ball touching grasper until touched").Ok, "编译:主语不是我身上的东西 ⇒ 退回");
      Check (not Verdict_Of ("do grasper touching the cup until touched").Ok, "编译:认不出的名字 ⇒ 退回");
      Check (not Dry ("repeat until touched:" & ASCII.LF & "do grasper open until settled" & ASCII.LF & "end").Ok,
             "空转:等一个这段程序里永远不会发生的结局 ⇒ 停不下来 ⇒ 退回");
      Check (Dry ("repeat until touched:" & ASCII.LF & "do grasper into the ball until touched or 30 steps" & ASCII.LF & "end").Ok,
             "空转:出口真到得了的循环放行");
      Check (not Dry ("run fly").Ok, "空转:run 一个没 to 过的名字 ⇒ 退回");
      declare
         Wide : Plan.Item_Facts := Th;
      begin
         Wide.Size := 0.30;
         Facts.Replace_Element (2, Wide);
         Check (not Dry ("do grasper close the ball until free").Ok, "空转:张不到那么开却要合 ⇒ 退回");
      end;
   end;

   --  ── 几何(视线、三角化、朝向拟合)──
   declare
      G : Geom.Cam_Geo;
      Rv : constant Geom.V3 := [0.3, -0.2, 0.5];
      R : constant Geom.M3 := Geom.Rodrigues (Rv);
      Back : constant Geom.V3 := Geom.Rot_Vec (R);
      Pw : constant Geom.V3 := [0.27, 0.03, 0.80];
      function Pose_At (X, Y, Z : Long_Float) return Plug.Arm_Pose is
        ([X, Y, Z, 0.707, 0.0, 0.0, 0.7072]);
      Obs : Geom.Obs_Vectors.Vector;
      Ok : Boolean;
      Ang : Long_Float;
   begin
      Check (Geom.Norm ([Back (0) - Rv (0), Back (1) - Rv (1), Back (2) - Rv (2)]) < 1.0e-9, "几何:转向量 → 矩阵 → 转向量 回得来");
      G.F := 397.0; G.Cx := 320.0; G.Cy := 240.0;
      --  真的相机朝向:取 CA1 实测那台(相机在手系里的列)
      --  四位小数的矩阵不正交;经转向量再回来就是一个真正的旋转
      G.R_Ce := Geom.Rodrigues (Geom.Rot_Vec ([[0.0038, 0.4971, -0.8677], [-1.0, 0.0032, -0.0026], [0.0015, 0.8677, 0.4971]]));
      G.Valid := True;
      --  投影再沿视线回去,应指向同一个点
      declare
         P0 : constant Plug.Arm_Pose := Pose_At (0.30, -0.35, 0.92);
         U, V : Long_Float;
         Front : Boolean;
      begin
         Geom.Project (G, P0, Pw, U, V, Front);
         Check (Front and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0, "几何:球投在画面里 (" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ")");
         declare
            D : constant Geom.V3 := Geom.Ray (G, P0, U, V);
            To : constant Geom.V3 := [Pw (0) - P0 (0), Pw (1) - P0 (1), Pw (2) - P0 (2)];
            N : constant Long_Float := Geom.Norm (To);
         begin
            Check (abs (D (0) * To (0) / N + D (1) * To (1) / N + D (2) * To (2) / N - 1.0) < 1.0e-9, "几何:像素回推的视线正对着那个点");
         end;
      end;
      --  五个平移位姿看同一个点 ⇒ 交点回到那个点
      for K in 0 .. 4 loop
         declare
            P : constant Plug.Arm_Pose := Pose_At (0.30 + 0.03 * Long_Float (K mod 3), -0.35 + 0.02 * Long_Float (K / 2), 0.92 + 0.03 * Long_Float (K mod 2));
            U, V : Long_Float;
            Front : Boolean;
         begin
            Geom.Project (G, P, Pw, U, V, Front);
            Obs.Append (Geom.Obs'(Pose => P, U => U, V => V));
         end;
      end loop;
      declare
         T : constant Geom.V3 := Geom.Triangulate (G, Obs);
      begin
         Check (Geom.Norm ([T (0) - Pw (0), T (1) - Pw (1), T (2) - Pw (2)]) < 1.0e-6, "几何:五条视线交回原点(差 " & Codec.Fmt (Geom.Norm ([T (0) - Pw (0), T (1) - Pw (1), T (2) - Pw (2)]) * 1000.0, 3) & " mm)");
      end;
      --  盲拟合朝向:只给像素和位姿,应解回同一个朝向
      declare
         G2 : Geom.Cam_Geo;
      begin
         G2.F := G.F; G2.Cx := G.Cx; G2.Cy := G.Cy;
         Geom.Fit (G2, Obs, Ok);
         Ang := Geom.Norm (Geom.Rot_Vec (Geom.Mul (Geom.Tr (G2.R_Ce), G.R_Ce)));
         Check (Ok and then Ang < 0.01, "几何:盲拟合解回相机朝向(差 " & Codec.Fmt (Ang * 57.3, 2) & "°,残差 " & Codec.Fmt (G2.Rms, 3) & " px)");
         --  故意给错的朝向,同一批观测的交点就不在原处 —— 这条焊缝会响
         declare
            Bad : Geom.Cam_Geo := G;
            T : Geom.V3;
         begin
            Bad.R_Ce := Geom.Rodrigues ([0.0, 0.0, 1.0]);
            T := Geom.Triangulate (Bad, Obs);
            Check (Geom.Norm ([T (0) - Pw (0), T (1) - Pw (1), T (2) - Pw (2)]) > 0.01, "几何:朝向错了交点就错(焊缝会响)");
         end;
      end;
      --  存 / 读
      declare
         Gs, Gs2 : Geom.Geo_Vectors.Vector;
         Note : String (1 .. 160);
      begin
         Gs.Append (Geom.No_Geo); Gs.Append (G);
         Gs (1).Tip := [0.0, -0.0045, -0.085]; Gs (1).Gap := 0.0889; Gs (1).Tip_Valid := True;
         Geom.Save ("/tmp/selfcheck_geo.json", Gs);
         Geom.Load ("/tmp/selfcheck_geo.json", Gs2, 2, Note);
         Check (Natural (Gs2.Length) = 2 and then Gs2 (1).Valid and then Gs2 (1).Tip_Valid and then abs (Gs2 (1).Gap - 0.0889) < 1.0e-6
                and then abs (Gs2 (1).R_Ce (0, 2) - G.R_Ce (0, 2)) < 1.0e-6 and then not Gs2 (0).Valid, "几何:存了再读回来一样");
      end;
   end;
   --  键盘上不许有死键(QW4:press 被我漏了一整条产生式,按了 5 次都白按)。
   --  规矩:任何在这一版【说得出口但落地就退回】的词,都不许出现在交给解码器的语法里。
   declare
      G_Head : constant String := Sinew.EBNF (Plan.Usable_Rels (False), "grasper pusher");
      G_Own  : constant String := Sinew.EBNF (Plan.Usable_Rels (True), "grasper pusher");
      function Has (S, W : String) return Boolean is
        (for some I in S'First .. S'Last - W'Length + 1 => S (I .. I + W'Length - 1) = W);
   begin
      Check (not Has (G_Head, "press") and then not Has (G_Own, "press"),
             "键盘:press 这一版做不了 ⇒ 语法里一个 press 都不许有");
      Check (not Has (G_Head, """me""") and then not Has (G_Own, """me"""),
             "键盘:me 在这具身体上绑不上 ⇒ 语法里不许给这个键");
      Check (Has (G_Head, """grasper""") and then Has (G_Own, """grasper"""),
             "键盘:能绑上的角色必须给");
      Check (Has (G_Head, """onto""") and then not Has (G_Own, """onto"""),
             "键盘:onto 只在别的眼里说得出口,自己那只眼里不许给");
   end;
   Put_Line ((if Fails = 0 then "🟢 自检全过" else "🔴 自检失败" & Natural'Image (Fails) & " 条"));
   if Fails > 0 then
      raise Program_Error;
   end if;
end Selfcheck;
