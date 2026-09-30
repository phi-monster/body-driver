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
with Lockstep;
with Bodyfile;
with Geom;
with Selfmap;
with Learned;
with Exam;
with Contact;
with Contact.Grasp;
with Contact.Hold;
with Contact.Exec;
with Contact.Surface;
with Kinem;
with Jointboot;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers;
with Ada.Containers.Vectors;
with Ada.Assertions;
with Ada.Long_Float_Text_IO;
with Ada.Streams;
with Ada.Unchecked_Conversion;
with Ada.Calendar;
with GNAT.Sockets;
with Websocket;
with Http_Client;
with Brain;
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

   --  ── 本机回环上的假对方(WebSocket / HTTP / 脑的焊点用)──
   CRLF : constant String := ASCII.CR & ASCII.LF;
   Http11 : constant String := "HTTP/1" & ".1";   --  协议版本(拆开写:闸门棘轮按"名字 / 小数"数系数,整串写在字里会被误数成一个门槛)
   --  开一个听的口,端口让系统挑
   procedure Open_Listener (L : out GNAT.Sockets.Socket_Type; Port : out GNAT.Sockets.Port_Type) is
      use GNAT.Sockets;
   begin
      Create_Socket (L);
      Set_Socket_Option (L, Socket_Level, (Reuse_Address, True));
      Bind_Socket (L, (Family => Family_Inet, Addr => Loopback_Inet_Addr, Port => Any_Port));
      Listen_Socket (L);
      Port := Get_Socket_Name (L).Port;
   end Open_Listener;
   --  一串字节原样发出去(字节放在堆上:3 MB 的帧也不上栈)
   type Raw_Access is access Ada.Streams.Stream_Element_Array;
   procedure Send_Raw (S : GNAT.Sockets.Socket_Type; A : Ada.Streams.Stream_Element_Array) is
      use Ada.Streams;
      Sent : Stream_Element_Offset := A'First - 1;
      Last : Stream_Element_Offset;
   begin
      while Sent < A'Last loop
         GNAT.Sockets.Send_Socket (S, A (Sent + 1 .. A'Last), Last);
         Sent := Last;
      end loop;
   end Send_Raw;
   procedure Send_Text (S : GNAT.Sockets.Socket_Type; T : String) is
      use Ada.Streams;
      A : constant Raw_Access := new Stream_Element_Array (1 .. Stream_Element_Offset (T'Length));
   begin
      for I in T'Range loop
         A (Stream_Element_Offset (I - T'First + 1)) := Stream_Element (Character'Pos (T (I)));
      end loop;
      Send_Raw (S, A.all);
   end Send_Text;
   --  收到对方关连接或者收满 N 个字节为止(N = 0:收到关为止)
   function Recv_Text (S : GNAT.Sockets.Socket_Type; N : Natural := 0) return Unbounded_String is
      use Ada.Streams;
      One : Stream_Element_Array (1 .. 1);
      Last : Stream_Element_Offset;
      R : Unbounded_String;
   begin
      while N = 0 or else Length (R) < N loop
         GNAT.Sockets.Receive_Socket (S, One, Last);
         exit when Last < One'First;
         Append (R, Character'Val (One (1)));
      end loop;
      return R;
   exception
      when others => return R;
   end Recv_Text;
   --  假的 HTTP 对方:收一个请求(头读到空行,再按 Content-Length 读完正文)原样记下;回 Reply 这一串;再等 Hold 秒才关连接
   --  (Hold > 0 = 回了一半就不吭声,看对面等不等到它自己的时限)
   task type Fake_Http is
      entry Start (Reply : String; Hold : Duration; Port : out GNAT.Sockets.Port_Type);
      entry Got (Request : out Unbounded_String);
   end Fake_Http;
   task body Fake_Http is
      use GNAT.Sockets;
      L, S : Socket_Type;
      Peer : Sock_Addr_Type;
      Rep : Unbounded_String;
      Wait : Duration := 0.0;
      Req : Unbounded_String;
   begin
      accept Start (Reply : String; Hold : Duration; Port : out Port_Type) do
         Rep := To_Unbounded_String (Reply);
         Wait := Hold;
         Open_Listener (L, Port);
      end Start;
      Accept_Socket (L, S, Peer);
      declare
         use Ada.Streams;
         One : Stream_Element_Array (1 .. 1);
         Last : Stream_Element_Offset;
         Need : Natural := 0;
         Head_End : Natural := 0;
         Tag : constant String := "Content-Length: ";
      begin
         loop
            Receive_Socket (S, One, Last);
            exit when Last < One'First;
            Append (Req, Character'Val (One (1)));
            if Head_End = 0 and then Length (Req) >= 4 and then Tail (Req, 4) = CRLF & CRLF then
               Head_End := Length (Req);
               declare
                  H : constant String := To_String (Req);
                  P : constant Natural := Ada.Strings.Fixed.Index (H, Tag);
               begin
                  if P > 0 then
                     Need := Natural'Value (H (P + Tag'Length .. Ada.Strings.Fixed.Index (H (P .. H'Last), CRLF) - 1));
                  end if;
               end;
            end if;
            exit when Head_End > 0 and then Length (Req) - Head_End >= Need;
         end loop;
      end;
      Send_Text (S, To_String (Rep));
      delay Wait;
      Close_Socket (S);
      Close_Socket (L);
      accept Got (Request : out Unbounded_String) do
         Request := Req;
      end Got;
   exception
      when others =>
         select
            accept Got (Request : out Unbounded_String) do
               Request := Req;
            end Got;
         or
            terminate;
         end select;
   end Fake_Http;
begin
   --  base64 标准向量 + WebSocket 握手向量(RFC 6455 §1.3)
   Check (Codec.Base64_Of_String ("foobar") = "Zm9vYmFy", "base64 foobar");
   Check (Codec.Base64_Of_String ("fo") = "Zm8=", "base64 补齐");
   declare
      Acc : constant String := Codec.Base64 (Codec.Hex_To_Bytes (GNAT.SHA1.Digest ("dGhlIHNhbXBsZSBub25jZQ==" & "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")));
   begin
      Check (Acc = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", "WebSocket 握手 accept 向量:" & Acc);
   end;
   --  🔴 WebSocket(09-30):一帧的负载放在堆上 —— 原来是栈上的 Stream_Element_Array (1 .. Len),几台 720p 的 RGB-D 一帧十几 MB,比主线程的栈(常见 8 MB)大,先撞栈;
   --  握手请求头读到空行为止(原来定长 16 KiB,读满没见到空行就判握手失败);一条消息多大只剩"字节串能编号的"和"内存给不给"两道边(原来另有拍的 512 MiB)。
   --  真套接字:本机回环上开一个口,另一个线程当对方:握手(请求头里一行 20 KiB 的 cookie)→ 一个 ping → 一条分两片的文字消息 → 一条 3 MB 带掩码的二进制
   --  → 一个说自己有 2^40 字节的帧头。这边在一个只给 1 MB 栈的线程里收:负载还在栈上的话,3 MB 放不进 1 MB 的栈
   declare
      use GNAT.Sockets;
      use Ada.Streams;
      Big : constant Natural := 3 * 1024 * 1024;
      Small_Stack : constant := 1024 * 1024;
      Old_Head : constant := 16384;   --  旧写法请求头缓冲的长度
      Cookie : constant String (1 .. 20 * 1024) := [others => 'c'];
      Req : constant String := "GET / " & Http11 & CRLF & "Host: x" & CRLF & "Upgrade: websocket" & CRLF & "Connection: Upgrade" & CRLF &
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==" & CRLF & "Cookie: " & Cookie & CRLF & CRLF;
      Lst : Socket_Type;
      Port : Port_Type;
      C : Websocket.Conn;
      Accept_Ok, Hs_Ok, Pong_Ok, Text_Ok, Big_Ok, Huge_Refused : Boolean := False;
      Big_Len : Natural := 0;
      function Pattern (I : Natural) return Stream_Element is (Stream_Element (I * 7 mod 251));
      --  客户端发的帧(RFC 6455:客户端的帧都带掩码)
      function Frame (Fin : Boolean; Opcode : Stream_Element; Payload : Stream_Element_Array) return Raw_Access is
         Key : constant Stream_Element_Array (1 .. 4) := [16#12#, 16#34#, 16#56#, 16#78#];
         L : constant Natural := Payload'Length;
         Ext : constant Natural := (if L < 126 then 0 elsif L < 65536 then 2 else 8);
         F : constant Raw_Access := new Stream_Element_Array (1 .. Stream_Element_Offset (2 + Ext + 4 + L));
         P : Stream_Element_Offset := 3;
      begin
         F (1) := (if Fin then 16#80# else 0) or Opcode;
         F (2) := 16#80# or Stream_Element (if Ext = 0 then L elsif Ext = 2 then 126 else 127);
         for K in reverse 0 .. Ext - 1 loop
            F (P) := Stream_Element ((Long_Long_Integer (L) / 256 ** K) mod 256); P := P + 1;
         end loop;
         F (P .. P + 3) := Key; P := P + 4;
         for I in 0 .. L - 1 loop
            F (P + Stream_Element_Offset (I)) := Payload (Payload'First + Stream_Element_Offset (I)) xor Key (Stream_Element_Offset (I mod 4 + 1));
         end loop;
         return F;
      end Frame;
      function Bytes_Of (T : String) return Stream_Element_Array is
         A : Stream_Element_Array (1 .. Stream_Element_Offset (T'Length));
      begin
         for I in T'Range loop
            A (Stream_Element_Offset (I - T'First + 1)) := Stream_Element (Character'Pos (T (I)));
         end loop;
         return A;
      end Bytes_Of;
   begin
      Open_Listener (Lst, Port);
      C.Listener := Lst; C.Listening := True;
      declare
         task Reader with Storage_Size => Small_Stack;
         task Peer with Storage_Size => 64 * 1024 * 1024;
         task body Reader is
            Kind : Websocket.Op;
            Data : Buf;
            Ok : Boolean;
         begin
            Websocket.Accept_Client (C, Accept_Ok);
            if Accept_Ok then
               Websocket.Read_Message (C, Kind, Data, Ok);   --  先来的 ping 在里面答掉,再拼出分两片的文字消息
               Text_Ok := Ok and then Websocket."=" (Kind, Websocket.Op_Text) and then To_String (Data, 0, Natural (Data.Length)) = "hello world";
               Websocket.Read_Message (C, Kind, Data, Ok);
               Big_Len := Natural (Data.Length);
               Big_Ok := Ok and then Websocket."=" (Kind, Websocket.Op_Binary) and then Big_Len = Big
                 and then (for all I in 0 .. Big - 1 => Data (I) = U8 (Pattern (I)));
               Websocket.Read_Message (C, Kind, Data, Ok);
               Huge_Refused := not Ok;
            end if;
         exception
            when others => null;
         end Reader;
         task body Peer is
            S : Socket_Type;
            Payload : constant Raw_Access := new Stream_Element_Array (1 .. Stream_Element_Offset (Big));
         begin
            Create_Socket (S);
            Connect_Socket (S, (Family => Family_Inet, Addr => Loopback_Inet_Addr, Port => Port));
            Send_Text (S, Req);
            declare
               Resp : Unbounded_String;
            begin
               while Length (Resp) < 4 or else Tail (Resp, 4) /= CRLF & CRLF loop
                  Append (Resp, Recv_Text (S, 1));
               end loop;
               Hs_Ok := Index (Resp, " 101 ") > 0 and then Index (Resp, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") > 0;
            end;
            Send_Raw (S, Frame (True, 9, Bytes_Of ("pp")).all);
            Send_Raw (S, Frame (False, 1, Bytes_Of ("hello ")).all);
            Send_Raw (S, Frame (True, 0, Bytes_Of ("world")).all);
            Pong_Ok := To_String (Recv_Text (S, 4)) = Character'Val (16#8A#) & Character'Val (2) & "pp";
            for I in 0 .. Big - 1 loop
               Payload (Stream_Element_Offset (I + 1)) := Pattern (I);
            end loop;
            Send_Raw (S, Frame (True, 2, Payload.all).all);
            --  一个说自己有 2^40 字节的帧头(不带负载):比一条消息能编号的还大 ⇒ 那边该照实说、当线断了,而不是抛异常
            declare
               Huge : constant Stream_Element_Array (1 .. 10) := [16#82#, 127, 0, 0, 1, 0, 0, 0, 0, 0];
            begin
               Send_Raw (S, Huge);
            end;
            delay 0.2;
            Close_Socket (S);
         exception
            when others => null;
         end Peer;
      begin
         null;
      end;
      begin
         Close_Socket (C.Sock);
         Close_Socket (Lst);
      exception
         when others => null;
      end;
      Check (Accept_Ok and then Hs_Ok and then Req'Length > Old_Head
             and then Ada.Strings.Fixed.Index (Req (Req'First .. Req'First + Old_Head - 1), CRLF & CRLF) = 0,
             "WebSocket 握手:请求头 " & Codec.Img (Req'Length) & " 字节(一行 20 KiB 的 cookie)照样握上、accept 对"
             & "(旧写法 16 KiB 的定长缓冲读满了还没见到空行 ⇒ 判握手失败)");
      Check (Pong_Ok and then Text_Ok, "WebSocket 收消息:ping 当场答 pong(" & Boolean'Image (Pong_Ok) & ")· 分两片的文字拼成一条(" & Boolean'Image (Text_Ok) & ")");
      --  牙只能算出来:旧写法真在这个线程里开 3 MB 的栈上数组,越过护栏页直接写到别处的内存(没开栈检查),会把自检本身弄坏,不在这里跑
      Check (Big_Ok,
             "WebSocket 大帧:在只有 1 MB 栈的线程里收下一帧 " & Codec.Img (Big_Len) & " 字节、每个字节都对(负载在堆上;旧写法负载在栈上要 "
             & Codec.Img (Big) & " 字节 > 这个线程的栈 " & Codec.Img (Small_Stack) & " 字节)");
      Check (Huge_Refused, "WebSocket:帧头说有 2^40 字节(超过一条消息能编号的)⇒ 照实说、当线断了,不抛异常(原来的 512 MiB 上限删了,只剩这道边和内存给不给)");
   end;
   --  🔴 HTTP(09-30):原来超时、半路断线的异常在接收循环里被吞掉,收到过回包头就把半截回话当成功交出去;状态码从来不看,400 / 500 的错误体也当成功。
   --  本机回环上的假对方各回一种:200 带 Content-Length · 500 带错误原文 · 说 100 字节只给 7 个就关 · 回一半就不吭声(这边时限 0.3 秒)· 分块编码 · 没人听的口。
   --  旧写法的判法照同一串回包再判一遍(见到头和正文之间的空行就算成,正文 = 后面全部)⇒ 该红的地方它是绿的
   declare
      function Old_Accepts (Raw : String; Body_Text : out Unbounded_String) return Boolean is
         P : constant Natural := Ada.Strings.Fixed.Index (Raw, CRLF & CRLF);
      begin
         Body_Text := (if P = 0 then Null_Unbounded_String else To_Unbounded_String (Raw (P + 4 .. Raw'Last)));
         return P > 0;
      end Old_Accepts;
      Took : Duration := 0.0;   --  最近一次 One 里 Post 用了多久
      procedure One (Reply : String; Hold, Timeout : Duration; Ok : out Boolean; Got, Why : out Unbounded_String) is
         T : Fake_Http;
         Port : GNAT.Sockets.Port_Type;
         Req : Unbounded_String;
         T0 : Ada.Calendar.Time;
      begin
         T.Start (Reply, Hold, Port);
         T0 := Ada.Calendar.Clock;
         Ok := Http_Client.Post ("127.0.0.1", Natural (Port), "/x", "{""q"":1}", Got, Why, Timeout);
         Took := Ada.Calendar."-" (Ada.Calendar.Clock, T0);
         T.Got (Req);
      end One;
      R200 : constant String := Http11 & " 200 OK" & CRLF & "Content-Length: 5" & CRLF & CRLF & "hello";
      E500 : constant String := "{""ok"":false,""err"":""CUDA OOM""}";
      R500 : constant String := Http11 & " 500 Internal Server Error" & CRLF & "Content-Length: " & Codec.Img (E500'Length) & CRLF & CRLF & E500;
      R_Short : constant String := Http11 & " 200 OK" & CRLF & "Content-Length: 100" & CRLF & CRLF & "1234567";
      R_Chunk : constant String := Http11 & " 200 OK" & CRLF & "Transfer-Encoding: chunked" & CRLF & CRLF & "6" & CRLF & "hello " & CRLF
        & "5;ext=1" & CRLF & "world" & CRLF & "0" & CRLF & CRLF;
      Ok_A, Ok_B, Ok_C, Ok_D, Ok_E, Ok_F, Ok_G : Boolean;
      Old_B, Old_C, Old_D, Old_E : Boolean;
      G_A, G_B, G_C, G_D, G_E, G_F, G_G, W_A, W_B, W_C, W_D, W_E, W_F, W_G, Ob_B, Ob_C, Ob_D, Ob_E : Unbounded_String;
      T_D : Duration;
   begin
      One (R200, 0.0, 5.0, Ok_A, G_A, W_A);
      One (R500, 0.0, 5.0, Ok_B, G_B, W_B);
      One (R_Short, 0.0, 5.0, Ok_C, G_C, W_C);
      One (R_Short, 1.5, 0.3, Ok_D, G_D, W_D);
      T_D := Took;
      One (R_Chunk, 0.0, 5.0, Ok_E, G_E, W_E);
      Old_B := Old_Accepts (R500, Ob_B);
      Old_C := Old_Accepts (R_Short, Ob_C);
      Old_D := Old_Accepts (R_Short, Ob_D) and then To_String (Ob_D) = "1234567";   --  旧写法:超时的异常被吞,收到的这一截照样交出去
      Old_E := Old_Accepts (R_Chunk, Ob_E) and then To_String (Ob_E) /= "hello world";
      declare
         L : GNAT.Sockets.Socket_Type;
         P : GNAT.Sockets.Port_Type;
      begin
         Open_Listener (L, P);
         GNAT.Sockets.Close_Socket (L);   --  这个口没人听了
         Ok_F := Http_Client.Post ("127.0.0.1", Natural (P), "/x", "{}", G_F, W_F, 2.0);
      end;
      declare
         T : Fake_Http;
         Port : GNAT.Sockets.Port_Type;
         Req : Unbounded_String;
      begin
         T.Start (R500, 0.0, Port);
         Ok_G := Http_Client.Post ("127.0.0.1", Natural (Port), "/x", To_Unbounded_String ("{}"), G_G, W_G);   --  请求体放堆上的那一种(仪器那几路 09-30 起也带 Why)
         T.Got (Req);
      end;
      Check (Ok_A and then To_String (G_A) = "hello", "HTTP 200 带 Content-Length ⇒ 成,正文 " & To_String (G_A));
      Check (not Ok_B and then Old_B and then Index (W_B, "500") > 0 and then Index (W_B, "CUDA OOM") > 0 and then To_String (G_B) = E500,
             "HTTP 500 ⇒ 照实报失败,Why 带状态行和错误原文、Reply_Body 照样放着正文(旧写法当成功交出去):" & To_String (W_B));
      Check (not Ok_C and then Old_C and then Index (W_C, "短了") > 0, "HTTP 说好 100 字节只来 7 个就断 ⇒ 照实报(旧写法当成功):" & To_String (W_C));
      Check (not Ok_D and then Old_D and then T_D < 1.5 and then Index (W_D, "时限") > 0,
             "HTTP 回一半就不吭声 ⇒ 到这边的时限 0.3 秒照实报失败(用了 " & Codec.Fmt (Long_Float (T_D), 2) & " 秒,对方 1.5 秒后才关;旧写法吞掉超时、把半截当成功):" & To_String (W_D));
      Check (Ok_E and then Old_E and then To_String (G_E) = "hello world", "HTTP 分块编码 ⇒ 按块拼回 " & To_String (G_E) & "(旧写法把块长和 CRLF 一起当正文)");
      Check (not Ok_F and then Index (W_F, "连") > 0, "HTTP 没人听的口 ⇒ 照实报:" & To_String (W_F));
      Check (not Ok_G and then Index (W_G, "500") > 0, "HTTP 请求体放堆上的那一种:500 同样报失败、原因带出来:" & To_String (W_G));
   end;
   --  🔴 脑(09-30):请求里不再带 max_tokens(原来 80 / 700)、写程序那一问不再带 temperature(原来 0.7)—— 驱动替脑拍的数,交回服务端 / 模型自己的生成配置;
   --  认名字那一问仍是 temperature 0(贪心,要稳)。回包按 JSON 读;finish_reason = length(写到服务端上限被截断)照实说,不把半截话当程序;
   --  服务端回错(400 上下文装不下)时错误原文一字不落进 Err —— 执行器靠里面的 "maximum context length" 把清单减半(act.adb 问脑那一段)
   declare
      function Chat_Reply (Content, Finish : String) return String is
         B : constant String := "{""id"":""x"",""object"":""chat.completion"",""choices"":[{""index"":0,""message"":{""role"":""assistant"",""content"":"""
           & Json.Escape (Content) & """},""finish_reason"":""" & Finish & """}],""usage"":{""prompt_tokens"":10}}";
      begin
         return Http11 & " 200 OK" & CRLF & "Content-Type: application/json" & CRLF & "Content-Length: " & Codec.Img (B'Length) & CRLF & CRLF & B;
      end Chat_Reply;
      Too_Long : constant String := "{""object"":""error"",""message"":""This model's maximum context length is 8192 tokens. However, you requested 9000 tokens."","
        & """type"":""BadRequestError"",""code"":400}";
      R400 : constant String := Http11 & " 400 Bad Request" & CRLF & "Content-Length: " & Codec.Img (Too_Long'Length) & CRLF & CRLF & Too_Long;
      --  旧的两份请求头(原文):写程序那一问带 max_tokens 700、temperature 0.7,认名字那一问带 max_tokens 80
      Old_Ask : constant String := "{""model"":""eye"",""max_tokens"":700,""temperature"":0.7,";
      Old_Locate : constant String := "{""model"":""eye"",""max_tokens"":80,""temperature"":0,";
      Rgb : Buf;
      procedure Ask_Once (Reply : String; Ok : out Boolean; Prog, Err, Req : out Unbounded_String) is
         T : Fake_Http;
         Port : GNAT.Sockets.Port_Type;
      begin
         T.Start (Reply, 0.0, Port);
         Ok := Brain.Ask ("127.0.0.1", Natural (Port), "pick it", "a body", "nothing yet", "grammar", "", "", "", "", 3, 3, 0, 1, 1, Rgb, 10, 10, Prog, Err);
         T.Got (Req);
      end Ask_Once;
      Ok1, Ok2, Ok3, Ok4 : Boolean;
      P1, E1, Q1, P2, E2, Q2, P3, E3, Q3, E4, Q4 : Unbounded_String;
      Found : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
   begin
      for I in 1 .. 10 * 10 * 3 loop
         Rgb.Append (U8 (I mod 256));
      end loop;
      Ask_Once (Chat_Reply ("say hello" & ASCII.LF, "stop"), Ok1, P1, E1, Q1);
      Ask_Once (Chat_Reply ("say hel", "length"), Ok2, P2, E2, Q2);
      Ask_Once (R400, Ok3, P3, E3, Q3);
      declare
         T : Fake_Http;
         Port : GNAT.Sockets.Port_Type;
      begin
         T.Start (Chat_Reply ("{""found"":true,""bbox_2d"":[100,200,300,400]}", "stop"), 0.0, Port);
         Ok4 := Brain.Locate ("127.0.0.1", Natural (Port), "cup", Rgb, 10, 10, Found, X0, Y0, X1, Y1, E4);
         T.Got (Q4);
      end;
      Check (Ok1 and then To_String (P1) = "say hello" & ASCII.LF and then Index (Q1, "max_tokens") = 0 and then Index (Q1, "temperature") = 0
             and then Index (Q1, "structured_outputs") > 0
             and then Ada.Strings.Fixed.Index (Old_Ask, "max_tokens") > 0 and then Ada.Strings.Fixed.Index (Old_Ask, """temperature"":0.7") > 0,
             "脑写程序:请求里不带 max_tokens、不带 temperature(旧请求带着 700 和 0.7),程序原样读回");
      Check (Ok4 and then Found and then X0 = 1 and then Y0 = 2 and then X1 = 3 and then Y1 = 4
             and then Index (Q4, "max_tokens") = 0 and then Index (Q4, """temperature"":0,") > 0
             and then Ada.Strings.Fixed.Index (Old_Locate, "max_tokens") > 0,
             "脑认名字:请求里不带 max_tokens(旧请求带着 80)、仍是 temperature 0;框读回 (" & Codec.Img (X0) & "," & Codec.Img (Y0) & ")–(" & Codec.Img (X1) & "," & Codec.Img (Y1) & ")");
      Check (not Ok2 and then Index (E2, "截断") > 0, "脑的回答写到服务端上限被截断(finish_reason = length)⇒ 照实说,不当程序:" & To_String (E2));
      Check (not Ok3 and then Index (E3, "maximum context length") > 0 and then Index (E3, "400") > 0,
             "脑回 400 上下文装不下 ⇒ Err 里带着服务端原文(执行器靠 ""maximum context length"" 把清单上限往下压;"
             & "HTTP 那层改成照实报失败以后,要是这里还只说""连不上脑"",这句话就丢了):" & To_String (E3));
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
   --  🔴 msgpack 的有符号整数(09-30):原来 Integer_8 (Unsigned_8 (…)) 这类是按【值】转换,负数(高位是 1)当场抛 Constraint_Error;
   --  自检原来只测了 −7(负 fixint,不走这几行)。每一种有符号宽度的负数都过一遍:int8 / 16 / 32 / 64 的 −1 和最小值(字节手拼 —— Put_Int 会挑最短的写法)、
   --  Put_Int 写 −33 / −129 / −32769 / −2^31−1 / 最小值再读回;nd 数组 i1 / i2 / i4 / i8 的负读数(小端、大端);
   --  以前一律读成"没读数"的 f2 / u2 / u4 / u8 / b1 也认;装不进有符号 64 位的无符号数照大小读(原来悄悄截成最大的有符号数)
   declare
      use Interfaces;
      type U8_List is array (Positive range <>) of U8;
      function Of_List (L : U8_List) return Buf is
         B : Buf;
      begin
         for X of L loop
            B.Append (X);
         end loop;
         return B;
      end Of_List;
      Broken : constant Long_Float := -12345.0;   --  自检自己的记号:读坏了
      function One (L : U8_List) return Long_Float is
         D : Msgpack.Doc;
      begin
         return (if Msgpack.Decode (Of_List (L), D) then Msgpack.Num (D, 0) else Broken);
      exception
         when others => return Broken;
      end One;
      function Back (V : Long_Long_Integer; Tag : out U8) return Long_Long_Integer is
         S : Buf;
         D : Msgpack.Doc;
      begin
         Msgpack.Put_Int (S, V);
         Tag := S (0);
         return (if Msgpack.Decode (S, D) then D.Nodes (0).I else 0);
      exception
         when others => Tag := 0; return 0;
      end Back;
      function Nd (T : String; L : U8_List) return Floats is
         S : Buf;
         D : Msgpack.Doc;
         Data : constant Buf := Of_List (L);
      begin
         Msgpack.Put_Map (S, 4);
         Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
         Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, T);
         Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 1); Msgpack.Put_Int (S, 1);
         Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Data, 0, Natural (Data.Length));
         return (if Msgpack.Decode (S, D) then Msgpack.Numbers (D, 0) else F64_Vectors.Empty_Vector);
      exception
         when others => return F64_Vectors.Empty_Vector;
      end Nd;
      type F_List is array (Positive range <>) of Long_Float;
      function Same (V : Floats; W : F_List) return Boolean is
        (Natural (V.Length) = W'Length and then (for all I in W'Range => V (I - W'First) = W (I)));
      Min64 : constant Long_Float := Long_Float (Long_Long_Integer'First);
      T1, T2, T3, T4, T5 : U8 := 0;
      Scalars_Ok, Puts_Ok, Nd_Ok, New_Types_Ok, Rejects_Ok, Big_U_Ok : Boolean;
      Old_Crashes_8, Old_Crashes_Put : Boolean := False;
      H : constant Floats := Nd ("<f2", [16#00#, 16#3C#, 16#00#, 16#C0#, 16#FF#, 16#7B#, 16#01#, 16#00#, 16#00#, 16#7C#, 16#00#, 16#7E#]);
      function Old_Knows (T : String) return Boolean is (T (T'Last - 1 .. T'Last) in "f4" | "f8" | "i4" | "i8" | "u1");
   begin
      Scalars_Ok := One ([16#D0#, 16#FF#]) = -1.0 and then One ([16#D0#, 16#80#]) = -128.0
        and then One ([16#D1#, 16#FF#, 16#FF#]) = -1.0 and then One ([16#D1#, 16#80#, 0]) = -32768.0
        and then One ([16#D2#, 16#FF#, 16#FF#, 16#FF#, 16#FF#]) = -1.0 and then One ([16#D2#, 16#80#, 0, 0, 0]) = -2147483648.0
        and then One ([16#D3#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#]) = -1.0
        and then One ([16#D3#, 16#80#, 0, 0, 0, 0, 0, 0, 0]) = Min64;
      Puts_Ok := Back (-33, T1) = -33 and then T1 = 16#D0# and then Back (-129, T2) = -129 and then T2 = 16#D1#
        and then Back (-32769, T3) = -32769 and then T3 = 16#D2# and then Back (-2147483649, T4) = -2147483649 and then T4 = 16#D3#
        and then Back (Long_Long_Integer'First, T5) = Long_Long_Integer'First and then T5 = 16#D3#;
      Nd_Ok := Same (Nd ("|i1", [16#FF#, 16#80#, 5]), [-1.0, -128.0, 5.0])
        and then Same (Nd ("<i2", [16#FF#, 16#FF#, 16#00#, 16#80#, 16#2C#, 16#01#]), [-1.0, -32768.0, 300.0])
        and then Same (Nd (">i2", [16#FF#, 16#FE#]), [1 => -2.0])
        and then Same (Nd ("<i4", [16#FF#, 16#FF#, 16#FF#, 16#FF#, 0, 0, 0, 16#80#]), [-1.0, -2147483648.0])
        and then Same (Nd ("<i8", [16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 0, 0, 0, 0, 0, 0, 0, 16#80#]), [-1.0, Min64]);
      New_Types_Ok := Same (Nd ("<u2", [16#FF#, 16#FF#]), [1 => 65535.0]) and then Same (Nd ("<u4", [16#FF#, 16#FF#, 16#FF#, 16#FF#]), [1 => 4294967295.0])
        and then Same (Nd ("<u8", [16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#]), [1 => 18446744073709551615.0])
        and then Same (Nd ("|b1", [0, 1, 7]), [0.0, 1.0, 1.0])
        and then Natural (H.Length) = 6 and then H (0) = 1.0 and then H (1) = -2.0 and then H (2) = 65504.0 and then H (3) = 2.0 ** (-24)
        and then H (4) > Long_Float'Last and then H (5) /= H (5)
        and then not Old_Knows ("<i2") and then not Old_Knows ("<f2") and then not Old_Knows ("|b1");
      Rejects_Ok := Nd ("<i4", [1, 2, 3, 4, 5]).Is_Empty and then Nd ("<c8", [0, 0, 0, 0, 0, 0, 0, 0]).Is_Empty;
      Big_U_Ok := One ([16#CF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#, 16#FF#]) = 18446744073709551615.0;
      --  牙:旧写法按值转换,同一个字节 0xFF 当场抛异常
      declare
         pragma Warnings (Off, "*is not modified*");   --  易失 = 不让编译器把这一次转换提前算掉,要它在运行时真做一遍
         V : U8 := 16#FF# with Volatile;
         pragma Warnings (On, "*is not modified*");
         I8 : Integer_8;
      begin
         I8 := Integer_8 (V);
         Old_Crashes_8 := I8 = 0;
      exception
         when Constraint_Error => Old_Crashes_8 := True;
      end;
      declare
         pragma Warnings (Off, "*is not modified*");
         V : Long_Long_Integer := -2147483649 with Volatile;
         pragma Warnings (On, "*is not modified*");
         U : Unsigned_64;
      begin
         U := Unsigned_64 (Integer_64 (V));
         Old_Crashes_Put := U = 0;
      exception
         when Constraint_Error => Old_Crashes_Put := True;
      end;
      Check (Scalars_Ok and then Old_Crashes_8,
             "msgpack int8 / 16 / 32 / 64 的 −1 和最小值读得回来(旧写法 Integer_8 (Unsigned_8 (0xFF)) 按值转换,当场抛 Constraint_Error)");
      Check (Puts_Ok and then Old_Crashes_Put,
             "msgpack Put_Int −33 / −129 / −32769 / −2^31−1 / 最小值:类型字节挑对、读回一样(旧写法 Unsigned_64 (Integer_64 (−2^31−1)) 一写就抛异常)");
      Check (Nd_Ok, "nd 数组 i1 / i2(小端、大端)/ i4 / i8 的负读数读得回来(旧写法 i4 / i8 负数抛异常,i1 / i2 根本不认)");
      Check (New_Types_Ok and then Rejects_Ok,
             "nd 数组 u2 / u4 / u8 / b1 / f2(1、−2、65504、最小次正规、无穷、不是数)都认(旧写法只认 f4 / f8 / i4 / i8 / u1,别的静悄悄读成没读数);"
             & "长度不是元素宽度整数倍的、复数的 ⇒ 空");
      Check (Big_U_Ok, "msgpack uint64 2^64−1 照大小读成 1.8e19(旧写法截成 2^63−1,值就错了)");
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
   --  🔴 JSON 读得严、数写得准(09-30):true / false / null 要整个词 —— 原来看头一个字母就往后跳 4 / 5 个字符,身体文件里的 "nan," 被读成 null 还吃掉了逗号
   --  (读错一位不报);顶层的值后面还跟着东西 = 不是一份 JSON;Json.Number 写出去读回来一个比特不差(原来身体文件按定点小数印:
   --  1e-7 印成 0.000000 读回来是 0、NaN 印成 nan、2.5e20 印成 inf,后两个都不是 JSON)
   declare
      use Interfaces;
      function To_LF is new Ada.Unchecked_Conversion (Unsigned_64, Long_Float);
      function To_U is new Ada.Unchecked_Conversion (Long_Float, Unsigned_64);
      D : Json.Doc;
      E : Unbounded_String;
      Lit_Ok, Bad_Rejected, Tail_Ok : Boolean;
      Seed : Unsigned_64 := 16#9E37_79B9_7F4A_7C15#;
      Tried, Exact, Short_Exact : Natural := 0;
      NaN : constant Long_Float := To_LF (16#7FF8_0000_0000_0000#);
      Inf : constant Long_Float := To_LF (16#7FF0_0000_0000_0000#);
      Old_Src : constant String := "[1.0,nan,2.0]";
   begin
      Lit_Ok := Json.Parse ("[true,false,null]", D, E) and then Json.Count (D, 0) = 3 and then Json.Bool (D, Json.Child (D, 0, 0))
        and then not Json.Bool (D, Json.Child (D, 0, 1)) and then Json.Is_Null (D, Json.Child (D, 0, 2));
      Bad_Rejected := not Json.Parse (Old_Src, D, E) and then not Json.Parse ("{""a"":nan,""b"":2}", D, E) and then not Json.Parse ("[tru]", D, E)
        and then not Json.Parse ("[nul]", D, E) and then not Json.Parse ("[inf]", D, E) and then not Json.Parse ("[-inf]", D, E);
      Tail_Ok := not Json.Parse ("{} x", D, E) and then not Json.Parse ("[1][2]", D, E) and then Json.Parse ("{} " & ASCII.LF, D, E);
      for K in 1 .. 20_000 loop
         Seed := Seed xor Shift_Left (Seed, 13); Seed := Seed xor Shift_Right (Seed, 7); Seed := Seed xor Shift_Left (Seed, 17);
         declare
            X : constant Long_Float := To_LF (Seed);
            S : String (1 .. 40);
         begin
            if Json.Finite (X) then
               Tried := Tried + 1;
               if To_U (Long_Float'Value (Json.Number (X))) = To_U (X) then
                  Exact := Exact + 1;
               end if;
               Ada.Long_Float_Text_IO.Put (S, X, Aft => 15, Exp => 1);   --  牙:少印一位(16 位有效数字)
               if To_U (Long_Float'Value (S)) = To_U (X) then
                  Short_Exact := Short_Exact + 1;
               end if;
            end if;
         end;
      end loop;
      Check (Lit_Ok and then Bad_Rejected and then Tail_Ok and then Old_Src (6 .. 9) = "nan,",
             "JSON 只认整个 true / false / null:nan / tru / nul / inf 都读不成(旧写法见 n 就跳 4 个字,""" & Old_Src (6 .. 9)
             & """ 连逗号一起被当成 null 吃掉);顶层后面跟着东西 = 不是 JSON");
      Check (Tried > 0 and then Exact = Tried and then Short_Exact < Tried,
             "Json.Number:" & Codec.Img (Tried) & " 个随机双精度写出去读回来一个比特不差(少印一位只有 " & Codec.Img (Short_Exact) & " 个一样)");
      Check (Json.Number (NaN) = "null" and then Json.Number (Inf) = "null" and then Json.Number (-Inf) = "null"
             and then Long_Float'Value (Json.Number (1.0e-7)) = 1.0e-7 and then Codec.Fmt (1.0e-7, 6) = "0.000000"
             and then Long_Float'Value (Json.Number (2.5e20)) = 2.5e20 and then Codec.Fmt (2.5e20, 6) = "inf" and then Codec.Fmt (NaN, 6) = "nan"
             and then Json.Parse ("{""a"":null,""b"":1.5}", D, E)
             and then Json.Real (D, Json.Get (D, 0, "a")) /= Json.Real (D, Json.Get (D, 0, "a")) and then Json.Real (D, Json.Get (D, 0, "b")) = 1.5
             and then Json.Num (D, Json.Get (D, 0, "a")) = 0.0,
             "Json.Number:NaN / ±无穷写成 null、Real 读回来还是 NaN(Num 读成 0);1e-7、2.5e20 读回一样(旧写法 Fmt 印成 0.000000、inf,NaN 印成 nan)");
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
   --  🔴 看没看见动了 = 两次比较、不共用一帧(Picture.Seen_Twice;09-28 DR1:无人机的抓握通道什么都不带,头顶眼里渲染闪的像素被当成两瓣手指)。
   --  64×48、底 30、静止对地板 0:A1 / A2 = 动之前那头的两帧,B1 / B2 = 动之后那头的两帧
   declare
      W : constant := 64;
      H : constant := 48;
      Bg : Buf;
      Fl : Picture.Floor_Map;
      function With_Block (Img : Buf; X0, Y0, S : Natural; V : U8) return Buf is
         R : Buf := Img;
      begin
         for Y in Y0 .. Y0 + S - 1 loop
            for X in X0 .. X0 + S - 1 loop
               R.Replace_Element (Y * W + X, V);
            end loop;
         end loop;
         return R;
      end With_Block;
      Real, Old_Shared, New_Shared, Each, Real_And_Flick : Picture.Regions;
   begin
      for I in 1 .. W * H loop
         Bg.Append (30);
      end loop;
      Fl := Picture.Null_Floor (Bg, Bg, W, H, Picture.Min_Pixels (W, H));
      --  ① 真动的:动之后那头两帧同一处都多了一块 8×8
      Real := Picture.Seen_Twice (Bg, With_Block (Bg, 10, 10, 8, 200), Bg, With_Block (Bg, 10, 10, 8, 200), Fl, W, H);
      --  ② 只在一帧里闪一块 3×3:原来"推过去、推回来"共用这一帧,两次比较都算变了;不共用一帧就看不见
      declare
         Fk : constant Buf := With_Block (Bg, 40, 30, 3, 70);
      begin
         Old_Shared := Picture.Components (Picture.Both (Picture.Moved (Bg, Fk, Fl), Picture.Moved (Fk, Bg, Fl)), W, H, Picture.Min_Pixels (W, H));
         New_Shared := Picture.Seen_Twice (Bg, Fk, Bg, Bg, Fl, W, H);
      end;
      --  ③ 四帧各在各的地方闪
      Each := Picture.Seen_Twice (With_Block (Bg, 2, 2, 3, 70), With_Block (Bg, 20, 5, 3, 70),
                                  With_Block (Bg, 50, 40, 3, 70), With_Block (Bg, 30, 25, 3, 70), Fl, W, H);
      --  ④ 真动的 + 其中一帧另有一处闪:只认真动的那块
      Real_And_Flick := Picture.Seen_Twice (Bg, With_Block (With_Block (Bg, 10, 10, 8, 200), 40, 30, 3, 70), Bg, With_Block (Bg, 10, 10, 8, 200), Fl, W, H);
      Check (Natural (Real.Length) = 1 and then Real (0).Count = 64
             and then Natural (Old_Shared.Length) = 1 and then New_Shared.Is_Empty and then Each.Is_Empty
             and then Natural (Real_And_Flick.Length) = 1 and then Real_And_Flick (0).Count = 64,
             "看没看见动了(两次比较、不共用一帧):真动的 8×8 看见(" & Codec.Img (Natural (Real.Length)) & " 块)· 只在一帧里闪的 3×3:共用那一帧的老比法当成动了("
             & Codec.Img (Natural (Old_Shared.Length)) & " 块)、新比法 " & Codec.Img (Natural (New_Shared.Length)) & " 块 · 四帧各闪各的 "
             & Codec.Img (Natural (Each.Length)) & " 块 · 真动的 + 一帧闪:" & Codec.Img (Natural (Real_And_Flick.Length)) & " 块");
   end;
   --  🔴 两拨分不分得开(Picture.Split;09-30 审计 G4:原来"类间方差 ≥ 总方差一半"几乎从不拒 —— 单峰高斯 0.64、均匀 0.75、
   --  灰度噪声的半正态 0.67 全算"分得开";桌面上只占 1% 的白东西一眼分得开,反而 0.45 被拒)。新判法:直方图里有一道按置信界站得住的谷才算。
   --  每组 2 万个 8 位灰度级(Cut_Bright 一只眼抽样的量级),固定种子伪随机:单峰三种 ⇒ NaN;真两拨三种 ⇒ 分界落在两拨之间。
   --  老判法(64 格 Otsu + 一半,09-30 以前 picture.adb 原样)在这里同一组数重算一遍当牙
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      N_Each : constant := 20_000;
      function U01 return Long_Float is (Long_Float (FR.Random (Gen)));
      function Gauss return Long_Float is   --  标准正态(Box–Muller)
         U1 : constant Long_Float := Long_Float'Max (1.0e-12, 1.0 - U01);
         U2 : constant Long_Float := U01;
      begin
         return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
      end Gauss;
      function Level (X : Long_Float) return Long_Float is (Long_Float'Rounding (Long_Float'Max (0.0, Long_Float'Min (255.0, X))));
      procedure Old_Split (F : Floats; Split_Ok : out Boolean; T : out Long_Float) is
         Bins : constant := 64;
         Hh : array (0 .. Bins - 1) of Long_Float := [others => 0.0];
         Lo : Long_Float := Long_Float'Last;
         Hi : Long_Float := Long_Float'First;
         Total, Sum_All, W0, Sum0, Tot_Var : Long_Float := 0.0;
         Best_Var : Long_Float := -1.0;
      begin
         Split_Ok := False; T := 0.0;
         for X of F loop
            Lo := Long_Float'Min (Lo, X); Hi := Long_Float'Max (Hi, X);
         end loop;
         if not (Hi > Lo) then
            return;
         end if;
         for X of F loop
            declare
               B : constant Natural := Natural'Min (Bins - 1, Natural (Long_Float'Floor ((X - Lo) / (Hi - Lo) * Long_Float (Bins))));
            begin
               Hh (B) := Hh (B) + 1.0; Total := Total + 1.0; Sum_All := Sum_All + Long_Float (B);
            end;
         end loop;
         for B in 0 .. Bins - 2 loop
            W0 := W0 + Hh (B); Sum0 := Sum0 + Hh (B) * Long_Float (B);
            if W0 > 0.0 and then Total - W0 > 0.0 then
               declare
                  Var : constant Long_Float := W0 * (Total - W0) * (Sum0 / W0 - (Sum_All - Sum0) / (Total - W0)) ** 2;
               begin
                  if Var > Best_Var then
                     Best_Var := Var; T := Lo + (Long_Float (B) + 1.0) / Long_Float (Bins) * (Hi - Lo);
                  end if;
               end;
            end if;
         end loop;
         for B in 0 .. Bins - 1 loop
            Tot_Var := Tot_Var + Hh (B) * (Long_Float (B) - Sum_All / Total) ** 2;
         end loop;
         Split_Ok := Tot_Var > 0.0 and then Best_Var / Total >= 0.5 * Tot_Var;
      end Old_Split;
      Gauss_1, Flat_1, Noise_1, Table_White, Two_Tone, Noise_Fingers : Floats;
      T_Gauss, T_Flat, T_Noise, T_White, T_Two, T_Fing : Long_Float;
      O_Gauss, O_Flat, O_Noise, O_White, O_Two, O_Fing : Boolean;
      Ot : Long_Float;
   begin
      FR.Reset (Gen, 20260930);
      for I in 1 .. N_Each loop
         Gauss_1.Append (Level (128.0 + 10.0 * Gauss));                                           --  单峰:高斯 σ = 10 级
         Flat_1.Append (Level (60.0 + 140.0 * U01));                                              --  单峰:均匀 60..200
         Noise_1.Append (abs (Level (100.0 + 2.0 * Gauss) - Level (100.0 + 2.0 * Gauss)));        --  单峰:静止两帧的灰度差(半正态)
         Table_White.Append ((if I mod 100 = 0 then Level (240.0 + 4.0 * Gauss) else Level (135.0 + 12.0 * Gauss)));   --  桌面 + 1% 白东西
         Two_Tone.Append ((if I mod 2 = 0 then Level (100.0 + 10.0 * Gauss) else Level (140.0 + 10.0 * Gauss)));        --  两种一样多、隔 4σ
         Noise_Fingers.Append ((if I mod 100 = 0 then Level (72.0 + 16.0 * U01)
                                else abs (Level (100.0 + 2.0 * Gauss) - Level (100.0 + 2.0 * Gauss))));                --  噪声 + 1% 手指变化
      end loop;
      T_Gauss := Picture.Split (Gauss_1); T_Flat := Picture.Split (Flat_1); T_Noise := Picture.Split (Noise_1);
      T_White := Picture.Split (Table_White); T_Two := Picture.Split (Two_Tone); T_Fing := Picture.Split (Noise_Fingers);
      Old_Split (Gauss_1, O_Gauss, Ot); Old_Split (Flat_1, O_Flat, Ot); Old_Split (Noise_1, O_Noise, Ot);
      Old_Split (Table_White, O_White, Ot); Old_Split (Two_Tone, O_Two, Ot); Old_Split (Noise_Fingers, O_Fing, Ot);
      Check (Picture.Is_Nan (T_Gauss) and then Picture.Is_Nan (T_Flat) and then Picture.Is_Nan (T_Noise),
             "两拨:单峰三种(高斯、均匀、静止噪声的差)都分不开 ⇒ NaN(老判法说分得开:" & Boolean'Image (O_Gauss) & " /" & Boolean'Image (O_Flat)
             & " /" & Boolean'Image (O_Noise) & ",该 FALSE)");
      Check (O_Gauss and then O_Flat and then O_Noise, "两拨·牙:老判法(类间方差 ≥ 一半)同一组单峰全都当成分得开");
      Check (not Picture.Is_Nan (T_White) and then T_White > 180.0 and then T_White < 225.0
             and then not Picture.Is_Nan (T_Two) and then T_Two > 110.0 and then T_Two < 130.0
             and then not Picture.Is_Nan (T_Fing) and then T_Fing > 10.0 and then T_Fing < 72.0,
             "两拨:真两拨都分得开,分界在两拨之间 —— 桌面 135 + 1% 白 240:" & Codec.Fmt (T_White, 1) & " · 100 / 140 各一半:" & Codec.Fmt (T_Two, 1)
             & " · 静止噪声 + 1% 手指 72–88:" & Codec.Fmt (T_Fing, 1));
      Check (not O_White, "两拨·牙:老判法把桌面 + 1% 白东西判成分不开(类间方差不到一半)");
   end;
   --  🔴 中位绝对偏差为 0 时往上换分位,σ 的换算跟着分位走(Picture.Cut;09-30 审计 G4 / H6):|x − 中位| 的 q 分位 = σ·Φ⁻¹((1+q)/2)。
   --  原来每一档都乘 1.4826(只对 q = 0.5 对),q = 0.9 时 σ 放大 1.4826 × 1.645 = 2.44 倍。
   --  造一张按 1 mm 量化的深度图:桌面 0.80 m、噪声 σ = 0.35 mm(量化后 85% 的像素一点不差 ⇒ 中位绝对偏差 = 0、0.75 分位也是 0,落到 0.9 那一档 = 1 mm),
   --  中间一块近 3 mm 的东西(16×12,鼓出背景面 4 mm):新换算 σ = 1 mm / 1.645 ⇒ 门 = 中位 + 1.82 mm,切得出来;
   --  老换算 σ = 1.4826 mm ⇒ 门 = 中位 + 4.45 mm,比这块鼓出来的还高 ⇒ 切不出来(牙 = 同一张图、σ 倍数乘上老换算多出来的 1.4826 × Φ⁻¹(0.95))
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      W : constant := 96;
      H : constant := 72;
      Dep : Floats := Filled (W * H, 0.80);
      New_R, Old_R : Picture.Regions;
      function U01 return Long_Float is (Long_Float (FR.Random (Gen)));
      function Gauss return Long_Float is
         U1 : constant Long_Float := Long_Float'Max (1.0e-12, 1.0 - U01);
         U2 : constant Long_Float := U01;
      begin
         return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
      end Gauss;
      Old_Extra : constant Long_Float := 1.4826 * 1.6448536;   --  老换算在 q = 0.9 这一档多乘出来的倍数
      At_Block : Boolean := False;
   begin
      FR.Reset (Gen, 20260931);
      for I in 0 .. W * H - 1 loop
         Dep.Replace_Element (I, 0.80 + 0.001 * Long_Float'Rounding (0.35 * Gauss));
      end loop;
      for Y in 30 .. 41 loop
         for X in 40 .. 55 loop
            Dep.Replace_Element (Y * W + X, 0.797);
         end loop;
      end loop;
      New_R := Picture.Cut (Dep, W, H, 0.125, 3.0);
      Old_R := Picture.Cut (Dep, W, H, 0.125, 3.0 * Old_Extra);
      if Natural (New_R.Length) = 1 then
         At_Block := New_R (0).X0 >= 38 and then New_R (0).X1 <= 57 and then New_R (0).Y0 >= 28 and then New_R (0).Y1 <= 43;
      end if;
      Check (Natural (New_R.Length) = 1 and then At_Block,
             "量化深度(中位绝对偏差 = 0):σ 按 0.9 分位 ÷ Φ⁻¹(0.95) 换 ⇒ 那块近 3 mm 的东西切出" & Codec.Img (Natural (New_R.Length))
             & " 块(该 1 块、就在它那儿)");
      Check (Old_R.Is_Empty, "量化深度·牙:老换算(每档都乘 1.4826)同一张图切出" & Codec.Img (Natural (Old_R.Length)) & " 块(门比它鼓出来的还高 ⇒ 该 0)");
   end;
   --  🔴 伸长比:像素当单位方块(二阶矩各加 1/12;Picture.Components / Cut_Colour / Region_Of_Mask 同一份),一像素宽、20 长的线 = 20,两像素宽的 = 10(长宽比)。
   --  原来按点算:一像素宽的短轴为零 ⇒ 哨兵 1000;两像素宽的 √((20² − 1)/12 ÷ 0.25) = 11.5 —— 只宽一个像素就差 87 倍,这个数进了 act.adb 的"像不像"
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      W : constant := 40;
      H : constant := 20;
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      Rs : Picture.Regions;
      E1, E2 : Long_Float := 0.0;
      function Old_Elong (Len, Wd : Natural) return Long_Float is   --  老算法:按点的矩,短轴为零记 1000
        (if Long_Float (Wd * Wd - 1) / 12.0 > 1.0e-9 then Sqrt ((Long_Float (Len * Len - 1) / 12.0) / (Long_Float (Wd * Wd - 1) / 12.0)) else 1.0e3);
   begin
      for X in 5 .. 24 loop
         M.Replace_Element (3 * W + X, True);
         M.Replace_Element (10 * W + X, True);
         M.Replace_Element (11 * W + X, True);
      end loop;
      Rs := Picture.Components (M, W, H, 4);
      for R of Rs loop
         if R.Count = 20 then
            E1 := R.Elong;
         elsif R.Count = 40 then
            E2 := R.Elong;
         end if;
      end loop;
      Check (abs (E1 - 20.0) < 1.0e-9 and then abs (E2 - 10.0) < 1.0e-9,
             "伸长比:一像素宽 20 长的线 " & Codec.Fmt (E1, 3) & "(该 20)· 两像素宽 " & Codec.Fmt (E2, 3) & "(该 10)");
      Check (Old_Elong (20, 1) / Old_Elong (20, 2) > 80.0,
             "伸长比·牙:老算法一像素宽 " & Codec.Fmt (Old_Elong (20, 1), 1) & "、两像素宽 " & Codec.Fmt (Old_Elong (20, 2), 2) & "(真长宽比只差 2 倍)");
   end;
   --  🔴 扫描时"到了"按每个关节各自的门(Selfmap.Joints_Arrived + Kinem.Clean_Tol;09-28 H1:人形别的关节还偏 0.001–0.009 rad 就读了格子,
   --  每根轴单独起步只收偏不到 Clean_Tol 的格子,两只手运动学都没量成)。按 H1 量到的收法造:扫的那根一拍就到目标(0.221),
   --  上一段那根从 +0.304 rad 回起点(−0.4),每拍剩 0.64;这一格一步 0.0295 ⇒ 老门 = 三分之一格 ≈ 0.0098,新门:别的关节按 Clean_Tol(640 宽 ≈ 0.00084)
   declare
      Tgt : Bytes.Floats;
      Now : Bytes.Floats;
      Tol : constant Long_Float := 0.0295 / 3.0;   --  这一格一步 0.0295 的三分之一(合成数,同 H1 那一格)
      Ct : constant Long_Float := Kinem.Clean_Tol (640.0);
      Tols : Bytes.Floats;
      Old_Beat, New_Beat : Natural := 0;
      Old_Off, New_Off : Long_Float := 0.0;
      A_Old, A_New : Natural := 0;
   begin
      Tgt.Append (-0.4); Tgt.Append (0.221); Tgt.Append (0.1);
      Tols.Append (Ct); Tols.Append (Tol); Tols.Append (Ct);
      for K in 1 .. 30 loop
         declare
            E0 : constant Long_Float := 0.304 * 0.64 ** K;
         begin
            Now.Clear; Now.Append (-0.4 + E0); Now.Append (0.221); Now.Append (0.1);
            A_Old := (if Selfmap.Joints_Arrived (Now, Tgt, Bytes.F64_Vectors.Empty_Vector, Tol) then A_Old + 1 else 0);
            A_New := (if Selfmap.Joints_Arrived (Now, Tgt, Tols, Tol) then A_New + 1 else 0);
            if A_Old = 2 and then Old_Beat = 0 then
               Old_Beat := K; Old_Off := E0;
            end if;
            if A_New = 2 and then New_Beat = 0 then
               New_Beat := K; New_Off := E0;
            end if;
         end;
      end loop;
      Check (Old_Beat = 9 and then Old_Off > Ct and then New_Beat = 15 and then New_Off < Ct,
             "扫描按各关节自己的门算到:老门(三分之一格 " & Codec.Fmt (Tol, 4) & ")第 " & Codec.Img (Old_Beat) & " 拍就读,那一刻别的关节还偏 "
             & Codec.Fmt (Old_Off, 4) & "(> 收格子的门 " & Codec.Fmt (Ct, 5) & ",格子不干净,同 H1)· 新门第 " & Codec.Img (New_Beat)
             & " 拍读,偏 " & Codec.Fmt (New_Off, 5) & "(在门里)");
   end;
   --  🔴 配进一只手的腕眼、落在它长在眼上的那一格的不要(Kinem.On_Eye_Grid;09-28 H4 / H5 对齐:第二只手的桌面点配到第一只手自己那只白手上 69 / 29 对,
   --  按真值全错、差 300–450 px)。640 × 480、32 × 24 格(同扫描,20 px 一格),长在眼上的格点 (10,10)、(30,10):落在它们那一格里(连格子边上)算,
   --  邻格、画面外不算
   declare
      Eye : Kinem.Px_Vectors.Vector;
   begin
      Eye.Append (Kinem.Px'(U => (0.0 + 0.5) * 640.0 / 32.0, V => (0.0 + 0.5) * 480.0 / 24.0));
      Eye.Append (Kinem.Px'(U => (1.0 + 0.5) * 640.0 / 32.0, V => (0.0 + 0.5) * 480.0 / 24.0));
      Check (Kinem.On_Eye_Grid (Eye, 12.5, 17.0, 640, 480) and then Kinem.On_Eye_Grid (Eye, 19.99, 19.99, 640, 480)
             and then Kinem.On_Eye_Grid (Eye, 20.0, 5.0, 640, 480) and then not Kinem.On_Eye_Grid (Eye, 45.0, 10.0, 640, 480)
             and then not Kinem.On_Eye_Grid (Eye, 10.0, 25.0, 640, 480) and then not Kinem.On_Eye_Grid (Eye, -1.0, 5.0, 640, 480),
             "落在自己手上的那一格:格点 (10,10)、(30,10) 那两格里的点(连 19.99 / 20.0 格子边)算,邻格 (50,10)、(10,30) 和画面外不算");
   end;
   --  🔴 眼转了一下,画面里哪些点长在眼上(Kinem.Fit_Eye_Turn + Classify_Rides;09-30 V1B69:按灰度判时手一转光照就变,第 1 只手判不出哪头张开)。
   --  合成的眼:焦距 400、640 × 480,问那张格点(Kinem.Grid_U / Grid_V)。世界 = 离眼 2–3.5 单位的斜面,眼绕它后面 0.02 单位的一点转 0.161 弧度
   --  (轴斜着:x5 那一下绕世界竖直轴,在腕眼里是斜的;驱动发的位姿就是眼的位姿,这里再让转的中心偏一点,世界点带一点视差)。世界点配点噪声 0.3 px、二十个里一个是乱配(±30 px);
   --  长在眼上的点 = 画面下方两块"手指"(同 x5 腕眼:左 0–110、右 520–640、下 260–480)里的格点,配点照 V1B69 实测那样糟:没有纹理,
   --  散在 12 px 半径的圆里,五个里一个被周围的世界拖过去两成(实测手指格点挪了中位 2.2 px、九成 11.7 px,世界挪 72 px)。
   --  要:长在眼上的判成 Rides、世界判成 World(各错不到 2%);眼没转(配到的地方 = 问的点 + 0.05 px 噪声)⇒ 全是 Unknown;
   --  长在眼上的占六成(手指铺满下面六成画面)⇒ 全是 Unknown(照实判不了,不猜)。
   --  🦷 同一批配点按开机认手指那一条(挪不到 Geom.Trip_Px = 1 px 算没挪)判:手指格点只有少数"没挪"(红)
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      use type Kinem.Ride;
      Fr : constant Long_Float := 400.0;
      Cx0 : constant Long_Float := 320.0;
      Cy0 : constant Long_Float := 240.0;
      Wd : constant Positive := 640;
      Ht : constant Positive := 480;
      N_G : constant Natural := Kinem.Gx * Kinem.Gy;
      Th : constant Long_Float := 0.161;
      Ax : constant Geom.V3 := [0.2 / Sqrt (1.13), 1.0 / Sqrt (1.13), 0.3 / Sqrt (1.13)];
      Rr : constant Geom.M3 := Geom.Rodrigues ([Th * Ax (0), Th * Ax (1), Th * Ax (2)]);
      Piv : constant Geom.V3 := [0.0, 0.0, 0.02];   --  转的中心在眼后面 0.02 单位(相机系 -z 朝前 ⇒ +z 是后面;景物的 1%,一点视差)
      Eye : Geom.Cam_Geo := Geom.No_Geo;
      type Lcg is mod 2 ** 31;
      Seed : Lcg := 12345;
      function Rnd return Long_Float is   --  0..1 的伪随机(线性同余,固定种子 ⇒ 每次一样)
      begin
         Seed := Seed * 1103515245 + 12345;
         return Long_Float (Seed) / Long_Float (Lcg'Modulus);
      end Rnd;
      function Finger (U, V : Long_Float; Wide : Boolean) return Boolean is
        (V >= (if Wide then 190.0 else 260.0) and then (U <= (if Wide then 640.0 else 110.0) or else U >= 520.0));
      Out_W : Bools;   --  世界点里转出去以后出了画幅的(Make 记下)
      procedure Make (Turned, Wide : Boolean; Pu, Pv, Bu, Bv : out Kinem.Vec; Is_F : out Bools) is
      begin
         Is_F.Clear;
         Out_W.Clear;
         for Gyy in 0 .. Kinem.Gy - 1 loop
            for Gxx in 0 .. Kinem.Gx - 1 loop
               declare
                  K : constant Natural := Gyy * Kinem.Gx + Gxx;
                  U : constant Long_Float := Kinem.Grid_U (Gxx, Wd);
                  V : constant Long_Float := Kinem.Grid_V (Gyy, Ht);
                  F_Here : constant Boolean := Finger (U, V, Wide);
                  --  这个像素看出去的世界点:视线 (x, y, -1)(相机系,+y 朝上 ⇒ 像素 v 往下是 -y),深度按斜面
                  Dz : constant Long_Float := 2.0 + 1.5 * V / Long_Float (Ht);
                  Xc : constant Geom.V3 := [(U - Cx0) / Fr * Dz, -(V - Cy0) / Fr * Dz, -Dz];
                  --  眼绕 Piv 转 Rr ⇒ 世界点在新的眼系里 = Rrᵀ (X − Piv) + Piv
                  Rel : constant Geom.V3 := Geom.Ap (Geom.Tr (Rr), [Xc (0) - Piv (0), Xc (1) - Piv (1), Xc (2) - Piv (2)]);
                  Xn : constant Geom.V3 := [Rel (0) + Piv (0), Rel (1) + Piv (1), Rel (2) + Piv (2)];
                  Uw : constant Long_Float := Cx0 + Fr * Xn (0) / (-Xn (2));
                  Vw : constant Long_Float := Cy0 - Fr * Xn (1) / (-Xn (2));
                  A1 : constant Long_Float := Rnd;
                  A2 : constant Long_Float := Rnd;
                  A3 : constant Long_Float := Rnd;
               begin
                  Pu (K) := U; Pv (K) := V;
                  Is_F.Append (F_Here);
                  Out_W.Append (Turned and then not F_Here and then (Uw < 0.0 or else Vw < 0.0 or else Uw >= Long_Float (Wd) or else Vw >= Long_Float (Ht)));
                  if not Turned then
                     Bu (K) := U + 0.05 * (A1 - 0.5); Bv (K) := V + 0.05 * (A2 - 0.5);
                  elsif F_Here then
                     declare
                        Rad : constant Long_Float := 12.0 * Sqrt (A1);
                        Ang : constant Long_Float := 2.0 * Ada.Numerics.Pi * A2;
                        Drag : constant Long_Float := (if A3 < 0.2 then 0.2 else 0.0);
                     begin
                        Bu (K) := U + Rad * Cos (Ang) + Drag * (Uw - U);
                        Bv (K) := V + Rad * Sin (Ang) + Drag * (Vw - V);
                     end;
                  elsif Out_W.Last_Element then
                     --  转出去以后出了画幅:转出去那一帧里没有它的真对应,配点仪器交回原地附近一个编的(V1B73 实测:右边那一条几乎不挪)
                     Bu (K) := U + 2.0 * (A1 - 0.5); Bv (K) := V + 2.0 * (A2 - 0.5);
                  elsif A3 < 0.05 then
                     Bu (K) := Uw + 60.0 * (A1 - 0.5); Bv (K) := Vw + 60.0 * (A2 - 0.5);
                  else
                     Bu (K) := Uw + 0.6 * (A1 - 0.5) * 1.7320508; Bv (K) := Vw + 0.6 * (A2 - 0.5) * 1.7320508;   --  均匀分布 ±0.3·√3 ⇒ 标准差 0.3 px
                  end if;
               end;
            end loop;
         end loop;
      end Make;
      Pu, Pv, Bu, Bv : Kinem.Vec (0 .. N_G - 1);
      Rd : Kinem.Ride_Vec (0 .. N_G - 1);
      Is_F : Bools;
      Sig : Long_Float;
      Settled : Boolean;
      N_F, N_W, F_Ok, W_Ok, F_Gate, Unk_Idle, Unk_Wide : Natural := 0;
      F_Bad, W_Bad, N_Out, Out_Bad, Out_Old : Natural := 0;
      --  驱动的两步:按这一批点拟合眼转了多少,再拿它判同一批点
      procedure Rides_On_Eye (G : Geom.Cam_Geo; Pu, Pv, Bu, Bv : Kinem.Vec; R : out Kinem.Ride_Vec; Sig_Px : out Long_Float; Settled : out Boolean) is
         Rot : Geom.V3;
         Fitted : Boolean;
      begin
         Kinem.Fit_Eye_Turn (G, Wd, Ht, Pu, Pv, Bu, Bv, Rot, Sig_Px, Settled, Fitted);
         if Fitted then
            Kinem.Classify_Rides (G, Rot, Sig_Px, Wd, Ht, Pu, Pv, Bu, Bv, R);
         else
            R := [others => Kinem.Unknown];
         end if;
      end Rides_On_Eye;
   begin
      Eye.F := Fr; Eye.Cx := Cx0; Eye.Cy := Cy0; Eye.Valid := True;   --  这只眼的焦距、主点(针孔,没畸变)
      Make (True, False, Pu, Pv, Bu, Bv, Is_F);
      Rides_On_Eye (Eye, Pu, Pv, Bu, Bv, Rd, Sig, Settled);
      for K in 0 .. N_G - 1 loop
         if Is_F (K) then
            N_F := N_F + 1;
            if Rd (K) = Kinem.Rides then
               F_Ok := F_Ok + 1;
            elsif Rd (K) = Kinem.World then
               F_Bad := F_Bad + 1;
            end if;
            if Sqrt ((Bu (K) - Pu (K)) ** 2 + (Bv (K) - Pv (K)) ** 2) < Geom.Trip_Px then
               F_Gate := F_Gate + 1;
            end if;
         else
            N_W := N_W + 1;
            if Rd (K) = Kinem.World then
               W_Ok := W_Ok + 1;
            elsif Rd (K) = Kinem.Rides then
               W_Bad := W_Bad + 1;
            end if;
            if Out_W (K) then
               N_Out := N_Out + 1;
               --  🦷 原来(不管出没出画幅)按近的判:配到的地方离原处近 ⇒ 判成长在眼上
               if Rd (K) = Kinem.Rides then
                  Out_Bad := Out_Bad + 1;
               end if;
            end if;
         end if;
      end loop;
      Make (False, False, Pu, Pv, Bu, Bv, Is_F);
      Rides_On_Eye (Eye, Pu, Pv, Bu, Bv, Rd, Sig, Settled);
      for K in 0 .. N_G - 1 loop
         if Rd (K) = Kinem.Unknown then
            Unk_Idle := Unk_Idle + 1;
         end if;
      end loop;
      declare
         Sig_Turn : Long_Float;
         N_Wide : Natural := 0;
      begin
         Make (True, False, Pu, Pv, Bu, Bv, Is_F);
         Rides_On_Eye (Eye, Pu, Pv, Bu, Bv, Rd, Sig_Turn, Settled);
         Make (True, True, Pu, Pv, Bu, Bv, Is_F);
         for K in 0 .. N_G - 1 loop
            if Is_F (K) then
               N_Wide := N_Wide + 1;
            end if;
         end loop;
         Rides_On_Eye (Eye, Pu, Pv, Bu, Bv, Rd, Sig, Settled);
         for K in 0 .. N_G - 1 loop
            if Rd (K) = Kinem.Unknown then
               Unk_Wide := Unk_Wide + 1;
            end if;
         end loop;
         --  🦷 同一批配点按原来的判法(出了画幅的也按近的判):出了画幅的世界点配回原处附近 ⇒ 判成长在眼上
         for K in 0 .. N_G - 1 loop
            if Out_W (K) and then Sqrt ((Bu (K) - Pu (K)) ** 2 + (Bv (K) - Pv (K)) ** 2) < 2.0 then
               Out_Old := Out_Old + 1;
            end if;
         end loop;
         Check (N_F > 0 and then N_W > 0 and then 50 * F_Bad < N_F and then 50 * W_Bad < N_W and then 2 * F_Ok >= N_F and then 2 * W_Ok >= N_W
                and then N_Out > 0 and then Out_Bad = 0,
                "眼转了一下哪些点长在眼上:手指格点判成长在眼上 " & Codec.Img (F_Ok) & " / " & Codec.Img (N_F) & "(判成世界 " & Codec.Img (F_Bad) & ")、世界格点判成世界 "
                & Codec.Img (W_Ok) & " / " & Codec.Img (N_W) & "(判成长在眼上 " & Codec.Img (W_Bad) & ";别的是分不开)· 转出画幅的世界点 " & Codec.Img (N_Out)
                & " 个一个都没判成长在眼上(量到的配点噪声 " & Codec.Fmt (Sig_Turn, 2) & " px)");
         Check (Out_Old > 0, "🦷 转出画幅的世界点按原来的判法(近的那个):" & Codec.Img (Out_Old) & " / " & Codec.Img (N_Out) & " 个会判成长在眼上(V1B73 右边那一条就是这样连进瓣里的)");
         Check (Unk_Idle = N_G, "眼没转:全部格点两种说法分不开(" & Codec.Img (Unk_Idle) & " / " & Codec.Img (N_G) & ")");
         Check (10 * N_Wide >= 6 * N_G and then Unk_Wide = N_G,
                "长在眼上的占 " & Codec.Img (N_Wide) & " / " & Codec.Img (N_G) & "(过半):转动拟合成没转 ⇒ 全部分不开(" & Codec.Img (Unk_Wide) & ",照实判不了)");
         Check (2 * F_Gate < N_F, "🦷 同一批配点按开机认手指的 1 px 门:手指格点只有 " & Codec.Img (F_Gate) & " / " & Codec.Img (N_F) & " 算没挪(旧量法漏掉大半)");
      end;
   end;
   --  🔴 接触集重写(09-29):托住它要多大的摩擦、每单位重量最少要夹多紧(Contact.Hold)—— 能手算的几条:
   --  ① 两处正对的点接触夹在重心两侧,抬 = 托住单位重量:法向力之和 = 1/μ(每边 1/(2μ));不靠摩擦做不到、靠一点摩擦就做得到(要的摩擦 → 0);
   --  ② 重心偏出夹持线 0.05、指肚能拧(半径 0.01):竖着的摩擦 1/μ + 拧住 0.05/(μ·0.01) = 6/μ;点接触(不能拧)⇒ 托不住;
   --  ③ 两个面各歪 0.3 rad(同向):要的摩擦 = tan 0.3;④ 线性规划本身:min x1 + x2、x1 + 2 x2 = 4 ⇒ 2;x1 = -1 ⇒ 做不到
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      package Hd renames Contact.Hold;
      Ts : Hd.Touch_Vectors.Vector;
      L : constant Hd.Load := (F => [0.0, 0.0, 1.0], C => [0.0, 0.0, 0.0], M => [0.0, 0.0, 0.0]);
      L_Off : constant Hd.Load := (F => [0.0, 0.0, 1.0], C => [0.0, 0.05, 0.0], M => [0.0, 0.0, 0.0]);
      S1, M1, S2, S2p, M3 : Long_Float;
      Al : constant Long_Float := 0.3;
      Obj1, Obj2 : Long_Float;
      Ok1, Ok2 : Boolean;
   begin
      Ts.Append (Hd.Touch'(P => [-0.02, 0.0, 0.0], N => [1.0, 0.0, 0.0], Twist_R => 0.0));
      Ts.Append (Hd.Touch'(P => [0.02, 0.0, 0.0], N => [-1.0, 0.0, 0.0], Twist_R => 0.0));
      S1 := Hd.Squeeze (Ts, L, 0.5);
      M1 := Hd.Mu_Need (Ts, L);
      S2p := Hd.Squeeze (Ts, L_Off, 0.5);
      for I in 0 .. 1 loop
         declare
            T : Hd.Touch := Ts (I);
         begin
            T.Twist_R := 0.01;
            Ts.Replace_Element (I, T);
         end;
      end loop;
      S2 := Hd.Squeeze (Ts, L_Off, 0.5);
      Ts.Clear;
      Ts.Append (Hd.Touch'(P => [-0.02, 0.0, 0.0], N => [Cos (Al), Sin (Al), 0.0], Twist_R => 0.0));
      Ts.Append (Hd.Touch'(P => [0.02, 0.0, 0.0], N => [-Cos (Al), Sin (Al), 0.0], Twist_R => 0.0));
      M3 := Hd.Mu_Need (Ts, L);
      Hd.Min_Sum ([1.0, 2.0], 1, 2, [4.0], Obj1, Ok1);
      Hd.Min_Sum ([1.0, 0.0], 1, 2, [-1.0], Obj2, Ok2);
      Check (abs (S1 - 2.0) < 1.0e-6 and then M1 < 1.0e-6 and then abs (S2 - 12.0) < 1.0e-4 and then S2p = Hd.No_Way
             and then abs (M3 - Tan (Al)) < 1.0e-3 * Tan (Al) and then Ok1 and then abs (Obj1 - 2.0) < 1.0e-9 and then not Ok2,
             "接触集·托住要多紧:正对夹在重心两侧 μ=0.5 ⇒ 法向力之和 " & Codec.Fmt (S1, 4) & "(要 2)、要的摩擦 " & Codec.Fmt (M1, 7)
             & " · 重心偏 0.05、指肚能拧 0.01 ⇒ " & Codec.Fmt (S2, 4) & "(要 12)、点接触 ⇒ " & (if S2p = Hd.No_Way then "托不住" else Codec.Fmt (S2p, 4))
             & " · 两面各歪 0.3 rad ⇒ 要的摩擦 " & Codec.Fmt (M3, 4) & "(tan 0.3 = " & Codec.Fmt (Tan (Al), 4) & ")· 线性规划 " & Codec.Fmt (Obj1, 4)
             & " / 做不到的那一条 " & (if Ok2 then "说做得到(错)" else "说做不到"));
   end;
   --  🔴 接触集重写(09-29):几何上让量出来的手真合一次挑下手处(Contact.Grasp)。手 = x5 这种两块相向合:两个尖在眼前 9 cm、相距 9 cm,
   --  手指沿合拢方向厚 1 cm(碰东西的两面相距 8 cm),指肚宽 1.5 cm,手落位的误差 2 mm;
   --  东西都平躺在桌上(z = 0,上 = +z),表面点 2 mm 一个(顶面 + 往下补到桌面)。
   --  ① 平条(沿 x 宽 2 cm、沿 y 长 20 cm、厚 1 cm):两个接触点落在条的两条长边上(x = ±1 cm)、法向 ±x、从上面进(竖着或斜着都行,由那个数定)、
   --     候选里交出去的手的朝向合出来的方向 = 两个接触点的连线(PLAN 的"故意转 90° ⇒ 红":离线把候选的朝向绕工具轴转 90° 存,这一条红)、
   --     两个面正对 ⇒ 要的摩擦 < 0.1、夹在重心附近(< 半个指肚宽);前 5 名里没有一个接触点落在条面中间(顺着长边夹 = 两块落在条上,一个都不许有);
   --  ② 一根 1 cm 宽的刀刃 + 一头一个把手圈(外径 4 cm、内径 2.5 cm,厚 4 mm):前 3 名打出来看,第 1 名两个接触点在料的两侧、相距 < 张口;
   --  ③ 12 cm 见方的板(张口 8 cm):只有斜着夹一个角那几把(两条边各歪 45° ⇒ 要的摩擦约 1、离中心 > 4 cm),这一批里最不要摩擦的就是它们,照实交出去;
   --     每处接触的法向 = 它所在那面墙朝里的法向(指肚的边擦在斜墙上:力沿墙的法向,不沿指肚);
   --  ④ 直径 4 cm 的圆柱:两点正对、要的摩擦很小;⑤ 全都够不着 ⇒ 一个都不给,账上记"够不着";
   --  ⑥ 边长 6 cm 的正三角形(高 5.2 cm):第 1 名一块压顶点(尖顶在指肚面上 ⇒ 法向 = 那一块合拢的方向)、一块压底面(法向 +y);
   --     交出去的所有候选里压在三个尖上的接触,法向都 = 那一块合拢的方向;
   --  ⑦ 边长 1 cm 的正六棱柱(每条边比 1.5 cm 的指肚窄):第 1 名夹在两条对边上(x = ±0.8 cm)、法向 ±x、要的摩擦 < 0.1
   --     (最先碰到的那一点落在对边的一头时,旁边那条斜边往后退,不许把它的斜率当成接触法向)。
   --  牙(09-29 离线各拆一处跑过,每一处都有焊点变红):法向一律按指肚的朝向 / 不看指肚边外面 ⇒ ③⑥ 红;斜率反号 ⇒ ③ 红;按格取最近点 ⇒ ③⑥ 红;
   --  旧的"一边在往后退、另一边平或没点就按退的那边的斜率" ⇒ ⑦ 红;尖按陡的那一边的斜率 ⇒ ⑥ 红;候选的朝向绕工具轴转 90° 存 ⇒ ① 红;
   --  不看旁边的东西 / 手指厚当 0 ⇒ ⑧ 厚手指那半红;不先合、张到头下去 ⇒ ⑧ 薄手指那半红;
   --  ⑧ 两件挨着放:平条 + 右边 1 cm 外一个方块(x 2–5 cm、y ±3 cm、高 3 cm,当"旁边的东西"给):手指厚 1 cm 塞不进那道 1 cm 的缝 ⇒
   --     第 1 名的两处接触都在方块的 y 范围外(再让半个指肚宽),账上有"旁边的东西挡着";反面对照:手指厚 4 mm 塞得进 ⇒ 第 1 名夹在条的正中(离重心 < 半个指肚宽)
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      package Cg2 renames Contact.Grasp;
      Hm : constant Cg2.Hand_Model := Cg2.Two_Pads ([-0.045, 0.0, -0.09], [0.045, 0.0, -0.09], 0.015, 0.01, 0.002);
      Hm_Thin : constant Cg2.Hand_Model := Cg2.Two_Pads ([-0.042, 0.0, -0.09], [0.042, 0.0, -0.09], 0.015, 0.004, 0.002);
      None : Contact.V3_Vectors.Vector;
      function Always (R : Geom.M3; T : Contact.V3) return Boolean is (True);
      function Never (R : Geom.M3; T : Contact.V3) return Boolean is (False);
      Pitch : constant Long_Float := 0.002;
      --  平躺的东西的表面点:顶面 z = Thick 上按 In_Shape 取点(2 mm 一格);轮廓边上的点(上下左右有一个不在形状里)往下每 2 mm 补一层侧壁到桌面
      function Slab (X0, X1, Y0, Y1, Thick : Long_Float; In_Shape : access function (X, Y : Long_Float) return Boolean) return Contact.V3_Vectors.Vector is
         V : Contact.V3_Vectors.Vector;
         Nx : constant Natural := Natural ((X1 - X0) / Pitch);
         Ny : constant Natural := Natural ((Y1 - Y0) / Pitch);
         Nz : constant Natural := Natural (Thick / Pitch);
      begin
         for I in 0 .. Nx loop
            for J in 0 .. Ny loop
               declare
                  X : constant Long_Float := X0 + Pitch * Long_Float (I);
                  Y : constant Long_Float := Y0 + Pitch * Long_Float (J);
               begin
                  if In_Shape (X, Y) then
                     V.Append (Contact.V3'([X, Y, Thick]));
                     if not In_Shape (X - Pitch, Y) or else not In_Shape (X + Pitch, Y) or else not In_Shape (X, Y - Pitch) or else not In_Shape (X, Y + Pitch) then
                        for K in 0 .. Nz - 1 loop
                           V.Append (Contact.V3'([X, Y, Pitch * Long_Float (K)]));
                        end loop;
                     end if;
                  end if;
               end;
            end loop;
         end loop;
         return V;
      end Slab;
      function Bar (X, Y : Long_Float) return Boolean is (abs X <= 0.01 and then abs Y <= 0.1);
      function Blade (X, Y : Long_Float) return Boolean is
        ((abs X <= 0.005 and then Y >= -0.06 and then Y <= 0.06) or else (X * X + (Y - 0.08) ** 2 <= 0.02 ** 2 and then X * X + (Y - 0.08) ** 2 >= 0.0125 ** 2));
      function Plate (X, Y : Long_Float) return Boolean is (abs X <= 0.06 and then abs Y <= 0.06);
      function Disc (X, Y : Long_Float) return Boolean is (X * X + Y * Y <= 0.02 ** 2);
      --  边长 6 cm 的正三角形(高 3√3 cm):顶点朝 +y、在 (0, 2√3 cm),底边在 y = −√3 cm,形心在原点
      Tri_Top : constant Long_Float := 0.02 * Sqrt (3.0);
      Tri_Base : constant Long_Float := -0.01 * Sqrt (3.0);
      function Tri (X, Y : Long_Float) return Boolean is (Y >= Tri_Base and then Y <= Tri_Top - Sqrt (3.0) * abs X);
      function Hex (X, Y : Long_Float) return Boolean is (abs X <= 0.005 * Sqrt (3.0) and then abs Y <= 0.01 - abs X / Sqrt (3.0));   --  边长 1 cm、两条对边竖着
      function Sg (X : Long_Float) return Long_Float is (if X < 0.0 then -1.0 else 1.0);
      Fd : Cg2.Cand_Vectors.Vector;
      Stt : Cg2.Plan_Stats;
   begin
      --  ①
      Cg2.Plan (Slab (-0.012, 0.012, -0.102, 0.102, 0.01, Bar'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
      declare
         Ok : Boolean := not Fd.Is_Empty;
      begin
         if Ok then
            declare
               C0 : constant Cg2.Candidate := Fd (0);
               P0 : constant Contact.V3 := C0.Touches (0).P;
               P1 : constant Contact.V3 := C0.Touches (1).P;
               Ok_U : Boolean;
            begin
               Ok := Natural (C0.Touches.Length) = 2 and then abs (abs P0 (0) - 0.01) <= 1.5 * Pitch and then abs (abs P1 (0) - 0.01) <= 1.5 * Pitch
                 and then P0 (0) * P1 (0) < 0.0 and then -C0.Touches (0).N (0) * Sg (P0 (0)) > 0.95 and then -C0.Touches (1).N (0) * Sg (P1 (0)) > 0.95
                 and then C0.Approach (2) < -0.49 and then C0.Mu_Nom < 0.1 and then C0.Com_Off < 0.0075
                 and then abs Contact.Dot (Contact.Unit (Contact.V3'([P1 (0) - P0 (0), P1 (1) - P0 (1), P1 (2) - P0 (2)]), Ok_U), Geom.Ap (C0.R, Hm.Pads (0).Dir)) > 0.95;
               Put_Line ("     · 平条第 1 名:接触 (" & Codec.Fmt (P0 (0), 4) & "," & Codec.Fmt (P0 (1), 4) & "," & Codec.Fmt (P0 (2), 4) & ") / (" & Codec.Fmt (P1 (0), 4) & ","
                         & Codec.Fmt (P1 (1), 4) & "," & Codec.Fmt (P1 (2), 4) & ") · 进场 (" & Codec.Fmt (C0.Approach (0), 2) & "," & Codec.Fmt (C0.Approach (1), 2) & ","
                         & Codec.Fmt (C0.Approach (2), 2) & ") · 要的摩擦 " & Codec.Fmt (C0.Mu_Nom, 4) & " / 最坏 " & Codec.Fmt (C0.Mu_Worst, 4) & " · 每单位重量要夹 "
                         & Codec.Fmt (C0.Squeeze, 3) & " · 离重心 " & Codec.Fmt (C0.Com_Off, 4) & " · 试了 " & Codec.Img (Stt.Poses) & " 个位姿,落在料上 "
                         & Codec.Img (Stt.Landed_On) & "、合空 " & Codec.Img (Stt.Air) & "、顶到手掌 " & Codec.Img (Stt.Palm_Hit) & "、没对中 " & Codec.Img (Stt.Unbalanced)
                         & " · 摩擦按 " & Codec.Fmt (Stt.Mu_Ref, 4));
            end;
            for C of Fd loop
               for T of C.Touches loop
                  if abs T.P (0) < 0.005 then
                     Ok := False;   --  有接触点落在条的中间(顺着长边夹的那种)
                  end if;
               end loop;
            end loop;
         end if;
         Check (Ok, "接触集·平条:两个接触点落在条的两条长边上(x = ±1 cm)、法向 ±x 朝里、手的朝向合出来的方向 = 两点的连线、从上面进、要的摩擦 < 0.1、离重心 < 半个指肚宽;"
                & "前 5 名里没有一个接触点落在条面中间");
      end;
      --  ②
      Cg2.Plan (Slab (-0.022, 0.022, -0.062, 0.102, 0.004, Blade'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
      for I in 0 .. Natural'Min (3, Natural (Fd.Length)) - 1 loop
         Put_Line ("     · 刀刃 + 把手圈第 " & Codec.Img (I + 1) & " 名:接触 (" & Codec.Fmt (Fd (I).Touches (0).P (0), 4) & "," & Codec.Fmt (Fd (I).Touches (0).P (1), 4) & ") / ("
                   & Codec.Fmt (Fd (I).Touches (1).P (0), 4) & "," & Codec.Fmt (Fd (I).Touches (1).P (1), 4) & ") 相距 " & Codec.Fmt (Fd (I).Width, 4) & " · 进场 ("
                   & Codec.Fmt (Fd (I).Approach (0), 2) & "," & Codec.Fmt (Fd (I).Approach (1), 2) & "," & Codec.Fmt (Fd (I).Approach (2), 2) & ") · 要的摩擦 "
                   & Codec.Fmt (Fd (I).Mu_Nom, 4) & " / 最坏 " & Codec.Fmt (Fd (I).Mu_Worst, 4) & " · 要夹 " & Codec.Fmt (Fd (I).Squeeze, 3) & " · 离重心 "
                   & Codec.Fmt (Fd (I).Com_Off, 4) & " · 重心 (" & Codec.Fmt (Stt.Com (0), 4) & "," & Codec.Fmt (Stt.Com (1), 4) & ")");
      end loop;
      Check (not Fd.Is_Empty and then Fd (0).Width < 0.08,
             "接触集·刀刃 + 把手圈:第 1 名两个接触点在料的两侧、相距 " & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Width, 4)) & "(< 张口 8 cm)");
      --  ③
      Cg2.Plan (Slab (-0.062, 0.062, -0.062, 0.062, 0.01, Plate'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
      for I in 0 .. Natural'Min (2, Natural (Fd.Length)) - 1 loop
         Put_Line ("     · 板第 " & Codec.Img (I + 1) & " 名:接触 (" & Codec.Fmt (Fd (I).Touches (0).P (0), 4) & "," & Codec.Fmt (Fd (I).Touches (0).P (1), 4) & ","
                   & Codec.Fmt (Fd (I).Touches (0).P (2), 4) & ") 法向 (" & Codec.Fmt (Fd (I).Touches (0).N (0), 2) & "," & Codec.Fmt (Fd (I).Touches (0).N (1), 2) & ","
                   & Codec.Fmt (Fd (I).Touches (0).N (2), 2) & ") / (" & Codec.Fmt (Fd (I).Touches (1).P (0), 4) & "," & Codec.Fmt (Fd (I).Touches (1).P (1), 4) & ","
                   & Codec.Fmt (Fd (I).Touches (1).P (2), 4) & ") 法向 (" & Codec.Fmt (Fd (I).Touches (1).N (0), 2) & "," & Codec.Fmt (Fd (I).Touches (1).N (1), 2) & ","
                   & Codec.Fmt (Fd (I).Touches (1).N (2), 2) & ") · 进场 (" & Codec.Fmt (Fd (I).Approach (0), 2) & "," & Codec.Fmt (Fd (I).Approach (1), 2) & ","
                   & Codec.Fmt (Fd (I).Approach (2), 2) & ") · 眼 (" & Codec.Fmt (Fd (I).T (0), 3) & "," & Codec.Fmt (Fd (I).T (1), 3) & "," & Codec.Fmt (Fd (I).T (2), 3) & ")");
      end loop;
      declare
         Corner : Boolean := not Fd.Is_Empty;
         Wall_N : Boolean := not Fd.Is_Empty;
      begin
         for C of Fd loop
            if C.Mu_Nom < 0.8 or else C.Com_Off < 0.04 then
               Corner := False;
            end if;
            --  接触在哪面墙上(离中心哪个坐标大)⇒ 法向该是那面墙朝里的法向
            for T of C.Touches loop
               if Contact.Dot (T.N, (if abs T.P (0) > abs T.P (1) then Contact.V3'([-Sg (T.P (0)), 0.0, 0.0]) else Contact.V3'([0.0, -Sg (T.P (1)), 0.0]))) < 0.95 then
                  Wall_N := False;
               end if;
            end loop;
         end loop;
         Check (Corner and then Wall_N and then Stt.Landed_On > 0 and then Stt.Mu_Ref >= 0.8,
                "接触集·12 cm 见方的板(张口 8 cm):只有斜着夹一个角的(" & Codec.Img (Natural (Fd.Length)) & " 个,每处法向"
                & (if Wall_N then "都是它那面墙朝里的法向" else "有不是它那面墙朝里的法向的") & ",第 1 名要的摩擦 "
                & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Mu_Nom, 3)) & "、离中心 " & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Com_Off, 3))
                & "),摩擦按这一批最不要摩擦的那一把算 " & Codec.Fmt (Stt.Mu_Ref, 3) & ";试了 " & Codec.Img (Stt.Poses) & " 个位姿、落在料上 " & Codec.Img (Stt.Landed_On));
      end;
      --  ④
      Cg2.Plan (Slab (-0.022, 0.022, -0.022, 0.022, 0.03, Disc'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
      Check (not Fd.Is_Empty and then abs (Fd (0).Width - 0.04) < 3.0 * Pitch and then Fd (0).Mu_Nom < 0.2,
             "接触集·直径 4 cm 的圆柱:两点相距 " & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Width, 4)) & "(要约 0.04),要的摩擦 "
             & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Mu_Nom, 4)));
      --  ⑤
      Cg2.Plan (Slab (-0.012, 0.012, -0.102, 0.102, 0.01, Bar'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Never'Access, 5, Fd, Stt);
      Check (Fd.Is_Empty and then Stt.Unreachable > 0, "接触集·按量到的关节范围一个都反解不出来 ⇒ 一个都不给,账上 " & Codec.Img (Stt.Unreachable) & " 个反解不出来");
      --  ⑥ 正三角形:要全部候选(最多 200 个)看尖上的接触
      Cg2.Plan (Slab (-0.032, 0.032, -0.022, 0.038, 0.01, Tri'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 200, Fd, Stt);
      declare
         --  采样以后的三个尖(2 mm 一格里还在三角形里的最外那一点)
         Vx : constant array (0 .. 2) of Contact.V3 := [[0.0, 0.034, 0.0], [-0.028, -0.016, 0.0], [0.028, -0.016, 0.0]];
         N_V, Bad_V : Natural := 0;
         First_Ok : Boolean := False;
      begin
         for C of Fd loop
            for I in 0 .. Natural (C.Touches.Length) - 1 loop
               for V of Vx loop
                  if (C.Touches (I).P (0) - V (0)) ** 2 + (C.Touches (I).P (1) - V (1)) ** 2 < (1.5 * Pitch) ** 2 then
                     N_V := N_V + 1;
                     if Contact.Dot (C.Touches (I).N, Geom.Ap (C.R, Hm.Pads (I).Dir)) < 0.95 then
                        Bad_V := Bad_V + 1;
                     end if;
                  end if;
               end loop;
            end loop;
         end loop;
         if not Fd.Is_Empty then
            declare
               C : constant Cg2.Candidate := Fd (0);
            begin
               for Iv in 0 .. 1 loop
                  if (C.Touches (Iv).P (0) - Vx (0) (0)) ** 2 + (C.Touches (Iv).P (1) - Vx (0) (1)) ** 2 < (1.5 * Pitch) ** 2
                    and then Contact.Dot (C.Touches (Iv).N, Geom.Ap (C.R, Hm.Pads (Iv).Dir)) >= 0.95
                    and then abs (C.Touches (1 - Iv).P (1) - Vx (1) (1)) <= 1.5 * Pitch and then C.Touches (1 - Iv).N (1) >= 0.95
                  then
                     First_Ok := True;
                  end if;
               end loop;
               Put_Line ("     · 三角形第 1 名:接触 (" & Codec.Fmt (C.Touches (0).P (0), 4) & "," & Codec.Fmt (C.Touches (0).P (1), 4) & ") 法向 (" & Codec.Fmt (C.Touches (0).N (0), 2) & ","
                         & Codec.Fmt (C.Touches (0).N (1), 2) & ") / (" & Codec.Fmt (C.Touches (1).P (0), 4) & "," & Codec.Fmt (C.Touches (1).P (1), 4) & ") 法向 ("
                         & Codec.Fmt (C.Touches (1).N (0), 2) & "," & Codec.Fmt (C.Touches (1).N (1), 2) & ") · 进场 (" & Codec.Fmt (C.Approach (0), 2) & ","
                         & Codec.Fmt (C.Approach (1), 2) & "," & Codec.Fmt (C.Approach (2), 2) & ") · 要的摩擦 " & Codec.Fmt (C.Mu_Nom, 4) & " / 最坏 " & Codec.Fmt (C.Mu_Worst, 4));
            end;
         end if;
         Check (First_Ok and then N_V > 0 and then Bad_V = 0,
                "接触集·正三角形:第 1 名" & (if First_Ok then "一块压顶点(法向 = 那一块合拢的方向)、一块压底面(法向 +y)" else "不是压顶点 + 压底面那一把")
                & ";" & Codec.Img (Natural (Fd.Length)) & " 个候选里压在尖上的接触 " & Codec.Img (N_V) & " 处,法向不是那一块合拢方向的 " & Codec.Img (Bad_V) & " 处");
      end;
      --  ⑦ 正六棱柱
      Cg2.Plan (Slab (-0.012, 0.012, -0.012, 0.012, 0.02, Hex'Access), None, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
      declare
         Ok : Boolean := not Fd.Is_Empty and then Fd (0).Mu_Nom < 0.1;
      begin
         if not Fd.Is_Empty then
            for T of Fd (0).Touches loop
               if abs (abs T.P (0) - 0.008) > 1.5 * Pitch or else -T.N (0) * Sg (T.P (0)) < 0.95 then
                  Ok := False;
               end if;
            end loop;
            Put_Line ("     · 六棱柱第 1 名:接触 (" & Codec.Fmt (Fd (0).Touches (0).P (0), 4) & "," & Codec.Fmt (Fd (0).Touches (0).P (1), 4) & ") 法向 ("
                      & Codec.Fmt (Fd (0).Touches (0).N (0), 2) & "," & Codec.Fmt (Fd (0).Touches (0).N (1), 2) & ") / (" & Codec.Fmt (Fd (0).Touches (1).P (0), 4) & ","
                      & Codec.Fmt (Fd (0).Touches (1).P (1), 4) & ") 法向 (" & Codec.Fmt (Fd (0).Touches (1).N (0), 2) & "," & Codec.Fmt (Fd (0).Touches (1).N (1), 2)
                      & ") · 要的摩擦 " & Codec.Fmt (Fd (0).Mu_Nom, 4) & " / 最坏 " & Codec.Fmt (Fd (0).Mu_Worst, 4));
         end if;
         Check (Ok, "接触集·边长 1 cm 的六棱柱(每条边比指肚窄):第 1 名夹在两条对边上(x = ±0.8 cm)、法向 ±x 朝里、要的摩擦 < 0.1");
      end;
      --  ⑧ 两件挨着放
      declare
         function Box (X, Y : Long_Float) return Boolean is (X >= 0.02 and then X <= 0.05 and then abs Y <= 0.03);
         Nb : constant Contact.V3_Vectors.Vector := Slab (0.018, 0.052, -0.032, 0.032, 0.03, Box'Access);
         Bar_P : constant Contact.V3_Vectors.Vector := Slab (-0.012, 0.012, -0.102, 0.102, 0.01, Bar'Access);
         Fd_T : Cg2.Cand_Vectors.Vector;
         St_T : Cg2.Plan_Stats;
         Clear_Y : constant Long_Float := 0.03 + 0.0075 - Pitch;   --  方块的 y 范围再让半个指肚宽(留一个采样间距)
         Ok_Thick, Ok_Thin : Boolean;
      begin
         Cg2.Plan (Bar_P, Nb, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm, 0.0, 0.09, Always'Access, 5, Fd, Stt);
         Cg2.Plan (Bar_P, Nb, Pitch, 0.0005, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], Hm_Thin, 0.0, 0.09, Always'Access, 5, Fd_T, St_T);
         Ok_Thick := not Fd.Is_Empty and then Stt.Blocked > 0;
         if not Fd.Is_Empty then
            for T of Fd (0).Touches loop
               if abs T.P (1) < Clear_Y then
                  Ok_Thick := False;
               end if;
            end loop;
         end if;
         Ok_Thin := not Fd_T.Is_Empty and then Fd_T (0).Com_Off < 0.0075;
         Check (Ok_Thick and then Ok_Thin,
                "接触集·两件挨着放(条右边 1 cm 外一个方块):手指厚 1 cm ⇒ 第 1 名接触在 y = " & (if Fd.Is_Empty then "-" else Codec.Fmt (Fd (0).Touches (0).P (1), 4) & " / "
                & Codec.Fmt (Fd (0).Touches (1).P (1), 4)) & "(要 |y| ≥ " & Codec.Fmt (Clear_Y, 4) & ")、被旁边的东西挡掉 " & Codec.Img (Stt.Blocked)
                & " 个位姿;手指厚 4 mm ⇒ 第 1 名离重心 " & (if Fd_T.Is_Empty then "-" else Codec.Fmt (Fd_T (0).Com_Off, 4)) & "(要 < 0.0075)");
      end;
   end;
   --  🔴 顶面补侧壁(Contact.Surface.Walls_To_Support,09-29):8 × 8 个顶面点(间距 1 cm、离面 3 cm;格子边长两个间距 ⇒ 4 × 4 格)
   --  ⇒ 只从轮廓那一圈 12 格的 48 个点往下补(每个 2 层:2 cm、1 cm 高),中间 4 格的 16 个不补;面斜着放(法向不是 z,点的格子和函数自己的格子转开了)
   --  ⇒ 中间那 16 个照样不补、补出来的点都在顶面和面之间。牙:整块往下填(原来的 Extrude_To_Support)⇒ 中间也补了
   declare
      Top, Out1, Out2 : Contact.V3_Vectors.Vector;
      Ok_Z, Ok_T : Boolean := True;
      Nt : constant Geom.V3 := [0.0, 0.6, 0.8];
      Et : constant Geom.V3 := [1.0, 0.0, 0.0];
      Ft : constant Geom.V3 := [0.0, 0.8, -0.6];   --  和 Nt、Et 都垂直
      Top2 : Contact.V3_Vectors.Vector;
      function Inner (A, B : Long_Float) return Boolean is (A > 0.02 and then A < 0.06 and then B > 0.02 and then B < 0.06);
   begin
      for I in 0 .. 7 loop
         for J in 0 .. 7 loop
            Top.Append (Geom.V3'[0.005 + 0.01 * Long_Float (I), 0.005 + 0.01 * Long_Float (J), 0.03]);
            Top2.Append (Geom.V3'[(0.005 + 0.01 * Long_Float (I)) * Et (0) + (0.005 + 0.01 * Long_Float (J)) * Ft (0) + 0.03 * Nt (0),
                                  (0.005 + 0.01 * Long_Float (I)) * Et (1) + (0.005 + 0.01 * Long_Float (J)) * Ft (1) + 0.03 * Nt (1),
                                  (0.005 + 0.01 * Long_Float (I)) * Et (2) + (0.005 + 0.01 * Long_Float (J)) * Ft (2) + 0.03 * Nt (2)]);
         end loop;
      end loop;
      Contact.Surface.Walls_To_Support (Top, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], 0.01, Out1);
      for K in 64 .. Natural (Out1.Length) - 1 loop
         if not (abs (Out1 (K) (2) - 0.02) < 1.0e-9 or else abs (Out1 (K) (2) - 0.01) < 1.0e-9) or else Inner (Out1 (K) (0), Out1 (K) (1)) then
            Ok_Z := False;
         end if;
      end loop;
      Contact.Surface.Walls_To_Support (Top2, Nt, [0.0, 0.0, 0.0], 0.01, Out2);
      for K in 64 .. Natural (Out2.Length) - 1 loop
         declare
            Q : constant Geom.V3 := Out2 (K);
            Hn : constant Long_Float := Q (0) * Nt (0) + Q (1) * Nt (1) + Q (2) * Nt (2);   --  离面多高
            Ae : constant Long_Float := Q (0) * Et (0) + Q (1) * Et (1) + Q (2) * Et (2);
            Af : constant Long_Float := Q (0) * Ft (0) + Q (1) * Ft (1) + Q (2) * Ft (2);
         begin
            --  格子和点阵转开以后,边上那一圈最多厚到一格的对角(2√2 cm)⇒ 只核离边 3 cm 以上的正中那几个不补
            if Hn <= 0.0 or else Hn >= 0.03 or else (Ae > 0.03 and then Ae < 0.05 and then Af > 0.03 and then Af < 0.05) then
               Ok_T := False;
            end if;
         end;
      end loop;
      Check (Natural (Out1.Length) = 64 + 96 and then Ok_Z and then Ok_T and then Natural (Out2.Length) > 64,
             "顶面补侧壁:8 × 8 个顶面点离面 3 cm ⇒ 只从轮廓那一圈 48 个往下补两层(一共 " & Codec.Img (Natural (Out1.Length)) & " 个,要 160),中间 16 个不补;"
             & "面斜着放 ⇒ " & Codec.Img (Natural (Out2.Length)) & " 个,正中那几个照样不补、补的都在顶面和面之间");
   end;
   --  🔴 朝向定没定住按量到的焦距和画幅判(09-30,Geom.Pointing_Lost,换掉"朝向 ± ≥ 1 弧度"):640×480(半幅对角线 400 px)。
   --  长焦 F = 4000:朝向 ± 0.3 rad 让投影挪 1200 px(画面三倍远)⇒ 定不住;广角 F = 150:± 1.2 rad 只挪 180 px ⇒ 还在画面里、定得住;
   --  F = 400 时两种判法一样(± 0.5 定得住、± 1.2 定不住)。牙:旧的"≥ 1 弧度"长焦那组当成定住了、广角那组当成没定
   declare
      Gp : Geom.Cam_Geo;
      function Old_Lost (Rot_Sd : Long_Float) return Boolean is (Rot_Sd >= 1.0);
   begin
      Gp.Cx := 320.0; Gp.Cy := 240.0;
      Check (Geom.Pointing_Lost (Gp, 4000.0, 0.3) and then not Geom.Pointing_Lost (Gp, 150.0, 1.2)
             and then not Geom.Pointing_Lost (Gp, 400.0, 0.5) and then Geom.Pointing_Lost (Gp, 400.0, 1.2)
             and then not Old_Lost (0.3) and then Old_Lost (1.2),
             "朝向定没定住:长焦 4000 px ± 0.3 rad ⇒ 定不住、广角 150 px ± 1.2 rad ⇒ 定得住、400 px 时和旧判法一样 · 牙:旧的 ≥ 1 弧度两头都判反");
   end;
   --  🔴 两眼交点有多不准(Geom.Meet_Sd,09-30):视线的角度噪声到交点那么远就是垂直于视线的位置噪声,几条合起来求协方差。
   --  两条正交的视线(沿 x、沿 y,各离交点 1 单位,角度噪声 0.001)⇒ 沿 x、y 各 0.001、沿 z 0.001/√2;两条都近乎竖直、只差 2° 的视线
   --  (头顶眼和腕眼都往下看,H53 那种)⇒ 沿竖直方向的不准是横着的几十倍。牙:交点到两条视线的偏差(Spread)在这种时候是 0,看不出它不准
   declare
      Rays : Geom.Sight_Vectors.Vector;
      Sds : Floats;
      Ok : Boolean;
      Spread, Sx, Sz, Sv, Sh : Long_Float;
      P : Geom.V3;
      Half : constant Long_Float := 0.5 * 2.0 * Ada.Numerics.Pi / 180.0;   --  两条视线夹 2°,各偏竖直 1°
      use Ada.Numerics.Long_Elementary_Functions;
   begin
      Rays.Append (Geom.Sight'(O => [-1.0, 0.0, 0.0], D => [1.0, 0.0, 0.0]));
      Rays.Append (Geom.Sight'(O => [0.0, -1.0, 0.0], D => [0.0, 1.0, 0.0]));
      Sds.Append (0.001); Sds.Append (0.001);
      P := Geom.Meet (Rays, Ok, Spread);
      Sx := Geom.Meet_Sd (Rays, Sds, P, [1.0, 0.0, 0.0]);
      Sz := Geom.Meet_Sd (Rays, Sds, P, [0.0, 0.0, 1.0]);
      Check (Ok and then abs (Sx - 0.001) < 1.0e-9 and then abs (Sz - 0.001 / Sqrt (2.0)) < 1.0e-9,
             "两眼交点的不准·两条正交视线:沿 x " & Codec.Fmt (Sx, 6) & "(真 0.001)、沿 z " & Codec.Fmt (Sz, 6) & "(真 0.000707)");
      Rays.Clear;
      Rays.Append (Geom.Sight'(O => [-Sin (Half), 0.0, Cos (Half)], D => [Sin (Half), 0.0, -Cos (Half)]));
      Rays.Append (Geom.Sight'(O => [Sin (Half), 0.0, Cos (Half)], D => [-Sin (Half), 0.0, -Cos (Half)]));
      P := Geom.Meet (Rays, Ok, Spread);
      Sv := Geom.Meet_Sd (Rays, Sds, P, [0.0, 0.0, 1.0]);
      Sh := Geom.Meet_Sd (Rays, Sds, P, [1.0, 0.0, 0.0]);
      Check (Ok and then Spread < 1.0e-9 and then Sv > 20.0 * Sh,
             "两眼交点的不准·两条近乎竖直的视线:交点到视线的偏差 " & Codec.Fmt (Spread, 9) & "(看着很准),沿竖直不准 " & Codec.Fmt (Sv, 5)
             & "、横着 " & Codec.Fmt (Sh, 6) & "(竖直是横着的 " & Codec.Fmt (Sv / Sh, 0) & " 倍)");
      Sds.Clear; Sds.Append (0.001);
      Check (Geom.Meet_Sd (Rays, Sds, P, [0.0, 0.0, 1.0]) = Long_Float'Last, "两眼交点的不准·噪声条数和视线对不上 ⇒ 量不出");
   end;

   --  🔴 接触集往下伸看着走(Act.Plan_Descent,09-30;owner 09-29"为啥会有 3mm 这种数字"):悬停离下手处 9 cm,下手时尖比它顶面低 2 cm;
   --  顶面横着准到 0.2 mm、高低准到 2 mm,尖 1.5 mm,到位差 0.5 mm ⇒ 带子半宽 = 3 × √(0.2² + 2² + 1.5² + 0.5²) mm ≈ 7.7 mm;
   --  带子里一步 = 手自己的不准 3 × √(1.5² + 0.5²) mm 的一半 ≈ 2.4 mm;先一条命令下到"尖碰到顶面那一层之前一条带子、再留两步空走"处,
   --  小步探过带子,剩下到下手处一条命令。牙:原来每步 4 倍最小一档、全程小步(C1 那样 19 步;这里按 0.3 mm 一档算 75 步)
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Floor : constant Long_Float := 0.0003;
      D : constant Act.Descent := Act.Plan_Descent (0.09, 0.02, 0.0002, 0.002, 1.0, 0.0015, 0.0005, 0.0, Floor);
      Band_True : constant Long_Float := 3.0 * Sqrt (0.0002 ** 2 + 0.002 ** 2 + 0.0015 ** 2 + 0.0005 ** 2);
      Step_True : constant Long_Float := 0.5 * 3.0 * Sqrt (0.0015 ** 2 + 0.0005 ** 2);
      Steps_New : constant Long_Float := 1.0 + (D.Fine_End - D.Fast) / D.Lstep + 1.0;
      Steps_Old : constant Long_Float := 0.09 / (4.0 * Floor);
      U : constant Act.Descent := Act.Plan_Descent (0.09, 0.02, 0.0002, Long_Float'Last, 1.0, 0.0015, 0.0005, 0.0, Floor);
   begin
      Check (abs (D.Band - Band_True) < 1.0e-12 and then abs (D.Lstep - Step_True) < 1.0e-12
             and then abs (D.Fast - (0.09 - 0.02 - Band_True - 2.0 * Step_True)) < 1.0e-12 and then abs (D.Fine_End - (0.07 + Band_True)) < 1.0e-12,
             "接触集往下伸·带子半宽 " & Codec.Fmt (1000.0 * D.Band, 2) & " mm、带子里一步 " & Codec.Fmt (1000.0 * D.Lstep, 2) & " mm、先一条命令下 "
             & Codec.Fmt (1000.0 * D.Fast, 1) & " mm、小步探到 " & Codec.Fmt (1000.0 * D.Fine_End, 1) & " mm");
      Check (Steps_New < Steps_Old and then D.Fine_End - D.Fast >= 2.0 * D.Band + 2.0 * D.Lstep - 1.0e-12,
             "接触集往下伸·要 " & Codec.Fmt (Steps_New, 1) & " 步(原来全程 4 倍最小一档要 " & Codec.Fmt (Steps_Old, 0) & " 步),带子前留够两步空走、小步盖住整条带子");
      Check (U.Band = Long_Float'Last and then U.Fast = 0.0 and then U.Fine_End = 0.09 and then abs (U.Lstep - Step_True) < 1.0e-12,
             "接触集往下伸·顶面高低量不出 ⇒ 没有'碰不到它'的那一段,全程小步探");
   end;

   --  🔴 接触集接进执行层(Act.Plan_Contact,09-29):一根 2 cm 宽、12 cm 长(比张口 9 cm 长:顺着长边夹不下)、离面 3 cm 的条沿 x 躺在面上(顶面点间距 2 mm),
   --  x5 那样的两瓣手(眼系两个尖 (±0.045, −0.013, −0.091),指肚宽 1 cm、看得见的厚 2 mm)⇒ 挑出的那一组:合拢方向沿量宽度的 y(不顺着长边 x),
   --  两处接触落在条的两侧(|y| ≈ 1 cm)、法向朝里;把条转 90° 沿 y 放 ⇒ 合拢方向跟着转到 x。身体文件里没有每一瓣的尖 ⇒ 照实说要从零量一次
   declare
      Cx : Act.Context;
      Fx : Plug.Frame;
      Gx : Geom.Cam_Geo;
      Pick : Contact.Grasp.Candidate;
      Nt : Unbounded_String;
      Okp : Boolean;
      procedure Any_Reach (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
         pragma Unreferenced (Arm, Pose);
      begin
         Pos_Err := 0.0; Rot_Err := 0.0;
      end Any_Reach;
      procedure Bar (Along_X : Boolean) is
      begin
         Cx.Sil_Pts := Contact.V3_Vectors.Empty_Vector;
         for I in 0 .. 60 loop
            for J in 0 .. 10 loop
               declare
                  A : constant Long_Float := -0.06 + 0.002 * Long_Float (I);
                  B : constant Long_Float := -0.01 + 0.002 * Long_Float (J);
               begin
                  Cx.Sil_Pts.Append (Geom.V3'(if Along_X then [0.5 + A, B, 0.03] else [0.5 + B, A, 0.03]));
               end;
            end loop;
         end loop;
         Cx.Sil_Valid := True; Cx.Sil_Name := To_Unbounded_String ("bar"); Cx.Sil_Cam := 1; Cx.Sil_N := [0.0, 0.0, 1.0];
         Cx.Sil_P0 := Cx.Sil_Pts.First_Element; Cx.Sil_Pitch := 0.002; Cx.Sil_Err := 0.0005;
      end Bar;
      function Jaw_World (P : Contact.Grasp.Candidate) return Geom.V3 is (Geom.Ap (P.R, [1.0, 0.0, 0.0]));
      Jx1, Jx2 : Geom.V3;
      Sides_Ok : Boolean := False;
      Ok1, Ok2 : Boolean := False;
   begin
      Gx.Valid := True; Gx.F := 400.0; Gx.Cx := 320.0; Gx.Cy := 240.0; Gx.Gap := 0.09;
      Gx.Tip := [0.0, -0.013, -0.091]; Gx.Tip_Valid := True; Gx.Tip_Touch := True; Gx.Tip_Sd := 0.0005;
      Gx.Lobes.Append (Geom.Lobe_Geo'(Tip => [0.045, -0.013, -0.091], Wide => 0.01, Thin => 0.002));
      Gx.Lobes.Append (Geom.Lobe_Geo'(Tip => [-0.045, -0.013, -0.091], Wide => 0.01, Thin => 0.002));
      Cx.Geo.Append (Geom.No_Geo); Cx.Geo.Append (Gx);
      Cx.Map.Amp := Bytes.F64_Vectors.To_Vector (0.0, 6);
      Cx.Map.Amp.Replace_Element (0, 0.001); Cx.Map.Amp.Replace_Element (3, 0.0025);
      Cx.Touch_Valid := True; Cx.Touch_Pt := [0.0, 0.0, 0.0]; Cx.Touch_N := [0.0, 0.0, 1.0];
      Plug.Set_Reach (Any_Reach'Unrestricted_Access);
      Bar (True);
      Act.Plan_Contact (Cx, Fx, 0, 1, To_Unbounded_String ("bar"), Pick, Nt, Okp);
      Ok1 := Okp;
      Jx1 := Jaw_World (Pick);
      if Okp and then Natural (Pick.Touches.Length) = 2 then
         Sides_Ok := abs (abs Pick.Touches (0).P (1) - 0.01) < 0.003 and then abs (abs Pick.Touches (1).P (1) - 0.01) < 0.003
           and then Pick.Touches (0).P (1) * Pick.Touches (1).P (1) < 0.0
           and then Pick.Touches (0).N (1) * Pick.Touches (0).P (1) < 0.0 and then Pick.Touches (1).N (1) * Pick.Touches (1).P (1) < 0.0;
      end if;
      Bar (False);
      Act.Plan_Contact (Cx, Fx, 0, 1, To_Unbounded_String ("bar"), Pick, Nt, Okp);
      Ok2 := Okp;
      Jx2 := Jaw_World (Pick);
      --  没有每一瓣的尖(老身体文件)
      declare
         G2 : Geom.Cam_Geo := Cx.Geo (1);
         Nt3 : Unbounded_String;
         Ok3 : Boolean;
      begin
         G2.Lobes.Clear;
         Cx.Geo.Replace_Element (1, G2);
         Act.Plan_Contact (Cx, Fx, 0, 1, To_Unbounded_String ("bar"), Pick, Nt3, Ok3);
         Plug.Set_Reach (null);
         Check (Ok1 and then Sides_Ok and then abs Jx1 (1) > 0.95 and then Ok2 and then abs Jx2 (0) > 0.95 and then not Ok3
                and then Index (Nt3, "measured once from scratch") > 0,
                "接触集接进执行层:沿 x 躺的 2 cm 宽的条 ⇒ 合拢方向在世界里 (" & Codec.Fmt (Jx1 (0), 2) & "," & Codec.Fmt (Jx1 (1), 2) & "," & Codec.Fmt (Jx1 (2), 2)
                & ")(要沿 y)、两处接触在条两侧、法向朝里 · 条转 90° ⇒ (" & Codec.Fmt (Jx2 (0), 2) & "," & Codec.Fmt (Jx2 (1), 2) & ")(要沿 x)· 没有每一瓣的尖 ⇒ 照实说");
      end;
   end;
   --  🔴 到过的范围(Jointboot:到过的范围 + 往外一步、记尽头、碰上东西不记、越过尽头删掉;09-29 owner"已知范围,越用越大"):合成的 6 关节胳膊装上
   --  (同上面运动学那条的几何),假身体只按关节命令走 —— 第 4 个关节真尽头 0.9 弧度(反解不知道);到过的范围一开始每个关节 ±0.3、往外一步 0.2。
   --  ① 要去一个第 4 个关节得转到 1.3 的位姿:每条命令都只到"到过的范围 + 一步"里,手到了那儿范围长了才再往前(重发的旗子 Held_Back);
   --     走到 0.9 卡住、别的关节都到了 ⇒ 记下这一头(之后问"够不够得着"那个位姿就解不到了);
   --  ② 手压在东西上:要到范围外的那个关节没走到一半,同时别的关节被顶偏 ⇒ 不记;同样没走到一半、别的关节都到了 ⇒ 记(正反对照);
   --  ③ 读数越过了记下的尽头 ⇒ 删掉(那个位姿又够得着了);④ 纯函数:两个关节都没走到 ⇒ 分不清、不记;只出范围一丝(不到一档)⇒ 当范围里;
   --  ⑤ 开机扫描 ⇒ 尽头 / 到过的范围 / 往外一步(Set_Ranges);
   --  ⑥ 走真的 Selfmap.Go(锁步里一只假手发命令,主线程当假身体):命令隔一拍才起效、每拍每个关节最多转 0.1 弧度、第 4 个关节真尽头 0.9,
   --     读数噪声 1 µm(真 x5 4e-5 m;反解每次重解的数值抖动约 1e-8 弧度,在它下面)——
   --     (a) 要转到 1.3(慢步,等停):先按记下的尽头解出要到的关节、每个关节夹到"到过的范围 + 一步"里发;截住以后手一动、范围一长就重发,
   --         一条 Go 里走到 0.9、停下、记下这一头;拍数 ≤ 走的 9 拍 + 起效 1 拍 + 停下 2 拍 = 12;
   --     (b) 同样 1.3,按压的那种步(Press:沿命令方向停下就读):截住时不许先收,照样 12 拍走到 0.9、记下;(c) 要到 0.8:8 + 1 + 2 = 11 拍走到、不记尽头;
   --     三条里别的关节一直不动(< 1e-6 弧度)。牙(09-29 离线各拆一处跑过):直接在夹过的范围里反解(c629b87 那一版)⇒
   --     被夹住的那一点由别的关节凑、别的关节被拉出去 0.557 弧度、14 / 14 / 13 拍,红;只在停稳以后才重发(V1B63 那一版)⇒ 27 / 22 / 17 拍,红;
   --     截住时 Press 照样先收 ⇒ 第 2 拍停在 0.1,红
   declare
      use Geom;
      use type Plug.Limit_State;
      Wax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]];
      Pax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 0.05], [0.0, 0.0, 0.12], [0.25, 0.0, 0.12], [0.45, 0.0, 0.16], [0.5, 0.0, 0.16], [0.55, 0.0, 0.16]];
      C0 : constant V3 := [0.6, 0.0, 0.22];
      M : Kinem.Model;
      True_End : constant Long_Float := 0.9;
      Step : constant Long_Float := 0.2;
      Got0 : constant Long_Float := 0.3;
      Fr : Plug.Frame;
      type Six is array (0 .. 5) of Long_Float;
      Lo_Seen, Hi_Seen : Six;
      function Q6 (J : Natural; V : Long_Float) return Floats is
         Q : Floats;
      begin
         for K in 0 .. 5 loop
            Q.Append (if K = J then V else 0.0);
         end loop;
         return Q;
      end Q6;
      function Pose_Of (Q : Floats) return Plug.Arm_Pose is
         R : M3;
         T : V3;
      begin
         Kinem.FK (M, Q, R, T);
         return Kinem.To_Pose (R, T);
      end Pose_Of;
      --  假身体停在 Q 上:喂 N 拍(第一拍是动,后面是停着)
      procedure Feed (Q : Floats; N : Positive) is
      begin
         for I in 1 .. N loop
            Fr.Joints.Clear; Fr.Joints.Append (Q);
            Jointboot.Pose_Hook (Fr);
         end loop;
         for J in 0 .. 5 loop
            Lo_Seen (J) := Long_Float'Min (Lo_Seen (J), Q (J)); Hi_Seen (J) := Long_Float'Max (Hi_Seen (J), Q (J));
         end loop;
      end Feed;
      procedure Setup is
         W : Jointboot.Arm_World;
         Ws : Jointboot.Arm_World_Vectors.Vector;
      begin
         W.Group := 0; W.Valid := True; W.Model := M; W.S := 1.0; W.Eye_W := 640;
         for J in 0 .. 5 loop
            W.Lo.Append (Long_Float'First); W.Hi.Append (Long_Float'Last);
            W.Got_Lo.Append (-Got0); W.Got_Hi.Append (Got0); W.Step_Lo.Append (Step); W.Step_Hi.Append (Step);
            Lo_Seen (J) := -Got0; Hi_Seen (J) := Got0;
         end loop;
         Ws.Append (W);
         Jointboot.Install (Ws, Identity, [0.0, 0.0, 0.0], Joint_Noise => 0.0);
         Feed (Q6 (0, 0.0), 3);
      end Setup;
      --  发一条位姿命令,假身体照关节目标走(第 4 个关节到 0.9 为止;Push = 另把第 Pj 个关节顶偏 Pv);返回关节目标
      function Command (Goal : Plug.Arm_Pose; Pj : Integer := -1; Pv : Long_Float := 0.0; Short_J : Integer := -1; Short_At : Long_Float := 0.0) return Floats is
         C : Plug.Cmd;
         Ok : Boolean;
         Qb : Floats;
      begin
         C.Kind := Plug.Ee; C.Arm := 0; C.Pose := Goal;
         Jointboot.Cmd_Hook (C, Ok);
         if not Ok then
            return Qb;
         end if;
         Qb := C.Q;
         if Qb (4) > True_End then
            Qb.Replace_Element (4, True_End);
         end if;
         if Short_J >= 0 then
            Qb.Replace_Element (Short_J, Short_At);
         end if;
         if Pj >= 0 then
            Qb.Replace_Element (Pj, Qb (Pj) + Pv);
         end if;
         Feed (Qb, 3);
         return C.Q;
      end Command;
      Goal : Plug.Arm_Pose;
      Pe0, Re0, Pe1, Re1, Pe2, Re2 : Long_Float;
      Okr : Boolean;
      Cmds : Natural := 0;
      In_Step : Boolean := True;
      Max_4 : Long_Float := 0.0;
   begin
      M.N := 6; M.F := 400.0; M.Cx := 320.0; M.Cy := 240.0; M.Valid := True; M.Q0 := Q6 (0, 0.0);
      for I in 0 .. 5 loop
         M.Ax (I).W := Wax (I);
         M.Ax (I).P := [Pax (I) (0) - C0 (0), Pax (I) (1) - C0 (1), Pax (I) (2) - C0 (2)];
      end loop;
      --  ①
      Setup;
      Goal := Pose_Of (Q6 (4, 1.3));
      Plug.Reach (0, Goal, Pe0, Re0, Okr);
      loop
         declare
            Before_Lo : constant Six := Lo_Seen;
            Before_Hi : constant Six := Hi_Seen;
            Q : constant Floats := Command (Goal);
         begin
            exit when Q.Is_Empty;
            Cmds := Cmds + 1;
            for J in 0 .. 5 loop
               if Q (J) > Before_Hi (J) + Step + 1.0e-9 or else Q (J) < Before_Lo (J) - Step - 1.0e-9 then
                  In_Step := False;
               end if;
            end loop;
            Max_4 := Long_Float'Max (Max_4, Q (4));
            exit when Jointboot.Held_Back (0) /= Plug.Held_Grown or else Cmds >= 40;
         end;
      end loop;
      Plug.Reach (0, Goal, Pe1, Re1, Okr);
      Check (Okr and then In_Step and then Cmds >= 4 and then Cmds < 40 and then Pe0 < 1.0e-6 and then Re0 < 1.0e-6 and then (Pe1 > 1.0e-4 or else Re1 > 1.0e-4)
             and then abs (Hi_Seen (4) - True_End) < 1.0e-12,
             "到过的范围·大转拆开、卡住那一头记下:要把第 4 个关节转到 1.3(真尽头 0.9、到过 ±0.3、往外一步 0.2)," & Codec.Img (Cmds)
             & " 条命令、每条都在到过的范围 + 一步里(第 4 个关节最远要到 " & Codec.Fmt (Max_4, 3) & ");停在 0.9 以后记下这一头 ⇒ 问够不够得着:记之前差 "
             & Codec.Fmt (Pe0, 7) & " / " & Codec.Fmt (Re0, 7) & ",记之后 " & Codec.Fmt (Pe1, 4) & " / " & Codec.Fmt (Re1, 4) & " rad");
      --  ③ 读数越过了记下的尽头(真尽头其实更远)⇒ 删掉
      Feed (Q6 (4, 0.95), 3);
      Plug.Reach (0, Goal, Pe2, Re2, Okr);
      Check (Pe2 < 1.0e-6 and then Re2 < 1.0e-6,
             "到过的范围·读数到了 0.95、越过记下的 0.9 ⇒ 那个尽头删掉,那个位姿又够得着了(差 " & Codec.Fmt (Pe2, 7) & " / " & Codec.Fmt (Re2, 7) & ")");
      --  ② 正反对照:第 4 个关节要到 0.5(到过 0.3、往外一步正好 0.5),只走到 0.35(不到要往外走的 0.2 的一半)。
      --  用第 4 个(腕转)不用第 2 个:第 1–3 个是三根平行的俯仰轴,第 2 个记了尽头,反解换成手肘翻过去的那个解照样到得了,"够不着了"判不出来
      declare
         Goal2 : constant Plug.Arm_Pose := Pose_Of (Q6 (4, 0.5));
         Q : Floats;
         Pe_C, Re_C, Pe_E, Re_E : Long_Float;
      begin
         Setup;
         Q := Command (Goal2, Pj => 1, Pv => 0.14, Short_J => 4, Short_At => 0.35);   --  别的关节(第 1 个)被顶偏 0.14 = 手压在东西上
         Plug.Reach (0, Goal2, Pe_C, Re_C, Okr);
         Setup;
         Q := Command (Goal2, Short_J => 4, Short_At => 0.35);                       --  别的关节都到了 = 关节到头
         Plug.Reach (0, Goal2, Pe_E, Re_E, Okr);
         Check (not Q.Is_Empty and then Pe_C < 1.0e-6 and then Re_C < 1.0e-6 and then (Pe_E > 1.0e-4 or else Re_E > 1.0e-4),
                "到过的范围·碰上东西 vs 关节到头:第 4 个关节只走到 0.35(要 0.5)—— 别的关节被顶偏 0.14 ⇒ 不记(还够得着,差 " & Codec.Fmt (Pe_C, 7)
                & ");别的关节都到了 ⇒ 记下 0.35(那个位姿解不到了,差 " & Codec.Fmt (Pe_E, 4) & " / " & Codec.Fmt (Re_E, 4) & " rad)");
      end;
      --  ⑥
      declare
         Lk : Plug.Link;
         Mp : Selfmap.Body_Map;
         Fr0 : Plug.Frame;
         Goal_G : Plug.Arm_Pose;
         Press_G : Boolean := False;
         Go_Frames : Natural := 0;
         Go_Ok : Boolean := False;
         Body_Q, Act_Q, Pend_Q : Floats;   --  假身体此刻的关节 / 正在走向的目标 / 这一拍收到、下一拍才起效的目标
         Max_Other : Long_Float := 0.0;    --  三条里第 4 个以外的关节离开 0 最远到过多少
         task type Go_Hand;
         task body Go_Hand is
            Fr : Plug.Frame := Fr0;
            Dl : Table.Vec;
         begin
            Lockstep.Begin_Hand (0);
            Selfmap.Go (Lk, Mp, 0, Goal_G, Bytes.F64_Vectors.Empty_Vector, Fr, Dl, Go_Frames, Go_Ok, Press => Press_G);
            Lockstep.Done;
         end Go_Hand;
         --  走一条 Go;返回主线程走了几拍
         function Run_Go (Goal : Plug.Arm_Pose; Quick : Boolean) return Natural is
            Beats : Natural := 0;
         begin
            Setup;
            Goal_G := Goal; Press_G := Quick;
            Body_Q := Q6 (0, 0.0); Act_Q := Body_Q; Pend_Q := Body_Q;
            Fr0.Joints.Clear; Fr0.Joints.Append (Body_Q);
            Jointboot.Pose_Hook (Fr0);
            Lockstep.Clear;
            Plug.Lock_Begin;
            declare
               Hd : Go_Hand;
            begin
               Lockstep.Start (0, Hd'Identity);
               loop
                  Lockstep.Run (0);
                  exit when Lockstep.Finished (0);
                  Beats := Beats + 1;
                  Act_Q := Pend_Q;
                  declare
                     Mg : constant Plug.Cmd := Plug.Lock_Merged;
                  begin
                     if not Mg.Qs.Is_Empty then
                        Pend_Q := Mg.Qs (0);
                     end if;
                  end;
                  for J in 0 .. 5 loop
                     Body_Q.Replace_Element (J, Body_Q (J) + Long_Float'Max (-0.1, Long_Float'Min (0.1, Act_Q (J) - Body_Q (J))));
                  end loop;
                  if Body_Q (4) > True_End then
                     Body_Q.Replace_Element (4, True_End);
                  end if;
                  for J in 0 .. 5 loop
                     if J /= 4 then
                        Max_Other := Long_Float'Max (Max_Other, abs Body_Q (J));
                     end if;
                  end loop;
                  declare
                     Ff : Plug.Frame;
                  begin
                     Ff.Joints.Append (Body_Q);
                     Jointboot.Pose_Hook (Ff);
                     Plug.Lock_Feed (Ff);
                  end;
               end loop;
            end;
            Plug.Lock_End;
            Lockstep.Clear;
            return Beats;
         end Run_Go;
         Ba, Bb, Bc : Natural;
         Ea, Eb : Long_Float;
         Q4c : Long_Float;
         Pe_A, Re_A, Pe_C, Re_C : Long_Float;
         Oka : Boolean;
      begin
         Mp.Settle := 2; Mp.EE_Noise := 1.0e-6; Mp.Rot_Noise := 1.0e-6;
         Ba := Run_Go (Pose_Of (Q6 (4, 1.3)), False);
         Ea := Body_Q (4);
         Plug.Reach (0, Pose_Of (Q6 (4, 1.3)), Pe_A, Re_A, Oka);
         Bb := Run_Go (Pose_Of (Q6 (4, 1.3)), True);
         Eb := Body_Q (4);
         Bc := Run_Go (Pose_Of (Q6 (4, 0.8)), False);
         Q4c := Body_Q (4);
         Plug.Reach (0, Pose_Of (Q6 (4, 0.85)), Pe_C, Re_C, Oka);   --  没记尽头 ⇒ 比 0.8 再远一点(真尽头以内)照样解得到
         Check (abs (Ea - True_End) < 1.0e-9 and then Ba <= 12 and then (Pe_A > 1.0e-4 or else Re_A > 1.0e-4)
                and then abs (Eb - True_End) < 1.0e-9 and then Bb <= 12
                and then abs (Q4c - 0.8) < 1.0e-6 and then Bc <= 11 and then Pe_C < 1.0e-6 and then Re_C < 1.0e-6 and then Max_Other < 1.0e-6,
                "到过的范围·走真的 Go(命令隔一拍起效、每拍最多 0.1 弧度、真尽头 0.9):要 1.3 慢步 ⇒ 一条 Go " & Codec.Img (Ba) & " 拍停在 " & Codec.Fmt (Ea, 3)
                & "、记下尽头(之后问够不够得着差 " & Codec.Fmt (Re_A, 3) & " rad);按压的那种步 ⇒ " & Codec.Img (Bb) & " 拍停在 " & Codec.Fmt (Eb, 3)
                & ";要 0.8 ⇒ " & Codec.Img (Bc) & " 拍走到 " & Codec.Fmt (Q4c, 4) & "、不记尽头(问 0.85 差 " & Codec.Fmt (Pe_C, 7) & ")(拍数要 ≤ 12 / 12 / 11)"
                & " · 别的关节最远离开 0 " & Codec.Fmt (Max_Other, 9) & " 弧度(要 < 1e-6)");
      end;
      --  ⑦ 压的那一步(Selfmap.Go 的 Press,09-29):沿命令方向停下就读 —— 真的 Go + 锁步里一只假手,眼往下 2 cm,假身体三种:
      --     (i) 空中:每拍走还差的九成 ⇒ 到了就收(≤ 4 拍);(ii) 被挡住:只走到三成就停,手腕还在每拍转 3e-4 弧度地蠕动
      --     (V1B50 真值:顶住以后每拍还转约 0.004°)⇒ 沿命令方向连着两拍不挪就读(≤ 6 拍);同一条不按 Press 走要等到上限(≥ 12 拍);
      --     (iii) 胳膊慢慢漂(每拍只走还差的 5%;V1B65 伸远了跟不上)⇒ 沿命令方向一直在挪,不许提前读(到上限才收)。
      --     牙:原来的快读(到了量出来的稳定拍数就读)⇒ (iii) 第 5 拍就读、红
      declare
         Lk : Plug.Link;
         Mp : Selfmap.Body_Map;
         Fr0 : Plug.Frame;
         Goal_P : Plug.Arm_Pose;
         Press_P : Boolean := True;
         Go_Frames : Natural := 0;
         Go_Ok : Boolean := False;
         Mode : Natural := 0;
         Body_Q, Pend_Q, Act_Q, Q0 : Floats;
         task type Go_Hand;
         task body Go_Hand is
            Fr : Plug.Frame := Fr0;
            Dl : Table.Vec;
         begin
            Lockstep.Begin_Hand (0);
            Selfmap.Go (Lk, Mp, 0, Goal_P, Bytes.F64_Vectors.Empty_Vector, Fr, Dl, Go_Frames, Go_Ok, Press => Press_P, Tol => 0.005, Tol_Rot => 0.0025);
            Lockstep.Done;
         end Go_Hand;
         function Run (M_Mode : Natural; With_Press : Boolean) return Natural is
            Beats : Natural := 0;
            Home : constant Plug.Arm_Pose := Pose_Of (Q6 (0, 0.0));
         begin
            Setup;
            Mode := M_Mode; Press_P := With_Press;
            Goal_P := Home; Goal_P (2) := Home (2) - 0.02;
            Body_Q := Q6 (0, 0.0); Pend_Q := Body_Q; Act_Q := Body_Q; Q0 := Body_Q;
            Fr0.Joints.Clear; Fr0.Joints.Append (Body_Q);
            Jointboot.Pose_Hook (Fr0);
            Lockstep.Clear;
            Plug.Lock_Begin;
            declare
               Hd : Go_Hand;
            begin
               Lockstep.Start (0, Hd'Identity);
               loop
                  Lockstep.Run (0);
                  exit when Lockstep.Finished (0);
                  Beats := Beats + 1;
                  Act_Q := Pend_Q;
                  declare
                     Mg : constant Plug.Cmd := Plug.Lock_Merged;
                  begin
                     if not Mg.Qs.Is_Empty then
                        Pend_Q := Mg.Qs (0);
                     end if;
                  end;
                  for J in 0 .. 5 loop
                     declare
                        Rate : constant Long_Float := (if Mode = 2 then 0.05 else 0.9);   --  每拍走还差的几成(比例)
                        Nx : Long_Float := Body_Q (J) + Rate * (Act_Q (J) - Body_Q (J));
                        Lim3 : constant Long_Float := abs (0.3 * (Act_Q (J) - Q0 (J)));   --  被挡住:离起点最多三成(比例)
                     begin
                        if Mode = 1 then
                           Nx := (if J = 5 then Body_Q (J) + 3.0e-4 else Q0 (J) + Long_Float'Max (-Lim3, Long_Float'Min (Lim3, Nx - Q0 (J))));
                        end if;
                        Body_Q.Replace_Element (J, Nx);
                     end;
                  end loop;
                  declare
                     Ff : Plug.Frame;
                  begin
                     Ff.Joints.Append (Body_Q);
                     Jointboot.Pose_Hook (Ff);
                     Plug.Lock_Feed (Ff);
                  end;
               end loop;
            end;
            Plug.Lock_End;
            Lockstep.Clear;
            return Beats;
         end Run;
         B1, B2, B2n, B3 : Natural;
      begin
         Mp.Settle := 5; Mp.EE_Noise := 1.0e-6; Mp.Rot_Noise := 1.0e-6;
         B1 := Run (0, True);
         B2 := Run (1, True);
         B2n := Run (1, False);
         B3 := Run (2, True);
         Check (B1 <= 4 and then B2 <= 6 and then B2n >= 12 and then B3 >= 12 + Mp.Settle,
                "压的那一步(Go 的 Press):空中 " & Codec.Img (B1) & " 拍到(要 ≤ 4);被挡住、手腕还在蠕动 ⇒ " & Codec.Img (B2)
                & " 拍就读(要 ≤ 6;不按 Press 要 " & Codec.Img (B2n) & " 拍);胳膊慢慢漂 ⇒ " & Codec.Img (B3) & " 拍才收(要 ≥ " & Codec.Img (12 + Mp.Settle) & ",不许提前读)");
      end;
      Plug.Set_Hooks (null, null); Plug.Set_Reach (null); Plug.Set_Limit (null);
      --  ④ 纯函数
      declare
         use type Jointboot.End_Verdict;
         Glo, Ghi, Qc, Qa, Qn, Stp : Floats;
         Jx : Integer;
         Hs : Boolean;
         V1, V2, V3, V4 : Jointboot.End_Verdict;
         Tol : constant Long_Float := Kinem.Clean_Tol (640.0);
      begin
         for J in 0 .. 5 loop
            Glo.Append (-Got0); Ghi.Append (Got0); Qa.Append (0.0); Stp.Append (Step);
         end loop;
         Qc := Q6 (0, 0.5); Qc.Replace_Element (1, 0.5);
         Qn := Q6 (0, 0.31); Qn.Replace_Element (1, 0.32);
         V1 := Jointboot.Judge_End (Qc, Qa, Qn, Glo, Ghi, Stp, Stp, Tol, Jx, Hs);
         Qc := Q6 (0, Got0 + 0.5 * Tol);
         Qn := Q6 (0, Got0);
         V2 := Jointboot.Judge_End (Qc, Qa, Qn, Glo, Ghi, Stp, Stp, Tol, Jx, Hs);
         --  V1B64 那一幕:只多要 0.05(不到半步 0.1)、只走到 0.31(不到一半)⇒ 不核;同一处多要一整步 0.2、只走到 0.31 ⇒ 记(正反对照)
         Qc := Q6 (0, Got0 + 0.05); Qa := Q6 (0, Got0);
         Qn := Q6 (0, 0.31);
         V3 := Jointboot.Judge_End (Qc, Qa, Qn, Glo, Ghi, Stp, Stp, Tol, Jx, Hs);
         Qc := Q6 (0, Got0 + Step);
         V4 := Jointboot.Judge_End (Qc, Qa, Qn, Glo, Ghi, Stp, Stp, Tol, Jx, Hs);
         Check (V1 = Jointboot.Ambiguous and then V2 = Jointboot.Reached and then V3 = Jointboot.Reached and then V4 = Jointboot.End_Hit,
                "到过的范围·判尽头:两个关节都要到范围外、都没走到一半 ⇒ 分不清是哪一个(" & V1'Image & ",不记);只出范围半档(" & Codec.Fmt (0.5 * Tol, 5)
                & ")⇒ 当在范围里、到了(" & V2'Image & ");只多要 0.05(不到半步)、只走到 0.31 ⇒ 不核(" & V3'Image & ",V1B64 那种假尽头不记);"
                & "多要一整步、同样只到 0.31 ⇒ " & V4'Image & "(记)");
      end;
      --  ⑤ 开机扫描 ⇒ 尽头、到过的范围、往外一步
      declare
         D : Jointboot.Sweep_Data;
         W : Jointboot.Arm_World;
      begin
         D.W := 640; D.H := 480;
         for Fk in 0 .. 2 loop
            declare
               Fi : Kinem.Frame_Info;
            begin
               Fi.Joint := (if Fk = 0 then -1 else 1);
               Fi.Q := Q6 (1, (if Fk = 0 then 0.0 elsif Fk = 1 then -0.4 else 0.25));
               D.Frames.Append (Fi);
            end;
         end loop;
         for J in 0 .. 5 loop
            D.Has_Lo.Append (False); D.Has_Hi.Append (J = 1);
            D.Step_Lo.Append (0.1); D.Step_Hi.Append (0.05);
         end loop;
         Jointboot.Set_Ranges (D, W);
         Check (W.Hi (1) = 0.25 and then W.Lo (1) = Long_Float'First and then W.Hi (0) = Long_Float'Last and then W.Got_Lo (1) = -0.4 and then W.Got_Hi (1) = 0.25
                and then W.Got_Lo (0) = 0.0 and then W.Step_Lo (1) = 0.1 and then W.Step_Hi (1) = 0.05 and then W.Eye_W = 640,
                "到过的范围·开机扫描 ⇒ 第 1 个关节往正是关节到头停的:尽头 0.25、往负没尽头;到过的范围 [-0.4, 0.25];往外一步 0.1 / 0.05;画幅 640");
      end;
   end;
   --  🔴 扫描时碰上东西不是关节尽头(Jointboot.Sweep_Stops / Sweep_Stop_Is_End;09-28 H4 + owner 定的"到过的范围"):用在线量到的数 ——
   --  H4 人形第 0 关节往正:这一格命令 0.2011 走满、第 5 关节被顶偏 0.139 ⇒ 停、不记界;x5 V1B59 第 1 关节往负:命令 0.2602 只到 0.0822、别的关节偏 0.001 ⇒ 停、记界;
   --  关节自己没转到三分之一、同时别的关节被顶偏 ⇒ 仍是碰上东西、不记界;走满、谁也没被顶 ⇒ 不停
   Check (Jointboot.Sweep_Stops (0.2011, 0.139, 0.2011) and then not Jointboot.Sweep_Stop_Is_End (0.2011, 0.139, 0.2011)
          and then Jointboot.Sweep_Stops (0.0822, 0.001, 0.2602) and then Jointboot.Sweep_Stop_Is_End (0.0822, 0.001, 0.2602)
          and then Jointboot.Sweep_Stops (0.02, 0.139, 0.2011) and then not Jointboot.Sweep_Stop_Is_End (0.02, 0.139, 0.2011)
          and then not Jointboot.Sweep_Stops (0.2, 0.001, 0.2011),
          "扫描停下记不记界:碰桌(别的关节被顶偏 0.139)停、不记界;x5 真到头(实到 0.0822 / 0.2602、别的只偏 0.001)停、记界;自己停住又顶偏别人仍不记;走满没顶偏不停");
   --  🔴 几个关节一起动的格子怎么排(Jointboot.Multi_Cells / Multi_Up,09-30 C 组):一组 N 个关节走 N + 1 格,正负取 Sylvester 型 Hadamard 矩阵的行 ⇒
   --  N = 1 .. 24 每一个 [全 1 | 各关节的正负] 都满秩(按素数 2^31 − 1 的余数消元数秩:整数运算、不靠浮点容差;余数下满秩 ⇒ 实数下也满秩);
   --  N = 7(人形一条胳膊,8 格)各关节的正负两两正交、正负各一半。牙:原来 8 格、(格子号 × 37 + 关节号 × 11) mod 16 < 8 为正 ⇒
   --  N = 9 秩只有 8(要 10),第 0 与第 8 个关节正负恰好相反、第 0 与第 16 个完全相同 —— 这几个关节的效果分不开
   declare
      P : constant Long_Long_Integer := 2147483647;   --  素数 2^31 − 1:两个余数相乘不出 64 位
      type Mat is array (Natural range <>, Natural range <>) of Long_Long_Integer;
      function Pow_Mod (B, E : Long_Long_Integer) return Long_Long_Integer is
         R : Long_Long_Integer := 1;
         Bb : Long_Long_Integer := B mod P;
         Ee : Long_Long_Integer := E;
      begin
         while Ee > 0 loop
            if Ee mod 2 = 1 then
               R := R * Bb mod P;
            end if;
            Bb := Bb * Bb mod P;
            Ee := Ee / 2;
         end loop;
         return R;
      end Pow_Mod;
      function Rank (M0 : Mat) return Natural is
         M : Mat := M0;
         R : Natural := M'First (1);
      begin
         for I in M'Range (1) loop
            for J in M'Range (2) loop
               M (I, J) := M (I, J) mod P;
            end loop;
         end loop;
         for C in M'Range (2) loop
            exit when R > M'Last (1);
            declare
               Pv : Integer := -1;
            begin
               for I in R .. M'Last (1) loop
                  if M (I, C) /= 0 then
                     Pv := I;
                     exit;
                  end if;
               end loop;
               if Pv >= 0 then
                  for J in M'Range (2) loop
                     declare
                        T : constant Long_Long_Integer := M (R, J);
                     begin
                        M (R, J) := M (Pv, J); M (Pv, J) := T;
                     end;
                  end loop;
                  declare
                     Inv : constant Long_Long_Integer := Pow_Mod (M (R, C), P - 2);   --  费马小定理求逆
                  begin
                     for I in R + 1 .. M'Last (1) loop
                        declare
                           Fct : constant Long_Long_Integer := M (I, C) * Inv mod P;
                        begin
                           for J in M'Range (2) loop
                              M (I, J) := (M (I, J) - Fct * M (R, J)) mod P;
                           end loop;
                        end;
                     end loop;
                  end;
                  R := R + 1;
               end if;
            end;
         end loop;
         return R - M'First (1);
      end Rank;
      function New_Design (N : Natural) return Mat is
         M : Mat (0 .. Jointboot.Multi_Cells (N) - 1, 0 .. N);
      begin
         for C in M'Range (1) loop
            M (C, 0) := 1;
            for J in 0 .. N - 1 loop
               M (C, J + 1) := (if Jointboot.Multi_Up (C, J) then 1 else -1);
            end loop;
         end loop;
         return M;
      end New_Design;
      function Old_Design (N : Natural) return Mat is
         M : Mat (0 .. 7, 0 .. N);
      begin
         for Cb in 1 .. 8 loop
            M (Cb - 1, 0) := 1;
            for J in 0 .. N - 1 loop
               M (Cb - 1, J + 1) := (if ((Cb * 37 + J * 11) mod 16) < 8 then 1 else -1);   --  原来的排法(09-30 以前的 jointboot)
            end loop;
         end loop;
         return M;
      end Old_Design;
      Full_All : Boolean := True;
      Bad_N : Natural := 0;
      D7 : constant Mat := New_Design (7);
      Orth7 : Boolean := True;
      O9 : constant Mat := Old_Design (9);
      O17 : constant Mat := Old_Design (17);
      Opp_0_8, Same_0_16 : Boolean := True;
   begin
      for N in 1 .. 24 loop
         if Rank (New_Design (N)) /= N + 1 then
            Full_All := False; Bad_N := N;
         end if;
      end loop;
      for A in 1 .. 7 loop
         declare
            Sa : Long_Long_Integer := 0;
         begin
            for C in D7'Range (1) loop
               Sa := Sa + D7 (C, A);
            end loop;
            Orth7 := Orth7 and then Sa = 0;
         end;
         for B in A + 1 .. 7 loop
            declare
               Dt : Long_Long_Integer := 0;
            begin
               for C in D7'Range (1) loop
                  Dt := Dt + D7 (C, A) * D7 (C, B);
               end loop;
               Orth7 := Orth7 and then Dt = 0;
            end;
         end loop;
      end loop;
      for C in 0 .. 7 loop
         Opp_0_8 := Opp_0_8 and then O9 (C, 1) = -O9 (C, 9);
         Same_0_16 := Same_0_16 and then O17 (C, 1) = O17 (C, 17);
      end loop;
      Check (Full_All and then Orth7 and then D7'Length (1) = 8 and then Rank (O9) < 10 and then Opp_0_8 and then Same_0_16,
             "几个关节一起动的格子:N 个关节 N + 1 格、正负取 Hadamard 的行 ⇒ N = 1 .. 24 都满秩" & (if Full_All then "" else "(N = " & Codec.Img (Bad_N) & " 不满秩)")
             & ";7 个关节 8 格两两正交、正负各一半" & (if Orth7 then "" else "(不正交!)")
             & " · 牙:原来 8 格的排法 N = 9 秩只有 " & Codec.Img (Rank (O9)) & "(要 10)、第 0 与第 8 个关节正负"
             & (if Opp_0_8 then "恰好相反" else "不相反?") & "、第 0 与第 16 个" & (if Same_0_16 then "完全相同" else "不同?"));
   end;
   --  🔴 几个关节一起动时每个关节走多远(Jointboot.Multi_Offset):预计画面挪一格(640 宽的 1/5 = 128 px),不越过扫到过的那一头。
   --  扫描同驱动:头一格 0.03,下一格按"挪一格 ÷ 头一格量的每读数单位像素"放大、一次最多四倍 ——
   --  离眼远的关节(1000 px / 单位)三格走到 0.03 + 0.12 + 0.128 = 0.278 ⇒ 走 0.128(画面挪 128 px = 一格);离眼近的腕转(100 px / 单位)
   --  被"最多四倍"压住,三格只走到 0.03 + 0.12 + 0.48 = 0.63 ⇒ 走满 0.63(挪 63 px);这一边头一格没量到画面挪 ⇒ 走满扫到过的 0.09。
   --  牙:原来一律走到那一头的一半 ⇒ 远的挪 139 px(多一成)、近的只挪 31.5 px(一格的四分之一,扫到过的那一截白扔一半)
   declare
      Gw : constant Long_Float := 640.0 / 5.0;
      function Reach (Px : Long_Float) return Long_Float is   --  同驱动的三格:头一格 0.03,之后按画面放大、最多四倍、最少一半
         Step : Long_Float := 0.03;
         Sum : Long_Float := Step;
      begin
         for K in 2 .. 3 loop
            Step := Step * Long_Float'Max (0.5, Long_Float'Min (4.0, Gw / (Step * Px)));
            Sum := Sum + Step;
         end loop;
         return Sum;
      end Reach;
      Far : constant Long_Float := Jointboot.Multi_Offset (Gw, 1000.0, Reach (1000.0));
      Near : constant Long_Float := Jointboot.Multi_Offset (Gw, 100.0, Reach (100.0));
      Unmeasured : constant Long_Float := Jointboot.Multi_Offset (Gw, 0.0, 0.09);
      Old_Far : constant Long_Float := 0.5 * Reach (1000.0);
      Old_Near : constant Long_Float := 0.5 * Reach (100.0);
   begin
      Check (abs (Reach (1000.0) - 0.278) < 1.0e-9 and then abs (Reach (100.0) - 0.63) < 1.0e-9
             and then abs (1000.0 * Far - Gw) < 1.0e-9 and then abs (Near - Reach (100.0)) < 1.0e-12 and then Unmeasured = 0.09
             and then abs (1000.0 * Old_Far - Gw) > 0.05 * Gw and then Old_Near < 0.5 * Near + 1.0e-12,
             "几个关节一起动时每个关节走多远:离眼远的(1000 px/单位,扫到过 " & Codec.Fmt (Reach (1000.0), 3) & ")走 " & Codec.Fmt (Far, 3) & "、画面挪 "
             & Codec.Fmt (1000.0 * Far, 1) & " px(一格 " & Codec.Fmt (Gw, 0) & ");离眼近的腕转(100 px/单位,扫到过 " & Codec.Fmt (Reach (100.0), 2) & ")走满、挪 "
             & Codec.Fmt (100.0 * Near, 1) & " px;没量到画面挪的走满扫到过的 " & Codec.Fmt (Unmeasured, 2) & " · 牙:原来走一半 ⇒ 远的挪 "
             & Codec.Fmt (1000.0 * Old_Far, 1) & " px、近的只挪 " & Codec.Fmt (100.0 * Old_Near, 1) & " px");
   end;
   --  🔴 重挑内点到门里的那一组不再变(Jointboot.Until_Settled,对齐里两处精修走的那一条;09-30 C 组):一维位置,门 = 3 × 残差中位(中位同驱动:
   --  排好序取第 N/2 个),内点求平均,起步 = 全体平均。12 个内点等距铺在 [-1, 1]、8 个野点 4 × 1.3^k 一个比一个远 ⇒ 每遍剥掉一个:
   --  解 8 遍门里的那一组不再变、位置回到 0(真值);门里的那一组来回变 ⇒ 来回转(Cycled,2 遍);每遍都给一组没见过的(3 条观测)⇒
   --  解满 3 + 1 遍停(Capped);门里不够 ⇒ Too_Few、一遍不解。牙:原来固定三轮 ⇒ 停在 2.13(内点只铺在 [-1, 1]),再挑一遍门里的那一组还在变
   declare
      use type Jointboot.Settle_Verdict;
      package Sorting is new F64_Vectors.Generic_Sorting;
      Xs : Floats;
      Mu : Long_Float := 0.0;
      procedure Pick (U : out Bools; Enough : out Boolean) is
         Res : Floats;
         Gate : Long_Float;
         N_In : Natural := 0;
      begin
         for X of Xs loop
            Res.Append (abs (X - Mu));
         end loop;
         declare
            Srt : Floats := Res;
         begin
            Sorting.Sort (Srt);
            Gate := 3.0 * Srt (Natural (Srt.Length) / 2);
         end;
         U.Clear;
         for E of Res loop
            U.Append (E < Gate);
            if E < Gate then
               N_In := N_In + 1;
            end if;
         end loop;
         Enough := N_In > 0;
      end Pick;
      procedure Solve (U : Bools) is
         S : Long_Float := 0.0;
         N_In : Natural := 0;
      begin
         for I in 0 .. Natural (Xs.Length) - 1 loop
            if U (I) then
               S := S + Xs (I); N_In := N_In + 1;
            end if;
         end loop;
         Mu := S / Long_Float (N_In);
      end Solve;
      procedure Run is new Jointboot.Until_Settled (Pick, Solve);
      Calls : Natural := 0;
      --  来回转:不管解成什么,单数遍给一组、双数遍给另一组
      procedure Pick_Alt (U : out Bools; Enough : out Boolean) is
      begin
         Calls := Calls + 1;
         U.Clear; U.Append (Calls mod 2 = 1); U.Append (True); U.Append (Calls mod 2 = 0);
         Enough := True;
      end Pick_Alt;
      procedure Solve_None (U : Bools) is null;
      procedure Run_Alt is new Jointboot.Until_Settled (Pick_Alt, Solve_None);
      --  每遍一组没见过的(3 条观测的 8 种按二进制顺着给)
      procedure Pick_New (U : out Bools; Enough : out Boolean) is
      begin
         Calls := Calls + 1;
         U.Clear;
         for B in 0 .. 2 loop
            U.Append ((Calls / 2 ** B) mod 2 = 1);
         end loop;
         Enough := True;
      end Pick_New;
      procedure Run_New is new Jointboot.Until_Settled (Pick_New, Solve_None);
      procedure Pick_Few (U : out Bools; Enough : out Boolean) is
      begin
         U.Clear; U.Append (False);
         Enough := False;
      end Pick_Few;
      procedure Run_Few is new Jointboot.Until_Settled (Pick_Few, Solve_None);
      function Mean_All return Long_Float is
         S : Long_Float := 0.0;
      begin
         for X of Xs loop
            S := S + X;
         end loop;
         return S / Long_Float (Xs.Length);
      end Mean_All;
      Rounds, R_Alt, R_New, R_Few : Natural;
      V, V_Alt, V_New, V_Few : Jointboot.Settle_Verdict;
      Mu3 : Long_Float;
      Still : Boolean;
   begin
      for K in 0 .. 11 loop
         Xs.Append (-1.0 + 2.0 * Long_Float (K) / 11.0);
      end loop;
      for K in 0 .. 7 loop
         Xs.Append (4.0 * 1.3 ** K);
      end loop;
      --  牙:原来那样固定三轮
      declare
         U, U4 : Bools;
         En : Boolean;
      begin
         Mu := Mean_All;
         for Round in 1 .. 3 loop
            Pick (U, En);
            Solve (U);
         end loop;
         Mu3 := Mu;
         Pick (U4, En);
         Still := not Bool_Vectors."=" (U, U4);
      end;
      Mu := Mean_All;
      Run (Rounds, V);
      Calls := 0;
      Run_Alt (R_Alt, V_Alt);
      Calls := 0;
      Run_New (R_New, V_New);
      Run_Few (R_Few, V_Few);
      Check (V = Jointboot.Settled and then Rounds = 8 and then abs Mu < 1.0e-12 and then V_Alt = Jointboot.Cycled and then R_Alt = 2
             and then V_New = Jointboot.Capped and then R_New = 4 and then V_Few = Jointboot.Too_Few and then R_Few = 0
             and then Still and then abs (Mu3 - 2.1278) < 1.0e-3,
             "重挑内点到不再变:12 个内点 + 8 个一个比一个远的野点 ⇒ 解 " & Codec.Img (Rounds) & " 遍定下来(" & V'Image & ",要 8 遍)、位置 " & Codec.Fmt (Mu, 6)
             & "(真 0)· 来回变 ⇒ " & V_Alt'Image & "(" & Codec.Img (R_Alt) & " 遍)· 每遍一组新的 ⇒ " & V_New'Image & "(" & Codec.Img (R_New)
             & " 遍 = 3 条观测 + 1)· 门里不够 ⇒ " & V_Few'Image & " · 牙:原来固定三轮 ⇒ 停在 " & Codec.Fmt (Mu3, 3) & "、"
             & (if Still then "再挑一遍门里的那一组还在变" else "(再挑一遍没变?)"));
   end;
   --  🔴 碰到没有(Selfmap.Blocked;09-29 台架,V1B66 满精度的数):
   --  ① V1B66 第 2 只手头一下的轻碰:第一档空走少走 0.00086、第二档已经压着 0.00578 ⇒ 第二档认出(牙:旧的"前两档平均当底、两档之差当抖动"
   --     ⇒ 底 0.00332、门 0.0181,后面 9 档最多 0.01266,一档都认不出);② 空走的小步差一丝(0.001430 → 0.001439)⇒ 不认(牙:旧的
   --     "第一步 + 3 × 静止噪声 0"⇒ 认成碰到,V1B60 / V1B65 的虚认);③ 碰上的小步 0.00718(空走 0.00163)⇒ 认出;④ 第一步没有可比的 ⇒ 不认;
   --  ⑤ 读数噪声 1e-3 ⇒ 多少走 2e-3 不认、4e-3 认;⑥ 前两步空走之差 0.0006 ⇒ 门跟着放宽
   declare
      Old_Base : constant Long_Float := 0.5 * (0.00086 + 0.00578);
      Old_Gate : constant Long_Float := Old_Base + 3.0 * abs (0.00578 - 0.00086);
      Later : constant array (1 .. 9) of Long_Float := [0.00971, 0.01053, 0.01110, 0.01163, 0.01191, 0.01218, 0.01239, 0.01255, 0.01266];
      Old_Miss : Boolean := True;
   begin
      for X of Later loop
         if X > Old_Gate then
            Old_Miss := False;
         end if;
      end loop;
      Check (Selfmap.Blocked (0.00578, 0.00086, 0.0, 1, 0.0135, 0.0) and then Old_Miss
             and then not Selfmap.Blocked (0.001439, 0.001430, 0.001425, 2, 0.054, 0.0) and then 0.001439 > 0.001430
             and then Selfmap.Blocked (0.00718, 0.00163, 0.00160, 2, 0.054, 0.0)
             and then not Selfmap.Blocked (0.5, 0.0, 0.0, 0, 0.054, 0.0)
             and then not Selfmap.Blocked (0.0036, 0.0016, 0.0016, 2, 0.054, 1.0e-3) and then Selfmap.Blocked (0.0056, 0.0016, 0.0016, 2, 0.054, 1.0e-3)
             and then not Selfmap.Blocked (0.0030, 0.0016, 0.0010, 2, 0.054, 0.0) and then Selfmap.Blocked (0.0040, 0.0016, 0.0010, 2, 0.054, 0.0),
             "碰到没有(Blocked):V1B66 轻碰第二档已压着 0.00578 ⇒ 认出(旧的两档平均当底 ⇒ 门 " & Codec.Fmt (Old_Gate, 5) & ",后面 9 档都认不出);"
             & "空走差一丝 0.001430 → 0.001439 ⇒ 不认(旧的门 = 第一步 ⇒ 认成碰到);碰上的小步 0.00718 ⇒ 认出;第一步不判;噪声大、空走抖得大 ⇒ 门跟着放宽");
   end;
   --  🔴 Settle(一条命令从发出到读数停住用了几拍)是量出来的,不缺省、不封顶(Selfmap.Settle_Beats / Settle_Since,09-30):
   --  ① 慢的手(命令隔两拍才起效、慢慢收尾,读数噪声 0.001):每拍挪动 0 0 0.05 0.03 0.012 0.004 0.0008 0.0009 ⇒ 第 8 拍停住(挪动到了噪声以内、不再变小);
   --     牙:原来开机前半段从来不量、恒为缺省 2,后半段取 Go 用的拍数又夹在 6 拍以内 ⇒ 2 / 6,都比真的 8 少(慢身体一条命令没走完就被当成停了);
   --  ② V1B65 录下的 x5 探针(每拍挪动 9e-5、1.1e-5、2e-6,静止噪声不到 5e-7):一路还在变小 ⇒ 还没停住 ⇒ 0(这一条量不出,不拿"看了几拍"顶);
   --  ③ 一直没动起来 ⇒ 0;④ Settle_Since 只看发命令那一拍(帧号 From_Seq)以后记下的拍,之前那一拍的大挪动不算
   declare
      type Lf_Array is array (Positive range <>) of Long_Float;
      function Fl (A : Lf_Array) return Bytes.Floats is
         R : Bytes.Floats;
      begin
         for X of A loop
            R.Append (X);
         end loop;
         return R;
      end Fl;
      Slow : constant Lf_Array := [0.0, 0.0, 0.05, 0.03, 0.012, 0.004, 0.0008, 0.0009];
      S1 : constant Natural := Selfmap.Settle_Beats (Fl (Slow), 0.001);
      S2 : constant Natural := Selfmap.Settle_Beats (Fl ([9.0e-5, 1.1e-5, 2.0e-6]), 5.0e-7);
      S3 : constant Natural := Selfmap.Settle_Beats (Fl ([0.0, 0.0, 0.0, 0.0]), 0.001);
      Old_First : constant Natural := 2;                             --  原来的缺省(前半段从来不量)
      Old_Second : constant Natural := Natural'Min (Slow'Length, 6);   --  原来后半段:Go 用的拍数、夹在 6 以内
      Lk : Plug.Link;
      S4 : Natural;
   begin
      for I in 0 .. Slow'Length loop
         declare
            B : Plug.Beat;
         begin
            B.Seq := 10 + I;
            B.Q_Chg.Append (if I = 0 then 0.3 else Slow (I));   --  帧号 10 = 发命令之前那一拍,挪了一大截(上一条命令的尾巴)
            B.Q_Chg.Append (0.0);                               --  另一组一动不动
            Lk.Beats.Append (B);
         end;
      end loop;
      S4 := Selfmap.Settle_Since (Lk, 10, 0.001);
      Check (S1 = 8 and then S2 = 0 and then S3 = 0 and then S4 = 8 and then Old_First < S1 and then Old_Second < S1,
             "Settle 量出来:慢的手 ⇒ 第 " & Codec.Img (S1) & " 拍停住(要 8)· x5 探针还在变小 ⇒ " & Codec.Img (S2) & "(量不出,要 0)· 没动起来 ⇒ "
             & Codec.Img (S3) & " · 按帧号从发命令那一拍往后数 ⇒ " & Codec.Img (S4) & "(要 8)· 牙:原来前半段 " & Codec.Img (Old_First)
             & "、后半段夹到 " & Codec.Img (Old_Second) & ",都比 8 少");
   end;
   --  🔴 等画面停稳照实说停没停(Selfmap.Wait_Still,09-30):锁步里一只假手等,主线程当假身体 —— 32×24 的画面里一块 8×8 的亮块:
   --  一直在挪(每拍 2 像素)⇒ 等满 12 拍、Ok = False、Used = 12(牙:原来超时照样 Ok = True —— 握区就在还在动的画面上量);
   --  挪三拍就停 ⇒ Ok = True、第 5 拍就停稳(没等满)。判得了的相机才算数(Pictures_Still):后一帧没收到画面(占位)⇒ 判不了 ⇒ 不说静止;
   --  两帧都是占位 ⇒ 一台都判不了 ⇒ 不说静止(牙:原来空画面比出来一个动的像素都没有 ⇒ 当成静止)
   declare
      W : constant := 32;
      H : constant := 24;
      Lk : Plug.Link;
      Mp : Selfmap.Body_Map;
      Fr0 : Plug.Frame;
      Max_W : constant := 12;
      Used_W : Natural := 0;
      Ok_W : Boolean := True;
      function Pic (X0 : Natural) return Plug.Cam is
         C : Plug.Cam;
      begin
         C.W := W; C.H := H;
         for I in 0 .. W * H - 1 loop
            C.Gray.Append (if I mod W in X0 .. X0 + 7 and then I / W in 8 .. 15 then 200 else 30);
         end loop;
         return C;
      end Pic;
      task type Wait_Hand;
      task body Wait_Hand is
         Fr : Plug.Frame := Fr0;
      begin
         Lockstep.Begin_Hand (0);
         Selfmap.Wait_Still (Lk, Mp, Fr, Max_W, Used_W, Ok_W);
         Lockstep.Done;
      end Wait_Hand;
      --  假身体:前 Moving 拍每拍把亮块往右挪 2 像素,之后停住
      procedure Run (Moving : Natural) is
         Beat : Natural := 0;
      begin
         Fr0 := (others => <>);
         Fr0.Cams.Append (Pic (0));
         Lockstep.Clear;
         Plug.Lock_Begin;
         declare
            Hd : Wait_Hand;
         begin
            Lockstep.Start (0, Hd'Identity);
            loop
               Lockstep.Run (0);
               exit when Lockstep.Finished (0);
               Beat := Beat + 1;
               declare
                  Ff : Plug.Frame;
               begin
                  Ff.Cams.Append (Pic (2 * Natural'Min (Beat, Moving)));
                  Plug.Lock_Feed (Ff);
               end;
            end loop;
         end;
         Plug.Lock_End;
         Lockstep.Clear;
      end Run;
      Ok_Moving, Ok_Settle : Boolean;
      Used_Moving, Used_Settle : Natural;
      Hole : constant Plug.Cam := (others => <>);
      Bf, Af, Hf : Plug.Cam_Vectors.Vector;
      Old_Blank : Natural := 0;   --  原来:拿空画面比,超过地板的像素一个都没有
   begin
      Mp.Floors.Append (Picture.Null_Floor (Pic (0).Gray, Pic (0).Gray, W, H, Picture.Min_Pixels (W, H)));
      Run (Max_W + 1);
      Ok_Moving := Ok_W; Used_Moving := Used_W;
      Run (3);
      Ok_Settle := Ok_W; Used_Settle := Used_W;
      Bf.Append (Pic (0)); Af.Append (Hole); Hf.Append (Hole);
      for B of Picture.Moved (Pic (0).Gray, Hole.Gray, Mp.Floors (0)) loop
         if B then
            Old_Blank := Old_Blank + 1;
         end if;
      end loop;
      Check (not Ok_Moving and then Used_Moving = Max_W and then Ok_Settle and then Used_Settle < Max_W
             and then Selfmap.Pictures_Still (Mp, Bf, Bf) and then not Selfmap.Pictures_Still (Mp, Bf, Af) and then not Selfmap.Pictures_Still (Mp, Hf, Af)
             and then Old_Blank = 0,
             "等画面停稳:一直在挪 ⇒ 等满 " & Codec.Img (Used_Moving) & " 拍、" & (if Ok_Moving then "说停了(错)" else "照实说没停稳")
             & " · 挪三拍就停 ⇒ " & Codec.Img (Used_Settle) & " 拍停稳 · 后一帧没收到画面 ⇒ 不说静止 · 两帧都没收到 ⇒ 不说静止"
             & " · 牙:原来超时照样说停了;空画面比出来动的像素 " & Codec.Img (Old_Blank) & " 个 ⇒ 当成静止");
   end;
   --  🔴 抓握通道带不带手指是量出来的(Act.Has_Fingers;09-28 DR1 / DR2:无人机开机说了"握区量不了",干活时照样列两瓣手指一组爪心):
   --  一条臂一个抓握通道,两台相机的握区都没量成 ⇒ 没手指;其中一台量成 ⇒ 有;两条臂只有第 2 条量成 ⇒ 第 1 条没有、第 2 条有、整具有
   declare
      C : Act.Context;
      Hd : Zone.Hand;
      Zv : Zone.Hand_Zone;
      None_Ok, One_Ok, Two_Arms_Ok : Boolean;
   begin
      C.Map.Arms := 1;
      C.Map.Jaws.Append (1);
      Hd.Arm := 0; Hd.K := 0;
      Hd.Zones.Append (Zone.Hand_Zone'(others => <>));
      Hd.Zones.Append (Zone.Hand_Zone'(others => <>));
      C.Hands.Append (Hd);
      None_Ok := not Act.Has_Fingers (C, 0) and then not Act.Arm_Has_Fingers (C, 0) and then not Act.Any_Fingers (C);
      Zv.Valid := True;
      C.Hands (0).Zones.Replace_Element (1, Zv);
      One_Ok := Act.Has_Fingers (C, 0) and then Act.Any_Fingers (C);
      C.Map.Arms := 2;
      C.Map.Jaws.Append (1);
      C.Hands (0).Zones.Replace_Element (1, Zone.Hand_Zone'(others => <>));
      Hd.Arm := 1;
      Hd.Zones.Replace_Element (0, Zv);
      C.Hands.Append (Hd);
      Two_Arms_Ok := not Act.Arm_Has_Fingers (C, 0) and then Act.Arm_Has_Fingers (C, 1) and then Act.Any_Fingers (C);
      Check (None_Ok and then One_Ok and then Two_Arms_Ok,
             "抓握通道带不带手指按量的:哪台相机都没量出握区 ⇒ 没有(" & Boolean'Image (None_Ok) & ")· 一台量出 ⇒ 有(" & Boolean'Image (One_Ok)
             & ")· 两条臂只有第 2 条量出 ⇒ 分得开(" & Boolean'Image (Two_Arms_Ok) & ")");
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
   --  🔴 顶住的判断看五样(09-30):Free_Res / Null_Res 原来只加前三样(u、v、远近)—— 09-08 表加到五行时别处的 0 .. 2 都改了,Norm3 不是循环、漏了。
   --  一步只在"看着多大"上和表对不上(推过去它该变大、实际没变 = 顶住了),零表永远赢不了。同一组数:新的两步判顶住;旧的只加前三样,残差是 0
   declare
      E : Table.Effect;
      Cmd : Table.Vec := Table.Zero_Vec;
      Old_Res : Long_Float := -1.0;
   begin
      Table.Reset (E, 1, 1.0e-9);                          --  先验极小:这几步几乎改不动表,只看判据
      Table.Set_Col (E, 0, [0.0, 0.0, 0.0, 0.5, 0.0]);     --  这个通道只改"看着多大"
      Cmd (0) := 1.0;
      for K in 1 .. 3 loop
         Table.Update (E, Cmd, [0.0, 0.0, 0.0, 0.0, 0.0], 0.01, 0.0);   --  推了,画面上它一点没变
      end loop;
      declare
         Miss : constant Table.Vec3 := Table.Predict (E, Cmd);   --  表说该变多少(实际 0)
      begin
         Old_Res := Ada.Numerics.Long_Elementary_Functions.Sqrt (Miss (0) ** 2 + Miss (1) ** 2 + Miss (2) ** 2);
      end;
      Check (Table.Blocked (E) and then E.Free_Res > 0.4 and then Old_Res = 0.0,
             "顶住看五样:只在""看着多大""上对不上也判顶住(走的表差 " & Codec.Fmt (E.Free_Res, 3) & ";旧写法只加前三样,差 " & Codec.Fmt (Old_Res, 3) & " ⇒ 永远不判)");
   end;
   --  🔴 响应表的天花板(09-30):要的通道比 Max_Ch 多 ⇒ 当场报(Pre);原来 Reset / Norm / Solve 用 Natural'Min 悄悄截成前 64 个
   declare
      E : Table.Effect;
      pragma Warnings (Off, "*is not modified*");   --  易失:不让编译器提前判出"前提不成立",要在运行时真调一次
      N : Natural := Table.Max_Ch + 1 with Volatile;
      pragma Warnings (On, "*is not modified*");
      Raised : Boolean := False;
   begin
      begin
         Table.Reset (E, N, 1.0);
      exception
         when Ada.Assertions.Assertion_Error => Raised := True;
      end;
      Check (Raised and then Natural'Min (N, Table.Max_Ch) /= N,
             "响应表要 " & Codec.Img (N) & " 个通道(天花板 " & Codec.Img (Table.Max_Ch) & ")⇒ 当场报(旧写法 Natural'Min 悄悄只留前 " & Codec.Img (Table.Max_Ch) & " 个)");
   end;
   --  🔴 带硬约束的解(09-30):硬约束行正交化成 Q ——
   --  ① 原来 Q 只开 Rows × 8 = 40 行,硬约束的秩比 40 大时多出来的方向悄悄不收(不受保护);现在开到通道数(秩的上限);
   --  ② 原来"剩下的长度 > 0 就收":和已有方向线性相关的行剩下的是舍入误差,也被归一成一个乱方向 ⇒ P = I − QᵀQ 不再是投影,软约束那一步会挪动硬约束已经达成的行;
   --     现在按数值秩的标准门收、正交化做两遍。两组数:48 个通道 × 50 行硬约束(秩 48 > 40)、6 个通道里同一条硬约束写了两遍再加一条它们的组合。
   --  判:解出来的 A 和只解硬约束的 A1 比,硬约束那几行一点没动;旧写法的复刻(一遍、> 0 就收、最多 40 行)同一组数算出的 P 漏掉硬约束方向
   declare
      Seed : Long_Long_Integer := 20260930;
      function Rnd return Long_Float is
      begin
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Seed mod 2001) / 1000.0 - 1.0;
      end Rnd;
      --  旧写法的复刻:一遍格拉姆-施密特、剩下的长度 > 0 就收、最多收 Cap_Rows 行;返回 max |硬约束行 × P| —— 真投影下这是 0
      function Old_Leak (Hard : Table.Term_Vectors.Vector; N, Cap_Rows : Natural) return Long_Float is
         Q : array (0 .. Cap_Rows - 1, 0 .. N - 1) of Long_Float := [others => [others => 0.0]];
         NQ : Natural := 0;
         P : array (0 .. N - 1, 0 .. N - 1) of Long_Float := [others => [others => 0.0]];
         Worst : Long_Float := 0.0;
      begin
         for T of Hard loop
            for R in 0 .. Table.Rows - 1 loop
               if T.W (R) > 0.0 and then NQ < Cap_Rows then
                  declare
                     V : array (0 .. N - 1) of Long_Float;
                     Nm : Long_Float := 0.0;
                  begin
                     for C in 0 .. N - 1 loop
                        V (C) := T.E.B (C, R);
                     end loop;
                     for K in 0 .. NQ - 1 loop
                        declare
                           Dt : Long_Float := 0.0;
                        begin
                           for C in 0 .. N - 1 loop
                              Dt := Dt + V (C) * Q (K, C);
                           end loop;
                           for C in 0 .. N - 1 loop
                              V (C) := V (C) - Dt * Q (K, C);
                           end loop;
                        end;
                     end loop;
                     for C in 0 .. N - 1 loop
                        Nm := Nm + V (C) ** 2;
                     end loop;
                     Nm := Ada.Numerics.Long_Elementary_Functions.Sqrt (Nm);
                     if Nm > 0.0 then
                        for C in 0 .. N - 1 loop
                           Q (NQ, C) := V (C) / Nm;
                        end loop;
                        NQ := NQ + 1;
                     end if;
                  end;
               end if;
            end loop;
         end loop;
         for I in 0 .. N - 1 loop
            P (I, I) := 1.0;
         end loop;
         for K in 0 .. NQ - 1 loop
            for I in 0 .. N - 1 loop
               for J in 0 .. N - 1 loop
                  P (I, J) := P (I, J) - Q (K, I) * Q (K, J);
               end loop;
            end loop;
         end loop;
         for T of Hard loop
            for R in 0 .. Table.Rows - 1 loop
               if T.W (R) > 0.0 then
                  for J in 0 .. N - 1 loop
                     declare
                        S : Long_Float := 0.0;
                     begin
                        for C in 0 .. N - 1 loop
                           S := S + T.E.B (C, R) * P (C, J);
                        end loop;
                        Worst := Long_Float'Max (Worst, abs S);
                     end;
                  end loop;
               end if;
            end loop;
         end loop;
         return Worst;
      end Old_Leak;
      --  新写法:解出来的 A 比只解硬约束的 A1,硬约束那几行挪了多少
      procedure New_Leak (Hard, Soft : Table.Term_Vectors.Vector; N : Natural; Soft_Has_Room : Boolean; Moved : out Long_Float; Ok : out Boolean) is
         Cap : constant Table.Vec := [others => 1.0e6];
         Act : Table.Mask := [others => False];
         Damp : constant Table.Vec := [others => 1.0e-9];
         A, A1 : Table.Vec;
         Ok1 : Boolean;
      begin
         for C in 0 .. N - 1 loop
            Act (C) := True;
         end loop;
         Table.Solve (Hard, N, Cap, Act, Damp, A1, Ok1);
         Table.Solve_Priority (Hard, Soft, N, Cap, Act, Damp, A, Ok);
         Ok := Ok and then Ok1;
         Moved := 0.0;
         for T of Hard loop
            for R in 0 .. Table.Rows - 1 loop
               if T.W (R) > 0.0 then
                  declare
                     S : Long_Float := 0.0;
                  begin
                     for C in 0 .. N - 1 loop
                        S := S + T.E.B (C, R) * (A (C) - A1 (C));
                     end loop;
                     Moved := Long_Float'Max (Moved, abs S);
                  end;
               end if;
            end loop;
         end loop;
         --  硬约束没占满自由度时,软约束也得真的被用上(不然"硬的没动"是白给的):解和 A1 不一样;
         --  秩满(48 个通道 × 50 行)时软约束没有剩下的自由度,A = A1 才是对的
         if Soft_Has_Room then
            Ok := Ok and then (for some C in 0 .. N - 1 => abs (A (C) - A1 (C)) > 1.0e-6);
         end if;
      end New_Leak;
      function Random_Term (N : Natural; Rows_On : Natural) return Table.Term is
         T : Table.Term;
      begin
         Table.Reset (T.E, N, 1.0);
         for C in 0 .. N - 1 loop
            declare
               Col : Table.Vec3;
            begin
               for R in 0 .. Table.Rows - 1 loop
                  Col (R) := Rnd;
               end loop;
               Table.Set_Col (T.E, C, Col);
            end;
         end loop;
         for R in 0 .. Table.Rows - 1 loop
            T.Err (R) := Rnd;
            T.W (R) := (if R < Rows_On then 1.0 else 0.0);
         end loop;
         return T;
      end Random_Term;
      Hard48, Soft48, Hard6, Soft6 : Table.Term_Vectors.Vector;
      Moved48, Moved6, Leak48, Leak6 : Long_Float;
      Ok48, Ok6 : Boolean;
   begin
      for K in 1 .. 10 loop
         Hard48.Append (Random_Term (48, Table.Rows));
      end loop;
      Soft48.Append (Random_Term (48, Table.Rows));
      declare
         T1 : constant Table.Term := Random_Term (6, 2);
         T3 : Table.Term := T1;
      begin
         Hard6.Append (T1);
         Hard6.Append (T1);   --  同一条硬约束写了两遍
         for C in 0 .. 5 loop
            T3.E.B (C, 0) := 0.37 * T1.E.B (C, 0) + 1.3 * T1.E.B (C, 1);   --  再加一条它们的组合
         end loop;
         T3.W := [1.0, 0.0, 0.0, 0.0, 0.0];
         Hard6.Append (T3);
      end;
      Soft6.Append (Random_Term (6, 3));
      New_Leak (Hard48, Soft48, 48, False, Moved48, Ok48);
      New_Leak (Hard6, Soft6, 6, True, Moved6, Ok6);
      Leak48 := Old_Leak (Hard48, 48, Table.Rows * 8);
      Leak6 := Old_Leak (Hard6, 6, Table.Rows * 8);
      Check (Ok48 and then Moved48 < 1.0e-9 and then Leak48 > 1.0e-3,
             "硬约束秩 48 > 40:软约束那一步硬约束行挪了 " & Codec.Fmt (Moved48, 12) & "(旧写法 Q 只收 40 行,P 漏掉的硬约束方向 " & Codec.Fmt (Leak48, 3) & ")");
      Check (Ok6 and then Moved6 < 1.0e-9 and then Leak6 > 1.0e-3,
             "同一条硬约束写两遍 + 一条组合:软约束那一步硬约束行挪了 " & Codec.Fmt (Moved6, 12) & "(旧写法把舍入误差归一成乱方向,P 漏掉 " & Codec.Fmt (Leak6, 3) & ")");
   end;
   --  🔴 有上下限的解做到不再变(09-30):越限的夹住、固定、再解,原来固定做 3 遍 —— 第 3 遍还在夹就交出去,后夹的那个的贡献没分给剩下的通道。
   --  离线搜出来的一组 4 通道(每遍多夹一个,要 4 遍):新写法剩下那个自由通道的梯度是 0;旧写法的复刻做 3 遍,同一组数那个通道差 0.14
   declare
      type V4 is array (0 .. 3) of Long_Float;
      type M4 is array (0 .. 3, 0 .. 3) of Long_Float;
      Bs : constant array (0 .. 3) of Table.Vec3 :=
        [[-0.75, -0.25, -0.75, 1.0, 0.0], [0.0, 0.25, -0.5, 0.25, 0.0], [0.25, 0.75, 0.0, 0.25, 0.0], [0.25, 0.25, 0.0, 0.5, 0.0]];
      Err : constant Table.Vec3 := [0.0, -1.75, 0.5, -1.5, 0.0];
      Dmp : constant Long_Float := 1.0e-9;
      G : M4 := [others => [others => 0.0]];
      H : V4 := [others => 0.0];
      --  旧写法的复刻:同一个 G、h,越限的夹住、固定、再解,最多做 Max_R 遍
      function Clamp_Solve (Max_R : Natural) return V4 is
         A : V4 := [others => 0.0];
         Fixed : array (0 .. 3) of Boolean := [others => False];
      begin
         for Round in 1 .. Max_R loop
            declare
               Idx : array (0 .. 3) of Natural := [others => 0];
               M : Natural := 0;
               S : M4 := [others => [others => 0.0]];
               Rhs, X : V4 := [others => 0.0];
               Any : Boolean := False;
            begin
               for I in 0 .. 3 loop
                  if not Fixed (I) then
                     Idx (M) := I; M := M + 1;
                  end if;
               end loop;
               exit when M = 0;
               for P in 0 .. M - 1 loop
                  Rhs (P) := H (Idx (P));
                  for J in 0 .. 3 loop
                     if Fixed (J) then
                        Rhs (P) := Rhs (P) - G (Idx (P), J) * A (J);
                     end if;
                  end loop;
                  for Q in 0 .. M - 1 loop
                     S (P, Q) := G (Idx (P), Idx (Q));
                  end loop;
               end loop;
               for C in 0 .. M - 1 loop
                  for R in C + 1 .. M - 1 loop
                     declare
                        F : constant Long_Float := S (R, C) / S (C, C);
                     begin
                        for Q in C .. M - 1 loop
                           S (R, Q) := S (R, Q) - F * S (C, Q);
                        end loop;
                        Rhs (R) := Rhs (R) - F * Rhs (C);
                     end;
                  end loop;
               end loop;
               for R in reverse 0 .. M - 1 loop
                  declare
                     Sm : Long_Float := Rhs (R);
                  begin
                     for Q in R + 1 .. M - 1 loop
                        Sm := Sm - S (R, Q) * X (Q);
                     end loop;
                     X (R) := Sm / S (R, R);
                  end;
               end loop;
               for P in 0 .. M - 1 loop
                  if X (P) > 1.0 then
                     A (Idx (P)) := 1.0; Fixed (Idx (P)) := True; Any := True;
                  elsif X (P) < -1.0 then
                     A (Idx (P)) := -1.0; Fixed (Idx (P)) := True; Any := True;
                  else
                     A (Idx (P)) := X (P);
                  end if;
               end loop;
               exit when not Any;
            end;
         end loop;
         return A;
      end Clamp_Solve;
      E : Table.Effect;
      T : Table.Term;
      Terms : Table.Term_Vectors.Vector;
      Cap : Table.Vec := Table.Zero_Vec;
      Act : Table.Mask := [others => False];
      A : Table.Vec;
      Ok : Boolean;
      Old3, Full : V4;
      Grad_New, Grad_Old : Long_Float := 0.0;
   begin
      Table.Reset (E, 4, 1.0);
      for C in 0 .. 3 loop
         Table.Set_Col (E, C, Bs (C));
         Cap (C) := 1.0; Act (C) := True;
         for D in 0 .. 3 loop
            for R in 0 .. Table.Rows - 1 loop
               G (C, D) := G (C, D) + Bs (C) (R) * Bs (D) (R);
            end loop;
         end loop;
         G (C, C) := G (C, C) + Dmp;
         for R in 0 .. Table.Rows - 1 loop
            H (C) := H (C) + Bs (C) (R) * Err (R);
         end loop;
      end loop;
      T.E := E; T.Err := Err; T.W := [others => 1.0];
      Terms.Append (T);
      Table.Solve (Terms, 4, Cap, Act, [others => Dmp], A, Ok);
      Old3 := Clamp_Solve (3);
      Full := Clamp_Solve (5);
      for D in 0 .. 3 loop
         Grad_New := Grad_New + G (0, D) * A (D);
         Grad_Old := Grad_Old + G (0, D) * Old3 (D);
      end loop;
      Grad_New := Grad_New - H (0); Grad_Old := Grad_Old - H (0);
      Check (Ok and then (for all C in 0 .. 3 => abs (A (C) - Full (C)) < 1.0e-9) and then abs A (0) < 1.0 and then abs Grad_New < 1.0e-9
             and then abs (Old3 (0) - A (0)) > 0.05 and then abs Grad_Old > 0.1,
             "有上下限的解做到不再变:第 4 遍夹住最后一个越限的,剩下的自由通道 " & Codec.Fmt (A (0), 4) & " 梯度 " & Codec.Fmt (Grad_New, 9)
             & "(旧写法 3 遍就交:" & Codec.Fmt (Old3 (0), 4) & ",梯度 " & Codec.Fmt (Grad_Old, 3) & ")");
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
   --  🔴 判哪头张开时绕世界竖直轴转的那一下(Zone.Turn_Step,09-30):步子按推的那个通道(绕 z,第 5 个)自己的探针幅度定 ——
   --  三个转动通道的探针幅度 0.001 / 0.002 / 0.004 ⇒ 只推第 5 个、64 × 0.004;牙:原来按第 3 个(绕 x)的幅度定步子、推的却是第 5 个 ⇒ 64 × 0.001
   declare
      Mz : Selfmap.Body_Map;
      Tv : Table.Vec;
      Old_Step : Long_Float;
   begin
      Mz.Per_Arm := Chan.Per_Arm;
      Mz.Amp := Bytes.Zeros (2 * Chan.Per_Arm);
      Mz.Amp.Replace_Element (Chan.Per_Arm + 3, 0.001); Mz.Amp.Replace_Element (Chan.Per_Arm + 4, 0.002); Mz.Amp.Replace_Element (Chan.Per_Arm + 5, 0.004);
      Tv := Zone.Turn_Step (Mz, 1);
      Old_Step := 64.0 * Mz.Amp (Chan.Per_Arm + 3);
      Check (abs (Tv (5) - 64.0 * 0.004) < 1.0e-12 and then Tv (3) = 0.0 and then Tv (4) = 0.0 and then Tv (0) = 0.0 and then Tv (1) = 0.0 and then Tv (2) = 0.0
             and then abs (Old_Step - Tv (5)) > 1.0e-6
             and then (for all K in Tv'Range => Zone.Turn_Step (Mz, 2) (K) = 0.0),
             "绕竖直轴转的那一下:第 2 只手三个转动探针 0.001 / 0.002 / 0.004 ⇒ 只推绕 z 的那个 " & Codec.Fmt (Tv (5), 3) & "(= 64 × 0.004)"
             & " · 没量过的手 ⇒ 不转 · 牙:原来按绕 x 的幅度定步子 ⇒ " & Codec.Fmt (Old_Step, 3));
   end;
   --  🔴 抓握通道推一下等它停住:等满了还在走就照实说没停住,不当"推到这儿停了"(Zone.Measure 里的 Go_Jaw,09-30):
   --  锁步里一只假手真跑 Zone.Measure,主线程当假身体 —— 抓握读数每拍只朝目标走 0.01(慢的手:从 0.5 推到 -0.5 要 100 拍),画面不动。
   --  新:往小那边推的第一下等满 40 拍读数还在走 ⇒ 没停住 ⇒ 这一头量不出、握区照实说量不了(Ok = False,没记下哪一头);
   --  牙:原来等满 40 拍照样 Good = True ⇒ 那时的读数 0.1 被当成"推到这儿停了",挪了 0.4 < 命令 1.0 的一半 ⇒ 判成到头,把 0.1 当成合空那一头(真的一头还在 -0.5 往外)
   declare
      W : constant := 16;
      H : constant := 12;
      Lk : Plug.Link;
      Mz : Selfmap.Body_Map;
      Fr0 : Plug.Frame;
      Hz : Zone.Hand;
      Ok_Z : Boolean := True;
      Jaw_Now : Long_Float := 0.5;
      Jaw_Tgt : Long_Float := 0.5;
      Beats : Natural := 0;
      At_40 : Long_Float := 0.0;   --  第一下推出去第 40 拍时的读数(原来就拿它当"推到这儿停了")
      Push_Start : Natural := 0;   --  第一下推是第几拍发的
      Pic : Plug.Cam;
      procedure Fake_Cmd (C : in out Plug.Cmd; Ok : out Boolean) is
      begin
         if not C.Jaw.Is_Empty then
            Jaw_Tgt := C.Jaw (0);
         end if;
         C.Kind := Plug.Joint; C.Q := Bytes.F64_Vectors.Empty_Vector;
         Ok := True;
      end Fake_Cmd;
      task type Zone_Hand;
      task body Zone_Hand is
         Fr : Plug.Frame := Fr0;
      begin
         Lockstep.Begin_Hand (0);
         Zone.Measure (Lk, Mz, 0, 0, Fr, Hz, Ok_Z, Host => "", Port => 0, Eyes => Geom.Geo_Vectors.Empty_Vector);   --  判哪头张开之前就停(慢的手没停住),用不着配点仪器
         Lockstep.Done;
      end Zone_Hand;
      function Frame_Now return Plug.Frame is
         Ff : Plug.Frame;
      begin
         Ff.EE.Append (Plug.Arm_Pose'[0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]);
         Ff.Jaw.Append (Bytes.F64_Vectors.To_Vector (Jaw_Now, 1));
         Ff.Cams.Append (Pic);
         return Ff;
      end Frame_Now;
      Old_End : Boolean;
   begin
      Pic.W := W; Pic.H := H;
      for I in 0 .. W * H - 1 loop
         Pic.Gray.Append (U8 (40 + I mod 7));
      end loop;
      Mz.Floors.Append (Picture.Null_Floor (Pic.Gray, Pic.Gray, W, H, Picture.Min_Pixels (W, H)));
      Mz.Jaw_Noise := 1.0e-6; Mz.Per_Arm := Chan.Per_Arm; Mz.Amp := Bytes.Zeros (Chan.Per_Arm); Mz.Cam_On_Arm.Append (0);
      Fr0 := Frame_Now;
      Plug.Set_Hooks (null, Fake_Cmd'Unrestricted_Access);
      Lockstep.Clear;
      Plug.Lock_Begin;
      declare
         Hd : Zone_Hand;
      begin
         Lockstep.Start (0, Hd'Identity);
         loop
            Lockstep.Run (0);
            exit when Lockstep.Finished (0);
            Beats := Beats + 1;
            if Push_Start = 0 and then Jaw_Tgt < 0.0 then
               Push_Start := Beats;
            end if;
            Jaw_Now := Jaw_Now + Long_Float'Max (-0.01, Long_Float'Min (0.01, Jaw_Tgt - Jaw_Now));
            if Push_Start > 0 and then Beats = Push_Start + 39 then
               At_40 := Jaw_Now;
            end if;
            Plug.Lock_Feed (Frame_Now);
         end loop;
      end;
      Plug.Lock_End;
      Lockstep.Clear;
      Plug.Set_Hooks (null, null);
      Old_End := -1.0 * (At_40 - 0.5) < 0.5 * 1.0;   --  原来 Sweep 的到头判据:读数挪不到命令(1.0)的一半
      Check (not Ok_Z and then Push_Start > 0 and then Beats = Push_Start + 39 and then Hz.Empty_Close = 0.0 and then abs (At_40 - 0.1) < 1.0e-9 and then Old_End,
             "慢的抓握通道:第一下推出去等满 " & Codec.Img (Beats - Push_Start + 1) & " 拍读数还在走(读数 " & Codec.Fmt (At_40, 2) & ",要去 -0.5)⇒ "
             & (if Ok_Z then "照样量下去(错)" else "照实说没停住、握区这回不量") & " · 牙:原来这时 Good = True、挪了 0.4 不到命令的一半 ⇒ "
             & (if Old_End then "把 0.10 当成推到头" else "(牙没咬住)"));
   end;
   --  🔴 没点名的抓握通道发这一集给过它的最后一个目标,不发此刻的读数(Plug.Jaw_Values,V1B24 2026-09-27:碰桌面时手指被沿滑轨往里推,
   --  "保持此刻的读数"把推合了的读数锁住,爪子合上,后一瓣量短 13 mm)。给了 0.3 ⇒ 发 0.3;下一条没给、读数被推到 0.8 ⇒ 还发 0.3;
   --  对方复位(清空)⇒ 发读数 0.8;一次没给过的通道 ⇒ 发读数。
   --  09-30 没读数不编数:这一拍没收到读数 ⇒ 照发上一回发出去的那一串(插头规矩 ②);这一集一次没发过又没读数 ⇒ 空(这一组这回不发);
   --  五指手只点名第 0 根(给 0.2)⇒ 五个数照发(0.2 + 其余四根的读数),下一拍没读数 ⇒ 还是那五个。
   --  牙:原来的 Jaw_Value(下面照抄一份)在"一次没发过、没读数"时发 1.0(x5 夹爪"1 = 张开"的约定,拿着东西时等于松手),而且只发 1 个数
   declare
      L, L2, L3 : Plug.Link;
      C1, C2, Cf : Plug.Cmd;
      Cur, Cur5, None : Bytes.Floats;
      V1, V2, V3, V4, V5, V6, V7, V8 : Bytes.Floats;
      Old_Set : Plug.Floats_Vectors.Vector;
      --  原来的写法(照抄):第 K 个数;没给、没给过、没读数 ⇒ 1.0;发几个 = max(1, 读数个数)
      function Old_Value (K : Natural; Mine : Boolean; C : Plug.Cmd; Cur : Bytes.Floats) return Long_Float is
      begin
         while Natural (Old_Set.Length) <= 0 loop
            Old_Set.Append (Bytes.F64_Vectors.Empty_Vector);
         end loop;
         if Mine and then K < Natural (C.Jaw.Length) then
            return C.Jaw (K);
         elsif K < Natural (Old_Set (0).Length) then
            return Old_Set (0) (K);
         elsif K < Natural (Cur.Length) then
            return Cur (K);
         end if;
         return 1.0;
      end Old_Value;
      Old_N : constant Natural := Natural'Max (1, Natural (None.Length));
      Old_V : constant Long_Float := Old_Value (0, False, C2, None);
      function Just (V : Bytes.Floats; X : Long_Float) return Boolean is (Natural (V.Length) = 1 and then V (0) = X);
      function Five (V : Bytes.Floats; X0, X : Long_Float) return Boolean is
        (Natural (V.Length) = 5 and then V (0) = X0 and then (for all K in 1 .. 4 => V (K) = X));
   begin
      C1.Jaw.Append (0.3);
      Cur.Append (1.0);
      V1 := Plug.Jaw_Values (L, 0, True, C1, Cur);
      Cur.Replace_Element (0, 0.8);
      V2 := Plug.Jaw_Values (L, 0, False, C2, Cur);
      V4 := Plug.Jaw_Values (L, 1, False, C2, Cur);
      L.Jaw_Set.Clear; L.Jaw_Sent.Clear;
      V3 := Plug.Jaw_Values (L, 0, False, C2, Cur);
      V5 := Plug.Jaw_Values (L, 0, False, C2, None);    --  这一拍没读数:照发上一回发出去的 0.8
      V6 := Plug.Jaw_Values (L2, 0, False, C2, None);   --  一次没发过、没读数:空
      Cf.Jaw.Append (0.2);
      for K in 1 .. 5 loop
         Cur5.Append (0.5);
      end loop;
      V7 := Plug.Jaw_Values (L3, 0, True, Cf, Cur5);
      V8 := Plug.Jaw_Values (L3, 0, False, C2, None);
      Check (Just (V1, 0.3) and then Just (V2, 0.3) and then Just (V3, 0.8) and then Just (V4, 0.8) and then Just (V5, 0.8) and then V6.Is_Empty
             and then Five (V7, 0.2, 0.5) and then Five (V8, 0.2, 0.5)
             and then Old_N = 1 and then Old_V = 1.0,
             "抓握通道:给了 0.3 发 0.3 · 没给、读数被推到 0.8 还发 0.3 · 复位后发读数 0.8 · 一次没给过的那一组发读数 0.8 · 这一拍没读数照发上一回的 "
             & (if Just (V5, 0.8) then "0.8" else "(错)") & " · 一次没发过又没读数 ⇒ " & (if V6.Is_Empty then "这一组不发" else "发了(错)")
             & " · 五指手只给第 0 根 0.2 ⇒ 发 " & Codec.Img (Natural (V7.Length)) & " 个数、没读数的下一拍还是 " & Codec.Img (Natural (V8.Length))
             & " 个(要 5 / 5)· 牙:原来的写法这时发 " & Codec.Img (Old_N) & " 个数、值 " & Codec.Fmt (Old_V, 1) & "(编的 1.0 = x5 的张开)");
   end;
   --  🔴 保持动作(Plug.Hold_Action:发第一条命令之前每拍回给对方的"照现在保持")每组抓握照读数的个数发(09-30):
   --  一组抓握报 5 个数(五指手)⇒ 发 5 个(原来每只手只发 1 个 = 形状不对);下一拍这组读数没来 ⇒ 照发上一回的 5 个;
   --  一次没发过、这一拍又没读数 ⇒ 这个键不发(原来发编的 1.0)。关节那一键照读数回声
   declare
      S, Out1 : Buf;
      D1, D2, Dh : Msgpack.Doc;
      L, L2 : Plug.Link;
      procedure Build (With_Hand : Boolean) is
      begin
         S.Clear;
         Msgpack.Put_Map (S, 1);
         Msgpack.Put_Str (S, "obs"); Msgpack.Put_Map (S, (if With_Hand then 2 else 1));
         if With_Hand then
            Msgpack.Put_Str (S, "hand"); Msgpack.Put_Array (S, 5);
            for I in 1 .. 5 loop
               Msgpack.Put_Float (S, 0.4);
            end loop;
         end if;
         Msgpack.Put_Str (S, "elbow"); Msgpack.Put_Array (S, 6);
         for I in 1 .. 6 loop
            Msgpack.Put_Float (S, 0.1);
         end loop;
      end Build;
      function Hand_Of (B : Buf) return Bytes.Floats is
      begin
         if not Msgpack.Decode (B, Dh) or else Msgpack.Key (Dh, 0, "hand") < 0 then
            return Bytes.F64_Vectors.Empty_Vector;
         end if;
         return Msgpack.Numbers (Dh, Msgpack.Key (Dh, 0, "hand"));
      end Hand_Of;
      H1, H2, H3 : Bytes.Floats;
      Elbow_Ok : Boolean;
      --  牙:原来的写法 —— 每只手的抓握一栏只发 1 个数,没读数发 1.0
      function Old_Hand (J : Bytes.Floats) return Bytes.Floats is
        (if J.Is_Empty then Bytes.F64_Vectors.To_Vector (1.0, 1) else Bytes.F64_Vectors.To_Vector (J (0), 1));
      Old1, Old3 : Bytes.Floats;
   begin
      Build (True);
      Check (Msgpack.Decode (S, D1), "保持动作:带五指手的那一帧解得开");
      Build (False);
      Check (Msgpack.Decode (S, D2), "保持动作:抓握读数没来的那一帧解得开");
      Layout.Recognise (D1, Msgpack.Key (D1, 0, "obs"), L.Lay);
      L2.Lay := L.Lay;
      L.Last := D1; L.Last_Obs := Msgpack.Key (D1, 0, "obs");
      Out1 := Plug.Hold_Action (L);
      H1 := Hand_Of (Out1);
      Elbow_Ok := Msgpack.Key (Dh, 0, "elbow") >= 0 and then Natural (Msgpack.Numbers (Dh, Msgpack.Key (Dh, 0, "elbow")).Length) = 6;
      L.Last := D2; L.Last_Obs := Msgpack.Key (D2, 0, "obs");
      H2 := Hand_Of (Plug.Hold_Action (L));
      L2.Last := D2; L2.Last_Obs := Msgpack.Key (D2, 0, "obs");
      H3 := Hand_Of (Plug.Hold_Action (L2));
      Old1 := Old_Hand (Msgpack.Numbers (D1, Msgpack.Key (D1, Msgpack.Key (D1, 0, "obs"), "hand")));
      Old3 := Old_Hand (Bytes.F64_Vectors.Empty_Vector);
      Check (Natural (H1.Length) = 5 and then (for all X of H1 => X = 0.4) and then Elbow_Ok
             and then Natural (H2.Length) = 5 and then (for all X of H2 => X = 0.4) and then H3.Is_Empty
             and then Natural (Old1.Length) = 1 and then Natural (Old3.Length) = 1 and then Old3 (0) = 1.0,
             "保持动作:五指手报 5 个 ⇒ 发 " & Codec.Img (Natural (H1.Length)) & " 个(要 5)、关节 6 个照回 · 下一拍这组读数没来 ⇒ 照发上一回的 "
             & Codec.Img (Natural (H2.Length)) & " 个 · 一次没发过又没读数 ⇒ " & (if H3.Is_Empty then "这个键不发" else "发了(错)")
             & " · 牙:原来发 " & Codec.Img (Natural (Old1.Length)) & " 个数,没读数发 " & Codec.Fmt (Old3 (0), 1) & "(x5 的张开)");
   end;
   --  🔴 某台相机这一拍没收到画面(不是图 / 数据不够)⇒ 那一格留占位,后面相机的下标不许前移(Plug.Frame_Of / Note_Beat,09-30):
   --  三台相机 c0 / c1 / c2(灰度 10 / 20 / 30),第二拍 c1 的数据不够 ⇒ F.Cams 还是 3 格、第 1 格是占位、第 2 格是 c2 的画面;
   --  逐拍的账里 c1 这一拍和下一拍都记"没量"(不当"画面没变"),c2 这一拍和它自己上一拍比(没变 = 0)。
   --  牙:原来丢一台就不占位(下面照抄那一段)⇒ 只剩 2 格,第 1 格装的是 c2 的画面,"c1 这一拍变了多少"拿 c2 和 c1 的上一帧比出 10
   declare
      S : Buf;
      D1, D2 : Msgpack.Doc;
      L : Plug.Link;
      F1, F2, F3 : Plug.Frame;
      Old : Plug.Cam_Vectors.Vector;
      procedure Img (Name : String; V : Interfaces.Unsigned_8; Short : Boolean) is
         Px : Buf;
      begin
         for I in 1 .. 4 * 3 * 3 loop
            Px.Append (V);
         end loop;
         Msgpack.Put_Str (S, Name); Msgpack.Put_Map (S, 4);
         Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
         Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "|u1");
         Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, 3); Msgpack.Put_Int (S, 4); Msgpack.Put_Int (S, 3);
         Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Px, 0, (if Short then 4 * 3 * 3 - 1 else 4 * 3 * 3));
      end Img;
      procedure Build (Drop_C1 : Boolean) is
      begin
         S.Clear;
         Msgpack.Put_Map (S, 1);
         Msgpack.Put_Str (S, "obs"); Msgpack.Put_Map (S, 4);
         Img ("c0", 10, False); Img ("c1", 20, Drop_C1); Img ("c2", 30, False);
         Msgpack.Put_Str (S, "elbow"); Msgpack.Put_Array (S, 6);
         for I in 1 .. 6 loop
            Msgpack.Put_Float (S, 0.1);
         end loop;
      end Build;
      procedure Take (D : Msgpack.Doc; F : out Plug.Frame) is
      begin
         L.Last := D; L.Last_Obs := Msgpack.Key (D, 0, "obs");
         L.Seq := L.Seq + 1;
         F := (others => <>);
         Plug.Frame_Of (L, F);
         Plug.Note_Beat (L, F);
      end Take;
      B2, B3 : Plug.Beat;
      Old_Chg1 : Long_Float := 0.0;
   begin
      Build (False);
      Check (Msgpack.Decode (S, D1), "相机占位:三台相机那一帧解得开");
      Build (True);
      Check (Msgpack.Decode (S, D2), "相机占位:c1 数据不够的那一帧解得开");
      Layout.Recognise (D1, Msgpack.Key (D1, 0, "obs"), L.Lay);
      Take (D1, F1);
      Take (D2, F2);
      B2 := L.Beats.Last_Element;
      Take (D1, F3);
      B3 := L.Beats.Last_Element;
      --  原来的写法(照抄):收到的才 Append
      for Ci in 0 .. Natural (L.Lay.Cams.Length) - 1 loop
         declare
            N : constant Integer := Layout.Find (D2, Msgpack.Key (D2, 0, "obs"), L.Lay.Cams (Ci));
            W, H, First, Len : Natural;
            C : Plug.Cam;
         begin
            if Layout.Is_Image (D2, N, W, H) then
               Msgpack.Nd_Data (D2, N, First, Len);
               if Len >= W * H * 3 then
                  C.W := W; C.H := H;
                  for I in 0 .. W * H - 1 loop
                     C.Gray.Append (D2.Raw.Element (First + 3 * I));
                  end loop;
                  Old.Append (C);
               end if;
            end if;
         end;
      end loop;
      if Natural (Old.Length) >= 2 and then Natural (Old (1).Gray.Length) = 4 * 3 then
         Old_Chg1 := abs (Long_Float (Old (1).Gray (0)) - Long_Float (F1.Cams (1).Gray (0)));
      end if;
      Check (Natural (L.Lay.Cams.Length) = 3 and then Natural (F2.Cams.Length) = 3 and then not Plug.Has_Picture (F2.Cams (1))
             and then Plug.Has_Picture (F2.Cams (2)) and then F2.Cams (2).Gray (0) = 30 and then F2.Cams (0).Gray (0) = 10
             and then Natural (B2.Img_Ok.Length) = 3 and then B2.Img_Ok (0) and then not B2.Img_Ok (1) and then B2.Img_Ok (2) and then B2.Img_Chg (2) = 0.0
             and then not B3.Img_Ok (1) and then B3.Img_Ok (2)
             and then Natural (Old.Length) = 2 and then Old (1).Gray (0) = 30 and then Old_Chg1 = 10.0,
             "相机占位:c1 这一拍没收到 ⇒ 还是 " & Codec.Img (Natural (F2.Cams.Length)) & " 格,第 1 格" & (if Plug.Has_Picture (F2.Cams (1)) then "有画面(错)" else "是占位")
             & "、第 2 格灰度 " & Codec.Img (Natural (F2.Cams (2).Gray (0))) & "(要 30 = c2)· c1 这一拍、下一拍都记没量;c2 和它自己上一拍比 "
             & Codec.Fmt (B2.Img_Chg (2), 1) & " · 牙:原来只剩 " & Codec.Img (Natural (Old.Length)) & " 格、第 1 格是 c2(灰度 "
             & Codec.Img (Natural (Old (1).Gray (0))) & "),'c1 变了' " & Codec.Fmt (Old_Chg1, 1));
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
            declare
               --  这一块框里的平均颜色(自检自己算;驱动里原来那个 Picture.Mean_Colour 没人调,09-30 删了)
               Sr, Sg, Sb : Long_Float := 0.0;
               N : Natural := 0;
            begin
               for Y in R.Y0 .. Natural'Min (R.Y1, H - 1) loop
                  for X in R.X0 .. Natural'Min (R.X1, W - 1) loop
                     Sr := Sr + Long_Float (RGB.Element (3 * (Y * W + X)));
                     Sg := Sg + Long_Float (RGB.Element (3 * (Y * W + X) + 1));
                     Sb := Sb + Long_Float (RGB.Element (3 * (Y * W + X) + 2));
                     N := N + 1;
                  end loop;
               end loop;
               Cr := Sr / Long_Float (Natural'Max (1, N)); Cg := Sg / Long_Float (Natural'Max (1, N)); Cb := Sb / Long_Float (Natural'Max (1, N));
            end;
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
   --  🔴 语法里没有拍的上限(Sinew.EBNF / Grammar / Parse,09-30 owner 的规矩:拍的数不许):一个词几个字母、名字几个词、一句话几个字、
   --  一段几行、一段几条约束、块里几行、数有几位,全用重复;解析这一头也不截词、不限块套几层、大得装不进 Natural 的数照实退回。
   --  拿一个小的 GBNF 匹配器(按"能走到哪些位置"的集合一步步推,不回溯)对着驱动交给解码器的那份文法试 ——
   --  名字一个词 27 个字母(pinkandwhitestripedcupcakes)· 名字五个词 · 五行 · 100 个字的一句 say · 一段三条约束 + or 250 steps · repeat 块里三行
   --  ⇒ 新文法全接得住;牙:把新文法按原来的写法还原(一个词最多 24 个字母 —— 照 mintgreenscissors 定的、名字最多 3 个词、最多 4 行、
   --  一句话最多 81 个字、约束最多 2 条、数最多两位、块里最多 2 行)⇒ 这几句一句都接不住;最普通的两句两份都接得住(还原得对、匹配器也对)
   declare
      type Node_Kind is (N_Alt, N_Seq, N_Opt, N_Star, N_Plus, N_Lit, N_Class, N_Ref);
      type G_Node is record
         K : Node_Kind := N_Seq;
         Kids : Bytes.Ints;
         Txt : Unbounded_String;   --  N_Lit:字面;N_Class:这一类里有哪些字符;N_Ref:规则名
         Neg : Boolean := False;
      end record;
      package G_Vectors is new Ada.Containers.Vectors (Natural, G_Node);
      type G_Tree is record
         Nodes : G_Vectors.Vector;
         Names : Bytes.Strs;
         Roots : Bytes.Ints;
         Ok : Boolean := True;      --  文法本身写得对不对(括号、引号配得上,引到的规则都有)
      end record;
      function Build (G : String) return G_Tree is
         T : G_Tree;
         Src : Unbounded_String;
         P : Natural := 1;
         function At_End return Boolean is (P > Length (Src));
         function Cur return Character is (Element (Src, P));
         procedure Skip is
         begin
            while not At_End and then Cur = ' ' loop
               P := P + 1;
            end loop;
         end Skip;
         function Add (N : G_Node) return Natural is
         begin
            T.Nodes.Append (N);
            return Natural (T.Nodes.Length) - 1;
         end Add;
         function Esc return Character is
         begin
            if Cur = '\' and then P < Length (Src) then
               P := P + 1;
               return (if Cur = 'n' then ASCII.LF else Cur);
            end if;
            return Cur;
         end Esc;
         function Alt return Natural;
         function Prim return Natural is
            N : G_Node;
         begin
            Skip;
            if At_End then
               T.Ok := False;
               return Add (N);
            end if;
            if Cur = '"' then
               N.K := N_Lit;
               P := P + 1;
               while not At_End and then Cur /= '"' loop
                  Append (N.Txt, Esc);
                  P := P + 1;
               end loop;
               T.Ok := T.Ok and then not At_End;
               P := P + 1;
               return Add (N);
            elsif Cur = '[' then
               N.K := N_Class;
               P := P + 1;
               if not At_End and then Cur = '^' then
                  N.Neg := True;
                  P := P + 1;
               end if;
               while not At_End and then Cur /= ']' loop
                  declare
                     C0 : constant Character := Esc;
                  begin
                     if P + 2 <= Length (Src) and then Element (Src, P + 1) = '-' and then Element (Src, P + 2) /= ']' then
                        for X in C0 .. Element (Src, P + 2) loop
                           Append (N.Txt, X);
                        end loop;
                        P := P + 3;
                     else
                        Append (N.Txt, C0);
                        P := P + 1;
                     end if;
                  end;
               end loop;
               T.Ok := T.Ok and then not At_End;
               P := P + 1;
               return Add (N);
            elsif Cur = '(' then
               P := P + 1;
               declare
                  A : constant Natural := Alt;
               begin
                  Skip;
                  if not At_End and then Cur = ')' then
                     P := P + 1;
                  else
                     T.Ok := False;
                  end if;
                  return A;
               end;
            end if;
            N.K := N_Ref;
            while not At_End and then Cur in 'a' .. 'z' | '0' .. '9' | '_' | '-' loop
               Append (N.Txt, Cur);
               P := P + 1;
            end loop;
            if Length (N.Txt) = 0 then
               T.Ok := False;
               P := P + 1;
            end if;
            return Add (N);
         end Prim;
         function Term return Natural is
            A : constant Natural := Prim;
            N : G_Node;
         begin
            if not At_End and then Cur in '?' | '*' | '+' then
               N.K := (case Cur is when '?' => N_Opt, when '*' => N_Star, when others => N_Plus);
               N.Kids.Append (A);
               P := P + 1;
               return Add (N);
            end if;
            return A;
         end Term;
         function Seq return Natural is
            N : G_Node;
         begin
            loop
               Skip;
               exit when At_End or else Cur in ')' | '|';
               N.Kids.Append (Term);
            end loop;
            return Add (N);
         end Seq;
         function Alt return Natural is
            N : G_Node;
         begin
            N.K := N_Alt;
            N.Kids.Append (Seq);
            loop
               Skip;
               exit when At_End or else Cur /= '|';
               P := P + 1;
               N.Kids.Append (Seq);
            end loop;
            return Add (N);
         end Alt;
         From : Natural := G'First;
      begin
         for I in G'First .. G'Last + 1 loop
            if I > G'Last or else G (I) = ASCII.LF then
               declare
                  Ln : constant String := G (From .. I - 1);
                  Def : constant Natural := Ada.Strings.Fixed.Index (Ln, " ::= ");
               begin
                  if Ln'Length > 0 then
                     if Def = 0 then
                        T.Ok := False;
                     else
                        T.Names.Append (Ln (Ln'First .. Def - 1));
                        Src := To_Unbounded_String (Ln (Def + 5 .. Ln'Last));
                        P := 1;
                        T.Roots.Append (Alt);
                        T.Ok := T.Ok and then At_End;
                     end if;
                  end if;
               end;
               From := I + 1;
            end if;
         end loop;
         for N of T.Nodes loop
            if N.K = N_Ref and then not T.Names.Contains (To_String (N.Txt)) then
               T.Ok := False;
            end if;
         end loop;
         return T;
      end Build;
      function Accepts (T : G_Tree; Input : String) return Boolean is
         Last : constant Natural := Input'Length;
         function None return Bools is (Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Last + 1)));
         function Union (A, B : Bools) return Bools is
            R : Bools := A;
         begin
            for I in 0 .. Last loop
               if B (I) then
                  R.Replace_Element (I, True);
               end if;
            end loop;
            return R;
         end Union;
         function Same (A, B : Bools) return Boolean is (for all I in 0 .. Last => A (I) = B (I));
         function Match (Id : Natural; From : Bools) return Bools is
            Nd : constant G_Node := T.Nodes (Id);
            R : Bools := None;
         begin
            if (for all X of From => not X) then
               return From;
            end if;
            case Nd.K is
               when N_Lit =>
                  declare
                     S : constant String := To_String (Nd.Txt);
                  begin
                     for Q in 0 .. Last loop
                        if From (Q) and then Q + S'Length <= Last and then Input (Input'First + Q .. Input'First + Q + S'Length - 1) = S then
                           R.Replace_Element (Q + S'Length, True);
                        end if;
                     end loop;
                  end;
               when N_Class =>
                  for Q in 0 .. Last - 1 loop
                     if From (Q) and then (Ada.Strings.Fixed.Index (To_String (Nd.Txt), [1 => Input (Input'First + Q)]) > 0) /= Nd.Neg then
                        R.Replace_Element (Q + 1, True);
                     end if;
                  end loop;
               when N_Seq =>
                  R := From;
                  for K of Nd.Kids loop
                     R := Match (Natural (K), R);
                  end loop;
               when N_Alt =>
                  for K of Nd.Kids loop
                     R := Union (R, Match (Natural (K), From));
                  end loop;
               when N_Opt =>
                  R := Union (From, Match (Natural (Nd.Kids (0)), From));
               when N_Star | N_Plus =>
                  declare
                     Acc : Bools := (if Nd.K = N_Star then From else Match (Natural (Nd.Kids (0)), From));
                     Nx : Bools;
                  begin
                     loop
                        Nx := Union (Acc, Match (Natural (Nd.Kids (0)), Acc));
                        exit when Same (Nx, Acc);
                        Acc := Nx;
                     end loop;
                     R := Acc;
                  end;
               when N_Ref =>
                  for I in 0 .. Natural (T.Names.Length) - 1 loop
                     if T.Names (I) = To_String (Nd.Txt) then
                        R := Match (Natural (T.Roots (I)), From);
                     end if;
                  end loop;
            end case;
            return R;
         end Match;
         Start : Bools := None;
      begin
         Start.Replace_Element (0, True);
         for I in 0 .. Natural (T.Names.Length) - 1 loop
            if T.Names (I) = "root" then
               declare
                  Ends : constant Bools := Match (Natural (T.Roots (I)), Start);
               begin
                  return Ends (Last);
               end;
            end if;
         end loop;
         return False;
      end Accepts;
      --  原来的写法:把新文法里的重复还原成当时的嵌套可选项 / 写死的个数
      function Old_Of (G : String) return String is
         function Tail (N : Natural; Cls : String) return String is (if N = 0 then "" else "(" & Cls & " " & Tail (N - 1, Cls) & ")?");
         R : Unbounded_String := To_Unbounded_String (G);
         procedure Swap (A, B : String) is
            Out_S : Unbounded_String;
            S : constant String := To_String (R);
            I : Natural := S'First;
         begin
            while I <= S'Last loop
               if I + A'Length - 1 <= S'Last and then S (I .. I + A'Length - 1) = A then
                  Append (Out_S, B);
                  I := I + A'Length;
               else
                  Append (Out_S, S (I));
                  I := I + 1;
               end if;
            end loop;
            R := Out_S;
         end Swap;
      begin
         Swap ("[a-zA-Z] ([a-zA-Z0-9 ,.=\'])*", "[a-zA-Z] " & Tail (80, "[a-zA-Z0-9 ,.=\']"));
         Swap ("([a-z])*", Tail (23, "[a-z]"));
         Swap ("root ::= line (line)*", "root ::= line (line)? (line)? (line)?");
         Swap ("name ::= w ("" "" w)*", "name ::= w ("" "" w)? ("" "" w)?");
         Swap ("cons ("" and "" cons)*", "cons ("" and "" cons)?");
         Swap ("num ::= [1-9] ([0-9])*", "num ::= [1-9] ([0-9])?");
         Swap ("simple (simple)*", "simple (simple)?");
         return To_String (R);
      end Old_Of;
      Outs : constant String := "touched stuck slipped lost settled stalled timeout";
      NL : constant String := "" & ASCII.LF;
      Qn : constant String := Sinew.EBNF ("touching above", "grasper", Outs, "height");
      Fn : constant String := Sinew.EBNF ("touching above", "grasper", Outs);
      Tq, Tq_Old, Tf, Tf_Old : G_Tree;
      Long_Say : constant String := "say " & [1 .. 100 => 'a'] & NL;
      Five : constant String := "say one" & NL & "say two" & NL & "say three" & NL & "say four" & NL & "say five" & NL;
      Q_Plain : constant String := "do scissors height up until settled" & NL;
      Q_Long_Word : constant String := "do pinkandwhitestripedcupcakes height up until settled" & NL;
      Q_Five_Words : constant String := "do the big red toy car height up until settled" & NL;
      F_Plain : constant String := "do grasper still until settled" & NL;
      F_Three : constant String := "do grasper touching the red cube and grasper still and grasper open until settled or 250 steps" & NL;
      F_Block : constant String := "repeat 2 times:" & NL & "do grasper still until settled" & NL & "do grasper open until settled" & NL
                                   & "do grasper still until settled" & NL & "end" & NL;
      function Both_Plain_Ok return Boolean is
        (Accepts (Tq, Q_Plain) and then Accepts (Tq_Old, Q_Plain) and then Accepts (Tf, F_Plain) and then Accepts (Tf_Old, F_Plain));
      New_All, Old_None : Boolean;
   begin
      Tq := Build (Qn); Tq_Old := Build (Old_Of (Qn)); Tf := Build (Fn); Tf_Old := Build (Old_Of (Fn));
      New_All := Accepts (Tq, Q_Long_Word) and then Accepts (Tq, Q_Five_Words) and then Accepts (Tq, Five) and then Accepts (Tq, Long_Say)
                 and then Accepts (Tf, F_Three) and then Accepts (Tf, F_Block);
      Old_None := not Accepts (Tq_Old, Q_Long_Word) and then not Accepts (Tq_Old, Q_Five_Words) and then not Accepts (Tq_Old, Five)
                  and then not Accepts (Tq_Old, Long_Say) and then not Accepts (Tf_Old, F_Three) and then not Accepts (Tf_Old, F_Block);
      Check (Tq.Ok and then Tf.Ok and then Tq_Old.Ok and then Tf_Old.Ok and then Both_Plain_Ok and then New_All and then Old_None
             and then Ada.Strings.Fixed.Index (Qn, "([a-z] ([a-z]") = 0 and then Ada.Strings.Fixed.Index (Fn, "(line)?") = 0,
             "语法没有拍的上限:新文法" & (if New_All then "接得住" else "有接不住的(错)") & " 27 个字母的名字、五个词的名字、五行、100 个字的 say、三条约束 + 250 steps、块里三行"
             & " · 牙:按原来的写法还原 ⇒ " & (if Old_None then "一句都接不住" else "有接得住的(牙没咬住)")
             & " · 两份都接得住最普通的两句:" & (if Both_Plain_Ok then "是" else "否(错)")
             & " · 文法本身配得上:" & (if Tq.Ok and then Tf.Ok then "是" else "否(错)"));
   end;
   --  🔴 解析这一头也不截(Sinew.Parse,09-30):一行 70 个词(名字 66 个词)⇒ 整行读完、名字 66 个词;牙:原来一行最多 64 个词,
   --  多出来的静悄悄丢掉 —— 同一行只读前 64 个词,until 被丢掉、这一段退回;or 99999999999999999999 steps ⇒ 照实退回(不抛异常;
   --  牙:原来 Natural'Value 直接抛 Constraint_Error);repeat 套 40 层 ⇒ 读得下(牙:原来块栈只有 32 格,第 33 层越界)
   declare
      use Sinew;
      Name66 : Unbounded_String;
      First64 : Unbounded_String;
      Big : constant String := "99999999999999999999";
      P_Long, P_Cut, P_Big, P_Deep, P_250 : Program;
      Old_Raised, Old_Deep : Boolean := False;
      Old_Val : Natural := 0;
      Deep : Unbounded_String;
      Nw : Natural := 0;
   begin
      for I in 1 .. 66 loop
         Append (Name66, (if I > 1 then " " else "") & "w" & [1 .. 1 + I mod 5 => 'q']);
      end loop;
      P_Long := Sinew.Parse ("do grasper touching " & To_String (Name66) & " until settled");
      declare
         Words : constant String := "do grasper touching " & To_String (Name66) & " until settled";
      begin
         for Ch of Words loop
            if Ch = ' ' then
               Nw := Nw + 1;
            end if;
            exit when Nw = 64;
            Append (First64, Ch);
         end loop;
      end;
      P_Cut := Sinew.Parse (To_String (First64));
      P_Big := Sinew.Parse ("do grasper still until settled or " & Big & " steps");
      begin
         Old_Val := Natural'Value (Big);   --  原来的读法
      exception
         when Constraint_Error =>
            Old_Raised := True;
      end;
      for I in 1 .. 40 loop
         Append (Deep, "repeat 2 times:" & ASCII.LF);
      end loop;
      Append (Deep, "do grasper still until settled" & ASCII.LF);
      for I in 1 .. 40 loop
         Append (Deep, "end" & ASCII.LF);
      end loop;
      P_Deep := Sinew.Parse (To_String (Deep));
      declare
         type Old_Stack is array (1 .. 32) of Natural;   --  原来的块栈
         Os : Old_Stack := [others => 0];
      begin
         for Level in 1 .. Ada.Strings.Unbounded.Count (Deep, "repeat") loop
            Os (Level) := Level;
         end loop;
         Nw := Nw + Os (Os'Last);   --  读一下(不让优化器把上面那几格连同越界检查一起删掉);走到这儿 = 没越界(Old_Deep 还是 False)
      exception
         when Constraint_Error =>
            Old_Deep := True;
      end;
      P_250 := Sinew.Parse ("do grasper still until settled or 250 steps");
      Check (P_Long.Ok and then Natural (P_Long.Code.Length) = 1 and then To_String (P_Long.Code (0).Cons (0).Obj.Word) = To_String (Name66)
             and then not P_Cut.Ok
             and then not P_Big.Ok and then Old_Raised
             and then P_Deep.Ok and then Old_Deep
             and then P_250.Ok and then P_250.Code (0).Max_Steps = 250,
             "解析不截:一行 70 个词 ⇒ " & (if P_Long.Ok then "读完、名字 66 个词" else "退回(错)") & "(牙:原来只读前 " & Codec.Img (Nw) & " 个词 ⇒ "
             & (if P_Cut.Ok then "读成了(牙没咬住)" else "until 丢了、退回") & ")· or " & Big & " steps ⇒ "
             & (if P_Big.Ok then "收了(错)" else "照实退回:" & To_String (P_Big.Err)) & "(牙:原来 Natural'Value " & (if Old_Raised then "抛异常" else "没抛,读成 " & Codec.Img (Old_Val)) & ")"
             & " · repeat 套 40 层 ⇒ " & (if P_Deep.Ok then "读得下" else "退回(错):" & To_String (P_Deep.Err))
             & "(牙:原来 32 格的块栈第 33 层" & (if Old_Deep then "越界" else "没越界") & ")· or 250 steps ⇒ " & Codec.Img (P_250.Code (0).Max_Steps));
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
   --  🔴 不动的眼按标定板解(Geom.Fit_Fixed_Board,09-30 修的几样):合成的头顶眼(焦距 288、640×480、斜着看桌子),板上的点铺在桌面上,
   --  每个点在这只眼里配点噪声 Sh(像素)、配到的像素按 Sh 抖
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Gt : Geom.Cam_Geo;
      Seed : Long_Long_Integer := 11;
      function Jit return Long_Float is   --  确定性伪随机 ±1(测试数据自己的抖动)
      begin
         Seed := (Seed * 1103515245 + 12345) mod 2147483648;
         return Long_Float (Integer ((Seed / 65536) mod 2001) - 1000) / 1000.0;
      end Jit;
      --  Nx × Ny 个点铺在桌面上(x −0.3..0.3、y −0.3..0.05、高 0.8 上下 5 cm 起伏,米,合成),画面里的才要
      procedure Board (Nx, Ny : Positive; Sh : Long_Float; Sc : out Geom.Scene_Pt_Vectors.Vector) is
      begin
         Sc.Clear;
         for I in 0 .. Nx - 1 loop
            for J in 0 .. Ny - 1 loop
               declare
                  Pw : constant Geom.V3 := [-0.3 + 0.6 * Long_Float (I) / Long_Float (Nx - 1), -0.3 + 0.35 * Long_Float (J) / Long_Float (Ny - 1),
                                            0.8 + 0.05 * Sin (Long_Float (I + 2 * J))];
                  U, V : Long_Float;
                  Fr : Boolean;
               begin
                  Geom.Project_Fixed (Gt, Pw, U, V, Fr);
                  if Fr and then U > 0.0 and then U < 640.0 and then V > 0.0 and then V < 480.0 then
                     Sc.Append (Geom.Scene_Pt'(Pw => Pw, Cov => [others => [others => 0.0]], U => U + Sh * Jit, V => V + Sh * Jit, Sh => Sh, Views => 3));
                  end if;
               end;
            end loop;
         end loop;
      end Board;
      function Off_By (G : Geom.Cam_Geo) return Long_Float is (Geom.Norm ([G.Pos (0) - Gt.Pos (0), G.Pos (1) - Gt.Pos (1), G.Pos (2) - Gt.Pos (2)]));
      Sc : Geom.Scene_Pt_Vectors.Vector;
   begin
      Gt.F := 288.0; Gt.Cx := 320.0; Gt.Cy := 240.0; Gt.R_Ce := Geom.Rodrigues ([0.3, 0.1, 0.0]); Gt.Pos := [0.05, -0.45, 1.75]; Gt.Fixed := True; Gt.Valid := True;
      --  ① 焦距先验那一槽只算一条方程(Param_Sd):同一块 6 点的板,焦距不给、一次不带先验、一次带一条宽得没分量的先验(±1e9 px)。
      --  两次解一样、残差一样,差的只是方程条数:不确定度之比 = √((12 − 7) ÷ (13 − 7));牙:旧写法把先验那一槽算两条 ⇒ √(5 ÷ 7)
      Board (2, 3, 0.3, Sc);
      declare
         Ga, Gb : Geom.Cam_Geo;
         Ra, Rb : Geom.Fixed_Report;
         Oka, Okb : Boolean;
      begin
         Ga.F := 0.0; Ga.Cx := 320.0; Ga.Cy := 240.0;
         Gb := Ga; Gb.F_Prior := 288.0; Gb.F_Prior_Sd := 1.0e9;
         Geom.Fit_Fixed_Board (Ga, Sc, Ra, Oka);
         Geom.Fit_Fixed_Board (Gb, Sc, Rb, Okb);
         declare
            N : constant Natural := Ra.Scene_Used;
            Ratio : constant Long_Float := (if Oka and then Okb and then Ga.F_Sd > 0.0 then Gb.F_Sd / Ga.F_Sd else 0.0);
            Want : constant Long_Float := Sqrt (Long_Float (2 * N - 7) / Long_Float (2 * N + 1 - 7));
            Old : constant Long_Float := Sqrt (Long_Float (2 * N - 7) / Long_Float (2 * N + 2 - 7));
         begin
            Check (Oka and then Okb and then N = Natural (Sc.Length) and then Rb.Scene_Used = N and then abs (Ratio - Want) < 1.0e-6 and then abs (Ratio - Old) > 0.01,
                   "不动的眼·先验一槽一条方程:" & Codec.Img (N) & " 点的板,带宽先验和不带的焦距不确定度之比 " & Codec.Fmt (Ratio, 6) & "(该 √(5/6) = "
                   & Codec.Fmt (Want, 6) & ")· 牙:旧写法算两条 ⇒ " & Codec.Fmt (Old, 6));
         end;
      end;
      --  ② 从现位姿起步(Start_Here,核对时走这条):120 个点的板里 30 个配错,错得一层比一层小(300 / 100 / 30 / 10 / 4 px 各 6 个,
      --  Sh = 0.2 px);起点偏 0.6°。门从粗到细收到挑出来的那一批不再变 ⇒ 30 个全踢掉、一个好的都不踢,位置差 < 3 mm;
      --  牙:要收的遍数 ≥ 4 = 旧写法固定收三遍,第三遍挑出来的还在变也照样交
      Board (12, 10, 0.2, Sc);
      for K in 0 .. 29 loop
         declare
            Tier : constant array (0 .. 4) of Long_Float := [300.0, 100.0, 30.0, 10.0, 4.0];   --  配错多少像素(合成)
            P : Geom.Scene_Pt := Sc (K * 4 + 1);
         begin
            P.U := P.U + Tier (K / 6) * Cos (2.4 * Long_Float (K)); P.V := P.V + Tier (K / 6) * Sin (2.4 * Long_Float (K));
            Sc.Replace_Element (K * 4 + 1, P);
         end;
      end loop;
      declare
         Gs, Gz : Geom.Cam_Geo;
         Rs, Rz : Geom.Fixed_Report;
         Oks, Okz : Boolean;
         Rounds_S : Natural;
      begin
         Gs := Gt; Gs.R_Ce := Geom.Mul (Gt.R_Ce, Geom.Rodrigues ([0.01, -0.005, 0.0]));
         Geom.Fit_Fixed_Board (Gs, Sc, Rs, Oks, Start_Here => True);
         Rounds_S := Geom.Refits;
         Check (Oks and then Rs.Scene_Used = Natural (Sc.Length) - 30 and then Gs.Dropped = 30 and then Off_By (Gs) < 0.003 and then Rounds_S >= 4,
                "不动的眼·从现位姿起步:" & Codec.Img (Natural (Sc.Length)) & " 个点里 30 个一层比一层小地配错 ⇒ 踢掉 " & Codec.Img (Gs.Dropped) & " 个、位置差 "
                & Codec.Fmt (1000.0 * Off_By (Gs), 2) & " mm · 牙:门收了 " & Codec.Img (Rounds_S) & " 遍才定(旧写法固定三遍)");
         --  ③ 同一块板从零盲搜:按中位 3 倍踢到不再变(Reselect_Loop)⇒ 同样 30 个全踢掉
         Gz.F := 288.0; Gz.Cx := 320.0; Gz.Cy := 240.0;
         Geom.Fit_Fixed_Board (Gz, Sc, Rz, Okz);
         Check (Okz and then Rz.Scene_Used = Natural (Sc.Length) - 30 and then Gz.Dropped = 30 and then Off_By (Gz) < 0.003,
                "不动的眼·从零盲搜:同一块板 ⇒ 踢掉 " & Codec.Img (Gz.Dropped) & " 个(重解 " & Codec.Img (Geom.Refits) & " 遍)、位置差 " & Codec.Fmt (1000.0 * Off_By (Gz), 2) & " mm");
      end;
      --  ④ 焦距给了(288)、板挤成 1 mm 见方的一小团 ⇒ 位置分不开、解不出;报原因里印给的焦距,不带 ±(牙:旧写法印 P (5) = 相机位置的 z,后面 ± 0.0 px)
      Sc.Clear;
      for I in 0 .. 5 loop
         declare
            Pw : constant Geom.V3 := [0.0005 * Long_Float (I mod 3), 0.0005 * Long_Float (I / 3), 0.85];   --  米,合成
            U, V : Long_Float;
            Fr : Boolean;
         begin
            Geom.Project_Fixed (Gt, Pw, U, V, Fr);
            Sc.Append (Geom.Scene_Pt'(Pw => Pw, Cov => [others => [others => 0.0]], U => U + 0.3 * Jit, V => V + 0.3 * Jit, Sh => 0.3, Views => 3));
         end;
      end loop;
      declare
         Gf : Geom.Cam_Geo;
         Rf : Geom.Fixed_Report;
         Okf : Boolean;
      begin
         Gf.F := 288.0; Gf.Cx := 320.0; Gf.Cy := 240.0;
         Geom.Fit_Fixed_Board (Gf, Sc, Rf, Okf);
         declare
            W : constant String := To_String (Geom.Why);
         begin
            Check (not Okf and then Ada.Strings.Fixed.Index (W, "焦距 288.0 px(给的") > 0 and then Ada.Strings.Fixed.Index (W, "± 0.0 px") = 0,
                   "不动的眼·焦距给了也解不出时,报原因印给的焦距:" & W);
         end;
      end;
      --  ⑤ 核对的细门按板配得多细(Geom.Board_Rms):101 个点的误差按瑞利分布的分位数摆(σ = 0.5 px,中位正好 σ√(2 ln 2))⇒ 均方根 = σ√2,差 < 1e-12;
      --  牙:旧写法中位 × 1.2 差 0.09%(6.6e-4 px)
      Sc.Clear;
      for I in 0 .. 100 loop
         declare
            Pw : constant Geom.V3 := [-0.3 + 0.006 * Long_Float (I), -0.2 + 0.002 * Long_Float (I), 0.85];   --  米,合成
            U, V : Long_Float;
            Fr : Boolean;
            R : constant Long_Float := 0.5 * Sqrt (-2.0 * Log (1.0 - (Long_Float (I) + 0.5) / 101.0));   --  瑞利分布的分位数(σ = 0.5 px)
         begin
            Geom.Project_Fixed (Gt, Pw, U, V, Fr);
            Sc.Append (Geom.Scene_Pt'(Pw => Pw, Cov => [others => [others => 0.0]], U => U + R * Cos (2.4 * Long_Float (I)), V => V + R * Sin (2.4 * Long_Float (I)),
                                      Sh => 0.0, Views => 3));
         end;
      end loop;
      declare
         Br : constant Long_Float := Geom.Board_Rms (Gt, Sc, Long_Float'Last);
         Old : constant Long_Float := 1.2 * 0.5 * Sqrt (2.0 * Log (2.0));
      begin
         Check (abs (Br - 0.5 * Sqrt (2.0)) < 1.0e-12 and then abs (Old - 0.5 * Sqrt (2.0)) > 1.0e-4,
                "核对的细门:瑞利分位数摆的误差 ⇒ 均方根 " & Codec.Fmt (Br, 9) & " px(真 σ√2 = " & Codec.Fmt (0.5 * Sqrt (2.0), 9) & ")· 牙:中位 × 1.2 ⇒ " & Codec.Fmt (Old, 9));
      end;
   end;
   --  🔴 离群重挑到不再变(09-30,Geom.Reselect / Reselect_Loop,Fit_Rig 和 Fit_Fixed_Board 同一套)。玩具:一堆数求平均,残差 = 离平均多远;
   --  好的在 [−1, 1] 里匀开。① 一层比一层小的错(1000 / 100 / 30 / 10 各 3 个,40 个好的)⇒ 挑到不再变:12 个全出去、平均回到 0;
   --  牙:旧的固定三遍还留 3 个、第四遍还在变;只挑一遍留 9 个。② 三成错(+50 × 8、−50 × 4,30 个好的)⇒ 一遍全出去;
   --  牙:旧的"踢掉的到了四分之一就全放回"把 12 个全放回,平均偏到 4.76。③ 错的占六成、一个比一个大(5 × 2^(k/3),30 个)⇒ 挑到进解的不到一半 ⇒
   --  Broken,照实报;牙:旧的固定三遍交出一个还在变、平均 26 的"解"
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      use type Geom.Reselect_End;
      type Toy is (Cascade, Quarter, Majority);
      procedure Run (K : Toy) is
         Xs : Geom.Param_Vec (0 .. 99) := [others => 0.0];
         N : Natural := 0;
         N_Good : constant Natural := (case K is when Cascade => 40, when Quarter => 30, when Majority => 20);
         Mean : Long_Float := 0.0;
         procedure Add (X : Long_Float) is
         begin
            Xs (N) := X; N := N + 1;
         end Add;
         procedure Solve (S : Geom.Flags) is
            Sum : Long_Float := 0.0;
            C : Natural := 0;
         begin
            for I in S'Range loop
               if not S (I) then
                  Sum := Sum + Xs (I); C := C + 1;
               end if;
            end loop;
            Mean := (if C > 0 then Sum / Long_Float (C) else 0.0);
         end Solve;
         procedure Errs (Rs : out Geom.Param_Vec) is
         begin
            for I in Rs'Range loop
               Rs (I) := abs (Xs (I) - Mean);
            end loop;
         end Errs;
      begin
         for I in 0 .. N_Good - 1 loop
            Add (-1.0 + 2.0 * (Long_Float (I) + 0.5) / Long_Float (N_Good));
         end loop;
         case K is
            when Cascade =>
               for T of Geom.Param_Vec'[1000.0, 100.0, 30.0, 10.0] loop
                  for M in 1 .. 3 loop
                     Add (T);
                  end loop;
               end loop;
            when Quarter =>
               for M in 1 .. 12 loop
                  Add (if M <= 8 then 50.0 else -50.0);
               end loop;
            when Majority =>
               for M in 0 .. 29 loop
                  Add (5.0 * 2.0 ** (Long_Float (M) / 3.0));
               end loop;
         end case;
         declare
            Sk : Geom.Flags (0 .. N - 1) := [others => False];
            Kept, Rounds : Natural;
            How : Geom.Reselect_End;
            Bad_In, Bad_Old : Natural := 0;
            Mean_New, Mean_Old : Long_Float;
            Still : Boolean := False;   --  旧写法交出来的那一批再挑一遍还变不变
            Old_Kept : Natural;
         begin
            Solve (Sk);
            Geom.Reselect_Loop (Errs'Access, Solve'Access, Sk, Kept, Rounds, How);
            Mean_New := Mean;
            for I in N_Good .. N - 1 loop
               if not Sk (I) then
                  Bad_In := Bad_In + 1;
               end if;
            end loop;
            --  旧写法:Cascade / Majority 按 Fit_Fixed_Board 的固定三遍;Quarter 按 Fit_Rig 的一遍 + "到四分之一全放回"
            declare
               So : Geom.Flags (0 .. N - 1) := [others => False];
               Rs : Geom.Param_Vec (0 .. N - 1);
               Ch : Boolean;
            begin
               Solve (So);
               if K = Quarter then
                  Errs (Rs);
                  Geom.Reselect (Rs, So, Ch, Old_Kept);
                  if 4 * (N - Old_Kept) >= N then
                     So := [others => False];
                  end if;
                  Solve (So);
               else
                  for R in 1 .. 3 loop
                     Errs (Rs);
                     Geom.Reselect (Rs, So, Ch, Old_Kept);
                     Solve (So);
                  end loop;
                  Errs (Rs);
                  declare
                     Probe : Geom.Flags := So;
                  begin
                     Geom.Reselect (Rs, Probe, Still, Old_Kept);
                  end;
               end if;
               Mean_Old := Mean;
               for I in N_Good .. N - 1 loop
                  if not So (I) then
                     Bad_Old := Bad_Old + 1;
                  end if;
               end loop;
            end;
            case K is
               when Cascade =>
                  Check (How = Geom.Settled and then Bad_In = 0 and then Kept = N_Good and then abs Mean_New < 1.0e-9 and then Rounds >= 4
                         and then Bad_Old > 0 and then Still,
                         "离群重挑·一层比一层小:挑到不再变(重解 " & Codec.Img (Rounds) & " 遍)⇒ 12 个错的全出去、平均 " & Codec.Fmt (Mean_New, 6)
                         & " · 牙:旧的固定三遍还留 " & Codec.Img (Bad_Old) & " 个、平均 " & Codec.Fmt (Mean_Old, 3) & "、再挑还在变");
               when Quarter =>
                  Check (How = Geom.Settled and then Bad_In = 0 and then abs Mean_New < 1.0e-9 and then Bad_Old = 12 and then abs Mean_Old > 1.0,
                         "离群重挑·三成错:" & Codec.Img (N - Kept) & " 个全出去、平均 " & Codec.Fmt (Mean_New, 6) & " · 牙:旧的踢到四分之一就全放回 ⇒ 平均 "
                         & Codec.Fmt (Mean_Old, 3));
               when Majority =>
                  Check (How = Geom.Broken and then 2 * Kept < N and then Bad_Old > 0 and then Still,
                         "离群重挑·错的占六成:挑到进解的只剩 " & Codec.Img (Kept) & " / " & Codec.Img (N) & " ⇒ 不是离群,照实报解不出 · 牙:旧的固定三遍交出平均 "
                         & Codec.Fmt (Mean_Old, 1) & " 的解(还留 " & Codec.Img (Bad_Old) & " 个错的、再挑还在变)");
            end case;
         end;
      end Run;
   begin
      Run (Cascade);
      Run (Quarter);
      Run (Majority);
   end;
   --  🔴 握区的手指像素随身体文件存、装回(Bodyfile,游程,2026-09-26):以前不存 ⇒ 装回身体后一个指尖都认不出,开机碰桌面量指尖直接"没量到"(X5C3)。
   --  合成:8×6 画面里两块手指(左上 2×3、右下 3×2)⇒ 存了再装回,每一格一样;指尖像素(Zone.Tip_Section)也一样
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
      Wd1, Th1, Wd2, Th2 : Long_Float := 0.0;
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
         Zone.Tip_Section (Z, Z.A, 8, 6, U1, V1, Wd1, Th1, Ok1);
         Zone.Tip_Section (H2 (0).Zones (0), H2 (0).Zones (0).A, 8, 6, U2, V2, Wd2, Th2, Ok2);
      end if;
      Check (Got and then Same and then Ok1 and then Ok2 and then U1 = U2 and then V1 = V2,
             "握区的手指像素随身体文件存、装回:" & (if Got then "装上了" else "没装上(" & To_String (Note) & ")") & " · 每一格" & (if Same then "一样" else "不一样")
             & " · 指尖像素 (" & Codec.Fmt (U1, 2) & "," & Codec.Fmt (V1, 2) & ") → (" & Codec.Fmt (U2, 2) & "," & Codec.Fmt (V2, 2) & ")");
   end;
   --  🔴 身体文件的数写得准、每条臂几个抓握通道跟着存(09-30):
   --  ① 原来按 Codec.Fmt 定点印:读数噪声 1e-7 印成 0.000000 读回来是 0、NaN 印成 nan、2.5e20 印成 inf —— 后两个不是 JSON,整份读不回来或读错一位;
   --     现在每个数写出去读回来一个比特不差,不是有限数的写成 null、读回来还是 NaN(手指深度读不到 = NaN,是个正常的"没量到")
   --  ② 原来不存 M.Jaws ⇒ 装回以后一律当 1 个(body_driver 的 `else 1`、act.adb 的 Jaws_Of):五指手第 1 号往后的握区全丢,存盘又把少了的写回去;
   --     旧文件没有这一项 ⇒ 照实说"没记,要重量",不按 1 个猜
   declare
      use Interfaces;
      function To_LF is new Ada.Unchecked_Conversion (Unsigned_64, Long_Float);
      NaN : constant Long_Float := To_LF (16#7FF8_0000_0000_0000#);
      M1, M2, M3 : Selfmap.Body_Map;
      H1, H2, H3 : Zone.Hand_Vectors.Vector;
      T1, T2, T3 : Act.Effect_Vectors.Vector;
      S1, S2, S3 : Schema.Map;
      Note2, Note3 : Unbounded_String;
      Got2, Got3, Valid_Json : Boolean := False;
      Path : constant String := "/tmp/bd_selfcheck_body_exact.json";
      Old_Path : constant String := "/tmp/bd_selfcheck_body_nojaws.json";
      Text : Unbounded_String;
      D : Json.Doc;
      E : Unbounded_String;
      Old_Count : Natural := 0;
      Exact_Ok, Hands_Ok : Boolean := False;
      function Read_All (P : String) return Unbounded_String is
         F : File_Type;
         R : Unbounded_String;
      begin
         Open (F, In_File, P);
         while not End_Of_File (F) loop
            Append (R, Get_Line (F));
         end loop;
         Close (F);
         return R;
      end Read_All;
   begin
      M1.Arms := 1; M1.N_Cams := 1; M1.Per_Arm := Chan.Per_Arm; M1.Channels := Chan.Per_Arm;
      for Ch in 0 .. Chan.Per_Arm - 1 loop
         M1.Amp.Append (0.0065 + Long_Float (Ch) * 1.0e-9); M1.Delivered.Append (1.0 / 3.0);   --  合成:第 6 位以后才不一样、除不尽
      end loop;
      M1.Cam_On_Arm.Append (0);
      M1.EE_Noise := 1.0e-7; M1.Rot_Noise := NaN; M1.Jaw_Noise := 2.5e20;
      M1.Jaws.Append (5);   --  一条五指手
      for K in 0 .. 4 loop
         declare
            H : Zone.Hand;
            Z : Zone.Hand_Zone;
         begin
            Z.Valid := True; Z.N_Lobes := 1; Z.Cu := 0.1 * Long_Float (K + 1); Z.Cv := 0.5;
            Z.Depth := (if K = 2 then NaN else 0.3);   --  第 2 根手指的深度读不到
            Z.A := (True, 0, 0, 1, 1, 0.1, 0.2, 4);
            for I in 0 .. 8 * 6 - 1 loop
               Z.Fingers.Append (I mod 5 = K);
            end loop;
            H.Arm := 0; H.K := K; H.Zones.Append (Z);
            H1.Append (H);
         end;
      end loop;
      Bodyfile.Save (Path, "selfcheck", M1, H1, T1, S1);
      Text := Read_All (Path);
      Valid_Json := Json.Parse (To_String (Text), D, E);
      Got2 := Bodyfile.Load (Path, "selfcheck", M2, H2, T2, S2, Note2);
      Exact_Ok := Got2 and then M2.EE_Noise = 1.0e-7 and then M2.Rot_Noise /= M2.Rot_Noise and then M2.Jaw_Noise = 2.5e20
        and then Natural (M2.Amp.Length) = Chan.Per_Arm
        and then (for all Ch in 0 .. Chan.Per_Arm - 1 => M2.Amp (Ch) = M1.Amp (Ch) and then M2.Delivered (Ch) = M1.Delivered (Ch));
      Hands_Ok := Got2 and then Bodyfile.Jaws_Recorded (M2) and then M2.Jaws (0) = 5 and then Natural (H2.Length) = 5
        and then (for all K in 0 .. 4 => H2 (K).K = K and then H2 (K).Zones (0).Cu = H1 (K).Zones (0).Cu)
        and then H2 (2).Zones (0).Depth /= H2 (2).Zones (0).Depth and then H2 (1).Zones (0).Depth = 0.3;
      --  09-30 以前的文件:没有 "jaws" 这一项
      declare
         Tag : constant String := """jaws"":[5],";
         P : constant Natural := Index (Text, Tag);
         Old_Text : Unbounded_String := Text;
         F : File_Type;
      begin
         if P > 0 then
            Delete (Old_Text, P, P + Tag'Length - 1);
         end if;
         Create (F, Out_File, Old_Path);
         Put (F, To_String (Old_Text));
         Close (F);
      end;
      Got3 := Bodyfile.Load (Old_Path, "selfcheck", M3, H3, T3, S3, Note3);
      --  牙:旧写法 body_driver 里"这条臂合空几次"那一句,装回这份旧文件得 1
      Old_Count := (if 0 < Natural (M3.Jaws.Length) then Natural'Max (1, M3.Jaws (0)) else 1);
      Check (Valid_Json and then Exact_Ok and then Codec.Fmt (1.0e-7, 6) = "0.000000" and then Codec.Fmt (2.5e20, 6) = "inf" and then Codec.Fmt (NaN, 6) = "nan",
             "身体文件的数写出去读回来一个比特不差:噪声 1e-7、2.5e20、1/3 都一样,NaN 写成 null 读回来还是 NaN,整份是合法 JSON"
             & "(旧写法印成 0.000000、inf、nan)");
      Check (Hands_Ok,
             "身体文件记下每条臂几个抓握通道:五指手存 5 装回 5,五根手指的握区全回来(第 2 根读不到的深度还是 NaN)"
             & (if Got2 then "" else "(没装上:" & To_String (Note2) & ")"));
      Check (Got3 and then not Bodyfile.Jaws_Recorded (M3) and then Index (Note3, "没记") > 0 and then Old_Count = 1,
             "旧的身体文件没记抓握通道数 ⇒ 装回时照实说要重量:" & To_String (Note3) & "(旧写法按 1 个合空,五指手只剩第 0 号)");
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
               Seen : Boolean;
               D : constant Geom.V3 := Geom.Cam_Dir (Gd, U0, V0, Seen);
               U, V : Long_Float;
               Fr : Boolean;
            begin
               Geom.Cam_Pixel (Gd, D, U, V, Fr);
               Worst := Long_Float'Max (Worst, (if Fr and then Seen then abs (U - U0) + abs (V - V0) else 1.0e9));   --  这个镜头一直单调:每个像素都得去得了
            end;
         end loop;
      end loop;
      declare
         Dd_Ok : Boolean;
         Dd : constant Geom.V3 := Geom.Cam_Dir (Gd, 0.0, 0.0, Dd_Ok);
         Dp0 : constant Geom.V3 := [(0.0 - 320.0) / 397.0, -(0.0 - 240.0) / 397.0, -1.0];
         Np : constant Long_Float := Geom.Norm (Dp0);
         Cs : constant Long_Float := (Dd (0) * Dp0 (0) + Dd (1) * Dp0 (1) + Dd (2) * Dp0 (2)) / Np;
      begin
         Corner_Deg := Arccos (Long_Float'Min (1.0, Cs)) * 57.29578;   --  弧度 → 度(换算,无量纲)
      end;
      Check (Worst < 1.0e-6 and then Corner_Deg > 1.0,
             "镜头畸变:像素 → 视线 → 像素,最大差 " & Codec.Fmt (Worst, 9) & " px · 角上那一点去畸变的视线比理想针孔的偏 " & Codec.Fmt (Corner_Deg, 2) & "°");
   end;
   --  🔴 去畸变改成一维牛顿法、去不了照实报(09-30,Geom.Cam_Dir 的 Ok):强桶形 K1 = −0.35(合成;折回半径 r* = 1/√(−3K1) = 0.976,
   --  那儿畸变后的半径最大 = 0.6506)。① 折回半径以内、靠近它的三个点(r = 0.95 / 0.97 / 0.975)解回的视线和真的差 < 1e-9 弧度;
   --  牙:旧写法(不动点迭代,50 次封顶)同一组像素差 4e-4 ~ 6e-3 弧度也不报 —— 迭代的斜率在 r* 处正好是 1。
   --  ② 畸变后离主点 0.66(比能到的 0.6506 还远,没有哪条视线落在这儿)⇒ Ok = False、零向量,不带 Ok 的 Cam_Dir / Ray 也是零向量,
   --  Hit_Plane 不拿它当视线;牙:旧写法给出 r = 2.39 的"视线",投回去离这个像素一千多像素。③ K1 = −0.2、F = 397 的镜头:
   --  画幅角上(畸变后 1.0 > 能到的 0.861)照实说去不了,画面中心去得了
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Gk : Geom.Cam_Geo;
      Phi : constant Long_Float := 0.5;   --  方位(弧度,合成)
      type Radii is array (Positive range <>) of Long_Float;
      Near_Fold : constant Radii := [0.95, 0.97, 0.975];
      --  旧写法原样(09-30 以前的 Geom.Undistort)
      procedure Old_Undistort (K1, Xd, Yd : Long_Float; X, Y : out Long_Float) is
      begin
         X := Xd; Y := Yd;
         for It in 1 .. 50 loop
            declare
               D : constant Long_Float := 1.0 + K1 * (X * X + Y * Y);
               Xn, Yn : Long_Float;
            begin
               exit when D <= 0.0;
               Xn := Xd / D; Yn := Yd / D;
               exit when abs (Xn - X) + abs (Yn - Y) < 1.0e-12;
               X := Xn; Y := Yn;
            end;
         end loop;
      end Old_Undistort;
      function Ang (A, B : Geom.V3) return Long_Float is
        (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, (A (0) * B (0) + A (1) * B (1) + A (2) * B (2)) / (Geom.Norm (A) * Geom.Norm (B))))));
      New_Worst, Old_Best : Long_Float := 0.0;
      All_Seen : Boolean := True;
   begin
      Gk.F := 400.0; Gk.Cx := 320.0; Gk.Cy := 240.0; Gk.K1 := -0.35;
      Old_Best := Long_Float'Last;
      for R of Near_Fold loop
         declare
            Rd : constant Long_Float := R * (1.0 + Gk.K1 * R * R);
            Truth : constant Geom.V3 := [R * Cos (Phi), R * Sin (Phi), -1.0];
            Seen : Boolean;
            D : constant Geom.V3 := Geom.Cam_Dir (Gk, Gk.Cx + Gk.F * Rd * Cos (Phi), Gk.Cy - Gk.F * Rd * Sin (Phi), Seen);
            Xo, Yo : Long_Float;
         begin
            All_Seen := All_Seen and then Seen;
            New_Worst := Long_Float'Max (New_Worst, (if Seen then Ang (D, Truth) else 1.0));
            Old_Undistort (Gk.K1, Rd * Cos (Phi), Rd * Sin (Phi), Xo, Yo);
            Old_Best := Long_Float'Min (Old_Best, Ang ([Xo, Yo, -1.0], Truth));
         end;
      end loop;
      Check (All_Seen and then New_Worst < 1.0e-9 and then Old_Best > 1.0e-4,
             "去畸变(牛顿):K1 −0.35 折回半径 0.976 以内 r = 0.95 / 0.97 / 0.975 解回的视线最多差 " & Codec.Fmt (New_Worst, 12)
             & " 弧度 · 牙:旧的不动点迭代(50 次封顶)同一组最少也差 " & Codec.Fmt (Old_Best, 5) & " 弧度,也不报");
      declare
         Rd : constant Long_Float := 0.66;   --  畸变后离主点(归一化,合成;能到的最大是 0.6506)
         U : constant Long_Float := Gk.Cx + Gk.F * Rd * Cos (Phi);
         V : constant Long_Float := Gk.Cy - Gk.F * Rd * Sin (Phi);
         Seen : Boolean;
         D : constant Geom.V3 := Geom.Cam_Dir (Gk, U, V, Seen);
         Dw : constant Geom.V3 := Geom.Ray (Gk, [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0], U, V);
         Hok : Boolean;
         Hit : constant Geom.V3 := Geom.Hit_Plane ([0.0, 0.0, 1.0], Dw, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], Hok);
         pragma Unreferenced (Hit);
         Xo, Yo, Uo, Vo : Long_Float;
         Fr : Boolean;
         Gw : constant Geom.Cam_Geo := (Gk with delta F => 397.0, K1 => -0.2);
         Seen_Corner, Seen_Center : Boolean;
         Corner : constant Geom.V3 := Geom.Cam_Dir (Gw, 0.0, 0.0, Seen_Corner);
         Center : constant Geom.V3 := Geom.Cam_Dir (Gw, 320.0, 240.0, Seen_Center);
      begin
         Old_Undistort (Gk.K1, Rd * Cos (Phi), Rd * Sin (Phi), Xo, Yo);
         Geom.Cam_Pixel (Gk, [Xo, Yo, -1.0], Uo, Vo, Fr);
         Check (not Seen and then Geom.Norm (D) = 0.0 and then Geom.Norm (Dw) = 0.0 and then not Hok and then Sqrt ((Uo - U) ** 2 + (Vo - V) ** 2) > 100.0
                and then not Seen_Corner and then Geom.Norm (Corner) = 0.0 and then Seen_Center and then abs (Center (2) + 1.0) < 1.0e-12,
                "去畸变:畸变后 0.66 比这个镜头能到的 0.6506 还远 ⇒ 去不了、零向量,Ray 也是零、交不了面 · 牙:旧写法给出 r = "
                & Codec.Fmt (Sqrt (Xo * Xo + Yo * Yo), 2) & " 的视线,投回去离这个像素 " & Codec.Fmt (Sqrt ((Uo - U) ** 2 + (Vo - V) ** 2), 0)
                & " px · K1 −0.2 的镜头画幅角上去不了、中心去得了");
      end;
   end;
   --  🔴 整幅掩膜 ⇒ 框、像素数、形心、主轴(Picture.Region_Of_Mask,SAM 出掩膜后用):合成 40×30 画幅里一条 20×4 的横条(x 10..29、y 5..8)
   --  ⇒ 框 [10 5 29 8]、80 px、形心 (19.5, 6.5)、主轴水平、伸长比 = 长 ÷ 宽 = 5(像素当单位方块);空掩膜 ⇒ 不成
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
             and then abs (40.0 * Rg.Cu - 19.5) < 1.0e-9 and then abs (30.0 * Rg.Cv - 6.5) < 1.0e-9 and then abs (Rg.Av) < 1.0e-9 and then abs (Rg.Elong - 5.0) < 1.0e-9 and then not Ok0,
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
   --  🔴 装回身体文件(⑤,Act.Geo_Install Keep_Tips):几何文件里存的指尖、张口、步幅(一条命令走多远 / 转多远)都并回来 ——
   --  09-28 S1A1(剪刀正式一集 200 拍):只并了指尖和张口,步幅在这一集里整套重量,吃掉 122 拍;不装回(从零量)时一样都不并
   declare
      C1, C2 : Act.Context;
      Path : constant String := "/tmp/bd_selfcheck_reload";
      Stored, Fresh : Geom.Geo_Vectors.Vector;
      G : Geom.Cam_Geo;
      F1 : Plug.Frame;
      Ref : Plug.Cam;
      No_Board, One_Board : Geom.Scene_Pt_Vectors.Vector;
      C3 : Act.Context;
   begin
      One_Board.Append (Geom.Scene_Pt'(Pw => [0.1, 0.2, 0.0], U => 10.0, V => 20.0, Sh => 0.0, Views => 3, Cov => [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]]));
      G.Valid := True; G.F := 397.0; G.Cx := 320.0; G.Cy := 240.0;
      Fresh.Append (G);   --  前半段装回的那份:还没有指尖、步幅
      G.Tip := [0.87, -0.24, -1.75]; G.Tip_Valid := True; G.Tip_Touch := True; G.Gap := 1.766;
      G.Stride := 0.888; G.Stride_Rot := 0.161;
      G.Lobes.Append (Geom.Lobe_Geo'(Tip => [0.0123, -0.2345, -1.6875], Wide => 0.1932, Thin => 0.0317));
      G.Lobes.Append (Geom.Lobe_Geo'(Tip => [-0.0071, -0.2468, 1.7011], Wide => 0.2011, Thin => 0.0299));
      G.Tip_Sd := 0.00417;
      Stored.Append (G);
      Geom.Save (Path & ".geo.json", Stored);
      Act.Geo_Install (F1, C1, Path, Fresh, No_Board, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.001, Ref, Keep_Tips => True);
      Act.Geo_Install (F1, C2, Path, Fresh, No_Board, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.001, Ref, Keep_Tips => False);
      Check (C1.Geo (0).Tip_Valid and then abs (C1.Geo (0).Gap - 1.766) < 1.0e-4 and then abs (C1.Geo (0).Stride - 0.888) < 1.0e-4
             and then abs (C1.Geo (0).Stride_Rot - 0.161) < 1.0e-4 and then not C2.Geo (0).Tip_Valid and then C2.Geo (0).Stride = 0.0
             and then Natural (C1.Geo (0).Lobes.Length) = 2 and then C2.Geo (0).Lobes.Is_Empty and then abs (C1.Geo (0).Tip_Sd - 0.00417) < 1.0e-6
             and then abs (C1.Geo (0).Lobes (1).Tip (2) - 1.7011) < 1.0e-6 and then abs (C1.Geo (0).Lobes (0).Wide - 0.1932) < 1.0e-6
             and then abs (C1.Geo (0).Lobes (1).Thin - 0.0299) < 1.0e-6,
             "装回身体文件:存的指尖、张口、步幅(" & Codec.Fmt (C1.Geo (0).Stride, 3) & " / 转 " & Codec.Fmt (C1.Geo (0).Stride_Rot, 3)
             & ")、每一瓣的尖和截面(" & Codec.Img (Natural (C1.Geo (0).Lobes.Length)) & " 瓣)都并回来,开机不重量 · 从零量时一样都不并");
      --  有板 ⇒ 东西躺的面装上就是板的那张(S1A1:没登记,不动的眼看见的东西被按指尖高度放到了空中);没板 ⇒ 不登记
      Act.Geo_Install (F1, C3, Path, Fresh, One_Board, [0.0, 0.0, 0.765], [0.0, 0.0, 1.0], 0.001, Ref, Keep_Tips => True);
      Check (C3.Touch_Valid and then abs (C3.Touch_Pt (2) - 0.765) < 1.0e-9 and then abs (C3.Touch_N (2) - 1.0) < 1.0e-9 and then not C3.Touch_Fresh
             and then not C1.Touch_Valid,
             "装回身体文件:有板 ⇒ 东西躺的面装上就是板的那张(还不算这一集碰过)· 没板 ⇒ 不登记");
   end;
   --  🔴 一条命令转得到的最大一档按运动学算(09-28 S1A2,Act.Kin_Turn_Reach):假反解只让从起点转到 1.2 弧度以内 ⇒ 从一档 0.01 起翻倍,
   --  最后一档转得到的是 0.64(1.28 转不到就停);没有运动学(没挂反解)⇒ 0。原来按阶梯只推到第三档(64 倍一档)就停,阶梯的顶当成了身体的顶
   declare
      P0 : constant Plug.Arm_Pose := [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
      procedure Fake_Reach (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
         pragma Unreferenced (Arm);
      begin
         Pos_Err := 0.0;
         --  两个朝向的夹角 = 2·arccos|q₀·q|(四元数;自检自己算,驱动里原来那个 Geom.Angle_Between 09-30 没人调了、删了)
         Rot_Err := (if 2.0 * Ada.Numerics.Long_Elementary_Functions.Arccos (Long_Float'Min (1.0, abs (P0 (3) * Pose (3) + P0 (4) * Pose (4) + P0 (5) * Pose (5) + P0 (6) * Pose (6))))
                        <= 1.2 then 0.0 else 1.0);
      end Fake_Reach;
      R1, R0 : Long_Float;
   begin
      Plug.Set_Reach (Fake_Reach'Unrestricted_Access);
      R1 := Act.Kin_Turn_Reach (0, P0, 0.01, 0.001, 0.001);
      Plug.Set_Reach (null);
      R0 := Act.Kin_Turn_Reach (0, P0, 0.01, 0.001, 0.001);
      Check (abs (R1 - 0.64) < 1.0e-9 and then R0 = 0.0,
             "一条命令转得到的最大一档按运动学问反解:反解只让转到 1.2 弧度 ⇒ " & Codec.Fmt (R1, 3) & " 弧度(翻倍的最后一档)· 没有运动学 ⇒ " & Codec.Fmt (R0, 3));
   end;
   --  🔴 换倾角碰量指尖(2026-09-28,Geom.Tilt_Dir / Turn_To / Press_Of / Fit_Presses):合成的手 —— 两个指尖是半径 5 mm 的球(球心在眼前 79 mm、
   --  左右 ±45 mm,同 x5 的量级)、手掌三点;每一下让 Tilt_Dir 那个方向(这一瓣的视线朝方位 Azim 斜 θ,θ = 两瓣视线夹角的三分之一)转到朝正下,
   --  往下落到手上真的最低那一点碰到面,接触高度加 ±0.2 mm 的噪声;一瓣压 6 下(朝下 1 下 + 方位 0 / 72 / 144 / 216 / 288° 各 1 下)。
   --  门 1 mm:横向只靠 sin θ,±0.2 mm 的高度噪声斜 20° 时放大 2–2.5 倍(5 个斜的 / 去掉一下剩 4 个斜的)⇒ 0.4–0.9 mm。
   --  ① 6 下全好:每瓣解回离真的尖(视线朝下时球上最低那一点)< 1 mm;② 斜着的一下被别的东西先顶住(停高 5 mm)⇒ 认出、去掉,解 < 1 mm;
   --  ③ 朝下那一下被顶住 ⇒ 同样认出;④ 8 下里两下被顶住 ⇒ 认出(< 1 mm)或不收,不许收错的;⑤ 另一瓣压的那几下当"这一瓣不许在面之下"核:
   --  好的解都满足,假造一下"那一刻这一瓣的尖在面之下 5 mm" ⇒ 不收;⑥ 斜 θ 的每一下里真的最低点都是压的这一瓣(另一根手指、手掌都更高,纯几何);
   --  ⑦ 只按组里残差收会收错(原来的写法):斜着的一下被顶住 5 mm 时 4 下的组里残差都在一小步以内 —— 现在按"别的几下预测它"⇒ 不收
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Gt : Geom.Cam_Geo;   --  R_Ce = 单位、Off = 0 ⇒ 手的位姿就是眼的位姿(只读关节那一路)
      Table_Z : constant Long_Float := 0.765;   --  面高(米,合成)
      Nn : constant Geom.V3 := [0.0, 0.0, 1.0];
      P0 : constant Geom.V3 := [0.0, 0.0, Table_Z];
      Down : constant Geom.V3 := [0.0, 0.0, -1.0];
      Rt : constant Long_Float := 0.005;   --  指尖球的半径(米,合成)
      type V3_Arr is array (Natural range <>) of Geom.V3;
      Ctr : constant V3_Arr := [[0.045, 0.0, -0.079], [-0.045, 0.0, -0.079]];                 --  指尖球心(相机系,米,合成)
      Palm : constant V3_Arr := [[0.0, 0.0, -0.03], [0.0, 0.02, -0.03], [0.0, -0.02, -0.03]];  --  手掌(相机系,米,合成)
      Noise : constant array (0 .. 7) of Long_Float := [0.00015, -0.0002, 0.0001, -0.00005, 0.0002, -0.00015, 0.00005, 0.0001];   --  接触高度噪声(米,合成)
      Gate : constant Long_Float := 0.002;   --  一小步(米,合成)
      Block : constant Long_Float := 0.005;  --  被别的东西先顶住,停高 5 mm(米,合成)
      Tol : constant Long_Float := 0.001;    --  解回的门 1 mm(米,见上)
      Home : constant Plug.Arm_Pose := [0.1, -0.2, 1.0, 1.0, 0.0, 0.0, 0.0];
      Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算)
      --  朝下那一下之后的方位:五个各差 72°,再补压两个方位中间的(36°、108°)
      Az : constant array (0 .. 6) of Long_Float := [0.0, 72.0 * Deg, 144.0 * Deg, 216.0 * Deg, 288.0 * Deg, 36.0 * Deg, 108.0 * Deg];
      function Dir (K : Natural) return Geom.V3 is
        ([Ctr (K) (0) / Geom.Norm (Ctr (K)), Ctr (K) (1) / Geom.Norm (Ctr (K)), Ctr (K) (2) / Geom.Norm (Ctr (K))]);
      Beta : constant Long_Float := Arccos (Dir (0) (0) * Dir (1) (0) + Dir (0) (1) * Dir (1) (1) + Dir (0) (2) * Dir (1) (2));
      Dirs : Geom.V3_Vectors.Vector;
      Theta : Long_Float;   --  同驱动:Geom.Tilt_Angle
      Lowest_Ok : Boolean := True;   --  ⑥
      --  一下:压第 K 瓣,斜 Tilt、方位 Azim;真的最低点碰面,Up_By = 被别的东西先顶住、停高多少
      function Press (K : Natural; Tilt, Azim, Up_By, Nz : Long_Float) return Geom.Press_Eq is
         Rv : constant Geom.V3 := Geom.Turn_To (Geom.Tilt_Dir (Dir (K), Tilt, Azim), Down);   --  起点的位姿不转 ⇒ 相机系 = 世界系
         Av : Table.Vec := Table.Zero_Vec;
         P : Plug.Arm_Pose;
         R : Geom.M3;
         Low : Long_Float := Long_Float'Last;
         Who : Integer := -1;
      begin
         Av (3) := Rv (0); Av (4) := Rv (1); Av (5) := Rv (2);
         P := Chan.Compose (Home, Av);
         R := Geom.Quat_To_R (P);
         for I in Ctr'Range loop
            if Geom.Ap (R, Ctr (I)) (2) - Rt < Low then
               Low := Geom.Ap (R, Ctr (I)) (2) - Rt; Who := I;
            end if;
         end loop;
         for Q of Palm loop
            if Geom.Ap (R, Q) (2) < Low then
               Low := Geom.Ap (R, Q) (2); Who := 99;
            end if;
         end loop;
         if Who /= K then
            Lowest_Ok := False;
         end if;
         P (2) := Table_Z - Low + Up_By + Nz;
         return Geom.Press_Of (Gt, P, P0, Nn);
      end Press;
      function Truth (K : Natural) return Geom.V3 is ([Ctr (K) (0) + Rt * Dir (K) (0), Ctr (K) (1) + Rt * Dir (K) (1), Ctr (K) (2) + Rt * Dir (K) (2)]);
      function Err (F : Geom.Press_Fit; K : Natural) return Long_Float is
        (Geom.Norm ([F.X (0) - Truth (K) (0), F.X (1) - Truth (K) (1), F.X (2) - Truth (K) (2)]));
      --  第 K 瓣压 N_Press 下(第 0 下朝下,其后按 Az 的次序斜),Bad = 哪几下被顶住(下标;-1 = 没有)
      function Presses (K, N_Press : Natural; Bad1, Bad2 : Integer := -1) return Geom.Press_Eq_Vectors.Vector is
         E : Geom.Press_Eq_Vectors.Vector;
      begin
         for I in 0 .. N_Press - 1 loop
            E.Append (Press (K, (if I = 0 then 0.0 else Theta), (if I = 0 then 0.0 else Az (I - 1)),
                             (if I = Bad1 or else I = Bad2 then Block else 0.0), Noise ((I + 3 * K) mod Noise'Length)));
         end loop;
         return E;
      end Presses;
      F0, F1, Fb, Fn, F2b, Fo, Fx, F4 : Geom.Press_Fit;
      With_Other, Bogus : Geom.Press_Eq_Vectors.Vector;
      Raw_Would : Boolean := True;   --  ⑦:原来的写法(4 下、组里残差 ≤ 一小步)会不会收那一下被顶住的
   begin
      Gt.Valid := True; Gt.F := 397.0; Gt.Cx := 320.0; Gt.Cy := 240.0;
      Dirs.Append (Dir (0)); Dirs.Append (Dir (1));
      Theta := Geom.Tilt_Angle (Dirs, 0, 0.0);
      F0 := Geom.Fit_Presses (Presses (0, 6), Gate);
      F1 := Geom.Fit_Presses (Presses (1, 6), Gate);
      Fb := Geom.Fit_Presses (Presses (0, 6, Bad1 => 3), Gate);
      Fn := Geom.Fit_Presses (Presses (0, 6, Bad1 => 0), Gate);
      F2b := Geom.Fit_Presses (Presses (0, 8, Bad1 => 2, Bad2 => 5), Gate);
      F4 := Geom.Fit_Presses (Presses (0, 4, Bad1 => 2), Gate);
      --  ⑦ 原来的写法:4 下一起按最小二乘解,组里每一下的残差都 ≤ 一小步就收
      declare
         E : constant Geom.Press_Eq_Vectors.Vector := Presses (0, 4, Bad1 => 2);
         M : Geom.M3 := [others => [others => 0.0]];
         V : Geom.V3 := [others => 0.0];
         X : Geom.V3;
      begin
         for Q of E loop
            for R in 0 .. 2 loop
               for S in 0 .. 2 loop
                  M (R, S) := M (R, S) + Q.A (R) * Q.A (S);
               end loop;
               V (R) := V (R) + Q.A (R) * Q.B;
            end loop;
         end loop;
         X := Geom.Solve3 (M, V);
         for Q of E loop
            if abs (Q.A (0) * X (0) + Q.A (1) * X (1) + Q.A (2) * X (2) - Q.B) > Gate then
               Raw_Would := False;
            end if;
         end loop;
      end;
      With_Other := Presses (0, 6);
      for E of Presses (1, 6) loop
         With_Other.Append (Geom.Press_Eq'(A => E.A, B => E.B, Aimed => False));
      end loop;
      Fo := Geom.Fit_Presses (With_Other, Gate);
      Bogus := With_Other;
      --  另一瓣压的第 2 下改成"那一刻这一瓣真的尖在面之下 5 mm"(B = A·真的尖 + 5 mm)
      Bogus.Replace_Element (7, Geom.Press_Eq'(A => Bogus (7).A, B => Bogus (7).A (0) * Truth (0) (0) + Bogus (7).A (1) * Truth (0) (1) + Bogus (7).A (2) * Truth (0) (2) + Block,
                                               Aimed => False));
      Fx := Geom.Fit_Presses (Bogus, Gate);
      Check (F0.Ok and then F1.Ok and then Natural (F0.Used.Length) = 6 and then Err (F0, 0) < Tol and then Err (F1, 1) < Tol,
             "换倾角碰:两瓣视线夹角 " & Codec.Fmt (Beta / Deg, 1) & "°、斜 θ = " & Codec.Fmt (Theta / Deg, 1)
             & "° ⇒ 6 下全好,两瓣解回离真的尖 " & Codec.Fmt (1000.0 * Err (F0, 0), 3) & " / " & Codec.Fmt (1000.0 * Err (F1, 1), 3) & " mm(< 1)· 别的几下预测每一下最多差 "
             & Codec.Fmt (1000.0 * F0.Worst, 3) & " mm · 不确定度 (" & Codec.Fmt (1000.0 * F0.Sd (0), 2) & "," & Codec.Fmt (1000.0 * F0.Sd (1), 2) & "," & Codec.Fmt (1000.0 * F0.Sd (2), 2) & ") mm");
      Check (Fb.Ok and then not Fb.Used.Contains (3) and then Err (Fb, 0) < Tol and then Fn.Ok and then not Fn.Used.Contains (0) and then Err (Fn, 0) < Tol,
             "换倾角碰:一下被顶住(停高 5 mm)⇒ 认出、去掉:斜的那一下 " & (if Fb.Ok then Codec.Fmt (1000.0 * Err (Fb, 0), 3) & " mm(用了 " & Codec.Img (Natural (Fb.Used.Length)) & " 下)" else "没收")
             & "、朝下那一下 " & (if Fn.Ok then Codec.Fmt (1000.0 * Err (Fn, 0), 3) & " mm(用了 " & Codec.Img (Natural (Fn.Used.Length)) & " 下)" else "没收"));
      Check ((not F2b.Ok or else Err (F2b, 0) < Tol) and then (not F4.Ok or else Err (F4, 0) < Tol),
             "换倾角碰:8 下里两下被顶住 ⇒ " & (if F2b.Ok then "认出,解 " & Codec.Fmt (1000.0 * Err (F2b, 0), 3) & " mm(用了 " & Codec.Img (Natural (F2b.Used.Length)) & " 下)" else "不收" & (if F2b.Ambiguous then "(认不出)" else ""))
             & " · 只压 4 下、斜的一下被顶住 ⇒ " & (if F4.Ok then "收了,解 " & Codec.Fmt (1000.0 * Err (F4, 0), 2) & " mm" else "不收(补压)"));
      Check (Fo.Ok and then Err (Fo, 0) < Tol and then Fo.Low > 0.0 and then not Fx.Ok,
             "换倾角碰:另一瓣压的 6 下当核 ⇒ 这一瓣在那几下里都在面之上(最低 " & Codec.Fmt (1000.0 * Fo.Low, 1) & " mm)、解照样 "
             & Codec.Fmt (1000.0 * Err (Fo, 0), 3) & " mm · 假造一下它在面之下 5 mm ⇒ " & (if Fx.Ok then "收了(错)" else "不收"));
      Check (Lowest_Ok, "换倾角碰:斜 θ = 夹角的三分之一的每一下里,真的最低点都是压的这一瓣(另一根手指、手掌都更高)");
      --  ⑧ 另一根手指长得多(球心沿它的视线往外挪):压第 0 瓣斜着的几下里先碰到的可能是它。长 30 mm:它只比这一瓣低一点点(比一小步小,认不出),
      --  解照样是这一瓣的尖,差要在 V1 的考试线 5 mm 以内(或不收);长 80 mm:它先碰到时停高几厘米 ⇒ 那几下被当成停早了去掉,
      --  解是这一瓣的尖(< 5 mm)、或不收、或解离它那条视线更近被 Ray_Owner 认出 —— 不许把它的尖当成第 0 瓣的
      declare
         V1_Line : constant Long_Float := 0.005;   --  V1 每瓣的考试线 5 mm(米,PLAN)
         procedure Long_Finger (Extra : Long_Float) is
            Long_Ctr : constant V3_Arr := [Ctr (0), [Ctr (1) (0) + Extra * Dir (1) (0), Ctr (1) (1) + Extra * Dir (1) (1), Ctr (1) (2) + Extra * Dir (1) (2)]];
            E : Geom.Press_Eq_Vectors.Vector;
            Fl : Geom.Press_Fit;
            Other_Low : Natural := 0;   --  斜着的几下里另一根手指先碰到的下数
         begin
            for I in 0 .. 5 loop
               declare
                  Rv : constant Geom.V3 := Geom.Turn_To (Geom.Tilt_Dir (Dir (0), (if I = 0 then 0.0 else Theta), (if I = 0 then 0.0 else Az (I - 1))), Down);
                  Av : Table.Vec := Table.Zero_Vec;
                  P : Plug.Arm_Pose;
                  R : Geom.M3;
                  Low : Long_Float := Long_Float'Last;
                  Who : Natural := 0;
               begin
                  Av (3) := Rv (0); Av (4) := Rv (1); Av (5) := Rv (2);
                  P := Chan.Compose (Home, Av);
                  R := Geom.Quat_To_R (P);
                  for J in Long_Ctr'Range loop
                     if Geom.Ap (R, Long_Ctr (J)) (2) - Rt < Low then
                        Low := Geom.Ap (R, Long_Ctr (J)) (2) - Rt; Who := J;
                     end if;
                  end loop;
                  if Who = 1 then
                     Other_Low := Other_Low + 1;
                  end if;
                  P (2) := Table_Z - Low + Noise (I);
                  E.Append (Geom.Press_Of (Gt, P, P0, Nn));
               end;
            end loop;
            Fl := Geom.Fit_Presses (E, Gate);
            Check (Other_Low > 0 and then (not Fl.Ok or else Geom.Ray_Owner (Fl.X, Dirs) /= 0 or else Err (Fl, 0) < V1_Line),
                   "换倾角碰:另一根手指长 " & Codec.Fmt (1000.0 * Extra, 0) & " mm ⇒ 压第 0 瓣的 6 下里 " & Codec.Img (Other_Low) & " 下先碰到的是它 ⇒ "
                   & (if not Fl.Ok then "几下对不上、不收" elsif Geom.Ray_Owner (Fl.X, Dirs) /= 0 then "收下的解离第 1 瓣的视线更近 ⇒ 认出碰着的不是这一瓣"
                      else "解是这一瓣的尖,差 " & Codec.Fmt (1000.0 * Err (Fl, 0), 1) & " mm(用了 " & Codec.Img (Natural (Fl.Used.Length)) & " 下,自报不确定度 ("
                           & Codec.Fmt (1000.0 * Fl.Sd (0), 2) & "," & Codec.Fmt (1000.0 * Fl.Sd (1), 2) & "," & Codec.Fmt (1000.0 * Fl.Sd (2), 2) & ") mm)"));
         end Long_Finger;
      begin
         Long_Finger (0.03);
         Long_Finger (0.08);
      end;
      Check (Raw_Would and then not F4.Ok,
             "换倾角碰:原来的写法(4 下、组里残差 ≤ 一小步)" & (if Raw_Would then "会收下被顶住 5 mm 的那一组" else "不收(焊点前提不成立)")
             & " —— 按别的几下预测它 ⇒ " & (if F4.Ok then "也收了(错)" else "不收"));
   end;
   --  🔴 瓣按"长在眼上"补全(Zone.Refine_Probes / Zone.Apply_Refine,09-30 V1B69:左边那根手指上半截贴着暗墙、变化掩码里没有,尖认低 46 px)。
   --  合成的眼 640 × 480(焦距 400、主点正中);手指从画面下边伸进来(同 x5 腕眼):右瓣框 x 540–619、y 300–479,左瓣框 x 20–99、y 300–479,
   --  合到的区(区框)x 250–400、y 350–479。真的手指:左 x 20–99、y ≥ 240(变化掩码只有 y ≥ 300 那一截,上面 60 px 缺了),右 x 530–619、y ≥ 300(内侧一溜也缺了);
   --  眼绕自己的竖直轴转 0.16 弧度:手指像素配到原处,别的按这个转动挪(Classify_Rides 拿同一个转动判)。
   --  要:左瓣的尖(Tip_Section)从 y > 300 挪到 y < 270;右瓣的尖留在右边(u > 500);区框里一个像素都不补。
   --  🦷 不补:左瓣的尖还在 y > 300
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Wd : constant := 640;
      Ht : constant := 480;
      Z : Zone.Hand_Zone;
      Eye : Geom.Cam_Geo := Geom.No_Geo;
      Rot : constant Geom.V3 := [0.0, 0.16, 0.0];
      Gr : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Kinem.Gx * Kinem.Gy));
      Ps : Zone.Probe_Vectors.Vector;
      Mu, Mv : Bytes.Floats;
      Added : Natural;
      function True_Finger (X, Y : Long_Float) return Boolean is
        ((X >= 20.0 and then X < 100.0 and then Y >= 240.0) or else (X >= 530.0 and then X < 620.0 and then Y >= 300.0));
      Z0 : Zone.Hand_Zone;
      Ul0, Vl0, Ur0, Vr0, Ul1, Vl1, Ur1, Vr1, Wdt, Th : Long_Float := 0.0;
      Okl0, Okr0, Okl1, Okr1 : Boolean;
      Zone_Added : Natural := 0;
   begin
      Eye.F := 400.0; Eye.Cx := 320.0; Eye.Cy := 240.0; Eye.Valid := True;
      Z.Valid := True; Z.N_Lobes := 2;
      Z.A := (Valid => True, X0 => 540, Y0 => 300, X1 => 619, Y1 => 479, Cu => 0.9, Cv => 0.8, Count => 14400);
      Z.B := (Valid => True, X0 => 20, Y0 => 300, X1 => 99, Y1 => 479, Cu => 0.1, Cv => 0.8, Count => 14400);
      Z.X0 := 250; Z.Y0 := 350; Z.X1 := 400; Z.Y1 := 479;
      Z.Fingers := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Wd * Ht));
      for Y in 0 .. Ht - 1 loop
         for X in 0 .. Wd - 1 loop
            if (Y >= 300 and then (X in 540 .. 619 or else X in 20 .. 99)) or else (X in 250 .. 400 and then Y >= 350) then
               Z.Fingers.Replace_Element (Y * Wd + X, True);
            end if;
         end loop;
      end loop;
      Z0 := Z;
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            Gr.Replace_Element (Gyy * Kinem.Gx + Gxx, True_Finger (Kinem.Grid_U (Gxx, Wd), Kinem.Grid_V (Gyy, Ht)));
         end loop;
      end loop;
      Ps := Zone.Refine_Probes (Z, Wd, Ht, Gr);
      declare
         Rm : constant Geom.M3 := Geom.Rodrigues (Rot);
      begin
         for P of Ps loop
            if True_Finger (P.U, P.V) then
               Mu.Append (P.U); Mv.Append (P.V);
            else
               declare
                  Ok : Boolean;
                  D : constant Geom.V3 := Geom.Cam_Dir (Eye, P.U, P.V, Ok);
                  U1, V1 : Long_Float;
                  Front : Boolean;
               begin
                  Geom.Cam_Pixel (Eye, Geom.Ap (Rm, D), U1, V1, Front);
                  Mu.Append (U1); Mv.Append (V1);
               end;
            end if;
         end loop;
      end;
      Zone.Apply_Refine (Z, Wd, Ht, Ps, Mu, Mv, Eye, Rot, 0.3, Added);
      for Y in 350 .. 479 loop
         for X in 250 .. 400 loop
            if Z.Fingers.Element (Y * Wd + X) /= Z0.Fingers.Element (Y * Wd + X) then
               Zone_Added := Zone_Added + 1;
            end if;
         end loop;
      end loop;
      Zone.Tip_Section (Z0, Zone.Lobe_Of (Z0, 1), Wd, Ht, Ul0, Vl0, Wdt, Th, Okl0);
      Zone.Tip_Section (Z0, Zone.Lobe_Of (Z0, 0), Wd, Ht, Ur0, Vr0, Wdt, Th, Okr0);
      Zone.Tip_Section (Z, Zone.Lobe_Of (Z, 1), Wd, Ht, Ul1, Vl1, Wdt, Th, Okl1);
      Zone.Tip_Section (Z, Zone.Lobe_Of (Z, 0), Wd, Ht, Ur1, Vr1, Wdt, Th, Okr1);
      Check (Okl1 and then Okr1 and then Vl1 < 270.0 and then Ur1 > 500.0 and then Zone_Added = 0 and then Added > 0,
             "瓣按长在眼上补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素、补进 " & Codec.Img (Added) & " 个 · 左瓣的尖 v " & Codec.Fmt (Vl0, 1) & " → " & Codec.Fmt (Vl1, 1)
             & "(要 < 270)· 右瓣的尖 u " & Codec.Fmt (Ur0, 1) & " → " & Codec.Fmt (Ur1, 1) & "(要 > 500)· 区框里补进 " & Codec.Img (Zone_Added) & " 个(要 0)");
      Check (Okl0 and then Vl0 > 300.0, "🦷 不补:左瓣的尖还在 v = " & Codec.Fmt (Vl0, 1) & "(变化掩码缺的那一截认不出)");
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
   --  🔴 挑空地只在量过、而且此刻还找得到的桌面上(Act.Board_Free_Spots,09-28 V1B47):板 1.2 cm 一格铺在 0.765 m 的面上,
   --  y < 0 那一半一个点都没有(没量过:开机时手自己挡着的那块,V1B47 里那儿放着电子琴);手指宽上限 2.5 cm,压的那一瓣落在 (0.3, 0.6) cm
   --  (有点那半的边上,原来"R 之内有一个躺在面上的板点"就收)⇒ 不收、往里挪:落点圈整个在有点那半里 —— 落点 y ≥ R − 半格,也不挪得太远(≤ R + 两格);
   --  y < 0 那半也铺满 ⇒ 不挪。板铺满、落点四周 3 cm 的点标成"此刻在不动的眼里找不到" ⇒ 当没量过、挪出去(落点离那片中心 ≥ 3 cm + R − 一格);
   --  标记和板对不上号(个数不等 = 板重建过)⇒ 不认、按量的那一刻 ⇒ 不挪;落点那儿一个 5 mm 高的小东西标成找不到也照样挡(挪到离它 R 以外)
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      Cell : constant Long_Float := 0.012;   --  格距(米,合成)
      Rw : constant Long_Float := 0.025;     --  手指宽上限(米,合成)
      Z0 : constant Long_Float := 0.765;     --  面高(米,合成)
      function Pt (X, Y, Z : Long_Float) return Geom.Scene_Pt is
        (Geom.Scene_Pt'(Pw => [X, Y, Z], U => 0.0, V => 0.0, Sh => 0.0, Views => 9,
                        Cov => [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]]));
      procedure Base (C : in out Act.Context) is
      begin
         C.Board_Plane := True; C.Board_Pt := [0.0, 0.0, Z0]; C.Board_N := [0.0, 0.0, 1.0]; C.Board_Rms := 0.001;
      end Base;
      Lp : Geom.V3_Vectors.Vector;
      Tb : Bytes.Floats;
      Half, Full, Hidden, Stale, Bump : Act.Context;
      Dh, Df, Dd, Ds, Db : Geom.V3_Vectors.Vector;
      function Land (D : Geom.V3_Vectors.Vector) return Geom.V3 is
        (if D.Is_Empty then [99.0, 99.0, 99.0] else [Lp (0) (0) + D (0) (0), Lp (0) (1) + D (0) (1), Lp (0) (2) + D (0) (2)]);
      function Flat (V : Geom.V3) return Long_Float is (Sqrt (V (0) ** 2 + V (1) ** 2));
   begin
      Base (Half); Base (Full);
      for I in -12 .. 12 loop
         for J in -12 .. 12 loop
            Full.Board.Append (Pt (Cell * Long_Float (I), Cell * Long_Float (J), Z0));
            if J >= 0 then
               Half.Board.Append (Pt (Cell * Long_Float (I), Cell * Long_Float (J), Z0));
            end if;
         end loop;
      end loop;
      Hidden := Full; Stale := Full; Bump := Full;
      for P of Full.Board loop
         Hidden.Board_Seen.Append (Sqrt (P.Pw (0) ** 2 + P.Pw (1) ** 2) > 0.03);
         Bump.Board_Seen.Append (Sqrt (P.Pw (0) ** 2 + P.Pw (1) ** 2) > 1.0e-9);
      end loop;
      Stale.Board_Seen := Hidden.Board_Seen;
      Stale.Board_Seen.Append (True);   --  多一个 = 和板对不上号
      Bump.Board.Replace_Element (12 * 25 + 12, Pt (0.0, 0.0, Z0 + 0.005));   --  (0, 0) 那一格(I = J = 0)
      Lp.Append (Geom.V3'[0.003, 0.006, Z0]);
      Lp.Append (Geom.V3'[0.09, 0.006, Z0]);
      Tb.Append (0.0); Tb.Append (1.0);
      Act.Board_Free_Spots (Half, Lp, Tb, Rw, Dh);
      Act.Board_Free_Spots (Full, Lp, Tb, Rw, Df);
      Act.Board_Free_Spots (Hidden, Lp, Tb, Rw, Dd);
      Act.Board_Free_Spots (Stale, Lp, Tb, Rw, Ds);
      Act.Board_Free_Spots (Bump, Lp, Tb, Rw, Db);
      declare
         Lh : constant Geom.V3 := Land (Dh);
         Ld : constant Geom.V3 := Land (Dd);
         Lb : constant Geom.V3 := Land (Db);
         H_Ok : constant Boolean := not Dh.Is_Empty and then Geom.Norm (Dh (0)) > 0.0 and then Lh (1) >= Rw - 0.5 * Cell and then Geom.Norm (Dh (0)) <= Rw + 2.0 * Cell;
         F_Ok : constant Boolean := not Df.Is_Empty and then Geom.Norm (Df (0)) = 0.0;
         D_Ok : constant Boolean := not Dd.Is_Empty and then Flat (Ld) >= 0.03 + Rw - Cell;
         S_Ok : constant Boolean := not Ds.Is_Empty and then Geom.Norm (Ds (0)) = 0.0;
         B_Ok : constant Boolean := not Db.Is_Empty and then Flat (Lb) > Rw;
      begin
         Check (H_Ok and then F_Ok and then D_Ok and then S_Ok and then B_Ok,
                "挑空地只在量过、此刻还找得到的桌面上:有点那半的边上 ⇒ 挪到 (" & Codec.Fmt (Lh (0), 3) & "," & Codec.Fmt (Lh (1), 3) & ") m(要 y ≥ "
                & Codec.Fmt (Rw - 0.5 * Cell, 3) & ")" & (if H_Ok then "" else "(错)") & " · 铺满 ⇒ 不挪" & (if F_Ok then "" else "(错)")
                & " · 四周 3 cm 找不到 ⇒ 落点离中心 " & Codec.Fmt (Flat (Ld), 3) & " m(要 ≥ " & Codec.Fmt (0.03 + Rw - Cell, 3) & ")" & (if D_Ok then "" else "(错)")
                & " · 标记对不上号 ⇒ 不认、不挪" & (if S_Ok then "" else "(错)")
                & " · 5 mm 小东西标成找不到照样挡 ⇒ 落点离它 " & Codec.Fmt (Flat (Lb), 3) & " m" & (if B_Ok then "" else "(错)"));
      end;
   end;
   --  🔴 压之前看见的高出面的点照样挡(Act.Board_Free_Spots + C.Seen_Above,09-30 V1B70 / V1B73):板 21×21 个点铺满 0.765 m 的面(2 cm 一格),
   --  压的那一瓣落在 (0,0)、另一瓣落在 (0.05,0),另一瓣视线斜 90°(离压的那一点 ρ 处手指至少高 ρ)。另一瓣连线 3 cm 处看见一个比面高 4 cm 的点
   --  (琴的一角:板上没有它 —— 开机时手自己挡着)⇒ 要挪,挪完那一点碰不着另一根手指:离两个落点的连线超过手指宽上限,或者比那儿的手指低
   --  (离压的那一点 ρ 处手指至少高 ρ);同一处看见的点只高 0.5 mm(在面的离散里)⇒ 不挪。
   --  🦷 不算看见的点:原处就收(另一瓣压在琴上)
   declare
      Cell : constant Long_Float := 0.02;   --  格距(米,合成)
      Z0 : constant Long_Float := 0.765;    --  面高(米,合成)
      Rw : constant Long_Float := 0.01;     --  手指宽上限(米,合成)
      Small_Cov : constant Geom.M3 := [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]];
      function Pt (X, Y, Z : Long_Float) return Geom.Scene_Pt is
        (Geom.Scene_Pt'(Pw => [X, Y, Z], U => 0.0, V => 0.0, Sh => 0.0, Views => 9, Cov => Small_Cov));
      Flat, Keys, Low : Act.Context;
      Lp : Geom.V3_Vectors.Vector;
      Tb : Bytes.Floats;
      Df, Dk, Dl : Geom.V3_Vectors.Vector;
      Side_Ok : Boolean := False;
   begin
      Flat.Board_Plane := True; Flat.Board_Pt := [0.0, 0.0, Z0]; Flat.Board_N := [0.0, 0.0, 1.0]; Flat.Board_Rms := 0.001;
      for I in -10 .. 10 loop
         for J in -10 .. 10 loop
            Flat.Board.Append (Pt (Cell * Long_Float (I), Cell * Long_Float (J), Z0));
         end loop;
      end loop;
      Keys := Flat; Low := Flat;
      Keys.Seen_Above.Append (Pt (0.03, 0.0, Z0 + 0.04));
      Low.Seen_Above.Append (Pt (0.03, 0.0, Z0 + 0.0005));
      Lp.Append (Geom.V3'[0.0, 0.0, Z0]);
      Lp.Append (Geom.V3'[0.05, 0.0, Z0]);
      Tb.Append (0.0); Tb.Append (1.0);
      Act.Board_Free_Spots (Flat, Lp, Tb, Rw, Df);
      Act.Board_Free_Spots (Keys, Lp, Tb, Rw, Dk);
      Act.Board_Free_Spots (Low, Lp, Tb, Rw, Dl);
      if not Dk.Is_Empty then
         declare
            use Ada.Numerics.Long_Elementary_Functions;
            Ax : constant Long_Float := Dk (0) (0); Ay : constant Long_Float := Dk (0) (1);
            Bx : constant Long_Float := 0.05 + Dk (0) (0);
            T : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (1.0, (0.03 - Ax) / (Bx - Ax)));
            Rho : constant Long_Float := T * (Bx - Ax);   --  沿连线离压的那一点多远
         begin
            Side_Ok := Sqrt ((0.03 - (Ax + T * (Bx - Ax))) ** 2 + (0.0 - Ay) ** 2) > Rw or else 0.04 < Rho;
         end;
      end if;
      Check (not Dk.Is_Empty and then Geom.Norm (Dk (0)) > 0.0 and then Side_Ok and then not Dl.Is_Empty and then Geom.Norm (Dl (0)) = 0.0,
             "压之前看见的高出面的点照样挡:另一瓣连线 3 cm 处高 4 cm 的点 ⇒ 挪 " & (if Dk.Is_Empty then "-" else Codec.Fmt (Geom.Norm (Dk (0)), 3)) & " m、"
             & "挪完那一点碰不着另一根手指" & (if Side_Ok then "" else "(错)") & " · 只高 0.5 mm ⇒ 不挪" & (if not Dl.Is_Empty and then Geom.Norm (Dl (0)) = 0.0 then "" else "(错)"));
      Check (not Df.Is_Empty and then Geom.Norm (Df (0)) = 0.0, "🦷 不算看见的点:原处就收(另一瓣压在琴上)");
   end;
   --  🔴 压之前先看底下的那一对立体像(Act.Seen_Above_Of,09-30):合成的眼焦距 400、640 × 480、朝正下(相机系 -z = 世界 -z),
   --  往下压的第一步从离面 0.25 m 走到 0.20 m。面 z = 0(离散 1 mm);面上一块 4 cm 高的盒子,顶面 x 0.02–0.08、y −0.03–0.03 m。
   --  问 Kinem 那张格点,配点噪声 ±0.2 px(均匀),往返差 ±0.2 px。另有三样:手指(左下角 u < 100、v > 380:长在眼上,配到原处)、
   --  "动着的东西"(右上角 u > 500、v < 120 的格点沿垂直于过画面中心那条线的方向多挪 8 px:和只平移的眼对不上)。
   --  要:盒子顶上的格点全判成高出面(判出来的离真高 4 cm 在 5 mm 内),面上的一个都不判,手指、动着的一个都不判。
   --  🦷 两帧对不对得上不核:动着的那些按高度判,有的就成了高出面;🦷 门只按面的离散、不按每一点自己沿法向的不确定度:画面中心附近
   --  (眼平移的方向,视差小)面上的点有的判成高出面
   declare
      use Ada.Numerics.Long_Elementary_Functions;
      G : Geom.Cam_Geo := Geom.No_Geo;
      Cx : Act.Context;
      P0 : constant Plug.Arm_Pose := [0.0, 0.0, 0.25, 1.0, 0.0, 0.0, 0.0];
      P1 : constant Plug.Arm_Pose := [0.0, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0];
      Box_H : constant Long_Float := 0.04;
      Qu, Qv, Mu, Mv, Bu, Bv : Bytes.Floats;
      Is_Box, Is_F, Is_Mv : Bools;
      type Lcg is mod 2 ** 31;
      Seed : Lcg := 777;
      function Rnd return Long_Float is   --  0..1 的伪随机(线性同余,固定种子)
      begin
         Seed := Seed * 1103515245 + 12345;
         return Long_Float (Seed) / Long_Float (Lcg'Modulus);
      end Rnd;
      Above : Geom.Scene_Pt_Vectors.Vector;
      Matched, Tri : Natural;
      Sig : Long_Float;
      N_Box, Box_Hit, Wrong, Mv_Tooth, Ep_Tooth : Natural := 0;
      Box_Err : Long_Float := 0.0;
   begin
      G.Valid := True; G.F := 400.0; G.Cx := 320.0; G.Cy := 240.0;
      Cx.Board_Plane := True; Cx.Board_Pt := [0.0, 0.0, 0.0]; Cx.Board_N := [0.0, 0.0, 1.0]; Cx.Board_Rms := 0.001;
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            declare
               U : constant Long_Float := Kinem.Grid_U (Gxx, 640);
               V : constant Long_Float := Kinem.Grid_V (Gyy, 480);
               Dx : constant Long_Float := (U - 320.0) / 400.0;
               Dy : constant Long_Float := -(V - 240.0) / 400.0;
               --  先看落不落在盒子顶上(视线从 0.25 m 往下到 z = 盒高),不在就落在面上
               Xb : constant Long_Float := Dx * (0.25 - Box_H);
               Yb : constant Long_Float := Dy * (0.25 - Box_H);
               On_Box : constant Boolean := Xb >= 0.02 and then Xb <= 0.08 and then Yb >= -0.03 and then Yb <= 0.03;
               Zp : constant Long_Float := (if On_Box then Box_H else 0.0);
               Xp : constant Long_Float := Dx * (0.25 - Zp);
               Yp : constant Long_Float := Dy * (0.25 - Zp);
               Fing : constant Boolean := U < 100.0 and then V > 380.0;
               Movr : constant Boolean := U > 500.0 and then V < 120.0;
               U1 : Long_Float := 320.0 + 400.0 * Xp / (0.20 - Zp);
               V1 : Long_Float := 240.0 - 400.0 * Yp / (0.20 - Zp);
            begin
               if Fing then
                  U1 := U; V1 := V;
               elsif Movr then
                  declare
                     Rr : constant Long_Float := Sqrt ((U - 320.0) ** 2 + (V - 240.0) ** 2);
                  begin
                     U1 := U1 + 8.0 * (-(V - 240.0)) / Rr; V1 := V1 + 8.0 * (U - 320.0) / Rr;
                  end;
               end if;
               Qu.Append (U); Qv.Append (V);
               Mu.Append (U1 + 0.4 * (Rnd - 0.5)); Mv.Append (V1 + 0.4 * (Rnd - 0.5));
               Bu.Append (U + 0.4 * (Rnd - 0.5)); Bv.Append (V + 0.4 * (Rnd - 0.5));
               Is_Box.Append (On_Box and then not Fing and then not Movr);
               Is_F.Append (Fing); Is_Mv.Append (Movr);
               if On_Box and then not Fing and then not Movr then
                  N_Box := N_Box + 1;
               end if;
            end;
         end loop;
      end loop;
      Act.Seen_Above_Of (Cx, G, P0, P1, 640, 480, Qu, Qv, Mu, Mv, Bu, Bv, Above, Matched, Tri, Sig);
      --  每个判出来的点归回它是哪一个格点:按它投回第一帧落在哪个格点上(最近的那个)
      for A of Above loop
         declare
            Pu0, Pv0 : Long_Float;
            Fr : Boolean;
            Best : Natural := 0;
            Bd : Long_Float := Long_Float'Last;
         begin
            Geom.Project (G, P0, A.Pw, Pu0, Pv0, Fr);
            for I in 0 .. Natural (Qu.Length) - 1 loop
               if (Qu (I) - Pu0) ** 2 + (Qv (I) - Pv0) ** 2 < Bd then
                  Bd := (Qu (I) - Pu0) ** 2 + (Qv (I) - Pv0) ** 2; Best := I;
               end if;
            end loop;
            if Is_Box (Best) then
               Box_Hit := Box_Hit + 1;
               Box_Err := Long_Float'Max (Box_Err, abs (A.Pw (2) - Box_H));
            else
               Wrong := Wrong + 1;
            end if;
         end;
      end loop;
      --  🦷 两种旧判法各自在同一批配点上会判出几个假的
      for I in 0 .. Natural (Qu.Length) - 1 loop
         if not Is_Box (I) and then not Is_F (I) then
            declare
               Rays : Geom.Sight_Vectors.Vector;
               Okm : Boolean;
               Spread : Long_Float;
               X : Geom.V3;
            begin
               Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P0), D => Geom.Ray (G, P0, Qu (I), Qv (I))));
               Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P1), D => Geom.Ray (G, P1, Mu (I), Mv (I))));
               X := Geom.Meet (Rays, Okm, Spread);
               if Okm and then X (2) > 3.0 * Cx.Board_Rms then
                  if Is_Mv (I) then
                     Mv_Tooth := Mv_Tooth + 1;
                  else
                     Ep_Tooth := Ep_Tooth + 1;
                  end if;
               end if;
            end;
         end if;
      end loop;
      Check (N_Box > 0 and then Box_Hit = N_Box and then Wrong = 0 and then Box_Err < 0.005,
             "压之前看底下:问 " & Codec.Img (Natural (Qu.Length)) & " 个、配上 " & Codec.Img (Matched) & "、交成且两帧对得上 " & Codec.Img (Tri)
             & "(配点噪声 " & Codec.Fmt (Sig, 3) & " px)· 盒子顶上 " & Codec.Img (Box_Hit) & " / " & Codec.Img (N_Box) & " 判成高出面(离真高最多差 "
             & Codec.Fmt (Box_Err * 1000.0, 1) & " mm)· 面上、手指、动着的判成高出面的 " & Codec.Img (Wrong) & " 个(要 0)");
      Check (Mv_Tooth > 0, "🦷 两帧对不对得上不核:动着的格点有 " & Codec.Img (Mv_Tooth) & " 个按高度判成高出面");
      Check (Ep_Tooth > 0, "🦷 门只按面的离散:面上的格点有 " & Codec.Img (Ep_Tooth) & " 个判成高出面(眼平移方向附近视差小)");
   end;
   --  🔴 几只手按拍对齐(Lockstep + Plug.Lock_*,09-28 PLAN ⑧ (g)):两只假手,第 1 只走 3 条(第 0 组关节目标 1、2、3)、第 2 只走 5 条(第 1 组 11–15),
   --  每一条走 Selfmap.Go(发命令的只有这一处;假帧里没有读数 ⇒ 等满两拍就算停)⇒ 一共 10 拍(不是 6 + 10 = 16 拍:两只手同时走);
   --  每一拍合成的那条命令里两组都在:第 0 组是第 ⌈拍/2⌉ 条的目标(走完了停在最后的 3)、第 1 组是 10 + ⌈拍/2⌉;手的任务里认得出自己是第几只手,主线程认出自己不是
   declare
      Lk : Plug.Link;
      Rounds : Natural := 0;
      Merge_Ok : Boolean := True;
      Main_Is : constant Integer := Lockstep.Current_Hand;
      Seen_Me : array (0 .. 1) of Integer := [others => -9];
      task type Fake_Hand (H, Steps, Base : Natural);
      task body Fake_Hand is
         Fr : Plug.Frame;
         Mp : Selfmap.Body_Map;
         Dl : Table.Vec;
         Nf : Natural;
         Ok : Boolean;
      begin
         Lockstep.Begin_Hand (H);
         Seen_Me (H) := Lockstep.Current_Hand;
         for I in 1 .. Steps loop
            declare
               Q : Bytes.Floats;
            begin
               Q.Append (Long_Float (Base + I));
               Selfmap.Go (Lk, Mp, H, [others => 0.0], Bytes.F64_Vectors.Empty_Vector, Fr, Dl, Nf, Ok, Joints => Q, Group => H);
            end;
         end loop;
         Lockstep.Done;
      end Fake_Hand;
   begin
      Lockstep.Clear;
      Plug.Lock_Begin;
      declare
         H0 : Fake_Hand (0, 3, 0);
         H1 : Fake_Hand (1, 5, 10);
      begin
         Lockstep.Start (0, H0'Identity);
         Lockstep.Start (1, H1'Identity);
         loop
            Lockstep.Run (0);
            Lockstep.Run (1);
            exit when Lockstep.Finished (0) and then Lockstep.Finished (1);
            Rounds := Rounds + 1;
            declare
               M : constant Plug.Cmd := Plug.Lock_Merged;
            begin
               if Natural (M.Groups.Length) /= 2 or else M.Qs (0) (0) /= Long_Float (Natural'Min ((Rounds + 1) / 2, 3)) or else M.Qs (1) (0) /= Long_Float (10 + (Rounds + 1) / 2) then
                  Merge_Ok := False;
               end if;
            end;
         end loop;
      end;
      Plug.Lock_End;
      Lockstep.Clear;
      Check (Rounds = 10 and then Merge_Ok and then Seen_Me (0) = 0 and then Seen_Me (1) = 1 and then Main_Is = -1,
             "几只手按拍对齐:3 条和 5 条命令(一条两拍)两只手一共走了 " & Codec.Img (Rounds) & " 拍(要 10,一只一只走是 16)· 每拍合成的命令两组目标"
             & (if Merge_Ok then "都对" else "不对") & " · 手的任务认得出自己(" & Integer'Image (Seen_Me (0)) & "," & Integer'Image (Seen_Me (1)) & "),主线程"
             & (if Main_Is = -1 then "不是手" else "被认成了手"));
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
      --  零向量 = 那个像素去不了畸变、没有视线(09-30,Geom.Cam_Dir):不算一条。两条真视线交在 (0, 0.75, 0),再加一条从 (1,1,1) 出发的零向量 ⇒ 交点不动;
      --  牙:按旧写法把它也当一条(I − 0·0ᵀ = I,等于把交点往它的起点拽)⇒ 交点被拽开
      Rs.Clear;
      Rs.Append (Geom.Sight'(O => [0.0, 0.0, 1.0], D => [0.0, 0.6, -0.8]));
      Rs.Append (Geom.Sight'(O => [1.0, 0.0, 1.0], D => [-0.6246950475544243, 0.4685212856658182, -0.6246950475544243]));   --  (1,0,1) → (0,0.75,0) 归一
      declare
         P2 : Geom.V3;
         Ok2 : Boolean;
         Sp2 : Long_Float;
         A : Geom.M3 := [others => [others => 0.0]];
         B : Geom.V3 := [others => 0.0];
         Old_P : Geom.V3;
      begin
         P := Geom.Meet (Rs, Ok, Sp);
         Rs.Append (Geom.Sight'(O => [1.0, 1.0, 1.0], D => [0.0, 0.0, 0.0]));
         P2 := Geom.Meet (Rs, Ok2, Sp2);
         for R of Rs loop   --  旧写法:每一条都进最小二乘,零向量也算
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  A (I, J) := A (I, J) + (if I = J then 1.0 else 0.0) - R.D (I) * R.D (J);
                  B (I) := B (I) + ((if I = J then 1.0 else 0.0) - R.D (I) * R.D (J)) * R.O (J);
               end loop;
            end loop;
         end loop;
         Old_P := Geom.Solve3 (A, B);
         Check (Ok and then Ok2 and then Geom.Norm ([P2 (0) - P (0), P2 (1) - P (1), P2 (2) - P (2)]) < 1.0e-12 and then abs (P (1) - 0.75) < 1.0e-9
                and then Geom.Norm ([Old_P (0) - P (0), Old_P (1) - P (1), Old_P (2) - P (2)]) > 0.1,
                "两眼交点:多一条零向量(去不了畸变的像素)⇒ 交点不动 (" & Codec.Fmt (P2 (0), 3) & "," & Codec.Fmt (P2 (1), 3) & "," & Codec.Fmt (P2 (2), 3)
                & ") · 牙:当成一条视线 ⇒ 被拽到 (" & Codec.Fmt (Old_P (0), 3) & "," & Codec.Fmt (Old_P (1), 3) & "," & Codec.Fmt (Old_P (2), 3) & ")");
      end;
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
   --  ── 认身体:身体给的 3×3 内参认成内参(09-28 硬件组 PR #1)。3×3 浮点也满足"浮点 + 二维",原来 Is_Depth 在前、把它收成一张
   --  尺寸对不上的深度图再丢掉;驱动本来就不读身体给的内参(焦距自己量),可认错了就是认错了 ⇒ 先认内参再认深度 ──
   declare
      S : Buf;
      D : Msgpack.Doc;
      Lk : Layout.Body_Layout;
      Img, Kb : Buf;
      procedure F4 (B : in out Buf; B0, B1, B2, B3 : Interfaces.Unsigned_8) is
      begin
         B.Append (B0); B.Append (B1); B.Append (B2); B.Append (B3);
      end F4;
   begin
      for I in 1 .. 12 loop   --  2×2×3 的图
         Img.Append (100);
      end loop;
      --  [300 0 1; 0 300 1; 0 0 1](float32 小端:300 = 43960000,1 = 3F800000)
      F4 (Kb, 0, 0, 16#96#, 16#43#); F4 (Kb, 0, 0, 0, 0); F4 (Kb, 0, 0, 16#80#, 16#3F#);
      F4 (Kb, 0, 0, 0, 0); F4 (Kb, 0, 0, 16#96#, 16#43#); F4 (Kb, 0, 0, 16#80#, 16#3F#);
      F4 (Kb, 0, 0, 0, 0); F4 (Kb, 0, 0, 0, 0); F4 (Kb, 0, 0, 16#80#, 16#3F#);
      Msgpack.Put_Map (S, 1);
      Msgpack.Put_Str (S, "obs"); Msgpack.Put_Map (S, 2);
      Msgpack.Put_Str (S, "cam"); Msgpack.Put_Map (S, 2);
      Msgpack.Put_Str (S, "color"); Msgpack.Put_Map (S, 4);
      Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
      Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "|u1");
      Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 3);
      Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Img, 0, 12);
      Msgpack.Put_Str (S, "intrinsic"); Msgpack.Put_Map (S, 4);
      Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
      Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "<f4");
      Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 2); Msgpack.Put_Int (S, 3); Msgpack.Put_Int (S, 3);
      Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Kb, 0, 36);
      Msgpack.Put_Str (S, "elbow"); Msgpack.Put_Array (S, 6);
      for I in 1 .. 6 loop
         Msgpack.Put_Float (S, 0.1);
      end loop;
      Check (Msgpack.Decode (S, D), "认身体:带 3×3 内参的那一帧解得开");
      Layout.Recognise (D, Msgpack.Key (D, 0, "obs"), Lk);
      Check (Natural (Lk.Cams.Length) = 1 and then Natural (Lk.Intr.Length) = 1 and then not Lk.Intr (0).Segs.Is_Empty and then Lk.Depth.Is_Empty,
             "认身体:身体给的 3×3 内参配到它那台相机上(" & (if Natural (Lk.Intr.Length) = 1 and then not Lk.Intr (0).Segs.Is_Empty then "配上了" else "没配上")
             & "),没被当成深度图(驱动照样不读它,焦距自己量)");
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
   end;


   --  🔴 LM 阻尼一直往上调到步子挪不动 X 为止(09-30,Kinem.Robust_LM;原来一轮最多调 8 次阻尼、都没降就判"到底了"整个退出):
   --  r(x) = x⁴ − 1 从 x = 0.01 起 —— 那里斜率只有 4e-6,不加阻尼的一步跳到 2e5,阻尼(1e-3 起、每次 ×10)要到 1e6、第 10 次才第一次降
   --  ⇒ 要解到 x = 1、交出"收住了"。牙:同一个起点按原来的 8 次(阻尼 1e-3 … 1e4)一步一步算,每一步代价都涨 ⇒ 旧写法停在 0.01 当"到底了"
   declare
      procedure Quartic (X : Kinem.Vec; R : out Kinem.Vec) is
      begin
         R (R'First) := X (X'First) ** 4 - 1.0;
      end Quartic;
      X : Kinem.Vec (0 .. 0) := [0.01];
      Done : Boolean;
      Old_Stuck : Boolean := True;   --  原来的 8 次都没降
   begin
      declare
         X0 : constant Long_Float := 0.01;
         H : constant Long_Float := 1.0e-6;   --  同下面给 Robust_LM 的差分步
         R0 : constant Long_Float := X0 ** 4 - 1.0;
         J : constant Long_Float := ((X0 + H) ** 4 - 1.0 - R0) / H;
         Lam : Long_Float := 1.0e-3;          --  同 Robust_LM 起步的阻尼
         Up : constant Long_Float := 10.0;    --  同 Robust_LM 的阻尼放大倍数
      begin
         for Try in 1 .. 8 loop
            declare
               D : constant Long_Float := -J * R0 / (J * J * (1.0 + Lam) + 1.0e-12);   --  同 Robust_LM 的一维:A = J²、B = −J r、对角加 1e-12
            begin
               if ((X0 + D) ** 4 - 1.0) ** 2 < R0 ** 2 then
                  Old_Stuck := False;
               end if;
            end;
            Lam := Lam * Up;
         end loop;
      end;
      Kinem.Robust_LM (X, 1, 0, 100, [1.0e-6], Quartic'Access, Done);
      Check (Done and then abs (X (0) - 1.0) < 1.0e-6 and then Old_Stuck,
             "LM 阻尼调到步子挪不动为止:x⁴ − 1 从 0.01 起解到 x = " & Long_Float'Image (X (0)) & (if Done then "、收住了" else "、没收住")
             & "(要 1 ± 1e-6);原来的 8 次阻尼" & (if Old_Stuck then "每一步都让代价涨,旧写法停在 0.01" else "有一步降了(牙没咬住)"));
   end;
   --  🔴 Huber 的门按量到的噪声定(09-30,Kinem.Huber_K;Robust_LM 的调用约定:残差先除以量到的 σ):一维的位置,2000 个样本,
   --  噪声 σ = 0.02 px(配点很准的相机),一成离群、都偏在 +6σ(0.12 px:在 max(3 px, 3 倍中位)的挑内点门里面,挑不掉)。
   --  残差除以起点量到的噪声(Mad_Sigma × 起点残差的中位,同 kinem 自己的几处)、门 Huber_K ⇒ 离群的被压下去,偏差 < 0.35σ。
   --  牙:同一批样本按原来的门"1 像素"(kinem 自己喂像素残差、门 1.0)—— 0.12 px 全在门里,和最小二乘一样,偏差 ≈ 一成 × 6σ = 0.6σ(要 > 0.5σ)
   declare
      package FR renames Ada.Numerics.Float_Random;
      Gen : FR.Generator;
      N : constant := 2000;
      S : constant Long_Float := 0.02;
      Y : Kinem.Vec (0 .. N - 1);
      Sig : Long_Float := 1.0;
      X_New, X_Old : Kinem.Vec (0 .. 0);
      D_New, D_Old : Boolean;
      procedure R_New (X : Kinem.Vec; R : out Kinem.Vec) is
      begin
         for I in 0 .. N - 1 loop
            R (R'First + I) := (Y (I) - X (X'First)) / Sig;
         end loop;
      end R_New;
      --  旧的门 1 px 换成新约定:残差 × Huber_K 以后门 Huber_K 就落在 1 px(代价差一个常数倍,解一样)
      procedure R_Old (X : Kinem.Vec; R : out Kinem.Vec) is
      begin
         for I in 0 .. N - 1 loop
            R (R'First + I) := (Y (I) - X (X'First)) * Kinem.Huber_K;
         end loop;
      end R_Old;
   begin
      FR.Reset (Gen, 20260930);
      for I in 0 .. N - 1 loop
         declare
            A : constant Long_Float := Long_Float'Max (1.0e-12, Long_Float (FR.Random (Gen)));
            B : constant Long_Float := Long_Float (FR.Random (Gen));
         begin
            Y (I) := (if I mod 10 = 0 then 6.0 * S else S * Ada.Numerics.Long_Elementary_Functions.Sqrt (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (A))
                                                          * Ada.Numerics.Long_Elementary_Functions.Cos (2.0 * Ada.Numerics.Pi * B));
         end;
      end loop;
      declare
         M0 : Long_Float := 0.0;
         Ab : Floats;
         package Sorting is new F64_Vectors.Generic_Sorting;
      begin
         for V of Y loop
            M0 := M0 + V;
         end loop;
         M0 := M0 / Long_Float (N);   --  起点 = 平均(被离群的拉偏)
         for V of Y loop
            Ab.Append (abs (V - M0));
         end loop;
         Sorting.Sort (Ab);
         Sig := Kinem.Mad_Sigma * Ab (N / 2);   --  起点量到的噪声
         X_New := [M0]; X_Old := [M0];
      end;
      Kinem.Robust_LM (X_New, N, N, 100, [1.0e-6], R_New'Access, D_New);
      Kinem.Robust_LM (X_Old, N, N, 100, [1.0e-6], R_Old'Access, D_Old);
      Check (D_New and then abs X_New (0) < 0.35 * S and then abs X_Old (0) > 0.5 * S,
             "Huber 的门按量到的噪声定:σ = 0.02 px、一成离群在 +6σ ⇒ 按量到的噪声(" & Codec.Fmt (Sig, 4) & " px)解出的偏差 "
             & Codec.Fmt (X_New (0) / S, 2) & "σ(要 < 0.35σ);牙:门 1 像素的偏差 " & Codec.Fmt (X_Old (0) / S, 2) & "σ(和最小二乘一样,要 > 0.5σ)");
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
         --  考试(09-28 从 ④ 挪上来,长在眼上那条也用):全关节 ±30° 随机 30 个姿势,按训练帧定倍数,中位 / 最大(mm)
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
         Put_Line ("    运动学·做到不再变:③ 挑内点 " & Codec.Img (Rep.Rounds) & " 轮、量到的噪声 " & Codec.Fmt (Rep.Sig_Px, 3) & " px · ④ " & Codec.Img (Rep.Mv_Passes)
                   & " 遍(留下那遍 " & Codec.Img (Rep.Mv_Rounds) & " 轮)、量到的噪声 " & Codec.Fmt (Rep.Mv_Sig_Px, 3) & " px(配点加的 0.3)· 碰到保险上限:"
                   & (if Length (Rep.Unsettled) = 0 then "没有" else To_String (Rep.Unsettled)));
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
         --  🔴 配点噪声从重投影残差怎么量(09-30,Kinem.Track_Points 的 Sig_Px:jointboot 拿它当三角点的协方差进对齐白化):真模型、这一批配点
         --  (每一笔每个方向 0.3 px 高斯、5% 乱配没挑)、起点那帧出发的轨迹(同驱动)⇒ Sig_Px 要 0.27–0.36 px(乱配把中位抬高约 6%)。
         --  牙:原来 1.4826 × 二维残差长度的中位 —— 多视图轨迹上长度的中位 = 1.1774σ ⇒ 同一批点投回去算,偏大到 0.5 px 以上(要 > 0.45)
         declare
            use Ada.Numerics.Long_Elementary_Functions;
            Tp : Kinem.Track_Pt_Vectors.Vector;
            Sg : Long_Float;
            Lens : Floats;
            Old_Sig : Long_Float := 0.0;
            package Sorting is new F64_Vectors.Generic_Sorting;
         begin
            Kinem.Track_Points (Truth, Frames, Cs, Only_I => 0, Min_Views => 1, Tracks => Tp, Sig_Px => Sg);
            --  返回的点投回每一笔那一帧(真模型):同一个起点像素 = 同一条轨迹
            for C of Cs loop
               if C.I = 0 and then C.Pt >= 0 then
                  for T of Tp loop
                     if T.U = C.Ua and then T.V = C.Va then
                        declare
                           Rr : M3;
                           Tt : V3;
                           Pc : V3;
                        begin
                           Kinem.FK (Truth, Frames (C.J).Q, Rr, Tt);
                           Pc := Ap (Tr (Rr), [T.X (0) - Tt (0), T.X (1) - Tt (1), T.X (2) - Tt (2)]);
                           if Pc (2) < 0.0 then
                              Lens.Append (Sqrt ((Cx + F_True * Pc (0) / (-Pc (2)) - C.Ub) ** 2 + (Cy - F_True * Pc (1) / (-Pc (2)) - C.Vb) ** 2));
                           end if;
                        end;
                        exit;
                     end if;
                  end loop;
               end if;
            end loop;
            if not Lens.Is_Empty then
               Sorting.Sort (Lens);
               Old_Sig := 1.4826 * Lens (Natural (Lens.Length) / 2);
            end if;
            Check (Sg > 0.27 and then Sg < 0.36 and then Old_Sig > 0.45,
                   "配点噪声从重投影残差量:真的 0.3 px ⇒ Sig_Px " & Codec.Fmt (Sg, 3) & " px(要 0.27–0.36;垂直于对极线那一分量的 Mad_Sigma × 中位)· "
                   & Codec.Img (Natural (Tp.Length)) & " 条轨迹 " & Codec.Img (Natural (Lens.Length)) & " 笔;牙:原来 1.4826 × 二维长度的中位 = "
                   & Codec.Fmt (Old_Sig, 3) & " px(要 > 0.45:偏大)");
         end;
         --  🔴 ④ 够不够解要数上每条轨迹的远近(09-30,Kinem.Refine_Mv 开头):20 笔各自成一条轨迹(不是起点那帧出发的:一条只一笔)⇒
         --  行 = 2 × 20 + 约束 13 = 53 < 待解 37 + 20 = 57,解不了 ⇒ 模型原样不动、交回 0 条轨迹。牙:原来只比 2 × 笔数 = 40 > 37 ⇒ 当成解得了
         declare
            Cz : Kinem.Corr_Vectors.Vector;
            Mt : Kinem.Model := Truth;
            Rz : Kinem.Fit_Report;
            Np : constant Natural := 3 * Truth.N + 3 * Truth.N + 1;   --  6 根转的轴:方向 3 + 轴上点 3,加对数焦距
            N_Reg : constant Natural := 2 * Truth.N + 1;
            Same : Boolean := True;
         begin
            for C of Cs loop
               exit when Natural (Cz.Length) >= 20;
               if C.I /= 0 and then C.Pt >= 0 then
                  Cz.Append (C);
               end if;
            end loop;
            Kinem.Refine_Tracks (Frames, Cz, Mt, Rz);
            for K in 0 .. Natural (Frames.Length) - 1 loop
               declare
                  Ra, Rb : M3;
                  Ta, Tb : V3;
               begin
                  Kinem.FK (Truth, Frames (K).Q, Ra, Ta);
                  Kinem.FK (Mt, Frames (K).Q, Rb, Tb);
                  Same := Same and then Ta = Tb and then Ra = Rb;
               end;
            end loop;
            Check (Same and then Rz.Mv_Tracks = 0 and then 2 * 20 + N_Reg <= Np + 20 and then 2 * 20 > Np,
                   "④ 够不够解数上远近:20 条一笔的轨迹 ⇒ 行" & Natural'Image (2 * 20 + N_Reg) & " 不多于待解" & Natural'Image (Np + 20)
                   & ",模型" & (if Same then "原样不动" else "被改了") & "、交回" & Natural'Image (Rz.Mv_Tracks) & " 条;牙:原来 2 × 笔数"
                   & Natural'Image (2 * 20) & " > Np" & Natural'Image (Np) & " 就解");
         end;
         --  🔴 长在眼上的像素(09-28 人形 H2 / H3):同一条胳膊、同一批配点,每一对再加 12 × 10 个钉在画面同一处的格点(像腕眼里自己的手:
         --  下半幅中间一块,占全部配点近一半;人形实测 26%;每一笔带 0.3 px 噪声,四分之一再抖 2.2 px = 软手指)⇒ 要:正好认出这 120 个像素、
         --  焦距 1% 内、考试中位 < 1 mm / 最大 < 5 mm(同上面那条)。起点那帧出发的给轨迹号(同驱动:问同一张格点)
         declare
            Cs_E : Kinem.Corr_Vectors.Vector := Cs;
            M_E : Kinem.Model;
            Rep_E : Kinem.Fit_Report;
            Ok_E : Boolean;
            Em, Ex : Long_Float := 0.0;
         begin
            for I in 0 .. Natural (Frames.Length) - 1 loop
               for J in 0 .. Natural (Frames.Length) - 1 loop
                  if Paired (I, J) then
                     for Gu in 0 .. 11 loop
                        for Gv in 0 .. 9 loop
                           declare
                              U : constant Long_Float := 210.0 + 20.0 * Long_Float (Gu);
                              V : constant Long_Float := 170.0 + 10.0 * Long_Float (Gv);
                              Jit : constant Long_Float := (if U01 < 0.25 then 2.2 else 0.0);
                              Th : constant Long_Float := 2.0 * Ada.Numerics.Pi * U01;
                           begin
                              Cs_E.Append (Kinem.Corr'(I => I, J => J, Ua => U, Va => V, Ub => U + 0.3 * Gauss + Jit * Cos (Th), Vb => V + 0.3 * Gauss + Jit * Sin (Th),
                                                       Pt => (if I = 0 then 1_000_000 + 10 * Gu + Gv else -1)));
                           end;
                        end loop;
                     end loop;
                  end if;
               end loop;
            end loop;
            Kinem.Fit (Frames, 0, Cs_E, Cx, Cy, 640.0, M_E, Rep_E, Ok_E);
            if Ok_E then
               Exam (M_E, Em, Ex);
            end if;
            Put_Line ("    运动学·画面里钉着自己的手:配点 " & Codec.Img (Rep_E.N_Corr) & "(钉住的 " & Codec.Img (Natural (Cs_E.Length) - Natural (Cs.Length))
                      & ")· 认出长在眼上的像素 " & Codec.Img (Rep_E.Eye_Px) & " 个、去掉 " & Codec.Img (Rep_E.Eye_Corrs) & " 笔 · 焦距 " & Codec.Fmt (Rep_E.F, 1)
                      & " · 定比例三对起步 " & Codec.Fmt (Rep_E.Rho_Start_Px, 3) & " → " & Codec.Fmt (Rep_E.Rho_Px, 3) & " px · 考试中位 " & Codec.Fmt (Em, 2)
                      & " / 最大 " & Codec.Fmt (Ex, 2) & " mm");
            Check (Ok_E and then Rep_E.Eye_Px = 120 and then Rep_E.Eye_Corrs = Natural (Cs_E.Length) - Natural (Cs.Length)
                   and then abs (Rep_E.F - F_True) < 0.01 * F_True and then Em < 1.0 and then Ex < 5.0,
                   "运动学·腕眼画面近一半配点钉住不动(自己的手):正好认出 120 个长在眼上的像素(认出 " & Codec.Img (Rep_E.Eye_Px) & ")、焦距 " & Codec.Fmt (Rep_E.F, 1)
                   & "(要 1% 内)、考试中位 " & Codec.Fmt (Em, 2) & " / 最大 " & Codec.Fmt (Ex, 2) & " mm(要 < 1 / < 5)");
         end;
      end;
   end;

   --  🔴 长在眼上的像素 · 判法本身(09-28 人形 H2,Kinem.Eye_Pixels / Off_Eye / Single_Joint):手做的一段扫描 ——
   --  参照帧 0;关节 0、1 各单独转两格(0.1 / 0.3);关节 2 单独转两格但它在相机下游(眼没动:世界不挪);帧 7 两个关节一起动(关节 1 转得多);
   --  帧 8 扫关节 1 时关节 0 被顶偏 0.01。12 个世界格点按转角挪;要认出的:A(每一对都不挪)、H(软手指:关节 0、1 各只有一格不挪,另一格抖 2.2 px ——
   --  H3 实测人形手指这样,"每一格都不挪"才算就漏掉它)。不许认的,每个只留一种诱惑(少哪条规则哪个就被认进来):
   --  P 在关节 0 的转轴方向附近(转 0.1 不挪、转 0.3 挪 3.5 px,关节 1 照挪)、C 只有关节 0 每一格都不挪(要两个关节);
   --  D 除了关节 0 只在一起动的帧 7(0 → 7、交叉对 1 → 7)里不挪、E 只在相机下游的关节 2 里不挪(那几对多数没挪 = 眼没动,不算数)、
   --  F 只在被顶偏的帧 8 里不挪 —— D、E、F 在关节 1 单独转的格子里没配上。去掉:从 A、H 出发的配点不管哪一对都去掉(交叉对里那几笔也去掉)
   declare
      function Q3 (A, B, C : Long_Float) return Floats is
         Q : Floats;
      begin
         Q.Append (A); Q.Append (B); Q.Append (C);
         return Q;
      end Q3;
      Frames : Kinem.Frame_Vectors.Vector;
      Cs : Kinem.Corr_Vectors.Vector;
      Eye : Kinem.Px_Vectors.Vector;
      Kept : Kinem.Corr_Vectors.Vector;
      N_A, N_H : Natural := 0;
      type Kind is (World, A, H, P, C, D, E, F);
      --  这个像素在参照帧 → 第 Fr 帧里挪多少(像素,朝 +u)
      function Flow (Kd : Kind; Fr : Natural) return Long_Float is
         Q : constant Floats := Frames (Fr).Q;
      begin
         case Kd is
            when World => return 200.0 * (abs Q (0) + abs Q (1));   --  相机下游的关节 2 不挪
            when A => return 0.2;
            when H => return (if Fr in 2 .. 3 then 2.2 else 0.3);
            when P => return (if abs Q (1) > 0.0 then 10.0 elsif abs Q (0) > 0.2 then 3.5 else 0.4);
            when C => return (if abs Q (1) > 0.0 then 10.0 else 0.3);
            when D => return (if Fr = 7 then 0.1 else 0.3);
            when E => return 0.3;
            when F => return (if Fr in 1 .. 2 | 8 then 0.3 else 9.0);
         end case;
      end Flow;
      procedure Add (Kd : Kind; U, V : Long_Float; I, J : Natural) is
      begin
         if Kd in D .. F and then (I in 3 .. 4 or else J in 3 .. 4) then
            return;   --  关节 1 单独转的格子里没配上
         end if;
         Cs.Append (Kinem.Corr'(I => I, J => J, Ua => U, Va => V, Ub => U + Flow (Kd, J) - (if I = 0 then 0.0 else Flow (Kd, I)), Vb => V, Pt => -1));
      end Add;
      procedure Add_All (I, J : Natural) is
      begin
         for K in 0 .. 11 loop
            Add (World, 100.0 + 20.0 * Long_Float (K), 100.0, I, J);
         end loop;
         Add (A, 400.0, 300.0, I, J); Add (H, 420.0, 320.0, I, J); Add (P, 300.0, 50.0, I, J); Add (C, 250.0, 200.0, I, J);
         Add (D, 260.0, 220.0, I, J); Add (E, 270.0, 240.0, I, J); Add (F, 280.0, 260.0, I, J);
      end Add_All;
      Dmax : constant Long_Float := Kinem.Clean_Tol (640.0);
      Only_Ah : Boolean;
   begin
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.0, 0.0, 0.0), Joint => -1));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.1, 0.0, 0.0), Joint => 0));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.3, 0.0, 0.0), Joint => 0));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.0, 0.1, 0.0), Joint => 1));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.0, 0.3, 0.0), Joint => 1));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.0, 0.0, 0.1), Joint => 2));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.0, 0.0, 0.3), Joint => 2));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.1, 0.2, 0.0), Joint => -1));
      Frames.Append (Kinem.Frame_Info'(Q => Q3 (0.01, 0.2, 0.0), Joint => 1));
      for J in 1 .. 8 loop
         Add_All (0, J);
      end loop;
      Add_All (1, 7);
      Eye := Kinem.Eye_Pixels (Frames, 0, Cs, 640.0);
      Kept := Kinem.Off_Eye (Eye, Cs);
      for Cc of Cs loop
         if Cc.Ua = 400.0 and then Cc.Va = 300.0 then
            N_A := N_A + 1;
         elsif Cc.Ua = 420.0 and then Cc.Va = 320.0 then
            N_H := N_H + 1;
         end if;
      end loop;
      --  Eye_Pixels 按像素排好序交出来:(400, 300) 在 (420, 320) 前面
      Only_Ah := Natural (Eye.Length) = 2 and then Eye (0).U = 400.0 and then Eye (0).V = 300.0 and then Eye (1).U = 420.0 and then Eye (1).V = 320.0;
      Check (Only_Ah and then Natural (Kept.Length) = Natural (Cs.Length) - N_A - N_H and then N_A = 9 and then N_H = 9
             and then Kinem.Single_Joint (Frames, 0, 5, Dmax) = 2 and then Kinem.Single_Joint (Frames, 0, 7, Dmax) = -1
             and then Kinem.Single_Joint (Frames, 0, 8, Dmax) = -1 and then Kinem.Single_Joint (Frames, 0, 0, Dmax) = -1,
             "长在眼上的像素:只认出每一对都不挪的 A 和软手指 H(认出 " & Codec.Img (Natural (Eye.Length)) & " 个" & (if Only_Ah then "、就是 A、H" else "") & ");"
             & "一根转轴方向附近的、只一个关节不挪的、交叉对 / 一起动 / 被顶偏的帧里不挪的、相机下游关节里不挪的都不认;从 A、H 出发的 " & Codec.Img (N_A + N_H)
             & " 笔(连交叉对那几笔)都去掉(剩 " & Codec.Img (Natural (Kept.Length)) & " / " & Codec.Img (Natural (Cs.Length)) & ");只动了一个关节的帧认得对");
   end;

   --  🔴 一组关节读数多于 12 个也不截(09-30:Kinem.Model.Ax 跟着读数个数走;原来按 12 根开死,Fit_World / Single_Joint / Eye_Pixels 都取
   --  min(12, 读数个数),多出来的关节照样带着眼动,运动学错而且不报)。一组 14 个读数:
   --  ① 扫第 3 个关节那一格,第 13 个被顶偏 0.01(比 Clean_Tol 大)⇒ 不算"只动了一个关节"(-1);牙:原来只查前 12 个 ⇒ 认成只动了第 3 个;
   --  ② 格点 E 在第 12、13 个关节各自单独转的格子里都不挪(别的格点都挪)⇒ 认成长在眼上的像素;牙:原来第 12、13 个关节的格子根本不算单独转的格子;
   --  ③ 14 根轴的模型:只转第 13 根,眼跟着动(FK),反解从零位解回那个位姿(IK,要 < 1e-6)
   declare
      Frames : Kinem.Frame_Vectors.Vector;
      Cs : Kinem.Corr_Vectors.Vector;
      Q0 : Floats;
      Dmax : constant Long_Float := Kinem.Clean_Tol (640.0);
      Old_N : constant Natural := Natural'Min (12, 14);   --  原来最多查几个关节
      Old_Clean : Boolean := True;
      Eye : Kinem.Px_Vectors.Vector;
      M : Kinem.Model;
      function Q_With (J : Natural; V : Long_Float) return Floats is
         Q : Floats := Q0;
      begin
         Q.Replace_Element (J, V);
         return Q;
      end Q_With;
      Pe, Re : Long_Float := 1.0;
      Moves : Long_Float := 0.0;
   begin
      for J in 0 .. 13 loop
         Q0.Append (0.0);
      end loop;
      declare
         Q1 : Floats := Q_With (3, 0.1);
      begin
         Q1.Replace_Element (13, 0.01);
         Frames.Append (Kinem.Frame_Info'(Q => Q0, Joint => -1));
         Frames.Append (Kinem.Frame_Info'(Q => Q1, Joint => 3));
         for K in 0 .. Old_N - 1 loop
            if K /= 3 and then abs (Q1 (K) - Q0 (K)) >= Dmax then
               Old_Clean := False;
            end if;
         end loop;
      end;
      --  ②:帧 2、3 只转第 12 / 13 个关节,帧 4 只转第 0 个;世界的 10 个格点每一格都挪,E(300, 200)只在帧 2、3 不挪
      Frames.Append (Kinem.Frame_Info'(Q => Q_With (12, 0.2), Joint => 12));
      Frames.Append (Kinem.Frame_Info'(Q => Q_With (13, 0.2), Joint => 13));
      Frames.Append (Kinem.Frame_Info'(Q => Q_With (0, 0.2), Joint => 0));
      for Fr in 2 .. 4 loop
         for K in 0 .. 9 loop
            Cs.Append (Kinem.Corr'(I => 0, J => Fr, Ua => 100.0 + 20.0 * Long_Float (K), Va => 100.0, Ub => 140.0 + 20.0 * Long_Float (K), Vb => 100.0, Pt => -1));
         end loop;
         Cs.Append (Kinem.Corr'(I => 0, J => Fr, Ua => 300.0, Va => 200.0, Ub => (if Fr = 4 then 340.0 else 300.2), Vb => 200.0, Pt => -1));
      end loop;
      Eye := Kinem.Eye_Pixels (Frames, 0, Cs, 640.0);
      --  ③
      M.N := 14; M.Q0 := Q0; M.Valid := True; M.F := 400.0; M.Cx := 320.0; M.Cy := 240.0;
      for J in 0 .. 13 loop
         M.Ax (J).W := (if J mod 2 = 0 then [0.0, 0.0, 1.0] else [0.0, 1.0, 0.0]);
         M.Ax (J).P := [0.05 * Long_Float (J + 1), 0.0, 0.0];
      end loop;
      declare
         Qt, Qs : Floats;
         Rt, R0 : Geom.M3;
         Tt, T0 : Geom.V3;
         Empty : Floats;
      begin
         Qt := Q_With (13, 0.5);
         Kinem.FK (M, Q0, R0, T0);
         Kinem.FK (M, Qt, Rt, Tt);
         Moves := Geom.Norm ([Tt (0) - T0 (0), Tt (1) - T0 (1), Tt (2) - T0 (2)]);
         Kinem.IK (M, Rt, Tt, Q0, Empty, Empty, Qs, Pe, Re);
      end;
      Check (Kinem.Single_Joint (Frames, 0, 1, Dmax) = -1 and then Old_Clean
             and then Natural (Eye.Length) = 1 and then Eye (0).U = 300.0 and then Eye (0).V = 200.0 and then Old_N < 13
             and then Natural (M.Ax.V.Length) = 14 and then Moves > 0.1 and then Pe < 1.0e-6 and then Re < 1.0e-6,
             "14 个关节的读数不截:第 13 个被顶偏 ⇒ 不算只动了一个关节(原来只查前" & Natural'Image (Old_N) & " 个 ⇒ "
             & (if Old_Clean then "认成只动了第 3 个" else "也认出来了(牙没咬住)") & ");只在第 12、13 个关节的格子里不挪的格点认成长在眼上("
             & Codec.Img (Natural (Eye.Length)) & " 个;原来这两个关节的格子不算);14 根轴:只转第 13 根眼挪 " & Codec.Fmt (Moves, 3)
             & "、反解差 " & Long_Float'Image (Pe) & " /" & Long_Float'Image (Re));
   end;

   --  🔴 运动学·沿轴走的关节(09-27 无人机那一半):合成的龙门架(像箱上的无人机:三个沿世界 x / y / z 走的关节,再绕机身中心 yaw / pitch / roll),
   --  机身中心下 3 cm 的眼朝下(偏 8°)看桌面,离桌 0.6 m,读数:走的按米、转的按弧度。扫描同驱动:每个关节单独两个方向各 3 格
   --  (走的累计 0.03 / 0.15 / 0.34、转的 0.03 / 0.15 / 0.45,同驱动"头一格 = 读数量级的 3%、之后按画面挪画幅宽 1/5 放大"的量级)+ 7 格几个关节一起动
   --  (同驱动 09-30:关节数 + 1 格、正负取 Hadamard 的行、每个关节预计画面挪一格 —— 头一格和起点挪动的中位 ÷ 转角 —— 不越过扫到过的那一头);
   --  配对同驱动:起点 ↔ 每一格(轨迹)、每段头两格、相邻关节头一格之间、一起动的相邻两格;
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
      Px_Of : array (0 .. 5, 0 .. 1) of Long_Float := [others => [others => 0.0]];   --  每个关节往负 / 往正头一格:每读数单位画面挪几像素(同驱动)
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
                     --  同驱动:头一格和起点那一对挪动的中位 ÷ 实到的转角 = 每读数单位挪几像素
                     declare
                        package Sorting is new F64_Vectors.Generic_Sorting;
                        Vh : constant Uv_Ptr := Views (Natural (Frames.Length) - 1);
                        Dv : Floats;
                     begin
                        for P in 0 .. Npt - 1 loop
                           if Views (0) (P).U >= 0.0 and then Vh (P).U >= 0.0 then
                              Dv.Append (Sqrt ((Vh (P).U - Views (0) (P).U) ** 2 + (Vh (P).V - Views (0) (P).V) ** 2));
                           end if;
                        end loop;
                        if not Dv.Is_Empty then
                           Sorting.Sort (Dv);
                           Px_Of (J, D) := Dv (Natural (Dv.Length) / 2) / Offs (J, 1);
                        end if;
                     end;
                  elsif K = 2 then
                     Add_Pair (Natural (Frames.Length) - 2, Natural (Frames.Length) - 1);
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      for Cb_K in 1 .. Jointboot.Multi_Cells (6) loop
         declare
            Q : Floats := Zeros6;
         begin
            for J in 0 .. 5 loop
               declare
                  Up : constant Boolean := Jointboot.Multi_Up (Cb_K - 1, J);   --  同驱动的排法
               begin
                  Q.Replace_Element (J, (if Up then 1.0 else -1.0) * Jointboot.Multi_Offset (640.0 / 5.0, Px_Of (J, (if Up then 1 else 0)), Offs (J, 3)));
               end;
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
         Put_Line ("    龙门架·做到不再变:③ 挑内点 " & Codec.Img (Rep.Rounds) & " 轮、量到的噪声 " & Codec.Fmt (Rep.Sig_Px, 3) & " px · ④ " & Codec.Img (Rep.Mv_Passes)
                   & " 遍(留下那遍 " & Codec.Img (Rep.Mv_Rounds) & " 轮)、量到的噪声 " & Codec.Fmt (Rep.Mv_Sig_Px, 3) & " px(配点加的 0.3)· 碰到保险上限:"
                   & (if Length (Rep.Unsettled) = 0 then "没有" else To_String (Rep.Unsettled)));
         Check (Types_Ok and then abs (Rep.F - F_True) < 0.005 * F_True and then Emax < 1.0 and then Rmax < 0.05,
                "运动学·龙门架(三走三转,像无人机):轴的类型" & (if Types_Ok then "认对" else "认错") & "(转 / 走的残差 px:" & To_String (T) & ")· 焦距 "
                & Codec.Fmt (Rep.F, 1) & "(真 400,要 0.5% 内)· 扫到的范围里 30 个随机姿势最大 " & Codec.Fmt (Emax, 3) & " mm(要 < 1)、朝向 " & Codec.Fmt (Rmax, 4) & "°(要 < 0.05)"
                & " · 认成长在眼上的像素 " & Codec.Img (Rep.Eye_Px) & " 个、去掉 " & Codec.Img (Rep.Eye_Corrs) & " / " & Codec.Img (Rep.N_Corr) & " 笔");
         --  🔴 龙门架·少一个点也解对(09-28):同一份数据各去掉一个世界点(从它出发的全部配点)再解三次 —— 去掉哪一个都要回到同一个解。
         --  ④ 以前只做一遍,在平谷里停早:这三份停在焦距 415.5、最大 17 mm(别的份走到 400;多做几遍都到 400.11、0.284 px ⇒ 不是另一个坑,是没走完);
         --  只把阻尼每轮归位,第二份照样停在 415.6 ⇒ ④ 改成"从上一遍的结果再做,直到残差中位不再降"(Kinem.Refine_Until_Done)
         declare
            Gen3 : FR.Generator;
            Worst_F, Worst_E, Worst_R : Long_Float := 0.0;
            Passes : Unbounded_String;
         begin
            FR.Reset (Gen3, 20260928);
            for K in 2 .. 4 loop
               declare
                  Drop : constant Kinem.Corr := Cs ((K * 7919) mod Natural (Cs.Length));
                  Cz : Kinem.Corr_Vectors.Vector;
                  Mz : Kinem.Model;
                  Rz : Kinem.Fit_Report;
                  Okz : Boolean;
                  Sxy2, Sxx2, Em, Rm : Long_Float := 0.0;
                  Rt2, Rf2 : M3;
                  Tt2, Tf2 : V3;
               begin
                  for C of Cs loop
                     if not (C.Ua = Drop.Ua and then C.Va = Drop.Va) then
                        Cz.Append (C);
                     end if;
                  end loop;
                  Kinem.Fit (Frames, 0, Cz, Cx, Cy, 640.0, Mz, Rz, Okz);
                  if Okz then
                     for Fk in 0 .. Natural (Frames.Length) - 1 loop
                        Kinem.FK (Truth, Frames (Fk).Q, Rt2, Tt2);
                        Kinem.FK (Mz, Frames (Fk).Q, Rf2, Tf2);
                        for X in 0 .. 2 loop
                           Sxy2 := Sxy2 + Tf2 (X) * Tt2 (X); Sxx2 := Sxx2 + Tf2 (X) * Tf2 (X);
                        end loop;
                     end loop;
                     for Tn in 1 .. 30 loop
                        declare
                           Q : Floats;
                           S2 : constant Long_Float := (if Sxx2 > 0.0 then Sxy2 / Sxx2 else 0.0);
                        begin
                           for X in 0 .. 5 loop
                              Q.Append ((2.0 * Long_Float (FR.Random (Gen3)) - 1.0) * Offs (X, 3));
                           end loop;
                           Kinem.FK (Truth, Q, Rt2, Tt2);
                           Kinem.FK (Mz, Q, Rf2, Tf2);
                           Em := Long_Float'Max (Em, 1000.0 * Sqrt ((S2 * Tf2 (0) - Tt2 (0)) ** 2 + (S2 * Tf2 (1) - Tt2 (1)) ** 2 + (S2 * Tf2 (2) - Tt2 (2)) ** 2));
                           Rm := Long_Float'Max (Rm, Norm (Rot_Vec (Mul (Tr (Rt2), Rf2))) / Deg);
                        end;
                     end loop;
                  else
                     Em := 1.0e9; Rm := 1.0e9;   --  没解出来 = 最差(哨兵)
                  end if;
                  Worst_F := Long_Float'Max (Worst_F, abs (Rz.F - F_True) / F_True);
                  Worst_E := Long_Float'Max (Worst_E, Em); Worst_R := Long_Float'Max (Worst_R, Rm);
                  Append (Passes, " " & Codec.Fmt (Rz.F, 1) & "(" & Codec.Img (Rz.Mv_Passes) & " 遍、" & Codec.Fmt (Rz.Mv_Px, 3) & " px、" & Codec.Fmt (Em, 2) & " mm)");
               end;
            end loop;
            Check (Worst_F < 0.005 and then Worst_E < 1.0 and then Worst_R < 0.05,
                   "运动学·龙门架各少一个点再解三次都回到同一个解:焦距(④ 做了几遍、重投影中位、考试最大)" & To_String (Passes) & " · 焦距最多差 "
                   & Codec.Fmt (100.0 * Worst_F, 2) & "%(要 < 0.5%)、最大 " & Codec.Fmt (Worst_E, 3) & " mm(要 < 1)、朝向 " & Codec.Fmt (Worst_R, 4) & "°(要 < 0.05)");
         end;
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
                      Tip => [0.1, -0.2, -1.7], Gap => 1.75, Stride => 0.888, Stride_Rot => 0.161, Fixed => True, Pos => [5.7, -2.75, 10.35],
                      Lobes => Geom.Lobe_Geo_Vectors.Empty_Vector, Tip_Sd => 0.0);   --  不动的眼没有手指:这两样恒为空(kin 文件不存)
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
               W.Got_Lo.Append (-0.25 - 0.01 * Long_Float (J)); W.Got_Hi.Append (0.35 + 0.02 * Long_Float (J + A));   --  到过的范围:到过的范围、往外一步
               W.Step_Lo.Append (0.0123 * Long_Float (J + 1)); W.Step_Hi.Append (0.0456 * Long_Float (J + 1));
            end loop;
            W.Eye_W := 8;
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
               if Natural (K2.Worlds (A).Got_Lo.Length) /= 6 or else Natural (K2.Worlds (A).Got_Hi.Length) /= 6
                 or else Natural (K2.Worlds (A).Step_Lo.Length) /= 6 or else Natural (K2.Worlds (A).Step_Hi.Length) /= 6
               then
                  Worst := 1.0;
               else
                  Cmp (K.Worlds (A).Got_Lo (J), K2.Worlds (A).Got_Lo (J)); Cmp (K.Worlds (A).Got_Hi (J), K2.Worlds (A).Got_Hi (J));
                  Cmp (K.Worlds (A).Step_Lo (J), K2.Worlds (A).Step_Lo (J)); Cmp (K.Worlds (A).Step_Hi (J), K2.Worlds (A).Step_Hi (J));
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
            Cmp (Long_Float (K.Worlds (A).Eye_W), Long_Float (K2.Worlds (A).Eye_W));
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
             & "(要 < 1e-8;不动的眼整份相机几何、板上每个点的每一项、每根轴是转是走、没量到头的界、到过的范围和往外一步、画幅、核对用的图、钥匙原样回来)· " & To_String (Note));
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
            Ada.Text_IO.Put_Line (Fo, (if L'Length >= 4 and then L (L'First .. L'First + 3) = "kin " then "kin 3" else L));
         end loop;
         Ada.Text_IO.Close (Fo);
         Jointboot.Load_Kin (Path, K3, Ok3, Note3);
         Check (not Ok3, "⑤ 旧版前半段文件(kin 3:没存关节到过的范围)不装回 ⇒ 从零量:" & To_String (Note3));
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


   Put_Line ((if Fails = 0 then "🟢 自检全过" else "🔴 自检失败" & Natural'Image (Fails) & " 条"));
   if Fails > 0 then
      raise Program_Error;
   end if;
end Selfcheck;
