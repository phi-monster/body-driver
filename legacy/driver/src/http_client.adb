with GNAT.Sockets; use GNAT.Sockets;
with Ada.Streams; use Ada.Streams;
with Ada.Real_Time; use Ada.Real_Time;
with Ada.Exceptions;
with Ada.Text_IO;
with Ada.Characters.Handling;
with Ada.Strings.Fixed;
with Ada.Unchecked_Deallocation;
with Codec;
package body Http_Client is
   use type Ada.Exceptions.Exception_Id;
   CRLF : constant String := ASCII.CR & ASCII.LF;
   Blank : constant String := CRLF & CRLF;   --  头和正文之间的空行
   Http_Tag : constant String := "HTTP/";    --  状态行打头的那几个字

   --  套接字的超时按 timeval 记,最小一格是 1 微秒(格式);比一格短的会被记成 0,而 0 = 永远等 ⇒ 剩下不到一格就算时间到了
   Timeval_Tick : constant Duration := 0.000_001;
   Time_Up : exception;

   type Bytes_Access is access Stream_Element_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Stream_Element_Array, Bytes_Access);
   type Text_Access is access String;
   procedure Free is new Ada.Unchecked_Deallocation (String, Text_Access);

   --  一次往套接字里写 / 从里面读多少字节 = 操作系统给这个套接字的缓冲有多大(原来拍的 64 KB;这个大小只管系统调用几次,不管对错)。
   --  缓冲开在堆上:几 MB 的系统缓冲也不上栈
   function Piece (S : Socket_Type; Name : Option_Name) return Natural is
      O : constant Option_Type := Get_Socket_Option (S, Socket_Level, Name);
   begin
      if O.Size = 0 then
         raise Socket_Error with "操作系统说这个套接字的缓冲是 0 字节";
      end if;
      return O.Size;
   end Piece;

   --  下一次会卡住的收 / 发最多等到 Deadline:整次请求一个时限,不是每读一次一个时限(原来是每次 receive 各等 1800 秒,对方一点一点吐字节能无限拖)
   procedure Arm (S : Socket_Type; Receiving : Boolean; Deadline : Time) is
      Left : constant Duration := To_Duration (Deadline - Clock);
   begin
      if Left < Timeval_Tick then
         raise Time_Up;
      end if;
      if Receiving then
         Set_Socket_Option (S, Socket_Level, (Receive_Timeout, Left));
      else
         Set_Socket_Option (S, Socket_Level, (Send_Timeout, Left));
      end if;
   end Arm;

   --  发 N 个字符(第 I 个由 Char_At 给):按系统缓冲那么大一段一段地拷进堆上的缓冲再发,整份请求不在栈上摆成一个大数组
   procedure Send_Chars (S : Socket_Type; N : Natural; Char_At : not null access function (I : Positive) return Character;
                         Size : Natural; Deadline : Time) is
      A : Bytes_Access := new Stream_Element_Array (1 .. Stream_Element_Offset (Natural'Min (Size, N)));
      Done : Natural := 0;
   begin
      while Done < N loop
         declare
            M : constant Natural := Natural'Min (Size, N - Done);
            Sent : Stream_Element_Offset := 0;
            Last : Stream_Element_Offset;
         begin
            for I in 1 .. M loop
               A (Stream_Element_Offset (I)) := Stream_Element (Character'Pos (Char_At (Done + I)));
            end loop;
            while Sent < Stream_Element_Offset (M) loop
               Arm (S, False, Deadline);
               Send_Socket (S, A (Sent + 1 .. Stream_Element_Offset (M)), Last);
               if Last < Sent + 1 then
                  raise Socket_Error with "对端不收了";
               end if;
               Sent := Last;
            end loop;
            Done := Done + M;
         end;
      end loop;
      Free (A);
   exception
      when others =>
         Free (A);
         raise;
   end Send_Chars;

   procedure Send_Text (S : Socket_Type; T : String; Size : Natural; Deadline : Time) is
      function Char_At (I : Positive) return Character is (T (T'First + I - 1));
   begin
      Send_Chars (S, T'Length, Char_At'Access, Size, Deadline);
   end Send_Text;

   --  分块编码的正文拼回来:每块 = 十六进制的长度(分号后面的扩展不管)· CRLF · 那么多字节 · CRLF;长度 0 的块收尾(后面的尾部头不管)。
   --  拼不回来(长度看不懂、块被截断)⇒ False
   function Dechunk (B : in out Unbounded_String) return Boolean is
      R : Unbounded_String;
      I : Positive := 1;
   begin
      loop
         if I > Length (B) then
            return False;
         end if;
         declare
            E : constant Natural := Index (B, CRLF, I);
         begin
            if E = 0 then
               return False;
            end if;
            declare
               Line : constant String := Slice (B, I, E - 1);
               Semi : constant Natural := Ada.Strings.Fixed.Index (Line, ";");
               Hex : constant String := Ada.Strings.Fixed.Trim ((if Semi = 0 then Line else Line (Line'First .. Semi - 1)), Ada.Strings.Both);
               N : constant Natural := Natural'Value ("16#" & Hex & "#");   --  Ada 的十六进制写法
               From : constant Positive := E + CRLF'Length;
            begin
               if N = 0 then
                  B := R;
                  return True;
               end if;
               if From + N - 1 + CRLF'Length > Length (B) or else Slice (B, From + N, From + N - 1 + CRLF'Length) /= CRLF then
                  return False;
               end if;
               Append (R, Unbounded_Slice (B, From, From + N - 1));
               I := From + N + CRLF'Length;
            end;
         end;
      end loop;
   exception
      when Constraint_Error =>
         return False;
   end Dechunk;

   --  回包拆成 状态行 · 头 · 正文:2xx 才算成;正文按 Content-Length 核长短,分块编码按块拼回
   function Parse (Raw : Unbounded_String; Where : String; Reply_Body, Why : out Unbounded_String) return Boolean is
      P : constant Natural := Index (Raw, Blank);
   begin
      Reply_Body := Null_Unbounded_String;
      Why := Null_Unbounded_String;
      if P = 0 then
         Why := To_Unbounded_String ("从 " & Where & " 收到的不是完整的 HTTP 回包(" & Codec.Img (Length (Raw)) & " 字节,头和正文之间没有空行)");
         return False;
      end if;
      declare
         Head : constant String := Slice (Raw, 1, P - 1);
         Bdy : Unbounded_String := Unbounded_Slice (Raw, P + Blank'Length, Length (Raw));
         E1 : constant Natural := Ada.Strings.Fixed.Index (Head, CRLF);
         Status : constant String := (if E1 = 0 then Head else Head (Head'First .. E1 - 1));
         Sp1 : constant Natural := Ada.Strings.Fixed.Index (Status, " ");
         Code : Natural := 0;
         --  头里某一项的值:名字不分大小写,没有这一项 = ""
         function Header (Name : String) return String is
            Lo : constant String := Ada.Characters.Handling.To_Lower (Name);
            I : Natural := (if E1 = 0 then Head'Last + 1 else E1 + CRLF'Length);
         begin
            while I <= Head'Last loop
               declare
                  J : constant Natural := Ada.Strings.Fixed.Index (Head (I .. Head'Last), CRLF);
                  Line : constant String := Head (I .. (if J = 0 then Head'Last else J - 1));
                  C : constant Natural := Ada.Strings.Fixed.Index (Line, ":");
               begin
                  if C > 0 and then Ada.Characters.Handling.To_Lower (Ada.Strings.Fixed.Trim (Line (Line'First .. C - 1), Ada.Strings.Both)) = Lo then
                     return Ada.Strings.Fixed.Trim (Line (C + 1 .. Line'Last), Ada.Strings.Both);
                  end if;
                  I := (if J = 0 then Head'Last + 1 else J + CRLF'Length);
               end;
            end loop;
            return "";
         end Header;
      begin
         if Status'Length < Http_Tag'Length or else Status (Status'First .. Status'First + Http_Tag'Length - 1) /= Http_Tag or else Sp1 = 0 then
            Why := To_Unbounded_String ("从 " & Where & " 回来的不是 HTTP:" & Status);
            return False;
         end if;
         declare
            Rest : constant String := Status (Sp1 + 1 .. Status'Last);
            Sp2 : constant Natural := Ada.Strings.Fixed.Index (Rest, " ");
         begin
            Code := Natural'Value (if Sp2 = 0 then Rest else Rest (Rest'First .. Sp2 - 1));
         exception
            when Constraint_Error =>
               Why := To_Unbounded_String ("从 " & Where & " 回来的状态行看不懂:" & Status);
               return False;
         end;
         if Ada.Strings.Fixed.Index (Ada.Characters.Handling.To_Lower (Header ("Transfer-Encoding")), "chunked") > 0 then
            if not Dechunk (Bdy) then
               Reply_Body := Bdy;
               Why := To_Unbounded_String ("从 " & Where & " 回来的分块正文拼不回来(截断了,或者块长看不懂)");
               return False;
            end if;
         elsif Header ("Content-Length") /= "" then
            declare
               N : Natural;
            begin
               N := Natural'Value (Header ("Content-Length"));
               if Length (Bdy) < N then
                  Reply_Body := Bdy;
                  Why := To_Unbounded_String ("从 " & Where & " 回来的正文短了:对方说 " & Codec.Img (N) & " 字节,只收到 "
                                              & Codec.Img (Length (Bdy)) & " 字节(半路断了)");
                  return False;
               end if;
               Bdy := Unbounded_Slice (Bdy, 1, N);   --  说好多长就是多长
            exception
               when Constraint_Error =>
                  Why := To_Unbounded_String ("从 " & Where & " 回来的 Content-Length 看不懂:" & Header ("Content-Length"));
                  return False;
            end;
         end if;
         Reply_Body := Bdy;
         if Code not in 200 .. 299 then   --  HTTP 状态码 2xx = 成了(格式)
            Why := To_Unbounded_String ("从 " & Where & " 回来的是 " & Status & ":") & Bdy;
            return False;
         end if;
         return True;
      end;
   end Parse;

   --  连上、发请求头、交给 Send_Body 发请求体、读到对方关连接,再拆回包
   function Exchange (Host : String; Port : Natural; Path : String; Body_Len : Natural;
                      Send_Body : not null access procedure (S : Socket_Type; Size : Natural; Deadline : Time);
                      Reply_Body, Why : out Unbounded_String; Timeout_S : Duration) return Boolean is
      Start : constant Time := Clock;
      Deadline : constant Time := Start + To_Time_Span (Timeout_S);
      Where : constant String := Host & ":" & Codec.Img (Port) & Path;
      Req : constant String :=
        "POST " & Path & " HTTP/1.1" & CRLF &
        "Host: " & Host & CRLF &
        "Content-Type: application/json" & CRLF &
        "Content-Length: " & Codec.Img (Body_Len) & CRLF &
        "Connection: close" & Blank;
      S : Socket_Type := No_Socket;
      Opened : Boolean := False;
      Addr : Sock_Addr_Type;
      Raw : Unbounded_String;
      Doing : Unbounded_String;   --  正在干哪一步(出事时照实说是在哪一步)
      Rx : Bytes_Access;
      Tx : Text_Access;
      function Used return String is
        (Codec.Fmt (Long_Float (To_Duration (Clock - Start)), 1) & " 秒,收到 " & Codec.Img (Length (Raw)) & " 字节");
      procedure Close is
      begin
         if Opened then
            Opened := False;
            Close_Socket (S);
         end if;
      exception
         when others => null;
      end Close;
   begin
      Reply_Body := Null_Unbounded_String;
      Why := Null_Unbounded_String;
      Doing := To_Unbounded_String ("连 " & Where);
      Create_Socket (S);
      Opened := True;
      Addr.Addr := Addresses (Get_Host_By_Name (Host), 1);
      Addr.Port := Port_Type (Port);
      declare
         St : Selector_Status;
         Left : constant Duration := To_Duration (Deadline - Clock);
      begin
         if Left < Timeval_Tick then
            raise Time_Up;
         end if;
         Connect_Socket (S, Addr, Timeout => Left, Status => St);
         if St /= Completed then
            raise Time_Up;
         end if;
      end;
      Doing := To_Unbounded_String ("给 " & Where & " 发请求");
      Send_Text (S, Req, Piece (S, Send_Buffer), Deadline);
      Send_Body (S, Piece (S, Send_Buffer), Deadline);
      Doing := To_Unbounded_String ("等 " & Where & " 回话");
      declare
         Size : constant Natural := Piece (S, Receive_Buffer);
         Last : Stream_Element_Offset;
      begin
         Rx := new Stream_Element_Array (1 .. Stream_Element_Offset (Size));
         Tx := new String (1 .. Size);
         loop
            Arm (S, True, Deadline);
            Receive_Socket (S, Rx.all, Last);
            exit when Last < Rx'First;   --  对方关了连接 = 回完了
            for I in 1 .. Last loop
               Tx (Natural (I)) := Character'Val (Rx (I));
            end loop;
            Append (Raw, Tx (1 .. Natural (Last)));
         end loop;
      end;
      Free (Rx);
      Free (Tx);
      Close;
      return Parse (Raw, Where, Reply_Body, Why);
   exception
      when E : others =>
         --  时限到了和别的出事分开说:我们自己数到了时限(Time_Up),或者收 / 发等到套接字的超时 —— 系统报的是"暂时没有数据"(EAGAIN)
         if Ada.Exceptions.Exception_Identity (E) = Time_Up'Identity
           or else (Ada.Exceptions.Exception_Identity (E) = Socket_Error'Identity and then Resolve_Exception (E) = Resource_Temporarily_Unavailable)
         then
            Why := Doing & " 时到了时限 " & Codec.Fmt (Long_Float (Timeout_S), 1) & " 秒还没完(" & Used & ")";
         else
            Why := Doing & " 时出事:" & Ada.Exceptions.Exception_Message (E) & "(" & Used & ")";
         end if;
         Free (Rx);
         Free (Tx);
         Close;
         return False;
   end Exchange;

   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body, Why : out Unbounded_String; Timeout_S : Duration := Default_Timeout) return Boolean is
      procedure Send_Body (S : Socket_Type; Size : Natural; Deadline : Time) is
      begin
         Send_Text (S, Body_Text, Size, Deadline);
      end Send_Body;
   begin
      return Exchange (Host, Port, Path, Body_Text'Length, Send_Body'Access, Reply_Body, Why, Timeout_S);
   end Post;

   function Post (Host : String; Port : Natural; Path : String; Body_Text : Unbounded_String;
                  Reply_Body, Why : out Unbounded_String; Timeout_S : Duration := Default_Timeout) return Boolean is
      procedure Send_Body (S : Socket_Type; Size : Natural; Deadline : Time) is
         function Char_At (I : Positive) return Character is (Element (Body_Text, I));
      begin
         Send_Chars (S, Length (Body_Text), Char_At'Access, Size, Deadline);
      end Send_Body;
   begin
      return Exchange (Host, Port, Path, Length (Body_Text), Send_Body'Access, Reply_Body, Why, Timeout_S);
   end Post;

end Http_Client;
