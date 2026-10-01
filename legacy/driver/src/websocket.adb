with Ada.Streams; use Ada.Streams;
with Ada.Text_IO;
with Ada.Exceptions;
with Ada.Containers;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with GNAT.SHA1;
with Codec;
with Interfaces; use Interfaces;
package body Websocket is
   use GNAT.Sockets;
   use type Ada.Containers.Count_Type;
   GUID : constant String := "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

   --  一帧的负载放在堆上。原来是栈上的 Stream_Element_Array (1 .. Len):几台 720p 的 RGB-D 一帧十几 MB,
   --  比主线程的栈(常见 8 MB)大,先撞栈(Storage_Error),轮不到任何上限(09-30 审计 H6 查出)
   type Bytes_Access is access Stream_Element_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Stream_Element_Array, Bytes_Access);

   function Img (V : Unsigned_64) return String is (Ada.Strings.Fixed.Trim (Unsigned_64'Image (V), Ada.Strings.Both));

   procedure Read_Exact (C : in out Conn; A : out Stream_Element_Array; Ok : out Boolean) is
      Last : Stream_Element_Offset;
      Got : Stream_Element_Offset := A'First - 1;
   begin
      Ok := False;
      while Got < A'Last loop
         Receive_Socket (C.Sock, A (Got + 1 .. A'Last), Last);
         if Last < Got + 1 then
            return;      --  对端关了
         end if;
         Got := Last;
      end loop;
      Ok := True;
   exception
      when others => Ok := False;
   end Read_Exact;

   procedure Send_All (C : in out Conn; A : Stream_Element_Array; Ok : out Boolean) is
      Last : Stream_Element_Offset;
      Sent : Stream_Element_Offset := A'First - 1;
   begin
      Ok := False;
      while Sent < A'Last loop
         Send_Socket (C.Sock, A (Sent + 1 .. A'Last), Last);
         if Last < Sent + 1 then
            return;
         end if;
         Sent := Last;
      end loop;
      Ok := True;
   exception
      when others => Ok := False;
   end Send_All;

   procedure Listen (Port : Natural; C : in out Conn; Ok : out Boolean) is
   begin
      Ok := False;
      Create_Socket (C.Listener);
      Set_Socket_Option (C.Listener, Socket_Level, (Reuse_Address, True));
      Bind_Socket (C.Listener, (Family => Family_Inet, Addr => Any_Inet_Addr, Port => Port_Type (Port)));
      Listen_Socket (C.Listener);
      C.Listening := True;
      Accept_Client (C, Ok);
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("[链] 占不住端口 " & Codec.Img (Port) & ":" & Ada.Exceptions.Exception_Message (E));
         Ok := False;
   end Listen;

   procedure Handshake (C : in out Conn; Ok : out Boolean) is
      --  请求头读到空行(CRLF CRLF)为止,缓冲跟着长。原来是定长 16 KiB(拍的):请求头带大 cookie、经代理加了头,读满还没见到空行就判握手失败
      Head : Unbounded_String;
      One : Stream_Element_Array (1 .. 1);
      Blank : constant String := ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF;
      Key_Tag : constant String := "Sec-WebSocket-Key:";
   begin
      Ok := False;
      loop
         Read_Exact (C, One, Ok);
         if not Ok then
            return;
         end if;
         Append (Head, Character'Val (One (1)));
         exit when Length (Head) >= Blank'Length and then Tail (Head, Blank'Length) = Blank;
      end loop;
      Ok := False;
      declare
         H : constant String := To_String (Head);
         P : constant Natural := Ada.Strings.Fixed.Index (H, Key_Tag);
         E : Natural;
      begin
         if P = 0 then
            return;
         end if;
         E := Ada.Strings.Fixed.Index (H, ASCII.CR & "", P);
         if E = 0 then
            return;
         end if;
         declare
            Key : constant String := Ada.Strings.Fixed.Trim (H (P + Key_Tag'Length .. E - 1), Ada.Strings.Both);
            Hex : constant String := GNAT.SHA1.Digest (Key & GUID);
            Acc : constant String := Codec.Base64 (Codec.Hex_To_Bytes (Hex));
            Resp : constant String :=
              "HTTP/1.1 101 Switching Protocols" & ASCII.CR & ASCII.LF &
              "Upgrade: websocket" & ASCII.CR & ASCII.LF &
              "Connection: Upgrade" & ASCII.CR & ASCII.LF &
              "Sec-WebSocket-Accept: " & Acc & ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF;
            A : Stream_Element_Array (1 .. Stream_Element_Offset (Resp'Length));
         begin
            for I in Resp'Range loop
               A (Stream_Element_Offset (I)) := Stream_Element (Character'Pos (Resp (I)));
            end loop;
            Send_All (C, A, Ok);
         end;
      end;
   end Handshake;

   procedure Accept_Client (C : in out Conn; Ok : out Boolean) is
      Addr : Sock_Addr_Type;
   begin
      Ok := False;
      if not C.Listening then
         return;
      end if;
      if C.Open then
         begin
            Close_Socket (C.Sock);
         exception
            when others => null;
         end;
         C.Open := False;
      end if;
      Accept_Socket (C.Listener, C.Sock, Addr);
      C.Open := True;
      Handshake (C, Ok);
      if not Ok then
         Close_Socket (C.Sock);
         C.Open := False;
      end if;
   exception
      when others => Ok := False;
   end Accept_Client;

   procedure Send_Frame (C : in out Conn; Opcode : Unsigned_8; Data : Buf; Ok : out Boolean) is
      L : constant Natural := Natural (Data.Length);
      Hdr : Buf;
      A : Bytes_Access;
   begin
      Hdr.Append (16#80# or Opcode);
      if L < 126 then
         Hdr.Append (Unsigned_8 (L));
      elsif L < 65536 then
         Hdr.Append (126);
         Hdr.Append (Unsigned_8 (L / 256)); Hdr.Append (Unsigned_8 (L mod 256));
      else
         Hdr.Append (127);
         for K in reverse 0 .. 7 loop
            Hdr.Append (Unsigned_8 (Shift_Right (Unsigned_64 (L), 8 * K) and 16#FF#));
         end loop;
      end if;
      --  整帧也在堆上拼(同 Read_Message:大的一帧不上栈)
      A := new Stream_Element_Array (1 .. Stream_Element_Offset (Natural (Hdr.Length) + L));
      declare
         P : Stream_Element_Offset := 1;
      begin
         for B of Hdr loop
            A (P) := Stream_Element (B); P := P + 1;
         end loop;
         for B of Data loop
            A (P) := Stream_Element (B); P := P + 1;
         end loop;
      end;
      Send_All (C, A.all, Ok);
      Free (A);
   exception
      when Storage_Error =>
         Ada.Text_IO.Put_Line ("[链] 要发的一帧 " & Codec.Img (L) & " 字节,内存里放不下 ⇒ 没发出去");
         Free (A);
         Ok := False;
   end Send_Frame;

   procedure Send_Binary (C : in out Conn; Data : Buf; Ok : out Boolean) is
   begin
      if not C.Open then
         Ok := False;
         return;
      end if;
      Send_Frame (C, 2, Data, Ok);
   end Send_Binary;

   procedure Read_Message (C : in out Conn; Kind : out Op; Data : out Buf; Ok : out Boolean) is
      H2 : Stream_Element_Array (1 .. 2);
      Fin, Masked : Boolean;
      Opcode : Unsigned_8;
      Len : Unsigned_64;
      Mask : Stream_Element_Array (1 .. 4) := [others => 0];
      Msg_Op : Unsigned_8 := 0;
   begin
      Kind := Op_None;
      Data.Clear;
      Ok := C.Open;
      if not Ok then
         return;
      end if;
      loop
         Read_Exact (C, H2, Ok);
         if not Ok then
            return;
         end if;
         Fin := (Unsigned_8 (H2 (1)) and 16#80#) /= 0;
         Opcode := Unsigned_8 (H2 (1)) and 16#0F#;
         Masked := (Unsigned_8 (H2 (2)) and 16#80#) /= 0;
         Len := Unsigned_64 (Unsigned_8 (H2 (2)) and 16#7F#);
         if Len = 126 then
            declare
               X : Stream_Element_Array (1 .. 2);
            begin
               Read_Exact (C, X, Ok);
               if not Ok then
                  return;
               end if;
               Len := Unsigned_64 (X (1)) * 256 + Unsigned_64 (X (2));
            end;
         elsif Len = 127 then
            declare
               X : Stream_Element_Array (1 .. 8);
            begin
               Read_Exact (C, X, Ok);
               if not Ok then
                  return;
               end if;
               Len := 0;
               for I in X'Range loop
                  Len := Shift_Left (Len, 8) or Unsigned_64 (X (I));
               end loop;
            end;
         end if;
         --  一条消息多大,只有两道边,都不是拍的:收下来的字节串按 Natural 编号(最多 Natural'Last 个字节),再就是内存给不给(分配不到照实说)。
         --  原来另有一道"一条消息最多 512 MiB"(拍的),删了
         if Len > Unsigned_64 (Natural'Last) or else Len + Unsigned_64 (Data.Length) > Unsigned_64 (Natural'Last) then
            Ada.Text_IO.Put_Line ("[链] 对方说这一帧有 " & Img (Len) & " 字节,连同这条消息已收的 " & Img (Unsigned_64 (Data.Length))
                                  & " 字节,超过一条消息能编号的 " & Codec.Img (Natural'Last) & " 字节 ⇒ 当线断了");
            Ok := False;
            return;
         end if;
         if Masked then
            Read_Exact (C, Mask, Ok);
            if not Ok then
               return;
            end if;
         end if;
         declare
            N : constant Natural := Natural (Len);
            Payload : Bytes_Access;
         begin
            begin
               Payload := new Stream_Element_Array (1 .. Stream_Element_Offset (N));
            exception
               when Storage_Error =>
                  Ada.Text_IO.Put_Line ("[链] 这一帧 " & Codec.Img (N) & " 字节,内存里放不下 ⇒ 当线断了");
                  Ok := False;
                  return;
            end;
            if N > 0 then
               Read_Exact (C, Payload.all, Ok);
               if not Ok then
                  Free (Payload);
                  return;
               end if;
            end if;
            if Masked then
               for I in Payload'Range loop
                  Payload (I) := Payload (I) xor Mask ((I - Payload'First) mod Mask'Length + Mask'First);
               end loop;
            end if;
            case Opcode is
               when 8 =>
                  Kind := Op_Close;
                  Free (Payload);
                  return;
               when 9 =>
                  declare
                     Pong : Buf;
                     Pong_Ok : Boolean;
                  begin
                     for B of Payload.all loop
                        Pong.Append (Unsigned_8 (B));
                     end loop;
                     Send_Frame (C, 10, Pong, Pong_Ok);
                  end;
               when 10 =>
                  null;
               when 0 | 1 | 2 =>
                  if Opcode /= 0 then
                     Msg_Op := Opcode;
                     Data.Clear;
                  end if;
                  Data.Reserve_Capacity (Data.Length + Ada.Containers.Count_Type (N));
                  for B of Payload.all loop
                     Data.Append (Unsigned_8 (B));
                  end loop;
                  if Fin then
                     Kind := (if Msg_Op = 1 then Op_Text else Op_Binary);
                     Free (Payload);
                     return;
                  end if;
               when others =>
                  null;
            end case;
            Free (Payload);
         end;
      end loop;
   end Read_Message;

end Websocket;
