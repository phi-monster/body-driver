with Ada.Streams; use Ada.Streams;
with Ada.Text_IO;
with Ada.Exceptions;
with Ada.Containers;
with Ada.Strings.Fixed;
with GNAT.SHA1;
with Codec;
with Interfaces; use Interfaces;
package body Websocket is
   use GNAT.Sockets;
   GUID : constant String := "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

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
      Head : String (1 .. 16384);
      N : Natural := 0;
      One : Stream_Element_Array (1 .. 1);
      Done : Boolean := False;
      Key_Tag : constant String := "Sec-WebSocket-Key:";
   begin
      Ok := False;
      while N < Head'Last and then not Done loop
         Read_Exact (C, One, Ok);
         exit when not Ok;
         N := N + 1;
         Head (N) := Character'Val (One (1));
         if N >= 4 and then Head (N - 3 .. N) = ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF then
            Done := True;
         end if;
      end loop;
      if not Done then
         Ok := False;
         return;
      end if;
      declare
         H : constant String := Head (1 .. N);
         P : constant Natural := Ada.Strings.Fixed.Index (H, Key_Tag);
         E : Natural;
      begin
         if P = 0 then
            Ok := False;
            return;
         end if;
         E := Ada.Strings.Fixed.Index (H, ASCII.CR & "", P);
         if E = 0 then
            Ok := False;
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
      declare
         Total : constant Natural := Natural (Hdr.Length) + L;
         A : Stream_Element_Array (1 .. Stream_Element_Offset (Total));
         P : Stream_Element_Offset := 1;
      begin
         for B of Hdr loop
            A (P) := Stream_Element (B); P := P + 1;
         end loop;
         for B of Data loop
            A (P) := Stream_Element (B); P := P + 1;
         end loop;
         Send_All (C, A, Ok);
      end;
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
      Len : Natural;
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
         Len := Natural (Unsigned_8 (H2 (2)) and 16#7F#);
         if Len = 126 then
            declare
               X : Stream_Element_Array (1 .. 2);
            begin
               Read_Exact (C, X, Ok);
               if not Ok then
                  return;
               end if;
               Len := Natural (X (1)) * 256 + Natural (X (2));
            end;
         elsif Len = 127 then
            declare
               X : Stream_Element_Array (1 .. 8);
               V : Unsigned_64 := 0;
            begin
               Read_Exact (C, X, Ok);
               if not Ok then
                  return;
               end if;
               for I in X'Range loop
                  V := Shift_Left (V, 8) or Unsigned_64 (X (I));
               end loop;
               if V > 512 * 1024 * 1024 then
                  Ok := False;
                  return;
               end if;
               Len := Natural (V);
            end;
         end if;
         if Masked then
            Read_Exact (C, Mask, Ok);
            if not Ok then
               return;
            end if;
         end if;
         declare
            Payload : Stream_Element_Array (1 .. Stream_Element_Offset (Len));
            Chunk : Buf;
         begin
            if Len > 0 then
               Read_Exact (C, Payload, Ok);
               if not Ok then
                  return;
               end if;
            end if;
            Chunk.Reserve_Capacity (Ada.Containers.Count_Type (Len));
            for I in 0 .. Len - 1 loop
               declare
                  B : Unsigned_8 := Unsigned_8 (Payload (Stream_Element_Offset (I + 1)));
               begin
                  if Masked then
                     B := B xor Unsigned_8 (Mask (Stream_Element_Offset (I mod 4 + 1)));
                  end if;
                  Chunk.Append (B);
               end;
            end loop;
            case Opcode is
               when 8 =>
                  Kind := Op_Close;
                  return;
               when 9 =>
                  declare
                     Pong_Ok : Boolean;
                  begin
                     Send_Frame (C, 10, Chunk, Pong_Ok);
                  end;
               when 10 =>
                  null;
               when 0 | 1 | 2 =>
                  if Opcode /= 0 then
                     Msg_Op := Opcode;
                     Data.Clear;
                  end if;
                  for B of Chunk loop
                     Data.Append (B);
                  end loop;
                  if Fin then
                     Kind := (if Msg_Op = 1 then Op_Text else Op_Binary);
                     return;
                  end if;
               when others =>
                  null;
            end case;
         end;
      end loop;
   end Read_Message;

   procedure Close (C : in out Conn) is
   begin
      if C.Open then
         Close_Socket (C.Sock);
         C.Open := False;
      end if;
      if C.Listening then
         Close_Socket (C.Listener);
         C.Listening := False;
      end if;
   exception
      when others => null;
   end Close;
end Websocket;
