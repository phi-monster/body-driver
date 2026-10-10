with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with GNAT.SHA1;
with Interfaces;
with Driver.Base64;

package body Driver.Wire is

   use Driver.Bytes;
   use GNAT.Sockets;
   use Interfaces;
   use type Offset;
   use type Byte;

   Accept_Suffix : constant String := "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
   --  RFC 6455 4.2.2: appended to the client's key before hashing.

   Line_End : constant String := ASCII.CR & ASCII.LF;
   Blank    : constant String := Line_End & Line_End;

   type Byte_Array_Access is access Byte_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Byte_Array, Byte_Array_Access);

   protected body Send_Lock is
      entry Seize when not Busy is
      begin
         Busy := True;
      end Seize;

      procedure Release is
      begin
         Busy := False;
      end Release;
   end Send_Lock;

   procedure Read_Exactly (C : Connection; Item : out Byte_Array; Ok : out Boolean) is
      Got  : Offset := Item'First - 1;
      Last : Offset;
   begin
      Ok := False;
      while Got < Item'Last loop
         Receive_Socket (C.Peer, Item (Got + 1 .. Item'Last), Last);
         if Last <= Got then
            return;   --  the other end closed the connection
         end if;
         Got := Last;
      end loop;
      Ok := True;
   exception
      when Socket_Error =>
         Ok := False;
   end Read_Exactly;

   procedure Send_All (C : Connection; Item : Byte_Array; Ok : out Boolean) is
      Sent : Offset := Item'First - 1;
      Last : Offset;
   begin
      Ok := False;
      while Sent < Item'Last loop
         Send_Socket (C.Peer, Item (Sent + 1 .. Item'Last), Last);
         if Last <= Sent then
            return;
         end if;
         Sent := Last;
      end loop;
      Ok := True;
   exception
      when Socket_Error =>
         Ok := False;
   end Send_All;

   function Digest_Bytes (Hex : String) return Byte_Array is
      R : Byte_Array (1 .. Offset (Hex'Length / 2));
   begin
      for I in R'Range loop
         R (I) := Byte'Value ("16#" & Hex (Hex'First + 2 * Natural (I - 1) .. Hex'First + 2 * Natural (I - 1) + 1) & "#");
      end loop;
      return R;
   end Digest_Bytes;

   function Accept_Key (Key : String) return String is
     (Driver.Base64.Encode (Digest_Bytes (GNAT.SHA1.Digest (Key & Accept_Suffix))));

   procedure Read_Head (C : Connection; Head : out Ada.Strings.Unbounded.Unbounded_String; Ok : out Boolean) is
      use Ada.Strings.Unbounded;
      One : Byte_Array (1 .. 1);
   begin
      Head := Null_Unbounded_String;
      loop
         Read_Exactly (C, One, Ok);
         if not Ok then
            return;
         end if;
         Append (Head, Character'Val (One (1)));
         exit when Length (Head) >= Blank'Length
           and then Slice (Head, Length (Head) - Blank'Length + 1, Length (Head)) = Blank;
      end loop;
   end Read_Head;

   function Header_Value (Head : String; Name : String) return String is
      --  The value of a header (Name in lower case, with its colon), or "".
      Lower : String := Head;
   begin
      for Ch of Lower loop
         if Ch in 'A' .. 'Z' then
            Ch := Character'Val (Character'Pos (Ch) + 32);
         end if;
      end loop;
      declare
         P : constant Natural := Ada.Strings.Fixed.Index (Lower, Name);
         E : constant Natural := (if P = 0 then 0 else Ada.Strings.Fixed.Index (Head, "" & ASCII.CR, P));
      begin
         if P = 0 or else E = 0 then
            return "";
         end if;
         return Ada.Strings.Fixed.Trim (Head (P + Name'Length .. E - 1), Ada.Strings.Both);
      end;
   end Header_Value;

   procedure Handshake (C : Connection; Ok : out Boolean) is
      Head : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Read_Head (C, Head, Ok);
      if not Ok then
         return;
      end if;
      declare
         Key : constant String := Header_Value (Ada.Strings.Unbounded.To_String (Head), "sec-websocket-key:");
      begin
         Ok := Key /= "";
         if Ok then
            Send_All (C, To_Bytes ("HTTP/1.1 101 Switching Protocols" & Line_End
                                   & "Upgrade: websocket" & Line_End
                                   & "Connection: Upgrade" & Line_End
                                   & "Sec-WebSocket-Accept: " & Accept_Key (Key) & Blank), Ok);
         end if;
      end;
   end Handshake;

   procedure Drop_Peer (C : in out Connection) is
   begin
      C.Open := False;
      if C.Peer /= No_Socket then
         Close_Socket (C.Peer);
         C.Peer := No_Socket;
      end if;
   exception
      when Socket_Error =>
         C.Peer := No_Socket;
   end Drop_Peer;

   procedure Listen (C : in out Connection; Port : Natural; Ok : out Boolean) is
   begin
      Create_Socket (C.Listener);
      Set_Socket_Option (C.Listener, Socket_Level, (Reuse_Address, True));
      Bind_Socket (C.Listener, (Family => Family_Inet, Addr => Any_Inet_Addr, Port => Port_Type (Port)));
      Listen_Socket (C.Listener);
      Ok := True;
   exception
      when Socket_Error =>
         Ok := False;
   end Listen;

   procedure Accept_Client (C : in out Connection; Ok : out Boolean) is
      Address : Sock_Addr_Type;
   begin
      --  The previous client's socket is closed here rather than when it went
      --  away, so no other task can still be using it.
      Drop_Peer (C);
      C.Client := False;
      Accept_Socket (C.Listener, C.Peer, Address);
      --  A reply goes out the moment it is written. With Nagle's algorithm the
      --  payload of a frame, written after its header, waited for the robot's
      --  acknowledgement of the header, which the robot delays: every reply
      --  of a few bytes took 40 ms to arrive (A54: the robot's get_action 42 ms
      --  a beat for the driver's 0.2).
      Set_Socket_Option (C.Peer, IP_Protocol_For_TCP_Level, (No_Delay, True));
      Handshake (C, Ok);
      if Ok then
         C.Open := True;
      else
         Drop_Peer (C);
      end if;
   exception
      when Socket_Error =>
         Ok := False;
   end Accept_Client;

   procedure Connect (C : in out Connection; Host : String; Port : Natural; Ok : out Boolean) is
      Nonce : Byte_Array (1 .. 16);   --  RFC 6455 4.1: a random 16-byte key
      Head  : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Drop_Peer (C);
      C.Client := True;
      Mask_Keys.Reset (C.Keys);
      for B of Nonce loop
         B := Mask_Keys.Random (C.Keys);
      end loop;
      Create_Socket (C.Peer);
      Set_Socket_Option (C.Peer, IP_Protocol_For_TCP_Level, (No_Delay, True));   --  as for a robot (Accept_Client)
      Connect_Socket (C.Peer, (Family => Family_Inet, Addr => Addresses (Get_Host_By_Name (Host), 1),
                               Port => Port_Type (Port)));
      declare
         Key  : constant String := Driver.Base64.Encode (Nonce);
         Port_Text : constant String := Ada.Strings.Fixed.Trim (Natural'Image (Port), Ada.Strings.Both);
      begin
         Send_All (C, To_Bytes ("GET / HTTP/1.1" & Line_End
                                & "Host: " & Host & ":" & Port_Text & Line_End
                                & "Upgrade: websocket" & Line_End
                                & "Connection: Upgrade" & Line_End
                                & "Sec-WebSocket-Key: " & Key & Line_End
                                & "Sec-WebSocket-Version: 13" & Blank), Ok);
         if Ok then
            Read_Head (C, Head, Ok);
         end if;
         if Ok then
            declare
               H : constant String := Ada.Strings.Unbounded.To_String (Head);
               Status : constant String := "HTTP/1.1 101";
            begin
               Ok := H'Length >= Status'Length
                 and then H (H'First .. H'First + Status'Length - 1) = Status
                 and then Header_Value (H, "sec-websocket-accept:") = Accept_Key (Key);
            end;
         end if;
      end;
      if Ok then
         C.Open := True;
      else
         Drop_Peer (C);
      end if;
   exception
      when Socket_Error | Host_Error =>
         Drop_Peer (C);
         Ok := False;
   end Connect;

   procedure Send_Frame (C : in out Connection; Opcode : Byte; Data : Byte_Array; Ok : out Boolean) is
      Header : Buffer;
      L      : constant Unsigned_64 := Unsigned_64 (Data'Length);
      Mask   : constant Byte := (if C.Client then 16#80# else 0);
   begin
      Header.Append (16#80# or Opcode);   --  FIN and the opcode
      if L < 126 then
         Header.Append (Mask or Byte (L));
      elsif L < 2 ** 16 then
         Header.Append (Mask or 126);
         Header.Append (Byte (Shift_Right (L, 8) and 16#FF#));
         Header.Append (Byte (L and 16#FF#));
      else
         Header.Append (Mask or 127);
         for K in reverse 0 .. 7 loop
            Header.Append (Byte (Shift_Right (L, 8 * K) and 16#FF#));
         end loop;
      end if;
      C.Sending.Seize;
      begin
         if C.Client then
            declare
               Key    : Byte_Array (1 .. 4);
               Masked : Byte_Array_Access := new Byte_Array (Data'Range);
            begin
               for B of Key loop
                  B := Mask_Keys.Random (C.Keys);
               end loop;
               Header.Append (Key);
               for I in Data'Range loop
                  Masked (I) := Data (I) xor Key ((I - Data'First) mod 4 + 1);
               end loop;
               Send_All (C, Header.To_Array, Ok);
               if Ok then
                  Send_All (C, Masked.all, Ok);
               end if;
               Free (Masked);
            end;
         else
            Send_All (C, Header.To_Array, Ok);
            if Ok then
               Send_All (C, Data, Ok);
            end if;
         end if;
      exception
         when others =>
            C.Sending.Release;
            raise;
      end;
      C.Sending.Release;
   end Send_Frame;

   procedure Send (C : in out Connection; Data : Byte_Array; Ok : out Boolean; Kind : Message_Kind := Binary) is
   begin
      Ok := C.Open;
      if Ok then
         Send_Frame (C, (if Kind = Text then 1 else 2), Data, Ok);
      end if;
   end Send;

   procedure Disconnect (C : in out Connection) is
   begin
      --  Shutting the socket down wakes a Receive blocked on it in another
      --  task; the socket itself is closed when the connection is reused.
      C.Open := False;
      if C.Peer /= No_Socket then
         Shutdown_Socket (C.Peer);
      end if;
   exception
      when Socket_Error =>
         null;   --  already shut down by the other end
   end Disconnect;

   procedure Receive (C : in out Connection; Kind : out Message_Kind; Data : in out Buffer) is
      Two     : Byte_Array (1 .. 2);
      Ok      : Boolean;
      Message : Byte := 0;
   begin
      Data.Clear;
      Kind := Closed;
      if not C.Open then
         return;
      end if;
      loop
         Read_Exactly (C, Two, Ok);
         exit when not Ok;
         declare
            Fin    : constant Boolean := (Two (1) and 16#80#) /= 0;
            Opcode : constant Byte := Two (1) and 16#0F#;
            Masked : constant Boolean := (Two (2) and 16#80#) /= 0;
            Length : Unsigned_64 := Unsigned_64 (Two (2) and 16#7F#);
            Mask   : Byte_Array (1 .. 4) := [others => 0];
         begin
            if Length = 126 then
               declare
                  X : Byte_Array (1 .. 2);
               begin
                  Read_Exactly (C, X, Ok);
                  exit when not Ok;
                  Length := Unsigned_64 (X (1)) * 2 ** 8 + Unsigned_64 (X (2));
               end;
            elsif Length = 127 then
               declare
                  X : Byte_Array (1 .. 8);
               begin
                  Read_Exactly (C, X, Ok);
                  exit when not Ok;
                  Length := 0;
                  for B of X loop
                     Length := Shift_Left (Length, 8) or Unsigned_64 (B);
                  end loop;
               end;
            end if;
            exit when Length > Unsigned_64 (Natural'Last - Data.Length);
            if Masked then
               Read_Exactly (C, Mask, Ok);
               exit when not Ok;
            end if;
            declare
               Payload : Byte_Array_Access := new Byte_Array (1 .. Offset (Length));
            begin
               if Length > 0 then
                  Read_Exactly (C, Payload.all, Ok);
               end if;
               if Ok and then Masked then
                  --  Four bytes a turn against the key's four, the rest one by one:
                  --  an observation is megabytes, and a remainder a byte is slow.
                  declare
                     Whole : constant Offset := Payload'First + (Payload'Length / 4) * 4 - 1;
                     I     : Offset := Payload'First;
                  begin
                     while I <= Whole loop
                        Payload (I) := Payload (I) xor Mask (1);
                        Payload (I + 1) := Payload (I + 1) xor Mask (2);
                        Payload (I + 2) := Payload (I + 2) xor Mask (3);
                        Payload (I + 3) := Payload (I + 3) xor Mask (4);
                        I := I + 4;
                     end loop;
                     for J in Whole + 1 .. Payload'Last loop
                        Payload (J) := Payload (J) xor Mask ((J - 1) mod 4 + 1);
                     end loop;
                  end;
               end if;
               if Ok then
                  case Opcode is
                     when 8 =>
                        Free (Payload);
                        exit;   --  close
                     when 9 =>
                        Send_Frame (C, 10, Payload.all, Ok);   --  answer a ping with a pong
                     when 0 | 1 | 2 =>
                        if Opcode /= 0 then
                           Message := Opcode;
                           Data.Clear;
                        end if;
                        Data.Append (Payload.all);
                        if Fin then
                           Kind := (if Message = 1 then Text else Binary);
                           Free (Payload);
                           return;
                        end if;
                     when others =>
                        null;
                  end case;
               end if;
               Free (Payload);
               exit when not Ok;
            end;
         end;
      end loop;
      Disconnect (C);
      Kind := Closed;
   end Receive;

end Driver.Wire;
