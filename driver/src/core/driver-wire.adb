with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with GNAT.SHA1;
with Interfaces;

package body Driver.Wire is

   use Driver.Bytes;
   use GNAT.Sockets;
   use Interfaces;
   use type Offset;
   use type Byte;

   Accept_Suffix : constant String := "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
   --  RFC 6455 4.2.2: appended to the client's key before hashing.

   type Byte_Array_Access is access Byte_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Byte_Array, Byte_Array_Access);

   procedure Read_Exactly (C : Connection; Item : out Byte_Array; Ok : out Boolean) is
      Got  : Offset := Item'First - 1;
      Last : Offset;
   begin
      Ok := False;
      while Got < Item'Last loop
         Receive_Socket (C.Client, Item (Got + 1 .. Item'Last), Last);
         if Last <= Got then
            return;   --  the client closed the connection
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
         Send_Socket (C.Client, Item (Sent + 1 .. Item'Last), Last);
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

   function Base64 (Data : Byte_Array) return String is
      Alphabet : constant String := "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
      R : Ada.Strings.Unbounded.Unbounded_String;
      I : Offset := Data'First;
   begin
      while I <= Data'Last loop
         declare
            B0 : constant Natural := Natural (Data (I));
            B1 : constant Natural := (if I + 1 <= Data'Last then Natural (Data (I + 1)) else 0);
            B2 : constant Natural := (if I + 2 <= Data'Last then Natural (Data (I + 2)) else 0);
            Group : constant Natural := B0 * 2 ** 16 + B1 * 2 ** 8 + B2;
         begin
            Ada.Strings.Unbounded.Append (R, Alphabet (Group / 2 ** 18 + 1));
            Ada.Strings.Unbounded.Append (R, Alphabet ((Group / 2 ** 12) mod 2 ** 6 + 1));
            Ada.Strings.Unbounded.Append
              (R, (if I + 1 <= Data'Last then Alphabet ((Group / 2 ** 6) mod 2 ** 6 + 1) else '='));
            Ada.Strings.Unbounded.Append (R, (if I + 2 <= Data'Last then Alphabet (Group mod 2 ** 6 + 1) else '='));
         end;
         I := I + 3;
      end loop;
      return Ada.Strings.Unbounded.To_String (R);
   end Base64;

   function Digest_Bytes (Hex : String) return Byte_Array is
      R : Byte_Array (1 .. Offset (Hex'Length / 2));
   begin
      for I in R'Range loop
         R (I) := Byte'Value ("16#" & Hex (Hex'First + 2 * Natural (I - 1) .. Hex'First + 2 * Natural (I - 1) + 1) & "#");
      end loop;
      return R;
   end Digest_Bytes;

   procedure Handshake (C : Connection; Ok : out Boolean) is
      Head  : Ada.Strings.Unbounded.Unbounded_String;
      One   : Byte_Array (1 .. 1);
      Blank : constant String := ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF;
      Tag   : constant String := "sec-websocket-key:";
   begin
      loop
         Read_Exactly (C, One, Ok);
         if not Ok then
            return;
         end if;
         Ada.Strings.Unbounded.Append (Head, Character'Val (One (1)));
         exit when Ada.Strings.Unbounded.Length (Head) >= Blank'Length
           and then Ada.Strings.Unbounded.Slice
                      (Head, Ada.Strings.Unbounded.Length (Head) - Blank'Length + 1,
                       Ada.Strings.Unbounded.Length (Head)) = Blank;
      end loop;
      declare
         H     : constant String := Ada.Strings.Unbounded.To_String (Head);
         Lower : String := H;
      begin
         for Ch of Lower loop
            if Ch in 'A' .. 'Z' then
               Ch := Character'Val (Character'Pos (Ch) + 32);
            end if;
         end loop;
         declare
            P : constant Natural := Ada.Strings.Fixed.Index (Lower, Tag);
            E : constant Natural := (if P = 0 then 0 else Ada.Strings.Fixed.Index (H, "" & ASCII.CR, P));
         begin
            Ok := P > 0 and then E > 0;
            if not Ok then
               return;
            end if;
            declare
               Key      : constant String := Ada.Strings.Fixed.Trim (H (P + Tag'Length .. E - 1), Ada.Strings.Both);
               Answer   : constant String := Base64 (Digest_Bytes (GNAT.SHA1.Digest (Key & Accept_Suffix)));
               Response : constant String :=
                 "HTTP/1.1 101 Switching Protocols" & ASCII.CR & ASCII.LF
                 & "Upgrade: websocket" & ASCII.CR & ASCII.LF
                 & "Connection: Upgrade" & ASCII.CR & ASCII.LF
                 & "Sec-WebSocket-Accept: " & Answer & Blank;
            begin
               Send_All (C, To_Bytes (Response), Ok);
            end;
         end;
      end;
   end Handshake;

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
      if C.Open then
         Close_Socket (C.Client);
         C.Open := False;
      end if;
      Accept_Socket (C.Listener, C.Client, Address);
      C.Open := True;
      Handshake (C, Ok);
      if not Ok then
         Close_Socket (C.Client);
         C.Open := False;
      end if;
   exception
      when Socket_Error =>
         Ok := False;
   end Accept_Client;

   procedure Send_Frame (C : Connection; Opcode : Byte; Data : Byte_Array; Ok : out Boolean) is
      Header : Buffer;
      L      : constant Unsigned_64 := Unsigned_64 (Data'Length);
   begin
      Header.Append (16#80# or Opcode);   --  FIN and the opcode
      if L < 126 then
         Header.Append (Byte (L));
      elsif L < 2 ** 16 then
         Header.Append (Byte'(126));
         Header.Append (Byte (Shift_Right (L, 8) and 16#FF#));
         Header.Append (Byte (L and 16#FF#));
      else
         Header.Append (Byte'(127));
         for K in reverse 0 .. 7 loop
            Header.Append (Byte (Shift_Right (L, 8 * K) and 16#FF#));
         end loop;
      end if;
      Send_All (C, Header.To_Array, Ok);
      if Ok then
         Send_All (C, Data, Ok);
      end if;
   end Send_Frame;

   procedure Send (C : in out Connection; Data : Byte_Array; Ok : out Boolean) is
   begin
      Ok := C.Open;
      if Ok then
         Send_Frame (C, 2, Data, Ok);
      end if;
   end Send;

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
                  for I in Payload'Range loop
                     Payload (I) := Payload (I) xor Mask ((I - 1) mod 4 + 1);
                  end loop;
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
      Close_Socket (C.Client);
      C.Open := False;
      Kind := Closed;
   end Receive;

end Driver.Wire;
