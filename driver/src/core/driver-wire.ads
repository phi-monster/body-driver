--  The WebSocket server the robot connects to (RFC 6455, the subset the
--  protocol needs): one client at a time, binary and text messages, pings
--  answered, and a new client accepted on the same port after a disconnect.

with GNAT.Sockets;
with Driver.Bytes;

package Driver.Wire is

   type Connection is limited private;

   procedure Listen (C : in out Connection; Port : Natural; Ok : out Boolean);
   --  Binds the port; does not wait for a client.

   procedure Accept_Client (C : in out Connection; Ok : out Boolean);
   --  Waits for the next client and completes the handshake.

   type Message_Kind is (Binary, Text, Closed);

   procedure Receive (C : in out Connection; Kind : out Message_Kind; Data : in out Driver.Bytes.Buffer);
   --  The next complete message; Closed when the client went away.

   procedure Send (C : in out Connection; Data : Driver.Bytes.Byte_Array; Ok : out Boolean);
   --  One binary message.

private

   type Connection is limited record
      Listener : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Client   : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Open     : Boolean := False;
   end record;

end Driver.Wire;
