--  WebSocket connections (RFC 6455, the subset the protocol needs): the
--  server end the robot connects to (one client at a time, and a new client
--  accepted on the same port after a disconnect), and the client end the
--  wire tools use to reach a driver. Binary and text messages; pings are
--  answered.
--
--  One task may receive on a connection while another sends on it: sends,
--  including the pongs Receive answers with, are serialized, and either task
--  can end the connection for both.

with Ada.Numerics.Discrete_Random;
with GNAT.Sockets;
with Driver.Bytes;

package Driver.Wire is

   type Connection is limited private;

   procedure Listen (C : in out Connection; Port : Natural; Ok : out Boolean);
   --  Binds the port; does not wait for a client.

   procedure Accept_Client (C : in out Connection; Ok : out Boolean);
   --  Waits for the next client and completes the handshake.

   procedure Connect (C : in out Connection; Host : String; Port : Natural; Ok : out Boolean);
   --  Opens a connection to a WebSocket server as its client. The frames this
   --  end sends are masked, as the RFC requires of a client.

   type Message_Kind is (Binary, Text, Closed);

   procedure Receive (C : in out Connection; Kind : out Message_Kind; Data : in out Driver.Bytes.Buffer);
   --  The next complete message; Closed when the other end went away.

   procedure Send
     (C : in out Connection; Data : Driver.Bytes.Byte_Array; Ok : out Boolean; Kind : Message_Kind := Binary)
     with Pre => Kind /= Closed;
   --  One message.

   procedure Disconnect (C : in out Connection);
   --  Ends the connection; a Receive waiting on it returns Closed.

private

   package Mask_Keys is new Ada.Numerics.Discrete_Random (Driver.Bytes.Byte);

   --  Socket output may block, so it happens outside the protected action:
   --  the protected object is only the lock.
   protected type Send_Lock is
      entry Seize;
      procedure Release;
   private
      Busy : Boolean := False;
   end Send_Lock;

   type Connection is limited record
      Listener : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Peer     : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Open     : Boolean := False with Atomic;
      Client   : Boolean := False;   --  this end is the client: its frames are masked
      Sending  : Send_Lock;
      Keys     : Mask_Keys.Generator;
   end record;

end Driver.Wire;
