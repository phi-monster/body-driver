--  WebSocket 服务端(RFC 6455 最小子集):握手、读帧(拆掩码、答 ping)、写二进制帧、重新接受。
--  零依赖:SHA-1 用 GNAT.SHA1,套接字用 GNAT.Sockets。
with Bytes; use Bytes;
with GNAT.Sockets;
package Websocket is
   type Conn is record
      Listener : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Sock : GNAT.Sockets.Socket_Type := GNAT.Sockets.No_Socket;
      Open : Boolean := False;
      Listening : Boolean := False;
   end record;
   type Op is (Op_Binary, Op_Text, Op_Close, Op_None);

   procedure Listen (Port : Natural; C : in out Conn; Ok : out Boolean);
   procedure Accept_Client (C : in out Conn; Ok : out Boolean);
   procedure Read_Message (C : in out Conn; Kind : out Op; Data : out Buf; Ok : out Boolean);
   procedure Send_Binary (C : in out Conn; Data : Buf; Ok : out Boolean);
   procedure Close (C : in out Conn);
end Websocket;
