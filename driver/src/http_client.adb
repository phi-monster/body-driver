with GNAT.Sockets; use GNAT.Sockets;
with Ada.Streams; use Ada.Streams;
with Ada.Strings.Fixed;
with Codec;
package body Http_Client is
   --  超时是接线协议(秒),无量纲于身体
   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean is
      S : Socket_Type;
      Addr : Sock_Addr_Type;
      Req : constant String :=
        "POST " & Path & " HTTP/1.1" & ASCII.CR & ASCII.LF &
        "Host: " & Host & ASCII.CR & ASCII.LF &
        "Content-Type: application/json" & ASCII.CR & ASCII.LF &
        "Content-Length: " & Codec.Img (Body_Text'Length) & ASCII.CR & ASCII.LF &
        "Connection: close" & ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF;
      All_Text : Unbounded_String;
   begin
      Reply_Body := Null_Unbounded_String;
      Create_Socket (S);
      Addr.Addr := Addresses (Get_Host_By_Name (Host), 1);
      Addr.Port := Port_Type (Port);
      Set_Socket_Option (S, Socket_Level, (Receive_Timeout, Timeout_S));
      Connect_Socket (S, Addr);
      declare
         Full : constant String := Req & Body_Text;
         A : Stream_Element_Array (1 .. Stream_Element_Offset (Full'Length));
         Last : Stream_Element_Offset;
         Sent : Stream_Element_Offset := 0;
      begin
         for I in Full'Range loop
            A (Stream_Element_Offset (I - Full'First + 1)) := Stream_Element (Character'Pos (Full (I)));
         end loop;
         while Sent < A'Last loop
            Send_Socket (S, A (Sent + 1 .. A'Last), Last);
            exit when Last < Sent + 1;
            Sent := Last;
         end loop;
      end;
      declare
         Chunk : Stream_Element_Array (1 .. 65536);
         Last : Stream_Element_Offset;
      begin
         loop
            Receive_Socket (S, Chunk, Last);
            exit when Last < 1;
            for I in 1 .. Last loop
               Append (All_Text, Character'Val (Chunk (I)));
            end loop;
         end loop;
      exception
         when others => null;
      end;
      Close_Socket (S);
      declare
         T : constant String := To_String (All_Text);
         P : constant Natural := Ada.Strings.Fixed.Index (T, ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF);
      begin
         if P = 0 then
            return False;
         end if;
         Reply_Body := To_Unbounded_String (T (P + 4 .. T'Last));
         return True;
      end;
   exception
      when others =>
         begin
            Close_Socket (S);
         exception
            when others => null;
         end;
         return False;
   end Post;
end Http_Client;
