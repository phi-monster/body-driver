with GNAT.Sockets; use GNAT.Sockets;
with Ada.Streams; use Ada.Streams;
with Codec;
package body Http_Client is
   --  一段一段地发(每段 64 KB,协议上的分块大小):整份请求不在栈上摆成一个大数组
   Chunk_Len : constant := 65536;

   procedure Send_Text (S : Socket_Type; T : String) is
      First : Natural := T'First;
   begin
      while First <= T'Last loop
         declare
            Last_Ch : constant Natural := Natural'Min (T'Last, First + Chunk_Len - 1);
            A : Stream_Element_Array (1 .. Stream_Element_Offset (Last_Ch - First + 1));
            Sent : Stream_Element_Offset := 0;
            Last : Stream_Element_Offset;
         begin
            for I in First .. Last_Ch loop
               A (Stream_Element_Offset (I - First + 1)) := Stream_Element (Character'Pos (T (I)));
            end loop;
            while Sent < A'Last loop
               Send_Socket (S, A (Sent + 1 .. A'Last), Last);
               if Last < Sent + 1 then
                  raise Socket_Error;   --  对端不收了
               end if;
               Sent := Last;
            end loop;
            First := Last_Ch + 1;
         end;
      end loop;
   end Send_Text;

   --  连上、发请求头、交给 Send_Body 发请求体、读回应答体
   function Exchange (Host : String; Port : Natural; Path : String; Body_Len : Natural;
                      Send_Body : access procedure (S : Socket_Type);
                      Reply_Body : out Unbounded_String; Timeout_S : Duration) return Boolean is
      S : Socket_Type;
      Addr : Sock_Addr_Type;
      Req : constant String :=
        "POST " & Path & " HTTP/1.1" & ASCII.CR & ASCII.LF &
        "Host: " & Host & ASCII.CR & ASCII.LF &
        "Content-Type: application/json" & ASCII.CR & ASCII.LF &
        "Content-Length: " & Codec.Img (Body_Len) & ASCII.CR & ASCII.LF &
        "Connection: close" & ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF;
      All_Text : Unbounded_String;
   begin
      Reply_Body := Null_Unbounded_String;
      Create_Socket (S);
      Addr.Addr := Addresses (Get_Host_By_Name (Host), 1);
      Addr.Port := Port_Type (Port);
      Set_Socket_Option (S, Socket_Level, (Receive_Timeout, Timeout_S));
      Connect_Socket (S, Addr);
      Send_Text (S, Req);
      Send_Body (S);
      declare
         Chunk : Stream_Element_Array (1 .. Chunk_Len);
         Last : Stream_Element_Offset;
      begin
         loop
            Receive_Socket (S, Chunk, Last);
            exit when Last < 1;
            declare
               Piece : String (1 .. Natural (Last));
            begin
               for I in 1 .. Last loop
                  Piece (Natural (I)) := Character'Val (Chunk (I));
               end loop;
               Append (All_Text, Piece);
            end;
         end loop;
      exception
         when others => null;
      end;
      Close_Socket (S);
      declare
         P : constant Natural := Index (All_Text, ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF);
      begin
         if P = 0 then
            return False;
         end if;
         Reply_Body := Unbounded_Slice (All_Text, P + 4, Length (All_Text));
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
   end Exchange;

   --  超时是接线协议(秒),无量纲于身体(同规格里那一行)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean is
      procedure Send_Body (S : Socket_Type) is
      begin
         Send_Text (S, Body_Text);
      end Send_Body;
   begin
      return Exchange (Host, Port, Path, Body_Text'Length, Send_Body'Access, Reply_Body, Timeout_S);
   end Post;

   --  超时是接线协议(秒),无量纲于身体(同规格里那一行)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : Unbounded_String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean is
      procedure Send_Body (S : Socket_Type) is
         N : constant Natural := Length (Body_Text);
         First : Natural := 1;
      begin
         while First <= N loop
            declare
               Last_Ch : constant Natural := Natural'Min (N, First + Chunk_Len - 1);
            begin
               Send_Text (S, Slice (Body_Text, First, Last_Ch));
               First := Last_Ch + 1;
            end;
         end loop;
      end Send_Body;
   begin
      return Exchange (Host, Port, Path, Length (Body_Text), Send_Body'Access, Reply_Body, Timeout_S);
   end Post;
end Http_Client;
