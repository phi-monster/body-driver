with Ada.Characters.Handling;
with Ada.Exceptions;
with Ada.Streams;
with Ada.Strings.Fixed;
with Ada.Unchecked_Deallocation;
with GNAT.Sockets;

package body Driver.Http is

   use Ada.Streams;
   use GNAT.Sockets;

   type Bytes_Access is access Stream_Element_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Stream_Element_Array, Bytes_Access);
   --  Socket buffers can be megabytes; they live on the heap, not on a task's stack.

   CRLF  : constant String := ASCII.CR & ASCII.LF;
   Blank : constant String := CRLF & CRLF;

   procedure Ignore (Data : String; Stop : out Boolean) is
      pragma Unreferenced (Data);
   begin
      Stop := False;
   end Ignore;

   procedure Send_Text (S : Socket_Type; Text : String) is
      Piece : constant Natural := Natural'Max (1, Get_Socket_Option (S, Socket_Level, Send_Buffer).Size);
      Data  : Bytes_Access := new Stream_Element_Array (1 .. Stream_Element_Offset (Piece));
      First : Natural := Text'First;
   begin
      while First <= Text'Last loop
         declare
            Last : constant Natural := Natural'Min (Text'Last, First + Piece - 1);
            Size : constant Stream_Element_Offset := Stream_Element_Offset (Last - First + 1);
            Sent : Stream_Element_Offset := 0;
            Upto : Stream_Element_Offset;
         begin
            for I in 1 .. Size loop
               Data (I) := Character'Pos (Text (First + Natural (I) - 1));
            end loop;
            while Sent < Size loop
               Send_Socket (S, Data (Sent + 1 .. Size), Upto);
               if Upto <= Sent then
                  raise Socket_Error with "the server stopped accepting data";
               end if;
               Sent := Upto;
            end loop;
            First := Last + 1;
         end;
      end loop;
      Free (Data);
   exception
      when others =>
         Free (Data);
         raise;
   end Send_Text;

   --  The value of a header, case-insensitive; empty when absent.
   function Header (Head, Name : String) return String is
      Lower : constant String := Ada.Characters.Handling.To_Lower (Name);
      I     : Natural := Head'First;
   begin
      while I <= Head'Last loop
         declare
            J    : constant Natural := Ada.Strings.Fixed.Index (Head (I .. Head'Last), CRLF);
            Line : constant String := Head (I .. (if J = 0 then Head'Last else J - 1));
            C    : constant Natural := Ada.Strings.Fixed.Index (Line, ":");
         begin
            if C > 0 and then Ada.Characters.Handling.To_Lower
                                (Ada.Strings.Fixed.Trim (Line (Line'First .. C - 1), Ada.Strings.Both)) = Lower
            then
               return Ada.Strings.Fixed.Trim (Line (C + 1 .. Line'Last), Ada.Strings.Both);
            end if;
            I := (if J = 0 then Head'Last + 1 else J + CRLF'Length);
         end;
      end loop;
      return "";
   end Header;

   function Exchange
     (Host      : String;
      Port      : Natural;
      Path      : String;
      Body_Text : String;
      On_Data   : not null access procedure (Data : String; Stop : out Boolean)) return Response
   is
      R       : Response;
      S       : Socket_Type := No_Socket;
      Opened  : Boolean := False;
      Where   : constant String := Host & ":" & Ada.Strings.Fixed.Trim (Natural'Image (Port), Ada.Strings.Both) & Path;
      Pending : Unbounded_String;   --  received, not yet parsed
      Head_Done, Stopped, Finished : Boolean := False;
      Chunked : Boolean := False;
      Remaining : Integer := -1;    --  body bytes still expected; -1 until the server closes
      Chunk_Left : Natural := 0;    --  bytes left in the current chunk
      In_Chunk : Boolean := False;

      procedure Deliver (Data : String) is
      begin
         if Data'Length = 0 or else Stopped then
            return;
         end if;
         Append (R.Body_Text, Data);
         On_Data (Data, Stopped);
      end Deliver;

      --  Consumes what can be consumed of Pending.
      procedure Digest is
      begin
         if not Head_Done then
            declare
               P : constant Natural := Index (Pending, Blank);
            begin
               if P = 0 then
                  return;
               end if;
               declare
                  Head   : constant String := Slice (Pending, 1, P - 1);
                  E      : constant Natural := Ada.Strings.Fixed.Index (Head, CRLF);
                  Status : constant String := (if E = 0 then Head else Head (Head'First .. E - 1));
                  Sp     : constant Natural := Ada.Strings.Fixed.Index (Status, " ");
               begin
                  R.Status := Natural'Value (Status (Sp + 1 .. Natural'Min (Status'Last, Sp + 3)));
                  Chunked := Ada.Strings.Fixed.Index
                    (Ada.Characters.Handling.To_Lower (Header (Head, "Transfer-Encoding")), "chunked") > 0;
                  if not Chunked and then Header (Head, "Content-Length") /= "" then
                     Remaining := Natural'Value (Header (Head, "Content-Length"));
                  end if;
               exception
                  when Constraint_Error =>
                     R.Why := To_Unbounded_String ("the reply from " & Where & " is not HTTP: " & Status);
                     Finished := True;
                     return;
               end;
               Delete (Pending, 1, P + Blank'Length - 1);
               Head_Done := True;
            end;
         end if;
         if not Chunked then
            if Remaining >= 0 then
               declare
                  Take : constant Natural := Natural'Min (Remaining, Length (Pending));
               begin
                  Deliver (Slice (Pending, 1, Take));
                  Delete (Pending, 1, Take);
                  Remaining := Remaining - Take;
                  Finished := Remaining = 0;
               end;
            else
               Deliver (To_String (Pending));
               Pending := Null_Unbounded_String;
            end if;
            return;
         end if;
         --  Chunked: size line CRLF, data, CRLF; a zero size ends the body.
         loop
            exit when Stopped or else Finished;
            if In_Chunk then
               declare
                  Take : constant Natural := Natural'Min (Chunk_Left, Length (Pending));
               begin
                  Deliver (Slice (Pending, 1, Take));
                  Delete (Pending, 1, Take);
                  Chunk_Left := Chunk_Left - Take;
                  exit when Chunk_Left > 0;
                  exit when Length (Pending) < CRLF'Length;
                  Delete (Pending, 1, CRLF'Length);
                  In_Chunk := False;
               end;
            else
               declare
                  E : constant Natural := Index (Pending, CRLF);
               begin
                  exit when E = 0;
                  declare
                     Line : constant String := Slice (Pending, 1, E - 1);
                     Semi : constant Natural := Ada.Strings.Fixed.Index (Line, ";");
                     Size : constant String :=
                       Ada.Strings.Fixed.Trim ((if Semi = 0 then Line else Line (Line'First .. Semi - 1)),
                                               Ada.Strings.Both);
                  begin
                     Chunk_Left := Natural'Value ("16#" & Size & "#");
                     Delete (Pending, 1, E + CRLF'Length - 1);
                     if Chunk_Left = 0 then
                        Finished := True;
                     else
                        In_Chunk := True;
                     end if;
                  exception
                     when Constraint_Error =>
                        R.Why := To_Unbounded_String ("unreadable chunk size from " & Where & ": " & Line);
                        Finished := True;
                  end;
               end;
            end if;
         end loop;
      end Digest;

      Request : constant String :=
        "POST " & Path & " HTTP/1.1" & CRLF
        & "Host: " & Host & CRLF
        & "Content-Type: application/json" & CRLF
        & "Content-Length: " & Ada.Strings.Fixed.Trim (Natural'Image (Body_Text'Length), Ada.Strings.Both) & CRLF
        & "Connection: close" & Blank;
   begin
      Create_Socket (S);
      Opened := True;
      --  The body goes out right after the headers, not after the service has
      --  acknowledged them (Nagle's algorithm against a delayed acknowledgement: 40
      --  ms a call; Driver.Wire.Accept_Client).
      Set_Socket_Option (S, IP_Protocol_For_TCP_Level, (No_Delay, True));
      Connect_Socket (S, (Family => Family_Inet, Addr => Addresses (Get_Host_By_Name (Host), 1),
                          Port => Port_Type (Port)));
      Send_Text (S, Request);
      Send_Text (S, Body_Text);
      declare
         Data : Bytes_Access := new Stream_Element_Array
           (1 .. Stream_Element_Offset (Natural'Max (1, Get_Socket_Option (S, Socket_Level, Receive_Buffer).Size)));
         Last : Stream_Element_Offset;
      begin
         loop
            exit when Stopped or else Finished;
            Receive_Socket (S, Data.all, Last);
            exit when Last < Data'First;   --  the server closed the connection
            for I in 1 .. Last loop
               Append (Pending, Character'Val (Data (I)));
            end loop;
            Digest;
         end loop;
         Free (Data);
      exception
         when others =>
            Free (Data);
            raise;
      end;
      Close_Socket (S);
      Opened := False;
      if Length (R.Why) > 0 then
         return R;
      elsif not Head_Done then
         R.Why := To_Unbounded_String ("the connection to " & Where & " closed before a complete reply");
      elsif not Stopped and then ((Remaining > 0) or else (Chunked and then not Finished)) then
         R.Why := To_Unbounded_String ("the reply from " & Where & " was cut short");
      elsif R.Status not in 200 .. 299 then
         R.Why := To_Unbounded_String (Where & " answered with status" & Natural'Image (R.Status));
      else
         R.Ok := True;
      end if;
      return R;
   exception
      when E : others =>
         if Opened then
            begin
               Close_Socket (S);
            exception
               when others => null;
            end;
         end if;
         R.Ok := False;
         R.Why := To_Unbounded_String ("talking to " & Where & ": " & Ada.Exceptions.Exception_Message (E));
         return R;
   end Exchange;

   function Post (Host : String; Port : Natural; Path : String; Body_Text : String) return Response is
     (Exchange (Host, Port, Path, Body_Text, Ignore'Access));

   function Post_Streaming
     (Host      : String;
      Port      : Natural;
      Path      : String;
      Body_Text : String;
      On_Data   : not null access procedure (Data : String; Stop : out Boolean)) return Response is
     (Exchange (Host, Port, Path, Body_Text, On_Data));

end Driver.Http;
