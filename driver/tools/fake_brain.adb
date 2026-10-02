--  fake_brain PORT: a brain service written from docs/brain-service.md
--  alone, to test the wiring between the driver and a brain.
--
--  Question one (the streamed request) is answered with server-sent events
--  carrying one say line; question two (a request with a response format)
--  with found false. The body wired to it runs its rounds and never moves:
--  a brain that wrote motion on its own would be a scripted program.

with Ada.Command_Line;
with Ada.Streams;
with Ada.Strings.Fixed;
with Ada.Strings.Maps.Constants;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with GNAT.Sockets;

procedure Fake_Brain is

   use Ada.Strings.Unbounded;
   use GNAT.Sockets;
   use type Ada.Streams.Stream_Element_Offset;

   CRLF : constant String := [ASCII.CR, ASCII.LF];
   LF   : constant String := [ASCII.LF];

   Sentence : constant String := "say I am a fake brain: I only speak";

   --  Question one is the streamed one: its request sets "stream" to true.
   function Streamed (Request : String) return Boolean is
      Key : constant String := """stream""";
      At_Key : constant Natural := Ada.Strings.Fixed.Index (Request, Key);
      I      : Natural := At_Key + Key'Length;
   begin
      if At_Key = 0 then
         return False;
      end if;
      while I <= Request'Last and then Request (I) in ' ' | ':' | ASCII.HT | ASCII.LF | ASCII.CR loop
         I := I + 1;
      end loop;
      return I + 3 <= Request'Last and then Request (I .. I + 3) = "true";
   end Streamed;

   procedure Send (Client : Socket_Type; Text : String) is
      Data : Ada.Streams.Stream_Element_Array (1 .. Ada.Streams.Stream_Element_Offset (Text'Length));
      Last : Ada.Streams.Stream_Element_Offset;
      From : Ada.Streams.Stream_Element_Offset := Data'First;
   begin
      for I in Text'Range loop
         Data (Ada.Streams.Stream_Element_Offset (I - Text'First + 1)) := Character'Pos (Text (I));
      end loop;
      while From <= Data'Last loop
         Send_Socket (Client, Data (From .. Data'Last), Last);
         From := Last + 1;
      end loop;
   end Send;

   --  The request: its head up to the blank line, then as many body bytes
   --  as Content-Length says.
   function Read_Request (Client : Socket_Type) return String is
      Got    : Unbounded_String;
      Buffer : Ada.Streams.Stream_Element_Array (1 .. 4096);
      Last   : Ada.Streams.Stream_Element_Offset;
      Head_End, Body_Length : Natural := 0;
   begin
      loop
         Receive_Socket (Client, Buffer, Last);
         exit when Last < Buffer'First;
         for I in Buffer'First .. Last loop
            Append (Got, Character'Val (Buffer (I)));
         end loop;
         if Head_End = 0 then
            Head_End := Ada.Strings.Fixed.Index (To_String (Got), CRLF & CRLF);
            if Head_End > 0 then
               declare
                  Head  : constant String := Ada.Strings.Fixed.Translate
                    (Slice (Got, 1, Head_End), Ada.Strings.Maps.Constants.Lower_Case_Map);
                  Field : constant String := "content-length:";
                  At_F  : constant Natural := Ada.Strings.Fixed.Index (Head, Field);
               begin
                  if At_F > 0 then
                     declare
                        Stop : Natural := At_F + Field'Length;
                     begin
                        while Stop <= Head'Last and then Head (Stop) /= ASCII.CR loop
                           Stop := Stop + 1;
                        end loop;
                        Body_Length := Natural'Value (Head (At_F + Field'Length .. Stop - 1));
                     end;
                  end if;
               end;
            end if;
         end if;
         exit when Head_End > 0 and then Length (Got) >= Head_End + CRLF'Length * 2 - 1 + Body_Length;
      end loop;
      return To_String (Got);
   end Read_Request;

   function Event (Data : String) return String is ("data: " & Data & LF & LF);

   procedure Answer_Program (Client : Socket_Type) is
   begin
      Send (Client, "HTTP/1.1 200 OK" & CRLF & "Content-Type: text/event-stream" & CRLF & "Connection: close" & CRLF
            & CRLF);
      Send (Client, Event ("{""choices"":[{""index"":0,""delta"":{""content"":""" & Sentence & "\n""}}]}"));
      Send (Client, Event ("{""choices"":[{""index"":0,""delta"":{},""finish_reason"":""stop""}]}"));
      Send (Client, Event ("[DONE]"));
   end Answer_Program;

   procedure Answer_Where (Client : Socket_Type) is
      Reply : constant String :=
        "{""choices"":[{""index"":0,""message"":{""role"":""assistant"",""content"":"
        & """{\""found\"":false,\""bbox_2d\"":[0,0,0,0]}""},""finish_reason"":""stop""}]}";
   begin
      Send (Client, "HTTP/1.1 200 OK" & CRLF & "Content-Type: application/json" & CRLF & "Content-Length:"
            & Natural'Image (Reply'Length) & CRLF & "Connection: close" & CRLF & CRLF & Reply);
   end Answer_Where;

   Server  : Socket_Type;
   Client  : Socket_Type;
   Address : Sock_Addr_Type;
begin
   if Ada.Command_Line.Argument_Count /= 1 then
      Ada.Text_IO.Put_Line ("usage: fake_brain PORT");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Create_Socket (Server);
   Set_Socket_Option (Server, Socket_Level, (Reuse_Address, True));
   Bind_Socket (Server, (Family_Inet, Any_Inet_Addr, Port_Type'Value (Ada.Command_Line.Argument (1))));
   Listen_Socket (Server);
   Ada.Text_IO.Put_Line ("fake brain listening on port " & Ada.Command_Line.Argument (1));
   loop
      Accept_Socket (Server, Client, Address);
      declare
         Request : constant String := Read_Request (Client);
      begin
         if Streamed (Request) then
            Answer_Program (Client);
            Ada.Text_IO.Put_Line ("write a program: " & Sentence);
         else
            Answer_Where (Client);
            Ada.Text_IO.Put_Line ("where is it: found false");
         end if;
      exception
         when Socket_Error =>
            Ada.Text_IO.Put_Line ("the driver closed the connection");
      end;
      Close_Socket (Client);
   end loop;
end Fake_Brain;
