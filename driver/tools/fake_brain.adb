--  fake_brain PORT [DIR]: a brain service written from docs/brain-service.md
--  alone, to test the wiring between the driver and a brain.
--
--  Without DIR, question one (the streamed request) is answered with
--  server-sent events carrying one say line, and question two (a request
--  with a response format) with found false. The body wired to it runs its
--  rounds and never moves: a brain that wrote motion on its own would be a
--  scripted program.
--
--  With DIR, a person is the brain. Question N is written into DIR as N.ask,
--  its text, and N.bmp, its picture; the answer is read from N.answer once
--  that file is there: the program for question one, the JSON object of the
--  page's section 4 for question two. An answer is written elsewhere and
--  moved in, so that it is never read half written.

with Ada.Command_Line;
with Ada.Directories;
with Ada.Streams;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Maps;
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
   Nothing  : constant String := "{""found"":false,""bbox_2d"":[0,0,0,0]}";
   Poll     : constant Duration := 0.5;   --  how often a person's answer is looked for; a person takes seconds

   Folder : constant String := (if Ada.Command_Line.Argument_Count = 2 then Ada.Command_Line.Argument (2) else "");

   Blanks : constant Ada.Strings.Maps.Character_Set :=
     Ada.Strings.Maps.To_Set (" " & ASCII.HT & ASCII.LF & ASCII.CR);

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

   function Bytes (Text : String) return Ada.Streams.Stream_Element_Array is
      Data : Ada.Streams.Stream_Element_Array (1 .. Ada.Streams.Stream_Element_Offset (Text'Length));
   begin
      for I in Text'Range loop
         Data (Ada.Streams.Stream_Element_Offset (I - Text'First + 1)) := Character'Pos (Text (I));
      end loop;
      return Data;
   end Bytes;

   procedure Send (Client : Socket_Type; Text : String) is
      Data : constant Ada.Streams.Stream_Element_Array := Bytes (Text);
      Last : Ada.Streams.Stream_Element_Offset;
      From : Ada.Streams.Stream_Element_Offset := Data'First;
   begin
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

   --  Text as the inside of a JSON string.
   function Escaped (Text : String) return String is
      Hex    : constant String := "0123456789abcdef";
      Result : Unbounded_String;
   begin
      for C of Text loop
         case C is
            when '"' => Append (Result, "\""");
            when '\' => Append (Result, "\\");
            when ASCII.LF => Append (Result, "\n");
            when ASCII.CR => Append (Result, "\r");
            when ASCII.HT => Append (Result, "\t");
            when ASCII.NUL .. ASCII.BS | ASCII.VT | ASCII.FF | ASCII.SO .. ASCII.US =>
               Append (Result, "\u00" & Hex (Character'Pos (C) / Hex'Length + 1)
                       & Hex (Character'Pos (C) mod Hex'Length + 1));
            when others => Append (Result, C);
         end case;
      end loop;
      return To_String (Result);
   end Escaped;

   --  The JSON string whose opening quote is just before First, its escapes
   --  undone; a \u escape becomes its UTF-8 bytes.
   function Unescaped (Text : String; First : Positive) return String is
      Result : Unbounded_String;
      I      : Positive := First;

      procedure Put_Code (Code : Natural) is
      begin
         if Code < 16#80# then
            Append (Result, Character'Val (Code));
         elsif Code < 16#800# then
            Append (Result, Character'Val (16#C0# + Code / 16#40#));
            Append (Result, Character'Val (16#80# + Code mod 16#40#));
         else
            Append (Result, Character'Val (16#E0# + Code / 16#1000#));
            Append (Result, Character'Val (16#80# + Code / 16#40# mod 16#40#));
            Append (Result, Character'Val (16#80# + Code mod 16#40#));
         end if;
      end Put_Code;
   begin
      while I <= Text'Last and then Text (I) /= '"' loop
         if Text (I) = '\' and then I < Text'Last then
            I := I + 1;
            case Text (I) is
               when 'n' => Append (Result, ASCII.LF);
               when 't' => Append (Result, ASCII.HT);
               when 'r' => Append (Result, ASCII.CR);
               when 'b' => Append (Result, ASCII.BS);
               when 'f' => Append (Result, ASCII.FF);
               when 'u' =>
                  if I + 4 <= Text'Last then
                     Put_Code (Natural'Value ("16#" & Text (I + 1 .. I + 4) & "#"));
                     I := I + 4;
                  end if;
               when others => Append (Result, Text (I));   --  \" \\ \/
            end case;
         else
            Append (Result, Text (I));
         end if;
         I := I + 1;
      end loop;
      return To_String (Result);
   end Unescaped;

   --  The string value of the first member of the request named Key; empty
   --  when there is none.
   function Member (Request, Key : String) return String is
      Quoted : constant String := """" & Key & """";
      From   : Positive := Request'First;
   begin
      loop
         declare
            At_Key : constant Natural := Ada.Strings.Fixed.Index (Request (From .. Request'Last), Quoted);
            I      : Natural := At_Key + Quoted'Length;
         begin
            exit when At_Key = 0;
            while I <= Request'Last and then Ada.Strings.Maps.Is_In (Request (I), Blanks) loop
               I := I + 1;
            end loop;
            if I <= Request'Last and then Request (I) = ':' then
               I := I + 1;
               while I <= Request'Last and then Ada.Strings.Maps.Is_In (Request (I), Blanks) loop
                  I := I + 1;
               end loop;
               if I < Request'Last and then Request (I) = '"' then
                  return Unescaped (Request, I + 1);
               end if;
            end if;
            From := At_Key + 1;
         end;
      end loop;
      return "";
   end Member;

   --  The bytes a base64 text stands for; a character outside the alphabet
   --  (the padding) is skipped.
   function Decoded (Text : String) return Ada.Streams.Stream_Element_Array is
      Sextet_Bits : constant := 6;
      Byte_Bits   : constant := 8;
      Result      : Ada.Streams.Stream_Element_Array
        (1 .. Ada.Streams.Stream_Element_Offset (Text'Length * Sextet_Bits / Byte_Bits));
      Last        : Ada.Streams.Stream_Element_Offset := 0;
      Bits, Value : Natural := 0;   --  Value holds the Bits low bits not yet put out
      Sextet      : Integer;
   begin
      for C of Text loop
         Sextet := (case C is
                      when 'A' .. 'Z' => Character'Pos (C) - Character'Pos ('A'),
                      when 'a' .. 'z' => Character'Pos (C) - Character'Pos ('a') + 26,
                      when '0' .. '9' => Character'Pos (C) - Character'Pos ('0') + 52,
                      when '+'        => 62,
                      when '/'        => 63,
                      when others     => -1);
         if Sextet >= 0 then
            Value := Value * 2 ** Sextet_Bits + Sextet;
            Bits := Bits + Sextet_Bits;
            if Bits >= Byte_Bits then
               Bits := Bits - Byte_Bits;
               Last := Last + 1;
               Result (Last) := Ada.Streams.Stream_Element (Value / 2 ** Bits);
               Value := Value mod 2 ** Bits;
            end if;
         end if;
      end loop;
      return Result (1 .. Last);
   end Decoded;

   procedure Write_File (Path : String; Data : Ada.Streams.Stream_Element_Array) is
      F : Ada.Streams.Stream_IO.File_Type;
   begin
      Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Path);
      Ada.Streams.Stream_IO.Write (F, Data);
      Ada.Streams.Stream_IO.Close (F);
   end Write_File;

   function Read_File (Path : String) return String is
      F    : Ada.Streams.Stream_IO.File_Type;
      Data : Ada.Streams.Stream_Element_Array (1 .. Ada.Streams.Stream_Element_Offset (Ada.Directories.Size (Path)));
      Last : Ada.Streams.Stream_Element_Offset;
   begin
      Ada.Streams.Stream_IO.Open (F, Ada.Streams.Stream_IO.In_File, Path);
      Ada.Streams.Stream_IO.Read (F, Data, Last);
      Ada.Streams.Stream_IO.Close (F);
      return Text : String (1 .. Natural (Last)) do
         for I in Text'Range loop
            Text (I) := Character'Val (Data (Ada.Streams.Stream_Element_Offset (I)));
         end loop;
      end return;
   end Read_File;

   Asked : Natural := 0;   --  questions put to the person so far

   --  The person's answer to a request: the question goes into Folder, and
   --  the answer is waited for there.
   function Person (Request : String; Program : Boolean) return String is
      Number : constant String := Ada.Strings.Fixed.Trim (Natural'Image (Asked), Ada.Strings.Left);
      Base   : constant String := Ada.Directories.Compose (Folder, Number);
      Url    : constant String := Member (Request, "url");
   begin
      Write_File (Base & ".bmp", Decoded (Url (Ada.Strings.Fixed.Index (Url & ",", ",") + 1 .. Url'Last)));
      Write_File (Base & ".ask", Bytes ((if Program then "write a program" else "where is it") & LF & LF
                                        & Member (Request, "text") & LF));
      Ada.Text_IO.Put_Line ("question " & Number & (if Program then ": write a program" else ": where is it")
                            & ", in " & Base & ".ask and " & Base & ".bmp; waiting for " & Base & ".answer");
      while not Ada.Directories.Exists (Base & ".answer") loop
         delay Poll;
      end loop;
      return Read_File (Base & ".answer");
   end Person;

   function Event (Data : String) return String is ("data: " & Data & LF & LF);

   procedure Answer_Program (Client : Socket_Type; Program : String) is
   begin
      Send (Client, "HTTP/1.1 200 OK" & CRLF & "Content-Type: text/event-stream" & CRLF & "Connection: close" & CRLF
            & CRLF);
      Send (Client, Event ("{""choices"":[{""index"":0,""delta"":{""content"":""" & Escaped (Program) & """}}]}"));
      Send (Client, Event ("{""choices"":[{""index"":0,""delta"":{},""finish_reason"":""stop""}]}"));
      Send (Client, Event ("[DONE]"));
   end Answer_Program;

   procedure Answer_Where (Client : Socket_Type; Box : String) is
      Reply : constant String :=
        "{""choices"":[{""index"":0,""message"":{""role"":""assistant"",""content"":""" & Escaped (Box)
        & """},""finish_reason"":""stop""}]}";
   begin
      Send (Client, "HTTP/1.1 200 OK" & CRLF & "Content-Type: application/json" & CRLF & "Content-Length:"
            & Natural'Image (Reply'Length) & CRLF & "Connection: close" & CRLF & CRLF & Reply);
   end Answer_Where;

   Server  : Socket_Type;
   Client  : Socket_Type;
   Address : Sock_Addr_Type;
begin
   if Ada.Command_Line.Argument_Count not in 1 .. 2 then
      Ada.Text_IO.Put_Line ("usage: fake_brain PORT [DIR]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   if Folder'Length > 0 then
      if not Ada.Directories.Exists (Folder) then
         Ada.Directories.Create_Path (Folder);
      end if;
      declare
         Search : Ada.Directories.Search_Type;
      begin
         Ada.Directories.Start_Search (Search, Folder, "", [Ada.Directories.Ordinary_File => True, others => False]);
         if Ada.Directories.More_Entries (Search) then
            Ada.Text_IO.Put_Line ("fake_brain: " & Folder & " is not empty, and an old answer in it would be read"
                                  & " as a new one");
            Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
            return;
         end if;
         Ada.Directories.End_Search (Search);
      end;
   end if;
   Create_Socket (Server);
   Set_Socket_Option (Server, Socket_Level, (Reuse_Address, True));
   Bind_Socket (Server, (Family_Inet, Any_Inet_Addr, Port_Type'Value (Ada.Command_Line.Argument (1))));
   Listen_Socket (Server);
   Ada.Text_IO.Put_Line ("fake brain listening on port " & Ada.Command_Line.Argument (1)
                         & (if Folder'Length > 0 then ", a person answering in " & Folder else ""));
   loop
      Accept_Socket (Server, Client, Address);
      declare
         Request : constant String := Read_Request (Client);
         Program : constant Boolean := Streamed (Request);
      begin
         if Folder'Length > 0 then
            Asked := Asked + 1;
            declare
               Said : constant String := Person (Request, Program);
            begin
               if Program then
                  Answer_Program (Client, (if Said'Length > 0 and then Said (Said'Last) = ASCII.LF then Said
                                           else Said & LF));
               else
                  Answer_Where (Client, Ada.Strings.Fixed.Trim (Said, Blanks, Blanks));
               end if;
               Ada.Text_IO.Put_Line ("question" & Asked'Image & " answered");
            end;
         elsif Program then
            Answer_Program (Client, Sentence & LF);
            Ada.Text_IO.Put_Line ("write a program: " & Sentence);
         else
            Answer_Where (Client, Nothing);
            Ada.Text_IO.Put_Line ("where is it: found false");
         end if;
      exception
         when Socket_Error =>
            Ada.Text_IO.Put_Line ("the driver closed the connection");
      end;
      Close_Socket (Client);
   end loop;
end Fake_Brain;
