--  feed RECORDING DRIVER_HOST DRIVER_PORT [MESSAGES]
--
--  Plays the robot's side of a recording (Driver.Recording format,
--  uncompressed) to a running driver: every robot message is sent unchanged,
--  one at a time, and the driver's reply awaited before the next, as the
--  robot waits. Each reply is checked against the protocol
--  (docs/body-protocol.md 4): its type answers the request, it echoes the
--  request's identifiers and step, and every get_action reply carries one
--  action map with the command keys of the recorded driver's actions and the
--  same number of values for each. Stops after MESSAGES robot messages when
--  given. Exits with failure when any reply breaks the protocol.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Bytes;
with Driver.Msgpack;
with Driver.Protocol;
with Driver.Recording;
with Driver.Wire;

procedure Feed is

   use Ada.Command_Line;
   use Ada.Strings.Unbounded;
   use Ada.Text_IO;
   use Driver.Msgpack;
   use type Driver.Recording.Record_Kind;
   use type Driver.Wire.Message_Kind;
   use type Driver.Bytes.Byte_Array;

   package Key_Counts is new Ada.Containers.Indefinite_Ordered_Maps (String, Natural);
   use type Key_Counts.Map;

   function Value_Count (Doc : Document; N : Node) return Natural is
     (if Is_Numeric (Doc, N) then Numbers (Doc, N)'Length else Driver.Msgpack.Count (Doc, N));

   function Action_Keys (Doc : Document) return Key_Counts.Map is
      --  The keys of the first action of a get_action reply with their value
      --  counts; empty when the reply carries no action.
      Result : constant Node := Lookup (Doc, Lookup (Doc, Root (Doc), "payload"), "result");
      Action : constant Node := Element (Doc, Result, 1);
      Keys   : Key_Counts.Map;
   begin
      if Kind_Of (Doc, Action) = Map_Value then
         for I in 1 .. Driver.Msgpack.Count (Doc, Action) loop
            Keys.Include (Text (Doc, Key (Doc, Action, I)), Value_Count (Doc, Value (Doc, Action, I)));
         end loop;
      end if;
      return Keys;
   end Action_Keys;

   function Image (Keys : Key_Counts.Map) return String is
      S : Unbounded_String;
   begin
      for C in Keys.Iterate loop
         Append (S, (if Length (S) = 0 then "" else ", ") & Key_Counts.Key (C) & Natural'Image (Key_Counts.Element (C)));
      end loop;
      return "{" & To_String (S) & "}";
   end Image;

   Path      : Unbounded_String;
   Limit     : Natural := 0;
   Reference : Key_Counts.Map;
   Problems  : Natural := 0;
   Sent      : Natural := 0;
   Actions   : Natural := 0;

   procedure Problem (What : String) is
   begin
      Problems := Problems + 1;
      if Problems <= 20 then
         Put_Line ("  " & What);
      end if;
   end Problem;

   procedure Find_Reference is
      --  The action keys of the recorded driver: its first non-empty action.
      R       : Driver.Recording.Reader;
      Kind    : Driver.Recording.Record_Kind;
      Ns      : Long_Long_Integer;
      Payload : Driver.Bytes.Buffer;
      Ok      : Boolean;
   begin
      Driver.Recording.Open (R, To_String (Path), Ok);
      while Ok loop
         Driver.Recording.Next (R, Kind, Ns, Payload, Ok);
         exit when not Ok;
         if Kind = Driver.Recording.Driver_Message then
            declare
               Doc     : Document;
               Decoded : Boolean;
            begin
               Decode (Payload.To_Array, Doc, Decoded);
               if Decoded and then not Action_Keys (Doc).Is_Empty then
                  Reference := Action_Keys (Doc);
                  exit;
               end if;
            end;
         end if;
      end loop;
      Driver.Recording.Close (R);
   end Find_Reference;

   procedure Check (Request_Bytes, Reply_Bytes : Driver.Bytes.Byte_Array) is
      Request : Driver.Protocol.Request;
      Reply   : Document;
      Ok      : Boolean;
   begin
      Driver.Protocol.Decode (Request_Bytes, Request, Ok);
      if not Ok then
         return;   --  not a protocol message; the driver's answer to it is not checked
      end if;
      Decode (Reply_Bytes, Reply, Ok);
      if not Ok or else Kind_Of (Reply, Root (Reply)) /= Map_Value then
         Problem ("message" & Natural'Image (Sent) & " answered with something that is not a map");
         return;
      end if;
      declare
         Want : constant String := Driver.Protocol.Reply_Type (Request.Kind);
         Got  : constant String := Text (Reply, Lookup (Reply, Root (Reply), "message_type"));
         Top  : constant Node := Root (Request.Doc);
      begin
         if Got /= Want then
            Problem ("message" & Natural'Image (Sent) & ": " & Want & " expected, " & Got & " received");
         end if;
         for I in 1 .. Driver.Msgpack.Count (Request.Doc, Top) loop
            declare
               Field : constant String := Text (Request.Doc, Key (Request.Doc, Top, I));
            begin
               if Driver.Protocol.Is_Echoed (Field) then
                  declare
                     Mine, Theirs : Driver.Bytes.Buffer;
                     Echo : constant Node := Lookup (Reply, Root (Reply), Field);
                  begin
                     Put_Node (Mine, Request.Doc, Value (Request.Doc, Top, I));
                     if Echo /= No_Node then
                        Put_Node (Theirs, Reply, Echo);
                     end if;
                     if Echo = No_Node or else Mine.To_Array /= Theirs.To_Array then
                        Problem ("message" & Natural'Image (Sent) & ": " & Field & " not echoed");
                     end if;
                  end;
               end if;
            end;
         end loop;
      end;
      if Driver.Protocol.Wants_Action (Request) then
         declare
            Keys : constant Key_Counts.Map := Action_Keys (Reply);
         begin
            if Keys.Is_Empty then
               Problem ("message" & Natural'Image (Sent) & ": get_action answered without an action");
            else
               Actions := Actions + 1;
               if Keys /= Reference then
                  Problem ("message" & Natural'Image (Sent) & ": action " & Image (Keys)
                           & " differs from the recorded driver's " & Image (Reference));
               end if;
            end if;
         end;
      end if;
   end Check;

   Link    : Driver.Wire.Connection;
   R       : Driver.Recording.Reader;
   Kind    : Driver.Recording.Record_Kind;
   Ns      : Long_Long_Integer;
   Payload : Driver.Bytes.Buffer;
   Reply   : Driver.Bytes.Buffer;
   Got     : Driver.Wire.Message_Kind;
   Ok      : Boolean;

begin
   if Argument_Count not in 3 .. 4 then
      Put_Line (Standard_Error, "usage: feed RECORDING DRIVER_HOST DRIVER_PORT [MESSAGES]");
      Set_Exit_Status (Failure);
      return;
   end if;
   Path := To_Unbounded_String (Argument (1));
   if Argument_Count = 4 then
      Limit := Natural'Value (Argument (4));
   end if;
   Find_Reference;
   Driver.Wire.Connect (Link, Argument (2), Natural'Value (Argument (3)), Ok);
   if not Ok then
      Put_Line (Standard_Error, "feed: no WebSocket server at " & Argument (2) & ":" & Argument (3));
      Set_Exit_Status (Failure);
      return;
   end if;
   Driver.Recording.Open (R, To_String (Path), Ok);
   if not Ok then
      Put_Line (Standard_Error, "feed: " & To_String (Path) & " is not a recording");
      Set_Exit_Status (Failure);
      return;
   end if;
   loop
      Driver.Recording.Next (R, Kind, Ns, Payload, Ok);
      exit when not Ok or else (Limit > 0 and then Sent >= Limit);
      if Kind = Driver.Recording.Robot_Message then
         Driver.Wire.Send (Link, Payload.To_Array, Ok);
         if Ok then
            Driver.Wire.Receive (Link, Got, Reply);
         end if;
         if not Ok or else Got = Driver.Wire.Closed then
            Problem ("the driver closed the connection after" & Natural'Image (Sent) & " messages");
            exit;
         end if;
         Sent := Sent + 1;
         Check (Payload.To_Array, Reply.To_Array);
      end if;
   end loop;
   Driver.Recording.Close (R);
   Driver.Wire.Disconnect (Link);
   Put_Line ("sent" & Natural'Image (Sent) & " messages," & Natural'Image (Actions) & " actions,"
             & Natural'Image (Problems) & " problems");
   if Problems > 0 then
      Set_Exit_Status (Failure);
   end if;
end Feed;
