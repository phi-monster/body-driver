with Ada.Containers.Indefinite_Ordered_Maps;
with Driver.Bytes;
with Driver.Http;
with Driver.Recording;

package body Driver.Services is

   type Endpoint is record
      Host : Unbounded_String;
      Port : Natural := 0;
   end record;

   Endpoints : array (Service) of Endpoint;

   procedure Configure (S : Service; Host : String; Port : Natural) is
   begin
      Endpoints (S) := (Host => To_Unbounded_String (Host), Port => Port);
   end Configure;

   function Name (S : Service) return String is (case S is when Brain => "brain", when Instrument => "instrument");

   procedure Record_Request (S : Service; Path, Request : String) is
   begin
      Driver.Recording.Write_Shared
        (Driver.Recording.Service_Request, Driver.Bytes.To_Bytes (Name (S) & ASCII.LF & Path & ASCII.LF & Request));
   end Record_Request;

   procedure Record_Reply (S : Service; R : Reply) is
   begin
      Driver.Recording.Write_Shared
        (Driver.Recording.Service_Reply,
         Driver.Bytes.To_Bytes (Name (S) & ASCII.LF & (if R.Ok then "ok" else To_String (R.Why)) & ASCII.LF
                                & To_String (R.Text)));
   end Record_Reply;

   function Unconfigured (S : Service) return Boolean is (Endpoints (S).Port = 0);

   function Not_Configured (S : Service) return Reply is
     (Ok => False, Text => Null_Unbounded_String,
      Why => To_Unbounded_String ("no address was given for the " & Name (S) & " service"));

   function Call (S : Service; Path : String; Request : String) return Reply is
   begin
      if Unconfigured (S) then
         return Not_Configured (S);
      end if;
      Record_Request (S, Path, Request);
      declare
         H : constant Driver.Http.Response :=
           Driver.Http.Post (To_String (Endpoints (S).Host), Endpoints (S).Port, Path, Request);
         R : constant Reply := (Ok => H.Ok, Text => H.Body_Text, Why => H.Why);
      begin
         Record_Reply (S, R);
         return R;
      end;
   end Call;

   function Call_Streaming
     (S       : Service;
      Path    : String;
      Request : String;
      On_Text : not null access procedure (Chunk : String; Stop : out Boolean)) return Reply
   is
      Line : Unbounded_String;
      Data : Unbounded_String;   --  every event's data, which is what Text returns

      --  Server-sent events: lines "data: <payload>", events separated by a
      --  blank line; "[DONE]" ends an OpenAI-compatible stream.
      procedure On_Data (Piece : String; Stop : out Boolean) is
      begin
         Stop := False;
         for C of Piece loop
            if C = ASCII.LF then
               declare
                  L : constant String := To_String (Line);
                  T : constant String := (if L'Length > 0 and then L (L'Last) = ASCII.CR
                                          then L (L'First .. L'Last - 1) else L);
                  Prefix : constant String := "data:";
               begin
                  Line := Null_Unbounded_String;
                  if T'Length >= Prefix'Length and then T (T'First .. T'First + Prefix'Length - 1) = Prefix then
                     declare
                        Payload : constant String := T (T'First + Prefix'Length .. T'Last);
                        Event   : constant String :=
                          (if Payload'Length > 0 and then Payload (Payload'First) = ' '
                           then Payload (Payload'First + 1 .. Payload'Last) else Payload);
                     begin
                        if Event = "[DONE]" then
                           Stop := True;
                           return;
                        end if;
                        Append (Data, Event & ASCII.LF);
                        On_Text (Event, Stop);
                        if Stop then
                           return;
                        end if;
                     end;
                  end if;
               end;
            else
               Append (Line, C);
            end if;
         end loop;
      end On_Data;
   begin
      if Unconfigured (S) then
         return Not_Configured (S);
      end if;
      Record_Request (S, Path, Request);
      declare
         H : constant Driver.Http.Response :=
           Driver.Http.Post_Streaming (To_String (Endpoints (S).Host), Endpoints (S).Port, Path, Request,
                                       On_Data'Access);
         R : constant Reply := (Ok => H.Ok, Text => Data, Why => H.Why);
      begin
         Record_Reply (S, R);
         return R;
      end;
   end Call_Streaming;

   --  Asynchronous calls: each service has one worker task that serves its
   --  queue in order; results wait in a protected table until collected.

   type Job is record
      T             : Ticket := 0;
      Path, Request : Unbounded_String;
   end record;

   package Job_Lists is new Ada.Containers.Indefinite_Ordered_Maps (Ticket, Job);
   package Reply_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Ticket, Reply);

   type Queue_Array is array (Service) of Job_Lists.Map;

   protected Results is
      procedure Enqueue (S : Service; Path, Request : String; T : out Ticket);
      procedure Enqueue_Stop (S : Service);
      entry Next_Brain (J : out Job);
      entry Next_Instrument (J : out Job);
      procedure Put (T : Ticket; R : Reply);
      function Has (T : Ticket) return Boolean;
      procedure Take (T : Ticket; R : out Reply);
   private
      Last_Ticket : Ticket := 0;
      Queues      : Queue_Array;
      Ready       : Reply_Maps.Map;
   end Results;

   protected body Results is
      procedure Enqueue (S : Service; Path, Request : String; T : out Ticket) is
      begin
         Last_Ticket := Last_Ticket + 1;
         T := Last_Ticket;
         Queues (S).Insert (T, (T => T, Path => To_Unbounded_String (Path), Request => To_Unbounded_String (Request)));
      end Enqueue;

      procedure Enqueue_Stop (S : Service) is
      begin
         --  Ticket 0 is never issued and sorts first: the worker stops next.
         Queues (S).Include (0, (T => 0, Path => Null_Unbounded_String, Request => Null_Unbounded_String));
      end Enqueue_Stop;

      entry Next_Brain (J : out Job) when not Queues (Brain).Is_Empty is
      begin
         J := Queues (Brain).First_Element;
         Queues (Brain).Delete_First;
      end Next_Brain;

      entry Next_Instrument (J : out Job) when not Queues (Instrument).Is_Empty is
      begin
         J := Queues (Instrument).First_Element;
         Queues (Instrument).Delete_First;
      end Next_Instrument;

      procedure Put (T : Ticket; R : Reply) is
      begin
         Ready.Include (T, R);
      end Put;

      function Has (T : Ticket) return Boolean is (Ready.Contains (T));

      procedure Take (T : Ticket; R : out Reply) is
      begin
         R := Ready.Element (T);
         Ready.Delete (T);
      end Take;
   end Results;

   task type Worker (S : Service);
   type Worker_Access is access Worker;

   task body Worker is
      J : Job;
   begin
      loop
         case S is
            when Brain      => Results.Next_Brain (J);
            when Instrument => Results.Next_Instrument (J);
         end case;
         exit when J.T = 0;   --  the stop job from Shut_Down
         Results.Put (J.T, Call (S, To_String (J.Path), To_String (J.Request)));
      end loop;
   end Worker;

   --  Workers start with the first submission, so a program that never
   --  submits (the self test) has no task keeping it alive.
   Workers : array (Service) of Worker_Access;

   procedure Shut_Down is
   begin
      for S in Service loop
         if Workers (S) /= null then
            Results.Enqueue_Stop (S);
         end if;
      end loop;
   end Shut_Down;

   function Submit (S : Service; Path : String; Request : String; Beat : Driver.Clock.Beat) return Ticket is
      T : Ticket;
   begin
      if Workers (S) = null then
         Workers (S) := new Worker (S);
      end if;
      Driver.Recording.Write_Shared
        (Driver.Recording.Service_Request,
         Driver.Bytes.To_Bytes (Name (S) & " submitted at beat" & Driver.Clock.Beat'Image (Beat)));
      Results.Enqueue (S, Path, Request, T);
      return T;
   end Submit;

   function Ready (T : Ticket) return Boolean is (Results.Has (T));

   function Collect (T : Ticket) return Reply is
      R : Reply;
   begin
      Results.Take (T, R);
      return R;
   end Collect;

end Driver.Services;
