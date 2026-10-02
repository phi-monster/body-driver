with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Vectors;
with Ada.Strings.Fixed;
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

   function Image (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   --  Calls run in the decider, in the workers and in the main program, so
   --  their numbers come from one protected counter.
   protected Calls is
      procedure Next (N : out Positive);
   private
      Last : Natural := 0;
   end Calls;

   protected body Calls is
      procedure Next (N : out Positive) is
      begin
         Last := Last + 1;
         N := Last;
      end Next;
   end Calls;

   function Head (S : Service; Submitted_At : String) return String is
      --  The first line of a call's records (Driver.Recording).
      N : Positive;
   begin
      Calls.Next (N);
      return Name (S) & " " & Image (N) & (if Submitted_At = "" then "" else " beat " & Submitted_At);
   end Head;

   procedure Record_Request (Call_Head, Path, Request : String) is
   begin
      Driver.Recording.Write_Shared
        (Driver.Recording.Service_Request, Driver.Bytes.To_Bytes (Call_Head & ASCII.LF & Path & ASCII.LF & Request));
   end Record_Request;

   procedure Record_Reply (Call_Head : String; R : Reply) is
   begin
      Driver.Recording.Write_Shared
        (Driver.Recording.Service_Reply,
         Driver.Bytes.To_Bytes (Call_Head & ASCII.LF & (if R.Ok then "ok" else To_String (R.Why)) & ASCII.LF
                                & To_String (R.Text)));
   end Record_Reply;

   function Unconfigured (S : Service) return Boolean is (Endpoints (S).Port = 0);

   function Not_Configured (S : Service) return Reply is
     (Ok => False, Text => Null_Unbounded_String,
      Why => To_Unbounded_String ("no address was given for the " & Name (S) & " service"), Lasting => True);

   function Recorded_Call (S : Service; Path, Request, Submitted_At : String) return Reply is
   begin
      if Unconfigured (S) then
         return Not_Configured (S);
      end if;
      declare
         Call_Head : constant String := Head (S, Submitted_At);
      begin
         Record_Request (Call_Head, Path, Request);
         declare
            H : constant Driver.Http.Response :=
              Driver.Http.Post (To_String (Endpoints (S).Host), Endpoints (S).Port, Path, Request);
            R : constant Reply := (Ok => H.Ok, Text => H.Body_Text, Why => H.Why, Lasting => False);
         begin
            Record_Reply (Call_Head, R);
            return R;
         end;
      end;
   end Recorded_Call;

   function Call (S : Service; Path : String; Request : String) return Reply is
     (Recorded_Call (S, Path, Request, Submitted_At => ""));

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
      declare
         Call_Head : constant String := Head (S, Submitted_At => "");
      begin
         Record_Request (Call_Head, Path, Request);
         declare
            H : constant Driver.Http.Response :=
              Driver.Http.Post_Streaming (To_String (Endpoints (S).Host), Endpoints (S).Port, Path, Request,
                                          On_Data'Access);
            --  An error reply is not a stream: its body (a JSON error, say) is
            --  the text, as for a blocking call.
            R : constant Reply :=
              (Ok => H.Ok, Text => (if H.Ok then Data else H.Body_Text), Why => H.Why, Lasting => False);
         begin
            Record_Reply (Call_Head, R);
            return R;
         end;
      end;
   end Call_Streaming;

   --  Asynchronous calls: each service has one worker task that serves its
   --  queue in order; results wait in a protected table until collected.

   type Job is record
      T             : Ticket := 0;
      Path, Request : Unbounded_String;
      Beat          : Driver.Clock.Beat := 0;
   end record;

   package Job_Lists is new Ada.Containers.Indefinite_Ordered_Maps (Ticket, Job);
   package Reply_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Ticket, Reply);

   type Queue_Array is array (Service) of Job_Lists.Map;

   protected Results is
      procedure New_Ticket (T : out Ticket);
      procedure Enqueue (S : Service; J : Job);
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
      procedure New_Ticket (T : out Ticket) is
      begin
         Last_Ticket := Last_Ticket + 1;
         T := Last_Ticket;
      end New_Ticket;

      procedure Enqueue (S : Service; J : Job) is
      begin
         Queues (S).Insert (J.T, J);
      end Enqueue;

      procedure Enqueue_Stop (S : Service) is
      begin
         --  Ticket 0 is never issued and sorts first: the worker stops next.
         Queues (S).Include (0, (T => 0, Path => Null_Unbounded_String, Request => Null_Unbounded_String, Beat => 0));
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

   function Beat_Image (B : Driver.Clock.Beat) return String is (Image (Natural (B)));

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
         Results.Put (J.T, Recorded_Call (S, To_String (J.Path), To_String (J.Request), Beat_Image (J.Beat)));
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

   --  Replay state. A replay is one task, the replay program's own, so this
   --  needs no protection.

   type Waiting_Call is record
      T             : Ticket;
      S             : Service;
      Path, Request : Unbounded_String;
   end record;

   type Unclaimed_Reply is record
      S             : Service;
      Path, Request : Unbounded_String;
      R             : Reply;
   end record;

   package Waiting_Lists is new Ada.Containers.Indefinite_Vectors (Positive, Waiting_Call);
   package Unclaimed_Lists is new Ada.Containers.Indefinite_Vectors (Positive, Unclaimed_Reply);
   package Beat_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Ticket, Driver.Clock.Beat, "<", Driver.Clock."=");

   Replaying    : Boolean := False;
   Recorded     : Service_Set := [others => False];
   Waiting      : Waiting_Lists.Vector;
   Unclaimed    : Unclaimed_Lists.Vector;
   Submitted_At : Beat_Maps.Map;
   Current_Beat : Driver.Clock.Beat := 0;

   procedure Start_Replay (Recorded : Service_Set) is
   begin
      Replaying := True;
      Services.Recorded := Recorded;
   end Start_Replay;

   procedure Replay_Reply (S : Service; Path, Request : String; R : Reply) is
   begin
      for I in 1 .. Natural (Waiting.Length) loop
         declare
            W : constant Waiting_Call := Waiting (I);
         begin
            if W.S = S and then W.Path = Path and then W.Request = Request then
               Results.Put (W.T, R);
               Waiting.Delete (I);
               return;
            end if;
         end;
      end loop;
      Unclaimed.Append (Unclaimed_Reply'(S => S, Path => To_Unbounded_String (Path), Request => To_Unbounded_String (Request),
                         R => R));
   end Replay_Reply;

   procedure Replay_Beat (Beat : Driver.Clock.Beat) is
   begin
      Current_Beat := Beat;
   end Replay_Beat;

   procedure End_Replay is
   begin
      Replaying := False;
      Recorded := [others => False];
      Waiting.Clear;
      Unclaimed.Clear;
      Submitted_At.Clear;
   end End_Replay;

   procedure Submit_Replayed (S : Service; Path, Request : String; T : Ticket) is
   begin
      if not Recorded (S) then
         Results.Put (T, Call (S, Path, Request));
         return;
      end if;
      for I in 1 .. Natural (Unclaimed.Length) loop
         declare
            U : constant Unclaimed_Reply := Unclaimed (I);
         begin
            if U.S = S and then U.Path = Path and then U.Request = Request then
               Results.Put (T, U.R);
               Unclaimed.Delete (I);
               return;
            end if;
         end;
      end loop;
      Waiting.Append (Waiting_Call'(T => T, S => S, Path => To_Unbounded_String (Path), Request => To_Unbounded_String (Request)));
   end Submit_Replayed;

   function Submit (S : Service; Path : String; Request : String; Beat : Driver.Clock.Beat) return Ticket is
      T : Ticket;
   begin
      Results.New_Ticket (T);
      if Replaying then
         Submitted_At.Insert (T, Beat);
         Submit_Replayed (S, Path, Request, T);
         return T;
      end if;
      if Unconfigured (S) then
         --  Answered at once, and no worker is started for a service that
         --  cannot be reached: a worker would outlive the program.
         Results.Put (T, Not_Configured (S));
         return T;
      end if;
      if Workers (S) = null then
         Workers (S) := new Worker (S);
      end if;
      Results.Enqueue (S, (T => T, Path => To_Unbounded_String (Path), Request => To_Unbounded_String (Request),
                           Beat => Beat));
      return T;
   end Submit;

   function Ready (T : Ticket) return Boolean is
     (Results.Has (T)
      and then (not Replaying or else Driver.Clock."<" (Submitted_At.Element (T), Current_Beat)));

   function Collect (T : Ticket) return Reply is
      R : Reply;
   begin
      Results.Take (T, R);
      if Replaying then
         Submitted_At.Delete (T);
      end if;
      return R;
   end Collect;

end Driver.Services;
