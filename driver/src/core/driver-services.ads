--  The external services: the brain (an OpenAI-compatible chat endpoint,
--  docs/brain-service.md) and the instrument (segmentation, matching and
--  tracking, docs/instrument-service.md).
--
--  Every request and reply is recorded with the beat it belongs to, so a
--  replay feeds the same replies back at the same beats. Deciders call the
--  blocking forms; estimators submit and collect replies on later beats so
--  the main loop never waits on a service.

with Ada.Strings.Unbounded;
with Driver.Clock;

package Driver.Services is

   use Ada.Strings.Unbounded;

   type Service is (Brain, Instrument);

   procedure Configure (S : Service; Host : String; Port : Natural);

   type Reply is record
      Ok   : Boolean := False;
      Text : Unbounded_String;   --  the reply body, also when the service reported an error
      Why  : Unbounded_String;   --  what went wrong when Ok is False
      Lasting : Boolean := False;
      --  The same call fails the same way whenever it is made (the service has
      --  no address): asking again cannot help. Every other failure may pass.
   end record;

   function Call (S : Service; Path : String; Request : String) return Reply;
   --  Blocking POST of a JSON request.

   function Call_Streaming
     (S       : Service;
      Path    : String;
      Request : String;
      On_Text : not null access procedure (Chunk : String; Stop : out Boolean)) return Reply;
   --  Blocking POST whose reply arrives as server-sent events; On_Text sees
   --  each event's data as it arrives and may stop the stream early. The
   --  returned Text holds everything received up to the stop.

   type Ticket is private;

   --  Submit is for the estimators, which all run on the main loop; deciders
   --  use the blocking calls.
   function Submit (S : Service; Path : String; Request : String; Beat : Driver.Clock.Beat) return Ticket;
   function Ready (T : Ticket) return Boolean;
   function Collect (T : Ticket) return Reply
     with Pre => Ready (T);

   procedure Shut_Down;
   --  Lets the workers behind Submit finish; the main program calls it before exiting.

   --  Replay. A replay runs no deciders, only estimators: their submitted
   --  calls are answered from the recording, or by the live service when the
   --  recording holds no replies of that service, and a reply becomes Ready
   --  on the first beat after the one it was submitted at whichever source
   --  answered it, so a replay never depends on how fast it runs.

   type Service_Set is array (Service) of Boolean;

   procedure Start_Replay (Recorded : Service_Set);
   --  From now on a submitted call of a Recorded service waits for
   --  Replay_Reply; one of any other service is a blocking call to its
   --  configured address, made when it is submitted.

   procedure Replay_Reply (S : Service; Path, Request : String; R : Reply);
   --  A reply the recording holds. It answers the oldest submitted call of S
   --  with this path and request that has no reply yet, or else the next one
   --  submitted.

   procedure Replay_Beat (Beat : Driver.Clock.Beat);
   --  The beat the replay is about to feed.

   procedure End_Replay;
   --  Back to live calls; replayed calls still waiting are dropped.

private

   type Ticket is new Natural;

end Driver.Services;
