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

private

   type Ticket is new Natural;

end Driver.Services;
