--  The tasks every Parallel_For runs its pieces on, made once for the
--  program and kept: one fewer than the machine has processors, the caller
--  being the other. Each call of Parallel_For used to make a task for each
--  of its indices and end them all: a fit's Jacobian made forty a round for
--  hundreds of rounds, and the aligner made one per processor for every
--  keyframe's every pass, ninety-six on the box for two points each, so that
--  the box aligned a keyframe more slowly than a laptop with ten.
--
--  A call hands each index to an idle worker, or does it itself when none is
--  idle, and returns once every index has ended; a worker that is given an
--  index runs it and reports back idle. A piece of work that itself calls
--  Parallel_For hands its own indices on the same way, and runs itself what
--  no worker is free for: nothing ever waits for a worker, so nested calls
--  cannot deadlock. The first exception any index raises is raised again by
--  the call, once all its indices have ended, as Parallel_For promises.
private package Driver.Workers is

   type Work_Access is access procedure (Index : Positive);

   function Size return Positive;
   --  How many indices a Run does at once: the workers and the caller.

   procedure Run (Work : not null Work_Access; First : Positive; Last : Natural);
   --  Work (I) for every I in First .. Last, each once, at once as far as the
   --  workers are idle; returns when all have ended. Work is only ever called
   --  before Run returns, so a procedure nested in the caller may be given
   --  (through 'Unrestricted_Access: the pool cannot know how deep the work's
   --  declaration is, and never keeps the access past the call).

end Driver.Workers;
