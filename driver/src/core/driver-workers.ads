--  The tasks every Parallel_For runs its pieces on, made once for the
--  program and kept: one fewer than the machine has processors, the caller
--  being the other. Each call of Parallel_For used to make a task for each
--  of its indices and end them all: a fit's Jacobian made forty a round for
--  hundreds of rounds, and the aligner made one per processor for every
--  keyframe's every pass, ninety-six on the box for two points each, so that
--  the box aligned a keyframe more slowly than a laptop with ten.
--
--  A call takes on as many of the idle workers as it has indices beyond one,
--  and each of them and the caller take its indices one after another until
--  none is left; it returns once every index has ended, and a worker reports
--  back idle when it finds none left. A call that handed each index to one
--  worker, or did it itself when none was idle, waited on its slowest while
--  the others idled (the step 2 bench's nine-joint humanoid s2_1, 10-21: the
--  threads waited a third of their time in the order search). A piece of work
--  that itself calls Parallel_For takes on the workers then idle the same way
--  and does the rest itself: nothing ever waits for a worker, so nested calls
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
