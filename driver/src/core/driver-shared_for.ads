--  Work done for every index from First to Last by as many tasks as the
--  machine has processors (as many as there are indices, when fewer), each
--  taking the next index nobody has taken when it is free: works of unequal
--  length share the processors, and no task is left with a run of the longest
--  ones while the others wait. The works must not touch what another of them
--  writes; each index's result is then what it would be done alone, in any
--  order. An exception raised by any of them is raised again by the call,
--  once all have ended (the first one raised, when several were); the indices
--  nobody had taken by then are not done.

generic
   with procedure Work (Index : Positive);
procedure Driver.Shared_For (First : Positive; Last : Natural);
--  Nothing is done when Last is below First.
