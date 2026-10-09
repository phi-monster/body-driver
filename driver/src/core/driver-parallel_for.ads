--  Work done for several indices at once: each index but the first on a task
--  of its own, the first on the caller's, and the call returns when every one
--  has. The works must not touch what another of them writes; each index's
--  result is then what it would be done alone, in any order. An exception
--  raised by any of them is raised again by the call, once all have ended (the
--  first one raised, when several were).

generic
   with procedure Work (Index : Positive);
procedure Driver.Parallel_For (First : Positive; Last : Natural);
--  Nothing is done when Last is below First.
