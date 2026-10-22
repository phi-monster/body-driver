with Ada.Environment_Variables;
with Ada.Exceptions;
with Ada.Real_Time;
with System.Atomic_Operations.Exchange;
with System.Atomic_Operations.Integer_Arithmetic;
with System.Multiprocessors;

package body Driver.Workers is

   use type Ada.Real_Time.Time;
   use type Ada.Real_Time.Time_Span;

   Count : constant Natural :=
     (if Ada.Environment_Variables.Exists ("BD_WORKERS")
      then Natural'Value (Ada.Environment_Variables.Value ("BD_WORKERS"))
      else Natural (System.Multiprocessors.Number_Of_CPUs) - 1);
   --  The workers; the caller of a Run is the one more. BD_WORKERS sets how many, where many drivers share the
   --  processors (a bench of fits, one an arm: each its own processor, none waiting on the others' workers).

   function Size return Positive is (Count + 1);

   type Counter is range Integer'First .. Integer'Last with Atomic;
   package Counting is new System.Atomic_Operations.Integer_Arithmetic (Counter);
   package Swapping is new System.Atomic_Operations.Exchange (Counter);

   --  One call of Run: its indices, how far they have been taken and how many have ended, and the first failure
   --  among them. Lives on the caller's stack for the call; a worker reaches it only through its slot, which the
   --  call empties, and waits for every worker looking at it to leave, before it returns.
   type Batch is limited record
      Work   : Work_Access;
      Next   : aliased Counter := 0;   --  the first index none has taken yet
      Last   : Counter := 0;
      Ended  : aliased Counter := 0;   --  how many indices have ended
      Failed : aliased Counter := 0;   --  1 once an index failed: the first to set it keeps its occurrence
      Kept   : Ada.Exceptions.Exception_Occurrence;
   end record;

   type Batch_Pointer is access all Batch with Atomic;

   --  Where a call puts its batch for the workers to find: one a caller at a time, two for every processor (each
   --  caller with one call of its own work under it). A call that finds none free does its indices itself.
   type Slot is limited record
      Held  : aliased Counter := 0;      --  1 while a call holds it
      Work  : Batch_Pointer := null;     --  the call's batch while it has indices out
      Users : aliased Counter := 0;      --  workers looking at the batch now
   end record;

   Slots : array (1 .. 2 * Size) of Slot;

   Rung   : aliased Counter := 0;    --  calls that have put out a batch so far: a change sends idle workers looking
   Posted : aliased Counter := 0;    --  slots with a batch in them now
   Acks   : aliased Counter := 0;    --  wakings answered, for the measure of what one costs

   --  How long an idle worker looks for new work before it sleeps, in nanoseconds: what a sleep and a waking cost
   --  (measured as the pool starts, the median over the workers). Waiting longer for work than a waking costs saves
   --  nothing, and sleeping sooner pays it again on every call: looking that long costs at most twice the best
   --  choice whatever the gaps between the calls are. Every call used to hand its work to each worker in a
   --  rendezvous, one after another: on the Mac a call of 64 indices of 3 us each took 436 us with nine workers
   --  and 208 us with none (tools/pool_bench, 10-22), and on the big machine a step 2 fit took 39 s with 127 workers
   --  against 33 s with 63 (k202's, r19).
   Look_For : aliased Counter := 0;

   procedure Take_Part (B : not null access Batch);
   --  Takes runs of B's indices one after another until none is left, runs each and counts them ended. A run is
   --  the indices left over the pool's size, rounded up (guided self-scheduling): many at first, single ones at the
   --  end, so that the last to end ends about when the others do.

   procedure Take_Part (B : not null access Batch) is
      Left, From, Upto : Counter;
   begin
      loop
         Left := B.Last - B.Next + 1;
         exit when Left <= 0;
         From := Counting.Atomic_Fetch_And_Add (B.Next, (Left + Counter (Size) - 1) / Counter (Size));
         exit when From > B.Last;
         Upto := Counter'Min (B.Last, From + (Left + Counter (Size) - 1) / Counter (Size) - 1);
         for Index in From .. Upto loop
            begin
               B.Work (Positive (Index));
            exception
               when E : others =>
                  declare
                     Prior : aliased Counter := 0;
                  begin
                     if Swapping.Atomic_Compare_And_Exchange (B.Failed, Prior, 1) then
                        Ada.Exceptions.Save_Occurrence (B.Kept, E);
                     end if;
                  end;
            end;
         end loop;
         Counting.Atomic_Add (B.Ended, Upto - From + 1);
      end loop;
   end Take_Part;

   --  Every slot with a batch in it whose indices are not all taken: its part taken. Whether any was.
   function Take_Any return Boolean is
      Found : Boolean := False;
   begin
      if Posted > 0 then
         for S of Slots loop
            if S.Work /= null then
               Counting.Atomic_Add (S.Users, 1);
               declare
                  B : constant Batch_Pointer := S.Work;
               begin
                  if B /= null and then B.Next <= B.Last then
                     Take_Part (B);
                     Found := True;
                  end if;
               end;
               Counting.Atomic_Subtract (S.Users, 1);
            end if;
         end loop;
      end if;
      return Found;
   end Take_Any;

   type Worker_Stack is array (Positive range <>) of Positive;

   --  The workers asleep, by number.
   protected Idle is
      procedure Push (Worker : Positive);
      procedure Pop (Worker : out Natural);
      --  Zero when none sleeps.
      procedure Remove (Worker : Positive; Was_There : out Boolean);
      --  Takes Worker off the stack if it is there.
   private
      Stack : Worker_Stack (1 .. Natural'Max (1, Count));
      Top   : Natural := 0;
   end Idle;

   protected body Idle is
      procedure Push (Worker : Positive) is
      begin
         Top := Top + 1;
         Stack (Top) := Worker;
      end Push;

      procedure Pop (Worker : out Natural) is
      begin
         if Top = 0 then
            Worker := 0;
         else
            Worker := Stack (Top);
            Top := Top - 1;
         end if;
      end Pop;

      procedure Remove (Worker : Positive; Was_There : out Boolean) is
      begin
         Was_There := False;
         for K in 1 .. Top loop
            if Stack (K) = Worker then
               Stack (K .. Top - 1) := Stack (K + 1 .. Top);
               Top := Top - 1;
               Was_There := True;
               return;
            end if;
         end loop;
      end Remove;
   end Idle;

   task type Worker (Number : Positive) is
      entry Wake;
   end Worker;

   type Worker_Access is access Worker;
   Pool : array (1 .. Count) of Worker_Access;

   --  Whether a batch out has indices none has taken yet: its slot looked at as a worker looks at it (Take_Any).
   function Work_Left return Boolean is
      Found : Boolean := False;
   begin
      if Posted > 0 then
         for S of Slots loop
            exit when Found;
            if S.Work /= null then
               Counting.Atomic_Add (S.Users, 1);
               declare
                  B : constant Batch_Pointer := S.Work;
               begin
                  Found := B /= null and then B.Next <= B.Last;
               end;
               Counting.Atomic_Subtract (S.Users, 1);
            end if;
         end loop;
      end if;
      return Found;
   end Work_Left;

   --  Two sleeping workers woken while a batch out has indices left, by whoever was woken for it: the wakings spread
   --  as a tree, each woken worker waking two more, and every sleeper is up after as many wakings in a row as the
   --  logarithm of their number. A caller that woke every sleeper itself, one after another, spent 400 of a call's
   --  480 us on the big machine with 127 workers (tools/pool_bench, 128 indices of 3 us, 10-22); woken while the
   --  batch was merely out, every worker rose for the three eyes of every beat (a kiwi's step 3, the eyes 9 ms a
   --  beat with 127 workers against 5 with 11).
   procedure Wake_Two is
      W : Natural;
   begin
      for Woken in 1 .. 2 loop
         exit when not Work_Left;
         Idle.Pop (W);
         exit when W = 0;
         Pool (W).Wake;
      end loop;
   end Wake_Two;

   task body Worker is
      Seen  : Counter;
      Since : Ada.Real_Time.Time;
   begin
      loop
         Seen := Rung;
         if not Take_Any then
            --  Nothing to take: it looks for new work as long as a waking costs, then sleeps.
            Since := Ada.Real_Time.Clock;
            while Rung = Seen
              and then Ada.Real_Time.Clock - Since < Ada.Real_Time.Nanoseconds (Integer (Look_For))
            loop
               null;
            end loop;
            if Rung = Seen then
               Idle.Push (Number);
               if Rung /= Seen then
                  --  Work was put out while it lay down: up again, unless a caller has already taken it off the
                  --  stack to wake it, whose call it then answers.
                  declare
                     Was_There : Boolean;
                  begin
                     Idle.Remove (Number, Was_There);
                     if not Was_There then
                        accept Wake;
                        Counting.Atomic_Add (Acks, 1);
                        Wake_Two;
                     end if;
                  end;
               else
                  select
                     accept Wake;
                     Counting.Atomic_Add (Acks, 1);
                     Wake_Two;
                  or
                     terminate;
                  end select;
               end if;
            end if;
         end if;
      end loop;
   end Worker;

   procedure Run (Work : not null Work_Access; First : Positive; Last : Natural) is
      Mine : aliased Batch;
      Held : Natural := 0;
   begin
      if Last < First then
         return;
      end if;
      Mine.Work := Work;
      Mine.Next := Counter (First);
      Mine.Last := Counter (Last);
      --  A slot, where there are workers and more indices than one.
      if Count > 0 and then Last > First then
         for S in Slots'Range loop
            declare
               Prior : aliased Counter := 0;
            begin
               if Slots (S).Held = 0 and then Swapping.Atomic_Compare_And_Exchange (Slots (S).Held, Prior, 1) then
                  Held := S;
                  exit;
               end if;
            end;
         end loop;
      end if;
      if Held > 0 then
         Slots (Held).Work := Mine'Unchecked_Access;
         Counting.Atomic_Add (Posted, 1);
         Counting.Atomic_Add (Rung, 1);
         --  One sleeping worker woken, which wakes two more (Wake_Two), and so on while the batch is out; the ones
         --  still looking find it themselves.
         declare
            W : Natural;
         begin
            Idle.Pop (W);
            if W > 0 then
               Pool (W).Wake;
            end if;
         end;
      end if;
      Take_Part (Mine'Access);
      while Mine.Ended < Counter (Last - First + 1) loop
         null;
      end loop;
      if Held > 0 then
         Slots (Held).Work := null;
         Counting.Atomic_Subtract (Posted, 1);
         while Slots (Held).Users > 0 loop
            null;
         end loop;
         Slots (Held).Held := 0;
      end if;
      if Mine.Failed > 0 then
         Ada.Exceptions.Reraise_Occurrence (Mine.Kept);
      end if;
   end Run;

begin
   for N in Pool'Range loop
      Pool (N) := new Worker (N);
   end loop;
   --  What a sleep and a waking cost: each worker, once asleep, woken and timed until it answers.
   if Count > 0 then
      declare
         Times : array (1 .. Count) of Integer;
         W     : Natural;
         Before : Counter;
         Start : Ada.Real_Time.Time;
      begin
         for K in Times'Range loop
            loop
               Idle.Pop (W);
               exit when W > 0;
            end loop;
            Before := Acks;
            Start := Ada.Real_Time.Clock;
            Pool (W).Wake;
            while Acks = Before loop
               null;
            end loop;
            Times (K) := Integer (Real (Ada.Real_Time.To_Duration (Ada.Real_Time.Clock - Start)) * 1.0e9);
         end loop;
         --  The median, by insertion: as many as the workers.
         for I in 2 .. Times'Last loop
            declare
               T : constant Integer := Times (I);
               P : Natural := I - 1;
            begin
               while P >= 1 and then Times (P) > T loop
                  Times (P + 1) := Times (P);
                  P := P - 1;
               end loop;
               Times (P + 1) := T;
            end;
         end loop;
         Look_For := Counter (Times ((Times'First + Times'Last) / 2));
      end;
   end if;
end Driver.Workers;
