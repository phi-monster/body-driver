with Ada.Environment_Variables;
with Ada.Exceptions;
with System.Multiprocessors;

package body Driver.Workers is

   Count : constant Natural :=
     (if Ada.Environment_Variables.Exists ("BD_WORKERS")
      then Natural'Value (Ada.Environment_Variables.Value ("BD_WORKERS"))
      else Natural (System.Multiprocessors.Number_Of_CPUs) - 1);
   --  The workers; the caller of a Run is the one more. BD_WORKERS sets how many, where many drivers share the
   --  processors (a bench of fits, one an arm: each its own processor, none waiting on the others' workers).

   function Size return Positive is (Count + 1);

   --  What one call of Run has out: how many of its indices have not ended,
   --  and the first failure among them. Lives on the caller's stack for the
   --  call; the workers reach it through an access that the call outlives.
   protected type Batch is
      procedure Start (First, Last : Positive);
      procedure Take (Index : out Natural);
      --  The next of its indices none has taken yet, and nought when every one has been.
      procedure Ended (E : Ada.Exceptions.Exception_Occurrence);
      --  One index ended, with E the failure when it failed (Null_Occurrence when it did not).
      procedure Join;
      procedure Leave;
      --  A worker taken on, and that worker done with the batch: it reaches the batch no more after Leave.
      entry Wait;
      --  Every index has ended and every worker taken on has left: a worker may be taken on and find every index
      --  gone, and the batch must outlive its last look.
      procedure Raise_Kept;
   private
      Next : Positive := 1;
      Upto : Natural := 0;
      Left : Natural := 0;
      Taken_On : Natural := 0;
      Kept : Ada.Exceptions.Exception_Occurrence;
      Has  : Boolean := False;
   end Batch;

   protected body Batch is
      procedure Start (First, Last : Positive) is
      begin
         Next := First;
         Upto := Last;
         Left := Last - First + 1;
      end Start;

      procedure Take (Index : out Natural) is
      begin
         if Next <= Upto then
            Index := Next;
            Next := Next + 1;
         else
            Index := 0;
         end if;
      end Take;

      procedure Ended (E : Ada.Exceptions.Exception_Occurrence) is
         use type Ada.Exceptions.Exception_Id;
      begin
         if not Has and then Ada.Exceptions.Exception_Identity (E) /= Ada.Exceptions.Null_Id then
            Ada.Exceptions.Save_Occurrence (Kept, E);
            Has := True;
         end if;
         Left := Left - 1;
      end Ended;

      procedure Join is
      begin
         Taken_On := Taken_On + 1;
      end Join;

      procedure Leave is
      begin
         Taken_On := Taken_On - 1;
      end Leave;

      entry Wait when Left = 0 and then Taken_On = 0 is
      begin
         null;
      end Wait;

      procedure Raise_Kept is
      begin
         if Has then
            Ada.Exceptions.Reraise_Occurrence (Kept);
         end if;
      end Raise_Kept;
   end Batch;

   type Batch_Access is access all Batch;

   type Job is record
      Work  : Work_Access;
      Of_It : Batch_Access;
   end record;

   procedure Do_Jobs (J : Job);
   --  Takes the batch's indices one after another until none is left, runs each and tells the batch it ended, with
   --  the failure if it failed.

   procedure Do_Jobs (J : Job) is
      Index : Natural;
   begin
      loop
         J.Of_It.Take (Index);
         exit when Index = 0;
         declare
            Failed : Boolean := False;
         begin
            begin
               J.Work (Index);
            exception
               when E : others =>
                  J.Of_It.Ended (E);
                  Failed := True;
            end;
            if not Failed then
               J.Of_It.Ended (Ada.Exceptions.Null_Occurrence);
            end if;
         end;
      end loop;
   end Do_Jobs;

   type Worker_Stack is array (Positive range <>) of Positive;

   --  The workers that have nothing to do, by number.
   protected Idle is
      procedure Push (Worker : Positive);
      procedure Pop (Worker : out Natural);
      --  Zero when none is idle.
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
   end Idle;

   task type Worker (Number : Positive) is
      entry Go (J : Job);
   end Worker;

   task body Worker is
      Mine : Job;
   begin
      Idle.Push (Number);
      loop
         select
            accept Go (J : Job) do
               Mine := J;
            end Go;
         or
            terminate;
         end select;
         Do_Jobs (Mine);
         Mine.Of_It.Leave;
         Idle.Push (Number);
      end loop;
   end Worker;

   type Worker_Access is access Worker;
   Pool : array (1 .. Count) of Worker_Access;

   procedure Run (Work : not null Work_Access; First : Positive; Last : Natural) is
      Mine : aliased Batch;
   begin
      if Last < First then
         return;
      end if;
      Mine.Start (First, Last);
      declare
         J : constant Job := (Work => Work, Of_It => Mine'Unchecked_Access);
         W : Natural;
      begin
         --  As many of the idle workers as there are indices beyond the caller's own; then the caller takes its
         --  share with them.
         for Recruit in First + 1 .. Last loop
            Idle.Pop (W);
            exit when W = 0;
            Mine.Join;
            Pool (W).Go (J);
         end loop;
         Do_Jobs (J);
      end;
      Mine.Wait;
      Mine.Raise_Kept;
   end Run;

begin
   for N in Pool'Range loop
      Pool (N) := new Worker (N);
   end loop;
end Driver.Workers;
