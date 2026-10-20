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
      procedure Start (Indices : Positive);
      procedure Ended (E : Ada.Exceptions.Exception_Occurrence);
      --  One index ended, with E the failure when it failed (Null_Occurrence when it did not).
      entry Wait;
      --  Every index has ended.
      procedure Raise_Kept;
   private
      Left : Natural := 0;
      Kept : Ada.Exceptions.Exception_Occurrence;
      Has  : Boolean := False;
   end Batch;

   protected body Batch is
      procedure Start (Indices : Positive) is
      begin
         Left := Indices;
      end Start;

      procedure Ended (E : Ada.Exceptions.Exception_Occurrence) is
         use type Ada.Exceptions.Exception_Id;
      begin
         if not Has and then Ada.Exceptions.Exception_Identity (E) /= Ada.Exceptions.Null_Id then
            Ada.Exceptions.Save_Occurrence (Kept, E);
            Has := True;
         end if;
         Left := Left - 1;
      end Ended;

      entry Wait when Left = 0 is
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
      Index : Positive;
      Of_It : Batch_Access;
   end record;

   procedure Do_Job (J : Job);
   --  Runs the index and tells its batch it ended, with the failure if it failed.

   procedure Do_Job (J : Job) is
   begin
      begin
         J.Work (J.Index);
      exception
         when E : others =>
            J.Of_It.Ended (E);
            return;
      end;
      J.Of_It.Ended (Ada.Exceptions.Null_Occurrence);
   end Do_Job;

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
         Do_Job (Mine);
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
      Mine.Start (Last - First + 1);
      for I in First .. Last loop
         declare
            J : constant Job := (Work => Work, Index => I, Of_It => Mine'Unchecked_Access);
            W : Natural;
         begin
            Idle.Pop (W);
            if W = 0 then
               Do_Job (J);
            else
               Pool (W).Go (J);
            end if;
         end;
      end loop;
      Mine.Wait;
      Mine.Raise_Kept;
   end Run;

begin
   for N in Pool'Range loop
      Pool (N) := new Worker (N);
   end loop;
end Driver.Workers;
