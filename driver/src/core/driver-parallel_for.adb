with Ada.Exceptions;

procedure Driver.Parallel_For (First : Positive; Last : Natural) is

   protected Failure is
      procedure Keep (E : Ada.Exceptions.Exception_Occurrence);
      procedure Raise_Kept;
   private
      Kept : Ada.Exceptions.Exception_Occurrence;
      Has  : Boolean := False;
   end Failure;

   protected body Failure is
      procedure Keep (E : Ada.Exceptions.Exception_Occurrence) is
      begin
         if not Has then
            Ada.Exceptions.Save_Occurrence (Kept, E);
            Has := True;
         end if;
      end Keep;

      procedure Raise_Kept is
      begin
         if Has then
            Ada.Exceptions.Reraise_Occurrence (Kept);
         end if;
      end Raise_Kept;
   end Failure;

   task type Worker is
      entry Start (Index : Positive);
   end Worker;

   task body Worker is
      Mine : Positive;
   begin
      accept Start (Index : Positive) do
         Mine := Index;
      end Start;
      Work (Mine);
   exception
      when E : others =>
         Failure.Keep (E);
   end Worker;

begin
   if Last < First then
      return;
   end if;
   declare
      Workers : array (First + 1 .. Last) of Worker;
   begin
      for I in Workers'Range loop
         Workers (I).Start (I);
      end loop;
      begin
         Work (First);
      exception
         when E : others =>
            Failure.Keep (E);
      end;
   end;   --  every worker has ended here
   Failure.Raise_Kept;
end Driver.Parallel_For;
