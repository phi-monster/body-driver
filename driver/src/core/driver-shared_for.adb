with System.Multiprocessors;
with Driver.Parallel_For;

procedure Driver.Shared_For (First : Positive; Last : Natural) is

   --  The next index nobody has taken; past Last when all are, or when a work failed.
   protected Next is
      procedure Take (Index : out Natural);
      procedure Stop;
   private
      Free : Natural := First;
   end Next;

   protected body Next is
      procedure Take (Index : out Natural) is
      begin
         Index := Free;
         if Free <= Last then
            Free := Free + 1;
         end if;
      end Take;

      procedure Stop is
      begin
         Free := Last + 1;
      end Stop;
   end Next;

   procedure Share (Task_Number : Positive) is
      pragma Unreferenced (Task_Number);
      Index : Natural;
   begin
      loop
         Next.Take (Index);
         exit when Index > Last;
         begin
            Work (Index);
         exception
            when others =>
               Next.Stop;
               raise;
         end;
      end loop;
   end Share;

   procedure Shares is new Driver.Parallel_For (Share);
begin
   if Last < First then
      return;
   end if;
   Shares (1, Natural'Min (Last - First + 1, Natural (System.Multiprocessors.Number_Of_CPUs)));
end Driver.Shared_For;
