with Driver.Workers;

procedure Driver.Parallel_For (First : Positive; Last : Natural) is
   --  Work may be nested in the caller, and a generic formal has no access
   --  of its own: this one stands for it. The workers only ever call it
   --  before Run returns (Driver.Workers), so the access does not outlive it.
   procedure Call (Index : Positive);
   procedure Call (Index : Positive) is
   begin
      Work (Index);
   end Call;
begin
   Driver.Workers.Run (Call'Unrestricted_Access, First, Last);
end Driver.Parallel_For;
