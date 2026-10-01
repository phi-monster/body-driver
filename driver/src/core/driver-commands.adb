package body Driver.Commands is

   procedure Set_Target (C : in out Command; G : Group_Id; Values : Real_Array) is
   begin
      C.Targets.Include (G, Values);
   end Set_Target;

   procedure Clear_Target (C : in out Command; G : Group_Id) is
   begin
      C.Targets.Exclude (G);
   end Clear_Target;

   function Has_Target (C : Command; G : Group_Id) return Boolean is (C.Targets.Contains (G));

   function Target (C : Command; G : Group_Id) return Real_Array is (C.Targets.Element (G));

   function Is_Hold (C : Command) return Boolean is (C.Targets.Is_Empty);

end Driver.Commands;
