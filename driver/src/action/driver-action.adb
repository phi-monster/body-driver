package body Driver.Action is

   --  Path C replaces these placeholders.

   Unbuilt : exception;

   function Quantities (C : Context) return Name_Vectors.Vector is (raise Unbuilt with "Quantities");

   function Can_Bind (C : Context; R : Role) return Boolean is (raise Unbuilt with "Can_Bind");

   function Check (C : Context; W : Want) return Verdict is (raise Unbuilt with "Check");

   procedure Execute (C : in out Context; W : Want; R : out Result) is
   begin
      raise Unbuilt with "Execute";
   end Execute;

end Driver.Action;
