separate (Act)
procedure Thing_Shape (C : Context; F : Plug.Frame; Arm : Integer; Name : Unbounded_String; Shape, Rest : out Contact.V3_Vectors.Vector) is
   Rp : Boolean;
   pragma Warnings (Off, Rp);   --  重投过没有只在接触集的那句话里说
begin
   Shape.Clear;
   Rest.Clear;
   if Arm >= 0 and then C.Wld.Holding and then C.Wld.Held_Arm = Arm and then not C.Held_Shape.Is_Empty and then Natural (Arm) < Natural (F.EE.Length) then
      declare
         P0 : constant Plug.Arm_Pose := C.Held_Pose;
         P1 : constant Plug.Arm_Pose := F.EE (Natural (Arm));
         Rd : constant Geom.M3 := Geom.Mul (Geom.Quat_To_R (P1), Geom.Tr (Geom.Quat_To_R (P0)));   --  手从合上那一刻起转了多少(世界系)
      begin
         for Q of C.Held_Shape loop
            declare
               D : constant Geom.V3 := Geom.Ap (Rd, [Q (0) - P0 (0), Q (1) - P0 (1), Q (2) - P0 (2)]);
            begin
               Shape.Append (Geom.V3'[P1 (0) + D (0), P1 (1) + D (1), P1 (2) + D (2)]);
            end;
         end loop;
      end;
      Rest := C.Held_Shape;
   else
      Solid_Of (C, Name, Shape, Rp);
      Rest := Shape;
   end if;
end Thing_Shape;
