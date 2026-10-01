separate (Act)
procedure Solid_Of (C : Context; Name : Unbounded_String; Shape : out Contact.V3_Vectors.Vector; Reprojected : out Boolean) is
   Top : Contact.V3_Vectors.Vector := C.Sil_Pts;
begin
   Shape.Clear;
   Reprojected := False;
   if not C.Sil_Valid or else C.Sil_Name /= Name or else not (C.Touch_Valid or else C.Board_Plane) then
      return;
   end if;
   if C.Touch_Valid and then not C.Sil_Rays.Is_Empty then
      declare
         P0 : constant Geom.V3 := Plane_Point (C, C.Sil_P0, C.Sil_N, Say => False);
         Dropped : Natural;
         Again : Contact.V3_Vectors.Vector;
      begin
         if Geom.Norm ([P0 (0) - C.Sil_P0 (0), P0 (1) - C.Sil_P0 (1), P0 (2) - C.Sil_P0 (2)]) > C.Sil_Pitch then
            Contact.Surface.On_Plane (C.Sil_Rays, P0, C.Sil_N, Again, Dropped);
            if not Again.Is_Empty then
               Top := Again;
               Reprojected := True;
            end if;
         end if;
      end;
   end if;
   Contact.Surface.Walls_To_Support (Top, Lie_N (C), Lie_P (C), C.Sil_Pitch, Shape);
end Solid_Of;
