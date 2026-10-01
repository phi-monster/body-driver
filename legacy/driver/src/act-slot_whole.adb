separate (Act)
procedure Slot_Whole (C : Context; F : Plug.Frame; Cam : Natural; Slot : Integer; Whole, Edge : out Boolean; Name : out Unbounded_String;
                      Named : Unbounded_String := Null_Unbounded_String) is
begin
   Whole := True; Edge := False; Name := Null_Unbounded_String;
   if Length (Named) > 0 then
      Name := Named;
      declare
         Bx : constant Integer := Boxed_By (C, Cam, Named);
      begin
         if Bx >= 0 then
            declare
               B : constant Boxed_Thing := C.Boxed (Natural (Bx));
            begin
               Edge := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cam).W or else B.Y1 + 1 >= F.Cams (Cam).H;
               Whole := not Edge;
            end;
         end if;
      end;
      return;
   end if;
   if Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam) then
      declare
         R : constant Picture.Region := World.Get (C.Wld, Cam, Natural (Slot)).R;
         Bx : constant Integer := Boxed_Index (C, Cam, R.Cu, R.Cv);
      begin
         if Bx >= 0 then
            declare
               B : constant Boxed_Thing := C.Boxed (Natural (Bx));
            begin
               Edge := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cam).W or else B.Y1 + 1 >= F.Cams (Cam).H;
               Whole := not Edge;
               Name := B.Name;
            end;
         end if;
      end;
   end if;
end Slot_Whole;
