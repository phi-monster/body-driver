separate (Geom)
function Solve3 (A : M3; B : V3) return V3 is
   M : M3 := A;
   R : V3 := B;
begin
   for Col in 0 .. 2 loop
      declare
         Piv : Natural := Col;
      begin
         for Rw in Col + 1 .. 2 loop
            if abs (M (Rw, Col)) > abs (M (Piv, Col)) then
               Piv := Rw;
            end if;
         end loop;
         if Piv /= Col then
            for J in 0 .. 2 loop
               declare
                  T : constant Long_Float := M (Col, J);
               begin
                  M (Col, J) := M (Piv, J); M (Piv, J) := T;
               end;
            end loop;
            declare
               T : constant Long_Float := R (Col);
            begin
               R (Col) := R (Piv); R (Piv) := T;
            end;
         end if;
         if abs (M (Col, Col)) < 1.0e-15 then
            return [0.0, 0.0, 0.0];
         end if;
         for Rw in 0 .. 2 loop
            if Rw /= Col then
               declare
                  Fct : constant Long_Float := M (Rw, Col) / M (Col, Col);
               begin
                  for J in 0 .. 2 loop
                     M (Rw, J) := M (Rw, J) - Fct * M (Col, J);
                  end loop;
                  R (Rw) := R (Rw) - Fct * R (Col);
               end;
            end if;
         end loop;
      end;
   end loop;
   return [R (0) / M (0, 0), R (1) / M (1, 1), R (2) / M (2, 2)];
end Solve3;
