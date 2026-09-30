separate (Act.Run_Segment)
function Watch_Things (Fr : Plug.Frame) return Boolean is
   Regs : Picture.Regions;
   Have : Boolean := False;
begin
   for P of Pts loop
      if P.Kind = Thing_Pt then
         if not Have then
            Regs := Cut_Things (C, Fr, Cam);
            Have := True;
         end if;
         declare
            Found : Boolean := False;
            Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;   --  和 Retrack 同一个认领半径(比例,无量纲)
         begin
            for R of Regs loop
               if R.Count * 3 >= P.Count and then R.Count <= P.Count * 3 and then Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Tol then
                  Found := True;
                  if R.Cu < Track_Win or else R.Cu > 1.0 - Track_Win or else R.Cv < Track_Win or else R.Cv > 1.0 - Track_Win then
                     Note.Halted := True;
                     return True;
                  end if;
               end if;
            end loop;
            if not Found then
               Note.Halted := True;
               return True;
            end if;
         end;
      end if;
   end loop;
   return False;
end Watch_Things;
