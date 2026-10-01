with Driver.Conventions;
with Driver.Robot.Channels;
with Driver.Robot.Regression;

package body Driver.Robot.Stillness is

   function Group_Still (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (not Channels.Moving (M, G, Beat));

   function Eye_Still (M : Model; E : Eye_Id; Beat : Natural) return Boolean is
      S : Eye_Stream renames M.Eyes.Constant_Reference (E);
      N : constant Natural := Cells (S.Grid);
   begin
      if N = 0 or else Natural (S.Noise.Length) /= N or else Natural (S.Textured.Length) /= N
        or else Beat >= Natural (S.Measured.Length) or else not S.Measured (Beat)
      then
         return True;
      end if;
      declare
         Q    : Real := 0.0;
         Used : Natural := 0;
      begin
         for C in 0 .. N - 1 loop
            if S.Textured (C) then
               declare
                  Sigma : constant Real := S.Noise (C);
                  D2    : constant Real := S.Du (Beat * N + C) ** 2 + S.Dv (Beat * N + C) ** 2;
               begin
                  if Sigma > 0.0 then
                     Q := Q + D2 / (Sigma * Sigma);
                     Used := Used + 1;
                  elsif D2 > 0.0 then
                     --  A cell that is exactly still at rest moved.
                     return False;
                  end if;
               end;
            end if;
         end loop;
         --  Two components per cell.
         return Used = 0 or else Regression.Z_Of (Q, 2 * Used) <= Driver.Conventions.Z;
      end;
   end Eye_Still;

   function All_Still (M : Model; Beat : Natural) return Boolean is
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if not Group_Still (M, G, Beat) then
            return False;
         end if;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         if not Eye_Still (M, E, Beat) then
            return False;
         end if;
      end loop;
      return True;
   end All_Still;

end Driver.Robot.Stillness;
