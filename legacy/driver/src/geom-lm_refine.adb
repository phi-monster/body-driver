separate (Geom)
procedure LM_Refine (P : in out Param_Vec; N_Obs : Natural; Steps : Param_Vec; Iters : Positive;
                     Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                     Cur : in out Long_Float) is
   Lm_Loosen : constant := 3;
   Lm_Tighten : constant := 10;
   Np : constant Natural := P'Length;
   Lam : Long_Float := 1.0e-3;   --  阻尼(无量纲)
   Rv : Big_Vec_Ptr := New_Vec (2 * N_Obs);
   Rp : Big_Vec_Ptr := New_Vec (2 * N_Obs);
   J : Big_Mat_Ptr := new Big_Mat (0 .. Integer (2 * N_Obs) - 1, 0 .. Integer (Np) - 1);
   procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
   begin
      Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
   end Fill_R;
begin
   for It in 1 .. Iters loop
      declare
         R0 : Long_Float;
      begin
         Resid (P, R0, Fill_R'Access);
      end;
      for K in 0 .. Np - 1 loop
         declare
            Pp : Param_Vec := P;
            procedure Fill_P (I : Natural; Du, Dv : Long_Float) is
            begin
               Rp (2 * I) := Du; Rp (2 * I + 1) := Dv;
            end Fill_P;
            Dummy : Long_Float;
            H : constant Long_Float := Steps (Steps'First + K);
         begin
            Pp (P'First + K) := Pp (P'First + K) + H;
            Resid (Pp, Dummy, Fill_P'Access);
            for I in 0 .. 2 * N_Obs - 1 loop
               J (I, K) := (Rp (I) - Rv (I)) / H;
            end loop;
         end;
      end loop;
      declare
         A : array (0 .. Np - 1, 0 .. Np - 1) of Long_Float := [others => [others => 0.0]];
         B : array (0 .. Np - 1) of Long_Float := [others => 0.0];
         Dlt : array (0 .. Np - 1) of Long_Float := [others => 0.0];
      begin
         for K in 0 .. Np - 1 loop
            for M in 0 .. Np - 1 loop
               for I in 0 .. 2 * N_Obs - 1 loop
                  A (K, M) := A (K, M) + J (I, K) * J (I, M);
               end loop;
            end loop;
            for I in 0 .. 2 * N_Obs - 1 loop
               B (K) := B (K) - J (I, K) * Rv (I);
            end loop;
         end loop;
         for K in 0 .. Np - 1 loop
            A (K, K) := A (K, K) * (1.0 + Lam) + 1.0e-12;
         end loop;
         for Col in 0 .. Np - 1 loop
            declare
               Piv : Natural := Col;
            begin
               for Rw in Col + 1 .. Np - 1 loop
                  if abs (A (Rw, Col)) > abs (A (Piv, Col)) then
                     Piv := Rw;
                  end if;
               end loop;
               if Piv /= Col then
                  for M in 0 .. Np - 1 loop
                     declare
                        T : constant Long_Float := A (Col, M);
                     begin
                        A (Col, M) := A (Piv, M); A (Piv, M) := T;
                     end;
                  end loop;
                  declare
                     T : constant Long_Float := B (Col);
                  begin
                     B (Col) := B (Piv); B (Piv) := T;
                  end;
               end if;
               if abs (A (Col, Col)) > 1.0e-18 then
                  for Rw in 0 .. Np - 1 loop
                     if Rw /= Col then
                        declare
                           Fct : constant Long_Float := A (Rw, Col) / A (Col, Col);
                        begin
                           for M in 0 .. Np - 1 loop
                              A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                           end loop;
                           B (Rw) := B (Rw) - Fct * B (Col);
                        end;
                     end if;
                  end loop;
               end if;
            end;
         end loop;
         for K in 0 .. Np - 1 loop
            Dlt (K) := (if abs (A (K, K)) > 1.0e-18 then B (K) / A (K, K) else 0.0);
         end loop;
         declare
            Pn : Param_Vec := P;
            Cn : Long_Float;
         begin
            for K in 0 .. Np - 1 loop
               Pn (P'First + K) := Pn (P'First + K) + Dlt (K);
            end loop;
            Resid (Pn, Cn, null);
            if Cn < Cur then
               P := Pn; Cur := Cn; Lam := Lam / Long_Float (Lm_Loosen);
            else
               Lam := Lam * Long_Float (Lm_Tighten);
            end if;
         end;
      end;
      exit when Lam > 1.0e6;
   end loop;
   Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
end LM_Refine;
