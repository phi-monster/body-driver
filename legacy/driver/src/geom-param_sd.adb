separate (Geom)
procedure Param_Sd (P : Param_Vec; N_Obs : Natural; Prior : Boolean; Steps : Param_Vec;
                    Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                    Sd : out Param_Vec) is
   Np : constant Natural := P'Length;
   Rows : constant Natural := (if Prior and then N_Obs > 0 then 2 * N_Obs - 1 else 2 * N_Obs);   --  真方程几条
   Rv : Big_Vec_Ptr := New_Vec (2 * N_Obs);
   Rp : Big_Vec_Ptr := New_Vec (2 * N_Obs);
   J : Big_Mat_Ptr := new Big_Mat (0 .. Integer (2 * N_Obs) - 1, 0 .. Integer (Np) - 1);
   A : array (0 .. Np - 1, 0 .. 2 * Np - 1) of Long_Float := [others => [others => 0.0]];   --  [JᵀJ | I],高斯-约当求逆
   procedure Fill_R (I : Natural; Du, Dv : Long_Float) is
   begin
      Rv (2 * I) := Du; Rv (2 * I + 1) := Dv;
   end Fill_R;
   Sum : Long_Float := 0.0;
   Sigma2 : Long_Float;
   R0 : Long_Float;
   Undet : array (0 .. Np - 1) of Boolean := [others => False];   --  没有信息的参数
begin
   Sd := [others => 0.0];
   if Np = 0 or else Rows <= Np then
      Sd := [others => Long_Float'Last];   --  方程不比未知数多:什么都定不了
      Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
      return;
   end if;
   Resid (P, R0, Fill_R'Access);
   for I in 0 .. 2 * N_Obs - 1 loop
      Sum := Sum + Rv (I) * Rv (I);
   end loop;
   Sigma2 := Sum / Long_Float (Rows - Np);
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
   for K in 0 .. Np - 1 loop
      for M in 0 .. Np - 1 loop
         for I in 0 .. 2 * N_Obs - 1 loop
            A (K, M) := A (K, M) + J (I, K) * J (I, M);
         end loop;
      end loop;
      A (K, Np + K) := 1.0;
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
            for M in 0 .. 2 * Np - 1 loop
               declare
                  T : constant Long_Float := A (Col, M);
               begin
                  A (Col, M) := A (Piv, M); A (Piv, M) := T;
               end;
            end loop;
         end if;
         if abs (A (Col, Col)) <= 1.0e-18 then
            --  这一列没有信息(比如一个点的观测全被踢成离群,它的三列全零):这个参数不确定度无穷,别的参数照算
            --  (G1O 2026-09-24 右眼:残差 0.64 px 的好解被"± inf"整个否掉)
            Undet (Col) := True;
         else
            declare
               D : constant Long_Float := A (Col, Col);
            begin
               for M in 0 .. 2 * Np - 1 loop
                  A (Col, M) := A (Col, M) / D;
               end loop;
            end;
            for Rw in 0 .. Np - 1 loop
               if Rw /= Col and then A (Rw, Col) /= 0.0 then
                  declare
                     Fct : constant Long_Float := A (Rw, Col);
                  begin
                     for M in 0 .. 2 * Np - 1 loop
                        A (Rw, M) := A (Rw, M) - Fct * A (Col, M);
                     end loop;
                  end;
               end if;
            end loop;
         end if;
      end;
   end loop;
   for K in 0 .. Np - 1 loop
      Sd (Sd'First + K) := (if Undet (K) then Long_Float'Last else Sqrt (Long_Float'Max (0.0, Sigma2 * A (K, Np + K))));
   end loop;
   Free_Vec (Rv); Free_Vec (Rp); Free_Mat (J);
end Param_Sd;
