with Ada.Unchecked_Deallocation;
with Driver.Roots;

package body Driver.Numerics.Dense is

   --  The processor's own square root, inline (Driver.Roots), where the library's was a call and two tests.
   function Sqrt (X : Real) return Real renames Driver.Roots.Sqrt;


   procedure Cholesky (A : Real_Matrix; L : out Real_Matrix; Positive_Definite : out Boolean) is
      --  Every index below runs over the square's own N, the contract's: no check after it can fail. A is read in
      --  place (it was copied to bounds from one first), each index its first plus the place, less one, in that order:
      --  an aggregate's first is Integer'First, and its first less one overflows (core.cholesky, 10-28).
      pragma Suppress (All_Checks);
      N   : constant Natural := A'Length (1);
      I0  : constant Integer := A'First (1);
      J0  : constant Integer := A'First (2);
      F   : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      Sum : Real;
   begin
      L := [others => [others => 0.0]];
      Positive_Definite := False;
      for J in 1 .. N loop
         Sum := A (I0 + J - 1, J0 + J - 1);
         for K in 1 .. J - 1 loop
            Sum := Sum - F (J, K) ** 2;
         end loop;
         if not (Sum > 0.0) then
            return;
         end if;
         F (J, J) := Sqrt (Sum);
         for I in J + 1 .. N loop
            Sum := A (I0 + I - 1, J0 + J - 1);
            for K in 1 .. J - 1 loop
               Sum := Sum - F (I, K) * F (J, K);
            end loop;
            F (I, J) := Sum / F (J, J);
         end loop;
      end loop;
      L := F;
      Positive_Definite := True;
   end Cholesky;

   function Cholesky_Solve (L : Real_Matrix; B : Real_Vector) return Real_Vector is
      --  The contract's square and vector: no check below can fail. The factor is read in place (it was copied to
      --  bounds from one at every solve), each index its first plus the place, less one, in that order (Cholesky).
      pragma Suppress (All_Checks);
      N  : constant Natural := L'Length (1);
      I0 : constant Integer := L'First (1);
      J0 : constant Integer := L'First (2);
      Rh : constant Real_Vector (1 .. N) := B;
      Y  : Real_Vector (1 .. N) := [others => 0.0];
      X  : Real_Vector (1 .. N) := [others => 0.0];
      Sum : Real;
   begin
      for I in 1 .. N loop
         Sum := Rh (I);
         for K in 1 .. I - 1 loop
            Sum := Sum - L (I0 + I - 1, J0 + K - 1) * Y (K);
         end loop;
         Y (I) := Sum / L (I0 + I - 1, J0 + I - 1);
      end loop;
      for I in reverse 1 .. N loop
         Sum := Y (I);
         for K in I + 1 .. N loop
            Sum := Sum - L (I0 + K - 1, J0 + I - 1) * X (K);
         end loop;
         X (I) := Sum / L (I0 + I - 1, J0 + I - 1);
      end loop;
      return Result : Real_Vector (B'Range) do
         Result := X;
      end return;
   end Cholesky_Solve;

   procedure Cholesky_Solve (L : Real_Matrix; B : Real_Vector; X : out Real_Vector) is
      --  The contract's square and vectors: no check below can fail (the function's, written into X).
      pragma Suppress (All_Checks);
      N  : constant Natural := L'Length (1);
      I0 : constant Integer := L'First (1);
      J0 : constant Integer := L'First (2);
      B0 : constant Integer := B'First;
      X0 : constant Integer := X'First;
      Y  : Real_Vector (1 .. N);
      Sum : Real;
   begin
      for I in 1 .. N loop
         Sum := B (B0 + I - 1);
         for K in 1 .. I - 1 loop
            Sum := Sum - L (I0 + I - 1, J0 + K - 1) * Y (K);
         end loop;
         Y (I) := Sum / L (I0 + I - 1, J0 + I - 1);
      end loop;
      for I in reverse 1 .. N loop
         Sum := Y (I);
         for K in I + 1 .. N loop
            Sum := Sum - L (I0 + K - 1, J0 + I - 1) * X (X0 + K - 1);
         end loop;
         X (X0 + I - 1) := Sum / L (I0 + I - 1, J0 + I - 1);
      end loop;
   end Cholesky_Solve;

   type Matrix_Access is access Real_Matrix;
   type Vector_Access is access Real_Vector;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Vector, Vector_Access);

   procedure Least_Squares
     (A         : Real_Matrix;
      B         : Real_Vector;
      X         : out Real_Vector;
      Full_Rank : out Boolean)
   is
      M : constant Natural := A'Length (1);
      N : constant Natural := A'Length (2);
      --  The working copies are on the heap: a design matrix has a row per
      --  observation (presses, sightings), which can outgrow a task's stack.
      Q_Copy : Matrix_Access := new Real_Matrix (1 .. M, 1 .. N);
      Y_Copy : Vector_Access := new Real_Vector (1 .. M);

      procedure Solve (Q : in out Real_Matrix; Y : in out Real_Vector) is
         Diagonal : Real_Vector (1 .. N);
         Scale    : Real := 0.0;
      begin
         X := [others => 0.0];
         Full_Rank := False;
         for I in 1 .. M loop
            for J in 1 .. N loop
               Scale := Real'Max (Scale, abs Q (I, J));
            end loop;
         end loop;
         if Scale = 0.0 then
            return;
         end if;
         --  Householder reflections turn A into R, applied to B on the way.
         for K in 1 .. N loop
            declare
               Norm_K : Real := 0.0;
            begin
               for I in K .. M loop
                  Norm_K := Norm_K + Q (I, K) ** 2;
               end loop;
               Norm_K := Sqrt (Norm_K);
               --  A column that has nothing left after removing the previous
               --  ones (relative to the matrix scale) makes the problem rank
               --  deficient; the bound is the round-off of the reflections.
               if Norm_K <= Real'Model_Epsilon * Scale * Real (M) then
                  return;
               end if;
               if Q (K, K) > 0.0 then
                  Norm_K := -Norm_K;
               end if;
               for I in K .. M loop
                  Q (I, K) := Q (I, K) / (-Norm_K);
               end loop;
               Q (K, K) := Q (K, K) + 1.0;
               for J in K + 1 .. N loop
                  declare
                     S : Real := 0.0;
                  begin
                     for I in K .. M loop
                        S := S + Q (I, K) * Q (I, J);
                     end loop;
                     S := -S / Q (K, K);
                     for I in K .. M loop
                        Q (I, J) := Q (I, J) + S * Q (I, K);
                     end loop;
                  end;
               end loop;
               declare
                  S : Real := 0.0;
               begin
                  for I in K .. M loop
                     S := S + Q (I, K) * Y (I);
                  end loop;
                  S := -S / Q (K, K);
                  for I in K .. M loop
                     Y (I) := Y (I) + S * Q (I, K);
                  end loop;
               end;
               Diagonal (K) := Norm_K;
            end;
         end loop;
         --  Back substitution with R (upper part of Q, diagonal kept aside).
         declare
            Sol : Real_Vector (1 .. N) := [others => 0.0];
         begin
            for K in reverse 1 .. N loop
               declare
                  S : Real := Y (K);
               begin
                  for J in K + 1 .. N loop
                     S := S - Q (K, J) * Sol (J);
                  end loop;
                  Sol (K) := S / Diagonal (K);
               end;
            end loop;
            X := Sol;
         end;
         Full_Rank := True;
      end Solve;

   begin
      Q_Copy.all := A;
      Y_Copy.all := B;
      Solve (Q_Copy.all, Y_Copy.all);
      Free (Q_Copy);
      Free (Y_Copy);
   exception
      when others =>
         Free (Q_Copy);
         Free (Y_Copy);
         raise;
   end Least_Squares;

   procedure Small_Eigensystem (A : Real_Matrix; Values : out Real_Vector; Vectors : out Real_Matrix) is
      --  Every index runs over the contract's own N: no check below can fail.
      pragma Suppress (All_Checks);
      N : constant Natural := A'Length (1);
      D : Real_Matrix (1 .. N, 1 .. N);
      V : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
   begin
      for P in 1 .. N loop
         V (P, P) := 1.0;
         for Q in 1 .. N loop
            D (P, Q) := A (A'First (1) + Integer'Min (P, Q) - 1, A'First (2) + Integer'Max (P, Q) - 1);
         end loop;
      end loop;
      for Sweep in 1 .. Real'Machine_Mantissa loop
         exit when (for all P in 1 .. N - 1 => (for all Q in P + 1 .. N => D (P, Q) = 0.0));
         for P in 1 .. N - 1 loop
            for Q in P + 1 .. N loop
               if D (P, Q) /= 0.0 then
                  if abs D (P, P) + abs D (P, Q) = abs D (P, P) and then abs D (Q, Q) + abs D (P, Q) = abs D (Q, Q) then
                     D (P, Q) := 0.0;
                     D (Q, P) := 0.0;
                  else
                     declare
                        H     : constant Real := D (Q, Q) - D (P, P);
                        T     : Real;
                        C, S  : Real;
                        Tau   : Real;
                        Apq   : constant Real := D (P, Q);
                     begin
                        if abs H + abs Apq = abs H then
                           T := Apq / H;
                        else
                           declare
                              Theta : constant Real := H / (2.0 * Apq);
                           begin
                              T := 1.0 / (abs Theta + Sqrt (Theta * Theta + 1.0));
                              if Theta < 0.0 then
                                 T := -T;
                              end if;
                           end;
                        end if;
                        C := 1.0 / Sqrt (T * T + 1.0);
                        S := T * C;
                        Tau := S / (1.0 + C);
                        D (P, P) := D (P, P) - T * Apq;
                        D (Q, Q) := D (Q, Q) + T * Apq;
                        D (P, Q) := 0.0;
                        D (Q, P) := 0.0;
                        for R in 1 .. N loop
                           if R /= P and then R /= Q then
                              declare
                                 Grp : constant Real := D (R, P);
                                 Grq : constant Real := D (R, Q);
                              begin
                                 D (R, P) := Grp - S * (Grq + Tau * Grp);
                                 D (R, Q) := Grq + S * (Grp - Tau * Grq);
                                 D (P, R) := D (R, P);
                                 D (Q, R) := D (R, Q);
                              end;
                           end if;
                        end loop;
                        for R in 1 .. N loop
                           declare
                              Vrp : constant Real := V (R, P);
                              Vrq : constant Real := V (R, Q);
                           begin
                              V (R, P) := Vrp - S * (Vrq + Tau * Vrp);
                              V (R, Q) := Vrq + S * (Vrp - Tau * Vrq);
                           end;
                        end loop;
                     end;
                  end if;
               end if;
            end loop;
         end loop;
      end loop;
      for P in 1 .. N loop
         Values (Values'First + P - 1) := D (P, P);
         for Q in 1 .. N loop
            Vectors (Vectors'First (1) + Q - 1, Vectors'First (2) + P - 1) := V (Q, P);
         end loop;
      end loop;
   end Small_Eigensystem;

end Driver.Numerics.Dense;
