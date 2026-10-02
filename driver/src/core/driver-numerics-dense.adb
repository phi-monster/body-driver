with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;

package body Driver.Numerics.Dense is

   use Ada.Numerics.Long_Elementary_Functions;

   procedure Cholesky (A : Real_Matrix; L : out Real_Matrix; Positive_Definite : out Boolean) is
      N   : constant Natural := A'Length (1);
      Src : constant Real_Matrix (1 .. N, 1 .. N) := A;
      F   : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      Sum : Real;
   begin
      L := [others => [others => 0.0]];
      Positive_Definite := False;
      for J in 1 .. N loop
         Sum := Src (J, J);
         for K in 1 .. J - 1 loop
            Sum := Sum - F (J, K) ** 2;
         end loop;
         if not (Sum > 0.0) then
            return;
         end if;
         F (J, J) := Sqrt (Sum);
         for I in J + 1 .. N loop
            Sum := Src (I, J);
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
      N  : constant Natural := L'Length (1);
      F  : constant Real_Matrix (1 .. N, 1 .. N) := L;
      Rh : constant Real_Vector (1 .. N) := B;
      Y  : Real_Vector (1 .. N) := [others => 0.0];
      X  : Real_Vector (1 .. N) := [others => 0.0];
      Sum : Real;
   begin
      for I in 1 .. N loop
         Sum := Rh (I);
         for K in 1 .. I - 1 loop
            Sum := Sum - F (I, K) * Y (K);
         end loop;
         Y (I) := Sum / F (I, I);
      end loop;
      for I in reverse 1 .. N loop
         Sum := Y (I);
         for K in I + 1 .. N loop
            Sum := Sum - F (K, I) * X (K);
         end loop;
         X (I) := Sum / F (I, I);
      end loop;
      return Result : Real_Vector (B'Range) do
         Result := X;
      end return;
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

end Driver.Numerics.Dense;
