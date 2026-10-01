with Driver.Conventions;
with Driver.Distributions;
with Driver.Stats;

package body Driver.Robot.Regression is


   Huber_K : constant := Driver.Conventions.Z;
   --  A residual beyond Z sigma is significantly not noise: from there on
   --  its weight falls inversely with its size.

   --  The pseudo-inverse of a symmetric positive semi-definite matrix, and
   --  its numerical rank: eigenvalues below the round-off of the largest
   --  one, accumulated over the dimension, count as zero.
   procedure Pseudo_Inverse (A : Real_Matrix; Inverse : out Real_Matrix; Rank : out Natural) is
      N       : constant Natural := A'Length (1);
      S       : constant Real_Matrix (1 .. N, 1 .. N) := A;
      Values  : Real_Vector (1 .. N);
      Vectors : Real_Matrix (1 .. N, 1 .. N);
      Largest : Real := 0.0;
   begin
      Inverse := [others => [others => 0.0]];
      Rank := 0;
      if N = 0 then
         return;
      end if;
      Eigensystem ((S + Transpose (S)) / 2.0, Values, Vectors);
      for V of Values loop
         Largest := Real'Max (Largest, abs V);
      end loop;
      if Largest = 0.0 then
         return;
      end if;
      declare
         Floor : constant Real := Largest * Real (N) * Real'Model_Epsilon;
         P     : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      begin
         for K in 1 .. N loop
            if Values (K) > Floor then
               Rank := Rank + 1;
               for I in 1 .. N loop
                  for J in 1 .. N loop
                     P (I, J) := P (I, J) + Vectors (I, K) * Vectors (J, K) / Values (K);
                  end loop;
               end loop;
            end if;
         end loop;
         Inverse := P;
      end;
   end Pseudo_Inverse;

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real) return Fit is
      N  : constant Natural := X'Length (1);
      P  : constant Natural := X'Length (2);
      R0 : constant Integer := X'First (1) - 1;
      C0 : constant Integer := X'First (2) - 1;
      Y0 : constant Integer := Y'First - 1;
      W  : Real_Array (1 .. N) := [others => 1.0];
      B  : Real_Array (1 .. P) := [others => 0.0];
      A  : Real_Matrix (1 .. P, 1 .. P);
      Residual : Real_Array (1 .. N);
      S  : Real := 0.0;
      Converged : Boolean := False;

      --  Weighted normal equations and their solution at the current weights.
      procedure Weighted_Solve is
         Rhs  : Real_Vector (1 .. P) := [others => 0.0];
         Inv  : Real_Matrix (1 .. P, 1 .. P);
         Rank : Natural;
      begin
         A := [others => [others => 0.0]];
         for I in 1 .. N loop
            if W (I) > 0.0 then
               for J in 1 .. P loop
                  declare
                     Xj : constant Real := W (I) * X (R0 + I, C0 + J);
                  begin
                     if Xj /= 0.0 then
                        Rhs (J) := Rhs (J) + Xj * Y (Y0 + I);
                        for K in J .. P loop
                           A (J, K) := A (J, K) + Xj * X (R0 + I, C0 + K);
                        end loop;
                     end if;
                  end;
               end loop;
            end if;
         end loop;
         for J in 1 .. P loop
            for K in 1 .. J - 1 loop
               A (J, K) := A (K, J);
            end loop;
         end loop;
         Pseudo_Inverse (A, Inv, Rank);
         declare
            Beta : constant Real_Vector := Inv * Rhs;
         begin
            for J in 1 .. P loop
               B (J) := Beta (J);
            end loop;
         end;
         for I in 1 .. N loop
            declare
               Predicted : Real := 0.0;
            begin
               for J in 1 .. P loop
                  Predicted := Predicted + X (R0 + I, C0 + J) * B (J);
               end loop;
               Residual (I) := Y (Y0 + I) - Predicted;
            end;
         end loop;
         S := Real'Max (Floor, Driver.Stats.Robust_Sigma (Residual));
      end Weighted_Solve;

      function Weight (R : Real) return Real is
      begin
         if S = 0.0 then
            --  More than half the residuals vanish exactly and there is no
            --  floor: only exact fits count.
            return (if R = 0.0 then 1.0 else 0.0);
         end if;
         declare
            U : constant Real := abs R / S;
         begin
            return (if U <= Huber_K then 1.0 else Huber_K / U);
         end;
      end Weight;

   begin
      Weighted_Solve;
      --  One pass per observation bounds the iterations; they stop as soon
      --  as the coefficients stop changing.
      for Pass in 1 .. N loop
         declare
            Before : constant Real_Array := B;
            Size, Moved : Real := 0.0;
         begin
            for I in 1 .. N loop
               W (I) := Weight (Residual (I));
            end loop;
            Weighted_Solve;
            for J in 1 .. P loop
               Size := Real'Max (Size, abs B (J));
               Moved := Real'Max (Moved, abs (B (J) - Before (J)));
            end loop;
            if Moved <= Driver.Conventions.Unchanged_Fraction * Size then
               Converged := True;
               exit;
            end if;
         end;
      end loop;
      return (Columns => P, Beta => B, Scale => S, Normal => A, Converged => Converged);
   end Solve;

   function Count_Significant (Count, Trials : Natural; Rate : Real) return Boolean is
   begin
      if Count = 0 then
         return False;
      end if;
      --  P (X >= k) for X binomial (n, p) is P (F < (n - k + 1) p / (k (1 - p)))
      --  for F with 2 k and 2 (n - k + 1) degrees of freedom.
      declare
         K : constant Real := Real (Count);
         N : constant Real := Real (Trials);
         Tail : constant Real :=
           1.0 - Driver.Distributions.F_Upper_Tail
                   ((N - K + 1.0) * Rate / (K * (1.0 - Rate)), 2 * Count, 2 * (Trials - Count + 1));
      begin
         return Tail < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      end;
   end Count_Significant;

   procedure Test_Block (F : Fit; First, Last : Positive; Statistic : out Real; Freedom : out Natural) is
      K    : constant Natural := Last - First + 1;
      Inv  : Real_Matrix (1 .. F.Columns, 1 .. F.Columns);
      Rank : Natural;
      Cov  : Real_Matrix (1 .. K, 1 .. K);
      Cinv : Real_Matrix (1 .. K, 1 .. K);
      Beta : Real_Vector (1 .. K);
   begin
      Pseudo_Inverse (F.Normal, Inv, Rank);
      for I in 1 .. K loop
         Beta (I) := F.Beta (First + I - 1);
         for J in 1 .. K loop
            Cov (I, J) := F.Scale * F.Scale * Inv (First + I - 1, First + J - 1);
         end loop;
      end loop;
      Pseudo_Inverse (Cov, Cinv, Freedom);
      Statistic := (if Freedom = 0 then 0.0 else Beta * (Cinv * Beta));
   end Test_Block;

end Driver.Robot.Regression;
