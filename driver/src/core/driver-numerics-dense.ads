--  Dense linear algebra beyond what the standard real arrays provide.
--
--  The standard package already gives products, transposes, Solve, Inverse
--  and the symmetric Eigensystem; this adds the factorizations the fitting
--  code needs: Cholesky for normal equations, and Householder QR for least
--  squares that stays accurate when the normal equations would not.

package Driver.Numerics.Dense with Preelaborate is

   procedure Cholesky (A : Real_Matrix; L : out Real_Matrix; Positive_Definite : out Boolean)
     with Pre => A'Length (1) = A'Length (2)
                 and then L'Length (1) = A'Length (1) and then L'Length (2) = A'Length (2);
   --  A = L * Transpose (L) with L lower triangular. Positive_Definite is
   --  False, and L meaningless, when a pivot is not strictly positive.

   function Cholesky_Solve (L : Real_Matrix; B : Real_Vector) return Real_Vector
     with Pre => L'Length (1) = L'Length (2) and then B'Length = L'Length (1);
   --  Solves L * Transpose (L) * X = B for a factor from Cholesky.

   procedure Least_Squares
     (A         : Real_Matrix;
      B         : Real_Vector;
      X         : out Real_Vector;
      Full_Rank : out Boolean)
     with Pre => B'Length = A'Length (1) and then X'Length = A'Length (2)
                 and then A'Length (1) >= A'Length (2);
   --  Minimizes |A X - B| by Householder QR. Full_Rank is False when a
   --  column is, to working precision, a combination of the others; X is
   --  then not unique and is left at zero.

end Driver.Numerics.Dense;
