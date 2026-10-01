--  The probability distributions the one significance test needs: the tail
--  of the Gaussian and the quantile of Student's t, which is what a sigma
--  measured from a few samples follows.
--
--  Both come from their defining functions, the regularized incomplete gamma
--  and beta functions, evaluated by their series and continued fractions to
--  working precision. Nothing here is chosen: every expansion runs until its
--  next term no longer changes the result in floating point, and a quantile
--  is bisected until its bracket cannot shrink any further.

package Driver.Distributions with Pure is

   function Gaussian_Two_Sided_Tail (Z : Real) return Real
     with Pre => Z >= 0.0;
   --  P (|X| > Z) for a standard Gaussian X.

   function Student_T_Two_Sided_Tail (T : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => T >= 0.0;
   --  P (|X| > T) for X following Student's t with that many degrees of freedom.

   function Student_T_Quantile (Two_Sided_Tail : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => Two_Sided_Tail > 0.0 and then Two_Sided_Tail <= 1.0;
   --  The T >= 0 with Student_T_Two_Sided_Tail (T, Degrees_Of_Freedom) = Two_Sided_Tail.

end Driver.Distributions;
