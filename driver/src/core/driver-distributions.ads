--  The probability distributions the one significance test needs: the tail
--  of the Gaussian, Student's t (what a difference over a sigma measured from
--  a few samples follows), the chi-square (the squared length of a Gaussian
--  difference of several dimensions) and Fisher's F (that length over an
--  estimated sigma).
--
--  All come from their defining functions, the regularized incomplete gamma
--  and beta functions, evaluated by their series and continued fractions to
--  working precision. Nothing here is chosen: every expansion runs until its
--  next term no longer changes the result in floating point, and a quantile
--  is bisected until its bracket cannot shrink any further.

package Driver.Distributions with Pure is

   function Gaussian_Two_Sided_Tail (Z : Real) return Real
     with Pre => Z >= 0.0;
   --  P (|X| > Z) for a standard Gaussian X.

   function Gaussian_Two_Sided_Quantile (Two_Sided_Tail : Real) return Real
     with Pre => Two_Sided_Tail > 0.0 and then Two_Sided_Tail <= 1.0;
   --  The Z >= 0 with Gaussian_Two_Sided_Tail (Z) = Two_Sided_Tail.

   function Student_T_Two_Sided_Tail (T : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => T >= 0.0;
   --  P (|X| > T) for X following Student's t with that many degrees of freedom.

   function Student_T_Quantile (Two_Sided_Tail : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => Two_Sided_Tail > 0.0 and then Two_Sided_Tail <= 1.0;
   --  The T >= 0 with Student_T_Two_Sided_Tail (T, Degrees_Of_Freedom) = Two_Sided_Tail.

   function Chi_Square_Upper_Tail (X : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => X >= 0.0;
   --  P (Q > X) for Q chi-square with that many degrees of freedom.

   function Chi_Square_Quantile (Upper_Tail : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => Upper_Tail > 0.0 and then Upper_Tail <= 1.0;
   --  The X >= 0 with Chi_Square_Upper_Tail (X, Degrees_Of_Freedom) = Upper_Tail.

   function Chi_Square_Deviate (X : Real; Degrees_Of_Freedom : Positive) return Real
     with Pre => X >= 0.0;
   --  The Gaussian deviate as rare as X is for that chi-square: the Z with
   --  Gaussian_Two_Sided_Tail (Z) = Chi_Square_Upper_Tail (X); Real'Last when
   --  the tail is below the smallest representable probability. A chi-square
   --  statistic is significant exactly when its deviate exceeds Z.

   function F_Upper_Tail (F : Real; Numerator, Denominator : Positive) return Real
     with Pre => F >= 0.0;
   --  P (X > F) for X following Fisher's F with those degrees of freedom.

   function F_Quantile (Upper_Tail : Real; Numerator, Denominator : Positive) return Real
     with Pre => Upper_Tail > 0.0 and then Upper_Tail <= 1.0;
   --  The F >= 0 with F_Upper_Tail (F, Numerator, Denominator) = Upper_Tail.

end Driver.Distributions;
