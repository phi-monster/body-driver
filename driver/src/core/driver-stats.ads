--  Sample statistics: running moments, robust spread, correlation and a
--  straight-line fit with its standard errors.

with Driver.Uncertain;

package Driver.Stats with Preelaborate is

   type Accumulator is private;
   --  Running mean and variance (Welford), numerically stable in one pass.

   procedure Add (A : in out Accumulator; X : Real);
   function Count (A : Accumulator) return Natural;
   function Mean (A : Accumulator) return Real
     with Pre => Count (A) > 0;
   function Variance (A : Accumulator) return Real
     with Pre => Count (A) > 1;
   --  The unbiased sample variance.

   function Mean_Estimate (A : Accumulator) return Driver.Uncertain.Estimate;
   --  The mean with the standard error of the mean and N - 1 degrees of
   --  freedom; unknown below two samples.

   function Median (X : Real_Array) return Real
     with Pre => X'Length > 0;
   --  Exact, X untouched: a sample of any size (its keys are copied to the heap).

   function Median_In_Place (X : in out Real_Array) return Real
     with Pre => X'Length > 0;
   --  Median's value, X's elements reordered: no copy, for an array that is the caller's scratch (the many medians
   --  of a few values each a search takes, whose heap copies were a tenth of the arm fit's stage 1, 10-20).

   function Robust_Sigma (X : Real_Array) return Real
     with Pre => X'Length > 0;
   --  The standard deviation implied by the median absolute deviation for
   --  Gaussian data; a minority of outliers does not move it.

   function Sigma_Of_Deviation (D : Real) return Real;
   --  The standard deviation Robust_Sigma gives for a median absolute deviation of D.

   function Median_Sigma (Sigma : Real; N : Positive) return Real;
   --  The standard error of the median of N Gaussian samples whose own
   --  standard deviation is Sigma: the root of pi over two times Sigma over
   --  the root of N (the median's asymptotic efficiency against the mean).

   function Mad_Degrees_Of_Freedom (N : Natural) return Natural;
   --  The degrees of freedom of a standard deviation told by the median
   --  absolute deviation of N Gaussian samples (its efficiency against the
   --  sample standard deviation); 0 below two samples, and never rounded
   --  down to none above.

   function Latest_Regime (X : Real_Array; Floor : Real := 0.0) return Positive
     with Pre => X'Length > 0 and then Floor >= 0.0, Post => Latest_Regime'Result in X'Range;
   --  The first index of the latest stationary stretch of the samples, in
   --  their order: the older half of the stretch and the newer have the same
   --  mean within their scatter (Robust_Sigma, with the degrees of freedom of
   --  a median absolute deviation), and no finer than Floor, a scatter the caller knows
   --  its samples cannot show less of: the rounding of counts to whole ones);
   --  while they differ, the older half is dropped. A stretch too short to split (fewer than two samples in its
   --  older half) is taken as it is. The readings' rest is measured on such a
   --  stretch (Driver.Robot.Channels), and so is an eye's (Driver.Robot.Lockin).

   function Correlation (X, Y : Real_Array) return Real
     with Pre => X'Length = Y'Length and then X'Length > 1;
   --  Pearson correlation; zero when either sample has no spread.

   type Line is record
      Slope, Intercept : Driver.Uncertain.Estimate;
      Residual_Sigma   : Real := Real'Last;
   end record;

   function Fit_Line (X, Y : Real_Array) return Line
     with Pre => X'Length = Y'Length and then X'Length > 2;
   --  Ordinary least squares Y = Slope * X + Intercept, with standard errors
   --  from the scatter of the residuals (N - 2 degrees of freedom).

private

   type Accumulator is record
      N    : Natural := 0;
      Mean : Real := 0.0;
      M2   : Real := 0.0;
   end record;

end Driver.Stats;
