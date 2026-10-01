--  Sample statistics: running moments, robust spread, correlation and a
--  straight-line fit with its standard errors.

with Driver.Uncertain;

package Driver.Stats with Pure is

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

   function Robust_Sigma (X : Real_Array) return Real
     with Pre => X'Length > 0;
   --  The standard deviation implied by the median absolute deviation for
   --  Gaussian data; a minority of outliers does not move it.

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
