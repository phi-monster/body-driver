--  Robust linear regression with a significance test for blocks of
--  coefficients.
--
--  The fit is iteratively reweighted least squares with Huber's weights to
--  convergence, cut at the significance level Z: a convex loss, so where it starts does not matter, and an
--  observation far outside the model (a motion too large for a
--  linearization) keeps only a weight inversely proportional to its
--  residual. A redescending loss would drop such observations entirely,
--  but it also drops sound ones whenever the measurement noise is far
--  below the model's own small systematic error, which is the case for a
--  rendered image. The scale is re-measured every iteration from the median
--  absolute residual, never below the given floor. Columns that are
--  combinations of others (channels that always moved together) are
--  handled by a pseudo-inverse, and a block test then tests what the data
--  can tell about that block and reports its rank as degrees of freedom.

private package Driver.Robot.Regression is

   use Driver.Numerics.Arrays;

   type Fit (Columns : Natural) is record
      Beta      : Real_Array (1 .. Columns);
      Scale     : Real;                                   --  robust sigma of one residual
      Normal    : Real_Matrix (1 .. Columns, 1 .. Columns);  --  X' W X at the final weights
      Inverse   : Real_Matrix (1 .. Columns, 1 .. Columns);  --  its pseudo-inverse, as the final pass solved with it
      Converged : Boolean;
      Passes    : Natural := 0;                              --  reweighting rounds the fit took, for the log of its cost
   end record;

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real) return Fit
     with Pre => X'Length (1) = Y'Length and then X'Length (2) > 0 and then Floor >= 0.0;
   --  X has one row per observation and one column per regressor. Floor is
   --  the least sigma a residual can have (the resolution of the measured
   --  quantity): exact data would otherwise give a zero scale, and every
   --  observation that is not fitted exactly would lose all its weight.

   type Count_Array is array (Positive range <>) of Natural;

   type Start (Columns : Natural) is record
      Normal  : Real_Matrix (1 .. Columns, 1 .. Columns);
      Inverse : Real_Matrix (1 .. Columns, 1 .. Columns);
      Support : Count_Array (1 .. Columns);   --  the observations each column is not nought at
   end record;
   --  A fit's first pass, at equal weights: its normal equations and their pseudo-inverse are X's alone, the same for
   --  every Y regressed on X, and made once for all of them. The lock-in regresses each cell's two displacements on the
   --  same pushes, and most cells over the same rows: two pseudo-inverses a cell, a humanoid's 31 by 31 (h401 on the
   --  big machine, 10-22: 255 cells an eye, some 600 reweighting passes beside).

   function Start_Of (X : Real_Matrix) return Start
     with Pre => X'Length (2) > 0, Post => Start_Of'Result.Columns = X'Length (2);

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real; From : Start) return Fit
     with Pre => X'Length (1) = Y'Length and then X'Length (2) > 0 and then Floor >= 0.0
                 and then From.Columns = X'Length (2);
   --  Solve, its first pass From Start_Of (X): the same fit to the bit.

   function Count_Tail (Count, Trials : Natural; Rate : Real) return Real
     with Pre => Count <= Trials and then Rate > 0.0 and then Rate < 1.0;
   --  The exact binomial probability of at least Count events in that many independent trials at Rate per trial
   --  (one for none): the chance that chance alone made that many.

   function Count_Significant (Count, Trials : Natural; Rate : Real; Tests : Positive := 1) return Boolean
     with Pre => Count <= Trials and then Rate > 0.0 and then Rate < 1.0;
   --  Count events in that many independent trials are more than chance at
   --  Rate per trial explains: the exact binomial probability of at least
   --  Count is below the tail Z has for a Gaussian (through its relation to
   --  Fisher's F), over Tests when this count is the best of that many
   --  (Bonferroni). Used for how many tests of a family alarmed.

   function Explained_Nonnegative (X : Real_Matrix; Y : Real_Array; Used : out Natural) return Real
     with Pre => X'Length (1) = Y'Length;
   --  The fraction of the variance of Y about its mean explained by the
   --  least-squares fit Y = a + X b with every b >= 0 (Lawson and Hanson's
   --  active set, on X and Y centred so the intercept a is free), and how
   --  many of the b came out positive: R squared, for an F test of a fit in
   --  which every regressor can only add to Y.

   function Explained_Nonnegative
     (Gram : Real_Matrix; Cross : Real_Vector; Total : Real; Used : out Natural) return Real
     with Pre => Gram'Length (1) = Cross'Length and then Gram'Length (2) = Cross'Length;
   --  The same from the fit's sums alone: Gram the columns' products with each other about their means, Cross their
   --  products with Y about the means, Total Y's squares about its mean, all scaled alike (each times the rows, say:
   --  the coefficients and the fraction explained do not change). The active set reads the data only through these
   --  (Explained_Nonnegative above makes them from the rows on every pass).

   function Covariance_Of (F : Fit) return Real_Matrix
     with Post => Covariance_Of'Result'Length (1) = F.Columns and then Covariance_Of'Result'Length (2) = F.Columns;
   --  The coefficients' covariance: Scale ** 2 times the pseudo-inverse of Normal (Inverse, as the final pass made it).
   --  Made once for a fit and given to each of its tests below: each made it anew, and the lock-in tests every group and
   --  every column of each cell's two fits (a humanoid's 30 channels: some 64 pseudo-inverses of 31 by 31 a cell, 510
   --  cells an estimate).

   function Coefficient_Variances (F : Fit) return Real_Array;
   function Coefficient_Variances (F : Fit; Cov : Real_Matrix) return Real_Array
     with Pre => Cov'Length (1) = F.Columns and then Cov'Length (2) = F.Columns;
   --  The variance of each coefficient: the diagonal of the covariance (Covariance_Of, or Cov made by it).

   procedure Test_Block (F : Fit; First, Last : Positive; Statistic : out Real; Freedom : out Natural)
     with Pre => First <= Last and then Last <= F.Columns;
   procedure Test_Block
     (F : Fit; Cov : Real_Matrix; First, Last : Positive; Statistic : out Real; Freedom : out Natural)
     with Pre => First <= Last and then Last <= F.Columns
                 and then Cov'Length (1) = F.Columns and then Cov'Length (2) = F.Columns;
   --  The Wald statistic of coefficients First .. Last against zero, with
   --  the covariance Scale ** 2 times the pseudo-inverse of Normal, and its
   --  degrees of freedom (the numerical rank of that block). Statistics of
   --  independent responses add, and so do their degrees of freedom.

end Driver.Robot.Regression;
