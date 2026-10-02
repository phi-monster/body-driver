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
      Converged : Boolean;
   end record;

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real) return Fit
     with Pre => X'Length (1) = Y'Length and then X'Length (2) > 0 and then Floor >= 0.0;
   --  X has one row per observation and one column per regressor. Floor is
   --  the least sigma a residual can have (the resolution of the measured
   --  quantity): exact data would otherwise give a zero scale, and every
   --  observation that is not fitted exactly would lose all its weight.

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

   function Coefficient_Variances (F : Fit) return Real_Array;
   --  The variance of each coefficient: Scale ** 2 times the diagonal of the
   --  pseudo-inverse of Normal.

   procedure Test_Block (F : Fit; First, Last : Positive; Statistic : out Real; Freedom : out Natural)
     with Pre => First <= Last and then Last <= F.Columns;
   --  The Wald statistic of coefficients First .. Last against zero, with
   --  the covariance Scale ** 2 times the pseudo-inverse of Normal, and its
   --  degrees of freedom (the numerical rank of that block). Statistics of
   --  independent responses add, and so do their degrees of freedom.

end Driver.Robot.Regression;
