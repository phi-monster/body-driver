--  Measured quantities with their uncertainty, and the driver's single test
--  of significance.
--
--  Every gate in the driver has the form "is this difference larger than Z
--  standard deviations of its own measured noise" (Driver.Conventions.Z),
--  that is, does noise alone produce a difference this large less often than
--  it exceeds Z for a Gaussian. A difference of several dimensions is tested
--  by its length in units of its own covariance against the threshold with
--  that same tail probability for that many dimensions: measured along the
--  direction it happened to take, isotropic noise in three dimensions would
--  pass Z ten times as often.
--
--  A sigma measured from a few samples is itself uncertain: the difference
--  over it follows Student's t, whose tails are heavier than the Gaussian's.
--  An estimate therefore carries how many degrees of freedom its sigma rests
--  on, and the test widens Z to the t quantile with the same tail
--  probability, so a gate raises false alarms equally often whether its
--  sigma came from three samples or was known.

with Driver.Numerics;

package Driver.Uncertain with Pure is

   use Driver.Numerics;

   type Estimate is record
      Value              : Real := 0.0;
      Sigma              : Real := Real'Last;
      Degrees_Of_Freedom : Natural := 0;
   end record;
   --  Sigma is the standard deviation of Value; Real'Last means unknown.
   --  Degrees_Of_Freedom is how many the sigma was estimated with; 0 means
   --  the sigma is known, or rests on so many samples that it may as well be.

   Unknown : constant Estimate := (Value => 0.0, Sigma => Real'Last, Degrees_Of_Freedom => 0);

   function Known (E : Estimate) return Boolean is (E.Sigma < Real'Last);

   function Significant (Difference, Sigma : Real; Degrees_Of_Freedom : Natural := 0) return Boolean;
   --  abs Difference > T * Sigma, with T = Z when the sigma is known and the
   --  Student t quantile at the tail probability Z has for a Gaussian when it
   --  rests on that many degrees of freedom. A zero sigma makes any nonzero
   --  difference significant; an unknown sigma makes none significant.

   function Difference (A, B : Estimate) return Estimate;
   --  A - B for independent estimates: the sigmas add in quadrature and the
   --  degrees of freedom combine by Welch and Satterthwaite (rounded down).

   function Significant (A, B : Estimate) return Boolean;
   --  Significant (Difference (A, B)) against zero.

   type Gate is private;
   --  The test above with its threshold worked out once, for code that tests
   --  many differences with the same degrees of freedom (every pixel of an
   --  image, say). It is the same rule, not another one.

   function Scalar_Gate (Degrees_Of_Freedom : Natural := 0) return Gate;
   --  Significant (G, D, S) = Significant (D, S, Degrees_Of_Freedom).

   function Vector_Gate (Dimensions : Positive; Degrees_Of_Freedom : Natural := 0) return Gate;
   --  For the length of a difference of that many dimensions whose every
   --  component has the given sigma. Its length over sigma is not Gaussian:
   --  it is a chi (or, with an estimated sigma, the square root of Dimensions
   --  times an F), and the threshold is taken at the same tail probability Z
   --  has for a Gaussian, so a two-dimensional displacement alarms as rarely
   --  as a scalar.

   function Significant (G : Gate; Difference, Sigma : Real) return Boolean;
   --  abs Difference > threshold * Sigma, with the zero and unknown sigmas of
   --  the scalar rule.

   function Threshold (G : Gate) return Real;
   --  The multiple of sigma beyond which a difference is significant.

   type Point_Estimate is record
      Mean       : Vec3 := Zero3;
      Covariance : Mat3 := [others => [others => Real'Last]];
   end record;

   function Known (P : Point_Estimate) return Boolean is (P.Covariance (1, 1) < Real'Last);

   function Sigma_Along (Covariance : Mat3; Direction : Vec3) return Real;
   --  The standard deviation of the component along a unit direction.

   function Significant (A, B : Point_Estimate) return Boolean;
   --  Whether two independent points are apart: the Mahalanobis length of
   --  their separation under the sum of their covariances, against
   --  Vector_Gate (3). A direction the covariance pins down to within its own
   --  rounding is given that rounding as its variance, so a separation along
   --  it is significant unless it is rounding too.

   function Distance (A, B : Point_Estimate) return Estimate;
   --  How far apart two independent points are, with the first-order sigma
   --  of that length (along the separation). For the size of a separation;
   --  whether there is one at all is Significant.

   type Direction_Estimate is record
      Unit_Vector : Vec3 := Zero3;
      Sigma       : Real := Real'Last;   --  angular, in radians
   end record;

   type Pose_Estimate is record
      Pose                : Rigid := Identity;
      Position_Covariance : Mat3 := [others => [others => Real'Last]];
      Rotation_Covariance : Mat3 := [others => [others => Real'Last]];
      --  Rotation_Covariance is that of the small rotation vector that takes
      --  the estimate to the truth, expressed in the parent frame.
   end record;

   function Position (P : Pose_Estimate) return Point_Estimate is
     ((Mean => P.Pose.Translation, Covariance => P.Position_Covariance));

   type Ray_Estimate is record
      Origin    : Point_Estimate;
      Direction : Direction_Estimate;
   end record;
   --  A line of sight: the points Origin + s * Direction for s >= 0.

private

   type Gate is record
      Multiple : Real := 0.0;
   end record;

end Driver.Uncertain;
