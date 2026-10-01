--  Measured quantities with their uncertainty, and the driver's single test
--  of significance.
--
--  Every gate in the driver has the form "is this difference larger than Z
--  standard deviations of its own measured noise" (Driver.Conventions.Z).
--  Vector-valued differences are tested along their own direction, so one
--  rule covers scalars, points, directions and poses alike.
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

   type Point_Estimate is record
      Mean       : Vec3 := Zero3;
      Covariance : Mat3 := [others => [others => Real'Last]];
   end record;

   function Known (P : Point_Estimate) return Boolean is (P.Covariance (1, 1) < Real'Last);

   function Sigma_Along (Covariance : Mat3; Direction : Vec3) return Real;
   --  The standard deviation of the component along a unit direction.

   function Significant (A, B : Point_Estimate) return Boolean;
   --  The separation of two independent points, tested along itself.

   function Distance (A, B : Point_Estimate) return Estimate;

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

end Driver.Uncertain;
