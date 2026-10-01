--  Measured quantities with their uncertainty, and the driver's single test
--  of significance.
--
--  Every gate in the driver has the form "is this difference larger than Z
--  standard deviations of its own measured noise" (Driver.Conventions.Z).
--  Vector-valued differences are tested along their own direction, so one
--  rule covers scalars, points, directions and poses alike.

with Driver.Numerics;

package Driver.Uncertain with Pure is

   use Driver.Numerics;

   type Estimate is record
      Value : Real := 0.0;
      Sigma : Real := Real'Last;
   end record;
   --  Sigma is the standard deviation of Value; Real'Last means unknown.

   Unknown : constant Estimate := (Value => 0.0, Sigma => Real'Last);

   function Known (E : Estimate) return Boolean is (E.Sigma < Real'Last);

   function Significant (Difference, Sigma : Real) return Boolean;
   --  abs Difference > Z * Sigma. A zero sigma makes any nonzero difference
   --  significant; an unknown sigma makes none significant.

   function Significant (A, B : Estimate) return Boolean;
   --  The difference of two independent estimates against their combined sigma.

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
