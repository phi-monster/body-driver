--  Measurement geometry: where lines of sight meet, and planes measured
--  from points, each with its uncertainty.
--
--  Every layer that measures something in space uses these: the body to fit
--  its cameras and kinematics, the hand to find a fingertip (where a line of
--  sight meets the surface it pressed on), the world to place things and the
--  surfaces they rest on. Nothing here knows about bodies or scenes; the
--  inputs are points and rays with their covariances, and every output
--  carries the covariance that follows from them to first order.

with Driver.Numerics;
with Driver.Uncertain;

package Driver.Geometry is

   use Driver.Numerics;
   use Driver.Uncertain;

   type Ray_Array is array (Positive range <>) of Ray_Estimate;

   procedure Meet (Rays : Ray_Array; Point : out Point_Estimate; Ok : out Boolean);
   --  The point nearest to every ray, each weighted by its own uncertainty
   --  perpendicular to itself where it passes the point (its origin's
   --  covariance and its angular sigma times the distance travelled). Ok is
   --  False with fewer than two rays, when the rays are parallel to working
   --  precision, or when the point lies behind one of the rays.

   --  A plane n . x = n . Centre. Centre is the weighted centroid of what
   --  measured it, so there its offset and its tilt are uncorrelated, and the
   --  uncertainty of the plane's height grows with the distance from Centre.
   type Plane_Estimate is record
      Centre       : Vec3 := Zero3;
      Normal       : Vec3 := Zero3;         --  unit; Orient turns it to the side it was seen or touched from
      Tangent_1    : Vec3 := Zero3;         --  unit, perpendicular to Normal
      Tangent_2    : Vec3 := Zero3;         --  Cross (Normal, Tangent_1)
      Offset_Sigma : Real := Real'Last;     --  along Normal, at Centre
      Tilt_11      : Real := Real'Last;     --  covariance of the tilt towards Tangent_1 and Tangent_2,
      Tilt_12      : Real := 0.0;           --  in radians squared: a small change of the normal is
      Tilt_22      : Real := Real'Last;     --  a * Tangent_1 + b * Tangent_2
      Points       : Natural := 0;          --  how many points it rests on
      Scatter      : Real := Real'Last;     --  chi square per degree of freedom of the points' residuals
   end record;

   function Known (P : Plane_Estimate) return Boolean is (P.Offset_Sigma < Real'Last);

   function Height (P : Plane_Estimate; X : Vec3) return Real;
   --  Signed distance of X from the plane along its normal.

   function Height_Sigma (P : Plane_Estimate; X : Vec3) return Real;
   --  The plane's own uncertainty of that height at X (offset and tilt).

   function Height (P : Plane_Estimate; X : Point_Estimate) return Estimate;
   --  Height of an uncertain point, combining the point's and the plane's uncertainty.

   procedure Intersect (P : Plane_Estimate; R : Ray_Estimate; Point : out Point_Estimate; Distance : out Estimate;
                        Ok : out Boolean);
   --  Where the ray meets the plane, and how far along the ray. Ok is False
   --  when the ray is parallel to the plane or meets it behind its origin.

   type Point_Array is array (Positive range <>) of Point_Estimate;
   type Flag_Array is array (Positive range <>) of Boolean;

   procedure Fit (Points : Point_Array; Use_Point : Flag_Array; P : out Plane_Estimate; Ok : out Boolean)
     with Pre => Points'First = Use_Point'First and then Points'Last = Use_Point'Last;
   --  Weighted total least squares over the points marked in Use_Point, each
   --  weighted by its variance along the normal (iterated, since the normal
   --  is what is being found). The uncertainty is scaled by the residuals'
   --  own scatter, so a rough or slightly bent surface reports what it is.
   --  Ok is False with fewer points than a plane and its scatter need, or
   --  when they lie on a line.

   procedure Fit_Robust
     (Points    : Point_Array;
      Start     : Flag_Array;
      P         : out Plane_Estimate;
      Inliers   : out Flag_Array;
      Ok        : out Boolean)
     with Pre => Points'First = Start'First and then Points'Last = Start'Last
                 and then Points'First = Inliers'First and then Points'Last = Inliers'Last;
   --  Fits the points marked in Start, then keeps re-selecting, among all the
   --  points, those whose height is not significant against the combined
   --  uncertainty of the point and the plane, and refits, until the selection
   --  stops changing. Start is the hypothesis (all points, or those near a
   --  touch); a majority of points off the plane in Start can mislead it.
   --  Ok is False when the selection has not settled after one pass per point.

   procedure Orient (P : in out Plane_Estimate; Towards : Vec3);
   --  Turns the normal, if needed, to point to the side of Towards (the eye
   --  that saw the points, or the hand that touched them).

end Driver.Geometry;
