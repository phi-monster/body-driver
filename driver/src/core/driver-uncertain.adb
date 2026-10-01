with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;

package body Driver.Uncertain is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Significant (Difference, Sigma : Real) return Boolean is
   begin
      if Sigma >= Real'Last then
         return False;
      end if;
      return abs Difference > Driver.Conventions.Z * Sigma;
   end Significant;

   function Combined (A, B : Real) return Real is
     (if A >= Real'Last or else B >= Real'Last then Real'Last else Sqrt (A * A + B * B));

   function Significant (A, B : Estimate) return Boolean is
     (Significant (A.Value - B.Value, Combined (A.Sigma, B.Sigma)));

   function Sigma_Along (Covariance : Mat3; Direction : Vec3) return Real is
   begin
      if Covariance (1, 1) >= Real'Last then
         return Real'Last;
      end if;
      return Sqrt (Real'Max (0.0, Direction * (Covariance * Direction)));
   end Sigma_Along;

   function Distance (A, B : Point_Estimate) return Estimate is
      D : constant Vec3 := A.Mean - B.Mean;
      L : constant Real := abs D;
   begin
      if not Known (A) or else not Known (B) then
         return (Value => L, Sigma => Real'Last);
      elsif L = 0.0 then
         --  No direction to project on; the largest spread bounds the error.
         declare
            S : Real := 0.0;
         begin
            for I in 1 .. 3 loop
               S := Real'Max (S, A.Covariance (I, I) + B.Covariance (I, I));
            end loop;
            return (Value => 0.0, Sigma => Sqrt (S));
         end;
      end if;
      return (Value => L, Sigma => Sigma_Along (A.Covariance + B.Covariance, D / L));
   end Distance;

   function Significant (A, B : Point_Estimate) return Boolean is
      D : constant Estimate := Distance (A, B);
   begin
      return Significant (D.Value, D.Sigma);
   end Significant;

end Driver.Uncertain;
