with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;

package body Driver.Uncertain is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Tail return Real is (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));
   --  How rarely a Gaussian difference exceeds Z of its sigmas: every gate's false-alarm rate.

   function Scalar_Gate (Degrees_Of_Freedom : Natural := 0; Tests : Positive := 1) return Gate is
      Alpha : constant Real := Tail / Real (Tests);
   begin
      --  A single test with a known sigma is Z itself, not its round trip
      --  through the quantile.
      return (Multiple => (if Degrees_Of_Freedom > 0 then Driver.Distributions.Student_T_Quantile (Alpha, Degrees_Of_Freedom)
                           elsif Tests = 1 then Driver.Conventions.Z
                           else Driver.Distributions.Gaussian_Two_Sided_Quantile (Alpha)));
   end Scalar_Gate;

   function Vector_Gate (Dimensions : Positive; Degrees_Of_Freedom : Natural := 0; Tests : Positive := 1)
     return Gate
   is
      Alpha : constant Real := Tail / Real (Tests);
   begin
      return (Multiple => (if Degrees_Of_Freedom = 0 then Sqrt (Driver.Distributions.Chi_Square_Quantile (Alpha, Dimensions))
                           else Sqrt (Real (Dimensions)
                                      * Driver.Distributions.F_Quantile (Alpha, Dimensions, Degrees_Of_Freedom))));
   end Vector_Gate;

   function Threshold (G : Gate) return Real is (G.Multiple);

   function Significant (G : Gate; Difference, Sigma : Real) return Boolean is
     (Sigma < Real'Last and then abs Difference > G.Multiple * Sigma);

   function Significant (Difference, Sigma : Real; Degrees_Of_Freedom : Natural := 0) return Boolean is
     (Sigma < Real'Last and then Significant (Scalar_Gate (Degrees_Of_Freedom), Difference, Sigma));

   function Difference (A, B : Estimate) return Estimate is
   begin
      if A.Sigma >= Real'Last or else B.Sigma >= Real'Last then
         return (Value => A.Value - B.Value, Sigma => Real'Last, Degrees_Of_Freedom => 0);
      end if;
      declare
         Va : constant Real := A.Sigma * A.Sigma;
         Vb : constant Real := B.Sigma * B.Sigma;
         --  A known variance contributes nothing to the uncertainty of the sum's variance.
         Spread : constant Real :=
           (if A.Degrees_Of_Freedom > 0 then Va * Va / Real (A.Degrees_Of_Freedom) else 0.0)
           + (if B.Degrees_Of_Freedom > 0 then Vb * Vb / Real (B.Degrees_Of_Freedom) else 0.0);
         Dof : Natural := 0;
      begin
         if Spread > 0.0 then
            Dof := Natural'Max (1, Natural (Real'Floor ((Va + Vb) * (Va + Vb) / Spread)));
         end if;
         return (Value => A.Value - B.Value, Sigma => Sqrt (Va + Vb), Degrees_Of_Freedom => Dof);
      end;
   end Difference;

   function Significant (A, B : Estimate) return Boolean is
      D : constant Estimate := Difference (A, B);
   begin
      return Significant (D.Value, D.Sigma, D.Degrees_Of_Freedom);
   end Significant;

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
         return (Value => L, Sigma => Real'Last, Degrees_Of_Freedom => 0);
      elsif L = 0.0 then
         --  No direction to project on; the largest spread bounds the error.
         declare
            S : Real := 0.0;
         begin
            for I in 1 .. 3 loop
               S := Real'Max (S, A.Covariance (I, I) + B.Covariance (I, I));
            end loop;
            return (Value => 0.0, Sigma => Sqrt (S), Degrees_Of_Freedom => 0);
         end;
      end if;
      return (Value => L, Sigma => Sigma_Along (A.Covariance + B.Covariance, D / L), Degrees_Of_Freedom => 0);
   end Distance;

   function Significant (A, B : Point_Estimate) return Boolean is
      Separation : constant Vec3 := A.Mean - B.Mean;
      Values     : Vec3;
      Vectors    : Mat3;
      Squared    : Real := 0.0;   --  the squared Mahalanobis length
   begin
      if not Known (A) or else not Known (B) then
         return False;
      end if;
      Symmetric_Eigensystem (A.Covariance + B.Covariance, Values, Vectors);
      declare
         --  The rounding error of the largest variance: below it the
         --  eigendecomposition cannot tell a variance from zero.
         Floor : constant Real := Values (Values'First) * Real (Vec3'Length) * Real'Epsilon;
      begin
         for I in Values'Range loop
            declare
               Along    : Real := 0.0;
               Variance : constant Real := Real'Max (Values (I), Floor);
            begin
               for J in Separation'Range loop
                  Along := Along + Vectors (J, I) * Separation (J);
               end loop;
               if Variance > 0.0 then
                  Squared := Squared + Along * Along / Variance;
               elsif Along /= 0.0 then
                  return True;   --  both points exact: any separation is significant
               end if;
            end;
         end loop;
      end;
      return Sqrt (Squared) > Threshold (Vector_Gate (Vec3'Length));
   end Significant;

end Driver.Uncertain;
