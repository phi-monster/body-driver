with Ada.Containers.Ordered_Maps;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;

package body Driver.Uncertain is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Tail return Real is (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));
   --  How rarely a Gaussian difference exceeds Z of its sigmas: every gate's false-alarm rate.

   --  Every gate worked out once: a gate is a pure function of what it is asked for, and the fit and the lock-in ask
   --  the same few of them for every sighting and every cell. The Student quantile behind the test of each sighting of
   --  a fit's tracks, its root found anew every time, was six tenths of the fit's last stage, and that stage took 53
   --  to 304 s a fit while the body held still (f15w on 10-15, sampled).
   type Gate_Key is record
      Vector     : Boolean;    --  a vector gate (Vector_Gate), or a scalar one
      Dimensions : Positive;
      Freedom    : Natural;
      Tests      : Positive;
   end record;

   function "<" (A, B : Gate_Key) return Boolean is
     (A.Vector < B.Vector
      or else (A.Vector = B.Vector
               and then (A.Dimensions < B.Dimensions
                         or else (A.Dimensions = B.Dimensions
                                  and then (A.Freedom < B.Freedom
                                            or else (A.Freedom = B.Freedom and then A.Tests < B.Tests))))));

   package Gate_Maps is new Ada.Containers.Ordered_Maps (Gate_Key, Real);

   type Known_Gate is record
      Found    : Boolean := False;
      Multiple : Real := 0.0;
   end record;

   protected Gates_Known is
      function Get (Key : Gate_Key) return Known_Gate;
      procedure Keep (Key : Gate_Key; Multiple : Real);
   private
      Map : Gate_Maps.Map;
   end Gates_Known;

   protected body Gates_Known is
      function Get (Key : Gate_Key) return Known_Gate is
         C : constant Gate_Maps.Cursor := Map.Find (Key);
      begin
         return (if Gate_Maps.Has_Element (C) then (Found => True, Multiple => Gate_Maps.Element (C)) else (others => <>));
      end Get;

      procedure Keep (Key : Gate_Key; Multiple : Real) is
      begin
         Map.Include (Key, Multiple);
      end Keep;
   end Gates_Known;

   --  The Gaussian quantile of the gates' tail over so many tests, worked out once: the Student quantile of every new
   --  number of degrees of freedom starts from it, and worked it out anew (a fit's sightings ask for a new number at
   --  nearly every one: 0.7 s of the fit's thread of 19 s in it, s02's dump replayed and sampled on 10-16).
   package Quantile_Maps is new Ada.Containers.Ordered_Maps (Positive, Real);

   protected Gaussians_Known is
      function Get (Tests : Positive) return Known_Gate;
      procedure Keep (Tests : Positive; Quantile : Real);
   private
      Map : Quantile_Maps.Map;
   end Gaussians_Known;

   protected body Gaussians_Known is
      function Get (Tests : Positive) return Known_Gate is
         C : constant Quantile_Maps.Cursor := Map.Find (Tests);
      begin
         return (if Quantile_Maps.Has_Element (C) then (Found => True, Multiple => Quantile_Maps.Element (C))
                 else (others => <>));
      end Get;

      procedure Keep (Tests : Positive; Quantile : Real) is
      begin
         Map.Include (Tests, Quantile);
      end Keep;
   end Gaussians_Known;

   function Gaussian_Of (Tests : Positive) return Real is
      Known : constant Known_Gate := Gaussians_Known.Get (Tests);
   begin
      if Known.Found then
         return Known.Multiple;
      end if;
      declare
         Quantile : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (Tail / Real (Tests));
      begin
         Gaussians_Known.Keep (Tests, Quantile);
         return Quantile;
      end;
   end Gaussian_Of;

   function Scalar_Gate (Degrees_Of_Freedom : Natural := 0; Tests : Positive := 1) return Gate is
      Key   : constant Gate_Key := (Vector => False, Dimensions => 1, Freedom => Degrees_Of_Freedom, Tests => Tests);
      Known : constant Known_Gate := Gates_Known.Get (Key);
      Alpha : constant Real := Tail / Real (Tests);
   begin
      if Known.Found then
         return (Multiple => Known.Multiple);
      end if;
      declare
         --  A single test with a known sigma is Z itself, not its round trip
         --  through the quantile.
         Multiple : constant Real := (if Degrees_Of_Freedom > 0 then Driver.Distributions.Student_T_Quantile (Alpha, Degrees_Of_Freedom, Gaussian_Of (Tests))
                           elsif Tests = 1 then Driver.Conventions.Z
                           else Driver.Distributions.Gaussian_Two_Sided_Quantile (Alpha));
      begin
         Gates_Known.Keep (Key, Multiple);
         return (Multiple => Multiple);
      end;
   end Scalar_Gate;

   function Vector_Gate (Dimensions : Positive; Degrees_Of_Freedom : Natural := 0; Tests : Positive := 1)
     return Gate
   is
      Key   : constant Gate_Key :=
        (Vector => True, Dimensions => Dimensions, Freedom => Degrees_Of_Freedom, Tests => Tests);
      Known : constant Known_Gate := Gates_Known.Get (Key);
      Alpha : constant Real := Tail / Real (Tests);
   begin
      if Known.Found then
         return (Multiple => Known.Multiple);
      end if;
      declare
         Multiple : constant Real := (if Degrees_Of_Freedom = 0 then Sqrt (Driver.Distributions.Chi_Square_Quantile (Alpha, Dimensions))
                           else Sqrt (Real (Dimensions)
                                      * Driver.Distributions.F_Quantile (Alpha, Dimensions, Degrees_Of_Freedom)));
      begin
         Gates_Known.Keep (Key, Multiple);
         return (Multiple => Multiple);
      end;
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
            Dof := Satterthwaite_Count ((Va + Vb) * (Va + Vb) / Spread);
         end if;
         return (Value => A.Value - B.Value, Sigma => Sqrt (Va + Vb), Degrees_Of_Freedom => Dof);
      end;
   end Difference;

   function Satterthwaite_Count (Count : Real) return Natural is
     (if Count >= Real (Natural'Last) then 0 else Natural'Max (1, Natural (Real'Floor (Count))));

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
