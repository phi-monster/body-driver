with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Distributions;
with Driver.Geometry;

package body Driver.World.Pairs is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;


   function Round_Trip (A : Driver.Instrument.Answer; From : Driver.Images.Pixel) return Real is
     (Sqrt ((A.Back.U - From.U) ** 2 + (A.Back.V - From.V) ** 2));

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real) is
      N       : constant Natural := Trips'Length / 2;
      Squared : Real_Array (1 .. N);   --  each round trip's squared length
      Sorted  : Real_Array (1 .. N);
      Best    : Real := Real'First;    --  the best likelihood found so far
      Wrong   : constant Real := 1.0 / Area;   --  a wrong match's density

      procedure Fit (Start : Real) is
         --  EM from a right match's spread Start, half the trips taken right.
         S2   : Real := Start * Start;   --  one coordinate's variance
         Pi   : Real := 0.5;              --  the share of right ones
         Last : Real := Real'First;
      begin
         loop
            declare
               Weight : Real := 0.0;
               Spread : Real := 0.0;
               Like   : Real := 0.0;
            begin
               for D of Squared loop
                  declare
                     G : constant Real := Exp (-D / (2.0 * S2)) / (2.0 * Ada.Numerics.Pi * S2);
                     P : constant Real := Pi * G + (1.0 - Pi) * Wrong;
                     W : constant Real := Pi * G / P;
                  begin
                     Like := Like + Log (P);
                     Weight := Weight + W;
                     Spread := Spread + W * D;
                  end;
               end loop;
               exit when Like <= Last or else Weight <= 0.0 or else Spread <= 0.0;
               Last := Like;
               if Like > Best then
                  Best := Like;
                  Sigma := Sqrt (S2);
                  Right := Pi * Real (N);
               end if;
               S2 := Spread / (2.0 * Weight);
               Pi := Weight / Real (N);
            end;
         end loop;
      end Fit;

      Rank : Positive := 1;
      procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Real, Real_Array);
   begin
      Sigma := Real'Last;
      Right := 0.0;
      if N = 0 then
         return;
      end if;
      for I in 1 .. N loop
         Squared (I) := Trips (Trips'First + 2 * (I - 1)) ** 2 + Trips (Trips'First + 2 * I - 1) ** 2;
      end loop;
      if (for all D of Squared => D = 0.0) then
         --  Every pixel came back exactly: no error to see.
         Sigma := 0.0;
         Right := Real (N);
         return;
      end if;
      Sorted := Squared;
      Sort (Sorted);
      loop
         if Sorted (Rank) > 0.0 then
            Fit (Sqrt (Sorted (Rank) / 2.0));
         end if;
         exit when Rank = N;
         Rank := Positive'Min (N, 2 * Rank);
      end loop;
   end Matcher_Error;

   function Across_Basis (U : Vec3; Second : Boolean) return Vec3 is
      --  Two unit directions across U, from the coordinate axis furthest from it.
      Axis : constant Vec3 :=
        (if abs U (1) <= abs U (2) and then abs U (1) <= abs U (3) then [1.0, 0.0, 0.0]
         elsif abs U (2) <= abs U (3) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
      E1 : constant Vec3 := Unit (Cross (U, Axis));
   begin
      return (if Second then Cross (U, E1) else E1);
   end Across_Basis;

   function Misfit (Rays : Driver.Geometry.Ray_Array; X : Vec3) return Real is
      --  How far the point is from each line, across it, in units of the
      --  line's own uncertainty there: a chi square.
      Chi : Real := 0.0;
   begin
      for R of Rays loop
         declare
            U  : constant Vec3 := R.Direction.Unit_Vector;
            E1 : constant Vec3 := Across_Basis (U, False);
            E2 : constant Vec3 := Across_Basis (U, True);
            T  : constant Real := (X - R.Origin.Mean) * U;
            M  : constant Mat3 := R.Origin.Covariance + ((T * R.Direction.Sigma) ** 2) * Identity3;
            D  : constant Vec3 := X - R.Origin.Mean;
            A  : constant Real := E1 * (M * E1);
            B  : constant Real := E1 * (M * E2);
            C  : constant Real := E2 * (M * E2);
            P  : constant Real := D * E1;
            Q  : constant Real := D * E2;
            Det : constant Real := A * C - B * B;
         begin
            Chi := Chi + (C * P * P - 2.0 * B * P * Q + A * Q * Q) / Det;
         end;
      end loop;
      return Chi;
   end Misfit;

   procedure Triangulate
     (First, Second : Driver.World.Cameras.Camera'Class;
      Points        : Driver.Instrument.Point_Array;
      Own           : Natural;
      Answers       : Driver.Instrument.Answer_Array;
      Kept          : out Match_Vectors.Vector;
      Apart         : out Natural;
      Error         : out Real)
   is
      --  The points whose round trips measure the matcher's error: those
      --  around the region, or all of them when nothing around was asked.
      First_Sample : constant Positive := (if Own < Points'Length then Own + 1 else 1);
      Around : Natural := 0;
   begin
      Kept.Clear;
      Error := Real'Last;
      Apart := 0;
      for I in First_Sample .. Points'Length loop
         Around := Around + Boolean'Pos (Answers (Answers'First + I - 1).Found);
      end loop;
      if Around = 0 then
         return;
      end if;
      declare
         Trips      : Real_Array (1 .. 2 * Around);
         Trip_Sigma : Real;   --  one coordinate of a right match's round trip
         Right      : Real;   --  how many round trips are of right matches
         K          : Natural := 0;
      begin
         for I in First_Sample .. Points'Length loop
            declare
               A : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
               P : constant Driver.Images.Pixel := Points (Points'First + I - 1);
            begin
               if A.Found then
                  Trips (K + 1) := A.Back.U - P.U;
                  Trips (K + 2) := A.Back.V - P.V;
                  K := K + 2;
               end if;
            end;
         end loop;
         Matcher_Error (Trips, Real (First.Width) * Real (First.Height), Trip_Sigma, Right);
         if not (Trip_Sigma < Real'Last) or else Right < 1.0 then
            return;
         end if;
         declare
            --  The right round trips' two coordinates each are the error's
            --  degrees of freedom.
            Gate       : constant Driver.Uncertain.Gate := Vector_Gate (2, Natural (Real'Floor (2.0 * Right)));
            --  A round trip is two matchings; one of them errs by that over the square root of two.
            Match_Sigma : constant Real := Trip_Sigma / Sqrt (2.0);
            --  Two lines of sight meeting in a point leave one degree of freedom.
            Freedom     : constant Positive := 1;
         begin
            Error := Match_Sigma;
            for I in 1 .. Own loop
               declare
                  A : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
                  P : constant Driver.Images.Pixel := Points (Points'First + I - 1);
               begin
                  if A.Found and then not Significant (Gate, Round_Trip (A, P), Trip_Sigma) then
                     declare
                        From_First  : constant Ray_Estimate := First.Ray (P);
                        From_Second : Ray_Estimate := Second.Ray (A.To);
                        Turn        : constant Real := Driver.World.Cameras.Radians_Per_Pixel (Second, A.To);
                        X           : Point_Estimate;
                        Met         : Boolean;
                     begin
                        if Turn < Real'Last and then From_Second.Direction.Sigma < Real'Last then
                           From_Second.Direction.Sigma :=
                             Sqrt (From_Second.Direction.Sigma ** 2 + (Match_Sigma * Turn) ** 2);
                           Driver.Geometry.Meet ([From_First, From_Second], X, Met);
                           if Met then
                              if Significant
                                (Driver.Distributions.Chi_Square_Deviate
                                   (Misfit ([From_First, From_Second], X.Mean), Freedom), 1.0)
                              then
                                 Apart := Apart + 1;
                              else
                                 Kept.Append (Match'(In_First => P, In_Second => A.To, Point => X));
                              end if;
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end loop;
         end;
      end;
   end Triangulate;

end Driver.World.Pairs;
