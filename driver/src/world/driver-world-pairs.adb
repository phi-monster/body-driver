with Ada.Numerics.Long_Elementary_Functions;
with Driver.Distributions;
with Driver.Geometry;
with Driver.Stats;

package body Driver.World.Pairs is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Mad_Efficiency : constant := 0.367_5;
   --  The asymptotic efficiency of the median absolute deviation for Gaussian
   --  data: its scale is worth that share of as many degrees of freedom.

   function Round_Trip (A : Driver.Instrument.Answer; From : Driver.Images.Pixel) return Real is
     (Sqrt ((A.Back.U - From.U) ** 2 + (A.Back.V - From.V) ** 2));

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
      Apart         : out Natural)
   is
      Around : Natural := 0;
   begin
      Kept.Clear;
      Apart := 0;
      for I in Own + 1 .. Points'Length loop
         Around := Around + Boolean'Pos (Answers (Answers'First + I - 1).Found);
      end loop;
      if Around = 0 then
         return;
      end if;
      declare
         Trips : Real_Array (1 .. 2 * Around);
         K     : Natural := 0;
      begin
         for I in Own + 1 .. Points'Length loop
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
         declare
            Trip_Sigma : constant Real := Driver.Stats.Robust_Sigma (Trips);
            Gate       : constant Driver.Uncertain.Gate :=
              Vector_Gate (2, Natural (Real'Floor (Mad_Efficiency * Real (2 * Around))));
            --  A round trip is two matchings; one of them errs by that over the square root of two.
            Match_Sigma : constant Real := Trip_Sigma / Sqrt (2.0);
            --  Two lines of sight meeting in a point leave one degree of freedom.
            Freedom     : constant Positive := 1;
         begin
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
