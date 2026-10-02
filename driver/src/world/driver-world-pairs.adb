with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Distributions;
with Driver.Geometry;

package body Driver.World.Pairs is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;


   function Round_Trip (A : Driver.Instrument.Answer; From : Driver.Images.Pixel) return Real is
     (Sqrt ((A.Back.U - From.U) ** 2 + (A.Back.V - From.V) ** 2));

   procedure Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real)
   is
      --  Errors of right matches, a centred isotropic Gaussian in that many
      --  dimensions, mixed with those of wrong ones, spread evenly over a
      --  range of that Measure; each error given by its squared length. The
      --  mixture's maximum likelihood (EM, started from every doubling rank of
      --  the errors' lengths, each run until its likelihood stops growing):
      --  Sigma of one coordinate of a right match's error, and how many of the
      --  errors are of right ones.
      N      : constant Natural := Squared'Length;
      Sorted : Real_Array := Squared;
      Best   : Real := Real'First;   --  the best likelihood found so far
      Wrong  : constant Real := 1.0 / Measure;   --  a wrong match's density
      K      : constant Real := Real (Dimensions);

      procedure Fit (Start : Real) is
         --  EM from a right match's spread Start, half the errors taken right.
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
                     G : constant Real := Exp (-D / (2.0 * S2)) / (2.0 * Ada.Numerics.Pi * S2) ** (K / 2.0);
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
               S2 := Spread / (K * Weight);
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
      if (for all D of Squared => D = 0.0) then
         --  Every error is nought: no error to see.
         Sigma := 0.0;
         Right := Real (N);
         return;
      end if;
      Sort (Sorted);
      loop
         if Sorted (Sorted'First + Rank - 1) > 0.0 then
            Fit (Sqrt (Sorted (Sorted'First + Rank - 1) / K));
         end if;
         exit when Rank = N;
         Rank := Positive'Min (N, 2 * Rank);
      end loop;
   end Mixture;

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real) is
      Squared : Real_Array (1 .. Trips'Length / 2);   --  each round trip's squared length
   begin
      for I in Squared'Range loop
         Squared (I) := Trips (Trips'First + 2 * (I - 1)) ** 2 + Trips (Trips'First + 2 * I - 1) ** 2;
      end loop;
      Mixture (Squared, 2, Area, Sigma, Right);
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
            Gate      : constant Driver.Uncertain.Gate := Vector_Gate (2, Natural (Real'Floor (2.0 * Right)));
            --  Two lines of sight meeting in a point leave one degree of freedom.
            Freedom   : constant Positive := 1;
            --  The matches that came back, and how far, in the second eye's
            --  pixels, each one's two lines of sight pass each other.
            Candidate : array (1 .. Own) of Natural := [others => 0];
            Off       : Real_Array (1 .. Own);
            Count     : Natural := 0;
            Diagonal  : constant Real := Sqrt (Real (Second.Width) ** 2 + Real (Second.Height) ** 2);
            Line_Sigma : Real;
            Line_Right : Real;
         begin
            for I in 1 .. Own loop
               declare
                  A : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
                  P : constant Driver.Images.Pixel := Points (Points'First + I - 1);
               begin
                  if A.Found and then not Significant (Gate, Round_Trip (A, P), Trip_Sigma) then
                     declare
                        R1   : constant Ray_Estimate := First.Ray (P);
                        R2   : constant Ray_Estimate := Second.Ray (A.To);
                        Turn : constant Real := Driver.World.Cameras.Radians_Per_Pixel (Second, A.To);
                        U1   : constant Vec3 := R1.Direction.Unit_Vector;
                        U2   : constant Vec3 := R2.Direction.Unit_Vector;
                        W0   : constant Vec3 := R1.Origin.Mean - R2.Origin.Mean;
                        B    : constant Real := U1 * U2;
                        Skew : constant Real := 1.0 - B * B;
                     begin
                        if Turn < Real'Last and then R1.Direction.Sigma < Real'Last and then R2.Direction.Sigma < Real'Last
                          and then Skew > 0.0
                        then
                           declare
                              --  The closest points of the two lines.
                              S  : constant Real := (B * (U2 * W0) - U1 * W0) / Skew;
                              T  : constant Real := ((U2 * W0) - B * (U1 * W0)) / Skew;
                              Q1 : constant Vec3 := R1.Origin.Mean + S * U1;
                              Q2 : constant Vec3 := R2.Origin.Mean + T * U2;
                           begin
                              if S > 0.0 and then T > 0.0 then
                                 Count := Count + 1;
                                 Candidate (Count) := I;
                                 Off (Count) := abs (Q1 - Q2) / (T * Turn);
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Count = 0 then
               return;
            end if;
            --  The matcher's error across the line the first sight draws in the
            --  second eye, from the matches whose lines pass close, told from
            --  the wrong ones, which pass anywhere within the image: a matcher
            --  can be wrong yet come back, and only the geometry tells.
            declare
               Squared : Real_Array (1 .. Count);
            begin
               for C in Squared'Range loop
                  Squared (C) := Off (C) ** 2;
               end loop;
               Mixture (Squared, 1, 2.0 * Diagonal, Line_Sigma, Line_Right);
            end;
            if not (Line_Sigma < Real'Last) or else Line_Right < 1.0 then
               return;
            end if;
            Error := Line_Sigma;
            for C in 1 .. Count loop
               declare
                  I           : constant Positive := Candidate (C);
                  A           : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
                  P           : constant Driver.Images.Pixel := Points (Points'First + I - 1);
                  From_First  : constant Ray_Estimate := First.Ray (P);
                  From_Second : Ray_Estimate := Second.Ray (A.To);
                  Turn        : constant Real := Driver.World.Cameras.Radians_Per_Pixel (Second, A.To);
                  X           : Point_Estimate;
                  Met         : Boolean;
               begin
                  From_Second.Direction.Sigma := Sqrt (From_Second.Direction.Sigma ** 2 + (Line_Sigma * Turn) ** 2);
                  Driver.Geometry.Meet ([From_First, From_Second], X, Met);
                  if Met then
                     if Significant
                       (Driver.Distributions.Chi_Square_Deviate (Misfit ([From_First, From_Second], X.Mean), Freedom),
                        1.0)
                     then
                        Apart := Apart + 1;
                     else
                        Kept.Append (Match'(In_First => P, In_Second => A.To, Point => X, First => <>));
                     end if;
                  end if;
               end;
            end loop;
         end;
      end;
   end Triangulate;

end Driver.World.Pairs;
