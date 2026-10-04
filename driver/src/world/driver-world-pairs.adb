with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Geometry;

package body Driver.World.Pairs is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   --  Everything sized by matches lives on the heap: the estimates also run
   --  in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Index_Array is array (Positive range <>) of Natural;
   type Index_Access is access Index_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);

   function Round_Trip (A : Driver.Instrument.Answer; From : Driver.Images.Pixel) return Real is
     (Sqrt ((A.Back.U - From.U) ** 2 + (A.Back.V - From.V) ** 2));

   procedure Fit_Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural)
   is
      --  Errors of right matches, a centred isotropic Gaussian in that many
      --  dimensions, mixed with those of wrong ones, spread evenly over a
      --  range of that Measure; each error given by its squared length. The
      --  mixture's maximum likelihood, climbed from every doubling rank of the
      --  errors' lengths until it stops growing: Sigma of one coordinate of a
      --  right match's error, and how many of the errors are of right ones.
      N      : constant Natural := Squared'Length;
      Best   : Real := Real'First;   --  the best likelihood found so far
      Wrong  : constant Real := 1.0 / Measure;   --  a wrong match's density
      K      : constant Real := Real (Dimensions);

      type Point is record
         S2 : Real;   --  one coordinate's variance of a right match's error
         Pi : Real;   --  the share of right ones
      end record;

      --  The likelihood at a point, its slope and its curvature there, and
      --  the step EM takes from it: one pass over the errors.
      type Look is record
         Like                      : Real := 0.0;
         Slope_Pi, Slope_S2        : Real := 0.0;
         Bend_PP, Bend_PS, Bend_SS : Real := 0.0;
         EM                        : Point := (S2 => 0.0, Pi => 0.0);
      end record;

      function Seen_At (P : Point) return Look is
         --  With G the right ones' density at an error, A and B its first and
         --  second derivatives by the variance over itself, F the mixture's
         --  density and W the share of it the right ones hold.
         L      : Look;
         C      : constant Real := (2.0 * Ada.Numerics.Pi * P.S2) ** (-K / 2.0);
         Weight : Real := 0.0;
         Spread : Real := 0.0;
      begin
         Passes := Passes + 1;
         for D of Squared loop
            declare
               G : constant Real := C * Exp (-D / (2.0 * P.S2));
               F : constant Real := P.Pi * G + (1.0 - P.Pi) * Wrong;
            begin
               if not (F > 0.0) then
                  --  Every error taken right, and this one none could be.
                  L.Like := Real'First;
                  return L;
               end if;
               declare
                  A : constant Real := (D - K * P.S2) / (2.0 * P.S2 ** 2);
                  B : constant Real := (K * P.S2 - 2.0 * D) / (2.0 * P.S2 ** 3);
                  W : constant Real := P.Pi * G / F;
               begin
                  L.Like := L.Like + Log (F);
                  L.Slope_Pi := L.Slope_Pi + (G - Wrong) / F;
                  L.Slope_S2 := L.Slope_S2 + W * A;
                  L.Bend_PP := L.Bend_PP - ((G - Wrong) / F) ** 2;
                  L.Bend_PS := L.Bend_PS + Wrong * G * A / F ** 2;
                  L.Bend_SS := L.Bend_SS + W * (A ** 2 + B) - (W * A) ** 2;
                  Weight := Weight + W;
                  Spread := Spread + W * D;
               end;
            end;
         end loop;
         L.EM := (S2 => (if Weight > 0.0 then Spread / (K * Weight) else 0.0), Pi => Weight / Real (N));
         return L;
      end Seen_At;

      --  Every error taken right, at their own spread: where a step that
      --  would take more than all of them right lands.
      All_Right : Point := (S2 => 0.0, Pi => 1.0);

      function Newton (P : Point; L : Look; Concave : out Boolean; Rest : out Real) return Point is
         --  Newton's step, where the likelihood is concave; P itself where it
         --  is not. With every error taken right and the likelihood rising
         --  still towards more, the step is along the variance alone. Rest is
         --  the step's length in units of the estimate's own uncertainty, its
         --  covariance the inverse of the likelihood's curvature, squared:
         --  where the likelihood is concave, how far the maximum still is.
         Det : constant Real := L.Bend_PP * L.Bend_SS - L.Bend_PS ** 2;
      begin
         Concave := False;
         Rest := Real'Last;
         if P.Pi = 1.0 and then L.Slope_Pi >= 0.0 then
            if L.Bend_SS < 0.0 then
               Concave := True;
               Rest := L.Slope_S2 ** 2 / (-L.Bend_SS);
               return (S2 => P.S2 - L.Slope_S2 / L.Bend_SS, Pi => 1.0);
            end if;
            return P;
         elsif L.Bend_PP < 0.0 and then Det > 0.0 then
            declare
               D_S2 : constant Real := -(L.Bend_PP * L.Slope_S2 - L.Bend_PS * L.Slope_Pi) / Det;
               D_Pi : constant Real := -(L.Bend_SS * L.Slope_Pi - L.Bend_PS * L.Slope_S2) / Det;
            begin
               Concave := True;
               Rest := -(L.Bend_PP * D_Pi ** 2 + 2.0 * L.Bend_PS * D_Pi * D_S2 + L.Bend_SS * D_S2 ** 2);
               return (S2 => P.S2 + D_S2, Pi => P.Pi + D_Pi);
            end;
         else
            return P;
         end if;
      end Newton;

      function Valid (P : Point) return Boolean is (P.S2 > 0.0 and then P.Pi > 0.0 and then P.Pi <= 1.0);

      procedure Fit (Start : Real) is
         --  From a right match's spread Start, half the errors taken right:
         --  Newton's step when it raises the likelihood, else EM's, which
         --  never lowers it, until the likelihood is concave and its maximum
         --  is nearer than Unchanged_Fraction of the estimate's own
         --  uncertainty (Driver.Conventions), or no step raises it any more.
         --  EM alone gets there too, but by thousands of steps where the
         --  likelihood is flat, as it is when nearly every error is a right
         --  one; and EM's own steps say nothing of how far the maximum is.
         P : Point := (S2 => Start * Start, Pi => 0.5);
         L : Look := Seen_At (P);
      begin
         loop
            if L.Like > Best then
               Best := L.Like;
               Sigma := Sqrt (P.S2);
               Right := P.Pi * Real (N);
            end if;
            declare
               Concave : Boolean;
               Rest    : Real;
               Step    : Point := Newton (P, L, Concave, Rest);
               Next    : Look;
               Moved   : Boolean := False;
            begin
               exit when Concave and then Rest <= Driver.Conventions.Unchanged_Fraction ** 2;
               if Step.Pi >= 1.0 and then P.Pi < 1.0 then
                  Step := All_Right;
               end if;
               if Step /= P and then Valid (Step) then
                  Next := Seen_At (Step);
                  Moved := Next.Like > L.Like;
               end if;
               if not Moved then
                  Step := L.EM;
                  if Step /= P and then Valid (Step) then
                     Next := Seen_At (Step);
                     Moved := Next.Like > L.Like;
                  end if;
               end if;
               exit when not Moved;
               P := Step;
               L := Next;
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
      for D of Squared loop
         All_Right.S2 := All_Right.S2 + D;
      end loop;
      All_Right.S2 := All_Right.S2 / (K * Real (N));
      declare
         Sorted : Real_Access := new Real_Array'(Squared);
      begin
         Sort (Sorted.all);
         loop
            if Sorted (Sorted'First + Rank - 1) > 0.0 then
               Fit (Sqrt (Sorted (Sorted'First + Rank - 1) / K));
            end if;
            exit when Rank = N;
            Rank := Positive'Min (N, 2 * Rank);
         end loop;
         Free (Sorted);
      end;
   end Fit_Mixture;

   function Finite (X : Point_Estimate) return Boolean is
     (for all A in 1 .. 3 =>
        X.Mean (A) = X.Mean (A) and then abs X.Mean (A) <= Real'Last
        and then (for all B in 1 .. 3 => X.Covariance (A, B) = X.Covariance (A, B)
                                         and then abs X.Covariance (A, B) <= Real'Last));
   --  Every coordinate and covariance a number (a NaN is not equal to itself).

   function Ball (Dimensions : Positive; Radius : Real) return Real is
      --  The measure of the errors no longer than Radius: a segment of both
      --  signs, a disc, a ball (each two dimensions more multiply it by
      --  2 pi Radius squared over the dimensions).
      Measure : Real := (if Dimensions mod 2 = 0 then 1.0 else 2.0 * Radius);
      D       : Natural := (if Dimensions mod 2 = 0 then 0 else 1);
   begin
      while D < Dimensions loop
         D := D + 2;
         Measure := Measure * 2.0 * Ada.Numerics.Pi * Radius ** 2 / Real (D);
      end loop;
      return Measure;
   end Ball;

   procedure Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural)
   is
      --  The mixture fitted in a window that closes on the right matches: the
      --  errors the fit's own gate holds, the wrong ones among them taken as
      --  spread evenly over the window, until the window holds every error it
      --  held before. Wrong matches are not spread evenly over the whole
      --  range: a matcher that is lost near what it was asked comes back near
      --  it, and over the whole range those errors look like a wide Gaussian
      --  that swallows the right ones. Within the gate of that wide fit, they
      --  are what is spread evenly, and the right ones stand out again.
      Errors : Real_Access := new Real_Array'(Squared);
      Count  : Natural := Squared'Length;
      Window : Real := Measure;
   begin
      loop
         Fit_Mixture (Errors (Errors'First .. Errors'First + Count - 1), Dimensions, Window, Sigma, Right, Passes);
         exit when Right < 1.0 or else not (Sigma > 0.0 and then Sigma < Real'Last);
         declare
            Edge : constant Real :=
              Driver.Uncertain.Threshold
                (Driver.Uncertain.Vector_Gate (Dimensions, Natural (Real'Floor (Real (Dimensions) * Right))))
              * Sigma;
            Kept : Natural := 0;
         begin
            for I in Errors'First .. Errors'First + Count - 1 loop
               if Errors (I) <= Edge ** 2 then
                  Errors (Errors'First + Kept) := Errors (I);
                  Kept := Kept + 1;
               end if;
            end loop;
            exit when Kept = Count;
            Count := Kept;
            Window := Ball (Dimensions, Edge);
         end;
      end loop;
      Free (Errors);
   end Mixture;

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real) is
      Squared : Real_Access := new Real_Array (1 .. Trips'Length / 2);   --  each round trip's squared length
      Passes  : Natural := 0;
   begin
      for I in Squared'Range loop
         Squared (I) := Trips (Trips'First + 2 * (I - 1)) ** 2 + Trips (Trips'First + 2 * I - 1) ** 2;
      end loop;
      Mixture (Squared.all, 2, Area, Sigma, Right, Passes);
      Free (Squared);
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
      Unplaced      : out Natural;
      Error         : out Real;
      Error_Freedom : out Natural)
   is
      --  The points whose round trips measure the matcher's error: those
      --  around the region, or all of them when nothing around was asked.
      First_Sample : constant Positive := (if Own < Points'Length then Own + 1 else 1);
      Around : Natural := 0;
   begin
      Kept.Clear;
      Error := Real'Last;
      Error_Freedom := 0;
      Apart := 0;
      Unplaced := 0;
      for I in First_Sample .. Points'Length loop
         Around := Around + Boolean'Pos (Answers (Answers'First + I - 1).Found);
      end loop;
      if Around = 0 then
         return;
      end if;
      declare
         Trips      : Real_Access := new Real_Array (1 .. 2 * Around);
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
         Matcher_Error (Trips.all, Real (First.Width) * Real (First.Height), Trip_Sigma, Right);
         Free (Trips);
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
            Candidate : Index_Access := new Index_Array (1 .. Own);
            Off       : Real_Access := new Real_Array (1 .. Own);
            Count     : Natural := 0;
            Diagonal  : constant Real := Sqrt (Real (Second.Width) ** 2 + Real (Second.Height) ** 2);
            Line_Sigma : Real;
            Line_Right : Real;
            procedure Place;
            --  The candidates placed in the scene, by the line error the
            --  ones that passed close tell.
            procedure Place is
            begin
               if Count = 0 then
                  return;
               end if;
               --  The matcher's error across the line the first sight draws in the
               --  second eye, from the matches whose lines pass close, told from
               --  the wrong ones, which pass anywhere within the image: a matcher
               --  can be wrong yet come back, and only the geometry tells.
               declare
                  Squared : Real_Access := new Real_Array (1 .. Count);
                  Passes  : Natural := 0;
               begin
                  for C in Squared'Range loop
                     Squared (C) := Off (C) ** 2;
                  end loop;
                  Mixture (Squared.all, 1, 2.0 * Diagonal, Line_Sigma, Line_Right, Passes);
                  Free (Squared);
               end;
               if not (Line_Sigma < Real'Last) or else Line_Right < 1.0 then
                  return;
               end if;
               Error := Line_Sigma;
               Error_Freedom := Natural (Real'Floor (Line_Right));
               declare
                  --  How far along the first sight a point lies rests on the line
                  --  error's degrees of freedom.
                  Depth_Gate : constant Driver.Uncertain.Gate := Scalar_Gate (Natural (Real'Floor (Line_Right)));
               begin
                  for C in 1 .. Count loop
                     declare
                        I           : constant Positive := Candidate (C);
                        A           : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
                        P           : constant Driver.Images.Pixel := Points (Points'First + I - 1);
                        From_First  : constant Ray_Estimate := First.Ray (P);
                        From_Second : Ray_Estimate := Second.Ray (A.To);
                        Turn        : constant Real := Driver.World.Cameras.Radians_Per_Pixel (Second, A.To);
                        U1          : constant Vec3 := From_First.Direction.Unit_Vector;
                        X           : Point_Estimate;
                        Met         : Boolean;
                     begin
                        From_Second.Direction.Sigma :=
                          Sqrt (From_Second.Direction.Sigma ** 2 + (Line_Sigma * Turn) ** 2);
                        Driver.Geometry.Meet ([From_First, From_Second], X, Met);
                        if Met and then not Finite (X) then
                           --  Met where no number can say: no place either.
                           Unplaced := Unplaced + 1;
                        elsif Met then
                           if Significant
                             (Driver.Distributions.Chi_Square_Deviate (Misfit ([From_First, From_Second], X.Mean),
                                                                       Freedom),
                              1.0)
                           then
                              Apart := Apart + 1;
                           elsif not Significant (Depth_Gate, U1 * (X.Mean - From_First.Origin.Mean),
                                                  Sqrt (Real'Max (0.0, U1 * (X.Covariance * U1))))
                           then
                              --  The two sights are too near parallel to tell how far
                              --  along them the point is: it is no place in the scene.
                              Unplaced := Unplaced + 1;
                           else
                              Kept.Append (Match'(In_First => P, In_Second => A.To, Point => X, First => <>));
                           end if;
                        end if;
                     end;
                  end loop;
               end;
            end Place;
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
            Place;
            Free (Candidate);
            Free (Off);
         end;
      end;
   end Triangulate;

end Driver.World.Pairs;
