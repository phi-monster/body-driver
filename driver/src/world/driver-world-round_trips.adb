with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Uncertain;

package body Driver.World.Round_Trips is

   use Ada.Numerics.Long_Elementary_Functions;

   --  Everything sized by matches lives on the heap: the estimates also run
   --  in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

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

end Driver.World.Round_Trips;
