with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Distributions;
with Ada.Unchecked_Conversion;
with Ada.Unchecked_Deallocation;
with Interfaces;

package body Driver.Stats is

   use Ada.Numerics.Long_Elementary_Functions;

   Gaussian_MAD_To_Sigma : constant := 1.482_602_218_505_602;
   --  1 / Phi^-1 (3/4): the median absolute deviation of a unit Gaussian.

   procedure Add (A : in out Accumulator; X : Real) is
      Delta_Before : Real;
   begin
      A.N := A.N + 1;
      Delta_Before := X - A.Mean;
      A.Mean := A.Mean + Delta_Before / Real (A.N);
      A.M2 := A.M2 + Delta_Before * (X - A.Mean);
   end Add;

   function Count (A : Accumulator) return Natural is (A.N);

   function Mean (A : Accumulator) return Real is (A.Mean);

   function Variance (A : Accumulator) return Real is (A.M2 / Real (A.N - 1));

   function Mean_Estimate (A : Accumulator) return Driver.Uncertain.Estimate is
   begin
      if A.N < 2 then
         return (Value => (if A.N = 1 then A.Mean else 0.0), Sigma => Real'Last, Degrees_Of_Freedom => 0);
      end if;
      --  The variance is estimated from the N samples less the mean they fixed.
      return (Value => A.Mean, Sigma => Sqrt (Variance (A) / Real (A.N)), Degrees_Of_Freedom => A.N - 1);
   end Mean_Estimate;

   --  Order statistics by the values' bits. Setting the sign bit of a
   --  non-negative IEEE double, and inverting every bit of a negative one,
   --  gives keys that order as the values do, a total order (-0 below +0,
   --  NaNs at the ends). The K-th smallest value is the K-th smallest key,
   --  found by selection on a copy of the keys: three-way partitions about
   --  the median of three keys, each keeping only the part that holds rank K.
   --  The copy is on the heap: samples run to millions (a ratio for every
   --  still cell of every beat), more than a stack holds. The sample itself
   --  is untouched. (A radix selection without a copy, eight passes over the
   --  sample and 256 counters each, cost more than the values for the
   --  hundreds a fit's keyframe medians take: 13% of the arm fit's time in
   --  its profile of 10-15.)

   subtype Key is Interfaces.Unsigned_64;
   use type Key;

   function Bits is new Ada.Unchecked_Conversion (Real, Key);
   function Value_Of is new Ada.Unchecked_Conversion (Key, Real);

   Sign : constant Key := 2 ** (Key'Size - 1);

   function Ordered (V : Real) return Key is
     (if (Bits (V) and Sign) = 0 then Bits (V) or Sign else not Bits (V));

   function Unordered (K : Key) return Real is
     (Value_Of (if (K and Sign) /= 0 then K and not Sign else not K));

   type Key_Array is array (Positive range <>) of Key;
   type Key_Access is access Key_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Key_Array, Key_Access);

   --  The Rank-th smallest of Keys, which it reorders.
   function Select_Rank (Keys : in out Key_Array; Rank : Positive) return Key is
      Lo : Integer := Keys'First;
      Hi : Integer := Keys'Last;
   begin
      loop
         if Lo >= Hi then
            return Keys (Rank);
         end if;
         declare
            Mid   : constant Integer := Lo + (Hi - Lo) / 2;
            A     : constant Key := Keys (Lo);
            B     : constant Key := Keys (Mid);
            C     : constant Key := Keys (Hi);
            Pivot : constant Key :=
              (if A < B then (if B < C then B elsif A < C then C else A)
               else (if A < C then A elsif B < C then C else B));
            Lt    : Integer := Lo;   --  Keys (Lo .. Lt - 1) < Pivot
            I     : Integer := Lo;   --  Keys (Lt .. I - 1) = Pivot
            Gt    : Integer := Hi;   --  Keys (Gt + 1 .. Hi) > Pivot
         begin
            while I <= Gt loop
               declare
                  K : constant Key := Keys (I);
               begin
                  if K < Pivot then
                     Keys (I) := Keys (Lt);
                     Keys (Lt) := K;
                     Lt := Lt + 1;
                     I := I + 1;
                  elsif K > Pivot then
                     Keys (I) := Keys (Gt);
                     Keys (Gt) := K;
                     Gt := Gt - 1;
                  else
                     I := I + 1;
                  end if;
               end;
            end loop;
            if Rank < Lt then
               Hi := Lt - 1;
            elsif Rank > Gt then
               Lo := Gt + 1;
            else
               return Pivot;
            end if;
         end;
      end loop;
   end Select_Rank;

   generic
      with function Value (V : Real) return Real;
   function Median_Of (X : Real_Array) return Real;
   --  The median of Value (V) over X.

   function Median_Of (X : Real_Array) return Real is
      N       : constant Positive := X'Length;
      Keys    : Key_Access := new Key_Array (1 .. N);
      Lower   : Real;
      At_Most : Natural := 0;
      Above   : Real := Real'Last;
   begin
      for I in 0 .. N - 1 loop
         Keys (I + 1) := Ordered (Value (X (X'First + I)));
      end loop;
      Lower := Unordered (Select_Rank (Keys.all, (N + 1) / 2));
      Free (Keys);
      if N mod 2 = 1 then
         return Lower;
      end if;
      --  The next order statistic is Lower again when more than half the
      --  values are at most Lower, else the least value above it.
      for V of X loop
         if Value (V) <= Lower then
            At_Most := At_Most + 1;
         else
            Above := Real'Min (Above, Value (V));
         end if;
      end loop;
      return (Lower + (if At_Most > N / 2 then Lower else Above)) / 2.0;
   end Median_Of;

   function Itself (V : Real) return Real is (V);
   function Median_Of_Values is new Median_Of (Itself);

   function Median (X : Real_Array) return Real is (Median_Of_Values (X));

   function Robust_Sigma (X : Real_Array) return Real is
      Center : constant Real := Median (X);
      function Deviation (V : Real) return Real is (abs (V - Center));
      function Median_Of_Deviations is new Median_Of (Deviation);
   begin
      return Gaussian_MAD_To_Sigma * Median_Of_Deviations (X);
   end Robust_Sigma;

   function Median_Sigma (Sigma : Real; N : Positive) return Real is
   begin
      return Sqrt (Ada.Numerics.Pi / 2.0) * Sigma / Sqrt (Real (N));
   end Median_Sigma;

   function Correlation (X, Y : Real_Array) return Real is
      Mx, My : Accumulator;
      Sxy : Real := 0.0;
      Sxx : Real := 0.0;
      Syy : Real := 0.0;
   begin
      for I in X'Range loop
         Add (Mx, X (I));
         Add (My, Y (I - X'First + Y'First));
      end loop;
      for I in X'Range loop
         declare
            Dx : constant Real := X (I) - Mean (Mx);
            Dy : constant Real := Y (I - X'First + Y'First) - Mean (My);
         begin
            Sxy := Sxy + Dx * Dy;
            Sxx := Sxx + Dx * Dx;
            Syy := Syy + Dy * Dy;
         end;
      end loop;
      if Sxx = 0.0 or else Syy = 0.0 then
         return 0.0;
      end if;
      return Sxy / Sqrt (Sxx * Syy);
   end Correlation;

   function Fit_Line (X, Y : Real_Array) return Line is
      N   : constant Real := Real (X'Length);
      Mx, My : Accumulator;
      Sxx : Real := 0.0;
      Sxy : Real := 0.0;
      Sse : Real := 0.0;
   begin
      for I in X'Range loop
         Add (Mx, X (I));
         Add (My, Y (I - X'First + Y'First));
      end loop;
      for I in X'Range loop
         declare
            Dx : constant Real := X (I) - Mean (Mx);
         begin
            Sxx := Sxx + Dx * Dx;
            Sxy := Sxy + Dx * (Y (I - X'First + Y'First) - Mean (My));
         end;
      end loop;
      if Sxx = 0.0 then
         return (Slope => Driver.Uncertain.Unknown, Intercept => Driver.Uncertain.Unknown,
                 Residual_Sigma => Real'Last);
      end if;
      declare
         B : constant Real := Sxy / Sxx;
         A : constant Real := Mean (My) - B * Mean (Mx);
      begin
         for I in X'Range loop
            Sse := Sse + (Y (I - X'First + Y'First) - (A + B * X (I))) ** 2;
         end loop;
         declare
            S2 : constant Real := Sse / (N - 2.0);
         begin
            --  The residual variance rests on the points less the two parameters they fixed.
            return (Slope          => (Value => B, Sigma => Sqrt (S2 / Sxx), Degrees_Of_Freedom => X'Length - 2),
                    Intercept      => (Value              => A,
                                       Sigma              => Sqrt (S2 * (1.0 / N + Mean (Mx) ** 2 / Sxx)),
                                       Degrees_Of_Freedom => X'Length - 2),
                    Residual_Sigma => Sqrt (S2));
         end;
      end;
   end Fit_Line;

   function Mad_Degrees_Of_Freedom (N : Natural) return Natural is
      Half : constant Real := 0.5;
      C    : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (Half);
      Phi  : constant Real := Ada.Numerics.Long_Elementary_Functions.Exp (-C * C / 2.0)
                              / Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0 * Ada.Numerics.Pi);
   begin
      --  Rounding a handful of samples down to none would call the sigma known.
      return (if N < 2 then 0 else Natural'Max (1, Natural (Real'Floor (8.0 * C * C * Phi * Phi * Real (N)))));
   end Mad_Degrees_Of_Freedom;

   function Latest_Regime (X : Real_Array; Floor : Real := 0.0) return Positive is
      Start : Positive := X'First;
   begin
      loop
         declare
            N      : constant Natural := X'Last - Start + 1;
            N_Old  : constant Natural := N / 2;
            N_New  : constant Natural := N - N_Old;
            Mid    : constant Natural := Start + N_Old;
            Mean_Old, Mean_New : Real := 0.0;
            S_Old, S_New       : Real;
         begin
            exit when N_Old < 2;
            for V of X (Start .. Mid - 1) loop
               Mean_Old := Mean_Old + V;
            end loop;
            for V of X (Mid .. X'Last) loop
               Mean_New := Mean_New + V;
            end loop;
            Mean_Old := Mean_Old / Real (N_Old);
            Mean_New := Mean_New / Real (N_New);
            S_Old := Real'Max (Robust_Sigma (X (Start .. Mid - 1)), Floor);
            S_New := Real'Max (Robust_Sigma (X (Mid .. X'Last)), Floor);
            exit when not Driver.Uncertain.Significant
              (Mean_New - Mean_Old,
               Ada.Numerics.Long_Elementary_Functions.Sqrt (S_Old ** 2 / Real (N_Old) + S_New ** 2 / Real (N_New)),
               Mad_Degrees_Of_Freedom (Natural'Min (N_Old, N_New)));
            Start := Mid;
         end;
      end loop;
      return Start;
   end Latest_Regime;

end Driver.Stats;
