with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Conversion;
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
   --  gives keys that order as the values do. The K-th smallest value is found
   --  one digit of its key at a time, most significant first, by counting the
   --  values whose keys share the digits found so far: eight passes, nothing
   --  copied, the sample untouched. Samples run to millions (a ratio for every
   --  still cell of every beat), more than a stack holds.

   subtype Key is Interfaces.Unsigned_64;
   use type Key;

   function Bits is new Ada.Unchecked_Conversion (Real, Key);
   function Value_Of is new Ada.Unchecked_Conversion (Key, Real);

   Sign : constant Key := 2 ** (Key'Size - 1);

   Digit_Bits : constant := 8;   --  a byte of the key per pass; any width gives the same result
   Radix      : constant := 2 ** Digit_Bits;

   function Ordered (V : Real) return Key is
     (if (Bits (V) and Sign) = 0 then Bits (V) or Sign else not Bits (V));

   function Unordered (K : Key) return Real is
     (Value_Of (if (K and Sign) /= 0 then K and not Sign else not K));

   generic
      with function Value (V : Real) return Real;
   function Median_Of (X : Real_Array) return Real;
   --  The median of Value (V) over X.

   function Median_Of (X : Real_Array) return Real is
      N       : constant Positive := X'Length;
      Rank    : Positive := (N + 1) / 2;   --  the rank sought among the values that share Prefix
      Prefix  : Key := 0;                  --  the key's digits found so far, in place
      Found   : Key := 0;                  --  which bits of Prefix are found
      Lower   : Real;
      At_Most : Natural := 0;
      Above   : Real := Real'Last;
   begin
      for Place in reverse 0 .. Key'Size / Digit_Bits - 1 loop
         declare
            Unit   : constant Key := Radix ** Place;
            Counts : array (Key range 0 .. Radix - 1) of Natural := [others => 0];
            Digit  : Key := 0;
         begin
            for V of X loop
               declare
                  K : constant Key := Ordered (Value (V));
               begin
                  if (K and Found) = Prefix then
                     Counts (K / Unit mod Radix) := Counts (K / Unit mod Radix) + 1;
                  end if;
               end;
            end loop;
            while Counts (Digit) < Rank loop
               Rank := Rank - Counts (Digit);
               Digit := Digit + 1;
            end loop;
            Prefix := Prefix or Digit * Unit;
            Found := Found or (Radix - 1) * Unit;
         end;
      end loop;
      Lower := Unordered (Prefix);
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

end Driver.Stats;
