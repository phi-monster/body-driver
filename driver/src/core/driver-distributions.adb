with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Distributions is

   use Ada.Numerics.Long_Elementary_Functions;

   Precision : constant Real := Real'Model_Epsilon;
   --  An expansion has converged when its next factor or term changes the
   --  result by no more than this relative amount.

   Tiny : constant Real := Real'Model_Small / Real'Model_Epsilon;
   --  Stands in for a zero denominator in Lentz's method for continued fractions.

   Guard : constant Positive := Real'Mantissa ** 2;
   --  Far more terms than these expansions need at working precision (a few
   --  tens for the arguments used here); it only ends a computation that
   --  would otherwise never end.

   Half : constant := 0.5;

   --  Stirling's series for ln Gamma, ln Gamma (y) = (y - 1/2) ln y - y + ln (2 pi) / 2 + S (y)
   --  with S (y) = sum over k of B_2k / (2k (2k - 1) y^(2k - 1)), the Bernoulli numbers B_2k.
   Bernoulli : constant array (1 .. 9) of Real :=
     [1.0 / 6.0, -1.0 / 30.0, 1.0 / 42.0, -1.0 / 30.0, 5.0 / 66.0, -691.0 / 2730.0, 7.0 / 6.0, -3617.0 / 510.0,
      43867.0 / 798.0];
   --  The last one is not summed: its term bounds what the others leave out.

   Half_Log_Two_Pi : constant := 0.918_938_533_204_672_741_780_329_736_406;
   --  ln (2 pi) / 2.

   function Series_Term (K : Positive; Y : Real) return Real is
     (Bernoulli (K) / (Real (2 * K * (2 * K - 1)) * Y ** (2 * K - 1)));

   function Series (Y : Real) return Real is
      S : Real := 0.0;
   begin
      for K in reverse 1 .. Bernoulli'Last - 1 loop
         S := S + Series_Term (K, Y);
      end loop;
      return S;
   end Series;

   function Converged (Y : Real) return Boolean is
     --  The first term left out is below the rounding of ln Gamma (y).
     (abs Series_Term (Bernoulli'Last, Y) <= Precision * abs ((Y - Half) * Log (Y) - Y + Half_Log_Two_Pi));

   function Log_Gamma (X : Real) return Real is
      --  Raised by Gamma (y + 1) = y Gamma (y) until the series has converged.
      Y     : Real := X;
      Shift : Real := 0.0;
   begin
      while not Converged (Y) loop
         Shift := Shift + Log (Y);
         Y := Y + 1.0;
      end loop;
      return (Y - Half) * Log (Y) - Y + Half_Log_Two_Pi + Series (Y) - Shift;
   end Log_Gamma;

   function Log_Gamma_Halves (Halves : Positive) return Real is (Log_Gamma (Real (Halves) * Half));
   --  ln Gamma (Halves / 2), to rounding and in constant time for any argument.

   function Log_One_Plus (U : Real) return Real is
      --  ln (1 + u) without the cancellation of forming 1 + u first (the
      --  rounding of 1 + u is divided out again).
      W : constant Real := 1.0 + U;
   begin
      if W = 1.0 then
         return U;
      end if;
      return Log (W) * U / (W - 1.0);
   end Log_One_Plus;

   function Log_Beta_Halves (A_Halves, B_Halves : Positive) return Real is
      --  ln B (a, b) = ln Gamma (b) + ln Gamma (a) - ln Gamma (a + b) with a the
      --  larger. When the series has converged at a, the difference of the two
      --  large logarithms is written without them:
      --    -b ln a - (a + b - 1/2) ln (1 + b / a) + b + S (a) - S (a + b),
      --  which keeps the rounding of a quantity like a ln a out of the result.
      A : constant Real := Real (Natural'Max (A_Halves, B_Halves)) * Half;
      B : constant Real := Real (Natural'Min (A_Halves, B_Halves)) * Half;
   begin
      if not Converged (A) then
         return Log_Gamma (A) + Log_Gamma (B) - Log_Gamma (A + B);
      end if;
      return Log_Gamma (B) - B * Log (A) - (A + B - Half) * Log_One_Plus (B / A) + B + Series (A) - Series (A + B);
   end Log_Beta_Halves;

   function Clamp_Away_From_Zero (X : Real) return Real is (if abs X < Tiny then Tiny else X);

   function Upper_Incomplete_Gamma (A_Halves : Positive; X, Log_Gamma_A : Real) return Real;
   --  Q (a, x) = Gamma (a, x) / Gamma (a) with a = A_Halves / 2, given
   --  ln Gamma (a), by the series below a + 1 and by the continued fraction
   --  above it (where each converges quickly).

   function Upper_Incomplete_Gamma (A_Halves : Positive; X, Log_Gamma_A : Real) return Real is
      A     : constant Real := Real (A_Halves) * Half;
      Front : Real;
   begin
      if X <= 0.0 then
         return 1.0;
      end if;
      Front := Exp (-X + A * Log (X) - Log_Gamma_A);
      if X < A + 1.0 then
         declare
            Term : Real := 1.0 / A;
            Sum  : Real := Term;
            Ap   : Real := A;
         begin
            for N in 1 .. Guard loop
               Ap := Ap + 1.0;
               Term := Term * X / Ap;
               Sum := Sum + Term;
               exit when abs Term <= abs Sum * Precision;
            end loop;
            return 1.0 - Sum * Front;
         end;
      end if;
      declare
         B : Real := X + 1.0 - A;
         C : Real := 1.0 / Tiny;
         D : Real := 1.0 / B;
         H : Real := D;
      begin
         for I in 1 .. Guard loop
            declare
               An : constant Real := -Real (I) * (Real (I) - A);
               Factor : Real;
            begin
               B := B + 2.0;
               D := 1.0 / Clamp_Away_From_Zero (An * D + B);
               C := Clamp_Away_From_Zero (B + An / C);
               Factor := D * C;
               H := H * Factor;
               exit when abs (Factor - 1.0) <= Precision;
            end;
         end loop;
         return Front * H;
      end;
   end Upper_Incomplete_Gamma;

   function Beta_Fraction (A, B, X : Real) return Real;
   --  The continued fraction of the incomplete beta function (Lentz's method).

   function Beta_Fraction (A, B, X : Real) return Real is
      Sum_Ab : constant Real := A + B;
      A_Up   : constant Real := A + 1.0;
      A_Down : constant Real := A - 1.0;
      C      : Real := 1.0;
      D      : Real := 1.0 / Clamp_Away_From_Zero (1.0 - Sum_Ab * X / A_Up);
      H      : Real := D;
   begin
      for M in 1 .. Guard loop
         declare
            Mr     : constant Real := Real (M);
            Twice  : constant Real := 2.0 * Mr;
            Even   : constant Real := Mr * (B - Mr) * X / ((A_Down + Twice) * (A + Twice));
            Odd    : constant Real := -(A + Mr) * (Sum_Ab + Mr) * X / ((A + Twice) * (A_Up + Twice));
            Factor : Real;
         begin
            D := 1.0 / Clamp_Away_From_Zero (1.0 + Even * D);
            C := Clamp_Away_From_Zero (1.0 + Even / C);
            H := H * D * C;
            D := 1.0 / Clamp_Away_From_Zero (1.0 + Odd * D);
            C := Clamp_Away_From_Zero (1.0 + Odd / C);
            Factor := D * C;
            H := H * Factor;
            exit when abs (Factor - 1.0) <= Precision;
         end;
      end loop;
      return H;
   end Beta_Fraction;

   function Incomplete_Beta (A_Halves, B_Halves : Positive; X, Log_Beta : Real) return Real;
   --  I_x (a, b) with a = A_Halves / 2 and b = B_Halves / 2, given ln B (a, b);
   --  the fraction is used on the side of the mean where it converges
   --  quickly, and the symmetry I_x (a, b) = 1 - I_(1-x) (b, a) gives the other.

   function Incomplete_Beta (A_Halves, B_Halves : Positive; X, Log_Beta : Real) return Real is
      A : constant Real := Real (A_Halves) * Half;
      B : constant Real := Real (B_Halves) * Half;
   begin
      if X <= 0.0 then
         return 0.0;
      elsif X >= 1.0 then
         return 1.0;
      end if;
      declare
         Front : constant Real := Exp (A * Log (X) + B * Log (1.0 - X) - Log_Beta);
      begin
         if X < (A + 1.0) / (A + B + 2.0) then
            return Front * Beta_Fraction (A, B, X) / A;
         end if;
         return 1.0 - Front * Beta_Fraction (B, A, 1.0 - X) / B;
      end;
   end Incomplete_Beta;

   function Beta_Quantile (Target : Real; A_Halves, B_Halves : Positive) return Real;
   --  The x in (0, 1] with I_x (a, b) = Target (I_x increases with x),
   --  bisected until the bracket cannot shrink.

   function Beta_Quantile (Target : Real; A_Halves, B_Halves : Positive) return Real is
      Log_Beta : constant Real := Log_Beta_Halves (A_Halves, B_Halves);
      Lo  : Real := 0.0;
      Hi  : Real := 1.0;
      Mid : Real;
   begin
      loop
         Mid := Lo + (Hi - Lo) * Half;
         exit when Mid <= Lo or else Mid >= Hi;
         if Incomplete_Beta (A_Halves, B_Halves, Mid, Log_Beta) < Target then
            Lo := Mid;
         else
            Hi := Mid;
         end if;
      end loop;
      --  Hi is never zero; at the end it is one step of working precision from Lo.
      return Hi;
   end Beta_Quantile;

   function Gaussian_Two_Sided_Tail (Z : Real) return Real is
     (Upper_Incomplete_Gamma (1, Z * Z * Half, Log_Gamma_Halves (1)));
   --  P (|X| > z) = erfc (z / sqrt 2) = Q (1/2, z^2 / 2).

   function Chi_Square_Upper_Tail (X : Real; Degrees_Of_Freedom : Positive) return Real is
     (Upper_Incomplete_Gamma (Degrees_Of_Freedom, X * Half, Log_Gamma_Halves (Degrees_Of_Freedom)));
   --  P (Q > x) = Q (k / 2, x / 2).

   function Chi_Square_Quantile (Upper_Tail : Real; Degrees_Of_Freedom : Positive) return Real is
      Log_Gamma_A : constant Real := Log_Gamma_Halves (Degrees_Of_Freedom);
      function Tail (X : Real) return Real is (Upper_Incomplete_Gamma (Degrees_Of_Freedom, X * Half, Log_Gamma_A));
      Lo  : Real := 0.0;
      Hi  : Real := Real (Degrees_Of_Freedom);
      Mid : Real;
   begin
      --  The tail falls from one towards zero; widen from the mean until it
      --  is below the target, then bisect.
      while Tail (Hi) >= Upper_Tail loop
         Lo := Hi;
         Hi := 2.0 * Hi;
      end loop;
      loop
         Mid := Lo + (Hi - Lo) * Half;
         exit when Mid <= Lo or else Mid >= Hi;
         if Tail (Mid) >= Upper_Tail then
            Lo := Mid;
         else
            Hi := Mid;
         end if;
      end loop;
      return Lo;
   end Chi_Square_Quantile;

   function Gaussian_Two_Sided_Quantile (Two_Sided_Tail : Real) return Real is
     (Sqrt (Chi_Square_Quantile (Two_Sided_Tail, 1)));
   --  |X| > z exactly when X^2, a chi-square of one degree, exceeds z^2.

   function Chi_Square_Deviate (X : Real; Degrees_Of_Freedom : Positive) return Real is
      Tail : constant Real := Chi_Square_Upper_Tail (X, Degrees_Of_Freedom);
   begin
      if Tail <= 0.0 then
         return Real'Last;
      end if;
      return Gaussian_Two_Sided_Quantile (Real'Min (1.0, Tail));
   end Chi_Square_Deviate;

   function Student_T_Two_Sided_Tail (T : Real; Degrees_Of_Freedom : Positive) return Real is
      Nu : constant Real := Real (Degrees_Of_Freedom);
   begin
      return Incomplete_Beta (Degrees_Of_Freedom, 1, Nu / (Nu + T * T), Log_Beta_Halves (Degrees_Of_Freedom, 1));
   end Student_T_Two_Sided_Tail;

   function Student_T_Quantile (Two_Sided_Tail : Real; Degrees_Of_Freedom : Positive) return Real is
      --  The tail is I_x (nu / 2, 1 / 2) with x = nu / (nu + t^2). Its bisection
      --  costs time in proportion to nu (the continued fraction near x = 1), so
      --  at large nu the Cornish-Fisher series about the Gaussian quantile
      --  (Abramowitz and Stegun 26.7.5) is used instead, wherever its last term
      --  is below the rounding of the result: there the two agree to rounding.
      Z  : constant Real := Gaussian_Two_Sided_Quantile (Two_Sided_Tail);
      N  : constant Real := Real (Degrees_Of_Freedom);
      First  : constant Real := (Z ** 3 + Z) / 4.0;
      Second : constant Real := (5.0 * Z ** 5 + 16.0 * Z ** 3 + 3.0 * Z) / 96.0;
      Third  : constant Real := (3.0 * Z ** 7 + 19.0 * Z ** 5 + 17.0 * Z ** 3 - 15.0 * Z) / 384.0;
      Fourth : constant Real := (79.0 * Z ** 9 + 776.0 * Z ** 7 + 1482.0 * Z ** 5 - 1920.0 * Z ** 3 - 945.0 * Z) / 92_160.0;
   begin
      if abs (Fourth / N ** 4) <= Real'Epsilon * Z then
         return Z + First / N + Second / N ** 2 + Third / N ** 3 + Fourth / N ** 4;
      end if;
      declare
         X : constant Real := Beta_Quantile (Two_Sided_Tail, Degrees_Of_Freedom, 1);
      begin
         return Sqrt (N * (1.0 - X) / X);
      end;
   end Student_T_Quantile;

   function F_Upper_Tail (F : Real; Numerator, Denominator : Positive) return Real is
      D1 : constant Real := Real (Numerator);
      D2 : constant Real := Real (Denominator);
   begin
      return Incomplete_Beta (Denominator, Numerator, D2 / (D2 + D1 * F), Log_Beta_Halves (Denominator, Numerator));
   end F_Upper_Tail;

   function F_Quantile (Upper_Tail : Real; Numerator, Denominator : Positive) return Real is
      --  The tail is I_x (d2 / 2, d1 / 2) with x = d2 / (d2 + d1 f).
      X : constant Real := Beta_Quantile (Upper_Tail, Denominator, Numerator);
   begin
      return Real (Denominator) * (1.0 - X) / (Real (Numerator) * X);
   end F_Quantile;

end Driver.Distributions;
