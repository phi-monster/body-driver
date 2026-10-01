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

   function Log_Gamma_Halves (Halves : Positive) return Real;
   --  ln Gamma (Halves / 2), exactly by the recurrence Gamma (x + 1) = x Gamma (x)
   --  from Gamma (1/2) = sqrt (pi) or Gamma (1) = 1.

   function Log_Gamma_Halves (Halves : Positive) return Real is
      Sum : Real := (if Halves mod 2 = 1 then Half * Log (Ada.Numerics.Pi) else 0.0);
      J   : Positive := (if Halves mod 2 = 1 then 1 else 2);
   begin
      while J < Halves loop
         Sum := Sum + Log (Real (J) * Half);
         J := J + 2;
      end loop;
      return Sum;
   end Log_Gamma_Halves;

   function Log_Beta_Halves (A_Halves, B_Halves : Positive) return Real is
     (Log_Gamma_Halves (A_Halves) + Log_Gamma_Halves (B_Halves) - Log_Gamma_Halves (A_Halves + B_Halves));

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
      --  The tail is I_x (nu / 2, 1 / 2) with x = nu / (nu + t^2).
      X : constant Real := Beta_Quantile (Two_Sided_Tail, Degrees_Of_Freedom, 1);
   begin
      return Sqrt (Real (Degrees_Of_Freedom) * (1.0 - X) / X);
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
