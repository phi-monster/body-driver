with Ada.Numerics.Long_Elementary_Functions;

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
         return (Value => (if A.N = 1 then A.Mean else 0.0), Sigma => Real'Last);
      end if;
      return (Value => A.Mean, Sigma => Sqrt (Variance (A) / Real (A.N)));
   end Mean_Estimate;

   procedure Sort (X : in out Real_Array) is
   begin
      --  Heap sort: O(n log n) in place, also for image-sized samples.
      declare
         N : constant Natural := X'Length;
         O : constant Integer := X'First - 1;
         procedure Sift (Start, Stop : Natural) is
            Root : Natural := Start;
            Child : Natural;
            T : Real;
         begin
            loop
               Child := 2 * Root;
               exit when Child > Stop;
               if Child < Stop and then X (O + Child) < X (O + Child + 1) then
                  Child := Child + 1;
               end if;
               exit when not (X (O + Root) < X (O + Child));
               T := X (O + Root);
               X (O + Root) := X (O + Child);
               X (O + Child) := T;
               Root := Child;
            end loop;
         end Sift;
         T : Real;
      begin
         for Start in reverse 1 .. N / 2 loop
            Sift (Start, N);
         end loop;
         for Stop in reverse 2 .. N loop
            T := X (O + 1);
            X (O + 1) := X (O + Stop);
            X (O + Stop) := T;
            Sift (1, Stop - 1);
         end loop;
      end;
   end Sort;

   function Median (X : Real_Array) return Real is
      S : Real_Array := X;
      N : constant Natural := S'Length;
      O : constant Integer := S'First - 1;
   begin
      Sort (S);
      if N mod 2 = 1 then
         return S (O + (N + 1) / 2);
      end if;
      return (S (O + N / 2) + S (O + N / 2 + 1)) / 2.0;
   end Median;

   function Robust_Sigma (X : Real_Array) return Real is
      M : constant Real := Median (X);
      D : Real_Array (X'Range);
   begin
      for I in X'Range loop
         D (I) := abs (X (I) - M);
      end loop;
      return Gaussian_MAD_To_Sigma * Median (D);
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
            return (Slope          => (Value => B, Sigma => Sqrt (S2 / Sxx)),
                    Intercept      => (Value => A, Sigma => Sqrt (S2 * (1.0 / N + Mean (Mx) ** 2 / Sxx))),
                    Residual_Sigma => Sqrt (S2));
         end;
      end;
   end Fit_Line;

end Driver.Stats;
