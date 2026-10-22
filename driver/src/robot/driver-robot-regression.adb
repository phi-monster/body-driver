with Ada.Containers.Ordered_Maps;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Stats;

package body Driver.Robot.Regression is

   use Ada.Numerics.Long_Elementary_Functions;

   --  Everything sized by the observations lives on the heap: the estimates
   --  also run in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Vector_Access is access Real_Vector;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Vector, Vector_Access);
   type Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);

   Huber_K : constant := Driver.Conventions.Z;
   --  A residual beyond Z sigma is significantly not noise: from there on
   --  its weight falls inversely with its size.

   --  The pseudo-inverse of a symmetric positive semi-definite matrix, and
   --  its numerical rank: eigenvalues below the round-off of the largest
   --  one, accumulated over the dimension, count as zero. The matrix is first
   --  scaled to a unit diagonal (D^-1 A D^-1, inverted back with D^-1 on both
   --  sides), so what counts as round-off does not depend on the units of the
   --  regressors: a column a billion times larger than another must not make
   --  the other one look like zero. Its eigensystem is Small_Eigensystem's, the standard one's arrays on the secondary
   --  stack and its software square root spared: a lock-in of a humanoid's 30 channels took 8.3 s an eye over its 766
   --  weighted solves (h401 on the big machine, 10-22: 608 rows, 255 cells).
   procedure Pseudo_Inverse (A : Real_Matrix; Inverse : out Real_Matrix; Rank : out Natural) is
      N       : constant Natural := A'Length (1);
      Scale   : Real_Vector (1 .. N);
      S       : Real_Matrix (1 .. N, 1 .. N);
      Values  : Real_Vector (1 .. N);
      Vectors : Real_Matrix (1 .. N, 1 .. N);
      Largest : Real := 0.0;
   begin
      Inverse := [others => [others => 0.0]];
      Rank := 0;
      if N = 0 then
         return;
      end if;
      for J in 1 .. N loop
         declare
            D : constant Real := A (A'First (1) + J - 1, A'First (2) + J - 1);
         begin
            Scale (J) := (if D > 0.0 then Sqrt (D) else 1.0);
         end;
      end loop;
      for I in 1 .. N loop
         for J in 1 .. N loop
            S (I, J) := A (A'First (1) + I - 1, A'First (2) + J - 1) / (Scale (I) * Scale (J));
         end loop;
      end loop;
      --  S is symmetric to the bit: A is, and so are the scales' products.
      Driver.Numerics.Dense.Small_Eigensystem (S, Values, Vectors);
      for V of Values loop
         Largest := Real'Max (Largest, abs V);
      end loop;
      if Largest = 0.0 then
         return;
      end if;
      declare
         Floor : constant Real := Largest * Real (N) * Real'Model_Epsilon;
         P     : Real_Matrix (1 .. N, 1 .. N) := [others => [others => 0.0]];
      begin
         for K in 1 .. N loop
            if Values (K) > Floor then
               Rank := Rank + 1;
               for I in 1 .. N loop
                  for J in 1 .. N loop
                     P (I, J) := P (I, J) + Vectors (I, K) * Vectors (J, K) / Values (K);
                  end loop;
               end loop;
            end if;
         end loop;
         for I in 1 .. N loop
            for J in 1 .. N loop
               P (I, J) := P (I, J) / (Scale (I) * Scale (J));
            end loop;
         end loop;
         Inverse := P;
      end;
   end Pseudo_Inverse;

   function Start_Of (X : Real_Matrix) return Start is
      N  : constant Natural := X'Length (1);
      P  : constant Natural := X'Length (2);
      R0 : constant Integer := X'First (1) - 1;
      C0 : constant Integer := X'First (2) - 1;
      Rank : Natural;
   begin
      return Result : Start (P) do
         --  As Solve_On's first pass at weights of one: the same sums in the same order.
         Result.Normal := [others => [others => 0.0]];
         Result.Support := [others => 0];
         for I in 1 .. N loop
            for J in 1 .. P loop
               declare
                  Xj : constant Real := X (R0 + I, C0 + J);
               begin
                  if Xj /= 0.0 then
                     for K in J .. P loop
                        Result.Normal (J, K) := Result.Normal (J, K) + Xj * X (R0 + I, C0 + K);
                     end loop;
                     Result.Support (J) := Result.Support (J) + 1;
                  end if;
               end;
            end loop;
         end loop;
         for J in 1 .. P loop
            for K in 1 .. J - 1 loop
               Result.Normal (J, K) := Result.Normal (K, J);
            end loop;
         end loop;
         Pseudo_Inverse (Result.Normal, Result.Inverse, Rank);
      end return;
   end Start_Of;

   function Solve_On
     (X : Real_Matrix; Y : Real_Array; Floor : Real; W, Residual : in out Real_Array; From : Start) return Fit
   is
      N  : constant Natural := X'Length (1);
      P  : constant Natural := X'Length (2);
      R0 : constant Integer := X'First (1) - 1;
      C0 : constant Integer := X'First (2) - 1;
      Y0 : constant Integer := Y'First - 1;
      B  : Real_Array (1 .. P) := [others => 0.0];
      A  : Real_Matrix (1 .. P, 1 .. P);
      S  : Real := 0.0;
      Converged : Boolean := False;
      Passes    : Natural := 0;
      Inv       : Real_Matrix (1 .. P, 1 .. P);   --  the pseudo-inverse of A, as the latest pass solved with it
      Whole_Rhs : Real_Vector (1 .. P) := [others => 0.0];   --  X' Y at weights of one, the first pass's

      --  The coefficients at the right-hand side, the residuals and their scale: the rest of a pass.
      procedure Finish (Rhs : Real_Vector) is
      begin
         declare
            Beta : constant Real_Vector := Inv * Rhs;
         begin
            for J in 1 .. P loop
               B (J) := Beta (J);
            end loop;
         end;
         for I in 1 .. N loop
            declare
               Predicted : Real := 0.0;
            begin
               for J in 1 .. P loop
                  Predicted := Predicted + X (R0 + I, C0 + J) * B (J);
               end loop;
               Residual (I) := Y (Y0 + I) - Predicted;
            end;
         end loop;
         S := Real'Max (Floor, Driver.Stats.Robust_Sigma (Residual));
      end Finish;

      --  The first pass: the normal equations and their pseudo-inverse at weights of one are X's, made once for it
      --  (Start_Of); the right-hand side alone is Y's.
      procedure First_Pass is
      begin
         for I in 1 .. N loop
            for J in 1 .. P loop
               declare
                  Xj : constant Real := X (R0 + I, C0 + J);
               begin
                  if Xj /= 0.0 then
                     Whole_Rhs (J) := Whole_Rhs (J) + Xj * Y (Y0 + I);
                  end if;
               end;
            end loop;
         end loop;
         A := From.Normal;
         Inv := From.Inverse;
         Finish (Whole_Rhs);
      end First_Pass;

      --  The normal equations at the current weights: those at weights of one less, for every observation its weight
      --  took something from, what it took. Huber's weights leave whole every observation within Z of the scale, and
      --  summed over every observation at every pass the equations were most of the lock-in's time once its rows grew:
      --  12000 rows of 31 columns, the sums of 31 by 31 products, at each of a cell's passes (h403 and h405, humanoids,
      --  the big machine, 10-22). A column left with no observation of weight is nought, as the weighted sums made it.
      procedure Weighted_Solve is
         Rhs  : Real_Vector := Whole_Rhs;
         Left : Count_Array := From.Support;
         Rank : Natural;
      begin
         A := From.Normal;
         for I in 1 .. N loop
            if W (I) < 1.0 then
               declare
                  Taken : constant Real := 1.0 - W (I);
               begin
                  for J in 1 .. P loop
                     declare
                        Xj : constant Real := X (R0 + I, C0 + J);
                     begin
                        if Xj /= 0.0 then
                           Rhs (J) := Rhs (J) - Taken * Xj * Y (Y0 + I);
                           for K in J .. P loop
                              A (J, K) := A (J, K) - Taken * Xj * X (R0 + I, C0 + K);
                           end loop;
                           if W (I) = 0.0 then
                              Left (J) := Left (J) - 1;
                           end if;
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end loop;
         for J in 1 .. P loop
            if Left (J) = 0 then
               Rhs (J) := 0.0;
               for K in 1 .. P loop
                  A (J, K) := 0.0;
                  A (K, J) := 0.0;
               end loop;
            end if;
         end loop;
         for J in 1 .. P loop
            for K in 1 .. J - 1 loop
               A (J, K) := A (K, J);
            end loop;
         end loop;
         Pseudo_Inverse (A, Inv, Rank);
         Finish (Rhs);
      end Weighted_Solve;

      function Weight (R : Real) return Real is
      begin
         if S = 0.0 then
            --  More than half the residuals vanish exactly and there is no
            --  floor: only exact fits count.
            return (if R = 0.0 then 1.0 else 0.0);
         end if;
         declare
            U : constant Real := abs R / S;
         begin
            return (if U <= Huber_K then 1.0 else Huber_K / U);
         end;
      end Weight;

   begin
      First_Pass;
      --  One pass per observation bounds the iterations; they stop as soon
      --  as the coefficients stop changing.
      for Pass in 1 .. N loop
         declare
            Before : constant Real_Array := B;
            Size, Moved : Real := 0.0;
         begin
            for I in 1 .. N loop
               W (I) := Weight (Residual (I));
            end loop;
            Weighted_Solve;
            Passes := Passes + 1;
            for J in 1 .. P loop
               Size := Real'Max (Size, abs B (J));
               Moved := Real'Max (Moved, abs (B (J) - Before (J)));
            end loop;
            if Moved <= Driver.Conventions.Unchanged_Fraction * Size then
               Converged := True;
               exit;
            end if;
         end;
      end loop;
      return (Columns => P, Beta => B, Scale => S, Normal => A, Inverse => Inv, Converged => Converged,
              Passes => Passes);
   end Solve_On;

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real) return Fit is
   begin
      return Solve (X, Y, Floor, Start_Of (X));
   end Solve;

   function Solve (X : Real_Matrix; Y : Real_Array; Floor : Real; From : Start) return Fit is
      W        : Real_Access := new Real_Array'(1 .. X'Length (1) => 1.0);
      Residual : Real_Access := new Real_Array (1 .. X'Length (1));
      Result   : constant Fit := Solve_On (X, Y, Floor, W.all, Residual.all, From);
   begin
      Free (W);
      Free (Residual);
      return Result;
   end Solve;

   --  Each count asked once: the answer is a pure function of the four arguments, and the lock-in asks at every
   --  beat of every eye whether the cells beyond the flow's reach are a significant majority (Mark_Reach).
   type Count_Key is record
      Count, Trials : Natural;
      Rate          : Real;
      Tests         : Positive;
   end record;
   function "<" (A, B : Count_Key) return Boolean is
     (A.Count < B.Count
      or else (A.Count = B.Count
               and then (A.Trials < B.Trials
                         or else (A.Trials = B.Trials
                                  and then (A.Rate < B.Rate or else (A.Rate = B.Rate and then A.Tests < B.Tests))))));
   package Count_Maps is new Ada.Containers.Ordered_Maps (Count_Key, Boolean);
   protected Counts_Known is
      procedure Get (Key : Count_Key; Value : out Boolean; Found : out Boolean);
      procedure Keep (Key : Count_Key; Value : Boolean);
   private
      Map : Count_Maps.Map;
   end Counts_Known;
   protected body Counts_Known is
      procedure Get (Key : Count_Key; Value : out Boolean; Found : out Boolean) is
         C : constant Count_Maps.Cursor := Map.Find (Key);
      begin
         Found := Count_Maps.Has_Element (C);
         Value := Found and then Count_Maps.Element (C);
      end Get;
      procedure Keep (Key : Count_Key; Value : Boolean) is
      begin
         Map.Include (Key, Value);
      end Keep;
   end Counts_Known;

   function Count_Significant_Computed (Count, Trials : Natural; Rate : Real; Tests : Positive) return Boolean;

   function Count_Significant (Count, Trials : Natural; Rate : Real; Tests : Positive := 1) return Boolean is
      Key   : constant Count_Key := (Count, Trials, Rate, Tests);
      Value, Found : Boolean;
   begin
      Counts_Known.Get (Key, Value, Found);
      if not Found then
         Value := Count_Significant_Computed (Count, Trials, Rate, Tests);
         Counts_Known.Keep (Key, Value);
      end if;
      return Value;
   end Count_Significant;

   function Count_Significant_Computed (Count, Trials : Natural; Rate : Real; Tests : Positive) return Boolean is
   begin
      if Count = 0 then
         return False;
      end if;
      --  P (X >= k) for X binomial (n, p) is P (F < (n - k + 1) p / (k (1 - p)))
      --  for F with 2 k and 2 (n - k + 1) degrees of freedom.
      declare
         K : constant Real := Real (Count);
         N : constant Real := Real (Trials);
         Tail : constant Real :=
           1.0 - Driver.Distributions.F_Upper_Tail
                   ((N - K + 1.0) * Rate / (K * (1.0 - Rate)), 2 * Count, 2 * (Trials - Count + 1));
      begin
         return Tail < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z) / Real (Tests);
      end;
   end Count_Significant_Computed;

   function Explained_On
     (X : Real_Matrix; Y : Real_Array; Xc : in out Real_Matrix; Yc : in out Real_Vector; Used : out Natural) return Real
   is
      N  : constant Natural := X'Length (1);
      P  : constant Natural := X'Length (2);
      R0 : constant Integer := X'First (1) - 1;
      C0 : constant Integer := X'First (2) - 1;
      Y0 : constant Integer := Y'First - 1;
      B  : Real_Vector (1 .. P) := [others => 0.0];
      Positive_Set : array (1 .. P) of Boolean := [others => False];
      Total : Real := 0.0;

      --  The least-squares coefficients on the columns in the positive set,
      --  zero elsewhere.
      function Restricted return Real_Vector is
         A    : Real_Matrix (1 .. P, 1 .. P) := [others => [others => 0.0]];
         R    : Real_Vector (1 .. P) := [others => 0.0];
         Inv  : Real_Matrix (1 .. P, 1 .. P);
         Rank : Natural;
      begin
         for J in 1 .. P loop
            if Positive_Set (J) then
               for I in 1 .. N loop
                  R (J) := R (J) + Xc (I, J) * Yc (I);
               end loop;
               for K in 1 .. P loop
                  if Positive_Set (K) then
                     for I in 1 .. N loop
                        A (J, K) := A (J, K) + Xc (I, J) * Xc (I, K);
                     end loop;
                  end if;
               end loop;
            end if;
         end loop;
         Pseudo_Inverse (A, Inv, Rank);
         return Inv * R;
      end Restricted;

      function Residual_Sum return Real is
         S : Real := 0.0;
      begin
         for I in 1 .. N loop
            declare
               F : Real := 0.0;
            begin
               for J in 1 .. P loop
                  F := F + Xc (I, J) * B (J);
               end loop;
               S := S + (Yc (I) - F) ** 2;
            end;
         end loop;
         return S;
      end Residual_Sum;
   begin
      Used := 0;
      if N = 0 then
         return 0.0;
      end if;
      declare
         Mean : Real := 0.0;
      begin
         for I in 1 .. N loop
            Mean := Mean + Y (Y0 + I);
         end loop;
         Mean := Mean / Real (N);
         for I in 1 .. N loop
            Yc (I) := Y (Y0 + I) - Mean;
            Total := Total + Yc (I) ** 2;
         end loop;
      end;
      for J in 1 .. P loop
         declare
            Mean : Real := 0.0;
         begin
            for I in 1 .. N loop
               Mean := Mean + X (R0 + I, C0 + J);
            end loop;
            Mean := Mean / Real (N);
            for I in 1 .. N loop
               Xc (I, J) := X (R0 + I, C0 + J) - Mean;
            end loop;
         end;
      end loop;
      if Total = 0.0 then
         return 0.0;
      end if;
      --  Each round admits the column that most lowers the residual; a
      --  column leaves when keeping it would need a negative coefficient.
      --  Every column enters and leaves at most once per round, so as many
      --  rounds as there are columns, squared, bound it.
      for Round in 1 .. P * P loop
         declare
            Best  : Natural := 0;
            Slope : Real := 0.0;
         begin
            for J in 1 .. P loop
               if not Positive_Set (J) then
                  declare
                     W : Real := 0.0;
                  begin
                     for I in 1 .. N loop
                        declare
                           F : Real := 0.0;
                        begin
                           for K in 1 .. P loop
                              F := F + Xc (I, K) * B (K);
                           end loop;
                           W := W + Xc (I, J) * (Yc (I) - F);
                        end;
                     end loop;
                     if W > Slope then
                        Slope := W;
                        Best := J;
                     end if;
                  end;
               end if;
            end loop;
            exit when Best = 0;
            Positive_Set (Best) := True;
            for Inner in 1 .. P loop
               declare
                  S : constant Real_Vector := Restricted;
                  Alpha : Real := 1.0;
                  Feasible : Boolean := True;
               begin
                  for J in 1 .. P loop
                     if Positive_Set (J) and then S (J) <= 0.0 then
                        Feasible := False;
                        if B (J) - S (J) > 0.0 then
                           Alpha := Real'Min (Alpha, B (J) / (B (J) - S (J)));
                        end if;
                     end if;
                  end loop;
                  if Feasible then
                     B := S;
                     exit;
                  end if;
                  for J in 1 .. P loop
                     if Positive_Set (J) then
                        B (J) := B (J) + Alpha * (S (J) - B (J));
                        if B (J) <= 0.0 then
                           B (J) := 0.0;
                           Positive_Set (J) := False;
                        end if;
                     end if;
                  end loop;
               end;
            end loop;
         end;
      end loop;
      for J in 1 .. P loop
         if Positive_Set (J) and then B (J) > 0.0 then
            Used := Used + 1;
         end if;
      end loop;
      return Real'Max (0.0, 1.0 - Residual_Sum / Total);
   end Explained_On;

   function Explained_Nonnegative (X : Real_Matrix; Y : Real_Array; Used : out Natural) return Real is
      Xc     : Matrix_Access := new Real_Matrix (1 .. X'Length (1), 1 .. X'Length (2));
      Yc     : Vector_Access := new Real_Vector (1 .. X'Length (1));
      Result : constant Real := Explained_On (X, Y, Xc.all, Yc.all, Used);
   begin
      Free (Xc);
      Free (Yc);
      return Result;
   end Explained_Nonnegative;

   function Explained_Nonnegative
     (Gram : Real_Matrix; Cross : Real_Vector; Total : Real; Used : out Natural) return Real
   is
      P : constant Natural := Cross'Length;
      B : Real_Vector (1 .. P) := [others => 0.0];
      Positive_Set : array (1 .. P) of Boolean := [others => False];

      function G (J, K : Positive) return Real is (Gram (Gram'First (1) + J - 1, Gram'First (2) + K - 1));
      function C (J : Positive) return Real is (Cross (Cross'First + J - 1));

      --  The least-squares coefficients on the columns in the positive set, zero elsewhere.
      function Restricted return Real_Vector is
         A    : Real_Matrix (1 .. P, 1 .. P) := [others => [others => 0.0]];
         R    : Real_Vector (1 .. P) := [others => 0.0];
         Inv  : Real_Matrix (1 .. P, 1 .. P);
         Rank : Natural;
      begin
         for J in 1 .. P loop
            if Positive_Set (J) then
               R (J) := C (J);
               for K in 1 .. P loop
                  if Positive_Set (K) then
                     A (J, K) := G (J, K);
                  end if;
               end loop;
            end if;
         end loop;
         Pseudo_Inverse (A, Inv, Rank);
         return Inv * R;
      end Restricted;

      --  The squares of Y about its mean the fit leaves: Total less twice B against Cross plus B against Gram.
      function Residual_Sum return Real is
         S : Real := Total;
      begin
         for J in 1 .. P loop
            S := S - 2.0 * B (J) * C (J);
            for K in 1 .. P loop
               S := S + B (J) * G (J, K) * B (K);
            end loop;
         end loop;
         return S;
      end Residual_Sum;
   begin
      Used := 0;
      if Total <= 0.0 then
         return 0.0;
      end if;
      --  As Explained_On: each round admits the column that most lowers the residual; a column leaves when keeping
      --  it would need a negative coefficient.
      for Round in 1 .. P * P loop
         declare
            Best  : Natural := 0;
            Slope : Real := 0.0;
         begin
            for J in 1 .. P loop
               if not Positive_Set (J) then
                  declare
                     W : Real := C (J);
                  begin
                     for K in 1 .. P loop
                        W := W - G (J, K) * B (K);
                     end loop;
                     if W > Slope then
                        Slope := W;
                        Best := J;
                     end if;
                  end;
               end if;
            end loop;
            exit when Best = 0;
            Positive_Set (Best) := True;
            for Inner in 1 .. P loop
               declare
                  S : constant Real_Vector := Restricted;
                  Alpha : Real := 1.0;
                  Feasible : Boolean := True;
               begin
                  for J in 1 .. P loop
                     if Positive_Set (J) and then S (J) <= 0.0 then
                        Feasible := False;
                        if B (J) - S (J) > 0.0 then
                           Alpha := Real'Min (Alpha, B (J) / (B (J) - S (J)));
                        end if;
                     end if;
                  end loop;
                  if Feasible then
                     B := S;
                     exit;
                  end if;
                  for J in 1 .. P loop
                     if Positive_Set (J) then
                        B (J) := B (J) + Alpha * (S (J) - B (J));
                        if B (J) <= 0.0 then
                           B (J) := 0.0;
                           Positive_Set (J) := False;
                        end if;
                     end if;
                  end loop;
               end;
            end loop;
         end;
      end loop;
      for J in 1 .. P loop
         if Positive_Set (J) and then B (J) > 0.0 then
            Used := Used + 1;
         end if;
      end loop;
      return Real'Max (0.0, 1.0 - Residual_Sum / Total);
   end Explained_Nonnegative;

   function Covariance_Of (F : Fit) return Real_Matrix is
      Result : Real_Matrix (1 .. F.Columns, 1 .. F.Columns);
   begin
      --  The pseudo-inverse of Normal the final pass solved with, the same to the bit as made again from it.
      for I in Result'Range (1) loop
         for J in Result'Range (2) loop
            Result (I, J) := F.Scale * F.Scale * F.Inverse (I, J);
         end loop;
      end loop;
      return Result;
   end Covariance_Of;

   function Coefficient_Variances (F : Fit; Cov : Real_Matrix) return Real_Array is
      Result : Real_Array (1 .. F.Columns);
   begin
      for J in Result'Range loop
         Result (J) := Cov (Cov'First (1) + J - 1, Cov'First (2) + J - 1);
      end loop;
      return Result;
   end Coefficient_Variances;

   function Coefficient_Variances (F : Fit) return Real_Array is (Coefficient_Variances (F, Covariance_Of (F)));

   procedure Test_Block
     (F : Fit; Cov : Real_Matrix; First, Last : Positive; Statistic : out Real; Freedom : out Natural)
   is
      K     : constant Natural := Last - First + 1;
      Block : Real_Matrix (1 .. K, 1 .. K);
      Cinv  : Real_Matrix (1 .. K, 1 .. K);
      Beta  : Real_Vector (1 .. K);
   begin
      for I in 1 .. K loop
         Beta (I) := F.Beta (First + I - 1);
         for J in 1 .. K loop
            Block (I, J) := Cov (Cov'First (1) + First + I - 2, Cov'First (2) + First + J - 2);
         end loop;
      end loop;
      Pseudo_Inverse (Block, Cinv, Freedom);
      Statistic := (if Freedom = 0 then 0.0 else Beta * (Cinv * Beta));
   end Test_Block;

   procedure Test_Block (F : Fit; First, Last : Positive; Statistic : out Real; Freedom : out Natural) is
   begin
      Test_Block (F, Covariance_Of (F), First, Last, Statistic, Freedom);
   end Test_Block;

end Driver.Robot.Regression;
