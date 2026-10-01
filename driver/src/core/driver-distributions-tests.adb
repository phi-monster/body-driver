with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Stats;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Distributions.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use Driver.Uncertain;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   procedure Gaussian_Tail is
   begin
      --  erfc (z / sqrt 2) at a few points (Abramowitz and Stegun, 26.2).
      Check_Close (Gaussian_Two_Sided_Tail (0.0), 1.0, 1.0e-15, "tail at 0");
      Check_Close (Gaussian_Two_Sided_Tail (1.0), 0.317_310_507_862_914_0, 1.0e-14, "tail at 1");
      Check_Close (Gaussian_Two_Sided_Tail (1.959_963_984_540_054), 0.05, 1.0e-14, "tail at 1.96");
      Check_Close (Gaussian_Two_Sided_Tail (3.0), 0.002_699_796_063_260_187, 1.0e-16, "tail at 3");
      Check_Close (Gaussian_Two_Sided_Tail (5.0) / 5.733_031_437_583_878e-7, 1.0, 1.0e-12, "tail at 5 (relative)");
   end Gaussian_Tail;

   procedure T_Quantile is
      Alpha : constant Real := Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
   begin
      --  Two-sided critical values of published t tables.
      Check_Close (Student_T_Quantile (0.05, 1), 12.706_204_736, 1.0e-6, "t 0.05, 1 dof");
      Check_Close (Student_T_Quantile (0.05, 5), 2.570_581_836, 1.0e-8, "t 0.05, 5 dof");
      Check_Close (Student_T_Quantile (0.05, 10), 2.228_138_852, 1.0e-8, "t 0.05, 10 dof");
      Check_Close (Student_T_Quantile (0.01, 4), 4.604_094_871, 1.0e-8, "t 0.01, 4 dof");
      Check_Close (Student_T_Quantile (0.01, 30), 2.749_995_654, 1.0e-8, "t 0.01, 30 dof");
      --  Closed forms at the driver's own tail: one dof is the Cauchy
      --  distribution, tan (pi (1 - a) / 2); two dof give (1 - a) sqrt (2 / (1 - (1 - a)^2)).
      Check_Close (Student_T_Quantile (Alpha, 1) / Tan (Ada.Numerics.Pi * (1.0 - Alpha) / 2.0), 1.0, 1.0e-10,
                   "Cauchy quantile");
      Check_Close (Student_T_Quantile (Alpha, 2), (1.0 - Alpha) * Sqrt (2.0 / (1.0 - (1.0 - Alpha) ** 2)), 1.0e-9,
                   "two-dof quantile");
      Check_Close (Student_T_Two_Sided_Tail (Student_T_Quantile (Alpha, 7), 7), Alpha, 1.0e-15, "tail of the quantile");
      Check_Close (Student_T_Quantile (Alpha, 1_000_000), Driver.Conventions.Z, 1.0e-5, "many dof is Gaussian");
   end T_Quantile;

   procedure Known_Sigma_Is_Z is
   begin
      Check (Significant (3.000_001, 1.0) and then not Significant (2.999_999, 1.0), "known sigma is not Z");
      Check (Significant (3.000_001, 1.0, 0) and then not Significant (2.999_999, 1.0, 0), "0 dof is not Z");
      declare
         T2 : constant Real := Student_T_Quantile (Gaussian_Two_Sided_Tail (Driver.Conventions.Z), 2);
      begin
         Check (T2 > 10.0, "two dof did not widen the threshold");
         Check (Significant (T2 * 1.000_001, 1.0, 2) and then not Significant (T2 * 0.999_999, 1.0, 2),
                "two dof is not the t quantile");
      end;
   end Known_Sigma_Is_Z;

   procedure False_Alarms is
      --  Samples of a Gaussian with mean 0: how often is their mean
      --  significantly different from 0, with its sigma estimated from them?
      Trials : constant := 100_000;
      Nominal : constant Real := Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      Spread : constant Real := Sqrt (Nominal * (1.0 - Nominal) / Real (Trials));
      Sizes : constant array (1 .. 2) of Positive := [3, 6];
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 21);
      for N of Sizes loop
         declare
            With_Dof, Without : Natural := 0;
         begin
            for T in 1 .. Trials loop
               declare
                  A : Driver.Stats.Accumulator;
                  M : Estimate;
               begin
                  for I in 1 .. N loop
                     Driver.Stats.Add (A, Gaussian);
                  end loop;
                  M := Driver.Stats.Mean_Estimate (A);
                  if Significant (M.Value, M.Sigma, M.Degrees_Of_Freedom) then
                     With_Dof := With_Dof + 1;
                  end if;
                  if Significant (M.Value, M.Sigma) then
                     Without := Without + 1;
                  end if;
               end;
            end loop;
            Check (abs (Real (With_Dof) / Real (Trials) - Nominal) <= Driver.Conventions.Z * Spread,
                   "false alarms with" & Integer'Image (N) & " samples:" & Natural'Image (With_Dof) & " of"
                   & Integer'Image (Trials));
            --  Ignoring the degrees of freedom alarms far more often.
            Check (Real (Without) / Real (Trials) > 5.0 * Nominal, "plain Z did not over-alarm with few samples");
         end;
      end loop;
   end False_Alarms;

   procedure Welch is
      function D (Sa : Real; Na : Natural; Sb : Real; Nb : Natural) return Natural is
        (Difference ((Value => 0.0, Sigma => Sa, Degrees_Of_Freedom => Na),
                     (Value => 0.0, Sigma => Sb, Degrees_Of_Freedom => Nb)).Degrees_Of_Freedom);
   begin
      --  (Va + Vb)^2 / (Va^2 / Na + Vb^2 / Nb), rounded down.
      Check (D (1.0, 4, 1.0, 4) = 8, "equal variances of 4 dof each make 8");
      Check (D (1.0, 10, 2.0, 5) = 7, "1 (10 dof) and 4 (5 dof) make 25 / 3.3 = 7.6");
      Check (D (1.0, 0, 1.0, 5) = 20, "a known variance adds to the sum, not to its uncertainty");
      Check (D (1.0, 0, 2.0, 0) = 0, "two known sigmas give a known sigma");
      Check_Close (Difference ((10.0, 3.0, 4), (4.0, 4.0, 0)).Sigma, 5.0, 1.0e-12, "sigmas add in quadrature");
   end Welch;

   procedure Chi_Square_And_F is
      Alpha : constant Real := Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
   begin
      --  Closed forms: two degrees are an exponential, one is a squared Gaussian.
      Check_Close (Chi_Square_Upper_Tail (3.0, 2), Exp (-1.5), 1.0e-15, "chi square 2 dof tail");
      Check_Close (Chi_Square_Upper_Tail (9.0, 1), Gaussian_Two_Sided_Tail (3.0), 1.0e-16, "chi square 1 dof tail");
      Check_Close (Chi_Square_Quantile (Alpha, 2), -2.0 * Log (Alpha), 1.0e-9, "chi square 2 dof quantile");
      --  Published critical values.
      Check_Close (Chi_Square_Quantile (0.05, 3), 7.814_727_903, 1.0e-8, "chi square 0.05, 3 dof");
      Check_Close (Chi_Square_Quantile (0.05, 10), 18.307_038_05, 1.0e-7, "chi square 0.05, 10 dof");
      Check_Close (Chi_Square_Upper_Tail (9.487_729_037, 4), 0.05, 1.0e-10, "chi square tail, 4 dof");
      Check_Close (F_Quantile (0.05, 2, 10), 4.102_821_015, 1.0e-8, "F 0.05, 2 and 10 dof");
      Check_Close (F_Quantile (0.05, 5, 20), 2.710_889_837, 1.0e-8, "F 0.05, 5 and 20 dof");
      --  F with one numerator degree is t squared.
      Check_Close (F_Quantile (Alpha, 1, 6), Student_T_Quantile (Alpha, 6) ** 2, 1.0e-9, "F (1, 6) against t (6)");
      Check_Close (Gaussian_Two_Sided_Quantile (0.05), 1.959_963_985, 1.0e-9, "Gaussian quantile at 0.05");
      Check_Close (Gaussian_Two_Sided_Quantile (Alpha), Driver.Conventions.Z, 1.0e-9, "Gaussian quantile at Z's tail");
      Check_Close (Chi_Square_Deviate (4.0, 1), 2.0, 1.0e-9, "one-dof deviate is the square root");
      Check_Close (Chi_Square_Deviate (Chi_Square_Quantile (Alpha, 5), 5), Driver.Conventions.Z, 1.0e-6,
                   "a chi square at Z's tail has deviate Z");
      Check (Chi_Square_Deviate (1.0e5, 1) = Real'Last, "an underflowing tail is not beyond every deviate");
   end Chi_Square_And_F;

   procedure Vector_False_Alarms is
      --  Two-dimensional Gaussian differences, tested by their length.
      Trials  : constant := 100_000;
      Nominal : constant Real := Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      Spread  : constant Real := Sqrt (Nominal * (1.0 - Nominal) / Real (Trials));
      Known   : constant Gate := Vector_Gate (2);
      Plain   : constant Gate := Scalar_Gate;
      Measured : constant Gate := Vector_Gate (2, 8);
      By_Vector, By_Length, By_Measured : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 23);
      Check_Close (Threshold (Vector_Gate (1)), Driver.Conventions.Z, 1.0e-9, "a one-dimensional vector is a scalar");
      Check_Close (Threshold (Vector_Gate (1, 6)), Threshold (Scalar_Gate (6)), 1.0e-8, "one dimension with 6 dof is t");
      for T in 1 .. Trials loop
         declare
            Length : constant Real := Sqrt (Gaussian ** 2 + Gaussian ** 2);
            Sum    : Real := 0.0;
         begin
            if Significant (Known, Length, 1.0) then
               By_Vector := By_Vector + 1;
            end if;
            if Significant (Plain, Length, 1.0) then
               By_Length := By_Length + 1;
            end if;
            --  The sigma estimated from four other two-dimensional samples (8 dof).
            for I in 1 .. 8 loop
               Sum := Sum + Gaussian ** 2;
            end loop;
            if Significant (Measured, Length, Sqrt (Sum / 8.0)) then
               By_Measured := By_Measured + 1;
            end if;
         end;
      end loop;
      Check (abs (Real (By_Vector) / Real (Trials) - Nominal) <= Driver.Conventions.Z * Spread,
             "two-dimensional differences with a known sigma alarm off the nominal rate:" & Natural'Image (By_Vector));
      Check (abs (Real (By_Measured) / Real (Trials) - Nominal) <= Driver.Conventions.Z * Spread,
             "two-dimensional differences with a measured sigma alarm off the nominal rate:"
             & Natural'Image (By_Measured));
      --  Testing the length against the scalar threshold alarms about four times as often (exp (-4.5)).
      Check (Real (By_Length) / Real (Trials) > 3.0 * Nominal, "the scalar threshold on a length did not over-alarm");
   end Vector_False_Alarms;

   procedure Register is
   begin
      Driver.Tests.Register ("distributions.gaussian", "the Gaussian tail is off its tabulated values",
                             Gaussian_Tail'Access);
      Driver.Tests.Register ("distributions.chi_square", "the chi-square or F tails and quantiles are off their tables",
                             Chi_Square_And_F'Access);
      Driver.Tests.Register ("uncertain.vector", "a vector's length alarms more often than a scalar at the same Z",
                             Vector_False_Alarms'Access);
      Driver.Tests.Register ("distributions.t_quantile", "the t quantile is off its tables or closed forms",
                             T_Quantile'Access);
      Driver.Tests.Register ("uncertain.known_sigma", "a known sigma is not tested against Z, or few dof not widened",
                             Known_Sigma_Is_Z'Access);
      Driver.Tests.Register ("uncertain.false_alarms",
                             "a sigma estimated from a few samples raises more false alarms than Z promises",
                             False_Alarms'Access);
      Driver.Tests.Register ("uncertain.welch", "two estimates' degrees of freedom combine wrongly", Welch'Access);
   end Register;

end Driver.Distributions.Tests;
