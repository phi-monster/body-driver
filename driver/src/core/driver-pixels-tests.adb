with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Tests;

package body Driver.Pixels.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   Side : constant := 128;

   --  A grey frame: a smooth ramp, plus Lift inside the square [32, 64), plus
   --  Gaussian noise of the given sigma, rounded to whole levels.
   function Frame (Noise, Lift : Real) return Driver.Images.Image is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            declare
               Inside : constant Boolean := Row in 32 .. 63 and then Column in 32 .. 63;
               V : constant Real := 60.0 + Real (Row + Column) * 0.5 + (if Inside then Lift else 0.0) + Noise * Gaussian;
               L : constant Byte := Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K : constant Offset := Offset (3 * (Row * Side + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Frame;

   function View_Of (Frames, Noise, Lift : Real) return View is
      V : View := Empty (Side, Side);
   begin
      for I in 1 .. Natural (Frames) loop
         Add (V, Frame (Noise, Lift));
      end loop;
      return V;
   end View_Of;

   function Changed_Inside (M : Driver.Images.Mask; Inside : Boolean) return Natural is
      N : Natural := 0;
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            if (Row in 32 .. 63 and then Column in 32 .. 63) = Inside and then Driver.Images.Contains (M, Column, Row) then
               N := N + 1;
            end if;
         end loop;
      end loop;
      return N;
   end Changed_Inside;

   procedure Still_Noise is
      --  Two views of four frames each of a still scene with noise of two
      --  levels: every pixel is tested, and at most about 0.27 % should be
      --  called changed. Welch's degrees of freedom, rounded down, make the
      --  test a little stricter than nominal, never looser.
      Nominal : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      Pixels  : constant Real := Real (Side * Side);
      Spread  : constant Real := Sqrt (Nominal * (1.0 - Nominal) / Pixels);
      Total   : Natural := 0;
      Rounds  : constant := 4;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 31);
      for R in 1 .. Rounds loop
         declare
            M : constant Driver.Images.Mask := Changed (View_Of (4.0, 2.0, 0.0), View_Of (4.0, 2.0, 0.0));
         begin
            Total := Total + Driver.Images.Count (M);
         end;
      end loop;
      Check (Real (Total) / (Real (Rounds) * Pixels) <= Nominal + Driver.Conventions.Z * Spread / Sqrt (Real (Rounds)),
             "still pixels called changed:" & Natural'Image (Total) & " of" & Natural'Image (Rounds * Side * Side));
      Check (Total > 0, "no still pixel was ever called changed: the test has lost its threshold");
   end Still_Noise;

   procedure Real_Change is
      M : Driver.Images.Mask;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 37);
      --  Twenty levels against noise of two, fourteen sigmas of the means'
      --  difference: seen almost everywhere in the square (a pixel whose four
      --  frames happen to scatter widely has as few as three degrees of
      --  freedom, and a t of three needs ten sigmas).
      M := Changed (View_Of (4.0, 2.0, 0.0), View_Of (4.0, 2.0, 20.0));
      Check (Changed_Inside (M, True) >= 1000, "a change of fourteen sigmas was missed:"
             & Natural'Image (Changed_Inside (M, True)) & " of 1024");
      Check (Changed_Inside (M, False) < 100, "a change of the square spilled outside it");
   end Real_Change;

   procedure Noiseless_Renders is
      --  A renderer that repeats itself exactly: the variance is the floor,
      --  so a one-level change is three and a half sigmas of two frames.
      M : Driver.Images.Mask;
   begin
      M := Changed (View_Of (2.0, 0.0, 0.0), View_Of (2.0, 0.0, 1.0));
      Check (Changed_Inside (M, True) = 32 * 32 and then Changed_Inside (M, False) = 0,
             "a one-level change of an exact render was not seen exactly:" & Natural'Image (Changed_Inside (M, True))
             & Natural'Image (Changed_Inside (M, False)));
      M := Changed (View_Of (2.0, 0.0, 0.0), View_Of (2.0, 0.0, 0.0));
      Check (Driver.Images.Count (M) = 0, "identical renders called changed");
      --  One frame each: only the floor says how much a pixel varies.
      M := Changed (View_Of (1.0, 0.0, 0.0), View_Of (1.0, 0.0, 1.0));
      Check (Changed_Inside (M, True) = 0, "one frame each claimed a one-level change (2.4 sigmas)");
      M := Changed (View_Of (1.0, 0.0, 0.0), View_Of (1.0, 0.0, 2.0));
      Check (Changed_Inside (M, True) = 32 * 32, "one frame each missed a two-level change (4.9 sigmas)");
   end Noiseless_Renders;

   procedure Bulk_Reads is
      V : constant View := View_Of (4.0, 2.0, 0.0);
      M : Real_Array (1 .. Side * Side);
      S : Real_Array (1 .. Side * Side);
   begin
      Means (V, M);
      Variances (V, S);
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            if M (Row * Side + Column + 1) /= Mean (V, Column, Row)
              or else S (Row * Side + Column + 1) /= Variance (V, Column, Row)
            then
               Check (False, "the whole-view reads differ at column" & Column'Image & ", row" & Row'Image);
               return;
            end if;
         end loop;
      end loop;
   end Bulk_Reads;

   procedure Register is
   begin
      Driver.Tests.Register ("pixels.still", "still pixels are called changed more often than Z promises",
                             Still_Noise'Access);
      Driver.Tests.Register ("pixels.change", "a change far beyond the noise is missed or spills", Real_Change'Access);
      Driver.Tests.Register ("pixels.renders", "exact renders are judged without the quantization floor",
                             Noiseless_Renders'Access);
      Driver.Tests.Register ("pixels.bulk", "the whole-view means or variances differ from the per-pixel ones",
                             Bulk_Reads'Access);
   end Register;

end Driver.Pixels.Tests;
