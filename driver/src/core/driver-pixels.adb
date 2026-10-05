with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Distributions;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Pixels is

   use Ada.Numerics.Long_Elementary_Functions;

   Quantization : constant := 1.0 / 12.0;
   --  The variance of a value rounded to whole levels: a uniform step of one level.

   type Real_Array_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Array_Access);

   function Empty (Width, Height : Positive) return View is
      --  The zero fields are filled on the heap: two VGA fields (4.9 MB) built
      --  as aggregates are stack temporaries, more than a task's stack holds.
      Zeros : Real_Array_Access := new Real_Array (1 .. Width * Height);
   begin
      for Z of Zeros.all loop
         Z := 0.0;
      end loop;
      return Result : constant View :=
        (Width  => Width,
         Height => Height,
         Count  => 0,
         Means  => Real_Holders.To_Holder (Zeros.all),
         Sums   => Real_Holders.To_Holder (Zeros.all))
      do
         Free (Zeros);
      end return;
   end Empty;

   function Width (V : View) return Natural is (V.Width);
   function Height (V : View) return Natural is (V.Height);
   function Frames (V : View) return Natural is (V.Count);

   procedure Add (V : in out View; I : Driver.Images.Image) is
      Mr : constant Real_Holders.Reference_Type := V.Means.Reference;
      Sr : constant Real_Holders.Reference_Type := V.Sums.Reference;
      M  : Real_Array renames Mr.Element.all;
      S  : Real_Array renames Sr.Element.all;
      N  : constant Real := Real (V.Count + 1);
      --  The frame's luma in one pass over its bytes, on the heap: a frame
      --  does not fit on every task's stack.
      Frame : Real_Array_Access := new Real_Array (M'Range);
   begin
      Driver.Images.Luma (I, Frame.all);
      --  Welford's update, one pass and numerically stable.
      for K in M'Range loop
         declare
            Delta_Before : constant Real := Frame (K) - M (K);
         begin
            M (K) := M (K) + Delta_Before / N;
            S (K) := S (K) + Delta_Before * (Frame (K) - M (K));
         end;
      end loop;
      Free (Frame);
      V.Count := V.Count + 1;
   end Add;

   --  Single values are read through references: Holder.Element would copy
   --  the whole frame for every pixel.

   function Mean (V : View; Column, Row : Natural) return Real is
     (V.Means.Constant_Reference.Element (Row * V.Width + Column + 1));

   function Sample_Variance_Of (Sum : Real; Frames : Natural) return Real is
     (if Frames > 1 then Sum / Real (Frames - 1) else 0.0);

   function Floored (Sample : Real) return Real is (Real'Max (Quantization, Sample));

   function Sample_Variance (V : View; K : Positive) return Real is
     (Sample_Variance_Of (V.Sums.Constant_Reference.Element (K), V.Count));

   function Variance (V : View; Column, Row : Natural) return Real is
     (Floored (Sample_Variance (V, Row * V.Width + Column + 1)));

   procedure Means (V : View; Into : out Real_Array) is
      M : constant Real_Holders.Constant_Reference_Type := V.Means.Constant_Reference;
   begin
      Into := M.Element.all;
   end Means;

   procedure Variances (V : View; Into : out Real_Array) is
      S : constant Real_Holders.Constant_Reference_Type := V.Sums.Constant_Reference;
   begin
      for K in 0 .. Into'Length - 1 loop
         Into (Into'First + K) := Floored (Sample_Variance_Of (S.Element (K + 1), V.Count));
      end loop;
   end Variances;

   function Started_Spread (Difference : Real_Array; Centre : Real) return Real is
      --  The spread the pixels near the typical difference show: the lower
      --  quarter of their distances from it, by the median of the nearer half
      --  of the distances. The median distance itself (the usual median
      --  absolute deviation) is the farthest a Gaussian's pixels reach among
      --  their nearer half, so it is carried off by any change as large as the
      --  rest of the view; the lower quarter is still among the unchanged
      --  pixels when they are no more than the nearer half of what is left.
      --  What a Gaussian's lower quarter of absolute values reaches, in
      --  sigmas, is that of the Gaussian whose two-sided tail beyond it is
      --  three quarters.
      Quarter  : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (0.75);
      Away     : Real_Array_Access := new Real_Array (Difference'Range);
      Nearer   : Real_Array_Access := new Real_Array (Difference'Range);
      Count    : Natural := 0;
      Middle   : Real;
      Result   : Real;
   begin
      for K in Difference'Range loop
         Away (K) := abs (Difference (K) - Centre);
      end loop;
      Middle := Driver.Stats.Median (Away.all);
      --  The distances below the median one: an exact render has many equal
      --  ones, and those at the median itself are the nearer half only when
      --  none are below it.
      for V of Away.all loop
         if V < Middle then
            Count := Count + 1;
            Nearer (Nearer'First + Count - 1) := V;
         end if;
      end loop;
      if Count = 0 then
         for V of Away.all loop
            if V <= Middle then
               Count := Count + 1;
               Nearer (Nearer'First + Count - 1) := V;
            end if;
         end loop;
      end if;
      Result := Driver.Stats.Median (Nearer (Nearer'First .. Nearer'First + Count - 1)) / Quarter;
      Free (Away);
      Free (Nearer);
      return Result;
   end Started_Spread;

   function Compare (A, B : View) return Comparison is
      Pixels : constant Positive := A.Width * A.Height;
      --  The least spread of a difference of two means: each is known no
      --  better than the quantization of its frames.
      Floor    : constant Real := Sqrt (Quantization / Real (A.Count) + Quantization / Real (B.Count));
      --  Every pixel is one test of a family of as many as there are pixels.
      Multiple : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Tests => Pixels));
      Ar       : constant Real_Holders.Constant_Reference_Type := A.Means.Constant_Reference;
      Br       : constant Real_Holders.Constant_Reference_Type := B.Means.Constant_Reference;
      --  A frame's worth of reals lives on the heap, not on a task's stack.
      Difference : Real_Array_Access := new Real_Array (1 .. Pixels);
      Centre   : Real;
      Spread   : Real;
      Kept     : Natural := Pixels + 1;   --  how many pixels the last pass kept; no pass keeps more
      Result   : Comparison;
      Changed  : Natural := 0;
   begin
      for K in Difference'Range loop
         Difference (K) := Br.Element (K) - Ar.Element (K);
      end loop;
      Centre := Driver.Stats.Median (Difference.all);
      Spread := Real'Max (Floor, Started_Spread (Difference.all, Centre));
      --  Each pass keeps the pixels within the gate of the spread so far and
      --  takes their mean and standard deviation as the typical difference and
      --  the spread of the pixels that did not change. Keeping fewer and fewer
      --  pixels, it ends when a pass keeps no fewer than the one before.
      loop
         declare
            Seen : Driver.Stats.Accumulator;
         begin
            for D of Difference.all loop
               if abs (D - Centre) <= Multiple * Spread then
                  Driver.Stats.Add (Seen, D);
               end if;
            end loop;
            exit when Driver.Stats.Count (Seen) = 0 or else Driver.Stats.Count (Seen) >= Kept;
            Kept := Driver.Stats.Count (Seen);
            Centre := Driver.Stats.Mean (Seen);
            Spread := Real'Max (Floor, (if Kept > 1 then Sqrt (Driver.Stats.Variance (Seen)) else 0.0));
         end;
      end loop;
      Result.Spread := Spread;
      Result.Beyond := Multiple * Spread;
      Result.Changed := Driver.Images.Create (A.Width, A.Height);
      for K in Difference'Range loop
         if abs (Difference (K) - Centre) > Result.Beyond then
            Changed := Changed + 1;
            Driver.Images.Include (Result.Changed, (K - 1) mod A.Width, (K - 1) / A.Width);
         end if;
      end loop;
      Free (Difference);
      --  The typical difference is the median's to find only while the pixels
      --  that did not change are the majority.
      Result.Trusted := 2 * Changed < Pixels;
      if not Result.Trusted then
         Result.Changed := Driver.Images.Create (A.Width, A.Height);
      end if;
      return Result;
   end Compare;

end Driver.Pixels;
