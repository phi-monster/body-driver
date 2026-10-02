with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
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

   function Mean_Estimate (V : View; Means : Real_Array; K : Positive) return Driver.Uncertain.Estimate is
      Sample : constant Real := Sample_Variance (V, K);
   begin
      --  A variance at the floor is known; one above it rests on the frames less one.
      return (Value              => Means (K),
              Sigma              => Sqrt (Floored (Sample) / Real (V.Count)),
              Degrees_Of_Freedom => (if Sample > Quantization then V.Count - 1 else 0));
   end Mean_Estimate;

   function Changed (A, B : View) return Driver.Images.Mask is
      Result : Driver.Images.Mask := Driver.Images.Create (A.Width, A.Height);
      --  The thresholds are worked out once, up to the two views' degrees of
      --  freedom together. Welch can give more when one variance sits at the
      --  known floor; the threshold for fewer degrees is the larger one, so
      --  taking it there only makes the test stricter.
      Most   : constant Natural := (A.Count - 1) + (B.Count - 1);
      Gates  : array (0 .. Most) of Driver.Uncertain.Gate;
      Ar     : constant Real_Holders.Constant_Reference_Type := A.Means.Constant_Reference;
      Br     : constant Real_Holders.Constant_Reference_Type := B.Means.Constant_Reference;
   begin
      for D in Gates'Range loop
         Gates (D) := Driver.Uncertain.Scalar_Gate (D);
      end loop;
      for Row in 0 .. A.Height - 1 loop
         for Column in 0 .. A.Width - 1 loop
            declare
               K : constant Positive := Row * A.Width + Column + 1;
               D : constant Driver.Uncertain.Estimate :=
                 Driver.Uncertain.Difference (Mean_Estimate (A, Ar.Element.all, K), Mean_Estimate (B, Br.Element.all, K));
            begin
               if Driver.Uncertain.Significant (Gates (Natural'Min (Most, D.Degrees_Of_Freedom)), D.Value, D.Sigma) then
                  Driver.Images.Include (Result, Column, Row);
               end if;
            end;
         end loop;
      end loop;
      return Result;
   end Changed;

end Driver.Pixels;
