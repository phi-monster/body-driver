with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Clock;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics;
with Driver.Numerics.Dense;

package body Driver.Alignment is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   subtype Real_Access is Reals_Access;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Reals_Access);
   procedure Free_Counted is new Ada.Unchecked_Deallocation (Counted, Counted_Access);

   overriding procedure Adjust (Item : in out Shared_Reals) is
   begin
      if Item.Held /= null then
         System.Atomic_Counters.Increment (Item.Held.Count);
      end if;
   end Adjust;

   overriding procedure Finalize (Item : in out Shared_Reals) is
   begin
      if Item.Held /= null then
         if System.Atomic_Counters.Decrement (Item.Held.Count) then
            Free (Item.Held.Values);
            Free_Counted (Item.Held);
         end if;
         Item.Held := null;
      end if;
   end Finalize;

   --  A copy of the values, made to be shared and not written again.
   function Shared (Values : Real_Array) return Shared_Reals is
     (Ada.Finalization.Controlled with Held => new Counted'(Count => <>, Values => new Real_Array'(Values)));

   --  The values themselves, made to be shared and not written again; the caller's access is given up.
   procedure Adopt (Item : out Shared_Reals; Values : in out Real_Access) is
   begin
      Item := (Ada.Finalization.Controlled with Held => new Counted'(Count => <>, Values => Values));
      Values := null;
   end Adopt;

   type Mask_Array is array (Positive range <>) of Boolean;

   Parameters : constant := 8;
   --  The warp: the place (2), the linear part (4), the light's gain and offset (2).

   Least_Half : constant := 2;
   --  A patch reaches at least this many pixels either side of its point: a central difference reaches one
   --  pixel and bilinear interpolation another.

   function Side_Of (Half : Natural) return Positive is (2 * Half + 1);
   --  The side of a patch that reaches Half pixels either side of its point.

   Least_Side : constant Positive := Side_Of (Least_Half);

   Most_Variants : constant := 3;
   --  The other fits of a patch an answer's covariance rests on: its middle at the finest size, and the next two
   --  coarser sizes.

   Z : constant Real := Driver.Conventions.Z;

   Light_Noise_Floor : constant Real := 2.0 * Quantization;
   --  The variance of the difference of two rounded levels: what two noiseless pictures cannot agree on.

   ---------------------------------------------------------------------------
   --  The pictures

   function Half_Patch (Width, Height : Natural) return Natural is
      --  The patch is as many pixels wide as the geometric middle between a pixel and the picture's size
      --  (Driver.Robot.Flow's cells), half of it either side of the point.
      Side : constant Real := Sqrt (Sqrt (Real (Width) * Real (Height)));
   begin
      return Natural (Real'Rounding (Side / 2.0));
   end Half_Patch;

   --  The pole of the cubic B-spline's interpolation filter, and the gain that keeps a constant a constant.
   Pole : constant Real := Sqrt (3.0) - 2.0;
   Gain : constant Real := 6.0;

   --  The coefficients of the cubic B-spline through a line of samples (Unser's recursive filter, pole
   --  Pole): the samples are the spline's values at the pixel centres, so reading the spline between them is
   --  interpolation that agrees with every pixel exactly. The line is Count values from index First by Step;
   --  beyond its ends it is taken to repeat itself mirrored, which only the pixels within two of an edge feel
   --  (and those are never read).
   procedure Filter_Line (C : in out Real_Array; First, Step, Count : Natural) is
      --  How many samples the first coefficient depends on to working precision: Pole ** Horizon is below epsilon.
      Horizon : constant Natural := Natural'Min (Count, Natural (Real'Ceiling (Log (Real'Epsilon) / Log (abs Pole))));
      Plus    : Real_Array (1 .. Count);
      Sum     : Real := 0.0;
      Power   : Real := 1.0;
   begin
      if Count < 2 then
         return;
      end if;
      for K in 0 .. Horizon - 1 loop
         Sum := Sum + Power * C (First + K * Step);
         Power := Power * Pole;
      end loop;
      Plus (1) := Gain * Sum;
      for K in 2 .. Count loop
         Plus (K) := Gain * C (First + (K - 1) * Step) + Pole * Plus (K - 1);
      end loop;
      C (First + (Count - 1) * Step) := Pole / (Pole ** 2 - 1.0) * (Plus (Count) + Pole * Plus (Count - 1));
      for K in reverse 0 .. Count - 2 loop
         C (First + K * Step) := Pole * (C (First + (K + 1) * Step) - Plus (K + 1));
      end loop;
   end Filter_Line;

   function Make_Level (Pixels : Real_Array; Width, Height : Natural) return Level is
      Spline : Real_Access := new Real_Array'(Pixels);
      Result : Level;
   begin
      for Row in 0 .. Height - 1 loop
         Filter_Line (Spline.all, Spline'First + Row * Width, 1, Width);
      end loop;
      for Column in 0 .. Width - 1 loop
         Filter_Line (Spline.all, Spline'First + Column, Width, Height);
      end loop;
      Result := (Width => Width, Height => Height, Luma => Shared (Pixels), Spline => <>);
      Adopt (Result.Spline, Spline);
      return Result;
   end Make_Level;

   procedure Halve
     (Pixels                : Real_Array;
      Width, Height         : Natural;
      Into                  : out Real_Access;
      New_Width, New_Height : out Natural)
   is
      --  Each pixel of the half-size picture is the binomial average of four by four of the full-size one centred
      --  on the corner the two by two it replaces meet at, which keeps the corner-origin coordinates of both
      --  pictures the same (a place is divided by two and nothing more) and leaves nothing a halving would alias.
      Taps : constant array (-1 .. 2) of Real := [1.0 / 8.0, 3.0 / 8.0, 3.0 / 8.0, 1.0 / 8.0];
   begin
      New_Width := Width / Halving;
      New_Height := Height / Halving;
      Into := new Real_Array (1 .. New_Width * New_Height);
      for R in 0 .. New_Height - 1 loop
         for C in 0 .. New_Width - 1 loop
            declare
               Sum : Real := 0.0;
            begin
               for B in -1 .. 2 loop
                  for A in -1 .. 2 loop
                     declare
                        X : constant Integer := Integer'Max (0, Integer'Min (Width - 1, Halving * C + A));
                        Y : constant Integer := Integer'Max (0, Integer'Min (Height - 1, Halving * R + B));
                     begin
                        Sum := Sum + Taps (A) * Taps (B) * Pixels (Pixels'First + Y * Width + X);
                     end;
                  end loop;
               end loop;
               Into (R * New_Width + C + 1) := Sum;
            end;
         end loop;
      end loop;
   end Halve;

   function Pyramid_Of (I : Driver.Images.Image) return Pyramid is
      Width  : constant Natural := Driver.Images.Width (I);
      Height : constant Natural := Driver.Images.Height (I);
   begin
      if Width = 0 or else Height = 0 then
         return Result : Pyramid;
      end if;
      declare
         Luma : Real_Access := new Real_Array (1 .. Width * Height);
      begin
         Driver.Images.Luma (I, Luma.all);
         return Result : constant Pyramid := Pyramid_Of (Width, Height, Luma.all) do
            Free (Luma);
         end return;
      end;
   end Pyramid_Of;

   function Pyramid_Of (Width, Height : Natural; Luma : Real_Array) return Pyramid is
      Result  : Pyramid;
      Width_Now  : Natural := Width;
      Height_Now : Natural := Height;
      Current : Real_Access;
   begin
      if Width = 0 or else Height = 0 then
         return Result;
      end if;
      Current := new Real_Array (1 .. Width * Height);
      Current.all := Luma;
      loop
         Result.Sizes.Append (Make_Level (Current.all, Width_Now, Height_Now));
         exit when Half_Patch (Width_Now / Halving, Height_Now / Halving) < Least_Half
           or else Width_Now / Halving < Least_Side or else Height_Now / Halving < Least_Side;
         declare
            Next : Real_Access;
            Next_Width, Next_Height : Natural;
         begin
            Halve (Current.all, Width_Now, Height_Now, Next, Next_Width, Next_Height);
            Free (Current);
            Current := Next;
            Width_Now := Next_Width;
            Height_Now := Next_Height;
         end;
      end loop;
      Free (Current);
      return Result;
   end Pyramid_Of;

   function Levels (P : Pyramid) return Natural is (Natural (P.Sizes.Length));

   --  The value of a picture at a place (corner-origin coordinates) by the cubic B-spline whose coefficients are
   --  given, and its derivatives along u and v; not Ok when the sixteen coefficients around the place are not all
   --  in the picture. A picture read between its pixels by the spline through them is smooth to the second
   --  derivative and agrees with every pixel exactly; a bilinear reading has a corner at every pixel centre, which
   --  a fit lands on and bounces across when two pictures agree exactly, and a cubic convolution reads a texture
   --  of three pixels' period (the finest the pixels hold) wrong by a tenth of its strength at the places between
   --  pixels, a phase-dependent error that decides between two periods.
   procedure Sample
     (Coefficients  : Real_Array;
      Width, Height : Natural;
      U, V          : Real;
      Value, Dx, Dy : out Real;
      Ok            : out Boolean)
   is
      X : constant Real := U - 0.5;
      Y : constant Real := V - 0.5;

      --  The whole part of a place at or beyond 1: the conversion to an integer rounds to the nearest, which is
      --  the whole part or one above it. Real'Floor calls the run-time's software attribute (System.Fat_Lflt), and
      --  the spline is sampled at every pixel of every pass: 1532 of the 25366 working samples of s07's estimate
      --  at beat 2842 (replayed, 10-16) were in that call.
      function Whole (P : Real) return Natural is
         N : constant Natural := Natural (P);
      begin
         return (if Real (N) > P then N - 1 else N);
      end Whole;
   begin
      Value := 0.0;
      Dx := 0.0;
      Dy := 0.0;
      if X < 1.0 or else Y < 1.0 or else X >= Real (Width - 2) or else Y >= Real (Height - 2) then
         Ok := False;
         return;
      end if;
      declare
         I  : constant Natural := Whole (X);
         J  : constant Natural := Whole (Y);
         Fx : constant Real := X - Real (I);
         Fy : constant Real := Y - Real (J);
         type Taps is array (0 .. 3) of Real;
         Wx, Sx, Wy, Sy : Taps;   --  weights and slopes of the taps one before to two after

         --  The pieces of the cubic B-spline (Basis, Basis_Slope) at T + 1, T, T - 1 and T - 2, written out: the
         --  weights and slopes of the four taps at a fraction T of the way from the tap at 0 to the one at 1.
         procedure Weights (T : Real; W, S : out Taps) is
            U : constant Real := 1.0 - T;
         begin
            W := [U ** 3 / 6.0,
                  (4.0 - 6.0 * T * T + 3.0 * T ** 3) / 6.0,
                  (1.0 + 3.0 * T + 3.0 * T * T - 3.0 * T ** 3) / 6.0,
                  T ** 3 / 6.0];
            S := [-(U * U) / 2.0,
                  (3.0 * T * T - 4.0 * T) / 2.0,
                  (1.0 + 2.0 * T - 3.0 * T * T) / 2.0,
                  T * T / 2.0];
         end Weights;
      begin
         Weights (Fx, Wx, Sx);
         Weights (Fy, Wy, Sy);
         for Row in 0 .. 3 loop
            declare
               Base : constant Natural := Coefficients'First + (J + Row - 1) * Width + I - 1;
               Across, Slope_Across : Real := 0.0;
            begin
               for Column in 0 .. 3 loop
                  Across := Across + Wx (Column) * Coefficients (Base + Column);
                  Slope_Across := Slope_Across + Sx (Column) * Coefficients (Base + Column);
               end loop;
               Value := Value + Wy (Row) * Across;
               Dx := Dx + Wy (Row) * Slope_Across;
               Dy := Dy + Sy (Row) * Across;
            end;
         end loop;
         Ok := True;
      end;
   end Sample;

   function Level_Width (P : Pyramid; Level : Natural) return Natural is (P.Sizes (Level).Width);

   function Level_Height (P : Pyramid; Level : Natural) return Natural is (P.Sizes (Level).Height);

   procedure Sample_Level
     (P : Pyramid; Level : Natural; U, V : Real; Value, Du, Dv : out Real; Ok : out Boolean)
   is
      Size : Driver.Alignment.Level renames P.Sizes (Level);
   begin
      Sample (Size.Spline.Held.Values.all, Size.Width, Size.Height, U, V, Value, Du, Dv, Ok);
   end Sample_Level;

   ---------------------------------------------------------------------------
   --  How smooth a field of residuals or pixels is

   --  The variance inflation of a field on a square grid of the given side (row after row): neighbours that
   --  agree are one observation, not two, so a mean of N values of a field with lag-one autocorrelation r along
   --  each axis has the variance of one of N (1 - r) (1 - r) / ((1 + r) (1 + r)) independent values. Over the
   --  neighbours both present; the autocorrelation is never taken below zero (a field that alternates is no
   --  more informative than one that does not).
   function Inflation (Field : Real_Array; Present : Mask_Array; Side : Positive) return Real is
      Result : Real := 1.0;
   begin
      for Axis in 1 .. 2 loop
         declare
            Step : constant Positive := (if Axis = 1 then 1 else Side);
            Cross, Left, Right : Real := 0.0;
         begin
            for K in Field'Range loop
               if Present (K) and then K + Step <= Field'Last and then Present (K + Step)
                 and then (Axis = 2 or else (K - Field'First) mod Side < Side - 1)
               then
                  Cross := Cross + Field (K) * Field (K + Step);
                  Left := Left + Field (K) ** 2;
                  Right := Right + Field (K + Step) ** 2;
               end if;
            end loop;
            if Left > 0.0 and then Right > 0.0 then
               declare
                  R : constant Real :=
                    Real'Max (0.0, Real'Min (1.0 - 1.0 / Real (Field'Length), Cross / Sqrt (Left * Right)));
               begin
                  Result := Result * (1.0 + R) / (1.0 - R);
               end;
            end if;
         end;
      end loop;
      return Result;
   end Inflation;

   ---------------------------------------------------------------------------
   --  Small matrices

   function Inverse_Of (M : Linear_Part) return Linear_Part is
      Det : constant Real := M.UU * M.VV - M.UV * M.VU;
   begin
      return (UU => M.VV / Det, UV => -M.UV / Det, VU => -M.VU / Det, VV => M.UU / Det);
   end Inverse_Of;

   --  The singular values of a two by two matrix, the larger first.
   procedure Singular_Values (M : Linear_Part; Largest, Smallest : out Real) is
      Total : constant Real := M.UU ** 2 + M.UV ** 2 + M.VU ** 2 + M.VV ** 2;
      Det   : constant Real := M.UU * M.VV - M.UV * M.VU;
   begin
      Largest := Sqrt ((Total + Sqrt (Real'Max (0.0, Total ** 2 - 4.0 * Det ** 2))) / 2.0);
      Smallest := (if Largest > 0.0 then abs Det / Largest else 0.0);
   end Singular_Values;

   --  The eigenvalues of a covariance, the larger first.
   procedure Eigenvalues (C : Place_Covariance; Largest, Smallest : out Real) is
      Mean : constant Real := (C.UU + C.VV) / 2.0;
      Skew : constant Real := Sqrt (((C.UU - C.VV) / 2.0) ** 2 + C.UV ** 2);
   begin
      Largest := Mean + Skew;
      Smallest := Mean - Skew;
   end Eigenvalues;

   ---------------------------------------------------------------------------
   --  The fit of one level

   type Warp is record
      Qu, Qv : Real := 0.0;              --  the place in the second picture, at the level's scale
      Linear : Linear_Part := Identity_Part;
      Gain, Offset : Real := 0.0;
   end record;

   type Pass_Result (N : Natural) is record
      Rss      : Real := 0.0;
      Count    : Natural := 0;
      H        : Real_Matrix (1 .. Parameters, 1 .. Parameters) := [others => [others => 0.0]];
      G        : Real_Vector (1 .. Parameters) := [others => 0.0];
      Residual : Real_Array (1 .. N) := [others => 0.0];
      Seen     : Real_Array (1 .. N) := [others => 0.0];
      Present  : Mask_Array (1 .. N) := [others => False];
   end record;

   type Prior is record
      Centre_U, Centre_V : Real;   --  of the place, at the level's scale
      Place_UU, Place_UV, Place_VV : Real;
      --  the place's precision (the inverse of how widely it may lie), at the level's scale
      Linear             : Linear_Part;
      Sigma_Linear       : Real;
   end record;

   function Penalty (T : Warp; P : Prior; Noise : Real) return Real is
      --  Half the sum of squares the pixels leave and of the departures from what the prior holds, each over the
      --  noise of the pixels (the pixels' share is added by the caller).
     (Noise * (((T.Qu - P.Centre_U) ** 2 * P.Place_UU + 2.0 * (T.Qu - P.Centre_U) * (T.Qv - P.Centre_V) * P.Place_UV
                + (T.Qv - P.Centre_V) ** 2 * P.Place_VV)
               + ((T.Linear.UU - P.Linear.UU) ** 2 + (T.Linear.UV - P.Linear.UV) ** 2
                  + (T.Linear.VU - P.Linear.VU) ** 2 + (T.Linear.VV - P.Linear.VV) ** 2) / P.Sigma_Linear ** 2));

   --  Adds the prior's information to the pixels' normal equations.
   procedure Add_Prior (H : in out Real_Matrix; G : in out Real_Vector; T : Warp; P : Prior; Noise : Real) is
      Linear : constant Real := Noise / P.Sigma_Linear ** 2;
      Du     : constant Real := T.Qu - P.Centre_U;
      Dv     : constant Real := T.Qv - P.Centre_V;
   begin
      H (1, 1) := H (1, 1) + Noise * P.Place_UU;
      H (1, 2) := H (1, 2) + Noise * P.Place_UV;
      H (2, 1) := H (2, 1) + Noise * P.Place_UV;
      H (2, 2) := H (2, 2) + Noise * P.Place_VV;
      G (1) := G (1) + Noise * (P.Place_UU * Du + P.Place_UV * Dv);
      G (2) := G (2) + Noise * (P.Place_UV * Du + P.Place_VV * Dv);
      for K in 3 .. 6 loop
         H (K, K) := H (K, K) + Linear;
      end loop;
      G (3) := G (3) + Linear * (T.Linear.UU - P.Linear.UU);
      G (4) := G (4) + Linear * (T.Linear.UV - P.Linear.UV);
      G (5) := G (5) + Linear * (T.Linear.VU - P.Linear.VU);
      G (6) := G (6) + Linear * (T.Linear.VV - P.Linear.VV);
   end Add_Prior;

   --  Solves (H + prior) step = -(G + prior gradient) with the unknowns scaled by their own curvature; not Ok
   --  when the matrix is not positive definite.
   procedure Newton_Step
     (H : Real_Matrix; G : Real_Vector; Step : out Real_Vector; Ok : out Boolean)
   is
      Scale  : Real_Vector (1 .. Parameters);
      Scaled : Real_Matrix (1 .. Parameters, 1 .. Parameters);
      Chol   : Real_Matrix (1 .. Parameters, 1 .. Parameters);
      Rhs    : Real_Vector (1 .. Parameters);
   begin
      Step := [others => 0.0];
      for K in Scale'Range loop
         if H (K, K) <= 0.0 then
            Ok := False;
            return;
         end if;
         Scale (K) := 1.0 / Sqrt (H (K, K));
      end loop;
      for I in Scaled'Range (1) loop
         for J in Scaled'Range (2) loop
            Scaled (I, J) := Scale (I) * H (I, J) * Scale (J);
         end loop;
         Rhs (I) := -Scale (I) * G (I);
      end loop;
      Driver.Numerics.Dense.Cholesky (Scaled, Chol, Ok);
      if Ok then
         declare
            Y : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Chol, Rhs);
         begin
            for K in Step'Range loop
               Step (K) := Scale (K) * Y (K);
            end loop;
         end;
      end if;
   end Newton_Step;

   --  The inverse of the normal equations (the covariance of the unknowns over the noise), by the same scaling and
   --  factorization as the step, so that what can be stepped can be inverted.
   procedure Information_Inverse (H : Real_Matrix; Inv : out Real_Matrix; Ok : out Boolean) is
      Scale  : Real_Vector (1 .. Parameters);
      Scaled : Real_Matrix (1 .. Parameters, 1 .. Parameters);
      Chol   : Real_Matrix (1 .. Parameters, 1 .. Parameters);
   begin
      Inv := [others => [others => 0.0]];
      for K in Scale'Range loop
         if H (K, K) <= 0.0 then
            Ok := False;
            return;
         end if;
         Scale (K) := 1.0 / Sqrt (H (K, K));
      end loop;
      for I in Scaled'Range (1) loop
         for J in Scaled'Range (2) loop
            Scaled (I, J) := Scale (I) * H (I, J) * Scale (J);
         end loop;
      end loop;
      Driver.Numerics.Dense.Cholesky (Scaled, Chol, Ok);
      if Ok then
         for J in 1 .. Parameters loop
            declare
               Unit : Real_Vector (1 .. Parameters) := [others => 0.0];
            begin
               Unit (J) := 1.0;
               declare
                  Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Chol, Unit);
               begin
                  for I in 1 .. Parameters loop
                     Inv (I, J) := Scale (I) * Column (I) * Scale (J);
                  end loop;
               end;
            end;
         end loop;
      end if;
   end Information_Inverse;

   --  The derivatives of a pixel's residual by the warp's unknowns: the place (through the second picture's own
   --  slopes), the linear part (through the pixel's offset from the point), and the light's gain and offset.
   function Jacobian_Row (Di, Dj, Sx, Sy, Template : Real) return Real_Vector is
     ([Sx, Sy, Sx * Di, Sx * Dj, Sy * Di, Sy * Dj, -Template, -1.0]);

   --  One pass over the patch at a warp: the pixels' residuals, normal equations and sums. Template and Valid are
   --  the first picture's patch (resampled at the offsets Offsets_U, Offsets_V from the point, in the level's
   --  pixels), Second the level of the second picture.
   procedure Pass
     (T        : Warp;
      Template : Real_Array;
      Valid    : Mask_Array;
      Half     : Natural;
      Luma : Real_Array;
      Width, Height : Natural;
      Result   : out Pass_Result)
   is
      K : Natural := 0;
   begin
      Result.Rss := 0.0;
      Result.Count := 0;
      Result.H := [others => [others => 0.0]];
      Result.G := [others => 0.0];
      Result.Present := [others => False];
      Result.Residual := [others => 0.0];
      Result.Seen := [others => 0.0];
      for J in -Integer (Half) .. Integer (Half) loop
         for I in -Integer (Half) .. Integer (Half) loop
            K := K + 1;
            if Valid (K) then
               declare
                  Di : constant Real := Real (I);
                  Dj : constant Real := Real (J);
                  Yu : constant Real := T.Qu + T.Linear.UU * Di + T.Linear.UV * Dj;
                  Yv : constant Real := T.Qv + T.Linear.VU * Di + T.Linear.VV * Dj;
                  S, Sx, Sy : Real;
                  Ok : Boolean;
               begin
                  Sample (Luma, Width, Height, Yu, Yv, S, Sx, Sy, Ok);
                  if Ok then
                     declare
                        R   : constant Real := S - (T.Gain * Template (K) + T.Offset);
                        --  The row of the Jacobian at this pixel (Jacobian_Row), written out: a function returning
                        --  an array a pixel at a time was a third of a pass on the Pi.
                        Row : constant array (1 .. Parameters) of Real :=
                          [Sx, Sy, Sx * Di, Sx * Dj, Sy * Di, Sy * Dj, -Template (K), -1.0];
                     begin
                        for A in 1 .. Parameters loop
                           declare
                              Ra : constant Real := Row (A);
                           begin
                              for B in A .. Parameters loop
                                 Result.H (A, B) := Result.H (A, B) + Ra * Row (B);
                              end loop;
                              Result.G (A) := Result.G (A) + Ra * R;
                           end;
                        end loop;
                        Result.Rss := Result.Rss + R * R;
                        Result.Count := Result.Count + 1;
                        Result.Present (K) := True;
                        Result.Residual (K) := R;
                        Result.Seen (K) := S;
                     end;
                  end if;
               end;
            end if;
         end loop;
      end loop;
      for A in 1 .. Parameters loop
         for B in 1 .. A - 1 loop
            Result.H (A, B) := Result.H (B, A);
         end loop;
      end loop;
   end Pass;

   --  The information the first picture's own slopes carry through the fitted gain and warp (the slopes the second
   --  picture would have if it were the first, which none of its noise is in), over the pixels a pass found present.
   --  Taken once, where the fit ends: it reads no picture, and every pass took it before (R1 on the Pi, 10-13).
   procedure Own_Information
     (T        : Warp;
      Template : Real_Array;
      Slope_U  : Real_Array;
      Slope_V  : Real_Array;
      Present  : Mask_Array;
      Half     : Natural;
      Own_H    : out Real_Matrix)
   is
      Det : constant Real := T.Linear.UU * T.Linear.VV - T.Linear.UV * T.Linear.VU;
      K   : Natural := 0;
   begin
      Own_H := [others => [others => 0.0]];
      for J in -Integer (Half) .. Integer (Half) loop
         for I in -Integer (Half) .. Integer (Half) loop
            K := K + 1;
            if Present (K) then
               declare
                  Own : constant Real_Vector (1 .. Parameters) :=
                    Jacobian_Row
                      (Real (I), Real (J),
                       T.Gain * (T.Linear.VV * Slope_U (K) - T.Linear.VU * Slope_V (K)) / Det,
                       T.Gain * (T.Linear.UU * Slope_V (K) - T.Linear.UV * Slope_U (K)) / Det,
                       Template (K));
               begin
                  for A in 1 .. Parameters loop
                     for B in A .. Parameters loop
                        Own_H (A, B) := Own_H (A, B) + Own (A) * Own (B);
                     end loop;
                  end loop;
               end;
            end if;
         end loop;
      end loop;
      for A in 1 .. Parameters loop
         for B in 1 .. A - 1 loop
            Own_H (A, B) := Own_H (B, A);
         end loop;
      end loop;
   end Own_Information;

   ---------------------------------------------------------------------------
   --  The first picture's patch at a level

   --  Samples the first picture on the offsets Warp maps onto the second picture's pixel grid
   --  (Template (k) is the first picture at From + Linear^-1 offset (k)), or on its own grid when Linear is the
   --  identity.
   procedure Resample
     (Luma : Real_Array;
      Width, Height : Natural;
      From_U, From_V : Real;
      Inverse        : Linear_Part;
      Half           : Natural;
      Template       : out Real_Array;
      Slope_U, Slope_V : out Real_Array;
      Valid          : out Mask_Array)
   is
      K : Natural := 0;
   begin
      for J in -Integer (Half) .. Integer (Half) loop
         for I in -Integer (Half) .. Integer (Half) loop
            K := K + 1;
            declare
               Di : constant Real := Real (I);
               Dj : constant Real := Real (J);
               U  : constant Real := From_U + Inverse.UU * Di + Inverse.UV * Dj;
               V  : constant Real := From_V + Inverse.VU * Di + Inverse.VV * Dj;
               Value, Dx, Dy : Real;
               Ok : Boolean;
            begin
               Sample (Luma, Width, Height, U, V, Value, Dx, Dy, Ok);
               Template (K) := Value;
               Slope_U (K) := Dx;
               Slope_V (K) := Dy;
               Valid (K) := Ok;
            end;
         end loop;
      end loop;
   end Resample;

   ---------------------------------------------------------------------------
   --  The refinement of one level

   type Summary is record
      Fitted      : Warp;              --  at the full size
      Cov         : Place_Covariance;        --  of the place, at the full size, inflated by the residuals' smoothness
      Correlation : Real := 0.0;
      Rss         : Real := 0.0;       --  what the fit leaves unexplained, over the pixels it used
      Patch       : Real := 0.0;       --  the patch's own spread (its pixels' standard deviation)
      Noise       : Real := 0.0;       --  the variance of one pixel's residual
      Spread      : Real := 1.0;       --  how much smoother than white the residuals are (Inflation)
      Count       : Natural := 0;
      Converged   : Boolean := False;
      Informative : Boolean := False;  --  the normal equations could be solved
      Passes      : Natural := 0;      --  the passes over the patch the fit took (Work_Report)
      Iterations  : Natural := 0;      --  the Newton steps among them (the rest are the line search's trials)
   end record;

   --  The aligner's work, counted where it is done, in whichever task (Work_Report).
   type Work_Count is record
      Calls      : Natural := 0;
      Passes     : Natural := 0;    --  cells scanned, or passes over a patch
      Iterations : Natural := 0;    --  Newton steps, for the fits
      Seconds    : Duration := 0.0;
   end record;
   type Work_Counts is array (Work_Phase) of Work_Count;

   --  The retries with a wider patch, by what the first attempt failed for: how many, and how many then found.
   type Retry_Count is record
      Tried, Found : Natural := 0;
   end record;
   type Retry_Counts is array (Reason) of Retry_Count;

   --  Which of a window's peaks the place chosen came from (Align_At refines every peak of the scan): the scan's
   --  best, its second, or a later one; and how many winners lay further below the best peak's correlation than
   --  one pixel of misalignment costs the patch (Drop), the gap a refinement could close. For the measurement of
   --  whether refining every peak is needed (R6's seventh start on the Pi, 10-14: ten peak fits an asked point,
   --  424 of the aligner's 524 CPU seconds over 30 keyframes).
   type Winner_Count is record
      First, Second, Later, Beyond_Drop : Natural := 0;
      Largest_Over_Drop : Real := 0.0;   --  the largest gap, in drops, a winner closed
   end record;

   protected Work is
      procedure Add (Phase : Work_Phase; Passes : Natural; Seconds : Duration; Iterations : Natural := 0);
      procedure Add_Retry (First : Reason; Found : Boolean);
      procedure Add_Winner (Rank : Positive; Gap, Drop : Real);
      procedure Take (Into : out Work_Counts; Retries : out Retry_Counts; Winners : out Winner_Count);
   private
      Totals  : Work_Counts;
      Retried : Retry_Counts;
      Won     : Winner_Count;
   end Work;

   protected body Work is
      procedure Add (Phase : Work_Phase; Passes : Natural; Seconds : Duration; Iterations : Natural := 0) is
      begin
         Totals (Phase).Calls := Totals (Phase).Calls + 1;
         Totals (Phase).Passes := Totals (Phase).Passes + Passes;
         Totals (Phase).Iterations := Totals (Phase).Iterations + Iterations;
         Totals (Phase).Seconds := Totals (Phase).Seconds + Seconds;
      end Add;

      procedure Add_Retry (First : Reason; Found : Boolean) is
      begin
         Retried (First).Tried := Retried (First).Tried + 1;
         if Found then
            Retried (First).Found := Retried (First).Found + 1;
         end if;
      end Add_Retry;

      procedure Add_Winner (Rank : Positive; Gap, Drop : Real) is
      begin
         case Rank is
            when 1 => Won.First := Won.First + 1;
            when 2 => Won.Second := Won.Second + 1;
            when others => Won.Later := Won.Later + 1;
         end case;
         if Gap > Drop then
            Won.Beyond_Drop := Won.Beyond_Drop + 1;
         end if;
         if Drop > 0.0 then
            Won.Largest_Over_Drop := Real'Max (Won.Largest_Over_Drop, Gap / Drop);
         end if;
      end Add_Winner;

      procedure Take (Into : out Work_Counts; Retries : out Retry_Counts; Winners : out Winner_Count) is
      begin
         Into := Totals;
         Retries := Retried;
         Totals := [others => <>];
         Retried := [others => <>];
         Winners := Won;
         Won := (others => <>);
      end Take;
   end Work;

   function Work_Report return String is
      W : Work_Counts;
      R : Retry_Counts;
      V : Winner_Count;

      function Retries_Text (From : Reason) return String is
        (if From = Reason'Last then ""
         else (if R (From).Tried > 0
               then " " & Reason'Image (From) & R (From).Tried'Image & "/" & Natural'Image (R (From).Found)
               else "") & Retries_Text (Reason'Succ (From)));

      function Tenths (D : Duration) return String is
         T : constant Natural := Natural (D * 10);
         Whole : constant String := Natural'Image (T / 10);
         Tenth : constant String := Natural'Image (T mod 10);
      begin
         return Whole (Whole'First + 1 .. Whole'Last) & "." & Tenth (Tenth'First + 1 .. Tenth'Last);
      end Tenths;

      function Phase_Text (P : Work_Phase; Unit : String) return String is
        (Work_Phase'Image (P) & W (P).Calls'Image & " times," & W (P).Passes'Image & " " & Unit
         & (if W (P).Iterations > 0 then " in" & W (P).Iterations'Image & " steps" else "") & ", "
         & Tenths (W (P).Seconds) & " s");
   begin
      Work.Take (W, R, V);
      return Phase_Text (Scanning, "cells") & "; " & Phase_Text (Peak_Fitting, "passes") & "; "
        & Phase_Text (Level_Fitting, "passes") & "; " & Phase_Text (Variant_Fitting, "passes") & "; "
        & Phase_Text (Retrying, "found") & " (tried/found by the first failure:" & Retries_Text (Reason'First) & ")"
        & "; the place chosen was the scan's best peak" & V.First'Image & " times, its second" & V.Second'Image
        & ", a later one" & V.Later'Image & ";" & V.Beyond_Drop'Image
        & " lay below the best by more than a pixel costs the patch (the largest gap "
        & Tenths (Duration (V.Largest_Over_Drop)) & " of that)";
   end Work_Report;

   type Summary_Array is array (Positive range <>) of Summary;
   type Summary_Access is access Summary_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Summary_Array, Summary_Access);

   type Natural_Array is array (Positive range <>) of Natural;
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);

   --  What a search of a window keeps for every place of it. A window of a few hundred pixels across at a fine level
   --  has some hundreds of thousands of them, more than a task's stack holds (the stack of a task that a Parallel_For
   --  starts is a fraction of the main task's): these arrays live on the heap and go with the search, however it ends.
   type Candidates (Count : Natural) is new Ada.Finalization.Limited_Controlled with record
      Rho   : Reals_Access := new Real_Array'(1 .. Count => -2.0);   --  -2: not a candidate
      Peaks : Natural_Access := new Natural_Array'(1 .. Count => 0);
   end record;

   overriding procedure Finalize (Item : in out Candidates) is
   begin
      Free (Item.Rho);
      Free (Item.Peaks);
   end Finalize;

   --  And of the places that are peaks of their own: the fit of each, and how well it scores.
   type Peak_Fits (Count : Natural) is new Ada.Finalization.Limited_Controlled with record
      Fits  : Summary_Access := new Summary_Array (1 .. Count);
      Score : Reals_Access := new Real_Array'(1 .. Count => Real'First);
   end record;

   overriding procedure Finalize (Item : in out Peak_Fits) is
   begin
      Free (Item.Fits);
      Free (Item.Score);
   end Finalize;

   --  Gauss-Newton on the whole warp at the level of the pictures' halvings Step, from Start (given at the full
   --  size), directly on the pixels.
   procedure Fit_Level
     (First, Second : Pyramid;
      Step          : Natural;
      Query         : Prediction;
      Sigma_Linear  : Real;
      Ridge         : Place_Covariance;
      Start         : Warp;
      Info          : out Summary;
      Limit         : Natural := Natural'Last;
      Grow          : Positive := 1;
      Final         : Boolean := True)
   is
      Scale   : constant Real := Real (Halving) ** Step;
      --  How far a step may still move the place before the fit has settled, in the place's own standard errors:
      --  the unchanged fraction where the answer is the one used, one standard error where the next level's fit
      --  takes the answer as its start (the levels above the pixels, in the descent): a step the fit cannot tell
      --  from none is nothing to hand on. (The peaks' fits at the coarse level settle to the end: fitted to a
      --  standard error and settled again only where they could still win, R1's cost rose by a sixth, most
      --  peaks being contenders.)
      Tolerance : constant Real := (if Final then Driver.Conventions.Unchanged_Fraction else 1.0);
      A_Level : Level renames First.Sizes (Step);
      B_Level : Level renames Second.Sizes (Step);
      A_Spline : Real_Array renames A_Level.Spline.Held.Values.all;
      B_Spline : Real_Array renames B_Level.Spline.Held.Values.all;

      Half : constant Natural := Grow * Half_Patch (A_Level.Width, A_Level.Height);
      Side : constant Positive := Side_Of (Half);
      N    : constant Positive := Side * Side;

      Template : Real_Array (1 .. N);
      Valid    : Mask_Array (1 .. N);
      Slope_U  : Real_Array (1 .. N);
      Slope_V  : Real_Array (1 .. N);

      --  The place may lie as widely as the ridge says, the patch's own window: that holds a direction the pixels
      --  leave open where the prediction put it, and costs the directions they close nothing measurable.
      Ridge_Level : constant Place_Covariance :=
        (UU => Ridge.UU / Scale ** 2, UV => Ridge.UV / Scale ** 2, VV => Ridge.VV / Scale ** 2);
      Ridge_Det   : constant Real := Ridge_Level.UU * Ridge_Level.VV - Ridge_Level.UV ** 2;
      Held : constant Prior :=
        (Centre_U => Query.To.U / Scale, Centre_V => Query.To.V / Scale,
         Place_UU => Ridge_Level.VV / Ridge_Det, Place_UV => -Ridge_Level.UV / Ridge_Det,
         Place_VV => Ridge_Level.UU / Ridge_Det,
         Linear => Query.Linear, Sigma_Linear => Sigma_Linear);

      Current : Pass_Result (N);
      Trial   : Pass_Result (N);
      T       : Warp := (Qu => Start.Qu / Scale, Qv => Start.Qv / Scale, Linear => Start.Linear,
                         Gain => Start.Gain, Offset => Start.Offset);
      Noise   : Real;
      Cost    : Real;
      Last_Moved : Real := 0.0;   --  the length of the step before, in standard errors; none yet

      function Noise_Of (P : Pass_Result) return Real is
        (Real'Max (Light_Noise_Floor, P.Rss / Real (P.Count - Parameters)));

      procedure Run (W : Warp; Into : out Pass_Result) is
      begin
         Pass (W, Template, Valid, Half, B_Spline, B_Level.Width, B_Level.Height, Into);
         Info.Passes := Info.Passes + 1;
      end Run;
   begin
      Info := (Fitted => Start, others => <>);
      Resample (A_Spline, A_Level.Width, A_Level.Height, Query.From.U / Scale, Query.From.V / Scale,
                Identity_Part, Half, Template, Slope_U, Slope_V, Valid);
      --  Only the pixels within Limit of the point, when the patch is to be used in part.
      if Limit < Half then
         declare
            K : Natural := 0;
         begin
            for J in -Integer (Half) .. Integer (Half) loop
               for I in -Integer (Half) .. Integer (Half) loop
                  K := K + 1;
                  if abs I > Limit or else abs J > Limit then
                     Valid (K) := False;
                  end if;
               end loop;
            end loop;
         end;
      end if;
      Run (T, Current);
      if Current.Count <= Parameters then
         Info.Count := Current.Count;
         return;
      end if;
      Noise := Noise_Of (Current);
      Cost := 0.5 * (Current.Rss + Penalty (T, Held, Noise));
      Info.Informative := True;
      for Iteration in 1 .. Real'Machine_Mantissa loop
         Info.Iterations := Info.Iterations + 1;
         declare
            H : Real_Matrix (1 .. Parameters, 1 .. Parameters) := Current.H;
            G : Real_Vector (1 .. Parameters) := Current.G;
            Increment : Real_Vector (1 .. Parameters);
            Inv : Real_Matrix (1 .. Parameters, 1 .. Parameters);
            Ok : Boolean;
         begin
            Add_Prior (H, G, T, Held, Noise);
            Newton_Step (H, G, Increment, Ok);
            if not Ok then
               Info.Informative := False;
               exit;
            end if;
            Information_Inverse (H, Inv, Ok);
            if not Ok then
               Info.Informative := False;
               exit;
            end if;
            declare
               --  The place's own information (what the whole warp leaves it, the other unknowns being unknown too),
               --  and the length of the step in its own standard errors.
               Q_UU : constant Real := Noise * Inv (1, 1);
               Q_UV : constant Real := Noise * Inv (1, 2);
               Q_VV : constant Real := Noise * Inv (2, 2);
               Q_Det : constant Real := Q_UU * Q_VV - Q_UV ** 2;
               Fraction : Real := 1.0;
               Accepted : Boolean := False;
               Trial_Cost : Real := Cost;
               Candidate : Warp := T;

               function Length_Of (Fraction : Real) return Real is
                  Du : constant Real := Fraction * Increment (1);
                  Dv : constant Real := Fraction * Increment (2);
               begin
                  if Q_Det <= 0.0 then
                     return Real'Last;
                  end if;
                  return Sqrt (Real'Max (0.0, (Q_VV * Du ** 2 - 2.0 * Q_UV * Du * Dv + Q_UU * Dv ** 2) / Q_Det));
               end Length_Of;
            begin
               loop
                  Candidate := (Qu => T.Qu + Fraction * Increment (1), Qv => T.Qv + Fraction * Increment (2),
                                Linear =>
                                  (UU => T.Linear.UU + Fraction * Increment (3),
                                   UV => T.Linear.UV + Fraction * Increment (4),
                                   VU => T.Linear.VU + Fraction * Increment (5),
                                   VV => T.Linear.VV + Fraction * Increment (6)),
                                Gain => T.Gain + Fraction * Increment (7),
                                Offset => T.Offset + Fraction * Increment (8));
                  Run (Candidate, Trial);
                  if Trial.Count > Parameters then
                     Trial_Cost := 0.5 * (Trial.Rss + Penalty (Candidate, Held, Noise));
                     if Trial_Cost <= Cost then
                        Accepted := True;
                        exit;
                     end if;
                  end if;
                  Fraction := Fraction / 2.0;
                  exit when Length_Of (Fraction) <= Tolerance;
               end loop;
               if not Accepted then
                  Info.Converged := True;
                  exit;
               end if;
               declare
                  Moved : constant Real := Length_Of (Fraction);
               begin
                  T := Candidate;
                  Current := Trial;
                  Noise := Noise_Of (Current);
                  Cost := 0.5 * (Current.Rss + Penalty (T, Held, Noise));
                  --  Settled when a step is a hundredth of the place's standard error, or when steps shrink by a ratio
                  --  and what the ones to come would add up to (a geometric series: the last step squared over how
                  --  much it fell short of the one before) is less than a standard error over Z, which no test the
                  --  driver makes can tell from none.
                  if Moved <= Tolerance
                    or else (Final and then Last_Moved > Moved and then Moved ** 2 / (Last_Moved - Moved) <= 1.0 / Z)
                  then
                     Info.Converged := True;
                     exit;
                  end if;
                  Last_Moved := Moved;
               end;
            end;
         end;
      end loop;
      --  What the fit leaves.
      Info.Count := Current.Count;
      Info.Fitted := (Qu => T.Qu * Scale, Qv => T.Qv * Scale, Linear => T.Linear, Gain => T.Gain, Offset => T.Offset);
      if not Info.Informative or else Current.Count <= Parameters then
         Info.Informative := False;
         return;
      end if;
      declare
         S2  : constant Real := Noise_Of (Current);
         H   : Real_Matrix (1 .. Parameters, 1 .. Parameters) := Current.H;
         G   : Real_Vector (1 .. Parameters) := Current.G;
      begin
         Add_Prior (H, G, T, Held, S2);
         declare
            Inv : Real_Matrix (1 .. Parameters, 1 .. Parameters);
            Ok  : Boolean;
         begin
            Information_Inverse (H, Inv, Ok);
            if not Ok then
               Info.Informative := False;
               return;
            end if;
            Info.Rss := Current.Rss;
            Info.Noise := S2;
            Info.Spread := Inflation (Current.Residual, Current.Present, Side);
            --  What the pixels' noise alone allows, over the whole patch.
            Info.Cov := (UU => S2 * Inv (1, 1) * Info.Spread * Scale ** 2,
                         UV => S2 * Inv (1, 2) * Info.Spread * Scale ** 2,
                         VV => S2 * Inv (2, 2) * Info.Spread * Scale ** 2);
            --  What the patch's own parts say of it: the scores of the pixels (their gradient times what they leave)
            --  summed over a tiling of the patch, whose parts that disagree about where the place is (a contour
            --  that moves with the surface in front of it, a shading that moved) add up to far more than noise does
            --  (the cluster-robust, or sandwich, covariance: the inverse information on either side of the sum of
            --  the squares of the parts' scores).
            declare
               Own_H : Real_Matrix (1 .. Parameters, 1 .. Parameters);
               Own_G : Real_Vector (1 .. Parameters) := Current.G;
               Own_Inv : Real_Matrix (1 .. Parameters, 1 .. Parameters);
               Own_Ok  : Boolean;
            begin
               Own_Information (T, Template, Slope_U, Slope_V, Current.Present, Half, Own_H);
               Add_Prior (Own_H, Own_G, T, Held, S2);
               Information_Inverse (Own_H, Own_Inv, Own_Ok);
               if not Own_Ok then
                  Info.Informative := False;
                  return;
               end if;
               declare
                  Theirs : constant Place_Covariance :=
                    (UU => S2 * Own_Inv (1, 1) * Info.Spread * Scale ** 2,
                     UV => S2 * Own_Inv (1, 2) * Info.Spread * Scale ** 2,
                     VV => S2 * Own_Inv (2, 2) * Info.Spread * Scale ** 2);
               begin
                  if Theirs.UU + Theirs.VV > Info.Cov.UU + Info.Cov.VV then
                     Info.Cov := Theirs;
                  end if;
               end;
            end;
            declare
               Per_Axis : constant Positive := Positive (Real'Ceiling (Sqrt (2.0 * Real (Parameters))));
               Scores   : Real_Matrix (1 .. Per_Axis * Per_Axis, 1 .. Parameters) := [others => [others => 0.0]];
               Middle   : Real_Matrix (1 .. Parameters, 1 .. Parameters) := [others => [others => 0.0]];
               K        : Natural := 0;
            begin
               for J in -Integer (Half) .. Integer (Half) loop
                  for I in -Integer (Half) .. Integer (Half) loop
                     K := K + 1;
                     if Current.Present (K) then
                        declare
                           Di : constant Real := Real (I);
                           Dj : constant Real := Real (J);
                           Yu : constant Real := T.Qu + T.Linear.UU * Di + T.Linear.UV * Dj;
                           Yv : constant Real := T.Qv + T.Linear.VU * Di + T.Linear.VV * Dj;
                           S, Sx, Sy : Real;
                           Ok_Here : Boolean;
                        begin
                           Sample (B_Spline, B_Level.Width, B_Level.Height, Yu, Yv, S, Sx, Sy, Ok_Here);
                           if Ok_Here then
                              declare
                                 Cluster : constant Positive :=
                                   ((J + Integer (Half)) * Per_Axis / Side) * Per_Axis
                                   + (I + Integer (Half)) * Per_Axis / Side + 1;
                                 R   : constant Real := Current.Residual (K);
                                 Row : constant Real_Vector (1 .. Parameters) :=
                                   Jacobian_Row (Di, Dj, Sx, Sy, Template (K));
                              begin
                                 for A in 1 .. Parameters loop
                                    Scores (Cluster, A) := Scores (Cluster, A) + Row (A) * R;
                                 end loop;
                              end;
                           end if;
                        end;
                     end if;
                  end loop;
               end loop;
               for C in Scores'Range (1) loop
                  for A in 1 .. Parameters loop
                     for B in 1 .. Parameters loop
                        Middle (A, B) := Middle (A, B) + Scores (C, A) * Scores (C, B);
                     end loop;
                  end loop;
               end loop;
               declare
                  Sandwich : constant Real_Matrix := Inv * Middle * Inv;
                  Own_Trace : constant Real := Info.Cov.UU + Info.Cov.VV;
                  Theirs    : constant Place_Covariance :=
                    (UU => Sandwich (1, 1) * Scale ** 2, UV => Sandwich (1, 2) * Scale ** 2,
                     VV => Sandwich (2, 2) * Scale ** 2);
               begin
                  if Theirs.UU + Theirs.VV > Own_Trace then
                     Info.Cov := Theirs;
                  end if;
               end;
            end;
         end;
         --  The correlation of the patch with the second picture's at the fit.
         declare
            Mean_T, Mean_S : Real := 0.0;
            Stt, Sss, Sst  : Real := 0.0;
            Count : constant Real := Real (Current.Count);
         begin
            for K in 1 .. N loop
               if Current.Present (K) then
                  Mean_T := Mean_T + Template (K);
                  Mean_S := Mean_S + Current.Seen (K);
               end if;
            end loop;
            Mean_T := Mean_T / Count;
            Mean_S := Mean_S / Count;
            for K in 1 .. N loop
               if Current.Present (K) then
                  Stt := Stt + (Template (K) - Mean_T) ** 2;
                  Sss := Sss + (Current.Seen (K) - Mean_S) ** 2;
                  Sst := Sst + (Template (K) - Mean_T) * (Current.Seen (K) - Mean_S);
               end if;
            end loop;
            if Stt > 0.0 and then Sss > 0.0 then
               Info.Correlation := Sst / Sqrt (Stt * Sss);
               Info.Patch := Sqrt (Stt / Count);
            end if;
         end;
      end;
   end Fit_Level;

   ---------------------------------------------------------------------------
   --  The alignment

   function Fail (Why : Reason; Verdict : Alignment.Verdict := Not_Found) return Answer is
     ((Verdict => Verdict, Because => Why, To => (U => 0.0, V => 0.0), Cov => (others => 0.0),
       Linear => Identity_Part, Correlation => 0.0, Fit_Rms => 0.0, Patch_Rms => 0.0, Degrees_Of_Freedom => 0,
       Narrow => False));

   --  The squared length of a displacement in the standard deviations of a covariance (its Mahalanobis length).
   function Squared_Length (Du, Dv : Real; C : Place_Covariance) return Real is
     ((C.VV * Du ** 2 - 2.0 * C.UV * Du * Dv + C.UU * Dv ** 2) / (C.UU * C.VV - C.UV ** 2));

   Odds_Needed : constant Real :=
     Log ((1.0 - Driver.Distributions.Gaussian_Two_Sided_Tail (Z)) / Driver.Distributions.Gaussian_Two_Sided_Tail (Z));
   --  How much likelier (in logarithm) the best place must be than the best other place for it to be called the
   --  place at the significance Z stands for: the odds at which the chance of the other being the place is that tail.

   --  How many independent observations a patch holds, for its correlation with a picture that has nothing to do
   --  with it: Bartlett's count, the pixels over the sum of the squares of the patch's own autocorrelation over all
   --  lags (a patch whose pixels agree with their neighbours far off holds fewer observations than it has pixels,
   --  and a texture of one wavelength holds a few), never fewer than one nor more than its pixels.
   function Independent_Samples (Template : Real_Array; Valid : Mask_Array; Side : Positive) return Real is
      Count : Natural := 0;
      Sum   : Real := 0.0;
      Half  : constant Natural := (Side - 1) / 2;
   begin
      for K in Template'Range loop
         if Valid (K) then
            Count := Count + 1;
            Sum := Sum + Template (K);
         end if;
      end loop;
      if Count < 2 then
         return 1.0;
      end if;
      declare
         Mean : constant Real := Sum / Real (Count);
         Spread : Real := 0.0;
         Squares : Real := 1.0;   --  the lag at nothing, whose autocorrelation is one
      begin
         for K in Template'Range loop
            if Valid (K) then
               Spread := Spread + (Template (K) - Mean) ** 2;
            end if;
         end loop;
         Spread := Spread / Real (Count);
         if Spread <= 0.0 then
            return 1.0;
         end if;
         --  Every lag once on one side of nothing (the other side's is the same), doubled.
         for Dy in 0 .. Half loop
            for Dx in (if Dy = 0 then 1 else -Half) .. Half loop
               declare
                  Pairs : Natural := 0;
                  Total : Real := 0.0;
               begin
                  for Row in 0 .. Side - 1 - Dy loop
                     for Column in Integer'Max (0, -Dx) .. Integer'Min (Side - 1, Side - 1 - Dx) loop
                        declare
                           A : constant Positive := Template'First + Row * Side + Column;
                           B : constant Positive := A + Dy * Side + Dx;
                        begin
                           if Valid (A) and then Valid (B) then
                              Pairs := Pairs + 1;
                              Total := Total + (Template (A) - Mean) * (Template (B) - Mean);
                           end if;
                        end;
                     end loop;
                  end loop;
                  if Pairs > 0 then
                     Squares := Squares + 2.0 * (Total / Real (Pairs) / Spread) ** 2;
                  end if;
               end;
            end loop;
         end loop;
         return Real'Min (Real (Count), Real'Max (1.0, Real (Count) / Squares));
      end;
   end Independent_Samples;

   --  Whether an answer beats chance: where the prediction leaves several independent places of the patch open
   --  (its one-standard-deviation ellipse holds as many), a correlation as high as the best of that many unrelated
   --  places reaches, at the significance of the one gate shared among them, is no evidence of a match. The
   --  correlation of a patch of so many independent observations with an unrelated picture follows Student's t
   --  with that many less two degrees of freedom. Where the prediction leaves one place open its own belief is the
   --  evidence the pixels need not add to.
   function Beats_Chance
     (First : Pyramid; Query : Prediction; Plain : Place_Covariance; Linear : Linear_Part; Correlation : Real)
      return Boolean
   is
      Top   : Level renames First.Sizes (0);
      Half  : constant Natural := Half_Patch (Top.Width, Top.Height);
      Side  : constant Positive := Side_Of (Half);
      N     : constant Positive := Side * Side;
      Template : Real_Array (1 .. N);
      Slope_U  : Real_Array (1 .. N);
      Slope_V  : Real_Array (1 .. N);
      Valid    : Mask_Array (1 .. N);
      Present  : Natural := 0;
   begin
      Resample (Top.Spline.Held.Values.all, Top.Width, Top.Height, Query.From.U, Query.From.V, Inverse_Of (Linear),
                Half, Template, Slope_U, Slope_V, Valid);
      for K in 1 .. N loop
         if Valid (K) then
            Present := Present + 1;
         end if;
      end loop;
      if Present < 2 then
         return True;
      end if;
      declare
         Independent : constant Real := Independent_Samples (Template, Valid, Side);
         Area   : constant Real :=
           Ada.Numerics.Pi * Sqrt (Real'Max (0.0, Plain.UU * Plain.VV - Plain.UV ** 2));
         Places : constant Real := Area * Independent / Real (Present);
      begin
         if Places <= 1.0 then
            return True;
         end if;
         declare
            Freedom : constant Positive := Positive'Max (1, Natural (Real'Floor (Independent)) - 2);
            T       : constant Real :=
              Driver.Distributions.Student_T_Quantile
                (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z) / Places, Freedom);
         begin
            return Correlation >= T / Sqrt (Real (Freedom) + T ** 2);
         end;
      end;
   end Beats_Chance;

   --  The search of the window at the level Coarse, and the refinement down to the full size.
   function Align_At
     (First, Second : Pyramid;
      Query         : Prediction;
      Coarse        : Natural;
      Plain         : Place_Covariance;
      Radius        : Real;
      Grow          : Positive;
      Must_Beat     : Boolean := False;
      Correlation_To_Beat : Real := 0.0) return Answer
   is
      Scale   : constant Real := Real (Halving) ** Coarse;
      A_Level : Level renames First.Sizes (Coarse);
      B_Level : Level renames Second.Sizes (Coarse);
      A_Spline : Real_Array renames A_Level.Spline.Held.Values.all;
      B_Luma : Real_Array renames B_Level.Luma.Held.Values.all;

      Half : constant Natural := Grow * Half_Patch (A_Level.Width, A_Level.Height);
      Side : constant Positive := Side_Of (Half);
      N    : constant Positive := Side * Side;
      Top_Half : constant Natural := Half_Patch (First.Sizes (0).Width, First.Sizes (0).Height);   --  the patch the fit ends on

      Template : Real_Array (1 .. N);
      Valid    : Mask_Array (1 .. N);
      Present  : Natural := 0;
      Slope_U  : Real_Array (1 .. N);
      Slope_V  : Real_Array (1 .. N);

      --  The window, at this level.
      Centre_U : constant Real := Query.To.U / Scale;
      Centre_V : constant Real := Query.To.V / Scale;
      Window : constant Place_Covariance :=
        (UU => Query.Cov.UU / Scale ** 2 + Quantization, UV => Query.Cov.UV / Scale ** 2,
         VV => Query.Cov.VV / Scale ** 2 + Quantization);
      Extent_U : constant Real := Window_Multiple * Sqrt (Window.UU);
      Extent_V : constant Real := Window_Multiple * Sqrt (Window.VV);
      Low_X  : constant Integer := Integer (Real'Floor (Centre_U - Extent_U - 0.5));
      High_X : constant Integer := Integer (Real'Ceiling (Centre_U + Extent_U - 0.5));
      Low_Y  : constant Integer := Integer (Real'Floor (Centre_V - Extent_V - 0.5));
      High_Y : constant Integer := Integer (Real'Ceiling (Centre_V + Extent_V - 0.5));

      --  The candidates are the pixels of the window: cell (A, B) of the grid is the place (Low_X + 1/2 + A,
      --  Low_Y + 1/2 + B), the centre of a pixel of this level. The fit refines whichever are worth it to the places
      --  between them.
      Across : constant Positive := High_X - Low_X + 1;
      Down   : constant Positive := High_Y - Low_Y + 1;

      Window_Candidates : Candidates (Across * Down);
      Rho    : Real_Array renames Window_Candidates.Rho.all;
      Tested : Natural := 0;
      Flat   : Natural := 0;

      Peaks      : Natural_Array renames Window_Candidates.Peaks.all;
      Peak_Count : Natural := 0;
      Scan_From  : Duration := 0.0;   --  when the scan of the window began (Work_Report)

      --  The place of a cell, at this level's scale, and the pixel it is the centre of.
      function Cell_X (Index : Positive) return Integer is (Low_X + (Index - 1) mod Across);
      function Cell_Y (Index : Positive) return Integer is (Low_Y + (Index - 1) / Across);
      function Cell_U (Index : Positive) return Real is (Real (Cell_X (Index)) + 0.5);
      function Cell_V (Index : Positive) return Real is (Real (Cell_Y (Index)) + 0.5);

      function In_Picture (X, Y : Integer) return Boolean is
        (X >= 0 and then Y >= 0 and then X < B_Level.Width and then Y < B_Level.Height);
      function Pixel_Of (X, Y : Integer) return Real is (B_Luma (B_Luma'First + Y * B_Level.Width + X));
   begin
      Resample (A_Spline, A_Level.Width, A_Level.Height, Query.From.U / Scale, Query.From.V / Scale,
                Inverse_Of (Query.Linear), Half, Template, Slope_U, Slope_V, Valid);
      for K in 1 .. N loop
         if Valid (K) then
            Present := Present + 1;
         end if;
      end loop;
      if 2 * Present < N then
         return Fail (Edge_Of_Picture);
      end if;

      --  Every candidate in the window: the correlation of the patch with the second picture's pixels there.
      Scan_From := Driver.Clock.Seconds;
      for B in 0 .. Down - 1 loop
         for A in 0 .. Across - 1 loop
            declare
               Index : constant Positive := B * Across + A + 1;
               Dx : constant Real := Cell_U (Index) - Centre_U;
               Dy : constant Real := Cell_V (Index) - Centre_V;
            begin
               if Squared_Length (Dx, Dy, Window) <= Window_Multiple ** 2 then
                  Tested := Tested + 1;
                  declare
                     Cx : constant Integer := Cell_X (Index);
                     Cy : constant Integer := Cell_Y (Index);
                     Sum_S, Sum_T, Sum_SS, Sum_TT, Sum_ST : Real := 0.0;
                     Count : Natural := 0;
                     K : Natural := 0;
                  begin
                     for J in -Integer (Half) .. Integer (Half) loop
                        for I in -Integer (Half) .. Integer (Half) loop
                           K := K + 1;
                           if Valid (K) and then In_Picture (Cx + I, Cy + J) then
                              declare
                                 S : constant Real := Pixel_Of (Cx + I, Cy + J);
                                 T : constant Real := Template (K);
                              begin
                                 Count := Count + 1;
                                 Sum_S := Sum_S + S;
                                 Sum_T := Sum_T + T;
                                 Sum_SS := Sum_SS + S * S;
                                 Sum_TT := Sum_TT + T * T;
                                 Sum_ST := Sum_ST + S * T;
                              end;
                           end if;
                        end loop;
                     end loop;
                     if 2 * Count >= N then
                        declare
                           C   : constant Real := Real (Count);
                           Vss : constant Real := Sum_SS - Sum_S ** 2 / C;
                           Vtt : constant Real := Sum_TT - Sum_T ** 2 / C;
                           Vst : constant Real := Sum_ST - Sum_S * Sum_T / C;
                        begin
                           if Vss > 0.0 and then Vtt > 0.0 then
                              Rho (Index) := Vst / Sqrt (Vss * Vtt);
                           else
                              Flat := Flat + 1;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end;
         end loop;
      end loop;

      Work.Add (Scanning, Tested, Driver.Clock.Seconds - Scan_From);

      --  The places that are peaks of their own, the best first: a place is a peak when no candidate within a
      --  pixel of it is better (a tie goes to the one met first).
      declare
         Count : Natural := 0;
      begin
         for K in Rho'Range loop
            if Rho (K) > -2.0 then
               declare
                  Col : constant Natural := (K - 1) mod Across;
                  Row : constant Natural := (K - 1) / Across;
                  Peak : Boolean := True;
               begin
                  for Dr in -1 .. 1 loop
                     for Dc in -1 .. 1 loop
                        if (Dr /= 0 or else Dc /= 0)
                          and then Col + Dc >= 0 and then Col + Dc < Across
                          and then Row + Dr >= 0 and then Row + Dr < Down
                        then
                           declare
                              Other : constant Positive := Row * Across + Col + 1 + Dr * Across + Dc;
                           begin
                              if Rho (Other) > Rho (K) or else (Rho (Other) = Rho (K) and then Other < K) then
                                 Peak := False;
                              end if;
                           end;
                        end if;
                     end loop;
                  end loop;
                  if Peak then
                     Count := Count + 1;
                     Peaks (Count) := K;
                  end if;
               end;
            end if;
         end loop;
         if Count = 0 then
            return (if Flat > 0 then Fail (No_Texture) else Fail (Outside_Picture, Not_In_View));
         end if;
         Peak_Count := Count;
         --  The best first (insertion).
         for K in 2 .. Count loop
            declare
               Held : constant Positive := Peaks (K);
               J    : Natural := K - 1;
            begin
               while J >= 1 and then Rho (Peaks (J)) < Rho (Held) loop
                  Peaks (J + 1) := Peaks (J);
                  J := J - 1;
               end loop;
               Peaks (J + 1) := Held;
            end;
         end loop;
      end;

      declare
         --  The scan's best correlation: what a refusal from here on reports, for a retry to beat.
         Best_Rho : constant Real := Rho (Peaks (1));

         function Refuse (Why : Reason) return Answer is (Fail (Why) with delta Correlation => Best_Rho);
      begin
         --  A retry with a wider patch is fitted only where its scan shows more agreement at its best than the
         --  narrower patch did: the wide patch's fits cost four times the narrow one's, and R1's retries found a
         --  place for one in sixteen (2506 tried, 161 found), the scan telling most of the rest apart for nothing.
         if Must_Beat and then Best_Rho <= Correlation_To_Beat then
            return Refuse (No_Evidence);
         end if;
      end;

      declare
         Best_Rho : constant Real := Rho (Peaks (1));

         function Refuse (Why : Reason; Verdict : Alignment.Verdict := Not_Found) return Answer is
           (Fail (Why, Verdict) with delta Correlation => Best_Rho);

         Tries  : constant Positive := Peak_Count + 1;   --  every peak, and the place the prediction itself says
         Peak_Scratch : Peak_Fits (Peak_Count + 1);
         Fits   : Summary_Array renames Peak_Scratch.Fits.all;
         Nearest : constant Positive :=
           Integer'Min (Down - 1, Integer'Max (0, Integer (Real'Rounding (Centre_V - (Real (Low_Y) + 0.5))))) * Across
           + Integer'Min (Across - 1, Integer'Max (0, Integer (Real'Rounding (Centre_U - (Real (Low_X) + 0.5))))) + 1;
         Fitted : Summary;
         Start  : Warp;
         Linear_Sigma : constant Real :=
           (if Query.Linear_Sigma < Real'Last then Query.Linear_Sigma else Radius / Real (Top_Half));
         --  What the pixels may do to the place without being asked to: nothing beyond the window, its own
         --  size (the warp's linear part, if its error is not known, may move the patch's rim as far as the window).
         Ridge : constant Place_Covariance :=
           (UU => Window_Multiple ** 2 * Plain.UU, UV => Window_Multiple ** 2 * Plain.UV,
            VV => Window_Multiple ** 2 * Plain.VV);

         --  The warp the candidate at a cell starts the refinement from: the predicted linear part, the place
         --  of the cell, and the gain and offset that bring the patch to the pixels there.
         function Start_At (Index : Positive) return Warp is
            Cx : constant Integer := Cell_X (Index);
            Cy : constant Integer := Cell_Y (Index);
            Mean_T, Mean_S, Stt, Sst : Real := 0.0;
            Count : Natural := 0;
            K : Natural := 0;
         begin
            for J in -Integer (Half) .. Integer (Half) loop
               for I in -Integer (Half) .. Integer (Half) loop
                  K := K + 1;
                  if Valid (K) and then In_Picture (Cx + I, Cy + J) then
                     Count := Count + 1;
                     Mean_T := Mean_T + Template (K);
                     Mean_S := Mean_S + Pixel_Of (Cx + I, Cy + J);
                  end if;
               end loop;
            end loop;
            Mean_T := Mean_T / Real (Count);
            Mean_S := Mean_S / Real (Count);
            K := 0;
            for J in -Integer (Half) .. Integer (Half) loop
               for I in -Integer (Half) .. Integer (Half) loop
                  K := K + 1;
                  if Valid (K) and then In_Picture (Cx + I, Cy + J) then
                     Stt := Stt + (Template (K) - Mean_T) ** 2;
                     Sst := Sst + (Template (K) - Mean_T) * (Pixel_Of (Cx + I, Cy + J) - Mean_S);
                  end if;
               end loop;
            end loop;
            return (Qu => Cell_U (Index) * Scale, Qv => Cell_V (Index) * Scale, Linear => Query.Linear,
                    Gain => Sst / Stt, Offset => Mean_S - Sst / Stt * Mean_T);
         end Start_At;

         Best, Runner_Up, Reference : Natural := 0;
      begin
         --  Every peak, refined where it was found, so that none is penalized for the pixel the search put it on.
         for K in 1 .. Tries loop
            if K <= Peak_Count then
               declare
                  T0 : constant Duration := Driver.Clock.Seconds;
               begin
                  Fit_Level (First, Second, Coarse, Query, Linear_Sigma, Ridge, Start_At (Peaks (K)), Fits (K),
                             Grow => Grow);
                  Work.Add (Peak_Fitting, Fits (K).Passes, Driver.Clock.Seconds - T0, Fits (K).Iterations);
               end;
            elsif Rho (Nearest) > -2.0 then
               declare
                  Here : Warp := Start_At (Nearest);
                  T0   : constant Duration := Driver.Clock.Seconds;
               begin
                  Here.Qu := Query.To.U;
                  Here.Qv := Query.To.V;
                  Fit_Level (First, Second, Coarse, Query, Linear_Sigma, Ridge, Here, Fits (K), Grow => Grow);
                  Work.Add (Peak_Fitting, Fits (K).Passes, Driver.Clock.Seconds - T0, Fits (K).Iterations);
               end;
            end if;
            if Fits (K).Informative
              and then (Reference = 0
                        or else Fits (K).Rss / Real (Fits (K).Count)
                                < Fits (Reference).Rss / Real (Fits (Reference).Count))
            then
               Reference := K;
            end if;
         end loop;
         if Reference = 0 then
            return Refuse (Not_Solvable);
         end if;

         --  Which one is the place: the one that leaves least unexplained, in the noise the best of them leaves,
         --  with neighbouring residuals that agree counted once, and that lies nearest the prediction in its own
         --  standard deviations (what is known beforehand weighs the candidates, never the answer's spread: the
         --  covariance below is the pixels' alone).
         --
         declare
            Noise  : constant Real := Fits (Reference).Noise;
            Spread : constant Real := Fits (Reference).Spread;
            Pixels : constant Real := Real (Fits (Reference).Count);
            Score  : Real_Array renames Peak_Scratch.Score.all;
         begin
            for K in 1 .. Tries loop
               if Fits (K).Informative then
                  Score (K) := -0.5 * (Pixels * (Fits (K).Rss / Real (Fits (K).Count)) / (Noise * Spread)
                                       + Squared_Length (Fits (K).Fitted.Qu - Query.To.U,
                                                         Fits (K).Fitted.Qv - Query.To.V, Plain));
                  if Best = 0 or else Score (K) > Score (Best) then
                     Best := K;
                  end if;
               end if;
            end loop;
            --  The best of the places that are not the same place (apart by more than their covariances allow).
            for K in 1 .. Tries loop
               if K /= Best and then Fits (K).Informative
                 and then Squared_Length
                   (Fits (K).Fitted.Qu - Fits (Best).Fitted.Qu, Fits (K).Fitted.Qv - Fits (Best).Fitted.Qv,
                    (UU => Fits (K).Cov.UU + Fits (Best).Cov.UU, UV => Fits (K).Cov.UV + Fits (Best).Cov.UV,
                     VV => Fits (K).Cov.VV + Fits (Best).Cov.VV)) > Window_Multiple ** 2
                 and then (Runner_Up = 0 or else Score (K) > Score (Runner_Up))
               then
                  Runner_Up := K;
               end if;
            end loop;
            if Runner_Up > 0 and then Score (Best) - Score (Runner_Up) < Odds_Needed then
               return Refuse (Ambiguous);
            end if;
         end;
         --  For the measurement (Work.Add_Winner): which peak won, how far its scan correlation lay below the
         --  best peak's, and what one pixel of misalignment costs this patch's correlation: to second order,
         --  the slope's energy along the steeper axis over twice the patch's variance.
         declare
            Mean_T, Var_T, Energy_U, Energy_V : Real := 0.0;
            Used : Natural := 0;
         begin
            for K in 1 .. N loop
               if Valid (K) then
                  Used := Used + 1;
                  Mean_T := Mean_T + Template (K);
               end if;
            end loop;
            if Used > 0 then
               Mean_T := Mean_T / Real (Used);
               for K in 1 .. N loop
                  if Valid (K) then
                     Var_T := Var_T + (Template (K) - Mean_T) ** 2;
                     Energy_U := Energy_U + Slope_U (K) ** 2;
                     Energy_V := Energy_V + Slope_V (K) ** 2;
                  end if;
               end loop;
               if Var_T > 0.0 then
                  Work.Add_Winner
                    (Rank => (if Best <= Peak_Count then Best else Tries),
                     Gap  => Rho (Peaks (1)) - (if Best <= Peak_Count then Rho (Peaks (Best)) else Rho (Nearest)),
                     Drop => Real'Max (Energy_U, Energy_V) / (2.0 * Var_T));
               end if;
            end if;
         end;
         Fitted := Fits (Best);

         --  Down the levels to the pixels: each level's fit starts the next, and only the one at the pixels settles
         --  to the unchanged fraction (the best peak was settled at the coarse level already).
         for Step in reverse Finest .. Coarse - 1 loop
            Start := Fitted.Fitted;
            declare
               T0 : constant Duration := Driver.Clock.Seconds;
            begin
               Fit_Level (First, Second, Step, Query, Linear_Sigma, Ridge, Start, Fitted,
                          Grow => Grow, Final => Step = Finest);
               Work.Add (Level_Fitting, Fitted.Passes, Driver.Clock.Seconds - T0, Fitted.Iterations);
            end;
            if not Fitted.Informative then
               return Refuse (Not_Solvable);
            end if;
         end loop;
         if not Fitted.Converged then
            return Refuse (Not_Settled);
         end if;

         --  The verdict on the answer at the pixels.
         declare
            Answer_U : constant Real := Fitted.Fitted.Qu;
            Answer_V : constant Real := Fitted.Fitted.Qv;
            Largest, Smallest, Ridge_Largest, Ridge_Smallest : Real;
            Slack : constant Real := Scale ** 2 * Quantization;
            Wide : constant Place_Covariance :=
              (UU => Plain.UU + Slack, UV => Plain.UV, VV => Plain.VV + Slack);
            Dx : constant Real := Answer_U - Query.To.U;
            Dy : constant Real := Answer_V - Query.To.V;
            Cov : Place_Covariance := Fitted.Cov;
            Variants : Natural := 0;
         begin
            if Answer_U < 0.0 or else Answer_V < 0.0 or else Answer_U > Real (Second.Sizes (0).Width)
              or else Answer_V > Real (Second.Sizes (0).Height)
            then
               return Refuse (Outside_Picture, Not_In_View);
            end if;
            if 2 * Fitted.Count < Side_Of (Grow * Top_Half) ** 2 then
               return Refuse (Edge_Of_Picture);
            end if;
            if Squared_Length (Dx, Dy, Wide) > Window_Multiple ** 2 then
               return Refuse (Left_Window);
            end if;
            if Fitted.Correlation <= 0.0 then
               return Refuse (No_Evidence);
            end if;

            --  How the answer moves with the choices the fit made. The noise of one fit does not say how far a place
            --  is from where the patch pins it when the patch is not what the model takes it for (a contour that moves
            --  with another surface than the texture beside it, a shading that moved, a texture finer than the
            --  pictures hold): the answer then depends on how much of the patch is used and at what scale. So the
            --  patch is fitted again from the answer on its middle (half its side, the factor of a halving) and on
            --  each of the next two coarser levels of the pictures, and what those places do around the answer's
            --  (their sample covariance, with as many degrees of freedom as there are of them) is added to what the
            --  noise allows. Where nothing depends on the choice the variants agree and add nothing.
            declare
               Last_Half  : constant Natural :=
                 Grow * Half_Patch (First.Sizes (Finest).Width, First.Sizes (Finest).Height);
               Deepest    : constant Natural := Natural'Min (Levels (First), Levels (Second)) - 1;
               Shift_U, Shift_V : Real_Array (1 .. Most_Variants);
               Other      : Summary;

               function Sum (Shift : Real_Array) return Real is
                  Result : Real := 0.0;
               begin
                  for K in 1 .. Variants loop
                     Result := Result + Shift (K);
                  end loop;
                  return Result;
               end Sum;

               procedure Take (Level_Wanted : Natural; Limit : Natural) is
                  T0 : constant Duration := Driver.Clock.Seconds;
               begin
                  Fit_Level (First, Second, Level_Wanted, Query, Linear_Sigma, Ridge, Fitted.Fitted, Other,
                             Limit => Limit, Grow => Grow);
                  Work.Add (Variant_Fitting, Other.Passes, Driver.Clock.Seconds - T0, Other.Iterations);
                  if Other.Informative then
                     Variants := Variants + 1;
                     Shift_U (Variants) := Other.Fitted.Qu - Answer_U;
                     Shift_V (Variants) := Other.Fitted.Qv - Answer_V;
                  end if;
               end Take;
            begin
               Take (Finest, Last_Half / Halving);
               for Step in 1 .. Most_Variants - 1 loop
                  if Finest + Step <= Deepest then
                     Take (Finest + Step, Natural'Last);
                  end if;
               end loop;
               if Variants > 0 then
                  declare
                     --  The answer is the first estimate, at no shift; the sample covariance about the mean of all.
                     Mean_U : constant Real := Sum (Shift_U) / Real (Variants + 1);
                     Mean_V : constant Real := Sum (Shift_V) / Real (Variants + 1);
                     Uu : Real := Mean_U ** 2;
                     Uv : Real := Mean_U * Mean_V;
                     Vv : Real := Mean_V ** 2;
                  begin
                     for K in 1 .. Variants loop
                        Uu := Uu + (Shift_U (K) - Mean_U) ** 2;
                        Uv := Uv + (Shift_U (K) - Mean_U) * (Shift_V (K) - Mean_V);
                        Vv := Vv + (Shift_V (K) - Mean_V) ** 2;
                     end loop;
                     Cov := (UU => Cov.UU + Uu / Real (Variants), UV => Cov.UV + Uv / Real (Variants),
                             VV => Cov.VV + Vv / Real (Variants));
                  end;
               end if;
            end;

            Eigenvalues (Cov, Largest, Smallest);
            Eigenvalues (Ridge, Ridge_Largest, Ridge_Smallest);
            if Smallest <= 0.0 then
               return Refuse (Not_Solvable);
            end if;
            --  The pixels must tell the place along some direction better than the window does: the data's own
            --  precision there at least that of the window (then the covariance is no more than half the window's),
            --  or the answer is the prediction's own belief given back.
            if Smallest > Ridge_Smallest / 2.0 then
               return Refuse (Uninformative);
            end if;
            return (Verdict => Found, Because => Matched, To => (U => Answer_U, V => Answer_V), Cov => Cov,
                    Linear => Fitted.Fitted.Linear, Correlation => Fitted.Correlation,
                    Fit_Rms => Sqrt (Fitted.Rss / Real (Fitted.Count)), Patch_Rms => Fitted.Patch,
                    Degrees_Of_Freedom => Variants, Narrow => False);
         end;
      end;
   end Align_At;

   function Window_Level (First, Second : Pyramid; Radius : Real) return Natural is
      Deepest : constant Natural := Natural'Min (Levels (First), Levels (Second)) - 1;
      Chosen  : Natural := 0;
      Least   : Real := Real'Last;
   begin
      for Step in 0 .. Deepest loop
         declare
            H    : constant Natural := Half_Patch (Second.Sizes (Step).Width, Second.Sizes (Step).Height);
            Need : constant Real := Radius / Real (Halving) ** Step + 1.0;
         begin
            if Need <= Real (H) then
               return Step;
            elsif Need / Real (H) < Least then
               Least := Need / Real (H);
               Chosen := Step;
            end if;
         end;
      end loop;
      return Chosen;
   end Window_Level;

   function Largest_Window (First, Second : Pyramid) return Real is
     (Real'Min (Real (Second.Sizes (0).Width), Real (Second.Sizes (0).Height)) / 2.0);

   function Window_Radius (Sigma : Real) return Real is (Window_Multiple * Sqrt (Sigma * Sigma + Quantization));

   function Sigma_Of_Window (Radius : Real) return Real is
     (Sqrt (Real'Max (0.0, (Radius / Window_Multiple) ** 2 - Quantization)));

   function Finest_Radius (Width, Height : Natural) return Real is (Real (Half_Patch (Width, Height)) - 1.0);

   function Largest_Sigma (First, Second : Pyramid) return Real is
      Widest : constant Real := Largest_Window (First, Second);
      Sigma  : Real := Sqrt (Real'Max (0.0, (Widest / Window_Multiple) ** 2 - Quantization));
   begin
      --  Align takes the radius of a round prediction as Window_Multiple * Sqrt (Sigma ** 2 + Quantization): step
      --  down until that is no more than the widest, whatever the rounding of the inversion above.
      while Sigma > 0.0 and then Window_Multiple * Sqrt (Sigma * Sigma + Quantization) > Widest loop
         Sigma := Real'Pred (Sigma);
      end loop;
      return Sigma;
   end Largest_Sigma;

   --  Align for pictures at about the same scale: the warp's scale within one halving.
   function Align_Matched (First, Second : Pyramid; Query : Prediction; Widen : Boolean) return Answer is
      Second_Top : Level renames Second.Sizes (0);
      --  The prediction's covariance, and what a place on a grid of pixels cannot be known to beyond.
      Plain      : constant Place_Covariance :=
        (UU => Query.Cov.UU + Quantization, UV => Query.Cov.UV, VV => Query.Cov.VV + Quantization);
      Radius_U : constant Real := Window_Multiple * Sqrt (Plain.UU);
      Radius_V : constant Real := Window_Multiple * Sqrt (Plain.VV);
      Radius   : constant Real := Real'Max (Radius_U, Radius_V);
      Largest_Scale, Smallest_Scale : Real;
      Det : constant Real := Query.Linear.UU * Query.Linear.VV - Query.Linear.UV * Query.Linear.VU;
   begin
      --  The warp must be one the levels compare: no mirror, and no direction made smaller by more than a halving
      --  (the pictures' pixels would be skipped over there; made larger, they are read between).
      Singular_Values (Query.Linear, Largest_Scale, Smallest_Scale);
      if Det <= 0.0 or else Smallest_Scale < 1.0 / Real (Halving) then
         return Fail (Bad_Warp);
      end if;
      --  Beyond the picture altogether.
      if Query.To.U + Radius_U < 0.0 or else Query.To.U - Radius_U > Real (Second_Top.Width)
        or else Query.To.V + Radius_V < 0.0 or else Query.To.V - Radius_V > Real (Second_Top.Height)
      then
         return Fail (Outside_Picture, Not_In_View);
      end if;
      --  The level the window is searched at (Window_Level), so long as the window is narrower than the picture
      --  (Largest_Window).
      declare
         Chosen : constant Natural := Window_Level (First, Second, Radius);
         Narrow : constant Boolean :=
           Radius / Real (Halving) ** Chosen + 1.0
             <= Real (Half_Patch (Second.Sizes (Chosen).Width, Second.Sizes (Chosen).Height));
      begin
         if not Narrow and then Radius > Largest_Window (First, Second) then
            return Fail (Window_Too_Large);
         end if;
         declare
            Level_Used : constant Natural := Chosen;

            --  An answer stands only if it beats chance among the places the prediction leaves open.
            function Checked (Found : Answer) return Answer is
              (if Found.Verdict = Driver.Alignment.Found
                    and then not Beats_Chance (First, Query, Plain, Found.Linear, Found.Correlation)
               then (Fail (No_Evidence) with delta Correlation => Found.Correlation) else Found);

            Answered   : constant Answer := Checked (Align_At (First, Second, Query, Level_Used, Plain, Radius, 1));

            --  Whether the window was one a patch carries at the finest size (and was searched there).
            Fine_Window : constant Boolean := Narrow and then Level_Used = Finest;

            function Marked (Found : Answer) return Answer is (Found with delta Narrow => Fine_Window);
         begin
            --  What a patch of the usual size cannot decide for lack of what lies around the point (it is
            --  ambiguous, tells the place no better than the window does, or is carried off by what the
            --  patch holds) may be decided by one twice as wide, the factor of a level of the pictures.
            if Widen and then Answered.Verdict = Not_Found
              and then Answered.Because in Ambiguous | Uninformative | No_Evidence | Left_Window | Not_Settled
            then
               declare
                  Wider : constant Answer :=
                    Checked (Align_At (First, Second, Query, Level_Used, Plain, Radius, 2,
                                       Must_Beat => True, Correlation_To_Beat => Answered.Correlation));
               begin
                  Work.Add (Retrying, (if Wider.Verdict = Found then 1 else 0), 0.0);
                  Work.Add_Retry (Answered.Because, Wider.Verdict = Found);
                  if Wider.Verdict = Found then
                     return Marked (Wider);
                  end if;
               end;
            end if;
            return Marked (Answered);
         end;
      end;
   end Align_Matched;

   --  The pictures from the level Down on: the halvings of the same picture, the first of them the full size now.
   function Shifted (P : Pyramid; Down : Natural) return Pyramid is
      Result : Pyramid;
   begin
      for K in Down .. P.Sizes.Last_Index loop
         Result.Sizes.Append (P.Sizes (K));
      end loop;
      return Result;
   end Shifted;

   function Scaled (L : Linear_Part; By : Real) return Linear_Part is
     ((UU => L.UU * By, UV => L.UV * By, VU => L.VU * By, VV => L.VV * By));

   function Align (First, Second : Pyramid; Query : Prediction; Widen : Boolean := True) return Answer is
      Largest, Smallest : Real;
   begin
      Singular_Values (Query.Linear, Largest, Smallest);
      --  A warp that changes the scale by more than a halving compares the pictures at levels that differ: the
      --  one that shows the point larger is halved until the two show it at about one size (its coordinates, the
      --  warp and the prediction's covariance with it), and the answer is carried back.
      if Query.Linear.UU * Query.Linear.VV - Query.Linear.UV * Query.Linear.VU <= 0.0 or else Smallest <= 0.0 then
         return Align_Matched (First, Second, Query, Widen);
      end if;
      declare
         --  The first picture is halved until no direction of it is made smaller by more than a halving; where the
         --  warp only enlarges, the second is halved until its smallest enlargement is about one.
         Shrinks : constant Boolean := Smallest < 1.0 / Real (Halving);
         Down    : constant Natural :=
           (if Shrinks then Natural (Real'Rounding (Log (1.0 / Smallest, Real (Halving))))
            elsif Smallest > 1.0 then Natural (Real'Rounding (Log (Smallest, Real (Halving))))
            else 0);
         Size    : constant Real := Real (Halving) ** Down;
      begin
         if Down = 0 or else (Shrinks and then Down >= Levels (First))
           or else (not Shrinks and then Down >= Levels (Second))
         then
            return Align_Matched (First, Second, Query, Widen);
         elsif Shrinks then
            declare
               Smaller : constant Pyramid := Shifted (First, Down);
               Result  : Answer :=
                 Align_Matched
                   (Smaller, Second,
                    (From => (U => Query.From.U / Size, V => Query.From.V / Size), To => Query.To,
                     Linear => Scaled (Query.Linear, Size), Cov => Query.Cov,
                     Linear_Sigma => (if Query.Linear_Sigma < Real'Last then Query.Linear_Sigma * Size
                                      else Real'Last)),
                    Widen);
            begin
               if Result.Verdict = Found then
                  Result.Linear := Scaled (Result.Linear, 1.0 / Size);
               end if;
               return Result;
            end;
         else
            declare
               Smaller : constant Pyramid := Shifted (Second, Down);
               Result  : Answer :=
                 Align_Matched
                   (First, Smaller,
                    (From => Query.From, To => (U => Query.To.U / Size, V => Query.To.V / Size),
                     Linear => Scaled (Query.Linear, 1.0 / Size),
                     Cov => (UU => Query.Cov.UU / Size ** 2, UV => Query.Cov.UV / Size ** 2,
                             VV => Query.Cov.VV / Size ** 2),
                     Linear_Sigma => (if Query.Linear_Sigma < Real'Last then Query.Linear_Sigma / Size
                                      else Real'Last)),
                    Widen);
            begin
               if Result.Verdict = Found then
                  Result.To := (U => Result.To.U * Size, V => Result.To.V * Size);
                  Result.Cov := (UU => Result.Cov.UU * Size ** 2, UV => Result.Cov.UV * Size ** 2,
                                 VV => Result.Cov.VV * Size ** 2);
                  Result.Linear := Scaled (Result.Linear, Size);
               end if;
               return Result;
            end;
         end if;
      end;
   end Align;

end Driver.Alignment;
