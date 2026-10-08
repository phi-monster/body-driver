with Ada.Containers.Generic_Array_Sort;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;

package body Driver.Robot.Hand.Slide is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;

   --  A frame's worth of luma lives on the heap, never on a task's stack.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   --  What 8-bit luma cannot tell: a level is known to a uniform step of one,
   --  whose variance is 1/12 (Driver.Pixels uses the same floor).
   Quantization : constant Real := 1.0 / 12.0;

   --  How far either side of a point of an edge its step is read, in pixels: a pair of places two pixels apart
   --  are on the two sides of an edge that is blurred over a pixel or two, and the reading between pixels, which
   --  spreads a step over one, does not take it from them.
   Across_Distance : constant Real := 1.0;

   function Empty return Patch is (others => <>);

   --  The luma of a picture at a place in it, read between its pixels by their distances; a place out of it is
   --  not read.
   function Read
     (Luma : Real_Array; Width, Height : Positive; X, Y : Real; Seen : out Boolean) return Real
   is
      X0 : Natural;
      Y0 : Natural;
      X1 : Natural;
      Y1 : Natural;
      Fx : Real;
      Fy : Real;
   begin
      Seen := X >= 0.0 and then X <= Real (Width - 1) and then Y >= 0.0 and then Y <= Real (Height - 1);
      if not Seen then
         return 0.0;
      end if;
      X0 := Natural (Real'Floor (X));
      Y0 := Natural (Real'Floor (Y));
      X1 := Natural'Min (X0 + 1, Width - 1);
      Y1 := Natural'Min (Y0 + 1, Height - 1);
      Fx := X - Real (X0);
      Fy := Y - Real (Y0);
      return (1.0 - Fy) * ((1.0 - Fx) * Luma (Luma'First + Y0 * Width + X0) + Fx * Luma (Luma'First + Y0 * Width + X1))
        + Fy * ((1.0 - Fx) * Luma (Luma'First + Y1 * Width + X0) + Fx * Luma (Luma'First + Y1 * Width + X1));
   end Read;

   --  The score of the points (Cols, Rows) with the ways across (Us, Vs) moved By along (U, V).
   function Score_Of
     (Cols, Rows  : Natural_Array;
      Us, Vs      : Real_Array;
      Dark_Inside : Boolean;
      Luma        : Real_Array;
      Width       : Positive;
      Height      : Positive;
      U, V, By    : Real) return Real
   is
      Total  : Real := 0.0;   --  the weights of all the points
      Seen   : Real := 0.0;   --  and of those whose pair of places to read are both in the picture
      Summed : Real := 0.0;
   begin
      for I in Cols'Range loop
         declare
            Weight : constant Real := abs (Us (I) * U + Vs (I) * V);
            X      : constant Real := Real (Cols (I)) + By * U;
            Y      : constant Real := Real (Rows (I)) + By * V;
            In_Out, In_In : Boolean;
            Outer  : constant Real := Read (Luma, Width, Height, X + Across_Distance * Us (I),
                                            Y + Across_Distance * Vs (I), In_Out);
            Inner  : constant Real := Read (Luma, Width, Height, X - Across_Distance * Us (I),
                                            Y - Across_Distance * Vs (I), In_In);
         begin
            Total := Total + Weight;
            if In_Out and then In_In then
               Seen := Seen + Weight;
               Summed := Summed + Weight * (if Dark_Inside then Outer - Inner else Inner - Outer);
            end if;
         end;
      end loop;
      return (if Seen > 0.0 and then 2.0 * Seen >= Total then Summed / Seen else Real'First);
   end Score_Of;

   --  The greatest of a curve of scores at the shifts -K .. K (Curve (K + 1 + S) is that at shift S), the
   --  shift nearest none among equals, placed between its neighbours by the parabola through the three.
   type Peak_Found is record
      Proper : Boolean := False;   --  it is not at an end of the curve, and the curve is a peak there
      By     : Real := 0.0;
      Score  : Real := Real'First;
   end record;

   function Peak_Of (Curve : Real_Array; K : Natural) return Peak_Found is
      Result : Peak_Found;
      Best   : Integer := 0;
   begin
      for D in 0 .. K loop
         for Sign in 0 .. 1 loop
            declare
               S : constant Integer := (if Sign = 0 then Integer (D) else -Integer (D));
            begin
               if (D > 0 or else Sign = 0) and then Curve (K + 1 + S) > Result.Score then
                  Result.Score := Curve (K + 1 + S);
                  Best := S;
               end if;
            end;
         end loop;
      end loop;
      Result.By := Real (Best);
      if Result.Score > Real'First and then abs Best < Integer (K) then
         declare
            Before : constant Real := Curve (K + Best);
            After  : constant Real := Curve (K + 2 + Best);
            Bend   : constant Real := Before - 2.0 * Result.Score + After;
         begin
            if Before > Real'First and then After > Real'First and then Bend < 0.0 then
               Result.Proper := True;
               Result.By := Real (Best) + Real'Max (-0.5, Real'Min (0.5, (Before - After) / (2.0 * Bend)));
            end if;
         end;
      end if;
      return Result;
   end Peak_Of;

   function Curve_Of
     (Cols, Rows  : Natural_Array;
      Us, Vs      : Real_Array;
      Dark_Inside : Boolean;
      Luma        : Real_Array;
      Width       : Positive;
      Height      : Positive;
      U, V        : Real;
      K           : Natural) return Real_Array
   is
      Curve : Real_Array (1 .. 2 * K + 1);
   begin
      for S in -Integer (K) .. Integer (K) loop
         Curve (K + 1 + S) := Score_Of (Cols, Rows, Us, Vs, Dark_Inside, Luma, Width, Height, U, V, Real (S));
      end loop;
      return Curve;
   end Curve_Of;

   function Before (A, B : Real) return Boolean is (A < B);
   procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Real, Real_Array, Before);

   --  The middle of the scores that are scores, and the spread about it, from the median of the distances from it
   --  (the greatest score is a few shifts among all of them and does not move either).
   procedure Spread_Of (Curve : Real_Array; Middle, Spread : out Real) is
      Count : Natural := 0;
   begin
      Middle := 0.0;
      Spread := 0.0;
      for C of Curve loop
         if C > Real'First then
            Count := Count + 1;
         end if;
      end loop;
      if Count = 0 then
         return;
      end if;
      declare
         Scores : Real_Array (1 .. Count);
         Next   : Natural := 0;
      begin
         for C of Curve loop
            if C > Real'First then
               Next := Next + 1;
               Scores (Next) := C;
            end if;
         end loop;
         Sort (Scores);
         Middle := Scores ((Count + 1) / 2);
         for I in Scores'Range loop
            Scores (I) := abs (Scores (I) - Middle);
         end loop;
         Sort (Scores);
         --  The scale of a Gaussian from the median of the distances from the middle.
         Spread := Scores ((Count + 1) / 2) / 0.674_489_75;
      end;
   end Spread_Of;

   --  The peak of a curve that is nearest to Rest of those that stand out of the other scores. Each run of
   --  consecutive shifts whose score is more than the gate's threshold (one test of as many as there are shifts)
   --  of the curve's spread above its middle has its greatest, placed by the parabola through it and its two
   --  neighbours, and the one nearest Rest is the finger's: it has slid as little as the picture lets it have.
   function Chosen_Peak
     (Curve : Real_Array; K : Natural; Rest : Real; Middle, Spread : out Real) return Peak_Found
   is
      Level    : Real;
      Chosen   : Peak_Found;
      Run_Best : Integer := 0;
      In_Run   : Boolean := False;

      procedure Close_Run is
      begin
         if In_Run then
            In_Run := False;
            if abs Run_Best < Integer (K) then
               declare
                  Here   : constant Real := Curve (K + 1 + Run_Best);
                  Before : constant Real := Curve (K + Run_Best);
                  After  : constant Real := Curve (K + 2 + Run_Best);
                  Bend   : constant Real := Before - 2.0 * Here + After;
               begin
                  if Before > Real'First and then After > Real'First and then Bend < 0.0 then
                     declare
                        By : constant Real :=
                          Real (Run_Best) + Real'Max (-0.5, Real'Min (0.5, (Before - After) / (2.0 * Bend)));
                     begin
                        if not Chosen.Proper or else abs (By - Rest) < abs (Chosen.By - Rest) then
                           Chosen := (Proper => True, By => By, Score => Here);
                        end if;
                     end;
                  end if;
               end;
            end if;
         end if;
      end Close_Run;
   begin
      Spread_Of (Curve, Middle, Spread);
      Level := Middle + Threshold (Scalar_Gate (Tests => 2 * K + 1)) * Real'Max (Spread, Sqrt (Quantization));
      for S in -Integer (K) .. Integer (K) loop
         declare
            C : constant Real := Curve (K + 1 + S);
         begin
            if C > Level then
               if not In_Run then
                  In_Run := True;
                  Run_Best := S;
               elsif C > Curve (K + 1 + Run_Best) then
                  Run_Best := S;
               end if;
            else
               Close_Run;
            end if;
         end;
      end loop;
      Close_Run;
      return Chosen;
   end Chosen_Peak;

   --  The middle of the points across the way of the shift: the line through the patch's middle along it that parts
   --  its two halves.
   function Cut_Of (Cols, Rows : Natural_Array; U, V : Real) return Real is
      Sorted : Real_Array (Cols'Range);
   begin
      for I in Cols'Range loop
         Sorted (I) := U * Real (Rows (I)) - V * Real (Cols (I));
      end loop;
      Sort (Sorted);
      return Sorted ((Sorted'First + Sorted'Last) / 2);
   end Cut_Of;

   --  The greatest score of one half of the points, the lower or the upper across the way of the shift.
   function Half_Peak
     (Cols, Rows  : Natural_Array;
      Us, Vs      : Real_Array;
      Dark_Inside : Boolean;
      Luma        : Real_Array;
      Width       : Positive;
      Height      : Positive;
      U, V        : Real;
      K           : Natural;
      Cut         : Real;
      Rest        : Real;
      Lower       : Boolean) return Peak_Found
   is
      Count : Natural := 0;
   begin
      for I in Cols'Range loop
         if (U * Real (Rows (I)) - V * Real (Cols (I)) <= Cut) = Lower then
            Count := Count + 1;
         end if;
      end loop;
      declare
         Part_Cols, Part_Rows : Natural_Array (1 .. Count);
         Part_Us, Part_Vs     : Real_Array (1 .. Count);
         Next                 : Natural := 0;
      begin
         for I in Cols'Range loop
            if (U * Real (Rows (I)) - V * Real (Cols (I)) <= Cut) = Lower then
               Next := Next + 1;
               Part_Cols (Next) := Cols (I);
               Part_Rows (Next) := Rows (I);
               Part_Us (Next) := Us (I);
               Part_Vs (Next) := Vs (I);
            end if;
         end loop;
         declare
            Middle, Spread : Real;
         begin
            return Chosen_Peak (Curve_Of (Part_Cols, Part_Rows, Part_Us, Part_Vs, Dark_Inside, Luma, Width, Height, U, V, K),
                                K, Rest, Middle, Spread);
         end;
      end;
   end Half_Peak;

   package Natural_Vectors is new Ada.Containers.Vectors (Positive, Natural);
   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);

   function Take (Lobe, Region : Mask; Luma : Real_Array; U, V, Reach : Real) return Patch is
      W      : constant Natural := Width (Lobe);
      H      : constant Natural := Height (Lobe);
      K      : constant Natural := Natural (Real'Ceiling (Real'Max (Reach, 2.0)));
      Cols   : Natural_Vectors.Vector;
      Rows   : Natural_Vectors.Vector;
      Across_U, Across_V : Real_Vectors.Vector;
      Window : constant := 3;   --  how near a pixel not the lobe's makes a pixel of the lobe a point of its edge
   begin
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            if Contains (Lobe, C, R) and then Contains (Region, C, R) then
               declare
                  Sx, Sy : Real := 0.0;
                  Beyond : Natural := 0;
               begin
                  for Dy in -Window .. Window loop
                     for Dx in -Window .. Window loop
                        declare
                           X : constant Integer := C + Dx;
                           Y : constant Integer := R + Dy;
                        begin
                           if (Dx /= 0 or else Dy /= 0) and then X in 0 .. W - 1 and then Y in 0 .. H - 1
                             and then not Contains (Lobe, X, Y)
                           then
                              Sx := Sx + Real (Dx);
                              Sy := Sy + Real (Dy);
                              Beyond := Beyond + 1;
                           end if;
                        end;
                     end loop;
                  end loop;
                  if Beyond > 0 and then (Sx /= 0.0 or else Sy /= 0.0) then
                     declare
                        Length : constant Real := Sqrt (Sx * Sx + Sy * Sy);
                     begin
                        Cols.Append (C);
                        Rows.Append (R);
                        Across_U.Append (Sx / Length);
                        Across_V.Append (Sy / Length);
                     end;
                  end if;
               end;
            end if;
         end loop;
      end loop;
      declare
         N    : constant Natural := Natural (Cols.Length);
         Step : Real := 0.0;   --  the free picture's step of luma across the edge, summed over the points
      begin
         if N = 0 then
            return Empty;
         end if;
         declare
            C_Array : Natural_Array (1 .. N);
            R_Array : Natural_Array (1 .. N);
            U_Array : Real_Array (1 .. N);
            V_Array : Real_Array (1 .. N);
            Dark    : Boolean;
            Rest    : Real := 0.0;
            Rest_Low, Rest_High : Real := 0.0;
         begin
            for I in 1 .. N loop
               C_Array (I) := Cols (I);
               R_Array (I) := Rows (I);
               U_Array (I) := Across_U (I);
               V_Array (I) := Across_V (I);
               declare
                  Seen_Out, Seen_In : Boolean;
                  Outer : constant Real :=
                    Read (Luma, W, H, Real (Cols (I)) + Across_Distance * Across_U (I),
                          Real (Rows (I)) + Across_Distance * Across_V (I), Seen_Out);
                  Inner : constant Real :=
                    Read (Luma, W, H, Real (Cols (I)) - Across_Distance * Across_U (I),
                          Real (Rows (I)) - Across_Distance * Across_V (I), Seen_In);
               begin
                  if Seen_Out and then Seen_In then
                     Step := Step + (Outer - Inner);
                  end if;
               end;
            end loop;
            Dark := Step >= 0.0;
            declare
               Middle, Spread : Real;
               Found : constant Peak_Found :=
                 Chosen_Peak (Curve_Of (C_Array, R_Array, U_Array, V_Array, Dark, Luma, W, H, U, V, K), K, 0.0,
                              Middle, Spread);
               Cut   : constant Real := Cut_Of (C_Array, R_Array, U, V);
               Low   : constant Peak_Found :=
                 Half_Peak (C_Array, R_Array, U_Array, V_Array, Dark, Luma, W, H, U, V, K, Cut, 0.0, True);
               High  : constant Peak_Found :=
                 Half_Peak (C_Array, R_Array, U_Array, V_Array, Dark, Luma, W, H, U, V, K, Cut, 0.0, False);
            begin
               if Found.Proper then
                  Rest := Found.By;
               end if;
               Rest_Low := (if Low.Proper then Low.By else 0.0);
               Rest_High := (if High.Proper then High.By else 0.0);
            end;
            return (Column      => Index_Holders.To_Holder (C_Array),
                    Row         => Index_Holders.To_Holder (R_Array),
                    Across_U    => Real_Holders.To_Holder (U_Array),
                    Across_V    => Real_Holders.To_Holder (V_Array),
                    Dark_Inside => Dark,
                    U           => U,
                    V           => V,
                    Reach       => Reach,
                    Rest        => Rest,
                    Rest_Low    => Rest_Low,
                    Rest_High   => Rest_High);
         end;
      end;
   end Take;

   function Take (Lobe, Region : Mask; View : Driver.Pixels.View; U, V, Reach : Real) return Patch is
      Means : Real_Access := new Real_Array (1 .. Driver.Pixels.Width (View) * Driver.Pixels.Height (View));
   begin
      Driver.Pixels.Means (View, Means.all);
      declare
         Made : constant Patch := Take (Lobe, Region, Means.all, U, V, Reach);
      begin
         Free (Means);
         return Made;
      end;
   end Take;

   function Points (P : Patch) return Natural is (if P.Column.Is_Empty then 0 else P.Column.Element'Length);

   function See (Luma : Real_Array; Width, Height : Positive) return Picture is
     (Width => Width, Height => Height, Luma => Real_Holders.To_Holder (Luma));

   function See (Image : Driver.Images.Image) return Picture is
      Seen : Real_Access := new Real_Array (1 .. Driver.Images.Width (Image) * Driver.Images.Height (Image));
   begin
      Driver.Images.Luma (Image, Seen.all);
      declare
         Made : constant Picture := See (Seen.all, Driver.Images.Width (Image), Driver.Images.Height (Image));
      begin
         Free (Seen);
         return Made;
      end;
   end See;

   function Score (P : Patch; Under : Picture; By : Real) return Real is
   begin
      if P.Column.Is_Empty or else Under.Luma.Is_Empty then
         return Real'First;
      end if;
      return Score_Of (P.Column.Element, P.Row.Element, P.Across_U.Element, P.Across_V.Element, P.Dark_Inside,
                       Under.Luma.Constant_Reference.Element.all, Under.Width, Under.Height, P.U, P.V, By);
   end Score;

   procedure Measure (P : Patch; Under : Picture; Result : out Shift) is
      K : constant Natural := Natural (Real'Ceiling (Real'Max (P.Reach, 2.0)));
   begin
      Result := (others => <>);
      if P.Column.Is_Empty or else Under.Luma.Is_Empty then
         return;
      end if;
      declare
         Cols       : constant Natural_Array := P.Column.Element;
         Rows       : constant Natural_Array := P.Row.Element;
         Us         : constant Real_Array := P.Across_U.Element;
         Vs         : constant Real_Array := P.Across_V.Element;
         Under_Luma : Real_Array renames Under.Luma.Constant_Reference.Element.all;
         All_Curve  : constant Real_Array :=
           Curve_Of (Cols, Rows, Us, Vs, P.Dark_Inside, Under_Luma, Under.Width, Under.Height, P.U, P.V, K);
         Middle, Spread : Real;
         All_Peak   : constant Peak_Found := Chosen_Peak (All_Curve, K, P.Rest, Middle, Spread);
         Cut        : constant Real := Cut_Of (Cols, Rows, P.U, P.V);
         Lower      : constant Peak_Found :=
           Half_Peak (Cols, Rows, Us, Vs, P.Dark_Inside, Under_Luma, Under.Width, Under.Height, P.U, P.V, K, Cut,
                      (if All_Peak.Proper then All_Peak.By else P.Rest_Low), True);
         Higher     : constant Peak_Found :=
           Half_Peak (Cols, Rows, Us, Vs, P.Dark_Inside, Under_Luma, Under.Width, Under.Height, P.U, P.V, K, Cut,
                      (if All_Peak.Proper then All_Peak.By else P.Rest_High), False);
         Greatest   : constant Peak_Found := Peak_Of (All_Curve, K);
      begin
         Result.Still := All_Curve (K + 1);
         Result.Typical := Spread;
         --  What is told of the finger is the peak nearest where it stood free of those that stand out of the
         --  other scores, one test of as many as there are shifts; with none the greatest score, which is not
         --  known to be anything.
         if All_Peak.Proper then
            Result.Peak := All_Peak.Score;
            Result.By := All_Peak.By - P.Rest;
         elsif Greatest.Score > Real'First then
            Result.Peak := Greatest.Score;
            Result.By := Greatest.By - P.Rest;
         end if;
         Result.Known := All_Peak.Proper and then Lower.Proper and then Higher.Proper;
         if Result.Known then
            --  The patch's own estimate is as far from the truth as half the difference of its two halves',
            --  each from where it stood free, and the shifts of whole pixels it was found among are known to a
            --  twelfth of a pixel's square.
            Result.Sigma :=
              Sqrt ((((Lower.By - P.Rest_Low) - (Higher.By - P.Rest_High)) / 2.0) ** 2 + Quantization);
         end if;
      end;
   end Measure;

end Driver.Robot.Hand.Slide;
