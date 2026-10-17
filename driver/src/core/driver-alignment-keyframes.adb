with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Stats;
with Driver.Uncertain;
with System.Multiprocessors;

package body Driver.Alignment.Keyframes is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Z : constant Real := Driver.Conventions.Z;

   Least_Points : constant := 2 * Terms;
   --  Found points a field is fitted to at the least: as many as would leave its scatter as many degrees of
   --  freedom as it has coefficients.

   type Natural_Array is array (Positive range <>) of Natural;

   --  Where a point of the reference was found in a keyframe, and how well the aligner said it knew.
   type Located is record
      Found : Boolean := False;
      Place : Driver.Images.Pixel := (U => 0.0, V => 0.0);
      Sigma : Real := 0.0;   --  the larger of the answer's two standard deviations
   end record;

   type Located_Array is array (Positive range <>) of Located;

   --  How far a point moves along each axis.
   type Displacement is record
      Du, Dv : Real := 0.0;
   end record;

   ---------------------------------------------------------------------------
   --  Smooth fields

   --  The terms of a quadratic at a place, the coordinates taken as fractions of the half picture from its centre.
   function Basis (C : Chain; P : Driver.Images.Pixel) return Real_Array is
      X : constant Real := (P.U - Real (C.Width) / 2.0) / (Real (C.Width) / 2.0);
      Y : constant Real := (P.V - Real (C.Height) / 2.0) / (Real (C.Height) / 2.0);
   begin
      return [1.0, X, Y, X * X, X * Y, Y * Y];
   end Basis;

   function Evaluate (C : Chain; Coefficients : Real_Array; P : Driver.Images.Pixel) return Real is
      B   : constant Real_Array := Basis (C, P);
      Sum : Real := 0.0;
   begin
      for K in B'Range loop
         Sum := Sum + Coefficients (K) * B (K);
      end loop;
      return Sum;
   end Evaluate;

   --  The derivative of the place a field carries a point to by the point's place: the identity and the field's
   --  gradient.
   function Gradient (C : Chain; F : Field_Fit; P : Driver.Images.Pixel) return Linear_Part is
      Half_U : constant Real := Real (C.Width) / 2.0;
      Half_V : constant Real := Real (C.Height) / 2.0;
      X      : constant Real := (P.U - Half_U) / Half_U;
      Y      : constant Real := (P.V - Half_V) / Half_V;
   begin
      return (UU => 1.0 + (F.U (2) + 2.0 * F.U (4) * X + F.U (5) * Y) / Half_U,
              UV => (F.U (3) + F.U (5) * X + 2.0 * F.U (6) * Y) / Half_V,
              VU => (F.V (2) + 2.0 * F.V (4) * X + F.V (5) * Y) / Half_U,
              VV => 1.0 + (F.V (3) + F.V (5) * X + 2.0 * F.V (6) * Y) / Half_V);
   end Gradient;

   --  The points of the found ones that most of them agree with, by an affine field of displacements through the
   --  three points of a sample, the sample whose field leaves the least median of its residuals; the points whose
   --  residual along each axis is within Z robust scales (the median of a residual's length over the median of the
   --  length of a two-dimensional Gaussian's). The samples are spread triples: the points in order along an axis
   --  (the picture's two and its two diagonals), one from each third, so that all the points get their turn
   --  and no three lie close. False when no sample has a field.
   procedure Robust_Start (C : Chain; Placed : Located_Array; Keep : in out Flag_Array) is
      Count : constant Natural := Placed'Length;
      Seen  : Natural := 0;
      Found : Boolean := False;
   begin
      for I in 1 .. Count loop
         if Placed (I).Found then
            Seen := Seen + 1;
         end if;
      end loop;
      if Seen < 2 * Terms then
         return;
      end if;
      declare
         Index : Natural_Array (1 .. Seen);
         Along : Real_Array (1 .. Seen);
         Third : constant Positive := Seen / 3;
         Best  : Real := Real'Last;
         Plane : Real_Array (1 .. 6) := [others => 0.0];   --  the best sample's field, u then v: constant, x, y
         Residual : Real_Array (1 .. Seen);
         K : Natural := 0;

         --  The fraction of the half picture from the centre, as Basis has it.
         function X_Of (I : Positive) return Real is
           ((C.Points (I).U - Real (C.Width) / 2.0) / (Real (C.Width) / 2.0));
         function Y_Of (I : Positive) return Real is
           ((C.Points (I).V - Real (C.Height) / 2.0) / (Real (C.Height) / 2.0));

         procedure Try (A, B, D : Positive) is
            M      : Real_Matrix (1 .. 3, 1 .. 3);
            Bu, Bv : Real_Vector (1 .. 3);
            Xu, Xv : Real_Vector (1 .. 3);
            Full_U, Full_V : Boolean;
            Rows   : constant array (1 .. 3) of Positive := [A, B, D];
         begin
            for R in 1 .. 3 loop
               M (R, 1) := 1.0;
               M (R, 2) := X_Of (Index (Rows (R)));
               M (R, 3) := Y_Of (Index (Rows (R)));
               Bu (R) := Placed (Index (Rows (R))).Place.U - C.Points (Index (Rows (R))).U;
               Bv (R) := Placed (Index (Rows (R))).Place.V - C.Points (Index (Rows (R))).V;
            end loop;
            Driver.Numerics.Dense.Least_Squares (M, Bu, Xu, Full_U);
            Driver.Numerics.Dense.Least_Squares (M, Bv, Xv, Full_V);
            if not (Full_U and then Full_V) then
               return;
            end if;
            for S in 1 .. Seen loop
               declare
                  P  : constant Positive := Index (S);
                  Pu : constant Real := Xu (1) + Xu (2) * X_Of (P) + Xu (3) * Y_Of (P);
                  Pv : constant Real := Xv (1) + Xv (2) * X_Of (P) + Xv (3) * Y_Of (P);
               begin
                  Residual (S) := Sqrt ((Placed (P).Place.U - C.Points (P).U - Pu) ** 2
                                        + (Placed (P).Place.V - C.Points (P).V - Pv) ** 2);
               end;
            end loop;
            declare
               Middle : constant Real := Driver.Stats.Median (Residual);
            begin
               if Middle < Best then
                  Best := Middle;
                  Plane := [Xu (1), Xu (2), Xu (3), Xv (1), Xv (2), Xv (3)];
                  Found := True;
               end if;
            end;
         end Try;
      begin
         for I in 1 .. Count loop
            if Placed (I).Found then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
         for Family in 1 .. 4 loop
            --  The points in order along the axis of the family, by insertion.
            for S in 1 .. Seen loop
               Along (S) :=
                 (case Family is
                     when 1 => X_Of (Index (S)),
                     when 2 => Y_Of (Index (S)),
                     when 3 => X_Of (Index (S)) + Y_Of (Index (S)),
                     when others => X_Of (Index (S)) - Y_Of (Index (S)));
            end loop;
            declare
               Order : Natural_Array (1 .. Seen) := [others => 0];
            begin
               for S in 1 .. Seen loop
                  declare
                     J : Natural := S - 1;
                  begin
                     while J >= 1 and then Along (Order (J)) > Along (S) loop
                        Order (J + 1) := Order (J);
                        J := J - 1;
                     end loop;
                     Order (J + 1) := S;
                  end;
               end loop;
               for S in 1 .. Third loop
                  Try (Order (S), Order (S + Third), Order (S + 2 * Third));
               end loop;
            end;
         end loop;
         if not Found then
            return;
         end if;
         --  The points the best sample's field agrees with.
         for S in 1 .. Seen loop
            declare
               P  : constant Positive := Index (S);
               Pu : constant Real := Plane (1) + Plane (2) * X_Of (P) + Plane (3) * Y_Of (P);
               Pv : constant Real := Plane (4) + Plane (5) * X_Of (P) + Plane (6) * Y_Of (P);
               Du : constant Real := Placed (P).Place.U - C.Points (P).U - Pu;
               Dv : constant Real := Placed (P).Place.V - C.Points (P).V - Pv;
               Scale : constant Real := Best / Sqrt (2.0 * Log (2.0));
            begin
               Keep (P) := abs Du <= Z * Scale and then abs Dv <= Z * Scale;
            end;
         end loop;
      end;
   end Robust_Start;

   Max_Degree : constant := 2;
   --  The quadratic of the place is the highest field (Terms of its terms).

   function Terms_Of (Degree : Natural) return Positive is ((Degree + 1) * (Degree + 2) / 2);
   --  The terms of a polynomial of that degree in the place's two coordinates: Basis lists them lowest degree first,
   --  so the terms of a lower degree are the first ones.

   --  One degree's least-squares fit of the displacements of the points whose bases are the rows of A: its
   --  coefficients (zero beyond its terms), the inverse of its normal matrix (row by row, as Field_Fit has it), and
   --  its leave-one-out error: the mean over the points of the squared error each would be predicted with by the
   --  fit of the others, both axes together. Not Ok when the points are too few for the degree (as many as its
   --  terms twice), or its normal matrix is not positive definite, or a point alone decides its own fit.
   procedure Fit_Degree
     (A      : Real_Matrix;
      Bu, Bv : Real_Vector;
      Degree : Natural;
      Xu, Xv : out Real_Array;
      Inv    : out Real_Array;
      Leave  : out Real;
      Ok     : out Boolean)
   is
      Rows   : constant Natural := A'Length (1);
      P      : constant Positive := Terms_Of (Degree);
      Normal : Real_Matrix (1 .. P, 1 .. P) := [others => [others => 0.0]];
      Factor : Real_Matrix (1 .. P, 1 .. P);
      Gain   : Real_Matrix (1 .. P, 1 .. P);   --  the inverse of Normal
      Rhs_U, Rhs_V : Real_Vector (1 .. P) := [others => 0.0];
      Pd     : Boolean;

      function Basis_At (R, I : Positive) return Real is (A (A'First (1) + R - 1, A'First (2) + I - 1));
   begin
      Xu := [others => 0.0];
      Xv := [others => 0.0];
      Inv := [others => 0.0];
      Leave := Real'Last;
      Ok := False;
      if Rows < 2 * P then
         return;
      end if;
      for R in 1 .. Rows loop
         for I in 1 .. P loop
            Rhs_U (I) := Rhs_U (I) + Basis_At (R, I) * Bu (Bu'First + R - 1);
            Rhs_V (I) := Rhs_V (I) + Basis_At (R, I) * Bv (Bv'First + R - 1);
            for J in 1 .. P loop
               Normal (I, J) := Normal (I, J) + Basis_At (R, I) * Basis_At (R, J);
            end loop;
         end loop;
      end loop;
      Driver.Numerics.Dense.Cholesky (Normal, Factor, Pd);
      if not Pd then
         return;
      end if;
      for J in 1 .. P loop
         declare
            Unit   : Real_Vector (1 .. P) := [others => 0.0];
            Column : Real_Vector (1 .. P);
         begin
            Unit (J) := 1.0;
            Column := Driver.Numerics.Dense.Cholesky_Solve (Factor, Unit);
            for I in 1 .. P loop
               Gain (I, J) := Column (I);
               Inv ((I - 1) * Terms + J) := Column (I);
            end loop;
         end;
      end loop;
      declare
         Coefficients_U : constant Real_Vector := Gain * Rhs_U;
         Coefficients_V : constant Real_Vector := Gain * Rhs_V;
         Sum            : Real := 0.0;
      begin
         for I in 1 .. P loop
            Xu (I) := Coefficients_U (I);
            Xv (I) := Coefficients_V (I);
         end loop;
         for R in 1 .. Rows loop
            declare
               Leverage : Real := 0.0;
               Miss_U   : Real := Bu (Bu'First + R - 1);
               Miss_V   : Real := Bv (Bv'First + R - 1);
            begin
               for I in 1 .. P loop
                  Miss_U := Miss_U - Xu (I) * Basis_At (R, I);
                  Miss_V := Miss_V - Xv (I) * Basis_At (R, I);
                  for J in 1 .. P loop
                     Leverage := Leverage + Basis_At (R, I) * Gain (I, J) * Basis_At (R, J);
                  end loop;
               end loop;
               if 1.0 - Leverage <= Real'Epsilon then
                  return;
               end if;
               Sum := Sum + ((Miss_U / (1.0 - Leverage)) ** 2 + (Miss_V / (1.0 - Leverage)) ** 2) / 2.0;
            end;
         end loop;
         Leave := Sum / Real (Rows);
      end;
      Ok := True;
   end Fit_Degree;

   --  The displacements of the found points as a smooth field, fitted to the points that lie within Z robust
   --  scales of it, until the points that do are the same, starting from those a robust affine field agrees with
   --  (Robust_Start). Its degree (a constant, an affine map or a quadratic of the place) is the one whose
   --  leave-one-out error over those points is the least: a quadratic fitted to few or crowded points carries its
   --  error to the places they do not cover, which is no field to extrapolate (Variance says how far to believe it
   --  at a place). Agree says which points the final field agrees with; none when it has no field.
   procedure Fit_Field
     (C : Chain; Placed : Located_Array; Result : out Field_Fit; Agree : out Flag_Array)
   is
      Count  : constant Natural := Placed'Length;
      Keep   : Flag_Array (1 .. Count);
      Seen   : Natural := 0;
   begin
      Result := (others => <>);
      for I in 1 .. Count loop
         Keep (I) := Placed (I).Found;
         if Keep (I) then
            Seen := Seen + 1;
         end if;
      end loop;
      Robust_Start (C, Placed, Keep);
      for Pass in 1 .. Real'Machine_Mantissa loop
         declare
            Used : Natural := 0;
         begin
            for I in 1 .. Count loop
               if Keep (I) then
                  Used := Used + 1;
               end if;
            end loop;
            exit when Used < Least_Points;
            declare
               A      : Real_Matrix (1 .. Used, 1 .. Terms);
               Bu, Bv : Real_Vector (1 .. Used);
               Best   : Real := Real'Last;
               K      : Natural := 0;
            begin
               for I in 1 .. Count loop
                  if Keep (I) then
                     K := K + 1;
                     declare
                        Row : constant Real_Array := Basis (C, C.Points (I));
                     begin
                        for J in 1 .. Terms loop
                           A (K, J) := Row (J);
                        end loop;
                        Bu (K) := Placed (I).Place.U - C.Points (I).U;
                        Bv (K) := Placed (I).Place.V - C.Points (I).V;
                     end;
                  end if;
               end loop;
               Result.Valid := False;
               for Degree in 0 .. Max_Degree loop
                  declare
                     Xu, Xv : Real_Array (1 .. Terms);
                     Inv    : Real_Array (1 .. Terms * Terms);
                     Leave  : Real;
                     Ok     : Boolean;
                  begin
                     Fit_Degree (A, Bu, Bv, Degree, Xu, Xv, Inv, Leave, Ok);
                     if Ok and then Leave < Best then
                        Best := Leave;
                        Result.U := Xu;
                        Result.V := Xv;
                        Result.Inverse := Inv;
                        Result.Order := Terms_Of (Degree);
                        Result.Valid := True;
                     end if;
                  end;
               end loop;
            end;
            exit when not Result.Valid;
            declare
               Ru, Rv : Real_Array (1 .. Seen);
               K      : Natural := 0;
            begin
               for I in 1 .. Count loop
                  if Placed (I).Found then
                     K := K + 1;
                     Ru (K) := Placed (I).Place.U - C.Points (I).U - Evaluate (C, Result.U, C.Points (I));
                     Rv (K) := Placed (I).Place.V - C.Points (I).V - Evaluate (C, Result.V, C.Points (I));
                  end if;
               end loop;
               declare
                  Su   : constant Real := Driver.Stats.Robust_Sigma (Ru);
                  Sv   : constant Real := Driver.Stats.Robust_Sigma (Rv);
                  Next : Flag_Array (1 .. Count) := [others => False];
               begin
                  Result.Sigma := Sqrt ((Su ** 2 + Sv ** 2) / 2.0);
                  Result.Support := Used;
                  K := 0;
                  for I in 1 .. Count loop
                     if Placed (I).Found then
                        K := K + 1;
                        Next (I) := abs Ru (K) <= Z * Su and then abs Rv (K) <= Z * Sv;
                     end if;
                  end loop;
                  exit when Next = Keep;
                  Keep := Next;
               end;
            end;
         end;
      end loop;
      if Result.Valid then
         Agree := Keep;
      else
         Agree := [others => False];
      end if;
   end Fit_Field;

   --  The variance of the displacement the field gives at a place: what it leaves of the points it was fitted to,
   --  and, since its coefficients are estimated from them, the more the farther the place lies from them (the
   --  variance of a prediction from a least-squares fit).
   function Variance (C : Chain; F : Field_Fit; P : Driver.Images.Pixel) return Real is
      B   : constant Real_Array := Basis (C, P);
      Sum : Real := 0.0;
   begin
      for I in 1 .. F.Order loop
         for J in 1 .. F.Order loop
            Sum := Sum + B (I) * F.Inverse ((I - 1) * Terms + J) * B (J);
         end loop;
      end loop;
      return F.Sigma ** 2 * (1.0 + Sum);
   end Variance;

   ---------------------------------------------------------------------------
   --  Predictions

   function Dot (A, B : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for K in A'Range loop
         Sum := Sum + A (K) * B (K);
      end loop;
      return Sum;
   end Dot;

   --  How far a change of readings can have moved the view, at most: its entries' sizes times the pixels a unit of
   --  each moves it.
   function Moved_By (Change, Shift : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for K in Change'Range loop
         Sum := Sum + abs Change (K) * Shift (K);
      end loop;
      return Sum;
   end Moved_By;

   function To_Array (V : Real_Vectors.Vector) return Real_Array is
      Result : Real_Array (1 .. Natural (V.Length));
   begin
      for K in Result'Range loop
         Result (K) := V (K);
      end loop;
      return Result;
   end To_Array;

   --  Whether a change of readings is none the noise of the channels can tell from nothing: a difference of two
   --  readings has the noise of each.
   function Is_None (Change, Noise : Real_Array) return Boolean is
     (for all K in Change'Range => not Driver.Uncertain.Significant (Change (K), Noise (K) * Sqrt (2.0)));

   --  Whether every channel the change moved (by more than the noise tells) has a known shift: when one has not,
   --  how far the change can have moved the view is not known at all.
   function Shifts_Known (Change, Noise, Shift : Real_Array) return Boolean is
     (for all K in Change'Range =>
        Shift (K) > 0.0 or else not Driver.Uncertain.Significant (Change (K), Noise (K) * Sqrt (2.0)));

   --  How far along the direction of Change an earlier keyframe lies (signed, in the units of the readings), and
   --  how far at most the part of its change off that direction can have moved the view: the sizes of that part's
   --  entries times the pixels a unit of each channel moves the view; as far as anything when an entry of that part
   --  the noise can tell is of a channel whose shift is not known.
   procedure Along (Before : Frame; Change, Noise, Shift : Real_Array; Amount, Off : out Real) is
      Past : constant Real_Array := To_Array (Before.Change);
      Own  : constant Real := Dot (Change, Change);
      Unit : constant Real := Dot (Past, Change) / Own;
   begin
      Amount := Unit * Sqrt (Own);
      Off := 0.0;
      for K in Change'Range loop
         if Shift (K) > 0.0 then
            Off := Off + abs (Past (K) - Unit * Change (K)) * Shift (K);
         elsif Driver.Uncertain.Significant (Past (K) - Unit * Change (K), Noise (K) * Sqrt (2.0)) then
            Off := Real'Last;
            return;
         end if;
      end loop;
   end Along;

   function Order_Of (Second : Natural) return Positive is (if Second = 0 then 1 else Orders);

   --  The weights of the fields of one earlier keyframe (Second = 0) or two, which lie at the amounts Amounts along
   --  the direction, for a keyframe at the amount Target: a field linear in the amount through the reference
   --  (where nothing has moved) and the first keyframe, or the quadratic in the amount through the reference and
   --  both.
   procedure Weights (Amounts : Real_Array; First, Second : Natural; Target : Real; W1, W2 : out Real) is
      A1 : constant Real := Amounts (First);
   begin
      if Second = 0 then
         W1 := Target / A1;
         W2 := 0.0;
      else
         declare
            A2 : constant Real := Amounts (Second);
         begin
            W1 := Target * (A2 - Target) / (A1 * (A2 - A1));
            W2 := Target * (Target - A1) / (A2 * (A2 - A1));
         end;
      end if;
   end Weights;

   --  The standard deviation of what the fields carry into a prediction at a place: their scatter there (Variance:
   --  more where they were fitted to no points near) and their off-direction parts, through the weights.
   function Carried_At (C : Chain; Guess : Candidate; P : Driver.Images.Pixel) return Real is
      One   : constant Real :=
        Guess.W1 ** 2 * (Variance (C, C.Frames (Guess.First).Field, P) + Guess.Off1 ** 2);
      Other : constant Real :=
        (if Guess.Second = 0 then 0.0
         else Guess.W2 ** 2 * (Variance (C, C.Frames (Guess.Second).Field, P) + Guess.Off2 ** 2));
   begin
      return Sqrt (One + Other);
   end Carried_At;

   function Move_Of (C : Chain; P : Driver.Images.Pixel; Guess : Candidate) return Displacement is
      Result : Displacement :=
        (Du => Guess.W1 * Evaluate (C, C.Frames (Guess.First).Field.U, P),
         Dv => Guess.W1 * Evaluate (C, C.Frames (Guess.First).Field.V, P));
   begin
      if Guess.Second > 0 then
         Result.Du := Result.Du + Guess.W2 * Evaluate (C, C.Frames (Guess.Second).Field.U, P);
         Result.Dv := Result.Dv + Guess.W2 * Evaluate (C, C.Frames (Guess.Second).Field.V, P);
      end if;
      return Result;
   end Move_Of;

   --  The standard deviation a prediction states of a move: what the fields carry and the relative error of the
   --  move.
   function Stated_Sigma (C : Chain; Guess : Candidate; P : Driver.Images.Pixel; Move : Displacement) return Real is
     (Sqrt (Carried_At (C, Guess, P) ** 2 + (Guess.Relative ** 2) * (Move.Du ** 2 + Move.Dv ** 2)));

   --  The predictions of every point from the fields of the earlier keyframes of a candidate.
   procedure Extrapolate (C : Chain; Guess : Candidate; Into : out Prediction_Array) is
   begin
      for K in Into'Range loop
         declare
            P     : constant Driver.Images.Pixel := C.Points (K - Into'First + 1);
            Move  : constant Displacement := Move_Of (C, P, Guess);
            Slope_1    : constant Linear_Part := Gradient (C, C.Frames (Guess.First).Field, P);
            Slope_2    : constant Linear_Part :=
              (if Guess.Second = 0 then Identity_Part else Gradient (C, C.Frames (Guess.Second).Field, P));
            Sigma : constant Real := Stated_Sigma (C, Guess, P, Move);
         begin
            Into (K) :=
              (From => P, To => (U => P.U + Move.Du, V => P.V + Move.Dv),
               Linear => (UU => 1.0 + Guess.W1 * (Slope_1.UU - 1.0) + Guess.W2 * (Slope_2.UU - 1.0),
                          UV => Guess.W1 * Slope_1.UV + Guess.W2 * Slope_2.UV,
                          VU => Guess.W1 * Slope_1.VU + Guess.W2 * Slope_2.VU,
                          VV => 1.0 + Guess.W1 * (Slope_1.VV - 1.0) + Guess.W2 * (Slope_2.VV - 1.0)),
               Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2),
               Linear_Sigma => Real'Last);
         end;
      end loop;
   end Extrapolate;

   ---------------------------------------------------------------------------
   --  Predictions as sums of single-channel moves

   --  Whether a change of the channel moves the view by more than the aligner errs by (Precision: the scatter of the
   --  keyframes' fields, Scatter). Shift is the pixels a unit of the channel moves the view: zero when that is not
   --  known, and the change then moves it by no amount known.
   function Moves_View (Change, Shift : Real_Array; K : Positive; Precision : Real) return Boolean is
     (Driver.Uncertain.Significant (Change (K) * Shift (K), Precision));

   --  Whether every channel but K that the change moved, by more than the noise of its readings tells, has a known
   --  shift: what the change did besides channel K is then bounded.
   function Others_Known (Change, Noise, Shift : Real_Array; K : Positive) return Boolean is
     (for all I in Change'Range =>
        I = K or else Shift (I) > 0.0 or else not Driver.Uncertain.Significant (Change (I), Noise (I) * Sqrt (2.0)));

   function Channels_Moving (Change, Shift : Real_Array; Precision : Real) return Natural is
      Count : Natural := 0;
   begin
      for K in Change'Range loop
         if Moves_View (Change, Shift, K, Precision) then
            Count := Count + 1;
         end if;
      end loop;
      return Count;
   end Channels_Moving;

   --  The variance of what the fields of a combination carry into a prediction at a place: the scatter of each
   --  (Variance) and its off-channel part, through its weight.
   function Carried_By (C : Chain; Combo : Combination; P : Driver.Images.Pixel) return Real is
      Sum : Real := 0.0;
   begin
      for Each of Combo.Parts loop
         Sum := Sum + Each.Weight ** 2 * (Variance (C, C.Frames (Each.Frame).Field, P) + Each.Off ** 2);
      end loop;
      return Sum;
   end Carried_By;

   function Move_By (C : Chain; Combo : Combination; P : Driver.Images.Pixel) return Displacement is
      Result : Displacement;
   begin
      for Each of Combo.Parts loop
         Result.Du := Result.Du + Each.Weight * Evaluate (C, C.Frames (Each.Frame).Field.U, P);
         Result.Dv := Result.Dv + Each.Weight * Evaluate (C, C.Frames (Each.Frame).Field.V, P);
      end loop;
      return Result;
   end Move_By;

   function Stated_Sigma (C : Chain; Combo : Combination; P : Driver.Images.Pixel; Move : Displacement) return Real is
     (Sqrt (Carried_By (C, Combo, P) + (Combo.Relative ** 2) * (Move.Du ** 2 + Move.Dv ** 2)));

   --  The predictions of every point from the sum of the moves of a combination.
   procedure Extrapolate (C : Chain; Combo : Combination; Into : out Prediction_Array) is
   begin
      for K in Into'Range loop
         declare
            P     : constant Driver.Images.Pixel := C.Points (K - Into'First + 1);
            Move  : constant Displacement := Move_By (C, Combo, P);
            Sigma : constant Real := Stated_Sigma (C, Combo, P, Move);
            Slope : Linear_Part := Identity_Part;
         begin
            for Each of Combo.Parts loop
               declare
                  G : constant Linear_Part := Gradient (C, C.Frames (Each.Frame).Field, P);
               begin
                  Slope := (UU => Slope.UU + Each.Weight * (G.UU - 1.0), UV => Slope.UV + Each.Weight * G.UV,
                            VU => Slope.VU + Each.Weight * G.VU, VV => Slope.VV + Each.Weight * (G.VV - 1.0));
               end;
            end loop;
            Into (K) :=
              (From => P, To => (U => P.U + Move.Du, V => P.V + Move.Dv), Linear => Slope,
               Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2), Linear_Sigma => Real'Last);
         end;
      end loop;
   end Extrapolate;

   --  How curved the moves are, from every keyframe that has an estimate of it, each believed as far as its weight
   --  (Frame): the weighted mean of the estimates. A weight is the inverse variance of its estimate, so the mean
   --  stands as many standard errors above zero as the keyframes together can tell; curvature that the keyframes do
   --  not separate from none (the noise of a move hardly bigger than the aligner's, over the tiny distance it
   --  went, is a thousand times any curvature) is none.
   function Curvature_Of (Sum, Total : Real) return Real is
     (if Total > 0.0 and then Driver.Uncertain.Significant (Sum / Total, 1.0 / Sqrt (Total)) and then Sum > 0.0
      then Sqrt (Sum / Total) else 0.0);

   function Pooled_Order (C : Chain; Order : Positive) return Real is
      Sum, Total : Real := 0.0;
   begin
      for F of C.Frames loop
         Sum := Sum + F.Weight (Order) * F.Squared (Order);
         Total := Total + F.Weight (Order);
      end loop;
      return Curvature_Of (Sum, Total);
   end Pooled_Order;

   --  The same of the predictions as sums of single-channel moves; until one has been judged, the curvature of the
   --  linear extrapolation along a direction (the moves of the channels are curved alike, and a sum of them is as
   --  much more so as the amounts it adds).
   function Pooled_Sum (C : Chain) return Real is
      Sum, Total : Real := 0.0;
   begin
      for F of C.Frames loop
         Sum := Sum + F.Weight_Sum * F.Squared_Sum;
         Total := Total + F.Weight_Sum;
      end loop;
      return (if Total > 0.0 then Curvature_Of (Sum, Total) else Pooled_Order (C, 1));
   end Pooled_Sum;

   ---------------------------------------------------------------------------

   procedure Begin_Reference (C : out Chain; Reference : Driver.Images.Image; Points : Point_Array) is
   begin
      C := (Pictures => Pyramid_Of (Reference), Width => Driver.Images.Width (Reference),
            Height => Driver.Images.Height (Reference), others => <>);
      for P of Points loop
         C.Points.Append (P);
      end loop;
   end Begin_Reference;

   function Has_Reference (C : Chain) return Boolean is (Levels (C.Pictures) > 0);

   function Point_Count (C : Chain) return Natural is (Natural (C.Points.Length));

   function Keyframes (C : Chain) return Natural is (Natural (C.Frames.Length));

   function Scatter (C : Chain) return Real is
      Sigmas : Real_Array (1 .. Natural (C.Frames.Length));
      Count  : Natural := 0;
      --  Measured where nothing moved when such a keyframe has a field (the
      --  reference's still twin: the points are found again in a picture
      --  that differs by the sensor's noise alone, and what the field leaves
      --  is the aligner's own error); else over the keyframes that moved,
      --  where what the field leaves is also what the scene's depth adds to
      --  a smooth field, and grows with the motion. The keyframe step rests on
      --  this error, and the sweep, the keyframes it keeps and the fit's
      --  reading of which joints a keyframe moved all rest on the step: on a
      --  fake body whose arm eye saw boxes, the error over the moved frames
      --  grew from 0.2 to 0.5 px during the sweep, the step with it, and the
      --  levels the ladders had climbed no longer counted as motion (V4: no
      --  joint had a keyframe of its own after 64 followed keyframes, and the
      --  frames that moved the joints together were not even new).
      procedure Gather (Moved : Boolean) is
      begin
         Count := 0;
         for F of C.Frames loop
            if F.Moved = Moved and then F.Field.Valid then
               Count := Count + 1;
               Sigmas (Count) := F.Field.Sigma;
            end if;
         end loop;
      end Gather;
   begin
      Gather (Moved => False);
      if Count = 0 then
         Gather (Moved => True);
      end if;
      return (if Count = 0 then 0.0 else Driver.Stats.Median (Sigmas (1 .. Count)));
   end Scatter;

   procedure Predict
     (C       : in out Chain;
      Change  : Driver.Real_Array;
      Noise   : Driver.Real_Array;
      Shift   : Driver.Real_Array;
      Offered : Prediction_Array;
      Into    : out Prediction_Array;
      From    : out Source)
   is
      Changes : constant Real_Array (1 .. Change'Length) := Change;
      Noises  : constant Real_Array (1 .. Noise'Length) := Noise;
      Shifts  : constant Real_Array (1 .. Shift'Length) := Shift;
      Frames  : constant Natural := Natural (C.Frames.Length);
      Count   : constant Natural := Natural (C.Points.Length);

      --  The points have not moved, to within Spread.
      procedure Stay (Spread : Real) is
      begin
         for K in Into'Range loop
            declare
               P : constant Driver.Images.Pixel := C.Points (K - Into'First + 1);
            begin
               Into (K) := (From => P, To => P, Linear => Identity_Part,
                            Cov => (UU => Spread ** 2, UV => 0.0, VV => Spread ** 2), Linear_Sigma => Real'Last);
            end;
         end loop;
      end Stay;
   begin
      C.Latest := [others => <>];
      C.Combined := (others => <>);
      From := Same_Pose;
      if Count = 0 or else Is_None (Changes, Noises) then
         Stay (0.0);
         C.Last := Same_Pose;
         return;
      end if;
      declare
         Own     : constant Real := Sqrt (Dot (Changes, Changes));
         --  How far the change can have moved the points, at most: a unit of each channel moves the view by
         --  its shift. When no shift is known, as far as the aligner searches.
         Moved   : constant Real := Moved_By (Changes, Shifts);
         Reach   : constant Real :=
           (if Moved > 0.0 and then Shifts_Known (Changes, Noises, Shifts) then Moved
            else Largest_Sigma (C.Pictures, C.Pictures));
         --  How far the aligner errs on these keyframes' views: what a change must move the view by to be told from none.
         Precision : constant Real := Scatter (C);
         Amounts : Real_Array (1 .. Frames) := [others => 0.0];
         Off     : Real_Array (1 .. Frames) := [others => 0.0];
         Usable  : Flag_Array (1 .. Frames) := [others => False];
         Sigmas  : Real_Array (1 .. Count);
         Curve   : Order_Errors := [others => 0.0];   --  per order, the largest curvature the keyframes along it showed
         Best    : Real := Reach;
         Chosen  : Natural := 0;   --  the order of the candidate that states the least, when one states less than Best
         Offer_Wins : Boolean := False;
         Combined_Wins : Boolean := False;

         --  What every usable keyframe's field says at every point, taken once: its scatter there and its
         --  displacement. The candidates below ask them Frames squared times (118 keyframes: 7,000 pairs over 230
         --  points, four field evaluations each, which grew with the square of the keyframes; item 2). A candidate's
         --  arithmetic is the same as Carried_At, Move_Of and Stated_Sigma, on these.
         type Real_Grid is array (Positive range <>, Positive range <>) of Real;
         type Grid_Access is access Real_Grid;
         procedure Free is new Ada.Unchecked_Deallocation (Real_Grid, Grid_Access);
         Field_Var : Grid_Access := new Real_Grid'(1 .. Frames => [1 .. Count => 0.0]);
         Field_U   : Grid_Access := new Real_Grid'(1 .. Frames => [1 .. Count => 0.0]);
         Field_V   : Grid_Access := new Real_Grid'(1 .. Frames => [1 .. Count => 0.0]);

         --  The prediction from the earlier keyframes First and Second (unless zero), kept if it states the least
         --  of its order.
         procedure Consider (First, Second : Natural) is
            Order    : constant Positive := Order_Of (Second);
            Dist     : constant Real :=
              abs (Own - Amounts (First)) * (if Second = 0 then 1.0 else abs (Own - Amounts (Second)));
            Guess    : Candidate :=
              (Valid => True, First => First, Second => Second, Curve => Curve (Order), Dist => Dist,
               Relative => Curve (Order) * Dist, Off1 => Off (First), Off2 => (if Second = 0 then 0.0 else Off (Second)),
               others => <>);
         begin
            Weights (Amounts, First, Second, Own, Guess.W1, Guess.W2);
            for K in Sigmas'Range loop
               declare
                  One     : constant Real := Guess.W1 ** 2 * (Field_Var (First, K) + Guess.Off1 ** 2);
                  Other   : constant Real :=
                    (if Second = 0 then 0.0 else Guess.W2 ** 2 * (Field_Var (Second, K) + Guess.Off2 ** 2));
                  Carried : constant Real := Sqrt (One + Other);
                  Du      : Real := Guess.W1 * Field_U (First, K);
                  Dv      : Real := Guess.W1 * Field_V (First, K);
               begin
                  if Second > 0 then
                     Du := Du + Guess.W2 * Field_U (Second, K);
                     Dv := Dv + Guess.W2 * Field_V (Second, K);
                  end if;
                  Sigmas (K) := Sqrt (Carried ** 2 + (Guess.Relative ** 2) * (Du ** 2 + Dv ** 2));
               end;
            end loop;
            Guess.Median := Driver.Stats.Median (Sigmas);
            if Guess.Median < C.Latest (Order).Median then
               C.Latest (Order) := Guess;
            end if;
         end Consider;
      begin
         if Offered'Length > 0 then
            for K in Sigmas'Range loop
               declare
                  Cov : Place_Covariance renames Offered (Offered'First + K - 1).Cov;
               begin
                  Sigmas (K) := Sqrt (Real'Max (Cov.UU, Cov.VV));
               end;
            end loop;
            if Driver.Stats.Median (Sigmas) < Best then
               Best := Driver.Stats.Median (Sigmas);
               Offer_Wins := True;
            end if;
         end if;
         for J in 1 .. Frames loop
            Along (C.Frames (J), Changes, Noises, Shifts, Amounts (J), Off (J));
            --  A keyframe that tells nothing of the direction (the reference's twin, or one at right angles), or
            --  whose off-direction part alone carries more into the prediction than not having moved at all,
            --  gives no prediction that could state less.
            Usable (J) := C.Frames (J).Field.Valid and then Amounts (J) /= 0.0
              and then not Is_None (To_Array (C.Frames (J).Change), Noises)
              and then Off (J) < Real'Last and then Own * Off (J) < Reach * abs Amounts (J);
         end loop;
         for J in 1 .. Frames loop
            if Usable (J) then
               for K in 1 .. Count loop
                  Field_Var (J, K) := Variance (C, C.Frames (J).Field, C.Points (K));
                  Field_U (J, K) := Evaluate (C, C.Frames (J).Field.U, C.Points (K));
                  Field_V (J, K) := Evaluate (C, C.Frames (J).Field.V, C.Points (K));
               end loop;
            end if;
         end loop;
         --  How curved the moves are: what the keyframes showed, each believed as far as it stood above the noise.
         for Order in 1 .. Orders loop
            Curve (Order) := Pooled_Order (C, Order);
         end loop;
         for J1 in 1 .. Frames loop
            if Usable (J1) then
               Consider (J1, 0);
               for J2 in J1 + 1 .. Frames loop
                  if Usable (J2) and then Amounts (J1) /= Amounts (J2) then
                     Consider (J1, J2);
                  end if;
               end loop;
            end if;
         end loop;
         --  A change of several channels at once is, to first order, the moves of each: the sum of the moves of the
         --  keyframes that changed one channel each, the nearest in amount to this change's (of the same sign when
         --  there is one), scaled to it.
         --  Only when every channel the change moved has a known shift: a channel whose shift is not known moves the
         --  view by no amount known, and was counted as moving it by none (f15h on 10-15: keyframes of six channels
         --  turned 1 to 8 units each, four of them of no known shift, predicted from two channels' moves stating
         --  0.1 px, and not one point was found). So for the keyframes the sum is made of.
         if Frames > 0 and then Shifts_Known (Changes, Noises, Shifts)
           and then Channels_Moving (Changes, Shifts, Precision) >= 2
         then
            declare
               Combo : Combination;
               Whole : Boolean := True;
            begin
               for K in Changes'Range loop
                  if Moves_View (Changes, Shifts, K, Precision) then
                     declare
                        Nearest_Frame : Natural := 0;
                        Same_Sign     : Boolean := False;
                        Gap           : Real := Real'Last;
                     begin
                        for J in 1 .. Frames loop
                           declare
                              Past : constant Real_Array := To_Array (C.Frames (J).Change);
                           begin
                              if C.Frames (J).Field.Valid and then Moves_View (Past, Shifts, K, Precision)
                                and then Channels_Moving (Past, Shifts, Precision) = 1
                                and then Others_Known (Past, Noises, Shifts, K)
                              then
                                 declare
                                    Same  : constant Boolean := Past (K) * Changes (K) > 0.0;
                                    Apart : constant Real := abs Log (abs Past (K) / abs Changes (K));
                                 begin
                                    if (Same and then not Same_Sign) or else (Same = Same_Sign and then Apart < Gap) then
                                       Nearest_Frame := J;
                                       Same_Sign := Same;
                                       Gap := Apart;
                                    end if;
                                 end;
                              end if;
                           end;
                        end loop;
                        if Nearest_Frame = 0 then
                           Whole := False;
                        else
                           declare
                              Past : constant Real_Array := To_Array (C.Frames (Nearest_Frame).Change);
                              Away : Real := 0.0;
                           begin
                              for I in Past'Range loop
                                 if I /= K then
                                    Away := Away + abs Past (I) * Shifts (I);
                                 end if;
                              end loop;
                              Combo.Parts.Append
                                (Part'(Frame => Nearest_Frame, Weight => Changes (K) / Past (K), Off => Away));
                              Combo.Amount := Combo.Amount + abs Changes (K);
                           end;
                        end if;
                     end;
                  end if;
               end loop;
               if Whole then
                  Combo.Valid := True;
                  Combo.Relative := Pooled_Sum (C) * Combo.Amount;
                  for K in Sigmas'Range loop
                     Sigmas (K) := Stated_Sigma (C, Combo, C.Points (K), Move_By (C, Combo, C.Points (K)));
                  end loop;
                  Combo.Median := Driver.Stats.Median (Sigmas);
                  C.Combined := Combo;
               end if;
            end;
         end if;
         --  Where two candidates can be made they disagree by as much as the move is curved beyond what either
         --  allows for: the curvature each allows for was measured on keyframes aligned in the windows earlier
         --  predictions set, which cannot show an error wider than those windows (s29 replayed and judged against
         --  the simulator's truth, 10-17: keyframes extrapolated to 33 degrees of the wrist's roll stated 1.0 px and
         --  were found 9 px off the true motion). Each candidate's covariance at a point takes the largest
         --  disagreement of another there, as the outer product of the difference of their moves, and the one that
         --  then states the least is chosen.
         declare
            type Move_Grid is array (0 .. Orders, 1 .. Count) of Displacement;   --  0: the combination
            Moves    : Move_Grid;
            Apart    : Move_Grid := [others => [others => (Du => 0.0, Dv => 0.0)]];
            Own_Sig  : array (0 .. Orders, 1 .. Count) of Real := [others => [others => 0.0]];
            Can      : array (0 .. Orders) of Boolean := [others => False];
            Median_Of : array (0 .. Orders) of Real := [others => Real'Last];
         begin
            Can (0) := C.Combined.Valid;
            for Order in 1 .. Orders loop
               Can (Order) := C.Latest (Order).Valid;
            end loop;
            for X in 0 .. Orders loop
               if Can (X) then
                  for K in 1 .. Count loop
                     declare
                        P : constant Driver.Images.Pixel := C.Points (K);
                     begin
                        if X = 0 then
                           Moves (X, K) := Move_By (C, C.Combined, P);
                           Own_Sig (X, K) := Stated_Sigma (C, C.Combined, P, Moves (X, K));
                        else
                           Moves (X, K) := Move_Of (C, P, C.Latest (X));
                           Own_Sig (X, K) := Stated_Sigma (C, C.Latest (X), P, Moves (X, K));
                        end if;
                     end;
                  end loop;
               end if;
            end loop;
            for X in 0 .. Orders loop
               if Can (X) then
                  for K in 1 .. Count loop
                     for Y in 0 .. Orders loop
                        if Y /= X and then Can (Y) then
                           declare
                              D : constant Displacement :=
                                (Du => Moves (Y, K).Du - Moves (X, K).Du, Dv => Moves (Y, K).Dv - Moves (X, K).Dv);
                           begin
                              if D.Du ** 2 + D.Dv ** 2 > Apart (X, K).Du ** 2 + Apart (X, K).Dv ** 2 then
                                 Apart (X, K) := D;
                              end if;
                           end;
                        end if;
                     end loop;
                     Sigmas (K) := Sqrt (Own_Sig (X, K) ** 2 + Real'Max (Apart (X, K).Du ** 2, Apart (X, K).Dv ** 2));
                  end loop;
                  Median_Of (X) := Driver.Stats.Median (Sigmas);
               end if;
            end loop;
            for Order in 1 .. Orders loop
               if Can (Order) and then Median_Of (Order) < Best then
                  Best := Median_Of (Order);
                  Chosen := Order;
                  Offer_Wins := False;
               end if;
            end loop;
            if Can (0) and then Median_Of (0) < Best then
               Best := Median_Of (0);
               Chosen := 0;
               Offer_Wins := False;
               Combined_Wins := True;
            end if;
            if Offer_Wins then
               From := Supplied;
               Into := Offered;
            elsif Combined_Wins then
               From := Extrapolated;
               Extrapolate (C, C.Combined, Into);
            elsif Chosen = 0 then
               From := New_Direction;
               Stay (Reach);
            else
               From := Extrapolated;
               Extrapolate (C, C.Latest (Chosen), Into);
            end if;
            if From = Extrapolated then
               for K in Into'Range loop
                  declare
                     D : Displacement renames Apart ((if Combined_Wins then 0 else Chosen), K - Into'First + 1);
                  begin
                     Into (K).Cov := (UU => Into (K).Cov.UU + D.Du ** 2, UV => Into (K).Cov.UV + D.Du * D.Dv,
                                      VV => Into (K).Cov.VV + D.Dv ** 2);
                  end;
               end loop;
            end if;
         end;
         C.Last := From;
         Free (Field_Var);
         Free (Field_U);
         Free (Field_V);
      end;
   end Predict;

   --  The aligner's refusal of a point may be the prediction's doing and not the point's: it found no place within
   --  what the prediction allowed, or several, or none that beats chance.
   function Prediction_Caused (A : Answer) return Boolean is
     (A.Verdict = Not_Found
      and then A.Because in Window_Too_Large | Ambiguous | No_Evidence | Left_Window | Not_Settled | Bad_Warp);

   function Agreeing_Of (Flags : Flag_Array) return Natural is
      Count : Natural := 0;
   begin
      for F of Flags loop
         if F then
            Count := Count + 1;
         end if;
      end loop;
      return Count;
   end Agreeing_Of;

   --  The points of the reference that Only marks, into the second picture from their predictions, each point an
   --  index of its own handed to whichever worker is idle (a point's alignment reads the two pyramids and writes only
   --  its own answer and mark); where an answer was found, its mark has the place it says, as the field of
   --  displacements is fitted to the marks. The points were cut into one run of them a processor: a point's
   --  alignment takes from one pass to dozens, and the task that waits for the round waited for the slowest run
   --  (s07's estimate at beat 2842, replayed on 10-16: 1171 of its 6756 samples aligning).
   procedure Align_Some
     (C       : Chain;
      Second  : Pyramid;
      Given   : Prediction_Array;
      Only    : Flag_Array;
      Answers : in out Answer_Array;
      Marks   : in out Located_Array;
      Widen   : Boolean := True)
   is
      Count  : constant Natural := Given'Length;
      Asked  : array (1 .. Count) of Positive;   --  the points Only marks, in their order
      Number : Natural := 0;

      procedure One (Index : Positive) is
         K      : constant Positive := Asked (Index);
         Result : Answer renames Answers (Answers'First + K - 1);
      begin
         Result := Align (C.Pictures, Second, Given (Given'First + K - 1), Widen);
         Marks (K) :=
           (if Result.Verdict = Found
            then (Found => True, Place => Result.To, Sigma => Sqrt (Real'Max (Result.Cov.UU, Result.Cov.VV)))
            else (others => <>));
      end One;

      procedure Run is new Driver.Parallel_For (One);
   begin
      for K in 1 .. Count loop
         if Only (Only'First + K - 1) then
            Number := Number + 1;
            Asked (Number) := K;
         end if;
      end loop;
      Run (1, Number);
   end Align_Some;

   --  Where a keyframe's own field puts a point, to the uncertainty the field has there.
   function Asked_From (C : Chain; Field : Field_Fit; P : Driver.Images.Pixel) return Prediction is
      Spread : constant Real := Variance (C, Field, P);
   begin
      return (From => P, To => (U => P.U + Evaluate (C, Field.U, P), V => P.V + Evaluate (C, Field.V, P)),
              Linear => Gradient (C, Field, P), Cov => (UU => Spread, UV => 0.0, VV => Spread),
              Linear_Sigma => Real'Last);
   end Asked_From;

   --  The points the prediction did not place (refused for what it said of where to look, or found where the field of
   --  the others does not agree) are asked again from the keyframe's own field, which the answers that agree with
   --  one another have made: where it puts them, to the uncertainty it has there. So are the points never asked: the rounds
   --  that make the field stop once it settles, and the points they left are the fit's as much as any (f15q on 10-15:
   --  2132 of 3570 points of an arm's first fourteen keyframes never asked, its joints sharing too few points from one
   --  keyframe to the next for their distances to be told apart). So are the points found where
   --  it agrees but from a window wider than a patch at the full size: their answers are not ones to rest a fit on
   --  until something better than the prediction is known of the move (Answer.Narrow), and the field is that (f15h on
   --  10-15: a keyframe predicted stating 19.8 px had 118 answers on its field and none narrow, so not one was found
   --  and the keyframe was not followed). The field is then fitted to all
   --  the answers, and the asking goes on while more of them agree.
   procedure Settle
     (C       : Chain;
      Second  : Pyramid;
      Answers : in out Answer_Array;
      Marks   : in out Located_Array;
      Field   : in out Field_Fit;
      Agree   : in out Flag_Array)
   is
      Count    : constant Natural := Marks'Length;
      Given    : Prediction_Array (1 .. Count);
      Todo     : Flag_Array (1 .. Count);
      Agreeing : Natural := Agreeing_Of (Agree);
   begin
      loop
         exit when not Field.Valid;
         Todo := [others => False];
         for K in 1 .. Count loop
            declare
               Seen : Answer renames Answers (Answers'First + K - 1);
            begin
               if (not Agree (K) and then (Prediction_Caused (Seen) or else Seen.Verdict = Found))
                 or else (Agree (K) and then Seen.Verdict = Found and then not Seen.Narrow)
                 or else Seen.Because = Not_Asked
               then
                  Todo (K) := True;
                  Given (K) := Asked_From (C, Field, C.Points (K));
               end if;
            end;
         end loop;
         exit when Agreeing_Of (Todo) = 0;
         Align_Some (C, Second, Given, Todo, Answers, Marks, Widen => False);
         declare
            Refit : Field_Fit;
            Again : Flag_Array (1 .. Count);
         begin
            Fit_Field (C, Marks, Refit, Again);
            exit when not Refit.Valid or else Agreeing_Of (Again) <= Agreeing;
            Field := Refit;
            Agree := Again;
            Agreeing := Agreeing_Of (Again);
         end;
      end loop;
   end Settle;

   procedure Agreement (C : Chain; Answers : Answer_Array; Agree : out Flag_Array) is
      Marks : Located_Array (1 .. Answers'Length);
      Field : Field_Fit;
      Mine  : Flag_Array (1 .. Answers'Length);
   begin
      for K in Marks'Range loop
         declare
            Seen : Answer renames Answers (Answers'First + K - 1);
         begin
            if Seen.Verdict = Found then
               Marks (K) := (Found => True, Place => Seen.To, Sigma => Sqrt (Real'Max (Seen.Cov.UU, Seen.Cov.VV)));
            end if;
         end;
      end loop;
      Fit_Field (C, Marks, Field, Mine);
      Agree := Mine;
   end Agreement;

   procedure Realign
     (C       : Chain;
      Image   : Driver.Images.Image;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array)
   is
      Second : constant Pyramid := Pyramid_Of (Image);
      Marks  : Located_Array (1 .. Given'Length);
      Field  : Field_Fit;
      Mine   : Flag_Array (1 .. Given'Length);
   begin
      Align_Some (C, Second, Given, [1 .. Given'Length => True], Answers, Marks);
      Fit_Field (C, Marks, Field, Mine);
      Settle (C, Second, Answers, Marks, Field, Mine);
      Agree := Mine;
   end Realign;

   --  Whether a field puts every point where another does, within what the two know of the place there.
   procedure Forget_Last (C : in out Chain) is
   begin
      C.Frames.Delete_Last;
      C.Last := Supplied;
      C.Latest := [others => <>];
      C.Combined := (others => <>);
   end Forget_Last;

   function Unmoved (C : Chain; Old, Now : Field_Fit) return Boolean is
   begin
      for P of C.Points loop
         declare
            S : constant Real := Sqrt (Variance (C, Old, P) + Variance (C, Now, P));
         begin
            if Driver.Uncertain.Significant (Evaluate (C, Now.U, P) - Evaluate (C, Old.U, P), S)
              or else Driver.Uncertain.Significant (Evaluate (C, Now.V, P) - Evaluate (C, Old.V, P), S)
            then
               return False;
            end if;
         end;
      end loop;
      return True;
   end Unmoved;

   ---------------------------------------------------------------------------
   --  The view's common motion

   --  A place where a point's patch correlates with the keyframe better than at the eight places around it, all
   --  within the window: how far the point moved to get there, in pixels of the size scanned, and the correlation.
   type Peak is record
      Du, Dv : Real := 0.0;
      Rho    : Real := 0.0;
   end record;

   type Peak_Array is array (Positive range <>) of Peak;
   type Peaks_Access is access Peak_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Peak_Array, Peaks_Access);
   type Peak_Lists is array (Positive range <>) of Peaks_Access;
   type Flags_Access is access Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flags_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Reals_Access);

   --  The field of displacements d (p) = M p + T of a place p, in pixels of the size scanned.
   type Affine is record
      M11, M12, M21, M22, Tu, Tv : Real := 0.0;
   end record;

   function Move_U (F : Affine; U, V : Real) return Real is (F.M11 * U + F.M12 * V + F.Tu);
   function Move_V (F : Affine; U, V : Real) return Real is (F.M21 * U + F.M22 * V + F.Tv);

   --  Every point's peaks over a window of Radius (pixels of the full size) at size Size of the pyramids: the
   --  candidates are the second picture's pixels there, as Align scans a window (Align_At), and a point whose patch
   --  lies more than half outside the reference's picture has none.
   procedure Scan_Peaks
     (C : Chain; Second : Pyramid; Riding : Flag_Array; Size : Natural; Radius : Real; Peaks : out Peak_Lists;
      Around : Affine := (others => 0.0))
   is
      Scale  : constant Real := Real (Halving) ** Size;
      Half   : constant Natural := Half_Patch (C.Pictures.Sizes (Size).Width, C.Pictures.Sizes (Size).Height);
      Side   : constant Positive := 2 * Half + 1;
      Reach  : constant Real := Radius / Scale;
      B_Level : Level renames Second.Sizes (Size);
      B_Luma  : Real_Array renames B_Level.Luma.Held.Values.all;

      procedure One (K : Positive) is
         P  : constant Driver.Images.Pixel := C.Points (K);
         Pu : constant Real := P.U / Scale;
         Pv : constant Real := P.V / Scale;
         --  The window's centre: where Around puts the point (the point itself for no field).
         Cu : constant Real := Pu + Move_U (Around, Pu, Pv);
         Cv : constant Real := Pv + Move_V (Around, Pu, Pv);
         X0 : constant Integer := Integer (Real'Floor (Cu - Reach - 0.5));
         X1 : constant Integer := Integer (Real'Ceiling (Cu + Reach - 0.5));
         Y0 : constant Integer := Integer (Real'Floor (Cv - Reach - 0.5));
         Y1 : constant Integer := Integer (Real'Ceiling (Cv + Reach - 0.5));
         Across : constant Positive := X1 - X0 + 1;
         Down   : constant Positive := Y1 - Y0 + 1;
         Template : Real_Array (1 .. Side * Side);
         Valid    : array (1 .. Side * Side) of Boolean := [others => False];
         Present  : Natural := 0;
         Rho      : Reals_Access := new Real_Array'(1 .. Across * Down => -2.0);   --  -2: no candidate there
         Count    : Natural := 0;
         function Cell (X, Y : Integer) return Positive is ((Y - Y0) * Across + (X - X0) + 1);
         function In_Picture (X, Y : Integer) return Boolean is
           (X >= 0 and then Y >= 0 and then X < B_Level.Width and then Y < B_Level.Height);
      begin
         Peaks (K) := null;
         if Riding'Length > 0 and then Riding (Riding'First + K - 1) then
            Free (Rho);
            return;
         end if;
         declare
            I : Natural := 0;
            Value, Du, Dv : Real;
            Ok : Boolean;
         begin
            for J in -Integer (Half) .. Integer (Half) loop
               for Ii in -Integer (Half) .. Integer (Half) loop
                  I := I + 1;
                  Sample_Level (C.Pictures, Size, Pu + Real (Ii), Pv + Real (J), Value, Du, Dv, Ok);
                  Template (I) := Value;
                  Valid (I) := Ok;
                  if Ok then
                     Present := Present + 1;
                  end if;
               end loop;
            end loop;
         end;
         if 2 * Present < Side * Side then
            Free (Rho);
            return;
         end if;
         for Y in Y0 .. Y1 loop
            for X in X0 .. X1 loop
               if (Real (X) + 0.5 - Cu) ** 2 + (Real (Y) + 0.5 - Cv) ** 2 <= Reach ** 2 then
                  declare
                     Sum_S, Sum_T, Sum_SS, Sum_TT, Sum_ST : Real := 0.0;
                     N : Natural := 0;
                     I : Natural := 0;
                  begin
                     for J in -Integer (Half) .. Integer (Half) loop
                        for Ii in -Integer (Half) .. Integer (Half) loop
                           I := I + 1;
                           if Valid (I) and then In_Picture (X + Ii, Y + J) then
                              declare
                                 S : constant Real := B_Luma (B_Luma'First + (Y + J) * B_Level.Width + X + Ii);
                                 T : constant Real := Template (I);
                              begin
                                 N := N + 1;
                                 Sum_S := Sum_S + S;
                                 Sum_T := Sum_T + T;
                                 Sum_SS := Sum_SS + S * S;
                                 Sum_TT := Sum_TT + T * T;
                                 Sum_ST := Sum_ST + S * T;
                              end;
                           end if;
                        end loop;
                     end loop;
                     if 2 * N >= Side * Side then
                        declare
                           Nr  : constant Real := Real (N);
                           Vss : constant Real := Sum_SS - Sum_S ** 2 / Nr;
                           Vtt : constant Real := Sum_TT - Sum_T ** 2 / Nr;
                           Vst : constant Real := Sum_ST - Sum_S * Sum_T / Nr;
                        begin
                           if Vss > 0.0 and then Vtt > 0.0 then
                              Rho (Cell (X, Y)) := Vst / Sqrt (Vss * Vtt);
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end loop;
         end loop;
         --  The peaks: better than every one of the eight candidates around, all of which are in the window (a
         --  place on the window's rim is the end of a slope, not a place of the point's own).
         declare
            function Is_Peak (X, Y : Integer) return Boolean is
               Here : constant Real := Rho (Cell (X, Y));
            begin
               if Here <= 0.0 then
                  return False;
               end if;
               for Ey in -1 .. 1 loop
                  for Ex in -1 .. 1 loop
                     if Ex /= 0 or else Ey /= 0 then
                        if X + Ex not in X0 .. X1 or else Y + Ey not in Y0 .. Y1
                          or else Rho (Cell (X + Ex, Y + Ey)) <= -2.0
                          or else Rho (Cell (X + Ex, Y + Ey)) > Here
                        then
                           return False;
                        end if;
                     end if;
                  end loop;
               end loop;
               return True;
            end Is_Peak;
         begin
            for Y in Y0 .. Y1 loop
               for X in X0 .. X1 loop
                  if Is_Peak (X, Y) then
                     Count := Count + 1;
                  end if;
               end loop;
            end loop;
            if Count > 0 then
               Peaks (K) := new Peak_Array (1 .. Count);
               Count := 0;
               for Y in Y0 .. Y1 loop
                  for X in X0 .. X1 loop
                     if Is_Peak (X, Y) then
                        Count := Count + 1;
                        Peaks (K) (Count) :=
                          (Du => Real (X) + 0.5 - Pu, Dv => Real (Y) + 0.5 - Pv, Rho => Rho (Cell (X, Y)));
                     end if;
                  end loop;
               end loop;
            end if;
         end;
         Free (Rho);
      end One;

      procedure Run is new Driver.Parallel_For (One);
   begin
      Run (Peaks'First, Peaks'Last);
   end Scan_Peaks;

   procedure Common_Motion
     (C      : Chain;
      Second : Pyramid;
      Spread : Real;
      Riding : Flag_Array;
      Into   : out Prediction_Array;
      Result : out Motion;
      Stood  : out Flag_Array;
      Moved  : out Flag_Array)
   is
      Count  : constant Natural := Natural (C.Points.Length);
      Radius : constant Real := Window_Radius (Spread);
      Start  : constant Natural := Window_Level (C.Pictures, Second, Radius);
      --  Two places, each known to the pixel of the size scanned, agree within the window of their difference.
      Tolerance : constant Real := Window_Multiple * Sqrt (2.0 * Quantization);
      --  How sure the drawing must be not to have missed a field: the tail probability Z has.
      Miss : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Z);
      Gen  : Ada.Numerics.Float_Random.Generator;
      --  The field told at a coarser size, the size's scale, and how far off it puts a point (pixels of the full
      --  size): every finer size down to the finest tried refines it (Refit), each scanning only as far as it is
      --  uncertain, centred where it puts each point.
      Have_Prior   : Boolean := False;
      Prior        : Affine;
      Prior_Scale  : Real := 1.0;
      Prior_Spread : Real := 0.0;
      --  The sizes tried: from the one Align searches the window at, finer while the field is not told, down to the
      --  coarsest size where a place known to its pixel is a window a patch carries at the full size (Align's own
      --  test of a narrow window): a finer size narrows the prediction no further and scans four times the places
      --  a halving. A coarse size may hold too few places of their own to tell any (the boot's rigs, 64 by 48:
      --  told at 32 by 24, not at 16 by 12; the finest tried there is 32 by 24, at 320 by 240 it is 80 by 60).
      function Finest_Tried return Natural is
         Finest_Narrow : Natural := Finest;
      begin
         for Size in Finest .. Start loop
            exit when Window_Radius (Real (Halving) ** Size * Sqrt (Quantization)) + 1.0
                        > Real (Half_Patch (C.Width, C.Height));
            Finest_Narrow := Size;
         end loop;
         return Finest_Narrow;
      end Finest_Tried;
      Finest_Size : constant Natural := Finest_Tried;
   begin
      Result := (others => <>);
      Stood := [others => False];
      Moved := [others => False];
      Ada.Numerics.Float_Random.Reset (Gen, Count);
      for Size in reverse Finest_Size .. Start loop
         declare
            Scale : constant Real := Real (Halving) ** Size;
            Peaks : Peak_Lists (1 .. Count) := [others => null];
            --  Per point and peak: still drawn from (not a place of a field already taken).
            type Flag_Lists is array (1 .. Count) of Flags_Access;
            Open  : Flag_Lists := [others => null];
            Usable : Natural := 0;

            --  The points at the size, kept as plain numbers: the draws read them on several tasks at once (Draw),
            --  and a vector's element is read through a reference that counts its readers.
            Point_U : constant Real_Array (1 .. Count) := [for K in 1 .. Count => C.Points (K).U / Scale];
            Point_V : constant Real_Array (1 .. Count) := [for K in 1 .. Count => C.Points (K).V / Scale];

            function U_Of (K : Positive) return Real is (Point_U (K));
            function V_Of (K : Positive) return Real is (Point_V (K));

            --  The open peak of point K nearest the field's displacement there, and its squared distance from it.
            --  Of the open peaks within the tolerance down the window, the nearest (none farther is ever on a field). The
            --  peaks run down the window row by row (Scan_Peaks): the first of them is found by halving, and a draw
            --  costs the rows within the tolerance, not every peak of the point (f15c replayed on 10-15: at the full
            --  size a point had hundreds, and the draws sat for ten minutes).
            procedure Nearest (F : Affine; K : Positive; Index : out Natural; Gap : out Real) is
               Du : constant Real := Move_U (F, U_Of (K), V_Of (K));
               Dv : constant Real := Move_V (F, U_Of (K), V_Of (K));
            begin
               Index := 0;
               Gap := Real'Last;
               if Peaks (K) /= null then
                  declare
                     P  : Peak_Array renames Peaks (K).all;
                     Lo : Positive := P'First;
                     Hi : Positive := P'Last + 1;
                  begin
                     while Lo < Hi loop
                        declare
                           Mid : constant Positive := (Lo + Hi) / 2;
                        begin
                           if P (Mid).Dv < Dv - Tolerance then
                              Lo := Mid + 1;
                           else
                              Hi := Mid;
                           end if;
                        end;
                     end loop;
                     for I in Lo .. P'Last loop
                        exit when P (I).Dv > Dv + Tolerance;
                        if Open (K) (I) then
                           declare
                              D : constant Real := (P (I).Du - Du) ** 2 + (P (I).Dv - Dv) ** 2;
                           begin
                              if D < Gap then
                                 Gap := D;
                                 Index := I;
                              end if;
                           end;
                        end if;
                     end loop;
                  end;
               end if;
            end Nearest;

            function On_Field (F : Affine; K : Positive) return Boolean is
               I   : Natural;
               Gap : Real;
            begin
               Nearest (F, K, I, Gap);
               return I > 0 and then Gap <= Tolerance ** 2;
            end On_Field;

            function Agreeing (F : Affine) return Natural is
               N : Natural := 0;
            begin
               for K in 1 .. Count loop
                  if On_Field (F, K) then
                     N := N + 1;
                  end if;
               end loop;
               return N;
            end Agreeing;

            --  The affine field through the places on F, by least squares; F itself when they do not determine one.
            function Refit (F : Affine) return Affine is
               N : Natural := 0;
               A : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
               Bu, Bv : Real_Vector (1 .. 3) := [others => 0.0];
            begin
               for K in 1 .. Count loop
                  declare
                     I   : Natural;
                     Gap : Real;
                  begin
                     Nearest (F, K, I, Gap);
                     if I > 0 and then Gap <= Tolerance ** 2 then
                        declare
                           Row : constant Real_Vector (1 .. 3) := [U_Of (K), V_Of (K), 1.0];
                        begin
                           N := N + 1;
                           for R in 1 .. 3 loop
                              for S in 1 .. 3 loop
                                 A (R, S) := A (R, S) + Row (R) * Row (S);
                              end loop;
                              Bu (R) := Bu (R) + Row (R) * Peaks (K) (I).Du;
                              Bv (R) := Bv (R) + Row (R) * Peaks (K) (I).Dv;
                           end loop;
                        end;
                     end if;
                  end;
               end loop;
               if N < 3 then
                  return F;
               end if;
               declare
                  L      : Real_Matrix (1 .. 3, 1 .. 3);
                  Definite : Boolean;
               begin
                  Driver.Numerics.Dense.Cholesky (A, L, Definite);
                  if not Definite then
                     return F;
                  end if;
                  declare
                     Xu : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, Bu);
                     Xv : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, Bv);
                  begin
                     return (M11 => Xu (1), M12 => Xu (2), Tu => Xu (3), M21 => Xv (1), M22 => Xv (2), Tv => Xv (3));
                  end;
               end;
            end Refit;

            --  The field the most points agree with, of those drawn from two open places at a time (a turn, a scale
            --  and a shift), the points drawn alike and each point's places in proportion to their correlation, as
            --  many draws as find with probability 1 - Miss a field that half the points have a typical place on,
            --  fewer once the best so far shows that many find it; then refitted while more points agree.
            procedure Draw (Best : out Affine; Best_N : out Natural) is
               Drawable : Natural := 0;
               Total    : Natural := 0;
               Index    : array (1 .. Count) of Natural := [others => 0];   --  the points with open places
               function Weight (K : Positive) return Real is
                  Sum : Real := 0.0;
               begin
                  for I in Peaks (K)'Range loop
                     if Open (K) (I) then
                        Sum := Sum + Peaks (K) (I).Rho;
                     end if;
                  end loop;
                  return Sum;
               end Weight;
               function Pick_Place (K : Positive) return Positive is
                  Target : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen)) * Weight (K);
                  Sum    : Real := 0.0;
                  Last   : Positive := 1;
               begin
                  for I in Peaks (K)'Range loop
                     if Open (K) (I) then
                        Sum := Sum + Peaks (K) (I).Rho;
                        Last := I;
                        if Sum >= Target then
                           return I;
                        end if;
                     end if;
                  end loop;
                  return Last;
               end Pick_Place;
               function Pick_Point return Positive is
                  R : constant Natural :=
                    Natural'Min (Drawable - 1,
                                 Natural (Real'Floor (Real (Ada.Numerics.Float_Random.Random (Gen)) * Real (Drawable))));
               begin
                  return Index (R + 1);
               end Pick_Point;
               function Needed (W : Real) return Natural is
                 (if W >= 1.0 then 1
                  elsif W <= 0.0 then Natural'Last
                  else Natural (Real'Min (Real (Natural'Last), Real'Ceiling (Log (Miss) / Log (1.0 - W * W)))));
               Cap    : Natural;
               Enough : Natural;
               Drawn  : Natural := 0;

               --  The draws go in batches of one a processor, as Align_Some splits its points: a batch's fields drawn
               --  from the generator in order, as the draws one after another draw them, the points agreeing with
               --  each counted at once (Driver.Parallel_For: Agreeing reads only the places and the points), and the
               --  batch taken in order, each draw only while the draws one at a time would still go on, the generator
               --  set back to where it stood after the last draw taken. The best field, its count, the draws needed
               --  and the generator are those of the draws one at a time, to the bit. One at a time, the draws were
               --  most of the aligner's time on the task that waits for it: s07's estimate at beat 2842, replayed
               --  (10-16), aligned for 9.9 s on that task, 5.4 of them drawing, with nine workers idle.
               Batch : constant Positive := Positive'Max (1, Natural (System.Multiprocessors.Number_Of_CPUs));
               type Field_Draw is record
                  Valid : Boolean := False;   --  two points apart, a view the aligner compares
                  G     : Affine := (others => 0.0);
                  N     : Natural := 0;       --  the points agreeing with G
               end record;
               Draws : array (1 .. Batch) of Field_Draw;
               After : array (1 .. Batch) of Ada.Numerics.Float_Random.State;

               procedure Count_Agreeing (J : Positive) is
               begin
                  if Draws (J).Valid then
                     Draws (J).N := Agreeing (Draws (J).G);
                  end if;
               end Count_Agreeing;
               procedure Count_All is new Driver.Parallel_For (Count_Agreeing);

               --  A draw's field: two points, and a place of each when they are not the same point.
               function Drawn_Field return Field_Draw is
                  K1 : constant Positive := Pick_Point;
                  K2 : constant Positive := Pick_Point;
               begin
                  if K1 /= K2 then
                     declare
                        P1 : constant Peak := Peaks (K1) (Pick_Place (K1));
                        P2 : constant Peak := Peaks (K2) (Pick_Place (K2));
                        Dpu : constant Real := U_Of (K2) - U_Of (K1);
                        Dpv : constant Real := V_Of (K2) - V_Of (K1);
                        Dqu : constant Real := Dpu + P2.Du - P1.Du;
                        Dqv : constant Real := Dpv + P2.Dv - P1.Dv;
                        Den : constant Real := Dpu ** 2 + Dpv ** 2;
                     begin
                        --  Two points closer than the tolerance tell no turn.
                        if Den > Tolerance ** 2 then
                           declare
                              A  : constant Real := (Dpu * Dqu + Dpv * Dqv) / Den;
                              B  : constant Real := (Dpu * Dqv - Dpv * Dqu) / Den;
                              Sq : constant Real := A ** 2 + B ** 2;
                           begin
                              --  A view the aligner can compare: no scale beyond a halving either way (Align).
                              if Sq >= 1.0 / Real (Halving) ** 2 and then Sq <= Real (Halving) ** 2 then
                                 declare
                                    Q1u : constant Real := U_Of (K1) + P1.Du;
                                    Q1v : constant Real := V_Of (K1) + P1.Dv;
                                 begin
                                    --  The view turned and scaled by (A, B) and shifted: q = (A, B) p + T at both
                                    --  points, so d (p) = q - p = ((A, B) - I) p + T.
                                    return (Valid => True,
                                            G     => (M11 => A - 1.0, M12 => -B, M21 => B, M22 => A - 1.0,
                                                      Tu => Q1u - (A * U_Of (K1) - B * V_Of (K1)),
                                                      Tv => Q1v - (B * U_Of (K1) + A * V_Of (K1))),
                                            N     => 0);
                                 end;
                              end if;
                           end;
                        end if;
                     end;
                  end if;
                  return (others => <>);
               end Drawn_Field;
            begin
               Best := (others => 0.0);
               Best_N := 0;
               for K in 1 .. Count loop
                  if Peaks (K) /= null and then (for some F of Open (K).all => F) then
                     Drawable := Drawable + 1;
                     Index (Drawable) := K;
                     for F of Open (K).all loop
                        if F then
                           Total := Total + 1;
                        end if;
                     end loop;
                  end if;
               end loop;
               if Drawable < 2 then
                  return;
               end if;
               Cap := Needed (0.5 * Real (Drawable) / Real (Total));
               Enough := Cap;
               while Drawn < Natural'Min (Cap, Enough) loop
                  declare
                     Asked : constant Positive := Natural'Min (Batch, Natural'Min (Cap, Enough) - Drawn);
                     Taken : Natural := 0;
                  begin
                     for J in 1 .. Asked loop
                        Draws (J) := Drawn_Field;
                        Ada.Numerics.Float_Random.Save (Gen, After (J));
                     end loop;
                     Count_All (1, Asked);
                     for J in 1 .. Asked loop
                        exit when Drawn >= Natural'Min (Cap, Enough);
                        Drawn := Drawn + 1;
                        Taken := J;
                        if Draws (J).Valid and then Draws (J).N > Best_N then
                           declare
                              G : constant Affine := Draws (J).G;
                           begin
                              Best := G;
                              Best_N := Draws (J).N;
                              --  How often a draw is of two places on this field: the share of the points on it,
                              --  times the chance their place on it is the one drawn.
                              declare
                                 Chance : Real := 0.0;
                              begin
                                 for K in 1 .. Count loop
                                    declare
                                       I   : Natural;
                                       Gap : Real;
                                    begin
                                       Nearest (G, K, I, Gap);
                                       if I > 0 and then Gap <= Tolerance ** 2 then
                                          Chance := Chance + Peaks (K) (I).Rho / Weight (K);
                                       end if;
                                    end;
                                 end loop;
                                 Enough := Needed (Chance / Real (Drawable));
                              end;
                           end;
                        end if;
                     end loop;
                     if Taken < Asked then
                        Ada.Numerics.Float_Random.Reset (Gen, After (Taken));
                     end if;
                  end;
               end loop;
               Result.Draws := Result.Draws + Drawn;
               if Best_N > 0 then
                  loop
                     declare
                        Next   : constant Affine := Refit (Best);
                        Next_N : constant Natural := Agreeing (Next);
                     begin
                        exit when Next_N <= Best_N;
                        Best := Next;
                        Best_N := Next_N;
                     end;
                  end loop;
               end if;
            end Draw;

            Best, Other : Affine;
            Best_N, Other_N : Natural := 0;
            On_Best : Flag_Array (1 .. Count) := [others => False];
            Was_Prior : constant Boolean := Have_Prior;
            Rescaled  : constant Affine :=
              (M11 => Prior.M11, M12 => Prior.M12, M21 => Prior.M21, M22 => Prior.M22,
               Tu => Prior.Tu * Prior_Scale / Scale, Tv => Prior.Tv * Prior_Scale / Scale);
            Refined   : Boolean := False;
            Stayed    : Flag_Array (1 .. Count) := [others => False];   --  its best place is where it stood
         begin
            if Was_Prior then
               Scan_Peaks (C, Second, Riding, Size, Window_Radius (Prior_Spread), Peaks, Around => Rescaled);
            else
               Scan_Peaks (C, Second, Riding, Size, Radius, Peaks);
            end if;
            --  A point whose best place is where it stood shows nothing of how far the view moved: it rides with the
            --  eye (the hand's jaws in a wrist eye's view, whose edges are the sharpest thing of a coarse size), or the
            --  view moved there by less than this size resolves. It does not draw the field: the jaws drew one near
            --  none for keyframes that moved the view 16 to 25 px, the arm's base and shoulder had no keyframe of their
            --  own, and the arm was swept five passes and not fitted (f15q on 10-15: told 6.3 -3.7 px where the truth
            --  moved 25.0 0.8). Where most points stayed, the view moved by less than this size resolves: a finer size
            --  is asked, and at the finest the field is none (the selftest of a first keyframe of two joints, the view
            --  moved 2.5 px). A field told at a coarser size is refined over windows about it; so is one about a base.
            if not Was_Prior then
               Result.Unmoved := 0;
               for K in 1 .. Count loop
                  if Peaks (K) /= null then
                     declare
                        Top : Positive := Peaks (K)'First;
                     begin
                        for I in Peaks (K)'Range loop
                           if Peaks (K) (I).Rho > Peaks (K) (Top).Rho then
                              Top := I;
                           end if;
                        end loop;
                        if Peaks (K) (Top).Du ** 2 + Peaks (K) (Top).Dv ** 2 <= Tolerance ** 2 then
                           Stayed (K) := True;
                           Result.Unmoved := Result.Unmoved + 1;
                        end if;
                     end;
                  end if;
               end loop;
            end if;
            for K in 1 .. Count loop
               if Peaks (K) /= null then
                  Open (K) := new Flag_Array'(Peaks (K)'Range => not Stayed (K));
                  if not Stayed (K) then
                     Usable := Usable + 1;
                  end if;
               end if;
            end loop;
            if Was_Prior then
               --  The field told at a coarser size, refitted on the places nearest it while more points agree: it
               --  was told against every other field there, and the scan here reaches only as far as it is uncertain.
               declare
                  F : Affine := Rescaled;
                  N : Natural := Agreeing (F);
               begin
                  loop
                     declare
                        G      : constant Affine := Refit (F);
                        Agrees : constant Natural := Agreeing (G);
                     begin
                        exit when Agrees <= N;
                        F := G;
                        N := Agrees;
                     end;
                  end loop;
                  Best := F;
                  Best_N := N;
               end;
               Refined := Best_N >= Least_Points;
               for K in 1 .. Count loop
                  On_Best (K) := On_Field (Best, K);
               end loop;
            elsif Real (Result.Unmoved) - Real (Usable) > Z * Sqrt (Real (Result.Unmoved + Usable)) then
               --  Significantly more points stayed than moved (the sign test of two fields): the view moved by less
               --  than this size resolves, and the few that moved were drawn by their lookalikes (f15u on 10-15: a
               --  wrist turn moved the view 2 to 3 px, 128 of 169 points stood, and the 41 others told a turn of 11
               --  degrees; no point was found, the turn's pixels a unit came out absurd, and every later keyframe of
               --  the arm was asked over windows of millions of pixels). A finer size is asked, and at the finest the
               --  field is none, at the places the points that stayed took. Where those that stayed are a lesser
               --  part, they ride with the eye (the jaws) and the others tell the field.
               Result.Level := Size;
               Result.Points := Result.Unmoved;
               if Size = Finest_Size then
                  Best := (others => 0.0);
                  Best_N := Result.Unmoved;
                  Result.Agree := Best_N;
                  Result.Own := Best_N;
                  Result.Other := 0;
                  Result.Told := True;
                  On_Best := Stayed;
               end if;
            else
               Result.Level := Size;
               Result.Points := Usable;
               Draw (Best, Best_N);
               Result.Agree := Best_N;
               if Best_N > 0 then
                  --  The next best field: drawn from the places the best one does not take.
                  for K in 1 .. Count loop
                     On_Best (K) := On_Field (Best, K);
                     if Peaks (K) /= null then
                        declare
                           Du : constant Real := Move_U (Best, U_Of (K), V_Of (K));
                           Dv : constant Real := Move_V (Best, U_Of (K), V_Of (K));
                        begin
                           for I in Peaks (K)'Range loop
                              if (Peaks (K) (I).Du - Du) ** 2 + (Peaks (K) (I).Dv - Dv) ** 2 <= Tolerance ** 2 then
                                 Open (K) (I) := False;
                              end if;
                           end loop;
                        end;
                     end if;
                  end loop;
                  Draw (Other, Other_N);
                  --  Each point for the field whose place it correlates with better, where it has a place on either: a
                  --  texture that repeats puts most points on a field one period off as well (the boot's rigs: 35 of 40
                  --  points on a field 6 px off the true one), and there the point's own correlation tells them apart.
                  --  A point that stayed where it was votes for neither: it drew no field.
                  Result.Own := 0;
                  Result.Other := 0;
                  for K in 1 .. Count loop
                     declare
                        function Place_On (F : Affine) return Real is
                           Du  : constant Real := Move_U (F, U_Of (K), V_Of (K));
                           Dv  : constant Real := Move_V (F, U_Of (K), V_Of (K));
                           Rho : Real := Real'First;
                        begin
                           if Peaks (K) /= null then
                              for I in Peaks (K)'Range loop
                                 if (Peaks (K) (I).Du - Du) ** 2 + (Peaks (K) (I).Dv - Dv) ** 2 <= Tolerance ** 2 then
                                    Rho := Real'Max (Rho, Peaks (K) (I).Rho);
                                 end if;
                              end loop;
                           end if;
                           return Rho;
                        end Place_On;
                        On_Best  : constant Real := Place_On (Best);
                        On_Other : constant Real := (if Other_N > 0 then Place_On (Other) else Real'First);
                     begin
                        if Stayed (K) then
                           null;
                        elsif On_Best > On_Other then
                           Result.Own := Result.Own + 1;
                        elsif On_Other > On_Best then
                           Result.Other := Result.Other + 1;
                        end if;
                     end;
                  end loop;
                  --  Told: the points prefer one of the two fields significantly (a sign test: each point is for either
                  --  as likely when the two are as good), and the field they prefer is the view's even where fewer
                  --  points have a place on it: a texture that repeats lends the field one period off as many places or
                  --  more (the selftest of two joints, the points that stood still left out: 184 points on the field
                  --  drawn first, 167 preferring the next and 18 it).
                  if Real (Result.Other) - Real (Result.Own) > Z * Sqrt (Real (Result.Own + Result.Other)) then
                     declare
                        Was     : constant Affine := Best;
                        Was_N   : constant Natural := Best_N;
                        Was_Own : constant Natural := Result.Own;
                     begin
                        Best := Other;
                        Best_N := Other_N;
                        Other := Was;
                        Other_N := Was_N;
                        Result.Own := Result.Other;
                        Result.Other := Was_Own;
                        Result.Agree := Best_N;
                     end;
                     --  The places of the field drawn first were taken from the drawing of the next, which are open.
                     for K in 1 .. Count loop
                        On_Best (K) := On_Field (Best, K);
                     end loop;
                  end if;
                  Result.Told :=
                    Real (Result.Own) - Real (Result.Other) > Z * Sqrt (Real (Result.Own + Result.Other));
                  --  Where the field is told, a point whose best place was where it stood while the field moves it
                  --  there beyond a place the size resolves stood against the view; one on the field that moved
                  --  moved with it.
                  if Result.Told then
                     for K in 1 .. Count loop
                        if Peaks (K) /= null then
                           declare
                              Du : constant Real := Move_U (Best, U_Of (K), V_Of (K));
                              Dv : constant Real := Move_V (Best, U_Of (K), V_Of (K));
                              Far : constant Boolean := Du ** 2 + Dv ** 2 > Tolerance ** 2;
                           begin
                              Stood (K) := Stayed (K) and then Far;
                              Moved (K) := On_Best (K) and then not Stayed (K) and then Far;
                           end;
                        end if;
                     end loop;
                  end if;
               end if;
            end if;
            if (if Was_Prior then Refined else Result.Told) then
               declare
                  Residuals : Real_Array (1 .. 2 * Count);
                  R         : Natural := 0;
                  Sigma     : Real;
               begin
                  for K in 1 .. Count loop
                     if On_Best (K) then
                        --  Its place on the field, among all its places (the open ones are no longer all).
                        declare
                           Du : constant Real := Move_U (Best, U_Of (K), V_Of (K));
                           Dv : constant Real := Move_V (Best, U_Of (K), V_Of (K));
                           Gap : Real := Real'Last;
                           Iu, Iv : Real := 0.0;
                        begin
                           for I in Peaks (K)'Range loop
                              if (Peaks (K) (I).Du - Du) ** 2 + (Peaks (K) (I).Dv - Dv) ** 2 < Gap then
                                 Gap := (Peaks (K) (I).Du - Du) ** 2 + (Peaks (K) (I).Dv - Dv) ** 2;
                                 Iu := Peaks (K) (I).Du - Du;
                                 Iv := Peaks (K) (I).Dv - Dv;
                              end if;
                           end loop;
                           Residuals (R + 1) := Iu;
                           Residuals (R + 2) := Iv;
                           R := R + 2;
                        end;
                     end if;
                  end loop;
                  Sigma := Driver.Stats.Robust_Sigma (Residuals (1 .. R));
                  Result.Sigma := Sigma * Scale;
                  Prior_Spread := Sqrt (Sigma ** 2 + Quantization) * Scale;
                  Result.Turn := Arctan (Best.M21 - Best.M12, 2.0 + Best.M11 + Best.M22);
                  declare
                     Cu : constant Real := Real (C.Width) / 2.0;
                     Cv : constant Real := Real (C.Height) / 2.0;
                  begin
                     Result.Shift_U := Scale * Move_U (Best, Cu / Scale, Cv / Scale);
                     Result.Shift_V := Scale * Move_V (Best, Cu / Scale, Cv / Scale);
                  end;
                  --  Every point where the field puts it: as far off as the places on it lie from it, and a
                  --  place known to a pixel of the size scanned; the patch turned and stretched as the field
                  --  turns and stretches the view.
                  for K in Into'Range loop
                     declare
                        P  : constant Driver.Images.Pixel := C.Points (K - Into'First + 1);
                        Pu : constant Real := P.U / Scale;
                        Pv : constant Real := P.V / Scale;
                        Spread : constant Real := (Sigma ** 2 + Quantization) * Scale ** 2;
                     begin
                        Into (K) :=
                          (From => P,
                           To => (U => P.U + Scale * Move_U (Best, Pu, Pv), V => P.V + Scale * Move_V (Best, Pu, Pv)),
                           Linear => (UU => 1.0 + Best.M11, UV => Best.M12, VU => Best.M21, VV => 1.0 + Best.M22),
                           Cov => (UU => Spread, UV => 0.0, VV => Spread),
                           Linear_Sigma => Real'Last);
                     end;
                  end loop;
               end;
               Result.Level := Size;
               Result.Agree := Best_N;
               Prior := Best;
               Prior_Scale := Scale;
               Have_Prior := True;
            end if;
            for K in 1 .. Count loop
               Free (Peaks (K));
               Free (Open (K));
            end loop;
            --  Finer while not told; once told, finer while the refinement holds.
            exit when Was_Prior and then not Refined;
         end;
      end loop;
   end Common_Motion;

   function Last_Motion (C : Chain) return Motion is (C.Motion);

   procedure Keep_Evidence (C : in out Chain; Stood, Moved : Flag_Array) is
   begin
      C.Stood.Clear;
      C.Moved_On.Clear;
      for K in Stood'Range loop
         C.Stood.Append (Stood (K));
         C.Moved_On.Append (Moved (K));
      end loop;
   end Keep_Evidence;

   procedure Stood_And_Moved (C : Chain; Stood, Moved : out Flag_Array) is
   begin
      for K in Stood'Range loop
         declare
            P : constant Positive := K - Stood'First + 1;
         begin
            Stood (K) := P <= Natural (C.Stood.Length) and then C.Stood (P);
            Moved (Moved'First + K - Stood'First) := P <= Natural (C.Moved_On.Length) and then C.Moved_On (P);
         end;
      end loop;
   end Stood_And_Moved;

   procedure Add
     (C       : in out Chain;
      Image   : Driver.Images.Image;
      Change  : Driver.Real_Array;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array;
      Riding  : Flag_Array := No_Riding)
   is
      Second : constant Pyramid := Pyramid_Of (Image);
      Count  : constant Natural := Given'Length;
      Marks  : Located_Array (1 .. Count);
      Mine   : Flag_Array (1 .. Count) := [others => False];
      Next   : Frame;

      --  Whether the field puts the point inside the picture: where the prediction was of any use.
      function In_View (P : Driver.Images.Pixel) return Boolean is
        (P.U + Evaluate (C, Next.Field.U, P) in 0.0 .. Real (C.Width)
         and then P.V + Evaluate (C, Next.Field.V, P) in 0.0 .. Real (C.Height));
   begin
      for X of Change loop
         Next.Change.Append (X);
      end loop;
      Answers := [others => Not_Asked_Answer];
      C.Motion := (others => <>);
      C.Stood.Clear;
      C.Moved_On.Clear;
      --  The points by doubling: first the fewest a field stands on, spread over the picture (every Stride-th
      --  point, Stride the largest power of two that leaves that many), then twice as many each round, the new
      --  points predicted from the field so far where there is one; the rounds end when a round's field puts
      --  every point where the one before did, within what the two know, or when every point is aligned. The
      --  points left unasked are so answered. (R1 on the Pi: 349 points a keyframe, every one aligned, 3490
      --  alignments and 132 s for ten keyframes, for fields of twelve terms.)
      declare
         Asked  : Prediction_Array (1 .. Count) := Given;
         --  A point that rides with the eye tells nothing of how the view moved: it is not asked, and neither the
         --  common motion nor the keyframe's field rests on it (s30 on 10-17: the jaws' points drew both to no
         --  motion where the wrist had turned the view 8 to 88 degrees).
         Done   : Flag_Array (1 .. Count) :=
           [for K in 1 .. Count => Riding'Length > 0 and then Riding (Riding'First + K - 1)];
         Stride : Positive := 1;
      begin
         --  A prediction whose windows are wider than a patch at the full size (where several places may fit a patch
         --  alike), from a new direction or from the keyframes before: the view's common motion first, which the
         --  points must agree on, searched about where they stood as far as the prediction puts them and errs. A
         --  window a patch carries at the full size is searched there as given: there is nothing to tell apart. None
         --  told, a new direction asks no point, and a prediction from the keyframes is searched as it was given.
         --  On a servo arm whose every keyframe stands a few units off the reference in the joints not asked (its
         --  landings), the keyframes' predictions erred by more than their windows and the points slid along the
         --  table's grain out of them (f15w on 10-15, arm 2: keyframes 22-25 found 8, 6, 16 and 10 points of 255,
         --  keyframe 28, whose view moved 4.6 px, found 9 in a window stating 7 px; with the common motion first 126,
         --  190, 134, 88 and 169; arm 1, whose predictions stated a pixel or two, the same to keyframe 35 and 12 to
         --  35 more in four of the five after it).
         declare
            Sigmas   : Real_Array (1 .. Natural'Max (1, Count)) := [others => 0.0];
            Farthest : Real := 0.0;   --  the farthest the prediction puts a point from where it stood
            Typical  : Real := 0.0;   --  the median of what the prediction states of its points
         begin
            for K in 1 .. Count loop
               declare
                  G : Prediction renames Given (Given'First + K - 1);
               begin
                  Sigmas (K) := Sqrt (Real'Max (G.Cov.UU, G.Cov.VV));
                  Farthest := Real'Max (Farthest, Sqrt ((G.To.U - G.From.U) ** 2 + (G.To.V - G.From.V) ** 2));
               end;
            end loop;
            if Count > 0 then
               Typical := Driver.Stats.Median (Sigmas (1 .. Count));
            end if;
            if Count > 0 and then Window_Radius (Typical) + 1.0 > Real (Half_Patch (C.Width, C.Height)) then
               declare
                  Common : Motion;
                  --  The search is about where the points stood: its window reaches the farthest place the
                  --  prediction puts one, and the prediction's own window beyond it. No wider than the aligner searches:
                  --  a prediction wider than that says only "anywhere" (f15c replayed on 10-15: a new direction's
                  --  window, from a channel's shift measured far beyond the view's, was over 40,000 pixels across, and
                  --  the count of its places overflowed).
                  Spread : constant Real :=
                    (if C.Last = New_Direction then Sqrt (Given (Given'First).Cov.UU)
                     else Farthest / Window_Multiple + Typical);
               begin
                  declare
                     Stood, Moved : Flag_Array (1 .. Count);
                  begin
                     Common_Motion (C, Second, Real'Min (Spread, Largest_Sigma (C.Pictures, Second)), Riding, Asked, Common,
                                    Stood, Moved);
                     Keep_Evidence (C, Stood, Moved);
                  end;
                  Common.Searched := True;
                  C.Motion := Common;
                  if not Common.Told then
                     if C.Last = New_Direction then
                        Done := [others => True];
                     else
                        Asked := Given;
                     end if;
                  end if;
               end;
            elsif Count > 0 and then C.Last = Extrapolated then
               --  A prediction from the keyframes before whose windows a patch carries is checked against the
               --  view's common motion, searched as far as the prediction puts the points and errs: where the two
               --  disagree by more than the common motion states, the prediction's covariance there takes the
               --  rest of the disagreement, as the outer product of it. The curvature an extrapolation allows for
               --  is measured on keyframes aligned in the windows earlier extrapolations set, which hold no error
               --  wider than themselves (s29 replayed and judged against the simulator's truth, 10-17: keyframes
               --  extrapolated to 17 and 33 degrees of the wrist's roll stated 1.5 and 1.0 px and were found 5 and
               --  9 px off the true motion, along the table's grain).
               declare
                  Common : Motion;
                  Places : Prediction_Array (1 .. Count);
               begin
                  declare
                     Stood, Moved : Flag_Array (1 .. Count);
                  begin
                     Common_Motion (C, Second, Real'Min (Farthest / Window_Multiple + Typical, Largest_Sigma (C.Pictures, Second)),
                                 Riding, Places, Common,
                                    Stood, Moved);
                     Keep_Evidence (C, Stood, Moved);
                  end;
                  Common.Searched := True;
                  C.Motion := Common;
                  if Common.Told then
                     for K in 1 .. Count loop
                        declare
                           G     : Prediction renames Given (Given'First + K - 1);
                           Du    : constant Real := Places (K).To.U - G.To.U;
                           Dv    : constant Real := Places (K).To.V - G.To.V;
                           Apart : constant Real := Sqrt (Du ** 2 + Dv ** 2);
                           Rest  : constant Real := Real'Max (0.0, Apart - Sqrt (Places (K).Cov.UU));
                        begin
                           if Apart > 0.0 and then Rest > 0.0 then
                              Asked (K).Cov := (UU => G.Cov.UU + (Rest / Apart) ** 2 * Du ** 2,
                                                UV => G.Cov.UV + (Rest / Apart) ** 2 * Du * Dv,
                                                VV => G.Cov.VV + (Rest / Apart) ** 2 * Dv ** 2);
                           end if;
                        end;
                     end loop;
                  end if;
               end;
            end if;
         end;
         while Count / (2 * Stride) >= Least_Points loop
            Stride := 2 * Stride;
         end loop;
         loop
            declare
               Round : Flag_Array (1 .. Count) := [others => False];
               Any   : Boolean := False;
            begin
               for K in 1 .. Count loop
                  if not Done (K) and then (K - 1) mod Stride = 0 then
                     Round (K) := True;
                     Done (K) := True;
                     Any := True;
                     if Next.Field.Valid then
                        Asked (K) := Asked_From (C, Next.Field, C.Points (K));
                     end if;
                  end if;
               end loop;
               if Any then
                  --  A wider patch only while the keyframe has no field to stand on: with one, the next round
                  --  asks more points instead, at the usual cost.
                  Align_Some (C, Second, Asked, Round, Answers, Marks, Widen => not Next.Field.Valid);
                  declare
                     Refit   : Field_Fit;
                     Again   : Flag_Array (1 .. Count);
                     Settled : Boolean := False;
                  begin
                     Fit_Field (C, Marks, Refit, Again);
                     if Refit.Valid then
                        Settled := Next.Field.Valid and then Unmoved (C, Next.Field, Refit);
                        Next.Field := Refit;
                        Mine := Again;
                     end if;
                     exit when Settled;
                  end;
               end if;
               exit when Stride = 1;
               Stride := Stride / 2;
            end;
         end loop;
      end;
      Settle (C, Second, Answers, Marks, Next.Field, Mine);
      Agree := Mine;
      --  How far the extrapolation of each order that was tried was off, beyond what the fields it rested on carry:
      --  what the keyframe says of the curvature of the move (Frame). It is judged against the keyframe's field at
      --  every point that stays in the picture, not only at the points that were answered: a prediction off by more
      --  than the aligner could follow leaves no answer to judge it by. What the field itself does not know there
      --  (Variance) is not counted.
      if Next.Field.Valid then
         for Order in 1 .. Orders loop
            if C.Latest (Order).Valid and then C.Latest (Order).Dist > 0.0 then
               declare
                  Guess  : Candidate renames C.Latest (Order);
                  Excess : Real_Array (1 .. Count);
                  Signal : Real_Array (1 .. Count);
                  Judged : Natural := 0;
               begin
                  --  Per point, how far it was off beyond what the fields carry and this field knows, over the size
                  --  of its move (both per axis, as variances); a move the field's own uncertainty there does not
                  --  separate from none tells nothing. The keyframe's estimate is the median of them: a few points
                  --  the aligner could not follow do not set the window of the rest. How far it can be believed is
                  --  the signal the median point carries, the curvature's share of the squared error over what else
                  --  there is in it, squared.
                  for K in 1 .. Count loop
                     declare
                        P       : constant Driver.Images.Pixel := C.Points (K);
                        Move    : constant Displacement := Move_Of (C, P, Guess);
                        Size    : constant Real := (Move.Du ** 2 + Move.Dv ** 2) / 2.0;
                        Know    : constant Real := Variance (C, Next.Field, P);
                        Carried : constant Real := Carried_At (C, Guess, P) ** 2;
                     begin
                        if In_View (P) and then Size > Know then
                           Judged := Judged + 1;
                           Excess (Judged) :=
                             (((Evaluate (C, Next.Field.U, P) - Move.Du) ** 2
                               + (Evaluate (C, Next.Field.V, P) - Move.Dv) ** 2) / 2.0 - Carried - Know) / Size;
                           Signal (Judged) :=
                             Guess.Dist ** 2 * Size / Real'Max (Carried + Know + Next.Field.Sigma ** 2, Real'Epsilon * Size);
                        end if;
                     end;
                  end loop;
                  if Judged > 0 then
                     Next.Squared (Order) := Driver.Stats.Median (Excess (1 .. Judged)) / Guess.Dist ** 2;
                     Next.Weight (Order) := Driver.Stats.Median (Signal (1 .. Judged)) ** 2;
                  end if;
               end;
            end if;
         end loop;
         --  The same of the sum of single-channel moves, per unit of the squared sum of the amounts.
         if C.Combined.Valid and then C.Combined.Amount > 0.0 then
            declare
               Combo  : Combination renames C.Combined;
               Excess : Real_Array (1 .. Count);
               Signal : Real_Array (1 .. Count);
               Judged : Natural := 0;
            begin
               for K in 1 .. Count loop
                  declare
                     P       : constant Driver.Images.Pixel := C.Points (K);
                     Move    : constant Displacement := Move_By (C, Combo, P);
                     Size    : constant Real := (Move.Du ** 2 + Move.Dv ** 2) / 2.0;
                     Know    : constant Real := Variance (C, Next.Field, P);
                     Carried : constant Real := Carried_By (C, Combo, P);
                  begin
                     if In_View (P) and then Size > Know then
                        Judged := Judged + 1;
                        Excess (Judged) :=
                          (((Evaluate (C, Next.Field.U, P) - Move.Du) ** 2
                            + (Evaluate (C, Next.Field.V, P) - Move.Dv) ** 2) / 2.0 - Carried - Know) / Size;
                        Signal (Judged) :=
                          Combo.Amount ** 2 * Size / Real'Max (Carried + Know + Next.Field.Sigma ** 2, Real'Epsilon * Size);
                     end if;
                  end;
               end loop;
               if Judged > 0 then
                  Next.Squared_Sum := Driver.Stats.Median (Excess (1 .. Judged)) / Combo.Amount ** 2;
                  Next.Weight_Sum := Driver.Stats.Median (Signal (1 .. Judged)) ** 2;
               end if;
            end;
         end if;
      end if;
      Next.Moved := C.Last /= Same_Pose;
      C.Last := Supplied;
      C.Latest := [others => <>];
      C.Combined := (others => <>);
      C.Frames.Append (Next);
   end Add;

end Driver.Alignment.Keyframes;
