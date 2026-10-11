with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
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

   --  How far along the direction of Change an earlier keyframe lies (signed, in the units of the readings), and
   --  how far at most the part of its change off that direction can have moved the view: the sizes of that part's
   --  entries times the pixels a unit of each channel moves the view.
   procedure Along (Before : Frame; Change, Shift : Real_Array; Amount, Off : out Real) is
      Past : constant Real_Array := To_Array (Before.Change);
      Own  : constant Real := Dot (Change, Change);
      Unit : constant Real := Dot (Past, Change) / Own;
   begin
      Amount := Unit * Sqrt (Own);
      Off := 0.0;
      for K in Change'Range loop
         Off := Off + abs (Past (K) - Unit * Change (K)) * Shift (K);
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
         Reach   : constant Real := (if Moved > 0.0 then Moved else Largest_Sigma (C.Pictures, C.Pictures));
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
            Along (C.Frames (J), Changes, Shifts, Amounts (J), Off (J));
            --  A keyframe that tells nothing of the direction (the reference's twin, or one at right angles), or
            --  whose off-direction part alone carries more into the prediction than not having moved at all,
            --  gives no prediction that could state less.
            Usable (J) := C.Frames (J).Field.Valid and then Amounts (J) /= 0.0
              and then not Is_None (To_Array (C.Frames (J).Change), Noises)
              and then Own * Off (J) < Reach * abs Amounts (J);
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
         if Frames > 0 and then Channels_Moving (Changes, Shifts, Precision) >= 2 then
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
         for Order in 1 .. Orders loop
            if C.Latest (Order).Valid and then C.Latest (Order).Median < Best then
               Best := C.Latest (Order).Median;
               Chosen := Order;
               Offer_Wins := False;
            end if;
         end loop;
         if C.Combined.Valid and then C.Combined.Median < Best then
            Best := C.Combined.Median;
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

   --  The points of the reference that Only marks, into the second picture from their predictions, in as many pieces
   --  at once as the machine has processors (a point's alignment reads the two pyramids and writes only its own
   --  answer and mark); where an answer was found, its mark has the place it says, as the field of displacements is
   --  fitted to the marks.
   procedure Align_Some
     (C       : Chain;
      Second  : Pyramid;
      Given   : Prediction_Array;
      Only    : Flag_Array;
      Answers : in out Answer_Array;
      Marks   : in out Located_Array)
   is
      Count  : constant Natural := Given'Length;
      Pieces : constant Natural := Natural'Max (1, Natural'Min (Count, Natural (System.Multiprocessors.Number_Of_CPUs)));

      procedure Piece (Index : Positive) is
         First : constant Positive := (Index - 1) * Count / Pieces + 1;
         Last  : constant Natural := Index * Count / Pieces;
      begin
         for K in First .. Last loop
            if Only (Only'First + K - 1) then
               declare
                  Result : Answer renames Answers (Answers'First + K - 1);
               begin
                  Result := Align (C.Pictures, Second, Given (Given'First + K - 1));
                  Marks (K) :=
                    (if Result.Verdict = Found
                     then (Found => True, Place => Result.To, Sigma => Sqrt (Real'Max (Result.Cov.UU, Result.Cov.VV)))
                     else (others => <>));
               end;
            end if;
         end loop;
      end Piece;

      procedure Run is new Driver.Parallel_For (Piece);
   begin
      Run (1, Pieces);
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
   --  one another have made: where it puts them, to the uncertainty it has there. The field is then fitted to all
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
               if not Agree (K) and then (Prediction_Caused (Seen) or else Seen.Verdict = Found) then
                  Todo (K) := True;
                  Given (K) := Asked_From (C, Field, C.Points (K));
               end if;
            end;
         end loop;
         exit when Agreeing_Of (Todo) = 0;
         Align_Some (C, Second, Given, Todo, Answers, Marks);
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

   procedure Add
     (C       : in out Chain;
      Image   : Driver.Images.Image;
      Change  : Driver.Real_Array;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array)
   is
      Second : constant Pyramid := Pyramid_Of (Image);
      Count  : constant Natural := Given'Length;
      Marks  : Located_Array (1 .. Count);
      Mine   : Flag_Array (1 .. Count);
      Next   : Frame;

      --  Whether the field puts the point inside the picture: where the prediction was of any use.
      function In_View (P : Driver.Images.Pixel) return Boolean is
        (P.U + Evaluate (C, Next.Field.U, P) in 0.0 .. Real (C.Width)
         and then P.V + Evaluate (C, Next.Field.V, P) in 0.0 .. Real (C.Height));
   begin
      for X of Change loop
         Next.Change.Append (X);
      end loop;
      Align_Some (C, Second, Given, [1 .. Count => True], Answers, Marks);
      Fit_Field (C, Marks, Next.Field, Mine);
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
