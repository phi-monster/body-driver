with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Uncertain;

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

   package Located_Vectors is new Ada.Containers.Vectors (Positive, Located);

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
   procedure Robust_Start (C : Chain; Placed : Located_Vectors.Vector; Keep : in out Flag_Array) is
      Count : constant Natural := Natural (Placed.Length);
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

   --  The displacements of the found points as a smooth field, fitted to the points that lie within Z robust
   --  scales of it, until the points that do are the same, starting from those a robust affine field agrees with
   --  (Robust_Start). Agree says which points the final field agrees with; none when it has no field.
   procedure Fit_Field
     (C : Chain; Placed : Located_Vectors.Vector; Result : out Field_Fit; Agree : out Flag_Array)
   is
      Count  : constant Natural := Natural (Placed.Length);
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
               Xu, Xv : Real_Vector (1 .. Terms);
               Full_U, Full_V : Boolean;
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
               Driver.Numerics.Dense.Least_Squares (A, Bu, Xu, Full_U);
               Driver.Numerics.Dense.Least_Squares (A, Bv, Xv, Full_V);
               exit when not (Full_U and then Full_V);
               for J in 1 .. Terms loop
                  Result.U (J) := Xu (J);
                  Result.V (J) := Xv (J);
               end loop;
               Result.Valid := True;
            end;
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

   --  The standard deviation of what the fields carry into a prediction: their scatter and their off-direction
   --  parts, through the weights.
   function Carried (C : Chain; Off : Real_Array; First, Second : Natural; W1, W2 : Real) return Real is
      One   : constant Real := (W1 * C.Frames (First).Field.Sigma) ** 2 + (W1 * Off (First)) ** 2;
      Other : constant Real :=
        (if Second = 0 then 0.0 else (W2 * C.Frames (Second).Field.Sigma) ** 2 + (W2 * Off (Second)) ** 2);
   begin
      return Sqrt (One + Other);
   end Carried;

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
   function Stated_Sigma (Guess : Candidate; Move : Displacement) return Real is
     (Sqrt (Guess.Carry ** 2 + (Guess.Relative ** 2) * (Move.Du ** 2 + Move.Dv ** 2)));

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
            Sigma : constant Real := Stated_Sigma (Guess, Move);
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
   begin
      for F of C.Frames loop
         if F.Moved and then F.Field.Valid then
            Count := Count + 1;
            Sigmas (Count) := F.Field.Sigma;
         end if;
      end loop;
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
         Amounts : Real_Array (1 .. Frames) := [others => 0.0];
         Off     : Real_Array (1 .. Frames) := [others => 0.0];
         Usable  : Flag_Array (1 .. Frames) := [others => False];
         Sigmas  : Real_Array (1 .. Count);
         Curve   : Order_Errors := [others => 0.0];   --  per order, the largest curvature the keyframes along it showed
         Best    : Real := Reach;
         Chosen  : Natural := 0;   --  the order of the candidate that states the least, when one states less than Best
         Offer_Wins : Boolean := False;

         --  The prediction from the earlier keyframes First and Second (unless zero), kept if it states the least
         --  of its order.
         procedure Consider (First, Second : Natural) is
            Order    : constant Positive := Order_Of (Second);
            Dist     : constant Real :=
              abs (Own - Amounts (First)) * (if Second = 0 then 1.0 else abs (Own - Amounts (Second)));
            Guess    : Candidate :=
              (Valid => True, First => First, Second => Second, Curve => Curve (Order), Dist => Dist,
               Relative => Curve (Order) * Dist, others => <>);
         begin
            Weights (Amounts, First, Second, Own, Guess.W1, Guess.W2);
            Guess.Carry := Carried (C, Off, First, Second, Guess.W1, Guess.W2);
            for K in Sigmas'Range loop
               Sigmas (K) := Stated_Sigma (Guess, Move_Of (C, C.Points (K), Guess));
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
               for Order in 1 .. Orders loop
                  Curve (Order) := Real'Max (Curve (Order), C.Frames (J).Curvature (Order));
               end loop;
            end if;
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
         for Order in 1 .. Orders loop
            if C.Latest (Order).Valid and then C.Latest (Order).Median < Best then
               Best := C.Latest (Order).Median;
               Chosen := Order;
               Offer_Wins := False;
            end if;
         end loop;
         if Offer_Wins then
            From := Supplied;
            Into := Offered;
         elsif Chosen = 0 then
            From := New_Direction;
            Stay (Reach);
         else
            From := Extrapolated;
            Extrapolate (C, C.Latest (Chosen), Into);
         end if;
         C.Last := From;
      end;
   end Predict;

   procedure Agreement (C : Chain; Answers : Answer_Array; Agree : out Flag_Array) is
      Placed : Located_Vectors.Vector;
      Field  : Field_Fit;
   begin
      for K in Answers'Range loop
         Placed.Append
           ((if Answers (K).Verdict = Found
             then Located'(Found => True, Place => Answers (K).To,
                           Sigma => Sqrt (Real'Max (Answers (K).Cov.UU, Answers (K).Cov.VV)))
             else Located'(others => <>)));
      end loop;
      Fit_Field (C, Placed, Field, Agree);
   end Agreement;

   procedure Realign
     (C       : Chain;
      Image   : Driver.Images.Image;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array)
   is
      Second : constant Pyramid := Pyramid_Of (Image);
      Placed : Located_Vectors.Vector;
      Field  : Field_Fit;
   begin
      for K in Given'Range loop
         declare
            Result : Answer renames Answers (Answers'First + K - Given'First);
            Place  : Located;
         begin
            Result := Align (C.Pictures, Second, Given (K));
            if Result.Verdict = Found then
               Place := (Found => True, Place => Result.To, Sigma => Sqrt (Real'Max (Result.Cov.UU, Result.Cov.VV)));
            end if;
            Placed.Append (Place);
         end;
      end loop;
      Fit_Field (C, Placed, Field, Agree);
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
      Placed : Located_Vectors.Vector;
      Next   : Frame;
   begin
      for X of Change loop
         Next.Change.Append (X);
      end loop;
      for K in Given'Range loop
         declare
            Result : Answer renames Answers (Answers'First + K - Given'First);
            Place  : Located;
         begin
            Result := Align (C.Pictures, Second, Given (K));
            if Result.Verdict = Found then
               Place := (Found => True, Place => Result.To, Sigma => Sqrt (Real'Max (Result.Cov.UU, Result.Cov.VV)));
            end if;
            Placed.Append (Place);
         end;
      end loop;
      Fit_Field (C, Placed, Next.Field, Agree);
      --  How far the extrapolation of each order that was tried was off, beyond what it stated: the relative error
      --  its next use must allow for.
      for Order in 1 .. Orders loop
         if C.Latest (Order).Valid then
            declare
               Guess         : Candidate renames C.Latest (Order);
               Excess, Shown : Real := 0.0;
            begin
               for K in 1 .. Natural (C.Points.Length) loop
                  if Placed (K).Found and then Agree (Agree'First + K - 1) then
                     declare
                        P    : constant Driver.Images.Pixel := C.Points (K);
                        Move : constant Displacement := Move_Of (C, P, Guess);
                     begin
                        Excess := Excess
                          + ((Placed (K).Place.U - P.U - Move.Du) ** 2
                             + (Placed (K).Place.V - P.V - Move.Dv) ** 2) / 2.0
                          - Stated_Sigma (Guess, Move) ** 2 - Placed (K).Sigma ** 2;
                        Shown := Shown + (Move.Du ** 2 + Move.Dv ** 2) / 2.0;
                     end;
                  end if;
               end loop;
               Next.Curvature (Order) :=
                 (if Shown > 0.0 and then Guess.Dist > 0.0
                  then Sqrt (Guess.Curve ** 2 + Real'Max (0.0, Excess) / (Shown * Guess.Dist ** 2))
                  else Guess.Curve);
            end;
         end if;
      end loop;
      Next.Moved := C.Last /= Same_Pose;
      C.Last := Supplied;
      C.Latest := [others => <>];
      C.Frames.Append (Next);
   end Add;

end Driver.Alignment.Keyframes;
