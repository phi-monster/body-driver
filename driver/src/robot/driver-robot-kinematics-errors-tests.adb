with Ada.Numerics.Long_Elementary_Functions;
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Fit;
with Driver.Tests;

package body Driver.Robot.Kinematics.Errors.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   function Gaussian (G : in out Generator) return Real is
      U1 : constant Real := Real'Max (Uniform (G), 1.0e-12);
      U2 : constant Real := Uniform (G);
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   ---------------------------------------------------------------------------
   --  A world of errors with a known structure

   --  Points on a picture of 640 x 480 pixels, seen in keyframes; the error of
   --  each sighting is the sum of
   --    a field over the picture that is the same in every keyframe (a point's
   --    own error, near points alike),
   --    a field of the keyframe's own over the picture,
   --    a shift every point of the keyframe has,
   --    and noise of its own,
   --  each with a spread in pixels (0: none); across and down are drawn alike
   --  and correlated by the Cross entries (the persistent field's and the
   --  shifts'); the fields' covariances fall as exp (-d / Range). Drop_Every:
   --  a point is missing from a keyframe now and then.
   type Structure is record
      Persistent_Sigma : Real := 0.0;
      Persistent_Range : Real := 90.0;
      Persistent_Cross : Real := 0.0;
      Keyframe_Sigma   : Real := 0.0;
      Keyframe_Range   : Real := 60.0;
      Shift_Sigma      : Real := 0.0;
      Shift_Cross      : Real := 0.0;
      Shift_Bias       : Real := 0.0;   --  what every keyframe's shift has in common, both ways
      Noise_Sigma      : Real := 0.05;
      Drop_Every       : Natural := 0;
   end record;

   --  Size is Points * Frames, Doubled twice that.
   type World (Points, Frames, Size, Doubled : Positive) is record
      Count        : Natural := 0;
      Frame, Track : Natural_Array (1 .. Size) := [others => 1];
      U0, V0       : Real_Array (1 .. Size) := [others => 0.0];
      Error        : Real_Array (1 .. Doubled) := [others => 0.0];
      --  What was drawn, to compare what is measured with: the mean squares
      --  of the persistent field and of the keyframes' shifts (both components
      --  together), the shifts' across with down, and the mean product of the
      --  shifts of two keyframes.
      Persistent_Square, Shift_Square, Shift_Cross, Shift_Between : Real := 0.0;
      --  and where the points are and the persistent field at them.
      Pixel_U, Pixel_V, Field_U, Field_V : Real_Array (1 .. Points) := [others => 0.0];
   end record;

   --  A correlated draw: the points' errors with covariance Variance exp (-d / Range_Px).
   procedure Field
     (G        : in out Generator;
      U, V     : Real_Array;
      Variance : Real;
      Range_Px : Real;
      Result   : out Real_Array)
   is
      N      : constant Natural := U'Length;
      Cov    : Real_Matrix (1 .. N, 1 .. N);
      Factor : Real_Matrix (1 .. N, 1 .. N);
      Z      : Real_Array (1 .. N);
      Ok     : Boolean;
   begin
      for A in 1 .. N loop
         for B in 1 .. N loop
            Cov (A, B) := Variance * Exp (-Sqrt ((U (A) - U (B)) ** 2 + (V (A) - V (B)) ** 2) / Range_Px)
              + (if A = B then 1.0e-9 else 0.0);
         end loop;
         Z (A) := Gaussian (G);
      end loop;
      Driver.Numerics.Dense.Cholesky (Cov, Factor, Ok);
      Check (Ok, "the covariance of a field of errors is not positive definite");
      for A in 1 .. N loop
         Result (A) := 0.0;
         for B in 1 .. A loop
            Result (A) := Result (A) + Factor (A, B) * Z (B);
         end loop;
      end loop;
   end Field;

   procedure Draw (W : in out World; G : in out Generator; S : Structure) is
      Pu, Pv : Real_Array (1 .. W.Points);
      Au, Av : Real_Array (1 .. W.Points);
      Ku, Kv : Real_Array (1 .. W.Points);
      Sum_U, Sum_V, Squares : Real := 0.0;
   begin
      for T in 1 .. W.Points loop
         Pu (T) := 640.0 * Uniform (G);
         Pv (T) := 480.0 * Uniform (G);
      end loop;
      Field (G, Pu, Pv, S.Persistent_Sigma ** 2, S.Persistent_Range, Au);
      Field (G, Pu, Pv, S.Persistent_Sigma ** 2, S.Persistent_Range, Av);
      for T in 1 .. W.Points loop
         --  Down correlated with across, of the same spread.
         Av (T) := S.Persistent_Cross * Au (T) + Sqrt (1.0 - S.Persistent_Cross ** 2) * Av (T);
         W.Persistent_Square := W.Persistent_Square + Au (T) ** 2 + Av (T) ** 2;
         W.Pixel_U (T) := Pu (T);
         W.Pixel_V (T) := Pv (T);
         W.Field_U (T) := Au (T);
         W.Field_V (T) := Av (T);
      end loop;
      W.Persistent_Square := W.Persistent_Square / Real (2 * W.Points);
      for F in 1 .. W.Frames loop
         declare
            Z_Across : constant Real := Gaussian (G);
            Shift_U  : constant Real := S.Shift_Bias + S.Shift_Sigma * Z_Across;
            Shift_V  : constant Real :=
              S.Shift_Bias
              + S.Shift_Sigma * (S.Shift_Cross * Z_Across + Sqrt (1.0 - S.Shift_Cross ** 2) * Gaussian (G));
         begin
            Field (G, Pu, Pv, S.Keyframe_Sigma ** 2, S.Keyframe_Range, Ku);
            Field (G, Pu, Pv, S.Keyframe_Sigma ** 2, S.Keyframe_Range, Kv);
            W.Shift_Square := W.Shift_Square + Shift_U ** 2 + Shift_V ** 2;
            W.Shift_Cross := W.Shift_Cross + Shift_U * Shift_V;
            Sum_U := Sum_U + Shift_U;
            Sum_V := Sum_V + Shift_V;
            Squares := Squares + Shift_U ** 2 + Shift_V ** 2;
            for T in 1 .. W.Points loop
               if S.Drop_Every = 0 or else (F + 3 * T) mod S.Drop_Every /= 0 then
                  W.Count := W.Count + 1;
                  W.Frame (W.Count) := F;
                  W.Track (W.Count) := T;
                  W.U0 (W.Count) := Pu (T);
                  W.V0 (W.Count) := Pv (T);
                  W.Error (2 * W.Count - 1) := Au (T) + Ku (T) + Shift_U + S.Noise_Sigma * Gaussian (G);
                  W.Error (2 * W.Count) := Av (T) + Kv (T) + Shift_V + S.Noise_Sigma * Gaussian (G);
               end if;
            end loop;
         end;
      end loop;
      W.Shift_Square := W.Shift_Square / Real (2 * W.Frames);
      W.Shift_Cross := W.Shift_Cross / Real (W.Frames);
      if W.Frames > 1 then
         W.Shift_Between := (Sum_U ** 2 + Sum_V ** 2 - Squares) / Real (2 * W.Frames * (W.Frames - 1));
      end if;
   end Draw;

   procedure Measure_World (W : World; M : out Model) is
   begin
      Measure (W.Frame (1 .. W.Count), W.Track (1 .. W.Count), W.U0 (1 .. W.Count), W.V0 (1 .. W.Count),
               W.Error (1 .. 2 * W.Count), M);
   end Measure_World;

   ---------------------------------------------------------------------------
   --  The isotonic fit

   procedure Falling_Pools_Violators is
      Fitted : Real_Array (1 .. 4);
   begin
      --  3 then 4 rises: pooled to their mean.
      Falling ([5.0, 3.0, 4.0, 1.0], [1.0, 1.0, 1.0, 1.0], Fitted);
      Check (Fitted = [5.0, 3.5, 3.5, 1.0], "a rise after a fall is not pooled to the mean of the two: "
             & Real'Image (Fitted (1)) & Real'Image (Fitted (2)) & Real'Image (Fitted (3)) & Real'Image (Fitted (4)));
      --  Weighted by how many pairs each mean is of.
      Falling ([2.0, 5.0, 1.0, 0.5], [3.0, 1.0, 1.0, 1.0], Fitted);
      Check_Close (Fitted (1), 2.75, 1.0e-12, "pooled mean of 3 pairs at 2 and 1 pair at 5");
      Check_Close (Fitted (2), 2.75, 1.0e-12, "pooled mean of 3 pairs at 2 and 1 pair at 5, second");
      Check_Close (Fitted (3), 1.0, 1.0e-12, "a mean already below its neighbours is moved");
      --  A fall is left as it is.
      Falling ([4.0, 3.0, 2.0, 1.0], [1.0, 2.0, 3.0, 4.0], Fitted);
      Check (Fitted = [4.0, 3.0, 2.0, 1.0], "a sequence that already falls is changed");
      --  A pooled block can reach back over several: 1 2 3 4 pool all the way.
      Falling ([1.0, 2.0, 3.0, 4.0], [1.0, 1.0, 1.0, 1.0], Fitted);
      Check (Fitted = [2.5, 2.5, 2.5, 2.5], "a sequence that only rises is not pooled to one level");
      --  Nothing is known where there are no pairs: the next level is taken.
      Falling ([4.0, 0.0, 2.0, 1.0], [1.0, 0.0, 1.0, 1.0], Fitted);
      Check (Fitted = [4.0, 2.0, 2.0, 1.0], "a bin with no pairs does not take the level of the next one that has: "
             & Real'Image (Fitted (1)) & Real'Image (Fitted (2)) & Real'Image (Fitted (3)) & Real'Image (Fitted (4)));
   end Falling_Pools_Violators;

   ---------------------------------------------------------------------------
   --  What is measured

   function Persistent_At (M : Model; D : Real) return Real is
     ((Persistent (M, D) (Both_Across) + Persistent (M, D) (Both_Down)) / 2.0);

   function Shared_At (M : Model; D : Real) return Real is
     ((Shared (M, D) (Both_Across) + Shared (M, D) (Both_Down)) / 2.0);

   --  The errors of a world drawn with a structure, measured: the covariance
   --  of one point's errors in two keyframes is the persistent field's, and so
   --  is that of two points at each distance (across and down together, and
   --  across with down, which the field gives the opposite sign: it rises to
   --  zero from below); near points of a keyframe err more alike than the
   --  same points in two; each function falls with the distance and is never
   --  below zero.
   procedure Measure_Reads_The_Structure is
      Gen : Generator := (State => 4_711);
      W   : World (Points => 70, Frames => 40, Size => 70 * 40, Doubled => 2 * 70 * 40);
      M   : Model;
      Total, Own_Level : Real;
   begin
      Draw (W, Gen, (Persistent_Sigma => 0.12, Persistent_Cross => -0.8, Keyframe_Sigma => 0.07,
                     Shift_Sigma => 0.06, Drop_Every => 9, others => <>));
      Measure_World (W, M);
      Check (Measured (M), "errors of 70 points in 40 keyframes are not measured");
      Total := 0.0;
      for I in 1 .. 2 * W.Count loop
         Total := Total + W.Error (I) ** 2;
      end loop;
      Total := Total / Real (2 * W.Count);
      Check_Close ((Alone (M) (Both_Across) + Alone (M) (Both_Down)) / 2.0, Total, 1.0e-9 * Total,
                   "a sighting's own covariance is the mean square of its errors");
      Own_Level := (Same_Track (M) (Both_Across) + Same_Track (M) (Both_Down)) / 2.0;
      Check_Close (Own_Level, W.Persistent_Square, 0.25 * W.Persistent_Square,
                   "the covariance of a point's errors in two keyframes is not that of the field they share");
      Check (Same_Track (M) (Across_Down) < -0.3 * W.Persistent_Square,
             "a point's errors across and down, in two keyframes, are not opposite:"
             & Real'Image (Same_Track (M) (Across_Down)));
      --  Points near each other err alike in every keyframe, far apart not
      --  (the field they share, as it was drawn, averaged over the pairs of
      --  points at those distances: the model's mean over the same pairs).
      declare
         Edges : constant Real_Array := [0.0, 60.0, 120.0, 240.0, 480.0, 1_000.0];
      begin
         for E in 1 .. Edges'Length - 1 loop
            declare
               Drawn, Found, Drawn_Cross, Found_Cross : Real := 0.0;
               Pairs                                  : Natural := 0;
            begin
               for A in 1 .. W.Points - 1 loop
                  for B in A + 1 .. W.Points loop
                     declare
                        D : constant Real := Sqrt ((W.Pixel_U (A) - W.Pixel_U (B)) ** 2
                                                   + (W.Pixel_V (A) - W.Pixel_V (B)) ** 2);
                     begin
                        if D >= Edges (E) and then D < Edges (E + 1) then
                           Pairs := Pairs + 1;
                           Drawn := Drawn + 0.5 * (W.Field_U (A) * W.Field_U (B) + W.Field_V (A) * W.Field_V (B));
                           Found := Found + Persistent_At (M, D);
                           Drawn_Cross := Drawn_Cross
                             + 0.5 * (W.Field_U (A) * W.Field_V (B) + W.Field_V (A) * W.Field_U (B));
                           Found_Cross := Found_Cross + Persistent (M, D) (Across_Down);
                        end if;
                     end;
                  end loop;
               end loop;
               if Pairs > 0 then
                  Check_Close (Found / Real (Pairs), Drawn / Real (Pairs), 0.25 * W.Persistent_Square,
                               "the covariance of points" & Edges (E)'Image & " to" & Edges (E + 1)'Image
                               & " pixels apart, in two keyframes, is not that of the field they share");
                  Check_Close (Found_Cross / Real (Pairs), Drawn_Cross / Real (Pairs), 0.25 * W.Persistent_Square,
                               "the covariance across with down of points" & Edges (E)'Image & " to"
                               & Edges (E + 1)'Image & " pixels apart, in two keyframes, is not that of the field");
               end if;
            end;
         end loop;
      end;
      --  Within a keyframe the nearby points err more alike still: by the
      --  keyframe's field and its shift.
      Check (Shared_At (M, 40.0) > Persistent_At (M, 40.0) + 0.5 * W.Shift_Square,
             "points of one keyframe, 40 pixels apart, err no more alike than points of two");
      Check (Added_By_Keyframe (M) (Both_Across) > 0.0 and then Added_By_Keyframe (M) (Both_Down) > 0.0,
             "a keyframe adds nothing to the covariance of its points");
      --  Each variance falls with the distance and is never below zero.
      declare
         Last_P, Last_S : Entries := [others => Real'Last];
      begin
         for Step in 0 .. 150 loop
            declare
               D : constant Real := 5.0 * Real (Step);
               P : constant Entries := Persistent (M, D);
               S : constant Entries := Shared (M, D);
            begin
               Check (P (Both_Across) <= Last_P (Both_Across) and then P (Both_Down) <= Last_P (Both_Down)
                      and then S (Both_Across) <= Last_S (Both_Across) and then S (Both_Down) <= Last_S (Both_Down),
                      "a covariance rises with the distance, at" & Real'Image (D));
               Check (P (Both_Across) >= 0.0 and then P (Both_Down) >= 0.0 and then S (Both_Across) >= 0.0
                      and then S (Both_Down) >= 0.0, "a variance is below zero, at" & Real'Image (D));
               Last_P := P;
               Last_S := S;
            end;
         end loop;
      end;
      --  Where the persistent errors have fallen to half is read from the function (the field's own range is
      --  90 pixels: exp (-d / 90) is half at 62).
      Check (Half_Distance_Persistent (M) > 0.0 and then Half_Distance_Persistent (M) < 300.0,
             "the persistent errors fall to half at" & Real'Image (Half_Distance_Persistent (M)) & " pixels");
      Free (M);
   end Measure_Reads_The_Structure;

   --  A world of nothing but a shift every point of a keyframe has, and
   --  noise: no point errs alike in two keyframes, and what a keyframe adds is
   --  the shifts' mean square at every distance, across with down included
   --  (the shifts across and down are opposed).
   procedure Measure_Reads_Shifts_Alone is
      Gen : Generator := (State => 2_024);
      W   : World (Points => 40, Frames => 40, Size => 40 * 40, Doubled => 2 * 40 * 40);
      M   : Model;
      Own_Level : Real;
   begin
      Draw (W, Gen, (Shift_Sigma => 0.06, Shift_Cross => -0.7, others => <>));
      Measure_World (W, M);
      Check (Measured (M), "errors of 40 points in 40 keyframes are not measured");
      Own_Level := (Same_Track (M) (Both_Across) + Same_Track (M) (Both_Down)) / 2.0;
      Check (abs Own_Level < 0.15 * W.Shift_Square,
             "points that err alike in no two keyframes do:" & Real'Image (Own_Level));
      for Step in 0 .. 15 loop
         Check (Persistent_At (M, 50.0 * Real (Step)) < 0.15 * W.Shift_Square,
                "points that err alike in no two keyframes do, at" & Real'Image (50.0 * Real (Step)) & " pixels");
      end loop;
      Check_Close ((Added_By_Keyframe (M) (Both_Across) + Added_By_Keyframe (M) (Both_Down)) / 2.0, W.Shift_Square,
                   0.3 * W.Shift_Square, "what a keyframe adds is not the shift every point of it has");
      Check_Close (Added_By_Keyframe (M) (Across_Down), W.Shift_Cross, 0.3 * W.Shift_Square,
                   "what a keyframe adds across with down is not the shifts' across with down:"
                   & Real'Image (Added_By_Keyframe (M) (Across_Down)) & " against" & Real'Image (W.Shift_Cross));
      Free (M);
   end Measure_Reads_Shifts_Alone;

   --  Few keyframes, a shift each (about a common one) and nothing else: a
   --  keyframe's pairs of points are a sixth of all the pairs of two points'
   --  sightings, and are not to be counted among those of two keyframes, whose
   --  covariance is the shifts' mean product between two keyframes, which is
   --  not their mean square (nor the square of their mean).
   procedure Measure_Keeps_Keyframes_Apart is
      Gen : Generator := (State => 77);
      W   : World (Points => 50, Frames => 6, Size => 50 * 6, Doubled => 2 * 50 * 6);
      M   : Model;
   begin
      Draw (W, Gen, (Shift_Sigma => 0.08, Shift_Bias => 0.1, Noise_Sigma => 0.005, others => <>));
      Measure_World (W, M);
      Check (Measured (M), "errors of 50 points in 6 keyframes are not measured");
      for Step in 0 .. 12 loop
         Check_Close (Persistent_At (M, 50.0 * Real (Step)), Real'Max (0.0, W.Shift_Between),
                      0.02 * W.Shift_Square,
                      "the covariance of points in two keyframes, at" & Real'Image (50.0 * Real (Step))
                      & " pixels, is not the shifts' between two keyframes:" & Real'Image (W.Shift_Between)
                      & " against a mean square of" & Real'Image (W.Shift_Square));
      end loop;
      Free (M);
   end Measure_Keeps_Keyframes_Apart;

   procedure Nothing_Is_Measured_Without_Pairs is
      Gen : Generator := (State => 99);
      W   : World (Points => 12, Frames => 1, Size => 12, Doubled => 24);
      M   : Model;
      Rows : constant Real_Matrix (1 .. 24, 1 .. 3) := [others => [others => 1.0]];
      Meat_Of : Real_Matrix (1 .. 3, 1 .. 3);
   begin
      --  One keyframe: nothing tells a point's error in one keyframe from another's.
      Draw (W, Gen, (Persistent_Sigma => 0.1, Persistent_Range => 80.0, Keyframe_Sigma => 0.05,
                     Keyframe_Range => 50.0, Shift_Sigma => 0.04, others => <>));
      Measure_World (W, M);
      Check (not Measured (M), "the errors of one keyframe are measured");
      Meat (M, W.Frame (1 .. W.Count), W.Track (1 .. W.Count), W.U0 (1 .. W.Count), W.V0 (1 .. W.Count), Rows,
            Meat_Of);
      Check (Meat_Of = [1 .. 3 => [1 .. 3 => 0.0]], "an unmeasured model has a spread");
      Free (M);
      --  Every point seen once.
      declare
         Frame : constant Natural_Array := [1, 2, 3, 4];
         Track : constant Natural_Array := [1, 2, 3, 4];
         U0    : constant Real_Array := [10.0, 200.0, 300.0, 400.0];
         V0    : constant Real_Array := [10.0, 20.0, 30.0, 40.0];
         Err   : constant Real_Array := [0.1, -0.1, 0.2, 0.0, -0.2, 0.1, 0.05, 0.0];
      begin
         Measure (Frame, Track, U0, V0, Err, M);
         Check (not Measured (M), "the errors of points that were each seen once are measured");
         Free (M);
      end;
      --  One point.
      declare
         Frame : constant Natural_Array := [1, 2, 3];
         Track : constant Natural_Array := [1, 1, 1];
         U0    : constant Real_Array := [10.0, 10.0, 10.0];
         V0    : constant Real_Array := [10.0, 10.0, 10.0];
         Err   : constant Real_Array := [0.1, -0.1, 0.2, 0.0, -0.2, 0.1];
      begin
         Measure (Frame, Track, U0, V0, Err, M);
         Check (not Measured (M), "the errors of one point are measured");
         Free (M);
      end;
   end Nothing_Is_Measured_Without_Pairs;

   ---------------------------------------------------------------------------
   --  The middle of the sandwich

   --  Meat against the sum over every pair of sightings and of their two
   --  errors of the covariance times the outer product of the gradients, term
   --  by term: the grouping that keeps it cheap (a point's rows summed, a
   --  keyframe's far level as one sum, the near part pair by pair) must not
   --  change what is summed.
   procedure Meat_Is_The_Sum_Over_Pairs is
      Gen    : Generator := (State => 31_337);
      W      : World (Points => 16, Frames => 7, Size => 16 * 7, Doubled => 2 * 16 * 7);
      M      : Model;
      P      : constant := 4;
   begin
      Draw (W, Gen, (Persistent_Sigma => 0.10, Persistent_Range => 80.0, Persistent_Cross => 0.3,
                     Keyframe_Sigma => 0.07, Keyframe_Range => 50.0, Shift_Sigma => 0.05, Shift_Cross => 0.5,
                     Shift_Bias => 0.02, Noise_Sigma => 0.06, Drop_Every => 5));
      declare
         Rows    : Real_Matrix (1 .. 2 * W.Count, 1 .. P);
         Fast    : Real_Matrix (1 .. P, 1 .. P);
         Sum     : Real_Matrix (1 .. P, 1 .. P) := [others => [others => 0.0]];
         Biggest : Real := 0.0;
      begin
         for I in 1 .. 2 * W.Count loop
            for Q in 1 .. P loop
               Rows (I, Q) := Gaussian (Gen);
            end loop;
         end loop;
         Measure_World (W, M);
         Check (Measured (M), "the world of the pair sum is not measured");
         Meat (M, W.Frame (1 .. W.Count), W.Track (1 .. W.Count), W.U0 (1 .. W.Count), W.V0 (1 .. W.Count),
               Rows, Fast);
         for I in 1 .. W.Count loop
            for J in 1 .. W.Count loop
               declare
                  C : constant Entries :=
                    (if I = J then Alone (M)
                     elsif W.Track (I) = W.Track (J) then Same_Track (M)
                     elsif W.Frame (I) = W.Frame (J)
                     then Shared (M, Sqrt ((W.U0 (I) - W.U0 (J)) ** 2 + (W.V0 (I) - W.V0 (J)) ** 2))
                     else Persistent (M, Sqrt ((W.U0 (I) - W.U0 (J)) ** 2 + (W.V0 (I) - W.V0 (J)) ** 2)));
               begin
                  for A in 1 .. 2 loop
                     for B in 1 .. 2 loop
                        declare
                           Coefficient : constant Real :=
                             (if A = 1 and then B = 1 then C (Both_Across)
                              elsif A = 2 and then B = 2 then C (Both_Down) else C (Across_Down));
                        begin
                           for Q in 1 .. P loop
                              for R in 1 .. P loop
                                 Sum (Q, R) := Sum (Q, R)
                                   + Coefficient * Rows (2 * I - 2 + A, Q) * Rows (2 * J - 2 + B, R);
                              end loop;
                           end loop;
                        end;
                     end loop;
                  end loop;
               end;
            end loop;
         end loop;
         for Q in 1 .. P loop
            for R in 1 .. P loop
               Biggest := Real'Max (Biggest, abs Sum (Q, R));
            end loop;
         end loop;
         Check (Biggest > 0.0, "the pair sum is zero");
         for Q in 1 .. P loop
            for R in 1 .. P loop
               Check_Close (Fast (Q, R), Sum (Q, R), 1.0e-9 * Biggest,
                            "the middle of the sandwich differs from the sum over pairs at" & Q'Image & ","
                            & R'Image);
            end loop;
         end loop;
      end;
      Free (M);
   end Meat_Is_The_Sum_Over_Pairs;

   --  The covariance of a sandwich is positive semi-definite: the negative
   --  eigenvalues of a spread that is not are set to zero where the inverse
   --  normal equations are the identity, and what that takes is reported.
   procedure Sandwich_Is_Clipped is
      package Fit renames Driver.Robot.Kinematics.Fit;
      Covariance : Real_Matrix (1 .. 2, 1 .. 2);
      Clipped    : Real;
      Ok         : Boolean;
   begin
      Fit.Sandwich ([[1.0, 0.0], [0.0, 1.0]], [[2.0, 0.0], [0.0, -1.0]], Covariance, Clipped, Ok);
      Check (Ok, "the sandwich of the identity is refused");
      Check_Close (Covariance (1, 1), 2.0, 1.0e-12, "the positive part of the spread is changed");
      Check_Close (Covariance (2, 2), 0.0, 1.0e-12, "the negative part of the spread is kept");
      Check_Close (Clipped, 1.0 / 3.0, 1.0e-12, "the share taken by the clip is not a third");
      --  A spread that is positive semi-definite is left as it is, in any units of the terms.
      Fit.Sandwich ([[100.0, 0.0], [0.0, 0.01]], [[0.01, 0.0], [0.0, 4.0]], Covariance, Clipped, Ok);
      Check (Ok, "a sandwich of well-conditioned equations is refused");
      Check_Close (Clipped, 0.0, 0.0, "the clip takes from a spread that is positive semi-definite");
      Check_Close (Covariance (1, 1), 100.0 * 0.01 * 100.0, 1.0e-9, "the sandwich of a diagonal spread, first");
      Check_Close (Covariance (2, 2), 0.01 * 4.0 * 0.01, 1.0e-12, "the sandwich of a diagonal spread, second");
      --  Equations that are not positive definite give no covariance.
      Fit.Sandwich ([[1.0, 2.0], [2.0, 1.0]], [[1.0, 0.0], [0.0, 1.0]], Covariance, Clipped, Ok);
      Check (not Ok, "a sandwich of equations that are not positive definite is accepted");
   end Sandwich_Is_Clipped;

   procedure Register is
   begin
      Driver.Tests.Register ("robot.errors.falling", "a rise in the means by distance is kept instead of pooled, or a "
                             & "bin without pairs is given a level of its own", Falling_Pools_Violators'Access);
      Driver.Tests.Register ("robot.errors.measure", "the covariance of errors read from residuals does not fall with "
                             & "the distance, goes below zero, or misses the persistent field's or the keyframes' "
                             & "share", Measure_Reads_The_Structure'Access);
      Driver.Tests.Register ("robot.errors.shifts", "errors that are nothing but a shift each keyframe gives all its "
                             & "points are read as errors a point has in every keyframe, or as none a keyframe "
                             & "adds", Measure_Reads_Shifts_Alone'Access);
      Driver.Tests.Register ("robot.errors.apart", "the pairs of one keyframe's points are counted among the pairs "
                             & "of two keyframes, so that a shift of each keyframe reads as an error a point has in "
                             & "every keyframe", Measure_Keeps_Keyframes_Apart'Access);
      Driver.Tests.Register ("robot.errors.unpaired", "errors of one keyframe, of points seen once or of one point "
                             & "are taken to tell how errors depend on each other",
                             Nothing_Is_Measured_Without_Pairs'Access);
      Driver.Tests.Register ("robot.errors.meat", "the middle of the sandwich is not the sum over every pair of "
                             & "sightings of their covariance times their gradients",
                             Meat_Is_The_Sum_Over_Pairs'Access);
      Driver.Tests.Register ("robot.errors.clip", "a spread that is not positive semi-definite gives a covariance "
                             & "that is not, or a clip that depends on the units of the terms",
                             Sandwich_Is_Clipped'Access);
   end Register;

end Driver.Robot.Kinematics.Errors.Tests;
