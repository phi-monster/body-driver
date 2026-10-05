with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;

package body Driver.Robot.Kinematics.Errors is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   type Real_Access is access Real_Array;
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);

   --  The rows of a table of sums by distance: the three products of a pair's
   --  errors (Pair_Kind), then how many pairs they are of.
   Count_Row : constant := Pair_Kind'Pos (Pair_Kind'Last) + 2;

   function Row_Of (Kind : Pair_Kind) return Positive is (Pair_Kind'Pos (Kind) + 1);

   function Distance (Ua, Va, Ub, Vb : Real) return Real is (Sqrt ((Ua - Ub) ** 2 + (Va - Vb) ** 2));

   ---------------------------------------------------------------------------
   --  Grouping

   --  The positions 1 .. Key'Length by their key, 1 .. Keys: those with key K
   --  are Order (Start (K) .. Start (K + 1) - 1).
   procedure Group_By (Key : Natural_Array; Keys : Natural; Start, Order : out Natural_Access) is
      Count : Natural_Access := new Natural_Array'(1 .. Keys => 0);
      Next  : Natural_Access := new Natural_Array'(1 .. Keys => 0);
   begin
      Start := new Natural_Array'(1 .. Keys + 1 => 0);
      Order := new Natural_Array'(1 .. Key'Length => 0);
      for K of Key loop
         Count (K) := Count (K) + 1;
      end loop;
      Start (1) := 1;
      for K in 1 .. Keys loop
         Start (K + 1) := Start (K) + Count (K);
         Next (K) := Start (K);
      end loop;
      for I in Key'Range loop
         Order (Next (Key (I))) := I;
         Next (Key (I)) := Next (Key (I)) + 1;
      end loop;
      Free (Next);
      Free (Count);
   end Group_By;

   ---------------------------------------------------------------------------
   --  Isotonic regression

   procedure Falling (Mean, Weight : Real_Array; Fitted : out Real_Array) is
      N      : constant Natural := Mean'Length;
      Level  : Real_Access := new Real_Array'(1 .. N => 0.0);
      Mass   : Real_Access := new Real_Array'(1 .. N => 0.0);
      Last   : Natural_Access := new Natural_Array'(1 .. N => 0);
      Blocks : Natural := 0;
      First  : Positive := 1;
   begin
      for I in 1 .. N loop
         if Weight (Weight'First + I - 1) > 0.0 then
            Blocks := Blocks + 1;
            Level (Blocks) := Mean (Mean'First + I - 1);
            Mass (Blocks) := Weight (Weight'First + I - 1);
            Last (Blocks) := I;
            while Blocks > 1 and then Level (Blocks - 1) < Level (Blocks) loop
               Level (Blocks - 1) := (Level (Blocks - 1) * Mass (Blocks - 1) + Level (Blocks) * Mass (Blocks))
                 / (Mass (Blocks - 1) + Mass (Blocks));
               Mass (Blocks - 1) := Mass (Blocks - 1) + Mass (Blocks);
               Last (Blocks - 1) := Last (Blocks);
               Blocks := Blocks - 1;
            end loop;
         end if;
      end loop;
      Fitted := [others => 0.0];
      for B in 1 .. Blocks loop
         for I in First .. (if B = Blocks then N else Last (B)) loop
            Fitted (Fitted'First + I - 1) := Level (B);
         end loop;
         First := Last (B) + 1;
      end loop;
      Free (Level);
      Free (Mass);
      Free (Last);
   end Falling;

   function Squares (Mean, Weight, Fitted : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for I in Mean'Range loop
         Sum := Sum + Weight (I) * (Mean (I) - Fitted (I)) ** 2;
      end loop;
      return Sum;
   end Squares;

   --  What a table of sums by distance (the products and their number) says
   --  of the covariance by distance: the mean of each product, made to fall
   --  with the distance and, for an error with itself, never below zero; the
   --  product of across with down may be of either sign and falls to zero
   --  from either side, the side that fits better.
   function Covariance_By_Distance (Sums : Real_Matrix) return Matrix_Access is
      Bins   : constant Natural := Sums'Length (2) - 1;
      Result : constant Matrix_Access := new Real_Matrix'[1 .. Count_Row => [0 .. Bins => 0.0]];
      Mean   : Real_Access := new Real_Array'(1 .. Bins + 1 => 0.0);
      Weight : Real_Access := new Real_Array'(1 .. Bins + 1 => 0.0);
      Down   : Real_Access := new Real_Array'(1 .. Bins + 1 => 0.0);
      Up     : Real_Access := new Real_Array'(1 .. Bins + 1 => 0.0);
   begin
      for B in 0 .. Bins loop
         Weight (B + 1) := Real'Max (0.0, Sums (Count_Row, B));
         Result (Count_Row, B) := Weight (B + 1);
      end loop;
      for Kind in Pair_Kind loop
         for B in 0 .. Bins loop
            Mean (B + 1) := (if Weight (B + 1) > 0.0 then Sums (Row_Of (Kind), B) / Weight (B + 1) else 0.0);
         end loop;
         Falling (Mean.all, Weight.all, Down.all);
         if Kind = Across_Down then
            declare
               Negated : constant Real_Array := [for I in 1 .. Bins + 1 => -Mean (I)];
            begin
               Falling (Negated, Weight.all, Up.all);
               for I in 1 .. Bins + 1 loop
                  Up (I) := -Up (I);
               end loop;
            end;
            --  Rising from below zero is the same fall seen from the other side.
            if Squares (Mean.all, Weight.all, Up.all) < Squares (Mean.all, Weight.all, Down.all) then
               Down.all := Up.all;
            end if;
         else
            for I in 1 .. Bins + 1 loop
               Down (I) := Real'Max (0.0, Down (I));
            end loop;
         end if;
         for B in 0 .. Bins loop
            Result (Row_Of (Kind), B) := Down (B + 1);
         end loop;
      end loop;
      Free (Mean);
      Free (Weight);
      Free (Down);
      Free (Up);
      return Result;
   end Covariance_By_Distance;

   --  The first distance from which the table no longer changes.
   function Reach (T : Real_Matrix) return Natural is
      Bins : constant Natural := T'Last (2);
      R    : Natural := Bins;
   begin
      while R > 0 and then (for all E in 1 .. 3 => T (E, R - 1) = T (E, Bins)) loop
         R := R - 1;
      end loop;
      return R;
   end Reach;

   ---------------------------------------------------------------------------
   --  The model

   function Bin_Of (M : Model; D : Real) return Natural is (Natural'Min (M.Bins, Natural (Real'Floor (D))));

   procedure Add (T : in out Real_Matrix; Bin : Natural; Au, Av, Bu, Bv, Pairs : Real) is
   begin
      T (Row_Of (Both_Across), Bin) := T (Row_Of (Both_Across), Bin) + Au * Bu;
      T (Row_Of (Both_Down), Bin) := T (Row_Of (Both_Down), Bin) + Av * Bv;
      T (Row_Of (Across_Down), Bin) := T (Row_Of (Across_Down), Bin) + 0.5 * (Au * Bv + Av * Bu);
      T (Count_Row, Bin) := T (Count_Row, Bin) + Pairs;
   end Add;

   procedure Measure
     (Frame, Track : Natural_Array;
      U0, V0       : Real_Array;
      Error        : Real_Array;
      M            : out Model)
   is
      K      : constant Natural := Frame'Length;
      Frames : Natural := 0;
      Tracks : Natural := 0;
   begin
      M := (Is_Measured => False, Bins => 0, Persistent => null, Shared => null,
            Same_Track => [others => 0.0], Alone => [others => 0.0]);
      for I in 1 .. K loop
         Frames := Natural'Max (Frames, Frame (I));
         Tracks := Natural'Max (Tracks, Track (I));
      end loop;
      if K = 0 then
         return;
      end if;
      declare
         Seen    : Natural_Access := new Natural_Array'(1 .. Tracks => 0);
         Pixel_U : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         Pixel_V : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         Sum_A   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         Sum_D   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         --  What each track's errors, squared and across with down, add up to.
         Own     : Real_Matrix (1 .. 3, 1 .. 1) := [others => [others => 0.0]];
         Same_Tr : Real_Matrix (1 .. 3, 1 .. 1) := [others => [others => 0.0]];
         Ordered : Real := 0.0;   --  the ordered pairs of sightings of one track
         Low_U, Low_V    : Real := Real'Last;
         High_U, High_V  : Real := -Real'Last;
         Present         : Natural := 0;
         Frames_Held     : Natural := 0;
         Frame_Start, Frame_Order : Natural_Access;
         Frame_Seen      : Natural_Access := new Natural_Array'(1 .. Frames => 0);
      begin
         for I in 1 .. K loop
            declare
               T : constant Positive := Track (I);
               A : constant Real := Error (2 * I - 1);
               D : constant Real := Error (2 * I);
            begin
               Seen (T) := Seen (T) + 1;
               Pixel_U (T) := U0 (I);
               Pixel_V (T) := V0 (I);
               Sum_A (T) := Sum_A (T) + A;
               Sum_D (T) := Sum_D (T) + D;
               Own (1, 1) := Own (1, 1) + A * A;
               Own (2, 1) := Own (2, 1) + D * D;
               Own (3, 1) := Own (3, 1) + A * D;
               Same_Tr (1, 1) := Same_Tr (1, 1) - A * A;
               Same_Tr (2, 1) := Same_Tr (2, 1) - D * D;
               Same_Tr (3, 1) := Same_Tr (3, 1) - A * D;
               Frame_Seen (Frame (I)) := Frame_Seen (Frame (I)) + 1;
            end;
         end loop;
         for F in 1 .. Frames loop
            if Frame_Seen (F) > 0 then
               Frames_Held := Frames_Held + 1;
            end if;
         end loop;
         Free (Frame_Seen);
         for T in 1 .. Tracks loop
            if Seen (T) > 0 then
               Present := Present + 1;
               Low_U := Real'Min (Low_U, Pixel_U (T));
               Low_V := Real'Min (Low_V, Pixel_V (T));
               High_U := Real'Max (High_U, Pixel_U (T));
               High_V := Real'Max (High_V, Pixel_V (T));
               Same_Tr (1, 1) := Same_Tr (1, 1) + Sum_A (T) * Sum_A (T);
               Same_Tr (2, 1) := Same_Tr (2, 1) + Sum_D (T) * Sum_D (T);
               Same_Tr (3, 1) := Same_Tr (3, 1) + Sum_A (T) * Sum_D (T);
               Ordered := Ordered + Real (Seen (T)) * Real (Seen (T) - 1);
            end if;
         end loop;
         if Frames_Held < 2 or else Present < 2 or else Ordered <= 0.0 then
            Free (Seen);
            Free (Pixel_U);
            Free (Pixel_V);
            Free (Sum_A);
            Free (Sum_D);
            return;
         end if;
         for Kind in Pair_Kind loop
            M.Alone (Kind) := Own (Row_Of (Kind), 1) / Real (K);
            M.Same_Track (Kind) := Same_Tr (Row_Of (Kind), 1) / Ordered;
         end loop;
         M.Bins := Natural (Real'Ceiling (Sqrt ((High_U - Low_U) ** 2 + (High_V - Low_V) ** 2)));
         Group_By (Frame, Frames, Frame_Start, Frame_Order);
         declare
            Same : Matrix_Access := new Real_Matrix'[1 .. Count_Row => [0 .. M.Bins => 0.0]];
            Pool : Matrix_Access := new Real_Matrix'[1 .. Count_Row => [0 .. M.Bins => 0.0]];
         begin
            --  Every pair of sightings, in the table of its two tracks' distance: the two
            --  tracks' sums of errors for all their pairs; and apart, the pairs of one keyframe.
            for T in 1 .. Tracks - 1 loop
               if Seen (T) > 0 then
                  for S in T + 1 .. Tracks loop
                     if Seen (S) > 0 then
                        Add (Pool.all, Bin_Of (M, Distance (Pixel_U (T), Pixel_V (T), Pixel_U (S), Pixel_V (S))),
                             Sum_A (T), Sum_D (T), Sum_A (S), Sum_D (S), Real (Seen (T)) * Real (Seen (S)));
                     end if;
                  end loop;
               end if;
            end loop;
            for F in 1 .. Frames loop
               for X in Frame_Start (F) .. Frame_Start (F + 1) - 2 loop
                  for Y in X + 1 .. Frame_Start (F + 1) - 1 loop
                     declare
                        A : constant Positive := Frame_Order (X);
                        B : constant Positive := Frame_Order (Y);
                     begin
                        if Track (A) /= Track (B) then
                           Add (Same.all, Bin_Of (M, Distance (U0 (A), V0 (A), U0 (B), V0 (B))),
                                Error (2 * A - 1), Error (2 * A), Error (2 * B - 1), Error (2 * B), 1.0);
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
            --  The pairs of two keyframes are the rest.
            for R in 1 .. Count_Row loop
               for B in 0 .. M.Bins loop
                  Pool (R, B) := Pool (R, B) - Same (R, B);
               end loop;
            end loop;
            M.Persistent := Covariance_By_Distance (Pool.all);
            M.Shared := Covariance_By_Distance (Same.all);
            Free (Pool);
            Free (Same);
         end;
         M.Is_Measured := True;
         Free (Seen);
         Free (Pixel_U);
         Free (Pixel_V);
         Free (Sum_A);
         Free (Sum_D);
         Free (Frame_Start);
         Free (Frame_Order);
      end;
   end Measure;

   function Measured (M : Model) return Boolean is (M.Is_Measured);

   function Alone (M : Model) return Entries is (M.Alone);

   function Same_Track (M : Model) return Entries is (M.Same_Track);

   function Persistent (M : Model; Distance : Real) return Entries is
     (if M.Persistent = null then [others => 0.0]
      else [for Kind in Pair_Kind => M.Persistent (Row_Of (Kind), Bin_Of (M, Distance))]);

   function Shared (M : Model; Distance : Real) return Entries is
     (if M.Shared = null then [others => 0.0]
      else [for Kind in Pair_Kind => M.Shared (Row_Of (Kind), Bin_Of (M, Distance))]);

   function Added_By_Keyframe (M : Model) return Entries is
      Sum   : Entries := [others => 0.0];
      Pairs : Real := 0.0;
   begin
      if M.Shared = null or else M.Persistent = null then
         return Sum;
      end if;
      for B in 0 .. M.Bins loop
         Pairs := Pairs + M.Shared (Count_Row, B);
         for Kind in Pair_Kind loop
            Sum (Kind) := Sum (Kind)
              + M.Shared (Count_Row, B) * (M.Shared (Row_Of (Kind), B) - M.Persistent (Row_Of (Kind), B));
         end loop;
      end loop;
      return (if Pairs > 0.0 then [for Kind in Pair_Kind => Sum (Kind) / Pairs] else Sum);
   end Added_By_Keyframe;

   function Half_Distance_Persistent (M : Model) return Real is
      Own : constant Real := 0.5 * (M.Same_Track (Both_Across) + M.Same_Track (Both_Down));
   begin
      if M.Persistent = null or else Own <= 0.0 then
         return 0.0;
      end if;
      for B in M.Persistent'Range (2) loop
         if M.Persistent (Count_Row, B) > 0.0
           and then 0.5 * (M.Persistent (Row_Of (Both_Across), B) + M.Persistent (Row_Of (Both_Down), B)) <= 0.5 * Own
         then
            return Real (B);
         end if;
      end loop;
      return Real (M.Bins);
   end Half_Distance_Persistent;

   procedure Free (M : in out Model) is
   begin
      if M.Persistent /= null then
         Free (M.Persistent);
      end if;
      if M.Shared /= null then
         Free (M.Shared);
      end if;
      M := (Is_Measured => False, Bins => 0, Persistent => null, Shared => null,
            Same_Track => [others => 0.0], Alone => [others => 0.0]);
   end Free;

   ---------------------------------------------------------------------------
   --  The middle of the sandwich

   procedure Meat
     (M      : Model;
      Frame  : Natural_Array;
      Track  : Natural_Array;
      U0, V0 : Real_Array;
      Rows   : Real_Matrix;
      Result : out Real_Matrix)
   is
      P      : constant Natural := Rows'Length (2);
      K      : constant Natural := Frame'Length;
      Frames : Natural := 0;
      Tracks : Natural := 0;

      --  Result += Coefficient * X Y', for the rows X and Y of a matrix.
      procedure Add_Outer (X, Y : Real_Matrix; Rx, Ry : Positive; Coefficient : Real) is
      begin
         if Coefficient /= 0.0 then
            for A in 1 .. P loop
               declare
                  Xa : constant Real := Coefficient * X (Rx, A);
               begin
                  if Xa /= 0.0 then
                     for B in 1 .. P loop
                        Result (A, B) := Result (A, B) + Xa * Y (Ry, B);
                     end loop;
                  end if;
               end;
            end loop;
         end if;
      end Add_Outer;
   begin
      Result := [others => [others => 0.0]];
      if not M.Is_Measured or else K = 0 or else P = 0 then
         return;
      end if;
      for I in 1 .. K loop
         Frames := Natural'Max (Frames, Frame (I));
         Tracks := Natural'Max (Tracks, Track (I));
      end loop;
      declare
         --  The excess of the covariance in one keyframe over that of two, and
         --  what it falls to far away: a share every pair of a keyframe holds.
         Excess    : Matrix_Access := new Real_Matrix'[1 .. 3 => [0 .. M.Bins => 0.0]];
         Level     : Entries;
         Near      : Natural;
         Seen      : Natural_Access := new Natural_Array'(1 .. Tracks => 0);
         Pixel_U   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         Pixel_V   : Real_Access := new Real_Array'(1 .. Tracks => 0.0);
         --  What each track's sightings hold of the gradient, across and down,
         --  and what the covariance carries of all the tracks' to it.
         Held      : constant Matrix_Access := new Real_Matrix'[1 .. 2 * Tracks => [1 .. P => 0.0]];
         Carried   : constant Matrix_Access := new Real_Matrix'[1 .. 2 * Tracks => [1 .. P => 0.0]];
         Frame_Start, Frame_Order : Natural_Access;
      begin
         for B in 0 .. M.Bins loop
            for E in 1 .. 3 loop
               Excess (E, B) := M.Shared (E, B) - M.Persistent (E, B);
            end loop;
         end loop;
         for Kind in Pair_Kind loop
            Level (Kind) := Excess (Row_Of (Kind), M.Bins);
         end loop;
         Near := Reach (Excess.all);
         for I in 1 .. K loop
            declare
               T : constant Positive := Track (I);
            begin
               Seen (T) := Seen (T) + 1;
               Pixel_U (T) := U0 (I);
               Pixel_V (T) := V0 (I);
               for Q in 1 .. P loop
                  Held (2 * T - 1, Q) := Held (2 * T - 1, Q) + Rows (2 * I - 1, Q);
                  Held (2 * T, Q) := Held (2 * T, Q) + Rows (2 * I, Q);
               end loop;
            end;
         end loop;

         --  Two sightings' errors, whoever they are of, are as alike as the
         --  tracks they are of: a point's own errors in two keyframes, those
         --  of two points in two keyframes at the distance of their pixels.
         for T in 1 .. Tracks loop
            if Seen (T) > 0 then
               for S in 1 .. Tracks loop
                  if Seen (S) > 0 then
                     declare
                        C : constant Entries :=
                          (if S = T then M.Same_Track
                           else Persistent (M, Distance (Pixel_U (T), Pixel_V (T), Pixel_U (S), Pixel_V (S))));
                     begin
                        for Q in 1 .. P loop
                           Carried (2 * T - 1, Q) := Carried (2 * T - 1, Q) + C (Both_Across) * Held (2 * S - 1, Q)
                             + C (Across_Down) * Held (2 * S, Q);
                           Carried (2 * T, Q) := Carried (2 * T, Q) + C (Across_Down) * Held (2 * S - 1, Q)
                             + C (Both_Down) * Held (2 * S, Q);
                        end loop;
                     end;
                  end if;
               end loop;
               Add_Outer (Held.all, Carried.all, 2 * T - 1, 2 * T - 1, 1.0);
               Add_Outer (Held.all, Carried.all, 2 * T, 2 * T, 1.0);
            end if;
         end loop;

         --  The pairs of one keyframe have more: the excess of Shared over
         --  Persistent. Its far level is a share of every pair of the keyframe
         --  (the sum of the keyframe's gradient rows, outer to itself, less
         --  each sighting alone); the part above it is between the pairs
         --  within Near pixels.
         Group_By (Frame, Frames, Frame_Start, Frame_Order);
         for F in 1 .. Frames loop
            declare
               First : constant Natural := Frame_Start (F);
               After : constant Natural := Frame_Start (F + 1);
               Sum   : Real_Matrix (1 .. 2, 1 .. P) := [others => [others => 0.0]];
               Local : Real_Matrix (1 .. 2, 1 .. P);
            begin
               for X in First .. After - 1 loop
                  for Q in 1 .. P loop
                     Sum (1, Q) := Sum (1, Q) + Rows (2 * Frame_Order (X) - 1, Q);
                     Sum (2, Q) := Sum (2, Q) + Rows (2 * Frame_Order (X), Q);
                  end loop;
               end loop;
               Add_Outer (Sum, Sum, 1, 1, Level (Both_Across));
               Add_Outer (Sum, Sum, 2, 2, Level (Both_Down));
               Add_Outer (Sum, Sum, 1, 2, Level (Across_Down));
               Add_Outer (Sum, Sum, 2, 1, Level (Across_Down));
               if Near > 0 then
                  for X in First .. After - 1 loop
                     declare
                        A : constant Positive := Frame_Order (X);
                     begin
                        Local := [others => [others => 0.0]];
                        for Y in First .. After - 1 loop
                           declare
                              B : constant Positive := Frame_Order (Y);
                              Bin : constant Natural :=
                                Bin_Of (M, Distance (U0 (A), V0 (A), U0 (B), V0 (B)));
                           begin
                              if Y /= X and then Bin < Near then
                                 declare
                                    Caa : constant Real := Excess (Row_Of (Both_Across), Bin) - Level (Both_Across);
                                    Cdd : constant Real := Excess (Row_Of (Both_Down), Bin) - Level (Both_Down);
                                    Cad : constant Real := Excess (Row_Of (Across_Down), Bin) - Level (Across_Down);
                                 begin
                                    for Q in 1 .. P loop
                                       Local (1, Q) := Local (1, Q) + Caa * Rows (2 * B - 1, Q) + Cad * Rows (2 * B, Q);
                                       Local (2, Q) := Local (2, Q) + Cad * Rows (2 * B - 1, Q) + Cdd * Rows (2 * B, Q);
                                    end loop;
                                 end;
                              end if;
                           end;
                        end loop;
                        for Q in 1 .. P loop
                           for R in 1 .. P loop
                              Result (Q, R) := Result (Q, R) + Rows (2 * A - 1, Q) * Local (1, R)
                                + Rows (2 * A, Q) * Local (2, R);
                           end loop;
                        end loop;
                     end;
                  end loop;
               end if;
            end;
         end loop;

         --  Each sighting with itself: its own errors' covariance, less what
         --  the sums above counted for it (its point's own in two keyframes,
         --  and the far level of its keyframe).
         for I in 1 .. K loop
            declare
               Own : constant Entries :=
                 [for Kind in Pair_Kind => M.Alone (Kind) - M.Same_Track (Kind) - Level (Kind)];
            begin
               Add_Outer (Rows, Rows, 2 * I - 1, 2 * I - 1, Own (Both_Across));
               Add_Outer (Rows, Rows, 2 * I, 2 * I, Own (Both_Down));
               Add_Outer (Rows, Rows, 2 * I - 1, 2 * I, Own (Across_Down));
               Add_Outer (Rows, Rows, 2 * I, 2 * I - 1, Own (Across_Down));
            end;
         end loop;
         Free (Seen);
         Free (Pixel_U);
         Free (Pixel_V);
         declare
            Spent : Matrix_Access := Held;
            Used  : Matrix_Access := Carried;
         begin
            Free (Spent);
            Free (Used);
         end;
         Free (Frame_Start);
         Free (Frame_Order);
         Free (Excess);
      end;
   end Meat;

end Driver.Robot.Kinematics.Errors;
