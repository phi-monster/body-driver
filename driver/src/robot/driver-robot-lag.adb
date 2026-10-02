with Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Vectors;
with Ada.Unchecked_Deallocation;
with Driver.Robot.Channels;
with Driver.Stats;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Robot.Flow;
with Driver.Robot.Regression;

package body Driver.Robot.Lag is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  Everything sized by beats lives on the heap: the estimates also run in
   --  the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Index_Array is array (Positive range <>) of Positive;
   type Index_Access is access Index_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);
   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);

   type Flags is array (Positive range <>) of Boolean;

   --  A series over the beats of the stream (beat B stored at B + 1), with
   --  gaps where it is not available.
   type Series (Length : Natural) is record
      Value : Real_Array (1 .. Length) := [others => 0.0];
      Have  : Flags (1 .. Length) := [others => False];
   end record;

   type Series_Access is access Series;
   procedure Free is new Ada.Unchecked_Deallocation (Series, Series_Access);

   --  Replaces the available values by their ranks among themselves, ties
   --  sharing the average rank.
   procedure Rank (S : in out Series) is
      N : Natural := 0;
   begin
      for B of S.Have loop
         if B then
            N := N + 1;
         end if;
      end loop;
      if N = 0 then
         return;
      end if;
      declare
         Index : Index_Access := new Index_Array (1 .. N);
         K     : Natural := 0;
         Ranks : Real_Access := new Real_Array (1 .. N);

         procedure Sift (Start, Stop : Natural) is
            Root  : Natural := Start;
            Child : Natural;
            T     : Positive;
         begin
            loop
               Child := 2 * Root;
               exit when Child > Stop;
               if Child < Stop and then S.Value (Index (Child)) < S.Value (Index (Child + 1)) then
                  Child := Child + 1;
               end if;
               exit when not (S.Value (Index (Root)) < S.Value (Index (Child)));
               T := Index (Root);
               Index (Root) := Index (Child);
               Index (Child) := T;
               Root := Child;
            end loop;
         end Sift;
      begin
         for I in S.Have'Range loop
            if S.Have (I) then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
         --  Heap sort of the indexes by value.
         for Start in reverse 1 .. N / 2 loop
            Sift (Start, N);
         end loop;
         for Stop in reverse 2 .. N loop
            declare
               T : constant Positive := Index (1);
            begin
               Index (1) := Index (Stop);
               Index (Stop) := T;
            end;
            Sift (1, Stop - 1);
         end loop;
         declare
            I : Positive := 1;
         begin
            while I <= N loop
               declare
                  J : Positive := I;
               begin
                  while J < N and then S.Value (Index (J + 1)) = S.Value (Index (I)) loop
                     J := J + 1;
                  end loop;
                  for L in I .. J loop
                     Ranks (L) := Real (I + J) / 2.0;
                  end loop;
                  I := J + 1;
               end;
            end loop;
         end;
         for L in 1 .. N loop
            S.Value (Index (L)) := Ranks (L);
         end loop;
         Free (Index);
         Free (Ranks);
      end;
   end Rank;

   --  The first difference of a series, in place, available where both
   --  terms are: from the last term back, so each term is differenced
   --  against the one before it while that one still holds its value.
   procedure Difference (S : in out Series) is
   begin
      for I in reverse S.Value'First + 1 .. S.Value'Last loop
         if S.Have (I) and then S.Have (I - 1) then
            S.Value (I) := S.Value (I) - S.Value (I - 1);
         else
            S.Value (I) := 0.0;
            S.Have (I) := False;
         end if;
      end loop;
      if S.Length > 0 then
         S.Value (S.Value'First) := 0.0;
         S.Have (S.Have'First) := False;
      end if;
   end Difference;

   --  The series takes more than one value.
   function Varies (S : Series) return Boolean is
     (for some I in S.Value'Range => S.Have (I)
        and then (for some J in S.Value'Range => S.Have (J) and then S.Value (J) /= S.Value (I)));

   --  How far a lag can be told at all: a push's response is told from the
   --  next push's only when it shows before that one starts, so the lag is
   --  identifiable up to the median stretch between one burst of pushes and
   --  the next (pushes that each cut the last one short, a ramp, are one
   --  burst; bursts of groups that start on the same beat are one start).
   --  Without two bursts, every shift the stream allows.
   function Identifiable (M : Model; Last : Natural) return Natural is
      package Natural_Vectors is new Ada.Containers.Vectors (Positive, Natural);
      package Sorting is new Natural_Vectors.Generic_Sorting;
      Starts : Natural_Vectors.Vector;
   begin
      for S of M.Groups loop
         if S.Commandable then
            declare
               Cut : Boolean := False;
            begin
               for E of S.Episodes loop
                  if not Cut then
                     Starts.Append (E.Start);
                  end if;
                  Cut := E.Ended and then not E.Settled;
               end loop;
            end;
         end if;
      end loop;
      Sorting.Sort (Starts);
      declare
         Gaps : Real_Access := new Real_Array (1 .. Natural (Starts.Length));
         K    : Natural := 0;
      begin
         for I in Starts.First_Index + 1 .. Starts.Last_Index loop
            if Starts (I) > Starts (I - 1) then
               K := K + 1;
               Gaps (K) := Real (Starts (I) - Starts (I - 1));
            end if;
         end loop;
         return Result : constant Natural :=
           (if K = 0 then Last else Natural'Min (Last, Natural (Driver.Stats.Median (Gaps (1 .. K)))))
         do
            Free (Gaps);
         end return;
      end;
   end Identifiable;

   function Longest (M : Model) return Natural is
     (if M.Beats > 0 then Identifiable (M, M.Beats - 1) else 0);

   procedure Measure (M : in out Model) is
      Last : constant Integer := M.Beats - 1;
      Bound : constant Natural := (if M.Beats > 0 then Identifiable (M, M.Beats - 1) else 0);
   begin
      M.Lags.Clear;
      M.Lag_Known.Clear;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         M.Lags.Append (0);
         M.Lag_Known.Append (False);
      end loop;
      if Last < 1 then
         return;
      end if;
      declare
         Speeds : array (M.Groups.First_Index .. M.Groups.Last_Index) of Series_Access := [others => null];
      begin
         for G in Speeds'Range loop
            Speeds (G) := new Series (M.Beats);
            if M.Groups (G).Commandable then
               for B in 1 .. Last loop
                  if Channels.Has_Reading (M, G, B) and then Channels.Has_Reading (M, G, B - 1) then
                     declare
                        Sum : Real := 0.0;
                     begin
                        for C in 1 .. M.Groups (G).Size loop
                           Sum := Sum + Channels.Change (M, G, B, C) ** 2;
                        end loop;
                        Speeds (G).Value (B + 1) := (if Channels.Pushed (M, G, B) then Sqrt (Sum) else 0.0);
                        Speeds (G).Have (B + 1) := True;
                     end;
                  end if;
               end loop;
               Difference (Speeds (G).all);
               Rank (Speeds (G).all);
            end if;
         end loop;
         declare
         --  The chance that noise fits the eye's motion at this shift as well as
         --  the pushes do: the eye's differenced motion ranks regressed on every
         --  varying group's differenced speed ranks at once, each of which can
         --  only add motion, against the F distribution of that fit.
         function Fit_Tail (Motion : Series; Shift : Integer) return Real is
            Varying : array (1 .. Natural (M.Groups.Length)) of Group_Id;
            K       : Natural := 0;
            Rows    : Natural := 0;
         begin
            for G in Speeds'Range loop
               if M.Groups (G).Commandable and then Varies (Speeds (G).all) then
                  K := K + 1;
                  Varying (K) := G;
               end if;
            end loop;
            for I in Motion.Value'Range loop
               if Motion.Have (I) and then I - Shift in Motion.Value'Range
                 and then (for all J in 1 .. K => Speeds (Varying (J)).Have (I - Shift))
               then
                  Rows := Rows + 1;
               end if;
            end loop;
            if K = 0 or else Rows <= K + 1 then
               return 1.0;
            end if;
            declare
               X    : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Rows, 1 .. K);
               Y    : Real_Access := new Real_Array (1 .. Rows);
               R    : Natural := 0;
               Used : Natural;
            begin
               for I in Motion.Value'Range loop
                  if Motion.Have (I) and then I - Shift in Motion.Value'Range
                    and then (for all J in 1 .. K => Speeds (Varying (J)).Have (I - Shift))
                  then
                     R := R + 1;
                     Y (R) := Motion.Value (I);
                     for J in 1 .. K loop
                        X (R, J) := Speeds (Varying (J)).Value (I - Shift);
                     end loop;
                  end if;
               end loop;
               declare
                  R2 : constant Real := Regression.Explained_Nonnegative (X.all, Y.all, Used);
               begin
                  Free (X);
                  Free (Y);
                  if Used = 0 or else R2 >= 1.0 then
                     return (if Used = 0 then 1.0 else 0.0);
                  end if;
                  return Driver.Distributions.F_Upper_Tail
                    ((R2 / Real (Used)) / ((1.0 - R2) / Real (Rows - Used - 1)), Used, Rows - Used - 1);
               end;
            end;
         end Fit_Tail;
      begin
         for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
            declare
               S      : Eye_Stream renames M.Eyes (E);
               N      : constant Natural := Cells (S.Grid);
               Motion : Series_Access := new Series (M.Beats);
               Best   : Real := 1.0;     --  the smallest chance of a fit this good by noise
               Lag    : Integer := 0;
               Tests  : Natural := 0;
            begin
               if N > 0 then
                  for B in 1 .. Natural'Min (Last, Natural (S.Measured.Length) - 1) loop
                     if S.Measured (B) then
                        declare
                           Sum : Real := 0.0;
                        begin
                           --  A cell whose content moved too far to be resolved moved at
                           --  least as far as a resolved displacement can go: half its
                           --  width and half its height.
                           for C in 0 .. N - 1 loop
                              if S.Resolved (B * N + C) then
                                 Sum := Sum + Sqrt (S.Du (B * N + C) ** 2 + S.Dv (B * N + C) ** 2);
                              elsif S.Condition (B * N + C) > 0.0 then
                                 declare
                                    X0, X1, Y0, Y1 : Natural;
                                 begin
                                    Flow.Bounds (S.Grid, C + 1, X0, X1, Y0, Y1);
                                    Sum := Sum + Sqrt ((Real (X1 - X0) / 2.0) ** 2 + (Real (Y1 - Y0) / 2.0) ** 2);
                                 end;
                              end if;
                           end loop;
                           Motion.Value (B + 1) := Sum / Real (N);
                           Motion.Have (B + 1) := True;
                        end;
                     end if;
                  end loop;
                  Difference (Motion.all);
                  Rank (Motion.all);
                  for Shift in -Bound .. Bound loop
                     declare
                        Tail : constant Real := Fit_Tail (Motion.all, Shift);
                     begin
                        Tests := Tests + 1;
                        if Tail < Best then
                           Best := Tail;
                           Lag := Shift;
                        end if;
                     end;
                  end loop;
                  --  The best of many shifts is one of a family: the lag is
                  --  measured only when it stands out of all of them.
                  if Tests > 0
                    and then Best < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z) / Real (Tests)
                  then
                     M.Lags.Replace_Element (E, Lag);
                     M.Lag_Known.Replace_Element (E, True);
                  end if;
               end if;
               Free (Motion);
            end;
         end loop;
         end;
         for G in Speeds'Range loop
            Free (Speeds (G));
         end loop;
      end;
   end Measure;

end Driver.Robot.Lag;
