with Ada.Containers;
with Ada.Unchecked_Deallocation;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Lag;
with Driver.Robot.Regression;
with Driver.Stats;

package body Driver.Robot.Lockin is

   --  Everything sized by beats, cells or pixels lives on the heap: the
   --  estimates also run in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Natural_Array is array (Positive range <>) of Natural;
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);
   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);
   type Flags is array (Positive range <>) of Boolean;
   type Flags_Access is access Flags;
   procedure Free is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
   type Flag_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Boolean;
   type Flag_Grid_Access is access Flag_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Grid, Flag_Grid_Access);
   type Real_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Real;
   type Real_Grid_Access is access Real_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Grid, Real_Grid_Access);
   type Natural_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Natural;
   type Natural_Grid_Access is access Natural_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Grid, Natural_Grid_Access);

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   --  A regressor: one channel of one commandable group.
   type Column is record
      Group   : Group_Id;
      Channel : Positive;
   end record;

   type Column_Array is array (Positive range <>) of Column;

   function Median_Shift (S : Eye_Stream; Kept, Column, Count : Positive) return Real is
      Values : Real_Access := new Real_Array (1 .. Count);
      K      : Natural := 0;
   begin
      for Cell in 0 .. Natural (S.Shifts.Length) / Kept - 1 loop
         if S.Shifts (Cell * Kept + Column - 1) > 0.0 then
            K := K + 1;
            Values (K) := S.Shifts (Cell * Kept + Column - 1);
         end if;
      end loop;
      return Result : constant Real := Driver.Stats.Median (Values.all) do
         Free (Values);
      end return;
   end Median_Shift;

   --  How many of an eye's textured cells show group G's motion, at least and
   --  at most, which is what a whole image moving takes (Measure): the cells
   --  whose displacement follows the group, and the cells too noisy to tell by
   --  themselves in the share that they show it together.
   --
   --  A silent cell says nothing against the whole image moving when its own
   --  noise would not let it show the motion. The cells that did respond
   --  measure the motion's energy, the squared displacement the group caused
   --  summed over the regression's rows: a cell's Wald statistic less its
   --  degrees of freedom, times its variance, estimates it without bias. A
   --  cell can tell a motion of the median of those energies when the
   --  statistic it would have, less Z of that statistic's own spread, reaches
   --  the critical value of its test; the energy over its variance that takes
   --  is Needed. Whether a cell can tell depends on its noise alone, not on
   --  whether it responded, so the weighted mean of the energies of all the
   --  cells that cannot is unbiased. As a share of the median, it Z standard
   --  errors below is the least share of those cells that show the motion, and
   --  Z standard errors above the most. A picture that moves only where it is
   --  best measured gains nothing by this, however many cells are too noisy to
   --  tell: together they show no energy.
   procedure Cells_Showing
     (G          : Group_Id;
      Textured   : Flags;
      Responding : Flag_Grid;
      Energy     : Real_Grid;
      Dof        : Natural_Grid;
      Noise      : Real_Vectors.Vector;
      Least, Most : out Natural)
   is
      Z        : constant Real := Driver.Conventions.Z;
      Tail     : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Z);
      Count    : Natural := 0;
      Largest  : Natural := 0;
      Energies : Real_Access := new Real_Array (1 .. Textured'Length);

      --  The energy over the variance at which a statistic of that many degrees
      --  of freedom, which is a non-central chi-square of mean K + Lambda and
      --  variance 2 (K + 2 Lambda) for Lambda that ratio, clears the critical
      --  value Q of its test by Z of its spread: the root of
      --  (K + Lambda - Q) ** 2 = Z ** 2 * 2 (K + 2 Lambda) above Q - K.
      function Needed_For (Freedom : Positive) return Real is
         K : constant Real := Real (Freedom);
         Q : constant Real := Driver.Distributions.Chi_Square_Quantile (Tail, Freedom);
      begin
         return (Q - K + 2.0 * Z ** 2) + Z * Sqrt (2.0 * (2.0 * Q - K) + 4.0 * Z ** 2);
      end Needed_For;
   begin
      for Cell in Textured'Range loop
         if Textured (Cell) then
            Largest := Natural'Max (Largest, Dof (Cell, G));
            if Responding (Cell, G) then
               Count := Count + 1;
               Energies (Count) := Energy (Cell, G);
            end if;
         end if;
      end loop;
      Least := Count;
      Most := Count;
      if Count > 0 and then Largest > 0 then
         declare
            Typical   : constant Real := Driver.Stats.Median (Energies (1 .. Count));
            Needed    : Real_Array (1 .. Largest) := [others => 0.0];   --  by degrees of freedom, found when first asked
            Able      : Natural := 0;   --  cells that could tell and responded
            Too_Noisy : Natural := 0;   --  cells that could not tell
            Weights, Weighted : Real := 0.0;
         begin
            for Cell in Textured'Range loop
               if Textured (Cell) and then Dof (Cell, G) > 0 and then Typical > 0.0 then
                  declare
                     Freedom : constant Positive := Dof (Cell, G);
                     Var     : constant Real := Noise (Cell - 1) ** 2;
                  begin
                     if Needed (Freedom) = 0.0 then
                        Needed (Freedom) := Needed_For (Freedom);
                     end if;
                     if Needed (Freedom) * Var <= Typical then
                        if Responding (Cell, G) then
                           Able := Able + 1;
                        end if;
                     else
                        declare
                           --  The variance of the cell's energy: Var ** 2 times that of its statistic.
                           Weight : constant Real := 1.0 / (2.0 * Real (Freedom) * Var ** 2 + 4.0 * Var * Typical);
                        begin
                           Too_Noisy := Too_Noisy + 1;
                           Weights := Weights + Weight;
                           Weighted := Weighted + Weight * Energy (Cell, G);
                        end;
                     end if;
                  end;
               end if;
            end loop;
            if Too_Noisy > 0 then
               declare
                  Mean  : constant Real := Weighted / Weights;
                  Sigma : constant Real := 1.0 / Sqrt (Weights);
                  Low   : constant Real := Real'Max (0.0, Real'Min (1.0, (Mean - Z * Sigma) / Typical));
                  High  : constant Real := Real'Max (0.0, Real'Min (1.0, (Mean + Z * Sigma) / Typical));
               begin
                  Least := Natural'Max (Count, Able + Natural (Real'Floor (Low * Real (Too_Noisy))));
                  Most := Natural'Max (Count, Able + Natural (Real'Ceiling (High * Real (Too_Noisy))));
               end;
            end if;
         end;
      end if;
      Free (Energies);
   end Cells_Showing;

   procedure Measure (M : in out Model) is
      Groups : constant Natural := Natural (M.Groups.Length);
      Eyes   : constant Natural := Natural (M.Eyes.Length);
      All_Columns : Natural := 0;

      --  A group's reading change as a cause: only while it is being pushed.
      --  A held group that moves a little because another one moved is not
      --  a cause, or a regression could credit it with the other's effect
      --  (its tiny motion is a scaled copy) and reject its own still pushes
      --  as outliers.
      function Pushed_Change (G : Group_Id; Beat : Natural; Channel : Positive) return Real is
        (if Channels.Pushed (M, G, Beat) then Channels.Change (M, G, Beat, Channel) else 0.0);
   begin
      M.Graph.Effects.Clear;
      M.Graph.Effects.Append (Eye_Effect'(others => <>), Ada.Containers.Count_Type (Groups * Eyes));
      for S of M.Groups loop
         if S.Commandable then
            All_Columns := All_Columns + S.Size;
         end if;
      end loop;
      if All_Columns = 0 or else M.Beats < 2 then
         return;
      end if;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S     : Eye_Stream renames M.Eyes (E);
            N     : constant Natural := Cells (S.Grid);
            Lag   : constant Integer := (if E <= M.Lags.Last_Index then M.Lags (E) else 0);
            Rows  : Natural := 0;

            --  A beat of the eye can be explained when its displacement was
            --  measured, every commandable group's readings exist for the
            --  beat it shows and the one before, and at most one group was
            --  being pushed then: a push that coincides with another is no
            --  reference for either (their effects cannot be told apart).
            function Usable (B : Natural) return Boolean is
               R      : constant Integer := B - Lag;
               Pushes : Natural := 0;
            begin
               if B >= Natural (S.Measured.Length) or else not S.Measured (B) or else R < 1 then
                  return False;
               end if;
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  if M.Groups (G).Commandable then
                     if not Channels.Has_Reading (M, G, R) or else not Channels.Has_Reading (M, G, R - 1) then
                        return False;
                     elsif Channels.Pushed (M, G, R) then
                        Pushes := Pushes + 1;
                     end if;
                  end if;
               end loop;
               return Pushes <= 1;
            end Usable;
         begin
            S.Noise.Clear;
            S.Textured.Clear;
            S.Kept_Groups.Clear;
            S.Kept_Channels.Clear;
            S.Gains.Clear;
            S.Gain_Variances.Clear;
            S.Shifts.Clear;
            if N > 0 then
               for B in 1 .. M.Beats - 1 loop
                  if Usable (B) then
                     Rows := Rows + 1;
                  end if;
               end loop;
            end if;
            --  A regression needs more observations than coefficients.
            if Rows > All_Columns + 1 then
               declare
                  Beat_Of : Natural_Access := new Natural_Array (1 .. Rows);
                  Cols    : Column_Array (1 .. All_Columns);
                  Kept    : Natural := 0;
               begin
                  declare
                     R : Natural := 0;
                  begin
                     for B in 1 .. M.Beats - 1 loop
                        if Usable (B) then
                           R := R + 1;
                           Beat_Of (R) := B;
                        end if;
                     end loop;
                  end;
                  --  Channels that never changed over these beats tell nothing.
                  for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                     if M.Groups (G).Commandable then
                        for C in 1 .. M.Groups (G).Size loop
                           declare
                              Moved : Boolean := False;
                           begin
                              for R in 1 .. Rows loop
                                 if Pushed_Change (G, Beat_Of (R) - Lag, C) /= 0.0 then
                                    Moved := True;
                                    exit;
                                 end if;
                              end loop;
                              if Moved then
                                 Kept := Kept + 1;
                                 Cols (Kept) := (Group => G, Channel => C);
                              end if;
                           end;
                        end loop;
                     end if;
                  end loop;
                  for K in 1 .. Kept loop
                     S.Kept_Groups.Append (Natural (Cols (K).Group));
                     S.Kept_Channels.Append (Cols (K).Channel);
                  end loop;
                  if Kept > 0 then
                     declare
                        X : Matrix_Access := new Real_Matrix (1 .. Rows, 1 .. Kept + 1);
                        Responding : Flag_Grid_Access :=
                          new Flag_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => False]];
                        --  What each cell's block of each group's columns measured: the energy of the
                        --  motion it saw (Cells_Showing) and the degrees of freedom of that test.
                        Energy : Real_Grid_Access :=
                          new Real_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => 0.0]];
                        Dof : Natural_Grid_Access :=
                          new Natural_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => 0]];
                        Textured : Flags_Access := new Flags'[1 .. N => False];
                        --  One cell: its displacements regressed on the pushes, over
                        --  the beats where the cell resolved one, and for every
                        --  group whether its block responds.
                        procedure Fit_Resolved (Cell : Positive; Here : Positive) is
                           Xc     : Matrix_Access := new Real_Matrix (1 .. Here, 1 .. Kept + 1);
                           U      : Real_Access := new Real_Array (1 .. Here);
                           V      : Real_Access := new Real_Array (1 .. Here);
                           Floors : Real_Access := new Real_Array (1 .. Here);
                           K      : Natural := 0;
                        begin
                           for R in 1 .. Rows loop
                              if S.Resolved (Beat_Of (R) * N + Cell - 1) then
                                 K := K + 1;
                                 for J in 1 .. Kept + 1 loop
                                    Xc (K, J) := X (R, J);
                                 end loop;
                                 U (K) := S.Du (Beat_Of (R) * N + Cell - 1);
                                 V (K) := S.Dv (Beat_Of (R) * N + Cell - 1);
                                 Floors (K) := Flow.Noise_Floor (S.Condition (Beat_Of (R) * N + Cell - 1),
                                                                 S.Luma_Variance (Cell - 1));
                              end if;
                           end loop;
                           declare
                              Floor : constant Real := Driver.Stats.Median (Floors.all);
                              Fu    : constant Regression.Fit := Regression.Solve (Xc.all, U.all, Floor);
                              Fv    : constant Regression.Fit := Regression.Solve (Xc.all, V.all, Floor);
                           begin
                              S.Noise.Append (Sqrt ((Fu.Scale ** 2 + Fv.Scale ** 2) / 2.0));
                              for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                                 declare
                                    First : Natural := 0;
                                    Last  : Natural := 0;
                                 begin
                                    for K in 1 .. Kept loop
                                       if Cols (K).Group = G then
                                          if First = 0 then
                                             First := K + 1;
                                          end if;
                                          Last := K + 1;
                                       end if;
                                    end loop;
                                    if First > 0 then
                                       declare
                                          Su, Sv : Real;
                                          Ku, Kv : Natural;
                                       begin
                                          Regression.Test_Block (Fu, First, Last, Su, Ku);
                                          Regression.Test_Block (Fv, First, Last, Sv, Kv);
                                          Responding (Cell, G) :=
                                            Ku + Kv > 0
                                            and then Driver.Distributions.Chi_Square_Deviate (Su + Sv, Ku + Kv)
                                                       > Driver.Conventions.Z;
                                          --  A statistic exceeds its degrees of freedom by the
                                          --  motion's energy over the variance, on average.
                                          Energy (Cell, G) := (Su - Real (Ku)) * Fu.Scale ** 2 + (Sv - Real (Kv)) * Fv.Scale ** 2;
                                          Dof (Cell, G) := Ku + Kv;
                                       end;
                                    end if;
                                 end;
                              end loop;
                              --  Each regressor's gain in this cell: its displacement per
                              --  reading unit in units of the cell's noise, squared, less
                              --  what estimating it adds on average (its own variance in
                              --  those units), where the cell responds to its group.
                              declare
                                 Var_U : constant Real_Array := Regression.Coefficient_Variances (Fu);
                                 Var_V : constant Real_Array := Regression.Coefficient_Variances (Fv);
                              begin
                                 for K in 1 .. Kept loop
                                    if Responding (Cell, Cols (K).Group) and then Fu.Scale > 0.0 and then Fv.Scale > 0.0 then
                                       declare
                                          Bu : constant Real := Fu.Beta (K + 1) / Fu.Scale;
                                          Bv : constant Real := Fv.Beta (K + 1) / Fv.Scale;
                                          Gu : constant Real := Var_U (K + 1) / Fu.Scale ** 2;
                                          Gv : constant Real := Var_V (K + 1) / Fv.Scale ** 2;
                                       begin
                                          S.Gains.Append (Bu ** 2 + Bv ** 2 - Gu - Gv);
                                          S.Gain_Variances.Append (4.0 * (Gu * Bu ** 2 + Gv * Bv ** 2));
                                          S.Shifts.Append (Sqrt (Fu.Beta (K + 1) ** 2 + Fv.Beta (K + 1) ** 2));
                                       end;
                                    else
                                       S.Gains.Append (0.0);
                                       S.Gain_Variances.Append (0.0);
                                       S.Shifts.Append (0.0);
                                    end if;
                                 end loop;
                              end;
                           end;
                           Free (Xc);
                           Free (U);
                           Free (V);
                           Free (Floors);
                        end Fit_Resolved;

                        procedure Fit_Cell (Cell : Positive) is
                           Cond : Real_Access := new Real_Array (1 .. Rows);
                           Here : Natural := 0;
                        begin
                           for R in 1 .. Rows loop
                              Cond (R) := S.Condition (Beat_Of (R) * N + Cell - 1);
                              if S.Resolved (Beat_Of (R) * N + Cell - 1) then
                                 Here := Here + 1;
                              end if;
                           end loop;
                           --  A cell can show a displacement when it has texture in two
                           --  directions, and it is tested when it resolved more
                           --  displacements than the fit has coefficients.
                           Textured (Cell) := Driver.Stats.Median (Cond.all) > 0.0 and then Here > Kept + 1;
                           Free (Cond);
                           if not Textured (Cell) then
                              S.Noise.Append (Real'Last);
                              S.Gains.Append (0.0, Ada.Containers.Count_Type (Kept));
                              S.Gain_Variances.Append (0.0, Ada.Containers.Count_Type (Kept));
                              S.Shifts.Append (0.0, Ada.Containers.Count_Type (Kept));
                              return;
                           end if;
                           Fit_Resolved (Cell, Here);
                        end Fit_Cell;
                     begin
                        for R in 1 .. Rows loop
                           X (R, 1) := 1.0;
                           for K in 1 .. Kept loop
                              X (R, K + 1) := Pushed_Change (Cols (K).Group, Beat_Of (R) - Lag, Cols (K).Channel);
                           end loop;
                        end loop;
                        for Cell in 1 .. N loop
                           Fit_Cell (Cell);
                        end loop;
                        for Cell in 1 .. N loop
                           S.Textured.Append (Textured (Cell));
                        end loop;
                        --  The per-cell test's own false-alarm rate.
                        declare
                           P0 : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
                           T  : Natural := 0;
                        begin
                           for Cell in 1 .. N loop
                              if Textured (Cell) then
                                 T := T + 1;
                              end if;
                           end loop;
                           for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                              declare
                                 Tested : Boolean := False;
                                 Count  : Natural := 0;
                                 Effect : Eye_Effect;
                              begin
                                 for K in 1 .. Kept loop
                                    Tested := Tested or else Cols (K).Group = G;
                                 end loop;
                                 if Tested and then T > 0 then
                                    for Cell in 1 .. N loop
                                       if Textured (Cell) and then Responding (Cell, G) then
                                          Count := Count + 1;
                                       end if;
                                    end loop;
                                    declare
                                       F    : constant Real := Real (Count) / Real (T);
                                       --  The cells that show the motion, at least and at most:
                                       --  those that respond, and the share of the cells too noisy
                                       --  to tell that show it together.
                                       Least, Most : Natural;
                                       --  An eye mostly sees the world: a whole image moves when
                                       --  a significant majority of what can move does.
                                       Half : constant Real := 0.5;
                                    begin
                                       Cells_Showing (G, Textured.all, Responding.all, Energy.all, Dof.all, S.Noise, Least, Most);
                                       Effect.Responding := Count;
                                       Effect.Textured := T;
                                       Effect.Fraction :=
                                         (Value => F, Sigma => Sqrt (F * (1.0 - F) / Real (T)), Degrees_Of_Freedom => 0);
                                       if not Regression.Count_Significant (Count, T, P0) then
                                          Effect.Verdict := Nothing;
                                       elsif Regression.Count_Significant (Least, T, Half) then
                                          Effect.Verdict := Whole;
                                       elsif Regression.Count_Significant (T - Most, T, Half) then
                                          Effect.Verdict := Patch;
                                       else
                                          Effect.Verdict := Undecided;
                                       end if;
                                    end;
                                 end if;
                                 M.Graph.Effects.Replace_Element ((Natural (G) - 1) * Eyes + Natural (E), Effect);
                              end;
                           end loop;
                        end;
                        Free (X);
                        Free (Responding);
                        Free (Energy);
                        Free (Dof);
                        Free (Textured);
                     end;
                  end if;
                  Free (Beat_Of);
               end;
            end if;
         end;
      end loop;
   end Measure;

   procedure Measure_Rest_Noise (M : in out Model) is
   begin
      for S of M.Eyes loop
         declare
            N     : constant Natural := Cells (S.Grid);
            Beats : constant Natural := Natural'Min (Natural (S.Judged.Length), Natural (S.Measured.Length));
            Count : Natural := 0;
         begin
            if N > 0 and then Natural (S.Luma_Variance.Length) >= N then
               for B in 1 .. Beats - 1 loop
                  if S.Measured (B) and then S.Judged (B) and then S.Still_At (B) and then S.Judged (B - 1)
                    and then S.Still_At (B - 1)
                  then
                     for C in 0 .. N - 1 loop
                        if S.Condition (B * N + C) > 0.0 then
                           Count := Count + 1;
                        end if;
                     end loop;
                  end if;
               end loop;
               if Count > 0 then
                  declare
                     --  One ratio per still cell and beat: on the heap, a long
                     --  stream has millions.
                     Ratios : Real_Access := new Real_Array (1 .. Count);
                     K      : Natural := 0;
                  begin
                     for B in 1 .. Beats - 1 loop
                        if S.Measured (B) and then S.Judged (B) and then S.Still_At (B) and then S.Judged (B - 1)
                          and then S.Still_At (B - 1)
                        then
                           for C in 0 .. N - 1 loop
                              if S.Condition (B * N + C) > 0.0 then
                                 K := K + 1;
                                 Ratios (K) := (S.Du (B * N + C) ** 2 + S.Dv (B * N + C) ** 2)
                                   / Flow.Noise_Floor (S.Condition (B * N + C), S.Luma_Variance (C)) ** 2;
                              end if;
                           end loop;
                        end if;
                     end loop;
                     S.Rest_Factor := Real'Max
                       (1.0, Sqrt (Driver.Stats.Median (Ratios.all) / Driver.Distributions.Chi_Square_Quantile (0.5, 2)));
                     Free (Ratios);
                  end;
               end if;
            end if;
         end;
      end loop;
      --  How many cells an eye counts as moved when no group is pushed: its own
      --  fingers' jitter, the scene, the rest of the body's noise.
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S    : Eye_Stream renames M.Eyes (E);
            --  An eye whose lag is not measured may show a push as late as
            --  any lag the stream can tell.
            Lag  : constant Natural :=
              (if E <= M.Lag_Known.Last_Index and then M.Lag_Known (E)
               then Natural (Integer'Max (0, M.Lags (E)))
               else Driver.Robot.Lag.Longest (M));
            Rest : Natural := 0;

            function At_Rest (B : Natural) return Boolean is
            begin
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  for K in Integer'Max (0, B - Lag) .. B loop
                     if M.Groups (G).Commandable and then Channels.Pushed (M, G, K) then
                        return False;
                     end if;
                  end loop;
               end loop;
               return B < Natural (S.Measured.Length) and then S.Measured (B);
            end At_Rest;
         begin
            for B in 1 .. M.Beats - 1 loop
               if At_Rest (B) then
                  Rest := Rest + 1;
               end if;
            end loop;
            S.Rest_Counts_Known := Rest > 1;
            if S.Rest_Counts_Known then
               declare
                  Count, Tested : Natural;
               begin
                  S.Rest_Count_Max := 0;
                  for B in 1 .. M.Beats - 1 loop
                     if At_Rest (B) then
                        Count_Moved (S, B, Count, Tested);
                        S.Rest_Count_Max := Natural'Max (S.Rest_Count_Max, Count);
                     end if;
                  end loop;
                  S.Rest_Count_Beats := Rest;
               end;
            end if;
         end;
      end loop;
   end Measure_Rest_Noise;

   --  How many textured cells moved beyond their noise at rest, or too far
   --  to be resolved, at the beat; how many were tested.
   procedure Count_Moved (S : Eye_Stream; Beat : Natural; Count, Tested : out Natural) is
      N    : constant Natural := Cells (S.Grid);
      Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
   begin
      Count := 0;
      Tested := 0;
      if N = 0 or else Beat >= Natural (S.Measured.Length) or else not S.Measured (Beat)
        or else Natural (S.Luma_Variance.Length) < N
      then
         return;
      end if;
      for C in 0 .. N - 1 loop
         declare
            Cond : constant Real := S.Condition (Beat * N + C);
         begin
            if Cond > 0.0 then
               Tested := Tested + 1;
               if not S.Resolved (Beat * N + C)
                 or else Driver.Uncertain.Significant
                   (Gate, Sqrt (S.Du (Beat * N + C) ** 2 + S.Dv (Beat * N + C) ** 2),
                    --  The cell's displacement noise at rest, as the eye's still
                    --  frames measured it.
                    S.Rest_Factor * Flow.Noise_Floor (Cond, S.Luma_Variance (C)))
               then
                  Count := Count + 1;
               end if;
            end if;
         end;
      end loop;
   end Count_Moved;

   function Moved (M : Model; E : Eye_Id; Beat : Natural) return Boolean is
      S : Eye_Stream renames M.Eyes (E);
      Count, Tested : Natural;
   begin
      Count_Moved (S, Beat, Count, Tested);
      if Tested = 0 then
         return False;
      end if;
      --  More than at any beat nobody pushed (its own fingers' jitter, the
      --  scene): a count at rest beats all n of them with chance 1 / (n + 1),
      --  whatever their distribution. Before that is measured, against the
      --  false alarms of the per-cell test alone.
      return (if S.Rest_Counts_Known
              then Count > S.Rest_Count_Max
              else Regression.Count_Significant
                     (Count, Tested, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)));
   end Moved;

   function Cell_Noise (M : Model; E : Eye_Id) return Real is
      S     : Eye_Stream renames M.Eyes (E);
      Count : Natural := 0;
   begin
      for X of S.Noise loop
         if X < Real'Last then
            Count := Count + 1;
         end if;
      end loop;
      if Count = 0 then
         return Real'Last;
      end if;
      declare
         Values : Real_Access := new Real_Array (1 .. Count);
         K      : Natural := 0;
      begin
         for X of S.Noise loop
            if X < Real'Last then
               K := K + 1;
               Values (K) := X;
            end if;
         end loop;
         return Result : constant Real := Driver.Stats.Median (Values.all) do
            Free (Values);
         end return;
      end;
   end Cell_Noise;

   function Shift (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Real is
      S      : Eye_Stream renames M.Eyes (E);
      Kept   : constant Natural := Natural (S.Kept_Groups.Length);
      Column : Natural := 0;
      Count  : Natural := 0;
   begin
      for K in 0 .. Kept - 1 loop
         if S.Kept_Groups (K) = Natural (G) and then S.Kept_Channels (K) = Channel then
            Column := K + 1;
         end if;
      end loop;
      if Column = 0 then
         return 0.0;
      end if;
      for Cell in 0 .. Natural (S.Shifts.Length) / Kept - 1 loop
         if S.Shifts (Cell * Kept + Column - 1) > 0.0 then
            Count := Count + 1;
         end if;
      end loop;
      return (if Count = 0 then 0.0 else Median_Shift (S, Kept, Column, Count));
   end Shift;

end Driver.Robot.Lockin;
