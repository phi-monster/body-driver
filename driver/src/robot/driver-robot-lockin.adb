with Ada.Containers;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Regression;
with Driver.Stats;

package body Driver.Robot.Lockin is

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
      Values : Real_Array (1 .. Count);
      K      : Natural := 0;
   begin
      for Cell in 0 .. Natural (S.Shifts.Length) / Kept - 1 loop
         if S.Shifts (Cell * Kept + Column - 1) > 0.0 then
            K := K + 1;
            Values (K) := S.Shifts (Cell * Kept + Column - 1);
         end if;
      end loop;
      return Driver.Stats.Median (Values);
   end Median_Shift;

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
                  Beat_Of : array (1 .. Rows) of Natural;
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
                        X : Real_Matrix (1 .. Rows, 1 .. Kept + 1);
                        Responding : array (1 .. N, M.Groups.First_Index .. M.Groups.Last_Index) of Boolean :=
                          [others => [others => False]];
                        Textured : array (1 .. N) of Boolean := [others => False];
                        --  One cell: its displacements regressed on the pushes, over
                        --  the beats where the cell resolved one, and for every
                        --  group whether its block responds.
                        procedure Fit_Resolved (Cell : Positive; Here : Positive) is
                           Xc     : Real_Matrix (1 .. Here, 1 .. Kept + 1);
                           U, V   : Real_Array (1 .. Here);
                           Floors : Real_Array (1 .. Here);
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
                              Floor : constant Real := Driver.Stats.Median (Floors);
                              Fu    : constant Regression.Fit := Regression.Solve (Xc, U, Floor);
                              Fv    : constant Regression.Fit := Regression.Solve (Xc, V, Floor);
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
                        end Fit_Resolved;

                        procedure Fit_Cell (Cell : Positive) is
                           Cond : Real_Array (1 .. Rows);
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
                           Textured (Cell) := Driver.Stats.Median (Cond) > 0.0 and then Here > Kept + 1;
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
                                       --  An eye mostly sees the world: a whole image moves when
                                       --  a significant majority of what can move does.
                                       Half : constant Real := 0.5;
                                    begin
                                       Effect.Responding := Count;
                                       Effect.Textured := T;
                                       Effect.Fraction :=
                                         (Value => F, Sigma => Sqrt (F * (1.0 - F) / Real (T)), Degrees_Of_Freedom => 0);
                                       if not Regression.Count_Significant (Count, T, P0) then
                                          Effect.Verdict := Nothing;
                                       elsif Regression.Count_Significant (Count, T, Half) then
                                          Effect.Verdict := Whole;
                                       elsif Regression.Count_Significant (T - Count, T, Half) then
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
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Measure;

   function Moved (M : Model; E : Eye_Id; Beat : Natural) return Boolean is
      S       : Eye_Stream renames M.Eyes (E);
      N       : constant Natural := Cells (S.Grid);
      Gate    : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
      Count   : Natural := 0;
      Tested  : Natural := 0;
   begin
      if N = 0 or else Beat >= Natural (S.Measured.Length) or else not S.Measured (Beat)
        or else Natural (S.Luma_Variance.Length) < N
      then
         return False;
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
                    --  The cell's displacement noise as the last lock-in measured
                    --  it, never below what its pixels' noise allows.
                    Real'Max (Flow.Noise_Floor (Cond, S.Luma_Variance (C)),
                              (if C < Natural (S.Noise.Length) and then S.Noise.Element (C) < Real'Last
                               then S.Noise.Element (C) else 0.0)))
               then
                  Count := Count + 1;
               end if;
            end if;
         end;
      end loop;
      return Tested > 0
        and then Regression.Count_Significant
          (Count, Tested, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));
   end Moved;

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
