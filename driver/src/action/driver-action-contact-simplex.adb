with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Action.Contact.Simplex is

   use Ada.Numerics.Long_Elementary_Functions;

   Zero : constant Real := Sqrt (Real'Model_Epsilon);

   function Minimize (A : Real_Matrix; B : Real_Vector; Cost : Real_Vector) return Solution is
      M  : constant Positive := A'Length (1);
      N  : constant Positive := A'Length (2);
      --  Inputs may be indexed from anywhere (a positional aggregate starts
      --  at Integer'First), so they are read by offset from their first index.
      function In_A (I, J : Positive) return Real is (A (A'First (1) + (I - 1), A'First (2) + (J - 1)));
      function In_B (I : Positive) return Real is (B (B'First + (I - 1)));
      function In_Cost (J : Positive) return Real is (Cost (Cost'First + (J - 1)));
      W  : constant Positive := N + M + 1;          --  variables, artificials, right-hand side
      T  : Real_Matrix (0 .. M, 1 .. W) := [others => [others => 0.0]];
      Basis : array (1 .. M) of Positive;
      S  : Solution (N);

      --  A basic variable that counts as zero is zero. The eliminations leave
      --  one that is zero in exact arithmetic as noise of either sign, and
      --  noise ties with nothing (Ratio_Test).
      procedure Settle is
      begin
         for I in 1 .. M loop
            if abs T (I, W) <= Zero then
               T (I, W) := 0.0;
            end if;
         end loop;
      end Settle;

      procedure Pivot (Row, Col : Positive) is
         P : constant Real := T (Row, Col);
      begin
         for J in 1 .. W loop
            T (Row, J) := T (Row, J) / P;
         end loop;
         for I in 0 .. M loop
            if I /= Row and then T (I, Col) /= 0.0 then
               declare
                  F : constant Real := T (I, Col);
               begin
                  for J in 1 .. W loop
                     T (I, J) := T (I, J) - F * T (Row, J);
                  end loop;
               end;
            end if;
         end loop;
         Basis (Row) := Col;
         Settle;
         S.Pivots := S.Pivots + 1;
      end Pivot;

      --  The row that leaves when column Enter enters: among rows with the
      --  least ratio, the one whose basic column is lowest; 0 when no row
      --  bounds it. Step is that least ratio. The rows at a degenerate vertex
      --  have a right-hand side of exactly zero (Settle), so their ratios are
      --  exactly zero and tie exactly.
      procedure Ratio_Test (Enter : Positive; Leave : out Natural; Step : out Real) is
      begin
         Leave := 0;
         Step := Real'Last;
         for I in 1 .. M loop
            if T (I, Enter) > Zero then
               declare
                  Ratio : constant Real := T (I, W) / T (I, Enter);
               begin
                  if Leave = 0 or else Ratio < Step or else (Ratio = Step and then Basis (I) < Basis (Leave)) then
                     Step := Ratio;
                     Leave := I;
                  end if;
               end;
            end if;
         end loop;
      end Ratio_Test;

      --  Pivots until no column below Allowed improves the objective (row 0).
      --  The column that improves it most per unit enters (Dantzig's rule)
      --  whenever its step moves the solution; when that step is zero, the
      --  pivot is chosen by Bland's rule instead: the lowest improving column
      --  enters. Every pivot of a cycle leaves the objective as it is, so all
      --  of them would be Bland's, and Bland's rule never cycles.
      procedure Run (Allowed : Positive; Bounded : out Boolean) is
      begin
         Bounded := True;
         loop
            declare
               Enter : Natural := 0;
               Leave : Natural := 0;
               Step  : Real;
            begin
               for J in 1 .. Allowed loop
                  if T (0, J) < -Zero and then (Enter = 0 or else T (0, J) < T (0, Enter)) then
                     Enter := J;
                  end if;
               end loop;
               exit when Enter = 0;
               Ratio_Test (Enter, Leave, Step);
               if Leave /= 0 and then Step <= Zero then
                  for J in 1 .. Allowed loop
                     if T (0, J) < -Zero then
                        Enter := J;
                        exit;
                     end if;
                  end loop;
                  Ratio_Test (Enter, Leave, Step);
               end if;
               if Leave = 0 then
                  Bounded := False;
                  return;
               end if;
               Pivot (Leave, Enter);
            end;
         end loop;
      end Run;

      Bounded  : Boolean;
      Residual : Real := 0.0;
   begin
      --  Rows scaled to unit size and signed so the right-hand side is not
      --  negative; the solution does not change, and Zero means the same in
      --  every row.
      for I in 1 .. M loop
         declare
            Size : Real := abs In_B (I);
         begin
            for J in 1 .. N loop
               Size := Real'Max (Size, abs In_A (I, J));
            end loop;
            if Size = 0.0 then
               Size := 1.0;
            end if;
            if In_B (I) < 0.0 then
               Size := -Size;
            end if;
            for J in 1 .. N loop
               T (I, J) := In_A (I, J) / Size;
            end loop;
            T (I, N + I) := 1.0;
            T (I, W) := In_B (I) / Size;
            Basis (I) := N + I;
         end;
      end loop;
      Settle;
      --  Phase one: minimize the sum of the artificial variables.
      for I in 1 .. M loop
         for J in 1 .. N loop
            T (0, J) := T (0, J) - T (I, J);
         end loop;
         T (0, W) := T (0, W) - T (I, W);
      end loop;
      Run (N + M, Bounded);
      for I in 1 .. M loop
         if Basis (I) > N then
            Residual := Residual + T (I, W);
         end if;
      end loop;
      if Residual > Zero * Real (M) then
         return S;
      end if;
      --  Artificial variables still basic at zero leave the basis; a row with
      --  nothing left among the real columns is redundant and stays as it is.
      for I in 1 .. M loop
         if Basis (I) > N then
            for J in 1 .. N loop
               if abs T (I, J) > Zero then
                  Pivot (I, J);
                  exit;
               end if;
            end loop;
         end if;
      end loop;
      --  Phase two: the real costs, artificial columns barred from entering.
      for J in 1 .. W loop
         T (0, J) := (if J <= N then In_Cost (J) else 0.0);
      end loop;
      for I in 1 .. M loop
         if Basis (I) <= N and then T (0, Basis (I)) /= 0.0 then
            declare
               C : constant Real := T (0, Basis (I));
            begin
               for J in 1 .. W loop
                  T (0, J) := T (0, J) - C * T (I, J);
               end loop;
            end;
         end if;
      end loop;
      Run (N, Bounded);
      if not Bounded then
         S.Result := Unbounded;
         S.Value := Real'First;
         return S;
      end if;
      for I in 1 .. M loop
         if Basis (I) <= N then
            S.X (Basis (I)) := T (I, W);
         end if;
      end loop;
      S.Result := Optimal;
      S.Value := -T (0, W);
      return S;
   end Minimize;

end Driver.Action.Contact.Simplex;
