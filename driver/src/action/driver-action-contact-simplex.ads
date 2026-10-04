--  Linear programs in standard form: minimize Cost * X subject to A * X = B
--  and X >= 0.
--
--  A dense two-phase simplex method. The column that improves the objective
--  most enters (Dantzig's rule) whenever its step moves the solution; a pivot
--  that would not move it is chosen by Bland's rule instead. Only pivots that
--  do not move can make a cycle, Bland's rule never cycles, so it ends
--  without any cap on its iterations. Rows are scaled to unit size before
--  solving, and an entry counts as zero below the square root of the machine
--  epsilon: the point where cancellation in the eliminations leaves no
--  trustworthy digit. Nothing else is chosen.

package Driver.Action.Contact.Simplex is

   type Status is (Optimal, Infeasible, Unbounded);

   type Solution (Columns : Natural) is record
      Result : Status := Infeasible;
      Value  : Real := Real'Last;               --  the minimum, for Optimal
      X      : Real_Vector (1 .. Columns) := [others => 0.0];
   end record;

   function Minimize (A : Real_Matrix; B : Real_Vector; Cost : Real_Vector) return Solution
     with Pre => A'Length (1) = B'Length and then A'Length (2) = Cost'Length and then A'Length (2) > 0
                 and then A'Length (1) > 0,
          Post => Minimize'Result.Columns = A'Length (2);

end Driver.Action.Contact.Simplex;
