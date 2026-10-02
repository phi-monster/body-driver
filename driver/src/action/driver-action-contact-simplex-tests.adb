with Driver.Tests;

package body Driver.Action.Contact.Simplex.Tests is

   use Driver.Tests;

   procedure Known_Optimum is
      --  min x1 + x2 + 3 x3 with x1 + 2 x2 + x3 = 4, 3 x1 + x2 + x3 = 6: the
      --  optimum is x = (1.6, 1.2, 0), value 2.8.
      S : constant Solution := Minimize ([[1.0, 2.0, 1.0], [3.0, 1.0, 1.0]], [4.0, 6.0], [1.0, 1.0, 3.0]);
   begin
      Check (S.Result = Optimal, "a feasible bounded program reported " & S.Result'Image);
      Check_Close (S.Value, 2.8, 1.0e-12, "optimal value");
      Check_Close (S.X (1), 1.6, 1.0e-12, "x1");
      Check_Close (S.X (2), 1.2, 1.0e-12, "x2");
      Check_Close (S.X (3), 0.0, 1.0e-12, "x3 stays out of the basis");
   end Known_Optimum;

   procedure Infeasible_Is_Said is
      --  x1 + x2 = -1 has no solution with x >= 0.
      S : constant Solution := Minimize ([[1.0, 1.0]], [-1.0], [1.0, 1.0]);
   begin
      Check (S.Result = Infeasible, "an impossible program reported " & S.Result'Image);
   end Infeasible_Is_Said;

   procedure Redundant_Rows is
      --  The same equality twice and a combination of the two: still the
      --  optimum of the single row, min 2 x1 + x2 with x1 + x2 = 1, x = (0, 1).
      S : constant Solution :=
        Minimize ([[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]], [1.0, 2.0, 3.0], [2.0, 1.0]);
   begin
      Check (S.Result = Optimal, "redundant rows made it " & S.Result'Image);
      Check_Close (S.Value, 1.0, 1.0e-12, "value with redundant rows");
   end Redundant_Rows;

   procedure Degenerate_Terminates is
      --  Beale's example, on which the most-negative-cost rule cycles for ever:
      --  Bland's rule reaches its optimum -1.25.
      S : constant Solution :=
        Minimize ([[1.0, 0.0, 0.0, 0.25, -8.0, -1.0, 9.0],
                   [0.0, 1.0, 0.0, 0.5, -12.0, -0.5, 3.0],
                   [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0]],
                  [0.0, 0.0, 1.0],
                  [0.0, 0.0, 0.0, -0.75, 20.0, -0.5, 6.0]);
   begin
      Check (S.Result = Optimal, "Beale's program reported " & S.Result'Image);
      Check_Close (S.Value, -1.25, 1.0e-12, "Beale's optimum");
   end Degenerate_Terminates;

   procedure Register is
   begin
      Driver.Tests.Register ("action.simplex.optimum", "the linear program misses the optimum of a small program",
                             Known_Optimum'Access);
      Driver.Tests.Register ("action.simplex.infeasible", "an impossible balance of forces is reported as possible",
                             Infeasible_Is_Said'Access);
      Driver.Tests.Register ("action.simplex.redundant", "dependent equalities make a possible balance look impossible",
                             Redundant_Rows'Access);
      Driver.Tests.Register ("action.simplex.degenerate", "a degenerate program cycles or stops short of its optimum",
                             Degenerate_Terminates'Access);
   end Register;

end Driver.Action.Contact.Simplex.Tests;
