with Driver.Tests;

package body Driver.World.Regions.Tests is

   use Driver.Images;
   use Driver.Tests;

   function Rectangle (C0, R0, C1, R1 : Natural) return Mask is
      M : Mask := Create (60, 40);
   begin
      for R in R0 .. R1 loop
         for C in C0 .. C1 loop
            Include (M, C, R);
         end loop;
      end loop;
      return M;
   end Rectangle;

   procedure Bent_Region is
      --  A C open to the right: its centroid lies in the opening, outside it.
      C_Shape : Mask := Rectangle (10, 5, 15, 34);
      P       : Pixel;
   begin
      for R in 5 .. 34 loop
         for C in 16 .. 40 loop
            if R <= 10 or else R >= 29 then
               Include (C_Shape, C, R);
            end if;
         end loop;
      end loop;
      P := Inner_Point (C_Shape);
      Check (Contains (C_Shape, Natural (Real'Floor (P.U)), Natural (Real'Floor (P.V))),
             "the inner point of a bent region is outside it");
      P := Inner_Point (Rectangle (10, 10, 30, 20));
      Check (P.U = 20.5 and then P.V = 15.5, "the inner point of a rectangle is not its centre");
      Check (Bounds (C_Shape) = (Column_0 => 10, Row_0 => 5, Column_1 => 40, Row_1 => 34), "the bounds are wrong");
   end Bent_Region;

   procedure Same_Or_Not is
      A : constant Mask := Rectangle (10, 10, 30, 20);
   begin
      Check (Same_Pixels (A, Rectangle (12, 11, 32, 21)), "a region two pixels over is not the same");
      Check (not Same_Pixels (A, Rectangle (35, 10, 50, 20)), "a region beside it is the same");
      --  A thing and the table it lies on, which the thing hides where it lies.
      declare
         Table : Mask := Rectangle (0, 0, 59, 39);
      begin
         for R in 10 .. 20 loop
            for C in 10 .. 30 loop
               Include (Table, C, R, False);
            end loop;
         end loop;
         Check (not Same_Pixels (A, Table), "a thing and the table around it are the same");
      end;
      Check (Radius (A) = 6.0, "the radius of a rectangle eleven rows high is not six");
      Check (not Same_Pixels (A, Create (60, 40)) and then not Same_Pixels (A, Create (61, 40)),
             "an empty region or one of another size is the same");
      Check (Overlap (A, Rectangle (25, 15, 40, 30)) = 6 * 6, "the overlap is not the pixels in both");
   end Same_Or_Not;

   procedure Register is
   begin
      Driver.Tests.Register ("world.regions.inner", "a region's own point lies outside it, or bounds are off",
                             Bent_Region'Access);
      Driver.Tests.Register ("world.regions.same", "two regions are judged the same pixels wrongly", Same_Or_Not'Access);
   end Register;

end Driver.World.Regions.Tests;
