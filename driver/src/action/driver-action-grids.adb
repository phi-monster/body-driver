package body Driver.Action.Grids is

   use Driver.Numerics.Arrays;

   function Key_Of (G : Grid; P : Vec3) return Key is
     ((I => Integer (Real'Floor (P (1) / G.Size)), J => Integer (Real'Floor (P (2) / G.Size)),
       K => Integer (Real'Floor (P (3) / G.Size))));

   procedure Start (G : out Grid; Cube : Real) is
   begin
      G.Size := Cube;
      G.Cubes.Clear;
   end Start;

   function Cube (G : Grid) return Real is (G.Size);

   procedure Add (G : in out Grid; Point : Vec3; Margin : Real) is
      K   : constant Key := Key_Of (G, Point);
      Pos : constant Cube_Maps.Cursor := G.Cubes.Find (K);
   begin
      if Cube_Maps.Has_Element (Pos) then
         G.Cubes.Reference (Pos).Append (Entry_Point'(Point => Point, Margin => Margin));
      else
         G.Cubes.Insert (K, Entry_Vectors.To_Vector (Entry_Point'(Point => Point, Margin => Margin), 1));
      end if;
   end Add;

   function Least_Gap (G : Grid; Q : Vec3) return Real is
      Least : Real := Real'Last;
      C     : constant Key := Key_Of (G, Q);
   begin
      for DI in -1 .. 1 loop
         for DJ in -1 .. 1 loop
            for DK in -1 .. 1 loop
               declare
                  Pos : constant Cube_Maps.Cursor := G.Cubes.Find ((I => C.I + DI, J => C.J + DJ, K => C.K + DK));
               begin
                  if Cube_Maps.Has_Element (Pos) then
                     for E of G.Cubes.Constant_Reference (Pos) loop
                        Least := Real'Min (Least, abs (Q - E.Point) - E.Margin);
                     end loop;
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      return Least;
   end Least_Gap;

end Driver.Action.Grids;
