with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;

package body Driver.Robot.Flow is

   use Ada.Numerics.Long_Elementary_Functions;

   --  Everything sized by pixels lives on the heap: no stack needs to hold
   --  a cell of any camera.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   function Grid_Of (Width, Height : Natural) return Cell_Grid is
     ((Width   => Width,
       Height  => Height,
       Columns => (if Width = 0 then 0 else Natural (Real'Floor (Sqrt (Real (Width))))),
       Rows    => (if Height = 0 then 0 else Natural (Real'Floor (Sqrt (Real (Height)))))));

   procedure Bounds (G : Cell_Grid; Cell : Positive; X0, X1, Y0, Y1 : out Natural) is
      Column : constant Natural := (Cell - 1) mod G.Columns;
      Row    : constant Natural := (Cell - 1) / G.Columns;
   begin
      X0 := Column * G.Width / G.Columns;
      X1 := (Column + 1) * G.Width / G.Columns;
      Y0 := Row * G.Height / G.Rows;
      Y1 := (Row + 1) * G.Height / G.Rows;
   end Bounds;

   procedure Displacements
     (G             : Cell_Grid;
      Before, After : Real_Array;
      Luma_Variance : Real_Array;
      Du, Dv        : out Real_Array;
      Condition     : out Real_Array;
      Resolved      : out Flag_Array)
   is
      W  : constant Natural := G.Width;
      H  : constant Natural := G.Height;
      B0 : constant Integer := Before'First;
      A0 : constant Integer := After'First;

      function At_Before (X, Y : Natural) return Real is (Before (B0 + Y * W + X));

      --  The floor of a coordinate the clamp below left at 0 or more: a
      --  conversion rounds to the nearest whole number, one too many when it
      --  rounded up. Equal to Real'Floor there, without its call.
      function Floor_Of (X : Real) return Natural is
         N : Natural := Natural (X);
      begin
         if Real (N) > X then
            N := N - 1;
         end if;
         return N;
      end Floor_Of;

      --  Where a column (or row) of the cell lands in After when the content
      --  moved by U (or V): the coordinate clamped to the image, the pixel
      --  before it (the last but one at the far border) and the fraction
      --  beyond that pixel, for After bilinearly interpolated there;
      --  positions beyond the image take the nearest border pixel. The same
      --  for every pixel of a column in one pass, so worked out once a pass.
      procedure Place (Coordinate : Real; Last : Natural; Pixel : out Natural; Fraction : out Real) is
         C : constant Real := Real'Max (0.0, Real'Min (Real (Last), Coordinate));
      begin
         Pixel := Natural'Min (Floor_Of (C), Last - 1);
         Fraction := C - Real (Pixel);
      end Place;

      --  The largest cell, for the gradients and the placements one cell at a
      --  time: every cell's bounds differ from it by at most one pixel.
      Most_X : constant Natural := (if G.Columns = 0 then 0 else W / G.Columns + 1);
      Most_Y : constant Natural := (if G.Rows = 0 then 0 else H / G.Rows + 1);
      Gx     : Real_Access := new Real_Array (1 .. Natural'Max (1, Most_X * Most_Y));
      Gy     : Real_Access := new Real_Array (1 .. Natural'Max (1, Most_X * Most_Y));
      type Index_Array is array (Natural range <>) of Natural;
      type Index_Access is access Index_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);
      Px     : Index_Access := new Index_Array (0 .. Natural'Max (1, Most_X));
      Py     : Index_Access := new Index_Array (0 .. Natural'Max (1, Most_Y));
      Fx     : Real_Access := new Real_Array (1 .. Natural'Max (1, Most_X) + 1);
      Fy     : Real_Access := new Real_Array (1 .. Natural'Max (1, Most_Y) + 1);
   begin
      for Cell in 1 .. Cells (G) loop
         declare
            X0, X1, Y0, Y1 : Natural;
            Sxx, Sxy, Syy : Real := 0.0;
            K : constant Integer := Du'First + Cell - 1;
         begin
            Bounds (G, Cell, X0, X1, Y0, Y1);
            --  Central differences need a neighbour on each side.
            X0 := Natural'Max (X0, 1);
            Y0 := Natural'Max (Y0, 1);
            X1 := Natural'Min (X1, W - 1);
            Y1 := Natural'Min (Y1, H - 1);
            declare
               Nx : constant Integer := X1 - X0;
               Ny : constant Integer := Y1 - Y0;
            begin
               Du (K) := 0.0;
               Dv (K) := 0.0;
               Condition (Condition'First + Cell - 1) := 0.0;
               Resolved (Resolved'First + Cell - 1) := False;
               if Nx > 0 and then Ny > 0 then
                  for Y in Y0 .. Y1 - 1 loop
                     for X in X0 .. X1 - 1 loop
                        declare
                           I : constant Positive := (Y - Y0) * Nx + (X - X0) + 1;
                        begin
                           Gx (I) := (At_Before (X + 1, Y) - At_Before (X - 1, Y)) / 2.0;
                           Gy (I) := (At_Before (X, Y + 1) - At_Before (X, Y - 1)) / 2.0;
                           Sxx := Sxx + Gx (I) * Gx (I);
                           Sxy := Sxy + Gx (I) * Gy (I);
                           Syy := Syy + Gy (I) * Gy (I);
                        end;
                     end loop;
                  end loop;
                  declare
                     Det   : constant Real := Sxx * Syy - Sxy * Sxy;
                     Trace : constant Real := Sxx + Syy;
                     U, V  : Real := 0.0;
                     Converged : Boolean := False;
                     Last_Step : Real := Real'Last;   --  the step before this one
                     Final     : Real := Real'Last;   --  the last step taken
                  begin
                     if Det > 0.0 then
                        Condition (Condition'First + Cell - 1) :=
                          (Trace - Sqrt (Real'Max (0.0, Trace * Trace - 4.0 * Det))) / 2.0;
                        --  Gauss-Newton on the translation, with the gradient of
                        --  the first frame, until a step no longer changes it;
                        --  one pass per pixel bounds a step count never reached.
                        for Pass in 1 .. Nx * Ny loop
                           declare
                              Bx, By : Real := 0.0;
                              Su, Sv : Real;
                           begin
                              for X in X0 .. X1 - 1 loop
                                 Place (Real (X) + U, W - 1, Px (X - X0), Fx (X - X0 + 1));
                              end loop;
                              for Y in Y0 .. Y1 - 1 loop
                                 Place (Real (Y) + V, H - 1, Py (Y - Y0), Fy (Y - Y0 + 1));
                              end loop;
                              for Y in Y0 .. Y1 - 1 loop
                                 declare
                                    Wy  : constant Real := Fy (Y - Y0 + 1);
                                    Row : constant Integer := A0 + Py (Y - Y0) * W;
                                 begin
                                    for X in X0 .. X1 - 1 loop
                                       declare
                                          I   : constant Positive := (Y - Y0) * Nx + (X - X0) + 1;
                                          Wx  : constant Real := Fx (X - X0 + 1);
                                          P00 : constant Integer := Row + Px (X - X0);
                                          --  After at the moved place, as At_After gave it.
                                          A   : constant Real :=
                                            (1.0 - Wy) * ((1.0 - Wx) * After (P00) + Wx * After (P00 + 1))
                                            + Wy * ((1.0 - Wx) * After (P00 + W) + Wx * After (P00 + W + 1));
                                          E   : constant Real := A - At_Before (X, Y);
                                       begin
                                          Bx := Bx + Gx (I) * E;
                                          By := By + Gy (I) * E;
                                       end;
                                    end loop;
                                 end;
                              end loop;
                              Su := -(Syy * Bx - Sxy * By) / Det;
                              Sv := -(Sxx * By - Sxy * Bx) / Det;
                              U := U + Su;
                              V := V + Sv;
                              Final := Sqrt (Su * Su + Sv * Sv);
                              --  Done when a step no longer changes the estimate,
                              --  or is below what the cell can resolve at all.
                              Converged := Sqrt (Su * Su + Sv * Sv)
                                <= Real'Max (Driver.Conventions.Unchanged_Fraction * Sqrt (U * U + V * V),
                                             Noise_Floor (Condition (Condition'First + Cell - 1),
                                                          Luma_Variance (Luma_Variance'First + Cell - 1)));
                              exit when Converged;
                              --  A step no smaller than the last one is not closing in on
                              --  anything, and a translation past half the cell cannot be
                              --  resolved: either way the cell is given up.
                              exit when Sqrt (Su * Su + Sv * Sv) >= Last_Step
                                or else abs U > Real (Nx) / 2.0 or else abs V > Real (Ny) / 2.0;
                              Last_Step := Sqrt (Su * Su + Sv * Sv);
                           end;
                        end loop;
                        Du (K) := U;
                        Dv (K) := V;
                        --  A translation is resolved when the iteration settled on
                        --  it, or its last step was no larger than its noise lets a
                        --  step be (a cell at rest steps by noise and stops shrinking
                        --  there), and the content moved less than half the cell:
                        --  beyond that most of the template has left the window it is
                        --  matched in, and the answer is whatever fits best.
                        Resolved (Resolved'First + Cell - 1) :=
                          (Converged
                           or else Driver.Uncertain.Significant
                                     (Driver.Uncertain.Vector_Gate (2), Final,
                                      Noise_Floor (Condition (Condition'First + Cell - 1),
                                                   Luma_Variance (Luma_Variance'First + Cell - 1))) = False)
                          and then abs U <= Real (Nx) / 2.0 and then abs V <= Real (Ny) / 2.0;
                     end if;
                  end;
               end if;
            end;
         end;
      end loop;
      Free (Gx);
      Free (Gy);
      Free (Px);
      Free (Py);
      Free (Fx);
      Free (Fy);
   end Displacements;

   function Noise_Floor (Condition, Luma_Variance : Real) return Real is
     (if Condition > 0.0 then Sqrt (2.0 * Luma_Variance / Condition) else Real'Last);

end Driver.Robot.Flow;
