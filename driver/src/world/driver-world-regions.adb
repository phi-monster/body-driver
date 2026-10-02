with Ada.Numerics.Long_Elementary_Functions;

package body Driver.World.Regions is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;

   function Same_Size (A, B : Mask) return Boolean is (Width (A) = Width (B) and then Height (A) = Height (B));

   type Distances is array (Natural range <>) of Real;

   function Depths (M : Mask) return Distances is
      --  Every pixel's distance from the region's outside along the region,
      --  in steps of 1 and the square root of 2 (two chamfer passes); the
      --  image's border counts as outside. Built where it is returned, off
      --  the stack: it is sized by pixels, and the estimates also run in the
      --  decider's task, whose stack is small.
      W : constant Natural := Width (M);
      H : constant Natural := Height (M);
      Diagonal : constant Real := Sqrt (2.0);
      function Index (C, R : Natural) return Natural is (R * W + C);
   begin
      return D : Distances (0 .. W * H - 1) do
         declare
            function At_Pixel (C, R : Integer) return Real is
              (if C < 0 or else R < 0 or else C >= W or else R >= H then 0.0 else D (Index (C, R)));
         begin
            for R in 0 .. H - 1 loop
               for C in 0 .. W - 1 loop
                  D (Index (C, R)) := (if Contains (M, C, R) then Real'Last else 0.0);
               end loop;
            end loop;
            for R in 0 .. H - 1 loop
               for C in 0 .. W - 1 loop
                  if D (Index (C, R)) > 0.0 then
                     D (Index (C, R)) := Real'Min (D (Index (C, R)),
                       Real'Min (Real'Min (At_Pixel (C - 1, R) + 1.0, At_Pixel (C, R - 1) + 1.0),
                                 Real'Min (At_Pixel (C - 1, R - 1) + Diagonal, At_Pixel (C + 1, R - 1) + Diagonal)));
                  end if;
               end loop;
            end loop;
            for R in reverse 0 .. H - 1 loop
               for C in reverse 0 .. W - 1 loop
                  if D (Index (C, R)) > 0.0 then
                     D (Index (C, R)) := Real'Min (D (Index (C, R)),
                       Real'Min (Real'Min (At_Pixel (C + 1, R) + 1.0, At_Pixel (C, R + 1) + 1.0),
                                 Real'Min (At_Pixel (C + 1, R + 1) + Diagonal, At_Pixel (C - 1, R + 1) + Diagonal)));
                  end if;
               end loop;
            end loop;
         end;
      end return;
   end Depths;

   function Radius (M : Mask) return Real is
      D    : constant Distances := Depths (M);
      Deep : Real := 0.0;
   begin
      for X of D loop
         Deep := Real'Max (Deep, X);
      end loop;
      return Deep;
   end Radius;

   function Inner_Point (M : Mask) return Pixel is
      --  Of the deepest pixels, the one nearest their own centroid: the
      --  middle of a plateau, as in a rectangle, and inside a bent region.
      W     : constant Natural := Width (M);
      D     : constant Distances := Depths (M);
      Deep  : constant Real := Radius (M);
      Sum_U, Sum_V : Real := 0.0;
      Count_Deep   : Natural := 0;
   begin
      for I in D'Range loop
         if D (I) = Deep then
            Sum_U := Sum_U + Real (I mod W);
            Sum_V := Sum_V + Real (I / W);
            Count_Deep := Count_Deep + 1;
         end if;
      end loop;
      declare
         U0   : constant Real := Sum_U / Real (Count_Deep);
         V0   : constant Real := Sum_V / Real (Count_Deep);
         Best : Natural := 0;
         Near : Real := Real'Last;
      begin
         for I in D'Range loop
            if D (I) = Deep and then (Real (I mod W) - U0) ** 2 + (Real (I / W) - V0) ** 2 < Near then
               Near := (Real (I mod W) - U0) ** 2 + (Real (I / W) - V0) ** 2;
               Best := I;
            end if;
         end loop;
         --  The pixel's centre.
         return (U => Real (Best mod W) + 0.5, V => Real (Best / W) + 0.5);
      end;
   end Inner_Point;

   function On (M : Mask; P : Pixel) return Boolean is
     (P.U >= 0.0 and then P.V >= 0.0 and then P.U < Real (Width (M)) and then P.V < Real (Height (M))
      and then Contains (M, Natural (Real'Floor (P.U)), Natural (Real'Floor (P.V))));

   function Same_Pixels (A, B : Mask) return Boolean is
     (Same_Size (A, B) and then Count (A) > 0 and then Count (B) > 0
      and then On (B, Inner_Point (A)) and then On (A, Inner_Point (B)));

   function Bounds (M : Mask) return Box is
      Result : Box := (Column_0 => Natural'Last, Row_0 => Natural'Last, Column_1 => 0, Row_1 => 0);
   begin
      for R in 0 .. Height (M) - 1 loop
         for C in 0 .. Width (M) - 1 loop
            if Contains (M, C, R) then
               Result := (Column_0 => Natural'Min (Result.Column_0, C), Row_0 => Natural'Min (Result.Row_0, R),
                          Column_1 => Natural'Max (Result.Column_1, C), Row_1 => Natural'Max (Result.Row_1, R));
            end if;
         end loop;
      end loop;
      return Result;
   end Bounds;

   function Overlap (A, B : Mask) return Natural is
      N : Natural := 0;
   begin
      if not Same_Size (A, B) then
         return 0;
      end if;
      for R in 0 .. Height (A) - 1 loop
         for C in 0 .. Width (A) - 1 loop
            if Contains (A, C, R) and then Contains (B, C, R) then
               N := N + 1;
            end if;
         end loop;
      end loop;
      return N;
   end Overlap;

end Driver.World.Regions;
