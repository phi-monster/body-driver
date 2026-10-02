with Ada.Containers.Generic_Array_Sort;
with Ada.Containers.Indefinite_Vectors;
with Ada.Numerics.Long_Elementary_Functions;

package body Driver.World.Supports is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Geometry.Flag_Array;

   function Faces_Up (P : Driver.Geometry.Plane_Estimate; Up : Direction_Estimate) return Boolean is
      Lean  : constant Real := P.Normal * Up.Unit_Vector;
      --  The normal's own turn, averaged over its two directions, with Up's.
      Sigma : constant Real := Sqrt ((P.Tilt_11 + P.Tilt_22) / 2.0 + Up.Sigma ** 2);
   begin
      return Lean > 0.0 and then Significant (Lean, Sigma);
   end Faces_Up;

   package Index_Vectors is new Ada.Containers.Vectors (Positive, Positive);

   type Index_Array is array (Positive range <>) of Positive;

   --  Which point sits at each place of the grid; 0 where none does.
   type Cell_Array is array (Integer range <>, Integer range <>) of Natural;

   function Lowest (Grid : Grid_Array; Of_Rows : Boolean) return Integer is
      Result : Integer := Integer'Last;
   begin
      for G of Grid loop
         Result := Integer'Min (Result, (if Of_Rows then G.Row else G.Column));
      end loop;
      return (if Grid'Length = 0 then 1 else Result);
   end Lowest;

   function Highest (Grid : Grid_Array; Of_Rows : Boolean) return Integer is
      Result : Integer := Integer'First;
   begin
      for G of Grid loop
         Result := Integer'Max (Result, (if Of_Rows then G.Row else G.Column));
      end loop;
      return (if Grid'Length = 0 then 0 else Result);
   end Highest;

   procedure Find
     (Points    : Driver.Geometry.Point_Array;
      Grid      : Grid_Array;
      Up        : Direction_Estimate;
      Seen_From : Vec3;
      Found     : out Surface_Vectors.Vector)
   is
      Cells  : Cell_Array (Lowest (Grid, False) .. Highest (Grid, False), Lowest (Grid, True) .. Highest (Grid, True)) :=
        [others => [others => 0]];

      function At_Cell (C, R : Integer) return Natural is
        (if C in Cells'Range (1) and then R in Cells'Range (2) then Cells (C, R) else 0);

      Left   : Driver.Geometry.Flag_Array (Points'Range) := [others => True];
      Height : array (Points'Range) of Real;
      Spread : array (Points'Range) of Real;
      U      : constant Vec3 := Up.Unit_Vector;
      Z      : constant Real := Threshold (Scalar_Gate);

      --  The points by height, so the points near one height are found by
      --  walking out from it; every walk reaches as far as two points' combined
      --  spread can, with the widest spread there is.
      Order  : Index_Array (Points'Range);
      Rank   : Index_Array (Points'Range);
      Widest : Real := 0.0;

      --  How many of the points left are level with each.
      Near   : array (Points'Range) of Natural := [others => 0];

      function Level (I, J : Positive) return Boolean is
        (not Significant (Height (I) - Height (J), Sqrt (Spread (I) ** 2 + Spread (J) ** 2)));
      --  Two points at heights along Up their own uncertainties cannot tell apart.

      function Reach (I : Positive) return Real is (Z * Sqrt (Spread (I) ** 2 + Widest ** 2));

      procedure Walk (I : Positive; Within : Real; Visit : not null access procedure (J : Positive)) is
         --  Every point left whose height is within that of point I's.
         K : Integer := Rank (I);
      begin
         while K >= Order'First and then Height (I) - Height (Order (K)) <= Within loop
            if Left (Order (K)) then
               Visit (Order (K));
            end if;
            K := K - 1;
         end loop;
         K := Rank (I) + 1;
         while K <= Order'Last and then Height (Order (K)) - Height (I) <= Within loop
            if Left (Order (K)) then
               Visit (Order (K));
            end if;
            K := K + 1;
         end loop;
      end Walk;

      procedure Remove (Leaving : Driver.Geometry.Flag_Array) is
         --  The points leave, and no longer count as level with any.
         Gone : Index_Vectors.Vector;
      begin
         for J in Points'Range loop
            if Leaving (J) and then Left (J) then
               Left (J) := False;
               Gone.Append (J);
            end if;
         end loop;
         for J of Gone loop
            declare
               procedure Uncount (I : Positive) is
               begin
                  if Level (I, J) then
                     Near (I) := Near (I) - 1;
                  end if;
               end Uncount;
            begin
               Walk (J, Reach (J), Uncount'Access);
            end;
         end loop;
      end Remove;

      procedure Take (Patch : Driver.Geometry.Flag_Array; P : Driver.Geometry.Plane_Estimate) is
         --  A support of the patch's points, reaching as far as they do.
         S : Surface := (Plane  => P, Low_1 => Real'Last, High_1 => Real'First,
                         Low_2  => Real'Last, High_2 => Real'First, Members => Member_Vectors.Empty_Vector);
      begin
         for J in Patch'Range loop
            if Patch (J) then
               declare
                  D : constant Vec3 := Points (J).Mean - P.Centre;
               begin
                  S.Low_1 := Real'Min (S.Low_1, D * P.Tangent_1);
                  S.High_1 := Real'Max (S.High_1, D * P.Tangent_1);
                  S.Low_2 := Real'Min (S.Low_2, D * P.Tangent_2);
                  S.High_2 := Real'Max (S.High_2, D * P.Tangent_2);
                  S.Members.Append (J);
               end;
            end if;
         end loop;
         Found.Append (S);
      end Take;

      function Patches (Among, Seeds : Driver.Geometry.Flag_Array) return Driver.Geometry.Flag_Array is
         --  The patches of grid neighbours among the points marked that hold a
         --  seed and a two by two block of the grid.
         Patch   : Driver.Geometry.Flag_Array (Points'Range) := [others => False];
         Visited : Driver.Geometry.Flag_Array (Points'Range) := [others => False];
         function Marked_At (C, R : Integer; I : out Positive) return Boolean is
            Here : constant Natural := At_Cell (C, R);
         begin
            I := Points'First;
            if Here /= 0 and then Among (Here) then
               I := Here;
               return True;
            end if;
            return False;
         end Marked_At;
      begin
         for Seed in Points'Range loop
            if Among (Seed) and then Seeds (Seed) and then not Visited (Seed) then
               declare
                  Component : Index_Vectors.Vector;
                  Head      : Positive := 1;
                  Has_Block : Boolean := False;
               begin
                  Visited (Seed) := True;
                  Component.Append (Seed);
                  while Head <= Natural (Component.Length) loop
                     declare
                        I : constant Positive := Component (Head);
                        G : constant Grid_Point := Grid (I);
                        Next, A, B, C : Positive;
                        Steps : constant array (1 .. 4) of Grid_Point :=
                          [(G.Column + 1, G.Row), (G.Column - 1, G.Row), (G.Column, G.Row + 1), (G.Column, G.Row - 1)];
                     begin
                        Head := Head + 1;
                        for S of Steps loop
                           if Marked_At (S.Column, S.Row, Next) and then not Visited (Next) then
                              Visited (Next) := True;
                              Component.Append (Next);
                           end if;
                        end loop;
                        if Marked_At (G.Column + 1, G.Row, A) and then Marked_At (G.Column, G.Row + 1, B)
                          and then Marked_At (G.Column + 1, G.Row + 1, C)
                        then
                           Has_Block := True;
                        end if;
                     end;
                  end loop;
                  if Has_Block then
                     for I of Component loop
                        Patch (I) := True;
                     end loop;
                  end if;
               end;
            end if;
         end loop;
         return Patch;
      end Patches;

      package Patch_Vectors is new Ada.Containers.Indefinite_Vectors
        (Positive, Driver.Geometry.Flag_Array, Driver.Geometry."=");

      procedure Grow_With
        (Seed  : Driver.Geometry.Flag_Array;
         Scale : Real;
         Dof   : Natural;
         Patch : out Driver.Geometry.Flag_Array;
         P     : out Driver.Geometry.Plane_Estimate;
         Ok    : out Boolean)
      is
         --  From the seed, the connected patch of grid neighbours the plane
         --  through it holds, with the points' spreads scaled by Scale (resting
         --  on Dof degrees of freedom; none when it is one). Each pass fits the
         --  patch and keeps those of it, and of the points left beside it, whose
         --  height off the plane is not significant against their scaled spread
         --  and the plane's there; then the part of them that holds the seed.
         --  Only neighbours are tried, so a plane fitted on a small patch is
         --  never stretched to far points it cannot tell apart. It ends when
         --  the patch stops changing, or when it comes back to an earlier one:
         --  then the points it held all through that cycle are the patch, those
         --  the refits kept flipping left out.
         Holds  : constant Driver.Uncertain.Gate := Scalar_Gate (Dof);
         Passed : Patch_Vectors.Vector;   --  the patches of the passes so far
      begin
         Patch := Seed;
         P := (others => <>);
         Ok := False;
         for Pass in Points'Range loop
            Driver.Geometry.Fit (Points, Patch, P, Ok);
            exit when not Ok;
            declare
               Candidate : Driver.Geometry.Flag_Array (Points'Range) := Patch;
               Next      : Driver.Geometry.Flag_Array (Points'Range) := [others => False];
               function Along (I : Positive) return Real is
                 (Sqrt (P.Normal * (Points (I).Covariance * P.Normal)));
            begin
               for I in Points'Range loop
                  if Patch (I) then
                     declare
                        G     : constant Grid_Point := Grid (I);
                        Steps : constant array (1 .. 4) of Grid_Point :=
                          [(G.Column + 1, G.Row), (G.Column - 1, G.Row), (G.Column, G.Row + 1), (G.Column, G.Row - 1)];
                     begin
                        for S of Steps loop
                           declare
                              Beside : constant Natural := At_Cell (S.Column, S.Row);
                           begin
                              if Beside /= 0 and then Left (Beside) then
                                 Candidate (Beside) := True;
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
               for I in Points'Range loop
                  if Candidate (I) then
                     Next (I) := not Significant
                       (Holds, Driver.Geometry.Height (P, Points (I).Mean),
                        Sqrt ((Scale * Along (I)) ** 2 + Driver.Geometry.Height_Sigma (P, Points (I).Mean) ** 2));
                  end if;
               end loop;
               Next := Patches (Next, Seed);
               if Next = Patch then
                  return;
               end if;
               declare
                  Back_To : constant Patch_Vectors.Extended_Index := Passed.Find_Index (Next);
               begin
                  if Back_To /= Patch_Vectors.No_Index then
                     declare
                        Core : Driver.Geometry.Flag_Array := Patch;
                     begin
                        for K in Back_To .. Passed.Last_Index loop
                           Core := Core and Passed (K);
                        end loop;
                        Patch := Patches (Core, Seed);
                        exit when not (for some F of Patch => F);
                        Driver.Geometry.Fit (Points, Patch, P, Ok);
                        return;
                     end;
                  end if;
               end;
               Passed.Append (Patch);
               Patch := Next;
               exit when not (for some F of Patch => F);
            end;
         end loop;
         Ok := False;
      end Grow_With;

      procedure Grow
        (Seed  : Driver.Geometry.Flag_Array;
         Patch : out Driver.Geometry.Flag_Array;
         P     : out Driver.Geometry.Plane_Estimate;
         Ok    : out Boolean)
      is
         --  The patch grown with the points' own spreads, and grown again with
         --  them scaled to how the patch it gave scatters about its plane
         --  (the Birge ratio), when significantly more than they say, until a
         --  patch comes again. The scale is fixed while a patch grows, so points
         --  the patch does not hold cannot widen it as they enter.
         Scale : Real := 1.0;
         Dof   : Natural := 0;
         Grown : Patch_Vectors.Vector;   --  the patches of the stages so far
      begin
         loop
            Grow_With (Seed, Scale, Dof, Patch, P, Ok);
            exit when not Ok or else Grown.Contains (Patch);
            Grown.Append (Patch);
            declare
               --  A plane takes three of its points' degrees of freedom.
               Freedom : constant Natural := P.Points - 3;
               Wider   : constant Boolean :=
                 Freedom > 0 and then P.Scatter > 1.0
                 and then Significant (Vector_Gate (Freedom), Sqrt (Real (Freedom) * P.Scatter), 1.0);
            begin
               Scale := (if Wider then Sqrt (P.Scatter) else 1.0);
               Dof := (if Wider then Freedom else 0);
            end;
         end loop;
      end Grow;

      function Lower (A, B : Positive) return Boolean is (Height (A) < Height (B));
      procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Positive, Index_Array, Lower);
   begin
      Found.Clear;
      if Up.Sigma >= Real'Last or else Points'Length = 0 then
         return;
      end if;
      for I in Points'Range loop
         Height (I) := U * Points (I).Mean;
         Spread (I) := Sqrt (U * (Points (I).Covariance * U));
         Widest := Real'Max (Widest, Spread (I));
         Order (I) := I;
         Cells (Grid (I).Column, Grid (I).Row) := I;
      end loop;
      Sort (Order);
      for K in Order'Range loop
         Rank (Order (K)) := K;
      end loop;
      for I in Points'Range loop
         declare
            procedure Count_Level (J : Positive) is
            begin
               if Level (I, J) then
                  Near (I) := Near (I) + 1;
               end if;
            end Count_Level;
         begin
            Walk (I, Reach (I), Count_Level'Access);
         end;
      end loop;
      loop
         declare
            Densest : Natural := 0;
            Most    : Natural := 0;
         begin
            --  The height most of the points left share.
            for I in Points'Range loop
               if Left (I) and then Near (I) > Most then
                  Most := Near (I);
                  Densest := I;
               end if;
            end loop;
            exit when Densest = 0;
            declare
               Start : Driver.Geometry.Flag_Array (Points'Range) := [others => False];
               procedure Mark_Start (J : Positive) is
               begin
                  Start (J) := Level (J, Densest);
               end Mark_Start;
            begin
               Walk (Densest, Reach (Densest), Mark_Start'Access);
               declare
                  --  A surface at this height shows as a patch of the grid
                  --  already; a slice of a wall is a row, and is fitted no plane.
                  Seed  : constant Driver.Geometry.Flag_Array := Patches (Start, Start);
                  Patch : Driver.Geometry.Flag_Array (Points'Range) := [others => False];
                  P     : Driver.Geometry.Plane_Estimate;
                  Ok    : Boolean := False;
               begin
                  if (for some F of Seed => F) then
                     Grow (Seed, Patch, P, Ok);
                  end if;
                  if Ok and then (for some F of Patch => F) then
                     Driver.Geometry.Orient (P, Seen_From);
                     if Faces_Up (P, Up) then
                        Take (Patch, P);
                     end if;
                     Remove ([for I in Points'Range => Patch (I) or else Seed (I) or else I = Densest]);
                  else
                     --  No surface around this point. The points level with it
                     --  may still be a surface around a height of their own: only
                     --  a patch tried and fitted no surface is not tried again.
                     Remove ([for I in Points'Range => Seed (I) or else I = Densest]);
                  end if;
               end;
            end;
         end;
      end loop;
   end Find;

   function Mostly (S : Surface; Of_It : not null access function (Member : Positive) return Boolean)
     return Boolean
   is
      On : Natural := 0;
   begin
      for M of S.Members loop
         On := On + Boolean'Pos (Of_It (M));
      end loop;
      return 2 * On > Natural (S.Members.Length);
   end Mostly;

   function Under
     (Surfaces : Surface_Vectors.Vector;
      Points   : Driver.Geometry.Point_Array;
      Up       : Direction_Estimate;
      Own      : not null access function (Member : Positive) return Boolean) return Support
   is
      Best : Support;

      function Its_Own_Face (S : Surface) return Boolean is (Mostly (S, Own));
   begin
      if Points'Length = 0 or else Up.Sigma >= Real'Last then
         return Best;
      end if;
      for F in Surfaces.First_Index .. Surfaces.Last_Index loop
         declare
            S      : constant Surface := Surfaces (F);
            P      : constant Driver.Geometry.Plane_Estimate := S.Plane;
            Lowest : Positive := Points'First;
            Reach  : Real := Real'Last;   --  how low the lowest so far says the thing surely reaches

            function Surely_Down_To (I : Positive) return Real is
               --  A point says the thing reaches down to its height or lower,
               --  surely only to as high as its uncertainty takes it: one of
               --  a family of as many points.
               H : constant Estimate := Driver.Geometry.Height (P, Points (I));
            begin
               return (if H.Sigma < Real'Last
                       then H.Value
                            + Threshold (Scalar_Gate (H.Degrees_Of_Freedom, Tests => Points'Length)) * H.Sigma
                       else Real'Last);
            end Surely_Down_To;
         begin
            --  The lowest point is the one that says the thing reaches lowest
            --  at its own uncertainty: a point uncertain by a hand's breadth,
            --  whatever its mean, says little of how low the thing is.
            for I in Points'Range loop
               if Surely_Down_To (I) < Reach then
                  Reach := Surely_Down_To (I);
                  Lowest := I;
               end if;
            end loop;
            declare
               D    : constant Vec3 := Points (Lowest).Mean - P.Centre;
               A    : constant Real := D * P.Tangent_1;
               B    : constant Real := D * P.Tangent_2;
               H    : constant Estimate := Driver.Geometry.Height (P, Points (Lowest));
               --  From along the normal to along Up.
               Lean : constant Real := P.Normal * Up.Unit_Vector;
               Up_H : constant Estimate := (Value => H.Value / Lean, Sigma => H.Sigma / Lean,
                                            Degrees_Of_Freedom => H.Degrees_Of_Freedom);
            begin
               if not Its_Own_Face (S)
                 and then A >= S.Low_1 and then A <= S.High_1 and then B >= S.Low_2 and then B <= S.High_2
                 and then not (H.Value < 0.0
                               and then Significant (Scalar_Gate (H.Degrees_Of_Freedom, Tests => Points'Length),
                                                     H.Value, H.Sigma))
                 and then (Best.Index = 0 or else Up_H.Value < Best.Height.Value)
               then
                  Best := (Index => F, Height => Up_H, Touching => False);
               end if;
            end;
         end;
      end loop;
      --  Its lowest point on the support within their uncertainties: the eyes
      --  see it touch. Above it: they see no lower, and what they do not see
      --  may reach down to the support.
      Best.Touching := Best.Index /= 0
        and then not (Best.Height.Value > 0.0
                      and then Significant (Scalar_Gate (Best.Height.Degrees_Of_Freedom, Tests => Points'Length),
                                            Best.Height.Value, Best.Height.Sigma));
      return Best;
   end Under;

end Driver.World.Supports;
