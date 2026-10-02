with Ada.Containers.Ordered_Maps;

package body Driver.Action.Contact is

   function Still (Pivot : Vec3) return Twist is (Linear => Zero3, Angular => Zero3, Pivot => Pivot);

   function Slide (Linear : Vec3) return Twist is (Linear => Linear, Angular => Zero3, Pivot => Zero3);

   function Rotation (Axis : Vec3; Angle : Real; Pivot : Vec3) return Twist is
     (Linear => Zero3, Angular => Angle * Unit (Axis), Pivot => Pivot);

   function Moves (T : Twist) return Boolean is (abs T.Linear > 0.0 or else abs T.Angular > 0.0);

   function Velocity (T : Twist; P : Vec3) return Vec3 is (T.Linear + Cross (T.Angular, P - T.Pivot));

   function Apply (T : Twist; P : Vec3) return Vec3 is
     (T.Pivot + Exp (T.Angular) * (P - T.Pivot) + T.Linear);

   function Scaled (T : Twist; Factor : Real) return Twist is
     (Linear => Factor * T.Linear, Angular => Factor * T.Angular, Pivot => T.Pivot);

   procedure Plane_Basis (Normal : Vec3; E1, E2 : out Vec3) is
      --  The seed is the coordinate axis least aligned with the normal, so the
      --  cross product never vanishes.
      N    : constant Vec3 := Unit (Normal);
      Seed : Vec3 := Zero3;
      Low  : Positive := 1;
   begin
      for K in 2 .. 3 loop
         if abs N (K) < abs N (Low) then
            Low := K;
         end if;
      end loop;
      Seed (Low) := 1.0;
      E1 := Unit (Cross (N, Seed));
      E2 := Cross (N, E1);
   end Plane_Basis;

   type Cell is record
      I, J : Integer;
   end record;

   function "<" (A, B : Cell) return Boolean is (A.I < B.I or else (A.I = B.I and then A.J < B.J));

   type Sum is record
      Total : Vec3 := Zero3;
      Count : Natural := 0;
   end record;

   package Cell_Maps is new Ada.Containers.Ordered_Maps (Cell, Sum);

   function Footing_Of (Points : Point_Vectors.Vector; On, Up : Vec3; Pitch : Real) return Footing is
      Cells  : Cell_Maps.Map;
      E1, E2 : Vec3;
      U      : Vec3;
      F      : Footing;
   begin
      if Points.Is_Empty or else not (abs Up > 0.0) or else not (Pitch > 0.0) then
         return No_Footing;
      end if;
      U := Unit (Up);
      Plane_Basis (U, E1, E2);
      for P of Points loop
         declare
            Height : constant Real := (P - On) * U;
         begin
            if abs Height <= Pitch then
               declare
                  Flat : constant Vec3 := P - Height * U;
                  Key  : constant Cell :=
                    (I => Integer (Real'Floor ((Flat * E1) / Pitch)), J => Integer (Real'Floor ((Flat * E2) / Pitch)));
                  Pos  : constant Cell_Maps.Cursor := Cells.Find (Key);
               begin
                  if Cell_Maps.Has_Element (Pos) then
                     declare
                        S : Sum := Cell_Maps.Element (Pos);
                     begin
                        S.Total := S.Total + Flat;
                        S.Count := S.Count + 1;
                        Cells.Replace_Element (Pos, S);
                     end;
                  else
                     Cells.Insert (Key, (Total => Flat, Count => 1));
                  end if;
               end;
            end if;
         end;
      end loop;
      if Cells.Is_Empty then
         return No_Footing;
      end if;
      F := (Present => True, Up => U, Foot => Point_Vectors.Empty_Vector, Pitch => Pitch);
      for S of Cells loop
         --  The mean of a cell, not its first point, so the footing is not
         --  biased toward one corner of every cell.
         F.Foot.Append (S.Total / Real (S.Count));
      end loop;
      return F;
   end Footing_Of;

   function Base_Of (Points : Point_Vectors.Vector; Up : Vec3; Pitch : Real) return Footing is
      Lowest : Real := Real'Last;
      At_Low : Vec3 := Zero3;
   begin
      if Points.Is_Empty or else not (abs Up > 0.0) or else not (Pitch > 0.0) then
         return No_Footing;
      end if;
      for P of Points loop
         if P * Unit (Up) < Lowest then
            Lowest := P * Unit (Up);
            At_Low := P;
         end if;
      end loop;
      --  The plane through the lowest point; Footing_Of keeps everything
      --  within one Pitch of it, which is the lowest layer.
      return Footing_Of (Points, At_Low, Up, Pitch);
   end Base_Of;

end Driver.Action.Contact;
