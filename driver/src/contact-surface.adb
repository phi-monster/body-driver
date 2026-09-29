with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Contact.Surface is

   procedure On_Plane (Rays : Geom.Sight_Vectors.Vector; P0, N : V3; Pts : in out V3_Vectors.Vector; Dropped : out Natural) is
      Ok : Boolean;
   begin
      Dropped := 0;
      for R of Rays loop
         declare
            H : constant V3 := Geom.Hit_Plane (R.O, R.D, P0, N, Ok);
         begin
            if Ok then
               Pts.Append (H);
            else
               Dropped := Dropped + 1;
            end if;
         end;
      end loop;
   end On_Plane;

   procedure Walls_To_Support (Top : V3_Vectors.Vector; N, Support_P : V3; Pitch : Long_Float; Pts : out V3_Vectors.Vector) is
      Ok : Boolean;
      Nu : constant V3 := Unit (N, Ok);
      Ax : constant V3 := (if abs Nu (0) <= abs Nu (1) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);   --  x、y 里离 N 更远的那根
      E1 : V3;
      E2 : V3;
      type Cell is record
         I, J : Integer;
      end record;
      function "<" (A, B : Cell) return Boolean is (A.I < B.I or else (A.I = B.I and then A.J < B.J));
      package Cell_Sets is new Ada.Containers.Ordered_Sets (Cell);
      Occ : Cell_Sets.Set;
      Side : constant Long_Float := 2.0 * Pitch;   --  格子边长两个采样间距(对角邻居在 √2 个以内,取 2 个;纯几何)
      function Cell_Of (P : V3) return Cell is
        (I => Integer (Long_Float'Floor (Dot (P, E1) / Side)), J => Integer (Long_Float'Floor (Dot (P, E2) / Side)));
   begin
      Pts := Top;
      if not Ok or else Pitch <= 0.0 then
         return;
      end if;
      declare
         C1 : constant V3 := Cross (Nu, Ax);
         Ok1 : Boolean;
      begin
         E1 := Unit (C1, Ok1);
         E2 := Cross (Nu, E1);
      end;
      for P of Top loop
         Occ.Include (Cell_Of (P));
      end loop;
      for P of Top loop
         declare
            C : constant Cell := Cell_Of (P);
            Edge : constant Boolean := not Occ.Contains ((C.I + 1, C.J)) or else not Occ.Contains ((C.I - 1, C.J))
              or else not Occ.Contains ((C.I, C.J + 1)) or else not Occ.Contains ((C.I, C.J - 1));
            H : constant Long_Float := Dot ([P (0) - Support_P (0), P (1) - Support_P (1), P (2) - Support_P (2)], Nu);
            K : Positive := 1;
         begin
            if Edge then
               while Long_Float (K) * Pitch < H - 0.5 * Pitch loop   --  最低一层离面至少半个间距(一半,纯数学:面本身不补,碰桌面归面管)
                  Pts.Append (V3'([P (0) - Long_Float (K) * Pitch * Nu (0), P (1) - Long_Float (K) * Pitch * Nu (1), P (2) - Long_Float (K) * Pitch * Nu (2)]));
                  K := K + 1;
               end loop;
            end if;
         end;
      end loop;
   end Walls_To_Support;

end Contact.Surface;
