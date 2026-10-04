with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Action.Contact.Simplex;
with Driver.Conventions;

package body Driver.Action.Contact.Wrench is

   use Ada.Numerics.Long_Elementary_Functions;
   Pi : constant := Ada.Numerics.Pi;

   Edges : constant Positive :=
     Positive (Real'Ceiling (Pi / Arccos (1.0 - Driver.Conventions.Unchanged_Fraction)));
   --  An inscribed polygon of K edges reaches cos (pi / K) of its cone in the
   --  worst direction; this is the least K that loses less than
   --  Unchanged_Fraction there.

   Round_Off : constant Real := Sqrt (Real'Model_Epsilon);

   subtype Wrench6 is Real_Vector (1 .. 6);
   --  Force, then moment about the thing's centre.

   package Generator_Vectors is new Ada.Containers.Vectors (Positive, Wrench6);

   function Wrench_Of (F, Point, Centre : Vec3) return Wrench6 is
      M : constant Vec3 := Cross (Point - Centre, F);
   begin
      return [F (1), F (2), F (3), M (1), M (2), M (3)];
   end Wrench_Of;

   --  Where each edge of the polygon lies around the cone, computed once.
   type Edge_Table is array (0 .. Edges - 1) of Real;
   Edge_Cos : constant Edge_Table := [for K in Edge_Table'Range => Cos (2.0 * Pi * Real (K) / Real (Edges))];
   Edge_Sin : constant Edge_Table := [for K in Edge_Table'Range => Sin (2.0 * Pi * Real (K) / Real (Edges))];

   --  The edges of a friction cone about the unit N, each carrying one unit
   --  of normal force. An edge's moment is the normal's plus its tangential
   --  part's, each taken once: the cross product is linear.
   procedure Add_Cone (To : in out Generator_Vectors.Vector; N, Point, Centre : Vec3; Mu : Real) is
      T1, T2 : Vec3;
   begin
      Plane_Basis (N, T1, T2);
      declare
         R  : constant Vec3 := Point - Centre;
         MN : constant Vec3 := Cross (R, N);
         M1 : constant Vec3 := Cross (R, T1);
         M2 : constant Vec3 := Cross (R, T2);
      begin
         for K in Edge_Table'Range loop
            declare
               C : constant Real := Mu * Edge_Cos (K);
               S : constant Real := Mu * Edge_Sin (K);
            begin
               To.Append (Wrench6'[N (1) + (C * T1 (1) + S * T2 (1)), N (2) + (C * T1 (2) + S * T2 (2)),
                                   N (3) + (C * T1 (3) + S * T2 (3)), MN (1) + (C * M1 (1) + S * M2 (1)),
                                   MN (2) + (C * M1 (2) + S * M2 (2)), MN (3) + (C * M1 (3) + S * M2 (3))]);
            end;
         end loop;
      end;
   end Add_Cone;

   type Flat is record
      X, Y  : Real;
      Index : Positive;
   end record;

   type Flat_Array is array (Positive range <>) of Flat;

   function "<" (A, B : Flat) return Boolean is (A.X < B.X or else (A.X = B.X and then A.Y < B.Y));

   procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Flat, Flat_Array);

   function Side_Of (O, A, B : Flat) return Real is ((A.X - O.X) * (B.Y - O.Y) - (A.Y - O.Y) * (B.X - O.X));

   type Index_Array is array (Positive range <>) of Positive;

   --  Indexes of the convex hull's vertices, counter-clockwise (Andrew's
   --  monotone chain); collinear middle points are left out.
   function Hull (P : Flat_Array) return Index_Array is
      S : Flat_Array := P;
      H : Flat_Array (1 .. 2 * P'Length + 1) := [others => (X => 0.0, Y => 0.0, Index => 1)];
      N : Natural := 0;
   begin
      if P'Length < 3 then
         return [for I in P'Range => P (I).Index];
      end if;
      Sort (S);
      for I in S'Range loop
         while N >= 2 and then Side_Of (H (N - 1), H (N), S (I)) <= 0.0 loop
            N := N - 1;
         end loop;
         N := N + 1;
         H (N) := S (I);
      end loop;
      declare
         Lower : constant Natural := N + 1;
      begin
         for I in reverse S'First .. S'Last - 1 loop
            while N >= Lower and then Side_Of (H (N - 1), H (N), S (I)) <= 0.0 loop
               N := N - 1;
            end loop;
            N := N + 1;
            H (N) := S (I);
         end loop;
      end;
      --  The chain ends where it began.
      return [for I in 1 .. N - 1 => H (I).Index];
   end Hull;

   function Foot_Hull (Base : Footing; Of_These : Point_Vectors.Vector) return Index_Array is
      E1, E2 : Vec3;
   begin
      Plane_Basis (Base.Up, E1, E2);
      return Hull ([for I in 1 .. Natural (Of_These.Length) =>
                     (X => Of_These (I) * E1, Y => Of_These (I) * E2, Index => I)]);
   end Foot_Hull;

   --  The footing's generators for this motion, all free. Its load may sit
   --  anywhere in the hull of the foot points that stay on the surface, so
   --  the hull's vertices carry it. A vertex that slips brings kinetic
   --  friction against its own slip, in proportion to its own load: for a
   --  slide this matches every load distribution with the same centre, and
   --  for a rotation it puts friction at the outermost points, which can only
   --  overstate the moment the body must overcome. A vertex that does not
   --  slip has its whole friction cone.
   procedure Add_Footing
     (To     : in out Generator_Vectors.Vector;
      Base   : Footing;
      Motion : Twist;
      Centre : Vec3;
      Mu     : Real;
      In_Way : out Boolean)
   is
      Reach : Real := 0.0;
      Tol   : Real;
      Stay  : Point_Vectors.Vector;
   begin
      In_Way := False;
      for Q of Base.Foot loop
         Reach := Real'Max (Reach, abs (Q - Motion.Pivot));
      end loop;
      --  Points closer than one Pitch cannot be told apart, so neither can
      --  speeds differing by the rotation rate times Pitch.
      Tol := abs Motion.Angular * Base.Pitch + Round_Off * (abs Motion.Linear + abs Motion.Angular * Reach);
      for Q of Base.Foot loop
         declare
            Normal : constant Real := Velocity (Motion, Q) * Base.Up;
         begin
            if Normal < -Tol then
               In_Way := True;
               return;
            elsif Normal <= Tol then
               Stay.Append (Q);
            end if;
         end;
      end loop;
      if Stay.Is_Empty then
         return;
      end if;
      for H of Foot_Hull (Base, Stay) loop
         declare
            Q       : constant Vec3 := Stay (H);
            V       : constant Vec3 := Velocity (Motion, Q);
            Sliding : constant Vec3 := V - Real'(V * Base.Up) * Base.Up;
         begin
            if abs Sliding > Tol then
               To.Append (Wrench_Of (Base.Up - Mu * Unit (Sliding), Q, Centre));
            else
               Add_Cone (To, Base.Up, Q, Centre, Mu);
            end if;
         end;
      end loop;
   end Add_Footing;

   function Need
     (Touches : Touch_Vectors.Vector;
      Base    : Footing;
      Motion  : Twist;
      Centre  : Vec3;
      Up      : Vec3;
      Mu      : Real) return Answer
   is
      Paid   : Generator_Vectors.Vector;   --  the body's: one unit of normal force each
      Free   : Generator_Vectors.Vector;   --  the footing's
      In_Way : Boolean := False;
      Length : Real := 0.0;
      Count  : Natural := 0;
   begin
      for T of Touches loop
         if not (abs T.Inward > 0.0) then
            return (No_Way, Unbalanced);
         end if;
         declare
            N : constant Vec3 := Unit (T.Inward);
         begin
            Add_Cone (Paid, N, T.Point, Centre, Mu);
            if T.Patch > 0.0 then
               --  Torsion and sliding friction each claim the whole normal
               --  force here, which is conservative against the true
               --  elliptic limit.
               declare
                  M : constant Vec3 := Cross (T.Point - Centre, N);
                  R : constant Vec3 := (Mu * T.Patch) * N;
               begin
                  Paid.Append (Wrench6'[N (1), N (2), N (3), M (1) + R (1), M (2) + R (2), M (3) + R (3)]);
                  Paid.Append (Wrench6'[N (1), N (2), N (3), M (1) - R (1), M (2) - R (2), M (3) - R (3)]);
               end;
            end if;
            if T.Tension then
               Add_Cone (Paid, -N, T.Point, Centre, Mu);
            end if;
            Length := Length + abs (T.Point - Centre);
            Count := Count + 1;
         end;
      end loop;
      if Base.Present then
         Add_Footing (Free, Base, Motion, Centre, Mu, In_Way);
         if In_Way then
            return (No_Way, Footing_In_Way);
         end if;
         for Q of Base.Foot loop
            Length := Length + abs (Q - Centre);
            Count := Count + 1;
         end loop;
      end if;
      if Paid.Is_Empty and then Free.Is_Empty then
         return (No_Way, Unbalanced);
      end if;
      --  Moments are divided by the mean lever arm so their rows weigh like
      --  the force rows; the solution does not depend on it.
      Length := (if Count > 0 and then Length > 0.0 then Length / Real (Count) else 1.0);
      declare
         U    : constant Vec3 := Unit (Up);
         Cols : constant Positive := Natural (Paid.Length) + Natural (Free.Length);
         A    : Real_Matrix (1 .. 6, 1 .. Cols);
         Cost : Real_Vector (1 .. Cols);
         J    : Natural := 0;
         procedure Column (G : Wrench6; C : Real) is
         begin
            J := J + 1;
            for R in 1 .. 3 loop
               A (R, J) := G (R);
               A (R + 3, J) := G (R + 3) / Length;
            end loop;
            Cost (J) := C;
         end Column;
      begin
         for G of Paid loop
            Column (G, 1.0);
         end loop;
         for G of Free loop
            Column (G, 0.0);
         end loop;
         declare
            use Driver.Action.Contact.Simplex;
            S : constant Solution := Minimize (A, [U (1), U (2), U (3), 0.0, 0.0, 0.0], Cost);
         begin
            if S.Result = Optimal then
               return (S.Value, None);
            end if;
            return (No_Way, Unbalanced);
         end;
      end;
   end Need;

   function Least_Distinct (Resolution : Real) return Real is (Tan (Real'Max (Resolution, Round_Off)));

   function Least_Friction
     (Touches    : Touch_Vectors.Vector;
      Base       : Footing;
      Motion     : Twist;
      Centre     : Vec3;
      Up         : Vec3;
      Resolution : Real;
      Below      : Real := No_Way) return Real
   is
      function Possible (Angle : Real) return Boolean is
        (Need (Touches, Base, Motion, Centre, Up, Tan (Angle)).Force < No_Way);
      Floor : constant Real := Arctan (Least_Distinct (Resolution));
      First : constant Answer := Need (Touches, Base, Motion, Centre, Up, Tan (Floor));
      Low   : Real := Floor;
      High  : Real := (if Below < No_Way then Arctan (Below) else Pi / 2.0);
      Found : Boolean := False;
   begin
      if First.Why = None then
         return Tan (Floor);
      elsif First.Why = Footing_In_Way or else High <= Low then
         return No_Way;
      elsif Below < No_Way then
         if not Possible (High) then
            return No_Way;
         end if;
         Found := True;
      end if;
      --  Bisection on the friction angle, where possibility only grows once
      --  the body's touches need friction at all.
      while High - Low > Real'Max (Resolution, Driver.Conventions.Unchanged_Fraction * High) loop
         declare
            Mid : constant Real := (Low + High) / 2.0;
         begin
            if Possible (Mid) then
               High := Mid;
               Found := True;
            else
               Low := Mid;
            end if;
         end;
      end loop;
      return (if Found then Tan (High) else No_Way);
   end Least_Friction;

   function Rests
     (Base   : Footing;
      Centre : Driver.Uncertain.Point_Estimate;
      Up     : Vec3;
      Mu     : Real) return Rest_Answer
   is
      use Driver.Uncertain;
      E1, E2 : Vec3;
   begin
      if not Base.Present or else not Known (Centre) then
         return (Rests => False, Margin => Real'First);
      end if;
      Plane_Basis (Base.Up, E1, E2);
      declare
         H        : constant Index_Array := Foot_Hull (Base, Base.Foot);
         Result   : Rest_Answer := (Rests => True, Margin => Real'Last);
         No_Touch : Touch_Vectors.Vector;
      begin
         if H'Length < 3 then
            return (Rests => False, Margin => Real'First);
         end if;
         for I in H'Range loop
            declare
               A  : constant Vec3 := Base.Foot (H (I));
               B  : constant Vec3 := Base.Foot (H (if I = H'Last then H'First else I + 1));
               Ex : constant Real := (B - A) * E1;
               Ey : constant Real := (B - A) * E2;
               --  Counter-clockwise hull: the outward normal of edge A to B
               --  is its direction turned clockwise.
               Out_Normal : constant Vec3 := Unit ((Ey * E1) - (Ex * E2));
               Outside    : constant Real := (Centre.Mean - A) * Out_Normal;
               Shifted    : constant Vec3 :=
                 Centre.Mean + (Driver.Conventions.Z * Sigma_Along (Centre.Covariance, Out_Normal)) * Out_Normal;
            begin
               Result.Margin := Real'Min (Result.Margin, -Outside);
               if Need (No_Touch, Base, Still (Shifted), Shifted, Up, Mu).Force = No_Way then
                  Result.Rests := False;
               end if;
            end;
         end loop;
         return Result;
      end;
   end Rests;

end Driver.Action.Contact.Wrench;
