with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Numerics;
with Driver.Conventions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Stats;

package body Driver.Robot.Hand.Shape is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;

   subtype Vec5 is Real_Vector (1 .. 5);

   --  Sized by pixels: on the heap, as the estimates also run in the
   --  decider's task, whose stack is small.
   type Point_Access is access Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Point_Array, Point_Access);
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   function From_Moves
     (Moves       : Driver.Robot.Hand.Lobes.Move_Array;
      Back        : Driver.Robot.Hand.Lobes.Move;
      Line        : not null access function (Px : Driver.Images.Pixel) return Vec3;
      Pixel_Sigma : Real) return Sighting_Array
   is
      function Angle_Between (A, B : Vec3) return Real is (Arctan (abs Cross (A, B), A * B));

      function Pitch (Px : Driver.Images.Pixel) return Real is
        ((Angle_Between (Line (Px), Line ((U => Px.U + 1.0, V => Px.V)))
          + Angle_Between (Line (Px), Line ((U => Px.U, V => Px.V + 1.0)))) / 2.0);
      --  The angle a pixel spans there: to its neighbours across and down, averaged.
   begin
      --  Built where it is returned, off the stack: as many as the lobe has pixels.
      return Result : Sighting_Array (1 .. Moves'Length + 1) do
         for I in Moves'Range loop
            Result (I - Moves'First + 1) := (At_Pixel => Moves (I).From,
                                             Open     => Line (Moves (I).From),
                                             Closed   => Line (Moves (I).To),
                                             Sigma    => Pixel_Sigma * Pitch (Moves (I).To),
                                             Pitch    => Pitch (Moves (I).From));
         end loop;
         --  The matcher's noise is where the closed tip matched back to.
         Result (Result'Last) := (At_Pixel => Back.To,
                                  Open     => Line (Back.To),
                                  Closed   => Line (Back.From),
                                  Sigma    => Pixel_Sigma * Pitch (Back.To),
                                  Pitch    => Pitch (Back.To));
      end return;
   end From_Moves;

   procedure Across (D : Vec3; A1, A2 : out Vec3);
   --  Two unit directions across a unit one, and across each other.

   procedure Across (D : Vec3; A1, A2 : out Vec3) is
      --  Crossed with the axis it lies least along, which is never parallel to it.
      K    : Positive := 1;
      Axis : Vec3 := Zero3;
   begin
      for I in 2 .. 3 loop
         if abs D (I) < abs D (K) then
            K := I;
         end if;
      end loop;
      Axis (K) := 1.0;
      A1 := Unit (Cross (D, Axis));
      A2 := Cross (D, A1);
   end Across;

   --  One sighting against a motion: the closed end's point is
   --  W = Depth * Rotation * Open + Shift, seen from the eye; its residual is
   --  how far W's direction is from Closed, as the tangent of the angle in
   --  the plane across Closed, and the derivatives of that residual with
   --  respect to the turn (a small rotation vector applied after Rotation),
   --  the shift's two components across itself, and the depth.
   type Jacobian is array (1 .. 5) of Vec3;

   type Terms is record
      In_Front : Boolean := False;   --  the point is ahead of the eye at the closed end
      Residual : Vec3 := Zero3;
      By_Turn  : Jacobian := [others => Zero3];
      By_Depth : Vec3 := Zero3;
      Range_At : Real := 0.0;        --  Closed . W: how far along Closed the point is
      Lever    : Vec3 := Zero3;      --  Rotation * Open
   end record;

   function Terms_Of (S : Sighting; Rotation : Mat3; Shift, A1, A2 : Vec3; Depth : Real) return Terms is
      B  : constant Vec3 := Rotation * S.Open;
      Q  : constant Vec3 := Depth * B;
      W  : constant Vec3 := Q + Shift;
      L  : constant Real := abs W;
   begin
      if L = 0.0 then
         return (In_Front => False, others => <>);
      end if;
      declare
         --  The chord from Closed to W's direction: the angle between them
         --  where it is small, and smooth however far apart they are, so a
         --  motion on the way to the right one may pass a point behind the
         --  eye without a wall in its path.
         Wn : constant Vec3 := (1.0 / L) * W;
         R  : constant Vec3 := Wn - S.Closed;
         --  The derivative of R with respect to W, applied to X.
         function D (X : Vec3) return Vec3 is ((1.0 / L) * (X - Real'(Wn * X) * Wn));
      begin
         return (In_Front => Real'(S.Closed * W) > 0.0,
                 Residual => R,
                 By_Turn  => [D (Cross ([1.0, 0.0, 0.0], Q)), D (Cross ([0.0, 1.0, 0.0], Q)),
                              D (Cross ([0.0, 0.0, 1.0], Q)), D (A1), D (A2)],
                 By_Depth => D (B),
                 Range_At => S.Closed * W,
                 Lever    => B);
      end;
   end Terms_Of;

   --  The depth of a sighting that fits a motion best, its closed end's
   --  distance from Closed's line made least; zero when the motion gives it
   --  no parallax, so that no depth fits it better than another.
   function Depth_Of (S : Sighting; Rotation : Mat3; Shift : Vec3) return Real is
      B      : constant Vec3 := Rotation * S.Open;
      B_Off  : constant Vec3 := B - Real'(S.Closed * B) * S.Closed;
      A_Off  : constant Vec3 := Shift - Real'(S.Closed * Shift) * S.Closed;
      Spread : constant Real := B_Off * B_Off;
   begin
      return (if Spread > 0.0 then -(B_Off * A_Off) / Spread else 0.0);
   end Depth_Of;

   --  A sighting's own part of the information on its depth, and how its
   --  depth leans on the motion: Coupling = a_turn,depth / a_depth,depth, so
   --  a change of the motion by Delta moves its depth by -Coupling . Delta.
   procedure Own_And_Coupling
     (S        : Sighting;
      T        : Terms;
      Own      : out Real;
      Coupling : out Vec5)
   is
      Weight : constant Real := 1.0 / S.Sigma ** 2;
      Ll     : constant Real := Weight * (T.By_Depth * T.By_Depth);
   begin
      Coupling := [others => 0.0];
      Own := Real'Last;
      if Ll > 0.0 then
         Own := 1.0 / Ll;
         for K in 1 .. 5 loop
            Coupling (K) := Weight * (T.By_Turn (K) * T.By_Depth) / Ll;
         end loop;
      end if;
   end Own_And_Coupling;

   function Fit
     (Eye           : Vec3;
      Sightings     : Sighting_Array;
      Open_Anchor   : Positive;
      Closed_Anchor : Positive;
      Bordered      : Boolean;
      Closed_Is_End : Boolean := True;
      Anchored      : Boolean := True;
      Pixels        : Driver.Images.Mask := Driver.Images.Create (0, 0)) return Lobe_Shape
   is
      Unknowns : constant := 5;   --  the turn's three and the shift's direction's two
      Result   : Lobe_Shape := (Eye           => Eye,
                                Open_Anchor   => Open_Anchor,
                                Closed_Anchor => Closed_Anchor,
                                Bordered      => Bordered,
                                Pixels        => Pixels,
                                Closed_Is_End => Closed_Is_End,
                                others        => <>);
      Points   : Point_Access := new Point_Array (Sightings'Range);
      Trial    : Real_Access := new Real_Array (Sightings'Range);
      Rotation : Mat3 := Identity3;
      Shift    : Vec3;
      A1, A2   : Vec3;
      Info     : Mat5;
      Kept     : Natural := 0;
      Kept_Scatter : Real := 1.0;
      Dof      : Natural := 0;

      procedure Fail (Why : String) is
      begin
         Result.Fitted := False;
         Result.Why := To_Unbounded_String (Why);
      end Fail;

      function Usable (I : Positive) return Boolean is
        (Sightings (I).Sigma > 0.0 and then Sightings (I).Sigma < Real'Last);

      --  Gauss-Newton on the motion and every kept depth together, the
      --  depths eliminated from the normal equations (the Schur complement:
      --  each depth only its own sighting sees). Each step, of the motion
      --  and of every depth with it, is cut by halves until it lowers the
      --  cost: from where the fit starts, a lobe that turns about a hinge is
      --  far from where one full step leads. It ends when a step moves the
      --  motion by less than Unchanged_Fraction of its own uncertainty, or
      --  when no part of a step that is not that small lowers the cost.
      --  False when the kept sightings do not fix the motion.
      function Converge return Boolean is
         Moves : Real_Access := new Real_Array (Sightings'Range);   --  each depth's part of the step

         function Cost_Of (R : Mat3; T : Vec3; Part : Real) return Real is
            B1, B2 : Vec3;
            Sum    : Real := 0.0;
         begin
            Across (T, B1, B2);
            for I in Sightings'Range loop
               if Points (I).Kept then
                  Trial (I) := Points (I).Depth + Part * Moves (I);
                  declare
                     After : constant Terms := Terms_Of (Sightings (I), R, T, B1, B2, Trial (I));
                  begin
                     Sum := Sum + (After.Residual * After.Residual) / Sightings (I).Sigma ** 2;
                  end;
               end if;
            end loop;
            return Sum;
         end Cost_Of;

         Cost : Real := 0.0;
      begin
         --  No step yet: the cost where the motion is. (An allocated array
         --  holds whatever the heap held, a NaN among it.)
         for M of Moves.all loop
            M := 0.0;
         end loop;
         Cost := Cost_Of (Rotation, Shift, 0.0);
         loop
            declare
               S        : Mat5 := [others => [others => 0.0]];
               G        : Vec5 := [others => 0.0];
               Lower    : Mat5;
               Definite : Boolean;
               Delta_M  : Vec5;
               Part     : Real := 1.0;
               Accepted : Boolean := False;
            begin
               for I in Sightings'Range loop
                  if Points (I).Kept then
                     declare
                        T : constant Terms :=
                          Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                        Weight : constant Real := 1.0 / Sightings (I).Sigma ** 2;
                        Ll     : constant Real := Weight * (T.By_Depth * T.By_Depth);
                        Tl     : Vec5;
                        Gl     : Real;
                     begin
                        if Ll > 0.0 then
                           Gl := Weight * (T.By_Depth * T.Residual);
                           for K in 1 .. 5 loop
                              Tl (K) := Weight * (T.By_Turn (K) * T.By_Depth);
                           end loop;
                           for K in 1 .. 5 loop
                              for L in 1 .. 5 loop
                                 S (K, L) := S (K, L) + Weight * (T.By_Turn (K) * T.By_Turn (L)) - Tl (K) * Tl (L) / Ll;
                              end loop;
                              G (K) := G (K) + Weight * (T.By_Turn (K) * T.Residual) - Tl (K) * Gl / Ll;
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
               Driver.Numerics.Dense.Cholesky (S, Lower, Definite);
               if not Definite then
                  Free (Moves);
                  return False;
               end if;
               Info := S;
               Delta_M := -Driver.Numerics.Dense.Cholesky_Solve (Lower, G);
               exit when Delta_M * (S * Delta_M) <= Driver.Conventions.Unchanged_Fraction ** 2;
               --  Each depth's part of the step: its own equation solved with
               --  the motion's step put in.
               for I in Sightings'Range loop
                  Moves (I) := 0.0;
                  if Points (I).Kept then
                     declare
                        T : constant Terms :=
                          Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                        Weight : constant Real := 1.0 / Sightings (I).Sigma ** 2;
                        Ll     : constant Real := Weight * (T.By_Depth * T.By_Depth);
                        Step   : Real := 0.0;
                     begin
                        if Ll > 0.0 then
                           Step := Weight * (T.By_Depth * T.Residual);
                           for K in 1 .. 5 loop
                              Step := Step + Weight * (T.By_Turn (K) * T.By_Depth) * Delta_M (K);
                           end loop;
                           Moves (I) := -Step / Ll;
                        end if;
                     end;
                  end if;
               end loop;
               --  Halve the step until it lowers the cost, while it is not
               --  yet negligible against the motion's own uncertainty.
               while not Accepted
                 and then (Part ** 2) * (Delta_M * (S * Delta_M)) > Driver.Conventions.Unchanged_Fraction ** 2
               loop
                  declare
                     Taken        : constant Vec5 := Part * Delta_M;
                     New_Rotation : constant Mat3 := Exp (Taken (1 .. 3)) * Rotation;
                     New_Shift    : constant Vec3 := Unit (Shift + Taken (4) * A1 + Taken (5) * A2);
                     New_Cost     : constant Real := Cost_Of (New_Rotation, New_Shift, Part);
                  begin
                     if New_Cost < Cost then
                        Accepted := True;
                        Cost := New_Cost;
                        Rotation := New_Rotation;
                        Shift := New_Shift;
                        Across (Shift, A1, A2);
                        for I in Sightings'Range loop
                           if Points (I).Kept then
                              Points (I).Depth := Trial (I);
                           end if;
                        end loop;
                     else
                        Part := Part / 2.0;
                     end if;
                  end;
               end loop;
               exit when not Accepted;
            end;
         end loop;
         Free (Moves);
         return True;
      end Converge;

      --  The signed distance of a kept sighting from where the motion puts
      --  it, across the line its depth slides it along (the depth takes the
      --  rest), in units of the matcher's noise; zero without parallax.
      function Off_Line (I : Positive; T : Terms) return Real is
         Slide  : constant Vec3 := T.By_Depth;
         Normal : constant Vec3 := Cross (Sightings (I).Closed, Slide);
      begin
         if abs Normal = 0.0 then
            return 0.0;
         end if;
         return (T.Residual * Unit (Normal)) / Sightings (I).Sigma;
      end Off_Line;

   begin
      Result.Sightings := Sighting_Holders.To_Holder (Sightings);
      --  The starts. A pure shift, whose direction every pair of lines of
      --  sight lies in the plane of (the epipole of the two views), least
      --  squares across all of them; its sign puts most points ahead of the
      --  eye. A lobe that turns little fits from there; one that turns a
      --  fifth of a radian about a hinge, seen mostly on one face, fits a
      --  wrong valley (a turn about another axis, the shift reversed) where
      --  its sightings scatter about twice their noise, and its sizes then
      --  do not stand out of their own uncertainty (Measure). And the four
      --  motions of the two views' essential matrix (Essential_Starts), which
      --  hold for a lobe however far it turns. The fit is run from each, and
      --  goes on from the one that leaves the least median residual over every
      --  sighting it can use (Try).
      declare
         M       : Mat3 := [others => [others => 0.0]];
         Values  : Vec3;
         Vectors : Mat3;
         Ahead   : Integer := 0;
      begin
         for I in Sightings'Range loop
            if Usable (I) then
               M := M + Outer (Cross (Sightings (I).Open, Sightings (I).Closed),
                               Cross (Sightings (I).Open, Sightings (I).Closed));
            end if;
         end loop;
         Symmetric_Eigensystem (M, Values, Vectors);
         Shift := [Vectors (1, 3), Vectors (2, 3), Vectors (3, 3)];
         if abs Shift = 0.0 then
            Fail ("its pixels do not move between its ends");
            Free (Points);
            Free (Trial);
            return Result;
         end if;
         Shift := Unit (Shift);
         for I in Sightings'Range loop
            if Usable (I) then
               declare
                  D : constant Real := Depth_Of (Sightings (I), Rotation, Shift);
               begin
                  Ahead := Ahead + (if D > 0.0 then 1 elsif D < 0.0 then -1 else 0);
               end;
            end if;
         end loop;
         if Ahead < 0 then
            Shift := -Shift;
         end if;
      end;
      Across (Shift, A1, A2);
      for I in Sightings'Range loop
         Points (I).Depth := (if Usable (I) then Depth_Of (Sightings (I), Rotation, Shift) else 0.0);
         Points (I).Kept := Usable (I) and then Points (I).Depth > 0.0;
      end loop;
      declare
         Best_Score    : Real := Real'Last;
         Best_Rotation : Mat3 := Rotation;
         Best_Shift    : Vec3 := Shift;
         Best_Points   : Point_Access := new Point_Array'(Points.all);
         Usable_Count  : Natural := 0;

         --  The motion fitted from a start: every usable sighting with its
         --  depth that fits the start best, kept when that is ahead of the
         --  eye; then the fit (Converge). Scored by the median of every
         --  usable sighting's distance from where the motion puts it, in
         --  units of its noise, one it puts behind the eye counted as
         --  unbounded: the right motion leaves its noise, a wrong valley
         --  twice it (the turning jaw above), and a motion that has half
         --  the lobe behind the eye leaves no score at all.
         procedure Try (Start_Rotation : Mat3; Start_Shift : Vec3) is
            Started : Natural := 0;
         begin
            if abs Start_Shift = 0.0 then
               return;
            end if;
            Rotation := Start_Rotation;
            Shift := Unit (Start_Shift);
            Across (Shift, A1, A2);
            for I in Sightings'Range loop
               Points (I).Depth := (if Usable (I) then Depth_Of (Sightings (I), Rotation, Shift) else 0.0);
               Points (I).Kept := Usable (I) and then Points (I).Depth > 0.0;
               Started := Started + Boolean'Pos (Points (I).Kept);
            end loop;
            if Started <= Unknowns or else not Converge then
               return;
            end if;
            declare
               Off : Real_Access := new Real_Array (1 .. Usable_Count);
               K   : Natural := 0;
               Score : Real;
            begin
               for I in Sightings'Range loop
                  if Usable (I) then
                     K := K + 1;
                     Off (K) := Real'Last;
                     if Points (I).Kept and then Points (I).Depth > 0.0 then
                        declare
                           T : constant Terms := Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                        begin
                           if T.In_Front then
                              Off (K) := abs Off_Line (I, T);
                           end if;
                        end;
                     end if;
                  end if;
               end loop;
               Score := Driver.Stats.Median (Off.all);
               Free (Off);
               if Score < Best_Score then
                  Best_Score := Score;
                  Best_Rotation := Rotation;
                  Best_Shift := Shift;
                  Best_Points.all := Points.all;
               end if;
            end;
         end Try;


         --  The hinge. A closer moves its lobe with one channel, and a
         --  joint's motion is a turn about an axis W and a shift across it;
         --  in the frame (E1, E2, W) its essential matrix has four entries,
         --  (A, B) down its last column and (E, F) along its last row, and
         --  each sighting's constraint is linear in them (as the arm fit's
         --  Planar_About has it for a joint's keyframes). Over the axes of
         --  Driver.Conventions.Axis_Directions, the half on one side, the four
         --  are the least eigenvector of the sightings' rows, weighted by
         --  their noise, and the axis is scored by the median Sampson residual
         --  of the sightings in units of their noise. From the best: the shift
         --  (-B, A) across it, the turn about it whose cosine goes as
         --  -(A E + B F) and sine as A F - B E, each way the shift may point.
         --  A lobe on a hinge, seen mostly on one face, starts there in its
         --  own valley however its depth reversal lies (Reversed_Start).
         procedure Axis_Starts is
            Golden     : constant Real := (3.0 - Sqrt (5.0)) * Ada.Numerics.Pi;
            Directions : constant Natural := Driver.Conventions.Axis_Directions;
            Best_Axis  : Vec3 := Zero3;
            Best_Four  : Real_Vector (1 .. 4) := [others => 0.0];
            Lowest     : Real := Real'Last;
            Residuals  : Real_Access := new Real_Array (1 .. Usable_Count);
         begin
            for I in 1 .. Directions / 2 loop
               declare
                  Z      : constant Real := 1.0 - 2.0 * (Real (I) - 0.5) / Real (Directions);
                  Rr     : constant Real := Sqrt (Real'Max (0.0, 1.0 - Z * Z));
                  W      : constant Vec3 := [Rr * Cos (Golden * Real (I)), Rr * Sin (Golden * Real (I)), Z];
                  E1, E2 : Vec3;
                  Sums   : Real_Matrix (1 .. 4, 1 .. 4) := [others => [others => 0.0]];
                  Values : Real_Vector (1 .. 4);
                  Vectors : Real_Matrix (1 .. 4, 1 .. 4);
                  Four   : Real_Vector (1 .. 4);
                  K      : Natural := 0;
                  subtype Four_Terms is Real_Vector (1 .. 4);
                  function Row (S : Sighting) return Four_Terms is
                     O : constant Vec3 := [E1 * S.Open, E2 * S.Open, W * S.Open];
                     C : constant Vec3 := [E1 * S.Closed, E2 * S.Closed, W * S.Closed];
                  begin
                     return [C (1) * O (3), C (2) * O (3), C (3) * O (1), C (3) * O (2)];
                  end Row;
               begin
                  Across (W, E1, E2);
                  for J in Sightings'Range loop
                     if Usable (J) then
                        declare
                           G      : constant Four_Terms := Row (Sightings (J));
                           Weight : constant Real := 1.0 / Sightings (J).Sigma ** 2;
                        begin
                           for P in 1 .. 4 loop
                              for Q in 1 .. 4 loop
                                 Sums (P, Q) := Sums (P, Q) + Weight * G (P) * G (Q);
                              end loop;
                           end loop;
                        end;
                     end if;
                  end loop;
                  Eigensystem (0.5 * (Sums + Transpose (Sums)), Values, Vectors);
                  Four := [for P in 1 .. 4 => Vectors (P, Vectors'Last (2))];
                  declare
                     --  The essential matrix in the world's frame: the four in (E1, E2, W).
                     Frame : constant Mat3 := [[E1 (1), E2 (1), W (1)], [E1 (2), E2 (2), W (2)], [E1 (3), E2 (3), W (3)]];
                     Local : constant Mat3 := [[0.0, 0.0, Four (1)], [0.0, 0.0, Four (2)], [Four (3), Four (4), 0.0]];
                     E     : constant Mat3 := Frame * Local * Transpose (Frame);
                  begin
                     for J in Sightings'Range loop
                        if Usable (J) then
                           declare
                              O  : constant Vec3 := Sightings (J).Open;
                              C  : constant Vec3 := Sightings (J).Closed;
                              Eo : constant Vec3 := E * O;
                              Ec : constant Vec3 := Transpose (E) * C;
                              Across_C : constant Vec3 := Eo - Real'(C * Eo) * C;
                              Across_O : constant Vec3 := Ec - Real'(O * Ec) * O;
                              Spread   : constant Real := Across_C * Across_C + Across_O * Across_O;
                           begin
                              K := K + 1;
                              Residuals (K) := (if Spread > 0.0 then abs (C * Eo) / Sqrt (Spread) / Sightings (J).Sigma
                                                else Real'Last);
                           end;
                        end if;
                     end loop;
                  end;
                  declare
                     Score : constant Real := Driver.Stats.Median (Residuals (1 .. K));
                  begin
                     if Score < Lowest then
                        Lowest := Score;
                        Best_Axis := W;
                        Best_Four := Four;
                     end if;
                  end;
               end;
            end loop;
            Free (Residuals);
            if Lowest < Real'Last then
               declare
                  E1, E2 : Vec3;
                  A      : Real renames Best_Four (1);
                  B      : Real renames Best_Four (2);
                  E      : Real renames Best_Four (3);
                  F      : Real renames Best_Four (4);
                  Turn   : constant Mat3 := Exp (Arctan (A * F - B * E, -(A * E + B * F)) * Best_Axis);
               begin
                  Across (Best_Axis, E1, E2);
                  Try (Turn, -B * E1 + A * E2);
                  Try (Turn, B * E1 - A * E2);
               end;
            end if;
         end Axis_Starts;

      begin
         for I in Sightings'Range loop
            Usable_Count := Usable_Count + Boolean'Pos (Usable (I));
         end loop;
         if Usable_Count > 0 then
            Try (Identity3, Shift);
            Axis_Starts;
         end if;
         Rotation := Best_Rotation;
         Shift := Best_Shift;
         Across (Shift, A1, A2);
         Points.all := Best_Points.all;
         Free (Best_Points);
      end;
      --  Fit, then leave out every sighting whose distance from the fit is
      --  significant against the matcher's noise, raised to the kept ones'
      --  own robust scatter when that is larger; until none is left out.
      loop
         Kept := 0;
         for P of Points.all loop
            Kept := Kept + Boolean'Pos (P.Kept);
         end loop;
         if Kept <= Unknowns then
            Fail ("fewer of its pixels fit one rigid motion than the motion has unknowns");
            exit;
         end if;
         if not Converge then
            Fail ("its two views do not fix how it moved between them");
            exit;
         end if;
         declare
            Off      : Real_Access := new Real_Array (1 .. Kept);
            K        : Natural := 0;
            Left_Out : Natural := 0;
         begin
            for I in Sightings'Range loop
               if Points (I).Kept then
                  declare
                     T : constant Terms := Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                  begin
                     K := K + 1;
                     Off (K) := (if T.In_Front then Off_Line (I, T) else 0.0);
                  end;
               end if;
            end loop;
            Kept_Scatter := Real'Max (1.0, Driver.Stats.Robust_Sigma (Off.all));
            Dof := Natural (Real'Floor (Mad_Efficiency * Real (Kept)));
            Free (Off);
            declare
               Gate_Of : constant Gate := Scalar_Gate (Dof);
            begin
               for I in Sightings'Range loop
                  if Points (I).Kept then
                     declare
                        T : constant Terms := Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                     begin
                        if not T.In_Front or else Points (I).Depth <= 0.0
                          or else Significant (Gate_Of, Off_Line (I, T), Kept_Scatter)
                        then
                           Points (I).Kept := False;
                           Left_Out := Left_Out + 1;
                        end if;
                     end;
                  end if;
               end loop;
            end;
            if Left_Out = 0 then
               Result.Fitted := True;
               exit;
            end if;
         end;
      end loop;
      if Result.Fitted then
         if Anchored and then (not Points (Open_Anchor).Kept or else not Points (Closed_Anchor).Kept) then
            Fail ("its tip does not move with the rest of it");
         else
            --  Each kept point's own share of its depth's variance, for unit noise.
            for I in Sightings'Range loop
               if Points (I).Kept then
                  declare
                     T        : constant Terms := Terms_Of (Sightings (I), Rotation, Shift, A1, A2, Points (I).Depth);
                     Coupling : Vec5;
                  begin
                     Own_And_Coupling (Sightings (I), T, Points (I).Own, Coupling);
                  end;
               end if;
            end loop;
            Result.Rotation := Rotation;
            Result.Shift := Shift;
            Result.Across_1 := A1;
            Result.Across_2 := A2;
            Result.Covariance := Inverse (Info);
            Result.Scatter := Kept_Scatter;
            Result.Kept := Kept;
         end if;
      end if;
      Result.Points := Point_Holders.To_Holder (Points.all);
      Free (Points);
      Free (Trial);
      return Result;
   end Fit;

   function Unfitted (Why : String) return Lobe_Shape is
     ((Why => To_Unbounded_String (Why), others => <>));

   function Not_Followed (Why : String) return Lobe_Shape is
     ((Why => To_Unbounded_String (Why), Unfollowed => True, others => <>));

   function Unfollowed (S : Lobe_Shape) return Boolean is (S.Unfollowed);

   function Fitted (S : Lobe_Shape) return Boolean is (S.Fitted);
   function Why (S : Lobe_Shape) return String is (To_String (S.Why));
   function Kept (S : Lobe_Shape) return Natural is (S.Kept);
   function Scatter (S : Lobe_Shape) return Real is (S.Scatter);

   --  Measuring: the shape scaled by its tips, and the extents of the points
   --  it then has in the tool frame.

   function Across_Variance (C : Mat3; D : Vec3) return Real is
     (Real'Max (0.0, (C (1, 1) + C (2, 2) + C (3, 3) - D * (C * D)) / 2.0));
   --  The variance of a point along one direction across a unit D, averaged
   --  over the two.

   --  Everything measured from one lobe's fit leans on the same fitted
   --  motion: an error Delta of the motion moves each depth by -Coupling .
   --  Delta (Own_And_Coupling), and so the scale and every extent, all
   --  together. So each carries its own variance apart from that, and its
   --  gradient G with respect to the motion; whatever is computed from them
   --  adds their gradients first, and the motion's part of its variance is
   --  G . Covariance G at the end, raised by the fit's scatter.

   function Shared (S : Lobe_Shape; G, H : Vec5) return Real is ((S.Scatter ** 2) * (G * (S.Covariance * H)));
   function Shared (S : Lobe_Shape; G : Vec5) return Real is (Shared (S, G, G));

   type Scaled is record
      Ok       : Boolean := False;
      Why      : Unbounded_String;
      Scale    : Real := 0.0;                   --  metres per unit of the fit
      Own      : Real := 0.0;                   --  its variance from the tips and the anchors' own matches
      Gradient : Vec5 := [others => 0.0];       --  with respect to the fitted motion
   end record;

   function Terms_At (S : Lobe_Shape; I : Positive) return Terms is
     (Terms_Of (S.Sightings.Constant_Reference.Element (I), S.Rotation, S.Shift, S.Across_1, S.Across_2,
                S.Points.Constant_Reference.Element (I).Depth));

   function Coupling_At (S : Lobe_Shape; I : Positive) return Vec5 is
      Own      : Real;
      Coupling : Vec5;
   begin
      Own_And_Coupling (S.Sightings.Constant_Reference.Element (I), Terms_At (S, I), Own, Coupling);
      return Coupling;
   end Coupling_At;

   --  The scale from the two tips: each tip's distance along its line of
   --  sight over the fit's distance there, the open tip's at the open end
   --  and the closed tip's at the closed end, where the motion carries its
   --  point. Both lean on the same motion, so their difference is tested
   --  with what they share taken out, and they are combined as two
   --  correlated estimates of one quantity. A lobe followed part of its
   --  travel (not Closed_Is_End) has its second anchor where it was last
   --  followed, which no press measured: the open tip's estimate alone.
   function Scale_Of (S : Lobe_Shape; Open_Tip, Closed_Tip : Point_Estimate) return Scaled is
      Sights  : Sighting_Array renames S.Sightings.Constant_Reference.Element.all;
      Points  : Point_Array renames S.Points.Constant_Reference.Element.all;
      R       : Scaled;
      U_O     : constant Vec3 := Sights (S.Open_Anchor).Open;
      Depth_O : constant Real := Points (S.Open_Anchor).Depth;
      Rho_O   : constant Real := (Open_Tip.Mean - S.Eye) * U_O;
      T_C     : constant Terms := Terms_At (S, S.Closed_Anchor);
      V_C     : constant Vec3 := Sights (S.Closed_Anchor).Closed;
      Lean_C  : constant Real := V_C * T_C.Lever;
      Rho_C   : constant Real := (Closed_Tip.Mean - S.Eye) * V_C;
      --  The closed range moves with the motion directly, by Closed . dW,
      --  and through its depth, Lean_C times the depth's move -C . Delta.
      Range_G : Vec5;
   begin
      declare
         C         : constant Vec5 := Coupling_At (S, S.Closed_Anchor);
         Q         : constant Vec3 := Points (S.Closed_Anchor).Depth * T_C.Lever;
         By_Motion : constant Jacobian := [Cross ([1.0, 0.0, 0.0], Q), Cross ([0.0, 1.0, 0.0], Q),
                                           Cross ([0.0, 0.0, 1.0], Q), S.Across_1, S.Across_2];
      begin
         for K in 1 .. 5 loop
            Range_G (K) := V_C * By_Motion (K) - Lean_C * C (K);
         end loop;
      end;
      if not S.Closed_Is_End then
         if Depth_O <= 0.0 or else Rho_O <= 0.0 then
            R.Why := To_Unbounded_String ("its open tip lies behind its eye");
            return R;
         end if;
         declare
            S_O : constant Real := Rho_O / Depth_O;
         begin
            R.Ok := True;
            R.Scale := S_O;
            R.Own := (S_O ** 2) * ((Sigma_Along (Open_Tip.Covariance, U_O) / Rho_O) ** 2
                                   + (S.Scatter ** 2) * Points (S.Open_Anchor).Own / Depth_O ** 2);
            R.Gradient := (S_O / Depth_O) * Coupling_At (S, S.Open_Anchor);
         end;
         return R;
      end if;
      if Depth_O <= 0.0 or else T_C.Range_At <= 0.0 or else Rho_O <= 0.0 or else Rho_C <= 0.0 then
         R.Why := To_Unbounded_String ("a tip lies behind its eye");
         return R;
      end if;
      declare
         S_O   : constant Real := Rho_O / Depth_O;
         S_C   : constant Real := Rho_C / T_C.Range_At;
         Own_O : constant Real :=
           (S_O ** 2) * ((Sigma_Along (Open_Tip.Covariance, U_O) / Rho_O) ** 2
                         + (S.Scatter ** 2) * Points (S.Open_Anchor).Own / Depth_O ** 2);
         Own_C : constant Real :=
           (S_C ** 2) * ((Sigma_Along (Closed_Tip.Covariance, V_C) / Rho_C) ** 2
                         + (S.Scatter ** 2) * (Lean_C ** 2) * Points (S.Closed_Anchor).Own / T_C.Range_At ** 2);
         G_O   : constant Vec5 := (S_O / Depth_O) * Coupling_At (S, S.Open_Anchor);
         G_C   : constant Vec5 := (-S_C / T_C.Range_At) * Range_G;
         V_Cl  : constant Real := Own_C + Shared (S, G_C);
         C_OC  : constant Real := Shared (S, G_O, G_C);
         --  The variance of their difference: what they share cancels.
         Apart : constant Real := Own_O + Own_C + Shared (S, G_O - G_C);
      begin
         if Significant (S_O - S_C, Sqrt (Apart)) then
            R.Why := To_Unbounded_String
              ("its tips at the two openings disagree on how far it is from its eye ("
               & Driver.Log.Image (S_O, 4) & " and " & Driver.Log.Image (S_C, 4) & " a unit, +- "
               & Driver.Log.Image (Sqrt (Apart), 4) & " apart)");
            return R;
         end if;
         declare
            --  The least-variance combination of two correlated estimates.
            W_O : constant Real := (V_Cl - C_OC) / Apart;
            W_C : constant Real := 1.0 - W_O;
         begin
            R.Ok := True;
            R.Scale := W_O * S_O + W_C * S_C;
            R.Own := (W_O ** 2) * Own_O + (W_C ** 2) * Own_C;
            R.Gradient := W_O * G_O + W_C * G_C;
         end;
      end;
      return R;
   end Scale_Of;

   --  One end of a lobe's points along a direction: the farthest kept point
   --  that way, and two things about where the true end is from it.
   --
   --  The surface goes on past the point's pixel to somewhere short of the
   --  next pixel out, which does not show it: by anything up to one pixel's
   --  step along the surface, uniformly. How far a step goes along the
   --  direction is how fast the coordinate changes across the image around
   --  the point: on a face the eye sees at a slant, far; on a face across
   --  the direction, nothing but its noise.
   --
   --  Each point is off by its own noise along the direction: the farthest
   --  of many is pushed out by the largest of their noises, which the family
   --  of all of them bounds (Driver.Uncertain, Tests), and is short of the
   --  true end by at most its own, which the single test bounds.
   --
   --  The end is the middle of both ranges, and its sigmas are what make Z
   --  of them reach their ends: the noise's, which pushes every end outward
   --  alike, so an extent adds its two ends' up; and the pixel's, which is
   --  each end's own.
   type Extreme is record
      Index : Positive := 1;
      Value : Real := 0.0;
      Noise : Real := 0.0;   --  sigma
      Pixel : Real := 0.0;   --  sigma
   end record;

   function Sigma (E : Extreme) return Real is (Sqrt (E.Noise ** 2 + E.Pixel ** 2));

   function End_Of (S : Lobe_Shape; Scale : Real; D : Vec3; Upper : Boolean) return Extreme is
      Sights : Sighting_Array renames S.Sightings.Constant_Reference.Element.all;
      Points : Point_Array renames S.Points.Constant_Reference.Element.all;
      Best   : Natural := 0;
      Y_Best : Real := 0.0;
      function Y (I : Positive) return Real is (D * (S.Eye + (Scale * Points (I).Depth) * Sights (I).Open));
   begin
      for I in Points'Range loop
         if Points (I).Kept and then (Best = 0 or else (if Upper then Y (I) > Y_Best else Y (I) < Y_Best)) then
            Best := I;
            Y_Best := Y (I);
         end if;
      end loop;
      declare
         Lean      : constant Real := D * Sights (Best).Open;
         Own       : constant Real := Scale * S.Scatter * Sqrt (Points (Best).Own) * abs Lean;
         Single    : constant Real := Threshold (Scalar_Gate);
         Family    : constant Real := Threshold (Scalar_Gate (Tests => S.Kept));
         Outward   : constant Real := (if Upper then 1.0 else -1.0);
         Here      : constant Driver.Images.Pixel := Sights (Best).At_Pixel;
         --  How fast the coordinate changes across the image there: the
         --  plane through the point and its kept neighbours in the eight
         --  around it, least squares, its slope a pixel along its steepest
         --  way; the step to the next pixel out, were the surface to go on.
         --  The largest single neighbour would take a diagonal's step for
         --  one pixel's.
         Normal    : Mat3 := [others => [others => 0.0]];
         Right     : Vec3 := Zero3;
         Fitted    : Boolean := False;
         Footprint : Real;
         Slope_U, Slope_V : Real := 0.0;   --  the plane's: how the coordinate changes a pixel across, and down
         Beyond    : Real := 0.0;          --  how much farther out the lobe's pixels with no told depth may take it
      begin
         for I in Points'Range loop
            if Points (I).Kept and then abs (Sights (I).At_Pixel.U - Here.U) <= 1.0
              and then abs (Sights (I).At_Pixel.V - Here.V) <= 1.0
            then
               declare
                  Row : constant Vec3 :=
                    [1.0, Sights (I).At_Pixel.U - Here.U, Sights (I).At_Pixel.V - Here.V];
               begin
                  Normal := Normal + Outer (Row, Row);
                  Right := Right + (Y (I) - Y_Best) * Row;
               end;
            end if;
         end loop;
         declare
            Lower    : Mat3;
            Definite : Boolean;
         begin
            Driver.Numerics.Dense.Cholesky (Normal, Lower, Definite);
            if Definite then
               declare
                  Plane : constant Vec3 := Driver.Numerics.Dense.Cholesky_Solve (Lower, Right);
               begin
                  Footprint := Sqrt (Plane (2) ** 2 + Plane (3) ** 2);
                  Slope_U := Plane (2);
                  Slope_V := Plane (3);
                  Fitted := True;
               end;
            end if;
         end;
         --  A point with too few neighbours to fit a plane has only what a
         --  pixel spans across its line of sight.
         if not Fitted then
            Footprint := Scale * Points (Best).Depth * Sights (Best).Pitch * Sqrt (Real'Max (0.0, 1.0 - Lean ** 2));
         end if;
         --  The lobe goes on past the point where its pixels show it and no kept sighting tells their depth (its
         --  pixels at the open end, Lobe_Shape.Pixels: unfollowed, or followed to no depth that fits): over those
         --  joined to the point through such pixels, as far out as the plane through it carries the coordinate
         --  there (without a plane, a pixel's span a pixel), and anywhere up to that, uniformly, as across the
         --  step past an end below. hand.shape.turn (10-18): a jaw turned 20 degrees about its root, the lower
         --  half of its end face followed to nothing, and its width along the hinge came out 0.0102 +- 0.0003
         --  where it is 0.0200.
         if Driver.Images.Width (S.Pixels) > 0 then
            declare
               W     : constant Natural := Driver.Images.Width (S.Pixels);
               H     : constant Natural := Driver.Images.Height (S.Pixels);
               Told  : Driver.Images.Mask := Driver.Images.Create (W, H);
               Been  : Driver.Images.Mask := Driver.Images.Create (W, H);
               type Place_Array is array (Positive range <>) of Natural;
               type Place_Access is access Place_Array;
               procedure Free is new Ada.Unchecked_Deallocation (Place_Array, Place_Access);
               Queue : Place_Access := new Place_Array (1 .. W * H);
               Head, Tail : Natural := 0;
               X0    : constant Integer := Integer (Real'Floor (Here.U));
               Y0    : constant Integer := Integer (Real'Floor (Here.V));
               function Inside (X, Y : Integer) return Boolean is (X >= 0 and then Y >= 0 and then X < W and then Y < H);
            begin
               for I in Points'Range loop
                  if Points (I).Kept then
                     declare
                        X : constant Integer := Integer (Real'Floor (Sights (I).At_Pixel.U));
                        Y : constant Integer := Integer (Real'Floor (Sights (I).At_Pixel.V));
                     begin
                        if Inside (X, Y) then
                           Driver.Images.Include (Told, X, Y);
                        end if;
                     end;
                  end if;
               end loop;
               if Inside (X0, Y0) then
                  Driver.Images.Include (Been, X0, Y0);
                  Tail := Tail + 1;
                  Queue (Tail) := Y0 * W + X0;
               end if;
               while Head < Tail loop
                  Head := Head + 1;
                  declare
                     X : constant Integer := Queue (Head) mod W;
                     Y : constant Integer := Queue (Head) / W;
                  begin
                     for Dy in -1 .. 1 loop
                        for Dx in -1 .. 1 loop
                           if Inside (X + Dx, Y + Dy) and then not Driver.Images.Contains (Been, X + Dx, Y + Dy)
                             and then Driver.Images.Contains (S.Pixels, X + Dx, Y + Dy)
                             and then not Driver.Images.Contains (Told, X + Dx, Y + Dy)
                           then
                              Driver.Images.Include (Been, X + Dx, Y + Dy);
                              Tail := Tail + 1;
                              Queue (Tail) := (Y + Dy) * W + X + Dx;
                              Beyond := Real'Max
                                (Beyond,
                                 (if Fitted
                                  then Outward * (Real (X + Dx - X0) * Slope_U + Real (Y + Dy - Y0) * Slope_V)
                                  else Footprint * Real (Integer'Max (abs (X + Dx - X0), abs (Y + Dy - Y0)))));
                           end if;
                        end loop;
                     end loop;
                  end;
               end loop;
               Free (Queue);
            end;
         end if;
         --  Along an edge many pixels end within a step of the end, each at
         --  its own part of a pixel from it, and the farthest of them is the
         --  nearest to it: of N such, it falls short of the end by the gap
         --  above the largest of N uniform values, on average a step over
         --  N + 1 (a single point, half a step).
         declare
            N   : Natural := 0;
            Gap : Real;
         begin
            for I in Points'Range loop
               if Points (I).Kept and then abs (Y (I) - Y_Best) <= Footprint then
                  N := N + 1;
               end if;
            end loop;
            Gap := Footprint / Real (N + 1);
            return (Index => Best,
                    Value => Y_Best + Outward * (Gap - (Family - Single) * Own / 2.0 + Beyond / 2.0),
                    Noise => (Family + Single) / (2.0 * Single) * Own,
                    Pixel => Sqrt (Gap ** 2 * Real (N) / Real (N + 2) + Beyond ** 2 / 12.0));
         end;
      end;
   end End_Of;


   function Point_At (S : Lobe_Shape; Scale : Real; I : Positive) return Vec3 is
     (S.Eye + (Scale * S.Points.Constant_Reference.Element (I).Depth)
              * S.Sightings.Constant_Reference.Element (I).Open);

   --  The extent of a lobe's points along a unit direction: its two ends,
   --  with the motion's uncertainty they share and the scale's. Span is the
   --  line from the low end to the high one, which a turn of the direction
   --  acts on; the caller knows how the direction is uncertain.
   function Extent
     (S : Lobe_Shape; Sc : Scaled; D : Vec3; Low, High : out Extreme; Span : out Vec3) return Estimate
   is
      Sights : Sighting_Array renames S.Sightings.Constant_Reference.Element.all;
   begin
      Low := End_Of (S, Sc.Scale, D, Upper => False);
      High := End_Of (S, Sc.Scale, D, Upper => True);
      Span := Point_At (S, Sc.Scale, High.Index) - Point_At (S, Sc.Scale, Low.Index);
      declare
         Value : constant Real := High.Value - Low.Value;
         --  The extent is the scale times the fit's extent: the motion moves
         --  both, the depths of its ends by -Coupling . Delta each.
         G     : constant Vec5 :=
           (Value / Sc.Scale) * Sc.Gradient
           - Sc.Scale * (Real'(D * Sights (High.Index).Open) * Coupling_At (S, High.Index)
                         - Real'(D * Sights (Low.Index).Open) * Coupling_At (S, Low.Index));
      begin
         return (Value              => Value,
                 Sigma              => Sqrt ((High.Noise + Low.Noise) ** 2 + High.Pixel ** 2 + Low.Pixel ** 2
                                             + (Value / Sc.Scale) ** 2 * Sc.Own + Shared (S, G)),
                 Degrees_Of_Freedom => 0);
      end;
   end Extent;

   function Standing (E : Estimate) return Boolean is
     (E.Value > 0.0 and then Significant (E.Value, E.Sigma, E.Degrees_Of_Freedom));
   --  A size stands out of its own uncertainty.

   function With_Turn (E : Estimate; Turn_Sigma, Lever : Real) return Estimate is
     ((Value => E.Value, Sigma => Sqrt (E.Sigma ** 2 + (Turn_Sigma * Lever) ** 2), Degrees_Of_Freedom => 0));
   --  An extent along a direction known to Turn_Sigma radians, which a turn
   --  of the direction moves by Lever per radian.

   --  Where one point lies along a direction: what of its uncertainty is
   --  the motion's and the scale's (its own match's is the end's).
   function Position_Variance (S : Lobe_Shape; Sc : Scaled; I : Positive; D : Vec3) return Real is
      Sights : Sighting_Array renames S.Sightings.Constant_Reference.Element.all;
      Points : Point_Array renames S.Points.Constant_Reference.Element.all;
      Lean   : constant Real := D * Sights (I).Open;
   begin
      return (Points (I).Depth * Lean) ** 2 * Sc.Own
        + Shared (S, Lean * (Points (I).Depth * Sc.Gradient - Sc.Scale * Coupling_At (S, I)));
   end Position_Variance;

   --  The narrowest way across points in a plane: the unit direction across
   --  one edge of their convex hull along which they spread least, which
   --  is where any set's least width lies. The hull is Andrew's monotone
   --  chain.
   type Flat is record
      X, Y : Real := 0.0;
   end record;

   type Flat_Array is array (Positive range <>) of Flat;
   type Flat_Access is access Flat_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flat_Array, Flat_Access);

   function "<" (A, B : Flat) return Boolean is (A.X < B.X or else (A.X = B.X and then A.Y < B.Y));
   procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Flat, Flat_Array);

   function Narrowest (Points : in out Flat_Array) return Flat is
      function Turn (O, A, B : Flat) return Real is ((A.X - O.X) * (B.Y - O.Y) - (A.Y - O.Y) * (B.X - O.X));
      Hull  : Flat_Access := new Flat_Array (1 .. 2 * Points'Length);
      K     : Natural := 0;
      Best  : Flat := (X => 1.0, Y => 0.0);
      Least : Real := Real'Last;
   begin
      Sort (Points);
      for P of Points loop
         while K >= 2 and then Turn (Hull (K - 1), Hull (K), P) <= 0.0 loop
            K := K - 1;
         end loop;
         K := K + 1;
         Hull (K) := P;
      end loop;
      declare
         Lower : constant Positive := K + 1;
      begin
         for I in reverse Points'First .. Points'Last - 1 loop
            while K >= Lower and then Turn (Hull (K - 1), Hull (K), Points (I)) <= 0.0 loop
               K := K - 1;
            end loop;
            K := K + 1;
            Hull (K) := Points (I);
         end loop;
      end;
      --  The chain ends where it began: K - 1 corners, each edge to the next.
      for E in 1 .. K - 1 loop
         declare
            Dx : constant Real := Hull (E + 1).X - Hull (E).X;
            Dy : constant Real := Hull (E + 1).Y - Hull (E).Y;
            L  : constant Real := Sqrt (Dx ** 2 + Dy ** 2);
         begin
            if L > 0.0 then
               declare
                  N    : constant Flat := (X => -Dy / L, Y => Dx / L);
                  Low  : Real := Real'Last;
                  High : Real := Real'First;
               begin
                  for V in 1 .. K - 1 loop
                     Low := Real'Min (Low, N.X * Hull (V).X + N.Y * Hull (V).Y);
                     High := Real'Max (High, N.X * Hull (V).X + N.Y * Hull (V).Y);
                  end loop;
                  if High - Low < Least then
                     Least := High - Low;
                     Best := N;
                  end if;
               end;
            end if;
         end;
      end loop;
      Free (Hull);
      return Best;
   end Narrowest;

   function Unit_Axis (K : Positive) return Vec3 is ([for I in 1 .. 3 => (if I = K then 1.0 else 0.0)]);


   function Measure (Shapes : Lobe_Shape_Array; Tips : Tip_Array) return Hand_Size is
      Result : Hand_Size (Shapes'Length);

      type Lobe_Work is record
         Ok       : Boolean := False;
         Scale    : Scaled;
         Closing  : Vec3 := Zero3;      --  the way its open tip moves as it closes, by its fitted motion
         Close_Sd : Real := Real'Last;
         Across_Sd : Real := 0.0;       --  radians: how far its way across may turn in the plane across its closing
         Middle   : Vec3 := Zero3;      --  of its points
         Across   : Vec3 := Zero3;      --  its narrowest way across its closing direction
         Along    : Vec3 := Zero3;      --  across both, toward its tip
         Hinged   : Boolean := False;   --  it turns about a hinge: its turn stands out of its own uncertainty
      end record;

      Work     : array (Shapes'Range) of Lobe_Work;
      All_Ok   : Boolean := Shapes'Length > 0;
      Pointing : Vec3 := Zero3;

      procedure Say (L : Positive; What : String) is
      begin
         Append (Result.Why, "lobe" & L'Image & ": " & What & "; ");
      end Say;

      subtype Mat35 is Real_Matrix (1 .. 3, 1 .. 5);
   begin
      for L in Shapes'Range loop
         declare
            S      : Lobe_Shape renames Shapes (L);
            Opened : constant Point_Estimate := Tips (L, Open);
            Shut   : constant Point_Estimate := Tips (L, Closed_Empty);
         begin
            if not S.Fitted then
               Say (L, (if Length (S.Why) > 0 then To_String (S.Why) else "its shape is not seen"));
            elsif not Known (Opened) or else (S.Closed_Is_End and then not Known (Shut)) then
               Say (L, (if S.Closed_Is_End then "its tip is not pressed at both openings yet"
                        else "its tip is not pressed at the open opening yet"));
            else
               Work (L).Scale := Scale_Of (S, Opened, Shut);
               if not Work (L).Scale.Ok then
                  Say (L, To_String (Work (L).Scale.Why));
               else
                  declare
                     Scale  : constant Real := Work (L).Scale.Scale;
                     From   : constant Vec3 := Opened.Mean - S.Eye;
                     Turned : constant Vec3 := S.Rotation * From;
                     --  The open tip's own way to the closed end: the same
                     --  point of the lobe, which the closed tip need not be.
                     Travel : constant Vec3 := Turned + Scale * S.Shift - From;
                     --  How the motion's error moves it: directly, and
                     --  through the scale.
                     By     : constant Mat35 :=
                       [for I in 1 .. 3 =>
                          [for K in 1 .. 5 =>
                             (case K is
                                 when 1 .. 3 => Cross (Unit_Axis (K), Turned) (I),
                                 when 4      => Scale * S.Across_1 (I),
                                 when others => Scale * S.Across_2 (I))
                             + S.Shift (I) * Work (L).Scale.Gradient (K)]];
                     Lag    : constant Mat3 := S.Rotation - Identity3;
                     Spread : constant Mat3 :=
                       (S.Scatter ** 2) * (By * S.Covariance * Transpose (By))
                       + Work (L).Scale.Own * Outer (S.Shift, S.Shift)
                       + Lag * Opened.Covariance * Transpose (Lag);
                  begin
                     if abs Travel = 0.0
                       or else not Significant (abs Travel, Sigma_Along (Spread, Unit (Travel)))
                     then
                        Say (L, "its tip does not move between the openings");
                     else
                        Work (L).Closing := Unit (Travel);
                        Work (L).Close_Sd := Sqrt (Across_Variance (Spread, Work (L).Closing)) / abs Travel;
                        declare
                           Sights : Sighting_Array renames S.Sightings.Constant_Reference.Element.all;
                           Points : Point_Array renames S.Points.Constant_Reference.Element.all;
                           B1, B2 : Vec3;
                           Flats  : Flat_Access := new Flat_Array (1 .. S.Kept);
                           K      : Natural := 0;
                           Sum    : Vec3 := Zero3;
                           Way    : Flat;
                        begin
                           Across (Work (L).Closing, B1, B2);
                           for I in Points'Range loop
                              if Points (I).Kept then
                                 declare
                                    X : constant Vec3 := S.Eye + (Scale * Points (I).Depth) * Sights (I).Open;
                                 begin
                                    K := K + 1;
                                    Flats (K) := (X => B1 * X, Y => B2 * X);
                                    Sum := Sum + X;
                                 end;
                              end if;
                           end loop;
                           Work (L).Middle := (1.0 / Real (K)) * Sum;
                           Way := Narrowest (Flats (1 .. K));
                           Free (Flats);
                           Work (L).Across := Way.X * B1 + Way.Y * B2;
                           --  A lobe that turns between its ends turns about a hinge, and its tip goes round it: its
                           --  closing direction is across the hinge, and the hinge is its way across, the lobe pointing
                           --  from it to its tip. Taken from the narrowest way across what is seen of it, a jaw seen from
                           --  above on its top face (hand.shape.turn) had its way across turned 0.15 rad toward whatever
                           --  of its sides showed, and its depth along the axis came 4.3 sigma off. The hinge is the
                           --  fitted turn's axis, when the turn stands out of its own uncertainty; it may turn in the
                           --  plane across the closing direction by its axis' uncertainty across it over the angle. A lobe
                           --  that slides has no hinge, and its way across is the narrowest.
                           declare
                              Turn_Vector : constant Vec3 := Driver.Numerics.Log (S.Rotation);
                              Turn        : constant Real := abs Turn_Vector;
                           begin
                              if Turn > 0.0 then
                                 declare
                                    Hinge     : constant Vec3 := (1.0 / Turn) * Turn_Vector;
                                    Turn_Cov  : constant Mat3 :=
                                      (S.Scatter ** 2) * [for I in 1 .. 3 => [for J in 1 .. 3 => S.Covariance (I, J)]];
                                    Turn_Sd   : constant Real := Sigma_Along (Turn_Cov, Hinge);
                                 begin
                                    if Significant (Turn, Turn_Sd) then
                                       Work (L).Across := Unit (Hinge - Real'(Hinge * Work (L).Closing) * Work (L).Closing);
                                       Work (L).Across_Sd := Sqrt (Across_Variance (Turn_Cov, Hinge)) / Turn;
                                       Work (L).Hinged := True;
                                    end if;
                                 end;
                              end if;
                           end;
                           Work (L).Along := Cross (Work (L).Closing, Work (L).Across);
                           if Real'(Work (L).Along * (Opened.Mean - Work (L).Middle)) < 0.0 then
                              Work (L).Along := -Work (L).Along;
                           end if;
                           Work (L).Ok := True;
                           Pointing := Pointing + Work (L).Along;
                        end;
                     end if;
                  end;
               end if;
            end if;
            All_Ok := All_Ok and then Work (L).Ok;
         end;
      end loop;
      if abs Pointing = 0.0 then
         return Result;
      end if;
      declare
         Axis     : constant Vec3 := Unit (Pointing);
         Axis_Var : Real := 0.0;
         Count    : Natural := 0;
      begin
         --  First the lobes' sizes and how well each lobe gives the axis: to
         --  the tilt of its closing direction, and, in the plane across it,
         --  to its width's uncertainty over its length.
         for L in Shapes'Range loop
            if Work (L).Ok then
               declare
                  S         : Lobe_Shape renames Shapes (L);
                  Sc        : Scaled renames Work (L).Scale;
                  Scale     : constant Real := Sc.Scale;
                  F         : constant Vec3 := Work (L).Closing;
                  Opened    : constant Point_Estimate := Tips (L, Open);
                  Low, High : Extreme;
                  Span      : Vec3;
                  Width     : Estimate := Extent (S, Sc, Work (L).Across, Low, High, Span);
               begin
                  --  The narrowest way across is least where it is, so turning
                  --  it within the plane does not widen it; tilting the closing
                  --  direction tilts the plane, by the span's part along it.
                  Width := With_Turn (Width, Work (L).Close_Sd, abs Real'(F * Span));
                  declare
                     Thick : Estimate := Extent (S, Sc, F, Low, High, Span);
                     Face  : constant Real := High.Value - F * Opened.Mean;
                     Face_V : constant Real :=
                       Sigma (High) ** 2 + Sigma_Along (Opened.Covariance, F) ** 2
                       + Position_Variance (S, Sc, High.Index, F)
                       + (Work (L).Close_Sd * abs Cross (F, Point_At (S, Scale, High.Index) - Opened.Mean)) ** 2;
                     Long  : constant Real :=
                       End_Of (S, Scale, Work (L).Along, Upper => True).Value
                       - End_Of (S, Scale, Work (L).Along, Upper => False).Value;
                  begin
                     Thick := With_Turn (Thick, Work (L).Close_Sd, abs Cross (F, Span));
                     --  A size is measured when it stands out of its own
                     --  uncertainty: a fit that explains the lobe's sightings
                     --  only at several times their noise raises that noise
                     --  with it, until its sizes say nothing.
                     --  The axis from every lobe whose closing is known, its sizes measured or not: the depth rests on
                     --  the axis and the lobes' root ends, not on their widths.
                     Axis_Var := Axis_Var + Work (L).Close_Sd ** 2
                       + (if Work (L).Across_Sd > 0.0 then Work (L).Across_Sd ** 2
                          elsif Long > 0.0 and then Width.Sigma < Real'Last then (Width.Sigma / Long) ** 2 else 0.0);
                     Count := Count + 1;
                     --  A lobe that turns about a hinge turns its faces as it turns: a side that faces its eye at one
                     --  opening faces away at the other, and the two views see across it along the hinge only where
                     --  both show it (hand.shape.turn, 10-18, its rig ray-cast: both views see the jaw's top, the upper
                     --  half of its root end, the lower half behind the palm, and the upper ends of its sides, and its
                     --  width came out 0.0102 +- 0.0003 where it is 0.0200).
                     Result.Sizes (L - Shapes'First + 1) :=
                       (Width     => (if not Work (L).Hinged and then Standing (Width) and then Standing (Thick)
                                      then Width else (others => <>)),
                        Thickness => (if Standing (Thick) and then (Work (L).Hinged or else Standing (Width))
                                      then Thick else (others => <>)),
                        Face      => (if Standing (Thick) and then (Work (L).Hinged or else Standing (Width))
                                      then (Value => Face, Sigma => Sqrt (Face_V), Degrees_Of_Freedom => 0)
                                      else (others => <>)));
                     if Work (L).Hinged then
                        Say (L, "its width along its hinge is not measured: it turns its faces as it turns, and what both"
                             & " openings show of it need not reach across it along the hinge");
                     end if;
                     if not Standing (Thick) or else not (Work (L).Hinged or else Standing (Width)) then
                        Say (L, "its sizes do not stand out of their own uncertainty (width "
                             & Driver.Log.Image (Width.Value, 4) & " +- " & Driver.Log.Image (Width.Sigma, 4)
                             & ", thickness " & Driver.Log.Image (Thick.Value, 4) & " +- "
                             & Driver.Log.Image (Thick.Sigma, 4) & "): one rigid motion explains its sightings only at"
                             & Real'Image (S.Scatter) & " times their noise");
                     end if;
                  end;
               end;
            end if;
         end loop;
         if not All_Ok or else Count = 0 then
            Append (Result.Why, "the axis waits for every lobe; ");
            return Result;
         end if;
         Result.Axis := (Unit_Vector => Axis, Sigma => Sqrt (Axis_Var) / Real (Count));
      end;
      return Result;
   end Measure;

   function Motion_Between (Sightings : Sighting_Array) return Two_View_Motion is
      S : constant Lobe_Shape :=
        Fit ([0.0, 0.0, 0.0], Sightings, Sightings'First, Sightings'First, Bordered => False, Anchored => False);
   begin
      if not S.Fitted then
         return (Fitted => False, Why => S.Why, others => <>);
      end if;
      declare
         Turn : constant Vec3 := Driver.Numerics.Log (S.Rotation);
         Cov  : constant Mat3 := (S.Scatter ** 2) * [for I in 1 .. 3 => [for J in 1 .. 3 => S.Covariance (I, J)]];
         --  The shift's two components across itself (Across_1, Across_2) carry its direction's uncertainty.
         Shift_Var : constant Real := (S.Scatter ** 2) * Real'Max (S.Covariance (4, 4), S.Covariance (5, 5));
      begin
         return (Fitted   => True,
                 Why      => Null_Unbounded_String,
                 Turn     => Turn,
                 Turn_Cov => Cov,
                 Shift    => S.Shift,
                 Shift_Sd => Sqrt (Shift_Var),
                 Kept     => S.Kept,
                 Scatter  => S.Scatter);
      end;
   end Motion_Between;


end Driver.Robot.Hand.Shape;
