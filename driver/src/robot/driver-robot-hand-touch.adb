with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Numerics.Dense;

package body Driver.Robot.Hand.Touch is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Plane_Unknowns : constant := 3;
   Point_Unknowns : constant := 3;
   --  A surface's error is its offset and its two tilts; a free tip is a point.

   function Pose_Known (Tool : Pose_Estimate) return Boolean is
     (Tool.Position_Covariance (1, 1) < Real'Last and then Tool.Rotation_Covariance (1, 1) < Real'Last);

   function Pose_Sigma (Tool : Pose_Estimate; Tip, Normal : Vec3) return Real is
      --  A turn of the tool by a small rotation vector w moves the tip by
      --  w x (R Tip), so its height by w . Lever.
      Lever : constant Vec3 := Cross (Tool.Pose.Rotation * Tip, Normal);
   begin
      return Sqrt (Normal * (Tool.Position_Covariance * Normal) + Lever * (Tool.Rotation_Covariance * Lever));
   end Pose_Sigma;
   --  The height noise a press has from the arm's pose alone.

   function With_Tangents (Centre, Normal : Vec3) return Geometry.Plane_Estimate is
      N    : constant Vec3 := Unit (Normal);
      --  Tangents from the coordinate axis furthest from the normal.
      Axis : constant Vec3 :=
        (if abs N (1) <= abs N (2) and then abs N (1) <= abs N (3) then [1.0, 0.0, 0.0]
         elsif abs N (2) <= abs N (3) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
      T1   : constant Vec3 := Unit (Cross (N, Axis));
   begin
      return (Centre => Centre, Normal => N, Tangent_1 => T1, Tangent_2 => Cross (N, T1), others => <>);
   end With_Tangents;

   function Moved (P : Geometry.Plane_Estimate; Offset, Tilt_1, Tilt_2 : Real) return Geometry.Plane_Estimate is
      --  The plane that is Offset higher at the centre and rises by Tilt_1
      --  and Tilt_2 per unit along the tangents (the corrections the press
      --  equations solve for); the tangents turn with it, so the tilts'
      --  covariance keeps its meaning.
      N  : constant Vec3 := Unit (P.Normal - Tilt_1 * P.Tangent_1 - Tilt_2 * P.Tangent_2);
      Along : constant Real := P.Tangent_1 * N;
      T1 : constant Vec3 := Unit (P.Tangent_1 - Along * N);
      Q  : Geometry.Plane_Estimate := P;
   begin
      Q.Centre := P.Centre + Offset * P.Normal;
      Q.Normal := N;
      Q.Tangent_1 := T1;
      Q.Tangent_2 := Cross (N, T1);
      return Q;
   end Moved;

   type Flags is array (Positive range <>) of Boolean;
   type Index_Array is array (Positive range <>) of Natural;

   --  Everything sized by presses lives on the heap: the estimates also run
   --  in the decider's task, whose stack is small. What is sized by the
   --  tips' and surfaces' unknowns stays.
   type Flags_Access is access Flags;
   type Index_Access is access Index_Array;
   type Vector_Access is access Real_Vector;
   type Matrix_Access is access Real_Matrix;
   type Result_Access is access Fit_Result;
   procedure Free is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Vector, Vector_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Fit_Result, Result_Access);

   function Fit
     (Presses  : Press_Array;
      Sights   : Sight_Array;
      Surfaces : Surface_Prior_Array;
      As       : Model := On_Sight) return Fit_Result
   is
      J : constant Natural := Sights'Length;
      K : constant Natural := Surfaces'Length;
      W : constant Positive := (if As = On_Sight then 1 else Point_Unknowns);
      --  The unknowns of a tip: the distance along its line, or its point.

      Result : Result_Access := new Fit_Result (Presses => Presses'Length, Sights => J, Surfaces => K);

      function Sight_Of (P : Press) return Positive is (P.Sight - Sights'First + 1);
      function Surface_Of (P : Press) return Positive is (P.Surface - Surfaces'First + 1);
      function Line (S : Positive) return Ray_Estimate is (Sights (Sights'First + S - 1));
      function Prior (F : Positive) return Surface_Prior is (Surfaces (Surfaces'First + F - 1));

      --  The current estimate: every tip's unknowns, every surface's plane,
      --  and each measured surface's correction to its prior.
      Tip_Q      : Real_Vector (1 .. W * J) := [others => 0.0];
      Nominal    : Plane_Array (1 .. K);
      Correction : Real_Vector (1 .. Plane_Unknowns * K) := [others => 0.0];

      function Tip_At (S : Positive) return Vec3 is
        (if As = On_Sight then Line (S).Origin.Mean + Tip_Q (S) * Line (S).Direction.Unit_Vector
         else Vec3 (Tip_Q (W * (S - 1) + 1 .. W * S)));

      Chosen  : Flags_Access := new Flags'(Presses'Range => True);
      Deleted : Vector_Access := new Real_Vector'(Presses'Range => 0.0);
      Dof_Of  : Index_Access := new Index_Array'(Presses'Range => 0);
      Stopped : Index_Array (1 .. J) := [others => 0];
      Sunk    : Index_Array (1 .. J) := [others => 0];

      --  The tips and surfaces some chosen press bears on, with the column
      --  of their first unknown (0: none).
      Column_Of_Sight   : Index_Array (1 .. J);
      Column_Of_Surface : Index_Array (1 .. K);
      Unknowns          : Natural := 0;
      Prior_Rows        : Natural := 0;
      Chosen_Count      : Natural := 0;

      procedure Lay_Out is
      begin
         Column_Of_Sight := [others => 0];
         Column_Of_Surface := [others => 0];
         Unknowns := 0;
         Prior_Rows := 0;
         Chosen_Count := 0;
         for I in Presses'Range loop
            if Chosen (I) then
               Chosen_Count := Chosen_Count + 1;
            end if;
         end loop;
         for S in 1 .. J loop
            if (for some I in Presses'Range => Chosen (I) and then Sight_Of (Presses (I)) = S) then
               Column_Of_Sight (S) := Unknowns + 1;
               Unknowns := Unknowns + W;
            end if;
         end loop;
         for F in 1 .. K loop
            if (for some I in Presses'Range => Chosen (I) and then Surface_Of (Presses (I)) = F) then
               Column_Of_Surface (F) := Unknowns + 1;
               Unknowns := Unknowns + Plane_Unknowns;
               if Prior (F).Measured then
                  Prior_Rows := Prior_Rows + Plane_Unknowns;
               end if;
            end if;
         end loop;
      end Lay_Out;

      --  The last step's solution: the tips' unknowns and the surfaces'
      --  corrections, their covariance, the noise scale and its degrees of
      --  freedom (none when the predicted noise set it).
      Solution_Q   : Real_Vector (1 .. W * J + Plane_Unknowns * K);
      Solution_Cov : Real_Matrix (1 .. W * J + Plane_Unknowns * K, 1 .. W * J + Plane_Unknowns * K);
      Noise        : Real := 1.0;
      Noise_Dof    : Natural := 0;

      --  One weighted least-squares solve at the current estimate over the
      --  chosen presses and the measured surfaces' priors. Every press says
      --     Row . x + Offset = 0 with sigma Sigma,
      --  x the tips' unknowns and the surfaces' corrections (an unknown
      --  surface's relative to its current plane, a measured one's relative
      --  to its prior). Also every chosen press's residual predicted from all
      --  the others, in units of its own deleted sigma.
      procedure Solve (Ok : out Boolean) is
         Rows : constant Natural := Chosen_Count + Prior_Rows;
      begin
         Ok := False;
         --  One equation beyond the unknowns checks the others.
         if Unknowns = 0 or else Rows <= Unknowns then
            return;
         end if;
         declare
            Dof   : constant Positive := Rows - Unknowns;
            A     : Matrix_Access := new Real_Matrix (1 .. Rows, 1 .. Unknowns);
            B     : Vector_Access := new Real_Vector (1 .. Rows);
            Base  : Matrix_Access := new Real_Matrix'(1 .. Rows => [1 .. Unknowns => 0.0]);
            Rhs   : Vector_Access := new Real_Vector'(1 .. Rows => 0.0);
            Sigma : Vector_Access := new Real_Vector'(1 .. Rows => 1.0);
            Press_Of_Row : Index_Access := new Index_Array'(1 .. Rows => 0);
            Q     : Real_Vector (1 .. Unknowns);
            Full  : Boolean;
            Row   : Natural := 0;
            Scale : Real := 1.0;
            Previous : Real := Real'Last;
            procedure Solved;
            --  The solve itself; it may end early, and its rows are
            --  released after.
            procedure Solved is
            begin
               for I in Presses'Range loop
                  if Chosen (I) then
                     declare
                        P    : Press renames Presses (I);
                        S    : constant Positive := Sight_Of (P);
                        F    : constant Positive := Surface_Of (P);
                        X    : constant Vec3 := Tip_At (S);
                        Y    : constant Vec3 := P.Tool.Pose * X;
                        N    : constant Vec3 := Nominal (F).Normal;
                        Lift : constant Vec3 := Transpose (P.Tool.Pose.Rotation) * N;
                        H    : constant Real := N * (Y - Nominal (F).Centre);
                        C0   : constant Positive := Column_Of_Sight (S);
                        CF   : constant Positive := Column_Of_Surface (F);
                     begin
                        Row := Row + 1;
                        Press_Of_Row (Row) := I;
                        --  The height is linear in the tip's unknowns.
                        if As = On_Sight then
                           Base (Row, C0) := Lift * Line (S).Direction.Unit_Vector;
                           Rhs (Row) := -(H - Base (Row, C0) * Tip_Q (S));
                        else
                           for C in 1 .. Point_Unknowns loop
                              Base (Row, C0 + C - 1) := Lift (C);
                           end loop;
                           Rhs (Row) := -(H - Lift * X);
                        end if;
                        --  A higher surface lowers the tip's height above it.
                        --  A measured surface's plane here is its prior, and its
                        --  correction is solved whole; an unknown one's plane is the
                        --  current estimate, corrected by a step.
                        Base (Row, CF) := -1.0;
                        Base (Row, CF + 1) := -(Nominal (F).Tangent_1 * (Y - Nominal (F).Centre));
                        Base (Row, CF + 2) := -(Nominal (F).Tangent_2 * (Y - Nominal (F).Centre));
                        Sigma (Row) := Pose_Sigma (P.Tool, X, N);
                     end;
                  end if;
               end loop;
               --  Each measured surface's prior, whitened: the offset alone, the
               --  two tilts through the inverse of their covariance's Cholesky factor.
               for F in 1 .. K loop
                  if Column_Of_Surface (F) > 0 and then Prior (F).Measured then
                     declare
                        P   : constant Geometry.Plane_Estimate := Prior (F).Plane;
                        CF  : constant Positive := Column_Of_Surface (F);
                        L11 : constant Real := Sqrt (P.Tilt_11);
                        L21 : constant Real := P.Tilt_12 / L11;
                        L22 : constant Real := Sqrt (Real'Max (P.Tilt_22 - L21 * L21, Real'Model_Small));
                     begin
                        Base (Row + 1, CF) := 1.0 / P.Offset_Sigma;
                        Base (Row + 2, CF + 1) := 1.0 / L11;
                        Base (Row + 3, CF + 1) := -L21 / (L11 * L22);
                        Base (Row + 3, CF + 2) := 1.0 / L22;
                        Row := Row + Plane_Unknowns;
                     end;
                  end if;
               end loop;
               --  The presses' variance in units of their predicted one: never
               --  below it, raised to what they actually scatter when that is
               --  more; it is the fixed point at which the chi square of all rows
               --  matches its degrees of freedom. The priors keep their own.
               loop
                  for R in 1 .. Rows loop
                     declare
                        S : constant Real := (if Press_Of_Row (R) > 0 then Sigma (R) * Sqrt (Scale) else 1.0);
                     begin
                        for C in 1 .. Unknowns loop
                           A (R, C) := Base (R, C) / S;
                        end loop;
                        B (R) := Rhs (R) / S;
                     end;
                  end loop;
                  Driver.Numerics.Dense.Least_Squares (A.all, B.all, Q, Full);
                  if not Full then
                     return;
                  end if;
                  declare
                     Res    : constant Real_Vector := A.all * Q - B.all;
                     Next   : constant Real := Real'Max (1.0, Scale * (Res * Res) / Real (Dof));
                     Change : constant Real := abs (Next - Scale);
                  begin
                     exit when Change <= Driver.Conventions.Unchanged_Fraction * Scale or else Change >= Previous;
                     Previous := Change;
                     Scale := Next;
                  end;
               end loop;
               declare
                  Inv : constant Real_Matrix := Inverse (Transpose (A.all) * A.all);
                  Res : constant Real_Vector := A.all * Q - B.all;
                  Chi : constant Real := Res * Res;
               begin
                  --  An unknown the presses fix only to round-off has no variance.
                  if (for some R in 1 .. Unknowns => not (Inv (R, R) > 0.0)) then
                     return;
                  end if;
                  Noise := Scale;
                  Noise_Dof := (if Scale > 1.0 then Dof else 0);
                  Solution_Q := [others => 0.0];
                  Solution_Cov := [others => [others => 0.0]];
                  for R in 1 .. Unknowns loop
                     Solution_Q (R) := Q (R);
                     for C in 1 .. Unknowns loop
                        Solution_Cov (R, C) := Inv (R, C);
                     end loop;
                  end loop;
                  Deleted.all := [others => 0.0];
                  Dof_Of.all := [others => 0];
                  for R in 1 .. Rows loop
                     if Press_Of_Row (R) > 0 then
                        declare
                           Row_R    : constant Real_Vector := [for C in 1 .. Unknowns => A (R, C)];
                           Free     : constant Real := 1.0 - Row_R * (Inv * Row_R);
                           --  The others' noise in the same units, at least the predicted one.
                           Without  : Real := 1.0 / Scale;
                        begin
                           if Free > Real'Model_Epsilon then
                              --  Leaving this press out removes Res^2 / (1 - h) from the chi square.
                              if Dof > 1 and then (Chi - Res (R) ** 2 / Free) / Real (Dof - 1) > Without then
                                 Without := (Chi - Res (R) ** 2 / Free) / Real (Dof - 1);
                                 Dof_Of (Press_Of_Row (R)) := Dof - 1;
                              end if;
                              --  The residual is the tip's height above the surface.
                              Deleted (Press_Of_Row (R)) := Res (R) / Sqrt (Without * Free);
                           end if;
                        end;
                     end if;
                  end loop;
               end;
               Ok := True;
            end Solved;
         begin
            Solved;
            Free (A);
            Free (B);
            Free (Base);
            Free (Rhs);
            Free (Sigma);
            Free (Press_Of_Row);
         end;
      end Solve;

      --  Takes a solution into the estimate; the size of the move in units
      --  of its own uncertainty.
      function Take return Real is
         Move : Real_Vector (1 .. Unknowns) := [others => 0.0];
      begin
         for S in 1 .. J loop
            if Column_Of_Sight (S) > 0 then
               for C in 1 .. W loop
                  Move (Column_Of_Sight (S) + C - 1) := Solution_Q (Column_Of_Sight (S) + C - 1) - Tip_Q (W * (S - 1) + C);
                  Tip_Q (W * (S - 1) + C) := Solution_Q (Column_Of_Sight (S) + C - 1);
               end loop;
            end if;
         end loop;
         for F in 1 .. K loop
            if Column_Of_Surface (F) > 0 then
               declare
                  CF : constant Positive := Column_Of_Surface (F);
               begin
                  if Prior (F).Measured then
                     for C in 1 .. Plane_Unknowns loop
                        Move (CF + C - 1) := Solution_Q (CF + C - 1) - Correction (Plane_Unknowns * (F - 1) + C);
                        Correction (Plane_Unknowns * (F - 1) + C) := Solution_Q (CF + C - 1);
                     end loop;
                  else
                     for C in 1 .. Plane_Unknowns loop
                        Move (CF + C - 1) := Solution_Q (CF + C - 1);
                     end loop;
                     Nominal (F) := Moved (Nominal (F), Solution_Q (CF), Solution_Q (CF + 1), Solution_Q (CF + 2));
                  end if;
               end;
            end if;
         end loop;
         declare
            Cov : constant Real_Matrix := [for R in 1 .. Unknowns => [for C in 1 .. Unknowns => Solution_Cov (R, C)]];
         begin
            return Move * (Inverse (Cov) * Move);
         end;
      end Take;

      --  The start: each unknown surface's normal against the lines of sight
      --  pressed into it, and with every normal held, the distances and the
      --  unknown surfaces' offsets, linear in them.
      procedure Start (Ok : out Boolean) is
         Into     : array (1 .. K) of Vec3 := [others => Zero3];
         Columns  : constant Natural := J + K;
         Rows     : constant Natural := Presses'Length;
      begin
         Ok := False;
         for F in 1 .. K loop
            if Prior (F).Measured then
               Nominal (F) := Prior (F).Plane;
            end if;
         end loop;
         for P of Presses loop
            Into (Surface_Of (P)) := Into (Surface_Of (P)) - P.Tool.Pose.Rotation * Line (Sight_Of (P)).Direction.Unit_Vector;
         end loop;
         if Rows < Columns then
            return;
         end if;
         declare
            A    : Matrix_Access := new Real_Matrix'(1 .. Rows => [1 .. Columns => 0.0]);
            B    : Vector_Access := new Real_Vector'(1 .. Rows => 0.0);
            Q    : Real_Vector (1 .. Columns);
            Full : Boolean;
            R    : Natural := 0;
            Used : Flags (1 .. Columns) := [others => False];
         begin
            for P of Presses loop
               declare
                  S : constant Positive := Sight_Of (P);
                  F : constant Positive := Surface_Of (P);
                  N : constant Vec3 := (if Prior (F).Measured then Nominal (F).Normal else Unit (Into (F)));
                  O : constant Vec3 := P.Tool.Pose * Line (S).Origin.Mean;
               begin
                  R := R + 1;
                  A (R, S) := N * (P.Tool.Pose.Rotation * Line (S).Direction.Unit_Vector);
                  Used (S) := True;
                  if Prior (F).Measured then
                     B (R) := N * (Nominal (F).Centre - O);
                  else
                     A (R, J + F) := -1.0;
                     Used (J + F) := True;
                     B (R) := -(N * O);
                  end if;
               end;
            end loop;
            --  Columns nothing bears on are pinned, so the rest can be solved.
            declare
               Pinned : Matrix_Access := new Real_Matrix'(1 .. Rows + Columns => [1 .. Columns => 0.0]);
               Rhs    : Vector_Access := new Real_Vector'(1 .. Rows + Columns => 0.0);
            begin
               for I in 1 .. Rows loop
                  for C in 1 .. Columns loop
                     Pinned (I, C) := A (I, C);
                  end loop;
                  Rhs (I) := B (I);
               end loop;
               for C in 1 .. Columns loop
                  if not Used (C) then
                     Pinned (Rows + C, C) := 1.0;
                  end if;
               end loop;
               Driver.Numerics.Dense.Least_Squares (Pinned.all, Rhs.all, Q, Full);
               Free (Pinned);
               Free (Rhs);
            end;
            Free (A);
            Free (B);
            if not Full then
               return;
            end if;
            for S in 1 .. J loop
               Tip_Q (S) := Q (S);
            end loop;
            for F in 1 .. K loop
               if not Prior (F).Measured then
                  declare
                     N     : constant Vec3 := Unit (Into (F));
                     Sum   : Vec3 := Zero3;
                     Count : Natural := 0;
                  begin
                     for P of Presses loop
                        if Surface_Of (P) = F then
                           Sum := Sum + P.Tool.Pose * (Line (Sight_Of (P)).Origin.Mean
                                                       + Q (Sight_Of (P)) * Line (Sight_Of (P)).Direction.Unit_Vector);
                           Count := Count + 1;
                        end if;
                     end loop;
                     if Count > 0 then
                        --  Through the contact points' centroid, at the solved offset.
                        Nominal (F) := With_Tangents (Sum / Real (Count) - ((N * (Sum / Real (Count))) - Q (J + F)) * N, N);
                     end if;
                  end;
               end if;
            end loop;
         end;
         Ok := True;
      end Start;

      function Done return Fit_Result;
      --  The result, with what the fit kept per press released.

      function Done return Fit_Result is
      begin
         return Copy : constant Fit_Result := Result.all do
            Free (Result);
            Free (Chosen);
            Free (Deleted);
            Free (Dof_Of);
         end return;
      end Done;

   begin
      for P of Presses loop
         if not Pose_Known (P.Tool) then
            return Done;
         end if;
      end loop;
      for S in 1 .. J loop
         if not Known (Line (S).Origin) or else Line (S).Direction.Sigma >= Real'Last then
            return Done;
         end if;
      end loop;
      declare
         Started : Boolean;
      begin
         Start (Started);
         if not Started then
            return Done;
         end if;
      end;
      if As = Free then
         --  From the tips on their lines and the surfaces they gave.
         declare
            Along : constant Fit_Result := Fit (Presses, Sights, Surfaces, On_Sight);
         begin
            if not Along.Ok then
               return Done;
            end if;
            for S in 1 .. J loop
               declare
                  X : constant Vec3 := (if Along.Tips (S).Ok then Along.Tips (S).Tip.Mean else Line (S).Origin.Mean);
               begin
                  Tip_Q (W * (S - 1) + 1 .. W * S) := X;
               end;
            end loop;
            for F in 1 .. K loop
               if not Prior (F).Measured and then Geometry.Known (Along.Planes (F)) then
                  Nominal (F) := With_Tangents (Along.Planes (F).Centre, Along.Planes (F).Normal);
               end if;
            end loop;
         end;
      end if;
      --  Solve, reweighting at the estimate until it moves by a negligible
      --  part of its own uncertainty or stops shrinking the move; then drop
      --  the press whose deleted residual is the most significant, one at a
      --  time, until none is.
      loop
         declare
            Previous : Real := Real'Last;
            Solved   : Boolean;
         begin
            loop
               Lay_Out;
               Solve (Solved);
               if not Solved then
                  return Done;
               end if;
               declare
                  Size : constant Real := Take;
               begin
                  exit when Size <= Driver.Conventions.Unchanged_Fraction ** 2 or else Size >= Previous;
                  Previous := Size;
               end;
            end loop;
         end;
         declare
            Worst : Natural := 0;
         begin
            for I in Presses'Range loop
               if Chosen (I) and then Significant (Deleted (I), 1.0, Dof_Of (I))
                 and then (Worst = 0 or else abs Deleted (I) > abs Deleted (Worst))
               then
                  Worst := I;
               end if;
            end loop;
            exit when Worst = 0;
            --  Residuals are heights above the surface: positive left the tip
            --  above it, negative below.
            if Deleted (Worst) > 0.0 then
               Stopped (Sight_Of (Presses (Worst))) := Stopped (Sight_Of (Presses (Worst))) + 1;
            else
               Sunk (Sight_Of (Presses (Worst))) := Sunk (Sight_Of (Presses (Worst))) + 1;
            end if;
            Chosen (Worst) := False;
         end;
      end loop;
      Result.Ok := True;
      for I in Presses'Range loop
         Result.Agrees (I - Presses'First + 1) := Chosen (I);
      end loop;
      Result.Scatter := Noise;
      for S in 1 .. J loop
         Result.Tips (S).Stopped := Stopped (S);
         Result.Tips (S).Sunk := Sunk (S);
         if Column_Of_Sight (S) > 0 then
            declare
               C0 : constant Positive := Column_Of_Sight (S);
            begin
               Result.Tips (S).Ok := True;
               for I in Presses'Range loop
                  if Chosen (I) and then Sight_Of (Presses (I)) = S then
                     Result.Tips (S).Used := Result.Tips (S).Used + 1;
                  end if;
               end loop;
               if As = On_Sight then
                  declare
                     U     : constant Vec3 := Line (S).Direction.Unit_Vector;
                     Dist  : constant Real := Tip_Q (S);
                     Var_S : constant Real := Solution_Cov (C0, C0);
                  begin
                     Result.Tips (S).Distance := (Value => Dist, Sigma => Sqrt (Var_S), Degrees_Of_Freedom => Noise_Dof);
                     --  Along the line as the presses fixed it, across it as the eye did.
                     Result.Tips (S).Tip :=
                       (Mean       => Line (S).Origin.Mean + Dist * U,
                        Covariance => Line (S).Origin.Covariance
                                      + (Dist * Line (S).Direction.Sigma) ** 2 * (Identity3 - Outer (U, U))
                                      + Var_S * Outer (U, U));
                  end;
               else
                  Result.Tips (S).Tip :=
                    (Mean       => Tip_At (S),
                     Covariance => [for R in 1 .. Point_Unknowns => [for C in 1 .. Point_Unknowns => Solution_Cov (C0 + R - 1, C0 + C - 1)]]);
               end if;
            end;
         end if;
      end loop;
      for F in 1 .. K loop
         if Column_Of_Surface (F) > 0 then
            declare
               CF : constant Positive := Column_Of_Surface (F);
               P  : Geometry.Plane_Estimate :=
                 (if Prior (F).Measured
                  then Moved (Prior (F).Plane, Correction (Plane_Unknowns * F - 2), Correction (Plane_Unknowns * F - 1),
                              Correction (Plane_Unknowns * F))
                  else Nominal (F));
               --  Offset and tilts; moved to the point where they are uncorrelated.
               C11 : constant Real := Solution_Cov (CF, CF);
               C12 : constant Real := Solution_Cov (CF, CF + 1);
               C13 : constant Real := Solution_Cov (CF, CF + 2);
               T11 : constant Real := Solution_Cov (CF + 1, CF + 1);
               T12 : constant Real := Solution_Cov (CF + 1, CF + 2);
               T22 : constant Real := Solution_Cov (CF + 2, CF + 2);
               Det : constant Real := T11 * T22 - T12 * T12;
               A   : constant Real := -(T22 * C12 - T12 * C13) / Det;
               B   : constant Real := -(T11 * C13 - T12 * C12) / Det;
            begin
               P.Centre := P.Centre + A * P.Tangent_1 + B * P.Tangent_2;
               P.Offset_Sigma := Sqrt (Real'Max (C11 + A * C12 + B * C13, Real'Model_Small));
               P.Tilt_11 := T11;
               P.Tilt_12 := T12;
               P.Tilt_22 := T22;
               P.Scatter := Noise;
               P.Points := 0;
               for I in Presses'Range loop
                  if Chosen (I) and then Surface_Of (Presses (I)) = F then
                     P.Points := P.Points + 1;
                  end if;
               end loop;
               Result.Planes (F) := P;
            end;
         elsif Prior (F).Measured then
            Result.Planes (F) := Prior (F).Plane;
         end if;
      end loop;
      return Done;
   end Fit;

end Driver.Robot.Hand.Touch;
