with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Numerics.Dense;

package body Driver.Robot.Hand.Touch is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Plane_Unknowns : constant := 3;
   --  A surface's error is its offset and its two tilts.

   function Lever (P : Press; Tip : Vec3) return Vec3 is (Cross (P.Tool.Pose.Rotation * Tip, P.Surface.Normal));
   --  A turn of the tool by a small rotation vector w moves the tip by
   --  w x (R Tip), so its height by w . Lever.

   function Pose_Known (P : Press) return Boolean is
     (Geometry.Known (P.Surface) and then P.Tool.Position_Covariance (1, 1) < Real'Last
      and then P.Tool.Rotation_Covariance (1, 1) < Real'Last);

   function Pose_Sigma (P : Press; Tip : Vec3) return Real is
     (Sqrt (P.Surface.Normal * (P.Tool.Position_Covariance * P.Surface.Normal)
            + Lever (P, Tip) * (P.Tool.Rotation_Covariance * Lever (P, Tip))));
   --  The height noise a press has from the arm's pose alone.

   function Residual (P : Press; Tip : Vec3) return Estimate is
      Y : constant Vec3 := P.Tool.Pose * Tip;
   begin
      if not Pose_Known (P) then
         return (Value => Geometry.Height (P.Surface, Y), Sigma => Real'Last, Degrees_Of_Freedom => 0);
      end if;
      --  The pose noise lies along the surface's normal; the surface adds its
      --  own uncertainty and the degrees of freedom it was fitted with.
      return Geometry.Height
        (P.Surface, Point_Estimate'(Mean       => Y,
                                    Covariance => (Pose_Sigma (P, Tip) ** 2) * Outer (P.Surface.Normal, P.Surface.Normal)));
   end Residual;

   --  The equations shared by both models. Press I says
   --     Row (I) . q + Plane (I) . p (Surface_Of (I)) + Offset (I) = 0
   --  with sigma Sigma (I), q the K unknowns of the tip and p the error of the
   --  surface it pressed on, which every press on that surface shares; each
   --  surface's estimate adds three prior equations on its own p.
   type Surface_Index is array (Positive range <>) of Positive;
   type Plane_Rows is array (Positive range <>, Positive range <>) of Real;

   type Equations (Presses, K : Positive) is record
      Row        : Real_Matrix (1 .. Presses, 1 .. K);
      Plane      : Plane_Rows (1 .. Presses, 1 .. Plane_Unknowns);
      Surface_Of : Surface_Index (1 .. Presses);
      Offset     : Real_Vector (1 .. Presses);
      Sigma      : Real_Vector (1 .. Presses);
   end record;

   type Selection is array (Positive range <>) of Boolean;
   type Count_Array is array (Positive range <>) of Natural;

   type Solution (K : Positive) is record
      Ok         : Boolean := False;
      Q          : Real_Vector (1 .. K);
      Covariance : Real_Matrix (1 .. K, 1 .. K);
      Degrees_Of_Freedom : Natural := 0;   --  of the covariance's scale; 0 when the predicted noise set it
      Used       : Natural := 0;
      Scatter    : Real := Real'Last;
   end record;

   --  The distinct surfaces pressed on, in order of first use.
   function Surfaces_Of (Presses : Press_Array; Index : out Surface_Index) return Natural
     with Pre => Index'Length = Presses'Length;

   function Surfaces_Of (Presses : Press_Array; Index : out Surface_Index) return Natural is
      Count : Natural := 0;
      Found : Surface_Index (Presses'Range) := [others => 1];
   begin
      for I in Presses'Range loop
         declare
            Same : Natural := 0;
         begin
            for J in Presses'First .. I - 1 loop
               if Geometry."=" (Presses (J).Surface, Presses (I).Surface) then
                  Same := Found (J);
                  exit;
               end if;
            end loop;
            if Same = 0 then
               Count := Count + 1;
               Same := Count;
            end if;
            Found (I) := Same;
         end;
      end loop;
      Index := Found;
      return Count;
   end Surfaces_Of;

   --  Weighted least squares over the selected presses and the surfaces'
   --  priors, and for every selected press its deleted residual (predicted
   --  by everything else) in units of its own deleted sigma, the rest's
   --  scatter raised to at least the predicted noise, with the degrees of
   --  freedom that sigma rests on (none when the predicted noise set it).
   procedure Solve_Selected
     (E        : Equations;
      Presses  : Press_Array;
      Surfaces : Positive;
      Chosen   : Selection;
      S        : out Solution;
      Deleted  : out Real_Vector;
      Dof_Of   : out Count_Array)
     with Pre => Chosen'Length = E.Presses and then Deleted'Length = E.Presses and then Dof_Of'Length = E.Presses;

   procedure Solve_Selected
     (E        : Equations;
      Presses  : Press_Array;
      Surfaces : Positive;
      Chosen   : Selection;
      S        : out Solution;
      Deleted  : out Real_Vector;
      Dof_Of   : out Count_Array)
   is
      M      : Natural := 0;
      Index  : array (1 .. E.Presses) of Natural := [others => 0];
      Prior  : constant Natural := Plane_Unknowns * Surfaces;
      Width  : constant Positive := E.K + Prior;
   begin
      S := (K => E.K, Ok => False, Q => [others => 0.0], Covariance => [others => [others => 0.0]],
            Degrees_Of_Freedom => 0, Used => 0, Scatter => Real'Last);
      Deleted := [others => 0.0];
      Dof_Of := [others => 0];
      for I in 1 .. E.Presses loop
         if Chosen (Chosen'First + I - 1) then
            M := M + 1;
            Index (M) := I;
         end if;
      end loop;
      --  One press beyond the tip's unknowns checks the others.
      if M <= E.K then
         return;
      end if;
      declare
         Dof   : constant Natural := M - E.K;
         Noise : Real := 1.0;
         --  The presses' variance in units of their predicted one: never
         --  below it, raised to what they actually scatter when that is more.
         A     : Real_Matrix (1 .. M + Prior, 1 .. Width);
         B     : Real_Vector (1 .. M + Prior);
         Q     : Real_Vector (1 .. Width);
         Full  : Boolean;

         --  The presses' rows with their noise scaled by Noise, and each
         --  surface's prior, whitened: the offset alone, the two tilts through
         --  the inverse of their covariance's Cholesky factor. The priors
         --  keep their own uncertainty whatever the presses' scatter.
         procedure Assemble;

         procedure Assemble is
         begin
            A := [others => [others => 0.0]];
            B := [others => 0.0];
            for J in 1 .. M loop
               declare
                  I     : constant Positive := Index (J);
                  Base  : constant Natural := E.K + Plane_Unknowns * (E.Surface_Of (I) - 1);
                  Sigma : constant Real := E.Sigma (I) * Sqrt (Noise);
               begin
                  for C in 1 .. E.K loop
                     A (J, C) := E.Row (I, C) / Sigma;
                  end loop;
                  for C in 1 .. Plane_Unknowns loop
                     A (J, Base + C) := E.Plane (I, C) / Sigma;
                  end loop;
                  B (J) := -E.Offset (I) / Sigma;
               end;
            end loop;
            for Surface in 1 .. Surfaces loop
               declare
                  P    : Geometry.Plane_Estimate;
                  Base : constant Natural := E.K + Plane_Unknowns * (Surface - 1);
                  Row  : constant Natural := M + Plane_Unknowns * (Surface - 1);
               begin
                  for I in Presses'Range loop
                     if E.Surface_Of (I - Presses'First + 1) = Surface then
                        P := Presses (I).Surface;
                        exit;
                     end if;
                  end loop;
                  declare
                     L11 : constant Real := Sqrt (P.Tilt_11);
                     L21 : constant Real := P.Tilt_12 / L11;
                     L22 : constant Real := Sqrt (Real'Max (P.Tilt_22 - L21 * L21, Real'Model_Small));
                  begin
                     A (Row + 1, Base + 1) := 1.0 / P.Offset_Sigma;
                     A (Row + 2, Base + 2) := 1.0 / L11;
                     A (Row + 3, Base + 2) := -L21 / (L11 * L22);
                     A (Row + 3, Base + 3) := 1.0 / L22;
                  end;
               end;
            end loop;
         end Assemble;

         Previous : Real := Real'Last;
      begin
         --  The noise scale is the fixed point at which the chi square of all
         --  rows matches its degrees of freedom; iterate until it moves by a
         --  negligible part of itself, or stops moving less.
         loop
            Assemble;
            Driver.Numerics.Dense.Least_Squares (A, B, Q, Full);
            if not Full then
               return;
            end if;
            declare
               Res    : constant Real_Vector := A * Q - B;
               Next   : constant Real := Real'Max (1.0, Noise * (Res * Res) / Real (Dof));
               Change : constant Real := abs (Next - Noise);
            begin
               exit when Change <= Driver.Conventions.Unchanged_Fraction * Noise or else Change >= Previous;
               Previous := Change;
               Noise := Next;
            end;
         end loop;
         declare
            Info : constant Real_Matrix := Transpose (A) * A;
            Inv  : constant Real_Matrix := Inverse (Info);
            Res  : constant Real_Vector := A * Q - B;
            Chi  : constant Real := Res * Res;
         begin
            S.Q := Q (1 .. E.K);
            S.Used := M;
            S.Scatter := Noise * Chi / Real (Dof);
            S.Degrees_Of_Freedom := (if Noise > 1.0 then Dof else 0);
            for R in 1 .. E.K loop
               for C in 1 .. E.K loop
                  S.Covariance (R, C) := Inv (R, C);
               end loop;
            end loop;
            S.Ok := True;
            for J in 1 .. M loop
               declare
                  Row      : constant Real_Vector := [for C in 1 .. Width => A (J, C)];
                  Leverage : constant Real := Row * (Inv * Row);
                  Free     : constant Real := 1.0 - Leverage;
                  --  The others' noise in the same units, at least the predicted one.
                  Scale    : Real := 1.0 / Noise;
               begin
                  if Free > Real'Model_Epsilon then
                     --  Removing this press removes Res^2 / (1 - h) from the chi square.
                     if Dof > 1 and then (Chi - Res (J) ** 2 / Free) / Real (Dof - 1) > Scale then
                        Scale := (Chi - Res (J) ** 2 / Free) / Real (Dof - 1);
                        Dof_Of (Index (J)) := Dof - 1;
                     end if;
                     Deleted (Index (J)) := Res (J) / Sqrt (Scale * Free);
                  else
                     --  A press nothing else checks cannot be judged by the others.
                     Deleted (Index (J)) := 0.0;
                  end if;
               end;
            end loop;
         end;
      end;
   end Solve_Selected;

   --  Builds the equations at the current estimate (their sigmas depend on
   --  where the tip is), solves, and drops the press whose deleted residual
   --  is the most significant, one at a time, until none is.
   generic
      with procedure Build (Q : Real_Vector; E : in out Equations);
   procedure Select_And_Solve
     (E        : in out Equations;
      Presses  : Press_Array;
      Surfaces : Positive;
      Start    : Real_Vector;
      S        : out Solution;
      Stopped  : out Natural;
      Sunk     : out Natural);

   procedure Select_And_Solve
     (E        : in out Equations;
      Presses  : Press_Array;
      Surfaces : Positive;
      Start    : Real_Vector;
      S        : out Solution;
      Stopped  : out Natural;
      Sunk     : out Natural)
   is
      Chosen  : Selection (1 .. E.Presses) := [others => True];
      Deleted : Real_Vector (1 .. E.Presses);
      Dof_Of  : Count_Array (1 .. E.Presses);
      Q       : Real_Vector := Start;
   begin
      Stopped := 0;
      Sunk := 0;
      loop
         --  Reweight at the estimate until it moves by a negligible part of
         --  its own uncertainty, or stops shrinking the move.
         declare
            Previous : Real := Real'Last;
         begin
            loop
               Build (Q, E);
               Solve_Selected (E, Presses, Surfaces, Chosen, S, Deleted, Dof_Of);
               if not S.Ok then
                  return;
               end if;
               declare
                  Move : constant Real_Vector := S.Q - Q;
                  Size : constant Real := Move * (Inverse (S.Covariance) * Move);
               begin
                  Q := S.Q;
                  exit when Size <= Driver.Conventions.Unchanged_Fraction ** 2 or else Size >= Previous;
                  Previous := Size;
               end;
            end loop;
         end;
         declare
            Worst : Natural := 0;
         begin
            for I in 1 .. E.Presses loop
               if Chosen (I) and then Significant (Deleted (I), 1.0, Dof_Of (I))
                 and then (Worst = 0 or else abs Deleted (I) > abs Deleted (Worst))
               then
                  Worst := I;
               end if;
            end loop;
            exit when Worst = 0;
            --  Equations are heights above the surface: a positive residual
            --  left the tip above it, a negative one below.
            if Deleted (Worst) > 0.0 then
               Stopped := Stopped + 1;
            else
               Sunk := Sunk + 1;
            end if;
            Chosen (Worst) := False;
         end;
      end loop;
   end Select_And_Solve;

   --  The press's height and how it moves with its surface's error p:
   --  a higher surface lowers the tip's height above it.
   procedure Fill_Surface (E : in out Equations; I : Positive; P : Press; Y : Vec3);

   procedure Fill_Surface (E : in out Equations; I : Positive; P : Press; Y : Vec3) is
   begin
      E.Plane (I, 1) := -1.0;
      E.Plane (I, 2) := -(P.Surface.Tangent_1 * (Y - P.Surface.Centre));
      E.Plane (I, 3) := -(P.Surface.Tangent_2 * (Y - P.Surface.Centre));
   end Fill_Surface;

   function Fit_On_Ray (Presses : Press_Array; Origin : Point_Estimate; Direction : Direction_Estimate)
     return Fit_Result
   is
      O : constant Vec3 := Origin.Mean;
      U : constant Vec3 := Direction.Unit_Vector;
      E : Equations (Presses => Presses'Length, K => 1);

      procedure Build (Q : Real_Vector; E : in out Equations);

      procedure Build (Q : Real_Vector; E : in out Equations) is
         X : constant Vec3 := O + Q (Q'First) * U;
      begin
         for I in 1 .. E.Presses loop
            declare
               P    : Press renames Presses (Presses'First + I - 1);
               Lift : constant Vec3 := Transpose (P.Tool.Pose.Rotation) * P.Surface.Normal;
               Y    : constant Vec3 := P.Tool.Pose * X;
            begin
               --  Height = Lift . (O + s U) + constant: linear in s.
               E.Row (I, 1) := Lift * U;
               E.Offset (I) := Geometry.Height (P.Surface, Y) - (Lift * U) * Q (Q'First);
               E.Sigma (I) := Pose_Sigma (P, X);
               Fill_Surface (E, I, P, Y);
            end;
         end loop;
      end Build;

      procedure Solve is new Select_And_Solve (Build);

      S        : Solution (K => 1);
      Stopped  : Natural;
      Sunk     : Natural;
      Result   : Fit_Result;
      Surfaces : Natural;
   begin
      if Presses'Length < 2 or else not Known (Origin) or else Direction.Sigma >= Real'Last then
         return Result;
      end if;
      for P of Presses loop
         if not Pose_Known (P) then
            return Result;
         end if;
      end loop;
      Surfaces := Surfaces_Of (Presses, E.Surface_Of);
      Solve (E, Presses, Surfaces, [1 => 0.0], S, Stopped, Sunk);
      Result.Stopped := Stopped;
      Result.Sunk := Sunk;
      if not S.Ok or else S.Q (1) <= 0.0 then
         return Result;
      end if;
      declare
         Dist  : constant Real := S.Q (1);
         Var_S : constant Real := S.Covariance (1, 1);
      begin
         Result.Ok := True;
         Result.Used := S.Used;
         Result.Scatter := S.Scatter;
         Result.Distance := (Value => Dist, Sigma => Sqrt (Var_S), Degrees_Of_Freedom => S.Degrees_Of_Freedom);
         Result.Tip :=
           (Mean       => O + Dist * U,
            Covariance => Origin.Covariance + (Dist * Direction.Sigma) ** 2 * (Identity3 - Outer (U, U))
                          + Var_S * Outer (U, U));
      end;
      return Result;
   end Fit_On_Ray;

   function Fit_Free (Presses : Press_Array) return Fit_Result is
      E : Equations (Presses => Presses'Length, K => 3);

      procedure Build (Q : Real_Vector; E : in out Equations);

      procedure Build (Q : Real_Vector; E : in out Equations) is
         X : constant Vec3 := Q;
      begin
         for I in 1 .. E.Presses loop
            declare
               P    : Press renames Presses (Presses'First + I - 1);
               Lift : constant Vec3 := Transpose (P.Tool.Pose.Rotation) * P.Surface.Normal;
               Y    : constant Vec3 := P.Tool.Pose * X;
            begin
               for C in Lift'Range loop
                  E.Row (I, C) := Lift (C);
               end loop;
               E.Offset (I) := Geometry.Height (P.Surface, Y) - Lift * X;
               E.Sigma (I) := Pose_Sigma (P, X);
               Fill_Surface (E, I, P, Y);
            end;
         end loop;
      end Build;

      procedure Solve is new Select_And_Solve (Build);

      S        : Solution (K => 3);
      Stopped  : Natural;
      Sunk     : Natural;
      Result   : Fit_Result;
      Surfaces : Natural;
   begin
      for P of Presses loop
         if not Pose_Known (P) then
            return Result;
         end if;
      end loop;
      if Presses'Length = 0 then
         return Result;
      end if;
      Surfaces := Surfaces_Of (Presses, E.Surface_Of);
      Solve (E, Presses, Surfaces, Zero3, S, Stopped, Sunk);
      Result.Stopped := Stopped;
      Result.Sunk := Sunk;
      if not S.Ok then
         return Result;
      end if;
      Result.Ok := True;
      Result.Used := S.Used;
      Result.Scatter := S.Scatter;
      Result.Tip := (Mean => S.Q, Covariance => S.Covariance);
      Result.Distance := Unknown;
      return Result;
   end Fit_Free;

end Driver.Robot.Hand.Touch;
