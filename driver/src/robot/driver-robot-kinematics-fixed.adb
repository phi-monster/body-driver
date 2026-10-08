with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Errors;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Fixed is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Fit.Flag_Array;

   --  Everything sized by sightings or points lives on the heap: the fit runs
   --  in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
   type Flag_Access is access Fit.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Flag_Array, Flag_Access);
   type Vec_Array is array (Positive range <>) of Vec3;
   type Vec_Access is access Vec_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Vec_Array, Vec_Access);

   --  The step of a central difference: the cube root of the float's
   --  resolution, relative to the size of what is stepped.
   Difference_Step : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);

   --  Huber's weight and cost at Z on a residual in units of its noise.
   function Huber (Value : Real) return Real is
     (if abs Value <= Driver.Conventions.Z then 1.0 else Driver.Conventions.Z / abs Value);

   function Huber_Cost (Value : Real) return Real is
     (if abs Value <= Driver.Conventions.Z then 0.5 * Value ** 2
      else Driver.Conventions.Z * (abs Value - 0.5 * Driver.Conventions.Z));

   --  What the eye is: its lens and its pose (camera to world).
   type State is record
      L : Fit.Lens;
      R : Mat3 := Identity3;
      C : Vec3 := Zero3;
   end record;

   --  A change of the 12 terms, in the terms of the report: the lens's by the
   --  logarithms of the focal lengths and the others added, the turn about the
   --  camera's own axes after the pose, the centre added.
   function Moved (S : State; X : Real_Array) return State is
     ((L => (Fx => S.L.Fx * Exp (X (X'First)), Fy => S.L.Fy * Exp (X (X'First + 1)),
             Cx => S.L.Cx + X (X'First + 2), Cy => S.L.Cy + X (X'First + 3),
             K1 => S.L.K1 + X (X'First + 4), K2 => S.L.K2 + X (X'First + 5)),
       R => S.R * Exp ([X (X'First + 6), X (X'First + 7), X (X'First + 8)]),
       C => S.C + [X (X'First + 9), X (X'First + 10), X (X'First + 11)]));

   ---------------------------------------------------------------------------
   --  The fit

   procedure Fit_Eye
     (Points            : Point_Array;
      Poses             : Pose_Array;
      Sightings         : Sighting_Array;
      Common            : Driver.Numerics.Arrays.Real_Matrix;
      Common_Covariance : Driver.Numerics.Arrays.Real_Matrix;
      Width, Height     : Positive;
      Start_Lens        : Fit.Lens;
      Start_Pose        : Rigid;
      Report            : out Fit_Report)
   is
      N       : constant Natural := Sightings'Length;
      Shared  : constant Natural := Common'Length (2);
      --  The terms' resolution of a pixel: a residual below what the float
      --  can tell of a pixel's coordinate is no residual.
      Floor   : constant Real := Real'Model_Epsilon * Real (Width + Height);
      Behind  : constant Real := Real (Width + Height);

      World   : Vec_Access := new Vec_Array (1 .. N);   --  every sighting's point in the world
      Seen    : Flag_Access := new Fit.Flag_Array (1 .. N);    --  in front of the eye at the start
      Fits    : Flag_Access := new Fit.Flag_Array (1 .. N);    --  the sightings that fit now
      Sigma   : Real := 1.0;
      --  What each residual's point adds to its variance, in pixels squared, at the state last looked at
      --  (Point_Variance): a point whose position is uncertain tells less of the eye.
      Extra   : Real_Access := new Real_Array (1 .. 2 * N);

      --  The root of each point's own covariance, Own = Root * Transpose (Root), from the eigenvectors of its
      --  symmetric part with the negative eigenvalues set to zero. What a position's uncertainty makes of a pixel is
      --  then the squared length of Transpose (Root) times the pixel's slope, which cannot be negative. The quadratic
      --  form summed term by term can: a track the arm fit leaves with no depth at all (A17's first fit had one at
      --  1.7E97 units, its log depth uncertain by 2.5E6) has a covariance of 1E207, its terms cancel to less than the
      --  rounding of their sum, and Scale_Of took the square root of one plus that over the noise (the replay raised
      --  there). A covariance with a negative eigenvalue makes the same, and says nothing about the pixel there.
      type Root_Array is array (Positive range <>) of Mat3;

      function Root_Of (Own : Mat3) return Mat3 is
         Values  : Vec3;
         Vectors : Mat3;
         Result  : Mat3 := [others => [others => 0.0]];
      begin
         Driver.Numerics.Symmetric_Eigensystem (Own, Values, Vectors);
         for K in 1 .. 3 loop
            if Values (K) > 0.0 then
               for A in 1 .. 3 loop
                  Result (A, K) := Vectors (A, K) * Sqrt (Values (K));
               end loop;
            end if;
         end loop;
         return Result;
      end Root_Of;

      Roots : constant Root_Array (Points'Range) := [for P in Points'Range => Root_Of (Points (P).Own)];

      type Free_Terms is array (Positive range <>) of Positive;
      Without_Distortion : constant Free_Terms := [1, 2, 3, 4, 7, 8, 9, 10, 11, 12];
      With_Distortion    : constant Free_Terms := [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];

      --  Where the eye in state S sees sighting K's point, and whether it is in front.
      procedure Predict (S : State; K : Positive; U, V : out Real; Ahead : out Boolean) is
         P : constant Vec3 := Transpose (S.R) * (World (K) - S.C);
      begin
         Fit.Project (S.L, P, U, V, Ahead);
      end Predict;

      --  Every residual, across then down, of every sighting (a point behind the eye is as wrong as the picture
      --  is big).
      procedure Residuals (S : State; R : out Real_Array) is
      begin
         for K in 1 .. N loop
            declare
               U, V  : Real;
               Ahead : Boolean;
            begin
               Predict (S, K, U, V, Ahead);
               if Ahead then
                  R (R'First + 2 * K - 2) := U - Sightings (Sightings'First + K - 1).U;
                  R (R'First + 2 * K - 1) := V - Sightings (Sightings'First + K - 1).V;
               else
                  R (R'First + 2 * K - 2) := Behind;
                  R (R'First + 2 * K - 1) := Behind;
               end if;
            end;
         end loop;
      end Residuals;

      --  How each residual's pixel moves with the position of its point (in the frame of the link it rides on),
      --  carried through that position's uncertainty: Extra, at the eye in state S.
      procedure Point_Variance (S : State) is
      begin
         Extra.all := [others => 0.0];
         for K in 1 .. N loop
            declare
               Sg    : Sighting renames Sightings (Sightings'First + K - 1);
               Root  : Mat3 renames Roots (Sg.Point);
               Rot   : constant Mat3 := Poses (Sg.Pose).Rotation;
               Slope : array (1 .. 3, 0 .. 1) of Real := [others => [others => 0.0]];
            begin
               for Axis in 1 .. 3 loop
                  declare
                     Step : constant Real :=
                       Difference_Step * Real'Max (1.0, abs Points (Sg.Point).Position (Axis));
                     Dx   : Vec3 := Zero3;
                     Ua, Va, Ub, Vb   : Real;
                     Ahead_A, Ahead_B : Boolean;
                  begin
                     Dx (Axis) := Step;
                     Fit.Project (S.L, Transpose (S.R) * (World (K) + Rot * Dx - S.C), Ua, Va, Ahead_A);
                     Fit.Project (S.L, Transpose (S.R) * (World (K) - Rot * Dx - S.C), Ub, Vb, Ahead_B);
                     if Ahead_A and then Ahead_B then
                        Slope (Axis, 0) := (Ua - Ub) / (2.0 * Step);
                        Slope (Axis, 1) := (Va - Vb) / (2.0 * Step);
                     end if;
                  end;
               end loop;
               for C in 0 .. 1 loop
                  declare
                     Along : constant Vec3 := Transpose (Root) * [Slope (1, C), Slope (2, C), Slope (3, C)];
                  begin
                     Extra (2 * K - 1 + C) := Along * Along;
                  end;
               end loop;
            end;
         end loop;
      end Point_Variance;

      --  How much dearer residual I is than the noise alone makes it, as a factor on its standard deviation.
      function Scale_Of (I : Positive) return Real is (Sqrt (1.0 + Extra (I) / Sigma ** 2));

      --  The noise the residuals show, per direction: the robust spread of those, in units of what each one's
      --  point and the noise make of it, of the sightings in front of the eye at the start.
      function Measured (R : Real_Array) return Real is
         Count : Natural := 0;
      begin
         for K in 1 .. N loop
            if Seen (K) then
               Count := Count + 1;
            end if;
         end loop;
         if Count = 0 then
            return Floor;
         end if;
         declare
            Both : Real_Access := new Real_Array (1 .. 2 * Count);
            I    : Natural := 0;
         begin
            for K in 1 .. N loop
               if Seen (K) then
                  Both (I + 1) := R (R'First + 2 * K - 2) / (Sigma * Scale_Of (2 * K - 1));
                  Both (I + 2) := R (R'First + 2 * K - 1) / (Sigma * Scale_Of (2 * K));
                  I := I + 2;
               end if;
            end loop;
            return Result : constant Real := Real'Max (Floor, Sigma * Driver.Stats.Robust_Sigma (Both.all)) do
               Free (Both);
            end return;
         end;
      end Measured;

      --  The residuals, in pixels, and the Jacobian of the residuals in units of Sigma for the terms Which
      --  (central differences about S), both in units of what each residual's point and the noise make of it.
      procedure Linearize (S : State; Which : Free_Terms; R : out Real_Array; J : out Real_Matrix) is
         Plus, Minus : Real_Access := new Real_Array (1 .. 2 * N);
      begin
         Point_Variance (S);
         Residuals (S, R);
         for T in Which'Range loop
            declare
               Step : constant Real := Difference_Step;
               Xp   : Real_Array (1 .. Terms) := [others => 0.0];
               Xm   : Real_Array (1 .. Terms) := [others => 0.0];
            begin
               Xp (Which (T)) := Step;
               Xm (Which (T)) := -Step;
               Residuals (Moved (S, Xp), Plus.all);
               Residuals (Moved (S, Xm), Minus.all);
               for I in 1 .. 2 * N loop
                  J (I, T) := (Plus (I) - Minus (I)) / (2.0 * Step * Sigma * Scale_Of (I));
               end loop;
            end;
         end loop;
         for I in 1 .. 2 * N loop
            R (R'First + I - 1) := R (R'First + I - 1) / Scale_Of (I);
         end loop;
         Free (Plus);
         Free (Minus);
      end Linearize;

      --  The normal equations of the sightings that fit, with Huber weights: A, and the gradient G at S.
      procedure Normal_Equations
        (R : Real_Array; J : Real_Matrix; A : out Real_Matrix; G : out Real_Vector; Cost : out Real)
      is
         P : constant Natural := J'Length (2);
      begin
         A := [others => [others => 0.0]];
         G := [others => 0.0];
         Cost := 0.0;
         for K in 1 .. N loop
            if Fits (K) then
               for C in 0 .. 1 loop
                  declare
                     I : constant Positive := 2 * K - 1 + C;
                     Z : constant Real := R (R'First + I - 1) / Sigma;
                     W : constant Real := Huber (Z);
                  begin
                     Cost := Cost + Huber_Cost (Z);
                     for A1 in 1 .. P loop
                        G (A1) := G (A1) - W * J (I, A1) * Z;
                        for B1 in A1 .. P loop
                           A (A1, B1) := A (A1, B1) + W * J (I, A1) * J (I, B1);
                        end loop;
                     end loop;
                  end;
               end loop;
            end if;
         end loop;
         for A1 in 1 .. P loop
            for B1 in 1 .. A1 - 1 loop
               A (A1, B1) := A (B1, A1);
            end loop;
         end loop;
      end Normal_Equations;

      function Cost_At (S : State) return Real is
         R    : Real_Access := new Real_Array (1 .. 2 * N);
         Cost : Real := 0.0;
      begin
         Point_Variance (S);
         Residuals (S, R.all);
         for K in 1 .. N loop
            if Fits (K) then
               Cost := Cost + Huber_Cost (R (2 * K - 1) / (Sigma * Scale_Of (2 * K - 1)))
                            + Huber_Cost (R (2 * K) / (Sigma * Scale_Of (2 * K)));
            end if;
         end loop;
         Free (R);
         return Cost;
      end Cost_At;

      --  Robust least squares by Levenberg-Marquardt on the terms Which, at the noise Sigma, over the sightings
      --  that fit, until a step cannot move any combination of the terms by more than Unchanged_Fraction of its
      --  standard error. Changed says whether any step was taken.
      procedure Refine (S : in out State; Which : Free_Terms; Changed : out Boolean) is
         P      : constant Natural := Which'Length;
         R      : Real_Access := new Real_Array (1 .. 2 * N);
         J      : Matrix_Access := new Real_Matrix (1 .. 2 * N, 1 .. P);
         Lambda : Real := Real'Model_Epsilon;
         Cost0  : Real;
         Done   : Boolean := False;
      begin
         Changed := False;
         Cost0 := Cost_At (S);
         while not Done loop
            declare
               A : Real_Matrix (1 .. P, 1 .. P);
               G : Real_Vector (1 .. P);
               Cost : Real;
               Lowered : Boolean := False;
               Moves : Boolean := True;
            begin
               Linearize (S, Which, R.all, J.all);
               Normal_Equations (R.all, J.all, A, G, Cost);
               while Moves and then not Lowered loop
                  declare
                     D  : Real_Matrix := A;
                     L  : Real_Matrix (1 .. P, 1 .. P);
                     Pd : Boolean;
                  begin
                     for Q in 1 .. P loop
                        D (Q, Q) := A (Q, Q) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                     end loop;
                     Driver.Numerics.Dense.Cholesky (D, L, Pd);
                     if Pd then
                        declare
                           Step : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, G);
                           X    : Real_Array (1 .. Terms) := [others => 0.0];
                        begin
                           for Q in 1 .. P loop
                              X (Which (Which'First + Q - 1)) := Step (Q);
                           end loop;
                           declare
                              Next : constant State := Moved (S, X);
                              C1   : constant Real := Cost_At (Next);
                           begin
                              Moves := (for some Q in 1 .. P => Step (Q) /= 0.0);
                              if Moves and then C1 < Cost0 then
                                 Done := not Fit.Moves_The_Fit (Cost0 - C1);
                                 S := Next;
                                 Cost0 := C1;
                                 Lambda := Lambda / 2.0;
                                 Lowered := True;
                                 Changed := True;
                              end if;
                           end;
                        end;
                     end if;
                     if not Lowered then
                        Lambda := 2.0 * Lambda;
                     end if;
                  end;
               end loop;
               if not Lowered then
                  Done := True;
               end if;
            end;
         end loop;
         Free (R);
         Free (J);
      end Refine;

      --  The sightings that fit at S, at the noise the residuals show: neither direction's residual
      --  significant against it, in units of what its point and the noise make of it. Sigma is made anew from the
      --  residuals, until it stops changing by more than Unchanged_Fraction of itself.
      procedure Choose (S : State; Same : out Boolean) is
         R    : Real_Access := new Real_Array (1 .. 2 * N);
         Next : Fit.Flag_Array (1 .. N);
      begin
         Point_Variance (S);
         Residuals (S, R.all);
         for Round in 0 .. N loop
            declare
               Before : constant Real := Sigma;
            begin
               Sigma := Measured (R.all);
               exit when Round > 0 and then abs (Sigma - Before) <= Driver.Conventions.Unchanged_Fraction * Before;
            end;
         end loop;
         for K in 1 .. N loop
            Next (K) := Seen (K)
              and then not Driver.Uncertain.Significant (R (2 * K - 1) / Scale_Of (2 * K - 1), Sigma)
              and then not Driver.Uncertain.Significant (R (2 * K) / Scale_Of (2 * K), Sigma);
         end loop;
         Same := Next = Fits.all;
         Fits.all := Next;
         Free (R);
      end Choose;

      --  Choosing and refining, again and again, until the choice stops changing and the fit with it.
      procedure Settle (S : in out State; Which : Free_Terms) is
         Same, Changed : Boolean := False;
         Rounds : Natural := 0;
      begin
         loop
            Choose (S, Same);
            Refine (S, Which, Changed);
            Rounds := Rounds + 1;
            exit when (Same and then not Changed) or else Rounds > N;
         end loop;
         --  The last step moved the noise: the choice is made at it once more.
         Choose (S, Same);
      end Settle;

      Eye : State := (L => Start_Lens, R => Start_Pose.Rotation, C => Start_Pose.Translation);

      --  The covariance of the terms Which at S: the sandwich of the normal equations around the spread of the
      --  gradient. The spread has two parts. One is the residuals': their covariance as a falling function of the
      --  distance between the points' reference pixels (read from the residuals themselves, as Errors reads the
      --  matcher's errors), applied to every pair of gradient rows, in units of what each residual's own point and
      --  the noise make of it, so that what a point's own uncertainty adds to its sighting is in it. The other
      --  is what every point has in common (the arm fit's terms): how the gradient moves with each point's
      --  position, carried through the uncertainty of the terms that move all the points together. Covariance is
      --  Terms x Terms with zeros where the term is not free; Ok is False when the normal equations are not
      --  positive definite.
      procedure Covariance_At
        (S : State; Which : Free_Terms; Covariance : out Real_Matrix; Ok : out Boolean)
      is
         P    : constant Natural := Which'Length;
         R    : Real_Access := new Real_Array (1 .. 2 * N);
         J    : Matrix_Access := new Real_Matrix (1 .. 2 * N, 1 .. P);
         A    : Real_Matrix (1 .. P, 1 .. P);
         G    : Real_Vector (1 .. P);
         Cost : Real;
         L    : Real_Matrix (1 .. P, 1 .. P);
         Pd   : Boolean;
         Meat : Real_Matrix (1 .. P, 1 .. P) := [others => [others => 0.0]];

         --  How far apart the arm's own eye saw two sightings' points.
         function Apart (K, M : Positive) return Real is
            Pa : Point renames Points (Sightings (Sightings'First + K - 1).Point);
            Pb : Point renames Points (Sightings (Sightings'First + M - 1).Point);
         begin
            return Sqrt ((Pa.U0 - Pb.U0) ** 2 + (Pa.V0 - Pb.V0) ** 2);
         end Apart;

         --  The mean product of two sightings' errors, across with across and down with down, in units of the noise.
         function Product (K, M : Positive) return Real is
           ((R (2 * K - 1) * R (2 * M - 1) + R (2 * K) * R (2 * M)) / (2.0 * Sigma ** 2));

         procedure Add_Residual_Spread is
            Longest : Real := 0.0;
            Count   : Natural := 0;
            Own     : Real := 0.0;
         begin
            for K in 1 .. N loop
               if Fits (K) then
                  Count := Count + 1;
                  Own := Own + Product (K, K);
                  for M in K + 1 .. N loop
                     if Fits (M) then
                        Longest := Real'Max (Longest, Apart (K, M));
                     end if;
                  end loop;
               end if;
            end loop;
            Own := Own / Real (Count);
            declare
               Bins   : constant Positive := Natural (Real'Ceiling (Longest)) + 1;
               Mean   : Real_Array (1 .. Bins) := [others => 0.0];
               Weight : Real_Array (1 .. Bins) := [others => 0.0];
               Level  : Real_Array (1 .. Bins);
            begin
               for K in 1 .. N loop
                  if Fits (K) then
                     for M in K + 1 .. N loop
                        if Fits (M) then
                           declare
                              D : constant Positive := Natural (Real'Rounding (Apart (K, M))) + 1;
                           begin
                              Mean (D) := Mean (D) + Product (K, M);
                              Weight (D) := Weight (D) + 1.0;
                           end;
                        end if;
                     end loop;
                  end if;
               end loop;
               for D in 1 .. Bins loop
                  if Weight (D) > 0.0 then
                     Mean (D) := Mean (D) / Weight (D);
                  end if;
               end loop;
               Driver.Robot.Kinematics.Errors.Falling (Mean, Weight, Level);
               for K in 1 .. N loop
                  if Fits (K) then
                     for M in K .. N loop
                        if Fits (M) then
                           declare
                              D   : constant Positive := Natural (Real'Rounding (Apart (K, M))) + 1;
                              Cov : constant Real := (if K = M then Own else Real'Max (0.0, Level (D)));
                           begin
                              for C in 0 .. 1 loop
                                 declare
                                    Wk : constant Real := Huber (R (2 * K - 1 + C) / Sigma);
                                    Wm : constant Real := Huber (R (2 * M - 1 + C) / Sigma);
                                 begin
                                    for A1 in 1 .. P loop
                                       for B1 in 1 .. P loop
                                          if K = M then
                                             Meat (A1, B1) := Meat (A1, B1)
                                               + Cov * Wk * Wm * J (2 * K - 1 + C, A1) * J (2 * M - 1 + C, B1);
                                          else
                                             Meat (A1, B1) := Meat (A1, B1)
                                               + Cov * Wk * Wm * (J (2 * K - 1 + C, A1) * J (2 * M - 1 + C, B1)
                                                                  + J (2 * M - 1 + C, A1) * J (2 * K - 1 + C, B1));
                                          end if;
                                       end loop;
                                    end loop;
                                 end;
                              end loop;
                           end;
                        end if;
                     end loop;
                  end if;
               end loop;
            end;
         end Add_Residual_Spread;

         procedure Add_Common_Spread is
            Moves : Matrix_Access := new Real_Matrix (1 .. P, 1 .. 3 * Points'Length);
            Shared_Part : Matrix_Access := new Real_Matrix (1 .. P, 1 .. Shared);
         begin
            Moves.all := [others => [others => 0.0]];
            Shared_Part.all := [others => [others => 0.0]];
            --  How the gradient moves with each point's position: the sum, over the point's sightings and both
            --  directions, of the weighted row of the Jacobian times how the residual moves with the position.
            for K in 1 .. N loop
               if Fits (K) then
                  declare
                     Sg  : Sighting renames Sightings (Sightings'First + K - 1);
                     Pt  : constant Positive := Sg.Point - Points'First + 1;
                     Rot : constant Mat3 := Poses (Sg.Pose).Rotation;
                  begin
                     for Axis in 1 .. 3 loop
                        declare
                           Step : constant Real :=
                             Difference_Step * Real'Max (1.0, abs Points (Sg.Point).Position (Axis));
                           Dx   : Vec3 := Zero3;
                           Ua, Va, Ub, Vb   : Real;
                           Ahead_A, Ahead_B : Boolean;
                        begin
                           Dx (Axis) := Step;
                           Fit.Project (S.L, Transpose (S.R) * (World (K) + Rot * Dx - S.C), Ua, Va, Ahead_A);
                           Fit.Project (S.L, Transpose (S.R) * (World (K) - Rot * Dx - S.C), Ub, Vb, Ahead_B);
                           if Ahead_A and then Ahead_B then
                              for C in 0 .. 1 loop
                                 declare
                                    W     : constant Real := Huber (R (2 * K - 1 + C) / Sigma);
                                    Slope : constant Real :=
                                      (if C = 0 then Ua - Ub else Va - Vb)
                                      / (2.0 * Step * Sigma * Scale_Of (2 * K - 1 + C));
                                 begin
                                    for A1 in 1 .. P loop
                                       Moves (A1, 3 * (Pt - 1) + Axis) :=
                                         Moves (A1, 3 * (Pt - 1) + Axis) + W * J (2 * K - 1 + C, A1) * Slope;
                                    end loop;
                                 end;
                              end loop;
                           end if;
                        end;
                     end loop;
                  end;
               end if;
            end loop;
            for Pt in 1 .. Points'Length loop
               for A1 in 1 .. P loop
                  for T in 1 .. Shared loop
                     for X in 1 .. 3 loop
                        Shared_Part (A1, T) := Shared_Part (A1, T)
                          + Moves (A1, 3 * (Pt - 1) + X)
                            * Common (Common'First (1) + 3 * (Pt - 1) + X - 1, Common'First (2) + T - 1);
                     end loop;
                  end loop;
               end loop;
            end loop;
            for A1 in 1 .. P loop
               for B1 in 1 .. P loop
                  for T in 1 .. Shared loop
                     for U in 1 .. Shared loop
                        Meat (A1, B1) := Meat (A1, B1)
                          + Shared_Part (A1, T) * Shared_Part (B1, U)
                            * Common_Covariance
                                (Common_Covariance'First (1) + T - 1, Common_Covariance'First (2) + U - 1);
                     end loop;
                  end loop;
               end loop;
            end loop;
            Free (Moves);
            Free (Shared_Part);
         end Add_Common_Spread;
      begin
         Covariance := [others => [others => 0.0]];
         Linearize (S, Which, R.all, J.all);
         Normal_Equations (R.all, J.all, A, G, Cost);
         Driver.Numerics.Dense.Cholesky (A, L, Pd);
         Ok := Pd;
         if Pd then
            declare
               Inverse_A  : Real_Matrix (1 .. P, 1 .. P);
               Spread     : Real_Matrix (1 .. P, 1 .. P);
               Clipped    : Real;
               Sandwiched : Boolean;
            begin
               for Q in 1 .. P loop
                  declare
                     E : Real_Vector (1 .. P) := [others => 0.0];
                  begin
                     E (Q) := 1.0;
                     declare
                        X : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (L, E);
                     begin
                        for Q2 in 1 .. P loop
                           Inverse_A (Q2, Q) := X (Q2);
                        end loop;
                     end;
                  end;
               end loop;
               Add_Residual_Spread;
               if Shared > 0 then
                  Add_Common_Spread;
               end if;
               Fit.Sandwich (Inverse_A, Meat, Spread, Clipped, Sandwiched);
               Ok := Sandwiched;
               for A1 in 1 .. P loop
                  for B1 in 1 .. P loop
                     Covariance (Which (Which'First + A1 - 1), Which (Which'First + B1 - 1)) := Spread (A1, B1);
                  end loop;
               end loop;
            end;
         end if;
         Free (R);
         Free (J);
      end Covariance_At;

      --  A stage's outcome.
      Stage_A, Stage_B : State;
      Fits_A, Fits_B   : Fit.Flag_Array (1 .. N);
      Sigma_A, Sigma_B : Real;
      Cov_A, Cov_B     : Real_Matrix (1 .. Terms, 1 .. Terms);
      Ok_A, Ok_B       : Boolean;
      Keep_B           : Boolean := False;

      Used    : Natural := 0;
      Visible : Natural := 0;
   begin
      Report := (others => <>);
      Report.Offered := N;
      Report.L := Start_Lens;
      Report.Pose := Start_Pose;
      for K in 1 .. N loop
         declare
            Sg : Sighting renames Sightings (Sightings'First + K - 1);
         begin
            World (K) := Poses (Sg.Pose) * Points (Sg.Point).Position;
         end;
      end loop;
      declare
         U, V  : Real;
         Ahead : Boolean;

         --  A point whose position or whose uncertainty is not a number, or is beyond what the float holds,
         --  tells nothing of the eye.
         function Finite (P : Point) return Boolean is
           ((for all A in 1 .. 3 => abs P.Position (A) < Real'Last)
            and then (for all A in 1 .. 3 => (for all B in 1 .. 3 => abs P.Own (A, B) < Real'Last)));
      begin
         for K in 1 .. N loop
            Predict (Eye, K, U, V, Ahead);
            Ahead := Ahead and then Finite (Points (Sightings (Sightings'First + K - 1).Point));
            Seen (K) := Ahead;
            Fits (K) := Ahead;
         end loop;
      end;
      --  Fewer sightings than a fit has terms leave nothing to tell the terms from the noise.
      declare
         Count : Natural := 0;
      begin
         for K in 1 .. N loop
            if Seen (K) then
               Count := Count + 1;
            end if;
         end loop;
         Visible := Count;
      end;
      if 2 * Visible <= Terms then
         Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
           ("the eye sees too few of the points to measure its lens and pose from");
         Free (World);
         Free (Seen);
         Free (Extra);
         Free (Fits);
         return;
      end if;
      Settle (Eye, Without_Distortion);
      Stage_A := Eye;
      Fits_A := Fits.all;
      Sigma_A := Sigma;
      Covariance_At (Stage_A, Without_Distortion, Cov_A, Ok_A);
      --  The distortion terms, kept only when as a pair they are significant.
      Settle (Eye, With_Distortion);
      Stage_B := Eye;
      Fits_B := Fits.all;
      Sigma_B := Sigma;
      Covariance_At (Stage_B, With_Distortion, Cov_B, Ok_B);
      if Ok_B then
         declare
            C11 : constant Real := Cov_B (5, 5);
            C12 : constant Real := Cov_B (5, 6);
            C22 : constant Real := Cov_B (6, 6);
            Det : constant Real := C11 * C22 - C12 * C12;
         begin
            if Det > 0.0 then
               declare
                  K1 : constant Real := Stage_B.L.K1;
                  K2 : constant Real := Stage_B.L.K2;
                  Chi : constant Real := (C22 * K1 * K1 - 2.0 * C12 * K1 * K2 + C11 * K2 * K2) / Det;
               begin
                  Keep_B := Chi > 0.0 and then Driver.Distributions.Chi_Square_Deviate (Chi, 2) > Driver.Conventions.Z;
               end;
            end if;
         end;
      end if;
      declare
         Final    : constant State := (if Keep_B then Stage_B else Stage_A);
         Cov      : constant Real_Matrix := (if Keep_B then Cov_B else Cov_A);
         Final_Ok : constant Boolean := (if Keep_B then Ok_B else Ok_A);
      begin
         Fits.all := (if Keep_B then Fits_B else Fits_A);
         Sigma := (if Keep_B then Sigma_B else Sigma_A);
         for K in 1 .. N loop
            if Fits (K) then
               Used := Used + 1;
            end if;
         end loop;
         Report.L := Final.L;
         Report.Pose := (Rotation => Final.R, Translation => Final.C);
         Report.Sigma_Px := Sigma;
         Report.Used := Used;
         Report.Distorted := Keep_B;
         --  The covariance as the fit has it: the turn is about the eye's own axes (the pose is its estimate
         --  turned by it), the centre is in the world.
         if Final_Ok then
            for A1 in 1 .. Terms loop
               for B1 in 1 .. Terms loop
                  Report.Covariance.Append (Cov (A1, B1));
               end loop;
            end loop;
         end if;
         if not Final_Ok then
            Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("the points' views do not determine the eye's lens and pose (the normal equations are singular)");
         elsif Used * 2 <= Terms then
            Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("too few of the points fit one eye to measure its" & Terms'Image & " terms from");
         else
            declare
               Sigma_Fx : constant Real := Final.L.Fx * Sqrt (Cov (1, 1));
               Sigma_Fy : constant Real := Final.L.Fy * Sqrt (Cov (2, 2));
               Turn_Var : constant Real := Cov (7, 7) + Cov (8, 8) + Cov (9, 9);
               Place_Var : constant Real := Cov (10, 10) + Cov (11, 11) + Cov (12, 12);
            begin
               if not Driver.Uncertain.Significant (Final.L.Fx, Sigma_Fx, 2 * Used - Terms)
                 or else not Driver.Uncertain.Significant (Final.L.Fy, Sigma_Fy, 2 * Used - Terms)
               then
                  Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                    ("the points leave the focal lengths undetermined:" & Driver.Log.Image (Final.L.Fx, 1) & " +-"
                     & Driver.Log.Image (Sigma_Fx, 1) & " and" & Driver.Log.Image (Final.L.Fy, 1) & " +-"
                     & Driver.Log.Image (Sigma_Fy, 1) & " px");
               elsif not (Driver.Conventions.Z * Sqrt (Turn_Var) < Ada.Numerics.Pi / 2.0) then
                  Report.Why := Ada.Strings.Unbounded.To_Unbounded_String
                    ("the points leave the eye's turn known only to" & Real'Image (Sqrt (Turn_Var)) & " rad");
               elsif not (Place_Var < Real'Last) then
                  Report.Why := Ada.Strings.Unbounded.To_Unbounded_String ("the points leave the eye's centre unknown");
               else
                  Report.Determined := True;
               end if;
            end;
         end if;
      end;
      Free (World);
      Free (Seen);
      Free (Extra);
      Free (Fits);
   end Fit_Eye;

   ---------------------------------------------------------------------------
   --  The start from a plane and the parallax off it


   --  The camera that the homography H of a plane and the parallax of the points off it give. A point at (X, Y)
   --  along the plane's axes and a height Z above it is seen at the pixel of H (X, Y, 1) + Z e, where e is a
   --  column of the camera's matrix, in the scale H has: found from the points off the plane, by robust least
   --  squares on their pixels (the points on it say nothing of e). The camera's matrix [h1 h2 e h3] is then
   --  K [R | t] up to a scale, with K upper triangular: the decomposition of its first three columns from the last
   --  row up gives the lens (the focal lengths and the principal point, the skew left out) and the rotation, the
   --  last column the translation. Not Found when the points leave e undetermined (all of them on the plane).
   procedure Parallax
     (H         : Mat3;
      Sightings : Sighting_Array;
      Along_1, Along_2, Height_Of : Real_Array;
      Off_Plane : Fit.Flag_Array;   --  the sighting's point is off the plane (the others' height is nothing)
      Frame     : Mat3;   --  the plane's axes as columns: E1, E2, Normal
      Origin    : Vec3;
      Lens      : out Fit.Lens;
      Pose      : out Rigid;
      Found     : out Boolean)
   is
      N      : constant Natural := Sightings'Length;
      E      : Vec3 := Zero3;
      Weight : Real_Access := new Real_Array (1 .. N);
      Rounds : Natural := 0;
      Ok     : Boolean := True;
      Again  : Boolean := True;

      --  Where the camera of the current E puts sighting J, and its depth there.
      procedure Predict (J : Positive; U, V, Depth : out Real) is
         Hx : constant Vec3 := H * [Along_1 (J), Along_2 (J), 1.0];
         Q  : constant Vec3 := Hx + Height_Of (J) * E;
      begin
         Depth := Q (3);
         U := (if Q (3) /= 0.0 then Q (1) / Q (3) else Real'Last);
         V := (if Q (3) /= 0.0 then Q (2) / Q (3) else Real'Last);
      end Predict;
   begin
      Lens := (others => <>);
      Pose := Identity;
      Found := False;
      Weight.all := [others => 1.0];
      while Again and then Ok and then Rounds <= N loop
         Rounds := Rounds + 1;
         declare
            A  : Mat3 := [others => [others => 0.0]];
            B  : Vec3 := Zero3;
            L  : Mat3;
            Pd : Boolean;
         begin
            for J in 1 .. N loop
               declare
                  Sg : Sighting renames Sightings (Sightings'First + J - 1);
                  Z  : constant Real := Height_Of (J);
                  Foot : constant Vec3 := [Along_1 (J), Along_2 (J), 1.0];
                  Hx : constant Vec3 := H * Foot;
                  Depth : constant Real := Hx (3) + Z * E (3);
               begin
                  if Off_Plane (J) and then Depth /= 0.0 then
                     declare
                        W  : constant Real := Weight (J) / Depth ** 2;
                        Cu : constant Vec3 := [-Z, 0.0, Z * Sg.U];
                        Cv : constant Vec3 := [0.0, -Z, Z * Sg.V];
                        Ru : constant Real := Hx (1) - Sg.U * Hx (3);
                        Rv : constant Real := Hx (2) - Sg.V * Hx (3);
                     begin
                        A := A + W * Outer (Cu, Cu) + W * Outer (Cv, Cv);
                        B := B + (W * Ru) * Cu + (W * Rv) * Cv;
                     end;
                  end if;
               end;
            end loop;
            Driver.Numerics.Dense.Cholesky (A, L, Pd);
            if not Pd then
               Ok := False;
            else
               declare
                  Next  : constant Vec3 := Driver.Numerics.Dense.Cholesky_Solve (L, B);
                  Both  : Real_Access := new Real_Array (1 .. 2 * N);
                  Sigma : Real;
               begin
                  Again := abs (Next - E) > Driver.Conventions.Unchanged_Fraction * abs Next;
                  E := Next;
                  for J in 1 .. N loop
                     declare
                        U, V, Depth : Real;
                     begin
                        Predict (J, U, V, Depth);
                        Both (2 * J - 1) := Sightings (Sightings'First + J - 1).U - U;
                        Both (2 * J) := Sightings (Sightings'First + J - 1).V - V;
                     end;
                  end loop;
                  Sigma := Driver.Stats.Robust_Sigma (Both.all);
                  for J in 1 .. N loop
                     Weight (J) := (if Sigma > 0.0
                                    then Huber (Real'Max (abs Both (2 * J - 1), abs Both (2 * J)) / Sigma) else 1.0);
                  end loop;
                  Free (Both);
               end;
            end if;
         end;
      end loop;
      Free (Weight);
      if not Ok then
         return;
      end if;
      declare
         --  K R = the first three columns, K upper triangular with a positive diagonal, found from the last row up.
         Row_1 : constant Vec3 := [H (1, 1), H (1, 2), E (1)];
         Row_2 : constant Vec3 := [H (2, 1), H (2, 2), E (2)];
         Row_3 : constant Vec3 := [H (3, 1), H (3, 2), E (3)];
         K33   : constant Real := abs Row_3;
      begin
         if K33 = 0.0 then
            return;
         end if;
         declare
            R3  : constant Vec3 := Row_3 / K33;
            K23 : constant Real := Row_2 * R3;
            V2  : constant Vec3 := Row_2 - K23 * R3;
            K22 : constant Real := abs V2;
         begin
            if K22 = 0.0 then
               return;
            end if;
            declare
               R2  : constant Vec3 := V2 / K22;
               K13 : constant Real := Row_1 * R3;
               K12 : constant Real := Row_1 * R2;
               V1  : constant Vec3 := Row_1 - K13 * R3 - K12 * R2;
               K11 : constant Real := abs V1;
            begin
               if K11 = 0.0 then
                  return;
               end if;
               declare
                  R1   : constant Vec3 := V1 / K11;
                  --  The matrix is known up to a sign; the one that makes R a rotation.
                  Sign : constant Real := (if R1 * Cross (R2, R3) < 0.0 then -1.0 else 1.0);
                  Rm   : constant Mat3 :=
                    [[Sign * R1 (1), Sign * R1 (2), Sign * R1 (3)],
                     [Sign * R2 (1), Sign * R2 (2), Sign * R2 (3)],
                     [Sign * R3 (1), Sign * R3 (2), Sign * R3 (3)]];
                  Last : constant Vec3 := [H (1, 3), H (2, 3), H (3, 3)];
                  P4   : constant Vec3 := Sign * Last;
                  T3   : constant Real := P4 (3) / K33;
                  T2   : constant Real := (P4 (2) - K23 * T3) / K22;
                  T1   : constant Real := (P4 (1) - K12 * T2 - K13 * T3) / K11;
                  T    : constant Vec3 := [T1, T2, T3];
                  --  X_camera = Rm (Frame^T (X - Origin)) + T.
                  To_Camera : constant Mat3 := Rm * Transpose (Frame);
               begin
                  Lens := (Fx => K11 / K33, Fy => K22 / K33, Cx => K13 / K33, Cy => K23 / K33, K1 => 0.0, K2 => 0.0);
                  Pose := (Rotation => Transpose (To_Camera), Translation => Origin - Transpose (To_Camera) * T);
                  Found := True;
               end;
            end;
         end;
      end;
   end Parallax;

   procedure Start
     (Points     : Point_Array;
      Poses      : Pose_Array;
      Sightings  : Sighting_Array;
      On_Plane   : Fit.Flag_Array;
      Plane      : Vec3;
      Lens       : out Fit.Lens;
      Pose       : out Rigid;
      Found      : out Boolean)
   is
      Seen : constant Fit.Sight_Plane := (Found => True, A => Plane, others => <>);
      Normal, E1, E2, Unused : Vec3;
      Frame   : Mat3;
      Origin  : Vec3;
      Count   : Natural := 0;
      N       : constant Natural := Sightings'Length;
   begin
      Lens := (others => <>);
      Pose := Identity;
      Found := False;
      if abs Plane = 0.0 then
         return;
      end if;
      Fit.Plane_Axes (Seen, Normal, E1, Unused);
      --  The plane's frame (E1, E2, Normal) is right-handed, whatever order the plane's axes come in.
      E2 := Cross (Normal, E1);
      Origin := Fit.Plane_Offset (Seen) * Normal;
      Frame := [[E1 (1), E2 (1), Normal (1)], [E1 (2), E2 (2), Normal (2)], [E1 (3), E2 (3), Normal (3)]];
      for K in Sightings'Range loop
         if On_Plane (K) then
            Count := Count + 1;
         end if;
      end loop;
      if Count <= 4 then
         return;
      end if;
      declare
         Pairs : Fit.Plane_Point_Array (1 .. Count);
         Fits  : Fit.Flag_Array (1 .. Count);
         H     : Mat3;
         Sigma : Real;
         Good  : Boolean;
         I     : Natural := 0;
         --  Every sighting's point in the plane's frame: along E1 and E2, and its height above the plane.
         Along_1 : Real_Access := new Real_Array (1 .. N);
         Along_2 : Real_Access := new Real_Array (1 .. N);
         Height_Of : Real_Access := new Real_Array (1 .. N);
      begin
         for K in Sightings'Range loop
            declare
               Sg : Sighting renames Sightings (K);
               X  : constant Vec3 := Poses (Sg.Pose) * Points (Sg.Point).Position - Origin;
               J  : constant Positive := K - Sightings'First + 1;
            begin
               Along_1 (J) := X * E1;
               Along_2 (J) := X * E2;
               Height_Of (J) := X * Normal;
               if On_Plane (K) then
                  I := I + 1;
                  Pairs (I) := (X => Along_1 (J), Y => Along_2 (J), U => Sg.U, V => Sg.V);
               end if;
            end;
         end loop;
         Fit.Plane_Homography (Pairs, H, Fits, Sigma, Good);
         if Good then
            declare
               Off : Fit.Flag_Array (1 .. N);
            begin
               for K in Sightings'Range loop
                  Off (K - Sightings'First + 1) := not On_Plane (K);
               end loop;
               Parallax (H, Sightings, Along_1.all, Along_2.all, Height_Of.all, Off, Frame, Origin, Lens, Pose, Found);
            end;
         end if;
         Free (Along_1);
         Free (Along_2);
         Free (Height_Of);
      end;
   end Start;

end Driver.Robot.Kinematics.Fixed;
