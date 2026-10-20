with Ada.Environment_Variables;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Clock;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Flow_Eye is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;

   type Flag_Array is array (Positive range <>) of Boolean;
   type Flag_Access is access Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   ---------------------------------------------------------------------------
   --  What a pose shows

   --  The picture is smoothed before its gradient and its changes are taken, both by the one Gaussian: a camera's pixel
   --  integrates the light over its area, an edge is a ramp a pixel wide, and its central difference is half the slope
   --  the pixel's change sees where the edge crosses it and all of it where the edge has not come (the fake's renders,
   --  10-20: at the true eye the rates the pictures showed were 1.14 to 1.26 times the model's, residuals a median 5.8
   --  and a tenth over 32 times their sigmas). Smoothed, the change at a pixel is the gradient there times the motion
   --  to the extent the smoothing leaves nothing at half the sampling rate: the least Gaussian whose response there is
   --  one of the picture's levels (exp (-pi^2 sigma^2 / 2) = 1 / 255), its weights kept to the last that is one level
   --  of its peak; the gradient is the Gaussian's derivative, scaled so that a ramp's is its slope.
   Levels : constant Real := 255.0;
   Smooth_Sigma  : constant Real := Sqrt (2.0 * Ada.Numerics.Long_Elementary_Functions.Log (Levels)) / Ada.Numerics.Pi;
   Smooth_Radius : constant Natural :=
     Natural (Real'Floor (Smooth_Sigma * Sqrt (2.0 * Ada.Numerics.Long_Elementary_Functions.Log (Levels))));

   type Kernel is array (Integer range -Smooth_Radius .. Smooth_Radius) of Real;

   function Weights return Kernel is
      Result : Kernel;
      Total  : Real := 0.0;
   begin
      for X in Kernel'Range loop
         Result (X) := Exp (-Real (X * X) / (2.0 * Smooth_Sigma * Smooth_Sigma));
         Total := Total + Result (X);
      end loop;
      for X in Kernel'Range loop
         Result (X) := Result (X) / Total;
      end loop;
      return Result;
   end Weights;

   Smoothing : constant Kernel := Weights;

   function Slopes return Kernel is
      Result : Kernel;
      Moment : Real := 0.0;
   begin
      for X in Kernel'Range loop
         Moment := Moment + Real (X * X) * Smoothing (X);
      end loop;
      for X in Kernel'Range loop
         Result (X) := Real (X) * Smoothing (X) / Moment;
      end loop;
      return Result;
   end Slopes;

   Slope : constant Kernel := Slopes;

   function Square_Sum (K : Kernel) return Real is
      Result : Real := 0.0;
   begin
      for X in K'Range loop
         Result := Result + K (X) * K (X);
      end loop;
      return Result;
   end Square_Sum;

   --  Picture convolved with Along across its rows and Across down its columns (correlation: the weight at X falls on
   --  the pixel X further on), into Into; the pixels within the radius of the border are left zero.
   procedure Filter
     (Picture : Real_Array; Width, Height : Positive; Along, Across : Kernel; Into : out Real_Array)
   is
      Rows_Done : Real_Access := new Real_Array (1 .. Width * Height);
   begin
      for K in Into'Range loop
         Into (K) := 0.0;
      end loop;
      for K in Rows_Done'Range loop
         Rows_Done (K) := 0.0;
      end loop;
      for R in 0 .. Height - 1 loop
         for C in Smooth_Radius .. Width - 1 - Smooth_Radius loop
            declare
               Sum : Real := 0.0;
            begin
               for X in Kernel'Range loop
                  Sum := Sum + Along (X) * Picture (Picture'First + R * Width + C + X);
               end loop;
               Rows_Done (1 + R * Width + C) := Sum;
            end;
         end loop;
      end loop;
      for R in Smooth_Radius .. Height - 1 - Smooth_Radius loop
         for C in Smooth_Radius .. Width - 1 - Smooth_Radius loop
            declare
               Sum : Real := 0.0;
            begin
               for X in Kernel'Range loop
                  Sum := Sum + Across (X) * Rows_Done (1 + (R + X) * Width + C);
               end loop;
               Into (Into'First + R * Width + C) := Sum;
            end;
         end loop;
      end loop;
      Free (Rows_Done);
   end Filter;

   function Evidence
     (Readings      : Real_Array;
      Still         : Real_Array;
      Background    : Real_Array;
      Noise         : Real;
      Minus, Plus   : Luma_Array;
      Steps         : Real_Array;
      Width, Height : Positive) return Pose_Access
   is
      N      : constant Natural := Readings'Length;
      Pixels : constant Positive := Width * Height;
      --  Every pixel one test of a family of as many as the picture has.
      Gate   : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Tests => Pixels));
      --  A pixel of Still is a mean whose own noise is Noise over the root of two (Noise is that of a difference of two
      --  such means); the filters' outputs carry it through their weights.
      Sigma_G : constant Real := Noise / Sqrt (2.0) * Sqrt (Square_Sum (Slope) * Square_Sum (Smoothing));
      Sigma_C : constant Real := Noise * Square_Sum (Smoothing);
      Keep    : Flag_Access := new Flag_Array (1 .. Pixels);
      Count   : Natural := 0;
      Gu_All  : Real_Access := new Real_Array (1 .. Pixels);
      Gv_All  : Real_Access := new Real_Array (1 .. Pixels);
      Changes : Real_Access := new Real_Array (1 .. Natural'Max (1, N) * Pixels);
      Diff    : Real_Access := new Real_Array (1 .. Pixels);
      Back_U, Back_V : Real_Access := new Real_Array (1 .. Pixels);
      function Change (J : Positive; K : Natural) return Real is (Changes ((J - 1) * Pixels + K + 1));
   begin
      Filter (Still, Width, Height, Slope, Smoothing, Gu_All.all);
      Filter (Still, Width, Height, Smoothing, Slope, Gv_All.all);
      for J in 1 .. N loop
         declare
            P : Real_Array renames Plus (Plus'First + J - 1).all;
            M : Real_Array renames Minus (Minus'First + J - 1).all;
         begin
            for K in 0 .. Pixels - 1 loop
               Diff (K + 1) := P (P'First + K) - M (M'First + K);
            end loop;
            Filter (Diff.all, Width, Height, Smoothing, Smoothing, Changes ((J - 1) * Pixels + 1 .. J * Pixels));
         end;
      end loop;
      Free (Diff);
      --  What the arm moves against: the picture with the arm elsewhere (Background, the poses' stills' per-pixel
      --  median), and where this pose's still shows it (agrees with it within the gate of the noise). Its own gradient
      --  moves with nothing, and where the smoothing takes it in with an edge of the arm the change there is the edge's
      --  alone (the fake's textured table, 10-20: at the true eye its gradient left the outline's pixels 25 to 45 % off;
      --  taken from the still only where no joint's turn changed the pixel, the band an edge sweeps kept it, and the
      --  principal point came out 13 to 16 px off on three seeds). The gradient of what moves is the smoothed gradient
      --  less the smoothed gradient of the background where it shows.
      declare
         Su, Sv : Real_Access := new Real_Array (1 .. Pixels);
         function Back_At (K : Natural) return Real is (Background (Background'First + K));
      begin
         for K in 1 .. Pixels loop
            Su (K) := 0.0;
            Sv (K) := 0.0;
         end loop;
         for R in 1 .. Height - 2 loop
            for C in 1 .. Width - 2 loop
               declare
                  K : constant Natural := R * Width + C;
               begin
                  if abs (Still (Still'First + K) - Back_At (K)) <= Gate * Noise then
                     Su (K + 1) := (Back_At (K + 1) - Back_At (K - 1)) / 2.0;
                     Sv (K + 1) := (Back_At (K + Width) - Back_At (K - Width)) / 2.0;
                  end if;
               end;
            end loop;
         end loop;
         Filter (Su.all, Width, Height, Smoothing, Smoothing, Back_U.all);
         Filter (Sv.all, Width, Height, Smoothing, Smoothing, Back_V.all);
         for K in 1 .. Pixels loop
            Gu_All (K) := Gu_All (K) - Back_U (K);
            Gv_All (K) := Gv_All (K) - Back_V (K);
         end loop;
         Free (Su);
         Free (Sv);
      end;
      for K in Keep'Range loop
         Keep (K) := False;
      end loop;
      for R in Smooth_Radius .. Height - 1 - Smooth_Radius loop
         for C in Smooth_Radius .. Width - 1 - Smooth_Radius loop
            declare
               K  : constant Natural := R * Width + C;
            begin
               if Sqrt (Gu_All (K + 1) ** 2 + Gv_All (K + 1) ** 2) > Gate * Sigma_G
                 and then (for some J in 1 .. N => abs Change (J, K) > Gate * Sigma_C)
               then
                  Keep (K + 1) := True;
                  Count := Count + 1;
               end if;
            end;
         end loop;
      end loop;
      --  One pixel of every cell a smoothing radius wide, the one whose changes stand out the most: the smoothed noise of
      --  pixels that far apart correlates by exp (-radius^2 / (4 sigma^2)), an eighth, and the pixels of one edge closer
      --  than that tell the fit the same thing again while each takes its time (10-20: some 20 000 pixels for two arms
      --  at six poses each, a fit minutes long).
      declare
         Cell   : constant Positive := Positive'Max (1, Smooth_Radius);
         Across : constant Positive := (Width + Cell - 1) / Cell;
         Down   : constant Positive := (Height + Cell - 1) / Cell;
         Best   : array (0 .. Across - 1, 0 .. Down - 1) of Natural := [others => [others => 0]];
         Most   : array (0 .. Across - 1, 0 .. Down - 1) of Real := [others => [others => -1.0]];
      begin
         for R in Smooth_Radius .. Height - 1 - Smooth_Radius loop
            for C in Smooth_Radius .. Width - 1 - Smooth_Radius loop
               declare
                  K    : constant Natural := R * Width + C;
                  Told : Real := 0.0;
               begin
                  if Keep (K + 1) then
                     for J in 1 .. N loop
                        Told := Told + (Change (J, K) / Sigma_C) ** 2;
                     end loop;
                     if Told > Most (C / Cell, R / Cell) then
                        Most (C / Cell, R / Cell) := Told;
                        Best (C / Cell, R / Cell) := K + 1;
                     end if;
                     Keep (K + 1) := False;
                  end if;
               end;
            end loop;
         end loop;
         Count := 0;
         for Q of Best loop
            if Q > 0 then
               Keep (Q) := True;
               Count := Count + 1;
            end if;
         end loop;
      end;
      declare
         Result : constant Pose_Access := new Pose_View (N, Count);
         I      : Natural := 0;
      begin
         Result.Readings := Readings;
         for R in Smooth_Radius .. Height - 1 - Smooth_Radius loop
            for C in Smooth_Radius .. Width - 1 - Smooth_Radius loop
               declare
                  K : constant Natural := R * Width + C;
               begin
                  if Keep (K + 1) then
                     I := I + 1;
                     Result.U (I) := Real (C) + 0.5;
                     Result.V (I) := Real (R) + 0.5;
                     Result.Gu (I) := Gu_All (K + 1);
                     Result.Gv (I) := Gv_All (K + 1);
                     Result.Back (I) := Sqrt (Back_U (K + 1) ** 2 + Back_V (K + 1) ** 2);
                     Result.Link (I) := 0;
                     for J in 1 .. N loop
                        declare
                           Step : constant Real := Steps (Steps'First + J - 1);
                           D    : constant Real := Change (J, K);
                        begin
                           Result.Rate (I, J) := (if Step /= 0.0 then D / (2.0 * Step) else 0.0);
                           Result.Sigma (I, J) := (if Step /= 0.0 then Sigma_C / (2.0 * abs Step) else Real'Last);
                           if Step /= 0.0 and then abs D > Gate * Sigma_C then
                              Result.Link (I) := J;
                           end if;
                        end;
                     end loop;
                  end if;
               end;
            end loop;
         end loop;
         Free (Keep);
         Free (Gu_All);
         Free (Gv_All);
         Free (Back_U);
         Free (Back_V);
         Free (Changes);
         return Result;
      end;
   end Evidence;

   ---------------------------------------------------------------------------
   --  The model

   --  A joint's turn at a pose, in the arm's reference frame: its axis, a point on it, its radians (or units of length)
   --  a reading unit, and whether it slides.
   type Twist is record
      W, P  : Vec3 := [0.0, 0.0, 0.0];
      C     : Real := 0.0;
      Slide : Boolean := False;
   end record;

   type Twist_Array is array (Positive range <>) of Twist;
   type Twist_Grid is array (Positive range <>, Positive range <>) of Twist;
   type Twist_Grid_Access is access Twist_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Twist_Grid, Twist_Grid_Access);

   --  Every joint's turn at the readings: each joint's axis and point carried by the joints before it, as the arm's fit
   --  composes them (Fit.Eye_At: each joint's step about its axis at the reference, the first of the chain first).
   function Twists_At (Joints : Fit.Joint_Array; Readings : Real_Array) return Twist_Array is
      Result : Twist_Array (1 .. Joints'Length);
      Before : Rigid := Identity;
   begin
      for J in 1 .. Joints'Length loop
         declare
            A : Fit.Joint renames Joints (Joints'First + J - 1);
         begin
            Result (J) := (W => Times (Before.Rotation, A.W), P => Before * A.P, C => A.C, Slide => A.Slide);
            Before := Before * Fit.Eye_At (Joints (Joints'First + J - 1 .. Joints'First + J - 1), [1],
                                           [Readings (Readings'First + J - 1)]);
         end;
      end loop;
      return Result;
   end Twists_At;

   --  The terms: the logarithm of the focal length, a turn about the camera's own axes on top of the base, the centre,
   --  and the principal point (pixels): a camera's optical axis meets its picture a few pixels off the middle (the
   --  fake's eyes up to three, a webcam's more), and held at the middle the offset turned the eye by its angle and
   --  moved its centre by that angle times its distance (seed 22, 10-20: 1.6 degrees and 17 mm).
   Terms : constant := 9;
   type Parameters is array (1 .. Terms) of Real;

   type Problem is record
      Poses  : access constant Pose_Array;
      Twists : Twist_Grid_Access;     --  per pose, per joint
      Width, Height : Positive := 1;
      Joints : Natural := 0;
      Cx, Cy : Real := 0.0;           --  the lens's principal point, and its focal length down over across
      Aspect : Real := 1.0;
      --  A rate's spread: Additive times the sigma its pictures state, and Relative times the rate (what the pictures
      --  share beyond their noise: an edge's place in the pixels, the turn's reach past the linear), in quadrature.
      Additive : Real := 1.0;
      Relative : Real := 0.0;
   end record;

   --  A pixel's line of sight in the camera's frame (its third coordinate one) at the focal length F, and its gradient
   --  carried to that frame: the change of the picture there per unit of a point's motion over its depth.
   type Sighting is record
      Xn, Yn : Real;
      Gn     : Vec3;
   end record;

   function Sight (P : Problem; F, Cx, Cy : Real; View : Pose_View; I : Positive) return Sighting is
      Fy : constant Real := F * P.Aspect;
      Xn : constant Real := (View.U (I) - Cx) / F;
      Yn : constant Real := (View.V (I) - Cy) / Fy;
   begin
      return (Xn => Xn, Yn => Yn,
              Gn => [F * View.Gu (I), Fy * View.Gv (I), -(F * View.Gu (I) * Xn + Fy * View.Gv (I) * Yn)]);
   end Sight;

   function Pose_Of (Base : Mat3; X : Parameters) return Rigid is
     ((Rotation => Times (Base, Driver.Numerics.Exp ([X (2), X (3), X (4)])), Translation => [X (5), X (6), X (7)]));

   --  One pixel's residuals at the terms, in units of their sigmas, the joints that move it in order (Count of them),
   --  its depth's inverse eliminated (Inverse_Depth the one the rates give, nought or less behind the eye); Ok False
   --  when the terms move no point on its line of sight.
   procedure Pixel_Residuals
     (P : Problem; V : Positive; I : Positive; Base : Mat3; X : Parameters; R : out Real_Array; Count : out Natural;
      Inverse_Depth : out Real; Ok : out Boolean)
   is
      View : Pose_View renames P.Poses (V).all;
      F    : constant Real := Exp (X (1));
      Pose : constant Rigid := Pose_Of (Base, X);
      Here : constant Sighting := Sight (P, F, X (8), X (9), View, I);
      Ray  : constant Vec3 := Times (Pose.Rotation, Vec3'[Here.Xn, Here.Yn, 1.0]);
      K    : constant Natural := View.Link (I);
      A, B, E, S : Real_Array (1 .. Natural'Max (1, K)) := [others => 0.0];
      Saa, Sab : Real := 0.0;
   begin
      Count := K;
      Ok := False;
      Inverse_Depth := 0.0;
      for J in 1 .. K loop
         declare
            T  : Twist renames P.Twists (V, J);
            Av : constant Vec3 :=
              (if T.Slide then Scaled (T.C, T.W) else Scaled (T.C, Cross (T.W, Minus (Pose.Translation, T.P))));
            Bv : constant Vec3 := (if T.Slide then [0.0, 0.0, 0.0] else Scaled (T.C, Cross (T.W, Ray)));
            Ac : constant Vec3 := Transposed_Times (Pose.Rotation, Av);
            Bc : constant Vec3 := Transposed_Times (Pose.Rotation, Bv);
         begin
            A (J) := Dot (Here.Gn, Ac);
            B (J) := Dot (Here.Gn, Bc);
            S (J) := Sqrt ((P.Additive * View.Sigma (I, J)) ** 2 + (P.Relative * View.Rate (I, J)) ** 2);
            E (J) := View.Rate (I, J) + B (J);
            Saa := Saa + A (J) ** 2 / S (J) ** 2;
            Sab := Sab + A (J) * E (J) / S (J) ** 2;
         end;
      end loop;
      if Saa <= 0.0 then
         return;
      end if;
      --  The depth's inverse the rates give, and the residuals at it taken no less than nought (a point at infinity): a
      --  pixel the terms put behind the eye costs what it misfits by from there, and the cost stays continuous where
      --  its depth passes infinity (a step that turned one of many thousand pixels so was refused, and the arms' fit
      --  together, 10-20, took damped steps of nothing).
      Inverse_Depth := -Sab / Saa;
      for J in 1 .. K loop
         R (R'First + J - 1) := (E (J) + Real'Max (0.0, Inverse_Depth) * A (J)) / S (J);
      end loop;
      Ok := True;
   end Pixel_Residuals;

   --  Every kept pixel's residuals in one vector, a pixel's in place even when it is not Ok (then zeros, and Seen
   --  False).
   type Index_Array is array (Positive range <>) of Natural;

   type Layout (Pixels_All, Rows : Natural) is record
      Pose_Of_Pixel  : Index_Array (1 .. Pixels_All);
      Pixel_Of_Pixel : Index_Array (1 .. Pixels_All);
      First_Row      : Index_Array (1 .. Pixels_All);
      Count          : Index_Array (1 .. Pixels_All);
   end record;

   type Layout_Access is access Layout;
   procedure Free is new Ada.Unchecked_Deallocation (Layout, Layout_Access);

   function Lay_Out (P : Problem) return Layout_Access is
      Pixels, Rows : Natural := 0;
   begin
      for V in P.Poses'Range loop
         for I in 1 .. P.Poses (V).Pixels loop
            if P.Poses (V).Link (I) >= 2 then
               Pixels := Pixels + 1;
               Rows := Rows + P.Poses (V).Link (I);
            end if;
         end loop;
      end loop;
      declare
         L : constant Layout_Access := new Layout (Pixels, Rows);
         Q : Natural := 0;
         Row : Positive := 1;
      begin
         for V in P.Poses'Range loop
            for I in 1 .. P.Poses (V).Pixels loop
               if P.Poses (V).Link (I) >= 2 then
                  Q := Q + 1;
                  L.Pose_Of_Pixel (Q) := V;
                  L.Pixel_Of_Pixel (Q) := I;
                  L.First_Row (Q) := Row;
                  L.Count (Q) := P.Poses (V).Link (I);
                  Row := Row + P.Poses (V).Link (I);
               end if;
            end loop;
         end loop;
         return L;
      end;
   end Lay_Out;

   procedure All_Residuals
     (P : Problem; L : Layout; Base : Mat3; X : Parameters; Kept : Flag_Array; R : out Real_Array;
      Seen : out Flag_Array; Sequential : Boolean := False)
   is
      procedure One (Q : Positive) is
         Count : Natural;
         Rho   : Real;
         Ok    : Boolean;
         Part  : Real_Array (1 .. L.Count (Q));
      begin
         if Kept (Q) then
            Pixel_Residuals (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Base, X, Part, Count, Rho, Ok);
         else
            Ok := False;
         end if;
         Seen (Q) := Ok;
         for J in 1 .. L.Count (Q) loop
            R (R'First + L.First_Row (Q) + J - 2) := (if Ok then Part (J) else 0.0);
         end loop;
      end One;
      procedure Every is new Driver.Parallel_For (One);
   begin
      if Sequential then
         for Q in 1 .. L.Pixels_All loop
            One (Q);
         end loop;
      else
         Every (1, L.Pixels_All);
      end if;
   end All_Residuals;

   function Cost_Of (R : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for X of R loop
         Sum := Sum + X * X;
      end loop;
      return Sum;
   end Cost_Of;

   type Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);

   --  One pixel's residuals (as Pixel_Residuals has them) and their derivatives in the terms at X, X's turn nought
   --  (Base the camera's turn): the depth's inverse eliminated as a projection (variable projection, Golub and
   --  Pereyra). With y the rates' misfit before the depth and a the depth's column, both over their sigmas, r = y + rho
   --  a for rho = -a.y / a.a, and dr = dy + rho da - a (da.y + a.dy + 2 rho a.da) / a.a. A joint's turn in the camera's
   --  frame turns with the terms' turn e as w x e, the camera's centre in it moves as the frame's rows, and the focal
   --  length scales the gradient across and down and the line of sight inversely. Written out, not through the vector
   --  operators: the secondary stack they take was most of a fit's time (10-20, the arm's fit).
   procedure Pixel_Jacobian
     (P : Problem; V : Positive; I : Positive; Base : Mat3; X : Parameters; R : out Real_Array; D : out Real_Matrix;
      Count : out Natural; Ok : out Boolean)
   is
      View : Pose_View renames P.Poses (V).all;
      F    : constant Real := Exp (X (1));
      Fy   : constant Real := F * P.Aspect;
      Du   : constant Real := View.U (I) - X (8);
      Dv   : constant Real := View.V (I) - X (9);
      Xn   : constant Real := Du / F;
      Yn   : constant Real := Dv / Fy;
      Gu   : constant Real := View.Gu (I);
      Gv   : constant Real := View.Gv (I);
      --  The gradient in the camera's frame, and its derivative in the focal length's logarithm; the line of sight's.
      G1 : constant Real := F * Gu;
      G2 : constant Real := Fy * Gv;
      G3 : constant Real := -(Gu * Du + Gv * Dv);
      K    : constant Natural := View.Link (I);
      A, B, Sg : Real_Array (1 .. Natural'Max (1, K)) := [others => 0.0];
      DA, DB : Real_Matrix (1 .. Natural'Max (1, K), 1 .. Terms) := [others => [others => 0.0]];
      function G_Dot (V1, V2, V3 : Real) return Real is (G1 * V1 + G2 * V2 + G3 * V3);
      function GF_Dot (V1, V2 : Real) return Real is (G1 * V1 + G2 * V2);
   begin
      Count := K;
      Ok := False;
      for J in 1 .. K loop
         declare
            T  : Twist renames P.Twists (V, J);
            C  : constant Real := T.C;
            --  The joint's axis in the camera's frame: the transposed turn on it.
            W1 : constant Real := Base (1, 1) * T.W (1) + Base (2, 1) * T.W (2) + Base (3, 1) * T.W (3);
            W2 : constant Real := Base (1, 2) * T.W (1) + Base (2, 2) * T.W (2) + Base (3, 2) * T.W (3);
            W3 : constant Real := Base (1, 3) * T.W (1) + Base (2, 3) * T.W (2) + Base (3, 3) * T.W (3);
         begin
            Sg (J) := Sqrt ((P.Additive * View.Sigma (I, J)) ** 2 + (P.Relative * View.Rate (I, J)) ** 2);
            if T.Slide then
               A (J) := C * G_Dot (W1, W2, W3);
               DA (J, 1) := C * GF_Dot (W1, W2);
               --  w x e1 = (0, w3, -w2), w x e2 = (-w3, 0, w1), w x e3 = (w2, -w1, 0).
               DA (J, 2) := C * G_Dot (0.0, W3, -W2);
               DA (J, 3) := C * G_Dot (-W3, 0.0, W1);
               DA (J, 4) := C * G_Dot (W2, -W1, 0.0);
               --  The principal point moves the gradient's third coordinate by the gradient (G3 = -g . (u - c)).
               DA (J, 8) := C * Gu * W3;
               DA (J, 9) := C * Gv * W3;
            else
               declare
                  Q1 : constant Real := X (5) - T.P (1);
                  Q2 : constant Real := X (6) - T.P (2);
                  Q3 : constant Real := X (7) - T.P (3);
                  --  The centre from the joint's point, in the camera's frame.
                  U1 : constant Real := Base (1, 1) * Q1 + Base (2, 1) * Q2 + Base (3, 1) * Q3;
                  U2 : constant Real := Base (1, 2) * Q1 + Base (2, 2) * Q2 + Base (3, 2) * Q3;
                  U3 : constant Real := Base (1, 3) * Q1 + Base (2, 3) * Q2 + Base (3, 3) * Q3;
                  --  w x u and w x m (m the line of sight, its third coordinate one).
                  A1 : constant Real := W2 * U3 - W3 * U2;
                  A2 : constant Real := W3 * U1 - W1 * U3;
                  A3 : constant Real := W1 * U2 - W2 * U1;
                  B1 : constant Real := W2 - W3 * Yn;
                  B2 : constant Real := W3 * Xn - W1;
                  B3 : constant Real := W1 * Yn - W2 * Xn;
               begin
                  A (J) := C * G_Dot (A1, A2, A3);
                  B (J) := C * G_Dot (B1, B2, B3);
                  DA (J, 1) := C * GF_Dot (A1, A2);
                  --  d(w x m) for dm = (-xn, -yn, 0): w x dm = (w3 yn, -w3 xn, -w1 yn + w2 xn).
                  DB (J, 1) := C * (GF_Dot (B1, B2) + G_Dot (W3 * Yn, -W3 * Xn, W2 * Xn - W1 * Yn));
                  --  The principal point: G3 moves by the gradient, the line of sight by minus one over the focal
                  --  length across (w x dm = (0, -w3, w2) / f) or down (w x dm = (w3, 0, -w1) / fy).
                  DA (J, 8) := C * Gu * A3;
                  DA (J, 9) := C * Gv * A3;
                  DB (J, 8) := C * (Gu * B3 + (G3 * W2 - G2 * W3) / F);
                  DB (J, 9) := C * (Gv * B3 + (G1 * W3 - G3 * W1) / Fy);
                  for E in 1 .. 3 loop
                     declare
                        --  dw = w x e, du = u x e.
                        Dw1 : constant Real := (case E is when 1 => 0.0, when 2 => -W3, when others => W2);
                        Dw2 : constant Real := (case E is when 1 => W3, when 2 => 0.0, when others => -W1);
                        Dw3 : constant Real := (case E is when 1 => -W2, when 2 => W1, when others => 0.0);
                        Du1 : constant Real := (case E is when 1 => 0.0, when 2 => -U3, when others => U2);
                        Du2 : constant Real := (case E is when 1 => U3, when 2 => 0.0, when others => -U1);
                        Du3 : constant Real := (case E is when 1 => -U2, when 2 => U1, when others => 0.0);
                        --  The centre moved along the world's axis E, in the camera's frame: the turn's row E.
                        Dt1 : constant Real := Base (E, 1);
                        Dt2 : constant Real := Base (E, 2);
                        Dt3 : constant Real := Base (E, 3);
                     begin
                        DA (J, 1 + E) := C * G_Dot (Dw2 * U3 - Dw3 * U2 + W2 * Du3 - W3 * Du2,
                                                    Dw3 * U1 - Dw1 * U3 + W3 * Du1 - W1 * Du3,
                                                    Dw1 * U2 - Dw2 * U1 + W1 * Du2 - W2 * Du1);
                        DB (J, 1 + E) := C * G_Dot (Dw2 - Dw3 * Yn, Dw3 * Xn - Dw1, Dw1 * Yn - Dw2 * Xn);
                        DA (J, 4 + E) := C * G_Dot (W2 * Dt3 - W3 * Dt2, W3 * Dt1 - W1 * Dt3, W1 * Dt2 - W2 * Dt1);
                     end;
                  end loop;
               end;
            end if;
         end;
      end loop;
      declare
         Aa, Ay : Real := 0.0;
         Rho    : Real;
      begin
         for J in 1 .. K loop
            Aa := Aa + (A (J) / Sg (J)) ** 2;
            Ay := Ay + (A (J) / Sg (J)) * ((View.Rate (I, J) + B (J)) / Sg (J));
         end loop;
         if Aa <= 0.0 then
            return;
         end if;
         Rho := -Ay / Aa;
         if Rho <= 0.0 then
            --  The depth's inverse held at nought (Pixel_Residuals): the residual is y, its derivative dy.
            for J in 1 .. K loop
               R (R'First + J - 1) := (View.Rate (I, J) + B (J)) / Sg (J);
               for T in 1 .. Terms loop
                  D (D'First (1) + J - 1, D'First (2) + T - 1) := DB (J, T) / Sg (J);
               end loop;
            end loop;
            Ok := True;
            return;
         end if;
         for J in 1 .. K loop
            R (R'First + J - 1) := ((View.Rate (I, J) + B (J)) + Rho * A (J)) / Sg (J);
         end loop;
         for T in 1 .. Terms loop
            declare
               S1, S2, S3 : Real := 0.0;   --  da.y, a.dy, a.da
            begin
               for J in 1 .. K loop
                  S1 := S1 + (DA (J, T) / Sg (J)) * ((View.Rate (I, J) + B (J)) / Sg (J));
                  S2 := S2 + (A (J) / Sg (J)) * (DB (J, T) / Sg (J));
                  S3 := S3 + (A (J) / Sg (J)) * (DA (J, T) / Sg (J));
               end loop;
               for J in 1 .. K loop
                  D (D'First (1) + J - 1, D'First (2) + T - 1) :=
                    DB (J, T) / Sg (J) + Rho * DA (J, T) / Sg (J) - (A (J) / Sg (J)) * (S1 + S2 + 2.0 * Rho * S3) / Aa;
               end loop;
            end;
         end loop;
         Ok := True;
      end;
   end Pixel_Jacobian;

   --  Every kept pixel's residuals and their derivatives (Pixel_Jacobian), rows as All_Residuals lays them out; a pixel
   --  not Ok has zeros and Seen False.
   procedure All_Jacobian
     (P : Problem; L : Layout; Base : Mat3; X : Parameters; Kept : Flag_Array; R : out Real_Array;
      J : out Real_Matrix; Seen : out Flag_Array; Sequential : Boolean)
   is
      procedure One (Q : Positive) is
         Count : Natural;
         Ok    : Boolean := False;
         Part  : Real_Array (1 .. L.Count (Q));
         Rows  : Real_Matrix (1 .. L.Count (Q), 1 .. Terms);
      begin
         if Kept (Q) then
            Pixel_Jacobian (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Base, X, Part, Rows, Count, Ok);
         end if;
         Seen (Q) := Ok;
         for K in 1 .. L.Count (Q) loop
            declare
               Row : constant Positive := L.First_Row (Q) + K - 1;
            begin
               R (R'First + Row - 1) := (if Ok then Part (K) else 0.0);
               for T in 1 .. Terms loop
                  J (Row, T) := (if Ok then Rows (K, T) else 0.0);
               end loop;
            end;
         end loop;
      end One;
      procedure Every is new Driver.Parallel_For (One);
   begin
      if Sequential then
         for Q in 1 .. L.Pixels_All loop
            One (Q);
         end loop;
      else
         Every (1, L.Pixels_All);
      end if;
   end All_Jacobian;

   --  One step of damped Gauss-Newton from X (Levenberg-Marquardt): the Jacobian over the kept pixels seen at X
   --  (Pixel_Jacobian), the damping doubled until a step lowers the cost and sees those pixels, then halved (as the
   --  arm's fit and the points' fit take theirs). Done when the step lowered the cost by less than one residual's worth
   --  (the cost over the freedom the pixels leave, and no less than one: a residual is in units of its sigma, and a
   --  step that moves the cost by less moves nothing the pictures tell; with residuals five times their sigmas a floor
   --  of one took the steps on for a thirtieth of that each, 10-20) or no damping found one that lowers it. Cost: at X
   --  when the step ends. Normal: the undamped normal equations at the step's start. Sequential: the pixels one after
   --  another (the step runs on a worker of its own already).
   procedure Refine_Step
     (P : Problem; L : Layout; Base : in out Mat3; X : in out Parameters; Kept : Flag_Array;
      Sequential : Boolean; Lambda : in out Real; Normal : out Real_Matrix; Cost : out Real; Done : out Boolean)
   is
      R      : Real_Access := new Real_Array (1 .. L.Rows);
      Seen   : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
      J      : Matrix_Access := new Real_Matrix (1 .. L.Rows, 1 .. Terms);
      Rt     : Real_Access := new Real_Array (1 .. L.Rows);
      St     : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
      JtJ    : Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];
      Jtr    : Real_Vector (1 .. Terms) := [others => 0.0];
      Freedom : Natural := 0;
      Worth   : Real;
      procedure Release is
      begin
         Free (R);
         Free (Seen);
         Free (J);
         Free (Rt);
         Free (St);
      end Release;
   begin
      Done := True;
      --  The turn the terms carry taken into the base: the derivatives are at a nought turn (Pixel_Jacobian).
      Base := Times (Base, Driver.Numerics.Exp ([X (2), X (3), X (4)]));
      X (2 .. 4) := [0.0, 0.0, 0.0];
      All_Jacobian (P, L, Base, X, Kept, R.all, J.all, Seen.all, Sequential);
      Cost := Cost_Of (R.all);
      for Q in 1 .. L.Pixels_All loop
         if Seen (Q) then
            Freedom := Freedom + L.Count (Q) - 1;
         end if;
      end loop;
      Worth := Real'Max (1.0, (if Freedom > 0 then Cost / Real (Freedom) else 1.0));
      for Row in 1 .. L.Rows loop
         for A in 1 .. Terms loop
            Jtr (A) := Jtr (A) + J (Row, A) * R (Row);
            for B in A .. Terms loop
               JtJ (A, B) := JtJ (A, B) + J (Row, A) * J (Row, B);
            end loop;
         end loop;
      end loop;
      for A in 1 .. Terms loop
         for B in 1 .. A - 1 loop
            JtJ (A, B) := JtJ (B, A);
         end loop;
      end loop;
      Normal := JtJ;
      loop
         declare
            A        : Real_Matrix := JtJ;
            Lower    : Real_Matrix (1 .. Terms, 1 .. Terms);
            Definite : Boolean;
         begin
            for T in 1 .. Terms loop
               A (T, T) := A (T, T) * (1.0 + Lambda) + Lambda * Real'Model_Small;
            end loop;
            Driver.Numerics.Dense.Cholesky (A, Lower, Definite);
            if Definite then
               declare
                  Step   : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, -Jtr);
                  Trial  : Parameters := X;
                  Cost_T : Real;
               begin
                  for T in 1 .. Terms loop
                     Trial (T) := X (T) + Step (T);
                  end loop;
                  All_Residuals (P, L, Base, Trial, Kept, Rt.all, St.all, Sequential);
                  Cost_T := Cost_Of (Rt.all);
                  --  The trial must see the pixels the step was taken on.
                  if Cost_T < Cost and then (for all Q in 1 .. L.Pixels_All => not Seen (Q) or else St (Q)) then
                     Done := Cost - Cost_T <= Worth;
                     Base := Times (Base, Driver.Numerics.Exp ([Trial (2), Trial (3), Trial (4)]));
                     Trial (2 .. 4) := [0.0, 0.0, 0.0];
                     X := Trial;
                     Cost := Cost_T;
                     Lambda := Lambda / 2.0;
                     Release;
                     return;
                  end if;
               end;
            end if;
            exit when Lambda >= Real'Last / 2.0;
            Lambda := 2.0 * Lambda;
         end;
      end loop;
      Release;
   end Refine_Step;

   --  Refine_Step from X until it is done; Normal holds the undamped normal equations of the last step.
   procedure Converge
     (P : Problem; L : Layout; Base : in out Mat3; X : in out Parameters; Kept : Flag_Array; Normal : out Real_Matrix;
      Ok : out Boolean)
   is
      Lambda : Real := Driver.Conventions.Initial_Damping;
      Cost   : Real;
      Done   : Boolean := False;
   begin
      for Iteration in 1 .. Real'Machine_Mantissa loop
         Refine_Step (P, L, Base, X, Kept, False, Lambda, Normal, Cost, Done);
         exit when Done;
      end loop;
      Ok := True;
   end Converge;

   ---------------------------------------------------------------------------
   --  The start

   --  The camera's centre and the pixels' depths for a turn and a focal length, by turns: the depths for the centre,
   --  then the centre for the depths (a linear least squares in its three coordinates), until the cost stops falling by
   --  a residual's worth. Starts with the eye looking along its turn at the joints' places from where the pixels'
   --  spread over the picture puts the arm's spread.
   procedure Solve_Centre
     (P : Problem; L : Layout; Base : Mat3; Ln_F : Real; Arm_Spread, Pixel_Spread : Real; Middle : Vec3;
      Kept : Flag_Array; X : out Parameters; Cost : out Real)
   is
      F    : constant Real := Exp (Ln_F);
      Away : constant Real := (if Pixel_Spread > 0.0 then F * Arm_Spread / Pixel_Spread else Arm_Spread);
      Last : Real := Real'Last;
   begin
      X := [Ln_F, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P.Cx, P.Cy];
      declare
         Centre : constant Vec3 := Minus (Middle, Scaled (Away, Times (Base, Vec3'[0.0, 0.0, 1.0])));
      begin
         X (5 .. 7) := [Centre (1), Centre (2), Centre (3)];
      end;
      for Round in 1 .. Real'Machine_Mantissa loop
         --  The centre that best fits the depths the pixels have at the centre now: the residuals are linear in it.
         declare
            Ata : Real_Matrix (1 .. 3, 1 .. 3) := [others => [others => 0.0]];
            Atb : Real_Vector (1 .. 3) := [others => 0.0];
            Pose : constant Rigid := Pose_Of (Base, X);
         begin
            for Q in 1 .. L.Pixels_All loop
               declare
                  V    : constant Positive := L.Pose_Of_Pixel (Q);
                  I    : constant Positive := L.Pixel_Of_Pixel (Q);
                  View : Pose_View renames P.Poses (V).all;
                  Part : Real_Array (1 .. L.Count (Q));
                  Count : Natural;
                  Rho  : Real;
                  Ok   : Boolean := False;
               begin
                  if Kept (Q) then
                     Pixel_Residuals (P, V, I, Base, X, Part, Count, Rho, Ok);
                  end if;
                  --  A point behind the eye held at infinity (Pixel_Residuals): the centre moves nothing it shows.
                  Rho := Real'Max (0.0, Rho);
                  if Ok then
                     declare
                        Here : constant Sighting := Sight (P, F, X (8), X (9), View, I);
                        Ray  : constant Vec3 := Times (Pose.Rotation, Vec3'[Here.Xn, Here.Yn, 1.0]);
                     begin
                        for J in 1 .. Count loop
                           declare
                              T : Twist renames P.Twists (V, J);
                              S : constant Real := View.Sigma (I, J);
                           begin
                              if not T.Slide then
                                 --  e = Rate + B + Rho A, A = F g N R' (C w x (t - p)): linear in t.
                                 declare
                                    Row : Vec3;
                                    Fixed : Real;
                                    Bc : constant Vec3 :=
                                      Transposed_Times (Pose.Rotation, Scaled (T.C, Cross (T.W, Ray)));
                                    B  : constant Real := Dot (Here.Gn, Bc);
                                    Wp : constant Vec3 :=
                                      Transposed_Times (Pose.Rotation, Scaled (T.C, Cross (T.W, T.P)));
                                    Gn : Vec3 renames Here.Gn;
                                    --  A = Gn . R' (C w x t) - Gn . R' (C w x p); Gn . R' (C w x t) = (R (C w x ... )
                                    Skew_Col : Vec3;
                                 begin
                                    for Axis in 1 .. 3 loop
                                       declare
                                          E_Axis : Vec3 := [0.0, 0.0, 0.0];
                                       begin
                                          E_Axis (Axis) := 1.0;
                                          Skew_Col :=
                                            Transposed_Times (Pose.Rotation, Scaled (T.C, Cross (T.W, E_Axis)));
                                          Row (Axis) := Rho * Dot (Gn, Skew_Col);
                                       end;
                                    end loop;
                                    Fixed := View.Rate (I, J) + B - Rho * Dot (Gn, Wp);
                                    for A1 in 1 .. 3 loop
                                       for A2 in 1 .. 3 loop
                                          Ata (A1, A2) := Ata (A1, A2) + Row (A1) * Row (A2) / S ** 2;
                                       end loop;
                                       Atb (A1) := Atb (A1) - Row (A1) * Fixed / S ** 2;
                                    end loop;
                                 end;
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end;
            end loop;
            declare
               Lower    : Real_Matrix (1 .. 3, 1 .. 3);
               Definite : Boolean;
            begin
               Driver.Numerics.Dense.Cholesky (Ata, Lower, Definite);
               exit when not Definite;
               declare
                  T : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, Atb);
               begin
                  X (5 .. 7) := [T (1), T (2), T (3)];
               end;
            end;
         end;
         --  The cost over the pixels sampled, one after another: the search runs its candidates on every worker
         --  already (Driver.Parallel_For in Fit_Eye), and a pool asked for more work inside its own work serialized it.
         declare
            Now : Real := 0.0;
            Ok_Count, Sampled, Freedom : Natural := 0;
         begin
            for Q in 1 .. L.Pixels_All loop
               if Kept (Q) then
                  Sampled := Sampled + 1;
                  declare
                     Part  : Real_Array (1 .. L.Count (Q));
                     Count : Natural;
                     Rho   : Real;
                     Ok    : Boolean;
                  begin
                     Pixel_Residuals (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Base, X, Part, Count, Rho, Ok);
                     if Ok then
                        Ok_Count := Ok_Count + 1;
                        Freedom := Freedom + Count - 1;
                        for E of Part loop
                           Now := Now + E * E;
                        end loop;
                     end if;
                  end;
               end if;
            end loop;
            --  A centre the terms move no more than half the pixels at is no start: a state lowers its cost by losing them.
            Cost := (if 2 * Ok_Count > Sampled then Now else Real'Last);
            --  Done when a round lowers the cost by less than one residual's worth (Refine_Step).
            exit when Last - Now <= Real'Max (1.0, (if Freedom > 0 then Now / Real (Freedom) else 1.0));
            Last := Now;
         end;
      end loop;
   end Solve_Centre;

   ---------------------------------------------------------------------------
   --  The search

   --  The starts that must end at one optimum for it to be found, and the finalists of a race: twice them (as the
   --  points' search has them, Hand_Eye).
   Agreeing  : constant := 2;
   Finalists : constant := 2 * Agreeing;

   type Search_Verdict is (No_Start, Not_Reproduced, Ambiguous, Found_One);

   --  A start on its way to an optimum over the sampled pixels: its turn and terms, the damping its refinement has come
   --  to, whether that has converged, its cost and its score (the cost over the freedom the pixels leave; lower is
   --  better), and the normal equations of its last step.
   type Racer is record
      Base   : Mat3 := Identity3;
      X      : Parameters := [others => 0.0];
      Lambda : Real := Driver.Conventions.Initial_Damping;
      Done   : Boolean := False;
      Alive  : Boolean := True;
      Cost   : Real := Real'Last;
      Score  : Real := Real'Last;
      Normal : Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];
   end record;

   type Racer_Array is array (Positive range <>) of Racer;
   type Racer_Access is access Racer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Racer_Array, Racer_Access);

   --  A state's cost over the pixels Kept and its score: the cost over the freedom the pixels leave (a row for each joint
   --  that moves one, less one for its depth); Real'Last when the terms move no more than half of them (as a start of
   --  Solve_Centre's).
   procedure Judge (P : Problem; L : Layout; Kept : Flag_Array; Base : Mat3; X : Parameters; Cost, Score : out Real) is
      R    : Real_Access := new Real_Array (1 .. L.Rows);
      Seen : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
      In_Front, Sampled, Freedom : Natural := 0;
   begin
      All_Residuals (P, L, Base, X, Kept, R.all, Seen.all, Sequential => True);
      Cost := Cost_Of (R.all);
      for Q in 1 .. L.Pixels_All loop
         if Kept (Q) then
            Sampled := Sampled + 1;
            if Seen (Q) then
               In_Front := In_Front + 1;
               Freedom := Freedom + L.Count (Q) - 1;
            end if;
         end if;
      end loop;
      Score := (if 2 * In_Front > Sampled and then Freedom > 0 then Cost / Real (Freedom) else Real'Last);
      Free (R);
      Free (Seen);
   end Judge;

   --  How far apart two optima are for the pictures: the difference of their terms (the turn as the second's from the
   --  first's, in the first's camera frame, as the terms turn it) through the first's normal equations over its score,
   --  a chi square over the terms.
   function Apart (A, B : Racer) return Real is
      Ra : constant Mat3 := Times (A.Base, Driver.Numerics.Exp ([A.X (2), A.X (3), A.X (4)]));
      Rb : constant Mat3 := Times (B.Base, Driver.Numerics.Exp ([B.X (2), B.X (3), B.X (4)]));
      Turn : constant Vec3 := Driver.Numerics.Log (Times (Transposed (Ra), Rb));
      D    : constant Real_Vector (1 .. Terms) :=
        [B.X (1) - A.X (1), Turn (1), Turn (2), Turn (3), B.X (5) - A.X (5), B.X (6) - A.X (6), B.X (7) - A.X (7),
         B.X (8) - A.X (8), B.X (9) - A.X (9)];
   begin
      if A.Score = Real'Last or else A.Score <= 0.0 then
         return Real'Last;
      end if;
      return Real'Max (0.0, D * (A.Normal * D)) / A.Score;
   end Apart;

   function Same_Optimum (A, B : Racer) return Boolean is
     (Apart (A, B) < Real'Last
      and then Driver.Distributions.Chi_Square_Deviate (Real'Max (Real'Model_Epsilon, Apart (A, B)), Terms)
               <= Driver.Conventions.Z);

   --  One race over Rotations turns of the spiral (Driver.Numerics.Spiral_Rotation) at the focal lengths of the scan:
   --  the lengths from the half width seen at a right angle's half, half an octave apart, as the arm's own fit and the
   --  points' search start theirs (Hand_Eye.Scan_Focals), widened by a length on the side whose end scores best until a
   --  length inside scores at least as well as both ends (no field of view is assumed); every candidate's centre and
   --  depths by Solve_Centre. The best-ranked candidates, as many as there are turns, start the race: each takes a step
   --  of its refinement over the sample, the worse half drops out, the rest take twice as many, until the finalists are
   --  left; they run to their ends. Found_One when two of them ended at the best optimum and none ended at another that
   --  the pixels do not tell from it (Ambiguous); Reached: the finalists that ended at the best optimum.
   procedure Search_At
     (P : Problem; L : Layout; Sample : Flag_Array; Rotations : Positive; Arm_Spread, Pixel_Spread : Real;
      Middle : Vec3; Width : Positive; Best : out Racer; Verdict : out Search_Verdict; Reached : out Natural)
   is
      Half_Octave : constant Real := Ada.Numerics.Long_Elementary_Functions.Log (2.0) / 2.0;
      Start_Ln    : constant Real := Ada.Numerics.Long_Elementary_Functions.Log (Real (Width) / 2.0);
      --  The steps the scan may take either way: as far as the points' search goes, a logarithm of half what a real
      --  holds (Hand_Eye.Scan_Focals).
      Reach  : constant Positive :=
        Positive (Real'Floor (Ada.Numerics.Long_Elementary_Functions.Log (Real'Last) / 2.0 / Half_Octave));
      type Cost_Column is array (1 .. Rotations) of Real;
      type X_Column is array (1 .. Rotations) of Parameters;
      type Cost_Access is access Cost_Column;
      type X_Access is access X_Column;
      procedure Free is new Ada.Unchecked_Deallocation (Cost_Column, Cost_Access);
      procedure Free is new Ada.Unchecked_Deallocation (X_Column, X_Access);
      type Column is record
         Done  : Boolean := False;
         Least : Real := Real'Last;
         Costs : Cost_Access;
         Xs    : X_Access;
      end record;
      Columns   : array (-Reach .. Reach) of Column;
      Low, High : Integer := 0;
      Racers    : Racer_Access;
      Count     : Natural := 0;

      procedure Score (Step : Integer) is
         Ln_F : constant Real := Start_Ln + Real (Step) * Half_Octave;
         C    : Column renames Columns (Step);
         procedure One (K : Positive) is
         begin
            Solve_Centre (P, L, Driver.Numerics.Spiral_Rotation (K, Rotations), Ln_F, Arm_Spread, Pixel_Spread, Middle,
                          Sample, C.Xs (K), C.Costs (K));
         end One;
         procedure Every is new Driver.Parallel_For (One);
      begin
         C.Costs := new Cost_Column;
         C.Xs := new X_Column;
         Every (1, Rotations);
         C.Done := True;
         for K in 1 .. Rotations loop
            C.Least := Real'Min (C.Least, C.Costs (K));
         end loop;
      end Score;
   begin
      Verdict := No_Start;
      Reached := 0;
      Best := (others => <>);
      Low := -1;
      High := 1;
      for Step in Low .. High loop
         Score (Step);
      end loop;
      loop
         declare
            Inside : Real := Real'Last;
         begin
            for Step in Low + 1 .. High - 1 loop
               Inside := Real'Min (Inside, Columns (Step).Least);
            end loop;
            exit when Inside <= Columns (Low).Least and then Inside <= Columns (High).Least;
            if Columns (Low).Least <= Columns (High).Least then
               exit when Low = Columns'First;
               Low := Low - 1;
               Score (Low);
            else
               exit when High = Columns'Last;
               High := High + 1;
               Score (High);
            end if;
         end;
      end loop;
      --  The best-ranked candidates start the race, as many as there are turns.
      declare
         Total : Natural := 0;
      begin
         for Step in Low .. High loop
            for K in 1 .. Rotations loop
               if Columns (Step).Costs (K) < Real'Last then
                  Total := Total + 1;
               end if;
            end loop;
         end loop;
         Racers := new Racer_Array (1 .. Natural'Min (Rotations, Total));
         declare
            Taken : array (Low .. High, 1 .. Rotations) of Boolean := [others => [others => False]];
         begin
            while Count < Racers'Length loop
               declare
                  Pick_S : Integer := 0;
                  Pick_K : Natural := 0;
                  Least  : Real := Real'Last;
               begin
                  for Step in Low .. High loop
                     for K in 1 .. Rotations loop
                        if not Taken (Step, K) and then Columns (Step).Costs (K) < Least then
                           Least := Columns (Step).Costs (K);
                           Pick_S := Step;
                           Pick_K := K;
                        end if;
                     end loop;
                  end loop;
                  exit when Pick_K = 0;
                  Taken (Pick_S, Pick_K) := True;
                  Count := Count + 1;
                  Racers (Count).Base := Driver.Numerics.Spiral_Rotation (Pick_K, Rotations);
                  Racers (Count).X := Columns (Pick_S).Xs (Pick_K);
               end;
            end loop;
         end;
      end;
      for Step in Low .. High loop
         Free (Columns (Step).Costs);
         Free (Columns (Step).Xs);
      end loop;
      if Count = 0 then
         Free (Racers);
         return;
      end if;
      --  The race: every racer takes Budget steps, the worse half drops out, the budget doubles.
      declare
         Alive  : Natural := Count;
         Budget : Positive := 1;
         procedure Run_One (C : Positive) is
            Racing : Racer renames Racers (C);
         begin
            if Racing.Alive then
               for Step in 1 .. Budget loop
                  exit when Racing.Done;
                  Refine_Step (P, L, Racing.Base, Racing.X, Sample, True, Racing.Lambda, Racing.Normal,
                               Racing.Cost, Racing.Done);
               end loop;
               Judge (P, L, Sample, Racing.Base, Racing.X, Racing.Cost, Racing.Score);
            end if;
         end Run_One;
         procedure Run_Every is new Driver.Parallel_For (Run_One);
      begin
         while Alive > Finalists loop
            Run_Every (1, Count);
            declare
               Keep : constant Natural := Natural'Max (Finalists, (Alive + 1) / 2);
            begin
               for C in 1 .. Count loop
                  if Racers (C).Alive then
                     declare
                        Better : Natural := 0;
                     begin
                        for Other in 1 .. Count loop
                           if Racers (Other).Alive and then Other /= C
                             and then (Racers (Other).Score < Racers (C).Score
                                       or else (Racers (Other).Score = Racers (C).Score and then Other < C))
                           then
                              Better := Better + 1;
                           end if;
                        end loop;
                        if Better >= Keep or else Racers (C).Score = Real'Last then
                           Racers (C).Alive := False;
                        end if;
                     end;
                  end if;
               end loop;
               Alive := 0;
               for C in 1 .. Count loop
                  if Racers (C).Alive then
                     Alive := Alive + 1;
                  end if;
               end loop;
            end;
            Budget := 2 * Budget;
         end loop;
      end;
      --  The finalists to their ends.
      declare
         procedure End_One (C : Positive) is
            Racing : Racer renames Racers (C);
         begin
            if Racing.Alive then
               for Iteration in 1 .. Real'Machine_Mantissa loop
                  exit when Racing.Done;
                  Refine_Step (P, L, Racing.Base, Racing.X, Sample, True, Racing.Lambda, Racing.Normal,
                               Racing.Cost, Racing.Done);
               end loop;
               --  The normal equations at the end, for the distances.
               declare
                  Lambda : Real := Racing.Lambda;
                  Base   : Mat3 := Racing.Base;
                  X      : Parameters := Racing.X;
                  Cost   : Real;
                  Done   : Boolean;
               begin
                  Refine_Step (P, L, Base, X, Sample, True, Lambda, Racing.Normal, Cost, Done);
               end;
               Judge (P, L, Sample, Racing.Base, Racing.X, Racing.Cost, Racing.Score);
            end if;
         end End_One;
         procedure End_Every is new Driver.Parallel_For (End_One);
         Top   : Natural := 0;
         Rival : Boolean := False;
      begin
         End_Every (1, Count);
         --  The finalists refined on every pixel and judged on them all: the sample's optimum is the sample's (seed 22,
         --  10-20: the race's eye left 3.46 times the pixels' sigmas where the true one leaves 3.13).
         declare
            Every_Pixel : constant Flag_Array (1 .. L.Pixels_All) := [others => True];
            Ok          : Boolean;
            --  Finalists that ended at one optimum of the sample are refined once, as the first of them: the others
            --  take its end.
            Twin_Of     : array (1 .. Count) of Natural := [others => 0];
         begin
            for C in 1 .. Count loop
               if Racers (C).Alive and then Racers (C).Score < Real'Last then
                  for D in 1 .. C - 1 loop
                     if Twin_Of (D) = 0 and then Racers (D).Alive and then Racers (D).Score < Real'Last
                       and then Same_Optimum (Racers (D), Racers (C))
                     then
                        Twin_Of (C) := D;
                        exit;
                     end if;
                  end loop;
               end if;
            end loop;
            for C in 1 .. Count loop
               if Racers (C).Alive and then Racers (C).Score < Real'Last and then Twin_Of (C) = 0 then
                  Converge (P, L, Racers (C).Base, Racers (C).X, Every_Pixel, Racers (C).Normal, Ok);
                  Judge (P, L, Every_Pixel, Racers (C).Base, Racers (C).X, Racers (C).Cost, Racers (C).Score);
               end if;
            end loop;
            for C in 1 .. Count loop
               if Twin_Of (C) /= 0 then
                  Racers (C).Base := Racers (Twin_Of (C)).Base;
                  Racers (C).X := Racers (Twin_Of (C)).X;
                  Racers (C).Normal := Racers (Twin_Of (C)).Normal;
                  Racers (C).Cost := Racers (Twin_Of (C)).Cost;
                  Racers (C).Score := Racers (Twin_Of (C)).Score;
               end if;
            end loop;
         end;
         for C in 1 .. Count loop
            if Racers (C).Alive and then Racers (C).Score < Real'Last
              and then (Top = 0 or else Racers (C).Score < Racers (Top).Score)
            then
               Top := C;
            end if;
         end loop;
         if Top = 0 then
            Free (Racers);
            return;
         end if;
         Best := Racers (Top);
         Verdict := Not_Reproduced;
         for C in 1 .. Count loop
            if Racers (C).Alive and then Racers (C).Score < Real'Last then
               if C = Top or else Same_Optimum (Racers (Top), Racers (C)) then
                  Reached := Reached + 1;
               elsif Driver.Distributions.Chi_Square_Deviate
                       (Real'Max (Real'Model_Epsilon, (Racers (C).Cost - Racers (Top).Cost) / Racers (Top).Score),
                        Terms)
                     <= Driver.Conventions.Z
               then
                  Rival := True;
               end if;
            end if;
         end loop;
         if Rival then
            Verdict := Ambiguous;
         elsif Reached >= Agreeing then
            Verdict := Found_One;
         end if;
      end;
      Free (Racers);
   end Search_At;

   --  The start: the race run at counts of turns that double (Search_At), from as many as it has finalists, until a
   --  doubling changes nothing: the best optimum of the race is the one of the race before (Same_Optimum) and so is its
   --  verdict, or the race finds that optimum with two of its starts ending at it (Found_One). The spiral covers the
   --  turns twice as finely each time, and an optimum whose basin no start of the coarser race fell in shows when one
   --  of the finer race does (as the points' search, Hand_Eye.Search). The count stops doubling at the sampled pixels'
   --  number: a start for every pixel sampled is as fine as the sample can tell starts apart.
   procedure Search
     (P : Problem; L : Layout; Sample : Flag_Array; Arm_Spread, Pixel_Spread : Real; Middle : Vec3; Width : Positive;
      Best : out Racer; Verdict : out Search_Verdict; Rotations : out Positive)
   is
      Sampled     : Natural := 0;
      Was         : Racer;
      Was_Verdict : Search_Verdict := No_Start;
      Was_Reached : Natural := 0;
      First       : Boolean := True;
   begin
      for S of Sample loop
         if S then
            Sampled := Sampled + 1;
         end if;
      end loop;
      Rotations := Finalists;
      Best := (others => <>);
      Verdict := No_Start;
      loop
         declare
            Now     : Racer;
            Now_V   : Search_Verdict;
            Reached : Natural;
            Same    : Boolean := False;
            Last    : constant Boolean := Rotations >= Sampled;
            Holds   : Boolean;
            Began   : constant Duration := Driver.Clock.Seconds;
         begin
            Search_At (P, L, Sample, Rotations, Arm_Spread, Pixel_Spread, Middle, Width, Now, Now_V, Reached);
            if not First and then Now_V /= No_Start and then Was_Verdict /= No_Start then
               --  The same optimum, or another the pixels do not tell from it: their costs (each over every pixel the
               --  arm moved) apart by no significant chi square over the terms. A finer race will not choose between
               --  two eyes the pixels cannot (seed 22, arm 2, 10-20: races at 16 to 128 turns ended at 154.3, 153.1
               --  and 154.4 px, each optimum apart from the one before by its own covariance, and the count doubled to
               --  the end).
               Same := Same_Optimum (Now, Was)
                 or else (Now.Score < Real'Last and then Was.Score < Real'Last
                          and then Driver.Distributions.Chi_Square_Deviate
                                     (Real'Max (Real'Model_Epsilon,
                                                abs (Now.Cost - Was.Cost) / Real'Max (Now.Score, Was.Score)),
                                      Terms) <= Driver.Conventions.Z);
            end if;
            --  The best optimum of this race is the best of the race before, and the starts of the two that ended at it
            --  are as many as the agreement asks for between them (the races' turns are other ones).
            if Same and then Now_V = Not_Reproduced and then Was_Verdict /= Ambiguous
              and then Reached + Was_Reached >= Agreeing
            then
               Now_V := Found_One;
            end if;
            Holds := Last
              or else (not First
                       and then ((Same and then (Now_V = Found_One or else Now_V = Was_Verdict))
                                 or else (Now_V = No_Start and then Was_Verdict = No_Start)));
            Best := Now;
            Verdict := Now_V;
            Driver.Log.Line (Driver.Log.Robot, "kinematics: the fixed eye's flow search at" & Rotations'Image
                             & " turns: " & Now_V'Image & ", reached" & Reached'Image & ", focal "
                             & Driver.Log.Image (Exp (Now.X (1)), 1) & ", score " & Driver.Log.Image (Now.Score, 2)
                             & ", " & Driver.Log.Image (Real (Driver.Clock.Seconds - Began), 1) & " s");
            exit when Holds;
            Was := Now;
            Was_Verdict := Now_V;
            Was_Reached := Reached;
            First := False;
            Rotations := Natural'Min (2 * Rotations, Natural'Max (Rotations + 1, Sampled));
         end;
      end loop;
   end Search;

   ---------------------------------------------------------------------------
   --  The fit of every arm the eye watched, through its one lens

   --  The fit's terms: the logarithm of the focal length and the principal point, the lens every arm is seen through,
   --  then each arm's turn about the camera's own axes and centre. An arm's own nine (Parameters) are these, taken out.
   Lens_Count : constant := 3;
   Arm_Count_Terms : constant := 6;

   function Global_Count (Arms : Positive) return Positive is (Lens_Count + Arm_Count_Terms * Arms);

   function Offset (A : Positive) return Natural is (Lens_Count + Arm_Count_Terms * (A - 1));

   function Local (X : Real_Array; A : Positive) return Parameters is
     ([X (X'First), X (X'First + Offset (A)), X (X'First + Offset (A) + 1), X (X'First + Offset (A) + 2),
       X (X'First + Offset (A) + 3), X (X'First + Offset (A) + 4), X (X'First + Offset (A) + 5),
       X (X'First + 1), X (X'First + 2)]);

   --  The place in the fit's terms of an arm's own term T.
   function Global_Of (A : Positive; T : Positive) return Positive is
     (case T is
         when 1      => 1,
         when 8      => 2,
         when 9      => 3,
         when others => Offset (A) + T - 1);

   type Arm_State is record
      P      : Problem;
      L      : Layout_Access;
      Base   : Mat3 := Identity3;
      Kept   : Flag_Access;
      Searched : Search_Verdict := No_Start;
      Spread_Of_Arm : Real := 0.0;   --  its joints' places about their middle
      Score    : Real := Real'Last;   --  its search's best: the cost over the freedom its sampled pixels leave
   end record;

   type Arm_State_Array is array (Positive range <>) of Arm_State;

   --  The fit's turn carried by the terms taken into each arm's base: the derivatives are at a nought turn.
   procedure Fold (S : in out Arm_State_Array; X : in out Real_Array) is
   begin
      for A in S'Range loop
         declare
            O : constant Natural := X'First + Offset (A);
         begin
            S (A).Base := Times (S (A).Base, Driver.Numerics.Exp ([X (O), X (O + 1), X (O + 2)]));
            X (O .. O + 2) := [0.0, 0.0, 0.0];
         end;
      end loop;
   end Fold;

   --  Every arm's kept pixels' residuals at X: the cost, the freedom they leave (a row each joint, less one for a
   --  pixel's depth), and per arm the pixels seen.
   type Seen_Array is array (Positive range <>) of Flag_Access;

   procedure Joint_Cost
     (S : Arm_State_Array; X : Real_Array; Seen : Seen_Array; Cost : out Real; Freedom : out Natural)
   is
   begin
      Cost := 0.0;
      Freedom := 0;
      for A in S'Range loop
         declare
            R : Real_Access := new Real_Array (1 .. S (A).L.Rows);
         begin
            All_Residuals (S (A).P, S (A).L.all, S (A).Base, Local (X, A), S (A).Kept.all, R.all, Seen (A).all);
            Cost := Cost + Cost_Of (R.all);
            for Q in 1 .. S (A).L.Pixels_All loop
               if Seen (A) (Q) then
                  Freedom := Freedom + S (A).L.Count (Q) - 1;
               end if;
            end loop;
            Free (R);
         end;
      end loop;
   end Joint_Cost;

   --  One step of damped Gauss-Newton on every arm at once (Refine_Step's, the arms' normal equations summed into the
   --  fit's terms).
   procedure Joint_Step
     (S : in out Arm_State_Array; X : in out Real_Array; Lambda : in out Real; Normal : out Real_Matrix;
      Cost : out Real; Done : out Boolean)
   is
      G      : constant Positive := X'Length;
      JtJ    : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
      Jtr    : Real_Vector (1 .. G) := [others => 0.0];
      Seen, Trial_Seen : Seen_Array (S'Range);
      Freedom : Natural := 0;
      Worth  : Real;
      procedure Release is
      begin
         for A in S'Range loop
            Free (Seen (A));
            Free (Trial_Seen (A));
         end loop;
      end Release;
   begin
      Done := True;
      Fold (S, X);
      Cost := 0.0;
      for A in S'Range loop
         Seen (A) := new Flag_Array (1 .. S (A).L.Pixels_All);
         Trial_Seen (A) := new Flag_Array (1 .. S (A).L.Pixels_All);
         declare
            L : Layout renames S (A).L.all;
            R : Real_Access := new Real_Array (1 .. L.Rows);
            J : Matrix_Access := new Real_Matrix (1 .. L.Rows, 1 .. Terms);
         begin
            All_Jacobian (S (A).P, L, S (A).Base, Local (X, A), S (A).Kept.all, R.all, J.all, Seen (A).all, False);
            Cost := Cost + Cost_Of (R.all);
            for Q in 1 .. L.Pixels_All loop
               if Seen (A) (Q) then
                  Freedom := Freedom + L.Count (Q) - 1;
               end if;
            end loop;
            for Row in 1 .. L.Rows loop
               for T in 1 .. Terms loop
                  declare
                     GT : constant Positive := Global_Of (A, T);
                  begin
                     Jtr (GT) := Jtr (GT) + J (Row, T) * R (Row);
                     for U in 1 .. Terms loop
                        JtJ (GT, Global_Of (A, U)) := JtJ (GT, Global_Of (A, U)) + J (Row, T) * J (Row, U);
                     end loop;
                  end;
               end loop;
            end loop;
            Free (R);
            Free (J);
         end;
      end loop;
      Normal := JtJ;
      Worth := Real'Max (1.0, (if Freedom > 0 then Cost / Real (Freedom) else 1.0));
      loop
         declare
            M        : Real_Matrix := JtJ;
            Lower    : Real_Matrix (1 .. G, 1 .. G);
            Definite : Boolean;
         begin
            for T in 1 .. G loop
               M (T, T) := M (T, T) * (1.0 + Lambda) + Lambda * Real'Model_Small;
            end loop;
            Driver.Numerics.Dense.Cholesky (M, Lower, Definite);
            if Definite then
               declare
                  Step    : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, -Jtr);
                  Trial   : Real_Array (X'Range) := X;
                  Cost_T  : Real;
                  Free_T  : Natural;
                  Keeps   : Boolean := True;
               begin
                  for T in 1 .. G loop
                     Trial (X'First + T - 1) := X (X'First + T - 1) + Step (T);
                  end loop;
                  Joint_Cost (S, Trial, Trial_Seen, Cost_T, Free_T);
                  for A in S'Range loop
                     for Q in 1 .. S (A).L.Pixels_All loop
                        if Seen (A) (Q) and then not Trial_Seen (A) (Q) then
                           Keeps := False;
                        end if;
                     end loop;
                  end loop;
                  --  The trial must see the pixels the step was taken on.
                  if Cost_T < Cost and then Keeps then
                     Done := Cost - Cost_T <= Worth;
                     X := Trial;
                     Fold (S, X);
                     Cost := Cost_T;
                     Lambda := Lambda / 2.0;
                     Release;
                     return;
                  end if;
               end;
            end if;
            exit when Lambda >= Real'Last / 2.0;
            Lambda := 2.0 * Lambda;
         end;
      end loop;
      Release;
   end Joint_Step;

   procedure Joint_Converge (S : in out Arm_State_Array; X : in out Real_Array; Normal : out Real_Matrix) is
      Lambda : Real := Driver.Conventions.Initial_Damping;
      Cost   : Real;
      Done   : Boolean := False;
   begin
      for Iteration in 1 .. Real'Machine_Mantissa loop
         Joint_Step (S, X, Lambda, Normal, Cost, Done);
         exit when Done;
      end loop;
   end Joint_Converge;

   --  An arm's start: its pixels sampled, its search, and the rates' spread at the search's eye (Problem's Additive
   --  and Relative). Searched No_Start when no turn and focal length move its pixels.
   procedure Start_Arm (S : in out Arm_State; Joints : Fit.Joint_Array; X : out Parameters) is
      P     : Problem renames S.P;
      L     : Layout renames S.L.all;
      Poses : Pose_Array renames P.Poses.all;
      Width : constant Positive := P.Width;
      Height : constant Positive := P.Height;
      Middle : Vec3 := [0.0, 0.0, 0.0];
      Arm_Spread, Pixel_Spread : Real := 0.0;
      Su, Sv, Suu, Svv : Real := 0.0;
      --  The search on one pixel of every patch of each pose's picture that the arm moves in (a patch as wide as the
      --  aligner's: twice Driver.Alignment.Half_Patch), the one whose rates stand out the most (the sum of their
      --  squares over their sigmas): neighbouring pixels of one edge tell the start no more than one of them, and
      --  every pixel at every candidate took the search minutes.
      Patch  : constant Positive := 2 * Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
      Sample : Flag_Array (1 .. L.Pixels_All) := [others => False];
   begin
      X := [others => 0.0];
      declare
         Across : constant Positive := (Width + Patch - 1) / Patch;
         Down   : constant Positive := (Height + Patch - 1) / Patch;
         Chosen : array (Poses'Range, 0 .. Across - 1, 0 .. Down - 1) of Natural :=
           [others => [others => [others => 0]]];
         Most   : array (Poses'Range, 0 .. Across - 1, 0 .. Down - 1) of Real :=
           [others => [others => [others => -1.0]]];
      begin
         for Q in Sample'Range loop
            declare
               V : constant Positive := L.Pose_Of_Pixel (Q);
               I : constant Positive := L.Pixel_Of_Pixel (Q);
               A : constant Natural :=
                 Natural'Min (Across - 1, Natural (Real'Floor (Poses (V).U (I) / Real (Patch))));
               B : constant Natural :=
                 Natural'Min (Down - 1, Natural (Real'Floor (Poses (V).V (I) / Real (Patch))));
               Told : Real := 0.0;
            begin
               for J in 1 .. L.Count (Q) loop
                  Told := Told + (Poses (V).Rate (I, J) / Poses (V).Sigma (I, J)) ** 2;
               end loop;
               if Told > Most (V, A, B) then
                  Most (V, A, B) := Told;
                  Chosen (V, A, B) := Q;
               end if;
            end;
         end loop;
         for Q of Chosen loop
            if Q > 0 then
               Sample (Q) := True;
            end if;
         end loop;
      end;
      for J in Joints'Range loop
         Middle := Plus (Middle, Joints (J).P);
      end loop;
      Middle := Scaled (1.0 / Real (Joints'Length), Middle);
      for J in Joints'Range loop
         Arm_Spread := Arm_Spread + Dot (Minus (Joints (J).P, Middle), Minus (Joints (J).P, Middle));
      end loop;
      Arm_Spread := Sqrt (Arm_Spread / Real (Joints'Length));
      S.Spread_Of_Arm := Arm_Spread;
      for Q in 1 .. L.Pixels_All loop
         declare
            View : Pose_View renames Poses (L.Pose_Of_Pixel (Q)).all;
            I    : constant Positive := L.Pixel_Of_Pixel (Q);
         begin
            Su := Su + View.U (I);
            Sv := Sv + View.V (I);
            Suu := Suu + View.U (I) ** 2;
            Svv := Svv + View.V (I) ** 2;
         end;
      end loop;
      Pixel_Spread := Sqrt (Real'Max (0.0, (Suu / Real (L.Pixels_All) - (Su / Real (L.Pixels_All)) ** 2)
                                      + (Svv / Real (L.Pixels_All) - (Sv / Real (L.Pixels_All)) ** 2)));
      declare
         Found     : Racer;
         Rotations : Positive;
      begin
         Search (P, L, Sample, Arm_Spread, Pixel_Spread, Middle, Width, Found, S.Searched, Rotations);
         if S.Searched = No_Start then
            return;
         end if;
         S.Base := Found.Base;
         S.Score := Found.Score;
         X := Found.X;
      end;
      --  The rates' spread at the search's eye: each rate's squared residual against its sigma squared and its own
      --  square (least squares through the origin, neither share below its floor: the pictures' noise is at least
      --  what they state, and no share is negative), so that a rate the pictures tell far beyond the linear model's
      --  reach weighs as much as the model holds it to and no more (seed 22, 10-20: residuals grew with the rates, a
      --  median 4.6 sigmas where the rates stood 100 to 300 sigmas out).
      declare
         Every_Pixel : constant Flag_Array (1 .. L.Pixels_All) := [others => True];
         R    : Real_Access := new Real_Array (1 .. L.Rows);
         Seen : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
         Sss, Ssr, Srr, Se_S, Se_R : Real := 0.0;   --  the normal equations of e = a s + k r over the rows
      begin
         All_Residuals (P, L, S.Base, X, Every_Pixel, R.all, Seen.all);
         for Q in 1 .. L.Pixels_All loop
            if Seen (Q) then
               declare
                  View : Pose_View renames Poses (L.Pose_Of_Pixel (Q)).all;
                  I    : constant Positive := L.Pixel_Of_Pixel (Q);
               begin
                  for K in 0 .. L.Count (Q) - 1 loop
                     declare
                        --  A pixel's depth takes one of its rows' freedoms: a row keeps (Count - 1) / Count of its
                        --  spread on the mean.
                        Kept_Share : constant Real := Real (L.Count (Q) - 1) / Real (L.Count (Q));
                        Sigma2 : constant Real := Kept_Share * View.Sigma (I, K + 1) ** 2;
                        Rate2  : constant Real := Kept_Share * View.Rate (I, K + 1) ** 2;
                        Raw2   : constant Real := (R (L.First_Row (Q) + K) * View.Sigma (I, K + 1)) ** 2;
                     begin
                        Sss := Sss + Sigma2 * Sigma2;
                        Ssr := Ssr + Sigma2 * Rate2;
                        Srr := Srr + Rate2 * Rate2;
                        Se_S := Se_S + Raw2 * Sigma2;
                        Se_R := Se_R + Raw2 * Rate2;
                     end;
                  end loop;
               end;
            end if;
         end loop;
         declare
            Det : constant Real := Sss * Srr - Ssr * Ssr;
            A2  : Real := (if Det > 0.0 then (Se_S * Srr - Se_R * Ssr) / Det else 1.0);
            K2  : Real := (if Det > 0.0 then (Sss * Se_R - Ssr * Se_S) / Det else 0.0);
         begin
            if A2 < 1.0 then
               A2 := 1.0;
               K2 := (if Srr > 0.0 then (Se_R - Ssr) / Srr else 0.0);
            end if;
            if K2 < 0.0 then
               K2 := 0.0;
               A2 := Real'Max (1.0, (if Sss > 0.0 then Se_S / Sss else 1.0));
            end if;
            P.Additive := Sqrt (A2);
            P.Relative := Sqrt (K2);
         end;
         Free (R);
         Free (Seen);
      end;
   end Start_Arm;

   procedure Fit_Eye
     (Arms          : Arm_Flow_Array;
      Width, Height : Positive;
      Result        : out Report)
   is
      Count : constant Positive := Arms'Length;
      G     : constant Positive := Global_Count (Count);
      S     : Arm_State_Array (1 .. Count);
      X     : Real_Array (1 .. G) := [others => 0.0];
      Normal : Real_Matrix (1 .. G, 1 .. G);
      Scatter : Real := 1.0;
      Kept_All, Offered_All : Natural := 0;
      Best_Score : Real := Real'Last;

      procedure Release is
      begin
         for A in S'Range loop
            Free (S (A).L);
            Free (S (A).P.Twists);
            Free (S (A).Kept);
         end loop;
      end Release;
   begin
      Result := (Arms => Count, Count => G, others => <>);
      for A in S'Range loop
         declare
            Arm : Arm_Flow renames Arms (Arms'First + A - 1);
         begin
            S (A).P := (Poses => Arm.Poses, Twists => null, Width => Width, Height => Height,
                        Joints => Arm.Joints'Length, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0,
                        Aspect => 1.0, Additive => 1.0, Relative => 0.0);
            S (A).P.Twists := new Twist_Grid (Arm.Poses'Range, 1 .. Natural'Max (1, Arm.Joints'Length));
            for V in Arm.Poses'Range loop
               declare
                  T : constant Twist_Array := Twists_At (Arm.Joints.all, Arm.Poses (V).Readings);
               begin
                  for J in T'Range loop
                     S (A).P.Twists (V, J) := T (J);
                  end loop;
               end;
            end loop;
            S (A).L := Lay_Out (S (A).P);
            S (A).Kept := new Flag_Array'(1 .. S (A).L.Pixels_All => True);
            Offered_All := Offered_All + S (A).L.Pixels_All;
         end;
      end loop;
      Result.Offered := Offered_All;
      if (for some A of S => A.L.Pixels_All < Terms) then
         Result.Why := To_Unbounded_String ("too few pixels that two joints or more move on an arm");
         Release;
         return;
      end if;
      --  Every arm's start; the lens of the arm whose search fits its pixels best (the least score): an arm seen on
      --  one side of the picture can end its search at a lens far off (seed 23, arm 2, 10-21: a focal length of 3e9 px,
      --  the eye at infinity), and a mean of the arms' lenses would start them all there.
      for A in S'Range loop
         declare
            Own : Parameters;
         begin
            Start_Arm (S (A), Arms (Arms'First + A - 1).Joints.all, Own);
            if S (A).Searched = No_Start then
               Result.Why := To_Unbounded_String ("no turn and focal length move an arm's pixels");
               Release;
               return;
            end if;
            for T in 2 .. 7 loop
               X (Global_Of (A, T)) := Own (T);
            end loop;
            if A = S'First or else S (A).Score < Best_Score then
               Best_Score := S (A).Score;
               X (1) := Own (1);
               X (2) := Own (8);
               X (3) := Own (9);
            end if;
         end;
         Result.Additive := Real'Max (Result.Additive, S (A).P.Additive);
         Result.Relative := Real'Max (Result.Relative, S (A).P.Relative);
      end loop;
      --  The refinement, the pixels off by more than the gate of the fit's scatter left out until none is.
      loop
         Joint_Converge (S, X, Normal);
         declare
            Seen    : Seen_Array (S'Range);
            Sum     : Real := 0.0;
            Freedom : Natural := 0;
            Dropped : Natural := 0;
            Pixels  : Natural := 0;
         begin
            for A in S'Range loop
               Seen (A) := new Flag_Array (1 .. S (A).L.Pixels_All);
            end loop;
            Joint_Cost (S, X, Seen, Sum, Freedom);
            Scatter := Real'Max (1.0, (if Freedom > G then Sqrt (Sum / Real (Freedom - G)) else 1.0));
            for A in S'Range loop
               Pixels := Pixels + S (A).L.Pixels_All;
            end loop;
            Kept_All := 0;
            for A in S'Range loop
               declare
                  L : Layout renames S (A).L.all;
                  R : Real_Access := new Real_Array (1 .. L.Rows);
                  Seen_A : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
               begin
                  All_Residuals (S (A).P, L, S (A).Base, Local (X, A), S (A).Kept.all, R.all, Seen_A.all);
                  for Q in 1 .. L.Pixels_All loop
                     if S (A).Kept (Q) then
                        if not Seen_A (Q) then
                           S (A).Kept (Q) := False;
                           Dropped := Dropped + 1;
                        else
                           declare
                              Own  : Real := 0.0;
                              Gate : constant Driver.Uncertain.Gate :=
                                Driver.Uncertain.Vector_Gate (L.Count (Q) - 1, 0, Tests => Pixels);
                           begin
                              for K in 0 .. L.Count (Q) - 1 loop
                                 Own := Own + R (L.First_Row (Q) + K) ** 2;
                              end loop;
                              if Driver.Uncertain.Significant (Gate, Sqrt (Own), Scatter) then
                                 S (A).Kept (Q) := False;
                                 Dropped := Dropped + 1;
                              end if;
                           end;
                        end if;
                     end if;
                     if S (A).Kept (Q) then
                        Kept_All := Kept_All + 1;
                     end if;
                  end loop;
                  Free (R);
                  Free (Seen_A);
               end;
            end loop;
            for A in S'Range loop
               Free (Seen (A));
            end loop;
            exit when Dropped = 0 or else Kept_All < G;
         end;
      end loop;
      Result.Used := Kept_All;
      Result.Scatter := Scatter;
      Result.Lens := (Fx => Exp (X (1)), Fy => Exp (X (1)), Cx => X (2), Cy => X (3), K1 | K2 => 0.0);
      for A in S'Range loop
         Result.Poses (A) := Pose_Of (S (A).Base, Local (X, A));
      end loop;
      declare
         Lower    : Real_Matrix (1 .. G, 1 .. G);
         Definite : Boolean;
      begin
         Driver.Numerics.Dense.Cholesky (Normal, Lower, Definite);
         if not Definite then
            Result.Why := To_Unbounded_String ("the pixels leave some term undetermined");
            Release;
            return;
         end if;
         for T in 1 .. G loop
            declare
               E      : Real_Vector (1 .. G) := [others => 0.0];
               Column : Real_Vector (1 .. G);
            begin
               E (T) := 1.0;
               Column := Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
               for Q in 1 .. G loop
                  Result.Covariance (Q, T) := Scatter ** 2 * Column (Q);
               end loop;
            end;
         end loop;
      end;
      --  The pictures of one pose share their errors (the arm's edges at their places in the pixels, the misfit where
      --  its parts meet), which the scatter of all the pixels' residuals does not tell (seed 22, 10-20: told within
      --  0.2 % of focal length and 0.7 mm of centre, off by 6.9 % and 43 mm). The covariance is widened to the eye's
      --  spread with one pose of one arm left out at a time (the jackknife over the poses) in every direction that
      --  spread exceeds it, and kept in the others (widened by the largest eigenvalue in every direction, it made one
      --  pose's pull on the turns, seed 20 10-20, the focal length's uncertainty too, 46 %). Each pose's shift is
      --  weighed by the rows it holds, N / m - 1 of N in all (the jackknife of groups of unequal size, Busing, Meijer
      --  and van der Leeden 1999), so that under the model the spread is the covariance itself. The joints are no
      --  such groups: one left out takes with it what the fit knows of some direction, exact rates or not, and a
      --  jackknife over them widened the synthetic arm's exact rates out of told (robot.flow.eye: 17.96 px unweighed,
      --  14.51 weighed). What one joint's rates share (seed 22 10-20: joint 1's 10 to 15 % against 4 to 7 % for the
      --  others, arm 2 off by 26.7 mm with a sigma of 6.3) is left to a model of their own.
      declare
         Poses_All : Natural := 0;
      begin
         for A in S'Range loop
            Poses_All := Poses_All + Arms (Arms'First + A - 1).Poses'Length;
         end loop;
         if Poses_All >= 2 then
            declare
               Blocks   : constant Natural := Poses_All;
               type Theta_Array is array (1 .. Blocks) of Real_Vector (1 .. G);
               Theta    : Theta_Array;                                    --  each block's shift of the terms
               Weight   : Real_Vector (1 .. Blocks) := [others => 0.0];   --  and its weight, N / m - 1
               N_Out    : Natural := 0;
               Rows_All : Natural := 0;

               --  The covariance widened to Spread in every direction Spread exceeds it, and kept in the others: with
               --  Lc Lc' the covariance, M = Lc^-1 Spread Lc^-T = V values V', W = V max (1, values) V', and the
               --  covariance Lc W Lc'.
               procedure Widen (Spread : Real_Matrix) is
                  Lc  : Real_Matrix (1 .. G, 1 .. G);
                  Def : Boolean;
                  function Forward (B : Real_Vector) return Real_Vector is
                     Y : Real_Vector (1 .. G);
                  begin
                     for I in 1 .. G loop
                        Y (I) := B (B'First + I - 1);
                        for K in 1 .. I - 1 loop
                           Y (I) := Y (I) - Lc (I, K) * Y (K);
                        end loop;
                        Y (I) := Y (I) / Lc (I, I);
                     end loop;
                     return Y;
                  end Forward;
               begin
                  Driver.Numerics.Dense.Cholesky (Result.Covariance, Lc, Def);
                  if not Def then
                     return;
                  end if;
                  declare
                     Y, M    : Real_Matrix (1 .. G, 1 .. G);
                     Values  : Real_Vector (1 .. G);
                     Vectors : Real_Matrix (1 .. G, 1 .. G);
                     W       : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                     T       : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                  begin
                     for C in 1 .. G loop
                        declare
                           Column : Real_Vector (1 .. G);
                        begin
                           for R in 1 .. G loop
                              Column (R) := Spread (R, C);
                           end loop;
                           Column := Forward (Column);
                           for R in 1 .. G loop
                              Y (R, C) := Column (R);
                           end loop;
                        end;
                     end loop;
                     --  M = (Lc^-1 Y')'.
                     for C in 1 .. G loop
                        declare
                           Column : Real_Vector (1 .. G);
                        begin
                           for R in 1 .. G loop
                              Column (R) := Y (C, R);
                           end loop;
                           Column := Forward (Column);
                           for R in 1 .. G loop
                              M (C, R) := Column (R);
                           end loop;
                        end;
                     end loop;
                     Driver.Numerics.Dense.Small_Eigensystem (M, Values, Vectors);
                     for V of Values loop
                        Result.Widened := Real'Max (Result.Widened, V);
                     end loop;
                     for I in 1 .. G loop
                        for A in 1 .. G loop
                           for B in 1 .. G loop
                              W (A, B) := W (A, B) + Real'Max (1.0, Values (I)) * Vectors (A, I) * Vectors (B, I);
                           end loop;
                        end loop;
                     end loop;
                     for A in 1 .. G loop
                        for B in 1 .. G loop
                           for K in 1 .. A loop
                              T (A, B) := T (A, B) + Lc (A, K) * W (K, B);
                           end loop;
                        end loop;
                     end loop;
                     for A in 1 .. G loop
                        for B in 1 .. G loop
                           Result.Covariance (A, B) := 0.0;
                           for K in 1 .. B loop
                              Result.Covariance (A, B) := Result.Covariance (A, B) + T (A, K) * Lc (B, K);
                           end loop;
                        end loop;
                     end loop;
                  end;
               end Widen;

               --  The jackknife's spread: the blocks' shifts, each weighted, averaged over the blocks.
               function Spread return Real_Matrix is
                  Result_S : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
               begin
                  for K in 1 .. N_Out loop
                     for A in 1 .. G loop
                        for B in 1 .. G loop
                           Result_S (A, B) := Result_S (A, B) + Weight (K) * Theta (K) (A) * Theta (K) (B) / Real (N_Out);
                        end loop;
                     end loop;
                  end loop;
                  return Result_S;
               end Spread;

               --  The fit again without pose V of arm A; the rows it leaves out weigh its shift.
               procedure Leave_Out (A, V : Positive) is
                  S_V      : Arm_State_Array := S;
                  X_V      : Real_Array := X;
                  Normal_V : Real_Matrix (1 .. G, 1 .. G);
                  Rows_Out : Natural := 0;
               begin
                  S_V (A).Kept := new Flag_Array'(S (A).Kept.all);
                  for Q in 1 .. S (A).L.Pixels_All loop
                     if S (A).Kept (Q) then
                        if S (A).L.Pose_Of_Pixel (Q) = V then
                           S_V (A).Kept (Q) := False;
                           Rows_Out := Rows_Out + S (A).L.Count (Q);
                        end if;
                     end if;
                  end loop;
                  if Rows_Out > 0 and then Rows_Out < Rows_All then
                     Joint_Converge (S_V, X_V, Normal_V);
                     N_Out := N_Out + 1;
                     Weight (N_Out) := Real (Rows_All) / Real (Rows_Out) - 1.0;
                     Theta (N_Out) := [others => 0.0];
                     Theta (N_Out) (1) := X_V (1) - X (1);
                     Theta (N_Out) (2) := X_V (2) - X (2);
                     Theta (N_Out) (3) := X_V (3) - X (3);
                     for B in S'Range loop
                        declare
                           Turn : constant Vec3 :=
                             Driver.Numerics.Log
                               (Times (Transposed (Pose_Of (S (B).Base, Local (X, B)).Rotation),
                                       Pose_Of (S_V (B).Base, Local (X_V, B)).Rotation));
                           O    : constant Natural := Offset (B);
                        begin
                           Theta (N_Out) (O + 1) := Turn (1);
                           Theta (N_Out) (O + 2) := Turn (2);
                           Theta (N_Out) (O + 3) := Turn (3);
                           Theta (N_Out) (O + 4) := X_V (O + 4) - X (O + 4);
                           Theta (N_Out) (O + 5) := X_V (O + 5) - X (O + 5);
                           Theta (N_Out) (O + 6) := X_V (O + 6) - X (O + 6);
                        end;
                     end loop;
                  end if;
                  Free (S_V (A).Kept);
               end Leave_Out;
            begin
               for A in S'Range loop
                  for Q in 1 .. S (A).L.Pixels_All loop
                     if S (A).Kept (Q) then
                        Rows_All := Rows_All + S (A).L.Count (Q);
                     end if;
                  end loop;
               end loop;
               for A in S'Range loop
                  for V in Arms (Arms'First + A - 1).Poses'Range loop
                     Leave_Out (A, V);
                  end loop;
               end loop;
               Widen (Spread);
            end;
         end if;
      end;
      --  How far the covariance leaves the arms' points in the picture: each kept pixel's point (at the depth the fit
      --  puts it) projected by the terms moved along the covariance, the worst of them.
      declare
         Worst : Real := 0.0;
         Eps3  : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);
      begin
         for A in S'Range loop
            declare
               L     : Layout renames S (A).L.all;
               Poses : Pose_Array renames Arms (Arms'First + A - 1).Poses.all;
               XA    : constant Parameters := Local (X, A);
            begin
               for Q in 1 .. L.Pixels_All loop
                  if S (A).Kept (Q) then
                     declare
                        V    : constant Positive := L.Pose_Of_Pixel (Q);
                        I    : constant Positive := L.Pixel_Of_Pixel (Q);
                        View : Pose_View renames Poses (V).all;
                        Part : Real_Array (1 .. L.Count (Q));
                        Count_Q : Natural;
                        Rho  : Real;
                        Ok_P : Boolean;
                     begin
                        Pixel_Residuals (S (A).P, V, I, S (A).Base, XA, Part, Count_Q, Rho, Ok_P);
                        --  A point at infinity (behind the eye, Pixel_Residuals) has no place to move.
                        if Ok_P and then Rho > 0.0 then
                           declare
                              Pose  : constant Rigid := Pose_Of (S (A).Base, XA);
                              F     : constant Real := Exp (XA (1));
                              Here  : constant Sighting := Sight (S (A).P, F, XA (8), XA (9), View, I);
                              Point : constant Vec3 :=
                                Plus (Pose.Translation,
                                      Scaled (1.0 / Rho, Times (Pose.Rotation, Vec3'[Here.Xn, Here.Yn, 1.0])));
                              Ju, Jv : Real_Vector (1 .. G) := [others => 0.0];
                           begin
                              for T in 1 .. Terms loop
                                 declare
                                    H : constant Real :=
                                      (if T <= 4 then Eps3 else Eps3 * Real'Max (S (A).Spread_Of_Arm, abs XA (T)));
                                    function Project (Y : Parameters) return Vec3 is
                                       Pp : constant Rigid := Pose_Of (S (A).Base, Y);
                                       C  : constant Vec3 :=
                                         Transposed_Times (Pp.Rotation, Minus (Point, Pp.Translation));
                                       Fl : constant Real := Exp (Y (1));
                                    begin
                                       return [Fl * C (1) / C (3) + Y (8), Fl * S (A).P.Aspect * C (2) / C (3) + Y (9),
                                               0.0];
                                    end Project;
                                    Up, Down : Parameters := XA;
                                 begin
                                    Up (T) := XA (T) + H;
                                    Down (T) := XA (T) - H;
                                    declare
                                       PU : constant Vec3 := Project (Up);
                                       PD : constant Vec3 := Project (Down);
                                    begin
                                       Ju (Global_Of (A, T)) := (PU (1) - PD (1)) / (2.0 * H);
                                       Jv (Global_Of (A, T)) := (PU (2) - PD (2)) / (2.0 * H);
                                    end;
                                 end;
                              end loop;
                              declare
                                 Vu : constant Real := Ju * (Result.Covariance * Ju);
                                 Vv : constant Real := Jv * (Result.Covariance * Jv);
                              begin
                                 Worst := Real'Max (Worst, Sqrt (Real'Max (0.0, Real'Max (Vu, Vv))));
                              end;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end loop;
         Result.Spread := Worst;
      end;
      if (for some A of S => A.Searched = Ambiguous) then
         Result.Why := To_Unbounded_String ("an arm's search ended at two eyes its pixels do not tell apart");
      elsif (for some A of S => A.Searched /= Found_One) then
         Result.Why := To_Unbounded_String ("no two of an arm's search's starts ended at its best eye, nor did two races");
      elsif Result.Spread * Driver.Conventions.Z >= Real (Driver.Alignment.Half_Patch (Width, Height)) then
         Result.Why := To_Unbounded_String
           ("the pixels leave the arms' points in the picture uncertain by" & Driver.Log.Image (Result.Spread, 2)
            & " px");
      else
         Result.Determined := True;
      end if;
      Release;
   end Fit_Eye;

   procedure Judge_At
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Scatter       : out Real;
      In_Front      : out Natural)
   is
      Held : aliased constant Pose_Array := Poses;
      P    : Problem := (Poses => Held'Unchecked_Access, Twists => null, Width => Width, Height => Height,
                         Joints => Joints'Length, Cx => Lens.Cx, Cy => Lens.Cy,
                         Aspect => Lens.Fy / Lens.Fx, Additive => 1.0, Relative => 0.0);
      L    : Layout_Access;
   begin
      P.Twists := new Twist_Grid (Poses'Range, 1 .. Natural'Max (1, Joints'Length));
      for V in Poses'Range loop
         declare
            T : constant Twist_Array := Twists_At (Joints, Poses (V).Readings);
         begin
            for J in T'Range loop
               P.Twists (V, J) := T (J);
            end loop;
         end;
      end loop;
      L := Lay_Out (P);
      declare
         X    : constant Parameters :=
           [Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fx), 0.0, 0.0, 0.0,
            Pose.Translation (1), Pose.Translation (2), Pose.Translation (3), Lens.Cx, Lens.Cy];
         Kept : constant Flag_Array (1 .. L.Pixels_All) := [others => True];
         R    : Real_Access := new Real_Array (1 .. L.Rows);
         Seen : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
         Sum  : Real := 0.0;
         Freedom : Natural := 0;
      begin
         All_Residuals (P, L.all, Pose.Rotation, X, Kept, R.all, Seen.all);
         In_Front := 0;
         for Q in 1 .. L.Pixels_All loop
            if Seen (Q) then
               declare
                  Part  : Real_Array (1 .. L.Count (Q));
                  Count : Natural;
                  Rho   : Real;
                  Ok    : Boolean;
               begin
                  Pixel_Residuals (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Pose.Rotation, X, Part, Count, Rho, Ok);
                  if Rho > 0.0 then
                     In_Front := In_Front + 1;
                  end if;
               end;
               Freedom := Freedom + L.Count (Q) - 1;
               for K in 0 .. L.Count (Q) - 1 loop
                  Sum := Sum + R (L.First_Row (Q) + K) ** 2;
               end loop;
            end if;
         end loop;
         Scatter := (if Freedom > 0 then Sqrt (Sum / Real (Freedom)) else Real'Last);
         --  For the study (BD_FLOW_STUDY naming a file): every pixel the terms move, its pose, place, gradient,
         --  link, the inverse depth the model gives it and each joint's measured rate against the model's.
         if Ada.Environment_Variables.Exists ("BD_FLOW_STUDY") then
            declare
               use Ada.Text_IO;
               File : File_Type;
            begin
               Create (File, Out_File, Ada.Environment_Variables.Value ("BD_FLOW_STUDY"));
               for Q in 1 .. L.Pixels_All loop
                  if Seen (Q) then
                     declare
                        V    : constant Positive := L.Pose_Of_Pixel (Q);
                        I    : constant Positive := L.Pixel_Of_Pixel (Q);
                        View : Pose_View renames Poses (V).all;
                        Part : Real_Array (1 .. L.Count (Q));
                        Count : Natural;
                        Rho  : Real;
                        Ok   : Boolean;
                     begin
                        Pixel_Residuals (P, V, I, Pose.Rotation, X, Part, Count, Rho, Ok);
                        Put (File, Positive'Image (V) & Real'Image (View.U (I)) & Real'Image (View.V (I))
                             & Real'Image (View.Gu (I)) & Real'Image (View.Gv (I)) & Natural'Image (View.Link (I))
                             & Real'Image (Rho) & Real'Image (View.Back (I)));
                        for J in 1 .. L.Count (Q) loop
                           Put (File, Real'Image (View.Rate (I, J)) & Real'Image (Part (J) * View.Sigma (I, J))
                                & Real'Image (View.Sigma (I, J)));
                        end loop;
                        New_Line (File);
                     end;
                  end if;
               end loop;
               Close (File);
            end;
         end if;
         Free (R);
         Free (Seen);
      end;
      Free (L);
      Free (P.Twists);
   end Judge_At;

   procedure Synthesize
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Deviates      : not null access function return Real)
   is
      Held : aliased constant Pose_Array := Poses;
      P    : Problem := (Poses => Held'Unchecked_Access, Twists => null, Width => Width, Height => Height,
                         Joints => Joints'Length, Cx => Lens.Cx, Cy => Lens.Cy,
                         Aspect => Lens.Fy / Lens.Fx, Additive => 1.0, Relative => 0.0);
      L    : Layout_Access;
      X    : constant Parameters :=
        [Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fx), 0.0, 0.0, 0.0,
         Pose.Translation (1), Pose.Translation (2), Pose.Translation (3), Lens.Cx, Lens.Cy];
   begin
      P.Twists := new Twist_Grid (Poses'Range, 1 .. Natural'Max (1, Joints'Length));
      for V in Poses'Range loop
         declare
            T : constant Twist_Array := Twists_At (Joints, Poses (V).Readings);
         begin
            for J in T'Range loop
               P.Twists (V, J) := T (J);
            end loop;
         end;
      end loop;
      L := Lay_Out (P);
      for Q in 1 .. L.Pixels_All loop
         declare
            V     : constant Positive := L.Pose_Of_Pixel (Q);
            I     : constant Positive := L.Pixel_Of_Pixel (Q);
            View  : Pose_View renames Poses (V).all;
            Part  : Real_Array (1 .. L.Count (Q));
            Count : Natural;
            Rho   : Real;
            Ok    : Boolean;
         begin
            Pixel_Residuals (P, V, I, Pose.Rotation, X, Part, Count, Rho, Ok);
            if Ok then
               for J in 1 .. Count loop
                  --  The model's rate at the eye (the measured less its residual), and the pictures' noise on it.
                  View.Rate (I, J) :=
                    View.Rate (I, J) - Part (J) * View.Sigma (I, J) + Deviates.all * View.Sigma (I, J);
               end loop;
            end if;
         end;
      end loop;
      Free (L);
      Free (P.Twists);
   end Synthesize;

end Driver.Robot.Kinematics.Flow_Eye;
