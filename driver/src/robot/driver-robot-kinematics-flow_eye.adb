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
with Driver.Stats;
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

   function Smoothing_Radius return Natural is (Smooth_Radius);

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
      --  alone (the fake's textured table, 10-20: at the true eye its gradient left the outline's pixels 25 to 45 %
      --  off; taken from the still only where no joint's turn changed the pixel, the band an edge sweeps kept it, and
      --  the principal point came out 13 to 16 px off on three seeds). The gradient of what moves is the smoothed
      --  gradient less the smoothed gradient of the background where it shows.
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
      --  One pixel of every cell a smoothing radius wide, the one whose changes stand out the most: the smoothed noise
      --  of pixels that far apart correlates by exp (-radius^2 / (4 sigma^2)), an eighth, and the pixels of one edge
      --  closer than that tell the fit the same thing again while each takes its time (10-20: some 20 000 pixels for
      --  two arms at six poses each, a fit minutes long).
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

   function Best_Turn
     (Still, Turned : Real_Array;
      Turn          : Real;
      Noise         : Real;
      Width, Height : Positive) return Real
   is
      Pixels  : constant Positive := Width * Height;
      Gate    : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Tests => Pixels));
      Sigma_G : constant Real := Noise / Sqrt (2.0) * Sqrt (Square_Sum (Slope) * Square_Sum (Smoothing));
      Sigma_C : constant Real := Noise * Square_Sum (Smoothing);
      Gu      : Real_Access := new Real_Array (1 .. Pixels);
      Gv      : Real_Access := new Real_Array (1 .. Pixels);
      Change  : Real_Access := new Real_Array (1 .. Pixels);
      Diff    : Real_Access := new Real_Array (1 .. Pixels);
      Moved   : Natural := 0;
      Result  : Real := Turn;
   begin
      Filter (Still, Width, Height, Slope, Smoothing, Gu.all);
      Filter (Still, Width, Height, Smoothing, Slope, Gv.all);
      for K in 0 .. Pixels - 1 loop
         Diff (K + 1) := Turned (Turned'First + K) - Still (Still'First + K);
      end loop;
      Filter (Diff.all, Width, Height, Smoothing, Smoothing, Change.all);
      for K in 1 .. Pixels loop
         if Sqrt (Gu (K) ** 2 + Gv (K) ** 2) > Gate * Sigma_G and then abs Change (K) > Gate * Sigma_C then
            Moved := Moved + 1;
         end if;
      end loop;
      if Moved > 0 then
         declare
            Motions, Slopes : Real_Access := new Real_Array (1 .. Moved);
            I : Natural := 0;
         begin
            for K in 1 .. Pixels loop
               declare
                  G : constant Real := Sqrt (Gu (K) ** 2 + Gv (K) ** 2);
               begin
                  if G > Gate * Sigma_G and then abs Change (K) > Gate * Sigma_C then
                     I := I + 1;
                     Motions (I) := abs Change (K) / G / Turn;
                     Slopes (I) := G;
                  end if;
               end;
            end loop;
            declare
               Per_Unit : constant Real := Driver.Stats.Median_In_Place (Motions.all);
               Slope_M  : constant Real := Driver.Stats.Median_In_Place (Slopes.all);
               Best     : constant Real :=
                 (3.0 * Smooth_Sigma ** 2 * Sigma_C / (Sqrt (2.0) * Slope_M)) ** (1.0 / 3.0);
            begin
               if Per_Unit > 0.0 then
                  Result := Best / Per_Unit;
               end if;
            end;
            Free (Motions);
            Free (Slopes);
         end;
      end if;
      Free (Gu);
      Free (Gv);
      Free (Change);
      Free (Diff);
      return Result;
   end Best_Turn;

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
   --  moved its centre by that angle times its distance (seed 22, 10-20: 1.6 degrees and 17 mm); and the logarithm of
   --  the pixels' aspect (the focal length down over across): held square, the fake's eyes (aspects 0.9960 to 1.0007)
   --  left seed 25's arms' joints 1.69 px from the true eye's places against a spread of 1.62 and the eye's terms 237
   --  chi squares over 15 off the truth, seed 22's 832 (10-21).
   Terms : constant := 10;
   type Parameters is array (1 .. Terms) of Real;

   type Problem is record
      Poses  : access constant Pose_Array;
      Twists : Twist_Grid_Access;     --  per pose, per joint
      Width, Height : Positive := 1;
      Joints : Natural := 0;
      Cx, Cy : Real := 0.0;           --  where the fit starts the principal point, and the focal length down over
      Aspect : Real := 1.0;           --  across (the terms carry them)
      --  The lens's bend the views are read through (Sight), at the terms' focal lengths and principal point: nought,
      --  a pinhole's views as they are.
      K1, K2 : Real := 0.0;
      --  A rate's spread: Additive times the sigma its pictures state, and Relative times the rate (what the pictures
      --  share beyond their noise: an edge's place in the pixels, the turn's reach past the linear), in quadrature.
      Additive : Real := 1.0;
      Relative : Real := 0.0;
      --  Each joint's unit times its own factor (the fit's units' terms, Fit_Eye): none, each one.
      Unit : Real_Access;
   end record;

   function Unit_Of (P : Problem; J : Positive) return Real is (if P.Unit = null then 1.0 else P.Unit (J));

   --  A pixel's line of sight in the camera's frame (its third coordinate one) at the focal lengths F across and Fy
   --  down, and its gradient carried to that frame: the change of the picture there per unit of a point's motion over
   --  its depth.
   type Sighting is record
      Xn, Yn : Real;
      Gn     : Vec3;
   end record;

   --  Pixel I of a view as the flows' pinhole takes it, at the focal lengths F across and Fy down and the principal
   --  point: through the lens's radial distortion K1, K2 where it bends (its line of sight by Fit.Ray, the picture's
   --  gradient carried to the pinhole's pixels by the distortion's derivative), as it is where it bends nought; its
   --  rates and their sigmas are the pixel's own. Pixel_Jacobian's derivatives are a pinhole's (Fit_Eye's flows bend
   --  nought).
   function Sight (F, Fy, Cx, Cy, K1, K2 : Real; View : Pose_View; I : Positive) return Sighting is
   begin
      if K1 = 0.0 and then K2 = 0.0 then
         declare
            Xn : constant Real := (View.U (I) - Cx) / F;
            Yn : constant Real := (View.V (I) - Cy) / Fy;
         begin
            return (Xn => Xn, Yn => Yn,
                    Gn => [F * View.Gu (I), Fy * View.Gv (I), -(F * View.Gu (I) * Xn + Fy * View.Gv (I) * Yn)]);
         end;
      end if;
      declare
         M  : constant Vec3 := Fit.Ray ((Fx => F, Fy => Fy, Cx => Cx, Cy => Cy, K1 => K1, K2 => K2),
                                        View.U (I), View.V (I));
         Xn : constant Real := M (1);
         Yn : constant Real := M (2);
         R2 : constant Real := Xn * Xn + Yn * Yn;
         D  : constant Real := 1.0 + K1 * R2 + K2 * R2 * R2;
         S  : constant Real := 2.0 * K1 + 4.0 * K2 * R2;
         --  The picture's gradient in the pinhole's pixels: the distorted place (x D, y D) moves with the pinhole's
         --  as d(x D)/dx = D + S x^2, d(x D)/dy = S x y, d(y D)/dy = D + S y^2.
         Gu : constant Real := View.Gu (I) * (D + S * Xn * Xn) + View.Gv (I) * (Fy / F * S * Xn * Yn);
         Gv : constant Real := View.Gu (I) * (F / Fy * S * Xn * Yn) + View.Gv (I) * (D + S * Yn * Yn);
      begin
         return (Xn => Xn, Yn => Yn, Gn => [F * Gu, Fy * Gv, -(F * Gu * Xn + Fy * Gv * Yn)]);
      end;
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
      Here : constant Sighting := Sight (F, F * Exp (X (10)), X (8), X (9), P.K1, P.K2, View, I);
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
            C  : constant Real := T.C * Unit_Of (P, J);
            Av : constant Vec3 :=
              (if T.Slide then Scaled (C, T.W) else Scaled (C, Cross (T.W, Minus (Pose.Translation, T.P))));
            Bv : constant Vec3 := (if T.Slide then [0.0, 0.0, 0.0] else Scaled (C, Cross (T.W, Ray)));
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
   type Index_Access is access Index_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);

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
   --  frame turns with the terms' turn e as w x e, the camera's centre in it moves as the frame's rows, the focal
   --  length scales the gradient across and down and the line of sight inversely, and the aspect the same down alone
   --  (dGy = Gy, dyn = -yn). Written out, not through the vector operators: the secondary stack they take was most of
   --  a fit's time (10-20, the arm's fit).
   procedure Pixel_Jacobian
     (P : Problem; V : Positive; I : Positive; Base : Mat3; X : Parameters; R : out Real_Array; D : out Real_Matrix;
      D_Unit : out Real_Matrix; Count : out Natural; Ok : out Boolean)
   is
      View : Pose_View renames P.Poses (V).all;
      F    : constant Real := Exp (X (1));
      Fy   : constant Real := F * Exp (X (10));
      Du   : constant Real := View.U (I) - X (8);
      Dv   : constant Real := View.V (I) - X (9);
      Xn   : constant Real := Du / F;
      Yn   : constant Real := Dv / Fy;
      Gu   : constant Real := View.Gu (I);
      Gv   : constant Real := View.Gv (I);
      --  The gradient in the camera's frame, and its derivative in the focal length's logarithm; the line of sight's.
      Gx : constant Real := F * Gu;
      Gy : constant Real := Fy * Gv;
      Gz : constant Real := -(Gu * Du + Gv * Dv);
      K    : constant Natural := View.Link (I);
      A, B, Sg : Real_Array (1 .. Natural'Max (1, K)) := [others => 0.0];
      DA, DB : Real_Matrix (1 .. Natural'Max (1, K), 1 .. Terms) := [others => [others => 0.0]];
      function G_Dot (V1, V2, V3 : Real) return Real is (Gx * V1 + Gy * V2 + Gz * V3);
      function GF_Dot (V1, V2 : Real) return Real is (Gx * V1 + Gy * V2);
   begin
      Count := K;
      Ok := False;
      for J in 1 .. K loop
         declare
            T  : Twist renames P.Twists (V, J);
            C  : constant Real := T.C * Unit_Of (P, J);
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
               --  The principal point moves the gradient's third coordinate by the gradient (Gz = -g . (u - c)).
               DA (J, 8) := C * Gu * W3;
               DA (J, 9) := C * Gv * W3;
               DA (J, 10) := C * Gy * W2;
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
                  --  The principal point: Gz moves by the gradient, the line of sight by minus one over the focal
                  --  length across (w x dm = (0, -w3, w2) / f) or down (w x dm = (w3, 0, -w1) / fy).
                  DA (J, 8) := C * Gu * A3;
                  DA (J, 9) := C * Gv * A3;
                  DB (J, 8) := C * (Gu * B3 + (Gz * W2 - Gy * W3) / F);
                  DB (J, 9) := C * (Gv * B3 + (Gx * W3 - Gz * W1) / Fy);
                  --  The aspect: Gy and yn scale, w x dm = (w3 yn, 0, -w1 yn) for dm = (0, -yn, 0).
                  DA (J, 10) := C * Gy * A2;
                  DB (J, 10) := C * (Gy * B2 + Yn * (Gx * W3 - Gz * W1));
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
               for U in 1 .. K loop
                  D_Unit (D_Unit'First (1) + J - 1, D_Unit'First (2) + U - 1) :=
                    (if U = J then B (J) / Sg (J) else 0.0);
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
         --  A joint's unit scales its own a and b: da = a_U e_U, dy = b_U e_U.
         for U in 1 .. K loop
            declare
               A_U : constant Real := A (U) / Sg (U);
               B_U : constant Real := B (U) / Sg (U);
               Y_U : constant Real := (View.Rate (I, U) + B (U)) / Sg (U);
            begin
               for J in 1 .. K loop
                  D_Unit (D_Unit'First (1) + J - 1, D_Unit'First (2) + U - 1) :=
                    (if J = U then B_U + Rho * A_U else 0.0)
                    - (A (J) / Sg (J)) * A_U * (Y_U + B_U + 2.0 * Rho * A_U) / Aa;
               end loop;
            end;
         end loop;
         Ok := True;
      end;
   end Pixel_Jacobian;

   --  Every kept pixel's residuals and their derivatives (Pixel_Jacobian), rows as All_Residuals lays them out, and in
   --  JU the derivatives in each joint's unit (a column each, of as many joints as JU has columns: none when it has
   --  none); a pixel not Ok has zeros and Seen False.
   procedure All_Jacobian
     (P : Problem; L : Layout; Base : Mat3; X : Parameters; Kept : Flag_Array; R : out Real_Array;
      J : out Real_Matrix; JU : out Real_Matrix; Seen : out Flag_Array; Sequential : Boolean)
   is
      procedure One (Q : Positive) is
         Count : Natural;
         Ok    : Boolean := False;
         Part  : Real_Array (1 .. L.Count (Q));
         Rows  : Real_Matrix (1 .. L.Count (Q), 1 .. Terms);
         Units : Real_Matrix (1 .. L.Count (Q), 1 .. L.Count (Q));
      begin
         if Kept (Q) then
            Pixel_Jacobian (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Base, X, Part, Rows, Units, Count, Ok);
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
               for U in JU'Range (2) loop
                  JU (Row, U) := (if Ok and then U - JU'First (2) + 1 <= L.Count (Q)
                                  then Units (K, U - JU'First (2) + 1) else 0.0);
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
      No_Units : Real_Matrix (1 .. L.Rows, 1 .. 0);   --  one arm's refinement holds its units
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
      All_Jacobian (P, L, Base, X, Kept, R.all, J.all, No_Units, Seen.all, Sequential);
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
      X := [Ln_F, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P.Cx, P.Cy, Ada.Numerics.Long_Elementary_Functions.Log (P.Aspect)];
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
                        Here : constant Sighting := Sight (F, F * Exp (X (10)), X (8), X (9), P.K1, P.K2, View, I);
                        Ray  : constant Vec3 := Times (Pose.Rotation, Vec3'[Here.Xn, Here.Yn, 1.0]);
                     begin
                        for J in 1 .. Count loop
                           declare
                              T : Twist renames P.Twists (V, J);
                              S : constant Real := View.Sigma (I, J);
                              U : constant Real := T.C * Unit_Of (P, J);   --  the joint's unit
                           begin
                              if not T.Slide then
                                 --  e = Rate + B + Rho A, A = F g N R' (C w x (t - p)): linear in t.
                                 declare
                                    Row : Vec3;
                                    Fixed : Real;
                                    Bc : constant Vec3 :=
                                      Transposed_Times (Pose.Rotation, Scaled (U, Cross (T.W, Ray)));
                                    B  : constant Real := Dot (Here.Gn, Bc);
                                    Wp : constant Vec3 :=
                                      Transposed_Times (Pose.Rotation, Scaled (U, Cross (T.W, T.P)));
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
                                            Transposed_Times (Pose.Rotation, Scaled (U, Cross (T.W, E_Axis)));
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
            --  A centre the terms move no more than half the pixels at is no start: a state lowers its cost by losing
            --  them.
            Cost := (if 2 * Ok_Count > Sampled then Now else Real'Last);
            --  Done when a round lowers the cost by less than one residual's worth (Refine_Step).
            exit when Last - Now <= Real'Max (1.0, (if Freedom > 0 then Now / Real (Freedom) else 1.0));
            Last := Now;
         end;
      end loop;
   end Solve_Centre;

   ---------------------------------------------------------------------------
   --  The search

   --  The starts that must end at one optimum for it to be found (one could be chance, two are a reproduction), and
   --  the finalists of a race: twice them, so that two may agree and two disagree.
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

   --  A state's cost over the pixels Kept and its score: the cost over the freedom the pixels leave (a row for each
   --  joint that moves one, less one for its depth); Real'Last when the terms move no more than half of them (as a
   --  start of Solve_Centre's).
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
         B.X (8) - A.X (8), B.X (9) - A.X (9), B.X (10) - A.X (10)];
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
   --  the lengths from the half width seen at a right angle's half, half an octave apart, as the arm's own fit starts
   --  its, widened by a length on the side whose end scores best until a length inside scores at least as well as both
   --  ends (no field of view is assumed); every candidate's centre and depths by Solve_Centre. The best-ranked
   --  candidates, as many as there are turns, start the race: each takes a step of its refinement over the sample, the
   --  worse half drops out, the rest take twice as many, until the finalists are left; they run to their ends.
   --  Found_One when two of them ended at the best optimum and none ended at another that the pixels do not tell from
   --  it (Ambiguous); Reached: the finalists that ended at the best optimum.
   procedure Search_At
     (P : Problem; L : Layout; Sample : Flag_Array; Rotations : Positive; Arm_Spread, Pixel_Spread : Real;
      Middle : Vec3; Width : Positive; Best : out Racer; Verdict : out Search_Verdict; Reached : out Natural)
   is
      Half_Octave : constant Real := Ada.Numerics.Long_Elementary_Functions.Log (2.0) / 2.0;
      Start_Ln    : constant Real := Ada.Numerics.Long_Elementary_Functions.Log (Real (Width) / 2.0);
      --  The steps the scan may take either way: as far as a real's logarithm reaches, in half octaves.
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
   --  of the finer race does. The count stops doubling at the sampled pixels'
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

   --  The fit's terms: the logarithm of the focal length, the principal point and the logarithm of the aspect, the lens
   --  every arm is seen through, then each arm's turn about the camera's own axes and centre. An arm's own ten
   --  (Parameters) are these, taken out.
   Lens_Count : constant := 4;
   Arm_Count_Terms : constant := 6;

   function Global_Count (Arms : Positive) return Positive is (Lens_Count + Arm_Count_Terms * Arms);

   function Offset (A : Positive) return Natural is (Lens_Count + Arm_Count_Terms * (A - 1));

   function Local (X : Real_Array; A : Positive) return Parameters is
     ([X (X'First), X (X'First + Offset (A)), X (X'First + Offset (A) + 1), X (X'First + Offset (A) + 2),
       X (X'First + Offset (A) + 3), X (X'First + Offset (A) + 4), X (X'First + Offset (A) + 5),
       X (X'First + 1), X (X'First + 2), X (X'First + 3)]);

   --  The place in the fit's terms of an arm's own term T.
   function Global_Of (A : Positive; T : Positive) return Positive is
     (case T is
         when 1      => 1,
         when 8      => 2,
         when 9      => 3,
         when 10     => 4,
         when others => Offset (A) + T - 1);

   type Arm_State is record
      P      : Problem;
      L      : Layout_Access;
      Base   : Mat3 := Identity3;
      Kept   : Flag_Access;
      Searched : Search_Verdict := No_Start;
      Spread_Of_Arm : Real := 0.0;   --  its joints' places about their middle
      Score    : Real := Real'Last;   --  its search's best: the cost over the freedom its sampled pixels leave
      Unit_Sigma : Real_Access;   --  per joint, its unit's relative standard deviation (none: the units are exact)
      Units_At   : Index_Access;  --  per joint, its unit's logarithm's place in the fit's terms (nought: held)
   end record;

   type Arm_State_Array is array (Positive range <>) of Arm_State;

   --  Each arm's joints' units as the terms X have them.
   procedure Set_Units (S : Arm_State_Array; X : Real_Array) is
   begin
      for A in S'Range loop
         if S (A).Units_At /= null then
            for J in S (A).Units_At'Range loop
               if S (A).Units_At (J) > 0 then
                  S (A).P.Unit (J) := Exp (X (X'First + S (A).Units_At (J) - 1));
               end if;
            end loop;
         end if;
      end loop;
   end Set_Units;

   --  What the units' terms cost against the arms' fits (each unit's logarithm nought, its standard deviation the
   --  relative one the arm's fit states).
   function Prior_Cost (S : Arm_State_Array; X : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for A in S'Range loop
         if S (A).Units_At /= null then
            for J in S (A).Units_At'Range loop
               if S (A).Units_At (J) > 0 then
                  Sum := Sum + (X (X'First + S (A).Units_At (J) - 1) / S (A).Unit_Sigma (J)) ** 2;
               end if;
            end loop;
         end if;
      end loop;
      return Sum;
   end Prior_Cost;

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
      Set_Units (S, X);
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
      Cost := Cost + Prior_Cost (S, X);
   end Joint_Cost;

   --  One step of damped Gauss-Newton on every arm at once (Refine_Step's, the arms' normal equations summed into the
   --  fit's terms, the units' terms with their priors among them).
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
      Set_Units (S, X);
      Cost := 0.0;
      for A in S'Range loop
         Seen (A) := new Flag_Array (1 .. S (A).L.Pixels_All);
         Trial_Seen (A) := new Flag_Array (1 .. S (A).L.Pixels_All);
         declare
            L  : Layout renames S (A).L.all;
            R  : Real_Access := new Real_Array (1 .. L.Rows);
            J  : Matrix_Access := new Real_Matrix (1 .. L.Rows, 1 .. Terms);
            JU : Matrix_Access :=
              new Real_Matrix (1 .. L.Rows, 1 .. (if S (A).Units_At = null then 0 else S (A).P.Joints));
         begin
            All_Jacobian
              (S (A).P, L, S (A).Base, Local (X, A), S (A).Kept.all, R.all, J.all, JU.all, Seen (A).all, False);
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
               --  The units' columns, against the terms' and each other.
               for U in JU'Range (2) loop
                  if S (A).Units_At (U) > 0 then
                     declare
                        GU : constant Positive := S (A).Units_At (U);
                     begin
                        Jtr (GU) := Jtr (GU) + JU (Row, U) * R (Row);
                        for T in 1 .. Terms loop
                           JtJ (Global_Of (A, T), GU) := JtJ (Global_Of (A, T), GU) + J (Row, T) * JU (Row, U);
                           JtJ (GU, Global_Of (A, T)) := JtJ (GU, Global_Of (A, T)) + J (Row, T) * JU (Row, U);
                        end loop;
                        for U2 in JU'Range (2) loop
                           if S (A).Units_At (U2) > 0 then
                              JtJ (GU, S (A).Units_At (U2)) :=
                                JtJ (GU, S (A).Units_At (U2)) + JU (Row, U) * JU (Row, U2);
                           end if;
                        end loop;
                     end;
                  end if;
               end loop;
            end loop;
            Free (R);
            Free (J);
            Free (JU);
         end;
      end loop;
      --  The units' priors: each logarithm nought with the relative standard deviation its arm's fit states.
      for A in S'Range loop
         if S (A).Units_At /= null then
            for U in S (A).Units_At'Range loop
               if S (A).Units_At (U) > 0 then
                  declare
                     GU : constant Positive := S (A).Units_At (U);
                  begin
                     JtJ (GU, GU) := JtJ (GU, GU) + 1.0 / S (A).Unit_Sigma (U) ** 2;
                     Jtr (GU) := Jtr (GU) + X (X'First + GU - 1) / S (A).Unit_Sigma (U) ** 2;
                  end;
               end if;
            end loop;
         end if;
      end loop;
      Cost := Cost + Prior_Cost (S, X);
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

   --  A covariance widened to a spread in every direction the spread exceeds it, and kept in the others: with Lc Lc'
   --  the covariance, M = Lc^-1 Spread Lc^-T = V values V', W = V max (1, values) V', and the covariance Lc W Lc'.
   --  Widened: the most it was widened in any direction, raised to the largest value. Nothing when the covariance is
   --  not positive definite.
   procedure Widen_Covariance (Covariance : in out Real_Matrix; Spread : Real_Matrix; Widened : in out Real) is
      G   : constant Positive := Covariance'Length (1);
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
      Driver.Numerics.Dense.Cholesky (Covariance, Lc, Def);
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
                  Column (R) := Spread (Spread'First (1) + R - 1, Spread'First (2) + C - 1);
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
            Widened := Real'Max (Widened, V);
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
               Covariance (Covariance'First (1) + A - 1, Covariance'First (2) + B - 1) := 0.0;
               for K in 1 .. B loop
                  Covariance (Covariance'First (1) + A - 1, Covariance'First (2) + B - 1) :=
                    Covariance (Covariance'First (1) + A - 1, Covariance'First (2) + B - 1) + T (A, K) * Lc (B, K);
               end loop;
            end loop;
         end loop;
      end;
   end Widen_Covariance;

   procedure Fit_Eye
     (Arms          : Arm_Flow_Array;
      Width, Height : Positive;
      Result        : out Report)
   is
      Count : constant Positive := Arms'Length;
      G     : constant Positive := Global_Count (Count);
      --  The units' terms: one for every joint whose arm states its unit's deviation, after the lens's and the poses'.
      function Units_Stated return Natural is
         Sum : Natural := 0;
      begin
         for Arm of Arms loop
            if Arm.Unit_Sigma /= null then
               for Sigma of Arm.Unit_Sigma.all loop
                  if Sigma > 0.0 then
                     Sum := Sum + 1;
                  end if;
               end loop;
            end if;
         end loop;
         return Sum;
      end Units_Stated;
      G_Full : constant Positive := G + Units_Stated;
      S     : Arm_State_Array (1 .. Count);
      X     : Real_Array (1 .. G_Full) := [others => 0.0];
      Normal : Real_Matrix (1 .. G_Full, 1 .. G_Full);
      Next_Unit : Positive := G + 1;
      Scatter : Real := 1.0;
      Kept_All, Offered_All : Natural := 0;
      Best_Score : Real := Real'Last;

      procedure Release is
      begin
         for A in S'Range loop
            Free (S (A).L);
            Free (S (A).P.Twists);
            Free (S (A).Kept);
            Free (S (A).P.Unit);
            Free (S (A).Unit_Sigma);
            Free (S (A).Units_At);
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
                        Aspect => 1.0, Additive => 1.0, Relative => 0.0, Unit => null, K1 | K2 => 0.0);
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
            S (A).P.Unit := new Real_Array'(1 .. Natural'Max (1, Arm.Joints'Length) => 1.0);
            if Arm.Unit_Sigma /= null and then Arm.Unit_Sigma'Length = Arm.Joints'Length then
               S (A).Unit_Sigma := new Real_Array (1 .. Arm.Joints'Length);
               S (A).Unit_Sigma.all := Arm.Unit_Sigma.all;
               S (A).Units_At := new Index_Array'(1 .. Arm.Joints'Length => 0);
               for J in 1 .. Arm.Joints'Length loop
                  if Arm.Unit_Sigma (Arm.Unit_Sigma'First + J - 1) > 0.0 then
                     S (A).Units_At (J) := Next_Unit;
                     Next_Unit := Next_Unit + 1;
                  end if;
               end loop;
            end if;
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
               X (4) := Own (10);
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
            --  The residuals' own spread: the units' priors are no residuals of the pictures.
            Sum := Sum - Prior_Cost (S, X);
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
      Result.Lens := (Fx => Exp (X (1)), Fy => Exp (X (1) + X (4)), Cx => X (2), Cy => X (3), K1 | K2 => 0.0);
      for A in S'Range loop
         Result.Poses (A) := Pose_Of (S (A).Base, Local (X, A));
      end loop;
      --  The covariance of the lens's and the poses' terms: their block of the inverse of every term's normal
      --  equations, the units' uncertainty taken in.
      declare
         Lower    : Real_Matrix (1 .. G_Full, 1 .. G_Full);
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
               E      : Real_Vector (1 .. G_Full) := [others => 0.0];
               Column : Real_Vector (1 .. G_Full);
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

               --  The covariance widened to Spread in every direction Spread exceeds it (Widen_Covariance).
               procedure Widen (Spread : Real_Matrix) is
               begin
                  Widen_Covariance (Result.Covariance, Spread, Result.Widened);
               end Widen;

               --  The jackknife's spread: the blocks' shifts, each weighted, averaged over the blocks.
               function Spread return Real_Matrix is
                  Result_S : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
               begin
                  for K in 1 .. N_Out loop
                     for A in 1 .. G loop
                        for B in 1 .. G loop
                           Result_S (A, B) :=
                             Result_S (A, B) + Weight (K) * Theta (K) (A) * Theta (K) (B) / Real (N_Out);
                        end loop;
                     end loop;
                  end loop;
                  return Result_S;
               end Spread;

               --  The fit again without pose V of arm A; the rows it leaves out weigh its shift.
               procedure Leave_Out (A, V : Positive) is
                  S_V      : Arm_State_Array := S;
                  X_V      : Real_Array := X;
                  Normal_V : Real_Matrix (1 .. G_Full, 1 .. G_Full);
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
                     Theta (N_Out) (4) := X_V (4) - X (4);
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
               Set_Units (S, X);   --  the units as the fit has them again, after the poses' refits
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
                              Here  : constant Sighting :=
                                Sight (F, F * Exp (XA (10)), XA (8), XA (9), S (A).P.K1, S (A).P.K2, View, I);
                              Point : constant Vec3 :=
                                Plus (Pose.Translation,
                                      Scaled (1.0 / Rho, Times (Pose.Rotation, Vec3'[Here.Xn, Here.Yn, 1.0])));
                              Ju, Jv : Real_Vector (1 .. G) := [others => 0.0];
                           begin
                              for T in 1 .. Terms loop
                                 declare
                                    H : constant Real :=
                                      (if T <= 4 or else T = 10 then Eps3
                                       else Eps3 * Real'Max (S (A).Spread_Of_Arm, abs XA (T)));
                                    function Project (Y : Parameters) return Vec3 is
                                       Pp : constant Rigid := Pose_Of (S (A).Base, Y);
                                       C  : constant Vec3 :=
                                         Transposed_Times (Pp.Rotation, Minus (Point, Pp.Translation));
                                       Fl : constant Real := Exp (Y (1));
                                    begin
                                       return [Fl * C (1) / C (3) + Y (8), Fl * Exp (Y (10)) * C (2) / C (3) + Y (9), 0.0];
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
         Result.Why := To_Unbounded_String
           ("no two of an arm's search's starts ended at its best eye, nor did two races");
      elsif Result.Spread * Driver.Conventions.Z >= Real (Driver.Alignment.Half_Patch (Width, Height)) then
         Result.Why := To_Unbounded_String
           ("the pixels leave the arms' points in the picture uncertain by" & Driver.Log.Image (Result.Spread, 2)
            & " px");
      else
         Result.Determined := True;
      end if;
      Release;
   end Fit_Eye;

   ---------------------------------------------------------------------------
   --  The fit again from the world the arms' own eyes saw

   procedure Refine_By_Scene
     (Arms          : Arm_Flow_Array;
      Worlds        : Arm_World_Array;
      Fixed         : Real_Array;
      Width, Height : Positive;
      Result        : in out Report)
   is
      Count  : constant Positive := Arms'Length;
      G_F    : constant Positive := Terms_For (Count);   --  the flows' terms, Fit_Eye's report's
      --  The scene's terms: the flows', and after them the lens's two of radial distortion, which the flows (a
      --  pinhole's) do not tell and the places across the picture do: a real lens bends its picture's rim by pixels,
      --  and held straight the bend went into the focal length and the eye's distance (the fakes' lenses, 10-21).
      G      : constant Positive := G_F + 2;
      Lens0  : constant Fit.Lens := Result.Lens;
      Poses0 : constant Rigid_Array := Result.Poses;
      Eps3   : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);
      Fixed_Pyramid : constant Driver.Alignment.Pyramid := Driver.Alignment.Pyramid_Of (Width, Height, Fixed);

      --  A point of the world an arm's eye showed: the arm, its picture, the pixel there, the point the pixel's line of
      --  sight meets the arm's table at, and the points its eight neighbours' lines of sight meet it at (Around (I, J)
      --  the pixel I across and J down from it, Around (0, 0) the point itself), for the warp to second order.
      type Neighbours is array (-1 .. 1, -1 .. 1) of Vec3;
      type Sight is record
         Arm, Picture : Positive := 1;
         U0, V0       : Real := 0.0;
         X            : Vec3 := [0.0, 0.0, 0.0];
         Around       : Neighbours := [others => [others => [0.0, 0.0, 0.0]]];
      end record;
      type Sight_Array is array (Positive range <>) of Sight;
      type Sight_Access is access Sight_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Sight_Array, Sight_Access);

      --  A place the fixed eye's picture showed a point at, and its standard deviations across and down; the pixel of
      --  its arm's eye's picture it is the point of (U0, V0).
      type Match is record
         Arm    : Positive := 1;
         Picture : Positive := 1;
         U0, V0 : Real := 0.0;
         X      : Vec3 := [0.0, 0.0, 0.0];
         U, V   : Real := 0.0;
         Su, Sv : Real := 0.0;
      end record;
      type Match_Array is array (Positive range <>) of Match;
      type Match_Access is access Match_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Match_Array, Match_Access);

      type Pyramid_Array is array (Positive range <>) of Driver.Alignment.Pyramid;
      type Pyramid_Access is access Pyramid_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Pyramid_Array, Pyramid_Access);

      --  The arms' joints at every pose of their flows, in their arms' reference frames: where the verdict weighs the
      --  eye, as Fit_Eye weighs it at the arms' points.
      type Joint_Point is record
         Arm : Positive := 1;
         X   : Vec3 := [0.0, 0.0, 0.0];
      end record;
      type Joint_Point_Array is array (Positive range <>) of Joint_Point;
      type Joint_Point_Access is access Joint_Point_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Joint_Point_Array, Joint_Point_Access);

      function Pictures_Of (A : Positive) return Natural is
        (if Worlds (Worlds'First + A - 1).Pictures = null then 0
         else Worlds (Worlds'First + A - 1).Pictures'Length);

      --  The terms are the fit's (Report.Covariance's order), as changes from the flow's: the focal length's logarithm,
      --  the principal point, the aspect's logarithm, and each arm's turn about the camera's own axes and centre.
      function Lens_Of (Z : Real_Vector) return Fit.Lens is
        ((Fx => Lens0.Fx * Exp (Z (1)), Fy => Lens0.Fy * Exp (Z (1) + Z (4)), Cx => Lens0.Cx + Z (2),
          Cy => Lens0.Cy + Z (3), K1 => Lens0.K1 + Z (G_F + 1), K2 => Lens0.K2 + Z (G_F + 2)));

      function Pose_Of (Z : Real_Vector; A : Positive) return Rigid is
        ((Rotation    => Times (Poses0 (A).Rotation,
                                Driver.Numerics.Exp
                                  (Vec3'[Z (Offset (A) + 1), Z (Offset (A) + 2), Z (Offset (A) + 3)])),
          Translation => [Poses0 (A).Translation (1) + Z (Offset (A) + 4),
                          Poses0 (A).Translation (2) + Z (Offset (A) + 5),
                          Poses0 (A).Translation (3) + Z (Offset (A) + 6)]));

      --  Where the fixed eye of those terms puts a point of arm A's frame, when in front of it, inside the lens's fold
      --  and in its picture. Past where the lens's map of the radius turns back (its slope 1 + 3 K1 r^2 + 5 K2 r^4 no
      --  longer positive) the map folds a point back inside the picture that no lens shows there.
      function Seen (Z : Real_Vector; A : Positive; X : Vec3; U, V : out Real) return Boolean is
         P : constant Rigid := Pose_Of (Z, A);
         C : constant Vec3 := Transposed_Times (P.Rotation, Minus (X, P.Translation));
         L : constant Fit.Lens := Lens_Of (Z);
         In_Front : Boolean;
      begin
         Fit.Project (L, C, U, V, In_Front);
         if In_Front then
            declare
               R2 : constant Real := (C (1) ** 2 + C (2) ** 2) / C (3) ** 2;
            begin
               In_Front := 1.0 + 3.0 * L.K1 * R2 + 5.0 * L.K2 * R2 * R2 > 0.0;
            end;
         end if;
         return In_Front and then U >= 0.0 and then U <= Real (Width) and then V >= 0.0 and then V <= Real (Height);
      end Seen;

      --  Where the fixed eye of those terms puts a point of arm A's frame on its picture's plane, in front of it or
      --  not: a misfit that vanished off the picture let the places' fit run to an eye that saw none of them (the
      --  fake's seed 22, 10-21: a focal length of 39 px, every misfit nought).
      procedure Place (Z : Real_Vector; A : Positive; X : Vec3; U, V : out Real) is
         P : constant Rigid := Pose_Of (Z, A);
         C : constant Vec3 := Transposed_Times (P.Rotation, Minus (X, P.Translation));
         L : constant Fit.Lens := Lens_Of (Z);
         --  The lens's own map (Fit.Project's) without its test of the point's side.
         Xn : constant Real := C (1) / C (3);
         Yn : constant Real := C (2) / C (3);
         R2 : constant Real := Xn * Xn + Yn * Yn;
         D  : constant Real := 1.0 + L.K1 * R2 + L.K2 * R2 * R2;
      begin
         U := L.Cx + L.Fx * Xn * D;
         V := L.Cy + L.Fy * Yn * D;
      end Place;

      --  The point of arm A's table a pixel of one of its eye's pictures sees: its line of sight from the eye's centre
      --  to the plane, in front of the eye.
      function On_Table (W : Wrist_Picture; L : Fit.Lens; T : Driver.Geometry.Plane_Estimate; U, V : Real;
                         X : out Vec3) return Boolean
      is
         D     : constant Vec3 := Times (W.Eye.Rotation, Fit.Ray (L, U, V));
         Along : constant Real := Dot (D, T.Normal);
      begin
         X := W.Eye.Translation;
         if Along = 0.0 then
            return False;
         end if;
         declare
            S : constant Real := Dot (Minus (T.Centre, W.Eye.Translation), T.Normal) / Along;
         begin
            X := Plus (W.Eye.Translation, Scaled (S, D));
            return S > 0.0;
         end;
      end On_Table;

      --  The picture shows the world at the place, by its mask.
      function World_At (W : Wrist_Picture; U, V : Real) return Boolean is
         X : constant Integer := Integer (Real'Floor (U));
         Y : constant Integer := Integer (Real'Floor (V));
      begin
         return W.World = null
           or else (X in 0 .. W.Width - 1 and then Y in 0 .. W.Height - 1 and then W.World (Y * W.Width + X + 1));
      end World_At;

      Total_Pictures : Natural := 0;
      Sights   : Sight_Access;
      Pyramids : Pyramid_Access;
      Joints   : Joint_Point_Access;
      --  The arms' flows as Fit_Eye measured them, the evidence beside the places: each arm's problem (its poses, its
      --  joints' twists at each, the rates' spread Fit_Eye found, never under the pictures' own sigma), the pixels two
      --  joints or more move, and those kept.
      type Problem_Array is array (Positive range <>) of Problem;
      type Layout_Array is array (Positive range <>) of Layout_Access;
      type Kept_Array is array (Positive range <>) of Flag_Access;
      Flows     : Problem_Array (1 .. Count);
      Layouts   : Layout_Array (1 .. Count);
      Flow_Kept : Kept_Array (1 .. Count);
      Flow_Pixels : Natural := 0;
      --  What the arms' flows tell of the eye, as Fit_Eye's own covariance holds it (widened by its jackknife over the
      --  poses): the yardstick the places' information is weighed by, direction by direction (Split). Where the places
      --  tell a direction less than the flows do (a table's plane leaves one open: the focal length against the
      --  distance), the flows' residuals place the eye along it, the places' terms held; every other direction is the
      --  places' alone. The flows' own estimate, every term of it off by their outlines' bias, held the open direction
      --  where it stood when given as a prior beside the places, and was called told 4 sigmas off (the fake's seeds 20
      --  and 24 at turns of 0.004, 10-21); at their own weight beside the places their bias in an arm's pose (its
      --  centre 30 mm off) took the eye with it; along the open direction alone, at the places' other terms, the flows
      --  put the eye within 0.25 % of the focal length on all five seeds.
      Honest_Lower : Real_Matrix (1 .. G_F, 1 .. G_F) := [others => [others => 0.0]];   --  of Fit_Eye's covariance
      Honest_Known : Boolean := False;
      --  The lens's bend, which the flows' own estimate (Fit_Eye's, a pinhole's) has no term for and their covariance
      --  no width: each of its two terms' prior information (one over its variance), as wide as the lens can bend
      --  before it folds its picture back at the farthest corner (its map of the radius, r (1 + K1 r^2 + K2 r^4), stops
      --  increasing there at K1 = -1 / (3 r^2) alone, at K2 = -1 / (5 r^4) alone) over Z, about a lens that bends
      --  nought; taken beside the places' information, whose terms the bend's are.
      Bend_Prior : Real_Vector (1 .. 2) := [0.0, 0.0];

      --  The bend's terms at Z, and what its prior charges for them.
      function Bend_Of (Z : Real_Vector; K : Positive) return Real is
        (if K = 1 then Lens0.K1 + Z (G_F + 1) else Lens0.K2 + Z (G_F + 2));
      function Bend_Cost (Z : Real_Vector) return Real is
        (Bend_Prior (1) * Bend_Of (Z, 1) ** 2 + Bend_Prior (2) * Bend_Of (Z, 2) ** 2);
      --  Some direction of the flows' has no least of their misfit (Line_Search): the eye is not told.
      Flows_Untold : Boolean := False;
      Began        : constant Duration := Driver.Clock.Seconds;   --  for the log: how long the rounds take
      Aligned_At, Fitted_At : Duration := 0.0;   --  and when the round's places were found, and fitted

      --  Arm A's flow residuals at the terms Z, every pixel's depth eliminated (nought where a pixel is left out), with
      --  the joints' turns Twists in place of the arm's own when given (a variant's, Arm_World.Variants).
      procedure Flow_Residuals (Z : Real_Vector; A : Positive; R : out Real_Array; Twists : Twist_Grid_Access := null)
      is
         L    : constant Fit.Lens := Lens_Of (Z);
         Pz   : constant Rigid := Pose_Of (Z, A);
         P    : Problem := Flows (A);
         X    : constant Parameters :=
           [Ada.Numerics.Long_Elementary_Functions.Log (L.Fx), 0.0, 0.0, 0.0,
            Pz.Translation (1), Pz.Translation (2), Pz.Translation (3), L.Cx, L.Cy,
            Ada.Numerics.Long_Elementary_Functions.Log (L.Fy / L.Fx)];
         Seen : Flag_Access := new Flag_Array (1 .. Layouts (A).Pixels_All);
      begin
         P.Cx := L.Cx;
         P.Cy := L.Cy;
         P.K1 := L.K1;
         P.K2 := L.K2;
         if Twists /= null then
            P.Twists := Twists;
         end if;
         All_Residuals (P, Layouts (A).all, Pz.Rotation, X, Flow_Kept (A).all, R, Seen.all);
         Free (Seen);
      end Flow_Residuals;

      --  The flows' misfits squared at Z, of every pose but pose Skip_Pose of arm Skip_Arm (none left out by default).
      function Flow_Cost (Z : Real_Vector; Skip_Arm, Skip_Pose : Natural := 0) return Real is
         Sum : Real := 0.0;
      begin
         for A in 1 .. Count loop
            declare
               L : Layout renames Layouts (A).all;
               R : Real_Access := new Real_Array (1 .. L.Rows);
            begin
               Flow_Residuals (Z, A, R.all);
               for Q in 1 .. L.Pixels_All loop
                  if A /= Skip_Arm or else L.Pose_Of_Pixel (Q) /= Skip_Pose then
                     for K in 0 .. L.Count (Q) - 1 loop
                        Sum := Sum + R (L.First_Row (Q) + K) ** 2;
                     end loop;
                  end if;
               end loop;
               Free (R);
            end;
         end loop;
         return Sum;
      end Flow_Cost;

      --  The flows' misfits squared at Z pose by pose: Into (Pose_Slot (A, V)) the sum over pose V of arm A's pixels,
      --  the arms' poses one after another.
      function Pose_Slot (A, V : Natural) return Natural is
         Before : Natural := 0;
      begin
         for B in 1 .. A - 1 loop
            Before := Before + Arms (Arms'First + B - 1).Poses'Length;
         end loop;
         return Before + V - Arms (Arms'First + A - 1).Poses'First + 1;
      end Pose_Slot;

      Pose_Slots : constant Natural := Pose_Slot (Count, Arms (Arms'Last).Poses'Last);

      procedure Pose_Costs (Z : Real_Vector; Into : out Real_Array) is
      begin
         Into := [others => 0.0];
         for A in 1 .. Count loop
            declare
               L : Layout renames Layouts (A).all;
               R : Real_Access := new Real_Array (1 .. L.Rows);
            begin
               Flow_Residuals (Z, A, R.all);
               for Q in 1 .. L.Pixels_All loop
                  declare
                     S : constant Positive := Pose_Slot (A, L.Pose_Of_Pixel (Q));
                  begin
                     for K in 0 .. L.Count (Q) - 1 loop
                        Into (S) := Into (S) + R (L.First_Row (Q) + K) ** 2;
                     end loop;
                  end;
               end loop;
               Free (R);
            end;
         end loop;
      end Pose_Costs;

      --  The places' information (Ata, the normal matrix of their misfits with the bend's prior) weighed against the
      --  flows': the directions Dirs (a column each) and Lambda. The flows' estimate has no bend: in the flows' terms,
      --  the directions where S Dirs = Lambda Honest^-1 Dirs, S the places' information with the bend eliminated
      --  (A_FF - A_FK A_KK^-1 A_KF), each of one sigma by the flows, the bend along each as the places put it at those
      --  terms (-A_KK^-1 A_KF); Lambda under one: the places tell the direction less than the flows. Then the bend's
      --  own two at the flows' terms held, each of one sigma by the places, Lambda one: the places'. With the bend's
      --  prior taken for the flows' information instead, the bend went with the flows' directions where the places told
      --  it less than the prior: the flows' misfit, then read a round late and so with no bend in it, moved it as the
      --  directions' other terms moved it, and their jackknife left it out of the covariance (the fake's seed 27 told
      --  4.8 sigmas off along its arms' poses, its bend -0.014 for -0.044; robot.flow.scene's rounds fitted in 150 and
      --  470 s, its eye not settled, 10-21).
      procedure Split (Ata : Real_Matrix; Dirs : out Real_Matrix; Lambda : out Real_Vector) is
         B, U, C  : Real_Matrix (1 .. G_F, 1 .. G_F);
         S        : Real_Matrix (1 .. G_F, 1 .. G_F);   --  the places' information on the flows' terms, the bend free
         Akk      : Real_Matrix (1 .. 2, 1 .. 2);
         Cond     : Real_Matrix (1 .. 2, 1 .. G_F);    --  the bend the places put at a change of the flows' terms
         Lower_K  : Real_Matrix (1 .. 2, 1 .. 2);      --  of A_KK^-1
         Lambda_F : Real_Vector (1 .. G_F);
         Ok       : Boolean;
      begin
         for P in 1 .. 2 loop
            for Q in 1 .. 2 loop
               Akk (P, Q) := Ata (G_F + P, G_F + Q);
            end loop;
         end loop;
         declare
            Akk_Inv : constant Real_Matrix := Inverse (Akk);
         begin
            for K in 1 .. 2 loop
               for Q in 1 .. G_F loop
                  Cond (K, Q) := -(Akk_Inv (1, K) * Ata (G_F + 1, Q) + Akk_Inv (2, K) * Ata (G_F + 2, Q));
               end loop;
            end loop;
            Driver.Numerics.Dense.Cholesky (Akk_Inv, Lower_K, Ok);
         end;
         for P in 1 .. G_F loop
            for Q in 1 .. G_F loop
               S (P, Q) := Ata (P, Q) + Ata (P, G_F + 1) * Cond (1, Q) + Ata (P, G_F + 2) * Cond (2, Q);
            end loop;
         end loop;
         for P in 1 .. G_F loop
            for Q in P + 1 .. G_F loop
               S (P, Q) := (S (P, Q) + S (Q, P)) / 2.0;
               S (Q, P) := S (P, Q);
            end loop;
         end loop;
         for M in 1 .. G_F loop
            for Q in 1 .. G_F loop
               declare
                  Sum : Real := 0.0;
               begin
                  for K in 1 .. G_F loop
                     Sum := Sum + S (M, K) * Honest_Lower (K, Q);
                  end loop;
                  C (M, Q) := Sum;
               end;
            end loop;
         end loop;
         for P in 1 .. G_F loop
            for Q in 1 .. G_F loop
               declare
                  Sum : Real := 0.0;
               begin
                  for M in 1 .. G_F loop
                     Sum := Sum + Honest_Lower (M, P) * C (M, Q);
                  end loop;
                  B (P, Q) := Sum;
               end;
            end loop;
         end loop;
         Driver.Numerics.Dense.Small_Eigensystem (B, Lambda_F, U);
         Dirs := [others => [others => 0.0]];
         for I in 1 .. G_F loop
            Lambda (I) := Lambda_F (I);
            for P in 1 .. G_F loop
               declare
                  Sum : Real := 0.0;
               begin
                  for K in 1 .. G_F loop
                     Sum := Sum + Honest_Lower (P, K) * U (K, I);
                  end loop;
                  Dirs (P, I) := Sum;
               end;
            end loop;
            for K in 1 .. 2 loop
               declare
                  Sum : Real := 0.0;
               begin
                  for Q in 1 .. G_F loop
                     Sum := Sum + Cond (K, Q) * Dirs (Q, I);
                  end loop;
                  Dirs (G_F + K, I) := Sum;
               end;
            end loop;
         end loop;
         for K in 1 .. 2 loop
            Lambda (G_F + K) := 1.0;
            for P in 1 .. 2 loop
               Dirs (G_F + P, G_F + K) := Lower_K (P, K);
            end loop;
         end loop;
      end Split;

      --  Where along direction V from Z the flows' misfit (every pose but Skip_Pose of Skip_Arm) is least, in V's units
      --  (the flows' sigma along it): from nought by steps either way that double until the misfit rises, then by
      --  Brent's method within that bracket (a parabola through the three least misfits found, a golden section where
      --  the parabola's least would not shrink the bracket enough) until the least is within half Unchanged_Fraction of
      --  a sigma. Golden sections alone took sixteen misfits a search or more, most of the refinement's time.
      --  A misfit that falls as far as the float goes (or turns to no number) has no least there: Real'Last, and the
      --  direction is one the flows do not tell (Flows_Untold); the synthetic arms of robot.flow.scene first ran so,
      --  their eye's distance and focal length growing together without end.
      function Line_Search (Z, V : Real_Vector; Skip_Arm, Skip_Pose : Natural := 0) return Real is
         function F (S : Real) return Real is
            X : Real_Vector (1 .. G);
         begin
            for T in 1 .. G loop
               X (T) := Z (T) + S * V (T);
            end loop;
            return Flow_Cost (X, Skip_Arm, Skip_Pose);
         end F;
         Golden : constant Real := (3.0 - Sqrt (5.0)) / 2.0;   --  the golden section's smaller share
         F0     : constant Real := F (0.0);
         Lo, Hi : Real := 0.0;
         --  The least misfit found (X), the next (W) and the one before it (Vv), as Brent's method keeps them.
         X, W, Vv    : Real := 0.0;
         Fx, Fw, Fv  : Real := F0;
      begin
         declare
            Fp : constant Real := F (1.0);
            Fm : constant Real := F (-1.0);
         begin
            if Fp >= F0 and then Fm >= F0 then
               Lo := -1.0;
               Hi := 1.0;
               W := (if Fp <= Fm then 1.0 else -1.0);
               Fw := Real'Min (Fp, Fm);
               Vv := -W;
               Fv := Real'Max (Fp, Fm);
            else
               declare
                  Sign   : constant Real := (if Fp < Fm then 1.0 else -1.0);
                  Before : Real := 0.0;
                  F_Before : Real := F0;
                  Here   : Real := 1.0;
                  F_Here : Real := Real'Min (Fp, Fm);
                  Step   : Real := 1.0;
                  Next   : Real := 2.0;
                  F_Next : Real := F_Here;
               begin
                  for Doubling in 1 .. Real'Machine_Mantissa loop
                     Step := 2.0 * Step;
                     Next := Here + Step;
                     F_Next := F (Sign * Next);
                     exit when not (F_Next < F_Here);
                     if Doubling = Real'Machine_Mantissa then
                        Flows_Untold := True;
                        return Real'Last;
                     end if;
                     Before := Here;
                     F_Before := F_Here;
                     Here := Next;
                     F_Here := F_Next;
                  end loop;
                  Lo := Real'Min (Sign * Before, Sign * Next);
                  Hi := Real'Max (Sign * Before, Sign * Next);
                  X := Sign * Here;
                  Fx := F_Here;
                  W := (if F_Before <= F_Next then Sign * Before else Sign * Next);
                  Fw := Real'Min (F_Before, F_Next);
                  Vv := (if F_Before <= F_Next then Sign * Next else Sign * Before);
                  Fv := Real'Max (F_Before, F_Next);
               end;
            end if;
         end;
         declare
            A   : Real := Lo;
            B   : Real := Hi;
            --  the step before last (E) and the last (D); E starts as the bracket, so that the first step may be the
            --  parabola's through the three misfits the bracketing found
            E   : Real := Hi - Lo;
            D   : Real := 0.0;
            Tol : constant Real := Driver.Conventions.Unchanged_Fraction / 4.0;
         begin
            loop
               declare
                  M         : constant Real := (A + B) / 2.0;
                  U         : Real;
                  Parabolic : Boolean := False;
               begin
                  --  The least lies in the bracket and the bracket within 2 Tol of X either way.
                  exit when abs (X - M) <= 2.0 * Tol - (B - A) / 2.0;
                  if abs E > Tol then
                     declare
                        R0   : constant Real := (X - W) * (Fx - Fv);
                        Q0   : constant Real := (X - Vv) * (Fx - Fw);
                        P    : Real := (X - Vv) * Q0 - (X - W) * R0;
                        Q    : Real := 2.0 * (Q0 - R0);
                        Last : constant Real := E;
                     begin
                        if Q > 0.0 then
                           P := -P;
                        else
                           Q := -Q;
                        end if;
                        if abs P < abs (Q * Last / 2.0) and then P > Q * (A - X) and then P < Q * (B - X) then
                           E := D;
                           D := P / Q;
                           Parabolic := True;
                           if X + D - A < 2.0 * Tol or else B - (X + D) < 2.0 * Tol then
                              D := (if M >= X then Tol else -Tol);
                           end if;
                        end if;
                     end;
                  end if;
                  if not Parabolic then
                     E := (if X >= M then A - X else B - X);
                     D := Golden * E;
                  end if;
                  U := (if abs D >= Tol then X + D elsif D >= 0.0 then X + Tol else X - Tol);
                  declare
                     Fu : constant Real := F (U);
                  begin
                     if Fu <= Fx then
                        if U >= X then
                           A := X;
                        else
                           B := X;
                        end if;
                        Vv := W;
                        Fv := Fw;
                        W := X;
                        Fw := Fx;
                        X := U;
                        Fx := Fu;
                     else
                        if U < X then
                           A := U;
                        else
                           B := U;
                        end if;
                        if Fu <= Fw or else W = X then
                           Vv := W;
                           Fv := Fw;
                           W := U;
                           Fw := Fu;
                        elsif Fu <= Fv or else Vv = X or else Vv = W then
                           Vv := U;
                           Fv := Fu;
                        end if;
                     end if;
                  end;
               end;
            end loop;
            return X;
         end;
      end Line_Search;

      --  The step the places tell, in their own directions (Lambda one or more): each direction's share of Atb over its
      --  Lambda.
      function Places_Step (Dirs : Real_Matrix; Lambda, Atb : Real_Vector) return Real_Vector is
         D : Real_Vector (1 .. G) := [others => 0.0];
      begin
         for I in 1 .. G loop
            if Lambda (I) >= 1.0 then
               declare
                  Share : Real := 0.0;
               begin
                  for P in 1 .. G loop
                     Share := Share + Dirs (P, I) * Atb (P);
                  end loop;
                  for P in 1 .. G loop
                     D (P) := D (P) + Dirs (P, I) * Share / Lambda (I);
                  end loop;
               end;
            end if;
         end loop;
         return D;
      end Places_Step;

      --  The pictures of the arms before arm A, as the pyramids and the matches number every arm's one after another.
      function Pictures_Before (A : Positive) return Natural is
         Sum : Natural := 0;
      begin
         for B in 1 .. A - 1 loop
            Sum := Sum + Pictures_Of (B);
         end loop;
         return Sum;
      end Pictures_Before;

      --  The joints' points of the arms before arm A, as Joints holds every arm's (each pose's, joint after joint).
      function Joints_Before (A : Positive) return Natural is
         Sum : Natural := 0;
      begin
         for B in 1 .. A - 1 loop
            Sum := Sum + Arms (Arms'First + B - 1).Poses'Length * Arms (Arms'First + B - 1).Joints'Length;
         end loop;
         return Sum;
      end Joints_Before;

      --  Whether arm A's variants (Arm_World.Variants) are of its pictures and its joints: an eye's frame for every
      --  picture and as many joints as its flow's, in pairs.
      function Variants_Fit (A : Positive) return Boolean is
         World : Arm_World renames Worlds (Worlds'First + A - 1);
      begin
         return World.Variants /= null and then World.Variants'Length mod 2 = 0
           and then (for all V of World.Variants.all =>
                       V.Joints /= null and then V.Eyes /= null
                       and then V.Joints'Length = Arms (Arms'First + A - 1).Joints'Length
                       and then V.Eyes'Length = Pictures_Of (A));
      end Variants_Fit;

      --  The arms' variants' pairs, every arm's one after another.
      function Variant_Pairs return Natural is
         Sum : Natural := 0;
      begin
         for A in 1 .. Count loop
            if Variants_Fit (A) then
               Sum := Sum + Worlds (Worlds'First + A - 1).Variants'Length / 2;
            end if;
         end loop;
         return Sum;
      end Variant_Pairs;
   begin
      for A in 1 .. Count loop
         Total_Pictures := Total_Pictures + Pictures_Of (A);
      end loop;
      --  The flows' problems, from Fit_Eye's evidence of every arm (its estimate only the start, its covariance no
      --  evidence: given as a prior beside the places, the flows' own estimate, its other terms off by their outlines'
      --  bias, held the open direction where it stood and was called told 4 sigmas off, the fake's seeds 20 and 24 at
      --  turns of 0.004, 10-21; along that direction alone, at the places' other terms, the flows put the eye within
      --  0.25 % of the focal length on all five seeds).
      for A in 1 .. Count loop
         declare
            F : Arm_Flow renames Arms (Arms'First + A - 1);
         begin
            Flows (A) := (Poses => F.Poses,
                          Twists => new Twist_Grid (F.Poses'Range, 1 .. Natural'Max (1, F.Joints'Length)),
                          Width => Width, Height => Height, Joints => F.Joints'Length, Cx => Lens0.Cx, Cy => Lens0.Cy,
                          Aspect => Lens0.Fy / Lens0.Fx, Additive => Real'Max (1.0, Result.Additive),
                          Relative => Result.Relative, Unit => null, K1 => Lens0.K1, K2 => Lens0.K2);
            for V in F.Poses'Range loop
               declare
                  T : constant Twist_Array := Twists_At (F.Joints.all, F.Poses (V).Readings);
               begin
                  for J in T'Range loop
                     Flows (A).Twists (V, J) := T (J);
                  end loop;
               end;
            end loop;
            Layouts (A) := Lay_Out (Flows (A));
            Flow_Kept (A) := new Flag_Array'(1 .. Layouts (A).Pixels_All => True);
            Flow_Pixels := Flow_Pixels + Layouts (A).Pixels_All;
         end;
      end loop;
      --  The flows' covariance over their terms, and the bend's prior at the farthest corner (Bend_Prior).
      declare
         Corner    : constant Real :=
           Sqrt (Real'Max (Lens0.Cx, Real (Width) - Lens0.Cx) ** 2 / Lens0.Fx ** 2
                 + Real'Max (Lens0.Cy, Real (Height) - Lens0.Cy) ** 2 / Lens0.Fy ** 2);
         Flows_Cov : Real_Matrix (1 .. G_F, 1 .. G_F);
      begin
         for P in 1 .. G_F loop
            for Q in 1 .. G_F loop
               Flows_Cov (P, Q) := Result.Covariance (P, Q);
            end loop;
         end loop;
         Driver.Numerics.Dense.Cholesky (Flows_Cov, Honest_Lower, Honest_Known);
         Bend_Prior := [(3.0 * Corner ** 2 * Driver.Conventions.Z) ** 2,
                        (5.0 * Corner ** 4 * Driver.Conventions.Z) ** 2];
      end;
      if not Honest_Known then
         Driver.Log.Line (Driver.Log.Robot, "flow eye: the flows' covariance is not definite, nothing to weigh the"
                          & " world's points against");
         for A in 1 .. Count loop
            Free (Flows (A).Twists);
            Free (Layouts (A));
            Free (Flow_Kept (A));
         end loop;
         return;
      end if;
      --  Every arm's eye's pictures, and the points of its table on a grid a patch apart in each.
      Pyramids := new Pyramid_Array (1 .. Natural'Max (1, Total_Pictures));
      declare
         Found  : Natural := 0;
         Upper  : Natural := 0;
         Placed : Natural := 0;
      begin
         for A in 1 .. Count loop
            declare
               World : Arm_World renames Worlds (Worlds'First + A - 1);
            begin
               if Driver.Geometry.Known (World.Table) then
                  for P in 1 .. Pictures_Of (A) loop
                     declare
                        W    : Wrist_Picture renames World.Pictures (World.Pictures'First + P - 1);
                        Grid : constant Positive :=
                          2 * Positive'Max (1, Driver.Alignment.Half_Patch (W.Width, W.Height));
                     begin
                        Upper := Upper + (W.Width / Grid + 1) * (W.Height / Grid + 1);
                     end;
                  end loop;
               end if;
            end;
         end loop;
         Sights := new Sight_Array (1 .. Natural'Max (1, Upper));
         for A in 1 .. Count loop
            declare
               World : Arm_World renames Worlds (Worlds'First + A - 1);
            begin
               for P in 1 .. Pictures_Of (A) loop
                  declare
                     W    : Wrist_Picture renames World.Pictures (World.Pictures'First + P - 1);
                     Grid : constant Positive := 2 * Positive'Max (1, Driver.Alignment.Half_Patch (W.Width, W.Height));
                     Gu   : Natural := Grid / 2;
                  begin
                     Placed := Placed + 1;
                     Pyramids (Placed) := Driver.Alignment.Pyramid_Of (W.Width, W.Height, W.Luma.all);
                     if Driver.Geometry.Known (World.Table) then
                        while Gu < W.Width - Grid / 2 loop
                           declare
                              Gv : Natural := Grid / 2;
                           begin
                              while Gv < W.Height - Grid / 2 loop
                                 declare
                                    U0 : constant Real := Real (Gu) + 0.5;
                                    V0 : constant Real := Real (Gv) + 0.5;
                                    Around : Neighbours;
                                    All_On : Boolean := True;
                                 begin
                                    for I in -1 .. 1 loop
                                       for J in -1 .. 1 loop
                                          All_On := All_On
                                            and then World_At (W, U0 + Real (I), V0 + Real (J))
                                            and then On_Table (W, World.Lens, World.Table, U0 + Real (I), V0 + Real (J),
                                                               Around (I, J));
                                       end loop;
                                    end loop;
                                    if All_On then
                                       Found := Found + 1;
                                       Sights (Found) := (Arm => A, Picture => Placed, U0 => U0, V0 => V0,
                                                          X => Around (0, 0), Around => Around);
                                    end if;
                                 end;
                                 Gv := Gv + Grid;
                              end loop;
                           end;
                           Gu := Gu + Grid;
                        end loop;
                     end if;
                  end;
               end loop;
            end;
         end loop;
         --  Shrunk to the points found.
         declare
            Kept : constant Sight_Access := new Sight_Array'(Sights (1 .. Found));
         begin
            Free (Sights);
            Sights := Kept;
         end;
      end;
      if Sights'Length = 0 then
         Driver.Log.Line (Driver.Log.Robot, "flow eye: no arm's eye saw its table, nothing of the world to place by");
         Free (Sights);
         Free (Pyramids);
         return;
      end if;
      --  The arms' joints at every pose of their flows.
      declare
         Total : Natural := 0;
      begin
         for A in 1 .. Count loop
            Total := Total + Arms (Arms'First + A - 1).Poses'Length * Arms (Arms'First + A - 1).Joints'Length;
         end loop;
         Joints := new Joint_Point_Array (1 .. Natural'Max (1, Total));
         Total := 0;
         for A in 1 .. Count loop
            declare
               Flow_Of : Arm_Flow renames Arms (Arms'First + A - 1);
            begin
               for V of Flow_Of.Poses.all loop
                  declare
                     T : constant Twist_Array := Twists_At (Flow_Of.Joints.all, V.Readings);
                  begin
                     for J in T'Range loop
                        Total := Total + 1;
                        Joints (Total) := (Arm => A, X => T (J).P);
                     end loop;
                  end;
               end loop;
            end;
         end loop;
      end;
      --  Rounds: every point predicted by the terms the round before left and aligned into the fixed eye's picture
      --  across the scales (Driver.Alignment.Align_At_Scale), within the widest window the finest size searches every
      --  round (a window the round before's scatter wide, 0.07 px, kept only the places that agreed with the eye
      --  already, and the eye stayed where it started: the fake's seed 20, 10-21, focal length 1.8 % off, its own
      --  sigma 0.14 %); the terms fitted to the places the aligner found
      --  narrowly, from the flow's estimate (only a start: given its covariance beside the places, a biased flow
      --  held the eye where it stood and was called told, the fake's seed 20 on 10-21 with 208 places kept of 1941
      --  and its joints 17 px off), the places off by more than Z robust sigmas left out until none is. Settled when a
      --  round after the first moves the arms' joints in the picture by no more than its own spread there (the places
      --  found again from where the round before left the eye put it there again); the rounds go on while one of the
      --  two, the places' scatter or that move, still falls. No place found at the first: the flow's eye stands.
      declare
         Widest   : constant Real :=
           Driver.Alignment.Sigma_Of_Window (Driver.Alignment.Finest_Radius (Width, Height));
         Window   : constant Real := Widest;
         Y        : Real_Vector (1 .. G) := [others => 0.0];
         Cov      : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
         --  Cov's share in the flows' directions
         Cov_F    : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
         Definite : Boolean := False;
         Spread   : Real := Real'Last;
         Spread_F : Real := Real'Last;   --  the spread the flows' directions leave alone
         --  The arms' fits' share of the covariance (Arm_World.Variants), and the spread with it: the round's own.
         Cov_V    : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
         Spread_T : Real := Real'Last;
         Settled  : Boolean := False;
         Last_Scatter, Last_Moved : Real := Real'Last;
         Was_U, Was_V : Real_Array (Joints'Range) := [others => 0.0];
         Was_Seen     : Flag_Array (Joints'Range) := [others => False];
         Rounds   : Natural := 0;
         Kept_All : Natural := 0;
         Places_Told : Natural := 0;   --  the last split's directions of the flows' terms the places tell (Report)
         Widened  : Real := 1.0;   --  the most the pictures' jackknife widened the covariance in any direction
         No_World : Boolean := False;   --  the first round found no point to place by
      begin
         for J in Joints'Range loop
            Was_Seen (J) := Seen (Y, Joints (J).Arm, Joints (J).X, Was_U (J), Was_V (J));
         end loop;
         loop
            Rounds := Rounds + 1;
            declare
               type Slot is record
                  Got          : Boolean := False;
                  U, V, Su, Sv : Real := 0.0;
                  --  why the aligner found it not, when it did not (Not_Asked: the point not in the picture)
                  Because      : Driver.Alignment.Reason := Driver.Alignment.Not_Asked;
               end record;
               type Slot_Array is array (Positive range <>) of Slot;
               type Slot_Access is access Slot_Array;
               procedure Free is new Ada.Unchecked_Deallocation (Slot_Array, Slot_Access);
               Slots : Slot_Access := new Slot_Array (Sights'Range);
               procedure One (I : Positive) is
                  S : Sight renames Sights (I);
                  --  Where the eye puts the pixel and its neighbours (the pixel I across and J down from it)
                  Pu, Pv : array (-1 .. 1, -1 .. 1) of Real;
                  Seen_All : Boolean := True;
               begin
                  for I in -1 .. 1 loop
                     for J in -1 .. 1 loop
                        Seen_All := Seen_All and then Seen (Y, S.Arm, S.Around (I, J), Pu (I, J), Pv (I, J));
                     end loop;
                  end loop;
                  if Seen_All then
                     declare
                        --  The warp to second order about the pixel, from its neighbours' places: its linear
                        --  part by central differences, its bend by the second ones (Driver.Alignment.Bend_Part).
                        --  With the linear part by the differences forward alone and no bend, the aligner's places
                        --  at the true eye's prediction erred with the warp's curvature (their correlation 0.08 to
                        --  0.42, the fake's seeds 20 to 24) and robustly by 0.12 to 0.17 px; so asked, by 0.07 to
                        --  0.11, the correlation gone (10-21).
                        Query : constant Driver.Alignment.Prediction :=
                          (From   => (U => S.U0, V => S.V0),
                           To     => (U => Pu (0, 0), V => Pv (0, 0)),
                           Linear => (UU => (Pu (1, 0) - Pu (-1, 0)) / 2.0, UV => (Pu (0, 1) - Pu (0, -1)) / 2.0,
                                      VU => (Pv (1, 0) - Pv (-1, 0)) / 2.0, VV => (Pv (0, 1) - Pv (0, -1)) / 2.0),
                           Cov    => (UU => Window ** 2, UV => 0.0, VV => Window ** 2),
                           Linear_Sigma => Real'Last);
                        Bend : constant Driver.Alignment.Bend_Part :=
                          (U_II => (Pu (1, 0) + Pu (-1, 0) - 2.0 * Pu (0, 0)) / 2.0,
                           U_IJ => (Pu (1, 1) - Pu (1, -1) - Pu (-1, 1) + Pu (-1, -1)) / 4.0,
                           U_JJ => (Pu (0, 1) + Pu (0, -1) - 2.0 * Pu (0, 0)) / 2.0,
                           V_II => (Pv (1, 0) + Pv (-1, 0) - 2.0 * Pv (0, 0)) / 2.0,
                           V_IJ => (Pv (1, 1) - Pv (1, -1) - Pv (-1, 1) + Pv (-1, -1)) / 4.0,
                           V_JJ => (Pv (0, 1) + Pv (0, -1) - 2.0 * Pv (0, 0)) / 2.0);
                        Answer : constant Driver.Alignment.Answer :=
                          Driver.Alignment.Align_At_Scale (Pyramids (S.Picture), Fixed_Pyramid, Query, Bend => Bend);
                        use type Driver.Alignment.Verdict;
                     begin
                        --  A place counts where the aligner tells it narrower than the window it was looked for in: one
                        --  stated as wide or wider tells no more than the prediction did, and such places held the eye
                        --  where the flows had put it (the fake's seed 26 at four poses, 10-22: 174 of 707 kept, stated 5
                        --  to 12 px across a window of 2, the eye told 123 mm off within the arms' reach; without them 388
                        --  places at 0.08 px, the eye told 7 mm off, its terms' chi square 15.5 over 16).
                        if Answer.Verdict = Driver.Alignment.Found and then Answer.Narrow
                          and then Answer.Cov.UU > 0.0 and then Answer.Cov.VV > 0.0
                          and then Answer.Cov.UU < Window ** 2 and then Answer.Cov.VV < Window ** 2
                        then
                           Slots (I) := (Got => True, U => Answer.To.U, V => Answer.To.V,
                                         Su => Sqrt (Answer.Cov.UU), Sv => Sqrt (Answer.Cov.VV),
                                         Because => Driver.Alignment.Matched);
                        else
                           Slots (I).Because :=
                             (if Answer.Verdict = Driver.Alignment.Found then Driver.Alignment.Matched
                              else Answer.Because);
                        end if;
                     end;
                  end if;
               end One;
               procedure Every is new Driver.Parallel_For (One);
               Got : Natural := 0;
               Missed : Unbounded_String;
            begin
               Every (Sights'First, Sights'Last);
               Aligned_At := Driver.Clock.Seconds - Began;
               for S of Slots.all loop
                  if S.Got then
                     Got := Got + 1;
                  end if;
               end loop;
               --  The points not found, by why (a found answer not narrow: MATCHED; a point out of the picture:
               --  NOT_ASKED), for the round's line.
               declare
                  Tally : array (Driver.Alignment.Reason) of Natural := [others => 0];
               begin
                  for S of Slots.all loop
                     if not S.Got then
                        Tally (S.Because) := Tally (S.Because) + 1;
                     end if;
                  end loop;
                  for R in Tally'Range loop
                     if Tally (R) > 0 then
                        Append (Missed, " " & Driver.Alignment.Reason'Image (R) & Tally (R)'Image);
                     end if;
                  end loop;
               end;
               declare
                  Matches : Match_Access := new Match_Array (1 .. Natural'Max (1, Got));
                  Kept    : Flag_Access := new Flag_Array (1 .. Natural'Max (1, Got));
                  K       : Natural := 0;
                  --  Per pair of the arms' variants (Arm_World.Variants): the eye's shift for it (a row each), and each
                  --  of the arms' joints' own move in the picture with it at the eye as it stands (a column each).
                  Pairs   : constant Natural := Variant_Pairs;
                  Shift_V : Matrix_Access :=
                    new Real_Matrix'(1 .. Natural'Max (1, Pairs) => [1 .. G => 0.0]);
                  Move_U  : Matrix_Access :=
                    new Real_Matrix'(Joints'Range => [1 .. Natural'Max (1, Pairs) => 0.0]);
                  Move_V  : Matrix_Access :=
                    new Real_Matrix'(Joints'Range => [1 .. Natural'Max (1, Pairs) => 0.0]);

                  function Kept_Count return Natural is
                     C : Natural := 0;
                  begin
                     for I in 1 .. Got loop
                        if Kept (I) then
                           C := C + 1;
                        end if;
                     end loop;
                     return C;
                  end Kept_Count;

                  --  The kept places' misfits at the terms, over their sigmas.
                  function Residuals (Z : Real_Vector) return Real_Array is
                     R : Real_Array (1 .. 2 * Natural'Max (1, Kept_Count)) := [others => 0.0];
                     N : Natural := 0;
                  begin
                     for I in 1 .. Got loop
                        if Kept (I) then
                           declare
                              M    : Match renames Matches (I);
                              U, V : Real;
                           begin
                              Place (Z, M.Arm, M.X, U, V);
                              R (N + 1) := (U - M.U) / M.Su;
                              R (N + 2) := (V - M.V) / M.Sv;
                              N := N + 2;
                           end;
                        end if;
                     end loop;
                     return R;
                  end Residuals;

                  --  The places' misfits squared.
                  function Cost (R : Real_Array) return Real is
                     Sum : Real := 0.0;
                  begin
                     for X of R loop
                        Sum := Sum + X * X;
                     end loop;
                     return Sum;
                  end Cost;

                  --  The kept places' misfits' derivatives in the terms at Z (central differences), a row each.
                  procedure Jacobian_Into (Z : Real_Vector; Jac : out Real_Matrix) is
                  begin
                     for T in 1 .. G loop
                        declare
                           Up, Down : Real_Vector := Z;
                        begin
                           Up (T) := Z (T) + Eps3;
                           Down (T) := Z (T) - Eps3;
                           declare
                              Ru : constant Real_Array := Residuals (Up);
                              Rd : constant Real_Array := Residuals (Down);
                           begin
                              for I in Jac'Range (1) loop
                                 Jac (I, T) := (Ru (I) - Rd (I)) / (2.0 * Eps3);
                              end loop;
                           end;
                        end;
                     end loop;
                  end Jacobian_Into;

                  --  The normal matrix of the places' misfits at the terms (their part over Scale) with the flow's, and
                  --  the step's right side.
                  procedure Normal_At
                    (Z : Real_Vector; R0 : Real_Array; Ata : out Real_Matrix; Atb : out Real_Vector;
                     Scale : Real := 1.0)
                  is
                     Jac : Matrix_Access := new Real_Matrix (R0'Range, 1 .. G);
                  begin
                     Jacobian_Into (Z, Jac.all);
                     for P in 1 .. G loop
                        Atb (P) := 0.0;
                        for Q in 1 .. G loop
                           Ata (P, Q) := 0.0;
                        end loop;
                     end loop;
                     for I in R0'Range loop
                        for P in 1 .. G loop
                           if Jac (I, P) /= 0.0 then
                              for Q in 1 .. G loop
                                 Ata (P, Q) := Ata (P, Q) + Jac (I, P) * Jac (I, Q) / Scale;
                              end loop;
                              Atb (P) := Atb (P) - Jac (I, P) * R0 (I) / Scale;
                           end if;
                        end loop;
                     end loop;
                     --  The bend's prior, as it is (no misfit scales it).
                     for K in 1 .. 2 loop
                        Ata (G_F + K, G_F + K) := Ata (G_F + K, G_F + K) + Bend_Prior (K);
                        Atb (G_F + K) := Atb (G_F + K) - Bend_Prior (K) * Bend_Of (Z, K);
                     end loop;
                     Free (Jac);
                  end Normal_At;

                  --  How the round's estimate answers a change of its evidence, to first order, as its own steps take
                  --  it: the places' Gauss-Newton step in their directions (Split's with Lambda one or more,
                  --  Places_Step), then the flows' along theirs at the places' terms (Gauss-Newton on the flows'
                  --  residuals, their directions together). In Split's coordinates (Dirs, a flows' sigma each) a
                  --  places' shift A moves the flows' directions by -N^-1 C A, N the flows' normal matrix along their
                  --  directions and C its products with the places', and a change of the flows' residuals whose
                  --  products with their directions' are B by -N^-1 B. The covariance (Cov) is carried through that
                  --  map: what the places leave uncertain (theirs and the pictures' jackknife) moves the flows'
                  --  directions with it, which the flows' own jackknife, the places' terms held, does not see (the
                  --  fake's seed 22 at eight poses, 10-21: the flows' least through the true eye's other terms
                  --  +0.13 % in the focal length, the eye left at +0.49 %, its arms' joints 1.46 px off against a
                  --  spread of 0.75); Cov_F its share in the flows' directions. Each of the arms' variants
                  --  (Arm_World.Variants) is taken as such a change: its places and its flows at Y, the eye's shift
                  --  for it, a pair's half the difference of its two ends' (Shift_V; Cov_V the shifts' covariance),
                  --  and its arm's joints' own move in the picture (Move_U, Move_V). Ok False when the flows' normal
                  --  matrix is not definite.
                  procedure Respond
                    (R0          : Real_Array;
                     Jac         : Real_Matrix;
                     Per_Freedom : Real;
                     Dirs        : Real_Matrix;
                     Lambda      : Real_Vector;
                     Cov, Cov_F  : in out Real_Matrix;
                     Ok          : out Boolean)
                  is
                     Flows_Of : Natural := 0;                                    --  the flows' directions
                     Which    : array (1 .. G) of Natural := [others => 0];     --  a direction's place among them
                     Along    : array (1 .. G) of Positive := [others => 1];    --  the direction of each
                  begin
                     for I in 1 .. G loop
                        if Lambda (I) < 1.0 then
                           Flows_Of := Flows_Of + 1;
                           Which (I) := Flows_Of;
                           Along (Flows_Of) := I;
                        end if;
                     end loop;
                     Ok := True;
                     Cov_V := [others => [others => 0.0]];
                     declare
                        M    : constant Positive := Natural'Max (1, Flows_Of);
                        type Residual_Grid is array (1 .. Count, 1 .. M) of Real_Access;
                        Base : array (1 .. Count) of Real_Access := [others => null];
                        Grad : Residual_Grid := [others => [others => null]];
                        Nf   : Real_Matrix (1 .. M, 1 .. M) := [others => [others => 0.0]];
                        Nf_L : Real_Matrix (1 .. M, 1 .. M) := [others => [others => 0.0]];
                        Cf   : Real_Matrix (1 .. M, 1 .. G) := [others => [others => 0.0]];
                        Kf   : Real_Matrix (1 .. M, 1 .. G) := [others => [others => 0.0]];   --  N^-1 C
                        Dinv : constant Real_Matrix := Inverse (Dirs);

                        --  Arm A's flow residuals' change per unit of direction I (one flows' sigma), by central
                        --  differences a sigma of the direction's own either way: the flows' for theirs, the places'
                        --  (one over the root of Lambda) for the places'. A flows' sigma along a direction the places
                        --  tell well took the bend, which the flows are read through, past where the lens folds (the
                        --  fake's seed 26, 10-21: the flows' responses 1E11 to 1E17, the eye's covariance 1E14 px).
                        procedure Along_Direction (A, I : Positive; H : out Real_Array) is
                           Up, Down : Real_Vector (1 .. G);
                           Rd : Real_Access := new Real_Array (H'Range);
                           Step : constant Real := (if Lambda (I) > 1.0 then 1.0 / Sqrt (Lambda (I)) else 1.0);
                        begin
                           for T in 1 .. G loop
                              Up (T) := Y (T) + Step * Dirs (T, I);
                              Down (T) := Y (T) - Step * Dirs (T, I);
                           end loop;
                           Flow_Residuals (Up, A, H);
                           Flow_Residuals (Down, A, Rd.all);
                           for Q in H'Range loop
                              H (Q) := (H (Q) - Rd (Q)) / (2.0 * Step);
                           end loop;
                           Free (Rd);
                        end Along_Direction;

                        function Product (X, W : Real_Array) return Real is
                           Sum : Real := 0.0;
                        begin
                           for Q in X'Range loop
                              Sum := Sum + X (Q) * W (W'First + Q - X'First);
                           end loop;
                           return Sum;
                        end Product;

                        --  One end of a variant V of arm A: the eye's shift for it (Shift) and the arm's joints in the
                        --  picture at Y as the variant has them (Pu, Pv: each pose's, joint after joint, as Joints).
                        procedure Variant_End
                          (A : Positive; V : Arm_Variant; Shift : out Real_Vector; Pu, Pv : out Real_Array)
                        is
                           World   : Arm_World renames Worlds (Worlds'First + A - 1);
                           Flow_Of : Arm_Flow renames Arms (Arms'First + A - 1);
                           Atb     : Real_Vector (1 .. G) := [others => 0.0];
                           Row     : Natural := 0;
                           Coord   : Real_Vector (1 .. G);
                        begin
                           --  The places: each of the arm's kept points where the variant's eye, lens and table put it.
                           for I in 1 .. Got loop
                              if Kept (I) then
                                 if Matches (I).Arm = A then
                                    declare
                                       Mt    : Match renames Matches (I);
                                       Local : constant Positive := Mt.Picture - Pictures_Before (A);
                                       W     : Wrist_Picture := World.Pictures (World.Pictures'First + Local - 1);
                                       X     : Vec3;
                                       U, Vv : Real;
                                    begin
                                       W.Eye := V.Eyes (V.Eyes'First + Local - 1);
                                       if On_Table (W, V.Lens, V.Table, Mt.U0, Mt.V0, X) then
                                          Place (Y, A, X, U, Vv);
                                          declare
                                             Du : constant Real := (U - Mt.U) / Mt.Su - R0 (R0'First + Row);
                                             Dv : constant Real := (Vv - Mt.V) / Mt.Sv - R0 (R0'First + Row + 1);
                                          begin
                                             for T in 1 .. G loop
                                                Atb (T) := Atb (T)
                                                  - (Jac (Jac'First (1) + Row, T) * Du
                                                     + Jac (Jac'First (1) + Row + 1, T) * Dv) / Per_Freedom;
                                             end loop;
                                          end;
                                       end if;
                                    end;
                                 end if;
                                 Row := Row + 2;
                              end if;
                           end loop;
                           Shift := Places_Step (Dirs, Lambda, Atb);
                           Coord := Dinv * Shift;
                           --  The flows: the variant's turns at the arm's poses.
                           declare
                              Tw : Twist_Grid_Access :=
                                new Twist_Grid (Flow_Of.Poses'Range, 1 .. Natural'Max (1, Flow_Of.Joints'Length));
                              Rv : Real_Access := new Real_Array (1 .. Layouts (A).Rows);
                              B  : Real_Vector (1 .. M) := [others => 0.0];
                              Q  : Natural := 0;
                           begin
                              for Pose in Flow_Of.Poses'Range loop
                                 declare
                                    T : constant Twist_Array := Twists_At (V.Joints.all, Flow_Of.Poses (Pose).Readings);
                                 begin
                                    for J in T'Range loop
                                       Tw (Pose, J) := T (J);
                                       Place (Y, A, T (J).P, Pu (Pu'First + Q), Pv (Pv'First + Q));
                                       Q := Q + 1;
                                    end loop;
                                 end;
                              end loop;
                              if Flows_Of > 0 then
                                 Flow_Residuals (Y, A, Rv.all, Tw);
                                 for F in 1 .. Flows_Of loop
                                    for R in Rv'Range loop
                                       B (F) := B (F) + Grad (A, F) (R) * (Rv (R) - Base (A) (R));
                                    end loop;
                                    for I in 1 .. G loop
                                       if Which (I) = 0 then
                                          B (F) := B (F) + Cf (F, I) * Coord (I);
                                       end if;
                                    end loop;
                                 end loop;
                                 declare
                                    Tf : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Nf_L, B);
                                 begin
                                    for F in 1 .. Flows_Of loop
                                       Coord (Along (F)) := -Tf (F);
                                    end loop;
                                 end;
                              end if;
                              Shift := Dirs * Coord;
                              Free (Tw);
                              Free (Rv);
                           end;
                        end Variant_End;
                     begin
                        if Flows_Of > 0 then
                           for A in 1 .. Count loop
                              Base (A) := new Real_Array (1 .. Layouts (A).Rows);
                              Flow_Residuals (Y, A, Base (A).all);
                              for F in 1 .. Flows_Of loop
                                 Grad (A, F) := new Real_Array (1 .. Layouts (A).Rows);
                                 Along_Direction (A, Along (F), Grad (A, F).all);
                              end loop;
                           end loop;
                           for A in 1 .. Count loop
                              declare
                                 H : Real_Access := new Real_Array (1 .. Layouts (A).Rows);
                              begin
                                 for I in 1 .. G loop
                                    if Which (I) = 0 then
                                       Along_Direction (A, I, H.all);
                                       for F in 1 .. Flows_Of loop
                                          Cf (F, I) := Cf (F, I) + Product (Grad (A, F).all, H.all);
                                       end loop;
                                    else
                                       for F in 1 .. Flows_Of loop
                                          Nf (F, Which (I)) :=
                                            Nf (F, Which (I)) + Product (Grad (A, F).all, Grad (A, Which (I)).all);
                                       end loop;
                                    end if;
                                 end loop;
                                 Free (H);
                              end;
                           end loop;
                           Driver.Numerics.Dense.Cholesky (Nf, Nf_L, Ok);
                           if Ok then
                              for I in 1 .. G loop
                                 if Which (I) = 0 then
                                    declare
                                       Column : Real_Vector (1 .. M);
                                    begin
                                       for F in 1 .. M loop
                                          Column (F) := Cf (F, I);
                                       end loop;
                                       Column := Driver.Numerics.Dense.Cholesky_Solve (Nf_L, Column);
                                       for F in 1 .. M loop
                                          Kf (F, I) := Column (F);
                                       end loop;
                                    end;
                                 end if;
                              end loop;
                           end if;
                        end if;
                        if Ok then
                           --  The covariance through the map, in Split's coordinates: L Cd L', L the identity with
                           --  -N^-1 C from the places' directions into the flows'.
                           declare
                              Lm : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                              Cd : Real_Matrix (1 .. G, 1 .. G);
                              Ct : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                           begin
                              for I in 1 .. G loop
                                 Lm (I, I) := 1.0;
                                 if Which (I) > 0 then
                                    for J in 1 .. G loop
                                       if Which (J) = 0 then
                                          Lm (I, J) := -Kf (Which (I), J);
                                       end if;
                                    end loop;
                                 end if;
                              end loop;
                              Cd := Lm * (Dinv * Cov * Transpose (Dinv)) * Transpose (Lm);
                              for I in 1 .. G loop
                                 for J in 1 .. G loop
                                    if Which (I) > 0 and then Which (J) > 0 then
                                       Ct (I, J) := Cd (I, J);
                                    end if;
                                 end loop;
                              end loop;
                              Cov := Dirs * Cd * Transpose (Dirs);
                              Cov_F := Dirs * Ct * Transpose (Dirs);
                              for I in 1 .. G loop
                                 for J in I + 1 .. G loop
                                    Cov (I, J) := (Cov (I, J) + Cov (J, I)) / 2.0;
                                    Cov (J, I) := Cov (I, J);
                                    Cov_F (I, J) := (Cov_F (I, J) + Cov_F (J, I)) / 2.0;
                                    Cov_F (J, I) := Cov_F (I, J);
                                 end loop;
                              end loop;
                           end;
                           --  The arms' variants, pair by pair.
                           declare
                              Pair : Natural := 0;
                           begin
                              for A in 1 .. Count loop
                                 if Variants_Fit (A) then
                                    declare
                                       World   : Arm_World renames Worlds (Worlds'First + A - 1);
                                       Flow_Of : Arm_Flow renames Arms (Arms'First + A - 1);
                                       First_J : constant Natural := Joints_Before (A);
                                       Points  : constant Positive :=
                                         Natural'Max (1, Flow_Of.Poses'Length * Flow_Of.Joints'Length);
                                    begin
                                       for Kp in 1 .. World.Variants'Length / 2 loop
                                          Pair := Pair + 1;
                                          declare
                                             Shift  : array (1 .. 2) of Real_Vector (1 .. G);
                                             Pu, Pv : array (1 .. 2) of Real_Array (1 .. Points);
                                          begin
                                             for E in 1 .. 2 loop
                                                Variant_End
                                                  (A, World.Variants (World.Variants'First + 2 * (Kp - 1) + E - 1),
                                                   Shift (E), Pu (E), Pv (E));
                                             end loop;
                                             for T in 1 .. G loop
                                                Shift_V (Pair, T) := (Shift (2) (T) - Shift (1) (T)) / 2.0;
                                             end loop;
                                             for Q in 1 .. Flow_Of.Poses'Length * Flow_Of.Joints'Length loop
                                                Move_U (First_J + Q, Pair) := (Pu (2) (Q) - Pu (1) (Q)) / 2.0;
                                                Move_V (First_J + Q, Pair) := (Pv (2) (Q) - Pv (1) (Q)) / 2.0;
                                             end loop;
                                          end;
                                       end loop;
                                    end;
                                 end if;
                              end loop;
                              for P in 1 .. G loop
                                 for Q in 1 .. G loop
                                    for K2 in 1 .. Pair loop
                                       Cov_V (P, Q) := Cov_V (P, Q) + Shift_V (K2, P) * Shift_V (K2, Q);
                                    end loop;
                                 end loop;
                              end loop;
                           end;
                        end if;
                        for A in 1 .. Count loop
                           Free (Base (A));
                           for F in 1 .. M loop
                              Free (Grad (A, F));
                           end loop;
                        end loop;
                     end;
                  end Respond;

                  Dropped : Natural;
                  Moved   : Real := 0.0;
                  Scatter : Real := Real'Last;   --  of the kept places from the terms, pixels
               begin
                  for I in Slots'Range loop
                     if Slots (I).Got then
                        K := K + 1;
                        Matches (K) := (Arm => Sights (I).Arm, Picture => Sights (I).Picture,
                                        U0 => Sights (I).U0, V0 => Sights (I).V0, X => Sights (I).X,
                                        U => Slots (I).U, V => Slots (I).V,
                                        Su => Slots (I).Su, Sv => Slots (I).Sv);
                        Kept (K) := True;
                     end if;
                  end loop;
                  Free (Slots);
                  if Got > 0 then
                     loop
                        --  The places' directions and the flows' alternately, until the flows move the eye along theirs
                        --  by no more than Unchanged_Fraction of a sigma: damped Gauss-Newton on the places in the
                        --  directions they tell (Split, Places_Step), each step halved until it lowers their cost,
                        --  until a step lowers it by less than one residual's worth; then the flows along each of
                        --  theirs.
                        for Pass in 1 .. Real'Machine_Mantissa loop
                           declare
                              Along : Real := 0.0;   --  the most a flows' direction moved the eye, in its sigmas
                           begin
                              loop
                                 declare
                                    R0     : constant Real_Array := Residuals (Y);
                                    C0     : constant Real := Cost (R0) + Bend_Cost (Y);
                                    Ata    : Real_Matrix (1 .. G, 1 .. G);
                                    Atb    : Real_Vector (1 .. G);
                                    Dirs   : Real_Matrix (1 .. G, 1 .. G);
                                    Lambda : Real_Vector (1 .. G);
                                    Better : Boolean := False;
                                 begin
                                    Normal_At (Y, R0, Ata, Atb);
                                    Split (Ata, Dirs, Lambda);
                                    declare
                                       D     : Real_Vector := Places_Step (Dirs, Lambda, Atb);
                                       Trial : Real_Vector (1 .. G);
                                    begin
                                       for Halving in 1 .. Real'Machine_Mantissa loop
                                          for T in 1 .. G loop
                                             Trial (T) := Y (T) + D (T);
                                          end loop;
                                          declare
                                             C1 : constant Real := Cost (Residuals (Trial)) + Bend_Cost (Trial);
                                          begin
                                             if C1 < C0 then
                                                Y := Trial;
                                                Better := C0 - C1 >= Real'Max (1.0, C0 / Real (R0'Length));
                                                exit;
                                             end if;
                                          end;
                                          for T in 1 .. G loop
                                             D (T) := D (T) / 2.0;
                                          end loop;
                                       end loop;
                                    end;
                                    exit when not Better;
                                 end;
                              end loop;
                              declare
                                 R0     : constant Real_Array := Residuals (Y);
                                 Ata    : Real_Matrix (1 .. G, 1 .. G);
                                 Atb    : Real_Vector (1 .. G);
                                 Dirs   : Real_Matrix (1 .. G, 1 .. G);
                                 Lambda : Real_Vector (1 .. G);
                              begin
                                 Normal_At (Y, R0, Ata, Atb);
                                 Split (Ata, Dirs, Lambda);
                                 for I in 1 .. G loop
                                    if Lambda (I) < 1.0 then
                                       declare
                                          V : Real_Vector (1 .. G);
                                          S : Real;
                                       begin
                                          for P in 1 .. G loop
                                             V (P) := Dirs (P, I);
                                          end loop;
                                          S := Line_Search (Y, V);
                                          if S < Real'Last then
                                             for P in 1 .. G loop
                                                Y (P) := Y (P) + S * V (P);
                                             end loop;
                                             Along := Real'Max (Along, abs S);
                                          end if;
                                       end;
                                    end if;
                                 end loop;
                              end;
                              exit when Along <= Driver.Conventions.Unchanged_Fraction;
                           end;
                        end loop;
                        --  The places off by more than Z robust sigmas left out, until none is.
                        declare
                           R     : constant Real_Array := Residuals (Y);
                           Sigma : constant Real := Driver.Stats.Robust_Sigma (R);
                           N     : Natural := 0;
                        begin
                           Dropped := 0;
                           for I in 1 .. Got loop
                              if Kept (I) then
                                 if Sqrt (R (N + 1) ** 2 + R (N + 2) ** 2) > Driver.Conventions.Z * Sigma * Sqrt (2.0)
                                 then
                                    Kept (I) := False;
                                    Dropped := Dropped + 1;
                                 end if;
                                 N := N + 2;
                              end if;
                           end loop;
                        end;
                        --  And the flows' pixels off by more than the gate of their spread, as Fit_Eye leaves them out.
                        for A in 1 .. Count loop
                           declare
                              L : Layout renames Layouts (A).all;
                              R : Real_Access := new Real_Array (1 .. L.Rows);
                           begin
                              Flow_Residuals (Y, A, R.all);
                              for Q in 1 .. L.Pixels_All loop
                                 if Flow_Kept (A) (Q) then
                                    declare
                                       Own  : Real := 0.0;
                                       Gate : constant Driver.Uncertain.Gate :=
                                         Driver.Uncertain.Vector_Gate (L.Count (Q) - 1, 0, Tests => Flow_Pixels);
                                    begin
                                       for K in 0 .. L.Count (Q) - 1 loop
                                          Own := Own + R (L.First_Row (Q) + K) ** 2;
                                       end loop;
                                       if Driver.Uncertain.Significant (Gate, Sqrt (Own), 1.0) then
                                          Flow_Kept (A) (Q) := False;
                                          Dropped := Dropped + 1;
                                       end if;
                                    end;
                                 end if;
                              end loop;
                              Free (R);
                           end;
                        end loop;
                        exit when Dropped = 0 or else Kept_Count = 0;
                     end loop;
                  end if;
                  Kept_All := Kept_Count;
                  Fitted_At := Driver.Clock.Seconds - Began;
                  --  The covariance: in the places' directions the inverse of their normal matrix over their misfit
                  --  per freedom (never below one); in the flows', what the flows' jackknife over their poses leaves
                  --  (each pose of each arm left out, its shift along the direction weighed by the pixels it holds).
                  --  Each jackknife's variance is taken at its freedoms: one over G groups states it to G - 1 of them,
                  --  and an error over a sigma so known follows Student's t, whose variance is that many freedoms over
                  --  two fewer (Driver.Uncertain's rule for a sigma from few samples, here as a variance). Taken at the
                  --  jackknife's word alone, two poses of the boot's (four groups) told five of the fake's nine bent
                  --  seeds with their arms' reach 1 to 2 cm off and the eye's terms six times their stated variance
                  --  (10-22). The flows' information is taken as their jackknife states it: weighed once by their
                  --  estimate's misfit against the places where those tell a direction better (34c4c788), it was taken
                  --  at 1/2.7 to 1/17 - that misfit is their estimate's alone, several per cent off in the focal
                  --  length, where the refinement uses them only along their own directions with the places' terms
                  --  held - and once the places tell every direction a plane can (the boot's scene stills, Flows), the
                  --  flows' errors along the two it cannot were 0.5 to 1.15 of their jackknife's sigma; the weighing
                  --  left seven of seventeen eyes told that were 2 to 4 mm off, and without it, the stills doubled as the
                  --  boot doubles them, all nine of the fake's bent seeds were told 3.5 to 9.3 mm off within each arm's
                  --  reach, their terms' chi square 151.9 over 144 (10-22).
                  declare
                     R0    : constant Real_Array := Residuals (Y);
                     Per_Freedom : constant Real :=
                       Real'Max (1.0, Cost_Of (R0) / Real (Natural'Max (1, R0'Length - G)));
                     Ata    : Real_Matrix (1 .. G, 1 .. G);
                     Atb    : Real_Vector (1 .. G);
                     Dirs   : Real_Matrix (1 .. G, 1 .. G);
                     Lambda : Real_Vector (1 .. G);
                  begin
                     Normal_At (Y, R0, Ata, Atb, Scale => Per_Freedom);
                     Split (Ata, Dirs, Lambda);
                     Definite := (for all I in 1 .. G => Lambda (I) > 0.0) and then not Flows_Untold;
                     Places_Told := 0;
                     for I in 1 .. G_F loop   --  the bend's own two are the places' always
                        if Lambda (I) >= 1.0 then
                           Places_Told := Places_Told + 1;
                        end if;
                     end loop;
                     Cov := [others => [others => 0.0]];
                     Cov_F := [others => [others => 0.0]];
                     for I in 1 .. G loop
                        declare
                           Var : Real := 0.0;
                        begin
                           if Lambda (I) >= 1.0 then
                              Var := 1.0 / Lambda (I);
                           else
                              declare
                                 V      : Real_Vector (1 .. G);
                                 Groups : Natural := 0;
                                 Sum    : Real := 0.0;
                                 Pixels : Natural := 0;
                              begin
                                 for P in 1 .. G loop
                                    V (P) := Dirs (P, I);
                                 end loop;
                                 for A in 1 .. Count loop
                                    for Q in 1 .. Layouts (A).Pixels_All loop
                                       Pixels := Pixels + Boolean'Pos (Flow_Kept (A) (Q));
                                    end loop;
                                 end loop;
                                 --  Every pose's misfit a flows' sigma either way along the direction and at the
                                 --  estimate (Pose_Costs, three misfits in all): the misfit without a pose is the
                                 --  parabola through the three totals less the pose's own, its shift the parabola's
                                 --  least against the whole's. A line search for every pose left out took two misfits
                                 --  of every three the refinement took.
                                 declare
                                    Ym, Yp     : Real_Vector (1 .. G);
                                    Cm, C0, Cp : Real_Array (1 .. Natural'Max (1, Pose_Slots));
                                    Tm, T0, Tp : Real := 0.0;
                                    --  Where the parabola through the misfits at -1, 0 and 1 is least; none when it
                                    --  does not open upwards.
                                    function Least (Fm, F0, Fp : Real; Opens : out Boolean) return Real is
                                       Curvature : constant Real := Fp - 2.0 * F0 + Fm;
                                    begin
                                       Opens := Curvature > 0.0;
                                       return (if Opens then (Fm - Fp) / (2.0 * Curvature) else 0.0);
                                    end Least;
                                    Opens : Boolean;
                                    Whole : Real;
                                 begin
                                    for T in 1 .. G loop
                                       Ym (T) := Y (T) - V (T);
                                       Yp (T) := Y (T) + V (T);
                                    end loop;
                                    Pose_Costs (Ym, Cm);
                                    Pose_Costs (Y, C0);
                                    Pose_Costs (Yp, Cp);
                                    for K in 1 .. Pose_Slots loop
                                       Tm := Tm + Cm (K);
                                       T0 := T0 + C0 (K);
                                       Tp := Tp + Cp (K);
                                    end loop;
                                    Whole := Least (Tm, T0, Tp, Opens);
                                    Definite := Definite and then Opens;
                                    for A in 1 .. Count loop
                                       for Pose in Arms (Arms'First + A - 1).Poses'Range loop
                                          declare
                                             Held : Natural := 0;
                                             K    : constant Positive := Pose_Slot (A, Pose);
                                          begin
                                             for Q in 1 .. Layouts (A).Pixels_All loop
                                                if Flow_Kept (A) (Q) and then Layouts (A).Pose_Of_Pixel (Q) = Pose then
                                                   Held := Held + 1;
                                                end if;
                                             end loop;
                                             if Held > 0 and then Held < Pixels then
                                                declare
                                                   S : constant Real :=
                                                     Least (Tm - Cm (K), T0 - C0 (K), Tp - Cp (K), Opens) - Whole;
                                                begin
                                                   Definite := Definite and then Opens;
                                                   Groups := Groups + 1;
                                                   Sum := Sum + (Real (Pixels) / Real (Held) - 1.0) * S * S;
                                                end;
                                             end if;
                                          end;
                                       end loop;
                                    end loop;
                                 end;
                                 if Groups > 3 then
                                    Var := Sum / Real (Groups) * Real (Groups - 1) / Real (Groups - 3);
                                 else
                                    Definite := False;
                                 end if;
                              end;
                           end if;
                           for P in 1 .. G loop
                              for Q in 1 .. G loop
                                 Cov (P, Q) := Cov (P, Q) + Dirs (P, I) * Var * Dirs (Q, I);
                                 if Lambda (I) < 1.0 then
                                    Cov_F (P, Q) := Cov_F (P, Q) + Dirs (P, I) * Var * Dirs (Q, I);
                                 end if;
                              end loop;
                           end loop;
                        end;
                     end loop;
                     --  What one picture's places share (where its eye stood, its pixels' errors together) their
                     --  scatter does not tell: the covariance widened to the eye's spread with one picture's places
                     --  left out at a time, each shift weighed by the places the picture holds (as Fit_Eye's poses).
                     if Definite then
                        declare
                           Jac      : Matrix_Access := new Real_Matrix (R0'Range, 1 .. G);
                           Spread_M : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                           Groups   : Natural := 0;
                           Total    : constant Natural := Kept_Count;
                        begin
                           Jacobian_Into (Y, Jac.all);
                           for P in Pyramids'Range loop
                              declare
                                 Ap  : Real_Matrix (1 .. G, 1 .. G) := [others => [others => 0.0]];
                                 Gp  : Real_Vector (1 .. G) := [others => 0.0];
                                 Mp  : Natural := 0;
                                 Row : Natural := 0;
                              begin
                                 for I in 1 .. Got loop
                                    if Kept (I) then
                                       if Matches (I).Picture = P then
                                          Mp := Mp + 1;
                                          for Q in Row + 1 .. Row + 2 loop
                                             for A in 1 .. G loop
                                                for B in 1 .. G loop
                                                   Ap (A, B) := Ap (A, B) + Jac (Q, A) * Jac (Q, B) / Per_Freedom;
                                                end loop;
                                                Gp (A) := Gp (A) + Jac (Q, A) * R0 (Q) / Per_Freedom;
                                             end loop;
                                          end loop;
                                       end if;
                                       Row := Row + 2;
                                    end if;
                                 end loop;
                                 if Mp > 0 and then Mp < Total then
                                    declare
                                       Theta : constant Real_Vector := Places_Step (Dirs, Lambda, Gp);
                                       W     : constant Real := Real (Total) / Real (Mp) - 1.0;
                                    begin
                                       Groups := Groups + 1;
                                       for A in 1 .. G loop
                                          for B in 1 .. G loop
                                             Spread_M (A, B) := Spread_M (A, B) + W * Theta (A) * Theta (B);
                                          end loop;
                                       end loop;
                                    end;
                                 end if;
                              end;
                           end loop;
                           if Groups > 3 then
                              for A in 1 .. G loop
                                 for B in 1 .. G loop
                                    Spread_M (A, B) := Spread_M (A, B) / Real (Groups) * Real (Groups - 1) / Real (Groups - 3);
                                 end loop;
                              end loop;
                           elsif Groups >= 2 then
                              Definite := False;
                           end if;
                           Widen_Covariance (Cov, Spread_M, Widened);
                           declare
                              Responded : Boolean;
                           begin
                              Respond (R0, Jac.all, Per_Freedom, Dirs, Lambda, Cov, Cov_F, Responded);
                              Definite := Definite and then Responded;
                           end;
                           Free (Jac);
                        end;
                     end if;
                  end;
                  --  How far the kept places lie from where the terms put them, in pixels: the next round's window.
                  declare
                     Off : Real_Array (1 .. 2 * Natural'Max (1, Kept_All));
                     N   : Natural := 0;
                  begin
                     for I in 1 .. Got loop
                        if Kept (I) then
                           declare
                              U, V : Real;
                           begin
                              Place (Y, Matches (I).Arm, Matches (I).X, U, V);
                              Off (N + 1) := U - Matches (I).U;
                              Off (N + 2) := V - Matches (I).V;
                              N := N + 2;
                           end;
                        end if;
                     end loop;
                     if N > 0 then
                        Scatter := Driver.Stats.Robust_Sigma (Off (1 .. N));
                     end if;
                  end;
                  --  The arms' joints in the picture: how far this round moved them, and how far the covariance
                  --  leaves them (the worse axis, the worst joint).
                  Spread := 0.0;
                  Spread_F := 0.0;
                  Spread_T := 0.0;
                  for J in Joints'Range loop
                     declare
                        U, V : Real;
                        Inside : constant Boolean := Seen (Y, Joints (J).Arm, Joints (J).X, U, V);
                        Ju, Jv : Real_Vector (1 .. G) := [others => 0.0];
                     begin
                        if Inside and then Was_Seen (J) then
                           Moved := Real'Max (Moved, Sqrt ((U - Was_U (J)) ** 2 + (V - Was_V (J)) ** 2));
                        end if;
                        Was_U (J) := U;
                        Was_V (J) := V;
                        Was_Seen (J) := Inside;
                        if Definite and then Inside then
                           for T in 1 .. G loop
                              declare
                                 Up, Down : Real_Vector := Y;
                                 Uu, Vu, Ud, Vd : Real;
                                 B1, B2 : Boolean;
                              begin
                                 Up (T) := Y (T) + Eps3;
                                 Down (T) := Y (T) - Eps3;
                                 B1 := Seen (Up, Joints (J).Arm, Joints (J).X, Uu, Vu);
                                 B2 := Seen (Down, Joints (J).Arm, Joints (J).X, Ud, Vd);
                                 if B1 or else B2 then
                                    Ju (T) := (Uu - Ud) / (2.0 * Eps3);
                                    Jv (T) := (Vu - Vd) / (2.0 * Eps3);
                                 end if;
                              end;
                           end loop;
                           declare
                              Vuu : constant Real := Ju * (Cov * Ju);
                              Vvv : constant Real := Jv * (Cov * Jv);
                              --  with the arms' variants: the eye's shift for each pair and the joint's own move
                              Tuu : Real := Vuu;
                              Tvv : Real := Vvv;
                           begin
                              for P in 1 .. Pairs loop
                                 declare
                                    Du : Real := Move_U (J, P);
                                    Dv : Real := Move_V (J, P);
                                 begin
                                    for T in 1 .. G loop
                                       Du := Du + Ju (T) * Shift_V (P, T);
                                       Dv := Dv + Jv (T) * Shift_V (P, T);
                                    end loop;
                                    Tuu := Tuu + Du ** 2;
                                    Tvv := Tvv + Dv ** 2;
                                 end;
                              end loop;
                              Spread_T := Real'Max (Spread_T, Sqrt (Real'Max (0.0, Real'Max (Tuu, Tvv))));
                              Spread := Real'Max (Spread, Sqrt (Real'Max (0.0, Real'Max (Vuu, Vvv))));
                              Spread_F := Real'Max
                                (Spread_F, Sqrt (Real'Max (0.0, Real'Max (Ju * (Cov_F * Ju), Jv * (Cov_F * Jv)))));
                           end;
                        end if;
                     end;
                  end loop;
                  if not Definite then
                     Spread := Real'Last;
                     Spread_T := Real'Last;
                  end if;
                  declare
                     --  Of each arm, the places kept and the cells of the picture a patch wide they fall in (how much
                     --  of the picture they cover).
                     function Per_Arm return String is
                        Text  : Unbounded_String;
                        Side  : constant Positive :=
                          2 * Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
                        Cells_U : constant Positive := (Width + Side - 1) / Side;
                        Cells_V : constant Positive := (Height + Side - 1) / Side;
                     begin
                        for A in 1 .. Count loop
                           declare
                              C      : Natural := 0;
                              Cells  : Natural := 0;
                              Marked : Flag_Array (1 .. Cells_U * Cells_V) := [others => False];
                           begin
                              for I in 1 .. Got loop
                                 if Kept (I) and then Matches (I).Arm = A then
                                    C := C + 1;
                                    declare
                                       Cu : constant Natural := Natural'Min
                                         (Cells_U - 1, Natural (Real'Floor (Real'Max (0.0, Matches (I).U))) / Side);
                                       Cv : constant Natural := Natural'Min
                                         (Cells_V - 1, Natural (Real'Floor (Real'Max (0.0, Matches (I).V))) / Side);
                                    begin
                                       if not Marked (Cv * Cells_U + Cu + 1) then
                                          Marked (Cv * Cells_U + Cu + 1) := True;
                                          Cells := Cells + 1;
                                       end if;
                                    end;
                                 end if;
                              end loop;
                              Append (Text, (if A > 1 then "," else "") & C'Image & " in" & Cells'Image & " cells");
                           end;
                        end loop;
                        return "per arm" & To_String (Text) & " of" & Natural'Image (Cells_U * Cells_V);
                     end Per_Arm;
                     --  The first arm's own terms' variances: its turn's (1 to 3) and its centre's (4 to 6).
                     function Arm_1 (K : Positive) return Real is (Cov (Offset (1) + K, Offset (1) + K));
                  begin
                  Driver.Log.Line (Driver.Log.Robot, "flow eye: the world's points, round" & Rounds'Image & ":"
                                   & Sights'Length'Image & " asked," & Got'Image & " found," & Kept_All'Image
                                   & " kept (" & Per_Arm & "; not found:" & To_String (Missed) & "), "
                                   & Driver.Log.Image (Scatter, 2)
                                   & " px from the eye, aligned at " & Driver.Log.Image (Real (Aligned_At), 1)
                                   & " s, fitted at " & Driver.Log.Image (Real (Fitted_At), 1) & ", done at "
                                   & Driver.Log.Image (Real (Driver.Clock.Seconds - Began), 1) & ", widened "
                                   & Driver.Log.Image (Widened, 1) & "; the arms' joints "
                                   & "moved " & Driver.Log.Image (Moved, 2) & " px, spread "
                                   & Driver.Log.Image (Spread, 2) & " px (the flows' directions "
                                   & Driver.Log.Image (Spread_F, 2) & ", with the arms' fits "
                                   & Driver.Log.Image (Spread_T, 2) & "), the places telling" & Places_Told'Image
                                   & " of the flows' terms' directions, focal "
                                   & Driver.Log.Image (Lens_Of (Y).Fx, 1)
                                   & (if Definite
                                      then "; sigmas: focal " & Driver.Log.Image (100.0 * Sqrt (Cov (1, 1)), 2)
                                           & " %, principal"
                                           & " point " & Driver.Log.Image (Sqrt (Cov (2, 2)), 2) & " "
                                           & Driver.Log.Image (Sqrt (Cov (3, 3)), 2) & " px, aspect "
                                           & Driver.Log.Image (100.0 * Sqrt (Cov (4, 4)), 2) & " %, arm 1 centre "
                                           & Driver.Log.Image (1000.0 * Sqrt (Arm_1 (4) + Arm_1 (5) + Arm_1 (6)), 1)
                                           & " mm, turn "
                                           & Driver.Log.Image (180.0 / Ada.Numerics.Pi
                                                               * Sqrt (Arm_1 (1) + Arm_1 (2) + Arm_1 (3)), 2)
                                           & " deg"
                                           & ", bend " & Driver.Log.Image (Lens_Of (Y).K1, 4) & " "
                                           & Driver.Log.Image (Lens_Of (Y).K2, 4) & " (sigmas "
                                           & Driver.Log.Image (Sqrt (Cov (G_F + 1, G_F + 1)), 4) & " "
                                           & Driver.Log.Image (Sqrt (Cov (G_F + 2, G_F + 2)), 4)
                                           & ", the flows' directions' share "
                                           & Driver.Log.Image
                                               (Cov_F (G_F + 1, G_F + 1)
                                                / Real'Max (Cov (G_F + 1, G_F + 1), Real'Model_Small), 2)
                                           & ")"
                                      else ""));
                  end;
                  Free (Matches);
                  Free (Kept);
                  Free (Shift_V);
                  Free (Move_U);
                  Free (Move_V);
                  if Rounds = 1 and then Kept_All = 0 then
                     Driver.Log.Line (Driver.Log.Robot, "flow eye: no point of the world was found in the fixed eye's"
                                      & " picture; the flow's place stands");
                     No_World := True;
                     exit;
                  end if;
                  Settled := Rounds > 1 and then Got > 0 and then Definite and then Moved <= Spread;
                  exit when Settled
                    or else not (Scatter < Last_Scatter or else Moved < Last_Moved);
                  Last_Scatter := Scatter;
                  Last_Moved := Moved;
               end;
            end;
         end loop;
         if No_World then
            Free (Sights);
            Free (Pyramids);
            Free (Joints);
            return;
         end if;
         --  The eye as the rounds left it, with the places' covariance, judged as Fit_Eye judges.
         Result.Lens := Lens_Of (Y);
         for A in 1 .. Count loop
            Result.Poses (A) := Pose_Of (Y, A);
         end loop;
         if Definite then
            --  The flows' terms' share: the distortion's own uncertainty is in it, its terms marginalized.
            for P in 1 .. G_F loop
               for Q in 1 .. G_F loop
                  Result.Covariance (P, Q) := Cov (P, Q) + Cov_V (P, Q);
               end loop;
            end loop;
         end if;
         Result.Spread := Spread_T;
         Result.Places_Told := Places_Told;
         Result.Determined := False;
         if not Settled then
            Result.Why := To_Unbounded_String
              ("the world's points did not settle the eye: its last round moved the arms' joints farther than their"
               & " spread");
         elsif Spread_T * Driver.Conventions.Z >= Real (Driver.Alignment.Half_Patch (Width, Height)) then
            Result.Why := To_Unbounded_String
              ("the world's points leave the arms' joints in the picture uncertain by"
               & Driver.Log.Image (Spread_T, 2) & " px");
         else
            Result.Why := Null_Unbounded_String;
            Result.Determined := True;
         end if;
      end;
      Free (Sights);
      Free (Pyramids);
      Free (Joints);
      for A in 1 .. Count loop
         Free (Flows (A).Twists);
         Free (Layouts (A));
         Free (Flow_Kept (A));
      end loop;
   end Refine_By_Scene;

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
                         Aspect => Lens.Fy / Lens.Fx, Additive => 1.0, Relative => 0.0, Unit => null, K1 | K2 => 0.0);
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
            Pose.Translation (1), Pose.Translation (2), Pose.Translation (3), Lens.Cx, Lens.Cy,
            Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fy / Lens.Fx)];
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
                  Pixel_Residuals
                    (P, L.Pose_Of_Pixel (Q), L.Pixel_Of_Pixel (Q), Pose.Rotation, X, Part, Count, Rho, Ok);
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

   function Residuals_At
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Relative      : Real) return Real_Array
   is
      Held : aliased constant Pose_Array := Poses;
      P    : Problem := (Poses => Held'Unchecked_Access, Twists => null, Width => Width, Height => Height,
                         Joints => Joints'Length, Cx => Lens.Cx, Cy => Lens.Cy,
                         Aspect => Lens.Fy / Lens.Fx, Additive => 1.0, Relative => Relative, Unit => null,
                         K1 | K2 => 0.0);
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
            Pose.Translation (1), Pose.Translation (2), Pose.Translation (3), Lens.Cx, Lens.Cy,
            Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fy / Lens.Fx)];
         Kept : constant Flag_Array (1 .. L.Pixels_All) := [others => True];
         R    : Real_Access := new Real_Array (1 .. L.Rows);
         Seen : Flag_Access := new Flag_Array (1 .. L.Pixels_All);
      begin
         All_Residuals (P, L.all, Pose.Rotation, X, Kept, R.all, Seen.all);
         return Result : constant Real_Array := R.all do
            Free (R);
            Free (Seen);
            Free (L);
            Free (P.Twists);
         end return;
      end;
   end Residuals_At;

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
                         Aspect => Lens.Fy / Lens.Fx, Additive => 1.0, Relative => 0.0, Unit => null, K1 | K2 => 0.0);
      L    : Layout_Access;
      X    : constant Parameters :=
        [Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fx), 0.0, 0.0, 0.0,
         Pose.Translation (1), Pose.Translation (2), Pose.Translation (3), Lens.Cx, Lens.Cy,
         Ada.Numerics.Long_Elementary_Functions.Log (Lens.Fy / Lens.Fx)];
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
