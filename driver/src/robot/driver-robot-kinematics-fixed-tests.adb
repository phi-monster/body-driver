with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Tests;

package body Driver.Robot.Kinematics.Fixed.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   function Gaussian (G : in out Generator) return Real is
      U1 : constant Real := Real'Max (Uniform (G), 1.0e-12);
      U2 : constant Real := Uniform (G);
   begin
      return Sqrt (-2.0 * Ln (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   ---------------------------------------------------------------------------
   --  The world: the first arm's reference eye at the origin looking along z with a lens of its own, a table one
   --  unit from it, its normal Table_N towards the eye, and boxes standing on it; and, above and behind them, the
   --  eye under test, with a lens of its own whose principal point is far from the middle of its picture.

   Table_N : constant Vec3 := Unit ([0.0, -0.6, -0.8]);
   Table_O : constant Real := -1.0;
   Table_X : constant Vec3 := [1.0, 0.0, 0.0];
   Table_Y : constant Vec3 := Cross (Table_N, Table_X);
   Plane_A : constant Vec3 := (1.0 / Table_O) * Table_N;   --  A * X = 1 on the table

   Arm_Lens  : constant Fit.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
   Eye_Lens  : constant Fit.Lens := (Fx => 330.0, Fy => 322.0, Cx => 352.0, Cy => 206.0, K1 => 0.0, K2 => 0.0);
   Wide_Lens : constant Fit.Lens := (Fx => 330.0, Fy => 322.0, Cx => 352.0, Cy => 206.0, K1 => -0.1, K2 => 0.0);

   function Eye_Pose return Rigid is
      Centre : constant Vec3 := 0.5 * Table_X + 1.0 * Table_Y + Table_O * Table_N;
      Eye    : constant Vec3 := Centre + 3.2 * Table_N - 1.2 * Table_Y;
      Z_Axis : constant Vec3 := Unit (Centre - Eye);
      Y_Axis : constant Vec3 := Cross (Z_Axis, Table_X);
   begin
      return (Rotation    => [[Table_X (1), Y_Axis (1), Z_Axis (1)],
                              [Table_X (2), Y_Axis (2), Z_Axis (2)],
                              [Table_X (3), Y_Axis (3), Z_Axis (3)]],
              Translation => Eye);
   end Eye_Pose;

   --  Where an eye of that lens and pose sees a point of the world, if in front and in its 640 x 480 picture.
   procedure Seen_By (L : Fit.Lens; T : Rigid; X : Vec3; U, V : out Real; Inside : out Boolean) is
      Ahead : Boolean;
   begin
      Fit.Project (L, Transpose (T.Rotation) * (X - T.Translation), U, V, Ahead);
      Inside := Ahead and then U in 0.0 .. 640.0 and then V in 0.0 .. 480.0;
   end Seen_By;

   type Vec_Array is array (Positive range <>) of Vec3;

   --  A scene of Table + Box points, the sightings the eye under test made of them and what is known of them: each
   --  point's estimate (its truth and an error of Own_Sigma in each coordinate), and the reference pixels the arm's
   --  eye saw them at.
   type Scene (Count : Positive) is record
      Truth     : Vec_Array (1 .. Count);
      Points    : Point_Array (1 .. Count);
      Sightings : Sighting_Array (1 .. Count);
      On_Table  : Fit.Flag_Array (1 .. Count);
   end record;

   --  Points drawn on the table and on boxes (a height above it), seen by both eyes; a share of the eye's answers
   --  are false (a pixel anywhere in the picture).
   function Draw
     (G           : in out Generator;
      Table, Box  : Natural;
      Wrong_Share : Real;
      Noise       : Real;
      Own_Sigma   : Real;
      L           : Fit.Lens := Eye_Lens) return Scene
   is
      Pose   : constant Rigid := Eye_Pose;
      Result : Scene (Table + Box);
      Made   : Natural := 0;
   begin
      while Made < Result.Count loop
         declare
            On_Table : constant Boolean := Made < Table;
            S : constant Real := (2.0 * Uniform (G) - 1.0) * 0.7;
            T : constant Real := (2.0 * Uniform (G) - 1.0) * 0.45;
            H : constant Real := (if On_Table then 0.0 else 0.1 + 0.3 * Uniform (G));
            X : constant Vec3 := Table_O * Table_N + S * Table_X + T * Table_Y + H * Table_N;
            Ua, Va, U, V : Real;
            In_Arm, In_Eye : Boolean;
         begin
            Seen_By (Arm_Lens, Identity, X, Ua, Va, In_Arm);
            Seen_By (L, Pose, X, U, V, In_Eye);
            if In_Arm and then In_Eye then
               Made := Made + 1;
               declare
                  Wrong : constant Boolean := Uniform (G) < Wrong_Share;
                  Own   : constant Real := Own_Sigma ** 2;
               begin
                  Result.Truth (Made) := X;
                  Result.On_Table (Made) := On_Table;
                  Result.Points (Made) :=
                    (Position => X + [Own_Sigma * Gaussian (G), Own_Sigma * Gaussian (G),
                                      Own_Sigma * Gaussian (G)],
                     Own      => [[Own, 0.0, 0.0], [0.0, Own, 0.0], [0.0, 0.0, Own]],
                     U0       => Ua,
                     V0       => Va);
                  Result.Sightings (Made) :=
                    (Point => Made, Pose => 1,
                     U     => (if Wrong then 640.0 * Uniform (G) else U + Noise * Gaussian (G)),
                     V     => (if Wrong then 480.0 * Uniform (G) else V + Noise * Gaussian (G)));
               end;
            end if;
         end;
      end loop;
      return Result;
   end Draw;

   --  The covariance entry of terms A and B.
   function Entry_Of (R : Fit_Report; A, B : Positive) return Real is
     (R.Covariance.Element (R.Covariance.First_Index + (A - 1) * Terms + B - 1));

   --  How many sigmas away from the truth a fit lies: the error of its terms against its own covariance, as a
   --  Gaussian deviate of the same tail as the chi square of the whole error (Z is the most it may be).
   function Deviate (R : Fit_Report; L : Fit.Lens; T : Rigid) return Real is
      E     : Real_Vector (1 .. Terms) := [others => 0.0];
      Turn  : constant Vec3 := Driver.Numerics.Log (Transpose (T.Rotation) * R.Pose.Rotation);
      Free  : Natural := 0;
      Index : array (1 .. Terms) of Positive := [others => 1];
   begin
      E (1) := Ln (R.L.Fx / L.Fx);
      E (2) := Ln (R.L.Fy / L.Fy);
      E (3) := R.L.Cx - L.Cx;
      E (4) := R.L.Cy - L.Cy;
      E (5) := R.L.K1 - L.K1;
      E (6) := R.L.K2 - L.K2;
      for A in 1 .. 3 loop
         E (6 + A) := Turn (A);
         E (9 + A) := R.Pose.Translation (A) - T.Translation (A);
      end loop;
      for A in 1 .. Terms loop
         if Entry_Of (R, A, A) > 0.0 then
            Free := Free + 1;
            Index (Free) := A;
         end if;
      end loop;
      if Free = 0 then
         return Real'Last;
      end if;
      declare
         C       : Real_Matrix (1 .. Free, 1 .. Free);
         D       : Real_Vector (1 .. Free);
         Values  : Real_Vector (1 .. Free);
         Vectors : Real_Matrix (1 .. Free, 1 .. Free);
         Square  : Real := 0.0;
      begin
         for A in 1 .. Free loop
            D (A) := E (Index (A));
            for B in 1 .. Free loop
               C (A, B) := (Entry_Of (R, Index (A), Index (B)) + Entry_Of (R, Index (B), Index (A))) / 2.0;
            end loop;
         end loop;
         Eigensystem (C, Values, Vectors);
         for K in 1 .. Free loop
            declare
               Along : Real := 0.0;
            begin
               for A in 1 .. Free loop
                  Along := Along + Vectors (A, K) * D (A);
               end loop;
               if Values (K) <= 0.0 then
                  return Real'Last;
               end if;
               Square := Square + Along ** 2 / Values (K);
            end;
         end loop;
         return Driver.Distributions.Chi_Square_Deviate (Real'Max (1.0e-12, Square), Free);
      end;
   end Deviate;

   Nothing : constant Real_Matrix (1 .. 0, 1 .. 0) := [others => [others => 0.0]];

   --  The start from the table and the fit, as the driver makes them; Found says whether the table gave a start.
   procedure Measure
     (S          : Scene;
      Report     : out Fit_Report;
      Start_Lens : out Fit.Lens;
      Found      : out Boolean;
      Common     : Real_Matrix := Nothing;
      Common_Cov : Real_Matrix := Nothing)
   is
      Poses : constant Pose_Array := [1 => Identity];
      Pose  : Rigid;
   begin
      Report := (others => <>);
      Start (S.Points, Poses, S.Sightings, S.On_Table, Plane_A, Start_Lens, Pose, Found);
      if Found then
         Fit_Eye (S.Points, Poses, S.Sightings, Common, Common_Cov, 640, 480, Start_Lens, Pose, Report);
      end if;
   end Measure;

   function Image (R : Fit_Report) return String is
     (Real'Image (R.L.Fx) & " x" & Real'Image (R.L.Fy) & " c (" & Real'Image (R.L.Cx) & "," & Real'Image (R.L.Cy)
      & "), used" & R.Used'Image & " of" & R.Offered'Image & ": " & Ada.Strings.Unbounded.To_String (R.Why));


   --  What a test found, for the log.
   procedure Say (What : String; R : Fit_Report; L : Fit.Lens; T : Rigid) is
   begin
      Driver.Log.Line (Driver.Log.Robot, "fixed eye test (" & What & "): "
                       & (if R.Determined then "determined" else "not determined") & ", " & Image (R)
                       & (if R.Covariance.Is_Empty then ""
                          else "; " & Driver.Log.Image (Deviate (R, L, T), 2) & " sigmas off the truth"));
   end Say;
   ---------------------------------------------------------------------------
   --  Tests

   --  A fifth of the answers false, a lens whose principal point lies 32 and 34 pixels from the middle: the start
   --  (the table's homography and the parallax of the points off the table, which give the whole camera and take
   --  the principal point from the data, not from the middle) is rough, and the fit finds the lens from it, with
   --  its covariance holding the truth within Z.
   procedure Recovers_The_Lens_And_Pose is
      G : Generator;
      S : constant Scene := Draw (G, Table => 90, Box => 70, Wrong_Share => 0.2, Noise => 0.3, Own_Sigma => 0.002);
      R : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
   begin
      Measure (S, R, Start_Lens, Found);
      Say ("recover", R, Eye_Lens, Eye_Pose);
      Check (Found, "no start from the table's view");
      Check (R.Determined, "the eye is not determined: " & Image (R));
      if R.Determined then
         declare
            Away : constant Real := Deviate (R, Eye_Lens, Eye_Pose);
         begin
            Check (Away <= Driver.Conventions.Z,
                   "the eye is" & Real'Image (Away) & " sigmas off the truth: " & Image (R));
            Check (abs (R.L.Cx - Eye_Lens.Cx) <= Driver.Conventions.Z * Sqrt (Entry_Of (R, 3, 3))
                   and then abs (R.L.Cy - Eye_Lens.Cy) <= Driver.Conventions.Z * Sqrt (Entry_Of (R, 4, 4)),
                   "the principal point is not found: " & Image (R));
            Check (not R.Distorted, "a lens without distortion was given distortion terms");
            Check (R.Used >= S.Count / 2,
                   "far fewer than the true answers fit:" & R.Used'Image & " of" & S.Count'Image);
         end;
      end if;
   end Recovers_The_Lens_And_Pose;

   --  Too few true answers measure nothing: a handful, or a few among many false ones. The eye is not determined,
   --  or, if it is, it is no further from the truth than its own covariance says.
   procedure Too_Few_Answers is
      G : Generator;
      Handful : constant Scene := Draw (G, Table => 3, Box => 2, Wrong_Share => 0.0, Noise => 0.3, Own_Sigma => 0.002);
      Drowned : constant Scene :=
        Draw (G, Table => 80, Box => 80, Wrong_Share => 0.9, Noise => 0.3, Own_Sigma => 0.002);
      R : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
   begin
      Measure (Handful, R, Start_Lens, Found);
      Say ("handful", R, Eye_Lens, Eye_Pose);
      Check (not R.Determined, "five true answers determine an eye: " & Image (R));
      Measure (Drowned, R, Start_Lens, Found);
      Say ("drowned", R, Eye_Lens, Eye_Pose);
      Check (not R.Determined or else Deviate (R, Eye_Lens, Eye_Pose) <= Driver.Conventions.Z,
             "an eye of a tenth true answers is determined and wrong: " & Image (R));
   end Too_Few_Answers;

   --  Points all on one plane fix the eye's pose only for a focal length of its choosing: the focal length trades
   --  against the pose, and the points' own uncertainty leaves it as large as itself. The eye is not determined,
   --  where the same eye with boxes on the table is.
   procedure Table_Alone_Is_Not_Enough is
      G : Generator;
      Flat : constant Scene := Draw (G, Table => 160, Box => 0, Wrong_Share => 0.2, Noise => 0.3, Own_Sigma => 0.002);
      R : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
      --  A start near the truth, for the fit to run from though the points give none.
      Near_Lens : constant Fit.Lens :=
        (Fx => 1.05 * Eye_Lens.Fx, Fy => 0.95 * Eye_Lens.Fy, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Near_Pose : constant Rigid :=
        (Rotation    => Eye_Pose.Rotation * Driver.Numerics.Exp ([0.01, -0.01, 0.005]),
         Translation => Eye_Pose.Translation + [0.05, -0.05, 0.03]);
   begin
      Measure (Flat, R, Start_Lens, Found);
      Check (not Found, "points all on the table give a start: they show no parallax to complete the camera");
      Fit_Eye (Flat.Points, [1 => Identity], Flat.Sightings, Nothing, Nothing, 640, 480, Near_Lens, Near_Pose, R);
      Say ("plane", R, Eye_Lens, Eye_Pose);
      Check (not R.Determined, "an eye is determined by points on one plane: " & Image (R));
   end Table_Alone_Is_Not_Enough;

   --  A lens with radial distortion is given it, within Z of its own covariance.
   procedure Distortion_When_Significant is
      G : Generator;
      S : constant Scene :=
        Draw (G, Table => 90, Box => 70, Wrong_Share => 0.2, Noise => 0.3, Own_Sigma => 0.002, L => Wide_Lens);
      R : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
   begin
      Measure (S, R, Start_Lens, Found);
      Say ("distortion", R, Wide_Lens, Eye_Pose);
      Check (R.Determined, "the distorted eye is not determined: " & Image (R));
      Check (R.Distorted, "a lens with distortion was not given distortion terms: " & Image (R));
      if R.Determined then
         Check (Deviate (R, Wide_Lens, Eye_Pose) <= Driver.Conventions.Z,
                "the distorted eye is" & Real'Image (Deviate (R, Wide_Lens, Eye_Pose)) & " sigmas off the truth: "
                & Image (R));
      end if;
   end Distortion_When_Significant;

   --  The points all share an error: the arm fit's depth scale, one factor on every point's position. The
   --  covariance that carries it holds the truth within Z; the one that does not, on the same data, is too sure.
   procedure Common_Error_Is_Carried is
      G : Generator;
      S : Scene := Draw (G, Table => 90, Box => 70, Wrong_Share => 0.2, Noise => 0.3, Own_Sigma => 0.002);
      Scale : constant Real := 0.02;     --  every point is that much too far
      Rows  : Real_Matrix (1 .. 3 * S.Count, 1 .. 1);
      Cov   : constant Real_Matrix (1 .. 1, 1 .. 1) := [[0.02 ** 2]];
      With_It, Without : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
   begin
      for I in 1 .. S.Count loop
         S.Points (I).Position := (1.0 + Scale) * S.Points (I).Position;
         for A in 1 .. 3 loop
            Rows (3 * (I - 1) + A, 1) := S.Points (I).Position (A);
         end loop;
      end loop;
      Measure (S, With_It, Start_Lens, Found, Rows, Cov);
      Measure (S, Without, Start_Lens, Found);
      Say ("common, carried", With_It, Eye_Lens, Eye_Pose);
      Say ("common, left out", Without, Eye_Lens, Eye_Pose);
      Check (With_It.Determined, "the eye is not determined: " & Image (With_It));
      if With_It.Determined then
         Check (Deviate (With_It, Eye_Lens, Eye_Pose) <= Driver.Conventions.Z,
                "the eye whose points share an error is" & Real'Image (Deviate (With_It, Eye_Lens, Eye_Pose))
                & " sigmas off the truth, though the covariance carries it");
      end if;
      Check (not Without.Determined or else Deviate (Without, Eye_Lens, Eye_Pose) > Driver.Conventions.Z,
             "the covariance that leaves the points' common error out holds the truth within Z: the scene does not "
             & "show what carrying it does");
   end Common_Error_Is_Carried;

   --  Points that ride on a link: a hand's features seen at twelve poses of the arm, the eye's lens and pose found
   --  from them as from points at rest.
   procedure Points_On_A_Link is
      G      : Generator;
      Pose   : constant Rigid := Eye_Pose;
      Count  : constant := 24;
      Poses_Made : constant := 12;
      Poses  : Pose_Array (1 .. Poses_Made);
      Points : Point_Array (1 .. Count);
      Seen   : Sighting_Array (1 .. Count * Poses_Made);
      Made   : Natural := 0;
      R      : Fit_Report;
      Start  : constant Rigid :=
        (Rotation    => Pose.Rotation * Driver.Numerics.Exp ([0.08, -0.06, 0.05]),
         Translation => Pose.Translation + [0.15, -0.1, 0.12]);
      Lens0  : constant Fit.Lens := (Fx => 300.0, Fy => 300.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      Around : constant Vec3 := 0.4 * Table_X + 0.7 * Table_Y + (Table_O + 0.25) * Table_N;   --  above the table
   begin
      for K in Poses'Range loop
         Poses (K) :=
           (Rotation    => Driver.Numerics.Exp ([0.5 * Gaussian (G), 0.5 * Gaussian (G), 0.5 * Gaussian (G)]),
            Translation => Around + [0.25 * Gaussian (G), 0.2 * Gaussian (G), 0.2 * Gaussian (G)]);
      end loop;
      for I in Points'Range loop
         Points (I) := (Position => [0.12 * Gaussian (G), 0.12 * Gaussian (G), 0.12 * Gaussian (G)],
                        Own => [others => [others => 0.0]], U0 => 0.0, V0 => 0.0);
         for K in Poses'Range loop
            declare
               U, V   : Real;
               Inside : Boolean;
            begin
               Seen_By (Eye_Lens, Pose, Poses (K) * Points (I).Position, U, V, Inside);
               if Inside then
                  Made := Made + 1;
                  Seen (Made) := (Point => I, Pose => K, U => U + 0.2 * Gaussian (G), V => V + 0.2 * Gaussian (G));
               end if;
            end;
         end loop;
      end loop;
      Fit_Eye (Points, Poses, Seen (1 .. Made), Nothing, Nothing, 640, 480, Lens0, Start, R);
      Say ("link", R, Eye_Lens, Pose);
      Check (R.Determined, "an eye measured from points on a link is not determined: " & Image (R));
      if R.Determined then
         Check (Deviate (R, Eye_Lens, Pose) <= Driver.Conventions.Z,
                "the eye measured from points on a link is" & Real'Image (Deviate (R, Eye_Lens, Pose))
                & " sigmas off the truth: " & Image (R));
      end if;
   end Points_On_A_Link;

   --  A track the arm fit leaves with no depth at all has an own covariance whose quadratic form, summed term by term,
   --  is rounding (A17's first fit had one at 1.7E97 units, its log depth uncertain by 2.5E6: the variance it made of
   --  a pixel came out minus four, and the square root of the scale it made of the residual raised); and one with a
   --  negative eigenvalue (the arm fit's own errors can make it) makes a negative one. A dozen of the points off the
   --  table have the one or the other here; the eye is measured all the same, from the rest, within Z of its
   --  covariance.
   procedure Points_Of_Great_Or_Negative_Covariance is
      G : Generator;
      S : Scene := Draw (G, Table => 90, Box => 70, Wrong_Share => 0.2, Noise => 0.3, Own_Sigma => 0.002);
      R : Fit_Report;
      Start_Lens : Fit.Lens;
      Found : Boolean;
      Changed : Natural := 0;
   begin
      for I in S.Points'Range loop
         if not S.On_Table (I) and then Changed < 12 then
            Changed := Changed + 1;
            S.Points (I).Own :=
              (if Changed mod 2 = 0
               then [[-0.01, 0.0, 0.0], [0.0, -0.01, 0.0], [0.0, 0.0, -0.01]]
               else 1.0e18 * Driver.Numerics.Outer (S.Points (I).Position, S.Points (I).Position));
         end if;
      end loop;
      Check (Changed = 12, "fewer than a dozen points off the table to give a covariance to:" & Changed'Image);
      Measure (S, R, Start_Lens, Found);
      Say ("great and negative covariances", R, Eye_Lens, Eye_Pose);
      Check (Found, "no start from the table's view");
      Check (R.Determined, "the eye is not determined: " & Image (R));
      if R.Determined then
         Check (Deviate (R, Eye_Lens, Eye_Pose) <= Driver.Conventions.Z,
                "the eye is" & Real'Image (Deviate (R, Eye_Lens, Eye_Pose)) & " sigmas off the truth: " & Image (R));
      end if;
   end Points_Of_Great_Or_Negative_Covariance;

   --  A fit whose cost is not quadratic over its own sigma has a covariance that does not stand, however small
   --  every sigma it states: it must find that out itself and say so. At three pixels of noise on the answers (ten
   --  times the noise the other scenes have, every answer true), some of the scenes drawn in turn come out with every
   --  criterion that rests on the sigmas passing, and the cost still rising by less than Z - 1 sigmas' worth where
   --  the covariance puts Z of them: such a fit is not determined, and says that is why.
   procedure Quadratic_Over_Its_Own_Sigma is
      G : Generator;
      Rejected : Natural := 0;
   begin
      for Scene_Number in 1 .. 30 loop
         declare
            S : constant Scene :=
              Draw (G, Table => 90, Box => 70, Wrong_Share => 0.0, Noise => 3.0, Own_Sigma => 0.002);
            R : Fit_Report;
            Start_Lens : Fit.Lens;
            Found : Boolean;
         begin
            Measure (S, R, Start_Lens, Found);
            if Found and then not R.Covariance.Is_Empty then
               Check (not R.Determined or else R.Linear_To >= Driver.Conventions.Z - 1.0,
                      "scene" & Scene_Number'Image & " is determined though its cost rises as for"
                      & Real'Image (R.Linear_To) & " sigmas where its covariance puts Z");
               --  Rejected for that reason alone: the only reason the fit gives (every criterion that rests on
               --  its sigmas passed, and the old fit would have been called determined).
               if Ada.Strings.Unbounded.Index (R.Why, "the fit is not quadratic") = 1
                 and then Ada.Strings.Unbounded.Index (R.Why, "; ") = 0
               then
                  Rejected := Rejected + 1;
                  Check (not R.Determined, "scene" & Scene_Number'Image & " is rejected and determined");
               end if;
            end if;
         end;
      end loop;
      Check (Rejected > 0, "no fit of thirty at three pixels was found not quadratic over its own sigma");
   end Quadratic_Over_Its_Own_Sigma;

   procedure Register is
   begin
      Register ("robot.fixed.recover",
                "an eye with a lens of its own, its principal point far from the middle and a fifth of its answers "
                & "false, is not measured within Z of its own covariance from the start the table and the "
                & "parallax of the other points give", Recovers_The_Lens_And_Pose'Access);
      Register ("robot.fixed.few",
                "an eye is determined by a handful of true answers, or is determined and wrong among answers nine in "
                & "ten of which are false", Too_Few_Answers'Access);
      Register ("robot.fixed.plane",
                "an eye is determined by points that all lie on one plane, though they leave its focal length free",
                Table_Alone_Is_Not_Enough'Access);
      Register ("robot.fixed.distortion",
                "a lens with radial distortion is not given it, or is off the truth by more than Z of its covariance",
                Distortion_When_Significant'Access);
      Register ("robot.fixed.common",
                "an error every point shares, as the arm fit's depth scale, is left out of the eye's covariance: the "
                & "truth lies beyond Z of it", Common_Error_Is_Carried'Access);
      Register ("robot.fixed.link",
                "an eye is not measured from points that ride on a link of an arm, seen at several poses of it",
                Points_On_A_Link'Access);
      Register ("robot.fixed.psd",
                "a point whose own covariance has a negative eigenvalue, or is so great that its quadratic form is "
                & "rounding, stops the fit of an eye (the scale of its residual is a square root of a negative number)",
                Points_Of_Great_Or_Negative_Covariance'Access);
      Register ("robot.fixed.quadratic",
                "a fit whose cost is not quadratic over its own sigma is called determined, though its covariance "
                & "does not stand", Quadratic_Over_Its_Own_Sigma'Access);
   end Register;

end Driver.Robot.Kinematics.Fixed.Tests;
