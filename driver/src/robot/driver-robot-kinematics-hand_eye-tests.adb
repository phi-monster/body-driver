with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Tests;
with GNAT.OS_Lib;

package body Driver.Robot.Kinematics.Hand_Eye.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   --  A fit from no prior: every arm starts from the search (the driver's own fits pass the eye's last one).
   procedure Fit_Eye (Arms : Watched_Array; Width, Height : Positive; Report : out Fit_Report) is
      Nothing : Fit_Report;
   begin
      Hand_Eye.Fit_Eye (Arms, Width, Height, Nothing, Report);
   end Fit_Eye;

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
   --  The world: an arm of six joints standing in its own frame, a fixed eye that watches it from a metre and a half
   --  away with a lens of its own whose principal point is far from the middle of its picture, and points that ride
   --  on the arm's links (and some that stand still), seen by the eye at the arm's keyframes.

   Width  : constant := 640;
   Height : constant := 480;

   Joint_Count : constant := 6;

   Eye_Lens  : constant Fit.Lens := (Fx => 300.0, Fy => 294.0, Cx => 331.0, Cy => 247.0, K1 => 0.0, K2 => 0.0);
   Wide_Lens : constant Fit.Lens := (Fx => 300.0, Fy => 294.0, Cx => 331.0, Cy => 247.0, K1 => -0.15, K2 => 0.0);

   --  A serial arm: a turn about the vertical at its base, three pitches and two rolls along it.
   Arm_Joints : constant Fit.Joint_Array (1 .. Joint_Count) :=
     [(W => [0.0, 0.0, 1.0], P => [0.00, 0.0, 0.00], C => 1.0, Slide => False),
      (W => [0.0, 1.0, 0.0], P => [0.00, 0.0, 0.15], C => 1.0, Slide => False),
      (W => [0.0, 1.0, 0.0], P => [0.25, 0.0, 0.15], C => 1.0, Slide => False),
      (W => [1.0, 0.0, 0.0], P => [0.25, 0.0, 0.15], C => 1.0, Slide => False),
      (W => [0.0, 1.0, 0.0], P => [0.45, 0.0, 0.15], C => 1.0, Slide => False),
      (W => [1.0, 0.0, 0.0], P => [0.45, 0.0, 0.15], C => 1.0, Slide => False)];

   --  Where the points of each link are (at the reference readings), and the points that stand still.
   Link_Centre : constant array (1 .. Joint_Count) of Vec3 :=
     [[0.05, 0.00, 0.10], [0.12, 0.04, 0.20], [0.25, 0.05, 0.22],
      [0.35, -0.05, 0.15], [0.45, 0.05, 0.20], [0.52, 0.00, 0.15]];
   Still_Centre : constant Vec3 := [0.30, 0.12, 0.02];

   --  The eye's camera frame looking from a place at another: x to the right, y down, z forward.
   function Looking (From, At_Point : Vec3) return Rigid is
      Z_Axis : constant Vec3 := Unit (At_Point - From);
      X_Axis : constant Vec3 := Unit (Cross (Z_Axis, [0.0, 0.0, 1.0]));
      Y_Axis : constant Vec3 := Cross (Z_Axis, X_Axis);
   begin
      return (Rotation    => [[X_Axis (1), Y_Axis (1), Z_Axis (1)],
                              [X_Axis (2), Y_Axis (2), Z_Axis (2)],
                              [X_Axis (3), Y_Axis (3), Z_Axis (3)]],
              Translation => From);
   end Looking;

   --  The room: the first arm's frame. The second arm stands across from the first, turned half a turn about the
   --  vertical, a little to the side.
   Second_In_Room : constant Rigid :=
     (Rotation => [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], Translation => [0.9, 0.3, 0.0]);

   Eye_In_Room : constant Rigid := Looking ([0.45, -1.4, 0.8], [0.45, 0.15, 0.18]);

   function Eye_In_Arm (Arm : Positive) return Rigid is
     (if Arm = 1 then Eye_In_Room else Inverse (Second_In_Room) * Eye_In_Room);

   --  Where an eye of that lens and pose sees a point of the arm's frame, if in front and in its picture.
   procedure Seen_By (L : Fit.Lens; T : Rigid; X : Vec3; U, V : out Real; Inside : out Boolean) is
      Ahead : Boolean;
   begin
      Fit.Project (L, Transpose (T.Rotation) * (X - T.Translation), U, V, Ahead);
      Inside := Ahead and then U in 0.0 .. Real (Width) and then V in 0.0 .. Real (Height);
   end Seen_By;

   type Link_Truth is array (Positive range <>) of Natural;
   type Link_Matrix is array (Positive range <>, Positive range <>) of Natural;

   --  What a scene is made of: each arm as Fit_Eye is given it, and the truth.
   type Scene (Arms, Tracks : Positive) is record
      Watching : Watched_Array (1 .. Arms);
      Link     : Link_Matrix (1 .. Arms, 1 .. Tracks);   --  the link every track rides on (0: none)
      Pose     : Pose_Array (1 .. Arms);
   end record;

   procedure Free_Watched is new Ada.Unchecked_Deallocation (Watched, Watched_Access);

   procedure Release (S : in out Scene) is
   begin
      for W of S.Watching loop
         Free_Watched (W);
      end loop;
   end Release;

   --  The arm's keyframes: the reference, three of each joint alone (so the links can be read), and the rest at
   --  readings all joints share in.
   function Keyframes (G : in out Generator; Random_Frames : Natural) return Real_Matrix is
      Alone  : constant Real_Array (1 .. 3) := [0.25, -0.25, 0.4];
      Frames : constant Positive := 1 + Joint_Count * Alone'Length + Random_Frames;
      Result : Real_Matrix (1 .. Frames, 1 .. Joint_Count) := [others => [others => 0.0]];
      Row    : Positive := 1;
   begin
      for J in 1 .. Joint_Count loop
         for A of Alone loop
            Row := Row + 1;
            Result (Row, J) := A;
         end loop;
      end loop;
      for K in 1 .. Random_Frames loop
         Row := Row + 1;
         for J in 1 .. Joint_Count loop
            Result (Row, J) := (2.0 * Uniform (G) - 1.0) * 0.35;
         end loop;
      end loop;
      return Result;
   end Keyframes;

   --  How the arm's joints are known to the fit: the truth, or the truth moved by errors of the sizes Tilt (radians),
   --  Shift (the arm's unit) and Rate (a fraction of a joint's scale), the covariance the arm's fit would have of
   --  them stated alongside when Stated.
   procedure Know_Joints
     (G                : in out Generator;
      Tilt, Shift, Rate : Real;
      Joint            : out Fit.Joint_Array;
      Covariance       : in out Fit.Real_Lists.Vector;
      Stated           : Boolean)
   is
      Terms : constant Natural := Fit.Joint_Terms * Joint'Length;
      Var   : Real_Array (1 .. Terms) := [others => 0.0];
   begin
      Joint := Arm_Joints;
      for J in Joint'Range loop
         declare
            E1, E2 : Vec3;
            O      : constant Natural := Fit.Joint_Terms * (J - 1);
         begin
            Fit.Across (Joint (J).W, E1, E2);
            Joint (J).W := Unit (Joint (J).W + Tilt * Gaussian (G) * E1 + Tilt * Gaussian (G) * E2);
            Joint (J).P := Joint (J).P + Shift * Gaussian (G) * E1 + Shift * Gaussian (G) * E2;
            Joint (J).C := Joint (J).C * (1.0 + Rate * Gaussian (G));
            Var (O + 1) := Tilt ** 2;
            Var (O + 2) := Tilt ** 2;
            Var (O + 3) := Shift ** 2;
            Var (O + 4) := Shift ** 2;
            Var (O + 5) := (Rate * Arm_Joints (J).C) ** 2;
         end;
      end loop;
      Covariance.Clear;
      if Stated then
         for A in 1 .. Terms loop
            for B in 1 .. Terms loop
               Covariance.Append ((if A = B then Var (A) else 0.0));
            end loop;
         end loop;
      end if;
   end Know_Joints;

   --  Where a point at rest at the arm's reference readings is at keyframe F of the arm's readings Change, when it rides on
   --  link Link (none: it stands still).
   function Moved_Of (Link : Natural; Rest : Vec3; Change : Real_Matrix; F : Positive) return Vec3 is
      Reading : Real_Array (1 .. Link);
   begin
      if Link = 0 then
         return Rest;
      end if;
      for J in Reading'Range loop
         Reading (J) := Change (F, J);
      end loop;
      return Fit.Eye_At (Arm_Joints (1 .. Link), [for I in 1 .. Link => I], Reading) * Rest;
   end Moved_Of;

   --  Tracks on the arms' links and standing still, seen by the eye at the arm's keyframes: a share of the answers
   --  false (a pixel anywhere in the picture), the others true to Noise pixels, each stating Noise as its own sigma.
   --  The arms are known to the fit as Know_Joints says.
   function Draw
     (G             : in out Generator;
      Arms          : Positive;
      Tracks        : Positive;
      Random_Frames : Natural;
      Wrong_Share   : Real;
      Noise         : Real;
      L             : Fit.Lens := Eye_Lens;
      Still_Share   : Real := 0.15;
      Tilt          : Real := 0.0;
      Shift         : Real := 0.0;
      Rate          : Real := 0.0;
      Stated        : Boolean := False) return Scene
   is
      Result : Scene (Arms, Tracks);
   begin
      for A in 1 .. Arms loop
         declare
            Change   : constant Real_Matrix := Keyframes (G, Random_Frames);
            Frames   : constant Positive := Change'Length (1);
            Eye      : constant Rigid := Eye_In_Arm (A);
            Rest     : array (1 .. Tracks) of Vec3;
            Track    : Track_Array (1 .. Tracks);
            Seen     : Sighting_Array (1 .. Tracks * (Frames - 1));
            Made     : Natural := 0;
            Visible  : constant Real_Vector (1 .. Joint_Count) := [others => 0.02];
            Count    : Natural := 0;
            Kept     : Link_Truth (1 .. Tracks) := [others => 0];
            Joint    : Fit.Joint_Array (1 .. Joint_Count);
            Cov      : Fit.Real_Lists.Vector;
         begin
            while Count < Tracks loop
               declare
                  Still : constant Boolean := Uniform (G) < Still_Share;
                  Lnk   : constant Natural := (if Still then 0 else 1 + Natural (Real'Floor (6.0 * Uniform (G))) mod 6);
                  X0    : constant Vec3 :=
                    (if Still then Still_Centre else Link_Centre (Lnk))
                    + (if Still then 0.12 else 0.06) * [Gaussian (G), Gaussian (G), Gaussian (G)];
                  U0, V0 : Real;
                  Inside : Boolean;
               begin
                  Seen_By (L, Eye, X0, U0, V0, Inside);
                  if Inside then
                     Count := Count + 1;
                     Rest (Count) := X0;
                     Kept (Count) := Lnk;
                     Track (Count) := (U0 => U0, V0 => V0);
                  end if;
               end;
            end loop;
            for T in 1 .. Tracks loop
               for F in 2 .. Frames loop
                  declare
                     U, V    : Real;
                     Inside  : Boolean;
                  begin
                     Seen_By (L, Eye, Moved_Of (Kept (T), Rest (T), Change, F), U, V, Inside);
                     if Inside then
                        Made := Made + 1;
                        declare
                           Wrong : constant Boolean := Uniform (G) < Wrong_Share;
                        begin
                           Seen (Made) :=
                             (Track       => T,
                              Frame       => F,
                              U           => (if Wrong then Real (Width) * Uniform (G) else U + Noise * Gaussian (G)),
                              V           => (if Wrong then Real (Height) * Uniform (G) else V + Noise * Gaussian (G)),
                              Own         => Noise,
                              Own_Freedom => 0);
                        end;
                     end if;
                  end;
               end loop;
            end loop;
            Know_Joints (G, Tilt, Shift, Rate, Joint, Cov, Stated);
            Result.Watching (A) := new Watched (Joints => Joint_Count, Frames => Frames, Tracks => Tracks,
                                                Sightings => Made);
            Result.Watching (A).Joint := Joint;
            Result.Watching (A).Change := Change;
            Result.Watching (A).Visible := Visible;
            Result.Watching (A).Covariance := Cov;
            Result.Watching (A).Track := Track;
            Result.Watching (A).Seen := Seen (1 .. Made);
            for T in 1 .. Tracks loop
               Result.Link (A, T) := Kept (T);
            end loop;
            Result.Pose (A) := Eye;
         end;
      end loop;
      return Result;
   end Draw;

   ---------------------------------------------------------------------------
   --  What a fit is judged by

   --  The covariance entry of terms A and B.
   function Entry_Of (R : Fit_Report; A, B : Positive) return Real is
     (R.Covariance.Element (R.Covariance.First_Index + (A - 1) * Terms (Natural (R.Arms.Length)) + B - 1));

   --  The chi square of the fit's error against its own covariance over the terms the covariance holds (Free of them;
   --  none when it holds none). Square is Real'Last when the covariance is not positive.
   procedure Statistic (R : Fit_Report; L : Fit.Lens; Poses : Pose_Array; Square : out Real; Free : out Natural) is
      Arms  : constant Positive := Poses'Length;
      Total : constant Positive := Terms (Arms);
      E     : Real_Vector (1 .. Total) := [others => 0.0];
      Index : array (1 .. Total) of Positive := [others => 1];
   begin
      Square := Real'Last;
      Free := 0;
      if R.Covariance.Is_Empty then
         return;
      end if;
      E (1) := Ln (R.L.Fx / L.Fx);
      E (2) := Ln (R.L.Fy / L.Fy);
      E (3) := R.L.Cx - L.Cx;
      E (4) := R.L.Cy - L.Cy;
      E (5) := R.L.K1 - L.K1;
      E (6) := R.L.K2 - L.K2;
      for A in 1 .. Arms loop
         declare
            Turn : constant Vec3 :=
              Driver.Numerics.Log (Transpose (Poses (Poses'First + A - 1).Rotation) * R.Arms (A).Pose.Rotation);
            O    : constant Natural := Fit.Lens_Terms + Pose_Terms * (A - 1);
         begin
            for C in 1 .. 3 loop
               E (O + C) := Turn (C);
               E (O + 3 + C) := R.Arms (A).Pose.Translation (C) - Poses (Poses'First + A - 1).Translation (C);
            end loop;
         end;
      end loop;
      for A in 1 .. Total loop
         if Entry_Of (R, A, A) > 0.0 then
            Free := Free + 1;
            Index (Free) := A;
         end if;
      end loop;
      if Free = 0 then
         return;
      end if;
      declare
         C       : Real_Matrix (1 .. Free, 1 .. Free);
         D       : Real_Vector (1 .. Free);
         Values  : Real_Vector (1 .. Free);
         Vectors : Real_Matrix (1 .. Free, 1 .. Free);
         Sum     : Real := 0.0;
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
                  return;
               end if;
               Sum := Sum + Along ** 2 / Values (K);
            end;
         end loop;
         Square := Sum;
      end;
   end Statistic;

   function Deviate (R : Fit_Report; L : Fit.Lens; Poses : Pose_Array) return Real is
      Square : Real;
      Free   : Natural;
   begin
      Statistic (R, L, Poses, Square, Free);
      if Free = 0 or else Square = Real'Last then
         return Real'Last;
      end if;
      return Driver.Distributions.Chi_Square_Deviate (Real'Max (1.0e-12, Square), Free);
   end Deviate;

   --  The lens as a fit has it, with its sigmas where it has a covariance.
   function Image (R : Fit_Report) return String is
     (Driver.Log.Image (R.L.Fx, 2) & " x" & Driver.Log.Image (R.L.Fy, 2) & " c (" & Driver.Log.Image (R.L.Cx, 2) & ","
      & Driver.Log.Image (R.L.Cy, 2) & ")"
      & (if R.Covariance.Is_Empty then ""
         else " +-" & Driver.Log.Image (R.L.Fx * Sqrt (Entry_Of (R, 1, 1)), 2) & " x"
              & Driver.Log.Image (R.L.Fy * Sqrt (Entry_Of (R, 2, 2)), 2) & " c +-"
              & Driver.Log.Image (Sqrt (Entry_Of (R, 3, 3)), 2) & "," & Driver.Log.Image (Sqrt (Entry_Of (R, 4, 4)), 2))
      & ", used" & R.Used'Image & " of" & R.Offered'Image & ": " & Ada.Strings.Unbounded.To_String (R.Why));

   --  What a test found, for the log.
   procedure Say (What : String; R : Fit_Report; L : Fit.Lens; Poses : Pose_Array) is
   begin
      Driver.Log.Line (Driver.Log.Robot, "hand eye test (" & What & "): "
                       & (if R.Determined then "determined" else "not determined") & ", " & Image (R)
                       & (if R.Covariance.Is_Empty then ""
                          else "; " & Driver.Log.Image (Deviate (R, L, Poses), 2) & " sigmas off the truth"));
   end Say;

   ---------------------------------------------------------------------------
   --  Tests

   --  An eye with a lens of its own, its principal point far from the middle, watches an arm move; a twentieth of the
   --  answers are false. The lens and the pose are found from nothing but the arm's motion, within Z of the fit's own
   --  covariance, and the points that stand still are left out.
   procedure Recovers_The_Lens_And_Pose is
      G : Generator;
      S : Scene := Draw (G, Arms => 1, Tracks => 90, Random_Frames => 30, Wrong_Share => 0.05, Noise => 0.3);
      R : Fit_Report;
   begin
      Fit_Eye (S.Watching, Width, Height, R);
      Say ("recover", R, Eye_Lens, S.Pose);
      Check (R.Determined, "the eye is not determined: " & Image (R));
      if R.Determined then
         Check (Deviate (R, Eye_Lens, S.Pose) <= Driver.Conventions.Z,
                "the eye is" & Real'Image (Deviate (R, Eye_Lens, S.Pose)) & " sigmas off the truth: " & Image (R));
         Check (R.Arms (1).Placed, "the arm's eye is not placed");
         Check (not R.Distorted, "a lens without distortion was given distortion terms");
         Check (abs (R.L.Cx - Eye_Lens.Cx) <= Driver.Conventions.Z * Sqrt (Entry_Of (R, 3, 3))
                and then abs (R.L.Cy - Eye_Lens.Cy) <= Driver.Conventions.Z * Sqrt (Entry_Of (R, 4, 4)),
                "the principal point is not found: " & Image (R));
         declare
            Still_Taken, Still_Total, Moving_Lost, Moving_Total : Natural := 0;
         begin
            for T in 1 .. S.Tracks loop
               if S.Link (1, T) = 0 then
                  Still_Total := Still_Total + 1;
                  if R.Arms (1).Link.Element (T) /= 0 then
                     Still_Taken := Still_Taken + 1;
                  end if;
               else
                  Moving_Total := Moving_Total + 1;
                  if R.Arms (1).Link.Element (T) = 0 then
                     Moving_Lost := Moving_Lost + 1;
                  end if;
               end if;
            end loop;
            Check (Still_Taken = 0, "points that stand still were taken to ride on a link:" & Still_Taken'Image
                   & " of" & Still_Total'Image);
            Check (2 * Moving_Lost < Moving_Total, "most of the points that ride on a link were left out:"
                   & Moving_Lost'Image & " of" & Moving_Total'Image);
         end;
      end if;
      Release (S);
   end Recovers_The_Lens_And_Pose;

   --  Two arms through one lens: the second arm stands across from the first and has its own frame, its own readings
   --  and its own points; the eye is one. Both are placed within Z of the covariance, and the lens is held tighter
   --  than the first arm alone holds it.
   procedure Two_Arms_Share_The_Lens is
      G : Generator;
      S : Scene := Draw (G, Arms => 2, Tracks => 70, Random_Frames => 24, Wrong_Share => 0.05, Noise => 0.3);
      R, First_Only : Fit_Report;
   begin
      Fit_Eye (S.Watching, Width, Height, R);
      Fit_Eye (S.Watching (1 .. 1), Width, Height, First_Only);
      Say ("two arms", R, Eye_Lens, S.Pose);
      Say ("first arm alone", First_Only, Eye_Lens, S.Pose (1 .. 1));
      Check (R.Determined, "two arms do not determine the eye: " & Image (R));
      if R.Determined then
         Check (R.Arms (1).Placed and then R.Arms (2).Placed, "an arm is not placed: " & Image (R));
         Check (Deviate (R, Eye_Lens, S.Pose) <= Driver.Conventions.Z,
                "the eye is" & Real'Image (Deviate (R, Eye_Lens, S.Pose)) & " sigmas off the truth: " & Image (R));
         if First_Only.Determined then
            Check (Entry_Of (R, 1, 1) < Entry_Of (First_Only, 1, 1)
                   and then Entry_Of (R, 3, 3) < Entry_Of (First_Only, 3, 3),
                   "two arms hold the focal length and the principal point no tighter than one: sigma of Fx"
                   & Real'Image (Sqrt (Entry_Of (R, 1, 1))) & " against"
                   & Real'Image (Sqrt (Entry_Of (First_Only, 1, 1))));
         end if;
      end if;
      Release (S);
   end Two_Arms_Share_The_Lens;

   --  Exact answers: the fit's optimum is the truth. The lens and the pose come back to what the arithmetic of the
   --  residuals and their derivatives allows, which shows the derivatives are right where noise would hide it.
   procedure Exact_Answers_Are_Exact is
      G : Generator;
      S : Scene := Draw (G, Arms => 1, Tracks => 60, Random_Frames => 24, Wrong_Share => 0.0, Noise => 1.0e-6);
      R : Fit_Report;
   begin
      Fit_Eye (S.Watching, Width, Height, R);
      Say ("exact", R, Eye_Lens, S.Pose);
      Check (abs (R.L.Fx / Eye_Lens.Fx - 1.0) < 1.0e-5 and then abs (R.L.Fy / Eye_Lens.Fy - 1.0) < 1.0e-5
             and then abs (R.L.Cx - Eye_Lens.Cx) < 1.0e-3 and then abs (R.L.Cy - Eye_Lens.Cy) < 1.0e-3,
             "exact answers do not give the lens back: " & Image (R));
      Check (abs Driver.Numerics.Log (Transpose (S.Pose (1).Rotation) * R.Arms (1).Pose.Rotation) < 1.0e-5
             and then abs (R.Arms (1).Pose.Translation - S.Pose (1).Translation) < 1.0e-4,
             "exact answers do not give the pose back: " & Image (R));
      Release (S);
   end Exact_Answers_Are_Exact;

   --  A lens with radial distortion is given it, within Z of its own covariance.
   procedure Distortion_When_Significant is
      G : Generator;
      S : Scene := Draw (G, Arms => 1, Tracks => 90, Random_Frames => 30, Wrong_Share => 0.05, Noise => 0.3,
                         L => Wide_Lens);
      R : Fit_Report;
   begin
      Fit_Eye (S.Watching, Width, Height, R);
      Say ("distortion", R, Wide_Lens, S.Pose);
      Check (R.Determined, "the distorted eye is not determined: " & Image (R));
      Check (R.Distorted, "a lens with distortion was not given distortion terms: " & Image (R));
      if R.Determined then
         Check (Deviate (R, Wide_Lens, S.Pose) <= Driver.Conventions.Z,
                "the distorted eye is" & Real'Image (Deviate (R, Wide_Lens, S.Pose)) & " sigmas off the truth: "
                & Image (R));
      end if;
      Release (S);
   end Distortion_When_Significant;

   --  The arm's own joints are known only to the covariance its fit gave them. The eye is fitted at the joints as they
   --  were found, whose errors move every sighting alike and so are not in the residuals: the covariance that carries
   --  them holds the truth within Z, and is wider than the one that does not, on the same sightings and the same joints.
   procedure Arm_Uncertainty_Is_Carried is
      Tilt  : constant Real := 0.02;
      Shift : constant Real := 0.004;
      Rate  : constant Real := 0.005;
      With_It, Without : Fit_Report;
      G1 : Generator;
      G2 : Generator;
      Stated : Scene := Draw (G1, Arms => 1, Tracks => 90, Random_Frames => 30, Wrong_Share => 0.05, Noise => 0.3,
                              Tilt => Tilt, Shift => Shift, Rate => Rate, Stated => True);
      Left_Out : Scene := Draw (G2, Arms => 1, Tracks => 90, Random_Frames => 30, Wrong_Share => 0.05, Noise => 0.3,
                                Tilt => Tilt, Shift => Shift, Rate => Rate, Stated => False);
   begin
      Fit_Eye (Stated.Watching, Width, Height, With_It);
      Fit_Eye (Left_Out.Watching, Width, Height, Without);
      Say ("arm uncertainty, carried", With_It, Eye_Lens, Stated.Pose);
      Say ("arm uncertainty, left out", Without, Eye_Lens, Left_Out.Pose);
      Check (With_It.Determined, "the eye is not determined: " & Image (With_It));
      if With_It.Determined then
         Check (Deviate (With_It, Eye_Lens, Stated.Pose) <= Driver.Conventions.Z,
                "the eye whose arm is uncertain is" & Real'Image (Deviate (With_It, Eye_Lens, Stated.Pose))
                & " sigmas off the truth, though the covariance carries it");
         if Without.Determined then
            Check (Entry_Of (With_It, 1, 1) > Entry_Of (Without, 1, 1) or else Entry_Of (With_It, 7, 7) > Entry_Of (Without, 7, 7),
                   "the covariance that carries the arm's uncertainty is no wider than the one that leaves it out");
         end if;
      end if;
      Release (Stated);
      Release (Left_Out);
   end Arm_Uncertainty_Is_Carried;

   --  Too little to go on: a handful of tracks seen at a couple of keyframes. The eye is not determined, or, if it is,
   --  it is no further from the truth than its own covariance says.
   procedure Too_Little_To_Go_On is
      G : Generator;
      Handful : Scene := Draw (G, Arms => 1, Tracks => 3, Random_Frames => 0, Wrong_Share => 0.0, Noise => 0.3);
      Short   : Scene := Draw (G, Arms => 1, Tracks => 60, Random_Frames => 0, Wrong_Share => 0.0, Noise => 0.3);
      R : Fit_Report;
   begin
      Fit_Eye (Handful.Watching, Width, Height, R);
      Say ("handful", R, Eye_Lens, Handful.Pose);
      Check (not R.Determined or else Deviate (R, Eye_Lens, Handful.Pose) <= Driver.Conventions.Z,
             "three tracks determine an eye, wrongly: " & Image (R));
      Fit_Eye (Short.Watching, Width, Height, R);
      Say ("one keyframe each", R, Eye_Lens, Short.Pose);
      Check (not R.Determined or else Deviate (R, Eye_Lens, Short.Pose) <= Driver.Conventions.Z,
             "an eye is determined and wrong from the arm's single-joint keyframes alone: " & Image (R));
      Release (Handful);
      Release (Short);
   end Too_Little_To_Go_On;

   --  A fit that starts from the eye an earlier fit found, with more keyframes, finds the same eye as one that
   --  searches: the two are within each other's sigmas, and the prior's fit of the same keyframes it was made from
   --  is no different from the search's.
   procedure Prior_Starts_The_Fit is
      G : Generator;
      S : Scene := Draw (G, Arms => 1, Tracks => 80, Random_Frames => 30, Wrong_Share => 0.05, Noise => 0.3);
      Early, Searched, Warm : Fit_Report;
      Whole : constant Watched_Access := S.Watching (1);
      Fewer : Watched_Access;
      Kept  : Natural := 0;
   begin
      --  The first half of the keyframes: those of a fit made before the rest were taken.
      for Seen of Whole.Seen loop
         if Seen.Frame <= Whole.Frames / 2 then
            Kept := Kept + 1;
         end if;
      end loop;
      Fewer := new Watched (Joints => Whole.Joints, Frames => Whole.Frames / 2, Tracks => Whole.Tracks, Sightings => Kept);
      Fewer.Joint := Whole.Joint;
      Fewer.Visible := Whole.Visible;
      Fewer.Track := Whole.Track;
      for F in 1 .. Fewer.Frames loop
         for J in 1 .. Fewer.Joints loop
            Fewer.Change (F, J) := Whole.Change (F, J);
         end loop;
      end loop;
      Kept := 0;
      for Seen of Whole.Seen loop
         if Seen.Frame <= Fewer.Frames then
            Kept := Kept + 1;
            Fewer.Seen (Kept) := Seen;
         end if;
      end loop;
      Fit_Eye ([1 => Fewer], Width, Height, Early);
      Fit_Eye (S.Watching, Width, Height, Searched);
      Fit_Eye (S.Watching, Width, Height, Early, Warm);
      Say ("early", Early, Eye_Lens, S.Pose);
      Say ("searched", Searched, Eye_Lens, S.Pose);
      Say ("warm", Warm, Eye_Lens, S.Pose);
      Check (Early.Determined and then Searched.Determined and then Warm.Determined,
             "a fit is not determined: " & Image (Early) & " | " & Image (Searched) & " | " & Image (Warm));
      if Searched.Determined and then Warm.Determined then
         Check (abs (Warm.L.Fx - Searched.L.Fx) <= Driver.Conventions.Z * Searched.L.Fx * Sqrt (Entry_Of (Searched, 1, 1))
                and then abs (Warm.L.Cx - Searched.L.Cx) <= Driver.Conventions.Z * Sqrt (Entry_Of (Searched, 3, 3)),
                "the fit from a prior is not the fit from a search: " & Image (Warm) & " against " & Image (Searched));
         Check (Deviate (Warm, Eye_Lens, S.Pose) <= Driver.Conventions.Z,
                "the fit from a prior is" & Real'Image (Deviate (Warm, Eye_Lens, S.Pose)) & " sigmas off the truth");
      end if;
      Free_Watched (Fewer);
      Release (S);
   end Prior_Starts_The_Fit;

   Ends_Guard : constant String :=
     "a fit of an eye never ends: its refinement doubles its damping for ever when the matrix it damps is no matrix "
     & "(a sighting at no pixel, an arm that does not move)";

   --  A fit ends whatever it is given: no arm, no tracks, no sightings, a sighting at no pixel, an arm that never
   --  moved. Each in a task that is waited for Bound; a task that spins cannot be stopped, nor the program ended with
   --  it, so a fit that does not end says so as a failed test does, and ends the program.
   procedure Ends_Whatever_It_Is_Given is
      Bound : constant Duration := 120.0;
      G     : Generator;
      S     : Scene := Draw (G, Arms => 1, Tracks => 30, Random_Frames => 8, Wrong_Share => 0.05, Noise => 0.3);
      Still : Scene := Draw (G, Arms => 1, Tracks => 30, Random_Frames => 0, Wrong_Share => 0.0, Noise => 0.3);
      --  Nothing divided by nothing, from a number the compiler does not know to be zero.
      Nought      : constant Real := S.Watching (1).Track (1).U0 - S.Watching (1).Track (1).U0;
      Not_A_Pixel : constant Real := Nought / Nought;

      procedure Fit_In_Time (What : String; Watching : Watched_Array; R : out Fit_Report) is
         Raised : Boolean := True;
         task Fitting is
            entry Done;
         end Fitting;
         task body Fitting is
         begin
            begin
               Fit_Eye (Watching, Width, Height, R);
               Raised := False;
            exception
               when others =>
                  null;
            end;
            accept Done;
         end Fitting;
      begin
         select
            Fitting.Done;
            Check (not Raised, What & ": the fit raised an exception");
         or
            delay Bound;
            Ada.Text_IO.Put_Line ("FAIL  robot.handeye.ends  (guards: " & Ends_Guard & ")");
            Ada.Text_IO.Put_Line ("      " & What & ": the fit did not end in" & Duration'Image (Bound)
                                  & " seconds; the program ends with it");
            GNAT.OS_Lib.OS_Exit (1);
         end select;
      end Fit_In_Time;

      Report : Fit_Report;
      None   : Watched_Array (1 .. 0);
   begin
      Fit_In_Time ("no arm", None, Report);
      Check (not Report.Determined, "an eye is determined by no arm");
      declare
         Wrong : Watched_Access := new Watched'(S.Watching (1).all);
      begin
         Wrong.Seen (1).U := Not_A_Pixel;
         Fit_In_Time ("a sighting at no pixel", [1 => Wrong], Report);
         Check (Report.Used <= Report.Offered, "more sightings fit than were offered");
         Free_Watched (Wrong);
      end;
      declare
         Never_Moved : Watched_Access := new Watched'(Still.Watching (1).all);
      begin
         Never_Moved.Change := [others => [others => 0.0]];
         Fit_In_Time ("an arm that never moved", [1 => Never_Moved], Report);
         Check (not Report.Determined, "an eye is determined by an arm that never moved: " & Image (Report));
         Free_Watched (Never_Moved);
      end;
      declare
         Nothing_Seen : Watched_Access := new Watched'(S.Watching (1).all);
         Empty        : Watched_Access :=
           new Watched (Joints => Joint_Count, Frames => Nothing_Seen.Frames, Tracks => Nothing_Seen.Tracks,
                        Sightings => 0);
      begin
         Empty.Joint := Nothing_Seen.Joint;
         Empty.Change := Nothing_Seen.Change;
         Empty.Visible := Nothing_Seen.Visible;
         Empty.Track := Nothing_Seen.Track;
         Fit_In_Time ("tracks no keyframe sighted", [1 => Empty], Report);
         Check (not Report.Determined, "an eye is determined by no sighting: " & Image (Report));
         Free_Watched (Nothing_Seen);
         Free_Watched (Empty);
      end;
      Release (S);
      Release (Still);
   end Ends_Whatever_It_Is_Given;

   procedure Register is
   begin
      Register ("robot.handeye.recover",
                "an eye watching an arm move, a twentieth of its answers false, is not measured within Z of its own "
                & "covariance from the arm's motion alone, or the points that stand still are taken to ride on a link",
                Recovers_The_Lens_And_Pose'Access);
      Register ("robot.handeye.two",
                "two arms watched by one eye do not give both poses within Z of the covariance, or hold the lens no "
                & "tighter than one arm alone", Two_Arms_Share_The_Lens'Access);
      Register ("robot.handeye.exact",
                "answers with no noise do not give back the lens and the pose they were made from, to what the "
                & "derivatives and the arithmetic allow", Exact_Answers_Are_Exact'Access);
      Register ("robot.handeye.prior",
                "a fit that starts from the eye an earlier fit found, with more keyframes, is not the fit a search finds, or is "
                & "off the truth beyond Z of its covariance", Prior_Starts_The_Fit'Access);
      Register ("robot.handeye.ends", Ends_Guard, Ends_Whatever_It_Is_Given'Access);
      Register ("robot.handeye.distortion",
                "a lens with radial distortion is not given it, or is off the truth by more than Z of its covariance",
                Distortion_When_Significant'Access);
      Register ("robot.handeye.arm",
                "an arm known only to the covariance its fit gave its joints is not carried into the eye's: the truth "
                & "lies beyond Z of it, or it is no wider than the one that leaves the arm out",
                Arm_Uncertainty_Is_Carried'Access);
      Register ("robot.handeye.little",
                "an eye is determined by three tracks or by the keyframes of single joints alone, and is wrong",
                Too_Little_To_Go_On'Access);
   end Register;

end Driver.Robot.Kinematics.Hand_Eye.Tests;
