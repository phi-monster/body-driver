with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Robot.Kinematics.Errors;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Hand_Eye is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Fit.Flag_Array;
   use type Fit.Lens;

   --  Everything sized by sightings or tracks lives on the heap: the fit runs in the decider's task, whose stack is
   --  small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Count_Access is access Fit.Count_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Count_Array, Count_Access);
   type Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
   type Vec_Array is array (Positive range <>) of Vec3;
   type Vec_Store is access Vec_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Vec_Array, Vec_Store);

   --  A value per hypothesis of what a track rides on: nothing (0) or the link after a joint.
   type Hypothesis_Array is array (Natural range <>) of Real;

   --  The step of a central difference: the cube root of the float's resolution, relative to the size of what is
   --  stepped.
   Difference_Step : constant Real := Real'Model_Epsilon ** (1.0 / 3.0);

   function Ln (X : Real) return Real is (Ada.Numerics.Long_Elementary_Functions.Log (X));

   --  Huber's weight and cost at Z on a residual in units of its scale.
   function Huber (Value : Real) return Real is
     (if abs Value <= Driver.Conventions.Z then 1.0 else Driver.Conventions.Z / abs Value);

   function Huber_Cost (Value : Real) return Real is
     (if abs Value <= Driver.Conventions.Z then 0.5 * Value ** 2
      else Driver.Conventions.Z * (abs Value - 0.5 * Driver.Conventions.Z));

   ---------------------------------------------------------------------------
   --  The arms' links in motion

   --  Where every link stands at every keyframe, in the arm's reference frame: Moves (J, F) takes a point of the arm's
   --  reference frame that rides on the link after the J-th joint to where it is at keyframe F (Fit.Eye_At of the
   --  first J joints at that keyframe's reading change; none move a point that rides on no joint).
   type Rigid_Grid is array (Natural range <>, Positive range <>) of Rigid;
   type Grid_Access is access Rigid_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Rigid_Grid, Grid_Access);

   --  A watched arm's joints stand in their order along it (Watched), so the chain through the first K of them is
   --  1 to K.
   function In_Order (K : Natural) return Fit.Chain_Order is ([for I in 1 .. K => I]);

   function Moves_Of (Joint : Fit.Joint_Array; Change : Real_Matrix) return Grid_Access is
      N      : constant Natural := Joint'Length;
      Frames : constant Natural := Change'Length (1);
      Grid   : constant Grid_Access := new Rigid_Grid (0 .. N, 1 .. Frames);

   begin
      for F in 1 .. Frames loop
         Grid (0, F) := Driver.Numerics.Identity;
         for J in 1 .. N loop
            declare
               Readings : Real_Array (1 .. J);
            begin
               for C in 1 .. J loop
                  Readings (C) := Change (Change'First (1) + F - 1, Change'First (2) + C - 1);
               end loop;
               Grid (J, F) := Fit.Eye_At (Joint (Joint'First .. Joint'First + J - 1), In_Order (J), Readings);
            end;
         end loop;
      end loop;
      return Grid;
   end Moves_Of;

   --  Joint J's terms moved by H (the arm's own covariance is of these): the tilt of its axis along one of the two
   --  directions across it, the point on it along one, its reading scale.
   function Moved_Joints (Joint : Fit.Joint_Array; J : Positive; T : Fit.Joint_Term; H : Real) return Fit.Joint_Array is
      Result : Fit.Joint_Array := Joint;
      A      : Fit.Joint renames Result (Result'First + J - 1);
      E1, E2 : Vec3;
   begin
      Fit.Across (A.W, E1, E2);
      case T is
         when Fit.Tilt_1  => A.W := Driver.Numerics.Unit (A.W + H * E1);
         when Fit.Tilt_2  => A.W := Driver.Numerics.Unit (A.W + H * E2);
         when Fit.Point_1 => A.P := A.P + H * E1;
         when Fit.Point_2 => A.P := A.P + H * E2;
         when Fit.Scale   => A.C := A.C + H;
      end case;
      return Result;
   end Moved_Joints;

   ---------------------------------------------------------------------------
   --  Which joints move a point

   function Infer_Link (A : Watched; T : Positive) return Natural is
      --  Per keyframe, the one joint it moved (0: none or several) from the keyframe the track was chosen at.
      Alone    : Count_Access := new Fit.Count_Array (1 .. A.Frames);
      Result   : Natural := 0;
      R        : constant Positive := A.Track (T).Frame;
      --  Joint C's reading change from the track's keyframe to keyframe F.
      function Change (F, C : Positive) return Real is (A.Change (F, C) - A.Change (R, C));
   begin
      for F in 1 .. A.Frames loop
         declare
            Moved : Natural := 0;
            Which : Natural := 0;
         begin
            for C in 1 .. A.Joints loop
               if A.Visible (C) > 0.0 and then abs Change (F, C) >= A.Visible (C) then
                  Moved := Moved + 1;
                  Which := C;
               end if;
            end loop;
            Alone (F) := (if Moved = 1 then Which else 0);
         end;
      end loop;
      for J in 1 .. A.Joints loop
         declare
            --  The track's displacement against the joint's reading change over the keyframes it moved alone, through
            --  the origin (no change, no move), each sighting at its own stated sigma: its coefficient and how far
            --  it is from none, in the scatter the line leaves where that is more than the sightings state.
            Count          : Natural := 0;
            Sxx, Sxu, Sxv  : Real := 0.0;
            --  Each sighting at the sigma the aligner stated of it, the variance of a place known to a pixel where it
            --  stated none; a stated sigma too small shows in the scatter about the line, which widens the line's
            --  (Inflate below). Every sighting used to be floored at a pixel's quantization, 0.29 px a coordinate, where
            --  the aligner errs by about 0.02 on still twins: the view of s38's fixed eye 1 (servo seed 6 of the fake
            --  robot, 10-17, replayed) found 23 to 26 of its 26 points at the keyframes of single joints, moved 0.16 to
            --  0.67 px, and no track was given a link.
            function Variance_Of (Own : Real) return Real is
              (if Own > 0.0 then Own ** 2 else Driver.Alignment.Quantization);
         begin
            for S of A.Seen loop
               if S.Track = T and then S.Frame <= A.Frames and then Alone (S.Frame) = J then
                  declare
                     Variance : constant Real := Variance_Of (S.Own);
                     X        : constant Real := Change (S.Frame, J);
                  begin
                     Count := Count + 1;
                     Sxx := Sxx + X * X / Variance;
                     Sxu := Sxu + X * (S.U - A.Track (T).U0) / Variance;
                     Sxv := Sxv + X * (S.V - A.Track (T).V0) / Variance;
                  end;
               end if;
            end loop;
            if Count >= 2 and then Sxx > 0.0 then
               declare
                  Bu     : constant Real := Sxu / Sxx;
                  Bv     : constant Real := Sxv / Sxx;
                  Spread : Real := 0.0;
               begin
                  for S of A.Seen loop
                     if S.Track = T and then S.Frame <= A.Frames and then Alone (S.Frame) = J then
                        declare
                           Variance : constant Real := Variance_Of (S.Own);
                           X        : constant Real := Change (S.Frame, J);
                        begin
                           Spread := Spread + ((S.U - A.Track (T).U0 - Bu * X) ** 2
                                               + (S.V - A.Track (T).V0 - Bv * X) ** 2) / Variance;
                        end;
                     end if;
                  end loop;
                  declare
                     Freedom : constant Natural := 2 * Count - 2;
                     Inflate : constant Real := Real'Max (1.0, Sqrt (Spread / Real (Freedom)));
                     Sigma   : constant Real := Inflate / Sqrt (Sxx);
                  begin
                     if Driver.Uncertain.Significant (Bu, Sigma, Freedom)
                       or else Driver.Uncertain.Significant (Bv, Sigma, Freedom)
                     then
                        Result := J;
                     end if;
                  end;
               end;
            end if;
         end;
      end loop;
      Free (Alone);
      return Result;
   end Infer_Link;

   ---------------------------------------------------------------------------
   --  What the fit works on

   type Grid_Array is array (Positive range <>) of Grid_Access;

   --  Every arm's tracks and sightings, one after another: the arm, the track (numbered over all the arms), the
   --  keyframe (of its own arm) and the pixel of each sighting, and the pixel each track was chosen at.
   type Problem (Arms, Tracks, Sightings : Natural) is record
      Joints        : Fit.Count_Array (1 .. Arms);
      Moves         : Grid_Array (1 .. Arms);
      First_Track   : Fit.Count_Array (1 .. Arms);       --  the number of the arm's first track, less one
      Track_Arm     : Fit.Count_Array (1 .. Tracks);
      U0, V0        : Real_Array (1 .. Tracks);
      Track_Frame   : Fit.Count_Array (1 .. Tracks);   --  the keyframe each was chosen at
      Sight_Arm     : Fit.Count_Array (1 .. Sightings);
      Sight_Track   : Fit.Count_Array (1 .. Sightings);
      Sight_Frame   : Fit.Count_Array (1 .. Sightings);
      Sight_U       : Real_Array (1 .. Sightings);
      Sight_V       : Real_Array (1 .. Sightings);
      Sight_Own     : Real_Array (1 .. Sightings);
   end record;

   type Problem_Access is access Problem;

   function Build (Arms : Watched_Array) return Problem_Access is
      Tracks, Sightings : Natural := 0;
   begin
      for A of Arms loop
         Tracks := Tracks + A.Tracks;
         Sightings := Sightings + A.Sightings;
      end loop;
      declare
         P : constant Problem_Access := new Problem (Arms'Length, Tracks, Sightings);
         T, S : Natural := 0;
      begin
         for K in Arms'Range loop
            declare
               A : Watched renames Arms (K).all;
               N : constant Positive := K - Arms'First + 1;
            begin
               P.Joints (N) := A.Joints;
               P.Moves (N) := Moves_Of (A.Joint, A.Change);
               P.First_Track (N) := T;
               for I in 1 .. A.Tracks loop
                  P.Track_Arm (T + I) := N;
                  P.U0 (T + I) := A.Track (I).U0;
                  P.V0 (T + I) := A.Track (I).V0;
                  P.Track_Frame (T + I) := A.Track (I).Frame;
               end loop;
               for I in 1 .. A.Sightings loop
                  P.Sight_Arm (S + I) := N;
                  P.Sight_Track (S + I) := T + A.Seen (I).Track;
                  P.Sight_Frame (S + I) := A.Seen (I).Frame;
                  P.Sight_U (S + I) := A.Seen (I).U;
                  P.Sight_V (S + I) := A.Seen (I).V;
                  P.Sight_Own (S + I) := A.Seen (I).Own;
               end loop;
               T := T + A.Tracks;
               S := S + A.Sightings;
            end;
         end loop;
         return P;
      end;
   end Build;

   --  The motion of a point riding on Link from the keyframe its track was chosen at to sighting I's: the link's motion
   --  at the sighting's keyframe after the inverse of its motion at the track's (none at the arm's reference). Tracks
   --  were chosen at the reference alone, and an eye that saw the arm small kept 6 to 11 points of it there, where the
   --  arm carried across its view showed it 21 to 60 more (the bench's fixed eye over table arms, 10-20).
   function Move_Of (P : Problem; A : Positive; Link : Natural; I : Positive) return Rigid is
     (if P.Track_Frame (P.Sight_Track (I)) = 1 then P.Moves (A) (Link, P.Sight_Frame (I))
      else P.Moves (A) (Link, P.Sight_Frame (I)) * Inverse (P.Moves (A) (Link, P.Track_Frame (P.Sight_Track (I)))));

   procedure Release (P : in out Problem_Access) is
      procedure Free is new Ada.Unchecked_Deallocation (Problem, Problem_Access);
   begin
      for G of P.Moves loop
         Free (G);
      end loop;
      Free (P);
   end Release;

   ---------------------------------------------------------------------------
   --  The eye

   type Pose_Value is record
      R : Mat3 := Identity3;   --  the camera's turn in the arm's reference frame
      C : Vec3 := Zero3;       --  and its centre
   end record;

   type Pose_Values is array (Positive range <>) of Pose_Value;

   type State (Arms : Positive) is record
      L    : Fit.Lens;
      Pose : Pose_Values (1 .. Arms);
   end record;

   --  The first of a pose's six terms in the vector of all: the lens's six come first.
   function Pose_At (Arm : Positive) return Natural is (Fit.Lens_Terms + Pose_Terms * (Arm - 1));

   function Moved_Lens (L : Fit.Lens; Term : Positive; X : Real) return Fit.Lens is
     (case Term is
         when 1 => (Fx => L.Fx * Exp (X), Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2),
         when 2 => (Fx => L.Fx, Fy => L.Fy * Exp (X), Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2),
         when 3 => (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx + X, Cy => L.Cy, K1 => L.K1, K2 => L.K2),
         when 4 => (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy + X, K1 => L.K1, K2 => L.K2),
         when 5 => (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1 + X, K2 => L.K2),
         when others => (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2 + X));

   --  A change X of the terms, as a fit reports them: the lens's by the logarithms of the focal lengths and the others
   --  added, each arm's turn about the camera's own axes after its pose, its centre added.
   function Moved_By (S : State; X : Real_Array) return State is
      Result : State := S;
   begin
      for T in 1 .. Fit.Lens_Terms loop
         Result.L := Moved_Lens (Result.L, T, X (X'First + T - 1));
      end loop;
      for A in 1 .. S.Arms loop
         declare
            O : constant Natural := X'First + Pose_At (A) - 1;
         begin
            Result.Pose (A).R := S.Pose (A).R * Driver.Numerics.Exp ([X (O + 1), X (O + 2), X (O + 3)]);
            Result.Pose (A).C := S.Pose (A).C + [X (O + 4), X (O + 5), X (O + 6)];
         end;
      end loop;
      return Result;
   end Moved_By;

   type Free_Terms is array (Positive range <>) of Positive;

   ---------------------------------------------------------------------------
   --  What the eye sees of a track at a keyframe, and how that moves with the terms

   --  The pixel of a point of the eye's frame, and how it moves with the point (two rows, three columns) and with the
   --  lens's terms (two rows, six columns), the lens's distortion Fit.Project has.
   procedure Project_Partials
     (L        : Fit.Lens;
      X        : Vec3;
      U, V     : out Real;
      By_Point : out Real_Matrix;
      By_Lens  : out Real_Matrix;
      Ahead    : out Boolean)
   is
   begin
      By_Point := [others => [others => 0.0]];
      By_Lens := [others => [others => 0.0]];
      Ahead := X (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if Ahead then
         declare
            Xn : constant Real := X (1) / X (3);
            Yn : constant Real := X (2) / X (3);
            R2 : constant Real := Xn * Xn + Yn * Yn;
            D  : constant Real := 1.0 + L.K1 * R2 + L.K2 * R2 * R2;
            --  D's slope in R2, and the point's own: how the normalised coordinates move with it, and D with those.
            Dr : constant Real := L.K1 + 2.0 * L.K2 * R2;
            Dx : constant Real := 2.0 * Xn * Dr;
            Dy : constant Real := 2.0 * Yn * Dr;
            --  d(Xn, Yn) / d(X, Y, Z)
            Xx : constant Real := 1.0 / X (3);
            Xz : constant Real := -X (1) / (X (3) * X (3));
            Yy : constant Real := 1.0 / X (3);
            Yz : constant Real := -X (2) / (X (3) * X (3));
            --  d(U, V) / d(Xn, Yn)
            Uxn : constant Real := L.Fx * (D + Xn * Dx);
            Uyn : constant Real := L.Fx * Xn * Dy;
            Vxn : constant Real := L.Fy * Yn * Dx;
            Vyn : constant Real := L.Fy * (D + Yn * Dy);
         begin
            U := L.Fx * Xn * D + L.Cx;
            V := L.Fy * Yn * D + L.Cy;
            By_Point (1, 1) := Uxn * Xx;
            By_Point (1, 2) := Uyn * Yy;
            By_Point (1, 3) := Uxn * Xz + Uyn * Yz;
            By_Point (2, 1) := Vxn * Xx;
            By_Point (2, 2) := Vyn * Yy;
            By_Point (2, 3) := Vxn * Xz + Vyn * Yz;
            By_Lens (1, 1) := L.Fx * Xn * D;
            By_Lens (1, 3) := 1.0;
            By_Lens (1, 5) := L.Fx * Xn * R2;
            By_Lens (1, 6) := L.Fx * Xn * R2 * R2;
            By_Lens (2, 2) := L.Fy * Yn * D;
            By_Lens (2, 4) := 1.0;
            By_Lens (2, 5) := L.Fy * Yn * R2;
            By_Lens (2, 6) := L.Fy * Yn * R2 * R2;
         end;
      end if;
   end Project_Partials;

   --  Whether a track's light is the arm's to follow: it rides on a link that moves.
   function On_Arm (Link : Fit.Count_Array; T : Positive) return Boolean is (Link (T) >= 1);

   --  The residuals (the place the model puts each sighting at, less the pixel it was seen at: across and down for
   --  each, in pixels) and how they move with the terms Which and with the log of the track's depth. A sighting of a
   --  track that rides on no link, or that is behind the eye, has no residual (zero) and no derivative, and is not
   --  Seen.
   procedure Linearize
     (P     : Problem;
      S     : State;
      Link  : Fit.Count_Array;
      Lz    : Real_Array;
      Which : Free_Terms;
      Seen  : out Fit.Flag_Array;
      R     : out Real_Array;
      Jg    : out Real_Matrix;
      Jd    : out Real_Array)
   is
      W : constant Natural := Which'Length;
      type Vec3_Pair is array (1 .. Fit.Lens_Terms) of Vec3;
      --  A track's line of sight and how it moves with the lens's terms.
      type Ray_Grid is array (Positive range <>) of Vec3_Pair;
      type Ray_Grid_Access is access Ray_Grid;
      procedure Free is new Ada.Unchecked_Deallocation (Ray_Grid, Ray_Grid_Access);
      Slopes : Ray_Grid_Access := new Ray_Grid (1 .. P.Tracks);
      Lines  : Vec_Store := new Vec_Array (1 .. P.Tracks);
      Lens_Free : Fit.Flag_Array (1 .. Fit.Lens_Terms) := [others => False];
   begin
      Seen := [others => False];
      R := [others => 0.0];
      Jg := [others => [others => 0.0]];
      Jd := [others => 0.0];
      for K in Which'Range loop
         if Which (K) <= Fit.Lens_Terms then
            Lens_Free (Which (K)) := True;
         end if;
      end loop;
      for T in 1 .. P.Tracks loop
         if On_Arm (Link, T) then
            Lines (T) := Fit.Ray (S.L, P.U0 (T), P.V0 (T));
            for Term in 1 .. Fit.Lens_Terms loop
               Slopes (T) (Term) := Zero3;
               if Lens_Free (Term) then
                  Slopes (T) (Term) :=
                    (Fit.Ray (Moved_Lens (S.L, Term, Difference_Step), P.U0 (T), P.V0 (T))
                     - Fit.Ray (Moved_Lens (S.L, Term, -Difference_Step), P.U0 (T), P.V0 (T)))
                    / (2.0 * Difference_Step);
               end if;
            end loop;
         end if;
      end loop;
      for I in 1 .. P.Sightings loop
         declare
            T : constant Positive := P.Sight_Track (I);
            A : constant Positive := P.Sight_Arm (I);
         begin
            if On_Arm (Link, T) then
               declare
                  Pose  : Pose_Value renames S.Pose (A);
                  Z     : constant Real := Exp (Lz (T));
                  Link_Of : constant Natural := Link (T);
                  Move  : constant Rigid := Move_Of (P, A, Link_Of, I);
                  --  The products written out (Driver.Numerics): the general operators' every result took the
                  --  secondary stack, the fixed eye's search most of its time (robot.head.noisy, 10-20).
                  X0c   : constant Vec3 := Driver.Numerics.Scaled (Z, Lines (T));
                  X0a   : constant Vec3 := Driver.Numerics.Plus (Driver.Numerics.Times (Pose.R, X0c), Pose.C);
                  Xka   : constant Vec3 := Move * X0a;
                  Xkc   : constant Vec3 := Driver.Numerics.Transposed_Times (Pose.R, Driver.Numerics.Minus (Xka, Pose.C));
                  Q     : constant Mat3 :=
                    Driver.Numerics.Times (Driver.Numerics.Times (Driver.Numerics.Transposed (Pose.R), Move.Rotation), Pose.R);
                  Up, Vp : Real;
                  By_Point : Real_Matrix (1 .. 2, 1 .. 3);
                  By_Lens  : Real_Matrix (1 .. 2, 1 .. Fit.Lens_Terms);
                  Ahead    : Boolean;
               begin
                  Project_Partials (S.L, Xkc, Up, Vp, By_Point, By_Lens, Ahead);
                  if Ahead then
                     Seen (I) := True;
                     R (2 * I - 1) := Up - P.Sight_U (I);
                     R (2 * I) := Vp - P.Sight_V (I);
                     --  How the point (in the eye's frame, at this keyframe) moves with the log depth.
                     declare
                        Along : constant Vec3 := Driver.Numerics.Times (Q, X0c);
                     begin
                        for C in 1 .. 2 loop
                           Jd (2 * I - 2 + C) := By_Point (C, 1) * Along (1) + By_Point (C, 2) * Along (2)
                                                  + By_Point (C, 3) * Along (3);
                        end loop;
                     end;
                     for K in Which'Range loop
                        declare
                           Term : constant Positive := Which (K);
                           Column : constant Positive := K - Which'First + 1;
                           Moves_By : Vec3 := Zero3;      --  how the point in the eye's frame moves with the term
                           Direct_U, Direct_V : Real := 0.0;   --  and the pixel apart from it
                        begin
                           if Term <= Fit.Lens_Terms then
                              Moves_By := Driver.Numerics.Scaled (Z, Driver.Numerics.Times (Q, Slopes (T) (Term)));
                              Direct_U := By_Lens (1, Term);
                              Direct_V := By_Lens (2, Term);
                           elsif Term > Pose_At (A) and then Term <= Pose_At (A) + Pose_Terms then
                              declare
                                 Own : constant Positive := Term - Pose_At (A);
                              begin
                                 if Own <= 3 then
                                    --  The turn: Skew (Xkc) - Q Skew (X0c), a column of it.
                                    declare
                                       Left  : constant Mat3 := Driver.Numerics.Skew (Xkc);
                                       Right : constant Mat3 := Driver.Numerics.Times (Q, Driver.Numerics.Skew (X0c));
                                    begin
                                       for Row in 1 .. 3 loop
                                          Moves_By (Row) := Left (Row, Own) - Right (Row, Own);
                                       end loop;
                                    end;
                                 else
                                    --  The centre: Transpose (R) (A - I), a column of it.
                                    declare
                                       Axis : constant Positive := Own - 3;
                                       Column : constant Vec3 :=
                                         [Move.Rotation (1, Axis) - Identity3 (1, Axis), Move.Rotation (2, Axis) - Identity3 (2, Axis),
                                          Move.Rotation (3, Axis) - Identity3 (3, Axis)];
                                       Back   : constant Vec3 := Driver.Numerics.Transposed_Times (Pose.R, Column);
                                    begin
                                       Moves_By := Back;
                                    end;
                                 end if;
                              end;
                           end if;
                           Jg (2 * I - 1, Column) := By_Point (1, 1) * Moves_By (1) + By_Point (1, 2) * Moves_By (2)
                                                      + By_Point (1, 3) * Moves_By (3) + Direct_U;
                           Jg (2 * I, Column) := By_Point (2, 1) * Moves_By (1) + By_Point (2, 2) * Moves_By (2)
                                                  + By_Point (2, 3) * Moves_By (3) + Direct_V;
                        end;
                     end loop;
                  end if;
               end;
            end if;
         end;
      end loop;
      Free (Slopes);
      Free (Lines);
      pragma Unreferenced (W);
   end Linearize;

   ---------------------------------------------------------------------------
   --  The residuals alone, and what the fit holds of the tracks

   --  A track that rides on no link has no place in the model; a sighting of one is not Seen. A sighting behind the eye
   --  is wrong by as much as the picture is big.
   procedure Residuals_Only
     (P    : Problem;
      S    : State;
      Link : Fit.Count_Array;
      Lz   : Real_Array;
      Seen : out Fit.Flag_Array;
      R    : out Real_Array)
   is
   begin
      Seen := [others => False];
      R := [others => 0.0];
      for I in 1 .. P.Sightings loop
         declare
            T : constant Positive := P.Sight_Track (I);
            A : constant Positive := P.Sight_Arm (I);
         begin
            if On_Arm (Link, T) then
               declare
                  Pose  : Pose_Value renames S.Pose (A);
                  Move  : constant Rigid := Move_Of (P, A, Link (T), I);
                  X0c   : constant Vec3 := Driver.Numerics.Scaled (Exp (Lz (T)), Fit.Ray (S.L, P.U0 (T), P.V0 (T)));
                  Xka   : constant Vec3 := Move * Driver.Numerics.Plus (Driver.Numerics.Times (Pose.R, X0c), Pose.C);
                  Xkc   : constant Vec3 := Driver.Numerics.Transposed_Times (Pose.R, Driver.Numerics.Minus (Xka, Pose.C));
                  Up, Vp : Real;
                  Ahead : Boolean;
               begin
                  Fit.Project (S.L, Xkc, Up, Vp, Ahead);
                  if Ahead then
                     Seen (I) := True;
                     R (2 * I - 1) := Up - P.Sight_U (I);
                     R (2 * I) := Vp - P.Sight_V (I);
                  end if;
               end;
            end if;
         end;
      end loop;
   end Residuals_Only;

   --  What the fit holds besides the state: the link of every track, the log of its depth, which sightings have a place
   --  in front of the eye and which of those fit, and how far the residuals lie (their error model).
   type Work (Tracks, Sightings : Natural) is record
      Link  : Fit.Count_Array (1 .. Tracks);
      Lz    : Real_Array (1 .. Tracks);
      Seen  : Fit.Flag_Array (1 .. Sightings);
      Fits  : Fit.Flag_Array (1 .. Sightings);
      Model : Fit.Error_Model;
   end record;

   type Work_Access is access Work;

   procedure Release (W : in out Work_Access) is
      procedure Free is new Ada.Unchecked_Deallocation (Work, Work_Access);
   begin
      Free (W);
   end Release;

   --  Sighting I's scale: what the model leaves of any sighting, and what the aligner stated of this one as far as the
   --  residuals bear it out.
   function Scale_Of (P : Problem; W : Work; I : Positive) return Real is
     (Fit.Scale_Of (W.Model, P.Sight_Own (I)));

   --  The error model at which the residuals of the sightings in front of the eye are typical of their scales.
   procedure Measure_Floor (P : Problem; W : in out Work; R : Real_Array) is
      Count : Natural := 0;
   begin
      for I in 1 .. P.Sightings loop
         if W.Seen (I) then
            Count := Count + 1;
         end if;
      end loop;
      if Count = 0 then
         W.Model := (Floor => 0.0, Gain => 1.0);
         return;
      end if;
      declare
         Both : Real_Access := new Real_Array (1 .. 2 * Count);
         Own  : Real_Access := new Real_Array (1 .. 2 * Count);
         K    : Natural := 0;
      begin
         for I in 1 .. P.Sightings loop
            if W.Seen (I) then
               Both (K + 1) := R (2 * I - 1);
               Both (K + 2) := R (2 * I);
               Own (K + 1) := P.Sight_Own (I);
               Own (K + 2) := P.Sight_Own (I);
               K := K + 2;
            end if;
         end loop;
         W.Model := Fit.Model_Of (Both.all, Own.all);
         Free (Both);
         Free (Own);
      end;
   end Measure_Floor;

   --  The sightings that fit at the residuals R: in front of the eye, neither residual significant against its scale.
   --  The floor is read from the residuals first. Same says the choice did not change.
   procedure Choose (P : Problem; W : in out Work; R : Real_Array; Same : out Boolean) is
      Next : Fit.Flag_Array (1 .. P.Sightings);
   begin
      Measure_Floor (P, W, R);
      for I in 1 .. P.Sightings loop
         Next (I) := W.Seen (I)
           and then not Driver.Uncertain.Significant (R (2 * I - 1), Scale_Of (P, W, I))
           and then not Driver.Uncertain.Significant (R (2 * I), Scale_Of (P, W, I));
      end loop;
      Same := Next = W.Fits;
      W.Fits := Next;
   end Choose;

   --  The robust cost of the sightings that fit: each at its scale, and one that has gone behind the eye as wrong as
   --  the picture is big.
   function Cost_Of (P : Problem; W : Work; R : Real_Array; Behind : Real; Seen : Fit.Flag_Array) return Real is
      Cost : Real := 0.0;
   begin
      for I in 1 .. P.Sightings loop
         if W.Fits (I) then
            if Seen (I) then
               Cost := Cost + Huber_Cost (R (2 * I - 1) / Scale_Of (P, W, I))
                            + Huber_Cost (R (2 * I) / Scale_Of (P, W, I));
            else
               Cost := Cost + 2.0 * Huber_Cost (Behind / Scale_Of (P, W, I));
            end if;
         end if;
      end loop;
      return Cost;
   end Cost_Of;

   --  How well a state explains the tracks that ride on a link, as the log of the likelihood of their sightings: each
   --  is either an inlier, with the Gaussian density of its two residuals at the scale its own stated sigma and the
   --  floor read from all the residuals together give, or an outlier, with the density of a pixel anywhere in the
   --  picture, whichever is the more likely; a sighting behind the eye, or of a track Placed says has no place, is an
   --  outlier. States that explain the sightings differently are comparable by it, which a sum of Huber's costs is
   --  not (an outlier costs the more the farther the state puts the point from it, and that differs from state to
   --  state by more than the inliers' costs do), and the difference of two, at one floor, is the log of their
   --  likelihood ratio. Model is the error model to judge the residuals by, or one of a negative floor to read it from
   --  them (it comes back).
   procedure Likelihood
     (P      : Problem;
      S      : State;
      Link   : Fit.Count_Array;
      Lz     : Real_Array;
      Placed : Fit.Flag_Array;
      Width, Height : Positive;
      Model  : in out Fit.Error_Model;
      Total  : out Real)
   is
      Elsewhere : constant Real := -Ln (Real (Width) * Real (Height));
      R         : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Both      : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Own       : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Seen      : Fit.Flag_Array (1 .. P.Sightings);
      Counted   : Natural := 0;
   begin
      Total := 0.0;
      Residuals_Only (P, S, Link, Lz, Seen, R.all);
      for I in 1 .. P.Sightings loop
         if On_Arm (Link, P.Sight_Track (I)) and then Seen (I) and then Placed (P.Sight_Track (I)) then
            Both (Counted + 1) := R (2 * I - 1);
            Both (Counted + 2) := R (2 * I);
            Own (Counted + 1) := P.Sight_Own (I);
            Own (Counted + 2) := P.Sight_Own (I);
            Counted := Counted + 2;
         end if;
      end loop;
      if Model.Floor < 0.0 then
         Model := Fit.Model_Of (Both (1 .. Counted), Own (1 .. Counted));
      end if;
      begin
         for I in 1 .. P.Sightings loop
            if On_Arm (Link, P.Sight_Track (I)) then
               if Seen (I) and then Placed (P.Sight_Track (I)) then
                  declare
                     Scale : constant Real := Fit.Scale_Of (Model, P.Sight_Own (I));
                     Inlier : constant Real :=
                       -(R (2 * I - 1) ** 2 + R (2 * I) ** 2) / (2.0 * Scale ** 2) - Ln (2.0 * Ada.Numerics.Pi * Scale ** 2);
                  begin
                     Total := Total + Real'Max (Inlier, Elsewhere);
                  end;
               else
                  Total := Total + Elsewhere;
               end if;
            end if;
         end loop;
      end;
      Free (R);
      Free (Both);
      Free (Own);
   end Likelihood;

   --  The normal equations of the sightings that fit, with Huber weights, in units of the scales: A and the gradient G
   --  (minus, so that A step = G) of the terms, and per track the depth's own: its information, its cross information
   --  with the terms and its gradient.
   procedure Normals
     (P  : Problem;
      W  : Work;
      R  : Real_Array;
      Jg : Real_Matrix;
      Jd : Real_Array;
      A  : out Real_Matrix;
      G  : out Real_Vector;
      Dd : out Real_Array;
      Dg : out Real_Matrix;
      Gd : out Real_Array)
   is
      Count : constant Natural := Jg'Length (2);
   begin
      A := [others => [others => 0.0]];
      G := [others => 0.0];
      Dd := [others => 0.0];
      Dg := [others => [others => 0.0]];
      Gd := [others => 0.0];
      for I in 1 .. P.Sightings loop
         if W.Fits (I) then
            declare
               Scale : constant Real := Scale_Of (P, W, I);
               T     : constant Positive := P.Sight_Track (I);
            begin
               for C in 0 .. 1 loop
                  declare
                     K  : constant Positive := 2 * I - 1 + C;
                     Z  : constant Real := R (K) / Scale;
                     Wt : constant Real := Huber (Z);
                     Jk : constant Real := Jd (K) / Scale;
                  begin
                     for A1 in 1 .. Count loop
                        declare
                           Ja : constant Real := Jg (K, Jg'First (2) + A1 - 1) / Scale;
                        begin
                           G (A1) := G (A1) - Wt * Ja * Z;
                           Dg (T, A1) := Dg (T, A1) + Wt * Jk * Ja;
                           for B1 in A1 .. Count loop
                              A (A1, B1) := A (A1, B1) + Wt * Ja * Jg (K, Jg'First (2) + B1 - 1) / Scale;
                           end loop;
                        end;
                     end loop;
                     Dd (T) := Dd (T) + Wt * Jk * Jk;
                     Gd (T) := Gd (T) - Wt * Jk * Z;
                  end;
               end loop;
            end;
         end if;
      end loop;
      for A1 in 1 .. Count loop
         for B1 in 1 .. A1 - 1 loop
            A (A1, B1) := A (B1, A1);
         end loop;
      end loop;
   end Normals;

   --  The information of the prior that holds a depth to the factor its perspective is linear in: one e-fold of its
   --  logarithm. A depth the sightings hardly feel (the information in it is a rounding error from nothing) would
   --  otherwise be stepped by the residual over a slope that is no more than rounding, a factor of billions, and
   --  every damping that is to hold that one step would hold every other term to nothing. For a depth the sightings
   --  do feel, the information is the thousands and this adds none worth the name.
   Depth_Prior : constant Real := 1.0;

   --  The reduced step of the terms and the steps of the depths' logarithms, for a damping.
   procedure Step_Of
     (A      : Real_Matrix;
      G      : Real_Vector;
      Dd     : Real_Array;
      Dg     : Real_Matrix;
      Gd     : Real_Array;
      Lambda : Real;
      Step   : out Real_Vector;
      Depth  : out Real_Array;
      Ok     : out Boolean)
   is
      Count : constant Natural := G'Length;
      Sm    : Real_Matrix (1 .. Count, 1 .. Count) := A;
      Rhs   : Real_Vector (1 .. Count) := G;
      L     : Real_Matrix (1 .. Count, 1 .. Count);
      Damped : Real_Array (1 .. Dd'Length);
   begin
      for Q in 1 .. Count loop
         Sm (Q, Q) := A (Q, Q) * (1.0 + Lambda) + Lambda * Real'Model_Small;
      end loop;
      for T in 1 .. Dd'Length loop
         Damped (T) := Dd (T) * (1.0 + Lambda) + Depth_Prior;
         if Damped (T) > 0.0 then
            for Q in 1 .. Count loop
               if Dg (T, Q) /= 0.0 then
                  Rhs (Q) := Rhs (Q) - Dg (T, Q) * Gd (T) / Damped (T);
                  for B in 1 .. Count loop
                     Sm (Q, B) := Sm (Q, B) - Dg (T, Q) * Dg (T, B) / Damped (T);
                  end loop;
               end if;
            end loop;
         end if;
      end loop;
      Driver.Numerics.Dense.Cholesky (Sm, L, Ok);
      Step := [others => 0.0];
      Depth := [others => 0.0];
      if Ok then
         Step := Driver.Numerics.Dense.Cholesky_Solve (L, Rhs);
         for T in 1 .. Dd'Length loop
            if Damped (T) > 0.0 then
               declare
                  Sum : Real := Gd (T);
               begin
                  for Q in 1 .. Count loop
                     Sum := Sum - Dg (T, Q) * Step (Q);
                  end loop;
                  Depth (T) := Sum / Damped (T);
               end;
            end if;
         end loop;
      end if;
   end Step_Of;

   --  One step of the robust least squares by Levenberg-Marquardt on the terms Which and the depths, at the scales
   --  Floor makes, over the sightings that fit: the damping is doubled until a step lowers the cost or moves nothing,
   --  and halved after one that does. Moved says whether a step was taken; Converged, that the step could not move any
   --  combination of the terms by more than Unchanged_Fraction of its standard error (or none lowered the cost).
   procedure Refine_Step
     (P             : Problem;
      S             : in out State;
      W             : in out Work;
      Which         : Free_Terms;
      Width, Height : Positive;
      Lambda        : in out Real;
      Moved         : out Boolean;
      Converged     : out Boolean)
   is
      Count  : constant Natural := Which'Length;
      Behind : constant Real := Real (Width + Height);
      R      : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Rn     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Jg     : Matrix_Access := new Real_Matrix (1 .. 2 * P.Sightings, 1 .. Count);
      Jd     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Seen_N : Fit.Flag_Array (1 .. P.Sightings);
      Cost0  : Real;
      A      : Real_Matrix (1 .. Count, 1 .. Count);
      G      : Real_Vector (1 .. Count);
      Dd     : Real_Array (1 .. P.Tracks);
      Dg     : Matrix_Access := new Real_Matrix (1 .. P.Tracks, 1 .. Count);
      Gd     : Real_Array (1 .. P.Tracks);
      Lowered : Boolean := False;
      Wanders : Boolean := True;
   begin
      Moved := False;
      Converged := False;
      Residuals_Only (P, S, W.Link, W.Lz, W.Seen, R.all);
      Cost0 := Cost_Of (P, W, R.all, Behind, W.Seen);
      Linearize (P, S, W.Link, W.Lz, Which, W.Seen, R.all, Jg.all, Jd.all);
      Normals (P, W, R.all, Jg.all, Jd.all, A, G, Dd, Dg.all, Gd);
      --  The damping is doubled until a step lowers the cost or moves nothing, and no further than a float goes.
      while Wanders and then not Lowered and then Lambda < Real'Last / 2.0 loop
         declare
            Step  : Real_Vector (1 .. Count);
            Depth : Real_Array (1 .. P.Tracks);
            Ok    : Boolean;
         begin
            Step_Of (A, G, Dd, Dg.all, Gd, Lambda, Step, Depth, Ok);
            if Ok then
               declare
                  X     : Real_Array (1 .. Terms (S.Arms)) := [others => 0.0];
                  Next  : State (S.Arms);
                  Lz_N  : Real_Array (1 .. P.Tracks) := W.Lz;
                  C1    : Real;
               begin
                  for Q in 1 .. Count loop
                     X (Which (Which'First + Q - 1)) := Step (Q);
                  end loop;
                  Next := Moved_By (S, X);
                  for T in 1 .. P.Tracks loop
                     Lz_N (T) := W.Lz (T) + Depth (T);
                  end loop;
                  --  A step moves the state when it changes it in floating point, not when a term of it is not
                  --  zero.
                  Wanders := (for some T in 1 .. P.Tracks => Lz_N (T) /= W.Lz (T))
                    or else Next.L /= S.L or else (for some A2 in 1 .. S.Arms => Next.Pose (A2) /= S.Pose (A2));
                  if Wanders then
                     Residuals_Only (P, Next, W.Link, Lz_N, Seen_N, Rn.all);
                     C1 := Cost_Of (P, W, Rn.all, Behind, Seen_N);
                     if C1 < Cost0 then
                        Converged := not Fit.Moves_The_Fit (Cost0 - C1);
                        S := Next;
                        W.Lz := Lz_N;
                        W.Seen := Seen_N;
                        Lambda := Lambda / 2.0;
                        Lowered := True;
                        Moved := True;
                     end if;
                  end if;
               end;
            end if;
            if not Lowered then
               Lambda := 2.0 * Lambda;
            end if;
         end;
      end loop;
      if not Lowered then
         Converged := True;
      end if;
      Free (R);
      Free (Rn);
      Free (Jg);
      Free (Jd);
      Free (Dg);
   end Refine_Step;

   --  Refine_Step until it converges, from the damping the convention starts with. Changed says whether any step was
   --  taken.
   procedure Refine
     (P             : Problem;
      S             : in out State;
      W             : in out Work;
      Which         : Free_Terms;
      Width, Height : Positive;
      Changed       : out Boolean)
   is
      Lambda    : Real := Driver.Conventions.Initial_Damping;
      Moved     : Boolean;
      Converged : Boolean := False;
   begin
      Changed := False;
      while not Converged loop
         Refine_Step (P, S, W, Which, Width, Height, Lambda, Moved, Converged);
         Changed := Changed or else Moved;
      end loop;
   end Refine;

   --  Choosing the sightings that fit and refining on them, again and again, until a choice is one made before: the
   --  fit has stopped changing (the choice before it was refined on), or it goes round in a circle of choices, or
   --  as many choices have been made as a float has digits. The last refinement moved the noise, so the choice is
   --  made at its end once more, and stands.
   procedure Settle
     (P : Problem; S : in out State; W : in out Work; Which : Free_Terms; Width, Height : Positive)
   is
      type Flags_Access is access Fit.Flag_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Fit.Flag_Array, Flags_Access);
      type Choice_List is array (1 .. Real'Machine_Mantissa) of Flags_Access;
      Began   : Choice_List := [others => null];
      Passes  : Natural := 0;
      Same    : Boolean;
      Changed : Boolean;
      R       : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
   begin
      loop
         Residuals_Only (P, S, W.Link, W.Lz, W.Seen, R.all);
         Choose (P, W, R.all, Same);
         exit when Passes = Began'Last or else (for some Earlier of Began (1 .. Passes) => Earlier.all = W.Fits);
         Passes := Passes + 1;
         Began (Passes) := new Fit.Flag_Array'(W.Fits);
         Refine (P, S, W, Which, Width, Height, Changed);
      end loop;
      for Choice of Began loop
         Free (Choice);
      end loop;
      Free (R);
   end Settle;

   ---------------------------------------------------------------------------
   --  The start

   --  The K-th of N rotations of the Super-Fibonacci spiral (Alexa, 2022), which covers the rotations evenly: a unit
   --  quaternion of two circles, the angle of one 2 pi s over the square root of two and the angle of the other 2 pi
   --  s over the real root of x^4 = x + 4 (found by Newton's method), s the rotation's place along the spiral.
   function Spiral_Rotation (K, N : Positive) return Mat3 is
      Phi : constant Real := Sqrt (2.0);
      Psi : Real := 1.5;
   begin
      for Step in 1 .. Real'Machine_Mantissa loop
         declare
            Miss : constant Real := Psi ** 4 - Psi - 4.0;
            Slope : constant Real := 4.0 * Psi ** 3 - 1.0;
         begin
            exit when Miss = 0.0 or else abs (Miss / Slope) <= Real'Epsilon * Psi;
            Psi := Psi - Miss / Slope;
         end;
      end loop;
      declare
         S : constant Real := Real (K) - 0.5;
         T : constant Real := S / Real (N);
         D : constant Real := 2.0 * Ada.Numerics.Pi * S;
         A : constant Real := Sqrt (T);
         B : constant Real := Sqrt (1.0 - T);
      begin
         return Driver.Numerics.To_Matrix
           ((W => A * Sin (D / Phi), X => A * Cos (D / Phi), Y => B * Sin (D / Psi), Z => B * Cos (D / Psi)));
      end;
   end Spiral_Rotation;

   --  The skew matrix of the line of sight B, as a Mat3.
   function Cross_Of (B : Vec3) return Mat3 is (Driver.Numerics.Skew (B));

   --  For the rotation R of the eye's frame into arm A's (the lens L being known), the eye's centre in the arm's
   --  frame and the depth of every track in Tracks that best explain the sightings of those tracks, in least squares
   --  on the angle between each sighting's line of sight and the point where the link's motion put the point: linear
   --  in both (the line of sight B of the pixel it was seen at, and the point at the keyframe in the eye's frame, z Q H
   --  + Transpose (R) ((A - I) C + t) with Q = Transpose (R) A R, H the track's reference line of sight, z its
   --  depth, A and t the link's motion: B x that is zero). The depths are eliminated track by track; Depth is zero for
   --  a track no sighting of which is in the set; Cost is the robust cost the answer has in pixels, at the scale
   --  a pixel's rounding and each sighting's own state give; a centre or a depth behind the eye is as wrong as the
   --  picture is big.
   procedure Solve_At
     (P        : Problem;
      Arm      : Positive;
      Pose     : Pose_Value;            --  the rotation R; the centre is solved
      L        : Fit.Lens;
      Link     : Fit.Count_Array;       --  of every track; 0 leaves it out
      Width, Height : Positive;
      Centre   : out Vec3;
      Depth    : out Real_Array;        --  per track
      Score    : out Real;
      Ok       : out Boolean)
   is
      Ri     : constant Mat3 := Transpose (Pose.R);
      Saa    : Real_Array (1 .. P.Tracks) := [others => 0.0];
      Sag    : Real_Matrix (1 .. P.Tracks, 1 .. 3) := [others => [others => 0.0]];
      Sar    : Real_Array (1 .. P.Tracks) := [others => 0.0];
      H3     : Mat3 := [others => [others => 0.0]];
      G3     : Vec3 := Zero3;
      Anything : Boolean := False;
   begin
      Centre := Zero3;
      Depth := [others => 0.0];
      Score := Real'First;
      Ok := False;
      for I in 1 .. P.Sightings loop
         declare
            T : constant Positive := P.Sight_Track (I);
         begin
            if P.Sight_Arm (I) = Arm and then On_Arm (Link, T) then
               declare
                  Move : constant Rigid := Move_Of (P, Arm, Link (T), I);
                  B    : constant Vec3 := Fit.Ray (L, P.Sight_U (I), P.Sight_V (I));
                  H    : constant Vec3 := Fit.Ray (L, P.U0 (T), P.V0 (T));
                  Cr   : constant Mat3 := Cross_Of (B);
                  Q    : constant Mat3 := Ri * Move.Rotation * Pose.R;
                  U    : constant Vec3 := Cr * (Q * H);
                  M    : constant Mat3 := Cr * (Ri * (Move.Rotation - Identity3));
                  C    : constant Vec3 := Cr * (Ri * Move.Translation);
               begin
                  Anything := True;
                  Saa (T) := Saa (T) + U * U;
                  Sar (T) := Sar (T) + U * C;
                  for A in 1 .. 3 loop
                     Sag (T, A) := Sag (T, A) + U * [M (1, A), M (2, A), M (3, A)];
                     G3 (A) := G3 (A) + [M (1, A), M (2, A), M (3, A)] * C;
                     for B2 in 1 .. 3 loop
                        H3 (A, B2) := H3 (A, B2) + [M (1, A), M (2, A), M (3, A)] * [M (1, B2), M (2, B2), M (3, B2)];
                     end loop;
                  end loop;
               end;
            end if;
         end;
      end loop;
      if not Anything then
         return;
      end if;
      for T in 1 .. P.Tracks loop
         if Saa (T) > 0.0 then
            for A in 1 .. 3 loop
               G3 (A) := G3 (A) - Sag (T, A) * Sar (T) / Saa (T);
               for B2 in 1 .. 3 loop
                  H3 (A, B2) := H3 (A, B2) - Sag (T, A) * Sag (T, B2) / Saa (T);
               end loop;
            end loop;
         end if;
      end loop;
      declare
         Lower : Mat3;
         Pd    : Boolean;
      begin
         Driver.Numerics.Dense.Cholesky (H3, Lower, Pd);
         if not Pd then
            return;
         end if;
         Centre := Driver.Numerics.Dense.Cholesky_Solve (Lower, -G3);
      end;
      for T in 1 .. P.Tracks loop
         if Saa (T) > 0.0 then
            Depth (T) := -(Sar (T) + Sag (T, 1) * Centre (1) + Sag (T, 2) * Centre (2) + Sag (T, 3) * Centre (3))
                         / Saa (T);
         end if;
      end loop;
      declare
         S      : State (P.Arms);
         Placed : Fit.Flag_Array (1 .. P.Tracks);
         Lz     : Real_Array (1 .. P.Tracks);
         Model  : Fit.Error_Model := (Floor => -1.0, Gain => 1.0);
      begin
         S.L := L;
         S.Pose (Arm) := (R => Pose.R, C => Centre);
         for T in 1 .. P.Tracks loop
            Placed (T) := Depth (T) > 0.0;
            Lz (T) := (if Placed (T) then Ln (Depth (T)) else 0.0);
         end loop;
         Likelihood (P, S, Link, Lz, Placed, Width, Height, Model, Score);
         Ok := True;
      end;
   end Solve_At;

   --  The terms of a stage: the lens's (without the distortion's unless Distortion) when the lens is free, and the
   --  poses of the arms that count.
   function Terms_For (Active : Fit.Flag_Array; Lens_Free, Distortion : Boolean) return Free_Terms is
      Lens_Count : constant Natural := (if not Lens_Free then 0 elsif Distortion then Fit.Lens_Terms else 4);
      Count      : Natural := Lens_Count;
   begin
      for A in Active'Range loop
         if Active (A) then
            Count := Count + Pose_Terms;
         end if;
      end loop;
      declare
         Result : Free_Terms (1 .. Count);
         K      : Natural := 0;
      begin
         for T in 1 .. Lens_Count loop
            K := K + 1;
            Result (K) := T;
         end loop;
         for A in Active'Range loop
            if Active (A) then
               for T in 1 .. Pose_Terms loop
                  K := K + 1;
                  Result (K) := Pose_At (A - Active'First + 1) + T;
               end loop;
            end if;
         end loop;
         return Result;
      end;
   end Terms_For;

   --  The state an arm's start gives, with the tracks of the start set at the depths it solved.
   procedure Start_From
     (P      : Problem;
      Arm    : Positive;
      S      : in out State;
      W      : in out Work;
      Pose   : Pose_Value;
      Centre : Vec3;
      Depth  : Real_Array;
      Link   : Fit.Count_Array)
   is
   begin
      S.Pose (Arm) := (R => Pose.R, C => Centre);
      for T in 1 .. P.Tracks loop
         if P.Track_Arm (T) = Arm then
            W.Link (T) := (if Depth (T) > 0.0 then Link (T) else 0);
            W.Lz (T) := (if Depth (T) > 0.0 then Ln (Depth (T)) else 0.0);
         end if;
      end loop;
   end Start_From;

   ---------------------------------------------------------------------------
   --  Which link each track rides on, from the model

   --  Track T's sightings under the hypothesis that it rides on the link Hyp (0: it stays where it was seen first), at
   --  the eye the state gives: the depth along its reference line of sight that suits the hypothesis best, and the cost
   --  the sightings have there. The depths the sightings themselves propose (each one's own, the depth at which the
   --  moved point comes nearest its line of sight) are tried, the best of them is refined by reweighted least squares
   --  (the weights Huber's at the scales) until the depth changes by less than the unchanged fraction, and the better
   --  of the two is kept. Nothing is proposed (Cost is Real'Last) when no sighting proposes a depth in front of the
   --  eye.
   --  The sightings of the track are Order (First .. First + N - 1).
   procedure Under_Hypothesis
     (P             : Problem;
      S             : State;
      W             : Work;
      T             : Positive;
      Hyp           : Natural;
      Order         : Fit.Count_Array;
      First, N      : Positive;
      Behind        : Real;
      Lz            : out Real;
      Cost          : out Real)
   is
      A      : constant Positive := P.Track_Arm (T);
      Pose   : Pose_Value renames S.Pose (A);
      H      : constant Vec3 := Fit.Ray (S.L, P.U0 (T), P.V0 (T));
      Along  : Vec_Store := new Vec_Array (1 .. N);   --  per sighting, the moved point at depth one, less the shift
      By     : Vec_Store := new Vec_Array (1 .. N);   --  and the shift
      Ang_U  : Vec_Store := new Vec_Array (1 .. N);   --  the same crossed with the sighting's line of sight
      Ang_C  : Vec_Store := new Vec_Array (1 .. N);

      function Cost_At (Z : Real) return Real is
         Sum : Real := 0.0;
      begin
         for K in 1 .. N loop
            declare
               I     : constant Positive := Order (First + K - 1);
               U, V  : Real;
               Front : Boolean;
               Scale : constant Real := Scale_Of (P, W, I);
            begin
               Fit.Project (S.L, Z * Along (K) + By (K), U, V, Front);
               Sum := Sum + (if Front
                             then Huber_Cost ((U - P.Sight_U (I)) / Scale) + Huber_Cost ((V - P.Sight_V (I)) / Scale)
                             else 2.0 * Huber_Cost (Behind / Scale));
            end;
         end loop;
         return Sum;
      end Cost_At;
   begin
      Lz := 0.0;
      Cost := Real'Last;
      for K in 1 .. N loop
         declare
            I    : constant Positive := Order (First + K - 1);
            Move : constant Rigid := Move_Of (P, A, Hyp, I);
            Q    : constant Mat3 :=
              Driver.Numerics.Times (Driver.Numerics.Times (Driver.Numerics.Transposed (Pose.R), Move.Rotation), Pose.R);
            Cr   : constant Mat3 := Cross_Of (Fit.Ray (S.L, P.Sight_U (I), P.Sight_V (I)));
         begin
            Along (K) := Driver.Numerics.Times (Q, H);
            By (K) := Driver.Numerics.Transposed_Times
              (Pose.R, Driver.Numerics.Plus (Driver.Numerics.Minus (Driver.Numerics.Times (Move.Rotation, Pose.C), Pose.C),
                                             Move.Translation));
            Ang_U (K) := Driver.Numerics.Times (Cr, Along (K));
            Ang_C (K) := Driver.Numerics.Times (Cr, By (K));
         end;
      end loop;
      if Hyp = 0 then
         declare
            Sum : Real := 0.0;
         begin
            for K in 1 .. N loop
               declare
                  I     : constant Positive := Order (First + K - 1);
                  Scale : constant Real := Scale_Of (P, W, I);
               begin
                  Sum := Sum + Huber_Cost ((P.U0 (T) - P.Sight_U (I)) / Scale)
                             + Huber_Cost ((P.V0 (T) - P.Sight_V (I)) / Scale);
               end;
            end loop;
            Cost := Sum;
         end;
      else
         declare
            Best_Z : Real := 0.0;
         begin
            for K in 1 .. N loop
               if Ang_U (K) * Ang_U (K) > 0.0 then
                  declare
                     Z : constant Real := -(Ang_U (K) * Ang_C (K)) / (Ang_U (K) * Ang_U (K));
                  begin
                     if Z > 0.0 then
                        declare
                           C : constant Real := Cost_At (Z);
                        begin
                           if C < Cost then
                              Cost := C;
                              Best_Z := Z;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end loop;
            if Best_Z > 0.0 then
               declare
                  Z : Real := Best_Z;
               begin
                  for Iteration in 1 .. Real'Machine_Mantissa loop
                     declare
                        Num, Den : Real := 0.0;
                        Next     : Real;
                        Moved_By, Z_Before : Real;
                     begin
                        for K in 1 .. N loop
                           declare
                              I     : constant Positive := Order (First + K - 1);
                              U, V  : Real;
                              Front : Boolean;
                              Scale : constant Real := Scale_Of (P, W, I);
                              Wt    : Real;
                           begin
                              Fit.Project (S.L, Z * Along (K) + By (K), U, V, Front);
                              Wt := (if Front
                                     then Huber (Sqrt ((U - P.Sight_U (I)) ** 2 + (V - P.Sight_V (I)) ** 2) / Scale)
                                     else 0.0);
                              Num := Num + Wt * (Ang_U (K) * Ang_C (K));
                              Den := Den + Wt * (Ang_U (K) * Ang_U (K));
                           end;
                        end loop;
                        exit when Den <= 0.0;
                        Next := -Num / Den;
                        exit when Next <= 0.0;
                        Moved_By := abs (Next - Z);
                        Z_Before := Z;
                        Z := Next;
                        exit when Moved_By <= Driver.Conventions.Unchanged_Fraction * Z_Before;
                     end;
                  end loop;
                  if Cost_At (Z) < Cost then
                     Cost := Cost_At (Z);
                     Best_Z := Z;
                  end if;
               end;
            end if;
            if Best_Z > 0.0 then
               Lz := Ln (Best_Z);
            end if;
         end;
      end if;
      Free (Along);
      Free (By);
      Free (Ang_U);
      Free (Ang_C);
   end Under_Hypothesis;

   --  Every track again, under every hypothesis of what it rides on (nothing: it stays where it was seen first, or the
   --  link after any of its arm's joints), at the eye the state gives: the cost its sightings have at the depth that
   --  suits the hypothesis best (Under_Hypothesis). A track rides on the link of the lowest hypothesis whose cost is
   --  within the unchanged fraction of the least (a joint that no keyframe moved does not tell its link from the one
   --  before: the link before is what is known), and on nothing when that is no link at all or when no more than half
   --  of its sightings fit it (a robust estimate has no more to stand on than a majority), or when it has fewer
   --  sightings than a hypothesis can be tested by (one more than its depth absorbs). Only the tracks of the arms that
   --  Active says have an eye are looked at. A track that keeps its link keeps its depth. Changed is how many tracks
   --  took another link than they had.
   procedure Reassign
     (P             : Problem;
      S             : State;
      Active        : Fit.Flag_Array;
      W             : in out Work;
      Width, Height : Positive;
      Changed       : out Natural)
   is
      Behind : constant Real := Real (Width + Height);
      Counts : Fit.Count_Array (1 .. P.Tracks) := [others => 0];
      First  : Fit.Count_Array (1 .. P.Tracks + 1);
      Order  : Count_Access := new Fit.Count_Array (1 .. P.Sightings);
   begin
      Changed := 0;
      for I in 1 .. P.Sightings loop
         Counts (P.Sight_Track (I)) := Counts (P.Sight_Track (I)) + 1;
      end loop;
      First (1) := 1;
      for T in 1 .. P.Tracks loop
         First (T + 1) := First (T) + Counts (T);
      end loop;
      declare
         Fill : Fit.Count_Array (1 .. P.Tracks) := [others => 0];
      begin
         for I in 1 .. P.Sightings loop
            declare
               T : constant Positive := P.Sight_Track (I);
            begin
               Order (First (T) + Fill (T)) := I;
               Fill (T) := Fill (T) + 1;
            end;
         end loop;
      end;
      for T in 1 .. P.Tracks loop
         if Active (P.Track_Arm (T)) then
            declare
               A         : constant Positive := P.Track_Arm (T);
               N         : constant Natural := Counts (T);
               Cost_Of_H : Hypothesis_Array (0 .. P.Joints (A)) := [others => Real'Last];
               Lz_Of_H   : Hypothesis_Array (0 .. P.Joints (A)) := [others => 0.0];
               Least     : Real := Real'Last;
               Pick      : Natural := 0;
            begin
               if N >= 2 then
                  for Hyp in 0 .. P.Joints (A) loop
                     Under_Hypothesis
                       (P, S, W, T, Hyp, Order.all, First (T), N, Behind, Lz_Of_H (Hyp), Cost_Of_H (Hyp));
                     Least := Real'Min (Least, Cost_Of_H (Hyp));
                  end loop;
                  for Hyp in Cost_Of_H'Range loop
                     if Cost_Of_H (Hyp) <= Least * (1.0 + Driver.Conventions.Unchanged_Fraction) then
                        Pick := Hyp;
                        exit;
                     end if;
                  end loop;
                  if Pick >= 1 then
                     --  How many of the sightings fit the pick: neither residual significant against its scale.
                     declare
                        Z       : constant Real := Exp (Lz_Of_H (Pick));
                        H       : constant Vec3 := Fit.Ray (S.L, P.U0 (T), P.V0 (T));
                        Fitting : Natural := 0;
                     begin
                        for K in 1 .. N loop
                           declare
                              I     : constant Positive := Order (First (T) + K - 1);
                              Move  : constant Rigid := Move_Of (P, A, Pick, I);
                              Pose  : Pose_Value renames S.Pose (A);
                              Xkc   : constant Vec3 :=
                                Driver.Numerics.Transposed_Times
                                  (Pose.R, Driver.Numerics.Minus
                                             (Move * Driver.Numerics.Plus (Driver.Numerics.Times (Pose.R, Driver.Numerics.Scaled (Z, H)),
                                                                           Pose.C), Pose.C));
                              U, V  : Real;
                              Front : Boolean;
                              Scale : constant Real := Scale_Of (P, W, I);
                           begin
                              Fit.Project (S.L, Xkc, U, V, Front);
                              if Front and then not Driver.Uncertain.Significant (U - P.Sight_U (I), Scale)
                                and then not Driver.Uncertain.Significant (V - P.Sight_V (I), Scale)
                              then
                                 Fitting := Fitting + 1;
                              end if;
                           end;
                        end loop;
                        if 2 * Fitting <= N then
                           Pick := 0;
                        end if;
                     end;
                  end if;
               end if;
               if Pick /= W.Link (T) then
                  Changed := Changed + 1;
                  W.Link (T) := Pick;
                  if Pick >= 1 then
                     W.Lz (T) := Lz_Of_H (Pick);
                  end if;
               end if;
            end;
         end if;
      end loop;
      Free (Order);
   end Reassign;

   --  The refinement of the terms Which over the arms Active and the reading of the tracks' links from the model, each
   --  from the other, for as long as the likelihood of the sightings of the tracks that were on a link when it
   --  began rises (the links change a track's link or take it from the arm, and a round that loses likelihood goes
   --  round in a circle); the state and the work of the round with the most come back. Ends when a reading of the
   --  links changes none.
   procedure Settle_Links
     (P             : Problem;
      S             : in out State;
      Active        : Fit.Flag_Array;
      W             : in out Work;
      Which         : Free_Terms;
      Width, Height : Positive)
   is
      Set     : constant Fit.Count_Array := W.Link;
      Placed  : Fit.Flag_Array (1 .. P.Tracks);
      Best_S  : State (P.Arms) := S;
      Best_W  : Work_Access := new Work (P.Tracks, P.Sightings);
      Last    : Real := Real'First;
      Changed : Natural;
   begin
      Best_W.all := W;
      loop
         Settle (P, S, W, Which, Width, Height);
         declare
            Model : Fit.Error_Model := (Floor => -1.0, Gain => 1.0);
            Now   : Real;
         begin
            for T in 1 .. P.Tracks loop
               Placed (T) := W.Link (T) >= 1;
            end loop;
            Likelihood (P, S, Set, W.Lz, Placed, Width, Height, Model, Now);
            exit when Now <= Last;
            Last := Now;
         end;
         Best_S := S;
         Best_W.all := W;
         Reassign (P, S, Active, W, Width, Height, Changed);
         exit when Changed = 0;
      end loop;
      S := Best_S;
      W := Best_W.all;
      Release (Best_W);
   end Settle_Links;

   --  The normal equations with the depths eliminated: what the sightings leave each combination of the terms known.
   function Eliminated (A : Real_Matrix; Dd : Real_Array; Dg : Real_Matrix) return Real_Matrix is
      Result : Real_Matrix := A;
   begin
      for T in Dd'Range loop
         if Dd (T) > 0.0 then
            for Q in A'Range (1) loop
               for B in A'Range (2) loop
                  Result (Q, B) := Result (Q, B) - Dg (T, Q) * Dg (T, B) / Dd (T);
               end loop;
            end loop;
         end if;
      end loop;
      return Result;
   end Eliminated;

   --  The reduced normal equations of the terms Which at the state S, over the sightings that fit at its residuals.
   function Information (P : Problem; S : State; W : in out Work; Which : Free_Terms) return Real_Matrix is
      Count : constant Natural := Which'Length;
      R     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Jg    : Matrix_Access := new Real_Matrix (1 .. 2 * P.Sightings, 1 .. Count);
      Jd    : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      A     : Real_Matrix (1 .. Count, 1 .. Count);
      G     : Real_Vector (1 .. Count);
      Dd    : Real_Array (1 .. P.Tracks);
      Dg    : Matrix_Access := new Real_Matrix (1 .. P.Tracks, 1 .. Count);
      Gd    : Real_Array (1 .. P.Tracks);
      Same  : Boolean;
   begin
      Residuals_Only (P, S, W.Link, W.Lz, W.Seen, R.all);
      Choose (P, W, R.all, Same);
      Linearize (P, S, W.Link, W.Lz, Which, W.Seen, R.all, Jg.all, Jd.all);
      Normals (P, W, R.all, Jg.all, Jd.all, A, G, Dd, Dg.all, Gd);
      Free (R);
      Free (Jg);
      Free (Jd);
      return Result : constant Real_Matrix := Eliminated (A, Dd, Dg.all) do
         Free (Dg);
      end return;
   end Information;

   --  How far the state To is from the state From in the terms Which, each in the units the fit moves it by: the
   --  logarithms of the focal lengths, pixels, the turn about the eye's own axes (a rotation vector) and the centre.
   function Between (From, To : State; Which : Free_Terms) return Real_Vector is
      Result : Real_Vector (1 .. Which'Length);
   begin
      for K in Which'Range loop
         declare
            Term : constant Positive := Which (K);
            Place : constant Positive := K - Which'First + 1;
         begin
            if Term <= Fit.Lens_Terms then
               Result (Place) :=
                 (case Term is
                     when 1      => Ln (To.L.Fx / From.L.Fx),
                     when 2      => Ln (To.L.Fy / From.L.Fy),
                     when 3      => To.L.Cx - From.L.Cx,
                     when 4      => To.L.Cy - From.L.Cy,
                     when 5      => To.L.K1 - From.L.K1,
                     when others => To.L.K2 - From.L.K2);
            else
               Result (Place) := 0.0;
               for A in 1 .. From.Arms loop
                  if Term > Pose_At (A) and then Term <= Pose_At (A) + Pose_Terms then
                     declare
                        Own  : constant Positive := Term - Pose_At (A);
                        Turn : constant Vec3 := Driver.Numerics.Log (Transpose (From.Pose (A).R) * To.Pose (A).R);
                     begin
                        Result (Place) := (if Own <= 3 then Turn (Own) else To.Pose (A).C (Own - 3) - From.Pose (A).C (Own - 3));
                     end;
                  end if;
               end loop;
            end if;
         end;
      end loop;
      return Result;
   end Between;

   --  How many squared standard errors the state To is from the state From, by what the sightings leave From's terms
   --  known: the chi square of the difference. Two starts that end this near each other, in the terms of the fit, have
   --  ended at one optimum, whatever differs in the links between the points and the arm's joints that each ended with.
   function Distance_Squared (P : Problem; From : State; From_Work : Work; To : State; Which : Free_Terms) return Real is
      Copy   : Work_Access := new Work'(From_Work);
      A      : constant Real_Matrix := Information (P, From, Copy.all, Which);
      D      : constant Real_Vector := Between (From, To, Which);
   begin
      Release (Copy);
      return D * (A * D);
   end Distance_Squared;

   type State_Access is access State;
   procedure Free is new Ada.Unchecked_Deallocation (State, State_Access);

   --  A start of the search on its way to an optimum: its state and work, the damping its refinement has come to, whether
   --  that has converged on the sightings chosen, and how likely the sightings are at it.
   type Racer is record
      S      : State_Access;
      W      : Work_Access;
      Lambda : Real := Driver.Conventions.Initial_Damping;
      Done   : Boolean := False;
      Alive  : Boolean := True;
      Score  : Real := Real'First;
   end record;

   type Racer_Array is array (Positive range <>) of Racer;
   type Racer_Access is access Racer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Racer_Array, Racer_Access);

   --  The focal lengths a search scores its starts at, the lens being unknown, and every rotation's score at each
   --  (Solve_At), column by column. From the half width seen at a right angle's half, where the arm's own fit starts
   --  its focal search (Fit.Fit), the lengths stand half an octave apart, as that search's first bracket is an
   --  octave about its start; the scan widens by a length on the side whose end the best rotation scores best at, as
   --  that search widens its bracket, until a length inside scores at least as well as both ends: the basin of the
   --  best start is bracketed, and the race refines the lens from there. No field of view is assumed: the scan used
   --  to stop at lenses seeing 40 and 140 degrees across, the range a robot's camera was taken to have, every
   --  1.25 times the one before.
   type Focal_Scan (Rotations, Focals : Natural) is record
      Focal  : Real_Array (1 .. Focals);
      Scores : Real_Matrix (1 .. Rotations, 1 .. Focals);
   end record;
   type Scan_Access is access Focal_Scan;
   procedure Free is new Ada.Unchecked_Deallocation (Focal_Scan, Scan_Access);

   function Scan_Focals
     (P : Problem; Arm : Positive; Start_Link : Fit.Count_Array; Rotations : Positive; Width, Height : Positive)
      return Scan_Access
   is
      package Columns is new Ada.Containers.Indefinite_Ordered_Maps (Integer, Real_Array);
      Half_Octave : constant Real := Sqrt (2.0);
      Start       : constant Real := Real (Width) / 2.0;
      Scanned     : Columns.Map;
      Low, High   : Integer := 0;   --  the steps scanned, the length at step K being Start * Half_Octave ** K

      function Focal (Step : Integer) return Real is (Start * Half_Octave ** Step);

      --  Every rotation's start at once (Driver.Parallel_For): each solves its own, from the problem it only reads.
      procedure Score (Step : Integer) is
         L        : constant Fit.Lens :=
           (Fx | Fy => Focal (Step), Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, K1 | K2 => 0.0);
         Column   : Real_Array (1 .. Rotations);
         procedure One (K : Positive) is
            Depth    : Real_Access := new Real_Array (1 .. P.Tracks);
            Centre   : Vec3;
            Score_At : Real;
            Ok       : Boolean;
         begin
            Solve_At (P, Arm, (R => Spiral_Rotation (K, Rotations), C => Zero3), L, Start_Link, Width, Height,
                      Centre, Depth.all, Score_At, Ok);
            Column (K) := (if Ok then Score_At else Real'First);
            Free (Depth);
         end One;
         procedure Every is new Driver.Parallel_For (One);
      begin
         Every (1, Rotations);
         Scanned.Insert (Step, Column);
      end Score;

      --  The best rotation's score at the step.
      function Best (Step : Integer) return Real is
         Column : constant Real_Array := Scanned.Element (Step);
      begin
         return Most : Real := Real'First do
            for X of Column loop
               Most := Real'Max (Most, X);
            end loop;
         end return;
      end Best;
   begin
      Low := -1;
      High := 1;
      for Step in Low .. High loop
         Score (Step);
      end loop;
      loop
         declare
            Inside : Real := Real'First;
         begin
            for Step in Low + 1 .. High - 1 loop
               Inside := Real'Max (Inside, Best (Step));
            end loop;
            exit when Inside >= Best (Low) and then Inside >= Best (High);
            if Best (Low) >= Best (High) then
               exit when abs Ln (Focal (Low - 1)) >= Ln (Real'Last) / 2.0;
               Low := Low - 1;
               Score (Low);
            else
               exit when abs Ln (Focal (High + 1)) >= Ln (Real'Last) / 2.0;
               High := High + 1;
               Score (High);
            end if;
         end;
      end loop;
      declare
         Result : constant Scan_Access := new Focal_Scan (Rotations, High - Low + 1);
      begin
         for Step in Low .. High loop
            Result.Focal (Step - Low + 1) := Focal (Step);
            for K in 1 .. Rotations loop
               Result.Scores (K, Step - Low + 1) := Scanned.Element (Step) (K);
            end loop;
         end loop;
         return Result;
      end;
   end Scan_Focals;

   --  How a search for an arm's start ended: with no candidate that a start could be made of, with starts that no two
   --  ended at one optimum, with two eyes that explain the points alike, or with the start.
   type Search_Verdict is (No_Start, Not_Reproduced, Ambiguous, Found_One);

   --  The starts that must end at one optimum for it to be found, and the finalists of a race: twice them.
   Agreeing  : constant := 2;
   Finalists : constant := 2 * Agreeing;

   --  The start of arm Arm from the tracks in Start_Link (the link of each: 0 leaves it out), by the search over
   --  rotations (and focal lengths unless the lens is given). The candidates are ranked by what their best centre and
   --  depths leave; as many as there are rotations start a race. Each racer takes a step of its refinement (the arm's
   --  pose, and the lens too unless it is given), the sightings it fits chosen anew at every step; then the less
   --  likely half of them is dropped and the rest take twice as many steps, until the finalists are left (twice the
   --  starts that must end at one optimum): a start that is hopeless is dropped after a few steps, and one that
   --  is not is followed to its end. Each finalist is then refined to its end, the tracks' links read from the model
   --  as it goes. The most likely of the optima they end at is the start. A start off by less than the radius the
   --  refinement converges from ends at the optimum, and the sample of rotations is as many as make the chance of none
   --  within that radius a tail of Z. Another optimum, a local one that many starts end at, can be the most likely of
   --  the first few, which is why the starts race and are not taken one by one. The state and the work come back as the
   --  most likely optimum has them (the arm's pose, the lens unless it was given, the arm's tracks at the links and
   --  depths of the start) when the verdict is Found_One: two finalists ended at it, an optimum that only one start
   --  led to being no more than a start that ended somewhere, whatever its likelihood, and no finalist ended at
   --  another that the sightings do not tell from it (Ambiguous: the points would fit that eye as well).
   procedure Search_At
     (P             : Problem;
      Arm           : Positive;
      Start_Link    : Fit.Count_Array;
      Lens_Known    : Boolean;
      Width, Height : Positive;
      Rotations     : Positive;
      S             : in out State;
      W             : in out Work;
      Verdict       : out Search_Verdict;
      Best_S        : out State_Access;
      Best_W        : out Work_Access)
   is
      Scan      : Scan_Access :=
        (if Lens_Known then null else Scan_Focals (P, Arm, Start_Link, Rotations, Width, Height));
      Focals    : constant Real_Array := (if Lens_Known then [S.L.Fx] else Scan.Focal);
      Scores    : Matrix_Access := new Real_Matrix (1 .. Rotations, 1 .. Focals'Length);
      Taken     : Fit.Flag_Array (1 .. Rotations * Focals'Length) := [others => False];
      Active    : Fit.Flag_Array (1 .. P.Arms) := [others => False];
      Which     : Free_Terms (1 .. (if Lens_Known then 0 else 4) + Pose_Terms);
      Racers    : Racer_Access;
      Count     : Natural := 0;
      Picked_K  : Fit.Count_Array (1 .. Rotations) := [others => 0];   --  each racer's rotation and focal length
      Picked_M  : Fit.Count_Array (1 .. Rotations) := [others => 0];

      function Lens_At (M : Positive) return Fit.Lens is
        (if Lens_Known then S.L
         else (Fx => Focals (M), Fy => Focals (M), Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, K1 => 0.0,
               K2 => 0.0));

      --  A racer's likelihood at the state it is at, and the error model it was judged by: over every track the start was
      --  given, one it lost for want of a depth in front of the eye an outlier throughout.
      procedure Likelihood_Of (State_Now : State; Work_Now : Work; Model : in out Fit.Error_Model; Total : out Real) is
         Placed : Fit.Flag_Array (1 .. P.Tracks);
      begin
         for T in 1 .. P.Tracks loop
            Placed (T) := Work_Now.Link (T) >= 1;
         end loop;
         Likelihood (P, State_Now, Start_Link, Work_Now.Lz, Placed, Width, Height, Model, Total);
      end Likelihood_Of;

      --  The racer takes one step: the sightings it fits chosen anew, and a step of its refinement unless it has
      --  converged on that choice.
      procedure Advance (Racing : in out Racer) is
         R     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
         Same  : Boolean;
         Moved : Boolean;
      begin
         Residuals_Only (P, Racing.S.all, Racing.W.Link, Racing.W.Lz, Racing.W.Seen, R.all);
         Choose (P, Racing.W.all, R.all, Same);
         if not (Racing.Done and then Same) then
            Refine_Step (P, Racing.S.all, Racing.W.all, Which, Width, Height, Racing.Lambda, Moved, Racing.Done);
         end if;
         Free (R);
      end Advance;

   begin
      Active (Arm) := True;
      Which := Terms_For (Active, not Lens_Known, False);
      Verdict := No_Start;
      Best_S := null;
      Best_W := null;
      if Lens_Known then
         declare
            procedure One (K : Positive) is
               Depth    : Real_Access := new Real_Array (1 .. P.Tracks);
               Centre   : Vec3;
               Score_At : Real;
               Ok       : Boolean;
            begin
               for M in Focals'Range loop
                  Solve_At (P, Arm, (R => Spiral_Rotation (K, Rotations), C => Zero3), Lens_At (M), Start_Link, Width,
                            Height, Centre, Depth.all, Score_At, Ok);
                  Scores (K, M) := (if Ok then Score_At else Real'First);
               end loop;
               Free (Depth);
            end One;
            procedure Every is new Driver.Parallel_For (One);
         begin
            Every (1, Rotations);
         end;
      else
         Scores.all := Scan.Scores;
         Free (Scan);
      end if;
      Racers := new Racer_Array (1 .. Rotations);
      --  The best-ranked candidates start the race.
      while Count < Rotations loop
         declare
            Pick_K, Pick_M : Natural := 0;
            Most           : Real := Real'First;
         begin
            for K in 1 .. Rotations loop
               for M in Focals'Range loop
                  if not Taken ((K - 1) * Focals'Length + M) and then Scores (K, M) > Most then
                     Most := Scores (K, M);
                     Pick_K := K;
                     Pick_M := M;
                  end if;
               end loop;
            end loop;
            exit when Pick_K = 0;
            Taken ((Pick_K - 1) * Focals'Length + Pick_M) := True;
            Count := Count + 1;
            Picked_K (Count) := Pick_K;
            Picked_M (Count) := Pick_M;
         end;
      end loop;
      declare
         procedure Start_One (C : Positive) is
            Pose     : constant Pose_Value := (R => Spiral_Rotation (Picked_K (C), Rotations), C => Zero3);
            Depth    : Real_Access := new Real_Array (1 .. P.Tracks);
            Centre   : Vec3;
            Score_At : Real;
            Ok       : Boolean;
         begin
            Racers (C).S := new State'(S);
            Racers (C).S.L := Lens_At (Picked_M (C));
            Racers (C).W := new Work (P.Tracks, P.Sightings);
            Racers (C).W.Link := [others => 0];
            Racers (C).W.Lz := [others => 0.0];
            Racers (C).W.Seen := [others => False];
            Racers (C).W.Fits := [others => False];
            Racers (C).W.Model := (Floor => Sqrt (Driver.Alignment.Quantization), Gain => 1.0);
            Solve_At (P, Arm, Pose, Racers (C).S.L, Start_Link, Width, Height, Centre, Depth.all, Score_At, Ok);
            Start_From (P, Arm, Racers (C).S.all, Racers (C).W.all, Pose, Centre, Depth.all, Start_Link);
            Free (Depth);
         end Start_One;
         procedure Start_Every is new Driver.Parallel_For (Start_One);
      begin
         Start_Every (1, Count);
      end;
      --  The race: every racer takes Budget steps, the less likely half drops out, the budget doubles.
      declare
         Alive  : Natural := Count;
         Budget : Positive := 1;
      begin
         while Alive > Finalists loop
            declare
               procedure Run_One (C : Positive) is
               begin
                  if Racers (C).Alive then
                     for Step in 1 .. Budget loop
                        Advance (Racers (C));
                     end loop;
                     declare
                        Model : Fit.Error_Model := (Floor => -1.0, Gain => 1.0);
                     begin
                        Likelihood_Of (Racers (C).S.all, Racers (C).W.all, Model, Racers (C).Score);
                     end;
                  end if;
               end Run_One;
               procedure Run_Every is new Driver.Parallel_For (Run_One);
            begin
               Run_Every (1, Count);
            end;
            --  Drop the worse half (the first of equals stay): each racer that has fewer more likely than itself
            --  than the keepers number stays.
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
                             and then (Racers (Other).Score > Racers (C).Score
                                       or else (Racers (Other).Score = Racers (C).Score and then Other < C))
                           then
                              Better := Better + 1;
                           end if;
                        end loop;
                        if Better >= Keep then
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
      --  The finalists to their ends, and then judged together by one floor (the largest of their own: each state's
      --  own floor is read from its residuals, and a difference of floors would count as a difference of fits).
      declare
         Ends   : Natural := 0;
         Ended  : array (1 .. Finalists) of Natural := [others => 0];   --  the racer each finalist is
         Likely : Real_Array (1 .. Finalists) := [others => Real'First];
         Common : Fit.Error_Model := (Floor => -1.0, Gain => 0.0);   --  the widest of the racers' models
         Best   : Natural := 0;
         Hits   : Natural := 0;   --  the finalists that ended at the best optimum
         Rival  : Boolean := False;   --  one that ended elsewhere that the sightings do not tell from it
      begin
         declare
            procedure Settle_One (C : Positive) is
            begin
               if Racers (C).Alive then
                  Settle_Links (P, Racers (C).S.all, Active, Racers (C).W.all, Which, Width, Height);
               end if;
            end Settle_One;
            procedure Settle_Every is new Driver.Parallel_For (Settle_One);
         begin
            Settle_Every (1, Count);
         end;
         for C in 1 .. Count loop
            if Racers (C).Alive then
               Ends := Ends + 1;
               Ended (Ends) := C;
               declare
                  Model : Fit.Error_Model := (Floor => -1.0, Gain => 1.0);
                  Total : Real;
               begin
                  Likelihood_Of (Racers (C).S.all, Racers (C).W.all, Model, Total);
                  Common := (Floor => Real'Max (Common.Floor, Model.Floor), Gain => Real'Max (Common.Gain, Model.Gain));
               end;
            end if;
         end loop;
         for E in 1 .. Ends loop
            declare
               Model : Fit.Error_Model := Common;
            begin
               Likelihood_Of (Racers (Ended (E)).S.all, Racers (Ended (E)).W.all, Model, Likely (E));
            end;
            if Best = 0 or else Likely (E) > Likely (Best) then
               Best := E;
            end if;
         end loop;
         --  Two ends are one optimum when the sightings do not tell the second from the first in the terms of the
         --  fit (Distance_Squared's chi square is not significant). An end that is another optimum that the
         --  likelihood does not set lower than the best by a significant ratio (its chi square over the terms the fit
         --  moved) is a rival: the sightings do not say which of the two eyes is the one.
         if Best > 0 then
            Best_S := new State'(Racers (Ended (Best)).S.all);
            Best_W := new Work'(Racers (Ended (Best)).W.all);
            Verdict := Not_Reproduced;
            for E in 1 .. Ends loop
               declare
                  Apart : constant Real :=
                    (if E = Best then 0.0
                     else Distance_Squared (P, Racers (Ended (Best)).S.all, Racers (Ended (Best)).W.all,
                                            Racers (Ended (E)).S.all, Which));
                  Gap   : constant Real := 2.0 * (Likely (Best) - Likely (E));
               begin
                  if Driver.Distributions.Chi_Square_Deviate (Real'Max (Real'Model_Epsilon, Apart), Which'Length)
                    <= Driver.Conventions.Z
                  then
                     Hits := Hits + 1;
                  elsif Driver.Distributions.Chi_Square_Deviate (Real'Max (Real'Model_Epsilon, Gap), Which'Length)
                    <= Driver.Conventions.Z
                  then
                     Rival := True;
                     --  The rival and the best, for the log: what the two eyes are (focal, centre in the arm's frame).
                     declare
                        function Eye_Image (S : State) return String is
                          ("focal " & Driver.Log.Image (S.L.Fx, 1) & " x " & Driver.Log.Image (S.L.Fy, 1) & ", centre ("
                           & Driver.Log.Image (S.Pose (Arm).C (1), 3) & ", " & Driver.Log.Image (S.Pose (Arm).C (2), 3)
                           & ", " & Driver.Log.Image (S.Pose (Arm).C (3), 3) & "), its axis ("
                           & Driver.Log.Image (S.Pose (Arm).R (1, 3), 3) & ", " & Driver.Log.Image (S.Pose (Arm).R (2, 3), 3)
                           & ", " & Driver.Log.Image (S.Pose (Arm).R (3, 3), 3) & ")");
                     begin
                        Driver.Log.Line (Driver.Log.Robot, "kinematics: a fixed eye's search for arm" & Arm'Image
                                         & ": best " & Eye_Image (Racers (Ended (Best)).S.all) & "; rival "
                                         & Eye_Image (Racers (Ended (E)).S.all) & ", likelihoods "
                                         & Driver.Log.Image (Likely (Best), 1) & " and " & Driver.Log.Image (Likely (E), 1));
                     end;
                  end if;
               end;
            end loop;
            if Rival then
               Verdict := Ambiguous;
            elsif Hits >= Agreeing then
               Verdict := Found_One;
               if not Lens_Known then
                  S.L := Racers (Ended (Best)).S.L;
               end if;
               S.Pose (Arm) := Racers (Ended (Best)).S.Pose (Arm);
               for T in 1 .. P.Tracks loop
                  if P.Track_Arm (T) = Arm then
                     W.Link (T) := Racers (Ended (Best)).W.Link (T);
                     W.Lz (T) := Racers (Ended (Best)).W.Lz (T);
                  end if;
               end loop;
               W.Model := Racers (Ended (Best)).W.Model;
            end if;
         end if;
      end;
      for C in 1 .. Count loop
         Free (Racers (C).S);
         Release (Racers (C).W);
      end loop;
      Free (Racers);
      Free (Scores);
   end Search_At;

   --  The start of arm Arm, the race run at counts of rotations that double (Search_At), from as many as it has
   --  finalists, until a doubling changes nothing: the best optimum of the race is the one of the race before it, at
   --  half the count (Distance_Squared within Z over the terms the fit moves), and so is its verdict, or the race finds
   --  that optimum with two of its starts ending at it (Found_One). The spiral covers the rotations twice as finely
   --  each time, and an optimum whose basin no start of the coarser race fell in shows when one of the finer race
   --  does: a coarse race's two optima the sightings did not tell apart (Ambiguous), or its optimum no two starts
   --  reached (Not_Reproduced), is the coarse race's and not the eye's until a finer one says the same. No basin is
   --  assumed: the count was fixed by the turn the refinement had converged from in two studies (A22 and A79, 60
   --  degrees), the measure of one pair of runs. The count stops doubling at the sightings' number: a start for every
   --  sighting is as fine as the sightings can tell starts apart.
   procedure Search
     (P             : Problem;
      Arm           : Positive;
      Start_Link    : Fit.Count_Array;
      Lens_Known    : Boolean;
      Width, Height : Positive;
      S             : in out State;
      W             : in out Work;
      Verdict       : out Search_Verdict)
   is
      Active      : Fit.Flag_Array (1 .. P.Arms) := [others => False];
      Rotations   : Positive := Finalists;
      Was_S       : State_Access;   --  the best optimum of the race before, its work and its verdict
      Was_W       : Work_Access;
      Was_Verdict : Search_Verdict := No_Start;
      First       : Boolean := True;
   begin
      Active (Arm) := True;
      loop
         declare
            Which  : constant Free_Terms := Terms_For (Active, not Lens_Known, False);
            Try_S  : State := S;
            Try_W  : Work_Access := new Work'(W);
            Now_S  : State_Access;
            Now_W  : Work_Access;
            Same   : Boolean := False;   --  the race found the best optimum of the race before
            Last   : constant Boolean := Rotations >= P.Sightings;
            Holds  : Boolean;
         begin
            Search_At (P, Arm, Start_Link, Lens_Known, Width, Height, Rotations, Try_S, Try_W.all, Verdict, Now_S, Now_W);
            if Now_S /= null and then Was_S /= null and then Was_W /= null then
               Same := Driver.Distributions.Chi_Square_Deviate
                         (Real'Max (Real'Model_Epsilon, Distance_Squared (P, Was_S.all, Was_W.all, Now_S.all, Which)),
                          Which'Length) <= Driver.Conventions.Z;
            end if;
            Holds := Last
              or else (not First
                       and then ((Same and then (Verdict = Found_One or else Verdict = Was_Verdict))
                                 or else (Verdict = No_Start and then Was_Verdict = No_Start)));
            if Verdict = Found_One and then Holds then
               S := Try_S;
               W := Try_W.all;
            end if;
            Release (Try_W);
            if Was_S /= null then
               Free (Was_S);
               Release (Was_W);
            end if;
            Was_S := Now_S;
            Was_W := Now_W;
            Was_Verdict := Verdict;
            First := False;
            exit when Holds;
            Rotations := Natural'Min (2 * Rotations, Natural'Max (Rotations + 1, P.Sightings));
         end;
      end loop;
      if Was_S /= null then
         Free (Was_S);
         Release (Was_W);
      end if;
   end Search;

   ---------------------------------------------------------------------------
   --  How sure the fit is

   --  The cost of the sightings that fit at the state S with every track's depth at the best it can be: the depths by
   --  Newton steps on their own logarithms (a depth moves its own track's residuals and no other's, so one pass of the
   --  residuals with every depth moved a little gives every slope) until none moves by more than the unchanged
   --  fraction, the least cost met on the way (a step that overshoots does not make the profile look higher than it
   --  is).
   function Profile_Cost (P : Problem; S : State; W : Work; Width, Height : Positive) return Real is
      Step     : constant Real := Difference_Step;
      Behind   : constant Real := Real (Width + Height);
      Lz       : Real_Array (1 .. P.Tracks) := W.Lz;
      R        : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Up       : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Down     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Seen     : Fit.Flag_Array (1 .. P.Sightings);
      Least    : Real := Real'Last;
   begin
      for Iteration in 1 .. Real'Machine_Mantissa loop
         declare
            Plus, Minus         : Real_Array (1 .. P.Tracks);
            Gradient, Curvature : Real_Array (1 .. P.Tracks) := [others => 0.0];
            Largest             : Real := 0.0;
         begin
            for T in 1 .. P.Tracks loop
               Plus (T) := Lz (T) + Step;
               Minus (T) := Lz (T) - Step;
            end loop;
            Residuals_Only (P, S, W.Link, Lz, Seen, R.all);
            Least := Real'Min (Least, Cost_Of (P, W, R.all, Behind, Seen));
            Residuals_Only (P, S, W.Link, Plus, Seen, Up.all);
            Residuals_Only (P, S, W.Link, Minus, Seen, Down.all);
            for I in 1 .. P.Sightings loop
               if W.Fits (I) then
                  for C in 0 .. 1 loop
                     declare
                        K     : constant Positive := 2 * I - 1 + C;
                        T     : constant Positive := P.Sight_Track (I);
                        Scale : constant Real := Scale_Of (P, W, I);
                        Slope : constant Real := (Up (K) - Down (K)) / (2.0 * Step) / Scale;
                        Z     : constant Real := R (K) / Scale;
                        Wt    : constant Real := Huber (Z);
                     begin
                        Gradient (T) := Gradient (T) + Wt * Slope * Z;
                        Curvature (T) := Curvature (T) + Wt * Slope * Slope;
                     end;
                  end loop;
               end if;
            end loop;
            for T in 1 .. P.Tracks loop
               if Curvature (T) > 0.0 then
                  Lz (T) := Lz (T) - Gradient (T) / Curvature (T);
                  Largest := Real'Max (Largest, abs (Gradient (T) / Curvature (T)));
               end if;
            end loop;
            exit when Largest <= Driver.Conventions.Unchanged_Fraction;
         end;
      end loop;
      Residuals_Only (P, S, W.Link, Lz, Seen, R.all);
      Least := Real'Min (Least, Cost_Of (P, W, R.all, Behind, Seen));
      Free (R);
      Free (Up);
      Free (Down);
      return Least;
   end Profile_Cost;

   --  Whether the cost is quadratic over the range the covariance gives the terms: in the frame where the normal
   --  equations are the identity (the terms taken through L^T, A = L L^T) the covariance's principal directions are the
   --  eigenvectors of L^T Sigma L, and a displacement of Z sigmas of the normal equations along one, either way,
   --  raises a quadratic cost by Z^2 / 2. Least is the smallest of the square roots of twice the rise the cost has
   --  there (the depths at their best for each displaced state): how many sigmas of the cost itself that displacement
   --  is, Z wherever the cost is quadratic over it and fewer where it flattens. Real'Last when there are no free terms
   --  or the normal equations are not positive definite.
   procedure Linearity
     (P             : Problem;
      S             : State;
      W             : Work;
      Which         : Free_Terms;
      Reduced       : Real_Matrix;
      Cov           : Real_Matrix;
      Width, Height : Positive;
      Least         : out Real)
   is
      Count : constant Natural := Which'Length;
      L     : Real_Matrix (1 .. Count, 1 .. Count);
      Pd    : Boolean;
   begin
      Least := Real'Last;
      Driver.Numerics.Dense.Cholesky (Reduced, L, Pd);
      if not Pd then
         return;
      end if;
      declare
         Sub     : Real_Matrix (1 .. Count, 1 .. Count);
         Values  : Real_Vector (1 .. Count);
         Vectors : Real_Matrix (1 .. Count, 1 .. Count);
         Base    : constant Real := Profile_Cost (P, S, W, Width, Height);
      begin
         for A1 in 1 .. Count loop
            for B1 in 1 .. Count loop
               Sub (A1, B1) := Cov (Which (Which'First + A1 - 1), Which (Which'First + B1 - 1));
            end loop;
         end loop;
         declare
            Whitened : constant Real_Matrix := Transpose (L) * Sub * L;
         begin
            Eigensystem (0.5 * (Whitened + Transpose (Whitened)), Values, Vectors);
         end;
         for K in 1 .. Count loop
            for Way in 0 .. 1 loop
               declare
                  Whole : Real_Vector (1 .. Count);
                  Step  : Real_Vector (1 .. Count) := [others => 0.0];
                  X     : Real_Array (1 .. Terms (S.Arms)) := [others => 0.0];
                  Rise  : Real;
               begin
                  for A1 in 1 .. Count loop
                     Whole (A1) := (if Way = 0 then 1.0 else -1.0) * Driver.Conventions.Z * Vectors (A1, K);
                  end loop;
                  --  The displacement in the terms: L^T Step = Whole, L being lower triangular.
                  for A1 in reverse 1 .. Count loop
                     declare
                        Rest : Real := Whole (A1);
                     begin
                        for B1 in A1 + 1 .. Count loop
                           Rest := Rest - L (B1, A1) * Step (B1);
                        end loop;
                        Step (A1) := Rest / L (A1, A1);
                     end;
                  end loop;
                  for A1 in 1 .. Count loop
                     X (Which (Which'First + A1 - 1)) := Step (A1);
                  end loop;
                  Rise := Profile_Cost (P, Moved_By (S, X), W, Width, Height) - Base;
                  Least := Real'Min (Least, Sqrt (2.0 * Real'Max (0.0, Rise)));
               end;
            end loop;
         end loop;
      end;
   end Linearity;

   --  Each row of the gradient less its track's share along the depth (the depths eliminated), in place.
   procedure Reduce_Rows (P : Problem; Jd : Real_Array; Dd : Real_Array; Dg : Real_Matrix; Jg : in out Real_Matrix) is
   begin
      for I in 1 .. P.Sightings loop
         declare
            T : constant Positive := P.Sight_Track (I);
         begin
            if Dd (T) > 0.0 then
               for C in 0 .. 1 loop
                  for Q in Jg'Range (2) loop
                     Jg (2 * I - 1 + C, Q) := Jg (2 * I - 1 + C, Q) - Jd (2 * I - 1 + C) * Dg (T, Q) / Dd (T);
                  end loop;
               end loop;
            end if;
         end;
      end loop;
   end Reduce_Rows;

   --  What each arm's own uncertainty adds to the middle of the sandwich. The eye is fitted at the joints as the arm's
   --  fit gave them, whose terms are uncertain by the covariance that fit gave; the gradient of the eye's fit moves
   --  with them. Each term of an arm's joints is moved by its standard deviation either way, the arm's moves are made
   --  again, and the response of the (reduced) gradient is the difference of the two over twice the step; the
   --  response, the arm's covariance of the terms and the response again are added to Meat.
   procedure Add_Arm_Spread
     (P    : Problem;
      Arms : Watched_Array;
      S    : State;
      W    : Work;
      R    : Real_Array;
      Rows : Real_Matrix;
      Meat : in out Real_Matrix)
   is
      Count : constant Natural := Rows'Length (2);
   begin
      for Ar in 1 .. P.Arms loop
         declare
            Data  : Watched renames Arms (Arms'First + Ar - 1).all;
            Terms_Count : constant Natural := Fit.Joint_Terms * Data.Joints;
         begin
            if Terms_Count > 0 and then Natural (Data.Covariance.Length) = Terms_Count ** 2 then
               declare
                  Held     : constant Rigid_Grid := P.Moves (Ar).all;
                  Response : Matrix_Access := new Real_Matrix (1 .. Count, 1 .. Terms_Count);
                  Moves    : Fit.Flag_Array (1 .. Terms_Count) := [others => False];
                  Up_R     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
                  Down_R   : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
                  Seen     : Fit.Flag_Array (1 .. P.Sightings);

                  function Variance (Q, B : Positive) return Real is
                    (Data.Covariance (Data.Covariance.First_Index + (Q - 1) * Terms_Count + B - 1));
               begin
                  Response.all := [others => [others => 0.0]];
                  for J in 1 .. Data.Joints loop
                     for Tm in Fit.Joint_Term loop
                        declare
                           Q     : constant Positive := Fit.Joint_Terms * (J - 1) + Fit.Joint_Term'Pos (Tm) + 1;
                           Sigma : constant Real := (if Variance (Q, Q) > 0.0 then Sqrt (Variance (Q, Q)) else 0.0);
                        begin
                           if Sigma > 0.0 then
                              declare
                                 Up_Moves   : Grid_Access :=
                                   Moves_Of (Moved_Joints (Data.Joint, J, Tm, Sigma), Data.Change);
                                 Down_Moves : Grid_Access :=
                                   Moves_Of (Moved_Joints (Data.Joint, J, Tm, -Sigma), Data.Change);
                              begin
                                 Moves (Q) := True;
                                 P.Moves (Ar).all := Up_Moves.all;
                                 Residuals_Only (P, S, W.Link, W.Lz, Seen, Up_R.all);
                                 P.Moves (Ar).all := Down_Moves.all;
                                 Residuals_Only (P, S, W.Link, W.Lz, Seen, Down_R.all);
                                 P.Moves (Ar).all := Held;
                                 Free (Up_Moves);
                                 Free (Down_Moves);
                              end;
                              for I in 1 .. P.Sightings loop
                                 if W.Fits (I) and then P.Sight_Arm (I) = Ar then
                                    for C in 0 .. 1 loop
                                       declare
                                          K     : constant Positive := 2 * I - 1 + C;
                                          Scale : constant Real := Scale_Of (P, W, I);
                                          Slope : constant Real := (Up_R (K) - Down_R (K)) / (2.0 * Sigma) / Scale;
                                          Wt    : constant Real := Huber (R (K) / Scale);
                                       begin
                                          for Qr in 1 .. Count loop
                                             Response (Qr, Q) := Response (Qr, Q)
                                               + Wt * Rows (K, Rows'First (2) + Qr - 1) / Scale * Slope;
                                          end loop;
                                       end;
                                    end loop;
                                 end if;
                              end loop;
                           end if;
                        end;
                     end loop;
                  end loop;
                  for Qr in 1 .. Count loop
                     for Br in 1 .. Count loop
                        for Tq in 1 .. Terms_Count loop
                           if Moves (Tq) and then Response (Qr, Tq) /= 0.0 then
                              for Tb in 1 .. Terms_Count loop
                                 if Moves (Tb) then
                                    Meat (Qr, Br) := Meat (Qr, Br)
                                      + Response (Qr, Tq) * Response (Br, Tb) * Variance (Tq, Tb);
                                 end if;
                              end loop;
                           end if;
                        end loop;
                     end loop;
                  end loop;
                  Free (Response);
                  Free (Up_R);
                  Free (Down_R);
               end;
            end if;
         end;
      end loop;
   end Add_Arm_Spread;

   --  The covariance of the terms Which at S, as a matrix of all the terms (zeros where one is not free): the
   --  sandwich of the normal equations (the depths eliminated) around the spread of the gradient. The spread has two
   --  parts. One is the residuals': their covariance as a falling function of the distance between the points'
   --  reference pixels, read from the residuals themselves (Errors), applied to every pair of gradient rows. The
   --  other is what every sighting of an arm has in common, the arm's own uncertainty (Add_Arm_Spread). Ok is False
   --  when the normal equations are not positive definite or the errors' dependence cannot be told, and Why says
   --  which. Reduced is the reduced normal matrix over Which, for the test of the cost's linearity, and White the
   --  mean square of the residuals in units of their scales.
   procedure Covariance_At
     (P       : Problem;
      Arms    : Watched_Array;
      S       : State;
      W       : in out Work;
      Which   : Free_Terms;
      Cov     : out Real_Matrix;
      Reduced : out Real_Matrix;
      White   : out Real;
      Ok      : out Boolean;
      Why     : in out Ada.Strings.Unbounded.Unbounded_String)
   is
      Count : constant Natural := Which'Length;
      R     : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      Jg    : Matrix_Access := new Real_Matrix (1 .. 2 * P.Sightings, 1 .. Count);
      Jd    : Real_Access := new Real_Array (1 .. 2 * P.Sightings);
      A     : Real_Matrix (1 .. Count, 1 .. Count);
      G     : Real_Vector (1 .. Count);
      Dd    : Real_Array (1 .. P.Tracks);
      Dg    : Matrix_Access := new Real_Matrix (1 .. P.Tracks, 1 .. Count);
      Gd    : Real_Array (1 .. P.Tracks);
      Same  : Boolean;
      Used  : Natural := 0;
      Equal : Real_Array (1 .. Count) := [others => 0.0];
      Scaled, Lower, Inverse : Real_Matrix (1 .. Count, 1 .. Count);
      Pd    : Boolean;
   begin
      Cov := [others => [others => 0.0]];
      Reduced := [others => [others => 0.0]];
      White := 0.0;
      Ok := False;
      Residuals_Only (P, S, W.Link, W.Lz, W.Seen, R.all);
      Choose (P, W, R.all, Same);
      Linearize (P, S, W.Link, W.Lz, Which, W.Seen, R.all, Jg.all, Jd.all);
      Normals (P, W, R.all, Jg.all, Jd.all, A, G, Dd, Dg.all, Gd);
      for I in 1 .. P.Sightings loop
         if W.Fits (I) then
            Used := Used + 1;
         end if;
      end loop;
      --  The reduced normal equations, each term in units of the standard error it would have were the others known.
      Reduced := Eliminated (A, Dd, Dg.all);
      for Q in 1 .. Count loop
         Equal (Q) := (if Reduced (Q, Q) > 0.0 then 1.0 / Sqrt (Reduced (Q, Q)) else 0.0);
      end loop;
      for Q in 1 .. Count loop
         for B in 1 .. Count loop
            Scaled (Q, B) := Equal (Q) * Reduced (Q, B) * Equal (B);
         end loop;
      end loop;
      Driver.Numerics.Dense.Cholesky (Scaled, Lower, Pd);
      Pd := Pd and then (for all E of Equal => E > 0.0);
      if not Pd then
         Ada.Strings.Unbounded.Append
           (Why, "the points' views do not determine the eye's lens and pose (the normal equations are singular)");
      else
         for Q in 1 .. Count loop
            declare
               E : Real_Vector (1 .. Count) := [others => 0.0];
            begin
               E (Q) := 1.0;
               declare
                  Column : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
               begin
                  for B in 1 .. Count loop
                     Inverse (B, Q) := Equal (B) * Column (B) * Equal (Q);
                  end loop;
               end;
            end;
         end loop;
         Reduce_Rows (P, Jd.all, Dd, Dg.all, Jg.all);
         declare
            Rows       : Matrix_Access := new Real_Matrix (1 .. 2 * Used, 1 .. Count);
            Error      : Real_Access := new Real_Array (1 .. 2 * Used);
            Frame      : Driver.Natural_Array (1 .. Used);
            Track      : Driver.Natural_Array (1 .. Used);
            Ref_U      : Real_Access := new Real_Array (1 .. Used);
            Ref_V      : Real_Access := new Real_Array (1 .. Used);
            Frame_Base : Fit.Count_Array (1 .. P.Arms) := [others => 0];
            Taken      : Natural := 0;
            Sum_Sq     : Real := 0.0;
            Meat       : Real_Matrix (1 .. Count, 1 .. Count) := [others => [others => 0.0]];
            Kinds      : Errors.Model;
         begin
            --  The keyframes of different arms are different pictures of the eye: they are numbered on from one arm to
            --  the next.
            for Ar in 2 .. P.Arms loop
               Frame_Base (Ar) := Frame_Base (Ar - 1) + P.Moves (Ar - 1)'Length (2);
            end loop;
            for I in 1 .. P.Sightings loop
               if W.Fits (I) then
                  declare
                     T     : constant Positive := P.Sight_Track (I);
                     Scale : constant Real := Scale_Of (P, W, I);
                  begin
                     Taken := Taken + 1;
                     Frame (Taken) := Frame_Base (P.Sight_Arm (I)) + P.Sight_Frame (I);
                     Track (Taken) := T;
                     Ref_U (Taken) := P.U0 (T);
                     Ref_V (Taken) := P.V0 (T);
                     for C in 0 .. 1 loop
                        declare
                           K : constant Positive := 2 * I - 1 + C;
                           Z : constant Real := R (K) / Scale;
                        begin
                           Error (2 * Taken - 1 + C) := Huber (Z) * R (K);
                           Sum_Sq := Sum_Sq + Z * Z;
                           for Q in 1 .. Count loop
                              Rows (2 * Taken - 1 + C, Q) := Jg (K, Q) / Scale ** 2;
                           end loop;
                        end;
                     end loop;
                  end;
               end if;
            end loop;
            White := (if Used > 0 then Sum_Sq / Real (2 * Used) else 0.0);
            Errors.Measure (Frame, Track, Ref_U.all, Ref_V.all, Error.all, Kinds);
            if not Errors.Measured (Kinds) then
               Ada.Strings.Unbounded.Append
                 (Why, "the sightings are too few to tell how their errors depend on each other");
            else
               Errors.Meat (Kinds, Frame, Track, Ref_U.all, Ref_V.all, Rows.all, Meat);
               Add_Arm_Spread (P, Arms, S, W, R.all, Jg.all, Meat);
               declare
                  Spread     : Real_Matrix (1 .. Count, 1 .. Count);
                  Clipped    : Real;
                  Sandwiched : Boolean;
               begin
                  Fit.Sandwich (Inverse, Meat, Spread, Clipped, Sandwiched, Floor => White);
                  if Sandwiched then
                     Ok := True;
                     for Q in 1 .. Count loop
                        for B in 1 .. Count loop
                           Cov (Which (Which'First + Q - 1), Which (Which'First + B - 1)) := Spread (Q, B);
                        end loop;
                     end loop;
                  else
                     Ada.Strings.Unbounded.Append (Why, "the sightings' covariance has no positive semi-definite form");
                  end if;
               end;
            end if;
            Errors.Free (Kinds);
            Free (Rows);
            Free (Error);
            Free (Ref_U);
            Free (Ref_V);
         end;
      end if;
      Free (R);
      Free (Jg);
      Free (Jd);
      Free (Dg);
   end Covariance_At;

   ---------------------------------------------------------------------------
   --  The fit

   procedure Fit_Eye
     (Arms          : Watched_Array;
      Width, Height : Positive;
      Prior         : Fit_Report;
      Report        : out Fit_Report)
   is
      use Ada.Strings.Unbounded;
      N_Arms : constant Natural := Arms'Length;
      Why    : Unbounded_String;

      --  Every criterion the eye fails is told, the covariance's own first: the sigmas of the others are read from it.
      procedure Say (Text : String) is
      begin
         if Length (Why) > 0 then
            Append (Why, "; ");
         end if;
         Append (Why, Text);
      end Say;
   begin
      Report := (others => <>);
      if N_Arms = 0 then
         Report.Why := To_Unbounded_String ("no arm is watched");
         return;
      end if;
      declare
         P          : Problem_Access := Build (Arms);
         S          : State (N_Arms);
         W          : Work_Access := new Work (P.Tracks, P.Sightings);
         Active     : Fit.Flag_Array (1 .. N_Arms) := [others => False];
         Placed     : Fit.Flag_Array (1 .. N_Arms) := [others => False];
         Lens_Known : Boolean := False;
         Final_W    : Work_Access;

         --  The reports of the arms, from the state and the work the fit ends with.
         procedure Describe_Arms (Final : State; Held : Work) is
            Sight_First : Natural := 0;
         begin
            for A in 1 .. N_Arms loop
               declare
                  Watching : Watched renames Arms (Arms'First + A - 1).all;
                  Entry_Of : Arm_Report;
               begin
                  Entry_Of.Placed := Placed (A);
                  if Active (A) then
                     Entry_Of.Pose := (Rotation => Final.Pose (A).R, Translation => Final.Pose (A).C);
                  end if;
                  for I in 1 .. Watching.Tracks loop
                     declare
                        T : constant Positive := P.First_Track (A) + I;
                     begin
                        Entry_Of.Link.Append (Held.Link (T));
                        Entry_Of.Depth.Append ((if Held.Link (T) >= 1 then Exp (Held.Lz (T)) else 0.0));
                     end;
                  end loop;
                  for I in 1 .. Watching.Sightings loop
                     Entry_Of.Fits.Append (Held.Fits (Sight_First + I));
                  end loop;
                  Sight_First := Sight_First + Watching.Sightings;
                  Report.Arms.Append (Entry_Of);
               end;
            end loop;
         end Describe_Arms;
      begin
         W.Link := [others => 0];
         W.Lz := [others => 0.0];
         W.Seen := [others => False];
         W.Fits := [others => False];
         --  The start of each arm in turn: from the points whose link the keyframes that moved one joint alone tell, by
         --  the search over rotations and, until some arm has given it, over lenses. The arm after the first starts
         --  from the first's lens.
         for A in 1 .. N_Arms loop
            declare
               Watching : Watched renames Arms (Arms'First + A - 1).all;
               Start    : Fit.Count_Array (1 .. P.Tracks) := [others => 0];
               Any      : Boolean := False;
               Found    : Boolean := False;
               Verdict  : Search_Verdict := No_Start;
            begin
               for I in 1 .. Watching.Tracks loop
                  Start (P.First_Track (A) + I) := Infer_Link (Watching, I);
                  Any := Any or else Start (P.First_Track (A) + I) >= 1;
               end loop;
               if Natural (Prior.Arms.Length) >= A and then Prior.Arms (A).Placed
                 and then Natural (Prior.Arms (A).Link.Length) = Watching.Tracks
                 and then Natural (Prior.Arms (A).Depth.Length) = Watching.Tracks
                 and then Prior.L.Fx > 0.0 and then Prior.L.Fy > 0.0
                 and then (not Lens_Known or else (Prior.L.Fx = S.L.Fx and then Prior.L.Fy = S.L.Fy))
               then
                  --  The earlier fit placed this arm: it starts from there, the lens without its distortion (the
                  --  distortion is tried again at the end).
                  S.L := (Fx => Prior.L.Fx, Fy => Prior.L.Fy, Cx => Prior.L.Cx, Cy => Prior.L.Cy, K1 => 0.0, K2 => 0.0);
                  S.Pose (A) := (R => Prior.Arms (A).Pose.Rotation, C => Prior.Arms (A).Pose.Translation);
                  for I in 1 .. Watching.Tracks loop
                     declare
                        T : constant Positive := P.First_Track (A) + I;
                        L : constant Natural := Prior.Arms (A).Link (I);
                        D : constant Real := Prior.Arms (A).Depth (Prior.Arms (A).Depth.First_Index + I - 1);
                     begin
                        W.Link (T) := (if L >= 1 and then D > 0.0 then L else 0);
                        W.Lz (T) := (if L >= 1 and then D > 0.0 then Ln (D) else 0.0);
                     end;
                  end loop;
                  Found := True;
               elsif Any then
                  Search (P.all, A, Start, Lens_Known, Width, Height, S, W.all, Verdict);
                  Found := Verdict = Found_One;
               end if;
               if Found then
                  Active (A) := True;
                  Lens_Known := True;
               else
                  declare
                     --  What the links were read from, told with the reason: per joint the keyframes that turned it
                     --  alone, and the tracks given a link (s38, servo seed 6 of the fake robot, 10-17, replayed: its
                     --  fixed eyes never determined, and nothing said what the links had to go on).
                     Alone  : array (1 .. Watching.Joints) of Natural := [others => 0];
                     Linked : Natural := 0;
                     Per    : Unbounded_String;
                  begin
                     for F in 1 .. Watching.Frames loop
                        declare
                           Moved, Which : Natural := 0;
                        begin
                           for C in 1 .. Watching.Joints loop
                              if Watching.Visible (C) > 0.0 and then abs Watching.Change (F, C) >= Watching.Visible (C) then
                                 Moved := Moved + 1;
                                 Which := C;
                              end if;
                           end loop;
                           if Moved = 1 then
                              Alone (Which) := Alone (Which) + 1;
                           end if;
                        end;
                     end loop;
                     for I in 1 .. Watching.Tracks loop
                        if Start (P.First_Track (A) + I) >= 1 then
                           Linked := Linked + 1;
                        end if;
                     end loop;
                     for C in Alone'Range loop
                        Append (Per, Alone (C)'Image);
                     end loop;
                     Say ("arm" & A'Image
                          & (case Verdict is
                                when Ambiguous      => " has points that two eyes in different poses explain alike",
                                when Not_Reproduced => " has points whose best eye the starts do not agree on",
                                when others         => " has no points whose motion an eye in any pose explains")
                          & " (" & Linked'Image & " of" & Watching.Tracks'Image & " tracks given a link, from"
                          & Watching.Sightings'Image & " sightings; keyframes that turned one joint alone, by joint:"
                          & To_String (Per) & ")");
                  end;
               end if;
            end;
         end loop;
         if not (for some Is_Active of Active => Is_Active) then
            Report.Why := Why;
            Report.Used := 0;
            Report.L := S.L;
            Describe_Arms (S, W.all);
            Release (W);
            Release (P);
            return;
         end if;
         declare
            Which_A : constant Free_Terms := Terms_For (Active, True, False);
            Which_B : constant Free_Terms := Terms_For (Active, True, True);
            T_All   : constant Positive := Terms (N_Arms);
            Stage_A, Stage_B : State (N_Arms);
            Work_A, Work_B   : Work_Access;
            Cov_A, Cov_B     : Real_Matrix (1 .. T_All, 1 .. T_All);
            Red_A            : Real_Matrix (1 .. Which_A'Length, 1 .. Which_A'Length);
            Red_B            : Real_Matrix (1 .. Which_B'Length, 1 .. Which_B'Length);
            White_A, White_B : Real;
            Ok_A, Ok_B       : Boolean;
            Why_A, Why_B     : Unbounded_String;
            Keep_B           : Boolean := False;
         begin
            --  Every arm's pose and the lens together, with the links read again from the model until they stop
            --  changing.
            Settle_Links (P.all, S, Active, W.all, Which_A, Width, Height);
            Stage_A := S;
            Work_A := new Work'(W.all);
            Covariance_At (P.all, Arms, Stage_A, Work_A.all, Which_A, Cov_A, Red_A, White_A, Ok_A, Why_A);
            --  The distortion terms, kept only when as a pair they are significant.
            Stage_B := S;
            Work_B := new Work'(W.all);
            Settle (P.all, Stage_B, Work_B.all, Which_B, Width, Height);
            Covariance_At (P.all, Arms, Stage_B, Work_B.all, Which_B, Cov_B, Red_B, White_B, Ok_B, Why_B);
            if Ok_B then
               declare
                  C11 : constant Real := Cov_B (5, 5);
                  C12 : constant Real := Cov_B (5, 6);
                  C22 : constant Real := Cov_B (6, 6);
                  Det : constant Real := C11 * C22 - C12 * C12;
               begin
                  if Det > 0.0 then
                     declare
                        K1  : constant Real := Stage_B.L.K1;
                        K2  : constant Real := Stage_B.L.K2;
                        Chi : constant Real := (C22 * K1 * K1 - 2.0 * C12 * K1 * K2 + C11 * K2 * K2) / Det;
                     begin
                        Keep_B := Chi > 0.0
                          and then Driver.Distributions.Chi_Square_Deviate (Chi, 2) > Driver.Conventions.Z;
                     end;
                  end if;
               end;
            end if;
            declare
               Final    : constant State := (if Keep_B then Stage_B else Stage_A);
               Cov      : constant Real_Matrix := (if Keep_B then Cov_B else Cov_A);
               Final_Ok : constant Boolean := (if Keep_B then Ok_B else Ok_A);
               Terms_Free : constant Natural := (if Keep_B then Which_B'Length else Which_A'Length);
               Lens_Ok  : Boolean := False;
               Used     : Natural := 0;
               Offered  : Natural := 0;
               Linear_To : Real := Real'Last;
               --  The independent units the sightings come in: a point errs alike in all its views and a picture's
               --  points err alike, so the sightings tell no more than the fewer of the points seen twice or more
               --  and the pictures seen in, less the terms they are to measure (the degrees of freedom of the
               --  covariance, after Cameron, Gelbach and Miller's two-way clusters).
               Times_Seen   : Fit.Count_Array (1 .. P.Tracks) := [others => 0];
               Frame_Base   : Fit.Count_Array (1 .. N_Arms) := [others => 0];
               Picture_Used : Fit.Flag_Array (1 .. P.Sightings + 1) := [others => False];
               Tracks_Seen, Pictures_Seen : Natural := 0;
               Freedom      : Integer;
            begin
               Final_W := (if Keep_B then Work_B else Work_A);
               for A in 2 .. N_Arms loop
                  Frame_Base (A) := Frame_Base (A - 1) + P.Moves (A - 1)'Length (2);
               end loop;
               for I in 1 .. P.Sightings loop
                  if Final_W.Fits (I) then
                     Used := Used + 1;
                     Times_Seen (P.Sight_Track (I)) := Times_Seen (P.Sight_Track (I)) + 1;
                     Picture_Used (Frame_Base (P.Sight_Arm (I)) + P.Sight_Frame (I)) := True;
                  end if;
                  if Final_W.Seen (I) then
                     Offered := Offered + 1;
                  end if;
               end loop;
               for T in 1 .. P.Tracks loop
                  if Times_Seen (T) >= 2 then
                     Tracks_Seen := Tracks_Seen + 1;
                  end if;
               end loop;
               for Used_Picture of Picture_Used loop
                  if Used_Picture then
                     Pictures_Seen := Pictures_Seen + 1;
                  end if;
               end loop;
               Freedom := Integer'Min (Tracks_Seen, Pictures_Seen) - Terms_Free;
               Report.L := Final.L;
               Report.Sigma_Px := Final_W.Model.Floor;
               Report.Used := Used;
               Report.Offered := Offered;
               Report.Distorted := Keep_B;
               if Final_Ok then
                  if Keep_B then
                     Linearity (P.all, Stage_B, Work_B.all, Which_B, Red_B, Cov_B, Width, Height, Linear_To);
                  else
                     Linearity (P.all, Stage_A, Work_A.all, Which_A, Red_A, Cov_A, Width, Height, Linear_To);
                  end if;
                  Report.Linear_To := Linear_To;
                  for A1 in 1 .. T_All loop
                     for B1 in 1 .. T_All loop
                        Report.Covariance.Append (Cov (A1, B1));
                     end loop;
                  end loop;
               end if;
               if not Final_Ok then
                  Say (To_String (if Keep_B then Why_B else Why_A));
               elsif Freedom <= 0 then
                  Say ("too few of the points and pictures fit one eye to measure its" & Terms_Free'Image
                       & " terms from:" & Tracks_Seen'Image & " points seen twice or more, in" & Pictures_Seen'Image
                       & " pictures");
               else
                  declare
                     Sigma_Fx : constant Real := Final.L.Fx * Sqrt (Cov (1, 1));
                     Sigma_Fy : constant Real := Final.L.Fy * Sqrt (Cov (2, 2));
                     Failed   : Boolean := False;
                  begin
                     if Linear_To < Driver.Conventions.Z - 1.0 then
                        --  The sigmas rest on the cost being quadratic over them. One sigma is the standard deviation
                        --  of a significance itself: a displacement the covariance puts Z sigmas away that the cost
                        --  puts more than that nearer is not where the covariance says.
                        Say ("the fit is not quadratic over its own sigma: a displacement of "
                             & Driver.Log.Image (Driver.Conventions.Z, 0) & " sigmas along a principal direction of its"
                             & " covariance changes the cost by only" & Driver.Log.Image (Linear_To, 1)
                             & ", so its covariance does not stand");
                        Failed := True;
                     end if;
                     if not Driver.Uncertain.Significant (Final.L.Fx, Sigma_Fx, Freedom)
                       or else not Driver.Uncertain.Significant (Final.L.Fy, Sigma_Fy, Freedom)
                     then
                        Say ("the points leave the focal lengths undetermined:" & Driver.Log.Image (Final.L.Fx, 1)
                             & " +-" & Driver.Log.Image (Sigma_Fx, 1) & " and" & Driver.Log.Image (Final.L.Fy, 1)
                             & " +-" & Driver.Log.Image (Sigma_Fy, 1) & " px");
                        Failed := True;
                     end if;
                     if not (Driver.Conventions.Z * Sqrt (Cov (3, 3)) < Real (Width) / 2.0
                             and then Driver.Conventions.Z * Sqrt (Cov (4, 4)) < Real (Height) / 2.0)
                     then
                        --  The optical axis is not located in the picture to within half of it.
                        Say ("the points leave the eye's principal point known only to within"
                             & Driver.Log.Image (Sqrt (Cov (3, 3)), 1) & " and"
                             & Driver.Log.Image (Sqrt (Cov (4, 4)), 1)
                             & " px in a picture of" & Width'Image & " by" & Height'Image);
                        Failed := True;
                     end if;
                     Lens_Ok := not Failed;
                     for A in 1 .. N_Arms loop
                        if Active (A) then
                           declare
                              O          : constant Natural := Pose_At (A);
                              Turn_Var   : constant Real := Cov (O + 1, O + 1) + Cov (O + 2, O + 2) + Cov (O + 3, O + 3);
                              Centre     : constant Vec3 := Final.Pose (A).C;
                              Away       : constant Real := Sqrt (Dot (Centre, Centre));
                              Centre_Cov : constant Mat3 :=
                                [for I in 1 .. 3 => [for J in 1 .. 3 => Cov (O + 3 + I, O + 3 + J)]];
                              Away_Sigma : constant Real :=
                                (if Away > 0.0 then Driver.Uncertain.Sigma_Along (Centre_Cov, (1.0 / Away) * Centre)
                                 else Real'Last);
                              T          : constant Real :=
                                Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Natural (Freedom)));
                           begin
                              --  The turn within a quarter turn, and the eye's distance from the arm's eye significant
                              --  against its own sigma, each at the sigmas the covariance's freedom allows (Student's
                              --  t). A centre whose variance was merely finite was taken as placed: 0.3 to 4.6 m from
                              --  the arm's eye where the truth stands 0.6 to 0.7 m, two of them said determined (the
                              --  bench's fixed eye over table arms, 10-20: 6 to 24 points of the arm, 13 to 33 pictures).
                              Placed (A) := T * Sqrt (Turn_Var) < Ada.Numerics.Pi / 2.0
                                and then Driver.Uncertain.Significant (Away, Away_Sigma, Natural (Freedom));
                              if not Placed (A) then
                                 Say ("the points leave arm" & A'Image & "'s eye turn known only to"
                                      & Driver.Log.Image (Sqrt (Turn_Var), 3) & " rad, its distance from the arm's eye"
                                      & Driver.Log.Image (Away, 3) & " +-" & Driver.Log.Image (Away_Sigma, 3));
                              end if;
                           end;
                        end if;
                     end loop;
                  end;
               end if;
               Report.Determined := Lens_Ok and then (for some Is_Placed of Placed => Is_Placed);
               Report.Why := Why;
               Describe_Arms (Final, Final_W.all);
            end;
            Release (Work_A);
            Release (Work_B);
         end;
         Release (W);
         Release (P);
      end;
   end Fit_Eye;

end Driver.Robot.Kinematics.Hand_Eye;
