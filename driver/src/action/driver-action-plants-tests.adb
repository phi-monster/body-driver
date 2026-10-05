with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Action.Contact;
with Driver.Action.Contact.Wrench;
with Driver.Clock;
with Driver.Conventions;

package body Driver.Action.Plants.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Uncertain;
   use type Arm_Id;
   use type Hand_Id;
   use type Thing_Id;
   use type Driver.Action.Contact.Wrench.Obstacle;

   package Contact renames Driver.Action.Contact;
   package Wrench renames Driver.Action.Contact.Wrench;
   package Random renames Ada.Numerics.Float_Random;

   Pi : constant := Ada.Numerics.Pi;

   --  Round-off of poses composed beat after beat: a point this far into the
   --  table is still on it.
   Touching : constant Real := 1.0e-9;

   Signs : constant array (1 .. 2) of Real := [-1.0, 1.0];

   function Gauss (W : World) return Real is
      U1 : constant Real := Real'Max (Real (Random.Random (W.Noise)), Real'Model_Small);
      U2 : constant Real := Real (Random.Random (W.Noise));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Pi * U2);
   end Gauss;

   function Noisy (W : World; P : Vec3; Sigma : Real) return Vec3 is
     (P + Sigma * [Gauss (W), Gauss (W), Gauss (W)]);

   function Height (W : World; P : Vec3) return Real is ((P - W.Table.Translation) * W.Up);

   function Flat (W : World; V : Vec3) return Vec3 is (V - Real'(V * W.Up) * W.Up);

   --  Rigid interpolation: a share S of the way from A to B.
   function Between (A, B : Rigid; S : Real) return Rigid is
     ((Rotation    => A.Rotation * Exp (S * Log (Transpose (A.Rotation) * B.Rotation)),
       Translation => A.Translation + S * (B.Translation - A.Translation)));

   function Index_Of (W : World; T : Thing_Id) return Positive is
   begin
      for I in 1 .. Natural (W.Things.Length) loop
         if W.Things (I).Id = T then
            return I;
         end if;
      end loop;
      raise Program_Error with "no such thing";
   end Index_Of;

   function Arm_Index (W : World; A : Arm_Id) return Positive is
   begin
      for I in 1 .. Natural (W.Arms.Length) loop
         if W.Arms (I).Id = A then
            return I;
         end if;
      end loop;
      raise Program_Error with "no such arm";
   end Arm_Index;

   function Hand_Of_Arm (W : World; A : Arm_Id) return Natural is
   begin
      for I in 1 .. Natural (W.Hands.Length) loop
         if W.Hands (I).Arm = A then
            return I;
         end if;
      end loop;
      return 0;
   end Hand_Of_Arm;

   function Hand_Index (W : World; H : Hand_Id) return Positive is
   begin
      for I in 1 .. Natural (W.Hands.Length) loop
         if W.Hands (I).Id = H then
            return I;
         end if;
      end loop;
      raise Program_Error with "no such hand";
   end Hand_Index;

   --  The extreme of a thing along a direction, from its parts' corners and rims.
   function Extreme (M : Model; Pose : Rigid; D : Vec3) return Real is
      E : Real := Real'First;
   begin
      for P of M.Parts loop
         declare
            F : constant Rigid := Pose * P.Pose;
         begin
            case P.Kind is
               when Block =>
                  for X of Signs loop
                     for Y of Signs loop
                        for Z of Signs loop
                           E := Real'Max (E, (F * [X * P.Sizes (1), Y * P.Sizes (2), Z * P.Sizes (3)]) * D);
                        end loop;
                     end loop;
                  end loop;
               when Cylinder | Tube =>
                  declare
                     A   : constant Vec3 := Rotate (F, [0.0, 0.0, 1.0]);
                     Off : constant Real := P.Sizes (1) * Sqrt (Real'Max (0.0, 1.0 - Real'(A * D) ** 2));
                  begin
                     for Z of Signs loop
                        E := Real'Max (E, (F.Translation + Z * P.Sizes (3) * A) * D + Off);
                     end loop;
                  end;
            end case;
         end;
      end loop;
      return E;
   end Extreme;

   function Lowest_Of (W : World; I : Positive) return Real is
     (-Extreme (W.Things (I).Shape, W.Things (I).Pose, -W.Up) - W.Table.Translation * W.Up);

   function Highest_Of (W : World; I : Positive) return Real is
     (Extreme (W.Things (I).Shape, W.Things (I).Pose, W.Up) - W.Table.Translation * W.Up);

   function Centre_Of (W : World; I : Positive) return Vec3 is (W.Things (I).Pose * Centre (W.Things (I).Shape));

   function Radius_Of (W : World; I : Positive) return Real is (W.Things (I).Bound);

   function Inside_Thing (W : World; I : Positive; P : Vec3) return Boolean is
     (abs (P - Centre_Of (W, I)) <= W.Things (I).Bound
      and then Inside (W.Things (I).Shape, Inverse (W.Things (I).Pose) * P));

   --  Its whole surface in the world, where it is now.
   function World_Points (W : World; I : Positive; Pose : Rigid) return Contact.Point_Vectors.Vector is
      P : Contact.Point_Vectors.Vector;
   begin
      for S of W.Things (I).Points loop
         P.Append (Pose * S.Point);
      end loop;
      return P;
   end World_Points;

   --  The body of an arm in its tool frame: lobe boxes and palm, or plate.
   procedure Box_Points (C, Half : Vec3; Spacing : Real; Into : in out Contact.Point_Vectors.Vector) is
   begin
      for Axis in 1 .. 3 loop
         declare
            A1 : constant Positive := (if Axis = 1 then 2 else 1);
            A2 : constant Positive := (if Axis = 3 then 2 else 3);
            N1 : constant Positive := Positive'Max (1, Natural (Real'Ceiling (2.0 * Half (A1) / Spacing)));
            N2 : constant Positive := Positive'Max (1, Natural (Real'Ceiling (2.0 * Half (A2) / Spacing)));
         begin
            for S of Signs loop
               for I in 0 .. N1 loop
                  for J in 0 .. N2 loop
                     declare
                        Q : Vec3 := C;
                     begin
                        Q (Axis) := C (Axis) + S * Half (Axis);
                        Q (A1) := C (A1) - Half (A1) + 2.0 * Half (A1) * Real (I) / Real (N1);
                        Q (A2) := C (A2) - Half (A2) + 2.0 * Half (A2) * Real (J) / Real (N2);
                        Into.Append (Q);
                     end;
                  end loop;
               end loop;
            end loop;
         end;
      end loop;
   end Box_Points;

   function Tip_X (H : Sim_Hand; K : Real; F : Real) return Real is
     (K * ((H.Opening + H.Thickness) / 2.0 - F * H.Opening / 2.0));

   function Lobe_Points (H : Sim_Hand; K : Real; F : Real; Spacing : Real) return Contact.Point_Vectors.Vector is
      P : Contact.Point_Vectors.Vector;
   begin
      Box_Points ([Tip_X (H, K, F), 0.0, H.Depth / 2.0], [H.Thickness / 2.0, H.Width / 2.0, H.Depth / 2.0], Spacing, P);
      return P;
   end Lobe_Points;

   --  The face of lobe K that looks at the other lobe.
   function Face_Points (H : Sim_Hand; K : Real; F : Real; Spacing : Real) return Contact.Point_Vectors.Vector is
      P  : Contact.Point_Vectors.Vector;
      X  : constant Real := Tip_X (H, K, F) - K * H.Thickness / 2.0;
      NY : constant Positive := Positive'Max (1, Natural (Real'Ceiling (H.Width / Spacing)));
      NZ : constant Positive := Positive'Max (1, Natural (Real'Ceiling (H.Depth / Spacing)));
   begin
      for I in 0 .. NY loop
         for J in 0 .. NZ loop
            P.Append (Vec3'([X, -H.Width / 2.0 + H.Width * Real (I) / Real (NY), H.Depth * Real (J) / Real (NZ)]));
         end loop;
      end loop;
      return P;
   end Face_Points;

   function Body_Points (W : World; A : Positive) return Contact.Point_Vectors.Vector is
      P : Contact.Point_Vectors.Vector;
      H : constant Natural := Hand_Of_Arm (W, W.Arms (A).Id);
   begin
      if H /= 0 then
         declare
            G : constant Sim_Hand := W.Hands (H);
         begin
            for K of Signs loop
               P.Append (Lobe_Points (G, K, G.Fraction, W.Pitch));
            end loop;
            Box_Points ([0.0, 0.0, -G.Thickness / 2.0], [G.Opening / 2.0 + G.Thickness, G.Width / 2.0, G.Thickness / 2.0],
                        W.Pitch, P);
         end;
      elsif W.Arms (A).Plate_Radius > 0.0 then
         declare
            R : constant Real := W.Arms (A).Plate_Radius;
            N : constant Positive := Positive'Max (1, Natural (Real'Ceiling (R / W.Pitch)));
         begin
            P.Append (Zero3);
            for I in 1 .. N loop
               declare
                  Ring   : constant Real := R * Real (I) / Real (N);
                  Around : constant Positive := Positive'Max (1, Natural (Real'Ceiling (2.0 * Pi * Ring / W.Pitch)));
               begin
                  for J in 0 .. Around - 1 loop
                     P.Append (Vec3'([Ring * Cos (2.0 * Pi * Real (J) / Real (Around)),
                                Ring * Sin (2.0 * Pi * Real (J) / Real (Around)), 0.0]));
                  end loop;
               end;
            end loop;
         end;
      end if;
      return P;
   end Body_Points;

   function Held_By_Arm (W : World; A : Positive) return Natural is
      H : constant Natural := Hand_Of_Arm (W, W.Arms (A).Id);
   begin
      if H /= 0 then
         for I in 1 .. Natural (W.Things.Length) loop
            if W.Things (I).Held_By = W.Hands (H).Id then
               return I;
            end if;
         end loop;
      end if;
      return 0;
   end Held_By_Arm;

   function Can_Give_Way (W : World; I : Positive) return Boolean is
     (not W.Things (I).Fixed and then W.Things (I).Joint.Kind = Loose and then W.Things (I).Held_By = 0);

   function Joint_Pose (J : Sim_Joint; Q : Real) return Rigid is
   begin
      case J.Kind is
         when Hinge =>
            declare
               R : constant Mat3 := Exp (Q * J.Axis);
            begin
               return (Rotation => R * J.Rest.Rotation, Translation => R * (J.Rest.Translation - J.Point) + J.Point);
            end;
         when Rail =>
            return (Rotation => J.Rest.Rotation, Translation => J.Rest.Translation + Q * J.Axis);
         when Loose =>
            return J.Rest;
      end case;
   end Joint_Pose;

   --  The bottom points of thing I that lie over thing K's top, or over the
   --  table when K is 0, as its footing there.
   function Footing_On (W : World; I : Positive; K : Natural) return Contact.Footing is
      Low   : constant Real := Lowest_Of (W, I);
      Feet  : Contact.Point_Vectors.Vector;
      Plane : constant Vec3 := W.Table.Translation + Low * W.Up;
   begin
      for P of World_Points (W, I, W.Things (I).Pose) loop
         if Height (W, P) - Low <= W.Pitch / 2.0
           and then (K = 0 or else Inside_Thing (W, K, P - (W.Pitch / 2.0) * W.Up))
         then
            Feet.Append (P);
         end if;
      end loop;
      return Contact.Footing_Of (Feet, Plane, W.Up, W.Pitch);
   end Footing_On;

   --  What thing I lies on: 0 the table, else the thing under it; -1 nothing.
   function Under (W : World; I : Positive) return Integer is
      Low : constant Real := Lowest_Of (W, I);
   begin
      if abs Low <= W.Pitch / 2.0 then
         return 0;
      end if;
      for K in 1 .. Natural (W.Things.Length) loop
         if K /= I and then abs (Highest_Of (W, K) - Low) <= W.Pitch / 2.0
           and then abs Flat (W, Centre_Of (W, K) - Centre_Of (W, I)) < Radius_Of (W, K) + Radius_Of (W, I)
         then
            return K;
         end if;
      end loop;
      return -1;
   end Under;

   procedure Translate (W : in out World; I : Positive; D : Vec3) is
   begin
      W.Things (I).Pose.Translation := W.Things (I).Pose.Translation + D;
   end Translate;

   --  Let a thing down onto whatever is under it; one that would not rest on
   --  another thing slides off it onto the table.
   procedure Drop (W : in out World; I : Positive) is
      Low  : constant Real := Lowest_Of (W, I);
      Top  : Real := 0.0;
      Onto : Natural := 0;
   begin
      for K in 1 .. Natural (W.Things.Length) loop
         if K /= I and then W.Things (K).Held_By = 0 then
            declare
               H : constant Real := Highest_Of (W, K);
            begin
               if H <= Low + W.Pitch / 2.0 and then H > Top
                 and then abs Flat (W, Centre_Of (W, K) - Centre_Of (W, I)) < Radius_Of (W, K) + Radius_Of (W, I)
               then
                  Top := H;
                  Onto := K;
               end if;
            end;
         end if;
      end loop;
      Translate (W, I, -(Low - Top) * W.Up);
      if Onto /= 0 then
         declare
            C : constant Point_Estimate := (Mean => Centre_Of (W, I), Covariance => [others => [others => 0.0]]);
            R : constant Wrench.Rest_Answer := Wrench.Rests (Footing_On (W, I, Onto), C, W.Up, W.Things (I).Mu);
         begin
            if not R.Rests then
               declare
                  Away : constant Vec3 := Flat (W, Centre_Of (W, I) - Centre_Of (W, Onto));
                  Gap  : constant Real := Radius_Of (W, I) + Radius_Of (W, Onto) - abs Away;
               begin
                  Translate (W, I, (Gap + W.Pitch) * (if abs Away > 0.0 then Unit (Away) else Flat (W, Rotate (W.Table,
                                                       [1.0, 0.0, 0.0]))));
                  Translate (W, I, -Lowest_Of (W, I) * W.Up);
               end;
            end if;
         end;
      end if;
   end Drop;

   --  A closer stops within one of its sub-steps (half a pitch) of what it
   --  meets; a face point is touching when a little more than that ahead of
   --  it is inside the thing, and not so far ahead as to pass through a wall.
   Probe : constant Real := 0.75;

   --  The touches of a hand's lobes on the thing they hold, in the world.
   function Grip_Touches (W : World; H : Positive; I : Positive; Tool : Rigid) return Contact.Touch_Vectors.Vector is
      G : constant Sim_Hand := W.Hands (H);
      T : Contact.Touch_Vectors.Vector;
   begin
      for K of Signs loop
         declare
            Inward : constant Vec3 := Rotate (Tool, [-K, 0.0, 0.0]);
            Pts    : Contact.Point_Vectors.Vector;
            Sum    : Vec3 := Zero3;
         begin
            for P of Face_Points (G, K, G.Fraction, W.Pitch / 2.0) loop
               if Inside_Thing (W, I, Tool * P + (Probe * W.Pitch) * Inward) then
                  Pts.Append (Tool * P);
                  Sum := Sum + Tool * P;
               end if;
            end loop;
            if not Pts.Is_Empty then
               declare
                  Mid   : constant Vec3 := Sum / Real (Pts.Length);
                  Lever : Real := 0.0;
               begin
                  --  Under even pressure the patch resists turning about its
                  --  normal with the mean distance of its points from the middle.
                  for Q of Pts loop
                     Lever := Lever + abs (Q - Mid) / Real (Pts.Length);
                  end loop;
                  T.Append (Contact.Touch'(Point => Mid, Inward => Inward, Patch => Lever, Tension => False));
               end;
            end if;
         end;
      end loop;
      return T;
   end Grip_Touches;

   --  Moves arm A's tool to Next if nothing stops it: the table and things
   --  that cannot give way stop it; loose things are pushed along the table;
   --  a held thing comes along while its grip carries it, or moves only along
   --  its joint and takes the hand with it.
   procedure Try_Pose (W : in out World; A : Positive; Next : Rigid; Blocked : out Boolean) is
      Now    : constant Rigid := W.Arms (A).Tool;
      Points : constant Contact.Point_Vectors.Vector := Body_Points (W, A);
      Held   : constant Natural := Held_By_Arm (W, A);
      H      : constant Natural := Hand_Of_Arm (W, W.Arms (A).Id);
      Reach_Of_Body : Real := 0.0;
   begin
      Blocked := False;
      for P of Points loop
         Reach_Of_Body := Real'Max (Reach_Of_Body, abs P);
      end loop;
      for P of Points loop
         if Height (W, Next * P) < -Touching then
            Blocked := True;
            return;
         end if;
      end loop;
      for K in 1 .. Natural (W.Things.Length) loop
         if K /= Held and then abs (Centre_Of (W, K) - Next.Translation) <= W.Things (K).Bound + Reach_Of_Body then
            for P of Points loop
               if Inside_Thing (W, K, Next * P) then
                  declare
                     D : constant Vec3 := Next * P - Now * P;
                  begin
                     if Can_Give_Way (W, K) and then Under (W, K) = 0
                       and then abs Flat (W, D) >= -Real'(D * W.Up)
                     then
                        Translate (W, K, Flat (W, D));
                     else
                        Blocked := True;
                        return;
                     end if;
                  end;
               end if;
            end loop;
         end if;
      end loop;
      if Held /= 0 then
         declare
            T : Sim_Thing renames W.Things (Held);
         begin
            if T.Joint.Kind /= Loose then
               --  It moves only along its joint; the hand goes where the grip goes.
               declare
                  J  : constant Sim_Joint := T.Joint;
                  Q  : Real := J.Q;
               begin
                  case J.Kind is
                     when Rail =>
                        Q := J.Q + (Next.Translation - Now.Translation) * J.Axis;
                     when Hinge =>
                        declare
                           A0 : constant Vec3 := Now.Translation - J.Point;
                           A1 : constant Vec3 := Next.Translation - J.Point;
                           P0 : constant Vec3 := A0 - Real'(A0 * J.Axis) * J.Axis;
                           P1 : constant Vec3 := A1 - Real'(A1 * J.Axis) * J.Axis;
                        begin
                           Q := J.Q + Arctan (Cross (P0, P1) * J.Axis, P0 * P1);
                        end;
                     when Loose =>
                        null;
                  end case;
                  Q := Real'Min (J.High, Real'Max (J.Low, Q));
                  if abs (Q - J.Q) <= Touching then
                     Blocked := True;
                     return;
                  end if;
                  T.Moved_Q := T.Moved_Q + abs (Q - J.Q);
                  T.Joint.Q := Q;
                  T.Pose := Joint_Pose (T.Joint, Q);
                  W.Arms (A).Tool := T.Pose * Inverse (T.Grip);
                  return;
               end;
            end if;
            declare
               New_Pose : constant Rigid := Next * T.Grip;
               Low      : constant Real :=
                 -Extreme (T.Shape, New_Pose, -W.Up) - W.Table.Translation * W.Up;
               Middle   : constant Vec3 := New_Pose * Centre (T.Shape);
            begin
               for K in 1 .. Natural (W.Things.Length) loop
                  if K /= Held and then abs (Centre_Of (W, K) - Middle) <= W.Things (K).Bound + T.Bound then
                     for P of World_Points (W, Held, New_Pose) loop
                        if Inside_Thing (W, K, P) then
                           Blocked := True;
                           return;
                        end if;
                     end loop;
                  end if;
               end loop;
               if Low < -Touching then
                  Blocked := True;
                  return;
               end if;
               declare
                  Old_Centre : constant Vec3 := Centre_Of (W, Held);
                  Turn       : constant Vec3 := Log (New_Pose.Rotation * Transpose (T.Pose.Rotation));
                  Motion     : constant Contact.Twist :=
                    (Linear => New_Pose * Centre (T.Shape) - Old_Centre, Angular => Turn, Pivot => Old_Centre);
                  Base       : constant Contact.Footing :=
                    (if abs Lowest_Of (W, Held) <= Touching then Footing_On (W, Held, 0) else Contact.No_Footing);
                  Need       : constant Wrench.Answer :=
                    Wrench.Need (Grip_Touches (W, H, Held, Now), Base, Motion, Old_Centre, W.Up, T.Mu);
               begin
                  if Need.Why = Wrench.None then
                     T.Pose := New_Pose;
                  else
                     --  The grip does not carry it: it stays, or falls if nothing bears it.
                     T.Held_By := 0;
                     W.Hands (H).Stopped := False;
                     Drop (W, Held);
                  end if;
               end;
            end;
         end;
      end if;
      W.Arms (A).Tool := Next;
   end Try_Pose;

   procedure Step_Arm (W : in out World; A : Positive) is
      Arm : constant Sim_Arm := W.Arms (A);
   begin
      if not Arm.Active or else Arm.Blocked then
         return;
      end if;
      declare
         Next  : constant Rigid := Between (Arm.Tool, Arm.Target, Arm.Rate);
         Shift : constant Real := abs (Next.Translation - Arm.Tool.Translation);
         Turn  : constant Real := Angle (Transpose (Arm.Tool.Rotation) * Next.Rotation);
         Reach_Of_Body : Real := 0.0;
      begin
         for P of Body_Points (W, A) loop
            Reach_Of_Body := Real'Max (Reach_Of_Body, abs P);
         end loop;
         if Held_By_Arm (W, A) /= 0 then
            Reach_Of_Body := Reach_Of_Body + Radius_Of (W, Held_By_Arm (W, A))
              + abs (Centre_Of (W, Held_By_Arm (W, A)) - Arm.Tool.Translation);
         end if;
         if Shift + Turn * Reach_Of_Body <= W.Sigma then
            W.Arms (A).Active := False;
            return;
         end if;
         declare
            Pieces : constant Positive :=
              Positive'Max (1, Natural (Real'Ceiling ((Shift + Turn * Reach_Of_Body) / (W.Pitch / 2.0))));
            From   : constant Rigid := Arm.Tool;
            Stop   : Boolean := False;
         begin
            for I in 1 .. Pieces loop
               Try_Pose (W, A, Between (From, Next, Real (I) / Real (Pieces)), Stop);
               if Stop then
                  W.Arms (A).Blocked := True;
                  exit;
               end if;
               W.Moved := True;
            end loop;
         end;
      end;
   end Step_Arm;

   procedure Step_Hand (W : in out World; H : Positive) is
      G    : constant Sim_Hand := W.Hands (H);
      A    : constant Positive := Arm_Index (W, G.Arm);
      Tool : constant Rigid := W.Arms (A).Tool;
   begin
      if G.Stopped or else abs (G.Goal - G.Fraction) <= 1.0e-9 then
         return;
      end if;
      if G.Goal < G.Fraction then
         for I in 1 .. Natural (W.Things.Length) loop
            if W.Things (I).Held_By = G.Id then
               W.Things (I).Held_By := 0;
               Drop (W, I);
            end if;
         end loop;
         W.Hands (H).Fraction := G.Fraction + G.Rate * (G.Goal - G.Fraction);
         if abs (W.Hands (H).Fraction - G.Goal) <= 1.0e-6 then
            W.Hands (H).Fraction := G.Goal;
         end if;
         W.Moved := True;
         return;
      end if;
      declare
         To     : constant Real := (if G.Goal - G.Fraction <= 1.0e-6 then G.Goal
                                    else G.Fraction + G.Rate * (G.Goal - G.Fraction));
         Pieces : constant Positive :=
           Positive'Max (1, Natural (Real'Ceiling ((To - G.Fraction) * G.Opening / 2.0 / (W.Pitch / 2.0))));
      begin
         for Piece in 1 .. Pieces loop
            declare
               F  : constant Real := G.Fraction + (To - G.Fraction) * Real (Piece) / Real (Pieces);
               Touched : array (1 .. 2) of Natural := [0, 0];
            begin
               for L in 1 .. 2 loop
                  for P of Face_Points (W.Hands (H), Signs (L), F, W.Pitch / 2.0) loop
                     for K in 1 .. Natural (W.Things.Length) loop
                        if Inside_Thing (W, K, Tool * P) then
                           Touched (L) := K;
                        end if;
                     end loop;
                  end loop;
               end loop;
               if Touched (1) /= 0 and then Touched (1) = Touched (2) then
                  W.Hands (H).Stopped := True;
                  if W.Things (Touched (1)).Held_By = 0 and then not W.Things (Touched (1)).Fixed then
                     W.Things (Touched (1)).Held_By := G.Id;
                     W.Things (Touched (1)).Grip := Inverse (Tool) * W.Things (Touched (1)).Pose;
                  end if;
                  return;
               end if;
               for L in 1 .. 2 loop
                  if Touched (L) /= 0 and then Touched (3 - L) = 0 then
                     declare
                        K : constant Positive := Touched (L);
                        D : constant Vec3 := Rotate (Tool, [-Signs (L) * (F - W.Hands (H).Fraction) * G.Opening / 2.0,
                                                              0.0, 0.0]);
                     begin
                        if Can_Give_Way (W, K) and then Under (W, K) = 0 then
                           Translate (W, K, Flat (W, D));
                        elsif W.Things (K).Fixed then
                           W.Hands (H).Stopped := True;
                           return;
                        end if;
                     end;
                  end if;
               end loop;
               W.Hands (H).Fraction := F;
               W.Moved := True;
            end;
         end loop;
      end;
   end Step_Hand;

   procedure Advance (W : in out World) is
   begin
      W.Beat := W.Beat + 1;
      W.Moved := False;
      for T of W.Things loop
         if abs T.Drift > 0.0 and then T.Held_By = 0 then
            T.Pose.Translation := T.Pose.Translation + T.Drift;
            W.Moved := True;
         end if;
      end loop;
      for A in 1 .. Natural (W.Arms.Length) loop
         declare
            Arm : Sim_Arm renames W.Arms (A);
         begin
            while not Arm.Pending.Is_Empty and then Arm.Pending.First_Element.Due <= W.Beat loop
               declare
                  Goal  : constant Rigid := Arm.Pending.First_Element.Goal;
                  Share : constant Real :=
                    Arm.Delivery_Low + (Arm.Delivery_High - Arm.Delivery_Low) * Real (Random.Random (W.Noise));
               begin
                  Arm.Start := Arm.Tool;
                  Arm.Commanded := Goal;
                  Arm.Target := Between (Arm.Tool, Goal, Share);
                  Arm.Active := True;
                  Arm.Blocked := False;
                  Arm.Pending.Delete_First;
               end;
            end loop;
         end;
      end loop;
      for H of W.Hands loop
         if H.Due <= W.Beat and then H.Next_Goal /= H.Goal then
            H.Goal := H.Next_Goal;
            H.Stopped := False;
         end if;
      end loop;
      for A in 1 .. Natural (W.Arms.Length) loop
         Step_Arm (W, A);
      end loop;
      for H in 1 .. Natural (W.Hands.Length) loop
         Step_Hand (W, H);
      end loop;
   end Advance;

   function At_Rest (W : World) return Boolean is
     ((for all A of W.Arms => A.Pending.Is_Empty and then (not A.Active or else A.Blocked))
      and then (for all H of W.Hands => H.Due <= W.Beat and then H.Next_Goal = H.Goal
                and then (H.Stopped or else abs (H.Goal - H.Fraction) <= 1.0e-9)));

   procedure Start (W : in out World; Place : Rigid; Sigma, Pitch : Real; Seed : Integer) is
   begin
      W.Table := Place;
      W.Up := Rotate (Place, [0.0, 0.0, 1.0]);
      W.Sigma := Sigma;
      W.Pitch := Pitch;
      W.Eye := Place * (Rotation => Exp ([-2.16, 0.0, 0.0]), Translation => [0.0, -0.6, 0.4]);
      Random.Reset (W.Noise, Seed);
   end Start;

   procedure Add_Arm (W : in out World; Base : Vec3; Reach : Real; Tool : Rigid; Lag : Natural; Rate : Real;
                      Delivery_Low, Delivery_High : Real; Wrist, Tilt : Real; Plate_Radius : Real := 0.0)
   is
      T : constant Rigid := W.Table * Tool;
   begin
      W.Arms.Append (Sim_Arm'(Id            => Arm_Id (Natural (W.Arms.Length) + 1),
                      Base          => W.Table * Base,
                      Reach         => Reach,
                      Neutral       => T.Rotation,
                      Wrist         => Wrist,
                      Tilt          => Tilt,
                      Tool          => T,
                      Lag           => Lag,
                      Rate          => Rate,
                      Delivery_Low  => Delivery_Low,
                      Delivery_High => Delivery_High,
                      Plate_Radius  => Plate_Radius,
                      Pending       => Command_Vectors.Empty_Vector,
                      Start         => T,
                      Target        => T,
                      Commanded     => T,
                      Active        => False,
                      Blocked       => False));
   end Add_Arm;

   procedure Add_Gripper (W : in out World; Arm : Arm_Id; Opening, Width, Thickness, Depth : Real) is
   begin
      W.Hands.Append (Sim_Hand'(Id => Hand_Id (Natural (W.Hands.Length) + 1), Arm => Arm, Opening => Opening, Width => Width,
                       Thickness => Thickness, Depth => Depth, Fraction => 0.0, Goal => 0.0, Due => 0,
                       Next_Goal => 0.0, Rate => W.Arms (Arm_Index (W, Arm)).Rate, Stopped => False));
   end Add_Gripper;

   procedure Add_Thing (W : in out World; M : Model; Place : Rigid; Mu : Real; Fixed : Boolean := False) is
      Points : constant Sample_Vectors.Vector := Surface_Points (M, Identity, W.Pitch);
      Bound  : Real := 0.0;
   begin
      for P of Points loop
         Bound := Real'Max (Bound, abs (P.Point - Centre (M)));
      end loop;
      W.Things.Append (Sim_Thing'(Id => Thing_Id (Natural (W.Things.Length) + 1), Shape => M, Pose => W.Table * Place,
                        Fixed => Fixed, Mu => Mu, Points => Points, Bound => Bound + W.Pitch, others => <>));
      if not Fixed then
         Drop (W, Natural (W.Things.Length));
      end if;
      W.Things (Natural (W.Things.Length)).Seen := W.Things (Natural (W.Things.Length)).Pose;
   end Add_Thing;

   procedure Set_Joint (W : in out World; T : Thing_Id; J : Sim_Joint) is
      I : constant Positive := Index_Of (W, T);
   begin
      W.Things (I).Joint := (J with delta Axis => Rotate (W.Table, J.Axis), Point => W.Table * J.Point,
                             Rest => W.Things (I).Pose, Q => 0.0);
   end Set_Joint;

   procedure Set_Drift (W : in out World; T : Thing_Id; Drift : Vec3) is
   begin
      W.Things (Index_Of (W, T)).Drift := Rotate (W.Table, Drift);
   end Set_Drift;

   function Truth (W : World; T : Thing_Id) return Sim_Thing is (W.Things (Index_Of (W, T)));

   function Lowest (W : World; T : Thing_Id) return Real is (Lowest_Of (W, Index_Of (W, T)));

   function Rests_On (W : World; T, Other : Thing_Id) return Boolean is
      I : constant Positive := Index_Of (W, T);
      K : constant Positive := Index_Of (W, Other);
   begin
      return Under (W, I) = K
        and then Wrench.Rests (Footing_On (W, I, K), (Mean => Centre_Of (W, I), Covariance => [others => [others => 0.0]]),
                               W.Up, W.Things (I).Mu).Rests;
   end Rests_On;

   overriding procedure Look (W : in out World; S : out Snapshot) is
      Sigma : constant Real := W.Sigma;
      Cov   : constant Mat3 := (Sigma * Sigma) * Identity3;
   begin
      S := (Beat => Driver.Clock.Beat (W.Beat), Up => (Unit_Vector => W.Up, Sigma => Sigma), Still => not W.Moved,
            others => <>);
      for A of W.Arms loop
         declare
            Surface : Sample_Vectors.Vector;
            Lever   : Real := 0.0;
            --  It does not move once a beat would move it by no more than
            --  its noise, and a step has to stand out of two noisy readings.
            Least   : constant Real := Real'Max (Driver.Conventions.Z * Sqrt (2.0) * Sigma,
                                                 Sigma / (A.Rate * A.Delivery_Low));
         begin
            for P of Body_Points (W, Arm_Index (W, A.Id)) loop
               Lever := Real'Max (Lever, abs P);
               if A.Plate_Radius > 0.0 then
                  Surface.Append (Sample'(Point => P, Normal => [0.0, 0.0, 1.0]));
               end if;
            end loop;
            S.Arms.Append (Arm_State'(Id          => A.Id,
                            Tool        => (Pose => (Rotation => A.Tool.Rotation,
                                                     Translation => Noisy (W, A.Tool.Translation, Sigma)),
                                            Position_Covariance => Cov, Rotation_Covariance => Cov),
                            Step        => (Value => Least, Sigma => Sigma, Degrees_Of_Freedom => 0),
                            Turn_Step   => (Value => Least / Lever, Sigma => Sigma, Degrees_Of_Freedom => 0),
                            Lag         => (Value => Real (A.Lag), Sigma => Sigma, Degrees_Of_Freedom => 0),
                            Rate        => (Value => A.Rate, Sigma => Sigma, Degrees_Of_Freedom => 0),
                            Surface     => Surface,
                            Carries_Eye => False,
                            Carries_All => False));
         end;
      end loop;
      for H of W.Hands loop
         declare
            G : Hand_State := Gripper (H.Arm, H.Id, H.Opening, H.Width, H.Thickness, H.Depth, Sigma);
            F_Sigma : constant Real := Sigma / H.Opening;
         begin
            G.Fraction := (Value => H.Fraction + F_Sigma * Gauss (W), Sigma => F_Sigma, Degrees_Of_Freedom => 0);
            S.Hands.Append (G);
         end;
      end loop;
      S.Surfaces.Append (Floor (1, W.Table, Sigma));
      for I in 1 .. Natural (W.Things.Length) loop
         declare
            T     : constant Sim_Thing := W.Things (I);
            U     : constant Integer := Under (W, I);
            Seen  : constant Rigid := (Rotation => T.Pose.Rotation, Translation => Noisy (W, T.Pose.Translation, Sigma));
            X     : Thing_State :=
              Thing_Of (T.Id, T.Shape, Seen, W.Pitch, Sigma,
                        (if U = 0 then 1 elsif U > 0 then Surface_Id (1 + Natural (W.Things (U).Id)) else 0));
         begin
            X.Best_Eye := 1;
            X.Held_By := T.Held_By;
            --  Moved since the last look, by more than the eyes' noise.
            X.Moving := Significant (abs (T.Pose.Translation - T.Seen.Translation)
                                     + Radius_Of (W, I) * Angle (Transpose (T.Seen.Rotation) * T.Pose.Rotation), Sigma);
            W.Things (I).Seen := T.Pose;
            X.Height := (Value => (if U = 0 then Lowest_Of (W, I) elsif U > 0 then Lowest_Of (W, I) - Highest_Of (W, U)
                                   else Lowest_Of (W, I)),
                         Sigma => Sigma, Degrees_Of_Freedom => 0);
            if T.Joint.Kind /= Loose then
               X.Joint.Kind := Axis_Unknown;
               if Significant (T.Moved_Q * Radius_Of (W, I), Sigma) then
                  X.Joint := (Kind  => (if T.Joint.Kind = Hinge then Revolute else Prismatic),
                              Base  => 0,
                              Axis  => (Unit_Vector => T.Joint.Axis, Sigma => Sigma / (T.Moved_Q * Radius_Of (W, I))),
                              Point => (Mean => T.Joint.Point, Covariance => Cov));
               end if;
            end if;
            for L of W.Learned loop
               if L.Thing = T.Id then
                  X.Friction := L.Bounds;
               end if;
            end loop;
            S.Things.Append (X);
            if U > 0 then
               S.Surfaces.Append (Surface_State'(Id     => Surface_Id (1 + Natural (W.Things (U).Id)),
                                   Point  => (Mean => W.Table.Translation + Highest_Of (W, U) * W.Up, Covariance => Cov),
                                   Normal => (Unit_Vector => W.Up, Sigma => Sigma),
                                   Of_Thing => W.Things (U).Id));
            end if;
         end;
      end loop;
      S.Eyes.Append (Eye_State'(Pose => (Pose => W.Eye, Position_Covariance => Cov, Rotation_Covariance => Cov), On_Arm => 0));
   end Look;

   overriding function Reach (W : World; Goal : Arm_Goal) return Reach_Answer is
      A : constant Sim_Arm := W.Arms (Arm_Index (W, Goal.Arm));
   begin
      if abs (Goal.Tool.Translation - A.Base) > A.Reach then
         return (Status => Unreachable, Why => To_Unbounded_String ("too far from its base"));
      elsif Goal.Position_Only then
         return (Status => Reachable, Why => Null_Unbounded_String);
      end if;
      declare
         Rel  : constant Mat3 := Transpose (A.Neutral) * Goal.Tool.Rotation;
         Z    : constant Vec3 := Rel * [0.0, 0.0, 1.0];
         Bend : constant Real := Arccos (Real'Max (-1.0, Real'Min (1.0, Z (3))));
         Axis : constant Vec3 := Cross ([0.0, 0.0, 1.0], Z);
         Swing : constant Mat3 := (if abs Axis > 0.0 then Exp (Bend * Unit (Axis)) else Identity3);
         Twist : constant Vec3 := Log (Transpose (Swing) * Rel);
      begin
         if Bend > A.Tilt then
            return (Status => Unreachable, Why => To_Unbounded_String ("the wrist bends no further"));
         elsif abs Twist (3) > A.Wrist then
            return (Status => Unreachable, Why => To_Unbounded_String ("the wrist turns no further"));
         end if;
      end;
      return (Status => Reachable, Why => Null_Unbounded_String);
   end Reach;

   procedure Put_Tool (W : in out World; Arm : Arm_Id; Pose : Rigid; Blocked : out Boolean) is
      A     : constant Positive := Arm_Index (W, Arm);
      From  : constant Rigid := W.Arms (A).Tool;
      Shift : constant Real := abs (Pose.Translation - From.Translation);
      Turn  : constant Real := Angle (Transpose (From.Rotation) * Pose.Rotation);
      Reach_Of_Body : Real := 0.0;
   begin
      Blocked := False;
      for P of Body_Points (W, A) loop
         Reach_Of_Body := Real'Max (Reach_Of_Body, abs P);
      end loop;
      if Held_By_Arm (W, A) /= 0 then
         Reach_Of_Body := Reach_Of_Body + Radius_Of (W, Held_By_Arm (W, A))
           + abs (Centre_Of (W, Held_By_Arm (W, A)) - From.Translation);
      end if;
      declare
         Pieces : constant Positive :=
           Positive'Max (1, Natural (Real'Ceiling ((Shift + Turn * Reach_Of_Body) / (W.Pitch / 2.0))));
      begin
         for I in 1 .. Pieces loop
            Try_Pose (W, A, Between (From, Pose, Real (I) / Real (Pieces)), Blocked);
            exit when Blocked;
            W.Moved := True;
         end loop;
      end;
   end Put_Tool;

   procedure Tick (W : in out World) is
   begin
      Advance (W);
   end Tick;

   procedure Set_Closer (W : in out World; Hand : Hand_Id; Fraction : Real) is
      H : constant Positive := Hand_Index (W, Hand);
   begin
      W.Hands (H).Due := W.Beat + W.Arms (Arm_Index (W, W.Hands (H).Arm)).Lag;
      W.Hands (H).Next_Goal := Fraction;
   end Set_Closer;

   function Closer_Done (W : World; Hand : Hand_Id) return Boolean is
      H : constant Sim_Hand := W.Hands (Hand_Index (W, Hand));
   begin
      return H.Due <= W.Beat and then H.Next_Goal = H.Goal
        and then (H.Stopped or else abs (H.Goal - H.Fraction) <= 1.0e-9);
   end Closer_Done;

   function Closer_Stopped (W : World; Hand : Hand_Id) return Boolean is (W.Hands (Hand_Index (W, Hand)).Stopped);

   overriding procedure Move (W : in out World; O : Order; R : out Report) is
      Rejected : array (1 .. Natural (W.Arms.Length)) of Boolean := [others => False];
      Beats   : Natural := 0;
   begin
      R := (others => <>);
      for G of O.Arms loop
         declare
            I    : constant Positive := Arm_Index (W, G.Arm);
            Goal : constant Rigid := (if G.Position_Only
                                      then (Rotation => W.Arms (I).Tool.Rotation, Translation => G.Tool.Translation)
                                      else G.Tool);
            Can  : constant Reach_Answer := Reach (W, (G with delta Tool => Goal));
         begin
            if Can.Status /= Reachable then
               Rejected (I) := True;
               R.Arms.Append (Arm_Result'(Arm => G.Arm, Outcome => Refused, Delivered => Unknown, Why => Can.Why));
            else
               W.Arms (I).Pending.Append (Command'(Due => W.Beat + W.Arms (I).Lag, Goal => Goal));
            end if;
         end;
      end loop;
      for C of O.Closers loop
         declare
            H : constant Positive := Hand_Index (W, C.Hand);
         begin
            W.Hands (H).Due := W.Beat + W.Arms (Arm_Index (W, W.Hands (H).Arm)).Lag;
            W.Hands (H).Next_Goal := C.Fraction;
         end;
      end loop;
      loop
         Advance (W);
         Beats := Beats + 1;
         exit when not O.Settle or else At_Rest (W) or else W.Beat >= W.Last_Beat;
      end loop;
      for G of O.Arms loop
         declare
            I : constant Positive := Arm_Index (W, G.Arm);
            A : constant Sim_Arm := W.Arms (I);
            D : constant Vec3 := A.Commanded.Translation - A.Start.Translation;
            Went : constant Vec3 := A.Tool.Translation - A.Start.Translation;
            Turned : constant Real := Angle (Transpose (A.Start.Rotation) * A.Commanded.Rotation);
            Share : constant Real :=
              (if abs D > 0.0 then Real'(Went * D) / Real'(D * D)
               elsif Turned > 0.0 then Angle (Transpose (A.Start.Rotation) * A.Tool.Rotation) / Turned
               else 1.0);
            Off : constant Real := abs (A.Commanded.Translation - A.Tool.Translation)
              + Angle (Transpose (A.Commanded.Rotation) * A.Tool.Rotation);
         begin
            if not Rejected (I) then
               R.Arms.Append (Arm_Result'(Arm       => G.Arm,
                               Outcome   => (if A.Blocked then Blocked
                                             elsif not Significant (Off, W.Sigma) then Reached else Short),
                               Delivered => (Value => Share, Sigma => W.Sigma / Real'Max (abs D, W.Sigma),
                                             Degrees_Of_Freedom => 0),
                               Why       => Null_Unbounded_String));
            end if;
         end;
      end loop;
      for C of O.Closers loop
         declare
            H : constant Sim_Hand := W.Hands (Hand_Index (W, C.Hand));
         begin
            R.Closers.Append (Closer_Result'(Hand => C.Hand, Outcome => (if H.Stopped then Blocked
                                                            elsif abs (H.Fraction - H.Goal) <= 1.0e-6 then Reached
                                                            else Short)));
         end;
      end loop;
      R.Beats := Beats;
   end Move;

   overriding function Predicted (W : World; T : Thing_Id; Beats : Natural) return Point_Estimate is
      I : constant Positive := Index_Of (W, T);
   begin
      return (Mean       => Centre_Of (W, I) + Real (Beats) * W.Things (I).Drift,
              Covariance => (W.Sigma * W.Sigma) * Identity3);
   end Predicted;

   overriding procedure Learn (W : in out World; L : Lesson) is
   begin
      if L.Kind = Friction_Learned then
         for B of W.Learned loop
            if B.Thing = L.Thing then
               B.Bounds := L.Bounds;
               return;
            end if;
         end loop;
         W.Learned.Append (Learned_Bound'(Thing => L.Thing, Bounds => L.Bounds));
      end if;
   end Learn;

   overriding function Episode_Over (W : World) return Boolean is (W.Beat >= W.Last_Beat);

   --  The still eye sees a cone of this half angle about its optical axis.
   Half_View : constant Real := 0.7;

   overriding function In_View (W : World; Point : Vec3) return Boolean is
      Q : constant Vec3 := Transpose (W.Eye.Rotation) * (Point - W.Eye.Translation);
   begin
      return Q (3) > 0.0 and then Arctan (Sqrt (Q (1) ** 2 + Q (2) ** 2), Q (3)) <= Half_View;
   end In_View;

   overriding procedure Within (W : in out World; During : not null access procedure) is
      pragma Unreferenced (W);
   begin
      During.all;
   end Within;

end Driver.Action.Plants.Tests;
