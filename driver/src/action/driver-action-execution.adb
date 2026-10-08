with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Action.Contact;
with Driver.Action.Contact.Search;
with Driver.Action.Contact.Wrench;
with Driver.Action.Goals;
with Driver.Action.Grids;
with Driver.Action.Monitor;
with Driver.Conventions;
with Driver.Log;
with Driver.Numerics;

package body Driver.Action.Execution is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Uncertain;
   use Driver.Action.Snapshots;
   use Driver.Action.Plants;
   use type Arm_Id;
   use type Hand_Id;
   use type Thing_Id;
   use type Surface_Id;
   use type Driver.Action.Goals.Quantity;
   use type Driver.Action.Goals.Pair_Relation;

   package Contact renames Driver.Action.Contact;
   package Search renames Driver.Action.Contact.Search;
   package Wrench renames Driver.Action.Contact.Wrench;

   Z  : constant Real := Driver.Conventions.Z;
   Pi : constant := Ada.Numerics.Pi;

   function Img (X : Real) return String is (Driver.Log.Image (X, 3));
   function Img (N : Integer) return String is (Driver.Log.Image (N));

   type State is record
      S       : Snapshot;
      Account : Unbounded_String;
      Tried   : Unbounded_String;
   end record;
   --  What a run has seen last, what it did, and what it tried that failed.

   procedure Say (X : in out State; Line : String) is
   begin
      if Length (X.Account) > 0 then
         Append (X.Account, "; ");
      end if;
      Append (X.Account, Line);
   end Say;

   procedure Note_Tried (X : in out State; Line : String) is
   begin
      if Length (X.Tried) > 0 then
         Append (X.Tried, "; ");
      end if;
      Append (X.Tried, Line);
   end Note_Tried;

   procedure Look (P : in out Plant'Class; X : in out State) is
   begin
      P.Look (X.S);
   end Look;

   function Gravity (S : Snapshot) return Vec3 is
     (if S.Up.Sigma < Real'Last and then abs S.Up.Unit_Vector > 0.0 then Unit (S.Up.Unit_Vector) else Zero3);

   function Largest_Sigma (C : Mat3) return Real is
     (Sqrt (Real'Max (C (1, 1), Real'Max (C (2, 2), C (3, 3)))));

   --  The tool pose after a share S of a unit twist: the tool turns about the
   --  twist's axis with whatever it holds, then shifts.
   function Moved (G : Contact.Twist; S : Real; Tool : Rigid) return Rigid is
      R : constant Mat3 := Exp (S * G.Angular);
   begin
      return (Rotation => R * Tool.Rotation, Translation => R * (Tool.Translation - G.Pivot) + G.Pivot + S * G.Linear);
   end Moved;

   --  The goal of taking an arm's tool to Tool, its orientation too, by any path.
   function Goal_Of (A : Arm_Id; Tool : Rigid) return Arm_Goal is
     ((Arm => A, Tool => Tool, Position_Only => False, others => <>));

   function One_Arm (S : Snapshot; A : Arm_Id; Tool : Rigid; Slack : Real := Real'Last) return Order;
   --  An order for one arm to take its tool to Tool, and to keep it within
   --  Slack of the straight path there on the way, as the engine cleared that
   --  path by that much; but not within less than the arm can tell its pose
   --  to, where keeping closer to the line learns nothing. Real'Last: not
   --  bounded.

   --  The smallest step the arm delivers distinguishably, for a twist: a
   --  length for a shift, an angle for a turn.
   function Resolution (A : Arm_State; G : Contact.Twist) return Real is
     (if abs G.Angular > 0.0 then A.Turn_Step.Value else A.Step.Value);

   --  The tool is at Goal as far as the arm can tell: nearer than its
   --  smallest step, or not significantly away given its own uncertainty.
   function At_Goal (A : Arm_State; Goal : Rigid) return Boolean is
      Off  : constant Real := abs (Goal.Translation - A.Tool.Pose.Translation);
      Turn : constant Real := Angle (Transpose (A.Tool.Pose.Rotation) * Goal.Rotation);
   begin
      return (Off <= A.Step.Value
              or else not Significant (Position (A.Tool), (Mean => Goal.Translation,
                                                           Covariance => [others => [others => 0.0]])))
        and then (Turn <= A.Turn_Step.Value
                  or else not Significant (Vector_Gate (3), Turn, Largest_Sigma (A.Tool.Rotation_Covariance)));
   end At_Goal;

   --  Brings the arm's tool to Goal, sending it again from wherever it got to
   --  until it is there as far as it can tell, the plant says it is, or a
   --  send moves it by nothing it can tell from noise.
   procedure Go (P : in out Plant'Class; X : in out State; A : Arm_Id; Goal : Rigid; Outcome : out Step_Outcome;
                 Why : out Unbounded_String; Slack : Real := Real'Last)
   is
   begin
      Why := Null_Unbounded_String;
      loop
         Look (P, X);
         declare
            R : Report;
         begin
            if At_Goal (Arm (X.S, A), Goal) then
               Outcome := Reached;
               return;
            elsif P.Episode_Over then
               Outcome := Short;
               return;
            end if;
            P.Move (One_Arm (X.S, A, Goal, Slack), R);
            declare
               Res : constant Arm_Result := R.Arms.First_Element;
            begin
               case Res.Outcome is
                  when Refused =>
                     Outcome := Refused;
                     Why := Res.Why;
                     Look (P, X);
                     return;
                  when Blocked =>
                     --  A push that stopped short after moving some may have
                     --  delivered only part of what was asked, as a body that
                     --  does not carry out all of it does; asked again from
                     --  where it got to, it either goes on or moves nothing,
                     --  and only that is a block.
                     Look (P, X);
                     if At_Goal (Arm (X.S, A), Goal) then
                        Outcome := Reached;
                        return;
                     elsif not (Known (Res.Delivered) and then Res.Delivered.Value > 0.0
                                and then Significant (Res.Delivered.Value, Res.Delivered.Sigma,
                                                      Res.Delivered.Degrees_Of_Freedom))
                     then
                        Outcome := Blocked;
                        return;
                     end if;
                  when Reached =>
                     Outcome := Reached;
                     Look (P, X);
                     return;
                  when Short =>
                     if not (Res.Delivered.Value > 0.0
                             and then Significant (Res.Delivered.Value, Res.Delivered.Sigma,
                                                   Res.Delivered.Degrees_Of_Freedom))
                     then
                        Look (P, X);
                        Outcome := (if At_Goal (Arm (X.S, A), Goal) then Reached else Blocked);
                        return;
                     end if;
               end case;
            end;
         end;
      end loop;
   end Go;

   procedure Set_Closer (P : in out Plant'Class; X : in out State; H : Hand_Id; Fraction : Real;
                         Outcome : out Step_Outcome)
   is
      R : Report;
   begin
      P.Move ((Arms    => Arm_Goal_Vectors.Empty_Vector,
               Closers => Closer_Goal_Vectors.To_Vector ((Hand => H, Fraction => Fraction), 1),
               Settle  => True), R);
      Outcome := (if R.Closers.Is_Empty then Refused else R.Closers.First_Element.Outcome);
      Look (P, X);
   end Set_Closer;

   --  The finest spacing of any thing's samples: what the body's points are
   --  laid at; Real'Last when no thing has samples.
   function Finest_Pitch (S : Snapshot) return Real is
      Spacing : Real := Real'Last;
   begin
      for T of S.Things loop
         if T.Pitch > 0.0 then
            Spacing := Real'Min (Spacing, T.Pitch);
         end if;
      end loop;
      return Spacing;
   end Finest_Pitch;

   --  The measured parts of the arm that can meet something, in the world
   --  with the tool at Tool: the lobes' faces and backs back to the depth of
   --  the hand, their ends, and the arm's own surface.
   function Body_Points (E : Search.Effector; Tool : Rigid; Spacing : Real) return Contact.Point_Vectors.Vector is
      Pts : Contact.Point_Vectors.Vector;
   begin
      for Pd of E.Pads loop
         declare
            F  : constant Real := E.Closers (Pd.Closer).Now;
            C  : constant Vec3 := Pd.Open + F * (Pd.Closed - Pd.Open) + E.Band * E.Along;   --  the lobe's end
            Across : constant Vec3 := Cross (Pd.Facing, E.Along);
            Wd : constant Vec3 := (if abs Across > 0.0 then Unit (Across) else Zero3);
            NW : constant Positive := Positive'Max (1, Natural (Real'Ceiling (2.0 * Pd.Half_Width / Spacing)));
            NL : constant Positive := Positive'Max (1, Natural (Real'Ceiling (E.Depth / Spacing)));
         begin
            for I in 0 .. NW loop
               for J in 0 .. NL loop
                  declare
                     Q : constant Vec3 := C + (2.0 * Real (I) / Real (NW) - 1.0) * Pd.Half_Width * Wd
                                            - (E.Depth * Real (J) / Real (NL)) * E.Along;
                  begin
                     Pts.Append (Tool * Q);
                     Pts.Append (Tool * (Q - Pd.Thickness * Pd.Facing));
                  end;
               end loop;
            end loop;
         end;
      end loop;
      for En of E.Ends loop
         Pts.Append (Tool * En.Point);
      end loop;
      for Sf of E.Surface loop
         Pts.Append (Tool * Sf.Point);
      end loop;
      return Pts;
   end Body_Points;

   --  How far from the tool any measured part of the arm lies, or any part of
   --  what it holds: what a turn of the tool carries about.
   function Lever_Of (S : Snapshot; A : Arm_Id) return Real is
      E       : constant Search.Effector := Search.Effector_Of (S, A);
      Spacing : constant Real := Finest_Pitch (S);
      Far     : Real := 0.0;
   begin
      for Q of Body_Points (E, E.Tool, (if Spacing = Real'Last then Arm (S, A).Step.Value else Spacing)) loop
         Far := Real'Max (Far, abs (Q - E.Tool.Translation));
      end loop;
      for T of S.Things loop
         if T.Held_By /= 0 and then Has_Hand (S, T.Held_By) and then Hand (S, T.Held_By).Arm = A then
            for Smp of T.Samples loop
               Far := Real'Max (Far, abs (Smp.Point - E.Tool.Translation));
            end loop;
         end if;
      end loop;
      return Far;
   end Lever_Of;

   function One_Arm (S : Snapshot; A : Arm_Id; Tool : Rigid; Slack : Real := Real'Last) return Order is
      Sigma : constant Real := Search.Effector_Of (S, A).Sigma;
   begin
      return (Arms     => Arm_Goal_Vectors.To_Vector
                            ((Goal_Of (A, Tool)
                              with delta Clearance => (if Slack = Real'Last or else Sigma = Real'Last then Real'Last
                                                       else Real'Max (Slack, Z * Sigma)),
                                         Lever     => Lever_Of (S, A)), 1),
              Closers  => Closer_Goal_Vectors.Empty_Vector,
              Settle   => True);
   end One_Arm;

   --  The margin a body part known to Sigma keeps from a thing's samples:
   --  Z of the two sigmas together, and half a pitch between samples.
   function Margin_From (T : Thing_State; Sigma : Real) return Real is
     (Z * Sqrt (Sigma ** 2 + T.Sigma ** 2) + T.Pitch / 2.0);

   --  The margin the body keeps from a path it means to take, known to Sigma:
   --  Z of it, and half the finest spacing of the samples a path is cleared by;
   --  Real'Last where Sigma is not measured.
   function Margin_Of (S : Snapshot; Sigma : Real) return Real is
     (if Sigma = Real'Last then Real'Last
      else Z * Sigma + (if Finest_Pitch (S) < Real'Last then Finest_Pitch (S) / 2.0 else 0.0));

   --  The samples of every thing but Except and Held, filed for nearness.
   function Obstacles (S : Snapshot; Sigma : Real; Except, Held : Thing_Id'Base) return Grids.Grid is
      G    : Grids.Grid;
      Cube : Real := 0.0;
   begin
      for T of S.Things loop
         if T.Id /= Except and then T.Id /= Held and then T.Sigma < Real'Last then
            Cube := Real'Max (Cube, Margin_From (T, Sigma));
         end if;
      end loop;
      Grids.Start (G, (if Cube > 0.0 then Cube else Real'Last));
      for T of S.Things loop
         if T.Id /= Except and then T.Id /= Held and then T.Sigma < Real'Last then
            for Smp of T.Samples loop
               Grids.Add (G, Smp.Point, Margin_From (T, Sigma));
            end loop;
         end if;
      end loop;
      return G;
   end Obstacles;

   --  The least distance from any of the points to the surfaces and the
   --  filed samples, less the margin each needs: exact when below zero, and
   --  never below zero when the true one is not (Driver.Action.Grids).
   function Least_Gap (S : Snapshot; Near : Grids.Grid; Points : Contact.Point_Vectors.Vector; Sigma : Real)
     return Real
   is
      Least : Real := Real'Last;
   begin
      for F of S.Surfaces loop
         if F.Of_Thing = 0 and then abs F.Normal.Unit_Vector > 0.0 then
            declare
               N      : constant Vec3 := Unit (F.Normal.Unit_Vector);
               Margin : constant Real := Z * Sqrt (Sigma ** 2 + Largest_Sigma (F.Point.Covariance) ** 2);
            begin
               for Q of Points loop
                  Least := Real'Min (Least, Real'((Q - F.Point.Mean) * N) - Margin);
               end loop;
            end;
         end if;
      end loop;
      for Q of Points loop
         Least := Real'Min (Least, Grids.Least_Gap (Near, Q));
      end loop;
      return Least;
   end Least_Gap;

   --  By how much the arm's measured parts, and what it holds, moving straight
   --  from one tool pose to another, keep clear of the surfaces and of every
   --  other thing, at the nearest: the least gap along the way, less the gap
   --  the start lacks where it begins nearer than the margin asks (coming no
   --  nearer than it was at the start is always clear). Below zero as soon
   --  as the way is not clear, by no more than is known of how far; Real'Last
   --  where nothing is measured to keep clear of.
   function Slack_Of (S : Snapshot; E : Search.Effector; From, To : Rigid; Held : Thing_Id'Base; Near : Grids.Grid)
     return Real
   is
      Spacing : constant Real := Finest_Pitch (S);
      Lever   : Real := 0.0;
      Carried : Contact.Point_Vectors.Vector;
   begin
      if Spacing = Real'Last then
         return Real'Last;
      end if;
      if Held /= 0 and then Has_Thing (S, Held) then
         for Smp of Thing (S, Held).Samples loop
            Carried.Append (Smp.Point);
         end loop;
      end if;
      declare
         function At_Pose (Tool : Rigid) return Contact.Point_Vectors.Vector is
            Pts   : Contact.Point_Vectors.Vector := Body_Points (E, Tool, Spacing);
            Carry : constant Rigid := Tool * Inverse (From);
         begin
            for C of Carried loop
               Pts.Append (Carry * C);
            end loop;
            return Pts;
         end At_Pose;
         Start_Gap : constant Real := Least_Gap (S, Near, At_Pose (From), E.Sigma);
         Lacking   : constant Real := Real'Min (0.0, Start_Gap);   --  what the start lacks of the margin
         Least     : Real := Start_Gap;
      begin
         for P of At_Pose (From) loop
            Lever := Real'Max (Lever, abs (P - From.Translation));
         end loop;
         declare
            Shift  : constant Real := abs (To.Translation - From.Translation);
            Turn   : constant Real := Angle (Transpose (From.Rotation) * To.Rotation);
            Pieces : constant Positive := Positive'Max (1, Natural (Real'Ceiling ((Shift + Turn * Lever) / Spacing)));
         begin
            for I in 1 .. Pieces loop
               declare
                  Share : constant Real := Real (I) / Real (Pieces);
                  Pose  : constant Rigid :=
                    (Rotation    => From.Rotation
                                      * Exp (Share * Driver.Numerics.Log (Transpose (From.Rotation) * To.Rotation)),
                     Translation => From.Translation + Share * (To.Translation - From.Translation));
               begin
                  Least := Real'Min (Least, Least_Gap (S, Near, At_Pose (Pose), E.Sigma));
                  if Least < Lacking then
                     return Least - Lacking;
                  end if;
               end;
            end loop;
         end;
         return (if Least = Real'Last then Real'Last else Least - Lacking);
      end;
   end Slack_Of;

   --  Whether the way keeps clear: it has no negative slack.
   function Clear_Way (S : Snapshot; E : Search.Effector; From, To : Rigid; Held : Thing_Id'Base; Near : Grids.Grid)
     return Boolean is (Slack_Of (S, E, From, To, Held, Near) >= 0.0);

   --  By how much the arm may leave the straight path of a step to Tool that
   --  moves thing T (held by this arm or pushed by it): as much as the path
   --  keeps clear of everything but T, with T carried along when held. A
   --  thing that slides along a surface keeps no clearance from it, so a step
   --  of it may leave its path by no more than the arm can tell.
   function Step_Slack (S : Snapshot; A : Arm_Id; T : Thing_Id'Base; Tool : Rigid) return Real is
      E    : constant Search.Effector := Search.Effector_Of (S, A);
      Held : constant Thing_Id'Base :=
        (if T /= 0 and then Has_Thing (S, T) and then Thing (S, T).Held_By /= 0 then T else 0);
   begin
      return Slack_Of (S, E, E.Tool, Tool, Held, Obstacles (S, E.Sigma, T, Held));
   end Step_Slack;

   --  Whether the arm's travel can bring its body to Tool, its closers at
   --  Fractions: there it keeps the clearance travel keeps from the surfaces
   --  and from every thing, or comes no nearer than it is now.
   function Free_At (S : Snapshot; A : Arm_Id; Near : Grids.Grid; Tool : Rigid;
                     Fractions : Search.Real_Vectors.Vector) return Boolean
   is
      E       : Search.Effector := Search.Effector_Of (S, A);
      Spacing : constant Real := Finest_Pitch (S);
   begin
      if Spacing = Real'Last then
         return True;
      end if;
      for K in 1 .. Natural'Min (Natural (E.Closers.Length), Natural (Fractions.Length)) loop
         E.Closers (K).Now := Fractions (K);
      end loop;
      declare
         Gap : constant Real := Least_Gap (S, Near, Body_Points (E, Tool, Spacing), E.Sigma);
      begin
         return Gap >= 0.0 or else Gap >= Least_Gap (S, Near, Body_Points (E, E.Tool, Spacing), E.Sigma);
      end;
   end Free_At;

   --  Takes the arm to Goal by the lowest clear way: straight when that is
   --  clear, else over the top, raised by its own reach doubled until the
   --  way over is clear; planned again from wherever each step got it.
   procedure Travel (P : in out Plant'Class; X : in out State; A : Arm_Id; Goal : Rigid; Except, Held : Thing_Id'Base;
                     Outcome : out Step_Outcome; Why : out Unbounded_String)
   is
      --  The goal was found clear by the arm's uncertainty where it began; the
      --  arm is less sure of its pose farther from where it was measured, and
      --  a way judged by that on the way would refuse the pose it is going to
      --  for the margin the search had kept.
      Sigma : constant Real := Search.Effector_Of (X.S, A).Sigma;
   begin
      Why := Null_Unbounded_String;
      loop
         declare
            E    : constant Search.Effector := (Search.Effector_Of (X.S, A) with delta Sigma => Sigma);
            From : constant Rigid := E.Tool;
            Now  : constant Arm_State := Arm (X.S, A);
            Up   : constant Vec3 := Gravity (X.S);
            Near : constant Grids.Grid := Obstacles (X.S, E.Sigma, Except, Held);
         begin
            if At_Goal (Now, Goal) then
               Outcome := Reached;
               return;
            end if;
            declare
               Straight : constant Real :=
                 (if abs Up > 0.0 then Slack_Of (X.S, E, From, Goal, Held, Near) else Real'Last);
            begin
               if Straight >= 0.0 then
                  Go (P, X, A, Goal, Outcome, Why, Straight);
                  return;
               end if;
            end;
            declare
               --  The way over runs at one level, up from where the arm is,
               --  across, and down to the goal: the higher of the two to begin
               --  with (a hand already above what is in the way needs no
               --  rise), then raised by its own reach, doubled.
               Base_Level : constant Real := Real'Max (From.Translation * Up, Goal.Translation * Up);
               Lift : Real := 0.0;
               Via_1, Via_2 : Rigid;
               Found : Boolean := False;
               --  Raised until the way over is clear or out of reach, where the
               --  body's reach can be asked.
               procedure Over_The_Top is
               begin
                  loop
                     Via_1 := (Rotation    => From.Rotation,
                               Translation => From.Translation + (Base_Level + Lift - From.Translation * Up) * Up);
                     Via_2 := (Rotation    => Goal.Rotation,
                               Translation => Goal.Translation + (Base_Level + Lift - Goal.Translation * Up) * Up);
                     exit when P.Reach (Goal_Of (A, Via_1)).Status /= Reachable
                       or else P.Reach (Goal_Of (A, Via_2)).Status /= Reachable;
                     if Clear_Way (X.S, E, From, Via_1, Held, Near)
                       and then Clear_Way (X.S, E, Via_1, Via_2, Held, Near)
                       and then Clear_Way (X.S, E, Via_2, Goal, Held, Near)
                     then
                        Found := True;
                        exit;
                     end if;
                     Lift := (if Lift > 0.0 then 2.0 * Lift else Real'Max (E.Depth + E.Sigma, Now.Step.Value));
                  end loop;
               end Over_The_Top;
            begin
               P.Within (Over_The_Top'Access);
               if not Found then
                  Outcome := Refused;
                  Why := To_Unbounded_String ("no clear way there: straight is blocked and every way over the top "
                                              & "up to " & Img (Lift) & " above the higher of the two ends is out of reach");
                  return;
               end if;
               --  The first leg not done yet.
               declare
                  Raised : constant Boolean :=
                    Real'((From.Translation - Via_1.Translation) * Up) >= -Now.Step.Value;
                  Over   : constant Boolean :=
                    abs (From.Translation - Via_2.Translation) <= Now.Step.Value;
                  Leg    : constant Rigid := (if Over then Goal elsif Raised then Via_2 else Via_1);
               begin
                  Go (P, X, A, Leg, Outcome, Why, Slack_Of (X.S, E, From, Leg, Held, Near));
                  if Outcome in Refused | Blocked then
                     return;
                  end if;
               end;
            end;
         end;
      end loop;
   end Travel;

   --  Everything measured near the thing but itself and what holds it.
   function Beside_Of (S : Snapshot; T : Thing_Id) return Contact.Point_Vectors.Vector is
      B : Contact.Point_Vectors.Vector;
   begin
      for O of S.Things loop
         if O.Id /= T then
            for Smp of O.Samples loop
               B.Append (Smp.Point);
            end loop;
         end if;
      end loop;
      return B;
   end Beside_Of;


   function Busy (S : Snapshot; A : Arm_Id; T : Thing_Id'Base) return Boolean is
     (for some O of S.Things => O.Id /= T and then O.Held_By /= 0 and then Has_Hand (S, O.Held_By)
                                and then Hand (S, O.Held_By).Arm = A);
   --  The arm holds something else.

   type Grip is record
      Arm      : Arm_Id := Arm_Id'First;
      Hands    : Search.Closer_Vectors.Vector;   --  the closers that close on it
      Closing  : Boolean := False;               --  it is held between lobes, not touched by one part
      Searched : Boolean := False;               --  Chosen is the contact set this run picked
      Chosen   : Search.Candidate;
   end record;

   function Grip_Of_Arm (S : Snapshot; A : Arm_Id; Closing : Boolean) return Grip is
     ((Arm => A, Hands => Search.Effector_Of (S, A).Closers, Closing => Closing, Searched => False, Chosen => <>));

   --  The arm a role binds to now: of those that can play it (a grasper
   --  closes lobes on things, a pusher touches without closing, me carries
   --  the whole body and every eye), the nearest to Near when there is
   --  something to be near, else the first measured.
   function Bound (S : Snapshot; R : Role; Near : Vec3; Has_Near : Boolean; A : out Arm_Id) return Boolean is
      Best  : Real := Real'Last;
      Found : Boolean := False;
   begin
      A := Arm_Id'First;
      for Arm_S of S.Arms loop
         declare
            E    : constant Search.Effector := Search.Effector_Of (S, Arm_S.Id);
            Fits : constant Boolean :=
              (case R is
                  when Grasper => E.Closes,
                  when Pusher  => not E.Closes and then not (E.Surface.Is_Empty and then E.Ends.Is_Empty),
                  when Me      => Arm_S.Carries_All);
         begin
            if Fits and then (if Has_Near then abs (E.Tool.Translation - Near) < Best else not Found) then
               Best := abs (E.Tool.Translation - Near);
               A := Arm_S.Id;
               Found := True;
            end if;
         end;
      end loop;
      return Found;
   end Bound;

   function Bindable (S : Snapshot; R : Role) return Boolean is
      A : Arm_Id;
   begin
      return Bound (S, R, Zero3, False, A);
   end Bindable;

   function Bound_Arm (S : Snapshot; R : Role; A : out Arm_Id) return Boolean is (Bound (S, R, Zero3, False, A));

   function Usable (S : Snapshot; R : Relation) return Boolean is
      Arms      : constant Boolean := not S.Arms.Is_Empty;
      Up        : constant Boolean := abs Gravity (S) > 0.0;
      Still_Eye : constant Boolean := (for some E of S.Eyes => E.On_Arm = 0 and then Known (Position (E.Pose)));
   begin
      return (case R is
                 when Touching | Press | Clear | Still    => Arms,
                 when Above | Below | Onto | Off | Facing => Arms and then Up,
                 when Left | Right | Nearer | Farther     => Arms and then Still_Eye,
                 when Close | Open                        => Bindable (S, Grasper),
                 when Into                                => False);
   end Usable;

   function Role_Word (R : Role) return String is
     (case R is when Me => "me", when Grasper => "grasper", when Pusher => "pusher");

   --  Brings a part of the body into the contact set the search picks for
   --  Motion, over every arm free to do it (or only Only_Arm); on a closing
   --  set, closes on the thing. A closing that finds nothing between the
   --  lobes is tried again from a new look only if it changed something.
   --  Touch_Only takes a single touch and stops where the last straight
   --  stretch begins, for the caller to come in until it meets the thing.
   procedure Acquire (P : in out Plant'Class; X : in out State; T : Thing_Id; Motion : Contact.Twist; G : out Grip;
                      Ok : out Boolean; Only_Arm : Arm_Id'Base := 0; Touch_Only : Boolean := False)
   is
   begin
      Ok := False;
      loop
         declare
            Best  : Search.Candidate;
            Force : Real := Real'Last;
            Arm_Of_Best : Arm_Id := Arm_Id'First;
            Before_Centre : constant Point_Estimate := Thing (X.S, T).Centre;
            --  Every arm's search, where the body's reach can be asked.
            procedure Searching is
            begin
               for A of X.S.Arms loop
                  if Only_Arm /= 0 and then A.Id /= Only_Arm then
                     null;
                  elsif Busy (X.S, A.Id, T) then
                     Note_Tried (X, "arm " & Img (Integer (A.Id)) & " holds something else");
                  else
                     declare
                        E     : constant Search.Effector := Search.Effector_Of (X.S, A.Id);
                        Arm_Id_Now : constant Arm_Id := A.Id;
                        --  What the travel there keeps clear of: everything.
                        Near  : constant Grids.Grid := Obstacles (X.S, E.Sigma, 0, 0);
                        function Can_Reach (Tool : Rigid) return Boolean is
                          (P.Reach (Goal_Of (Arm_Id_Now, Tool)).Status = Reachable);
                        function Can_Be_Free (Tool : Rigid; Fractions : Search.Real_Vectors.Vector) return Boolean is
                          (Free_At (X.S, Arm_Id_Now, Near, Tool, Fractions));
                        C     : Search.Candidate;
                        Found : Boolean;
                        Acc   : Search.Account;
                     begin
                        Search.Find (Search.Shape_Of (X.S, T), Beside_Of (X.S, T), E, Motion, Gravity (X.S),
                                     Thing (X.S, T).Friction, Can_Reach'Access, Can_Be_Free'Access, C, Found, Acc,
                                     Touch_Only);
                        if Found and then C.Force < Force then
                           Best := C;
                           Force := C.Force;
                           Arm_Of_Best := A.Id;
                        elsif not Found then
                           Note_Tried (X, "arm " & Img (Integer (A.Id)) & ": " & Search.Say (Acc));
                        end if;
                     end;
                  end if;
               end loop;
            end Searching;
         begin
            P.Within (Searching'Access);
            if Force = Real'Last then
               return;
            end if;
            declare
               E   : constant Search.Effector := Search.Effector_Of (X.S, Arm_Of_Best);
               Out_Come : Step_Outcome;
               Why : Unbounded_String;
               Several : constant Boolean := Natural (Best.Touches.Length) > 1;
            begin
               G := (Arm => Arm_Of_Best, Hands => E.Closers, Closing => Several, Searched => True, Chosen => Best);
               Say (X, "I meet it with arm " & Img (Integer (Arm_Of_Best)) & " at "
                    & Img (Natural (Best.Touches.Length)) & " touch" & (if Several then "es" else "")
                    & ", needing friction " & Img (Best.Mu_Worst) & " at most");
               if G.Closing then
                  for K in 1 .. Natural (E.Closers.Length) loop
                     Set_Closer (P, X, E.Closers (K).Hand, Best.Before (K), Out_Come);
                  end loop;
               end if;
               Travel (P, X, Arm_Of_Best, Best.Hover, 0, 0, Out_Come, Why);
               if Out_Come in Refused | Blocked then
                  Note_Tried (X, "going to where the last straight stretch begins: "
                              & (if Out_Come = Refused then To_String (Why) else "something stopped the arm"));
                  return;
               end if;
               if Touch_Only then
                  Ok := True;
                  return;
               end if;
               --  The lobes were cleared of the thing by the search's margin all the way in.
               Go (P, X, Arm_Of_Best, Best.Tool, Out_Come, Why,
                   Slack => Margin_Of (X.S, Sqrt (E.Sigma ** 2 + Thing (X.S, T).Sigma ** 2)));
               if Out_Come = Refused then
                  Note_Tried (X, "coming in to touch it: " & To_String (Why));
                  return;
               elsif Out_Come = Blocked then
                  Say (X, "something stopped my hand " & Img (abs (Arm (X.S, Arm_Of_Best).Tool.Pose.Translation
                                                                  - Best.Tool.Translation))
                       & " short of where it was to touch");
               end if;
               if not G.Closing then
                  Ok := True;
                  return;
               end if;
               for K in 1 .. Natural (E.Closers.Length) loop
                  Set_Closer (P, X, E.Closers (K).Hand, 1.0, Out_Come);
               end loop;
               --  Held: the closers stopped short of closed on nothing.
               declare
                  Holding : Boolean := True;
               begin
                  for C of E.Closers loop
                     declare
                        F : constant Estimate := Hand (X.S, C.Hand).Fraction;
                     begin
                        if not (Known (F) and then Significant (1.0 - F.Value, F.Sigma, F.Degrees_Of_Freedom)) then
                           Holding := False;
                        end if;
                     end;
                  end loop;
                  if Holding then
                     Say (X, "my lobes closed on it");
                     Ok := True;
                     return;
                  end if;
               end;
               Note_Tried (X, "closing arm " & Img (Integer (Arm_Of_Best)) & "'s lobes on it: they closed on nothing");
               for C of E.Closers loop
                  Set_Closer (P, X, C.Hand, 0.0, Out_Come);
               end loop;
               if not Significant (Before_Centre, Thing (X.S, T).Centre) then
                  Note_Tried (X, "it did not move, so the same try would end the same way");
                  return;
               end if;
               Say (X, "it moved while I closed, so I look again and choose anew");
            end;
         end;
      end loop;
   end Acquire;

   --  The points that move in a step: the thing's surface, or when no thing
   --  is moved, the arm's own parts that can meet something.
   function Moving_Points (S : Snapshot; T : Thing_Id'Base; A : Arm_Id) return Contact.Point_Vectors.Vector is
      Pts : Contact.Point_Vectors.Vector;
   begin
      if T /= 0 then
         for Smp of Thing (S, T).Samples loop
            Pts.Append (Smp.Point);
         end loop;
         return Pts;
      end if;
      declare
         E       : constant Search.Effector := Search.Effector_Of (S, A);
         Spacing : constant Real := Finest_Pitch (S);
      begin
         return Body_Points (E, E.Tool, (if Spacing = Real'Last then Arm (S, A).Step.Value else Spacing));
      end;
   end Moving_Points;

   --  The arm's touching parts as one side of a relation: their middle (the
   --  lobes' faces, else its own surface, else its tool), and their points.
   function Effector_Item (S : Snapshot; A : Arm_Id) return Goals.Item is
      E   : constant Search.Effector := Search.Effector_Of (S, A);
      Sum : Vec3 := Zero3;
      N   : Natural := 0;
      It  : Goals.Item;
   begin
      for Pd of E.Pads loop
         Sum := Sum + E.Tool * (Pd.Open + E.Closers (Pd.Closer).Now * (Pd.Closed - Pd.Open));
         N := N + 1;
      end loop;
      if N = 0 then
         for Sf of E.Surface loop
            Sum := Sum + E.Tool * Sf.Point;
            N := N + 1;
         end loop;
      end if;
      It.Centre := (Mean => (if N > 0 then Sum / Real (N) else E.Tool.Translation),
                    Covariance => Arm (S, A).Tool.Position_Covariance);
      It.Sigma := E.Sigma;
      for Q of Moving_Points (S, 0, A) loop
         It.Samples.Append (Sample'(Point => Q, Normal => Zero3));
      end loop;
      return It;
   end Effector_Item;

   function Part_Point (S : Snapshot; A : Arm_Id) return Point_Estimate is (Effector_Item (S, A).Centre);

   --  How far along the unit twist the moving points can go before one of
   --  them meets a surface or a thing other than Except, and the band of
   --  that distance's uncertainty.
   procedure Contact_Ahead (S : Snapshot; Moving : Contact.Point_Vectors.Vector; Sigma : Real; Except : Thing_Id'Base;
                            G : Contact.Twist; Ahead, Band : out Real)
   is
   begin
      Ahead := Real'Last;
      Band := 0.0;
      for Q0 of Moving loop
         declare
            V  : constant Vec3 := Contact.Velocity (G, Q0);
            VV : constant Real := V * V;
         begin
            if VV > 0.0 then
               for F of S.Surfaces loop
                  if F.Of_Thing = 0 and then abs F.Normal.Unit_Vector > 0.0 then
                     declare
                        N       : constant Vec3 := Unit (F.Normal.Unit_Vector);
                        In_Rate : constant Real := -Real'(V * N);
                        H       : constant Real := (Q0 - F.Point.Mean) * N;
                        B       : constant Real := Z * Sqrt (Sigma ** 2 + Largest_Sigma (F.Point.Covariance) ** 2);
                        --  The motion goes into the surface only by more than the surface's orientation, known to
                        --  its angular sigma and never better than rounding, allows it to go along it: a rotation
                        --  about its normal, or a slide along it, does not approach it, and a rate that is not told
                        --  from none would put the contact anywhere from here to infinity.
                        Approach : constant Boolean :=
                          In_Rate > 0.0
                          and then (F.Normal.Sigma = Real'Last
                                    or else Significant (In_Rate, Sqrt (VV) * Real'Max (F.Normal.Sigma, Real (Vec3'Length) * Real'Epsilon)));
                     begin
                        if Approach and then H > -B and then Real'Max (0.0, H) / In_Rate < Ahead then
                           Ahead := Real'Max (0.0, H) / In_Rate;
                           Band := B / In_Rate;
                        end if;
                     end;
                  end if;
               end loop;
               for O of S.Things loop
                  if O.Id /= Except then
                     declare
                        B        : constant Real := Z * Sqrt (Sigma ** 2 + O.Sigma ** 2);
                        Reach_Of : constant Real := O.Pitch / 2.0 + B;
                        Behind   : constant Real := B / Sqrt (VV);   --  how far behind is still at it, in the noise
                     begin
                        --  As for a surface: a sample passed by less than the noise
                        --  of the two, whose surface the motion goes into, is met
                        --  now, not ignored.
                        for Q of O.Samples loop
                           declare
                              D     : constant Vec3 := Q.Point - Q0;
                              Along : constant Real := Real'(D * V) / VV;
                           begin
                              if (Along > 0.0 or else (Along > -Behind and then Real'(V * Q.Normal) < 0.0))
                                and then Real'Max (0.0, Along) < Ahead
                                and then abs (D - Along * V) <= Reach_Of
                              then
                                 Ahead := Real'Max (0.0, Along);
                                 Band := Behind;
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
            end if;
         end;
      end loop;
   end Contact_Ahead;

   --  The thing's own lowest layer over what bears it, as its footing.
   function Base_Over_Support (S : Snapshot; T : Thing_Id) return Contact.Footing is
      X    : constant Thing_State := Thing (S, T);
      N    : constant Vec3 := Goals.Up_Of (S, T);
      Low  : Real := Real'Last;
      Pts  : Contact.Point_Vectors.Vector;
      Under : constant Thing_Id'Base :=
        (if X.Support /= 0 and then Has_Surface (S, X.Support) then Surface (S, X.Support).Of_Thing else 0);
      Band : constant Real := Real'Max (X.Pitch, Z * X.Sigma);
   begin
      if not (abs N > 0.0) then
         return Contact.No_Footing;
      end if;
      for Smp of X.Samples loop
         Low := Real'Min (Low, Smp.Point * N);
      end loop;
      for Smp of X.Samples loop
         if Smp.Point * N - Low <= Band then
            if Under = 0 or else not Has_Thing (S, Under) then
               Pts.Append (Smp.Point);
            else
               declare
                  Below : constant Thing_State := Thing (S, Under);
                  Over  : Boolean := False;
               begin
                  for Q of Below.Samples loop
                     if Q.Normal * N > 0.0 then
                        declare
                           D : constant Vec3 := Q.Point - Smp.Point;
                        begin
                           if abs (D - Real'(D * N) * N) <= Below.Pitch then
                              Over := True;
                              exit;
                           end if;
                        end;
                     end if;
                  end loop;
                  if Over then
                     Pts.Append (Smp.Point);
                  end if;
               end;
            end if;
         end if;
      end loop;
      return Contact.Footing_Of (Pts, Low * N, N, X.Pitch);
   end Base_Over_Support;

   --  Moves the subject step by step along what Next_Goal wants, until the
   --  monitor names an ending. The subject is the thing T, held or touched
   --  as G says, or when T is 0 the arm's own touching parts. A relation
   --  that ends in contact (Arrive_By_Touch) goes on past where the geometry
   --  says it holds, by the arm's smallest step, until the touch is felt:
   --  the arm is stopped where the contact is, or the thing Felt, which the
   --  subject is coming in to, gives way and is seen to move.
   procedure Carry (P : in out Plant'Class; X : in out State; T : Thing_Id'Base; G : Grip;
                    Next_Goal : not null access function (S : Snapshot) return Goals.Answer;
                    Wanted : Ending_Set; Max_Steps : Natural; Final : out Ending;
                    Arrive_By_Touch : Boolean := False; Felt : Thing_Id'Base := 0)
   is
      Watch     : Monitor.Watch := Monitor.Start;
      Start     : constant Point_Estimate :=
        (if T /= 0 then Thing (X.S, T).Centre else Effector_Item (X.S, G.Arm).Centre);
      Up0       : constant Vec3 := (if T /= 0 then Goals.Up_Of (X.S, T) else Gravity (X.S));
      Supported : constant Boolean := T /= 0 and then Thing (X.S, T).Support /= 0;
      --  A thing that was moving already says nothing by moving on.
      Felt_Still : constant Boolean := Felt /= 0 and then Has_Thing (X.S, Felt) and then not Thing (X.S, Felt).Moving;
   begin
      loop
         declare
            Goal    : constant Goals.Answer := Next_Goal (X.S);
            Now     : constant Arm_State := Arm (X.S, G.Arm);
            Centre  : constant Point_Estimate :=
              (if T /= 0 then Thing (X.S, T).Centre else Effector_Item (X.S, G.Arm).Centre);
            Sigma   : constant Real := (if T /= 0 then Thing (X.S, T).Sigma else Search.Effector_Of (X.S, G.Arm).Sigma);
            Pushing : constant Boolean := Goal.Ok and then Goal.Done and then Arrive_By_Touch;
            F       : Monitor.Facts;
            Step    : Real := 0.0;
            Ahead, Band : Real := Real'Last;
            Res     : Arm_Result;

            --  How far this step goes, where the body's reach and view can be
            --  asked: as far as the contact ahead, the goal, a wanted freedom
            --  and the reach and view allow.
            procedure Choosing is
               Fine  : constant Real := Resolution (Now, Goal.Motion);
               Limit : Real := Real'Last;
               function Fits (S : Real) return Boolean is
                 (P.Reach (Goal_Of (G.Arm, Moved (Goal.Motion, S, Now.Tool.Pose))).Status = Reachable
                  and then P.In_View (Contact.Apply (Contact.Scaled (Goal.Motion, S), Centre.Mean)));
            begin
               --  The contact is somewhere in its band ahead: a step goes up to
               --  the band's near end; from there one step crosses the band,
               --  since being stopped anywhere in it is the touch that was
               --  expected (the finest step, over a band that is thousands of
               --  them wide, would take thousands of steps to find it).
               if Ahead < Real'Last then
                  Limit := (if Ahead - Band > Fine then Ahead - Band else Real'Max (Fine, Ahead + Band));
               end if;
               if Pushing then
                  Limit := (if Ahead < Real'Last then Real'Max (Fine, Ahead + Band) else Fine);
               elsif Known (Goal.Gap) then
                  Limit := Real'Min (Limit, Real'Max (Fine, Real'Min (Goal.Gap.Value, Goal.Leg)));
               elsif Goal.Leg < Real'Last then
                  Limit := Real'Min (Limit, Real'Max (Fine, Goal.Leg));
               end if;
               if Wanted (Free) and then Supported then
                  --  The rise is judged against its own noise: go just far enough for that.
                  Limit := Real'Min (Limit, Real'Max (Fine, Z * Sqrt (2.0) * Largest_Sigma (Start.Covariance)
                                                       - Real'((Centre.Mean - Start.Mean) * Up0)));
               end if;
               --  A move turns no more than half a circle (past it the same
               --  pose is nearer the other way, and the pose after a whole
               --  circle fits wherever the pose before it did), and no more
               --  than keeps the tool's straight path within the margin of the
               --  arc the thing is to follow: the tool goes along the chord
               --  while it turns, which leaves the arc by the lever times one
               --  minus the cosine of half the turn.
               if abs Goal.Motion.Angular > 0.0 then
                  declare
                     Axis   : constant Vec3 := Unit (Goal.Motion.Angular);
                     Apart  : constant Vec3 := Now.Tool.Pose.Translation - Goal.Motion.Pivot;
                     Lever  : constant Real := abs (Apart - Real'(Apart * Axis) * Axis);
                     Margin : constant Real := Margin_Of (X.S, Sigma);
                     Turn   : constant Real :=
                       (if Margin < 2.0 * Lever then 2.0 * Arccos (1.0 - Margin / Lever) else Pi);
                  begin
                     Limit := Real'Min (Limit, Real'Min (Pi, Turn) / abs Goal.Motion.Angular);
                  end;
               end if;
               if Limit < Real'Last and then Fits (Limit) then
                  Step := Limit;
               else
                  --  Doubled from the smallest step while it fits, then halved back down to it.
                  declare
                     Low  : Real := 0.0;
                     High : Real := Fine;
                  begin
                     while High < Limit and then Fits (High) loop
                        Low := High;
                        High := 2.0 * High;
                     end loop;
                     High := Real'Min (High, Limit);
                     while High - Low > Fine loop
                        if Fits ((Low + High) / 2.0) then
                           Low := (Low + High) / 2.0;
                        else
                           High := (Low + High) / 2.0;
                        end if;
                     end loop;
                     Step := Low;
                  end;
               end if;
               if Step < Fine then
                  Step := 0.0;
               end if;
            end Choosing;
         begin
            if Goal.Ok and then (not Goal.Done or else Pushing) then
               Contact_Ahead (X.S, Moving_Points (X.S, T, G.Arm), Sigma, T, Goal.Motion, Ahead, Band);
               P.Within (Choosing'Access);
            end if;
            F.Commanded := Step > 0.0;
            F.Exhausted := Goal.Ok and then (not Goal.Done or else Pushing) and then Step = 0.0;
            if F.Commanded then
               declare
                  R    : Report;
                  Tool : constant Rigid := Moved (Goal.Motion, Step, Now.Tool.Pose);
               begin
                  P.Move (One_Arm (X.S, G.Arm, Tool, Step_Slack (X.S, G.Arm, T, Tool)), R);
                  Res := R.Arms.First_Element;
               end;
            else
               --  Nothing to move: one beat passes while the scene is watched.
               declare
                  R : Report;
               begin
                  P.Move ((Arms => Arm_Goal_Vectors.Empty_Vector, Closers => Closer_Goal_Vectors.Empty_Vector,
                           Settle => False), R);
               end;
            end if;
            Look (P, X);
            declare
               After_Tool : constant Rigid := Arm (X.S, G.Arm).Tool.Pose;
               Carried    : constant Rigid := After_Tool * Inverse (Now.Tool.Pose);
               Delivered  : constant Real := (if F.Commanded and then Known (Res.Delivered)
                                              then Real'Max (0.0, Res.Delivered.Value) * Step else 0.0);
            begin
               F.Blocked := F.Commanded and then Res.Outcome = Blocked and then Delivered < Ahead - Band;
               F.Touch := F.Commanded and then Res.Outcome = Blocked and then not F.Blocked;
               if F.Commanded and then Felt_Still and then Has_Thing (X.S, Felt) and then Thing (X.S, Felt).Moving then
                  F.Touch := True;
                  F.Blocked := False;
               end if;
               if F.Commanded and then Res.Outcome = Refused then
                  F.Blocked := True;
                  Say (X, "the arm refused the step: " & To_String (Res.Why));
               end if;
               if T = 0 then
                  F.Seen := True;
                  F.Followable := True;
               elsif Has_Thing (X.S, T) then
                  declare
                     Now_T : constant Thing_State := Thing (X.S, T);
                  begin
                     F.Seen := Now_T.Seen;
                     F.Followable := Known (Now_T.Centre);
                     F.Height_Gain := (Value => Real'((Now_T.Centre.Mean - Start.Mean) * Up0),
                                       Sigma => Sqrt (2.0) * Largest_Sigma (Start.Covariance), Degrees_Of_Freedom => 0);
                     if F.Commanded and then G.Closing then
                        --  Where it was, carried by the hand's measured motion:
                        --  the two hand readings and the lever of their turn add
                        --  their uncertainty to that of where it was.
                        declare
                           Lever : constant Real := abs (Centre.Mean - Now.Tool.Pose.Translation);
                           Turns : constant Real := Largest_Sigma (Now.Tool.Rotation_Covariance) ** 2
                             + Largest_Sigma (Arm (X.S, G.Arm).Tool.Rotation_Covariance) ** 2;
                        begin
                           F.Carried_To := (Mean       => Carried * Centre.Mean,
                                            Covariance => Centre.Covariance + Now.Tool.Position_Covariance
                                              + Arm (X.S, G.Arm).Tool.Position_Covariance
                                              + (Lever ** 2 * Turns) * Identity3);
                        end;
                        F.Carried_At := Now_T.Centre;
                     end if;
                  end;
               else
                  F.Seen := False;
                  F.Followable := False;
               end if;
               if G.Closing and then not G.Hands.Is_Empty then
                  declare
                     Fr : constant Estimate := Hand (X.S, G.Hands.First_Element.Hand).Fraction;
                  begin
                     F.Closed_Short := (Value => 1.0 - Fr.Value, Sigma => Fr.Sigma,
                                        Degrees_Of_Freedom => Fr.Degrees_Of_Freedom);
                  end;
               end if;
               F.Still := X.S.Still;
               --  The gap as it is after the step, against what the step owed.
               declare
                  After : constant Goals.Answer := Next_Goal (X.S);
               begin
                  F.Gap := (if After.Ok and then not Pushing then After.Gap else Unknown);
               end;
               F.Owed := Delivered;
               F.Out_Of_Beats := P.Episode_Over;
            end;
            Monitor.Step (Watch, F);
            if Monitor.Fired (Watch, F, Wanted, Max_Steps) then
               Final := Monitor.Ending_Of (Watch, F, Wanted, Max_Steps);
               declare
                  Now_Centre : constant Vec3 :=
                    (if T = 0 then Effector_Item (X.S, G.Arm).Centre.Mean
                     elsif Has_Thing (X.S, T) then Thing (X.S, T).Centre.Mean else Start.Mean);
                  Moved_By   : constant Vec3 := Now_Centre - Start.Mean;
               begin
                  Say (X, (if T = 0 then "my hand" else "it") & " moved by " & Img (abs Moved_By) & ", "
                       & Img (Real'(Moved_By * Up0)) & " of it up, in " & Img (Monitor.Steps (Watch)) & " steps");
               end;
               if F.Exhausted and then Final in Stuck | Settled then
                  Say (X, "I could go no further: a step of the smallest size would leave my reach or my view");
               end if;
               if not Goal.Ok then
                  Say (X, To_String (Goal.Why));
               end if;
               return;
            end if;
         end;
      end loop;
   end Carry;

   --  A thing brought down onto a surface is let go when it rests there:
   --  the closers open, the hand backs out the way it came, and a look
   --  confirms that it stayed.
   procedure Put_Down (P : in out Plant'Class; X : in out State; T : Thing_Id; G : Grip) is
      Thing_Now : constant Thing_State := Thing (X.S, T);
      Rest : constant Wrench.Rest_Answer :=
        Wrench.Rests (Base_Over_Support (X.S, T), Thing_Now.Centre, Goals.Up_Of (X.S, T), Thing_Now.Friction.Low);
      Out_Come : Step_Outcome;
      Why : Unbounded_String;
   begin
      if not G.Closing then
         return;
      elsif not Rest.Rests then
         Say (X, "I keep holding it: it would not rest there, its centre is not over its footing by "
              & Img (Rest.Margin));
         return;
      end if;
      for C of G.Hands loop
         Set_Closer (P, X, C.Hand, 0.0, Out_Come);
      end loop;
      declare
         E    : constant Search.Effector := Search.Effector_Of (X.S, G.Arm);
         Back : constant Rigid :=
           (Rotation => E.Tool.Rotation, Translation => E.Tool.Translation - E.Depth * Rotate (E.Tool, E.Along));
      begin
         Go (P, X, G.Arm, Back, Out_Come, Why);
      end;
      if Has_Thing (X.S, T) and then Thing (X.S, T).Held_By = 0
        and then not Significant (Thing_Now.Centre, Thing (X.S, T).Centre)
      then
         Say (X, "I let go of it and it stayed where I put it");
      elsif Has_Thing (X.S, T) then
         Say (X, "I let go of it and it moved by " & Img (abs (Thing (X.S, T).Centre.Mean - Thing_Now.Centre.Mean)));
      end if;
   end Put_Down;

   --  The quantities this body can change now, in the keyboard's order.
   function Quantity_Of (S : Snapshot; Index : Positive; Q : out Goals.Quantity) return Boolean is
      Can  : constant Goals.Quantity_Set := Goals.Changeable (S);
      Seen : Natural := 0;
   begin
      Q := Goals.Quantity'First;
      for K in Goals.Quantity loop
         if Can (K) then
            Seen := Seen + 1;
            if Seen = Index then
               Q := K;
               return True;
            end if;
         end if;
      end loop;
      return False;
   end Quantity_Of;

   --  Takes hold of the thing for Motion unless a hand holds it already.
   procedure Hold_It (P : in out Plant'Class; X : in out State; T : Thing_Id; Motion : Contact.Twist; G : out Grip;
                      Ok : out Boolean)
   is
   begin
      if Thing (X.S, T).Held_By /= 0 and then Has_Hand (X.S, Thing (X.S, T).Held_By) then
         G := Grip_Of_Arm (X.S, Hand (X.S, Thing (X.S, T).Held_By).Arm, Closing => True);
         Say (X, "I hold it already");
         Ok := True;
      else
         Acquire (P, X, T, Motion, G, Ok);
         if not Ok then
            Note_Tried (X, "no part of me could take it that way");
         end if;
      end if;
   end Hold_It;

   --  A grip that let the thing slip needed more friction than it has.
   procedure Learn_From_Slip (P : in out Plant'Class; X : in out State; T : Thing_Id; G : Grip) is
      B : Friction_Bounds := Thing (X.S, T).Friction;
   begin
      if G.Closing and then G.Searched then
         B.High := Real'Min (B.High, G.Chosen.Mu_Nominal);
         P.Learn ((Kind => Friction_Learned, Thing => T, Bounds => B));
         Say (X, "it slipped out of a grip that needed friction " & Img (G.Chosen.Mu_Nominal)
              & ", so I take its friction to be less than that from now on");
      end if;
   end Learn_From_Slip;

   procedure Run_Change (P : in out Plant'Class; X : in out State; W : Want; Final : out Ending) is
      T : constant Thing_Id := W.Thing;
      Q : Goals.Quantity;
   begin
      Final := Refused;
      if not Has_Thing (X.S, T) then
         Note_Tried (X, "I looked for it among the " & Img (Natural (X.S.Things.Length))
                     & " things I measure now and it is not one of them");
         return;
      elsif not Quantity_Of (X.S, W.Quantity, Q) then
         Note_Tried (X, "this body cannot measure and change that quantity now");
         return;
      end if;
      declare
         First : constant Goals.Answer := Goals.Twist_Of (X.S, T, Q, W.Increase);
         Hold  : Grip;
         Ok    : Boolean;
         function Next_Goal (S : Snapshot) return Goals.Answer is
           (if Has_Thing (S, T) then Goals.Twist_Of (S, T, Q, W.Increase)
            else (Ok => False, Why => To_Unbounded_String ("I no longer see it"), others => <>));
      begin
         if not First.Ok then
            Note_Tried (X, To_String (First.Why));
            return;
         end if;
         Hold_It (P, X, T, First.Motion, Hold, Ok);
         if not Ok then
            return;
         end if;
         Carry (P, X, T, Hold, Next_Goal'Access, W.Until_Endings, W.Max_Steps, Final);
         if Final = Touched and then Q = Goals.Height and then not W.Increase then
            Put_Down (P, X, T, Hold);
         elsif Final = Slipped then
            Learn_From_Slip (P, X, T, Hold);
         end if;
      end;
   end Run_Change;

   function Pair_Of (R : Relation; Pair : out Goals.Pair_Relation) return Boolean is
   begin
      case R is
         when Above   => Pair := Goals.Above;
         when Below   => Pair := Goals.Below;
         when Left    => Pair := Goals.Left;
         when Right   => Pair := Goals.Right;
         when Nearer  => Pair := Goals.Nearer;
         when Farther => Pair := Goals.Farther;
         when Onto    => Pair := Goals.Onto;
         when Off     => Pair := Goals.Off;
         when Facing  => Pair := Goals.Facing;
         when others  =>
            Pair := Goals.Above;
            return False;
      end case;
      return True;
   end Pair_Of;

   --  Moves the subject (thing T, or arm G.Arm's own parts when T is 0)
   --  through the points of the plan over the object, planned once and again
   --  only when the object is measured to have moved; the last point of a
   --  plan that ends by touch is passed until the touch is felt.
   procedure Follow_Over (P : in out Plant'Class; X : in out State; T : Thing_Id'Base; G : Grip;
                          Subject_Of, Object_Of : not null access function (S : Snapshot) return Goals.Item;
                          Object_Pitch : Real; R : Goals.Pair_Relation; W : Want; Final : out Ending)
   is
      --  What the mover needs to pass over without meeting it: the reach of
      --  the contact test ahead, and the arm's own resolution on top.
      function Margin (S : Snapshot) return Real is
        (Object_Pitch / 2.0 + Z * Sqrt (Subject_Of (S).Sigma ** 2 + Object_Of (S).Sigma ** 2)
         + Arm (S, G.Arm).Step.Value);
      Route   : Goals.Plan := Goals.Over_Plan (X.S, Subject_Of (X.S), Object_Of (X.S), R, Margin (X.S));
      Planned : Point_Estimate := Object_Of (X.S).Centre;
      K       : Positive := 1;

      function Next (S : Snapshot) return Goals.Answer is
         Fine : constant Real := Arm (S, G.Arm).Step.Value;
         --  The subject is at a point of the route as far as it can tell:
         --  within the arm's smallest step of it, or not significantly away
         --  given the subject's own uncertainty (a thing seen by an eye is
         --  never found at the very point the arm took it to).
         function Is_At (Point : Vec3) return Boolean is
           (abs (Point - Subject_Of (S).Centre.Mean) <= Fine
            or else not Significant (Subject_Of (S).Centre,
                                     Point_Estimate'(Mean => Point, Covariance => [others => [others => 0.0]])));
      begin
         if Significant (Planned, Object_Of (S).Centre) then
            Route := Goals.Over_Plan (S, Subject_Of (S), Object_Of (S), R, Margin (S));
            Planned := Object_Of (S).Centre;
            K := 1;
         end if;
         if not Route.Ok then
            return (Ok => False, Why => Route.Why, others => <>);
         end if;
         declare
            C : constant Vec3 := Subject_Of (S).Centre.Mean;
         begin
            while K < Route.Count and then Is_At (Route.Points (K)) loop
               K := K + 1;
            end loop;
            declare
               D    : constant Vec3 := Route.Points (K) - C;
               Last : constant Vec3 := (if K > 1 then Route.Points (K) - Route.Points (K - 1) else D);
               Rest : Real := abs D;
            begin
               for J in K + 1 .. Route.Count loop
                  Rest := Rest + abs (Route.Points (J) - Route.Points (J - 1));
               end loop;
               if K = Route.Count and then Is_At (Route.Points (K)) then
                  return (Ok => True, Done => True, Leg => Real'Last, Why => Null_Unbounded_String,
                          Gap => (Value => 0.0, Sigma => Subject_Of (S).Sigma, Degrees_Of_Freedom => 0),
                          Motion => (if abs Last > 0.0 then Contact.Slide (Unit (Last)) else Contact.Still (C)));
               end if;
               return (Ok => True, Done => False, Leg => abs D, Why => Null_Unbounded_String,
                       Gap => (Value => Rest, Sigma => Subject_Of (S).Sigma, Degrees_Of_Freedom => 0),
                       Motion => Contact.Slide (Unit (D)));
            end;
         end;
      end Next;
   begin
      if not Route.Ok then
         Final := Refused;
         Note_Tried (X, To_String (Route.Why));
         return;
      end if;
      Carry (P, X, T, G, Next'Access, W.Until_Endings, W.Max_Steps, Final, Arrive_By_Touch => Route.By_Touch);
   end Follow_Over;

   --  Watches the scene beat by beat, moving nothing, until an ending.
   procedure Wait (P : in out Plant'Class; X : in out State; W : Want; Final : out Ending) is
      Watch : Monitor.Watch := Monitor.Start;
      R     : Report;
   begin
      loop
         P.Move ((Arms => Arm_Goal_Vectors.Empty_Vector, Closers => Closer_Goal_Vectors.Empty_Vector,
                  Settle => False), R);
         Look (P, X);
         declare
            F : constant Monitor.Facts := (Still => X.S.Still, Out_Of_Beats => P.Episode_Over, others => <>);
         begin
            Monitor.Step (Watch, F);
            if Monitor.Fired (Watch, F, W.Until_Endings, W.Max_Steps) then
               Final := Monitor.Ending_Of (Watch, F, W.Until_Endings, W.Max_Steps);
               Say (X, "I kept still for " & Img (Monitor.Steps (Watch)) & " beats");
               return;
            end if;
         end;
      end loop;
   end Wait;

   procedure Run_Interval (P : in out Plant'Class; X : in out State; W : Want; Final : out Ending) is
      C : Constraint;
   begin
      Final := Refused;
      if W.Constraints.Is_Empty then
         Note_Tried (X, "the interval asks for nothing");
         return;
      end if;
      C := W.Constraints.First_Element;
      for K of W.Constraints loop
         if K.Must then
            C := K;
            exit;
         end if;
      end loop;
      if C.Subject.Kind = Role_Operand and then C.Relation in Still | Clear then
         Wait (P, X, W, Final);
         return;
      end if;
      declare
         Has_Object : constant Boolean :=
           (case C.Object.Kind is
               when Thing_Operand => Has_Thing (X.S, C.Object.Thing),
               when Place_Operand => Has_Place (X.S, C.Object.Place),
               when others        => False);
         Near : constant Vec3 :=
           (case C.Object.Kind is
               when Thing_Operand => (if Has_Object then Thing (X.S, C.Object.Thing).Centre.Mean else Zero3),
               when Place_Operand => (if Has_Object then Place (X.S, C.Object.Place).Point.Mean else Zero3),
               when others        => Zero3);
         A    : Arm_Id;
         Pair : Goals.Pair_Relation;
         function Object_Item (S : Snapshot) return Goals.Item is
           (case C.Object.Kind is
               when Thing_Operand => Goals.Item_Of (S, C.Object.Thing),
               when Place_Operand => Goals.Point_Item (Place (S, C.Object.Place).Point),
               when others        => Goals.Point_Item ((others => <>)));
         function Object_Pitch (S : Snapshot) return Real is
           (if C.Object.Kind = Thing_Operand and then Has_Thing (S, C.Object.Thing)
            then Thing (S, C.Object.Thing).Pitch else 0.0);
      begin
         if C.Object.Kind in Thing_Operand | Place_Operand and then not Has_Object then
            Note_Tried (X, "I do not measure what it is to be " & Relation'Image (C.Relation) & " now");
            return;
         end if;
         case C.Subject.Kind is
            when Role_Operand =>
               if not Bound (X.S, C.Subject.The_Role, Near, Has_Object, A) then
                  Note_Tried (X, "I have no part that can be " & Role_Word (C.Subject.The_Role) & " now");
                  return;
               end if;
               case C.Relation is
                  when Close =>
                     if C.Object.Kind = Thing_Operand then
                        declare
                           Up : constant Goals.Answer := Goals.Twist_Of (X.S, C.Object.Thing, Goals.Height, True);
                           G  : Grip;
                           Ok : Boolean;
                        begin
                           Acquire (P, X, C.Object.Thing, Up.Motion, G, Ok, Only_Arm => A);
                           if not Ok then
                              Note_Tried (X, "no way to close on it");
                              return;
                           end if;
                           Final := (if W.Until_Endings (Stuck) then Stuck
                                     elsif W.Until_Endings (Touched) then Touched else Settled);
                        end;
                     else
                        declare
                           Out_Come : Step_Outcome;
                        begin
                           for H of Search.Effector_Of (X.S, A).Closers loop
                              Set_Closer (P, X, H.Hand, 1.0, Out_Come);
                           end loop;
                           Final := (if Out_Come = Blocked then Stuck else Settled);
                        end;
                     end if;
                     return;
                  when Open =>
                     declare
                        Out_Come : Step_Outcome;
                     begin
                        for H of Search.Effector_Of (X.S, A).Closers loop
                           Set_Closer (P, X, H.Hand, 0.0, Out_Come);
                        end loop;
                        Say (X, "I opened arm " & Img (Integer (A)) & "'s lobes");
                     end;
                     Wait (P, X, W, Final);
                     return;
                  when Touching | Press =>
                     if C.Object.Kind = Thing_Operand then
                        declare
                           G  : Grip;
                           Ok : Boolean;
                        begin
                           Acquire (P, X, C.Object.Thing, Contact.Still (Near), G, Ok, Only_Arm => A,
                                    Touch_Only => True);
                           if not Ok then
                              Note_Tried (X, "no part of arm " & Img (Integer (A)) & " can touch it");
                              return;
                           end if;
                           declare
                              --  In along the last straight stretch, past where the
                              --  touch should be if need be, until it is felt.
                              In_Way : constant Vec3 := G.Chosen.Tool.Translation - G.Chosen.Hover.Translation;
                              function Toward_Touch (Unused : Snapshot) return Goals.Answer is
                                (Ok => True, Done => False, Why => Null_Unbounded_String, Leg => Real'Last,
                                 Motion => Contact.Slide (Unit (In_Way)), Gap => Unknown);
                           begin
                              Carry (P, X, 0, Grip_Of_Arm (X.S, A, Closing => False), Toward_Touch'Access,
                                     W.Until_Endings, W.Max_Steps, Final, Arrive_By_Touch => True,
                                     Felt => C.Object.Thing);
                           end;
                        end;
                     else
                        declare
                           function Toward_Place (S : Snapshot) return Goals.Answer is
                              Here : constant Goals.Item := Effector_Item (S, A);
                              D    : constant Vec3 := Object_Item (S).Centre.Mean - Here.Centre.Mean;
                              Done : constant Boolean := not Significant (Here.Centre, Object_Item (S).Centre);
                           begin
                              return (Ok => True, Done => Done, Why => Null_Unbounded_String, Leg => Real'Last,
                                      Motion => Contact.Slide (if abs D > 0.0 then Unit (D) else Gravity (S)),
                                      Gap => (Value => abs D, Sigma => Here.Sigma, Degrees_Of_Freedom => 0));
                           end Toward_Place;
                        begin
                           Carry (P, X, 0, Grip_Of_Arm (X.S, A, Closing => False), Toward_Place'Access,
                                  W.Until_Endings, W.Max_Steps, Final);
                        end;
                     end if;
                     return;
                  when others =>
                     if not Pair_Of (C.Relation, Pair) then
                        Note_Tried (X, "I cannot do " & Relation'Image (C.Relation) & " with a part of me yet");
                        return;
                     elsif C.Object.Kind not in Thing_Operand | Place_Operand then
                        Note_Tried (X, "it needs something to be " & Relation'Image (C.Relation) & " of");
                        return;
                     end if;
                     declare
                        function Steer (S : Snapshot) return Goals.Answer is
                          (Goals.Toward (S, Effector_Item (S, A), Object_Item (S), Pair,
                                         Rotate (Arm (S, A).Tool.Pose, Search.Effector_Of (S, A).Along),
                                         Largest_Sigma (Arm (S, A).Tool.Rotation_Covariance)));
                        function Mine (S : Snapshot) return Goals.Item is (Effector_Item (S, A));
                     begin
                        if Pair in Goals.Above | Goals.Below | Goals.Onto then
                           Follow_Over (P, X, 0, Grip_Of_Arm (X.S, A, Closing => False), Mine'Access,
                                        Object_Item'Access, Object_Pitch (X.S), Pair, W, Final);
                        else
                           Carry (P, X, 0, Grip_Of_Arm (X.S, A, Closing => False), Steer'Access, W.Until_Endings,
                                  W.Max_Steps, Final);
                        end if;
                     end;
                     return;
               end case;
            when Thing_Operand =>
               declare
                  T : constant Thing_Id := C.Subject.Thing;
               begin
                  if not Has_Thing (X.S, T) then
                     Note_Tried (X, "I do not see the thing that is to move");
                     return;
                  elsif not Pair_Of (C.Relation, Pair) or else C.Object.Kind not in Thing_Operand | Place_Operand then
                     Note_Tried (X, "a thing can be moved " & Relation'Image (C.Relation)
                                 & " only of another thing or a place");
                     return;
                  end if;
                  declare
                     function Steer (S : Snapshot) return Goals.Answer is
                        Axis_Sigma : Real;
                        L : constant Vec3 :=
                          (if Has_Thing (S, T) then Goals.Long_Axis (S, T, Axis_Sigma) else Zero3);
                     begin
                        if not Has_Thing (S, T) then
                           return (Ok => False, Why => To_Unbounded_String ("I no longer see it"), others => <>);
                        end if;
                        return Goals.Toward (S, Goals.Item_Of (S, T), Object_Item (S), Pair, L,
                                             (if abs L > 0.0 then Axis_Sigma else Real'Last));
                     end Steer;
                     First : constant Goals.Answer := Steer (X.S);
                     G     : Grip;
                     Ok    : Boolean;
                  begin
                     if not First.Ok then
                        Note_Tried (X, To_String (First.Why));
                        return;
                     elsif First.Done and then not (Pair = Goals.Onto and then W.Until_Endings (Touched)) then
                        Say (X, To_String (First.Why));
                     end if;
                     Hold_It (P, X, T, First.Motion, G, Ok);
                     if not Ok then
                        return;
                     end if;
                     if Pair in Goals.Above | Goals.Below | Goals.Onto then
                        declare
                           function Held_Thing (S : Snapshot) return Goals.Item is (Goals.Item_Of (S, T));
                        begin
                           Follow_Over (P, X, T, G, Held_Thing'Access, Object_Item'Access, Object_Pitch (X.S), Pair,
                                        W, Final);
                        end;
                     else
                        Carry (P, X, T, G, Steer'Access, W.Until_Endings, W.Max_Steps, Final);
                     end if;
                     if Final = Touched and then Pair = Goals.Onto then
                        Put_Down (P, X, T, G);
                     elsif Final = Slipped then
                        Learn_From_Slip (P, X, T, G);
                     end if;
                  end;
               end;
            when others =>
               Note_Tried (X, "only a part of me or a thing can be made to move");
         end case;
      end;
   end Run_Interval;

   procedure Execute (P : in out Driver.Action.Plants.Plant'Class; W : Want; R : out Result) is
      X : State;
   begin
      R := (Final => Refused, Tried => Null_Unbounded_String, Account => Null_Unbounded_String);
      Look (P, X);
      case W.Kind is
         when Change =>
            Run_Change (P, X, W, R.Final);
         when Interval =>
            Run_Interval (P, X, W, R.Final);
      end case;
      R.Account := X.Account;
      R.Tried := X.Tried;
   end Execute;

end Driver.Action.Execution;
