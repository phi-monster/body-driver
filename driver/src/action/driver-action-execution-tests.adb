with Ada.Numerics;
with Ada.Text_IO;
with Driver.Action.Goals;
with Driver.Action.Plants;
with Driver.Action.Plants.Tests;
with Driver.Action.Snapshots;
with Driver.Action.Snapshots.Tests;
with Driver.Clock;
with Driver.Log;
with Driver.Numerics;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Action.Execution.Tests is

   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use Driver.Action.Snapshots;
   use Driver.Action.Snapshots.Tests;
   use type Driver.Action.Snapshots.Hand_Id;
   use type Driver.Action.Snapshots.Arm_Id;

   package Plants renames Driver.Action.Plants;
   package Sim renames Driver.Action.Plants.Tests;

   Pi : constant := Ada.Numerics.Pi;

   Sigma : constant Real := 0.0005;
   Pitch : constant Real := 0.005;

   Upright : constant Rigid := Identity;
   Turned  : constant Rigid := (Rotation => Exp ([0.9, -1.3, 2.2]), Translation => [2.0, 1.0, -0.5]);

   type Rigid_Array is array (Positive range <>) of Rigid;
   Both_Frames : constant Rigid_Array := [Upright, Turned];

   --  Tool z pointing down at the table.
   function Down_At (X, Y, Z : Real) return Rigid is ((Rotation => Exp ([Pi, 0.0, 0.0]), Translation => [X, Y, Z]));

   function On_Table (X, Y, About : Real) return Rigid is
     ((Rotation => Exp ([0.0, 0.0, About]), Translation => [X, Y, 0.0]));

   --  One arm with a two-lobe gripper above the table, late by two beats and
   --  delivering 70 to 85 percent of every step.
   procedure One_Gripper (W : in out Sim.World; Place : Rigid; Seed : Integer) is
   begin
      Sim.Start (W, Place, Sigma, Pitch, Seed);
      Sim.Add_Arm (W, Base => [0.0, -0.3, 0.0], Reach => 0.8, Tool => Down_At (0.0, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0);
      Sim.Add_Gripper (W, 1, Opening => 0.08, Width => 0.015, Thickness => 0.01, Depth => 0.04);
   end One_Gripper;

   function Endings (E : Ending) return Ending_Set is
      S : Ending_Set := [others => False];
   begin
      S (E) := True;
      return S;
   end Endings;

   function Height_Want (T : Thing_Id; Up : Boolean; Until_Ending : Ending; Steps : Natural := 0) return Want is
     ((Kind => Change, Until_Endings => Endings (Until_Ending), Max_Steps => Steps, Eye => Any_Eye, Anyway => False,
       Thing => T, Quantity => 1, Increase => Up));

   procedure Run (W : in out Sim.World; Wanted : Want; R : out Result) is
      Start_Beat : constant Natural := W.Beat;
      Start_Time : constant Duration := Driver.Clock.Seconds;
   begin
      Execute (W, Wanted, R);
      Ada.Text_IO.Put_Line ("      " & Ending'Image (R.Final) & " after " & Driver.Log.Image (W.Beat - Start_Beat)
                            & " beats, " & Driver.Log.Image (Real (Driver.Clock.Seconds - Start_Time), 2) & " s: "
                            & To_String (R.Account) & (if Length (R.Tried) > 0 then " | tried: " & To_String (R.Tried)
                                                       else ""));
   end Run;

   procedure Bar_Up_And_Down is
   begin
      for Place of Both_Frames loop
         declare
            W : Sim.World;
            R : Result;
         begin
            One_Gripper (W, Place, 7);
            Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.3), Mu => 0.6);
            Run (W, Height_Want (1, True, Free), R);
            Check (R.Final = Free, "a bar lifted until free does not end free");
            Check (Sim.Lowest (W, 1) > 0.0, "a bar said to be free still lies on the table");
            Check (Sim.Truth (W, 1).Held_By = 1, "a lifted bar is not in the hand");
            Run (W, Height_Want (1, False, Touched), R);
            Check (R.Final = Touched, "a bar put down does not end touched");
            Check (abs Sim.Lowest (W, 1) < Pitch, "a bar put down is not on the table");
            Check (Sim.Truth (W, 1).Held_By = 0, "a bar put down on the table is not let go");
         end;
      end loop;
   end Bar_Up_And_Down;

   function Thing_Of (T : Thing_Id) return Operand is ((Kind => Thing_Operand, Thing => T));
   function Role_Of (R : Role) return Operand is ((Kind => Role_Operand, The_Role => R));

   function Interval_Want (Subject : Operand; R : Relation; Object : Operand; Until_Ending : Ending;
                           Steps : Natural := 0) return Want
   is
      W : Want (Interval);
   begin
      W.Until_Endings := Endings (Until_Ending);
      W.Max_Steps := Steps;
      W.Constraints.Append (Constraint'(Subject => Subject, Relation => R, Object => Object, Step => Unspecified,
                             Strength => Unspecified, Must => False));
      return W;
   end Interval_Want;

   type Model_Array is array (Positive range <>) of Model;

   --  The five shapes of the weld matrix, each with where it lies.
   Shapes : constant Model_Array :=
     [Bar (0.2, 0.02, 0.02), Block (0.04, 0.04, 0.04), Upright_Cylinder (0.025, 0.08), Scissors (0.18, 0.016, 0.006),
      Cup (0.03, 0.004, 0.08)];

   Names : constant array (Shapes'Range) of access constant String :=
     [new String'("bar"), new String'("block"), new String'("cylinder"), new String'("scissors"), new String'("cup")];

   procedure Each_Shape_Up is
   begin
      for K in Shapes'Range loop
         declare
            W : Sim.World;
            R : Result;
         begin
            One_Gripper (W, Turned, 11 + K);
            Sim.Add_Thing (W, Shapes (K), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
            Ada.Text_IO.Put_Line ("      " & Names (K).all & ":");
            Run (W, Height_Want (1, True, Free), R);
            Check (R.Final = Free, "the " & Names (K).all & " lifted until free does not end free");
            Check (Sim.Lowest (W, 1) > 0.0 and then Sim.Truth (W, 1).Held_By = 1,
                   "the " & Names (K).all & " said to be free is not up in the hand");
            Run (W, Height_Want (1, True, Settled), R);
            Check (R.Final = Settled and then Sim.Lowest (W, 1) > 0.05,
                   "the " & Names (K).all & " lifted until settled does not go well up");
         end;
      end loop;
   end Each_Shape_Up;

   procedure Each_Shape_Onto_A_Block is
   begin
      for K in Shapes'Range loop
         declare
            W : Sim.World;
            R : Result;
         begin
            One_Gripper (W, Turned, 23 + K);
            Sim.Add_Thing (W, Shapes (K), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
            Sim.Add_Thing (W, Block (0.08, 0.08, 0.05), On_Table (-0.12, 0.08, 0.2), Mu => 0.6);
            Ada.Text_IO.Put_Line ("      " & Names (K).all & ":");
            Run (W, Interval_Want (Thing_Of (1), Onto, Thing_Of (2), Touched), R);
            Check (R.Final = Touched, "the " & Names (K).all & " put onto the block does not end touched");
            Check (Sim.Rests_On (W, 1, 2), "the " & Names (K).all & " does not rest on the block");
            Check (Sim.Truth (W, 1).Held_By = 0, "the " & Names (K).all & " put onto the block is not let go");
         end;
      end loop;
   end Each_Shape_Onto_A_Block;

   procedure Touch_A_Bar is
      W : Sim.World;
      R : Result;
   begin
      One_Gripper (W, Turned, 41);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
      declare
         Before : constant Rigid := Sim.Truth (W, 1).Pose;
      begin
         Run (W, Interval_Want (Role_Of (Grasper), Touching, Thing_Of (1), Touched, 40), R);
         Check (R.Final = Touched, "touching a bar does not end touched");
         Check (abs (Sim.Truth (W, 1).Pose.Translation - Before.Translation) < 2.0 * Pitch,
                "touching a bar shoves it along");
      end;
   end Touch_A_Bar;

   procedure Over_An_Obstacle is
      W : Sim.World;
      R : Result;
   begin
      One_Gripper (W, Turned, 43);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.12, 0.4), Mu => 0.6);
      --  A wall that cannot give way, between the hand and the bar.
      Sim.Add_Thing (W, Block (0.3, 0.02, 0.2), On_Table (0.05, 0.02, 0.0), Mu => 0.6, Fixed => True);
      Run (W, Height_Want (1, True, Free), R);
      Check (R.Final = Free, "a bar behind a wall is not lifted");
      Check (Sim.Truth (W, 2).Pose.Translation = Sim.Table_Frame (W) * [0.05, 0.02, 0.0],
             "the wall was moved");
   end Over_An_Obstacle;

   --  A hand that starts at the top of what its arm can reach cannot rise any
   --  further; it goes over the wall at the height it has, across and down.
   procedure A_Hand_At_The_Top_Of_Its_Reach_Goes_Over is
      W : Sim.World;
      R : Result;
   begin
      Sim.Start (W, Turned, Sigma, Pitch, 89);
      Sim.Add_Arm (W, Base => [0.05, 0.0, -0.1], Reach => 0.38, Tool => Down_At (0.0, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0);
      Sim.Add_Gripper (W, 1, Opening => 0.08, Width => 0.015, Thickness => 0.01, Depth => 0.04);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.12, 0.4), Mu => 0.6);
      Sim.Add_Thing (W, Block (0.3, 0.02, 0.2), On_Table (0.05, 0.02, 0.0), Mu => 0.6, Fixed => True);
      Run (W, Height_Want (1, True, Free), R);
      Check (R.Final = Free, "a bar behind a wall is not lifted by a hand that cannot rise above where it starts");
      Check (Sim.Truth (W, 2).Pose.Translation = Sim.Table_Frame (W) * [0.05, 0.02, 0.0], "the wall was moved");
   end A_Hand_At_The_Top_Of_Its_Reach_Goes_Over;

   --  How far the thing has turned about the table's up since Before.
   function Turned_About_Up (W : Sim.World; T : Thing_Id; Before : Rigid) return Real is
      Up  : constant Vec3 := Rotate (Sim.Table_Frame (W), [0.0, 0.0, 1.0]);
      Rot : constant Vec3 := Driver.Numerics.Log (Sim.Truth (W, T).Pose.Rotation * Transpose (Before.Rotation));
   begin
      return Rot * Up;
   end Turned_About_Up;

   procedure Long_Shapes_Turned is
      Long : constant Model_Array := [Bar (0.2, 0.02, 0.02), Scissors (0.18, 0.016, 0.006)];
   begin
      for K in Long'Range loop
         declare
            W : Sim.World;
            R : Result;
            Before : Rigid;
         begin
            One_Gripper (W, Turned, 71 + K);
            Sim.Add_Thing (W, Long (K), On_Table (0.05, 0.05, 0.4), Mu => 0.6);
            Before := Sim.Truth (W, 1).Pose;
            Run (W, (Kind => Change, Until_Endings => Endings (Timeout), Max_Steps => 3, Eye => Any_Eye,
                     Anyway => False, Thing => 1, Quantity => 2, Increase => True), R);
            Check (R.Final = Timeout, "a heading turned for three steps does not end by its step limit");
            Check (Turned_About_Up (W, 1, Before) > 0.0, "heading up does not turn it counter-clockwise about up");
            Check (abs Sim.Lowest (W, 1) < Pitch, "turning its heading took it off the table");
         end;
      end loop;
   end Long_Shapes_Turned;

   --  An arm whose joints are read far more finely than anything an eye sees:
   --  the fitted arms of the measured bodies are read a thousand times finer
   --  than their eyes place a thing, so its smallest step is a tiny share of
   --  the uncertainty of where a thing is.
   Finer : constant := 1000;
   Reading : constant Real := Sigma / Real (Finer);

   procedure Fine_Gripper (W : in out Sim.World; Place : Rigid; Seed : Integer) is
   begin
      Sim.Start (W, Place, Sigma, Pitch, Seed);
      Sim.Add_Arm (W, Base => [0.0, -0.3, 0.0], Reach => 0.8, Tool => Down_At (0.0, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0,
                   Reading_Sigma => Reading);
      Sim.Add_Gripper (W, 1, Opening => 0.08, Width => 0.015, Thickness => 0.01, Depth => 0.04);
   end Fine_Gripper;

   --  Where the thing is known to Sigma and the table to as much, a contact is
   --  somewhere in a band several sigmas wide ahead of a thing brought down;
   --  found by steps of the arm's smallest size, it takes as many of them as
   --  the arm is finer than that band is wide, a beat or more each.
   procedure A_Fine_Arm_Puts_Down is
      W : Sim.World;
      R : Result;
   begin
      Fine_Gripper (W, Turned, 81);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.3), Mu => 0.6);
      Run (W, Height_Want (1, True, Free), R);
      Check (R.Final = Free, "a bar is not lifted by an arm finer than the eyes");
      W.Last_Beat := W.Beat + Finer;
      Run (W, Height_Want (1, False, Touched), R);
      Check (R.Final = Touched, "a bar is not found to touch the table within as many beats as the arm is finer than the eyes");
      Check (abs Sim.Lowest (W, 1) < Pitch and then Sim.Truth (W, 1).Held_By = 0,
             "a bar put down by an arm finer than the eyes is not on the table, let go");
   end A_Fine_Arm_Puts_Down;

   --  The route over a block is up, across and down; the thing is never found
   --  at the very point the arm took it to, only near it, as near as an eye
   --  sees, so a point is reached when the thing is not significantly away.
   procedure A_Fine_Arm_Goes_Over_And_Down is
      W : Sim.World;
      R : Result;
   begin
      Fine_Gripper (W, Turned, 83);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
      Sim.Add_Thing (W, Block (0.08, 0.08, 0.05), On_Table (-0.12, 0.08, 0.2), Mu => 0.6);
      W.Last_Beat := Finer;
      Run (W, Interval_Want (Thing_Of (1), Onto, Thing_Of (2), Touched), R);
      Check (R.Final = Touched, "a bar is not put onto a block by an arm finer than the eyes");
      Check (Sim.Rests_On (W, 1, 2) and then Sim.Truth (W, 1).Held_By = 0,
             "a bar put onto a block by an arm finer than the eyes does not rest on it, let go");
   end A_Fine_Arm_Goes_Over_And_Down;

   --  An arm is the less sure of its pose the farther it has gone from where it
   --  was measured. The search finds where to take hold of a thing clear of
   --  everything by the margin the arm's uncertainty at the start asks; a way
   --  judged on the way by the larger uncertainty where the arm has come to
   --  would refuse the very pose it is going to.
   procedure A_Travel_Keeps_The_Margin_It_Was_Cleared_By is
      W : Sim.World;
      R : Result;
   begin
      Sim.Start (W, Turned, Sigma, Pitch, 91);
      Sim.Add_Arm (W, Base => [0.0, -0.3, 0.0], Reach => 0.8, Tool => Down_At (0.0, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0,
                   Reading_Sigma => Reading, Spread => 0.007);
      Sim.Add_Gripper (W, 1, Opening => 0.08, Width => 0.015, Thickness => 0.01, Depth => 0.04);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.12, 0.4), Mu => 0.6);
      --  A wall between the hand and the bar, so the way there is over the top, in legs, each judged anew.
      Sim.Add_Thing (W, Block (0.3, 0.02, 0.2), On_Table (0.05, 0.02, 0.0), Mu => 0.6, Fixed => True);
      Run (W, Height_Want (1, True, Free), R);
      Check (R.Final = Free, "a bar behind a wall is not lifted by an arm that is less sure of its pose where it has gone");
   end A_Travel_Keeps_The_Margin_It_Was_Cleared_By;

   --  A step of a heading goes as far as the arm can turn it, not the finest
   --  turn it can tell: round the thing's own axis nothing is ahead to meet,
   --  though the table is under it and the axis is square to the table within
   --  rounding, and the hand's straight path leaves the arc the thing follows
   --  by no more than the margin it keeps.
   procedure A_Step_Of_Heading_Is_A_Turn is
      W      : Sim.World;
      R      : Result;
      S      : Snapshot;
      Before : Rigid;
   begin
      Fine_Gripper (W, Turned, 85);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.05, 0.05, 0.4), Mu => 0.6);
      W.Look (S);
      Before := Sim.Truth (W, 1).Pose;
      Run (W, (Kind => Change, Until_Endings => Endings (Timeout), Max_Steps => 1, Eye => Any_Eye, Anyway => False,
               Thing => 1, Quantity => 2, Increase => True), R);
      Check (Turned_About_Up (W, 1, Before) > 100.0 * S.Arms (1).Turn_Step.Value,
             "a step of heading is not many times the smallest turn the arm can tell");
      Check (abs Sim.Lowest (W, 1) < Pitch, "turning its heading took it off the table");
   end A_Step_Of_Heading_Is_A_Turn;

   --  A body whose pushes that stopped short are all said to be blocked, as
   --  the live one's are, whether they moved some or none.
   type Stalling (W : not null access Sim.World) is limited new Plants.Plant with null record;

   overriding procedure Look (P : in out Stalling; S : out Snapshot);
   overriding procedure Within (P : in out Stalling; During : not null access procedure);
   overriding function Reach (P : Stalling; Goal : Plants.Arm_Goal) return Plants.Reach_Answer;
   overriding procedure Move (P : in out Stalling; O : Plants.Order; R : out Plants.Report);
   overriding function Predicted (P : Stalling; T : Thing_Id; Beats : Natural) return Driver.Uncertain.Point_Estimate;
   overriding procedure Learn (P : in out Stalling; L : Plants.Lesson);
   overriding function Episode_Over (P : Stalling) return Boolean;
   overriding function In_View (P : Stalling; Point : Vec3) return Boolean;

   procedure Look (P : in out Stalling; S : out Snapshot) is
   begin
      P.W.Look (S);
   end Look;

   procedure Within (P : in out Stalling; During : not null access procedure) is
   begin
      P.W.Within (During);
   end Within;

   function Reach (P : Stalling; Goal : Plants.Arm_Goal) return Plants.Reach_Answer is (P.W.Reach (Goal));

   procedure Move (P : in out Stalling; O : Plants.Order; R : out Plants.Report) is
      use type Plants.Step_Outcome;
   begin
      P.W.Move (O, R);
      for K in 1 .. Natural (R.Arms.Length) loop
         if R.Arms (K).Outcome = Plants.Short then
            R.Arms (K).Outcome := Plants.Blocked;
         end if;
      end loop;
   end Move;

   function Predicted (P : Stalling; T : Thing_Id; Beats : Natural) return Driver.Uncertain.Point_Estimate
   is (P.W.Predicted (T, Beats));

   procedure Learn (P : in out Stalling; L : Plants.Lesson) is
   begin
      P.W.Learn (L);
   end Learn;

   function Episode_Over (P : Stalling) return Boolean is (P.W.Episode_Over);

   function In_View (P : Stalling; Point : Vec3) return Boolean is (P.W.In_View (Point));

   --  An arm that carries out only part of every move, and says it was
   --  stopped, is not blocked for good: asked again from where it got to, it
   --  goes on, and only a push that moves nothing is a block.
   procedure A_Push_That_Stopped_Short_Is_Asked_Again is
      W : aliased Sim.World;
      P : Stalling (W'Access);
      R : Result;
   begin
      One_Gripper (W, Turned, 87);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
      Execute (P, Height_Want (1, True, Free), R);
      Ada.Text_IO.Put_Line ("      " & Ending'Image (R.Final) & ": " & To_String (R.Account)
                            & (if Length (R.Tried) > 0 then " | tried: " & To_String (R.Tried) else ""));
      Check (R.Final = Free, "a bar is not lifted by a body whose partly carried out moves are said to be blocked");
   end A_Push_That_Stopped_Short_Is_Asked_Again;

   procedure Usable_By_What_Is_Measured is
      W : Sim.World;
      S : Snapshot;
   begin
      One_Gripper (W, Turned, 51);
      Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.4), Mu => 0.6);
      W.Look (S);
      Check (Usable (S, Left) and then Usable (S, Nearer) and then Usable (S, Onto) and then Usable (S, Close)
             and then Usable (S, Touching), "a relation this body can measure and bring about is not offered");
      Check (not Usable (S, Into), "into is offered though it is not built");
      S.Eyes (1).On_Arm := 1;
      Check (not Usable (S, Left) and then not Usable (S, Farther), "left or farther is offered with no still eye");
      S.Up := (others => <>);
      Check (not Usable (S, Onto) and then not Usable (S, Above), "onto or above is offered with no gravity measured");
      S.Hands.Clear;
      Check (not Usable (S, Close) and then not Usable (S, Open), "close or open is offered with no grasper");
      S.Arms.Clear;
      Check (not Usable (S, Touching) and then not Usable (S, Still), "a relation is offered to a body with no arm");
   end Usable_By_What_Is_Measured;

   procedure Roles_Bind_To_What_Can_Play_Them is
      W : Sim.World;
      S : Snapshot;
      A : Arm_Id;
   begin
      One_Gripper (W, Turned, 53);
      Sim.Add_Arm (W, Base => [0.3, -0.3, 0.0], Reach => 0.8, Tool => Down_At (0.3, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0,
                   Plate_Radius => 0.02);
      W.Look (S);
      declare
         Has : constant Boolean := Bound_Arm (S, Grasper, A);
      begin
         Check (Has and then A = 1, "the grasper is not bound to the arm with the gripper");
      end;
      declare
         Has : constant Boolean := Bound_Arm (S, Pusher, A);
      begin
         Check (Has and then A = 2, "the pusher is not bound to the arm that ends in a plate");
      end;
      declare
         Has : constant Boolean := Bound_Arm (S, Me, A);
      begin
         Check (not Has and then A = Arm_Id'First, "me is bound though no arm carries the whole body");
      end;
      declare
         Middle : constant Vec3 := Part_Point (S, 1).Mean;
         Tool   : constant Rigid := Arm (S, 1).Tool.Pose;
         Inside : constant Vec3 := Transpose (Tool.Rotation) * (Middle - Tool.Translation);
      begin
         --  Between the lobes of the gripper: on its axis, within its depth.
         Check (abs Inside (1) < Sigma * 10.0 and then Inside (3) > 0.0 and then Inside (3) < 0.04,
                "the grasper's point is not between its lobes");
      end;
   end Roles_Bind_To_What_Can_Play_Them;

   procedure Every_Quantity_Has_A_Meaning is
   begin
      for Q in Goals.Quantity loop
         Check (Meaning (Goals.Word (Q))'Length > 0, "the quantity " & Goals.Word (Q) & " has no meaning to show");
      end loop;
      Check (Meaning ("weight") = "", "a word that is no quantity is given a meaning");
   end Every_Quantity_Has_A_Meaning;

   procedure Register is
   begin
      Register ("action.sheet.usable", "a relation is offered that this body cannot measure or bring about",
                Usable_By_What_Is_Measured'Access);
      Register ("action.sheet.meaning", "a quantity is offered without its meaning, or a non-quantity gets one",
                Every_Quantity_Has_A_Meaning'Access);
      Register ("action.sheet.roles", "a role binds to an arm that cannot play it, or its point is not on the part",
                Roles_Bind_To_What_Can_Play_Them'Access);
      Register ("action.run.bar", "a bar is not lifted off the table and put back, or not let go",
                Bar_Up_And_Down'Access);
      Register ("action.run.up", "one of the five shapes is not lifted, or not well up when settled",
                Each_Shape_Up'Access);
      Register ("action.run.onto", "one of the five shapes is not put onto a block to rest there",
                Each_Shape_Onto_A_Block'Access);
      Register ("action.run.turn", "a long thing's heading is not turned the way up turns it, on the table",
                Long_Shapes_Turned'Access);
      Register ("action.run.touch", "touching a thing does not stop at the touch", Touch_A_Bar'Access);
      Register ("action.run.detour", "the hand goes through a wall instead of over it", Over_An_Obstacle'Access);
      Register ("action.run.detour.top", "a hand that cannot rise above where it starts is refused a way over a wall",
                A_Hand_At_The_Top_Of_Its_Reach_Goes_Over'Access);
      Register ("action.run.margin", "a way is judged by the larger uncertainty where the arm has come to, and refuses "
                & "the pose the search cleared by the one at the start", A_Travel_Keeps_The_Margin_It_Was_Cleared_By'Access);
      Register ("action.run.fine.down", "an arm much finer than the eyes creeps to the table by its smallest steps",
                A_Fine_Arm_Puts_Down'Access);
      Register ("action.run.fine.over", "an arm much finer than the eyes stalls on the route over a block",
                A_Fine_Arm_Goes_Over_And_Down'Access);
      Register ("action.run.fine.heading", "a step of heading is only the finest turn the arm can tell",
                A_Step_Of_Heading_Is_A_Turn'Access);
      Register ("action.run.again", "a push that moved some and stopped short is taken for a block at once",
                A_Push_That_Stopped_Short_Is_Asked_Again'Access);
   end Register;

end Driver.Action.Execution.Tests;
