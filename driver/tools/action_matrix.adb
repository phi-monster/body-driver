--  action_matrix BODY_FILE UNITS_PER_METRE [CASE_FILTER] [OPTIONS]
--
--  The action layer's whole matrix of motions on a measured body, every arm
--  move through the real motion layer (Action_Rig): the five shapes lifted,
--  put down, slid along the table, pushed, turned, put onto another and
--  stacked; a door, an insertion and a twist; and the faults of a body (an
--  effect several beats late, a step delivered in part, a wall to go round, a
--  strike, a dodge). For every case it prints what the brain asked in the
--  language's words, the ending the layer gave, what it accounts for, and the
--  motion that was reached against the motion that was asked, from the
--  simulated world's truth.
--
--  The scene is put where the body measured its own table: the table is the
--  plane its arm's eye saw (Robot.Table_In_Arm), up is the body's up, the
--  first thing lies where the eye's line of sight meets the table at the
--  arm's reference pose, and the readings start at zero. The hand is the
--  simulated gripper of the self tests on the arm's tool frame (or, for a
--  pusher, the simulated plate); UNITS_PER_METRE scales the self tests'
--  metric scene into the body's own unit (the fit makes the root mean square
--  of its keyframes' eye positions one unit, so the scene's size has to come
--  from outside the body: the scorer's "scale m per unit" gives it, 67.2
--  units per metre for A17).
--
--  CASE_FILTER keeps the cases whose name contains it. OPTIONS is a string
--  of letters: v prints every arm move; t every reach the action layer asked
--  of the motion layer and every beat in which the world stopped the arm, with
--  how far the joints' own straight line bowed from the tool's; i runs an
--  ideal body whose arm carries the tool along the straight line between the
--  poses a beat's joint targets put it at, to tell what the action layer does
--  with a path it can rely on from what the motion layer's path does to it.

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Exceptions;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Maps.Constants;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Action_Rig;
with Driver.Action;
with Driver.Action.Execution;
with Driver.Action.Plants;
with Driver.Action.Plants.Tests;
with Driver.Action.Snapshots;
with Driver.Action.Snapshots.Tests;
with Driver.Beats;
with Driver.Commands;
with Driver.Geometry;
with Driver.Log;
with Driver.Numerics;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Stats;
with Driver.World;

procedure Action_Matrix is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Action;
   use type Driver.Action.Plants.Step_Outcome;
   use type Driver.Observations.Group_Id;
   use type Driver.Robot.Hand.Hand_Id;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   package Sim renames Driver.Action.Plants.Tests;
   package Shapes_Of renames Driver.Action.Snapshots.Tests;
   package Plants renames Driver.Action.Plants;

   use type Sim.Joint_Kind;

   Pi : constant := Ada.Numerics.Pi;

   function Img (X : Real; Digits_After : Positive := 3) return String is (Driver.Log.Image (X, Digits_After));
   function Img (N : Integer) return String is (Driver.Log.Image (N));

   function Lower (S : String) return String is
     (Ada.Strings.Fixed.Translate (S, Ada.Strings.Maps.Constants.Lower_Case_Map));

   procedure Say (S : String) is
   begin
      Ada.Text_IO.Put_Line (S);
      Ada.Text_IO.Flush;
   end Say;

   Path    : constant String := Ada.Command_Line.Argument (1);
   Scale   : constant Real := Real'Value (Ada.Command_Line.Argument (2));
   Filter  : constant String := (if Ada.Command_Line.Argument_Count >= 3 then Ada.Command_Line.Argument (3) else "");
   Options : constant String := (if Ada.Command_Line.Argument_Count >= 4 then Ada.Command_Line.Argument (4) else "");
   Verbose : constant Boolean := Ada.Strings.Fixed.Index (Options, "v") > 0;   --  every arm move
   Tracing : constant Boolean := Ada.Strings.Fixed.Index (Options, "t") > 0;   --  every reach asked, every beat stopped
   Ideal   : constant Boolean := Ada.Strings.Fixed.Index (Options, "i") > 0;   --  an arm that follows the tool's straight line

   --  The self tests' scene, in the body's unit.
   Pitch     : constant Real := 0.005 * Scale;
   Sigma     : constant Real := 0.0005 * Scale;
   Opening   : constant Real := 0.08 * Scale;
   Width     : constant Real := 0.015 * Scale;
   Thickness : constant Real := 0.01 * Scale;
   Depth     : constant Real := 0.04 * Scale;

   --  Where the still eye is, in the table's frame.
   Eye_At : constant Vec3 := [0.0, -0.6 * Scale, 0.4 * Scale];

   --  How far the simulated arm's wrist turns about the tool's axis in one grasp.
   Wrist_Range : constant Real := Pi;   --  either way from its middle: it turns twice that in all

   function Metres (Units : Real) return String is (Img (Units / Scale * 1000.0, 1) & " mm");

   type Shape_Kind is (Bar, Block, Cylinder, Scissors, Cup);
   type Hand_Kind is (Gripper, Plate);

   function Shape (K : Shape_Kind) return Shapes_Of.Model is
     (case K is
         when Bar      => Shapes_Of.Bar (0.2 * Scale, 0.02 * Scale, 0.02 * Scale),
         when Block    => Shapes_Of.Block (0.04 * Scale, 0.04 * Scale, 0.04 * Scale),
         when Cylinder => Shapes_Of.Upright_Cylinder (0.025 * Scale, 0.08 * Scale),
         when Scissors => Shapes_Of.Scissors (0.18 * Scale, 0.016 * Scale, 0.006 * Scale),
         when Cup      => Shapes_Of.Cup (0.03 * Scale, 0.004 * Scale, 0.08 * Scale));

   function Endings (E : Ending) return Ending_Set is
      S : Ending_Set := [others => False];
   begin
      S (E) := True;
      return S;
   end Endings;

   --  A want and the language's words for it.
   type Asked is record
      Want  : Driver.Action.Want;
      Words : Unbounded_String;
   end record;

   package Asked_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Asked);

   function Quantity_Word (Index : Positive) return String is
     (case Index is when 1 => "height", when 2 => "heading", when others => "tilt");

   function Steps_Words (Steps : Natural) return String is (if Steps > 0 then " or" & Steps'Image & " steps" else "");

   function Change_Of (T : Positive; Index : Positive; Up : Boolean; Until_Ending : Ending; Steps : Natural := 0)
     return Asked
   is
      W : constant Want :=
        (Kind => Change, Until_Endings => Endings (Until_Ending), Max_Steps => Steps, Eye => Any_Eye,
         Anyway => False, Thing => Driver.World.Thing_Id (T), Quantity => Index, Increase => Up);
   begin
      return (Want => W,
              Words => To_Unbounded_String
                ("do thing" & T'Image & " " & Quantity_Word (Index) & (if Up then " up" else " down") & " until "
                 & Lower (Ending'Image (Until_Ending)) & Steps_Words (Steps)));
   end Change_Of;

   function Interval_Of (Subject : Operand; R : Relation; Object : Operand; Until_Ending : Ending;
                         Words_Subject, Words_Object : String; Steps : Natural := 0) return Asked
   is
      W : Want (Interval);
   begin
      W.Until_Endings := Endings (Until_Ending);
      W.Max_Steps := Steps;
      W.Constraints.Append (Constraint'(Subject => Subject, Relation => R, Object => Object, Step => Unspecified,
                                        Strength => Unspecified, Must => False));
      return (Want => W,
              Words => To_Unbounded_String
                ("do " & Words_Subject & " " & Lower (Relation'Image (R))
                 & (if Words_Object'Length > 0 then " " & Words_Object else "") & " until "
                 & Lower (Ending'Image (Until_Ending)) & Steps_Words (Steps)));
   end Interval_Of;

   function Thing_Operand_Of (T : Positive) return Operand is
     ((Kind => Thing_Operand, Thing => Driver.World.Thing_Id (T)));
   function Role_Operand_Of (R : Role) return Operand is ((Kind => Role_Operand, The_Role => R));
   Nothing_Operand : constant Operand := (Kind => Nothing);

   function Onto_Of (T, Other : Positive; Until_Ending : Ending := Touched) return Asked is
     (Interval_Of (Thing_Operand_Of (T), Onto, Thing_Operand_Of (Other), Until_Ending,
                   "thing" & T'Image, "thing" & Other'Image));

   function Wait_Of (Steps : Natural) return Asked is
     (Interval_Of (Role_Operand_Of (Grasper), Still, Nothing_Operand, Settled, "grasper", "", Steps));

   --  What the scene holds: things at spots of the table's frame (the first
   --  ahead of the arm's eye where its line of sight meets the table).
   type Thing_Spec is record
      Model : Shapes_Of.Model;
      X, Y  : Real := 0.0;      --  metres, ahead and to the left
      About : Real := 0.0;      --  heading about up, radians
      Mu    : Real := 0.6;
      Fixed : Boolean := False;
      Joint : Sim.Sim_Joint;    --  in the table's frame, in units
      Drift : Vec3 := Zero3;    --  per beat, in the table's frame, in units
   end record;

   package Thing_Spec_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Thing_Spec);

   type Faults is record
      Lag   : Natural := 0;     --  beats a command takes to take effect
      Share : Real := 1.0;      --  of the commanded displacement the arm delivers
   end record;

   type Case_Kind is
     (Lift, Put_Down, Slide, Push, Turn, Onto, Stack, Door, Insert, Twist, Late, Partial, Detour, Strike, Dodge);

   type Scenario is record
      Name   : Unbounded_String;
      Kind   : Case_Kind := Lift;
      Shape  : Shape_Kind := Block;
      Hand   : Hand_Kind := Gripper;
      Fault  : Faults;
      Things : Thing_Spec_Vectors.Vector;
      Wants  : Asked_Vectors.Vector;
      Budget : Natural := 4000;     --  beats the run may take
   end record;

   --  What the simulated world held after a want.
   subtype Thing_Count is Positive range 1 .. 4;
   type Reals_Of_Things is array (Thing_Count) of Real;
   type Pairs is array (Thing_Count, Thing_Count) of Boolean;

   type Truth is record
      Things : Sim.Thing_Vectors.Vector;
      Lows   : Reals_Of_Things := [others => 0.0];
      Rests  : Pairs := [others => [others => False]];
      Tool   : Rigid;
      Beat   : Natural := 0;
   end record;

   package Truth_Vectors is new Ada.Containers.Vectors (Natural, Truth);

   type Result_Array is array (Positive range <>) of Result;

   type Outcome_Of_Run is record
      Results : Result_Array (1 .. 8);
      Done    : Natural := 0;           --  wants run to their end
      Beats   : Natural := 0;
      Seconds : Duration := 0.0;
      Failure : Unbounded_String;
      Truths  : Truth_Vectors.Vector;   --  0: before, J: after want J
      Moves   : Action_Rig.Move_Vectors.Vector;
      Start   : Rigid;                  --  the tool at the start, in the world
      Place   : Rigid;                  --  the table's frame in the world
      Eye     : Rigid;                  --  the still eye's pose in the world
      Turned  : Real := 0.0;            --  how far the first thing's heading went in all, beat by beat (signed)
   end record;

   --  An angle brought into (-pi, pi].
   function Wrapped (A : Real) return Real is
      X : Real := A;
   begin
      while X > Pi loop
         X := X - 2.0 * Pi;
      end loop;
      while X < -Pi loop
         X := X + 2.0 * Pi;
      end loop;
      return X;
   end Wrapped;

   --  A frame whose z is Up and whose x is Ahead (horizontal), at Point.
   function Level (Up, Ahead : Vec3; Point : Vec3) return Rigid is
      Z : constant Vec3 := Up;
      X : constant Vec3 := Unit (Ahead - Real'(Ahead * Z) * Z);
      Y : constant Vec3 := Cross (Z, X);
   begin
      return (Rotation => [[X (1), Y (1), Z (1)], [X (2), Y (2), Z (2)], [X (3), Y (3), Z (3)]], Translation => Point);
   end Level;

   procedure Execute_Scenario (S : Scenario; Out_Run : out Outcome_Of_Run) is
      M   : aliased Driver.Robot.Model;
      W   : aliased Sim.World;
      Ok  : Boolean;
      Why : Unbounded_String;
      Began : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      Driver.Robot.Load_Body (M, Path, Ok, Why);
      if not Ok then
         Out_Run.Failure := To_Unbounded_String ("the body file was not loaded: " & To_String (Why));
         return;
      end if;
      declare
         Readings : Driver.Observations.Reading_Vectors.Vector;
         Up       : constant Vec3 := Driver.Robot.Up (M).Unit_Vector;
         Table    : constant Driver.Geometry.Plane_Estimate := Driver.Robot.Table_In_Arm (M, 1);
      begin
         for G in 1 .. Driver.Robot.Group_Count (M) loop
            Readings.Append (Real_Array'(1 .. Driver.Robot.Group_Size (M, Driver.Robot.Group_Id (G)) => 0.0));
         end loop;
         declare
            Start  : constant Rigid := Driver.Robot.Tool_Pose (M, 1, Action_Rig.Observation_Of (M, Readings, 0)).Pose;
            Facing : constant Vec3 := Start.Rotation * Vec3'[0.0, 0.0, 1.0];
            High   : constant Real := (Start.Translation - Table.Centre) * Up;
            Spot   : constant Vec3 := Start.Translation + (High / (-(Facing * Up))) * Facing;
            Place  : constant Rigid := Level (Up, Facing, Spot);
            P      : aliased Action_Rig.Rig (M'Access, W'Access);
            Count  : constant Positive := Positive (S.Wants.Length);

            Done_Runs : Natural := 0 with Atomic;
            Finished  : Boolean := False with Atomic;
            Failure   : Unbounded_String;
            Results   : Result_Array (1 .. 8);
            Truths    : Truth_Vectors.Vector;

            --  The world as it is now, taken in a beat's window, where nothing else touches it.
            procedure Take_Truth is
               T : Truth;
               N : constant Natural := Natural'Min (4, Natural (W.Things.Length));
            begin
               T.Things := W.Things;
               T.Tool := W.Arms.First_Element.Tool;
               T.Beat := W.Beat;
               for K in 1 .. N loop
                  T.Lows (K) := Sim.Lowest (W, Driver.World.Thing_Id (K));
                  for L in 1 .. N loop
                     if K /= L then
                        T.Rests (K, L) := Sim.Rests_On (W, Driver.World.Thing_Id (K), Driver.World.Thing_Id (L));
                     end if;
                  end loop;
               end loop;
               Truths.Append (T);
            end Take_Truth;

            task Decider;
            task body Decider is
            begin
               for J in 1 .. Count loop
                  Driver.Action.Execution.Execute (P, S.Wants (J).Want, Results (J));
                  Done_Runs := J;
                  Driver.Beats.Within_A_Beat (Take_Truth'Access);
               end loop;
               Finished := True;
            exception
               when E : others =>
                  Driver.Beats.Release;
                  Failure := To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
                  Finished := True;
            end Decider;

            Sent    : Driver.Commands.Command := Driver.Commands.Hold;
            Pending : Driver.Commands.Command;
            Beat    : Natural := 0;
            Delayed : array (0 .. S.Fault.Lag) of Driver.Commands.Command := [others => Driver.Commands.Hold];
            Total   : Real := 0.0;
            Before  : Real;
            --  The first thing's heading about the table's up now, in the world.
            function Heading_Now return Real is
               V : constant Vec3 := Inverse (Place).Rotation * (W.Things.First_Element.Pose.Rotation * Vec3'[1.0, 0.0, 0.0]);
            begin
               return Arctan (V (2), V (1));
            end Heading_Now;
         begin
            Sim.Start (W, Place, Sigma, Pitch, 7);
            --  The still eye looks at the scene from the side, as in the self tests.
            W.Eye := Place * (Rotation => Exp ([-2.16, 0.0, 0.0]), Translation => Eye_At);
            Sim.Add_Arm (W, Base => Zero3, Reach => Real'Last, Tool => Inverse (Place) * Start, Lag => S.Fault.Lag,
                         Rate => 1.0, Delivery_Low => 1.0, Delivery_High => 1.0, Wrist => Wrist_Range, Tilt => Pi,
                         Plate_Radius => (if S.Hand = Plate then 0.02 * Scale else 0.0));
            if S.Hand = Gripper then
               Sim.Add_Gripper (W, 1, Opening => Opening, Width => Width, Thickness => Thickness, Depth => Depth);
            end if;
            for T of S.Things loop
               Sim.Add_Thing (W, T.Model,
                              (Rotation => Exp ([0.0, 0.0, T.About]), Translation => [T.X * Scale, T.Y * Scale, 0.0]),
                              Mu => T.Mu, Fixed => T.Fixed);
               declare
                  Id : constant Driver.World.Thing_Id := Driver.World.Thing_Id (W.Things.Length);
               begin
                  if T.Joint.Kind /= Sim.Loose then
                     Sim.Set_Joint (W, Id, T.Joint);
                  end if;
                  if abs T.Drift > 0.0 then
                     Sim.Set_Drift (W, Id, T.Drift);
                  end if;
               end;
            end loop;
            Take_Truth;
            Before := (if S.Things.Is_Empty then 0.0 else Heading_Now);
            while not Finished and then Beat < S.Budget loop
               declare
                  O    : constant Driver.Observations.Observation := Action_Rig.Observation_Of (M, Readings, Beat);
                  Took : Boolean := False;
               begin
                  Driver.Robot.Observe (M, O, Sent);
                  loop
                     Driver.Beats.Offer (O.Beat, O, Sent, Took);
                     exit when Took or else Finished;
                     delay 0.0;
                  end loop;
                  Pending := Driver.Commands.Hold;
                  if Took then
                     Driver.Beats.Await (Pending);
                  end if;
                  --  The body's faults: a command takes effect Lag beats late and delivers a share of its step.
                  declare
                     Due : Driver.Commands.Command := Pending;
                  begin
                     if S.Fault.Lag > 0 then
                        Delayed (Beat mod (S.Fault.Lag + 1)) := Pending;
                        Due := Delayed ((Beat + 1) mod (S.Fault.Lag + 1));
                     end if;
                     if S.Fault.Share < 1.0 then
                        for G in 1 .. Driver.Robot.Group_Count (M) loop
                           declare
                              Id : constant Driver.Robot.Group_Id := Driver.Robot.Group_Id (G);
                           begin
                              if Driver.Commands.Has_Target (Due, Id) and then Id = Driver.Robot.Arm_Group (M, 1) then
                                 declare
                                    Now  : constant Real_Array := Readings (Id);
                                    Goal : constant Real_Array := Driver.Commands.Target (Due, Id);
                                    Part : Real_Array := Now;
                                 begin
                                    for C in Part'Range loop
                                       Part (C) := Now (C) + S.Fault.Share * (Goal (C - Part'First + Goal'First) - Now (C));
                                    end loop;
                                    Driver.Commands.Set_Target (Due, Id, Part);
                                 end;
                              end if;
                           end;
                        end loop;
                     end if;
                     Action_Rig.Robot_Beat (M, W, 1, Readings, Due);
                  end;
                  if not S.Things.Is_Empty then
                     declare
                        Now : constant Real := Heading_Now;
                     begin
                        Total := Total + Wrapped (Now - Before);
                        Before := Now;
                     end;
                  end if;
                  Sent := Pending;
                  Beat := Beat + 1;
               end;
            end loop;
            if not Finished then
               abort Decider;
            end if;
            Out_Run.Results := Results;
            Out_Run.Done := Done_Runs;
            Out_Run.Beats := Beat;
            Out_Run.Failure := Failure;
            Out_Run.Truths := Truths;
            Out_Run.Moves := P.Moves;
            Out_Run.Start := Start;
            Out_Run.Place := Place;
            Out_Run.Eye := W.Eye;
            Out_Run.Turned := Total;
         end;
      end;
      Out_Run.Seconds := Ada.Calendar."-" (Ada.Calendar.Clock, Began);
   end Execute_Scenario;

   ----------------------------------------------------------------------
   --  Truth, read from the snapshots after each want.

   type Verdict is record
      Pass : Boolean := False;
      What : Unbounded_String;      --  what was reached against what was asked
   end record;

   function Table_Of (R : Outcome_Of_Run; P : Rigid) return Rigid is (Inverse (R.Place) * P);

   function Centre_Of (R : Outcome_Of_Run; J : Natural; Id : Positive) return Vec3 is
      T : Sim.Sim_Thing renames R.Truths (J).Things (Id);
   begin
      return Table_Of (R, T.Pose) * Shapes_Of.Centre (T.Shape);
   end Centre_Of;

   --  The direction of the thing's own x about the table's up, counter-clockwise seen from above.
   function Heading_Of (R : Outcome_Of_Run; J : Natural; Id : Positive) return Real is
      V : constant Vec3 := Table_Of (R, R.Truths (J).Things (Id).Pose).Rotation * Vec3'[1.0, 0.0, 0.0];
   begin
      return Arctan (V (2), V (1));
   end Heading_Of;

   function Held_At (R : Outcome_Of_Run; J : Natural; Id : Positive) return Boolean is
     (R.Truths (J).Things (Id).Held_By /= 0);

   function Held_Words (R : Outcome_Of_Run; J : Natural; Id : Positive) return String is
     (if Held_At (R, J, Id) then "held" else "let go");

   function Moved_Along (R : Outcome_Of_Run; J : Natural; Id : Positive) return Real is
     (abs (Centre_Of (R, J, Id) - Centre_Of (R, 0, Id)));

   function Final_Of (R : Outcome_Of_Run; J : Positive) return Ending is (R.Results (J).Final);

   function Says (V : Boolean; Yes, No : String) return String is (if V then Yes else No);

   --  Where the thing's middle falls along the still eye's image columns (x over z of the eye's frame):
   --  the column grows to the right.
   function Column_Of (R : Outcome_Of_Run; J : Natural; Id : Positive) return Real is
      Q : constant Vec3 := Transpose (R.Eye.Rotation) * (R.Place * Centre_Of (R, J, Id) - R.Eye.Translation);
   begin
      return Q (1) / Q (3);
   end Column_Of;

   --  How far from its middle the farthest part of a shape lies, as the scene builds it.
   function Reach_Of (K : Shape_Kind) return Real is
     (case K is
         when Bar      => 0.1 * Scale,
         when Block    => 0.0 * Scale,
         when Cylinder => 0.0 * Scale,
         when Scissors => 0.09 * Scale,
         when Cup      => 0.07 * Scale);

   function Judge (S : Scenario; R : Outcome_Of_Run) return Verdict is
      V    : Verdict;
      Last : constant Natural := R.Done;
      procedure Say_It (Text : String) is
      begin
         V.What := To_Unbounded_String (Text);
      end Say_It;
   begin
      if Last < S.Wants.Last_Index then
         Say_It ("the run stopped after" & Last'Image & " of" & S.Wants.Last_Index'Image & " wants"
                 & (if Length (R.Failure) > 0 then ": the decider failed" else ""));
         return V;
      end if;
      case S.Kind is
         when Lift | Detour =>
            declare
               Rise : constant Real := R.Truths (1).Lows (1) - R.Truths (0).Lows (1);
            begin
               V.Pass := Final_Of (R, 1) = Free and then Rise > 0.0 and then Held_At (R, 1, 1)
                 and then (S.Kind = Lift or else Moved_Along (R, 1, 2) < Pitch);
               Say_It ("asked: off the table; reached: rose " & Img (Rise) & " units (" & Metres (Rise) & "), "
                       & Held_Words (R, 1, 1));
            end;
         when Put_Down =>
            declare
               Low : constant Real := R.Truths (2).Lows (1);
            begin
               V.Pass := Final_Of (R, 1) = Free and then Final_Of (R, 2) = Touched and then abs Low < Pitch
                 and then not Held_At (R, 2, 1);
               Say_It ("asked: back on the table; reached: " & Img (Low) & " units over it (" & Metres (Low) & "), "
                       & Held_Words (R, 2, 1));
            end;
         when Slide | Push =>
            declare
               Before : constant Real := Column_Of (R, 0, 1) - Column_Of (R, 0, 2);
               After  : constant Real := Column_Of (R, 1, 1) - Column_Of (R, 1, 2);
               Went   : constant Real := Moved_Along (R, 1, 1);
               Off    : constant Real := R.Truths (1).Lows (1);
            begin
               V.Pass := Final_Of (R, 1) = Settled and then After < 0.0 and then abs Off < Pitch;
               Say_It ("asked: left of thing 2 in the still eye's image (columns apart" & Img (Before, 4) & " to below 0); "
                       & "reached:" & Says (After < 0.0, " left of it by ", " still right of it by ") & Img (abs After, 4)
                       & " columns, moved " & Img (Went) & " units (" & Metres (Went) & "), " & Img (Off)
                       & " units off the table, " & Held_Words (R, 1, 1));
            end;
         when Turn =>
            declare
               Turned : constant Real := Wrapped (Heading_Of (R, 1, 1) - Heading_Of (R, 0, 1));
               --  The least turn that moves the farthest part of the shape by one sample pitch.
               Seen   : constant Real := (if Reach_Of (S.Shape) > 0.0 then Pitch / Reach_Of (S.Shape) else 0.0);
            begin
               if Reach_Of (S.Shape) = 0.0 then
                  V.Pass := Final_Of (R, 1) = Refused;
                  Say_It ("asked: turn it; it has no long side, so no heading to turn; reached: " & Ending'Image (Final_Of (R, 1))
                          & ", turned " & Img (Turned) & " rad");
               else
                  V.Pass := Final_Of (R, 1) = Timeout and then Turned > Seen and then abs R.Truths (1).Lows (1) < Pitch;
                  Say_It ("asked: heading up (counter-clockwise), three steps; reached: turned " & Img (Turned, 4)
                          & " rad (" & Img (Turned * 180.0 / Pi, 2) & " deg; a pitch at its far end is " & Img (Seen, 4) & " rad), "
                          & Img (R.Truths (1).Lows (1)) & " units off the table, " & Held_Words (R, 1, 1));
               end if;
            end;
         when Onto | Late | Partial =>
            declare
               Gap : constant Real := R.Truths (1).Lows (1);
            begin
               V.Pass := Final_Of (R, 1) = Touched and then R.Truths (1).Rests (1, 2) and then not Held_At (R, 1, 1);
               Say_It ("asked: resting on thing 2; reached: " & Says (R.Truths (1).Rests (1, 2), "resting on it", "not resting on it")
                       & ", " & Img (Gap) & " units over the table, " & Img (abs (Centre_Of (R, 1, 1) - Centre_Of (R, 1, 2)), 3)
                       & " units from its middle, " & Held_Words (R, 1, 1));
            end;
         when Stack =>
            declare
               Slip : constant Real := abs (Centre_Of (R, 2, 1) - Centre_Of (R, 1, 1));
            begin
               V.Pass := Final_Of (R, 1) = Touched and then R.Truths (2).Rests (1, 2) and then not Held_At (R, 2, 1)
                 and then Slip < Pitch;
               Say_It ("asked: stacked on thing 2 and staying; reached: " & Says (R.Truths (2).Rests (1, 2), "resting on it", "not resting on it")
                       & ", " & Held_Words (R, 2, 1) & ", moved " & Img (Slip) & " units after the let-go");
            end;
         when Door =>
            declare
               Swing : constant Real := R.Truths (1).Things (1).Joint.Q;
               Seen  : constant Real := Pitch / (0.25 * Scale);   --  a pitch at the door's far end
            begin
               V.Pass := Swing > Seen;
               Say_It ("asked: swing the door open (its heading up); reached: the hinge turned " & Img (Swing, 4) & " rad of "
                       & Img (R.Truths (1).Things (1).Joint.High) & " (" & Img (Swing * 180.0 / Pi, 2) & " deg; a pitch at its far end is "
                       & Img (Seen, 4) & " rad), " & Held_Words (R, 1, 1));
            end;
         when Insert =>
            declare
               D : constant Real := abs (Centre_Of (R, 1, 1) - Centre_Of (R, 1, 2));
            begin
               V.Pass := Final_Of (R, 1) = Touched and then D < 0.01 * Scale and then abs R.Truths (1).Lows (1) < Pitch;
               Say_It ("asked: the peg into the sleeve; reached: " & Img (D) & " units (" & Metres (D) & ") from the sleeve's axis, "
                       & Img (R.Truths (1).Lows (1)) & " units over the table, " & Held_Words (R, 1, 1));
            end;
         when Twist =>
            --  A twist goes on past what the wrist turns in one grasp.
            begin
               V.Pass := R.Turned > 2.0 * Wrist_Range;
               Say_It ("asked: turn it on, further than the wrist turns in one grasp (" & Img (2.0 * Wrist_Range, 3) & " rad); reached: "
                       & Ending'Image (Final_Of (R, 1)) & ", turned " & Img (R.Turned, 4) & " rad in all ("
                       & Img (R.Turned * 180.0 / Pi, 2) & " deg), " & Img (R.Truths (1).Lows (1)) & " units off the table, "
                       & Held_Words (R, 1, 1));
            end;
         when Strike =>
            declare
               Moved : constant Real := abs (R.Truths (Last).Tool.Translation - R.Truths (0).Tool.Translation);
            begin
               V.Pass := Final_Of (R, 1) = Touched;
               Say_It ("asked: hit it hard (a speed at the touch; the language has none); reached: " & Ending'Image (Final_Of (R, 1))
                       & ", the hand moved " & Img (Moved) & " units (" & Metres (Moved) & "), thing 1 moved "
                       & Img (Moved_Along (R, Last, 1)) & " units");
            end;
         when Dodge =>
            declare
               Moved : constant Real := abs (R.Truths (Last).Tool.Translation - R.Truths (0).Tool.Translation);
               Came  : constant Real := Moved_Along (R, Last, 2);
            begin
               V.Pass := Moved > Pitch;
               Say_It ("asked: keep clear of thing 2, which drifts at the hand; reached: " & Ending'Image (Final_Of (R, 1))
                       & ", the hand moved " & Img (Moved) & " units (" & Metres (Moved) & ") while thing 2 came " & Img (Came)
                       & " units (" & Metres (Came) & ")");
            end;
      end case;
      return V;
   end Judge;

   procedure Show_Run (S : Scenario; Out_Run : Outcome_Of_Run; V : Verdict) is
   begin
      Say ("== " & To_String (S.Name) & ": " & (if V.Pass then "PASS" else "FAIL") & "  [" & Img (Real (Out_Run.Seconds), 1) & " s]");
      for J in 1 .. S.Wants.Last_Index loop
         Say ("  the brain asked: " & To_String (S.Wants (J).Words));
         if J <= Out_Run.Done then
            Say ("  ending: " & Ending'Image (Out_Run.Results (J).Final) & " | " & To_String (Out_Run.Results (J).Account)
                 & (if Length (Out_Run.Results (J).Tried) > 0 then " | tried: " & To_String (Out_Run.Results (J).Tried)
                    else ""));
         else
            Say ("  ending: (not run)");
         end if;
      end loop;
      Say ("  " & To_String (V.What));
      if Length (Out_Run.Failure) > 0 then
         Say ("  the decider failed: " & To_String (Out_Run.Failure));
      end if;
      declare
         N : constant Natural := Natural (Out_Run.Moves.Length);
         Counts : array (Plants.Step_Outcome) of Natural := [others => 0];
         Shift, Turn : Real_Array (1 .. Natural'Max (1, N)) := [others => 0.0];
         Followed : Natural := 0;
         function Largest (X : Real_Array) return Real is
            L : Real := Real'First;
         begin
            for V of X loop
               L := Real'Max (L, V);
            end loop;
            return L;
         end Largest;
      begin
         for R of Out_Run.Moves loop
            Counts (R.Outcome) := Counts (R.Outcome) + 1;
            if R.Outcome /= Plants.Refused then
               Followed := Followed + 1;
               Shift (Followed) := abs (R.Reached.Translation - R.Asked.Translation);
               Turn (Followed) := Angle (Transpose (R.Asked.Rotation) * R.Reached.Rotation);
            end if;
         end loop;
         Say ("  arm moves:" & N'Image & " (reached" & Counts (Plants.Reached)'Image & ", short"
              & Counts (Plants.Short)'Image & ", blocked" & Counts (Plants.Blocked)'Image & ", refused"
              & Counts (Plants.Refused)'Image & "), " & Img (Out_Run.Beats) & " beats");
         if Followed > 0 then
            Say ("  asked against reached, position (units): median " & Img (Driver.Stats.Median (Shift (1 .. Followed)))
                 & ", largest " & Img (Largest (Shift (1 .. Followed))) & "; turn (rad): median "
                 & Img (Driver.Stats.Median (Turn (1 .. Followed))) & ", largest " & Img (Largest (Turn (1 .. Followed))));
         end if;
         if Verbose then
            for K in 1 .. N loop
               declare
                  R  : constant Action_Rig.Move_Record := Out_Run.Moves (K);
                  Up : constant Vec3 := Rotate (Out_Run.Place, [0.0, 0.0, 1.0]);
                  function Over (Tool : Rigid) return Real is (Real'((Tool.Translation - Out_Run.Place.Translation) * Up));
               begin
                  Say ("    move" & K'Image & ": " & Plants.Step_Outcome'Image (R.Outcome) & ", asked "
                       & Img (Over (R.Asked)) & " over the table, reached " & Img (Over (R.Reached))
                       & "; apart " & Img (abs (R.Reached.Translation - R.Asked.Translation)) & " units, "
                       & Img (Angle (Transpose (R.Asked.Rotation) * R.Reached.Rotation)) & " rad"
                       & (if Length (R.Why) > 0 then " | " & To_String (R.Why) else ""));
               end;
            end loop;
         else
            for R of Out_Run.Moves loop
               if R.Outcome = Plants.Refused then
                  Say ("  refused: " & To_String (R.Why));
                  exit;
               end if;
            end loop;
         end if;
      end;
   end Show_Run;

   Passed, Failed : Natural := 0;
   Names_Failed   : Unbounded_String;

   procedure Run_Case (S : Scenario) is
      R : Outcome_Of_Run;
   begin
      if Filter'Length > 0 and then Ada.Strings.Fixed.Index (To_String (S.Name), Filter) = 0 then
         return;
      end if;
      Action_Rig.Trace := Tracing;
      Action_Rig.Cartesian := Ideal;
      Action_Rig.Reach_Log.Clear;
      Action_Rig.Beat_Log.Clear;
      Execute_Scenario (S, R);
      declare
         V : constant Verdict := Judge (S, R);
      begin
         Show_Run (S, R, V);
         if Tracing then
            declare
               First : constant Natural := Natural'Max (1, Natural (Action_Rig.Reach_Log.Length) - 24);
            begin
               Say ("  the last reaches asked of the motion layer (" & Img (Natural (Action_Rig.Reach_Log.Length)) & " in all):");
               for K in First .. Natural (Action_Rig.Reach_Log.Length) loop
                  declare
                     Q : constant Action_Rig.Reach_Record := Action_Rig.Reach_Log (K);
                  begin
                     Say ("    reach" & K'Image & ": shift " & Img (abs (Q.Asked.Translation - Q.From.Translation), 4)
                          & " units, turn " & Img (Angle (Transpose (Q.From.Rotation) * Q.Asked.Rotation), 4) & " rad: "
                          & Plants.Reach_Status'Image (Q.Status) & (if Length (Q.Why) > 0 then " | " & To_String (Q.Why) else "")
                          & (if Q.Bow > 0.0 then " | the straight path leaves a bow of " & Img (Q.Bow, 4) & " units" else ""));
                  end;
               end loop;
               declare
                  Worst : Real := 0.0;
                  --  Positions in the table's frame: ahead, left, up.
                  function Spot (Tool : Rigid) return String is
                     T : constant Vec3 := Table_Of (R, Tool).Translation;
                  begin
                     return Img (T (1), 3) & ", " & Img (T (2), 3) & ", " & Img (T (3), 3);
                  end Spot;
               begin
                  for B of Action_Rig.Beat_Log loop
                     Worst := Real'Max (Worst, B.Bow);
                  end loop;
                  Say ("  the joints' own straight line bowed from the tool's straight line by up to " & Img (Worst, 4)
                       & " units (" & Metres (Worst) & ") in a beat, over" & Action_Rig.Beat_Log.Length'Image & " beats with a target");
                  for B of Action_Rig.Beat_Log loop
                     if B.Stopped then
                        Say ("    stopped by the world (table frame: ahead, left, up): the beat began at " & Spot (B.From)
                             & ", its joints' targets put the tool at " & Spot (B.To) & ", it stopped at " & Spot (B.At_Stop)
                             & ", bow " & Img (B.Bow, 4));
                     end if;
                  end loop;
               end;
            end;
         end if;
         if V.Pass then
            Passed := Passed + 1;
         else
            Failed := Failed + 1;
            Append (Names_Failed, "  " & To_String (S.Name) & ASCII.LF);
         end if;
      end;
   end Run_Case;

   ----------------------------------------------------------------------
   --  The cases.

   function Called (Kind, What : String; About : Real := 0.0) return Unbounded_String is
     (To_Unbounded_String (Kind & " " & What & (if About /= 0.0 then " across" else "")));

   function Alone (K : Shape_Kind; About : Real) return Thing_Spec_Vectors.Vector is
      V : Thing_Spec_Vectors.Vector;
   begin
      V.Append (Thing_Spec'(Model => Shape (K), About => About, others => <>));
      return V;
   end Alone;

   function Lift_Case (K : Shape_Kind; About : Real) return Scenario is
      S : Scenario;
   begin
      S.Name := Called ("lift", Lower (Shape_Kind'Image (K)), About);
      S.Kind := Lift;
      S.Shape := K;
      S.Things := Alone (K, About);
      S.Wants.Append (Change_Of (1, 1, True, Free));
      return S;
   end Lift_Case;

   function Put_Down_Case (K : Shape_Kind; About : Real) return Scenario is
      S : Scenario := Lift_Case (K, About);
   begin
      S.Name := Called ("put down", Lower (Shape_Kind'Image (K)), About);
      S.Kind := Put_Down;
      S.Wants.Append (Change_Of (1, 1, False, Touched));
      return S;
   end Put_Down_Case;

   --  A second thing for the first to be moved by: a low block beside it, to
   --  the left, or on the eye's side.
   function Marker_Block (X, Y : Real; Z : Real := 0.01) return Thing_Spec is
     ((Model => Shapes_Of.Block (0.06 * Scale, 0.06 * Scale, Z * Scale), X => X, Y => Y, Fixed => True, others => <>));

   function Slide_Case (K : Shape_Kind; Hand : Hand_Kind) return Scenario is
      S : Scenario;
   begin
      S.Name := Called ((if Hand = Gripper then "slide" else "push"), Lower (Shape_Kind'Image (K)));
      S.Kind := (if Hand = Gripper then Slide else Push);
      S.Shape := K;
      S.Hand := Hand;
      S.Things := Alone (K, 0.0);
      --  A low block behind it and to the eye's side (the still eye's image columns grow ahead): the thing is to
      --  be left of it, which takes it backwards along the table, past the block's side.
      S.Things.Append (Marker_Block (-0.12, -0.15));
      S.Wants.Append (Interval_Of (Thing_Operand_Of (1), Left, Thing_Operand_Of (2), Settled, "thing 1", "thing 2"));
      return S;
   end Slide_Case;

   function Turn_Case (K : Shape_Kind; About : Real) return Scenario is
      S : Scenario;
   begin
      S.Name := Called ("turn", Lower (Shape_Kind'Image (K)), About);
      S.Kind := Turn;
      S.Shape := K;
      S.Things := Alone (K, About);
      S.Wants.Append (Change_Of (1, 2, True, Timeout, 3));
      return S;
   end Turn_Case;

   --  The block a thing is put onto: eight centimetres square and five high, to the left.
   function Target_Block (Wide : Real := 0.08; High : Real := 0.05) return Thing_Spec is
     ((Model => Shapes_Of.Block (Wide * Scale, Wide * Scale, High * Scale), X => 0.0, Y => 0.12, others => <>));

   function Onto_Case (K : Shape_Kind; About : Real) return Scenario is
      S : Scenario;
   begin
      S.Name := Called ("onto", Lower (Shape_Kind'Image (K)), About);
      S.Kind := Onto;
      S.Shape := K;
      S.Things := Alone (K, About);
      S.Things.Append (Target_Block);
      S.Wants.Append (Onto_Of (1, 2));
      return S;
   end Onto_Case;

   function Stack_Case (K : Shape_Kind; About : Real) return Scenario is
      S : Scenario := Onto_Case (K, About);
   begin
      S.Name := Called ("stack", Lower (Shape_Kind'Image (K)), About);
      S.Kind := Stack;
      --  A small base, only a little wider than the hand's lobes are apart.
      S.Things.Replace_Element (2, Target_Block (Wide => 0.06, High => 0.04));
      S.Wants.Append (Wait_Of (10));
      return S;
   end Stack_Case;

   function Fault_Case (Name : String; Fault : Faults; K : Shape_Kind) return Scenario is
      S : Scenario := Onto_Case (K, 0.0);
   begin
      S.Name := To_Unbounded_String (Name & " " & Lower (Shape_Kind'Image (K)));
      S.Kind := (if Fault.Lag > 0 then Late else Partial);
      S.Fault := Fault;
      return S;
   end Fault_Case;

   function Detour_Case (K : Shape_Kind) return Scenario is
      S : Scenario := Lift_Case (K, 0.0);
   begin
      S.Name := To_Unbounded_String ("detour " & Lower (Shape_Kind'Image (K)));
      S.Kind := Detour;
      --  A wall that cannot give way, between the hand and the thing, lower than the hand starts.
      S.Things.Append (Thing_Spec'(Model => Shapes_Of.Block (0.03 * Scale, 0.30 * Scale, 0.10 * Scale),
                                   X => -0.12, Y => 0.0, Fixed => True, others => <>));
      return S;
   end Detour_Case;

   function Door_Case return Scenario is
      S : Scenario;
      J : Sim.Sim_Joint;
   begin
      S.Name := To_Unbounded_String ("door open");
      S.Kind := Door;
      --  A slab on edge, hinged at its end nearer the arm: it opens by turning counter-clockwise.
      J := (Kind => Sim.Hinge, Axis => [0.0, 0.0, 1.0], Point => [-0.125 * Scale, 0.0, 0.0], Low => 0.0, High => 1.5,
            others => <>);
      S.Things.Append (Thing_Spec'(Model => Shapes_Of.Block (0.25 * Scale, 0.02 * Scale, 0.15 * Scale), Joint => J,
                                   others => <>));
      S.Wants.Append (Change_Of (1, 2, True, Timeout, 8));
      return S;
   end Door_Case;

   function Sleeve (Outer, Inner, High : Real) return Shapes_Of.Model is
      M : Shapes_Of.Model;
   begin
      M.Parts.Append (Shapes_Of.Part'(Kind => Shapes_Of.Tube, Pose => (Rotation => Identity3, Translation => [0.0, 0.0, High / 2.0]),
                                      Sizes => [Outer, Inner, High / 2.0]));
      return M;
   end Sleeve;

   function Insert_Case return Scenario is
      S : Scenario;
   begin
      S.Name := To_Unbounded_String ("insert peg");
      S.Kind := Insert;
      --  A peg standing on the table, and a sleeve fixed beside it that it is to go into.
      S.Things.Append (Thing_Spec'(Model => Shapes_Of.Upright_Cylinder (0.012 * Scale, 0.08 * Scale), others => <>));
      S.Things.Append (Thing_Spec'(Model => Sleeve (0.03 * Scale, 0.016 * Scale, 0.05 * Scale), X => 0.0, Y => 0.12,
                                   Fixed => True, others => <>));
      S.Wants.Append (Interval_Of (Thing_Operand_Of (1), Into, Thing_Operand_Of (2), Touched, "thing 1", "thing 2"));
      return S;
   end Insert_Case;

   function Twist_Case return Scenario is
      S : Scenario;
   begin
      S.Name := To_Unbounded_String ("twist bar");
      S.Kind := Twist;
      S.Shape := Bar;
      S.Things := Alone (Bar, 0.0);
      S.Wants.Append (Change_Of (1, 2, True, Timeout, 30));
      S.Budget := 8000;
      return S;
   end Twist_Case;

   function Strike_Case return Scenario is
      S : Scenario;
   begin
      S.Name := To_Unbounded_String ("strike");
      S.Kind := Strike;
      S.Things := Alone (Block, 0.0);
      --  The closest the language comes: press the thing, and as hard as it says.
      declare
         W : Asked := Interval_Of (Role_Operand_Of (Grasper), Press, Thing_Operand_Of (1), Touched, "grasper", "thing 1");
      begin
         W.Want.Constraints.Replace_Element (1, (Subject => Role_Operand_Of (Grasper), Relation => Press,
                                                 Object => Thing_Operand_Of (1), Step => Unspecified,
                                                 Strength => Hard, Must => False));
         W.Words := To_Unbounded_String ("do grasper press thing 1 hard until touched");
         S.Wants.Append (W);
      end;
      return S;
   end Strike_Case;

   function Dodge_Case return Scenario is
      S : Scenario;
   begin
      S.Name := To_Unbounded_String ("dodge");
      S.Kind := Dodge;
      S.Things := Alone (Block, 0.0);
      --  A block coming at the hand along the table, beat after beat.
      S.Things.Append (Thing_Spec'(Model => Shapes_Of.Block (0.04 * Scale, 0.04 * Scale, 0.04 * Scale), X => -0.3, Y => 0.0,
                                   Drift => [0.004 * Scale, 0.0, 0.0], others => <>));
      S.Wants.Append (Interval_Of (Role_Operand_Of (Grasper), Clear, Thing_Operand_Of (2), Timeout, "grasper", "thing 2", 20));
      return S;
   end Dodge_Case;

begin
   Say ("body " & Path & ", " & Img (Scale, 2) & " units per metre");
   for K in Shape_Kind loop
      Run_Case (Lift_Case (K, 0.0));
      if K in Bar | Scissors | Cup then
         Run_Case (Lift_Case (K, Pi / 2.0));
      end if;
   end loop;
   for K in Shape_Kind loop
      Run_Case (Put_Down_Case (K, 0.0));
   end loop;
   for K in Shape_Kind loop
      Run_Case (Slide_Case (K, Gripper));
   end loop;
   for K in Shape_Kind loop
      Run_Case (Slide_Case (K, Plate));
   end loop;
   for K in Shape_Kind loop
      Run_Case (Turn_Case (K, 0.0));
   end loop;
   for K in Shape_Kind loop
      Run_Case (Onto_Case (K, 0.0));
   end loop;
   for K in Shape_Kind loop
      Run_Case (Stack_Case (K, 0.0));
   end loop;
   Run_Case (Door_Case);
   Run_Case (Insert_Case);
   Run_Case (Twist_Case);
   Run_Case (Fault_Case ("late", (Lag => 3, Share => 1.0), Block));
   Run_Case (Fault_Case ("partial", (Lag => 0, Share => 0.75), Block));
   Run_Case (Detour_Case (Bar));
   Run_Case (Strike_Case);
   Run_Case (Dodge_Case);
   Say ("passed" & Passed'Image & ", failed" & Failed'Image);
   if Length (Names_Failed) > 0 then
      Say ("failed:");
      Ada.Text_IO.Put (To_String (Names_Failed));
   end if;
end Action_Matrix;
