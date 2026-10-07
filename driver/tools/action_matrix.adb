--  action_matrix BODY_FILE UNITS_PER_METRE CAMERA_HEIGHT
--
--  The action layer's five shapes on a measured body, every arm move
--  through the real motion layer (Action_Rig): each shape lifted until free
--  and put back down until touched, and put onto a block until touched, in
--  two placements. "table" puts the table where the scene had it, the arm's
--  eye CAMERA_HEIGHT metres above it and the shape where the eye's line of
--  sight meets it; "reach" puts the shape between the lobes as the arm
--  stands, the table under it. The hand is the simulated gripper of the self
--  tests, on the arm's eye's frame; UNITS_PER_METRE scales the self tests'
--  metric scene into the body's own unit (the fit makes the root mean square
--  of its keyframes' eye positions one unit, so the scene's size has to come
--  from outside the body; A10's truth gives 52.1: its left wrist eye is
--  0.1825 m from the first joint's axis, which turns that eye 0.951 units
--  per 0.1 rad).
--
--  For every arm move the tool pose the action layer asked for is printed
--  against the one the readings reached, and every run's ending and
--  account.

with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Numerics;
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
with Driver.Log;
with Driver.Numerics;
with Driver.Observations;
with Driver.Robot;
with Driver.Stats;

procedure Action_Matrix is

   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Action;
   use type Driver.Action.Plants.Step_Outcome;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   package Sim renames Driver.Action.Plants.Tests;
   package Shapes_Of renames Driver.Action.Snapshots.Tests;

   Pi : constant := Ada.Numerics.Pi;

   function Img (X : Real) return String is (Driver.Log.Image (X, 4));

   function Largest (X : Real_Array) return Real is
      L : Real := Real'First;
   begin
      for V of X loop
         L := Real'Max (L, V);
      end loop;
      return L;
   end Largest;

   procedure Say (S : String) is
   begin
      Ada.Text_IO.Put_Line (S);
   end Say;

   Path  : constant String := Ada.Command_Line.Argument (1);
   Scale : constant Real := Real'Value (Ada.Command_Line.Argument (2));
   Eye_Height : constant Real := Real'Value (Ada.Command_Line.Argument (3)) * Scale;

   --  The self tests' scene, in the body's unit.
   Pitch : constant Real := 0.005 * Scale;
   Sigma : constant Real := 0.0005 * Scale;
   Opening   : constant Real := 0.08 * Scale;
   Width     : constant Real := 0.015 * Scale;
   Thickness : constant Real := 0.01 * Scale;
   Depth     : constant Real := 0.04 * Scale;

   type Shape_Kind is (Bar, Block, Cylinder, Scissors, Cup);
   type Placement is (Table, Reach);
   type Run_Kind is (Up_And_Down, Onto_A_Block);

   function Shape (K : Shape_Kind) return Shapes_Of.Model is
     (case K is
         when Bar      => Shapes_Of.Bar (0.2 * Scale, 0.02 * Scale, 0.02 * Scale),
         when Block    => Shapes_Of.Block (0.04 * Scale, 0.04 * Scale, 0.04 * Scale),
         when Cylinder => Shapes_Of.Upright_Cylinder (0.025 * Scale, 0.08 * Scale),
         when Scissors => Shapes_Of.Scissors (0.18 * Scale, 0.016 * Scale, 0.006 * Scale),
         when Cup      => Shapes_Of.Cup (0.03 * Scale, 0.004 * Scale, 0.08 * Scale));

   --  How high the shape's middle stands over the table.
   function Middle_Height (K : Shape_Kind) return Real is
     (case K is
         when Bar      => 0.01 * Scale,
         when Block    => 0.02 * Scale,
         when Cylinder => 0.04 * Scale,
         when Scissors => 0.003 * Scale,
         when Cup      => 0.04 * Scale);

   function Endings (E : Ending) return Ending_Set is
      S : Ending_Set := [others => False];
   begin
      S (E) := True;
      return S;
   end Endings;

   function Height_Want (Up : Boolean; Until_Ending : Ending) return Want is
     ((Kind => Change, Until_Endings => Endings (Until_Ending), Max_Steps => 0, Eye => Any_Eye, Anyway => False,
       Thing => 1, Quantity => 1, Increase => Up));

   function Onto_Want return Want is
      W : Want (Interval);
   begin
      W.Until_Endings := Endings (Touched);
      W.Constraints.Append (Constraint'(Subject => (Kind => Thing_Operand, Thing => 1), Relation => Onto,
                                        Object => (Kind => Thing_Operand, Thing => 2), Step => Unspecified,
                                        Strength => Unspecified, Must => False));
      return W;
   end Onto_Want;

   --  A frame whose z is Up, at Point.
   function Level (Up : Vec3; Point : Vec3) return Rigid is
      Z : constant Vec3 := Up;
      X : Vec3 := Cross ([0.0, 1.0, 0.0], Z);
      Y : Vec3;
   begin
      if abs X = 0.0 then
         X := Cross ([1.0, 0.0, 0.0], Z);
      end if;
      X := X / abs X;
      Y := Cross (Z, X);
      return (Rotation => [[X (1), Y (1), Z (1)], [X (2), Y (2), Z (2)], [X (3), Y (3), Z (3)]], Translation => Point);
   end Level;

   Beat_Budget : constant := 6000;

   procedure Run (K : Shape_Kind; Where : Placement; What : Run_Kind) is
      M  : aliased Driver.Robot.Model;
      W  : aliased Sim.World;
      Ok : Boolean;
      Why : Unbounded_String;
   begin
      Driver.Robot.Load_Body (M, Path, Ok, Why);
      if not Ok then
         Say ("the body file was not loaded: " & To_String (Why));
         return;
      end if;
      declare
         Readings : Driver.Observations.Reading_Vectors.Vector;
         Up       : constant Vec3 := Driver.Robot.Up (M).Unit_Vector;
      begin
         for G in 1 .. Driver.Robot.Group_Count (M) loop
            Readings.Append (Real_Array'(1 .. Driver.Robot.Group_Size (M, Driver.Robot.Group_Id (G)) => 0.0));
         end loop;
         declare
            Start  : constant Rigid := Driver.Robot.Tool_Pose (M, 1, Action_Rig.Observation_Of (M, Readings, 0)).Pose;
            Facing : constant Vec3 := Start.Rotation * Vec3'[0.0, 0.0, 1.0];
            Tips   : constant Vec3 := Start * Vec3'[0.0, 0.0, Depth];
            --  Where the shape's middle goes, and the table under it.
            Middle : constant Vec3 :=
              (case Where is
                  when Reach => Tips,
                  when Table => Start.Translation + (Eye_Height / (-(Facing * Up))) * Facing
                                + Middle_Height (K) * Up);
            Place  : constant Rigid := Level (Up, Middle - Middle_Height (K) * Up);
            P      : aliased Action_Rig.Rig (M'Access, W'Access);
            Count  : constant Positive := (if What = Up_And_Down then 2 else 1);
            Results : array (1 .. 2) of Result;
            Done_Runs : Natural := 0 with Atomic;
            Finished  : Boolean := False with Atomic;
            Failure   : Unbounded_String;

            --  The J-th want of the run.
            function Want_Of (J : Positive) return Want is
              (if What = Onto_A_Block then Onto_Want
               elsif J = 1 then Height_Want (True, Free)
               else Height_Want (False, Touched));

            task Decider;
            task body Decider is
            begin
               for J in 1 .. Count loop
                  Driver.Action.Execution.Execute (P, Want_Of (J), Results (J));
                  Done_Runs := J;
                  exit when What = Up_And_Down and then J = 1 and then Results (1).Final /= Free;
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
         begin
            Sim.Start (W, Place, Sigma, Pitch, 7);
            --  The still eye looks at the shape from in front, as in the self tests.
            W.Eye := Place * (Rotation => Exp ([-2.16, 0.0, 0.0]), Translation => [0.0, -0.6 * Scale, 0.4 * Scale]);
            Sim.Add_Arm (W, Base => Zero3, Reach => Real'Last, Tool => Inverse (Place) * Start, Lag => 0, Rate => 1.0,
                         Delivery_Low => 1.0, Delivery_High => 1.0, Wrist => Pi, Tilt => Pi);
            Sim.Add_Gripper (W, 1, Opening => Opening, Width => Width, Thickness => Thickness, Depth => Depth);
            Sim.Add_Thing (W, Shape (K), (Rotation => Identity3, Translation => Zero3), Mu => 0.6);
            if What = Onto_A_Block then
               --  A low block beside it, along the table's first axis.
               Sim.Add_Thing (W, Shapes_Of.Block (0.08 * Scale, 0.08 * Scale, 0.01 * Scale),
                              (Rotation => Identity3, Translation => [0.06 * Scale + 0.1 * Scale, 0.0, 0.0]), Mu => 0.6);
            end if;
            while not Finished and then Beat < Beat_Budget loop
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
                  Action_Rig.Robot_Beat (M, W, 1, Readings, Pending);
                  Sent := Pending;
                  Beat := Beat + 1;
               end;
            end loop;
            if not Finished then
               abort Decider;
            end if;
            Say (Shape_Kind'Image (K) & " " & Placement'Image (Where) & " " & Run_Kind'Image (What) & ": "
                 & Driver.Log.Image (Beat) & " beats");
            if Length (Failure) > 0 then
               Say ("  the decider failed: " & To_String (Failure));
            end if;
            for J in 1 .. Done_Runs loop
               Say ("  " & Ending'Image (Results (J).Final) & ": " & To_String (Results (J).Account)
                    & (if Length (Results (J).Tried) > 0 then " | tried: " & To_String (Results (J).Tried) else ""));
            end loop;
            declare
               N : constant Natural := Natural (P.Moves.Length);
               Shift, Turn : Real_Array (1 .. Natural'Max (1, N)) := [others => 0.0];
               Followed : Natural := 0;
               Counts : array (Driver.Action.Plants.Step_Outcome) of Natural := [others => 0];
            begin
               for R of P.Moves loop
                  Counts (R.Outcome) := Counts (R.Outcome) + 1;
                  if R.Outcome /= Driver.Action.Plants.Refused then
                     Followed := Followed + 1;
                     Shift (Followed) := abs (R.Reached.Translation - R.Asked.Translation);
                     Turn (Followed) := Angle (Transpose (R.Asked.Rotation) * R.Reached.Rotation);
                  end if;
               end loop;
               Say ("  arm moves:" & N'Image & ", reached" & Counts (Driver.Action.Plants.Reached)'Image
                    & ", short" & Counts (Driver.Action.Plants.Short)'Image & ", blocked"
                    & Counts (Driver.Action.Plants.Blocked)'Image & ", refused"
                    & Counts (Driver.Action.Plants.Refused)'Image);
               if Followed > 0 then
                  Say ("  asked against reached, position (units): median " & Img (Driver.Stats.Median (Shift (1 .. Followed)))
                       & ", largest " & Img (Largest (Shift (1 .. Followed)))
                       & "; turn (rad): median " & Img (Driver.Stats.Median (Turn (1 .. Followed)))
                       & ", largest " & Img (Largest (Turn (1 .. Followed))));
               end if;
               for R of P.Moves loop
                  if R.Outcome = Driver.Action.Plants.Refused then
                     Say ("  refused: " & To_String (R.Why));
                     exit;
                  end if;
               end loop;
            end;
         end;
      end;
   end Run;

begin
   for Where in Placement loop
      for K in Shape_Kind loop
         for What in Run_Kind loop
            Run (K, Where, What);
         end loop;
      end loop;
   end loop;
end Action_Matrix;
