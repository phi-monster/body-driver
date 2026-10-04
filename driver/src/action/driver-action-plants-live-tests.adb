with Ada.Environment_Variables;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Text_IO;
with Driver.Robot.Motion;
with Driver.Uncertain;
with Driver.Numerics;
with Driver.Action.Contact;
use Ada.Numerics.Long_Elementary_Functions;
use Driver.Uncertain;
use Driver.Numerics;
use Driver.Numerics.Arrays;
use Driver.Action.Contact;
with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Tests;
with Driver.World;

package body Driver.Action.Plants.Live.Tests is

   use Driver.Tests;

   --  A body that has measured nothing yet: one group of two readings that
   --  takes commands, one eye showing a grey picture. Every beat the robot
   --  reaches what it was sent.
   type Run_Kind is (Change_Height, Touch_With_Grasper);

   function Want_For (Kind : Run_Kind) return Want is
      Settles : Ending_Set := [others => False];
   begin
      Settles (Settled) := True;
      case Kind is
         when Change_Height =>
            return (Kind => Change, Until_Endings => Settles, Max_Steps => 0, Eye => Any_Eye, Anyway => False,
                    Thing => 1, Quantity => 1, Increase => True);
         when Touch_With_Grasper =>
            declare
               W : Want (Interval);
            begin
               W.Until_Endings := Settles;
               W.Constraints.Append (Constraint'(Subject  => (Kind => Role_Operand, The_Role => Grasper),
                                                 Relation => Touching,
                                                 Object   => (Kind => Thing_Operand, Thing => 1),
                                                 Step     => Unspecified, Strength => Unspecified, Must => False));
               return W;
            end;
      end case;
   end Want_For;

   procedure Run_On_Unmeasured_Body (Kind : Run_Kind; Final : out Ending; Tried : out Unbounded_String;
                                     Check_Ok : out Boolean; Finished_Run : out Boolean; Beats : out Natural;
                                     Moved : out Boolean)
   is
      M : aliased Driver.Robot.Model;
      H : aliased Driver.Robot.Hand.Hands;
      S : aliased Driver.World.Scene;
      Wanted : constant Want := Want_For (Kind);
      Got : Result;
      Finished : Boolean := False with Atomic;
      Verdict_Ok : Boolean := True with Atomic;

      task Decider;
      task body Decider is
         C : Context (M'Access, H'Access, S'Access);
         procedure Gate is
            V : constant Verdict := Check (C, Wanted);
         begin
            Verdict_Ok := V.Ok;
         end Gate;
      begin
         Driver.Beats.Within_A_Beat (Gate'Access);
         Execute (C, Wanted, Got);
         Finished := True;
      exception
         when others =>
            Driver.Beats.Release;
            Finished := True;
      end Decider;

      Side  : constant := 8;
      Bytes : constant := 3 * Side * Side;
      Grey  : constant Driver.Bytes.Byte_Array (1 .. Bytes) := [others => 128];
      Sent : Driver.Commands.Command;
      Now  : Real_Array (1 .. 2) := [0.0, 0.0];
      Bound : constant := 400;
   begin
      Beats := 0;
      Moved := False;
      for B in 0 .. Bound loop
         exit when Finished;
         declare
            O       : Driver.Observations.Observation;
            Took    : Boolean := False;
            Pending : Driver.Commands.Command;
         begin
            O.Beat := Driver.Clock.Beat (B);
            O.Images.Append (Driver.Images.Create (8, 8, Grey));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Readings.Append (Now);
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            if B = 0 then
               Driver.Commands.Set_Target (Sent, 1, Now);
            end if;
            Driver.Robot.Observe (M, O, Sent);
            Driver.Robot.Hand.Observe (H, M, O, Sent);
            Driver.World.Observe (S, M, H, O, Sent);
            loop
               Driver.Beats.Offer (O.Beat, O, Sent, Took);
               exit when Took or else Finished;
               delay 0.0;
            end loop;
            exit when not Took;
            Driver.Beats.Await (Pending);
            Moved := Moved or else not Driver.Commands.Is_Hold (Pending);
            if Driver.Commands.Has_Target (Pending, 1) then
               Driver.Commands.Set_Target (Sent, 1, Driver.Commands.Target (Pending, 1));
               Now := Driver.Commands.Target (Sent, 1);
            end if;
            Beats := B + 1;
         end;
      end loop;
      if not Finished then
         abort Decider;
      end if;
      Finished_Run := Finished;
      Final := Got.Final;
      Tried := Got.Tried;
      Check_Ok := Verdict_Ok;
   end Run_On_Unmeasured_Body;

   procedure Unmeasured_Body_Refuses is
   begin
      for Kind in Run_Kind loop
         declare
            Final : Ending;
            Tried : Unbounded_String;
            Ok, Done, Moved : Boolean;
            Beats : Natural;
         begin
            Run_On_Unmeasured_Body (Kind, Final, Tried, Ok, Done, Beats, Moved);
            Check (not Moved, "the action layer sent targets to a body that has measured nothing ("
                   & Run_Kind'Image (Kind) & ")");
            Check (Done, "the action layer does not finish on a body that has measured nothing ("
                   & Run_Kind'Image (Kind) & ")");
            Check (not Ok, "the gates pass a want on a body that has measured nothing (" & Run_Kind'Image (Kind) & ")");
            Check (Final = Refused and then Length (Tried) > 0,
                   "a want on a body that has measured nothing is not refused with what was tried ("
                   & Run_Kind'Image (Kind) & "): " & Ending'Image (Final));
         end;
      end loop;
   end Unmeasured_Body_Refuses;

   procedure Probe_Body is
      M   : Driver.Robot.Model;
      Ok  : Boolean;
      Why : Unbounded_String;
      O   : Driver.Observations.Observation;
   begin
      if not Ada.Environment_Variables.Exists ("BD_BODY") then
         return;
      end if;
      Driver.Robot.Load_Body (M, Ada.Environment_Variables.Value ("BD_BODY"), Ok, Why);
      Ada.Text_IO.Put_Line ("load " & Ok'Image & ": " & To_String (Why));
      for G in 1 .. Driver.Robot.Group_Count (M) loop
         O.Readings.Append (Real_Array'(1 .. Driver.Robot.Group_Size (M, Driver.Robot.Group_Id (G)) => 0.0));
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      end loop;
      declare
         T  : constant Pose_Estimate := Driver.Robot.Tool_Pose (M, 1, O);
         U  : constant Direction_Estimate := Driver.Robot.Up (M);
         R  : constant Mat3 := T.Pose.Rotation;
         function Img (V : Vec3) return String is (V (1)'Image & V (2)'Image & V (3)'Image);
      begin
         Ada.Text_IO.Put_Line ("arms" & Driver.Robot.Arm_Count (M)'Image & " tool at" & Img (T.Pose.Translation)
                               & " |t|" & Real'(abs T.Pose.Translation)'Image);
         Ada.Text_IO.Put_Line ("tool x" & Img ([R (1, 1), R (2, 1), R (3, 1)]) & " y" & Img ([R (1, 2), R (2, 2), R (3, 2)])
                               & " z" & Img ([R (1, 3), R (2, 3), R (3, 3)]));
         Ada.Text_IO.Put_Line ("up" & Img (U.Unit_Vector) & " z.up" & Real'([R (1, 3), R (2, 3), R (3, 3)] * U.Unit_Vector)'Image
                               & " pos sigma" & Sqrt (T.Position_Covariance (1, 1))'Image
                               & " rot sigma" & Sqrt (T.Rotation_Covariance (1, 1))'Image);
         declare
            E1, E2 : Vec3;
            N : constant Vec3 := U.Unit_Vector;
         begin
            Plane_Basis (N, E1, E2);
            for Dir in 1 .. 6 loop
               declare
                  D : constant Vec3 := (case Dir is when 1 => N, when 2 => -N, when 3 => E1, when 4 => -E1,
                                         when 5 => E2, when others => -E2);
               begin
                  for K in 0 .. 8 loop
                     declare
                        Dist : constant Real := 0.005 * 2.0 ** K;
                        Goal : constant Driver.Robot.Motion.Pose_Goal :=
                          (Pose => (Rotation => R, Translation => T.Pose.Translation + Dist * D), Position_Only => False);
                        P : constant Driver.Robot.Motion.Plan := Driver.Robot.Motion.Plan_Reach (M, 1, O, Goal);
                        use type Driver.Robot.Motion.Plan_Status;
                     begin
                        Ada.Text_IO.Put_Line ("dir" & Dir'Image & " dist" & Dist'Image & " "
                                              & Driver.Robot.Motion.Status (P)'Image
                                              & (if Driver.Robot.Motion.Status (P) /= Driver.Robot.Motion.Planned
                                                 then " " & Driver.Robot.Motion.Why (P) else ""));
                        exit when Driver.Robot.Motion.Status (P) /= Driver.Robot.Motion.Planned;
                     end;
                  end loop;
               end;
            end loop;
            --  Turning the tool about up and about its own axes.
            for Axis in 1 .. 4 loop
               for K in 0 .. 6 loop
                  declare
                     A_Ax : constant Vec3 := (case Axis is when 1 => N, when 2 => [R (1, 1), R (2, 1), R (3, 1)],
                                              when 3 => [R (1, 2), R (2, 2), R (3, 2)],
                                              when others => [R (1, 3), R (2, 3), R (3, 3)]);
                     Ang : constant Real := 0.05 * 2.0 ** K;
                     Goal : constant Driver.Robot.Motion.Pose_Goal :=
                       (Pose => (Rotation => Exp (Ang * A_Ax) * R, Translation => T.Pose.Translation),
                        Position_Only => False);
                     P : constant Driver.Robot.Motion.Plan := Driver.Robot.Motion.Plan_Reach (M, 1, O, Goal);
                     use type Driver.Robot.Motion.Plan_Status;
                  begin
                     Ada.Text_IO.Put_Line ("turn axis" & Axis'Image & " angle" & Ang'Image & " "
                                           & Driver.Robot.Motion.Status (P)'Image);
                     exit when Driver.Robot.Motion.Status (P) /= Driver.Robot.Motion.Planned;
                  end;
               end loop;
            end loop;
         end;
         for J in 1 .. 6 loop
            declare
               O2 : Driver.Observations.Observation := O;
               Q  : Real_Array := O.Readings (1);
            begin
               Q (J) := 0.1;
               O2.Readings.Replace_Element (1, Q);
               declare
                  T2 : constant Pose_Estimate := Driver.Robot.Tool_Pose (M, 1, O2);
               begin
                  Ada.Text_IO.Put_Line ("joint" & J'Image & " +0.1 moves the tool by"
                                        & Real'(abs (T2.Pose.Translation - T.Pose.Translation))'Image
                                        & " and turns it by" & Angle (Transpose (R) * T2.Pose.Rotation)'Image);
               end;
            end;
         end loop;
         for E in 1 .. Driver.Robot.Eye_Count (M) loop
            declare
               P : constant Pose_Estimate := Driver.Robot.Eye_Pose (M, Driver.Robot.Eye_Id (E), O);
            begin
               Ada.Text_IO.Put_Line ("eye" & E'Image & " mount " & Driver.Robot.Eye_Mount (M, Driver.Robot.Eye_Id (E)).Kind'Image
                                     & " at" & Img (P.Pose.Translation));
            end;
         end loop;
         for G in 1 .. Driver.Robot.Group_Count (M) loop
            for C in 1 .. Driver.Robot.Group_Size (M, Driver.Robot.Group_Id (G)) loop
               declare
                  V : constant Estimate := Driver.Robot.Visible_Step (M, Driver.Robot.Group_Id (G), C);
               begin
                  if Known (V) then
                     Ada.Text_IO.Put_Line ("group" & G'Image & " channel" & C'Image & " visible step" & V.Value'Image
                                           & " role " & Driver.Robot.Role (M, Driver.Robot.Group_Id (G))'Image);
                  end if;
               end;
            end loop;
         end loop;
      end;
   end Probe_Body;

   procedure Register is
   begin
      Register ("action.probe.body", "probe", Probe_Body'Access);
      Register ("action.live.unmeasured", "the action layer over the real lower layers deadlocks with the main loop, "
                & "raises, or moves on a body that has measured nothing", Unmeasured_Body_Refuses'Access);
   end Register;

end Driver.Action.Plants.Live.Tests;
