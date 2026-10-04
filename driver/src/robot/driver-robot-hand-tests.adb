with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Robot.Hand.Aims.Tests;
with Driver.Robot.Hand.Frames.Tests;
with Driver.Robot.Hand.Lobes.Tests;
with Driver.Robot.Hand.Presses.Tests;
with Driver.Robot.Hand.Tips.Tests;
with Driver.Robot.Hand.Sweep.Tests;
with Driver.Robot.Hand.Touch.Tests;
with Driver.Robot.Hand.Views.Tests;
with Driver.Tests;

package body Driver.Robot.Hand.Tests is

   use Driver.Tests;

   procedure Unmeasured_Body is
      --  From the first beat the main loop feeds the hand a body that knows
      --  nothing about itself yet: an arm-like group, a closer-like group
      --  and an eye. Nothing may be found, and nothing may fail.
      M : Driver.Robot.Model;
      H : Hands;
      O : Observation;
   begin
      O.Readings.Append (Real_Array'[0.1, 0.2, 0.3]);
      O.Readings.Append (Real_Array'[1 => 0.04]);
      O.Images.Append (Driver.Images.Create (4, 3, [1 .. 36 => Driver.Bytes.Byte'First]));
      for Beat in 1 .. 3 loop
         O.Beat := Driver.Clock.Beat (Beat);
         Observe (H, M, O, Driver.Commands.Hold);
      end loop;
      Check (Hand_Count (H) = 0 and then Describe (H) = "", "an unmeasured body was given a hand");
   end Unmeasured_Body;

   procedure Upper_End is
      --  A closer resting at the upper end of its travel, 0 to 1, swept from
      --  there by steps of 0.1 while its eye says every view would extend an
      --  end (as A10's did while its reading stayed at 1.0): down, it follows
      --  until its lower end; up, it does not follow the first push, the
      --  level at which the downward sweep answered, and its doubling stops
      --  there. A sweep that doubled while the eye said so would ask without
      --  end; a float's worth of doublings stands for that here.
      Reading : Real := 1.0;
      Asked   : Natural := 0;
      Runaway : exception;
      procedure Push (Offset : Real; Followed : out Boolean) is
         To : constant Real := Real'Max (0.0, Real'Min (1.0, 1.0 + Offset));
      begin
         Asked := Asked + 1;
         if Asked > Real'Machine_Mantissa then
            raise Runaway;
         end if;
         Followed := To /= Reading;
         Reading := To;
      end Push;
      function Extends return Boolean is (True);
      Down_Pushes, Up_Pushes : Natural := 0;
      Down_Answered, Up_Answered : Boolean := False;
   begin
      Sweep_Way (-1.0, 0.1, Push'Access, Extends'Access, Down_Pushes, Down_Answered);
      Reading := 1.0;
      Asked := 0;
      Sweep_Way (1.0, 0.1, Push'Access, Extends'Access, Up_Pushes, Up_Answered);
      Check (Down_Answered and then Down_Pushes = 6,
             "down from its upper end the closer was pushed" & Down_Pushes'Image & " times, not to its lower end"
             & " (0.9, 0.8, 0.6, 0.2, 0 and once more)");
      Check (not Up_Answered and then Up_Pushes = 1,
             "up from its upper end the closer was pushed" & Up_Pushes'Image & " times, not once");
   exception
      when Runaway =>
         Check (False, "a closer at its upper end was asked ever further up while its reading stayed there");
   end Upper_End;

   procedure Press_Overshoot is
      --  A tip pressed onto a stiff surface by a stiff position controller,
      --  the surface ten times as stiff: a step that would end below the
      --  surface is cut short at it, the tip settling where the two push back
      --  equally, a tenth of the way in of the step's overshoot past the
      --  surface. From 0.2 above it, the tool's Least 0.001:
      --  (a) the contact predicted where it is, its sigma 0.002: the overshoot
      --      at contact is at most the larger of the two, within about as many
      --      steps as doubling from Least takes to cover the descent, and the
      --      band's twice Z;
      --  (b) nothing predicting it: at most Least;
      --  (c) predicted 3 Z sigma too low, as when an obstacle the eyes did not
      --      see lies under the tip: the fast part meets it, and its overshoot
      --      is that step's, past the bound: the known failure, stated here;
      --  (d) where it is predicted, the tip read at contact lies within the
      --      bound of the surface.
      --  The old descent, doubling until blocked, overshoots by about half the
      --  descent and reads the tip that much in.
      Least     : constant Real := 0.001;
      Sigma     : constant Real := 0.002;
      Stiffer   : constant Real := 10.0;
      Z         : constant Real := Threshold (Scalar_Gate);
      Tip, Over : Real := 0.0;
      Predicted : Real := 0.0;
      Predict   : Boolean := True;
      Last_Step : Real := 0.0;
      Steps     : Natural;
      procedure Lower (By : Real; Reached : out Boolean) is
         Target : constant Real := Tip - By;
      begin
         Last_Step := By;
         Reached := Target >= 0.0;
         if Reached then
            Tip := Target;
         else
            Over := -Target;
            Tip := Target / (1.0 + Stiffer);
         end if;
      end Lower;
      function Gap return Estimate is
        (if Predict then (Value => Tip - Predicted, Sigma => Sigma, Degrees_Of_Freedom => 0) else Unknown);
      procedure Press (From : Real) is
      begin
         Tip := From;
         Over := 0.0;
         Descend (Gap'Access, Least, Lower'Access, Steps);
      end Press;
      Bound : constant Real := Real'Max (Sigma, Least);
   begin
      Predicted := 0.0;
      Predict := True;
      Press (0.2);
      Check (Over <= Bound, "(a) the press overshot the predicted contact by" & Over'Image & ", past" & Bound'Image);
      Check (Steps <= Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (0.2 / Least, 2.0))) + 2 * Natural (Real'Ceiling (Z)) + 1,
             "(a) the press took" & Steps'Image & " steps");
      Check (abs Tip <= Bound, "(d) the tip read at contact lies" & Real'Image (abs Tip) & " from the surface");
      Predict := False;
      Press (0.2);
      Check (Over <= Least, "(b) with nothing predicting the surface the press overshot it by" & Over'Image);
      Predicted := -3.0 * Z * Sigma;
      Predict := True;
      Press (0.2);
      Check (Over > Bound and then Over <= Last_Step,
             "(c) an obstacle the prediction misses was met with an overshoot of" & Over'Image
             & ", not by the fast step past the bound");
   end Press_Overshoot;

   procedure Roles_Re_Read is
      --  A group the body first takes for a closer of an arm whose eye sees
      --  it, then re-reads as an arm of its own (as A10's boot did with its
      --  group 3): the hand takes it for a closer to sweep, then never again.
      M    : Driver.Robot.Model;
      H    : Hands;
      O    : Observation;
      Sent : Driver.Commands.Command;
   begin
      O.Beat := 1;
      O.Readings.Append (Real_Array'[0.1, 0.2, 0.3]);
      O.Readings.Append (Real_Array'[1 => 0.04]);
      O.Images.Append (Driver.Images.Create (4, 3, [1 .. 36 => Driver.Bytes.Byte'First]));
      O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      Driver.Commands.Set_Target (Sent, 1, [0.1, 0.2, 0.3]);
      Driver.Commands.Set_Target (Sent, 2, [0.04]);
      Driver.Robot.Observe (M, O, Sent);
      M.Graph.Roles.Clear;
      M.Graph.Roles.Append (Arm);
      M.Graph.Roles.Append (Closer);
      M.Graph.Arm_Of.Clear;
      M.Graph.Arm_Of.Append (1);
      M.Graph.Arm_Of.Append (1);
      M.Graph.Arms.Clear;
      M.Graph.Arms.Append (1);
      M.Graph.Mounts.Clear;
      M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => 1));
      Observe (H, M, O, Sent);
      Check (Sweepable (H, M, 2), "a closer its own arm's eye sees is not taken for one to sweep");
      M.Graph.Roles.Replace_Element (2, Arm);
      M.Graph.Arm_Of.Replace_Element (2, 2);
      M.Graph.Arms.Append (2);
      Check (not Sweepable (H, M, 2), "a group the body re-read as an arm is still taken for a closer to sweep");
      O.Beat := 2;
      Driver.Robot.Observe (M, O, Sent);
      Observe (H, M, O, Sent);
      Check (not Sweepable (H, M, 2), "a group the body re-read as an arm is taken for a closer again");
   end Roles_Re_Read;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.unmeasured", "an unmeasured body makes the hand fail or invent a hand",
                             Unmeasured_Body'Access);
      Driver.Tests.Register ("hand.measure.end", "a closer at an end of its travel is asked ever further past it",
                             Upper_End'Access);
      Driver.Tests.Register ("hand.measure.press", "a press overshoots the contact by more than its prediction admits",
                             Press_Overshoot'Access);
      Driver.Tests.Register ("hand.measure.roles", "a group the body re-read as an arm is swept as a closer",
                             Roles_Re_Read'Access);
      Driver.Robot.Hand.Frames.Tests.Register;
      Driver.Robot.Hand.Aims.Tests.Register;
      Driver.Robot.Hand.Views.Tests.Register;
      Driver.Robot.Hand.Lobes.Tests.Register;
      Driver.Robot.Hand.Sweep.Tests.Register;
      Driver.Robot.Hand.Presses.Tests.Register;
      Driver.Robot.Hand.Touch.Tests.Register;
      Driver.Robot.Hand.Tips.Tests.Register;
   end Register;

end Driver.Robot.Hand.Tests;
