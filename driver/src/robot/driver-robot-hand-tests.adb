with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Robot.Hand.Aims.Tests;
with Driver.Robot.Hand.Frames.Tests;
with Driver.Robot.Hand.Lobes.Tests;
with Driver.Robot.Hand.Lowering.Tests;
with Driver.Robot.Hand.Presses.Tests;
with Driver.Robot.Hand.Pressing.Tests;
with Driver.Robot.Hand.Selfsight.Tests;
with Driver.Robot.Hand.Shape.Tests;
with Driver.Robot.Hand.Slide.Tests;
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
      function Shows return Showing is (Something_New);
      Down_Pushes, Up_Pushes, Unseen, Longest : Natural := 0;
      Down_Answered, Up_Answered, Formed : Boolean := False;
   begin
      Sweep_Way (-1.0, 0.1, 0.1, 1, Push'Access, Shows'Access, Down_Pushes, Unseen, Longest, Formed, Down_Answered);
      Reading := 1.0;
      Asked := 0;
      Sweep_Way (1.0, 0.1, 0.1, 1, Push'Access, Shows'Access, Up_Pushes, Unseen, Longest, Formed, Up_Answered);
      Check (Down_Answered and then Down_Pushes = 6,
             "down from its upper end the closer was pushed" & Down_Pushes'Image & " times, not to its lower end"
             & " (0.9, 0.8, 0.6, 0.2, 0 and once more)");
      Check (not Up_Answered and then Up_Pushes = 1,
             "up from its upper end the closer was pushed" & Up_Pushes'Image & " times, not once");
   exception
      when Runaway =>
         Check (False, "a closer at its upper end was asked ever further up while its reading stayed there");
   end Upper_End;

   procedure Views_Never_Form is
      --  A12: the push is followed, and its view never forms (the body never
      --  came to rest within its readings' noise), so Shows says Not_Yet at
      --  every asking. The way ends after the askings the caller measured a
      --  view to need, unformed, with nothing more pushed. Without that end
      --  it asked for an hour; a float's worth of askings stands for that.
      Asked   : Natural := 0;
      Pushed  : Natural := 0;
      Runaway : exception;
      procedure Push (Offset : Real; Followed : out Boolean) is
         pragma Unreferenced (Offset);
      begin
         Pushed := Pushed + 1;
         Followed := True;
      end Push;
      function Shows return Showing is
      begin
         Asked := Asked + 1;
         if Asked > 10 * Real'Machine_Mantissa then
            raise Runaway;
         end if;
         return Not_Yet;
      end Shows;
      Pushes, Unseen, Longest : Natural := 0;
      Formed, Answered        : Boolean := True;
      Measured                : constant Positive := 7;
   begin
      Sweep_Way (-1.0, 1.0e-5, 1.0e-2, Measured, Push'Access, Shows'Access, Pushes, Unseen, Longest, Formed, Answered);
      Check (not Formed and then Pushes = 1 and then Pushed = 1 and then Asked = Measured and then Longest = Measured,
             "a push whose view never forms ended its way after" & Asked'Image & " askings and" & Pushed'Image
             & " pushes, not after the" & Measured'Image & " a view was measured to take, unformed");
   exception
      when Runaway =>
         Check (False, "a push whose view never forms is waited for without end, as A12's was for an hour");
   end Views_Never_Form;

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
      --  (b) nothing predicting it: fast, doubling from Least, and blocked by
      --      the step that meets the surface, about a doubling's number of
      --      steps (the owner's rule, 10-05: the most aggressive choice);
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
      Steps     : Descent_Steps;
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
      function Above return Heights is
        ((Tip => (if Predict then (Value => Tip - Predicted, Sigma => Sigma, Degrees_Of_Freedom => 0) else Unknown),
          Eye => Unknown, others => <>));
      procedure Press (From : Real) is
      begin
         Tip := From;
         Over := 0.0;
         Descend (Above'Access, Least, Lower'Access, Steps);
      end Press;
      Bound : constant Real := Real'Max (Sigma, Least);
   begin
      Predicted := 0.0;
      Predict := True;
      Press (0.2);
      Check (Over <= Bound, "(a) the press overshot the predicted contact by" & Over'Image & ", past" & Bound'Image);
      Check (Total (Steps) <= Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (0.2 / Least, 2.0)))
                              + 2 * Natural (Real'Ceiling (Z)) + 1
             and then Steps.Blind = 0 and then Steps.Fast > 0 and then Steps.Band > 0,
             "(a) the press took" & Steps.Fast'Image & " fast," & Steps.Band'Image & " banded and" & Steps.Blind'Image
             & " blind steps");
      Check (abs Tip <= Bound, "(d) the tip read at contact lies" & Real'Image (abs Tip) & " from the surface");
      Predict := False;
      Press (0.2);
      --  Doubling from Least, the step that meets the surface is the first
      --  whose sum passes the descent: some 2^k Least past it at most.
      Check (Over > 0.0 and then Over <= Last_Step,
             "(b) with nothing predicting the surface the press was not blocked by its step, overshooting by"
             & Over'Image);
      Check (Steps.Fast = 0 and then Steps.Band = 0 and then Steps.Blind = Total (Steps)
             and then Total (Steps) <= Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log
                                                                 (0.2 / Least + 1.0, 2.0))),
             "(b) with nothing predicting the surface the press took" & Steps.Fast'Image & " fast," & Steps.Band'Image
             & " banded and" & Steps.Blind'Image & " blind steps, not a doubling's worth");
      Predicted := -3.0 * Z * Sigma;
      Predict := True;
      Press (0.2);
      Check (Over > Bound and then Over <= Last_Step,
             "(c) an obstacle the prediction misses was met with an overshoot of" & Over'Image
             & ", not by the fast step past the bound");
      --  (e) the press that fixed the tip was stopped above the surface by something under it, so
      --  the prediction is a bound and the surface is 10 below where it says. The band ends, the steps
      --  double again, and the surface is met within a doubling's worth of steps more than (a) takes,
      --  not by creeping at the band's size to the end of the room (A17's second press).
      Predicted := 10.0;
      Predict := True;
      Press (10.2);
      Check (Over > 0.0 and then Over <= Last_Step, "(e) the surface 10 below the predicted contact was not met by a step");
      Check (Total (Steps) <= Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (10.2 / Least, 2.0)))
                              + 2 * Natural (Real'Ceiling (Z)) + Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (10.2 / Sigma, 2.0)))
                              + 3,
             "(e) the press took" & Steps.Fast'Image & " fast," & Steps.Band'Image & " banded and" & Steps.Blind'Image
             & " blind steps to reach a surface 10 below the prediction");
      Check (Steps.Blind > 0, "(e) the steps past the band were not doubling again");
   end Press_Overshoot;

   procedure Eye_Room_Caps_The_Steps is
      --  A hand lowered with nothing to stop it, as A16's third press was:
      --  the arm gives way to the doubling, and the tip is nowhere near the
      --  table when the steps pass the height of the eye above it. The eye is
      --  0.05 above the tip, the tip starts 0.2 above the table, the tool's
      --  Least is 0.001 and the eye's height is known to 0.001:
      --  (a) no step takes the eye nearer the table than Z sigma of its height,
      --      and the descent ends, Spent, when less than Least is left;
      --  (b) the steps double up to the last, cut to what is left: the cut
      --      step is counted, and there is one;
      --  (c) with the eye's height unknown nothing caps the steps, which go on
      --      as long as the arm gives way.
      Least  : constant Real := 0.001;
      Sigma  : constant Real := 0.001;
      Z      : constant Real := Threshold (Scalar_Gate);
      Tip    : Real := 0.0;
      Known_Eye : Boolean := True;
      Lowest : Real := Real'Last;   --  the eye's height at its lowest
      Steps  : Descent_Steps;
      Pushes : Natural := 0;
      Slides : Boolean := False;   --  the hand slides: its steps are reached and lower nothing
      procedure Lower (By : Real; Reached : out Boolean) is
      begin
         Pushes := Pushes + 1;
         if not Slides then
            Tip := Tip - By;
         end if;
         Lowest := Real'Min (Lowest, Tip + 0.05);
         Reached := Pushes < 40;   --  the arm gives way until the test ends it
      end Lower;
      function Above return Heights is
        ((Tip => Unknown,
          Eye => (if Known_Eye then (Value => Tip + 0.05, Sigma => Sigma, Degrees_Of_Freedom => 0) else Unknown),
          others => <>));
   begin
      Tip := 0.2;
      Descend (Above'Access, Least, Lower'Access, Steps);
      Check (Steps.Spent, "the eye went on lowering past the table: " & Pushes'Image & " pushes, the eye at"
             & Real'Image (Lowest));
      Check (Lowest >= Z * Sigma - 1.0e-12, "a step took the eye to" & Real'Image (Lowest) & ", nearer the table than Z sigma");
      Check (Lowest - Z * Sigma < Least, "the descent ended with" & Real'Image (Lowest - Z * Sigma) & " of the eye's room unused");
      Check (Steps.Capped = 1 and then Total (Steps) = Pushes and then Steps.Blind = Total (Steps),
             "the last step was not the one cut to the room left: capped" & Steps.Capped'Image & " of" & Pushes'Image);
      Known_Eye := False;
      Pushes := 0;
      Tip := 0.2;
      Descend (Above'Access, Least, Lower'Access, Steps);
      Check (not Steps.Spent and then Steps.Capped = 0 and then Pushes = 40,
             "with the eye's height unknown the steps were stopped: " & Pushes'Image & " pushes");
      --  (d) steps the arm reaches, none of which lowers the eye (a hand that slides along the table
      --  it met, as A17's did for 6000 beats): the eye's room stays, and the steps end on the schedule
      --  that covers it, doubling from Least, a step cut to the room and one that finds none: 10 here.
      Known_Eye := True;
      Pushes := 0;
      Tip := 0.2;
      Slides := True;
      Descend (Above'Access, Least, Lower'Access, Steps);
      Check (Steps.Spent and then Pushes <= 10 and then Pushes >= 8,
             "steps that did not lower the eye went on for" & Pushes'Image & " pushes, not the schedule's 10");
   end Eye_Room_Caps_The_Steps;

   procedure Stalled_Descent is
      --  A hand lowered onto a table its tip is predicted 0.34 above (sigma 0.04), as A17's third press was: the
      --  arm follows every step, the hand goes down with each until its tip is on the table, and the steps after
      --  that are followed by the arm and take the hand nowhere. The descent ends at the first of them, a press
      --  and not a descent spent: the room the eye has is nearly all there, and a descent that ran on to spend it
      --  would be the press thrown away.
      Least  : constant Real := 0.001;
      Sigma  : constant Real := 0.04;
      Tip    : Real := 0.34;   --  its height above the table
      Pushes : Natural := 0;
      Steps  : Descent_Steps;
      Stalled : Boolean := False;   --  the last step was followed and took the hand nowhere
      procedure Lower (By : Real; Reached : out Boolean) is
      begin
         Pushes := Pushes + 1;
         Reached := True;   --  the arm follows every step
         if Tip - By >= 0.0 then
            Tip := Tip - By;
         else
            Stalled := True;
         end if;
      end Lower;
      function Above return Heights is
        ((Tip     => (Value => Tip, Sigma => Sigma, Degrees_Of_Freedom => 0),
          Eye     => (Value => Tip + 2.0, Sigma => 0.001, Degrees_Of_Freedom => 0),
          Stalled => Stalled));
   begin
      Descend (Above'Access, Least, Lower'Access, Steps);
      Check (Steps.Stalled and then not Steps.Spent, "the descent of a hand that stopped going down ended stalled:"
             & Steps.Stalled'Image & ", spent:" & Steps.Spent'Image);
      Check (Pushes = Total (Steps) and then Pushes < 40, "the descent took" & Pushes'Image & " pushes");
      Check (Tip < 0.2, "the hand was stopped at" & Real'Image (Tip) & " above the table, the tip not pressed onto it"
             & " as far as the free steps took it");
   end Stalled_Descent;

   procedure Held_Closer is
      --  A closer asked back to its open reading with a finger on the table (A17): it stays where it stood, and
      --  moves at once when the hand stands clear of the table by anything.
      --  (a) held: one raise, the least, frees it, and it is not raised again;
      --  (b) jammed by something else: one raise, the least, and no more, since the closer did not move;
      --  (c) already there: no raise;
      --  (d) a hand that cannot be raised: the closer is left as it is.
      Open    : constant Real := 1.0;
      Reading : Real;
      Clear   : Real;      --  how far the hand stands off the table
      Jammed  : Boolean;
      Can     : Boolean := True;
      Asked   : Natural;
      Raises  : Natural;
      Raised  : Real;
      function There return Boolean is (abs (Reading - Open) < 1.0e-9);
      function Now return Real is (Reading);
      function Moves (Before, After : Real) return Boolean is (abs (After - Before) > 1.0e-9);
      procedure Ask is
      begin
         Asked := Asked + 1;
         if not Jammed and then Clear > 0.0 then
            Reading := Open;
         end if;
      end Ask;
      procedure Raise_Hand (First : Boolean; Done : out Boolean) is
      begin
         Done := Can;
         if Can then
            Raised := (if First then 0.05 else 2.0 * Raised);
            Clear := Clear + Raised;
         end if;
      end Raise_Hand;
      procedure Case_Of (Start : Real; Is_Jammed, Raisable : Boolean) is
      begin
         Reading := Start;
         Clear := 0.0;
         Jammed := Is_Jammed;
         Can := Raisable;
         Asked := 0;
         Raised := 0.0;
         Free_Closer (There'Access, Now'Access, Moves'Access, Ask'Access, Raise_Hand'Access, Raises);
      end Case_Of;
   begin
      Case_Of (0.6, Is_Jammed => False, Raisable => True);
      Check (Raises = 1 and then There and then Asked = 2, "(a) a closer held by the table was raised" & Raises'Image
             & " times and asked" & Asked'Image & " times, not raised once and asked twice");
      Case_Of (0.6, Is_Jammed => True, Raisable => True);
      Check (Raises = 1 and then not There, "(b) a closer jammed by something else was raised" & Raises'Image & " times, not once");
      Case_Of (Open, Is_Jammed => False, Raisable => True);
      Check (Raises = 0 and then Asked = 1, "(c) a closer already there was raised" & Raises'Image & " times");
      Case_Of (0.6, Is_Jammed => False, Raisable => False);
      Check (Raises = 0 and then not There, "(d) a hand that cannot be raised was raised" & Raises'Image & " times");
   end Held_Closer;

   procedure Poses_From_Raises is
      --  A hand measured from a reloaded body file: the eye has seen the closer's readings from no pose, or one,
      --  and a deviation needs two (A25h logged "0 poses, two are needed" at every round). The arm raises the
      --  eye, the least that shows and twice that after it, and the pose is asked after a beat at a time:
      --  (a) the eye has the poses already: no raise;
      --  (b) the first pose is the eye's own at the readings, the next needs a raise that shows (the first
      --      raise is too small: it is raised twice), and its frame is kept some beats after;
      --  (c) more poses are more raises, each twice the one before;
      --  (d) a frame that is not kept in as long as a view takes to form: the raise is made again, each asking
      --      ends at that wait and not at the end of time, and it ends where the arm can raise no more;
      --  (e) an eye that cannot be raised: no pose, and it says so.
      Wanted    : Positive := 2;
      Have      : Natural;
      Shows_At  : Real;      --  the raise from which the eye's picture changes enough to keep a frame
      Lag       : Natural;   --  beats from a raise to its kept frame
      Since     : Natural;   --  beats since the last raise
      Total     : Real;      --  how far the eye has been raised
      Last      : Real;      --  the last raise
      Limit     : Natural := 0;   --  how many raises the arm can make
      Made      : Natural;
      Pending   : Boolean;   --  a raise that shows has been made and its frame is not kept yet
      Raises    : Natural;
      Reached   : Boolean;
      Asked     : Natural;
      function Poses return Natural is
      begin
         Asked := Asked + 1;
         Since := Since + 1;
         if Pending and then Since >= Lag then
            Have := Have + 1;
            Pending := False;
         end if;
         return Have;
      end Poses;
      procedure Raise_Eye (First : Boolean; Raised : out Boolean) is
      begin
         Raised := Made < Limit;
         if Raised then
            Made := Made + 1;
            Last := (if First then 0.01 else 2.0 * Last);
            Total := Total + Last;
            Since := 0;
            Pending := Last >= Shows_At;
         end if;
      end Raise_Eye;
      procedure Case_Of (Poses_Seen : Natural; Needed : Positive; Raise_Shows_At : Real; Kept_After : Natural;
                         Wait : Positive; Can_Raise : Natural) is
      begin
         Wanted := Needed;
         Have := Poses_Seen;
         Shows_At := Raise_Shows_At;
         Lag := Kept_After;
         Since := 0;
         Total := 0.0;
         Last := 0.0;
         Limit := Can_Raise;
         Made := 0;
         Pending := False;
         Asked := 0;
         Gather_Poses (Wanted, Wait, Poses'Access, Raise_Eye'Access, Raises, Reached);
      end Case_Of;
   begin
      Case_Of (2, 2, 0.0, 3, 100, 10);
      Check (Reached and then Raises = 0 and then Asked = 2, "(a) an eye with the poses was raised" & Raises'Image
             & " times and asked" & Asked'Image & " times");
      Case_Of (1, 2, 0.015, 3, 10, 10);
      Check (Reached and then Raises = 2 and then abs (Total - 0.03) < 1.0e-12 and then Have = 2,
             "(b) the first raise was too small to show: raised" & Raises'Image & " times by" & Real'Image (Total)
             & ", reached:" & Reached'Image & ", poses" & Have'Image);
      Case_Of (1, 4, 0.0, 3, 100, 10);
      Check (Reached and then Raises = 3 and then abs (Total - 0.07) < 1.0e-12,
             "(c) three more poses took" & Raises'Image & " raises by" & Real'Image (Total) & ", not 3 raises by 0.07");
      Case_Of (1, 2, 0.0, 50, 10, 3);
      Check (not Reached and then Raises = 3 and then Asked <= 3 * 11 + 3,
             "(d) a frame never kept in the wait was asked after" & Asked'Image & " beats with" & Raises'Image
             & " raises, reached:" & Reached'Image & ", not three waits of ten beats");
      Case_Of (1, 2, 0.0, 3, 100, 0);
      Check (not Reached and then Raises = 0 and then Have = 1, "(e) an eye that cannot be raised was raised"
             & Raises'Image & " times and reached its poses:" & Reached'Image);
   end Poses_From_Raises;

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

   procedure Unswept_Closer_Says_Why is
      --  A closer the hand watches in its arm's own eye and has swept nothing
      --  of makes no hand; the hands' account (what the boot logs after it
      --  has pressed, or not) says so, with why, for the closer and its
      --  channel, not nothing: A14's hand phase ended on "measured" and
      --  nothing else, and no one could tell a hand that was not found from
      --  a press that was not made.
      M    : Driver.Robot.Model;
      H    : Hands;
      O    : Observation;
      Sent : Driver.Commands.Command;
      function Mentions (Text, Part : String) return Boolean is (Ada.Strings.Fixed.Index (Text, Part) > 0);
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
      Check (Hand_Count (H) = 0, "a hand was found of a closer nothing was swept of");
      declare
         Account : constant String := Describe (H);
      begin
         Check (Mentions (Account, "closer group 2") and then Mentions (Account, "its own: no hand")
                and then Mentions (Account, "channel 1: its two ends were not both seen still"),
                "the account of a closer that was not swept does not say so: """ & Account & """");
      end;
   end Unswept_Closer_Says_Why;

   procedure Dropped_Hand is
      --  A29: the estimators read the roles again between two held beats of the decider (a recompute of the
      --  heavier estimates), Find_Pairs found no hand of the closer, and the press in progress, which held an
      --  index into the hands, read one that was gone: CONSTRAINT_ERROR, and the boot with it. Measure is run
      --  over a body whose closer is re-read as an arm at beat Drop (the next beat of the hand drops its hand,
      --  as the estimators did), for every Drop from the first beat on: it ends, with the hand pressed or not,
      --  and never raises.
      Rays : constant Vec3 := Unit ([0.04, 0.03, 0.13]);

      procedure Run (Drop : Natural; Finished, Raised, Dropped : out Boolean) is
         M       : Driver.Robot.Model;
         H       : Hands;
         Sent    : Driver.Commands.Command;
         Arm_At  : Real_Array (1 .. 3) := [0.1, 0.2, 0.3];
         Pinch   : Real_Array (1 .. 1) := [0.04];
         Done    : Boolean := False with Atomic;
         Died    : Boolean := False with Atomic;

         function Observed (Beat : Natural) return Observation is
            O : Observation;
         begin
            O.Beat := Driver.Clock.Beat (Beat);
            O.Readings.Append (Arm_At);
            O.Readings.Append (Pinch);
            O.Images.Append (Driver.Images.Create (4, 3, [1 .. 36 => Driver.Bytes.Byte'First]));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            return O;
         end Observed;
      begin
         Dropped := False;
         --  The body's graph is the test's: an arm of three channels carrying the eye, and its closer; a body that
         --  measured nothing would derive another from the stream.
         M.From_File (Stored_Graph) := True;
         Driver.Commands.Set_Target (Sent, 1, Arm_At);
         Driver.Commands.Set_Target (Sent, 2, Pinch);
         Driver.Robot.Observe (M, Observed (0), Sent);
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
         declare
            Rows : Sight_Rows (1 .. 1);
         begin
            for W in Opening loop
               Rows (1) (W) :=
                 (Known  => True,
                  Pixel  => (U => 0.0, V => 0.0),
                  Ray    => (Origin    => (Mean => Zero3, Covariance => [others => [others => 0.0]]),
                             Direction => (Unit_Vector => Rays, Sigma => 2.5e-4)),
                  Spread => 0.0,
                  Pitch  => 0.0);
            end loop;
            Adopt (H, 2, 1, 1, [1 => 0.04], [1 => 0.0], Rows);
         end;
         Observe (H, M, Observed (0), Sent);
         Check (Hand_Count (H) = 1, "the hand to drop was not given");
         Check (Exists (H, 1) and then not Exists (H, 2), "a hand is not said to be there, or one that is not is");
         declare
            task Decider;
            task body Decider is
            begin
               Measure (H, M);
               Done := True;
            exception
               when others =>
                  Driver.Beats.Release;
                  Died := True;
                  Done := True;
            end Decider;
         begin
            for B in 1 .. 2_000 loop
               exit when Done;
               declare
                  O       : constant Observation := Observed (B);
                  Took    : Boolean := False;
                  Pending : Driver.Commands.Command;
               begin
                  if B = Drop then
                     M.Graph.Roles.Replace_Element (2, Arm);
                     M.Graph.Arm_Of.Replace_Element (2, 2);
                     M.Graph.Arms.Append (2);
                     Dropped := True;
                  end if;
                  Driver.Robot.Observe (M, O, Sent);
                  Observe (H, M, O, Sent);
                  loop
                     Driver.Beats.Offer (O.Beat, O, Sent, Took);
                     exit when Took or else Done;
                     delay 0.0;
                  end loop;
                  exit when not Took;
                  Driver.Beats.Await (Pending);
                  --  The robot reaches what it was sent.
                  if Driver.Commands.Has_Target (Pending, 1) then
                     Driver.Commands.Set_Target (Sent, 1, Driver.Commands.Target (Pending, 1));
                     Arm_At := Driver.Commands.Target (Sent, 1);
                  end if;
                  if Driver.Commands.Has_Target (Pending, 2) then
                     Driver.Commands.Set_Target (Sent, 2, Driver.Commands.Target (Pending, 2));
                     Pinch := Driver.Commands.Target (Sent, 2);
                  end if;
               end;
            end loop;
            if not Done then
               abort Decider;
            end if;
         end;
         Finished := Done;
         Raised := Died;
      end Run;

      Finished, Raised, Dropped : Boolean;
      Ran_Out : Natural := 0;
   begin
      Run (0, Finished, Raised, Dropped);
      Check (Finished and then not Raised, "the decider did not end on the undisturbed body (no hand dropped)");
      for Drop in 1 .. 60 loop
         Run (Drop, Finished, Raised, Dropped);
         if not Dropped then
            Ran_Out := Ran_Out + 1;
         end if;
         Check (Finished and then not Raised,
                "the decider " & (if Raised then "raised" else "did not end") & " when the hand was dropped at beat"
                & Drop'Image);
      end loop;
      Check (Ran_Out < 60, "the decider ended before any drop was made, so nothing was tested");
   end Dropped_Hand;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.measure.gone", "a hand dropped by the estimators between two beats of a press "
                             & "makes the decider raise", Dropped_Hand'Access);
      Driver.Tests.Register ("hand.unmeasured", "an unmeasured body makes the hand fail or invent a hand",
                             Unmeasured_Body'Access);
      Driver.Tests.Register ("hand.measure.account", "a closer no hand is made of ends the measurement without a "
                             & "word of why", Unswept_Closer_Says_Why'Access);
      Driver.Tests.Register ("hand.measure.end", "a closer at an end of its travel is asked ever further past it",
                             Upper_End'Access);
      Driver.Tests.Register ("hand.measure.unformed", "a push whose view never forms is waited for without end (A12)",
                             Views_Never_Form'Access);
      Driver.Tests.Register ("hand.measure.press", "a press overshoots the contact by more than its prediction admits, "
                             & "or creeps when nothing predicts it", Press_Overshoot'Access);
      Driver.Tests.Register ("hand.measure.room", "a step lowers the eye below the table its arm's own eye saw",
                             Eye_Room_Caps_The_Steps'Access);
      Driver.Tests.Register ("hand.measure.stall", "a hand the arm follows and that stops going down is pushed on to the "
                             & "end of its room, or is not a press", Stalled_Descent'Access);
      Driver.Tests.Register ("hand.measure.held", "a closer held by the table is not freed by raising the hand, or a closer "
                             & "nothing holds is raised for", Held_Closer'Access);
      Driver.Tests.Register ("hand.measure.poses", "an eye that has seen a closer's readings from too few poses is raised "
                             & "for more, or one that has enough is, or the wait for a frame is not bounded",
                             Poses_From_Raises'Access);
      Driver.Tests.Register ("hand.measure.roles", "a group the body re-read as an arm is swept as a closer",
                             Roles_Re_Read'Access);
      Driver.Robot.Hand.Frames.Tests.Register;
      Driver.Robot.Hand.Aims.Tests.Register;
      Driver.Robot.Hand.Views.Tests.Register;
      Driver.Robot.Hand.Selfsight.Tests.Register;
      Driver.Robot.Hand.Lobes.Tests.Register;
      Driver.Robot.Hand.Sweep.Tests.Register;
      Driver.Robot.Hand.Lowering.Tests.Register;
      Driver.Robot.Hand.Presses.Tests.Register;
      Driver.Robot.Hand.Pressing.Tests.Register;
      Driver.Robot.Hand.Slide.Tests.Register;
      Driver.Robot.Hand.Touch.Tests.Register;
      Driver.Robot.Hand.Tips.Tests.Register;
      Driver.Robot.Hand.Shape.Tests.Register;
   end Register;

end Driver.Robot.Hand.Tests;
