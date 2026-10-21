with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Robot.Hand.Presses is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Moved_Into (From, To : Pose_Estimate) return Direction_Estimate is
      --  The way the tool went from one pose to the other, in its frame at the
      --  second; unknown when the move is not significant against the poses'
      --  own uncertainty.
      Unknown_Direction : Direction_Estimate;
      Step : constant Point_Estimate := (Mean => To.Pose.Translation - From.Pose.Translation,
                                         Covariance => From.Position_Covariance + To.Position_Covariance);
   begin
      if not Known (Step) or else not Significant (Position (From), Position (To)) then
         return Unknown_Direction;
      end if;
      declare
         Length : constant Real := abs Step.Mean;
         U      : constant Vec3 := (1.0 / Length) * Step.Mean;
         --  Across the move, the positions' uncertainty turns it by that much
         --  over its length.
         Across : constant Real := Sqrt ((Step.Covariance (1, 1) + Step.Covariance (2, 2) + Step.Covariance (3, 3)
                                          - U * (Step.Covariance * U)) / 2.0);
      begin
         return (Unit_Vector => Transpose (To.Pose.Rotation) * U, Sigma => Across / Length);
      end;
   end Moved_Into;

   --  The way the tool went by a displacement Step, in its frame at To; unknown when the step is not significant
   --  against its own uncertainty along it. One fit puts both of a step's ends, and what it does not know of where the
   --  tool stands moves them alike: added as two poses' own, their uncertainties made every short step of s42's
   --  descents (servo seed 6 of the fake robot, 10-17) no move at all, and all ten of its presses were given to no
   --  lobe for want of the way they pressed.
   function Moved_Along (Step : Point_Estimate; To : Pose_Estimate) return Direction_Estimate is
      Unknown_Direction : Direction_Estimate;
      Length : constant Real := (if Known (Step) then abs Step.Mean else 0.0);
   begin
      if not (Length > 0.0) then
         return Unknown_Direction;
      end if;
      declare
         U      : constant Vec3 := (1.0 / Length) * Step.Mean;
         Along  : constant Real := Sqrt (Real'Max (0.0, U * (Step.Covariance * U)));
         Across : constant Real := Sqrt (Real'Max (0.0, (Step.Covariance (1, 1) + Step.Covariance (2, 2)
                                                         + Step.Covariance (3, 3) - U * (Step.Covariance * U)) / 2.0));
      begin
         if not Driver.Uncertain.Significant (Length, Along) then
            return Unknown_Direction;
         end if;
         return (Unit_Vector => Transpose (To.Pose.Rotation) * U, Sigma => Across / Length);
      end;
   end Moved_Along;

   --  The block's first beat: the tool there, and the way it came from where it last stood still: by the step between
   --  the arm's readings there and here when Step_Of says what it is, else by the two poses.
   procedure Drive (W : in out Watcher; Tool : Pose_Estimate; Arm : Real_Array; Step_Of : Step_Function) is
   begin
      W.State := Driven;
      W.Commands := 0;
      W.Block_At := Tool;
      W.Approach :=
        (if not W.Stood then (others => <>)
         elsif Step_Of /= null and then not W.Last_Arm.Is_Empty and then Arm'Length = W.Last_Arm.Element'Length
         then Moved_Along (Step_Of (W.Last_Arm.Element, Arm), Tool)
         else Moved_Into (W.Last_Still, Tool));
   end Drive;

   procedure Met (W : in out Watcher; Tool : Pose_Estimate; Arm, From : Real_Array := [1 .. 0 => 0.0];
                  Step_Of : Step_Function := null) is
   begin
      if W.State = Free then
         if From'Length > 0 and then From'Length = Arm'Length then
            W.Last_Arm := Reading_Holders.To_Holder (From);
            W.Stood := True;
         end if;
         Drive (W, Tool, Arm, Step_Of);
      end if;
   end Met;

   procedure Observe
     (W       : in out Watcher;
      Beat    : Driver.Clock.Beat;
      Blocked : Boolean;
      Pushing : Boolean;
      Still   : Boolean;
      Tool    : Pose_Estimate;
      Arm     : Real_Array;
      Closer  : Real_Array;
      Found   : out Boolean;
      Press   : out Event;
      Retargeted : Boolean := False;
      Asked   : Pose_Estimate := (others => <>);
      Step_Of : Step_Function := null;
      Held    : Boolean := False)
   is
      --  The command takes the tool back nearer where it stood before the push than where the block left it: a
      --  retreat, which a hold at the readings the block left (the arm easing back a little since) never is.
      function Away return Boolean is
        (W.Stood and then Known (Position (Asked))
         and then abs (Asked.Pose.Translation - W.Last_Still.Pose.Translation)
                    < abs (Asked.Pose.Translation - W.Block_At.Pose.Translation));
   begin
      Found := False;
      Press := (others => <>);
      case W.State is
         when Free =>
            if Blocked then
               Drive (W, Tool, Arm, Step_Of);
            elsif Still and then not Pushing then
               --  Where the tool stood still while free, no push under way: a push that met the table holds the tool
               --  still against it, as far as its pose tells, for as long as the push takes to be judged blocked, and
               --  that pose is the block's, not where the press came from (A90: every press's way was unknown, given
               --  to no lobe, and no tip was fitted).
               W.Stood := True;
               W.Last_Still := Tool;
               W.Last_Arm := Reading_Holders.To_Holder (Arm);
               W.Closer_Still := Reading_Holders.To_Holder (Closer);
            end if;
         when Driven =>
            --  The verdict stands until another push begins: that is the let-go. Or until another command takes
            --  effect: a hold at the readings the block left asks the arm nothing it can see, begins no push
            --  for the step tracker, and is a let-go all the same (A35's first press: the arm still easing back
            --  at the hold, the retreat the next command).
            if Retargeted and then Away then
               --  Taken away before letting go: no press, and free again at the next rest unblocked.
               W.State := Settling;
            else
               if Retargeted then
                  W.Commands := W.Commands + 1;
               end if;
               --  Or the command already holds the arm where it stands (Held): a push stopped where it was read
               --  the moment what it met was felt was held there before its verdict came, the one command after the
               --  block, and no later one changed it. z7 (servo seed 7 of the fake robot, 10-18): its press at open
               --  stopped so at beat 7060, was judged blocked at 7063, the hold the measure then sent was the stop's,
               --  and the watcher waited for a let-go 7068 beats, the stream's length; its next press, 12,000 more.
               if not Blocked or else W.Commands > 0 or else Held then
                  W.State := Let_Go;
               end if;
            end if;
         when Let_Go =>
            if Retargeted then
               W.Commands := W.Commands + 1;
            end if;
         when Settling =>
            null;
      end case;
      --  The rest is after the let-go has ended, not at the beat it begins: a
      --  body that answers a push late is still at that beat, and its pose is
      --  the one under the push.
      case W.State is
         when Let_Go =>
            if W.Commands > 1 then
               --  A second command took effect before the hand rested: the rest that comes is the retreat's, where the
               --  hand is not on what it pressed. Not a press.
               W.State := Settling;
            elsif Still and then not Pushing then
               Found := True;
               Press := (Tool     => Tool,
                         Arm      => Reading_Holders.To_Holder (Arm),
                         Approach => W.Approach,
                         Closer   => Reading_Holders.To_Holder (Closer),
                         Before   => (if W.Stood then W.Closer_Still else Reading_Holders.Empty_Holder),
                         Beat     => Beat);
               W.State := Settling;
            end if;
         when Settling =>
            --  Whatever the let-go was judged, it is not a press.
            if Still and then not Blocked then
               W.State := Free;
               W.Stood := True;
               W.Last_Still := Tool;
               W.Last_Arm := Reading_Holders.To_Holder (Arm);
               W.Closer_Still := Reading_Holders.To_Holder (Closer);
            end if;
         when Free | Driven =>
            null;
      end case;
   end Observe;

end Driver.Robot.Hand.Presses;
