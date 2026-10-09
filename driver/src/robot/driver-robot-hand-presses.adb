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
      Retargeted : Boolean := False)
   is
   begin
      Found := False;
      Press := (others => <>);
      case W.State is
         when Free =>
            if Blocked then
               W.State := Driven;
               W.Commands := 0;
               W.Approach := (if W.Stood then Moved_Into (W.Last_Still, Tool) else (others => <>));
            elsif Still then
               W.Stood := True;
               W.Last_Still := Tool;
            end if;
         when Driven =>
            --  The verdict stands until another push begins: that is the let-go. Or until another command takes
            --  effect: a hold at the readings the block left asks the arm nothing it can see, begins no push
            --  for the step tracker, and is a let-go all the same (A35's first press: the arm still easing back
            --  at the hold, the retreat the next command).
            if Retargeted then
               W.Commands := W.Commands + 1;
            end if;
            if not Blocked or else W.Commands > 0 then
               W.State := Let_Go;
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
                         Beat     => Beat);
               W.State := Settling;
            end if;
         when Settling =>
            --  Whatever the let-go was judged, it is not a press.
            if Still and then not Blocked then
               W.State := Free;
               W.Stood := True;
               W.Last_Still := Tool;
            end if;
         when Free | Driven =>
            null;
      end case;
   end Observe;

end Driver.Robot.Hand.Presses;
