with Driver.Conventions;

package body Driver.Robot.Hand.Lowering is

   use Driver.Numerics.Arrays;

   function Down (Pose : Rigid; Point, Into : Vec3) return Real is (Into * (Pose * Point));
   --  How far along Into a point of the tool frame stands with the tool at Pose.

   function Pushes (T : Track) return Natural is (T.Count);

   procedure Judge
     (T      : in out Track;
      From   : Pose_Estimate;
      Target : Pose_Estimate;
      To     : Pose_Estimate;
      Into   : Vec3;
      Where  : Points;
      Joints : Joint_Push;
      Said   : out Judgment)
   is
      --  A share of free pushes has a scatter from three on.
      Enough      : constant Boolean := T.Count > 2;
      --  What the push asked, and whether the shortfall of it is something the one test of motion sees: the floor
      --  under both measures.
      Seen        : constant Boolean := Joints.Asked and then Joints.Seen;
      Lowering    : Real := 0.0;        --  how far the push asked the hand to go down: its points' mean
      Reach       : Real := 0.0;        --  and how far it asked any point to go, any way
      Any_Stopped : Boolean := False;   --  some point stopped short as free pushes do not
      Worst       : Judgment;           --  of the points, the one that fell shortest by share
      Stall       : Judgment;           --  and, of those that stopped, the same
      Biggest     : Real := 0.0;        --  the largest share, either way, of any point
      --  The readings: the share of the ask's length they stopped short of the target by, and whether
      --  that is more than free pushes stop short by.
      Joint_Share : constant Real :=
        (if Joints.Asked and then Joints.Length > 0.0 then Joints.Short / Joints.Length else 0.0);
      Joint_Stops : constant Boolean :=
        Enough and then Seen and then Joint_Share > Driver.Conventions.Z * T.Joint_Largest;
   begin
      Said := (others => <>);
      for P of Where loop
         Lowering := Lowering + (Down (Target.Pose, P, Into) - Down (From.Pose, P, Into));
         Reach := Real'Max (Reach, abs (Target.Pose * P - From.Pose * P));
      end loop;
      Lowering := Lowering / Real (Natural'Max (1, Where'Length));
      if Lowering <= 0.0 then
         --  A retreat or a hold: the next descent is compared with itself.
         T := (others => <>);
         return;
      end if;
      if not Joints.Asked then
         return;
      end if;
      --  A point's shortfall is a share of what the push asked of the hand, not of what it asked of the point: a
      --  push that turns the tool about its origin asks the origin nothing down, and a share of that is no number.
      --  Nor of how far the push asked the hand to go down: A22's aims turned the hand about the way down, asking
      --  every point of it 2e-16 down by the rounding of the turn and delivering 4.7e-8 up, a share of 2.3e8
      --  that stood as the largest any free push fell short by and left the descent after it with nothing to
      --  be compared with. The farthest any point was asked to go, any way, is no less than the mean of what
      --  they were asked down (so never nothing here), and for a push straight down it is what was asked down.
      for I in Where'Range loop
         declare
            Start : constant Real := Down (From.Pose, Where (I), Into);
            Ask   : constant Real := Down (Target.Pose, Where (I), Into) - Start;
            Got   : constant Real := Down (To.Pose, Where (I), Into) - Start;
            Share : constant Real := (Ask - Got) / Reach;
            Here  : constant Judgment :=
              (Result => Lowered, Point => I, Asked => Ask, Went => Got, Share => Share, Free => T.Largest,
               others => <>);
         begin
            Biggest := Real'Max (Biggest, abs Share);
            if I = Where'First or else Share > Worst.Share then
               Worst := Here;
            end if;
            --  A point asked to go down that stopped, by a larger share than any free push fell short by: one
            --  that was asked up and went further up has stopped nothing.
            if Enough and then Seen and then Ask > 0.0 and then Share > Driver.Conventions.Z * T.Largest then
               if not Any_Stopped or else Share > Stall.Share then
                  Stall := Here;
               end if;
               Any_Stopped := True;
            end if;
         end;
      end loop;
      Said := (if Any_Stopped then Stall else Worst);
      Said.Point_Stalled := Any_Stopped;
      Said.Joint_Share := Joint_Share;
      Said.Joint_Free := T.Joint_Largest;
      Said.Joint_Stalled := Joint_Stops;
      if Any_Stopped or else Joint_Stops then
         Said.Result := Stalled;
         return;
      end if;
      Said.Result := (if Enough then Lowered else Too_Few);
      T.Count := T.Count + 1;
      T.Largest := Real'Max (T.Largest, Biggest);
      T.Joint_Largest := Real'Max (T.Joint_Largest, Joint_Share);
   end Judge;

end Driver.Robot.Hand.Lowering;
