with Ada.Containers.Vectors;
with Driver.Beats;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Hand.Pressing;
with Driver.Robot.Hand.Selfsight;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;

--  The decider that measures the hands. Every closer channel is swept from
--  where it is to both ends of its travel, pushing further with doubling
--  steps from the smallest step the eyes see for as long as each step shows
--  the eye something the last end did not; the estimators find the lobes from
--  the views that leaves (Hand.Observe). Then every lobe's tip is pressed at
--  both openings onto the table its arm's eye saw, in the arm's own frame
--  (Driver.Robot.Hand.Pressing): aimed by a
--  turn about the eye, first straight along its line of sight, then tilted
--  away from the other lobes by doubling multiples of the angle between the
--  hand's lines of sight, until a press stops agreeing with the others or
--  cannot be reached; a press descends with doubling steps from the smallest
--  move that tells from the arm's own noise until the arm is blocked, lets go
--  so the hand rests, and lifts back. The presses are found in the stream by
--  the estimators, like the views. The decider reads the models only between
--  Driver.Beats.Next and Send, holding the body for that beat, as
--  Driver.Robot.Motion does.

separate (Driver.Robot.Hand)
procedure Measure (H : in out Hands; M : in out Model) is

   use type Driver.Robot.Motion.Plan_Status;
   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Robot.Hand.Lobes.Placing;

   procedure Hold_Beat (Read : access procedure (O : Observation)) is
      procedure During is
      begin
         if Read /= null then
            Read (Driver.Beats.Latest.all);
         end if;
      end During;
   begin
      Driver.Beats.Within_A_Beat (During'Access);
   end Hold_Beat;
   --  One held beat, and what the models say at it.

   function Own_Pair (G : Group_Id) return Natural is
   begin
      for I in H.Data.Pairs.First_Index .. H.Data.Pairs.Last_Index loop
         if H.Data.Pairs (I).Group = G then
            return I;
         end if;
      end loop;
      return 0;
   end Own_Pair;
   --  Read inside a held beat.

   --  What the decider holds from one beat to the next may be gone at the next: the estimators read the roles
   --  again between two held beats (a recompute of the heavier estimates), Find_Pairs finds no hand of a closer
   --  that is no longer one, and an index into the hands is out of range or another hand's. A29's press in
   --  progress read the hand it measured at Found (2) when none was left. A hand is held here by its closer
   --  group, found again at each beat it is read, and a press holds its record only to know it again (Present).
   function Index_Of (Group : Group_Id) return Natural is
   begin
      for I in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         if H.Data.Found (I).Group = Group then
            return Natural (I);
         end if;
      end loop;
      return 0;
   end Index_Of;
   --  The hand made of that closer group now, 0 if none. Read inside a held beat.

   function Present (Id : Hand_Id; R : Hand_Record) return Boolean is
     (Id <= H.Data.Found.Last_Index and then H.Data.Found (Id).Group = R.Group and then H.Data.Found (Id).Arm = R.Arm);
   --  The hand at Id is the hand R was read from. Read inside a held beat, as every index into the hands is.

   package Group_Lists is new Ada.Containers.Vectors (Positive, Group_Id);

   --  One part of one hand, from its closer group: its closer swept, or its lobes pressed, by the body's roles as
   --  they are when each begins. The closers are swept one after another: a sweep reads the views its own eye has
   --  of its fingers, and another arm moving in that eye's view spoils them (A59, both swept at once: no hand was
   --  made of the closer whose eye saw the other arm move; A58 lost the ends of a view it had placed the lobes
   --  from). The hands are pressed at once (Driver.Beats.At_Once, a hand a lane): each moves only its own arm and
   --  closer, reads and changes the models only in its held beats, and keeps what follows here, its own, from one
   --  beat to the next.
   type Hand_Part is (Sweep_Part, Press_Part);
   procedure One_Hand (Closer : Group_Id; Part : Hand_Part) is

      Lost : Boolean := False;
      --  Set by a read that found the hand under measure gone: the press in progress ends, the arm let go and taken
      --  back, and nothing more of that hand is pressed. Cleared where the next hand begins.

      Aim_Short : Boolean := False;
      --  Set by a press that did not begin because the arm did not reach the pose it was aimed at (blocked by
      --  something of its own, or short of it): a tilt this arm cannot make from where it stands, and not a press that
      --  stopped short of the table. Set afresh by each press.

      Learned_An_End : Boolean := False;
      --  Set by a press whose aim stopped on something of the arm's own that moved an end the arm showed
      --  (Driver.Robot.Motion.Note_Stopped): the same aim, planned again, is turned about the way down past it. Set
      --  afresh by each press.

      procedure Move_Group (G : Group_Id; Target : Real_Array; Followed : out Boolean) is
         --  One step of the group to Target, settled, and one more still beat so
         --  the eye's view there has two frames. Followed: the body judged that
         --  the step moved the group along its ask (not Blocked: pushing did not
         --  move it at all).
         Command : Driver.Commands.Command;
         Report  : Driver.Robot.Motion.Step_Report;
      begin
         Driver.Commands.Set_Target (Command, G, Target);
         Driver.Robot.Motion.Step (M, Command, Report);
         Hold_Beat (null);
         Followed := Report.Outcome /= Driver.Robot.Motion.Blocked;
      end Move_Group;

      procedure Move_Group (G : Group_Id; Target : Real_Array) is
         Followed : Boolean;
      begin
         Move_Group (G, Target, Followed);
      end Move_Group;

      procedure Move_Channel (G : Group_Id; Channel : Positive; To : Real; Followed : out Boolean) is
         --  One channel to To, the group's other channels as they read now.
         Now : Driver.Robot.Hand.Views.Reading_Holders.Holder;
         procedure Read_Now (O : Observation) is
         begin
            Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (G));
         end Read_Now;
      begin
         Hold_Beat (Read_Now'Access);
         declare
            Target : Real_Array := Now.Element;
         begin
            Target (Target'First + Channel - 1) := To;
            Move_Group (G, Target, Followed);
         end;
      end Move_Channel;

      procedure Move_Channel (G : Group_Id; Channel : Positive; To : Real) is
         Followed : Boolean;
      begin
         Move_Channel (G, Channel, To, Followed);
      end Move_Channel;

      --  One channel back to a reading the closer has been at, and the hand raised away from the table if it stays
      --  short of it. A closer that stays short of a reading it has been at is held by something, and a finger
      --  resting on the table is held by it: it cannot slide along what it presses on. A17's, asked back to its
      --  open reading 1.0 at the end of its sweep, stood between 0.59 and 0.686 for seventy beats, and reached 1.0
      --  in four once the aim had lifted the hand off the table. The hand is raised along the way up, in steps that
      --  double from the least move of its tool, while each raise sets the closer moving, until the closer
      --  arrives; a raise after which the closer has not moved was not what held it, so the hand is not raised
      --  again, nor higher than the eye stands above the table (past that the table is not what holds it).
      procedure Return_Channel (G : Group_Id; C : Positive; To : Real) is
         Reading : Real := To;        --  the channel's reading now
         Arrived : Boolean := True;   --  within its noise of To, or of a step no eye tells from it
         Placed  : Boolean := False;  --  the closer is one an eye on an arm watches
         Arm     : Arm_Id := 1;
         Eye     : Eye_Id := 1;
         Plan    : Driver.Robot.Motion.Plan;
         Report  : Driver.Robot.Motion.Step_Report;
         First   : Boolean := True;   --  the raise now is the first
         By      : Real := 0.0;       --  the raise now, doubling
         Raised  : Real := 0.0;       --  and the raises so far
         Room    : Estimate := Unknown;   --  how high the eye stands above the table, along the way up
         Noise   : Real := 0.0;
         Raises  : Natural;

         procedure Read_Closer (O : Observation) is
            P : constant Natural := Own_Pair (G);
            R : constant Real_Array := O.Readings.Element (G);
         begin
            Reading := R (R'First + C - 1);
            Noise := Sqrt (2.0) * Reading_Noise (M, G, C);
            Arrived := not Significant (Reading - To, Noise)
              or else (Known (Visible_Step (M, G, C)) and then abs (Reading - To) < Visible_Step (M, G, C).Value);
            Placed := P > 0;
            if Placed then
               Arm := H.Data.Pairs (P).Arm;
               Eye := H.Data.Pairs (P).Eye;
            end if;
         end Read_Closer;

         procedure Read_Raise (O : Observation) is
            --  The arm and its eye are those of the closer's pair now, not those of the beat the closer was read at: the
            --  estimators may have dropped the pair between the two.
            P : constant Natural := Own_Pair (G);
         begin
            Room := Unknown;
            if P = 0 then
               return;
            end if;
            Arm := H.Data.Pairs (P).Arm;
            Eye := H.Data.Pairs (P).Eye;
            declare
               Up : constant Direction_Estimate := Up_In_Arm (M, Arm);
            begin
               if Up.Sigma = Real'Last or else Eye_Mount (M, Eye).Kind /= Arm_Carried then
                  return;
               end if;
               By := (if First then Driver.Robot.Hand.Pressing.Least_Push (M, Arm, O) else 2.0 * By);
               Plan := Driver.Robot.Hand.Pressing.Lowered (M, Arm, O, Up.Unit_Vector, By);
               Room := Driver.Robot.Hand.Pressing.Gap
                 (M, Arm, O, (Mean => Eye_In_Tool (M, Eye, O).Pose.Translation, Covariance => [others => [others => 0.0]]),
                  Table_In_Arm (M, Arm), -Up.Unit_Vector);
            end;
         end Read_Raise;

         function Is_There return Boolean is
         begin
            Hold_Beat (Read_Closer'Access);
            return Arrived or else not Placed;
         end Is_There;

         function Reads return Real is (Reading);

         function Moved_By (Before, After : Real) return Boolean is (Significant (After - Before, Noise));

         procedure Ask_Again is
         begin
            Move_Channel (G, C, To);
         end Ask_Again;

         --  The hand raised along the way up, not past the eye's height above the table.
         procedure Raise_Once (Is_First : Boolean; Done : out Boolean) is
         begin
            Done := False;
            First := Is_First;
            Hold_Beat (Read_Raise'Access);
            if not Known (Room) or else Raised + By > Room.Value
              or else Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned
            then
               return;
            end if;
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image & ", asked back to "
                             & Driver.Log.Image (To, 6) & ", reads " & Driver.Log.Image (Reading, 6) & ": it is held; the hand"
                             & " is raised by " & Driver.Log.Image (By, 4) & " (" & Driver.Log.Image (Raised, 4) & " before) of the "
                             & Driver.Log.Image (Room.Value, 4) & " the eye stands above the table");
            Driver.Robot.Motion.Follow (M, Plan, Report);
            Done := Report.Outcome = Driver.Robot.Motion.Reached;
            if Done then
               Raised := Raised + By;
            end if;
         end Raise_Once;
      begin
         Free_Closer (Is_There'Access, Reads'Access, Moved_By'Access, Ask_Again'Access, Raise_Once'Access, Raises);
      end Return_Channel;

      --  How long a push's view took to form, the longest of every push this
      --  decider made: a view waits at most that long. Before any view formed,
      --  as many beats as the stream had when the sweep began: no wait is longer
      --  than all the waiting so far (Driver.Robot.Steps waits so).
      Longest_Formed : Natural := 0;

      --  The arm moves a closer's own eye so that it sees the closer's readings as they are now from Wanted poses of
      --  the rest of the body, which a deviation of the robot from its surroundings is taken over (Selfsight). A hand
      --  is measured from its own arm's motion and not from what the boot did before it: the boot's sweeps gave A22's
      --  closers dozens of poses at their start readings, and a reloaded body file gives none (A25h logged "0 poses,
      --  two are needed" at every round). The eye is raised along the way up the body measured (Up_In_Arm), heights
      --  above the table: first by the least move whose readings an eye can see (a plan's last readings, tried by the
      --  body's one test of motion before anything moves), then by twice that, and so on, until the eye has the poses
      --  or cannot be raised further.
      procedure Gather_Own_Poses (G : Group_Id; Wanted : Positive; Reached : out Boolean) is
         Placed : Boolean := False;   --  the closer is one an eye on an arm watches
         Arm    : Arm_Id := 1;
         Eye    : Eye_Id := 1;
         Have   : Natural := 0;       --  the poses its eye has seen its readings from
         Began  : Natural := 0;       --  the beats the stream had
         Plan   : Driver.Robot.Motion.Plan;
         Ready  : Boolean := False;   --  the raise is planned and its readings show
         Why    : Ada.Strings.Unbounded.Unbounded_String;   --  when it is not, why
         First  : Boolean := True;    --  the raise now is the first
         By     : Real := 0.0;        --  the raise now, doubling
         Report : Driver.Robot.Motion.Step_Report;
         Raises : Natural;

         procedure Read_Poses (O : Observation) is
            P : constant Natural := Own_Pair (G);
         begin
            Began := Natural (O.Beat);
            Placed := P > 0;
            Have := 0;
            if Placed then
               Arm := H.Data.Pairs (P).Arm;
               Eye := H.Data.Pairs (P).Eye;
               Have := Sweeps.Poses (H.Data.Pairs (P).Sweep, O.Readings.Element (G));
            end if;
         end Read_Poses;

         function Poses_Now return Natural is
         begin
            Hold_Beat (Read_Poses'Access);
            return Have;
         end Poses_Now;

         procedure Plan_Raise (O : Observation) is
            --  The arm and its eye are those of the closer's pair now (see Read_Raise).
            P : constant Natural := Own_Pair (G);
         begin
            Ready := False;
            Why := Ada.Strings.Unbounded.To_Unbounded_String ("the closer's eye and arm are no longer a pair of the hand");
            if P = 0 then
               return;
            end if;
            Arm := H.Data.Pairs (P).Arm;
            Eye := H.Data.Pairs (P).Eye;
            declare
               Up    : constant Direction_Estimate := Up_In_Arm (M, Arm);
               Group : constant Group_Id := Arm_Group (M, Arm);
               Now   : constant Real_Array := O.Readings.Element (Group);
            begin
               Why := Ada.Strings.Unbounded.To_Unbounded_String ("where up is, in the arm's frame, is not measured");
               if Up.Sigma = Real'Last then
                  return;
               end if;
               By := (if First then Driver.Robot.Hand.Pressing.Least_Push (M, Arm, O) else 2.0 * By);
               Why := Ada.Strings.Unbounded.To_Unbounded_String
                 ("no raise within the arm's reach moves its readings by what an eye sees");
               for Doubling in 1 .. Real'Machine_Mantissa loop
                  --  A raise is made for the poses it gives the eye, whatever its turn: its
                  --  plan reaches the place, not the turn (A68 and A69: arm 2's raises
                  --  with the turn held were refused for a turn the model left 8.4e-4 rad
                  --  off, the closer had no second pose at its low reading, and no hand was
                  --  made of it).
                  Plan := Driver.Robot.Hand.Pressing.Lowered (M, Arm, O, Up.Unit_Vector, By, Turn_Free => True);
                  if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
                     Why := Ada.Strings.Unbounded.To_Unbounded_String (Driver.Robot.Motion.Why (Plan));
                     exit;
                  end if;
                  declare
                     Goal : constant Real_Array := Driver.Robot.Motion.Last_Readings (Plan);
                     Step : constant Real_Array := [for I in Goal'Range => Goal (I) - Now (Now'First + I - Goal'First)];
                  begin
                     if Driver.Robot.Channels.Visible (M, Group, Step) then
                        Ready := True;
                        return;
                     end if;
                  end;
                  By := 2.0 * By;
               end loop;
            end;
         end Plan_Raise;

         procedure Raise_Eye (Is_First : Boolean; Raised : out Boolean) is
         begin
            Raised := False;
            First := Is_First;
            Hold_Beat (Plan_Raise'Access);
            if not Ready then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & ": its eye" & Eye'Image & " on arm" & Arm'Image
                                & " cannot be raised: " & Ada.Strings.Unbounded.To_String (Why));
               return;
            end if;
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & ": its eye" & Eye'Image & " on arm" & Arm'Image
                             & " has seen its readings from" & Have'Image & " poses of the rest of the body, and"
                             & Natural'Image (Driver.Robot.Hand.Selfsight.Needed) & " are needed to tell the robot from its surroundings;"
                             & " the arm raises the eye by " & Driver.Log.Image (By, 4)
                             & (if Is_First then ", the least that its readings show" else ", twice the raise before"));
            Driver.Robot.Motion.Follow (M, Plan, Report);
            Raised := Report.Outcome = Driver.Robot.Motion.Reached;
         end Raise_Eye;
      begin
         Hold_Beat (Read_Poses'Access);
         Reached := Placed and then Have >= Wanted;
         if not Placed or else Reached then
            return;
         end if;
         Driver.Robot.Hand.Gather_Poses
           (Wanted, Positive'Max (1, (if Longest_Formed > 0 then Longest_Formed else Began)), Poses_Now'Access,
            Raise_Eye'Access, Raises, Reached);
         Hold_Beat (Read_Poses'Access);
         Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & ": after" & Raises'Image & " raises its eye has seen"
                          & " its readings from" & Have'Image & " poses, of the" & Wanted'Image & " asked"
                          & (if Reached then "" else "; it cannot be raised further, or its picture did not rest to keep a frame"));
      end Gather_Own_Poses;

      --  A channel whose ends the eye has seen and whose lobes were not placed gets more poses while more poses can
      --  place them: the eye has seen the readings from fewer than a deviation needs (Unlocated), or the changed pixels
      --  do not fall in two groups by how much they vary over the poses (Unplaced, Unseparated). The poses are asked to
      --  double, and the estimators place the lobes again once they have (Driver.Robot.Hand.Sweep).
      --  The poses are gathered at the end of the travel the eye has seen from fewer poses, the closer brought there
      --  first: a separation needs poses at both ends, and gathering more where the closer happened to stop gave A70's
      --  arm 2 159 poses at its open end and none at its closed one, and no hand.
      procedure Top_Up_Poses (G : Group_Id; C : Positive) is
         Status : Sweeps.Progress := Sweeps.Measured;
         Helps  : Boolean := False;   --  more poses could place the lobes
         Have   : Natural := 0;
         Fewer  : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  the readings at the end with fewer poses
         There  : Boolean := False;   --  the closer stands at that end
         procedure Read_Status (O : Observation) is
            P : constant Natural := Own_Pair (G);
         begin
            Status := Sweeps.Measured;
            Helps := False;
            There := True;
            if P > 0 and then C <= Sweeps.Channels (H.Data.Pairs (P).Sweep) then
               declare
                  S : Sweeps.State renames H.Data.Pairs (P).Sweep;
               begin
                  Status := Sweeps.Status (S, C);
                  Have := Sweeps.Poses (S, O.Readings.Element (G));
                  Helps := Status = Sweeps.Unlocated
                    or else (Status = Sweeps.Unplaced
                             and then Sweeps.Located_Of (S, C).How = Driver.Robot.Hand.Lobes.Unseparated);
                  if Helps and then Sweeps.Has_Ends (S, C) then
                     declare
                        Low    : constant Real_Array := Sweeps.Low_Closer (S, C);
                        High   : constant Real_Array := Sweeps.High_Closer (S, C);
                        Now    : constant Real_Array := O.Readings.Element (G);
                        At_Low : constant Boolean := Sweeps.Poses (S, Low) <= Sweeps.Poses (S, High);
                        To_Low, To_High : Real := 0.0;
                     begin
                        for I in Now'Range loop
                           To_Low := To_Low + (Now (I) - Low (Low'First + I - Now'First)) ** 2;
                           To_High := To_High + (Now (I) - High (High'First + I - Now'First)) ** 2;
                        end loop;
                        Fewer := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (if At_Low then Low else High);
                        Have := Sweeps.Poses (S, Fewer.Element);
                        --  The closer stands at the end it is nearer.
                        There := At_Low = (To_Low <= To_High);
                     end;
                  end if;
               end;
            end if;
         end Read_Status;
         Reached : Boolean;
      begin
         loop
            Hold_Beat (Read_Status'Access);
            exit when not Helps;
            if not There then
               Move_Group (G, Fewer.Element);
            end if;
            Gather_Own_Poses (G, Positive'Max (Driver.Robot.Hand.Selfsight.Needed, 2 * Have), Reached);
            exit when not Reached;
            --  The estimators place again at the next beat that sees the poses doubled; one more beat to read it.
            Hold_Beat (null);
            Hold_Beat (null);
         end loop;
      end Top_Up_Poses;

      procedure Sweep_Channel (G : Group_Id; C : Positive) is
         Start : Real := 0.0;
         Step  : Estimate;
         Pixel : Real := 0.0;   --  the push that moves the own eye's view by a pixel
         Still_A_Closer : Boolean := False;
         Began : Natural := 0;   --  the beats the stream had when the sweep began
         procedure Read_Start (O : Observation) is
            R : constant Real_Array := O.Readings.Element (G);
            P : constant Natural := Own_Pair (G);
         begin
            Began := Natural (O.Beat);
            Start := R (R'First + C - 1);
            Step := Visible_Step (M, G, C);
            Pixel := (if P > 0 then Seen_By (Driver.Robot.Lockin.Shift (M, H.Data.Pairs (P).Eye, G, C)) else 0.0);
            Still_A_Closer := Sweepable (H, M, G);
         end Read_Start;
      begin
         Hold_Beat (Read_Start'Access);
         if not Still_A_Closer then
            Driver.Log.Line (Driver.Log.Robot, "hand: group" & G'Image & " is no longer a closer the hand watches;"
                             & " channel" & C'Image & " not swept");
            return;
         end if;
         if not Known (Step) then
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                             & " has no visible step measured; not swept");
            return;
         end if;
         --  Each way doubles its push from the visible step while the reading
         --  follows it and, once its views have shown something, while each push
         --  shows them something new (Sweep_Way); the first push the reading does
         --  not follow is the channel's end that way (a closer at an end of its
         --  travel answers only away from it), where the doubling stops. A
         --  channel that follows neither way at its first push, the amount the
         --  eyes saw it move by, is stuck where it is.
         Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image & " from "
                          & Driver.Log.Image (Start, 6) & ", pushed from " & Driver.Log.Image (Step.Value, 4)
                          & ", the step the eyes can see it make at all, and without its views seeing it at most to "
                          & (if Pixel > 0.0 then Driver.Log.Image (Pixel, 4) & ", where its view moves a pixel"
                             else "that: how far its view moves is not measured"));
         declare
            Answered_Ways : Natural := 0;
            Gone          : Boolean := False;   --  the group is no longer a closer the hand watches
            procedure Push (Offset : Real; Followed : out Boolean) is
            begin
               --  A push not made is not followed: the sweep ends.
               Followed := False;
               if not Gone then
                  Move_Channel (G, C, Start + Offset, Followed);
               end if;
            end Push;
            function Shows return Driver.Robot.Hand.Showing is
               Result : Driver.Robot.Hand.Showing := Driver.Robot.Hand.Not_Yet;
               procedure Read_Shows (O : Observation) is
                  pragma Unreferenced (O);
                  P : constant Natural := Own_Pair (G);
                  function Moved (Before, After : Real_Array) return Boolean is (Rest_Moved (M, G, Before, After));
               begin
                  Gone := P = 0 or else not Sweepable (H, M, G);
                  if Gone then
                     Result := Driver.Robot.Hand.Nothing_New;
                  elsif Sweeps.Gathered (H.Data.Pairs (P).Sweep) then
                     Result := (if Sweeps.Would_Extend (H.Data.Pairs (P).Sweep, C, Moved'Access)
                                then Driver.Robot.Hand.Something_New
                                else Driver.Robot.Hand.Nothing_New);
                  end if;
               end Read_Shows;
            begin
               Hold_Beat (Read_Shows'Access);
               return Result;
            end Shows;
            Unformed : Boolean := False;   --  some push's view did not form in time
         begin
            for Way of Real_Array'[-1.0, 1.0] loop
               exit when Unformed;
               declare
                  Pushes, Unseen, Longest : Natural;
                  Formed, Answered        : Boolean;
                  Wait_At_Most            : constant Positive :=
                    Positive'Max (1, (if Longest_Formed > 0 then Longest_Formed else Began));
               begin
                  Driver.Robot.Hand.Sweep_Way (Way, Step.Value, Pixel, Wait_At_Most, Push'Access, Shows'Access, Pushes,
                                               Unseen, Longest, Formed, Answered);
                  Answered_Ways := Answered_Ways + Boolean'Pos (Answered);
                  if Formed then
                     Longest_Formed := Natural'Max (Longest_Formed, Longest);
                  end if;
                  Unformed := not Formed;
                  Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                   & (if Way < 0.0 then " down" else " up") & ":" & Pushes'Image & " pushes,"
                                   & Unseen'Image & " of them before its views showed it anything; "
                                   & (if not Formed
                                      then "the view after its last push did not form within" & Wait_At_Most'Image
                                           & " beats, as long as " & (if Longest_Formed > 0 then "any view took before"
                                                                     else "the stream had run before the sweep")
                                      elsif Answered then "following from the first" else "at its end there"));
               end;
               Return_Channel (G, C, Start);
            end loop;
            if Gone then
               Driver.Log.Line (Driver.Log.Robot, "hand: group" & G'Image & " is no longer a closer the hand watches;"
                                & " its sweep of channel" & C'Image & " ends");
               return;
            end if;
            if Unformed then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                & " not swept: its own eye's views did not form (the eye's picture did not stop"
                                & " changing, or the rest of the body kept moving by what an eye can see)");
               return;
            end if;
            if Answered_Ways = 0 then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                & " follows neither way by its visible step; it is stuck, and not swept");
               return;
            end if;
         end;
         --  The estimators place the lobes once both ends are seen: open the
         --  channel at the end they show open.
         declare
            Open_At    : Real := Start;
            Known_Open : Boolean := False;
            procedure Read_Open (O : Observation) is
               pragma Unreferenced (O);
               P : constant Natural := Own_Pair (G);
            begin
               --  Its ends are asked for again: the views can have moved on since the lobes were placed from them.
               if P > 0 and then Sweeps.Status (H.Data.Pairs (P).Sweep, C) = Sweeps.Measured
                 and then Sweeps.Closing_Known (H.Data.Pairs (P).Sweep, C)
                 and then Sweeps.Has_Ends (H.Data.Pairs (P).Sweep, C)
               then
                  Known_Open := True;
                  Open_At := Driver.Robot.Hand.Views.Reading
                    ((if Sweeps.Closed_End_Is_High (H.Data.Pairs (P).Sweep, C)
                      then Sweeps.Low_End (H.Data.Pairs (P).Sweep, C) else Sweeps.High_End (H.Data.Pairs (P).Sweep, C)), C);
               end if;
            end Read_Open;
         begin
            Hold_Beat (Read_Open'Access);
            if Known_Open then
               Return_Channel (G, C, Open_At);
            end if;
         end;
      end Sweep_Channel;

      --  One press of a lobe at an opening: the hand turned about the eye so
      --  that Along points down into the table, then lowered until blocked
      --  (Driver.Robot.Hand.Descend), let go, and lifted back; all of it in the
      --  arm's own frame (Driver.Robot.Hand.Pressing), which needs the arm
      --  fitted and its eye's table found, not its place in the world. The
      --  contact is predicted once the presses so far fix the lobe's tip and the
      --  surface (Tips.Tip, Tips.Surface): its height above that plane along the
      --  way down, its sigma the plane's there with the tip's and the tool
      --  pose's. Before that nothing predicts it (a lobe's tip rides with its
      --  eye, so no view of the surface tells how far below the tip it is) and
      --  the descent doubles until blocked, no step taking the eye below the
      --  table its arm's own eye saw (Driver.Robot.Hand.Descend). The way back is
      --  to the arm's readings the descent began from, not to a pose: the arm is
      --  fitted again as it moves, and a pose of the arm's frame kept through a
      --  press would be in a frame that has moved.
      --  False when the arm cannot reach it or the body does not say where down
      --  is, or the eye had no room left above the table and nothing was met.
      --  The closer's readings are those of the opening, within their noise and not those of the other.
      function Closer_At (R : Hand_Record; Which : Opening) return Boolean is
         Now  : Driver.Robot.Hand.Views.Reading_Holders.Holder;
         Seen : Opening;
         procedure Read_Closer (O : Observation) is
         begin
            Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (R.Group));
         end Read_Closer;
      begin
         Hold_Beat (Read_Closer'Access);
         return Opening_Of (R, M, Now.Element, Seen) and then Seen = Which;
      end Closer_At;

      function Press_Once (Id : Hand_Id; R : Hand_Record; Lobe : Positive; Which : Opening; Along : Vec3) return Boolean is
         Aimed  : Driver.Robot.Hand.Pressing.Aimed;
         Plan   : Driver.Robot.Motion.Plan;
         Report : Driver.Robot.Motion.Step_Report;
         Arm_Is : Group_Id;
         Arm_Now : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  the arm's readings at the last Read_Arm
         Aim_At  : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  and where the descent began
         Began   : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  and where the aim began
         Stalls  : Natural := 0;   --  the pushes the watcher had judged stalled when the step now under way began

         --  The poses that point Along down are a circle of them (a turn of the hand about the way down, through the
         --  eye), and the arm's joints reach some of the circle and not others (Pressing.Aim_Reaching): the first whose
         --  aim is planned, and whose descent to the contact the presses so far predict is planned too (an end a channel
         --  showed, Driver.Robot.End_Of, refuses a path past it), is the aim; none is a tilt the arm cannot make from
         --  where it stands.
         Reaches : Boolean := False;   --  an aim, and the descent from it to the predicted contact, are planned
         Unmeasured_Aim : Boolean := False;   --  the aim at the least rotation was not planned for want of a measured arm
         Why_Not : Ada.Strings.Unbounded.Unbounded_String;   --  why the aim at the least rotation was not
         Yaw     : Real := 0.0;   --  the turn about the way down the aim took

         procedure Read_Aim (O : Observation) is
         begin
            Lost := not Present (Id, R);
            if Lost then
               return;
            end if;
            Arm_Is := Arm_Group (M, R.Arm);
            Began := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
            Stalls := H.Data.Found (Id).Stalls;
            declare
               B : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
            begin
               Driver.Robot.Hand.Pressing.Aim_Reaching
                 (M, R.Arm, R.Eye, O, Along, Driver.Robot.Hand.Tips.Tip (B, Lobe, Which), Driver.Robot.Hand.Tips.Surface (B),
                  Aimed, Yaw, Reaches, Unmeasured_Aim, Why_Not);
            end;
         end Read_Aim;

         procedure Note_The_Stop (O : Observation) is
            pragma Unreferenced (O);
            Noted : Boolean;
         begin
            Driver.Robot.Motion.Note_Stopped (M, Arm_Is, Noted);
            Learned_An_End := Learned_An_End or else Noted;
         end Note_The_Stop;

         Least : Real;   --  the least push, where the aim leaves the tool, kept above zero so that the doubling begins
         By : Real := 0.0;
         Descended : Real := 0.0;   --  how far the pushes that were reached have lowered the tool
         procedure Read_Lower (O : Observation) is
         begin
            Lost := not Present (Id, R);
            if Lost then
               return;
            end if;
            Plan := Driver.Robot.Hand.Pressing.Lowered (M, R.Arm, O, Aimed.Into, By);
            Stalls := H.Data.Found (Id).Stalls;
         end Read_Lower;

         Unplanned : Boolean := False;   --  a step could not be planned
         procedure Lower (Step : Real; Reached : out Boolean) is
         begin
            By := Step;
            Hold_Beat (Read_Lower'Access);
            if Lost then
               Reached := False;
               return;
            end if;
            if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
               Unplanned := True;
               Reached := False;
               return;
            end if;
            Driver.Robot.Motion.Follow (M, Plan, Report);
            Reached := Report.Outcome = Driver.Robot.Motion.Reached;
            if Reached then
               Descended := Descended + Step;
            end if;
         end Lower;

         function Above return Driver.Robot.Hand.Heights is
            --  The lobe's tip above the surface the presses so far fixed, and the
            --  eye above the table its arm's own eye saw, along the way down.
            Result : Driver.Robot.Hand.Heights;
            procedure Read_Above (O : Observation) is
            begin
               Lost := not Present (Id, R);
               if Lost then
                  return;
               end if;
               declare
                  B   : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
                  Eye : constant Point_Estimate :=
                    (Mean => Eye_In_Tool (M, R.Eye, O).Pose.Translation, Covariance => [others => [others => 0.0]]);
               begin
                  Result.Tip := Driver.Robot.Hand.Pressing.Gap
                    (M, R.Arm, O, Driver.Robot.Hand.Tips.Tip (B, Lobe, Which), Driver.Robot.Hand.Tips.Surface (B), Aimed.Into);
                  Result.Eye := Driver.Robot.Hand.Pressing.Gap (M, R.Arm, O, Eye, Table_In_Arm (M, R.Arm), Aimed.Into);
                  --  The watcher judges every push from the stream (Driver.Robot.Hand.Lowering), as the press is found:
                  --  one verdict, here as in a replay. It has judged the step before this one stalled.
                  Result.Stalled := H.Data.Found (Id).Stalls > Stalls;
               end;
            end Read_Above;
         begin
            Hold_Beat (Read_Above'Access);
            return Result;
         end Above;
         Steps : Driver.Robot.Hand.Descent_Steps;

         --  What the hand has found of its press since the let-go: the presses its watcher found at a rest, and the most
         --  beats a decider waits for one: no wait is longer than all the waiting so far, the stream's length (it is the
         --  hand settling on what it pressed that ends the wait, Driver.Robot.Hand.Presses, and a creep that decays does).
         Rests_Now : Natural := 0;
         Allowed   : Natural := 0;
         procedure Read_Rests (O : Observation) is
            pragma Unreferenced (O);
         begin
            Lost := not Present (Id, R);
            if Lost then
               return;
            end if;
            Rests_Now := H.Data.Found (Id).Rests;
            Allowed := M.Beats;
         end Read_Rests;

         procedure Read_Arm (O : Observation) is
            --  The arm's group was found at the aim (Arm_Is) and is not looked up again: it is the arm's readings that
            --  are wanted here, and they are there whatever became of the hand.
         begin
            Arm_Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
         end Read_Arm;

         --  The hand this press measures is gone: it ends where it stands, with the arm let go and taken back to where
         --  the descent began (or to where it stands, when it never moved), and nothing of it is kept.
         procedure End_Lost (Moved : Boolean) is
         begin
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the hand this press of lobe" & Lobe'Image & " measures is gone:"
                             & " its closer group, group" & R.Group'Image & ", is no longer a closer of arm" & R.Arm'Image
                             & " as the estimators have it between two beats of the press; the press ends here"
                             & (if Moved then ", the arm let go and taken back to where the descent began" else ""));
            if Moved then
               Hold_Beat (Read_Arm'Access);
               Move_Group (Arm_Is, Arm_Now.Element);
               Move_Group (Arm_Is, Aim_At.Element);
            end if;
         end End_Lost;
      begin
         Aim_Short := False;
         Learned_An_End := False;
         Hold_Beat (Read_Aim'Access);
         if Lost then
            End_Lost (Moved => False);
            return False;
         end if;
         if not Aimed.Ok then
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": up, the arm's pose or the eye's mount is unmeasured;"
                             & " no press");
            return False;
         end if;
         if not Reaches then
            if Unmeasured_Aim then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": cannot aim a press: "
                                & Ada.Strings.Unbounded.To_String (Why_Not));
               return False;
            end if;
            --  Nothing was moved: the tilt is one the arm cannot make from where it stands, as a press that stopped
            --  short of the table says, and half of it is tried.
            Aim_Short := True;
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": no aim of a press of lobe" & Lobe'Image & " at "
                             & (if Which = Open then "open" else "closed") & " is planned, the hand turned about the way down by "
                             & "any eighth of a turn: " & Ada.Strings.Unbounded.To_String (Why_Not));
            return False;
         end if;
         if Yaw /= 0.0 then
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the aim of a press of lobe" & Lobe'Image
                             & " at the least rotation is not planned (" & Ada.Strings.Unbounded.To_String (Why_Not)
                             & "); the hand turned about the way down by " & Driver.Log.Image (Yaw, 4) & " rad it is");
         end if;
         Plan := Aimed.Plan;
         Least := Real'Max (Real'Model_Epsilon, Aimed.Least);
         Driver.Robot.Motion.Follow (M, Plan, Report);
         Hold_Beat (Read_Arm'Access);
         --  An aim the arm did not complete is not a press: it stopped on something of its own or short of the pose,
         --  and lowering the hand from where it stands would press at no pose this aim chose (A27's first press of
         --  hand 2, the third joint at +0.07 for the -0.05 asked, lowered from there and fitted as the tip, 12.39 from
         --  the eye). The arm goes back to where the aim began, and the caller is told a tilt cannot be made.
         if Report.Outcome /= Driver.Robot.Motion.Reached then
            Aim_Short := True;
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the arm did not reach the aim of a press of lobe" & Lobe'Image
                             & " at " & (if Which = Open then "open" else "closed") & ", turning the hand "
                             & Driver.Log.Image (Aimed.Turn, 4) & " rad (" & Ada.Strings.Unbounded.To_String (Report.Detail)
                             & "): no press is made from where it stopped, and the arm is taken back to where the aim began");
            Hold_Beat (Note_The_Stop'Access);   --  the aim is in free air: what stopped it is the body's, and noted
            Move_Group (Arm_Is, Began.Element);
            return False;
         end if;
         Aim_At := Arm_Now;
         --  The closer is at the opening the press is made at, or is brought there first: a press made
         --  while the closer is on its way is a press at no opening (A17's first press, at 0.686 of an
         --  opening at 1.0: the finger that held it back was off the table only once the aim turned the
         --  hand). It is asked once more, and if it does not come, no press is made.
         if not Closer_At (R, Which) then
            Move_Group (R.Group, R.Readings (Which).Element);
            if not Closer_At (R, Which) then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the closer of lobe" & Lobe'Image & " does not come to its "
                                & (if Which = Open then "open" else "closed") & " opening: no press is made at it");
               return False;
            end if;
         end if;
         declare
            First : constant Driver.Robot.Hand.Heights := Above;
         begin
            if Lost then
               End_Lost (Moved => True);
               return False;
            end if;
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": pressing lobe" & Lobe'Image & " at "
                             & (if Which = Open then "open" else "closed") & ", its line of sight tilted "
                             & Driver.Log.Image (Driver.Robot.Hand.Aims.Spread (R.Lobes (Lobe).Sights (Which).Ray.Direction.Unit_Vector, [1 => Along]), 4)
                             & " rad from straight down, aimed by turning the hand " & Driver.Log.Image (Aimed.Turn, 4) & " rad"
                             & (if Known (First.Tip)
                                then ", " & Driver.Log.Image (First.Tip.Value, 4) & " +- " & Driver.Log.Image (First.Tip.Sigma, 4)
                                     & " above the surface the presses so far fixed"
                                else ", nothing yet predicting the surface below its tip: doubling from "
                                     & Driver.Log.Image (Least, 4) & " until blocked")
                             & (if Known (First.Eye)
                                then "; the eye " & Driver.Log.Image (First.Eye.Value, 4) & " above the table"
                                else "; the eye's height above the table unknown"));
         end;
         Driver.Robot.Hand.Descend (Above'Access, Least, Lower'Access, Steps);
         if Lost then
            End_Lost (Moved => True);
            return False;
         end if;
         if Steps.Stalled then
            --  The last step was followed and took the hand nowhere: it is not part of the lowering.
            Descended := Descended - By;
         elsif not Unplanned and then not Steps.Spent then
            --  The last push was not completed. When the tip stands above the contact the presses so far predict by more
            --  than Z of its sigma the hand is in free air, and what stopped the push is the body's: the channel that
            --  fell short of the rest has stopped at an end, and says so (A22's press 2 at beat 13827, the third joint at
            --  -0.0359 for the -0.0578 asked, the hand 8.7 cm above the table). On the table it is not.
            declare
               Stopped : constant Driver.Robot.Hand.Heights := Above;
            begin
               if Lost then
                  End_Lost (Moved => True);
                  return False;
               end if;
               if Driver.Robot.Hand.In_Free_Air (Stopped) then
                  Hold_Beat (Note_The_Stop'Access);
               end if;
            end;
         end if;
         --  One line a press, for the boot's account of where its time went:
         --  how many pushes, and why each was as long as it was.
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": press of lobe" & Lobe'Image & " at "
                          & (if Which = Open then "open" else "closed") & ":"
                          & Natural'Image (Driver.Robot.Hand.Total (Steps)) & " pushes,"
                          & Steps.Fast'Image & " fast and" & Steps.Band'Image
                          & " within Z sigma of the contact its presses predict," & Steps.Blind'Image
                          & " doubling from " & Driver.Log.Image (Least, 4) & " with nothing predicting it (before a prediction, or past its band),"
                          & Steps.Capped'Image & " cut to the eye's room above the table; "
                          & (if Unplanned then "then it cannot press lower: " & Driver.Robot.Motion.Why (Plan)
                             elsif Steps.Spent then "then the eye has no room left above the table, or the steps that cover it were made,"
                                  & " and nothing was met, after lowering "
                                  & Driver.Log.Image (Descended, 4)
                             elsif Steps.Stalled then "stalled, the arm followed the last push by " & Driver.Log.Image (By, 4)
                                  & " and the hand did not go down with it, after lowering " & Driver.Log.Image (Descended, 4)
                             else "blocked, the last push by " & Driver.Log.Image (By, 4) & " after lowering "
                                  & Driver.Log.Image (Descended, 4)));
         if Unplanned or else Steps.Spent then
            --  Nothing was met and no more steps are made, or the next step is one the arm cannot be taken through (an
            --  end a channel showed is on the way down, with nothing predicting the contact to have told so before): the
            --  hand goes back to where the descent began, as after a press, for the presses after it begin there. A
            --  tilt that cannot be lowered is one that cannot be made, and half of it is tried.
            Aim_Short := Unplanned;
            Hold_Beat (Read_Arm'Access);
            Move_Group (Arm_Is, Aim_At.Element);
            return False;
         end if;
         --  Let go: the arm held where the block left it, so the hand rests there, and the arm does not leave until it
         --  has. The watcher takes the press when the hand has settled on what it pressed (Driver.Robot.Hand.Presses); a
         --  retreat begun while the readings still ease back would give it the rest at the aim, where the hand is not on
         --  what it pressed (A35's first press: a tip 11.4457 from the eye, the eye's height at the aim, for the 3.8 it
         --  was). The wait ends when the watcher has found the press, or at the stream's length (it never comes to that
         --  while the hand's creep decays).
         declare
            Rests_Before : Natural;
            Waited       : Natural := 0;
         begin
            Hold_Beat (Read_Rests'Access);
            if Lost then
               End_Lost (Moved => True);
               return False;
            end if;
            Rests_Before := Rests_Now;
            Hold_Beat (Read_Arm'Access);
            Move_Group (Arm_Is, Arm_Now.Element);
            loop
               Hold_Beat (Read_Rests'Access);
               exit when Lost or else Rests_Now > Rests_Before or else Waited >= Allowed;
               Waited := Waited + 1;
            end loop;
            if Lost then
               End_Lost (Moved => True);
               return False;
            end if;
            if Rests_Now = Rests_Before then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": no press of lobe" & Lobe'Image & " was found at the rest after"
                                & " the let-go: the arm did not rest in" & Waited'Image & " beats, the stream's length"
                                & "; the arm is taken back to where the descent began");
               Move_Group (Arm_Is, Aim_At.Element);
               return False;
            end if;
            Move_Group (Arm_Is, Aim_At.Element);
            return True;
         end;
      end Press_Once;

      --  A press; when its aim stopped on an end the arm showed by stopping there (Learned_An_End), the same press once
      --  more: the aim is planned again with the end known, turned about the way down past it if it was in the way at the
      --  pose it was first planned at, or not planned at all (Aim_Short) and no arm move is spent (A34: lobe 2 at open,
      --  the fourth joint at -2.1746 for the -2.1870 asked, and the tilt then halved under the least that tells a tip).
      function Press_Past_Its_Ends
        (Id : Hand_Id; R : Hand_Record; Lobe : Positive; Which : Opening; Along : Vec3) return Boolean
      is
         Done : Boolean := Press_Once (Id, R, Lobe, Which, Along);
      begin
         if not Done and then not Lost and then Aim_Short and then Learned_An_End then
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the aim of a press of lobe" & Lobe'Image & " at "
                             & (if Which = Open then "open" else "closed")
                             & " stopped on an end the arm showed by it: the same aim is planned again past it");
            Done := Press_Once (Id, R, Lobe, Which, Along);
         end if;
         return Done;
      end Press_Past_Its_Ends;

      procedure Press_Lobe (Id : Hand_Id; R : Hand_Record; Lobe : Positive; Which : Opening) is
         Sight  : constant Vec3 := R.Lobes (Lobe).Sights (Which).Ray.Direction.Unit_Vector;
         Lines  : Driver.Robot.Hand.Aims.Direction_Array (1 .. Natural (R.Lobes.Length));
         K      : Natural := 0;
      begin
         for L in 1 .. Natural (R.Lobes.Length) loop
            if L /= Lobe and then R.Lobes (L).Sights (Which).Known then
               K := K + 1;
               Lines (K) := R.Lobes (L).Sights (Which).Ray.Direction.Unit_Vector;
            end if;
         end loop;
         declare
            Rest : constant Driver.Robot.Hand.Aims.Direction_Array := Lines (1 .. K);
            Away : constant Vec3 :=
              (if K > 0 then Driver.Robot.Hand.Aims.Away_From (Sight, Rest) else Driver.Robot.Hand.Aims.Any_Across (Sight));
            --  The hand's own angle: between its lines of sight, or for a lone
            --  lobe the angle it travels through between the openings.
            Other_End : constant Opening := (if Which = Open then Closed_Empty else Open);
            Scale : constant Real :=
              (if K > 0 then Driver.Robot.Hand.Aims.Spread (Sight, Rest)
               elsif R.Lobes (Lobe).Sights (Other_End).Known
               then Driver.Robot.Hand.Aims.Spread (Sight, [1 => R.Lobes (Lobe).Sights (Other_End).Ray.Direction.Unit_Vector])
               else 0.0);
            Agreed  : Boolean := True;    --  the latest press is one its tip rests on
            Checked : Boolean := False;   --  the lobe's tip is confirmed by a second press from another pose
            Least   : Real := Real'Last;  --  the least tilt that tells the tip from a stop that does not move with it
            procedure Read_Agreed (O : Observation) is
            begin
               Lost := not Present (Id, R);
               if Lost then
                  return;
               end if;
               declare
                  Book : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
               begin
                  Agreed := Driver.Robot.Hand.Tips.Latest_Agrees (Book);
                  Checked := Driver.Robot.Hand.Tips.Confirmed (Book, Lobe, Which);
                  Least := Driver.Robot.Hand.Aims.Least_Tilt
                    (Driver.Robot.Hand.Tips.Distance (Book, Lobe, Which), Driver.Robot.Hand.Pressing.Least_Push (M, R.Arm, O));
               end;
            end Read_Agreed;
         begin
            Hold_Beat (Read_Agreed'Access);
            if Lost then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " is not pressed: the hand is gone"
                                & " (its closer group is no longer a closer of its arm)");
               return;
            end if;
            if Checked then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " at "
                                & (if Which = Open then "open" else "closed") & " is not pressed: its tip is confirmed already");
               return;
            end if;
            if not Press_Past_Its_Ends (Id, R, Lobe, Which, Sight) then
               return;   --  Lost, or a press that could not be made: the next lobe is the caller's to go on to
            end if;
            Hold_Beat (Read_Agreed'Access);
            if Lost then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " is pressed no more: the hand is gone");
               return;
            end if;
            if Checked then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " at "
                                & (if Which = Open then "open" else "closed") & " pressed once, straight, and its tip is"
                                & " confirmed: presses made for other lobes had landed on it");
               return;
            end if;
            if Scale <= 0.0 then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " at "
                                & (if Which = Open then "open" else "closed") & " pressed once, straight, and not tilted:"
                                & " no other line of sight of the hand is known to tilt away from");
               return;
            end if;
            --  Leaning to either side of away, half-way to across.
            for Side of Real_Array'[-1.0, 1.0] loop
               exit when Checked;
               declare
                  Lean  : constant Vec3 := Exp ((Side * Ada.Numerics.Pi / 4.0) * Sight) * Away;
                  Bound : Real := Ada.Numerics.Pi / 2.0;   --  the least tilt this side was found not to make
                  Tilt  : Real := Driver.Robot.Hand.Aims.First_Tilt (Scale, Least, Bound);
                  Made  : Natural := 0;
                  Why   : Ada.Strings.Unbounded.Unbounded_String :=
                    Ada.Strings.Unbounded.To_Unbounded_String ("tilted to a right angle");
               begin
                  --  The hand's own angle, and no less than the least that tells the tip from a stop (Aims.First_Tilt), then
                  --  double while the presses are ones the tip rests on, half when one stops short of the table
                  --  (Driver.Robot.Hand.Aims.Next_Tilt).
                  while Tilt > 0.0 and then Tilt < Bound loop
                     if not Press_Past_Its_Ends (Id, R, Lobe, Which, Driver.Robot.Hand.Aims.Tilted (Sight, Lean, Tilt)) then
                        if Lost then
                           return;   --  Press_Once said so
                        end if;
                        if not Aim_Short then
                           Why := Ada.Strings.Unbounded.To_Unbounded_String ("a press could not be made");
                           exit;
                        end if;
                        --  The arm could not be taken to the tilt: one it cannot make from here, as a press that stopped
                        --  short of the table says (it was not pressed, which is what it costs): half of it is tried.
                        Driver.Robot.Hand.Aims.Next_Tilt (Tilt, Stalled => True, Bound => Bound, Least => Least);
                     else
                        Made := Made + 1;
                        Hold_Beat (Read_Agreed'Access);
                        if Lost then
                           Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image
                                            & " is pressed no more: the hand is gone");
                           return;
                        end if;
                        if Checked then
                           Why := Ada.Strings.Unbounded.To_Unbounded_String
                             ("the tip is confirmed: a press from another pose landed on it");
                           exit;
                        end if;
                        Driver.Robot.Hand.Aims.Next_Tilt (Tilt, Stalled => not Agreed, Bound => Bound, Least => Least);
                     end if;
                  end loop;
                  if Bound < Ada.Numerics.Pi / 2.0 and then not Checked and then Tilt = 0.0 then
                     Why := Ada.Strings.Unbounded.To_Unbounded_String
                       ("a press stopped short of the table at" & Driver.Log.Image (Bound, 4)
                        & " rad, and the tilts under it are none that tells the tip from a stop");
                  end if;
                  Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " at "
                                   & (if Which = Open then "open" else "closed") & " tilted "
                                   & (if Side < 0.0 then "one way" else "the other") & ":" & Made'Image
                                   & " presses, then " & Ada.Strings.Unbounded.To_String (Why));
               end;
            end loop;
         end;
      end Press_Lobe;

      Of_Arm   : Arm_Id'Base := 0;
      Channels : Natural := 0;
      Now      : Boolean := False;   --  the closer can be swept: an eye on its arm watches it
      Is_One   : Boolean := False;   --  the body takes it for a closer
      procedure Read_Group (O : Observation) is
         pragma Unreferenced (O);
         P : constant Natural := Own_Pair (Closer);
      begin
         Now := Sweepable (H, M, Closer);
         Is_One := Role (M, Closer) = Driver.Robot.Closer;
         Of_Arm := Closer_Arm (M, Closer);
         Channels := (if P > 0 then Sweeps.Channels (H.Data.Pairs (P).Sweep) else 0);
      end Read_Group;
   begin
      if Part = Sweep_Part then
         --  Sweep every channel of the closer when an eye on its arm watches it.
         Hold_Beat (Read_Group'Access);
         if Now then
            --  The eye sees the closer's readings from the poses a deviation needs before the closer is moved, or
            --  the arm gives it them.
            declare
               Enough : Boolean;
            begin
               Gather_Own_Poses (Closer, Driver.Robot.Hand.Selfsight.Needed, Enough);
            end;
            for C in 1 .. Channels loop
               Sweep_Channel (Closer, C);
            end loop;
            for C in 1 .. Channels loop
               Top_Up_Poses (Closer, C);
            end loop;
         elsif Is_One then
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & Closer'Image & " of arm" & Of_Arm'Image
                             & " is not swept: no eye on its arm watches it, so no hand is made of it");
         end if;
         return;
      end if;
      --  Press every lobe of the hand at both openings. The hand is held by its closer group and found again
      --  where it is read, never by an index kept from an earlier beat: the estimators read the roles again between
      --  two held beats, and a hand whose group the body no longer takes for a closer of its arm is gone.
      for Which in Opening loop
         declare
            Id   : Hand_Id := Hand_Id'First;
            R    : Hand_Record;
            Here : Boolean := False;
            procedure Read_Hand (O : Observation) is
               pragma Unreferenced (O);
               There : constant Natural := Index_Of (Closer);
            begin
               Here := There > 0
                 and then Role (M, Closer) = Driver.Robot.Closer
                 and then Closer_Arm (M, Closer) = H.Data.Found (Hand_Id (There)).Arm;
               if Here then
                  Id := Hand_Id (There);
                  R := H.Data.Found (Id);
               end if;
            end Read_Hand;
         begin
            Lost := False;
            Hold_Beat (Read_Hand'Access);
            if Here then
               Move_Group (R.Group, R.Readings (Which).Element);
               for L in 1 .. Natural (R.Lobes.Length) loop
                  if R.Lobes (L).Sights (Which).Known then
                     Press_Lobe (Id, R, L, Which);
                     exit when Lost;
                  else
                     Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & L'Image & " at "
                                      & (if Which = Open then "open" else "closed")
                                      & " is not pressed: its tip is not seen in the hand's eye at this opening");
                  end if;
               end loop;
            elsif Which = Open then
               Driver.Log.Line (Driver.Log.Robot, "hand: the hand of closer group" & Closer'Image & " is not pressed:"
                                & " no hand is made of it, or its group is no longer a closer of its arm");
            end if;
         end;
      end loop;
   end One_Hand;

   Text : Ada.Strings.Unbounded.Unbounded_String;
   procedure Read_Description (O : Observation) is
      pragma Unreferenced (O);
   begin
      Text := Ada.Strings.Unbounded.To_Unbounded_String (Describe (H));
   end Read_Description;

   --  The closers to measure: every group the body takes for a closer, or whose closer an eye on its arm watches,
   --  or of which a hand is already made, when the hands begin.
   Closers : Group_Lists.Vector;
   procedure Read_Closers (O : Observation) is
      pragma Unreferenced (O);
   begin
      Closers.Clear;
      for G in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Sweepable (H, M, G) or else Role (M, G) = Closer then
            Closers.Append (G);
         end if;
      end loop;
      for I in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         if not Closers.Contains (H.Data.Found (I).Group) then
            Closers.Append (H.Data.Found (I).Group);
         end if;
      end loop;
   end Read_Closers;

   procedure Hand_Lane (Lane : Positive) is
   begin
      One_Hand (Closers (Lane), Press_Part);
   end Hand_Lane;

   procedure Hands_At_Once is new Driver.Beats.At_Once (Hand_Lane);

begin
   if H.Data = null then
      H.Data := new Hand_Data;
   end if;
   Hold_Beat (Read_Closers'Access);
   if Closers.Is_Empty then
      Driver.Log.Line (Driver.Log.Robot, "hand: no closer was found, so nothing is swept or pressed");
   else
      for G of Closers loop
         One_Hand (G, Sweep_Part);
      end loop;
      if Natural (Closers.Length) = 1 then
         One_Hand (Closers.First_Element, Press_Part);
      else
         for G of Closers loop
            Driver.Log.Line (Driver.Log.Robot, "hand: the hand of closer group" & G'Image & " is pressed with the"
                             & Natural'Image (Natural (Closers.Length)) & " hands at once");
         end loop;
         Hands_At_Once (Positive (Closers.Length));
      end if;
   end if;
   Hold_Beat (Read_Description'Access);
   Driver.Log.Line (Driver.Log.Robot, "hand: measured" & ASCII.LF & Ada.Strings.Unbounded.To_String (Text));
end Measure;
