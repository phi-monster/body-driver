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

   --  The most beats the eye has taken to settle after a move (Longest_Settling), and no fewer than a push takes to
   --  show in it at all: the body's delay and the eye's lag, both kept in the body file. A boot begun from one has
   --  seen few moves of its own: its closer's sweep waited one beat for a view, had none, and no hand was made of
   --  the closer (f16j on 10-15, from f16h's body kept after its sweeps).
   function Settling_Of (E : Eye_Id) return Natural is
      Delay_Beats : Natural;
      Delay_Known : Boolean;
   begin
      Driver.Robot.Steps.Body_Delay (M, Delay_Beats, Delay_Known);
      return Natural'Max (Longest_Settling (M, E),
                          (if Delay_Known then Delay_Beats else 0)
                          + (if Lag_Known (M, E) then Natural'Max (0, Image_Lag (M, E)) else 0));
   end Settling_Of;

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

   --  A hand to measure is named by its closer group, or by its arm alone for the hand an arm's tool is when the
   --  arm has no closer (Group 0).
   type Hand_Key is record
      Group : Group_Id'Base := 0;
      Arm   : Arm_Id'Base := 0;
   end record;
   package Key_Lists is new Ada.Containers.Vectors (Positive, Hand_Key);
   --  The hand the arm's tool is, now; 0 if none. Read inside a held beat.
   function Index_Of_Tool (Arm : Arm_Id) return Natural is
   begin
      for I in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         if H.Data.Found (I).Group = 0 and then H.Data.Found (I).Arm = Arm then
            return Natural (I);
         end if;
      end loop;
      return 0;
   end Index_Of_Tool;

   --  One part of one hand, from its closer group: its closer swept, or its lobes pressed, by the body's roles as
   --  they are when each begins. The closers are swept one after another: a sweep reads the views its own eye has
   --  of its fingers, and another arm moving in that eye's view spoils them (A59, both swept at once: no hand was
   --  made of the closer whose eye saw the other arm move; A58 lost the ends of a view it had placed the lobes
   --  from). The hands are pressed at once (Driver.Beats.At_Once, a hand a lane): each moves only its own arm and
   --  closer, reads and changes the models only in its held beats, and keeps what follows here, its own, from one
   --  beat to the next.
   type Hand_Part is (Sweep_Part, Press_Part);
   procedure One_Hand (Key : Hand_Key; Part : Hand_Part) is
      Closer : constant Group_Id'Base := Key.Group;

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
      --  above the table: first by the least move that moves its picture by what its cells tell (a plan's last
      --  readings, tried by Lockin.Shows_Step before anything moves), then by twice that, and so on, until the eye has
      --  the poses or cannot be raised further.
      procedure Gather_Own_Poses (G : Group_Id; Wanted : Positive; Reached : out Boolean) is
         Placed : Boolean := False;   --  the closer is one an eye on an arm watches
         Arm    : Arm_Id := 1;
         Eye    : Eye_Id := 1;
         Have   : Natural := 0;       --  the poses its eye has seen its readings from
         Began  : Natural := 0;       --  the beats the stream had
         Settles : Natural := 0;     --  the most beats the own eye has taken to settle after a move
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
            Settles := (if Placed then Settling_Of (H.Data.Pairs (P).Eye) else 0);
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
                 ("no raise within the arm's reach moves its eye's picture by what the eye's cells tell");
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
                     --  A pose is one the eye was carried to (Hand.Eye_Moved): the raise moves its picture by what
                     --  its cells tell, and a raise its readings show is not always one (A73: arm 2's first raises
                     --  were of a few millionths, the least the tool's place told at its reference readings, and its
                     --  eye gathered nine poses that told the fingers from nothing).
                     if Driver.Robot.Lockin.Shows_Step (M, Eye, Group, Step) then
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
            if not Raised then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & ": the raise of its eye ended "
                                & Report.Outcome'Image & " after" & Report.Beats'Image & " beats: "
                                & Ada.Strings.Unbounded.To_String (Report.Detail));
            end if;
         end Raise_Eye;
      begin
         Hold_Beat (Read_Poses'Access);
         Reached := Placed and then Have >= Wanted;
         if not Placed or else Reached then
            return;
         end if;
         Driver.Robot.Hand.Gather_Poses
           (Wanted, Driver.Robot.Hand.Wait_For_A_View (Longest_Formed, Settles, Began), Poses_Now'Access,
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
      procedure Top_Up_Poses (G : Group_Id; C : Positive) is
         Status : Sweeps.Progress := Sweeps.Measured;
         Helps  : Boolean := False;   --  more poses could place the lobes
         Have   : Natural := 0;
         Share, Doubt : Real := 0.0;   --  of the last placing (Lobes.Located)
         procedure Read_Status (O : Observation) is
            P : constant Natural := Own_Pair (G);
         begin
            Status := Sweeps.Measured;
            Helps := False;
            if P > 0 and then C <= Sweeps.Channels (H.Data.Pairs (P).Sweep) then
               declare
                  S : Sweeps.State renames H.Data.Pairs (P).Sweep;
               begin
                  Status := Sweeps.Status (S, C);
                  Have := Sweeps.Poses (S, O.Readings.Element (G));
                  Helps := Status = Sweeps.Unlocated
                    or else (Status = Sweeps.Unplaced
                             and then Sweeps.Located_Of (S, C).How = Driver.Robot.Hand.Lobes.Unseparated);
                  Share := Sweeps.Located_Of (S, C).Share;
                  Doubt := Sweeps.Located_Of (S, C).Doubt;
               end;
            end if;
         end Read_Status;
         Reached : Boolean;
         --  What the last placing told the lobes apart by: the share of the changed pixels' variance its cut
         --  explains, and the mixture's own doubt. A doubling of the poses that raises the one and lowers the
         --  other not at all shows the poses are not what is missing, and the hand is not raised again for more
         --  (A91's hand 1 doubled its raises with the lobes no further apart, up to the arm's end; item 5).
         Last_Share : Real := -1.0;
         Last_Doubt : Real := Real'Last;
      begin
         loop
            Hold_Beat (Read_Status'Access);
            exit when not Helps;
            if Status = Sweeps.Unplaced and then Last_Share >= 0.0 and then Share <= Last_Share and then Doubt >= Last_Doubt then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image & ": doubling the"
                                & " poses to" & Have'Image & " told the lobes no further apart (the cut explains "
                                & Driver.Log.Image (Share, 3) & " of their variance as before, doubt " & Driver.Log.Image (Doubt, 2)
                                & "); the hand is not raised again for more");
               exit;
            end if;
            Last_Share := Share;
            Last_Doubt := Doubt;
            Gather_Own_Poses (G, Positive'Max (Driver.Robot.Hand.Selfsight.Needed, 2 * Have), Reached);
            exit when not Reached;
            --  The estimators place again at the next beat that sees the poses doubled; one more beat to read it.
            Hold_Beat (null);
            Hold_Beat (null);
         end loop;
      end Top_Up_Poses;

      --  The closer stopped between two neighbouring views of its travel between which some lobe's tip moves farther
      --  than the matcher searches at its finest (Driver.Robot.Hand.Follow): the readings between them cut in as
      --  many equal parts as that move is times what the matcher searches (a tip that moves evenly with the reading
      --  then moves no farther over each), the closer stood at each in turn until its view has the frames a view is
      --  judged by, and then the gaps looked for again, until no step of any lobe is wider, or a part is no longer
      --  apart from the one before by more than the channel's noise: the estimators then follow each lobe from the
      --  open end to the closed one and fit its shape. A view is kept as a stop when the closer leaves it (Views),
      --  so the closer goes on to the next part as soon as the view it stands in can be judged. The
      --  sweep's own stops double as it pushes and leave the last half of the travel in one step (f16y: from 4.65 to
      --  0.70 of a travel from 8.70), which no patch carries. A view of a setting between the ends is a stop only
      --  when the rest of the body stands as it stood when the ends were seen (Views), so the closer's arm is taken
      --  back there first: the poses the hand gathered for its deviation leave it elsewhere (a01 on 10-16: raised by
      --  3.04 after the sweep, and the view at half way never became a stop). Then the closer is opened again.
      procedure Fill_Stops (G : Group_Id; C : Positive) is
         Measured : Boolean := False;   --  the channel's lobes are placed and which end is closed is known
         Found    : Boolean := False;   --  a step too wide
         From, To : Real := 0.0;        --  between these readings
         Widest   : Real := 0.0;        --  its tip's move there, pixels
         Limit    : Real := 0.0;        --  what the matcher searches at its finest
         Stops    : Natural := 0;
         Open_At  : Real := 0.0;
         Sigma    : Real := 0.0;        --  the channel's noise (Closer_Sigma)
         Began    : Natural := 0;
         Settles  : Natural := 0;
         --  The gaps whose parts saw no tip: passed over when the gaps are looked for again, so that the others are
         --  cut (a13 on 10-16: the first gap was such a one, near the closed end, and the cutting stopped there with
         --  an 87-pixel step left on the open side).
         package Gap_Lists is new Ada.Containers.Vectors (Positive, Real);
         Unseen_From, Unseen_To : Gap_Lists.Vector;
         function Unseen (A, B : Real) return Boolean is
           (for some K in Unseen_From.First_Index .. Unseen_From.Last_Index =>
              Unseen_From (K) = A and then Unseen_To (K) = B);
         procedure Read_Gap (O : Observation) is
            P : constant Natural := Own_Pair (G);
         begin
            Found := False;
            Measured := False;
            if Began = 0 then
               Began := Natural (O.Beat);
            end if;
            if P = 0 or else C > Sweeps.Channels (H.Data.Pairs (P).Sweep) then
               return;
            end if;
            Settles := Settling_Of (H.Data.Pairs (P).Eye);
            declare
               S : Sweeps.State renames H.Data.Pairs (P).Sweep;
            begin
               if Sweeps.Status (S, C) /= Sweeps.Measured or else not Sweeps.Closing_Known (S, C) then
                  return;
               end if;
               Measured := True;
               Stops := Sweeps.Stops (S, C);
               Sigma := Closer_Sigma (M, G) (C);
               declare
                  Lobes  : constant Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector := Sweeps.Lobes_Of (S, C);
                  Low_R  : constant Real := Sweeps.End_Reading (S, C, High => False);
                  High_R : constant Real := Sweeps.End_Reading (S, C, High => True);
               begin
                  Open_At := (if Sweeps.Closed_End_Is_High (S, C) then Low_R else High_R);
                  for Lobe_No in 1 .. Natural (Lobes.Length) loop
                     declare
                        L         : constant Driver.Robot.Hand.Lobes.Lobe := Lobes (Lobe_No);
                        --  Along the travel from the open end, the stops by their readings, to the closed end: the
                        --  lobe is followed from the open end as far as each step is one the matcher follows
                        --  (Hand.Followed_Shape), so the gaps nearest the open end are cut first. Cut from the low end,
                        --  a14 (10-16) cut 41 stops into the gaps by the closed end, its tip unseen there, before the
                        --  13 pixels of the first step from the open end.
                        Open_Low  : constant Boolean := Sweeps.Closed_End_Is_High (S, C);
                        --  The view at place K of the travel from the low end: 0 the low end, Stops + 1 the high.
                        procedure View (K : Natural; Seen : out Boolean; Tip : out Driver.Images.Pixel; R : out Real) is
                        begin
                           --  Where its tip region is (Lobes.Tip_Track), as Hand.Followed_Shape measures the steps.
                           if K = 0 then
                              Seen := L.Tip_Known_Here;
                              Tip := Driver.Robot.Hand.Lobes.Tip_Track (L.Here, L.Centre_Here, L.Tip_Here);
                              R := Low_R;
                           elsif K > Stops then
                              Seen := L.Tip_Known_There;
                              Tip := Driver.Robot.Hand.Lobes.Tip_Track (L.There, L.Centre_There, L.Tip_There);
                              R := High_R;
                           else
                              declare
                                 At_Stop : constant Sweeps.Stop_Tips := Sweeps.Tips_At_Stop (S, C, K);
                              begin
                                 Seen := Lobe_No in At_Stop'Range and then At_Stop (Lobe_No).Known;
                                 Tip := (if Seen
                                         then Driver.Robot.Hand.Lobes.Tip_Track
                                                (At_Stop (Lobe_No).Region, At_Stop (Lobe_No).Centre, At_Stop (Lobe_No).Tip)
                                         else (0.0, 0.0));
                                 R := Sweeps.Stop_Reading (S, C, K);
                              end;
                           end if;
                        end View;
                        Last_Seen : Boolean;
                        Last_Tip  : Driver.Images.Pixel;
                        Last_R    : Real;
                     begin
                        Limit := Driver.Alignment.Finest_Radius (Driver.Images.Width (L.Here), Driver.Images.Height (L.Here));
                        View ((if Open_Low then 0 else Stops + 1), Last_Seen, Last_Tip, Last_R);
                        for Step in 1 .. Stops + 1 loop
                           declare
                              Seen : Boolean;
                              Tip  : Driver.Images.Pixel;
                              R    : Real;
                           begin
                              View ((if Open_Low then Step else Stops + 1 - Step), Seen, Tip, R);
                              if Seen then
                                 if Last_Seen
                                   and then Sqrt ((Tip.U - Last_Tip.U) ** 2 + (Tip.V - Last_Tip.V) ** 2) > Limit
                                   and then not Unseen (Last_R, R)
                                 then
                                    Found := True;
                                    From := Last_R;
                                    To := R;
                                    Widest := Sqrt ((Tip.U - Last_Tip.U) ** 2 + (Tip.V - Last_Tip.V) ** 2);
                                    return;
                                 end if;
                                 Last_Seen := True;
                                 Last_Tip := Tip;
                                 Last_R := R;
                              end if;
                           end;
                        end loop;
                     end;
                  end loop;
               end;
            end;
         end Read_Gap;
         Added    : Natural := 0;
         Followed : Boolean := True;
         Why      : Ada.Strings.Unbounded.Unbounded_String;
         Arm_Is   : Group_Id'Base := 0;
         Arm_At   : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  the arm's readings when the ends were seen
         procedure Read_Ends_Pose (O : Observation) is
            P : constant Natural := Own_Pair (G);
         begin
            Arm_Is := 0;
            if P = 0 or else C > Sweeps.Channels (H.Data.Pairs (P).Sweep)
              or else not Sweeps.Has_Ends (H.Data.Pairs (P).Sweep, C)
            then
               return;
            end if;
            Arm_Is := Arm_Group (M, H.Data.Pairs (P).Arm);
            declare
               Rest  : constant Real_Array := Sweeps.End_Rest (H.Data.Pairs (P).Sweep, C);
               First : Natural := Rest'First;   --  the rest lays every other group's readings out in group order
            begin
               for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
                  if Other /= G then
                     if Other = Arm_Is then
                        Arm_At := Driver.Robot.Hand.Views.Reading_Holders.To_Holder
                          (Rest (First .. First + O.Readings.Element (Other)'Length - 1));
                     end if;
                     First := First + O.Readings.Element (Other)'Length;
                  end if;
               end loop;
            end;
         end Read_Ends_Pose;
      begin
         Hold_Beat (Read_Ends_Pose'Access);
         if Arm_Is > 0 and then not Arm_At.Is_Empty then
            Move_Group (Arm_Is, Arm_At.Element);
         end if;
         loop
            Hold_Beat (Read_Gap'Access);
            exit when not Measured or else not Found;
            declare
               Parts  : constant Positive := Positive (Real'Ceiling (Widest / Limit));
               Before : constant Natural := Stops;
               Wait_At_Most : constant Positive := Driver.Robot.Hand.Wait_For_A_View (Longest_Formed, Settles, Began);
               Last   : Real := From;
               Gathered : Boolean := False;
               Jumped : Boolean := False;   --  the gap was passed over: no part of it is apart from the one before
               procedure Read_Gathered (O : Observation) is
                  pragma Unreferenced (O);
                  P : constant Natural := Own_Pair (G);
               begin
                  Gathered := P > 0 and then Sweeps.Gathered (H.Data.Pairs (P).Sweep);
               end Read_Gathered;
            begin
               for K in 1 .. Parts - 1 loop
                  declare
                     Part   : constant Real := From + (To - From) * Real (K) / Real (Parts);
                     Waited : Natural := 0;
                  begin
                     --  Readings no further apart than the channel's noise: the tip jumps there however finely the
                     --  closer stops, and the gap is passed over as one no stop sees through, the others cut (a14 on
                     --  10-16: 84 pixels between 6.559 and 6.562 ended the cutting with 13 pixels left uncut at the
                     --  open end).
                     if not Significant (Part - Last, Sqrt (2.0) * Sigma) then
                        Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                         & ": between " & Driver.Log.Image (From, 6) & " and " & Driver.Log.Image (To, 6)
                                         & ", where a tip moves" & Natural'Image (Natural (Widest)) & " pixels, a part of"
                                         & Parts'Image & " is no further from the one before than the channel's noise:"
                                         & " the tip jumps there, and the other gaps are cut");
                        Unseen_From.Append (From);
                        Unseen_To.Append (To);
                        Jumped := True;
                        exit;
                     end if;
                     Move_Channel (G, C, Part, Followed);
                     if not Followed then
                        Why := Ada.Strings.Unbounded.To_Unbounded_String
                          ("the closer did not follow to " & Driver.Log.Image (Part, 6));
                        exit;
                     end if;
                     loop
                        Hold_Beat (Read_Gathered'Access);
                        exit when Gathered or else Waited >= Wait_At_Most;
                        Waited := Waited + 1;
                     end loop;
                     if not Gathered then
                        Why := Ada.Strings.Unbounded.To_Unbounded_String
                          ("its view at " & Driver.Log.Image (Part, 6) & " did not form within" & Wait_At_Most'Image
                           & " beats");
                        exit;
                     end if;
                     Last := Part;
                     Added := Added + 1;
                  end;
               end loop;
               --  On to the far side of the gap: the closer leaves the last part's view, which is kept as a stop.
               if Ada.Strings.Unbounded.Length (Why) = 0 then
                  Move_Channel (G, C, To, Followed);
                  Hold_Beat (Read_Gap'Access);
               end if;
               exit when Ada.Strings.Unbounded.Length (Why) > 0 or else (Stops <= Before and then not Jumped);
               --  The gap found again between the same two views: the parts cut into it saw no tip of that lobe
               --  (Read_Gap passes over a stop without one), so the tip is not seen through the readings between and
               --  cutting finer gives nothing more to follow (its farthest pixel left the picture's border for its
               --  real tip, a03 on 10-16: 62 pixels between views however close their readings). Where the parts saw
               --  the tip and the widest step is now between two of them, the tip moves through those readings
               --  unevenly, and they are cut again until the channel's noise ends it: a gap left as wide after one cut
               --  was taken for a jump (a09 on 10-16: 25 pixels between 2.20 and 5.45 cut in two), and the lobe's
               --  shape was never followed.
               declare
                  Was_From : constant Real := From;
                  Was_To   : constant Real := To;
               begin
                  Hold_Beat (Read_Gap'Access);
                  if Found and then From = Was_From and then To = Was_To then
                     Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                      & ": between " & Driver.Log.Image (From, 6) & " and " & Driver.Log.Image (To, 6)
                                      & " a tip moves" & Natural'Image (Natural (Widest)) & " pixels, and the"
                                      & Parts'Image & " parts cut into it saw no tip: it is not seen through those"
                                      & " readings, and the other gaps are cut");
                     Unseen_From.Append (From);
                     Unseen_To.Append (To);
                  end if;
               end;
            end;
         end loop;
         if Measured then
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image & ": stopped at"
                             & Added'Image & " more readings between views its lobes' tips moved farther apart than the"
                             & " matcher searches at its finest (" & Driver.Log.Image (Limit, 1) & " pixels)"
                             & (if Ada.Strings.Unbounded.Length (Why) = 0 then "; every step now one it follows"
                                else "; it stops there: " & Ada.Strings.Unbounded.To_String (Why)));
            if Added > 0 then
               Return_Channel (G, C, Open_At);
            end if;
         end if;
      end Fill_Stops;

      procedure Sweep_Channel (G : Group_Id; C : Positive) is
         Start : Real := 0.0;
         Step  : Estimate;
         Pixel : Real := 0.0;   --  the push that moves the own eye's view by a pixel
         Still_A_Closer : Boolean := False;
         Began : Natural := 0;   --  the beats the stream had when the sweep began
         Settles : Natural := 0; --  the most beats the own eye has taken to settle after a move
         procedure Read_Start (O : Observation) is
            R : constant Real_Array := O.Readings.Element (G);
            P : constant Natural := Own_Pair (G);
         begin
            Began := Natural (O.Beat);
            Settles := (if P > 0 then Settling_Of (H.Data.Pairs (P).Eye) else 0);
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
                    Driver.Robot.Hand.Wait_For_A_View (Longest_Formed, Settles, Began);
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
            Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder
              (if R.Group > 0 then O.Readings.Element (R.Group) else Real_Array'(1 .. 0 => 0.0));
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
               --  The radius of the tip's region at the contact: the lobe's angular spread about its tip, at the tip's
               --  distance from the eye; nothing while the distance is not fitted.
               Distance  : constant Estimate := Driver.Robot.Hand.Tips.Distance (B, Lobe, Which);
               Footprint : constant Real :=
                 (if Known (Distance) then abs Distance.Value * R.Lobes (Lobe).Sights (Which).Spread else 0.0);
            begin
               Driver.Robot.Hand.Pressing.Aim_Reaching
                 (M, R.Arm, R.Eye, O, Along, Driver.Robot.Hand.Tips.Tip (B, Lobe, Which), Driver.Robot.Hand.Tips.Surface (B),
                  Aimed, Yaw, Reaches, Unmeasured_Aim, Why_Not, Footprint => Footprint);
            end;
         end Read_Aim;

         Off : Driver.Robot.Hand.Pressing.Aim_Offset;   --  where the aim's move left the tool, against the aim
         procedure Read_Reached (O : Observation) is
         begin
            Arm_Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
            Off := Driver.Robot.Hand.Pressing.Off_Aim (M, R.Arm, O, Aimed);
         end Read_Reached;

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
         Step_From : Driver.Robot.Hand.Views.Reading_Holders.Holder;   --  the arm's readings where the last step began
         Step_Reached : Boolean := False;   --  and the arm went the whole of it
         procedure Read_Lower (O : Observation) is
         begin
            Lost := not Present (Id, R);
            if Lost then
               return;
            end if;
            Plan := Driver.Robot.Hand.Pressing.Lowered (M, R.Arm, O, Aimed.Into, By);
            Stalls := H.Data.Found (Id).Stalls;
            Step_From := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
         end Read_Lower;

         Unplanned : Boolean := False;   --  a step could not be planned
         Felt      : Boolean := False;   --  the last step was stopped: the closer was pushed off the opening

         --  The closer was pushed off the opening the press is made at: its lobe met something, which the closer
         --  tells at once and the arm's readings only once the push has stalled (Motion.Step's Stop). A69's first
         --  press at open: the closer, held open, was pushed from 1.0 to 0.39 within six beats of the finger meeting
         --  the table, the push was judged blocked 77 beats later, the arm then crept 450 beats as the closer came
         --  back to 0.68, and the press was at no opening. Read between Next and Send, from the latest beat.
         function Closer_Left return Boolean is
            O    : constant Driver.Beats.Observation_View := Driver.Beats.Latest;
            Seen : Opening;
         begin
            return Present (Id, R) and then R.Group > 0 and then Driver.Observations.Has_Reading (O.all, R.Group)
              and then not (Opening_Of (R, M, O.Readings.Element (R.Group), Seen) and then Seen = Which);
         end Closer_Left;

         procedure Lower (Step : Real; Outcome : out Driver.Robot.Hand.Lowered) is
         begin
            By := Step;
            Step_Reached := False;
            Hold_Beat (Read_Lower'Access);
            if Lost then
               Outcome := Driver.Robot.Hand.Ended;
               return;
            end if;
            if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
               Unplanned := True;
               Outcome := Driver.Robot.Hand.Ended;
               return;
            end if;
            Driver.Robot.Motion.Follow (M, Plan, Report, Stop => Closer_Left'Access);
            Felt := Report.Stopped;
            Step_Reached := Report.Outcome = Driver.Robot.Motion.Reached and then not Report.Stopped;
            if Step_Reached then
               Descended := Descended + Step;
               Outcome := Driver.Robot.Hand.Reached;
            else
               Outcome := Driver.Robot.Hand.Met;
            end if;
         end Lower;

         --  Back to where the last step began, or to where the descent began, by the arm's readings there; the stall
         --  count read again where it stands, so that the next step's verdict is its own, and the closer brought back to
         --  the opening the press is made at when what the step met pushed it off. Reached is False when the arm or the
         --  closer does not get there.
         procedure Back (To_Start : Boolean; Reached : out Boolean) is
            procedure Read_Stalls (O : Observation) is
               pragma Unreferenced (O);
            begin
               Lost := not Present (Id, R);
               if not Lost then
                  Stalls := H.Data.Found (Id).Stalls;
               end if;
            end Read_Stalls;
            Goal : constant Real_Array := (if To_Start then Aim_At.Element else Step_From.Element);
            --  Every channel of the arm where it rests within its noise of Goal, or nearer than a step an eye can see.
            procedure Read_Arrival (O : Observation) is
               Now : constant Real_Array := O.Readings.Element (Arm_Is);
            begin
               Reached := Now'Length = Goal'Length
                 and then (for all C in 1 .. Now'Length =>
                             not Significant (Now (Now'First + C - 1) - Goal (Goal'First + C - 1),
                                              Sqrt (2.0) * Reading_Noise (M, Arm_Is, C))
                             or else (Known (Visible_Step (M, Arm_Is, C))
                                      and then abs (Now (Now'First + C - 1) - Goal (Goal'First + C - 1))
                                               < Visible_Step (M, Arm_Is, C).Value));
            end Read_Arrival;
            Waited : Natural;
         begin
            Move_Group (Arm_Is, Goal, Reached);
            --  A way back judged blocked can still get there: let go of what it pressed, the arm bounces, its first
            --  beats swing against the ask, and the step was given up as blocked while the arm went on to stand where
            --  it was sent (f16q on 10-15: the jaw's press at open, sent back to where its last step began, was given
            --  up blocked at its third beat and stood at its target by its tenth; "the arm did not get back", and
            --  nothing was pressed). Where the arm comes to rest is what tells.
            if not Reached then
               Driver.Robot.Motion.Settle (M, Waited);
               Hold_Beat (Read_Arrival'Access);
            end if;
            if To_Start then
               Descended := 0.0;
            elsif Step_Reached then
               Descended := Descended - By;
            end if;
            Hold_Beat (Read_Stalls'Access);
            --  The closer is judged where it rests: let go of what it pressed, it shakes for a while about its place
            --  (f16t on 10-15: 0.0016 units off its opening at the beat it was read, back on it between shakes).
            if Reached then
               Driver.Robot.Motion.Settle (M, Waited);
            end if;
            declare
               Arm_Back : constant Boolean := Reached;
            begin
               if Reached and then not Lost and then not Closer_At (R, Which) then
                  --  Landed where it was sent (Motion.Step's Land: a servo's steady error under its load sent again): a
                  --  closer let go of a press stood 0.001 units short of its opening, nothing its eye sees and more than
                  --  its readings' noise, and the press was refused (f16s on 10-15).
                  declare
                     Command : Driver.Commands.Command;
                     Report  : Driver.Robot.Motion.Step_Report;
                  begin
                     Driver.Commands.Set_Target (Command, R.Group, R.Readings (Which).Element);
                     Driver.Robot.Motion.Step (M, Command, Report, Land => True);
                     Driver.Robot.Motion.Settle (M, Waited);
                  end;
                  Reached := Closer_At (R, Which);
               end if;
               if not Reached and then not Lost then
                  declare
                     Off : Ada.Strings.Unbounded.Unbounded_String;
                     procedure Read_Off (O : Observation) is
                        Now : constant Real_Array := O.Readings.Element (Arm_Is);
                        Cl  : constant Real_Array := O.Readings.Element (R.Group);
                        Was : constant Real_Array := R.Readings (Which).Element;
                     begin
                        for C in 1 .. Natural'Min (Now'Length, Goal'Length) loop
                           Ada.Strings.Unbounded.Append
                             (Off, " c" & C'Image & " " & Driver.Log.Image (Now (Now'First + C - 1) - Goal (Goal'First + C - 1), 4));
                        end loop;
                        Ada.Strings.Unbounded.Append (Off, "; the closer off its opening by");
                        for C in 1 .. Natural'Min (Cl'Length, Was'Length) loop
                           Ada.Strings.Unbounded.Append
                             (Off, " " & Driver.Log.Image (Cl (Cl'First + C - 1) - Was (Was'First + C - 1), 6));
                        end loop;
                     end Read_Off;
                  begin
                     Hold_Beat (Read_Off'Access);
                     Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the way back "
                                      & (if Arm_Back then "took the arm there and not the closer"
                                         else "did not take the arm there")
                                      & "; the arm off where it was sent by" & Ada.Strings.Unbounded.To_String (Off));
                  end;
               end if;
            end;
            Reached := Reached and then not Lost;
            Felt := False;
         end Back;

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
                  --  How far the tool stands below where the descent began, by the arm's fit now at both readings.
                  declare
                     At_Start : Observation;
                  begin
                     At_Start.Readings := O.Readings;
                     At_Start.Readings.Replace_Element (Arm_Is, Aim_At.Element);
                     Result.Down := (Tool_In_Arm (M, R.Arm, O).Pose.Translation
                                     - Tool_In_Arm (M, R.Arm, At_Start).Pose.Translation) * Aimed.Into;
                  end;
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

         --  The last push, judged blocked, was stopped by a joint's end the arm has shown (Driver.Robot.Stopped_By_An_End),
         --  read in a held beat: the body's own stop, no contact.
         Ended_On_Joint : Boolean := False;
         procedure Read_End (O : Observation) is
         begin
            Ended_On_Joint := Driver.Robot.Stopped_By_An_End (M, R.Arm, O);
         end Read_End;

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
                             & (if Ada.Strings.Unbounded.Length (Why_Not) > 0
                                then " at the least rotation is not planned (" & Ada.Strings.Unbounded.To_String (Why_Not) & ")"
                                else " is taken where the arm's fit places the tool most surely")
                             & "; the hand turned about the way down by " & Driver.Log.Image (Yaw, 4) & " rad it is");
         end if;
         Plan := Aimed.Plan;
         Least := Real'Max (Real'Model_Epsilon, Aimed.Least);
         Driver.Robot.Motion.Follow (M, Plan, Report);
         Hold_Beat (Read_Reached'Access);
         --  An aim the arm did not complete is not a press: it stopped on something of its own or short of the pose,
         --  and lowering the hand from where it stands would press at no pose this aim chose (A27's first press of
         --  hand 2, the third joint at +0.07 for the -0.05 asked, lowered from there and fitted as the tip, 12.39 from
         --  the eye). The arm goes back to where the aim began, and the caller is told a tilt cannot be made. Whether
         --  it completed it is the tool's pose against the aim's (Pressing.At_Aim), whatever the joints' verdict.
         if not Driver.Robot.Hand.Pressing.At_Aim (Off) then
            Aim_Short := True;
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the arm did not reach the aim of a press of lobe" & Lobe'Image
                             & " at " & (if Which = Open then "open" else "closed") & ", turning the hand "
                             & Driver.Log.Image (Aimed.Turn, 4) & " rad (" & Ada.Strings.Unbounded.To_String (Report.Detail)
                             & " the tool stands " & Driver.Log.Image (Off.Place, 5) & " from the aim's place, told against "
                             & Driver.Log.Image (Off.Place_Sigma, 5) & ", and turned " & Driver.Log.Image (Off.Turn, 5)
                             & " rad from its turn, told against " & Driver.Log.Image (Off.Turn_Sigma, 5)
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
         Driver.Robot.Hand.Descend (Above'Access, Least, Lower'Access, Back'Access, Steps);
         if Lost then
            End_Lost (Moved => True);
            return False;
         end if;
         if Steps.Stalled then
            --  The last step was followed and took the hand nowhere: it is not part of the lowering.
            Descended := Descended - By;
         elsif not Unplanned and then not Steps.Spent and then not Steps.Unbacked then
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
            --  A push stopped at a joint's end the arm has shown is the body's own stop, wherever the tip stands: no press is
            --  made of it and none is waited for; the tilt is one that cannot be lowered, and half of it is tried.
            Hold_Beat (Read_End'Access);
            if Ended_On_Joint then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the descent of a press of lobe" & Lobe'Image & " at "
                                & (if Which = Open then "open" else "closed") & " stopped on an end the arm has shown (the"
                                & " channel that stopped it stands at its end): no press is made of it, the stop is the"
                                & " body's, and the arm is taken back to where the descent began");
               Hold_Beat (Note_The_Stop'Access);
               Aim_Short := True;
               Hold_Beat (Read_Arm'Access);
               Move_Group (Arm_Is, Aim_At.Element);
               return False;
            end if;
         end if;
         --  One line a press, for the boot's account of where its time went:
         --  how many pushes, and why each was as long as it was.
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": press of lobe" & Lobe'Image & " at "
                          & (if Which = Open then "open" else "closed") & ":"
                          & Natural'Image (Driver.Robot.Hand.Total (Steps)) & " pushes,"
                          & Steps.Fast'Image & " fast and" & Steps.Band'Image
                          & " within Z sigma of the contact its presses predict," & Steps.Blind'Image
                          & " doubling from " & Driver.Log.Image (Least, 4) & " with nothing predicting it (before a prediction, or past its band),"
                          & Steps.Capped'Image & " cut to the eye's room above the table," & Steps.Probes'Image
                          & " halving the span the contact lay in once a step met something more than the least push past where"
                          & " nothing was, the arm taken back" & Steps.Backs'Image & " times, and" & Steps.Pressed'Image
                          & " from where the descent began to no more than the least push past that span; "
                          & (if Unplanned then "then it cannot press lower: " & Driver.Robot.Motion.Why (Plan)
                             elsif Steps.Unbacked then "then the arm did not get back from what a step met: nothing is pressed"
                             elsif Steps.Spent then "then the eye has no room left above the table, or the steps that cover it were made,"
                                  & " and nothing was met, after lowering "
                                  & Driver.Log.Image (Descended, 4)
                             elsif Steps.Stalled then "stalled, the arm followed the last push by " & Driver.Log.Image (By, 4)
                                  & " and the hand did not go down with it, after lowering " & Driver.Log.Image (Descended, 4)
                             elsif Felt then "stopped where the closer was pushed off its opening, the last push by "
                                  & Driver.Log.Image (By, 4) & " after lowering " & Driver.Log.Image (Descended, 4)
                             else "blocked, the last push by " & Driver.Log.Image (By, 4) & " after lowering "
                                  & Driver.Log.Image (Descended, 4)));
         if Unplanned or else Steps.Spent or else Steps.Unbacked then
            --  Nothing was met and no more steps are made, or the next step is one the arm cannot be taken through (an
            --  end a channel showed is on the way down, with nothing predicting the contact to have told so before): the
            --  hand goes back to where the descent began, as after a press, for the presses after it begin there. A
            --  tilt that cannot be lowered is one that cannot be made, and half of it is tried: one the arm cannot be
            --  taken through (Unplanned), and one whose eye came within the least push of the table with nothing met
            --  (Spent), a lesser tilt keeping the eye higher over the tip. Spent was not one: a17's tilted presses of
            --  lobe 1 at open (10-16), its sight 1.287 rad from straight down and its tip 12.5 above the surface, were
            --  lowered 8.2 and 9.5 until the eye had no room left, each side gave up there, and the tip at open was
            --  never confirmed, the hand's depth never measured.
            Aim_Short := Unplanned or else Steps.Spent;
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
            Longest      : Natural;
            Waited       : Natural := 0;
            --  A press stopped where the closer was pushed off its opening (Felt) ended its push unblocked: the
            --  watcher is told what it met, the tool where the stop left it (Presses.Met).
            procedure Mark_Met (O : Observation) is
            begin
               Lost := not Present (Id, R);
               if not Lost then
                  Driver.Robot.Hand.Presses.Met (H.Data.Found (Id).Watch, Tool_In_Arm (M, R.Arm, O));
               end if;
            end Mark_Met;
         begin
            if Felt then
               Hold_Beat (Mark_Met'Access);
               if Lost then
                  End_Lost (Moved => True);
                  return False;
               end if;
            end if;
            Hold_Beat (Read_Rests'Access);
            if Lost then
               End_Lost (Moved => True);
               return False;
            end if;
            Rests_Before := Rests_Now;
            --  The stream's length at the let-go: read again at every beat of the wait, it grew with the wait and never
            --  bounded it (A86's hand 2 waited 5,600 beats, to the end of the run).
            Longest := Allowed;
            Hold_Beat (Read_Arm'Access);
            Move_Group (Arm_Is, Arm_Now.Element);
            loop
               Hold_Beat (Read_Rests'Access);
               exit when Lost or else Rests_Now > Rests_Before or else Waited >= Longest;
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
         --  The lobe of a tool no closer moves has no line of sight: it is pressed along the way down, the
         --  surface's normal turned into the tool frame where the tool stands, read in a held beat; without a
         --  surface known there is no way down and no press.
         Lined      : constant Boolean := R.Lobes (Lobe).Channel > 0;
         Sight      : Vec3 := (if Lined then R.Lobes (Lobe).Sights (Which).Ray.Direction.Unit_Vector else Zero3);
         Down_Known : Boolean := Lined;
         procedure Read_Down (O : Observation) is
            Fitted  : constant Driver.Geometry.Plane_Estimate := Driver.Robot.Hand.Tips.Surface (H.Data.Found (Id).Book);
            --  Before any press has fitted it, the table the arm's eye saw (Table_In_Arm), in the arm's frame as the
            --  presses are, which they then correct (Watch). The book's own surface alone comes from presses, and a
            --  tool that is pressed only along the way down never had a first one (f15t on 10-15: arm 1 fitted, its
            --  table seen, its tool "not pressed: no surface is known to press it on", the hand never measured).
            Surface : constant Driver.Geometry.Plane_Estimate :=
              (if Driver.Geometry.Known (Fitted) then Fitted else Table_In_Arm (M, R.Arm));
         begin
            Down_Known := Present (Id, R) and then Driver.Geometry.Known (Surface) and then abs Surface.Normal > 0.0;
            if Down_Known then
               Sight := Transpose (Tool_In_Arm (M, R.Arm, O).Pose.Rotation) * (-Unit (Surface.Normal));
            end if;
         end Read_Down;
         Lines  : Driver.Robot.Hand.Aims.Direction_Array (1 .. Natural (R.Lobes.Length));
         K      : Natural := 0;
      begin
         if not Lined then
            Hold_Beat (Read_Down'Access);
            if not Down_Known then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": the tool of arm" & R.Arm'Image
                                & " is not pressed: no surface is known to press it on");
               return;
            end if;
         end if;
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
               --  A lobe with no line of its own is tilted through the whole quarter turn: nothing of its tip's
               --  direction is known, and a tip anywhere needs presses from three directions (Touch).
               elsif not Lined then Ada.Numerics.Pi / 2.0
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
         --  The hand an arm's tool is has no channel to sweep.
         if Closer = 0 then
            return;
         end if;
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
            for C in 1 .. Channels loop
               Fill_Stops (Closer, C);
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
               There : constant Natural := (if Closer > 0 then Index_Of (Closer) else Index_Of_Tool (Key.Arm));
            begin
               Here := There > 0
                 and then (Closer = 0
                           or else (Role (M, Closer) = Driver.Robot.Closer
                                    and then Closer_Arm (M, Closer) = H.Data.Found (Hand_Id (There)).Arm));
               if Here then
                  Id := Hand_Id (There);
                  R := H.Data.Found (Id);
               end if;
            end Read_Hand;
         begin
            Lost := False;
            --  The hand an arm's tool is has one setting: it is pressed at the first.
            exit when Closer = 0 and then Which = Closed_Empty;
            Hold_Beat (Read_Hand'Access);
            if Here then
               if R.Group > 0 then
                  Move_Group (R.Group, R.Readings (Which).Element);
               end if;
               for L in 1 .. Natural (R.Lobes.Length) loop
                  if R.Lobes (L).Sights (Which).Known or else R.Lobes (L).Channel = 0 then
                     Press_Lobe (Id, R, L, Which);
                     exit when Lost;
                  else
                     Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & L'Image & " at "
                                      & (if Which = Open then "open" else "closed")
                                      & " is not pressed: its line of sight at this opening is not known (its tip is not seen in"
                                      & " the hand's eye there, or the arm is not fitted, so the eye has no place in the tool frame)");
                  end if;
               end loop;
            elsif Which = Open then
               Driver.Log.Line (Driver.Log.Robot,
                                (if Closer > 0
                                 then "hand: the hand of closer group" & Closer'Image & " is not pressed:"
                                      & " no hand is made of it, or its group is no longer a closer of its arm"
                                 else "hand: the tool of arm" & Key.Arm'Image & " is not pressed: no hand is made of it"
                                      & " (the arm has a closer, or is not fitted)"));
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

   --  The hands to measure: every group the body takes for a closer, or whose closer an eye on its arm watches,
   --  and every hand already made (of a closer, or of an arm's tool, which has none), when the hands begin.
   Keys : Key_Lists.Vector;
   function Has_Key (G : Group_Id'Base; A : Arm_Id'Base) return Boolean is
     (for some K of Keys => K.Group = G and then (G > 0 or else K.Arm = A));
   procedure Read_Closers (O : Observation) is
      pragma Unreferenced (O);
   begin
      Keys.Clear;
      for G in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Sweepable (H, M, G) or else Role (M, G) = Closer then
            Keys.Append (Hand_Key'(Group => G, Arm => Closer_Arm (M, G)));
         end if;
      end loop;
      for I in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         if not Has_Key (H.Data.Found (I).Group, H.Data.Found (I).Arm) then
            Keys.Append (Hand_Key'(Group => H.Data.Found (I).Group, Arm => H.Data.Found (I).Arm));
         end if;
      end loop;
   end Read_Closers;

   function Image (K : Hand_Key) return String is
     (if K.Group > 0 then "the hand of closer group" & K.Group'Image else "the tool of arm" & K.Arm'Image);

   procedure Hand_Lane (Lane : Positive) is
   begin
      One_Hand (Keys (Lane), Press_Part);
   end Hand_Lane;

   procedure Hands_At_Once is new Driver.Beats.At_Once (Hand_Lane);

begin
   if H.Data = null then
      H.Data := new Hand_Data;
   end if;
   Hold_Beat (Read_Closers'Access);
   if Keys.Is_Empty then
      Driver.Log.Line (Driver.Log.Robot, "hand: no closer and no tool hand was found, so nothing is swept or pressed");
   else
      for K of Keys loop
         One_Hand (K, Sweep_Part);
      end loop;
      if Natural (Keys.Length) = 1 then
         One_Hand (Keys.First_Element, Press_Part);
      else
         for K of Keys loop
            Driver.Log.Line (Driver.Log.Robot, "hand: " & Image (K) & " is pressed with the"
                             & Natural'Image (Natural (Keys.Length)) & " hands at once");
         end loop;
         Hands_At_Once (Positive (Keys.Length));
      end if;
   end if;
   Hold_Beat (Read_Description'Access);
   Driver.Log.Line (Driver.Log.Robot, "hand: measured" & ASCII.LF & Ada.Strings.Unbounded.To_String (Text));
end Measure;
