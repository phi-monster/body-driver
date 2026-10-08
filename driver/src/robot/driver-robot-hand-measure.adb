with Driver.Beats;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Hand.Pressing;
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
         Up : constant Direction_Estimate := Up_In_Arm (M, Arm);
      begin
         Room := Unknown;
         if Up.Sigma = Real'Last or else Eye_Mount (M, Eye).Kind /= Arm_Carried then
            return;
         end if;
         By := (if First then Driver.Robot.Hand.Pressing.Least_Push (M, Arm, O) else 2.0 * By);
         Plan := Driver.Robot.Hand.Pressing.Lowered (M, Arm, O, Up.Unit_Vector, By);
         Room := Driver.Robot.Hand.Pressing.Gap
           (M, Arm, O, (Mean => Eye_In_Tool (M, Eye, O).Pose.Translation, Covariance => [others => [others => 0.0]]),
            Table_In_Arm (M, Arm), -Up.Unit_Vector);
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
            if P > 0 and then Sweeps.Status (H.Data.Pairs (P).Sweep, C) = Sweeps.Measured
              and then Sweeps.Closing_Known (H.Data.Pairs (P).Sweep, C)
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

      procedure Read_Aim (O : Observation) is
      begin
         Driver.Robot.Hand.Pressing.Aim (M, R.Arm, R.Eye, O, Along, Aimed);
      end Read_Aim;

      Least : Real;   --  the least push, where the aim leaves the tool, kept above zero so that the doubling begins
      By : Real := 0.0;
      Descended : Real := 0.0;   --  how far the pushes that were reached have lowered the tool
      procedure Read_Lower (O : Observation) is
      begin
         Plan := Driver.Robot.Hand.Pressing.Lowered (M, R.Arm, O, Aimed.Into, By);
      end Read_Lower;

      Unplanned : Boolean := False;   --  a step could not be planned
      Stalls    : Natural := 0;       --  the pushes the watcher judged stalled when the step began, and after it
      procedure Read_Stalls (O : Observation) is
         pragma Unreferenced (O);
      begin
         Stalls := H.Data.Found (Id).Stalls;
      end Read_Stalls;

      procedure Lower (Step : Real; Result : out Driver.Robot.Hand.Push_Result) is
         Before : Natural;
      begin
         By := Step;
         Hold_Beat (Read_Lower'Access);
         if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
            Unplanned := True;
            Result := Driver.Robot.Hand.Stopped;
            return;
         end if;
         Hold_Beat (Read_Stalls'Access);
         Before := Stalls;
         Driver.Robot.Motion.Follow (M, Plan, Report);
         --  The watcher judges the push from the stream (Driver.Robot.Hand.Lowering), as the press is found: one
         --  verdict, here as in a replay. A push that was Blocked is a stop whatever it said.
         Hold_Beat (Read_Stalls'Access);
         if Report.Outcome /= Driver.Robot.Motion.Reached then
            Result := Driver.Robot.Hand.Stopped;
         elsif Stalls > Before then
            Result := Driver.Robot.Hand.Stalled;
         else
            Result := Driver.Robot.Hand.Lowered;
            Descended := Descended + Step;
         end if;
      end Lower;

      function Above return Driver.Robot.Hand.Heights is
         --  The lobe's tip above the surface the presses so far fixed, and the
         --  eye above the table its arm's own eye saw, along the way down.
         Result : Driver.Robot.Hand.Heights;
         procedure Read_Above (O : Observation) is
            B   : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
            Eye : constant Point_Estimate :=
              (Mean => Eye_In_Tool (M, R.Eye, O).Pose.Translation, Covariance => [others => [others => 0.0]]);
         begin
            Result.Tip := Driver.Robot.Hand.Pressing.Gap
              (M, R.Arm, O, Driver.Robot.Hand.Tips.Tip (B, Lobe, Which), Driver.Robot.Hand.Tips.Surface (B), Aimed.Into);
            Result.Eye := Driver.Robot.Hand.Pressing.Gap (M, R.Arm, O, Eye, Table_In_Arm (M, R.Arm), Aimed.Into);
         end Read_Above;
      begin
         Hold_Beat (Read_Above'Access);
         return Result;
      end Above;
      Steps : Driver.Robot.Hand.Descent_Steps;

      procedure Read_Arm (O : Observation) is
      begin
         Arm_Is := Arm_Group (M, R.Arm);
         Arm_Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
      end Read_Arm;
   begin
      Hold_Beat (Read_Aim'Access);
      if not Aimed.Ok then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": up, the arm's pose or the eye's mount is unmeasured;"
                          & " no press");
         return False;
      end if;
      Plan := Aimed.Plan;
      Least := Real'Max (Real'Model_Epsilon, Aimed.Least);
      if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": cannot aim a press: " & Driver.Robot.Motion.Why (Plan));
         return False;
      end if;
      Driver.Robot.Motion.Follow (M, Plan, Report);
      Hold_Beat (Read_Arm'Access);
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
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": pressing lobe" & Lobe'Image & " at "
                          & (if Which = Open then "open" else "closed") & ", aimed by turning the hand "
                          & Driver.Log.Image (Aimed.Turn, 4) & " rad"
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
      if Unplanned then
         return False;
      end if;
      if Steps.Spent then
         --  Nothing was met and no more steps are made: the hand goes back to where the
         --  descent began, as after a press, for the presses after it begin there.
         Hold_Beat (Read_Arm'Access);
         Move_Group (Arm_Is, Aim_At.Element);
         return False;
      end if;
      --  Let go: the arm held where the block left it, so the hand rests; then
      --  back to where the descent began.
      Hold_Beat (Read_Arm'Access);
      Move_Group (Arm_Is, Arm_Now.Element);
      Move_Group (Arm_Is, Aim_At.Element);
      return True;
   end Press_Once;

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
            pragma Unreferenced (O);
            Book : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
         begin
            Agreed := Driver.Robot.Hand.Tips.Latest_Agrees (Book);
            Checked := Driver.Robot.Hand.Tips.Confirmed (Book, Lobe, Which);
            Least := Driver.Robot.Hand.Aims.Least_Tilt (Driver.Robot.Hand.Tips.Distance (Book, Lobe, Which));
         end Read_Agreed;
      begin
         Hold_Beat (Read_Agreed'Access);
         if Checked then
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & Lobe'Image & " at "
                             & (if Which = Open then "open" else "closed") & " is not pressed: its tip is confirmed already");
            return;
         end if;
         if not Press_Once (Id, R, Lobe, Which, Sight) then
            return;
         end if;
         Hold_Beat (Read_Agreed'Access);
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
               Tilt  : Real := Scale;
               Bound : Real := Ada.Numerics.Pi / 2.0;   --  the least tilt this side was found not to make
               Made  : Natural := 0;
               Why   : Ada.Strings.Unbounded.Unbounded_String :=
                 Ada.Strings.Unbounded.To_Unbounded_String ("tilted to a right angle");
            begin
               --  The hand's own angle, then double while the presses are ones
               --  the tip rests on, half when one stops short of the table
               --  (Driver.Robot.Hand.Aims.Next_Tilt).
               while Tilt > 0.0 and then Tilt < Bound loop
                  if not Press_Once (Id, R, Lobe, Which, Driver.Robot.Hand.Aims.Tilted (Sight, Lean, Tilt)) then
                     Why := Ada.Strings.Unbounded.To_Unbounded_String ("a press could not be made");
                     exit;
                  end if;
                  Made := Made + 1;
                  Hold_Beat (Read_Agreed'Access);
                  if Checked then
                     Why := Ada.Strings.Unbounded.To_Unbounded_String
                       ("the tip is confirmed: a press from another pose landed on it");
                     exit;
                  end if;
                  Driver.Robot.Hand.Aims.Next_Tilt (Tilt, Stalled => not Agreed, Bound => Bound, Least => Least);
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

   Count : Natural := 0;
   procedure Read_Groups (O : Observation) is
      pragma Unreferenced (O);
   begin
      Count := Group_Count (M);
   end Read_Groups;
   procedure Read_Hands (O : Observation) is
      pragma Unreferenced (O);
   begin
      Count := Natural (H.Data.Found.Length);
   end Read_Hands;
   Text : Ada.Strings.Unbounded.Unbounded_String;
   procedure Read_Description (O : Observation) is
      pragma Unreferenced (O);
   begin
      Text := Ada.Strings.Unbounded.To_Unbounded_String (Describe (H));
   end Read_Description;

begin
   if H.Data = null then
      H.Data := new Hand_Data;
   end if;
   --  Sweep every channel of every closer an eye on its arm watches, by the
   --  body's roles as they are when that closer's sweep begins: the boot
   --  re-reads them, and a group it no longer takes for a closer is not
   --  swept as one, while one it came to take for one is.
   Hold_Beat (Read_Groups'Access);
   for G in 1 .. Group_Id'Base (Count) loop
      declare
         Now      : Boolean := False;
         Is_One   : Boolean := False;   --  the body takes it for a closer
         Of_Arm   : Arm_Id'Base := 0;
         Channels : Natural := 0;
         procedure Read_Group (O : Observation) is
            pragma Unreferenced (O);
            P : constant Natural := Own_Pair (G);
         begin
            Now := Sweepable (H, M, G);
            Is_One := Role (M, G) = Closer;
            Of_Arm := Closer_Arm (M, G);
            Channels := (if P > 0 then Sweeps.Channels (H.Data.Pairs (P).Sweep) else 0);
         end Read_Group;
      begin
         Hold_Beat (Read_Group'Access);
         if Now then
            for C in 1 .. Channels loop
               Sweep_Channel (G, C);
            end loop;
         elsif Is_One then
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " of arm" & Of_Arm'Image
                             & " is not swept: no eye on its arm watches it, so no hand is made of it");
         end if;
      end;
   end loop;
   --  Press every lobe of every hand found, at both openings: a hand whose
   --  group the body no longer takes for a closer of its arm is gone by then.
   Hold_Beat (Read_Hands'Access);
   if Count = 0 then
      Driver.Log.Line (Driver.Log.Robot, "hand: no hand was found, so nothing is pressed; below, what became of each closer");
   end if;
   for Id in 1 .. Hand_Id'Base (Count) loop
      for Which in Opening loop
         declare
            R    : Hand_Record;
            Here : Boolean := False;
            procedure Read_Hand (O : Observation) is
               pragma Unreferenced (O);
            begin
               Here := Id <= H.Data.Found.Last_Index
                 and then Role (M, H.Data.Found (Id).Group) = Closer
                 and then Closer_Arm (M, H.Data.Found (Id).Group) = H.Data.Found (Id).Arm;
               if Here then
                  R := H.Data.Found (Id);
               end if;
            end Read_Hand;
         begin
            Hold_Beat (Read_Hand'Access);
            if Here then
               Move_Group (R.Group, R.Readings (Which).Element);
               for L in 1 .. Natural (R.Lobes.Length) loop
                  if R.Lobes (L).Sights (Which).Known then
                     Press_Lobe (Id, R, L, Which);
                  else
                     Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": lobe" & L'Image & " at "
                                      & (if Which = Open then "open" else "closed")
                                      & " is not pressed: its tip is not seen in the hand's eye at this opening");
                  end if;
               end loop;
            elsif Which = Open then
               Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & " is not pressed: its closer group is no longer"
                                & " a closer of its arm");
            end if;
         end;
      end loop;
   end loop;
   Hold_Beat (Read_Description'Access);
   Driver.Log.Line (Driver.Log.Robot, "hand: measured" & ASCII.LF & Ada.Strings.Unbounded.To_String (Text));
end Measure;
