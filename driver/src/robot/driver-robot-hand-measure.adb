with Driver.Beats;
with Driver.Geometry;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;

--  The decider that measures the hands. Every closer channel is swept from
--  where it is to both ends of its travel, pushing further with doubling
--  steps from the smallest step the eyes see for as long as each step shows
--  the eye something the last end did not; the estimators find the lobes from
--  the views that leaves (Hand.Observe). Then every lobe's tip is pressed at
--  both openings onto whatever lies below the hand along gravity: aimed by a
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
         if H.Data.Pairs (I).Group = G and then H.Data.Pairs (I).Own then
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

   procedure Sweep_Channel (G : Group_Id; C : Positive) is
      Start : Real := 0.0;
      Step  : Estimate;
      Pixel : Real := 0.0;   --  the push that moves the own eye's view by a pixel
      Never : Ada.Strings.Unbounded.Unbounded_String;
      Still_A_Closer : Boolean := False;
      procedure Read_Start (O : Observation) is
         R : constant Real_Array := O.Readings.Element (G);
         P : constant Natural := Own_Pair (G);
      begin
         Start := R (R'First + C - 1);
         Step := Visible_Step (M, G, C);
         Pixel := (if P > 0 then Seen_By (Driver.Robot.Lockin.Shift (M, H.Data.Pairs (P).Eye, G, C)) else 0.0);
         Never := Ada.Strings.Unbounded.To_Unbounded_String
           (if P > 0 then Sweeps.Refusal (H.Data.Pairs (P).Sweep) else "");
         Still_A_Closer := Sweepable (H, M, G);
      end Read_Start;
   begin
      Hold_Beat (Read_Start'Access);
      if not Still_A_Closer then
         Driver.Log.Line (Driver.Log.Robot, "hand: group" & G'Image & " is no longer a closer the hand watches;"
                          & " channel" & C'Image & " not swept");
         return;
      end if;
      if Ada.Strings.Unbounded.Length (Never) > 0 then
         Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                          & " not swept: the instrument can never answer ("
                          & Ada.Strings.Unbounded.To_String (Never) & ")");
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
            begin
               Gone := P = 0 or else not Sweepable (H, M, G);
               if Gone then
                  Result := Driver.Robot.Hand.Nothing_New;
               elsif Sweeps.Gathered (H.Data.Pairs (P).Sweep) then
                  Result := (if Sweeps.Would_Extend (H.Data.Pairs (P).Sweep, C) then Driver.Robot.Hand.Something_New
                             else Driver.Robot.Hand.Nothing_New);
               end if;
            end Read_Shows;
         begin
            Hold_Beat (Read_Shows'Access);
            return Result;
         end Shows;
      begin
         for Way of Real_Array'[-1.0, 1.0] loop
            declare
               Pushes, Unseen : Natural;
               Answered       : Boolean;
            begin
               Driver.Robot.Hand.Sweep_Way (Way, Step.Value, Pixel, Push'Access, Shows'Access, Pushes, Unseen,
                                            Answered);
               Answered_Ways := Answered_Ways + Boolean'Pos (Answered);
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                                & (if Way < 0.0 then " down" else " up") & ":" & Pushes'Image & " pushes,"
                                & Unseen'Image & " of them before its views showed it anything; "
                                & (if Answered then "following from the first" else "at its end there"));
            end;
            Move_Channel (G, C, Start);
         end loop;
         if Gone then
            Driver.Log.Line (Driver.Log.Robot, "hand: group" & G'Image & " is no longer a closer the hand watches;"
                             & " its sweep of channel" & C'Image & " ends");
            return;
         end if;
         if Answered_Ways = 0 then
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                             & " follows neither way by its visible step; it is stuck, and not swept");
            return;
         end if;
      end;
      --  The estimators ask the instrument once both ends are seen; wait for
      --  its answer, then open the channel.
      declare
         Pending    : Boolean := True;
         Open_At    : Real := Start;
         Known_Open : Boolean := False;
         procedure Read_Pending (O : Observation) is
            pragma Unreferenced (O);
            P : constant Natural := Own_Pair (G);
         begin
            Pending := P > 0 and then (Sweeps.Status (H.Data.Pairs (P).Sweep, C) = Sweeps.Requested
                                       or else Sweeps.Wants_Correspondences (H.Data.Pairs (P).Sweep, C));
            if P > 0 and then Sweeps.Status (H.Data.Pairs (P).Sweep, C) = Sweeps.Measured
              and then Sweeps.Closing_Known (H.Data.Pairs (P).Sweep, C)
            then
               Known_Open := True;
               Open_At := Driver.Robot.Hand.Views.Reading
                 ((if Sweeps.Closed_End_Is_High (H.Data.Pairs (P).Sweep, C)
                   then Sweeps.Low_End (H.Data.Pairs (P).Sweep, C) else Sweeps.High_End (H.Data.Pairs (P).Sweep, C)), C);
            end if;
         end Read_Pending;
      begin
         loop
            Hold_Beat (Read_Pending'Access);
            exit when not Pending;
         end loop;
         if Known_Open then
            Move_Channel (G, C, Open_At);
         end if;
      end;
   end Sweep_Channel;

   --  One press of a lobe at an opening: the hand turned about the eye so
   --  that Along points down along gravity, then lowered until blocked
   --  (Driver.Robot.Hand.Descend), let go, and lifted back. The contact is
   --  predicted once the presses so far fix the lobe's tip and the surface
   --  (Tips.Tip, Tips.Surface): its height above that plane along Up, its
   --  sigma the plane's there with the tip's and the tool pose's. Before that
   --  nothing predicts it (a lobe's tip rides with its eye, so no view of the
   --  surface tells how far below the tip it is) and the descent creeps.
   --  False when the arm cannot reach it or the body does not say where down is.
   function Press_Once (Id : Hand_Id; R : Hand_Record; Lobe : Positive; Which : Opening; Along : Vec3) return Boolean is
      Above  : Rigid;
      Plan   : Driver.Robot.Motion.Plan;
      Report : Driver.Robot.Motion.Step_Report;
      Into   : Vec3 := Zero3;
      Least  : Real := Real'Last;   --  the smallest move of the tool that tells from its noise
      Ok     : Boolean := False;
      Arm_Is : Group_Id;
      Arm_Now : Driver.Robot.Hand.Views.Reading_Holders.Holder;

      procedure Read_Aim (O : Observation) is
         Tool    : constant Pose_Estimate := Tool_Pose (M, R.Arm, O);
         Down    : constant Direction_Estimate := Up (M);
         Values  : Vec3;
         Vectors : Mat3;
      begin
         Ok := Down.Sigma < Real'Last and then Tool.Position_Covariance (1, 1) < Real'Last
           and then Eye_Mount (M, R.Eye).Kind = Arm_Carried;
         if not Ok then
            return;
         end if;
         Into := -Down.Unit_Vector;
         Symmetric_Eigensystem (Tool.Position_Covariance, Values, Vectors);
         Least := Threshold (Vector_Gate (Vec3'Length))
           * Sqrt (Real'Max (Values (1), Real'Max (Values (2), Values (3))));
         Above := Driver.Robot.Hand.Aims.Turned_About
           (Tool.Pose, Eye_In_Tool (M, R.Eye, O).Pose.Translation, Along, Into);
         Plan := Driver.Robot.Motion.Plan_Reach (M, R.Arm, O, (Pose => Above, Position_Only => False));
      end Read_Aim;

      By : Real := 0.0;
      procedure Read_Lower (O : Observation) is
         Tool : constant Pose_Estimate := Tool_Pose (M, R.Arm, O);
      begin
         Plan := Driver.Robot.Motion.Plan_Reach
           (M, R.Arm, O, (Pose          => (Rotation => Tool.Pose.Rotation, Translation => Tool.Pose.Translation + By * Into),
                          Position_Only => False));
      end Read_Lower;

      Unplanned : Boolean := False;   --  a step could not be planned
      procedure Lower (Step : Real; Reached : out Boolean) is
      begin
         By := Step;
         Hold_Beat (Read_Lower'Access);
         if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
            Unplanned := True;
            Reached := False;
            return;
         end if;
         Driver.Robot.Motion.Follow (M, Plan, Report);
         Reached := Report.Outcome = Driver.Robot.Motion.Reached;
      end Lower;

      function Gap return Estimate is
         --  The lobe's tip above the surface the presses so far fixed, along Up.
         Result : Estimate := Unknown;
         procedure Read_Gap (O : Observation) is
            B    : Driver.Robot.Hand.Tips.Book renames H.Data.Found (Id).Book;
            Tip  : constant Point_Estimate := Driver.Robot.Hand.Tips.Tip (B, Lobe, Which);
            P    : constant Driver.Geometry.Plane_Estimate := Driver.Robot.Hand.Tips.Surface (B);
            Tool : constant Pose_Estimate := Tool_Pose (M, R.Arm, O);
         begin
            if Known (Tip) and then Driver.Geometry.Known (P) and then Tool.Position_Covariance (1, 1) < Real'Last then
               declare
                  Rot    : constant Mat3 := Tool.Pose.Rotation;
                  Here   : constant Vec3 := Tool.Pose * Tip.Mean;
                  --  A turn of the tool by a small rotation vector w moves the
                  --  tip by w x (R Tip), so its height by w . Lever.
                  Lever  : constant Vec3 := Cross (Rot * Tip.Mean, P.Normal);
                  On_It  : constant Estimate :=
                    Driver.Geometry.Height
                      (P, Point_Estimate'(Mean       => Here,
                                          Covariance => Rot * Tip.Covariance * Transpose (Rot)
                                                        + Tool.Position_Covariance));
                  Lean   : constant Real := P.Normal * (-Into);
               begin
                  if Lean > 0.0 then
                     Result := (Value              => On_It.Value / Lean,
                                Sigma              => Sqrt (On_It.Sigma ** 2
                                                            + Lever * (Tool.Rotation_Covariance * Lever)) / Lean,
                                Degrees_Of_Freedom => On_It.Degrees_Of_Freedom);
                  end if;
               end;
            end if;
         end Read_Gap;
      begin
         Hold_Beat (Read_Gap'Access);
         return Result;
      end Gap;
      Steps : Driver.Robot.Hand.Descent_Steps;

      procedure Read_Arm (O : Observation) is
      begin
         Arm_Is := Arm_Group (M, R.Arm);
         Arm_Now := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (O.Readings.Element (Arm_Is));
      end Read_Arm;

      procedure Read_Back (O : Observation) is
      begin
         Plan := Driver.Robot.Motion.Plan_Reach (M, R.Arm, O, (Pose => Above, Position_Only => False));
      end Read_Back;
   begin
      Hold_Beat (Read_Aim'Access);
      if not Ok then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": up, the arm's pose or the eye's mount is unmeasured;"
                          & " no press");
         return False;
      end if;
      if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": cannot aim a press: " & Driver.Robot.Motion.Why (Plan));
         return False;
      end if;
      Driver.Robot.Motion.Follow (M, Plan, Report);
      declare
         First : constant Estimate := Gap;
      begin
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": pressing lobe" & Lobe'Image & " at "
                          & (if Which = Open then "open" else "closed")
                          & (if Known (First)
                             then ", " & Driver.Log.Image (First.Value, 4) & " +- " & Driver.Log.Image (First.Sigma, 4)
                                  & " above the surface the presses so far fixed"
                             else ", nothing yet predicting the surface below its tip: by "
                                  & Driver.Log.Image (Least, 4) & " a step until blocked"));
      end;
      Driver.Robot.Hand.Descend (Gap'Access, Least, Lower'Access, Steps);
      --  One line a press, for the boot's account of where its time went:
      --  how many pushes, and why each was as long as it was.
      Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": press of lobe" & Lobe'Image & " at "
                       & (if Which = Open then "open" else "closed") & ":"
                       & Natural'Image (Driver.Robot.Hand.Total (Steps)) & " pushes,"
                       & Steps.Fast'Image & " fast and" & Steps.Band'Image
                       & " within Z sigma of the contact its presses predict," & Steps.Crept'Image
                       & " crept by " & Driver.Log.Image (Least, 4) & " with nothing predicting it; "
                       & (if Unplanned then "then it cannot press lower: " & Driver.Robot.Motion.Why (Plan)
                          else "blocked, the last push by " & Driver.Log.Image (By, 4)));
      if Unplanned then
         return False;
      end if;
      --  Let go: the arm held where the block left it, so the hand rests.
      Hold_Beat (Read_Arm'Access);
      Move_Group (Arm_Is, Arm_Now.Element);
      Hold_Beat (Read_Back'Access);
      if Driver.Robot.Motion.Status (Plan) = Driver.Robot.Motion.Planned then
         Driver.Robot.Motion.Follow (M, Plan, Report);
      end if;
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
         Agreed : Boolean := True;
         procedure Read_Agreed (O : Observation) is
            pragma Unreferenced (O);
         begin
            Agreed := Driver.Robot.Hand.Tips.Latest_Agrees (H.Data.Found (Id).Book);
         end Read_Agreed;
      begin
         if not Press_Once (Id, R, Lobe, Which, Sight) or else Scale <= 0.0 then
            return;
         end if;
         --  Leaning to either side of away, half-way to across.
         for Side of Real_Array'[-1.0, 1.0] loop
            declare
               Lean : constant Vec3 := Exp ((Side * Ada.Numerics.Pi / 4.0) * Sight) * Away;
               Tilt : Real := Scale;
            begin
               while Tilt < Ada.Numerics.Pi / 2.0 loop
                  exit when not Press_Once (Id, R, Lobe, Which, Driver.Robot.Hand.Aims.Tilted (Sight, Lean, Tilt));
                  Hold_Beat (Read_Agreed'Access);
                  exit when not Agreed;
                  Tilt := 2.0 * Tilt;
               end loop;
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
         Channels : Natural := 0;
         procedure Read_Group (O : Observation) is
            pragma Unreferenced (O);
            P : constant Natural := Own_Pair (G);
         begin
            Now := Sweepable (H, M, G);
            Channels := (if P > 0 then Sweeps.Channels (H.Data.Pairs (P).Sweep) else 0);
         end Read_Group;
      begin
         Hold_Beat (Read_Group'Access);
         if Now then
            for C in 1 .. Channels loop
               Sweep_Channel (G, C);
            end loop;
         end if;
      end;
   end loop;
   --  Press every lobe of every hand found, at both openings: a hand whose
   --  group the body no longer takes for a closer of its arm is gone by then.
   Hold_Beat (Read_Hands'Access);
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
                  end if;
               end loop;
            end if;
         end;
      end loop;
   end loop;
   Hold_Beat (Read_Description'Access);
   Driver.Log.Line (Driver.Log.Robot, "hand: measured" & ASCII.LF & Ada.Strings.Unbounded.To_String (Text));
end Measure;
