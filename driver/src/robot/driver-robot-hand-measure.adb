with Driver.Beats;
with Driver.Robot.Hand.Aims;
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

   procedure Move_Group (G : Group_Id; Target : Real_Array) is
      --  One step of the group to Target, settled, and one more still beat so
      --  the eye's view there has two frames.
      Command : Driver.Commands.Command;
      Report  : Driver.Robot.Motion.Step_Report;
   begin
      Driver.Commands.Set_Target (Command, G, Target);
      Driver.Robot.Motion.Step (M, Command, Report);
      Hold_Beat (null);
   end Move_Group;

   procedure Move_Channel (G : Group_Id; Channel : Positive; To : Real) is
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
         Move_Group (G, Target);
      end;
   end Move_Channel;

   procedure Sweep_Channel (G : Group_Id; C : Positive) is
      Start : Real := 0.0;
      Step  : Estimate;
      procedure Read_Start (O : Observation) is
         R : constant Real_Array := O.Readings.Element (G);
      begin
         Start := R (R'First + C - 1);
         Step := Visible_Step (M, G, C);
      end Read_Start;
   begin
      Hold_Beat (Read_Start'Access);
      if not Known (Step) then
         Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " channel" & C'Image
                          & " has no visible step measured; not swept");
         return;
      end if;
      for Way of Real_Array'[-1.0, 1.0] loop
         declare
            Offset  : Real := Step.Value;
            Extends : Boolean := True;
            procedure Read_Extends (O : Observation) is
               pragma Unreferenced (O);
               P : constant Natural := Own_Pair (G);
            begin
               Extends := P > 0 and then Sweeps.Would_Extend (H.Data.Pairs (P).Sweep, C);
            end Read_Extends;
         begin
            while Extends loop
               Move_Channel (G, C, Start + Way * Offset);
               Hold_Beat (Read_Extends'Access);
               Offset := 2.0 * Offset;
            end loop;
         end;
         Move_Channel (G, C, Start);
      end loop;
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

   --  One press: the hand turned about the eye so that Along points down
   --  along gravity, then lowered until blocked, let go, and lifted back.
   --  False when the arm cannot reach it or the body does not say where down is.
   function Press_Once (Id : Hand_Id; R : Hand_Record; Along : Vec3) return Boolean is
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
      By := Least;
      loop
         Hold_Beat (Read_Lower'Access);
         if Driver.Robot.Motion.Status (Plan) /= Driver.Robot.Motion.Planned then
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": cannot press lower: "
                             & Driver.Robot.Motion.Why (Plan));
            return False;
         end if;
         Driver.Robot.Motion.Follow (M, Plan, Report);
         exit when Report.Outcome /= Driver.Robot.Motion.Reached;
         By := 2.0 * By;
      end loop;
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
         if not Press_Once (Id, R, Sight) or else Scale <= 0.0 then
            return;
         end if;
         --  Leaning to either side of away, half-way to across.
         for Side of Real_Array'[-1.0, 1.0] loop
            declare
               Lean : constant Vec3 := Exp ((Side * Ada.Numerics.Pi / 4.0) * Sight) * Away;
               Tilt : Real := Scale;
            begin
               while Tilt < Ada.Numerics.Pi / 2.0 loop
                  exit when not Press_Once (Id, R, Driver.Robot.Hand.Aims.Tilted (Sight, Lean, Tilt));
                  Hold_Beat (Read_Agreed'Access);
                  exit when not Agreed;
                  Tilt := 2.0 * Tilt;
               end loop;
            end;
         end loop;
      end;
   end Press_Lobe;

   Count : Natural := 0;
   procedure Read_Pairs (O : Observation) is
      pragma Unreferenced (O);
   begin
      Count := Natural (H.Data.Pairs.Length);
   end Read_Pairs;
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
   --  Sweep every channel of every closer an eye on its arm watches.
   Hold_Beat (Read_Pairs'Access);
   for I in 1 .. Count loop
      declare
         G        : Group_Id;
         Own      : Boolean := False;
         Channels : Natural := 0;
         procedure Read_Pair (O : Observation) is
            pragma Unreferenced (O);
         begin
            G := H.Data.Pairs (I).Group;
            Own := H.Data.Pairs (I).Own;
            Channels := Sweeps.Channels (H.Data.Pairs (I).Sweep);
         end Read_Pair;
      begin
         Hold_Beat (Read_Pair'Access);
         if Own then
            for C in 1 .. Channels loop
               Sweep_Channel (G, C);
            end loop;
         end if;
      end;
   end loop;
   --  Press every lobe of every hand found, at both openings.
   Hold_Beat (Read_Hands'Access);
   for Id in 1 .. Hand_Id'Base (Count) loop
      for Which in Opening loop
         declare
            R : Hand_Record;
            procedure Read_Hand (O : Observation) is
               pragma Unreferenced (O);
            begin
               R := H.Data.Found (Id);
            end Read_Hand;
         begin
            Hold_Beat (Read_Hand'Access);
            Move_Group (R.Group, R.Readings (Which).Element);
            for L in 1 .. Natural (R.Lobes.Length) loop
               if R.Lobes (L).Sights (Which).Known then
                  Press_Lobe (Id, R, L, Which);
               end if;
            end loop;
         end;
      end loop;
   end loop;
   Hold_Beat (Read_Description'Access);
   Driver.Log.Line (Driver.Log.Robot, "hand: measured" & ASCII.LF & Ada.Strings.Unbounded.To_String (Text));
end Measure;
