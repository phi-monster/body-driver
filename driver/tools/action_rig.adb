with Driver.Action.Plants.Live;
with Driver.Beats;
with Driver.Clock;
with Driver.Images;
with Driver.Robot.Motion;

package body Action_Rig is

   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use Driver.Uncertain;
   use type Driver.Robot.Group_Id;
   use type Driver.Robot.Motion.Plan_Status;

   package Motion renames Driver.Robot.Motion;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   procedure Must_Be_Inside (P : Rig; What : String) is
   begin
      if not P.Inside then
         raise Program_Error with What & " asked outside a beat's window, where the models change";
      end if;
   end Must_Be_Inside;

   procedure Must_Be_Outside (P : Rig; What : String) is
   begin
      if P.Inside then
         raise Program_Error with What & " asked inside a beat's window, which holds the beat it would wait for";
      end if;
   end Must_Be_Outside;

   overriding procedure Look (P : in out Rig; S : out Driver.Action.Snapshots.Snapshot) is
      procedure During is
      begin
         P.Last := Driver.Beats.Latest.all;
         P.World.Look (S);
         --  The arm is the body's: its tool from its readings, with the fit's
         --  uncertainty and the step its eye can see.
         for K in 1 .. Natural (S.Arms.Length) loop
            if Natural (S.Arms (K).Id) <= Driver.Robot.Arm_Count (P.Robot.all) then
               declare
                  Plate : constant Driver.Action.Snapshots.Sample_Vectors.Vector := S.Arms (K).Surface;
                  Mine  : Driver.Action.Snapshots.Arm_State :=
                    Driver.Action.Plants.Live.Arm_Of (P.Robot.all, S.Arms (K).Id, P.Last);
               begin
                  --  The plate on the arm is the world's (the live plant measures no surface of an arm).
                  Mine.Surface := Plate;
                  S.Arms.Replace_Element (K, Mine);
               end;
            end if;
         end loop;
         S.Up := Driver.Robot.Up (P.Robot.all);
         S.Beat := P.Last.Beat;
      end During;
   begin
      Must_Be_Outside (P, "a look");
      if not P.Started then
         P.Episode := Driver.Beats.Episode;
         P.Started := True;
      end if;
      Driver.Beats.Within_A_Beat (During'Access);
   end Look;

   overriding procedure Within (P : in out Rig; During : not null access procedure) is
      procedure Held is
      begin
         P.Last := Driver.Beats.Latest.all;
         P.Inside := True;
         --  The simulated world keeps the same rule: it answers its views and
         --  predictions only inside its own window.
         P.World.Within (During);
         P.Inside := False;
      exception
         when others =>
            P.Inside := False;
            raise;
      end Held;
   begin
      Must_Be_Outside (P, "a window");
      Driver.Beats.Within_A_Beat (Held'Access);
   end Within;

   overriding function Reach (P : Rig; Goal : Plants.Arm_Goal) return Plants.Reach_Answer is
   begin
      Must_Be_Inside (P, "a reach");
      declare
         Plan : constant Motion.Plan :=
           Motion.Plan_Reach (P.Robot.all, Goal.Arm, P.Last, (Pose => Goal.Tool, Position_Only => Goal.Position_Only));
         Answer : constant Plants.Reach_Answer :=
           (case Motion.Status (Plan) is
               when Motion.Planned     => (Status => Plants.Reachable, Why => Null_Unbounded_String),
               when Motion.Unreachable =>
                 (Status => Plants.Unreachable, Why => To_Unbounded_String (Motion.Why (Plan))),
               when Motion.Unmeasured  =>
                 (Status => Plants.Unmeasured, Why => To_Unbounded_String (Motion.Why (Plan))));
      begin
         if Trace then
            Reach_Log.Append (Reach_Record'(From   => Driver.Robot.Tool_Pose (P.Robot.all, Goal.Arm, P.Last).Pose,
                                            Asked  => Goal.Tool,
                                            Status => Answer.Status,
                                            Why    => Answer.Why));
         end if;
         return Answer;
      end;
   end Reach;

   --  As the live plant takes the motion layer's verdicts.
   function Outcome_Of (R : Motion.Step_Report) return Plants.Step_Outcome is
     (case R.Outcome is
         when Motion.Blocked | Motion.Short => Plants.Blocked,
         when Motion.Reached =>
           (if Known (R.Delivered) and then R.Delivered.Value < 1.0
              and then Significant (1.0 - R.Delivered.Value, R.Delivered.Sigma, R.Delivered.Degrees_Of_Freedom)
            then Plants.Short else Plants.Reached));

   procedure Nothing is null;

   overriding procedure Move (P : in out Rig; O : Plants.Order; R : out Plants.Report) is
   begin
      Must_Be_Outside (P, "a move");
      R := (others => <>);
      if O.Arms.Is_Empty and then O.Closers.Is_Empty then
         Driver.Beats.Within_A_Beat (Nothing'Access);
         R.Beats := 1;
         return;
      end if;
      for G of O.Arms loop
         declare
            Plan : Motion.Plan;
            Rec  : Move_Record := (Asked => G.Tool, Reached => G.Tool, others => <>);
            procedure Planning is
            begin
               P.Last := Driver.Beats.Latest.all;
               Plan := Motion.Plan_Reach (P.Robot.all, G.Arm, P.Last,
                                          (Pose => G.Tool, Position_Only => G.Position_Only),
                                          Clearance => G.Clearance, Lever => G.Lever);
            end Planning;
            procedure Arrived is
            begin
               P.Last := Driver.Beats.Latest.all;
               Rec.Reached := Driver.Robot.Tool_Pose (P.Robot.all, G.Arm, P.Last).Pose;
            end Arrived;
         begin
            Driver.Beats.Within_A_Beat (Planning'Access);
            if Motion.Status (Plan) = Motion.Planned then
               declare
                  Step : Motion.Step_Report;
               begin
                  Motion.Follow (P.Robot.all, Plan, Step);
                  Rec.Outcome := Outcome_Of (Step);
                  Rec.Why := Step.Detail;
                  R.Arms.Append (Plants.Arm_Result'(Arm => G.Arm, Outcome => Rec.Outcome, Delivered => Step.Delivered,
                                                    Why => Step.Detail));
                  R.Beats := R.Beats + Step.Beats;
               end;
               Driver.Beats.Within_A_Beat (Arrived'Access);
            else
               Rec.Outcome := Plants.Refused;
               Rec.Why := To_Unbounded_String (Motion.Why (Plan));
               R.Arms.Append (Plants.Arm_Result'(Arm => G.Arm, Outcome => Plants.Refused, Delivered => Unknown,
                                                 Why => Rec.Why));
            end if;
            P.Moves.Append (Rec);
         end;
      end loop;
      for C of O.Closers loop
         declare
            Done    : Boolean := False;
            Stopped : Boolean := False;
            Beats   : Natural := 0;
            procedure Aiming is
            begin
               Sim.Set_Closer (P.World.all, C.Hand, C.Fraction);
            end Aiming;
            procedure Watching is
            begin
               Done := Sim.Closer_Done (P.World.all, C.Hand);
               Stopped := Sim.Closer_Stopped (P.World.all, C.Hand);
            end Watching;
         begin
            Driver.Beats.Within_A_Beat (Aiming'Access);
            while not Done loop
               Driver.Beats.Within_A_Beat (Watching'Access);
               Beats := Beats + 1;
            end loop;
            R.Closers.Append (Plants.Closer_Result'(Hand => C.Hand,
                                                    Outcome => (if Stopped then Plants.Blocked else Plants.Reached)));
            R.Beats := R.Beats + Beats;
         end;
      end loop;
   end Move;

   overriding function Predicted (P : Rig; T : Driver.Action.Snapshots.Thing_Id; Beats : Natural)
     return Driver.Uncertain.Point_Estimate is
   begin
      Must_Be_Inside (P, "a prediction");
      return P.World.Predicted (T, Beats);
   end Predicted;

   overriding procedure Learn (P : in out Rig; L : Plants.Lesson) is
      procedure Recording is
      begin
         P.World.Learn (L);
      end Recording;
   begin
      Must_Be_Outside (P, "a lesson");
      Driver.Beats.Within_A_Beat (Recording'Access);
   end Learn;

   overriding function Episode_Over (P : Rig) return Boolean is
     (P.Started and then Driver.Beats.Episode /= P.Episode);

   overriding function In_View (P : Rig; Point : Vec3) return Boolean is
   begin
      Must_Be_Inside (P, "a view");
      return P.World.In_View (Point);
   end In_View;

   function Observation_Of (M : Driver.Robot.Model; Readings : Driver.Observations.Reading_Vectors.Vector;
                            Beat : Natural) return Driver.Observations.Observation
   is
      O : Driver.Observations.Observation;
   begin
      O.Beat := Driver.Clock.Beat (Beat);
      O.Readings := Readings;
      for G in 1 .. Natural (Readings.Length) loop
         O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      end loop;
      for E in 1 .. Driver.Robot.Eye_Count (M) loop
         O.Images.Append (Driver.Images.No_Image);
         O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      end loop;
      return O;
   end Observation_Of;

   procedure Robot_Beat
     (M        : Driver.Robot.Model;
      W        : in out Sim.World;
      Arm      : Driver.Robot.Arm_Id;
      Readings : in out Driver.Observations.Reading_Vectors.Vector;
      Command  : Driver.Commands.Command)
   is
      G_Arm : constant Driver.Robot.Group_Id := Driver.Robot.Arm_Group (M, Arm);

      function Tool_At (Q : Real_Array) return Rigid is
         R : Driver.Observations.Reading_Vectors.Vector := Readings;
      begin
         R.Replace_Element (G_Arm, Q);
         return Driver.Robot.Tool_Pose (M, Arm, Observation_Of (M, R, 0)).Pose;
      end Tool_At;
   begin
      for G in 1 .. Driver.Robot.Group_Count (M) loop
         declare
            Id : constant Driver.Robot.Group_Id := Driver.Robot.Group_Id (G);
         begin
            if Driver.Commands.Has_Target (Command, Id) then
               if Id = G_Arm then
                  declare
                     Q0     : constant Real_Array := Readings (Id);
                     Q1     : constant Real_Array := Driver.Commands.Target (Command, Id);
                     From   : constant Rigid := Tool_At (Q0);
                     To     : constant Rigid := Tool_At (Q1);
                     Lever  : constant Real := W.Hands.First_Element.Depth;
                     Span   : constant Real := abs (To.Translation - From.Translation)
                       + Lever * Angle (Transpose (From.Rotation) * To.Rotation);
                     Pieces : constant Positive := Positive'Max (1, Natural (Real'Ceiling (Span / (W.Pitch / 2.0))));
                     Good   : Real_Array := Q0;
                     Stop   : Boolean := False;
                     Rec    : Beat_Record := (From => From, To => To, At_Stop => From, others => <>);
                  begin
                     for K in 1 .. Pieces loop
                        declare
                           Q : Real_Array := Q0;
                        begin
                           for C in Q'Range loop
                              Q (C) := Q0 (C) + (Real (K) / Real (Pieces)) * (Q1 (C - Q'First + Q1'First) - Q0 (C));
                           end loop;
                           declare
                              Share : constant Real := Real (K) / Real (Pieces);
                              Joint : constant Rigid := Tool_At (Q);
                              Line  : constant Rigid :=
                                (Rotation    => From.Rotation * Exp (Share * Log (Transpose (From.Rotation) * To.Rotation)),
                                 Translation => (1.0 - Share) * From.Translation + Share * To.Translation);
                              Tool  : constant Rigid := (if Cartesian then Line else Joint);
                           begin
                              Rec.Bow := Real'Max (Rec.Bow, abs (Joint.Translation - Line.Translation));
                              Sim.Put_Tool (W, Arm, Tool, Stop);
                              Rec.Stopped := Stop;
                           end;
                           exit when Stop;
                           Good := Q;
                        end;
                     end loop;
                     Readings.Replace_Element (Id, Good);
                     if Trace then
                        Rec.At_Stop := Tool_At (Good);
                        Beat_Log.Append (Rec);
                     end if;
                  end;
               else
                  Readings.Replace_Element (Id, Driver.Commands.Target (Command, Id));
               end if;
            end if;
         end;
      end loop;
      Sim.Tick (W);
   end Robot_Beat;

end Action_Rig;
