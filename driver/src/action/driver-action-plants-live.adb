with Ada.Numerics.Long_Elementary_Functions;
with Driver.Beats;
with Driver.Commands;
with Driver.Geometry;
with Driver.Images;
with Driver.Robot.Motion;
with Driver.Stats;

package body Driver.Action.Plants.Live is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Uncertain;
   use type Driver.Robot.Mount_Kind;
   use type Driver.Robot.Motion.Plan_Status;
   use type Driver.Robot.Group_Id;
   use type Driver.Robot.Arm_Id;

   package Robot_Hand renames Driver.Robot.Hand;
   package World renames Driver.World;
   package Motion renames Driver.Robot.Motion;

   function Largest_Sigma (C : Mat3) return Real is
     (Sqrt (Real'Max (C (1, 1), Real'Max (C (2, 2), C (3, 3)))));

   --  How far a closer is closed: its readings projected on the way from its
   --  open readings to its readings closed on nothing, with the variance of
   --  that projection from each channel's noise at rest.
   function Fraction_Of (M : Driver.Robot.Model; H : Robot_Hand.Hands; Id : Robot_Hand.Hand_Id;
                         O : Driver.Observations.Observation) return Estimate
   is
      G : constant Driver.Robot.Group_Id := Robot_Hand.Closer_Group (H, Id);
   begin
      if Natural (G) > Natural (O.Readings.Length) or else not Driver.Observations.Has_Reading (O, G) then
         return Unknown;
      end if;
      declare
         Now    : constant Real_Array := O.Readings (G);
         Open   : constant Real_Array := Robot_Hand.Closer_Reading (H, Id, Robot_Hand.Open);
         Closed : constant Real_Array := Robot_Hand.Closer_Reading (H, Id, Robot_Hand.Closed_Empty);
         Way    : Real := 0.0;
         Along  : Real := 0.0;
         Var    : Real := 0.0;
      begin
         if Now'Length /= Open'Length or else Open'Length /= Closed'Length then
            return Unknown;
         end if;
         for K in Open'Range loop
            Way := Way + (Closed (K) - Open (K)) ** 2;
         end loop;
         if not (Way > 0.0) then
            return Unknown;
         end if;
         for K in Open'Range loop
            declare
               Channel : constant Positive := K - Open'First + 1;
               D       : constant Real := Closed (K) - Open (K);
            begin
               Along := Along + (Now (K - Open'First + Now'First) - Open (K)) * D;
               Var := Var + (D / Way) ** 2 * Driver.Robot.Reading_Noise (M, G, Channel) ** 2;
            end;
         end loop;
         return (Value => Along / Way, Sigma => Sqrt (Var), Degrees_Of_Freedom => 0);
      end;
   end Fraction_Of;

   --  How finely the arm places its tool: of every joint's smallest visible
   --  step, the largest shift and turn of the tool it makes, as the arm's
   --  kinematics give the tool at readings moved by that step. Unknown while
   --  a joint's step or the tool's pose is not measured.
   procedure Resolution_Of (M : Driver.Robot.Model; A : Arm_Id; O : Driver.Observations.Observation;
                            Shift, Turn : out Estimate)
   is
      G     : constant Driver.Robot.Group_Id := Driver.Robot.Arm_Group (M, A);
      Here  : constant Pose_Estimate := Driver.Robot.Tool_Pose (M, A, O);
   begin
      Shift := Unknown;
      Turn := Unknown;
      if not Known (Position (Here)) or else Natural (G) > Natural (O.Readings.Length)
        or else not Driver.Observations.Has_Reading (O, G)
      then
         return;
      end if;
      declare
         Readings : constant Real_Array := O.Readings (G);
         Most_Shift, Most_Turn : Real := 0.0;
      begin
         for C in 1 .. Driver.Robot.Group_Size (M, G) loop
            declare
               Least : constant Estimate := Driver.Robot.Visible_Step (M, G, C);
               Moved : Driver.Observations.Observation;
               R     : Real_Array := Readings;
            begin
               if not Known (Least) then
                  return;
               end if;
               R (R'First + C - 1) := R (R'First + C - 1) + Least.Value;
               Moved.Readings := O.Readings;
               Moved.Readings.Replace_Element (G, R);
               declare
                  There : constant Pose_Estimate := Driver.Robot.Tool_Pose (M, A, Moved);
               begin
                  if not Known (Position (There)) then
                     return;
                  end if;
                  Most_Shift := Real'Max (Most_Shift, abs (There.Pose.Translation - Here.Pose.Translation));
                  Most_Turn := Real'Max (Most_Turn, Angle (Transpose (Here.Pose.Rotation) * There.Pose.Rotation));
               end;
            end;
         end loop;
         Shift := (Value => Most_Shift, Sigma => Largest_Sigma (Here.Position_Covariance), Degrees_Of_Freedom => 0);
         Turn := (Value => Most_Turn, Sigma => Largest_Sigma (Here.Rotation_Covariance), Degrees_Of_Freedom => 0);
      end;
   end Resolution_Of;

   --  The spacing of a thing's samples: the median of each sample's distance
   --  to its nearest neighbour.
   function Spacing_Of (S : Sample_Vectors.Vector) return Real is
      N : constant Natural := Natural (S.Length);
   begin
      if N < 2 then
         return 0.0;
      end if;
      declare
         Nearest : Real_Array (1 .. N) := [others => Real'Last];
      begin
         for I in 1 .. N loop
            for J in 1 .. N loop
               if I /= J then
                  Nearest (I) := Real'Min (Nearest (I), abs (S (I).Point - S (J).Point));
               end if;
            end loop;
         end loop;
         return Driver.Stats.Median (Nearest);
      end;
   end Spacing_Of;

   function Arm_Of
     (Robot : Driver.Robot.Model;
      A     : Driver.Action.Snapshots.Arm_Id;
      O     : Driver.Observations.Observation) return Driver.Action.Snapshots.Arm_State
   is
      X : Arm_State;
   begin
      X.Id := A;
      X.Tool := Driver.Robot.Tool_Pose (Robot, A, O);
      Resolution_Of (Robot, A, O, X.Step, X.Turn_Step);
      X.Carries_Eye := (for some E in 1 .. Driver.Robot.Eye_Count (Robot) =>
                          Driver.Robot.Eye_Mount (Robot, Driver.Robot.Eye_Id (E)).Kind = Driver.Robot.Arm_Carried
                          and then Driver.Robot.Eye_Mount (Robot, Driver.Robot.Eye_Id (E)).Arm = A);
      X.Carries_All := Driver.Robot.Carrier_Group (Robot) /= 0
        and then Driver.Robot.Arm_Group (Robot, A) = Driver.Robot.Carrier_Group (Robot);
      return X;
   end Arm_Of;

   function Snapshot_Of
     (Robot : Driver.Robot.Model;
      Hands : Driver.Robot.Hand.Hands;
      Scene : Driver.World.Scene;
      O     : Driver.Observations.Observation) return Driver.Action.Snapshots.Snapshot
   is
      S : Snapshot;
   begin
      S.Beat := O.Beat;
      S.Up := Driver.Robot.Up (Robot);
      S.Still := Driver.Robot.Still (Robot);
      for E in 1 .. Driver.Robot.Eye_Count (Robot) loop
         declare
            Id    : constant Driver.Robot.Eye_Id := Driver.Robot.Eye_Id (E);
            Mount : constant Driver.Robot.Mount := Driver.Robot.Eye_Mount (Robot, Id);
         begin
            S.Eyes.Append (Eye_State'(Pose   => Driver.Robot.Eye_Pose (Robot, Id, O),
                                      On_Arm => (if Mount.Kind = Driver.Robot.Arm_Carried then Mount.Arm else 0)));
         end;
      end loop;
      for A in 1 .. Driver.Robot.Arm_Count (Robot) loop
         S.Arms.Append (Arm_Of (Robot, Arm_Id (A), O));
      end loop;
      for H in 1 .. Robot_Hand.Hand_Count (Hands) loop
         declare
            Id : constant Hand_Id := Hand_Id (H);
            X  : Hand_State;
         begin
            X.Id := Id;
            X.Arm := Robot_Hand.Arm_Of (Hands, Id);
            for L in 1 .. Robot_Hand.Lobe_Count (Hands, Id) loop
               declare
                  Open   : constant Point_Estimate := Robot_Hand.Tip_In_Tool (Hands, Id, L, Robot_Hand.Open);
                  Closed : constant Point_Estimate := Robot_Hand.Tip_In_Tool (Hands, Id, L, Robot_Hand.Closed_Empty);
               begin
                  X.Lobes.Append (Lobe_State'(Open_Tip   => Open.Mean,
                                              Closed_Tip => Closed.Mean,
                                              Tip_Sigma  => Real'Max (Largest_Sigma (Open.Covariance),
                                                                      Largest_Sigma (Closed.Covariance))));
               end;
            end loop;
            X.Fraction := Fraction_Of (Robot, Hands, Id, O);
            S.Hands.Append (X);
         end;
      end loop;
      for F in 1 .. World.Surface_Count (Scene) loop
         declare
            P : constant Driver.Geometry.Plane_Estimate := World.Plane_Of (Scene, Surface_Id (F));
         begin
            if P.Offset_Sigma < Real'Last and then abs P.Normal > 0.0 then
               S.Surfaces.Append (Surface_State'(Id       => Surface_Id (F),
                                                 Point    => (Mean => P.Centre,
                                                              Covariance => (P.Offset_Sigma ** 2) * Identity3),
                                                 Normal   => (Unit_Vector => Unit (P.Normal),
                                                              Sigma => Sqrt (Real'Max (P.Tilt_11, P.Tilt_22))),
                                                 Of_Thing => 0));
            end if;
         end;
      end loop;
      for P in 1 .. World.Place_Count (Scene) loop
         S.Places.Append (Place_State'(Id => Place_Id (P), Point => World.Where (Scene, Place_Id (P))));
      end loop;
      for T in 1 .. World.Thing_Count (Scene) loop
         declare
            Id   : constant Thing_Id := Thing_Id (T);
            X    : Thing_State;
            Most : Natural := 0;
            Seen : constant World.Sample_Array := World.Samples (Scene, Id);
            Rub  : constant World.Friction_Bounds := World.Friction (Scene, Id);
         begin
            X.Id := Id;
            X.Centre := World.Centre (Scene, Id);
            for K in Seen'Range loop
               X.Samples.Append (Sample'(Point => Seen (K).Point, Normal => Seen (K).Normal));
            end loop;
            X.Sigma := World.Sample_Sigma (Scene, Id);
            X.Pitch := Spacing_Of (X.Samples);
            --  A normal fitted to neighbours one spacing apart, each Sigma off.
            X.Normal_Sigma := (if X.Pitch > 0.0 and then X.Sigma < Real'Last then X.Sigma / X.Pitch else Real'Last);
            X.Support := World.Resting_On (Scene, Id);
            X.Height := World.Height_Above_Support (Scene, Id);
            X.Held_By := World.Held_By (Scene, Id);
            X.Moving := World.Moving (Scene, Id);
            X.Friction := (Low => Rub.Low, High => Rub.High);
            for E in 1 .. Natural (S.Eyes.Length) loop
               if World.Seen_In (Scene, Id, Driver.Robot.Eye_Id (E)) then
                  X.Seen := True;
                  if S.Eyes (E).On_Arm = 0 then
                     declare
                        Pixels : constant Natural :=
                          Driver.Images.Count (World.Region_In (Scene, Id, Driver.Robot.Eye_Id (E)));
                     begin
                        if Pixels > Most then
                           Most := Pixels;
                           X.Best_Eye := E;
                        end if;
                     end;
                  end if;
               end if;
            end loop;
            S.Things.Append (X);
         end;
      end loop;
      return S;
   end Snapshot_Of;

   --  The models are read and written only in a beat's window: the main loop
   --  changes them every beat, whether the decider takes the beat or not. What
   --  needs the models is asked inside a window; what takes a beat of its own
   --  would wait for the one the window holds, for ever, so it is refused.
   procedure Must_Be_Inside (P : Live; What : String) is
   begin
      if not P.Inside then
         raise Program_Error with What & " asked outside a beat's window, where the models change";
      end if;
   end Must_Be_Inside;

   procedure Must_Be_Outside (P : Live; What : String) is
   begin
      if P.Inside then
         raise Program_Error with What & " asked inside a beat's window, which holds the beat it would wait for";
      end if;
   end Must_Be_Outside;

   overriding procedure Look (P : in out Live; S : out Driver.Action.Snapshots.Snapshot) is
      procedure During is
      begin
         P.Last := Driver.Beats.Latest.all;
         S := Snapshot_Of (P.Robot.all, P.Hands.all, P.Scene.all, P.Last);
      end During;
   begin
      Must_Be_Outside (P, "a look");
      if not P.Started then
         P.Episode := Driver.Beats.Episode;
         P.Started := True;
      end if;
      Driver.Beats.Within_A_Beat (During'Access);
   end Look;

   overriding procedure Within (P : in out Live; During : not null access procedure) is
      procedure Held is
      begin
         P.Last := Driver.Beats.Latest.all;
         P.Inside := True;
         During.all;
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

   overriding function Reach (P : Live; Goal : Arm_Goal) return Reach_Answer is
   begin
      Must_Be_Inside (P, "a reach");
      declare
         Plan : constant Motion.Plan :=
           Motion.Plan_Reach (P.Robot.all, Goal.Arm, P.Last, (Pose => Goal.Tool, Position_Only => Goal.Position_Only),
                              Clearance => Goal.Clearance, Lever => Goal.Lever);
      begin
         case Motion.Status (Plan) is
            when Motion.Planned     =>
               return (Status => Reachable, Why => Null_Unbounded_String, Bow => Motion.Worst_Bow (Plan));
            when Motion.Unreachable =>
               return (Status => Unreachable, Why => To_Unbounded_String (Motion.Why (Plan)), Bow => 0.0);
            when Motion.Unmeasured  =>
               return (Status => Unmeasured, Why => To_Unbounded_String (Motion.Why (Plan)), Bow => 0.0);
         end case;
      end;
   end Reach;

   --  A blocked push is blocked whether it moved first or not; an unblocked
   --  step that delivered significantly less than the whole is short.
   function Outcome_Of (R : Motion.Step_Report) return Step_Outcome is
     (case R.Outcome is
         when Motion.Blocked | Motion.Short => Blocked,
         when Motion.Reached =>
           (if Known (R.Delivered) and then R.Delivered.Value < 1.0
              and then Significant (1.0 - R.Delivered.Value, R.Delivered.Sigma, R.Delivered.Degrees_Of_Freedom)
            then Short else Reached));

   procedure Nothing is null;

   --  Every arm goal and every closer goal of the order, one after another:
   --  the motion layer moves one plan or one set of targets at a time.
   overriding procedure Move (P : in out Live; O : Order; R : out Report) is
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
            --  Planned from the readings of the beat it is planned in.
            procedure Planning is
            begin
               P.Last := Driver.Beats.Latest.all;
               Plan := Motion.Plan_Reach (P.Robot.all, G.Arm, P.Last,
                                          (Pose => G.Tool, Position_Only => G.Position_Only),
                                          Clearance => G.Clearance, Lever => G.Lever);
            end Planning;
         begin
            Driver.Beats.Within_A_Beat (Planning'Access);
            if Motion.Status (Plan) = Motion.Planned then
               declare
                  Step : Motion.Step_Report;
               begin
                  Motion.Follow (P.Robot.all, Plan, Step);
                  R.Arms.Append (Arm_Result'(Arm => G.Arm, Outcome => Outcome_Of (Step), Delivered => Step.Delivered,
                                             Why => Step.Detail));
                  R.Beats := R.Beats + Step.Beats;
               end;
            else
               R.Arms.Append (Arm_Result'(Arm => G.Arm, Outcome => Refused, Delivered => Unknown,
                                          Why => To_Unbounded_String (Motion.Why (Plan))));
            end if;
         end;
      end loop;
      for C of O.Closers loop
         declare
            Targets : Driver.Commands.Command := Driver.Commands.Hold;
            Step    : Motion.Step_Report;
            --  The closer's readings open and closed, read in a beat's window.
            procedure Aiming is
               Open   : constant Real_Array := Robot_Hand.Closer_Reading (P.Hands.all, C.Hand, Robot_Hand.Open);
               Closed : constant Real_Array := Robot_Hand.Closer_Reading (P.Hands.all, C.Hand, Robot_Hand.Closed_Empty);
               Values : Real_Array (Open'Range);
            begin
               for K in Open'Range loop
                  Values (K) := Open (K) + C.Fraction * (Closed (K) - Open (K));
               end loop;
               Driver.Commands.Set_Target (Targets, Robot_Hand.Closer_Group (P.Hands.all, C.Hand), Values);
            end Aiming;
         begin
            Driver.Beats.Within_A_Beat (Aiming'Access);
            Motion.Step (P.Robot.all, Targets, Step);
            R.Closers.Append (Closer_Result'(Hand => C.Hand, Outcome => Outcome_Of (Step)));
            R.Beats := R.Beats + Step.Beats;
         end;
      end loop;
   end Move;

   overriding function Predicted (P : Live; T : Driver.Action.Snapshots.Thing_Id; Beats : Natural)
     return Point_Estimate is
   begin
      Must_Be_Inside (P, "a prediction");
      return World.Predicted (P.Scene.all, T, Beats);
   end Predicted;

   overriding procedure Learn (P : in out Live; L : Lesson) is
      procedure Recording is
      begin
         case L.Kind is
            when Friction_Learned =>
               World.Learn_Friction (P.Scene.all, L.Thing, (Low => L.Bounds.Low, High => L.Bounds.High));
            when Touched_At =>
               World.Touched (P.Scene.all, L.Thing, L.Point);
         end case;
      end Recording;
   begin
      Must_Be_Outside (P, "a lesson");
      --  The world is written in a beat's window too: the main loop writes it
      --  every beat, and the write goes into the recording (kind W) at the
      --  point of the stream where the beat is, where a replay applies it.
      Driver.Beats.Within_A_Beat (Recording'Access);
   end Learn;

   overriding function Episode_Over (P : Live) return Boolean is
     (P.Started and then Driver.Beats.Episode /= P.Episode);

   overriding function In_View (P : Live; Point : Vec3) return Boolean is
   begin
      Must_Be_Inside (P, "a view");
      for E in 1 .. Driver.Robot.Eye_Count (P.Robot.all) loop
         declare
            Id      : constant Driver.Robot.Eye_Id := Driver.Robot.Eye_Id (E);
            Px      : Driver.Images.Pixel;
            Visible : Boolean;
         begin
            if Driver.Robot.Eye_Mount (P.Robot.all, Id).Kind /= Driver.Robot.Arm_Carried then
               Driver.Robot.Project (P.Robot.all, Id, P.Last, Point, Px, Visible);
               if Visible then
                  return True;
               end if;
            end if;
         end;
      end loop;
      return False;
   end In_View;

end Driver.Action.Plants.Live;
