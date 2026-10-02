with Ada.Numerics.Long_Elementary_Functions;
with Driver.Beats;
with Driver.Commands;
with Driver.Images;
with Driver.Robot.Motion;

package body Driver.Action.Plants.Live is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Uncertain;
   use type Driver.Robot.Mount_Kind;
   use type Driver.Robot.Motion.Plan_Status;
   use type Driver.Robot.Group_Id;
   use type Driver.Robot.Arm_Id;
   use type Driver.World.Thing_Id;

   package Robot_Hand renames Driver.Robot.Hand;
   package World renames Driver.World;

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
      if not Driver.Observations.Has_Reading (O, G) then
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

   function Snapshot_Of
     (Robot : Driver.Robot.Model;
      Hands : Driver.Robot.Hand.Hands;
      Scene : Driver.World.Scene;
      O     : Driver.Observations.Observation;
      Learned : Driver.Action.Snapshots.Thing_Vectors.Vector := Driver.Action.Snapshots.Thing_Vectors.Empty_Vector)
     return Driver.Action.Snapshots.Snapshot
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
         declare
            Id : constant Arm_Id := Arm_Id (A);
            X  : Arm_State;
         begin
            X.Id := Id;
            X.Tool := Driver.Robot.Tool_Pose (Robot, Id, O);
            X.Carries_Eye := (for some E of S.Eyes => E.On_Arm = Id);
            X.Carries_All := Driver.Robot.Carrier_Group (Robot) /= 0
              and then Driver.Robot.Arm_Group (Robot, Id) = Driver.Robot.Carrier_Group (Robot);
            S.Arms.Append (X);
         end;
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
                                                                      Largest_Sigma (Closed.Covariance)),
                                              Width      => 0.0,
                                              Thickness  => 0.0));
               end;
            end loop;
            X.Fraction := Fraction_Of (Robot, Hands, Id, O);
            S.Hands.Append (X);
         end;
      end loop;
      for T in 1 .. World.Thing_Count (Scene) loop
         declare
            Id   : constant Thing_Id := Thing_Id (T);
            X    : Thing_State;
            Most : Natural := 0;
         begin
            X.Id := Id;
            X.Centre := World.Centre (Scene, Id);
            X.Sigma := (if Known (X.Centre) then Largest_Sigma (X.Centre.Covariance) else Real'Last);
            X.Support := World.Resting_On (Scene, Id);
            X.Height := World.Height_Above_Support (Scene, Id);
            X.Held_By := World.Held_By (Scene, Id);
            X.Moving := World.Moving (Scene, Id);
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
            for L of Learned loop
               if L.Id = Id then
                  X.Friction := L.Friction;
               end if;
            end loop;
            S.Things.Append (X);
         end;
      end loop;
      return S;
   end Snapshot_Of;

   overriding procedure Look (P : in out Live; S : out Driver.Action.Snapshots.Snapshot) is
      procedure During is
      begin
         P.Last := Driver.Beats.Latest.all;
      end During;
   begin
      if not P.Started then
         P.Episode := Driver.Beats.Episode;
         P.Started := True;
      end if;
      Driver.Beats.Within_A_Beat (During'Access);
      P.Looked := True;
      S := Snapshot_Of (P.Robot.all, P.Hands.all, P.Scene.all, P.Last, P.Learned);
   end Look;

   overriding function Reach (P : Live; Goal : Arm_Goal) return Reach_Answer is
      Plan : constant Driver.Robot.Motion.Plan :=
        Driver.Robot.Motion.Plan_Reach (P.Robot.all, Goal.Arm, P.Last,
                                        (Pose => Goal.Tool, Position_Only => Goal.Position_Only));
   begin
      case Driver.Robot.Motion.Status (Plan) is
         when Driver.Robot.Motion.Planned =>
            return (Status => Reachable, Why => Null_Unbounded_String);
         when Driver.Robot.Motion.Unreachable =>
            return (Status => Unreachable, Why => To_Unbounded_String (Driver.Robot.Motion.Why (Plan)));
         when Driver.Robot.Motion.Unmeasured =>
            return (Status => Unmeasured, Why => To_Unbounded_String (Driver.Robot.Motion.Why (Plan)));
      end case;
   end Reach;

   function Outcome_Of (O : Driver.Robot.Motion.Step_Outcome) return Step_Outcome is
     (case O is
         when Driver.Robot.Motion.Reached => Reached,
         when Driver.Robot.Motion.Blocked => Blocked,
         when Driver.Robot.Motion.Short   => Short);

   procedure Nothing is null;

   --  Every arm goal and every closer goal of the order, one after another:
   --  the motion layer moves one plan or one set of targets at a time.
   overriding procedure Move (P : in out Live; O : Order; R : out Report) is
   begin
      R := (others => <>);
      if O.Arms.Is_Empty and then O.Closers.Is_Empty then
         Driver.Beats.Within_A_Beat (Nothing'Access);
         R.Beats := 1;
         return;
      end if;
      for G of O.Arms loop
         declare
            Plan : constant Driver.Robot.Motion.Plan :=
              Driver.Robot.Motion.Plan_Reach (P.Robot.all, G.Arm, P.Last,
                                              (Pose => G.Tool, Position_Only => G.Position_Only));
         begin
            if Driver.Robot.Motion.Status (Plan) = Driver.Robot.Motion.Planned then
               declare
                  Step : Driver.Robot.Motion.Step_Report;
               begin
                  Driver.Robot.Motion.Follow (P.Robot.all, Plan, Step);
                  R.Arms.Append (Arm_Result'(Arm => G.Arm, Outcome => Outcome_Of (Step.Outcome),
                                             Delivered => Step.Delivered, Why => Step.Detail));
                  R.Beats := R.Beats + Step.Beats;
               end;
            else
               R.Arms.Append (Arm_Result'(Arm => G.Arm, Outcome => Refused, Delivered => Unknown,
                                          Why => To_Unbounded_String (Driver.Robot.Motion.Why (Plan))));
            end if;
         end;
      end loop;
      for C of O.Closers loop
         declare
            Open    : constant Real_Array := Robot_Hand.Closer_Reading (P.Hands.all, C.Hand, Robot_Hand.Open);
            Closed  : constant Real_Array := Robot_Hand.Closer_Reading (P.Hands.all, C.Hand, Robot_Hand.Closed_Empty);
            Targets : Driver.Commands.Command := Driver.Commands.Hold;
            Values  : Real_Array (Open'Range);
            Step    : Driver.Robot.Motion.Step_Report;
         begin
            for K in Open'Range loop
               Values (K) := Open (K) + C.Fraction * (Closed (K) - Open (K));
            end loop;
            Driver.Commands.Set_Target (Targets, Robot_Hand.Closer_Group (P.Hands.all, C.Hand), Values);
            Driver.Robot.Motion.Step (P.Robot.all, Targets, Step);
            R.Closers.Append (Closer_Result'(Hand => C.Hand, Outcome => Outcome_Of (Step.Outcome)));
            R.Beats := R.Beats + Step.Beats;
         end;
      end loop;
   end Move;

   overriding function Predicted (P : Live; T : Driver.Action.Snapshots.Thing_Id; Beats : Natural)
     return Point_Estimate
   is
      pragma Unreferenced (Beats);
   begin
      return World.Centre (P.Scene.all, T);
   end Predicted;

   overriding procedure Learn (P : in out Live; L : Lesson) is
   begin
      if L.Kind = Friction_Learned then
         for X of P.Learned loop
            if X.Id = L.Thing then
               X.Friction := L.Bounds;
               return;
            end if;
         end loop;
         declare
            X : Driver.Action.Snapshots.Thing_State;
         begin
            X.Id := L.Thing;
            X.Friction := L.Bounds;
            P.Learned.Append (X);
         end;
      end if;
   end Learn;

   overriding function Episode_Over (P : Live) return Boolean is
     (P.Started and then Driver.Beats.Episode /= P.Episode);

   overriding function In_View (P : Live; Point : Vec3) return Boolean is
   begin
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
