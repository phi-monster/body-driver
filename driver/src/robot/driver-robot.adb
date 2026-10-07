with Ada.Containers;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Directories;
with Ada.Text_IO;
with Driver.Bytes;
with Driver.Clock;
with Driver.Conventions;
with Driver.Log;
with Driver.Recording;
with Driver.Robot.Body_File;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Graph;
with Driver.Robot.Kinematics;
with Driver.Robot.Lag;
with Driver.Robot.Lockin;
with Driver.Robot.Steps;
with Driver.Robot.Stillness;

package body Driver.Robot is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Ada.Containers.Count_Type;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Luma_Access);
   type Flag_Access is access Flow.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flow.Flag_Array, Flag_Access);

   --  The displacement of every cell of the eye from its previous frame to
   --  its current one, appended to its stream.
   procedure Measure_Displacement (S : in out Eye_Stream)
     with Pre => S.Has_Previous and then Cells (S.Grid) > 0
   is
      --  Sized by cells: on the heap.
      N          : constant Positive := Cells (S.Grid);
      Du         : Luma_Access := new Real_Array (1 .. N);
      Dv         : Luma_Access := new Real_Array (1 .. N);
      Condition  : Luma_Access := new Real_Array (1 .. N);
      Cell_Noise : Luma_Access := new Real_Array (1 .. N);
      Resolved   : Flag_Access := new Flow.Flag_Array (1 .. N);
   begin
      --  A cell at rest moves by its pixels' noise times the eye's rest factor.
      for C in 1 .. N loop
         Cell_Noise (C) := S.Luma_Variance.Element (C - 1) * S.Rest_Factor ** 2;
      end loop;
      Flow.Displacements (S.Grid, S.Previous.all, S.Current.all, Cell_Noise.all, Du.all, Dv.all, Condition.all,
                          Resolved.all);
      for C in 1 .. N loop
         S.Du.Append (Du (C));
         S.Dv.Append (Dv (C));
         S.Condition.Append (Condition (C));
         S.Resolved.Append (Resolved (C));
      end loop;
      S.Measured.Append (True);
      Free (Du);
      Free (Dv);
      Free (Condition);
      Free (Cell_Noise);
      Free (Resolved);
   end Measure_Displacement;

   procedure Observe_Eyes (M : in out Model; O : Observation) is
      --  Some commandable group began to move at this beat: each picture's
      --  settle watch starts afresh when its frames show that beat, its lag
      --  later, so the first change it weighs is the move's own.
      Began_Moving : Boolean := False;
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if M.Groups (G).Commandable and then Channels.Moving (M, G, M.Beats)
           and then not (M.Beats > 0 and then Channels.Moving (M, G, M.Beats - 1))
         then
            Began_Moving := True;
         end if;
      end loop;
      M.Began_Moving.Append (Began_Moving);
      if M.Eyes.Is_Empty then
         for E in O.Images.First_Index .. O.Images.Last_Index loop
            M.Eyes.Append (Eye_Stream'(others => <>));
         end loop;
      end if;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S    : Eye_Stream renames M.Eyes (E);
            Have : constant Boolean := E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E);
         begin
            if Have then
               declare
                  Size : constant Natural := Driver.Images.Width (O.Images (E)) * Driver.Images.Height (O.Images (E));
               begin
                  if S.Current = null or else S.Current'Length /= Size then
                     Free (S.Current);
                     S.Current := new Real_Array (1 .. Size);
                  end if;
                  Driver.Images.Luma (O.Images (E), S.Current.all);
               end;
               if S.Grid.Width = 0 then
                  S.Grid := Flow.Grid_Of (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)));
                  S.Du.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Dv.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Condition.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Resolved.Append (False, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Measured.Append (False, Ada.Containers.Count_Type (M.Beats));
               end if;
            end if;
            --  An eye that appears late was not judged before.
            if S.Judged.Is_Empty and then M.Beats > 0 then
               S.Judged.Append (False, Ada.Containers.Count_Type (M.Beats));
               S.Still_At.Append (False, Ada.Containers.Count_Type (M.Beats));
            end if;
            declare
               N    : constant Natural := Cells (S.Grid);
               Same : constant Boolean :=
                 Have and then Driver.Images.Width (O.Images (E)) = S.Grid.Width
                 and then Driver.Images.Height (O.Images (E)) = S.Grid.Height;
            begin
               if Same and then S.Has_Previous then
                  Measure_Displacement (S);
               elsif N > 0 then
                  S.Du.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Dv.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Condition.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Resolved.Append (False, Ada.Containers.Count_Type (N));
                  S.Measured.Append (False);
               end if;
               if Have then
                  Stillness.Judge_Eye (S, O.Images (E), S.Current.all);
                  if S.Luma_Variance.Is_Empty then
                     Stillness.Measure_Luma_Noise (S);
                  end if;
               end if;
               --  An eye's first two frames start its noise and are not judged.
               S.Judged.Append (Have and then S.Has_Judged);
               S.Still_At.Append (Have and then S.Is_Still);
               --  Whether its picture has stopped changing (the one stop rule).
               declare
                  Shown : constant Integer := M.Beats - Natural'Max (0, Image_Lag (M, E));
                  Reset : constant Boolean := Shown >= 0 and then M.Began_Moving (Shown);
               begin
                  if not Same then
                     S.Has_Previous := False;
                     S.Has_Before := False;
                  end if;
                  Stillness.Watch (S, Reset);
               end;
               --  This frame is the next one's previous, and the previous one
               --  the next one's before; a missing frame, or one of another
               --  size, breaks the chain: the next displacement would span two
               --  beats.
               if Same then
                  declare
                     Spare : constant Luma_Access := S.Before;
                  begin
                     S.Before := S.Previous;
                     S.Previous := S.Current;
                     S.Current := Spare;
                  end;
               end if;
               S.Has_Before := Same and then S.Has_Previous;
               S.Has_Previous := Same;
            end;
         end;
      end loop;
   end Observe_Eyes;

   --  What came from a body file stands: it is not measured again in this
   --  session (Load_Body). The channels still mark the pushes, with the
   --  noise as reloaded; the pictures' luma noise belongs to the scene and
   --  is always measured.
   procedure Recompute (M : in out Model) is
      Start : constant Duration := Driver.Clock.Seconds;
   begin
      Channels.Measure (M);
      for S of M.Eyes loop
         if S.Has_Settled then
            Stillness.Measure_Luma_Noise (S);
         end if;
      end loop;
      if not M.From_File (Stored_Lags) then
         Lag.Measure (M);
      end if;
      if not M.From_File (Stored_Responses) then
         Lockin.Measure_Rest_Noise (M);
         Lockin.Measure (M);
      end if;
      if not M.From_File (Stored_Graph) then
         Graph.Derive (M);
      end if;
      if not M.From_File (Stored_Kinematics) then
         Kinematics.Refit (M);
      end if;
      M.Graph_Evidence := M.Beats;
      Driver.Log.Line (Driver.Log.Robot, "estimated from" & M.Beats'Image & " beats in"
                       & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s");
   end Recompute;

   --  A decider's call goes into the recording (kind E) inside its window,
   --  after the beat's observation and before its reply, so a replay
   --  recomputes at the same point; Observe's own recomputations are not
   --  recorded, since a replay makes them itself.
   procedure Estimate_Now (M : in out Model) is
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Estimates_Asked, Driver.Bytes.To_Bytes (""));
      Recompute (M);
   end Estimate_Now;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command) is
   begin
      Channels.Append (M, O, Sent);
      Steps.Track (M, M.Beats);
      Observe_Eyes (M, O);
      if not M.From_File (Stored_Kinematics) then
         Kinematics.Observe (M, O);
      end if;
      M.Beats := M.Beats + 1;
      --  The estimates are redone whenever the evidence behind them has
      --  doubled: a logarithmic number of times over any stream.
      if M.Beats >= 2 * M.Graph_Evidence then
         Recompute (M);
      end if;
   end Observe;

   --  The text read goes into the recording (kind F) before the body takes
   --  it, so a replay reloads the same text at the same point of the run.
   procedure Load_Body
     (M    : in out Model;
      Path : String;
      Ok   : out Boolean;
      Why  : out Ada.Strings.Unbounded.Unbounded_String)
   is
      use Ada.Strings.Unbounded;
   begin
      Ok := False;
      if not Ada.Directories.Exists (Path) then
         Why := To_Unbounded_String ("there is no body file " & Path);
         return;
      end if;
      declare
         F    : Ada.Text_IO.File_Type;
         Text : Unbounded_String;
      begin
         Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
         while not Ada.Text_IO.End_Of_File (F) loop
            Append (Text, Ada.Text_IO.Get_Line (F) & ASCII.LF);
         end loop;
         Ada.Text_IO.Close (F);
         Driver.Recording.Write_Shared
           (Driver.Recording.File_Read, Driver.Bytes.To_Bytes ("body " & Path & ASCII.LF & To_String (Text)));
         Load_Body_Text (M, To_String (Text), Ok, Why);
      exception
         when Ada.Text_IO.Name_Error | Ada.Text_IO.Use_Error | Ada.Text_IO.Data_Error =>
            Why := To_Unbounded_String ("the body file " & Path & " cannot be read");
      end;
   end Load_Body;

   procedure Load_Body_Text
     (M    : in out Model;
      Text : String;
      Ok   : out Boolean;
      Why  : out Ada.Strings.Unbounded.Unbounded_String) is
   begin
      Body_File.Read (M, Text, Ok, Why);
   end Load_Body_Text;

   function Reloaded (M : Model; Q : Stored) return Boolean is (M.From_File (Q));

   --  Booted once the kinematics of every arm that carries an eye are
   --  fitted, and of one at least: a function of the evidence, so a replay
   --  finds the boot where the run did.
   function Booted (M : Model) return Boolean is
      Any : Boolean := False;
   begin
      for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
         if M.Graph.Mounts (E).Kind = Arm_Carried then
            if not Kinematics.Fitted (M, M.Graph.Mounts (E).Arm) then
               return False;
            end if;
            Any := True;
         end if;
      end loop;
      return Any;
   end Booted;

   function Role (M : Model; G : Group_Id) return Group_Role is
     (if G <= M.Graph.Roles.Last_Index then M.Graph.Roles (G) else Unclassified);

   function Arm_Count (M : Model) return Natural is (Natural (M.Graph.Arms.Length));

   function Arm_Group (M : Model; A : Arm_Id) return Group_Id is (M.Graph.Arms (A));

   function Eye_Count (M : Model) return Natural is (Natural (M.Eyes.Length));

   function Eye_Mount (M : Model; E : Eye_Id) return Mount is
     (if E <= M.Graph.Mounts.Last_Index then M.Graph.Mounts (E) else (Kind => Unmeasured));

   --  The arm whose eye this is, when its kinematics are fitted; 0 otherwise.
   function Fitted_Arm (M : Model; E : Eye_Id) return Arm_Id'Base is
     (if E <= M.Graph.Mounts.Last_Index and then M.Graph.Mounts (E).Kind = Arm_Carried
        and then Kinematics.Fitted (M, M.Graph.Mounts (E).Arm)
      then M.Graph.Mounts (E).Arm else 0);

   --  The world: the frame of the first arm's eye at its reference keyframe,
   --  until the arms are measured into one frame.
   --  The arm's eye at O in the world, known when the arm is placed in it
   --  (Kinematics.In_World: the first arm's reference frame is the world)
   --  and O carries its readings.
   procedure Arm_Eye (M : Model; A : Arm_Id; O : Observation; T : out Rigid; Known_Pose : out Boolean) is
      G         : constant Group_Id := Arm_Group (M, A);
      Placement : Rigid;
      Scale     : Real;
   begin
      T := Identity;
      Kinematics.In_World (M, A, Placement, Scale, Known_Pose);
      Known_Pose := Known_Pose and then G <= O.Readings.Last_Index
                    and then O.Readings.Element (G)'Length = Group_Size (M, G);
      if Known_Pose then
         declare
            use Driver.Numerics.Arrays;
            E : constant Rigid := Kinematics.Eye_In_Reference (M, A, O.Readings.Element (G));
         begin
            T := (Rotation    => Placement.Rotation * E.Rotation,
                  Translation => Placement.Rotation * (Scale * E.Translation) + Placement.Translation);
         end;
      end if;
   end Arm_Eye;

   --  A pose with the uncertainty of the fit that gave it: the covariance of
   --  the fit's parameters, from the errors its residuals show, carried to the eye at the
   --  readings it was made at, and into the world with its arm's placement
   --  (Kinematics.World_Pose_Covariance).
   function With_Fit_Uncertainty (M : Model; A : Arm_Id; T : Rigid; Readings : Real_Array) return Pose_Estimate is
      Turn, Place : Mat3;
   begin
      Kinematics.World_Pose_Covariance (M, A, Readings, Turn, Place);
      return (Pose => T, Position_Covariance => Place, Rotation_Covariance => Turn);
   end With_Fit_Uncertainty;

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is
      A : constant Arm_Id'Base := Fitted_Arm (M, E);
      T : Rigid;
      K : Boolean;
   begin
      if A = 0 then
         return (others => <>);
      end if;
      Arm_Eye (M, A, O, T, K);
      return (if K then With_Fit_Uncertainty (M, A, T, O.Readings.Element (Arm_Group (M, A))) else (others => <>));
   end Eye_Pose;

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean)
   is
      A : constant Arm_Id'Base := Fitted_Arm (M, E);
      T : Rigid;
      K : Boolean;
   begin
      Px := (U => 0.0, V => 0.0);
      Visible := False;
      if A > 0 then
         Arm_Eye (M, A, O, T, K);
         if K then
            Kinematics.Project_In_Eye (M, A, Inverse (T) * Point, Px.U, Px.V, Visible);
            Visible := Visible and then Px.U in 0.0 .. Real (M.Eyes (E).Grid.Width)
                       and then Px.V in 0.0 .. Real (M.Eyes (E).Grid.Height);
         end if;
      end if;
   end Project;

   function Eye_Ray (M : Model; E : Eye_Id; Px : Driver.Images.Pixel) return Ray_Estimate is
      A : constant Arm_Id'Base := Fitted_Arm (M, E);
   begin
      if A = 0 then
         return (others => <>);
      end if;
      return (Origin    => (Mean => [0.0, 0.0, 0.0], Covariance => [others => [others => 0.0]]),
              Direction => (Unit_Vector => Kinematics.Ray_In_Eye (M, A, Px.U, Px.V),
                            Sigma       => Kinematics.Angle_Sigma (M, A)));
   end Eye_Ray;

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate is
      A : constant Arm_Id'Base := Fitted_Arm (M, E);
      T : Rigid;
      K : Boolean;
   begin
      if A = 0 then
         return (others => <>);
      end if;
      Arm_Eye (M, A, O, T, K);
      if not K then
         return (others => <>);
      end if;
      declare
         P : constant Pose_Estimate := With_Fit_Uncertainty (M, A, T, O.Readings.Element (Arm_Group (M, A)));
         D : constant Vec3 := Driver.Numerics.Arrays."*" (T.Rotation, Kinematics.Ray_In_Eye (M, A, Px.U, Px.V));
         S : constant Mat3 := P.Rotation_Covariance;
         --  The pose's turn moves the line by its part across the line: per
         --  axis across it, half of what the turn's variance leaves off it.
         Across : constant Real :=
           (if S (1, 1) < Real'Last and then S (2, 2) < Real'Last and then S (3, 3) < Real'Last
            then Real'Max (0.0, (S (1, 1) + S (2, 2) + S (3, 3) - Driver.Numerics.Arrays."*" (D, Driver.Numerics.Arrays."*" (S, D))) / 2.0)
            else Real'Last);
      begin
         return (Origin    => (Mean => T.Translation, Covariance => P.Position_Covariance),
                 Direction => (Unit_Vector => D,
                               Sigma       => (if Across < Real'Last
                                               then Sqrt (Kinematics.Angle_Sigma (M, A) ** 2 + Across)
                                               else Real'Last)));
      end;
   end Ray;

   --  The world is the first arm's frame.
   function Up (M : Model) return Direction_Estimate is (Up_In_Arm (M, 1));

   --  The tool frame of an arm is the frame of the eye it carries: what the
   --  driver measures of a hand it measures through that eye.
   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate is
   begin
      for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
         if M.Graph.Mounts (E).Kind = Arm_Carried and then M.Graph.Mounts (E).Arm = A then
            return Eye_Pose (M, E, O);
         end if;
      end loop;
      return (others => <>);
   end Tool_Pose;

   function Tool_In_Arm (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate is
   begin
      if Natural (A) > Arm_Count (M) or else not Kinematics.Fitted (M, A) then
         return (others => <>);
      end if;
      declare
         G : constant Group_Id := Arm_Group (M, A);
      begin
         if G > O.Readings.Last_Index or else O.Readings.Element (G)'Length /= Group_Size (M, G) then
            return (others => <>);
         end if;
         declare
            Readings : constant Real_Array := O.Readings.Element (G);
            Turn, Place : Mat3;
         begin
            Kinematics.Pose_Covariance (M, A, Readings, Turn, Place);
            return (Pose                => Kinematics.Eye_In_Reference (M, A, Readings),
                    Position_Covariance => Place,
                    Rotation_Covariance => Turn);
         end;
      end;
   end Tool_In_Arm;

   function Table_In_Arm (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate is
     (if Natural (A) <= Arm_Count (M) then Kinematics.Table (M, A) else (others => <>));

   function Up_In_Arm (M : Model; A : Arm_Id) return Direction_Estimate is
      P : constant Driver.Geometry.Plane_Estimate := Table_In_Arm (M, A);
   begin
      if not Driver.Geometry.Known (P) then
         return (others => <>);
      end if;
      declare
         --  The larger eigenvalue of the tilt's covariance.
         Mean   : constant Real := (P.Tilt_11 + P.Tilt_22) / 2.0;
         Spread : constant Real := Sqrt (((P.Tilt_11 - P.Tilt_22) / 2.0) ** 2 + P.Tilt_12 ** 2);
      begin
         return (Unit_Vector => P.Normal, Sigma => Sqrt (Mean + Spread));
      end;
   end Up_In_Arm;

   function Arm_Unit (M : Model; A : Arm_Id) return Estimate is
     (if Natural (A) <= Arm_Count (M) then Kinematics.Scale_In_World (M, A) else Unknown);

   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is
      pragma Unreferenced (O);
   begin
      --  The eye is the tool frame of the arm that carries it: exactly.
      return (if Fitted_Arm (M, E) > 0
              then (Pose => Identity, Position_Covariance => [others => [others => 0.0]],
                    Rotation_Covariance => [others => [others => 0.0]])
              else (others => <>));
   end Eye_In_Tool;

   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean is
   begin
      if Natural (A) > Arm_Count (M) then
         return False;
      end if;
      declare
         G  : constant Group_Id := Arm_Group (M, A);
         At_Beat : constant Natural := Natural (O.Beat);
      begin
         --  The push in effect at that beat: the latest one begun by then.
         for K in reverse 1 .. Steps.Episodes (M, G) loop
            declare
               E : Episode renames M.Groups (G).Episodes (K);
            begin
               if E.Start <= At_Beat then
                  return E.Ended and then E.End_At <= At_Beat and then E.Blocked;
               end if;
            end;
         end loop;
         return False;
      end;
   end Blocked;

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask is
     (if E <= O.Images.Last_Index then Driver.Images.Create (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)))
      else Driver.Images.Create (0, 0));

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate is (Unknown);

   function Still (M : Model) return Boolean is
     (Stillness.All_Still (M));

   function Group_Count (M : Model) return Natural is (Natural (M.Groups.Length));

   function Group_Size (M : Model; G : Group_Id) return Natural is
     (if G <= M.Groups.Last_Index then M.Groups (G).Size else 0);

   function Is_Commandable (M : Model; G : Group_Id) return Boolean is
     (G <= M.Groups.Last_Index and then M.Groups (G).Commandable);

   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real is
     (Channels.Noise (M, G, Channel));

   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate is
      Best : Estimate := Unknown;
   begin
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S    : Eye_Stream renames M.Eyes (E);
            Kept : constant Natural := Natural (S.Kept_Groups.Length);
            N    : constant Natural := (if Kept = 0 then 0 else Natural (S.Gains.Length) / Kept);
            Column : Natural := 0;
         begin
            for K in 0 .. Kept - 1 loop
               if S.Kept_Groups (K) = Natural (G) and then S.Kept_Channels (K) = Channel then
                  Column := K + 1;
               end if;
            end loop;
            if Column > 0 and then Response (M, G, E) in Patch | Undecided | Whole then
               declare
                  Gain, Spread : Real := 0.0;
               begin
                  for Cell in 0 .. N - 1 loop
                     Gain := Gain + S.Gains (Cell * Kept + Column - 1);
                     Spread := Spread + S.Gain_Variances (Cell * Kept + Column - 1);
                  end loop;
                  --  A step is seen when the displacement pattern it causes,
                  --  matched against the eye's cells, stands out of their
                  --  noise: a test of one degree, passed from Z / sqrt (Gain) on.
                  if Gain > 0.0 then
                     declare
                        Step : constant Real := Driver.Conventions.Z / Sqrt (Gain);
                     begin
                        if not Known (Best) or else Step < Best.Value then
                           Best := (Value => Step, Sigma => Step * Sqrt (Spread) / (2.0 * Gain), Degrees_Of_Freedom => 0);
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
      --  A step the reading cannot tell from its own noise is no step: the
      --  change of two readings stands Z of its sigmas above zero from there
      --  on. A lock-in that credits a group with the pictures' motion beside
      --  its tiny readings can fit a step of 1e-17, which no reading can show,
      --  and every user of the step would take a hair's shortfall for one an
      --  eye can see. Where the noise is not measured nothing tells a step
      --  from it, and there is none.
      if Known (Best) then
         declare
            Noise : constant Real := Channels.Noise (M, G, Channel);
         begin
            if Noise >= Real'Last then
               return Unknown;
            end if;
            Best.Value := Real'Max (Best.Value, Driver.Conventions.Z * Noise * Sqrt (2.0));
         end;
      end if;
      return Best;
   end Visible_Step;

   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response is
     (Graph.Effect (M, G, E).Verdict);

   function Responding (M : Model; G : Group_Id; E : Eye_Id) return Natural is
     (Graph.Effect (M, G, E).Responding);

   function Image_Lag (M : Model; E : Eye_Id) return Integer is
     (if E <= M.Lags.Last_Index then M.Lags (E) else 0);

   function Lag_Known (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Lag_Known.Last_Index and then M.Lag_Known (E));

   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base is
     (if Role (M, G) = Closer then M.Graph.Arm_Of (G) else 0);

   function Carrier_Group (M : Model) return Group_Id'Base is (M.Graph.Carrier);

   function Contract_Breach (M : Model; G : Group_Id) return Natural is
     (if G <= M.Graph.Breach.Last_Index then M.Graph.Breach (G) else 0);

   function Describe (M : Model) return String is
      use Ada.Strings.Unbounded;
      use Driver.Log;
      T : Unbounded_String;
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Append (T, "group" & Group_Id'Image (G) & ": " & Image (M.Groups (G).Size) & " values, "
                 & (if M.Groups (G).Commandable then "commandable" else "not commandable") & ", "
                 & Group_Role'Image (Role (M, G)));
         if M.Graph.Arm_Of.Length > 0 and then M.Graph.Arm_Of (G) > 0 then
            Append (T, " (arm" & Arm_Id'Image (M.Graph.Arm_Of (G)) & ")");
         end if;
         if Contract_Breach (M, G) > 0 then
            Append (T, ", breaks clause" & Natural'Image (Contract_Breach (M, G)));
         end if;
         for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
            declare
               F : constant Eye_Effect := Graph.Effect (M, G, E);
            begin
               if F.Verdict /= Unmeasured then
                  Append (T, "; eye" & Eye_Id'Image (E) & " " & Eye_Response'Image (F.Verdict) & " "
                          & Image (F.Responding) & "/" & Image (F.Textured));
               end if;
            end;
         end loop;
         Append (T, ASCII.LF);
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Mt : constant Mount := Eye_Mount (M, E);
         begin
            Append (T, "eye" & Eye_Id'Image (E) & ": "
                    & (if Lag_Known (M, E) then "image lag" & Integer'Image (Image_Lag (M, E)) & " beats, "
                        else "image lag unmeasured, ")
                    & Mount_Kind'Image (Mt.Kind)
                    & (if Mt.Kind = Arm_Carried then " on arm" & Arm_Id'Image (Mt.Arm) else "")
                    & ", rest noise " & Driver.Log.Image (M.Eyes (E).Rest_Factor, 2) & " times its floor"
                    & (if M.Eyes (E).Rest_Counts_Known
                        then ", at most" & M.Eyes (E).Rest_Count_Max'Image & " cells move at rest ("
                             & Driver.Log.Image (M.Eyes (E).Rest_Count_Beats) & " beats)"
                        else "") & ASCII.LF);
         end;
      end loop;
      return To_String (T);
   end Describe;

end Driver.Robot;
