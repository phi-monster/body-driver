with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Log;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Kinematics;
with Driver.Robot.Steps;

package body Driver.Robot.Hand.Pressing is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Robot.Motion.Plan_Status;

   --  The largest variance of a covariance along any axis.
   function Widest (C : Mat3) return Real is
      Values  : Vec3;
      Vectors : Mat3;
   begin
      Symmetric_Eigensystem (C, Values, Vectors);
      return Real'Max (Values (1), Real'Max (Values (2), Values (3)));
   end Widest;

   function Least_Push (M : Model; Arm : Arm_Id; O : Observation) return Real is
      Tool : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      if Tool.Position_Covariance (1, 1) = Real'Last then
         return Real'Last;   --  the tool's place is not measured
      end if;
      --  The tool's place in the arm's own unit (Kinematics.Place_Covariance_In_Unit): every length a press compares
      --  is in it, and what the fit leaves uncertain of the arm's lengths against its depths, which moves every place
      --  with the unit, is none of a move's noise there. Taken from Tool_In_Arm's covariance, a18's least push was
      --  3.6 units (10-16, a14's body: the eye 6.5 units below the reference, uncertain by 0.82 there, 12 per cent,
      --  where the truth puts the arm's tool within 0.16 mm, a seventieth of a unit, with its scale taken out), the
      --  least tilt that tells a tip from a stop 1.29 rad, which no press at the open opening could be lowered at.
      declare
         use type Driver.Observations.Group_Id;
         G : constant Group_Id := Arm_Group (M, Arm);
      begin
         if G > O.Readings.Last_Index or else O.Readings.Element (G)'Length /= Group_Size (M, G) then
            return Real'Last;
         end if;
         declare
            In_Unit : constant Mat3 := Kinematics.Place_Covariance_In_Unit (M, Arm, O.Readings.Element (G));
         begin
            if In_Unit (1, 1) = Real'Last then
               return Real'Last;
            end if;
            --  And no less than the least move of the tool the body executes: the place of a tool the fit holds to
            --  within nothing gave a least push of nothing, and a push under every joint's least executed step moves
            --  nothing (a servo's dead band); the press doubled from Real'Model_Epsilon and took 6130 beats to its
            --  hand's end (s39, servo seed 7 of the fake robot, 10-17: "doubling from 0.0000 until blocked"). The
            --  least of the tool's moves under one joint's least executed step, from where the arm stands.
            declare
               Now   : constant Real_Array := O.Readings.Element (G);
               Here  : constant Vec3 := Kinematics.Eye_In_Reference (M, Arm, Now).Translation;
               Moves : Real := Real'Last;
            begin
               for C in 1 .. Group_Size (M, G) loop
                  declare
                     Step : constant Real := Steps.Executed_Least (M, G, C);
                  begin
                     if Step > 0.0 and then Step < Real'Last then
                        declare
                           Then_R : Real_Array := Now;
                        begin
                           Then_R (Then_R'First + C - 1) := Then_R (Then_R'First + C - 1) + Step;
                           Moves := Real'Min (Moves, abs (Kinematics.Eye_In_Reference (M, Arm, Then_R).Translation - Here));
                        end;
                     end if;
                  end;
               end loop;
               return Real'Max (Threshold (Vector_Gate (Vec3'Length)) * Sqrt (Widest (In_Unit)),
                                (if Moves < Real'Last then Moves else 0.0));
            end;
         end;
      end;
   end Least_Push;

   procedure Aim
     (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed; Yaw : Real := 0.0;
      Shift : Vec3 := [0.0, 0.0, 0.0])
   is
      Tool    : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
      Down    : constant Direction_Estimate := Up_In_Arm (M, Arm);
      --  No eye saw a surface below the hand (no table in the arm's frame, its own eye's or the world's): the press
      --  goes by touch, along the lobe's line of sight as the hand stands. The first contact is where the surface is,
      --  and presses tilted about that line solve the surface and the tip together (Touch, an unmeasured surface);
      --  the aim turns nothing, and the yaw turns the hand about that line through the eye (item 14).
      Blind   : constant Boolean := Down.Sigma >= Real'Last;
   begin
      Result := (others => <>);
      Result.Ok := Tool.Position_Covariance (1, 1) < Real'Last and then Eye_Mount (M, Eye).Kind = Arm_Carried
        and then (Blind or else abs Down.Unit_Vector > 0.0) and then abs Along > 0.0;
      if not Result.Ok then
         return;
      end if;
      if Blind then
         Result.Into := Unit (Tool.Pose.Rotation * Along);
         Result.Above := Tool.Pose;
      else
         Result.Into := -Down.Unit_Vector;
         Result.Above := Driver.Robot.Hand.Aims.Turned_About
           (Tool.Pose, Eye_In_Tool (M, Eye, O).Pose.Translation, Along, Result.Into);
      end if;
      if Yaw /= 0.0 then
         declare
            Eye_Here : constant Vec3 := Eye_In_Tool (M, Eye, O).Pose.Translation;
            Eye_At   : constant Vec3 := Result.Above * Eye_Here;
            Rotation : constant Mat3 := Exp (Yaw * Result.Into) * Result.Above.Rotation;
         begin
            Result.Above := (Rotation => Rotation, Translation => Eye_At - Rotation * Eye_Here);
         end;
      end if;
      Result.Above.Translation := Result.Above.Translation + Shift;
      Result.Turn := Angle (Transpose (Tool.Pose.Rotation) * Result.Above.Rotation);
      --  The aim points the line of sight at "down" as the arm's eye saw the table: known to that direction's sigma; a
      --  blind aim is the tool's own turn, known as the fit knows it.
      --  The line of sight is pointed; the turn about it is Yaw where the arm reaches the whole pose, else the nearest
      --  pose's that points it (Free_About, Kinematics.Solve_Pose), and the aim is where the plan takes the tool.
      Result.Plan := Driver.Robot.Motion.Plan_Reach_In_Arm
        (M, Arm, O, (Pose => Result.Above, Position_Only => False, Free_About => Along),
         Goal_Turn_Sigma => (if Blind then 0.0 else Down.Sigma));
      Result.Yawed := Yaw;
      if Driver.Robot.Motion.Status (Result.Plan) = Driver.Robot.Motion.Planned then
         declare
            There : Observation := O;
            Asked : constant Mat3 := Result.Above.Rotation;
         begin
            There.Readings.Replace_Element (Arm_Group (M, Arm), Driver.Robot.Motion.Last_Readings (Result.Plan));
            Result.Least := Least_Push (M, Arm, There);
            Result.Above := Tool_In_Arm (M, Arm, There).Pose;
            Result.Turn := Angle (Transpose (Tool.Pose.Rotation) * Result.Above.Rotation);
            --  A turn the fit cannot tell from the one asked (its Angle_Sigma) is the one asked.
            declare
               Extra : constant Real := Real'(Driver.Numerics.Log (Result.Above.Rotation * Transpose (Asked)) * Result.Into);
            begin
               Result.Yawed := (if Driver.Uncertain.Significant (Extra, Driver.Robot.Kinematics.Angle_Sigma (M, Arm))
                                then Real'Remainder (Yaw + Extra, 2.0 * Ada.Numerics.Pi) else Yaw);
            end;
         end;
      end if;
   end Aim;

   function Off_Aim (M : Model; Arm : Arm_Id; O : Observation; A : Aimed) return Aim_Offset is
      Sigma : constant Real := Driver.Robot.Kinematics.Angle_Sigma (M, Arm);
      Down  : constant Direction_Estimate := Up_In_Arm (M, Arm);
      Tool  : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      if not A.Ok or else Tool.Position_Covariance (1, 1) = Real'Last then
         return (others => <>);
      end if;
      if Sigma = Real'Last then
         return (others => <>);
      end if;
      declare
         Values  : Vec3;
         Vectors : Mat3;
         --  What the arm's own landing leaves: the tool at the readings a free push falls short by, along the way
         --  the arm stands short of the aim's readings (all of the way when it stands no farther short than that).
         G       : constant Group_Id := Arm_Group (M, Arm);
         Target  : constant Real_Array := Driver.Robot.Motion.Last_Readings (A.Plan);
         Now     : constant Real_Array := O.Readings.Element (G);
         Free    : constant Real := Driver.Robot.Steps.Free_Landing (M, G);
         Short   : Real := 0.0;
         Allow_Place, Allow_Turn : Real := 0.0;
      begin
         Symmetric_Eigensystem (Tool.Position_Covariance, Values, Vectors);
         if Target'Length = Now'Length then
            for C in 0 .. Target'Length - 1 loop
               Short := Short + (Target (Target'First + C) - Now (Now'First + C)) ** 2;
            end loop;
            Short := Sqrt (Short);
            if Free > 0.0 and then Short > 0.0 then
               declare
                  There  : Observation := O;
                  Part   : constant Real := Real'Min (1.0, Free / Short);
                  Landed : Real_Array (Now'Range);
               begin
                  for C in 0 .. Now'Length - 1 loop
                     Landed (Now'First + C) :=
                       Target (Target'First + C) - Part * (Target (Target'First + C) - Now (Now'First + C));
                  end loop;
                  There.Readings.Replace_Element (G, Landed);
                  declare
                     At_Landing : constant Pose_Estimate := Tool_In_Arm (M, Arm, There);
                  begin
                     if At_Landing.Position_Covariance (1, 1) < Real'Last then
                        Allow_Place := abs (At_Landing.Pose.Translation - A.Above.Translation);
                        Allow_Turn := Angle (Transpose (At_Landing.Pose.Rotation) * A.Above.Rotation);
                     end if;
                  end;
               end;
            end if;
         end if;
         return (Place       => abs (Tool.Pose.Translation - A.Above.Translation),
                 Place_Sigma => Sqrt (Sigma ** 2 + Real'Max (Values (1), Real'Max (Values (2), Values (3)))),
                 Turn        => Angle (Transpose (Tool.Pose.Rotation) * A.Above.Rotation),
                 --  A blind aim's turn (no table: the tool's own) is told against the fit's sigma alone.
                 Turn_Sigma  => (if Down.Sigma < Real'Last then Sqrt (Sigma ** 2 + Down.Sigma ** 2) else Sigma),
                 Allow_Place => Allow_Place,
                 Allow_Turn  => Allow_Turn);
      end;
   end Off_Aim;

   procedure Aim_Reaching
     (M       : Model;
      Arm     : Arm_Id;
      Eye     : Eye_Id;
      O       : Observation;
      Along   : Vec3;
      Tip     : Point_Estimate;
      Surface : Driver.Geometry.Plane_Estimate;
      Result  : out Aimed;
      Yaw     : out Real;
      Reaches : out Boolean;
      Unmeasured : out Boolean;
      Why     : out Ada.Strings.Unbounded.Unbounded_String;
      Footprint : Real := 0.0)
   is
      Group : constant Group_Id := Arm_Group (M, Arm);
      Best       : Aimed;               --  the surest aim that reaches, so far
      Best_Yaw   : Real := 0.0;
      Best_Push  : Real := Real'Last;   --  its least push: how surely the fit places the tool there
      Any        : Boolean := False;
      Tried_Yaw  : Real;
      --  The first landing found under something standing on the table, and how near anything standing counted there:
      --  where the circle of turns was refused for that.
      Stood        : Boolean := False;
      Stood_At     : Vec3 := [0.0, 0.0, 0.0];
      Stood_Within : Real := 0.0;

      --  The circle of turns about the way down (Aim's Yaw), the aim moved by Shift.
      procedure Circle (Shift : Vec3) is
      begin
         for K in 0 .. 7 loop
            --  0, an eighth of a turn one way, the other, a quarter each way, three eighths each way, half a turn.
            Yaw := (if K mod 2 = 1 then 1.0 else -1.0) * Real ((K + 1) / 2) * (Ada.Numerics.Pi / 4.0);
            Aim (M, Arm, Eye, O, Along, Result, Yaw, Shift);
            exit when not Result.Ok;
            Reaches := Driver.Robot.Motion.Status (Result.Plan) = Driver.Robot.Motion.Planned;
            if K = 0 and then not Reaches then
               Unmeasured := Driver.Robot.Motion.Status (Result.Plan) = Driver.Robot.Motion.Unmeasured;
               Why := Ada.Strings.Unbounded.To_Unbounded_String (Driver.Robot.Motion.Why (Result.Plan));
            end if;
            if Reaches then
               declare
                  There : Observation := O;
                  Down  : Estimate;
               begin
                  There.Readings.Replace_Element (Group, Driver.Robot.Motion.Last_Readings (Result.Plan));
                  Down := Gap (M, Arm, There, Tip, Surface, Result.Into);
                  if Known (Down) and then Down.Value > 0.0 then
                     declare
                        Deep : constant Driver.Robot.Motion.Plan := Lowered (M, Arm, There, Result.Into, Down.Value);
                     begin
                        Reaches := Driver.Robot.Motion.Status (Deep) = Driver.Robot.Motion.Planned;
                        if K = 0 and then not Reaches then
                           Why := Ada.Strings.Unbounded.To_Unbounded_String
                             ("the descent from it to the contact the presses predict: " & Driver.Robot.Motion.Why (Deep));
                        end if;
                     end;
                     --  Where the tip lands, the tip at the aim lowered by the gap: something standing above the table there
                     --  (the arm's eye's tracks), within what the landing is uncertain by and the tip's region, is what the
                     --  press would meet, not the table. This aim is not taken; the next turn of the circle lands elsewhere.
                     if Reaches then
                        declare
                           Tool_There : constant Pose_Estimate := Tool_In_Arm (M, Arm, There);
                           Landing    : constant Vec3 := Tool_There.Pose * Tip.Mean + Down.Value * Result.Into;
                           Measured   : constant Boolean :=
                             Tool_There.Position_Covariance (1, 1) < Real'Last and then Tip.Covariance (1, 1) < Real'Last;
                           --  How far across the table the landing may lie: the tool's place, the tip's (turned into the
                           --  arm's frame, and moved by the tool's turn about it) and the gap's along the way in, as the
                           --  table's plane sees them. A tip known along its line of sight far worse than across it lands no
                           --  wider across the table than its line leans: taken by the widest axis of each, whatever its
                           --  direction, every tilted press of q15's open opening (servo seed 7 of the fake robot, 10-19;
                           --  its tip 4.06 +- 1.99 along its sight, its region 1.06 wide across it) was refused for 16 of
                           --  the eye's tracks within 6.43 of its landing, and no tip of it was ever confirmed.
                           Lever      : constant Vec3 := Tool_There.Pose.Rotation * Tip.Mean;
                           Across     : constant Mat3 := Identity3 - Outer (Unit (Surface.Normal), Unit (Surface.Normal));
                           Spread     : constant Mat3 :=
                             (if Measured
                              then Tool_There.Position_Covariance
                                   + Tool_There.Pose.Rotation * Tip.Covariance * Transpose (Tool_There.Pose.Rotation)
                                   + Skew (Lever) * Tool_There.Rotation_Covariance * Transpose (Skew (Lever))
                                   + Down.Sigma ** 2 * Outer (Result.Into, Result.Into)
                              else Identity3);
                           Within     : constant Real :=
                             (if Measured
                              then Driver.Conventions.Z * Sqrt (Widest (Across * Spread * Across)) + Footprint
                              else 0.0);
                           Standing   : constant Natural :=
                             (if Measured then Driver.Robot.Kinematics.Standing_Near (M, Arm, Landing, Within) else 0);
                        begin
                           if Standing > 0 then
                              Reaches := False;
                              if not Stood then
                                 Stood := True;
                                 Stood_At := Landing;
                                 Stood_Within := Within;
                              end if;
                              if K = 0 then
                                 Why := Ada.Strings.Unbounded.To_Unbounded_String
                                   ("something stands above the table where the tip would land:" & Standing'Image
                                    & " of the eye's tracks within " & Driver.Log.Image (Within, 4) & " of it");
                              end if;
                           end if;
                        end;
                     end if;
                  end if;
               end;
            end if;
            if Reaches then
               declare
                  There : Observation := O;
               begin
                  There.Readings.Replace_Element (Group, Driver.Robot.Motion.Last_Readings (Result.Plan));
                  declare
                     Push : constant Real := Least_Push (M, Arm, There);
                  begin
                     --  The least rotation of those as sure: the turns are tried from the least, and a later one is taken
                     --  only where the fit places the tool surer by more than the fraction that counts as no change. By the
                     --  bare comparison, h24 (servo seed 3, 10-17) took turns of 2.36 and 3.14 rad for differences no fit
                     --  tells, and the arm did not reach four of its aims (blocked, or given up still moving).
                     if not Any or else Push < (1.0 - Driver.Conventions.Unchanged_Fraction) * Best_Push then
                        Any := True;
                        Best := Result;
                        Best_Yaw := Result.Yawed;
                        Best_Push := Push;
                     end if;
                  end;
               end;
            end if;
         end loop;
      end Circle;
   begin
      Reaches := False;
      Unmeasured := False;
      Yaw := 0.0;
      Why := Ada.Strings.Unbounded.Null_Unbounded_String;
      Result := (others => <>);
      Circle ([0.0, 0.0, 0.0]);
      --  Every turn the arm reaches lands its tip on something standing on the table: the aim keeps the eye where it
      --  stands, and a tip on the lobe's line of sight lands below the eye at every turn of the circle. The press moves
      --  across the table, by the least that puts its landing on a place of the table nothing stands within the same
      --  reach of (Kinematics.Clear_Spot), and the circle is tried there once. y7 (servo seed 7 of the fake robot,
      --  10-18): every turn of its open presses landed within 2.64 of one track standing above the table, the tilts
      --  of the open opening were never pressed, and the hand's sizes stayed unmeasured.
      if not Any and then Stood then
         declare
            Spot  : Vec3;
            Found : Boolean;
         begin
            Driver.Robot.Kinematics.Clear_Spot (M, Arm, Stood_At, Stood_Within, Spot, Found);
            if Found then
               declare
                  Normal : constant Vec3 := Driver.Robot.Kinematics.Table (M, Arm).Normal;
                  Off    : constant Vec3 := Spot - Stood_At;
                  Across : constant Vec3 := Off - Real'(Off * Normal) * Normal;
               begin
                  Driver.Log.Line (Driver.Log.Robot, "hand: a press's every turn lands on something standing on the"
                                   & " table; it moves " & Driver.Log.Image (abs Across, 4)
                                   & " across the table, to the nearest place nothing stands within "
                                   & Driver.Log.Image (Stood_Within, 4) & " of");
                  Circle (Across);
               end;
            end if;
         end;
      end if;
      Tried_Yaw := Yaw;
      if Any then
         Result := Best;
         Yaw := Best_Yaw;
         Reaches := True;
      else
         Yaw := Tried_Yaw;
         Reaches := False;
      end if;
   end Aim_Reaching;

   function Lowered
     (M : Model; Arm : Arm_Id; O : Observation; Into : Vec3; By : Real; Turn_Free : Boolean := False)
      return Driver.Robot.Motion.Plan
   is
      Tool : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      --  The tool's direction that points along Into keeps pointing so, and its turn about it is held where the arm
      --  reaches the whole pose, else the nearest pose's (Free_About), as the aim leaves it: held whole, a descent of
      --  an arm with five joints asks a full pose at every waypoint.
      return Driver.Robot.Motion.Plan_Reach_In_Arm
        (M, Arm, O,
         (Pose          => (Rotation => Tool.Pose.Rotation, Translation => Tool.Pose.Translation + By * Into),
          Position_Only => Turn_Free,
          Free_About    => Transpose (Tool.Pose.Rotation) * Into));
   end Lowered;

   function Gap
     (M       : Model;
      Arm     : Arm_Id;
      O       : Observation;
      Tip     : Point_Estimate;
      Surface : Driver.Geometry.Plane_Estimate;
      Into    : Vec3) return Estimate
   is
      Tool : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      if not Known (Tip) or else not Driver.Geometry.Known (Surface) or else Tool.Position_Covariance (1, 1) = Real'Last
      then
         return Unknown;
      end if;
      declare
         Rot    : constant Mat3 := Tool.Pose.Rotation;
         Here   : constant Vec3 := Tool.Pose * Tip.Mean;
         --  A turn of the tool by a small rotation vector w moves the tip by
         --  w x (R Tip), so its height by w . Lever.
         Lever  : constant Vec3 := Cross (Rot * Tip.Mean, Surface.Normal);
         Where  : constant Point_Estimate :=
           (Mean => Here, Covariance => Rot * Tip.Covariance * Transpose (Rot) + Tool.Position_Covariance);
         On_It  : constant Estimate := Driver.Geometry.Height (Surface, Where);
         Lean   : constant Real := Surface.Normal * (-Into);
      begin
         if Lean <= 0.0 then
            return Unknown;
         end if;
         return (Value              => On_It.Value / Lean,
                 Sigma              => Sqrt (On_It.Sigma ** 2 + Lever * (Tool.Rotation_Covariance * Lever)) / Lean,
                 Degrees_Of_Freedom => On_It.Degrees_Of_Freedom);
      end;
   end Gap;

end Driver.Robot.Hand.Pressing;
