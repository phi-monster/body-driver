with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Log;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Kinematics;

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
      return Threshold (Vector_Gate (Vec3'Length)) * Sqrt (Widest (Tool.Position_Covariance));
   end Least_Push;

   procedure Aim
     (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed; Yaw : Real := 0.0)
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
      Result.Turn := Angle (Transpose (Tool.Pose.Rotation) * Result.Above.Rotation);
      --  The aim points the line of sight at "down" as the arm's eye saw the table: known to that direction's sigma; a
      --  blind aim is the tool's own turn, known as the fit knows it.
      Result.Plan := Driver.Robot.Motion.Plan_Reach_In_Arm
        (M, Arm, O, (Pose => Result.Above, Position_Only => False), Goal_Turn_Sigma => (if Blind then 0.0 else Down.Sigma));
      if Driver.Robot.Motion.Status (Result.Plan) = Driver.Robot.Motion.Planned then
         declare
            There : Observation := O;
         begin
            There.Readings.Replace_Element (Arm_Group (M, Arm), Driver.Robot.Motion.Last_Readings (Result.Plan));
            Result.Least := Least_Push (M, Arm, There);
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
      begin
         Symmetric_Eigensystem (Tool.Position_Covariance, Values, Vectors);
         return (Place       => abs (Tool.Pose.Translation - A.Above.Translation),
                 Place_Sigma => Sqrt (Sigma ** 2 + Real'Max (Values (1), Real'Max (Values (2), Values (3)))),
                 Turn        => Angle (Transpose (Tool.Pose.Rotation) * A.Above.Rotation),
                 --  A blind aim's turn (no table: the tool's own) is told against the fit's sigma alone.
                 Turn_Sigma  => (if Down.Sigma < Real'Last then Sqrt (Sigma ** 2 + Down.Sigma ** 2) else Sigma));
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
   begin
      Reaches := False;
      Unmeasured := False;
      Yaw := 0.0;
      Why := Ada.Strings.Unbounded.Null_Unbounded_String;
      Result := (others => <>);
      for K in 0 .. 7 loop
         --  0, a quarter turn one way, the other, half a turn each way, three quarters each way, a whole one.
         Yaw := (if K mod 2 = 1 then 1.0 else -1.0) * Real ((K + 1) / 2) * (Ada.Numerics.Pi / 4.0);
         Aim (M, Arm, Eye, O, Along, Result, Yaw);
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
                        Within     : constant Real :=
                          (if Measured
                           then Driver.Conventions.Z * Sqrt (Widest (Tool_There.Position_Covariance) + Widest (Tip.Covariance))
                                + Footprint
                           else 0.0);
                        Standing   : constant Natural :=
                          (if Measured then Driver.Robot.Kinematics.Standing_Near (M, Arm, Landing, Within) else 0);
                     begin
                        if Standing > 0 then
                           Reaches := False;
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
         exit when Reaches;
      end loop;
   end Aim_Reaching;

   function Lowered
     (M : Model; Arm : Arm_Id; O : Observation; Into : Vec3; By : Real; Turn_Free : Boolean := False)
      return Driver.Robot.Motion.Plan
   is
      Tool : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      return Driver.Robot.Motion.Plan_Reach_In_Arm
        (M, Arm, O,
         (Pose          => (Rotation => Tool.Pose.Rotation, Translation => Tool.Pose.Translation + By * Into),
          Position_Only => Turn_Free));
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
