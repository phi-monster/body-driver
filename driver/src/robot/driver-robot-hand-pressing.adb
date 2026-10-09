with Ada.Numerics.Long_Elementary_Functions;
with Driver.Robot.Hand.Aims;

package body Driver.Robot.Hand.Pressing is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Robot.Motion.Plan_Status;

   function Least_Push (M : Model; Arm : Arm_Id; O : Observation) return Real is
      Tool    : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
      Values  : Vec3;
      Vectors : Mat3;
   begin
      Symmetric_Eigensystem (Tool.Position_Covariance, Values, Vectors);
      return Threshold (Vector_Gate (Vec3'Length)) * Sqrt (Real'Max (Values (1), Real'Max (Values (2), Values (3))));
   end Least_Push;

   procedure Aim
     (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed; Yaw : Real := 0.0)
   is
      Tool    : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
      Down    : constant Direction_Estimate := Up_In_Arm (M, Arm);
   begin
      Result := (others => <>);
      Result.Ok := Down.Sigma < Real'Last and then Tool.Position_Covariance (1, 1) < Real'Last
        and then Eye_Mount (M, Eye).Kind = Arm_Carried;
      if not Result.Ok then
         return;
      end if;
      Result.Into := -Down.Unit_Vector;
      Result.Above := Driver.Robot.Hand.Aims.Turned_About
        (Tool.Pose, Eye_In_Tool (M, Eye, O).Pose.Translation, Along, Result.Into);
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
      Result.Plan := Driver.Robot.Motion.Plan_Reach_In_Arm (M, Arm, O, (Pose => Result.Above, Position_Only => False));
      if Driver.Robot.Motion.Status (Result.Plan) = Driver.Robot.Motion.Planned then
         declare
            There : Observation := O;
         begin
            There.Readings.Replace_Element (Arm_Group (M, Arm), Driver.Robot.Motion.Last_Readings (Result.Plan));
            Result.Least := Least_Push (M, Arm, There);
         end;
      end if;
   end Aim;

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
      Why     : out Ada.Strings.Unbounded.Unbounded_String)
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
               end if;
            end;
         end if;
         exit when Reaches;
      end loop;
   end Aim_Reaching;

   function Lowered
     (M : Model; Arm : Arm_Id; O : Observation; Into : Vec3; By : Real) return Driver.Robot.Motion.Plan
   is
      Tool : constant Pose_Estimate := Tool_In_Arm (M, Arm, O);
   begin
      return Driver.Robot.Motion.Plan_Reach_In_Arm
        (M, Arm, O,
         (Pose          => (Rotation => Tool.Pose.Rotation, Translation => Tool.Pose.Translation + By * Into),
          Position_Only => False));
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
