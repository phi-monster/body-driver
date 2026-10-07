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

   procedure Aim (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed) is
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
