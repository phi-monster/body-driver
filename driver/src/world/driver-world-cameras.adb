with Ada.Numerics.Long_Elementary_Functions;

package body Driver.World.Cameras is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;

   function Radians_Per_Pixel (C : Camera'Class; Px : Pixel) return Real is
      Here   : constant Ray_Estimate := C.Ray (Px);
      Beside : constant Ray_Estimate := C.Ray ((U => Px.U + 1.0, V => Px.V));
   begin
      if Here.Direction.Sigma >= Real'Last or else Beside.Direction.Sigma >= Real'Last then
         return Real'Last;
      end if;
      return Arctan (abs Cross (Here.Direction.Unit_Vector, Beside.Direction.Unit_Vector),
                     Here.Direction.Unit_Vector * Beside.Direction.Unit_Vector);
   end Radians_Per_Pixel;

   function Width (C : Of_Body) return Natural is
     (if C.Seen.Images.Last_Index < C.Eye then 0 else Driver.Images.Width (C.Seen.Images (C.Eye)));

   function Height (C : Of_Body) return Natural is
     (if C.Seen.Images.Last_Index < C.Eye then 0 else Driver.Images.Height (C.Seen.Images (C.Eye)));

   function Ray (C : Of_Body; Px : Pixel) return Ray_Estimate is
     (Driver.Robot.Ray (C.Robot.all, C.Eye, C.Seen.all, Px));

   procedure Project (C : Of_Body; Point : Vec3; Px : out Pixel; Visible : out Boolean) is
   begin
      Driver.Robot.Project (C.Robot.all, C.Eye, C.Seen.all, Point, Px, Visible);
   end Project;

   function Pose (C : Of_Body) return Pose_Estimate is (Driver.Robot.Eye_Pose (C.Robot.all, C.Eye, C.Seen.all));

   function Self_Mask (C : Of_Body) return Driver.Images.Mask is
     (Driver.Robot.Self_Mask (C.Robot.all, C.Eye, C.Seen.all));

end Driver.World.Cameras;
