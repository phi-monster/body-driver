with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Robot.Hand.Frames is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Pose_Known (Frame : Pose_Estimate) return Boolean is
     (Frame.Position_Covariance (1, 1) < Real'Last and then Frame.Rotation_Covariance (1, 1) < Real'Last);

   function Into (Frame : Pose_Estimate; Point : Point_Estimate) return Point_Estimate is
      R    : constant Mat3 := Frame.Pose.Rotation;
      Mean : constant Vec3 := Frame.Pose * Point.Mean;
   begin
      if not Known (Point) or else not Pose_Known (Frame) then
         return (Mean => Mean, Covariance => [others => [others => Real'Last]]);
      end if;
      declare
         --  A small turn w of the frame (in the parent) moves the point by
         --  w x p = -p x w.
         Lever : constant Mat3 := Skew (R * Point.Mean);
      begin
         return (Mean       => Mean,
                 Covariance => R * Point.Covariance * Transpose (R) + Frame.Position_Covariance
                               + Lever * Frame.Rotation_Covariance * Transpose (Lever));
      end;
   end Into;

   function Into (Frame : Pose_Estimate; Line : Ray_Estimate) return Ray_Estimate is
      U    : constant Vec3 := Frame.Pose.Rotation * Line.Direction.Unit_Vector;
      Turn : constant Mat3 := Frame.Rotation_Covariance;
   begin
      if Line.Direction.Sigma >= Real'Last or else not Pose_Known (Frame) then
         return (Origin    => Into (Frame, Line.Origin),
                 Direction => (Unit_Vector => U, Sigma => Real'Last));
      end if;
      declare
         --  The direction's sigma is one number for every direction across
         --  the line: the turn's variance about the two axes across it, averaged.
         Across : constant Real := (Turn (1, 1) + Turn (2, 2) + Turn (3, 3) - U * (Turn * U)) / 2.0;
      begin
         return (Origin    => Into (Frame, Line.Origin),
                 Direction => (Unit_Vector => U, Sigma => Sqrt (Line.Direction.Sigma ** 2 + Real'Max (0.0, Across))));
      end;
   end Into;

end Driver.Robot.Hand.Frames;
