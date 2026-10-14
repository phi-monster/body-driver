with Ada.Numerics.Long_Elementary_Functions;

package body Driver.World.Holding is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Still_Held (At_Hold, Now : Estimate) return Boolean is
     (Known (At_Hold) and then Known (Now)
      and then 1.0 - Now.Value > 0.0 and then Significant (1.0 - Now.Value, Now.Sigma, Now.Degrees_Of_Freedom)
      and then not (At_Hold.Value - Now.Value > 0.0
                    and then Significant (At_Hold.Value - Now.Value, Sqrt (At_Hold.Sigma ** 2 + Now.Sigma ** 2))));

   function Largest_Variance (C : Mat3) return Real is (Real'Max (C (1, 1), Real'Max (C (2, 2), C (3, 3))));

   procedure Carry
     (Points   : in out Driver.World.Pairs.Match_Vectors.Vector;
      Normals  : in out Driver.World.Estimates.Normal_Vectors.Vector;
      From, To : Pose_Estimate)
   is
      Motion : constant Rigid := To.Pose * Inverse (From.Pose);
      Turn   : constant Mat3 := Motion.Rotation;
      Turns  : constant Real := Largest_Variance (From.Rotation_Covariance) + Largest_Variance (To.Rotation_Covariance);
   begin
      for K in Points.First_Index .. Points.Last_Index loop
         declare
            M     : Driver.World.Pairs.Match := Points (K);
            Lever : constant Real := abs (M.Point.Mean - From.Pose.Translation);
         begin
            M.Point := (Mean       => Motion * M.Point.Mean,
                        Covariance => Times (Times (Turn, M.Point.Covariance), Transposed (Turn))
                                      + From.Position_Covariance + To.Position_Covariance
                                      + (Lever ** 2 * Turns) * Identity3);
            Points.Replace_Element (K, M);
         end;
      end loop;
      for K in Normals.First_Index .. Normals.Last_Index loop
         Normals.Replace_Element (K, Times (Turn, Normals (K)));
      end loop;
   end Carry;

end Driver.World.Holding;
