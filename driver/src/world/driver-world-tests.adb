with Driver.Bytes;
with Driver.Commands;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Tests;
with Driver.World.Pairs.Tests;
with Driver.World.Regions.Tests;
with Driver.World.Estimates.Tests;
with Driver.World.Supports.Tests;
with Driver.World.Tracking.Tests;

package body Driver.World.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;

   function Ray (C : Pinhole; Px : Driver.World.Cameras.Pixel) return Ray_Estimate is
      In_Eye : constant Vec3 := Unit ([(Px.U - C.Centre_U) / C.Focal, (Px.V - C.Centre_V) / C.Focal, 1.0]);
   begin
      return (Origin    => (Mean => C.Pose_In_World.Translation, Covariance => 1.0e-12 * Identity3),
              Direction => (Unit_Vector => C.Pose_In_World.Rotation * In_Eye, Sigma => C.Pixel_Sigma / C.Focal));
   end Ray;

   procedure Project (C : Pinhole; Point : Vec3; Px : out Driver.World.Cameras.Pixel; Visible : out Boolean) is
      P : constant Vec3 := Transpose (C.Pose_In_World.Rotation) * (Point - C.Pose_In_World.Translation);
   begin
      Px := (U => 0.0, V => 0.0);
      Visible := False;
      if P (3) <= 0.0 then
         return;
      end if;
      Px := (U => C.Centre_U + C.Focal * P (1) / P (3), V => C.Centre_V + C.Focal * P (2) / P (3));
      Visible := Px.U >= 0.0 and then Px.V >= 0.0 and then Px.U < Real (C.Columns) and then Px.V < Real (C.Rows);
   end Project;

   function Pose (C : Pinhole) return Pose_Estimate is
     ((Pose => C.Pose_In_World, Position_Covariance => 1.0e-12 * Identity3, Rotation_Covariance => 1.0e-12 * Identity3));

   function Looking_At (Eye, Target : Vec3; Focal : Real; Columns, Rows : Natural; Pixel_Sigma : Real)
     return Pinhole
   is
      Z : constant Vec3 := Unit (Target - Eye);
      --  Rows run down the image: y as near world -z as the axis allows.
      X : constant Vec3 := Unit (Cross (Z, [0.0, 0.0, -1.0]));
      Y : constant Vec3 := Cross (Z, X);
      R : constant Mat3 := [[X (1), Y (1), Z (1)], [X (2), Y (2), Z (2)], [X (3), Y (3), Z (3)]];
   begin
      return (Pose_In_World => (Rotation => R, Translation => Eye), Focal => Focal,
              Centre_U => Real (Columns) / 2.0, Centre_V => Real (Rows) / 2.0, Columns => Columns, Rows => Rows,
              Pixel_Sigma => Pixel_Sigma);
   end Looking_At;

   function Square (C0, R0 : Natural) return Driver.Images.Mask is
      M : Driver.Images.Mask := Driver.Images.Create (40, 30);
   begin
      for R in R0 .. R0 + 7 loop
         for C in C0 .. C0 + 7 loop
            Driver.Images.Include (M, C, R);
         end loop;
      end loop;
      return M;
   end Square;

   procedure Adopted_And_Remembered is
      --  An unmeasured body with one eye: things are adopted and kept by
      --  their pixels, places remembered, and a new episode forgets both.
      M : Driver.Robot.Model;
      H : Driver.Robot.Hand.Hands;
      S : Scene;
      O : Observation;
      First, Again, Other : Thing_Id;
      Home : Place_Id;
   begin
      O.Images.Append (Driver.Images.Create (40, 30, [1 .. 3600 => Driver.Bytes.Byte'Last]));
      O.Beat := 1;
      Adopt (S, M, 1, O, Square (5, 5), First);
      Adopt (S, M, 1, O, Square (6, 6), Again);
      Adopt (S, M, 1, O, Square (25, 15), Other);
      Check (Again = First and then Other /= First and then Thing_Count (S) = 2,
             "a region of the same pixels was made a new thing, or another one was not");
      Check (Seen_In (S, First, 1) and then not Seen_In (S, First, 2), "an adopted thing is not seen where it was adopted");
      Remember (S, (Mean => [0.1, 0.2, 0.3], Covariance => [others => [others => 0.0]]), Home);
      Check (Place_Count (S) = 1 and then Where (S, Home).Mean (2) = 0.2, "a remembered place is not where it was");
      --  An unmeasured body is never still: nothing is seen while it moves.
      Observe (S, M, H, O, Driver.Commands.Hold);
      Check (not Seen_In (S, First, 1), "a thing is seen while the body is not known to be still");
      New_Episode (S);
      Check (Thing_Count (S) = 0 and then Place_Count (S) = 0, "a new episode kept the things or the places");
   end Adopted_And_Remembered;

   procedure Register is
   begin
      Driver.Tests.Register ("world.scene.adopt", "things are not kept by their pixels, or an episode forgets nothing",
                             Adopted_And_Remembered'Access);
      Driver.World.Regions.Tests.Register;
      Driver.World.Pairs.Tests.Register;
      Driver.World.Supports.Tests.Register;
      Driver.World.Estimates.Tests.Register;
      Driver.World.Tracking.Tests.Register;
   end Register;

end Driver.World.Tests;
