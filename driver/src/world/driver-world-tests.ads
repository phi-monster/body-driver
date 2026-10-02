--  Self test of Driver.World (path B), and the pinhole cameras the world's
--  self tests stand in for the body's measured eyes with.

with Driver.Images;
with Driver.World.Cameras;

package Driver.World.Tests is

   procedure Register;

   type Pinhole is new Driver.World.Cameras.Camera with record
      Pose_In_World : Rigid;          --  camera frame: z along the optical axis, x along +U, y along +V
      Focal         : Real := 0.0;    --  pixels
      Centre_U      : Real := 0.0;
      Centre_V      : Real := 0.0;
      Columns       : Natural := 0;
      Rows          : Natural := 0;
      Pixel_Sigma   : Real := 0.0;    --  a line of sight's uncertainty, in pixels
   end record;

   overriding function Width (C : Pinhole) return Natural is (C.Columns);
   overriding function Height (C : Pinhole) return Natural is (C.Rows);
   overriding function Ray (C : Pinhole; Px : Driver.World.Cameras.Pixel) return Ray_Estimate;
   overriding procedure Project (C : Pinhole; Point : Vec3; Px : out Driver.World.Cameras.Pixel; Visible : out Boolean);
   overriding function Pose (C : Pinhole) return Pose_Estimate;
   overriding function Self_Mask (C : Pinhole) return Driver.Images.Mask is (Driver.Images.Create (C.Columns, C.Rows));
   --  A pinhole sees no robot.

   function Looking_At (Eye, Target : Vec3; Focal : Real; Columns, Rows : Natural; Pixel_Sigma : Real)
     return Pinhole;
   --  An eye at Eye whose optical axis passes through Target, image rows
   --  running as close to the world's -z as the axis allows.

end Driver.World.Tests;
