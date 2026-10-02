--  An eye at one beat, as the world's geometry needs it: the line of sight
--  through a pixel and where a point appears, both with the eye's measured
--  pose and lens at that beat.
--
--  The world's estimators take any Camera, so they run the same on the
--  body's measured eyes (Of_Body) and on the pinhole cameras of their self
--  tests; nothing here measures anything.

with Driver.Images;
with Driver.Robot;

package Driver.World.Cameras is

   subtype Pixel is Driver.Images.Pixel;

   type Camera is interface;

   function Width (C : Camera) return Natural is abstract;
   function Height (C : Camera) return Natural is abstract;

   function Ray (C : Camera; Px : Pixel) return Ray_Estimate is abstract;
   --  World frame; unknown when the eye is not measured.

   procedure Project (C : Camera; Point : Vec3; Px : out Pixel; Visible : out Boolean) is abstract;
   --  Visible is False behind the eye, outside the image, or when the eye is
   --  not measured.

   function Pose (C : Camera) return Pose_Estimate is abstract;
   --  The eye's frame in the world.

   function Radians_Per_Pixel (C : Camera'Class; Px : Pixel) return Real;
   --  How far the line of sight turns from one pixel to the next there,
   --  measured on the camera itself: what an image position's uncertainty in
   --  pixels is as an angle. Real'Last when the eye is not measured.

   --  An eye of the measured body at the beat of an observation; both must
   --  outlive it.
   type Of_Body (Robot : not null access constant Driver.Robot.Model;
                 Seen  : not null access constant Observation) is new Camera with record
      Eye : Eye_Id;
   end record;

   overriding function Width (C : Of_Body) return Natural;
   overriding function Height (C : Of_Body) return Natural;
   overriding function Ray (C : Of_Body; Px : Pixel) return Ray_Estimate;
   overriding procedure Project (C : Of_Body; Point : Vec3; Px : out Pixel; Visible : out Boolean);
   overriding function Pose (C : Of_Body) return Pose_Estimate;

end Driver.World.Cameras;
