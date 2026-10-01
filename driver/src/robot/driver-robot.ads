--  Layer 2: the robot's body, measured from its own readings, commands and
--  images.
--
--  Nothing here is declared by a user. Boot (Driver.Robot.Boot) measures the
--  body from zero, or reloads what an earlier boot measured and re-measures
--  what changed; Observe then refines the estimates every beat. Every
--  quantity carries its uncertainty.
--
--  Frames and units. The world frame is fixed to the scene; how boot anchors
--  it is documented in Driver.Robot.Boot. Lengths are in the body's own unit,
--  fixed at boot; no layer may assume metres or degrees, and nothing may
--  assume that +z is up (use Up). Camera frames have z along the optical
--  axis, x along +U and y along +V of the image (Driver.Images).
--
--  Ownership: path A owns this layer except Driver.Robot.Hand (path B).
--  Upper layers use only what this specification and Driver.Robot.Motion and
--  Driver.Robot.Hand export.

with Driver.Commands;
with Driver.Images;
with Driver.Numerics;
with Driver.Observations;
with Driver.Uncertain;

package Driver.Robot is

   use Driver.Numerics;
   use Driver.Uncertain;

   subtype Group_Id is Driver.Observations.Group_Id;
   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Observation is Driver.Observations.Observation;

   type Group_Role is (Unclassified, Arm, Carrier, Closer, Part, Sensor, Inert);
   --  Measured by pushing each commandable group (docs/body-protocol.md, 3):
   --    Arm      moving it moves the whole image of the eyes it carries, not of all
   --    Carrier  moving it moves the whole image of every eye (base, torso, drone)
   --    Closer   moving it moves a patch inside an arm's own eye (fingers, gripper)
   --    Part     moving it moves a patch outside every arm's eye
   --    Sensor   its readings change though it is not commanded
   --    Inert    nothing it is pushed to changes anything any eye sees

   type Arm_Id is new Positive;

   type Mount_Kind is (Unmeasured, World_Fixed, Arm_Carried, Carrier_Carried);

   type Mount (Kind : Mount_Kind := Unmeasured) is record
      case Kind is
         when Arm_Carried => Arm : Arm_Id;
         when others      => null;
      end case;
   end record;

   type Model is tagged limited private;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command);
   --  One beat: the observation, and Sent, the last command sent to the
   --  robot before it arrived (the one in effect while it was captured;
   --  holds included). Estimators only; never sends anything.

   function Booted (M : Model) return Boolean;

   function Role (M : Model; G : Group_Id) return Group_Role;

   function Arm_Count (M : Model) return Natural;
   function Arm_Group (M : Model; A : Arm_Id) return Group_Id
     with Pre => Natural (A) <= Arm_Count (M);

   function Eye_Count (M : Model) return Natural;
   function Eye_Mount (M : Model; E : Eye_Id) return Mount;

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate;
   --  The camera frame in the world frame at the beat of O.

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean);
   --  Where a world point appears in the eye; Visible is False when it falls
   --  behind the eye or outside the image.

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate;
   --  The line of sight through a pixel, in the world frame.

   function Up (M : Model) return Direction_Estimate;
   --  Away from gravity, in the world frame, as measured.

   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate;
   --  The last link of the arm in the world frame at the beat of O.

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask;
   --  The pixels of the eye that show the robot itself at the beat of O.

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate;
   --  The distance from a world point to the nearest surface of the body.

   function Still (M : Model) return Boolean;
   --  The one stillness judgment: at the latest beat no group and no eye
   --  changes significantly against its own measured noise.

private

   type Model is tagged limited record
      Is_Booted : Boolean := False;
   end record;

end Driver.Robot;
