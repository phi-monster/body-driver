--  A body and a world drawn at random, moved by the driver's commands, with
--  everything the lab's simulator was seen to do to a driver (the quirks),
--  each one switchable.
--
--  The body is a tree of links. Each link hangs from its parent by a joint
--  (fixed, revolute or prismatic) at an origin pose and moves along or about
--  the joint's axis by its position; what the robot reports of a joint is
--  that position in the robot's own reading unit, laid out in whatever groups
--  the body happens to have. Cameras ride on links. Solids (boxes and planes)
--  make the links' and the world's surfaces, for the renderer and for the
--  contacts.
--
--  Step takes one beat: the targets the driver asked take effect after the
--  response delay, each joint follows its target as a first-order lag and
--  stops at its own end, an arm whose hand meets the table or a thing stops
--  where it met it (with the small give the lab's robot has), and the quirks
--  add what they add. Nothing here reaches the driver but through the
--  readings and the pictures.

with Ada.Containers.Vectors;
with Ada.Numerics.Float_Random;
with Ada.Strings.Unbounded;
with Driver;
with Driver.Numerics;

package Fake_World is

   use Driver;
   use Driver.Numerics;
   use Ada.Strings.Unbounded;

   type Texture_Kind is (Flat, Noise, Stripes, Grain);

   type Material is record
      Kind     : Texture_Kind := Flat;
      Albedo   : Vec3 := [0.5, 0.5, 0.5];   --  reflectance per channel, 0 to 1
      Contrast : Real := 0.0;               --  how much the texture modulates the albedo, 0 to 1
      Scale    : Real := 1.0;               --  texture cycles per metre
      Gloss    : Real := 0.0;               --  specular share, 0 to 1
   end record;

   type Shape_Kind is (Box, Plane);

   --  A box of half sizes Half in its own frame, or the plane z = 0 of its
   --  own frame (facing +z, infinite). A static solid has Link 0 and its Pose
   --  in the world; a link's solid has Local, its pose in the link's frame,
   --  and its world Pose follows the link (Forward).
   type Solid is record
      Name   : Unbounded_String;
      Kind   : Shape_Kind := Box;
      Half   : Vec3 := [0.1, 0.1, 0.1];
      Local  : Rigid := Identity;
      Pose   : Rigid := Identity;
      Mat    : Material;
      Link   : Natural := 0;
      Arm    : Natural := 0;        --  the arm whose contacts it takes part in, 0 for the world's
      Finger : Boolean := False;    --  a finger: what presses, and what a surface presses shut
   end record;

   package Solid_Vectors is new Ada.Containers.Vectors (Positive, Solid);

   type Joint_Kind is (Fixed, Revolute, Prismatic);

   type Link is record
      Name       : Unbounded_String;
      Parent     : Natural := 0;             --  0 is the world
      Arm        : Natural := 0;             --  the arm it belongs to, 0 for none
      Kind       : Joint_Kind := Fixed;
      Origin     : Rigid := Identity;        --  the joint's frame in the parent's, at position zero
      Axis       : Vec3 := [0.0, 0.0, 1.0];  --  in the joint's frame
      Unit       : Real := 1.0;              --  radians or metres per reading unit
      Q          : Real := 0.0;              --  the position now, in reading units
      Target     : Real := 0.0;              --  the target in effect, in reading units
      Lo, Hi     : Real := 0.0;              --  the joint's own ends, in reading units
      Tau        : Real := 1.0;              --  beats of the first-order response
      Droop_Gain : Real := 0.0;              --  radians of sag per metre of horizontal lever, revolute joints
      Mirror     : Natural := 0;             --  a link whose position this one copies (the other finger)
      Pose       : Rigid := Identity;        --  the link's frame in the world, from Forward
      Moved_Beat : Natural := 0;             --  the last beat the position changed
      Touching   : Boolean := False;         --  a solid of this link rests on something
      Bounce     : Real := 0.0;              --  the easing-back offset a let-go left, reading units, applied in Q
      Bounce_Age : Natural := Natural'Last;  --  beats since the let-go; 'Last when none is going on
   end record;

   package Link_Vectors is new Ada.Containers.Vectors (Positive, Link);

   package Index_Vectors is new Ada.Containers.Vectors (Positive, Natural);

   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);

   --  A group of readings as the robot lays them out: channel i reports the
   --  link Joints (i)'s position, or the constant Values (i) when the group
   --  has no joints. Scalar groups are reported as a number, not a list.
   type Group is record
      Name        : Unbounded_String;
      Joints      : Index_Vectors.Vector;
      Values      : Real_Vectors.Vector;
      Commandable : Boolean := False;
      Scalar      : Boolean := False;
      Echoed      : Real_Vectors.Vector;   --  the last command received, in reading units
   end record;

   package Group_Vectors is new Ada.Containers.Vectors (Positive, Group);

   type Real_Access is access Real_Array;

   type Camera is record
      Name           : Unbounded_String;
      Link           : Natural := 0;           --  0 is the world (a fixed eye)
      Arm            : Natural := 0;
      Mount          : Rigid := Identity;      --  the optical frame in the link's frame (x right, y down, z forward)
      Width, Height  : Positive := 320;
      Fx, Fy, Cx, Cy : Real := 0.0;
      Pose           : Rigid := Identity;      --  the optical frame in the world, from Forward
      Shown          : Real_Access;            --  the picture as the renderer's accumulation shows it, RGB per pixel
      Fresh          : Boolean := True;        --  Shown holds nothing yet
   end record;

   package Camera_Vectors is new Ada.Containers.Vectors (Positive, Camera);

   --  Every quirk the lab's simulator was seen to have, on by default.
   type Quirks is record
      Image_Lag     : Boolean := True;   --  a picture shows the body as it stood the beat before
      Settle        : Boolean := True;   --  a picture accumulates over beats after a change of view
      Settle_Share  : Real := 0.25;      --  the share of a new frame that enters the accumulation each beat
      Zero_Noise    : Boolean := True;   --  readings at rest repeat exactly; after a push they jitter in the last bits
      Drift         : Boolean := True;   --  readings creep without a command, far below anything visible
      Delay_Beats   : Natural := 1;      --  beats before a command takes effect
      Droop         : Boolean := True;   --  a joint under load settles short of its target
      Creep         : Boolean := True;   --  a hand on the table never quite stands still
      Give          : Boolean := True;   --  an arm pushed against a surface goes a little way in
      Soft_Closer   : Boolean := True;   --  an open closer pressed on a surface is pressed shut
      Bounce        : Boolean := True;   --  an arm let go from a press eases back and shakes
      Reading_Noise : Real := 0.0;       --  a real robot's reading noise, in reading units; the simulator's is 0
   end record;

   type Due_Target is record
      Due   : Natural := 0;
      Joint : Positive := 1;
      Value : Real := 0.0;
   end record;

   package Due_Vectors is new Ada.Containers.Vectors (Positive, Due_Target);

   type World is record
      Rng     : Ada.Numerics.Float_Random.Generator;
      Seed    : Natural := 0;
      Table_Z : Real := 0.75;
      Arms    : Natural := 0;
      Tools   : Index_Vectors.Vector;   --  per arm, the link the hand hangs from
      Closers : Index_Vectors.Vector;   --  per arm, the link whose joint closes the hand (0 for none)
      Links   : Link_Vectors.Vector;
      Groups  : Group_Vectors.Vector;
      Cameras : Camera_Vectors.Vector;
      Solids  : Solid_Vectors.Vector;
      Quirk   : Quirks;
      Beat    : Natural := 0;
      Pending : Due_Vectors.Vector;   --  targets the delay has not let through yet
   end record;

   procedure Build (W : in out World; Seed : Natural; Width, Height : Positive);
   --  Draws the body and the world of Seed, with cameras of Width x Height,
   --  and puts the body at its rest pose.

   procedure Forward (W : in out World);
   --  Every link's, solid's and camera's pose in the world from the joints' positions.

   procedure Command (W : in out World; G : Positive; Values : Real_Array);
   --  The driver's target for group G, in reading units, to take effect after the delay.

   procedure Step (W : in out World);
   --  One beat of the body's motion.

   function Readings (W : in out World; G : Positive) return Real_Array;
   --  What group G reports this beat, in reading units (the quirks' noise is drawn here).

   function Echo (W : World; G : Positive) return Real_Array;
   --  The last command the robot received for group G, in reading units.

   function Random (W : in out World) return Real;
   --  Uniform in [0, 1).

   function Random (W : in out World; Low, High : Real) return Real;
   --  Uniform in [Low, High).

   function Describe (W : World) return String;
   --  The drawn body in words, for the run's notes: what the driver should find.

end Fake_World;
