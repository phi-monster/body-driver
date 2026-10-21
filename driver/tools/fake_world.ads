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
      Held_By  : Natural := 0;      --  a thing of the world: the arm whose fingers pinch it, 0 when none does
      Held_Rel : Rigid := Identity; --  where it stands in that arm's tool frame while held
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
      Rate       : Real := 0.0;              --  the most the position changes in a beat, radians or metres; 0 is unlimited
      Droop_Gain : Real := 0.0;              --  radians of sag per metre of horizontal lever, revolute joints
      Dead       : Real := 0.0;              --  the servo's tolerance, reading units: an ask within it moves nothing, and a
                                             --  move lands short of its target by up to it (Landing), as a servo bus does
      Landing    : Real := 0.0;              --  how far short of the target in effect this move lands, drawn with the target
      Mirror     : Natural := 0;             --  a link whose position this one copies (the other finger)
      Pose       : Rigid := Identity;        --  the link's frame in the world, from Forward
      Moved_Beat : Natural := 0;             --  the last beat the position changed
      Touching   : Boolean := False;         --  a solid of this link rests on something
      Bounce     : Real := 0.0;              --  the easing-back offset a let-go left, reading units, applied in Q
      Bounce_Age : Natural := Natural'Last;  --  beats since the let-go; 'Last when none is going on
      Play       : Real := 0.0;              --  the slack of its gears, reading units: the link turns this much without the
                                             --  servo (whose reading the robot reports) turning, and gravity takes it up
      Play_Offset : Real := 0.0;             --  how far the link stands from its reading now, taken up by gravity
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
      Integers    : Boolean := False;      --  read in whole units and sent as integers (a servo bus's ticks)
      Echoed      : Real_Vectors.Vector;   --  the last command received, in reading units
      Reported    : Real_Vectors.Vector;   --  this beat's readings, noise and all, drawn once a beat (Step)
      Silent      : Boolean := False;      --  its servos do not answer: not reported, commands not taken (a dead bus)
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
      Image_Noise   : Real := 0.7;       --  the sensor's noise on every picture, in grey levels: no picture at rest repeats exactly
      Zero_Noise    : Boolean := True;   --  readings at rest repeat exactly; after a push they jitter in the last bits
      Drift         : Boolean := True;   --  readings creep without a command, far below anything visible
      Delay_Beats   : Natural := 1;      --  beats before a command takes effect
      Droop         : Boolean := True;   --  a joint under load settles short of its target
      Dead          : Boolean := True;   --  a servo ignores an ask within its tolerance and lands short by up to it
      Creep         : Boolean := True;   --  a hand on the table never quite stands still
      Give          : Boolean := True;   --  an arm pushed against a surface goes a little way in
      Soft_Closer   : Boolean := True;   --  an open closer pressed on a surface is pressed shut
      Bounce        : Boolean := True;   --  an arm let go from a press eases back and shakes
      Play          : Boolean := True;   --  each joint's gear slack, taken up by gravity, unseen by its reading
      Reading_Noise : Real := 0.0;       --  a real robot's reading noise, in reading units; the simulator's is 0
   end record;

   --  What an arm ends in. Drawn: the seed decides.
   type Hand_Kind is (Drawn, Pair, Hook, Pair_And_Thumb, Fingers, Jaw);
   --  Pair            two plates sliding apart on one channel, 1 open and 0 closed (the lab's gripper)
   --  Hook            one prong slanting down from the palm: it presses and hooks, and nothing closes
   --  Pair_And_Thumb  the pair, and a plate that comes down on them on a second channel of the same group
   --  Fingers         five fingers side by side that curl down, each read as a group of its own
   --  Jaw             one jaw swung onto a jaw that does not move, on one channel (a desktop arm's gripper)

   type Hand_Kinds is array (1 .. 2) of Hand_Kind;

   package Kind_Vectors is new Ada.Containers.Vectors (Positive, Hand_Kind);

   --  What kind of body is drawn: two arms on a table, each with its eye on its wrist (the lab's, the kiwi's); a
   --  humanoid standing at the table, every joint of its body in one group, an eye in its head and on some of its
   --  wrists; a wheeled base that carries an arm and a mast with an eye; a drone, a body that its motors move
   --  through space with an eye on it. Every kind but the first has its limbs drawn: how many joints, about which
   --  of its axes each turns, how long each link is.
   type Body_Class is (Table_Arms, Humanoid, Base_And_Arm, Drone);

   --  What a test forces on the drawn body; the rest the seed decides.
   type Template is record
      Class       : Body_Class := Table_Arms;
      Hands       : Hand_Kinds := [others => Drawn];
      Eyeless     : Natural := 0;        --  the arm with no eye of its own; 0 for none
      Eye_Up      : Natural := 0;        --  the arm whose eye looks ahead at the wall, not down at the table; 0 for none
      Plain_Table : Boolean := False;    --  a table with no texture at all
      White_Room  : Boolean := False;    --  a white table and white walls, faint broad veins their only texture
      Dead_Arm    : Natural := 0;        --  the arm whose servos never answer (it stands at rest, seen); 0 for none
      Ticks       : Natural := 0;        --  the ticks a turn of every arm whose unit is a servo bus's; 0: drawn (1024 to 16384)
      Servo       : Boolean := False;    --  every arm reads in a servo bus's ticks, whatever its unit was drawn
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
      Kinds   : Kind_Vectors.Vector;    --  per arm, what it ends in
      Hand_Reach : Real_Vectors.Vector; --  per arm, how far from its tool its hand's parts reach, over the table
      Links   : Link_Vectors.Vector;
      Groups  : Group_Vectors.Vector;
      Cameras : Camera_Vectors.Vector;
      Solids  : Solid_Vectors.Vector;
      Quirk   : Quirks;
      Beat    : Natural := 0;
      Pending : Due_Vectors.Vector;   --  targets the delay has not let through yet
   end record;

   procedure Build (W : in out World; Seed : Natural; Width, Height : Positive; T : Template);
   --  Draws the body and the world of Seed, with cameras of Width x Height,
   --  what T forces forced, and puts the body at its rest pose. The hands are
   --  drawn from a generator of their own, so that forcing them changes
   --  nothing else of the body.

   procedure Forward (W : in out World);
   --  Every link's, solid's and camera's pose in the world from the joints' positions.

   procedure Command (W : in out World; G : Positive; Values : Real_Array);
   --  The driver's target for group G, in reading units, to take effect after the delay.

   procedure Step (W : in out World);
   --  One beat of the body's motion.

   function Readings (W : World; G : Positive) return Real_Array;
   --  What group G reports this beat, in reading units: the same values however often they are asked
   --  (the truth beside the run repeats them, and the scorer pairs the two by them).

   function Echo (W : World; G : Positive) return Real_Array;
   --  The last command the robot received for group G, in reading units.

   function Random (W : in out World) return Real;
   --  Uniform in [0, 1).

   function Random (W : in out World; Low, High : Real) return Real;
   --  Uniform in [Low, High).

   function Describe (W : World) return String;
   --  The drawn body in words, for the run's notes: what the driver should find.

end Fake_World;
