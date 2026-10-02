--  A simulated body and world for the action layer's self test.
--
--  Arms take a command Lag beats after it is sent and close a Rate share of
--  what is left every beat, toward a share of the commanded displacement
--  drawn for each command between Delivery_Low and Delivery_High; they reach
--  no farther than Reach from their base, turn the wrist no more than Wrist
--  about the tool's z, and stop where their moving parts (lobes, palm, plate
--  and whatever they hold) meet the table or a thing that cannot give way.
--  Closers close their lobes until both meet a thing; a thing between closed
--  lobes is held, and moves with the hand while the true friction lets the
--  grip carry it (Driver.Action.Contact.Wrench with the true contacts: the
--  same quasi-static law the planner uses, applied to the truth). Things
--  rest on the table or on each other, are pushed along by what presses into
--  them, fall onto what is under them when let go and slide off what cannot
--  bear them, may move only along a hinge or a rail, or move on their own.
--  Look gives everything with noise; the action layer sees nothing else.

with Ada.Containers.Vectors;
with Ada.Numerics.Float_Random;
with Driver.Action.Snapshots.Tests;

package Driver.Action.Plants.Tests is

   use Driver.Action.Snapshots.Tests;

   type Joint_Kind is (Loose, Hinge, Rail);

   type Sim_Joint is record
      Kind  : Joint_Kind := Loose;
      Axis  : Vec3 := Zero3;     --  unit, world frame
      Point : Vec3 := Zero3;     --  on a hinge's axis
      Low   : Real := 0.0;       --  the range of Q
      High  : Real := 0.0;
      Q     : Real := 0.0;       --  radians about a hinge, length along a rail
      Rest  : Rigid := Identity; --  the thing's pose at Q = 0
   end record;

   type Sim_Thing is record
      Id       : Thing_Id := 1;
      Shape    : Model;
      Pose     : Rigid := Identity;
      Fixed    : Boolean := False;   --  nothing moves it: a wall, a post, a frame
      Mu       : Real := 0.5;        --  its true friction against anything
      Joint    : Sim_Joint;
      Drift    : Vec3 := Zero3;      --  a thing that moves on its own, this much every beat
      Held_By  : Hand_Id'Base := 0;
      Grip     : Rigid := Identity;  --  its pose in the holding tool's frame
      Moved_Q  : Real := 0.0;        --  how far it has moved along its joint, as the eyes saw it
      Points   : Sample_Vectors.Vector;   --  its whole surface, in its own frame
      Bound    : Real := 0.0;        --  no point of it is farther than this from its centre
   end record;

   package Thing_Vectors is new Ada.Containers.Vectors (Positive, Sim_Thing);

   type Command is record
      Due  : Natural := 0;           --  the beat it takes effect
      Goal : Rigid := Identity;
   end record;

   package Command_Vectors is new Ada.Containers.Vectors (Positive, Command);

   type Sim_Arm is record
      Id            : Arm_Id := 1;
      Base          : Vec3 := Zero3;
      Reach         : Real := 0.0;
      Neutral       : Mat3 := Identity3;   --  tool orientation in the middle of the wrist's range
      Wrist         : Real := 0.0;         --  about the tool's z, either way from Neutral
      Tilt          : Real := 0.0;         --  of the tool's z away from Neutral's z
      Tool          : Rigid := Identity;
      Lag           : Natural := 0;
      Rate          : Real := 1.0;
      Delivery_Low  : Real := 1.0;
      Delivery_High : Real := 1.0;
      Plate_Radius  : Real := 0.0;         --  a lobeless arm ends in a disc of this radius facing the tool's z
      Pending       : Command_Vectors.Vector;
      Start         : Rigid := Identity;
      Target        : Rigid := Identity;
      Commanded     : Rigid := Identity;
      Active        : Boolean := False;
      Blocked       : Boolean := False;
   end record;

   package Arm_Vectors is new Ada.Containers.Vectors (Positive, Sim_Arm);

   type Sim_Hand is record
      Id        : Hand_Id := 1;
      Arm       : Arm_Id := 1;
      Opening   : Real := 0.0;   --  between the faces, open
      Width     : Real := 0.0;
      Thickness : Real := 0.0;
      Depth     : Real := 0.0;   --  from the palm to the tips, along the tool's z
      Fraction  : Real := 0.0;
      Goal      : Real := 0.0;
      Due       : Natural := 0;
      Next_Goal : Real := 0.0;
      Rate      : Real := 1.0;
      Stopped   : Boolean := False;
   end record;

   package Hand_Vectors is new Ada.Containers.Vectors (Positive, Sim_Hand);

   type Learned_Bound is record
      Thing  : Thing_Id := 1;
      Bounds : Friction_Bounds;
   end record;

   package Bound_Vectors is new Ada.Containers.Vectors (Positive, Learned_Bound);

   type World is limited new Plant with record
      Up        : Vec3 := [0.0, 0.0, 1.0];
      Table     : Rigid := Identity;     --  its plane z = 0 is the table, normal along Up
      Things    : Thing_Vectors.Vector;
      Arms      : Arm_Vectors.Vector;
      Hands     : Hand_Vectors.Vector;
      Eye       : Rigid := Identity;     --  a still eye
      Beat      : Natural := 0;
      Last_Beat : Natural := Natural'Last;
      Sigma     : Real := 0.0;           --  of every measured position
      Pitch     : Real := 0.0;           --  of the measured surfaces
      Noise     : Ada.Numerics.Float_Random.Generator;
      Learned   : Bound_Vectors.Vector;
      Moved     : Boolean := False;      --  something moved in the last beat
   end record;

   procedure Start (W : in out World; Place : Rigid; Sigma, Pitch : Real; Seed : Integer);
   --  An empty world turned by Place: the table is Place's plane z = 0,
   --  gravity along Place's z, a still eye in front of the table.

   procedure Add_Arm (W : in out World; Base : Vec3; Reach : Real; Tool : Rigid; Lag : Natural; Rate : Real;
                      Delivery_Low, Delivery_High : Real; Wrist, Tilt : Real; Plate_Radius : Real := 0.0);
   --  In the table's frame.

   procedure Add_Gripper (W : in out World; Arm : Arm_Id; Opening, Width, Thickness, Depth : Real);

   procedure Add_Thing (W : in out World; M : Model; Place : Rigid; Mu : Real; Fixed : Boolean := False);
   --  Place in the table's frame; it is let down onto whatever is under it.

   procedure Set_Joint (W : in out World; T : Thing_Id; J : Sim_Joint);
   procedure Set_Drift (W : in out World; T : Thing_Id; Drift : Vec3);

   function Truth (W : World; T : Thing_Id) return Sim_Thing;
   function Lowest (W : World; T : Thing_Id) return Real;
   --  Its lowest point above the table, along Up.
   function Rests_On (W : World; T, Other : Thing_Id) return Boolean;
   --  It lies on the other's top and would stay there.
   function Table_Frame (W : World) return Rigid is (W.Table);

   overriding procedure Look (W : in out World; S : out Snapshot);
   overriding function Reach (W : World; Goal : Arm_Goal) return Reach_Answer;
   overriding procedure Move (W : in out World; O : Order; R : out Report);
   overriding function Predicted (W : World; T : Thing_Id; Beats : Natural) return Driver.Uncertain.Point_Estimate;
   overriding procedure Learn (W : in out World; L : Lesson);
   overriding function Episode_Over (W : World) return Boolean;
   overriding function In_View (W : World; Point : Vec3) return Boolean;
   --  Within the still eye's cone of view.

end Driver.Action.Plants.Tests;
