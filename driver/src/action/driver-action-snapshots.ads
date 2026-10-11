--  What the action layer knows about the body and the world at one beat.
--
--  Every arm's tool pose and how finely it moves, every hand's lobes, every
--  thing's measured surface and centre, the surfaces things rest on and the
--  remembered places, each with its uncertainty. A plant fills it from the
--  lower layers' estimates (Driver.Action.Plants), a self test from its
--  simulated world; planning reads nothing else. Nothing in it is chosen, and
--  an estimate never measured stays Unknown rather than taking a default.
--
--  Frames: the tool frame of an arm is its last link (Driver.Robot.Tool_Pose);
--  a hand's lobes and an arm's own surface are given in it, so they move with
--  the arm. Lengths are in the body's own unit; no direction is up but Up.

with Ada.Containers.Vectors;
with Driver.Clock;
with Driver.Numerics;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Uncertain;
with Driver.World;

package Driver.Action.Snapshots is

   use Driver.Numerics;
   use Driver.Uncertain;

   subtype Arm_Id is Driver.Robot.Arm_Id;
   subtype Hand_Id is Driver.Robot.Hand.Hand_Id;
   subtype Thing_Id is Driver.World.Thing_Id;
   subtype Surface_Id is Driver.World.Surface_Id;
   subtype Place_Id is Driver.World.Place_Id;

   type Sample is record
      Point  : Vec3 := Zero3;
      Normal : Vec3 := Zero3;   --  unit and outward; zero where it was not measured
   end record;
   --  One measured point of a surface.

   package Sample_Vectors is new Ada.Containers.Vectors (Positive, Sample);

   type Arm_State is record
      Id          : Arm_Id;
      Tool        : Pose_Estimate;
      Step        : Estimate;   --  the smallest displacement a step of it delivers distinguishably
      Turn_Step   : Estimate;   --  the same for rotation, in radians
      Lag         : Estimate;   --  beats before a command takes effect
      Rate        : Estimate;   --  share of the remaining gap closed per beat once it does
      Surface     : Sample_Vectors.Vector;   --  its last link's measured surface, tool frame
      Carries_Eye : Boolean := False;        --  an eye rides on it
      Carries_All : Boolean := False;        --  it carries the whole body and every eye (me)
   end record;

   package Arm_Vectors is new Ada.Containers.Vectors (Positive, Arm_State);

   type Tip_Node is record
      Fraction : Real := 0.0;        --  how far its closer is closed at the setting: 0 open, 1 closed on nothing
      Tip      : Vec3 := Zero3;      --  the end of the lobe there, tool frame
      Sigma    : Real := Real'Last;  --  of the tip, the largest along any axis
   end record;

   package Node_Vectors is new Ada.Containers.Vectors (Positive, Tip_Node);

   type Lobe_State is record
      Path : Node_Vectors.Vector;
      --  Where the lobe's end is at every setting of its closer the body measured, from opened fully (fraction 0)
      --  to closed on nothing (fraction 1), by fraction; between two settings the end goes straight, as nothing
      --  of the way was measured, and the body carries what that interpolation can be wrong by in the sigma.
      --  Two nodes for a lobe whose eye saw it at its openings only. Nothing of its shape is assumed: what of it
      --  meets a thing is its end. A lobe that does not move faces the meeting point of the others.
   end record;

   function Open_Tip (L : Lobe_State) return Vec3;
   function Closed_Tip (L : Lobe_State) return Vec3;
   --  The ends of the path; Zero3 for an empty one.

   function Tip_Sigma (L : Lobe_State) return Real;
   --  The largest sigma along the path; Real'Last for an empty one.

   function Tip_At (L : Lobe_State; Fraction : Real) return Vec3;
   --  The lobe's end at that fraction: on the path between its two nodes around it; beyond an end, the end
   --  segment goes on.

   function Facing_At (L : Lobe_State; Fraction : Real) return Vec3;
   --  The unit direction the end moves in as the closer closes, at that fraction (the segment's); Zero3 where
   --  the lobe does not move.

   function Straight (Open_Tip, Closed_Tip : Vec3; Sigma : Real) return Lobe_State;
   --  A lobe measured at its openings only.

   package Lobe_Vectors is new Ada.Containers.Vectors (Positive, Lobe_State);

   type Hand_State is record
      Id       : Hand_Id;
      Arm      : Arm_Id;
      Lobes    : Lobe_Vectors.Vector;
      Depth    : Estimate;    --  how far a thing may enter between the tips before it meets the hand
      Fraction : Estimate;    --  how far it is closed now: 0 open, 1 closed on nothing
   end record;
   --  One closer: every lobe it moves goes along the straight path from its
   --  open to its closed tip, all in the same proportion.

   package Hand_Vectors is new Ada.Containers.Vectors (Positive, Hand_State);

   type Joint_Kind is (No_Joint, Revolute, Prismatic, Axis_Unknown);

   type Joint_State is record
      Kind  : Joint_Kind := No_Joint;
      Base  : Thing_Id'Base := 0;     --  what it moves against; 0 the world
      Axis  : Direction_Estimate;
      Point : Point_Estimate;         --  on a revolute axis
   end record;
   --  A thing that is a part of another one moves only along its joint;
   --  Axis_Unknown: it is attached, but its motion has not been measured yet.

   type Friction_Bounds is record
      Low  : Real := 0.0;
      High : Real := Real'Last;
   end record;
   --  What this body has learned of the thing's friction coefficient: it
   --  came along when a contact set needed Low, and failed one needing High.

   type Thing_State is record
      Id           : Thing_Id;
      Samples      : Sample_Vectors.Vector;   --  its measured surface, world frame
      Sigma        : Real := Real'Last;       --  position uncertainty of a sample
      Normal_Sigma : Real := Real'Last;       --  angular uncertainty of a sample's normal
      Pitch        : Real := 0.0;             --  spacing of the samples
      Centre       : Point_Estimate;          --  of mass: the centroid of its measured volume
      Support      : Surface_Id'Base := 0;    --  0: it rests on nothing measured
      Height       : Estimate;                --  above its support, along that surface's normal
      Seen         : Boolean := False;        --  by some eye at this beat
      Best_Eye     : Natural := 0;            --  of Eyes, the eye on no arm that shows it with the most pixels
      Moving       : Boolean := False;        --  significantly, against its own noise
      Held_By      : Hand_Id'Base := 0;
      Joint        : Joint_State;
      Friction     : Friction_Bounds;
   end record;

   package Thing_Vectors is new Ada.Containers.Vectors (Positive, Thing_State);

   type Surface_State is record
      Id       : Surface_Id;
      Point    : Point_Estimate;
      Normal   : Direction_Estimate;   --  away from the material, toward what rests on it
      Of_Thing : Thing_Id'Base := 0;   --  the thing whose top it is; 0 when it belongs to no thing
   end record;

   package Surface_Vectors is new Ada.Containers.Vectors (Positive, Surface_State);

   type Place_State is record
      Id    : Place_Id;
      Point : Point_Estimate;
   end record;

   package Place_Vectors is new Ada.Containers.Vectors (Positive, Place_State);

   type Eye_State is record
      Pose      : Pose_Estimate;
      On_Arm    : Arm_Id'Base := 0;   --  0: it rides on no arm
   end record;
   --  Camera frame: z along the optical axis, x along the image's columns.

   package Eye_Vectors is new Ada.Containers.Vectors (Positive, Eye_State);

   type Snapshot is record
      Beat     : Driver.Clock.Beat := 0;
      Up       : Direction_Estimate;   --  away from gravity
      Still    : Boolean := False;     --  no group, no eye and no thing changes against its own noise
      Arms     : Arm_Vectors.Vector;
      Hands    : Hand_Vectors.Vector;
      Things   : Thing_Vectors.Vector;
      Surfaces : Surface_Vectors.Vector;
      Places   : Place_Vectors.Vector;
      Eyes     : Eye_Vectors.Vector;
   end record;

   function Has_Thing (S : Snapshot; T : Thing_Id) return Boolean;
   function Thing (S : Snapshot; T : Thing_Id) return Thing_State
     with Pre => Has_Thing (S, T);

   function Has_Surface (S : Snapshot; Id : Surface_Id) return Boolean;
   function Surface (S : Snapshot; Id : Surface_Id) return Surface_State
     with Pre => Has_Surface (S, Id);

   function Has_Arm (S : Snapshot; A : Arm_Id) return Boolean;
   function Arm (S : Snapshot; A : Arm_Id) return Arm_State
     with Pre => Has_Arm (S, A);

   function Has_Hand (S : Snapshot; H : Hand_Id) return Boolean;
   function Hand (S : Snapshot; H : Hand_Id) return Hand_State
     with Pre => Has_Hand (S, H);

   function Has_Place (S : Snapshot; P : Place_Id) return Boolean;
   function Place (S : Snapshot; P : Place_Id) return Place_State
     with Pre => Has_Place (S, P);

   function Tool_Point (A : Arm_State; P : Vec3) return Vec3 is (A.Tool.Pose * P);
   --  A point given in the arm's tool frame, in the world frame now.

end Driver.Action.Snapshots;
