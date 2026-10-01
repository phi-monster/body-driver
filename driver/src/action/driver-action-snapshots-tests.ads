--  Fixtures for the action layer's self test: things built from boxes,
--  cylinders and tubes, their surfaces sampled the way an eye measures them
--  (no samples on faces resting on the support or inside another part), and
--  the hands the tests act with.
--
--  Everything a test builds is placed by a rigid pose, so a test can turn the
--  whole world and expect the same behaviour: nothing in the driver may
--  depend on which way the world frame points.

with Ada.Containers.Vectors;

package Driver.Action.Snapshots.Tests is

   type Part_Kind is (Block, Cylinder, Tube);

   type Part is record
      Kind  : Part_Kind;
      Pose  : Rigid;     --  in the thing's own frame; a cylinder's axis is its z
      Sizes : Vec3;
   end record;
   --  Sizes: a Block's half sizes; a Cylinder's radius, unused, half height;
   --  a Tube's outer radius, inner radius, half height (open at +z).

   package Part_Vectors is new Ada.Containers.Vectors (Positive, Part);

   type Model is record
      Parts : Part_Vectors.Vector;
   end record;
   --  A thing in its own frame, resting on the plane z = 0 of that frame.

   function Bar (Length, Width, Height : Real) return Model;
   function Block (X, Y, Z : Real) return Model;
   function Upright_Cylinder (Radius, Height : Real) return Model;
   function Scissors (Length, Blade_Width, Thickness : Real) return Model;
   --  Two thin blades crossing at the pivot and two rings for handles, flat.
   function Cup (Radius, Wall, Height : Real) return Model;
   --  A tube closed at the bottom with a handle on its side.

   function Inside (M : Model; P : Vec3) return Boolean;
   --  P, in the thing's frame, is within the material.

   function Centre (M : Model) return Vec3;
   --  The centroid of its volume, in its own frame.

   function Thing_Of
     (Id      : Thing_Id;
      M       : Model;
      Place   : Rigid;           --  the thing's frame in the world
      Pitch   : Real;
      Sigma   : Real;            --  position sigma of every sample and of the centre
      Support : Surface_Id'Base) return Thing_State;
   --  Its measured surface: every face sampled at Pitch with its outward
   --  normal, except where it rests on its frame's plane z = 0 or lies inside
   --  another part.

   function Floor (Id : Surface_Id; Place : Rigid; Sigma : Real) return Surface_State;
   --  The plane z = 0 of Place, normal +z of Place.

   function Gripper (Arm : Arm_Id; Hand : Hand_Id; Opening, Width, Thickness, Depth, Sigma : Real)
     return Hand_State;
   --  Two lobes closing toward each other along the tool's x, their ends
   --  Depth beyond the tool origin along the tool's z.

   function Five_Lobes (Arm : Arm_Id; Hand : Hand_Id; Radius, Width, Thickness, Depth, Sigma : Real)
     return Hand_State;
   --  Five lobes on a circle about the tool's z, closing toward its axis.

   function Arm_Of (Id : Arm_Id; Tool : Rigid; Sigma : Real) return Arm_State;
   --  An arm whose tool pose is known to Sigma, stepping by a tenth of it.

   function Plate_Arm (Id : Arm_Id; Tool : Rigid; Radius, Pitch, Sigma : Real) return Arm_State;
   --  An arm with no lobes: a flat disc of Radius ends it, facing the tool's z.

end Driver.Action.Snapshots.Tests;
