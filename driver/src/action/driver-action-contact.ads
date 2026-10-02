--  The contact set: where the body touches a thing, which way it may press
--  there, how the thing is to move, and the surface the thing lies on.
--
--  A contact set says nothing about which part of the body makes it or what
--  the task is: a suction cup is one touch that can also transmit tension, two
--  pads are two touches, the tip of a tool in the hand is one more touch. What
--  differs between tasks is only the wanted twist of the thing. Every length
--  is in the body's own unit; no direction is assumed to be up.

with Ada.Containers.Vectors;
with Driver.Numerics;

package Driver.Action.Contact is

   use Driver.Numerics;
   use Driver.Numerics.Arrays;

   type Twist is record
      Linear  : Vec3 := Zero3;   --  translation
      Angular : Vec3 := Zero3;   --  rotation vector: unit axis times angle in radians
      Pivot   : Vec3 := Zero3;   --  a point on the rotation axis
   end record;
   --  A rigid motion of the thing: a rotation about the axis through Pivot,
   --  followed by a translation. Only its direction matters to the physics;
   --  how far to go is decided step by step by the execution.

   function Still (Pivot : Vec3) return Twist;
   function Slide (Linear : Vec3) return Twist;
   function Rotation (Axis : Vec3; Angle : Real; Pivot : Vec3) return Twist
     with Pre => abs Axis > 0.0;

   function Moves (T : Twist) return Boolean;
   --  Not the identity.

   function Velocity (T : Twist; P : Vec3) return Vec3;
   --  The first-order velocity of a point of the thing: Linear + Angular x (P - Pivot).

   function Apply (T : Twist; P : Vec3) return Vec3;
   --  Where the point ends up after the whole motion.

   function Scaled (T : Twist; Factor : Real) return Twist;
   --  The same screw, Factor times as far.

   procedure Plane_Basis (Normal : Vec3; E1, E2 : out Vec3)
     with Pre => abs Normal > 0.0;
   --  Two unit directions that with Unit (Normal) form a right-handed
   --  orthonormal frame. Which ones they are carries no meaning.

   type Touch is record
      Point   : Vec3 := Zero3;      --  on the thing's surface
      Inward  : Vec3 := Zero3;      --  unit: the direction the body presses into the thing
      Patch   : Real := 0.0;        --  radius of the touching patch, for friction about Inward
      Tension : Boolean := False;   --  it can also draw the surface outward
   end record;

   package Touch_Vectors is new Ada.Containers.Vectors (Positive, Touch);

   package Point_Vectors is new Ada.Containers.Vectors (Positive, Vec3);

   type Footing is record
      Present : Boolean := False;
      Up      : Vec3 := Zero3;   --  unit normal of the surface, toward the thing
      Foot    : Point_Vectors.Vector;   --  the thing's points on the surface
      Pitch   : Real := 0.0;     --  their spacing: closer than this cannot be told apart
   end record;
   --  The surface a thing lies on, as one more touch of the contact set.

   No_Footing : constant Footing;

   function Footing_Of (Points : Point_Vectors.Vector; On, Up : Vec3; Pitch : Real) return Footing;
   --  The points lying within one Pitch of the plane through On with normal
   --  Up, projected onto it and thinned to one per Pitch-sized cell. Empty
   --  input, a zero Up or a non-positive Pitch give No_Footing.

   function Base_Of (Points : Point_Vectors.Vector; Up : Vec3; Pitch : Real) return Footing;
   --  The footing the thing would stand on if put down: its lowest layer of
   --  points along Up, one Pitch thick.

private

   No_Footing : constant Footing := (Present => False, Up => Zero3, Foot => Point_Vectors.Empty_Vector, Pitch => 0.0);

end Driver.Action.Contact;
