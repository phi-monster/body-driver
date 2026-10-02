--  The search for contact sets: where the body can meet a thing, for any
--  wanted twist.
--
--  The body's parts that can touch are measured: the faces of a closer's
--  lobes, each moving along the straight path from its open to its closed
--  end, all of a closer's lobes in the same proportion; the ends of the lobes;
--  and the arm's own measured surface. Two kinds of candidate come from one
--  rule, pinning a part to one measured sample of the thing, face against the
--  sample's normal:
--
--    a closer's face pinned to a sample leaves the rotation about that normal
--    free; the closer's other faces then close until they meet the thing, and
--    where they stop are the other touches;
--    any part pinned alone is a single touch.
--
--  Rotations are taken at the step that moves the farthest face corner by one
--  sample pitch, so no placement the samples can tell apart is skipped. The
--  hard conditions are all measured: no lobe comes down on the material, the
--  thing goes no deeper into the hand than the hand allows, no lobe passes
--  through the surface the thing lies on or through other things on the way
--  in, and both the touching pose and the pose where the last straight
--  stretch begins are reachable. Every surviving contact set goes through the
--  physical check (Driver.Action.Contact.Wrench) for the wanted twist, its
--  normals tilted Z of their sigma the worst way. The least friction under
--  which any candidate works is taken as the thing's, unless it is known to be
--  more; candidates are ranked by the force they need there. A candidate
--  needing at least the friction the thing once failed at is never taken.

with Ada.Containers.Vectors;
with Driver.Action.Snapshots;
with Driver.Uncertain;

package Driver.Action.Contact.Search is

   use Driver.Action.Snapshots;

   type Pad is record
      Closer     : Positive;        --  which of the effector's closers moves it
      Open       : Vec3 := Zero3;   --  face centre with the closer open, tool frame
      Closed     : Vec3 := Zero3;   --  the same, closed on nothing
      Facing     : Vec3 := Zero3;   --  unit: the way the face looks
      Half_Width : Real := 0.0;
      Thickness  : Real := 0.0;
   end record;

   package Pad_Vectors is new Ada.Containers.Vectors (Positive, Pad);

   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);

   type Closer_Info is record
      Hand : Hand_Id;
      Now  : Real := 0.0;   --  its fraction at this beat
   end record;

   package Closer_Vectors is new Ada.Containers.Vectors (Positive, Closer_Info);

   type Effector is record
      Arm     : Arm_Id;
      Tool    : Rigid;                   --  the tool frame at this beat
      Pads    : Pad_Vectors.Vector;
      Closers : Closer_Vectors.Vector;
      Ends    : Sample_Vectors.Vector;   --  the lobes' ends now, tool frame, facing Along
      Surface : Sample_Vectors.Vector;   --  the arm's own measured surface, tool frame
      Along   : Vec3 := Zero3;           --  unit, tool frame: from the hand toward the lobes' ends
      Depth   : Real := 0.0;             --  how far a thing may go in between the ends
      Sigma   : Real := Real'Last;       --  placement uncertainty of its touching parts
      Band    : Real := 0.0;             --  how far behind the lobes' ends a face's touch is set
      Closes  : Boolean := False;        --  some closer brings faces together
      Why_Not : Unbounded_String;        --  why it cannot close, when it cannot
   end record;
   --  A pad's points are where its face touches: Band behind the end of the
   --  lobe, the arm's resolution and Z of its placement sigma, so that the
   --  touch stays on the face wherever within its uncertainty the arm stops.

   function Effector_Of (S : Snapshot; A : Arm_Id) return Effector
     with Pre => Has_Arm (S, A);
   --  The arm with every closer it carries, from what was measured.

   type Shape is record
      Samples      : Sample_Vectors.Vector;
      Sigma        : Real := Real'Last;
      Normal_Sigma : Real := Real'Last;
      Pitch        : Real := 0.0;
      Centre       : Driver.Uncertain.Point_Estimate;
      Base         : Footing;          --  what bears it now; No_Footing when nothing does
      Floor_Point  : Vec3 := Zero3;    --  the surface it lies on, which nothing may pass through
      Floor_Up     : Vec3 := Zero3;    --  zero when it lies on no measured surface
      Floor_Sigma  : Real := 0.0;
   end record;

   function Shape_Of (S : Snapshot; T : Thing_Id) return Shape
     with Pre => Has_Thing (S, T);

   type Candidate is record
      Tool       : Rigid;               --  tool pose at the touch
      Hover      : Rigid;               --  tool pose where the last straight stretch begins
      Before     : Real_Vectors.Vector; --  each closer's fraction on the way in
      At_Touch   : Real_Vectors.Vector; --  each closer's fraction at the touch
      Touches    : Touch_Vectors.Vector;
      Mu_Nominal : Real := 0.0;         --  least friction, normals as measured
      Mu_Worst   : Real := 0.0;         --  least friction, normals tilted the worst way
      Force      : Real := Real'Last;   --  least normal force per unit weight, at the reference friction
   end record;

   type Account is record
      Placements      : Natural := 0;   --  pinned placements looked at
      Into_Material   : Natural := 0;   --  a lobe would come down on the thing
      Too_Deep        : Natural := 0;   --  the thing would reach the hand beyond its depth
      Through_Surface : Natural := 0;   --  a lobe would pass through the surface it lies on
      Through_Others  : Natural := 0;   --  a lobe would pass through another thing
      Close_On_Air    : Natural := 0;   --  the other faces would close on nothing
      Distinct        : Natural := 0;   --  different contact sets left
      Cannot_Balance  : Natural := 0;   --  no force within the cones gives the wanted motion
      Surface_In_Way  : Boolean := False;   --  the wanted motion goes into the surface it lies on
      Over_Bound      : Natural := 0;   --  needing friction the thing has failed at before
      Unreachable     : Natural := 0;   --  no reachable pose with a clear way in
      Reference_Mu    : Real := 0.0;
   end record;

   function Say (A : Account) return String;
   --  What was tried, in plain words, for a refusal.

   procedure Find
     (Thing     : Shape;
      Beside    : Point_Vectors.Vector;
      E         : Effector;
      Motion    : Twist;
      Up        : Vec3;
      Friction  : Friction_Bounds;
      Reachable : not null access function (Tool : Rigid) return Boolean;
      Best      : out Candidate;
      Found     : out Boolean;
      Tried     : out Account;
      Touch_Only : Boolean := False)
     with Pre => abs Up > 0.0;
   --  The best contact set of E on the thing for Motion. Beside are the
   --  measured surfaces of everything else near it. Touch_Only leaves out
   --  the sets that close on the thing, for a want to touch it, not hold it.

end Driver.Action.Contact.Search;
