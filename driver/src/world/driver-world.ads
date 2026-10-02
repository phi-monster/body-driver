--  Layer 3: the things around the robot, the surfaces they rest on, the
--  places it remembers, and how all of that moves.
--
--  Each thing has one estimate, shared by every eye and every beat: the
--  region it covers in each eye that sees it, its surface points, measured
--  where two eyes see the same point at the same instant, its centre, the
--  surface it rests on and its height above it along the measured Up, its
--  motion and where it will be, and what holds it. A thing is made from a
--  region an eye points at (Adopt); a region of the same pixels as a thing
--  already there is that thing. Names are not kept here: the brain layer
--  binds the brain's names to thing identities.
--
--  Surfaces are the planes things rest on, fitted to scene points that two
--  eyes see; places are points remembered by the brain's request. A new
--  episode forgets the things and places; surfaces stay, marked as earlier,
--  since a table does not move between episodes but must be seen again to
--  be trusted.
--
--  Everything comes from the recorded stream: images, the body's measured
--  geometry and the instrument's replies (Driver.Instrument, asked by the
--  estimators and read on later beats). Until a quantity is measured it is
--  reported unknown.
--
--  Ownership: path B.

with Driver.Commands;
with Driver.Geometry;
with Driver.Images;
with Driver.Numerics;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Uncertain;

private with Ada.Finalization;

package Driver.World is

   use Driver.Numerics;
   use Driver.Uncertain;

   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Observation is Driver.Observations.Observation;

   type Thing_Id is new Positive;
   type Surface_Id is new Positive;
   type Place_Id is new Positive;

   type Scene is tagged limited private;

   procedure Observe
     (S    : in out Scene;
      M    : Driver.Robot.Model;
      H    : Driver.Robot.Hand.Hands;
      O    : Observation;
      Sent : Driver.Commands.Command);
   --  Estimators only, one beat; Sent as in Driver.Robot.Observe.

   procedure New_Episode (S : in out Scene);
   --  Forgets the episode's things and places; surfaces measured before are
   --  kept but marked as belonging to the previous episode.

   function Thing_Count (S : Scene) return Natural;

   procedure Adopt
     (S      : in out Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask;
      Thing  : out Thing_Id);
   --  Makes a region of one eye a thing: the existing thing that occupies the
   --  same pixels, or a new one.

   function Seen_In (S : Scene; T : Thing_Id; E : Eye_Id) return Boolean;
   --  Seen by that eye at the latest beat.

   function Region_In (S : Scene; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask
     with Pre => Seen_In (S, T, E);

   function Centre (S : Scene; T : Thing_Id) return Point_Estimate;
   --  The middle of what the eyes see of it, on its seen surface. Its
   --  covariance is not that middle's own small uncertainty but how far the
   --  seen points, and the space under them down to its support, lie from
   --  it: a solid seen from one side has its own middle behind and under what
   --  is seen, somewhere within that.

   function Resting_On (S : Scene; T : Thing_Id) return Surface_Id'Base;
   --  The surface under it, or 0 when none is measured: the highest one its
   --  lowest point seen is over and not below.

   function Height_Above_Support (S : Scene; T : Thing_Id) return Estimate;
   --  Of its lowest point seen, along Up, from the surface it rests on;
   --  unknown when it rests on none.

   function Bottom_Seen (S : Scene; T : Thing_Id) return Boolean;
   --  Its lowest point seen is on the surface it rests on, within their
   --  uncertainties: the eyes see it touch. When not, Height_Above_Support
   --  is no height but a bound: the eyes see nothing of the thing lower, and
   --  what they do not see of it (the sides and underside of a box seen from
   --  above, the underside of a ball) may reach down to the surface, on which
   --  it may then rest. False when it rests on none.

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base;
   --  The hand holding it, or 0.

   function Moving (S : Scene; T : Thing_Id) return Boolean;
   --  Its motion is significant against its own measured noise.

   procedure Remember (S : in out Scene; Point : Point_Estimate; Place : out Place_Id);
   function Where (S : Scene; P : Place_Id) return Point_Estimate;

   --  What the action layer reads of a thing's shape.

   type Sample is record
      Point  : Vec3 := Zero3;
      Normal : Vec3 := Zero3;     --  unit and outward; zero where it was not measured
      Seen   : Boolean := False;  --  an eye saw this face
   end record;
   --  One measured point of a thing's surface, world frame.

   type Sample_Array is array (Positive range <>) of Sample;

   function Samples (S : Scene; T : Thing_Id) return Sample_Array;
   --  Empty until two eyes have seen the thing at the same instant.

   function Sample_Sigma (S : Scene; T : Thing_Id) return Real;
   --  The position uncertainty of a sample; Real'Last when none is measured.

   procedure Touched (S : in out Scene; T : Thing_Id; Point : Point_Estimate);
   --  A point of the thing's surface found by touching it: it is there.

   type Friction_Bounds is record
      Low  : Real := 0.0;
      High : Real := Real'Last;
   end record;
   --  What has been learned of a thing's friction coefficient: it came along
   --  when a contact needed Low, and failed one needing High.

   procedure Learn_Friction (S : in out Scene; T : Thing_Id; Bounds : Friction_Bounds);
   --  Narrows what is known by what an outcome showed.

   function Friction (S : Scene; T : Thing_Id) return Friction_Bounds;

   function Predicted (S : Scene; T : Thing_Id; Beats : Natural) return Point_Estimate;
   --  Where its centre will be that many beats after the latest, from its
   --  measured motion; a thing not moving stays where it is.

   --  Surfaces and places.

   function Surface_Count (S : Scene) return Natural;

   function Plane_Of (S : Scene; F : Surface_Id) return Driver.Geometry.Plane_Estimate;
   --  Its normal points away from the material, towards what rests on it.

   function Earlier (S : Scene; F : Surface_Id) return Boolean;
   --  Measured in an earlier episode and not seen again since.

   function Place_Count (S : Scene) return Natural;

private

   type Scene_Data;
   --  Completed in the body, which uses the layer's own packages.

   type Scene_Data_Access is access Scene_Data;

   type Scene is new Ada.Finalization.Limited_Controlled with record
      Data : Scene_Data_Access;
   end record;

   overriding procedure Finalize (S : in out Scene);

end Driver.World;
