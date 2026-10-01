--  Layer 3: the things around the robot, the surfaces they rest on, the
--  places it remembers, and how all of that moves.
--
--  Each thing has one estimate, shared by every eye and every beat: its
--  surface points and shape, the span two eyes agree on, its parts and their
--  axes, its motion, and what holds it. Names are not kept here: the brain
--  layer binds the brain's names to thing identities.
--
--  Ownership: path B.

with Driver.Commands;
with Driver.Images;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Uncertain;

package Driver.World is

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
   --  Estimators only, one beat.

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

   function Resting_On (S : Scene; T : Thing_Id) return Surface_Id'Base;
   --  The surface it rests on, or 0 when none is measured.

   function Height_Above_Support (S : Scene; T : Thing_Id) return Estimate;
   --  Along Up, from the surface it rests on; unknown when it rests on none.

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base;
   --  The hand holding it, or 0.

   function Moving (S : Scene; T : Thing_Id) return Boolean;
   --  Its motion is significant against its own measured noise.

   procedure Remember (S : in out Scene; Point : Point_Estimate; Place : out Place_Id);
   function Where (S : Scene; P : Place_Id) return Point_Estimate;

private

   type Scene is tagged limited record
      Things : Natural := 0;
   end record;

end Driver.World;
