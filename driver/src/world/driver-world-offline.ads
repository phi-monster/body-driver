--  The world's estimators fed by eyes other than the body's: the bench the
--  offline check on recordings (driver/tools/world_check) runs them on. It
--  gives them the simulator's true cameras and up in place of the body's
--  measured eyes, so whatever it then scores is the world layer's alone. The
--  estimators are Driver.World's own (Driver.World.Estimates), with the
--  same queries; the driver itself never uses this package.

with Driver.Geometry;
with Driver.Images;
with Driver.World.Cameras;
with Driver.World.Pairs;
with Driver.World.Supports;

private with Driver.World.Estimates;

package Driver.World.Offline is

   type Bench is limited private;

   procedure Observe
     (B         : in out Bench;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Up        : Direction_Estimate;
      Still     : Boolean;
      O         : Observation);
   --  One beat, as Driver.World.Observe, with these eyes, up and stillness.

   procedure Adopt (B : in out Bench; E : Eye_Id; O : Observation; Region : Driver.Images.Mask; Thing : out Thing_Id);
   procedure New_Episode (B : in out Bench);

   function Thing_Count (B : Bench) return Natural;
   function Seen_In (B : Bench; T : Thing_Id; E : Eye_Id) return Boolean;
   function Points_Of (B : Bench; T : Thing_Id) return Driver.World.Pairs.Match_Vectors.Vector;
   function Centre (B : Bench; T : Thing_Id) return Point_Estimate;
   function Resting_On (B : Bench; T : Thing_Id) return Surface_Id'Base;
   function Height_Above_Support (B : Bench; T : Thing_Id) return Estimate;

   function Surface_Count (B : Bench) return Natural;
   function Surface_Of (B : Bench; F : Surface_Id) return Driver.World.Supports.Surface;
   function Plane_Of (B : Bench; F : Surface_Id) return Driver.Geometry.Plane_Estimate;

private

   type Bench is limited record
      State : Driver.World.Estimates.State;
   end record;

end Driver.World.Offline;
