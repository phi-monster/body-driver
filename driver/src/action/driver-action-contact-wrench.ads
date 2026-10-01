--  The physical check of a contact set: one linear program serves every want.
--
--  Quasi-static: the motion is slow enough that inertia does not count, so the
--  body's touches, the footing and gravity balance, with forces in units of
--  the thing's weight. The body's touches move with the thing without
--  slipping, each within its friction cone, here an inscribed polygon whose
--  relative error stays below Driver.Conventions.Unchanged_Fraction; a touch
--  with a patch also resists torsion about its normal. The footing's part
--  follows from how each foot point moves under the twist: leaving the
--  surface it bears nothing; going into it the motion is impossible (the
--  surface is in the way); staying on it, it bears load anywhere in the convex
--  hull of the staying points, with friction against the slip of every
--  slipping point, or within its cone where nothing slips. The cost is the sum
--  of the body's normal forces; the surface's are free. One friction
--  coefficient serves the thing's touches with the body and with its footing.

with Driver.Uncertain;

package Driver.Action.Contact.Wrench is

   No_Way : constant Real := Real'Last;

   type Obstacle is (None, Footing_In_Way, Unbalanced);
   --  Footing_In_Way  the wanted motion drives a foot point into its surface
   --  Unbalanced      no forces within the cones balance gravity in that motion

   type Answer is record
      Force : Real := No_Way;          --  least sum of the body's normal forces, per unit weight
      Why   : Obstacle := Unbalanced;  --  None when Force < No_Way
   end record;

   function Need
     (Touches : Touch_Vectors.Vector;
      Base    : Footing;
      Motion  : Twist;
      Centre  : Vec3;
      Up      : Vec3;
      Mu      : Real) return Answer
     with Pre => Mu >= 0.0 and then abs Up > 0.0;
   --  Can the touches with the footing move the thing as Motion says, against
   --  gravity at Centre pointing along -Up, with friction coefficient Mu?

   function Least_Friction
     (Touches    : Touch_Vectors.Vector;
      Base       : Footing;
      Motion     : Twist;
      Centre     : Vec3;
      Up         : Vec3;
      Resolution : Real) return Real
     with Pre => abs Up > 0.0;
   --  The least friction coefficient for which Need finds a way; No_Way when
   --  none does. Resolution is the friction angle, in radians, below which two
   --  answers cannot be told apart (the angular uncertainty of the touches'
   --  normals); without one the search stops when its interval is below
   --  Unchanged_Fraction of the angle.

   type Rest_Answer is record
      Rests  : Boolean := False;
      Margin : Real := Real'First;   --  centre to the nearest edge of the foot's hull, inside positive
   end record;

   function Rests
     (Base   : Footing;
      Centre : Driver.Uncertain.Point_Estimate;
      Up     : Vec3;
      Mu     : Real) return Rest_Answer
     with Pre => Mu >= 0.0 and then abs Up > 0.0;
   --  Whether the footing alone keeps the thing in place, with no touch of the
   --  body: it must balance gravity with the centre moved Z standard
   --  deviations toward every edge of the foot's hull. A foot whose points
   --  span no area never rests anything.

end Driver.Action.Contact.Wrench;
