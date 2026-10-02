--  How to hold the hand for a press.
--
--  A press aims one lobe: the hand turns about the eye, which therefore stays
--  where it is and keeps its view, until the lobe's line of sight, tilted,
--  points the way the hand will be pressed. Tilting away from the hand's
--  other lobes keeps them behind the aimed tip; the tilts' sizes are set by
--  the hand itself, from the angle between its lobes' lines of sight. All of
--  it is plain geometry on what the hand measured; the decider that moves
--  the arm is Driver.Robot.Hand.Measure.

package Driver.Robot.Hand.Aims is

   function Tilted (Sight, Away : Vec3; Tilt : Real) return Vec3;
   --  The unit direction Tilt radians from Sight, turned towards Away (any
   --  direction not along Sight).

   function Turned_About (Tool : Rigid; Eye, Along, Into : Vec3) return Rigid;
   --  The tool's pose turned about the eye (a point in the tool frame) by the
   --  least rotation that points Along (a tool-frame direction) along Into (a
   --  world direction); the eye stays where it was.

   function Any_Across (V : Vec3) return Vec3;
   --  A unit direction across V.

   type Direction_Array is array (Positive range <>) of Vec3;

   function Away_From (Sight : Vec3; Rest : Direction_Array) return Vec3;
   --  The unit direction across Sight away from the other lobes' lines of
   --  sight (from their mean); zero when there are none or they lie along it.

   function Spread (Sight : Vec3; Rest : Direction_Array) return Real;
   --  The angle to the nearest other line of sight: the hand's own angular
   --  scale; zero when there are no others.

end Driver.Robot.Hand.Aims;
