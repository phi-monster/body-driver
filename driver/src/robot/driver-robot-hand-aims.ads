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
   --  direction in the frame the tool's pose is given in); the eye stays where
   --  it was.

   function Any_Across (V : Vec3) return Vec3;
   --  A unit direction across V.

   type Direction_Array is array (Positive range <>) of Vec3;

   function Away_From (Sight : Vec3; Rest : Direction_Array) return Vec3;
   --  The unit direction across Sight away from the other lobes' lines of
   --  sight (from their mean); zero when there are none or they lie along it.

   function Spread (Sight : Vec3; Rest : Direction_Array) return Real;
   --  The angle to the nearest other line of sight: the hand's own angular
   --  scale; zero when there are no others.

   function Least_Tilt (Distance : Estimate) return Real;
   --  The least tilt at which a second press can tell a tip from a stop on
   --  something that does not move with the tilt. A tip s along its line of
   --  sight, pressed straight, meets the surface s from the eye; the same
   --  stop of the eye, pressed tilted by T, meets it s / cos T: the hits
   --  differ by s T^2 / 2 at least, and that tells from the noise of two hits
   --  when it exceeds Z times the square root of 2 of the tip's sigma. Real'Last
   --  when the distance is not known: no tilt is then told from another.

   procedure Next_Tilt (Tilt : in out Real; Stalled : Boolean; Bound : in out Real; Least : Real);
   --  The tilt of the next press of a lobe's tip on one side, after one at
   --  Tilt that stopped short of the table (Stalled: no tip rests on it) or
   --  did not; zero when there is none. The tilts double while the presses
   --  are ones the tip rests on, as the hand's own angle sets them, and
   --  halve when one stops short: an arm that cannot make a tilt can often
   --  make half of it (A16: the aims of both tilted presses, 0.96 and 1.23
   --  rad, stood the wrist's fifth joint at 0.743 rad, 0.002 under the largest
   --  reading it had in the run, where the straight press's aim had it at
   --  0.258; both stopped on the arm itself). Bound is the least tilt
   --  found not to be made on this side, which none after it reaches; Least
   --  is Least_Tilt, which none under it is worth.

end Driver.Robot.Hand.Aims;
