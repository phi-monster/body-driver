--  An eye fixed in the world, placed by an arm's closer: where the eye saw the closer's parts move between its still
--  views at the closer's two readings (Driver.Robot.Flick), each with the arm held at a pose its fit knows. The parts
--  move the same way in the tool's frame wherever the arm holds the tool, so the centre of what changed is, at every
--  pose, where one point of the tool's frame shows: the beacon. Its place in the tool's frame is not known and is
--  fitted with the eye.
--
--  An eye placed from points tracked on the arm's surface had one point a cell of its picture, and an arm small in a
--  far eye's picture covers a few cells: s83 and y7 (servo seeds 6 and 7 of the fake robot, 10-18) gave eye 1 six and
--  seven points, fewer than its ten terms, and it was never placed. A centre of what a flick changed is one sighting a
--  pose, as many as the arm is held at. August's anchors were the part a matcher of appearance called the hand, a
--  different part at every pose, and its eye stuck at 30 cm; the centres of a flick, studied on the fake robot's truth
--  (tools/probe_study, seeds 7 and 3, 10-18), fall where one point of the tool's frame lands to 1.0 to 2.8 pixels
--  over sixty poses, the arm's joints drawn 0.15 to 0.6 rad about rest.
--
--  The eye's lens is pinhole (no distortion terms: a centre's spread is a pixel or two, and the distortion a pixel's
--  worth only at the picture's edge), its pose that of its camera frame (x right, y down, z ahead) in the arm's
--  reference frame, which is the world for the first arm.

with Ada.Strings.Unbounded;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Beacon is

   type Sighting is record
      Carrier : Driver.Numerics.Rigid;   --  the arm's own eye (the tool's frame) in the arm's reference frame
      U, V    : Real := 0.0;             --  the centre the fixed eye saw
      Sigma   : Real := 1.0;             --  of the centre, pixels, each way
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   Lens_Terms   : constant := 4;   --  the logarithms of Fx and Fy, Cx, Cy
   Pose_Terms   : constant := 6;   --  the camera's turn about its own axes, then its centre
   Beacon_Terms : constant := 3;   --  the beacon in the arm's eye's frame
   Terms        : constant := Lens_Terms + Pose_Terms + Beacon_Terms;

   type Report is record
      Determined : Boolean := False;
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Lens       : Fit.Lens;
      Pose       : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Beacon     : Driver.Numerics.Vec3 := [0.0, 0.0, 0.0];
      Covariance : Driver.Numerics.Arrays.Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];
      Scatter    : Real := 1.0;     --  the residuals' spread over their stated sigmas, never below one
      Pinhole    : Boolean := False;   --  the lens is a pinhole: one focal length, the principal point at the middle
      Used       : Natural := 0;    --  the sightings kept
      Offered    : Natural := 0;    --  the sightings given
   end record;

   procedure Dump (Seen : Sighting_Array; Width, Height : Positive; Name : String);
   --  For a fit studied alone (tools/driver-robot-beacon_replay): with BD_DUMP_FITS naming a folder, the sightings of
   --  a fit in a file there called beacon_NAME_N.txt, N counting the dumps of the run, every real to its last bit;
   --  nothing without the variable.

   procedure Fit_Eye (Seen : Sighting_Array; Width, Height : Positive; Result : out Report);
   --  The eye's lens, its pose and the beacon that put the beacon where the eye saw it at every sighting: a direct
   --  linear start (the beacon at the arm's eye, Hartley's normalized DLT, the camera split from its projection), then
   --  every term by damped Gauss-Newton on the sightings in units of their sigmas; a sighting off by more than Z of the
   --  fit's scatter is left out and the fit run again, until none is. Determined when the sightings kept outnumber the
   --  terms, the normal equations are positive definite, and the lens and the pose are told by the standard that the
   --  points on the arms meet (Hand_Eye.Judge_Lens, Judge_Pose) at the freedom the sightings have; the covariance is
   --  theirs, times the scatter squared.

end Driver.Robot.Kinematics.Beacon;
