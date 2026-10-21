--  An eye that no arm carries, placed in an arm's frame by how its picture changes while the arm's joints turn a
--  little at a pose: at a pixel on the arm the picture changes by its gradient times the motion of the arm's surface
--  there, that motion is the joint's turn (its axis and point as the arm's fit has them) at the place the pixel's line
--  of sight meets the arm, and the place's depth along the line is all a pixel adds; the turns of the joints that move
--  it tell that depth and leave the rest to the eye's lens and pose.
--
--  No point of the arm is followed from picture to picture and the arm needs no texture of its own: the picture's
--  gradient is wherever the arm's edges stand against what lies behind it, and every pixel along them is a sighting. A
--  fixed eye that tracked the arm's points followed 2.8 and 5.5 of them a step on the mean (the cold table boots e20
--  and e21, 10-20: the arm a dozen cells of a 320 by 240 picture, its texture finer than a pixel), and was never
--  placed.

with Ada.Strings.Unbounded;
with Driver.Geometry;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Flow_Eye is

   type Luma_Access is access Real_Array;
   type Luma_Array is array (Positive range <>) of Luma_Access;

   --  What one pose of the arm gives: the arm's readings there (each joint's change from the arm's reference, along the
   --  chain), and every pixel that some joint's turn changed: its centre, the picture's gradient there, the change of
   --  the picture there per reading unit of each joint with its standard deviation, and how many joints along the chain
   --  move it (the highest whose change stands out).
   type Pose_View (Joints, Pixels : Natural) is record
      Readings     : Real_Array (1 .. Joints);
      U, V, Gu, Gv : Real_Array (1 .. Pixels);
      Rate, Sigma  : Driver.Numerics.Arrays.Real_Matrix (1 .. Pixels, 1 .. Joints);
      Link         : Fit.Count_Array (1 .. Pixels);
      --  the size of the background's gradient Evidence took off the pixel's (where the background shows within the
      --  smoothing's reach of it; nought elsewhere)
      Back         : Real_Array (1 .. Pixels);
   end record;

   type Pose_Access is access Pose_View;
   type Pose_Array is array (Positive range <>) of Pose_Access;

   function Evidence
     (Readings      : Real_Array;
      Still         : Real_Array;
      Background    : Real_Array;
      Noise         : Real;
      Minus, Plus   : Luma_Array;
      Steps         : Real_Array;
      Width, Height : Positive) return Pose_Access
   with Pre => Still'Length = Width * Height and then Background'Length = Width * Height
               and then Minus'Length = Readings'Length
               and then Plus'Length = Readings'Length and then Steps'Length = Readings'Length;
   --  Still: the picture's luma at the pose, row after row, a mean of still frames; Noise: the standard deviation of
   --  one pixel of the difference of two such means where nothing changed; Minus (J) and Plus (J): the luma with joint
   --  J alone turned by -Steps (J) and +Steps (J) reading units from the pose. A pixel is kept where the gradient
   --  stands out of the noise and the change of some joint does, each pixel one test of a family of as many as the
   --  picture has (Driver.Uncertain, Tests).

   function Smoothing_Radius return Natural;
   --  How far, in whole pixels, the smoothing reaches that Evidence takes the picture's gradient and its changes
   --  through (for studies: tools/driver-robot-flow_bench).

   function Best_Turn
     (Still, Turned : Real_Array;
      Turn          : Real;
      Noise         : Real;
      Width, Height : Positive) return Real
   with Pre => Still'Length = Width * Height and then Turned'Length = Width * Height and then Turn > 0.0;
   --  The turn either way that measures a joint's rates best, from the picture at the pose (Still) and with the joint
   --  turned by Turn reading units (Turned), Noise as Evidence has it. The pixels the turn moved (their change and
   --  gradient standing out of the noise, as Evidence keeps them) move by the median of |change| / |gradient| over
   --  Turn pixels a unit; a central difference over a motion d either way errs by d^2 / (6 s^2) of the rate on an edge
   --  blurred by s (the smoothing's own sigma at least) and by the smoothed change's noise over 2 d |g| (|g| their
   --  median gradient), the two balanced at d = (3 s^2 sigma_C / (sqrt 2 |g|)) ^ (1/3) (the turn sweep of 10-20: the
   --  rates' bias +6 % at 0.001 rad, +1 % at 0.008, their spread up again at 0.016). Turn itself when no pixel moved.

   --  An arm the eye watched: its poses' evidence and its joints along its chain from the base, as its fit has them
   --  (its reference frame and its unit), and per joint the relative standard deviation of its unit as its fit states
   --  it (none: the units are exact). A unit off by 2 % moved an arm's centre by 1 to 5 mm on the model's own rates,
   --  its sigma 0.4 (seed 22, 10-21): each joint's unit is a term of the fit, held to its own by that deviation.
   type Arm_Flow is record
      Poses      : access constant Pose_Array;
      Joints     : access constant Fit.Joint_Array;
      Unit_Sigma : access constant Real_Array := null;
   end record;

   type Arm_Flow_Array is array (Positive range <>) of Arm_Flow;
   type Rigid_Array is array (Positive range <>) of Driver.Numerics.Rigid;

   --  The fit's terms for so many arms: the lens's three and each arm's six.
   function Terms_For (Arms : Positive) return Positive is (3 + 6 * Arms);

   type Report (Arms, Count : Positive) is record
      Determined : Boolean := False;
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Lens       : Fit.Lens;
      --  The camera's frame in each arm's reference frame.
      Poses      : Rigid_Array (1 .. Arms) := [others => Driver.Numerics.Identity];
      --  Of the logarithm of the focal length and the principal point, then of each arm's turn about the camera's own
      --  axes and centre, in that order.
      Covariance : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. Count) := [others => [others => 0.0]];
      Scatter    : Real := 1.0;    --  the residuals' spread over the sigmas the pictures state, never below one
      Used       : Natural := 0;   --  the pixels kept
      Offered    : Natural := 0;   --  the pixels that two joints or more move
      Spread     : Real := Real'Last;
      --  how far the fit leaves the place in the picture of the arm's points it saw: the worst of them, pixels
      Widened    : Real := 1.0;
      --  the most the covariance was widened in any direction for what the poses' pictures share (the jackknife over
      --  the poses)
      Additive, Relative : Real := 0.0;
      --  the rates' spread the fit found: times their stated sigmas, and a share of the rate itself
   end record;

   procedure Fit_Eye
     (Arms          : Arm_Flow_Array;
      Width, Height : Positive;
      Result        : out Report)
   with Pre => Result.Arms = Arms'Length and then Result.Count = Terms_For (Arms'Length);

   --  What an arm's own eye saw of the world at the poses of its flow, for placing the fixed eye by the world's points
   --  (Refine_By_Scene): its pictures (luma, row after row) with the eye's frame at each in the arm's reference frame,
   --  its lens, and the table its fit measured there (none when it measured none: the pictures then place nothing).
   type Pixel_Flags is array (Positive range <>) of Boolean;
   type Pixel_Flags_Access is access Pixel_Flags;

   type Wrist_Picture is record
      Luma          : Luma_Access;
      Width, Height : Positive := 1;
      Eye           : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      --  Per pixel, row after row, whether the eye sees the world there and not the arm's own body riding with it
      --  (the hand's jaws): none, the world everywhere.
      World         : Pixel_Flags_Access := null;
   end record;
   type Wrist_Picture_Array is array (Positive range <>) of Wrist_Picture;

   --  A fit of an arm that its own fit leaves as likely as itself: the fit's terms (its eye's lens, its joints, and the
   --  table that moves with every depth they move) moved by a standard deviation along one axis of their covariance.
   --  Its joints along the chain from the base (as Arm_Flow's), its eye's frame at each of its pictures (as
   --  Arm_World's, in their order), its eye's lens and its table. The scatter of the places and of the flows tells
   --  nothing of how far the arm's own fit is off: joints 0.2 degrees, 2 mm and half a per cent off left the fake's
   --  seed 22's arms' joints 2.06 px from the true eye's places against a spread of 0.98, its seed 24's units alone
   --  2.20 against 0.67 (10-21).
   type Arm_Variant is record
      Joints : access constant Fit.Joint_Array;
      Eyes   : access constant Rigid_Array;
      Lens   : Fit.Lens;
      Table  : Driver.Geometry.Plane_Estimate;
   end record;
   type Arm_Variant_Array is array (Positive range <>) of Arm_Variant;

   type Arm_World is record
      Pictures : access constant Wrist_Picture_Array := null;
      Lens     : Fit.Lens;
      Table    : Driver.Geometry.Plane_Estimate;
      --  The arm's fit's uncertainty: per axis of its covariance the fit moved along it either way, the lesser end
      --  first (2 K - 1, then 2 K); none, the fit taken as exact.
      Variants : access constant Arm_Variant_Array := null;
   end record;
   type Arm_World_Array is array (Positive range <>) of Arm_World;

   procedure Refine_By_Scene
     (Arms          : Arm_Flow_Array;
      Worlds        : Arm_World_Array;
      Fixed         : Real_Array;
      Width, Height : Positive;
      Result        : in out Report)
   with Pre => Worlds'Length = Arms'Length and then Result.Arms = Arms'Length
               and then Result.Count = Terms_For (Arms'Length) and then Fixed'Length = Width * Height;
   --  The eye Fit_Eye placed (Result), placed again with the world the arms' own eyes saw: the points of each arm's
   --  table its eye's pictures show (a grid a patch apart, where their lines of sight meet the table), each predicted
   --  into the fixed eye's picture (Fixed) by the eye as it stands and aligned there across the scales
   --  (Driver.Alignment.Align_At_Scale, the answers searched narrowly); the eye's terms fitted to the places found,
   --  from the flow's estimate, round after round until a round after the first moves the arms' joints in the picture
   --  by no more than the eye's spread there leaves them, or neither the places' scatter nor that move falls. A table
   --  of texture tells the eye's lens and pose to a fraction of a pixel where the arms' outlines told it to several
   --  (the fake's seeds 20, 22 and 23, 10-21: the arms' joints 0.65 to 2.48 px from the true eye's places, against 3.7
   --  to 16 px). No point of the world found (a table with no texture): Result as Fit_Eye left it. What the arms' own
   --  fits leave uncertain (Arm_World.Variants) is carried into the covariance and the spread: each variant's places
   --  and flows taken again at the eye as it stands, the eye moved as the round's own steps would move it for them (the
   --  places' Gauss-Newton step, the flows' along their directions), and the arms' joints in the picture moved by that
   --  and by their own move in the variant. Determined as Fit_Eye's: settled, the covariance definite and Z times
   --  Spread under the half patch.

   procedure Judge_At
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Scatter       : out Real;
      In_Front      : out Natural);
   --  What the model leaves at a given eye (a pinhole of that lens at that pose): the residuals' root mean square over
   --  their sigmas (every pixel's depth eliminated), and how many of the pixels that two joints or more move it puts in
   --  front of the eye. For a fit studied against the truth (tools/driver-robot-flow_bench). Joints: the arm's joints
   --  along its chain from the base, as its fit has them (its reference frame and its unit). The lens is a pinhole: one
   --  focal length, its principal point fitted from the picture's middle. The start is a race over the camera's turns
   --  and focal lengths, run at counts of turns that double until it finds one optimum twice (Search); the optimum is
   --  refined on every pixel, every pixel's depth eliminated at each step, the rates' spread found (a share of their
   --  sigmas and of themselves), the pixels off by more than the gate left out until none is, and the covariance
   --  widened by the eye's spread over the poses (the jackknife). Determined when the search found its optimum, the
   --  covariance is positive definite and Z times Spread is less than the half patch the aligner follows
   --  (Driver.Alignment.Half_Patch).

   function Residuals_At
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Relative      : Real) return Real_Array;
   --  Every pixel's residuals at a given eye (a pinhole of that lens at that pose) with the given joints, its depth
   --  eliminated, each in units of its rate's spread (its sigma and Relative times the rate, in quadrature): pixel
   --  after pixel and each one's joints in order, nought where the terms move no point on its line of sight, as many
   --  for the same poses whatever the joints and the eye. For a fit studied against the truth
   --  (tools/driver-robot-flow_bench).

   procedure Synthesize
     (Poses         : Pose_Array;
      Joints        : Fit.Joint_Array;
      Width, Height : Positive;
      Pose          : Driver.Numerics.Rigid;
      Lens          : Fit.Lens;
      Deviates      : not null access function return Real);
   --  Every pixel's rates made the model's at the given eye (each pixel's depth the one the measured rates give it)
   --  with a standard Gaussian deviate from Deviates times each rate's sigma added: the pictures' places and gradients
   --  as they were, rates the model holds exactly. For a fit studied against the truth (tools/driver-robot-flow_bench).

end Driver.Robot.Kinematics.Flow_Eye;
