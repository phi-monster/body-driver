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

   --  An arm the eye watched: its poses' evidence and its joints along its chain from the base, as its fit has them
   --  (its reference frame and its unit).
   type Arm_Flow is record
      Poses  : access constant Pose_Array;
      Joints : access constant Fit.Joint_Array;
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
