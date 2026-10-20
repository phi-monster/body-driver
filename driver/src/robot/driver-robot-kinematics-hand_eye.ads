--  The lens and the place of an eye that stands still in the world, measured from the arms it watches.
--
--  A fixed eye (the head's) sees an arm move, and that is all it needs: the arm's own fit (Driver.Robot.Kinematics.Fit,
--  from the pictures of the arm's eye) tells where every link of the arm stands at every reading, in the arm's
--  reference frame and in the arm's unit; the points of the arm that the fixed eye's pictures show at those readings
--  are then the sightings of an eye whose lens and place are unknown, of points whose places are unknown too, except
--  that each lies on the line of sight of the pixel it was chosen at and rides on one link of the arm. There is no
--  second eye's view of anything to be matched with it.
--
--  A Watched is what one arm gives of itself: its joints as fitted, the readings of its keyframes (the first is the
--  reference), the fit's covariance of its joints' terms, the points tracked in the fixed eye's reference picture (the
--  picture the eye gave when the arm's reference keyframe was taken), and where each was seen at every other keyframe.
--  A track's link is the number of joints that move it: it moves when any of the first Link joints does, and is moved
--  by none of the rest. None is told: Fit_Eye reads it from the sightings (the keyframes at which one joint alone
--  moved the arm tell which joints move a point, Infer_Link), starts the eye from the points that ride on the arm's
--  last link (the hand: the points whose motion is largest, and the surest of what they ride on), and when it has a
--  model of the eye reads every track's link again from how the points should have moved, until the links stop
--  changing.
--
--  The start is a search. The eye's turn is unknown over all of the rotations, the lens over every field of view; but
--  for a rotation and a focal length the eye's centre (in the arm's frame) and every point's depth are the solution of
--  linear equations (the line of sight of each sighting passes through the point the link's motion carried the point
--  to), so the rotations of a spiral that covers them evenly, in as many as make the chance that none lies within the
--  distance the refinement converges from at most Z's two-sided tail, are each scored by what the best centre and
--  depths leave, and the best are refined. The refinement is robust least squares (Huber at Z) on the lens, the pose
--  and the log of every depth, the depths eliminated from the normal equations. More than one arm: the lens is shared,
--  each arm has a pose, and the arms after the first start from the first's lens.
--
--  The covariance is the sandwich of the normal equations around the spread of the gradient (Errors), plus what the
--  arm's own uncertainty adds: how every sighting moves with the terms of the arm's joints, carried through the
--  covariance the arm's fit gives them. The eye is determined when its normal equations are positive definite, its
--  covariance finite and the cost quadratic over it, both focal lengths significant against their own sigma, the
--  principal point located within half the picture at Z sigmas, every arm's pose known (the turn to within a quarter
--  turn at Z sigmas); else Why says what is left out.
--
--  A pose is the eye's camera frame in the arm's reference frame: X_arm = Rotation * X_eye + Translation, in the arm's
--  unit. The camera has z along its optical axis, x along +U and y along +V.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Hand_Eye is

   Pose_Terms : constant := 6;
   --  A pose's terms: the turn about the camera's own axes (3) and the centre in the arm's frame (3).

   function Terms (Arms : Positive) return Positive is (Fit.Lens_Terms + Pose_Terms * Arms);
   --  The terms of the report's covariance: the lens's six as an arm's fit has them (the logarithms of Fx and Fy, Cx,
   --  Cy, K1, K2), then each arm's pose.

   --  A point of the arm that the fixed eye's reference picture shows.
   type Track is record
      U0, V0 : Real := 0.0;     --  where the picture it was chosen at shows it
      Frame  : Positive := 1;   --  the keyframe that picture is (the arm's reference is 1): a point of the arm the eye
                                --  took as the arm carried it into view rides from there
   end record;

   type Track_Array is array (Positive range <>) of Track;

   --  Where the eye saw a track at a keyframe of the arm (2 or more; the reference is 1).
   type Sighting is record
      Track       : Positive := 1;
      Frame       : Positive := 2;
      U, V        : Real := 0.0;
      Own         : Real := 0.0;   --  the standard deviation the aligner stated of the place, per axis (0: none)
      Own_Freedom : Natural := 0;  --  how many degrees of freedom it rests on (0: it is known)
   end record;

   type Sighting_Array is array (Positive range <>) of Sighting;

   --  What one arm gives of itself.
   type Watched (Joints, Frames, Tracks, Sightings : Natural) is record
      --  The joints in their order along the arm, from the base, and the readings' changes in the same order.
      Joint      : Fit.Joint_Array (1 .. Joints);
      Change     : Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. Joints);
      --  every keyframe's reading change from the reference (the arm's first keyframe, the eye's reference picture
      --  and the frame of the arm's fit), per joint (row 1 is zero)
      Visible    : Driver.Numerics.Arrays.Real_Vector (1 .. Joints);
      --  per joint, the reading change that moves the arm's own eye's view by what the aligner can tell
      Covariance : Fit.Real_Lists.Vector;
      --  of the joints' terms (Fit.Joint_Term, joint after joint, Fit.Joint_Terms each), row by row: the arm fit's
      --  own, or empty when it has none (the arm's uncertainty then adds nothing to the eye's)
      Track      : Track_Array (1 .. Tracks);
      Seen       : Sighting_Array (1 .. Sightings);
   end record;

   type Watched_Access is access Watched;

   type Watched_Array is array (Positive range <>) of Watched_Access;

   function Infer_Link (A : Watched; T : Positive) return Natural;
   --  The number of joints that move track T, from the keyframes at which one joint alone moved the arm (a change of
   --  that joint's reading beyond what the arm's eye sees, of no other's): the highest joint that a significant motion
   --  of the track, in proportion to that joint's reading change, goes with. 0 when none does, or when the keyframes
   --  tell no joint's effect. A joint that has no keyframe of its own to tell by is not counted.

   package Link_Vectors is new Ada.Containers.Vectors (Positive, Natural);

   type Arm_Report is record
      Placed : Boolean := False;
      --  the arm has an eye pose the sightings determine (its turn to within a quarter turn at Z sigmas, its centre
      --  finite); else Pose is the identity and the rest says what the start was
      Pose  : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      --  the eye's camera frame in the arm's reference frame
      Link  : Link_Vectors.Vector;
      --  per track, the number of joints that move it at the end; 0 where the track rides on nothing the model explains
      Depth : Fit.Real_Lists.Vector;
      --  per track, its depth along its reference line of sight, in the arm's unit
      Fits  : Flag_Vectors.Vector;
      --  per sighting (from 0), whether it fits the model at the noise the residuals show
   end record;

   package Arm_Report_Vectors is new Ada.Containers.Vectors (Positive, Arm_Report);

   type Fit_Report is record
      Determined : Boolean := False;
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      --  every criterion the eye fails (empty when it fails none), and the arms it has no pose for
      L          : Fit.Lens;
      Arms       : Arm_Report_Vectors.Vector;
      Covariance : Fit.Real_Lists.Vector;
      --  Terms (Arms) x Terms (Arms), row by row (zeros for the terms not kept)
      Sigma_Px   : Real := 0.0;    --  the noise the residuals show beyond what the sightings state
      Used       : Natural := 0;   --  the sightings that fit
      Offered    : Natural := 0;   --  the sightings the model has a place for
      Distorted  : Boolean := False;   --  the distortion terms are kept
      Linear_To  : Real := Real'Last;
      --  how many sigmas of the cost itself a displacement of Z sigmas of the normal equations is, the least over the
      --  covariance's principal directions and both ways: Z where the cost is quadratic over it
   end record;

   procedure Fit_Eye
     (Arms          : Watched_Array;
      Width, Height : Positive;
      Prior         : Fit_Report;
      Report        : out Fit_Report);
   --  The lens and the pose in every arm's frame of the eye that watched them, as above. Report.Arms has an element
   --  for every arm of Arms, in order. Prior is what an earlier fit of the same eye to the same arms (with fewer
   --  keyframes) found: an arm it placed starts from its pose, links and depths (and the lens from its lens) instead
   --  of from the search, which every later fit would repeat for nothing; the refinement and the reading of the links
   --  from the model go on from there, so the answer is the one a search would find if the optimum is the same, and is
   --  not determined if the new keyframes say otherwise (the next fit then has no prior to start from).

end Driver.Robot.Kinematics.Hand_Eye;
