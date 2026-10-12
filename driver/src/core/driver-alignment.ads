--  Where a point of one picture lies in another, found by comparing pixels
--  in the window that what is already known allows.
--
--  The caller knows something of where the point should be: geometry puts it
--  at a place in the second picture, with an uncertainty, and turns its
--  surroundings by a warp (the linear part of the map between the pictures,
--  with an uncertainty of its own). That prediction sets where to look: the
--  candidates are the pixels within the prediction's own covariance (as far
--  as a vector of two dimensions is significant, Driver.Uncertain), looked at
--  as coarsely as keeps the window within what a patch can carry (so a wide
--  window is searched at a coarse level of the pictures' halvings and a narrow
--  one at the finest), by the correlation of the first picture's patch,
--  resampled by the predicted warp, with the second picture's pixels. Every
--  place that correlates better than all its neighbours, and the place the
--  prediction itself gives, is then refined, from the coarse level down to the
--  pixels, by Gauss-Newton on the whole warp (the point's place and the four
--  entries of its linear part, with a gain and an offset of the light),
--  directly on the pixels read between them by the cubic spline through them,
--  until a step moves the place by less than Unchanged_Fraction of its own
--  standard error. The refined places are ranked by how little they leave
--  unexplained and how near they lie to the prediction in its own standard
--  deviations; the best is the answer.
--
--  What comes back is a measurement that does not depend on the prediction: the
--  place the pixels give, with the covariance their gradients allow and the
--  scatter of what the fit leaves unexplained show (inflated by how smooth
--  that scatter is, because neighbouring residuals that agree are one
--  observation, not many; and the larger of what the second picture's slopes
--  and the first's, through the fitted gain and warp, allow, because the
--  second picture's noise has slopes too, which a picture whose texture lies
--  under its noise counts as information it does not have), and by how far
--  the answer moves when the same patch
--  is fitted again on its middle and at the next coarser levels (a contour
--  that moves with another surface than the texture beside it, a texture finer
--  than the pictures hold, and a shading that moved all pull the place by more
--  than the noise of one fit says, and move with the choices of the fit). The
--  prediction only bounds the window and starts the fit, so the caller, whose
--  model made the prediction, can take the answer as evidence about its model
--  without counting its own belief twice. Along a direction the patch cannot
--  tell (an edge's own line, a flat patch) the covariance is as wide as the
--  picture: that is what is known.
--
--  The answer says so when it is not found: when the window leaves the picture
--  (not in view), when its patch has no texture, when a second place in the
--  window explains the patch as well as the best does (the two cannot be told
--  apart at the significance level every other gate of the driver uses), when
--  the best does not explain the patch beyond what an unrelated one would at
--  that many candidates, when the refinement leaves the window or does not
--  settle, or when the warp changes the scale beyond what one level of the
--  pictures compares. A patch that cannot tell is tried again twice as wide
--  before it is given up. No size of patch, number of levels or threshold is
--  chosen: the patch is as many pixels wide as the geometric middle between a
--  pixel and the picture (the cells of Driver.Robot.Flow), the levels run as
--  long as such a patch is at least a few pixels wide, and every test is the
--  driver's single gate.

with Driver.Images;

private with Ada.Containers.Vectors;
private with Ada.Finalization;
private with System.Atomic_Counters;

package Driver.Alignment is

   type Pyramid is private;

   --  The aligner's work, phase by phase: the scan of a prediction's window
   --  for candidates, the fits of the peaks found there, the fits down the
   --  levels to the pixels, and the variant fits the covariance rests on.
   type Work_Phase is (Scanning, Peak_Fitting, Level_Fitting, Variant_Fitting, Retrying);
   --  Retrying: the alignments tried again with a patch twice as wide; its passes count the ones then found.

   function Work_Report return String;
   --  The work done since the last report, in whichever tasks: per phase how
   --  many times it was done, how many cells were scanned or passes over a
   --  patch were made, and the seconds it took; the counts start again.
   --  A picture's luma at the full size and at every
   --  halving of it that still holds a patch. Made once per picture and used
   --  for every point asked of it.

   function Pyramid_Of (I : Driver.Images.Image) return Pyramid;

   function Levels (P : Pyramid) return Natural;
   --  The number of sizes, the full one included.

   function Level_Width (P : Pyramid; Level : Natural) return Natural
     with Pre => Level < Levels (P);
   function Level_Height (P : Pyramid; Level : Natural) return Natural
     with Pre => Level < Levels (P);
   --  The size of the picture at a level: each side halved Level times.

   procedure Sample_Level
     (P : Pyramid; Level : Natural; U, V : Real; Value, Du, Dv : out Real; Ok : out Boolean)
     with Pre => Level < Levels (P);
   --  The picture at a level, read between its pixels by the cubic spline through them: its value and its slopes along
   --  u and v at the place (U, V) (corner-origin coordinates of that level's pixels, the first pixel's centre at a
   --  half), in levels of luma per pixel of that level. Not Ok when the sixteen pixels around the place are not all
   --  in the picture.

   Halving : constant := 2;
   --  Each size of the pyramid is this many times smaller across than the one before: a prediction this many times
   --  narrower is searched at a size the pictures are this many times finer at.

   type Linear_Part is record
      UU, UV, VU, VV : Real := 0.0;
   end record;
   --  The derivative of the place in the second picture (u', v') by the
   --  place in the first (u, v): u' changes by UU per unit of u and UV per
   --  unit of v, v' by VU and VV.

   Identity_Part : constant Linear_Part := (UU => 1.0, UV => 0.0, VU => 0.0, VV => 1.0);

   type Place_Covariance is record
      UU, UV, VV : Real := 0.0;
   end record;
   --  Of a place: the variances of u and of v and their covariance, in pixels
   --  squared.

   type Prediction is record
      From         : Driver.Images.Pixel;
      To           : Driver.Images.Pixel;
      Linear       : Linear_Part := Identity_Part;
      Cov          : Place_Covariance;
      Linear_Sigma : Real := Real'Last;
      --  The standard deviation of each entry of Linear; Real'Last when it
      --  is not known.
   end record;
   --  Where the point From of the first picture is expected in the second
   --  (To, with the covariance Cov of that expectation) and how the second
   --  picture turns its surroundings.

   type Verdict is (Found, Not_Found, Not_In_View);

   type Reason is
     (Matched,
      Outside_Picture,    --  the whole window lies beyond the second picture
      Edge_Of_Picture,    --  the point is in the picture but its patch is mostly out of it
      Window_Too_Large,   --  as wide as the picture: it says nothing of where to look
      Bad_Warp,           --  a mirror, or a change of scale beyond what one level compares
      No_Texture,         --  the patch has no structure to compare
      Ambiguous,          --  another place explains the patch as well as the best
      No_Evidence,        --  the best place explains the patch no better than an unrelated one would
      Not_Solvable,       --  the normal equations of the refinement have no solution (the patch fixes too little)
      Not_Settled,        --  the refinement did not settle within the bits of precision there are
      Left_Window,        --  the refinement carried the place out of the window the prediction allows
      Uninformative,      --  the pixels leave the place as wide as the patch
      Not_Asked);         --  not aligned: the field of its keyframe settled on the points before it (Keyframes)

   type Answer is record
      Verdict     : Alignment.Verdict := Not_Found;
      Because     : Reason := No_Texture;
      To          : Driver.Images.Pixel;
      Cov         : Place_Covariance;
      Linear      : Linear_Part := Identity_Part;
      Correlation : Real := 0.0;
      --  Of the first picture's patch with the second's at the answer; zero when none was made.
      Fit_Rms     : Real := 0.0;   --  what the fit leaves unexplained at a pixel, in levels (root mean square)
      Patch_Rms   : Real := 0.0;   --  the patch's own spread, in levels
      Degrees_Of_Freedom : Natural := 0;
      --  How many other fits of the patch (its middle, the coarser levels) the covariance rests on: it is a sample
      --  covariance, so a difference over its sigma follows Student's t with that many degrees of freedom
      --  (Driver.Uncertain), not the Gaussian.
      Narrow : Boolean := False;
      --  The prediction's window was no wider than a patch at the full size, so it was searched where the pictures are
      --  finest. A window wider than that is searched at a coarser size, where several places that a patch cannot tell
      --  apart may lie within it and a wrong one may be taken for the place; such an answer is not one to rest a fit on
      --  before something better than the prediction has been found out about the move.
   end record;
   --  To, Cov and Linear are meaningful only when the verdict is Found.

   Not_Asked_Answer : constant Answer := (Verdict => Not_Found, Because => Not_Asked, others => <>);

   function Align (First, Second : Pyramid; Query : Prediction; Widen : Boolean := True) return Answer
     with Pre => Levels (First) > 0 and then Levels (Second) > 0;
   --  Widen: a point the usual patch cannot decide may be tried with one twice as wide (four times the
   --  pixels); the caller allows that only while it has nothing else to stand on (Keyframes: no field yet).
   --  R1 on the Pi: the wide retries found one point in twelve and were most of the aligner's time.

   function Largest_Window (First, Second : Pyramid) return Real
     with Pre => Levels (First) > 0 and then Levels (Second) > 0;
   --  The radius, in pixels of the full-size pictures, of the widest window of places Align searches for a point
   --  between these two pictures: half the second picture's smaller side. The prediction's covariance, taken at
   --  the gate for two dimensions, must lie within it, or the answer is Window_Too_Large: a window as wide as the
   --  picture says nothing of where to look. A window wider than a patch at every size of the pictures is searched
   --  at the size where it is the least wider than its patch, which tells the place no better than that size
   --  allows and often cannot tell it from another (Ambiguous).

   function Largest_Sigma (First, Second : Pyramid) return Real
     with Pre => Levels (First) > 0 and then Levels (Second) > 0;
   --  The largest standard deviation, along both axes alike, of a prediction whose window Align still searches
   --  (Largest_Window less what a place on a grid of pixels cannot be known to beyond): the width of "anywhere
   --  the aligner can look".

   function Window_Radius (Sigma : Real) return Real;
   --  The radius, in pixels, of the window of a prediction whose standard deviation is Sigma along both axes: as far
   --  as a vector of two dimensions is significant, and a place on a grid of pixels cannot be known to beyond.

   function Half_Patch (Width, Height : Natural) return Natural;
   --  Half the width of the patch Align matches a point by, in pictures of this size: the patch is as many pixels
   --  wide as the geometric middle between a pixel and the picture's size (Driver.Robot.Flow's cells), half of it
   --  either side of the point. A prediction's window no wider than a patch at the full size is searched where the
   --  pictures are, and such a point is followed (Driver.Robot.Kinematics.Followed).

private

   --  An array of reals made once and read after by any number of alignments at once (Driver.Parallel_For). The
   --  levels that share it count themselves with an atomic counter, and reading it writes nothing. (A reference
   --  into an Ada.Containers.Indefinite_Holders element counts itself in a field of the holder that every reader
   --  writes without a lock: read by tasks together, the count is lost, the holder never returns to zero and
   --  refuses its own finalization.)
   type Reals_Access is access Real_Array;

   type Counted is limited record
      Count  : System.Atomic_Counters.Atomic_Counter;
      Values : Reals_Access;
   end record;

   type Counted_Access is access Counted;

   type Shared_Reals is new Ada.Finalization.Controlled with record
      Held : Counted_Access;
   end record;

   overriding procedure Adjust (Item : in out Shared_Reals);
   overriding procedure Finalize (Item : in out Shared_Reals);

   type Level is record
      Width, Height : Natural := 0;
      Luma, Spline  : Shared_Reals;   --  the pixels, and the coefficients of the cubic spline through them
   end record;

   --  A pyramid is read by several points' alignments at once too: a reference into its vector without the
   --  tampering check is a plain pointer, where the check's counters are written by every reader.
   pragma Suppress (Tampering_Check);
   package Level_Vectors is new Ada.Containers.Vectors (Natural, Level);

   type Pyramid is record
      Sizes : Level_Vectors.Vector;
   end record;

end Driver.Alignment;
