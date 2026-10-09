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
--  observation, not many), and by how far the answer moves when the same patch
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

private with Ada.Containers.Indefinite_Holders;
private with Ada.Containers.Vectors;

package Driver.Alignment is

   type Pyramid is private;
   --  A picture's luma at the full size and at every
   --  halving of it that still holds a patch. Made once per picture and used
   --  for every point asked of it.

   function Pyramid_Of (I : Driver.Images.Image) return Pyramid;

   function Levels (P : Pyramid) return Natural;
   --  The number of sizes, the full one included.

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
      Window_Too_Large,   --  wider than a patch of the coarsest level can carry
      Bad_Warp,           --  a mirror, or a change of scale beyond what one level compares
      No_Texture,         --  the patch has no structure to compare
      Ambiguous,          --  another place explains the patch as well as the best
      No_Evidence,        --  the best place explains the patch no better than an unrelated one would
      Not_Solvable,       --  the normal equations of the refinement have no solution (the patch fixes too little)
      Not_Settled,        --  the refinement did not settle within the bits of precision there are
      Left_Window,        --  the refinement carried the place out of the window the prediction allows
      Uninformative);     --  the pixels leave the place as wide as the patch

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
   end record;
   --  To, Cov and Linear are meaningful only when the verdict is Found.

   function Align (First, Second : Pyramid; Query : Prediction) return Answer
     with Pre => Levels (First) > 0 and then Levels (Second) > 0;

private

   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Level is record
      Width, Height : Natural := 0;
      Luma, Spline  : Real_Holders.Holder;   --  the pixels, and the coefficients of the cubic spline through them
   end record;

   package Level_Vectors is new Ada.Containers.Vectors (Natural, Level);

   type Pyramid is record
      Sizes : Level_Vectors.Vector;
   end record;

end Driver.Alignment;
