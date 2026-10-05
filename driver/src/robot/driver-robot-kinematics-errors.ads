--  How the matcher's errors depend on each other, read from the residuals a
--  fit leaves behind, and what that does to the fit's covariance.
--
--  A sighting's error is two numbers, across and down, in pixels. A fit that
--  counts its sightings as independent, or clusters them by keyframe, is far
--  too sure when they are not (A11: its lens came out 20 to 40 chi squares
--  off the truth, six terms, where Z's tail is 21). They are not, in three
--  ways the residuals show:
--
--  * the same point errs alike in every keyframe (where the matcher finds
--    it is the same in every view), and so do points near each other: their
--    errors are a field over the reference picture that does not change with
--    the keyframe, alike up to a range of pixels;
--  * the points of one keyframe err alike (its rendering, its view) and,
--    nearer, more (a smooth field of that keyframe's own);
--  * what is left of each sighting is its own.
--
--  So the covariance of two sightings' errors is one of four functions: of
--  one sighting with itself (Alone); of two sightings of one point, which
--  are in two keyframes (Same_Track); of two points in two keyframes
--  (Persistent, of the distance between their reference pixels); and of two
--  points in one keyframe (Shared, of the same distance). Each is the mean
--  of the products of the residuals the fit left, by the distance of the
--  pixels, the products of one keyframe's pair (Shared) apart from those of
--  two keyframes' (Persistent), then made to fall with the distance and
--  never below zero: a field's covariance does not grow apart (isotonic
--  regression, Barlow et al.: no bin, no range, no bandwidth is chosen). The
--  range of each is where its fit stops falling; beyond it the covariance is
--  its last level, which is the error every point of a keyframe shares in
--  Shared. What the fit absorbed (a term it moved to fit the persistent
--  part) the residuals do not hold: the gradient rows of the fit are
--  orthogonal to a point's depth, which absorbs the part along its epipolar
--  line, so the part left is the part that moves the terms.
--
--  Meat is then the sum, over every pair of residual rows, of their
--  covariance times the outer product of their gradients (the score
--  contributions of the two): the middle of the sandwich of Conley (spatial
--  dependence), with the measured covariance for the products of the
--  residuals, which are noisy and, summed with the gradient, vanish at a
--  fit's optimum.

with Driver.Numerics;

package Driver.Robot.Kinematics.Errors is

   --  The covariance of one sighting's two errors with another's: across with
   --  across, down with down, and across with down (the same whichever
   --  sighting is taken first: the field is the same in every direction).
   type Pair_Kind is (Both_Across, Both_Down, Across_Down);
   type Entries is array (Pair_Kind) of Real;

   type Model is private;
   --  Heap-held: Free it.

   procedure Measure
     (Frame, Track : Natural_Array;
      U0, V0       : Real_Array;
      Error        : Real_Array;
      M            : out Model)
     with Pre => Frame'First = 1 and then Track'First = 1 and then Track'Length = Frame'Length
                 and then U0'First = 1 and then U0'Length = Frame'Length and then V0'First = 1
                 and then V0'Length = Frame'Length and then Error'First = 1
                 and then Error'Length = 2 * Frame'Length
                 and then (for all F of Frame => F > 0) and then (for all T of Track => T > 0);
   --  Sighting K is of the point Track (K) in keyframe Frame (K), whose
   --  reference pixel is (U0 (K), V0 (K)) and whose errors, across and down,
   --  are Error (2 K - 1) and Error (2 K): a fit's residuals, as its score
   --  sees them (clipped at Z of their scale). Not Measured unless two
   --  keyframes hold sightings, two points are sighted, and some point is
   --  sighted in two keyframes: then nothing can be told of how they depend.

   function Measured (M : Model) return Boolean;

   function Alone (M : Model) return Entries;
   --  The covariance of a sighting's errors with each other.

   function Same_Track (M : Model) return Entries;
   --  The covariance of one point's errors in two keyframes.

   function Persistent (M : Model; Distance : Real) return Entries;
   --  The covariance of the errors of two points, in two keyframes, whose
   --  reference pixels are Distance apart.

   function Shared (M : Model; Distance : Real) return Entries;
   --  The same for two points of one keyframe.

   function Added_By_Keyframe (M : Model) return Entries;
   --  What the errors of a keyframe add to the covariance of two of its points,
   --  over what two points of two keyframes have (Shared less Persistent),
   --  averaged over all the pairs of points that keyframes held: the error
   --  a keyframe gives its points.

   function Half_Distance_Persistent (M : Model) return Real;
   --  The distance at which Persistent has fallen to half of Same_Track (the
   --  covariance of a point with itself in another keyframe), in pixels, across
   --  and down alike: how far the errors of near points are alike, read from the
   --  function itself; the nearest distance the points were paired at if the
   --  function is below half there already, 0 when Same_Track is not above zero.

   procedure Meat
     (M      : Model;
      Frame  : Natural_Array;
      Track  : Natural_Array;
      U0, V0 : Real_Array;
      Rows   : Driver.Numerics.Arrays.Real_Matrix;
      Result : out Driver.Numerics.Arrays.Real_Matrix)
     with Pre => Frame'First = 1 and then Track'First = 1 and then Track'Length = Frame'Length
                 and then U0'First = 1 and then U0'Length = Frame'Length and then V0'First = 1
                 and then V0'Length = Frame'Length and then Rows'First (1) = 1 and then Rows'First (2) = 1
                 and then Rows'Length (1) = 2 * Frame'Length and then Result'First (1) = 1
                 and then Result'First (2) = 1 and then Result'Length (1) = Rows'Length (2)
                 and then Result'Length (2) = Rows'Length (2);
   --  The sum, over every pair of sightings and of their two errors, of the
   --  covariance of the pair times the outer product of the gradient rows
   --  (Rows, two a sighting, across then down): what the gradient of a fit
   --  spreads by, to be put between its inverse normal equations. Zero when
   --  M is not measured.

   procedure Free (M : in out Model);

   procedure Falling (Mean, Weight : Real_Array; Fitted : out Real_Array)
     with Pre => Mean'Length = Weight'Length and then Fitted'Length = Mean'Length
                 and then (for all W of Weight => W >= 0.0);
   --  The non-increasing sequence nearest Mean in the weighted least squares
   --  (isotonic regression), by pooling adjacent violators; a Mean with Weight 0
   --  has no value of its own and takes the level of the next one that has.

private

   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;

   type Model is record
      Is_Measured : Boolean := False;
      Bins        : Natural := 0;               --  distances 0 .. Bins pixels
      Persistent  : Matrix_Access;              --  (Pair_Kind, 0 .. Bins)
      Shared      : Matrix_Access;
      Same_Track  : Entries := [others => 0.0];
      Alone       : Entries := [others => 0.0];
   end record;

end Driver.Robot.Kinematics.Errors;
