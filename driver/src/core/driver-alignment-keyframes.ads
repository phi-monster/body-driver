--  The keyframes of one reference picture, aligned one after another: each is predicted from the ones before it
--  (or from a model the caller has), the points asked of the reference are found in it by Driver.Alignment, and
--  its answers are kept as what the next keyframe is predicted from.
--
--  A keyframe is a picture taken with the body still, at readings that differ from the reference's by Change (per
--  channel). While nothing is known of how the readings move the view, the displacements of the points between
--  the reference and a keyframe are all that tells where they will be in the next: when the next keyframe's change
--  of readings is a multiple of an earlier one's (the body moved the same joint, by another amount), the points
--  have moved by about that multiple of what they moved then. Each keyframe keeps its displacements as a smooth
--  field (a quadratic of the place in the picture, fitted robustly: the view of a plane turned about a point is
--  nearly that, and what is not is the scatter the prediction states), so that the noise of one point's answer is
--  not multiplied by the multiple. A prediction states the scatter of the fields it rests on carried by the weights
--  it gave them; how far the part of an earlier change that is no multiple of the next can have moved the view (the
--  sizes of its entries times the pixels a unit of each channel moves the view); and the relative error the
--  earlier extrapolations showed beyond the uncertainty they stated. One earlier keyframe gives a field linear in
--  the amount of the change, two a quadratic in it, which follows a view that is not linear in the joint; of the
--  predictions from one earlier keyframe and from every two the one that states the least is used, and it is used
--  only if it states less than not having moved at all, the points having moved as far as the change can have moved
--  them (the new direction).
--
--  A change of readings that no channel's noise separates from none is the same pose: the points have not moved.
--
--  Measured on A22 and A23 (driver/tools/align_study chain): a prediction made so follows a sweep whose displacement
--  quadruples at each keyframe to about 16 px of displacement; beyond that the noise of the fields it rests on,
--  multiplied by the extrapolation, outgrows the window the aligner searches (Driver.Alignment.Largest_Window),
--  and the caller's own model must predict.

with Ada.Containers.Vectors;
with Driver.Images;

package Driver.Alignment.Keyframes is

   type Chain is private;
   --  The reference's pyramid, its points, and the keyframes aligned so far.

   type Point_Array is array (Positive range <>) of Driver.Images.Pixel;
   type Prediction_Array is array (Positive range <>) of Prediction;
   type Answer_Array is array (Positive range <>) of Answer;

   procedure Begin_Reference (C : out Chain; Reference : Driver.Images.Image; Points : Point_Array);
   --  Starts the chain from the reference picture and the points that will be asked of it; no keyframe yet.

   function Has_Reference (C : Chain) return Boolean;

   function Point_Count (C : Chain) return Natural;
   --  How many points are asked of every keyframe.

   function Keyframes (C : Chain) return Natural;
   --  How many keyframes have been aligned.

   type Source is (Same_Pose, Extrapolated, New_Direction, Supplied);
   --  Same_Pose: the change of readings is none the noise can tell. Extrapolated: from earlier keyframes whose
   --  changes are multiples of this one's, or near. New_Direction: nothing earlier tells more than that the points
   --  have moved as far as the change can have moved them. Supplied: by the caller.

   No_Offer : constant Prediction_Array (1 .. 0) := [];
   --  For a caller with no model of its own to offer.

   procedure Predict
     (C       : in out Chain;
      Change  : Driver.Real_Array;
      Noise   : Driver.Real_Array;
      Shift   : Driver.Real_Array;
      Offered : Prediction_Array;
      Into    : out Prediction_Array;
      From    : out Source)
     with Pre => Has_Reference (C) and then Change'Length = Noise'Length and then Change'Length = Shift'Length
                 and then Into'Length = Point_Count (C) and then (for all N of Noise => N < Real'Last)
                 and then (Offered'Length = 0 or else Offered'Length = Point_Count (C));
   --  The prediction of every point of the reference in a keyframe at that change of readings (the keyframe's
   --  from the reference's, per channel), from the keyframes before it. Noise is the standard deviation of one
   --  reading of each channel, Shift the pixels the view moves per reading unit of it (zero when not measured: a
   --  change that moves the view by no amount known is predicted to have moved it as far as the aligner searches).
   --  Offered is the caller's own prediction (a fitted model's), or none: it competes with the rest, and the one
   --  that states the least (the median of the standard deviations over the points) is used.
   --  Into has an element per point. The chain remembers the predictions of each order that it made, for the Add
   --  that follows.

   procedure Add
     (C       : in out Chain;
      Image   : Driver.Images.Image;
      Change  : Driver.Real_Array;
      Given   : Prediction_Array;
      Answers : out Answer_Array)
     with Pre => Has_Reference (C) and then Given'Length = Point_Count (C) and then Answers'Length = Given'Length;
   --  Aligns every point of the reference into Image from its prediction (the one Predict gave, or the caller's
   --  own), and keeps the answers as a keyframe the next is predicted from. The predictions of each order that
   --  Predict made are judged by the answers (how far each was off beyond what it stated), whichever one was used,
   --  and the relative error that showed is what the next use of that order allows for.

private

   use type Driver.Images.Pixel;

   Terms : constant := 6;
   --  The coefficients of a quadratic of the place: one, the two coordinates, their squares and their product.

   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);
   package Pixel_Vectors is new Ada.Containers.Vectors (Positive, Driver.Images.Pixel);

   type Field_Fit is record
      U, V  : Real_Array (1 .. Terms) := [others => 0.0];   --  the displacement as a quadratic of the place, per axis
      Valid : Boolean := False;
      Sigma : Real := 0.0;   --  the robust scale of what the field leaves of the points it was fitted to
   end record;

   Orders : constant := 2;
   --  An extrapolation is linear in the amount of the change (one earlier keyframe: order 1) or quadratic in it (two
   --  earlier keyframes: order 2).

   type Order_Errors is array (1 .. Orders) of Real;

   type Frame is record
      Change    : Real_Vectors.Vector;   --  of the readings from the reference's, per channel
      Field     : Field_Fit;
      Curvature : Order_Errors := [others => 0.0];
      --  Per order, how far a prediction of this keyframe of that order was off beyond what the fields it rested on
      --  carried, relative to the move, per unit of the distance the extrapolation went (the amount between the
      --  keyframe and its earlier keyframe, or the product of its two): the Taylor remainder of a smooth move,
      --  which grows with that distance. Zero where there was none of that order.
   end record;

   --  A prediction from the fields of earlier keyframes.
   type Candidate is record
      Valid         : Boolean := False;
      First, Second : Natural := 0;   --  the earlier keyframes; Second is zero for order 1
      W1, W2        : Real := 0.0;    --  the weights of their fields
      Carry         : Real := 0.0;    --  the standard deviation the fields carry into the prediction
      Curve         : Real := 0.0;    --  the curvature it allows for (the largest the keyframes along it showed)
      Dist          : Real := 0.0;    --  how far it extrapolates, in amounts of the change (squared for order 2)
      Relative      : Real := 0.0;    --  the relative error of the move it allows for: Curve times Dist
      Median        : Real := Real'Last;   --  the median over the points of the standard deviation it states
   end record;

   type Candidate_Array is array (1 .. Orders) of Candidate;

   package Frame_Vectors is new Ada.Containers.Vectors (Positive, Frame);

   type Chain is record
      Pictures : Pyramid;
      Width    : Natural := 0;
      Height   : Natural := 0;
      Points   : Pixel_Vectors.Vector;
      Frames   : Frame_Vectors.Vector;
      Latest   : Candidate_Array;   --  per order, the prediction that stated the least for the keyframe being added
   end record;

end Driver.Alignment.Keyframes;
