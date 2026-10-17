--  The keyframes of one reference picture, aligned one after another: each is predicted from the ones before it
--  (or from a model the caller has), the points asked of the reference are found in it by Driver.Alignment, and
--  its answers are kept as what the next keyframe is predicted from.
--
--  A keyframe is a picture taken with the body still, at readings that differ from the reference's by Change (per
--  channel). While nothing is known of how the readings move the view, the displacements of the points between
--  the reference and a keyframe are all that tells where they will be in the next: when the next keyframe's change
--  of readings is a multiple of an earlier one's (the body moved the same joint, by another amount), the points
--  have moved by about that multiple of what they moved then. Each keyframe keeps its displacements as a smooth
--  field (a constant, an affine map or a quadratic of the place in the picture, whichever predicts its own points
--  best when each is left out in turn, fitted robustly: the view of a plane turned about a point is nearly a
--  quadratic, and what is not is the scatter the prediction states), so that the noise of one point's answer is
--  not multiplied by the multiple. A prediction states the scatter of the fields it rests on at the point asked,
--  carried by the weights it gave them (a field is the less known the farther the point lies from those it was
--  fitted to); how far the part of an earlier change that is no multiple of the next can have moved the view (the
--  sizes of its entries times the pixels a unit of each channel moves the view); and the relative error the
--  earlier extrapolations showed, judged at every point that stays in the picture and not only at those that were
--  answered, beyond the uncertainty they stated. A point the prediction did not place, or placed where the field of
--  the others does not agree, is asked again from the keyframe's own field. One earlier keyframe gives a field linear in
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
   type Flag_Array is array (Positive range <>) of Boolean;

   procedure Begin_Reference (C : out Chain; Reference : Driver.Images.Image; Points : Point_Array);
   --  Starts the chain from the reference picture and the points that will be asked of it; no keyframe yet.

   function Has_Reference (C : Chain) return Boolean;

   function Point_Count (C : Chain) return Natural;
   --  How many points are asked of every keyframe.

   function Keyframes (C : Chain) return Natural;
   --  How many keyframes have been aligned.

   function Scatter (C : Chain) return Real;
   --  How far the aligner errs here, in pixels: the median, over the keyframes predicted to be the reference again
   --  (the same pose: the reference's still twin) whose field has enough points, of the robust scale of what the
   --  smooth field of the displacements of their found points leaves of them; where no such keyframe has a field,
   --  the same over the keyframes that moved (there what the field leaves also holds what the scene's depth adds,
   --  and grows with the motion); zero when there is none.

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

   procedure Realign
     (C       : Chain;
      Image   : Driver.Images.Image;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array)
     with Pre => Has_Reference (C) and then Given'Length = Point_Count (C) and then Answers'Length = Given'Length
                 and then Agree'Length = Answers'Length;
   --  Aligns every point of the reference into Image from the given prediction, as Add does, and keeps nothing: the
   --  chain is not changed. For a keyframe aligned before from a prediction too uncertain to search, and now offered
   --  a better one. Agree is as Add's.

   procedure Forget_Last (C : in out Chain)
     with Pre => Keyframes (C) > 0;
   --  Drops the keyframe added last, as if Add had not been called for it: for a keyframe whose prediction was too
   --  uncertain to find anything, to be added again from another prediction (Kinematics.Align_Arm).

   function Prediction_Caused (A : Answer) return Boolean;
   --  The aligner's refusal may be the prediction's doing and not the point's: it found no place within what the
   --  prediction allowed, or several, or none that beats chance.

   procedure Agreement (C : Chain; Answers : Answer_Array; Agree : out Flag_Array)
     with Pre => Has_Reference (C) and then Answers'Length = Point_Count (C) and then Agree'Length = Answers'Length;
   --  Which of the answers found lie in a smooth field of displacements with the others (as Add says of a keyframe's).

   type Motion is record
      Told       : Boolean := False;   --  a field of displacements the points agree with, beyond any other
      Level      : Natural := 0;       --  the size of the pyramids it was told at, or tried at last
      Points     : Natural := 0;       --  the points that had places of their own to go to there
      Agree      : Natural := 0;       --  of them, the ones that have a place on the field
      Own, Other : Natural := 0;       --  on the field and not on the next best one, and the other way round
      Turn       : Real := 0.0;        --  how far the field turns the view, in radians
      Shift_U, Shift_V : Real := 0.0;  --  how far it moves the picture's centre, in pixels
      Sigma      : Real := 0.0;        --  how far the places of the points on it lie from it, in pixels
      Draws      : Natural := 0;       --  the pairs of places the fields were drawn from, over the sizes tried
      Unmoved    : Natural := 0;       --  the points whose best place was where they stood, which drew no field
      Searched   : Boolean := False;   --  it was searched: the keyframe's prediction had windows wider than a patch
   end record;
   --  The view's common motion a keyframe was asked from (Add): every keyframe whose prediction's windows are wider
   --  than a patch, a new direction's among them.

   procedure Stood_And_Moved (C : Chain; Stood, Moved : out Flag_Array)
     with Pre => Has_Reference (C) and then Stood'Length = Point_Count (C) and then Moved'Length = Stood'Length;
   --  Per point, of the view's common motion the keyframe added last was asked from: its best place was where it
   --  stood while the field told moved it there beyond what the size scanned resolves (Stood), or it moved, on the
   --  field (Moved); neither where no field was told or the point had no place there. A point of the world stands
   --  so only by a false peak: one that stands so again and again rides with the eye (Kinematics.Align_Arm).

   No_Riding : constant Flag_Array (1 .. 0) := [];

   procedure Add
     (C       : in out Chain;
      Image   : Driver.Images.Image;
      Change  : Driver.Real_Array;
      Given   : Prediction_Array;
      Answers : out Answer_Array;
      Agree   : out Flag_Array;
      Riding  : Flag_Array := No_Riding)
     with Pre => Has_Reference (C) and then Given'Length = Point_Count (C) and then Answers'Length = Given'Length
                 and then Agree'Length = Answers'Length
                 and then (Riding'Length = 0 or else Riding'Length = Point_Count (C));
   --  Aligns every point of the reference into Image from its prediction (the one Predict gave, or the caller's
   --  own), and keeps the answers as a keyframe the next is predicted from. Agree says which points the keyframe
   --  lies in a smooth field of displacements with (the displacements of the found points, a quadratic of the place,
   --  fitted to those that a robust affine field agrees with, then to those within Z robust scales of it): an
   --  answer that does not is a wrong answer more often than a right one, whatever it claims; none agree when there
   --  are too few found points for a field. The predictions of each order that
   --  Predict made are judged by the answers (how far each was off beyond what it stated), whichever one was used,
   --  and the relative error that showed is what the next use of that order allows for.
   --
   --  A keyframe Predict could only give a new direction (the points have moved as far as the aligner searches)
   --  is asked from the view's common motion instead: the field of displacements of a turned view that more of the
   --  points have places of their own on than on any other, significantly, at the coarsest size of the pyramids
   --  that tells it (Last_Motion). Searched alone over a window that wide, each point was refused or found where
   --  something that does not move lies (f15c on 10-15: the arm's base turned ten degrees, the view moved 90 px and
   --  turned with it, and every point searched at the 20 by 15 size was refused or found at no motion, where the
   --  arm's own hand and the grain of the table lie). When no field is told, no point is asked.
   --  Riding marks the points the caller knows to ride with the eye (its own body in its picture: they stay where
   --  they are whatever the view does): they tell nothing of the view's motion and are left out of it, though they
   --  are asked from it like the rest. Where they are most of the points, the field of no motion is theirs and
   --  wins (f15c on 10-15, arm 2: the wrist eye's picture mostly its own fingers, thumb and palm, the arm's base
   --  turned 44 degrees, the table moved 60 px, and the field the most points agreed on was no motion).

   function Last_Motion (C : Chain) return Motion;
   --  The view's common motion the keyframe added last was asked from; not Told when it was not asked so.

private

   use type Driver.Images.Pixel;

   Terms : constant := 6;
   --  The coefficients of a quadratic of the place: one, the two coordinates, their squares and their product.

   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);
   package Pixel_Vectors is new Ada.Containers.Vectors (Positive, Driver.Images.Pixel);
   package Flag_Vectors is new Ada.Containers.Vectors (Positive, Boolean);

   type Field_Fit is record
      U, V    : Real_Array (1 .. Terms) := [others => 0.0];   --  the displacement as a polynomial of the place, per axis
      Valid   : Boolean := False;
      Sigma   : Real := 0.0;   --  the robust scale of what the field leaves of the points it was fitted to
      Order   : Natural := 0;  --  the terms of the polynomial that are the field's (a constant, an affine map or all)
      Inverse : Real_Array (1 .. Terms * Terms) := [others => 0.0];
      --  the inverse of the normal matrix of those terms at the points it was fitted to, row by row: how far its
      --  coefficients are known, so how far it can be believed at a place those points are not at (Variance)
      Support : Natural := 0;  --  how many points it was fitted to
   end record;

   Orders : constant := 2;
   --  An extrapolation is linear in the amount of the change (one earlier keyframe: order 1) or quadratic in it (two
   --  earlier keyframes: order 2).

   type Order_Errors is array (1 .. Orders) of Real;

   type Frame is record
      Change    : Real_Vectors.Vector;   --  of the readings from the reference's, per channel
      Field     : Field_Fit;
      Moved     : Boolean := True;   --  it was not predicted to be the reference again (the same pose)
      Squared   : Order_Errors := [others => 0.0];
      Weight    : Order_Errors := [others => 0.0];
      --  Per order, what a prediction of this keyframe of that order says of how curved the move is: how far it was
      --  off beyond what the fields it rested on carried (and the keyframe's own field does not know), squared and
      --  relative to the squared move, per unit of the squared distance the extrapolation went (the amount between
      --  the keyframe and its earlier keyframe, or the product of its two): the Taylor remainder of a smooth move,
      --  which grows with that distance. An estimate with the noise of one measurement, so it may come out below
      --  zero; Weight is how far it can be believed: the square of how many times the noise it stands above at the
      --  median point (a move that is hardly more than the noise tells nothing of its curvature). Zero where there
      --  was no prediction of that order.
      Squared_Sum : Real := 0.0;
      Weight_Sum  : Real := 0.0;
      --  The same of a prediction of this keyframe as the sum of the moves of keyframes that changed one channel
      --  each (Combination), per unit of the squared sum of the amounts it changed the channels by.
   end record;

   --  A prediction from the fields of earlier keyframes.
   type Candidate is record
      Valid         : Boolean := False;
      First, Second : Natural := 0;   --  the earlier keyframes; Second is zero for order 1
      W1, W2        : Real := 0.0;    --  the weights of their fields
      Off1, Off2    : Real := 0.0;    --  how far at most the part of each keyframe's change off the direction moved
      Curve         : Real := 0.0;    --  the curvature it allows for (the largest the keyframes along it showed)
      Dist          : Real := 0.0;    --  how far it extrapolates, in amounts of the change (squared for order 2)
      Relative      : Real := 0.0;    --  the relative error of the move it allows for: Curve times Dist
      Median        : Real := Real'Last;   --  the median over the points of the standard deviation it states
   end record;

   type Candidate_Array is array (1 .. Orders) of Candidate;

   --  One channel's share of a combination: an earlier keyframe that changed that channel alone, and the multiple of
   --  its field that the change of the channel makes.
   type Part is record
      Frame  : Positive := 1;
      Weight : Real := 0.0;
      Off    : Real := 0.0;   --  how far at most the part of the keyframe's change off the channel moved the view
   end record;

   package Part_Vectors is new Ada.Containers.Vectors (Positive, Part);

   --  A prediction as the sum of the moves of earlier keyframes that changed one channel each, scaled to the amounts
   --  the new change has of them: a change of several channels at once is, to first order, the moves of each.
   type Combination is record
      Valid    : Boolean := False;
      Parts    : Part_Vectors.Vector;
      Amount   : Real := 0.0;   --  the sum of the sizes of the channels' changes
      Relative : Real := 0.0;   --  the relative error of the move it allows for: the curvature times the amount
      Median   : Real := Real'Last;   --  the median over the points of the standard deviation it states
   end record;

   package Frame_Vectors is new Ada.Containers.Vectors (Positive, Frame);

   type Chain is record
      Pictures : Pyramid;
      Width    : Natural := 0;
      Height   : Natural := 0;
      Points   : Pixel_Vectors.Vector;
      Frames   : Frame_Vectors.Vector;
      Latest   : Candidate_Array;   --  per order, the prediction that stated the least for the keyframe being added
      Combined : Combination;   --  the sum of single-channel moves considered for the keyframe being added
      Last     : Source := Supplied;   --  where the last prediction came from, until the keyframe it was for is added
      Motion   : Driver.Alignment.Keyframes.Motion;   --  the common motion the keyframe added last was asked from
      Stood, Moved_On : Flag_Vectors.Vector;   --  per point, of that motion (Stood_And_Moved)
   end record;

end Driver.Alignment.Keyframes;
