--  How far a pressed finger slid, in its eye's picture.
--
--  The eye rides on the arm's last link and a finger rides on it too, along
--  one axis: what the closer's reading says is where the finger would stand
--  free. Under a press it stands elsewhere. A16's first lobe slid 5.1 mm
--  inward along its axis while its tip pressed on the table, another lobe
--  17.5 mm at a later press, and the closer read 1.0000 throughout: the
--  reading does not carry the slide, the eye does, since the free finger's
--  pixels, which stand still in the picture whatever the arm does, are
--  somewhere else.
--
--  What stays of a finger between the picture of the sweep, where it stood
--  free, and the picture under a press is its edge, not its pixels. A finger
--  of a glossy black is a grey of 10 in one picture and of 60 in the other
--  where the room it reflects has changed; the table beside it is 130 in one
--  and 40 in the other where the arm has moved and the light with it; a
--  finger that enters its picture from the side and is of one tone has
--  nothing in its pixels that says how far along its way it stands. The edge
--  it leads with, between it and what is beyond, is where its picture and
--  the free one agree: a step in luma of one sign, across the line the lobe's
--  mask ends at. A patch is the points of that line near the lobe's tip, each
--  with the way across it, outward, and which side is the darker.
--
--  Measure moves the points along the way the finger closes and scores each
--  shift by the step of luma the picture shows across them, of the free sign,
--  weighted by how nearly across the way they are: the points of a flank that
--  runs along the way tell nothing of how far along it the finger stands. The
--  shift of the greatest score is the slide, to a part of a pixel; it is known
--  when that score stands out of the scores at the other shifts as one of as
--  many tests; and it is as far from the truth as half the difference of the
--  two halves of the patch, taken across the way, put it from each other: a
--  part nearer the eye than another, or an edge the picture lights otherwise,
--  puts them apart.

with Ada.Containers.Indefinite_Holders;
with Driver.Images;
with Driver.Pixels;

package Driver.Robot.Hand.Slide is

   type Patch is private;
   --  Copies are independent.

   function Empty return Patch;

   function Take (Lobe, Region : Driver.Images.Mask; Luma : Real_Array; U, V, Reach : Real) return Patch
     with Pre => Driver.Images.Width (Lobe) = Driver.Images.Width (Region)
                 and then Driver.Images.Height (Lobe) = Driver.Images.Height (Region)
                 and then Luma'Length = Driver.Images.Width (Lobe) * Driver.Images.Height (Lobe);
   --  The points of the lobe's edge within the region: the pixels of the
   --  lobe in it with a pixel that is not the lobe's near them, each with the
   --  way to what is near and not the lobe's, and which side of the edge the
   --  darker is, in the picture Luma (laid out as Driver.Images.Luma does, row
   --  after row) the lobe was found in. (U, V) is the unit way the finger moves
   --  in that picture between its openings, Reach the most it can have moved,
   --  in pixels: the finger's travel between its openings. Where the score of
   --  the points is greatest in the picture they were taken from, the finger
   --  where it stood, is where the slide is none: the places the points are
   --  read at are a pixel or so off the edge, and a slide is measured from there.

   function Take (Lobe, Region : Driver.Images.Mask; View : Driver.Pixels.View; U, V, Reach : Real) return Patch
     with Pre => Driver.Pixels.Frames (View) > 0
                 and then Driver.Pixels.Width (View) = Driver.Images.Width (Lobe)
                 and then Driver.Pixels.Height (View) = Driver.Images.Height (Lobe)
                 and then Driver.Images.Width (Lobe) = Driver.Images.Width (Region)
                 and then Driver.Images.Height (Lobe) = Driver.Images.Height (Region);
   --  The same, in the mean luma of the view.

   function Points (P : Patch) return Natural;

   type Picture is private;
   --  The luma of a picture taken under a press. Copies are independent.

   function See (Image : Driver.Images.Image) return Picture
     with Pre => not Driver.Images.Is_Empty (Image);

   function See (Luma : Real_Array; Width, Height : Positive) return Picture
     with Pre => Luma'Length = Width * Height;

   function Score (P : Patch; Under : Picture; By : Real) return Real;
   --  The step of luma, in levels, the picture shows across the patch's points
   --  moved By pixels along its way, of the free sign, weighted by
   --  how nearly across the way each lies (the sum over them of the weight
   --  and the step, over the sum of the weights). Real'First when fewer than
   --  half of the weights stay in the picture.

   type Shift is record
      Known   : Boolean := False;   --  the greatest score stands out of the rest, and so do those of both halves
      By      : Real := 0.0;        --  pixels along the way given; positive is along it
      Sigma   : Real := Real'Last;  --  pixels
      Peak    : Real := 0.0;        --  the greatest score, levels
      Still   : Real := 0.0;        --  and the score with the patch where it was
      Typical : Real := 0.0;        --  and the spread of the scores at the shifts that are not near the greatest, levels
   end record;

   procedure Measure (P : Patch; Under : Picture; Result : out Shift);
   --  Every shift of whole pixels along the patch's way, as far as its Reach
   --  either side, is scored; the greatest is placed between its neighbours,
   --  and its distance from where the greatest was in the free picture is the
   --  slide.

private

   package Index_Holders is new Ada.Containers.Indefinite_Holders (Natural_Array);
   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Patch is record
      Column, Row : Index_Holders.Holder;   --  the points
      Across_U    : Real_Holders.Holder;    --  the way across the edge at each, outward: a unit vector
      Across_V    : Real_Holders.Holder;
      Dark_Inside : Boolean := True;        --  the lobe is the darker side of its edge
      U, V, Reach : Real := 0.0;            --  the way it was taken for, and how far
      Rest        : Real := 0.0;            --  and where the score was greatest in the picture it was taken from,
      Rest_Low, Rest_High : Real := 0.0;    --  and where the score of each of its halves was
   end record;

   type Picture is record
      Width, Height : Natural := 0;
      Luma          : Real_Holders.Holder;
   end record;

end Driver.Robot.Hand.Slide;
