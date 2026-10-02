--  Lobes: the parts of a hand one closer channel moves, as one eye sees
--  them, from two still views of that eye at two readings of the channel
--  while everything else holds still.
--
--  The instrument's matcher says, for each pixel of one view, where it is in
--  the other and where that point matches back to. A pixel of a moving part
--  lands elsewhere and comes back to where it started; a pixel the moving
--  part covers in the other view has no partner, and its round trip fails;
--  a pixel of the still background lands on itself. Pixels that are known
--  not to have moved measure the matcher's own noise, and every judgment here
--  is against that noise.
--
--  A lobe is a piece of the moving part that is separate from the others in
--  at least one of the two views: fingers apart at the open end are separate
--  lobes even when they touch at the closed end, where each pixel is given
--  to the lobe it came from. Nothing here assumes how many lobes a hand has.
--
--  A pixel moves when its displacement is significant by the single test;
--  in an image of thousands of pixels dozens of still ones pass it by
--  chance. Whether a lobe is there at all is one question asked of every
--  pixel tested, so a lobe must hold at least one pixel whose displacement
--  passes the gate of that whole family (Driver.Uncertain, Tests); its other
--  pixels need only pass the single test.
--
--  A lobe's tip is its pixel farthest along the lobe from where the lobe is
--  attached: the robot's own pixels that did not move, or the image border
--  it comes in from. A lobe attached to neither has no tip in that eye.

with Ada.Containers.Vectors;
with Driver.Images;

package Driver.Robot.Hand.Lobes is

   subtype Pixel is Driver.Images.Pixel;
   subtype Mask is Driver.Images.Mask;

   type Correspondence is record
      From    : Pixel;
      To      : Pixel;      --  where the matcher put From in the other view
      Back    : Pixel;      --  where To matched back to in the first view
      Matched : Boolean := False;
   end record;

   type Correspondence_Array is array (Positive range <>) of Correspondence;

   type Matcher_Noise is record
      Displacement      : Estimate;   --  sigma of one coordinate of a still pixel's displacement, in pixels
      Round_Trip        : Estimate;   --  sigma of one coordinate of a round trip's error, in pixels
      Displacement_Gate : Gate;       --  for the two-dimensional lengths, worked out once
      Round_Trip_Gate   : Gate;
   end record;

   function Noise_Of (Still : Correspondence_Array) return Matcher_Noise;
   --  From pixels known not to have moved (their intensity did not change):
   --  robust spreads of their displacements and of their round trips, so a
   --  few of them that did move cannot set the noise.

   function Moving (Matches : Correspondence_Array; Noise : Matcher_Noise; Width, Height : Positive) return Mask;
   --  The pixels whose match lands significantly away from them and comes
   --  back to them within the round trip's noise.

   type Lobe is record
      Here, There         : Mask;            --  its pixels in the first view and in the other
      Count_Here          : Natural := 0;
      Count_There         : Natural := 0;
      Centre_Here         : Pixel;
      Centre_There        : Pixel;
      Tip_Here, Tip_There : Pixel;
      Tip_Known_Here      : Boolean := False;
      Tip_Known_There     : Boolean := False;
   end record;

   package Lobe_Vectors is new Ada.Containers.Vectors (Positive, Lobe);

   function Find
     (Forward  : Correspondence_Array;   --  first view to the other
      Backward : Correspondence_Array;   --  other view to the first
      Noise    : Matcher_Noise;
      Attached : Mask;                   --  the robot's own pixels that did not move (may be empty)
      Width, Height : Positive) return Lobe_Vectors.Vector;
   --  The lobes, each with its pixels and tip in both views.

   type Closing is (Towards_There, Towards_Here, Undecided);
   --  Towards_There  the lobes are closer together (or to what they are
   --                 attached to) in the other view: it is the closed end
   --  Towards_Here   the first view is the closed end
   --  Undecided      no significant difference either way

   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Closing;
   --  Which view has the lobes closer to each other; with a single lobe,
   --  closer to the robot's still pixels it can close against.

end Driver.Robot.Hand.Lobes;
