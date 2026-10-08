--  Lobes: the parts of a hand one closer channel moves, as one eye sees
--  them, from two still views of that eye at two readings of the channel
--  while everything else holds still.
--
--  The hand finds them from the change between the two views and from what
--  the eye's own motion shows of the robot (From_Change): the pixels that
--  changed are where a part was at one end and not at the other, and which
--  end each belongs to is told by the robot's look, not by the pictures'
--  texture, so that a smooth black finger is found as well as a textured one.
--  The matcher-based finding below (Find) is kept for the moves the shape is
--  fitted from; the hand does not call it.
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

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Driver.Images;
with Driver.Pixels;

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

   type Move is record
      From : Pixel;   --  a pixel of the lobe in one view
      To   : Pixel;   --  where the matcher put it in the other
   end record;

   type Move_Array is array (Positive range <>) of Move;

   package Move_Holders is new Ada.Containers.Indefinite_Holders (Move_Array);

   type Lobe is record
      Here, There         : Mask;            --  its pixels in the first view and in the other
      Count_Here          : Natural := 0;
      Count_There         : Natural := 0;
      Centre_Here         : Pixel;
      Centre_There        : Pixel;
      Tip_Here, Tip_There : Pixel;
      Tip_Known_Here      : Boolean := False;
      Tip_Known_There     : Boolean := False;
      Moves_Here          : Move_Holders.Holder;   --  every pixel of Here, with where it went in the other view
      Moves_There         : Move_Holders.Holder;   --  every pixel of There, with where it came from
      Tip_Move_Here       : Natural := 0;          --  the tip's own among them, 0 when it has none
      Tip_Move_There      : Natural := 0;
      Bordered_Here       : Boolean := False;      --  some pixel of it lies on the image's border
      Bordered_There      : Boolean := False;
   end record;
   --  The moves are the evidence for the lobe's shape (Driver.Robot.Hand.Shape):
   --  a pixel of the lobe and the same point of it in the other view.

   package Lobe_Vectors is new Ada.Containers.Vectors (Positive, Lobe);

   function Tip_Cap (Lobe_Mask : Mask; Centre, Tip : Pixel) return Mask;
   --  A lobe's tip region: its pixels within one mean width of the tip, along
   --  the way the tip lies from the lobe's centre; the mean width is the
   --  lobe's pixels over its length along that way. The point of a lobe that
   --  touches is somewhere in the region, not on the line through the tip
   --  pixel. Empty for a lobe whose tip is its centre.

   function Tip_Spread (Cap : Mask; Tip : Pixel) return Real;
   --  How far the pixels of a tip region lie from the tip pixel, per axis, in
   --  pixels (root mean square). Zero for an empty region.

   function Find
     (Forward  : Correspondence_Array;   --  first view to the other
      Backward : Correspondence_Array;   --  other view to the first
      Noise    : Matcher_Noise;
      Attached : Mask;                   --  the robot's own pixels that did not move (may be empty)
      Width, Height : Positive) return Lobe_Vectors.Vector;
   --  The lobes, each with its pixels and tip in both views.

   type Placing is (Placed, Nothing_Changed, Unseparated, One_Sided);
   --  Placed           the changed pixels were given to the two ends
   --  Nothing_Changed  no pixel changed
   --  Unseparated      the changed pixels do not fall in two groups by how
   --                   much they vary over the eye's poses: the poses did not
   --                   move the eye against its surroundings, or the robot
   --                   does not show in what changed
   --  One_Sided        what could be given to an end went to one end only, or
   --                   the parts it went to are attached to nothing, no larger
   --                   than the mixture's doubt, or fragments

   type Located is record
      How            : Placing := Nothing_Changed;
      Lobes          : Lobe_Vectors.Vector;
      Changed        : Natural := 0;   --  pixels that changed between the ends
      Seeds          : Natural := 0;   --  those the poses call the robot's at the anchored end
      Here, There    : Natural := 0;   --  those the robot's look gave to the first view, to the other
      Unassigned     : Natural := 0;   --  those it could not: left to neither end
      Cut            : Real := 0.0;    --  the deviation over the poses below which a changed pixel is a seed, levels
      Share          : Real := 0.0;    --  of the changed pixels' variance of log deviation the cut explains
      Rounds         : Natural := 0;   --  rounds the mixture took to settle (Give_To_Ends)
      Doubt          : Real := 0.0;    --  changed pixels expected to have been given to the wrong end: the mixture's own error
      Parts_Here     : Natural := 0;   --  parts of the pixels given to each end that are attached and larger than the doubt
      Parts_There    : Natural := 0;
   end record;

   function From_Change
     (Changed          : Mask;
      Here, There      : Driver.Pixels.View;
      Anchor           : Driver.Pixels.View;
      Anchored_At_Here : Boolean;
      Spread           : Real;
      Attached         : Mask) return Located
     with Pre => Driver.Images.Width (Changed) = Driver.Pixels.Width (Here)
                 and then Driver.Images.Height (Changed) = Driver.Pixels.Height (Here)
                 and then Driver.Pixels.Width (There) = Driver.Pixels.Width (Here)
                 and then Driver.Pixels.Height (There) = Driver.Pixels.Height (Here)
                 and then Driver.Pixels.Width (Anchor) = Driver.Pixels.Width (Here)
                 and then Driver.Pixels.Height (Anchor) = Driver.Pixels.Height (Here)
                 and then Driver.Pixels.Frames (Here) > 0 and then Driver.Pixels.Frames (There) > 0
                 and then Driver.Pixels.Frames (Anchor) >= 2;
   --  The lobes between two still views of an eye that rides on an arm, from
   --  the pixels that changed between them (Driver.Pixels.Compare), and the
   --  eye's views of the robot at the reading of the anchored end, over the
   --  poses of the arm (Driver.Robot.Hand.Selfsight).
   --
   --  A changed pixel is where a part was at one end and a part of the world
   --  at the other. Over the poses, the robot stays where it is in the eye's
   --  picture, and a pixel of the world varies; so among the changed pixels
   --  the ones that vary least are, as far as they go, the robot's at the
   --  anchored end. They are only seeds: the world that is flat or dark
   --  varies little too, and a glossy finger varies much. They say which
   --  end is which and begin a mixture of the two kinds of changed pixel
   --  (robot at the anchored end, robot at the other), told by how bright the
   --  robot and the world are where they changed and, last, by the labels of
   --  a pixel's neighbours (Give_To_Ends). A part of one label that the
   --  rest of the pixels explain better the other way round is turned. A
   --  pixel whose kind is not told to better than one test of Z alarms at is
   --  left unassigned, never guessed. Spread, that of the difference at a
   --  pixel that did not change, says whether the poses told anything: the
   --  world where it changed must vary over them by more than two views of it
   --  differ where it did not.
   --
   --  The lobes are made of the pixels given to each end by Lobes_Of_Sets.

   type Places is array (Positive range <>) of Natural;
   type Flags is array (Positive range <>) of Boolean;

   type Kind is (Neither, Anchored_End, Other_End);
   --  The end of a changed pixel's two ends that shows the robot: the anchored
   --  end, the other, or neither is told.
   type Kinds is array (Positive range <>) of Kind;

   procedure Tell_Ends
     (W, H     : Positive;
      At_Pixel : Places;    --  each changed pixel's place in the picture, row * W + column, in any order
      Seeds    : Flags;     --  those the poses call the robot's at the anchored end
      Anchored : Real_Array;   --  its brightness at the anchored end, in levels
      Other    : Real_Array;   --  and at the other
      Given    : out Kinds;
      Rounds   : out Natural;
      Doubt    : out Real)
     with Pre => At_Pixel'Length > 0 and then Seeds'Length = At_Pixel'Length and then Anchored'Length = At_Pixel'Length
                 and then Other'Length = At_Pixel'Length and then Given'Length = At_Pixel'Length;
   --  Each changed pixel shows the robot at the anchored end and the world at
   --  the other, or the robot at the other end and the world at the anchored:
   --  a mixture of two kinds of pixel, whose brightness at the two ends are
   --  the robot's and the world's in one order or in the other. A changed
   --  pixel has a brightness at each end (Anchored, Other, in whole levels),
   --  and the kinds are told by these: how bright the robot is and how bright
   --  the world is where it changed, each a histogram (every one begins with
   --  one pixel a bin), found together with every pixel's share in each kind
   --  by expectation and maximisation. The rounds begin, where the two ends
   --  differ most (the half of the pixels above their median difference),
   --  from which end shows the pixel darker: the robot is darker than the
   --  world where they changed, or lighter, the same way for most pixels of
   --  one hand, and the seeds say which. Elsewhere, and where the ends show
   --  one level, a pixel begins at its seed. The seeds, the pixels the poses
   --  call the robot's at the anchored end, are wrong where the world is
   --  flat, and histograms begun from them hold the world's grey as the
   --  robot's, where every group of pixels with a brightness of its own keeps
   --  the label its seeds gave it (A15's final ends: 69 % of the seeds right,
   --  and 47 % of the pixels given the end that shows them darker, against
   --  91 % begun from the order). Where two levels differ little their order
   --  is noise, which histograms begun from it sort into a robot and a world
   --  that are not there.
   --
   --  The seeds are heard in every round before the neighbours are, not only
   --  at the beginning: each kind has them at a rate of its own (the robot at
   --  the anchored end holds still in the eye's picture while the eye moves,
   --  the world does not), found with the histograms from the shares, and a
   --  pixel whose two brightnesses do not tell the kinds apart is told by
   --  whether the poses called it the robot's. Where the neighbours speak the
   --  seeds do not: a flat dark world is as still over the poses as the robot
   --  is, and where both ends are dark the seeds are a coin. Without them the
   --  histograms alone drift from the order of the levels to a labelling that
   --  explains itself: A16's final ends, a closed finger's lit face (10 177
   --  pixels of a grey a little darker than the table the other end shows
   --  there, hardly one a seed) went to the kind with the robot at the
   --  anchored end in nine rounds, 9 229 of them, and the lobes' tips were 60
   --  and 125 pixels low; with the seeds heard none went, and the tips are at
   --  the fingers' tips but one, 30 pixels low (53 % of the pixels the
   --  anchored end holds had seeds, 3 % of the other's). A15's final ends, the
   --  same way: three lobes of two fingers and closed tips 100 and 175 pixels
   --  low, then two lobes with their tips. When the rounds are over the seeds
   --  say which kind is which, as a whole: more of them in the kind with the
   --  robot at the anchored end than not, or the kinds are exchanged.
   --
   --  A flat patch is wrong as a whole and explains itself: robot and world
   --  lumas can be exchanged for it and the histograms follow. So when the
   --  rounds have settled (the labels they changed fewer than
   --  Unchanged_Fraction of the pixels), every connected part of one label is
   --  tried the other way round against the histograms of all the rest, and
   --  given the other label when that is the likelier by more than one test of
   --  Z tells apart (twice the log of the ratio above Z squared); the rounds
   --  begin again, until no part is turned or a pass turns no fewer than the
   --  one before.
   --
   --  Last the neighbours are heard: a pixel's eight, each with the label it
   --  has, the number of those of each kind a histogram too. The rounds end
   --  when the doubt, the expected error, has stopped changing by the
   --  convention for an iterative estimate (Unchanged_Fraction of itself, in
   --  each of two rounds in a row, or two rounds apart twice), or has gone
   --  round a cycle, or when a label has crossed the picture. A pixel is given
   --  to a kind only when the other kind's share of it is below what one test
   --  alarms at (Z), else to neither.

   function Lobes_Of_Sets
     (Here_Set, There_Set : Mask;
      Attached            : Mask;
      Doubt               : Real;
      Parts_Here          : out Natural;
      Parts_There         : out Natural) return Lobe_Vectors.Vector
     with Pre => Driver.Images.Width (Here_Set) = Driver.Images.Width (There_Set)
                 and then Driver.Images.Height (Here_Set) = Driver.Images.Height (There_Set);
   --  The lobes of the pixels given to each end (From_Change's last step).
   --  Each end's pixels are cleaned of what is one pixel across and split in
   --  their eight-connected parts. A part counts when it is attached (to the
   --  image's border or to Attached, the robot's own pixels that did not
   --  change) and holds more pixels than Doubt, the number of pixels expected
   --  to have been given to the wrong end: a smaller part could be made of
   --  nothing but those. Parts_Here and Parts_There are how many count at each
   --  end.
   --
   --  The lobes are the parts of the end that has the more of them (the first
   --  when equal): fingers that touch at one end are one part there and
   --  separate parts at the other. Each part of the other end goes to the
   --  lobe whose part is nearest to it (by their centres), and when that end
   --  has fewer parts than there are lobes, each of its pixels goes to the
   --  nearest lobe: the fingers that touched, each taking what is nearest.
   --  A lobe's tips are as Find gives them, every pixel of it having changed.

   type Closing is (Towards_There, Towards_Here, Undecided);
   --  Towards_There  the lobes are closer together (or to what they are
   --                 attached to) in the other view: it is the closed end
   --  Towards_Here   the first view is the closed end
   --  Undecided      no significant difference either way

   function Closing_Change (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Estimate;
   --  How far apart the lobes are in the other view less how far apart they
   --  are in the first, in pixels, summed over their pairs; with a single
   --  lobe, how far it is from the robot's still pixels it can close against.
   --  Its sigma is that of the lobes' centres, from the matcher's noise and
   --  the pixel grid. Unknown when there is nothing to compare.

   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Closing;
   --  Which view has the lobes closer to each other; with a single lobe,
   --  closer to the robot's still pixels it can close against: the sign of
   --  Closing_Change when it is significant.

   function Closing_Change (Lobes : Lobe_Vectors.Vector; Attached : Mask) return Estimate;
   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask) return Closing;
   --  The same for lobes of From_Change, whose pixels' places are known to
   --  the grid alone (a uniform square of side one) and not to a matcher.

end Driver.Robot.Hand.Lobes;
