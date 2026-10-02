--  Points two eyes see at the same instant.
--
--  The instrument matches pixels of one eye's image into the other's, with
--  round trips; the pixels around the region asked about are matched too.
--  A region pixel's match is kept when three tests pass, each against an
--  error measured on the matches themselves, the right ones told from the
--  wrong by a mixture (Matcher_Error):
--  - its round trip is not significant against the round trips of the
--    right matches;
--  - the two lines of sight, the second widened by the matcher's error
--    across the line the first draws in the second eye, meet within their
--    uncertainty. That error is measured from how far the matches' lines
--    pass each other: a matcher can be wrong yet come back, and only the
--    geometry tells;
--  - how far along the first sight they meet is significant: two sights
--    too near parallel meet anywhere along them, and the point is no place.
--  The point where they meet is then a point of what both eyes see, with
--  its covariance. A wrong match that lands on that line meets it whatever
--  it shows; no two eyes can tell those. Plain geometry on the cameras and
--  the matcher's replies: the caller talks to the instrument.

with Ada.Containers.Vectors;
with Driver.Images;
with Driver.Instrument;
with Driver.World.Cameras;

package Driver.World.Pairs is

   type Match is record
      In_First, In_Second : Driver.Images.Pixel;
      Point               : Point_Estimate;          --  world frame
      First               : Eye_Id := Eye_Id'First;   --  the eye In_First is a pixel of, as the caller says
   end record;

   package Match_Vectors is new Ada.Containers.Vectors (Positive, Match);

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real)
     with Pre => Trips'Length mod 2 = 0 and then Area > 0.0;
   --  The matcher's error from round trips (each two coordinates, from where
   --  a pixel was to where its match matched back to). A right match comes
   --  back by a centred Gaussian error; a wrong one lands anywhere in the
   --  image, of that Area. The two are told apart by the mixture's maximum
   --  likelihood (climbed from every doubling rank of the round trips'
   --  lengths until it stops growing), not by a robust
   --  spread of them all: when the second eye does not see most of what is
   --  asked, the wrong ones are most of them, and their spread is no
   --  matcher's error. The fit is then made again within its own gate, the
   --  wrong ones taken as spread over that window, until the window holds
   --  what it held: a matcher lost near what it was asked comes back near
   --  it, and over the whole image those look like a wide Gaussian that
   --  swallows the right ones. Sigma is one coordinate of a right match's
   --  round trip; Right, how many of the round trips the fit takes for right
   --  ones.

   procedure Triangulate
     (First, Second : Driver.World.Cameras.Camera'Class;
      Points        : Driver.Instrument.Point_Array;
      Own           : Natural;
      Answers       : Driver.Instrument.Answer_Array;
      Kept          : out Match_Vectors.Vector;
      Apart         : out Natural;
      Unplaced      : out Natural;
      Error         : out Real)
     with Pre => Answers'Length = Points'Length and then Own <= Points'Length;
   --  Points are pixels of First, the region's Own first, the pixels around
   --  it after; Answers are the matcher's, into Second's image of the same
   --  instant. The round trips of the pixels around measure the matcher's
   --  error; when none were asked (Own is every point: the background, say,
   --  which is itself what is asked about), those of all the points do.
   --  Error is the matcher's error measured across the lines the first sights
   --  draw in the second eye, in pixels (Real'Last when nothing measured it).
   --  Apart counts the matches that came back but whose lines of sight do not
   --  meet; Unplaced, those whose lines meet too far along them to tell how
   --  far. Nothing is kept when none of that sample came back to tell the
   --  matcher's error by.

private

   procedure Fit_Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural);
   --  One fit of that mixture, over the whole range: its maximum likelihood,
   --  climbed from every doubling rank of the errors' lengths.

   function Ball (Dimensions : Positive; Radius : Real) return Real;
   --  The measure of the errors in that many dimensions no longer than Radius.

   procedure Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural);
   --  The fit Matcher_Error and Triangulate make, of errors given by their
   --  squared lengths in that many dimensions, the wrong ones spread over a
   --  range of that Measure; Passes counts the times it went over them.

end Driver.World.Pairs;
