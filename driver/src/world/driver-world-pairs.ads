--  Points two eyes see at the same instant.
--
--  The instrument matches pixels of one eye's image into the other's, with
--  round trips; the pixels around the region asked about are matched too,
--  and the round trips of the right ones among them measure the matcher's
--  own error (Matcher_Error tells the right from the wrong). A region pixel's
--  match is kept when its round trip is not significant against that error
--  and the two lines of sight, the second widened by the matcher's error in
--  its image, meet within their uncertainty; the point where they meet is
--  then a point of what both eyes see, with its covariance. Plain geometry
--  on the cameras and the matcher's replies: the caller talks to the
--  instrument.

with Ada.Containers.Vectors;
with Driver.Images;
with Driver.Instrument;
with Driver.World.Cameras;

package Driver.World.Pairs is

   type Match is record
      In_First, In_Second : Driver.Images.Pixel;
      Point               : Point_Estimate;   --  world frame
   end record;

   package Match_Vectors is new Ada.Containers.Vectors (Positive, Match);

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real)
     with Pre => Trips'Length mod 2 = 0 and then Area > 0.0;
   --  The matcher's error from round trips (each two coordinates, from where
   --  a pixel was to where its match matched back to). A right match comes
   --  back by a centred Gaussian error; a wrong one lands anywhere in the
   --  image, of that Area. The two are told apart by the mixture's maximum
   --  likelihood (EM, started from every doubling rank of the round trips'
   --  lengths, each run until its likelihood stops growing), not by a robust
   --  spread of them all: when the second eye does not see most of what is
   --  asked, the wrong ones are most of them, and their spread is no
   --  matcher's error. Sigma is one coordinate of a right match's round trip;
   --  Right, how many of the round trips the fit takes for right ones.

   procedure Triangulate
     (First, Second : Driver.World.Cameras.Camera'Class;
      Points        : Driver.Instrument.Point_Array;
      Own           : Natural;
      Answers       : Driver.Instrument.Answer_Array;
      Kept          : out Match_Vectors.Vector;
      Apart         : out Natural;
      Error         : out Real)
     with Pre => Answers'Length = Points'Length and then Own <= Points'Length;
   --  Points are pixels of First, the region's Own first, the pixels around
   --  it after; Answers are the matcher's, into Second's image of the same
   --  instant. The round trips of the pixels around measure the matcher's
   --  error; when none were asked (Own is every point: the background, say,
   --  which is itself what is asked about), those of all the points do.
   --  Error is the matcher's error measured, in pixels per coordinate of one
   --  match (Real'Last when nothing measured it). Apart counts the matches
   --  that came back but whose lines of sight do not meet. Nothing is kept
   --  when none of that sample came back to tell the matcher's error by.

end Driver.World.Pairs;
