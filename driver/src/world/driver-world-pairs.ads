--  Points two eyes see at the same instant.
--
--  The instrument matches pixels of one eye's image into the other's, with
--  round trips; the pixels around the region asked about are matched too,
--  and their round trips measure the matcher's own error. A region pixel's
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

   procedure Triangulate
     (First, Second : Driver.World.Cameras.Camera'Class;
      Points        : Driver.Instrument.Point_Array;
      Own           : Natural;
      Answers       : Driver.Instrument.Answer_Array;
      Kept          : out Match_Vectors.Vector;
      Apart         : out Natural)
     with Pre => Answers'Length = Points'Length and then Own <= Points'Length;
   --  Points are pixels of First, the region's Own first, the pixels around
   --  it after; Answers are the matcher's, into Second's image of the same
   --  instant. Apart counts the matches that came back but whose lines of
   --  sight do not meet. Nothing is kept when nothing around the region came
   --  back to tell the matcher's error by.

end Driver.World.Pairs;
