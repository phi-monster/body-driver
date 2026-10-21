--  How far a thing may go into a hand, from pictures an eye off the hand took of it: the room in front of the hand
--  where those pictures see past it.
--
--  A hand's eye rides on the hand, and what it sees of the hand never moves in its picture: it cannot tell how deep
--  the room between the fingers goes. An eye that stands off the hand (a fixed eye placed against the arm) sees the
--  hand from outside, at poses the arm's fit knows: a point of the tool frame that some picture sees past (what lies
--  there in that picture is what lies behind the hand, the picture's backdrop) is no part of the hand; a point of the
--  hand is never seen past. The backdrop is each pixel's median over the pictures: the hand is shown at places far
--  enough apart that each pixel is clear of it in some of them. Nothing here asks how the hand is made, how many
--  fingers it has or how they move: only what the pictures see past. The way in is the hand's own axis as the
--  contact search takes it (Driver.Action.Contact.Search: from the tool frame's origin, the hand's eye, to the middle
--  of the tips), walked back from the middle of the tips toward that origin.
--
--  Where the shape followed through the closer's travel measured it (Driver.Robot.Hand.Shape), no depth was measured
--  in any boot: a jaw with no texture of its own, or ribbed along the way it closes, gives the matcher nothing to
--  follow (q15, servo seed 7 of the fake robot, 10-19: 0 of 178 points), and a jaw's shading changes all over as it
--  turns (a16, 10-16).

with Ada.Strings.Unbounded;
with Driver.Images;

package Driver.Robot.Hand.Hull is

   type View is record
      Picture       : Driver.Images.Image;
      Eye_From_Tool : Rigid := Identity;   --  a point of the tool frame into the frame of the eye that took the picture
      Blur          : Real := 0.0;         --  how far, in pixels, a point lands from where the fit puts it
      Hand_At       : Driver.Images.Pixel := (U => 0.0, V => 0.0);
      Hand_Radius   : Real := 0.0;         --  the hand lies within this of Hand_At, as far as the fit tells
   end record;
   --  A pixel's backdrop is its median over the pictures that do not have the hand within their Hand_Radius of it:
   --  the hand shown at one place over several turns covers the same pixels in all of them. A pixel every picture has
   --  the hand near has no backdrop, and is never seen past.
   type View_Array is array (Positive range <>) of View;

   type Projector is not null access procedure (P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the eye's frame lands in its picture.

   type Run is record
      Length  : Estimate;   --  how far the way is seen past, from From toward Toward; unknown when not at all
      Samples : Natural := 0;    --  the points of the way tested
      Seen    : Natural := 0;    --  of the last point tested, the pictures it lands in
      Past    : Natural := 0;    --  and those that see past it
      Why     : Ada.Strings.Unbounded.Unbounded_String;   --  why it is unknown, or what ended it
   end record;

   function Free_Run (Views : View_Array; Project : Projector; From, Toward : Vec3; Step : Real) return Run
     with Pre => Step > 0.0;
   --  From From toward Toward in steps of Step, every point of the way seen past by two pictures or more, as far as
   --  the first that is not. Two: in one picture alone something that stands still in front of the hand there (and
   --  so is the backdrop) would be taken for seeing past it. A picture sees past a point when no pixel some part of
   --  which lies within its Blur and half its step's footprint of where the point lands differs from the backdrop
   --  beyond what the picture's noise gives (the pixels tested together as one family, Driver.Uncertain.Scalar_Gate):
   --  the noise is the robust spread of the picture's differences from the backdrop, most of its pixels the backdrop
   --  itself. Every pixel the point may land on is tested, so the hand is taken larger than it is by as much as they
   --  reach past where it lands: the length falls short of the room, never past it (hand.hull.depth: 2.72 for 3, a
   --  pixel 0.18 there). The length is halfway between the last point seen past and the next, its sigma that of a
   --  spread even over the step. Unknown when From itself is not seen past (no picture sees between the tips), and
   --  when the whole way is (the hand is not met on it).

end Driver.Robot.Hand.Hull;
