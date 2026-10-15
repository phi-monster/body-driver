--  A lobe followed along its closer's travel: the evidence its shape is fitted from (Driver.Robot.Hand.Shape),
--  taken from the pictures alone.
--
--  The still views the lobe's own eye took of the closer at every setting it stood at (Driver.Robot.Hand.Views:
--  the two ends of the travel and the stops between them), ordered from the open end to the closed one, are
--  compared only with their neighbours: each point of the lobe at the open end is found again in the next view by
--  the matcher (Driver.Alignment), from there in the next, and so on to the closed end. Between neighbours no point
--  of the lobe moves farther than its tip does (a lobe swung about its root, or slid, moves its tip the most), and a
--  window that wide is searched for each; where that is no wider than the matcher's patch, the patch is found again
--  where the lobe has barely turned. The two ends, a whole travel apart, are never compared: the instrument that
--  once matched them answered nothing inside smooth black fingers (A14) and was dropped on 10-06, and the shape was
--  fitted from nothing since, the hand's depth never measured and no grasper bound on any body.
--
--  A step wider than a patch is not followed: the matcher's answer there is not one to rest a fit on
--  (Driver.Alignment.Answer's Narrow). The decider stops the closer between such neighbours until none is
--  (Driver.Robot.Hand.Measure). A point is kept at a step when the matcher finds it there and the place found
--  matches back to where it came from within their joint uncertainty, at the driver's single gate; a point lost at
--  a step is not followed further. The points asked are the lobe's pixels at the open end on a grid as fine as the
--  matcher's half patch, each whose patch is more the lobe's than not: a patch mostly of the still background
--  beside the lobe is found where the background stands.
with Ada.Containers.Generic_Array_Sort;
with Ada.Strings.Unbounded;
with Driver.Alignment;
with Driver.Images;
with Driver.Robot.Hand.Lobes;

package Driver.Robot.Hand.Follow is

   type Pyramid_Array is array (Positive range <>) of Driver.Alignment.Pyramid;
   type Pixel_Array is array (Positive range <>) of Driver.Images.Pixel;

   type Followed is record
      Moves : Driver.Robot.Hand.Lobes.Move_Holders.Holder;   --  every point followed to the closed end, and where
      Count : Natural := 0;                                  --  it is there; how many
      Asked : Natural := 0;                                  --  the points asked at the open end
      Sigma : Real := Real'Last;   --  of one coordinate of a followed point at the closed end, pixels: the root of
                                   --  the median, over the points, of the variance it gathered step by step
      Fewest : Natural := 0;       --  the points still followed after the step that lost the most of them
      Worst  : Natural := 0;       --  that step (the index of its first view); 0 when none was lost
      Lost   : Ada.Strings.Unbounded.Unbounded_String;   --  why the points were lost at that step, for the log
      Lost_Count : Natural := 0;   --  how many were lost there
      Why   : Ada.Strings.Unbounded.Unbounded_String;   --  why nothing was followed; empty when something was
   end record;

   function Run
     (Views : Pyramid_Array;          --  the still views, from the open end to the closed one
      Tips  : Pixel_Array;            --  the lobe's tip in each
      Lobe  : Driver.Images.Mask)     --  the lobe's pixels at the open end
      return Followed
     with Pre => Views'Length >= 2 and then Tips'First = Views'First and then Tips'Last = Views'Last;

   function Widest_Step (Tips : Pixel_Array; Width, Height : Positive) return Natural;
   --  The first step, by the index of its first view, whose tip moves farther than the matcher searches where
   --  pictures of this size are finest (Driver.Alignment.Finest_Radius): a step it cannot follow. Zero when none does.

   function Tip_Step (Tips : Pixel_Array; K : Positive) return Real
     with Pre => K in Tips'First .. Tips'Last - 1;
   --  How far the tip moves from view K to the next, in pixels.

private

   procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Real, Real_Array);

end Driver.Robot.Hand.Follow;
