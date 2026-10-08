--  Whether a push that asked the hand to go down took it down.
--
--  A press lowers the hand in steps, each planned as a translation of the tool
--  along the way down (Driver.Robot.Hand.Pressing.Lowered), so every point of
--  the hand is asked to go down by the same amount. The step tracker judges a
--  push by how far its group's readings came along their ask
--  (Driver.Robot.Steps), and that is not the same thing as how far the hand
--  went down. A17's third press lay its finger on the table and the arm went on
--  pushing for 4,600 beats: each push was followed by 92 per cent along its ask,
--  and 29 per cent of its length across the ask (joint 3 delivered a quarter of
--  its share while the joints beside it delivered all of theirs), so the hand
--  slid along the table and the point of it on the table went down by nothing;
--  every push was judged reached, and nothing ended the descent.
--
--  The readings tell where the hand's points are (Driver.Robot.Tool_In_Arm), so
--  a push is judged there: a point of the hand that was asked to go down by what
--  the tool's own noise can tell (Driver.Robot.Hand.Pressing.Least_Push) and
--  stopped short of where it was asked by as much, and by a larger share of
--  its ask than any push of the descent before it fell short by (more than Z
--  times), has stopped going down. Free pushes of one body fall short by one
--  share, however large the push (a joint held against gravity settles short of
--  its target by a share of the push), so the shares of the pushes before are
--  the measure of what a free push does, and a push is compared with them
--  once there are enough of them to have a scatter. The hand's points are the
--  tool's origin and the tips the hand has measured at the opening it stands
--  at; a hand turning about the finger on the table lowers its origin and
--  raises what lies beyond the finger, and that point stopped. Nothing is
--  compared with a stall among the first pushes (they would be taken for what
--  free pushes do), and the pushes before are forgotten when a push asks
--  nothing down (a retreat, a hold), so that a descent is compared with itself.
--
--  Plain geometry on poses; the models are read by Driver.Robot.Hand.

package Driver.Robot.Hand.Lowering is

   type Points is array (Positive range <>) of Vec3;
   --  Points of the tool frame.

   type Track is private;
   --  The pushes of one descent so far, as many as were not stalls, and the
   --  largest share of its ask any of them fell short by.

   type Verdict is
     (Not_Asked,   --  no point was asked to go down by what the tool's noise can tell
      Too_Few,     --  asked, and fewer pushes before it than a scatter of shares needs
      Lowered,     --  asked, and the hand went down as far as free pushes do
      Stalled);    --  asked, and a point of the hand stopped going down

   type Judgment is record
      Result : Verdict := Not_Asked;
      Point  : Natural := 0;     --  the index in Where of the point the verdict rests on; 0 when there is none
      Asked  : Real := 0.0;      --  how far it was asked to go along Into
      Went   : Real := 0.0;      --  and went
      Share  : Real := 0.0;      --  the part of its ask it fell short by
      Free   : Real := 0.0;      --  the largest share the pushes before it fell short by
   end record;

   procedure Judge
     (T      : in out Track;
      From   : Pose_Estimate;
      Target : Pose_Estimate;
      To     : Pose_Estimate;
      Into   : Vec3;
      Where  : Points;
      Least  : Real;
      Said   : out Judgment);
   --  One push of the arm that began with the tool at From, was asked to take
   --  it to Target and left it at To; Into the unit way down; Least the smallest
   --  move of the tool its noise tells (Pressing.Least_Push). The point the
   --  verdict rests on is the one of those asked to go down that fell short by
   --  the largest share; a stalled push is not counted among the free ones.

   function Pushes (T : Track) return Natural;
   --  How many pushes the track has counted.

private

   type Track is record
      Count   : Natural := 0;
      Largest : Real := 0.0;
   end record;

end Driver.Robot.Hand.Lowering;
