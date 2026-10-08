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
--  A push stopped when it fell short of its ask by more than free pushes do, in
--  something the body's own test of motion tells from none. Free pushes of one
--  body fall short by one share of their length, however long (a joint held
--  against gravity settles short of its target by a share of the push), so the
--  shares of the pushes of the descent before are the measure of what a free
--  push does, and a push is compared with them once there are enough to have a
--  scatter (three). It is measured twice, in what the readings say and at the
--  points of the hand, because either can be the first to show it:
--
--  The joints. How far the readings stopped from the push's target, the whole
--  vector and not only the part along the ask (the arm of a hand lying on a table
--  is pushed off the ask, and the part along it stays most of the way there),
--  as a share of the ask's length. Nothing of the arm's geometry is read.
--
--  The hand's points. The readings tell where they are (Driver.Robot.Tool_In_Arm),
--  and a point that stopped short of where it was asked has stopped when its
--  shortfall is a larger share of what the push asked of the hand (the most any
--  point was asked to go down: a push that turns the tool about its origin asks
--  the origin nothing, and a share of that is no number, as A17's aim gave one
--  of 1.6e9 and left the descent with nothing to compare a push with) than any
--  push of the descent before it fell short by. The points are the tool's origin
--  and the tips the hand has measured at the opening it stands at; a hand
--  turning about the finger on the table lowers its origin and raises what lies
--  beyond the finger, and that point stopped.
--
--  Either is a stall only when the push asked something the one test of motion
--  sees (Driver.Robot.Channels.Visible of its ask) and fell short of it by
--  something that test sees too (of the shortfall, the whole vector to the
--  target): the test is the body's own floor under both, since the tool's
--  uncertainty is that of a fit, common to the two poses a push is the difference
--  of, and says nothing of how little of a move is told. The pushes before are
--  forgotten when a push asks nothing down (a retreat, a hold), so that a descent
--  is compared with itself. A stall among the first three pushes is taken for
--  what free pushes do, and counted among them.
--
--  Plain arithmetic on poses and on the shortfalls; the models are read by
--  Driver.Robot.Hand.

package Driver.Robot.Hand.Lowering is

   type Points is array (Positive range <>) of Vec3;
   --  Points of the tool frame.

   type Track is private;
   --  The pushes of one descent so far, as many as were not stalls, and the
   --  largest share of its ask any of them fell short by, in the readings and
   --  at the points.

   type Verdict is
     (Not_Asked,   --  the push asked nothing the one test of motion sees, or nothing down
      Too_Few,     --  asked, and fewer pushes before it than a scatter of shares needs
      Lowered,     --  asked, and the hand went down as far as free pushes do
      Stalled);    --  asked, and the hand stopped going down

   type Joint_Push is record
      Asked  : Boolean := False;   --  what the push asked of the readings is a motion the one test of motion sees
      Length : Real := 0.0;        --  how far it asked, in reading units
      Short  : Real := 0.0;        --  how far short of its target the readings stopped, the whole vector
      Seen   : Boolean := False;   --  and that shortfall is a motion the one test of motion sees
   end record;

   type Judgment is record
      Result        : Verdict := Not_Asked;
      --  By the points of the hand:
      Point         : Natural := 0;     --  the index in Where of the point the verdict rests on; 0 when there is none
      Asked         : Real := 0.0;      --  how far it was asked to go along Into
      Went          : Real := 0.0;      --  and went
      Share         : Real := 0.0;      --  how far short it fell, as a share of what the push asked of the hand
      Free          : Real := 0.0;      --  the largest share the pushes before it fell short by
      Point_Stalled : Boolean := False; --  and that is a stall
      --  By the readings:
      Joint_Share   : Real := 0.0;      --  the share of the ask's length the readings stopped short of the target by
      Joint_Free    : Real := 0.0;      --  the largest share the pushes before it stopped short by
      Joint_Stalled : Boolean := False; --  and that is a stall
   end record;

   procedure Judge
     (T      : in out Track;
      From   : Pose_Estimate;
      Target : Pose_Estimate;
      To     : Pose_Estimate;
      Into   : Vec3;
      Where  : Points;
      Joints : Joint_Push;
      Said   : out Judgment);
   --  One push of the arm that began with the tool at From, was asked to take
   --  it to Target and left it at To; Into the unit way down; Joints what the
   --  readings did. The point the verdict rests on is the one of those asked
   --  to go down that fell short by the largest share; a stalled push is not
   --  counted among the free ones.

   function Pushes (T : Track) return Natural;
   --  How many pushes the track has counted.

private

   type Track is record
      Count         : Natural := 0;
      Largest       : Real := 0.0;
      Joint_Largest : Real := 0.0;
   end record;

end Driver.Robot.Hand.Lowering;
