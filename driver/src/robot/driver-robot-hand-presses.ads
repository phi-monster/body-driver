--  The presses an arm made, found in the stream.
--
--  A press is the arm blocked (commanded further than it went) and then at
--  rest once the push that drove it in has let go. The tool's pose at that
--  rest is the press's: the hand on what it pressed, with nothing pushing it
--  in. The pose at the block itself is the hand under the push, and the push
--  is not the same twice: the last step of a descent that doubles with
--  nothing to stop it can drive the hand in as far as the whole descent
--  before it was long, and the arm gives under that by as much as it is
--  driven. A16's first press eased back 0.52 mrad over the 83 beats after the
--  let-go: its line of sight met the table 0.6 mm nearer at the block than at
--  the rest, and the rest was the pose within 0.3 mm of the truth.
--
--  The push has let go when another begins, or when another command takes
--  effect for the arm (a hold at the readings the block left may ask the arm
--  nothing the step tracker sees, and begin no push, and is a let-go all the
--  same): a verdict of Blocked stands
--  until the next push starts, whatever that push is. A second command that
--  takes effect before the hand has rested is the retreat's: the rest that
--  follows is the aim's, where the hand is not on what it pressed, and no press
--  is found (A35's first press, a tip 11.4457 from the eye for the 3.8 it was:
--  the hold began no push, the arm was still easing back, and the retreat was
--  the next command; the watcher took the rest at the aim). The rest is the
--  first beat after that push has ended at which the arm's own readings are still (the
--  caller says so), not the body's: the eyes' pictures lag the arm and settle
--  after it, and the arm's next move does not wait for them (A16: the retreat
--  began three beats after the let-go's rest in two presses of three, with
--  the eyes still settling).
--  Nothing is read from the verdict of the let-go itself, which asks nothing
--  and was judged Blocked for the arm easing back (A16, 83 beats, 0.52 mrad
--  against a visible step of a few millionths): after a press the watcher
--  settles, and is free again only when the arm is at rest with no block in
--  its verdict, so that verdict cannot begin a press of its own.
--
--  The direction it was pressing is the way the tool moved from where it last
--  stood still before the block, in the tool's frame at the block. Any
--  driver's run gives the same presses, since only the stream decides them.
--  The poses are in the frame they are given in, which for a hand is its
--  arm's own; a press keeps the arm's readings too, which the pose is a
--  function of, since an arm's frame and unit are what its fit makes them and
--  move when it is fitted again.
--
--  A command that takes the tool back nearer where it stood still before the
--  push than where the block left it is no let-go: the hand left what it
--  pressed before it rested on it, and no press is found. A let-go holds the
--  arm where the block left it. A descent whose step met something further
--  past where nothing was than the least push the tool tells takes the hand
--  back before it rests, and halves the span the contact lies in, so that the
--  press it lets go of is a light one (Driver.Robot.Hand.Descend): those moves
--  back are retreats, not presses.

with Ada.Containers.Indefinite_Holders;
with Driver.Clock;

package Driver.Robot.Hand.Presses is

   package Reading_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Event is record
      Tool     : Pose_Estimate;              --  at rest after the let-go, in the arm's own frame
      Arm      : Reading_Holders.Holder;     --  the arm's readings then
      Approach : Direction_Estimate;         --  tool frame; unknown when the tool had not moved
      Closer   : Reading_Holders.Holder;     --  the closer group's readings then
      Before   : Reading_Holders.Holder;     --  and where the tool last stood still before the push; empty when it had not
      Beat     : Driver.Clock.Beat := 0;
   end record;
   --  What the press met can push the closer off where it stood: a jaw held open by its servo, its tip on the table,
   --  is pressed shut by the descent the table stops (f16y on 10-16: the jaw, held open, stood 3.6 % of its
   --  travel shut at the beat its tip met the table, and 2 % at the rest of the press, held there by the arm's weight
   --  on it). Before is where it stood as the press began.

   type Watcher is private;

   procedure Observe
     (W       : in out Watcher;
      Beat    : Driver.Clock.Beat;
      Blocked : Boolean;
      Pushing : Boolean;
      Still   : Boolean;
      Tool    : Pose_Estimate;
      Arm     : Real_Array;
      Closer  : Real_Array;
      Found   : out Boolean;
      Press   : out Event;
      Retargeted : Boolean := False;
      Asked   : Pose_Estimate := (others => <>));
   --  One beat of one arm, Arm its readings; Blocked the verdict on its
   --  latest push, Pushing that push still under way, Still its own readings
   --  at rest, Retargeted a new command for the arm took effect at the beat
   --  (Driver.Robot.Channels.Target_Changed), Asked the tool where the readings
   --  that command asks for would put it (unknown when not given). Found is
   --  True at the beat a press ends.

   procedure Met (W : in out Watcher; Tool : Pose_Estimate);
   --  What a press met, felt by the closer before the arm's push was judged blocked, taken for the block, the tool
   --  at Tool: the hand measure stops a press the moment what its lobe meets pushes the closer off the opening the
   --  press is made at (Measure's Closer_Left), long before the arm's readings stall, and the push then ends
   --  unblocked. The watcher saw no block, so no let-go and no rest: a20 on 10-16 waited 7041 beats, the stream's
   --  length, after one such press, and some 10,000 after another. Nothing when the watcher is not free.

private

   type Phase is (Free, Driven, Let_Go, Settling);
   --  Free      no press: where the tool last stood still is kept
   --  Driven    the arm was blocked; its push has not let go
   --  Let_Go    another push began; the first rest is the press
   --  Settling  the press was found; free again at the next rest unblocked

   type Watcher is record
      State      : Phase := Free;
      Stood      : Boolean := False;   --  a still pose is known
      Last_Still : Pose_Estimate;      --  where the tool last stood still while free
      Closer_Still : Reading_Holders.Holder;   --  the closer group's readings there
      Approach   : Direction_Estimate; --  measured at the first blocked beat
      Commands   : Natural := 0;       --  the new commands that took effect since the block
      Block_At   : Pose_Estimate;      --  the tool at the first blocked beat
   end record;

end Driver.Robot.Hand.Presses;
