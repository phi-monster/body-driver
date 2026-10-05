--  The presses an arm made, found in the stream.
--
--  A press is a run of beats at which the arm was blocked (commanded further
--  than it went), ended by the first beat at which the body is still again:
--  the tool's pose then is where the hand rests on what it pressed, after
--  the push that drove it in has let go. The direction it was pressing is
--  the way the tool moved from where it last stood still before the block,
--  in the tool's frame at the block. Any driver's run gives the same
--  presses, since only the stream decides them. The poses are in the frame
--  they are given in, which for a hand is its arm's own; a press keeps the
--  arm's readings too, which the pose is a function of, since an arm's frame
--  and unit are what its fit makes them and move when it is fitted again.

with Ada.Containers.Indefinite_Holders;
with Driver.Clock;

package Driver.Robot.Hand.Presses is

   package Reading_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Event is record
      Tool     : Pose_Estimate;              --  at rest after the block, in the arm's own frame
      Arm      : Reading_Holders.Holder;     --  the arm's readings then
      Approach : Direction_Estimate;         --  tool frame; unknown when the tool had not moved
      Closer   : Reading_Holders.Holder;     --  the closer group's readings then
      Beat     : Driver.Clock.Beat := 0;
   end record;

   type Watcher is private;

   procedure Observe
     (W       : in out Watcher;
      Beat    : Driver.Clock.Beat;
      Blocked : Boolean;
      Still   : Boolean;
      Tool    : Pose_Estimate;
      Arm     : Real_Array;
      Closer  : Real_Array;
      Found   : out Boolean;
      Press   : out Event);
   --  One beat of one arm, Arm its readings. Found is True at the beat a
   --  press ends.

private

   type Phase is (Free, Pressing);

   type Watcher is record
      State      : Phase := Free;
      Stood      : Boolean := False;   --  a still pose is known
      Last_Still : Pose_Estimate;      --  where the tool last stood still while free
      Approach   : Direction_Estimate; --  measured at the first blocked beat
   end record;

end Driver.Robot.Hand.Presses;
