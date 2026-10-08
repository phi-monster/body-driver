--  One closer group's measurement in its arm's own eye: the views of its
--  travel and the lobes each of its channels moves.
--
--  Fed beat by beat with what the body says (still or not, the closer's and
--  the arm's readings, the eye's image), it keeps the two ends of every
--  channel's travel (Driver.Robot.Hand.Views) and what the eye shows of the
--  robot itself as the arm moves (Driver.Robot.Hand.Selfsight). Once both ends
--  of a channel are seen it compares them (Driver.Pixels.Compare): the pixels
--  that changed are where a part was at one end and not at the other. It gives
--  them to the two ends by how the robot looks in this eye, and finds the lobes
--  and which end is closed (Driver.Robot.Hand.Lobes.From_Change). Nothing here
--  asks an instrument: the same logic runs live, on a recording and in a test.

with Driver.Clock;
with Driver.Images;
with Driver.Robot.Hand.Lobes;
with Driver.Robot.Hand.Selfsight;
with Driver.Robot.Hand.Views;

private with Ada.Containers.Indefinite_Holders;

package Driver.Robot.Hand.Sweep is

   type Progress is (Waiting, Nothing_Moves, Everything_Moves, Unlocated, Unplaced, Measured);
   --  Waiting           the channel's two ends have not both been seen still
   --  Nothing_Moves     nothing in this eye moves between its ends
   --  Everything_Moves  half of this eye's picture or more changes between its
   --                    ends: what moved cannot be told from what did not
   --  Unlocated         the eye has not been moved against its surroundings, by
   --                    the arm, at the readings of either end (the poses a
   --                    deviation needs are two)
   --  Unplaced          the pixels that changed could not be given to the two
   --                    ends and made lobes (Lobes.Placing says why)
   --  Measured          its lobes are found

   type State is private;

   function Start (Width, Height : Positive; Channels : Positive; Closer_Noise : Real_Array) return State
     with Pre => Closer_Noise'Length = Channels;

   procedure Observe
     (S          : in out State;
      Seen       : Observation;
      Still      : Boolean;
      Closer     : Real_Array;
      Rest       : Real_Array;
      Image      : Driver.Images.Image;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean);
   --  One beat: the closer's readings, every other group's, the eye's image,
   --  and the body's own test of whether the rest of it moved (Views).

   function Channels (S : State) return Positive;

   function Status (S : State; Channel : Positive) return Progress;

   function Would_Extend
     (S          : State;
      Channel    : Positive;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean) return Boolean;
   --  The view being gathered would extend the channel's travel (Views).

   function Gathered (S : State) return Boolean;
   --  The view being gathered can be judged (Views).

   function Poses (S : State; Key : Real_Array) return Natural;
   --  How many poses of the rest of the body the eye saw the closer's readings
   --  Key from, still (Selfsight.Poses): the two a deviation needs, and the
   --  more, the better it is.

   function Has_Ends (S : State; Channel : Positive) return Boolean;
   --  Both ends of the channel's travel are seen, still, each from two frames or more (Views).

   function Low_End (S : State; Channel : Positive) return Views.View
     with Pre => Has_Ends (S, Channel);
   function High_End (S : State; Channel : Positive) return Views.View
     with Pre => Has_Ends (S, Channel);

   function Lobes_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector
     with Pre => Status (S, Channel) = Measured;
   --  Here is the low end, There the high end.

   function Located_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Located;
   --  What the last placing of the channel's changed pixels made of them, for
   --  the log.

   function Closed_End_Is_High (S : State; Channel : Positive) return Boolean
     with Pre => Status (S, Channel) = Measured;

   function Closing_Known (S : State; Channel : Positive) return Boolean
     with Pre => Status (S, Channel) = Measured;
   --  The lobes come significantly closer at one end than at the other.

   function Unannounced (S : State; Channel : Positive) return Boolean;
   --  The channel's ends are new since the log last said what became of them.

   procedure Announce (S : in out State; Channel : Positive);
   --  The log has said it.

   function Account (S : State; Channel : Positive) return String;
   --  What became of the channel, in a sentence for the log: where it stands
   --  among the states above, and for a measured one how many lobes and which
   --  end is closed, or by how much, against what, the lobes' distances
   --  changed between the ends when that was not significant.

private

   type Channel_State is record
      Status      : Progress := Waiting;
      Low_From    : Driver.Clock.Beat := 0;   --  which views were analysed, to notice new ends
      High_From   : Driver.Clock.Beat := 0;
      Analysed    : Boolean := False;
      Poses_Low   : Natural := 0;             --  the poses of the arm the eye saw each end's readings from
      Poses_High  : Natural := 0;
      Changed     : Natural := 0;             --  the pixels that changed between the ends
      Spread      : Real := 0.0;              --  and the spread of the difference at one that did not
      Beyond      : Real := 0.0;
      Located     : Driver.Robot.Hand.Lobes.Located;
      Closing     : Driver.Robot.Hand.Lobes.Closing := Driver.Robot.Hand.Lobes.Undecided;
      Change      : Estimate;   --  how the lobes' distances changed between the ends (Lobes.Closing_Change)
      Announced   : Boolean := True;
   end record;

   type Channel_Array is array (Positive range <>) of Channel_State;
   package Channel_Holders is new Ada.Containers.Indefinite_Holders (Channel_Array);

   type State is record
      Width, Height : Natural := 0;
      Views         : Driver.Robot.Hand.Views.Tracker;
      Memory        : Driver.Robot.Hand.Selfsight.Memory;
      Per_Channel   : Channel_Holders.Holder;
   end record;

end Driver.Robot.Hand.Sweep;
