--  One closer group's measurement in its arm's own eye: the views of its
--  travel and the lobes each of its channels moves.
--
--  Fed beat by beat with what the body says (still or not, the closer's and
--  the arm's readings, the eye's image), it keeps the two ends of every
--  channel's travel (Driver.Robot.Hand.Views). Once both ends of a channel
--  are seen it asks for the matcher's correspondences between them, both
--  ways round with round trips, for every pixel of the box the change
--  between the ends covers: the changed pixels are the candidates, the
--  unchanged ones measure the matcher's own noise. Given the replies it
--  finds the lobes and which end is closed (Driver.Robot.Hand.Lobes). The
--  requests and replies are plain data, so the same logic runs live, on a
--  recording and in a test; the caller talks to the instrument.

with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Robot.Hand.Lobes;
with Driver.Robot.Hand.Views;

private with Ada.Containers.Indefinite_Holders;

package Driver.Robot.Hand.Sweep is

   type Progress is (Waiting, Requested, Unanswered, Nothing_Moves, Measured);
   --  Waiting        the channel's two ends have not both been seen still
   --  Requested      the correspondences between its ends are being asked for
   --  Unanswered     the instrument could not answer for these ends
   --  Nothing_Moves  nothing in this eye moves between its ends
   --  Measured       its lobes are found

   type State is private;

   function Start (Width, Height : Positive; Channels : Positive; Closer_Noise, Rest_Noise : Real_Array) return State
     with Pre => Closer_Noise'Length = Channels;
   --  Rest_Noise: the reading noise of every other group, in the order the
   --  readings are passed to Observe.

   procedure Observe
     (S      : in out State;
      Seen   : Observation;
      Still  : Boolean;
      Closer : Real_Array;
      Rest   : Real_Array;
      Image  : Driver.Images.Image);
   --  One beat: the closer's readings, every other group's, and the eye's image.

   function Channels (S : State) return Positive;

   function Status (S : State; Channel : Positive) return Progress;

   function Would_Extend (S : State; Channel : Positive) return Boolean;
   --  The view being gathered would extend the channel's travel (Views).

   function Wants_Correspondences (S : State; Channel : Positive) return Boolean;
   --  Both ends are seen, they differ, and nothing has been asked for them yet.

   function Low_End (S : State; Channel : Positive) return Views.View
     with Pre => Status (S, Channel) /= Waiting or else Wants_Correspondences (S, Channel);
   function High_End (S : State; Channel : Positive) return Views.View
     with Pre => Status (S, Channel) /= Waiting or else Wants_Correspondences (S, Channel);

   function Query_Points (S : State; Channel : Positive) return Driver.Instrument.Point_Array
     with Pre => Wants_Correspondences (S, Channel);
   --  The centres of every pixel of the box the change between the ends covers.

   procedure Asked (S : in out State; Channel : Positive)
     with Pre => Wants_Correspondences (S, Channel);
   --  The requests went out (Low_End to High_End and back).

   procedure Answer
     (S        : in out State;
      Channel  : Positive;
      Points   : Driver.Instrument.Point_Array;
      Forward  : Driver.Instrument.Answer_Array;   --  low end to high end
      Backward : Driver.Instrument.Answer_Array;   --  high end to low end
      Attached : Driver.Images.Mask)               --  the robot's own pixels in this eye (may be empty)
     with Pre => Status (S, Channel) = Requested
                 and then Forward'Length = Points'Length and then Backward'Length = Points'Length;
   --  Finds the channel's lobes from the replies.

   procedure Refuse (S : in out State; Channel : Positive)
     with Pre => Status (S, Channel) = Requested;
   --  The instrument could not answer: these ends are not asked for again,
   --  so a missing or failing service is not asked every beat; new ends are.

   function Lobes_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector
     with Pre => Status (S, Channel) = Measured;
   --  Here is the low end, There the high end.

   function Closed_End_Is_High (S : State; Channel : Positive) return Boolean
     with Pre => Status (S, Channel) = Measured;

   function Closing_Known (S : State; Channel : Positive) return Boolean
     with Pre => Status (S, Channel) = Measured;
   --  The lobes come significantly closer at one end than at the other.

private

   package Change_Holders is new Ada.Containers.Indefinite_Holders (Driver.Images.Mask, Driver.Images."=");

   type Channel_State is record
      Status      : Progress := Waiting;
      Low_From    : Driver.Clock.Beat := 0;   --  which views were analysed, to notice new ends
      High_From   : Driver.Clock.Beat := 0;
      Changed     : Change_Holders.Holder;
      Lobes       : Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector;
      Closing     : Driver.Robot.Hand.Lobes.Closing := Driver.Robot.Hand.Lobes.Undecided;
   end record;

   type Channel_Array is array (Positive range <>) of Channel_State;
   package Channel_Holders is new Ada.Containers.Indefinite_Holders (Channel_Array);

   type State is record
      Width, Height : Natural := 0;
      Views         : Driver.Robot.Hand.Views.Tracker;
      Per_Channel   : Channel_Holders.Holder;
   end record;

end Driver.Robot.Hand.Sweep;
