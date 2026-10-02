--  The readings of every group, channel by channel: the per-beat stream,
--  and how much a reading fluctuates at rest.
--
--  A reading is at rest at a beat when its target in effect did not change
--  since the previous beat. Its noise is the robust standard deviation of
--  its change from one rest beat to the next, divided by the square root of
--  two (the change of two independent readings): the beats where it still
--  settles after a push are a minority and do not move the median. A
--  reading that repeats exactly (a simulator, a quantized echo) has noise
--  zero, and then any change of it is significant.

private package Driver.Robot.Channels is

   procedure Append (M : in out Model; O : Observation; Sent : Driver.Commands.Command);
   --  Adds the beat to every group's stream, creating the streams at the first one.

   function Reading (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real
     with Pre => Has_Reading (M, G, Beat);

   function Has_Reading (M : Model; G : Group_Id; Beat : Natural) return Boolean;

   function Target (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real
     with Pre => Has_Target (M, G, Beat);

   function Has_Target (M : Model; G : Group_Id; Beat : Natural) return Boolean;

   function Change (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real
     with Pre => Beat > 0 and then Has_Reading (M, G, Beat) and then Has_Reading (M, G, Beat - 1);
   --  The reading at Beat minus the reading at Beat - 1.

   function Target_Changed (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   --  The target in effect at Beat differs from the one at Beat - 1 (or
   --  appeared or disappeared); False at the first beat.

   function Mad_Degrees_Of_Freedom (N : Natural) return Natural;
   --  How many degrees of freedom a sigma from the median absolute deviation
   --  of N Gaussian samples is worth: N times its asymptotic efficiency
   --  against the standard deviation, 8 c^2 phi (c)^2 with c the median of
   --  the absolute value of a unit Gaussian (about 0.37), and at least one
   --  from two samples on (a sigma from two values rests on one); none below.

   procedure Measure_Noise (M : in out Model);
   --  Re-measures the noise of every channel from the whole stream.

   function Noise (M : Model; G : Group_Id; Channel : Positive) return Real;
   --  Unknown (Real'Last) until measured.

   function Noise_Measured (M : Model; G : Group_Id) return Boolean;

   function Noise_Freedom (M : Model; G : Group_Id; Channel : Positive) return Natural;
   --  The degrees of freedom the channel's noise rests on (0: known or exact).
   --  Every channel of the group has its noise measured.

   function Visible (M : Model; G : Group_Id; D : Real_Array) return Boolean
     with Pre => D'First = 1 and then D'Length = Group_Size (M, G);
   --  A change D of the group's readings, one value per channel, is motion:
   --  a channel an eye watches changed by a step that eye can see
   --  (Visible_Step: below it no eye can tell, however a held reading
   --  jitters, and a joint held away from rest jitters far more than it was
   --  measured to at rest), or the change of the channels no eye watches is
   --  significant against their noise, as the change of two readings. The
   --  one test of motion.

   function Moving (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   --  The group moved at Beat: its change from Beat - 1 is Visible. False
   --  without a reading at Beat or Beat - 1, or before noise is measured.
   --  Push ends, the step tracker, stillness and keyframes all ask it.

   function Asked (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   --  A push starts at Beat: the target in effect changed and asks for
   --  motion, some channel's target differing significantly from the
   --  reading the group had at Beat - 1. A hold that repeats the reading or
   --  the last target asks for nothing.

   procedure Measure_Pushes (M : in out Model);
   --  Marks, for every commandable group, the beats at which it is being
   --  pushed: from a push's start while its reading has not moved yet (at
   --  most the longest response delay measured), and then as long as it keeps
   --  moving, overshoot included; the push ends at the first still beat after
   --  the reading moved, or where the next push starts. A group
   --  that moves without being pushed (a reaction to another group, sway)
   --  is not being pushed. Uses the noise, so Measure_Noise comes first.

   function Pushed (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   --  False before Measure_Pushes covered Beat.

   procedure Measure (M : in out Model);
   --  The noise and the pushes together: each needs the other, so they are
   --  measured in turn until the pushes found stop changing.

end Driver.Robot.Channels;
