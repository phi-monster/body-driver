--  The still views an eye takes of a closer group's travel: the evidence
--  the lobes are found from.
--
--  A view gathers the frames taken while the body is still and neither the
--  closer's readings nor any other group's moved by more than their own
--  noise; a reading that moves starts a new view. For each channel of the
--  closer the tracker keeps the two views at the lowest and the highest
--  reading the channel was seen still at, among the views whose other
--  readings equal those of the view it compares with: the two ends of that
--  channel's travel, with the rest of the body, and so the background,
--  unchanged between them. Nothing here knows what a finger is; the readings
--  and images are plain data. Each view keeps the observation it began with,
--  so the body's geometry at that beat can be asked for later.

with Ada.Containers.Indefinite_Holders;
with Driver.Clock;
with Driver.Images;
with Driver.Pixels;

package Driver.Robot.Hand.Views is

   package Reading_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type View is record
      Closer    : Reading_Holders.Holder;   --  the closer's readings while it was taken
      Rest      : Reading_Holders.Holder;   --  every other group's readings
      Frames    : Driver.Pixels.View;       --  per-pixel statistics of the frames
      Last      : Driver.Images.Image;      --  the latest frame, for the matcher
      Seen      : Observation;              --  the observation of its first beat
      From, To  : Driver.Clock.Beat := 0;
   end record;

   function Reading (V : View; Channel : Positive) return Real;
   --  The closer channel's reading while the view was taken.

   type Tracker is private;

   function Start (Width, Height : Positive; Closer_Noise, Rest_Noise : Real_Array) return Tracker;
   --  The noises are each reading's standard deviation at rest (path A
   --  measures them); a zero noise means the reading repeats exactly.

   procedure Observe
     (T      : in out Tracker;
      Seen   : Observation;
      Still  : Boolean;
      Closer : Real_Array;
      Rest   : Real_Array;
      Image  : Driver.Images.Image);
   --  One beat of the eye. Frames count only while the body is still.

   function Would_Extend (T : Tracker; Channel : Positive) return Boolean;
   --  The view being gathered, were it to end now, would extend one of the
   --  channel's ends: what a sweep asks before it pushes further.

   function Unseen_Travel (T : Tracker; Channel : Positive) return Boolean;
   --  The channel was seen still at significantly different readings, and
   --  the eye saw nothing change between them: its push moves nothing here.

   function Has_Ends (T : Tracker; Channel : Positive) return Boolean;
   --  Both ends of the channel's travel have a view of at least two frames,
   --  and the channel's reading differs significantly between them.

   function Low_End (T : Tracker; Channel : Positive) return View
     with Pre => Has_Ends (T, Channel);
   function High_End (T : Tracker; Channel : Positive) return View
     with Pre => Has_Ends (T, Channel);

   function Low_Beat (T : Tracker; Channel : Positive) return Driver.Clock.Beat
     with Pre => Has_Ends (T, Channel);
   function High_Beat (T : Tracker; Channel : Positive) return Driver.Clock.Beat
     with Pre => Has_Ends (T, Channel);
   --  The beat each end's view began: which views they are, without copying them.

private

   package View_Holders is new Ada.Containers.Indefinite_Holders (View);
   package Noise_Holders renames Reading_Holders;

   type End_Pair is record
      Low, High : View_Holders.Holder;
      Seen_Low  : Real := Real'Last;    --  the lowest and highest readings seen still,
      Seen_High : Real := Real'First;   --  each in a view of two frames or more
   end record;

   type End_Array is array (Positive range <>) of End_Pair;
   package End_Holders is new Ada.Containers.Indefinite_Holders (End_Array);

   type Tracker is record
      Width, Height : Natural := 0;
      Closer_Noise  : Noise_Holders.Holder;
      Rest_Noise    : Noise_Holders.Holder;
      Current       : View_Holders.Holder;   --  the view being gathered
      Ends          : End_Holders.Holder;    --  per channel
   end record;

end Driver.Robot.Hand.Views;
