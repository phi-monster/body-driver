--  Per-pixel statistics of an eye's still views, and which pixels changed
--  between two views beyond what the pixels that did not change show.
--
--  A view is one or more frames of one eye taken while nothing it sees
--  moved. Every pixel's luma gets a running mean and variance over the
--  frames. An 8-bit pixel cannot be known better than its quantization, so
--  its variance is never taken below that of a uniform step of one level
--  (1/12 per frame). Nothing here knows which eye or body the frames come
--  from; whoever collects the views owns that.
--
--  Two views of one scene differ a little everywhere: a renderer's lighting
--  moves with whatever moves in it, a camera's gain drifts. In A14's eyes the
--  difference of two views' means was one to three levels at most pixels while
--  the frames within a view agreed to the quantization, so a test against the
--  frames' own variance called 85 % of the pixels changed. The null is
--  therefore measured from the difference itself (Compare), not taken from
--  the frames.

with Driver.Images;

private with Ada.Containers.Indefinite_Holders;

package Driver.Pixels is

   type View is private;
   --  Copies are independent.

   function Empty (Width, Height : Positive) return View;

   procedure Add (V : in out View; I : Driver.Images.Image)
     with Pre => Driver.Images.Width (I) = Width (V) and then Driver.Images.Height (I) = Height (V);

   function Width (V : View) return Natural;
   function Height (V : View) return Natural;
   function Frames (V : View) return Natural;

   function Mean (V : View; Column, Row : Natural) return Real
     with Pre => Frames (V) > 0 and then Column < Width (V) and then Row < Height (V);
   --  The pixel's mean luma, in levels.

   function Variance (V : View; Column, Row : Natural) return Real
     with Pre => Frames (V) > 0 and then Column < Width (V) and then Row < Height (V);
   --  The variance of one frame's luma at the pixel, never below the
   --  quantization floor; with one frame, the floor.

   procedure Means (V : View; Into : out Real_Array)
     with Pre => Frames (V) > 0 and then Into'Length = Width (V) * Height (V);
   procedure Variances (V : View; Into : out Real_Array)
     with Pre => Frames (V) > 0 and then Into'Length = Width (V) * Height (V);
   --  Mean and Variance of every pixel at once, row after row as
   --  Driver.Images.Luma lays them out: a frame's worth without a call per pixel.

   type Comparison is record
      Trusted : Boolean := False;
      --  Fewer than half the pixels changed, so the measurement below holds.
      Changed : Driver.Images.Mask;
      --  The pixels that changed; none unless Trusted.
      Spread  : Real := 0.0;
      --  The standard deviation, in levels, of the difference at one pixel
      --  that did not change.
      Beyond  : Real := 0.0;
      --  How far a pixel's difference must lie from the typical one, in
      --  levels, to count as changed.
   end record;

   function Compare (A, B : View) return Comparison
     with Pre => Width (A) = Width (B) and then Height (A) = Height (B) and then Frames (A) > 0 and then Frames (B) > 0;
   --  Which pixels changed between the views: those whose difference of means
   --  lies beyond the spread the pixels that did not change show.
   --
   --  The assumption is that most of the view does not change. Under it the
   --  difference at an unchanged pixel is spread around a typical value (its
   --  median: a lighting shift of the whole view is no change) and the
   --  spread is found as the median absolute deviation, then as the standard
   --  deviation of the pixels within the gate of that spread, until no more
   --  pixels drop out; it is never below what two means of 8-bit frames
   --  cannot know of each other. Each pixel is one test of a family of as
   --  many tests as the view has pixels (Driver.Uncertain, Tests), so a view
   --  that did not change passes for changed no oftener than one single
   --  test does.
   --
   --  When half the view or more is called changed, the assumption has failed
   --  (the median, which finds the typical difference, holds only while the
   --  unchanged pixels are the majority): nothing is known of which pixels
   --  changed, and the answer says so, with no pixel marked. A trusted
   --  answer is one consistent with the assumption; a view whose larger part
   --  changed is not told from a view whose smaller part did.

private

   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type View is record
      Width, Height : Natural := 0;
      Count         : Natural := 0;
      Means, Sums   : Real_Holders.Holder;   --  per pixel, row-major: the mean and the sum of squared deviations
   end record;

end Driver.Pixels;
