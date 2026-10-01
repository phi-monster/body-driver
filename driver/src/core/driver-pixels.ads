--  Per-pixel statistics of an eye's still views, and which pixels changed
--  between two views beyond their own noise.
--
--  A view is one or more frames of one eye taken while nothing it sees
--  moved. Every pixel's luma gets a running mean and variance over the
--  frames. An 8-bit pixel cannot be known better than its quantization, so
--  its variance is never taken below that of a uniform step of one level
--  (1/12 per frame). A pixel changed between two views when the difference
--  of its means is significant against the two means' uncertainties: Welch's
--  t through Driver.Uncertain, the variance a pixel showed resting on its
--  frames less one, the floor counting as known. Nothing here knows which
--  eye or body the frames come from; whoever collects the views owns that.

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

   function Changed (A, B : View) return Driver.Images.Mask
     with Pre => Width (A) = Width (B) and then Height (A) = Height (B) and then Frames (A) > 0 and then Frames (B) > 0;
   --  The pixels whose mean luma differs significantly between the views.
   --  A view of one frame has nothing but the floor to say how much it
   --  varies, which is the truth only for a noiseless camera; views of real
   --  cameras need two frames or more.

private

   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type View is record
      Width, Height : Natural := 0;
      Count         : Natural := 0;
      Means, Sums   : Real_Holders.Holder;   --  per pixel, row-major: the mean and the sum of squared deviations
   end record;

end Driver.Pixels;
