--  One thing in one eye: the region it covers, kept for as long as the eye
--  sees nothing change on it, and found again when it does.
--
--  The region was measured on one image (adopted, or segmented by the
--  instrument). At every still beat the latest two frames, cut to a box
--  around the region (as far around it again as its own half-width), are
--  compared pixel by pixel with the frames it was measured on, against the
--  noise every pixel showed over those; when more of the region's pixels
--  changed than the test lets pass by chance, the region is lost. At the
--  next still beat (two frames after the change) its pixels are matched from
--  the image they were measured on into the latest one, both ways, together
--  with the pixels around it: their round trips measure the matcher's own
--  error, whether the thing, the eye or nothing moved, since a pixel the
--  matcher found again comes back whatever moved it. A region pixel came
--  back when its round trip is not significant against that error, and the
--  instrument segments the thing again around where the pixels that came
--  back went; with none coming back the thing is gone from this eye, to be
--  found in it again from elsewhere. The requests and replies are plain
--  data (the caller talks to the instrument), so the same logic runs live,
--  on a recording and in a test.

with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Pixels;

private with Ada.Containers.Indefinite_Holders;

package Driver.World.Tracking is

   subtype Mask is Driver.Images.Mask;
   subtype Image is Driver.Images.Image;

   type Phase is (Holding, Lost, Matching, Segmenting, Gone);
   --  Holding     the region shows what it showed when it was measured
   --  Lost        it changed; waiting for a still beat to look again
   --  Matching    its pixels are being matched into the latest image
   --  Segmenting  the thing is being segmented around where they went
   --  Gone        no pixel of it came back: it is not in this eye

   type Track is private;

   function Start (Region : Mask; On : Image; Beat : Driver.Clock.Beat) return Track
     with Pre => Driver.Images.Count (Region) > 0;
   --  A region measured on the image of that beat.

   procedure Observe (T : in out Track; Frame : Image; Beat : Driver.Clock.Beat; Still : Boolean);
   --  One beat of the eye; Still is the body's judgment for the beat.

   function State (T : Track) return Phase;

   function Seen (T : Track) return Boolean;
   --  Holding, and the latest beat was still.

   function Region (T : Track) return Mask;
   function Measured_At (T : Track) return Driver.Clock.Beat;
   function Measured_On (T : Track) return Image;
   --  The image the region was measured on.

   function Latest (T : Track) return Image;
   --  The image of the latest still beat.

   function Latest_Beat (T : Track) return Driver.Clock.Beat;

   --  Finding it again.

   function Wants_Match (T : Track) return Boolean;
   --  Lost, and the latest beat was still.

   function Match_Points (T : Track) return Driver.Instrument.Point_Array
     with Pre => Wants_Match (T);
   --  The centres of the region's pixels, in the image it was measured on,
   --  followed by those of the pixels around it.

   function Region_Points (T : Track) return Natural;
   --  How many of Match_Points are the region's own.

   procedure Asked_Match (T : in out Track)
     with Pre => Wants_Match (T);
   --  The points were submitted, from Measured_On to Latest, with round trips.

   procedure Matched (T : in out Track; Points : Driver.Instrument.Point_Array; Answers : Driver.Instrument.Answer_Array)
     with Pre => State (T) = Matching and then Answers'Length = Points'Length;

   function Wants_Segment (T : Track) return Boolean;

   procedure Segment_Prompt (T : Track; Around : out Driver.Instrument.Box; At_Point : out Driver.Instrument.Pixel)
     with Pre => Wants_Segment (T);
   --  The box of where the pixels that came back went, and where the
   --  region's own inner point went (or the nearest to it that came back).

   function Segment_On (T : Track) return Image
     with Pre => Wants_Segment (T);
   --  The image the prompts are in: the one the pixels were matched into.

   procedure Asked_Segment (T : in out Track)
     with Pre => Wants_Segment (T);
   --  The segmentation of Segment_On was submitted.

   procedure Segmented (T : in out Track; Found : Mask)
     with Pre => State (T) = Segmenting;
   --  The thing's region in the image the segmentation was asked on; an
   --  empty one means the instrument found nothing there.

   procedure Failed (T : in out Track)
     with Pre => State (T) in Matching | Segmenting;
   --  The instrument did not answer: the thing is gone from this eye until
   --  it is found in it again (from another eye, or adopted), so a failing
   --  service is not asked every still beat.

private

   package Image_Holders is new Ada.Containers.Indefinite_Holders (Image, Driver.Images."=");
   package Mask_Holders is new Ada.Containers.Indefinite_Holders (Mask, Driver.Images."=");

   type Track is record
      State          : Phase := Gone;
      Region         : Mask_Holders.Holder;
      Measured_Beat  : Driver.Clock.Beat := 0;
      Measured_Image : Image_Holders.Holder;
      Column_0, Row_0, Columns, Rows : Natural := 0;   --  the box around the region
      Own_Points     : Natural := 0;                   --  region pixels among the points asked
      Reference      : Driver.Pixels.View;             --  frames cut to the box since it was measured
      Older, Newer   : Image_Holders.Holder;           --  the latest two still frames, cut to the box
      Latest_Image   : Image_Holders.Holder;
      Latest_At      : Driver.Clock.Beat := 0;
      Was_Still      : Boolean := False;
      Asked_On       : Image_Holders.Holder;           --  the image the instrument was asked about
      Asked_At       : Driver.Clock.Beat := 0;
      Segment_Asked  : Boolean := False;
      Prompt_Box     : Driver.Instrument.Box;
      Prompt_Point   : Driver.Instrument.Pixel;
   end record;

end Driver.World.Tracking;
