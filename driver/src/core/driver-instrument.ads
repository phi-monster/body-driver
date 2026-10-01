--  The instrument: the service beside the driver that matches pixels between
--  images and segments things in them (docs/instrument-service.md).
--
--  Images travel as 24-bit BMP in base64 at their own size, or as the
--  numbers of frames stored on the service before. Every reply is checked
--  against what was asked (one answer per query point, run lengths adding up
--  to the image) and refused, never patched, when it does not fit. The
--  service's own confidence is reported but never used as a gate: what it
--  says is checked by geometry where it is used. Deciders call the blocking
--  forms; estimators submit a request and read its reply on a later beat,
--  so the main loop never waits (Driver.Services). Pixel coordinates are
--  those of Driver.Images, which are the service's.

with Ada.Strings.Unbounded;
with Driver.Clock;
with Driver.Images;
with Driver.Services;

package Driver.Instrument is

   use Ada.Strings.Unbounded;

   subtype Pixel is Driver.Images.Pixel;

   function Bitmap (I : Driver.Images.Image) return String;
   --  The image as a 24-bit BMP file, encoded in base64.

   --  Frames stored on the service (POST /frame).

   type Frame_Number is new Natural;

   procedure Store (I : Driver.Images.Image; Frame : out Frame_Number; Ok : out Boolean; Why : out Unbounded_String);

   --  Matching (POST /match): where points of one image are in another.

   type Source (Stored : Boolean := False) is record
      case Stored is
         when True  => Frame : Frame_Number;
         when False => Image : Driver.Images.Image;
      end case;
   end record;

   type Point_Array is array (Positive range <>) of Pixel;

   type Answer is record
      Found     : Boolean := False;   --  the service gave a finite answer
      To        : Pixel;              --  where the point is in the second image
      Back      : Pixel;              --  where To matches back to in the first, when asked
      Certainty : Real := 0.0;        --  the model's own value in [0, 1]; reported, never a gate
   end record;

   type Answer_Array is array (Positive range <>) of Answer;

   function Match_Request (A, B : Source; Points : Point_Array; Round_Trip : Boolean) return String;

   procedure Read_Match
     (Reply      : Driver.Services.Reply;
      Round_Trip : Boolean;
      Result     : out Answer_Array;
      Ok         : out Boolean;
      Why        : out Unbounded_String);
   --  Result has one element per query point of the request; Ok is False
   --  when the reply is an error or has another number of answers.

   procedure Match
     (A, B       : Source;
      Points     : Point_Array;
      Round_Trip : Boolean;
      Result     : out Answer_Array;
      Ok         : out Boolean;
      Why        : out Unbounded_String)
     with Pre => Result'Length = Points'Length;
   --  Blocking, for deciders.

   function Submit_Match
     (A, B : Source; Points : Point_Array; Round_Trip : Boolean; Beat : Driver.Clock.Beat)
      return Driver.Services.Ticket;
   --  For estimators: read the reply with Read_Match once it is ready.

   --  Segmentation (POST /segment): the pixels of the thing a box or points pick out.

   type Box is record
      X0, Y0, X1, Y1 : Real := 0.0;
   end record;

   type Prompt_Point is record
      At_Pixel : Pixel;
      On       : Boolean := True;   --  on the thing, or off it
   end record;

   type Prompt_Array is array (Positive range <>) of Prompt_Point;

   function Segment_Request (I : Driver.Images.Image; Has_Box : Boolean; Around : Box; Points : Prompt_Array)
     return String;

   procedure Read_Segment
     (Reply  : Driver.Services.Reply;
      Width  : Positive;
      Height : Positive;
      Region : out Driver.Images.Mask;
      Score  : out Real;
      Ok     : out Boolean;
      Why    : out Unbounded_String);
   --  Ok is False when the reply is an error, is for another image size, or
   --  its run lengths do not add up to the image.

   procedure Segment
     (I       : Driver.Images.Image;
      Has_Box : Boolean;
      Around  : Box;
      Points  : Prompt_Array;
      Region  : out Driver.Images.Mask;
      Score   : out Real;
      Ok      : out Boolean;
      Why     : out Unbounded_String);
   --  Blocking, for deciders.

   function Submit_Segment
     (I : Driver.Images.Image; Has_Box : Boolean; Around : Box; Points : Prompt_Array; Beat : Driver.Clock.Beat)
      return Driver.Services.Ticket;

end Driver.Instrument;
