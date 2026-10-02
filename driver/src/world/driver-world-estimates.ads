--  The scene's estimates and how each beat updates them: the world layer's
--  estimator, behind Driver.World.
--
--  It takes the eyes as a provider of cameras (Driver.World.Cameras), so it
--  runs the same on the body's measured eyes and on a self test's pinholes.
--  Every beat it feeds each thing's track in each eye; asks the instrument
--  for what the tracks want; matches each thing from every eye that sees it
--  into every other eye at the same instant, keeping the points both see
--  (Driver.World.Pairs) as the thing's samples and starting a track in an
--  eye that had none, segmented around where its pixels went; and reads
--  every reply on a later beat.

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Services;
with Driver.World.Cameras;
with Driver.World.Pairs;
with Driver.World.Tracking;

private package Driver.World.Estimates is

   type State is limited private;

   procedure Observe
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Still     : Boolean;
      O         : Observation);
   --  One beat. Camera_Of gives an eye's camera at the beat of an
   --  observation: this one, or the one a reply arriving now was asked at.

   procedure Adopt (S : in out State; E : Eye_Id; O : Observation; Region : Driver.Images.Mask; Thing : out Thing_Id);
   procedure New_Episode (S : in out State);

   function Thing_Count (S : State) return Natural;
   function Seen_In (S : State; T : Thing_Id; E : Eye_Id) return Boolean;
   function Region_In (S : State; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask;

   function Points_Of (S : State; T : Thing_Id) return Driver.World.Pairs.Match_Vectors.Vector;
   --  The latest points two eyes saw of it, with the pixels of the first eye
   --  they were seen at.

   function Points_Eye (S : State; T : Thing_Id) return Eye_Id;
   --  The eye those pixels are in.

   function Centre (S : State; T : Thing_Id) return Point_Estimate;

   procedure Touched (S : in out State; T : Thing_Id; Point : Point_Estimate);
   procedure Learn_Friction (S : in out State; T : Thing_Id; Bounds : Friction_Bounds);
   function Friction (S : State; T : Thing_Id) return Friction_Bounds;

   procedure Remember (S : in out State; Point : Point_Estimate; Place : out Place_Id);
   function Where (S : State; P : Place_Id) return Point_Estimate;
   function Place_Count (S : State) return Natural;

   function Surface_Count (S : State) return Natural;
   function Plane_Of (S : State; F : Surface_Id) return Driver.Geometry.Plane_Estimate;
   function Earlier (S : State; F : Surface_Id) return Boolean;

private

   package Point_Holders is new Ada.Containers.Indefinite_Holders (Driver.Instrument.Point_Array, Driver.Instrument."=");
   package Image_Holders is new Ada.Containers.Indefinite_Holders (Driver.Images.Image, Driver.Images."=");
   package Observation_Holders is new Ada.Containers.Indefinite_Holders (Observation, Driver.Observations."=");

   --  A thing in one eye: its track, and the instrument's request out for it.
   type Slot is record
      Has     : Boolean := False;
      Track   : Driver.World.Tracking.Track;
      Out_Now : Boolean := False;
      Ticket  : Driver.Services.Ticket;
      Points  : Point_Holders.Holder;   --  a match request's points
   end record;

   package Slot_Vectors is new Ada.Containers.Vectors (Eye_Id, Slot);

   --  A thing matched from one eye into another at one instant.
   type Cross is record
      From, Into : Eye_Id;
      Ticket     : Driver.Services.Ticket;
      Points     : Point_Holders.Holder;
      Own        : Natural := 0;
      Seen       : Observation_Holders.Holder;   --  the observation of that instant
      Inner      : Driver.Images.Pixel;           --  the region's inner point in From
   end record;

   package Cross_Vectors is new Ada.Containers.Vectors (Positive, Cross);

   --  The measurement of the From eye's region a pair was last asked about,
   --  so a pair is asked again only when that region was measured again.
   type Asked_Pair is record
      From, Into : Eye_Id;
      Measured   : Driver.Clock.Beat := 0;
   end record;

   package Asked_Vectors is new Ada.Containers.Vectors (Positive, Asked_Pair);

   --  A track being started in an eye that had none.
   type Start is record
      Eye    : Eye_Id;
      Ticket : Driver.Services.Ticket;
      On     : Image_Holders.Holder;
      Beat   : Driver.Clock.Beat := 0;
   end record;

   package Start_Vectors is new Ada.Containers.Vectors (Positive, Start);
   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point_Estimate);

   type Thing_Record is record
      Eyes      : Slot_Vectors.Vector;
      Crosses   : Cross_Vectors.Vector;
      Asked     : Asked_Vectors.Vector;
      Starts    : Start_Vectors.Vector;
      Points    : Driver.World.Pairs.Match_Vectors.Vector;
      Points_In : Eye_Id := Eye_Id'First;
      Points_At : Driver.Clock.Beat := 0;
      Has_Points : Boolean := False;
      Friction  : Friction_Bounds;
      Touches   : Point_Vectors.Vector;
   end record;

   package Thing_Vectors is new Ada.Containers.Vectors (Thing_Id, Thing_Record);

   type Surface_Record is record
      Plane   : Driver.Geometry.Plane_Estimate;
      Earlier : Boolean := False;
   end record;

   package Surface_Vectors is new Ada.Containers.Vectors (Surface_Id, Surface_Record);
   package Place_Vectors is new Ada.Containers.Vectors (Place_Id, Point_Estimate);

   type State is limited record
      Things   : Thing_Vectors.Vector;
      Surfaces : Surface_Vectors.Vector;
      Places   : Place_Vectors.Vector;
   end record;

end Driver.World.Estimates;
