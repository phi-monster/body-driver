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
--
--  The scene itself is measured at a still beat when nothing is being
--  looked for again: at the start of an episode, and again whenever a
--  thing's region changed, since what moved may have carried a surface with
--  it (the surfaces mostly of that thing's own points are dropped at once).
--  A grid of each eye's image, less the pixels showing the robot, is
--  matched into every other eye, and the supports are found among all the
--  points that gives (Driver.World.Supports), each pair's grid kept apart
--  from the others'. The grid is as many pixels apart as the square root of
--  the image's shorter side, so it has as many rows as each row has pixels
--  between points. A surface measured before in the episode stands where a
--  new measurement neither finds it again nor sees through it (a line of
--  sight crossing it to a point beyond): eyes that look elsewhere now say
--  nothing of it. A thing's points are each pair's latest, less those that
--  fall outside it now in an eye that holds it; an eye that lost it says
--  nothing either. A point of the scene that falls inside a thing's region
--  in an eye that holds it is the thing's own (on it, or hidden behind it),
--  so a thing never rests on its own top face; a thing's support is worked
--  out again only when its points or the surfaces change.

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Services;
with Driver.World.Cameras;
with Driver.World.Pairs;
with Driver.World.Supports;
with Driver.World.Tracking;

private package Driver.World.Estimates is

   type State is limited private;

   procedure Observe
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Up        : Direction_Estimate;
      Still     : Boolean;
      O         : Observation);
   --  One beat. Camera_Of gives an eye's camera at the beat of an
   --  observation: this one, or the one a reply arriving now was asked at;
   --  Up is the body's measured up.

   function Support_Of (S : State; T : Thing_Id) return Driver.World.Supports.Support;
   --  The support under the thing's points, and its height above it.

   function Resting_On (S : State; T : Thing_Id) return Surface_Id'Base;
   function Height_Above_Support (S : State; T : Thing_Id) return Estimate;
   --  As Driver.World's: no support and Unknown before one is found.

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
   function Surface_Of (S : State; F : Surface_Id) return Driver.World.Supports.Surface;

   function Scene_Round (S : State) return Natural;
   function Scene_Size (S : State) return Natural;
   function Scene_At (S : State; K : Positive) return Point_Estimate;
   function Scene_Grid_At (S : State; K : Positive) return Driver.World.Supports.Grid_Point;
   --  The latest measurement of the scene: which it was, and its points with
   --  their places in the grids they were asked on.

private

   package Point_Holders is new Ada.Containers.Indefinite_Holders (Driver.Instrument.Point_Array, Driver.Instrument."=");
   package Image_Holders is new Ada.Containers.Indefinite_Holders (Driver.Images.Image, Driver.Images."=");
   package Observation_Holders is new Ada.Containers.Indefinite_Holders (Observation, Driver.Observations."=");
   package Request_Holders is new Ada.Containers.Indefinite_Holders (String);

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
      Asked  : Request_Holders.Holder;   --  the request sent, as the instrument got it
   end record;

   package Start_Vectors is new Ada.Containers.Vectors (Positive, Start);
   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point_Estimate);

   --  What one pair of eyes last saw of a thing.
   type Pair_Seen is record
      From, Into : Eye_Id;
      Kept       : Driver.World.Pairs.Match_Vectors.Vector;
   end record;

   package Pair_Seen_Vectors is new Ada.Containers.Vectors (Positive, Pair_Seen);

   type Thing_Record is record
      Eyes      : Slot_Vectors.Vector;
      Crosses   : Cross_Vectors.Vector;
      Asked     : Asked_Vectors.Vector;
      Starts    : Start_Vectors.Vector;
      By_Pair   : Pair_Seen_Vectors.Vector;
      Points    : Driver.World.Pairs.Match_Vectors.Vector;   --  every pair's, inside it in the eyes holding it
      Points_In : Eye_Id := Eye_Id'First;
      Points_At : Driver.Clock.Beat := 0;
      Has_Points : Boolean := False;
      Friction  : Friction_Bounds;
      Touches   : Point_Vectors.Vector;
      Under     : Driver.World.Supports.Support;   --  its support, worked out again when its
      Under_Due : Boolean := True;                 --  points or the surfaces change
   end record;

   package Thing_Vectors is new Ada.Containers.Vectors (Thing_Id, Thing_Record);

   package Place_Vectors is new Ada.Containers.Vectors (Place_Id, Point_Estimate);

   --  A grid of one eye's image matched into another's at one still instant.
   type Background is record
      From, Into : Eye_Id;
      Ticket     : Driver.Services.Ticket;
      Points     : Point_Holders.Holder;
      Stride     : Positive := 1;
      Round      : Natural := 0;
   end record;

   package Background_Vectors is new Ada.Containers.Vectors (Positive, Background);

   --  A point of the scene: where both eyes saw it, and its place in the grid
   --  of the eye it was asked from.
   type Scene_Point is record
      Point      : Point_Estimate;
      Grid       : Driver.World.Supports.Grid_Point;
      From, Into : Eye_Id := Eye_Id'First;   --  the two eyes that saw it
   end record;

   package Scene_Point_Vectors is new Ada.Containers.Vectors (Positive, Scene_Point);

   type State is limited record
      Things      : Thing_Vectors.Vector;
      Surfaces    : Driver.World.Supports.Surface_Vectors.Vector;
      Scene       : Scene_Point_Vectors.Vector;   --  the points the surfaces were found among
      Scene_Round : Natural := 0;                 --  the measurement of the scene they came from
      Earlier     : Boolean := False;             --  the surfaces are an earlier episode's
      Due         : Boolean := True;              --  the scene is to be measured at the next still beat
      Round       : Natural := 0;                 --  the latest measurement of the scene asked
      Asking      : Background_Vectors.Vector;    --  its matches out now
      Incoming    : Scene_Point_Vectors.Vector;   --  and the points they gave so far
      Next_Column : Natural := 0;                 --  where the next pair's grid starts among them
      Seen_From   : Vec3 := Zero3;                --  where an eye that saw them stands
      Round_Seen  : Observation_Holders.Holder;   --  the observation they were asked at
      Up          : Direction_Estimate;
      Places      : Place_Vectors.Vector;
   end record;

end Driver.World.Estimates;
