--  The scene's estimates and how each beat updates them: the world layer's
--  estimator, behind Driver.World.
--
--  It takes the eyes as a provider of cameras (Driver.World.Cameras), so it
--  runs the same on the body's measured eyes and on a self test's pinholes.
--  Every beat it feeds each thing's track in each eye, keeps the eyes' still
--  pictures as views (Driver.World.Sight), and says what the eyes are to place
--  by themselves (Measure_Due); Measure places it, apart from the main loop.
--
--  The scene is measured at a still beat when nothing is being looked for
--  again: at the start of an episode, and again whenever a thing's region
--  changed, since what moved may have carried a surface with it (the surfaces
--  mostly of that thing's own points are dropped at once). A grid of the
--  picture of every eye that has views from more than one place, less the
--  pixels showing the robot, is placed from the views (Sight.Place), the table
--  among it, and the supports are found among all the points that gives
--  (Driver.World.Supports), each eye's grid kept apart from the others'. The
--  grid is as many pixels apart as the square root of the image's shorter
--  side, so it has as many rows as each row has pixels between points. A
--  surface measured before in the episode stands where a new measurement
--  neither finds it again nor sees through it (a line of sight crossing it to
--  a point beyond): eyes that look elsewhere now say nothing of it.
--
--  A thing's points are what the eyes that hold it place of their regions
--  (Sight.Place, at the pixels of a grid of each region a patch's half width
--  apart, those standing above the table): each region measured anew is placed
--  again, and its eye's points replace that eye's earlier ones. An eye that
--  holds no track of a thing whose points it sees starts one, segmented around
--  where the points fall in its picture. A point of the scene that falls
--  inside a thing's region in an eye that holds it is the thing's own (on it,
--  or hidden behind it), so a thing never rests on its own top face; a thing's
--  support is worked out again only when its points or the surfaces change.

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Driver.Clock;
with Driver.Images;
with Driver.Instrument;
with Driver.Services;
with Driver.World.Cameras;
with Driver.World.Points;
with Driver.World.Sight;
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
   function No_Gap_Seen (S : State; T : Thing_Id) return Boolean;
   --  As Driver.World's: no support and Unknown before one is found.

   function Measure_Due (S : State) return Boolean;
   --  The eyes have something to place by themselves (the scene, a thing's region measured anew) or a track lost at a
   --  still beat looks for its thing again: heavier than a beat allows, computed apart from the main loop
   --  (Driver.Apart) at the robot's estimates' turn.

   procedure Measure
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class);
   --  Does it, each part at the observation that made it due.

   procedure Keep_View
     (S         : in out State;
      E         : Eye_Id;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      O         : Observation);
   --  Eye E's picture at O, kept as a view the eyes place from (Driver.World.Sight.Keep_View): a still picture the
   --  world did not see at a still beat of its own, as one of an arm's sweep.

   function Scene_Pending (S : State) return Boolean;

   procedure Seen_Anew (S : in out State);
   --  As Driver.World.Seen_Anew: the scene is due.

   function Views_Of (S : State; E : Eye_Id) return Natural;

   function Free_Pixel (S : State; E : Eye_Id; Width, Height : Positive) return Driver.Images.Pixel;
   --  The middle of the eye's picture, or where its latest scene showed the robot riding with it there, the middle of
   --  the cell of its grid nearest it that did not: where a point the eye is turned to is to be seen.
   --  How many views of the eye are kept (Driver.World.Sight.Views_Of).

   procedure Adopt (S : in out State; E : Eye_Id; O : Observation; Region : Driver.Images.Mask; Thing : out Thing_Id);
   --  The region the eye shows the thing in at O, a track of it started; the points the eye places of it are placed
   --  apart at the next still beat it holds the thing at (Measure), unless the points are given (Place_Points).
   procedure New_Episode (S : in out State);

   procedure Boot_Begins (S : in out State);
   procedure Boot_Ends (S : in out State);
   --  As Driver.World's: while the boot is under way the scene is not measured.

   function Thing_Count (S : State) return Natural;
   function Seen_In (S : State; T : Thing_Id; E : Eye_Id) return Boolean;
   function Region_In (S : State; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask;

   function Points_Of (S : State; T : Thing_Id) return Driver.World.Points.Placed_Vectors.Vector;
   --  Its points, with the pixels of the eye that placed them.

   function Points_Eye (S : State; T : Thing_Id) return Eye_Id;

   package Normal_Vectors is new Ada.Containers.Vectors (Positive, Vec3, Driver.Numerics.Arrays."=");

   procedure Place_Points
     (S       : in out State;
      T       : Thing_Id;
      Points  : Driver.World.Points.Placed_Vectors.Vector;
      Normals : Normal_Vectors.Vector;
      Carried : Boolean := False)
     with Pre => Natural (Points.Length) = Natural (Normals.Length);
   --  Points an eye that holds the thing placed by itself (Driver.World.Sight), each with its normal (zero where not
   --  told), in place of the ones that eye placed before (their First). Each was found along its line of sight in
   --  views from elsewhere that bear one another out, so it needs nothing else to bear it out.
   --  Carried: the points are the ones placed before, carried where the hand that holds the thing took them
   --  (Driver.World.Holding), every beat it moves: the log says so when the hold begins and ends, not each beat.

   procedure Place_Pixels
     (S         : State;
      E         : Eye_Id;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      O         : Observation;
      Region    : Driver.Images.Mask;
      Points    : out Driver.World.Points.Placed_Vectors.Vector;
      Normals   : out Normal_Vectors.Vector;
      Asked     : out Natural;
      Counts    : out Driver.Robot.Surfaces.Placing_Counts);
   --  The points eye E's picture at O shows in the region that stand above the table, placed from the views kept
   --  (Sight.Place) at a grid of the region's pixels a patch's half width apart, less the pixels showing the robot
   --  (Asked of them): each with the pixel it was asked at and its normal.

   function Placed_Of (S : State; T : Thing_Id) return Driver.World.Points.Placed_Vectors.Vector;
   function Placed_Normals_Of (S : State; T : Thing_Id) return Normal_Vectors.Vector;
   --  The points an eye placed by itself (Place_Points), and their normals, as they stand.

   function Normals_Of (S : State; T : Thing_Id) return Normal_Vectors.Vector;
   --  Per point of Points_Of, in order, the normal its eye placed it with.

   procedure Note_Table (S : in out State; Plane : Driver.Geometry.Plane_Estimate);
   --  The table an arm's eye measured, in the world frame, its normal towards what rests on it: a surface things
   --  rest on beside the ones found among the scene's points, replaced when it is measured again; nothing while it
   --  is not known.

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

   use type Driver.Clock.Beat;

   package Point_Holders is new Ada.Containers.Indefinite_Holders (Driver.World.Tracking.Point_Array, Driver.World.Tracking."=");
   package Mask_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Driver.Images.Mask, Driver.Images."=");
   package Image_Holders is new Ada.Containers.Indefinite_Holders (Driver.Images.Image, Driver.Images."=");
   package Observation_Holders is new Ada.Containers.Indefinite_Holders (Observation, Driver.Observations."=");
   package Request_Holders is new Ada.Containers.Indefinite_Holders (String);
   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point_Estimate);

   --  A thing in one eye: its track, and the instrument's request out for it (a segmentation).
   type Slot is record
      Has       : Boolean := False;
      Pointed   : Boolean := False;   --  the region was given (the brain pointed at the thing here), not found
      Track     : Driver.World.Tracking.Track;
      Out_Now   : Boolean := False;
      Ticket    : Driver.Services.Ticket;
      Match_Due : Boolean := False;   --  lost at a still beat: its pixels are to be looked for in the latest picture
      Asked     : Point_Holders.Holder;   --  those pixels (Tracks.Match_Points)
      Tried_At  : Driver.Clock.Beat'Base := -1;   --  the thing's points it was last started from here (Start_Elsewhere)
      Tried_Box    : Driver.Instrument.Box;         --  and the box and point it was asked about then
      Tried_Prompt : Driver.Images.Pixel := (U => -1.0, V => -1.0);
      Placed_At : Driver.Clock.Beat'Base := -1;   --  the measurement of its region the eye's points were placed from
      Hulled_At : Driver.Clock.Beat'Base := -1;   --  and the one its outline last went into the thing's hull with
      Outline   : Point_Vectors.Vector;   --  a pointed region, carried out to the depth of what is seen of
                                          --  the thing; kept while the eye looks for it again
   end record;

   package Slot_Vectors is new Ada.Containers.Vectors (Eye_Id, Slot);

   --  A track being started in an eye that had none.
   type Start is record
      Eye    : Eye_Id;
      Ticket : Driver.Services.Ticket;
      On     : Image_Holders.Holder;
      Beat   : Driver.Clock.Beat := 0;
      Asked  : Request_Holders.Holder;   --  the request sent, as the instrument got it
   end record;

   package Start_Vectors is new Ada.Containers.Vectors (Positive, Start);

   type Thing_Record is record
      Eyes      : Slot_Vectors.Vector;
      Starts    : Start_Vectors.Vector;
      Points    : Driver.World.Points.Placed_Vectors.Vector;   --  every eye's that placed it
      Normals   : Normal_Vectors.Vector;                     --  per point of Points, its placed normal (Place_Points)
      Placed    : Driver.World.Points.Placed_Vectors.Vector;   --  the points an eye placed by itself (Place_Points)
      Placed_Normals : Normal_Vectors.Vector;
      Hull      : Driver.World.Points.Placed_Vectors.Vector;   --  from the outlines of two eyes or more (Hull)
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

   --  A point of the scene: where the eye that asked it placed it (Sight), and its place in the grid of that eye.
   type Scene_Point is record
      Point : Point_Estimate;
      Grid  : Driver.World.Supports.Grid_Point;
      Eye   : Eye_Id := Eye_Id'First;   --  the eye that asked it
      Pixel : Driver.Images.Pixel;      --  where it asked it
   end record;

   --  A region an eye holds, measured anew, to be placed: the thing, the eye, and the observation it was seen at.
   type Place_Request is record
      Thing : Thing_Id;
      Eye   : Eye_Id;
      Seen  : Observation_Holders.Holder;
   end record;

   package Place_Request_Vectors is new Ada.Containers.Vectors (Positive, Place_Request);

   --  A thing two eyes or more hold at a beat, an outline of it measured anew: its hull to work out apart (Hull).
   type Hull_Request is record
      Thing : Thing_Id;
      Seen  : Observation_Holders.Holder;
   end record;

   package Hull_Request_Vectors is new Ada.Containers.Vectors (Positive, Hull_Request);

   package Scene_Point_Vectors is new Ada.Containers.Vectors (Positive, Scene_Point);

   type State is limited record
      Things      : Thing_Vectors.Vector;
      Surfaces    : Driver.World.Supports.Surface_Vectors.Vector;
      Scene       : Scene_Point_Vectors.Vector;   --  the points the surfaces were found among
      Scene_Round : Natural := 0;                 --  the measurement of the scene they came from
      Earlier     : Boolean := False;             --  the surfaces are an earlier episode's
      Due         : Boolean := False;             --  the scene is to be measured at the next still beat: for an
      --  episode (New_Episode) and when it changed; nothing before an episode reads it, and measured during the boot
      --  it held the robot still (s77's cold boot, seed 6, 10-18: the scene measured at the arm's sweep kept the
      --  robot's models back 355 beats, a seventh of the sweep, and was measured again when the episode began).
      Booting     : Boolean := False;
      --  The body's boot is under way (Boot_Begins, Boot_Ends): the scene is not measured, its views kept. Nothing in
      --  the boot reads it, and measured at an arm's sweep it held the hall's robots 3 to 155 s of five minutes, the
      --  robot standing still meanwhile (f8s2, 10-28: Shadow seed 2 155 s, 18234 messages kept back).
      Round       : Natural := 0;                 --  the latest measurement of the scene
      Scene_Asked : Boolean := False;             --  it is due, to be measured apart (Measure)
      Round_Seen  : Observation_Holders.Holder;   --  the observation it is measured at
      Seen_From   : Vec3 := Zero3;                --  where an eye that placed its points stands
      Views       : Driver.World.Sight.Bank;      --  the eyes' still pictures
      Riding      : Mask_Vectors.Vector;
      --  Per eye, the pixels its latest scene showed riding with it (Driver.Robot.Surfaces.Placed.Riding), the cells
      --  of its grid there: the robot's own body in that eye, asked of nothing it places.
      To_Place    : Place_Request_Vectors.Vector; --  the regions measured anew, to be placed apart
      To_Hull     : Hull_Request_Vectors.Vector;  --  the things whose outlines are to meet apart
      Up          : Direction_Estimate;
      Places      : Place_Vectors.Vector;
      Table       : Driver.Geometry.Plane_Estimate;   --  the table an arm's eye measured, as noted (Note_Table)
   end record;

end Driver.World.Estimates;
