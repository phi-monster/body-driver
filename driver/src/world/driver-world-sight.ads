--  What the eyes place by themselves: the scene's points and a thing's, from the views of it the world keeps, with
--  no instrument matching one picture into another.
--
--  At every still beat each eye's picture is kept as a view, with the observation it was seen at (that picture
--  alone, and the readings that place the eye): a view of an eye that stands where a view of it kept before stood,
--  within a pixel's parallax at the table, takes that view's place, so that a view shows the scene as it was last
--  seen from there; one that stands apart from every view of that eye kept before is kept beside them. An eye that
--  rides on the body gathers views from wherever it was held still; an eye that stands in the world keeps one. The
--  pictures an arm's sweep held its eye still for are views of it too, kept the same way (Driver.World.Observe).
--
--  A pixel of an eye's picture now is placed from every view kept of any eye but those standing where it stands
--  (Driver.Robot.Surfaces.Place): its line of sight from the table towards the eye is a segment in each view that
--  sees its end at the table, the views scan it and bear one another out, and where three lines of sight meet, two
--  of the views telling the depth by themselves, the point is where they meet, with the covariance of their meeting.
--  Where a view stands is worked out when it is used, from the readings kept with it, so a fit refined since places
--  it better. The table bounds every segment: it is the measured surface everything the body reaches for rests on.
--  A view keeps its picture alone; the aligner's pyramid of it is built when it is used (a pyramid holds sixteen
--  bytes a pixel, a sweep's 47 views of a VGA eye 300 MB).

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Driver.Geometry;
with Driver.Robot.Surfaces;
with Driver.World.Cameras;

private package Driver.World.Sight is

   type Camera_Function is not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                                     return Driver.World.Cameras.Camera'Class;

   type Bank is private;

   procedure Keep (B : in out Bank; Eyes : Natural; Camera_Of : Camera_Function; Table : Driver.Geometry.Plane_Estimate;
                   O : Observation);
   --  The pictures of a still beat, kept as views of their eyes as above.

   procedure Keep_View (B : in out Bank; Camera_Of : Camera_Function; Table : Driver.Geometry.Plane_Estimate;
                        E : Eye_Id; O : Observation);
   --  Eye E's picture at O, kept as a view of it as above; none when the eye is not measured (its pose unknown) or
   --  its line of sight through the middle of its picture does not meet the table in front.

   function Views (B : Bank) return Natural;
   function Views_Of (B : Bank; E : Eye_Id) return Natural;
   --  How many views are kept, of every eye, of one.

   procedure Place
     (B         : Bank;
      Camera_Of : Camera_Function;
      Table     : Driver.Geometry.Plane_Estimate;
      E         : Eye_Id;
      O         : Observation;
      Queries   : Driver.Robot.Surfaces.Pixel_Array;
      Columns   : Positive;
      Result    : out Driver.Robot.Surfaces.Placed_Array;
      Counts    : out Driver.Robot.Surfaces.Placing_Counts)
     with Pre => Result'First = Queries'First and then Result'Last = Queries'Last;
   --  The queries of eye E's picture at O placed from the views kept, as above, in the world frame; none when the
   --  eye is not measured, the table not known, or no view stands apart from where the eye stands.

private

   package Observation_Holders is new Ada.Containers.Indefinite_Holders (Observation, Driver.Observations."=");

   type View_Record is record
      Eye   : Eye_Id;
      Seen  : Observation_Holders.Holder;   --  the observation, its eye's picture alone kept
      Stood : Vec3;                         --  where the eye stood when it was kept
   end record;

   package View_Vectors is new Ada.Containers.Vectors (Positive, View_Record);

   type Bank is record
      Kept : View_Vectors.Vector;
   end record;

end Driver.World.Sight;
