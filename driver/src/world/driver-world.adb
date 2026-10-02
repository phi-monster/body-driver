with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Clock;
with Driver.Instrument;
with Driver.Log;
with Driver.Services;
with Driver.World.Regions;
with Driver.World.Tracking;

package body Driver.World is

   use Ada.Strings.Unbounded;
   use type Driver.Observations.Camera_Id;
   use type Driver.World.Tracking.Phase;

   package Point_Holders is new Ada.Containers.Indefinite_Holders (Driver.Instrument.Point_Array, Driver.Instrument."=");

   --  A thing in one eye: its track, and the instrument's request out for it.
   type Slot is record
      Has     : Boolean := False;
      Track   : Driver.World.Tracking.Track;
      Out_Now : Boolean := False;
      Ticket  : Driver.Services.Ticket;
      Points  : Point_Holders.Holder;   --  a match request's points
   end record;

   Empty_Slot : Slot;
   --  No track; its ticket is never read while Out_Now is False.

   function Holding (T : Driver.World.Tracking.Track) return Slot is
      Result : Slot;
   begin
      Result.Has := True;
      Result.Track := T;
      return Result;
   end Holding;

   package Slot_Vectors is new Ada.Containers.Vectors (Eye_Id, Slot);
   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point_Estimate);

   type Thing_Record is record
      Eyes     : Slot_Vectors.Vector;
      Friction : Friction_Bounds;
      Touches  : Point_Vectors.Vector;
   end record;

   package Thing_Vectors is new Ada.Containers.Vectors (Thing_Id, Thing_Record);

   type Surface_Record is record
      Plane   : Driver.Geometry.Plane_Estimate;
      Earlier : Boolean := False;
   end record;

   package Surface_Vectors is new Ada.Containers.Vectors (Surface_Id, Surface_Record);
   package Place_Vectors is new Ada.Containers.Vectors (Place_Id, Point_Estimate);

   type Scene_Data is record
      Things   : Thing_Vectors.Vector;
      Surfaces : Surface_Vectors.Vector;
      Places   : Place_Vectors.Vector;
   end record;

   procedure Free is new Ada.Unchecked_Deallocation (Scene_Data, Scene_Data_Access);

   overriding procedure Finalize (S : in out Scene) is
   begin
      Free (S.Data);
   end Finalize;

   procedure Ensure (S : in out Scene) is
   begin
      if S.Data = null then
         S.Data := new Scene_Data;
      end if;
   end Ensure;

   function Has_Slot (R : Thing_Record; E : Eye_Id) return Boolean is
     (E <= R.Eyes.Last_Index and then R.Eyes (E).Has);

   procedure Put_Slot (R : in out Thing_Record; E : Eye_Id; S : Slot) is
   begin
      while R.Eyes.Last_Index < E loop
         R.Eyes.Append (Empty_Slot);
      end loop;
      R.Eyes.Replace_Element (E, S);
   end Put_Slot;

   --  Asks the instrument for what a slot's track wants, and reads what it answered.
   procedure Serve (Id : Thing_Id; E : Eye_Id; Here : in out Slot; Beat : Driver.Clock.Beat) is
      package Tracks renames Driver.World.Tracking;
   begin
      if Here.Out_Now and then Driver.Services.Ready (Here.Ticket) then
         declare
            Reply : constant Driver.Services.Reply := Driver.Services.Collect (Here.Ticket);
            Ok    : Boolean;
            Why   : Unbounded_String;
         begin
            Here.Out_Now := False;
            if Tracks.State (Here.Track) = Tracks.Matching then
               declare
                  Points  : constant Driver.Instrument.Point_Array := Here.Points.Element;
                  Answers : Driver.Instrument.Answer_Array (Points'Range);
               begin
                  Driver.Instrument.Read_Match (Reply, True, Answers, Ok, Why);
                  if Ok then
                     Tracks.Matched (Here.Track, Points, Answers);
                  else
                     Tracks.Failed (Here.Track);
                  end if;
               end;
            else
               declare
                  On     : constant Driver.Images.Image := Tracks.Latest (Here.Track);
                  Found  : Driver.Images.Mask;
                  Score  : Real;
               begin
                  Driver.Instrument.Read_Segment (Reply, Driver.Images.Width (On), Driver.Images.Height (On),
                                                  Found, Score, Ok, Why);
                  if Ok then
                     Tracks.Segmented (Here.Track, Found);
                  else
                     Tracks.Failed (Here.Track);
                  end if;
               end;
            end if;
            if not Ok then
               Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " in eye" & E'Image
                                & ": the instrument did not answer: " & To_String (Why));
            elsif Tracks.State (Here.Track) = Tracks.Gone then
               Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " is gone from eye" & E'Image);
            end if;
         end;
      end if;
      if not Here.Out_Now and then Tracks.Wants_Match (Here.Track) then
         declare
            Points : constant Driver.Instrument.Point_Array := Tracks.Match_Points (Here.Track);
         begin
            Here.Ticket := Driver.Instrument.Submit_Match
              ((Stored => False, Image => Tracks.Measured_On (Here.Track)),
               (Stored => False, Image => Tracks.Latest (Here.Track)), Points, True, Beat);
            Here.Points := Point_Holders.To_Holder (Points);
            Here.Out_Now := True;
            Tracks.Asked_Match (Here.Track);
            Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " changed in eye" & E'Image & "; looking for it");
         end;
      elsif not Here.Out_Now and then Tracks.Wants_Segment (Here.Track) then
         declare
            Around   : Driver.Instrument.Box;
            At_Point : Driver.Instrument.Pixel;
         begin
            Tracks.Segment_Prompt (Here.Track, Around, At_Point);
            Here.Ticket := Driver.Instrument.Submit_Segment
              (Tracks.Segment_On (Here.Track), True, Around, [1 => (At_Pixel => At_Point, On => True)], Beat);
            Here.Out_Now := True;
            Tracks.Asked_Segment (Here.Track);
         end;
      end if;
   end Serve;

   procedure Observe
     (S    : in out Scene;
      M    : Driver.Robot.Model;
      H    : Driver.Robot.Hand.Hands;
      O    : Observation;
      Sent : Driver.Commands.Command)
   is
      pragma Unreferenced (H, Sent);
      Still : constant Boolean := Driver.Robot.Still (M);
   begin
      Ensure (S);
      for Id in S.Data.Things.First_Index .. S.Data.Things.Last_Index loop
         declare
            R : Thing_Record := S.Data.Things (Id);
         begin
            for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if R.Eyes (E).Has and then E <= O.Images.Last_Index then
                  declare
                     Here : Slot := R.Eyes (E);
                  begin
                     Driver.World.Tracking.Observe (Here.Track, O.Images (E), O.Beat, Still);
                     Serve (Id, E, Here, O.Beat);
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
            S.Data.Things.Replace_Element (Id, R);
         end;
      end loop;
   end Observe;

   procedure New_Episode (S : in out Scene) is
   begin
      Ensure (S);
      S.Data.Things.Clear;
      S.Data.Places.Clear;
      for F in S.Data.Surfaces.First_Index .. S.Data.Surfaces.Last_Index loop
         declare
            R : Surface_Record := S.Data.Surfaces (F);
         begin
            R.Earlier := True;
            S.Data.Surfaces.Replace_Element (F, R);
         end;
      end loop;
   end New_Episode;

   function Thing_Count (S : Scene) return Natural is (if S.Data = null then 0 else Natural (S.Data.Things.Length));

   procedure Adopt
     (S      : in out Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask;
      Thing  : out Thing_Id)
   is
      pragma Unreferenced (M);
      Fresh : constant Driver.World.Tracking.Track := Driver.World.Tracking.Start (Region, O.Images (E), O.Beat);
   begin
      Ensure (S);
      --  The thing that covers the same pixels in this eye is this thing,
      --  measured again.
      for Id in S.Data.Things.First_Index .. S.Data.Things.Last_Index loop
         declare
            R : Thing_Record := S.Data.Things (Id);
         begin
            if Has_Slot (R, E)
              and then Driver.World.Regions.Same_Pixels (Driver.World.Tracking.Region (R.Eyes (E).Track), Region)
            then
               Put_Slot (R, E, Holding (Fresh));
               S.Data.Things.Replace_Element (Id, R);
               Thing := Id;
               return;
            end if;
         end;
      end loop;
      declare
         R : Thing_Record;
      begin
         Put_Slot (R, E, Holding (Fresh));
         S.Data.Things.Append (R);
         Thing := S.Data.Things.Last_Index;
         Driver.Log.Line (Driver.Log.World, "thing" & Thing'Image & ": adopted in eye" & E'Image & ","
                          & Driver.Images.Count (Region)'Image & " pixels");
      end;
   end Adopt;

   function Seen_In (S : Scene; T : Thing_Id; E : Eye_Id) return Boolean is
     (S.Data /= null and then T <= S.Data.Things.Last_Index and then Has_Slot (S.Data.Things (T), E)
      and then Driver.World.Tracking.Seen (S.Data.Things (T).Eyes (E).Track));

   function Region_In (S : Scene; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (Driver.World.Tracking.Region (S.Data.Things (T).Eyes (E).Track));

   --  Not measured yet: a thing's place in the world comes from two eyes.

   function Centre (S : Scene; T : Thing_Id) return Point_Estimate is
      pragma Unreferenced (S, T);
      Unmeasured : Point_Estimate;
   begin
      return Unmeasured;
   end Centre;

   function Resting_On (S : Scene; T : Thing_Id) return Surface_Id'Base is (0);

   function Height_Above_Support (S : Scene; T : Thing_Id) return Estimate is (Unknown);

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base is (0);

   function Moving (S : Scene; T : Thing_Id) return Boolean is (False);

   procedure Remember (S : in out Scene; Point : Point_Estimate; Place : out Place_Id) is
   begin
      Ensure (S);
      S.Data.Places.Append (Point);
      Place := S.Data.Places.Last_Index;
   end Remember;

   function Where (S : Scene; P : Place_Id) return Point_Estimate is (S.Data.Places (P));

   function Samples (S : Scene; T : Thing_Id) return Sample_Array is
      pragma Unreferenced (S, T);
   begin
      return [1 .. 0 => (others => <>)];
   end Samples;

   function Sample_Sigma (S : Scene; T : Thing_Id) return Real is (Real'Last);

   procedure Touched (S : in out Scene; T : Thing_Id; Point : Point_Estimate) is
      R : Thing_Record := S.Data.Things (T);
   begin
      R.Touches.Append (Point);
      S.Data.Things.Replace_Element (T, R);
   end Touched;

   procedure Learn_Friction (S : in out Scene; T : Thing_Id; Bounds : Friction_Bounds) is
      R : Thing_Record := S.Data.Things (T);
   begin
      R.Friction := (Low => Real'Max (R.Friction.Low, Bounds.Low), High => Real'Min (R.Friction.High, Bounds.High));
      S.Data.Things.Replace_Element (T, R);
   end Learn_Friction;

   function Friction (S : Scene; T : Thing_Id) return Friction_Bounds is (S.Data.Things (T).Friction);

   function Predicted (S : Scene; T : Thing_Id; Beats : Natural) return Point_Estimate is
      pragma Unreferenced (Beats);
   begin
      return Centre (S, T);
   end Predicted;

   function Surface_Count (S : Scene) return Natural is (if S.Data = null then 0 else Natural (S.Data.Surfaces.Length));

   function Plane_Of (S : Scene; F : Surface_Id) return Driver.Geometry.Plane_Estimate is (S.Data.Surfaces (F).Plane);

   function Earlier (S : Scene; F : Surface_Id) return Boolean is (S.Data.Surfaces (F).Earlier);

   function Place_Count (S : Scene) return Natural is (if S.Data = null then 0 else Natural (S.Data.Places.Length));

end Driver.World;
