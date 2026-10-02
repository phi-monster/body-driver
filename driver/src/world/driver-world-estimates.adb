with Ada.Strings.Unbounded;
with Driver.Log;
with Driver.World.Regions;

package body Driver.World.Estimates is

   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;
   use type Driver.Clock.Beat;
   use type Driver.World.Tracking.Phase;

   package Tracks renames Driver.World.Tracking;

   Empty_Slot : Slot;
   --  No track; its ticket is never read while Out_Now is False.

   function Holding (T : Tracks.Track) return Slot is
      Result : Slot;
   begin
      Result.Has := True;
      Result.Track := T;
      return Result;
   end Holding;

   function Has_Slot (R : Thing_Record; E : Eye_Id) return Boolean is
     (E <= R.Eyes.Last_Index and then R.Eyes (E).Has);

   function Holds (R : Thing_Record; E : Eye_Id) return Boolean is
     (Has_Slot (R, E) and then Tracks.State (R.Eyes (E).Track) = Tracks.Holding);

   procedure Put_Slot (R : in out Thing_Record; E : Eye_Id; S : Slot) is
   begin
      while R.Eyes.Last_Index < E loop
         R.Eyes.Append (Empty_Slot);
      end loop;
      R.Eyes.Replace_Element (E, S);
   end Put_Slot;

   function Has_Image (O : Observation; E : Eye_Id) return Boolean is
     (E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E));

   --  A track's own requests: matching its pixels into the latest image of
   --  its eye when it is lost, then segmenting around where they went.
   procedure Serve (Id : Thing_Id; E : Eye_Id; Here : in out Slot; Beat : Driver.Clock.Beat) is
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
                  On    : constant Driver.Images.Image := Tracks.Latest (Here.Track);
                  Found : Driver.Images.Mask;
                  Score : Real;
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

   function Measured_When_Asked (R : Thing_Record; From, Into : Eye_Id) return Driver.Clock.Beat'Base is
   begin
      for A of R.Asked loop
         if A.From = From and then A.Into = Into then
            return A.Measured;
         end if;
      end loop;
      return -1;
   end Measured_When_Asked;

   procedure Note_Asked (R : in out Thing_Record; From, Into : Eye_Id; Measured : Driver.Clock.Beat) is
   begin
      for I in R.Asked.First_Index .. R.Asked.Last_Index loop
         if R.Asked (I).From = From and then R.Asked (I).Into = Into then
            R.Asked.Replace_Element (I, (From => From, Into => Into, Measured => Measured));
            return;
         end if;
      end loop;
      R.Asked.Append (Asked_Pair'(From => From, Into => Into, Measured => Measured));
   end Note_Asked;

   function Pending (R : Thing_Record; From, Into : Eye_Id) return Boolean is
     (for some X of R.Crosses => X.From = From and then X.Into = Into);

   function Starting (R : Thing_Record; E : Eye_Id) return Boolean is (for some St of R.Starts => St.Eye = E);

   --  The replies to the pairs asked: the points both eyes saw, and a track
   --  started where the second eye had none.
   procedure Read_Crosses
     (Id        : Thing_Id;
      R         : in out Thing_Record;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Beat      : Driver.Clock.Beat)
   is
      I : Positive := 1;
   begin
      while I <= Natural (R.Crosses.Length) loop
         declare
            X : constant Cross := R.Crosses (I);
         begin
            if Driver.Services.Ready (X.Ticket) then
               R.Crosses.Delete (I);
               declare
                  Points  : constant Driver.Instrument.Point_Array := X.Points.Element;
                  Answers : Driver.Instrument.Answer_Array (Points'Range);
                  Ok      : Boolean;
                  Why     : Unbounded_String;
                  Kept    : Driver.World.Pairs.Match_Vectors.Vector;
                  Apart   : Natural := 0;
                  Seen    : constant Observation_Holders.Constant_Reference_Type := X.Seen.Constant_Reference;
               begin
                  Driver.Instrument.Read_Match (Driver.Services.Collect (X.Ticket), True, Answers, Ok, Why);
                  if Ok then
                     Driver.World.Pairs.Triangulate (Camera_Of (X.From, Seen.Element), Camera_Of (X.Into, Seen.Element),
                                                     Points, X.Own, Answers, Kept, Apart);
                  else
                     Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": the instrument did not match eye"
                                      & X.From'Image & " into eye" & X.Into'Image & ": " & To_String (Why));
                  end if;
                  if not Kept.Is_Empty then
                     R.Points := Kept;
                     R.Points_In := X.From;
                     R.Points_At := Seen.Element.Beat;
                     R.Has_Points := True;
                     Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ":" & Kept.Length'Image
                                      & " points seen by eyes" & X.From'Image & " and" & X.Into'Image & ","
                                      & Apart'Image & " matches whose lines did not meet");
                     --  The second eye had no track: segment the thing there around
                     --  where its pixels went, prompted where its inner point went.
                     if not Holds (R, X.Into) and then not Starting (R, X.Into) then
                        declare
                           Box     : Driver.Instrument.Box := (X0 => Real'Last, Y0 => Real'Last,
                                                               X1 => Real'First, Y1 => Real'First);
                           Prompt  : Driver.Images.Pixel;
                           Nearest : Real := Real'Last;
                           On      : constant Driver.Images.Image := Seen.Element.Images (X.Into);
                        begin
                           for M of Kept loop
                              Box := (X0 => Real'Min (Box.X0, M.In_Second.U), Y0 => Real'Min (Box.Y0, M.In_Second.V),
                                      X1 => Real'Max (Box.X1, M.In_Second.U), Y1 => Real'Max (Box.Y1, M.In_Second.V));
                              if (M.In_First.U - X.Inner.U) ** 2 + (M.In_First.V - X.Inner.V) ** 2 < Nearest then
                                 Nearest := (M.In_First.U - X.Inner.U) ** 2 + (M.In_First.V - X.Inner.V) ** 2;
                                 Prompt := M.In_Second;
                              end if;
                           end loop;
                           R.Starts.Append (Start'
                             ((Eye    => X.Into,
                               Ticket => Driver.Instrument.Submit_Segment
                                           (On, True, Box, [1 => (At_Pixel => Prompt, On => True)], Beat),
                               On     => Image_Holders.To_Holder (On),
                               Beat   => Seen.Element.Beat)));
                        end;
                     end if;
                  end if;
               end;
            else
               I := I + 1;
            end if;
         end;
      end loop;
   end Read_Crosses;

   procedure Read_Starts (Id : Thing_Id; R : in out Thing_Record) is
      I : Positive := 1;
   begin
      while I <= Natural (R.Starts.Length) loop
         declare
            St : constant Start := R.Starts (I);
         begin
            if Driver.Services.Ready (St.Ticket) then
               R.Starts.Delete (I);
               declare
                  On    : constant Driver.Images.Image := St.On.Element;
                  Found : Driver.Images.Mask;
                  Score : Real;
                  Ok    : Boolean;
                  Why   : Unbounded_String;
               begin
                  Driver.Instrument.Read_Segment (Driver.Services.Collect (St.Ticket), Driver.Images.Width (On),
                                                  Driver.Images.Height (On), Found, Score, Ok, Why);
                  if Ok and then Driver.Images.Count (Found) > 0 then
                     Put_Slot (R, St.Eye, Holding (Tracks.Start (Found, On, St.Beat)));
                     Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": found in eye" & St.Eye'Image & ","
                                      & Driver.Images.Count (Found)'Image & " pixels");
                  end if;
               end;
            else
               I := I + 1;
            end if;
         end;
      end loop;
   end Read_Starts;

   procedure Observe
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Still     : Boolean;
      O         : Observation)
   is
   begin
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         declare
            R : Thing_Record := S.Things (Id);
         begin
            for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if R.Eyes (E).Has and then Has_Image (O, E) then
                  declare
                     Here : Slot := R.Eyes (E);
                  begin
                     Tracks.Observe (Here.Track, O.Images (E), O.Beat, Still);
                     Serve (Id, E, Here, O.Beat);
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
            Read_Crosses (Id, R, Camera_Of, O.Beat);
            Read_Starts (Id, R);
            --  Ask every other eye about a thing one eye sees, once for every
            --  time its region there was measured.
            for From in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if R.Eyes (From).Has and then Tracks.Seen (R.Eyes (From).Track)
                 and then Tracks.Latest_Beat (R.Eyes (From).Track) = O.Beat
               then
                  for Into in 1 .. Eye_Id'Base (Eyes) loop
                     if Into /= From and then Has_Image (O, Into) and then not Pending (R, From, Into)
                       and then Measured_When_Asked (R, From, Into) < Tracks.Measured_At (R.Eyes (From).Track)
                     then
                        declare
                           T      : Tracks.Track renames R.Eyes (From).Track;
                           Points : constant Driver.Instrument.Point_Array := Tracks.Region_And_Around (T);
                        begin
                           R.Crosses.Append (Cross'
                             ((From   => From,
                               Into   => Into,
                               Ticket => Driver.Instrument.Submit_Match
                                           ((Stored => False, Image => O.Images (From)),
                                            (Stored => False, Image => O.Images (Into)), Points, True, O.Beat),
                               Points => Point_Holders.To_Holder (Points),
                               Own    => Tracks.Region_Points (T),
                               Seen   => Observation_Holders.To_Holder (O),
                               Inner  => Driver.World.Regions.Inner_Point (Tracks.Region (T)))));
                           Note_Asked (R, From, Into, Tracks.Measured_At (T));
                        end;
                     end if;
                  end loop;
               end if;
            end loop;
            S.Things.Replace_Element (Id, R);
         end;
      end loop;
   end Observe;

   procedure Adopt (S : in out State; E : Eye_Id; O : Observation; Region : Driver.Images.Mask; Thing : out Thing_Id)
   is
      Fresh : constant Tracks.Track := Tracks.Start (Region, O.Images (E), O.Beat);
   begin
      --  The thing that covers the same pixels in this eye is this thing,
      --  measured again.
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         declare
            R : Thing_Record := S.Things (Id);
         begin
            if Has_Slot (R, E) and then Driver.World.Regions.Same_Pixels (Tracks.Region (R.Eyes (E).Track), Region) then
               Put_Slot (R, E, Holding (Fresh));
               S.Things.Replace_Element (Id, R);
               Thing := Id;
               return;
            end if;
         end;
      end loop;
      declare
         R : Thing_Record;
      begin
         Put_Slot (R, E, Holding (Fresh));
         S.Things.Append (R);
         Thing := S.Things.Last_Index;
         Driver.Log.Line (Driver.Log.World, "thing" & Thing'Image & ": adopted in eye" & E'Image & ","
                          & Driver.Images.Count (Region)'Image & " pixels");
      end;
   end Adopt;

   procedure New_Episode (S : in out State) is
   begin
      S.Things.Clear;
      S.Places.Clear;
      for F in S.Surfaces.First_Index .. S.Surfaces.Last_Index loop
         declare
            R : Surface_Record := S.Surfaces (F);
         begin
            R.Earlier := True;
            S.Surfaces.Replace_Element (F, R);
         end;
      end loop;
   end New_Episode;

   function Thing_Count (S : State) return Natural is (Natural (S.Things.Length));

   function Seen_In (S : State; T : Thing_Id; E : Eye_Id) return Boolean is
     (T <= S.Things.Last_Index and then Has_Slot (S.Things (T), E) and then Tracks.Seen (S.Things (T).Eyes (E).Track));

   function Region_In (S : State; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (Tracks.Region (S.Things (T).Eyes (E).Track));

   function Points_Of (S : State; T : Thing_Id) return Driver.World.Pairs.Match_Vectors.Vector is (S.Things (T).Points);

   function Points_Eye (S : State; T : Thing_Id) return Eye_Id is (S.Things (T).Points_In);

   function Centre (S : State; T : Thing_Id) return Point_Estimate is
      R          : constant Thing_Record := S.Things (T);
      Unmeasured : Point_Estimate;
      Sum        : Vec3 := Zero3;
      Spread     : Mat3 := [others => [others => 0.0]];
   begin
      if not R.Has_Points then
         return Unmeasured;
      end if;
      for M of R.Points loop
         Sum := Sum + M.Point.Mean;
         Spread := Spread + M.Point.Covariance;
      end loop;
      --  The centroid of what both eyes saw. Its points share the eyes' pose
      --  errors, so their covariance is kept as one point's, not divided by
      --  their number.
      return (Mean       => (1.0 / Real (R.Points.Length)) * Sum,
              Covariance => (1.0 / Real (R.Points.Length)) * Spread);
   end Centre;

   procedure Touched (S : in out State; T : Thing_Id; Point : Point_Estimate) is
      R : Thing_Record := S.Things (T);
   begin
      R.Touches.Append (Point);
      S.Things.Replace_Element (T, R);
   end Touched;

   procedure Learn_Friction (S : in out State; T : Thing_Id; Bounds : Friction_Bounds) is
      R : Thing_Record := S.Things (T);
   begin
      R.Friction := (Low => Real'Max (R.Friction.Low, Bounds.Low), High => Real'Min (R.Friction.High, Bounds.High));
      S.Things.Replace_Element (T, R);
   end Learn_Friction;

   function Friction (S : State; T : Thing_Id) return Friction_Bounds is (S.Things (T).Friction);

   procedure Remember (S : in out State; Point : Point_Estimate; Place : out Place_Id) is
   begin
      S.Places.Append (Point);
      Place := S.Places.Last_Index;
   end Remember;

   function Where (S : State; P : Place_Id) return Point_Estimate is (S.Places (P));
   function Place_Count (S : State) return Natural is (Natural (S.Places.Length));
   function Surface_Count (S : State) return Natural is (Natural (S.Surfaces.Length));
   function Plane_Of (S : State; F : Surface_Id) return Driver.Geometry.Plane_Estimate is (S.Surfaces (F).Plane);
   function Earlier (S : State; F : Surface_Id) return Boolean is (S.Surfaces (F).Earlier);

end Driver.World.Estimates;
