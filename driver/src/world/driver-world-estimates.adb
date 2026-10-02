with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Log;
with Driver.World.Regions;

package body Driver.World.Estimates is

   use Ada.Numerics.Long_Elementary_Functions;
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

   function Inside (Region : Driver.Images.Mask; Px : Driver.Images.Pixel) return Boolean is
     (Px.U >= 0.0 and then Px.V >= 0.0
      and then Px.U < Real (Driver.Images.Width (Region)) and then Px.V < Real (Driver.Images.Height (Region))
      and then Driver.Images.Contains (Region, Natural (Real'Floor (Px.U)), Natural (Real'Floor (Px.V))));

   --  The scene's points the eye saw inside a thing's region there are the
   --  thing's own, unless another thing has them already.
   procedure Claim (Points : in out Scene_Point_Vectors.Vector; T : Thing_Id; E : Eye_Id; Region : Driver.Images.Mask) is
   begin
      for K in Points.First_Index .. Points.Last_Index loop
         declare
            P : Scene_Point := Points (K);
         begin
            if P.Owner = 0 and then P.Eye = E and then Inside (Region, P.Pixel) then
               P.Owner := T;
               Points.Replace_Element (K, P);
            end if;
         end;
      end loop;
   end Claim;

   procedure Claim (S : in out State; T : Thing_Id; E : Eye_Id; Region : Driver.Images.Mask) is
   begin
      Claim (S.Scene, T, E, Region);
      Claim (S.Incoming, T, E, Region);
   end Claim;

   procedure Read_Starts (S : in out State; Id : Thing_Id; R : in out Thing_Record) is
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
                     Claim (S, Id, St.Eye, Found);
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

   function Stride_Of (Image : Driver.Images.Image) return Positive is
     (Positive'Max (1, Natural (Real'Floor (Sqrt (Real (Natural'Min (Driver.Images.Width (Image),
                                                                      Driver.Images.Height (Image))))))));

   function Grid_Points (Image : Driver.Images.Image; Stride : Positive; Robot : Driver.Images.Mask)
     return Driver.Instrument.Point_Array
   is
      --  The centre of the middle pixel of each cell of the grid, but where
      --  the eye sees the robot.
      Columns : constant Natural := Driver.Images.Width (Image) / Stride;
      Rows    : constant Natural := Driver.Images.Height (Image) / Stride;
      Points  : Driver.Instrument.Point_Array (1 .. Columns * Rows);
      Count   : Natural := 0;
   begin
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            declare
               Px : constant Driver.Images.Pixel :=
                 (U => Real (C * Stride + Stride / 2) + 0.5, V => Real (R * Stride + Stride / 2) + 0.5);
            begin
               if not Inside (Robot, Px) then
                  Count := Count + 1;
                  Points (Count) := Px;
               end if;
            end;
         end loop;
      end loop;
      return Points (1 .. Count);
   end Grid_Points;

   function Settled (S : State) return Boolean is
     (for all R of S.Things =>
        (for all Here of R.Eyes => not Here.Has or else Tracks.State (Here.Track) in Tracks.Holding | Tracks.Gone));
   --  No thing is being looked for again in any eye.

   function Round_Open (S : State) return Boolean is (for some X of S.Asking => X.Round = S.Round);

   --  The scene is measured at a still beat when it is due and nothing is
   --  being looked for again: each eye's grid into every other eye.
   procedure Ask_Background
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Still     : Boolean;
      O         : Observation)
   is
   begin
      if not Still or else not S.Due or else Round_Open (S) or else not Settled (S) then
         return;
      end if;
      S.Round := S.Round + 1;
      S.Round_Seen := Observation_Holders.To_Holder (O);
      S.Incoming.Clear;
      S.Next_Column := 0;
      S.Due := False;
      declare
         Seen : constant Observation_Holders.Constant_Reference_Type := S.Round_Seen.Constant_Reference;
      begin
      for From in 1 .. Eye_Id'Base (Eyes) loop
         for Into in 1 .. Eye_Id'Base (Eyes) loop
            if From /= Into and then Has_Image (O, From) and then Has_Image (O, Into) then
               declare
                  Stride : constant Positive := Stride_Of (O.Images (From));
                  Points : constant Driver.Instrument.Point_Array :=
                    Grid_Points (O.Images (From), Stride, Camera_Of (From, Seen.Element).Self_Mask);
               begin
                  if Points'Length > 0 then
                     S.Asking.Append
                       (Background'(From   => From,
                                    Into   => Into,
                                    Ticket => Driver.Instrument.Submit_Match
                                                ((Stored => False, Image => O.Images (From)),
                                                 (Stored => False, Image => O.Images (Into)), Points, True, O.Beat),
                                    Points => Point_Holders.To_Holder (Points),
                                    Stride => Stride,
                                    Round  => S.Round));
                  end if;
               end;
            end if;
         end loop;
      end loop;
      end;
      if Round_Open (S) then
         Driver.Log.Line (Driver.Log.World, "the scene: measuring it, round" & S.Round'Image);
      end if;
   end Ask_Background;

   function Owner_At (S : State; E : Eye_Id; Px : Driver.Images.Pixel) return Thing_Id'Base is
   begin
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         if Has_Slot (S.Things (Id), E) and then Inside (Tracks.Region (S.Things (Id).Eyes (E).Track), Px) then
            return Id;
         end if;
      end loop;
      return 0;
   end Owner_At;

   procedure Surfaces_Changed (S : in out State) is
   begin
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         declare
            R : Thing_Record := S.Things (Id);
         begin
            R.Under_Due := True;
            S.Things.Replace_Element (Id, R);
         end;
      end loop;
   end Surfaces_Changed;

   procedure Read_Background
     (S         : in out State;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
      I      : Positive := 1;
      Closed : Boolean := False;   --  the latest round's last reply came this beat
   begin
      while I <= Natural (S.Asking.Length) loop
         declare
            X : constant Background := S.Asking (I);
         begin
            if Driver.Services.Ready (X.Ticket) then
               S.Asking.Delete (I);
               declare
                  Reply : constant Driver.Services.Reply := Driver.Services.Collect (X.Ticket);
               begin
                  --  A reply to an earlier round, or an earlier episode's, is
                  --  collected and dropped.
                  if X.Round = S.Round then
                     declare
                        Points  : constant Driver.Instrument.Point_Array := X.Points.Element;
                        Answers : Driver.Instrument.Answer_Array (Points'Range);
                        Ok      : Boolean;
                        Why     : Unbounded_String;
                        Kept    : Driver.World.Pairs.Match_Vectors.Vector;
                        Apart   : Natural := 0;
                        Seen    : constant Observation_Holders.Constant_Reference_Type :=
                          S.Round_Seen.Constant_Reference;
                        First   : constant Driver.World.Cameras.Camera'Class := Camera_Of (X.From, Seen.Element);
                        Second  : constant Driver.World.Cameras.Camera'Class := Camera_Of (X.Into, Seen.Element);
                        Robot   : constant Driver.Images.Mask := Second.Self_Mask;
                        Columns : constant Natural := Driver.Images.Width (Seen.Element.Images (X.From)) / X.Stride;
                        Added   : Natural := 0;
                     begin
                        Driver.Instrument.Read_Match (Reply, True, Answers, Ok, Why);
                        if Ok then
                           Driver.World.Pairs.Triangulate (First, Second, Points, Points'Length, Answers, Kept, Apart);
                        else
                           Driver.Log.Line (Driver.Log.World, "the scene: the instrument did not match eye"
                                            & X.From'Image & " into eye" & X.Into'Image & ": " & To_String (Why));
                        end if;
                        --  Each pair's grid apart from the others', so no two are
                        --  neighbours; a match landing on the robot in the second
                        --  eye is not the scene's.
                        for M of Kept loop
                           if not Inside (Robot, M.In_Second) then
                              S.Incoming.Append
                                (Scene_Point'(Point => M.Point,
                                              Grid  => (Column => S.Next_Column
                                                                  + Natural (Real'Floor (M.In_First.U)) / X.Stride,
                                                        Row    => Natural (Real'Floor (M.In_First.V)) / X.Stride),
                                              Eye   => X.From,
                                              Pixel => M.In_First,
                                              Owner => Owner_At (S, X.From, M.In_First)));
                              Added := Added + 1;
                           end if;
                        end loop;
                        S.Next_Column := S.Next_Column + Columns + 1;
                        if Added > 0 then
                           S.Seen_From := First.Pose.Pose.Translation;
                        end if;
                        Driver.Log.Line (Driver.Log.World, "the scene:" & Added'Image & " points seen by eyes"
                                         & X.From'Image & " and" & X.Into'Image & "," & Apart'Image
                                         & " matches whose lines did not meet");
                        Closed := not Round_Open (S);
                     end;
                  end if;
               end;
            else
               I := I + 1;
            end if;
         end;
      end loop;
      if Closed then
         if S.Incoming.Is_Empty then
            Driver.Log.Line (Driver.Log.World, "the scene: round" & S.Round'Image & " gave no points; the surfaces stay");
         else
            declare
               All_Points : Driver.Geometry.Point_Array (1 .. Natural (S.Incoming.Length));
               All_Grid   : Driver.World.Supports.Grid_Array (1 .. Natural (S.Incoming.Length));
               Found      : Driver.World.Supports.Surface_Vectors.Vector;
            begin
               for K in All_Points'Range loop
                  All_Points (K) := S.Incoming (K).Point;
                  All_Grid (K) := S.Incoming (K).Grid;
               end loop;
               Driver.World.Supports.Find (All_Points, All_Grid, S.Up, S.Seen_From, Found);
               S.Surfaces := Found;
               S.Scene := S.Incoming;
               S.Incoming.Clear;
               S.Earlier := False;
               Surfaces_Changed (S);
               Driver.Log.Line (Driver.Log.World, "the scene:" & Found.Length'Image & " surfaces things can rest on, from"
                                & S.Scene.Length'Image & " points");
            end;
         end if;
      end if;
   end Read_Background;

   function Mostly_Of (S : State; F : Driver.World.Supports.Surface; T : Thing_Id) return Boolean is
      On : Natural := 0;
   begin
      for M of F.Members loop
         On := On + Boolean'Pos (S.Scene (M).Owner = T);
      end loop;
      return 2 * On > Natural (F.Members.Length);
   end Mostly_Of;

   --  A thing's region changed: the surfaces made of its own points may have
   --  moved with it, so they are dropped, and the scene is measured again.
   procedure Changed (S : in out State; T : Thing_Id) is
      F       : Positive := 1;
      Dropped : Natural := 0;
   begin
      while F <= Natural (S.Surfaces.Length) loop
         if Mostly_Of (S, S.Surfaces (F), T) then
            S.Surfaces.Delete (F);
            Dropped := Dropped + 1;
         else
            F := F + 1;
         end if;
      end loop;
      S.Due := True;
      if Dropped > 0 then
         Driver.Log.Line (Driver.Log.World, "the scene: thing" & T'Image & " changed;" & Dropped'Image
                          & " surfaces of its own dropped");
      end if;
   end Changed;

   procedure Refresh_Supports (S : in out State) is
   begin
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         if S.Things (Id).Under_Due then
            declare
               R      : Thing_Record := S.Things (Id);
               Points : Driver.Geometry.Point_Array (1 .. Natural (R.Points.Length));
               function Own (Member : Positive) return Boolean is (S.Scene (Member).Owner = Id);
            begin
               for K in Points'Range loop
                  Points (K) := R.Points (K).Point;
               end loop;
               R.Under := Driver.World.Supports.Under (S.Surfaces, Points, S.Up, Own'Access);
               R.Under_Due := False;
               S.Things.Replace_Element (Id, R);
            end;
         end if;
      end loop;
   end Refresh_Supports;

   procedure Observe
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Up        : Direction_Estimate;
      Still     : Boolean;
      O         : Observation)
   is
      Moved : array (S.Things.First_Index .. S.Things.Last_Index) of Boolean := [others => False];
   begin
      S.Up := Up;
      Read_Background (S, Camera_Of);
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         declare
            R : Thing_Record := S.Things (Id);
         begin
            for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if R.Eyes (E).Has and then Has_Image (O, E) then
                  declare
                     Here : Slot := R.Eyes (E);
                     Was  : constant Tracks.Phase := Tracks.State (Here.Track);
                  begin
                     Tracks.Observe (Here.Track, O.Images (E), O.Beat, Still);
                     Moved (Id) := Moved (Id) or else (Was = Tracks.Holding and then Tracks.State (Here.Track) = Tracks.Lost);
                     Serve (Id, E, Here, O.Beat);
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
            declare
               Before : constant Driver.Clock.Beat := R.Points_At;
               Had    : constant Boolean := R.Has_Points;
            begin
               Read_Crosses (Id, R, Camera_Of, O.Beat);
               R.Under_Due := R.Under_Due or else R.Has_Points /= Had or else R.Points_At /= Before;
            end;
            Read_Starts (S, Id, R);
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
      for Id in Moved'Range loop
         if Moved (Id) then
            Changed (S, Id);
            Surfaces_Changed (S);
         end if;
      end loop;
      Ask_Background (S, Eyes, Camera_Of, Still, O);
      Refresh_Supports (S);
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
               Claim (S, Id, E, Region);
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
         Claim (S, Thing, E, Region);
         Driver.Log.Line (Driver.Log.World, "thing" & Thing'Image & ": adopted in eye" & E'Image & ","
                          & Driver.Images.Count (Region)'Image & " pixels");
      end;
   end Adopt;

   procedure New_Episode (S : in out State) is
   begin
      S.Things.Clear;
      S.Places.Clear;
      --  The surfaces stay, as an earlier episode's, until the scene is
      --  measured again; their points belong to no thing of this episode, and
      --  a round still out is dropped when it answers.
      S.Earlier := not S.Surfaces.Is_Empty;
      for K in S.Scene.First_Index .. S.Scene.Last_Index loop
         declare
            P : Scene_Point := S.Scene (K);
         begin
            P.Owner := 0;
            S.Scene.Replace_Element (K, P);
         end;
      end loop;
      S.Round := S.Round + 1;
      S.Incoming.Clear;
      S.Due := True;
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
   function Plane_Of (S : State; F : Surface_Id) return Driver.Geometry.Plane_Estimate is
     (S.Surfaces (Positive (F)).Plane);
   function Earlier (S : State; F : Surface_Id) return Boolean is
     (S.Earlier and then Natural (F) <= Natural (S.Surfaces.Length));


   function Support_Of (S : State; T : Thing_Id) return Driver.World.Supports.Support is (S.Things (T).Under);

   function Resting_On (S : State; T : Thing_Id) return Surface_Id'Base is
     (Surface_Id'Base (S.Things (T).Under.Index));

   function Height_Above_Support (S : State; T : Thing_Id) return Estimate is
     (if S.Things (T).Under.Index = 0 then Unknown else S.Things (T).Under.Height);

   function Surface_Of (S : State; F : Surface_Id) return Driver.World.Supports.Surface is (S.Surfaces (Positive (F)));

end Driver.World.Estimates;
