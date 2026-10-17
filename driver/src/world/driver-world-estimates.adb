with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Ada.Strings.Unbounded;
with Driver.Alignment;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot.Surfaces;
with Driver.World.Regions;

package body Driver.World.Estimates is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;
   use type Driver.Alignment.Verdict;
   use type Driver.World.Tracking.Phase;

   package Tracks renames Driver.World.Tracking;

   --  Everything sized by pixels, points, surfaces or things lives on the
   --  heap: the estimates also run in the decider's task, whose stack is small.
   type Answers_Access is access Driver.World.Tracking.Answer_Array;
   type Points_Access is access Driver.Geometry.Point_Array;
   type Grid_Access is access Driver.World.Supports.Grid_Array;
   type Thing_Flags is array (Thing_Id range <>) of Boolean;
   type Thing_Flags_Access is access Thing_Flags;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.World.Tracking.Answer_Array, Answers_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Geometry.Point_Array, Points_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.World.Supports.Grid_Array, Grid_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Thing_Flags, Thing_Flags_Access);

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

   --  A track's own requests: a lost track's pixels looked for in the latest picture of its eye, apart from the main
   --  loop (Measure: Match_Track), then the instrument segmenting the thing around where they went.
   procedure Serve (Id : Thing_Id; E : Eye_Id; Here : in out Slot; Beat : Driver.Clock.Beat) is
   begin
      if Here.Out_Now and then Driver.Services.Ready (Here.Ticket) then
         declare
            Reply : constant Driver.Services.Reply := Driver.Services.Collect (Here.Ticket);
            Ok    : Boolean;
            Why   : Unbounded_String;
            On    : constant Driver.Images.Image := Tracks.Latest (Here.Track);
            Found : Driver.Images.Mask;
            Score : Real;
         begin
            Here.Out_Now := False;
            Driver.Instrument.Read_Segment (Reply, Driver.Images.Width (On), Driver.Images.Height (On),
                                            Found, Score, Ok, Why);
            if Ok then
               Tracks.Segmented (Here.Track, Found);
            else
               Tracks.Failed (Here.Track);
               Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " in eye" & E'Image
                                & ": the instrument did not answer: " & To_String (Why));
            end if;
         end;
      end if;
      if not Here.Out_Now and then not Here.Match_Due and then Tracks.Wants_Match (Here.Track) then
         Here.Asked := Point_Holders.To_Holder (Tracks.Match_Points (Here.Track));
         Here.Match_Due := True;
         Tracks.Asked_Match (Here.Track);
         Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " changed in eye" & E'Image & "; looking for it");
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

   --  A lost track's pixels looked for in the latest picture of its eye, both ways, by the driver's aligner: each
   --  from where it was, within as far as the aligner searches, and its answer aligned back from where it was found,
   --  as widely; the round trips of the pixels around the region measure the aligner's own error, and the region's
   --  pixels that came back right within it say where the thing went (Tracks.Matched). One eye's pictures before and
   --  after: no other eye and no instrument.
   procedure Match_Track (Id : Thing_Id; E : Eye_Id; Here : in out Slot) is
      Points  : constant Tracks.Point_Array := Here.Asked.Element;
      Answers : Answers_Access := new Tracks.Answer_Array (Points'Range);
      Before  : constant Driver.Alignment.Pyramid := Driver.Alignment.Pyramid_Of (Tracks.Measured_On (Here.Track));
      After   : constant Driver.Alignment.Pyramid := Driver.Alignment.Pyramid_Of (Tracks.Latest (Here.Track));
      Wide    : constant Real := Driver.Alignment.Largest_Sigma (Before, After);
      Window  : constant Driver.Alignment.Place_Covariance := (UU => Wide ** 2, UV => 0.0, VV => Wide ** 2);
      Back_At : Natural := 0;
   begin
      for K in Points'Range loop
         declare
            P   : constant Driver.Images.Pixel := Points (K);
            Got : constant Driver.Alignment.Answer :=
              Driver.Alignment.Align (Before, After, (From => P, To => P, Linear => Driver.Alignment.Identity_Part,
                                                      Cov => Window, Linear_Sigma => Real'Last));
         begin
            Answers (K) := (Found => False, To => P, Back => P);
            if Got.Verdict = Driver.Alignment.Found then
               declare
                  Back : constant Driver.Alignment.Answer :=
                    Driver.Alignment.Align (After, Before, (From => Got.To, To => Got.To,
                                                            Linear => Driver.Alignment.Identity_Part,
                                                            Cov => Window, Linear_Sigma => Real'Last));
               begin
                  if Back.Verdict = Driver.Alignment.Found then
                     Answers (K) := (Found => True, To => Got.To, Back => Back.To);
                     Back_At := Back_At + 1;
                  end if;
               end;
            end if;
         end;
      end loop;
      Tracks.Matched (Here.Track, Points, Answers.all);
      Free (Answers);
      Here.Match_Due := False;
      Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & " in eye" & E'Image & ":" & Back_At'Image & " of"
                       & Points'Length'Image & " pixels found again both ways"
                       & (if Tracks.State (Here.Track) = Tracks.Gone then "; it is gone from this eye" else ""));
   end Match_Track;

   function Starting (R : Thing_Record; E : Eye_Id) return Boolean is (for some St of R.Starts => St.Eye = E);

   function Inside (Region : Driver.Images.Mask; Px : Driver.Images.Pixel) return Boolean is
     (Px.U >= 0.0 and then Px.V >= 0.0
      and then Px.U < Real (Driver.Images.Width (Region)) and then Px.V < Real (Driver.Images.Height (Region))
      and then Driver.Images.Contains (Region, Natural (Real'Floor (Px.U)), Natural (Real'Floor (Px.V))));

   function On_Thing
     (R         : Thing_Record;
      X         : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean;

   procedure Gather
     (S         : State;
      Id        : Thing_Id;
      R         : in out Thing_Record;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation)
   is
      Now         : Driver.World.Points.Placed_Vectors.Vector := R.Placed;
      Now_Normals : Normal_Vectors.Vector := R.Placed_Normals;
   begin
      --  Its points: every eye's that placed it (Place_Points). While none did, the scene's points above the table
      --  that fall inside its region in every eye that holds it and sees where they are (On_Thing): an eye that
      --  stands in the world places nothing by itself, and what the eyes the body holds at several places placed of
      --  the scene, seen through that eye's region of it, is the thing (the brain boxes a thing in such an eye as
      --  often as in one that rides on the body: s55 adopted the black box in the fixed eye, and nothing gave it a
      --  place). Where the eye that boxed it sees a point on the table, the table is not the thing.
      if Now.Is_Empty and then Driver.Geometry.Known (S.Table) then
         for P of S.Scene loop
            declare
               H : constant Real := Driver.Geometry.Height (S.Table, P.Point.Mean);
               N : constant Vec3 := S.Table.Normal;
               Sigma : constant Real :=
                 Sqrt (N * (P.Point.Covariance * N) + Driver.Geometry.Height_Sigma (S.Table, P.Point.Mean) ** 2);
            begin
               if H > 0.0 and then Significant (H, Sigma) and then On_Thing (R, P.Point.Mean, Camera_Of, Seen) then
                  Now.Append (Driver.World.Points.Placed'(Pixel => P.Pixel, Point => P.Point, Eye => P.Eye));
                  Now_Normals.Append (Zero3);
               end if;
            end;
         end loop;
      end if;
      if Driver.World.Points.Placed_Vectors."/=" (Now, R.Points) then
         if R.Placed.Is_Empty and then not Now.Is_Empty then
            Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ":" & Now.Length'Image
                             & " of the scene's points above the table fall inside its region in every eye that"
                             & " holds it; they are its points until an eye places its region");
         end if;
         R.Points := Now;
         R.Normals := Now_Normals;
         R.Points_At := Seen.Beat;
      end if;
      R.Has_Points := not R.Points.Is_Empty;
      --  How far the thing reaches as the eyes it was pointed at in see it:
      --  each region's pixels, on a grid as many pixels apart as the square
      --  root of the region's shorter side, carried out along their sights
      --  to the depth of the middle of what is seen of it. Its points may
      --  cover only a part of what those regions show. An eye that lost the
      --  thing and looks for it again keeps what it last showed of it, as the
      --  thing keeps the points the pairs saw: the thing is where it was
      --  until an eye sees it elsewhere. Its region then is of an earlier
      --  view, so it is carried out as it was, not again.
      for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
         if R.Eyes (E).Has
           and then (not R.Has_Points or else not R.Eyes (E).Pointed
                     or else Tracks.State (R.Eyes (E).Track) = Tracks.Gone)
           and then not R.Eyes (E).Outline.Is_Empty
         then
            declare
               Here : Slot := R.Eyes (E);
            begin
               Here.Outline.Clear;
               R.Eyes.Replace_Element (E, Here);
            end;
         end if;
      end loop;
      if R.Has_Points then
         declare
            Middle : Vec3 := Zero3;
         begin
            for M of R.Points loop
               Middle := Middle + M.Point.Mean;
            end loop;
            Middle := (1.0 / Real (R.Points.Length)) * Middle;
            for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if Holds (R, E) and then R.Eyes (E).Pointed
                 and then Driver.Images.Count (Tracks.Region (R.Eyes (E).Track)) > 0
               then
                  declare
                     Here       : Slot := R.Eyes (E);
                     Eye_Camera : constant Driver.World.Cameras.Camera'Class := Camera_Of (E, Seen);
                     Region     : constant Driver.Images.Mask := Tracks.Region (Here.Track);
                     B          : constant Driver.World.Regions.Box := Driver.World.Regions.Bounds (Region);
                     Stride     : constant Positive :=
                       Positive'Max (1, Natural (Real'Floor (Sqrt (Real (Natural'Min (B.Column_1 - B.Column_0 + 1,
                                                                                      B.Row_1 - B.Row_0 + 1))))));
                     Axis       : constant Vec3 := Middle - Eye_Camera.Pose.Pose.Translation;
                     Row        : Natural := B.Row_0;
                  begin
                     Here.Outline.Clear;
                     while Row <= B.Row_1 loop
                        declare
                           Column : Natural := B.Column_0;
                        begin
                           while Column <= B.Column_1 loop
                              if Driver.Images.Contains (Region, Column, Row) then
                                 declare
                                    Sight : constant Ray_Estimate :=
                                      Eye_Camera.Ray ((U => Real (Column) + 0.5, V => Real (Row) + 0.5));
                                    Along : constant Real := Sight.Direction.Unit_Vector * Axis;
                                 begin
                                    if Along > 0.0 and then Sight.Direction.Sigma < Real'Last then
                                       Here.Outline.Append
                                         (Point_Estimate'
                                            (Mean       => Sight.Origin.Mean
                                                           + (Real'(Axis * Axis) / Along) * Sight.Direction.Unit_Vector,
                                             Covariance => [others => [others => 0.0]]));
                                    end if;
                                 end;
                              end if;
                              Column := Column + Stride;
                           end loop;
                        end;
                        Row := Row + Stride;
                     end loop;
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
         end;
      end if;
   end Gather;

   function In_Reach (F : Driver.World.Supports.Surface; X : Vec3) return Boolean;

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
                     --  What the thing covers changed: so may what is its own.
                     R.Under_Due := True;
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

   function Settled (S : State) return Boolean is
     (for all R of S.Things =>
        (for all Here of R.Eyes => not Here.Has or else Tracks.State (Here.Track) in Tracks.Holding | Tracks.Gone));
   --  No thing is being looked for again in any eye.

   --  The scene is measured at a still beat when it is due and nothing is being looked for again, apart from the main
   --  loop (Measure: Measure_Scene), at the observation of that beat; not before some eye has views from more than one
   --  place, as nothing places a point from one place alone.
   procedure Ask_Scene (S : in out State; Eyes : Natural; Still : Boolean; O : Observation) is
   begin
      if Still and then S.Due and then not S.Scene_Asked and then Settled (S) and then Driver.Geometry.Known (S.Table)
        and then (for some E in 1 .. Eye_Id'Base (Eyes) => Driver.World.Sight.Views_Of (S.Views, E) > 1)
      then
         S.Round := S.Round + 1;
         S.Round_Seen := Observation_Holders.To_Holder (O);
         S.Due := False;
         S.Scene_Asked := True;
         Driver.Log.Line (Driver.Log.World, "the scene: measuring it, round" & S.Round'Image);
      end if;
   end Ask_Scene;

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

   function In_Reach (F : Driver.World.Supports.Surface; X : Vec3) return Boolean is
      --  Over the surface's reach, along its normal.
      D : constant Vec3 := X - F.Plane.Centre;
      A : constant Real := D * F.Plane.Tangent_1;
      B : constant Real := D * F.Plane.Tangent_2;
   begin
      return A >= F.Low_1 and then A <= F.High_1 and then B >= F.Low_2 and then B <= F.High_2;
   end In_Reach;

   function Found_Again
     (Old : Driver.World.Supports.Surface; Found : Driver.World.Supports.Surface_Vectors.Vector) return Boolean
   is
      --  A surface found now over the old one's reach, or the old one over the
      --  new one's, at a height the two planes' uncertainties do not part.
   begin
      for F of Found loop
         if In_Reach (Old, F.Plane.Centre) or else In_Reach (F, Old.Plane.Centre) then
            declare
               H : constant Estimate :=
                 Driver.Geometry.Height
                   (Old.Plane, Point_Estimate'(Mean       => F.Plane.Centre,
                                               Covariance => (F.Plane.Offset_Sigma ** 2)
                                                             * Driver.Numerics.Outer (F.Plane.Normal, F.Plane.Normal)));
            begin
               if not Significant (Scalar_Gate (H.Degrees_Of_Freedom), H.Value, H.Sigma) then
                  return True;
               end if;
            end;
         end if;
      end loop;
      return False;
   end Found_Again;

   function Seen_Through
     (Old       : Driver.World.Supports.Surface;
      Points    : Scene_Point_Vectors.Vector;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean
   is
      --  Points seen now significantly beyond the surface from an eye that saw
      --  them, each line of sight crossing the surface's plane inside its
      --  reach; and those points a patch of the grid they were asked on (a two
      --  by two block), as a surface itself must be to be found: then the eyes
      --  see through where the surface was. A lone point, or a row, is no more
      --  a hole in a surface than it is a surface: one wrong match, its depth
      --  slid along the line its first sight draws in the second eye, sees
      --  through anything.
      Tests : constant Positive := 2 * Natural'Max (1, Natural (Points.Length));
      function "<" (A, B : Driver.World.Supports.Grid_Point) return Boolean is
        (A.Column < B.Column or else (A.Column = B.Column and then A.Row < B.Row));
      package Cell_Sets is new Ada.Containers.Ordered_Sets (Driver.World.Supports.Grid_Point, "<", Driver.World.Supports."=");
      Through : Cell_Sets.Set;

      --  The eye that placed it on one side of the surface, the point significantly on the other.
      function Sees_Through (P : Scene_Point) return Boolean is
         Eye    : constant Vec3 := Camera_Of (P.Eye, Seen).Pose.Pose.Translation;
         Before : constant Real := Driver.Geometry.Height (Old.Plane, Eye);
         H      : constant Estimate := Driver.Geometry.Height (Old.Plane, P.Point);
      begin
         return Before /= 0.0 and then H.Value * Before < 0.0
           and then Significant (Scalar_Gate (H.Degrees_Of_Freedom, Tests => Tests), H.Value, H.Sigma)
           and then In_Reach (Old, Eye + (Before / (Before - H.Value)) * (P.Point.Mean - Eye));
      end Sees_Through;

      function Has (C, R : Integer) return Boolean is (Through.Contains ((Column => C, Row => R)));
   begin
      for P of Points loop
         if Sees_Through (P) then
            Through.Include (P.Grid);
         end if;
      end loop;
      for G of Through loop
         if Has (G.Column + 1, G.Row) and then Has (G.Column, G.Row + 1) and then Has (G.Column + 1, G.Row + 1) then
            return True;
         end if;
      end loop;
      return False;
   end Seen_Through;

   --  The supports among the scene's points just placed (Driver.World.Supports): a surface of this episode stands
   --  where this measurement neither found it again nor saw through it, its points coming along, since eyes that look
   --  elsewhere now say nothing of it.
   procedure Settle_Scene
     (S         : in out State;
      Incoming  : Scene_Point_Vectors.Vector;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
   begin
      if Incoming.Is_Empty then
         Driver.Log.Line (Driver.Log.World, "the scene: round" & S.Round'Image & " gave no points; the surfaces stay");
         return;
      end if;
      declare
         All_Points : Points_Access := new Driver.Geometry.Point_Array (1 .. Natural (Incoming.Length));
         All_Grid   : Grid_Access := new Driver.World.Supports.Grid_Array (1 .. Natural (Incoming.Length));
         Found      : Driver.World.Supports.Surface_Vectors.Vector;
         Scene      : Scene_Point_Vectors.Vector := Incoming;
         Carried    : Natural := 0;
      begin
         for K in All_Points'Range loop
            All_Points (K) := Incoming (K).Point;
            All_Grid (K) := Incoming (K).Grid;
         end loop;
         Driver.World.Supports.Find (All_Points.all, All_Grid.all, S.Up, S.Seen_From, Found);
         if not S.Earlier then
            for Old of S.Surfaces loop
               if not Found_Again (Old, Found)
                 and then not Seen_Through (Old, Incoming, Camera_Of, S.Round_Seen.Constant_Reference.Element)
               then
                  declare
                     Kept : Driver.World.Supports.Surface := Old;
                  begin
                     Kept.Members.Clear;
                     for M of Old.Members loop
                        Scene.Append (S.Scene (M));
                        Kept.Members.Append (Positive (Scene.Length));
                     end loop;
                     Found.Append (Kept);
                     Carried := Carried + 1;
                  end;
               end if;
            end loop;
         end if;
         S.Surfaces := Found;
         S.Scene := Scene;
         S.Scene_Round := S.Round;
         S.Earlier := False;
         Surfaces_Changed (S);
         Driver.Log.Line (Driver.Log.World, "the scene:" & Found.Length'Image & " surfaces things can rest on, from"
                          & All_Points'Length'Image & " points," & Carried'Image
                          & " of them measured before and not seen since");
         Free (All_Points);
         Free (All_Grid);
      end;
   end Settle_Scene;

   --  The scene at the observation it was asked at: a grid of the picture of every eye that has views from more than
   --  one place, less the pixels that show the robot, placed from the views (Driver.World.Sight), the table among it;
   --  each eye's grid kept apart from the others' so that no two are neighbours.
   procedure Measure_Scene
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
      Held        : constant Observation_Holders.Holder := S.Round_Seen;
      Seen        : constant Observation_Holders.Constant_Reference_Type := Held.Constant_Reference;
      Incoming    : Scene_Point_Vectors.Vector;
      Next_Column : Natural := 0;
   begin
      for E in 1 .. Eye_Id'Base (Eyes) loop
         if Has_Image (Seen.Element.all, E) and then Driver.World.Sight.Views_Of (S.Views, E) > 1 then
            declare
               Image   : constant Driver.Images.Image := Seen.Element.Images (E);
               Stride  : constant Positive := Stride_Of (Image);
               Columns : constant Natural := Driver.Images.Width (Image) / Stride;
               Rows    : constant Natural := Driver.Images.Height (Image) / Stride;
               Robot   : constant Driver.Images.Mask := Camera_Of (E, Seen.Element).Self_Mask;
            begin
               if Columns > 0 and then Rows > 0 then
                  declare
                     Queries : Driver.Robot.Surfaces.Pixel_Array (1 .. Columns * Rows);
                     Got     : Driver.Robot.Surfaces.Placed_Array (Queries'Range);
                     Asked, Added, Table_Points : Natural := 0;
                  begin
                     for R in 0 .. Rows - 1 loop
                        for C in 0 .. Columns - 1 loop
                           declare
                              Cell : constant Driver.Images.Pixel :=
                                (U => Real (C * Stride + Stride / 2) + 0.5, V => Real (R * Stride + Stride / 2) + 0.5);
                           begin
                              if Inside (Robot, Cell) then
                                 Queries (R * Columns + C + 1) := (U => -1.0, V => -1.0);
                              else
                                 Queries (R * Columns + C + 1) := Cell;
                                 Asked := Asked + 1;
                              end if;
                           end;
                        end loop;
                     end loop;
                     Driver.World.Sight.Place (S.Views, Camera_Of, S.Table, E, Seen.Element.all, Queries, Columns, Got);
                     for K in Got'Range loop
                        if Got (K).Found then
                           Incoming.Append
                             (Scene_Point'(Point => Got (K).Point,
                                           Pixel => Queries (K),
                                           Grid  => (Column => Next_Column + (K - Got'First) mod Columns,
                                                     Row    => (K - Got'First) / Columns),
                                           Eye   => E));
                           Added := Added + 1;
                           Table_Points := Table_Points + Boolean'Pos (Got (K).On_Table);
                        end if;
                     end loop;
                     Next_Column := Next_Column + Columns + 1;
                     if Added > 0 then
                        S.Seen_From := Camera_Of (E, Seen.Element).Pose.Pose.Translation;
                     end if;
                     Driver.Log.Line (Driver.Log.World, "the scene: eye" & E'Image & " placed" & Added'Image & " of its"
                                      & Asked'Image & " points from" & Driver.World.Sight.Views (S.Views)'Image
                                      & " views," & Table_Points'Image & " of them on the table");
                  end;
               end if;
            end;
         end if;
      end loop;
      Settle_Scene (S, Incoming, Camera_Of);
   end Measure_Scene;

   function On_Thing
     (R         : Thing_Record;
      X         : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean
   is
      --  A point of the scene inside the thing's region in every eye that has
      --  one and sees where the point is: inside what all those eyes see of
      --  the thing (on it, or hidden behind it in some eye), so a region one
      --  eye got wrong claims nothing the others keep apart.
      In_Some : Boolean := False;
   begin
      for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
         if R.Eyes (E).Has then
            declare
               Px      : Driver.Images.Pixel;
               Visible : Boolean;
            begin
               Camera_Of (E, Seen).Project (X, Px, Visible);
               if Visible then
                  if not Inside (Tracks.Region (R.Eyes (E).Track), Px) then
                     return False;
                  end if;
                  In_Some := True;
               end if;
            end;
         end if;
      end loop;
      return In_Some;
   end On_Thing;

   --  A thing's region changed: the surfaces made of its own points may have
   --  moved with it, so they are dropped, and the scene is measured again.
   procedure Changed
     (S         : in out State;
      T         : Thing_Id;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation)
   is
      F       : Positive := 1;
      Dropped : Natural := 0;
      R       : constant Thing_Record := S.Things (T);
      function Own (Member : Positive) return Boolean is (On_Thing (R, S.Scene (Member).Point.Mean, Camera_Of, Seen));
   begin
      while F <= Natural (S.Surfaces.Length) loop
         if Driver.World.Supports.Mostly (S.Surfaces (F), Own'Access) then
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

   procedure Refresh_Supports
     (S         : in out State;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) is
   begin
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         if S.Things (Id).Under_Due then
            declare
               R      : Thing_Record := S.Things (Id);
               Points : Points_Access := new Driver.Geometry.Point_Array (1 .. Natural (R.Points.Length));
               function Own (Member : Positive) return Boolean is
                 (On_Thing (R, S.Scene (Member).Point.Mean, Camera_Of, Seen));
            begin
               for K in Points'Range loop
                  Points (K) := R.Points (K).Point;
               end loop;
               R.Under := Driver.World.Supports.Under (S.Surfaces, Points.all, S.Up, Own'Access);
               Free (Points);
               R.Under_Due := False;
               S.Things.Replace_Element (Id, R);
            end;
         end if;
      end loop;
   end Refresh_Supports;

   --  An eye with no track of the thing, where the thing's points fall in its picture: a track started there, the
   --  instrument segmenting the thing around where they fall, prompted where their middle falls; once for every time
   --  the points were placed anew.
   procedure Start_Elsewhere
     (Id        : Thing_Id;
      R         : in out Thing_Record;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      O         : Observation;
      Seen      : not null access constant Observation)
   is
   begin
      if not R.Has_Points then
         return;
      end if;
      for Into in 1 .. Eye_Id'Base (Eyes) loop
         if Has_Image (O, Into) and then not Holds (R, Into) and then not Starting (R, Into)
           and then (not Has_Slot (R, Into) or else R.Eyes (Into).Tried_At < R.Points_At)
         then
            declare
               Cam    : constant Driver.World.Cameras.Camera'Class := Camera_Of (Into, Seen);
               Box    : Driver.Instrument.Box := (X0 => Real'Last, Y0 => Real'Last, X1 => Real'First, Y1 => Real'First);
               Middle : Vec3 := Zero3;
               Count  : Natural := 0;
            begin
               for M of R.Points loop
                  declare
                     Px      : Driver.Images.Pixel;
                     Visible : Boolean;
                  begin
                     Cam.Project (M.Point.Mean, Px, Visible);
                     if Visible then
                        Box := (X0 => Real'Min (Box.X0, Px.U), Y0 => Real'Min (Box.Y0, Px.V),
                                X1 => Real'Max (Box.X1, Px.U), Y1 => Real'Max (Box.Y1, Px.V));
                        Middle := Middle + M.Point.Mean;
                        Count := Count + 1;
                     end if;
                  end;
               end loop;
               declare
                  Here : Slot := (if Has_Slot (R, Into) then R.Eyes (Into) else Empty_Slot);
               begin
                  Here.Tried_At := R.Points_At;
                  if Has_Slot (R, Into) then
                     R.Eyes.Replace_Element (Into, Here);
                  else
                     while R.Eyes.Last_Index < Into loop
                        R.Eyes.Append (Empty_Slot);
                     end loop;
                     R.Eyes.Replace_Element (Into, Here);
                  end if;
               end;
               if Count > 0 then
                  declare
                     Prompt  : Driver.Images.Pixel;
                     Visible : Boolean;
                  begin
                     Cam.Project ((1.0 / Real (Count)) * Middle, Prompt, Visible);
                     if Visible then
                        declare
                           Request : constant String :=
                             Driver.Instrument.Segment_Request
                               (O.Images (Into), True, Box, [1 => (At_Pixel => Prompt, On => True)]);
                        begin
                           R.Starts.Append (Start'
                             ((Eye    => Into,
                               Ticket => Driver.Services.Submit (Driver.Services.Instrument, "/segment", Request, O.Beat),
                               On     => Image_Holders.To_Holder (O.Images (Into)),
                               Beat   => O.Beat,
                               Asked  => Request_Holders.To_Holder (Request))));
                           Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ":" & Count'Image
                                            & " of its points fall in eye" & Into'Image & "'s picture; segmenting it there");
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
   end Start_Elsewhere;

   --  A region an eye holds, measured anew, placed: the pixels of a grid of it a patch's half width apart, less those
   --  that show the robot, placed from the views (Driver.World.Sight); those standing above the table are the thing's
   --  own, its points from that eye (Place_Points).
   procedure Place_Pixels
     (S         : State;
      E         : Eye_Id;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      O         : Observation;
      Region    : Driver.Images.Mask;
      Points    : out Driver.World.Points.Placed_Vectors.Vector;
      Normals   : out Normal_Vectors.Vector;
      Asked     : out Natural)
   is
      Held    : constant Observation_Holders.Holder := Observation_Holders.To_Holder (O);
      Seen    : constant Observation_Holders.Constant_Reference_Type := Held.Constant_Reference;
      W       : constant Natural := Driver.Images.Width (Region);
      H       : constant Natural := Driver.Images.Height (Region);
      Step    : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (W, H));
      Columns : constant Positive := Positive'Max (1, (W + Step - 1) / Step);
      Rows    : constant Positive := Positive'Max (1, (H + Step - 1) / Step);
      Robot   : constant Driver.Images.Mask := Camera_Of (E, Seen.Element).Self_Mask;
      Queries : Driver.Robot.Surfaces.Pixel_Array (1 .. Columns * Rows);
      Got     : Driver.Robot.Surfaces.Placed_Array (Queries'Range);
   begin
      Points.Clear;
      Normals.Clear;
      Asked := 0;
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            declare
               Cell : constant Driver.Images.Pixel :=
                 (U => Real (C * Step + Step / 2) + 0.5, V => Real (R * Step + Step / 2) + 0.5);
            begin
               if Inside (Region, Cell) and then not Inside (Robot, Cell) then
                  Queries (R * Columns + C + 1) := Cell;
                  Asked := Asked + 1;
               else
                  Queries (R * Columns + C + 1) := (U => -1.0, V => -1.0);
               end if;
            end;
         end loop;
      end loop;
      if Asked = 0 then
         return;
      end if;
      Driver.World.Sight.Place (S.Views, Camera_Of, S.Table, E, O, Queries, Columns, Got);
      for K in Got'Range loop
         if Got (K).Found and then not Got (K).On_Table then
            Points.Append (Driver.World.Points.Placed'(Pixel => Queries (K), Point => Got (K).Point, Eye => E));
            Normals.Append (Got (K).Normal);
         end if;
      end loop;
   end Place_Pixels;

   procedure Place_Region
     (S         : in out State;
      Req       : Place_Request;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
      Points  : Driver.World.Points.Placed_Vectors.Vector;
      Normals : Normal_Vectors.Vector;
      Asked   : Natural;
   begin
      if Req.Thing > S.Things.Last_Index or else not Has_Slot (S.Things (Req.Thing), Req.Eye) then
         return;
      end if;
      Place_Pixels (S, Req.Eye, Camera_Of, Req.Seen.Element, Tracks.Region (S.Things (Req.Thing).Eyes (Req.Eye).Track),
                    Points, Normals, Asked);
      if Asked > 0 then
         Driver.Log.Line (Driver.Log.World, "thing" & Req.Thing'Image & ": eye" & Req.Eye'Image & " placed"
                          & Points.Length'Image & " of the" & Asked'Image & " points of its region above the table");
      end if;
      if not Points.Is_Empty then
         Place_Points (S, Req.Thing, Points, Normals);
      end if;
   end Place_Region;

   function Views_Of (S : State; E : Eye_Id) return Natural is (Driver.World.Sight.Views_Of (S.Views, E));

   procedure Keep_View
     (S         : in out State;
      E         : Eye_Id;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      O         : Observation) is
   begin
      Driver.World.Sight.Keep_View (S.Views, Camera_Of, S.Table, E, O);
   end Keep_View;

   procedure Observe
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Up        : Direction_Estimate;
      Still     : Boolean;
      O         : Observation)
   is
      Moved : Thing_Flags_Access := new Thing_Flags'(S.Things.First_Index .. S.Things.Last_Index => False);
      --  The eyes of this beat, for where points fall in them.
      Held  : constant Observation_Holders.Holder := Observation_Holders.To_Holder (O);
      Seen  : constant Observation_Holders.Constant_Reference_Type := Held.Constant_Reference;
   begin
      S.Up := Up;
      --  The still pictures, kept as views the eyes place from (Driver.World.Sight).
      if Still then
         Driver.World.Sight.Keep (S.Views, Eyes, Camera_Of, S.Table, O);
      end if;
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
                     --  A region measured anew, held at a still beat: placed apart (Measure: Place_Region).
                     if Tracks.Seen (Here.Track) and then Tracks.Latest_Beat (Here.Track) = O.Beat
                       and then Tracks.Measured_At (Here.Track) /= Here.Placed_At
                     then
                        Here.Placed_At := Tracks.Measured_At (Here.Track);
                        S.To_Place.Append (Place_Request'(Thing => Id, Eye => E, Seen => Held));
                     end if;
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
            declare
               Before : constant Driver.Clock.Beat := R.Points_At;
               Had    : constant Boolean := R.Has_Points;
            begin
               Gather (S, Id, R, Camera_Of, Seen.Element);
               R.Under_Due := R.Under_Due or else R.Has_Points /= Had or else R.Points_At /= Before;
            end;
            Read_Starts (Id, R);
            Start_Elsewhere (Id, R, Eyes, Camera_Of, O, Seen.Element);
            S.Things.Replace_Element (Id, R);
         end;
      end loop;
      if (for some M of Moved.all => M) or else (for some R of S.Things => R.Under_Due) then
         for Id in Moved'Range loop
            if Moved (Id) then
               Changed (S, Id, Camera_Of, Seen.Element);
               Surfaces_Changed (S);
            end if;
         end loop;
         Refresh_Supports (S, Camera_Of, Seen.Element);
      end if;
      Free (Moved);
      Ask_Scene (S, Eyes, Still, O);
   end Observe;

   function Measure_Due (S : State) return Boolean is
     (S.Scene_Asked or else not S.To_Place.Is_Empty
      or else (for some R of S.Things => (for some Here of R.Eyes => Here.Has and then Here.Match_Due)));

   procedure Measure
     (S         : in out State;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
   begin
      --  The tracks lost at a still beat: their pixels looked for again.
      for Id in S.Things.First_Index .. S.Things.Last_Index loop
         declare
            R : Thing_Record := S.Things (Id);
         begin
            for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
               if R.Eyes (E).Has and then R.Eyes (E).Match_Due then
                  declare
                     Here : Slot := R.Eyes (E);
                  begin
                     Match_Track (Id, E, Here);
                     R.Eyes.Replace_Element (E, Here);
                  end;
               end if;
            end loop;
            S.Things.Replace_Element (Id, R);
         end;
      end loop;
      --  The regions measured anew.
      for Req of S.To_Place loop
         Place_Region (S, Req, Camera_Of);
      end loop;
      S.To_Place.Clear;
      --  The scene.
      if S.Scene_Asked then
         Measure_Scene (S, Eyes, Camera_Of);
         S.Scene_Asked := False;
      end if;
   end Measure;

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
               Put_Slot (R, E, (Holding (Fresh) with delta Pointed => True));
               R.Under_Due := True;
               S.Things.Replace_Element (Id, R);
               Thing := Id;
               return;
            end if;
         end;
      end loop;
      declare
         R : Thing_Record;
      begin
         Put_Slot (R, E, (Holding (Fresh) with delta Pointed => True));
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
      S.To_Place.Clear;
      --  The surfaces stay, as an earlier episode's, until the scene is measured again; one asked and not measured yet
      --  is asked again at the next still beat.
      S.Earlier := not S.Surfaces.Is_Empty;
      S.Scene_Asked := False;
      S.Due := True;
   end New_Episode;

   function Thing_Count (S : State) return Natural is (Natural (S.Things.Length));

   function Seen_In (S : State; T : Thing_Id; E : Eye_Id) return Boolean is
     (T <= S.Things.Last_Index and then Has_Slot (S.Things (T), E) and then Tracks.Seen (S.Things (T).Eyes (E).Track));

   function Region_In (S : State; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (if T <= S.Things.Last_Index and then Has_Slot (S.Things (T), E) then Tracks.Region (S.Things (T).Eyes (E).Track)
      else Driver.Images.Create (0, 0));

   function Points_Of (S : State; T : Thing_Id) return Driver.World.Points.Placed_Vectors.Vector is (S.Things (T).Points);

   function Points_Eye (S : State; T : Thing_Id) return Eye_Id is (S.Things (T).Points_In);

   procedure Place_Points
     (S       : in out State;
      T       : Thing_Id;
      Points  : Driver.World.Points.Placed_Vectors.Vector;
      Normals : Normal_Vectors.Vector;
      Carried : Boolean := False)
   is
      R   : Thing_Record := S.Things (T);
      Eye : constant Eye_Id'Base := (if Points.Is_Empty then 0 else Points.First_Element.Eye);
      Kept_Points  : Driver.World.Points.Placed_Vectors.Vector;
      Kept_Normals : Normal_Vectors.Vector;
   begin
      --  The points another eye placed stand; this eye's earlier ones go, and all of them when a hand carried them.
      if not Carried then
         for K in R.Placed.First_Index .. R.Placed.Last_Index loop
            if R.Placed (K).Eye /= Eye then
               Kept_Points.Append (R.Placed (K));
               Kept_Normals.Append (R.Placed_Normals (K));
            end if;
         end loop;
      end if;
      for K in Points.First_Index .. Points.Last_Index loop
         Kept_Points.Append (Points (K));
         Kept_Normals.Append (Normals (K));
      end loop;
      R.Placed := Kept_Points;
      R.Placed_Normals := Kept_Normals;
      --  The region they were placed from is not placed again.
      if not Carried and then Eye > 0 and then Has_Slot (R, Eye) then
         declare
            Here : Slot := R.Eyes (Eye);
         begin
            Here.Placed_At := Tracks.Measured_At (Here.Track);
            R.Eyes.Replace_Element (Eye, Here);
         end;
      end if;
      R.Points := Kept_Points;
      R.Normals := Kept_Normals;
      R.Has_Points := not R.Points.Is_Empty;
      R.Under_Due := True;
      S.Things.Replace_Element (T, R);
      if not Carried then
         Driver.Log.Line (Driver.Log.World, "thing" & T'Image & ":" & Points.Length'Image
                          & " points placed by eye" & Eye'Image & " of" & Kept_Points.Length'Image & " in all");
      end if;
   end Place_Points;

   function Placed_Of (S : State; T : Thing_Id) return Driver.World.Points.Placed_Vectors.Vector is
     (S.Things (T).Placed);

   function Placed_Normals_Of (S : State; T : Thing_Id) return Normal_Vectors.Vector is (S.Things (T).Placed_Normals);

   function Normals_Of (S : State; T : Thing_Id) return Normal_Vectors.Vector is (S.Things (T).Normals);

   procedure Note_Table (S : in out State; Plane : Driver.Geometry.Plane_Estimate) is
      use type Driver.Geometry.Plane_Estimate;
   begin
      if not Driver.Geometry.Known (Plane) or else Plane = S.Table then
         return;
      end if;
      declare
         --  Its reach: the whole plane, since the eye measured it as one surface wherever its points lay.
         New_Surface : constant Driver.World.Supports.Surface :=
           (Plane  => Plane, Low_1 => Real'First, High_1 => Real'Last, Low_2 => Real'First, High_2 => Real'Last,
            Members => Driver.World.Supports.Member_Vectors.Empty_Vector);
         Replaced : Boolean := False;
      begin
         if Driver.Geometry.Known (S.Table) then
            for F in S.Surfaces.First_Index .. S.Surfaces.Last_Index loop
               if S.Surfaces (F).Plane = S.Table then
                  S.Surfaces.Replace_Element (F, New_Surface);
                  Replaced := True;
               end if;
            end loop;
         end if;
         if not Replaced then
            S.Surfaces.Append (New_Surface);
         end if;
         S.Table := Plane;
         Surfaces_Changed (S);
      end;
   end Note_Table;

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
      declare
         --  The middle of what the eyes saw of it. Its points share the eyes'
         --  pose errors, so their covariance is kept as one point's, not
         --  divided by their number. The thing's own middle is not there: a
         --  solid seen from one side reaches beside what its points cover (as
         --  far as its regions show it), behind and under it, as far as its
         --  support. So the covariance also holds how far the seen points, its
         --  outline at their depth, and the space under all of them down to
         --  the support lie from that middle: the thing's middle is somewhere
         --  within that.
         N      : constant Real := Real (R.Points.Length);
         Middle : constant Vec3 := (1.0 / N) * Sum;
         Extent : Mat3 := [others => [others => 0.0]];
         Count  : Natural := 0;
         procedure Add (X : Vec3) is
         begin
            Extent := Extent + Driver.Numerics.Outer (X - Middle, X - Middle);
            Count := Count + 1;
         end Add;
         procedure With_Foot (X : Vec3) is
         begin
            Add (X);
            if R.Under.Index /= 0 then
               declare
                  P    : constant Driver.Geometry.Plane_Estimate := S.Surfaces (R.Under.Index).Plane;
                  Lean : constant Real := P.Normal * S.Up.Unit_Vector;
               begin
                  if Lean /= 0.0 then
                     --  Its foot on the support, straight down along Up.
                     Add (X - (Driver.Geometry.Height (P, X) / Lean) * S.Up.Unit_Vector);
                  end if;
               end;
            end if;
         end With_Foot;
      begin
         for M of R.Points loop
            With_Foot (M.Point.Mean);
         end loop;
         for Here of R.Eyes loop
            for X of Here.Outline loop
               With_Foot (X.Mean);
            end loop;
         end loop;
         return (Mean       => Middle,
                 Covariance => (1.0 / N) * Spread + (1.0 / Real (Count)) * Extent);
      end;
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

   function No_Gap_Seen (S : State; T : Thing_Id) return Boolean is
     (S.Things (T).Under.Index /= 0 and then S.Things (T).Under.Touching);

   function Surface_Of (S : State; F : Surface_Id) return Driver.World.Supports.Surface is (S.Surfaces (Positive (F)));

   function Scene_Round (S : State) return Natural is (S.Scene_Round);
   function Scene_Size (S : State) return Natural is (Natural (S.Scene.Length));
   function Scene_At (S : State; K : Positive) return Point_Estimate is (S.Scene (K).Point);
   function Scene_Grid_At (S : State; K : Positive) return Driver.World.Supports.Grid_Point is (S.Scene (K).Grid);

end Driver.World.Estimates;
