with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
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

   --  Everything sized by pixels, points, surfaces or things lives on the
   --  heap: the estimates also run in the decider's task, whose stack is small.
   type Answers_Access is access Driver.Instrument.Answer_Array;
   type Points_Access is access Driver.Geometry.Point_Array;
   type Grid_Access is access Driver.World.Supports.Grid_Array;
   type Flags_Access is access Driver.Geometry.Flag_Array;
   type Thing_Flags is array (Thing_Id range <>) of Boolean;
   type Thing_Flags_Access is access Thing_Flags;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Instrument.Answer_Array, Answers_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Geometry.Point_Array, Points_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.World.Supports.Grid_Array, Grid_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Geometry.Flag_Array, Flags_Access);
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
                  Answers : Answers_Access := new Driver.Instrument.Answer_Array (Points'Range);
               begin
                  Driver.Instrument.Read_Match (Reply, True, Answers.all, Ok, Why);
                  if Ok then
                     Tracks.Matched (Here.Track, Points, Answers.all);
                  else
                     Tracks.Failed (Here.Track);
                  end if;
                  Free (Answers);
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
   function Inside (Region : Driver.Images.Mask; Px : Driver.Images.Pixel) return Boolean is
     (Px.U >= 0.0 and then Px.V >= 0.0
      and then Px.U < Real (Driver.Images.Width (Region)) and then Px.V < Real (Driver.Images.Height (Region))
      and then Driver.Images.Contains (Region, Natural (Real'Floor (Px.U)), Natural (Real'Floor (Px.V))));

   function Inside_Pointed
     (R         : Thing_Record;
      X         : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean
   is
      --  A point of the thing falls inside its region in every eye that holds
      --  it where it was pointed at and sees where the point is. A pair can be
      --  wrong yet meet, when the second eye's match slid along the line the
      --  first sight draws there: the point is then on the first sight but at
      --  another depth, and falls outside the thing elsewhere. A thing that
      --  moved leaves its earlier points outside its regions. Only the regions
      --  given judge: one this layer found itself (segmented around where a
      --  pair's pixels went) can be part of the thing or something else, and
      --  would throw out the thing's own points. An eye that lost the thing,
      --  as when something passes in front of it, says nothing either way: the
      --  thing is where it was until an eye sees it elsewhere.
   begin
      for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
         if Holds (R, E) and then R.Eyes (E).Pointed then
            declare
               Px      : Driver.Images.Pixel;
               Visible : Boolean;
            begin
               Camera_Of (E, Seen).Project (X, Px, Visible);
               if Visible and then not Inside (Tracks.Region (R.Eyes (E).Track), Px) then
                  return False;
               end if;
            end;
         end if;
      end loop;
      return True;
   end Inside_Pointed;

   function Seen_Where_Pointed
     (R         : Thing_Record;
      X         : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean
   is
      --  Some eye holding the thing where it was pointed at sees where X is.
   begin
      for E in R.Eyes.First_Index .. R.Eyes.Last_Index loop
         if Holds (R, E) and then R.Eyes (E).Pointed then
            declare
               Px      : Driver.Images.Pixel;
               Visible : Boolean;
            begin
               Camera_Of (E, Seen).Project (X, Px, Visible);
               if Visible then
                  return True;
               end if;
            end;
         end if;
      end loop;
      return False;
   end Seen_Where_Pointed;

   function Bare (O : Observation) return Observation_Holders.Holder is
      --  The observation without its images: what its eyes were, kept long.
      Copy : Observation := O;
   begin
      Copy.Images.Clear;
      return Observation_Holders.To_Holder (Copy);
   end Bare;

   function In_View
     (R          : Thing_Record;
      From, Into : Eye_Id;
      Second     : Driver.World.Cameras.Camera'Class) return Boolean
   is
      --  The thing in the second eye's view, as the points the other pairs saw
      --  place it; in view when no other pair saw it, as nothing says not.
      Seen_By_Others : Boolean := False;
   begin
      for P of R.By_Pair loop
         if P.From /= From or else P.Into /= Into then
            for M of P.Kept loop
               Seen_By_Others := True;
               declare
                  Px      : Driver.Images.Pixel;
                  Visible : Boolean;
               begin
                  Second.Project (M.Point.Mean, Px, Visible);
                  if Visible then
                     return True;
                  end if;
               end;
            end loop;
         end if;
      end loop;
      return not Seen_By_Others;
   end In_View;

   procedure Gather
     (R         : in out Thing_Record;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation)
   is
      --  The thing's points: every pair's latest, but those outside it now in
      --  an eye that holds it where it was pointed at. A pair whose second
      --  eye, as it was then, saw nothing of the thing as the other pairs
      --  place it saw something else, and is dropped: its first answer came
      --  before any other said where the thing is.
      Now : Driver.World.Pairs.Match_Vectors.Vector;
      K   : Positive := 1;
   begin
      while K <= Natural (R.By_Pair.Length) loop
         declare
            Kept_It : Boolean := True;
         begin
            if not R.By_Pair (K).Then_Seen.Is_Empty then
               declare
                  Then_Seen : constant Observation_Holders.Constant_Reference_Type :=
                    R.By_Pair (K).Then_Seen.Constant_Reference;
               begin
                  Kept_It := In_View (R, R.By_Pair (K).From, R.By_Pair (K).Into,
                                      Camera_Of (R.By_Pair (K).Into, Then_Seen.Element));
               end;
            end if;
            if Kept_It then
               K := K + 1;
            else
               R.By_Pair.Delete (K);
            end if;
         end;
      end loop;
      --  A pair's point is the thing's only once another pair bears it out:
      --  it lies within the thing as the other pairs' points place it, their
      --  spread about their middle and its own uncertainty taken together, by
      --  the one significance rule. One pair alone is two eyes alone, and two
      --  eyes cannot tell a wrong match that met on the line its first sight
      --  draws from a right one; another pair, through other eyes or other
      --  pixels, can.
      for P in R.By_Pair.First_Index .. R.By_Pair.Last_Index loop
         declare
            Elsewhere : Natural := 0;   --  the other pairs' points
            Middle : Vec3 := Zero3;
            Spread : Mat3 := [others => [others => 0.0]];
         begin
            for Q in R.By_Pair.First_Index .. R.By_Pair.Last_Index loop
               if Q /= P then
                  for M of R.By_Pair (Q).Kept loop
                     Elsewhere := Elsewhere + 1;
                     Middle := Middle + M.Point.Mean;
                  end loop;
               end if;
            end loop;
            if Elsewhere >= 4 then
               Middle := (1.0 / Real (Elsewhere)) * Middle;
               for Q in R.By_Pair.First_Index .. R.By_Pair.Last_Index loop
                  if Q /= P then
                     for M of R.By_Pair (Q).Kept loop
                        Spread := Spread + Driver.Numerics.Outer (M.Point.Mean - Middle, M.Point.Mean - Middle);
                     end loop;
                  end if;
               end loop;
               Spread := (1.0 / Real (Elsewhere - 1)) * Spread;
               for M of R.By_Pair (P).Kept loop
                  declare
                     D : constant Vec3 := M.Point.Mean - Middle;
                     C : constant Mat3 := Inverse (Spread + M.Point.Covariance);
                  begin
                     if Inside_Pointed (R, M.Point.Mean, Camera_Of, Seen)
                       and then not Significant
                         (Vector_Gate (3, Elsewhere - 1, Tests => Natural'Max (1, Natural (R.By_Pair (P).Kept.Length))),
                          Sqrt (Real'Max (0.0, D * (C * D))), 1.0)
                     then
                        Now.Append (M);
                     end if;
                  end;
               end loop;
            end if;
         end;
      end loop;
      if Driver.World.Pairs.Match_Vectors."/=" (Now, R.Points) then
         R.Points := Now;
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

   function On_Thing
     (R         : Thing_Record;
      X         : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation) return Boolean;

   procedure Consistent
     (S         : State;
      R         : Thing_Record;
      Eye       : Vec3;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Seen      : not null access constant Observation;
      Kept      : in out Driver.World.Pairs.Match_Vectors.Vector;
      Hidden    : out Natural;
      Apart     : out Natural)
   is
      --  The points a pair saw that the scene and the thing's other points bear
      --  out, each by the one significance rule at its own uncertainty:
      --  - not hidden: no surface of the scene, but the thing's own faces,
      --    lies between the first eye and the point, its line of sight
      --    crossing the surface's plane inside its reach with the point
      --    significantly beyond. The eye could not have seen it there.
      --  - not apart: it lies within the thing as its points from the other
      --    pairs and earlier beats place it, their spread about their middle
      --    and its own uncertainty taken together. A wrong match whose lines
      --    still met lies on the first eye's sight at another depth, often by
      --    metres, where nothing of the thing was.
      --  The thing is still between pushes, and its points are remembered: a
      --  new point that disagrees with them is not the thing. Before the thing
      --  has points enough to spread (four, for a spread in three dimensions),
      --  only the scene bears them out.
      Tests    : constant Positive := Natural'Max (1, Natural (Kept.Length));
      Own_Face : Flags_Access :=
        new Driver.Geometry.Flag_Array'(S.Surfaces.First_Index .. S.Surfaces.Last_Index => False);
      N        : constant Natural := Natural (R.Points.Length);
      Middle   : Vec3 := Zero3;
      Spread   : Mat3 := [others => [others => 0.0]];
      Kept_Now : Driver.World.Pairs.Match_Vectors.Vector;
      function Own (Member : Positive) return Boolean is (On_Thing (R, S.Scene (Member).Point.Mean, Camera_Of, Seen));
   begin
      Hidden := 0;
      Apart := 0;
      for F in Own_Face'Range loop
         Own_Face (F) := Driver.World.Supports.Mostly (S.Surfaces (F), Own'Access);
      end loop;
      if N >= 4 then
         for M of R.Points loop
            Middle := Middle + M.Point.Mean;
         end loop;
         Middle := (1.0 / Real (N)) * Middle;
         for M of R.Points loop
            Spread := Spread + Driver.Numerics.Outer (M.Point.Mean - Middle, M.Point.Mean - Middle);
         end loop;
         Spread := (1.0 / Real (N - 1)) * Spread;
      end if;
      for M of Kept loop
         declare
            Behind : Boolean := False;
            Off    : Boolean := False;
         begin
            for F in Own_Face'Range loop
               exit when Behind;
               if not Own_Face (F) then
                  declare
                     P      : constant Driver.Geometry.Plane_Estimate := S.Surfaces (F).Plane;
                     Before : constant Real := Driver.Geometry.Height (P, Eye);
                     H      : constant Estimate := Driver.Geometry.Height (P, M.Point);
                  begin
                     Behind := Before /= 0.0 and then H.Value * Before < 0.0
                       and then Significant (Scalar_Gate (H.Degrees_Of_Freedom, Tests => Tests), H.Value, H.Sigma)
                       and then In_Reach (S.Surfaces (F), Eye + (Before / (Before - H.Value)) * (M.Point.Mean - Eye));
                  end;
               end if;
            end loop;
            if not Behind and then N >= 4 then
               declare
                  D : constant Vec3 := M.Point.Mean - Middle;
                  C : constant Mat3 := Inverse (Spread + M.Point.Covariance);
               begin
                  Off := Significant (Vector_Gate (3, N - 1, Tests => Tests), Sqrt (Real'Max (0.0, D * (C * D))), 1.0);
               end;
            end if;
            if Behind then
               Hidden := Hidden + 1;
            elsif Off then
               Apart := Apart + 1;
            else
               Kept_Now.Append (M);
            end if;
         end;
      end loop;
      Kept := Kept_Now;
      Free (Own_Face);
   end Consistent;

   procedure Keep_View
     (R         : in out Thing_Record;
      Answer    : Pair_Seen;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class)
   is
      --  Another view bears a pair's points out as another pair does: a wrong
      --  match that met on the line its first sight drew in the second eye
      --  does not come back to the same place once either eye looks from
      --  elsewhere. From the same view it comes back: an answer from there
      --  only replaces the pair's latest.
      Latest, Earlier : Natural := 0;   --  the pair's entries, the earlier view first

      function Elsewhere (Old : Pair_Seen) return Boolean is
         --  The thing's middle, as the old answer placed it, lies in either eye
         --  at pixels the two answers' matcher errors tell apart, by the one
         --  significance rule.
         type Two_Eyes is array (1 .. 2) of Eye_Id;
      begin
         if Old.Then_Seen.Is_Empty or else Answer.Then_Seen.Is_Empty or else Old.Kept.Is_Empty
           or else not (Old.Error < Real'Last and then Answer.Error < Real'Last)
         then
            return False;
         end if;
         declare
            Before : constant Observation_Holders.Constant_Reference_Type := Old.Then_Seen.Constant_Reference;
            Now    : constant Observation_Holders.Constant_Reference_Type := Answer.Then_Seen.Constant_Reference;
            Sigma  : constant Real := Sqrt (Old.Error ** 2 + Answer.Error ** 2);
            Gate   : constant Driver.Uncertain.Gate := Vector_Gate (2, Natural'Min (Old.Freedom, Answer.Freedom));
            Middle : Vec3 := Zero3;
         begin
            for M of Old.Kept loop
               Middle := Middle + M.Point.Mean;
            end loop;
            Middle := (1.0 / Real (Old.Kept.Length)) * Middle;
            for E of Two_Eyes'[Answer.From, Answer.Into] loop
               declare
                  Then_Px, Now_Px           : Driver.Images.Pixel;
                  Then_Visible, Now_Visible : Boolean;
               begin
                  Camera_Of (E, Before.Element).Project (Middle, Then_Px, Then_Visible);
                  Camera_Of (E, Now.Element).Project (Middle, Now_Px, Now_Visible);
                  if Then_Visible and then Now_Visible
                    and then Significant (Gate, Sqrt ((Now_Px.U - Then_Px.U) ** 2 + (Now_Px.V - Then_Px.V) ** 2), Sigma)
                  then
                     return True;
                  end if;
               end;
            end loop;
            return False;
         end;
      end Elsewhere;
   begin
      for K in R.By_Pair.First_Index .. R.By_Pair.Last_Index loop
         if R.By_Pair (K).From = Answer.From and then R.By_Pair (K).Into = Answer.Into then
            Earlier := Latest;
            Latest := K;
         end if;
      end loop;
      if Latest = 0 then
         R.By_Pair.Append (Answer);
      elsif Elsewhere (R.By_Pair (Latest)) then
         if Earlier /= 0 then
            R.By_Pair.Delete (Earlier);
         end if;
         R.By_Pair.Append (Answer);
      else
         R.By_Pair.Replace_Element (Latest, Answer);
      end if;
   end Keep_View;

   procedure Read_Crosses
     (S         : State;
      Id        : Thing_Id;
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
                  Answers : Answers_Access := new Driver.Instrument.Answer_Array (Points'Range);
                  Ok      : Boolean;
                  Why     : Unbounded_String;
                  Kept    : Driver.World.Pairs.Match_Vectors.Vector;
                  Apart   : Natural := 0;
                  Far     : Natural := 0;
                  Error   : Real := Real'Last;
                  Freedom : Natural := 0;
                  Seen    : constant Observation_Holders.Constant_Reference_Type := X.Seen.Constant_Reference;
               begin
                  Driver.Instrument.Read_Match (Driver.Services.Collect (X.Ticket), True, Answers.all, Ok, Why);
                  if Ok then
                     Driver.World.Pairs.Triangulate (Camera_Of (X.From, Seen.Element), Camera_Of (X.Into, Seen.Element),
                                                     Points, X.Own, Answers.all, Kept, Apart, Far, Error, Freedom);
                     if Kept.Is_Empty then
                        declare
                           Found : Natural := 0;
                        begin
                           for A of Answers.all loop
                              Found := Found + Boolean'Pos (A.Found);
                           end loop;
                           Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": eyes" & X.From'Image & " and"
                                            & X.Into'Image & " placed none of its" & Points'Length'Image & " pixels;"
                                            & Found'Image & " were found in eye" & X.Into'Image & ", "
                                            & (if Error < Real'Last
                                               then "the matcher erring by " & Driver.Log.Image (Error, 2) & " px"
                                               else "their round trips gave no matcher's error")
                                            & "," & Apart'Image & " whose lines did not meet," & Far'Image
                                            & " too far to place");
                        end;
                     end if;
                  else
                     Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": the instrument did not match eye"
                                      & X.From'Image & " into eye" & X.Into'Image & ": " & To_String (Why));
                  end if;
                  Free (Answers);
                  declare
                     Met_Before : constant Natural := Natural (Kept.Length);
                     Within     : Driver.World.Pairs.Match_Vectors.Vector;
                     Judged     : Natural := 0;   --  met points a pointed eye holding the thing sees
                  begin
                     --  The second eye does not see the thing now, as its other
                     --  points place it: whatever the pair matched there is not
                     --  it, however well the lines met.
                     if not Kept.Is_Empty
                       and then not In_View (R, X.From, X.Into, Camera_Of (X.Into, Seen.Element))
                     then
                        Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": eye" & X.Into'Image
                                         & " does not see it now;" & Kept.Length'Image & " points of eyes"
                                         & X.From'Image & " and" & X.Into'Image & " are not it");
                        Kept.Clear;
                     end if;
                     for M of Kept loop
                        if Seen_Where_Pointed (R, M.Point.Mean, Camera_Of, Seen.Element) then
                           Judged := Judged + 1;
                        end if;
                        if Inside_Pointed (R, M.Point.Mean, Camera_Of, Seen.Element) then
                           Within.Append (Driver.World.Pairs.Match'(M with delta First => X.From));
                        end if;
                     end loop;
                     if Met_Before > Natural (Within.Length) then
                        Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ":" & Natural'Image
                                           (Met_Before - Natural (Within.Length))
                                         & " points of eyes" & X.From'Image & " and" & X.Into'Image
                                         & " fall outside it where it was pointed at");
                     end if;
                     Kept := Within;
                     --  A region this layer found, whose points the eyes where the
                     --  thing was pointed at see and none of them inside it there,
                     --  is not the thing: it is dropped, and asks for no more pairs.
                     if Has_Slot (R, X.From) and then not R.Eyes (X.From).Pointed and then Judged > 0
                       and then Kept.Is_Empty
                     then
                        Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": its region in eye" & X.From'Image
                                         & " is not it: none of the" & Judged'Image
                                         & " points it gave falls inside it where it was pointed at");
                        R.Eyes.Replace_Element (X.From, Empty_Slot);
                     end if;
                     declare
                        Hidden, Apart_From_It : Natural;
                        Before : constant Natural := Natural (Kept.Length);
                     begin
                        Consistent (S, R, Camera_Of (X.From, Seen.Element).Pose.Pose.Translation, Camera_Of,
                                    Seen.Element, Kept, Hidden, Apart_From_It);
                        if Natural (Kept.Length) < Before then
                           Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ": of eyes" & X.From'Image & " and"
                                            & X.Into'Image & "'s points," & Hidden'Image
                                            & " lie behind a surface the first eye sees, and" & Apart_From_It'Image
                                            & " apart from the thing's other points");
                        end if;
                     end;
                     --  What the pair saw before stands until it sees the thing
                     --  again: not seeing it now says nothing of where it is.
                     if not Kept.Is_Empty then
                        Keep_View (R, (From => X.From, Into => X.Into, Kept => Kept, Error => Error,
                                       Freedom => Freedom, Then_Seen => Bare (Seen.Element.all)),
                                   Camera_Of);
                        R.Points_In := X.From;
                     end if;
                  end;
                  if not Kept.Is_Empty then
                     Driver.Log.Line (Driver.Log.World, "thing" & Id'Image & ":" & Kept.Length'Image
                                      & " points seen by eyes" & X.From'Image & " and" & X.Into'Image & ","
                                      & Apart'Image & " matches whose lines did not meet," & Far'Image
                                      & " too far to place, the matcher erring by "
                                      & Driver.Log.Image (Error, 2) & " px");
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
                           declare
                              Request : constant String :=
                                Driver.Instrument.Segment_Request
                                  (On, True, Box, [1 => (At_Pixel => Prompt, On => True)]);
                           begin
                              R.Starts.Append (Start'
                                ((Eye    => X.Into,
                                  Ticket => Driver.Services.Submit
                                              (Driver.Services.Instrument, "/segment", Request, Beat),
                                  On     => Image_Holders.To_Holder (On),
                                  Beat   => Seen.Element.Beat,
                                  Asked  => Request_Holders.To_Holder (Request))));
                           end;
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

   function Grid_Points (Image : Driver.Images.Image; Stride : Positive; Robot : Driver.Images.Mask)
     return Driver.Instrument.Point_Array
   is
      --  The centre of the middle pixel of each cell of the grid, but where
      --  the eye sees the robot. Built where it is returned, off the stack.
      Columns : constant Natural := Driver.Images.Width (Image) / Stride;
      Rows    : constant Natural := Driver.Images.Height (Image) / Stride;
      Count   : Natural := 0;
      function Cell (C, R : Natural) return Driver.Images.Pixel is
        ((U => Real (C * Stride + Stride / 2) + 0.5, V => Real (R * Stride + Stride / 2) + 0.5));
   begin
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            Count := Count + Boolean'Pos (not Inside (Robot, Cell (C, R)));
         end loop;
      end loop;
      return Points : Driver.Instrument.Point_Array (1 .. Count) do
         Count := 0;
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               if not Inside (Robot, Cell (C, R)) then
                  Count := Count + 1;
                  Points (Count) := Cell (C, R);
               end if;
            end loop;
         end loop;
      end return;
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
      type Two_Eyes is array (1 .. 2) of Eye_Id;
      Tests : constant Positive := 2 * Natural'Max (1, Natural (Points.Length));
      function "<" (A, B : Driver.World.Supports.Grid_Point) return Boolean is
        (A.Column < B.Column or else (A.Column = B.Column and then A.Row < B.Row));
      package Cell_Sets is new Ada.Containers.Ordered_Sets (Driver.World.Supports.Grid_Point, "<", Driver.World.Supports."=");
      Through : Cell_Sets.Set;

      function Sees_Through (P : Scene_Point) return Boolean is
      begin
         for E of Two_Eyes'[P.From, P.Into] loop
            declare
               Eye    : constant Vec3 := Camera_Of (E, Seen).Pose.Pose.Translation;
               Before : constant Real := Driver.Geometry.Height (Old.Plane, Eye);
               H      : constant Estimate := Driver.Geometry.Height (Old.Plane, P.Point);
            begin
               --  The eye on one side, the point significantly on the other.
               if Before /= 0.0 and then H.Value * Before < 0.0
                 and then Significant (Scalar_Gate (H.Degrees_Of_Freedom, Tests => Tests), H.Value, H.Sigma)
                 and then In_Reach (Old, Eye + (Before / (Before - H.Value)) * (P.Point.Mean - Eye))
               then
                  return True;
               end if;
            end;
         end loop;
         return False;
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
                        Answers : Answers_Access := new Driver.Instrument.Answer_Array (Points'Range);
                        Ok      : Boolean;
                        Why     : Unbounded_String;
                        Kept    : Driver.World.Pairs.Match_Vectors.Vector;
                        Apart   : Natural := 0;
                        Far     : Natural := 0;
                        Error   : Real := Real'Last;
                        Freedom : Natural := 0;
                        Seen    : constant Observation_Holders.Constant_Reference_Type :=
                          S.Round_Seen.Constant_Reference;
                        First   : constant Driver.World.Cameras.Camera'Class := Camera_Of (X.From, Seen.Element);
                        Second  : constant Driver.World.Cameras.Camera'Class := Camera_Of (X.Into, Seen.Element);
                        Robot   : constant Driver.Images.Mask := Second.Self_Mask;
                        Columns : constant Natural := Driver.Images.Width (Seen.Element.Images (X.From)) / X.Stride;
                        Added   : Natural := 0;
                     begin
                        Driver.Instrument.Read_Match (Reply, True, Answers.all, Ok, Why);
                        if Ok then
                           Driver.World.Pairs.Triangulate
                             (First, Second, Points, Points'Length, Answers.all, Kept, Apart, Far, Error, Freedom);
                        else
                           Driver.Log.Line (Driver.Log.World, "the scene: the instrument did not match eye"
                                            & X.From'Image & " into eye" & X.Into'Image & ": " & To_String (Why));
                        end if;
                        Free (Answers);
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
                                              From  => X.From,
                                              Into  => X.Into));
                              Added := Added + 1;
                           end if;
                        end loop;
                        S.Next_Column := S.Next_Column + Columns + 1;
                        if Added > 0 then
                           S.Seen_From := First.Pose.Pose.Translation;
                        end if;
                        Driver.Log.Line (Driver.Log.World, "the scene:" & Added'Image & " points seen by eyes"
                                         & X.From'Image & " and" & X.Into'Image & "," & Apart'Image
                                         & " matches whose lines did not meet," & Far'Image & " too far to place,"
                                         & " the matcher erring by "
                                         & Driver.Log.Image (Error, 2) & " px");
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
               All_Points : Points_Access := new Driver.Geometry.Point_Array (1 .. Natural (S.Incoming.Length));
               All_Grid   : Grid_Access := new Driver.World.Supports.Grid_Array (1 .. Natural (S.Incoming.Length));
               Found      : Driver.World.Supports.Surface_Vectors.Vector;
               Scene      : Scene_Point_Vectors.Vector := S.Incoming;
               Carried    : Natural := 0;
            begin
               for K in All_Points'Range loop
                  All_Points (K) := S.Incoming (K).Point;
                  All_Grid (K) := S.Incoming (K).Grid;
               end loop;
               Driver.World.Supports.Find (All_Points.all, All_Grid.all, S.Up, S.Seen_From, Found);
               --  A surface of this episode stands where this measurement neither
               --  found it again nor saw through it: eyes that look elsewhere
               --  now say nothing of it. Its points come along, so its members
               --  are points of the scene still.
               if not S.Earlier then
                  for Old of S.Surfaces loop
                     if not Found_Again (Old, Found)
                       and then not Seen_Through (Old, S.Incoming, Camera_Of, S.Round_Seen.Constant_Reference.Element)
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
               S.Incoming.Clear;
               S.Earlier := False;
               Surfaces_Changed (S);
               Driver.Log.Line (Driver.Log.World, "the scene:" & Found.Length'Image & " surfaces things can rest on, from"
                                & All_Points'Length'Image & " points," & Carried'Image
                                & " of them measured before and not seen since");
               Free (All_Points);
               Free (All_Grid);
            end;
         end if;
      end if;
   end Read_Background;

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
               Read_Crosses (S, Id, R, Camera_Of, O.Beat);
               Gather (R, Camera_Of, Seen.Element);
               R.Under_Due := R.Under_Due or else R.Has_Points /= Had or else R.Points_At /= Before;
            end;
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
      Ask_Background (S, Eyes, Camera_Of, Still, O);
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
      --  The surfaces stay, as an earlier episode's, until the scene is
      --  measured again; a round still out is dropped when it answers.
      S.Earlier := not S.Surfaces.Is_Empty;
      S.Round := S.Round + 1;
      S.Incoming.Clear;
      S.Due := True;
   end New_Episode;

   function Thing_Count (S : State) return Natural is (Natural (S.Things.Length));

   function Seen_In (S : State; T : Thing_Id; E : Eye_Id) return Boolean is
     (T <= S.Things.Last_Index and then Has_Slot (S.Things (T), E) and then Tracks.Seen (S.Things (T).Eyes (E).Track));

   function Region_In (S : State; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (if T <= S.Things.Last_Index and then Has_Slot (S.Things (T), E) then Tracks.Region (S.Things (T).Eyes (E).Track)
      else Driver.Images.Create (0, 0));

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

   function Bottom_Seen (S : State; T : Thing_Id) return Boolean is
     (S.Things (T).Under.Index /= 0 and then S.Things (T).Under.Touching);

   function Surface_Of (S : State; F : Surface_Id) return Driver.World.Supports.Surface is (S.Surfaces (Positive (F)));

   function Scene_Round (S : State) return Natural is (S.Scene_Round);
   function Scene_Size (S : State) return Natural is (Natural (S.Scene.Length));
   function Scene_At (S : State; K : Positive) return Point_Estimate is (S.Scene (K).Point);
   function Scene_Grid_At (S : State; K : Positive) return Driver.World.Supports.Grid_Point is (S.Scene (K).Grid);

end Driver.World.Estimates;
