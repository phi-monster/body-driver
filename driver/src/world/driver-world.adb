with Ada.Containers.Ordered_Maps;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Msgpack;
with Driver.Recording;
with Driver.World.Cameras;
with Driver.Log;
with Driver.World.Estimates;
with Driver.World.Holding;
with Driver.World.Points;

package body Driver.World is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;

   --  A thing a hand holds (Hold): the hand, and once a beat has measured them the arm's tool, the closer's
   --  fraction and the thing's placed points as they were when it took the thing.
   type Hold_Record is record
      Thing   : Thing_Id;
      Hand    : Driver.Robot.Hand.Hand_Id;
      Taken   : Boolean := False;
      Tool    : Pose_Estimate;
      Closer  : Estimate;
      Points  : Driver.World.Points.Placed_Vectors.Vector;
      Normals : Driver.World.Estimates.Normal_Vectors.Vector;
   end record;

   package Hold_Vectors is new Ada.Containers.Vectors (Positive, Hold_Record);

   package Count_Vectors is new Ada.Containers.Vectors (Positive, Natural);

   type Scene_Data is record
      State : Driver.World.Estimates.State;
      Holds : Hold_Vectors.Vector;
      Swept : Count_Vectors.Vector;   --  per eye, how many of its arm's sweep's pictures are kept as views
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

   function Known_Thing (S : Scene; T : Thing_Id) return Boolean is
     (S.Data /= null and then Natural (T) <= Driver.World.Estimates.Thing_Count (S.Data.State));

   procedure Observe
     (S    : in out Scene;
      M    : Driver.Robot.Model;
      H    : Driver.Robot.Hand.Hands;
      O    : Observation;
      Sent : Driver.Commands.Command)
   is
      pragma Unreferenced (Sent);
      function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
        return Driver.World.Cameras.Camera'Class is
        (Driver.World.Cameras.Of_Body'(Robot => M'Access, Seen => Seen, Eye => E));
   begin
      Ensure (S);
      --  The table the first arm's eye measured (the world is that arm's frame): a surface things rest on, whether
      --  or not the scene's points show it.
      Driver.World.Estimates.Note_Table (S.Data.State, Driver.Robot.World_Table (M));
      --  The pictures each arm's sweep held its eye still for, kept as views of it from the beat its arm is fitted
      --  on: the places the body has seen the scene from before any still beat of the world's own. A sweep begun
      --  anew gives its pictures again.
      for E in 1 .. Eye_Id'Base (Driver.Robot.Eye_Count (M)) loop
         if Natural (E) <= Natural (O.Images.Length)
           and then Natural (O.Readings.Length) = Driver.Robot.Group_Count (M)
         then
            while Natural (S.Data.Swept.Length) < Natural (E) loop
               S.Data.Swept.Append (0);
            end loop;
            declare
               Swept : constant Natural := Driver.Robot.Sweep_Views (M, E);
               Taken : Natural := S.Data.Swept (Positive (E));
            begin
               if Swept < Taken then
                  Taken := 0;
               end if;
               for K in Taken + 1 .. Swept loop
                  Driver.World.Estimates.Keep_View
                    (S.Data.State, E, Camera_Of'Access, Driver.Robot.Sweep_View (M, E, K, O));
               end loop;
               S.Data.Swept.Replace_Element (Positive (E), Swept);
            end;
         end if;
      end loop;
      Driver.World.Estimates.Observe
        (S.Data.State, Driver.Robot.Eye_Count (M), Camera_Of'Access, Driver.Robot.Up (M), Driver.Robot.Still (M), O);
      --  What the hands hold goes with their arms (Driver.World.Holding).
      for K in reverse S.Data.Holds.First_Index .. S.Data.Holds.Last_Index loop
         declare
            Hd : Hold_Record := S.Data.Holds (K);
         begin
            if not Driver.Robot.Hand.Exists (H, Hd.Hand) then
               Driver.Log.Line (Driver.Log.World, "thing" & Hd.Thing'Image & ": hand" & Hd.Hand'Image
                                & " that held it is no longer measured; it stays where it was carried to");
               S.Data.Holds.Delete (K);
            else
               declare
                  Tool   : constant Pose_Estimate :=
                    Driver.Robot.Tool_Pose (M, Driver.Robot.Hand.Arm_Of (H, Hd.Hand), O);
                  Closer : constant Estimate := Driver.Robot.Hand.Fraction (H, M, Hd.Hand, O);
                  Posed  : constant Boolean := Tool.Position_Covariance (1, 1) < Real'Last
                                               and then Tool.Rotation_Covariance (1, 1) < Real'Last;
               begin
                  if not Hd.Taken then
                     if Posed and then Known (Closer) then
                        Hd := (Thing   => Hd.Thing, Hand => Hd.Hand, Taken => True, Tool => Tool, Closer => Closer,
                               Points  => Driver.World.Estimates.Placed_Of (S.Data.State, Hd.Thing),
                               Normals => Driver.World.Estimates.Placed_Normals_Of (S.Data.State, Hd.Thing));
                        S.Data.Holds.Replace_Element (K, Hd);
                        Driver.Log.Line (Driver.Log.World, "thing" & Hd.Thing'Image & ": held by hand" & Hd.Hand'Image
                                         & ", its closer at " & Driver.Log.Image (Closer.Value, 3) & " +- "
                                         & Driver.Log.Image (Closer.Sigma, 3) & "; its" & Hd.Points.Length'Image
                                         & " placed points go with the arm while the closer stays short of closed");
                     end if;
                  elsif not Driver.World.Holding.Still_Held (Hd.Closer, Closer) then
                     Driver.Log.Line (Driver.Log.World, "thing" & Hd.Thing'Image & ": let go by hand" & Hd.Hand'Image
                                      & ", its closer at "
                                      & (if Known (Closer) then Driver.Log.Image (Closer.Value, 3) else "unknown")
                                      & " (it held at " & Driver.Log.Image (Hd.Closer.Value, 3)
                                      & "); it stays where it was carried to");
                     S.Data.Holds.Delete (K);
                  elsif Posed and then not Hd.Points.Is_Empty then
                     declare
                        Points  : Driver.World.Points.Placed_Vectors.Vector := Hd.Points;
                        Normals : Driver.World.Estimates.Normal_Vectors.Vector := Hd.Normals;
                     begin
                        Driver.World.Holding.Carry (Points, Normals, Hd.Tool, Tool);
                        Driver.World.Estimates.Place_Points (S.Data.State, Hd.Thing, Points, Normals, Carried => True);
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Observe;

   function Measure_Due (S : Scene) return Boolean is
     (S.Data /= null and then Driver.World.Estimates.Measure_Due (S.Data.State));

   procedure Measure (S : in out Scene; M : Driver.Robot.Model) is
      function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
        return Driver.World.Cameras.Camera'Class is
        (Driver.World.Cameras.Of_Body'(Robot => M'Access, Seen => Seen, Eye => E));
   begin
      Ensure (S);
      Driver.World.Estimates.Measure (S.Data.State, Driver.Robot.Eye_Count (M), Camera_Of'Access);
   end Measure;

   function Free_Pixel (S : Scene; E : Eye_Id; Width, Height : Positive) return Driver.Images.Pixel is
     (if S.Data = null then (U => Real (Width) / 2.0, V => Real (Height) / 2.0)
      else Driver.World.Estimates.Free_Pixel (S.Data.State, E, Width, Height));

   function Scene_Pending (S : Scene) return Boolean is
     (S.Data /= null and then Driver.World.Estimates.Scene_Pending (S.Data.State));

   function Views_Of (S : Scene; E : Eye_Id) return Natural is
     (if S.Data = null then 0 else Driver.World.Estimates.Views_Of (S.Data.State, E));

   function Placed_In
     (S      : Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask) return Driver.Robot.Surfaces.Surface_Point_Array
   is
      function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
        return Driver.World.Cameras.Camera'Class is
        (Driver.World.Cameras.Of_Body'(Robot => M'Access, Seen => Seen, Eye => E));
      Points  : Driver.World.Points.Placed_Vectors.Vector;
      Normals : Driver.World.Estimates.Normal_Vectors.Vector;
      Asked   : Natural := 0;
      Counts  : Driver.Robot.Surfaces.Placing_Counts;
   begin
      if S.Data /= null then
         Driver.World.Estimates.Place_Pixels
           (S.Data.State, E, Camera_Of'Access, O, Region, Points, Normals, Asked, Counts);
         Driver.Log.Line (Driver.Log.World, "the region asked of eye" & E'Image & ": "
                          & Natural (Points.Length)'Image & " of its points stand above the table ("
                          & Driver.Robot.Surfaces.Image (Counts) & ")");
      end if;
      return Result : Driver.Robot.Surfaces.Surface_Point_Array (1 .. Natural (Points.Length)) do
         for K in Result'Range loop
            Result (K) := (Pixel  => Points (K).Pixel,
                           Point  => Points (K).Point,
                           Normal => Normals (K));
         end loop;
      end return;
   end Placed_In;

   procedure New_Episode (S : in out Scene) is
   begin
      Ensure (S);
      Driver.World.Estimates.New_Episode (S.Data.State);
      S.Data.Holds.Clear;
   end New_Episode;

   function Thing_Count (S : Scene) return Natural is
     (if S.Data = null then 0 else Driver.World.Estimates.Thing_Count (S.Data.State));

   --  A decider's write goes into the recording (kind W) before it is
   --  applied, as a msgpack array naming it; reals travel as float64, exact.
   procedure Record_Write (B : Driver.Bytes.Buffer) is
   begin
      Driver.Recording.Write_Shared (Driver.Recording.World_Written, B.To_Array);
   end Record_Write;

   procedure Seen_Anew (S : in out Scene) is
   begin
      if Driver.Recording.Shared_Started then
         declare
            B : Driver.Bytes.Buffer;
         begin
            Driver.Msgpack.Put_Array_Header (B, 1);
            Driver.Msgpack.Put_String (B, "seen anew");
            Record_Write (B);
         end;
      end if;
      Ensure (S);
      Driver.World.Estimates.Seen_Anew (S.Data.State);
   end Seen_Anew;

   procedure Put_Point (B : in out Driver.Bytes.Buffer; P : Point_Estimate) is
   begin
      Driver.Msgpack.Put_Array_Header (B, Vec3'Length + Mat3'Length (1) * Mat3'Length (2));
      for V of P.Mean loop
         Driver.Msgpack.Put_Float (B, V);
      end loop;
      for V of P.Covariance loop
         Driver.Msgpack.Put_Float (B, V);
      end loop;
   end Put_Point;

   function Point_Of (Doc : Driver.Msgpack.Document; N : Driver.Msgpack.Node) return Point_Estimate is
      P : Point_Estimate;
      K : Positive := 1;
   begin
      for V of P.Mean loop
         V := Driver.Msgpack.Number (Doc, Driver.Msgpack.Element (Doc, N, K));
         K := K + 1;
      end loop;
      for V of P.Covariance loop
         V := Driver.Msgpack.Number (Doc, Driver.Msgpack.Element (Doc, N, K));
         K := K + 1;
      end loop;
      return P;
   end Point_Of;

   procedure Adopt
     (S      : in out Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask;
      Placed : Driver.Robot.Surfaces.Surface_Point_Array;
      Thing  : out Thing_Id)
   is
      pragma Unreferenced (M);
   begin
      if Driver.Recording.Shared_Started then
         declare
            use type Driver.Bytes.Byte;
            W    : constant Natural := Driver.Images.Width (Region);
            H    : constant Natural := Driver.Images.Height (Region);
            Bits : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset ((W * H + 7) / 8)) := [others => 0];
            B    : Driver.Bytes.Buffer;
         begin
            --  Row-major, the first pixel in the lowest bit of the first byte.
            for Y in 0 .. H - 1 loop
               for X in 0 .. W - 1 loop
                  if Driver.Images.Contains (Region, X, Y) then
                     Bits (Driver.Bytes.Offset ((Y * W + X) / 8 + 1)) :=
                       Bits (Driver.Bytes.Offset ((Y * W + X) / 8 + 1)) or 2 ** ((Y * W + X) mod 8);
                  end if;
               end loop;
            end loop;
            Driver.Msgpack.Put_Array_Header (B, 7);
            Driver.Msgpack.Put_String (B, "adopt");
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (E));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (O.Beat));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (W));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (H));
            Driver.Msgpack.Put_Binary (B, Bits);
            --  The points the eye placed, as placed: a replay applies them as the run had them.
            Driver.Msgpack.Put_Array_Header (B, Placed'Length);
            for P of Placed loop
               Driver.Msgpack.Put_Array_Header (B, 4);
               Driver.Msgpack.Put_Float (B, P.Pixel.U);
               Driver.Msgpack.Put_Float (B, P.Pixel.V);
               Put_Point (B, P.Point);
               Driver.Msgpack.Put_Array_Header (B, Vec3'Length);
               for V of P.Normal loop
                  Driver.Msgpack.Put_Float (B, V);
               end loop;
            end loop;
            Record_Write (B);
         end;
      end if;
      Ensure (S);
      Driver.World.Estimates.Adopt (S.Data.State, E, O, Region, Thing);
      declare
         Points  : Driver.World.Points.Placed_Vectors.Vector;
         Normals : Driver.World.Estimates.Normal_Vectors.Vector;
      begin
         for P of Placed loop
            Points.Append (Driver.World.Points.Placed'(Pixel => P.Pixel, Point => P.Point, Eye => E));
            Normals.Append (P.Normal);
         end loop;
         if not Points.Is_Empty then
            Driver.World.Estimates.Place_Points (S.Data.State, Thing, Points, Normals);
         end if;
      end;
   end Adopt;

   procedure Adopt
     (S      : in out Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask;
      Thing  : out Thing_Id)
   is
      None : Driver.Robot.Surfaces.Surface_Point_Array (1 .. 0);
   begin
      Adopt (S, M, E, O, Region, None, Thing);
   end Adopt;

   function Seen_In (S : Scene; T : Thing_Id; E : Eye_Id) return Boolean is
     (Known_Thing (S, T) and then Driver.World.Estimates.Seen_In (S.Data.State, T, E));

   function Region_In (S : Scene; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (Driver.World.Estimates.Region_In (S.Data.State, T, E));

   function Centre (S : Scene; T : Thing_Id) return Point_Estimate is
      Unmeasured : Point_Estimate;
   begin
      return (if Known_Thing (S, T) then Driver.World.Estimates.Centre (S.Data.State, T) else Unmeasured);
   end Centre;

   function Resting_On (S : Scene; T : Thing_Id) return Surface_Id'Base is
     (if Known_Thing (S, T) then Driver.World.Estimates.Resting_On (S.Data.State, T) else 0);

   function Height_Above_Support (S : Scene; T : Thing_Id) return Estimate is
     (if Known_Thing (S, T) then Driver.World.Estimates.Height_Above_Support (S.Data.State, T) else Unknown);

   function No_Gap_Seen (S : Scene; T : Thing_Id) return Boolean is
     (Known_Thing (S, T) and then Driver.World.Estimates.No_Gap_Seen (S.Data.State, T));

   procedure Hold (S : in out Scene; T : Thing_Id; Hand : Driver.Robot.Hand.Hand_Id) is
   begin
      if Driver.Recording.Shared_Started then
         declare
            B : Driver.Bytes.Buffer;
         begin
            Driver.Msgpack.Put_Array_Header (B, 3);
            Driver.Msgpack.Put_String (B, "hold");
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (T));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (Hand));
            Record_Write (B);
         end;
      end if;
      Ensure (S);
      --  One hand holds a thing: a new hold of it is the one that stands.
      for K in reverse S.Data.Holds.First_Index .. S.Data.Holds.Last_Index loop
         if S.Data.Holds (K).Thing = T then
            S.Data.Holds.Delete (K);
         end if;
      end loop;
      S.Data.Holds.Append (Hold_Record'(Thing => T, Hand => Hand, others => <>));
   end Hold;

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base is
   begin
      if S.Data /= null then
         for Hd of S.Data.Holds loop
            if Hd.Thing = T then
               return Hd.Hand;
            end if;
         end loop;
      end if;
      return 0;
   end Held_By;

   --  Not measured yet: a thing's motion.

   function Moving (S : Scene; T : Thing_Id) return Boolean is (False);

   procedure Remember (S : in out Scene; Point : Point_Estimate; Place : out Place_Id) is
   begin
      if Driver.Recording.Shared_Started then
         declare
            B : Driver.Bytes.Buffer;
         begin
            Driver.Msgpack.Put_Array_Header (B, 2);
            Driver.Msgpack.Put_String (B, "remember");
            Put_Point (B, Point);
            Record_Write (B);
         end;
      end if;
      Ensure (S);
      Driver.World.Estimates.Remember (S.Data.State, Point, Place);
   end Remember;

   function Where (S : Scene; P : Place_Id) return Point_Estimate is (Driver.World.Estimates.Where (S.Data.State, P));

   function Samples (S : Scene; T : Thing_Id) return Sample_Array is
      Points : constant Driver.World.Points.Placed_Vectors.Vector :=
        (if Known_Thing (S, T) then Driver.World.Estimates.Points_Of (S.Data.State, T)
         else Driver.World.Points.Placed_Vectors.Empty_Vector);
      --  The normals the eye that placed a point gave it (Place_Points); zero for the pairs' points.
      Placed_Normals : constant Driver.World.Estimates.Normal_Vectors.Vector :=
        (if Known_Thing (S, T) then Driver.World.Estimates.Normals_Of (S.Data.State, T)
         else Driver.World.Estimates.Normal_Vectors.Empty_Vector);
      --  Each point by the pixel of the first eye it was seen at, so a
      --  point's neighbours on the surface are its neighbours in the image.
      type Key is record
         Eye         : Eye_Id;
         Column, Row : Integer;
      end record;
      function "<" (A, B : Key) return Boolean is
        (A.Eye < B.Eye or else (A.Eye = B.Eye and then (A.Row < B.Row or else (A.Row = B.Row and then A.Column < B.Column))));
      package Point_Maps is new Ada.Containers.Ordered_Maps (Key, Positive);
      Index : Point_Maps.Map;
      function Key_Of (M : Driver.World.Points.Placed) return Key is
        ((Eye => M.Eye, Column => Integer (Real'Floor (M.Pixel.U)), Row => Integer (Real'Floor (M.Pixel.V))));
      function At_Key (K : Key; Found : out Vec3) return Boolean is
         C : constant Point_Maps.Cursor := Index.Find (K);
      begin
         Found := Zero3;
         if Point_Maps.Has_Element (C) then
            Found := Points (Point_Maps.Element (C)).Point.Mean;
            return True;
         end if;
         return False;
      end At_Key;
   begin
      for I in 1 .. Natural (Points.Length) loop
         Index.Include (Key_Of (Points (I)), I);
      end loop;
      return Result : Sample_Array (1 .. Natural (Points.Length)) do
         for I in Result'Range loop
            declare
               K : constant Key := Key_Of (Points (I));
               Left, Right, Above, Below : Vec3;
               Normal : Vec3 := Zero3;
            begin
               --  The normal the eye placed the point with, else the surface's normal from the points beside it in
               --  the image; none where a neighbour is missing.
               if I <= Natural (Placed_Normals.Length) and then abs Placed_Normals (I) > 0.0 then
                  Normal := Placed_Normals (I);
               elsif At_Key ((K.Eye, K.Column - 1, K.Row), Left) and then At_Key ((K.Eye, K.Column + 1, K.Row), Right)
                 and then At_Key ((K.Eye, K.Column, K.Row - 1), Above)
                 and then At_Key ((K.Eye, K.Column, K.Row + 1), Below)
               then
                  declare
                     N : constant Vec3 := Cross (Right - Left, Below - Above);
                  begin
                     if abs N > 0.0 then
                        Normal := Unit (N);
                     end if;
                  end;
               end if;
               Result (I) := (Point => Points (I).Point.Mean, Normal => Normal, Seen => True);
            end;
         end loop;
      end return;
   end Samples;

   function Sample_Sigma (S : Scene; T : Thing_Id) return Real is
      Points : constant Driver.World.Points.Placed_Vectors.Vector :=
        (if Known_Thing (S, T) then Driver.World.Estimates.Points_Of (S.Data.State, T)
         else Driver.World.Points.Placed_Vectors.Empty_Vector);
      Sum : Real := 0.0;
   begin
      if Points.Is_Empty then
         return Real'Last;
      end if;
      --  A sample's position uncertainty: its covariance's mean variance per
      --  axis, averaged over the samples.
      for M of Points loop
         Sum := Sum + (M.Point.Covariance (1, 1) + M.Point.Covariance (2, 2) + M.Point.Covariance (3, 3)) / 3.0;
      end loop;
      return Sqrt (Sum / Real (Points.Length));
   end Sample_Sigma;

   procedure Touched (S : in out Scene; T : Thing_Id; Point : Point_Estimate) is
   begin
      if Driver.Recording.Shared_Started then
         declare
            B : Driver.Bytes.Buffer;
         begin
            Driver.Msgpack.Put_Array_Header (B, 3);
            Driver.Msgpack.Put_String (B, "touched");
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (T));
            Put_Point (B, Point);
            Record_Write (B);
         end;
      end if;
      Driver.World.Estimates.Touched (S.Data.State, T, Point);
   end Touched;

   procedure Learn_Friction (S : in out Scene; T : Thing_Id; Bounds : Friction_Bounds) is
   begin
      if Driver.Recording.Shared_Started then
         declare
            B : Driver.Bytes.Buffer;
         begin
            Driver.Msgpack.Put_Array_Header (B, 4);
            Driver.Msgpack.Put_String (B, "friction");
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (T));
            Driver.Msgpack.Put_Float (B, Bounds.Low);
            Driver.Msgpack.Put_Float (B, Bounds.High);
            Record_Write (B);
         end;
      end if;
      Driver.World.Estimates.Learn_Friction (S.Data.State, T, Bounds);
   end Learn_Friction;

   function Friction (S : Scene; T : Thing_Id) return Friction_Bounds is
     (Driver.World.Estimates.Friction (S.Data.State, T));

   procedure Replay_Write
     (S    : in out Scene;
      M    : Driver.Robot.Model;
      O    : Observation;
      Data : Driver.Bytes.Byte_Array;
      Ok   : out Boolean)
   is
      use Driver.Msgpack;
      Doc : Document;
      R   : Node;
      function Whole (I : Positive) return Long_Long_Integer is (Long_Long_Integer (Number (Doc, Element (Doc, R, I))));
   begin
      Decode (Data, Doc, Ok);
      if not Ok then
         return;
      end if;
      R := Root (Doc);
      Ok := Kind_Of (Doc, R) = Array_Value and then Count (Doc, R) > 0
        and then Kind_Of (Doc, Element (Doc, R, 1)) = String_Value;
      if not Ok then
         return;
      end if;
      declare
         What : constant String := Text (Doc, Element (Doc, R, 1));
         N    : constant Natural := Count (Doc, R);
      begin
         if What = "seen anew" and then N = 1 then
            Seen_Anew (S);
         elsif What = "remember" and then N = 2 then
            declare
               Place : Place_Id;
            begin
               Remember (S, Point_Of (Doc, Element (Doc, R, 2)), Place);
            end;
         elsif What = "touched" and then N = 3 then
            Touched (S, Thing_Id (Whole (2)), Point_Of (Doc, Element (Doc, R, 3)));
         elsif What = "hold" and then N = 3 then
            Hold (S, Thing_Id (Whole (2)), Driver.Robot.Hand.Hand_Id (Whole (3)));
         elsif What = "friction" and then N = 4 then
            Learn_Friction (S, Thing_Id (Whole (2)),
                            (Low => Number (Doc, Element (Doc, R, 3)), High => Number (Doc, Element (Doc, R, 4))));
         elsif What = "adopt" and then N in 6 .. 7 then
            declare
               W      : constant Natural := Natural (Whole (4));
               H      : constant Natural := Natural (Whole (5));
               Region : Driver.Images.Mask := Driver.Images.Create (W, H);
               Thing  : Thing_Id;
               procedure Unpack (Bits : Driver.Bytes.Byte_Array) is
                  use type Driver.Bytes.Byte;
                  use type Driver.Bytes.Offset;
               begin
                  for I in 0 .. W * H - 1 loop
                     if Bits (Bits'First + Driver.Bytes.Offset (I / 8)) / 2 ** (I mod 8) mod 2 = 1 then
                        Driver.Images.Include (Region, I mod W, I / W);
                     end if;
                  end loop;
               end Unpack;
            begin
               Read_Binary (Doc, Element (Doc, R, 6), Unpack'Access);
               if N = 7 then
                  --  The points the run's eye placed, as it placed them.
                  declare
                     List   : constant Node := Element (Doc, R, 7);
                     Placed : Driver.Robot.Surfaces.Surface_Point_Array (1 .. Count (Doc, List));
                  begin
                     for K in Placed'Range loop
                        declare
                           P : constant Node := Element (Doc, List, K);
                           Nm : constant Node := Element (Doc, P, 4);
                        begin
                           Placed (K) := (Pixel  => (U => Number (Doc, Element (Doc, P, 1)),
                                                     V => Number (Doc, Element (Doc, P, 2))),
                                          Point  => Point_Of (Doc, Element (Doc, P, 3)),
                                          Normal => [for A in 1 .. 3 => Number (Doc, Element (Doc, Nm, A))]);
                        end;
                     end loop;
                     Adopt (S, M, Eye_Id (Whole (2)), O, Region, Placed, Thing);
                  end;
               else
                  Adopt (S, M, Eye_Id (Whole (2)), O, Region, Thing);
               end if;
            end;
         else
            Ok := False;
         end if;
      end;
   end Replay_Write;

   function Predicted (S : Scene; T : Thing_Id; Beats : Natural) return Point_Estimate is
      pragma Unreferenced (Beats);
   begin
      return Centre (S, T);
   end Predicted;

   function Surface_Count (S : Scene) return Natural is
     (if S.Data = null then 0 else Driver.World.Estimates.Surface_Count (S.Data.State));

   function Plane_Of (S : Scene; F : Surface_Id) return Driver.Geometry.Plane_Estimate is
     (Driver.World.Estimates.Plane_Of (S.Data.State, F));

   function Earlier (S : Scene; F : Surface_Id) return Boolean is (Driver.World.Estimates.Earlier (S.Data.State, F));

   function Place_Count (S : Scene) return Natural is
     (if S.Data = null then 0 else Driver.World.Estimates.Place_Count (S.Data.State));

end Driver.World;
