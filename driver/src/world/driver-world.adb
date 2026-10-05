with Ada.Containers.Ordered_Maps;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Msgpack;
with Driver.Recording;
with Driver.World.Cameras;
with Driver.World.Estimates;
with Driver.World.Pairs;

package body Driver.World is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;

   type Scene_Data is record
      State : Driver.World.Estimates.State;
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
      pragma Unreferenced (H, Sent);
      function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
        return Driver.World.Cameras.Camera'Class is
        (Driver.World.Cameras.Of_Body'(Robot => M'Access, Seen => Seen, Eye => E));
   begin
      Ensure (S);
      Driver.World.Estimates.Observe
        (S.Data.State, Driver.Robot.Eye_Count (M), Camera_Of'Access, Driver.Robot.Up (M), Driver.Robot.Still (M), O);
   end Observe;

   procedure New_Episode (S : in out Scene) is
   begin
      Ensure (S);
      Driver.World.Estimates.New_Episode (S.Data.State);
   end New_Episode;

   function Thing_Count (S : Scene) return Natural is
     (if S.Data = null then 0 else Driver.World.Estimates.Thing_Count (S.Data.State));

   --  A decider's write goes into the recording (kind W) before it is
   --  applied, as a msgpack array naming it; reals travel as float64, exact.
   procedure Record_Write (B : Driver.Bytes.Buffer) is
   begin
      Driver.Recording.Write_Shared (Driver.Recording.World_Written, B.To_Array);
   end Record_Write;

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
            Driver.Msgpack.Put_Array_Header (B, 6);
            Driver.Msgpack.Put_String (B, "adopt");
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (E));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (O.Beat));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (W));
            Driver.Msgpack.Put_Integer (B, Long_Long_Integer (H));
            Driver.Msgpack.Put_Binary (B, Bits);
            Record_Write (B);
         end;
      end if;
      Ensure (S);
      Driver.World.Estimates.Adopt (S.Data.State, E, O, Region, Thing);
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

   --  Not measured yet: what holds a thing, and its motion.

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base is (0);

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
      Points : constant Driver.World.Pairs.Match_Vectors.Vector :=
        (if Known_Thing (S, T) then Driver.World.Estimates.Points_Of (S.Data.State, T)
         else Driver.World.Pairs.Match_Vectors.Empty_Vector);
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
      function Key_Of (M : Driver.World.Pairs.Match) return Key is
        ((Eye => M.First, Column => Integer (Real'Floor (M.In_First.U)), Row => Integer (Real'Floor (M.In_First.V))));
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
               --  The surface's normal from the points beside it in the image;
               --  none where a neighbour is missing.
               if At_Key ((K.Eye, K.Column - 1, K.Row), Left) and then At_Key ((K.Eye, K.Column + 1, K.Row), Right)
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
      Points : constant Driver.World.Pairs.Match_Vectors.Vector :=
        (if Known_Thing (S, T) then Driver.World.Estimates.Points_Of (S.Data.State, T)
         else Driver.World.Pairs.Match_Vectors.Empty_Vector);
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
         if What = "remember" and then N = 2 then
            declare
               Place : Place_Id;
            begin
               Remember (S, Point_Of (Doc, Element (Doc, R, 2)), Place);
            end;
         elsif What = "touched" and then N = 3 then
            Touched (S, Thing_Id (Whole (2)), Point_Of (Doc, Element (Doc, R, 3)));
         elsif What = "friction" and then N = 4 then
            Learn_Friction (S, Thing_Id (Whole (2)),
                            (Low => Number (Doc, Element (Doc, R, 3)), High => Number (Doc, Element (Doc, R, 4))));
         elsif What = "adopt" and then N = 6 then
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
               Adopt (S, M, Eye_Id (Whole (2)), O, Region, Thing);
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
