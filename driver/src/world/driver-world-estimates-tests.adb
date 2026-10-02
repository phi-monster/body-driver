with Ada.Containers.Vectors;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Json;
with Driver.Tests;
with Driver.World.Regions;
with Driver.World.Tests;
with Driver.World.Tracking;

package body Driver.World.Estimates.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Services.Ticket;
   use type Ada.Streams.Stream_Element_Offset;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  The generator returns [0, 1] with 1 included: U1 is drawn on (0, 1].
      U1 : Real;
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      loop
         U1 := Real (Ada.Numerics.Float_Random.Random (Gen));
         exit when U1 > 0.0;
      end loop;
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  The scene: a table 1 m square at z = 0 with a box 0.2 m square and
   --  0.08 m high in its middle, seen by two eyes from the same side.
   Columns    : constant := 160;
   Rows       : constant := 120;
   Box_Half   : constant := 0.1;
   Box_Top    : constant := 0.08;
   Table_Half : constant := 0.5;
   Match      : constant := 0.3;   --  the matcher's error, pixels per coordinate

   Eyes : constant array (Eye_Id range 1 .. 2) of Driver.World.Tests.Pinhole :=
     [Driver.World.Tests.Looking_At ([-0.15, -0.55, 0.55], [0.0, 0.0, 0.0], 150.0, Columns, Rows, 0.3),
      Driver.World.Tests.Looking_At ([0.15, -0.55, 0.55], [0.0, 0.0, 0.0], 150.0, Columns, Rows, 0.3)];

   Up : constant Direction_Estimate := (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 0.001);

   type Surface_Hit is (Nothing, Table, Box);

   procedure First_Hit (Origin, Direction : Vec3; Hit : out Surface_Hit; At_T : out Real) is
      Near : Real := Real'First;
      Far  : Real := Real'Last;
      Low  : constant Vec3 := [-Box_Half, -Box_Half, 0.0];
      High : constant Vec3 := [Box_Half, Box_Half, Box_Top];
      Into_Box : Boolean := True;
   begin
      Hit := Nothing;
      At_T := Real'Last;
      for A in 1 .. 3 loop
         if Direction (A) = 0.0 then
            Into_Box := Into_Box and then Origin (A) >= Low (A) and then Origin (A) <= High (A);
         else
            declare
               T1 : constant Real := (Low (A) - Origin (A)) / Direction (A);
               T2 : constant Real := (High (A) - Origin (A)) / Direction (A);
            begin
               Near := Real'Max (Near, Real'Min (T1, T2));
               Far := Real'Min (Far, Real'Max (T1, T2));
            end;
         end if;
      end loop;
      if Into_Box and then Near <= Far and then Near > 0.0 then
         Hit := Box;
         At_T := Near;
      end if;
      if Direction (3) /= 0.0 then
         declare
            T : constant Real := -Origin (3) / Direction (3);
            P : constant Vec3 := Origin + T * Direction;
         begin
            if T > 0.0 and then T < At_T and then abs P (1) <= Table_Half and then abs P (2) <= Table_Half then
               Hit := Table;
               At_T := T;
            end if;
         end;
      end if;
   end First_Hit;

   function Number (X : Real) return String renames Driver.Json.Number_Image;

   function Reply_Text (From, Into : Driver.World.Tests.Pinhole; Points : Driver.Instrument.Point_Array)
     return String
   is
      --  What a matcher answers: where the point the first eye sees there is in
      --  the second, if the second sees it at all, and the way back, each match
      --  in error by the matcher's own noise.
      Text : Unbounded_String := To_Unbounded_String ("{""ok"":true,""points"":[");
      Back : Unbounded_String := To_Unbounded_String ("],""back"":[");
   begin
      for I in Points'Range loop
         declare
            R       : constant Ray_Estimate := From.Ray (Points (I));
            Hit     : Surface_Hit;
            T       : Real;
            Px      : Driver.Images.Pixel;
            Visible : Boolean := False;
         begin
            First_Hit (R.Origin.Mean, R.Direction.Unit_Vector, Hit, T);
            if Hit /= Nothing then
               declare
                  X     : constant Vec3 := R.Origin.Mean + T * R.Direction.Unit_Vector;
                  C     : constant Vec3 := Into.Pose_In_World.Translation;
                  D     : constant Vec3 := Unit (X - C);
                  Again : Surface_Hit;
                  T2    : Real;
               begin
                  Into.Project (X, Px, Visible);
                  --  Hidden from the second eye behind something nearer: no match.
                  First_Hit (C, D, Again, T2);
                  Visible := Visible and then Again /= Nothing and then abs (C + T2 * D - X) < 1.0e-6;
               end;
            end if;
            Append (Text, (if I = Points'First then "" else ","));
            Append (Back, (if I = Points'First then "" else ","));
            if Visible then
               declare
                  E1 : constant Driver.Images.Pixel := (U => Match * Gaussian, V => Match * Gaussian);
                  E2 : constant Driver.Images.Pixel := (U => Match * Gaussian, V => Match * Gaussian);
               begin
                  Append (Text, "[" & Number (Px.U + E1.U) & "," & Number (Px.V + E1.V) & ",1]");
                  Append (Back, "[" & Number (Points (I).U + E1.U + E2.U) & ","
                          & Number (Points (I).V + E1.V + E2.V) & "]");
               end;
            else
               Append (Text, "[-1,-1,-1]");
               Append (Back, "[-1,-1]");
            end if;
         end;
      end loop;
      return To_String (Text) & To_String (Back) & "]}";
   end Reply_Text;

   package Ticket_Vectors is new Ada.Containers.Vectors (Positive, Driver.Services.Ticket);
   Answered : Ticket_Vectors.Vector;

   procedure Answer (T : Driver.Services.Ticket; From, Into : Eye_Id; Seen : Observation;
                     Points : Driver.Instrument.Point_Array) is
   begin
      if not Answered.Contains (T) then
         Driver.Services.Replay_Reply
           (Driver.Services.Instrument, "/match",
            Driver.Instrument.Match_Request ((Stored => False, Image => Seen.Images (From)),
                                             (Stored => False, Image => Seen.Images (Into)), Points, True),
            (Ok   => True, Text => To_Unbounded_String (Reply_Text (Eyes (From), Eyes (Into), Points)),
             Why  => Null_Unbounded_String, Lasting => False));
         Answered.Append (T);
      end if;
   end Answer;

   Wrong_Segments : Boolean := False;
   --  When set, the instrument segments a thing in a second eye as the whole
   --  image: a segmentation that went wrong.

   procedure Answer_All (S : State) is
      --  Every match the estimator has asked for, the scene's and the things',
      --  and, when wrong ones are wanted, every segmentation.
   begin
      for X of S.Asking loop
         Answer (X.Ticket, X.From, X.Into, S.Round_Seen.Element, X.Points.Element);
      end loop;
      for R of S.Things loop
         for X of R.Crosses loop
            Answer (X.Ticket, X.From, X.Into, X.Seen.Element, X.Points.Element);
         end loop;
         if Wrong_Segments then
            for St of R.Starts loop
               if not Answered.Contains (St.Ticket) then
                  Driver.Services.Replay_Reply
                    (Driver.Services.Instrument, "/segment", St.Asked.Element,
                     (Ok      => True,
                      Text    => To_Unbounded_String
                                   ("{""ok"":true,""w"":" & Driver.Json.Number_Image (Real (Columns))
                                    & ",""h"":" & Driver.Json.Number_Image (Real (Rows)) & ",""score"":1,""runs"":[0,"
                                    & Driver.Json.Number_Image (Real (Columns * Rows)) & "]}"),
                      Why     => Null_Unbounded_String,
                      Lasting => False));
                  Answered.Append (St.Ticket);
               end if;
            end loop;
         end if;
      end loop;
   end Answer_All;

   function Plain (Level : Driver.Bytes.Byte) return Driver.Images.Image is
     (Driver.Images.Create (Columns, Rows, [1 .. 3 * Columns * Rows => Level]));

   function Box_Region return Driver.Images.Mask is
      --  The pixels of the first eye that show the box.
      M : Driver.Images.Mask := Driver.Images.Create (Columns, Rows);
   begin
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            declare
               Ray : constant Ray_Estimate := Eyes (1).Ray ((U => Real (C) + 0.5, V => Real (R) + 0.5));
               Hit : Surface_Hit;
               T   : Real;
            begin
               First_Hit (Ray.Origin.Mean, Ray.Direction.Unit_Vector, Hit, T);
               if Hit = Box then
                  Driver.Images.Include (M, C, R);
               end if;
            end;
         end loop;
      end loop;
      return M;
   end Box_Region;

   function Over_Box (Base : Driver.Images.Image; Level : Driver.Bytes.Byte) return Driver.Images.Image is
      --  The first eye's frame with the box's pixels at another level.
      Region : constant Driver.Images.Mask := Box_Region;
      Data   : Driver.Bytes.Byte_Array (1 .. 3 * Columns * Rows) := [others => Driver.Bytes.Byte (128)];
      pragma Unreferenced (Base);
   begin
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            if Driver.Images.Contains (Region, C, R) then
               for K in 1 .. 3 loop
                  Data (Driver.Bytes.Offset (3 * (R * Columns + C) + K)) := Level;
               end loop;
            end if;
         end loop;
      end loop;
      return Driver.Images.Create (Columns, Rows, Data);
   end Over_Box;

   function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
     return Driver.World.Cameras.Camera'Class
   is
      pragma Unreferenced (Seen);
   begin
      return Eyes (E);
   end Camera_Of;
   --  The pinholes do not move.

   procedure Scene_Flow is
      S     : State;
      Beat  : Driver.Clock.Beat := 1;
      Gray  : constant Driver.Images.Image := Plain (128);
      Box_T : Thing_Id;
      Table_F, Top_F : Natural := 0;

      function Seen_Now (First : Driver.Images.Image) return Observation is
         O : Observation;
      begin
         O.Beat := Beat;
         O.Images.Append (First);
         O.Images.Append (Gray);
         return O;
      end Seen_Now;

      procedure Step (First : Driver.Images.Image := Gray) is
      begin
         Driver.Services.Replay_Beat (Beat);
         Observe (S, 2, Camera_Of'Access, Up, True, Seen_Now (First));
         Answer_All (S);
         Beat := Beat + 1;
      end Step;

      procedure Find_Surfaces is
         --  The table, its plane through z = 0, and the box top, at its height.
      begin
         Table_F := 0;
         Top_F := 0;
         for F in S.Surfaces.First_Index .. S.Surfaces.Last_Index loop
            declare
               P : constant Driver.Geometry.Plane_Estimate := S.Surfaces (F).Plane;
            begin
               if abs P.Centre (3) < 0.005 then
                  Table_F := F;
               elsif abs (P.Centre (3) - Box_Top) < 0.005 then
                  Top_F := F;
               end if;
            end;
         end loop;
      end Find_Surfaces;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      Answered.Clear;
      Driver.Services.Start_Replay ([Driver.Services.Instrument => True, others => False]);
      Step;   --  the scene asked
      Step;   --  and measured
      Find_Surfaces;
      Check (Surface_Count (S) = 2 and then Table_F > 0 and then Top_F > 0,
             "the scene held" & Surface_Count (S)'Image & " surfaces, not the table and the box top");

      --  The box, adopted in the first eye, is matched into the second at the
      --  next still beat and rests on the table, not on its own top.
      Adopt (S, 1, Seen_Now (Gray), Box_Region, Box_T);
      Step;
      Step;
      Check (S.Things (Box_T).Has_Points, "the adopted box has no points two eyes saw");
      declare
         Under : constant Driver.World.Supports.Support := Support_Of (S, Box_T);
      begin
         Check (Under.Index = Table_F and then not Significant
                  (Scalar_Gate (Under.Height.Degrees_Of_Freedom, Tests => Natural (S.Things (Box_T).Points.Length)),
                   Under.Height.Value, Under.Height.Sigma),
                "the box rests on surface" & Under.Index'Image & " at" & Under.Height.Value'Image
                & " m, not on the table");
      end;

      --  A new episode keeps the surfaces as earlier ones, until they are
      --  measured again.
      New_Episode (S);
      Check (Surface_Count (S) = 2 and then Earlier (S, 1), "a new episode lost the surfaces, or kept them as its own");
      Step;
      Step;
      Find_Surfaces;
      Check (Surface_Count (S) = 2 and then not Earlier (S, 1) and then Table_F > 0 and then Top_F > 0,
             "the surfaces were not measured again in the new episode:" & Surface_Count (S)'Image & " surfaces, earlier "
             & Boolean'Image (Earlier (S, 1)) & ", table" & Table_F'Image & ", top" & Top_F'Image);

      --  The box, segmented wrongly in the second eye as the whole image, still
      --  rests on the table; then it changes where it is: its own top goes
      --  with it, the table stays.
      Wrong_Segments := True;
      Adopt (S, 1, Seen_Now (Gray), Box_Region, Box_T);
      Step;
      Step;
      Step;
      Check (Seen_In (S, Box_T, 2), "the box was not segmented in the second eye");
      Check (Support_Of (S, Box_T).Index = Table_F,
             "a wrong region in one eye made the table the box's own face");
      Step (Over_Box (Gray, Driver.Bytes.Byte'Last));
      Step (Over_Box (Gray, Driver.Bytes.Byte'Last));
      Find_Surfaces;
      Check (Surface_Count (S) = 1 and then Table_F > 0,
             "after the box changed the scene held" & Surface_Count (S)'Image & " surfaces, not the table alone");
      Wrong_Segments := False;
      Driver.Services.End_Replay;
   end Scene_Flow;

   procedure Points_Stay is
      --  The box top's points, as two eyes saw them, with the box held in the
      --  first eye. They stay while it holds the box where they fall, while it
      --  measures the box again in the same place (as when the eye moved and
      --  the box was segmented anew), and while it loses the box (as when
      --  something passes in front); they go once it holds the box elsewhere.
      S     : State;
      Gray  : constant Driver.Images.Image := Plain (128);
      Top   : Driver.World.Pairs.Match_Vectors.Vector;
      Beat  : Driver.Clock.Beat := 1;
      Box   : constant Driver.Images.Mask := Box_Region;

      function Shifted return Driver.Images.Mask is
         --  The box's pixels in the first eye, a box's width further right.
         M : Driver.Images.Mask := Driver.Images.Create (Columns, Rows);
         B : constant Driver.World.Regions.Box := Driver.World.Regions.Bounds (Box);
         W : constant Natural := B.Column_1 - B.Column_0 + 1;
      begin
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 - W loop
               if Driver.Images.Contains (Box, C, R) then
                  Driver.Images.Include (M, C + W, R);
               end if;
            end loop;
         end loop;
         return M;
      end Shifted;

      procedure Hold (Region : Driver.Images.Mask) is
         R : Thing_Record := S.Things (1);
      begin
         R.Eyes.Replace_Element (1, Slot'(Has => True, Track => Driver.World.Tracking.Start (Region, Gray, Beat), others => <>));
         S.Things.Replace_Element (1, R);
      end Hold;

      procedure Step (First : Driver.Images.Image) is
         O : Observation;
      begin
         O.Beat := Beat;
         O.Images.Append (First);
         O.Images.Append (Gray);
         Observe (S, 2, Camera_Of'Access, Up, True, O);
         Beat := Beat + 1;
      end Step;

      function Kept return Natural is (Natural (S.Things (1).Points.Length));
      --  The instrument answers nothing here: the thing's points are given.
   begin
      for I in 0 .. 4 loop
         for J in 0 .. 4 loop
            declare
               X      : constant Vec3 := [Box_Half * Real (I - 2) / 2.5, Box_Half * Real (J - 2) / 2.5, Box_Top];
               P1, P2 : Driver.Images.Pixel;
               V1, V2 : Boolean;
            begin
               Eyes (1).Project (X, P1, V1);
               Eyes (2).Project (X, P2, V2);
               if V1 and then V2 then
                  Top.Append (Driver.World.Pairs.Match'(In_First => P1, In_Second => P2, First => 1,
                               Point    => (Mean => X, Covariance => [[1.0E-6, 0.0, 0.0], [0.0, 1.0E-6, 0.0],
                                                                      [0.0, 0.0, 1.0E-6]])));
               end if;
            end;
         end loop;
      end loop;
      declare
         R : Thing_Record;
      begin
         R.Eyes.Append (Slot'(Has => True, Track => Driver.World.Tracking.Start (Box, Gray, Beat), others => <>));
         R.By_Pair.Append (Pair_Seen'(From => 1, Into => 2, Kept => Top));
         S.Things.Append (R);
      end;
      Driver.Services.Start_Replay ([Driver.Services.Instrument => True, others => False]);
      Step (Gray);
      Check (Kept = Natural (Top.Length), "of" & Top.Length'Image & " points inside the box only" & Kept'Image
             & " were kept");
      Hold (Box);
      Step (Gray);
      Check (Kept = Natural (Top.Length), "the box measured again in the same place kept" & Kept'Image & " of"
             & Top.Length'Image & " points");
      Step (Gray);
      Step (Over_Box (Gray, Driver.Bytes.Byte'Last));
      Step (Over_Box (Gray, Driver.Bytes.Byte'Last));
      Check (Driver.World.Tracking."/="
               (Driver.World.Tracking.State (S.Things (1).Eyes (1).Track), Driver.World.Tracking.Holding),
             "the first eye still holds the box after its pixels changed");
      Check (Kept = Natural (Top.Length), "the box lost from sight kept" & Kept'Image & " of" & Top.Length'Image
             & " points");
      Hold (Shifted);
      Step (Gray);
      Check (Kept = 0, Kept'Image & " points stayed where the box no longer is");
      Driver.Services.End_Replay;
   end Points_Stay;

   procedure Register is
   begin
      Driver.Tests.Register ("world.scene.points_stay",
                             "a thing's points go when an eye measures it again or loses it, or stay when an eye holds"
                             & " it elsewhere", Points_Stay'Access);
      Driver.Tests.Register ("world.scene.flow",
                             "the scene is not measured, an adopted thing does not rest on the table it stands on, an"
                             & " episode loses or keeps its surfaces wrongly, or a moved thing leaves its top behind",
                             Scene_Flow'Access);
   end Register;

end Driver.World.Estimates.Tests;
