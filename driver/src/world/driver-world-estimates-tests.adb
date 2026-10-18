with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Json;
with Driver.Log;
with Driver.Tests;
with Driver.World.Tests;

package body Driver.World.Estimates.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Bytes.Offset;
   use type Driver.Observations.Camera_Id;

   --  The scene: a table 1 m square at z = 0 with a box 0.2 m square and 0.08 m high, both textured by smooth value
   --  noise at several scales (a texture the aligner can tell, as a cloth or a print is), each face its own.
   Columns    : constant := 320;
   Rows       : constant := 240;
   Focal      : constant := 300.0;
   Noise_Px   : constant := 0.3;    --  the aligner's error the eyes state, pixels
   Box_Half   : constant := 0.1;
   Box_Top    : constant := 0.08;
   Table_Half : constant := 0.5;
   Box_Shift  : constant Vec3 := [0.0, 0.0, 0.0];   --  where the box stands off the table's middle
   Plain_Luma : constant := 20.0;   --  a plain dark box's every face (Plain), as the fake's small black box
   Plain      : Boolean := False;

   function Hash (X, Y, Seed : Integer) return Real is
      H : Long_Long_Integer := Long_Long_Integer (X) * 73_856_093 + Long_Long_Integer (Y) * 19_349_663
                               + Long_Long_Integer (Seed) * 83_492_791;
   begin
      H := (H mod 1_000_003 + 1_000_003) mod 1_000_003;
      H := (H * 48_271) mod 2_147_483_647;
      return Real (H mod 10_007) / 10_006.0;
   end Hash;

   function Value_Noise (X, Y : Real; Seed : Integer) return Real is
      X0 : constant Integer := Integer (Real'Floor (X));
      Y0 : constant Integer := Integer (Real'Floor (Y));
      Fx : constant Real := X - Real (X0);
      Fy : constant Real := Y - Real (Y0);
      Sx : constant Real := Fx * Fx * (3.0 - 2.0 * Fx);
      Sy : constant Real := Fy * Fy * (3.0 - 2.0 * Fy);
      A  : constant Real := Hash (X0, Y0, Seed);
      B  : constant Real := Hash (X0 + 1, Y0, Seed);
      C  : constant Real := Hash (X0, Y0 + 1, Seed);
      D  : constant Real := Hash (X0 + 1, Y0 + 1, Seed);
   begin
      return A + (B - A) * Sx + (C - A) * Sy + (A - B - C + D) * Sx * Sy;
   end Value_Noise;

   function Texture (X, Y : Real; Seed : Integer) return Real is
     (40.0 + 60.0 * Value_Noise (X * 60.0, Y * 60.0, Seed) + 50.0 * Value_Noise (X * 160.0, Y * 160.0, Seed + 1)
      + 40.0 * Value_Noise (X * 360.0, Y * 360.0, Seed + 2));

   type Surface_Hit is (Nothing, Table, Box);

   --  Where a line of sight from Origin along D first meets the scene, what it meets, and the luma there.
   procedure First_Hit (Origin, D : Vec3; Hit : out Surface_Hit; T : out Real; Luma : out Real) is
      Low  : constant Vec3 := [-Box_Half, -Box_Half, 0.0] + Box_Shift;
      High : constant Vec3 := [Box_Half, Box_Half, Box_Top] + Box_Shift;
      Near : Real := Real'First;
      Far  : Real := Real'Last;
      Face : Natural := 0;
      Into : Boolean := True;
   begin
      Hit := Nothing;
      T := Real'Last;
      Luma := 0.0;
      for A in 1 .. 3 loop
         if D (A) = 0.0 then
            Into := Into and then Origin (A) >= Low (A) and then Origin (A) <= High (A);
         else
            declare
               T1 : constant Real := (Low (A) - Origin (A)) / D (A);
               T2 : constant Real := (High (A) - Origin (A)) / D (A);
            begin
               if Real'Min (T1, T2) > Near then
                  Near := Real'Min (T1, T2);
                  Face := A;
               end if;
               Far := Real'Min (Far, Real'Max (T1, T2));
            end;
         end if;
      end loop;
      if Into and then Near <= Far and then Near > 0.0 then
         Hit := Box;
         T := Near;
         declare
            P : constant Vec3 := Origin + Near * D - Box_Shift;
         begin
            Luma := (if Plain then Plain_Luma
                     else (case Face is
                              when 3      => Texture (P (1), P (2), 7),
                              when 1      => Texture (P (2), P (3), 11),
                              when others => Texture (P (1), P (3), 13)));
         end;
      elsif D (3) < 0.0 then
         declare
            S : constant Real := -Origin (3) / D (3);
            P : constant Vec3 := Origin + S * D;
         begin
            if abs P (1) <= Table_Half and then abs P (2) <= Table_Half then
               Hit := Table;
               T := S;
               Luma := Texture (P (1), P (2), 3);
            end if;
         end;
      end if;
   end First_Hit;

   --  The eye that rides on the body stands where its readings say (group 1: x, y, z), looking at the table's middle;
   --  the eye that stands in the world is at Standing.
   Target   : constant Vec3 := [0.0, 0.0, 0.0];
   Standing : constant Vec3 := [0.3, -0.5, 0.6];

   function Moving_Eye (At_Place : Vec3) return Driver.World.Tests.Pinhole is
     (Driver.World.Tests.Looking_At (At_Place, Target, Focal, Columns, Rows, Noise_Px));

   function Standing_Eye return Driver.World.Tests.Pinhole is
     (Driver.World.Tests.Looking_At (Standing, Target, Focal, Columns, Rows, Noise_Px));

   function Place_Of (O : Observation) return Vec3 is
     ([O.Readings.Element (1) (1), O.Readings.Element (1) (2), O.Readings.Element (1) (3)]);

   function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
     return Driver.World.Cameras.Camera'Class is
     (if E = 1 then Moving_Eye (Place_Of (Seen.all)) else Standing_Eye);

   function Render (C : Driver.World.Tests.Pinhole) return Driver.Images.Image is
      Bytes : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Columns * Rows));
   begin
      for Row in 0 .. Rows - 1 loop
         for Column in 0 .. Columns - 1 loop
            declare
               Ray  : constant Ray_Estimate := C.Ray ((U => Real (Column) + 0.5, V => Real (Row) + 0.5));
               Hit  : Surface_Hit;
               T    : Real;
               Luma : Real;
               B    : Driver.Bytes.Byte;
               K    : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * Columns + Column));
            begin
               First_Hit (Ray.Origin.Mean, Ray.Direction.Unit_Vector, Hit, T, Luma);
               B := Driver.Bytes.Byte (Natural'Min (255, Natural'Max (0, Natural (Real'Floor (Luma)))));
               Bytes (K + 1) := B;
               Bytes (K + 2) := B;
               Bytes (K + 3) := B;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Columns, Rows, Bytes);
   end Render;

   --  The pixels of an eye that show the box.
   function Box_Region (C : Driver.World.Tests.Pinhole) return Driver.Images.Mask is
      M : Driver.Images.Mask := Driver.Images.Create (Columns, Rows);
   begin
      for Row in 0 .. Rows - 1 loop
         for Column in 0 .. Columns - 1 loop
            declare
               Ray  : constant Ray_Estimate := C.Ray ((U => Real (Column) + 0.5, V => Real (Row) + 0.5));
               Hit  : Surface_Hit;
               T    : Real;
               Luma : Real;
            begin
               First_Hit (Ray.Origin.Mean, Ray.Direction.Unit_Vector, Hit, T, Luma);
               if Hit = Box then
                  Driver.Images.Include (M, Column, Row);
               end if;
            end;
         end loop;
      end loop;
      return M;
   end Box_Region;

   --  A beat: the moving eye at Place, both eyes' pictures.
   function Seen_At (Place : Vec3; Beat : Driver.Clock.Beat) return Observation is
      O : Observation;
   begin
      O.Beat := Beat;
      O.Images.Append (Render (Moving_Eye (Place)));
      O.Images.Append (Render (Standing_Eye));
      O.Readings.Append (Real_Array'[Place (1), Place (2), Place (3)]);
      return O;
   end Seen_At;

   --  The places the body held its eye at: a few centimetres apart to either side and above, as an arm's sweep holds
   --  its eye.
   Places : constant array (1 .. 5) of Vec3 :=
     [[0.0, -0.45, 0.45], [0.07, -0.44, 0.45], [-0.07, -0.44, 0.45], [0.0, -0.40, 0.50], [0.03, -0.45, 0.43]];

   Up : constant Direction_Estimate := (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 0.001);

   Table_Plane : constant Driver.Geometry.Plane_Estimate :=
     (Centre => [0.0, 0.0, 0.0], Normal => [0.0, 0.0, 1.0], Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
      Offset_Sigma => 1.0e-4, Tilt_11 => 1.0e-8, Tilt_12 => 0.0, Tilt_22 => 1.0e-8, Points => 100, Scatter => 1.0);

   --  Every segmentation the estimator asked, answered by the scene: the box's own pixels in that eye.
   procedure Answer_Segments (S : State) is
      procedure Answer (M : Driver.Images.Mask; Asked : String) is
         Text   : Unbounded_String;
         Length : Natural := 0;
         On     : Boolean := False;
      begin
         for Row in 0 .. Rows - 1 loop
            for Column in 0 .. Columns - 1 loop
               if Driver.Images.Contains (M, Column, Row) /= On then
                  Append (Text, (if Text = Null_Unbounded_String then "" else ",")
                          & Driver.Json.Number_Image (Real (Length)));
                  Length := 0;
                  On := not On;
               end if;
               Length := Length + 1;
            end loop;
         end loop;
         Append (Text, (if Text = Null_Unbounded_String then "" else ",") & Driver.Json.Number_Image (Real (Length)));
         Driver.Services.Replay_Reply
           (Driver.Services.Instrument, "/segment", Asked,
            (Ok      => True,
             Text    => To_Unbounded_String
                          ("{""ok"":true,""w"":" & Driver.Json.Number_Image (Real (Columns))
                           & ",""h"":" & Driver.Json.Number_Image (Real (Rows)) & ",""score"":1,""runs"":["
                           & To_String (Text) & "]}"),
             Why     => Null_Unbounded_String,
             Lasting => False));
      end Answer;
   begin
      for R of S.Things loop
         for St of R.Starts loop
            Answer (Box_Region (if St.Eye = 1 then Moving_Eye (Places (1)) else Standing_Eye), St.Asked.Element);
         end loop;
      end loop;
   end Answer_Segments;

   --  The views of the places held, a beat each, still; then the scene due again and a still beat at the first place,
   --  and what is due measured.
   procedure Gather_Views (S : in out State; Beat : in out Driver.Clock.Beat) is
   begin
      Note_Table (S, Table_Plane);
      for P of Places loop
         Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (P, Beat));
         Beat := Beat + 1;
      end loop;
   end Gather_Views;

   procedure Scene_Placed is
      S    : State;
      Beat : Driver.Clock.Beat := 1;
      Table_Found, Top_Found : Boolean := False;
   begin
      Gather_Views (S, Beat);
      New_Episode (S);
      Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
      Check (Measure_Due (S), "the scene is not due at a still beat with views from five places");
      Measure (S, 2, Camera_Of'Access);
      for F in 1 .. Surface_Count (S) loop
         declare
            P : constant Driver.Geometry.Plane_Estimate := Plane_Of (S, Surface_Id (F));
         begin
            if abs P.Normal (3) > 0.9 then
               Table_Found := Table_Found or else abs P.Centre (3) < 0.01;
               Top_Found := Top_Found or else abs (P.Centre (3) - Box_Top) < 0.01;
            end if;
         end;
      end loop;
      Check (Scene_Size (S) > 0, "the scene placed no point");
      Check (Table_Found, "the table is not among the surfaces things rest on");
      Check (Top_Found, "the box's top is not among the surfaces things rest on");
   end Scene_Placed;

   procedure Thing_Placed is
      S    : State;
      Beat : Driver.Clock.Beat := 1;
      T    : Thing_Id;
   begin
      Gather_Views (S, Beat);
      declare
         O : constant Observation := Seen_At (Places (1), Beat);
      begin
         Adopt (S, 1, O, Box_Region (Moving_Eye (Places (1))), T);
         Observe (S, 2, Camera_Of'Access, Up, True, O);
      end;
      Check (Measure_Due (S), "a region adopted and held at a still beat is not due to be placed");
      Measure (S, 2, Camera_Of'Access);
      declare
         Points : constant Driver.World.Points.Placed_Vectors.Vector := Points_Of (S, T);
         Off    : Natural := 0;
      begin
         for M of Points loop
            if abs M.Point.Mean (1) > Box_Half + 0.01 or else abs M.Point.Mean (2) > Box_Half + 0.01
              or else M.Point.Mean (3) > Box_Top + 0.01 or else M.Point.Mean (3) < 0.0
            then
               Off := Off + 1;
            end if;
         end loop;
         Driver.Log.Line (Driver.Log.World, "world test: the box's" & Points.Length'Image & " points placed,"
                          & Off'Image & " of them off the box");
         Check (Natural (Points.Length) > 0, "the box adopted got no points");
         Check (Off = 0, Off'Image & " of the box's points lie off it");
      end;
      Beat := Beat + 1;
      Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
      Measure (S, 2, Camera_Of'Access);
      declare
         C : constant Point_Estimate := Centre (S, T);
      begin
         Check (abs C.Mean (1) <= Box_Half and then abs C.Mean (2) <= Box_Half,
                "the box's centre is not over its footprint");
      end;
      Check (Resting_On (S, T) > 0, "the box rests on no surface");
   end Thing_Placed;

   procedure Started_Elsewhere is
      S    : State;
      Beat : Driver.Clock.Beat := 1;
      T    : Thing_Id;
   begin
      Driver.Services.Start_Replay ([Driver.Services.Instrument => True, others => False]);
      Gather_Views (S, Beat);
      declare
         O : constant Observation := Seen_At (Places (1), Beat);
      begin
         Adopt (S, 1, O, Box_Region (Moving_Eye (Places (1))), T);
         Observe (S, 2, Camera_Of'Access, Up, True, O);
      end;
      Measure (S, 2, Camera_Of'Access);
      --  The points fall in the standing eye's picture: a segmentation asked there, answered, a track started.
      for Again in 1 .. 3 loop
         Beat := Beat + 1;
         Driver.Services.Replay_Beat (Beat);
         Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
         Answer_Segments (S);
      end loop;
      Check (Seen_In (S, T, 2), "the standing eye holds no track of the box its points fall in");
      Driver.Services.End_Replay;
   exception
      when others =>
         Driver.Services.End_Replay;
         raise;
   end Started_Elsewhere;

   --  A thing boxed in the eye that stands in the world, which places nothing by itself: the scene's points the moving
   --  eye placed that fall inside its region there are its points.
   procedure Seen_Standing is
      S    : State;
      Beat : Driver.Clock.Beat := 1;
      T    : Thing_Id;
   begin
      Gather_Views (S, Beat);
      New_Episode (S);
      Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
      Measure (S, 2, Camera_Of'Access);
      Beat := Beat + 1;
      declare
         O : constant Observation := Seen_At (Places (1), Beat);
      begin
         Adopt (S, 2, O, Box_Region (Standing_Eye), T);
         Observe (S, 2, Camera_Of'Access, Up, True, O);
      end;
      declare
         Points : constant Driver.World.Points.Placed_Vectors.Vector := Points_Of (S, T);
         Off    : Natural := 0;
      begin
         for M of Points loop
            if abs M.Point.Mean (1) > Box_Half + 0.01 or else abs M.Point.Mean (2) > Box_Half + 0.01
              or else M.Point.Mean (3) > Box_Top + 0.01 or else M.Point.Mean (3) < 0.0
            then
               Off := Off + 1;
            end if;
         end loop;
         Driver.Log.Line (Driver.Log.World, "world test: the box boxed in the standing eye has" & Points.Length'Image
                          & " points," & Off'Image & " of them off the box");
         Check (Natural (Points.Length) > 0, "a box boxed in the standing eye got none of the scene's points");
         Check (Off = 0, Off'Image & " of the points it got lie off the box");
      end;
   end Seen_Standing;

   --  A plain box (no texture the aligner can tell, as the fake's small black box) boxed in the eye that stands in the
   --  world before the scene is measured, so that none of the scene's points are its: where its region says it stands
   --  has the moving eye segment it, and the two outlines meet in its points.
   procedure Plain_Box is
      S    : State;
      Beat : Driver.Clock.Beat := 1;
      T    : Thing_Id;
   begin
      Plain := True;
      Driver.Services.Start_Replay ([Driver.Services.Instrument => True, others => False]);
      Gather_Views (S, Beat);
      New_Episode (S);
      Beat := Beat + 1;
      declare
         O : constant Observation := Seen_At (Places (1), Beat);
      begin
         Adopt (S, 2, O, Box_Region (Standing_Eye), T);
         Observe (S, 2, Camera_Of'Access, Up, True, O);
      end;
      for Again in 1 .. 3 loop
         Beat := Beat + 1;
         Driver.Services.Replay_Beat (Beat);
         Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
         Answer_Segments (S);
      end loop;
      Check (Seen_In (S, T, 1), "the moving eye holds no track of the plain box where its region says it stands");
      --  Both eyes hold it: its outlines meet apart, and the next beat gathers what they gave.
      Measure (S, 2, Camera_Of'Access);
      Beat := Beat + 1;
      Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
      declare
         Points : constant Driver.World.Points.Placed_Vectors.Vector := Points_Of (S, T);
         Off    : Natural := 0;
         Top    : Real := Real'First;
      begin
         for M of Points loop
            --  Off the box beyond Z of its own uncertainty along each axis.
            for A in 1 .. 3 loop
               declare
                  Low  : constant Real := (if A = 3 then 0.0 else -Box_Half);
                  High : constant Real := (if A = 3 then Box_Top else Box_Half);
                  Sd   : constant Real := Sqrt (Real'Max (0.0, M.Point.Covariance (A, A)));
               begin
                  if M.Point.Mean (A) < Low - Driver.Conventions.Z * Sd or else M.Point.Mean (A) > High + Driver.Conventions.Z * Sd then
                     Off := Off + 1;
                  end if;
               end;
            end loop;
            Top := Real'Max (Top, M.Point.Mean (3));
         end loop;
         Driver.Log.Line (Driver.Log.World, "world test: the plain box has" & Points.Length'Image & " points," & Off'Image
                          & " of them off it beyond their uncertainty; the highest at" & Driver.Log.Image (Top, 3));
         Check (Natural (Points.Length) > 0, "the plain box boxed in the standing eye got no points");
         Check (Off = 0, Off'Image & " of its points lie off it beyond their uncertainty");
      end;
      declare
         C : constant Point_Estimate := Centre (S, T);
      begin
         Driver.Log.Line (Driver.Log.World, "world test: the plain box's centre at" & Driver.Log.Image (C.Mean (1), 3)
                          & Driver.Log.Image (C.Mean (2), 3) & Driver.Log.Image (C.Mean (3), 3));
         Check (abs C.Mean (1) <= Box_Half and then abs C.Mean (2) <= Box_Half,
                "the plain box's centre is not over its footprint");
      end;
      Plain := False;
      Driver.Services.End_Replay;
   exception
      when others =>
         Plain := False;
         Driver.Services.End_Replay;
         raise;
   end Plain_Box;

   --  The world's measuring runs apart from the main loop (Driver.Apart), in a task whose stack is GNAT's default.
   procedure Measured_In_A_Task is
      S       : State;
      Beat    : Driver.Clock.Beat := 1;
      Done    : Boolean := False with Atomic;
      Failure : Unbounded_String;
   begin
      Gather_Views (S, Beat);
      New_Episode (S);
      Observe (S, 2, Camera_Of'Access, Up, True, Seen_At (Places (1), Beat));
      declare
         task Measurer;
         task body Measurer is
         begin
            Measure (S, 2, Camera_Of'Access);
            Done := True;
         exception
            when E : others =>
               Failure := To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
               Done := True;
         end Measurer;
      begin
         null;
      end;
      Check (Done and then Failure = Null_Unbounded_String, "measuring in a task failed: " & To_String (Failure));
   end Measured_In_A_Task;

   procedure Register is
   begin
      Driver.Tests.Register ("world.sight.scene",
                             "an eye held at five places does not place the scene, or its supports miss the table or the"
                             & " box's top", Scene_Placed'Access);
      Driver.Tests.Register ("world.sight.thing",
                             "a box adopted in the moving eye gets no points, points off it, a centre off its footprint,"
                             & " or nothing to rest on", Thing_Placed'Access);
      Driver.Tests.Register ("world.sight.started",
                             "an eye that stands in the world starts no track of a thing whose points fall in its picture",
                             Started_Elsewhere'Access);
      Driver.Tests.Register ("world.sight.standing",
                             "a box boxed in an eye that stands in the world gets none of the scene's points the moving"
                             & " eye placed inside its region there, or points off it", Seen_Standing'Access);
      Driver.Tests.Register ("world.sight.plain",
                             "a plain box boxed in the eye that stands in the world gets no points from its outlines, or"
                             & " points off it, or a centre off its footprint", Plain_Box'Access);
      Driver.Tests.Register ("world.sight.task",
                             "the world's measuring fails in a task with the default stack", Measured_In_A_Task'Access);
   end Register;

end Driver.World.Estimates.Tests;
