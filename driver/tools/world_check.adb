--  world_check RECORDING TRUTH --inst HOST:PORT
--
--  The world layer alone, on a recording with side-file truth
--  (harness/robodojo_truth). The recorded frames go through the world's
--  estimators (Driver.World.Offline) with the simulator's true cameras (the
--  optical frame and lens of every eye at every beat) and the true up in
--  place of the body's own, so every error left is the world layer's. The
--  body's stillness judgment runs on the recorded readings and commands as
--  the main loop runs it (Driver.Robot), and the instrument answers live.
--
--  Once the scene has been measured, every true object is adopted in the
--  first eye the middle of its mesh falls in, segmented by the instrument
--  there, as the brain points at a thing; from then on the world layer is on
--  its own. At the end each thing is scored against its object's true pose
--  and visual mesh: how far its points lie from the object's surface (and in
--  units of their own uncertainty), which object most of them lie on, and its
--  support and its height above it against the true table top, taken as the
--  median of the objects' lowest points. Each surface found is reported
--  against the same table top.

with Ada.Command_Line;
with Ada.Containers.Hashed_Maps;
with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Ada.Unchecked_Deallocation;
with Interfaces;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Geometry;
with Driver.Images;
with Driver.Instrument;
with Driver.Json;
with Driver.Log;
with Driver.Msgpack;
with Driver.Numerics;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Replies;
with Driver.Robot;
with Driver.Services;
with Driver.Stats;
with Driver.Uncertain;
with Driver.World;
with Driver.World.Cameras;
with Driver.World.Offline;
with Driver.World.Pairs;
with Driver.World.Supports;

procedure World_Check is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Log;
   use Driver.Uncertain;
   use type Driver.Real_Array;
   use type Ada.Containers.Count_Type;
   use type Driver.Json.Node;
   use type Driver.Json.Kind;
   use type Driver.Msgpack.Node;
   use type Driver.Protocol.Message_Kind;
   use type Driver.Recording.Record_Kind;
   use type Driver.World.Surface_Id;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;
   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Observation is Driver.Observations.Observation;

   Millimetres_Per_Metre : constant := 1000.0;
   Degrees_Per_Radian    : constant := 180.0 / Ada.Numerics.Pi;

   package Pose_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Rigid);
   package Value_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Real_Array);
   package Name_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);
   package Key_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String);
   package Vertex_Vectors is new Ada.Containers.Vectors (Positive, Vec3);
   package Index_Vectors is new Ada.Containers.Vectors (Positive, Positive);

   function Largest (X : Real_Array) return Real is
      M : Real := X (X'First);
   begin
      for V of X loop
         M := Real'Max (M, V);
      end loop;
      return M;
   end Largest;

   function Quantile (X : Real_Array; Share : Real) return Real is
      --  The value that share of them do not exceed.
      Sorted : Real_Array := X;
   begin
      for I in Sorted'First + 1 .. Sorted'Last loop
         declare
            V : constant Real := Sorted (I);
            J : Integer := I - 1;
         begin
            while J >= Sorted'First and then Sorted (J) > V loop
               Sorted (J + 1) := Sorted (J);
               J := J - 1;
            end loop;
            Sorted (J + 1) := V;
         end;
      end loop;
      return Sorted (Sorted'First + Natural (Real'Floor (Share * Real (Sorted'Length - 1))));
   end Quantile;

   function Mm (X : Real) return String is (Image (Millimetres_Per_Metre * X, 1));

   ---------------------------------------------------------------------------
   --  Truth

   function Quaternion_Pose (X : Real_Array) return Rigid is
     (Rotation    => To_Matrix ((W => X (X'First + 3), X => X (X'First + 4), Y => X (X'First + 5),
                                 Z => X (X'First + 6))),
      Translation => [X (X'First), X (X'First + 1), X (X'First + 2)]);

   function Numbers_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Real_Array is
      R : Real_Array (1 .. Driver.Json.Count (Doc, N));
   begin
      for I in R'Range loop
         R (I) := Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, I));
      end loop;
      return R;
   end Numbers_Of;

   type Truth_Line is record
      Cameras : Pose_Maps.Map;    --  camera name -> optical frame
      Objects : Pose_Maps.Map;    --  object name -> pose of its root
      State   : Value_Maps.Map;   --  state key -> the observation's own values
   end record;

   package Truth_Vectors is new Ada.Containers.Vectors (Positive, Truth_Line);
   Truth : Truth_Vectors.Vector;

   type Lens is record
      Has_K         : Boolean := False;
      K             : Mat3 := Identity3;
      Width, Height : Natural := 0;
      Coefficients  : Real_Holders.Holder;   --  F-theta: angle = sum of c (i) r ** (i - 1), r in pixels
      Max_Field     : Real := 0.0;           --  F-theta: the full field of view, degrees
   end record;

   package Lens_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Lens);
   Lenses      : Lens_Maps.Map;
   Store       : Unbounded_String;
   Object_Keys : Key_Maps.Map;   --  object name -> geometry file in the store

   procedure Read_Truth (Path : String) is
      use Driver.Json;
      F   : Ada.Text_IO.File_Type;
      Doc : Document;
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
         if Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "geometry" then
            declare
               Objects : constant Node := Lookup (Doc, Root (Doc), "objects");
            begin
               Store := To_Unbounded_String (Text (Doc, Lookup (Doc, Root (Doc), "store")));
               for I in 1 .. Count (Doc, Objects) loop
                  if Kind_Of (Doc, Member_Value (Doc, Objects, I)) = String_Value then
                     Object_Keys.Include (Member_Name (Doc, Objects, I), Text (Doc, Member_Value (Doc, Objects, I)));
                  end if;
               end loop;
            end;
         elsif Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "observation" then
            declare
               L       : Truth_Line;
               Cameras : constant Node := Lookup (Doc, Root (Doc), "cameras");
               Objects : constant Node := Lookup (Doc, Root (Doc), "objects");
               State   : constant Node := Lookup (Doc, Root (Doc), "state");
            begin
               for I in 1 .. Count (Doc, State) loop
                  L.State.Include (Member_Name (Doc, State, I), Numbers_Of (Doc, Member_Value (Doc, State, I)));
               end loop;
               for I in 1 .. Count (Doc, Objects) loop
                  L.Objects.Include (Member_Name (Doc, Objects, I),
                                     Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, Member_Value (Doc, Objects, I),
                                                                               "pose"))));
               end loop;
               for I in 1 .. Count (Doc, Cameras) loop
                  declare
                     Name : constant String := Member_Name (Doc, Cameras, I);
                     C    : constant Node := Member_Value (Doc, Cameras, I);
                  begin
                     L.Cameras.Include (Name, Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, C, "pose"))));
                     if not Lenses.Contains (Name) then
                        declare
                           E          : Lens;
                           K          : constant Node := Lookup (Doc, C, "K");
                           Resolution : constant Real_Array := Numbers_Of (Doc, Lookup (Doc, C, "resolution"));
                           Ftheta     : constant Node := Lookup (Doc, C, "lens");
                        begin
                           E.Width := Natural (Resolution (1));
                           E.Height := Natural (Resolution (2));
                           E.Has_K := Kind_Of (Doc, K) = Array_Value;
                           if E.Has_K then
                              for A in 1 .. 3 loop
                                 for B in 1 .. 3 loop
                                    E.K (A, B) := Number (Doc, Element (Doc, Element (Doc, K, A), B));
                                 end loop;
                              end loop;
                           elsif Kind_Of (Doc, Ftheta) = Object_Value then
                              E.Coefficients := Real_Holders.To_Holder
                                (Numbers_Of (Doc, Lookup (Doc, Ftheta, "coefficients")));
                              E.Max_Field := Number (Doc, Lookup (Doc, Ftheta, "max_fov"));
                           end if;
                           Lenses.Include (Name, E);
                        end;
                     end if;
                  end;
               end loop;
               Truth.Append (L);
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Truth;

   function Known_Lens (L : Lens) return Boolean is (L.Has_K or else not L.Coefficients.Is_Empty);

   function True_Ray (L : Lens; U, V : Real) return Vec3 is
      --  The line of sight through a pixel, in the optical frame.
   begin
      if L.Has_K then
         return Unit ([(U - L.K (1, 3)) / L.K (1, 1), (V - L.K (2, 3)) / L.K (2, 2), 1.0]);
      end if;
      declare
         --  F-theta about the image centre (the rig puts its optical centre there).
         C     : constant Real_Array := L.Coefficients.Element;
         Du    : constant Real := U - Real (L.Width) / 2.0;
         Dv    : constant Real := V - Real (L.Height) / 2.0;
         R     : constant Real := Sqrt (Du * Du + Dv * Dv);
         Theta : Real := 0.0;
      begin
         for I in reverse C'Range loop
            Theta := Theta * R + C (I);
         end loop;
         if R = 0.0 then
            return [0.0, 0.0, 1.0];
         end if;
         return [Sin (Theta) * Du / R, Sin (Theta) * Dv / R, Cos (Theta)];
      end;
   end True_Ray;

   procedure True_Pixel (L : Lens; In_Eye : Vec3; U, V : out Real; Visible : out Boolean) is
   begin
      Visible := In_Eye (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if L.Has_K then
         if Visible then
            U := L.K (1, 1) * In_Eye (1) / In_Eye (3) + L.K (1, 3);
            V := L.K (2, 2) * In_Eye (2) / In_Eye (3) + L.K (2, 3);
            Visible := U >= 0.0 and then V >= 0.0 and then U < Real (L.Width) and then V < Real (L.Height);
         end if;
         return;
      end if;
      declare
         --  F-theta: invert angle (r) by bisection; it rises with r over the image.
         C     : constant Real_Array := L.Coefficients.Element;
         Rho   : constant Real := Sqrt (In_Eye (1) ** 2 + In_Eye (2) ** 2);
         Theta : constant Real := Arctan (Rho, In_Eye (3));
         Lo    : Real := 0.0;
         Hi    : Real := Real (Natural'Max (L.Width, L.Height));
         function Angle_At (R : Real) return Real is
            A : Real := 0.0;
         begin
            for I in reverse C'Range loop
               A := A * R + C (I);
            end loop;
            return A;
         end Angle_At;
      begin
         Visible := Theta <= L.Max_Field / 2.0 / Degrees_Per_Radian and then Angle_At (Hi) >= Theta;
         if not Visible or else Rho = 0.0 then
            U := Real (L.Width) / 2.0;
            V := Real (L.Height) / 2.0;
            return;
         end if;
         for Step in 1 .. Real'Machine_Mantissa loop
            if Angle_At ((Lo + Hi) / 2.0) < Theta then
               Lo := (Lo + Hi) / 2.0;
            else
               Hi := (Lo + Hi) / 2.0;
            end if;
         end loop;
         U := Real (L.Width) / 2.0 + Lo * In_Eye (1) / Rho;
         V := Real (L.Height) / 2.0 + Lo * In_Eye (2) / Rho;
         Visible := U >= 0.0 and then V >= 0.0 and then U < Real (L.Width) and then V < Real (L.Height);
      end;
   end True_Pixel;

   ---------------------------------------------------------------------------
   --  The true eyes, as cameras the world's estimators take

   type Truth_Camera is new Driver.World.Cameras.Camera with record
      Optical : Rigid;
      Of_Lens : Lens;
      Known   : Boolean := False;
   end record;

   overriding function Width (C : Truth_Camera) return Natural is (C.Of_Lens.Width);
   overriding function Height (C : Truth_Camera) return Natural is (C.Of_Lens.Height);
   overriding function Ray (C : Truth_Camera; Px : Driver.Images.Pixel) return Ray_Estimate;
   overriding procedure Project (C : Truth_Camera; Point : Vec3; Px : out Driver.Images.Pixel; Visible : out Boolean);
   overriding function Pose (C : Truth_Camera) return Pose_Estimate;
   overriding function Self_Mask (C : Truth_Camera) return Driver.Images.Mask;
   --  The truth here holds no image of the robot: its pixels stay in the scene.

   function Ray (C : Truth_Camera; Px : Driver.Images.Pixel) return Ray_Estimate is
      Unmeasured : Ray_Estimate;
   begin
      if not C.Known then
         return Unmeasured;
      end if;
      declare
         Here   : constant Vec3 := True_Ray (C.Of_Lens, Px.U, Px.V);
         Beside : constant Vec3 := True_Ray (C.Of_Lens, Px.U + 1.0, Px.V);
         Step   : constant Real := Arctan (abs Cross (Here, Beside), Here * Beside);
      begin
         --  The truth is exact; what a pixel shows is anywhere in it, so its
         --  line of sight is uncertain by a uniform pixel's spread.
         return (Origin    => (Mean => C.Optical.Translation, Covariance => [others => [others => 0.0]]),
                 Direction => (Unit_Vector => C.Optical.Rotation * Here, Sigma => Step / Sqrt (12.0)));
      end;
   end Ray;

   procedure Project (C : Truth_Camera; Point : Vec3; Px : out Driver.Images.Pixel; Visible : out Boolean)
   is
      U, V : Real;
   begin
      Px := (U => 0.0, V => 0.0);
      Visible := False;
      if C.Known then
         True_Pixel (C.Of_Lens, Transpose (C.Optical.Rotation) * (Point - C.Optical.Translation), U, V, Visible);
         Px := (U => U, V => V);
      end if;
   end Project;

   function Pose (C : Truth_Camera) return Pose_Estimate is
     ((Pose => C.Optical, Position_Covariance => [others => [others => 0.0]],
       Rotation_Covariance => [others => [others => 0.0]]));

   function Self_Mask (C : Truth_Camera) return Driver.Images.Mask is
     (Driver.Images.Create (C.Of_Lens.Width, C.Of_Lens.Height));

   Camera_Paths : Name_Vectors.Vector;   --  the layout's camera paths, in eye order

   function Camera_Name (Eye : Eye_Id) return String is
      --  The truth camera whose name is a component of the eye's layout path.
      Path : constant String := "/" & Camera_Paths (Positive (Eye)) & "/";
   begin
      for C in Lenses.Iterate loop
         if Ada.Strings.Fixed.Index (Path, "/" & Lens_Maps.Key (C) & "/") > 0 then
            return Lens_Maps.Key (C);
         end if;
      end loop;
      return "";
   end Camera_Name;

   package Line_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   Line_Of_Beat : Line_Vectors.Vector;   --  the truth line paired with each beat; 0 for none
   Robot        : aliased Driver.Robot.Model;   --  the body's own estimators, for its stillness and image lag

   function True_Camera (E : Eye_Id; Beat : Natural) return Truth_Camera is
      --  The eye as it was when the image of that beat was taken: the image
      --  trails the readings by the lag the body measured (Driver.Robot), and
      --  the truth's poses go with the readings.
      Name   : constant String := Camera_Name (E);
      Shot   : constant Integer := Beat - Driver.Robot.Image_Lag (Robot, E);
      Result : Truth_Camera;
   begin
      if Name /= "" and then Shot >= 0 and then Shot <= Line_Of_Beat.Last_Index and then Line_Of_Beat (Shot) > 0
        and then Truth (Line_Of_Beat (Shot)).Cameras.Contains (Name) and then Known_Lens (Lenses (Name))
      then
         Result.Optical := Truth (Line_Of_Beat (Shot)).Cameras (Name);
         Result.Of_Lens := Lenses (Name);
         Result.Known := True;
      end if;
      return Result;
   end True_Camera;

   function Camera_Of (E : Eye_Id; Seen : not null access constant Observation)
     return Driver.World.Cameras.Camera'Class is (True_Camera (E, Natural (Seen.Beat)));

   ---------------------------------------------------------------------------
   --  Meshes, and how far a point is from one's surface

   type Triangle is record
      A, B, C : Vec3;
   end record;

   package Triangle_Vectors is new Ada.Containers.Vectors (Positive, Triangle);

   type Mesh is record
      Vertices  : Vertex_Vectors.Vector;     --  in the object's frame
      Triangles : Triangle_Vectors.Vector;
   end record;

   package Mesh_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Mesh);
   Meshes : Mesh_Maps.Map;   --  by geometry key, each read once

   type Bytes_Access is access Driver.Bytes.Byte_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Bytes.Byte_Array, Bytes_Access);

   function Words (Path : String) return Bytes_Access is
      --  A whole raw file, on the heap: a mesh's run to tens of megabytes.
      use Ada.Streams.Stream_IO;
      F      : File_Type;
      Result : Bytes_Access;
      Last   : Ada.Streams.Stream_Element_Offset;
   begin
      Open (F, In_File, Path);
      Result := new Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (Size (F)));
      Read (F, Result.all, Last);
      Close (F);
      return Result;
   end Words;

   function Word_At (B : Driver.Bytes.Byte_Array; At_Byte : Ada.Streams.Stream_Element_Offset)
     return Interfaces.Unsigned_32 is
      use type Interfaces.Unsigned_32;
      use type Ada.Streams.Stream_Element_Offset;
   begin
      return Interfaces.Unsigned_32 (B (At_Byte))
        or Interfaces.Shift_Left (Interfaces.Unsigned_32 (B (At_Byte + 1)), 8)
        or Interfaces.Shift_Left (Interfaces.Unsigned_32 (B (At_Byte + 2)), 16)
        or Interfaces.Shift_Left (Interfaces.Unsigned_32 (B (At_Byte + 3)), 24);
   end Word_At;

   function To_Float is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, Interfaces.IEEE_Float_32);
   function To_Integer is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, Interfaces.Integer_32);

   function Mesh_Of (Key : String) return Mesh is
      --  The visual meshes of a stored geometry (the collision ones when it has
      --  no visual mesh), as triangles: a face of n corners is a fan of n - 2.
      use Driver.Json;
      use type Ada.Streams.Stream_Element_Offset;
      F       : Ada.Text_IO.File_Type;
      Doc     : Document;
      Ok      : Boolean;
      Why     : Unbounded_String;
      Result  : Mesh;
   begin
      if Meshes.Contains (Key) then
         return Meshes (Key);
      end if;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, To_String (Store) & "/" & Key & ".json");
      Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
      Ada.Text_IO.Close (F);
      if Ok then
         declare
            All_Meshes : constant Node := Lookup (Doc, Root (Doc), "meshes");
            Any_Visual : Boolean := False;
         begin
            for I in 1 .. Count (Doc, All_Meshes) loop
               Any_Visual := Any_Visual or else Is_True (Doc, Lookup (Doc, Element (Doc, All_Meshes, I), "visual"));
            end loop;
            for I in 1 .. Count (Doc, All_Meshes) loop
               declare
                  M : constant Node := Element (Doc, All_Meshes, I);
               begin
                  if Is_True (Doc, Lookup (Doc, M, (if Any_Visual then "visual" else "collision"))) then
                     declare
                        Dir     : constant String := To_String (Store) & "/";
                        Points  : Bytes_Access := Words (Dir & Text (Doc, Lookup (Doc, M, "points")));
                        Counts  : Bytes_Access := Words (Dir & Text (Doc, Lookup (Doc, M, "counts")));
                        Indices : Bytes_Access := Words (Dir & Text (Doc, Lookup (Doc, M, "indices")));
                        First   : constant Natural := Natural (Result.Vertices.Length);
                        Corner  : Ada.Streams.Stream_Element_Offset := Indices'First;
                        function Vertex (N : Interfaces.Integer_32) return Vec3 is
                          (Result.Vertices (First + Positive (Integer (N) + 1)));
                     begin
                        for V in 0 .. Points'Length / 12 - 1 loop
                           declare
                              B : constant Ada.Streams.Stream_Element_Offset :=
                                Points'First + Ada.Streams.Stream_Element_Offset (12 * V);
                           begin
                              Result.Vertices.Append
                                (Vec3'(Real (To_Float (Word_At (Points.all, B))), Real (To_Float (Word_At (Points.all, B + 4))),
                                       Real (To_Float (Word_At (Points.all, B + 8)))));
                           end;
                        end loop;
                        for Face in 0 .. Counts'Length / 4 - 1 loop
                           declare
                              N : constant Natural := Natural (To_Integer
                                (Word_At (Counts.all, Counts'First + Ada.Streams.Stream_Element_Offset (4 * Face))));
                              function Index_At (K : Natural) return Interfaces.Integer_32 is
                                (To_Integer (Word_At (Indices.all, Corner + Ada.Streams.Stream_Element_Offset (4 * K))));
                           begin
                              for K in 1 .. N - 2 loop
                                 Result.Triangles.Append
                                   (Triangle'(A => Vertex (Index_At (0)), B => Vertex (Index_At (K)),
                                     C => Vertex (Index_At (K + 1))));
                              end loop;
                              Corner := Corner + Ada.Streams.Stream_Element_Offset (4 * N);
                           end;
                        end loop;
                        Free (Points);
                        Free (Counts);
                        Free (Indices);
                     end;
                  end if;
               end;
            end loop;
         end;
      end if;
      Meshes.Include (Key, Result);
      return Result;
   end Mesh_Of;

   function Closest_On (T : Triangle; P : Vec3) return Vec3 is
      --  The point of the triangle nearest P (Ericson, Real-Time Collision
      --  Detection, 5.1.5): by the region of the triangle's plane P projects into.
      AB : constant Vec3 := T.B - T.A;
      AC : constant Vec3 := T.C - T.A;
      AP : constant Vec3 := P - T.A;
      BP : constant Vec3 := P - T.B;
      CP : constant Vec3 := P - T.C;
      D1 : constant Real := AB * AP;
      D2 : constant Real := AC * AP;
      D3 : constant Real := AB * BP;
      D4 : constant Real := AC * BP;
      D5 : constant Real := AB * CP;
      D6 : constant Real := AC * CP;
      VA : constant Real := D3 * D6 - D5 * D4;
      VB : constant Real := D5 * D2 - D1 * D6;
      VC : constant Real := D1 * D4 - D3 * D2;
   begin
      if D1 <= 0.0 and then D2 <= 0.0 then
         return T.A;
      elsif D3 >= 0.0 and then D4 <= D3 then
         return T.B;
      elsif VC <= 0.0 and then D1 >= 0.0 and then D3 <= 0.0 then
         return T.A + (D1 / (D1 - D3)) * AB;
      elsif D6 >= 0.0 and then D5 <= D6 then
         return T.C;
      elsif VB <= 0.0 and then D2 >= 0.0 and then D6 <= 0.0 then
         return T.A + (D2 / (D2 - D6)) * AC;
      elsif VA <= 0.0 and then D4 - D3 >= 0.0 and then D5 - D6 >= 0.0 then
         return T.B + ((D4 - D3) / ((D4 - D3) + (D5 - D6))) * (T.C - T.B);
      elsif VA + VB + VC = 0.0 then
         return T.A;   --  a triangle with no area: its corner
      end if;
      return T.A + (VB / (VA + VB + VC)) * AB + (VC / (VA + VB + VC)) * AC;
   end Closest_On;

   --  A mesh placed in the world, its triangles binned in a uniform grid of
   --  cells so the triangles near a point are found without trying them all.

   type Cell is record
      I, J, K : Integer := 0;
   end record;

   function Hash (C : Cell) return Ada.Containers.Hash_Type is
     (Ada.Containers."xor" (Ada.Containers."xor" (Ada.Containers."*" (Ada.Containers.Hash_Type'Mod (C.I), 73_856_093),
                                                Ada.Containers."*" (Ada.Containers.Hash_Type'Mod (C.J), 19_349_663)),
                           Ada.Containers."*" (Ada.Containers.Hash_Type'Mod (C.K), 83_492_791)));

   package Cell_Maps is new Ada.Containers.Hashed_Maps (Cell, Index_Vectors.Vector, Hash, "=", Index_Vectors."=");

   type Placed is record
      Triangles : Triangle_Vectors.Vector;   --  in the world
      Lowest    : Vec3 := Zero3;              --  its lowest vertex
      Middle    : Vec3 := Zero3;              --  the mean of its vertices
      Low, High : Vec3 := Zero3;              --  its box
      Size      : Real := 0.0;                --  a cell's edge
      Cells     : Cell_Maps.Map;
   end record;

   package Placed_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Placed);

   function Cell_Of (S : Placed; P : Vec3) return Cell is
     ((I => Integer (Real'Floor ((P (1) - S.Low (1)) / S.Size)),
       J => Integer (Real'Floor ((P (2) - S.Low (2)) / S.Size)),
       K => Integer (Real'Floor ((P (3) - S.Low (3)) / S.Size))));

   function Place (M : Mesh; Pose : Rigid) return Placed is
      S      : Placed;
      Lowest : Real := Real'Last;
      Sum    : Vec3 := Zero3;
      Extent : Real := 0.0;
   begin
      if M.Triangles.Is_Empty then
         return S;
      end if;
      S.Low := [others => Real'Last];
      S.High := [others => Real'First];
      for V of M.Vertices loop
         declare
            P : constant Vec3 := Pose * V;
         begin
            Sum := Sum + P;
            if P (3) < Lowest then
               Lowest := P (3);
               S.Lowest := P;
            end if;
            for A in 1 .. 3 loop
               S.Low (A) := Real'Min (S.Low (A), P (A));
               S.High (A) := Real'Max (S.High (A), P (A));
            end loop;
         end;
      end loop;
      S.Middle := (1.0 / Real (M.Vertices.Length)) * Sum;
      for T of M.Triangles loop
         declare
            W : constant Triangle := (A => Pose * T.A, B => Pose * T.B, C => Pose * T.C);
         begin
            S.Triangles.Append (W);
            for A in 1 .. 3 loop
               Extent := Extent + (Real'Max (W.A (A), Real'Max (W.B (A), W.C (A)))
                                   - Real'Min (W.A (A), Real'Min (W.B (A), W.C (A))));
            end loop;
         end;
      end loop;
      --  Cells as large as a triangle is on the average: a triangle then lies
      --  in a few cells, and a cell holds a few triangles. (Only the search's
      --  speed depends on it; the distance it finds is exact.)
      S.Size := Real'Max (Extent / Real (3 * S.Triangles.Length), Real'Model_Small);
      for N in S.Triangles.First_Index .. S.Triangles.Last_Index loop
         declare
            T  : constant Triangle := S.Triangles (N);
            Lo : constant Cell := Cell_Of (S, [Real'Min (T.A (1), Real'Min (T.B (1), T.C (1))),
                                               Real'Min (T.A (2), Real'Min (T.B (2), T.C (2))),
                                               Real'Min (T.A (3), Real'Min (T.B (3), T.C (3)))]);
            Hi : constant Cell := Cell_Of (S, [Real'Max (T.A (1), Real'Max (T.B (1), T.C (1))),
                                               Real'Max (T.A (2), Real'Max (T.B (2), T.C (2))),
                                               Real'Max (T.A (3), Real'Max (T.B (3), T.C (3)))]);
         begin
            for I in Lo.I .. Hi.I loop
               for J in Lo.J .. Hi.J loop
                  for K in Lo.K .. Hi.K loop
                     declare
                        C        : constant Cell := (I, J, K);
                        Position : constant Cell_Maps.Cursor := S.Cells.Find (C);
                     begin
                        if Cell_Maps.Has_Element (Position) then
                           S.Cells.Reference (Position).Append (N);
                        else
                           S.Cells.Insert (C, Index_Vectors.To_Vector (N, 1));
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
         end;
      end loop;
      return S;
   end Place;

   function Middle_Of (M : Mesh; Pose : Rigid) return Vec3 is
      --  The mean of the mesh's vertices, placed.
      Sum : Vec3 := Zero3;
   begin
      for V of M.Vertices loop
         Sum := Sum + Pose * V;
      end loop;
      return (if M.Vertices.Is_Empty then Pose.Translation else (1.0 / Real (M.Vertices.Length)) * Sum);
   end Middle_Of;

   function Distance (S : Placed; P : Vec3) return Real is
      --  How far P is from the mesh's surface: the cells within a cube about P
      --  are searched, the cube doubling until the nearest triangle found lies
      --  within it (then no triangle outside can be nearer), or it holds them all.
      Reach : Real := S.Size;
      Best  : Real := Real'Last;
   begin
      if S.Triangles.Is_Empty then
         return Real'Last;
      end if;
      loop
         declare
            Lo : constant Cell := Cell_Of (S, P - [Reach, Reach, Reach]);
            Hi : constant Cell := Cell_Of (S, P + [Reach, Reach, Reach]);
            All_In : constant Boolean :=
              (for all A in 1 .. 3 => P (A) - Reach <= S.Low (A) and then P (A) + Reach >= S.High (A));
         begin
            Best := Real'Last;
            for I in Lo.I .. Hi.I loop
               for J in Lo.J .. Hi.J loop
                  for K in Lo.K .. Hi.K loop
                     declare
                        Position : constant Cell_Maps.Cursor := S.Cells.Find ((I, J, K));
                     begin
                        if Cell_Maps.Has_Element (Position) then
                           for N of S.Cells.Constant_Reference (Position) loop
                              Best := Real'Min (Best, abs (P - Closest_On (S.Triangles (N), P)));
                           end loop;
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
            exit when Best <= Reach or else All_In;
            Reach := 2.0 * Reach;
         end;
      end loop;
      return Best;
   end Distance;

   ---------------------------------------------------------------------------
   --  The replay

   Path        : Unbounded_String;
   Truth_Path  : Unbounded_String;

   R        : Driver.Recording.Reader;
   Opened   : Boolean;
   Kind     : Driver.Recording.Record_Kind;
   Ns       : Long_Long_Integer;
   Payload  : Driver.Bytes.Buffer;
   More     : Boolean := True;

   Layout   : Driver.Observations.Layout;
   Known    : Boolean := False;
   Beat     : Natural := 0;
   Paired   : Natural := 0;
   Next_Line : Positive := 1;   --  the first truth line not yet paired
   Episodes : Natural := 0;

   Bench : Driver.World.Offline.Bench;
   Sent  : Driver.Commands.Command := Driver.Commands.Hold;
   Last  : Natural := 0;        --  the last paired beat

   Up : constant Direction_Estimate := (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 0.0);
   --  The truth's world frame has z up.

   --  The things adopted, and the object each was pointed at.
   type Adopted is record
      Object : Unbounded_String;
      Eye    : Eye_Id := 1;
      Prompt : Driver.Images.Pixel;
   end record;

   package Adopted_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Driver.World.Thing_Id, Adopted,
                                                                       Driver.World."<");
   Things      : Adopted_Maps.Map;
   Adopted_Yet : Boolean := False;
   Last_Surfaces : Natural := 0;

   function Same_State (O : Observation; L : Truth_Line) return Boolean is
      Common : Natural := 0;
   begin
      for C in L.State.Iterate loop
         declare
            Key : constant String := "state/" & Value_Maps.Key (C);
         begin
            for G in Layout.Groups.First_Index .. Layout.Groups.Last_Index loop
               if To_String (Layout.Groups (G).Path) = Key and then Driver.Observations.Has_Reading (O, G) then
                  if O.Readings.Element (G) /= Value_Maps.Element (C) then
                     return False;
                  end if;
                  Common := Common + 1;
               end if;
            end loop;
         end;
      end loop;
      return Common > 0;
   end Same_State;

   procedure Adopt_Objects (O : Observation) is
      --  Every true object, in the first eye the middle of its mesh falls in,
      --  segmented there by the instrument as a brain's pointing would be.
      L : constant Truth_Line := Truth (Line_Of_Beat (Beat));
   begin
      for C in L.Objects.Iterate loop
         declare
            Name : constant String := Pose_Maps.Key (C);
         begin
            if Object_Keys.Contains (Name) then
               declare
                  Its   : constant Mesh := Mesh_Of (Object_Keys (Name));
                  Where : constant Vec3 := Middle_Of (Its, Pose_Maps.Element (C));
                  Done  : Boolean := False;
                  Seen  : Unbounded_String;
               begin
                  --  Every eye the middle of its mesh falls in, as the truth puts it.
                  for E in 1 .. Eye_Id'Base (Natural (O.Images.Length)) loop
                     declare
                        Px      : Driver.Images.Pixel;
                        Visible : Boolean;
                     begin
                        True_Camera (E, Beat).Project (Where, Px, Visible);
                        if Visible then
                           Append (Seen, E'Image);
                        end if;
                     end;
                  end loop;
                  Ada.Text_IO.Put_Line ("beat" & Beat'Image & ": " & Name & " falls in eyes" & To_String (Seen));
                  for E in 1 .. Eye_Id'Base (Natural (O.Images.Length)) loop
                     exit when Done;
                     if Driver.Observations.Has_Image (O, E) and then not Its.Triangles.Is_Empty then
                        declare
                           Cam     : constant Truth_Camera := True_Camera (E, Beat);
                           Px      : Driver.Images.Pixel;
                           Visible : Boolean;
                        begin
                           Cam.Project (Where, Px, Visible);
                           if Visible then
                              declare
                                 Reply  : constant Driver.Services.Reply :=
                                   Driver.Services.Call
                                     (Driver.Services.Instrument, "/segment",
                                      Driver.Instrument.Segment_Request
                                        (O.Images (E), False, (others => <>), [1 => (At_Pixel => Px, On => True)]));
                                 Region : Driver.Images.Mask;
                                 Score  : Real;
                                 Ok     : Boolean;
                                 Why    : Unbounded_String;
                                 Thing  : Driver.World.Thing_Id;
                              begin
                                 Driver.Instrument.Read_Segment (Reply, Driver.Images.Width (O.Images (E)),
                                                                 Driver.Images.Height (O.Images (E)), Region, Score,
                                                                 Ok, Why);
                                 if Ok and then Driver.Images.Count (Region) > 0 then
                                    Driver.World.Offline.Adopt (Bench, E, O, Region, Thing);
                                    Things.Include (Thing, (Object => To_Unbounded_String (Name), Eye => E,
                                                            Prompt => Px));
                                    Ada.Text_IO.Put_Line
                                      ("beat" & Beat'Image & ": " & Name & " adopted as thing" & Thing'Image
                                       & " in eye" & E'Image & " at (" & Image (Px.U, 0) & "," & Image (Px.V, 0)
                                       & "):" & Driver.Images.Count (Region)'Image & " pixels");
                                 else
                                    Ada.Text_IO.Put_Line ("beat" & Beat'Image & ": " & Name & ": no segment in eye"
                                                          & E'Image & ": " & To_String (Why));
                                 end if;
                                 Done := True;
                              end;
                           end if;
                        end;
                     end if;
                  end loop;
                  if not Done then
                     Ada.Text_IO.Put_Line ("beat" & Beat'Image & ": " & Name & " is in no eye");
                  end if;
               end;
            end if;
         end;
      end loop;
   end Adopt_Objects;

   procedure Robot_Message (Data : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
      O   : Observation;
   begin
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok then
         return;
      end if;
      if Req.Kind = Driver.Protocol.Reset then
         Episodes := Episodes + 1;
         Driver.World.Offline.New_Episode (Bench);
         Things.Clear;
         Adopted_Yet := False;
      end if;
      if not Driver.Protocol.Has_Observation (Req) then
         return;
      end if;
      if not Known then
         Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
         if Known then
            for C of Layout.Cameras loop
               Camera_Paths.Append (To_String (C.Path));
            end loop;
         end if;
      end if;
      if not Known then
         return;
      end if;
      Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), O);
      Driver.Services.Replay_Beat (Driver.Clock.Beat (Beat));
      Driver.Robot.Observe (Robot, O, Sent);
      Line_Of_Beat.Append (0);
      for L in Next_Line .. Truth.Last_Index loop
         if Same_State (O, Truth (L)) then
            Line_Of_Beat.Replace_Element (Beat, L);
            Next_Line := L + 1;
            Paired := Paired + 1;
            exit;
         end if;
      end loop;
      if Line_Of_Beat (Beat) > 0 then
         declare
            Still : constant Boolean := Driver.Robot.Still (Robot);
         begin
            Driver.World.Offline.Observe (Bench, Natural (O.Images.Length), Camera_Of'Access, Up, Still, O);
            Last := Beat;
            if Driver.World.Offline.Surface_Count (Bench) /= Last_Surfaces then
               Last_Surfaces := Driver.World.Offline.Surface_Count (Bench);
               Ada.Text_IO.Put_Line ("beat" & Beat'Image & ":" & Last_Surfaces'Image & " surfaces");
            end if;
            if Still and then not Adopted_Yet and then Last_Surfaces > 0 then
               Adopted_Yet := True;
               Adopt_Objects (O);
            end if;
         end;
      end if;
      Beat := Beat + 1;
   end Robot_Message;

   procedure Driver_Message (Data : Driver.Bytes.Byte_Array) is
      use Driver.Msgpack;
      Doc    : Document;
      Ok     : Boolean;
      Action : Node;
   begin
      Decode (Data, Doc, Ok);
      Action := (if Ok and then Known
                 then Element (Doc, Lookup (Doc, Lookup (Doc, Root (Doc), "payload"), "result"), 1)
                 else No_Node);
      if Action /= No_Node then
         Sent := Driver.Replies.Read_Action (Layout, Doc, Action);
      end if;
   end Driver_Message;

   ---------------------------------------------------------------------------
   --  The score

   procedure Report is
      L         : constant Truth_Line := Truth (Line_Of_Beat (Last));
      Placed_At : Placed_Maps.Map;
      Lowest    : Real_Array (1 .. Natural (L.Objects.Length));
      N         : Natural := 0;
      Table_Top : Real := 0.0;

      function Plane_Z (P : Driver.Geometry.Plane_Estimate; X, Y : Real) return Real is
        ((P.Normal * P.Centre - P.Normal (1) * X - P.Normal (2) * Y) / P.Normal (3));
   begin
      for C in L.Objects.Iterate loop
         if Object_Keys.Contains (Pose_Maps.Key (C)) then
            declare
               S : constant Placed := Place (Mesh_Of (Object_Keys (Pose_Maps.Key (C))), Pose_Maps.Element (C));
            begin
               if not S.Triangles.Is_Empty then
                  Placed_At.Include (Pose_Maps.Key (C), S);
                  N := N + 1;
                  Lowest (N) := S.Lowest (3);
                  Ada.Text_IO.Put_Line ("object " & Pose_Maps.Key (C) & ": lowest point at z " & Mm (S.Lowest (3))
                                        & " mm," & S.Triangles.Length'Image & " triangles");
               end if;
            end;
         end if;
      end loop;
      if N = 0 then
         Ada.Text_IO.Put_Line ("no object has a mesh");
         return;
      end if;
      Table_Top := Driver.Stats.Median (Lowest (1 .. N));
      Ada.Text_IO.Put_Line ("table top (median of the objects' lowest points): z " & Mm (Table_Top) & " mm");

      for F in 1 .. Driver.World.Offline.Surface_Count (Bench) loop
         declare
            S    : constant Driver.World.Supports.Surface :=
              Driver.World.Offline.Surface_Of (Bench, Driver.World.Surface_Id (F));
            P    : constant Driver.Geometry.Plane_Estimate := S.Plane;
            Tilt : constant Real := Degrees_Per_Radian * Arctan (abs Cross (P.Normal, Up.Unit_Vector),
                                                                 P.Normal * Up.Unit_Vector);
         begin
            Ada.Text_IO.Put_Line
              ("surface" & F'Image & ":" & S.Members.Length'Image & " points, centre (" & Mm (P.Centre (1)) & ", "
               & Mm (P.Centre (2)) & ", " & Mm (P.Centre (3)) & ") mm, tilt " & Image (Tilt, 2)
               & " deg, reach " & Mm (S.High_1 - S.Low_1) & " by " & Mm (S.High_2 - S.Low_2) & " mm; "
               & Mm (P.Centre (3) - Table_Top) & " mm off the table top at its centre");
         end;
      end loop;

      for C in Things.Iterate loop
         declare
            T      : constant Driver.World.Thing_Id := Adopted_Maps.Key (C);
            A      : constant Adopted := Adopted_Maps.Element (C);
            Name   : constant String := To_String (A.Object);
            Points : constant Driver.World.Pairs.Match_Vectors.Vector := Driver.World.Offline.Points_Of (Bench, T);
            Line   : Unbounded_String := To_Unbounded_String
              ("thing" & T'Image & " (" & Name & ", eye" & A.Eye'Image & "):" & Points.Length'Image & " points");
         begin
            if not Points.Is_Empty and then Placed_At.Contains (Name) then
               declare
                  Own    : constant Placed := Placed_At (Name);
                  Off    : Real_Array (1 .. Natural (Points.Length));
                  Sigmas : Real_Array (1 .. Natural (Points.Length));
                  Count_Of : array (1 .. Natural (Placed_At.Length)) of Natural := [others => 0];
                  Names  : Name_Vectors.Vector;
               begin
                  for P in Placed_At.Iterate loop
                     Names.Append (Placed_Maps.Key (P));
                  end loop;
                  for I in Off'Range loop
                     declare
                        M     : constant Driver.World.Pairs.Match := Points (I);
                        Sigma : constant Real :=
                          Sqrt ((M.Point.Covariance (1, 1) + M.Point.Covariance (2, 2) + M.Point.Covariance (3, 3))
                                / 3.0);
                        Best  : Real := Real'Last;
                        Which : Natural := 0;
                     begin
                        Off (I) := Distance (Own, M.Point.Mean);
                        Sigmas (I) := (if Sigma > 0.0 then Off (I) / Sigma else Real'Last);
                        for K in Names.First_Index .. Names.Last_Index loop
                           declare
                              D : constant Real := Distance (Placed_At (Names (K)), M.Point.Mean);
                           begin
                              if D < Best then
                                 Best := D;
                                 Which := K;
                              end if;
                           end;
                        end loop;
                        if Which > 0 then
                           Count_Of (Which) := Count_Of (Which) + 1;
                        end if;
                     end;
                  end loop;
                  declare
                     Most : Positive := Count_Of'First;
                  begin
                     for K in Count_Of'Range loop
                        if Count_Of (K) > Count_Of (Most) then
                           Most := K;
                        end if;
                     end loop;
                     Append (Line, "; off its object's surface: median " & Mm (Driver.Stats.Median (Off)) & " mm, 90% "
                             & Mm (Quantile (Off, 0.9)) & " mm, largest " & Mm (Largest (Off)) & " mm (median "
                             & Image (Driver.Stats.Median (Sigmas), 2) & " of their own sigma); most lie on "
                             & Names (Most) & " (" & Count_Of (Most)'Image & " of" & Points.Length'Image & ")");
                  end;
                  declare
                     Centre : constant Point_Estimate := Driver.World.Offline.Centre (Bench, T);
                     Under  : constant Driver.World.Surface_Id'Base := Driver.World.Offline.Resting_On (Bench, T);
                     Height : constant Estimate := Driver.World.Offline.Height_Above_Support (Bench, T);
                     True_H : constant Real := Own.Lowest (3) - Table_Top;
                  begin
                     Append (Line, "; centre " & Mm (abs (Centre.Mean - Own.Middle)) & " mm from its mesh's middle");
                     if Under = 0 then
                        Append (Line, "; rests on nothing found");
                     else
                        declare
                           P : constant Driver.Geometry.Plane_Estimate :=
                             Driver.World.Offline.Plane_Of (Bench, Under);
                        begin
                           Append (Line, "; rests on surface" & Under'Image & " at " & Mm (Height.Value) & " +- "
                                   & Mm (Height.Sigma) & " mm (truth " & Mm (True_H) & " mm, off by "
                                   & Mm (Height.Value - True_H) & " mm); that surface is "
                                   & Mm (Plane_Z (P, Own.Lowest (1), Own.Lowest (2)) - Table_Top)
                                   & " mm off the table top under it");
                        end;
                     end if;
                  end;
               end;
            end if;
            Ada.Text_IO.Put_Line (To_String (Line));
         end;
      end loop;
   end Report;

   procedure Configure_Instrument (Address : String) is
      Colon : constant Natural := Ada.Strings.Fixed.Index (Address, ":", Ada.Strings.Backward);
   begin
      if Colon = 0 then
         Line (Core, "a service address is HOST:PORT, not " & Address);
         return;
      end if;
      Driver.Services.Configure (Driver.Services.Instrument, Address (Address'First .. Colon - 1),
                                 Natural'Value (Address (Colon + 1 .. Address'Last)));
   end Configure_Instrument;

begin
   if Ada.Command_Line.Argument_Count < 4 or else Ada.Command_Line.Argument (3) /= "--inst" then
      Line (Core, "usage: world_check RECORDING TRUTH --inst HOST:PORT");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Path := To_Unbounded_String (Ada.Command_Line.Argument (1));
   Truth_Path := To_Unbounded_String (Ada.Command_Line.Argument (2));
   Configure_Instrument (Ada.Command_Line.Argument (4));
   Read_Truth (To_String (Truth_Path));
   Ada.Text_IO.Put_Line ("truth:" & Truth.Length'Image & " observation lines," & Lenses.Length'Image & " cameras,"
                         & Object_Keys.Length'Image & " objects");
   --  Every service call answered live: the estimators ask other questions
   --  than the recorded run did.
   Driver.Services.Start_Replay ([others => False]);
   Driver.Recording.Open (R, To_String (Path), Opened);
   if not Opened then
      Line (Core, "cannot open the recording " & To_String (Path));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   while More loop
      Driver.Recording.Next (R, Kind, Ns, Payload, More);
      exit when not More;
      if Kind = Driver.Recording.Robot_Message then
         Payload.Query (Robot_Message'Access);
      elsif Kind = Driver.Recording.Driver_Message then
         Payload.Query (Driver_Message'Access);
      end if;
   end loop;
   Driver.Recording.Close (R);
   Ada.Text_IO.Put_Line ("replayed" & Beat'Image & " beats," & Paired'Image & " paired with truth,"
                         & Episodes'Image & " episode resets");
   if Last > 0 then
      Report;
   end if;
   Driver.Services.Shut_Down;
end World_Check;
