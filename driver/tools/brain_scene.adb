--  brain_scene RECORDING TRUTH PREFIX LAG BEAT...
--  brain_scene RECORDING TRUTH --survey
--
--  The scenes the brain's binding of names is measured on (brain_measure
--  bind): every eye's picture at each BEAT of a recording with side-file
--  truth (harness/robodojo_truth), and beside each picture what the truth
--  says every pixel of it shows, drawn from the true poses, cameras and
--  visual meshes with the nearest surface winning: one of the objects, the
--  robot, or neither. The labels only score what the binder did; nothing
--  the binder or the brain is given comes from them.
--
--  An image trails the readings it arrives with (Driver.Robot.Image_Lag):
--  the picture of beat B shows the scene as the truth paired with beat
--  B - LAG has it, and its labels are drawn from that truth. The truth is
--  paired with beats by the readings it repeats.
--
--  For each BEAT and eye E it writes PREFIX.BEAT.E.ppm (the picture),
--  PREFIX.BEAT.E.pgm (the labels: 0 neither, K the K-th object of the
--  truth's geometry line, 255 the robot) and PREFIX.BEAT.E.lit.ppm (the
--  objects tinted on the picture and the robot darkened, to check the truth
--  and the lag against the image), and prints one JSON line per beat: the
--  truth line drawn, the objects, and each eye's count of pixels of every
--  object.
--
--  --survey prints a line per beat instead, to choose scenes by: for each
--  eye, how far in pixels any object's root moved in its image between the
--  truth of the beat and that of each of the Lag_Span beats before it, and
--  how many objects' roots fall in its image.
--
--  The recording is read forward only, so it may come through a pipe:
--    zstd -dc wire.rec.zst | brain_scene /dev/stdin <(zstd -dc truth.jsonl.zst) PREFIX 1 20 300

with Ada.Command_Line;
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
with Driver.Images;
with Driver.Json;
with Driver.Log;
with Driver.Numerics;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;

procedure Brain_Scene is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use type Ada.Containers.Count_Type;
   use type Driver.Json.Kind;
   use type Driver.Recording.Record_Kind;
   use type Driver.Real_Array;
   use type Driver.Observations.Camera_Id;
   use type Interfaces.Unsigned_8;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;
   subtype Eye_Id is Driver.Observations.Camera_Id;

   Lag_Span : constant := 3;
   --  How many beats before a scene's beat its eyes are checked to have stood still over.

   Robot_Label : constant Interfaces.Unsigned_8 := 255;

   package Pose_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Rigid);
   package Value_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Real_Array);
   package Key_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String);
   package Name_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   package Beat_Vectors is new Ada.Containers.Vectors (Positive, Natural);

   function Image (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Both));

   ---------------------------------------------------------------------------
   --  The truth

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

   type Lens is record
      K             : Mat3 := Identity3;
      Width, Height : Natural := 0;
   end record;

   package Lens_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Lens);

   type Truth_Line is record
      Cameras : Pose_Maps.Map;    --  camera name -> optical frame
      Objects : Pose_Maps.Map;    --  object name -> pose of its root
      Links   : Pose_Maps.Map;    --  robot link name -> pose of its root
      State   : Value_Maps.Map;   --  state key -> the observation's own values
   end record;

   package Truth_Vectors is new Ada.Containers.Vectors (Positive, Truth_Line);

   Truth       : Truth_Vectors.Vector;
   Lenses      : Lens_Maps.Map;
   Store       : Unbounded_String;
   Object_Keys : Key_Maps.Map;      --  object name -> geometry file in the store
   Link_Keys   : Key_Maps.Map;      --  robot link name -> geometry file in the store
   Objects     : Name_Vectors.Vector;   --  in the order of the geometry line: label K is Objects (K)

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
               Things : constant Node := Lookup (Doc, Root (Doc), "objects");
               Links  : constant Node := Lookup (Doc, Root (Doc), "links");
            begin
               Store := To_Unbounded_String (Text (Doc, Lookup (Doc, Root (Doc), "store")));
               for I in 1 .. Count (Doc, Things) loop
                  if Kind_Of (Doc, Member_Value (Doc, Things, I)) = String_Value then
                     Object_Keys.Include (Member_Name (Doc, Things, I), Text (Doc, Member_Value (Doc, Things, I)));
                     Objects.Append (Member_Name (Doc, Things, I));
                  end if;
               end loop;
               for I in 1 .. Count (Doc, Links) loop
                  if Kind_Of (Doc, Member_Value (Doc, Links, I)) = String_Value then
                     Link_Keys.Include (Member_Name (Doc, Links, I), Text (Doc, Member_Value (Doc, Links, I)));
                  end if;
               end loop;
            end;
         elsif Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "observation" then
            declare
               L       : Truth_Line;
               Cameras : constant Node := Lookup (Doc, Root (Doc), "cameras");
               Things  : constant Node := Lookup (Doc, Root (Doc), "objects");
               State   : constant Node := Lookup (Doc, Root (Doc), "state");
               Links   : constant Node := Lookup (Doc, Root (Doc), "links");
            begin
               for I in 1 .. Count (Doc, State) loop
                  L.State.Include (Member_Name (Doc, State, I), Numbers_Of (Doc, Member_Value (Doc, State, I)));
               end loop;
               for I in 1 .. Count (Doc, Things) loop
                  L.Objects.Include (Member_Name (Doc, Things, I),
                                     Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, Member_Value (Doc, Things, I),
                                                                               "pose"))));
               end loop;
               for I in 1 .. Count (Doc, Links) loop
                  L.Links.Include (Member_Name (Doc, Links, I), Quaternion_Pose (Numbers_Of (Doc, Member_Value (Doc, Links, I))));
               end loop;
               for I in 1 .. Count (Doc, Cameras) loop
                  declare
                     Name : constant String := Member_Name (Doc, Cameras, I);
                     C    : constant Node := Member_Value (Doc, Cameras, I);
                     K    : constant Node := Lookup (Doc, C, "K");
                  begin
                     L.Cameras.Include (Name, Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, C, "pose"))));
                     --  Only pinhole cameras: every eye of these recordings has a K.
                     if not Lenses.Contains (Name) and then Kind_Of (Doc, K) = Array_Value then
                        declare
                           E          : Lens;
                           Resolution : constant Real_Array := Numbers_Of (Doc, Lookup (Doc, C, "resolution"));
                        begin
                           E.Width := Natural (Resolution (1));
                           E.Height := Natural (Resolution (2));
                           for A in 1 .. 3 loop
                              for B in 1 .. 3 loop
                                 E.K (A, B) := Number (Doc, Element (Doc, Element (Doc, K, A), B));
                              end loop;
                           end loop;
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

   ---------------------------------------------------------------------------
   --  Meshes, as world_check reads them from the truth's store

   type Triangle is record
      A, B, C : Vec3;
   end record;

   package Triangle_Vectors is new Ada.Containers.Vectors (Positive, Triangle);
   package Vertex_Vectors is new Ada.Containers.Vectors (Positive, Vec3);

   type Mesh is record
      Vertices  : Vertex_Vectors.Vector;     --  in the object's frame
      Triangles : Triangle_Vectors.Vector;
   end record;

   package Mesh_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Mesh);
   Meshes : Mesh_Maps.Map;

   type Bytes_Access is access Driver.Bytes.Byte_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Bytes.Byte_Array, Bytes_Access);

   function Words (Path : String) return Bytes_Access is
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
      F      : Ada.Text_IO.File_Type;
      Doc    : Document;
      Ok     : Boolean;
      Why    : Unbounded_String;
      Result : Mesh;
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

   ---------------------------------------------------------------------------
   --  Drawing the truth into an eye

   type Label_Array is array (Natural range <>) of Interfaces.Unsigned_8;
   type Depth_Array is array (Natural range <>) of Real;   --  1 / depth; 0 where nothing is drawn

   type Label_Access is access Label_Array;
   type Depth_Access is access Depth_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Label_Array, Label_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Depth_Array, Depth_Access);

   procedure Pixel_Of (L : Lens; Optical : Rigid; Point : Vec3; U, V, Depth : out Real) is
      In_Eye : constant Vec3 := Transpose (Optical.Rotation) * (Point - Optical.Translation);
   begin
      Depth := In_Eye (3);
      U := 0.0;
      V := 0.0;
      if Depth > 0.0 then
         U := L.K (1, 1) * In_Eye (1) / Depth + L.K (1, 3);
         V := L.K (2, 2) * In_Eye (2) / Depth + L.K (2, 3);
      end if;
   end Pixel_Of;

   procedure Draw
     (M       : Mesh;
      Pose    : Rigid;
      L       : Lens;
      Optical : Rigid;
      Label   : Interfaces.Unsigned_8;
      Labels  : in out Label_Array;
      Nearer  : in out Depth_Array)
   is
      --  Every triangle wholly in front of the eye, filled where pixel centres
      --  fall in it, nearest first: 1 / depth is affine across the image of a
      --  plane, so it is interpolated with the pixel's own weights.
      Half : constant := 0.5;   --  the centre of a pixel (Driver.Images)
   begin
      for T of M.Triangles loop
         declare
            Corners : constant array (1 .. 3) of Vec3 := [T.A, T.B, T.C];
            U, V, Inv : Real_Array (1 .. 3);
            Front     : Boolean := True;
         begin
            for K in Corners'Range loop
               declare
                  Depth : Real;
               begin
                  Pixel_Of (L, Optical, Pose * Corners (K), U (K), V (K), Depth);
                  Front := Front and then Depth > 0.0;
                  Inv (K) := (if Depth > 0.0 then 1.0 / Depth else 0.0);
               end;
            end loop;
            if Front then
               declare
                  Area : constant Real := (U (2) - U (1)) * (V (3) - V (1)) - (U (3) - U (1)) * (V (2) - V (1));
                  --  The triangle's pixel box, cut to the image before it is
                  --  counted in pixels: a corner just in front of the eye lands
                  --  arbitrarily far out.
                  function Cut (X : Real; Size : Natural) return Integer is
                    (Integer (Real'Floor (Real'Max (-1.0, Real'Min (Real (Size), X)))));
                  C0 : constant Integer := Integer'Max (0, Cut (Real'Min (U (1), Real'Min (U (2), U (3))), L.Width));
                  C1 : constant Integer :=
                    Integer'Min (L.Width - 1, Cut (Real'Max (U (1), Real'Max (U (2), U (3))), L.Width));
                  R0 : constant Integer := Integer'Max (0, Cut (Real'Min (V (1), Real'Min (V (2), V (3))), L.Height));
                  R1 : constant Integer :=
                    Integer'Min (L.Height - 1, Cut (Real'Max (V (1), Real'Max (V (2), V (3))), L.Height));
               begin
                  if Area /= 0.0 then
                     for Row in R0 .. R1 loop
                        for Column in C0 .. C1 loop
                           declare
                              X  : constant Real := Real (Column) + Half;
                              Y  : constant Real := Real (Row) + Half;
                              W1 : constant Real := ((U (2) - X) * (V (3) - Y) - (U (3) - X) * (V (2) - Y)) / Area;
                              W2 : constant Real := ((U (3) - X) * (V (1) - Y) - (U (1) - X) * (V (3) - Y)) / Area;
                              W3 : constant Real := 1.0 - W1 - W2;
                              At_Pixel : constant Natural := Row * L.Width + Column;
                           begin
                              if W1 >= 0.0 and then W2 >= 0.0 and then W3 >= 0.0 then
                                 declare
                                    Here : constant Real := W1 * Inv (1) + W2 * Inv (2) + W3 * Inv (3);
                                 begin
                                    if Here > Nearer (At_Pixel) then
                                       Nearer (At_Pixel) := Here;
                                       Labels (At_Pixel) := Label;
                                    end if;
                                 end;
                              end if;
                           end;
                        end loop;
                     end loop;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Draw;

   ---------------------------------------------------------------------------
   --  Files

   procedure Write_Bytes (Name, Header : String; Data : Driver.Bytes.Byte_Array) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
   begin
      Create (F, Out_File, Name);
      Write (F, Driver.Bytes.To_Bytes (Header));
      Write (F, Data);
      Close (F);
   end Write_Bytes;

   procedure Write_Picture (Name : String; I : Driver.Images.Image) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      procedure Put_Pixels (RGB : Driver.Bytes.Byte_Array) is
      begin
         Write (F, RGB);
      end Put_Pixels;
   begin
      Create (F, Out_File, Name);
      Write (F, Driver.Bytes.To_Bytes ("P6" & ASCII.LF & Image (Driver.Images.Width (I)) & " "
                                       & Image (Driver.Images.Height (I)) & ASCII.LF & "255" & ASCII.LF));
      Driver.Images.Query (I, Put_Pixels'Access);
      Close (F);
   end Write_Picture;

   type Tint is array (1 .. 3) of Natural;
   Tints : constant array (0 .. 5) of Tint :=
     [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], [0, 255, 255]];
   --  For looking only: neighbouring labels get different colours.

   ---------------------------------------------------------------------------
   --  The replay

   R       : Driver.Recording.Reader;
   Opened  : Boolean;
   Kind    : Driver.Recording.Record_Kind;
   Ns      : Long_Long_Integer;
   Payload : Driver.Bytes.Buffer;
   More    : Boolean := True;
   Layout  : Driver.Observations.Layout;
   Known   : Boolean := False;
   Beat    : Natural := 0;
   Next_Line : Positive := 1;
   Wanted  : Beat_Vectors.Vector;
   Prefix  : Unbounded_String;

   package Line_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   Line_Of_Beat : Line_Vectors.Vector;   --  the truth line paired with each beat; 0 for none

   function Same_State (O : Driver.Observations.Observation; L : Truth_Line) return Boolean is
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

   function Camera_Name (E : Eye_Id) return String is
      --  The truth camera whose name is a component of the eye's layout path.
      Path : constant String := "/" & To_String (Layout.Cameras (E).Path) & "/";
   begin
      for C in Lenses.Iterate loop
         if Ada.Strings.Fixed.Index (Path, "/" & Lens_Maps.Key (C) & "/") > 0 then
            return Lens_Maps.Key (C);
         end if;
      end loop;
      return "";
   end Camera_Name;

   function Moved (E : Eye_Id) return Real is
      --  The farthest any object's root moved in the eye's image between the
      --  truth of this beat and that of any of the Lag_Span beats before it.
      Name : constant String := Camera_Name (E);
      Now  : constant Truth_Line := Truth (Line_Of_Beat (Beat));
      Most : Real := 0.0;
   begin
      for Back in 1 .. Lag_Span loop
         if Beat >= Back and then Line_Of_Beat (Beat - Back) > 0 then
            declare
               Then_Line : constant Truth_Line := Truth (Line_Of_Beat (Beat - Back));
            begin
               for C in Now.Objects.Iterate loop
                  declare
                     U0, V0, D0, U1, V1, D1 : Real;
                  begin
                     Pixel_Of (Lenses (Name), Now.Cameras (Name), Pose_Maps.Element (C).Translation, U0, V0, D0);
                     Pixel_Of (Lenses (Name), Then_Line.Cameras (Name),
                               Then_Line.Objects (Pose_Maps.Key (C)).Translation, U1, V1, D1);
                     if D0 > 0.0 and then D1 > 0.0 then
                        Most := Real'Max (Most, Sqrt ((U1 - U0) ** 2 + (V1 - V0) ** 2));
                     end if;
                  end;
               end loop;
            end;
         else
            return Real'Last;
         end if;
      end loop;
      return Most;
   end Moved;

   procedure Write_Scene (O : Driver.Observations.Observation; Shown : Natural) is
      --  Shown: the beat whose truth the pictures show.
      L    : constant Truth_Line := Truth (Line_Of_Beat (Shown));
      Line : Unbounded_String := To_Unbounded_String
        ("{""beat"":" & Image (Beat) & ",""truth_line"":" & Image (Line_Of_Beat (Shown)) & ",""objects"":[");
   begin
      for K in Objects.First_Index .. Objects.Last_Index loop
         Append (Line, (if K > Objects.First_Index then "," else "") & Driver.Json.Quote (Objects (K)));
      end loop;
      Append (Line, "],""eyes"":[");
      for E in 1 .. Eye_Id'Base (Natural (O.Images.Length)) loop
         declare
            Name : constant String := Camera_Name (E);
            Base : constant String := To_String (Prefix) & "." & Image (Beat) & "." & Image (Natural (E));
         begin
            Append (Line, (if E > 1 then "," else "") & "{""eye"":" & Image (Natural (E)) & ",""camera"":"
                    & Driver.Json.Quote (Name));
            if Name /= "" and then Driver.Observations.Has_Image (O, E) and then L.Cameras.Contains (Name) then
               declare
                  Lens_Of : constant Lens := Lenses (Name);
                  Optical : constant Rigid := L.Cameras (Name);
                  W       : constant Natural := Lens_Of.Width;
                  H       : constant Natural := Lens_Of.Height;
                  Labels  : Label_Access := new Label_Array'(0 .. W * H - 1 => 0);
                  Nearer  : Depth_Access := new Depth_Array'(0 .. W * H - 1 => 0.0);
                  Counts  : array (0 .. Natural (Robot_Label)) of Natural := [others => 0];
                  Bytes   : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (W * H));
                  Lit     : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (3 * W * H));
                  Picture : constant Driver.Images.Image := O.Images (E);
               begin
                  for K in Objects.First_Index .. Objects.Last_Index loop
                     if L.Objects.Contains (Objects (K)) then
                        Draw (Mesh_Of (Object_Keys (Objects (K))), L.Objects (Objects (K)), Lens_Of, Optical,
                              Interfaces.Unsigned_8 (K), Labels.all, Nearer.all);
                     end if;
                  end loop;
                  for C in L.Links.Iterate loop
                     if Link_Keys.Contains (Pose_Maps.Key (C)) then
                        Draw (Mesh_Of (Link_Keys (Pose_Maps.Key (C))), Pose_Maps.Element (C), Lens_Of, Optical,
                              Robot_Label, Labels.all, Nearer.all);
                     end if;
                  end loop;
                  for P in Labels'Range loop
                     declare
                        use type Ada.Streams.Stream_Element_Offset;
                        At_Byte : constant Ada.Streams.Stream_Element_Offset := Ada.Streams.Stream_Element_Offset (P);
                        Column  : constant Natural := P mod W;
                        Row     : constant Natural := P / W;
                        Label   : constant Natural := Natural (Labels (P));
                        RGB     : constant Tint := [Driver.Images.Red (Picture, Column, Row),
                                                    Driver.Images.Green (Picture, Column, Row),
                                                    Driver.Images.Blue (Picture, Column, Row)];
                     begin
                        Counts (Label) := Counts (Label) + 1;
                        Bytes (At_Byte + 1) := Ada.Streams.Stream_Element (Label);
                        for J in 1 .. 3 loop
                           Lit (3 * At_Byte + Ada.Streams.Stream_Element_Offset (J)) := Ada.Streams.Stream_Element
                             (if Labels (P) = Robot_Label then RGB (J) / 4
                              elsif Label > 0 then (RGB (J) + Tints (Label mod Tints'Length) (J)) / 2
                              else RGB (J));
                        end loop;
                     end;
                  end loop;
                  Write_Picture (Base & ".ppm", Picture);
                  Write_Bytes (Base & ".pgm", "P5" & ASCII.LF & Image (W) & " " & Image (H) & ASCII.LF & "255" & ASCII.LF,
                               Bytes);
                  Write_Bytes (Base & ".lit.ppm", "P6" & ASCII.LF & Image (W) & " " & Image (H) & ASCII.LF & "255"
                               & ASCII.LF, Lit);
                  Append (Line, ",""image"":" & Driver.Json.Quote (Base & ".ppm") & ",""labels"":"
                          & Driver.Json.Quote (Base & ".pgm") & ",""robot"":"
                          & Image (Counts (Natural (Robot_Label))) & ",""pixels"":{");
                  for K in Objects.First_Index .. Objects.Last_Index loop
                     Append (Line, (if K > Objects.First_Index then "," else "") & Driver.Json.Quote (Objects (K)) & ":"
                             & Image (Counts (K)));
                  end loop;
                  Append (Line, "}");
                  Free (Labels);
                  Free (Nearer);
               end;
            end if;
            Append (Line, "}");
         end;
      end loop;
      Append (Line, "]}");
      Ada.Text_IO.Put_Line (To_String (Line));
      Ada.Text_IO.Flush;
   end Write_Scene;

   Survey : Boolean := False;   --  brain_scene RECORDING TRUTH --survey
   Lag    : Natural := 0;       --  the beats a picture trails its readings by

   procedure Survey_Line (O : Driver.Observations.Observation) is
      --  One line per beat, to choose scenes by: how far each eye moved
      --  (Moved) and how many objects' roots fall in its image.
      L    : constant Truth_Line := Truth (Line_Of_Beat (Beat));
      Line : Unbounded_String := To_Unbounded_String (Image (Beat));
   begin
      for E in 1 .. Eye_Id'Base (Natural (O.Images.Length)) loop
         declare
            Name  : constant String := Camera_Name (E);
            Roots : Natural := 0;
         begin
            if Name /= "" and then L.Cameras.Contains (Name) then
               for C in L.Objects.Iterate loop
                  declare
                     U, V, Depth : Real;
                  begin
                     Pixel_Of (Lenses (Name), L.Cameras (Name), Pose_Maps.Element (C).Translation, U, V, Depth);
                     if Depth > 0.0 and then U in 0.0 .. Real (Lenses (Name).Width)
                       and then V in 0.0 .. Real (Lenses (Name).Height)
                     then
                        Roots := Roots + 1;
                     end if;
                  end;
               end loop;
               Append (Line, " | eye " & Image (Natural (E)) & " moved " & Driver.Log.Image (Moved (E), 1)
                       & " px, " & Image (Roots) & " objects");
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line (To_String (Line));
   end Survey_Line;

   procedure Robot_Message (Data : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
      O   : Driver.Observations.Observation;
   begin
      Driver.Protocol.Decode (Data, Req, Ok);
      if not Ok or else not Driver.Protocol.Has_Observation (Req) then
         return;
      end if;
      if not Known then
         Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
      end if;
      if not Known then
         return;
      end if;
      Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), O);
      Line_Of_Beat.Append (0);
      for L in Next_Line .. Truth.Last_Index loop
         if Same_State (O, Truth (L)) then
            Line_Of_Beat.Replace_Element (Beat, L);
            Next_Line := L + 1;
            exit;
         end if;
      end loop;
      if Survey and then Line_Of_Beat (Beat) > 0 then
         Survey_Line (O);
      elsif Wanted.Contains (Beat) then
         if Beat >= Lag and then Line_Of_Beat (Beat - Lag) > 0 then
            Write_Scene (O, Beat - Lag);
         else
            Ada.Text_IO.Put_Line ("{""beat"":" & Image (Beat) & ",""truth_line"":0}");
         end if;
      end if;
      Beat := Beat + 1;
   end Robot_Message;

   Last_Wanted : Natural := 0;

begin
   Survey := Ada.Command_Line.Argument_Count = 3 and then Ada.Command_Line.Argument (3) = "--survey";
   if Ada.Command_Line.Argument_Count < 5 and then not Survey then
      Ada.Text_IO.Put_Line ("usage: brain_scene RECORDING TRUTH PREFIX LAG BEAT... | brain_scene RECORDING TRUTH --survey");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Prefix := To_Unbounded_String (Ada.Command_Line.Argument (3));
   if not Survey then
      Lag := Natural'Value (Ada.Command_Line.Argument (4));
   end if;
   for I in 5 .. Ada.Command_Line.Argument_Count loop
      Wanted.Append (Natural'Value (Ada.Command_Line.Argument (I)));
      Last_Wanted := Natural'Max (Last_Wanted, Wanted.Last_Element);
   end loop;
   if Survey then
      Last_Wanted := Natural'Last;
   end if;
   Read_Truth (Ada.Command_Line.Argument (2));
   Driver.Recording.Open (R, Ada.Command_Line.Argument (1), Opened);
   if not Opened then
      Ada.Text_IO.Put_Line ("cannot open the recording " & Ada.Command_Line.Argument (1));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   while More and then Beat <= Last_Wanted loop
      Driver.Recording.Next (R, Kind, Ns, Payload, More);
      if More and then Kind = Driver.Recording.Robot_Message then
         Payload.Query (Robot_Message'Access);
      end if;
   end loop;
   Driver.Recording.Close (R);
end Brain_Scene;
