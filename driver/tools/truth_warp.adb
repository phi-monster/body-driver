--  truth_warp TRUTH RUN_DIR [LAG [TABLE_HEIGHT [SUFFIX [OPTICAL_OFFSET]]]]
--
--  Where the simulator puts, in the second picture, the point of the scene
--  that a query point of the first picture shows: the projection the matchers'
--  answers are judged against.
--
--  TRUTH is the file the simulator wrote beside the run (harness/robodojo_truth,
--  read from standard input when it is /dev/stdin). RUN_DIR holds what
--  match_extract wrote of the recording: beats.txt (every observation's
--  readings, which pair it with its truth line as score does) and one
--  req_NNNN.txt per question asked of the instrument, with the camera and the
--  beat of each of its two pictures and its query points. Per question this
--  writes req_NNNN.truth: the surface point each query point shows (in the
--  first camera's frame, with its plane's normal), the part it belongs to, the
--  rigid motion of every part from the first picture's moment to the second
--  (in the first camera's frame to the second's), and where the point lands in
--  the second picture, with the verdict on whether it can be seen there.
--
--  The scene is rendered from the visual meshes the truth store holds (every
--  link, every object with a pose) into a z-buffer per picture, and the table
--  top is a plane of TABLE_HEIGHT metres, the one thing of the scene the store
--  does not hold. Each query point's ray is cut with the plane of the triangle
--  its pixel shows (not with the pixel's centre), so the sub-pixel position of
--  the point is exact on a planar surface.
--
--  LAG is how many truth lines before the one paired with an observation hold
--  the pose its picture was taken at (0: the same line). SUFFIX is appended to
--  the names of the files written (req_NNNN.truthLAGSUFFIX). OPTICAL_OFFSET is
--  how far, in metres along the camera's axis, the render's optical centre lies
--  from the camera frame the truth gives (0 by default).
--
--  A point's status: V seen in both pictures on a smooth surface; E seen but
--  within a pixel of a depth or part edge in either picture; O behind
--  something in the second picture; X outside the second picture; U on no
--  surface the store holds (a wall, the room, the sky).

with Ada.Command_Line;
with Ada.Directories;
with Ada.Unchecked_Deallocation;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Vectors;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Interfaces;
with Driver.Json;
with Driver.Numerics;

procedure Truth_Warp is

   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use type Driver.Json.Node;
   use type Driver.Real_Array;
   use type Interfaces.Integer_32;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   function Img (N : Integer) return String is (Ada.Strings.Fixed.Trim (Integer'Image (N), Ada.Strings.Left));

   function Padded (N : Natural) return String is
      S : constant String := Img (N);
   begin
      return (1 .. 4 - S'Length => '0') & S;
   end Padded;

   function Num (X : Real) return String is
      S : constant String := Real'Image (X);
   begin
      return " " & S;
   end Num;

   ---------------------------------------------------------------------------
   --  The truth lines

   package Pose_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Rigid);
   package Value_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Real_Array);

   type Truth_Line is record
      State   : Value_Maps.Map;
      Links   : Pose_Maps.Map;
      Cameras : Pose_Maps.Map;
      Objects : Pose_Maps.Map;
   end record;

   package Truth_Vectors is new Ada.Containers.Vectors (Positive, Truth_Line);
   Truth : Truth_Vectors.Vector;

   type Lens is record
      Fx, Fy, Cx, Cy : Real := 0.0;
      Width, Height  : Natural := 0;
   end record;

   package Lens_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Lens);
   Lenses : Lens_Maps.Map;

   Store : Unbounded_String;

   package Key_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String);
   Link_Keys, Object_Keys : Key_Maps.Map;

   function Numbers_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Real_Array is
      R : Real_Array (1 .. Driver.Json.Count (Doc, N));
   begin
      for I in R'Range loop
         R (I) := Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, I));
      end loop;
      return R;
   end Numbers_Of;

   function Quaternion_Pose (X : Real_Array) return Rigid is
     (Rotation    => To_Matrix ((W => X (X'First + 3), X => X (X'First + 4), Y => X (X'First + 5),
                                 Z => X (X'First + 6))),
      Translation => [X (X'First), X (X'First + 1), X (X'First + 2)]);

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
               Links   : constant Node := Lookup (Doc, Root (Doc), "links");
               Objects : constant Node := Lookup (Doc, Root (Doc), "objects");
            begin
               Store := To_Unbounded_String (Text (Doc, Lookup (Doc, Root (Doc), "store")));
               for I in 1 .. Count (Doc, Links) loop
                  if Kind_Of (Doc, Member_Value (Doc, Links, I)) = String_Value then
                     Link_Keys.Include (Member_Name (Doc, Links, I), Text (Doc, Member_Value (Doc, Links, I)));
                  end if;
               end loop;
               for I in 1 .. Count (Doc, Objects) loop
                  if Kind_Of (Doc, Member_Value (Doc, Objects, I)) = String_Value then
                     Object_Keys.Include (Member_Name (Doc, Objects, I), Text (Doc, Member_Value (Doc, Objects, I)));
                  end if;
               end loop;
            end;
         elsif Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "observation" then
            declare
               L       : Truth_Line;
               Links   : constant Node := Lookup (Doc, Root (Doc), "links");
               Objects : constant Node := Lookup (Doc, Root (Doc), "objects");
               Cameras : constant Node := Lookup (Doc, Root (Doc), "cameras");
               State   : constant Node := Lookup (Doc, Root (Doc), "state");
            begin
               for I in 1 .. Count (Doc, Links) loop
                  L.Links.Include (Member_Name (Doc, Links, I),
                                   Quaternion_Pose (Numbers_Of (Doc, Member_Value (Doc, Links, I))));
               end loop;
               for I in 1 .. Count (Doc, Objects) loop
                  if Lookup (Doc, Member_Value (Doc, Objects, I), "pose") /= No_Node then
                     L.Objects.Include (Member_Name (Doc, Objects, I),
                                        Quaternion_Pose
                                          (Numbers_Of (Doc, Lookup (Doc, Member_Value (Doc, Objects, I), "pose"))));
                  end if;
               end loop;
               for I in 1 .. Count (Doc, State) loop
                  L.State.Include (Member_Name (Doc, State, I), Numbers_Of (Doc, Member_Value (Doc, State, I)));
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
                        begin
                           E.Width := Natural (Resolution (1));
                           E.Height := Natural (Resolution (2));
                           E.Fx := Number (Doc, Element (Doc, Element (Doc, K, 1), 1));
                           E.Fy := Number (Doc, Element (Doc, Element (Doc, K, 2), 2));
                           E.Cx := Number (Doc, Element (Doc, Element (Doc, K, 1), 3));
                           E.Cy := Number (Doc, Element (Doc, Element (Doc, K, 2), 3));
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
   --  The recorded beats, paired with the truth lines by their readings

   package Beat_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   Line_Of : Beat_Vectors.Vector;   --  recorded beat -> truth line, 0 for none

   function Words_Of (Text : String) return Real_Array is
      Values : Real_Array (1 .. Text'Length);
      Count  : Natural := 0;
      I      : Natural := Text'First;
   begin
      while I <= Text'Last loop
         if Text (I) = ' ' then
            I := I + 1;
         else
            declare
               J : Natural := I;
            begin
               while J <= Text'Last and then Text (J) /= ' ' loop
                  J := J + 1;
               end loop;
               Count := Count + 1;
               Values (Count) := Real'Value (Text (I .. J - 1));
               I := J;
            end;
         end if;
      end loop;
      return Values (1 .. Count);
   end Words_Of;

   type Beat_State is record
      Names  : Unbounded_String;
      Values : Value_Maps.Map;
   end record;

   package Beat_State_Vectors is new Ada.Containers.Vectors (Natural, Beat_State);
   Beats : Beat_State_Vectors.Vector;

   procedure Read_Beats (Path : String) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            Line  : constant String := Ada.Text_IO.Get_Line (F);
            B     : Beat_State;
            First : Natural := Ada.Strings.Fixed.Index (Line, " | ");
         begin
            while First > 0 loop
               declare
                  Start : constant Natural := First + 3;
                  Next  : constant Natural := Ada.Strings.Fixed.Index (Line (Start .. Line'Last), " | ");
                  Last  : constant Natural := (if Next = 0 then Line'Last else Next - 1);
                  Part  : constant String := Line (Start .. Last);
                  Space : constant Natural := Ada.Strings.Fixed.Index (Part, " ");
               begin
                  if Space > 0 and then Part'Length > 6 and then Part (Part'First .. Part'First + 5) = "state/" then
                     B.Values.Include (Part (Part'First + 6 .. Space - 1), Words_Of (Part (Space + 1 .. Part'Last)));
                  end if;
                  First := Next;
               end;
            end loop;
            Beats.Append (B);
         end;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Beats;

   function Same_State (B : Beat_State; L : Truth_Line) return Boolean is
      Common : Natural := 0;
   begin
      for C in L.State.Iterate loop
         declare
            Key : constant String := Value_Maps.Key (C);
         begin
            if B.Values.Contains (Key) then
               declare
                  Mine   : constant Real_Array := B.Values (Key);
                  Theirs : constant Real_Array := Value_Maps.Element (C);
               begin
                  if Mine'Length /= Theirs'Length then
                     return False;
                  end if;
                  for I in Mine'Range loop
                     if abs (Mine (I) - Theirs (Theirs'First + I - Mine'First)) > 1.0E-9 * (1.0 + abs Mine (I)) then
                        return False;
                     end if;
                  end loop;
               end;
               Common := Common + 1;
            end if;
         end;
      end loop;
      return Common > 0;
   end Same_State;

   procedure Pair_Beats is
      Next   : Positive := 1;
      Paired : Natural := 0;
   begin
      for B in Beats.First_Index .. Beats.Last_Index loop
         Line_Of.Append (0);
         for L in Next .. Truth.Last_Index loop
            if Same_State (Beats (B), Truth (L)) then
               Line_Of (B) := L;
               Next := L + 1;
               Paired := Paired + 1;
               exit;
            end if;
         end loop;
      end loop;
      Ada.Text_IO.Put_Line (Img (Paired) & " of" & Natural'Image (Natural (Beats.Length)) & " beats paired with"
                            & Natural'Image (Natural (Truth.Length)) & " truth lines");
   end Pair_Beats;

   ---------------------------------------------------------------------------
   --  Meshes

   type Float_Array is array (Natural range <>) of Interfaces.IEEE_Float_32;
   type Int_Array is array (Natural range <>) of Interfaces.Integer_32;
   type Float_Access is access Float_Array;
   type Int_Access is access Int_Array;

   type Mesh is record
      Points : Float_Access;   --  x y z per vertex, in the part's frame
      Tris   : Int_Access;     --  three vertex indices per triangle
      Count  : Natural := 0;   --  triangles
      Verts  : Natural := 0;
   end record;

   package Mesh_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Mesh);
   Meshes : Mesh_Maps.Map;

   procedure Read_Floats (Path : String; Into : out Float_Access) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      N : Natural;
   begin
      Open (F, In_File, Path);
      N := Natural (Size (F)) / 4;
      Into := new Float_Array (0 .. N - 1);
      Float_Array'Read (Stream (F), Into.all);
      Close (F);
   end Read_Floats;

   procedure Read_Ints (Path : String; Into : out Int_Access) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      N : Natural;
   begin
      Open (F, In_File, Path);
      N := Natural (Size (F)) / 4;
      Into := new Int_Array (0 .. N - 1);
      Int_Array'Read (Stream (F), Into.all);
      Close (F);
   end Read_Ints;

   function Load (Key : String) return Mesh is
      use Driver.Json;
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
            Meshes_Node : constant Node := Lookup (Doc, Root (Doc), "meshes");
         begin
            for I in 1 .. Count (Doc, Meshes_Node) loop
               declare
                  M : constant Node := Element (Doc, Meshes_Node, I);
               begin
                  if Is_True (Doc, Lookup (Doc, M, "visual")) and then Result.Points = null then
                     declare
                        Counts  : Int_Access;
                        Indices : Int_Access;
                        Tris    : Natural := 0;
                     begin
                        Read_Floats (To_String (Store) & "/" & Text (Doc, Lookup (Doc, M, "points")), Result.Points);
                        Read_Ints (To_String (Store) & "/" & Text (Doc, Lookup (Doc, M, "counts")), Counts);
                        Read_Ints (To_String (Store) & "/" & Text (Doc, Lookup (Doc, M, "indices")), Indices);
                        Result.Verts := Result.Points'Length / 3;
                        for C of Counts.all loop
                           if C >= 3 then
                              Tris := Tris + Natural (C) - 2;
                           end if;
                        end loop;
                        Result.Tris := new Int_Array (0 .. 3 * Tris - 1);
                        declare
                           At_Index : Natural := 0;
                           Out_At   : Natural := 0;
                        begin
                           for C of Counts.all loop
                              for K in 1 .. Natural (C) - 2 loop
                                 Result.Tris (Out_At) := Indices (At_Index);
                                 Result.Tris (Out_At + 1) := Indices (At_Index + K);
                                 Result.Tris (Out_At + 2) := Indices (At_Index + K + 1);
                                 Out_At := Out_At + 3;
                              end loop;
                              At_Index := At_Index + Natural (C);
                           end loop;
                        end;
                        Result.Count := Tris;
                     end;
                  end if;
               end;
            end loop;
         end;
      end if;
      Meshes.Include (Key, Result);
      return Result;
   end Load;

   ---------------------------------------------------------------------------
   --  Parts: the links and the objects with a mesh

   type Part is record
      Name    : Unbounded_String;
      Is_Link : Boolean;
      Key     : Unbounded_String;
   end record;

   package Part_Vectors is new Ada.Containers.Vectors (Positive, Part);
   Parts : Part_Vectors.Vector;   --  part 0 is the table: part N here is part N + 1 outside

   Table_Height : Real := 0.765;
   Optical_Offset : Real := 0.0;

   function Pose_Of (L : Truth_Line; P : Part) return Rigid is
     (if P.Is_Link then L.Links (To_String (P.Name)) else L.Objects (To_String (P.Name)));

   function Has_Pose (L : Truth_Line; P : Part) return Boolean is
     (if P.Is_Link then L.Links.Contains (To_String (P.Name)) else L.Objects.Contains (To_String (P.Name)));

   ---------------------------------------------------------------------------
   --  Rendering

   type Plane_Array is array (Natural range <>) of Real;
   type Real_Access is access Plane_Array;
   type Int_Vector_Access is access Int_Array;

   procedure Free is new Ada.Unchecked_Deallocation (Plane_Array, Real_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Int_Array, Int_Vector_Access);

   Scratch_X, Scratch_Y, Scratch_Z : Real_Access;   --  a part's vertices on the screen, as many as the largest part has

   type Frame is record
      Camera, Beat : Natural := 0;
      Name         : Unbounded_String;
      Line         : Natural := 0;
      Lens         : Truth_Warp.Lens;
      Pose         : Rigid;       --  the optical frame in the world
      Depth        : Real_Access;   --  z of the nearest triangle per pixel, 0 where none
      Which        : Int_Vector_Access;   --  part index + 1 per pixel, 0 where none
      Tri          : Int_Vector_Access;   --  triangle per pixel
      Table        : Real_Access;   --  z of the table plane at the pixel's centre ray, 0 where none
   end record;

   function Camera_Name (Camera : Natural) return String is
     (case Camera is when 1 => "cam_head", when 2 => "cam_left_wrist", when 3 => "cam_right_wrist", when others => "");

   function Own_Link (Camera : Natural; Name : String) return Boolean is
      --  The camera's own housing is not in its picture.
      Robot : constant String := (if Camera = 2 then "robot0/" elsif Camera = 3 then "robot1/" else "");
   begin
      return Robot'Length > 0 and then Name'Length >= Robot'Length
        and then Name (Name'First .. Name'First + Robot'Length - 1) = Robot
        and then Ada.Strings.Fixed.Index (Name, "camera") > 0;
   end Own_Link;

   procedure Render (F : in out Frame; L : Truth_Line) is
      W : constant Natural := F.Lens.Width;
      H : constant Natural := F.Lens.Height;
      To_Camera : constant Rigid := Inverse (F.Pose);
      Near : constant Real := 0.001;
   begin
      F.Depth := new Plane_Array (0 .. W * H - 1);
      F.Which := new Int_Array (0 .. W * H - 1);
      F.Tri := new Int_Array (0 .. W * H - 1);
      F.Table := new Plane_Array (0 .. W * H - 1);
      F.Depth.all := [others => 0.0];
      F.Which.all := [others => 0];
      F.Tri.all := [others => 0];
      --  The table plane's depth along each pixel's centre ray.
      declare
         Eye : constant Real := F.Pose.Translation (3);
      begin
         for R in 0 .. H - 1 loop
            for C in 0 .. W - 1 loop
               declare
                  Ray   : constant Vec3 :=
                    [(Real (C) + 0.5 - F.Lens.Cx) / F.Lens.Fx, (Real (R) + 0.5 - F.Lens.Cy) / F.Lens.Fy, 1.0];
                  World : constant Vec3 := F.Pose.Rotation * Ray;
                  --  eye + s * World reaches the table height at s = (height - eye_z) / World_z, and s is the
                  --  depth along the optical axis because Ray's third component is one.
               begin
                  if abs World (3) > 1.0E-12 and then (Table_Height - Eye) / World (3) > 0.0 then
                     F.Table (R * W + C) := (Table_Height - Eye) / World (3);
                  else
                     F.Table (R * W + C) := 0.0;
                  end if;
               end;
            end loop;
         end loop;
      end;
      for P in Parts.First_Index .. Parts.Last_Index loop
         declare
            Pt : constant Part := Parts (P);
         begin
            if Has_Pose (L, Pt) and then not Own_Link (F.Camera, To_String (Pt.Name)) then
               declare
                  M     : constant Mesh := Load (To_String (Pt.Key));
                  Pose  : constant Rigid := To_Camera * Pose_Of (L, Pt);   --  the part's frame in the camera's
                  Px : Real_Access renames Scratch_X;
                  Py : Real_Access renames Scratch_Y;
                  Pz : Real_Access renames Scratch_Z;
               begin
                  if M.Points /= null and then M.Count > 0 then
                     for V in 0 .. M.Verts - 1 loop
                        declare
                           X : constant Vec3 :=
                             Pose * [Real (M.Points (3 * V)), Real (M.Points (3 * V + 1)), Real (M.Points (3 * V + 2))];
                        begin
                           Pz (V) := X (3);
                           if X (3) > Near then
                              Px (V) := F.Lens.Fx * X (1) / X (3) + F.Lens.Cx;
                              Py (V) := F.Lens.Fy * X (2) / X (3) + F.Lens.Cy;
                           else
                              Px (V) := 0.0;
                              Py (V) := 0.0;
                           end if;
                        end;
                     end loop;
                     for T in 0 .. M.Count - 1 loop
                        declare
                           I0 : constant Natural := Natural (M.Tris (3 * T));
                           I1 : constant Natural := Natural (M.Tris (3 * T + 1));
                           I2 : constant Natural := Natural (M.Tris (3 * T + 2));
                        begin
                           if Pz (I0) > Near and then Pz (I1) > Near and then Pz (I2) > Near then
                              declare
                                 X0 : constant Real := Px (I0);
                                 Y0 : constant Real := Py (I0);
                                 X1 : constant Real := Px (I1);
                                 Y1 : constant Real := Py (I1);
                                 X2 : constant Real := Px (I2);
                                 Y2 : constant Real := Py (I2);
                                 Min_X : constant Real := Real'Min (X0, Real'Min (X1, X2));
                                 Max_X : constant Real := Real'Max (X0, Real'Max (X1, X2));
                                 Min_Y : constant Real := Real'Min (Y0, Real'Min (Y1, Y2));
                                 Max_Y : constant Real := Real'Max (Y0, Real'Max (Y1, Y2));
                              begin
                                  if Max_X >= 0.0 and then Min_X < Real (W) and then Max_Y >= 0.0
                                    and then Min_Y < Real (H)
                                  then
                                    declare
                                       Area : constant Real := (X1 - X0) * (Y2 - Y0) - (X2 - X0) * (Y1 - Y0);
                                    begin
                                       if abs Area > 1.0E-12 then
                                          declare
                                             C0 : constant Natural :=
                                               Natural (Real'Max (0.0, Real'Floor (Min_X - 0.5)));
                                             C1 : constant Natural :=
                                               Natural (Real'Min (Real (W - 1), Real'Ceiling (Max_X)));
                                             R0 : constant Natural :=
                                               Natural (Real'Max (0.0, Real'Floor (Min_Y - 0.5)));
                                             R1 : constant Natural :=
                                               Natural (Real'Min (Real (H - 1), Real'Ceiling (Max_Y)));
                                             Iz0 : constant Real := 1.0 / Pz (I0);
                                             Iz1 : constant Real := 1.0 / Pz (I1);
                                             Iz2 : constant Real := 1.0 / Pz (I2);
                                          begin
                                             for R in R0 .. R1 loop
                                                for C in C0 .. C1 loop
                                                   declare
                                                      Sx : constant Real := Real (C) + 0.5;
                                                      Sy : constant Real := Real (R) + 0.5;
                                                      W0 : constant Real :=
                                                        ((X1 - Sx) * (Y2 - Sy) - (X2 - Sx) * (Y1 - Sy)) / Area;
                                                      W1 : constant Real :=
                                                        ((X2 - Sx) * (Y0 - Sy) - (X0 - Sx) * (Y2 - Sy)) / Area;
                                                      W2 : constant Real := 1.0 - W0 - W1;
                                                   begin
                                                      if W0 >= 0.0 and then W1 >= 0.0 and then W2 >= 0.0 then
                                                         declare
                                                            Z : constant Real := 1.0 / (W0 * Iz0 + W1 * Iz1 + W2 * Iz2);
                                                            At_Pixel : constant Natural := R * W + C;
                                                         begin
                                                            if F.Depth (At_Pixel) = 0.0
                                                              or else Z < F.Depth (At_Pixel)
                                                            then
                                                               F.Depth (At_Pixel) := Z;
                                                               F.Which (At_Pixel) := Interfaces.Integer_32 (P);
                                                               F.Tri (At_Pixel) := Interfaces.Integer_32 (T);
                                                            end if;
                                                         end;
                                                      end if;
                                                   end;
                                                end loop;
                                             end loop;
                                          end;
                                       end if;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Render;

   --  The nearest surface at a pixel: the depth along the optical axis and the part (0 the table, 0 depth none).
   procedure Nearest (F : Frame; Column, Row : Integer; Z : out Real; Which : out Natural) is
   begin
      Z := 0.0;
      Which := 0;
      if Column < 0 or else Row < 0 or else Column >= F.Lens.Width or else Row >= F.Lens.Height then
         return;
      end if;
      declare
         At_Pixel : constant Natural := Row * F.Lens.Width + Column;
         Mesh_Z   : constant Real := F.Depth (At_Pixel);
         Table_Z  : constant Real := F.Table (At_Pixel);
      begin
         if Mesh_Z > 0.0 and then (Table_Z = 0.0 or else Mesh_Z <= Table_Z) then
            Z := Mesh_Z;
            Which := Natural (F.Which (At_Pixel));
         elsif Table_Z > 0.0 then
            Z := Table_Z;
            Which := 0;
         end if;
      end;
   end Nearest;

   ---------------------------------------------------------------------------
   --  Rays

   type Surface is record
      Found  : Boolean := False;
      Part   : Natural := 0;       --  0 the table, else the index in Parts
      Point  : Vec3 := Zero3;      --  in the camera's frame
      Normal : Vec3 := [0.0, 0.0, 1.0];
      Edge   : Boolean := False;
   end record;

   function Ray_Of (F : Frame; U, V : Real) return Vec3 is
     ([(U - F.Lens.Cx) / F.Lens.Fx, (V - F.Lens.Cy) / F.Lens.Fy, 1.0]);

   function Cast (F : Frame; L : Truth_Line; U, V : Real) return Surface is
      Column : constant Integer := Integer (Real'Floor (U));
      Row    : constant Integer := Integer (Real'Floor (V));
      Z      : Real;
      Which  : Natural;
      S      : Surface;
      Ray    : constant Vec3 := Ray_Of (F, U, V);
      To_Camera : constant Rigid := Inverse (F.Pose);
   begin
      Nearest (F, Column, Row, Z, Which);
      if Z = 0.0 then
         return S;
      end if;
      S.Found := True;
      S.Part := Which;
      --  An edge: a neighbour belongs to another part, or the surface is not smooth across the pixel.
      declare
         Zl, Zr, Zu, Zd : Real;
         Wl, Wr, Wu, Wd : Natural;
         Iz : constant Real := 1.0 / Z;
      begin
         Nearest (F, Column - 1, Row, Zl, Wl);
         Nearest (F, Column + 1, Row, Zr, Wr);
         Nearest (F, Column, Row - 1, Zu, Wu);
         Nearest (F, Column, Row + 1, Zd, Wd);
         if Zl = 0.0 or else Zr = 0.0 or else Zu = 0.0 or else Zd = 0.0
           or else Wl /= Which or else Wr /= Which or else Wu /= Which or else Wd /= Which
           or else abs (1.0 / Zl - 2.0 * Iz + 1.0 / Zr) > 0.02 * Iz
           or else abs (1.0 / Zu - 2.0 * Iz + 1.0 / Zd) > 0.02 * Iz
         then
            S.Edge := True;
         end if;
      end;
      if Which = 0 then
         --  The table: the plane z = Table_Height in the world.
         declare
            N : constant Vec3 := To_Camera.Rotation * [0.0, 0.0, 1.0];
            --  a point of the plane in the camera frame
            P0 : constant Vec3 := To_Camera * [0.0, 0.0, Table_Height];
            D  : constant Real := N * P0;
            Denominator : constant Real := N * Ray;
         begin
            S.Normal := N;
            S.Point := (if abs Denominator > 1.0E-12 then (D / Denominator) * Ray else Ray);
         end;
      else
         declare
            Pt   : constant Part := Parts (Which);
            M    : constant Mesh := Load (To_String (Pt.Key));
            Pose : constant Rigid := To_Camera * Pose_Of (L, Pt);
            T    : constant Natural := Natural (F.Tri (Row * F.Lens.Width + Column));
            function Vertex (K : Natural) return Vec3 is
              (Pose * [Real (M.Points (3 * Natural (M.Tris (3 * T + K)))),
                       Real (M.Points (3 * Natural (M.Tris (3 * T + K)) + 1)),
                       Real (M.Points (3 * Natural (M.Tris (3 * T + K)) + 2))]);
            V0 : constant Vec3 := Vertex (0);
            V1 : constant Vec3 := Vertex (1);
            V2 : constant Vec3 := Vertex (2);
            N  : constant Vec3 := Cross (V1 - V0, V2 - V0);
         begin
            if abs N > 0.0 then
               S.Normal := Unit (N);
               declare
                  D : constant Real := S.Normal * V0;
                  Denominator : constant Real := S.Normal * Ray;
               begin
                  S.Point := (if abs Denominator > 1.0E-12 then (D / Denominator) * Ray else Ray);
               end;
            else
               S.Found := False;
            end if;
         end;
      end if;
      return S;
   end Cast;

   ---------------------------------------------------------------------------
   --  The questions

   type Request is record
      Number               : Natural;
      A_Camera, A_Beat     : Natural;
      B_Camera, B_Beat     : Natural;
      U, V                 : Real_Array (1 .. 4000);
      Count                : Natural := 0;
   end record;

   function Read_Request (Number : Natural) return Request is
      F : Ada.Text_IO.File_Type;
      R : Request;
   begin
      R.Number := Number;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Ada.Command_Line.Argument (2) & "/req_" & Padded (Number) & ".txt");
      declare
         Head  : constant String := Ada.Text_IO.Get_Line (F);   --  call N beat B
         A     : constant String := Ada.Text_IO.Get_Line (F);   --  a camera C beat B
         B     : constant String := Ada.Text_IO.Get_Line (F);
         Sizes : constant String := Ada.Text_IO.Get_Line (F);
         pragma Unreferenced (Head, Sizes);
         function Camera_Of (S : String) return Natural is
            At_Camera : constant Natural := Ada.Strings.Fixed.Index (S, "camera");
            At_Beat   : constant Natural := Ada.Strings.Fixed.Index (S, " beat");
         begin
            return Natural'Value (S (At_Camera + 6 .. At_Beat - 1));
         end Camera_Of;
         function Beat_Of (S : String) return Natural is
            At_Beat : constant Natural := Ada.Strings.Fixed.Index (S, " beat");
         begin
            return Natural'Value (S (At_Beat + 5 .. S'Last));
         end Beat_Of;
      begin
         R.A_Camera := Camera_Of (A);
         R.A_Beat := Beat_Of (A);
         R.B_Camera := Camera_Of (B);
         R.B_Beat := Beat_Of (B);
      end;
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            Line  : constant String := Ada.Text_IO.Get_Line (F);
            Bar   : constant Natural := Ada.Strings.Fixed.Index (Line, " | ");
            Mine  : constant Real_Array :=
              Words_Of ((if Bar > 0 then Line (Line'First + 1 .. Bar - 1) else Line (Line'First + 1 .. Line'Last)));
         begin
            if Line'Length > 2 and then Line (Line'First) = 'p' and then Mine'Length >= 2 then
               R.Count := R.Count + 1;
               R.U (R.Count) := Mine (1);
               R.V (R.Count) := Mine (2);
            end if;
         end;
      end loop;
      Ada.Text_IO.Close (F);
      return R;
   end Read_Request;

   ---------------------------------------------------------------------------
   --  The frames made so far

   type Frame_Access is access Frame;
   procedure Free_Frame is new Ada.Unchecked_Deallocation (Frame, Frame_Access);
   package Frame_Vectors is new Ada.Containers.Vectors (Positive, Frame_Access);
   Made : Frame_Vectors.Vector;
   Lag  : Natural := 0;

   function Frame_Of (Camera, Beat : Natural) return Frame_Access is
      Line : constant Natural := (if Line_Of (Beat) > Lag then Line_Of (Beat) - Lag else 0);
   begin
      for I in Made.First_Index .. Made.Last_Index loop
         if Made (I).Camera = Camera and then Made (I).Beat = Beat then
            declare
               Found : constant Frame_Access := Made (I);
            begin
               --  the one used last is the last to be let go
               Made.Delete (I);
               Made.Append (Found);
               return Found;
            end;
         end if;
      end loop;
      if Line = 0 then
         return null;
      end if;
      declare
         F    : constant Frame_Access := new Frame;
         Name : constant String := Camera_Name (Camera);
      begin
         F.Camera := Camera;
         F.Beat := Beat;
         F.Name := To_Unbounded_String (Name);
         F.Line := Line;
         F.Lens := Lenses (Name);
         F.Pose := Truth (Line).Cameras (Name);
         --  The render's optical centre may lie off the camera frame the truth gives along its axis.
         F.Pose.Translation := F.Pose.Translation + Optical_Offset * (F.Pose.Rotation * [0.0, 0.0, 1.0]);
         Render (F.all, Truth (Line));
         if Natural (Made.Length) >= 6 then
            declare
               Old : Frame_Access := Made.First_Element;
            begin
               --  the oldest frame's buffers are let go
               Made.Delete_First;
               Free (Old.Depth);
               Free (Old.Table);
               Free (Old.Which);
               Free (Old.Tri);
               Free_Frame (Old);
            end;
         end if;
         Made.Append (F);
         return F;
      end;
   end Frame_Of;

   function Project (L : Lens; X : Vec3) return Real_Array is
     ([L.Fx * X (1) / X (3) + L.Cx, L.Fy * X (2) / X (3) + L.Cy]);

   procedure Answer (Number : Natural) is
      Q : constant Request := Read_Request (Number);
      Fa : constant Frame_Access := Frame_Of (Q.A_Camera, Q.A_Beat);
      Fb : constant Frame_Access := Frame_Of (Q.B_Camera, Q.B_Beat);
      Out_File : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create
        (Out_File, Ada.Text_IO.Out_File,
         Ada.Command_Line.Argument (2) & "/req_" & Padded (Number) & ".truth" & Img (Lag)
         & (if Ada.Command_Line.Argument_Count >= 5 then Ada.Command_Line.Argument (5) else ""));
      if Fa = null or else Fb = null then
         Ada.Text_IO.Put_Line (Out_File, "unpaired");
         Ada.Text_IO.Close (Out_File);
         return;
      end if;
      declare
         La : Truth_Line renames Truth (Fa.Line);
         Lb : Truth_Line renames Truth (Fb.Line);
         A_To_World : constant Rigid := Fa.Pose;
         World_To_B : constant Rigid := Inverse (Fb.Pose);

         --  What carries a point seen on a part at the first moment to where that part has it at the second, in
         --  the cameras' frames.
         function Motion (P : Natural) return Rigid is
         begin
            if P = 0 then
               return World_To_B * A_To_World;
            end if;
            declare
               Pt : constant Part := Parts (P);
            begin
               if Has_Pose (La, Pt) and then Has_Pose (Lb, Pt) then
                  return World_To_B * Pose_Of (Lb, Pt) * Inverse (Pose_Of (La, Pt)) * A_To_World;
               else
                  return World_To_B * A_To_World;
               end if;
            end;
         end Motion;

         Used : array (0 .. Natural (Parts.Length)) of Boolean := [others => False];
      begin
         Ada.Text_IO.Put_Line
           (Out_File, "request" & Natural'Image (Number) & " a" & Natural'Image (Q.A_Camera) & Natural'Image (Q.A_Beat)
            & " line" & Natural'Image (Fa.Line) & " b" & Natural'Image (Q.B_Camera) & Natural'Image (Q.B_Beat)
            & " line" & Natural'Image (Fb.Line));
         Ada.Text_IO.Put_Line (Out_File, "lens_a " & Num (Fa.Lens.Fx) & Num (Fa.Lens.Cx) & Num (Fa.Lens.Cy)
                               & " lens_b " & Num (Fb.Lens.Fx) & Num (Fb.Lens.Cx) & Num (Fb.Lens.Cy));
         for K in 1 .. Q.Count loop
            declare
               S      : constant Surface := Cast (Fa.all, La, Q.U (K), Q.V (K));
               Status : Character := 'U';
               Qx, Qy : Real := -1.0;
               Jxx, Jxy, Jyx, Jyy : Real := 0.0;
            begin
               if S.Found then
                  Used (S.Part) := True;
                  declare
                     M  : constant Rigid := Motion (S.Part);
                     Xb : constant Vec3 := M * S.Point;
                  begin
                     if Xb (3) <= 0.001 then
                        Status := 'X';
                     else
                        declare
                           P : constant Real_Array := Project (Fb.Lens, Xb);
                        begin
                           Qx := P (1);
                           Qy := P (2);
                           if Qx < 0.0 or else Qy < 0.0 or else Qx >= Real (Fb.Lens.Width)
                             or else Qy >= Real (Fb.Lens.Height)
                           then
                              Status := 'X';
                           else
                              --  Is that surface the one seen at that pixel of the second picture?
                              declare
                                 Zb : Real;
                                 Wb : Natural;
                                 Column : constant Integer := Integer (Real'Floor (Qx));
                                 Row    : constant Integer := Integer (Real'Floor (Qy));
                                 Spread : Real := 0.0;
                              begin
                                 Nearest (Fb.all, Column, Row, Zb, Wb);
                                 if Zb = 0.0 then
                                    Status := 'U';
                                 else
                                    for Dr in -1 .. 1 loop
                                       for Dc in -1 .. 1 loop
                                          declare
                                             Zn : Real;
                                             Wn : Natural;
                                          begin
                                             Nearest (Fb.all, Column + Dc, Row + Dr, Zn, Wn);
                                             if Zn = 0.0 then
                                                Spread := Real'Last;
                                             elsif Spread < Real'Last then
                                                Spread := Real'Max (Spread, abs (1.0 / Zn - 1.0 / Zb));
                                             end if;
                                          end;
                                       end loop;
                                    end loop;
                                    --  Inverse depths: the point's, and what the second picture shows at its pixel.
                                    declare
                                       Mine  : constant Real := 1.0 / Xb (3);
                                       Shown : constant Real := 1.0 / Zb;
                                       Slack : constant Real :=
                                         (if Spread < Real'Last then Spread else Shown) + 1.0E-4 * Shown;
                                    begin
                                       if Mine < Shown - Slack then
                                          Status := 'O';   --  farther than what the second picture shows there
                                       elsif S.Edge or else Spread >= Real'Last or else Spread > 0.02 * Shown
                                         or else Mine > Shown + Slack
                                       then
                                          Status := 'E';
                                       else
                                          Status := 'V';
                                       end if;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                        --  The local affine part of the warp, from the surface plane.
                        declare
                           D : constant Real := S.Normal * S.Point;
                           function Where (Du, Dv : Real) return Real_Array is
                              Ray : constant Vec3 := Ray_Of (Fa.all, Q.U (K) + Du, Q.V (K) + Dv);
                              Den : constant Real := S.Normal * Ray;
                              X   : constant Vec3 := (if abs Den > 1.0E-12 then (D / Den) * Ray else Ray);
                              Y   : constant Vec3 := M * X;
                           begin
                              return Project (Fb.Lens, Y);
                           end Where;
                           Pu_Plus  : constant Real_Array := Where (0.5, 0.0);
                           Pu_Minus : constant Real_Array := Where (-0.5, 0.0);
                           Pv_Plus  : constant Real_Array := Where (0.0, 0.5);
                           Pv_Minus : constant Real_Array := Where (0.0, -0.5);
                        begin
                           Jxx := Pu_Plus (1) - Pu_Minus (1);
                           Jyx := Pu_Plus (2) - Pu_Minus (2);
                           Jxy := Pv_Plus (1) - Pv_Minus (1);
                           Jyy := Pv_Plus (2) - Pv_Minus (2);
                        end;
                     end if;
                  end;
               end if;
               Ada.Text_IO.Put_Line
                 (Out_File,
                  "point" & Natural'Image (K) & " " & Status & Natural'Image (S.Part) & Num (Qx) & Num (Qy)
                  & " X" & Num (S.Point (1)) & Num (S.Point (2)) & Num (S.Point (3))
                  & " n" & Num (S.Normal (1)) & Num (S.Normal (2)) & Num (S.Normal (3))
                  & " J" & Num (Jxx) & Num (Jxy) & Num (Jyx) & Num (Jyy));
            end;
         end loop;
         for P in Used'Range loop
            if Used (P) then
               declare
                  M : constant Rigid := Motion (P);
               begin
                  Ada.Text_IO.Put (Out_File, "motion" & Natural'Image (P));
                  for A in 1 .. 3 loop
                     for B in 1 .. 3 loop
                        Ada.Text_IO.Put (Out_File, Num (M.Rotation (A, B)));
                     end loop;
                  end loop;
                  Ada.Text_IO.Put_Line
                    (Out_File, Num (M.Translation (1)) & Num (M.Translation (2)) & Num (M.Translation (3)));
               end;
            end if;
         end loop;
      end;
      Ada.Text_IO.Close (Out_File);
   end Answer;

   Number : Natural := 1;
begin
   if Ada.Command_Line.Argument_Count < 2 then
      Ada.Text_IO.Put_Line ("usage: truth_warp TRUTH RUN_DIR [LAG [TABLE_HEIGHT]]");
      return;
   end if;
   if Ada.Command_Line.Argument_Count >= 3 then
      Lag := Natural'Value (Ada.Command_Line.Argument (3));
   end if;
   if Ada.Command_Line.Argument_Count >= 4 then
      Table_Height := Real'Value (Ada.Command_Line.Argument (4));
   end if;
   if Ada.Command_Line.Argument_Count >= 6 then
      Optical_Offset := Real'Value (Ada.Command_Line.Argument (6));
   end if;
   Read_Truth (Ada.Command_Line.Argument (1));
   Read_Beats (Ada.Command_Line.Argument (2) & "/beats.txt");
   Pair_Beats;
   for Name in Link_Keys.Iterate loop
      Parts.Append (Part'(Name => To_Unbounded_String (Key_Maps.Key (Name)), Is_Link => True,
                     Key => To_Unbounded_String (Key_Maps.Element (Name))));
   end loop;
   for Name in Object_Keys.Iterate loop
      Parts.Append (Part'(Name => To_Unbounded_String (Key_Maps.Key (Name)), Is_Link => False,
                     Key => To_Unbounded_String (Key_Maps.Element (Name))));
   end loop;
   declare
      Most : Natural := 1;
   begin
      for P of Parts loop
         declare
            M : constant Mesh := Load (To_String (P.Key));
         begin
            Most := Natural'Max (Most, M.Verts);
         end;
      end loop;
      Scratch_X := new Plane_Array (0 .. Most - 1);
      Scratch_Y := new Plane_Array (0 .. Most - 1);
      Scratch_Z := new Plane_Array (0 .. Most - 1);
      Ada.Text_IO.Put_Line
        (Img (Natural (Parts.Length)) & " parts, the largest of" & Natural'Image (Most) & " vertices");
   end;
   while Ada.Directories.Exists (Ada.Command_Line.Argument (2) & "/req_" & Padded (Number) & ".txt") loop
      Answer (Number);
      Number := Number + 1;
   end loop;
   Ada.Text_IO.Put_Line (Img (Number - 1) & " questions answered");
end Truth_Warp;
