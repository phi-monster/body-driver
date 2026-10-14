--  fake_robot --connect HOST:PORT --seed N --out DIR [--width W --height H] [--beats N] [--stop FILE]
--             [--scramble 0|1] [--dump N] [--quirk NAME=VALUE ...]
--
--  A robot for the driver to boot that is not a robot: a body and a world
--  drawn at random from the seed (Fake_World), rendered (Fake_Render), and
--  spoken to the driver over the body protocol exactly as the lab's
--  simulator speaks it (docs/body-protocol.md): a connection, hello, reset,
--  then every beat one call carrying the observation and one asking for the
--  action, which moves the body. The driver's binary is not changed in any
--  way, and nothing of the body reaches it but readings and pictures.
--
--  The observation is scrambled as the simulator's scrambler scrambles it
--  (harness/robodojo_scramble): every key renamed, every map in a drawn
--  order, every group of readings in a drawn channel order with a drawn sign,
--  scale and zero per channel; the draw goes to DIR/scramble.json for the
--  scorer. Truth goes beside the run in the simulator's own format
--  (harness/robodojo_truth): DIR/truth.jsonl and DIR/geometry, so that
--  driver/tools/score reads it unchanged.
--
--  The run ends after --beats beats, or when --stop FILE exists (the harness
--  makes it when the driver's log says the body is measured). --dump N
--  writes every camera's picture every N beats to DIR/frames as BMP. --rate
--  gives the robot's beats a second (10 unless given; 0 runs as fast as the
--  driver answers): a robot has its own rate, and the driver's recomputes
--  cost it beats in proportion.
--  --quirk turns one of Fake_World's quirks on or off (image_lag, settle,
--  zero_noise, drift, droop, creep, give, soft_closer, bounce; delay=N beats;
--  settle_share=X; image_noise=X grey levels; reading_noise=X). --hands KIND,KIND
--  forces what each arm ends in (pair, hook, thumb, fingers; drawn unless given),
--  --eyeless N leaves arm N without an eye of its own, --eye-up N points arm N's
--  eye ahead at the wall, --table plain gives a table with no texture.

with Ada.Calendar;
with Ada.Characters.Handling;
with Ada.Command_Line;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Vectors;
with Ada.Directories;
with Ada.Numerics;
with Ada.Numerics.Float_Random;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces;
with Driver;
with Driver.Bytes;
with Driver.Json;
with Driver.Msgpack;
with Driver.Numerics;
with Driver.Wire;
with Fake_Render;
with Fake_World;

procedure Fake_Robot is

   use Ada.Strings.Unbounded;
   use Ada.Text_IO;
   use Driver;
   use Driver.Numerics;
   use Fake_World;
   use type Interfaces.Unsigned_64;

   package String_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   package String_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String);
   package Random renames Ada.Numerics.Float_Random;

   function Trim (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));
   function Img (X : Real) return String is (Driver.Json.Number_Image (X));
   function Quote (S : String) return String is (Driver.Json.Quote (S));

   Host       : Unbounded_String;
   Port       : Natural := 0;
   Seed       : Natural := 0;
   Out_Dir    : Unbounded_String;
   Width      : Positive := 320;
   Height     : Positive := 240;
   Beat_Limit : Natural := 100_000;
   Stop_File  : Unbounded_String;
   Scrambled  : Boolean := True;
   Dump_Every : Natural := 0;
   Rate       : Real := 10.0;   --  beats a second; 0 runs as fast as the driver answers
   Shape      : Template;

   W : World;

   --  The scrambler's draw, in the shape the simulator's scrambler keeps it.
   type Group_Draw (N : Natural) is record
      Order : Natural_Array (1 .. N);   --  channel i of the drawn group is channel Order (i) of the real one, from 0
      Sign  : Real_Array (1 .. N);
      Scale : Real_Array (1 .. N);
      Zero  : Real_Array (1 .. N);
   end record;

   package Draw_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Group_Draw);

   Names         : String_Maps.Map;   --  real key -> drawn key
   Back          : String_Maps.Map;   --  drawn key -> real key
   Draws         : Draw_Maps.Map;     --  real last key of a group -> its draw
   Orders        : String_Maps.Map;   --  a map's path -> its real keys in the drawn order, joined by LF
   Scramble_Rng  : Random.Generator;
   Scramble_Seed : Natural := 0;

   Truth : File_Type;
   Calls : Natural := 0;

   type Byte_Access is access Driver.Bytes.Byte_Array;
   type Real_Access is access Real_Array;
   Pixels  : array (1 .. 16) of Byte_Access;   --  one buffer per camera, the picture sent
   Scratch : array (1 .. 16) of Real_Access;   --  one per camera, the picture rendered

   Link : Driver.Wire.Connection;

   Render_Time, Step_Time, Wire_Time : Duration := 0.0;

   function Option (Name : String; Default : String := "") return String;
   function Drawn (Real_Key : String) return String;
   procedure Draw_Group (Real_Key : String; N : Natural; Integers : Boolean);
   function Scrambled_Values (Real_Key : String; X : Real_Array; Integers : Boolean) return Real_Array;
   function Unscrambled_Values (Real_Key : String; Y : Real_Array) return Real_Array;
   function Key_Order (Path : String; Keys : String_Vectors.Vector) return String_Vectors.Vector;
   procedure Write_Scramble_Map;
   function Pose_Json (P : Rigid) return String;
   function Key_Of (Name : String) return String;
   procedure Write_Box_Geometry (Store : String; Key : String; S : Solid);
   procedure Write_Geometry_Line;
   procedure Write_Truth_Line;
   procedure Put_Group
     (B : in out Driver.Bytes.Buffer; Real_Key : String; Values : Real_Array; Scalar : Boolean; Integers : Boolean);
   procedure Put_Keys (B : in out Driver.Bytes.Buffer; Path : String; Keys : String_Vectors.Vector;
                       Put_Value : not null access procedure (Real_Key : String));
   procedure Encode_Observation (B : in out Driver.Bytes.Buffer);
   procedure Exchange (Message : Driver.Bytes.Buffer; Reply : in out Driver.Bytes.Buffer);
   procedure Send_Plain (Message_Type : String);
   procedure Send_Call (Func_Name : String; With_Observation : Boolean; Reply : in out Driver.Bytes.Buffer);
   procedure Take_Action (Reply : Driver.Bytes.Buffer);
   function Noise_Gauss return Real;
   procedure Take_Pictures;
   procedure Dump_Pictures;
   procedure Apply_Quirk (Setting : String);

   ---------------------------------------------------------------------------

   function Option (Name : String; Default : String := "") return String is
   begin
      for I in 1 .. Ada.Command_Line.Argument_Count - 1 loop
         if Ada.Command_Line.Argument (I) = Name then
            return Ada.Command_Line.Argument (I + 1);
         end if;
      end loop;
      return Default;
   end Option;

   ---------------------------------------------------------------------------
   --  The scrambler

   function Drawn (Real_Key : String) return String is
      Hex : constant String := "0123456789abcdef";
   begin
      if not Scrambled then
         return Real_Key;
      end if;
      if not Names.Contains (Real_Key) then
         loop
            declare
               S : String (1 .. 9) := "k00000000";
            begin
               for I in 2 .. 9 loop
                  S (I) := Hex (1 + Natural (Float'Floor (16.0 * Random.Random (Scramble_Rng))) mod 16);
               end loop;
               if not Back.Contains (S) and then not Names.Contains (S) then
                  Names.Insert (Real_Key, S);
                  Back.Insert (S, Real_Key);
                  exit;
               end if;
            end;
         end loop;
      end if;
      return Names (Real_Key);
   end Drawn;

   --  Integers says the group's values are whole numbers (a servo bus's ticks) and must stay whole once
   --  scrambled: the scale is then a whole power of two and the zero a whole number.
   procedure Draw_Group (Real_Key : String; N : Natural; Integers : Boolean) is
      D : Group_Draw (N);
   begin
      if Draws.Contains (Real_Key) then
         return;
      end if;
      for I in 1 .. N loop
         D.Order (I) := I - 1;
      end loop;
      if Scrambled then
         for I in reverse 2 .. N loop
            declare
               J : constant Positive := 1 + Natural (Float'Floor (Float (I) * Random.Random (Scramble_Rng))) mod I;
               T : constant Natural := D.Order (I);
            begin
               D.Order (I) := D.Order (J);
               D.Order (J) := T;
            end;
         end loop;
      end if;
      for I in 1 .. N loop
         if Scrambled and then Integers then
            D.Sign (I) := (if Random.Random (Scramble_Rng) < 0.5 then -1.0 else 1.0);
            D.Scale (I) := 2.0 ** (Integer (Float'Floor (3.0 * Random.Random (Scramble_Rng))) mod 3);
            D.Zero (I) := Real'Floor (Real (Random.Random (Scramble_Rng)) * 4096.0 - 2048.0);
         elsif Scrambled then
            D.Sign (I) := (if Random.Random (Scramble_Rng) < 0.5 then -1.0 else 1.0);
            D.Scale (I) := 2.0 ** (Integer (Float'Floor (7.0 * Random.Random (Scramble_Rng))) mod 7 - 3);
            D.Zero (I) := Real (Random.Random (Scramble_Rng)) * 2.0 * Ada.Numerics.Pi - Ada.Numerics.Pi;
         else
            D.Sign (I) := 1.0;
            D.Scale (I) := 1.0;
            D.Zero (I) := 0.0;
         end if;
      end loop;
      Draws.Insert (Real_Key, D);
   end Draw_Group;

   function Scrambled_Values (Real_Key : String; X : Real_Array; Integers : Boolean) return Real_Array is
      Y : Real_Array (1 .. X'Length);
   begin
      Draw_Group (Real_Key, X'Length, Integers);
      declare
         D : constant Group_Draw := Draws (Real_Key);
      begin
         for I in Y'Range loop
            Y (I) := D.Sign (I) * D.Scale (I) * X (X'First + D.Order (I)) + D.Zero (I);
         end loop;
      end;
      return Y;
   end Scrambled_Values;

   function Unscrambled_Values (Real_Key : String; Y : Real_Array) return Real_Array is
      D : constant Group_Draw := Draws (Real_Key);
      X : Real_Array (1 .. Y'Length);
   begin
      for I in X'Range loop
         X (1 + D.Order (I)) := (Y (Y'First + I - 1) - D.Zero (I)) / (D.Sign (I) * D.Scale (I));
      end loop;
      return X;
   end Unscrambled_Values;

   function Key_Order (Path : String; Keys : String_Vectors.Vector) return String_Vectors.Vector is
      Result : String_Vectors.Vector;
   begin
      if not Orders.Contains (Path) then
         declare
            Idx    : array (1 .. Natural (Keys.Length)) of Positive;
            Joined : Unbounded_String;
         begin
            for I in Idx'Range loop
               Idx (I) := I;
            end loop;
            if Scrambled then
               for I in reverse 2 .. Idx'Last loop
                  declare
                     Pick : constant Float := Random.Random (Scramble_Rng);
                     J    : constant Positive := 1 + Natural (Float'Floor (Float (I) * Pick)) mod I;
                     T    : constant Positive := Idx (I);
                  begin
                     Idx (I) := Idx (J);
                     Idx (J) := T;
                  end;
               end loop;
            end if;
            for I in Idx'Range loop
               Append (Joined, Keys (Idx (I)) & ASCII.LF);
            end loop;
            Orders.Insert (Path, To_String (Joined));
         end;
      end if;
      declare
         Text  : constant String := Orders (Path);
         Start : Positive := Text'First;
      begin
         for I in Text'Range loop
            if Text (I) = ASCII.LF then
               Result.Append (Text (Start .. I - 1));
               Start := I + 1;
            end if;
         end loop;
      end;
      return Result;
   end Key_Order;

   procedure Write_Scramble_Map is
      F     : File_Type;
      First : Boolean;
      procedure List (Name : String; Values : Real_Array; Integers : Boolean);
      procedure List (Name : String; Values : Real_Array; Integers : Boolean) is
      begin
         Put (F, """" & Name & """: [");
         for I in Values'Range loop
            Put (F, (if I = Values'First then "" else ", ")
                 & (if Integers then Trim (Integer (Values (I))) else Img (Values (I))));
         end loop;
         Put (F, "]");
      end List;
   begin
      Create (F, Out_File, To_String (Out_Dir) & "/scramble.json");
      Put (F, "{""seed"": " & Trim (Scramble_Seed) & ", ""names"": {");
      First := True;
      for C in Names.Iterate loop
         Put (F, (if First then "" else ", ") & Quote (String_Maps.Key (C)) & ": " & Quote (String_Maps.Element (C)));
         First := False;
      end loop;
      Put (F, "}, ""groups"": {");
      First := True;
      for C in Draws.Iterate loop
         declare
            D     : constant Group_Draw := Draw_Maps.Element (C);
            Order : Real_Array (1 .. D.N);
         begin
            for I in Order'Range loop
               Order (I) := Real (D.Order (I));
            end loop;
            Put (F, (if First then "" else ", ") & Quote (Draw_Maps.Key (C)) & ": {");
            List ("order", Order, True);
            Put (F, ", ");
            List ("sign", D.Sign, False);
            Put (F, ", ");
            List ("scale", D.Scale, False);
            Put (F, ", ");
            List ("zero", D.Zero, False);
            Put (F, "}");
            First := False;
         end;
      end loop;
      Put_Line (F, "}}");
      Close (F);
   end Write_Scramble_Map;

   ---------------------------------------------------------------------------
   --  Truth, in the simulator's side-file format

   function Pose_Json (P : Rigid) return String is
      Q : constant Quaternion := To_Quaternion (P.Rotation);
   begin
      return "[" & Img (P.Translation (1)) & "," & Img (P.Translation (2)) & "," & Img (P.Translation (3)) & ","
        & Img (Q.W) & "," & Img (Q.X) & "," & Img (Q.Y) & "," & Img (Q.Z) & "]";
   end Pose_Json;

   function Key_Of (Name : String) return String is
      S : String := Name;
   begin
      for Ch of S loop
         if not Ada.Characters.Handling.Is_Alphanumeric (Ch) then
            Ch := '_';
         end if;
      end loop;
      return "box_" & S;
   end Key_Of;

   --  A box mesh (eight corners, twelve triangles) in the frame of the link it
   --  belongs to, written to the geometry store as the simulator's hook
   --  writes meshes: raw little-endian float32 points, int32 counts and indices.
   procedure Write_Box_Geometry (Store : String; Key : String; S : Solid) is
      subtype Four is Driver.Bytes.Byte_Array (1 .. 4);
      function To_Bytes is new Ada.Unchecked_Conversion (Interfaces.IEEE_Float_32, Four);
      function To_Bytes is new Ada.Unchecked_Conversion (Interfaces.Integer_32, Four);
      F : Ada.Streams.Stream_IO.File_Type;
      Corner_Index : constant array (1 .. 36) of Natural :=
        [0, 1, 3, 0, 3, 2,   4, 6, 7, 4, 7, 5,   0, 4, 5, 0, 5, 1,
         2, 3, 7, 2, 7, 6,   0, 2, 6, 0, 6, 4,   1, 5, 7, 1, 7, 3];
   begin
      if Ada.Directories.Exists (Store & "/" & Key & ".json") then
         return;
      end if;
      Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Store & "/" & Key & ".0.points");
      for X in 0 .. 1 loop
         for Y in 0 .. 1 loop
            for Z in 0 .. 1 loop
               declare
                  P : constant Vec3 := S.Local * Vec3'[(if X = 0 then -S.Half (1) else S.Half (1)),
                                                       (if Y = 0 then -S.Half (2) else S.Half (2)),
                                                       (if Z = 0 then -S.Half (3) else S.Half (3))];
               begin
                  for I in 1 .. 3 loop
                     Ada.Streams.Stream_IO.Write (F, To_Bytes (Interfaces.IEEE_Float_32 (P (I))));
                  end loop;
               end;
            end loop;
         end loop;
      end loop;
      Ada.Streams.Stream_IO.Close (F);
      Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Store & "/" & Key & ".0.counts");
      for I in 1 .. 12 loop
         Ada.Streams.Stream_IO.Write (F, To_Bytes (Interfaces.Integer_32'(3)));
      end loop;
      Ada.Streams.Stream_IO.Close (F);
      Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Store & "/" & Key & ".0.indices");
      for I of Corner_Index loop
         Ada.Streams.Stream_IO.Write (F, To_Bytes (Interfaces.Integer_32 (I)));
      end loop;
      Ada.Streams.Stream_IO.Close (F);
      declare
         J : File_Type;
      begin
         Create (J, Out_File, Store & "/" & Key & ".json");
         Put_Line (J, "{""meshes"":[{""collision"":true,""visual"":true,""vertices"":8,""faces"":12,""points"":"""
                   & Key & ".0.points"",""counts"":""" & Key & ".0.counts"",""indices"":""" & Key
                   & ".0.indices""}],""shapes"":[]}");
         Close (J);
      end;
   end Write_Box_Geometry;

   procedure Write_Geometry_Line is
      Store : constant String := To_String (Out_Dir) & "/geometry";
      First : Boolean := True;
   begin
      Ada.Directories.Create_Path (Store);
      Put (Truth, "{""kind"":""geometry"",""store"":" & Quote (Store) & ",""links"":{");
      for S of W.Solids loop
         if S.Link > 0 then
            Write_Box_Geometry (Store, Key_Of (To_String (S.Name)), S);
            Put (Truth, (if First then "" else ",") & Quote (To_String (W.Links (S.Link).Name)) & ":"
                 & Quote (Key_Of (To_String (S.Name))));
            First := False;
         end if;
      end loop;
      Put (Truth, "},""joints"":[");
      First := True;
      for L of W.Links loop
         if L.Parent > 0 then
            Put (Truth, (if First then "" else ",") & "{""parent"":" & Quote (To_String (W.Links (L.Parent).Name))
                 & ",""child"":" & Quote (To_String (L.Name)) & ",""type"":"""
                 & (case L.Kind is
                       when Fixed => "PhysicsFixedJoint",
                       when Revolute => "PhysicsRevoluteJoint",
                       when Prismatic => "PhysicsPrismaticJoint") & """}");
            First := False;
         end if;
      end loop;
      Put (Truth, "],""objects"":{");
      First := True;
      for S of W.Solids loop
         if S.Link = 0 and then S.Kind = Box then
            Write_Box_Geometry (Store, Key_Of (To_String (S.Name)), (S with delta Local => Identity));
            Put (Truth, (if First then "" else ",") & Quote (To_String (S.Name)) & ":"
                 & Quote (Key_Of (To_String (S.Name))));
            First := False;
         end if;
      end loop;
      Put_Line (Truth, "}}");
   end Write_Geometry_Line;

   procedure Write_Truth_Line is
      First : Boolean := True;
   begin
      Calls := Calls + 1;
      Put (Truth, "{""kind"":""observation"",""call"":" & Trim (Calls) & ",""state"":{");
      for G in W.Groups.First_Index .. W.Groups.Last_Index loop
         if not W.Groups (G).Joints.Is_Empty then
            declare
               R : constant Real_Array := Readings (W, G);
            begin
               Put (Truth, (if First then "" else ",") & Quote (To_String (W.Groups (G).Name)) & ":[");
               for I in R'Range loop
                  Put (Truth, (if I = R'First then "" else ",") & Img (R (I)));
               end loop;
               Put (Truth, "]");
               First := False;
            end;
         end if;
      end loop;
      Put (Truth, "},""links"":{");
      First := True;
      for L of W.Links loop
         Put (Truth, (if First then "" else ",") & Quote (To_String (L.Name)) & ":" & Pose_Json (L.Pose));
         First := False;
      end loop;
      Put (Truth, "},""cameras"":{");
      First := True;
      for C of W.Cameras loop
         Put (Truth, (if First then "" else ",") & Quote (To_String (C.Name)) & ":{""pose"":" & Pose_Json (C.Pose)
              & ",""resolution"":[" & Trim (C.Width) & "," & Trim (C.Height) & "],""K"":[[" & Img (C.Fx) & ",0.0,"
              & Img (C.Cx) & "],[0.0," & Img (C.Fy) & "," & Img (C.Cy) & "],[0.0,0.0,1.0]],""lens"":null}");
         First := False;
      end loop;
      Put (Truth, "},""objects"":{");
      First := True;
      for S of W.Solids loop
         if S.Link = 0 and then S.Kind = Box then
            Put (Truth, (if First then "" else ",") & Quote (To_String (S.Name))
                 & ":{""label"":null,""type"":""rigid"",""pose"":" & Pose_Json (S.Pose) & "}");
            First := False;
         end if;
      end loop;
      Put_Line (Truth, "}}");
   end Write_Truth_Line;

   ---------------------------------------------------------------------------
   --  The observation, scrambled, as msgpack

   --  Integers: the values go as msgpack integers, as a servo bus's bridge sends its ticks (an integer reading
   --  resolves to one tick in the driver, a float to its last bit).
   procedure Put_Group
     (B : in out Driver.Bytes.Buffer; Real_Key : String; Values : Real_Array; Scalar : Boolean; Integers : Boolean)
   is
      Y : constant Real_Array := Scrambled_Values (Real_Key, Values, Integers);
   begin
      if Scalar and then Y'Length = 1 then
         Driver.Msgpack.Put_Float (B, Y (Y'First));
      else
         Driver.Msgpack.Put_Array_Header (B, Y'Length);
         for V of Y loop
            if Integers then
               Driver.Msgpack.Put_Integer (B, Long_Long_Integer (Real'Rounding (V)));
            else
               Driver.Msgpack.Put_Float (B, V);
            end if;
         end loop;
      end if;
   end Put_Group;

   procedure Put_Keys (B : in out Driver.Bytes.Buffer; Path : String; Keys : String_Vectors.Vector;
                       Put_Value : not null access procedure (Real_Key : String)) is
      Ordered : constant String_Vectors.Vector := Key_Order (Path, Keys);
   begin
      Driver.Msgpack.Put_Map_Header (B, Natural (Ordered.Length));
      for K of Ordered loop
         Driver.Msgpack.Put_String (B, (if Path = "" and then K = "instruction" then K else Drawn (K)));
         Put_Value (K);
      end loop;
   end Put_Keys;

   procedure Encode_Observation (B : in out Driver.Bytes.Buffer) is
      Top : String_Vectors.Vector;

      procedure Picture (C : Positive);
      procedure Picture (C : Positive) is
      begin
         Driver.Msgpack.Put_Map_Header (B, 4);
         Driver.Msgpack.Put_String (B, "nd");
         Driver.Msgpack.Put_Boolean (B, True);
         Driver.Msgpack.Put_String (B, "type");
         Driver.Msgpack.Put_String (B, "|u1");
         Driver.Msgpack.Put_String (B, "shape");
         Driver.Msgpack.Put_Array_Header (B, 3);
         Driver.Msgpack.Put_Integer (B, Long_Long_Integer (W.Cameras (C).Height));
         Driver.Msgpack.Put_Integer (B, Long_Long_Integer (W.Cameras (C).Width));
         Driver.Msgpack.Put_Integer (B, 3);
         Driver.Msgpack.Put_String (B, "data");
         Driver.Msgpack.Put_Binary (B, Pixels (C).all);
      end Picture;

      procedure Cam_Value (Cam : String);
      procedure Cam_Value (Cam : String) is
         C     : Positive := 1;
         Parts : String_Vectors.Vector;
         procedure Part_Value (Part : String);
         procedure Part_Value (Part : String) is
         begin
            if Part = "color" then
               Picture (C);
            else
               Put_Group (B, "shape", [Real (W.Cameras (C).Height), Real (W.Cameras (C).Width), 3.0], False, False);
            end if;
         end Part_Value;
      begin
         for I in W.Cameras.First_Index .. W.Cameras.Last_Index loop
            if To_String (W.Cameras (I).Name) = Cam then
               C := I;
            end if;
         end loop;
         Parts.Append ("color");
         Parts.Append ("shape");
         Put_Keys (B, "vision/" & Cam, Parts, Part_Value'Access);
      end Cam_Value;

      procedure Group_Value (Name : String);
      procedure Group_Value (Name : String) is
      begin
         for G in W.Groups.First_Index .. W.Groups.Last_Index loop
            if To_String (W.Groups (G).Name) = Name then
               Put_Group (B, Name, Readings (W, G), False, W.Groups (G).Integers);
            end if;
         end loop;
      end Group_Value;

      procedure Echo_Value (Name : String);
      procedure Echo_Value (Name : String) is
      begin
         for G in W.Groups.First_Index .. W.Groups.Last_Index loop
            if To_String (W.Groups (G).Name) = Name then
               Put_Group (B, Name, Echo (W, G), False, W.Groups (G).Integers);
            end if;
         end loop;
      end Echo_Value;

      procedure Const_Value (Name : String);
      procedure Const_Value (Name : String) is
      begin
         for G in W.Groups.First_Index .. W.Groups.Last_Index loop
            if To_String (W.Groups (G).Name) = Name then
               Put_Group (B, Name, Readings (W, G), W.Groups (G).Scalar, False);
            end if;
         end loop;
      end Const_Value;

      procedure Top_Value (Key : String);
      procedure Top_Value (Key : String) is
         Keys : String_Vectors.Vector;
      begin
         if Key = "vision" then
            for Cm of W.Cameras loop
               Keys.Append (To_String (Cm.Name));
            end loop;
            Put_Keys (B, "vision", Keys, Cam_Value'Access);
         elsif Key = "state" or else Key = "action" then
            for G of W.Groups loop
               if not G.Joints.Is_Empty and then (Key = "state" or else G.Commandable) then
                  Keys.Append (To_String (G.Name));
               end if;
            end loop;
            Put_Keys (B, Key, Keys, (if Key = "state" then Group_Value'Access else Echo_Value'Access));
         elsif Key = "additional_info" then
            for G of W.Groups loop
               if G.Joints.Is_Empty then
                  Keys.Append (To_String (G.Name));
               end if;
            end loop;
            Put_Keys (B, "additional_info", Keys, Const_Value'Access);
         elsif Key = "instruction" then
            Driver.Msgpack.Put_String (B, "Pick up the thing by 10 cm.");
         else
            Driver.Msgpack.Put_String (B, "v1.0");
         end if;
      end Top_Value;
   begin
      Top.Append ("vision");
      Top.Append ("state");
      Top.Append ("action");
      Top.Append ("data_format_version");
      Top.Append ("additional_info");
      Top.Append ("instruction");
      Put_Keys (B, "", Top, Top_Value'Access);
   end Encode_Observation;

   ---------------------------------------------------------------------------
   --  The wire

   procedure Exchange (Message : Driver.Bytes.Buffer; Reply : in out Driver.Bytes.Buffer) is
      Ok   : Boolean;
      Kind : Driver.Wire.Message_Kind;
      use type Driver.Wire.Message_Kind;
   begin
      Driver.Wire.Send (Link, Message.To_Array, Ok);
      if Ok then
         Driver.Wire.Receive (Link, Kind, Reply);
         Ok := Kind /= Driver.Wire.Closed;
      end if;
      if not Ok then
         Put_Line ("fake_robot: the driver went away");
         Close (Truth);
         Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
         raise Program_Error with "the driver closed the connection";
      end if;
   end Exchange;

   procedure Send_Plain (Message_Type : String) is
      M, R : Driver.Bytes.Buffer;
   begin
      Driver.Msgpack.Put_Map_Header (M, 2);
      Driver.Msgpack.Put_String (M, "message_type");
      Driver.Msgpack.Put_String (M, Message_Type);
      Driver.Msgpack.Put_String (M, "payload");
      Driver.Msgpack.Put_Map_Header (M, 0);
      Exchange (M, R);
   end Send_Plain;

   procedure Send_Call (Func_Name : String; With_Observation : Boolean; Reply : in out Driver.Bytes.Buffer) is
      M : Driver.Bytes.Buffer;
   begin
      Driver.Msgpack.Put_Map_Header (M, 3);
      Driver.Msgpack.Put_String (M, "message_type");
      Driver.Msgpack.Put_String (M, "call");
      Driver.Msgpack.Put_String (M, "step");
      Driver.Msgpack.Put_Integer (M, Long_Long_Integer (W.Beat));
      Driver.Msgpack.Put_String (M, "payload");
      Driver.Msgpack.Put_Map_Header (M, (if With_Observation then 2 else 1));
      Driver.Msgpack.Put_String (M, "func_name");
      Driver.Msgpack.Put_String (M, Func_Name);
      if With_Observation then
         Driver.Msgpack.Put_String (M, "obs");
         Encode_Observation (M);
      end if;
      Exchange (M, Reply);
   end Send_Call;

   --  The driver's action: every drawn key with its values, back through the draw.
   procedure Take_Action (Reply : Driver.Bytes.Buffer) is
      use Driver.Msgpack;
      Doc : Document;
      Ok  : Boolean;
   begin
      Decode (Reply.To_Array, Doc, Ok);
      if not Ok then
         return;
      end if;
      declare
         Result : constant Node := Lookup (Doc, Lookup (Doc, Root (Doc), "payload"), "result");
         Action : constant Node :=
           (if Kind_Of (Doc, Result) = Array_Value and then Driver.Msgpack.Count (Doc, Result) > 0
            then Element (Doc, Result, 1) else No_Node);
      begin
         if Action = No_Node or else Kind_Of (Doc, Action) /= Map_Value then
            return;
         end if;
         for I in 1 .. Driver.Msgpack.Count (Doc, Action) loop
            declare
               Drawn_Key : constant String := Text (Doc, Key (Doc, Action, I));
               Real_Key  : constant String :=
                 (if not Scrambled then Drawn_Key elsif Back.Contains (Drawn_Key) then Back (Drawn_Key) else "");
               V         : constant Node := Value (Doc, Action, I);
            begin
               if Real_Key'Length > 0 and then Is_Numeric (Doc, V) then
                  for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                     if To_String (W.Groups (G).Name) = Real_Key and then W.Groups (G).Commandable then
                        Command (W, G, Unscrambled_Values (Real_Key, Numbers (Doc, V)));
                     end if;
                  end loop;
               end if;
            end;
         end loop;
      end;
   end Take_Action;

   ---------------------------------------------------------------------------
   --  Pictures

   --  The sensor's noise: a cheap generator of its own (xorshift, four uniforms summed), since it is drawn for
   --  every value of every picture.
   Noise_State : Interfaces.Unsigned_64 := 88172645463325252;

   function Noise_Gauss return Real is
      use Interfaces;
      S : Real := 0.0;
   begin
      for I in 1 .. 4 loop
         Noise_State := Noise_State xor Shift_Left (Noise_State, 13);
         Noise_State := Noise_State xor Shift_Right (Noise_State, 7);
         Noise_State := Noise_State xor Shift_Left (Noise_State, 17);
         S := S + Real (Noise_State mod 2 ** 24) / Real (2 ** 24);
      end loop;
      return (S - 2.0) * 1.7320508075688772;
   end Noise_Gauss;

   procedure Take_Pictures is
   begin
      for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
         declare
            Cam   : Camera renames W.Cameras (C);
            Share : constant Real := (if W.Quirk.Settle and then not Cam.Fresh then W.Quirk.Settle_Share else 1.0);
         begin
            Fake_Render.Render (W, Cam, Scratch (C).all);
            for I in Scratch (C)'Range loop
               Cam.Shown (I) := (1.0 - Share) * Cam.Shown (I) + Share * Scratch (C) (I);
               declare
                  Noise : constant Real := (if W.Quirk.Image_Noise > 0.0 then W.Quirk.Image_Noise * Noise_Gauss else 0.0);
                  Seen  : constant Real := Cam.Shown (I) + Noise;
               begin
                  Pixels (C) (Driver.Bytes.Offset (I)) :=
                    Driver.Bytes.Byte (Integer (Real'Floor (Real'Max (0.0, Real'Min (255.0, Seen)) + 0.5)));
               end;
            end loop;
            Cam.Fresh := False;
         end;
      end loop;
   end Take_Pictures;

   --  A picture as a 24-bit BMP (rows bottom up, blue first, rows padded to four bytes), which every viewer reads.
   procedure Dump_Pictures is
      use type Driver.Bytes.Offset;
      use type Driver.Bytes.Byte_Array;
      function Little (V : Natural; Bytes : Positive) return Driver.Bytes.Byte_Array;
      function Little (V : Natural; Bytes : Positive) return Driver.Bytes.Byte_Array is
         R : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (Bytes));
         X : Natural := V;
      begin
         for I in R'Range loop
            R (I) := Driver.Bytes.Byte (X mod 256);
            X := X / 256;
         end loop;
         return R;
      end Little;
   begin
      Ada.Directories.Create_Path (To_String (Out_Dir) & "/frames");
      for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
         declare
            F     : Ada.Streams.Stream_IO.File_Type;
            Wd    : constant Positive := W.Cameras (C).Width;
            Ht    : constant Positive := W.Cameras (C).Height;
            Pad   : constant Natural := (4 - (3 * Wd) mod 4) mod 4;
            Size  : constant Natural := 54 + Ht * (3 * Wd + Pad);
            Name  : constant String := To_String (Out_Dir) & "/frames/beat" & Trim (W.Beat) & "_"
              & To_String (W.Cameras (C).Name) & ".bmp";
            Row   : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Wd + Pad)) := [others => 0];
         begin
            Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Name);
            Ada.Streams.Stream_IO.Write (F, Driver.Bytes.To_Bytes ("BM") & Little (Size, 4) & Little (0, 4));
            Ada.Streams.Stream_IO.Write (F, Little (54, 4) & Little (40, 4) & Little (Wd, 4) & Little (Ht, 4));
            Ada.Streams.Stream_IO.Write (F, Little (1, 2) & Little (24, 2) & Little (0, 4) & Little (Size - 54, 4));
            Ada.Streams.Stream_IO.Write (F, Little (2835, 4) & Little (2835, 4) & Little (0, 4) & Little (0, 4));
            for R in reverse 0 .. Ht - 1 loop
               for Col in 0 .. Wd - 1 loop
                  declare
                     K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (R * Wd + Col) + 1);
                  begin
                     Row (Driver.Bytes.Offset (3 * Col + 1)) := Pixels (C) (K + 2);
                     Row (Driver.Bytes.Offset (3 * Col + 2)) := Pixels (C) (K + 1);
                     Row (Driver.Bytes.Offset (3 * Col + 3)) := Pixels (C) (K);
                  end;
               end loop;
               Ada.Streams.Stream_IO.Write (F, Row);
            end loop;
            Ada.Streams.Stream_IO.Close (F);
         end;
      end loop;
   end Dump_Pictures;

   ---------------------------------------------------------------------------

   procedure Apply_Quirk (Setting : String) is
      Eq    : constant Natural := Ada.Strings.Fixed.Index (Setting, "=");
      Name  : constant String := (if Eq = 0 then Setting else Setting (Setting'First .. Eq - 1));
      Value : constant String := (if Eq = 0 then "1" else Setting (Eq + 1 .. Setting'Last));
      On    : constant Boolean := Value /= "0";
   begin
      if Name = "image_lag" then
         W.Quirk.Image_Lag := On;
      elsif Name = "settle" then
         W.Quirk.Settle := On;
      elsif Name = "settle_share" then
         W.Quirk.Settle_Share := Real'Value (Value);
      elsif Name = "image_noise" then
         W.Quirk.Image_Noise := Real'Value (Value);
      elsif Name = "zero_noise" then
         W.Quirk.Zero_Noise := On;
      elsif Name = "drift" then
         W.Quirk.Drift := On;
      elsif Name = "delay" then
         W.Quirk.Delay_Beats := Natural'Value (Value);
      elsif Name = "droop" then
         W.Quirk.Droop := On;
      elsif Name = "creep" then
         W.Quirk.Creep := On;
      elsif Name = "give" then
         W.Quirk.Give := On;
      elsif Name = "soft_closer" then
         W.Quirk.Soft_Closer := On;
      elsif Name = "bounce" then
         W.Quirk.Bounce := On;
      elsif Name = "reading_noise" then
         W.Quirk.Reading_Noise := Real'Value (Value);
      else
         Put_Line ("fake_robot: no quirk called " & Name);
         raise Program_Error;
      end if;
   end Apply_Quirk;

   Colon : Natural;
   Ok    : Boolean;

begin
   if Option ("--connect") = "" or else Option ("--out") = "" then
      Put_Line ("usage: fake_robot --connect HOST:PORT --seed N --out DIR [--width W --height H] [--beats N]"
                & " [--stop FILE] [--scramble 0|1] [--dump N] [--quirk NAME=VALUE ...]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Colon := Ada.Strings.Fixed.Index (Option ("--connect"), ":", Ada.Strings.Backward);
   Host := To_Unbounded_String (Option ("--connect") (Option ("--connect")'First .. Colon - 1));
   Port := Natural'Value (Option ("--connect") (Colon + 1 .. Option ("--connect")'Last));
   Seed := Natural'Value (Option ("--seed", "0"));
   Out_Dir := To_Unbounded_String (Option ("--out"));
   Width := Positive'Value (Option ("--width", "320"));
   Height := Positive'Value (Option ("--height", "240"));
   Beat_Limit := Natural'Value (Option ("--beats", "100000"));
   Stop_File := To_Unbounded_String (Option ("--stop"));
   Scrambled := Option ("--scramble", "1") /= "0";
   Dump_Every := Natural'Value (Option ("--dump", "0"));
   Rate := Real'Value (Option ("--rate", "10.0"));
   declare
      Hands : constant String := Option ("--hands");
      Comma : constant Natural := Ada.Strings.Fixed.Index (Hands, ",");
      function Kind_Of (Name : String) return Hand_Kind is
        (if Name = "pair" then Pair elsif Name = "hook" then Hook elsif Name = "thumb" then Pair_And_Thumb
         elsif Name = "fingers" then Fingers elsif Name = "" or else Name = "drawn" then Drawn
         else raise Program_Error with "no hand called " & Name);
   begin
      if Comma > 0 then
         Shape.Hands := [Kind_Of (Hands (Hands'First .. Comma - 1)), Kind_Of (Hands (Comma + 1 .. Hands'Last))];
      elsif Hands'Length > 0 then
         Shape.Hands := [others => Kind_Of (Hands)];
      end if;
      Shape.Eyeless := Natural'Value (Option ("--eyeless", "0"));
      Shape.Eye_Up := Natural'Value (Option ("--eye-up", "0"));
      Shape.Plain_Table := Option ("--table", "grain") = "plain";
   end;
   for I in 1 .. Ada.Command_Line.Argument_Count - 1 loop
      if Ada.Command_Line.Argument (I) = "--quirk" then
         Apply_Quirk (Ada.Command_Line.Argument (I + 1));
      end if;
   end loop;
   Ada.Directories.Create_Path (To_String (Out_Dir));
   Build (W, Seed, Width, Height, Shape);
   Scramble_Seed := Seed * 7919 + 17;
   Noise_State := Interfaces.Unsigned_64 (Seed) * 6364136223846793005 + 1442695040888963407;
   Random.Reset (Scramble_Rng, Integer (Scramble_Seed));
   for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
      Pixels (C) := new Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Width * Height));
      Scratch (C) := new Real_Array (1 .. 3 * Width * Height);
   end loop;
   declare
      F : File_Type;
   begin
      Create (F, Out_File, To_String (Out_Dir) & "/body.txt");
      Put (F, Describe (W));
      Close (F);
   end;
   Create (Truth, Out_File, To_String (Out_Dir) & "/truth.jsonl");
   Write_Geometry_Line;
   Put_Line ("fake_robot: seed" & Seed'Image & "," & W.Arms'Image & " arms,"
             & Natural'Image (Natural (W.Cameras.Length)) & " cameras of" & Width'Image & " x" & Height'Image
             & ", table at " & Img (W.Table_Z));
   for Try in 1 .. 120 loop
      Driver.Wire.Connect (Link, To_String (Host), Port, Ok);
      exit when Ok;
      delay 1.0;
   end loop;
   if not Ok then
      Put_Line ("fake_robot: no driver at " & To_String (Host) & ":" & Trim (Port));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Send_Plain ("hello");
   Send_Plain ("reset");
   declare
      use Ada.Calendar;
      Next : Time := Clock;
   begin
      loop
         declare
            T0     : constant Time := Clock;
         T1, T2 : Time;
         R      : Driver.Bytes.Buffer;
      begin
         --  The pictures the driver gets this beat show the body before this
         --  beat's motion (the renderer's lag), or after it without the quirk.
         if W.Quirk.Image_Lag then
            Take_Pictures;
            T1 := Clock;
            Step (W);
            T2 := Clock;
            Render_Time := Render_Time + (T1 - T0);
            Step_Time := Step_Time + (T2 - T1);
         else
            Step (W);
            T1 := Clock;
            Take_Pictures;
            T2 := Clock;
            Step_Time := Step_Time + (T1 - T0);
            Render_Time := Render_Time + (T2 - T1);
         end if;
         Write_Truth_Line;
         Send_Call ("update_obs", True, R);
         if W.Beat = 1 and then Scrambled then
            Write_Scramble_Map;
         end if;
         Send_Call ("get_action", False, R);
         Take_Action (R);
         Wire_Time := Wire_Time + (Clock - T2);
         if Dump_Every > 0 and then W.Beat mod Dump_Every = 1 mod Dump_Every then
            Dump_Pictures;
         end if;
         if W.Beat mod 200 = 0 then
            Put_Line ("fake_robot: beat" & W.Beat'Image & ", a beat takes render "
                      & Img (Real (Render_Time) * 1000.0 / Real (W.Beat)) & " ms, body "
                      & Img (Real (Step_Time) * 1000.0 / Real (W.Beat)) & " ms, driver "
                      & Img (Real (Wire_Time) * 1000.0 / Real (W.Beat)) & " ms");
         end if;
         exit when W.Beat >= Beat_Limit;
         exit when Length (Stop_File) > 0 and then Ada.Directories.Exists (To_String (Stop_File));
         --  A robot has its own rate; the driver is given the beats at it, not as fast as it can answer.
         if Rate > 0.0 then
            Next := Next + Duration (1.0 / Rate);
            if Next > Clock then
               delay until Next;
            else
               Next := Clock;
            end if;
         end if;
      end;
   end loop;
   end;
   Close (Truth);
   Driver.Wire.Disconnect (Link);
   Put_Line ("fake_robot: done after" & W.Beat'Image & " beats");
end Fake_Robot;
