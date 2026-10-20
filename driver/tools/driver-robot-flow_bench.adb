with Ada.Calendar;
with Ada.Command_Line;
with Ada.Directories;
with Ada.Environment_Variables;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Log;
with Driver.Numerics;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Flow_Eye;
with Fake_Render;
with Fake_World;

--  usage: driver-robot-flow_bench SEED [POSES [ARM]] The fake world of SEED (table arms), its arm ARM (1) swung to
--  POSES (6) poses about its rest (and the other arm too when FLOW_CACHE2 is set), at each the fixed eye's picture and,
--  for every joint, the pictures with the joint turned a little either way (each a mean of FLOW_FRAMES frames with the
--  camera's noise, FLOW_NOISE grey levels a channel); the arm's true joints as its fit would have them (the world's
--  frame and unit); the eye's lens and place fitted (Flow_Eye) and set against the truth.
procedure Driver.Robot.Flow_Bench is
   use Ada.Text_IO;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   package FW renames Fake_World;
   package Flow renames Driver.Robot.Kinematics.Flow_Eye;
   package Fit renames Driver.Robot.Kinematics.Fit;
   package EF renames Ada.Numerics.Long_Elementary_Functions;
   use type FW.Joint_Kind;
   use type Ada.Calendar.Time;

   function Setting (Name : String; Default : Real) return Real is
     (if Ada.Environment_Variables.Exists (Name) then Real'Value (Ada.Environment_Variables.Value (Name)) else Default);

   Noise_Level : constant Real := Setting ("FLOW_NOISE", 0.7);
   Frames      : constant Positive := Positive (Setting ("FLOW_FRAMES", 4.0));
   Turn        : constant Real := Setting ("FLOW_TURN", 0.004);   --  radians each way, every joint
   Swing       : constant Real := Setting ("FLOW_SWING", 0.35);   --  radians about rest, the poses' spread

   Rng : Ada.Numerics.Float_Random.Generator;
   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Rng)));
   function Gaussian return Real is
      U1 : constant Real := Real'Max (Uniform, 1.0E-12);
      U2 : constant Real := Uniform;
   begin
      return EF.Sqrt (-2.0 * EF.Log (U1)) * EF.Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Image (X : Real; Aft : Natural := 2) return String is (Driver.Log.Image (X, Aft));
   function Trim (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));

   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Flow.Luma_Access);
   type Rgb_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Rgb_Access);

   Seed   : constant Natural := Natural'Value (Ada.Command_Line.Argument (1));
   Poses  : constant Positive :=
     (if Ada.Command_Line.Argument_Count >= 2 then Positive'Value (Ada.Command_Line.Argument (2)) else 6);
   Arm    : constant Positive :=
     (if Ada.Command_Line.Argument_Count >= 3 then Positive'Value (Ada.Command_Line.Argument (3)) else 1);
   W      : FW.World;
   T      : FW.Template;
   Width  : constant Positive := 320;
   Height : constant Positive := 240;
   Eye    : Natural := 0;
   Began  : constant Ada.Calendar.Time := Ada.Calendar.Clock;

   --  The fixed eye's picture as the world stands now: a mean of Frames frames, each the render with the camera's noise
   --  on every channel and rounded to a level, in luma.
   function Picture return Flow.Luma_Access is
      Cam    : aliased constant FW.Camera := W.Cameras (Eye);
      Rgb    : Rgb_Access := new Real_Array (1 .. 3 * Width * Height);
      Result : constant Flow.Luma_Access := new Real_Array (1 .. Width * Height);
   begin
      Fake_Render.Render (W, Cam, Rgb.all);
      for K in Result'Range loop
         Result (K) := 0.0;
      end loop;
      for F in 1 .. Frames loop
         for K in 0 .. Width * Height - 1 loop
            declare
               function Level (X : Real) return Real is
                 (Real'Max (0.0, Real'Min (255.0, Real'Rounding (X + Noise_Level * Gaussian))));
               R : constant Real := Level (Rgb (Rgb'First + 3 * K));
               G : constant Real := Level (Rgb (Rgb'First + 3 * K + 1));
               B : constant Real := Level (Rgb (Rgb'First + 3 * K + 2));
            begin
               Result (Result'First + K) := Result (Result'First + K)
                 + (0.299 * R + 0.587 * G + 0.114 * B) / Real (Frames);
            end;
         end loop;
      end loop;
      Free (Rgb);
      return Result;
   end Picture;
begin
   --  A camera's pixel integrates the light over its area: an edge that moves a tenth of a pixel changes the pixel by a
   --  tenth of its contrast. One ray through the pixel's centre (the renderer's default) changes it by all of it or by
   --  nothing, and the flow is a step, not a gradient times a motion (at the truth the model left residuals 35 to 76
   --  times their sigmas).
   Fake_Render.Supersample := Positive (Setting ("FLOW_SUPERSAMPLE", 4.0));
   Fake_Render.Even_Light := Ada.Environment_Variables.Exists ("FLOW_FLAT");
   Fake_Render.Lens_Blur := Setting ("FLOW_BLUR", 0.0);
   Ada.Numerics.Float_Random.Reset (Rng, Seed + 1);
   FW.Build (W, Seed, Width, Height, T);
   --  FLOW_PLAIN: the world's own solids (the table, the room, the things on it) without their texture, each its one
   --  albedo: what the arm moves against shows no texture of its own (a study of the arm's outline).
   if Ada.Environment_Variables.Exists ("FLOW_PLAIN") then
      for K in W.Solids.First_Index .. W.Solids.Last_Index loop
         if W.Solids (K).Link = 0 then
            W.Solids (K).Mat.Kind := FW.Flat;
         end if;
      end loop;
   end if;
   FW.Forward (W);
   for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
      if W.Cameras (C).Link = 0 then
         Eye := C;
      end if;
   end loop;
   if Eye = 0 then
      Put_Line ("seed" & Seed'Image & ": no fixed eye");
      return;
   end if;
   declare
      type Pose_Array_Access is access Flow.Pose_Array;
      type Joint_Array_Access is access Fit.Joint_Array;
      --  An arm the eye watched: its number in the world, its poses' evidence, its true joints in the world's frame
      --  and unit (both arms' frames are the world's: the eye's pose in each is its pose in the world).
      type Arm_Set is record
         Number : Positive := 1;
         Views  : Pose_Array_Access;
         Joints : Joint_Array_Access;
         Pixels : Natural := 0;
      end record;
      --  FLOW_CACHE2 names a file of the other arm's pictures (read or written as FLOW_CACHE): both arms are then
      --  fitted together through the eye's one lens.
      Both  : constant Boolean := Ada.Environment_Variables.Exists ("FLOW_CACHE2");
      Sets  : array (1 .. (if Both then 2 else 1)) of Arm_Set;
      Truth : constant Rigid := W.Cameras (Eye).Pose;
      --  The noise of one pixel of the difference of two of the pictures' means where nothing changed: the camera's
      --  noise a channel through the luma's weights, over the frames, twice.
      Noise : constant Real :=
        Noise_Level * EF.Sqrt (0.299 ** 2 + 0.587 ** 2 + 0.114 ** 2) / EF.Sqrt (Real (Frames)) * EF.Sqrt (2.0);

      procedure Gather (Number : Positive; Cache_Name : String; Set : out Arm_Set) is
         Group : Natural := 0;
      begin
         Set.Number := Number;
         for G in W.Groups.First_Index .. W.Groups.Last_Index loop
            if To_String (W.Groups (G).Name) = "arm" & Trim (Number) & "_joints" then
               Group := G;
            end if;
         end loop;
         if Group = 0 then
            raise Constraint_Error with "seed" & Seed'Image & ": no arm" & Number'Image;
         end if;
         declare
            Links  : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
            N      : constant Natural := Natural (Links.Length);
            Rest   : Real_Array (1 .. N);
            package SIO renames Ada.Streams.Stream_IO;
            Cached : constant Boolean := Cache_Name /= "" and then Ada.Directories.Exists (Cache_Name);
            File   : SIO.File_Type;
            --  Every pose's pictures, kept until all are in: the background is their stills' per-pixel median.
            type Luma_Matrix is array (1 .. Poses) of Flow.Luma_Array (1 .. N);
            type Reading_Matrix is array (1 .. Poses) of Real_Array (1 .. N);
            All_Still   : Flow.Luma_Array (1 .. Poses);
            All_Minus, All_Plus : Luma_Matrix;
            All_Readings, All_Steps : Reading_Matrix;
         begin
            Set.Joints := new Fit.Joint_Array (1 .. N);
            Set.Views := new Flow.Pose_Array (1 .. Poses);
            for J in 1 .. N loop
               declare
                  L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
               begin
                  Rest (J) := L.Q;
                  Set.Joints (J) := (W => Times (L.Pose.Rotation, L.Axis), P => L.Pose.Translation, C => L.Unit,
                                     Slide => L.Kind = FW.Prismatic);
               end;
            end loop;
            --  The cache: read when it exists (the renders and their noise as they were), written when it does not,
            --  so that the fit can be studied again in seconds.
            if Cached then
               SIO.Open (File, SIO.In_File, Cache_Name);
            elsif Cache_Name /= "" then
               SIO.Create (File, SIO.Out_File, Cache_Name);
            end if;
            for V in 1 .. Poses loop
               declare
                  Readings : Real_Array (1 .. N);
                  Minus, Plus : Flow.Luma_Array (1 .. N);
                  Steps    : Real_Array (1 .. N);
                  Still    : Flow.Luma_Access;
               begin
                  if Cached then
                     Real_Array'Read (SIO.Stream (File), Readings);
                     Real_Array'Read (SIO.Stream (File), Steps);
                     Still := new Real_Array (1 .. Width * Height);
                     Real_Array'Read (SIO.Stream (File), Still.all);
                     for J in 1 .. N loop
                        Minus (J) := new Real_Array (1 .. Width * Height);
                        Real_Array'Read (SIO.Stream (File), Minus (J).all);
                        Plus (J) := new Real_Array (1 .. Width * Height);
                        Real_Array'Read (SIO.Stream (File), Plus (J).all);
                     end loop;
                  else
                     for J in 1 .. N loop
                        declare
                           L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                        begin
                           Readings (J) := (if V = 1 then 0.0 else (2.0 * Uniform - 1.0) * Swing / L.Unit);
                           L.Q := Rest (J) + Readings (J);
                           Steps (J) := Turn / L.Unit;
                        end;
                     end loop;
                     FW.Forward (W);
                     Still := Picture;
                     for J in 1 .. N loop
                        declare
                           L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                        begin
                           L.Q := Rest (J) + Readings (J) - Steps (J);
                           FW.Forward (W);
                           Minus (J) := Picture;
                           L.Q := Rest (J) + Readings (J) + Steps (J);
                           FW.Forward (W);
                           Plus (J) := Picture;
                           L.Q := Rest (J) + Readings (J);
                           FW.Forward (W);
                        end;
                     end loop;
                     if Cache_Name /= "" then
                        Real_Array'Write (SIO.Stream (File), Readings);
                        Real_Array'Write (SIO.Stream (File), Steps);
                        Real_Array'Write (SIO.Stream (File), Still.all);
                        for J in 1 .. N loop
                           Real_Array'Write (SIO.Stream (File), Minus (J).all);
                           Real_Array'Write (SIO.Stream (File), Plus (J).all);
                        end loop;
                     end if;
                  end if;
                  All_Still (V) := Still;
                  All_Minus (V) := Minus;
                  All_Plus (V) := Plus;
                  All_Readings (V) := Readings;
                  All_Steps (V) := Steps;
               end;
            end loop;
            if Cache_Name /= "" then
               SIO.Close (File);
            end if;
            --  The arm back at rest: the other arm is seen at its own rest by this one's pictures.
            for J in 1 .. N loop
               W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
            end loop;
            FW.Forward (W);
            declare
               Background : Real_Array (1 .. Width * Height);
               Column     : Real_Array (1 .. Poses);
            begin
               for K in Background'Range loop
                  for V in 1 .. Poses loop
                     Column (V) := All_Still (V) (K);
                  end loop;
                  for I in 2 .. Poses loop
                     for D in reverse 2 .. I loop
                        exit when Column (D - 1) <= Column (D);
                        declare
                           Swap : constant Real := Column (D);
                        begin
                           Column (D) := Column (D - 1);
                           Column (D - 1) := Swap;
                        end;
                     end loop;
                  end loop;
                  Background (K) := (if Poses mod 2 = 1 then Column ((Poses + 1) / 2)
                                     else (Column (Poses / 2) + Column (Poses / 2 + 1)) / 2.0);
               end loop;
               for V in 1 .. Poses loop
                  Set.Views (V) := Flow.Evidence (All_Readings (V), All_Still (V).all, Background, Noise,
                                                  All_Minus (V), All_Plus (V), All_Steps (V), Width, Height);
                  Set.Pixels := Set.Pixels + Set.Views (V).Pixels;
                  Put_Line ("arm" & Number'Image & " pose" & V'Image & ":" & Set.Views (V).Pixels'Image
                            & " pixels moved");
                  Free (All_Still (V));
                  for J in 1 .. N loop
                     Free (All_Minus (V) (J));
                     Free (All_Plus (V) (J));
                  end loop;
               end loop;
            end;
         end;
      end Gather;

      function Cache_Of (Name : String) return String is
        (if Ada.Environment_Variables.Exists (Name) then Ada.Environment_Variables.Value (Name) else "");
   begin
      Gather (Arm, Cache_Of ("FLOW_CACHE"), Sets (1));
      if Both then
         Gather (3 - Arm, Cache_Of ("FLOW_CACHE2"), Sets (2));
      end if;
      --  The model at the truth (the first arm): the residuals the true eye leaves, with the joints as given and with
      --  every joint's sense the other way (a convention of the fake's turns against the fit's).
      declare
         Scatter  : Real;
         In_Front : Natural;
         Flipped  : Fit.Joint_Array := Sets (1).Joints.all;
         Cam      : FW.Camera renames W.Cameras (Eye);
         Middle_Lens : constant Fit.Lens :=
           (Fx | Fy => Cam.Fx, Cx => Real (Width) / 2.0, Cy => Real (Height) / 2.0, K1 | K2 => 0.0);
         True_Lens   : constant Fit.Lens := (Fx => Cam.Fx, Fy => Cam.Fy, Cx => Cam.Cx, Cy => Cam.Cy, K1 | K2 => 0.0);
      begin
         Put_Line ("the eye's true aspect (fy / fx)" & Real'Image (Cam.Fy / Cam.Fx));
         for J in Flipped'Range loop
            Flipped (J).C := -Flipped (J).C;
         end loop;
         Flow.Judge_At (Sets (1).Views.all, Flipped, Width, Height, Truth, Middle_Lens, Scatter, In_Front);
         Put_Line ("at the truth, every joint turned the other way: scatter " & Image (Scatter, 3) & ", in front"
                   & In_Front'Image);
         Flow.Judge_At (Sets (1).Views.all, Sets (1).Joints.all, Width, Height, Truth, Middle_Lens, Scatter,
                        In_Front);
         Put_Line ("at the truth with the principal point at the middle and one focal length: scatter "
                   & Image (Scatter, 3) & ", in front" & In_Front'Image);
         for K in Sets'Range loop
            Flow.Judge_At (Sets (K).Views.all, Sets (K).Joints.all, Width, Height, Truth, True_Lens, Scatter,
                           In_Front);
            Put_Line ((if K = 1 then "at the truth: scatter " else "at the truth, arm" & Sets (K).Number'Image
                      & ": scatter ") & Image (Scatter, 3) & ", in front" & In_Front'Image & ", "
                      & Image (Real (Ada.Calendar.Clock - Began), 1) & " s");
         end loop;
         if Ada.Environment_Variables.Exists ("FLOW_TRUTH_ONLY") then
            return;
         end if;
         --  FLOW_SYNTHETIC: the rates made the model's at the true eye with the pictures' noise (Flow.Synthesize), for
         --  telling what the fit makes of the renders from what it makes of the model.
         if Ada.Environment_Variables.Exists ("FLOW_SYNTHETIC") then
            for K in Sets'Range loop
               Flow.Synthesize (Sets (K).Views.all, Sets (K).Joints.all, Width, Height, Truth, True_Lens,
                                Gaussian'Access);
            end loop;
            Put_Line ("the rates made the model's at the true eye");
         end if;
      end;
      declare
         In_Arms : Flow.Arm_Flow_Array (Sets'Range);
         Result  : Flow.Report (Arms => Sets'Length, Count => Flow.Terms_For (Sets'Length));
         Pixels_All : Natural := 0;
      begin
         for K in Sets'Range loop
            In_Arms (K) := (Poses  => Sets (K).Views.all'Unchecked_Access,
                            Joints => Sets (K).Joints.all'Unchecked_Access);
            Pixels_All := Pixels_All + Sets (K).Pixels;
         end loop;
         Flow.Fit_Eye (In_Arms, Width, Height, Result);
         declare
            Seconds : constant Real := Real (Ada.Calendar.Clock - Began);
            Cam     : FW.Camera renames W.Cameras (Eye);
            Line    : Unbounded_String;
         begin
            Line := To_Unbounded_String
              ("seed" & Seed'Image & " arms" & Sets'Length'Image & ": "
               & (if Result.Determined then "TOLD" else "untold")
               & ", focal " & Image (Result.Lens.Fx, 1) & " (truth " & Image (Cam.Fx, 1) & ", "
               & Image (100.0 * (Result.Lens.Fx / Cam.Fx - 1.0), 1) & " %, sigma "
               & Image (100.0 * EF.Sqrt (Result.Covariance (1, 1)), 2) & " %), principal point ("
               & Image (Result.Lens.Cx, 1) & ", " & Image (Result.Lens.Cy, 1) & ") of (" & Image (Cam.Cx, 1) & ", "
               & Image (Cam.Cy, 1) & ")");
            for K in Sets'Range loop
               declare
                  Off    : constant Real := abs Minus (Result.Poses (K).Translation, Truth.Translation);
                  Turned : constant Real :=
                    Driver.Numerics.Angle (Times (Transposed (Truth.Rotation), Result.Poses (K).Rotation));
                  O      : constant Natural := 3 + 6 * (K - 1);
               begin
                  Append (Line, "; arm" & Sets (K).Number'Image & " centre off by " & Image (1000.0 * Off, 1)
                          & " mm (sigma " & Image (1000.0 * EF.Sqrt (Result.Covariance (O + 4, O + 4)
                                                   + Result.Covariance (O + 5, O + 5)
                                                   + Result.Covariance (O + 6, O + 6)), 1)
                          & "), turn off by " & Image (180.0 / Ada.Numerics.Pi * Turned, 2) & " deg");
               end;
            end loop;
            Append (Line, "; used" & Result.Used'Image & " of" & Result.Offered'Image & " pixels (" & Pixels_All'Image
                    & " moved), scatter " & Image (Result.Scatter, 2) & ", spread of a rate "
                    & Image (Result.Additive, 2) & " sigmas and " & Image (100.0 * Result.Relative, 1) & " %"
                    & ", widened " & Image (Result.Widened, 1) & ", spread " & Image (Result.Spread, 2) & " px, "
                    & Image (Seconds, 1) & " s" & (if Result.Determined then "" else "; " & To_String (Result.Why)));
            Put_Line (To_String (Line));
         end;
      end;
   end;
end Driver.Robot.Flow_Bench;
