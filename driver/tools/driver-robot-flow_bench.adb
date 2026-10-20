with Ada.Calendar;
with Ada.Containers.Vectors;
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
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Flow_Eye;
with Driver.Stats;
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
   Unrounded   : constant Boolean := Ada.Environment_Variables.Exists ("FLOW_NO_ROUND");
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
   type Real_Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Real_Matrix_Access);

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
               --  FLOW_NO_ROUND (a study): the levels not rounded to whole grey levels.
               function Level (X : Real) return Real is
                 (Real'Max (0.0, Real'Min (255.0, (if Unrounded then X + Noise_Level * Gaussian
                                                       else Real'Rounding (X + Noise_Level * Gaussian)))));
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

      --  Double_Name (FLOW_RICHARDSON, FLOW_RICHARDSON2; a study): a cache of the same poses turned twice as far, each
      --  joint's change taken by Richardson's extrapolation, (4 D (d) - D (2 d)) / 3: the central difference's error
      --  of the second order in the turn taken off.
      procedure Gather (Number : Positive; Cache_Name : String; Set : out Arm_Set; Double_Name : String := "") is
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
            Doubled : constant Boolean := Double_Name /= "" and then Ada.Directories.Exists (Double_Name);
            Double  : SIO.File_Type;
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
               if Doubled then
                  SIO.Open (Double, SIO.In_File, Double_Name);
               end if;
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
                     if Doubled then
                        declare
                           R2, S2 : Real_Array (1 .. N);
                           P2     : Real_Array (1 .. Width * Height);
                           M2     : Real_Array (1 .. Width * Height);
                        begin
                           Real_Array'Read (SIO.Stream (Double), R2);
                           Real_Array'Read (SIO.Stream (Double), S2);
                           Real_Array'Read (SIO.Stream (Double), P2);   --  its still, unused
                           for J in 1 .. N loop
                              Real_Array'Read (SIO.Stream (Double), M2);
                              Real_Array'Read (SIO.Stream (Double), P2);
                              --  (Plus - Minus) / (2 d) made (4 D (d) - D (2 d)) / 3 with D (h) the change over 2 h
                              for K in M2'Range loop
                                 Plus (J) (K) := Minus (J) (K)
                                   + (8.0 * (Plus (J) (K) - Minus (J) (K)) - (P2 (K) - M2 (K))) / 6.0;
                              end loop;
                           end loop;
                        end;
                     end if;
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
                  --  FLOW_BEST_TURN: each joint's best turn (Flow_Eye.Best_Turn) from this pose's still and its
                  --  turn either way, in radians, against the turn rendered.
                  if Ada.Environment_Variables.Exists ("FLOW_BEST_TURN") then
                     for J in 1 .. N loop
                        declare
                           L    : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                           Best : constant Real :=
                             Flow.Best_Turn (Still.all, Plus (J).all, Steps (J), Noise, Width, Height);
                        begin
                           Put_Line ("arm" & Number'Image & " pose" & V'Image & " joint" & J'Image & ": best turn "
                                     & Image (Best * L.Unit, 5) & " rad, rendered " & Image (Steps (J) * L.Unit, 5));
                        end;
                     end loop;
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
            if Doubled then
               SIO.Close (Double);
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
               --  FLOW_TRUE_BACK (a study): the background as it is, this arm's solids taken out of the world (moved
               --  a kilometre below it) and the picture rendered, against the stills' median.
               if Ada.Environment_Variables.Exists ("FLOW_TRUE_BACK") then
                  declare
                     Saved  : array (W.Solids.First_Index .. W.Solids.Last_Index) of Rigid;
                     Bare   : Flow.Luma_Access;
                     Differ : Natural := 0;
                  begin
                     for I in Saved'Range loop
                        Saved (I) := W.Solids (I).Pose;
                        if W.Solids (I).Link > 0 and then W.Links (W.Solids (I).Link).Arm = Number then
                           W.Solids (I).Pose.Translation := [0.0, 0.0, -1000.0];
                        end if;
                     end loop;
                     Bare := Picture;
                     for I in Saved'Range loop
                        W.Solids (I).Pose := Saved (I);
                     end loop;
                     for K in Background'Range loop
                        if abs (Background (K) - Bare (K)) > 4.0 * Noise then
                           Differ := Differ + 1;
                        end if;
                        Background (K) := Bare (K);
                     end loop;
                     Free (Bare);
                     Put_Line ("arm" & Number'Image & ": the true background, the stills' median off it at"
                               & Differ'Image & " pixels");
                  end;
               end if;
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
      Gather (Arm, Cache_Of ("FLOW_CACHE"), Sets (1), Cache_Of ("FLOW_RICHARDSON"));
      if Both then
         Gather (3 - Arm, Cache_Of ("FLOW_CACHE2"), Sets (2), Cache_Of ("FLOW_RICHARDSON2"));
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
         --  FLOW_TRUTH_FLOW = K (BENCH): arm K's measured rates against the truth's at every kept pixel: the surface
         --  the pixel's centre sees (Fake_Render.Nearest), its motion as each joint turns it (nought on a surface no
         --  joint of the arm moves), through the true lens into the picture and through the pixel's gradient into a
         --  rate.
         --  Pixels by what their centre sees (the arm, or something the arm does not move: its outline's change spread
         --  there by the pictures' blur and the smoothing) and by whether a neighbour a pixel and a half away sees
         --  another solid: per class the count, the median of the measured over the truth's rate and the root mean
         --  square of their difference over its sigma. FLOW_TRUTH_DUMP names a file for every pixel's line.
         if Ada.Environment_Variables.Exists ("FLOW_TRUTH_FLOW") then
            declare
               K     : constant Positive := Positive'Value (Ada.Environment_Variables.Value ("FLOW_TRUTH_FLOW"));
               Group : Natural := 0;
               Dump  : File_Type;
               Dumping : constant Boolean := Ada.Environment_Variables.Exists ("FLOW_TRUTH_DUMP");
               type Class_Stat is record
                  Count : Natural := 0;
                  Sq    : Real := 0.0;
                  Rates : Natural := 0;
               end record;
               --  classes: 1 on the arm inside, 2 on the arm at an edge, 3 beside the arm
               Stats  : array (1 .. 3) of Class_Stat;
               package Row_Vectors is new Ada.Containers.Vectors (Positive, Real);
               Ratios : array (1 .. 3) of Row_Vectors.Vector;
               function Hit_Link (U, V : Real) return Natural is
                  Solid : Natural;
                  Along : Real;
               begin
                  Fake_Render.Nearest (W, W.Cameras (Eye), U, V, Solid, Along);
                  return (if Solid = 0 then 0 else W.Solids (Solid).Link);
               end Hit_Link;
            begin
               for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                  if To_String (W.Groups (G).Name) = "arm" & Trim (Sets (K).Number) & "_joints" then
                     Group := G;
                  end if;
               end loop;
               if Dumping then
                  Create (Dump, Out_File, Ada.Environment_Variables.Value ("FLOW_TRUTH_DUMP"));
               end if;
               declare
                  Links : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
                  N     : constant Natural := Natural (Links.Length);
                  Rest  : Real_Array (1 .. N);
                  Cam   : FW.Camera renames W.Cameras (Eye);
                  --  Whether joint J's link carries link L (L is it or below it).
                  function Carries (J : Positive; L : Natural) return Boolean is
                     Here : Natural := L;
                  begin
                     while Here /= 0 loop
                        if Here = Links (Links.First_Index + J - 1) then
                           return True;
                        end if;
                        Here := W.Links (Here).Parent;
                     end loop;
                     return False;
                  end Carries;
               begin
                  for J in 1 .. N loop
                     Rest (J) := W.Links (Links (Links.First_Index + J - 1)).Q;
                  end loop;
                  for V in Sets (K).Views'Range loop
                     declare
                        View : Flow.Pose_View renames Sets (K).Views (V).all;
                     begin
                        for J in 1 .. N loop
                           W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J) + View.Readings (J);
                        end loop;
                        FW.Forward (W);
                        for I in 1 .. View.Pixels loop
                           declare
                              Solid : Natural;
                              Along : Real;
                              Px    : constant Real := (View.U (I) - Cam.Cx) / Cam.Fx;
                              Py    : constant Real := (View.V (I) - Cam.Cy) / Cam.Fy;
                              Pn    : constant Real := EF.Sqrt (Px * Px + Py * Py + 1.0);
                              Link  : Natural;
                              Edge  : Boolean := False;
                              Class : Positive;
                           begin
                              Fake_Render.Nearest (W, Cam, View.U (I), View.V (I), Solid, Along);
                              Link := (if Solid = 0 then 0 else W.Solids (Solid).Link);
                              for D of Real_Array'[1.5, -1.5] loop
                                 Edge := Edge or else Hit_Link (View.U (I) + D, View.V (I)) /= Link
                                   or else Hit_Link (View.U (I), View.V (I) + D) /= Link;
                              end loop;
                              Class := (if Solid /= 0 and then Carries (1, Link) then (if Edge then 2 else 1) else 3);
                              Stats (Class).Count := Stats (Class).Count + 1;
                              for J in 1 .. View.Link (I) loop
                                 declare
                                    --  The true motion along the gradient at a place of the picture as joint J turns
                                    --  (the surface there, nought where J does not move it).
                                    function Along_Gradient (U, V : Real) return Real is
                                       S2 : Natural;
                                       A2 : Real;
                                       Qx : constant Real := (U - Cam.Cx) / Cam.Fx;
                                       Qy : constant Real := (V - Cam.Cy) / Cam.Fy;
                                       Qn : constant Real := EF.Sqrt (Qx * Qx + Qy * Qy + 1.0);
                                       Lk : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                                       Ax : constant Vec3 := Times (Lk.Pose.Rotation, Lk.Axis);
                                    begin
                                       Fake_Render.Nearest (W, Cam, U, V, S2, A2);
                                       if S2 = 0 or else not Carries (J, W.Solids (S2).Link) then
                                          return 0.0;
                                       end if;
                                       declare
                                          P2 : constant Vec3 := Cam.Pose.Translation
                                            + A2 * Times (Cam.Pose.Rotation, Vec3'[Qx / Qn, Qy / Qn, 1.0 / Qn]);
                                          V2 : constant Vec3 :=
                                            (if Lk.Kind = FW.Prismatic then Scaled (Lk.Unit, Ax)
                                             else Cross (Scaled (Lk.Unit, Ax), Minus (P2, Lk.Pose.Translation)));
                                          Pc : constant Vec3 :=
                                            Transposed_Times (Cam.Pose.Rotation, Minus (P2, Cam.Pose.Translation));
                                          Vc2 : constant Vec3 := Transposed_Times (Cam.Pose.Rotation, V2);
                                          G   : constant Real := EF.Sqrt (View.Gu (I) ** 2 + View.Gv (I) ** 2);
                                       begin
                                          return (View.Gu (I) * Cam.Fx * (Vc2 (1) * Pc (3) - Pc (1) * Vc2 (3))
                                                  + View.Gv (I) * Cam.Fy * (Vc2 (2) * Pc (3) - Pc (2) * Vc2 (3)))
                                            / (Pc (3) ** 2 * Real'Max (G, Real'Model_Small));
                                       end;
                                    end Along_Gradient;
                                    Centre : constant Real := Along_Gradient (View.U (I), View.V (I));
                                    Spread : Real := 0.0;
                                    --  The share of the smoothing's reach (whole pixels within its radius) whose
                                    --  surface joint J moves.
                                    function Moving_Share return Real is
                                       Moved, All_Of : Natural := 0;
                                       S3 : Natural;
                                       A3 : Real;
                                    begin
                                       for Dy in -Flow.Smoothing_Radius .. Flow.Smoothing_Radius loop
                                          for Dx in -Flow.Smoothing_Radius .. Flow.Smoothing_Radius loop
                                             if Dx * Dx + Dy * Dy <= Flow.Smoothing_Radius ** 2 then
                                                All_Of := All_Of + 1;
                                                Fake_Render.Nearest
                                                  (W, Cam, View.U (I) + Real (Dx), View.V (I) + Real (Dy), S3, A3);
                                                if S3 /= 0 and then Carries (J, W.Solids (S3).Link) then
                                                   Moved := Moved + 1;
                                                end if;
                                             end if;
                                          end loop;
                                       end loop;
                                       return Real (Moved) / Real (Natural'Max (1, All_Of));
                                    end Moving_Share;
                                 begin
                                    for D of Real_Array'[1.0, -1.0] loop
                                       Spread := Real'Max
                                         (Spread, abs (Along_Gradient (View.U (I) + D, View.V (I)) - Centre));
                                       Spread := Real'Max
                                         (Spread, abs (Along_Gradient (View.U (I), View.V (I) + D) - Centre));
                                    end loop;
                                    Spread := (if abs Centre > 0.0 then Spread / abs Centre else Real'Last);
                                 declare
                                    Lj   : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                                    Axis : constant Vec3 := Times (Lj.Pose.Rotation, Lj.Axis);
                                    X    : constant Vec3 :=
                                      (if Solid = 0 then [0.0, 0.0, 0.0]
                                       else Cam.Pose.Translation
                                            + Along * Times (Cam.Pose.Rotation, Vec3'[Px / Pn, Py / Pn, 1.0 / Pn]));
                                    Vel  : constant Vec3 :=
                                      (if Solid = 0 or else not Carries (J, Link) then [0.0, 0.0, 0.0]
                                       elsif Lj.Kind = FW.Prismatic then Scaled (Lj.Unit, Axis)
                                       else Cross (Scaled (Lj.Unit, Axis), Minus (X, Lj.Pose.Translation)));
                                    Xc   : constant Vec3 := Transposed_Times (Cam.Pose.Rotation,
                                                                              Minus (X, Cam.Pose.Translation));
                                    Vc   : constant Vec3 := Transposed_Times (Cam.Pose.Rotation, Vel);
                                    Du   : constant Real :=
                                      (if Solid = 0 then 0.0
                                       else Cam.Fx * (Vc (1) * Xc (3) - Xc (1) * Vc (3)) / Xc (3) ** 2);
                                    Dv   : constant Real :=
                                      (if Solid = 0 then 0.0
                                       else Cam.Fy * (Vc (2) * Xc (3) - Xc (2) * Vc (3)) / Xc (3) ** 2);
                                    Pred : constant Real := -(View.Gu (I) * Du + View.Gv (I) * Dv);
                                 begin
                                    Stats (Class).Sq :=
                                      Stats (Class).Sq + ((View.Rate (I, J) - Pred) / View.Sigma (I, J)) ** 2;
                                    Stats (Class).Rates := Stats (Class).Rates + 1;
                                    if abs Pred > 5.0 * View.Sigma (I, J) then
                                       Ratios (Class).Append (View.Rate (I, J) / Pred);
                                    end if;
                                    if Dumping then
                                       Put_Line (Dump, V'Image & I'Image & " " & Image (View.U (I), 1) & " "
                                                 & Image (View.V (I), 1) & Class'Image & J'Image & View.Link (I)'Image
                                                 & " " & Image (View.Rate (I, J), 4) & " " & Image (Pred, 4) & " "
                                                 & Image (View.Sigma (I, J), 4) & " "
                                                 & Image (EF.Sqrt (View.Gu (I) ** 2 + View.Gv (I) ** 2), 3) & " "
                                                 & Image (View.Back (I), 3) & " "
                                                 & Image ((if Solid = 0 then 0.0 else Xc (3)), 4) & " "
                                                 & Image (Real'Min (Spread, 1.0E6), 4) & " "
                                                 & Image (EF.Sqrt (Du ** 2 + Dv ** 2), 5) & " "
                                                 & Image (Moving_Share, 3));
                                    end if;
                                 end;
                                 end;
                              end loop;
                           end;
                        end loop;
                     end;
                  end loop;
                  for J in 1 .. N loop
                     W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
                  end loop;
                  FW.Forward (W);
               end;
               if Dumping then
                  Close (Dump);
               end if;
               for C in 1 .. 3 loop
                  declare
                     A : Real_Array (1 .. Natural (Ratios (C).Length));
                  begin
                     for I in A'Range loop
                        A (I) := Ratios (C) (I);
                     end loop;
                     Put_Line ("truth flow, "
                               & (case C is when 1 => "on the arm inside", when 2 => "on the arm at an edge",
                                   when others => "beside the arm")
                               & ":" & Stats (C).Count'Image
                               & " pixels," & Stats (C).Rates'Image & " rates, rms "
                               & Image (EF.Sqrt (Stats (C).Sq / Real (Natural'Max (1, Stats (C).Rates))), 2)
                               & " sigmas off the truth's; measured over the truth's, median "
                               & (if A'Length > 0 then Image (Driver.Stats.Median (A), 3) else "-") & " of"
                               & A'Length'Image);
                  end;
               end loop;
            end;
         end if;
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
         --  FLOW_CHAIN = K (BENCH, class B's fit): arm K's joints and the eye's lens fitted to every pose's rates, the
         --  eye held at its true pose (the arm's frame is the eye's for an arm that carries none): per joint its
         --  axis's tilt across itself (two), its line's move across it (two) and its unit's logarithm, and the lens's
         --  focal length's logarithm and principal point, by Levenberg-Marquardt on a central-difference Jacobian,
         --  from the truth tilted by FLOW_CHAIN_TILT degrees and moved by FLOW_CHAIN_MOVE metres along the first
         --  direction across each axis, its unit times 1 + FLOW_CHAIN_UNIT; each rate's spread its sigma and
         --  FLOW_CHAIN_REL of the rate in quadrature. Against the truth: each axis's angle, its line's distance from
         --  the true one's point, its unit, and the lens.
         if Ada.Environment_Variables.Exists ("FLOW_CHAIN") then
            declare
               K     : constant Positive := Positive'Value (Ada.Environment_Variables.Value ("FLOW_CHAIN"));
               function Env (Name : String; Default : Real) return Real is
                 (if Ada.Environment_Variables.Exists (Name) then Real'Value (Ada.Environment_Variables.Value (Name))
                  else Default);
               Rel   : constant Real := Env ("FLOW_CHAIN_REL", 0.0);
               Tilt  : constant Real := Env ("FLOW_CHAIN_TILT", 0.0) / 57.29577951308232;
               Move  : constant Real := Env ("FLOW_CHAIN_MOVE", 0.0);
               Unit  : constant Real := Env ("FLOW_CHAIN_UNIT", 0.0);
               Views : Flow.Pose_Array renames Sets (K).Views.all;
               True_J : Fit.Joint_Array renames Sets (K).Joints.all;
               N     : constant Positive := True_J'Length;
               T     : constant Positive := 3 + 5 * N;
               type Basis is array (1 .. N) of Vec3;
               E1, E2 : Basis;
               X     : Real_Vector (1 .. T) := [others => 0.0];
               function Joints_Of (Y : Real_Vector) return Fit.Joint_Array is
                  Result : Fit.Joint_Array (1 .. N);
               begin
                  for J in 1 .. N loop
                     declare
                        A  : Fit.Joint renames True_J (True_J'First + J - 1);
                        B  : constant Positive := 3 + 5 * (J - 1);
                        W  : constant Vec3 :=
                          [A.W (1) + Y (B + 1) * E1 (J) (1) + Y (B + 2) * E2 (J) (1),
                           A.W (2) + Y (B + 1) * E1 (J) (2) + Y (B + 2) * E2 (J) (2),
                           A.W (3) + Y (B + 1) * E1 (J) (3) + Y (B + 2) * E2 (J) (3)];
                        Nw : constant Real := abs W;
                     begin
                        Result (J) := (W => [W (1) / Nw, W (2) / Nw, W (3) / Nw],
                                       P => [A.P (1) + Y (B + 3) * E1 (J) (1) + Y (B + 4) * E2 (J) (1),
                                             A.P (2) + Y (B + 3) * E1 (J) (2) + Y (B + 4) * E2 (J) (2),
                                             A.P (3) + Y (B + 3) * E1 (J) (3) + Y (B + 4) * E2 (J) (3)],
                                       C => A.C * Ada.Numerics.Long_Elementary_Functions.Exp (Y (B + 5)),
                                       Slide => A.Slide);
                     end;
                  end loop;
                  return Result;
               end Joints_Of;
               function Lens_Of (Y : Real_Vector) return Fit.Lens is
                 ((Fx => True_Lens.Fx * Ada.Numerics.Long_Elementary_Functions.Exp (Y (1)),
                   Fy => True_Lens.Fy * Ada.Numerics.Long_Elementary_Functions.Exp (Y (1)),
                   Cx => True_Lens.Cx + Y (2), Cy => True_Lens.Cy + Y (3), K1 => 0.0, K2 => 0.0));
               function Res (Y : Real_Vector) return Real_Array is
                 (Flow.Residuals_At (Views, Joints_Of (Y), Width, Height, Truth, Lens_Of (Y), Rel));
               function Cost (R : Real_Array) return Real is
                  Sum : Real := 0.0;
               begin
                  for V of R loop
                     Sum := Sum + V * V;
                  end loop;
                  return Sum;
               end Cost;
               procedure Compare (Label : String; Y : Real_Vector) is
                  J1 : constant Fit.Joint_Array := Joints_Of (Y);
                  L1 : constant Fit.Lens := Lens_Of (Y);
               begin
                  for J in 1 .. N loop
                     declare
                        A  : Fit.Joint renames True_J (True_J'First + J - 1);
                        Cs : constant Real := Dot (A.W, J1 (J).W);
                        D  : constant Vec3 := Minus (A.P, J1 (J).P);
                        --  the true line's point's distance from the fitted line
                        Off : constant Real := abs Minus (D, Scaled (Dot (D, J1 (J).W), J1 (J).W));
                     begin
                        Put_Line ("chain " & Label & ": joint" & J'Image & " axis off "
                                  & Image (57.29577951308232 * Ada.Numerics.Long_Elementary_Functions.Arccos
                                             (Real'Max (-1.0, Real'Min (1.0, Cs))), 2)
                                  & " deg, line off " & Image (1000.0 * Off, 1) & " mm, unit "
                                  & Image (J1 (J).C / A.C, 4) & " of the truth's");
                     end;
                  end loop;
                  Put_Line ("chain " & Label & ": focal " & Image (L1.Fx / True_Lens.Fx, 4)
                            & " of the truth's, centre off "
                            & Image (L1.Cx - True_Lens.Cx, 2) & " " & Image (L1.Cy - True_Lens.Cy, 2) & " px");
               end Compare;
               Began : constant Ada.Calendar.Time := Ada.Calendar.Clock;
            begin
               for J in 1 .. N loop
                  Fit.Across (True_J (True_J'First + J - 1).W, E1 (J), E2 (J));
                  X (3 + 5 * (J - 1) + 1) := Tilt;
                  X (3 + 5 * (J - 1) + 3) := Move;
                  X (3 + 5 * (J - 1) + 5) := Ada.Numerics.Long_Elementary_Functions.Log (1.0 + Unit);
               end loop;
               declare
                  R0 : constant Real_Array := Res (X);
                  Zero : constant Real_Vector (1 .. T) := [others => 0.0];
               begin
                  Put_Line ("chain: the truth's cost " & Image (Cost (Res (Zero)), 1) & " over" & R0'Length'Image
                            & " rates; the start's " & Image (Cost (R0), 1));
               end;
               Compare ("start", X);
               declare
                  Lambda : Real := 1.0E-3;
                  Steps  : constant Real_Vector (1 .. T) :=
                    [for I in 1 .. T =>
                       (if I = 1 then 1.0E-5 elsif I <= 3 then 1.0E-3
                        elsif (I - 4) mod 5 in 0 .. 1 then 1.0E-5
                        elsif (I - 4) mod 5 in 2 .. 3 then 1.0E-6 else 1.0E-5)];
               begin
                  for It in 1 .. 40 loop
                     declare
                        R0  : constant Real_Array := Res (X);
                        C0  : constant Real := Cost (R0);
                        Jac : Real_Matrix_Access := new Real_Matrix (R0'Range, 1 .. T);
                        Ata : Real_Matrix (1 .. T, 1 .. T) := [others => [others => 0.0]];
                        Atb : Real_Vector (1 .. T) := [others => 0.0];
                        Done : Boolean := False;
                     begin
                        for C in 1 .. T loop
                           declare
                              Xp : Real_Vector := X;
                              Xm : Real_Vector := X;
                           begin
                              Xp (C) := X (C) + Steps (C);
                              Xm (C) := X (C) - Steps (C);
                              declare
                                 Rp : constant Real_Array := Res (Xp);
                                 Rm : constant Real_Array := Res (Xm);
                              begin
                                 for I in R0'Range loop
                                    Jac (I, C) := (Rp (I) - Rm (I)) / (2.0 * Steps (C));
                                 end loop;
                              end;
                           end;
                        end loop;
                        for I in R0'Range loop
                           for A in 1 .. T loop
                              if Jac (I, A) /= 0.0 then
                                 Atb (A) := Atb (A) + Jac (I, A) * R0 (I);
                                 for B in A .. T loop
                                    Ata (A, B) := Ata (A, B) + Jac (I, A) * Jac (I, B);
                                 end loop;
                              end if;
                           end loop;
                        end loop;
                        for A in 1 .. T loop
                           for B in A + 1 .. T loop
                              Ata (B, A) := Ata (A, B);
                           end loop;
                        end loop;
                        Free (Jac);
                        loop
                           declare
                              M     : Real_Matrix := Ata;
                              Lower : Real_Matrix (1 .. T, 1 .. T);
                              Ok    : Boolean;
                           begin
                              for A in 1 .. T loop
                                 M (A, A) := Ata (A, A) * (1.0 + Lambda) + 1.0E-12;
                              end loop;
                              Driver.Numerics.Dense.Cholesky (M, Lower, Ok);
                              if Ok then
                                 declare
                                    D  : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, Atb);
                                    X1 : Real_Vector := X;
                                 begin
                                    for A in 1 .. T loop
                                       X1 (A) := X (A) - D (A);
                                    end loop;
                                    declare
                                       C1 : constant Real := Cost (Res (X1));
                                    begin
                                       if C1 < C0 then
                                          X := X1;
                                          Lambda := Real'Max (1.0E-7, Lambda / 3.0);
                                          Put_Line ("chain: step" & It'Image & ", cost " & Image (C0, 1) & " -> "
                                                    & Image (C1, 1) & ", lambda " & Lambda'Image & ", "
                                                    & Image (Real (Ada.Calendar.Clock - Began), 1) & " s");
                                          Done := C0 - C1 < 1.0E-6 * C0;
                                          exit;
                                       end if;
                                    end;
                                 end;
                              end if;
                              Lambda := Lambda * 4.0;
                              if Lambda > 1.0E8 then
                                 Done := True;
                                 exit;
                              end if;
                           end;
                        end loop;
                        exit when Done;
                     end;
                  end loop;
               end;
               Compare ("fitted", X);
            end;
            if Ada.Environment_Variables.Exists ("FLOW_CHAIN_ONLY") then
               return;
            end if;
         end if;
         --  FLOW_SCALE = "K,J,F": every rate of arm K's joint J times F (a joint's rates sharing one error).
         if Ada.Environment_Variables.Exists ("FLOW_SCALE") then
            declare
               Text  : constant String := Ada.Environment_Variables.Value ("FLOW_SCALE");
               C1    : constant Natural := Ada.Strings.Fixed.Index (Text, ",");
               C2    : constant Natural := Ada.Strings.Fixed.Index (Text (C1 + 1 .. Text'Last), ",");
               K     : constant Positive := Positive'Value (Text (Text'First .. C1 - 1));
               J     : constant Positive := Positive'Value (Text (C1 + 1 .. C2 - 1));
               By    : constant Real := Real'Value (Text (C2 + 1 .. Text'Last));
            begin
               for V of Sets (K).Views.all loop
                  for I in 1 .. V.Pixels loop
                     if V.Link (I) >= J then
                        V.Rate (I, J) := By * V.Rate (I, J);
                     end if;
                  end loop;
               end loop;
               Put_Line ("arm" & K'Image & " joint" & J'Image & "'s rates times " & Image (By, 4));
            end;
         end if;
      end;
      declare
         In_Arms : Flow.Arm_Flow_Array (Sets'Range);
         Result  : Flow.Report (Arms => Sets'Length, Count => Flow.Terms_For (Sets'Length));
         Pixels_All : Natural := 0;
         --  FLOW_UNIT_SIGMA: every joint's unit stated uncertain by that share, as an arm's own fit states it.
         Unit_Share : constant Real := Setting ("FLOW_UNIT_SIGMA", 0.0);
         type Sigmas_Access is access Real_Array;
         Sigmas  : array (Sets'Range) of Sigmas_Access;
      begin
         for K in Sets'Range loop
            if Unit_Share > 0.0 then
               Sigmas (K) := new Real_Array'(Sets (K).Joints'Range => Unit_Share);
            end if;
            In_Arms (K) := (Poses      => Sets (K).Views.all'Unchecked_Access,
                            Joints     => Sets (K).Joints.all'Unchecked_Access,
                            Unit_Sigma => (if Sigmas (K) = null then null else Sigmas (K).all'Unchecked_Access));
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
