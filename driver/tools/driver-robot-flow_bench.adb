with Ada.Calendar;
with Ada.Containers.Vectors;
with Ada.Command_Line;
with Ada.Directories;
with Ada.Environment_Variables;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Numerics.Long_Real_Arrays;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Conventions;
with Driver.Log;
with Driver.Parallel_For;
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

   --  FLOW_ARM_ERROR's three numbers (a study, below): the tilt in radians (given in degrees), the move in metres, the
   --  unit's share; nought without it.
   function Arm_Error (Which : Positive) return Real is
   begin
      if not Ada.Environment_Variables.Exists ("FLOW_ARM_ERROR") then
         return 0.0;
      end if;
      declare
         Text : constant String := Ada.Environment_Variables.Value ("FLOW_ARM_ERROR");
         C1   : constant Natural := Ada.Strings.Fixed.Index (Text, ",");
         C2   : constant Natural := Ada.Strings.Fixed.Index (Text (C1 + 1 .. Text'Last), ",");
      begin
         return (case Which is
                    when 1 => Real'Value (Text (Text'First .. C1 - 1)) / 57.29577951308232,
                    when 2 => Real'Value (Text (C1 + 1 .. C2 - 1)),
                    when others => Real'Value (Text (C2 + 1 .. Text'Last)));
      end;
   end Arm_Error;

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
            Hand_U, Hand_V : Real_Array (1 .. Poses) := [others => -1.0];
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
                     --  FLOW_SPREAD = K (BENCH): each pose after the rest the one of K draws whose hand (the arm's last
                     --  link) the true eye sees farthest from where it saw the hand at the poses before, the hand in
                     --  the picture (a study of how far the eye's terms come apart with the arm shown all over the
                     --  picture, the truth choosing); one draw as without it.
                     declare
                        Draws : constant Positive := Positive (Setting ("FLOW_SPREAD", 1.0));
                        Cam   : FW.Camera renames W.Cameras (Eye);
                        Far_Best : Real := -1.0;
                        Best  : Real_Array (1 .. N) := [others => 0.0];
                        Q     : Real_Array (1 .. N);
                     begin
                        for D in 1 .. (if V = 1 then 1 else Draws) loop
                           for J in 1 .. N loop
                              declare
                                 L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                              begin
                                 Q (J) := (if V = 1 then 0.0 else (2.0 * Uniform - 1.0) * Swing / L.Unit);
                                 L.Q := Rest (J) + Q (J);
                              end;
                           end loop;
                           FW.Forward (W);
                           declare
                              Hand : constant Vec3 := W.Links (Links (Links.Last_Index)).Pose.Translation;
                              Xc   : constant Vec3 :=
                                Transposed_Times
                                  (Cam.Pose.Rotation, Driver.Numerics.Minus (Hand, Cam.Pose.Translation));
                              U, Vv : Real := -1.0;
                              Far  : Real := 0.0;
                           begin
                              if Xc (3) > 0.0 then
                                 U := Cam.Cx + Cam.Fx * Xc (1) / Xc (3);
                                 Vv := Cam.Cy + Cam.Fy * Xc (2) / Xc (3);
                              end if;
                              if U >= 0.0 and then U < Real (Width) and then Vv >= 0.0 and then Vv < Real (Height)
                              then
                                 Far := Real'Last;
                                 for Before in 1 .. V - 1 loop
                                    Far := Real'Min (Far, EF.Sqrt ((U - Hand_U (Before)) ** 2
                                                                   + (Vv - Hand_V (Before)) ** 2));
                                 end loop;
                              end if;
                              if Draws = 1 or else Far > Far_Best then
                                 Far_Best := Far;
                                 Best := Q;
                                 Hand_U (V) := U;
                                 Hand_V (V) := Vv;
                              end if;
                           end;
                        end loop;
                        if Draws > 1 then
                           Put_Line ("arm" & Number'Image & " pose" & V'Image & ": the hand at ("
                                     & Image (Hand_U (V), 1) & ", " & Image (Hand_V (V), 1) & "), "
                                     & Image (Far_Best, 1)
                                     & " px from the poses before");
                        end if;
                        Readings := Best;
                     end;
                     for J in 1 .. N loop
                        declare
                           L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                        begin
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
      --  FLOW_ARM_ERROR = "T,M,U" (a study): every arm's joints as a fit off the truth would hand them, each axis
      --  tilted along the two directions across it and its point moved along them by Gaussian draws of T degrees and M
      --  metres, its unit times 1 + a draw of U (a generator of their own, the seed's: the pictures' noise as without
      --  them; FLOW_ERROR_DRAW = K another draw of them, the seed's K-th); the flows fitted with them, the wrist eyes'
      --  pictures placed by them (the eye at rest where it truly is, as an arm's reference frame is its eye's), and the
      --  refinement handed those sigmas as every arm's variants (Arm_World.Variants): what step 2's errors leave the
      --  fixed eye, and whether its spread says so.
      if Ada.Environment_Variables.Exists ("FLOW_ARM_ERROR") then
         declare
            Tilt   : constant Real := Arm_Error (1);
            Move   : constant Real := Arm_Error (2);
            Gain   : constant Real := Arm_Error (3);
            Errors : Ada.Numerics.Float_Random.Generator;
            function Draw return Real is
               U1 : constant Real := Real'Max (Real (Ada.Numerics.Float_Random.Random (Errors)), 1.0E-12);
               U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Errors));
            begin
               return EF.Sqrt (-2.0 * EF.Log (U1)) * EF.Cos (2.0 * Ada.Numerics.Pi * U2);
            end Draw;
         begin
            Ada.Numerics.Float_Random.Reset
              (Errors, Seed + 1000 * Natural (Setting ("FLOW_ERROR_DRAW", 0.0)));
            for S of Sets loop
               for J in S.Joints'Range loop
                  declare
                     A      : Fit.Joint renames S.Joints (J);
                     E1, E2 : Vec3;
                     T1     : constant Real := Tilt * Draw;
                     T2     : constant Real := Tilt * Draw;
                     M1     : constant Real := Move * Draw;
                     M2     : constant Real := Move * Draw;
                     U      : constant Real := Gain * Draw;
                  begin
                     Fit.Across (A.W, E1, E2);
                     A.P := Plus (A.P, Plus (Scaled (M1, E1), Scaled (M2, E2)));
                     A.W := Unit (Plus (A.W, Plus (Scaled (T1, E1), Scaled (T2, E2))));
                     A.C := A.C * (1.0 + U);
                  end;
               end loop;
            end loop;
            Put_Line ("the arms' joints off the truth: tilted" & Image (Tilt * 57.29577951308232, 2) & " deg, moved"
                      & Image (1000.0 * Move, 1) & " mm, units times 1 + " & Image (Gain, 4));
         end;
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
         --  FLOW_PROFILE (BENCH): the direction the table's points leave the eye open along (its pixels' places
         --  unchanged to first order: the least singular vector of their places' Jacobian in the focal length's
         --  logarithm, the principal point, the turn about the camera's axes and the centre), and the flows' scatter
         --  of every arm along it through the true eye, by the focal length's change: where the arms' own motion puts
         --  the eye along the one direction the table cannot tell.
         --  FLOW_PROFILE_UNIT = K (a study): every joint's unit scaled by 1 + K before the profile: how far a share of
         --  every rate moves the flows' least along the direction.
         if Ada.Environment_Variables.Exists ("FLOW_PROFILE_UNIT") then
            for K in Sets'Range loop
               for J in Sets (K).Joints'Range loop
                  Sets (K).Joints (J).C := Sets (K).Joints (J).C
                    * (1.0 + Real'Value (Ada.Environment_Variables.Value ("FLOW_PROFILE_UNIT")));
               end loop;
            end loop;
         end if;
         if Ada.Environment_Variables.Exists ("FLOW_PROFILE") then
            declare
               package LA renames Ada.Numerics.Long_Real_Arrays;
               package Pt_Vectors is new Ada.Containers.Vectors (Positive, Vec3, Driver.Numerics.Arrays."=");
               Table_Pts : Pt_Vectors.Vector;
               Step_Px   : constant Positive := 8;
               function Moved_Pose (Z : LA.Real_Vector) return Rigid is
                 ((Rotation    => Times (Truth.Rotation, Driver.Numerics.Exp (Vec3'[Z (4), Z (5), Z (6)])),
                   Translation => [Truth.Translation (1) + Z (7), Truth.Translation (2) + Z (8),
                                   Truth.Translation (3) + Z (9)]));
               function Moved_Lens (Z : LA.Real_Vector) return Fit.Lens is
                 ((Fx => Cam.Fx * EF.Exp (Z (1)), Fy => Cam.Fy * EF.Exp (Z (1)), Cx => Cam.Cx + Z (2),
                   Cy => Cam.Cy + Z (3), K1 | K2 => 0.0));
               procedure Places (Z : LA.Real_Vector; Into : out LA.Real_Vector) is
                  P : constant Rigid := Moved_Pose (Z);
                  L : constant Fit.Lens := Moved_Lens (Z);
                  K : Natural := Into'First - 1;
               begin
                  for X of Table_Pts loop
                     declare
                        Xc : constant Vec3 := Transposed_Times (P.Rotation, Driver.Numerics.Minus (X, P.Translation));
                     begin
                        Into (K + 1) := L.Cx + L.Fx * Xc (1) / Xc (3);
                        Into (K + 2) := L.Cy + L.Fy * Xc (2) / Xc (3);
                        K := K + 2;
                     end;
                  end loop;
               end Places;
            begin
               --  The table's points the true eye sees, on a grid.
               for Gy in 0 .. Height / Step_Px - 1 loop
                  for Gx in 0 .. Width / Step_Px - 1 loop
                     declare
                        Solid : Natural;
                        Along : Real;
                        U  : constant Real := Real (Gx * Step_Px) + 0.5;
                        V  : constant Real := Real (Gy * Step_Px) + 0.5;
                        Px : constant Real := (U - Cam.Cx) / Cam.Fx;
                        Py : constant Real := (V - Cam.Cy) / Cam.Fy;
                        Pn : constant Real := EF.Sqrt (Px * Px + Py * Py + 1.0);
                     begin
                        Fake_Render.Nearest (W, Cam, U, V, Solid, Along);
                        if Solid > 0 and then To_String (W.Solids (Solid).Name) = "table" then
                           Table_Pts.Append (Driver.Numerics.Plus
                                               (Truth.Translation,
                                                Driver.Numerics.Scaled
                                                  (Along, Times (Truth.Rotation, Vec3'[Px / Pn, Py / Pn, 1.0 / Pn]))));
                        end if;
                     end;
                  end loop;
               end loop;
               declare
                  M    : constant Positive := 2 * Natural (Table_Pts.Length);
                  Jac  : LA.Real_Matrix (1 .. M, 1 .. 9);
                  Zero : constant LA.Real_Vector (1 .. 9) := [others => 0.0];
                  Eps  : constant Real := EF."**" (Real'Model_Epsilon, 1.0 / 3.0);
                  Scale : constant LA.Real_Vector (1 .. 9) := [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
               begin
                  for T in 1 .. 9 loop
                     declare
                        Up, Down : LA.Real_Vector (1 .. 9) := Zero;
                        Pu, Pd   : LA.Real_Vector (1 .. M);
                     begin
                        Up (T) := Eps;
                        Down (T) := -Eps;
                        Places (Up, Pu);
                        Places (Down, Pd);
                        for I in 1 .. M loop
                           Jac (I, T) := (Pu (I) - Pd (I)) / (2.0 * Eps) * Scale (T);
                        end loop;
                     end;
                  end loop;
                  declare
                     Normal  : constant LA.Real_Matrix := LA.Transpose (Jac) * Jac;
                     Values  : LA.Real_Vector (1 .. 9);
                     Vectors : LA.Real_Matrix (1 .. 9, 1 .. 9);
                     Least   : Positive := 1;
                     --  Per arm, the class of every pixel two joints or more move (Flow's layout, pose after pose):
                     --  1 its centre on the arm away from an edge, 2 on the arm at an edge (a neighbour a pixel and a
                     --  half away sees another solid), 3 beside the arm.
                     type Class_Access is access Fit.Count_Array;
                     Classes : array (Sets'Range) of Class_Access;
                  begin
                     LA.Eigensystem (Normal, Values, Vectors);
                     for K in Sets'Range loop
                        declare
                           Group : Natural := 0;
                           Count : Natural := 0;
                        begin
                           for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                              if To_String (W.Groups (G).Name) = "arm" & Trim (Sets (K).Number) & "_joints" then
                                 Group := G;
                              end if;
                           end loop;
                           for V of Sets (K).Views.all loop
                              for I in 1 .. V.Pixels loop
                                 Count := Count + Boolean'Pos (V.Link (I) >= 2);
                              end loop;
                           end loop;
                           Classes (K) := new Fit.Count_Array (1 .. Natural'Max (1, Count));
                           declare
                              Links : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
                              N     : constant Natural := Natural (Links.Length);
                              Rest  : Real_Array (1 .. N);
                              Q     : Natural := 0;
                              function Carries (L : Natural) return Boolean is
                                 Here : Natural := L;
                              begin
                                 while Here /= 0 loop
                                    if Here = Links (Links.First_Index) then
                                       return True;
                                    end if;
                                    Here := W.Links (Here).Parent;
                                 end loop;
                                 return False;
                              end Carries;
                              function Link_At (U, V : Real) return Natural is
                                 Solid : Natural;
                                 Along : Real;
                              begin
                                 Fake_Render.Nearest (W, Cam, U, V, Solid, Along);
                                 return (if Solid = 0 then 0 else W.Solids (Solid).Link);
                              end Link_At;
                           begin
                              for J in 1 .. N loop
                                 Rest (J) := W.Links (Links (Links.First_Index + J - 1)).Q;
                              end loop;
                              for V of Sets (K).Views.all loop
                                 for J in 1 .. N loop
                                    W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J) + V.Readings (J);
                                 end loop;
                                 FW.Forward (W);
                                 for I in 1 .. V.Pixels loop
                                    if V.Link (I) >= 2 then
                                       Q := Q + 1;
                                       declare
                                          L    : constant Natural := Link_At (V.U (I), V.V (I));
                                          Edge : Boolean := False;
                                       begin
                                          for D of Real_Array'[1.5, -1.5] loop
                                             Edge := Edge or else Link_At (V.U (I) + D, V.V (I)) /= L
                                               or else Link_At (V.U (I), V.V (I) + D) /= L;
                                          end loop;
                                          Classes (K) (Q) :=
                                            (if L /= 0 and then Carries (L) then (if Edge then 2 else 1)
                                                              else 3);
                                       end;
                                    end if;
                                 end loop;
                              end loop;
                              for J in 1 .. N loop
                                 W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
                              end loop;
                              FW.Forward (W);
                           end;
                        end;
                     end loop;
                     for I in 2 .. 9 loop
                        if Values (I) < Values (Least) then
                           Least := I;
                        end if;
                     end loop;
                     declare
                        Dir : LA.Real_Vector (1 .. 9);
                     begin
                        for I in 1 .. 9 loop
                           Dir (I) := Vectors (I, Least) / Vectors (1, Least);
                        end loop;
                        Put_Line ("profile: the table's" & Table_Pts.Length'Image & " points leave open, per unit of"
                                  & " the focal length's logarithm: principal point" & Real'Image (Dir (2))
                                  & Real'Image (Dir (3)) & " px, turn" & Real'Image (Dir (4)) & Real'Image (Dir (5))
                                  & Real'Image (Dir (6)) & ", centre" & Real'Image (Dir (7)) & Real'Image (Dir (8))
                                  & Real'Image (Dir (9)) & "; eigenvalues least" & Real'Image (Values (Least)));
                        for Step in -12 .. 12 loop
                           declare
                              E  : constant Real := Real (Step) * 0.005;
                              Z  : LA.Real_Vector (1 .. 9);
                              Pz : LA.Real_Vector (1 .. M);
                              P0 : LA.Real_Vector (1 .. M);
                              Worst : Real := 0.0;
                              Line  : Unbounded_String;
                              Total : Real := 0.0;
                              Sc    : Real;
                              Fr    : Natural;
                           begin
                              for I in 1 .. 9 loop
                                 Z (I) := E * Dir (I);
                              end loop;
                              Places (Z, Pz);
                              Places (Zero, P0);
                              for I in 1 .. M loop
                                 Worst := Real'Max (Worst, abs (Pz (I) - P0 (I)));
                              end loop;
                              for K in Sets'Range loop
                                 Flow.Judge_At (Sets (K).Views.all, Sets (K).Joints.all, Width, Height, Moved_Pose (Z),
                                                Moved_Lens (Z), Sc, Fr);
                                 Total := Total + Sc ** 2;
                                 Append (Line, " arm" & Sets (K).Number'Image & " " & Image (Sc, 3) & " (" & Trim (Fr)
                                         & ")");
                              end loop;
                              Put_Line ("profile: focal " & Image (100.0 * E, 1) & " %: table points move at most "
                                        & Image (Worst, 3) & " px; flow scatter" & To_String (Line)
                                        & ", sum of squares "
                                        & Image (Total, 2));
                              declare
                                 Sums  : Real_Array (1 .. 3) := [others => 0.0];
                                 Frees : array (1 .. 3) of Natural := [others => 0];
                                 Text  : Unbounded_String;
                              begin
                                 for K in Sets'Range loop
                                    declare
                                       R   : constant Real_Array :=
                                         Flow.Residuals_At (Sets (K).Views.all, Sets (K).Joints.all, Width, Height,
                                                            Moved_Pose (Z), Moved_Lens (Z), 0.0);
                                       Row : Natural := R'First - 1;
                                       Q   : Natural := 0;
                                    begin
                                       for V of Sets (K).Views.all loop
                                          for I in 1 .. V.Pixels loop
                                             if V.Link (I) >= 2 then
                                                Q := Q + 1;
                                                declare
                                                   C : constant Positive := Classes (K) (Q);
                                                   Any : Boolean := False;
                                                begin
                                                   for J in 1 .. V.Link (I) loop
                                                      Sums (C) := Sums (C) + R (Row + J) ** 2;
                                                      Any := Any or else R (Row + J) /= 0.0;
                                                   end loop;
                                                   if Any then
                                                      Frees (C) := Frees (C) + V.Link (I) - 1;
                                                   end if;
                                                end;
                                                Row := Row + V.Link (I);
                                             end if;
                                          end loop;
                                       end loop;
                                    end;
                                 end loop;
                                 for C in 1 .. 3 loop
                                    Append (Text, " " & (case C is when 1 => "inside", when 2 => "edge",
                                                         when others => "beside") & " "
                                            & Image (Sums (C), 1) & " /" & Frees (C)'Image);
                                 end loop;
                                 Put_Line ("profile classes: focal " & Image (100.0 * E, 1) & " %:" & To_String (Text));
                              end;
                           end;
                        end loop;
                     end;
                  end;
               end;
            end;
         end if;
         --  FLOW_SWAP = "I,J" (BENCH): the truth's scatter with joints I and J of every arm's chain swapped in order
         --  (their axes and points as at rest, their readings and rates with them; the pixels a joint between them
         --  moves alone are left out as before): how far the eye's flows tell apart two joints' order that the arm's
         --  own eye does not (step 2's untold pairs, crossing axes near the base).
         if Ada.Environment_Variables.Exists ("FLOW_SWAP") then
            declare
               Text : constant String := Ada.Environment_Variables.Value ("FLOW_SWAP");
               Cut  : constant Natural := Ada.Strings.Fixed.Index (Text, ",");
               I    : constant Positive := Positive'Value (Text (Text'First .. Cut - 1));
               J    : constant Positive := Positive'Value (Text (Cut + 1 .. Text'Last));
               procedure Swap_All (K : Positive) is
                  T : constant Fit.Joint := Sets (K).Joints (I);
               begin
                  Sets (K).Joints (I) := Sets (K).Joints (J);
                  Sets (K).Joints (J) := T;
                  for V of Sets (K).Views.all loop
                     declare
                        R : constant Real := V.Readings (I);
                     begin
                        V.Readings (I) := V.Readings (J);
                        V.Readings (J) := R;
                     end;
                     for P in 1 .. V.Pixels loop
                        declare
                           R : constant Real := V.Rate (P, I);
                           S : constant Real := V.Sigma (P, I);
                        begin
                           V.Rate (P, I) := V.Rate (P, J);
                           V.Sigma (P, I) := V.Sigma (P, J);
                           V.Rate (P, J) := R;
                           V.Sigma (P, J) := S;
                        end;
                     end loop;
                  end loop;
               end Swap_All;
            begin
               for K in Sets'Range loop
                  Swap_All (K);
                  Flow.Judge_At (Sets (K).Views.all, Sets (K).Joints.all, Width, Height, Truth, True_Lens, Scatter,
                                 In_Front);
                  Put_Line ("at the truth, arm" & Sets (K).Number'Image & ", joints" & I'Image & " and" & J'Image
                            & " swapped: scatter " & Image (Scatter, 3) & ", in front" & In_Front'Image);
                  Swap_All (K);
               end loop;
            end;
         end if;
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
               & Image (Cam.Cy, 1) & "), aspect " & Image (Result.Lens.Fy / Result.Lens.Fx, 5) & " of "
               & Image (Cam.Fy / Cam.Fx, 5) & " (sigma " & Image (100.0 * EF.Sqrt (Result.Covariance (4, 4)), 2) & " %)");
            for K in Sets'Range loop
               declare
                  Off    : constant Real := abs Minus (Result.Poses (K).Translation, Truth.Translation);
                  Turned : constant Real :=
                    Driver.Numerics.Angle (Times (Transposed (Truth.Rotation), Result.Poses (K).Rotation));
                  O      : constant Natural := Flow.Terms_For (K - Sets'First + 1) - 6;
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
         --  FLOW_REFINE (BENCH): the eye placed again by the driver's own Flow.Refine_By_Scene from the world the arms'
         --  wrist eyes see at their sets' poses (and FLOW_SCENE_EXTRA more, drawn as the sets' are), with their frames,
         --  their lenses and the table as step 2 would hand them (the truth's), and set against the truth as the flow's
         --  fit was, with the arms' joints in the picture (the refined eye's places against the true eye's).
         if Ada.Environment_Variables.Exists ("FLOW_REFINE") then
            declare
               Extra  : constant Natural := Natural (Setting ("FLOW_SCENE_EXTRA", 0.0));
               Cam    : FW.Camera renames W.Cameras (Eye);
               Fixed  : Flow.Luma_Access := Picture;
               type Pictures_Access is access Flow.Wrist_Picture_Array;
               Pictures : array (Sets'Range) of Pictures_Access;
               --  Per picture, the pixels whose ray meets the table first (the study's: a place on a thing on the table
               --  is not where the table's plane puts it).
               type Mask_List is array (Positive range <>) of Flow.Pixel_Flags_Access;
               type Mask_List_Access is access Mask_List;
               Table_Masks : array (Sets'Range) of Mask_List_Access;
               --  Per picture, where each pixel's ray meets the world first (its point) and which solid it meets (the
               --  study's FLOW_STUDY_HIT: the true points of the things on the table, not the table's plane's alone).
               type Hit_Map is array (Positive range <>) of Vec3;
               type Hit_Map_Access is access Hit_Map;
               type Solid_Map is array (Positive range <>) of Natural;
               type Solid_Map_Access is access Solid_Map;
               type Hit_List is array (Positive range <>) of Hit_Map_Access;
               type Hit_List_Access is access Hit_List;
               type Solid_List is array (Positive range <>) of Solid_Map_Access;
               type Solid_List_Access is access Solid_List;
               Hits       : array (Sets'Range) of Hit_List_Access;
               Hit_Solids : array (Sets'Range) of Solid_List_Access;
               type Variants_Access is access Flow.Arm_Variant_Array;
               type Joints_Access is access Fit.Joint_Array;
               type Eyes_Access is access Flow.Rigid_Array;
               Worlds   : Flow.Arm_World_Array (Sets'Range);
               Joints_True : Real_Vectors.Vector;   --  every set's joints at its poses, u and v by turns, in the truth
               Refined  : Flow.Report := Result;
               Began_R  : constant Ada.Calendar.Time := Ada.Calendar.Clock;
               --  FLOW_WRISTS (BENCH) names a file of the wrist pictures and their world masks: read when it exists,
               --  written when it does not (the renders were most of a refinement's run, minutes a seed).
               package WIO renames Ada.Streams.Stream_IO;
               Wrists_Name : constant String :=
                 (if Ada.Environment_Variables.Exists ("FLOW_WRISTS")
                  then Ada.Environment_Variables.Value ("FLOW_WRISTS")
                  else "");
               Wrists_Read : constant Boolean := Wrists_Name /= "" and then Ada.Directories.Exists (Wrists_Name);
               Wrists_File : WIO.File_Type;
            begin
               if Wrists_Read then
                  WIO.Open (Wrists_File, WIO.In_File, Wrists_Name);
               elsif Wrists_Name /= "" then
                  WIO.Create (Wrists_File, WIO.Out_File, Wrists_Name);
               end if;
               for K in Sets'Range loop
                  declare
                     Group : Natural := 0;
                     Wrist : Natural := 0;
                  begin
                     for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                        if To_String (W.Groups (G).Name) = "arm" & Trim (Sets (K).Number) & "_joints" then
                           Group := G;
                        end if;
                     end loop;
                     for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
                        if W.Cameras (C).Link /= 0 and then W.Cameras (C).Arm = Sets (K).Number then
                           Wrist := C;
                        end if;
                     end loop;
                     declare
                        Links : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
                        N     : constant Natural := Natural (Links.Length);
                        Rest  : Real_Array (1 .. N);
                        Draws : Ada.Numerics.Float_Random.Generator;
                        Given : constant Natural := Sets (K).Views'Length;
                        --  the readings of each picture from the rest, and where the wrist eye stands at rest
                        Change    : Real_Array (1 .. N);
                        Rest_Pose : Rigid;
                        Changes   : array (1 .. Given + Extra) of Real_Array (1 .. N);
                        Variants  : Variants_Access;
                     begin
                        Ada.Numerics.Float_Random.Reset (Draws, Seed + K);
                        Pictures (K) := new Flow.Wrist_Picture_Array (1 .. Given + Extra);
                        Table_Masks (K) := new Mask_List (1 .. Given + Extra);
                        Hits (K) := new Hit_List (1 .. Given + Extra);
                        Hit_Solids (K) := new Solid_List (1 .. Given + Extra);
                        for J in 1 .. N loop
                           Rest (J) := W.Links (Links (Links.First_Index + J - 1)).Q;
                        end loop;
                        FW.Forward (W);
                        Rest_Pose := W.Cameras (Wrist).Pose;
                        for V in 1 .. Given + Extra loop
                           for J in 1 .. N loop
                              declare
                                 L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                              begin
                                 Change (J) :=
                                   (if V <= Given
                                    then Sets (K).Views (Sets (K).Views'First + V - 1).Readings (J)
                                    else (2.0 * Real (Ada.Numerics.Float_Random.Random (Draws)) - 1.0)
                                           * Swing / L.Unit);
                                 L.Q := Rest (J) + Change (J);
                              end;
                           end loop;
                           Changes (V) := Change;
                           FW.Forward (W);
                           if V <= Given then
                              for J in 1 .. N loop
                                 declare
                                    X  : constant Vec3 := W.Links (Links (Links.First_Index + J - 1)).Pose.Translation;
                                    Xc : constant Vec3 :=
                                      Transposed_Times (Truth.Rotation, Driver.Numerics.Minus (X, Truth.Translation));
                                 begin
                                    if Xc (3) > 0.0 then
                                       Joints_True.Append (Cam.Cx + Cam.Fx * Xc (1) / Xc (3));
                                       Joints_True.Append (Cam.Cy + Cam.Fy * Xc (2) / Xc (3));
                                    end if;
                                 end;
                              end loop;
                           end if;
                           declare
                              C    : aliased constant FW.Camera := W.Cameras (Wrist);
                              --  where the arm's joints as given put the eye (FLOW_ARM_ERROR), else where it is
                              Placed : constant Rigid :=
                                (if Ada.Environment_Variables.Exists ("FLOW_ARM_ERROR")
                                 then Fit.Eye_At (Sets (K).Joints.all, [for I in 1 .. N => I], Change) * Rest_Pose
                                 else C.Pose);
                              Rgb  : Rgb_Access := new Real_Array (1 .. 3 * C.Width * C.Height);
                              Luma : constant Flow.Luma_Access := new Real_Array (1 .. C.Width * C.Height);
                           begin
                              if Wrists_Read then
                                 Real_Array'Read (WIO.Stream (Wrists_File), Luma.all);
                              else
                                 Fake_Render.Render (W, C, Rgb.all);
                                 for P in Luma'Range loop
                                    Luma (P) := 0.299 * Rgb (Rgb'First + 3 * (P - 1))
                                      + 0.587 * Rgb (Rgb'First + 3 * (P - 1) + 1)
                                      + 0.114 * Rgb (Rgb'First + 3 * (P - 1) + 2);
                                 end loop;
                                 if Wrists_Name /= "" then
                                    Real_Array'Write (WIO.Stream (Wrists_File), Luma.all);
                                 end if;
                              end if;
                              Free (Rgb);
                              declare
                                 --  The world's pixels by the truth (step 2's lock-in tells them in the boot): the ray
                                 --  through the pixel meets a solid of the world first, not the arm's own.
                                 Mask : constant Flow.Pixel_Flags_Access :=
                                   new Flow.Pixel_Flags (1 .. C.Width * C.Height);
                              begin
                                 if Wrists_Read then
                                    Flow.Pixel_Flags'Read (WIO.Stream (Wrists_File), Mask.all);
                                 end if;
                                 Table_Masks (K) (V) := new Flow.Pixel_Flags (1 .. C.Width * C.Height);
                                 Hits (K) (V) := new Hit_Map (1 .. C.Width * C.Height);
                                 Hit_Solids (K) (V) := new Solid_Map (1 .. C.Width * C.Height);
                                 for Y in 0 .. C.Height - 1 loop
                                    for X in 0 .. C.Width - 1 loop
                                       declare
                                          Solid : Natural;
                                          Along : Real;
                                          Px    : constant Real := (Real (X) + 0.5 - C.Cx) / C.Fx;
                                          Py    : constant Real := (Real (Y) + 0.5 - C.Cy) / C.Fy;
                                          Dc    : constant Vec3 :=
                                            Times (C.Pose.Rotation, Vec3'[Px, Py, 1.0]);
                                          Dn    : constant Real := EF.Sqrt (Dc (1) ** 2 + Dc (2) ** 2 + Dc (3) ** 2);
                                       begin
                                          Fake_Render.Nearest (W, C, Real (X) + 0.5, Real (Y) + 0.5, Solid, Along);
                                          Table_Masks (K) (V) (Y * C.Width + X + 1) :=
                                            Solid > 0 and then To_String (W.Solids (Solid).Name) = "table";
                                          Hit_Solids (K) (V) (Y * C.Width + X + 1) :=
                                            (if Solid > 0 and then W.Solids (Solid).Link = 0 and then Along < Real'Last
                                             then Solid else 0);
                                          Hits (K) (V) (Y * C.Width + X + 1) :=
                                            (if Hit_Solids (K) (V) (Y * C.Width + X + 1) > 0
                                             then Plus (C.Pose.Translation, Scaled (Along / Dn, Dc)) else [0.0, 0.0, 0.0]);
                                       end;
                                    end loop;
                                 end loop;
                                 if not Wrists_Read then
                                 for Y in 0 .. C.Height - 1 loop
                                    for X in 0 .. C.Width - 1 loop
                                       declare
                                          Solid : Natural;
                                          Along : Real;
                                       begin
                                          Fake_Render.Nearest (W, C, Real (X) + 0.5, Real (Y) + 0.5, Solid, Along);
                                          Mask (Y * C.Width + X + 1) := Solid > 0 and then W.Solids (Solid).Link = 0;
                                       end;
                                    end loop;
                                 end loop;
                                    if Wrists_Name /= "" then
                                       Flow.Pixel_Flags'Write (WIO.Stream (Wrists_File), Mask.all);
                                    end if;
                                 end if;
                                 --  FLOW_TABLE_ONLY (a study): the world's pixels the table's alone, not the things on it.
                                 if Ada.Environment_Variables.Exists ("FLOW_TABLE_ONLY") then
                                    Mask.all := Table_Masks (K) (V).all;
                                 end if;
                                 Pictures (K) (V) :=
                                   (Luma => Luma, Width => C.Width, Height => C.Height, Eye => Placed, World => Mask);
                              end;
                           end;
                        end loop;
                        for J in 1 .. N loop
                           W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
                        end loop;
                        FW.Forward (W);
                        --  FLOW_ARM_ERROR: the joints' sigmas as the arm's variants, every term of every joint one
                        --  sigma either way (Arm_World.Variants), each with the pictures' eyes placed by its joints.
                        if Ada.Environment_Variables.Exists ("FLOW_ARM_ERROR") then
                           Variants := new Flow.Arm_Variant_Array (1 .. 2 * Fit.Joint_Terms * N);
                           declare
                              Tilt_E : constant Real := Arm_Error (1);
                              Move_E : constant Real := Arm_Error (2);
                              Gain_E : constant Real := Arm_Error (3);
                              Along  : constant Fit.Chain_Order := [for I in 1 .. N => I];
                           begin
                              for J in 1 .. N loop
                                 for T in Fit.Joint_Term loop
                                    for E in 1 .. 2 loop
                                       declare
                                          Js     : constant Joints_Access := new Fit.Joint_Array'(Sets (K).Joints.all);
                                          A      : Fit.Joint renames Js (Js'First + J - 1);
                                          Eyes   : constant Eyes_Access := new Flow.Rigid_Array (1 .. Given + Extra);
                                          Sign   : constant Real := (if E = 1 then -1.0 else 1.0);
                                          E1, E2 : Vec3;
                                       begin
                                          Fit.Across (A.W, E1, E2);
                                          case T is
                                             when Fit.Tilt_1  => A.W := Unit (Plus (A.W, Scaled (Sign * Tilt_E, E1)));
                                             when Fit.Tilt_2  => A.W := Unit (Plus (A.W, Scaled (Sign * Tilt_E, E2)));
                                             when Fit.Point_1 => A.P := Plus (A.P, Scaled (Sign * Move_E, E1));
                                             when Fit.Point_2 => A.P := Plus (A.P, Scaled (Sign * Move_E, E2));
                                             when Fit.Scale   => A.C := A.C * (1.0 + Sign * Gain_E);
                                          end case;
                                          for V in Eyes'Range loop
                                             Eyes (V) := Fit.Eye_At (Js.all, Along, Changes (V)) * Rest_Pose;
                                          end loop;
                                          Variants (2 * (Fit.Joint_Terms * (J - 1) + Fit.Joint_Term'Pos (T)) + E) :=
                                            (Joints => Js.all'Unchecked_Access, Eyes => Eyes.all'Unchecked_Access,
                                             Lens   => (Fx => W.Cameras (Wrist).Fx, Fy => W.Cameras (Wrist).Fy,
                                                        Cx => W.Cameras (Wrist).Cx, Cy => W.Cameras (Wrist).Cy,
                                                        K1 | K2 => 0.0),
                                             Table  => (Centre => [0.0, 0.0, W.Table_Z], Normal => [0.0, 0.0, 1.0],
                                                        Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
                                                        Offset_Sigma => 0.0, others => <>));
                                       end;
                                    end loop;
                                 end loop;
                              end loop;
                           end;
                        end if;
                        Worlds (K) :=
                          (Pictures => Pictures (K).all'Unchecked_Access,
                           Lens     => (Fx => W.Cameras (Wrist).Fx, Fy => W.Cameras (Wrist).Fy,
                                        Cx => W.Cameras (Wrist).Cx, Cy => W.Cameras (Wrist).Cy, K1 | K2 => 0.0),
                           Table    => (Centre => [0.0, 0.0, W.Table_Z], Normal => [0.0, 0.0, 1.0],
                                        Tangent_1 => [1.0, 0.0, 0.0], Tangent_2 => [0.0, 1.0, 0.0],
                                        Offset_Sigma => 0.0, others => <>),
                           Variants => (if Variants = null then null else Variants.all'Unchecked_Access));
                     end;
                  end;
               end loop;
               if Wrists_Name /= "" then
                  WIO.Close (Wrists_File);
               end if;
               --  FLOW_REFINE_FROM_TRUTH (a study): the refinement started at the true eye; FLOW_REFINE_NO_PRIOR: the
               --  flow's covariance made a million times its terms' units, its estimate no evidence.
               if Ada.Environment_Variables.Exists ("FLOW_REFINE_FROM_TRUTH") then
                  Refined.Lens := (Fx => Cam.Fx, Fy => Cam.Fy, Cx => Cam.Cx, Cy => Cam.Cy, K1 | K2 => 0.0);
                  for K in Sets'Range loop
                     Refined.Poses (K) := Truth;
                  end loop;
               end if;
               if Ada.Environment_Variables.Exists ("FLOW_REFINE_NO_PRIOR") then
                  for A in Refined.Covariance'Range (1) loop
                     for B in Refined.Covariance'Range (2) loop
                        Refined.Covariance (A, B) := (if A = B then 1.0E6 else 0.0);
                     end loop;
                  end loop;
               end if;
               --  FLOW_TRUE_ASPECT (a study): the refinement started at the true eye's aspect (its two focal lengths'
               --  ratio), which it fits with the other terms.
               if Ada.Environment_Variables.Exists ("FLOW_TRUE_ASPECT") then
                  Refined.Lens.Fy := Refined.Lens.Fx * Cam.Fy / Cam.Fx;
               end if;
               --  FLOW_PLACE_STUDY (a study): the aligner's own errors where the refinement asks it, the true eye's
               --  prediction given: every arm's picture's table points on the refinement's grid, predicted into the
               --  fixed picture by the true eye (and their neighbours, its warp), aligned as Refine_By_Scene aligns
               --  them, against where the true eye sees them. How far off, against what is stated, and how the errors
               --  share: their mean over all, over each sixth of the picture (a field), within one cell a patch wide
               --  of the fixed picture across pictures, and within one wrist picture.
               if Ada.Environment_Variables.Exists ("FLOW_PLACE_STUDY") then
                  declare
                     use type Flow.Pixel_Flags_Access;
                     Fixed_P : constant Driver.Alignment.Pyramid :=
                       Driver.Alignment.Pyramid_Of (Width, Height, Fixed.all);
                     Window  : constant Real :=
                       Driver.Alignment.Sigma_Of_Window (Driver.Alignment.Finest_Radius (Width, Height));
                     Side    : constant Positive := 2 * Positive'Max (1, Driver.Alignment.Half_Patch (Width, Height));
                     Cells_U : constant Positive := (Width + Side - 1) / Side;
                     Cells_V : constant Positive := (Height + Side - 1) / Side;
                     type Found_Place is record
                        Eu, Ev, Su, Sv : Real;
                        Cell, Picture  : Positive;
                        Sixth          : Positive;
                        Scale          : Real;   --  the prediction's warp's scale (the geometric mean of its stretches)
                        --  what the warp's curvature over the wrist patch would move the patch's middle by (half the
                        --  second differences a half patch either way, both directions, over three: the mean square of
                        --  a place in the patch against its half width squared)
                        Bu, Bv         : Real;
                        Arm            : Positive;   --  of the arms in order (Sets)
                        X              : Vec3;       --  the table's point
                        Cuv            : Real;       --  the stated covariance across and down
                        --  the place found again with the prediction a pixel off across (Uu, Vu) and down (Uv, Vv),
                        --  less the place found: when both were found (Pulled)
                        Pulled         : Boolean;
                        Pull_Uu, Pull_Vu, Pull_Uv, Pull_Vv : Real;
                        On_Table       : Boolean;   --  the wrist picture's ray there meets the table first
                     end record;
                     package Found_Vectors is new Ada.Containers.Vectors (Positive, Found_Place);
                     Found   : Found_Vectors.Vector;
                     Asked   : Natural := 0;
                     Picture_No : Natural := 0;
                     --  FLOW_STUDY_CENTRED (a study of the caches rendered before 10-21): a pixel's content the light about
                     --  its rays' mean place (Fake_Render.Raw_Shift off its centre): the places the truth gives moved by
                     --  it, in the wrist pictures and the fixed one alike.
                     Centred : constant Boolean := Ada.Environment_Variables.Exists ("FLOW_STUDY_CENTRED");
                     Ray_U : constant Real := (if Centred then Fake_Render.Raw_Shift (1) else 0.0);
                     Ray_V : constant Real := (if Centred then Fake_Render.Raw_Shift (2) else 0.0);
                     --  FLOW_STUDY_HIT (a study): every point the true one the wrist picture's ray meets (the table or a
                     --  thing on it), a patch asked only where its pixel and its neighbours meet one solid, and only where
                     --  the fixed eye sees that point (its own ray meets it first).
                     Hit_Mode      : constant Boolean := Ada.Environment_Variables.Exists ("FLOW_STUDY_HIT");
                     Current_Hits  : Hit_Map_Access;
                     Current_Solid : Solid_Map_Access;
                     Current_Width, Current_Height : Positive := 1;
                     Centre_Solid  : Natural := 0;
                     function True_Place (X : Vec3; U, V : out Real) return Boolean is
                        C : constant Vec3 := Transposed_Times (Truth.Rotation, Minus (X, Truth.Translation));
                     begin
                        U := Cam.Cx + Cam.Fx * C (1) / C (3) - Ray_U;
                        V := Cam.Cy + Cam.Fy * C (2) / C (3) - Ray_V;
                        if not (C (3) > 0.0 and then U in 0.0 .. Real (Width) and then V in 0.0 .. Real (Height)) then
                           return False;
                        elsif not Hit_Mode then
                           return True;
                        end if;
                        declare
                           Solid : Natural;
                           Along : Real;
                           Far   : constant Real := EF.Sqrt (C (1) ** 2 + C (2) ** 2 + C (3) ** 2);
                        begin
                           Fake_Render.Nearest (W, Cam, U + Ray_U, V + Ray_V, Solid, Along);
                           --  Seen when nothing lies before it on the fixed eye's line of sight (a millimetre's give).
                           return Along >= Far - 1.0E-3;
                        end;
                     end True_Place;
                     function On_Table (Eye : Rigid; L : Fit.Lens; U, V : Real; X : out Vec3) return Boolean is
                        D     : constant Vec3 := Times (Eye.Rotation, Fit.Ray (L, U + Ray_U, V + Ray_V));
                        S_Hit : constant Real := (if D (3) /= 0.0 then (W.Table_Z - Eye.Translation (3)) / D (3) else -1.0);
                     begin
                        if Hit_Mode then
                           declare
                              Px : constant Integer := Integer (Real'Floor (U));
                              Py : constant Integer := Integer (Real'Floor (V));
                           begin
                              X := [0.0, 0.0, 0.0];
                              if Current_Hits = null or else Px not in 0 .. Current_Width - 1
                                or else Py not in 0 .. Current_Height - 1
                              then
                                 return False;
                              end if;
                              X := Current_Hits (Py * Current_Width + Px + 1);
                              return Centre_Solid > 0 and then Current_Solid (Py * Current_Width + Px + 1) = Centre_Solid;
                           end;
                        end if;
                        X := Plus (Eye.Translation, Scaled (S_Hit, D));
                        return S_Hit > 0.0;
                     end On_Table;
                  begin
                     for K in Sets'Range loop
                        declare
                           Wl : constant Fit.Lens := Worlds (K).Lens;
                           Local_No : Natural := 0;   --  the picture's place among its arm's
                        begin
                           for P of Pictures (K).all loop
                              Picture_No := Picture_No + 1;
                              Local_No := Local_No + 1;
                              Current_Hits := Hits (K) (Local_No);
                              Current_Solid := Hit_Solids (K) (Local_No);
                              Current_Width := P.Width;
                              Current_Height := P.Height;
                              declare
                                 Pyr  : constant Driver.Alignment.Pyramid :=
                                   Driver.Alignment.Pyramid_Of (P.Width, P.Height, P.Luma.all);
                                 Grid : constant Positive := 2 * Positive'Max (1, Driver.Alignment.Half_Patch (P.Width, P.Height));
                                 Gu   : Natural := Grid / 2;
                              begin
                                 while Gu < P.Width - Grid / 2 loop
                                    declare
                                       Gv : Natural := Grid / 2;
                                    begin
                                       while Gv < P.Height - Grid / 2 loop
                                          declare
                                             U0 : constant Real := Real (Gu) + 0.5;
                                             V0 : constant Real := Real (Gv) + 0.5;
                                             X, Xu, Xv : Vec3;
                                             Tu, Tv, Uu, Uv, Vu, Vv : Real;
                                             Mx : constant Natural := Gu;
                                             My : constant Natural := Gv;
                                          begin
                                             Centre_Solid := (if Hit_Mode then Current_Solid (My * P.Width + Mx + 1) else 0);
                                             --  FLOW_STUDY_HIT=whole: the patch's every pixel on that one solid (no edge in it).
                                             if Hit_Mode and then Ada.Environment_Variables.Value ("FLOW_STUDY_HIT") = "whole" then
                                                for Dy in -Grid / 2 .. Grid / 2 loop
                                                   for Dx in -Grid / 2 .. Grid / 2 loop
                                                      if Gu + Dx not in 0 .. P.Width - 1 or else Gv + Dy not in 0 .. P.Height - 1
                                                        or else Current_Solid ((Gv + Dy) * P.Width + Gu + Dx + 1) /= Centre_Solid
                                                      then
                                                         Centre_Solid := 0;
                                                      end if;
                                                   end loop;
                                                end loop;
                                             end if;
                                             if (P.World = null or else P.World (My * P.Width + Mx + 1))
                                               and then On_Table (P.Eye, Wl, U0, V0, X)
                                               and then On_Table (P.Eye, Wl, U0 + 1.0, V0, Xu)
                                               and then On_Table (P.Eye, Wl, U0, V0 + 1.0, Xv)
                                               and then True_Place (X, Tu, Tv) and then True_Place (Xu, Uu, Uv)
                                               and then True_Place (Xv, Vu, Vv)
                                             then
                                                Asked := Asked + 1;
                                                declare
                                                   Query : constant Driver.Alignment.Prediction :=
                                                     (From   => (U => U0, V => V0),
                                                      To     => (U => Tu, V => Tv),
                                                      Linear => (UU => Uu - Tu, UV => Vu - Tu, VU => Uv - Tv, VV => Vv - Tv),
                                                      Cov    => (UU => Window ** 2, UV => 0.0, VV => Window ** 2),
                                                      Linear_Sigma => Real'Last);
                                                   --  FLOW_PLACE_STUDY=bend: the true warp to second order (its linear
                                                   --  part by central differences, its bend), as the refinement asks.
                                                   Bent : constant Boolean :=
                                                     Ada.Environment_Variables.Value ("FLOW_PLACE_STUDY") = "bend";
                                                   Pu, Pv : array (-1 .. 1, -1 .. 1) of Real := [others => [others => 0.0]];
                                                   All_In : Boolean := True;
                                                   function Bent_Query return Driver.Alignment.Prediction is
                                                     ((From => Query.From, To => Query.To,
                                                       Linear => (UU => (Pu (1, 0) - Pu (-1, 0)) / 2.0,
                                                                  UV => (Pu (0, 1) - Pu (0, -1)) / 2.0,
                                                                  VU => (Pv (1, 0) - Pv (-1, 0)) / 2.0,
                                                                  VV => (Pv (0, 1) - Pv (0, -1)) / 2.0),
                                                       Cov => Query.Cov, Linear_Sigma => Real'Last));
                                                   function The_Bend return Driver.Alignment.Bend_Part is
                                                     ((U_II => (Pu (1, 0) + Pu (-1, 0) - 2.0 * Pu (0, 0)) / 2.0,
                                                       U_IJ => (Pu (1, 1) - Pu (1, -1) - Pu (-1, 1) + Pu (-1, -1)) / 4.0,
                                                       U_JJ => (Pu (0, 1) + Pu (0, -1) - 2.0 * Pu (0, 0)) / 2.0,
                                                       V_II => (Pv (1, 0) + Pv (-1, 0) - 2.0 * Pv (0, 0)) / 2.0,
                                                       V_IJ => (Pv (1, 1) - Pv (1, -1) - Pv (-1, 1) + Pv (-1, -1)) / 4.0,
                                                       V_JJ => (Pv (0, 1) + Pv (0, -1) - 2.0 * Pv (0, 0)) / 2.0));
                                                   function Answer_Of return Driver.Alignment.Answer is
                                                   begin
                                                      for I in -1 .. 1 loop
                                                         for J in -1 .. 1 loop
                                                            declare
                                                               Xn : Vec3;
                                                            begin
                                                               All_In := All_In
                                                                 and then On_Table (P.Eye, Wl, U0 + Real (I), V0 + Real (J), Xn)
                                                                 and then True_Place (Xn, Pu (I, J), Pv (I, J));
                                                            end;
                                                         end loop;
                                                      end loop;
                                                      return (if Bent and then All_In
                                                              then Driver.Alignment.Align_At_Scale
                                                                     (Pyr, Fixed_P, Bent_Query, Bend => The_Bend)
                                                              else Driver.Alignment.Align_At_Scale (Pyr, Fixed_P, Query));
                                                   end Answer_Of;
                                                   A : constant Driver.Alignment.Answer := Answer_Of;
                                                   --  The same asked with the prediction off by Du across and Dv down.
                                                   function Shifted (Du, Dv : Real) return Driver.Alignment.Answer is
                                                      Q : Driver.Alignment.Prediction := (if Bent and then All_In then Bent_Query else Query);
                                                   begin
                                                      Q.To := (U => Q.To.U + Du, V => Q.To.V + Dv);
                                                      return (if Bent and then All_In
                                                              then Driver.Alignment.Align_At_Scale (Pyr, Fixed_P, Q, Bend => The_Bend)
                                                              else Driver.Alignment.Align_At_Scale (Pyr, Fixed_P, Q));
                                                   end Shifted;
                                                   Hw : constant Real := Real (Grid / 2);
                                                   Curv_U, Curv_V : Real := 0.0;
                                                   use type Driver.Alignment.Verdict;
                                                begin
                                                   if A.Verdict = Driver.Alignment.Found and then A.Narrow
                                                     and then A.Cov.UU > 0.0 and then A.Cov.VV > 0.0
                                                   then
                                                      declare
                                                         Xa, Xb, Xc, Xd : Vec3;
                                                         Au, Av, Bu1, Bv1, Cu1, Cv1, Du1, Dv1 : Real;
                                                      begin
                                                         if On_Table (P.Eye, Wl, U0 + Hw, V0, Xa) and then On_Table (P.Eye, Wl, U0 - Hw, V0, Xb)
                                                           and then On_Table (P.Eye, Wl, U0, V0 + Hw, Xc)
                                                           and then On_Table (P.Eye, Wl, U0, V0 - Hw, Xd)
                                                           and then True_Place (Xa, Au, Av) and then True_Place (Xb, Bu1, Bv1)
                                                           and then True_Place (Xc, Cu1, Cv1) and then True_Place (Xd, Du1, Dv1)
                                                         then
                                                            Curv_U := ((Au + Bu1 - 2.0 * Tu) + (Cu1 + Du1 - 2.0 * Tu)) / 2.0 / 3.0;
                                                            Curv_V := ((Av + Bv1 - 2.0 * Tv) + (Cv1 + Dv1 - 2.0 * Tv)) / 2.0 / 3.0;
                                                         end if;
                                                      end;
                                                      declare
                                                         Ans_U  : constant Driver.Alignment.Answer := Shifted (1.0, 0.0);
                                                         Ans_V  : constant Driver.Alignment.Answer := Shifted (0.0, 1.0);
                                                         Pulled : constant Boolean :=
                                                           Ans_U.Verdict = Driver.Alignment.Found and then Ans_V.Verdict = Driver.Alignment.Found;
                                                      begin
                                                         Found.Append
                                                           (Found_Place'(Eu => A.To.U - Tu, Ev => A.To.V - Tv,
                                                             Su => EF.Sqrt (A.Cov.UU), Sv => EF.Sqrt (A.Cov.VV),
                                                             Cell => Natural'Min (Cells_V - 1, Natural (Real'Floor (Tv)) / Side) * Cells_U
                                                                     + Natural'Min (Cells_U - 1, Natural (Real'Floor (Tu)) / Side) + 1,
                                                             Picture => Picture_No,
                                                             Scale => EF.Sqrt (abs ((Uu - Tu) * (Vv - Tv) - (Vu - Tu) * (Uv - Tv))),
                                                             Bu => Curv_U, Bv => Curv_V,
                                                             Arm => K - Sets'First + 1, X => X, Cuv => A.Cov.UV,
                                                             Sixth => Natural'Min (1, Natural (Real'Floor (2.0 * Tv / Real (Height)))) * 3
                                                                      + Natural'Min (2, Natural (Real'Floor (3.0 * Tu / Real (Width)))) + 1,
                                                             On_Table => Table_Masks (K) (Local_No) (My * P.Width + Mx + 1),
                                                             Pulled => Pulled,
                                                             Pull_Uu => (if Pulled then Ans_U.To.U - A.To.U else 0.0),
                                                             Pull_Vu => (if Pulled then Ans_U.To.V - A.To.V else 0.0),
                                                             Pull_Uv => (if Pulled then Ans_V.To.U - A.To.U else 0.0),
                                                             Pull_Vv => (if Pulled then Ans_V.To.V - A.To.V else 0.0)));
                                                      end;
                                                   end if;
                                                end;
                                             end if;
                                          end;
                                          Gv := Gv + Grid;
                                       end loop;
                                    end;
                                    Gu := Gu + Grid;
                                 end loop;
                              end;
                           end loop;
                        end;
                     end loop;
                     declare
                        N : constant Natural := Natural (Found.Length);
                        Su_All, Mu, Mv, Ssq : Real := 0.0;
                        Stated : Real_Array (1 .. Natural'Max (1, N));
                        Err    : Real_Array (1 .. Natural'Max (1, 2 * N));
                     begin
                        for I in 1 .. N loop
                           Mu := Mu + Found (I).Eu;
                           Mv := Mv + Found (I).Ev;
                           Ssq := Ssq + Found (I).Eu ** 2 + Found (I).Ev ** 2;
                           Su_All := Su_All + Found (I).Su ** 2 + Found (I).Sv ** 2;
                           Stated (I) := EF.Sqrt ((Found (I).Su ** 2 + Found (I).Sv ** 2) / 2.0);
                           Err (2 * I - 1) := Found (I).Eu;
                           Err (2 * I) := Found (I).Ev;
                        end loop;
                        Put_Line ("place study: the truth's places moved by the rays' mean place, (" & Image (-Ray_U, 4) & ", "
                                  & Image (-Ray_V, 4) & ") px");
                        if N > 0 then
                           Put_Line ("place study: " & N'Image & " of" & Asked'Image & " found; errors rms "
                                     & Image (EF.Sqrt (Ssq / Real (2 * N)), 3) & " px (robust "
                                     & Image (Driver.Stats.Robust_Sigma (Err (1 .. 2 * N)), 3) & "), mean ("
                                     & Image (Mu / Real (N), 4) & ", " & Image (Mv / Real (N), 4) & ") px, stated rms "
                                     & Image (EF.Sqrt (Su_All / Real (2 * N)), 3) & " px, stated median "
                                     & Image (Driver.Stats.Median (Stated (1 .. N)), 3) & " px");
                           --  The errors by what is stated: the quarters of the stated sigmas.
                           declare
                              Q2 : constant Real := Driver.Stats.Median (Stated (1 .. N));
                              Lower, Upper : Real_Array (1 .. N);
                              Nl, Nu : Natural := 0;
                              Q1, Q3 : Real;
                           begin
                              for I in 1 .. N loop
                                 if Stated (I) < Q2 then
                                    Nl := Nl + 1;
                                    Lower (Nl) := Stated (I);
                                 else
                                    Nu := Nu + 1;
                                    Upper (Nu) := Stated (I);
                                 end if;
                              end loop;
                              Q1 := (if Nl > 0 then Driver.Stats.Median (Lower (1 .. Nl)) else Q2);
                              Q3 := (if Nu > 0 then Driver.Stats.Median (Upper (1 .. Nu)) else Q2);
                              for B in 1 .. 4 loop
                                 declare
                                    Sum : Real := 0.0;
                                    C   : Natural := 0;
                                    Lo  : constant Real := (case B is when 1 => 0.0, when 2 => Q1, when 3 => Q2, when others => Q3);
                                    Hi  : constant Real := (case B is when 1 => Q1, when 2 => Q2, when 3 => Q3, when others => Real'Last);
                                 begin
                                    for I in 1 .. N loop
                                       if Stated (I) >= Lo and then Stated (I) < Hi then
                                          Sum := Sum + Found (I).Eu ** 2 + Found (I).Ev ** 2;
                                          C := C + 1;
                                       end if;
                                    end loop;
                                    if C > 0 then
                                       Put_Line ("place study: stated " & Image (Lo, 3) & " to " & Image (Hi, 3) & " px: errors rms "
                                                 & Image (EF.Sqrt (Sum / Real (2 * C)), 3) & " px over" & C'Image);
                                    end if;
                                 end;
                              end loop;
                           end;
                           --  By the warp's scale (the fixed picture's pixels per wrist picture's), its quarters: the
                           --  mean error and the robust spread.
                           declare
                              Sc : Real_Array (1 .. N);
                           begin
                              for I in 1 .. N loop
                                 Sc (I) := Found (I).Scale;
                              end loop;
                              declare
                                 M2 : constant Real := Driver.Stats.Median (Sc);
                                 Lo_S, Hi_S : Real_Array (1 .. N);
                                 Nl, Nh : Natural := 0;
                              begin
                                 for I in 1 .. N loop
                                    if Sc (I) < M2 then
                                       Nl := Nl + 1;
                                       Lo_S (Nl) := Sc (I);
                                    else
                                       Nh := Nh + 1;
                                       Hi_S (Nh) := Sc (I);
                                    end if;
                                 end loop;
                                 declare
                                    M1 : constant Real := (if Nl > 0 then Driver.Stats.Median (Lo_S (1 .. Nl)) else M2);
                                    M3 : constant Real := (if Nh > 0 then Driver.Stats.Median (Hi_S (1 .. Nh)) else M2);
                                    Edges : constant array (1 .. 5) of Real := [0.0, M1, M2, M3, Real'Last];
                                 begin
                                    for B in 1 .. 4 loop
                                       declare
                                          Su_B, Sv_B : Real := 0.0;
                                          E : Real_Array (1 .. 2 * N);
                                          C : Natural := 0;
                                       begin
                                          for I in 1 .. N loop
                                             if Sc (I) >= Edges (B) and then Sc (I) < Edges (B + 1) then
                                                C := C + 1;
                                                Su_B := Su_B + Found (I).Eu;
                                                Sv_B := Sv_B + Found (I).Ev;
                                                E (2 * C - 1) := Found (I).Eu;
                                                E (2 * C) := Found (I).Ev;
                                             end if;
                                          end loop;
                                          if C > 0 then
                                             Put_Line ("place study: warp scale " & Image (Edges (B), 3) & " to "
                                                       & Image (Real'Min (Edges (B + 1), 99.0), 3) & ": mean ("
                                                       & Image (Su_B / Real (C), 4) & ", " & Image (Sv_B / Real (C), 4)
                                                       & ") px, robust " & Image (Driver.Stats.Robust_Sigma (E (1 .. 2 * C)), 3)
                                                       & " px over" & C'Image);
                                          end if;
                                       end;
                                    end loop;
                                 end;
                              end;
                           end;
                           --  The errors against what the warp's curvature would move the patch's middle by: the
                           --  slope through the origin and the correlation, both coordinates together.
                           declare
                              Sxy, Sxx, Syy : Real := 0.0;
                           begin
                              for I in 1 .. N loop
                                 if abs Found (I).Eu < 1.0 and then abs Found (I).Ev < 1.0 then
                                    Sxy := Sxy + Found (I).Bu * Found (I).Eu + Found (I).Bv * Found (I).Ev;
                                    Sxx := Sxx + Found (I).Bu ** 2 + Found (I).Bv ** 2;
                                    Syy := Syy + Found (I).Eu ** 2 + Found (I).Ev ** 2;
                                 end if;
                              end loop;
                              if Sxx > 0.0 and then Syy > 0.0 then
                                 Put_Line ("place study: the errors (under a pixel) against the warp's curvature: slope "
                                           & Image (Sxy / Sxx, 3) & ", correlation " & Image (Sxy / EF.Sqrt (Sxx * Syy), 3)
                                           & ", the curvature's rms " & Image (EF.Sqrt (Sxx / Real (2 * N)), 4) & " px");
                              end if;
                           end;
                           --  The places on the table and those on the things on it (their points are not where the
                           --  table's plane puts them): how many, their errors' mean, rms and robust spread.
                           for On in Boolean loop
                              declare
                                 E : Real_Array (1 .. 2 * N);
                                 C : Natural := 0;
                                 Su_O, Sv_O, Sq : Real := 0.0;
                              begin
                                 for I in 1 .. N loop
                                    if Found (I).On_Table = On then
                                       C := C + 1;
                                       E (2 * C - 1) := Found (I).Eu;
                                       E (2 * C) := Found (I).Ev;
                                       Su_O := Su_O + Found (I).Eu;
                                       Sv_O := Sv_O + Found (I).Ev;
                                       Sq := Sq + Found (I).Eu ** 2 + Found (I).Ev ** 2;
                                    end if;
                                 end loop;
                                 if C > 0 then
                                    Put_Line ("place study: " & (if On then "on the table" else "on a thing") & ":" & C'Image
                                              & " places, mean (" & Image (Su_O / Real (C), 4) & ", " & Image (Sv_O / Real (C), 4)
                                              & ") px, rms " & Image (EF.Sqrt (Sq / Real (2 * C)), 3) & ", robust "
                                              & Image (Driver.Stats.Robust_Sigma (E (1 .. 2 * C)), 3) & " px");
                                 end if;
                              end;
                           end loop;
                           --  The field: the mean error over each sixth of the fixed picture.
                           for Sx in 1 .. 6 loop
                              declare
                                 Su_S, Sv_S : Real := 0.0;
                                 C : Natural := 0;
                              begin
                                 for I in 1 .. N loop
                                    if Found (I).Sixth = Sx then
                                       Su_S := Su_S + Found (I).Eu;
                                       Sv_S := Sv_S + Found (I).Ev;
                                       C := C + 1;
                                    end if;
                                 end loop;
                                 if C > 0 then
                                    Put_Line ("place study: sixth" & Sx'Image & ": mean (" & Image (Su_S / Real (C), 4) & ", "
                                              & Image (Sv_S / Real (C), 4) & ") px over" & C'Image);
                                 end if;
                              end;
                           end loop;
                           --  What errors share: the correlation of two places' errors in the same cell (other
                           --  pictures), in the same wrist picture (other cells), and in neither.
                           declare
                              Same_Cell, Same_Pic, Neither : Real := 0.0;
                              N_Cell, N_Pic, N_Neither : Natural := 0;
                              Var : constant Real := Ssq / Real (2 * N);
                           begin
                              for I in 1 .. N loop
                                 for J in I + 1 .. N loop
                                    declare
                                       Prod : constant Real := (Found (I).Eu * Found (J).Eu + Found (I).Ev * Found (J).Ev) / 2.0;
                                    begin
                                       if Found (I).Cell = Found (J).Cell and then Found (I).Picture /= Found (J).Picture then
                                          Same_Cell := Same_Cell + Prod;
                                          N_Cell := N_Cell + 1;
                                       elsif Found (I).Picture = Found (J).Picture and then Found (I).Cell /= Found (J).Cell then
                                          Same_Pic := Same_Pic + Prod;
                                          N_Pic := N_Pic + 1;
                                       elsif Found (I).Cell /= Found (J).Cell then
                                          Neither := Neither + Prod;
                                          N_Neither := N_Neither + 1;
                                       end if;
                                    end;
                                 end loop;
                              end loop;
                              Put_Line ("place study: the errors' correlation in the same cell "
                                        & Image ((if N_Cell > 0 then Same_Cell / Real (N_Cell) else 0.0) / Var, 3) & " ("
                                        & N_Cell'Image & " pairs), in the same picture "
                                        & Image ((if N_Pic > 0 then Same_Pic / Real (N_Pic) else 0.0) / Var, 3) & " ("
                                        & N_Pic'Image & "), in neither "
                                        & Image ((if N_Neither > 0 then Neither / Real (N_Neither) else 0.0) / Var, 3) & " ("
                                        & N_Neither'Image & ")");
                           end;
                           --  What the places' errors do to the eye: its terms (the refinement's: its focal length's
                           --  logarithm, its principal point, and per arm a turn of the eye about its own axes and a
                           --  shift of its centre in the arm's frame) fitted to the places found, linearly about the
                           --  truth, each place whitened by the covariance it states (whole, or its diagonal alone, or
                           --  whole with a floor of FLOOR squared on both axes), the places off by more than Z robust
                           --  sigmas left out until none is; in the directions the places tell more than the flows (the
                           --  refinement's Split against the flows' covariance, Refined's), the eye's shift's chi square
                           --  against what the places state, and each group's share of its length in sigmas against the
                           --  share's sigma were the errors as stated. The floor: what makes the kept places' whitened
                           --  errors one sigma on the whole.
                           declare
                              type Flag_Set is array (Positive range <>) of Boolean;
                              Arms_N : constant Positive := Sets'Length;
                              G_N    : constant Positive := 3 + 6 * Arms_N;
                              Groups : constant Positive := Arms_N + 4 + 6 + 4 + 2;
                              function Quarter (X, Q1, Q2, Q3 : Real) return Positive is
                                (if X < Q1 then 1 elsif X < Q2 then 2 elsif X < Q3 then 3 else 4);
                              procedure Quartiles (X : Real_Array; Q1, Q2, Q3 : out Real) is
                                 M      : constant Real := Driver.Stats.Median (X);
                                 Lo, Hi : Real_Array (1 .. X'Length);
                                 Nl, Nh : Natural := 0;
                              begin
                                 for V of X loop
                                    if V < M then
                                       Nl := Nl + 1;
                                       Lo (Nl) := V;
                                    else
                                       Nh := Nh + 1;
                                       Hi (Nh) := V;
                                    end if;
                                 end loop;
                                 Q2 := M;
                                 Q1 := (if Nl > 0 then Driver.Stats.Median (Lo (1 .. Nl)) else M);
                                 Q3 := (if Nh > 0 then Driver.Stats.Median (Hi (1 .. Nh)) else M);
                              end Quartiles;
                              Sc_All : Real_Array (1 .. N);
                              S1, S2, S3, T1, T2, T3 : Real;
                              Flows_L  : Real_Matrix (1 .. G_N, 1 .. G_N);
                              Flows_Ok : Boolean;
                              --  A place's whitening: the Cholesky factor of what it states, the floor added.
                              procedure Whiten (F : Found_Place; Whole : Boolean; Floor : Real; A_S, B_S, C_S : out Real;
                                                Ok : out Boolean) is
                                 Cuu : constant Real := F.Su ** 2 + Floor;
                                 Cvv : constant Real := F.Sv ** 2 + Floor;
                                 Cuv : constant Real := (if Whole then F.Cuv else 0.0);
                              begin
                                 Ok := Cuu > 0.0 and then Cuu * Cvv > Cuv ** 2;
                                 A_S := EF.Sqrt (Real'Max (Cuu, 0.0));
                                 B_S := (if Ok then Cuv / A_S else 0.0);
                                 C_S := (if Ok then EF.Sqrt (Cvv - B_S ** 2) else 1.0);
                              end Whiten;
                              --  The places off by more than Z robust sigmas, whitened so, left out until none is.
                              procedure Trim (Whole : Boolean; Floor : Real; Keep : out Flag_Set; Kept_N : out Natural) is
                                 Wu, Wv : Real_Array (1 .. N) := [others => 0.0];
                              begin
                                 Kept_N := 0;
                                 for I in 1 .. N loop
                                    declare
                                       A_S, B_S, C_S : Real;
                                       Ok : Boolean;
                                    begin
                                       Whiten (Found (I), Whole, Floor, A_S, B_S, C_S, Ok);
                                       Keep (I) := Ok;
                                       if Ok then
                                          Wu (I) := Found (I).Eu / A_S;
                                          Wv (I) := (Found (I).Ev - B_S * Wu (I)) / C_S;
                                          Kept_N := Kept_N + 1;
                                       end if;
                                    end;
                                 end loop;
                                 loop
                                    declare
                                       R       : Real_Array (1 .. 2 * Natural'Max (1, Kept_N)) := [others => 0.0];
                                       M       : Natural := 0;
                                       Dropped : Natural := 0;
                                    begin
                                       for I in 1 .. N loop
                                          if Keep (I) then
                                             R (M + 1) := Wu (I);
                                             R (M + 2) := Wv (I);
                                             M := M + 2;
                                          end if;
                                       end loop;
                                       declare
                                          Sigma : constant Real := Driver.Stats.Robust_Sigma (R (1 .. Natural'Max (1, M)));
                                       begin
                                          for I in 1 .. N loop
                                             if Keep (I) and then EF.Sqrt (Wu (I) ** 2 + Wv (I) ** 2)
                                               > Driver.Conventions.Z * Sigma * EF.Sqrt (2.0)
                                             then
                                                Keep (I) := False;
                                                Dropped := Dropped + 1;
                                             end if;
                                          end loop;
                                       end;
                                       Kept_N := Kept_N - Dropped;
                                       exit when Dropped = 0 or else Kept_N = 0;
                                    end;
                                 end loop;
                              end Trim;
                              --  The floor: the kept places' whitened errors' mean square two (both axes) with it,
                              --  by bisection (none when they are within it already).
                              function Floor_Of return Real is
                                 Keep   : Flag_Set (1 .. N);
                                 Kept_N : Natural;
                                 function Mean_Square (Floor : Real) return Real is
                                    Sum : Real := 0.0;
                                 begin
                                    for I in 1 .. N loop
                                       if Keep (I) then
                                          declare
                                             A_S, B_S, C_S : Real;
                                             Ok : Boolean;
                                          begin
                                             Whiten (Found (I), True, Floor, A_S, B_S, C_S, Ok);
                                             declare
                                                Wu : constant Real := Found (I).Eu / A_S;
                                                Wv : constant Real := (Found (I).Ev - B_S * Wu) / C_S;
                                             begin
                                                Sum := Sum + Wu ** 2 + Wv ** 2;
                                             end;
                                          end;
                                       end if;
                                    end loop;
                                    return Sum / Real (Natural'Max (1, Kept_N));
                                 end Mean_Square;
                                 Lo, Hi : Real := 0.0;
                              begin
                                 Trim (True, 0.0, Keep, Kept_N);
                                 if Kept_N = 0 or else Mean_Square (0.0) <= 2.0 then
                                    return 0.0;
                                 end if;
                                 for I in 1 .. N loop
                                    if Keep (I) then
                                       Hi := Real'Max (Hi, Found (I).Eu ** 2 + Found (I).Ev ** 2);
                                    end if;
                                 end loop;
                                 for Step in 1 .. Real'Machine_Mantissa loop
                                    declare
                                       Mid : constant Real := (Lo + Hi) / 2.0;
                                    begin
                                       if Mean_Square (Mid) > 2.0 then
                                          Lo := Mid;
                                       else
                                          Hi := Mid;
                                       end if;
                                    end;
                                 end loop;
                                 return (Lo + Hi) / 2.0;
                              end Floor_Of;
                              procedure Influence (Whole : Boolean; Floor : Real) is
                                 Jw       : Real_Matrix (1 .. 2 * N, 1 .. G_N) := [others => [others => 0.0]];
                                 Ew       : Real_Vector (1 .. 2 * N) := [others => 0.0];
                                 Keep     : Flag_Set (1 .. N);
                                 Of_Group : array (1 .. N, 1 .. 5) of Positive := [others => [others => 1]];
                                 Kept_N   : Natural;
                              begin
                                 Trim (Whole, Floor, Keep, Kept_N);
                                 for I in 1 .. N loop
                                    declare
                                       F     : constant Found_Place := Found (I);
                                       C     : constant Vec3 :=
                                         Transposed_Times (Truth.Rotation, Minus (F.X, Truth.Translation));
                                       Xc    : constant Real := C (1) / C (3);
                                       Yc    : constant Real := C (2) / C (3);
                                       --  The place's derivatives in the point in the eye's frame.
                                       Du    : constant Vec3 := [Cam.Fx / C (3), 0.0, -Cam.Fx * Xc / C (3)];
                                       Dv    : constant Vec3 := [0.0, Cam.Fy / C (3), -Cam.Fy * Yc / C (3)];
                                       --  The point's derivatives in the eye's turn (the point across each axis).
                                       Turns : constant array (1 .. 3) of Vec3 :=
                                         [[0.0, C (3), -C (2)], [-C (3), 0.0, C (1)], [C (2), -C (1), 0.0]];
                                       Base  : constant Natural := 3 + 6 * (F.Arm - 1);
                                       A_S, B_S, C_S : Real;
                                       Ok    : Boolean;
                                       Ju, Jv : Real_Vector (1 .. G_N) := [others => 0.0];
                                    begin
                                       Whiten (F, Whole, Floor, A_S, B_S, C_S, Ok);
                                       Ju (1) := Cam.Fx * Xc;
                                       Jv (1) := Cam.Fy * Yc;
                                       Ju (2) := 1.0;
                                       Jv (3) := 1.0;
                                       for J in 1 .. 3 loop
                                          Ju (Base + J) :=
                                            Du (1) * Turns (J) (1) + Du (2) * Turns (J) (2) + Du (3) * Turns (J) (3);
                                          Jv (Base + J) :=
                                            Dv (1) * Turns (J) (1) + Dv (2) * Turns (J) (2) + Dv (3) * Turns (J) (3);
                                          --  The centre's shift along the arm's axis J moves the point the other way
                                          --  in the eye's frame (row J of the eye's turn).
                                          Ju (Base + 3 + J) := -(Du (1) * Truth.Rotation (J, 1) + Du (2) * Truth.Rotation (J, 2)
                                                                 + Du (3) * Truth.Rotation (J, 3));
                                          Jv (Base + 3 + J) := -(Dv (1) * Truth.Rotation (J, 1) + Dv (2) * Truth.Rotation (J, 2)
                                                                 + Dv (3) * Truth.Rotation (J, 3));
                                       end loop;
                                       if Keep (I) then
                                          for T in 1 .. G_N loop
                                             Jw (2 * I - 1, T) := Ju (T) / A_S;
                                             Jw (2 * I, T) := (Jv (T) - B_S * Ju (T) / A_S) / C_S;
                                          end loop;
                                          Ew (2 * I - 1) := F.Eu / A_S;
                                          Ew (2 * I) := (F.Ev - B_S * F.Eu / A_S) / C_S;
                                       end if;
                                       Of_Group (I, 1) := F.Arm;
                                       Of_Group (I, 2) := Arms_N + Quarter (F.Scale, S1, S2, S3);
                                       Of_Group (I, 3) := Arms_N + 4 + F.Sixth;
                                       Of_Group (I, 4) := Arms_N + 10 + Quarter (Stated (I), T1, T2, T3);
                                       Of_Group (I, 5) := Arms_N + 14 + (if F.On_Table then 1 else 2);
                                    end;
                                 end loop;
                                 declare
                                    type Normals is array (1 .. Groups) of Real_Matrix (1 .. G_N, 1 .. G_N);
                                    type Sides is array (1 .. Groups) of Real_Vector (1 .. G_N);
                                    Nm     : Real_Matrix (1 .. G_N, 1 .. G_N) := [others => [others => 0.0]];
                                    Bv     : Real_Vector (1 .. G_N) := [others => 0.0];
                                    Ng     : Normals := [others => [others => [others => 0.0]]];
                                    Bg     : Sides := [others => [others => 0.0]];
                                    Count  : array (1 .. Groups) of Natural := [others => 0];
                                    Sum_Sq : Real := 0.0;
                                 begin
                                    for I in 1 .. N loop
                                       if Keep (I) then
                                          for Row in 2 * I - 1 .. 2 * I loop
                                             Sum_Sq := Sum_Sq + Ew (Row) ** 2;
                                             for P in 1 .. G_N loop
                                                Bv (P) := Bv (P) + Jw (Row, P) * Ew (Row);
                                                for Q in 1 .. G_N loop
                                                   Nm (P, Q) := Nm (P, Q) + Jw (Row, P) * Jw (Row, Q);
                                                end loop;
                                             end loop;
                                             for K in 1 .. 5 loop
                                                declare
                                                   Gk : constant Positive := Of_Group (I, K);
                                                begin
                                                   for P in 1 .. G_N loop
                                                      Bg (Gk) (P) := Bg (Gk) (P) + Jw (Row, P) * Ew (Row);
                                                      for Q in 1 .. G_N loop
                                                         Ng (Gk) (P, Q) := Ng (Gk) (P, Q) + Jw (Row, P) * Jw (Row, Q);
                                                      end loop;
                                                   end loop;
                                                end;
                                             end loop;
                                          end loop;
                                          for K in 1 .. 5 loop
                                             Count (Of_Group (I, K)) := Count (Of_Group (I, K)) + 1;
                                          end loop;
                                       end if;
                                    end loop;
                                    --  The split: B = L' N L, L the flows' covariance's factor; the places' directions
                                    --  D = L U where B's eigenvalue is one or more.
                                    declare
                                       Bm, U  : Real_Matrix (1 .. G_N, 1 .. G_N);
                                       Lam    : Real_Vector (1 .. G_N);
                                       D      : Real_Matrix (1 .. G_N, 1 .. G_N) := [others => [others => 0.0]];
                                       Shift  : Real_Vector (1 .. G_N) := [others => 0.0];
                                       Chi    : Real := 0.0;
                                       Places : Natural := 0;
                                    begin
                                       Bm := Transpose (Flows_L) * Nm * Flows_L;
                                       Driver.Numerics.Dense.Small_Eigensystem (Bm, Lam, U);
                                       D := Flows_L * U;
                                       for K in 1 .. G_N loop
                                          if Lam (K) >= 1.0 then
                                             declare
                                                Along : Real := 0.0;
                                             begin
                                                for P in 1 .. G_N loop
                                                   Along := Along + D (P, K) * Bv (P);
                                                end loop;
                                                Chi := Chi + Along ** 2 / Lam (K);
                                                for P in 1 .. G_N loop
                                                   Shift (P) := Shift (P) + D (P, K) * Along / Lam (K);
                                                end loop;
                                                Places := Places + 1;
                                             end;
                                          end if;
                                       end loop;
                                       Put_Line ("place study: the eye the places tell ("
                                                 & (if not Whole then "the stated diagonal"
                                                    elsif Floor > 0.0 then "the stated covariance and a floor of "
                                                      & Image (EF.Sqrt (Floor), 3) & " px"
                                                    else "the stated covariance")
                                                 & "): chi square " & Image (Chi, 1) & " over the" & Places'Image
                                                 & " directions they tell; the whitened errors' mean square "
                                                 & Image (Sum_Sq / Real (Natural'Max (1, 2 * Kept_N)), 3) & ";"
                                                 & Kept_N'Image & " of" & N'Image & " kept");
                                       for Gk in 1 .. Groups loop
                                          if Count (Gk) > 0 and then Chi > 0.0 then
                                             declare
                                                Name : constant String :=
                                                  (if Gk <= Arms_N then "arm" & Gk'Image
                                                   elsif Gk <= Arms_N + 4 then "scale quarter" & Positive'Image (Gk - Arms_N)
                                                   elsif Gk <= Arms_N + 10 then "sixth" & Positive'Image (Gk - Arms_N - 4)
                                                   elsif Gk <= Arms_N + 14 then "stated quarter" & Positive'Image (Gk - Arms_N - 10)
                                                   elsif Gk = Arms_N + 15 then "on the table"
                                                   else "on a thing");
                                                Share, Var : Real := 0.0;
                                             begin
                                                for P in 1 .. G_N loop
                                                   Share := Share + Shift (P) * Bg (Gk) (P);
                                                   for Q in 1 .. G_N loop
                                                      Var := Var + Shift (P) * Ng (Gk) (P, Q) * Shift (Q);
                                                   end loop;
                                                end loop;
                                                Put_Line ("place study:   " & Name & ":" & Count (Gk)'Image
                                                          & " places, share " & Image (Share / EF.Sqrt (Chi), 2)
                                                          & " of " & Image (EF.Sqrt (Chi), 2) & " (sigma "
                                                          & Image (EF.Sqrt (Var / Chi), 2) & ")");
                                             end;
                                          end if;
                                       end loop;
                                    end;
                                 end;
                              end Influence;
                           begin
                              for I in 1 .. N loop
                                 Sc_All (I) := Found (I).Scale;
                              end loop;
                              Quartiles (Sc_All, S1, S2, S3);
                              Quartiles (Stated (1 .. N), T1, T2, T3);
                              Driver.Numerics.Dense.Cholesky (Refined.Covariance, Flows_L, Flows_Ok);
                              if Flows_Ok then
                                 Influence (Whole => False, Floor => 0.0);
                                 Influence (Whole => True, Floor => 0.0);
                                 declare
                                    Floor : constant Real := Floor_Of;
                                 begin
                                    if Floor > 0.0 then
                                       Influence (Whole => True, Floor => Floor);
                                    end if;
                                 end;
                              end if;
                              --  The aligner's pull: how far a place found moves when the prediction is a pixel
                              --  off across or down, over the places found all three times (a share of the pixel:
                              --  nought, the answer the picture's alone; one, the prediction's).
                              declare
                                 Puu, Pvu, Puv, Pvv : Real := 0.0;
                                 C : Natural := 0;
                                 Ru, Rv : Real_Array (1 .. N);
                              begin
                                 for I in 1 .. N loop
                                    if Found (I).Pulled then
                                       C := C + 1;
                                       Puu := Puu + Found (I).Pull_Uu;
                                       Pvu := Pvu + Found (I).Pull_Vu;
                                       Puv := Puv + Found (I).Pull_Uv;
                                       Pvv := Pvv + Found (I).Pull_Vv;
                                       Ru (C) := Found (I).Pull_Uu;
                                       Rv (C) := Found (I).Pull_Vv;
                                    end if;
                                 end loop;
                                 if C > 0 then
                                    Put_Line ("place study: the pull of a pixel's prediction: across "
                                              & Image (Puu / Real (C), 3) & " (down " & Image (Pvu / Real (C), 3)
                                              & "), down " & Image (Pvv / Real (C), 3) & " (across "
                                              & Image (Puv / Real (C), 3) & "), medians " & Image (Driver.Stats.Median (Ru (1 .. C)), 3)
                                              & " and " & Image (Driver.Stats.Median (Rv (1 .. C)), 3) & ", over" & C'Image);
                                 end if;
                              end;
                           end;
                        end if;
                     end;
                  end;
               end if;
               Flow.Refine_By_Scene (In_Arms, Worlds, Fixed.all, Width, Height, Refined);
               Put_Line ("the aligner's work in the refinement: " & Driver.Alignment.Work_Report);
               declare
                  Worst : Real := 0.0;
                  I     : Natural := Joints_True.First_Index;
                  Line  : Unbounded_String;
               begin
                  --  The refined eye's places of the joints against the true eye's (both arms' sets in order).
                  for K in Sets'Range loop
                     declare
                        Group : Natural := 0;
                     begin
                        for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                           if To_String (W.Groups (G).Name) = "arm" & Trim (Sets (K).Number) & "_joints" then
                              Group := G;
                           end if;
                        end loop;
                        declare
                           Links : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
                           N     : constant Natural := Natural (Links.Length);
                           Rest  : Real_Array (1 .. N);
                           P     : constant Rigid := Refined.Poses (K);
                        begin
                           for J in 1 .. N loop
                              Rest (J) := W.Links (Links (Links.First_Index + J - 1)).Q;
                           end loop;
                           for V in Sets (K).Views'Range loop
                              for J in 1 .. N loop
                                 W.Links (Links (Links.First_Index + J - 1)).Q :=
                                   Rest (J) + Sets (K).Views (V).Readings (J);
                              end loop;
                              FW.Forward (W);
                              for J in 1 .. N loop
                                 declare
                                    X  : constant Vec3 := W.Links (Links (Links.First_Index + J - 1)).Pose.Translation;
                                    Xt : constant Vec3 :=
                                      Transposed_Times (Truth.Rotation, Driver.Numerics.Minus (X, Truth.Translation));
                                    Xc : constant Vec3 :=
                                      Transposed_Times (P.Rotation, Driver.Numerics.Minus (X, P.Translation));
                                 begin
                                    if Xt (3) > 0.0 then
                                       if Xc (3) > 0.0 and then I + 1 <= Joints_True.Last_Index
                                         and then Joints_True (I) in 0.0 .. Real (Width)
                                         and then Joints_True (I + 1) in 0.0 .. Real (Height)
                                       then
                                          Worst := Real'Max
                                            (Worst, EF.Sqrt ((Refined.Lens.Cx + Refined.Lens.Fx * Xc (1) / Xc (3)
                                                              - Joints_True (I)) ** 2
                                                             + (Refined.Lens.Cy + Refined.Lens.Fy * Xc (2) / Xc (3)
                                                                - Joints_True (I + 1)) ** 2));
                                       end if;
                                       I := I + 2;
                                    end if;
                                 end;
                              end loop;
                           end loop;
                           for J in 1 .. N loop
                              W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
                           end loop;
                           FW.Forward (W);
                        end;
                     end;
                  end loop;
                  Line := To_Unbounded_String
                    ("refined seed" & Seed'Image & ": " & (if Refined.Determined then "TOLD" else "untold")
                     & ", focal " & Image (Refined.Lens.Fx, 1) & " ("
                     & Image (100.0 * (Refined.Lens.Fx / Cam.Fx - 1.0), 2)
                     & " %), principal point (" & Image (Refined.Lens.Cx, 1) & ", "
                     & Image (Refined.Lens.Cy, 1) & ") of ("
                     & Image (Cam.Cx, 1) & ", " & Image (Cam.Cy, 1) & "), aspect "
                     & Image (Refined.Lens.Fy / Refined.Lens.Fx, 5) & " of " & Image (Cam.Fy / Cam.Fx, 5));
                  --  The refined eye's terms against the truth over the covariance it states (the logarithm of the
                  --  focal length, the principal point, each arm's turn about the camera's own axes, its centre): the
                  --  chi square, about as many as the terms when the covariance is honest, and the focal length's and
                  --  the principal point's own sigmas.
                  declare
                     G      : constant Positive := Refined.Count;
                     E      : Real_Vector (1 .. G) := [others => 0.0];
                     Lower  : Real_Matrix (1 .. G, 1 .. G);
                     Pd     : Boolean;
                  begin
                     E (1) := EF.Log (Refined.Lens.Fx / Cam.Fx);
                     E (2) := Refined.Lens.Cx - Cam.Cx;
                     E (3) := Refined.Lens.Cy - Cam.Cy;
                     E (4) := EF.Log ((Refined.Lens.Fy / Refined.Lens.Fx) / (Cam.Fy / Cam.Fx));
                     for K in Sets'Range loop
                        declare
                           O    : constant Natural := Flow.Terms_For (K - Sets'First + 1) - 6;
                           P    : constant Rigid := Refined.Poses (K);
                           Turn : constant Vec3 :=
                             Driver.Numerics.Log (Times (Transposed (Truth.Rotation), P.Rotation));
                        begin
                           for X in 1 .. 3 loop
                              E (O + X) := Turn (X);
                              E (O + 3 + X) := P.Translation (X) - Truth.Translation (X);
                           end loop;
                        end;
                     end loop;
                     Driver.Numerics.Dense.Cholesky (Refined.Covariance, Lower, Pd);
                     if Pd then
                        declare
                           Y   : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
                           Chi : Real := 0.0;
                        begin
                           for X in 1 .. G loop
                              Chi := Chi + E (X) * Y (X);
                           end loop;
                           Put_Line ("refined seed" & Seed'Image & ": the eye's terms against its own covariance: "
                                     & "chi square " & Image (Chi, 1) & " over" & G'Image & "; focal "
                                     & Image (E (1) / EF.Sqrt (Refined.Covariance (1, 1)), 2) & " sigmas, principal "
                                     & "point " & Image (E (2) / EF.Sqrt (Refined.Covariance (2, 2)), 2) & " "
                                     & Image (E (3) / EF.Sqrt (Refined.Covariance (3, 3)), 2) & " sigmas, aspect "
                                     & Image (E (4) / EF.Sqrt (Refined.Covariance (4, 4)), 2) & " sigmas");
                           --  The error along each axis of the covariance, over that axis's sigma, the three largest:
                           --  which combinations of the terms the covariance states too little of (each axis by its
                           --  sigma and its three largest terms: 1 the focal length's logarithm, 2 and 3 the principal
                           --  point, 4 the aspect's logarithm, then per arm 3 of turn and 3 of centre).
                           declare
                              Values  : Real_Vector (1 .. G);
                              Vectors : Real_Matrix (1 .. G, 1 .. G);
                              Z       : Real_Vector (1 .. G);
                              Done    : array (1 .. G) of Boolean := [others => False];
                              --  How far the eye's terms moved by D move the table's points (a grid 4 cm apart the
                              --  true eye sees) in the picture, at most.
                              function Moves (D : Real_Vector) return Real is
                                 Worst : Real := 0.0;
                                 Fm    : constant Real := Refined.Lens.Fx * EF.Exp (D (1));
                              begin
                                 for K in Sets'Range loop
                                    declare
                                       O  : constant Natural := Flow.Terms_For (K - Sets'First + 1) - 6;
                                       P  : constant Rigid := Refined.Poses (K);
                                       Pm : constant Rigid :=
                                         (Rotation    =>
                                            Times (P.Rotation, Exp (Vec3'[D (O + 1), D (O + 2), D (O + 3)])),
                                          Translation =>
                                            [P.Translation (1) + D (O + 4), P.Translation (2) + D (O + 5),
                                             P.Translation (3) + D (O + 6)]);
                                    begin
                                       for I in -10 .. 10 loop
                                          for J in -5 .. 20 loop
                                             declare
                                                Xw : constant Vec3 := [0.04 * Real (I), 0.04 * Real (J), W.Table_Z];
                                                C0 : constant Vec3 :=
                                                  Transposed_Times (P.Rotation, Minus (Xw, P.Translation));
                                                C1 : constant Vec3 :=
                                                  Transposed_Times (Pm.Rotation, Minus (Xw, Pm.Translation));
                                                Ct : constant Vec3 :=
                                                  Transposed_Times (Truth.Rotation, Minus (Xw, Truth.Translation));
                                                Tu : constant Real :=
                                                  (if Ct (3) > 0.0 then Cam.Cx + Cam.Fx * Ct (1) / Ct (3) else -1.0);
                                                Tv : constant Real :=
                                                  (if Ct (3) > 0.0 then Cam.Cy + Cam.Fy * Ct (2) / Ct (3) else -1.0);
                                             begin
                                                if C0 (3) > 0.0 and then C1 (3) > 0.0 and then Tu in 0.0 .. Real (Width)
                                                  and then Tv in 0.0 .. Real (Height)
                                                then
                                                   Worst := Real'Max
                                                     (Worst, EF.Sqrt ((D (2) + Fm * C1 (1) / C1 (3)
                                                                            - Refined.Lens.Fx * C0 (1) / C0 (3)) ** 2
                                                                      + (D (3) + Fm * EF.Exp (D (4)) * Refined.Lens.Fy
                                                                         / Refined.Lens.Fx * C1 (2) / C1 (3)
                                                                         - Refined.Lens.Fy * C0 (2) / C0 (3)) ** 2));
                                                end if;
                                             end;
                                          end loop;
                                       end loop;
                                    end;
                                 end loop;
                                 return Worst;
                              end Moves;
                           begin
                              Driver.Numerics.Dense.Small_Eigensystem (Refined.Covariance, Values, Vectors);
                              for I in 1 .. G loop
                                 Z (I) := 0.0;
                                 for X in 1 .. G loop
                                    Z (I) := Z (I) + Vectors (X, I) * E (X);
                                 end loop;
                                 Z (I) := (if Values (I) > 0.0 then Z (I) / EF.Sqrt (Values (I)) else Real'Last);
                              end loop;
                              for Rank in 1 .. 3 loop
                                 declare
                                    Best : Natural := 0;
                                 begin
                                    for I in 1 .. G loop
                                       if not Done (I) and then (Best = 0 or else abs Z (I) > abs Z (Best)) then
                                          Best := I;
                                       end if;
                                    end loop;
                                    Done (Best) := True;
                                    declare
                                       Line : Unbounded_String := To_Unbounded_String
                                         ("refined seed" & Seed'Image & ": axis" & Rank'Image & ": "
                                          & Image (Z (Best), 2) & " sigmas, its sigma "
                                          & Real'Image (EF.Sqrt (Real'Max (0.0, Values (Best)))) & ", terms");
                                       Used : array (1 .. G) of Boolean := [others => False];
                                    begin
                                       for Term in 1 .. 3 loop
                                          declare
                                             Top : Natural := 0;
                                          begin
                                             for X in 1 .. G loop
                                                if not Used (X)
                                                  and then (Top = 0
                                                            or else abs Vectors (X, Best) > abs Vectors (Top, Best))
                                                then
                                                   Top := X;
                                                end if;
                                             end loop;
                                             Used (Top) := True;
                                             Append (Line, Top'Image & " (" & Image (Vectors (Top, Best), 2) & ")");
                                          end;
                                       end loop;
                                       declare
                                          D : Real_Vector (1 .. G);
                                       begin
                                          for X in 1 .. G loop
                                             D (X) := Z (Best) * EF.Sqrt (Real'Max (0.0, Values (Best)))
                                               * Vectors (X, Best);
                                          end loop;
                                          Append (Line, "; it moves the table's points by up to " & Image (Moves (D), 3)
                                                  & " px");
                                       end;
                                       Put_Line (To_String (Line));
                                    end;
                                 end;
                              end loop;
                           end;
                        end;
                     else
                        Put_Line ("refined seed" & Seed'Image & ": the covariance it states is not definite");
                     end if;
                  end;
                  --  The table as each arm's refined eye sees it against the true eye: its normal's angle and its
                  --  distance from the eye (a plane's points tell these; they leave what is along the table and about
                  --  its normal).
                  for K in Sets'Range loop
                     declare
                        N_W  : constant Vec3 := [0.0, 0.0, 1.0];
                        On_W : constant Vec3 := [0.0, 0.0, W.Table_Z];
                        P    : constant Rigid := Refined.Poses (K);
                        Nf   : constant Vec3 := Transposed_Times (P.Rotation, N_W);
                        Nt   : constant Vec3 := Transposed_Times (Truth.Rotation, N_W);
                        Df   : constant Real := Dot (N_W, Driver.Numerics.Minus (On_W, P.Translation));
                        Dt   : constant Real := Dot (N_W, Driver.Numerics.Minus (On_W, Truth.Translation));
                        --  the eye's centre moved along the table and its turn about the table's normal
                        Shift : constant Vec3 := Driver.Numerics.Minus (P.Translation, Truth.Translation);
                        Along : constant Real :=
                          EF.Sqrt (Real'Max (0.0, Dot (Shift, Shift) - Dot (Shift, N_W) ** 2));
                     begin
                        Put_Line ("refined arm" & Sets (K).Number'Image & ": the table's normal off by "
                                  & Image (180.0 / Ada.Numerics.Pi * EF.Arccos (Real'Min (1.0, Dot (Nf, Nt))), 2)
                                  & " deg, its distance off by " & Image (1000.0 * (Df - Dt), 1)
                                  & " mm; the centre along the"
                                  & " table " & Image (1000.0 * Along, 1) & " mm");
                     end;
                  end loop;
                  --  Points over the table at heights of nought, 5, 10 and 30 cm (a grid 2 cm apart over the part of
                  --  the table the true eye sees): how far each arm's refined eye puts them from the true eye's places,
                  --  at most.
                  for K in Sets'Range loop
                     declare
                        P      : constant Rigid := Refined.Poses (K);
                        Line_H : Unbounded_String;
                     begin
                        for H of Real_Array'[0.0, 0.05, 0.1, 0.3] loop
                           declare
                              Worst : Real := 0.0;
                              Seen  : Natural := 0;
                           begin
                              for I in -50 .. 50 loop
                                 for J in -50 .. 50 loop
                                    declare
                                       X  : constant Vec3 := [0.02 * Real (I), 0.02 * Real (J), W.Table_Z + H];
                                       Xt : constant Vec3 := Transposed_Times
                                         (Truth.Rotation, Driver.Numerics.Minus (X, Truth.Translation));
                                       Xc : constant Vec3 := Transposed_Times
                                         (P.Rotation, Driver.Numerics.Minus (X, P.Translation));
                                    begin
                                       if Xt (3) > 0.0 and then Xc (3) > 0.0 then
                                          declare
                                             Tu : constant Real := Cam.Cx + Cam.Fx * Xt (1) / Xt (3);
                                             Tv : constant Real := Cam.Cy + Cam.Fy * Xt (2) / Xt (3);
                                             Fu : constant Real := Refined.Lens.Cx + Refined.Lens.Fx * Xc (1) / Xc (3);
                                             Fv : constant Real := Refined.Lens.Cy + Refined.Lens.Fy * Xc (2) / Xc (3);
                                          begin
                                             if Tu in 0.0 .. Real (Width) and then Tv in 0.0 .. Real (Height) then
                                                Seen := Seen + 1;
                                                Worst := Real'Max (Worst, EF.Sqrt ((Fu - Tu) ** 2 + (Fv - Tv) ** 2));
                                             end if;
                                          end;
                                       end if;
                                    end;
                                 end loop;
                              end loop;
                              Append (Line_H, " at" & Natural'Image (Natural (100.0 * H)) & " cm "
                                      & Image (Worst, 2) & " px");
                           end;
                        end loop;
                        Put_Line ("refined arm" & Sets (K).Number'Image & ": the table's points at most"
                                  & To_String (Line_H));
                     end;
                  end loop;
                  for K in Sets'Range loop
                     Append (Line, "; arm" & Sets (K).Number'Image & " centre off by "
                             & Image (1000.0 * abs Minus (Refined.Poses (K).Translation, Truth.Translation), 1)
                             & " mm, turn off by "
                             & Image (180.0 / Ada.Numerics.Pi
                                      * Driver.Numerics.Angle (Times (Transposed (Truth.Rotation),
                                                                      Refined.Poses (K).Rotation)), 2) & " deg");
                  end loop;
                  Append (Line, "; the arms' joints at most " & Image (Worst, 2)
                          & " px from the true eye's places, spread "
                          & Image (Refined.Spread, 2) & " px, "
                          & Image (Real (Ada.Calendar.Clock - Began_R), 1) & " s"
                          & (if Refined.Determined then "" else "; " & To_String (Refined.Why)));
                  Put_Line (To_String (Line));
               end;
               for K in Sets'Range loop
                  for V in Pictures (K)'Range loop
                     declare
                        L : Flow.Luma_Access := Pictures (K) (V).Luma;
                     begin
                        Free (L);
                     end;
                  end loop;
               end loop;
               Free (Fixed);
            end;
         end if;
         --  FLOW_SCENE = S (BENCH): the eye placed again from the world the arms' own eyes saw, as step 2 measures it
         --  (where the points of the table and of what stands on it lie, and how they look from the wrist): each arm's
         --  wrist eye at each pose of its set; a grid of its picture's points (a patch apart) that lie on the world's
         --  solids, each predicted into the fixed eye by the arms' fit (Result: the place, and the warp the solid makes
         --  there, from the next pixels' points), within S px either way, and aligned across the scales
         --  (Driver.Alignment.Align_At_Scale); then the eye's lens and pose fitted to the places found narrowly
         --  (Gauss-Newton on a central-difference Jacobian, the places off by more than Z robust sigmas left out until
         --  none is), twice, each from where the last left the eye. Against the truth: how many were found, how far
         --  their places lie from the true eye's, and the eye.
         if Ada.Environment_Variables.Exists ("FLOW_SCENE") then
            declare
               Window    : Real := Setting ("FLOW_SCENE", 8.0);
               --  FLOW_SCENE_EXTRA = E: E more poses of each arm for its wrist eye to see the world from, drawn as
               --  the sets' are, the same every round.
               Extra     : constant Natural := Natural (Setting ("FLOW_SCENE_EXTRA", 0.0));
               Cam       : FW.Camera renames W.Cameras (Eye);
               True_Lens : constant Fit.Lens :=
                 (Fx => Cam.Fx, Fy => Cam.Fy, Cx => Cam.Cx, Cy => Cam.Cy, K1 | K2 => 0.0);
               Fixed     : Flow.Luma_Access := Picture;
               Fixed_Pyr : constant Driver.Alignment.Pyramid := Driver.Alignment.Pyramid_Of (Width, Height, Fixed.all);
               type Match is record
                  X      : Vec3;
                  U, V   : Real;
                  Su, Sv : Real;
               end record;
               package Match_Vectors is new Ada.Containers.Vectors (Positive, Match);
               use type Driver.Alignment.Verdict;
               Lens : Fit.Lens := Result.Lens;
               Pose : Rigid := Result.Poses (Result.Poses'First);
               Step : constant Real := EF."**" (Real'Model_Epsilon, 1.0 / 3.0);
               function Project (L : Fit.Lens; P : Rigid; X : Vec3; U, V : out Real) return Boolean is
                  Xc : constant Vec3 := Transposed_Times (P.Rotation, Driver.Numerics.Minus (X, P.Translation));
               begin
                  U := 0.0;
                  V := 0.0;
                  if Xc (3) <= 0.0 then
                     return False;
                  end if;
                  U := L.Cx + L.Fx * Xc (1) / Xc (3);
                  V := L.Cy + L.Fy * Xc (2) / Xc (3);
                  return U >= 0.0 and then U < Real (Width) and then V >= 0.0 and then V < Real (Height);
               end Project;
               --  The point of the world a wrist eye's pixel sees, when it is a world's solid's.
               function World_Point (C : FW.Camera; U, V : Real; X : out Vec3; Solid : out Natural) return Boolean is
                  Along : Real;
                  Px    : constant Real := (U - C.Cx) / C.Fx;
                  Py    : constant Real := (V - C.Cy) / C.Fy;
                  Pn    : constant Real := EF.Sqrt (Px * Px + Py * Py + 1.0);
               begin
                  Fake_Render.Nearest (W, C, U, V, Solid, Along);
                  X := [0.0, 0.0, 0.0];
                  if Solid = 0 or else Along >= Real'Last or else W.Solids (Solid).Link /= 0 then
                     return False;
                  end if;
                  X := C.Pose.Translation + Along * Times (C.Pose.Rotation, Vec3'[Px / Pn, Py / Pn, 1.0 / Pn]);
                  return True;
               end World_Point;
               --  A point of the world a wrist eye's picture shows: the picture, its place in it, the point, and the
               --  points the next pixels across and down show.
               type Sight is record
                  View      : Positive;
                  U0, V0    : Real;
                  X, Xu, Xv : Vec3;
               end record;
               package Sight_Vectors is new Ada.Containers.Vectors (Positive, Sight);
               use type Driver.Alignment.Pyramid;
               package Pyramid_Vectors is new Ada.Containers.Vectors (Positive, Driver.Alignment.Pyramid);
               package Point_Vectors is new Ada.Containers.Vectors (Positive, Vec3, Driver.Numerics.Arrays."=");
               Sights    : Sight_Vectors.Vector;
               Views     : Pyramid_Vectors.Vector;
               Joints_At : Point_Vectors.Vector;   --  the arms' joints at every pose of their sets, in the world
               --  FLOW_SCENE_ROUNDS: at most so many rounds; they end when a round moves the arms' joints in the
               --  picture by no more than the fit's own spread there.
               Rounds      : constant Positive := Positive (Setting ("FLOW_SCENE_ROUNDS", 8.0));
               Last_Spread : Real := Real'Last;
               Prev_U, Prev_V : Real_Vectors.Vector;
            begin
               --  The wrist eyes' pictures and the points of the world in them, once.
               for K in Sets'Range loop
                  declare
                     Group : Natural := 0;
                     Wrist : Natural := 0;
                  begin
                     for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                        if To_String (W.Groups (G).Name) = "arm" & Trim (Sets (K).Number) & "_joints" then
                           Group := G;
                        end if;
                     end loop;
                     for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
                        if W.Cameras (C).Link /= 0 and then W.Cameras (C).Arm = Sets (K).Number then
                           Wrist := C;
                        end if;
                     end loop;
                     if Group /= 0 and then Wrist /= 0 then
                        declare
                           Links : FW.Index_Vectors.Vector renames W.Groups (Group).Joints;
                           N     : constant Natural := Natural (Links.Length);
                           Rest  : Real_Array (1 .. N);
                           Draws : Ada.Numerics.Float_Random.Generator;
                           Given : constant Natural := Sets (K).Views'Length;
                        begin
                           Ada.Numerics.Float_Random.Reset (Draws, Seed + K);
                           for J in 1 .. N loop
                              Rest (J) := W.Links (Links (Links.First_Index + J - 1)).Q;
                           end loop;
                           for V in 1 .. Given + Extra loop
                              for J in 1 .. N loop
                                 declare
                                    L : FW.Link renames W.Links (Links (Links.First_Index + J - 1));
                                 begin
                                    L.Q := Rest (J)
                                      + (if V <= Given
                                         then Sets (K).Views (Sets (K).Views'First + V - 1).Readings (J)
                                         else (2.0 * Real (Ada.Numerics.Float_Random.Random (Draws)) - 1.0)
                                                * Swing / L.Unit);
                                 end;
                              end loop;
                              FW.Forward (W);
                              if V <= Given then
                                 for J in 1 .. N loop
                                    Joints_At.Append (W.Links (Links (Links.First_Index + J - 1)).Pose.Translation);
                                 end loop;
                              end if;
                              declare
                                 C    : aliased constant FW.Camera := W.Cameras (Wrist);
                                 Rgb  : Rgb_Access := new Real_Array (1 .. 3 * C.Width * C.Height);
                                 Luma : Real_Array (1 .. C.Width * C.Height);
                                 Grid : constant Positive := 2 * Driver.Alignment.Half_Patch (C.Width, C.Height);
                                 Gu   : Natural := Grid / 2;
                              begin
                                 Fake_Render.Render (W, C, Rgb.all);
                                 for P in Luma'Range loop
                                    Luma (P) := 0.299 * Rgb (Rgb'First + 3 * (P - 1))
                                      + 0.587 * Rgb (Rgb'First + 3 * (P - 1) + 1)
                                      + 0.114 * Rgb (Rgb'First + 3 * (P - 1) + 2);
                                 end loop;
                                 Free (Rgb);
                                 Views.Append (Driver.Alignment.Pyramid_Of (C.Width, C.Height, Luma));
                                 while Gu < C.Width - Grid / 2 loop
                                    declare
                                       Gv : Natural := Grid / 2;
                                    begin
                                       while Gv < C.Height - Grid / 2 loop
                                          declare
                                             U0 : constant Real := Real (Gu) + 0.5;
                                             V0 : constant Real := Real (Gv) + 0.5;
                                             X, Xu, Xv : Vec3;
                                             S, Su, Sv : Natural;
                                          begin
                                             if World_Point (C, U0, V0, X, S)
                                               and then World_Point (C, U0 + 1.0, V0, Xu, Su)
                                               and then World_Point (C, U0, V0 + 1.0, Xv, Sv)
                                               and then Su = S and then Sv = S
                                             then
                                                Sights.Append (Sight'(View => Views.Last_Index, U0 => U0, V0 => V0,
                                                                      X => X, Xu => Xu, Xv => Xv));
                                             end if;
                                          end;
                                          Gv := Gv + Grid;
                                       end loop;
                                    end;
                                    Gu := Gu + Grid;
                                 end loop;
                              end;
                           end loop;
                           for J in 1 .. N loop
                              W.Links (Links (Links.First_Index + J - 1)).Q := Rest (J);
                           end loop;
                           FW.Forward (W);
                        end;
                     end if;
                  end;
               end loop;
               Put_Line ("scene:" & Views.Length'Image & " wrist pictures," & Sights.Length'Image
                         & " points of the world in them");
               for Round in 1 .. Rounds loop
                  declare
                     Found : Match_Vectors.Vector;
                     Asked : Natural := 0;
                  begin
                     --  Every point aligned at once (Driver.Parallel_For: each writes its own slot), from plain copies
                     --  of the points and the pictures.
                     declare
                        Count : constant Natural := Natural (Sights.Length);
                        type Sight_Array is array (Positive range <>) of Sight;
                        type Sight_Access is access Sight_Array;
                        procedure Free is new Ada.Unchecked_Deallocation (Sight_Array, Sight_Access);
                        type Pyramid_Array is array (Positive range <>) of Driver.Alignment.Pyramid;
                        type Pyramid_Access is access Pyramid_Array;
                        procedure Free is new Ada.Unchecked_Deallocation (Pyramid_Array, Pyramid_Access);
                        type Slot is record
                           Asked, Got   : Boolean := False;
                           U, V, Su, Sv : Real := 0.0;
                        end record;
                        type Slot_Array is array (Positive range <>) of Slot;
                        type Slot_Access is access Slot_Array;
                        procedure Free is new Ada.Unchecked_Deallocation (Slot_Array, Slot_Access);
                        All_Sights : Sight_Access := new Sight_Array (1 .. Natural'Max (1, Count));
                        All_Views  : Pyramid_Access := new Pyramid_Array (1 .. Natural'Max (1, Natural (Views.Length)));
                        Slots      : Slot_Access := new Slot_Array (1 .. Natural'Max (1, Count));
                        procedure One (I : Positive) is
                           S : Sight renames All_Sights (I);
                           Tu, Tv, Uu, Uv, Vu, Vv : Real;
                        begin
                           if Project (Lens, Pose, S.X, Tu, Tv) and then Project (Lens, Pose, S.Xu, Uu, Uv)
                             and then Project (Lens, Pose, S.Xv, Vu, Vv)
                           then
                              declare
                                 Query : constant Driver.Alignment.Prediction :=
                                   (From   => (U => S.U0, V => S.V0),
                                    To     => (U => Tu, V => Tv),
                                    Linear => (UU => Uu - Tu, UV => Vu - Tu, VU => Uv - Tv, VV => Vv - Tv),
                                    Cov    => (UU => Window ** 2, UV => 0.0, VV => Window ** 2),
                                    Linear_Sigma => Real'Last);
                                 A : constant Driver.Alignment.Answer :=
                                   Driver.Alignment.Align_At_Scale (All_Views (S.View), Fixed_Pyr, Query);
                              begin
                                 Slots (I).Asked := True;
                                 if A.Verdict = Driver.Alignment.Found and then A.Narrow
                                   and then A.Cov.UU > 0.0 and then A.Cov.VV > 0.0
                                 then
                                    Slots (I) := (Asked => True, Got => True, U => A.To.U, V => A.To.V,
                                                  Su => EF.Sqrt (A.Cov.UU), Sv => EF.Sqrt (A.Cov.VV));
                                 end if;
                              end;
                           end if;
                        end One;
                        procedure Every is new Driver.Parallel_For (One);
                     begin
                        for I in 1 .. Count loop
                           All_Sights (I) := Sights (I);
                        end loop;
                        for I in 1 .. Natural (Views.Length) loop
                           All_Views (I) := Views (I);
                        end loop;
                        if Count > 0 then
                           Every (1, Count);
                        end if;
                        for I in 1 .. Count loop
                           if Slots (I).Asked then
                              Asked := Asked + 1;
                           end if;
                           if Slots (I).Got then
                              Found.Append (Match'(X => All_Sights (I).X, U => Slots (I).U, V => Slots (I).V,
                                                   Su => Slots (I).Su, Sv => Slots (I).Sv));
                           end if;
                        end loop;
                        Free (All_Sights);
                        Free (All_Views);
                        Free (Slots);
                     end;
                     --  How far the places found lie from the true eye's places of their points.
                     declare
                        Off : Real_Array (1 .. Natural'Max (1, Natural (Found.Length)));
                        Far : Natural := 0;
                     begin
                        for M in 1 .. Natural (Found.Length) loop
                           declare
                              Tu, Tv : Real;
                              In_View : constant Boolean := Project (True_Lens, Truth, Found (M).X, Tu, Tv);
                           begin
                              Off (M) := (if In_View then EF.Sqrt ((Found (M).U - Tu) ** 2 + (Found (M).V - Tv) ** 2)
                                          else Real'Last);
                              if Off (M) > 2.0 then
                                 Far := Far + 1;
                              end if;
                           end;
                        end loop;
                        Put_Line ("scene round" & Round'Image & ": asked" & Asked'Image & ", found narrowly"
                                  & Found.Length'Image & "; off the true eye's places: median "
                                  & (if Found.Is_Empty then "none" else Image (Driver.Stats.Median (Off), 2))
                                  & " px, beyond two pixels" & Far'Image);
                     end;
                     --  The eye's lens and pose fitted to the places found: terms the focal length's logarithm, the
                     --  principal point, a turn about the eye's own axes and the centre's move, from Lens and Pose.
                     declare
                        Kept : array (1 .. Natural (Found.Length)) of Boolean := [others => True];
                        Y    : Real_Vector (1 .. 9) := [others => 0.0];
                        function Lens_Of (Z : Real_Vector) return Fit.Lens is
                          ((Fx => Lens.Fx * EF.Exp (Z (1)), Fy => Lens.Fy * EF.Exp (Z (1)), Cx => Lens.Cx + Z (2),
                            Cy => Lens.Cy + Z (3), K1 | K2 => 0.0));
                        function Pose_Of (Z : Real_Vector) return Rigid is
                          ((Rotation => Times (Pose.Rotation, Driver.Numerics.Exp (Vec3'[Z (4), Z (5), Z (6)])),
                            Translation => [Pose.Translation (1) + Z (7), Pose.Translation (2) + Z (8),
                                            Pose.Translation (3) + Z (9)]));
                        function Count_Kept return Natural is
                           C : Natural := 0;
                        begin
                           for B of Kept loop
                              if B then
                                 C := C + 1;
                              end if;
                           end loop;
                           return C;
                        end Count_Kept;
                        function Residuals (Z : Real_Vector) return Real_Array is
                           R  : Real_Array (1 .. 2 * Natural'Max (1, Count_Kept)) := [others => 0.0];
                           I  : Natural := 0;
                           L1 : constant Fit.Lens := Lens_Of (Z);
                           P1 : constant Rigid := Pose_Of (Z);
                        begin
                           for M in Kept'Range loop
                              if Kept (M) then
                                 declare
                                    Xc : constant Vec3 :=
                                      Transposed_Times
                                        (P1.Rotation, Driver.Numerics.Minus (Found (M).X, P1.Translation));
                                    U  : constant Real := L1.Cx + L1.Fx * Xc (1) / Xc (3);
                                    V  : constant Real := L1.Cy + L1.Fy * Xc (2) / Xc (3);
                                 begin
                                    R (I + 1) := (U - Found (M).U) / Found (M).Su;
                                    R (I + 2) := (V - Found (M).V) / Found (M).Sv;
                                    I := I + 2;
                                 end;
                              end if;
                           end loop;
                           return R;
                        end Residuals;
                        function Cost (R : Real_Array) return Real is
                           Sum : Real := 0.0;
                        begin
                           for X of R loop
                              Sum := Sum + X * X;
                           end loop;
                           return Sum;
                        end Cost;
                        Dropped : Natural;
                     begin
                        if Count_Kept >= 5 then
                           loop
                              --  Gauss-Newton until a step lowers the cost by less than one residual's worth.
                              loop
                                 declare
                                    R0  : constant Real_Array := Residuals (Y);
                                    C0  : constant Real := Cost (R0);
                                    Jac : Real_Matrix (R0'Range, 1 .. 9);
                                    Ata : Real_Matrix (1 .. 9, 1 .. 9) := [others => [others => 0.0]];
                                    Atb : Real_Vector (1 .. 9) := [others => 0.0];
                                    Lower : Real_Matrix (1 .. 9, 1 .. 9);
                                    Ok    : Boolean;
                                    Better : Boolean := False;
                                 begin
                                    for T in 1 .. 9 loop
                                       declare
                                          Up, Down : Real_Vector := Y;
                                       begin
                                          Up (T) := Y (T) + Step;
                                          Down (T) := Y (T) - Step;
                                          declare
                                             Ru : constant Real_Array := Residuals (Up);
                                             Rd : constant Real_Array := Residuals (Down);
                                          begin
                                             for I in R0'Range loop
                                                Jac (I, T) := (Ru (I) - Rd (I)) / (2.0 * Step);
                                             end loop;
                                          end;
                                       end;
                                    end loop;
                                    for A in 1 .. 9 loop
                                       for B in 1 .. 9 loop
                                          for I in R0'Range loop
                                             Ata (A, B) := Ata (A, B) + Jac (I, A) * Jac (I, B);
                                          end loop;
                                       end loop;
                                       for I in R0'Range loop
                                          Atb (A) := Atb (A) - Jac (I, A) * R0 (I);
                                       end loop;
                                    end loop;
                                    Driver.Numerics.Dense.Cholesky (Ata, Lower, Ok);
                                    exit when not Ok;
                                    declare
                                       D     : Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, Atb);
                                       Trial : Real_Vector (1 .. 9);
                                    begin
                                       for Halving in 1 .. Real'Machine_Mantissa loop
                                          for T in 1 .. 9 loop
                                             Trial (T) := Y (T) + D (T);
                                          end loop;
                                          declare
                                             C1 : constant Real := Cost (Residuals (Trial));
                                          begin
                                             if C1 < C0 then
                                                Y := Trial;
                                                Better := C0 - C1 >= Real'Max (1.0, C0 / Real (R0'Length));
                                                exit;
                                             end if;
                                          end;
                                          for T in 1 .. 9 loop
                                             D (T) := D (T) / 2.0;
                                          end loop;
                                       end loop;
                                    end;
                                    exit when not Better;
                                 end;
                              end loop;
                              --  The places off by more than Z robust sigmas left out, until none is.
                              declare
                                 R     : constant Real_Array := Residuals (Y);
                                 Scale : constant Real := Driver.Stats.Robust_Sigma (R);
                                 I     : Natural := 0;
                              begin
                                 Dropped := 0;
                                 for M in Kept'Range loop
                                    if Kept (M) then
                                       if EF.Sqrt (R (I + 1) ** 2 + R (I + 2) ** 2)
                                         > Driver.Conventions.Z * Scale * EF.Sqrt (2.0)
                                       then
                                          Kept (M) := False;
                                          Dropped := Dropped + 1;
                                       end if;
                                       I := I + 2;
                                    end if;
                                 end loop;
                              end;
                              exit when Dropped = 0 or else Count_Kept < 5;
                           end loop;
                           --  The fit's covariance at the end (the residuals' scatter over their stated sigmas times
                           --  the normal matrix's inverse): each term's sigma, and how far the weakest direction lets
                           --  the focal length and the centre go together.
                           declare
                              R0    : constant Real_Array := Residuals (Y);
                              Jac   : Real_Matrix (R0'Range, 1 .. 9);
                              Ata   : Real_Matrix (1 .. 9, 1 .. 9) := [others => [others => 0.0]];
                              Lower : Real_Matrix (1 .. 9, 1 .. 9);
                              Ok    : Boolean;
                              Scale : constant Real := Real'Max (1.0, Cost (R0) / Real'Max (1.0, Real (R0'Length - 9)));
                           begin
                              for T in 1 .. 9 loop
                                 declare
                                    Up, Down : Real_Vector := Y;
                                 begin
                                    Up (T) := Y (T) + Step;
                                    Down (T) := Y (T) - Step;
                                    declare
                                       Ru : constant Real_Array := Residuals (Up);
                                       Rd : constant Real_Array := Residuals (Down);
                                    begin
                                       for I in R0'Range loop
                                          Jac (I, T) := (Ru (I) - Rd (I)) / (2.0 * Step);
                                       end loop;
                                    end;
                                 end;
                              end loop;
                              for A in 1 .. 9 loop
                                 for B in 1 .. 9 loop
                                    for I in R0'Range loop
                                       Ata (A, B) := Ata (A, B) + Jac (I, A) * Jac (I, B);
                                    end loop;
                                 end loop;
                              end loop;
                              Driver.Numerics.Dense.Cholesky (Ata, Lower, Ok);
                              if Ok then
                                 declare
                                    Cov : Real_Matrix (1 .. 9, 1 .. 9);
                                 begin
                                    for T in 1 .. 9 loop
                                       declare
                                          E : Real_Vector (1 .. 9) := [others => 0.0];
                                       begin
                                          E (T) := 1.0;
                                          declare
                                             Col : constant Real_Vector :=
                                               Driver.Numerics.Dense.Cholesky_Solve (Lower, E);
                                          begin
                                             for A in 1 .. 9 loop
                                                Cov (A, T) := Scale * Col (A);
                                             end loop;
                                          end;
                                       end;
                                    end loop;
                                    Put_Line ("scene round" & Round'Image & ": sigmas focal "
                                              & Image (100.0 * EF.Sqrt (Cov (1, 1)), 2) & " %, principal point "
                                              & Image (EF.Sqrt (Cov (2, 2)), 2) & " " & Image (EF.Sqrt (Cov (3, 3)), 2)
                                              & " px, turn " & Image (180.0 / Ada.Numerics.Pi
                                                * EF.Sqrt (Cov (4, 4) + Cov (5, 5) + Cov (6, 6)), 3)
                                              & " deg, centre " & Image (1000.0 * EF.Sqrt (Cov (7, 7) + Cov (8, 8)
                                                                                           + Cov (9, 9)), 2)
                                              & " mm; focal with the centre along the eye's axis: correlation "
                                              & Image (Cov (1, 9) / EF.Sqrt (Cov (1, 1) * Cov (9, 9)), 3));
                                    --  The arms' joints in the picture at every pose of their sets, as the eye's
                                    --  verdict weighs them: how far the fitted eye puts them from where the true eye
                                    --  does, and how far the fit's own covariance says it may (the worst point's
                                    --  standard deviation, both axes together), with the arms' fit's for the start.
                                    declare
                                       Errors, Spreads, Starts : Real_Vectors.Vector;
                                    begin
                                       for X of Joints_At loop
                                          declare
                                             Fu, Fv, Tu, Tv, Su, Sv : Real;
                                          begin
                                             if Project (Lens_Of (Y), Pose_Of (Y), X, Fu, Fv)
                                               and then Project (True_Lens, Truth, X, Tu, Tv)
                                             then
                                                Errors.Append (EF.Sqrt ((Fu - Tu) ** 2 + (Fv - Tv) ** 2));
                                                if Project (Result.Lens, Result.Poses (Result.Poses'First), X,
                                                            Su, Sv)
                                                then
                                                   Starts.Append (EF.Sqrt ((Su - Tu) ** 2 + (Sv - Tv) ** 2));
                                                end if;
                                                declare
                                                   Gu, Gv : Real_Vector (1 .. 9);
                                                   Vu, Vv : Real := 0.0;
                                                begin
                                                   for T in 1 .. 9 loop
                                                      declare
                                                         Up, Down : Real_Vector := Y;
                                                         Uu, Uv, Du, Dv : Real;
                                                         B1, B2 : Boolean;
                                                      begin
                                                         Up (T) := Y (T) + Step;
                                                         Down (T) := Y (T) - Step;
                                                         B1 := Project (Lens_Of (Up), Pose_Of (Up), X, Uu, Uv);
                                                         B2 := Project (Lens_Of (Down), Pose_Of (Down), X, Du, Dv);
                                                         Gu (T) := (if B1 and then B2
                                                                    then (Uu - Du) / (2.0 * Step) else 0.0);
                                                         Gv (T) := (if B1 and then B2
                                                                    then (Uv - Dv) / (2.0 * Step) else 0.0);
                                                      end;
                                                   end loop;
                                                   for A in 1 .. 9 loop
                                                      for B in 1 .. 9 loop
                                                         Vu := Vu + Gu (A) * Cov (A, B) * Gu (B);
                                                         Vv := Vv + Gv (A) * Cov (A, B) * Gv (B);
                                                      end loop;
                                                   end loop;
                                                   Spreads.Append (EF.Sqrt (Vu + Vv));
                                                end;
                                             end if;
                                          end;
                                       end loop;
                                       declare
                                          function Most (V : Real_Vectors.Vector) return Real is
                                             M : Real := 0.0;
                                          begin
                                             for X of V loop
                                                M := Real'Max (M, X);
                                             end loop;
                                             return M;
                                          end Most;
                                       begin
                                          Put_Line ("scene round" & Round'Image & ": the arms' joints in the picture"
                                                    & Errors.Length'Image & ": off the truth at most "
                                                    & Image (Most (Errors), 2) & " px (the arms' fit "
                                                    & Image (Most (Starts), 2) & "), the fit's own spread at most "
                                                    & Image (Most (Spreads), 2) & " px, the verdict's bar "
                                                    & Image (Real (Driver.Alignment.Half_Patch (Width, Height))
                                                             / Driver.Conventions.Z, 2) & " px");
                                          Last_Spread := Most (Spreads);
                                       end;
                                    end;
                                 end;
                              end if;
                           end;
                           Lens := Lens_Of (Y);
                           Pose := Pose_Of (Y);
                           --  The next round's window: how far the fitted eye leaves the places it kept, in pixels.
                           declare
                              Px : Real_Array (1 .. 2 * Count_Kept);
                              I  : Natural := 0;
                           begin
                              for M in Kept'Range loop
                                 if Kept (M) then
                                    declare
                                       Tu, Tv : Real;
                                       Seen   : constant Boolean := Project (Lens, Pose, Found (M).X, Tu, Tv);
                                    begin
                                       Px (I + 1) := (if Seen then Tu - Found (M).U else 0.0);
                                       Px (I + 2) := (if Seen then Tv - Found (M).V else 0.0);
                                       I := I + 2;
                                    end;
                                 end if;
                              end loop;
                              Window := Driver.Stats.Robust_Sigma (Px);
                           end;
                        end if;
                        Put_Line ("scene round" & Round'Image & ": kept" & Count_Kept'Image & " of" & Found.Length'Image
                                  & ", the next window " & Image (Window, 2) & " px"
                                  & "; focal " & Image (Lens.Fx, 1) & " (" & Image (100.0 * (Lens.Fx / Cam.Fx - 1.0), 2)
                                  & " %), principal point (" & Image (Lens.Cx, 1) & ", " & Image (Lens.Cy, 1) & ") of ("
                                  & Image (Cam.Cx, 1) & ", " & Image (Cam.Cy, 1) & "), centre off by "
                                  & Image (1000.0 * abs Driver.Numerics.Minus (Pose.Translation, Truth.Translation), 1)
                                  & " mm, turn off by "
                                  & Image (180.0 / Ada.Numerics.Pi
                                           * Driver.Numerics.Angle (Times (Transposed (Truth.Rotation), Pose.Rotation)),
                                           2) & " deg");
                     end;
                  end;
                  --  Converged: the eye this round moved the arms' joints in the picture by no more than its own spread
                  --  there.
                  declare
                     Moved        : Real := 0.0;
                     Now_U, Now_V : Real_Vectors.Vector;
                  begin
                     for X of Joints_At loop
                        declare
                           U, V : Real;
                           --  a joint out of the picture moves in it all the same
                           Seen : constant Boolean := Project (Lens, Pose, X, U, V) with Unreferenced;
                        begin
                           Now_U.Append (U);
                           Now_V.Append (V);
                        end;
                     end loop;
                     if Prev_U.Is_Empty then
                        Moved := Real'Last;
                     else
                        for K in Now_U.First_Index .. Now_U.Last_Index loop
                           Moved := Real'Max
                             (Moved, EF.Sqrt ((Now_U (K) - Prev_U (K)) ** 2 + (Now_V (K) - Prev_V (K)) ** 2));
                        end loop;
                     end if;
                     Prev_U := Now_U;
                     Prev_V := Now_V;
                     Put_Line ("scene round" & Round'Image & ": the arms' joints moved at most "
                               & (if Moved >= Real'Last then "(first round)" else Image (Moved, 2) & " px")
                               & " since the round before");
                     exit when Moved <= Last_Spread;
                  end;
               end loop;
               Free (Fixed);
            end;
         end if;
      end;
   end;
end Driver.Robot.Flow_Bench;
