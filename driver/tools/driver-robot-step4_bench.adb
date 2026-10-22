--  Step 4 of the boot on the fake robot's bodies (tools/fake_world), without beats: each closer channel of each arm
--  that has an eye of its own, the channel at its two ends seen still by that eye (Frames frames each, the camera's
--  noise on every channel of every frame, rounded to a level), the eye's views of the robot at the anchored end over
--  Poses poses of the arm (its other joints turned a little, so that the world moves in the picture and the hand does
--  not), and the driver's own code on them: which pixels changed (Driver.Pixels.Compare), the lobes and their tips
--  (Driver.Robot.Hand.Lobes.From_Change), each tip's region and outline (Tip_Cap, Tip_Outline). Scored against the
--  truth: the fingers the channel moves, the renderer's nearest solid at every pixel; each finger's end face, its
--  corners through the eye's lens.
--
--  Settings (environment): BENCH_FRAMES (frames a still view, 4), BENCH_POSES (the anchored end's poses, the rest's
--  among them, 4), BENCH_SWING (radians each joint of the arm turns at most for a pose, 0.1), BENCH_SUPERSAMPLE (4),
--  BENCH_BLUR (0.6 px), BENCH_NOISE (0.7 levels), BENCH_HAND (pair, thumb, fingers, jaw, hook: both arms' hands;
--  drawn otherwise).

with Ada.Command_Line;
with Ada.Environment_Variables;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Bytes;
with Driver.Images;
with Driver.Log;
with Driver.Numerics;
with Driver.Pixels;
with Driver.Robot.Hand.Lobes;
with Fake_Render;
with Fake_World;

procedure Driver.Robot.Step4_Bench is
   use Ada.Text_IO;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use type Ada.Streams.Stream_Element_Offset;
   package FW renames Fake_World;
   package Lobes renames Driver.Robot.Hand.Lobes;
   package EF renames Ada.Numerics.Long_Elementary_Functions;
   use type Lobes.Placing;

   function Setting (Name : String; Default : Real) return Real is
     (if Ada.Environment_Variables.Exists (Name) then Real'Value (Ada.Environment_Variables.Value (Name)) else Default);
   function Image (X : Real; Aft : Natural := 2) return String is (Driver.Log.Image (X, Aft));
   function Trim (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));

   Frames      : constant Positive := Positive (Setting ("BENCH_FRAMES", 4.0));
   Poses       : constant Positive := Positive (Setting ("BENCH_POSES", 4.0));
   Swing       : constant Real := Setting ("BENCH_SWING", 0.1);
   Noise_Level : constant Real := Setting ("BENCH_NOISE", 0.7);
   Width       : constant Positive := 320;
   Height      : constant Positive := 240;

   Rng : Ada.Numerics.Float_Random.Generator;
   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Rng)));
   function Gaussian return Real is
      U1 : constant Real := Real'Max (Uniform, 1.0E-12);
      U2 : constant Real := Uniform;
   begin
      return EF.Sqrt (-2.0 * EF.Log (U1)) * EF.Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   type Rgb_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Rgb_Access);

   --  One frame of a render: the camera's noise on every channel, rounded to a level.
   function Frame (Rgb : Real_Array) return Driver.Images.Image is
      Bytes : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (Rgb'Length));
   begin
      for K in Bytes'Range loop
         Bytes (K) := Driver.Bytes.Byte
           (Real'Max (0.0, Real'Min (255.0, Real'Rounding (Rgb (Rgb'First + Integer (K) - 1) + Noise_Level * Gaussian))));
      end loop;
      return Driver.Images.Create (Width, Height, Bytes);
   end Frame;

   --  Geometry in the picture.
   type Point is record
      U, V : Real := 0.0;
   end record;
   type Point_Array is array (Positive range <>) of Point;

   function Segment_Distance (P, A, B : Point) return Real is
      Du : constant Real := B.U - A.U;
      Dv : constant Real := B.V - A.V;
      L2 : constant Real := Du * Du + Dv * Dv;
      T  : constant Real :=
        (if L2 > 0.0 then Real'Max (0.0, Real'Min (1.0, ((P.U - A.U) * Du + (P.V - A.V) * Dv) / L2)) else 0.0);
   begin
      return EF.Sqrt ((P.U - A.U - T * Du) ** 2 + (P.V - A.V - T * Dv) ** 2);
   end Segment_Distance;

   --  How far P lies from the polygon Corners (in order around it): nought inside a convex one.
   function Polygon_Distance (P : Point; Corners : Point_Array) return Real is
      Best     : Real := Real'Last;
      Pos, Neg : Boolean := False;
   begin
      if Corners'Length = 0 then
         return Real'Last;
      end if;
      for I in Corners'Range loop
         declare
            A : constant Point := Corners (I);
            B : constant Point := Corners (if I = Corners'Last then Corners'First else I + 1);
            C : constant Real := (B.U - A.U) * (P.V - A.V) - (B.V - A.V) * (P.U - A.U);
         begin
            Best := Real'Min (Best, Segment_Distance (P, A, B));
            Pos := Pos or else C > 0.0;
            Neg := Neg or else C < 0.0;
         end;
      end loop;
      if Corners'Length >= 3 and then not (Pos and then Neg) then
         return 0.0;
      end if;
      return Best;
   end Polygon_Distance;

   Kind_Names : constant array (FW.Hand_Kind) of Unbounded_String :=
     [FW.Drawn => To_Unbounded_String ("drawn"), FW.Pair => To_Unbounded_String ("pair"),
      FW.Hook => To_Unbounded_String ("hook"), FW.Pair_And_Thumb => To_Unbounded_String ("thumb"),
      FW.Fingers => To_Unbounded_String ("fingers"), FW.Jaw => To_Unbounded_String ("jaw")];

   --  Totals over every lobe scored.
   Scored, Matched, Within_One : Natural := 0;
   Tip_Sum, Tip_Worst, Outline_Worst : Real := 0.0;
   Fingers_Seen, Lobes_Found, Channels_Placed, Channels_Asked : Natural := 0;

   procedure Measure_Channel (W : in out FW.World; Seed, A, Cam, Arm_Group, G : Positive; K : Positive) is
      L      : constant Positive := W.Groups (G).Joints (K);
      Rest   : constant Real := W.Links (L).Q;
      Lo     : constant Real := W.Links (L).Lo;
      Hi     : constant Real := W.Links (L).Hi;
      Low_View, High_View, Anchor : Driver.Pixels.View := Driver.Pixels.Empty (Width, Height);
      Anchored_Low : constant Boolean := abs (Rest - Lo) <= abs (Rest - Hi);
      --  The fingers the channel moves: its link's, and those of the links that copy it.
      type Flag_Array is array (Positive range <>) of Boolean;
      Moved  : Flag_Array (W.Solids.First_Index .. W.Solids.Last_Index) := [others => False];
      Count_Moved : Natural := 0;
      type Mask_Array is array (Positive range <>, Positive range <>) of Driver.Images.Mask;
      type Face_Array is array (Positive range <>, Positive range <>) of Point_Array (1 .. 4);
      type Inside_Array is array (Positive range <>, Positive range <>) of Natural;
      --  Per end (1 low, 2 high) and per solid: its pixels, its end face's corners, how many of them the picture holds.
      Truth  : Mask_Array (1 .. 2, Moved'Range);
      Own_Still : Driver.Images.Mask := Driver.Images.Create (Width, Height);   --  the arm's other solids at the low end (BENCH_ATTACH_TRUTH)
      Faces  : Face_Array (1 .. 2, Moved'Range);
      Inside : Inside_Array (1 .. 2, Moved'Range) := [others => [others => 0]];
      Prefix : constant String :=
        "seed" & Seed'Image & " arm" & A'Image & " " & To_String (Kind_Names (W.Kinds (A))) & " channel" & K'Image;

      procedure Set (Q : Real) is
      begin
         W.Links (L).Q := Q;
         FW.Forward (W);
      end Set;

      function Render return Rgb_Access is
         Rgb : constant Rgb_Access := new Real_Array (1 .. 3 * Width * Height);
         C   : aliased constant FW.Camera := W.Cameras (Cam);
      begin
         Fake_Render.Render (W, C, Rgb.all);
         return Rgb;
      end Render;

      --  The truth at end E: every pixel's nearest solid, and each moved finger's end face (its box's face at +y in
      --  its own frame, the far end of every finger the fake builds) through the eye's lens.
      procedure Truth_At (E : Positive) is
         C : constant FW.Camera := W.Cameras (Cam);
         To_Eye : constant Rigid := Inverse (C.Pose);
      begin
         for S in Moved'Range loop
            Truth (E, S) := Driver.Images.Create (Width, Height);
         end loop;
         for Row in 0 .. Height - 1 loop
            for Column in 0 .. Width - 1 loop
               declare
                  Hit   : Natural;
                  Along : Real;
               begin
                  Fake_Render.Nearest (W, C, Real (Column) + 0.5, Real (Row) + 0.5, Hit, Along);
                  if Hit in Moved'Range and then Moved (Hit) then
                     Driver.Images.Include (Truth (E, Hit), Column, Row);
                  elsif E = 1 and then Hit in Moved'Range and then W.Solids (Hit).Arm = A then
                     Driver.Images.Include (Own_Still, Column, Row);
                  end if;
               end;
            end loop;
         end loop;
         for S in Moved'Range loop
            if Moved (S) then
               declare
                  H     : constant Vec3 := W.Solids (S).Half;
                  Signs : constant array (1 .. 4, 1 .. 2) of Real :=
                    [[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]];
               begin
                  for I in 1 .. 4 loop
                     declare
                        Local : constant Vec3 := [Signs (I, 1) * H (1), H (2), Signs (I, 2) * H (3)];
                        P     : constant Vec3 := To_Eye * (W.Solids (S).Pose * Local);
                        U, V  : Real;
                        Front : Boolean;
                     begin
                        FW.Project (C, P, U, V, Front);
                        Faces (E, S) (I) := (U => U, V => V);
                        if Front and then U in 0.0 .. Real (Width) and then V in 0.0 .. Real (Height) then
                           Inside (E, S) := Inside (E, S) + 1;
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end loop;
      end Truth_At;

      procedure Add_Frames (V : in out Driver.Pixels.View; Rgb : Real_Array) is
      begin
         for F in 1 .. Frames loop
            Driver.Pixels.Add (V, Frame (Rgb));
         end loop;
      end Add_Frames;

      Rgb : Rgb_Access;
   begin
      for S in Moved'Range loop
         if W.Solids (S).Finger and then W.Solids (S).Link > 0
           and then (W.Solids (S).Link = L or else W.Links (W.Solids (S).Link).Mirror = L)
         then
            Moved (S) := True;
            Count_Moved := Count_Moved + 1;
         end if;
      end loop;
      Channels_Asked := Channels_Asked + 1;
      Set (Lo);
      Rgb := Render;
      Add_Frames (Low_View, Rgb.all);
      if Anchored_Low then
         Driver.Pixels.Add (Anchor, Frame (Rgb.all));
      end if;
      Free (Rgb);
      Truth_At (1);
      Set (Hi);
      Rgb := Render;
      Add_Frames (High_View, Rgb.all);
      if not Anchored_Low then
         Driver.Pixels.Add (Anchor, Frame (Rgb.all));
      end if;
      Free (Rgb);
      Truth_At (2);
      --  The anchored end's poses: the arm's joints each turned a little either way, the closer held there.
      Set (if Anchored_Low then Lo else Hi);
      declare
         Joints : FW.Index_Vectors.Vector renames W.Groups (Arm_Group).Joints;
         Saved  : array (Joints.First_Index .. Joints.Last_Index) of Real;
      begin
         for J in Saved'Range loop
            Saved (J) := W.Links (Joints (J)).Q;
         end loop;
         for P in 2 .. Poses loop
            for J in Saved'Range loop
               W.Links (Joints (J)).Q := Saved (J) + Swing * (2.0 * Uniform - 1.0) / W.Links (Joints (J)).Unit;
            end loop;
            FW.Forward (W);
            Rgb := Render;
            Driver.Pixels.Add (Anchor, Frame (Rgb.all));
            Free (Rgb);
         end loop;
         for J in Saved'Range loop
            W.Links (Joints (J)).Q := Saved (J);
         end loop;
      end;
      Set (Rest);
      declare
         Compared : constant Driver.Pixels.Comparison := Driver.Pixels.Compare (Low_View, High_View);
         Seen     : array (1 .. 2) of Natural := [0, 0];
         --  BENCH_ATTACH_STILL (a study): the lobes attached also to every pixel that did not change and that the poses
         --  call still (its deviation over them under the cut the placing's seeds have), robot or plain world alike:
         --  placed once with nothing attached for the cut, then again.
         function Still_Attached return Driver.Images.Mask is
            First : constant Lobes.Located :=
              Lobes.From_Change (Compared.Changed, Low_View, High_View, Anchor, Anchored_Low, Compared.Spread,
                                 Driver.Images.Create (0, 0));
            M     : Driver.Images.Mask := Driver.Images.Create (Width, Height);
         begin
            for Row in 0 .. Height - 1 loop
               for Column in 0 .. Width - 1 loop
                  if not Driver.Images.Contains (Compared.Changed, Column, Row)
                    and then EF.Sqrt (Driver.Pixels.Variance (Anchor, Column, Row)) < First.Cut
                  then
                     Driver.Images.Include (M, Column, Row);
                  end if;
               end loop;
            end loop;
            return M;
         end Still_Attached;
      begin
         for E in 1 .. 2 loop
            for S in Moved'Range loop
               if Moved (S) and then Driver.Images.Count (Truth (E, S)) > 0 then
                  Seen (E) := Seen (E) + 1;
               end if;
            end loop;
         end loop;
         Fingers_Seen := Fingers_Seen + Natural'Max (Seen (1), Seen (2));
         --  BENCH_DUMP (a directory): the channel's views and masks as pictures (PGM): the low and high ends' means, the
         --  anchor's deviation over its poses, the changed pixels, and the true fingers' pixels at each end.
         if Ada.Environment_Variables.Exists ("BENCH_DUMP") then
            declare
               Dir  : constant String := Ada.Environment_Variables.Value ("BENCH_DUMP");
               Stem : constant String := Dir & "/s" & Trim (Seed) & "_a" & Trim (A) & "_c" & Trim (K) & "_";
               procedure Write (Name : String; Value : access function (Column, Row : Natural) return Real) is
                  F : File_Type;
               begin
                  Create (F, Out_File, Stem & Name & ".pgm");
                  Put_Line (F, "P2");
                  Put_Line (F, Trim (Width) & " " & Trim (Height));
                  Put_Line (F, "255");
                  for Row in 0 .. Height - 1 loop
                     for Column in 0 .. Width - 1 loop
                        Put (F, Trim (Integer (Real'Max (0.0, Real'Min (255.0, Value (Column, Row))))) & " ");
                     end loop;
                     New_Line (F);
                  end loop;
                  Close (F);
               end Write;
               function Low_Mean (Column, Row : Natural) return Real is (Driver.Pixels.Mean (Low_View, Column, Row));
               function High_Mean (Column, Row : Natural) return Real is (Driver.Pixels.Mean (High_View, Column, Row));
               function Deviation (Column, Row : Natural) return Real is
                 (20.0 * EF.Sqrt (Driver.Pixels.Variance (Anchor, Column, Row)));
               function Changed (Column, Row : Natural) return Real is
                 (if Driver.Images.Contains (Compared.Changed, Column, Row) then 255.0 else 0.0);
               function Truth_Of (E : Positive; Column, Row : Natural) return Real is
                  Value : Real := 0.0;
               begin
                  for S in Moved'Range loop
                     if Moved (S) and then Driver.Images.Contains (Truth (E, S), Column, Row) then
                        Value := 255.0;
                     end if;
                  end loop;
                  return Value;
               end Truth_Of;
               function Truth_Low (Column, Row : Natural) return Real is (Truth_Of (1, Column, Row));
               function Truth_High (Column, Row : Natural) return Real is (Truth_Of (2, Column, Row));
            begin
               Write ("low", Low_Mean'Access);
               Write ("high", High_Mean'Access);
               Write ("anchor_dev", Deviation'Access);
               Write ("changed", Changed'Access);
               Write ("truth_low", Truth_Low'Access);
               Write ("truth_high", Truth_High'Access);
            end;
         end if;
         if not Compared.Trusted then
            Put_Line (Prefix & ": everything changed between the ends (" & Trim (Count_Moved) & " fingers moved)");
            return;
         elsif Driver.Images.Count (Compared.Changed) = 0 then
            Put_Line (Prefix & ": nothing changed between the ends (" & Trim (Count_Moved) & " fingers moved)");
            return;
         end if;
         declare
            Located : constant Lobes.Located :=
              Lobes.From_Change (Compared.Changed, Low_View, High_View, Anchor, Anchored_Low, Compared.Spread,
                                 (if Ada.Environment_Variables.Exists ("BENCH_ATTACH_TRUTH") then Own_Still
                                  elsif Ada.Environment_Variables.Exists ("BENCH_ATTACH_STILL") then Still_Attached
                                  else Driver.Images.Create (0, 0)));
         begin
            Put_Line (Prefix & ": " & Located.How'Image & "," & Natural (Located.Lobes.Length)'Image & " lobes for"
                      & Seen (1)'Image & " fingers seen low," & Seen (2)'Image & " high;"
                      & Driver.Images.Count (Compared.Changed)'Image & " px changed (spread "
                      & Image (Compared.Spread) & "), seeds" & Located.Seeds'Image & ", unassigned"
                      & Located.Unassigned'Image & ", doubt " & Image (Located.Doubt, 1) & ", the "
                      & (if Anchored_Low then "low" else "high") & " end anchored");
            if Ada.Environment_Variables.Exists ("BENCH_DUMP") then
               declare
                  F : File_Type;
               begin
                  Create (F, Out_File, Ada.Environment_Variables.Value ("BENCH_DUMP") & "/s" & Trim (Seed) & "_a"
                          & Trim (A) & "_c" & Trim (K) & "_still.pgm");
                  Put_Line (F, "P2");
                  Put_Line (F, Trim (Width) & " " & Trim (Height));
                  Put_Line (F, "255");
                  for Row in 0 .. Height - 1 loop
                     for Column in 0 .. Width - 1 loop
                        Put (F, (if Driver.Images.Width (Located.Still) = Width
                                   and then Driver.Images.Contains (Located.Still, Column, Row) then "255 " else "0 "));
                     end loop;
                     New_Line (F);
                  end loop;
                  Close (F);
               end;
            end if;
            if Located.How /= Lobes.Placed then
               return;
            end if;
            Channels_Placed := Channels_Placed + 1;
            Lobes_Found := Lobes_Found + Natural (Located.Lobes.Length);
            for N in Located.Lobes.First_Index .. Located.Lobes.Last_Index loop
               for E in 1 .. 2 loop
                  declare
                     Lb     : Lobes.Lobe renames Located.Lobes (N);
                     M      : constant Driver.Images.Mask := (if E = 1 then Lb.Here else Lb.There);
                     Known  : constant Boolean := (if E = 1 then Lb.Tip_Known_Here else Lb.Tip_Known_There);
                     Tip    : constant Driver.Images.Pixel := (if E = 1 then Lb.Tip_Here else Lb.Tip_There);
                     Centre : constant Driver.Images.Pixel := (if E = 1 then Lb.Centre_Here else Lb.Centre_There);
                     Best   : Natural := 0;
                     Most   : Natural := 0;
                     Union  : Natural := 0;
                  begin
                     for S in Moved'Range loop
                        if Moved (S) then
                           declare
                              Both : Natural := 0;
                           begin
                              for Row in 0 .. Height - 1 loop
                                 for Column in 0 .. Width - 1 loop
                                    if Driver.Images.Contains (M, Column, Row)
                                      and then Driver.Images.Contains (Truth (E, S), Column, Row)
                                    then
                                       Both := Both + 1;
                                    end if;
                                 end loop;
                              end loop;
                              if Both > Most then
                                 Most := Both;
                                 Best := S;
                              end if;
                           end;
                        end if;
                     end loop;
                     if Best > 0 then
                        Union := Driver.Images.Count (M) + Driver.Images.Count (Truth (E, Best)) - Most;
                     end if;
                     if not Known then
                        Put_Line ("  lobe" & N'Image & " " & (if E = 1 then "low" else "high") & ": no tip");
                     elsif Best = 0 then
                        Put_Line ("  lobe" & N'Image & " " & (if E = 1 then "low" else "high") & ": tip at ("
                                  & Image (Tip.U, 1) & ", " & Image (Tip.V, 1) & "), on no finger the channel moves ("
                                  & Driver.Images.Count (M)'Image & " px)");
                        Scored := Scored + 1;
                     else
                        declare
                           Face    : constant Point_Array := Faces (E, Best);
                           Off     : constant Real := Polygon_Distance ((U => Tip.U, V => Tip.V), Face);
                           Cap     : constant Driver.Images.Mask := Lobes.Tip_Cap (M, Centre, Tip);
                           Hull    : constant Lobes.Pixel_Vectors.Vector := Lobes.Tip_Outline (Cap);
                           Corners : Point_Array (1 .. Natural (Hull.Length));
                           Outline_Off : Real := 0.0;
                           Face_Off    : Real := 0.0;
                        begin
                           for I in Corners'Range loop
                              Corners (I) := (U => Hull (Hull.First_Index + I - 1).U,
                                              V => Hull (Hull.First_Index + I - 1).V);
                           end loop;
                           for I in Corners'Range loop
                              Outline_Off := Real'Max (Outline_Off, Polygon_Distance (Corners (I), Face));
                           end loop;
                           for I in Face'Range loop
                              if Face (I).U in 0.0 .. Real (Width) and then Face (I).V in 0.0 .. Real (Height) then
                                 Face_Off := Real'Max (Face_Off, Polygon_Distance (Face (I), Corners));
                              end if;
                           end loop;
                           Scored := Scored + 1;
                           Matched := Matched + 1;
                           Tip_Sum := Tip_Sum + Off;
                           Tip_Worst := Real'Max (Tip_Worst, Off);
                           Outline_Worst := Real'Max (Outline_Worst, Real'Max (Outline_Off, Face_Off));
                           if Off <= 1.0 then
                              Within_One := Within_One + 1;
                           end if;
                           Put_Line ("  lobe" & N'Image & " " & (if E = 1 then "low" else "high") & ": "
                                     & To_String (W.Solids (Best).Name) & " (shares" & Most'Image & " of"
                                     & Union'Image & " px), tip (" & Image (Tip.U, 1) & ", " & Image (Tip.V, 1)
                                     & ") off its end face by " & Image (Off) & " px; the tip's outline"
                                     & Corners'Length'Image & " corners, off the face by " & Image (Outline_Off)
                                     & " px, the face's" & Inside (E, Best)'Image & " corners in the picture off it by "
                                     & Image (Face_Off) & " px");
                        end;
                     end if;
                  end;
               end loop;
            end loop;
         end;
      end;
   end Measure_Channel;

   Forced : constant String :=
     (if Ada.Environment_Variables.Exists ("BENCH_HAND") then Ada.Environment_Variables.Value ("BENCH_HAND") else "");
begin
   Fake_Render.Supersample := Positive (Setting ("BENCH_SUPERSAMPLE", 4.0));
   Fake_Render.Lens_Blur := Setting ("BENCH_BLUR", 0.6);
   for Arg in 1 .. Ada.Command_Line.Argument_Count loop
      declare
         Seed : constant Natural := Natural'Value (Ada.Command_Line.Argument (Arg));
         W    : FW.World;
         T    : FW.Template;
      begin
         if Forced /= "" then
            T.Hands := [others => (if Forced = "pair" then FW.Pair elsif Forced = "thumb" then FW.Pair_And_Thumb
                                   elsif Forced = "fingers" then FW.Fingers elsif Forced = "jaw" then FW.Jaw
                                   elsif Forced = "hook" then FW.Hook else FW.Drawn)];
         end if;
         Ada.Numerics.Float_Random.Reset (Rng, Seed + 1);
         FW.Build (W, Seed, Width, Height, T);
         FW.Forward (W);
         for A in 1 .. W.Arms loop
            declare
               Cam       : Natural := 0;
               Arm_Group : Natural := 0;
               Closers   : Natural := 0;
               Arm_Name  : constant String := "arm" & Trim (A);
            begin
               for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
                  if W.Cameras (C).Link /= 0 and then W.Cameras (C).Arm = A then
                     Cam := C;
                  end if;
               end loop;
               for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                  if To_String (W.Groups (G).Name) = Arm_Name & "_joints" then
                     Arm_Group := G;
                  end if;
               end loop;
               if Cam = 0 or else Arm_Group = 0 then
                  Put_Line ("seed" & Seed'Image & " arm" & A'Image & ": "
                            & (if Cam = 0 then "no eye of its own" else "no joints group named " & Arm_Name & "_joints"));
               else
                  for G in W.Groups.First_Index .. W.Groups.Last_Index loop
                     declare
                        Name : constant String := To_String (W.Groups (G).Name);
                     begin
                        if Name = Arm_Name & "_closer"
                          or else (Name'Length > Arm_Name'Length + 7
                                   and then Name (Name'First .. Name'First + Arm_Name'Length + 6) = Arm_Name & "_finger")
                        then
                           Closers := Closers + 1;
                           for K in W.Groups (G).Joints.First_Index .. W.Groups (G).Joints.Last_Index loop
                              Measure_Channel (W, Seed, A, Cam, Arm_Group, G, K);
                           end loop;
                        end if;
                     end;
                  end loop;
                  if Closers = 0 then
                     Put_Line ("seed" & Seed'Image & " arm" & A'Image & " " & To_String (Kind_Names (W.Kinds (A)))
                               & ": no closer");
                  end if;
               end if;
            end;
         end loop;
      end;
   end loop;
   Put_Line ("over every channel: placed" & Channels_Placed'Image & " of" & Channels_Asked'Image & ";"
             & Lobes_Found'Image & " lobes for" & Fingers_Seen'Image & " fingers seen; tips scored" & Scored'Image
             & ", on a finger" & Matched'Image & ", within a pixel of its end face" & Within_One'Image
             & "; off it by " & Image ((if Matched > 0 then Tip_Sum / Real (Matched) else 0.0)) & " px on average, "
             & Image (Tip_Worst) & " at worst; outlines off by " & Image (Outline_Worst) & " px at worst");
end Driver.Robot.Step4_Bench;
