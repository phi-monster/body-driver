--  driver-robot-step2_bench SEED... : step 2 of the boot, the joints' order along an arm and its eye's lens, on the
--  arms of the fake robot's body of each seed (jaw hands, servo readings of 4096 ticks a turn, as the fake runs have
--  them), without pictures or beats: the points of the world the arm's eye sees at its reference pose, where they land
--  at every keyframe of a sweep of each joint alone and of the turns the fit asks to tell untold orders apart, with the
--  noise and the near misses the matcher was measured to have (BENCH_NOISE, BENCH_NEAR and its range), given to the
--  driver's own fit (Driver.Robot.Kinematics.Fit.Fit) as the boot gives it its keyframes' sightings. Each arm's line
--  says whether the order the fit tells is the body's, and how long it took: a version of the fit judged in seconds.

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Containers.Vectors;
with Ada.Environment_Variables;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Alignment;
with Driver.Numerics;
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Fit;
with Driver.Stats;
with Fake_World;

procedure Driver.Robot.Step2_Bench is
   use Ada.Text_IO;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   package Fit renames Driver.Robot.Kinematics.Fit;
   package FW renames Fake_World;
   use type Fit.Chain_Order;
   use type FW.Joint_Kind;
   package EF renames Ada.Numerics.Long_Elementary_Functions;

   function Setting (Name : String; Default : Real) return Real is
     (if Ada.Environment_Variables.Exists (Name) then Real'Value (Ada.Environment_Variables.Value (Name)) else Default);

   Noise      : constant Real := Setting ("BENCH_NOISE", 0.3);        --  px per axis, every sighting
   Near_Share : constant Real := Setting ("BENCH_NEAR", 0.2);         --  the share of sightings a near miss
   Near_Low   : constant Real := Setting ("BENCH_NEAR_LOW", 1.5);     --  px, a near miss's size, least
   Near_High  : constant Real := Setting ("BENCH_NEAR_HIGH", 4.0);    --  and most
   Rounds     : constant Natural := Natural (Setting ("BENCH_ROUNDS", 8.0));
   --  BENCH_DRAW (a study): another draw of the noise and the near misses for the same arm (nought: the arm's own).
   Draw       : constant Natural := Natural (Setting ("BENCH_DRAW", 0.0));
   Grid       : constant Positive := Positive (Setting ("BENCH_GRID", 16.0));   --  px between the points' rays
   Keep_Share : constant Real := Setting ("BENCH_KEEP", 0.5);         --  of the reference's points, a keyframe keeps
   Levels     : constant Real_Array := [8.0, 16.0, 32.0, 64.0];   --  the sweep's turns, by the px they move the view

   Rng : Ada.Numerics.Float_Random.Generator;
   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Rng)));
   function Gaussian return Real is
      U1 : constant Real := Real'Max (Uniform, 1.0E-12);
      U2 : constant Real := Uniform;
   begin
      return EF.Sqrt (-2.0 * EF.Log (U1)) * EF.Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  Thousandths of X, or none when X is no number the image holds.
   function Milli (X : Real) return String is
     (if X >= 1.0E6 or else X /= X then " none" else Natural'Image (Natural (1000.0 * Real'Max (0.0, X))));

   package Vec_Vectors is new Ada.Containers.Vectors (Positive, Vec3, Driver.Numerics.Arrays."=");
   package Row_Vectors is new Ada.Containers.Vectors (Positive, Real);
   package Sight_Vectors is new Ada.Containers.Vectors (Positive, Fit.Sighting, Fit."=");

   --  The nearest solid a ray from O along D meets: its distance, and whether it is the world's (no link's).
   procedure Cast (W : FW.World; O, D : Vec3; Hit : out Real; World_Solid : out Boolean) is
   begin
      Hit := Real'Last;
      World_Solid := False;
      for S of W.Solids loop
         declare
            Back : constant Rigid := Inverse (S.Pose);
            Ol   : constant Vec3 := Back * O;
            Dl   : constant Vec3 := Back.Rotation * D;
            T    : Real := Real'Last;
         begin
            case S.Kind is
               when FW.Plane =>
                  if Dl (3) < 0.0 and then Ol (3) > 0.0 then
                     T := -Ol (3) / Dl (3);
                  end if;
               when FW.Box =>
                  declare
                     Near : Real := 0.0;
                     Far  : Real := Real'Last;
                     Ok   : Boolean := True;
                  begin
                     for I in 1 .. 3 loop
                        if Dl (I) = 0.0 then
                           if abs Ol (I) > S.Half (I) then
                              Ok := False;
                           end if;
                        else
                           declare
                              T1 : constant Real := (-S.Half (I) - Ol (I)) / Dl (I);
                              T2 : constant Real := (S.Half (I) - Ol (I)) / Dl (I);
                           begin
                              Near := Real'Max (Near, Real'Min (T1, T2));
                              Far := Real'Min (Far, Real'Max (T1, T2));
                           end;
                        end if;
                     end loop;
                     if Ok and then Near <= Far and then Near > 0.0 then
                        T := Near;
                     end if;
                  end;
            end case;
            if T < Hit then
               Hit := T;
               World_Solid := S.Link = 0;
            end if;
         end;
      end loop;
   end Cast;

   --  Where a point of the world lands in camera C as the world stands now.
   procedure Land (W : FW.World; C : Positive; X : Vec3; U, V : out Real; Seen : out Boolean) is
      Cam : FW.Camera renames W.Cameras (C);
      P   : constant Vec3 := Inverse (Cam.Pose) * X;
   begin
      U := 0.0;
      V := 0.0;
      Seen := P (3) > 0.0;
      if Seen then
         U := Cam.Cx + Cam.Fx * P (1) / P (3);
         V := Cam.Cy + Cam.Fy * P (2) / P (3);
         Seen := U >= 0.0 and then U <= Real (Cam.Width) and then V >= 0.0 and then V <= Real (Cam.Height);
      end if;
   end Land;

   type Link_List is array (Positive range <>) of Positive;

   procedure Pose_At (W : in out FW.World; Path : Link_List; Readings : Real_Array) is
   begin
      for I in Readings'Range loop
         W.Links (Path (I)).Q := Readings (I);
      end loop;
      FW.Forward (W);
   end Pose_At;

   --  The run's draw of a group (BENCH_SCRAMBLE names its scramble.json): the driver's channel C reads the group's
   --  joint Order (C) + 1, times Sign (C) * Scale (C); none drawn, each channel its own joint, as is.
   procedure Draw_Of (Name : String; N : Positive; Order : out Fit.Chain_Order; Factor : out Real_Array) is
      Text : Unbounded_String;
      function Numbers (Key : String; From : Positive) return Real_Array is
         Result : Real_Array (1 .. N) := [others => 0.0];
         At_Key : constant Natural := Index (Text, """" & Key & """", From);
         I      : Natural;
      begin
         if At_Key = 0 then
            raise Constraint_Error with "no " & Key & " for " & Name;
         end if;
         I := Index (Text, "[", At_Key) + 1;
         for K in 1 .. N loop
            declare
               J : Natural := I;
            begin
               while Element (Text, J) /= ',' and then Element (Text, J) /= ']' loop
                  J := J + 1;
               end loop;
               Result (K) := Real'Value (Slice (Text, I, J - 1));
               I := J + 1;
            end;
         end loop;
         return Result;
      end Numbers;
   begin
      for C in 1 .. N loop
         Order (C) := C;
      end loop;
      Factor := [others => 1.0];
      --  No run's draw named: one of the bench's own, as the fake robot draws a servo bus's (a shuffle of the channels,
      --  each a sign and a scale of 1, 2 or 4), so that the channels' order says nothing of the joints' (the fit takes
      --  the first of equal orders in their lexicographic sequence).
      if not Ada.Environment_Variables.Exists ("BENCH_SCRAMBLE") then
         for I in reverse 2 .. N loop
            declare
               J : constant Positive := 1 + Natural (Real'Floor (Real (I) * Uniform)) mod I;
               T : constant Positive := Order (I);
            begin
               Order (I) := Order (J);
               Order (J) := T;
            end;
         end loop;
         for C in 1 .. N loop
            Factor (C) := (if Uniform < 0.5 then -1.0 else 1.0) * 2.0 ** (Natural (Real'Floor (3.0 * Uniform)) mod 3);
         end loop;
         return;
      end if;
      declare
         File : File_Type;
      begin
         Open (File, In_File, Ada.Environment_Variables.Value ("BENCH_SCRAMBLE"));
         while not End_Of_File (File) loop
            Append (Text, Get_Line (File));
         end loop;
         Close (File);
      end;
      declare
         At_Group : constant Natural := Index (Text, """groups""");
         At_Name  : constant Natural := Index (Text, """" & Name & """: {", At_Group);
         O        : constant Real_Array := Numbers ("order", At_Name);
         Sg       : constant Real_Array := Numbers ("sign", At_Name);
         Sc       : constant Real_Array := Numbers ("scale", At_Name);
      begin
         for C in 1 .. N loop
            Order (C) := Natural (O (C)) + 1;
            Factor (C) := Sg (C) * Sc (C);
         end loop;
      end;
   end Draw_Of;

   --  The eye's chain: every joint some group reports between the world and the link the eye rides on, from the world
   --  out; and the group the first of them is in, whose draw a run's scramble gives.
   function Chain_Of (W : FW.World; Cam : Positive) return Link_List is
      Up    : Link_List (1 .. Natural (W.Links.Length));
      Count : Natural := 0;
      L     : Natural := W.Cameras (Cam).Link;
   begin
      while L /= 0 loop
         if W.Links (L).Kind /= FW.Fixed
           and then (for some G of W.Groups => (for some J of G.Joints => J = L))
         then
            Count := Count + 1;
            Up (Count) := L;
         end if;
         L := W.Links (L).Parent;
      end loop;
      return [for I in 1 .. Count => Up (Count - I + 1)];
   end Chain_Of;

   function Group_Of (W : FW.World; L : Positive) return String is
   begin
      for G of W.Groups loop
         if (for some J of G.Joints => J = L) then
            return To_String (G.Name);
         end if;
      end loop;
      return "";
   end Group_Of;

   procedure Bench_Eye (W : in out FW.World; Seed : Natural; Cam : Positive) is
      Path   : constant Link_List := Chain_Of (W, Cam);
      Name   : constant String := To_String (W.Cameras (Cam).Name);
      Began  : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      if Path'Length < 2 then
         Put_Line ("seed" & Seed'Image & " eye " & Name & ": fewer than two joints carry it");
         return;
      end if;
      declare
         N        : constant Positive := Path'Length;
         Q0       : Real_Array (1 .. N);
         Points   : Vec_Vectors.Vector;
         U0s, V0s : Row_Vectors.Vector;
         Rows     : Row_Vectors.Vector;   --  every keyframe's reading changes, N a keyframe
         Flags    : Row_Vectors.Vector;   --  and which joints it asked, 1.0 or 0.0
         Seen_All : Sight_Vectors.Vector;
         Visible  : Real_Array (1 .. N) := [others => 0.0];   --  per joint of the group, in its own reading units
         Slopes   : Real_Array (1 .. N) := [others => 0.0];   --  px the view moves a reading unit of each
         Reached  : Real_Array (1 .. N) := [others => 0.0];   --  the widest turn of each the sweep took
         Truth    : Fit.Chain_Order (1 .. N);                   --  by the driver's channels
         Order_D  : Fit.Chain_Order (1 .. N);   --  the driver's channel C reads the group's joint Order_D (C)
         Factor   : Real_Array (1 .. N);        --  times this
         Of_Fake  : Fit.Chain_Order (1 .. N);   --  the channel that reads joint K
         Frames   : Natural := 1;   --  the reference is frame 1

         --  The keyframe at Readings: its sightings, noise and near misses and all; whether half the points stayed.
         procedure Keyframe (Readings : Real_Array; Asked : Real_Array; Kept : out Boolean) is
            Here : Sight_Vectors.Vector;
         begin
            Pose_At (W, Path, Readings);
            for P in Points.First_Index .. Points.Last_Index loop
               declare
                  U, V : Real;
                  Seen : Boolean;
               begin
                  Land (W, Cam, Points (P), U, V, Seen);
                  if Seen then
                     U := U + Noise * Gaussian;
                     V := V + Noise * Gaussian;
                     if Uniform < Near_Share then
                        declare
                           R   : constant Real := Near_Low + (Near_High - Near_Low) * Uniform;
                           Phi : constant Real := 2.0 * Ada.Numerics.Pi * Uniform;
                        begin
                           U := U + R * EF.Cos (Phi);
                           V := V + R * EF.Sin (Phi);
                        end;
                     end if;
                     Here.Append (Fit.Sighting'(Frame => Frames + 1, Track => P, U0 => U0s (P), V0 => V0s (P),
                                                U => U, V => V, Own => Noise, Own_Freedom => 0,
                                                Off_U => 0.0, Off_V => 0.0, Off_Told => True));
                  end if;
               end;
            end loop;
            Pose_At (W, Path, Q0);
            Kept := Real (Here.Length) >= Keep_Share * Real (Points.Length);
            if Kept then
               Frames := Frames + 1;
               for C in 1 .. N loop
                  Rows.Append (Factor (C) * (Readings (Order_D (C)) - Q0 (Order_D (C))));
                  Flags.Append (Asked (Order_D (C)));
               end loop;
               for S of Here loop
                  Seen_All.Append (S);
               end loop;
            end if;
         end Keyframe;

         --  The median residual across and down of the sightings of keyframes that moved several joints, with the
         --  joints J, lens Lx and depths of R in order C (the truth's diagnostics).
         function Multi_Score
           (J : Fit.Joint_Array; Lx : Fit.Lens; R : Fit.Fit_Report; C : Fit.Chain_Order) return Real is
            D : Row_Vectors.Vector;
         begin
            for X of Seen_All loop
               declare
                  Change : Real_Array (1 .. N);
                  Moving : Natural := 0;
                  Depth  : constant Real :=
                    (if X.Track <= Natural (R.Depths.Length) then R.Depths (R.Depths.First_Index + X.Track - 1)
                     else 0.0);
               begin
                  for K in 1 .. N loop
                     Change (K) := Rows ((X.Frame - 2) * N + K);
                     Moving := Moving + Boolean'Pos (Change (K) /= 0.0);
                  end loop;
                  if Moving >= 2 and then Depth > 0.0 then
                     declare
                        T     : constant Rigid := Fit.Eye_At (J, C, Change);
                        U, V  : Real;
                        Ahead : Boolean;
                     begin
                        Fit.Project (Lx, Inverse (T) * (Depth * Fit.Ray (Lx, X.U0, X.V0)), U, V, Ahead);
                        if Ahead then
                           D.Append (abs (U - X.U));
                           D.Append (abs (V - X.V));
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if D.Is_Empty then
               return Real'Last;
            end if;
            declare
               A : Real_Array (1 .. Natural (D.Length));
            begin
               for I in A'Range loop
                  A (I) := D (I);
               end loop;
               return Driver.Stats.Median (A);
            end;
         end Multi_Score;

         --  The median distance the points move in the eye from the reference to Readings, the noise left out.
         function Moved (Readings : Real_Array) return Real is
            D : Row_Vectors.Vector;
         begin
            Pose_At (W, Path, Readings);
            for P in Points.First_Index .. Points.Last_Index loop
               declare
                  U, V : Real;
                  Seen : Boolean;
               begin
                  Land (W, Cam, Points (P), U, V, Seen);
                  if Seen then
                     D.Append (EF.Sqrt ((U - U0s (P)) ** 2 + (V - V0s (P)) ** 2));
                  end if;
               end;
            end loop;
            Pose_At (W, Path, Q0);
            if D.Is_Empty then
               return Real'Last;
            end if;
            declare
               X : Real_Array (1 .. Natural (D.Length));
            begin
               for I in X'Range loop
                  X (I) := D (I);
               end loop;
               return Driver.Stats.Median (X);
            end;
         end Moved;

         function Within (J : Positive; Reading : Real) return Boolean is
            L : FW.Link renames W.Links (Path (J));
         begin
            return L.Lo >= L.Hi or else (Reading >= L.Lo and then Reading <= L.Hi);
         end Within;

         First_Scale : Real := 1.0;   --  of the shares, the first cells'
         Half_View   : Real := 0.0;   --  half the picture's least side, px
         --  The boot's brakes (Boot.Take): before a fit a joint's ladder climbs while the view moves no more than two
         --  patches; a fitted arm's, no more than half the picture's least side. Climbed blind until half the points
         --  left the view, one joint's levels reached a thousand pixels, and the tracks' fit, its turn per unit known
         --  to half a percent from the narrow ones, set those aside as not fitting and never fitted them (the bench's
         --  s36, 10-20: 0.8 to 1.3 px left on them, two to four sightings of a hundred kept).
         Patch_Px : constant Real :=
           2.0 * Real (Driver.Alignment.Half_Patch (W.Cameras (Cam).Width, W.Cameras (Cam).Height));
         Half_Px  : constant Real := Real (Natural'Min (W.Cameras (Cam).Width, W.Cameras (Cam).Height)) / 2.0;
         Next_Level  : array (1 .. N, 1 .. 2) of Real := [others => [others => Levels (Levels'First)]];
         Ladder_Open : array (1 .. N, 1 .. 2) of Boolean := [others => [others => True]];

         --  Joint J's ladder either way, each level twice the last, from where it stood while the view moves by no
         --  more than Most px, the keyframe keeps half the points, and the joint stays within its range; one level
         --  each way when Once (a pass of the boot's before a fit, below). Took: a level was asked.
         procedure Ladder (J : Positive; Most : Real; Once : Boolean; Took : in out Boolean) is
         begin
            for Way in 1 .. 2 loop
               declare
                  Taken : Boolean := False;
               begin
                  while Ladder_Open (J, Way) and then Next_Level (J, Way) <= Most
                    and then not (Once and then Taken)
                  loop
                     declare
                        Try  : Real_Array := Q0;
                        Ask  : Real_Array (1 .. N) := [others => 0.0];
                        Kept : Boolean;
                     begin
                        Try (J) := Q0 (J) + (if Way = 1 then 1.0 else -1.0) * Next_Level (J, Way) / Slopes (J);
                        Ask (J) := 1.0;
                        if not Within (J, Try (J)) then
                           Ladder_Open (J, Way) := False;
                        else
                           Keyframe (Try, Ask, Kept);
                           Taken := True;
                           Took := True;
                           if Kept then
                              Reached (J) := Real'Max (Reached (J), Next_Level (J, Way) / Slopes (J));
                              Next_Level (J, Way) := 2.0 * Next_Level (J, Way);
                           else
                              Ladder_Open (J, Way) := False;
                           end if;
                        end if;
                     end;
                  end loop;
               end;
            end loop;
         end Ladder;
         Asked_Any : Boolean := False;   --  Ladder's Took where nothing reads it

         --  Every row of the cells at Scale of each joint's share (its widest reach over the joints' count).
         procedure Cells (Scale : Real) is
            Rows : Positive := 1;
         begin
            while Rows < N + 1 loop
               Rows := 2 * Rows;
            end loop;
            for Row in 0 .. Rows - 1 loop
               declare
                  Try  : Real_Array := Q0;
                  Ask  : constant Real_Array (1 .. N) := [others => 1.0];
                  Kept : Boolean;
                  Ok   : Boolean := True;
               begin
                  for J in 1 .. N loop
                     declare
                        Bits : Natural := 0;
                        R    : Natural := Row;
                        Col  : Natural := J;
                     begin
                        while R > 0 and then Col > 0 loop
                           if R mod 2 = 1 and then Col mod 2 = 1 then
                              Bits := Bits + 1;
                           end if;
                           R := R / 2;
                           Col := Col / 2;
                        end loop;
                        if Slopes (J) > 0.0 then
                           Try (J) := Q0 (J) + (if Bits mod 2 = 0 then 1.0 else -1.0) * Scale * Reached (J) / Real (N);
                           Ok := Ok and then Within (J, Try (J));
                        end if;
                     end;
                  end loop;
                  if Ok then
                     Keyframe (Try, Ask, Kept);
                  end if;
               end;
            end loop;
         end Cells;

         --  The view's motion at Scale of the shares.
         function Motion_At (Scale : Real) return Real is
            Sum : Real := 0.0;
         begin
            for J in 1 .. N loop
               Sum := Sum + Scale * Reached (J) / Real (N) * Slopes (J);
            end loop;
            return Sum;
         end Motion_At;

      begin
         Ada.Numerics.Float_Random.Reset (Rng, Seed * 31 + Cam + 7919 * Draw);
         Draw_Of (Group_Of (W, Path (Path'First)), N, Order_D, Factor);
         for C in 1 .. N loop
            Of_Fake (Order_D (C)) := C;
         end loop;
         for J in 1 .. N loop
            Q0 (J) := W.Links (Path (J)).Q;
         end loop;
         Pose_At (W, Path, Q0);
         --  The points: where rays on a grid over the eye's picture meet the world, not the body.
         declare
            C : FW.Camera renames W.Cameras (Cam);
         begin
            for R in 0 .. (C.Height - 1) / Grid loop
               for K in 0 .. (C.Width - 1) / Grid loop
                  declare
                     U   : constant Real := Real (K * Grid) + Real (Grid) / 2.0;
                     V   : constant Real := Real (R * Grid) + Real (Grid) / 2.0;
                     Dc  : constant Vec3 := [(U - C.Cx) / C.Fx, (V - C.Cy) / C.Fy, 1.0];
                     Dw  : constant Vec3 := C.Pose.Rotation * ((1.0 / EF.Sqrt (Dc (1) ** 2 + Dc (2) ** 2 + 1.0)) * Dc);
                     Hit : Real;
                     Ok  : Boolean;
                  begin
                     Cast (W, C.Pose.Translation, Dw, Hit, Ok);
                     if Ok and then Hit < Real'Last then
                        Points.Append (C.Pose.Translation + Hit * Dw);
                        U0s.Append (U);
                        V0s.Append (V);
                     end if;
                  end;
               end loop;
            end loop;
         end;
         --  The truth: the chain from the world out, by the driver's channels.
         for I in 1 .. N loop
            Truth (I) := Of_Fake (I);
         end loop;
         --  The sweep: each joint alone, either way, by the turns that move the view the levels' pixels.
         for J in 1 .. N loop
            declare
               Turn  : Real := 1.0;
               Slope : Real := 0.0;
            begin
               for Doubling in 1 .. 40 loop
                  declare
                     Try : Real_Array := Q0;
                  begin
                     Try (J) := Q0 (J) + Turn;
                     if not Within (J, Try (J)) then
                        Try (J) := Q0 (J) - Turn;
                     end if;
                     declare
                        M : constant Real := Moved (Try);
                     begin
                        if M < Real'Last and then M >= 1.0 then
                           Slope := M / Turn;
                           exit;
                        end if;
                     end;
                  end;
                  Turn := 2.0 * Turn;
               end loop;
               Slopes (J) := Slope;
               if Slope > 0.0 then
                  Visible (J) := 3.0 * Noise / Slope;
                  Ladder (J, 2.0 * Patch_Px, Once => False, Took => Asked_Any);
               end if;
            end;
         end loop;
         --  Every joint at once, in the boot's cells (Boot.Together): each joint by its share, the widest level it was
         --  followed to over the joints' count, either way as the rows of a Sylvester-Hadamard matrix have it, at
         --  Scale of the shares. Before a fit the cells move the view no farther in all than two patches (Boot.Shares,
         --  the ladder's brake); a fitted arm's cells climb, doubled a level while the view moves no more than half
         --  the picture's least side (Boot.Together's climb). Taken at the widest at once, the climbing sweep's cells
         --  moved six joints together so far that no order's fit from the joints of one joint's keyframes came near
         --  them (the bench's s36, 10-20: every order of it, the true one too, above 1.2 px on them).
         declare
            Patch : Real renames Patch_Px;
            Half  : Real renames Half_Px;
            function Cell_Motion (Scale : Real) return Real is
               Sum : Real := 0.0;
            begin
               for J in 1 .. N loop
                  Sum := Sum + Scale * Reached (J) / Real (N) * Slopes (J);
               end loop;
               return Sum;
            end Cell_Motion;
         begin
            First_Scale := (if Cell_Motion (1.0) > 2.0 * Patch then 2.0 * Patch / Cell_Motion (1.0) else 1.0);
            Half_View := Half;
            Cells (First_Scale);
         end;
         --  The fits: the sweep's, then one a round after the turns of the pairs it leaves untold.
         declare
            Have_Prev   : Boolean := False;
            Prev_Joints : Fit.Joint_Array (1 .. N);
            Prev_Chain  : Fit.Chain_Order (1 .. N) := [for I in 1 .. N => I];
            Prev_L      : Fit.Lens;
            Prev        : Fit.Fit_Report;
            Fits        : Natural := 0;
            Told        : Boolean := False;
            Last_Untold : Natural := Natural'Last;
            Climbed     : Boolean := False;   --  the arm's cells climbed once it was fitted
            Stalled     : Natural := 0;
         begin
            for Round in 0 .. Rounds loop
               declare
                  F       : constant Positive := Frames;
                  Changes : Driver.Numerics.Arrays.Real_Matrix (1 .. F, 1 .. N) := [others => [others => 0.0]];
                  Asked   : Fit.Flag_Matrix (1 .. F, 1 .. N) := [others => [others => False]];
                  Seen    : Fit.Sighting_Array (1 .. Natural (Seen_All.Length));
                  Start   : Fit.Fit_Start (if Have_Prev then N else 0);
                  Joints  : Fit.Joint_Array (1 .. N);
                  Chain   : Fit.Chain_Order (1 .. N);
                  L       : Fit.Lens;
                  Report  : Fit.Fit_Report;
                  T0      : constant Ada.Calendar.Time := Ada.Calendar.Clock;
               begin
                  for K in 2 .. F loop
                     for J in 1 .. N loop
                        Changes (K, J) := Rows ((K - 2) * N + J);
                        Asked (K, J) := Flags ((K - 2) * N + J) > 0.0;
                     end loop;
                  end loop;
                  for I in Seen'Range loop
                     Seen (I) := Seen_All (I);
                  end loop;
                  if Have_Prev then
                     Start.Valid := True;
                     Start.Joints := Prev_Joints;
                     Start.Chain := Prev_Chain;
                     Start.L := Prev_L;
                     Start.Focal_Within := Prev.Focal_Within;
                     Start.Searched_Least := Prev.Searched_Least;
                     Start.Searched_Most := Prev.Searched_Most;
                     Start.Profiled_Over := Prev.Profiled_Over;
                     Start.Order_Told := Prev.Order_Told;
                     Start.Untold_Orders := Prev.Untold_Orders;
                     Start.Slide_Turns := Fit.Slide_Turns_Of (Prev, N);
                     Start.Whole := True;
                  end if;
                  Fit.Fit (Changes, Asked, [for C in 1 .. N => abs Factor (C) * Visible (Order_D (C))], Seen,
                           W.Cameras (Cam).Width, W.Cameras (Cam).Height, 0, Joints, Chain, L, Report, Who => "",
                           From => Start);
                  Fits := Fits + 1;
                  declare
                     Untold : constant Natural := Natural (Report.Untold_Orders.Length) / N;
                     Line   : Unbounded_String;
                  begin
                     for C of Chain loop
                        Append (Line, C'Image);
                     end loop;
                     Append (Line, ", stages s");
                     for X of Report.Seconds loop
                        Append (Line, Natural'Image (Natural (X)));
                     end loop;
                     Put_Line ("  seed" & Seed'Image & " eye " & Name & " round" & Round'Image & ":" & Frames'Image
                               & " keyframes," & Natural'Image (Seen'Length) & " sightings, fitted "
                               & Report.Fitted'Image & ", told " & Report.Order_Told'Image & ", untold orders"
                               & Untold'Image & ", chain"
                               & To_String (Line) & ", focal" & Natural'Image (Natural (L.Fx)) & " ("
                               & Natural'Image (Natural (W.Cameras (Cam).Fx)) & " true), median"
                               & Natural'Image (Natural (100.0 * Report.Median_Px)) & "/100 px, in"
                               & Duration'Image (Ada.Calendar."-" (Ada.Calendar.Clock, T0)) & " s"
                               & (if Report.Fitted then "" else ": " & To_String (Report.Why)));
                     Told := Report.Order_Told;
                     Have_Prev := Report.Fitted;
                     Prev_Joints := Joints;
                     Prev_Chain := Chain;
                     Prev_L := L;
                     Prev := Report;
                     if Report.Fitted and then not Climbed
                       and then not Ada.Environment_Variables.Exists ("BENCH_NO_CLIMB")
                     then
                        Climbed := True;
                        for J in 1 .. N loop
                           if Slopes (J) > 0.0 then
                              Ladder (J, Half_Px, Once => False, Took => Asked_Any);
                           end if;
                        end loop;
                        declare
                           Scale : Real := First_Scale;
                        begin
                           while Motion_At (2.0 * Scale) <= Half_View loop
                              Scale := 2.0 * Scale;
                              Cells (Scale);
                           end loop;
                        end;
                        goto Next_Round;
                     end if;
                     if not Report.Fitted then
                        --  The boot's passes before a fit (Boot's pass loop): a pass's first level of each push is
                        --  taken whatever the brake (Boot.Take), a doubling beyond the last, as far as half the picture
                        --  (a push then Widest), and the arm is fitted again; the passes end when no push climbs. The
                        --  bench ended at an arm's first round that was not fitted, where the boot goes on (the bench's
                        --  heads and drones, 10-20: their first rounds, within two patches, left the focal length
                        --  undetermined, and the bench said not fitted).
                        declare
                           More : Boolean := False;
                        begin
                           for J in 1 .. N loop
                              if Slopes (J) > 0.0 then
                                 Ladder (J, Half_Px, Once => True, Took => More);
                              end if;
                           end loop;
                           exit when not More;
                        end;
                        goto Next_Round;
                     end if;
                     for Kp in Report.Untold.First_Index .. Report.Untold.Last_Index loop
                        Put_Line ("    untold pair: channels" & Report.Untold (Kp).Earlier'Image & " and"
                                  & Report.Untold (Kp).Later'Image & " (the body's joints"
                                  & Order_D (Report.Untold (Kp).Earlier)'Image & " and"
                                  & Order_D (Report.Untold (Kp).Later)'Image & "), turns"
                                  & Real'Image (Report.Untold_Turns (Report.Untold_Turns.First_Index
                                     + 2 * (Kp - Report.Untold.First_Index))) & " and"
                                  & Real'Image (Report.Untold_Turns (Report.Untold_Turns.First_Index
                                     + 2 * (Kp - Report.Untold.First_Index) + 1)));
                     end loop;
                     exit when Told or else Report.Untold.Is_Empty;
                     Stalled := (if Untold >= Last_Untold then Stalled + 1 else 0);
                     Last_Untold := Untold;
                     exit when Ada.Environment_Variables.Exists ("BENCH_BOOT_STOP") and then Stalled > 0;
                  end;
                  --  The turns the fit asks: each untold pair turned together by its two turns, either way.
                  for K in Report.Untold.First_Index .. Report.Untold.Last_Index loop
                     declare
                        P  : constant Joint_Pair := Report.Untold (K);
                        Te : constant Real :=
                          Report.Untold_Turns (Report.Untold_Turns.First_Index + 2 * (K - Report.Untold.First_Index));
                        Tl : constant Real :=
                          Report.Untold_Turns
                            (Report.Untold_Turns.First_Index + 2 * (K - Report.Untold.First_Index) + 1);
                     begin
                        for Row in 0 .. 3 loop
                           declare
                              Try  : Real_Array := Q0;
                              Ask  : Real_Array (1 .. N) := [others => 0.0];
                              Kept : Boolean;
                              E    : constant Positive := Order_D (P.Earlier);
                              La   : constant Positive := Order_D (P.Later);
                              --  Row's signs in a Sylvester-Hadamard matrix of four, columns 1 and 2 (Boot.Together)
                              Se   : constant Real := (if Row mod 2 = 1 then -1.0 else 1.0);
                              Sl   : constant Real := (if Row / 2 = 1 then -1.0 else 1.0);
                           begin
                              Try (E) := Q0 (E) + Se * Te / Factor (P.Earlier);
                              Try (La) := Q0 (La) + Sl * Tl / Factor (P.Later);
                              Ask (E) := 1.0;
                              Ask (La) := 1.0;
                              if Within (E, Try (E)) and then Within (La, Try (La)) then
                                 Keyframe (Try, Ask, Kept);
                              end if;
                           end;
                        end loop;
                     end;
                  end loop;
               end;
               <<Next_Round>> null;
            end loop;
            --  With BENCH_TRUTH: every sighting against where the true joints put it, in the true order and in the one
            --  the fit took: whether the sightings tell the two apart at all, and how far the fit's joints are off.
            if Ada.Environment_Variables.Exists ("BENCH_TRUTH") and then Have_Prev then
               declare
                  Cam0  : constant Rigid := W.Cameras (Cam).Pose;
                  True_J : Fit.Joint_Array (1 .. N);
                  function Median_Of (Order : Fit.Chain_Order; Multi : Boolean) return Real is
                     D : Row_Vectors.Vector;
                  begin
                     for X of Seen_All loop
                        declare
                           Change : Real_Array (1 .. N);
                           Moved  : Natural := 0;
                        begin
                           for C in 1 .. N loop
                              Change (C) := Rows ((X.Frame - 2) * N + C);
                              Moved := Moved + Boolean'Pos (Change (C) /= 0.0);
                           end loop;
                           if not Multi or else Moved >= 2 then
                              declare
                                 T  : constant Rigid := Fit.Eye_At (True_J, Order, Change);
                                 Pk : constant Vec3 := Inverse (T) * (Inverse (Cam0) * Points (X.Track));
                                 U  : constant Real := W.Cameras (Cam).Cx + W.Cameras (Cam).Fx * Pk (1) / Pk (3);
                                 V  : constant Real := W.Cameras (Cam).Cy + W.Cameras (Cam).Fy * Pk (2) / Pk (3);
                              begin
                                 D.Append (EF.Sqrt ((X.U - U) ** 2 + (X.V - V) ** 2));
                              end;
                           end if;
                        end;
                     end loop;
                     declare
                        A : Real_Array (1 .. Natural (D.Length));
                     begin
                        for I in A'Range loop
                           A (I) := D (I);
                        end loop;
                        return (if A'Length = 0 then 0.0 else Driver.Stats.Median (A));
                     end;
                  end Median_Of;
               begin
                  Pose_At (W, Path, Q0);
                  for C in 1 .. N loop
                     declare
                        L    : FW.Link renames W.Links (Path (Order_D (C)));
                        Axis : constant Vec3 := Transpose (Cam0.Rotation) * (L.Pose.Rotation * L.Axis);
                        At_P : constant Vec3 := Inverse (Cam0) * L.Pose.Translation;
                        Along : constant Real := At_P (1) * Axis (1) + At_P (2) * Axis (2) + At_P (3) * Axis (3);
                        Near  : constant Vec3 := At_P - Along * Axis;
                     begin
                        True_J (C) := (W => Axis, P => Near, C => L.Unit / Factor (C), Slide => L.Kind = FW.Prismatic);
                     end;
                  end loop;
                  --  The true joints in the reference eye's frame, and for each pair the last fit left untold, the true
                  --  joints in the true order against the same joints with the two swapped at the turns the fit asked:
                  --  the median distance between the two places of the points the eye sees at both.
                  for C in 1 .. N loop
                     Put_Line ("    true joint c" & C'Image & " (body" & Order_D (C)'Image & "): "
                               & (if True_J (C).Slide then "slide" else "turn") & ", axis"
                               & True_J (C).W (1)'Image & True_J (C).W (2)'Image & True_J (C).W (3)'Image & ", near"
                               & True_J (C).P (1)'Image & True_J (C).P (2)'Image & True_J (C).P (3)'Image & ", unit"
                               & True_J (C).C'Image);
                  end loop;
                  --  The fit's lens against the eye's own: its focal lengths, its aspect, its distortion, and whether it
                  --  was fitted as a pinhole with square pixels (its aspect held at one, the distortion at nought).
                  Put_Line ("    fit lens: focal" & Prev_L.Fx'Image & Prev_L.Fy'Image & " for" & W.Cameras (Cam).Fx'Image
                            & W.Cameras (Cam).Fy'Image & ", aspect" & Real'Image (Prev_L.Fy / Prev_L.Fx) & " for"
                            & Real'Image (W.Cameras (Cam).Fy / W.Cameras (Cam).Fx) & ", distortion" & Prev_L.K1'Image
                            & Prev_L.K2'Image & (if Prev.Pinhole then ", a pinhole" else ", the extras fitted"));
                  --  The fit's joints against the true ones, as far as the fit's free scale lets them be set beside
                  --  each other: each axis's angle off the true one, and its unit over the true unit (by the sign of
                  --  the axis's).
                  for C in 1 .. N loop
                     declare
                        Fj : Fit.Joint renames Prev_Joints (C);
                        Dw : constant Real :=
                          Fj.W (1) * True_J (C).W (1) + Fj.W (2) * True_J (C).W (2) + Fj.W (3) * True_J (C).W (3);
                     begin
                        Put_Line ("    fit joint c" & C'Image & ": axis off"
                                  & Real'Image (57.29577951308232 * EF.Arccos (Real'Min (1.0, abs Dw))) & " deg, unit"
                                  & Real'Image ((if Dw < 0.0 then -Fj.C else Fj.C) / True_J (C).C) & " of the truth's");
                     end;
                  end loop;
                  --  The fit's own covariance against those errors: each axis's tilt along the two directions across
                  --  it and each turn's unit, over their standard deviations, and the chi square of them all together
                  --  (about as many as there are of them, when the covariance is honest). A slide's unit is a length,
                  --  in the fit's free scale: left out.
                  if Natural (Prev.Covariance.Length) = Fit.Terms (N) ** 2 then
                     declare
                        M     : constant Positive := Fit.Terms (N);
                        function V (P, Q : Positive) return Real is
                          (Prev.Covariance (Prev.Covariance.First_Index + (P - 1) * M + Q - 1));
                        Index : array (1 .. 3 * N) of Positive := [others => 1];
                        Names : array (1 .. 3 * N) of Unbounded_String;
                        E     : Real_Vector (1 .. 3 * N) := [others => 0.0];
                        K     : Natural := 0;
                        procedure Add (Term : Positive; Error : Real; Name : String) is
                        begin
                           K := K + 1;
                           Index (K) := Term;
                           Names (K) := To_Unbounded_String (Name);
                           E (K) := Error;
                        end Add;
                     begin
                        for C in 1 .. N loop
                           declare
                              Fj   : Fit.Joint renames Prev_Joints (C);
                              Dw   : constant Real :=
                                Fj.W (1) * True_J (C).W (1) + Fj.W (2) * True_J (C).W (2) + Fj.W (3) * True_J (C).W (3);
                              Sign : constant Real := (if Dw < 0.0 then -1.0 else 1.0);
                              E1, E2 : Vec3;
                              T1, T2, Tc : Positive;
                           begin
                              Fit.Across (Fj.W, E1, E2);
                              T1 := Fit.Term_Of (C, Fit.Tilt_1);
                              T2 := Fit.Term_Of (C, Fit.Tilt_2);
                              Tc := Fit.Term_Of (C, Fit.Scale);
                              Add (T1, Sign * (True_J (C).W (1) * E1 (1) + True_J (C).W (2) * E1 (2)
                                               + True_J (C).W (3) * E1 (3)), C'Image & "t1");
                              Add (T2, Sign * (True_J (C).W (1) * E2 (1) + True_J (C).W (2) * E2 (2)
                                               + True_J (C).W (3) * E2 (3)), C'Image & "t2");
                              if not Fj.Slide then
                                 Add (Tc, Sign * True_J (C).C - Fj.C, C'Image & "u");
                              end if;
                              Put_Line ("    fit joint c" & C'Image & " against its covariance: tilts"
                                        & Real'Image (E (K - (if Fj.Slide then 1 else 2))
                                                      / EF.Sqrt (V (T1, T1)))
                                        & Real'Image (E (K - (if Fj.Slide then 0 else 1)) / EF.Sqrt (V (T2, T2)))
                                        & " sigmas (sigma"
                                        & Real'Image (57.29577951308232 * EF.Sqrt (V (T1, T1) + V (T2, T2)))
                                        & " deg)"
                                        & (if Fj.Slide then ""
                                           else ", unit" & Real'Image (E (K) / EF.Sqrt (V (Tc, Tc))) & " sigmas (sigma"
                                                & Real'Image (EF.Sqrt (V (Tc, Tc)) / abs Fj.C) & " of it)"));
                           end;
                        end loop;
                        declare
                           Sub   : Real_Matrix (1 .. K, 1 .. K);
                           Lower : Real_Matrix (1 .. K, 1 .. K);
                           Pd    : Boolean;
                        begin
                           for P in 1 .. K loop
                              for Q in 1 .. K loop
                                 Sub (P, Q) := V (Index (P), Index (Q));
                              end loop;
                           end loop;
                           Driver.Numerics.Dense.Cholesky (Sub, Lower, Pd);
                           if Pd then
                              declare
                                 Y   : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lower, E (1 .. K));
                                 Chi : Real := 0.0;
                              begin
                                 for P in 1 .. K loop
                                    Chi := Chi + E (P) * Y (P);
                                 end loop;
                                 Put_Line ("    fit joints against their covariance: chi square" & Real'Image (Chi)
                                           & " over" & K'Image & " tilts and units");
                                 --  The error along each axis of that covariance over the axis's sigma, the three largest: which
                                 --  combination of the tilts and units the fit states too little of (its three largest terms:
                                 --  joint, then t1 or t2 for a tilt, u for a unit).
                                 declare
                                    Values  : Real_Vector (1 .. K);
                                    Vectors : Real_Matrix (1 .. K, 1 .. K);
                                    Z       : Real_Vector (1 .. K);
                                    Done    : array (1 .. K) of Boolean := [others => False];
                                    function Name (P : Positive) return String is (To_String (Names (P)));
                                 begin
                                    Driver.Numerics.Dense.Small_Eigensystem (Sub, Values, Vectors);
                                    for A in 1 .. K loop
                                       Z (A) := 0.0;
                                       for P in 1 .. K loop
                                          Z (A) := Z (A) + Vectors (P, A) * E (P);
                                       end loop;
                                       Z (A) := (if Values (A) > 0.0 then Z (A) / EF.Sqrt (Values (A)) else 0.0);
                                    end loop;
                                    for Rank in 1 .. 3 loop
                                       declare
                                          Best : Natural := 0;
                                       begin
                                          for A in 1 .. K loop
                                             if not Done (A) and then (Best = 0 or else abs Z (A) > abs Z (Best)) then
                                                Best := A;
                                             end if;
                                          end loop;
                                          Done (Best) := True;
                                          declare
                                             Line : Unbounded_String :=
                                               To_Unbounded_String ("    fit joints' axis" & Rank'Image & ":" & Real'Image (Z (Best))
                                                                    & " sigmas, its sigma" & Real'Image (EF.Sqrt (Values (Best))) & ",");
                                             Used : array (1 .. K) of Boolean := [others => False];
                                          begin
                                             for Top in 1 .. 3 loop
                                                declare
                                                   Big : Natural := 0;
                                                begin
                                                   for P in 1 .. K loop
                                                      if not Used (P) and then (Big = 0 or else abs Vectors (P, Best) > abs Vectors (Big, Best)) then
                                                         Big := P;
                                                      end if;
                                                   end loop;
                                                   Used (Big) := True;
                                                   Append (Line, " " & Name (Big) & " (" & Real'Image (Vectors (Big, Best)) & ")");
                                                end;
                                             end loop;
                                             Put_Line (To_String (Line));
                                          end;
                                       end;
                                    end loop;
                                 end;
                              end;
                           else
                              Put_Line ("    fit joints against their covariance: not positive definite");
                           end if;
                        end;
                     end;
                  else
                     Put_Line ("    fit joints against their covariance: none (" & Prev.Covariance.Length'Image
                               & " entries)");
                  end if;
                  for Kp in Prev.Untold.First_Index .. Prev.Untold.Last_Index loop
                     declare
                        E  : constant Positive := Prev.Untold (Kp).Earlier;
                        La : constant Positive := Prev.Untold (Kp).Later;
                        Te : constant Real := Prev.Untold_Turns (Prev.Untold_Turns.First_Index
                                                                 + 2 * (Kp - Prev.Untold.First_Index));
                        Tl : constant Real := Prev.Untold_Turns (Prev.Untold_Turns.First_Index
                                                                 + 2 * (Kp - Prev.Untold.First_Index) + 1);
                        Swapped : Fit.Chain_Order := Truth;
                        Cosine  : constant Real :=
                          True_J (E).W (1) * True_J (La).W (1) + True_J (E).W (2) * True_J (La).W (2)
                          + True_J (E).W (3) * True_J (La).W (3);
                        Cr : constant Vec3 := Cross (True_J (E).W, True_J (La).W);
                        Dp : constant Vec3 := True_J (La).P - True_J (E).P;
                        Gap : constant Real :=
                          (if abs Cr > 1.0E-9 then abs (Cr (1) * Dp (1) + Cr (2) * Dp (2) + Cr (3) * Dp (3)) / abs Cr
                           else abs Cross (Dp, True_J (E).W));
                     begin
                        for I in Swapped'Range loop
                           if Truth (I) = E then
                              Swapped (I) := La;
                           elsif Truth (I) = La then
                              Swapped (I) := E;
                           end if;
                        end loop;
                        Put_Line ("    untold c" & E'Image & " with c" & La'Image & ": axes"
                                  & Integer'Image (Integer (57.3 * EF.Arccos (Real'Max (-1.0, Real'Min (1.0, Cosine)))))
                                  & " deg apart, lines" & Integer'Image (Integer (1000.0 * Gap)) & " mm apart");
                        for Row in 0 .. 3 loop
                           declare
                              Change : Real_Array (1 .. N) := [others => 0.0];
                              T1, T2 : Rigid;
                              D      : Row_Vectors.Vector;
                           begin
                              Change (E) := (if Row mod 2 = 1 then -Te else Te);
                              Change (La) := (if Row / 2 = 1 then -Tl else Tl);
                              T1 := Fit.Eye_At (True_J, Truth, Change);
                              T2 := Fit.Eye_At (True_J, Swapped, Change);
                              for P of Points loop
                                 declare
                                    P1 : constant Vec3 := Inverse (T1) * (Inverse (Cam0) * P);
                                    P2 : constant Vec3 := Inverse (T2) * (Inverse (Cam0) * P);
                                 begin
                                    if P1 (3) > 0.0 and then P2 (3) > 0.0 then
                                       D.Append
                                         (EF.Sqrt ((W.Cameras (Cam).Fx * (P1 (1) / P1 (3) - P2 (1) / P2 (3))) ** 2
                                                   + (W.Cameras (Cam).Fy * (P1 (2) / P1 (3) - P2 (2) / P2 (3))) ** 2));
                                    end if;
                                 end;
                              end loop;
                              declare
                                 A : Real_Array (1 .. Natural (D.Length));
                              begin
                                 for I in A'Range loop
                                    A (I) := D (I);
                                 end loop;
                                 Put_Line ("      turns" & Change (E)'Image & " and" & Change (La)'Image
                                           & ": the two orders' places of" & A'Length'Image & " points a median"
                                           & Real'Image (if A'Length = 0 then 0.0 else Driver.Stats.Median (A))
                                           & " px apart");
                              end;
                           end;
                        end loop;
                        --  BENCH_SEG_STUDY (the truth's): the earlier joint turned far, by Theta radians, a new
                        --  reference taken there of the points its eye sees, and the later swept from it by the turn
                        --  that moves the view 32 px: the two orders' places of those points, the true joints in each,
                        --  a median so far apart; and the true order's against where the points truly land (the
                        --  model's own check).
                        if Ada.Environment_Variables.Exists ("BENCH_SEG_STUDY") then
                           for Which in 1 .. 2 loop
                           declare
                              Off : constant Positive := (if Which = 1 then E else La);
                              Sw  : constant Positive := (if Which = 1 then La else E);
                           begin
                           for Theta of Real_Array'[0.1, 0.25, 0.5, 1.0] loop
                              for Sign of Real_Array'[1.0, -1.0] loop
                                 declare
                                    Ch_O, Ch_K : Real_Array (1 .. N) := [others => 0.0];
                                    Q_O, Q_K   : Real_Array := Q0;
                                    Sweep_By      : constant Real :=
                                      32.0 * abs Factor (Sw) / Real'Max (Slopes (Order_D (Sw)), Real'Model_Small);
                                    Seg_Pts    : Vec_Vectors.Vector;
                                    D_Swap, D_True : Row_Vectors.Vector;
                                    C_Cam      : FW.Camera renames W.Cameras (Cam);
                                    Cam_O, Cam_K : Rigid;
                                 begin
                                    Ch_O (Off) := Sign * Theta / True_J (Off).C;
                                    Ch_K := Ch_O;
                                    Ch_K (Sw) := Sweep_By;
                                    for C in 1 .. N loop
                                       Q_O (Order_D (C)) := Q0 (Order_D (C)) + Ch_O (C) / Factor (C);
                                       Q_K (Order_D (C)) := Q0 (Order_D (C)) + Ch_K (C) / Factor (C);
                                    end loop;
                                    if (for all J in 1 .. N => Within (J, Q_O (J)) and then Within (J, Q_K (J))) then
                                       Pose_At (W, Path, Q_K);
                                       Cam_K := W.Cameras (Cam).Pose;
                                       Pose_At (W, Path, Q_O);
                                       Cam_O := W.Cameras (Cam).Pose;
                                       for R in 0 .. (C_Cam.Height - 1) / Grid loop
                                          for K in 0 .. (C_Cam.Width - 1) / Grid loop
                                             declare
                                                U   : constant Real := Real (K * Grid) + Real (Grid) / 2.0;
                                                V   : constant Real := Real (R * Grid) + Real (Grid) / 2.0;
                                                Dc  : constant Vec3 :=
                                                  [(U - C_Cam.Cx) / C_Cam.Fx, (V - C_Cam.Cy) / C_Cam.Fy, 1.0];
                                                Dw  : constant Vec3 :=
                                                  Cam_O.Rotation
                                                  * ((1.0 / EF.Sqrt (Dc (1) ** 2 + Dc (2) ** 2 + 1.0)) * Dc);
                                                Hit : Real;
                                                Ok  : Boolean;
                                             begin
                                                Cast (W, Cam_O.Translation, Dw, Hit, Ok);
                                                if Ok and then Hit < Real'Last then
                                                   Seg_Pts.Append (Cam_O.Translation + Hit * Dw);
                                                end if;
                                             end;
                                          end loop;
                                       end loop;
                                       Pose_At (W, Path, Q0);
                                       declare
                                          T_Rel_1 : constant Rigid :=
                                            Inverse (Fit.Eye_At (True_J, Truth, Ch_O))
                                            * Fit.Eye_At (True_J, Truth, Ch_K);
                                          T_Rel_2 : constant Rigid :=
                                            Inverse (Fit.Eye_At (True_J, Swapped, Ch_O))
                                            * Fit.Eye_At (True_J, Swapped, Ch_K);
                                       begin
                                          for P of Seg_Pts loop
                                             declare
                                                Po : constant Vec3 := Inverse (Cam_O) * P;
                                                P1 : constant Vec3 := Inverse (T_Rel_1) * Po;
                                                P2 : constant Vec3 := Inverse (T_Rel_2) * Po;
                                                Pt : constant Vec3 := Inverse (Cam_K) * P;
                                             begin
                                                if P1 (3) > 0.0 and then P2 (3) > 0.0 and then Pt (3) > 0.0 then
                                                   D_Swap.Append
                                                     (EF.Sqrt
                                                        ((C_Cam.Fx * (P1 (1) / P1 (3) - P2 (1) / P2 (3))) ** 2
                                                         + (C_Cam.Fy * (P1 (2) / P1 (3) - P2 (2) / P2 (3))) ** 2));
                                                   D_True.Append
                                                     (EF.Sqrt
                                                        ((C_Cam.Fx * (P1 (1) / P1 (3) - Pt (1) / Pt (3))) ** 2
                                                         + (C_Cam.Fy * (P1 (2) / P1 (3) - Pt (2) / Pt (3))) ** 2));
                                                end if;
                                             end;
                                          end loop;
                                       end;
                                       declare
                                          A : Real_Array (1 .. Natural (D_Swap.Length));
                                          B : Real_Array (1 .. Natural (D_True.Length));
                                       begin
                                          for I in A'Range loop
                                             A (I) := D_Swap (I);
                                             B (I) := D_True (I);
                                          end loop;
                                          Put_Line
                                            ("      segment: c" & Off'Image & " turned" & Real'Image (Sign * Theta)
                                             & " rad, c" & Sw'Image & " swept" & Sweep_By'Image & ":"
                                             & A'Length'Image & " points, the orders' places a median"
                                             & Real'Image (if A'Length = 0 then 0.0 else Driver.Stats.Median (A))
                                             & " px apart, the true order's from the truth"
                                             & Real'Image (if B'Length = 0 then 0.0
                                                          else Driver.Stats.Median (B)));
                                       end;
                                    else
                                       Put_Line ("      segment: c" & Off'Image & " turned" & Real'Image (Sign * Theta)
                                                 & " rad: out of its range");
                                    end if;
                                 end;
                              end loop;
                           end loop;
                           end;
                           end loop;
                        end if;
                     end;
                  end loop;
                  --  Every order scored as the fit's stage 4 scores it (its joints, lens and tracks' depths, the
                  --  sightings of keyframes that moved several joints): where the true order ranks among them.
                  declare
                     Order : Fit.Chain_Order (1 .. N);
                     Count : Natural := 0;
                     Better : Natural := 0;   --  orders that score below the true order
                     True_M : Real := 0.0;
                     Best_M : Real := Real'Last;
                     Best_O : Fit.Chain_Order (1 .. N);
                     function Score (C : Fit.Chain_Order) return Real is
                        D : Row_Vectors.Vector;
                     begin
                        for X of Seen_All loop
                           declare
                              Change : Real_Array (1 .. N);
                              Moved  : Natural := 0;
                              Depth  : constant Real :=
                                (if X.Track <= Natural (Prev.Depths.Length)
                                 then Prev.Depths (Prev.Depths.First_Index + X.Track - 1) else 0.0);
                           begin
                              for J in 1 .. N loop
                                 Change (J) := Rows ((X.Frame - 2) * N + J);
                                 Moved := Moved + Boolean'Pos (Change (J) /= 0.0);
                              end loop;
                              if Moved >= 2 and then Depth > 0.0 then
                                 declare
                                    T     : constant Rigid := Fit.Eye_At (Prev_Joints, C, Change);
                                    P     : constant Vec3 := Depth * Fit.Ray (Prev_L, X.U0, X.V0);
                                    U, V  : Real;
                                    Ahead : Boolean;
                                 begin
                                    Fit.Project (Prev_L, Inverse (T) * P, U, V, Ahead);
                                    if Ahead then
                                       D.Append (abs (U - X.U));
                                       D.Append (abs (V - X.V));
                                    end if;
                                 end;
                              end if;
                           end;
                        end loop;
                        declare
                           A : Real_Array (1 .. Natural (D.Length));
                        begin
                           for I in A'Range loop
                              A (I) := D (I);
                           end loop;
                           return (if A'Length = 0 then Real'Last else Driver.Stats.Median (A));
                        end;
                     end Score;
                     function Next (C : in out Fit.Chain_Order) return Boolean is
                        I : Natural := N - 1;
                     begin
                        while I >= 1 and then C (I) > C (I + 1) loop
                           I := I - 1;
                        end loop;
                        if I = 0 then
                           return False;
                        end if;
                        declare
                           J : Positive := N;
                           T : Positive;
                           Lo : Positive := I + 1;
                           Hi : Positive := N;
                        begin
                           while C (J) < C (I) loop
                              J := J - 1;
                           end loop;
                           T := C (I);
                           C (I) := C (J);
                           C (J) := T;
                           while Lo < Hi loop
                              T := C (Lo);
                              C (Lo) := C (Hi);
                              C (Hi) := T;
                              Lo := Lo + 1;
                              Hi := Hi - 1;
                           end loop;
                        end;
                        return True;
                     end Next;
                  begin
                     True_M := Score (Truth);
                     for I in 1 .. N loop
                        Order (I) := I;
                     end loop;
                     loop
                        declare
                           M : constant Real := Score (Order);
                        begin
                           Count := Count + 1;
                           Better := Better + Boolean'Pos (M < True_M);
                           if M < Best_M then
                              Best_M := M;
                              Best_O := Order;
                           end if;
                        end;
                        exit when not Next (Order);
                     end loop;
                     declare
                        Line : Unbounded_String;
                     begin
                        for C of Best_O loop
                           Append (Line, C'Image);
                        end loop;
                        Put_Line ("  rank seed" & Seed'Image & " eye " & Name
                                  & ": with the fit's joints the true order"
                                  & " scores" & Natural'Image (Natural (1000.0 * True_M)) & "/1000 px, below it"
                                  & Better'Image & " of" & Count'Image & " orders; the least" & To_String (Line)
                                  & Milli (Best_M));
                     end;
                  end;
                  --  And each of the two orders kept as told and fitted again from the fit's last joints and lens
                  --  (Fit_Start with the order told: the stages after the search, every joint's terms refined in that
                  --  order): which of the two the sightings fit the better once the joints are fitted to each.
                  for Which in 1 .. 2 loop
                     declare
                        F       : constant Positive := Frames;
                        Changes : Driver.Numerics.Arrays.Real_Matrix (1 .. F, 1 .. N) := [others => [others => 0.0]];
                        Asked   : Fit.Flag_Matrix (1 .. F, 1 .. N) := [others => [others => False]];
                        Seen    : Fit.Sighting_Array (1 .. Natural (Seen_All.Length));
                        Start   : Fit.Fit_Start (N);
                        Joints  : Fit.Joint_Array (1 .. N);
                        Chain   : Fit.Chain_Order (1 .. N);
                        L       : Fit.Lens;
                        Report  : Fit.Fit_Report;
                     begin
                        for K in 2 .. F loop
                           for J in 1 .. N loop
                              Changes (K, J) := Rows ((K - 2) * N + J);
                              Asked (K, J) := Flags ((K - 2) * N + J) > 0.0;
                           end loop;
                        end loop;
                        for I in Seen'Range loop
                           Seen (I) := Seen_All (I);
                        end loop;
                        Start.Valid := True;
                        Start.Joints := Prev_Joints;
                        Start.Chain := (if Which = 1 then Truth else Prev_Chain);
                        Start.L := Prev_L;
                        Start.Focal_Within := Prev.Focal_Within;
                        Start.Searched_Least := Prev.Searched_Least;
                        Start.Searched_Most := Prev.Searched_Most;
                        Start.Profiled_Over := Prev.Profiled_Over;
                        Start.Order_Told := False;
                        Start.Slide_Turns := Fit.Slide_Turns_Of (Prev, N);
                        Start.Fixed := True;
                        Start.Whole := True;
                        Fit.Fit (Changes, Asked, [for C in 1 .. N => abs Factor (C) * Visible (Order_D (C))], Seen,
                                 W.Cameras (Cam).Width, W.Cameras (Cam).Height, 0, Joints, Chain, L, Report, Who => "",
                                 From => Start);
                        declare
                           Line : Unbounded_String;
                        begin
                           for C of Chain loop
                              Append (Line, C'Image);
                           end loop;
                           Put_Line ("  refit seed" & Seed'Image & " eye " & Name & " from the "
                                     & (if Which = 1 then "true" else "told") & " order: kept" & To_String (Line)
                                     & ", told " & Report.Order_Told'Image & ", median px x 1000"
                                     & Natural'Image (Natural (1000.0 * Report.Median_Px)) & " over"
                                     & Report.Used'Image & " sightings, every sighting's"
                                     & Natural'Image (Natural (1000.0 * Report.All_Median_Px))
                                     & ", its keyframes of several joints"
                                     & Milli (Multi_Score (Joints, L, Report, Chain)));
                        end;
                     end;
                  end loop;
                  Put_Line ("  truth seed" & Seed'Image & " eye " & Name
                            & ": the true joints put the sightings, median px x 1000,"
                            & " of multi-joint keyframes: true order"
                            & Natural'Image (Natural (1000.0 * Median_Of (Truth, True)))
                            & ", the fit's order" & Natural'Image (Natural (1000.0 * Median_Of (Prev_Chain, True)))
                            & "; of all: true order" & Natural'Image (Natural (1000.0 * Median_Of (Truth, False))));
               end;
            end if;
            declare
               --  Right when the order told is the truth, or one the true joints make the same eye of at every turn of
               --  every pair and of all of them at once (joints that commute: two slides, a slide along a turn's axis).
               function Equivalent return Boolean is
                  Cam0   : constant Rigid := W.Cameras (Cam).Pose;
                  True_J : Fit.Joint_Array (1 .. N);
                  Tiny   : constant Real := EF.Sqrt (Real'Model_Epsilon);
                  function Same (D : Real_Array) return Boolean is
                     A : constant Rigid := Fit.Eye_At (True_J, Prev_Chain, D);
                     B : constant Rigid := Fit.Eye_At (True_J, Truth, D);
                     Scale : Real := 1.0;
                  begin
                     for K in 1 .. 3 loop
                        Scale := Real'Max (Scale, abs A.Translation (K));
                     end loop;
                     for P in 1 .. 3 loop
                        if abs (A.Translation (P) - B.Translation (P)) > Tiny * Scale then
                           return False;
                        end if;
                        for Q in 1 .. 3 loop
                           if abs (A.Rotation (P, Q) - B.Rotation (P, Q)) > Tiny then
                              return False;
                           end if;
                        end loop;
                     end loop;
                     return True;
                  end Same;
               begin
                  if not Have_Prev then
                     return False;
                  end if;
                  Pose_At (W, Path, Q0);
                  for C in 1 .. N loop
                     declare
                        Lk    : FW.Link renames W.Links (Path (Order_D (C)));
                        Axis  : constant Vec3 := Transpose (Cam0.Rotation) * (Lk.Pose.Rotation * Lk.Axis);
                        At_P  : constant Vec3 := Inverse (Cam0) * Lk.Pose.Translation;
                        Along : constant Real := At_P (1) * Axis (1) + At_P (2) * Axis (2) + At_P (3) * Axis (3);
                     begin
                        True_J (C) := (W => Axis, P => At_P - Along * Axis, C => Lk.Unit / Factor (C),
                                       Slide => Lk.Kind = FW.Prismatic);
                     end;
                  end loop;
                  for A in 1 .. N loop
                     for B in A .. N loop
                        for Row in 0 .. 3 loop
                           declare
                              D : Real_Array (1 .. N) := [others => 0.0];
                           begin
                              for C in 1 .. N loop
                                 if C = A or else C = B or else A = B then
                                    D (C) := (if (C = A) = (Row mod 2 = 1) then -1.0 else 1.0)
                                             * Factor (C) * Reached (Order_D (C));
                                 end if;
                              end loop;
                              if not Same (D) then
                                 return False;
                              end if;
                           end;
                        end loop;
                     end loop;
                  end loop;
                  return True;
               end Equivalent;
               Right : constant Boolean := Prev_Chain = Truth or else Equivalent;
               Line  : Unbounded_String;
               Want  : Unbounded_String;
            begin
               for C of Prev_Chain loop
                  Append (Line, C'Image);
               end loop;
               for C of Truth loop
                  Append (Want, C'Image);
               end loop;
               --  How many of the points stand off the table's plane (a box, a wall): what tells a lens from its
               --  plane where the eye turns about one axis only.
               declare
                  Off_Table : Natural := 0;
               begin
                  for X of Points loop
                     if abs (X (3) - W.Table_Z) > 1.0E-3 then
                        Off_Table := Off_Table + 1;
                     end if;
                  end loop;
                  Put_Line ("  seed" & Seed'Image & " eye " & Name & ": " & Off_Table'Image & " of"
                            & Points.Length'Image
                            & " points off the table's plane");
               end;
               Put_Line ("seed" & Seed'Image & " eye " & Name & ":" & N'Image & " joints, "
                         & (if not Have_Prev then "NOT FITTED"
                            elsif Told and then Right then "TOLD RIGHT"
                            elsif Told then "TOLD WRONG"
                            elsif Right then "untold, best right"
                            else "untold, best wrong")
                         & ", chain" & To_String (Line) & ", truth" & To_String (Want) & ", fits" & Fits'Image
                         & (if Have_Prev and then not Prev.Fitted then ", NOT DETERMINED (" & To_String (Prev.Why) & ")"
                            else "")
                         & ", keyframes" & Frames'Image & ", stalled rounds" & Stalled'Image & ", in"
                         & Duration'Image (Ada.Calendar."-" (Ada.Calendar.Clock, Began)) & " s");
            end;
         end;
      end;
   end Bench_Eye;

begin
   if Ada.Command_Line.Argument_Count = 0 then
      Put_Line (Standard_Error, "usage: step2_bench SEED[:ARM]...");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   for I in 1 .. Ada.Command_Line.Argument_Count loop
      declare
         Arg   : constant String := Ada.Command_Line.Argument (I);
         Colon : constant Natural := Index (To_Unbounded_String (Arg), ":");
         Seed  : constant Natural := Natural'Value (if Colon = 0 then Arg else Arg (Arg'First .. Colon - 1));
         Only  : constant Natural := (if Colon = 0 then 0 else Natural'Value (Arg (Colon + 1 .. Arg'Last)));
         W     : FW.World;
         T     : FW.Template;
         Kind  : constant String :=
           (if Ada.Environment_Variables.Exists ("BENCH_CLASS") then Ada.Environment_Variables.Value ("BENCH_CLASS")
            else "table");
         Eyes  : Natural := 0;
      begin
         T.Hands := [FW.Jaw, FW.Jaw];
         T.Servo := True;
         T.Ticks := 4096;
         T.Class := (if Kind = "humanoid" then FW.Humanoid elsif Kind = "base" then FW.Base_And_Arm
                     elsif Kind = "drone" then FW.Drone else FW.Table_Arms);
         FW.Build (W, Seed, Positive (Setting ("BENCH_WIDTH", 320.0)), Positive (Setting ("BENCH_HEIGHT", 240.0)), T);
         --  Every eye the body carries, in the order the body has them.
         for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
            if W.Cameras (C).Link > 0 then
               Eyes := Eyes + 1;
               if Only = 0 or else Only = Eyes then
                  Bench_Eye (W, Seed, C);
               end if;
            end if;
         end loop;
      end;
   end loop;
end Driver.Robot.Step2_Bench;
