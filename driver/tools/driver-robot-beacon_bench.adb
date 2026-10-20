with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Environment_Variables;
with Ada.Exceptions;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Log;
with Driver.Numerics;
with Driver.Robot.Kinematics.Beacon;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Placement;
with Fake_World;

procedure Driver.Robot.Beacon_Bench is
   use Ada.Text_IO;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   package FW renames Fake_World;
   package Beacon renames Driver.Robot.Kinematics.Beacon;
   package Placement renames Driver.Robot.Kinematics.Placement;
   package EF renames Ada.Numerics.Long_Elementary_Functions;
   use type FW.Joint_Kind;

   function Setting (Name : String; Default : Real) return Real is
     (if Ada.Environment_Variables.Exists (Name) then Real'Value (Ada.Environment_Variables.Value (Name)) else Default);

   Noise  : constant Real := Setting ("BEACON_NOISE", 1.5);    --  px per axis, every flick's centre
   Unit   : constant Real := Setting ("BEACON_UNIT", 0.012);   --  metres a length unit of the first arm's frame is
   Unit_2 : constant Real := Setting ("BEACON_UNIT2", 0.0156);  --  and of the other arms' (their units are their own)
   Place_Arms : constant Boolean := Ada.Environment_Variables.Exists ("BEACON_PLACE");   --  the other arms are put into the world
   Hidden : constant Real := Setting ("BEACON_HIDE", 0.03);    --  metres of the line to the point a solid may take
   Rounds : constant Natural := Natural (Setting ("BEACON_ROUNDS", 3.0));
   Fast   : constant Boolean := Ada.Environment_Variables.Exists ("BEACON_FAST");    --  the wide strategies only
   Trace  : constant Boolean := Ada.Environment_Variables.Exists ("BEACON_TRACE");   --  every fit, as it is made

   Rng : Ada.Numerics.Float_Random.Generator;
   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Rng)));
   function Gaussian return Real is
      U1 : constant Real := Real'Max (Uniform, 1.0E-12);
      U2 : constant Real := Uniform;
   begin
      return EF.Sqrt (-2.0 * EF.Log (U1)) * EF.Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Image (X : Real; Aft : Natural := 1) return String is (Driver.Log.Image (X, Aft));

   package Sight_Vectors is new Ada.Containers.Vectors (Positive, Beacon.Sighting, Beacon."=");
   package Pose_Vectors is new Ada.Containers.Vectors (Positive, Driver.Numerics.Vec3, Driver.Numerics.Arrays."=");

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

   package Pose_Lists is new Ada.Containers.Indefinite_Vectors (Positive, Real_Array);

   --  B's frame in A's.
   function Relative (A, B : Rigid) return Rigid is
     (Rotation    => Transpose (A.Rotation) * B.Rotation,
      Translation => Transpose (A.Rotation) * (B.Translation - A.Translation));

   --  The same frame with its lengths in the arm's units.
   function Scaled (R : Rigid; U : Real) return Rigid is
     (Rotation    => R.Rotation,
      Translation => [R.Translation (1) / U, R.Translation (2) / U, R.Translation (3) / U]);

   procedure Bench_Arm
     (W : in out FW.World; Seed : Natural; Arm : Positive; Wrist, Fixed : Positive;
      First_Wrist : Positive; First_Ref : Rigid)
   is
      Own_Unit : constant Real := (if Arm = 1 then Unit else Unit_2);
      Placing  : constant Boolean := Place_Arms and then Arm > 1;
      Path   : constant Link_List := Chain_Of (W, Wrist);
      N      : constant Natural := Path'Length;
      Closer : constant Natural := W.Closers (Arm);
   begin
      if N < 2 or else Closer = 0 then
         Put_Line ("seed" & Seed'Image & " arm" & Arm'Image & ": fewer than two joints or no closer");
         return;
      end if;
      declare
         Q0         : Real_Array (1 .. N);
         Down, Up   : Real_Array (1 .. N);   --  the room each joint has either side of the reference, in its readings
         Ref_Pose   : Rigid;
         Truth_Pose : Rigid;
         Lens       : constant FW.Camera := W.Cameras (Fixed);
         Truth_Lens : constant Driver.Robot.Kinematics.Fit.Lens :=
           (Fx => Lens.Fx, Fy => Lens.Fy, Cx => Lens.Cx, Cy => Lens.Cy, K1 => 0.0, K2 => 0.0);
         --  Putting the arm into the world: the eye as it is (in the first arm's frame and unit), the tables, the placement.
         Eye_Truth, Place_Truth : Rigid;
         Plane_World, Plane_Arm : Placement.Plane;

         procedure Sight (Q : Real_Array; Seen : out Boolean; S : out Beacon.Sighting) is
            U, V : Real;
         begin
            Pose_At (W, Path, Q);
            Land (W, Fixed, W.Links (Closer).Pose.Translation, U, V, Seen);
            S := (Carrier => Scaled (Relative (Ref_Pose, W.Cameras (Wrist).Pose), Own_Unit), U => U, V => V, Sigma => Noise);
            if Seen then
               declare
                  Eye         : constant Vec3 := W.Cameras (Fixed).Pose.Translation;
                  To          : constant Vec3 := W.Links (Closer).Pose.Translation - Eye;
                  Dist        : constant Real := abs To;
                  Hit         : Real;
                  World_Solid : Boolean;
               begin
                  Cast (W, Eye, [To (1) / Dist, To (2) / Dist, To (3) / Dist], Hit, World_Solid);
                  Seen := Hit >= Dist - Hidden;
               end;
            end if;
            if Seen then
               S.U := U + Noise * Gaussian;
               S.V := V + Noise * Gaussian;
            end if;
         end Sight;

         function Errors (R : Beacon.Report) return String is
            Turn : constant Vec3 := Driver.Numerics.Log (Transpose (R.Pose.Rotation) * Truth_Pose.Rotation);
            Away : constant Real := abs Truth_Pose.Translation;
         begin
            return "focal " & Image (100.0 * (R.Lens.Fx / Lens.Fx - 1.0)) & " % (claims +-"
              & Image (100.0 * EF.Sqrt (R.Covariance (1, 1))) & "), centre "
              & Image (EF.Sqrt ((R.Lens.Cx - Lens.Cx) ** 2 + (R.Lens.Cy - Lens.Cy) ** 2)) & " px, place "
              & Image (100.0 * abs (R.Pose.Translation - Truth_Pose.Translation) / Away) & " % of its distance (claims +-"
              & Image (100.0 * EF.Sqrt (R.Covariance (8, 8) + R.Covariance (9, 9) + R.Covariance (10, 10)) / Away)
              & "), turn " & Image (180.0 / Ada.Numerics.Pi * abs Turn) & " deg (claims +-"
              & Image (180.0 / Ada.Numerics.Pi * EF.Sqrt (R.Covariance (5, 5) + R.Covariance (6, 6) + R.Covariance (7, 7)))
              & ")";
         end Errors;

         function Errors_P (R : Placement.Report) return String is
            Turn : constant Vec3 := Driver.Numerics.Log (Transpose (R.Placement.Rotation) * Place_Truth.Rotation);
            Away : constant Real := abs (Eye_Truth.Translation - Place_Truth.Translation);
            C    : Real_Matrix renames R.Covariance;
         begin
            return "turn " & Image (180.0 / Ada.Numerics.Pi * abs Turn) & " deg (claims +-"
              & Image (180.0 / Ada.Numerics.Pi * EF.Sqrt (C (1, 1) + C (2, 2) + C (3, 3))) & "), place "
              & Image (100.0 * abs (R.Placement.Translation - Place_Truth.Translation) / Away)
              & " % of its distance (claims +-" & Image (100.0 * EF.Sqrt (C (4, 4) + C (5, 5) + C (6, 6)) / Away)
              & "), unit " & Image (100.0 * (R.Scale / (Own_Unit / Unit) - 1.0)) & " % (claims +-"
              & Image (100.0 * EF.Sqrt (C (7, 7))) & ")";
         end Errors_P;

         procedure Strategy (Name : String; Ladder : Boolean; Share : Real; Model : Boolean := False) is
            Seen_All         : Sight_Vectors.Vector;
            Visited, Told_At : Natural := 0;
            At_Told, Latest  : Beacon.Report;
            At_Told_P        : Placement.Report;
            Have_Latest      : Boolean := False;
            Poses            : Pose_Lists.Vector;
            Places           : Pose_Vectors.Vector;
            Vis_Down, Vis_Up : Real_Array (1 .. N) := [others => 0.0];
            Q                : Real_Array (1 .. N) := Q0;

            procedure Visit (At_Q : Real_Array; Seen : out Boolean) is
               S : Beacon.Sighting;
            begin
               Visited := Visited + 1;
               Sight (At_Q, Seen, S);
               if Seen then
                  Seen_All.Append (S);
                  if Placing and then Told_At = 0 and then Natural (Seen_All.Length) >= 6 then
                     declare
                        A  : constant Beacon.Sighting_Array := [for I in 1 .. Natural (Seen_All.Length) => Seen_All (I)];
                        RP : Placement.Report;
                     begin
                        Placement.Fit_Arm (A, Truth_Lens, Eye_Truth, Plane_World, Plane_Arm, RP);
                        if Trace then
                           Put_Line ("   " & Name & " at" & Natural'Image (Natural (Seen_All.Length)) & " sightings, "
                                     & (if RP.Determined then "told: " else "untold: ") & Errors_P (RP) & " " & To_String (RP.Why));
                        end if;
                        if RP.Determined then
                           Told_At := Natural (Seen_All.Length);
                           At_Told_P := RP;
                        end if;
                     end;
                  end if;
                  if not Placing and then Told_At = 0 and then Natural (Seen_All.Length) >= 6 then
                     declare
                        A : constant Beacon.Sighting_Array := [for I in 1 .. Natural (Seen_All.Length) => Seen_All (I)];
                        R : Beacon.Report;
                     begin
                        begin
                           Beacon.Fit_Eye (A, Lens.Width, Lens.Height, R);
                        exception
                           when E : others =>
                              Put_Line ("   fit raised " & Ada.Exceptions.Exception_Name (E) & " on" & A'Length'Image & " sightings:");
                              for X of A loop
                                 Put_Line ("     " & Image (X.U, 2) & Image (X.V, 2) & "  carrier " & Image (X.Carrier.Translation (1), 3)
                                           & Image (X.Carrier.Translation (2), 3) & Image (X.Carrier.Translation (3), 3));
                              end loop;
                              raise;
                        end;
                        Latest := R;
                        Have_Latest := True;
                        if Trace then
                           Put_Line ("   " & Name & " at" & Natural'Image (Natural (Seen_All.Length)) & " sightings, "
                                     & (if R.Determined then "told: " else "untold: ") & Errors (R));
                        end if;
                        if R.Determined then
                           Told_At := Natural (Seen_All.Length);
                           At_Told := R;
                        end if;
                     end;
                  end if;
               end if;
            end Visit;

            procedure Add (At_Q : Real_Array) is
            begin
               Poses.Append (At_Q);
               Pose_At (W, Path, At_Q);
               Places.Append (W.Links (Closer).Pose.Translation);
            end Add;

            --  Whether the eye, as the sightings so far place it, is to see the point at the pose: it lands in the picture
            --  in front of the camera.
            function Predicted (At_Q : Real_Array) return Boolean is
            begin
               if not Have_Latest then
                  return True;
               end if;
               Pose_At (W, Path, At_Q);
               declare
                  Carrier : constant Rigid := Scaled (Relative (Ref_Pose, W.Cameras (Wrist).Pose), Own_Unit);
                  In_Ref  : constant Vec3 := Carrier * Latest.Beacon;
                  In_Eye  : constant Vec3 := Inverse (Latest.Pose) * In_Ref;
               begin
                  if In_Eye (3) <= 0.0 then
                     return False;
                  end if;
                  declare
                     U : constant Real := Latest.Lens.Cx + Latest.Lens.Fx * In_Eye (1) / In_Eye (3);
                     V : constant Real := Latest.Lens.Cy + Latest.Lens.Fy * In_Eye (2) / In_Eye (3);
                  begin
                     return U >= 0.0 and then U <= Real (Lens.Width) and then V >= 0.0 and then V <= Real (Lens.Height);
                  end;
               end;
            end Predicted;

            --  The poses of Poses, the farthest from those visited first, until the eye is told.
            procedure Visit_All is
               Used    : array (1 .. Natural (Poses.Length)) of Boolean := [others => False];
               Nearest : array (1 .. Natural (Poses.Length)) of Real := [others => Real'Last];
               Next    : Natural := (if Poses.Is_Empty then 0 else 1);
               Seen    : Boolean;
            begin
               while Next > 0 and then Told_At = 0 loop
                  Used (Next) := True;
                  Visit (Poses (Next), Seen);
                  declare
                     Far : Natural := 0;
                  begin
                     for F in Used'Range loop
                        Nearest (F) := Real'Min (Nearest (F), abs (Places (F) - Places (Next)));
                        if not Used (F) and then (not Model or else Predicted (Poses (F)))
                          and then (Far = 0 or else Nearest (F) > Nearest (Far))
                        then
                           Far := F;
                        end if;
                     end loop;
                     if Far = 0 then   --  none is predicted to be seen: the farthest of those left
                        for F in Used'Range loop
                           if not Used (F) and then (Far = 0 or else Nearest (F) > Nearest (Far)) then
                              Far := F;
                           end if;
                        end loop;
                     end if;
                     Next := Far;
                  end;
               end loop;
            end Visit_All;

            function Room (J : Positive; Way : Real) return Real is (if Way < 0.0 then Down (J) else Up (J));
            function Seen_Room (J : Positive; Way : Real) return Real is (if Way < 0.0 then Vis_Down (J) else Vis_Up (J));

            --  Every two joints half way along Extent of each way.
            procedure Add_Pairs (Fraction : Real; Ladder_Extents : Boolean) is
            begin
               for J in 1 .. N loop
                  for K in J + 1 .. N loop
                     for Wj of Real_Array'[-1.0, 1.0] loop
                        for Wk of Real_Array'[-1.0, 1.0] loop
                           declare
                              Ej : constant Real := (if Ladder_Extents then Seen_Room (J, Wj) else Fraction * Room (J, Wj));
                              Ek : constant Real := (if Ladder_Extents then Seen_Room (K, Wk) else Fraction * Room (K, Wk));
                           begin
                              if Ej > 0.0 and then Ek > 0.0 then
                                 Q := Q0;
                                 Q (J) := Q0 (J) + Wj * 0.5 * Ej;
                                 Q (K) := Q0 (K) + Wk * 0.5 * Ek;
                                 Add (Q);
                              end if;
                           end;
                        end loop;
                     end loop;
                  end loop;
               end loop;
            end Add_Pairs;
         begin
            if not Ladder then
               Add (Q0);
               for J in 1 .. N loop
                  for Way of Real_Array'[-1.0, 1.0] loop
                     if Room (J, Way) > 0.0 then
                        for Part of Real_Array'[0.5, 1.0] loop
                           Q := Q0;
                           Q (J) := Q0 (J) + Way * Part * Share * Room (J, Way);
                           Add (Q);
                        end loop;
                     end if;
                  end loop;
               end loop;
               Add_Pairs (Share, False);
               Visit_All;
            else
               declare
                  Seen  : Boolean;
                  Level : Real;
                  Went  : Real;
               begin
                  Visit (Q0, Seen);
                  for J in 1 .. N loop
                     for Way of Real_Array'[-1.0, 1.0] loop
                        Level := Share * Room (J, Way);
                        Went := 0.0;
                        while Level > 0.0 and then Told_At = 0 loop
                           Q := Q0;
                           Q (J) := Q0 (J) + Way * Level;
                           Visit (Q, Seen);
                           exit when not Seen or else Level >= Room (J, Way);
                           Went := Level;
                           Level := Real'Min (Room (J, Way), 2.0 * Level);
                        end loop;
                        if Way < 0.0 then
                           Vis_Down (J) := Went;
                        else
                           Vis_Up (J) := Went;
                        end if;
                     end loop;
                  end loop;
                  if Told_At = 0 then
                     Add_Pairs (0.0, True);
                     Visit_All;
                  end if;
               end;
            end if;
            declare
               Seen_N : constant Natural := Natural (Seen_All.Length);
               Head   : constant String := "seed" & Seed'Image & " arm" & Arm'Image & " " & Name & ": visited"
                 & Visited'Image & ", seen" & Seen_N'Image;
            begin
               if Told_At > 0 then
                  Put_Line (Head & ", TOLD at" & Told_At'Image & " sightings: "
                            & (if Placing then Errors_P (At_Told_P) else Errors (At_Told)));
               elsif Seen_N < 6 then
                  Put_Line (Head & ", NOT TOLD: too few seen");
               else
                  declare
                     A : constant Beacon.Sighting_Array := [for I in 1 .. Seen_N => Seen_All (I)];
                     R : Beacon.Report;
                     RP : Placement.Report;
                  begin
                     if Placing then
                        Placement.Fit_Arm (A, Truth_Lens, Eye_Truth, Plane_World, Plane_Arm, RP);
                        Put_Line (Head & ", NOT TOLD: " & Errors_P (RP) & " (" & To_String (RP.Why) & ")");
                     else
                        Beacon.Fit_Eye (A, Lens.Width, Lens.Height, R);
                        Put_Line (Head & ", NOT TOLD: " & Errors (R) & " (" & To_String (R.Why) & ")");
                     end if;
                  end;
               end if;
            end;
         end Strategy;
      begin
         for I in 1 .. N loop
            Q0 (I) := W.Links (Path (I)).Q;
            Down (I) := Q0 (I) - W.Links (Path (I)).Lo;
            Up (I) := W.Links (Path (I)).Hi - Q0 (I);
         end loop;
         Pose_At (W, Path, Q0);
         Ref_Pose := W.Cameras (Wrist).Pose;
         Truth_Pose := Scaled (Relative (Ref_Pose, W.Cameras (Fixed).Pose), Own_Unit);
         if Placing then
            declare
               Up_World : constant Vec3 := [0.0, 0.0, 1.0];
               Rel      : constant Rigid := Relative (First_Ref, Ref_Pose);
               Table    : constant Real := W.Table_Z;
               Tilt     : constant Real := Setting ("BEACON_TILT", 0.01);   --  radians, a table's normal as an eye measured it
               Height   : constant Real := Setting ("BEACON_HEIGHT", 0.02);   --  the share of its distance a table's is told to
            begin
               Eye_Truth := Scaled (Relative (First_Ref, W.Cameras (Fixed).Pose), Unit);
               Place_Truth := Scaled (Rel, Unit);
               Plane_World.Normal := Transpose (First_Ref.Rotation) * Up_World;
               Plane_World.Distance := (Table - Up_World * First_Ref.Translation) / Unit;
               Plane_World.Sigma_Normal := Tilt;
               Plane_World.Sigma_Distance := Height * abs Plane_World.Distance;
               Plane_Arm.Normal := Transpose (Ref_Pose.Rotation) * Up_World;
               Plane_Arm.Distance := (Table - Up_World * Ref_Pose.Translation) / Own_Unit;
               Plane_Arm.Sigma_Normal := Tilt;
               Plane_Arm.Sigma_Distance := Height * abs Plane_Arm.Distance;
            end;
         end if;
         for Round in 1 .. Rounds loop
            Ada.Numerics.Float_Random.Reset (Rng, Round);
            if not Fast then
               Strategy ("range 0.05", False, 0.05);
               Strategy ("range 0.15", False, 0.15);
               Strategy ("range 0.4 ", False, 0.4);
               Strategy ("ladder 0.05", True, 0.05);
               Strategy ("ladder 0.15", True, 0.15);
            end if;
            Strategy ("range 1.0 ", False, 1.0);
            Strategy ("model 1.0 ", False, 1.0, Model => True);
         end loop;
      end;
   end Bench_Arm;

begin
   if Ada.Command_Line.Argument_Count = 0 then
      Put_Line (Standard_Error, "usage: beacon_bench SEED[:ARM]...");
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
         Fixed : Natural := 0;
         First_Wrist : Natural := 0;
         First_Ref   : Rigid := Identity;
      begin
         T.Hands := [FW.Jaw, FW.Jaw];
         T.Servo := True;
         T.Ticks := 4096;
         T.Class := FW.Table_Arms;
         FW.Build (W, Seed, 320, 240, T);
         for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
            if W.Cameras (C).Link = 0 and then Fixed = 0 then
               Fixed := C;
            end if;
         end loop;
         for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
            if W.Cameras (C).Link > 0 and then W.Cameras (C).Arm = 1 and then First_Wrist = 0 then
               First_Wrist := C;
            end if;
         end loop;
         if First_Wrist > 0 then
            First_Ref := W.Cameras (First_Wrist).Pose;
         end if;
         for A in 1 .. W.Arms loop
            if Only = 0 or else Only = A then
               declare
                  Wrist : Natural := 0;
               begin
                  for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
                     if W.Cameras (C).Link > 0 and then W.Cameras (C).Arm = A and then Wrist = 0 then
                        Wrist := C;
                     end if;
                  end loop;
                  if Wrist > 0 and then Fixed > 0 then
                     Bench_Arm (W, Seed, A, Wrist, Fixed, First_Wrist, First_Ref);
                  else
                     Put_Line ("seed" & Seed'Image & " arm" & A'Image & ": no eye of its own or no eye fixed in the world");
                  end if;
               end;
            end if;
         end loop;
      end;
   end loop;
end Driver.Robot.Beacon_Bench;
