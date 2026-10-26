with Ada.Environment_Variables;
with Ada.Long_Float_Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Alignment.Keyframes;
with Driver.Bytes;
with Driver.Images;
with Driver.Images.Files;
with Driver.Log;
with Driver.Numerics;
with Driver.Numerics.Dense;
with Driver.Recording;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics.Fit;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Kinematics.Base is

   package Keyframes renames Driver.Alignment.Keyframes;
   use type Driver.Observations.Camera_Id;
   use type Driver.Observations.Group_Id;

   type Rigid_List is array (Positive range <>) of Driver.Numerics.Rigid;
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   --  Radians printed as degrees, to so many digits.
   function Degrees (Radians : Real; Digits_After : Natural) return String is
     (Driver.Log.Image (Radians * 180.0 / Ada.Numerics.Pi, Digits_After));

   function Has_Pixels (I : Driver.Images.Image) return Boolean is
     (Driver.Images.Width (I) > 0 and then Driver.Images.Height (I) > 0);

   function Body_Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base is
      Carrier : constant Group_Id'Base := Carrier_Group (M);
      Best    : Eye_Id'Base := 0;
      Most    : Natural := 0;
      Best_Free : Boolean := False;   --  the best so far is carried by no arm
   begin
      if Carrier = 0 then
         return 0;
      end if;
      for E in 1 .. Eye_Count (M) loop
         declare
            Mount_Of : constant Mount := Eye_Mount (M, Eye_Id (E));
            Free     : constant Boolean := Mount_Of.Kind = Carrier_Carried;
            Count    : Natural := 0;
         begin
            --  An eye the carrier carries is one, whatever its pushes showed of it: the G1's walk turned its head's
            --  view beyond the flow's reach at 39 of the 49 beats the recognition pushed it, the lock-in measured
            --  nothing of it, and an arm's sweep was walked under the other arm's wrist eye instead, or not at all
            --  (n15s2, 10-27: four of the six G1 boots). An eye on another arm, where the carrier's pushes moved it
            --  whole and the arm's own do not: an arm that carries the other (a torso under a head) moves that eye
            --  with it.
            if (Free or else (Mount_Of.Kind = Arm_Carried and then Mount_Of.Arm /= A
                              and then Response (M, Carrier, Eye_Id (E)) = Whole))
              and then Response (M, Arm_Group (M, A), Eye_Id (E)) /= Whole
            then
               for T of M.Eyes (Eye_Id (E)).Textured loop
                  if T then
                     Count := Count + 1;
                  end if;
               end loop;
               if (Free and then not Best_Free) or else (Free = Best_Free and then Count > Most) then
                  Most := Count;
                  Best := Eye_Id (E);
                  Best_Free := Free;
               end if;
            end if;
         end;
      end loop;
      return Best;
   end Body_Eye_Of;

   procedure Begin_Walk (M : in out Model; A : Arm_Id) is
      K : Natural := Index_Of (M, A);
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("walk" & A'Image));
      if K = 0 then
         if Eye_Of (M, A) = 0 then
            return;
         end if;
         M.Kinematics.Append (Arm_Evidence'(Arm => A, Group => Arm_Group (M, A), Eye => Eye_Of (M, A), others => <>));
         K := M.Kinematics.Last_Index;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (K);
      begin
         R.Body_Eye := Body_Eye_Of (M, A);
         R.Walking := R.Body_Eye > 0;
         R.Still_Due := False;
         R.Stills.Clear;
         --  No keyframe while the body walks: the sweep that follows begins anew (Begin_Anew) from its own reference.
         R.Sweeping := False;
      end;
      Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & A'Image & "'s body walks, seen by eye"
                       & M.Kinematics (K).Body_Eye'Image);
   end Begin_Walk;

   procedure Take_Still (M : in out Model; A : Arm_Id) is
      K : constant Natural := Index_Of (M, A);
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("still" & A'Image));
      if K > 0 and then M.Kinematics (K).Walking then
         M.Kinematics (K).Still_Due := True;
      end if;
   end Take_Still;

   procedure Observe (M : Model; R : in out Arm_Evidence; O : Observation) is
   begin
      if R.Still_Due and then R.Walking and then R.Body_Eye > 0
        and then R.Eye <= O.Images.Last_Index and then R.Body_Eye <= O.Images.Last_Index
        and then Driver.Observations.Has_Image (O, R.Eye) and then Driver.Observations.Has_Image (O, R.Body_Eye)
      then
         R.Stills.Append (Body_Still'(Beat => M.Beats, Image => O.Images (R.Eye), Body_Image => O.Images (R.Body_Eye)));
         R.Still_Due := False;
         Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s walk: still"
                          & R.Stills.Length'Image & " at beat" & M.Beats'Image);
      end if;
   end Observe;

   --  Aligns pictures to a chain whose every one stands on its own (a still of the walk: Predict's new direction,
   --  each its own unit change, so that none extrapolates another and the view's common motion asks every one), and
   --  keeps what was found (Set_Of), less the points that stood where they were while the view's common motion moved
   --  them (Keyframes.Stood_And_Moved: Stood): they ride with the eye, the hand in its wrist eye, the robot's own arms
   --  in its head's, and a walk moves them not at all. Stood: how many were so left out.
   procedure Align_Pictures
     (Who      : String;
      C        : in out Keyframes.Chain;
      Pictures : Driver.Images.Image;
      Index, Count : Positive;
      U, V     : Real_Vectors.Vector;
      Still    : Flag_Vectors.Vector;
      Riding   : Keyframes.Flag_Array;
      Set      : out Match_Set;
      Stood    : out Natural)
   is
      Points  : constant Natural := Natural (U.Length);
      Change  : Real_Array (1 .. Count) := [others => 0.0];
      Noise   : constant Real_Array (1 .. Count) := [others => 0.0];
      Shift   : constant Real_Array (1 .. Count) := [others => 0.0];
      Given   : Keyframes.Prediction_Array (1 .. Points);
      Answers : Keyframes.Answer_Array (1 .. Points);
      Agree   : Keyframes.Flag_Array (1 .. Points);
      From    : Keyframes.Source;
   begin
      Change (Index) := 1.0;
      Keyframes.Predict (C, Change, Noise, Shift, Keyframes.No_Offer, Given, From);
      Keyframes.Add (C, Pictures, Change, Given, Answers, Agree, Riding);
      Set_Of (Who, Index, U, V, Still, Given, Answers, Agree, Set);
      Stood := 0;
      declare
         Stood_Here, Moved_Here : Keyframes.Flag_Array (1 .. Points);
      begin
         Keyframes.Stood_And_Moved (C, Stood_Here, Moved_Here);
         for I in 1 .. Points loop
            if Stood_Here (I) and then I - 1 < Natural (Set.Found.Length) and then Set.Found (I - 1) then
               Set.Found.Replace_Element (I - 1, False);
               Stood := Stood + 1;
            end if;
         end loop;
      end;
   end Align_Pictures;

   function Found_Count (S : Match_Set) return Natural is
      Count : Natural := 0;
   begin
      for F of S.Found loop
         if F then
            Count := Count + 1;
         end if;
      end loop;
      return Count;
   end Found_Count;

   procedure Align_Stills
     (Who            : String;
      Arm_Reference  : Driver.Images.Image;
      Body_Reference : Driver.Images.Image;
      Arm_U, Arm_V   : Real_Vectors.Vector;
      Body_U, Body_V : Real_Vectors.Vector;
      Arm_Still      : Flag_Vectors.Vector;
      Arm_Riding     : Flag_Vectors.Vector;
      Stills         : Body_Still_Vectors.Vector;
      Matches, Body_Matches : out Match_Set_Vectors.Vector)
   is
      Count  : constant Natural := Natural (Stills.Length);
      Arm_N  : constant Natural := Natural (Arm_U.Length);
      Body_N : constant Natural := Natural (Body_U.Length);
      Arm_Points  : Keyframes.Point_Array (1 .. Arm_N);
      Body_Points : Keyframes.Point_Array (1 .. Body_N);
      Riding   : Keyframes.Flag_Array (1 .. Arm_N);
      None     : constant Keyframes.Flag_Array (1 .. Body_N) := [others => False];
      No_Still : Flag_Vectors.Vector;
      Chain, Body_Chain : Keyframes.Chain;
   begin
      Matches.Clear;
      Body_Matches.Clear;
      if Count = 0 or else Arm_N = 0 or else Body_N = 0 then
         return;
      end if;
      for I in 1 .. Arm_N loop
         Arm_Points (I) := (U => Arm_U (I - 1), V => Arm_V (I - 1));
         Riding (I) := I - 1 < Natural (Arm_Riding.Length) and then Arm_Riding (I - 1);
      end loop;
      for J in 1 .. Body_N loop
         Body_Points (J) := (U => Body_U (J - 1), V => Body_V (J - 1));
      end loop;
      Keyframes.Begin_Reference (Chain, Arm_Reference, Arm_Points);
      Keyframes.Begin_Reference (Body_Chain, Body_Reference, Body_Points);
      for K in 1 .. Count loop
         declare
            Set, Body_Set : Match_Set;
            Stood, Body_Stood : Natural;
         begin
            Align_Pictures (Who, Chain, Stills (K).Image, K, Count, Arm_U, Arm_V, Arm_Still, Riding, Set, Stood);
            Align_Pictures (Who & ", the body's eye", Body_Chain, Stills (K).Body_Image, K, Count, Body_U, Body_V,
                            No_Still, None, Body_Set, Body_Stood);
            Driver.Log.Line (Driver.Log.Robot, "kinematics: " & Who & ", still" & K'Image & " (beat"
                             & Stills (K).Beat'Image & "):" & Found_Count (Set)'Image & " of" & Arm_N'Image
                             & " of the arm's eye's points found (" & Stood'Image & " more stood with the eye),"
                             & Found_Count (Body_Set)'Image & " of" & Body_N'Image & " of the body's eye's ("
                             & Body_Stood'Image & " more stood)");
            Matches.Append (Set);
            Body_Matches.Append (Body_Set);
         end;
      end loop;
      Keyframes.Release (Chain);
      Keyframes.Release (Body_Chain);
   end Align_Stills;

   --  Per point of one eye, whether it rides with the eye: at every still whose points moved (the median of their
   --  displacements from the reference beyond the two-dimensional gate at Z of the median sigma they state), it was
   --  found where it stood, within that gate of its own sigma; and it was found at one such still at least. A point
   --  of the world moves with the view at some still of a walk, the hand in a wrist eye at none: the hall's G1 walked
   --  in the simulator under its wrist eyes had 35 to 40 % of their points on its own hand, found where they stood
   --  as the table moved 2 to 20 px, and its stills adjusted with them put the head's eye 7 to 158 degrees from its
   --  turn on the wrist's (walkdump.py, seeds 1 to 3, 10-27).
   function Riding_Points (U, V : Real_Vectors.Vector; Matches : Match_Set_Vectors.Vector) return Flag_Vectors.Vector is
      Gate   : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
      N      : constant Natural := Natural (U.Length);
      Moved_Still : array (1 .. Natural'Max (1, Natural (Matches.Length))) of Boolean := [others => False];
      Result : Flag_Vectors.Vector;
   begin
      for S in 1 .. Natural (Matches.Length) loop
         declare
            Set    : Match_Set renames Matches (S);
            Shifts, Sigmas : Real_Access := new Real_Array (1 .. Natural'Max (1, N));
            K      : Natural := 0;
         begin
            for I in 0 .. Natural'Min (N, Natural (Set.Found.Length)) - 1 loop
               if Set.Found (I) then
                  K := K + 1;
                  Shifts (K) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                    ((Set.To_U (I) - U (I)) ** 2 + (Set.To_V (I) - V (I)) ** 2);
                  Sigmas (K) := Set.Sigma (I);
               end if;
            end loop;
            Moved_Still (S) := K > 0
              and then Driver.Uncertain.Significant
                         (Gate, Driver.Stats.Median (Shifts (1 .. K)), Driver.Stats.Median (Sigmas (1 .. K)));
            Free (Shifts);
            Free (Sigmas);
         end;
      end loop;
      for I in 0 .. N - 1 loop
         declare
            Seen_Moved : Boolean := False;   --  found at a still whose points moved
            Stood      : Boolean := True;    --  and where it stood at every such still
         begin
            for S in 1 .. Natural (Matches.Length) loop
               declare
                  Set : Match_Set renames Matches (S);
               begin
                  if Moved_Still (S) and then I < Natural (Set.Found.Length) and then Set.Found (I) then
                     Seen_Moved := True;
                     if Driver.Uncertain.Significant
                         (Gate, Ada.Numerics.Long_Elementary_Functions.Sqrt
                                  ((Set.To_U (I) - U (I)) ** 2 + (Set.To_V (I) - V (I)) ** 2),
                          Set.Sigma (I))
                     then
                        Stood := False;
                     end if;
                  end if;
               end;
            end loop;
            Result.Append (Seen_Moved and then Stood);
         end;
      end loop;
      return Result;
   end Riding_Points;

   procedure Calibrate_Stills
     (Arm_U, Arm_V, Body_U, Body_V : Real_Vectors.Vector;
      Matches, Body_Matches : Match_Set_Vectors.Vector;
      Start         : Driver.Rig.Lens;
      Body_Centre   : Driver.Rig.Place;
      Result        : out Driver.Rig.Calibration;
      Sightings     : out Natural)
   is
      Count  : constant Natural := Result.Stills;
      Arm_N  : constant Natural := Result.First_Points;
      Body_N : constant Natural := Result.Second_Points;
      Total  : Natural := 0;
      --  The points that ride with either eye: in no adjustment of the walk.
      Arm_Rides  : constant Flag_Vectors.Vector := Riding_Points (Arm_U, Arm_V, Matches);
      Body_Rides : constant Flag_Vectors.Vector := Riding_Points (Body_U, Body_V, Body_Matches);
      function Count_Of (F : Flag_Vectors.Vector) return Natural is
         C : Natural := 0;
      begin
         for X of F loop
            if X then
               C := C + 1;
            end if;
         end loop;
         return C;
      end Count_Of;
   begin
      Driver.Log.Line (Driver.Log.Robot, "kinematics: the walk's points riding with their eyes, left out:"
                       & Count_Of (Arm_Rides)'Image & " of" & Arm_N'Image & " of the arm's eye's,"
                       & Count_Of (Body_Rides)'Image & " of" & Body_N'Image & " of the body's");
      for S of Matches loop
         Total := Total + Found_Count (S);
      end loop;
      for S of Body_Matches loop
         Total := Total + Found_Count (S);
      end loop;
      declare
         List : Driver.Rig.Sighting_Array (1 .. Natural'Max (1, Total));
         K    : Natural := 0;
         Arm_Points  : Driver.Rig.Place_Array (1 .. Arm_N);
         Body_Points : Driver.Rig.Place_Array (1 .. Body_N);
      begin
         for I in 1 .. Arm_N loop
            Arm_Points (I) := (Arm_U (I - 1), Arm_V (I - 1));
         end loop;
         for J in 1 .. Body_N loop
            Body_Points (J) := (Body_U (J - 1), Body_V (J - 1));
         end loop;
         for Still in 1 .. Natural'Min (Count, Natural'Min (Natural (Matches.Length), Natural (Body_Matches.Length)))
         loop
            declare
               S  : Match_Set renames Matches (Still);
               Sb : Match_Set renames Body_Matches (Still);
            begin
               for I in 0 .. Natural'Min (Natural (S.Found.Length), Arm_N) - 1 loop
                  if S.Found (I) and then not Arm_Rides (I) then
                     K := K + 1;
                     List (K) := (Eye => 1, Point => I + 1, Still => Still, U => S.To_U (I), V => S.To_V (I),
                                  Sigma => S.Sigma (I));
                  end if;
               end loop;
               for J in 0 .. Natural'Min (Natural (Sb.Found.Length), Body_N) - 1 loop
                  if Sb.Found (J) and then not Body_Rides (J) then
                     K := K + 1;
                     List (K) := (Eye => 2, Point => J + 1, Still => Still, U => Sb.To_U (J), V => Sb.To_V (J),
                                  Sigma => Sb.Sigma (J));
                  end if;
               end loop;
            end;
         end loop;
         Sightings := K;
         if K = 0 or else Count = 0 then
            Result.Ok := False;
            return;
         end if;
         Driver.Rig.Calibrate (Arm_Points, Body_Points, List (1 .. K), Count, Start, Body_Centre, Result);
      end;
   end Calibrate_Stills;

   --  With BD_DUMP_FITS naming a folder (as the fits' inputs are, Kinematics.Fit), the walk's stills as the arm's sweep
   --  takes them up: walk_ARM_BEAT.txt (the reference keyframe's beat) with the sizes, the body eye's principal point,
   --  every point of both eyes (the arm's with whether it stands in its eye and whether it rides with it) and every
   --  still's beat, every real to its last bit; and the pictures beside it, the reference keyframe's of both eyes and
   --  every still's (Driver.Images.Files). For a walk studied alone (tools/walk_replay).
   procedure Dump_Walk (R : Arm_Evidence; Body_Centre : Driver.Rig.Place) is
      use Ada.Text_IO;
      function Trim (S : String) return String is (Ada.Strings.Fixed.Trim (S, Ada.Strings.Both));
      Name : constant String := Ada.Environment_Variables.Value ("BD_DUMP_FITS") & "/walk_" & Trim (R.Arm'Image)
        & "_" & Trim (R.Frames (1).Beat'Image);
      File : File_Type;
      procedure Put_Real (X : Real) is
      begin
         Put (File, " ");
         Ada.Long_Float_Text_IO.Put (File, X, Fore => 1, Aft => Real'Digits + 1, Exp => 1);
      end Put_Real;
      Still_Flags : constant Flag_Vectors.Vector := Unmoving (R);
   begin
      Create (File, Out_File, Name & ".txt");
      Put_Line (File, "who arm" & R.Arm'Image & " body eye" & R.Body_Eye'Image & " reference beat"
                & R.Frames (1).Beat'Image);
      Put_Line (File, "sizes" & R.Stills.Length'Image & R.Query_U.Length'Image & R.Body_Query_U.Length'Image);
      Put (File, "centre");
      Put_Real (Body_Centre.U);
      Put_Real (Body_Centre.V);
      New_Line (File);
      Put_Line (File, "arm");
      for I in 0 .. Natural (R.Query_U.Length) - 1 loop
         Put_Real (R.Query_U (I));
         Put_Real (R.Query_V (I));
         Put (File, Natural'Image
                (Boolean'Pos (I < Natural (Still_Flags.Length) and then Still_Flags (I))));
         Put (File, Natural'Image (Boolean'Pos (not Moving_Point (R, I))));
         New_Line (File);
      end loop;
      Put_Line (File, "body");
      for J in 0 .. Natural (R.Body_Query_U.Length) - 1 loop
         Put_Real (R.Body_Query_U (J));
         Put_Real (R.Body_Query_V (J));
         New_Line (File);
      end loop;
      Put_Line (File, "stills");
      for S of R.Stills loop
         Put_Line (File, Natural'Image (S.Beat));
      end loop;
      Close (File);
      Driver.Images.Files.Write_PPM (Name & "_ref_arm.ppm", R.Frames (1).Image);
      Driver.Images.Files.Write_PPM (Name & "_ref_body.ppm", R.Frames (1).Body_Image);
      for K in 1 .. Natural (R.Stills.Length) loop
         Driver.Images.Files.Write_PPM (Name & "_still_" & Trim (K'Image) & "_arm.ppm", R.Stills (K).Image);
         Driver.Images.Files.Write_PPM (Name & "_still_" & Trim (K'Image) & "_body.ppm", R.Stills (K).Body_Image);
      end loop;
   end Dump_Walk;

   --  The body eye's principal point taken for its lens: its picture's middle.
   function Body_Centre_Of (M : Model; R : Arm_Evidence) return Driver.Rig.Place is
     ((Real (M.Eyes (R.Body_Eye).Grid.Width) / 2.0, Real (M.Eyes (R.Body_Eye).Grid.Height) / 2.0));

   --  The stills of the walk aligned to both eyes' reference pictures (the reference keyframe's), and adjusted
   --  together (Driver.Rig.Calibrate): the first lens the arm's fit's where it has one, else none to start from.
   procedure Calibrate (M : Model; R : in out Arm_Evidence) is
      Count  : constant Natural := Natural (R.Stills.Length);
      Arm_N  : constant Natural := Natural (R.Query_U.Length);
      Body_N : constant Natural := Natural (R.Body_Query_U.Length);
      C      : Driver.Rig.Calibration (Count, Arm_N, Body_N);
      K      : Natural;
      W      : constant Natural := Driver.Images.Width (R.Frames (1).Image);
      H      : constant Natural := Driver.Images.Height (R.Frames (1).Image);
      Fitted : constant Boolean := R.Result.Fitted and then R.Result.Lens.Fx > 0.0 and then R.Result.Lens.Fy > 0.0;
      Start  : constant Driver.Rig.Lens :=
        (if Fitted then (Focal => Ada.Numerics.Long_Elementary_Functions.Sqrt (R.Result.Lens.Fx * R.Result.Lens.Fy),
                         Cx => R.Result.Lens.Cx, Cy => R.Result.Lens.Cy)
         else (Focal => 0.0, Cx => Real (W) / 2.0, Cy => Real (H) / 2.0));
   begin
      Calibrate_Stills (R.Query_U, R.Query_V, R.Body_Query_U, R.Body_Query_V, R.Still_Matches, R.Body_Still_Matches,
                        Start, Body_Centre_Of (M, R), C, K);
      R.Calibrated := True;
      R.Rig_With_Fit := Fitted;
      R.Rig_Ok := False;
      R.Rig_Stills := Count;
      if K = 0 then
         Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s walk: no still found a point");
         return;
      end if;
      R.Rig_Ok := C.Ok;
      --  A walk calibrated anew: its shifts are judged again at the arm's next fit, which is due now.
      R.Base_Verdict := Unjudged;
      R.Revision := R.Revision + 1;
      R.Rig_X := C.Second_On_First;
      R.Rig_Arm_Lens := C.First_Lens;
      R.Rig_Body_Lens := C.Second_Lens;
      R.Rig_Turn_Sigma := C.Turn_Sigma;
      R.Rig_Misfit := C.Misfit;
      R.Rig_Cov := C.Covariance;
      R.Rig_Arm_Inverse.Clear;
      R.Rig_Body_Inverse.Clear;
      --  A point no still saw within the cut is not placed: minus one.
      for I in 1 .. Arm_N loop
         R.Rig_Arm_Inverse.Append (if C.First_Placed (I) > 0 then C.First_Inverse (I) else -1.0);
      end loop;
      for J in 1 .. Body_N loop
         R.Rig_Body_Inverse.Append (if C.Second_Placed (J) > 0 then C.Second_Inverse (J) else -1.0);
      end loop;
      declare
         Placed_Arm, Placed_Body : Natural := 0;
      begin
         for X of R.Rig_Arm_Inverse loop
            if X >= 0.0 then
               Placed_Arm := Placed_Arm + 1;
            end if;
         end loop;
         for X of R.Rig_Body_Inverse loop
            if X >= 0.0 then
               Placed_Body := Placed_Body + 1;
            end if;
         end loop;
         Driver.Log.Line
           (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s walk, " & Count'Image & " stills and"
            & K'Image & " sightings"
            & (if Fitted then ", its fit's lens to start from" else ", no lens to start from")
            & ": " & (if C.Ok then "calibrated" else "NOT calibrated") & " at a cost of "
            & Driver.Log.Image (C.Cost, 1) & ", eye" & R.Body_Eye'Image & " turned "
            & Degrees (Angle (C.Second_On_First.Rotation), 2) & " deg on the arm's eye (+- "
            & Degrees (C.Turn_Sigma, 3) & "), at " & Driver.Log.Image (C.Second_On_First.Translation (1), 3) & " "
            & Driver.Log.Image (C.Second_On_First.Translation (2), 3) & " "
            & Driver.Log.Image (C.Second_On_First.Translation (3), 3)
            & " in the walk's scale; focal lengths " & Driver.Log.Image (C.First_Lens.Focal, 1) & " and "
            & Driver.Log.Image (C.Second_Lens.Focal, 1) & " px; misfit " & Driver.Log.Image (C.Misfit, 2)
            & " sigmas, the residuals' scale " & Driver.Log.Image (C.Scale, 2) & " of them, " & C.Kept'Image
            & " of" & C.Used'Image & " sightings within Z of it;" & Placed_Arm'Image
            & " of" & Arm_N'Image & " arm points and" & Placed_Body'Image & " of" & Body_N'Image
            & " body points placed");
      end;
   end Calibrate;

   procedure Align (M : Model; R : in out Arm_Evidence) is
   begin
      if R.Body_Eye = 0 or else R.Aligned = 0 or else R.Frames.Is_Empty or else R.Body_Query_U.Is_Empty
        or else R.Query_U.Is_Empty
      then
         return;
      end if;
      --  The walk's stills, once the reference keyframe (with the body eye's picture) is taken.
      if not R.Stills_Aligned and then not R.Stills.Is_Empty then
         declare
            Riding : Flag_Vectors.Vector;
         begin
            --  The points that ride with the arm's eye (its hand) tell nothing of the walk.
            for I in 0 .. Natural (R.Query_U.Length) - 1 loop
               Riding.Append (not Moving_Point (R, I));
            end loop;
            if Ada.Environment_Variables.Exists ("BD_DUMP_FITS") then
               Dump_Walk (R, Body_Centre_Of (M, R));
            end if;
            Align_Stills ("arm" & R.Arm'Image & "'s walk", R.Frames (1).Image, R.Frames (1).Body_Image, R.Query_U,
                          R.Query_V, R.Body_Query_U, R.Body_Query_V, Unmoving (R), Riding, R.Stills, R.Still_Matches,
                          R.Body_Still_Matches);
            for K in 1 .. Natural (R.Stills.Length) loop
               R.Stills (K).Image := Driver.Images.No_Image;
               R.Stills (K).Body_Image := Driver.Images.No_Image;
            end loop;
            R.Stills_Aligned := True;
         end;
         Calibrate (M, R);
      elsif R.Stills_Aligned and then R.Calibrated and then not R.Rig_Ok and then not R.Rig_With_Fit
        and then R.Result.Fitted
      then
         --  The stills told nothing with no lens to start from; once more, from the arm's first fit's.
         Calibrate (M, R);
      end if;
      --  The body eye's picture at every keyframe aligned so far.
      if R.Body_Aligned = 0 and then Has_Pixels (R.Frames (1).Body_Image) then
         declare
            Body_N : constant Natural := Natural (R.Body_Query_U.Length);
            Body_Points : Keyframes.Point_Array (1 .. Body_N);
         begin
            for J in 1 .. Body_N loop
               Body_Points (J) := (U => R.Body_Query_U (J - 1), V => R.Body_Query_V (J - 1));
            end loop;
            Keyframes.Begin_Reference (R.Body_Chain, R.Frames (1).Body_Image, Body_Points);
            R.Body_Aligned := 1;
         end;
      end if;
      if R.Body_Aligned > 0 then
         declare
            N : constant Natural := M.Groups (R.Group).Size;
            Noise : Real_Array (1 .. N);
            Shift : constant Real_Array (1 .. N) := [others => 0.0];
            Body_N : constant Natural := Natural (R.Body_Query_U.Length);
            No_Still : Flag_Vectors.Vector;
         begin
            for C in 1 .. N loop
               Noise (C) := Channels.Noise (M, R.Group, C);
            end loop;
            while R.Body_Aligned < R.Aligned and then (for all X of Noise => X < Real'Last) loop
               declare
                  F : constant Positive := R.Body_Aligned + 1;
                  Set : Match_Set := (Frame => F, others => <>);
               begin
                  if Has_Pixels (R.Frames (F).Body_Image) then
                     declare
                        Change  : Real_Array (1 .. N);
                        Given   : Keyframes.Prediction_Array (1 .. Body_N);
                        Answers : Keyframes.Answer_Array (1 .. Body_N);
                        Agree   : Keyframes.Flag_Array (1 .. Body_N);
                        From    : Keyframes.Source;
                     begin
                        for C in 1 .. N loop
                           Change (C) := R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1);
                        end loop;
                        --  How far the body's eye's view moves per unit of the arm's channels is not known: every
                        --  keyframe's is asked from the view's common motion, or from the keyframes before it where
                        --  they tell it.
                        Keyframes.Predict (R.Body_Chain, Change, Noise, Shift, Keyframes.No_Offer, Given, From);
                        Keyframes.Add (R.Body_Chain, R.Frames (F).Body_Image, Change, Given, Answers, Agree);
                        Set_Of ("arm" & R.Arm'Image & "'s body", F, R.Body_Query_U, R.Body_Query_V, No_Still, Given,
                                Answers, Agree, Set);
                     end;
                  end if;
                  R.Body_Matches.Append (Set);
                  R.Frames (F).Body_Image := Driver.Images.No_Image;
                  R.Body_Aligned := F;
               end;
            end loop;
         end;
      end if;
   end Align;

   --  The rows Driver.Rig.Locate takes: per point placed, its ray through the lens and its inverse depth, and where
   --  the keyframe saw it; the points the set found and the walk placed.
   procedure Locate_Eye
     (L : Driver.Rig.Lens; U0, V0 : Real_Vectors.Vector; Inverse : Real_Vectors.Vector; S : Match_Set;
      Start : Driver.Numerics.Rigid; Result : out Driver.Rig.Locate_Result)
   is
      Count  : Natural := 0;
      Stated : Natural := 0;
      Middle : Real := 0.0;   --  the median sigma the answers state, for one that states none
   begin
      for I in 0 .. Natural'Min (Natural (S.Found.Length), Natural (Inverse.Length)) - 1 loop
         if S.Found (I) and then Inverse (I) >= 0.0 then
            Count := Count + 1;
            if S.Sigma (I) > 0.0 then
               Stated := Stated + 1;
            end if;
         end if;
      end loop;
      if Stated > 0 then
         declare
            Sigmas : Real_Array (1 .. Stated);
            K      : Natural := 0;
         begin
            for I in 0 .. Natural'Min (Natural (S.Found.Length), Natural (Inverse.Length)) - 1 loop
               if S.Found (I) and then Inverse (I) >= 0.0 and then S.Sigma (I) > 0.0 then
                  K := K + 1;
                  Sigmas (K) := S.Sigma (I);
               end if;
            end loop;
            Middle := Driver.Stats.Median (Sigmas);
         end;
      end if;
      if Middle <= 0.0 then
         Result := (others => <>);
         return;
      end if;
      declare
         Rays    : Driver.Numerics.Arrays.Real_Matrix (1 .. Count, 1 .. 4);
         Seen_At : Driver.Rig.Place_Array (1 .. Count);
         Sigmas  : Real_Array (1 .. Count);
         K       : Natural := 0;
      begin
         for I in 0 .. Natural'Min (Natural (S.Found.Length), Natural (Inverse.Length)) - 1 loop
            if S.Found (I) and then Inverse (I) >= 0.0 then
               K := K + 1;
               declare
                  Ray : constant Vec3 := Driver.Rig.Ray (L, (U0 (I), V0 (I)));
               begin
                  Rays (K, 1) := Ray (1);
                  Rays (K, 2) := Ray (2);
                  Rays (K, 3) := Ray (3);
                  Rays (K, 4) := Inverse (I);
                  Seen_At (K) := (S.To_U (I), S.To_V (I));
                  Sigmas (K) := (if S.Sigma (I) > 0.0 then S.Sigma (I) else Middle);
               end;
            end if;
         end loop;
         if Count >= 6 then
            Driver.Rig.Locate (L, Rays, Seen_At, Sigmas, Start, Result);
         else
            Result := (others => <>);
         end if;
      end;
   end Locate_Eye;

   --  The arm's eye's turn at keyframe F as the arm's last fit has it (its joints composed by the keyframe's readings'
   --  changes, Fit.Eye_At), where that fit was made from this reference: a start for placing the eye there, which a
   --  keyframe turned far from every one placed before it needs (a wrist's roll); none (Ok False) otherwise.
   procedure Fitted_Turn (R : Arm_Evidence; F : Positive; Turn : out Mat3; Ok : out Boolean) is
      N : constant Natural := Natural (R.Result.Joints.Length);
   begin
      Turn := Identity3;
      Ok := R.Result.Fitted and then N > 0 and then Natural (R.Result.Chain.Length) = N
        and then R.Result.Reference_Beat = R.Frames (1).Beat and then Natural (R.Frames (F).Readings.Length) = N;
      if Ok then
         declare
            J      : Fit.Joint_Array (1 .. N);
            Chain  : Fit.Chain_Order (1 .. N);
            Change : Real_Array (1 .. N);
         begin
            for K in 1 .. N loop
               declare
                  Jf : constant Joint_Fit := R.Result.Joints (R.Result.Joints.First_Index + K - 1);
               begin
                  J (K) := (W => Jf.W, P => Jf.P, C => Jf.C, Slide => Jf.Slide);
               end;
               Chain (K) := R.Result.Chain (R.Result.Chain.First_Index + K - 1);
               Change (K) := R.Frames (F).Readings (K - 1) - R.Frames (1).Readings (K - 1);
            end loop;
            Turn := Fit.Eye_At (J, Chain, Change).Rotation;
         end;
      end if;
   end Fitted_Turn;

   --  A pose moved along the six terms Driver.Rig.Locate states its covariance in: a rotation vector on the left of its
   --  turn (the first three), then its place.
   function Moved_By (P : Driver.Numerics.Rigid; D : Real_Array) return Driver.Numerics.Rigid is
     ((Rotation    => Times (Exp ([D (D'First), D (D'First + 1), D (D'First + 2)]), P.Rotation),
       Translation => [for A in 1 .. 3 => P.Translation (A) + D (D'First + 2 + A)]));

   --  A six-term step along one of its terms.
   function Along (Term : Positive; By : Real) return Real_Array is
     ([for T in 1 .. 6 => (if T = Term then By else 0.0)]);

   --  A pose's six terms from the identity (Moved_By's).
   function Terms_Of (P : Driver.Numerics.Rigid) return Real_Array is
     (declare
         L : constant Vec3 := Driver.Numerics.Log (P.Rotation);
      begin
        [L (1), L (2), L (3), P.Translation (1), P.Translation (2), P.Translation (3)]);

   --  Where a still base would have put a point the arm's eye saw from E, against where it saw it, the base having
   --  moved by Base since the reference: the point along ray R of the reference picture at inverse depth Rho.
   procedure Shift_Of
     (L : Driver.Rig.Lens; Base, E : Driver.Numerics.Rigid; R : Vec3; Rho : Real; Du, Dv : out Real; Ok : out Boolean)
   is
      As_Seen, As_Still : Driver.Rig.Place;
      Ahead_1, Ahead_2  : Boolean;
   begin
      Driver.Rig.Project (L, Driver.Rig.Seen (E, R, Rho), As_Seen, Ahead_1);
      Driver.Rig.Project (L, Driver.Rig.Seen (Inverse (Base) * E, R, Rho), As_Still, Ahead_2);
      Ok := Ahead_1 and then Ahead_2;
      Du := (if Ok then As_Still.U - As_Seen.U else 0.0);
      Dv := (if Ok then As_Still.V - As_Seen.V else 0.0);
   end Shift_Of;

   --  The length of a six-term motion in its covariance (D' C^-1 D, rooted); Real'Last where C is not positive
   --  definite.
   function Whitened (D : Real_Array; C : Driver.Numerics.Arrays.Real_Matrix) return Real is
      Lf : Driver.Numerics.Arrays.Real_Matrix (1 .. 6, 1 .. 6);
      Pd : Boolean;
   begin
      Driver.Numerics.Dense.Cholesky (C, Lf, Pd);
      if not Pd then
         return Real'Last;
      end if;
      declare
         Rhs : constant Driver.Numerics.Arrays.Real_Vector (1 .. 6) := [for T in 1 .. 6 => D (D'First + T - 1)];
         X   : constant Driver.Numerics.Arrays.Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, Rhs);
         Q   : Real := 0.0;
      begin
         for T in 1 .. 6 loop
            Q := Q + Rhs (T) * X (X'First + T - 1);
         end loop;
         return Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (0.0, Q));
      end;
   end Whitened;

   procedure Shift_Sightings (M : Model; R : in out Arm_Evidence) is
      pragma Unreferenced (M);
      Arm_N  : constant Natural := Natural (R.Query_U.Length);
      Frames : constant Natural := Natural'Max (1, R.Aligned);
      subtype Six_By_Six is Driver.Numerics.Arrays.Real_Matrix (1 .. 6, 1 .. 6);
      type Pose_List is array (1 .. Frames) of Driver.Numerics.Rigid;
      type Flag_List is array (1 .. Frames) of Boolean;
      type Cov_List is array (1 .. Frames) of Six_By_Six;
      type Cov_Access is access Cov_List;
      procedure Free is new Ada.Unchecked_Deallocation (Cov_List, Cov_Access);
      type Six_List is array (1 .. Frames) of Real_Array (1 .. 6);
      type Six_Access is access Six_List;
      procedure Free is new Ada.Unchecked_Deallocation (Six_List, Six_Access);
      Arm_Poses, Body_Poses : Pose_List := [others => Identity];
      Arm_Ok, Body_Ok       : Flag_List := [others => False];
      Arm_Cov, Body_Cov     : Cov_Access := new Cov_List'(others => [others => [others => 0.0]]);
      --  Per keyframe, how the body eye's place moves per sigma of the body's focal length's logarithm (its six terms,
      --  as the body eye placed with that lens a sigma either way finds it), and whether both places were found.
      Per_Body_F : Six_Access := new Six_List'(others => [others => 0.0]);
      Body_F_Ok  : Flag_List := [others => False];
      Middle   : Real := 0.0;   --  the median inverse depth of the arm's points the walk placed, for those it did not
      Depths   : Real_Vectors.Vector;   --  per arm point, the inverse depth the shifts take
      Depth_Variances : Real_Vectors.Vector;   --  and its variance
      Turns, Steps : Real_Access := new Real_Array (1 .. Frames);
      Counted  : Natural := 0;
      Pixels   : Real_Access := new Real_Array (1 .. Natural'Max (1, R.Aligned * Natural'Max (1, Arm_N)));
      Sigmas_Px : Real_Access := new Real_Array (1 .. Natural'Max (1, R.Aligned * Natural'Max (1, Arm_N)));
      Moved_Pixels : Natural := 0;
      Refined  : Natural := 0;
      Seen_Count, Moved_Count : Natural := 0;
      Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (6);
      --  The walk's sigmas: of the body eye's frame on the arm's eye (its six terms) and of both focal lengths'
      --  logarithms (Driver.Rig.Calibration's Covariance).
      Rig_Sigma : constant Real_Array (R.Rig_Cov'Range (1)) :=
        [for T in R.Rig_Cov'Range (1) =>
           Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (0.0, R.Rig_Cov (T, T)))];
      --  Rig_Cov's terms after the frame's six: the arm eye's focal length's logarithm, then the body eye's.
      Arm_Focal_Term  : constant Positive := R.Rig_Cov'Last (1) - 1;
      Body_Focal_Term : constant Positive := R.Rig_Cov'Last (1);

      procedure Locate_Body (F : Positive; L : Driver.Rig.Lens; Start : Driver.Numerics.Rigid;
                             Result : out Driver.Rig.Locate_Result) is
      begin
         Locate_Eye (L, R.Body_Query_U, R.Body_Query_V, R.Rig_Body_Inverse, R.Body_Matches (F - 1), Start, Result);
      end Locate_Body;

      --  Every keyframe's eyes placed: the body's eye from the walk's points, the arm's from the given inverse depths,
      --  each from the turn the arm's last fit gives it there, from the pose of the keyframe nearest it in its readings
      --  that was placed, and from none (the best placed of those that place it).
      procedure Place_Eyes (Inverse : Real_Vectors.Vector; Body_Too : Boolean) is
      begin
         Arm_Ok := [others => False];
         Arm_Ok (1) := True;
         Arm_Poses (1) := Identity;
         if Body_Too then
            Body_Ok := [others => False];
            Body_Ok (1) := True;
            Body_F_Ok := [others => False];
         end if;
         for F in 2 .. R.Aligned loop
            if F - 1 <= Natural (R.Body_Matches.Length) then
               if Body_Too then
                  declare
                     Body_Seen : Driver.Rig.Locate_Result;
                  begin
                     Locate_Body (F, R.Rig_Body_Lens, Identity, Body_Seen);
                     Body_Ok (F) := Body_Seen.Ok;
                     Body_Poses (F) := Body_Seen.Pose;
                     Body_Cov (F) := Body_Seen.Covariance;
                     --  The body's lens a sigma either way: how the eye's place found moves with it.
                     if Body_Seen.Ok and then Rig_Sigma (Body_Focal_Term) > 0.0 then
                        declare
                           Up, Down : Driver.Rig.Locate_Result;
                           Lu, Ld   : Driver.Rig.Lens := R.Rig_Body_Lens;
                        begin
                           Lu.Focal := Lu.Focal * Ada.Numerics.Long_Elementary_Functions.Exp
                             (Rig_Sigma (Body_Focal_Term));
                           Ld.Focal := Ld.Focal * Ada.Numerics.Long_Elementary_Functions.Exp
                             (-Rig_Sigma (Body_Focal_Term));
                           Locate_Body (F, Lu, Body_Seen.Pose, Up);
                           Locate_Body (F, Ld, Body_Seen.Pose, Down);
                           if Up.Ok and then Down.Ok then
                              declare
                                 Tu : constant Vec3 :=
                                   Driver.Numerics.Log
                                     (Times (Up.Pose.Rotation, Transposed (Body_Seen.Pose.Rotation)));
                                 Td : constant Vec3 :=
                                   Driver.Numerics.Log
                                     (Times (Down.Pose.Rotation, Transposed (Body_Seen.Pose.Rotation)));
                              begin
                                 Per_Body_F (F) :=
                                   [for T in 1 .. 6 =>
                                      (if T <= 3 then (Tu (T) - Td (T)) / 2.0
                                       else (Up.Pose.Translation (T - 3) - Down.Pose.Translation (T - 3)) / 2.0)];
                                 Body_F_Ok (F) := True;
                              end;
                           end if;
                        end;
                     elsif Body_Seen.Ok then
                        Body_F_Ok (F) := True;   --  the lens's sigma nought: nothing moves with it
                     end if;
                  end;
               end if;
               declare
                  Nearest : Positive := 1;
                  Best    : Real := Real'Last;
                  Fitted  : Mat3;
                  Told    : Boolean;
                  Arm_Seen : Driver.Rig.Locate_Result;
               begin
                  Fitted_Turn (R, F, Fitted, Told);
                  for G in 1 .. F - 1 loop
                     if Arm_Ok (G) then
                        declare
                           D : Real := 0.0;
                        begin
                           for C in 0 .. Natural (R.Frames (F).Readings.Length) - 1 loop
                              D := D + (R.Frames (F).Readings (C) - R.Frames (G).Readings (C)) ** 2;
                           end loop;
                           if D < Best then
                              Best := D;
                              Nearest := G;
                           end if;
                        end;
                     end if;
                  end loop;
                  for Start of Rigid_List'(if Told
                                           then [(Rotation => Fitted, Translation => Zero3), Arm_Poses (Nearest),
                                                 Identity]
                                           else [Arm_Poses (Nearest), Identity])
                  loop
                     declare
                        Again : Driver.Rig.Locate_Result;
                     begin
                        Locate_Eye (R.Rig_Arm_Lens, R.Query_U, R.Query_V, Inverse, R.Matches (F - 1), Start, Again);
                        if Again.Ok and then (not Arm_Seen.Ok or else Again.Misfit < Arm_Seen.Misfit) then
                           Arm_Seen := Again;
                        end if;
                     end;
                  end loop;
                  Arm_Ok (F) := Arm_Seen.Ok;
                  Arm_Poses (F) := Arm_Seen.Pose;
                  Arm_Cov (F) := Arm_Seen.Covariance;
               end;
            end if;
         end loop;
      end Place_Eyes;

      procedure Release is
      begin
         Free (Turns);
         Free (Steps);
         Free (Pixels);
         Free (Sigmas_Px);
         Free (Arm_Cov);
         Free (Body_Cov);
         Free (Per_Body_F);
      end Release;
   begin
      R.Base.Clear;
      if not R.Rig_Ok or else R.Aligned < 2 or else Natural (R.Rig_Arm_Inverse.Length) /= Arm_N
        or else R.Base_Off or else R.Base_Verdict = Dropped
      then
         Release;
         return;
      end if;
      declare
         Placed : Real_Access := new Real_Array (1 .. Natural'Max (1, Arm_N));
         K      : Natural := 0;
      begin
         for X of R.Rig_Arm_Inverse loop
            if X >= 0.0 then
               K := K + 1;
               Placed (K) := X;
            end if;
         end loop;
         if K > 0 then
            Middle := Driver.Stats.Median (Placed (1 .. K));
         end if;
         Free (Placed);
      end;
      --  The eyes from the walk's inverse depths; then every arm point's inverse depth from where the arm's eye, so
      --  placed, saw it at every keyframe (Driver.Rig.Triangulate: the sweep moves the eye a few times as far as the
      --  walk did, and its depths come out several times surer), and the arm's eye placed again from those. A shift's
      --  step part goes with its point's inverse depth: the walk's depths 5 % off left a shift 0.08 px off in the
      --  median and 0.8 px at most of a lean's 2.5 (robot.base.shift, 10-27). Neither depends on the arm's fit, which
      --  the shifts feed.
      Place_Eyes (R.Rig_Arm_Inverse, Body_Too => True);
      for I in 0 .. Arm_N - 1 loop
         declare
            Count : Natural := 0;
            Start : constant Real := (if R.Rig_Arm_Inverse (I) >= 0.0 then R.Rig_Arm_Inverse (I) else Middle);
         begin
            for F in 2 .. R.Aligned loop
               if Arm_Ok (F) and then I < Natural (R.Matches (F - 1).Found.Length) and then R.Matches (F - 1).Found (I)
               then
                  Count := Count + 1;
               end if;
            end loop;
            declare
               Poses   : Driver.Rig.Rigid_Array (1 .. Count);
               Seen_At : Driver.Rig.Place_Array (1 .. Count);
               Sigmas  : Real_Array (1 .. Count);
               K       : Natural := 0;
               Inverse : Real := Start;
               Var     : Real := Real'Last;
               Ok      : Boolean := False;
            begin
               for F in 2 .. R.Aligned loop
                  if Arm_Ok (F) and then I < Natural (R.Matches (F - 1).Found.Length)
                    and then R.Matches (F - 1).Found (I)
                  then
                     K := K + 1;
                     Poses (K) := Arm_Poses (F);
                     Seen_At (K) := (R.Matches (F - 1).To_U (I), R.Matches (F - 1).To_V (I));
                     Sigmas (K) := R.Matches (F - 1).Sigma (I);
                  end if;
               end loop;
               if Count > 0 and then (for all X of Sigmas => X > 0.0) then
                  Driver.Rig.Triangulate
                    (R.Rig_Arm_Lens, Driver.Rig.Ray (R.Rig_Arm_Lens, (R.Query_U (I), R.Query_V (I))), Poses, Seen_At,
                     Sigmas, Start, Inverse, Var, Ok);
               end if;
               --  A point no keyframe placed keeps the walk's inverse depth, or the median's, known to no better than
               --  itself.
               Depths.Append (if Ok then Inverse else Start);
               Depth_Variances.Append (if Ok then Var else Start ** 2);
               if Ok then
                  Refined := Refined + 1;
               end if;
            end;
         end;
      end loop;
      Place_Eyes (Depths, Body_Too => False);
      R.Base.Append (Base_Shift'(Known => True, others => <>));   --  the reference: no motion
      for F in 2 .. R.Aligned loop
         declare
            Shift : Base_Shift;
         begin
            if Arm_Ok (F) and then Body_Ok (F) and then Body_F_Ok (F) then
               Seen_Count := Seen_Count + 1;
               Shift.Known := True;
               --  The body eye's motion since the reference, in its own sigmas: where no gate tells it from none, the
               --  base held still, and no sighting is moved (RB-Y1, 10-27: its eye 0.009 degrees off, and a walk that
               --  told nothing had put the arm's eye a hundred thousand times the scene's depth from it: every
               --  sighting moved a median 98 px). A place its points leave no residual of is known exactly: any
               --  motion of it is one.
               declare
                  Length : constant Real := Whitened (Terms_Of (Body_Poses (F)), Body_Cov (F));
               begin
                  Shift.Told := (if Length < Real'Last then Length / Driver.Uncertain.Threshold (Gate) else Real'Last);
                  Shift.Moved := (if Length < Real'Last then Shift.Told > 1.0
                                  else (for some V of Terms_Of (Body_Poses (F)) => V /= 0.0));
               end;
               if not Shift.Moved then
                  for I in 0 .. Arm_N - 1 loop
                     Shift.Shift_U.Append (0.0);
                     Shift.Shift_V.Append (0.0);
                     Shift.Variance.Append (0.0);
                  end loop;
               else
                  Moved_Count := Moved_Count + 1;
                  declare
                     X     : constant Driver.Numerics.Rigid := R.Rig_X;
                     X_T   : constant Vec3 := Driver.Numerics.Log (X.Rotation);
                     Mb    : constant Driver.Numerics.Rigid := Body_Poses (F);
                     E     : constant Driver.Numerics.Rigid := Arm_Poses (F);
                     L     : constant Driver.Rig.Lens := R.Rig_Arm_Lens;
                     --  The base's motion carried onto the arm's eye's frame by a frame of the body eye on it.
                     function Base_Of (Xf, Motion : Driver.Numerics.Rigid) return Driver.Numerics.Rigid is
                       (Xf * Motion * Inverse (Xf));
                     Base  : constant Driver.Numerics.Rigid := Base_Of (X, Mb);
                     --  Each term a sigma either way, as poses of the base and of the arm's eye: the body eye's six
                     --  (its place's own covariance), the walk's frame's six, the body's focal length (as it moves the
                     --  body eye's place, Per_Body_F), the arm eye's six; then, point by point, the arm's focal length
                     --  and the point's inverse depth.
                     Pose_Terms  : constant := 6;   --  a pose's: its turn's three and its place's three
                     Body_First  : constant := 1;
                     Frame_First : constant := Body_First + Pose_Terms;
                     Body_Focal  : constant := Frame_First + Pose_Terms;
                     Arm_First   : constant := Body_Focal + 1;
                     Pose_Pairs  : constant := Arm_First + Pose_Terms - 1;   --  the terms carried by poses
                     Arm_Focal   : constant := Pose_Pairs + 1;
                     Depth_Term  : constant := Arm_Focal + 1;
                     --  Term T a sigma up at 2 T - 1 of these, down at 2 T.
                     Bases, Eyes : array (1 .. 2 * Pose_Pairs) of Driver.Numerics.Rigid := [others => Identity];
                     Steps_Of : array (1 .. Pose_Pairs) of Real := [others => 0.0];
                     function Up (T : Positive) return Positive is (2 * T - 1);
                     function Down (T : Positive) return Positive is (2 * T);
                  begin
                     Shift.Turn := Angle (Base.Rotation);
                     Shift.Step := Ada.Numerics.Long_Elementary_Functions.Sqrt
                       (Dot (Base.Translation, Base.Translation));
                     Eyes := [others => E];
                     for T in 1 .. Pose_Terms loop
                        declare
                           Body_T  : constant Positive := Body_First + T - 1;
                           Frame_T : constant Positive := Frame_First + T - 1;
                           Arm_T   : constant Positive := Arm_First + T - 1;
                           D       : constant Real_Array (1 .. Pose_Terms) := Along (T, Rig_Sigma (T));
                           Xu : constant Driver.Numerics.Rigid :=
                             (Rotation    => Exp (Plus (X_T, [D (1), D (2), D (3)])),
                              Translation => Plus (X.Translation, [D (4), D (5), D (6)]));
                           Xd : constant Driver.Numerics.Rigid :=
                             (Rotation    => Exp (Minus (X_T, [D (1), D (2), D (3)])),
                              Translation => Minus (X.Translation, [D (4), D (5), D (6)]));
                        begin
                           Steps_Of (Body_T) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                             (Real'Max (0.0, Body_Cov (F) (T, T)));
                           Bases (Up (Body_T)) := Base_Of (X, Moved_By (Mb, Along (T, Steps_Of (Body_T))));
                           Bases (Down (Body_T)) := Base_Of (X, Moved_By (Mb, Along (T, -Steps_Of (Body_T))));
                           Steps_Of (Frame_T) := Rig_Sigma (T);
                           Bases (Up (Frame_T)) := Base_Of (Xu, Mb);
                           Bases (Down (Frame_T)) := Base_Of (Xd, Mb);
                           Steps_Of (Arm_T) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                             (Real'Max (0.0, Arm_Cov (F) (T, T)));
                           Bases (Up (Arm_T)) := Base;
                           Bases (Down (Arm_T)) := Base;
                           Eyes (Up (Arm_T)) := Moved_By (E, Along (T, Steps_Of (Arm_T)));
                           Eyes (Down (Arm_T)) := Moved_By (E, Along (T, -Steps_Of (Arm_T)));
                        end;
                     end loop;
                     Steps_Of (Body_Focal) := Rig_Sigma (Body_Focal_Term);
                     Bases (Up (Body_Focal)) := Base_Of (X, Moved_By (Mb, Per_Body_F (F)));
                     Bases (Down (Body_Focal)) :=
                       Base_Of (X, Moved_By (Mb, [for T in 1 .. Pose_Terms => -Per_Body_F (F) (T)]));
                     for I in 0 .. Arm_N - 1 loop
                        declare
                           Ray    : constant Vec3 := Driver.Rig.Ray (L, (R.Query_U (I), R.Query_V (I)));
                           Rho    : constant Real := Depths (I);
                           Du, Dv : Real;
                           Ok     : Boolean;
                           --  Per term, the shift's change over two sigmas of it: the Jacobian's columns times
                           --  the sigmas, so that a block's covariance is its correlations.
                           Cu, Cv : Real_Array (1 .. Depth_Term) := [others => 0.0];
                           All_Ok : Boolean := True;
                           Var_U, Var_V, Cov_UV : Real := 0.0;
                        begin
                           Shift_Of (L, Base, E, Ray, Rho, Du, Dv, Ok);
                           if Ok then
                              for P in 1 .. Pose_Pairs loop
                                 declare
                                    Uu, Vu, Ud, Vd : Real;
                                    O1, O2 : Boolean;
                                 begin
                                    Shift_Of (L, Bases (Up (P)), Eyes (Up (P)), Ray, Rho, Uu, Vu, O1);
                                    Shift_Of (L, Bases (Down (P)), Eyes (Down (P)), Ray, Rho, Ud, Vd, O2);
                                    All_Ok := All_Ok and then O1 and then O2;
                                    Cu (P) := (Uu - Ud) / 2.0;
                                    Cv (P) := (Vu - Vd) / 2.0;
                                 end;
                              end loop;
                              --  The arm's focal length a sigma either way (its rays and its pictures), and the
                              --  point's inverse depth.
                              declare
                                 Lu, Ld : Driver.Rig.Lens := L;
                                 Uu, Vu, Ud, Vd : Real;
                                 O1, O2, O3, O4 : Boolean;
                                 Sr : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                   (Real'Max (0.0, Depth_Variances (I)));
                              begin
                                 Lu.Focal := L.Focal * Ada.Numerics.Long_Elementary_Functions.Exp
                                   (Rig_Sigma (Arm_Focal_Term));
                                 Ld.Focal := L.Focal * Ada.Numerics.Long_Elementary_Functions.Exp
                                   (-Rig_Sigma (Arm_Focal_Term));
                                 Shift_Of (Lu, Base, E, Driver.Rig.Ray (Lu, (R.Query_U (I), R.Query_V (I))), Rho,
                                           Uu, Vu, O1);
                                 Shift_Of (Ld, Base, E, Driver.Rig.Ray (Ld, (R.Query_U (I), R.Query_V (I))), Rho,
                                           Ud, Vd, O2);
                                 Cu (Arm_Focal) := (Uu - Ud) / 2.0;
                                 Cv (Arm_Focal) := (Vu - Vd) / 2.0;
                                 Shift_Of (L, Base, E, Ray, Rho + Sr, Uu, Vu, O3);
                                 Shift_Of (L, Base, E, Ray, Real'Max (0.0, Rho - Sr), Ud, Vd, O4);
                                 Cu (Depth_Term) := (Uu - Ud) / 2.0;
                                 Cv (Depth_Term) := (Vu - Vd) / 2.0;
                                 All_Ok := All_Ok and then O1 and then O2 and then O3 and then O4;
                              end;
                           end if;
                           if Ok and then All_Ok then
                              --  The body eye's place (its own six), the arm eye's (its six) and the inverse depth
                              --  each alone; the walk's frame and both focal lengths together, by their correlations.
                              for P in 1 .. Pose_Terms loop
                                 for Q in 1 .. Pose_Terms loop
                                    declare
                                       Bp : constant Positive := Body_First + P - 1;
                                       Bq : constant Positive := Body_First + Q - 1;
                                       Ap : constant Positive := Arm_First + P - 1;
                                       Aq : constant Positive := Arm_First + Q - 1;
                                       Cb : constant Real :=
                                         (if Steps_Of (Bp) > 0.0 and then Steps_Of (Bq) > 0.0
                                          then Body_Cov (F) (P, Q) / (Steps_Of (Bp) * Steps_Of (Bq)) else 0.0);
                                       Ce : constant Real :=
                                         (if Steps_Of (Ap) > 0.0 and then Steps_Of (Aq) > 0.0
                                          then Arm_Cov (F) (P, Q) / (Steps_Of (Ap) * Steps_Of (Aq)) else 0.0);
                                    begin
                                       Var_U := Var_U + Cu (Bp) * Cb * Cu (Bq) + Cu (Ap) * Ce * Cu (Aq);
                                       Var_V := Var_V + Cv (Bp) * Cb * Cv (Bq) + Cv (Ap) * Ce * Cv (Aq);
                                       Cov_UV := Cov_UV + Cu (Bp) * Cb * Cv (Bq) + Cu (Ap) * Ce * Cv (Aq);
                                    end;
                                 end loop;
                              end loop;
                              declare
                                 --  The walk's terms in Rig_Cov's order: the frame's six, the arm's focal length, the
                                 --  body's; their columns in Cu, Cv.
                                 function Column (T : Positive) return Positive is
                                   (if T <= Pose_Terms then Frame_First + T - 1
                                    elsif T = Arm_Focal_Term then Arm_Focal else Body_Focal);
                              begin
                                 for P in Rig_Sigma'Range loop
                                    for Q in Rig_Sigma'Range loop
                                       declare
                                          Cr : constant Real :=
                                            (if Rig_Sigma (P) > 0.0 and then Rig_Sigma (Q) > 0.0
                                             then R.Rig_Cov (P, Q) / (Rig_Sigma (P) * Rig_Sigma (Q)) else 0.0);
                                       begin
                                          Var_U := Var_U + Cu (Column (P)) * Cr * Cu (Column (Q));
                                          Var_V := Var_V + Cv (Column (P)) * Cr * Cv (Column (Q));
                                          Cov_UV := Cov_UV
                                            + Cu (Column (P)) * Cr * Cv (Column (Q));
                                       end;
                                    end loop;
                                 end loop;
                              end;
                              Var_U := Var_U + Cu (Depth_Term) ** 2;
                              Var_V := Var_V + Cv (Depth_Term) ** 2;
                              Cov_UV := Cov_UV + Cu (Depth_Term) * Cv (Depth_Term);
                              Shift.Shift_U.Append (Du);
                              Shift.Shift_V.Append (Dv);
                              --  Per axis, the larger of the two: the shift's covariance's largest eigenvalue.
                              Shift.Variance.Append
                                ((Var_U + Var_V) / 2.0
                                 + Ada.Numerics.Long_Elementary_Functions.Sqrt
                                     (((Var_U - Var_V) / 2.0) ** 2 + Cov_UV ** 2));
                              if I < Natural (R.Matches (F - 1).Found.Length) and then R.Matches (F - 1).Found (I) then
                                 Moved_Pixels := Moved_Pixels + 1;
                                 Pixels (Moved_Pixels) :=
                                   Ada.Numerics.Long_Elementary_Functions.Sqrt (Du ** 2 + Dv ** 2);
                                 Sigmas_Px (Moved_Pixels) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                   (Shift.Variance.Last_Element);
                              end if;
                           else
                              --  Where some pose a sigma off sees the point behind the eye, the shift is not known:
                              --  its sighting is left to no fit (Shift_Variance Real'Last).
                              Shift.Shift_U.Append (0.0);
                              Shift.Shift_V.Append (0.0);
                              Shift.Variance.Append (Real'Last);
                           end if;
                        end;
                     end loop;
                     Counted := Counted + 1;
                     Turns (Counted) := Shift.Turn;
                     Steps (Counted) := Shift.Step;
                  end;
               end if;
            end if;
            R.Base.Append (Shift);
         end;
      end loop;
      Driver.Log.Line
        (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s base under its" & R.Aligned'Image
         & " keyframes, seen at" & Seen_Count'Image & ", moved beyond its eye's noise at" & Moved_Count'Image
         & ": turned a median "
         & (if Counted > 0 then Degrees (Driver.Stats.Median (Turns (1 .. Counted)), 3) else "?")
         & " deg, stepped "
         & (if Counted > 0 then Driver.Log.Image (Driver.Stats.Median (Steps (1 .. Counted)), 4) else "?")
         & " of the walk's scale; its sightings shifted a median "
         & (if Moved_Pixels > 0 then Driver.Log.Image (Driver.Stats.Median (Pixels (1 .. Moved_Pixels)), 3) else "?")
         & " px, known to a median "
         & (if Moved_Pixels > 0 then Driver.Log.Image (Driver.Stats.Median (Sigmas_Px (1 .. Moved_Pixels)), 3)
            else "?")
         & " px;" & Refined'Image & " of" & Arm_N'Image & " points placed again from the keyframes");
      Release;
   end Shift_Sightings;

   function Shift_U (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real is
     (if Frame <= Natural (R.Base.Length) and then R.Base (Frame).Known
        and then Point < Natural (R.Base (Frame).Shift_U.Length)
      then R.Base (Frame).Shift_U (Point) else 0.0);

   function Shift_V (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real is
     (if Frame <= Natural (R.Base.Length) and then R.Base (Frame).Known
        and then Point < Natural (R.Base (Frame).Shift_V.Length)
      then R.Base (Frame).Shift_V (Point) else 0.0);

   function Shift_Variance (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real is
     (if Frame <= Natural (R.Base.Length) and then R.Base (Frame).Known
        and then Point < Natural (R.Base (Frame).Variance.Length)
      then R.Base (Frame).Variance (Point) else 0.0);

end Driver.Robot.Kinematics.Base;
