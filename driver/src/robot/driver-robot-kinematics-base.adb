with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Alignment.Keyframes;
with Driver.Bytes;
with Driver.Images;
with Driver.Log;
with Driver.Numerics;
with Driver.Recording;
with Driver.Rig;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics.Fit;
with Driver.Stats;

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
            if Response (M, Carrier, Eye_Id (E)) = Whole
              and then (Free or else (Mount_Of.Kind = Arm_Carried and then Mount_Of.Arm /= A))
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
   --  keeps what was found (Set_Of).
   procedure Align_Pictures
     (Who      : String;
      C        : in out Keyframes.Chain;
      Pictures : Driver.Images.Image;
      Index, Count : Positive;
      U, V     : Real_Vectors.Vector;
      Still    : Flag_Vectors.Vector;
      Riding   : Keyframes.Flag_Array;
      Set      : out Match_Set)
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

   --  The stills of the walk aligned to both eyes' reference pictures (the reference keyframe's), and adjusted
   --  together (Driver.Rig.Calibrate): the first lens the arm's fit's where it has one, else none to start from.
   procedure Calibrate (M : Model; R : in out Arm_Evidence) is
      Count  : constant Natural := Natural (R.Stills.Length);
      Arm_N  : constant Natural := Natural (R.Query_U.Length);
      Body_N : constant Natural := Natural (R.Body_Query_U.Length);
      Sightings : Natural := 0;
   begin
      for S of R.Still_Matches loop
         Sightings := Sightings + Found_Count (S);
      end loop;
      for S of R.Body_Still_Matches loop
         Sightings := Sightings + Found_Count (S);
      end loop;
      declare
         List : Driver.Rig.Sighting_Array (1 .. Natural'Max (1, Sightings));
         K    : Natural := 0;
         Arm_Points  : Driver.Rig.Place_Array (1 .. Arm_N);
         Body_Points : Driver.Rig.Place_Array (1 .. Body_N);
         C    : Driver.Rig.Calibration (Count, Arm_N, Body_N);
         W    : constant Natural := Driver.Images.Width (R.Frames (1).Image);
         H    : constant Natural := Driver.Images.Height (R.Frames (1).Image);
         Wb   : constant Natural := M.Eyes (R.Body_Eye).Grid.Width;
         Hb   : constant Natural := M.Eyes (R.Body_Eye).Grid.Height;
         Fitted : constant Boolean := R.Result.Fitted and then R.Result.Lens.Fx > 0.0 and then R.Result.Lens.Fy > 0.0;
         Start  : constant Driver.Rig.Lens :=
           (if Fitted then (Focal => Ada.Numerics.Long_Elementary_Functions.Sqrt (R.Result.Lens.Fx * R.Result.Lens.Fy),
                            Cx => R.Result.Lens.Cx, Cy => R.Result.Lens.Cy)
            else (Focal => 0.0, Cx => Real (W) / 2.0, Cy => Real (H) / 2.0));
      begin
         for I in 1 .. Arm_N loop
            Arm_Points (I) := (R.Query_U (I - 1), R.Query_V (I - 1));
         end loop;
         for J in 1 .. Body_N loop
            Body_Points (J) := (R.Body_Query_U (J - 1), R.Body_Query_V (J - 1));
         end loop;
         for Still in 1 .. Count loop
            declare
               S  : Match_Set renames R.Still_Matches (Still);
               Sb : Match_Set renames R.Body_Still_Matches (Still);
            begin
               for I in 0 .. Natural (S.Found.Length) - 1 loop
                  if S.Found (I) then
                     K := K + 1;
                     List (K) := (Eye => 1, Point => I + 1, Still => Still, U => S.To_U (I), V => S.To_V (I),
                                  Sigma => S.Sigma (I));
                  end if;
               end loop;
               for J in 0 .. Natural (Sb.Found.Length) - 1 loop
                  if Sb.Found (J) then
                     K := K + 1;
                     List (K) := (Eye => 2, Point => J + 1, Still => Still, U => Sb.To_U (J), V => Sb.To_V (J),
                                  Sigma => Sb.Sigma (J));
                  end if;
               end loop;
            end;
         end loop;
         R.Calibrated := True;
         R.Rig_With_Fit := Fitted;
         R.Rig_Ok := False;
         R.Rig_Stills := Count;
         if K = 0 then
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s walk: no still found a point");
            return;
         end if;
         Driver.Rig.Calibrate (Arm_Points, Body_Points, List (1 .. K), Count, Start, (Real (Wb) / 2.0, Real (Hb) / 2.0),
                               C);
         R.Rig_Ok := C.Ok;
         R.Rig_X := C.Second_On_First;
         R.Rig_Arm_Lens := C.First_Lens;
         R.Rig_Body_Lens := C.Second_Lens;
         R.Rig_Turn_Sigma := C.Turn_Sigma;
         R.Rig_Misfit := C.Misfit;
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
            Count  : constant Positive := Positive (R.Stills.Length);
            Arm_N  : constant Natural := Natural (R.Query_U.Length);
            Body_N : constant Natural := Natural (R.Body_Query_U.Length);
            Arm_Points  : Keyframes.Point_Array (1 .. Arm_N);
            Body_Points : Keyframes.Point_Array (1 .. Body_N);
            Riding : Keyframes.Flag_Array (1 .. Arm_N);
            None   : constant Keyframes.Flag_Array (1 .. Body_N) := [others => False];
            No_Still : Flag_Vectors.Vector;
         begin
            for I in 1 .. Arm_N loop
               Arm_Points (I) := (U => R.Query_U (I - 1), V => R.Query_V (I - 1));
               --  The points that ride with the arm's eye (its hand) tell nothing of the walk.
               Riding (I) := not Moving_Point (R, I - 1);
            end loop;
            for J in 1 .. Body_N loop
               Body_Points (J) := (U => R.Body_Query_U (J - 1), V => R.Body_Query_V (J - 1));
            end loop;
            Keyframes.Begin_Reference (R.Walk_Chain, R.Frames (1).Image, Arm_Points);
            Keyframes.Begin_Reference (R.Body_Walk_Chain, R.Frames (1).Body_Image, Body_Points);
            R.Still_Matches.Clear;
            R.Body_Still_Matches.Clear;
            for K in 1 .. Count loop
               declare
                  Set, Body_Set : Match_Set;
               begin
                  Align_Pictures ("arm" & R.Arm'Image & "'s walk", R.Walk_Chain, R.Stills (K).Image, K, Count,
                                  R.Query_U, R.Query_V, Unmoving (R), Riding, Set);
                  Align_Pictures ("arm" & R.Arm'Image & "'s walk, the body's eye", R.Body_Walk_Chain,
                                  R.Stills (K).Body_Image, K, Count, R.Body_Query_U, R.Body_Query_V, No_Still, None,
                                  Body_Set);
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s walk, still" & K'Image
                                   & " (beat" & R.Stills (K).Beat'Image & "):" & Found_Count (Set)'Image & " of"
                                   & Arm_N'Image & " of the arm's eye's points found," & Found_Count (Body_Set)'Image
                                   & " of" & Body_N'Image & " of the body's eye's");
                  R.Still_Matches.Append (Set);
                  R.Body_Still_Matches.Append (Body_Set);
               end;
               R.Stills (K).Image := Driver.Images.No_Image;
               R.Stills (K).Body_Image := Driver.Images.No_Image;
            end loop;
            Keyframes.Release (R.Walk_Chain);
            Keyframes.Release (R.Body_Walk_Chain);
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

   procedure Shift_Sightings (M : Model; R : in out Arm_Evidence) is
      pragma Unreferenced (M);
      Arm_N  : constant Natural := Natural (R.Query_U.Length);
      Frames : constant Natural := Natural'Max (1, R.Aligned);
      type Pose_List is array (1 .. Frames) of Driver.Numerics.Rigid;
      type Flag_List is array (1 .. Frames) of Boolean;
      Arm_Poses, Body_Poses : Pose_List := [others => Identity];
      Arm_Ok, Body_Ok       : Flag_List := [others => False];
      Middle   : Real := 0.0;   --  the median inverse depth of the arm's points the walk placed, for those it did not
      Depths   : Real_Vectors.Vector;   --  per arm point, the inverse depth the shifts take
      Turns, Steps : Real_Access := new Real_Array (1 .. Frames);
      Counted  : Natural := 0;
      Pixels   : Real_Access := new Real_Array (1 .. Natural'Max (1, R.Aligned * Natural'Max (1, Arm_N)));
      Moved_Pixels : Natural := 0;
      Refined  : Natural := 0;

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
         end if;
         for F in 2 .. R.Aligned loop
            if F - 1 <= Natural (R.Body_Matches.Length) then
               if Body_Too then
                  declare
                     Body_Seen : Driver.Rig.Locate_Result;
                  begin
                     Locate_Eye (R.Rig_Body_Lens, R.Body_Query_U, R.Body_Query_V, R.Rig_Body_Inverse,
                                 R.Body_Matches (F - 1), Identity, Body_Seen);
                     Body_Ok (F) := Body_Seen.Ok;
                     Body_Poses (F) := Body_Seen.Pose;
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
               end;
            end if;
         end loop;
      end Place_Eyes;
   begin
      R.Base.Clear;
      if not R.Rig_Ok or else R.Aligned < 2 or else Natural (R.Rig_Arm_Inverse.Length) /= Arm_N then
         Free (Turns);
         Free (Steps);
         Free (Pixels);
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
                     Sigmas, Start, Inverse, Ok);
               end if;
               Depths.Append (if Ok then Inverse else Start);
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
            if Arm_Ok (F) and then Body_Ok (F) then
               declare
                  --  The base's motion carried onto the arm's eye's frame, and where a still base would have put it.
                  Base  : constant Driver.Numerics.Rigid := R.Rig_X * Body_Poses (F) * Inverse (R.Rig_X);
                  Still : constant Driver.Numerics.Rigid := Inverse (Base) * Arm_Poses (F);
                  L     : constant Driver.Rig.Lens := R.Rig_Arm_Lens;
               begin
                  Shift.Known := True;
                  Shift.Turn := Angle (Base.Rotation);
                  Shift.Step := Ada.Numerics.Long_Elementary_Functions.Sqrt (Dot (Base.Translation, Base.Translation));
                  for I in 0 .. Arm_N - 1 loop
                     declare
                        Ray               : constant Vec3 := Driver.Rig.Ray (L, (R.Query_U (I), R.Query_V (I)));
                        As_Seen, As_Still : Driver.Rig.Place;
                        Ahead_1, Ahead_2  : Boolean;
                     begin
                        Driver.Rig.Project (L, Driver.Rig.Seen (Arm_Poses (F), Ray, Depths (I)), As_Seen, Ahead_1);
                        Driver.Rig.Project (L, Driver.Rig.Seen (Still, Ray, Depths (I)), As_Still, Ahead_2);
                        if Ahead_1 and then Ahead_2 then
                           Shift.Shift_U.Append (As_Still.U - As_Seen.U);
                           Shift.Shift_V.Append (As_Still.V - As_Seen.V);
                           if I < Natural (R.Matches (F - 1).Found.Length) and then R.Matches (F - 1).Found (I) then
                              Moved_Pixels := Moved_Pixels + 1;
                              Pixels (Moved_Pixels) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                ((As_Still.U - As_Seen.U) ** 2 + (As_Still.V - As_Seen.V) ** 2);
                           end if;
                        else
                           Shift.Shift_U.Append (0.0);
                           Shift.Shift_V.Append (0.0);
                        end if;
                     end;
                  end loop;
                  Counted := Counted + 1;
                  Turns (Counted) := Shift.Turn;
                  Steps (Counted) := Shift.Step;
               end;
            end if;
            R.Base.Append (Shift);
         end;
      end loop;
      Driver.Log.Line
        (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & "'s base under its" & R.Aligned'Image
         & " keyframes, seen at" & Counted'Image & ": turned a median "
         & (if Counted > 0 then Degrees (Driver.Stats.Median (Turns (1 .. Counted)), 3) else "?")
         & " deg, stepped "
         & (if Counted > 0 then Driver.Log.Image (Driver.Stats.Median (Steps (1 .. Counted)), 4) else "?")
         & " of the walk's scale; its sightings shifted a median "
         & (if Moved_Pixels > 0 then Driver.Log.Image (Driver.Stats.Median (Pixels (1 .. Moved_Pixels)), 3) else "?")
         & " px;" & Refined'Image & " of" & Arm_N'Image & " points placed again from the keyframes");
      Free (Turns);
      Free (Steps);
      Free (Pixels);
   end Shift_Sightings;

   function Shift_U (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real is
     (if Frame <= Natural (R.Base.Length) and then R.Base (Frame).Known
        and then Point < Natural (R.Base (Frame).Shift_U.Length)
      then R.Base (Frame).Shift_U (Point) else 0.0);

   function Shift_V (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real is
     (if Frame <= Natural (R.Base.Length) and then R.Base (Frame).Known
        and then Point < Natural (R.Base (Frame).Shift_V.Length)
      then R.Base (Frame).Shift_V (Point) else 0.0);

end Driver.Robot.Kinematics.Base;
