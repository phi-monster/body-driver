with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Stillness;
with Driver.Stats;
with Driver.Uncertain;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Lockin;

package body Driver.Robot.Kinematics is

   --  Everything sized by beats, keyframes, queries or matches lives on the
   --  heap: the estimates also run in the decider's task, whose stack is
   --  small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Answer_Access is access Driver.Instrument.Answer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Instrument.Answer_Array, Answer_Access);
   type Point_Access is access Driver.Instrument.Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Instrument.Point_Array, Point_Access);
   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);
   type Flag_Array is array (Natural range <>) of Boolean;
   type Flag_Access is access Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flag_Access);

   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   --  The arm's eye: the one the graph mounts on it; 0 when none.
   function Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base is
   begin
      for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
         if M.Graph.Mounts (E).Kind = Arm_Carried and then M.Graph.Mounts (E).Arm = A then
            return E;
         end if;
      end loop;
      return 0;
   end Eye_Of;

   --  The centres of the eye's cells that respond to the group's push: they
   --  show the world the arm moves its eye through. Every textured cell when
   --  the lock-in has not told yet.
   procedure Query_Points (M : Model; E : Eye_Id; G : Group_Id; U, V : out Real_Vectors.Vector) is
      S    : Eye_Stream renames M.Eyes (E);
      N    : constant Natural := Cells (S.Grid);
      Kept : constant Natural := Natural (S.Kept_Groups.Length);
   begin
      U.Clear;
      V.Clear;
      for C in 1 .. N loop
         declare
            Responds : Boolean := Kept = 0;
            X0, X1, Y0, Y1 : Natural;
         begin
            for K in 0 .. Kept - 1 loop
               if S.Kept_Groups (K) = Natural (G) and then S.Shifts ((C - 1) * Kept + K) > 0.0 then
                  Responds := True;
               end if;
            end loop;
            if Responds and then (Natural (S.Textured.Length) < N or else S.Textured (C - 1)) then
               Flow.Bounds (S.Grid, C, X0, X1, Y0, Y1);
               U.Append (Real (X0 + X1) / 2.0);
               V.Append (Real (Y0 + Y1) / 2.0);
            end if;
         end;
      end loop;
   end Query_Points;

   function Readings_Of (M : Model; G : Group_Id; Beat : Natural) return Real_Array is
      Result : Real_Array (1 .. M.Groups (G).Size);
   begin
      for C in Result'Range loop
         Result (C) := Channels.Reading (M, G, Beat, C);
      end loop;
      return Result;
   end Readings_Of;

   --  Reads the answers ready in Pending into Into.
   procedure Collect_Into
     (R       : in out Arm_Evidence;
      Pending : in out Pending_Vectors.Vector;
      Into    : in out Match_Set_Vectors.Vector)
   is
      K : Natural := Pending.First_Index;
   begin
      while K <= Pending.Last_Index loop
         if Driver.Services.Ready (Pending (K).Ticket) then
            declare
               P      : constant Pending_Match := Pending (K);
               Reply  : constant Driver.Services.Reply := Driver.Services.Collect (P.Ticket);
               Result : Answer_Access := new Driver.Instrument.Answer_Array (1 .. Natural (R.Query_U.Length));
               Ok     : Boolean;
               Why    : Ada.Strings.Unbounded.Unbounded_String;
               Set    : Match_Set;
            begin
               Driver.Instrument.Read_Match (Reply, True, Result.all, Ok, Why);
               if Ok then
                  Set.Frame := P.Frame;
                  for A of Result.all loop
                     Set.To_U.Append (A.To.U);
                     Set.To_V.Append (A.To.V);
                     Set.Back_U.Append (A.Back.U);
                     Set.Back_V.Append (A.Back.V);
                     Set.Found.Append (A.Found);
                  end loop;
                  Into.Append (Set);
               elsif Reply.Lasting then
                  if not R.Unanswerable then
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image
                                      & " gets no matches, and asks no more: " & Ada.Strings.Unbounded.To_String (Why));
                  end if;
                  R.Unanswerable := True;
               else
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe" & P.Frame'Image
                                   & " has no matches: " & Ada.Strings.Unbounded.To_String (Why));
               end if;
               Free (Result);
               Pending.Delete (K);
            end;
         else
            K := K + 1;
         end if;
      end loop;
   end Collect_Into;

   procedure Collect (R : in out Arm_Evidence) is
   begin
      Collect_Into (R, R.Pending, R.Matches);
      Collect_Into (R, R.World_Pending, R.World_Matches);
   end Collect;

   --  The evidence is an arm's now: the graph still lists its group as an
   --  arm, under that number, carrying that eye. Evidence is kept by group,
   --  so an arm renumbered by a new one keeps it; an arm that stopped being
   --  one is not fitted.
   function Current (M : Model; R : Arm_Evidence) return Boolean is
     (R.Arm in 1 .. Arm_Id'Base (Arm_Count (M)) and then Arm_Group (M, R.Arm) = R.Group
      and then Eye_Of (M, R.Arm) = R.Eye);

   --  The evidence of arm A as the graph has it now; 0 when there is none.
   function Index_Of (M : Model; A : Arm_Id) return Natural is
   begin
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if M.Kinematics (K).Arm = A and then Current (M, M.Kinematics (K)) then
            return K;
         end if;
      end loop;
      return 0;
   end Index_Of;

   --  The noise of a round trip over the evidence's answers: the robust
   --  scale about zero of every answer's return to its query, both
   --  coordinates; 0 without answers.
   function Round_Trip_Sigma (R : Arm_Evidence) return Real is
      Queries : constant Natural := Natural (R.Query_U.Length);
      Count   : Natural := 0;
   begin
      for S of R.Matches loop
         for I in 0 .. Queries - 1 loop
            if S.Found (I) then
               Count := Count + 1;
            end if;
         end loop;
      end loop;
      if Count = 0 then
         return 0.0;
      end if;
      declare
         Trips : Real_Access := new Real_Array (1 .. 2 * Count);
         K     : Natural := 0;
      begin
         for S of R.Matches loop
            for I in 0 .. Queries - 1 loop
               if S.Found (I) then
                  Trips (K + 1) := abs (S.Back_U (I) - R.Query_U (I));
                  Trips (K + 2) := abs (S.Back_V (I) - R.Query_V (I));
                  K := K + 2;
               end if;
            end loop;
         end loop;
         return Sigma : constant Real :=
           Driver.Stats.Median (Trips.all) / Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5)
         do
            Free (Trips);
         end return;
      end;
   end Round_Trip_Sigma;

   function Held_Still (M : Model; A : Arm_Id; Beat : Natural) return Boolean is
      E : constant Eye_Id'Base := Eye_Of (M, A);
      G : constant Group_Id := Arm_Group (M, A);
   begin
      if E = 0 then
         return False;
      end if;
      --  The eye's picture by the one stop rule (Stillness.Eye_Settled), which
      --  holds for the latest beat.
      return Beat > 0 and then Stillness.Eye_Settled (M, E)
        and then Channels.Has_Reading (M, G, Beat) and then Channels.Has_Reading (M, G, Beat - 1)
        and then not Channels.Moving (M, G, Beat);
   end Held_Still;

   procedure Observe (M : in out Model; O : Observation) is
      Beat : constant Natural := M.Beats;
   begin
      --  Every arm's answers, whether or not the graph lists the arm now: a
      --  request is pending until its answer is read (Hold_While_Matching
      --  waits for all of them).
      for R of M.Kinematics loop
         Collect (R);
      end loop;
      for A in 1 .. Arm_Count (M) loop
         declare
            Arm : constant Arm_Id := Arm_Id (A);
            G   : constant Group_Id := Arm_Group (M, Arm);
            E   : constant Eye_Id'Base := Eye_Of (M, Arm);
            Index : Natural := 0;
         begin
            if E > 0 then
               for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                  if M.Kinematics (K).Group = G then
                     Index := K;
                  end if;
               end loop;
               --  A group met as an arm for the first time, or one whose eye the
               --  graph now tells differently, starts its evidence afresh; one
               --  renumbered by a new arm keeps it under its new number.
               if Index = 0 then
                  M.Kinematics.Append (Arm_Evidence'(Arm => Arm, Group => G, Eye => E, others => <>));
                  Index := M.Kinematics.Last_Index;
               elsif M.Kinematics (Index).Eye /= E then
                  M.Kinematics.Replace_Element (Index, (Arm => Arm, Group => G, Eye => E, others => <>));
               elsif M.Kinematics (Index).Arm /= Arm then
                  M.Kinematics (Index).Arm := Arm;
               end if;
               declare
                  R : Arm_Evidence renames M.Kinematics (Index);
               begin
                  if Held_Still (M, Arm, Beat)
                    and then E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E)
                  then
                     declare
                        Now   : constant Real_Array := Readings_Of (M, G, Beat);
                        Fresh : Boolean := True;
                     begin
                        --  A view worth matching shows something new: against every
                        --  keyframe, some channel moved by at least the step its eye
                        --  can see (Visible_Step). None is taken before those steps
                        --  are measured. The second keyframe is the reference's
                        --  still twin, at its pose: its matches are the matcher's
                        --  own error, known before the arm moves.
                        for F of R.Frames loop
                           declare
                              Apart : Boolean := False;
                           begin
                              for C in Now'Range loop
                                 declare
                                    V : constant Estimate := Visible_Step (M, G, C);
                                 begin
                                    if Known (V) and then abs (Now (C) - F.Readings (C - 1)) >= V.Value then
                                       Apart := True;
                                    end if;
                                 end;
                              end loop;
                              Fresh := Fresh and then Apart;
                           end;
                        end loop;
                        declare
                           Seeable : Boolean := False;
                        begin
                           for C in Now'Range loop
                              Seeable := Seeable or else Known (Visible_Step (M, G, C));
                           end loop;
                           Fresh := Seeable
                             and then (Fresh or else (Natural (R.Frames.Length) = 1 and then Beat > R.Frames (1).Beat));
                        end;
                        if Fresh and then not R.Unanswerable then
                           declare
                              K : Keyframe;
                           begin
                              K.Beat := Beat;
                              for X of Now loop
                                 K.Readings.Append (X);
                              end loop;
                              K.Image := O.Images (E);
                              R.Frames.Append (K);
                           end;
                           if Natural (R.Frames.Length) = 1 then
                              Query_Points (M, E, G, R.Query_U, R.Query_V);
                           elsif not R.Query_U.Is_Empty and then not R.Unanswerable then
                              declare
                                 Points : Point_Access := new Driver.Instrument.Point_Array (1 .. Natural (R.Query_U.Length));
                              begin
                                 for P in Points'Range loop
                                    Points (P) := (U => R.Query_U (P - 1), V => R.Query_V (P - 1));
                                 end loop;
                                 R.Pending.Append
                                   (Pending_Match'(Frame  => R.Frames.Last_Index,
                                     Ticket => Driver.Instrument.Submit_Match
                                       ((Stored => False, Image => R.Frames.First_Element.Image),
                                        (Stored => False, Image => O.Images (E)),
                                        Points.all, True, O.Beat)));
                                 Free (Points);
                              end;
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
      --  Every other arm's reference view, once taken, is matched against the
      --  first arm's: the first arm's query points, into it. Asked again when
      --  either reference is taken anew.
      declare
         W : constant Natural := Index_Of (M, 1);
      begin
         if W > 0 and then not M.Kinematics (W).Frames.Is_Empty and then not M.Kinematics (W).Query_U.Is_Empty
           and then not M.Kinematics (W).Unanswerable
         then
            for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
               declare
                  Asked_Of : constant Natural := M.Kinematics (W).Frames.First_Element.Beat;
               begin
                  if K /= W and then Current (M, M.Kinematics (K)) and then not M.Kinematics (K).Frames.Is_Empty
                    and then not M.Kinematics (K).Unanswerable
                    and then not (M.Kinematics (K).World_Asked
                                  and then M.Kinematics (K).World_Group = M.Kinematics (W).Group
                                  and then M.Kinematics (K).World_Reference = Asked_Of)
                  then
                     declare
                        Points : Point_Access := new Driver.Instrument.Point_Array (1 .. Natural (M.Kinematics (W).Query_U.Length));
                     begin
                        for P in Points'Range loop
                           Points (P) := (U => M.Kinematics (W).Query_U (P - 1), V => M.Kinematics (W).Query_V (P - 1));
                        end loop;
                        M.Kinematics (K).World_Matches.Clear;
                        M.Kinematics (K).World_Pending.Append
                          (Pending_Match'(Frame  => 1,
                                          Ticket => Driver.Instrument.Submit_Match
                                            ((Stored => False, Image => M.Kinematics (W).Frames.First_Element.Image),
                                             (Stored => False, Image => M.Kinematics (K).Frames.First_Element.Image),
                                             Points.all, True, O.Beat)));
                        M.Kinematics (K).World_Asked := True;
                        M.Kinematics (K).World_Group := M.Kinematics (W).Group;
                        M.Kinematics (K).World_Reference := Asked_Of;
                        Free (Points);
                     end;
                  end if;
               end;
            end loop;
         end if;
      end;
   end Observe;

   function Match_Noise (M : Model; A : Arm_Id) return Real is
     (if Index_Of (M, A) > 0 then Round_Trip_Sigma (M.Kinematics (Index_Of (M, A))) else 0.0);

   function Keyframe_Step (M : Model; A : Arm_Id; Channel : Positive) return Real is
      E : constant Eye_Id'Base := Eye_Of (M, A);
   begin
      if E = 0 then
         return 0.0;
      end if;
      declare
         Per_Unit : constant Real := Lockin.Shift (M, E, Arm_Group (M, A), Channel);
         --  The larger of the cells' displacement noise and the matcher's: a
         --  keyframe is judged by the matcher, its view by the cells.
         Noise    : constant Real := Real'Max (Lockin.Cell_Noise (M, E), Match_Noise (M, A));
      begin
         return (if Per_Unit > 0.0 and then Noise < Real'Last then Driver.Conventions.Z * Noise / Per_Unit else 0.0);
      end;
   end Keyframe_Step;

   function Twin_Answered (M : Model; A : Arm_Id) return Boolean is
   begin
      if Index_Of (M, A) = 0 then
         return False;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (Index_Of (M, A));
      begin
         return R.Unanswerable or else (Natural (R.Frames.Length) >= 2 and then R.Pending.Is_Empty);
      end;
   end Twin_Answered;

   function Matched (M : Model; A : Arm_Id) return Natural is
     (if Index_Of (M, A) > 0 then Natural (M.Kinematics (Index_Of (M, A)).Matches.Length) else 0);

   function Pending (M : Model) return Natural is
      K : Natural := 0;
   begin
      for R of M.Kinematics loop
         K := K + Natural (R.Pending.Length) + Natural (R.World_Pending.Length);
      end loop;
      return K;
   end Pending;

   --  Every arm but the first placed in the world, by its reference view of
   --  the first arm's tracked points (In_World).
   procedure Place (M : in out Model);

   type Track_Point_Access is access Fit.Track_Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Track_Point_Array, Track_Point_Access);

   procedure Refit (M : in out Model) is
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R : Arm_Evidence renames M.Kinematics (Index);
         begin
            if not Current (M, R) then
               R.Result := (others => <>);
            elsif not R.Matches.Is_Empty and then Natural (R.Matches.Length) /= R.Result.Matches
              and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index
            then
               declare
                  N       : constant Natural := M.Groups (R.Group).Size;
                  Frames  : constant Natural := Natural (R.Frames.Length);
                  Queries : constant Natural := Natural (R.Query_U.Length);
                  Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
                  Visible : Real_Array (1 .. N);
                  Sigma   : constant Real := Round_Trip_Sigma (R);
               begin
                  for F in 1 .. Frames loop
                     for C in 1 .. N loop
                        Changes (F, C) := R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1);
                     end loop;
                  end loop;
                  for C in 1 .. N loop
                     Visible (C) := Keyframe_Step (M, R.Arm, C);
                  end loop;
                  declare
                     function Round_Trip (S : Match_Set; I : Natural) return Boolean is
                       (S.Found (I)
                        and then (Sigma = 0.0
                                  or else (not Driver.Uncertain.Significant (S.Back_U (I) - R.Query_U (I), Sigma)
                                           and then not Driver.Uncertain.Significant (S.Back_V (I) - R.Query_V (I), Sigma))));

                     --  A keyframe tells the fit something only when its points
                     --  moved by more than the matcher errs: the median of their
                     --  displacements significant against a round trip's noise.
                     function Moved (S : Match_Set) return Boolean is
                        D : Real_Access := new Real_Array (1 .. Queries);
                        K : Natural := 0;
                     begin
                        for I in 0 .. Queries - 1 loop
                           if Round_Trip (S, I) then
                              K := K + 1;
                              D (K) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                ((S.To_U (I) - R.Query_U (I)) ** 2 + (S.To_V (I) - R.Query_V (I)) ** 2);
                           end if;
                        end loop;
                        return Result : constant Boolean :=
                          K > 0 and then Driver.Uncertain.Significant (Driver.Stats.Median (D (1 .. K)), Sigma)
                        do
                           Free (D);
                        end return;
                     end Moved;

                     Moving : Flag_Access := new Flag_Array (R.Matches.First_Index .. R.Matches.Last_Index);

                     function Returns (S : Match_Set; I : Natural) return Boolean is
                       (Round_Trip (S, I) and then (for some K in Moving'Range => Moving (K) and then R.Matches (K).Frame = S.Frame));
                     Kept : Natural := 0;
                  begin
                     for K in Moving'Range loop
                        Moving (K) := Moved (R.Matches (K));
                     end loop;
                     for S of R.Matches loop
                        for I in 0 .. Queries - 1 loop
                           if Returns (S, I) then
                              Kept := Kept + 1;
                           end if;
                        end loop;
                     end loop;
                     declare
                        type Sighting_Access is access Fit.Sighting_Array;
                        procedure Free is new Ada.Unchecked_Deallocation (Fit.Sighting_Array, Sighting_Access);
                        Seen   : Sighting_Access := new Fit.Sighting_Array (1 .. Kept);
                        K      : Natural := 0;
                        Joints : Fit.Joint_Array (1 .. N);
                        Lens   : Fit.Lens;
                        Report : Fit.Fit_Report;
                        Result : Arm_Fit;
                     begin
                        for S of R.Matches loop
                           for I in 0 .. Queries - 1 loop
                              if Returns (S, I) then
                                 K := K + 1;
                                 Seen (K) := (Frame => S.Frame, Track => I + 1,
                                              U0 => R.Query_U (I), V0 => R.Query_V (I),
                                              U => S.To_U (I), V => S.To_V (I));
                              end if;
                           end loop;
                        end loop;
                        Fit.Fit (Changes.all, Visible, Seen.all, M.Eyes (R.Eye).Grid.Width, M.Eyes (R.Eye).Grid.Height,
                                 Joints, Lens, Report);
                        --  The table its eye sees, away from it towards the eye,
                        --  and its tracks' points: Up is the first arm's (the world
                        --  is that eye's reference frame), and the points place the
                        --  other arms (Place).
                        if Report.Fitted then
                           declare
                              Normal : Vec3;
                              Offset, Offset_Sigma, Sigma : Real;
                              Found  : Boolean;
                              Points : Track_Point_Access := new Fit.Track_Point_Array (1 .. Queries);
                           begin
                              Fit.Table (Changes.all, Seen.all, Joints, Lens, Normal, Offset, Offset_Sigma, Sigma, Found);
                              Result.Table_Found := Found;
                              Result.Table_Normal := Normal;
                              Result.Table_Offset := Offset;
                              Result.Table_Offset_Sigma := Offset_Sigma;
                              Result.Table_Sigma := Sigma;
                              if Found and then R.Arm = 1 then
                                 M.Table_Up := (Unit_Vector => Normal, Sigma => Sigma);
                              end if;
                              --  The points where the fit put them: each track's refined
                              --  depth along its reference line of sight.
                              for I in 0 .. Queries - 1 loop
                                 declare
                                    D : constant Real :=
                                      (if I < Natural (Report.Depths.Length) then Report.Depths (Report.Depths.First_Index + I)
                                       else 0.0);
                                    H : constant Vec3 := Fit.Ray (Lens, R.Query_U (I), R.Query_V (I));
                                 begin
                                    Result.Track_Known.Append (D > 0.0);
                                    for X of H loop
                                       Result.Tracks.Append (D * X);
                                    end loop;
                                 end;
                              end loop;
                              Free (Points);
                           end;
                        end if;
                        Free (Seen);
                        Result.Fitted := Report.Fitted;
                        Result.Matches := Natural (R.Matches.Length);
                        Result.Used := Report.Used;
                        Result.Median_Px := Report.Median_Px;
                        Result.Sigma_Px := Report.Sigma_Px;
                        Result.Why := Report.Why;
                        Result.Reference := R.Frames (1).Readings;
                        for J of Joints loop
                           Result.Joints.Append (Joint_Fit'(W => J.W, P => J.P, C => J.C, Slide => J.Slide));
                        end loop;
                        Result.Lens := (Fx => Lens.Fx, Fy => Lens.Fy, Cx => Lens.Cx, Cy => Lens.Cy, K1 => Lens.K1, K2 => Lens.K2);
                        for X of Report.Covariance loop
                           Result.Covariance.Append (X);
                        end loop;
                        --  A fit that failed keeps the last one that held.
                        if Report.Fitted or else not R.Result.Fitted then
                           R.Result := Result;
                        else
                           R.Result.Matches := Result.Matches;
                           R.Result.Why := Result.Why;
                        end if;
                        Driver.Log.Line
                          (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " "
                           & (if Report.Fitted
                              then "fitted from" & Kept'Image & " sightings of" & Frames'Image & " keyframes,"
                                   & Report.Used'Image & " fit, median " & Driver.Log.Image (Report.Median_Px, 3)
                                   & " px, noise " & Driver.Log.Image (Report.Sigma_Px, 3) & " px; focal "
                                   & Driver.Log.Image (Lens.Fx, 2) & " x " & Driver.Log.Image (Lens.Fy, 2) & " px"
                              else "not fitted (stage" & Report.Stage'Image & "): "
                                   & Ada.Strings.Unbounded.To_String (Report.Why)));
                     end;
                     Free (Moving);
                  end;
                  Free (Changes);
               end;
            end if;
         end;
      end loop;
      Place (M);
   end Refit;

   procedure Place (M : in out Model) is
      W : constant Natural := Index_Of (M, 1);

      --  How well the arm's fit unit is known against its depths (Fit.Unit_Sigma).
      function Unit_Sigma_Of (R : Arm_Evidence) return Real is
         N      : constant Natural := Natural (R.Result.Joints.Length);
         Frames : constant Natural := Natural (R.Frames.Length);
      begin
         if N = 0 or else Frames = 0 or else Natural (R.Result.Reference.Length) /= N then
            return Real'Last;
         end if;
         declare
            Joints  : Fit.Joint_Array (1 .. N);
            Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
            Cov     : Fit.Real_Lists.Vector;
         begin
            for J in 1 .. N loop
               declare
                  F : constant Joint_Fit := R.Result.Joints (J);
               begin
                  Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
               end;
            end loop;
            for F in 1 .. Frames loop
               for C in 1 .. N loop
                  Changes (F, C) := R.Frames (F).Readings (C - 1) - R.Result.Reference (C - 1);
               end loop;
            end loop;
            for X of R.Result.Covariance loop
               Cov.Append (X);
            end loop;
            return Result : constant Real := Fit.Unit_Sigma (Joints, Changes.all, Cov) do
               Free (Changes);
            end return;
         end;
      end Unit_Sigma_Of;

      function Track (R : Arm_Fit; I : Natural) return Vec3 is
        ([R.Tracks (3 * I), R.Tracks (3 * I + 1), R.Tracks (3 * I + 2)]);
      function Known_Track (R : Arm_Fit; I : Natural) return Boolean is
        (I < Natural (R.Track_Known.Length) and then R.Track_Known (I));
   begin
      --  The first arm is the world as it is.
      if W > 0 then
         declare
            R : Arm_Fit renames M.Kinematics (W).Result;
         begin
            R.Placed := R.Fitted;
            R.Placement := Identity;
            R.Scale := 1.0;
            R.Scale_Sigma := 0.0;
            R.Placement_Covariance.Clear;
            R.Placement_Covariance.Append (0.0, 36);
         end;
      end if;
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if K /= W then
            declare
               R2 : Arm_Evidence renames M.Kinematics (K);
               Why : Ada.Strings.Unbounded.Unbounded_String;
            begin
               R2.Result.Placed := False;
               if W = 0 or else not M.Kinematics (W).Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("the first arm is not fitted");
               elsif not Current (M, R2) or else not R2.Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("it is not fitted");
               elsif R2.World_Matches.Is_Empty or else R2.World_Group /= M.Kinematics (W).Group
                 or else M.Kinematics (W).Frames.Is_Empty
                 or else R2.World_Reference /= M.Kinematics (W).Frames.First_Element.Beat
               then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("its view of the first arm's points is not answered yet");
               else
                  declare
                     R1   : Arm_Evidence renames M.Kinematics (W);
                     S    : constant Match_Set := R2.World_Matches.First_Element;
                     Q1   : constant Natural := Natural (R1.Query_U.Length);
                     Q2   : constant Natural := Natural (R2.Query_U.Length);
                     L2   : constant Fit.Lens := (Fx => R2.Result.Lens.Fx, Fy => R2.Result.Lens.Fy, Cx => R2.Result.Lens.Cx,
                                                  Cy => R2.Result.Lens.Cy, K1 => R2.Result.Lens.K1, K2 => R2.Result.Lens.K2);
                     Grid : constant Cell_Grid := M.Eyes (R2.Eye).Grid;
                     --  Half a cell of the arm's eye: how near one of its own query
                     --  points must lie to lend its depth.
                     Half_U : constant Real := Real (Grid.Width) / Real (Natural'Max (1, Grid.Columns)) / 2.0;
                     Half_V : constant Real := Real (Grid.Height) / Real (Natural'Max (1, Grid.Rows)) / 2.0;
                     Pairs  : Fit.Correspondence_Array (1 .. Q1);
                     Of_Query : array (1 .. Q1) of Natural := [others => 0];   --  each pair's query of the first arm
                     Both   : Fit.Point_Pair_Array (1 .. Q1);
                     Np, Nb : Natural := 0;
                     --  This match's own round-trip noise: across two arms' views the
                     --  matcher errs otherwise than along one arm's sweep.
                     Trip : Real := 0.0;
                  begin
                     declare
                        Trips : Real_Access := new Real_Array (1 .. 2 * Q1);
                        T     : Natural := 0;
                     begin
                        for I in 0 .. Q1 - 1 loop
                           if S.Found (I) then
                              Trips (T + 1) := abs (S.Back_U (I) - R1.Query_U (I));
                              Trips (T + 2) := abs (S.Back_V (I) - R1.Query_V (I));
                              T := T + 2;
                           end if;
                        end loop;
                        if T > 0 then
                           Trip := Driver.Stats.Median (Trips (1 .. T)) / Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5);
                        end if;
                        Free (Trips);
                     end;
                     for I in 0 .. Q1 - 1 loop
                        if S.Found (I) and then Known_Track (R1.Result, I)
                          and then (Trip = 0.0
                                    or else (not Driver.Uncertain.Significant (S.Back_U (I) - R1.Query_U (I), Trip)
                                             and then not Driver.Uncertain.Significant (S.Back_V (I) - R1.Query_V (I), Trip)))
                        then
                           Np := Np + 1;
                           Pairs (Np) := (X => Track (R1.Result, I), U => S.To_U (I), V => S.To_V (I));
                           Of_Query (Np) := I;
                           --  The same point in the arm's own frame: on the line of
                           --  sight through where it was found, at the depth the arm's
                           --  own tracks around there give it: inverse depth linear in
                           --  the pixel over those within a cell each way (exact on a
                           --  plane, which a view of one surface is between its tracks).
                           declare
                              A    : Mat3 := [others => [others => 0.0]];
                              B    : Vec3 := [0.0, 0.0, 0.0];
                              Near : Natural := 0;
                           begin
                              for J in 0 .. Q2 - 1 loop
                                 if Known_Track (R2.Result, J)
                                   and then abs (R2.Query_U (J) - S.To_U (I)) <= 2.0 * Half_U
                                   and then abs (R2.Query_V (J) - S.To_V (I)) <= 2.0 * Half_V
                                   and then Track (R2.Result, J) (3) > 0.0
                                 then
                                    declare
                                       Row : constant Vec3 := [1.0, (R2.Query_U (J) - S.To_U (I)) / Half_U,
                                                               (R2.Query_V (J) - S.To_V (I)) / Half_V];
                                    begin
                                       A := Driver.Numerics.Arrays."+" (A, Driver.Numerics.Outer (Row, Row));
                                       B := Driver.Numerics.Arrays."+" (B, Driver.Numerics.Arrays."*" (1.0 / Track (R2.Result, J) (3), Row));
                                       Near := Near + 1;
                                    end;
                                 end if;
                              end loop;
                              if Near >= 3 and then abs Driver.Numerics.Arrays.Determinant (A) > 0.0 then
                                 declare
                                    Inverse_Depth : constant Real := Driver.Numerics.Arrays."*" (Driver.Numerics.Arrays.Inverse (A), B) (1);
                                    H : constant Vec3 := Fit.Ray (L2, S.To_U (I), S.To_V (I));
                                 begin
                                    if Inverse_Depth > 0.0 then
                                       Nb := Nb + 1;
                                       Both (Nb) := (From => Driver.Numerics.Arrays."*" (1.0 / (Inverse_Depth * H (3)), H),
                                                     To   => Track (R1.Result, I));
                                    end if;
                                 end;
                              end if;
                           end;
                        end if;
                     end loop;
                     declare
                        Rotation    : Mat3;
                        Translation : Vec3;
                        Scale, Scale_Sigma, Spread : Real;
                        Used_S      : Natural;
                        Similar     : Boolean;
                     begin
                        Fit.Similarity (Both (1 .. Nb), Rotation, Translation, Scale, Scale_Sigma, Spread, Used_S, Similar);
                        if not Similar then
                           Why := Ada.Strings.Unbounded.To_Unbounded_String
                             ("of" & Np'Image & " of the first arm's points it found," & Nb'Image
                              & " are its own too, too few to tell its scale");
                        else
                           declare
                              Placement  : Rigid;
                              Covariance : Fit.Real_Lists.Vector;
                              Px         : Real;
                              Used_P     : Natural;
                              Resected   : Boolean;
                           begin
                              Fit.Resect_Pose (Pairs (1 .. Np), L2, (Rotation => Rotation, Translation => Translation),
                                               Placement, Covariance, Px, Used_P, Resected);
                              if Resected then
                                 --  The placement's uncertainty: the resection's own, and
                                 --  that of both lenses, which it holds fixed: the arm's
                                 --  own, through which its eye saw the points, and the
                                 --  first arm's, along whose lines of sight its points lie
                                 --  (their depths held). Each lens term is moved by its
                                 --  standard deviation either way and the resection redone
                                 --  from the placement found; the change per unit of the
                                 --  term, carried with the lens's covariance.
                                 declare
                                    Total : Driver.Numerics.Arrays.Real_Matrix (1 .. 6, 1 .. 6);
                                    function Lens_Of (L : Lens_Fit) return Fit.Lens is
                                      ((Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2));
                                    function Moved (L : Fit.Lens; Term : Positive; By : Real) return Fit.Lens is
                                      (case Term is
                                          when 1      => (L with delta Fx => L.Fx * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
                                          when 2      => (L with delta Fy => L.Fy * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
                                          when 3      => (L with delta Cx => L.Cx + By),
                                          when 4      => (L with delta Cy => L.Cy + By),
                                          when 5      => (L with delta K1 => L.K1 + By),
                                          when others => (L with delta K2 => L.K2 + By));
                                 begin
                                    for P in 1 .. 6 loop
                                       for Q in 1 .. 6 loop
                                          Total (P, Q) := Covariance (Covariance.First_Index + (P - 1) * 6 + Q - 1);
                                       end loop;
                                    end loop;
                                    for Owner in 1 .. 2 loop
                                       declare
                                          F     : constant Arm_Fit := (if Owner = 1 then R1.Result else R2.Result);
                                          Terms : constant Natural :=
                                            Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                                       (Real (Natural (F.Covariance.Length))));
                                          function Lc (P, Q : Positive) return Real is
                                            (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
                                          Jl : Driver.Numerics.Arrays.Real_Matrix (1 .. 6, 1 .. Fit.Lens_Terms) :=
                                            [others => [others => 0.0]];
                                       begin
                                          if Terms >= Fit.Lens_Terms and then Terms * Terms = Natural (F.Covariance.Length) then
                                             for K in 1 .. Fit.Lens_Terms loop
                                                if Lc (K, K) > 0.0 then
                                                   declare
                                                      Sigma_K : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Lc (K, K));
                                                      Ends    : array (1 .. 2) of Rigid;
                                                      Ok_Both : Boolean := True;
                                                   begin
                                                      for E in 1 .. 2 loop
                                                         declare
                                                            By     : constant Real := (if E = 1 then -Sigma_K else Sigma_K);
                                                            Var    : Fit.Correspondence_Array := Pairs (1 .. Np);
                                                            Lv     : Fit.Lens := L2;
                                                            Cv     : Fit.Real_Lists.Vector;
                                                            Pv     : Real;
                                                            Uv     : Natural;
                                                            Okv    : Boolean;
                                                         begin
                                                            if Owner = 2 then
                                                               Lv := Moved (L2, K, By);
                                                            else
                                                               declare
                                                                  L1 : constant Fit.Lens := Moved (Lens_Of (R1.Result.Lens), K, By);
                                                               begin
                                                                  for J in 1 .. Np loop
                                                                     Var (J).X := Driver.Numerics.Arrays."*"
                                                                       (Track (R1.Result, Of_Query (J)) (3),
                                                                        Fit.Ray (L1, R1.Query_U (Of_Query (J)), R1.Query_V (Of_Query (J))));
                                                                  end loop;
                                                               end;
                                                            end if;
                                                            Fit.Resect_Pose (Var, Lv, Placement, Ends (E), Cv, Pv, Uv, Okv);
                                                            Ok_Both := Ok_Both and then Okv;
                                                         end;
                                                      end loop;
                                                      if Ok_Both then
                                                         declare
                                                            use Driver.Numerics.Arrays;
                                                            Turn  : constant Vec3 :=
                                                              Driver.Numerics.Log (Ends (2).Rotation * Transpose (Ends (1).Rotation));
                                                            Shift : constant Vec3 := Ends (2).Translation - Ends (1).Translation;
                                                         begin
                                                            for P in 1 .. 3 loop
                                                               Jl (P, K) := Turn (P) / (2.0 * Sigma_K);
                                                               Jl (3 + P, K) := Shift (P) / (2.0 * Sigma_K);
                                                            end loop;
                                                         end;
                                                      end if;
                                                   end;
                                                end if;
                                             end loop;
                                             for P in 1 .. 6 loop
                                                for Q in 1 .. 6 loop
                                                   for A in 1 .. Fit.Lens_Terms loop
                                                      for B in 1 .. Fit.Lens_Terms loop
                                                         Total (P, Q) := Total (P, Q) + Jl (P, A) * Lc (A, B) * Jl (Q, B);
                                                      end loop;
                                                   end loop;
                                                end loop;
                                             end loop;
                                          end if;
                                       end;
                                    end loop;
                                    --  And how well each fit's unit is known against its own
                                    --  depths: the first arm's points, by which the eye is
                                    --  placed, scale its centre about the world's origin; the
                                    --  scale compares both arms' depths.
                                    declare
                                       Rel_1 : constant Real := Unit_Sigma_Of (R1);
                                       Rel_2 : constant Real := Unit_Sigma_Of (R2);
                                       C     : constant Vec3 := Placement.Translation;
                                    begin
                                       if Rel_1 < Real'Last then
                                          for P in 1 .. 3 loop
                                             for Q in 1 .. 3 loop
                                                Total (3 + P, 3 + Q) := Total (3 + P, 3 + Q) + Rel_1 ** 2 * C (P) * C (Q);
                                             end loop;
                                          end loop;
                                       end if;
                                       Scale_Sigma := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                         (Scale_Sigma ** 2
                                          + (if Rel_1 < Real'Last and then Rel_2 < Real'Last
                                             then Scale ** 2 * (Rel_1 ** 2 + Rel_2 ** 2) else 0.0));
                                    end;
                                    Covariance.Clear;
                                    for P in 1 .. 6 loop
                                       for Q in 1 .. 6 loop
                                          Covariance.Append (Total (P, Q));
                                       end loop;
                                    end loop;
                                 end;
                                 R2.Result.Placed := True;
                                 R2.Result.Placement := Placement;
                                 R2.Result.Scale := Scale;
                                 R2.Result.Scale_Sigma := Scale_Sigma;
                                 R2.Result.Placement_Covariance.Clear;
                                 for X of Covariance loop
                                    R2.Result.Placement_Covariance.Append (X);
                                 end loop;
                                 R2.Result.Placed_Px := Px;
                                 R2.Result.Placed_Points := Used_P;
                                 Driver.Log.Line
                                   (Driver.Log.Robot, "kinematics: arm" & R2.Arm'Image & " placed in the world by"
                                    & Used_P'Image & " of the first arm's points its eye saw (noise "
                                    & Driver.Log.Image (Px, 3) & " px), its scale " & Driver.Log.Image (Scale, 4)
                                    & " +- " & Driver.Log.Image (Scale_Sigma, 4) & " from" & Used_S'Image & " points both tracked");
                              else
                                 Why := Ada.Strings.Unbounded.To_Unbounded_String
                                   ("its eye cannot be placed among the" & Np'Image & " points of the first arm it found");
                              end if;
                           end;
                        end if;
                     end;
                  end;
               end if;
               if not R2.Result.Placed and then Current (M, R2) and then R2.Result.Fitted then
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R2.Arm'Image & " not placed in the world: "
                                   & Ada.Strings.Unbounded.To_String (Why));
               end if;
            end;
         end if;
      end loop;
   end Place;

   procedure In_World (M : Model; A : Arm_Id; Placement : out Rigid; Scale : out Real; Known : out Boolean) is
      K : constant Natural := Index_Of (M, A);
   begin
      Placement := Identity;
      Scale := 1.0;
      Known := K > 0 and then M.Kinematics (K).Result.Fitted and then M.Kinematics (K).Result.Placed;
      if Known then
         Placement := M.Kinematics (K).Result.Placement;
         Scale := M.Kinematics (K).Result.Scale;
      end if;
   end In_World;

   procedure World_Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3) is
      use Driver.Numerics.Arrays;
      K : constant Natural := Index_Of (M, A);
      Unknown : constant Mat3 := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
   begin
      Turn := Unknown;
      Place := Unknown;
      if K = 0 or else not M.Kinematics (K).Result.Placed
        or else Natural (M.Kinematics (K).Result.Placement_Covariance.Length) /= 36
      then
         return;
      end if;
      declare
         R      : Arm_Fit renames M.Kinematics (K).Result;
         Own_Turn, Own_Place : Mat3;
         Rp     : constant Mat3 := R.Placement.Rotation;
         T      : constant Vec3 := Eye_In_Reference (M, A, Readings).Translation;
         Lever  : constant Vec3 := Rp * T;
         J      : constant Mat3 := -Driver.Numerics.Skew (R.Scale * Lever);
         function C (P, Q : Positive) return Real is (R.Placement_Covariance ((P - 1) * 6 + Q - 1));
         Tt, Cc, Tc : Mat3;
      begin
         Pose_Covariance (M, A, Readings, Own_Turn, Own_Place);
         if Own_Turn (1, 1) = Real'Last or else Own_Place (1, 1) = Real'Last then
            return;
         end if;
         for P in 1 .. 3 loop
            for Q in 1 .. 3 loop
               Tt (P, Q) := C (P, Q);
               Cc (P, Q) := C (3 + P, 3 + Q);
               Tc (P, Q) := C (3 + P, Q);
            end loop;
         end loop;
         --  The eye in the world: turn Rp R_a, place Scale Rp t_a + centre. Its
         --  turn takes the arm's own and the placement's; its place the arm's
         --  own scaled, the centre's, the placement's turn on the lever to the
         --  eye, and the scale's along it.
         Turn := Rp * Own_Turn * Transpose (Rp) + Tt;
         Place := R.Scale ** 2 * (Rp * Own_Place * Transpose (Rp)) + Cc + J * Tt * Transpose (J)
           + Tc * Transpose (J) + J * Transpose (Tc) + R.Scale_Sigma ** 2 * Driver.Numerics.Outer (Lever, Lever);
      end;
   end World_Pose_Covariance;

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid is
   begin
      for R of M.Kinematics loop
         if R.Arm = A and then Current (M, R) and then R.Result.Fitted
           and then Natural (R.Result.Joints.Length) = Readings'Length
           and then Natural (R.Result.Reference.Length) = Readings'Length
         then
            declare
               Joints : Fit.Joint_Array (1 .. Readings'Length);
               Change : Real_Array (1 .. Readings'Length);
            begin
               for J in Joints'Range loop
                  declare
                     F : constant Joint_Fit := R.Result.Joints (J);
                  begin
                     Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                     Change (J) := Readings (Readings'First + J - 1) - R.Result.Reference (J - 1);
                  end;
               end loop;
               return Fit.Eye_At (Joints, Change);
            end;
         end if;
      end loop;
      return Identity;
   end Eye_In_Reference;

   procedure Pose_Covariance (M : Model; A : Arm_Id; Readings : Real_Array; Turn, Place : out Mat3) is
      R : constant Arm_Fit := (if Index_Of (M, A) > 0 then M.Kinematics (Index_Of (M, A)).Result else (others => <>));
   begin
      Turn := [[Real'Last, 0.0, 0.0], [0.0, Real'Last, 0.0], [0.0, 0.0, Real'Last]];
      Place := Turn;
      if R.Fitted and then Natural (R.Joints.Length) = Readings'Length
        and then Natural (R.Reference.Length) = Readings'Length
      then
         declare
            Joints : Fit.Joint_Array (1 .. Readings'Length);
            Change : Real_Array (1 .. Readings'Length);
            Cov    : Fit.Real_Lists.Vector;
         begin
            for J in Joints'Range loop
               declare
                  F : constant Joint_Fit := R.Joints (J);
               begin
                  Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                  Change (J) := Readings (Readings'First + J - 1) - R.Reference (J - 1);
               end;
            end loop;
            for X of R.Covariance loop
               Cov.Append (X);
            end loop;
            Fit.Pose_Covariance (Joints, Change, Cov, Turn, Place);
         end;
      end if;
   end Pose_Covariance;

   function Fitted (M : Model; A : Arm_Id) return Boolean is
     (Index_Of (M, A) > 0 and then M.Kinematics (Index_Of (M, A)).Result.Fitted);

   procedure Solve_Pose
     (M             : Model;
      A             : Arm_Id;
      Start         : Real_Array;
      Goal          : Rigid;
      Position_Only : Boolean;
      Q             : out Real_Array;
      Position_Off  : out Real;
      Turn_Off      : out Real)
   is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      N    : constant Natural := Start'Length;
      Rows : constant Positive := (if Position_Only then 3 else 6);
      X    : Real_Array (1 .. N) := Start;
      Lambda : Real := Real'Model_Epsilon;

      function Residual (V : Real_Array) return Real_Vector is
         T : constant Rigid := Eye_In_Reference (M, A, V);
         R : Real_Vector (1 .. Rows);
         D : constant Vec3 := T.Translation - Goal.Translation;
      begin
         R (1 .. 3) := D;
         if not Position_Only then
            R (4 .. 6) := Driver.Numerics.Log (Transpose (T.Rotation) * Goal.Rotation);
         end if;
         return R;
      end Residual;

      function Cost (R : Real_Vector) return Real is (R * R);

      R0 : Real_Vector (1 .. Rows);
   begin
      R0 := Residual (X);
      loop
         declare
            J : Real_Matrix (1 .. Rows, 1 .. N);
            Improved, Lowered : Boolean := False;
         begin
            for K in 1 .. N loop
               declare
                  Xp : Real_Array := X;
                  H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (K));
                  Rp : Real_Vector (1 .. Rows);
               begin
                  Xp (K) := Xp (K) + H;
                  Rp := Residual (Xp);
                  for I in 1 .. Rows loop
                     J (I, K) := (Rp (I) - R0 (I)) / H;
                  end loop;
               end;
            end loop;
            declare
               JtJ : constant Real_Matrix := Transpose (J) * J;
               G   : constant Real_Vector := -(Transpose (J) * R0);
            begin
               loop
                  declare
                     D  : Real_Matrix := JtJ;
                     Lf : Real_Matrix (1 .. N, 1 .. N);
                     Pd : Boolean;
                     Moves : Boolean := False;
                  begin
                     for P in 1 .. N loop
                        D (P, P) := JtJ (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                     end loop;
                     Driver.Numerics.Dense.Cholesky (D, Lf, Pd);
                     if Pd then
                        declare
                           Step : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, G);
                           Xn   : Real_Array (1 .. N);
                        begin
                           for P in 1 .. N loop
                              Xn (P) := X (P) + Step (P);
                           end loop;
                           Moves := (for some P in 1 .. N => Xn (P) /= X (P));
                           if Moves then
                              declare
                                 Rn : constant Real_Vector := Residual (Xn);
                              begin
                                 if Cost (Rn) < Cost (R0) then
                                    Improved := Cost (R0) - Cost (Rn) > Driver.Conventions.Unchanged_Fraction * Cost (R0);
                                    X := Xn;
                                    R0 := Rn;
                                    Lambda := Lambda / 2.0;
                                    Lowered := True;
                                 end if;
                              end;
                           end if;
                        end;
                     else
                        Moves := True;
                     end if;
                     exit when Lowered or else not Moves;
                     Lambda := 2.0 * Lambda;
                  end;
               end loop;
            end;
            exit when not Improved;
         end;
      end loop;
      Q := X;
      Position_Off := Sqrt (R0 (1) ** 2 + R0 (2) ** 2 + R0 (3) ** 2);
      Turn_Off := (if Position_Only then 0.0 else Sqrt (R0 (4) ** 2 + R0 (5) ** 2 + R0 (6) ** 2));
   end Solve_Pose;

   function Result_Of (M : Model; A : Arm_Id) return Arm_Fit is
     (if Index_Of (M, A) > 0 then M.Kinematics (Index_Of (M, A)).Result else (others => <>));

   function Lens_Of (M : Model; A : Arm_Id) return Fit.Lens is
      L : constant Lens_Fit := Result_Of (M, A).Lens;
   begin
      return (Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2);
   end Lens_Of;

   function Angle_Sigma (M : Model; A : Arm_Id) return Real is
      R : constant Arm_Fit := Result_Of (M, A);
   begin
      return (if R.Fitted and then R.Lens.Fx > 0.0 and then R.Lens.Fy > 0.0
              then R.Sigma_Px / Real'Min (R.Lens.Fx, R.Lens.Fy) else Real'Last);
   end Angle_Sigma;

   function Ray_In_Eye (M : Model; A : Arm_Id; U, V : Real) return Vec3 is
      H : constant Vec3 := Fit.Ray (Lens_Of (M, A), U, V);
   begin
      return Unit (H);
   end Ray_In_Eye;

   procedure Project_In_Eye (M : Model; A : Arm_Id; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      Fit.Project (Lens_Of (M, A), P, U, V, In_Front);
   end Project_In_Eye;

end Driver.Robot.Kinematics;
