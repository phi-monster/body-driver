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
               Result : Answer_Access := new Driver.Instrument.Answer_Array (1 .. P.Points);
               Ok     : Boolean;
               Why    : Ada.Strings.Unbounded.Unbounded_String;
               Set    : Match_Set;
            begin
               Driver.Instrument.Read_Match (Reply, True, Result.all, Ok, Why);
               if Ok then
                  Set.Frame := P.Frame;
                  Set.Eye := P.Eye;
                  for A of Result.all loop
                     Set.To_U.Append (A.To.U);
                     Set.To_V.Append (A.To.V);
                     Set.Back_U.Append (A.Back.U);
                     Set.Back_V.Append (A.Back.V);
                     Set.Found.Append (A.Found);
                  end loop;
                  Into.Append (Set);
               elsif Reply.Lasting and then P.Eye > 0 then
                  --  Another eye's view of the reference: that link is lost; the
                  --  arm's own sweep learns from its own requests whether the
                  --  instrument can answer at all.
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: eye" & P.Eye'Image & "'s view of arm" & R.Arm'Image
                                   & "'s reference gets no matches: " & Ada.Strings.Unbounded.To_String (Why));
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
      Collect_Into (R, R.Eye_Pending, R.Eye_Matches);
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
                              --  The reference's points into every other eye's view at
                              --  this beat: what links this arm to the others through
                              --  an eye that sees both (Place).
                              R.Eye_Matches.Clear;
                              if not R.Query_U.Is_Empty then
                                 declare
                                    Points : Point_Access :=
                                      new Driver.Instrument.Point_Array (1 .. Natural (R.Query_U.Length));
                                 begin
                                    for P in Points'Range loop
                                       Points (P) := (U => R.Query_U (P - 1), V => R.Query_V (P - 1));
                                    end loop;
                                    for Other in O.Images.First_Index .. O.Images.Last_Index loop
                                       if Other /= E and then Driver.Observations.Has_Image (O, Other) then
                                          R.Eye_Pending.Append
                                            (Pending_Match'(Frame  => 1,
                                                            Eye    => Natural (Other),
                                                            Points => Points'Length,
                                                            Ticket => Driver.Instrument.Submit_Match
                                                              ((Stored => False, Image => O.Images (E)),
                                                               (Stored => False, Image => O.Images (Other)),
                                                               Points.all, True, O.Beat)));
                                       end if;
                                    end loop;
                                    Free (Points);
                                 end;
                              end if;
                           elsif not R.Query_U.Is_Empty and then not R.Unanswerable then
                              declare
                                 Points : Point_Access := new Driver.Instrument.Point_Array (1 .. Natural (R.Query_U.Length));
                              begin
                                 for P in Points'Range loop
                                    Points (P) := (U => R.Query_U (P - 1), V => R.Query_V (P - 1));
                                 end loop;
                                 R.Pending.Append
                                   (Pending_Match'(Frame  => R.Frames.Last_Index,
                                     Eye    => 0,
                                     Points => Points'Length,
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
                                          Eye    => 0,
                                          Points => Points'Length,
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
         K := K + Natural (R.Pending.Length) + Natural (R.World_Pending.Length) + Natural (R.Eye_Pending.Length);
      end loop;
      return K;
   end Pending;

   --  Every arm but the first placed in the world, by its reference view of
   --  the first arm's tracked points (In_World).
   procedure Place (M : in out Model);

   --  The lens of a fit, as Fit has it.
   function Fit_Lens (L : Lens_Fit) return Fit.Lens is
     ((Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2));

   --  The lens moved along its term Term (in Fit.Lens_Terms' order) by By.
   function Moved (L : Fit.Lens; Term : Positive; By : Real) return Fit.Lens is
     (case Term is
         when 1      => (L with delta Fx => L.Fx * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
         when 2      => (L with delta Fy => L.Fy * Ada.Numerics.Long_Elementary_Functions.Exp (By)),
         when 3      => (L with delta Cx => L.Cx + By),
         when 4      => (L with delta Cy => L.Cy + By),
         when 5      => (L with delta K1 => L.K1 + By),
         when others => (L with delta K2 => L.K2 + By));

   type Sight_Access is access Fit.Sight_Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Sight_Point_Array, Sight_Access);
   type Fit_Flag_Access is access Fit.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Flag_Array, Fit_Flag_Access);
   type Plane_Point_Access is access Fit.Plane_Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fit.Plane_Point_Array, Plane_Point_Access);

   --  The arm's tracks as its reference eye has them through lens L: on
   --  their reference lines of sight, at the depths the fit refined.
   procedure Sights_Of (U, V : Real_Vectors.Vector; F : Arm_Fit; L : Fit.Lens; S : out Fit.Sight_Point_Array)
     with Pre => S'First = 1 and then S'Length = Natural (U.Length) and then Natural (V.Length) = Natural (U.Length)
   is
   begin
      for I in S'Range loop
         declare
            K : constant Natural := I - 1;
         begin
            S (I) :=
              (H     => Fit.Ray (L, U (K), V (K)),
               Depth => (if K < Natural (F.Track_Known.Length) and then F.Track_Known (K)
                         then F.Tracks (3 * K + 2) else 0.0),
               Sigma => (if K < Natural (F.Track_Sigmas.Length) then F.Track_Sigmas (K) else Real'Last));
         end;
      end loop;
   end Sights_Of;

   --  Which of the arm's tracks lie on its table (Fit.Dominant_Plane).
   procedure Table_Of (F : Arm_Fit; On : out Fit.Flag_Array)
     with Pre => On'First = 1
   is
   begin
      for I in On'Range loop
         On (I) := I - 1 < Natural (F.Table_On.Length) and then F.Table_On (I - 1);
      end loop;
   end Table_Of;

   --  The table normal's uncertainty, its points' and its lens's: the plane
   --  refitted on the same points with each lens term moved by its standard
   --  deviation either way (the depths held), the normal's change per unit of
   --  the term carried with the lens's covariance.
   function Table_Sigma_Of
     (U, V       : Real_Vectors.Vector;
      F          : Arm_Fit;
      L          : Fit.Lens;
      Plane      : Fit.Sight_Plane;
      Covariance : Fit.Real_Lists.Vector) return Real
   is
      Q     : constant Natural := Natural (U.Length);
      Terms : constant Natural := Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                             (Real (Natural (Covariance.Length))));
      N0    : constant Vec3 := Fit.Plane_Normal (Plane);
      Total : Mat3;
      Jl    : Driver.Numerics.Arrays.Real_Matrix (1 .. 3, 1 .. Fit.Lens_Terms) := [others => [others => 0.0]];
      On    : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q);
      S     : Sight_Access := new Fit.Sight_Point_Array (1 .. Q);

      function Lc (P, C : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Terms + C - 1));
   begin
      --  The points' share: the normal's change across the plane.
      declare
         use Driver.Numerics.Arrays;
         Length : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Plane.A * Plane.A);
         Pr     : constant Mat3 := Driver.Numerics.Identity3 - Driver.Numerics.Outer (N0, N0);
      begin
         Total := (1.0 / Length ** 2) * (Pr * Plane.Covariance * Pr);
      end;
      Table_Of (F, On.all);
      if Terms >= Fit.Lens_Terms and then Terms * Terms = Natural (Covariance.Length) then
         for K in 1 .. Fit.Lens_Terms loop
            if Lc (K, K) > 0.0 then
               declare
                  Sigma_K : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Lc (K, K));
                  Ends    : array (1 .. 2) of Vec3;
                  Ok      : Boolean := True;
               begin
                  for E in 1 .. 2 loop
                     declare
                        P : Fit.Sight_Plane;
                     begin
                        Sights_Of (U, V, F, Moved (L, K, (if E = 1 then -Sigma_K else Sigma_K)), S.all);
                        --  The depths held, the points move with their lines of sight.
                        Fit.Refit_Plane (S.all, On.all, P);
                        Ok := Ok and then P.Found;
                        Ends (E) := Fit.Plane_Normal (P);
                     end;
                  end loop;
                  if Ok then
                     for D in 1 .. 3 loop
                        Jl (D, K) := (Ends (2) (D) - Ends (1) (D)) / (2.0 * Sigma_K);
                     end loop;
                  end if;
               end;
            end if;
         end loop;
         for P in 1 .. 3 loop
            for C in 1 .. 3 loop
               for A in 1 .. Fit.Lens_Terms loop
                  for B in 1 .. Fit.Lens_Terms loop
                     Total (P, C) := Total (P, C) + Jl (P, A) * Lc (A, B) * Jl (C, B);
                  end loop;
               end loop;
            end loop;
         end loop;
      end if;
      Free (On);
      Free (S);
      declare
         Values  : Vec3;
         Vectors : Mat3;
      begin
         Driver.Numerics.Symmetric_Eigensystem (Total, Values, Vectors);
         return Ada.Numerics.Long_Elementary_Functions.Sqrt
           (Real'Max (0.0, Real'Max (Values (1), Real'Max (Values (2), Values (3)))));
      end;
   end Table_Sigma_Of;

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
                              Plane  : Fit.Sight_Plane;
                              Sights : Sight_Access := new Fit.Sight_Point_Array (1 .. Queries);
                              On     : Fit_Flag_Access := new Fit.Flag_Array (1 .. Queries);
                           begin
                              --  The points where the fit put them: each track's refined
                              --  depth along its reference line of sight.
                              for I in 0 .. Queries - 1 loop
                                 declare
                                    D : constant Real :=
                                      (if I < Natural (Report.Depths.Length) then Report.Depths (Report.Depths.First_Index + I)
                                       else 0.0);
                                    S : constant Real :=
                                      (if I < Natural (Report.Depth_Sigmas.Length)
                                       then Report.Depth_Sigmas (Report.Depth_Sigmas.First_Index + I) else Real'Last);
                                    H : constant Vec3 := Fit.Ray (Lens, R.Query_U (I), R.Query_V (I));
                                 begin
                                    Result.Track_Known.Append (D > 0.0);
                                    Result.Track_Sigmas.Append (S);
                                    for X of H loop
                                       Result.Tracks.Append (D * X);
                                    end loop;
                                    Sights (I + 1) := (H => H, Depth => D, Sigma => S);
                                 end;
                              end loop;
                              --  The table: the plane most of them lie on.
                              Fit.Dominant_Plane (Sights.all, Plane, On.all);
                              for B of On.all loop
                                 Result.Table_On.Append (B);
                              end loop;
                              Result.Table_Found := Plane.Found;
                              Result.Table_A := Plane.A;
                              Result.Table_Covariance := Plane.Covariance;
                              Result.Table_Normal := Fit.Plane_Normal (Plane);
                              Result.Table_Offset := Fit.Plane_Offset (Plane);
                              Result.Table_Offset_Sigma := Fit.Plane_Offset_Sigma (Plane);
                              Result.Table_Sigma :=
                                (if Plane.Found
                                 then Table_Sigma_Of (R.Query_U, R.Query_V, Result, Lens, Plane, Report.Covariance)
                                 else Real'Last);
                              if Plane.Found and then R.Arm = 1 then
                                 M.Table_Up := (Unit_Vector => Result.Table_Normal, Sigma => Result.Table_Sigma);
                              end if;
                              Free (Sights);
                              Free (On);
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

      --  Eye E's view shows group G move, as the lock-in measured it.
      function Sees (E : Natural; G : Group_Id) return Boolean is
         K : constant Natural := (Natural (G) - 1) * Natural (M.Eyes.Length) + E;
      begin
         return E > 0 and then K in M.Graph.Effects.First_Index .. M.Graph.Effects.Last_Index
           and then M.Graph.Effects (K).Verdict in Patch | Whole;
      end Sees;

      --  The match set of the arm's reference into eye E; 0 when there is none.
      function Set_Into (R : Arm_Evidence; E : Natural) return Natural is
      begin
         for K in R.Eye_Matches.First_Index .. R.Eye_Matches.Last_Index loop
            if R.Eye_Matches (K).Eye = E then
               return K;
            end if;
         end loop;
         return 0;
      end Set_Into;

      --  A way to link the second arm to the first: an eye that shows both
      --  arms move and has both arms' table points in its view. A fixed eye
      --  has each arm's reference matched into its view at that arm's
      --  reference beat; the arm's own eye has the first arm's reference
      --  matched into its own, and its own points where they are.
      type Link_Way is record
         Eye : Natural := 0;
         Own : Boolean := False;
      end record;

      --  The plane points of both arms in the eye of a way: each arm's tracks
      --  on its table, where its lines of sight through lenses L1 and L2 meet
      --  planes P1 and P2, in those planes' coordinates; and where the eye
      --  sees them: its pixels for a fixed eye, the second arm's lines of
      --  sight through L2 for its own. Which points enter depends only on the
      --  evidence, so every variant has the same points in the same order.
      procedure Points_Of
        (R1, R2  : Arm_Evidence;
         Way     : Link_Way;
         L1, L2  : Fit.Lens;
         P1, P2  : Fit.Sight_Plane;
         First   : out Plane_Point_Access;
         Second  : out Plane_Point_Access)
      is
         Q1 : constant Natural := Natural (R1.Query_U.Length);
         Q2 : constant Natural := Natural (R2.Query_U.Length);
         On1 : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q1);
         On2 : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q2);
         Set_1 : constant Natural := (if Way.Own then 0 else Set_Into (R1, Way.Eye));
         Set_2 : constant Natural := (if Way.Own then 0 else Set_Into (R2, Way.Eye));
         S1  : constant Match_Set := (if Way.Own then R2.World_Matches.First_Element else R1.Eye_Matches (Set_1));
         N1, N2 : Natural := 0;
         Normal_1, A1, B1, Normal_2, A2, B2 : Vec3;

         function Enters_1 (K : Natural) return Boolean is
           (On1 (K + 1) and then K < Natural (S1.Found.Length) and then S1.Found (K));
         function Enters_2 (K : Natural) return Boolean is
           (On2 (K + 1)
            and then (Way.Own
                      or else (K < Natural (R2.Eye_Matches (Set_2).Found.Length)
                               and then R2.Eye_Matches (Set_2).Found (K))));

         --  Where the eye sees a pixel of its view: as it is for a fixed eye,
         --  on the second arm's line of sight for its own.
         procedure In_Eye (U, V : Real; X, Y : out Real) is
         begin
            if Way.Own then
               declare
                  H : constant Vec3 := Fit.Ray (L2, U, V);
               begin
                  X := H (1);
                  Y := H (2);
               end;
            else
               X := U;
               Y := V;
            end if;
         end In_Eye;
      begin
         Table_Of (R1.Result, On1.all);
         Table_Of (R2.Result, On2.all);
         Fit.Plane_Axes (P1, Normal_1, A1, B1);
         Fit.Plane_Axes (P2, Normal_2, A2, B2);
         for K in 0 .. Q1 - 1 loop
            if Enters_1 (K) then
               N1 := N1 + 1;
            end if;
         end loop;
         for K in 0 .. Q2 - 1 loop
            if Enters_2 (K) then
               N2 := N2 + 1;
            end if;
         end loop;
         First := new Fit.Plane_Point_Array (1 .. N1);
         Second := new Fit.Plane_Point_Array (1 .. N2);
         N1 := 0;
         for K in 0 .. Q1 - 1 loop
            if Enters_1 (K) then
               declare
                  use Driver.Numerics.Arrays;
                  X : constant Vec3 := Fit.On_Plane (P1, Fit.Ray (L1, R1.Query_U (K), R1.Query_V (K)));
               begin
                  N1 := N1 + 1;
                  First (N1).X := X * A1;
                  First (N1).Y := X * B1;
                  In_Eye (S1.To_U (K), S1.To_V (K), First (N1).U, First (N1).V);
               end;
            end if;
         end loop;
         N2 := 0;
         for K in 0 .. Q2 - 1 loop
            if Enters_2 (K) then
               declare
                  use Driver.Numerics.Arrays;
                  X : constant Vec3 := Fit.On_Plane (P2, Fit.Ray (L2, R2.Query_U (K), R2.Query_V (K)));
               begin
                  N2 := N2 + 1;
                  Second (N2).X := X * A2;
                  Second (N2).Y := X * B2;
                  if Way.Own then
                     In_Eye (R2.Query_U (K), R2.Query_V (K), Second (N2).U, Second (N2).V);
                  else
                     Second (N2).U := R2.Eye_Matches (Set_2).To_U (K);
                     Second (N2).V := R2.Eye_Matches (Set_2).To_V (K);
                  end if;
               end;
            end if;
         end loop;
         Free (On1);
         Free (On2);
      end Points_Of;

      --  Each arm's table refitted through lens L on the same tracks.
      function Table_Through (R : Arm_Evidence; L : Fit.Lens) return Fit.Sight_Plane is
         Q  : constant Natural := Natural (R.Query_U.Length);
         S  : Sight_Access := new Fit.Sight_Point_Array (1 .. Q);
         On : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q);
         P  : Fit.Sight_Plane;
      begin
         Sights_Of (R.Query_U, R.Query_V, R.Result, L, S.all);
         Table_Of (R.Result, On.all);
         Fit.Refit_Plane (S.all, On.all, P);
         Free (S);
         Free (On);
         return P;
      end Table_Through;

      Outputs : constant := 7;   --  turn, centre, log scale
      type Output_Covariance is array (1 .. Outputs, 1 .. Outputs) of Real;

      --  The turn (world frame), the centre and the log scale of B against A.
      function Change (A, B : Rigid; Scale_A, Scale_B : Real) return Real_Array is
         use Driver.Numerics.Arrays;
         Turn  : constant Vec3 := Driver.Numerics.Log (B.Rotation * Transpose (A.Rotation));
         Shift : constant Vec3 := B.Translation - A.Translation;
      begin
         return [Turn (1), Turn (2), Turn (3), Shift (1), Shift (2), Shift (3),
                 Ada.Numerics.Long_Elementary_Functions.Log (Scale_B / Scale_A)];
      end Change;

      --  Placed through one way: the link, the placement and its covariance;
      --  Why when it cannot be.
      procedure Through
        (R1, R2     : Arm_Evidence;
         Way        : Link_Way;
         Placement  : out Rigid;
         Scale      : out Real;
         Covariance : out Output_Covariance;
         Link       : out Fit.Plane_Link;
         Placed     : out Boolean;
         Why        : out Ada.Strings.Unbounded.Unbounded_String)
      is
         L1 : constant Fit.Lens := Fit_Lens (R1.Result.Lens);
         L2 : constant Fit.Lens := Fit_Lens (R2.Result.Lens);
         P1 : constant Fit.Sight_Plane := Table_Through (R1, L1);
         P2 : constant Fit.Sight_Plane := Table_Through (R2, L2);
         First, Second : Plane_Point_Access;

         --  Placed again with the lenses, the planes or the similarity moved:
         --  the link refitted from the one found (Again) on the same points.
         procedure Variant
           (Lv1, Lv2 : Fit.Lens;
            Q1, Q2   : Fit.Sight_Plane;
            Lv       : in out Fit.Plane_Link;
            Again    : Boolean;
            Pv       : out Rigid;
            Sv       : out Real;
            Ok       : out Boolean)
         is
            F, S : Plane_Point_Access;
         begin
            Ok := Q1.Found and then Q2.Found;
            Pv := Driver.Numerics.Identity;
            Sv := 1.0;
            if not Ok then
               return;
            end if;
            if Again then
               Points_Of (R1, R2, Way, Lv1, Lv2, Q1, Q2, F, S);
               if F'Length = First'Length and then S'Length = Second'Length then
                  Fit.Plane_Chain_Again (F.all, S.all, Lv);
                  Ok := Lv.Found;
               else
                  Ok := False;
               end if;
               Free (F);
               Free (S);
            end if;
            if Ok then
               Fit.Chain_Placement (Q1, Q2, Lv, Pv, Sv);
               Ok := Sv > 0.0;
            end if;
         end Variant;

         --  One source's share of the covariance: the output's change per unit
         --  of each of its terms, carried with the source's own covariance.
         procedure Add
           (Jacobian : Driver.Numerics.Arrays.Real_Matrix;
            Source   : Driver.Numerics.Arrays.Real_Matrix)
         is
         begin
            for P in 1 .. Outputs loop
               for Q in 1 .. Outputs loop
                  for A in Source'Range (1) loop
                     for B in Source'Range (2) loop
                        Covariance (P, Q) := Covariance (P, Q)
                          + Jacobian (P, Jacobian'First (2) + A - Source'First (1)) * Source (A, B)
                            * Jacobian (Q, Jacobian'First (2) + B - Source'First (2));
                     end loop;
                  end loop;
               end loop;
            end loop;
         end Add;
      begin
         Placement := Driver.Numerics.Identity;
         Scale := 1.0;
         Covariance := [others => [others => 0.0]];
         Link := (others => <>);
         Placed := False;
         Why := Ada.Strings.Unbounded.Null_Unbounded_String;
         if not (P1.Found and then P2.Found) then
            Why := Ada.Strings.Unbounded.To_Unbounded_String ("an arm's table is not found");
            return;
         end if;
         Points_Of (R1, R2, Way, L1, L2, P1, P2, First, Second);
         Fit.Plane_Chain (First.all, Second.all, Link);
         if not Link.Found then
            Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("of their tables" & First'Length'Image & " and" & Second'Length'Image
               & " points are in its view, too few for a homography of each");
         elsif not Link.Consistent then
            Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("their tables are not one plane in its view:" & Link.Beyond'Image & " of the"
               & Natural'Image (2 * Link.Used) & " coordinates of the joint fit lie beyond Z of the homographies'"
               & " own noise," & Link.Apart'Image);
         else
            Fit.Chain_Placement (P1, P2, Link, Placement, Scale);
            Placed := True;
            --  The similarity's own uncertainty.
            declare
               Jacobian : Driver.Numerics.Arrays.Real_Matrix (1 .. Outputs, 1 .. 4) := [others => [others => 0.0]];
               Source   : Driver.Numerics.Arrays.Real_Matrix (1 .. 4, 1 .. 4);
            begin
               for A in 1 .. 4 loop
                  for B in 1 .. 4 loop
                     Source (A, B) := Link.Covariance (Link.Covariance.First_Index + (A - 1) * 4 + B - 1);
                  end loop;
               end loop;
               for K in 1 .. 4 loop
                  if Source (K, K) > 0.0 then
                     declare
                        Step : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Source (K, K));
                        Ends : array (1 .. 2) of Real_Array (1 .. Outputs);
                     begin
                        for E in 1 .. 2 loop
                           declare
                              By : constant Real := (if E = 1 then -Step else Step);
                              Lv : Fit.Plane_Link := Link;
                              Pv : Rigid;
                              Sv : Real;
                              Ok : Boolean;
                           begin
                              case K is
                                 when 1      => Lv.Scale := Link.Scale * Ada.Numerics.Long_Elementary_Functions.Exp (By);
                                 when 2      => Lv.Turn := Link.Turn + By;
                                 when 3      => Lv.Shift_X := Link.Shift_X + By;
                                 when others => Lv.Shift_Y := Link.Shift_Y + By;
                              end case;
                              Variant (L1, L2, P1, P2, Lv, False, Pv, Sv, Ok);
                              Ends (E) := Change (Placement, Pv, Scale, Sv);
                           end;
                        end loop;
                        for P in 1 .. Outputs loop
                           Jacobian (P, K) := (Ends (2) (P) - Ends (1) (P)) / (2.0 * Step);
                        end loop;
                     end;
                  end if;
               end loop;
               Add (Jacobian, Source);
            end;
            --  Each table's own uncertainty, along its covariance's axes.
            for Arm in 1 .. 2 loop
               declare
                  P       : constant Fit.Sight_Plane := (if Arm = 1 then P1 else P2);
                  Values  : Vec3;
                  Vectors : Mat3;
               begin
                  Driver.Numerics.Symmetric_Eigensystem (P.Covariance, Values, Vectors);
                  for Axis in 1 .. 3 loop
                     if Values (Axis) > 0.0 then
                        declare
                           use Driver.Numerics.Arrays;
                           Step : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Values (Axis));
                           Dir  : constant Vec3 := [Vectors (1, Axis), Vectors (2, Axis), Vectors (3, Axis)];
                           Ends : array (1 .. 2) of Real_Array (1 .. Outputs);
                           Ok_Both : Boolean := True;
                        begin
                           for E in 1 .. 2 loop
                              declare
                                 Moved_P : Fit.Sight_Plane := P;
                                 Lv : Fit.Plane_Link := Link;
                                 Pv : Rigid;
                                 Sv : Real;
                                 Ok : Boolean;
                              begin
                                 Moved_P.A := P.A + (if E = 1 then -Step else Step) * Dir;
                                 if Arm = 1 then
                                    Variant (L1, L2, Moved_P, P2, Lv, True, Pv, Sv, Ok);
                                 else
                                    Variant (L1, L2, P1, Moved_P, Lv, True, Pv, Sv, Ok);
                                 end if;
                                 Ok_Both := Ok_Both and then Ok;
                                 Ends (E) := Change (Placement, Pv, Scale, Sv);
                              end;
                           end loop;
                           if Ok_Both then
                              declare
                                 One_Sigma : Driver.Numerics.Arrays.Real_Matrix (1 .. Outputs, 1 .. 1);
                              begin
                                 for O in 1 .. Outputs loop
                                    One_Sigma (O, 1) := (Ends (2) (O) - Ends (1) (O)) / 2.0;
                                 end loop;
                                 Add (One_Sigma, [1 => [1 => 1.0]]);
                              end;
                           end if;
                        end;
                     end if;
                  end loop;
               end;
            end loop;
            --  Each arm's lens: its lines of sight move, the depths held; the
            --  table refitted on the same tracks, the link on the same points.
            for Arm in 1 .. 2 loop
               declare
                  F     : constant Arm_Fit := (if Arm = 1 then R1.Result else R2.Result);
                  Terms : constant Natural :=
                    Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real (Natural (F.Covariance.Length))));
                  Jacobian : Driver.Numerics.Arrays.Real_Matrix (1 .. Outputs, 1 .. Fit.Lens_Terms) :=
                    [others => [others => 0.0]];
                  Source   : Driver.Numerics.Arrays.Real_Matrix (1 .. Fit.Lens_Terms, 1 .. Fit.Lens_Terms);
               begin
                  if Terms >= Fit.Lens_Terms and then Terms * Terms = Natural (F.Covariance.Length) then
                     for A in 1 .. Fit.Lens_Terms loop
                        for B in 1 .. Fit.Lens_Terms loop
                           Source (A, B) := F.Covariance (F.Covariance.First_Index + (A - 1) * Terms + B - 1);
                        end loop;
                     end loop;
                     for K in 1 .. Fit.Lens_Terms loop
                        if Source (K, K) > 0.0 then
                           declare
                              Step : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Source (K, K));
                              Ends : array (1 .. 2) of Real_Array (1 .. Outputs);
                              Ok_Both : Boolean := True;
                           begin
                              for E in 1 .. 2 loop
                                 declare
                                    By  : constant Real := (if E = 1 then -Step else Step);
                                    Lv1 : constant Fit.Lens := (if Arm = 1 then Moved (L1, K, By) else L1);
                                    Lv2 : constant Fit.Lens := (if Arm = 2 then Moved (L2, K, By) else L2);
                                    Lv  : Fit.Plane_Link := Link;
                                    Pv  : Rigid;
                                    Sv  : Real;
                                    Ok  : Boolean;
                                 begin
                                    Variant (Lv1, Lv2, Table_Through (R1, Lv1), Table_Through (R2, Lv2), Lv, True,
                                             Pv, Sv, Ok);
                                    Ok_Both := Ok_Both and then Ok;
                                    Ends (E) := Change (Placement, Pv, Scale, Sv);
                                 end;
                              end loop;
                              if Ok_Both then
                                 for P in 1 .. Outputs loop
                                    Jacobian (P, K) := (Ends (2) (P) - Ends (1) (P)) / (2.0 * Step);
                                 end loop;
                              end if;
                           end;
                        end if;
                     end loop;
                     Add (Jacobian, Source);
                  end if;
               end;
            end loop;
            --  And how well each fit's unit is known against its own depths:
            --  the first arm's scales the world about its origin, so the
            --  centre; both compare in the scale.
            declare
               Rel_1 : constant Real := Unit_Sigma_Of (R1);
               Rel_2 : constant Real := Unit_Sigma_Of (R2);
               C     : constant Vec3 := Placement.Translation;
            begin
               if Rel_1 < Real'Last then
                  for P in 1 .. 3 loop
                     for Q in 1 .. 3 loop
                        Covariance (3 + P, 3 + Q) := Covariance (3 + P, 3 + Q) + Rel_1 ** 2 * C (P) * C (Q);
                     end loop;
                  end loop;
               end if;
               if Rel_1 < Real'Last and then Rel_2 < Real'Last then
                  Covariance (Outputs, Outputs) := Covariance (Outputs, Outputs) + Rel_1 ** 2 + Rel_2 ** 2;
               end if;
            end;
         end if;
         Free (First);
         Free (Second);
      end Through;
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
            R.Placed_Through := 0;
            R.Placement_Covariance.Clear;
            R.Placement_Covariance.Append (0.0, 36);
         end;
      end if;
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if K /= W then
            declare
               R2  : Arm_Evidence renames M.Kinematics (K);
               Why : Ada.Strings.Unbounded.Unbounded_String;
            begin
               R2.Result.Placed := False;
               if W = 0 or else not M.Kinematics (W).Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("the first arm is not fitted");
               elsif not Current (M, R2) or else not R2.Result.Fitted then
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("it is not fitted");
               else
                  declare
                     use type Ada.Strings.Unbounded.Unbounded_String;
                     R1    : Arm_Evidence renames M.Kinematics (W);
                     Best  : Real := Real'Last;
                     Ways  : Natural := 0;
                  begin
                     --  Every eye that shows both arms move, its own and the fixed
                     --  ones: the measured overlap.
                     for E in 1 .. Natural (M.Eyes.Length) loop
                        declare
                           Way   : constant Link_Way := (Eye => E, Own => E = Natural (R2.Eye));
                           Fixed : constant Boolean :=
                             E <= Natural (M.Graph.Mounts.Length) and then M.Graph.Mounts (Eye_Id (E)).Kind = World_Fixed;
                           Ready : constant Boolean :=
                             (if Way.Own
                              then not R2.World_Matches.Is_Empty and then R2.World_Group = R1.Group
                                   and then not R1.Frames.Is_Empty
                                   and then R2.World_Reference = R1.Frames.First_Element.Beat
                              else Set_Into (R1, E) > 0 and then Set_Into (R2, E) > 0);
                        begin
                           if (Fixed or else Way.Own) and then Sees (E, R1.Group) and then Sees (E, R2.Group) then
                              if not Ready then
                                 Why := Why & "eye" & E'Image & " shows both arms, but its view of their points is not"
                                        & " answered yet; ";
                              else
                                 declare
                                    Placement  : Rigid;
                                    Scale      : Real;
                                    Covariance : Output_Covariance;
                                    Link       : Fit.Plane_Link;
                                    Placed     : Boolean;
                                    Not_Placed : Ada.Strings.Unbounded.Unbounded_String;
                                 begin
                                    Ways := Ways + 1;
                                    Through (R1, R2, Way, Placement, Scale, Covariance, Link, Placed, Not_Placed);
                                    if not Placed then
                                       Why := Why & "through eye" & E'Image & ", " & Not_Placed & "; ";
                                    elsif Covariance (4, 4) + Covariance (5, 5) + Covariance (6, 6) < Best then
                                       Best := Covariance (4, 4) + Covariance (5, 5) + Covariance (6, 6);
                                       R2.Result.Placed := True;
                                       R2.Result.Placement := Placement;
                                       R2.Result.Scale := Scale;
                                       R2.Result.Scale_Sigma :=
                                         Scale * Ada.Numerics.Long_Elementary_Functions.Sqrt
                                                   (Real'Max (0.0, Covariance (Outputs, Outputs)));
                                       R2.Result.Placement_Covariance.Clear;
                                       for P in 1 .. 6 loop
                                          for Q in 1 .. 6 loop
                                             R2.Result.Placement_Covariance.Append (Covariance (P, Q));
                                          end loop;
                                       end loop;
                                       R2.Result.Placed_Px := Link.Sigma;
                                       R2.Result.Placed_Points := Link.Used;
                                       R2.Result.Placed_Through := E;
                                       Driver.Log.Line
                                         (Driver.Log.Robot, "kinematics: arm" & R2.Arm'Image & " placed in the world"
                                          & " through eye" & E'Image & (if Way.Own then " (its own)" else "")
                                          & ", which sees both arms' tables as one plane:" & Link.Used'Image
                                          & " of their points (noise " & Driver.Log.Image (Link.Sigma, 3) & " against "
                                          & Driver.Log.Image (Link.Apart, 3) & " apart); its scale "
                                          & Driver.Log.Image (Scale, 4) & " +- "
                                          & Driver.Log.Image (R2.Result.Scale_Sigma, 4) & ", its eye to "
                                          & Driver.Log.Image (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                                                (Covariance (4, 4) + Covariance (5, 5) + Covariance (6, 6)), 4)
                                          & " world units and "
                                          & Driver.Log.Image (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                                                (Covariance (1, 1) + Covariance (2, 2) + Covariance (3, 3)), 4)
                                          & " rad");
                                    end if;
                                 end;
                              end if;
                           end if;
                        end;
                     end loop;
                     if Ways = 0 and then Ada.Strings.Unbounded.Length (Why) = 0 then
                        Why := Ada.Strings.Unbounded.To_Unbounded_String ("no eye shows both it and the first arm move");
                     end if;
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
