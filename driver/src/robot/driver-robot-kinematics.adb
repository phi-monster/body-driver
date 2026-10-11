with Ada.Characters.Handling;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Alignment;
with Driver.Alignment.Keyframes;
with Driver.Clock;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Numerics.Dense;
with Driver.Parallel_For;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Fixed;
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
      Collect_Into (R, R.World_Pending, R.World_Matches);
      Collect_Into (R, R.Eye_Pending, R.Eye_Matches);
   end Collect;

   --  The arm's joints along it, from its base, as its fit found them.
   function Chain_Of (R : Arm_Fit) return Fit.Chain_Order is
     ([for K in 1 .. Natural (R.Chain.Length) => Positive (R.Chain (R.Chain.First_Index + K - 1))]);

   function Chain_Image (C : Fit.Chain_Order) return String is
     (if C'Length = 0 then ""
      elsif C'Length = 1 then Ada.Strings.Fixed.Trim (C (C'First)'Image, Ada.Strings.Left)
      else Ada.Strings.Fixed.Trim (C (C'First)'Image, Ada.Strings.Left) & ", " & Chain_Image (C (C'First + 1 .. C'Last)));

   --  How long each stage of a fit took, in seconds.
   function Stages_Image (Report : Fit.Fit_Report) return String is
      Text : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for K in Report.Seconds.First_Index .. Report.Seconds.Last_Index loop
         Ada.Strings.Unbounded.Append
           (Text, (if K = Report.Seconds.First_Index then "; its stages took " else ", ")
                  & Driver.Log.Image (Report.Seconds (K), 1));
      end loop;
      return Ada.Strings.Unbounded.To_String (Text) & (if Report.Seconds.Is_Empty then "" else " s");
   end Stages_Image;

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

   --  The noise of a round trip over the answers of Sets: the robust scale
   --  about zero of every answer's return to its query, both coordinates; 0
   --  without answers.
   function Round_Trip_Sigma (R : Arm_Evidence; Sets : Match_Set_Vectors.Vector) return Real is
      Queries : constant Natural := Natural (R.Query_U.Length);
      Count   : Natural := 0;
   begin
      for S of Sets loop
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
         for S of Sets loop
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

   --  How far the aligner errs on the arm's keyframes (Driver.Alignment.Keyframes.Scatter).
   function Match_Sigma (R : Arm_Evidence) return Real is (Driver.Alignment.Keyframes.Scatter (R.Chain));

   function Held_Still (M : Model; A : Arm_Id; Beat : Natural) return Boolean is
      E : constant Eye_Id'Base := Eye_Of (M, A);
      G : constant Group_Id := Arm_Group (M, A);
      --  Over the time the group's response takes to show (its measured push
      --  delay, and the beat itself): a joint on its way at a steady rate
      --  below the readings' noise per beat is not still, though no two beats
      --  tell (a reading creeping towards its command; item 21). Still at
      --  every beat of the span, and not moved over the span as a whole.
      Span : constant Positive := 1 + (if M.Groups (G).Delay_Known then M.Groups (G).Delay_Beats else 0);
   begin
      if E = 0 or else Beat < Span then
         return False;
      end if;
      --  The eye's picture by the one stop rule (Stillness.Eye_Settled), which
      --  holds for the latest beat.
      if not Stillness.Eye_Settled (M, E) then
         return False;
      end if;
      for B in Beat - Span + 1 .. Beat loop
         if not Channels.Has_Reading (M, G, B) or else not Channels.Has_Reading (M, G, B - 1)
           or else Channels.Moving (M, G, B)
         then
            return False;
         end if;
      end loop;
      declare
         Now   : constant Real_Array := Readings_Of (M, G, Beat);
         Since : constant Real_Array := Readings_Of (M, G, Beat - Span);
         D     : Real_Array (1 .. Now'Length);
      begin
         for C in D'Range loop
            D (C) := Now (Now'First + C - 1) - Since (Since'First + C - 1);
         end loop;
         return not Channels.Visible (M, G, D);
      end;
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
                        --  keyframe, some channel moved by at least the step the
                        --  matcher tells apart (Keyframe_Step: Z of the larger of the
                        --  cells' displacement noise and the aligner's own error, over
                        --  the pixels the eye's picture moves per unit of the channel),
                        --  the step the fit itself judges a keyframe's motion by. The
                        --  step the eye can see at all (Visible_Step), which the lock-in
                        --  tells over many beats and every cell, lies far below what one
                        --  pair of pictures resolves: on the fake body 7 microradians of
                        --  the wrist, a thousandth of a pixel, and a settling arm gave a
                        --  keyframe every other beat, 304 by beat 4224 against 41. Where
                        --  the matcher's step is not measured yet (no cell responds to the
                        --  channel), the step the eye can see stands in. None is taken
                        --  before either is measured. The second keyframe is the
                        --  reference's still twin, at its pose.
                        declare
                           Steps   : Real_Array (Now'Range) := [others => 0.0];
                           Seeable : Boolean := False;
                           At_Reference : Boolean := False;   --  at the reference's readings, to within the steps
                        begin
                           for C in Now'Range loop
                              declare
                                 Told   : constant Real := Keyframe_Step (M, Arm, C);
                                 Seen   : constant Estimate := Visible_Step (M, G, C);
                                 Jitter : constant Real := Channels.Noise (M, G, C);
                              begin
                                 if Told > 0.0 then
                                    --  And no nearer than the readings tell apart (Z of the noise of
                                    --  a difference of two readings, as Visible_Step has it): a
                                    --  matcher that errs by a thousandth of a pixel gives a step
                                    --  below the readings' own jitter, and an arm at rest gave a
                                    --  keyframe every thirty-seven beats at one pose (fake body Z9,
                                    --  arm 1: 295 of them, each aligned for nothing).
                                    Steps (C) :=
                                      (if Jitter < Real'Last
                                       then Real'Max (Told, Driver.Conventions.Z
                                                            * Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0) * Jitter)
                                       else Told);
                                    Seeable := True;
                                 elsif Known (Seen) then
                                    Steps (C) := Seen.Value;
                                    Seeable := True;
                                 end if;
                              end;
                           end loop;
                           for F in R.Frames.First_Index .. R.Frames.Last_Index loop
                              declare
                                 Apart : Boolean := False;
                              begin
                                 for C in Now'Range loop
                                    if Steps (C) > 0.0 and then abs (Now (C) - R.Frames (F).Readings (C - 1)) >= Steps (C) then
                                       Apart := True;
                                    end if;
                                 end loop;
                                 Fresh := Fresh and then Apart;
                                 if F = R.Frames.First_Index then
                                    At_Reference := not Apart;
                                 end if;
                              end;
                           end loop;
                           --  The reference's still twin: the first keyframe after it at its readings, whenever the
                           --  arm stands there again (two pictures of one pose: the matcher's own error, Match_Noise).
                           --  A frame at other readings is no twin, held still or not: the reference's next still beat
                           --  can come after a push (robot.sweep.twin: that frame was aligned as the same pose, and 0
                           --  of 40 points were found).
                           Fresh := Seeable
                             and then (Fresh
                                       or else (not R.Twin_Taken and then not R.Frames.Is_Empty
                                                and then Beat > R.Frames (1).Beat and then At_Reference));
                           if Fresh then
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
                              if At_Reference and then Natural (R.Frames.Length) > 1 then
                                 R.Twin_Taken := True;
                              end if;
                           end if;
                        end;
                        if Fresh then
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
     (if Index_Of (M, A) > 0 then Match_Sigma (M.Kinematics (Index_Of (M, A))) else 0.0);

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

   function Follow_Step (M : Model; A : Arm_Id; Channel : Positive) return Real is
      E : constant Eye_Id'Base := Eye_Of (M, A);
   begin
      if E = 0 then
         return 0.0;
      end if;
      declare
         Per_Unit : constant Real := Lockin.Shift (M, E, Arm_Group (M, A), Channel);
         Half     : constant Natural :=
           Driver.Alignment.Half_Patch (M.Eyes (E).Grid.Width, M.Eyes (E).Grid.Height);
      begin
         return (if Per_Unit > 0.0 then Real (Half) / Per_Unit else 0.0);
      end;
   end Follow_Step;

   function Twin_Answered (M : Model; A : Arm_Id) return Boolean is
   begin
      if Index_Of (M, A) = 0 then
         return False;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (Index_Of (M, A));
      begin
         return R.Twin_Taken;
      end;
   end Twin_Answered;

   procedure Begin_Anew (M : in out Model; A : Arm_Id) is
      K : constant Natural := Index_Of (M, A);
   begin
      if K > 0 then
         declare
            R : Arm_Evidence renames M.Kinematics (K);
         begin
            Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & A'Image & " begins anew at beat" & M.Beats'Image
                             & ", dropping" & R.Frames.Length'Image & " keyframes");
            M.Kinematics.Replace_Element (K, (Arm => R.Arm, Group => R.Group, Eye => R.Eye, others => <>));
         end;
      end if;
   end Begin_Anew;

   function Matched (M : Model; A : Arm_Id) return Natural is
     (if Index_Of (M, A) > 0 then Natural (M.Kinematics (Index_Of (M, A)).Matches.Length) else 0);

   function Pending (M : Model) return Natural is
      K : Natural := 0;
   begin
      for R of M.Kinematics loop
         K := K + Natural (R.World_Pending.Length) + Natural (R.Eye_Pending.Length);
      end loop;
      return K;
   end Pending;

   ---------------------------------------------------------------------------
   --  Aligning the keyframes

   package Keyframes renames Driver.Alignment.Keyframes;

   use type Driver.Alignment.Verdict;

   type Prediction_Access is access Keyframes.Prediction_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Prediction_Array, Prediction_Access);
   type Alignment_Access is access Keyframes.Answer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Answer_Array, Alignment_Access);
   type Keyframe_Point_Access is access Keyframes.Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Point_Array, Keyframe_Point_Access);
   type Agreement_Access is access Keyframes.Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Keyframes.Flag_Array, Agreement_Access);

   --  The lens of a fit, as Fit has it.
   function Fit_Lens (L : Lens_Fit) return Fit.Lens is
     ((Fx => L.Fx, Fy => L.Fy, Cx => L.Cx, Cy => L.Cy, K1 => L.K1, K2 => L.K2));

   function Axis (A : Positive) return Vec3 is
     (case A is when 1 => [1.0, 0.0, 0.0], when 2 => [0.0, 1.0, 0.0], when others => [0.0, 0.0, 1.0]);

   type Pixel_Derivative is array (1 .. 2) of Real;   --  of the pixel's u and v
   type Derivative_Array is array (1 .. 7) of Pixel_Derivative;
   --  By the eye's turn about each axis (1 to 3), its place along each (4 to 6) and the point's depth (7).

   --  The fitted arm's own prediction of where its reference's query points are in a keyframe at those readings.
   --  A query the fit gave a depth is a point in the reference eye's frame; the eye at the readings (the fit's
   --  joints) sees it where the lens puts it. The prediction's uncertainty is what the fit's own leaves the eye's
   --  turn and place at those readings (Pose_Covariance) and the point's depth, carried to the pixel by the
   --  derivative of the projection; the warp is the plane's homography, the table's for a track that lies on it
   --  and else the plane across the line of sight at the track's depth. A point without a depth is not
   --  predicted: its uncertainty is unbounded. Known is False when the arm is not fitted.
   procedure Offer_Of
     (M : Model; R : Arm_Evidence; Readings : Real_Array; Into : out Keyframes.Prediction_Array; Known : out Boolean)
   is
      use Driver.Numerics.Arrays;
      Queries   : constant Natural := Natural (R.Query_U.Length);
      Lens      : constant Fit.Lens := Fit_Lens (R.Result.Lens);
      Unbounded : constant Real := Real'Last / 4.0;
      Pose      : constant Rigid := Eye_In_Reference (M, R.Arm, Readings);
      Step      : constant Real := Ada.Numerics.Long_Elementary_Functions."**" (Real'Model_Epsilon, 1.0 / 3.0);
      Turn, Place : Mat3;

      --  Where the point X of the reference eye's frame lands when the eye's turn is moved by Omega (a rotation
      --  vector, in the reference frame) and its place by Shift.
      procedure Land (X, Omega, Shift : Vec3; Where : out Driver.Images.Pixel; Front : out Boolean) is
         Moved : constant Rigid :=
           (Rotation => Driver.Numerics.Exp (Omega) * Pose.Rotation, Translation => Pose.Translation + Shift);
      begin
         Fit.Project (Lens, Inverse (Moved) * X, Where.U, Where.V, Front);
      end Land;

      --  Where the plane A (the line of sight H meets it at depth 1 / (A * H)) carries the reference pixel.
      procedure Through_Plane (A : Vec3; U0, V0 : Real; Where : out Driver.Images.Pixel; Ok : out Boolean) is
         H     : constant Vec3 := Fit.Ray (Lens, U0, V0);
         Slope : constant Real := A * H;
      begin
         Where := (U => 0.0, V => 0.0);
         Ok := Slope > 0.0;
         if Ok then
            Land ((1.0 / Slope) * H, Zero3, Zero3, Where, Ok);
         end if;
      end Through_Plane;
   begin
      Pose_Covariance (M, R.Arm, Readings, Turn, Place);
      Known := R.Result.Fitted and then Natural (R.Result.Track_Known.Length) >= Queries
        and then Natural (R.Result.Tracks.Length) >= 3 * Queries;
      for K in Into'Range loop
         declare
            Q  : constant Natural := K - Into'First;
            U0 : constant Real := (if Q < Queries then R.Query_U (Q) else 0.0);
            V0 : constant Real := (if Q < Queries then R.Query_V (Q) else 0.0);
         begin
            Into (K) := (From => (U => U0, V => V0), To => (U => U0, V => V0),
                         Cov => (UU => Unbounded, UV => 0.0, VV => Unbounded), others => <>);
            if Known and then Q < Queries and then R.Result.Track_Known (Q)
              and then Q < Natural (R.Result.Track_Sigmas.Length)
              and then R.Result.Track_Sigmas (Q) < Unbounded and then Turn (1, 1) < Real'Last
              and then Place (1, 1) < Real'Last
            then
               declare
                  X     : constant Vec3 :=
                    [R.Result.Tracks (3 * Q), R.Result.Tracks (3 * Q + 1), R.Result.Tracks (3 * Q + 2)];
                  Here  : Driver.Images.Pixel;
                  Front : Boolean;
               begin
                  Land (X, Zero3, Zero3, Here, Front);
                  if Front then
                     declare
                        Scale : constant Real := abs X;
                        J     : Derivative_Array := [others => [0.0, 0.0]];
                        Plane : constant Vec3 :=
                          (if Q < Natural (R.Result.Table_On.Length) and then R.Result.Table_On (Q)
                              and then R.Result.Table_A * X > 0.0
                           then (1.0 / (R.Result.Table_A * X)) * R.Result.Table_A
                           else [0.0, 0.0, 1.0 / X (3)]);
                        P1, P2 : Driver.Images.Pixel;
                        Ok1, Ok2 : Boolean;
                        Variance : array (1 .. 3) of Real := [others => 0.0];   --  UU, UV, VV

                        function Pair (A, B : Positive) return Real is
                           Sum : Real := 0.0;
                        begin
                           for I in 1 .. 3 loop
                              for L in 1 .. 3 loop
                                 Sum := Sum + J (I) (A) * Turn (I, L) * J (L) (B)
                                   + J (3 + I) (A) * Place (I, L) * J (3 + L) (B);
                              end loop;
                           end loop;
                           return Sum + J (7) (A) * J (7) (B) * R.Result.Track_Sigmas (Q) ** 2;
                        end Pair;
                     begin
                        for A in 1 .. 3 loop
                           declare
                              Up, Down : Driver.Images.Pixel;
                              F1, F2   : Boolean;
                           begin
                              Land (X, Step * Axis (A), Zero3, Up, F1);
                              Land (X, -Step * Axis (A), Zero3, Down, F2);
                              J (A) := [(Up.U - Down.U) / (2.0 * Step), (Up.V - Down.V) / (2.0 * Step)];
                              Land (X, Zero3, (Step * Scale) * Axis (A), Up, F1);
                              Land (X, Zero3, (-Step * Scale) * Axis (A), Down, F2);
                              J (3 + A) := [(Up.U - Down.U) / (2.0 * Step * Scale),
                                            (Up.V - Down.V) / (2.0 * Step * Scale)];
                           end;
                        end loop;
                        declare
                           Up, Down : Driver.Images.Pixel;
                           F1, F2   : Boolean;
                        begin
                           Land ((1.0 + Step) * X, Zero3, Zero3, Up, F1);
                           Land ((1.0 - Step) * X, Zero3, Zero3, Down, F2);
                           J (7) := [(Up.U - Down.U) / (2.0 * Step), (Up.V - Down.V) / (2.0 * Step)];
                        end;
                        Variance := [Pair (1, 1), Pair (1, 2), Pair (2, 2)];
                        Into (K).To := Here;
                        Into (K).Cov := (UU => Variance (1), UV => Variance (2), VV => Variance (3));
                        Through_Plane (Plane, U0, V0, Here, Ok1);
                        Through_Plane (Plane, U0 + 1.0, V0, P1, Ok1);
                        Through_Plane (Plane, U0, V0 + 1.0, P2, Ok2);
                        if Ok1 and then Ok2 then
                           Into (K).Linear := (UU => P1.U - Here.U, UV => P2.U - Here.U,
                                               VU => P1.V - Here.V, VV => P2.V - Here.V);
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Offer_Of;

   --  The median standard deviation, over the points, that predictions state: the larger axis of each.
   function Stated_By (Given : Keyframes.Prediction_Array) return Real is
      Sigmas : Real_Access := new Real_Array (1 .. Given'Length);
   begin
      for K in Given'Range loop
         Sigmas (K - Given'First + 1) :=
           Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (Given (K).Cov.UU, Given (K).Cov.VV));
      end loop;
      return Result : constant Real := (if Given'Length = 0 then 0.0 else Driver.Stats.Median (Sigmas.all)) do
         Free (Sigmas);
      end return;
   end Stated_By;

   --  What a keyframe's answers say of where the reference's query points went: the answers found that agree with the
   --  keyframe's field of displacements (a wrong answer is as sure of itself as a right one, and is told from it by
   --  the others), but those whose uncertainty is more than Z times the keyframe's typical one (the median of those:
   --  a point that tells its place no better than that is not a sighting the fit can take as one among the rest).
   --  The keyframe is one to rest a fit on only when the typical answer of it came from a window no wider than a
   --  patch (Narrow): a wider one is searched where several places a patch cannot tell apart may lie in it, the
   --  answer rests on the prediction as much as on the pixels, and a keyframe whose answers do is a move the
   --  prediction had not followed. Its few answers from narrower windows are no sightings either: the fit takes the
   --  widest keyframe of a joint for the one that tells its axis best, and a keyframe of a handful of points is no
   --  such. The points refused for the prediction's uncertainty, those found that do not agree and those a wide
   --  window found are Wide: the prediction may have been what failed them, and a better one may find them.
   procedure Set_Of
     (F : Positive; R : Arm_Evidence; Given : Keyframes.Prediction_Array; Answers : Keyframes.Answer_Array;
      Agree : Keyframes.Flag_Array; Set : out Match_Set)
   is
      Counted : Natural := 0;
      Typical : Real := 0.0;
      function Sigma_Of (A : Driver.Alignment.Answer) return Real is
        (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real'Max (A.Cov.UU, A.Cov.VV)));
      function Agrees (I : Natural) return Boolean is (Agree (Agree'First + I - Answers'First));
      function Answered (I : Natural) return Boolean is (Answers (I).Verdict = Driver.Alignment.Found and then Agrees (I));
      Followed : Boolean := False;
      function Sighted (I : Natural) return Boolean is (Followed and then Answered (I) and then Answers (I).Narrow);
   begin
      declare
         Found, From_Narrow : Natural := 0;
      begin
         for I in Answers'Range loop
            if Answered (I) then
               Found := Found + 1;
               if Answers (I).Narrow then
                  From_Narrow := From_Narrow + 1;
               end if;
            end if;
         end loop;
         Followed := From_Narrow > Found - From_Narrow;   --  more of the answers came from narrow windows than from wider
      end;
      for I in Answers'Range loop
         if Sighted (I) then
            Counted := Counted + 1;
         end if;
      end loop;
      if Counted > 0 then
         declare
            Sigmas : Real_Access := new Real_Array (1 .. Counted);
            K      : Natural := 0;
         begin
            for I in Answers'Range loop
               if Sighted (I) then
                  K := K + 1;
                  Sigmas (K) := Sigma_Of (Answers (I));
               end if;
            end loop;
            Typical := Driver.Stats.Median (Sigmas.all);
            Free (Sigmas);
         end;
      end if;
      Set := (Frame => F, Eye => 0, Stated => Stated_By (Given), Followed => Followed, others => <>);
      for I in Answers'Range loop
         Set.To_U.Append (Answers (I).To.U);
         Set.To_V.Append (Answers (I).To.V);
         Set.Sigma.Append (if Answers (I).Verdict = Driver.Alignment.Found then Sigma_Of (Answers (I)) else 0.0);
         Set.Freedom.Append (if Answers (I).Verdict = Driver.Alignment.Found then Answers (I).Degrees_Of_Freedom else 0);
         --  There is no way back: an answer is judged by its own uncertainty, not by another answer.
         Set.Back_U.Append (R.Query_U (I - 1));
         Set.Back_V.Append (R.Query_V (I - 1));
         Set.Found.Append (Sighted (I) and then Sigma_Of (Answers (I)) <= Driver.Conventions.Z * Typical);
         if Keyframes.Prediction_Caused (Answers (I))
           or else (Answers (I).Verdict = Driver.Alignment.Found and then not Sighted (I))
         then
            Set.Wide := Set.Wide + 1;
         end if;
      end loop;
   end Set_Of;

   function Readings_Array (F : Keyframe) return Real_Array is
      Result : Real_Array (1 .. Natural (F.Readings.Length));
   begin
      for C in Result'Range loop
         Result (C) := F.Readings (C - 1);
      end loop;
      return Result;
   end Readings_Array;

   --  The arm's keyframes not yet aligned, in order.
   procedure Align_Arm (M : Model; R : in out Arm_Evidence) is
      N       : constant Natural := M.Groups (R.Group).Size;
      Queries : constant Natural := Natural (R.Query_U.Length);
      Noise   : Real_Array (1 .. N);
      Shift   : Real_Array (1 .. N);
      Start   : constant Duration := Driver.Clock.Seconds;
      First   : constant Natural := R.Aligned + 1;
      Found   : Natural := 0;
      Because : array (Driver.Alignment.Reason) of Natural := [others => 0];   --  why each point was or was not found

      --  The tally of the aligner's reasons, for the log: "; because: matched 5358, no_texture 1200, ...".
      function Reasons_Text return String is
         Text : Ada.Strings.Unbounded.Unbounded_String;
      begin
         for Why in Because'Range loop
            if Because (Why) > 0 then
               Ada.Strings.Unbounded.Append
                 (Text, (if Ada.Strings.Unbounded.Length (Text) = 0 then "; because: " else ", ")
                  & Ada.Characters.Handling.To_Lower (Why'Image) & Because (Why)'Image);
            end if;
         end loop;
         return Ada.Strings.Unbounded.To_String (Text);
      end Reasons_Text;
   begin
      for C in 1 .. N loop
         Noise (C) := Channels.Noise (M, R.Group, C);
         Shift (C) := Lockin.Shift (M, R.Eye, R.Group, C);
      end loop;
      if (for some X of Noise => X >= Real'Last) then
         return;
      end if;
      if R.Aligned = 0 then
         declare
            Points : Keyframe_Point_Access := new Keyframes.Point_Array (1 .. Queries);
         begin
            for I in Points'Range loop
               Points (I) := (U => R.Query_U (I - 1), V => R.Query_V (I - 1));
            end loop;
            Keyframes.Begin_Reference (R.Chain, R.Frames.First_Element.Image, Points.all);
            Free (Points);
         end;
         R.Aligned := 1;
      end if;
      while R.Aligned < Natural (R.Frames.Length) loop
         declare
            F       : constant Positive := R.Aligned + 1;
            Change  : Real_Array (1 .. N);
            Given   : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
            Offered : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
            Answers : Alignment_Access := new Keyframes.Answer_Array (1 .. Queries);
            Agree   : Agreement_Access := new Keyframes.Flag_Array (1 .. Queries);
            From    : Keyframes.Source;
            Known   : Boolean;
            Set     : Match_Set;
         begin
            for C in 1 .. N loop
               Change (C) := R.Frames (F).Readings (C - 1) - R.Frames.First_Element.Readings (C - 1);
            end loop;
            Offer_Of (M, R, Readings_Array (R.Frames (F)), Offered.all, Known);
            if Known then
               Keyframes.Predict (R.Chain, Change, Noise, Shift, Offered.all, Given.all, From);
            else
               Keyframes.Predict (R.Chain, Change, Noise, Shift, Keyframes.No_Offer, Given.all, From);
            end if;
            Keyframes.Add (R.Chain, R.Frames (F).Image, Change, Given.all, Answers.all, Agree.all);
            for A of Answers.all loop
               Because (A.Because) := Because (A.Because) + 1;
            end loop;
            Set_Of (F, R, Given.all, Answers.all, Agree.all, Set);
            declare
               Here : Natural := 0;
               Text : Ada.Strings.Unbounded.Unbounded_String;
            begin
               for Seen of Set.Found loop
                  if Seen then
                     Here := Here + 1;
                  end if;
               end loop;
               Found := Found + Here;
               --  Each keyframe's line, for the study of what the aligner finds at what motion: the change of every
               --  channel from the reference, how the prediction was made, and how many points were found.
               for C in 1 .. N loop
                  Ada.Strings.Unbounded.Append (Text, " " & Driver.Log.Image (Change (C), 5));
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe" & F'Image & " at beat"
                                & R.Frames (F).Beat'Image & ", change" & Ada.Strings.Unbounded.To_String (Text)
                                & ", predicted " & Ada.Characters.Handling.To_Lower (From'Image) & " stating "
                                & Driver.Log.Image (Stated_By (Given.all), 1) & " px:" & Here'Image
                                & " of" & Queries'Image & " points found");
            end;
            R.Matches.Append (Set);
            R.Aligned := F;
            --  A keyframe that refused nothing for its prediction needs its picture no more.
            if Set.Wide = 0 then
               R.Frames (F).Image := Driver.Images.No_Image;
            end if;
            Free (Given);
            Free (Offered);
            Free (Answers);
            Free (Agree);
         end;
      end loop;
      Driver.Log.Line
        (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " aligned keyframes" & First'Image & " to"
         & R.Aligned'Image & ":" & Found'Image & " of" & Natural'Image ((R.Aligned - First + 1) * Queries)
         & " points found, the aligner errs by " & Driver.Log.Image (Match_Sigma (R), 3) & " px, in "
         & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s" & Reasons_Text);
      --  The steps a keyframe of each channel needs, for the log: what the matcher tells apart against what the eye sees.
      declare
         Text : Ada.Strings.Unbounded.Unbounded_String;
      begin
         for C in 1 .. N loop
            declare
               Seen : constant Estimate := Visible_Step (M, R.Group, C);
            begin
               Ada.Strings.Unbounded.Append
                 (Text, " c" & C'Image & " " & Driver.Log.Image (Keyframe_Step (M, R.Arm, C), 6) & " (eye "
                  & (if Known (Seen) then Driver.Log.Image (Seen.Value, 6) else "?") & ")");
            end;
         end loop;
         Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe steps, matcher (eye):"
                          & Ada.Strings.Unbounded.To_String (Text));
      end;
   end Align_Arm;

   procedure Align_New (M : in out Model) is
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R : Arm_Evidence renames M.Kinematics (Index);
         begin
            if Current (M, R) and then Natural (R.Frames.Length) > R.Aligned and then not R.Query_U.Is_Empty
              and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index
            then
               Align_Arm (M, R);
            end if;
         end;
      end loop;
   end Align_New;

   --  Whether a prediction that states Sigma has a window narrower than the one a keyframe was aligned from, by enough to
   --  be worth aligning it again.
   function Narrowed (Was : Match_Set; Sigma : Real) return Boolean is
      Then_Radius : constant Real := Driver.Alignment.Window_Radius (Was.Stated);
      Now_Radius  : constant Real := Driver.Alignment.Window_Radius (Sigma);
   begin
      return (if Was.Followed then Now_Radius * Real (Driver.Alignment.Halving) <= Then_Radius
              else Then_Radius - Now_Radius >= 1.0);
   end Narrowed;

   function Realign (M : in out Model) return Boolean is
      Changed : Boolean := False;
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R       : Arm_Evidence renames M.Kinematics (Index);
            Queries : constant Natural := Natural (R.Query_U.Length);
            Done    : Natural := 0;
            Start   : constant Duration := Driver.Clock.Seconds;
         begin
            if Current (M, R) and then R.Result.Fitted and then R.Aligned >= 2 and then Queries > 0 then
               for S in R.Matches.First_Index .. R.Matches.Last_Index loop
                  if R.Matches (S).Wide > 0 and then Driver.Images.Width (R.Frames (R.Matches (S).Frame).Image) > 0 then
                     declare
                        F       : constant Positive := R.Matches (S).Frame;
                        Offered : Prediction_Access := new Keyframes.Prediction_Array (1 .. Queries);
                        Answers : Alignment_Access := new Keyframes.Answer_Array (1 .. Queries);
                        Agree   : Agreement_Access := new Keyframes.Flag_Array (1 .. Queries);
                        Known   : Boolean;
                     begin
                        Offer_Of (M, R, Readings_Array (R.Frames (F)), Offered.all, Known);
                        --  A keyframe is aligned again when the fitted arm gives it a window narrower by a pixel (the grid the
                        --  aligner's candidates lie on) than the one it was aligned from, and one a size of the pictures narrower if
                        --  what it answered was what a fit may rest on already (a keyframe the fit takes in, or not, by whether most
                        --  of its answers came from a window no wider than a patch: a better fit may narrow the window of the
                        --  next one to that). The windows narrow by a pixel at the least each time, so the repeating ends.
                        if Known and then Narrowed (R.Matches (S), Stated_By (Offered.all)) then
                           Keyframes.Realign (R.Chain, R.Frames (F).Image, Offered.all, Answers.all, Agree.all);
                           declare
                              Set : Match_Set;
                           begin
                              Set_Of (F, R, Offered.all, Answers.all, Agree.all, Set);
                              R.Matches.Replace_Element (S, Set);
                              if Set.Wide = 0 then
                                 R.Frames (F).Image := Driver.Images.No_Image;
                              end if;
                           end;
                           R.Revision := R.Revision + 1;
                           Done := Done + 1;
                           Changed := True;
                        end if;
                        Free (Offered);
                        Free (Answers);
                        Free (Agree);
                     end;
                  end if;
               end loop;
               if Done > 0 then
                  Driver.Log.Line
                    (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " aligned" & Done'Image
                     & " keyframes again from the fitted arm, in "
                     & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s");
               end if;
            end if;
         end;
      end loop;
      return Changed;
   end Realign;

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

   --  Every arm but the first placed in the world, by its reference view of
   --  the first arm's tracked points (In_World).
   procedure Place (M : in out Model);

   --  Every eye fixed in the world, from the first arm's points and the eye's answers to where they are
   --  (Fixed).
   procedure Fit_Fixed_Eyes (M : in out Model);

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

   --  The table in the arm's frame with its whole uncertainty: its points'
   --  scatter about it (Plane's, the sandwich of Refit_Plane) and the fit's
   --  terms, carried with their covariance through how they move it: every
   --  term through every depth together (Fit.Plane_Response), and each lens
   --  term through the lines of sight as well, the depths held (the plane
   --  refitted on the same points with the term moved by its standard
   --  deviation either way). Unknown without the fit's covariance.
   procedure Table_Estimate
     (U, V       : Real_Vectors.Vector;
      F          : Arm_Fit;
      L          : Fit.Lens;
      Sights     : Fit.Sight_Point_Array;
      Plane      : Fit.Sight_Plane;
      Gains      : Fit.Real_Lists.Vector;
      Covariance : Fit.Real_Lists.Vector;
      Estimate   : out Driver.Geometry.Plane_Estimate;
      Response   : out Real_Vectors.Vector)
   is
      use Driver.Numerics.Arrays;
      Q     : constant Natural := Natural (U.Length);
      Terms : constant Natural := Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                             (Real (Natural (Covariance.Length))));

      function Vc (P, C : Positive) return Real is (Covariance (Covariance.First_Index + (P - 1) * Terms + C - 1));
   begin
      if not Plane.Found or else Terms < Fit.Lens_Terms or else Terms * Terms /= Natural (Covariance.Length) then
         Estimate := (others => <>);
         Response.Clear;
         return;
      end if;
      declare
         On : Fit_Flag_Access := new Fit.Flag_Array (1 .. Q);
         S  : Sight_Access := new Fit.Sight_Point_Array (1 .. Q);
      begin
         Table_Of (F, On.all);
         declare
            T     : Real_Matrix := Fit.Plane_Response (Sights, On.all, Gains, Terms);
            Cv    : Real_Matrix (1 .. Terms, 1 .. Terms);
            Total : Mat3;
         begin
            for K in 1 .. Fit.Lens_Terms loop
               if Vc (K, K) > 0.0 then
                  declare
                     Sigma_K : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (Vc (K, K));
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
                           Ends (E) := P.A;
                        end;
                     end loop;
                     if Ok then
                        for D in 1 .. 3 loop
                           T (T'First (1) + D - 1, T'First (2) + K - 1) :=
                             T (T'First (1) + D - 1, T'First (2) + K - 1) + (Ends (2) (D) - Ends (1) (D)) / (2.0 * Sigma_K);
                        end loop;
                     end if;
                  end;
               end if;
            end loop;
            for P in 1 .. Terms loop
               for C in 1 .. Terms loop
                  Cv (P, C) := Vc (P, C);
               end loop;
            end loop;
            Total := Plane.Covariance + T * Cv * Transpose (T);
            Response.Clear;
            for A in 1 .. 3 loop
               for K in 1 .. Terms loop
                  Response.Append (T (T'First (1) + A - 1, T'First (2) + K - 1));
               end loop;
            end loop;
            Free (On);
            Free (S);
            Estimate := Fit.Plane_Estimate_Of (Plane, Total);
         end;
      end;
   end Table_Estimate;

   procedure Refit (M : in out Model) is
      Start : constant Duration := Driver.Clock.Seconds;
      Fitted, Placed : Duration;

      --  Each arm's fit reads the model and writes nothing but its own evidence's result: the arms are fitted
      --  at once, and the world is placed from them afterwards.
      procedure Fit_Arm (Index : Positive) is
         R : Arm_Evidence renames M.Kinematics (Index);
      begin
         if not Current (M, R) then
            R.Result := (others => <>);
         elsif not R.Matches.Is_Empty and then Natural (R.Matches.Length) + R.Revision /= R.Result.Matches
           and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index
         then
            declare
               N       : constant Natural := M.Groups (R.Group).Size;
               Frames  : constant Natural := Natural (R.Frames.Length);
               Queries : constant Natural := Natural (R.Query_U.Length);
               Changes : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
               Visible : Real_Array (1 .. N);
               Sigma   : constant Real := Match_Sigma (R);
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
                     Chain  : Fit.Chain_Order (1 .. N);
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
                                           U => S.To_U (I), V => S.To_V (I),
                                           Own => (if I < Natural (S.Sigma.Length) then S.Sigma (I) else 0.0),
                                           Own_Freedom => (if I < Natural (S.Freedom.Length) then S.Freedom (I) else 0));
                           end if;
                        end loop;
                     end loop;
                     --  The unit of length is the first fit's (Fit's Unit_Frames): a
                     --  keyframe taken after it refines every term, and moves none of
                     --  the lengths the world was measured in.
                     Fit.Fit (Changes.all, Visible, Seen.all, M.Eyes (R.Eye).Grid.Width, M.Eyes (R.Eye).Grid.Height,
                              R.Result.Unit_Frames, Joints, Chain, Lens, Report, Who => "arm" & R.Arm'Image);
                     --  The table its eye sees, away from it towards the eye,
                     --  and its tracks' points: Up is the first arm's (the world
                     --  is that eye's reference frame), and the points place the
                     --  other arms (Place).
                     if Report.Fitted then
                        declare
                           Plane  : Fit.Sight_Plane;
                           Response : Real_Vectors.Vector;
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
                                 Result.Track_Known.Append (Fit.Depth_Known (D, S));
                                 Result.Track_Sigmas.Append (S);
                                 for X of H loop
                                    Result.Tracks.Append (D * X);
                                 end loop;
                                 Sights (I + 1) := (H => H, Depth => D, Sigma => S);
                              end;
                           end loop;
                           --  How every depth moves with the fit's terms together, which a fixed eye
                           --  placed by these points carries (Fit_Fixed_Eyes). The fit has a track to the
                           --  last query that has a sighting: the queries after it, that no keyframe
                           --  answered, have no depth (above) and none that moves with a term.
                           declare
                              Terms : constant Natural :=
                                Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt
                                           (Real (Natural (Report.Covariance.Length))));
                           begin
                              while Natural (Report.Depth_Gains.Length) < Queries * Terms loop
                                 Report.Depth_Gains.Append (0.0);
                              end loop;
                           end;
                           for X of Report.Depth_Gains loop
                              Result.Depth_Gains.Append (X);
                           end loop;
                           --  The table: the plane most of them lie on, with its
                           --  whole uncertainty (Table_In_Arm).
                           Fit.Dominant_Plane (Sights.all, Plane, On.all);
                           for B of On.all loop
                              Result.Table_On.Append (B);
                           end loop;
                           Result.Table_Scatter := Plane.Covariance;
                           Table_Estimate (R.Query_U, R.Query_V, Result, Lens, Sights.all, Plane,
                                           Report.Depth_Gains, Report.Covariance, Result.Table, Response);
                           Result.Table_Response := Response;
                           --  For the link, which holds the lens apart: its points'
                           --  scatter and what the fit moves every depth by together.
                           Plane.Covariance :=
                             Fit.Plane_Covariance (Sights.all, On.all, Plane, Report.Depth_Gains, Report.Covariance);
                           Result.Table_A := Plane.A;
                           Result.Table_Covariance := Plane.Covariance;
                           Free (Sights);
                           Free (On);
                        end;
                     end if;
                     Free (Seen);
                     Result.Fitted := Report.Fitted;
                     Result.Unit_Frames :=
                       (if R.Result.Unit_Frames > 0 then R.Result.Unit_Frames elsif Report.Fitted then Frames else 0);
                     Result.Matches := Natural (R.Matches.Length) + R.Revision;
                     Result.Used := Report.Used;
                     Result.Median_Px := Report.Median_Px;
                     Result.Sigma_Px := Report.Sigma_Px;
                     Result.Why := Report.Why;
                     Result.Reference := R.Frames (1).Readings;
                     for J of Joints loop
                        Result.Joints.Append (Joint_Fit'(W => J.W, P => J.P, C => J.C, Slide => J.Slide));
                     end loop;
                     for P of Chain loop
                        Result.Chain.Append (P);
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
                                & "; its joints from the base, by their readings: " & Chain_Image (Chain)
                                & (if Report.Errors.Measured
                                   then "; errors, px: a sighting's own "
                                        & Driver.Log.Image (Report.Errors.Alone, 3) & ", a point's in every keyframe "
                                        & Driver.Log.Image (Report.Errors.Persistent, 3) & " (half as alike at "
                                        & Driver.Log.Image (Report.Errors.Persistent_Half, 0)
                                        & " px apart), a keyframe's added for its points "
                                        & Driver.Log.Image (Report.Errors.Keyframe, 3) & "; the clip took "
                                        & Driver.Log.Image (Report.Errors.Clipped, 3)
                                   else "")
                           else "not fitted (stage" & Report.Stage'Image & "): "
                                & Ada.Strings.Unbounded.To_String (Report.Why))
                        & Stages_Image (Report));
                  end;
                  Free (Moving);
               end;
               Free (Changes);
            end;
         end if;
      end Fit_Arm;

      procedure Fit_Arms is new Driver.Parallel_For (Fit_Arm);
   begin
      Fit_Arms (M.Kinematics.First_Index, M.Kinematics.Last_Index);
      Fitted := Driver.Clock.Seconds;
      Place (M);
      Placed := Driver.Clock.Seconds;
      Fit_Fixed_Eyes (M);
      Driver.Log.Line (Driver.Log.Robot, "kinematics: the arms fitted in "
                       & Driver.Log.Image (Real (Fitted - Start), 1) & " s, placed in "
                       & Driver.Log.Image (Real (Placed - Fitted), 1) & " s, the fixed eyes in "
                       & Driver.Log.Image (Real (Driver.Clock.Seconds - Placed), 1) & " s");
   end Refit;

   procedure Place (M : in out Model) is
      W : constant Natural := Index_Of (M, 1);

      --  How well the arm's fit unit is known against its depths (Fit.Unit_Sigma):
      --  the unit's keyframes, the first fit's.
      function Unit_Sigma_Of (R : Arm_Evidence) return Real is
         N      : constant Natural := Natural (R.Result.Joints.Length);
         Frames : constant Natural := R.Result.Unit_Frames;
      begin
         if N = 0 or else Frames = 0 or else Natural (R.Result.Reference.Length) /= N
           or else Natural (R.Result.Chain.Length) /= N
         then
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
            return Result : constant Real := Fit.Unit_Sigma (Joints, Chain_Of (R.Result), Changes.all, Cov) do
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

      --  A way to link the second arm to the first: an eye that shows both
      --  arms move and has both arms' table points in its view. A fixed eye
      --  has each arm's reference matched into its view at that arm's
      --  reference beat; the arm's own eye has the first arm's reference
      --  matched into its own, and its own points where they are.
      type Link_Way is record
         Eye : Natural := 0;
         Own : Boolean := False;
      end record;

      --  Each query point's track on the arm's table, by number among the
      --  side's tracks; 0 for a point not on it.
      type Index_Array is array (Natural range <>) of Natural;
      type Index_Access is access Index_Array;
      procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);

      --  The arm's side of a link: its lens and the lens's covariance, its
      --  table's tracks (Of_Query maps its query points to them).
      procedure Side_Of (R : Arm_Evidence; S : out Fit.Chain_Side; Of_Query : Index_Access) is
         F     : constant Arm_Fit := R.Result;
         Terms : constant Natural :=
           Natural (Ada.Numerics.Long_Elementary_Functions.Sqrt (Real (Natural (F.Covariance.Length))));
      begin
         S.L := Fit_Lens (F.Lens);
         S.Plane_Covariance := F.Table_Covariance;
         S.Lens_Covariance.Clear;
         S.Tracks.Clear;
         S.Answers.Clear;
         if Terms >= Fit.Lens_Terms and then Terms * Terms = Natural (F.Covariance.Length) then
            for P in 1 .. Fit.Lens_Terms loop
               for Q in 1 .. Fit.Lens_Terms loop
                  S.Lens_Covariance.Append (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
               end loop;
            end loop;
         end if;
         for I in Of_Query'Range loop
            Of_Query (I) := 0;
            if I < Natural (F.Table_On.Length) and then F.Table_On (I)
              and then I < Natural (F.Track_Known.Length) and then F.Track_Known (I)
            then
               --  Each element read into a constant of its own first: a container's
               --  element reference inside an aggregate actual does not outlive
               --  the aggregate (AddressSanitizer: stack use after scope).
               declare
                  U0    : constant Real := R.Query_U (I);
                  V0    : constant Real := R.Query_V (I);
                  Depth : constant Real := F.Tracks (3 * I + 2);
                  Sigma : constant Real := (if I < Natural (F.Track_Sigmas.Length) then F.Track_Sigmas (I) else Real'Last);
                  Track : constant Fit.Table_Track := (U0 => U0, V0 => V0, Depth => Depth, Sigma => Sigma);
               begin
                  S.Tracks.Append (Track);
               end;
               Of_Query (I) := Natural (S.Tracks.Length);
            end if;
         end loop;
      end Side_Of;

      --  Where an eye answered the side's tracks.
      procedure Answers_Of (S : in out Fit.Chain_Side; Set : Match_Set; Of_Query : Index_Access) is
      begin
         for I in 0 .. Natural (Set.Found.Length) - 1 loop
            if I <= Of_Query'Last and then Of_Query (I) > 0 and then Set.Found (I) then
               declare
                  U      : constant Real := Set.To_U (I);
                  V      : constant Real := Set.To_V (I);
                  Answer : constant Fit.Eye_Answer := (Track => Of_Query (I), U => U, V => V);
               begin
                  S.Answers.Append (Answer);
               end;
            end if;
         end loop;
      end Answers_Of;

      Outputs : constant := 7;   --  turn, centre, log scale
      type Output_Covariance is array (1 .. Outputs, 1 .. Outputs) of Real;

      --  Placed through one way: the link, the placement and its covariance;
      --  Why when it cannot be. A fixed eye has each arm's reference matched
      --  into its view; the second arm's own eye has the first arm's points
      --  matched into it, and its own lens sees them (Fit.Plane_Chain).
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
         S1, S2 : Fit.Chain_Side;
         Map1   : Index_Access := new Index_Array (0 .. Natural (R1.Query_U.Length) - 1);
         Map2   : Index_Access := new Index_Array (0 .. Natural (R2.Query_U.Length) - 1);
      begin
         Placement := Driver.Numerics.Identity;
         Scale := 1.0;
         Covariance := [others => [others => 0.0]];
         Link := (others => <>);
         Placed := False;
         Why := Ada.Strings.Unbounded.Null_Unbounded_String;
         Side_Of (R1, S1, Map1);
         Side_Of (R2, S2, Map2);
         if Way.Own then
            Answers_Of (S1, R2.World_Matches.First_Element, Map1);
         else
            Answers_Of (S1, R1.Eye_Matches (Set_Into (R1, Way.Eye)), Map1);
            Answers_Of (S2, R2.Eye_Matches (Set_Into (R2, Way.Eye)), Map2);
         end if;
         Free (Map1);
         Free (Map2);
         if not (Driver.Geometry.Known (R1.Result.Table) and then Driver.Geometry.Known (R2.Result.Table)) then
            Why := Ada.Strings.Unbounded.To_Unbounded_String ("an arm's table is not found");
            return;
         end if;
         Fit.Plane_Chain (S1, S2, Way.Own, Link);
         if not Link.Found then
            Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("of their tables' points" & S1.Answers.Length'Image & " and" & S2.Answers.Length'Image
               & " are in its view, too few for a homography of each and both");
         elsif not Link.Consistent then
            Why := Ada.Strings.Unbounded.To_Unbounded_String
              ("their tables are not one plane in its view: one homography for both leaves"
               & " F =" & Link.F'Image & " over the two apart (its noise" & Link.Joint'Image & " against"
               & Link.Sigma'Image & " px, " & Link.Used'Image & " points)");
         else
            declare
               Cov : Fit.Real_Lists.Vector;
            begin
               Fit.Chain_Placement (S1, S2, Link, Placement, Scale, Cov);
               if Natural (Cov.Length) = Outputs * Outputs then
                  for P in 1 .. Outputs loop
                     for Q in 1 .. Outputs loop
                        Covariance (P, Q) := Cov (Cov.First_Index + (P - 1) * Outputs + Q - 1);
                     end loop;
                  end loop;
                  Placed := Scale > 0.0;
               else
                  Why := Ada.Strings.Unbounded.To_Unbounded_String ("the link's uncertainty is not determined");
               end if;
            end;
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
                                          & " of their points (noise " & Driver.Log.Image (Link.Joint, 3) & " against "
                                          & Driver.Log.Image (Link.Sigma, 3) & " apart, F " & Driver.Log.Image (Link.F, 3)
                                          & "); its scale "
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

   ---------------------------------------------------------------------------
   --  Eyes fixed in the world

   function Square_Root (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Sqrt;

   type Fixed_Points is access Fixed.Point_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fixed.Point_Array, Fixed_Points);
   type Fixed_Sightings is access Fixed.Sighting_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Fixed.Sighting_Array, Fixed_Sightings);

   --  The graph mounts the eye fixed in the world.
   function Is_Fixed (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Graph.Mounts.Last_Index and then M.Graph.Mounts (E).Kind = World_Fixed);

   --  The fixed eye E measured from the first arm's evidence W (an index of Kinematics; 0 when there is none): the
   --  points the arm's eye tracked, each with the uncertainty of its depth (apart from the others') and the share
   --  of it every depth has in common (the arm's fit moves them together), and where the eye answered them.
   procedure Fit_Fixed_Eye (M : in out Model; W : Natural; E : Eye_Id) is
      use Ada.Strings.Unbounded;
      use Driver.Numerics.Arrays;
      From_Matches : constant Natural := (if W > 0 then M.Kinematics (W).Result.Matches else 0);
      From_Sets    : constant Natural := (if W > 0 then Natural (M.Kinematics (W).Eye_Matches.Length) else 0);

      --  The eye is not placed, and why; what it was measured from, to measure it again when that changes.
      procedure Leave (Why : String; Say : Boolean := True; Offered : Natural := 0) is
      begin
         M.Fixed_Eyes (E) := (Why => To_Unbounded_String (Why), Offered => Offered, Judged => True,
                              From_Matches => From_Matches, From_Sets => From_Sets, others => <>);
         if Say then
            Driver.Log.Line (Driver.Log.Robot, "kinematics: eye" & E'Image & " is fixed in the world, and not placed: "
                             & Why);
         end if;
      end Leave;
   begin
      if M.Fixed_Eyes (E).Judged and then M.Fixed_Eyes (E).From_Matches = From_Matches
        and then M.Fixed_Eyes (E).From_Sets = From_Sets
      then
         return;
      end if;
      if W = 0 or else not M.Kinematics (W).Result.Fitted then
         Leave ("the first arm is not fitted", Say => False);
         return;
      end if;
      declare
         R      : Arm_Evidence renames M.Kinematics (W);
         F      : Arm_Fit renames R.Result;
         K      : constant Natural := Set_Into (R, Natural (E));
         Q      : constant Natural := Natural (R.Query_U.Length);
         T      : constant Natural := Natural (Square_Root (Real (Natural (F.Covariance.Length))));
         Width  : constant Natural := M.Eyes (E).Grid.Width;
         Height : constant Natural := M.Eyes (E).Grid.Height;
      begin
         if K = 0 then
            Leave ("the first arm's reference has no answers from the eye");
            return;
         end if;
         if T < Fit.Lens_Terms or else T * T /= Natural (F.Covariance.Length)
           or else Natural (F.Depth_Gains.Length) /= Q * T or else Natural (F.Tracks.Length) /= 3 * Q
           or else Natural (F.Table_Response.Length) /= 3 * T
         then
            Leave ("the first arm's fit has no covariance of its points' depths");
            return;
         end if;
         if Width = 0 or else Height = 0 then
            Leave ("the eye's picture has no size");
            return;
         end if;
         declare
            Set    : Match_Set renames R.Eye_Matches (K);
            Single : Match_Set_Vectors.Vector;
            Sigma  : Real;
            Kept   : Natural := 0;

            --  The answer comes back to its query, within the noise of all the round trips.
            function Returns (I : Natural) return Boolean is
              (I < Natural (Set.Found.Length) and then I < Natural (Set.Back_U.Length) and then Set.Found (I)
               and then (Sigma = 0.0
                         or else (not Driver.Uncertain.Significant (Set.Back_U (I) - R.Query_U (I), Sigma)
                                  and then not Driver.Uncertain.Significant (Set.Back_V (I) - R.Query_V (I), Sigma))));

            --  And the arm has the point's depth, to an uncertainty it can state.
            function Usable (I : Natural) return Boolean is
              (Returns (I) and then I < Natural (F.Track_Known.Length) and then F.Track_Known (I)
               and then I < Natural (F.Track_Sigmas.Length) and then F.Track_Sigmas (I) < Real'Last
               and then I < Natural (F.Table_On.Length)
               and then (not F.Table_On (I)
                         or else F.Table_A * Fit.Ray (Fit_Lens (F.Lens), R.Query_U (I), R.Query_V (I)) > 0.0));
         begin
            Single.Append (Set);
            Sigma := Round_Trip_Sigma (R, Single);
            for I in 0 .. Q - 1 loop
               if Usable (I) then
                  Kept := Kept + 1;
               end if;
            end loop;
            if Kept = 0 then
               Leave ("no answer of the eye returns to its query");
               return;
            end if;
            declare
               Points   : Fixed_Points := new Fixed.Point_Array (1 .. Kept);
               Seen     : Fixed_Sightings := new Fixed.Sighting_Array (1 .. Kept);
               On_Plane : Fit_Flag_Access := new Fit.Flag_Array (1 .. Kept);
               Common   : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. 3 * Kept, 1 .. T + 3);
               Cov      : Matrix_Access := new Driver.Numerics.Arrays.Real_Matrix (1 .. T + 3, 1 .. T + 3);
               Poses    : constant Fixed.Pose_Array (1 .. 1) := [1 => Driver.Numerics.Identity];
               Lens     : constant Fit.Lens := Fit_Lens (F.Lens);
               Sigmas   : Real_Array (1 .. Fit.Lens_Terms);
               N        : Natural := 0;
               Report   : Fixed.Fit_Report;
               Start_Lens : Fit.Lens;
               Start_Pose : Rigid;
               Found    : Boolean;
            begin
               Cov.all := [others => [others => 0.0]];
               for A in 1 .. T loop
                  for B in 1 .. T loop
                     Cov (A, B) := F.Covariance (F.Covariance.First_Index + (A - 1) * T + B - 1);
                  end loop;
               end loop;
               for A in 1 .. 3 loop
                  for B in 1 .. 3 loop
                     Cov (T + A, T + B) := F.Table_Scatter (A, B);
                  end loop;
               end loop;
               for Term in 1 .. Fit.Lens_Terms loop
                  Sigmas (Term) := Square_Root (Real'Max (0.0, Cov (Term, Term)));
               end loop;
               for I in 0 .. Q - 1 loop
                  if Usable (I) then
                     N := N + 1;
                     --  Each element read into a constant of its own first (a container's element reference inside
                     --  an aggregate does not outlive the aggregate). A point on the arm's table is where its line
                     --  of sight meets the table: its depth comes from the plane, which every point on it shares,
                     --  not from its own refinement, whose error along the plane's normal would pass for the
                     --  structure that fixes the eye's lens (a plane alone fixes none).
                     declare
                        On  : constant Boolean := F.Table_On (I);
                        U0  : constant Real := R.Query_U (I);
                        V0  : constant Real := R.Query_V (I);
                        H   : constant Vec3 := Fit.Ray (Lens, U0, V0);
                        Depth_On : constant Real := (if On then 1.0 / (F.Table_A * H) else 0.0);
                        X1  : constant Real := F.Tracks (3 * I);
                        X2  : constant Real := F.Tracks (3 * I + 1);
                        X3  : constant Real := F.Tracks (3 * I + 2);
                        Own_X : constant Vec3 := [X1, X2, X3];
                        X   : constant Vec3 := (if On then Depth_On * H else Own_X);
                        S   : constant Real := F.Track_Sigmas (I);
                        Own : constant Mat3 :=
                          (if On then [others => [others => 0.0]] else S ** 2 * Driver.Numerics.Outer (X, X));
                        Answer_U : constant Real := Set.To_U (I);
                        Answer_V : constant Real := Set.To_V (I);
                     begin
                        Points (N) := (Position => X, Own => Own, U0 => U0, V0 => V0);
                        Seen (N) := (Point => N, Pose => 1, U => Answer_U, V => Answer_V);
                        On_Plane (N) := On;
                        --  How the point moves with each of the arm's terms: its line of sight with the lens's
                        --  terms, and its depth, which for a point on the table is the plane's (the plane moves
                        --  with the terms, Table_Response) and otherwise its own, which the other depths follow
                        --  (Depth_Gains).
                        for Term in 1 .. T loop
                           declare
                              Turn : Vec3 := [0.0, 0.0, 0.0];   --  how the line of sight moves
                              Move : Vec3;
                           begin
                              if Term <= Fit.Lens_Terms and then Sigmas (Term) > 0.0 then
                                 Turn := (Fit.Ray (Moved (Lens, Term, Sigmas (Term)), U0, V0)
                                          - Fit.Ray (Moved (Lens, Term, -Sigmas (Term)), U0, V0))
                                   / (2.0 * Sigmas (Term));
                              end if;
                              if On then
                                 declare
                                    Plane_Moves : constant Vec3 :=
                                      [F.Table_Response (Term - 1), F.Table_Response (T + Term - 1),
                                       F.Table_Response (2 * T + Term - 1)];
                                    Depth_Moves : constant Real :=
                                      -Depth_On ** 2 * (Plane_Moves * H + F.Table_A * Turn);
                                 begin
                                    Move := Depth_Moves * H + Depth_On * Turn;
                                 end;
                              else
                                 Move := X3 * Turn + F.Depth_Gains (I * T + Term - 1) * X;
                              end if;
                              for A in 1 .. 3 loop
                                 Common (3 * (N - 1) + A, Term) := Move (A);
                              end loop;
                           end;
                        end loop;
                        --  And with the plane's own error (its points' scatter about it), which every point on it
                        --  shares.
                        for B in 1 .. 3 loop
                           for A in 1 .. 3 loop
                              Common (3 * (N - 1) + A, T + B) := (if On then -Depth_On ** 2 * H (B) * H (A) else 0.0);
                           end loop;
                        end loop;
                     end;
                  end if;
               end loop;
               Fixed.Start (Points.all, Poses, Seen.all, On_Plane.all, F.Table_A, Start_Lens, Start_Pose, Found);
               if not Found then
                  --  A plane alone leaves the lens undetermined: the points off the table are what fix it.
                  declare
                     Off_Table : Natural := 0;
                  begin
                     for B of On_Plane.all loop
                        if not B then
                           Off_Table := Off_Table + 1;
                        end if;
                     end loop;
                     Leave ("the first arm's points give the eye no start (" & Kept'Image & " answers return,"
                            & Off_Table'Image & " off the first arm's table)", Offered => Kept);
                  end;
               else
                  Fixed.Fit_Eye (Points.all, Poses, Seen.all, Common.all, Cov.all, Width, Height,
                                 Start_Lens, Start_Pose, Report);
                  declare
                     Result : Fixed_Fit :=
                       (Known        => Report.Determined,
                        Arm          => R.Arm,
                        Lens         => (Fx => Report.L.Fx, Fy => Report.L.Fy, Cx => Report.L.Cx, Cy => Report.L.Cy,
                                         K1 => Report.L.K1, K2 => Report.L.K2),
                        Pose         => Report.Pose,
                        Used         => Report.Used,
                        Offered      => Report.Offered,
                        Sigma_Px     => Report.Sigma_Px,
                        Distorted    => Report.Distorted,
                        Why          => Report.Why,
                        Judged       => True,
                        From_Matches => From_Matches,
                        From_Sets    => From_Sets,
                        others       => <>);
                     Turn : constant Vec3 := Driver.Numerics.Log (Report.Pose.Rotation);

                     --  The standard deviation of term K of the covariance (0 when there is none).
                     function Sd (K : Positive) return Real is
                       (if Natural (Report.Covariance.Length) = Fixed.Terms ** 2
                        then Square_Root (Real'Max (0.0, Report.Covariance (Report.Covariance.First_Index
                                                                              + (K - 1) * Fixed.Terms + K - 1)))
                        else 0.0);

                     --  A value with its standard deviation, to that many places.
                     function Pm (X, S : Real; Places : Natural) return String is
                       (Driver.Log.Image (X, Places) & " +-" & Driver.Log.Image (S, Places));

                     Lens_Text : constant String :=
                       "focal " & Pm (Report.L.Fx, Report.L.Fx * Sd (1), 2) & " x "
                       & Pm (Report.L.Fy, Report.L.Fy * Sd (2), 2) & " px, centre " & Pm (Report.L.Cx, Sd (3), 2)
                       & ", " & Pm (Report.L.Cy, Sd (4), 2)
                       & (if Report.Distorted
                          then ", distortion " & Pm (Report.L.K1, Sd (5), 4) & ", " & Pm (Report.L.K2, Sd (6), 4)
                          else ", no distortion");
                     --  The camera's rotation as a rotation vector, its terms' sigmas about the camera's own axes.
                     Pose_Text : constant String :=
                       "in the world at " & Pm (Report.Pose.Translation (1), Sd (10), 3) & ", "
                       & Pm (Report.Pose.Translation (2), Sd (11), 3) & ", "
                       & Pm (Report.Pose.Translation (3), Sd (12), 3) & ", turned "
                       & Pm (Turn (1), Sd (7), 4) & ", " & Pm (Turn (2), Sd (8), 4) & ", " & Pm (Turn (3), Sd (9), 4);
                  begin
                     for X of Report.Covariance loop
                        Result.Covariance.Append (X);
                     end loop;
                     M.Fixed_Eyes (E) := Result;
                     Driver.Log.Line
                       (Driver.Log.Robot, "kinematics: eye" & E'Image & " fixed in the world: "
                        & (if Report.Determined then "placed" else "not placed") & " from" & Kept'Image
                        & " answers of" & Q'Image & " points," & Report.Used'Image & " fit, noise "
                        & Driver.Log.Image (Report.Sigma_Px, 3) & " px"
                        & (if Report.Linear_To < Real'Last
                           then ", its cost quadratic over " & Driver.Log.Image (Report.Linear_To, 1) & " of "
                                & Driver.Log.Image (Driver.Conventions.Z, 0) & " sigmas"
                           else "")
                        & "; " & Lens_Text & "; " & Pose_Text
                        & (if Report.Determined then "" else ": " & To_String (Report.Why)));
                  end;
               end if;
               Free (Points);
               Free (Seen);
               Free (On_Plane);
               Free (Common);
               Free (Cov);
            end;
         end;
      end;
   end Fit_Fixed_Eye;

   procedure Fit_Fixed_Eyes (M : in out Model) is
      W : constant Natural := Index_Of (M, 1);
   begin
      while M.Fixed_Eyes.Last_Index < M.Eyes.Last_Index loop
         M.Fixed_Eyes.Append (Fixed_Fit'(others => <>));
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         if Is_Fixed (M, E) then
            Fit_Fixed_Eye (M, W, E);
         else
            M.Fixed_Eyes (E) := (others => <>);
         end if;
      end loop;
   end Fit_Fixed_Eyes;

   function Fixed_Known (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Fixed_Eyes.Last_Index and then Is_Fixed (M, E) and then M.Fixed_Eyes (E).Known);

   function Fixed_Why (M : Model; E : Eye_Id) return String is
     (if not Is_Fixed (M, E) or else Fixed_Known (M, E) then ""
      elsif E > M.Fixed_Eyes.Last_Index or else not M.Fixed_Eyes (E).Judged then "not measured yet"
      else Ada.Strings.Unbounded.To_String (M.Fixed_Eyes (E).Why));

   --  The fixed eye's lens.
   function Fixed_Lens (M : Model; E : Eye_Id) return Fit.Lens is
     (if E <= M.Fixed_Eyes.Last_Index then Fit_Lens (M.Fixed_Eyes (E).Lens) else (others => <>));

   function Fixed_Pose (M : Model; E : Eye_Id) return Pose_Estimate is
      Terms : constant Natural := Fixed.Terms;
   begin
      if not Fixed_Known (M, E) or else Natural (M.Fixed_Eyes (E).Covariance.Length) /= Terms * Terms then
         return (others => <>);
      end if;
      declare
         use Driver.Numerics.Arrays;
         F : Fixed_Fit renames M.Fixed_Eyes (E);
         function C (P, Q : Positive) return Real is
           (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
         Turn, Place : Mat3;
      begin
         --  The turn is about the camera's own axes: in the world it is the pose's rotation of it.
         for P in 1 .. 3 loop
            for Q in 1 .. 3 loop
               Turn (P, Q) := C (Fit.Lens_Terms + P, Fit.Lens_Terms + Q);
               Place (P, Q) := C (Fit.Lens_Terms + 3 + P, Fit.Lens_Terms + 3 + Q);
            end loop;
         end loop;
         return (Pose                => F.Pose,
                 Position_Covariance => Place,
                 Rotation_Covariance => F.Pose.Rotation * Turn * Transpose (F.Pose.Rotation));
      end;
   end Fixed_Pose;

   function Fixed_Ray_In_Eye (M : Model; E : Eye_Id; U, V : Real) return Vec3 is
     (Unit (Fit.Ray (Fixed_Lens (M, E), U, V)));

   procedure Fixed_Project_In_Eye (M : Model; E : Eye_Id; P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      Fit.Project (Fixed_Lens (M, E), P, U, V, In_Front);
   end Fixed_Project_In_Eye;

   function Fixed_Angle_Sigma (M : Model; E : Eye_Id) return Real is
      L : constant Fit.Lens := Fixed_Lens (M, E);
   begin
      return (if Fixed_Known (M, E) and then L.Fx > 0.0 and then L.Fy > 0.0
              then M.Fixed_Eyes (E).Sigma_Px / Real'Min (L.Fx, L.Fy) else Real'Last);
   end Fixed_Angle_Sigma;

   function Fixed_Line_Sigma (M : Model; E : Eye_Id; U, V : Real; In_World : Boolean) return Real is
      use Driver.Numerics.Arrays;
      Terms : constant Natural := Fixed.Terms;
   begin
      if not Fixed_Known (M, E) or else Natural (M.Fixed_Eyes (E).Covariance.Length) /= Terms * Terms then
         return Real'Last;
      end if;
      declare
         F     : Fixed_Fit renames M.Fixed_Eyes (E);
         L     : constant Fit.Lens := Fixed_Lens (M, E);
         Free_Terms : constant Natural := Fit.Lens_Terms + (if In_World then 3 else 0);
         Turned : constant Mat3 := (if In_World then F.Pose.Rotation else Driver.Numerics.Identity3);
         D0     : constant Vec3 := Unit (Fit.Ray (L, U, V));
         --  How the direction (in the world, or in the eye) moves with each term: columns, lens terms then turn.
         J      : array (1 .. 3, 1 .. Free_Terms) of Real := [others => [others => 0.0]];
         Spin   : constant Mat3 := -(Turned * Driver.Numerics.Skew (D0));
         Var    : Real := 0.0;
         function C (P, Q : Positive) return Real is
           (F.Covariance (F.Covariance.First_Index + (P - 1) * Terms + Q - 1));
      begin
         for Term in 1 .. Fit.Lens_Terms loop
            declare
               S : constant Real := Square_Root (Real'Max (0.0, C (Term, Term)));
            begin
               if S > 0.0 then
                  declare
                     Change : constant Vec3 :=
                       (Turned * (Unit (Fit.Ray (Moved (L, Term, S), U, V))
                                  - Unit (Fit.Ray (Moved (L, Term, -S), U, V)))) / (2.0 * S);
                  begin
                     for A in 1 .. 3 loop
                        J (A, Term) := Change (A);
                     end loop;
                  end;
               end if;
            end;
         end loop;
         if In_World then
            for A in 1 .. 3 loop
               for B in 1 .. 3 loop
                  J (A, Fit.Lens_Terms + B) := Spin (A, B);
               end loop;
            end loop;
         end if;
         --  Half the trace of the direction's covariance: the variance per axis across the line.
         for A in 1 .. 3 loop
            for P in 1 .. Free_Terms loop
               for Q in 1 .. Free_Terms loop
                  Var := Var + J (A, P) * C (P, Q) * J (A, Q);
               end loop;
            end loop;
         end loop;
         return Square_Root (Fixed_Angle_Sigma (M, E) ** 2 + Var / 2.0);
      end;
   end Fixed_Line_Sigma;

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

   function Scale_In_World (M : Model; A : Arm_Id) return Estimate is
      K : constant Natural := Index_Of (M, A);
   begin
      if K = 0 or else not M.Kinematics (K).Result.Fitted or else not M.Kinematics (K).Result.Placed then
         return Driver.Uncertain.Unknown;
      end if;
      return (Value              => M.Kinematics (K).Result.Scale,
              Sigma              => M.Kinematics (K).Result.Scale_Sigma,
              Degrees_Of_Freedom => 0);
   end Scale_In_World;

   procedure Placement_Covariance (M : Model; A : Arm_Id; Turn, Centre : out Mat3) is
      K : constant Natural := Index_Of (M, A);
   begin
      Turn := [others => [others => 0.0]];
      Centre := [others => [others => 0.0]];
      for I in 1 .. 3 loop
         Turn (I, I) := Real'Last;
         Centre (I, I) := Real'Last;
      end loop;
      if K = 0 or else not M.Kinematics (K).Result.Fitted or else not M.Kinematics (K).Result.Placed
        or else Natural (M.Kinematics (K).Result.Placement_Covariance.Length) < 36
      then
         return;
      end if;
      declare
         C : Real_Vectors.Vector renames M.Kinematics (K).Result.Placement_Covariance;
      begin
         --  6 x 6, row by row: the turn first, then the centre.
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Turn (I, J) := C (C.First_Index + 6 * (I - 1) + (J - 1));
               Centre (I, J) := C (C.First_Index + 6 * (I + 2) + (J + 2));
            end loop;
         end loop;
      end;
   end Placement_Covariance;

   function World_Table (M : Model) return Driver.Geometry.Plane_Estimate is
   begin
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         if M.Kinematics (K).Result.Fitted and then M.Kinematics (K).Result.Placed
           and then M.Kinematics (K).Result.Placed_Through = 0
         then
            return M.Kinematics (K).Result.Table;
         end if;
      end loop;
      return (others => <>);
   end World_Table;

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
           and then Natural (R.Result.Chain.Length) = Readings'Length
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
               return Fit.Eye_At (Joints, Chain_Of (R.Result), Change);
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
        and then Natural (R.Chain.Length) = Readings'Length and then Natural (R.Reference.Length) = Readings'Length
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
            Fit.Pose_Covariance (Joints, Chain_Of (R), Change, Cov, Turn, Place);
         end;
      end if;
   end Pose_Covariance;

   function Framed (M : Model; A : Arm_Id; From, To : Natural) return Boolean is
      K : constant Natural := Index_Of (M, A);
   begin
      return K > 0 and then (for some F of M.Kinematics (K).Frames => F.Beat in From .. To);
   end Framed;

   function Followed (M : Model; A : Arm_Id; From, To : Natural) return Boolean is
      K : constant Natural := Index_Of (M, A);
   begin
      if K = 0 then
         return False;
      end if;
      declare
         R : Arm_Evidence renames M.Kinematics (K);
      begin
         for F in reverse 2 .. Natural (R.Frames.Length) loop
            if R.Frames (F).Beat in From .. To then
               for S of R.Matches loop
                  if S.Frame = F and then S.Eye = 0 then
                     return S.Followed;
                  end if;
               end loop;
               return False;
            end if;
         end loop;
         return False;
      end;
   end Followed;

   function Fitted (M : Model; A : Arm_Id) return Boolean is
     (Index_Of (M, A) > 0 and then M.Kinematics (Index_Of (M, A)).Result.Fitted);

   function Table (M : Model; A : Arm_Id) return Driver.Geometry.Plane_Estimate is
     (if Fitted (M, A) then M.Kinematics (Index_Of (M, A)).Result.Table else (others => <>));

   function Standing_Near (M : Model; A : Arm_Id; Point : Vec3; Within : Real) return Natural is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      K     : constant Natural := Index_Of (M, A);
      Count : Natural := 0;
   begin
      if not Fitted (M, A) or else not Driver.Geometry.Known (M.Kinematics (K).Result.Table) then
         return 0;
      end if;
      declare
         F : Arm_Fit renames M.Kinematics (K).Result;
         P : Driver.Geometry.Plane_Estimate renames F.Table;
      begin
         for T in 0 .. Natural (F.Track_Known.Length) - 1 loop
            if F.Track_Known (T) and then 3 * T + 2 < Natural (F.Tracks.Length)
              and then not (T < Natural (F.Table_On.Length) and then F.Table_On (T))
              and then T < Natural (F.Track_Sigmas.Length) and then F.Track_Sigmas (T) < Real'Last
            then
               declare
                  X : constant Vec3 := [F.Tracks (3 * T), F.Tracks (3 * T + 1), F.Tracks (3 * T + 2)];
                  H : constant Real := Driver.Geometry.Height (P, X);
                  --  The track's depth is uncertain by its logarithm's sigma: along its line of sight (the
                  --  reference eye is the origin), its length times that, of which the part along the normal
                  --  is the height's; with the plane's own there.
                  Length  : constant Real := abs X;
                  Along   : constant Real := Length * F.Track_Sigmas (T);
                  Facing  : constant Real := (if Length > 0.0 then Real'(Unit (X) * P.Normal) else 0.0);
                  Sigma   : constant Real :=
                    (if Length > 0.0
                     then Sqrt ((Along * abs Facing) ** 2 + Driver.Geometry.Height_Sigma (P, X) ** 2)
                     else Real'Last);
                  Off     : constant Vec3 := X - Point;
                  Across  : constant Real := Real'(Off * P.Normal);
                  Lateral : constant Real := abs (Off - Across * P.Normal);
               begin
                  if Sigma < Real'Last and then H > 0.0 and then Driver.Uncertain.Significant (H, Sigma)
                    and then Lateral <= Within
                  then
                     Count := Count + 1;
                  end if;
               end;
            end if;
         end loop;
      end;
      return Count;
   end Standing_Near;

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
                     --  No further than a float goes (as Fixed.Refine): past it the damping is infinite, the damped matrix no
                     --  number, never positive definite, and nothing would end the loop.
                     exit when Lowered or else not Moves or else Lambda >= Real'Last / 2.0;
                     Lambda := 2.0 * Lambda;
                  end;
               end loop;
            end;
            exit when not Improved;
         end;
      end loop;
      Q := Nearest_Readings (M, A, Start, X);
      Position_Off := Sqrt (R0 (1) ** 2 + R0 (2) ** 2 + R0 (3) ** 2);
      Turn_Off := (if Position_Only then 0.0 else Sqrt (R0 (4) ** 2 + R0 (5) ** 2 + R0 (6) ** 2));
   end Solve_Pose;

   function Conditioning (M : Model; A : Arm_Id; Q : Real_Array) return Real is
      use Driver.Numerics.Arrays;
      use Ada.Numerics.Long_Elementary_Functions;
      N : constant Natural := Q'Length;
   begin
      if N = 0 or else Index_Of (M, A) = 0 or else not M.Kinematics (Index_Of (M, A)).Result.Fitted then
         return 0.0;
      end if;
      declare
         T0 : constant Rigid := Eye_In_Reference (M, A, Q);
         J  : Real_Matrix (1 .. 6, 1 .. N);
      begin
         for K in 1 .. N loop
            declare
               Xp : Real_Array := Q;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Q (Q'First + K - 1));
               Tp : Rigid;
               D  : Vec3;
               W  : Vec3;
            begin
               Xp (Xp'First + K - 1) := Xp (Xp'First + K - 1) + H;
               Tp := Eye_In_Reference (M, A, Xp);
               D := Tp.Translation - T0.Translation;
               W := Driver.Numerics.Log (Transpose (T0.Rotation) * Tp.Rotation);
               for I in 1 .. 3 loop
                  J (I, K) := D (I) / H;
                  J (I + 3, K) := W (I) / H;
               end loop;
            end;
         end loop;
         declare
            Values : constant Real_Vector := Eigenvalues (Transpose (J) * J);
            Least  : Real := Real'Last;
            Most   : Real := 0.0;
         begin
            for V of Values loop
               Least := Real'Min (Least, V);
               Most := Real'Max (Most, V);
            end loop;
            return (if Most > 0.0 then Sqrt (Real'Max (0.0, Least) / Most) else 0.0);
         end;
      end;
   end Conditioning;

   function Nearest_Readings (M : Model; A : Arm_Id; Near, Readings : Real_Array) return Real_Array is
      Result : Real_Array (Readings'Range) := Readings;
      I      : constant Natural := Index_Of (M, A);
   begin
      if I > 0 and then Natural (M.Kinematics (I).Result.Joints.Length) = Readings'Length then
         for J in 1 .. Readings'Length loop
            declare
               F : constant Joint_Fit := M.Kinematics (I).Result.Joints (J);
            begin
               if not F.Slide and then F.C /= 0.0 then
                  declare
                     Period : constant Real := 2.0 * Ada.Numerics.Pi / abs F.C;
                     Here   : constant Real := Readings (Readings'First + J - 1);
                     Turns  : constant Real := Real'Rounding ((Here - Near (Near'First + J - 1)) / Period);
                  begin
                     Result (Result'First + J - 1) := Here - Turns * Period;
                  end;
               end if;
            end;
         end loop;
      end if;
      return Result;
   end Nearest_Readings;

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
