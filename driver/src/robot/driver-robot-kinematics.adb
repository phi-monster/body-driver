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

package body Driver.Robot.Kinematics is

   --  Samples too large for a stack live on the heap.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

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

   procedure Collect (R : in out Arm_Evidence) is
      K : Natural := R.Pending.First_Index;
   begin
      while K <= R.Pending.Last_Index loop
         if Driver.Services.Ready (R.Pending (K).Ticket) then
            declare
               P      : constant Pending_Match := R.Pending (K);
               Reply  : constant Driver.Services.Reply := Driver.Services.Collect (P.Ticket);
               Result : Driver.Instrument.Answer_Array (1 .. Natural (R.Query_U.Length));
               Ok     : Boolean;
               Why    : Ada.Strings.Unbounded.Unbounded_String;
               Set    : Match_Set;
            begin
               Driver.Instrument.Read_Match (Reply, True, Result, Ok, Why);
               if Ok then
                  Set.Frame := P.Frame;
                  for A of Result loop
                     Set.To_U.Append (A.To.U);
                     Set.To_V.Append (A.To.V);
                     Set.Back_U.Append (A.Back.U);
                     Set.Back_V.Append (A.Back.V);
                     Set.Found.Append (A.Found);
                  end loop;
                  R.Matches.Append (Set);
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
               R.Pending.Delete (K);
            end;
         else
            K := K + 1;
         end if;
      end loop;
   end Collect;

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
      for A in 1 .. Arm_Count (M) loop
         declare
            Arm : constant Arm_Id := Arm_Id (A);
            G   : constant Group_Id := Arm_Group (M, Arm);
            E   : constant Eye_Id'Base := Eye_Of (M, Arm);
            Index : Natural := 0;
         begin
            if E > 0 then
               for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                  if M.Kinematics (K).Arm = Arm then
                     Index := K;
                  end if;
               end loop;
               --  An arm met for the first time, or one whose group or eye the
               --  graph now tells differently, starts its evidence afresh.
               if Index = 0 then
                  M.Kinematics.Append (Arm_Evidence'(Arm => Arm, Group => G, Eye => E, others => <>));
                  Index := M.Kinematics.Last_Index;
               elsif M.Kinematics (Index).Group /= G or else M.Kinematics (Index).Eye /= E then
                  M.Kinematics.Replace_Element (Index, (Arm => Arm, Group => G, Eye => E, others => <>));
               end if;
               declare
                  R : Arm_Evidence renames M.Kinematics (Index);
               begin
                  Collect (R);
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
                        --  are measured.
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
                           Fresh := Fresh and then Seeable;
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
                                 Points : Driver.Instrument.Point_Array (1 .. Natural (R.Query_U.Length));
                              begin
                                 for P in Points'Range loop
                                    Points (P) := (U => R.Query_U (P - 1), V => R.Query_V (P - 1));
                                 end loop;
                                 R.Pending.Append
                                   (Pending_Match'(Frame  => R.Frames.Last_Index,
                                     Ticket => Driver.Instrument.Submit_Match
                                       ((Stored => False, Image => R.Frames.First_Element.Image),
                                        (Stored => False, Image => O.Images (E)),
                                        Points, True, O.Beat)));
                              end;
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Observe;

   function Matched (M : Model; A : Arm_Id) return Natural is
   begin
      for R of M.Kinematics loop
         if R.Arm = A then
            return Natural (R.Matches.Length);
         end if;
      end loop;
      return 0;
   end Matched;

   function Pending (M : Model) return Natural is
      K : Natural := 0;
   begin
      for R of M.Kinematics loop
         K := K + Natural (R.Pending.Length);
      end loop;
      return K;
   end Pending;

   procedure Refit (M : in out Model) is
   begin
      for Index in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R : Arm_Evidence renames M.Kinematics (Index);
         begin
            if not R.Matches.Is_Empty and then Natural (R.Matches.Length) /= R.Result.Matches
              and then R.Group <= M.Groups.Last_Index and then R.Eye <= M.Eyes.Last_Index
            then
               declare
                  N       : constant Natural := M.Groups (R.Group).Size;
                  Frames  : constant Natural := Natural (R.Frames.Length);
                  Queries : constant Natural := Natural (R.Query_U.Length);
                  Changes : Driver.Numerics.Arrays.Real_Matrix (1 .. Frames, 1 .. N);
                  Visible : Real_Array (1 .. N);
                  Count   : Natural := 0;
                  Trips   : Real_Access;
                  Sigma   : Real := 0.0;
               begin
                  for F in 1 .. Frames loop
                     for C in 1 .. N loop
                        Changes (F, C) := R.Frames (F).Readings (C - 1) - R.Frames (1).Readings (C - 1);
                     end loop;
                  end loop;
                  for C in 1 .. N loop
                     declare
                        V : constant Estimate := Visible_Step (M, R.Group, C);
                     begin
                        Visible (C) := (if Known (V) then V.Value else 0.0);
                     end;
                  end loop;
                  --  The noise of a round trip: the robust scale about zero of
                  --  every answer's return to its query, both coordinates.
                  for S of R.Matches loop
                     for I in 0 .. Queries - 1 loop
                        if S.Found (I) then
                           Count := Count + 1;
                        end if;
                     end loop;
                  end loop;
                  if Count > 0 then
                     Trips := new Real_Array (1 .. 2 * Count);
                     declare
                        K : Natural := 0;
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
                     end;
                     Sigma := Driver.Stats.Median (Trips.all) / Driver.Distributions.Gaussian_Two_Sided_Quantile (0.5);
                     Free (Trips);
                  end if;
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
                        D : Real_Array (1 .. Queries);
                        K : Natural := 0;
                     begin
                        for I in 0 .. Queries - 1 loop
                           if Round_Trip (S, I) then
                              K := K + 1;
                              D (K) := Ada.Numerics.Long_Elementary_Functions.Sqrt
                                ((S.To_U (I) - R.Query_U (I)) ** 2 + (S.To_V (I) - R.Query_V (I)) ** 2);
                           end if;
                        end loop;
                        return K > 0 and then Driver.Uncertain.Significant (Driver.Stats.Median (D (1 .. K)), Sigma);
                     end Moved;

                     Moving : array (R.Matches.First_Index .. R.Matches.Last_Index) of Boolean;

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
                        Fit.Fit (Changes, Visible, Seen.all, M.Eyes (R.Eye).Grid.Width, M.Eyes (R.Eye).Grid.Height,
                                 Joints, Lens, Report);
                        --  Up: the table the first arm's eye sees, away from it
                        --  towards the eye (the world is that eye's reference
                        --  frame).
                        if Report.Fitted and then R.Arm = 1 then
                           declare
                              Normal : Vec3;
                              Sigma  : Real;
                              Found  : Boolean;
                           begin
                              Fit.Table (Changes, Seen.all, Joints, Lens, Normal, Sigma, Found);
                              if Found then
                                 M.Table_Up := (Unit_Vector => Normal, Sigma => Sigma);
                              end if;
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
                  end;
               end;
            end if;
         end;
      end loop;
   end Refit;

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid is
   begin
      for R of M.Kinematics loop
         if R.Arm = A and then R.Result.Fitted
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

   function Fitted (M : Model; A : Arm_Id) return Boolean is
   begin
      for R of M.Kinematics loop
         if R.Arm = A then
            return R.Result.Fitted;
         end if;
      end loop;
      return False;
   end Fitted;

   procedure Solve_Pose
     (M             : Model;
      A             : Arm_Id;
      Start         : Real_Array;
      Goal          : Rigid;
      Position_Only : Boolean;
      Low, High     : Real_Array;
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

      function Clamp (V : Real_Array) return Real_Array is
         R : Real_Array (1 .. N) := V;
      begin
         for J in 1 .. N loop
            R (J) := Real'Max (Low (Low'First + J - 1), Real'Min (High (High'First + J - 1), V (V'First + J - 1)));
         end loop;
         return R;
      end Clamp;

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
      X := Clamp (X);
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
                           Xn := Clamp (Xn);
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
   begin
      for R of M.Kinematics loop
         if R.Arm = A then
            return R.Result;
         end if;
      end loop;
      return (others => <>);
   end Result_Of;

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
