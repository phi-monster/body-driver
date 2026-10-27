with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Pixels;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Regression;
with Driver.Stats;

package body Driver.Robot.Stillness is


   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Luma_Access);

   --  How many pixels of the frame differ from the settled view's mean
   --  significantly against the noise view's per-pixel noise. Means and
   --  Variances are the eye's buffers for the two views' pixels.
   function Changed_Pixels
     (View, Noise : Driver.Pixels.View; Luma : Real_Array; Means, Variances : in out Real_Array) return Natural
   is
      Settled : constant Natural := Driver.Pixels.Frames (View);
      --  The noise of a variance measured over K frames rests on K - 1
      --  degrees of freedom; one frame has only the known floor.
      Freedom : constant Natural := Driver.Pixels.Frames (Noise) - 1;
      --  The gate's own threshold, compared in squares: the same test without
      --  a square root per pixel (every variance is positive and finite).
      T       : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate (Freedom));
      --  The frame and the settled mean share the pixel's noise; the mean's
      --  is that over the settled frames.
      Scale   : constant Real := T * T * (1.0 + 1.0 / Real (Settled));
      Count   : Natural := 0;
   begin
      Driver.Pixels.Means (View, Means);
      Driver.Pixels.Variances (Noise, Variances);
      for K in 0 .. Luma'Length - 1 loop
         if (Luma (Luma'First + K) - Means (Means'First + K)) ** 2 > Scale * Variances (Variances'First + K) then
            Count := Count + 1;
         end if;
      end loop;
      return Count;
   end Changed_Pixels;

   procedure Judge_Eye (S : in out Eye_Stream; Frame : Driver.Images.Image; Luma : Real_Array; Moved : Boolean) is
      W : constant Positive := Driver.Images.Width (Frame);
      H : constant Positive := Driver.Images.Height (Frame);
      --  The still view begins again at this frame; the run it ends measured the noise, when it was the longest so far
      --  and held two frames.
      procedure Start_Again is
      begin
         if S.Noise_Is_Settled and then Driver.Pixels.Frames (S.Settled) >= 2 then
            S.Noise_View := S.Settled;
            S.Noise_Is_Settled := False;
         end if;
         S.Settled := Driver.Pixels.Empty (W, H);
         Driver.Pixels.Add (S.Settled, Frame);
         S.Run_Moved := False;
      end Start_Again;
   begin
      if not S.Has_Settled or else Driver.Pixels.Width (S.Settled) /= W or else Driver.Pixels.Height (S.Settled) /= H
      then
         S.Settled := Driver.Pixels.Empty (W, H);
         Driver.Pixels.Add (S.Settled, Frame);
         S.Noise_Is_Settled := True;
         S.Has_Settled := True;
         S.Run_Moved := False;
         S.Is_Still := False;
         S.Has_Judged := False;
         return;
      end if;
      --  The noise of a pixel needs two frames to be measured at all; until
      --  then the frames start the first run, unjudged.
      if S.Noise_Is_Settled and then Driver.Pixels.Frames (S.Settled) < 2 then
         if Moved then
            Start_Again;
         else
            Driver.Pixels.Add (S.Settled, Frame);
         end if;
         S.Is_Still := False;
         S.Has_Judged := False;
         return;
      end if;
      S.Has_Judged := True;
      if S.Means = null or else S.Means'Length /= W * H then
         Free (S.Means);
         Free (S.Variances);
         S.Means := new Real_Array (1 .. W * H);
         S.Variances := new Real_Array (1 .. W * H);
      end if;
      declare
         --  Every pixel's test alarms by chance at this rate.
         P0      : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
         Changed : constant Natural :=
           (if S.Noise_Is_Settled then Changed_Pixels (S.Settled, S.Settled, Luma, S.Means.all, S.Variances.all)
            else Changed_Pixels (S.Settled, S.Noise_View, Luma, S.Means.all, S.Variances.all));
      begin
         S.Is_Still := not Regression.Count_Significant (Changed, W * H, P0);
      end;
      --  A frame taken while the body moved shows no still picture of where it stands, however small the change one
      --  frame's test can see: a change too small for it, against the still view's noise, can be plain between two
      --  still views' means (SO-101's base camera on the hall, 10-27: its elbow turned 0.0084 rad either way for
      --  step 3's flows, the frames at the second turn joined the still view of the first, and the turn's change
      --  between the two views was a fifth to a third of the true turn's).
      if Moved then
         Start_Again;
      elsif S.Is_Still then
         Driver.Pixels.Add (S.Settled, Frame);
         --  The longest still run is the best measure of the eye's noise; a run that holds frames the noise called
         --  changed measures none.
         if not S.Noise_Is_Settled and then not S.Run_Moved
           and then Driver.Pixels.Frames (S.Settled) > Driver.Pixels.Frames (S.Noise_View)
         then
            S.Noise_Is_Settled := True;
         end if;
      else
         if S.Noise_Is_Settled then
            S.Noise_View := S.Settled;
            S.Noise_Is_Settled := False;
         end if;
         --  After the picture stopped (Watch, by the beats before this one) a frame the noise calls changed joins
         --  the still view all the same: a flicker that lasts after a move never comes back to the noise at rest
         --  (robot.settle.tail), and the still view is then the mean of the frames since the stop, the flicker
         --  averaged in it. Started again at every such frame, the still views of k201, k203 and k208 (kiwis from
         --  their swept bodies on the big machine, 10-22, an eye each whose content changed with the arm's move and
         --  whose MJPEG blocks then jumped by more than the noise measured on the content before) never held two
         --  frames, and their step 3 captures waited for them until their runs ended.
         if S.Watch_Done then
            Driver.Pixels.Add (S.Settled, Frame);
            S.Run_Moved := True;
         else
            S.Settled := Driver.Pixels.Empty (W, H);
            Driver.Pixels.Add (S.Settled, Frame);
            S.Run_Moved := False;
         end if;
      end if;
   end Judge_Eye;

   procedure Measure_Luma_Noise (S : in out Eye_Stream) is
      N : constant Natural := Cells (S.Grid);

      procedure Measure (Noise : Driver.Pixels.View) is
      begin
         if N = 0 or else Driver.Pixels.Width (Noise) /= S.Grid.Width
           or else Driver.Pixels.Height (Noise) /= S.Grid.Height
         then
            return;
         end if;
         S.Luma_Variance.Clear;
         for Cell in 1 .. N loop
            declare
               X0, X1, Y0, Y1 : Natural;
            begin
               Flow.Bounds (S.Grid, Cell, X0, X1, Y0, Y1);
               declare
                  V : Luma_Access := new Real_Array (1 .. (X1 - X0) * (Y1 - Y0));
                  K : Natural := 0;
               begin
                  for Row in Y0 .. Y1 - 1 loop
                     for Column in X0 .. X1 - 1 loop
                        K := K + 1;
                        V (K) := Driver.Pixels.Variance (Noise, Column, Row);
                     end loop;
                  end loop;
                  S.Luma_Variance.Append (Driver.Stats.Median (V.all));
                  Free (V);
               end;
            end;
         end loop;
      end Measure;
   begin
      if S.Noise_Is_Settled then
         Measure (S.Settled);
      else
         Measure (S.Noise_View);
      end if;
   end Measure_Luma_Noise;

   function Group_Still (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (Channels.Noise_Measured (M, G) and then Beat > 0 and then Channels.Has_Reading (M, G, Beat)
      and then Channels.Has_Reading (M, G, Beat - 1) and then not Channels.Moving (M, G, Beat));

   function Eye_Still (M : Model; E : Eye_Id) return Boolean is (M.Eyes (E).Is_Still);

   --  The mean change of A against B, each pixel's in units of its noise at
   --  rest (Variance: its luma variance over the eye's noise view, which
   --  Driver.Pixels never takes below the 8-bit quantization).
   --  (Deviation: the square roots of those variances, taken once for both
   --  of a beat's comparisons.)
   function Mean_Change (A, B, Deviation : Real_Array) return Real
     with Pre => A'Length = B'Length and then A'Length = Deviation'Length and then A'Length > 0
   is
      Sum : Real := 0.0;
   begin
      for I in A'Range loop
         Sum := Sum + abs (A (I) - B (I - A'First + B'First)) / Deviation (I - A'First + Deviation'First);
      end loop;
      return Sum / Real (A'Length);
   end Mean_Change;

   procedure Watch (S : in out Eye_Stream; Began_Moving : Boolean) is
      --  How much more a change over two beats is than over one when successive changes do not push the same
      --  way: independent changes add as roots of squares, the square root of two, the most any noise that
      --  stays about its mean (flicker, or a pixel that wanders and is pulled back) does. A drift adds them
      --  all, twice. A22's three eyes at rest: the middle of the ratio was 1.43 to 1.46, its quartiles 1.37 to 1.53.
      Adds_Up : constant Real := Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0);
   begin
      S.Change_1 := -1.0;
      S.Change_2 := -1.0;
      --  The pixels' noise at rest is what the eye's last judgment read of its
      --  noise view (Judge_Eye): none before the eye is judged.
      if S.Current /= null and then S.Previous /= null and then S.Has_Previous
        and then S.Current'Length = S.Previous'Length and then S.Current'Length > 0
        and then S.Has_Judged and then S.Variances /= null and then S.Variances'Length = S.Current'Length
      then
         if S.Deviations = null or else S.Deviations'Length /= S.Variances'Length then
            Free (S.Deviations);
            S.Deviations := new Real_Array (S.Variances'Range);
         end if;
         for I in S.Variances'Range loop
            S.Deviations (I) := Ada.Numerics.Long_Elementary_Functions.Sqrt (S.Variances (I));
         end loop;
         S.Change_1 := Mean_Change (S.Current.all, S.Previous.all, S.Deviations.all);
         if S.Before /= null and then S.Has_Before and then S.Before'Length = S.Current'Length then
            S.Change_2 := Mean_Change (S.Current.all, S.Before.all, S.Deviations.all);
         end if;
         --  The change from the frame the latest push began from (Set_Origin).
         if S.Has_Origin and then S.Origin /= null and then S.Origin'Length = S.Current'Length then
            S.From_Origin := Mean_Change (S.Current.all, S.Origin.all, S.Deviations.all);
         end if;
         --  The change from the anchor, against what rest makes (the stationary beats' median change and its scale): a
         --  picture farther from it than rest moves one, and farther than at the beat before.
         if S.Has_Anchor and then S.Anchor /= null and then S.Anchor'Length = S.Current'Length
           and then S.Watch_Rest < Real'Last
         then
            declare
               Away_Now : constant Real := Mean_Change (S.Current.all, S.Anchor.all, S.Deviations.all);
               Away     : constant Boolean :=
                 Away_Now > S.Watch_Rest + Driver.Conventions.Z * S.Watch_Rest_Scale and then Away_Now > S.Away_Last;
            begin
               S.Away_Run := (if Away then S.Away_Run + 1 else 0);
               S.Away_Last := Away_Now;
            end;
         end if;
      end if;
      if Began_Moving then
         S.Watch_Have := False;
         S.Watch_Done := False;
         S.Watch_Run := 0;
         S.Drift_Run := 0;
      end if;
      --  A beat without two frames to compare counts neither way.
      if S.Change_1 >= 0.0 then
         declare
            C1 : constant Real := S.Change_1;
            C2 : constant Real := S.Change_2;
            --  The step of the change from the beat before, positive while it shrinks; a stationary beat is one
            --  whose step is under the unchanged fraction of the change (converged) and not a shrinking one against
            --  what the change wanders by at a lasting level (the median of its steps at the stationary beats so far;
            --  a tail that decays under a large steady flicker shrinks by less than a hundredth of the whole and by
            --  many times that wander: robot.boot.settling held 18 keyframes on such tails).
            Step       : constant Real := (if S.Watch_Have then S.Watch_Last - C1 else 0.0);
            Converged  : constant Boolean :=
              S.Watch_Have and then abs Step <= Driver.Conventions.Unchanged_Fraction * S.Watch_Last;
            Shrinking  : constant Boolean :=
              S.Watch_Have and then S.Watch_Scatter_Counted > 0 and then Step > Driver.Conventions.Z * S.Watch_Scatter;
            --  The two-beat change adds up as rest's does, not as a drift's: within Z of the step and the wander,
            --  whichever is larger (a step that happens to be tiny on a noisy picture is no measure of what the two-beat
            --  change wanders by; the beat after a jump has a large step, and its two-beat change spans the jump).
            Adds_As_Rest : constant Boolean :=
              C2 < 0.0 or else C2 - Adds_Up * C1 <= Driver.Conventions.Z * Real'Max (abs Step, S.Watch_Scatter);
            --  A drift is converged and not shrinking, and is no rest: what rest makes is measured on beats that add
            --  up as rest does.
            Stationary : constant Boolean := Converged and then not Shrinking and then Adds_As_Rest;
            --  A drift: the two-beat change adds up as a drift's, the second beat going on the way the first went, and
            --  not shrinking. A body that keeps going while every reading stands (a speed held: a base, a walk, a
            --  flight) drifts its eyes' pictures beat after beat; a push settling shrinks.
            Drifting   : constant Boolean := S.Watch_Have and then not Adds_As_Rest and then not Shrinking;
         begin
            S.Watch_Run := (if Stationary then S.Watch_Run + 1 else 0);
            S.Drift_Run := (if Drifting then S.Drift_Run + 1 else 0);
            --  The picture has stopped when its change is what rest makes (the median of the change at the stationary
            --  beats so far, Z of their scale allowed: a residual of a move that is many times the noise is not rest,
            --  item 22), or when it has been stationary for two beats running at a level rest never showed (a lasting
            --  flicker that came with the move: waiting for it to end is waiting forever); and in either case the
            --  change over two beats adds up as rest's does. It used to be stopped as well when the change had fallen
            --  to a hundredth of the largest since the move: a hundredth of a jump is still many times the noise, and
            --  a fake body's eye was taken stopped one beat after a move while its picture still changed by two
            --  levels, 24 keyframes on changing pictures.
            if S.Watch_Have and then Adds_As_Rest
              and then ((S.Watch_Rest < Real'Last and then C1 <= S.Watch_Rest + Driver.Conventions.Z * S.Watch_Rest_Scale)
                        or else S.Watch_Run >= 2)
            then
               S.Watch_Done := True;
            end if;
            if Stationary then
               S.Watch_Diffs.Append (abs Step);
               S.Watch_Levels.Append (C1);
               if Natural (S.Watch_Diffs.Length) >= 2 * Natural'Max (1, S.Watch_Scatter_Counted) then
                  declare
                     V : Real_Array (1 .. Natural (S.Watch_Diffs.Length));
                     L : Real_Array (1 .. Natural (S.Watch_Levels.Length));
                  begin
                     for I in V'Range loop
                        V (I) := S.Watch_Diffs (S.Watch_Diffs.First_Index + I - 1);
                     end loop;
                     for I in L'Range loop
                        L (I) := S.Watch_Levels (S.Watch_Levels.First_Index + I - 1);
                     end loop;
                     S.Watch_Scatter := Driver.Stats.Median (V);
                     S.Watch_Scatter_Counted := V'Length;
                     if L'Length >= 2 then
                        S.Watch_Rest := Driver.Stats.Median (L);
                        S.Watch_Rest_Scale := Driver.Stats.Robust_Sigma (L);
                        S.Watch_Rest_Counted := L'Length;
                     end if;
                  end;
               end if;
            end if;
            S.Watch_Last := C1;
            S.Watch_Have := True;
         end;
      end if;
   end Watch;

   function Eye_Settled (M : Model; E : Eye_Id) return Boolean is (M.Eyes (E).Watch_Done);

   function Eye_Drift_Run (M : Model; E : Eye_Id) return Natural is (M.Eyes (E).Drift_Run);

   procedure Set_Anchor (M : in out Model; E : Eye_Id) is
      S : Eye_Stream renames M.Eyes (E);
      Latest : constant Luma_Access := Latest_Frame (S);
   begin
      if Latest = null then
         S.Has_Anchor := False;
         return;
      end if;
      if S.Anchor = null or else S.Anchor'Length /= Latest'Length then
         Free (S.Anchor);
         S.Anchor := new Real_Array (Latest'Range);
      end if;
      S.Anchor.all := Latest.all;
      S.Has_Anchor := True;
      S.Away_Run := 0;
      S.Away_Last := 0.0;
   end Set_Anchor;

   procedure Drop_Anchor (M : in out Model; E : Eye_Id) is
   begin
      M.Eyes (E).Has_Anchor := False;
      M.Eyes (E).Away_Run := 0;
   end Drop_Anchor;

   function Eye_Away_Run (M : Model; E : Eye_Id) return Natural is (M.Eyes (E).Away_Run);

   function Eye_From_Anchor (M : Model; E : Eye_Id) return Real is
     (if M.Eyes (E).Has_Anchor then M.Eyes (E).Away_Last else 0.0);

   procedure Set_Origin (M : in out Model; E : Eye_Id) is
      S : Eye_Stream renames M.Eyes (E);
      Latest : constant Luma_Access := Latest_Frame (S);
   begin
      S.From_Origin := Real'Last;
      S.Origin_At := M.Beats;
      if Latest = null then
         S.Has_Origin := False;
         return;
      end if;
      if S.Origin = null or else S.Origin'Length /= Latest'Length then
         Free (S.Origin);
         S.Origin := new Real_Array (Latest'Range);
      end if;
      S.Origin.all := Latest.all;
      S.Has_Origin := True;
   end Set_Origin;

   function Eye_From_Origin (M : Model; E : Eye_Id) return Real is
     (if M.Eyes (E).Has_Origin then M.Eyes (E).From_Origin else Real'Last);

   function Eye_Origin_At (M : Model; E : Eye_Id) return Natural is (M.Eyes (E).Origin_At);

   function Eye_Rest_Change (M : Model; E : Eye_Id) return Real is (M.Eyes (E).Watch_Rest);

   function Eye_Rest_Scale (M : Model; E : Eye_Id) return Real is (M.Eyes (E).Watch_Rest_Scale);

   function All_Still (M : Model) return Boolean is
   begin
      if M.Beats = 0 then
         return False;
      end if;
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if not Group_Still (M, G, M.Beats - 1) then
            return False;
         end if;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         if not Eye_Settled (M, E) then
            return False;
         end if;
      end loop;
      return True;
   end All_Still;

end Driver.Robot.Stillness;
