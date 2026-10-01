with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Pixels;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Regression;
with Driver.Stats;

package body Driver.Robot.Stillness is

   use Ada.Numerics.Long_Elementary_Functions;

   --  How many pixels of the frame differ from the settled view's mean
   --  significantly against the noise view's per-pixel noise.
   function Changed_Pixels (View, Noise : Driver.Pixels.View; Luma : Real_Array) return Natural is
      W       : constant Natural := Driver.Pixels.Width (View);
      Settled : constant Natural := Driver.Pixels.Frames (View);
      --  The noise of a variance measured over K frames rests on K - 1
      --  degrees of freedom; one frame has only the known floor.
      Freedom : constant Natural := Driver.Pixels.Frames (Noise) - 1;
      Gate    : constant Driver.Uncertain.Gate := Driver.Uncertain.Scalar_Gate (Freedom);
      Count   : Natural := 0;
   begin
      for Row in 0 .. Driver.Pixels.Height (View) - 1 loop
         for Column in 0 .. W - 1 loop
            declare
               D : constant Real := Luma (Luma'First + Row * W + Column) - Driver.Pixels.Mean (View, Column, Row);
               --  The frame and the settled mean share the pixel's noise; the
               --  mean's is that over the settled frames.
               V : constant Real := Driver.Pixels.Variance (Noise, Column, Row) * (1.0 + 1.0 / Real (Settled));
            begin
               if Driver.Uncertain.Significant (Gate, D, Sqrt (V)) then
                  Count := Count + 1;
               end if;
            end;
         end loop;
      end loop;
      return Count;
   end Changed_Pixels;

   procedure Judge_Eye (S : in out Eye_Stream; Frame : Driver.Images.Image; Luma : Real_Array) is
      W : constant Positive := Driver.Images.Width (Frame);
      H : constant Positive := Driver.Images.Height (Frame);
   begin
      if not S.Has_Settled or else Driver.Pixels.Width (S.Settled) /= W or else Driver.Pixels.Height (S.Settled) /= H
      then
         S.Settled := Driver.Pixels.Empty (W, H);
         Driver.Pixels.Add (S.Settled, Frame);
         S.Noise_Is_Settled := True;
         S.Has_Settled := True;
         S.Is_Still := False;
         return;
      end if;
      --  The noise of a pixel needs two frames to be measured at all; until
      --  then the frames start the first run, unjudged.
      if S.Noise_Is_Settled and then Driver.Pixels.Frames (S.Settled) < 2 then
         Driver.Pixels.Add (S.Settled, Frame);
         S.Is_Still := False;
         return;
      end if;
      declare
         --  Every pixel's test alarms by chance at this rate.
         P0      : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
         Changed : constant Natural :=
           (if S.Noise_Is_Settled then Changed_Pixels (S.Settled, S.Settled, Luma)
            else Changed_Pixels (S.Settled, S.Noise_View, Luma));
      begin
         S.Is_Still := not Regression.Count_Significant (Changed, W * H, P0);
      end;
      if S.Is_Still then
         Driver.Pixels.Add (S.Settled, Frame);
         --  The longest still run is the best measure of the eye's noise.
         if not S.Noise_Is_Settled and then Driver.Pixels.Frames (S.Settled) > Driver.Pixels.Frames (S.Noise_View) then
            S.Noise_Is_Settled := True;
         end if;
      else
         if S.Noise_Is_Settled then
            S.Noise_View := S.Settled;
            S.Noise_Is_Settled := False;
         end if;
         S.Settled := Driver.Pixels.Empty (W, H);
         Driver.Pixels.Add (S.Settled, Frame);
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
                  V : Real_Array (1 .. (X1 - X0) * (Y1 - Y0));
                  K : Natural := 0;
               begin
                  for Row in Y0 .. Y1 - 1 loop
                     for Column in X0 .. X1 - 1 loop
                        K := K + 1;
                        V (K) := Driver.Pixels.Variance (Noise, Column, Row);
                     end loop;
                  end loop;
                  S.Luma_Variance.Append (Driver.Stats.Median (V));
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
         if not Eye_Still (M, E) then
            return False;
         end if;
      end loop;
      return True;
   end All_Still;

end Driver.Robot.Stillness;
