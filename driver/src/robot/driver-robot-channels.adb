with Ada.Containers;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Distributions;
with Driver.Stats;

package body Driver.Robot.Channels is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  Everything sized by beats lives on the heap: the estimates also run in
   --  the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   procedure Append (M : in out Model; O : Observation; Sent : Driver.Commands.Command) is
   begin
      if M.Groups.Is_Empty then
         for G in O.Readings.First_Index .. O.Readings.Last_Index loop
            M.Groups.Append (Group_Stream'(Size => O.Readings.Element (G)'Length, others => <>));
         end loop;
      end if;
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            S : Group_Stream renames M.Groups (G);
            Earlier : constant Natural := Natural (S.Present.Length);
         begin
            --  A group whose first reading came late learns its size then;
            --  the beats before hold zeros that are never read.
            if S.Size = 0 and then G <= O.Readings.Last_Index and then O.Readings.Element (G)'Length > 0 then
               S.Size := O.Readings.Element (G)'Length;
               S.Values.Prepend (0.0, Ada.Containers.Count_Type (S.Size * Earlier));
               S.Targets.Prepend (0.0, Ada.Containers.Count_Type (S.Size * Earlier));
            end if;
         end;
         declare
            S : Group_Stream renames M.Groups (G);
            Have : constant Boolean := S.Size > 0 and then G <= O.Readings.Last_Index
                                       and then O.Readings.Element (G)'Length = S.Size;
            Aimed : constant Boolean := S.Size > 0 and then Driver.Commands.Has_Target (Sent, G)
                                        and then Driver.Commands.Target (Sent, G)'Length = S.Size;
         begin
            S.Present.Append (Have);
            if Have then
               for V of O.Readings.Element (G) loop
                  S.Values.Append (V);
               end loop;
            else
               S.Values.Append (0.0, Ada.Containers.Count_Type (S.Size));
            end if;
            S.Targeted.Append (Aimed);
            if Aimed then
               S.Commandable := True;
               for V of Driver.Commands.Target (Sent, G) loop
                  S.Targets.Append (V);
               end loop;
            else
               S.Targets.Append (0.0, Ada.Containers.Count_Type (S.Size));
            end if;
         end;
      end loop;
   end Append;

   function Has_Reading (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (G <= M.Groups.Last_Index and then Beat < Natural (M.Groups (G).Present.Length)
      and then M.Groups (G).Present (Beat));

   function Reading (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real is
     (M.Groups (G).Values (Beat * M.Groups (G).Size + Channel - 1));

   function Has_Target (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (G <= M.Groups.Last_Index and then Beat < Natural (M.Groups (G).Targeted.Length)
      and then M.Groups (G).Targeted (Beat));

   function Target (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real is
     (M.Groups (G).Targets (Beat * M.Groups (G).Size + Channel - 1));

   function Change (M : Model; G : Group_Id; Beat : Natural; Channel : Positive) return Real is
     (Reading (M, G, Beat, Channel) - Reading (M, G, Beat - 1, Channel));

   function Target_Changed (M : Model; G : Group_Id; Beat : Natural) return Boolean is
   begin
      if Beat = 0 then
         return False;
      elsif Has_Target (M, G, Beat) /= Has_Target (M, G, Beat - 1) then
         return True;
      elsif not Has_Target (M, G, Beat) then
         return False;
      end if;
      for C in 1 .. M.Groups (G).Size loop
         if Target (M, G, Beat, C) /= Target (M, G, Beat - 1, C) then
            return True;
         end if;
      end loop;
      return False;
   end Target_Changed;

   --  Offset of a group's first channel in the flat list of all channels.
   function First_Channel (M : Model; G : Group_Id) return Natural is
      N : Natural := 0;
   begin
      for H in M.Groups.First_Index .. G - 1 loop
         N := N + M.Groups (H).Size;
      end loop;
      return N;
   end First_Channel;

   function Mad_Degrees_Of_Freedom (N : Natural) return Natural is
      Half : constant Real := 0.5;
      C    : constant Real := Driver.Distributions.Gaussian_Two_Sided_Quantile (Half);
      Phi  : constant Real := Exp (-C * C / 2.0) / Sqrt (2.0 * Ada.Numerics.Pi);
   begin
      --  Rounding a handful of samples down to none would call the sigma known.
      return (if N < 2 then 0 else Natural'Max (1, Natural (Real'Floor (8.0 * C * C * Phi * Phi * Real (N)))));
   end Mad_Degrees_Of_Freedom;

   --  Whether the change into Beat is one of a resting reading: the beat and
   --  the one before have readings, and for a commanded group the target did
   --  not change and no push of it was under way.
   function At_Rest (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (Beat > 0 and then Has_Reading (M, G, Beat) and then Has_Reading (M, G, Beat - 1)
      and then (not M.Groups (G).Commandable
                or else (not Target_Changed (M, G, Beat) and then not Pushed (M, G, Beat))));

   --  One channel's noise from its rest changes. A resting reading either
   --  repeats exactly (a simulator between physics updates, a quantized or
   --  echoed value) or changes by its jitter; the changes that are not exact
   --  repeats measure the jitter. A channel that repeats exactly at every rest
   --  beat has noise zero: any change of it is motion.
   procedure Measure_Channel (M : in out Model; G : Group_Id; Channel : Positive; Rest : Natural) is
      --  Every nonzero change at rest, and its negation: a change at rest has
      --  no sign of its own, so its scale is measured about zero. About their
      --  own median, two-valued changes (a reading whose last bit flips back
      --  and forth, one way more often than the other) leave no spread at all.
      Moved : Real_Access := new Real_Array (1 .. 2 * Rest);
      K     : Natural := 0;
      Index : constant Natural := First_Channel (M, G) + Channel - 1;
   begin
      for B in 1 .. Natural (M.Groups (G).Present.Length) - 1 loop
         if At_Rest (M, G, B) and then Change (M, G, B, Channel) /= 0.0 then
            K := K + 1;
            Moved (K) := Change (M, G, B, Channel);
            Moved (Rest + K) := -Change (M, G, B, Channel);
         end if;
      end loop;
      if K = 0 then
         M.Noise.Replace_Element (Index, 0.0);
         M.Noise_Freedom.Replace_Element (Index, 0);
      else
         Moved (K + 1 .. 2 * K) := Moved (Rest + 1 .. Rest + K);
         M.Noise.Replace_Element (Index, Driver.Stats.Robust_Sigma (Moved (1 .. 2 * K)) / Sqrt (2.0));
         M.Noise_Freedom.Replace_Element (Index, Mad_Degrees_Of_Freedom (K));
      end if;
      Free (Moved);
   end Measure_Channel;

   procedure Measure_Noise (M : in out Model) is
      Total : Natural := 0;
   begin
      for S of M.Groups loop
         Total := Total + S.Size;
      end loop;
      M.Noise.Clear;
      M.Noise.Append (Real'Last, Ada.Containers.Count_Type (Total));
      M.Noise_Freedom.Clear;
      M.Noise_Freedom.Append (0, Ada.Containers.Count_Type (Total));
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            Rest : Natural := 0;
         begin
            for B in 1 .. Natural (M.Groups (G).Present.Length) - 1 loop
               if At_Rest (M, G, B) then
                  Rest := Rest + 1;
               end if;
            end loop;
            if Rest > 0 then
               for C in 1 .. M.Groups (G).Size loop
                  Measure_Channel (M, G, C, Rest);
               end loop;
            end if;
         end;
      end loop;
   end Measure_Noise;

   function Noise (M : Model; G : Group_Id; Channel : Positive) return Real is
      K : constant Natural := First_Channel (M, G) + Channel - 1;
   begin
      return (if K < Natural (M.Noise.Length) then M.Noise (K) else Real'Last);
   end Noise;

   function Noise_Measured (M : Model; G : Group_Id) return Boolean is
     (First_Channel (M, G) + M.Groups (G).Size <= Natural (M.Noise.Length));

   function Noise_Freedom (M : Model; G : Group_Id; Channel : Positive) return Natural is
      K : constant Natural := First_Channel (M, G) + Channel - 1;
   begin
      return (if K < Natural (M.Noise_Freedom.Length) then M.Noise_Freedom (K) else 0);
   end Noise_Freedom;

   --  Whether a difference of the group's readings, one value per channel,
   --  is significant against the channels' noise times Scale (one for a
   --  reading against an exact value, the square root of two for the change
   --  of two readings). A channel that repeats exactly has moved when it
   --  changed at all; the others are tested together, each over its own
   --  sigma, as one vector against the vector gate of as many dimensions
   --  (their squared deviations add to a chi-square), so a group of many
   --  channels raises no more false alarms than a single one. A channel
   --  whose noise is not measured yet gives no evidence.
   function Significant_Change (M : Model; G : Group_Id; D : Real_Array; Scale : Real) return Boolean is
      Q       : Real := 0.0;
      Tested  : Natural := 0;
      Freedom : Natural := Natural'Last;
   begin
      for C in D'Range loop
         declare
            Sigma : constant Real := Noise (M, G, C);
         begin
            if Sigma = 0.0 then
               if D (C) /= 0.0 then
                  return True;
               end if;
            elsif Sigma < Real'Last then
               Q := Q + (D (C) / (Sigma * Scale)) ** 2;
               Tested := Tested + 1;
               Freedom := Natural'Min (Freedom, Noise_Freedom (M, G, C));
            end if;
         end;
      end loop;
      return Tested > 0
        and then Driver.Uncertain.Significant
          (Driver.Uncertain.Vector_Gate (Tested, (if Freedom = Natural'Last then 0 else Freedom)), Sqrt (Q), 1.0);
   end Significant_Change;

   function Asked (M : Model; G : Group_Id; Beat : Natural) return Boolean is
   begin
      if Beat = 0 or else not Target_Changed (M, G, Beat) or else not Has_Target (M, G, Beat)
        or else not Has_Reading (M, G, Beat - 1)
      then
         return False;
      end if;
      declare
         Ask : Real_Array (1 .. M.Groups (G).Size);
         Unknown_Noise : Boolean := False;
      begin
         for C in Ask'Range loop
            Ask (C) := Target (M, G, Beat, C) - Reading (M, G, Beat - 1, C);
            Unknown_Noise := Unknown_Noise or else Noise (M, G, C) = Real'Last;
         end loop;
         --  Before the noise is measured, any change of target that differs
         --  from the reading asks for motion.
         if Unknown_Noise then
            return (for some A of Ask => A /= 0.0);
         end if;
         return Significant_Change (M, G, Ask, 1.0);
      end;
   end Asked;

   procedure Measure_Pushes (M : in out Model) is
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            Beats  : constant Natural := Natural (M.Groups (G).Present.Length);
            Delay_Beats : Natural := 0;
            Answered    : Boolean := False;
            Marks  : Flag_Vectors.Vector;
            Active : Boolean := False;
            Moved  : Boolean := False;
            Onset  : Natural := 0;
         begin
            if M.Groups (G).Commandable then
               --  The response delay: the longest wait from a push's start to
               --  its reading's first motion, over the pushes that were
               --  answered before the next one started.
               for B in 0 .. Beats - 1 loop
                  if Asked (M, G, B) then
                     for D in B .. Beats - 1 loop
                        exit when D > B and then Asked (M, G, D);
                        if Moving (M, G, D) then
                           Delay_Beats := Natural'Max (Delay_Beats, D - B);
                           Answered := True;
                           exit;
                        end if;
                     end loop;
                  end if;
               end loop;
               M.Groups (G).Delay_Beats := Delay_Beats;
               M.Groups (G).Delay_Known := Answered;
            end if;
            for B in 0 .. Beats - 1 loop
               if M.Groups (G).Commandable and then Asked (M, G, B) then
                  Active := True;
                  Onset := B;
                  Moved := Moving (M, G, B);
               elsif Active then
                  --  Until the reading first moves the push waits out the
                  --  delay; after that its response lasts, overshoot and all,
                  --  until the reading is still. A push that is not answered
                  --  within the delay is over.
                  if not Moved then
                     Moved := Moving (M, G, B);
                     if not Moved and then B - Onset >= Delay_Beats then
                        Active := False;
                     end if;
                  elsif not Moving (M, G, B) then
                     Active := False;
                  end if;
               end if;
               Marks.Append (Active);
            end loop;
            M.Groups (G).Pushed := Marks;
         end;
      end loop;
   end Measure_Pushes;

   function Pushed (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (G <= M.Groups.Last_Index and then Beat < Natural (M.Groups (G).Pushed.Length)
      and then M.Groups (G).Pushed (Beat));

   procedure Measure (M : in out Model) is
      Beats : Natural := 0;
   begin
      for S of M.Groups loop
         S.Pushed.Clear;
         Beats := Natural'Max (Beats, Natural (S.Present.Length));
      end loop;
      --  The noise is measured at rest, and rest is where no push is under
      --  way, which takes the noise to find: the two are measured in turn
      --  until the pushes found no longer change. The first round takes every
      --  change for motion (noise zero), so a push lasts as long as its
      --  reading changes at all and the tail of a slow response is never
      --  taken for rest; the noise measured outside those pushes is then the
      --  jitter alone, or zero where a reading has none. Each round can only
      --  move push marks, so as many rounds as beats bound it. The rounds can
      --  also alternate: a beat right after a push whose change is
      --  significant against the noise measured with it among the rest
      --  beats, and not against the noise measured without it (with few
      --  nonzero rest changes, their degrees of freedom move with it), is
      --  rest one round and pushed the next, for good (A9's replay spent 540
      --  s in such rounds at 1024 beats). When a round brings back the marks
      --  of the round before the last, every beat either of the two marked
      --  is taken as pushed, not as rest, and the noise is measured once more
      --  outside them.
      declare
         Total : Natural := 0;
      begin
         for S of M.Groups loop
            Total := Total + S.Size;
         end loop;
         M.Noise.Clear;
         M.Noise.Append (0.0, Ada.Containers.Count_Type (Total));
         M.Noise_Freedom.Clear;
         M.Noise_Freedom.Append (0, Ada.Containers.Count_Type (Total));
      end;
      declare
         --  The marks of the round before, and of the one before that.
         type Marks is array (M.Groups.First_Index .. M.Groups.Last_Index) of Flag_Vectors.Vector;
         Before, Older : Marks;
      begin
         for Round in 0 .. Beats loop
            if Round > 0 then
               Measure_Noise (M);
            end if;
            declare
               Same, Back : Boolean := True;
            begin
               for G in Before'Range loop
                  Older (G) := Before (G);
                  Before (G) := M.Groups (G).Pushed;
               end loop;
               Measure_Pushes (M);
               for G in Before'Range loop
                  Same := Same and then Flag_Vectors."=" (Before (G), M.Groups (G).Pushed);
                  Back := Back and then Flag_Vectors."=" (Older (G), M.Groups (G).Pushed);
               end loop;
               exit when Same and then Round > 0;
               if Back and then Round > 1 then
                  for G in Before'Range loop
                     for B in 0 .. Natural (M.Groups (G).Pushed.Length) - 1 loop
                        if B < Natural (Before (G).Length) and then Before (G) (B) then
                           M.Groups (G).Pushed.Replace_Element (B, True);
                        end if;
                     end loop;
                  end loop;
                  Measure_Noise (M);
                  exit;
               end if;
            end;
         end loop;
      end;
   end Measure;

   function Visible (M : Model; G : Group_Id; D : Real_Array) return Boolean is
      Rest    : Real_Array := D;
      Watched : Natural := 0;   --  channels an eye watches, their steps below what it sees
   begin
      for C in Rest'Range loop
         declare
            V : constant Estimate := Visible_Step (M, G, C);
         begin
            if Known (V) then
               --  A channel an eye watches moves when its change is a step the
               --  eye can see; below that no eye can tell, however the held
               --  reading jitters.
               if abs Rest (C) >= V.Value then
                  return True;
               end if;
               Rest (C) := 0.0;
               Watched := Watched + 1;
            end if;
         end;
      end loop;
      --  The others against their noise: the change of two readings has
      --  twice the variance of one.
      return Watched < Rest'Length and then Significant_Change (M, G, Rest, Sqrt (2.0));
   end Visible;

   function Moving (M : Model; G : Group_Id; Beat : Natural) return Boolean is
   begin
      if Beat = 0 or else not Has_Reading (M, G, Beat) or else not Has_Reading (M, G, Beat - 1) then
         return False;
      end if;
      declare
         D : Real_Array (1 .. M.Groups (G).Size);
      begin
         for C in D'Range loop
            D (C) := Change (M, G, Beat, C);
         end loop;
         return Visible (M, G, D);
      end;
   end Moving;

end Driver.Robot.Channels;
