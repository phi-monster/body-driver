with Driver.Log;
with Ada.Containers;
with Ada.Containers.Ordered_Maps;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
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
               S.Twin_Values.Prepend (0.0, Ada.Containers.Count_Type (S.Size * Earlier));
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
            --  The pair of this group's command key was found the other way round at this beat (Driver.Twins): what
            --  was kept for its readings was its echo, and the echo kept was its readings. Both turn round, and the
            --  range seen with them; the estimates are redone at once.
            if G <= O.Turned.Last_Index and then O.Turned (G)
              and then Natural (S.Twin_Values.Length) = Natural (S.Values.Length)
            then
               declare
                  Kept      : constant Real_Vectors.Vector := S.Values;
                  Kept_Bits : constant Count_Vectors.Vector := S.Precisions;
               begin
                  S.Values := S.Twin_Values;
                  S.Twin_Values := Kept;
                  S.Precisions := S.Twin_Precisions;
                  S.Twin_Precisions := Kept_Bits;
               end;
               S.Low_Seen.Clear;
               S.High_Seen.Clear;
               for B in 0 .. Natural (S.Present.Length) - 1 loop
                  if S.Present (B) then
                     for C in 0 .. S.Size - 1 loop
                        declare
                           V : constant Real := S.Values (B * S.Size + C);
                        begin
                           if Natural (S.Low_Seen.Length) < S.Size then
                              S.Low_Seen.Append (V);
                              S.High_Seen.Append (V);
                           else
                              S.Low_Seen.Replace_Element (C, Real'Min (S.Low_Seen (C), V));
                              S.High_Seen.Replace_Element (C, Real'Max (S.High_Seen (C), V));
                           end if;
                        end;
                     end loop;
                  end if;
               end loop;
               M.Due := True;
            end if;
            --  The wire format of a reading that did not come this beat is the last one's.
            if G <= O.Echoes.Last_Index and then O.Echoes.Element (G)'Length = S.Size then
               for V of O.Echoes.Element (G) loop
                  S.Twin_Values.Append (V);
               end loop;
               S.Twin_Precisions.Append (Driver.Observations.Echo_Precision_Of (O, G));
            else
               S.Twin_Values.Append (0.0, Ada.Containers.Count_Type (S.Size));
               S.Twin_Precisions.Append
                 (if S.Twin_Precisions.Is_Empty then Real'Machine_Mantissa else S.Twin_Precisions.Last_Element);
            end if;
            S.Present.Append (Have);
            S.Precisions.Append
              (if Have then Driver.Observations.Precision_Of (O, G)
               elsif S.Precisions.Is_Empty then Real'Machine_Mantissa else S.Precisions.Last_Element);
            if Have then
               for V of O.Readings.Element (G) loop
                  S.Values.Append (V);
               end loop;
               --  The readings the channels have moved through.
               if Natural (S.Low_Seen.Length) /= S.Size or else Natural (S.High_Seen.Length) /= S.Size then
                  S.Low_Seen.Clear;
                  S.High_Seen.Clear;
                  for V of O.Readings.Element (G) loop
                     S.Low_Seen.Append (V);
                     S.High_Seen.Append (V);
                  end loop;
               else
                  for C in 0 .. S.Size - 1 loop
                     declare
                        V : constant Real := O.Readings.Element (G) (O.Readings.Element (G)'First + C);
                     begin
                        S.Low_Seen.Replace_Element (C, Real'Min (S.Low_Seen (C), V));
                        S.High_Seen.Replace_Element (C, Real'Max (S.High_Seen (C), V));
                     end;
                  end loop;
               end if;
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

   function Mad_Degrees_Of_Freedom (N : Natural) return Natural renames Driver.Stats.Mad_Degrees_Of_Freedom;

   --  Whether the change into Beat is one of a resting reading: the beat and
   --  the one before have readings, and no push of any commanded group was
   --  under way. A reading moves as a reaction to a push of something else
   --  too (a sensor of what the arms do: the rig's sum of two joints), and
   --  the beats of other groups' pushes are its motion, not its rest: counted
   --  as rest, a long sweep was the latest regime of such a reading and its
   --  reactions became its noise, so nothing it did was visible any more
   --  (robot.roles, 10-12). A target that changed is no push unless it asked
   --  for motion (Asked): a group the driver never commanded is held by its
   --  own reading, sent again every beat, and that target changes at every
   --  beat by the reading's creep.
   function At_Rest (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (Beat > 0 and then Has_Reading (M, G, Beat) and then Has_Reading (M, G, Beat - 1)
      and then (for all H in M.Groups.First_Index .. M.Groups.Last_Index =>
                  not M.Groups (H).Commandable or else not Pushed (M, H, Beat)));

   --  One channel's noise from its rest changes. A resting reading either
   --  repeats exactly (a simulator between physics updates, a quantized or
   --  echoed value) or changes by its jitter; the changes that are not exact
   --  repeats measure the jitter. A channel that repeats exactly at every rest
   --  beat has no noise beyond its quantization (the floor below).
   procedure Measure_Channel (M : in out Model; G : Group_Id; Channel : Positive; Rest : Natural) is
      --  Every nonzero change at rest, and its negation: a change at rest has
      --  no sign of its own, so its scale is measured about zero. About their
      --  own median, two-valued changes (a reading whose last bit flips back
      --  and forth, one way more often than the other) leave no spread at all.
      Moved : Real_Access := new Real_Array (1 .. 2 * Rest);
      K     : Natural := 0;
      Index : constant Natural := First_Channel (M, G) + Channel - 1;
      --  Every change at rest in beat order, zeros included, to tell a reading
      --  that drifts from one that rests: a resting reading's changes have no
      --  trend; a joint still travelling (R0, 10-11: the sticky wrist servo
      --  crawling 660 ticks towards a stale goal after torque came on, 25
      --  beats into the boot's rest) shows a mean change significant against
      --  the changes' scatter. The oldest half is dropped while a trend shows;
      --  the noise is measured on the trend-free rest that remains, and is
      --  unknown when none remains, so the boot holds still on.
      All_Changes : Real_Access := new Real_Array (1 .. Rest);
      Count       : Natural := 0;
      Start       : Positive := 1;
   begin
      for B in 1 .. Natural (M.Groups (G).Present.Length) - 1 loop
         if At_Rest (M, G, B) then
            Count := Count + 1;
            All_Changes (Count) := Change (M, G, B, Channel);
         end if;
      end loop;
      --  Rest is stationary: the older half of the rest changes and the newer
      --  half have the same mean, within their scatter. A reading that drifts
      --  at one rate for good (a sensor's creep, A15) is stationary too, and
      --  its creep stays in its noise as before; a joint still travelling, or
      --  one that stopped travelling, has halves that differ, and the older is
      --  dropped until the rest that remains is one regime (Stats.Latest_Regime).
      --  Too short to split, it is taken as it is: the boot compares the noise
      --  across a doubling of the rest's beats before it trusts it.
      if Count >= Start then
         Start := Driver.Stats.Latest_Regime (All_Changes (Start .. Count));
      end if;
      if Count - Start + 1 < 2 and then Count >= 2 then
         --  Trending to its last beats: nothing of its rest is measured yet.
         M.Noise.Replace_Element (Index, Real'Last);
         M.Noise_Freedom.Replace_Element (Index, 0);
         Free (All_Changes);
         Free (Moved);
         return;
      end if;
      for I in Start .. Count loop
         if All_Changes (I) /= 0.0 then
            K := K + 1;
            Moved (K) := All_Changes (I);
            Moved (Rest + K) := -All_Changes (I);
         end if;
      end loop;
      Free (All_Changes);
      if K = 0 then
         M.Noise.Replace_Element (Index, 0.0);
         M.Noise_Freedom.Replace_Element (Index, 0);
      else
         Moved (K + 1 .. 2 * K) := Moved (Rest + 1 .. Rest + K);
         M.Noise.Replace_Element (Index, Driver.Stats.Robust_Sigma (Moved (1 .. 2 * K)) / Sqrt (2.0));
         M.Noise_Freedom.Replace_Element (Index, Mad_Degrees_Of_Freedom (K));
      end if;
      --  A reading that comes in whole units of its wire (an integer: a servo's ticks) is known no closer than
      --  half a unit either way: its noise is at least that of rounding to the unit, the unit over the root of
      --  twelve (the standard deviation of a uniform spread over one unit), however exactly it repeats at rest.
      --  Without it a reading that repeated exactly had a noise of nothing, so any single-tick flip was motion,
      --  asked for, or an answer: on the Pi (R3 and R4, 10-13) the held arm's ticks flipped when the other arm's
      --  push shook the base, pushes were taken to be answered 194, 293 and 444 beats late, and every look
      --  thereafter waited that long. A float's unit is its last bit: what repeats exactly there is kept exact
      --  (the rules on Resolution in Steps and Smallest_Step tell such a reading from none).
      declare
         Last : constant Natural := Natural (M.Groups (G).Present.Length) - 1;
      begin
         if Last < Natural (M.Groups (G).Precisions.Length) and then M.Groups (G).Precisions (Last) = 0 then
            declare
               Quantum : constant Real := Resolution (M, G, Last) / Sqrt (12.0);
            begin
               if M.Noise (Index) < Quantum then
                  M.Noise.Replace_Element (Index, Quantum);
               end if;
            end;
         end if;
      end;
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

   function Resolution (M : Model; G : Group_Id; Beat : Natural) return Real is
      Bits  : constant Natural := M.Groups (G).Precisions (Beat);
      Scale : Real := 0.0;
   begin
      if Bits = 0 then
         --  An integer on the wire: it changes by one unit or not at all.
         return 1.0;
      end if;
      for C in 1 .. M.Groups (G).Size loop
         Scale := Real'Max (Scale, abs Reading (M, G, Beat, C));
      end loop;
      return Real'Scaling (1.0, Real'Exponent (if Scale > 0.0 then Scale else 1.0) - Bits);
   end Resolution;

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
   --  The gate of a change of so many channels at so many degrees of freedom, searched once: a quantile is a pure
   --  function of its arguments, and the search is an inversion of a tail that takes thousands of operations. The
   --  lock-in asks at every beat of every group when it marks the usable beats (12 000 beats, 7 groups, 3 eyes: the
   --  marks were most of a 14 s recompute on a fake body), the stillness watch at every beat. Kept under a lock:
   --  the callers run on many tasks.
   type Gate_Key is record
      Tested, Freedom : Natural;
   end record;
   function "<" (A, B : Gate_Key) return Boolean is
     (A.Tested < B.Tested or else (A.Tested = B.Tested and then A.Freedom < B.Freedom));
   package Gate_Maps is new Ada.Containers.Ordered_Maps (Gate_Key, Driver.Uncertain.Gate, "<", Driver.Uncertain."=");
   protected Gates_Known is
      procedure Get (Key : Gate_Key; Value : out Driver.Uncertain.Gate; Found : out Boolean);
      procedure Keep (Key : Gate_Key; Value : Driver.Uncertain.Gate);
   private
      Map : Gate_Maps.Map;
   end Gates_Known;
   protected body Gates_Known is
      procedure Get (Key : Gate_Key; Value : out Driver.Uncertain.Gate; Found : out Boolean) is
         C : constant Gate_Maps.Cursor := Map.Find (Key);
      begin
         Found := Gate_Maps.Has_Element (C);
         if Found then
            Value := Gate_Maps.Element (C);
         end if;
      end Get;
      procedure Keep (Key : Gate_Key; Value : Driver.Uncertain.Gate) is
      begin
         Map.Include (Key, Value);
      end Keep;
   end Gates_Known;

   function Change_Gate (Tested : Positive; Freedom : Natural) return Driver.Uncertain.Gate is
      Key   : constant Gate_Key := (Tested, Freedom);
      Value : Driver.Uncertain.Gate;
      Found : Boolean;
   begin
      Gates_Known.Get (Key, Value, Found);
      if not Found then
         Value := Driver.Uncertain.Vector_Gate (Tested, Freedom);
         Gates_Known.Keep (Key, Value);
      end if;
      return Value;
   end Change_Gate;

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
          (Change_Gate (Tested, (if Freedom = Natural'Last then 0 else Freedom)), Sqrt (Q), 1.0);
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
               --  The response delay: the longest wait from a push's start to its reading's first motion, over
               --  the pushes that delivered what they asked (Steps' episodes: answered, ended, not blocked, and
               --  their delivered fraction significant). A push into a stop delivers nothing and may still see its
               --  reading creep when the servo yields after seconds (R4 on the Pi, 10-13: arm a's joint 4 asked
               --  29 ticks against its end moved after 75 beats; hand a's closed jaws after 77; hand b's after
               --  865), and a tick's flip when the other arm shakes the base is no answer either (R3: 194, 444):
               --  taken for responses, they made every look thereafter wait that long. The command's path is
               --  measured where the body followed the command.
               for E of M.Groups (G).Episodes loop
                  if E.Moved and then E.Ended and then not E.Blocked and then Known (E.Delivered)
                    and then E.Delivered.Value > 0.0
                    and then Driver.Uncertain.Significant (E.Delivered.Value, E.Delivered.Sigma, E.Delivered.Degrees_Of_Freedom)
                    and then E.Moved_At >= E.Start
                  then
                     Delay_Beats := Natural'Max (Delay_Beats, E.Moved_At - E.Start);
                     Answered := True;
                  end if;
               end loop;
               --  Logged when the longest wait grows (R0: a line every beat as the window slid over pushes of two to
               --  eight beats).
               if Answered and then (not M.Groups (G).Delay_Known or else Delay_Beats > M.Groups (G).Delay_Beats) then
                  Driver.Log.Line (Driver.Log.Robot, "group" & G'Image & " answers a push within" & Delay_Beats'Image
                                   & " beats, the longest wait of its pushes from their start to its readings' first motion");
               end if;
               M.Groups (G).Delay_Beats := Delay_Beats;
               M.Groups (G).Delay_Known := Answered;
            end if;
            declare
               --  The visible steps once per group and round, not once per beat: Steps_Of sums every eye's cells
               --  for every channel, and asked at every beat of every round it was most of a measurement (R4 on
               --  the Pi, 10-13: the channels' phase 389 s at 4467 beats; the lock-in takes the same table once,
               --  item 2).
               Steps : constant Real_Array := Steps_Of (M, G);
            begin
               for B in 0 .. Beats - 1 loop
                  if M.Groups (G).Commandable and then Asked (M, G, B) then
                     Active := True;
                     Onset := B;
                     Moved := Moving (M, G, B, Steps);
                  elsif Active then
                     --  Until the reading first moves the push waits out the
                     --  delay; after that its response lasts, overshoot and all,
                     --  until the reading is still. A push that is not answered
                     --  within the delay is over.
                     if not Moved then
                        Moved := Moving (M, G, B, Steps);
                        if not Moved and then B - Onset >= Delay_Beats then
                           Active := False;
                        end if;
                     elsif not Moving (M, G, B, Steps) then
                        Active := False;
                     end if;
                  end if;
                  Marks.Append (Active);
               end loop;
            end;
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
      --  A noise reloaded from a body file stands; the pushes are marked
      --  against it.
      if M.From_File (Stored_Noise) then
         Measure_Pushes (M);
         return;
      end if;
      --  The noise is measured at rest, and rest is where no push is under
      --  way, which takes the noise to find: the two are measured in turn
      --  until the pushes found no longer change. The first round takes every
      --  change for motion (noise zero), so a push lasts as long as its
      --  reading changes at all and the tail of a slow response is never
      --  taken for rest; the noise measured outside those pushes is then the
      --  jitter alone, or zero where a reading has none. From the second
      --  round on a beat can only be released from the pushes, never taken
      --  back: a change once judged jitter against a noise measured on more
      --  rest stays jitter, the rest the noise is measured on only grows, and
      --  the rounds end as soon as no beat is released. (Marks that could go
      --  both ways alternated: a beat right after a push whose change is
      --  significant against the noise measured with it among the rest
      --  beats, and not against the noise measured without it, was rest one
      --  round and pushed the next, for good; A9's replay spent 540 s in such
      --  rounds at 1024 beats, R3 replayed 1489 rounds at 1488 beats, and R4
      --  on the Pi, 10-13, 4468 rounds and 389 s at 4467 beats, the boot
      --  waiting for the estimate throughout.)
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
         --  The marks of the round before.
         type Marks is array (M.Groups.First_Index .. M.Groups.Last_Index) of Flag_Vectors.Vector;
         Before : Marks;
      begin
         for Round in 0 .. Beats loop
            M.Measure_Rounds := Round + 1;
            if Round > 0 then
               Measure_Noise (M);
            end if;
            declare
               Same : Boolean := True;
            begin
               for G in Before'Range loop
                  Before (G) := M.Groups (G).Pushed;
               end loop;
               Measure_Pushes (M);
               if Round > 0 then
                  --  A beat released from the pushes stays released.
                  for G in Before'Range loop
                     for B in 0 .. Natural (M.Groups (G).Pushed.Length) - 1 loop
                        if B < Natural (Before (G).Length) and then not Before (G) (B) then
                           M.Groups (G).Pushed.Replace_Element (B, False);
                        end if;
                     end loop;
                  end loop;
               end if;
               for G in Before'Range loop
                  Same := Same and then Flag_Vectors."=" (Before (G), M.Groups (G).Pushed);
               end loop;
               exit when Same and then Round > 0;
            end;
         end loop;
      end;
   end Measure;

   function Visible (M : Model; G : Group_Id; D : Real_Array; Steps : Real_Array) return Boolean is
      Seen : Real_Array := D;   --  the changes an eye could tell
   begin
      for C in Seen'Range loop
         --  A channel an eye watches changed by less than that eye can see
         --  is no change: below that no eye can tell, however the held
         --  reading jitters.
         if Steps (Steps'First + C - Seen'First) < Real'Last and then abs Seen (C) < Steps (Steps'First + C - Seen'First)
         then
            Seen (C) := 0.0;
         end if;
      end loop;
      --  What an eye could tell is motion when it is also significant against
      --  the noise: the change of two readings has twice the variance of one.
      --  A visible step is no evidence of its own (a lock-in that credits a
      --  group with the pictures' motion beside its tiny readings gives steps
      --  of 1e-17); a noise that is not measured gives none either.
      return Significant_Change (M, G, Seen, Sqrt (2.0));
   end Visible;

   function Steps_Of (M : Model; G : Group_Id) return Real_Array is
      Steps : Real_Array (1 .. M.Groups (G).Size) := [others => Real'Last];
   begin
      for C in Steps'Range loop
         declare
            V : constant Estimate := Visible_Step (M, G, C);
         begin
            if Known (V) then
               Steps (C) := V.Value;
            end if;
         end;
      end loop;
      return Steps;
   end Steps_Of;

   function Visible (M : Model; G : Group_Id; D : Real_Array) return Boolean is
     (Visible (M, G, D, Steps_Of (M, G)));

   function Moving (M : Model; G : Group_Id; Beat : Natural; Steps : Real_Array) return Boolean is
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
         return Visible (M, G, D, Steps);
      end;
   end Moving;

   function Moving (M : Model; G : Group_Id; Beat : Natural) return Boolean is
     (Moving (M, G, Beat, Steps_Of (M, G)));

end Driver.Robot.Channels;
