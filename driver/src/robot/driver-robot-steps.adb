with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Steps is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  The step along the push's ask by Amount, one value per channel.
   function Along_Ask (S : Group_Stream; Length, Amount : Real) return Real_Array is
      D : Real_Array (1 .. S.Size);
   begin
      for C in D'Range loop
         D (C) := Amount * S.Ask (C - 1) / Length;
      end loop;
      return D;
   end Along_Ask;

   --  Closes the push under way at Beat and judges it. Answered is False for
   --  a push the reading never moved for; Settled is False for one the next
   --  push cut short, which is not judged; Rested is False for one given up
   --  while its readings kept moving.
   procedure Finish (M : in out Model; G : Group_Id; Beat : Natural; Answered, Settled, Rested : Boolean) is
      S : Group_Stream renames M.Groups (G);
      E : Episode := S.Episodes.Last_Element;
   begin
      E.Ended := True;
      E.End_At := Beat;
      E.Settled := Settled;
      E.Rested := Rested;
      --  What the push executed, channel by channel: a channel that went along its ask by a step its readings tell
      --  (Z of the noise of two readings), and further than it fell short of the ask, executed an ask of that
      --  size, and the least such ask of the channel is kept (Group_Stream.Executed, Executed_Least). Moving at all
      --  is not executing: a sending again by a landing's error asks two or three units, and a servo settling a
      --  tick or two gave the kiwi's arm_b third joint an executed least of 2.4 units where its dead band stops
      --  asks below 19 (R6's second start, 10-14: the sweep's first level 2.449, from the readings' noise alone).
      --  Nor did a push born moving (Episode.Born_Moving) execute anything: its channels were on their way from
      --  the push before, and that motion answers that one (s11 on 10-16, servo seed 3: push 372 began at beat 4161
      --  while joint 2 still moved toward the 816.8 of push 371, at 802; it asked 3.2 to 805.2, read 806, and joint
      --  2's executed least became 3.2 where its landings stood 8 to 12 off: the fit counted those as turns).
      while Natural (S.Executed.Length) < S.Size loop
         S.Executed.Append (Real'Last);
      end loop;
      if E.Length > 0.0 and then not E.Born_Moving and then Channels.Has_Reading (M, G, Beat) then
         for C in 1 .. S.Size loop
            declare
               Ask   : constant Real := S.Ask (C - 1);
               Sigma : constant Real := Channels.Noise (M, G, C);
               Along : constant Real :=
                 (if Ask > 0.0 then 1.0 else -1.0) * (Channels.Reading (M, G, Beat, C) - S.From (C - 1));
            begin
               if Ask /= 0.0 and then Sigma < Real'Last and then Along > 0.0
                 and then Driver.Uncertain.Significant (Along, Sqrt (2.0) * Sigma)
                 and then Along > abs Ask - Along
               then
                  S.Executed.Replace_Element (C - 1, Real'Min (S.Executed (C - 1), abs Ask));
               end if;
            end;
         end loop;
      end if;
      --  A push that asks for less than the one test of motion sees is not judged by what the group did: whatever
      --  moved it meanwhile (a press let go, an arm relaxing from what it pressed) was not the push, there is no
      --  delivery to fall short of, and nothing says it was answered or not. It is neither blocked nor a free
      --  push, and what it delivered stays unknown. Nor is one judged that nothing answered and that asks, of
      --  every channel it asks a step of, less than the body is known to execute (Executed_Least): that nothing
      --  moved says nothing of a push within a servo's dead band (the kiwi's sweep, R5 on 10-14: first levels of
      --  2.4 to 3.8 units, every one called blocked where it stood, and no joint had a keyframe of its own). A
      --  push that was answered is judged whatever it asked: a joint that moved was not in its dead band, and one
      --  stopped half way by an obstacle is blocked. A channel no push has moved yet has no such least, and a
      --  push of it is judged as any: a joint held against its limit from the start answers nothing at any size,
      --  and is blocked.
      if Settled and then E.Length > 0.0 and then Channels.Has_Reading (M, G, Beat)
        and then Channels.Visible (M, G, Along_Ask (S, E.Length, E.Length))
        and then (Answered
                  or else (for some C in 1 .. S.Size =>
                             S.Asks (C - 1)
                             and then (S.Executed (C - 1) >= Real'Last
                                       or else abs S.Ask (C - 1) >= S.Executed (C - 1))))
      then
         declare
            Along, Spread : Real := 0.0;
            Noise_Known   : Boolean := True;
         begin
            for C in 1 .. S.Size loop
               declare
                  Unit  : constant Real := S.Ask (C - 1) / E.Length;
                  Sigma : constant Real := Channels.Noise (M, G, C);
               begin
                  Along := Along + Unit * (Channels.Reading (M, G, Beat, C) - S.From (C - 1));
                  Noise_Known := Noise_Known and then Sigma < Real'Last;
                  if Noise_Known then
                     Spread := Spread + (Unit * Sigma) ** 2;
                  end if;
               end;
            end loop;
            if Noise_Known then
               declare
                  --  A shortfall is the difference of two readings, the one
                  --  before the push and the one where it stopped.
                  Sigma : constant Real := Sqrt (2.0 * Spread);
                  Short : constant Real := E.Length - Along;
                  --  It falls short by more than the group's free pushes do.
                  Falls_Short : Boolean := False;
               begin
                  E.Shortfall := (Value => Short, Sigma => Sigma, Degrees_Of_Freedom => 0);
                  E.Delivered := (Value => Along / E.Length, Sigma => Sigma / E.Length, Degrees_Of_Freedom => 0);
                  if Natural (S.Free_Shortfalls.Length) > 2 then
                     declare
                        --  What the readings of the push, where it began and
                        --  where it stopped, can tell: their wire's resolution.
                        Floor   : constant Real :=
                          Real'Max (Channels.Resolution (M, G, E.Start - 1), Channels.Resolution (M, G, Beat));
                        --  The most any free push of the group fell short by,
                        --  either way, or by as much as the readings could
                        --  tell from none: their noise, and where they repeat
                        --  exactly the resolution of their wire.
                        Largest : Real := Real'Max (Sigma, Floor);
                     begin
                        for F of S.Free_Shortfalls loop
                           Largest := Real'Max (Largest, abs F);
                        end loop;
                        Falls_Short := Short > Driver.Conventions.Z * Largest;
                     end;
                  end if;
                  --  Free motion falls short too, so a push is blocked only by
                  --  falling short by more than the group's own free pushes
                  --  did, and by a shortfall the one test of motion sees,
                  --  spread along the ask as the push was: one an eye could
                  --  tell (where one watches) and the readings' noise could.
                  --  A shortfall no eye can tell from where the push was
                  --  asked to be is none, however exactly the readings tell
                  --  it. A push is blocked as well when nothing answered it:
                  --  it asked what that test sees (all that is judged here),
                  --  so that it did not seem to answer is evidence. One the
                  --  readings went against, by more than that, delivered less
                  --  than nothing: its shortfall is longer than its ask.
                  E.Blocked :=
                    not Answered
                    or else (Falls_Short and then Channels.Visible (M, G, Along_Ask (S, E.Length, Short)));
                  --  A free push's shortfall is where it landed, its readings still: one given up while they still moved
                  --  had not landed, and kept as free it raises the most a free push falls short by, and the shortfall a
                  --  push is blocked by with it (h27, servo seed 3, 10-17: the last pushes of four aims delivered 0.011
                  --  to 0.418 of theirs, two given up still moving, and none was judged blocked: each step "reached").
                  if Answered and then Rested and then not E.Blocked then
                     S.Free_Shortfalls.Append (Short);
                  end if;
               end;
            end if;
         end;
      end if;
      S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
   end Finish;

   function Free_Landing (M : Model; G : Group_Id) return Real is
   begin
      if G > M.Groups.Last_Index or else Natural (M.Groups (G).Free_Shortfalls.Length) <= 2 then
         return 0.0;
      end if;
      declare
         Off : Real_Array (1 .. Natural (M.Groups (G).Free_Shortfalls.Length));
         K   : Natural := 0;
      begin
         for F of M.Groups (G).Free_Shortfalls loop
            K := K + 1;
            Off (K) := abs F;
         end loop;
         return Driver.Stats.Median (Off) + Driver.Conventions.Z * Driver.Stats.Robust_Sigma (Off);
      end;
   end Free_Landing;

   function Executed_Least (M : Model; G : Group_Id; Channel : Positive) return Real is
     (if Channel <= Natural (M.Groups (G).Executed.Length) then M.Groups (G).Executed (Channel - 1) else Real'Last);

   procedure Start (M : in out Model; G : Group_Id; Beat : Natural) is
      S : Group_Stream renames M.Groups (G);
      E : Episode;
      Length, Near : Real := 0.0;
   begin
      S.From.Clear;
      S.Ask.Clear;
      S.Asks.Clear;
      for C in 1 .. S.Size loop
         S.From.Append (Channels.Reading (M, G, Beat - 1, C));
         S.Ask.Append (Channels.Target (M, G, Beat, C) - Channels.Reading (M, G, Beat - 1, C));
         Length := Length + S.Ask.Last_Element ** 2;
         S.Asks.Append (Channels.Visible (M, G, [for I in 1 .. S.Size => (if I = C then S.Ask.Last_Element else 0.0)]));
         if S.Asks.Last_Element then
            Near := Near + S.Ask.Last_Element ** 2;
         end if;
      end loop;
      E.Start := Beat;
      E.Length := Sqrt (Length);
      E.Moved := Channels.Moving (M, G, Beat);
      E.Moved_At := Beat;
      --  Moving at the beat before it started, before this push could act, the group was on its way from the push
      --  before: its motion at this push's start answers that one, not this one, and measured from such pushes
      --  alone the body's delay was 0 (seed 2 of the fake robot, 10-14: each probe level sent a beat after the one
      --  before, the first level's answer credited to the second, and every level thereafter read a beat before
      --  the body had moved). Motion at the start beat itself is not that: a body that answers within the beat
      --  moves at every push's start.
      E.Born_Moving := Beat > 0 and then Channels.Moving (M, G, Beat - 1);
      --  Where it starts from: its whole ask away from its target, nowhere from where it began.
      E.Progress_At := Beat - 1;
      E.Nearest := (Value => Sqrt (Near), others => <>);
      E.Farthest := (Value => 0.0, others => <>);
      S.Episodes.Append (E);
   end Start;

   --  Counts the quantity at the beat among those since its extreme (Welford's running mean and sum of squares).
   procedure Note (X : in out Extreme; Value : Real) is
      Off : constant Real := Value - X.Mean;
   begin
      X.Followed := X.Followed + 1;
      X.Mean := X.Mean + Off / Real (X.Followed);
      X.Spread := X.Spread + Off * (Value - X.Mean);
   end Note;

   --  Whether a quantity of a push went past its extreme by Gain by more
   --  than the group's own chatter makes: held against something, a group
   --  moves about, and the new extreme of that movement is no progress (a
   --  stick-slip against a table sets new extremes for as long as it is
   --  watched, at rarer and rarer beats, each a visible step). The beats
   --  since the extreme, that one included, say how much the quantity moves
   --  about, and a gain is the difference of two of them. With fewer than
   --  two there is no chatter to tell it from: a push that gains at every
   --  beat never has any.
   function Beyond_Chatter (X : Extreme; Gain : Real) return Boolean is
     (X.Followed < 2
      or else Driver.Uncertain.Significant (Gain, Sqrt (2.0 * X.Spread / Real (X.Followed - 1)), X.Followed - 1));

   --  Follows the moving push under way, in the channels it asked a step
   --  the one test of motion sees, and gives it up when its group is plainly
   --  going nowhere: it has come neither nearer its target nor farther from
   --  where it began for as long as it took to come as near or as far as it
   --  did, nor for less than the wait a push of the group may take to answer
   --  (a new target cannot show sooner). Coming nearer or farther is a step
   --  that test sees and the group's own chatter does not make
   --  (Beyond_Chatter). Nearer: a push that swings about its target as it
   --  settles comes nearer at every swing, and is no more given up than one
   --  coming to it from one side. Farther: an arm let go from a press springs
   --  back off what it pressed, away from the hold that let it go, and is
   --  not done until it has. A86's arm 2 was let go by a hold of the readings
   --  two beats before it took effect, while the arm chattered by 3e-3
   --  against the table: the hold asked 1.85e-3, the arm sprang back 0.02 off
   --  it over 30 beats, and moved about by 1e-4 every three beats for the
   --  5,600 beats the run had left. Followed along its ask alone, the push
   --  came nearer at its first beat by more than it asked, and the beats
   --  after, past its target, could never be short of it again: never given
   --  up, it was under way to the end of the run, and the hand waited for its
   --  press as long. A group that moves about, at its target, past it or
   --  short of it, is going nowhere wherever it is. A channel the push asked
   --  nothing that test sees is moved by something else, as Finish judges it.
   procedure Follow (M : in out Model; G : Group_Id; Beat : Natural; Wait : Natural) is
      S         : Group_Stream renames M.Groups (G);
      E         : Episode := S.Episodes.Last_Element;
      Away, Way : Real_Array (1 .. S.Size) := [others => 0.0];
      Near, Far, Asked : Real := 0.0;
      Nearer, Farther  : Boolean;
   begin
      if not Channels.Has_Reading (M, G, Beat) then
         return;
      end if;
      for C in Away'Range loop
         if S.Asks (C - 1) then
            Away (C) := Channels.Reading (M, G, Beat, C) - S.From (C - 1);
            Way (C) := S.Ask (C - 1);
            Near := Near + (S.Ask (C - 1) - Away (C)) ** 2;
            Far := Far + Away (C) ** 2;
            Asked := Asked + S.Ask (C - 1) ** 2;
         end if;
      end loop;
      Near := Sqrt (Near);
      Far := Sqrt (Far);
      Asked := Sqrt (Asked);
      --  Nearer by a step seen along the ask, farther by one seen along the way the group went.
      Nearer := Near < E.Nearest.Value
        and then Channels.Visible (M, G, [for C in Way'Range => Way (C) * (E.Nearest.Value - Near) / Asked])
        and then Beyond_Chatter (E.Nearest, E.Nearest.Value - Near);
      Farther := Far > E.Farthest.Value
        and then Channels.Visible (M, G, [for C in Away'Range => Away (C) * (Far - E.Farthest.Value) / Far])
        and then Beyond_Chatter (E.Farthest, Far - E.Farthest.Value);
      if Nearer then
         E.Nearest := (Value => Near, others => <>);
      end if;
      if Farther then
         E.Farthest := (Value => Far, others => <>);
      end if;
      Note (E.Nearest, Near);
      Note (E.Farthest, Far);
      if Nearer or else Farther then
         E.Progress_At := Beat;
      end if;
      S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
      if Beat - E.Progress_At > Integer'Max (E.Progress_At - E.Start, Wait) then
         Finish (M, G, Beat, Answered => True, Settled => True, Rested => False);
      end if;
   end Follow;

   --  The longest wait from a push to its reading's first motion over every
   --  answered push of the body: a command travels one path to every group,
   --  so a group that never answered yet (one held against its limit) is
   --  waited for as long as any answer took. Known is False before any push
   --  of the body was answered.
   procedure Body_Delay (M : Model; Beats : out Natural; Known : out Boolean) is
   begin
      Beats := 0;
      Known := False;
      for S of M.Groups loop
         if S.Commandable and then S.Delay_Known then
            Beats := Natural'Max (Beats, S.Delay_Beats);
            Known := True;
         end if;
      end loop;
   end Body_Delay;

   --  The beat the body's first push started at: the stream's age then. Before any push is answered, an
   --  unanswered push is waited for that long, the same for every push, not for its own start beat: the waits
   --  used to grow with the stream, each unanswered push waiting as long as the stream had lived, so the
   --  doublings of the first pushes from below a servo's dead band waited 60, 120, 240 and 480 beats (R6's
   --  seventh start on the Pi, 10-14: four asks of 0.6 to 4.9 units executed by nothing, 940 of the boot's
   --  first 1000 beats). A body slower than the first wait answers an earlier push while a later one waits,
   --  and its readings' motion is seen then.
   function First_Push (M : Model) return Natural is
      First : Natural := Natural'Last;
   begin
      for S of M.Groups loop
         if S.Commandable and then not S.Episodes.Is_Empty then
            First := Natural'Min (First, S.Episodes.First_Element.Start);
         end if;
      end loop;
      return (if First = Natural'Last then 0 else First);
   end First_Push;

   procedure Track (M : in out Model; Beat : Natural) is
      Delay_Beats : Natural;
      Delay_Known : Boolean;
      First_Wait  : constant Natural := First_Push (M);

      --  Some channel of the group has left where its push started, by more than its noise explains: a joint that
      --  travels slowly, less in a beat than its readings jitter, is never moving from one beat to the next
      --  (Channels.Moving) yet has gone; read as having gone nowhere at every level, the kiwi's elbow was probed
      --  to 8.8e16 units and called dead (R6's seventh and ninth starts, 10-14).
      function Left_Its_Start (G : Group_Id; Beat : Natural) return Boolean is
         S : Group_Stream renames M.Groups (G);
      begin
         if not Channels.Has_Reading (M, G, Beat) then
            return False;
         end if;
         for C in 1 .. S.Size loop
            if C - 1 < Natural (S.From.Length) then
               declare
                  Noise : constant Real := Channels.Noise (M, G, C);
                  --  Less the creep of the beats since the readings it started from (Channels.Drift): a creeping
                  --  reading left every start by more than its noise within a few beats, and every push of a closer
                  --  held open at its end was answered (f16f on 10-15: the jaw asked open to 7e13 units).
                  Since : constant Natural :=
                    (if S.Episodes.Is_Empty then 0
                     else Beat - Natural'Min (Beat, Natural'Max (1, S.Episodes.Last_Element.Start) - 1));
                  Gone  : constant Real :=
                    abs (Channels.Reading (M, G, Beat, C) - S.From (C - 1) - Channels.Drift (M, G, C) * Real (Since));
               begin
                  if Noise < Real'Last and then Noise > 0.0
                    and then Driver.Uncertain.Significant (Gone, Noise * Sqrt (2.0), Channels.Noise_Freedom (M, G, C))
                  then
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Left_Its_Start;

      --  The push is still closing on its target where the eyes cannot tell it from one beat to the next: its last
      --  two beats went along the ask by shrinking steps, and the run of them, carried on, still goes a step an eye
      --  can see (Channels.Visible). Called at rest there, a push was judged short by the approach it had left: a21 on
      --  10-16, arm 1's free pushes of 24 to 105 units ended 0.12 to 0.46 short and, as their last beats' ratio runs
      --  on, within 0.003 of their targets; a press's first push ended 0.0996 short of 1.69, and 0.02 so. The largest
      --  of those, taken for what the group's free pushes fall short by, let the table's pushes go free (0.31 under Z
      --  of 0.119, and so on up to pushes 1.44, 3.06 and 3.70 short), and the press never met the table. A push that
      --  creeps on by steps that do not shrink (a hand giving into what it presses) is closing on nothing, and is
      --  followed as before.
      function Closing (G : Group_Id; Beat : Natural) return Boolean is
         S : Group_Stream renames M.Groups (G);
         E : constant Episode := S.Episodes.Last_Element;
         Last, Before, Spread : Real := 0.0;
      begin
         if E.Length <= 0.0 or else Beat < E.Start + 2
           or else not (Channels.Has_Reading (M, G, Beat) and then Channels.Has_Reading (M, G, Beat - 1)
                        and then Channels.Has_Reading (M, G, Beat - 2))
         then
            return False;
         end if;
         for C in 1 .. S.Size loop
            declare
               Unit  : constant Real := S.Ask (C - 1) / E.Length;
               Sigma : constant Real := Channels.Noise (M, G, C);
            begin
               if Sigma = Real'Last then
                  return False;
               end if;
               Last := Last + Unit * (Channels.Reading (M, G, Beat, C) - Channels.Reading (M, G, Beat - 1, C));
               Before := Before + Unit * (Channels.Reading (M, G, Beat - 1, C) - Channels.Reading (M, G, Beat - 2, C));
               Spread := Spread + (Unit * Sigma) ** 2;
            end;
         end loop;
         if not (Last > 0.0 and then Last < Before and then Driver.Uncertain.Significant (Last, Sqrt (2.0 * Spread))) then
            return False;
         end if;
         declare
            Ratio : constant Real := Last / Before;
         begin
            return Channels.Visible (M, G, Along_Ask (S, E.Length, Last * Ratio / (1.0 - Ratio)));
         end;
      end Closing;
      --  The group stands still short of its push's target for no longer than it has paused before within a push
      --  that went on toward its target after (Group_Stream.Pause): a servo whose readings come in steps of its wire
      --  stands still a beat or more on its way. s41 (servo seed 6 of the fake robot, 10-17): a press's push of 752
      --  units stood still at beat 16902, was ended there as rested, and went on four beats more to the table with
      --  no new ask; the next step began where the arm passed at 16904, six to twelve units short of the table, the
      --  way back there lay inside the servos' dead band, the arm did not move, and the press was lost. A group that
      --  stands within the least ask the body executes of the target on every channel it was asked (the dead band it
      --  stops anywhere in), or no step an eye can see short of it, has landed: it does not pause there.
      function Pausing (G : Group_Id; Beat : Natural) return Boolean is
         S      : Group_Stream renames M.Groups (G);
         E      : constant Episode := S.Episodes.Last_Element;
         Short  : Real_Array (1 .. S.Size) := [others => 0.0];
         Within : Boolean := True;
         Still  : Natural := 0;
         B      : Natural := Beat;
      begin
         if S.Pause = 0 or else not Channels.Has_Reading (M, G, Beat) then
            return False;
         end if;
         for C in 1 .. S.Size loop
            if S.Asks (C - 1) then
               Short (C) := S.From (C - 1) + S.Ask (C - 1) - Channels.Reading (M, G, Beat, C);
               Within := Within and then Executed_Least (M, G, C) < Real'Last
                 and then abs Short (C) < Executed_Least (M, G, C);
            end if;
         end loop;
         if Within or else not Channels.Visible (M, G, Short) then
            return False;
         end if;
         while B > E.Moved_At and then not Channels.Moving (M, G, B) loop
            Still := Still + 1;
            B := B - 1;
         end loop;
         return Still <= S.Pause;
      end Pausing;

      --  A push ended at rest whose group then comes nearer its target than it ever had, by a step the one test of
      --  motion sees and its chatter does not make, with no new ask, had paused there, not rested: the beats from its
      --  end to that step are a pause of the group (Group_Stream.Pause). Only within as many beats as the push took
      --  from its start to its end: a push is given up on that scale (Follow), and motion later than that is not
      --  its own. A servo hunting a tick about where it stopped comes no nearer than it had, and is no pause.
      procedure Learn_Pause (G : Group_Id; Beat : Natural) is
         S     : Group_Stream renames M.Groups (G);
         E     : constant Episode := S.Episodes.Last_Element;
         Way   : Real_Array (1 .. S.Size) := [others => 0.0];
         Near, Asked : Real := 0.0;
      begin
         if not (E.Ended and then E.Rested and then E.Moved) or else E.Length <= 0.0 or else Beat <= E.End_At
           or else Beat - E.End_At > E.End_At - E.Start or else not Channels.Has_Reading (M, G, Beat)
           or else not Channels.Moving (M, G, Beat)
         then
            return;
         end if;
         for C in 1 .. S.Size loop
            if S.Asks (C - 1) then
               Way (C) := S.Ask (C - 1);
               Near := Near + (S.Ask (C - 1) - (Channels.Reading (M, G, Beat, C) - S.From (C - 1))) ** 2;
               Asked := Asked + S.Ask (C - 1) ** 2;
            end if;
         end loop;
         Near := Sqrt (Near);
         Asked := Sqrt (Asked);
         if Asked > 0.0 and then Near < E.Nearest.Value
           and then Channels.Visible (M, G, [for C in Way'Range => Way (C) * (E.Nearest.Value - Near) / Asked])
           and then Beyond_Chatter (E.Nearest, E.Nearest.Value - Near)
         then
            if Beat - E.End_At > S.Pause then
               S.Pause := Beat - E.End_At;
               Driver.Log.Line (Driver.Log.Robot, "steps: group" & G'Image & " stood still" & S.Pause'Image
                                & " beats within the push of beat" & E.Start'Image & ", short of its target, and went on"
                                & " toward it at beat" & Beat'Image & " with no new ask: a rest short of a target waits"
                                & " that long");
            end if;
         end if;
      end Learn_Pause;
   begin
      Body_Delay (M, Delay_Beats, Delay_Known);
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if M.Groups (G).Commandable then
            declare
               S      : Group_Stream renames M.Groups (G);
               Active : constant Boolean := not S.Episodes.Is_Empty and then not S.Episodes.Last_Element.Ended;
            begin
               if Channels.Asked (M, G, Beat) then
                  if Active then
                     Finish (M, G, Beat, Answered => True, Settled => False, Rested => False);
                  end if;
                  Start (M, G, Beat);
               elsif Active then
                  declare
                     E : Episode := S.Episodes.Last_Element;
                     --  How long a push may wait for its first motion: the body's delay once measured, else as long
                     --  as the stream had lived when the body was first pushed (First_Push).
                     Wait : constant Natural := (if Delay_Known then Delay_Beats else First_Wait);
                  begin
                     if not E.Moved then
                        if Channels.Moving (M, G, Beat) or else Left_Its_Start (G, Beat) then
                           E.Moved := True;
                           E.Moved_At := Beat;
                           S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
                           Follow (M, G, Beat, Natural'Max (Delay_Beats, E.Moved_At - E.Start));
                        elsif Beat - E.Start > Wait then
                           Finish (M, G, Beat, Answered => False, Settled => True, Rested => True);
                        end if;
                     elsif not Channels.Moving (M, G, Beat) and then not Closing (G, Beat) and then not Pausing (G, Beat)
                     then
                        Finish (M, G, Beat, Answered => True, Settled => True, Rested => True);
                     else
                        --  A group standing still within a pause (Pausing) is followed as one under way.
                        --  An answered push is given up by its own progress; the least it is followed for is the
                        --  body's delay where that is measured, else the wait this push itself took to answer:
                        --  not the beats the stream had before it (R4 on the Pi, 10-13: before any delay was
                        --  measured, the ladder's first answered levels, servos hunting a tick about their
                        --  targets, were followed for 266 and 524 beats).
                        Follow (M, G, Beat, Natural'Max (Delay_Beats, E.Moved_At - E.Start));
                     end if;
                  end;
               elsif not S.Episodes.Is_Empty then
                  Learn_Pause (G, Beat);
               end if;
            end;
         end if;
      end loop;
   end Track;

   function Limiter (M : Model; G : Group_Id; E : Episode) return Natural is
      Size : constant Natural := M.Groups (G).Size;
   begin
      if not (E.Ended and then E.Rested and then E.Blocked) or else E.Length <= 0.0 or else E.Start < 1
        or else not Channels.Has_Reading (M, G, E.Start - 1) or else not Channels.Has_Target (M, G, E.Start)
        or else not Channels.Has_Reading (M, G, E.End_At)
      then
         return 0;
      end if;
      declare
         --  The scatter of the group's free pushes, in reading units along their ask, as Finish measures it.
         Largest : Real := 0.0;
         --  What each channel contributed to the push's shortfall along its ask, as Finish weighs the channels: its
         --  share of the ask (the channel's ask over the ask's length) times its own shortfall, and the variance of
         --  that which its own readings bring. A channel asked nothing the one test of motion sees contributes none,
         --  however it moved (a joint that shifts when another stops has a share of the ask of a few thousandths).
         Short, Own : Real_Array (1 .. Size) := [others => 0.0];
         Best    : Natural := 0;
      begin
         if Natural (M.Groups (G).Free_Shortfalls.Length) > 2 then
            for F of M.Groups (G).Free_Shortfalls loop
               Largest := Real'Max (Largest, abs F);
            end loop;
         end if;
         for C in 1 .. Size loop
            declare
               Start : constant Real := Channels.Reading (M, G, E.Start - 1, C);
               Aim   : constant Real := Channels.Target (M, G, E.Start, C);
               Stop  : constant Real := Channels.Reading (M, G, E.End_At, C);
               Sigma : constant Real := Channels.Noise (M, G, C);
               Ask   : constant Real := Aim - Start;
               Unit  : constant Real := Ask / E.Length;
            begin
               if Sigma < Real'Last and then Ask /= 0.0
                 and then Channels.Visible (M, G, [for I in 1 .. Size => (if I = C then Ask else 0.0)])
               then
                  Short (C) := Unit * (Aim - Stop);
                  Own (C) := (Unit * Sqrt (2.0) * Sigma) ** 2;
               end if;
            end;
         end loop;
         --  The channel that fell short by the most along the ask, if it fell short at all: by more than Z deviations
         --  of its own readings and of the scatter of free pushes. That scatter is the whole shortfall's (one number a
         --  push, the sum of the channels' parts), so it bounds a part, and the difference of two parts, once.
         for C in 1 .. Size loop
            if Short (C) > Driver.Conventions.Z * Sqrt (Own (C) + Largest ** 2)
              and then (Best = 0 or else Short (C) > Short (Best))
            then
               Best := C;
            end if;
         end loop;
         if Best = 0 then
            return 0;
         end if;
         --  And by more than every other channel: the two channels' readings apart, and the scatter once.
         for C in 1 .. Size loop
            if C /= Best and then Short (Best) - Short (C) <= Driver.Conventions.Z * Sqrt (Own (Best) + Own (C) + Largest ** 2) then
               return 0;
            end if;
         end loop;
         return Best;
      end;
   end Limiter;

   function Episodes (M : Model; G : Group_Id) return Natural is
     (if G <= M.Groups.Last_Index then Natural (M.Groups (G).Episodes.Length) else 0);

   function Latest (M : Model; G : Group_Id) return Episode is (M.Groups (G).Episodes.Last_Element);

end Driver.Robot.Steps;
