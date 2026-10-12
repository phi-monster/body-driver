with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Robot.Channels;
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
      --  A push that asks for less than the one test of motion sees is not judged by what the group did: whatever
      --  moved it meanwhile (a press let go, an arm relaxing from what it pressed) was not the push, there is no
      --  delivery to fall short of, and nothing says it was answered or not. It is neither blocked nor a free
      --  push, and what it delivered stays unknown.
      if Settled and then E.Length > 0.0 and then Channels.Has_Reading (M, G, Beat)
        and then Channels.Visible (M, G, Along_Ask (S, E.Length, E.Length))
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
                  if Answered and then not E.Blocked then
                     S.Free_Shortfalls.Append (Short);
                  end if;
               end;
            end if;
         end;
      end if;
      S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
   end Finish;

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

   procedure Track (M : in out Model; Beat : Natural) is
      Delay_Beats : Natural;
      Delay_Known : Boolean;
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
                     --  How long a push may wait for its first motion.
                     Wait : constant Natural := (if Delay_Known then Delay_Beats else E.Start);
                  begin
                     if not E.Moved then
                        if Channels.Moving (M, G, Beat) then
                           E.Moved := True;
                           E.Moved_At := Beat;
                           S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
                           Follow (M, G, Beat, Wait);
                        elsif Beat - E.Start > Wait then
                           Finish (M, G, Beat, Answered => False, Settled => True, Rested => True);
                        end if;
                     elsif not Channels.Moving (M, G, Beat) then
                        Finish (M, G, Beat, Answered => True, Settled => True, Rested => True);
                     else
                        Follow (M, G, Beat, Wait);
                     end if;
                  end;
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
