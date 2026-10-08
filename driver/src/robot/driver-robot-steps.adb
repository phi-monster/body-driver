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
                        --  What the float arithmetic on the readings of the push,
                        --  where it began and where it stopped, can tell.
                        Floor   : constant Real :=
                          Real'Max (Channels.Resolution (M, G, E.Start - 1), Channels.Resolution (M, G, Beat));
                        --  The most any free push of the group fell short by,
                        --  either way, or by as much as the readings could
                        --  tell from none: their noise, and where they repeat
                        --  exactly the resolution of the float.
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
      Length : Real := 0.0;
   begin
      S.From.Clear;
      S.Ask.Clear;
      for C in 1 .. S.Size loop
         S.From.Append (Channels.Reading (M, G, Beat - 1, C));
         S.Ask.Append (Channels.Target (M, G, Beat, C) - Channels.Reading (M, G, Beat - 1, C));
         Length := Length + S.Ask.Last_Element ** 2;
      end loop;
      E.Start := Beat;
      E.Length := Sqrt (Length);
      E.Moved := Channels.Moving (M, G, Beat);
      E.Moved_At := Beat;
      --  Where it starts from: nothing of the ask made yet.
      E.Closest_At := Beat - 1;
      E.Closest := 0.0;
      S.Episodes.Append (E);
   end Start;

   --  Counts the beat's progress along the ask among those since the push
   --  came closest to its target (Welford's running mean and sum of squares).
   procedure Note (E : in out Episode; Along : Real) is
      Off : constant Real := Along - E.Mean_Along;
   begin
      E.Followed := E.Followed + 1;
      E.Mean_Along := E.Mean_Along + Off / Real (E.Followed);
      E.Spread_Along := E.Spread_Along + Off * (Along - E.Mean_Along);
   end Note;

   --  Whether a push came closer to its target by Gain than its closest point
   --  was, by more than its own chatter makes: held against something, a push
   --  moves about, and the new extreme of that movement is no progress (a
   --  stick-slip against a table sets new extremes for as long as it is
   --  watched, at rarer and rarer beats, each a visible step). The beats
   --  since its closest point, that one included, say how much it moves
   --  about, and a gain is the difference of two of them. With fewer than
   --  two there is no chatter to tell it from: a push that comes closer at
   --  every beat never has any.
   function Beyond_Chatter (E : Episode; Gain : Real) return Boolean is
     (E.Followed < 2
      or else Driver.Uncertain.Significant
                (Gain, Sqrt (2.0 * E.Spread_Along / Real (E.Followed - 1)), E.Followed - 1));

   --  Follows how close the moving push under way has come to its target,
   --  and gives it up when it is plainly going nowhere: still short of its
   --  target by a step the motion test would see, it has not come closer for
   --  as long as it took to come as close as it did, nor for less than the
   --  wait a push of the group may take to answer (a new target cannot show
   --  sooner). Coming closer is a step the motion test sees and its own
   --  chatter does not make (Beyond_Chatter).
   procedure Follow (M : in out Model; G : Group_Id; Beat : Natural; Wait : Natural) is
      S     : Group_Stream renames M.Groups (G);
      E     : Episode := S.Episodes.Last_Element;
      Along : Real := 0.0;
   begin
      if E.Length = 0.0 or else not Channels.Has_Reading (M, G, Beat) then
         return;
      end if;
      for C in 1 .. S.Size loop
         Along := Along + S.Ask (C - 1) / E.Length * (Channels.Reading (M, G, Beat, C) - S.From (C - 1));
      end loop;
      if Along > E.Closest and then Channels.Visible (M, G, Along_Ask (S, E.Length, Along - E.Closest))
        and then Beyond_Chatter (E, Along - E.Closest)
      then
         E.Closest := Along;
         E.Closest_At := Beat;
         E.Followed := 0;
         E.Mean_Along := 0.0;
         E.Spread_Along := 0.0;
         Note (E, Along);
         S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
      else
         Note (E, Along);
         S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
         if E.Closest < E.Length and then Channels.Visible (M, G, Along_Ask (S, E.Length, E.Length - E.Closest))
           and then Beat - E.Closest_At > Integer'Max (E.Closest_At - E.Start, Wait)
         then
            Finish (M, G, Beat, Answered => True, Settled => True, Rested => False);
         end if;
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

   function Episodes (M : Model; G : Group_Id) return Natural is
     (if G <= M.Groups.Last_Index then Natural (M.Groups (G).Episodes.Length) else 0);

   function Latest (M : Model; G : Group_Id) return Episode is (M.Groups (G).Episodes.Last_Element);

end Driver.Robot.Steps;
