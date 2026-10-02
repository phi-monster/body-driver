with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Stats;
with Driver.Robot.Channels;

package body Driver.Robot.Steps is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  Everything sized by beats or pushes lives on the heap: the estimates
   --  also run in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   --  How much two free pushes in a row differ in shortfall: the robust sigma
   --  of every consecutive pair's difference, which is what a push's shortfall
   --  minus the last free one's varies by when it too moves freely (the
   --  readings' noise included), and the degrees of freedom it rests on.
   procedure Pair_Scatter (Shortfalls : Real_Vectors.Vector; Sigma : out Real; Freedom : out Natural)
     with Pre => Natural (Shortfalls.Length) > 2
   is
      Pairs : Real_Access := new Real_Array (1 .. Natural (Shortfalls.Length) - 1);
   begin
      for K in Pairs'Range loop
         Pairs (K) := Shortfalls (K) - Shortfalls (K - 1);
      end loop;
      Sigma := Driver.Stats.Robust_Sigma (Pairs.all);
      Freedom := Channels.Mad_Degrees_Of_Freedom (Pairs'Length);
      Free (Pairs);
   end Pair_Scatter;

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
      if Settled and then E.Length > 0.0 and then Channels.Has_Reading (M, G, Beat) then
         declare
            Along, Spread : Real := 0.0;
            Noise_Known   : Boolean := True;
            --  A channel an eye watches stopped short of its ask by a step
            --  that eye can see.
            Seen_Short : Boolean := False;
            --  The ask and what came of it along the channels no eye watches,
            --  which are judged against the free pushes.
            Unwatched_Length, Unwatched_Along : Real := 0.0;
         begin
            for C in 1 .. S.Size loop
               declare
                  Ask   : constant Real := S.Ask (C - 1);
                  Unit  : constant Real := Ask / E.Length;
                  Sigma : constant Real := Channels.Noise (M, G, C);
                  Got   : constant Real := Channels.Reading (M, G, Beat, C) - S.From (C - 1);
                  V     : constant Estimate := Visible_Step (M, G, C);
               begin
                  Along := Along + Unit * Got;
                  Noise_Known := Noise_Known and then Sigma < Real'Last;
                  if Noise_Known then
                     Spread := Spread + (Unit * Sigma) ** 2;
                  end if;
                  if Known (V) then
                     if Ask /= 0.0 and then (Ask - Got) * (if Ask > 0.0 then 1.0 else -1.0) >= V.Value then
                        Seen_Short := True;
                     end if;
                  else
                     Unwatched_Length := Unwatched_Length + Unit * Ask;
                     Unwatched_Along := Unwatched_Along + Unit * Got;
                  end if;
               end;
            end loop;
            if Noise_Known then
               declare
                  --  A shortfall is the difference of two readings, the one
                  --  before the push and the one where it stopped.
                  Sigma : constant Real := Sqrt (2.0 * Spread);
                  Short : constant Real := E.Length - Along;
               begin
                  E.Shortfall := (Value => Short, Sigma => Sigma, Degrees_Of_Freedom => 0);
                  E.Delivered := (Value => Along / E.Length, Sigma => Sigma / E.Length, Degrees_Of_Freedom => 0);
               end;
               --  The channels an eye watches are judged by what the eye could
               --  tell, answered or not: a push smaller than their visible step
               --  moves nothing any eye can see, so that it did not seem to
               --  answer says nothing. The others are blocked when nothing
               --  answered them, or when they fell short by more than free
               --  pushes do.
               if Seen_Short then
                  E.Blocked := True;
               elsif Unwatched_Length > 0.0 and then not Answered then
                  E.Blocked := True;
               elsif Unwatched_Length > 0.0 then
                  declare
                     Short : constant Real := Unwatched_Length - Unwatched_Along;
                  begin
                     if Natural (S.Free_Shortfalls.Length) > 2 then
                        declare
                           Excess  : constant Real := Short - S.Free_Shortfalls.Last_Element;
                           Scatter : Real;
                           Freedom : Natural;
                        begin
                           Pair_Scatter (S.Free_Shortfalls, Scatter, Freedom);
                           E.Blocked := Excess > 0.0 and then Driver.Uncertain.Significant (Excess, Scatter, Freedom);
                        end;
                     end if;
                     if not E.Blocked then
                        S.Free_Shortfalls.Append (Short);
                     end if;
                  end;
               end if;
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

   --  The step along the push's ask by Amount, one value per channel.
   function Along_Ask (S : Group_Stream; Length, Amount : Real) return Real_Array is
      D : Real_Array (1 .. S.Size);
   begin
      for C in D'Range loop
         D (C) := Amount * S.Ask (C - 1) / Length;
      end loop;
      return D;
   end Along_Ask;

   --  Follows how close the moving push under way has come to its target,
   --  and gives it up when it is plainly going nowhere: still short of its
   --  target by a step the motion test would see, it has not come closer for
   --  as long as it took to come as close as it did, nor for less than the
   --  wait a push of the group may take to answer (a new target cannot show
   --  sooner).
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
      if Along > E.Closest and then Channels.Visible (M, G, Along_Ask (S, E.Length, Along - E.Closest)) then
         E.Closest := Along;
         E.Closest_At := Beat;
         S.Episodes.Replace_Element (S.Episodes.Last_Index, E);
      elsif E.Closest < E.Length and then Channels.Visible (M, G, Along_Ask (S, E.Length, E.Length - E.Closest))
        and then Beat - E.Closest_At > Integer'Max (E.Closest_At - E.Start, Wait)
      then
         Finish (M, G, Beat, Answered => True, Settled => True, Rested => False);
      end if;
   end Follow;

   procedure Track (M : in out Model; Beat : Natural) is
   begin
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
                     Wait : constant Natural := (if S.Delay_Known then S.Delay_Beats else E.Start);
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
