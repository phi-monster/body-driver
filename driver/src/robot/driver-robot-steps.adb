with Ada.Numerics.Long_Elementary_Functions;
with Driver.Stats;
with Driver.Robot.Channels;

package body Driver.Robot.Steps is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  How much two free pushes in a row differ in shortfall: the robust sigma
   --  of every consecutive pair's difference, which is what a push's shortfall
   --  minus the last free one's varies by when it too moves freely (the
   --  readings' noise included), and the degrees of freedom it rests on.
   procedure Pair_Scatter (Shortfalls : Real_Vectors.Vector; Sigma : out Real; Freedom : out Natural)
     with Pre => Natural (Shortfalls.Length) > 2
   is
      Pairs : Real_Array (1 .. Natural (Shortfalls.Length) - 1);
   begin
      for K in Pairs'Range loop
         Pairs (K) := Shortfalls (K) - Shortfalls (K - 1);
      end loop;
      Sigma := Driver.Stats.Robust_Sigma (Pairs);
      Freedom := Channels.Mad_Degrees_Of_Freedom (Pairs'Length);
   end Pair_Scatter;

   --  Closes the push under way at Beat and judges it. Answered is False for
   --  a push the reading never moved for; Settled is False for one the next
   --  push cut short, which is not judged.
   procedure Finish (M : in out Model; G : Group_Id; Beat : Natural; Answered, Settled : Boolean) is
      S : Group_Stream renames M.Groups (G);
      E : Episode := S.Episodes.Last_Element;
   begin
      E.Ended := True;
      E.End_At := Beat;
      E.Settled := Settled;
      if Settled and then E.Length > 0.0 and then Channels.Has_Reading (M, G, Beat) then
         declare
            Along, Spread : Real := 0.0;
            Known : Boolean := True;
         begin
            for C in 1 .. S.Size loop
               declare
                  Unit  : constant Real := S.Ask (C - 1) / E.Length;
                  Sigma : constant Real := Channels.Noise (M, G, C);
               begin
                  Along := Along + Unit * (Channels.Reading (M, G, Beat, C) - S.From (C - 1));
                  Known := Known and then Sigma < Real'Last;
                  if Known then
                     Spread := Spread + (Unit * Sigma) ** 2;
                  end if;
               end;
            end loop;
            if Known then
               declare
                  --  A shortfall is the difference of two readings, the one
                  --  before the push and the one where it stopped.
                  Sigma : constant Real := Sqrt (2.0 * Spread);
                  Short : constant Real := E.Length - Along;
               begin
                  E.Shortfall := (Value => Short, Sigma => Sigma, Degrees_Of_Freedom => 0);
                  E.Delivered := (Value => Along / E.Length, Sigma => Sigma / E.Length, Degrees_Of_Freedom => 0);
                  if not Answered then
                     E.Blocked := True;
                  elsif Natural (S.Free_Shortfalls.Length) > 2 then
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
      S.Episodes.Append (E);
   end Start;

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
                     Finish (M, G, Beat, Answered => True, Settled => False);
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
                        elsif Beat - E.Start > Wait then
                           Finish (M, G, Beat, Answered => False, Settled => True);
                        end if;
                     elsif not Channels.Moving (M, G, Beat) or else Channels.Converged (M, G, Beat, E.Length) then
                        Finish (M, G, Beat, Answered => True, Settled => True);
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
