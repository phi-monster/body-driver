with Ada.Containers;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Stats;

package body Driver.Robot.Channels is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

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

   procedure Measure_Noise (M : in out Model) is
      Total : Natural := 0;
   begin
      for S of M.Groups loop
         Total := Total + S.Size;
      end loop;
      M.Noise.Clear;
      M.Noise.Append (Real'Last, Ada.Containers.Count_Type (Total));
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            S : Group_Stream renames M.Groups (G);
            Beats : constant Natural := Natural (S.Present.Length);
            Rest  : Natural := 0;
         begin
            for B in 1 .. Beats - 1 loop
               if S.Present (B) and then S.Present (B - 1)
                 and then (not S.Commandable or else not Target_Changed (M, G, B))
               then
                  Rest := Rest + 1;
               end if;
            end loop;
            if Rest > 0 then
               for C in 1 .. S.Size loop
                  declare
                     D : Real_Array (1 .. Rest);
                     K : Natural := 0;
                  begin
                     for B in 1 .. Beats - 1 loop
                        if S.Present (B) and then S.Present (B - 1)
                          and then (not S.Commandable or else not Target_Changed (M, G, B))
                        then
                           K := K + 1;
                           D (K) := Change (M, G, B, C);
                        end if;
                     end loop;
                     M.Noise.Replace_Element (First_Channel (M, G) + C - 1, Driver.Stats.Robust_Sigma (D) / Sqrt (2.0));
                  end;
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

   function Asked (M : Model; G : Group_Id; Beat : Natural) return Boolean is
   begin
      if Beat = 0 or else not Target_Changed (M, G, Beat) or else not Has_Target (M, G, Beat)
        or else not Has_Reading (M, G, Beat - 1)
      then
         return False;
      end if;
      for C in 1 .. M.Groups (G).Size loop
         declare
            Sigma : constant Real := Noise (M, G, C);
            Ask   : constant Real := Target (M, G, Beat, C) - Reading (M, G, Beat - 1, C);
         begin
            if (Sigma < Real'Last and then Driver.Uncertain.Significant (Ask, Sigma))
              or else (Sigma = Real'Last and then Ask /= 0.0)
            then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Asked;

   --  The push started at Onset is still closing in at Beat: some channel it
   --  asked to move is significantly nearer its target than the beat before.
   function Closing_In (M : Model; G : Group_Id; Onset, Beat : Natural) return Boolean is
   begin
      if not Has_Reading (M, G, Beat) or else not Has_Reading (M, G, Beat - 1) or else not Has_Target (M, G, Beat) then
         return False;
      end if;
      for C in 1 .. M.Groups (G).Size loop
         declare
            Sigma : constant Real := Noise (M, G, C);
            Ask   : constant Real := Target (M, G, Onset, C) - Reading (M, G, Onset - 1, C);
            Gain  : constant Real := abs (Target (M, G, Beat, C) - Reading (M, G, Beat - 1, C))
                                     - abs (Target (M, G, Beat, C) - Reading (M, G, Beat, C));
         begin
            if Sigma < Real'Last and then Driver.Uncertain.Significant (Ask, Sigma)
              and then Gain > 0.0 and then Driver.Uncertain.Significant (Gain, Sigma * Sqrt (2.0))
            then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Closing_In;

   procedure Measure_Pushes (M : in out Model) is
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            Beats  : constant Natural := Natural (M.Groups (G).Present.Length);
            Delay_Beats : Natural := 0;
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
                           exit;
                        end if;
                     end loop;
                  end if;
               end loop;
            end if;
            for B in 0 .. Beats - 1 loop
               if M.Groups (G).Commandable and then Asked (M, G, B) then
                  Active := True;
                  Onset := B;
                  Moved := Moving (M, G, B);
               elsif Active then
                  --  Until the reading first moves the push waits out the
                  --  delay; after that it lasts while it closes in. A push
                  --  that is not answered within the delay is over.
                  if not Moved then
                     Moved := Moving (M, G, B);
                     if not Moved and then B - Onset >= Delay_Beats then
                        Active := False;
                     end if;
                  elsif not Closing_In (M, G, Onset, B) then
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

   function Moving (M : Model; G : Group_Id; Beat : Natural) return Boolean is
   begin
      if Beat = 0 or else not Has_Reading (M, G, Beat) or else not Has_Reading (M, G, Beat - 1) then
         return False;
      end if;
      for C in 1 .. M.Groups (G).Size loop
         declare
            Sigma : constant Real := Noise (M, G, C);
         begin
            --  The change of two readings has twice the variance of one.
            if Sigma < Real'Last and then Driver.Uncertain.Significant (Change (M, G, Beat, C), Sigma * Sqrt (2.0)) then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Moving;

end Driver.Robot.Channels;
