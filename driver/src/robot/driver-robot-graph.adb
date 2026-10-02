with Driver.Robot.Channels;
with Driver.Robot.Lag;

package body Driver.Robot.Graph is

   use type Driver.Observations.Group_Id;

   function Effect (M : Model; G : Group_Id; E : Eye_Id) return Eye_Effect is
      K : constant Natural := (Natural (G) - 1) * Natural (M.Eyes.Length) + Natural (E);
   begin
      return (if K in 1 .. Natural (M.Graph.Effects.Length) then M.Graph.Effects (K) else (others => <>));
   end Effect;

   procedure Derive (M : in out Model) is
      Eyes : constant Natural := Natural (M.Eyes.Length);
      Gr   : Body_Graph renames M.Graph;

      --  A group's own pushes: pushes it answered that no other commandable
      --  group began within the longest lag the stream can tell of. Until it
      --  has one, what the eyes saw while it moved cannot be told from what
      --  the groups that moved with it did, and it is not classified.
      function Pushed_Alone (G : Group_Id) return Boolean is
         Window : constant Natural := Lag.Longest (M);
      begin
         for E of M.Groups (G).Episodes loop
            if E.Moved then
               declare
                  Alone : Boolean := True;
               begin
                  for H in M.Groups.First_Index .. M.Groups.Last_Index loop
                     if H /= G and then M.Groups (H).Commandable then
                        for F of M.Groups (H).Episodes loop
                           if abs (F.Start - E.Start) <= Window then
                              Alone := False;
                           end if;
                        end loop;
                     end if;
                  end loop;
                  if Alone then
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Pushed_Alone;

      Own : array (M.Groups.First_Index .. M.Groups.Last_Index) of Boolean := [others => False];
   begin
      for G in Own'Range loop
         Own (G) := M.Groups (G).Commandable and then Pushed_Alone (G);
      end loop;
      Gr.Roles.Clear;
      Gr.Arm_Of.Clear;
      Gr.Breach.Clear;
      Gr.Arms.Clear;
      Gr.Mounts.Clear;
      Gr.Carrier := 0;
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Gr.Roles.Append (Unclassified);
         Gr.Arm_Of.Append (0);
         Gr.Breach.Append (0);
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         Gr.Mounts.Append (Mount'(Kind => Unmeasured));
      end loop;

      --  Groups that are not commanded: sensors or inert.
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if not M.Groups (G).Commandable then
            Gr.Roles.Replace_Element (G, Inert);
            for B in 1 .. M.Beats - 1 loop
               if Channels.Moving (M, G, B) then
                  Gr.Roles.Replace_Element (G, Sensor);
                  exit;
               end if;
            end loop;
         end if;
      end loop;

      --  Carrier and arms.
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if Own (G) then
            declare
               Whole_Eyes : Natural := 0;
            begin
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  if Effect (M, G, E).Verdict = Whole then
                     Whole_Eyes := Whole_Eyes + 1;
                  end if;
               end loop;
               --  Two eyes at least: with one, carrying it and carrying the
               --  body look the same.
               if Whole_Eyes = Eyes and then Eyes >= 2 then
                  if Gr.Carrier = 0 then
                     Gr.Carrier := G;
                     Gr.Roles.Replace_Element (G, Carrier);
                  end if;
               elsif Whole_Eyes > 0 then
                  Gr.Arms.Append (G);
                  Gr.Roles.Replace_Element (G, Arm);
                  Gr.Arm_Of.Replace_Element (G, Gr.Arms.Last_Index);
               end if;
            end;
         end if;
      end loop;

      --  Mounts: an eye rides on the arm whose push moves the largest share
      --  of it; on the carrier when only the carrier moves it whole; fixed
      --  in the world when no group does.
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Best_Arm   : Arm_Id'Base := 0;
            Best_Share : Real := 0.0;
         begin
            for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
               declare
                  F : constant Eye_Effect := Effect (M, Gr.Arms (A), E);
               begin
                  if F.Verdict = Whole and then F.Fraction.Value > Best_Share then
                     Best_Share := F.Fraction.Value;
                     Best_Arm := A;
                  end if;
               end;
            end loop;
            if Best_Arm > 0 then
               Gr.Mounts.Replace_Element (E, (Kind => Arm_Carried, Arm => Best_Arm));
            elsif Gr.Carrier > 0 and then Effect (M, Gr.Carrier, E).Verdict = Whole then
               Gr.Mounts.Replace_Element (E, (Kind => Carrier_Carried));
            else
               declare
                  Seen : Boolean := False;
               begin
                  for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                     Seen := Seen or else Effect (M, G, E).Verdict /= Unmeasured;
                  end loop;
                  if Seen then
                     Gr.Mounts.Replace_Element (E, (Kind => World_Fixed));
                  end if;
               end;
            end if;
         end;
      end loop;

      --  Closers and parts.
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if Own (G) and then Gr.Roles (G) = Unclassified then
            declare
               Best_Arm   : Arm_Id'Base := 0;
               Best_Count : Natural := 0;
               Patched    : Boolean := False;
               Unsure     : Boolean := False;
            begin
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  declare
                     F  : constant Eye_Effect := Effect (M, G, E);
                     Mt : constant Mount := Gr.Mounts (E);
                  begin
                     case F.Verdict is
                        when Patch =>
                           Patched := True;
                           if Mt.Kind = Arm_Carried and then F.Responding > Best_Count then
                              Best_Count := F.Responding;
                              Best_Arm := Mt.Arm;
                           end if;
                        when Undecided | Unmeasured =>
                           Unsure := True;
                        when Nothing | Whole =>
                           null;
                     end case;
                  end;
               end loop;
               if Best_Arm > 0 then
                  Gr.Roles.Replace_Element (G, Closer);
                  Gr.Arm_Of.Replace_Element (G, Best_Arm);
               elsif Patched then
                  Gr.Roles.Replace_Element (G, Part);
               elsif not Unsure then
                  Gr.Roles.Replace_Element (G, Inert);
               end if;
            end;
         end if;
      end loop;

      --  Porting contract, clauses 1 and 2.
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if M.Groups (G).Commandable then
            declare
               Pushed, Followed : Boolean := False;
            begin
               for B in 1 .. M.Beats - 1 loop
                  if Channels.Asked (M, G, B) then
                     Pushed := True;
                  end if;
                  if Pushed and then Channels.Moving (M, G, B) then
                     Followed := True;
                     exit;
                  end if;
               end loop;
               if Pushed and then not Followed then
                  Gr.Breach.Replace_Element (G, 1);
               elsif Followed and then Gr.Roles (G) = Inert then
                  Gr.Breach.Replace_Element (G, 2);
               end if;
            end;
         end if;
      end loop;
   end Derive;

end Driver.Robot.Graph;
