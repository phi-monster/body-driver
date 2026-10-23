with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Robot.Channels;
with Driver.Robot.Lag;

package body Driver.Robot.Graph is

   use Ada.Numerics.Long_Elementary_Functions;

   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   function Effect (M : Model; G : Group_Id; E : Eye_Id) return Eye_Effect is
      K : constant Natural := (Natural (G) - 1) * Natural (M.Eyes.Length) + Natural (E);
   begin
      return (if K in 1 .. Natural (M.Graph.Effects.Length) then M.Graph.Effects (K) else (others => <>));
   end Effect;

   procedure Derive (M : in out Model) is
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

      --  The eye whose view the keyframes of a group's arm tell moves with one of its channels (Arm_Evidence.Shift_Seen:
      --  the aligner found the points of the picture moving with the channel, not a patch of them), the one told of
      --  by the most points; zero where no keyframe told. The aligner measures over the whole picture, its windows
      --  reaching half of it, what the lock-in's flow measures a beat at a time, and its verdict stands: the lock-in,
      --  regressing every beat of the stream again at each estimate, read the arm's own eye as undecided or showing
      --  a patch during the arm's sweep and the measuring of its hand, the graph took the eye off the arm, and its
      --  keyframes and its hand went with it (f15f, f15j and f15o on 10-15).
      function Told_Eye (G : Group_Id) return Eye_Id'Base is
         Best : Eye_Id'Base := 0;
         Most : Natural := 0;
      begin
         for R of M.Kinematics loop
            if R.Group = G and then R.Eye in M.Eyes.First_Index .. M.Eyes.Last_Index then
               declare
                  Told : Natural := 0;
               begin
                  for C in 0 .. Natural'Min (Natural (R.Shift_Seen.Length), Natural (R.Shift_Told.Length)) - 1 loop
                     if R.Shift_Seen (C) then
                        Told := Told + R.Shift_Told (C);
                     end if;
                  end loop;
                  if Told > Most then
                     Most := Told;
                     Best := R.Eye;
                  end if;
               end;
            end if;
         end loop;
         return Best;
      end Told_Eye;

      Own : array (M.Groups.First_Index .. M.Groups.Last_Index) of Boolean := [others => False];

      type Eye_Flags is array (M.Eyes.First_Index .. M.Eyes.Last_Index) of Boolean;

      --  The eyes the arms of each group were for before this derivation: a group split into arms stays so while the lock-in
      --  says nothing of its channels (a boot begun from a body file has not read its streams yet; an estimate whose
      --  regression had too few rows has no column), and is split by what it says when it says something.
      Remembered : array (M.Groups.First_Index .. M.Groups.Last_Index) of Eye_Flags :=
        [others => [others => False]];

      --  Whether the lock-in has told something of some channel of group G in some eye.
      function Informed (G : Group_Id) return Boolean is
      begin
         for E in Gr.Channels.First_Index .. Gr.Channels.Last_Index loop
            for Item of Gr.Channels (E) loop
               if Item.Group = G and then Item.Effect.Verdict /= Unmeasured then
                  return True;
               end if;
            end loop;
         end loop;
         return False;
      end Informed;

      --  The eyes that some channel of group G moves whole while another eye decidedly shows it no whole motion (a
      --  patch, or nothing): a humanoid's neck moves the head's eye and not a wrist's, an arm its wrist's and not the
      --  head's, and its torso moves both. Each such eye rides on an arm of the group of its own, whose joints are the
      --  channels that move it. An eye that is undecided or not measured about a channel is no evidence against the
      --  channel moving it: a group's channels that all move every eye but the one the lock-in is undecided about are
      --  not told apart.
      function Moved_Apart (G : Group_Id) return Eye_Flags is
         Result : Eye_Flags := [others => False];
         package Set_Lists is new Ada.Containers.Vectors (Positive, Eye_Flags);
         Sets : Set_Lists.Vector;   --  the distinct sets of eyes that channels move apart from the rest
      begin
         for C in 1 .. M.Groups (G).Size loop
            declare
               Whole_In   : Eye_Flags := [others => False];
               Any_Whole  : Boolean := False;
               Any_Not    : Boolean := False;
            begin
               for E in Result'Range loop
                  if E <= Gr.Channels.Last_Index then
                     for Item of Gr.Channels (E) loop
                        if Item.Group = G and then Item.Channel = C then
                           case Item.Effect.Verdict is
                              when Whole =>
                                 Whole_In (E) := True;
                                 Any_Whole := True;
                              when Nothing | Patch =>
                                 Any_Not := True;
                              when Undecided | Unmeasured =>
                                 null;
                           end case;
                        end if;
                     end loop;
                  end if;
               end loop;
               if Any_Whole and then Any_Not then
                  for E in Result'Range loop
                     Result (E) := Result (E) or else Whole_In (E);
                  end loop;
                  if not Sets.Contains (Whole_In) then
                     Sets.Append (Whole_In);
                  end if;
               end if;
            end;
         end loop;
         --  Channels that all move the same eyes (an arm with two cameras) are one arm: the eyes are apart only where the
         --  channels' sets of them differ.
         return (if Natural (Sets.Length) >= 2 then Result else [others => False]);
      end Moved_Apart;

      --  The eye last mounted on the group as an arm (Group_Stream.Rider) still rides on it while it shows the group
      --  a patch of its view or is undecided: a sweep's small pushes leave the far cells of a whole view unanswered,
      --  and the lock-in, regressing on every push of the stream, falls back from whole (robot.boot.patch, 10-17:
      --  arm 2's eye whole at the recognition and undecided, 33 of 48 cells, at an estimate between its sweep's third
      --  and fourth passes; the arm was taken for a part and its sweep cut at 16 poses, in two runs of three). Only
      --  an eye in which the group moves nothing (Nothing), or that another arm moves whole, takes it off.
      function Still_Ridden (G : Group_Id) return Boolean is
        (M.Groups (G).Rider > 0 and then Effect (M, G, M.Groups (G).Rider).Verdict in Undecided | Patch);
   begin
      --  Once a push of the group was its own it stays so: the longest lag the stream can tell of is the median gap
      --  between its bursts, which the arms' sweeps, their pushes far apart, widen, and the recognition's pushes of a
      --  closer, alone when they were judged, came within it of the arm's later: the closer lost its role, and its arm
      --  its hand, at the first estimate after the sweep (s32, servo seed 6 of the fake robot, 10-17: "group 1 is no
      --  longer a closer of arm 1" at beat 3340, a hand of one lobe pressed for a tip it could not see; s31 at 11649).
      for G in Own'Range loop
         if M.Groups (G).Commandable and then not M.Groups (G).Own_Push and then Pushed_Alone (G) then
            M.Groups (G).Own_Push := True;
         end if;
         Own (G) := M.Groups (G).Commandable and then M.Groups (G).Own_Push;
      end loop;
      for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
         if A <= Gr.Arm_Eye.Last_Index and then Gr.Arm_Eye (A) /= 0 and then Gr.Arms (A) in Remembered'Range
           and then Gr.Arm_Eye (A) in Eye_Flags'Range
         then
            Remembered (Gr.Arms (A)) (Gr.Arm_Eye (A)) := True;
         end if;
      end loop;
      Gr.Roles.Clear;
      Gr.Arm_Of.Clear;
      Gr.Breach.Clear;
      Gr.Arms.Clear;
      Gr.Arm_Eye.Clear;
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

      --  Carrier and arms. The eyes that ride on the body: those some group moves whole. An eye fixed in the world
      --  is moved whole by none, and a carrier moves every other eye whole while the world's eye sees it as a patch:
      --  counted among the eyes it had to move, no group was ever a carrier where a fixed eye watched (the race's
      --  base bodies, 10-23: their bases taken for arms, their head eyes for fixed ones, 22 of 22).
      declare
         Ridden_Eyes : Natural := 0;
         Moves_Whole : array (M.Groups.First_Index .. M.Groups.Last_Index, M.Eyes.First_Index .. M.Eyes.Last_Index)
           of Boolean := [others => [others => False]];
      begin
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            if Own (G) then
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  Moves_Whole (G, E) := Effect (M, G, E).Verdict = Whole;
               end loop;
            end if;
         end loop;
         for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
            if (for some G in M.Groups.First_Index .. M.Groups.Last_Index => Moves_Whole (G, E)) then
               Ridden_Eyes := Ridden_Eyes + 1;
            end if;
         end loop;
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            if Own (G) then
               declare
                  Whole_Eyes : Natural := 0;
                  Apart      : constant Eye_Flags := (if Informed (G) then Moved_Apart (G) else Remembered (G));
                  Apart_Eyes : Natural := 0;
               begin
                  for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                     if Effect (M, G, E).Verdict = Whole then
                        Whole_Eyes := Whole_Eyes + 1;
                     end if;
                     if Apart (E) then
                        Apart_Eyes := Apart_Eyes + 1;
                     end if;
                  end loop;
                  --  Channels that move different eyes (two eyes at least that some channel moves and another decidedly does
                  --  not): an arm of the group for each such eye, however much of the view the group moves together.
                  if Apart_Eyes >= 2 then
                     Gr.Roles.Replace_Element (G, Arm);
                     for E in Apart'Range loop
                        if Apart (E) then
                           Gr.Arms.Append (G);
                           Gr.Arm_Eye.Append (E);
                           if Gr.Arm_Of (G) = 0 then
                              Gr.Arm_Of.Replace_Element (G, Gr.Arms.Last_Index);
                           end if;
                        end if;
                     end loop;
                  --  Two eyes at least: with one, carrying it and carrying the
                  --  body look the same. Every eye that rides on the body, and some other group moving some of them
                  --  whole and not all: something that moves by itself rides on it. An arm that carries two eyes at its
                  --  end moves them both, and no other group moves either.
                  elsif Whole_Eyes = Ridden_Eyes and then Ridden_Eyes >= 2
                    and then (for some H in M.Groups.First_Index .. M.Groups.Last_Index =>
                                H /= G
                                and then (for some E in M.Eyes.First_Index .. M.Eyes.Last_Index => Moves_Whole (H, E))
                                and then (for some E in M.Eyes.First_Index .. M.Eyes.Last_Index =>
                                            Moves_Whole (G, E) and then not Moves_Whole (H, E)))
                  then
                     if Gr.Carrier = 0 then
                        Gr.Carrier := G;
                        Gr.Roles.Replace_Element (G, Carrier);
                     end if;
                  elsif Whole_Eyes > 0 or else Told_Eye (G) > 0 or else Still_Ridden (G) then
                     Gr.Arms.Append (G);
                     Gr.Arm_Eye.Append (0);
                     Gr.Roles.Replace_Element (G, Arm);
                     Gr.Arm_Of.Replace_Element (G, Gr.Arms.Last_Index);
                  end if;
               end;
            end if;
         end loop;

      end;

      --  Mounts: an eye rides on the arm whose keyframes tell its view moves
      --  with the arm (Told_Eye); else on the arm whose push moves the
      --  largest share of it; on the carrier when only the carrier moves it
      --  whole; fixed in the world when no group does.
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Best_Arm   : Arm_Id'Base := 0;
            Best_Share : Real := 0.0;
            Keyed      : Arm_Id'Base := 0;
            Bound      : Arm_Id'Base := 0;   --  the arm of a split group that is for this eye
         begin
            for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
               if Gr.Arm_Eye (A) = E then
                  Bound := A;
               end if;
            end loop;
            for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
               if Gr.Arm_Eye (A) = 0 and then Told_Eye (Gr.Arms (A)) = E then
                  Keyed := A;
               end if;
            end loop;
            for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
               declare
                  F : constant Eye_Effect := Effect (M, Gr.Arms (A), E);
               begin
                  if Gr.Arm_Eye (A) = 0 and then F.Verdict = Whole and then F.Fraction.Value > Best_Share then
                     Best_Share := F.Fraction.Value;
                     Best_Arm := A;
                  end if;
               end;
            end loop;
            if Keyed = 0 and then Best_Arm = 0 then
               for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
                  if Gr.Arm_Eye (A) = 0 and then M.Groups (Gr.Arms (A)).Rider = E and then Still_Ridden (Gr.Arms (A)) then
                     Best_Arm := A;
                  end if;
               end loop;
            end if;
            if Bound > 0 then
               Gr.Mounts.Replace_Element (E, (Kind => Arm_Carried, Arm => Bound));
            elsif Keyed > 0 then
               Gr.Mounts.Replace_Element (E, (Kind => Arm_Carried, Arm => Keyed));
               M.Groups (Gr.Arms (Keyed)).Rider := E;
            elsif Best_Arm > 0 then
               Gr.Mounts.Replace_Element (E, (Kind => Arm_Carried, Arm => Best_Arm));
               M.Groups (Gr.Arms (Best_Arm)).Rider := E;
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
               Best_Eye   : Eye_Id'Base := 0;
               Best_Count : Natural := 0;
               Patched    : Boolean := False;
               Unsure     : Boolean := False;   --  some eye has not told
               Undecided_Eye : Boolean := False;   --  some eye told, but not enough to decide

               --  Another eye on an arm shows the group a patch of significantly more cells than Kept shows it now (a
               --  sign test: each cell that responds in either is as likely in both where the two show the group
               --  alike). Not where Kept was not measured: it then tells nothing.
               function Clearer (Kept : Eye_Id) return Boolean is
                  Now : constant Eye_Effect := Effect (M, G, Kept);
               begin
                  return Best_Eye > 0 and then Best_Eye /= Kept and then Now.Verdict /= Unmeasured
                    and then Real (Best_Count) - Real (Now.Responding)
                             > Driver.Conventions.Z * Sqrt (Real (Best_Count + Now.Responding));
               end Clearer;
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
                              Best_Eye := E;
                           end if;
                        when Undecided =>
                           Unsure := True;
                           Undecided_Eye := Undecided_Eye or else F.Could_Be_Whole;
                        when Unmeasured =>
                           Unsure := True;
                        when Nothing | Whole =>
                           null;
                     end case;
                  end;
               end loop;
               --  An eye that is undecided may ride on the group: whether it is
               --  an arm is still open, so it is neither a closer nor a part
               --  until every eye that saw it has decided. Not an eye whose most
               --  cells that may show the motion could not make its view whole
               --  (Eye_Effect.Could_Be_Whole): it cannot ride on the group, and
               --  held a jaw from its arm for good (f15s on 10-15: arm 2's jaw a
               --  patch of 11 cells in its own eye, the other arm's eye undecided
               --  between 19 and 133 cells of 255; no closer, no hand, the brain
               --  given only speech).
               --  The eye on an arm that last showed the group a closer of it (Group_Stream.Closer_Eye) still does while
               --  it tells nothing against it: not measured (the lock-in kept no column of the group at an estimate:
               --  s38, servo seed 6 of the fake robot, 10-17, replayed: at beat 2145 every eye UNMEASURED on its closer,
               --  the group unclassified, its closer's sweep and its hand dropped; s32, s36 and s37 lost theirs the
               --  same day), undecided, or a patch. An eye that may ride the group (undecided, its view possibly
               --  whole: the rule above) still leaves it unclassified; an eye in which the group moves nothing, or
               --  moves the whole view, takes the role off, and so does the eye leaving the arm. So does another eye
               --  on an arm whose patch of the group significantly outnumbers in cells what the eye that showed it
               --  shows now (Clearer): the eye that showed a closer first may have been the only eye then known to
               --  ride an arm (w11, servo seed 11 of the fake robot, 10-19: arm 1's closer a patch of 6 to 8 cells in
               --  arm 2's eye and of 36 in arm 1's own, which was known to ride arm 1 from beat 2543 on; the
               --  remembered eye kept the closer on arm 2, and arm 1 was given none).
               if Undecided_Eye then
                  null;
               elsif M.Groups (G).Closer_Eye > 0
                 and then Gr.Mounts (M.Groups (G).Closer_Eye).Kind = Arm_Carried
                 and then Effect (M, G, M.Groups (G).Closer_Eye).Verdict in Unmeasured | Undecided | Patch
                 and then not Clearer (M.Groups (G).Closer_Eye)
               then
                  Gr.Roles.Replace_Element (G, Closer);
                  Gr.Arm_Of.Replace_Element (G, Gr.Mounts (M.Groups (G).Closer_Eye).Arm);
               elsif Best_Arm > 0 then
                  Gr.Roles.Replace_Element (G, Closer);
                  Gr.Arm_Of.Replace_Element (G, Best_Arm);
                  M.Groups (G).Closer_Eye := Best_Eye;
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
