with Ada.Containers.Vectors;
with Driver.Robot.Channels;
with Driver.Robot.Lag;
with Driver.Robot.Lockin;
with Driver.Robot.Regression;

package body Driver.Robot.Graph is

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
      Best_Of : array (M.Eyes.First_Index .. M.Eyes.Last_Index) of Natural := [others => 0];   --  each eye's own group
      Most    : array (M.Groups.First_Index .. M.Groups.Last_Index, M.Eyes.First_Index .. M.Eyes.Last_Index) of Boolean :=
        [others => [others => False]];   --  the group moves the eye most (Best_Of)
      Carries : array (M.Groups.First_Index .. M.Groups.Last_Index, M.Eyes.First_Index .. M.Eyes.Last_Index) of Boolean :=
        [others => [others => False]];   --  the group carries the eye (Best_Of)
      Carried : array (M.Groups.First_Index .. M.Groups.Last_Index) of Natural := [others => 0];   --  the eyes it carries

      --  Group G moves eye E's view whole: as the lock-in finds it, or, where the lock-in measured nothing of the group
      --  in the eye (no beat of its pushes a row: the Shadow's arm, whose hand swung at every one, and the head of
      --  Stretch in seed 1, five paced rounds, 10-23), where its pushes left a significant majority of the view moved
      --  (Motion.Note_Stayed). Of 396 eyes of those rounds, 390 then mounted right, 363 with the lock-in's alone.
      function Whole_For (G : Group_Id; E : Eye_Id) return Boolean is
        (Effect (M, G, E).Verdict = Whole
         or else (Effect (M, G, E).Verdict = Unmeasured and then Natural (G) <= Natural (M.Eyes (E).Stayed_Cells.Length)
                  and then Lockin.Mostly (M.Eyes (E).Stayed_Moved (Natural (G) - 1), M.Eyes (E).Stayed_Cells (Natural (G) - 1))));

      --  The share of eye E's textured cells group G's pushes left moved beyond the flow's reach, as a mean over the
      --  pushes Motion.Note_Stayed measured; 0 where none was.
      function Far_Of (G : Group_Id; E : Eye_Id) return Real is
        (if Natural (G) <= Natural (M.Eyes (E).Stayed_Pushes.Length) and then M.Eyes (E).Stayed_Pushes (Natural (G) - 1) > 0
         then M.Eyes (E).Stayed_Far (Natural (G) - 1) / Real (M.Eyes (E).Stayed_Pushes (Natural (G) - 1)) else 0.0);

      --  The cells of eye E group G's pushes left moved, and the textured cells they were counted of, summed over the
      --  pushes Motion.Note_Stayed measured; and of them, those moved beyond the flow's reach.
      function Moved_Of (G : Group_Id; E : Eye_Id) return Natural is
        (if Natural (G) <= Natural (M.Eyes (E).Stayed_Moved.Length) then M.Eyes (E).Stayed_Moved (Natural (G) - 1) else 0);
      function Cells_Of (G : Group_Id; E : Eye_Id) return Natural is
        (if Natural (G) <= Natural (M.Eyes (E).Stayed_Cells.Length) then M.Eyes (E).Stayed_Cells (Natural (G) - 1) else 0);

      --  Group G's pushes left most of eye E's view unmoved: even the push of it that left the largest share moved left
      --  significantly more than half the textured cells unmoved. Not their sum: a push into an end, or one the body
      --  answered late, leaves nothing, and the test rig's arm, its push up leaving its eye's 48 cells unmoved and its
      --  push down 31 of them moved, was a part by the sum (robot.boot, 10-24).
      --  Its pushes left the eye's view mostly unmoved: no push of it left even the cells that did not move alone pulled
      --  together the same way (Stayed_Pulled: a far part of the view moved by less than any one cell can tell).
      function Unmoved (G : Group_Id; E : Eye_Id) return Boolean is
        (Natural (G) <= Natural (M.Eyes (E).Stayed_Best_Cells.Length)
         and then Natural (G) <= Natural (M.Eyes (E).Stayed_Best_Moved.Length)
         and then M.Eyes (E).Stayed_Best_Cells (Natural (G) - 1) > 0
         and then Lockin.Mostly (M.Eyes (E).Stayed_Best_Cells (Natural (G) - 1) - M.Eyes (E).Stayed_Best_Moved (Natural (G) - 1),
                                 M.Eyes (E).Stayed_Best_Cells (Natural (G) - 1))
         and then not (Natural (G) <= Natural (M.Eyes (E).Stayed_Pulled.Length)
                       and then M.Eyes (E).Stayed_Pulled (Natural (G) - 1) > 0));

      --  Group G may carry eye E: one of its own, not the carrier, that moves the eye's view whole.
      function Candidate (G : Group_Id; E : Eye_Id) return Boolean is
        (Own (G) and then G /= Gr.Carrier and then Whole_For (G, E));
      function Far_Cells_Of (G : Group_Id; E : Eye_Id) return Natural is
        (if Natural (G) <= Natural (M.Eyes (E).Stayed_Far_Cells.Length) then M.Eyes (E).Stayed_Far_Cells (Natural (G) - 1)
         else 0);

      --  Group G's pushes left eye E's cells beyond the flow's reach in a larger share than group H's pushes left eye F's,
      --  by more than chance (Regression.Shares_Apart on the cells counted): a push just seen leaves a view moved by a
      --  few tenths of a pixel and a cell or two beyond reach by noise, and by a cell the Shadow's hand, 3 to 7 of its
      --  wrist eye's 246 cells beyond reach against its arm's 0 to 2, moved that eye further than the arm whose camera
      --  it is, which moved 127 of them against the hand's 52 (seed 1, paced round p113a, 10-24).
      function Further (G : Group_Id; E : Eye_Id; H : Group_Id; F : Eye_Id) return Boolean is
        (Far_Of (G, E) > Far_Of (H, F)
         and then Driver.Robot.Regression.Shares_Apart
                    (Far_Cells_Of (G, E), Cells_Of (G, E), Far_Cells_Of (H, F), Cells_Of (H, F)));

      type Eye_Flags is array (M.Eyes.First_Index .. M.Eyes.Last_Index) of Boolean;

      --  The eyes the arms of each group were for before this derivation: a group split into arms stays so while the lock-in
      --  says nothing of its channels (a boot begun from a body file has not read its streams yet; an estimate whose
      --  regression had too few rows has no column), and is split by what it says when it says something of channels
      --  pushed one at a time (Group_Stream.Channels_Alone). Of channels only ever pushed together it tells nothing
      --  apart: their columns move together, and which of them takes the group's motion in an eye is the fit's to
      --  choose (RB-Y1's torso, pushed whole, its six channels each moving one, two or none of its three eyes whole
      --  at successive estimates, and parted for the wrist eyes, seeds 1, paced rounds p87 and p88, 10-24).
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
      --  A group an eye rode in an earlier derivation stays its arm while the eye is undecided about it or sees a patch of
      --  it, and while no other group is that eye's own now (Best_Of): ridden early by the wrist eye that its hand
      --  or its jaw filled half of, on few rows, the Shadow's hand and Stretch's gripper stayed arms once the eye rode its
      --  arm (seeds 1 and 2, paced round p76a, 10-23).
      function Still_Ridden (G : Group_Id) return Boolean is
        (M.Groups (G).Rider > 0 and then Effect (M, G, M.Groups (G).Rider).Verdict in Undecided | Patch
         and then Best_Of (M.Groups (G).Rider) in 0 | Natural (G));
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

      --  The carrier: the group a channel of which, held away from its rest command, kept the whole body going (the
      --  eyes' pictures drifting while every reading stood, and stopping once it was sent back: Motion.Settle), a speed
      --  that carries every eye it drifted; of two such groups, the one that drifted more eyes. It used to be the group
      --  that moved every eye riding on the body whole, two eyes at least, while another group moved some of them and
      --  not all: with one eye, carrying it and carrying the body look the same (the drone and the Go2, their carriers
      --  taken for arms in round 0, 10-23), and a torso that moves the head's and both arms' eyes whole carries no body
      --  (RB-Y1's, taken for the carrier in m4).
      declare
         Most : Natural := 0;
      begin
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            if Own (G) and then Keeps_Going (M, G) then
               declare
                  Kept : Natural := 0;
               begin
                  for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                     if Kept_Eye (M, G, E) then
                        Kept := Kept + 1;
                     end if;
                  end loop;
                  if Gr.Carrier = 0 or else Kept > Most then
                     Gr.Carrier := G;
                     Most := Kept;
                  end if;
               end;
            end if;
         end loop;
         if Gr.Carrier > 0 then
            Gr.Roles.Replace_Element (Gr.Carrier, Carrier);
         end if;
      end;

      --  Each eye's own group, the nearest of the groups that carry it. A group moves an eye most when it moves the
      --  eye's view whole (not the carrier, which moves every eye on the body) and its pushes left no other eye's view
      --  further beyond the flow's reach (Far_Of: a camera carried is turned and moved by the whole push, a part, its
      --  shadow and its reflection in a fixed eye sweep a share of its view, and a body that sways or rocks shifts a
      --  view by a little; an eye moved by a group that turns another eye further rides on neither, as far as that
      --  group goes). A group carries an eye when it moves its view whole and no other group that does left
      --  significantly more of the view moved, of its textured cells (Regression.Shares_Apart), while moving it at least
      --  as far beyond reach: a camera carried is displaced whole, where a part in its view (an arm in the head
      --  camera's, a hand before a wrist camera) or a view that swayed and came back is left moved in part; a body that
      --  rocks as another group moves leaves most of a view moved by little (Stretch's head camera as its arm moved,
      --  96 % of the cells moved and 8 % beyond reach, its head's own pushes 75 % and 38 %) and takes no carrier from it;
      --  and no group carries an eye whose view its pushes left mostly unmoved (significantly more than half its
      --  textured cells, Lockin.Mostly): the Panda's gripper, before its arm was recognized, whole by the lock-in in
      --  both its eyes as their views shook, its pushes leaving 10 and 2 % of them moved, carried both and rode the
      --  base camera on (seed 2, paced rounds p90a and p91b, 10-24); with no counts there is nothing against a group.
      --  Of the groups that move an eye most and carry it, the eye rides the one
      --  that carries the fewest eyes, the one whose pushes left the most of its view beyond reach of those that carry
      --  as many: a group carries every eye carried by a group it carries, RB-Y1's torso its head's camera and both
      --  wrists', its head the head's camera alone. By the most beyond reach alone, the torso took the head's camera
      --  in every boot of RB-Y1 that finished (paced rounds p79 to p87, 10-23 and 10-24), its pushes turning it six to fifteen
      --  times as far as the head's; and a wrist eye whose farthest-moving group, the torso, turned another eye
      --  further rode on nothing, its arm moving no eye further. Replayed over twenty paced rounds (p82 to p91, 10-23
      --  and 10-24), the own group of 1792 eyes was right in 1677 by the most beyond reach, in 1755 this way; of the
      --  eyes right before, two of the G1's are wrong now and one of Stretch's, its gripper filling its wrist eye's view
      --  as its arm did and its arm rocking the head's camera too (seed 2, round p89b).
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            Most (G, E) := Candidate (G, E)
              and then (for all F in M.Eyes.First_Index .. M.Eyes.Last_Index =>
                          F = E or else not (Further (G, F, G, E)));
         end loop;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            Carries (G, E) := Candidate (G, E)
              and then not Unmoved (G, E)
              and then not (Cells_Of (G, E) > 0
                            and then (for some H in M.Groups.First_Index .. M.Groups.Last_Index =>
                              H /= G and then Candidate (H, E) and then Cells_Of (H, E) > 0
                              and then Real (Moved_Of (H, E)) / Real (Cells_Of (H, E))
                                       > Real (Moved_Of (G, E)) / Real (Cells_Of (G, E))
                              and then not Further (G, E, H, E)
                              and then Driver.Robot.Regression.Shares_Apart
                                         (Moved_Of (H, E), Cells_Of (H, E), Moved_Of (G, E), Cells_Of (G, E))));
            if Carries (G, E) then
               Carried (G) := Carried (G) + 1;
            end if;
         end loop;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Fewest               : Natural := Natural'Last;
            Best_Far, Best_Share : Real := -1.0;
         begin
            for G in M.Groups.First_Index .. M.Groups.Last_Index loop
               if Most (G, E) and then Carries (G, E)
                 and then (Carried (G) < Fewest
                           or else (Carried (G) = Fewest
                                    and then (Far_Of (G, E) > Best_Far
                                              or else (Far_Of (G, E) = Best_Far
                                                       and then Effect (M, G, E).Fraction.Value > Best_Share))))
               then
                  Fewest := Carried (G);
                  Best_Far := Far_Of (G, E);
                  Best_Share := Effect (M, G, E).Fraction.Value;
                  Best_Of (E) := Natural (G);
               end if;
            end loop;
         end;
      end loop;

      --  Arms: every other group that moves some eye whole as its own group, or that carries an eye with the eye's own
      --  group and more eyes than it (a torso under a head and arms, which carries no eye of its own), or that some
      --  eye's keyframes tell it carries.
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if Own (G) and then G /= Gr.Carrier then
            declare
               Whole_Eyes : Natural := 0;
               Apart      : constant Eye_Flags :=
                 (if Informed (G) and then M.Groups (G).Channels_Alone then Moved_Apart (G) else Remembered (G));
               Apart_Eyes : Natural := 0;
               Above      : Boolean := False;   --  it carries an eye with the eye's own group, and more eyes than it
            begin
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  if Whole_For (G, E) and then Best_Of (E) = Natural (G) then
                     Whole_Eyes := Whole_Eyes + 1;
                  end if;
                  if Carries (G, E) and then Best_Of (E) > 0 and then Best_Of (E) /= Natural (G)
                    and then Carried (G) > Carried (Group_Id (Best_Of (E)))
                  then
                     Above := True;
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
               elsif Whole_Eyes > 0 or else Above or else Told_Eye (G) > 0 or else Still_Ridden (G) then
                  Gr.Arms.Append (G);
                  Gr.Arm_Eye.Append (0);
                  Gr.Roles.Replace_Element (G, Arm);
                  Gr.Arm_Of.Replace_Element (G, Gr.Arms.Last_Index);
               end if;
            end;
         end if;
      end loop;

      --  Mounts: an eye rides on the arm whose keyframes tell its view moves
      --  with the arm (Told_Eye); else on the arm whose push moves the
      --  largest share of it; on the carrier when only the carrier moves it
      --  whole; fixed in the world when no group does.
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Best_Arm   : Arm_Id'Base := 0;
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
            --  The arm of the eye's own group (Best_Of): of the arms that move it whole, by what their pushes left, and
            --  only the arm that moves no other eye more. By the share of the view alone the Panda's base camera rode
            --  on the arm that its arm, the arm's shadow and its reflection on the floor filled (round p66a, 10-23).
            for A in Gr.Arms.First_Index .. Gr.Arms.Last_Index loop
               if Gr.Arm_Eye (A) = 0 and then Natural (Gr.Arms (A)) = Best_Of (E) then
                  Best_Arm := A;
               end if;
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
            elsif Gr.Carrier > 0
              and then (Effect (M, Gr.Carrier, E).Verdict = Whole or else Kept_Eye (M, Gr.Carrier, E))
            then
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
               Patched    : Boolean := False;
               Unsure     : Boolean := False;   --  some eye has not told
               Undecided_Eye : Boolean := False;   --  some eye told, but not enough to decide
               Shown      : Eye_Flags := [others => False];   --  eyes on an arm that show the group a patch

               --  Another eye on an arm shows the group a patch, and the group's pushes left significantly more of its
               --  view moved beyond the flow's reach than of Kept's (Regression.Shares_Apart). Not where the lock-in has
               --  not measured the group in Kept: it then tells nothing.
               --  Eye E shows the group nearer than eye B: its pushes left significantly more of E's view beyond the
               --  flow's reach (Further); or as far, by more than chance, significantly more of it moved
               --  (Regression.Shares_Apart on the cells moved); or as either, more cells responding. By the raw share
               --  beyond reach, RB-Y1's left gripper, 1 cell of 249 beyond reach in its own wrist's eye against 1 of
               --  240 in the right wrist's, its pushes moving 61 and 28 of them, went to the right arm (seed 1, paced
               --  round p121a, 10-24).
               function Moved_More (E, B : Eye_Id) return Boolean is
                 (Cells_Of (G, E) > 0 and then Cells_Of (G, B) > 0
                  and then Real (Moved_Of (G, E)) / Real (Cells_Of (G, E))
                           > Real (Moved_Of (G, B)) / Real (Cells_Of (G, B))
                  and then Driver.Robot.Regression.Shares_Apart
                             (Moved_Of (G, E), Cells_Of (G, E), Moved_Of (G, B), Cells_Of (G, B)));
               function Nearer (E, B : Eye_Id) return Boolean is
                 (Further (G => G, E => E, H => G, F => B)
                  or else (not Further (G => G, E => B, H => G, F => E)
                           and then (Moved_More (E, B)
                                     or else (not Moved_More (E => B, B => E)
                                              and then Effect (M, G, E).Responding > Effect (M, G, B).Responding))));

               function Clearer (Kept : Eye_Id) return Boolean is
                 (Best_Eye > 0 and then Best_Eye /= Kept and then Effect (M, G, Kept).Verdict /= Unmeasured
                  and then Cells_Of (G, Kept) > 0 and then Cells_Of (G, Best_Eye) > 0
                  and then Real (Far_Cells_Of (G, Best_Eye)) / Real (Cells_Of (G, Best_Eye))
                           > Real (Far_Cells_Of (G, Kept)) / Real (Cells_Of (G, Kept))
                  and then Driver.Robot.Regression.Shares_Apart
                             (Far_Cells_Of (G, Best_Eye), Cells_Of (G, Best_Eye), Far_Cells_Of (G, Kept), Cells_Of (G, Kept)));

               --  An eye on an arm that shows the group a patch and rode none when the remembered eye was chosen: the
               --  remembered eye was never weighed against it (RB-Y1's right finger, seed 2, paced round p87b, 10-23:
               --  chosen in the left wrist's eye while the right wrist's was fixed in the world, it stayed there, 10 to 13
               --  cells of 229, the right wrist's eye on its arm since showing it in 23 of 252, not significantly more).
               function Rival_Since (Kept : Eye_Id) return Boolean is
                 (for some E in M.Eyes.First_Index .. M.Eyes.Last_Index =>
                    E /= Kept and then Shown (E)
                    and then not (Natural (E) <= Natural (M.Groups (G).Closer_Rivals.Length)
                                  and then M.Groups (G).Closer_Rivals (Natural (E) - 1)));
            begin
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  declare
                     F  : constant Eye_Effect := Effect (M, G, E);
                     Mt : constant Mount := Gr.Mounts (E);
                     --  A whole view that another group moves more of shows this group a part of it (Best_Of); so does a view
                     --  that another group is the eye's own of, where the lock-in is undecided about this group: the hand in
                     --  the Shadow's wrist eye, 133 of 253 cells, the eye riding its arm (paced round p67a, 10-23).
                     V  : constant Eye_Response :=
                       (if (F.Verdict = Whole and then Best_Of (E) /= Natural (G))
                          or else (F.Verdict = Undecided and then Best_Of (E) /= 0 and then Best_Of (E) /= Natural (G))
                        then Patch else F.Verdict);
                  begin
                     case V is
                        when Patch =>
                           Patched := True;
                           Shown (E) := Mt.Kind = Arm_Carried;
                           --  Of the eyes on arms, the one whose view the group's pushes left most beyond the flow's
                           --  reach, the most cells responding between eyes left as far: a closer's own wrist eye sees
                           --  it near, its fingers sweeping cells by more than the flow reaches, where an eye that sees
                           --  it from afar, or a body that rocks as it moves, has many cells respond and few moved far
                           --  (Stretch's gripper, seed 2, paced round p89a, 10-24: in its wrist's eye 107 of 255 cells
                           --  responding and 14 % of the view left beyond reach, in the head's eye 216 and 1 %, the robot
                           --  rocking as the gripper moved; by the cells responding, it went to the head's arm).
                           if Mt.Kind = Arm_Carried and then (Best_Eye = 0 or else Nearer (E, Best_Eye)) then
                              Best_Arm := Mt.Arm;
                              Best_Eye := E;
                           end if;
                        when Undecided =>
                           Unsure := True;
                           Undecided_Eye := Undecided_Eye or else (F.Could_Be_Whole and then not Unmoved (G, E));
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
               --  Nor an eye the group's pushes left mostly unmoved (Unmoved): no group carries it (Best_Of), and the
               --  Shadow's hand stayed unclassified while its base camera, undecided about it as its fingers flexed in
               --  view, was left 7 to 11 % moved by its pushes (seed 1, paced rounds p86, p89, p92, 10-23 and 10-24).
               --  The eye on an arm that last showed the group a closer of it (Group_Stream.Closer_Eye) still does while
               --  it tells nothing against it: not measured (the lock-in kept no column of the group at an estimate:
               --  s38, servo seed 6 of the fake robot, 10-17, replayed: at beat 2145 every eye UNMEASURED on its closer,
               --  the group unclassified, its closer's sweep and its hand dropped; s32, s36 and s37 lost theirs the
               --  same day), undecided, or a patch. An eye that may ride the group (undecided, its view possibly
               --  whole: the rule above) still leaves it unclassified; an eye in which the group moves nothing, or
               --  moves the whole view, takes the role off, and so does the eye leaving the arm. So does another eye
               --  on an arm whose view the group's pushes left significantly more of beyond reach (Clearer): the eye
               --  that showed a closer first may have been the only eye then known to
               --  ride an arm (w11, servo seed 11 of the fake robot, 10-19: arm 1's closer a patch of 6 to 8 cells in
               --  arm 2's eye and of 36 in arm 1's own, which was known to ride arm 1 from beat 2543 on; the
               --  remembered eye kept the closer on arm 2, and arm 1 was given none). Nor does it hold against an eye on an
               --  arm that rode none when it was chosen (Rival_Since): it was never weighed against it.
               if Undecided_Eye then
                  null;
               elsif M.Groups (G).Closer_Eye > 0
                 and then Gr.Mounts (M.Groups (G).Closer_Eye).Kind = Arm_Carried
                 and then (Effect (M, G, M.Groups (G).Closer_Eye).Verdict in Unmeasured | Undecided | Patch
                           or else (Effect (M, G, M.Groups (G).Closer_Eye).Verdict = Whole
                                    and then Best_Of (M.Groups (G).Closer_Eye) /= Natural (G)))
                 and then not Clearer (M.Groups (G).Closer_Eye)
                 and then not Rival_Since (M.Groups (G).Closer_Eye)
               then
                  Gr.Roles.Replace_Element (G, Closer);
                  Gr.Arm_Of.Replace_Element (G, Gr.Mounts (M.Groups (G).Closer_Eye).Arm);
               elsif Best_Arm > 0 then
                  Gr.Roles.Replace_Element (G, Closer);
                  Gr.Arm_Of.Replace_Element (G, Best_Arm);
                  M.Groups (G).Closer_Eye := Best_Eye;
                  M.Groups (G).Closer_Rivals.Clear;
                  for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                     M.Groups (G).Closer_Rivals.Append (Gr.Mounts (E).Kind = Arm_Carried);
                  end loop;
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
