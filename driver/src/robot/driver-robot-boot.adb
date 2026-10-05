with Ada.Containers.Vectors;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Driver.Beats;
with Driver.Commands;
with Driver.Log;
with Driver.Robot.Body_File;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;

package body Driver.Robot.Boot is

   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Robot.Motion.Channel_Ref;
   use type Driver.Observations.Group_Id;

   package Real_IO is new Ada.Text_IO.Float_IO (Real);

   --  A number in the log with its magnitude: probe sizes span many decades.
   function Scientific (X : Real) return String is
      S : String (1 .. 32);
   begin
      Real_IO.Put (S, X, Aft => 2, Exp => 3);
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Scientific;

   --  A repeatable uniform generator (Park and Miller's minimal standard),
   --  for push orders that no other group's repeats.
   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   type Push is record
      Channel : Positive;
      Sign    : Real;
   end record;

   type Push_Array is array (Positive range <>) of Push;

   procedure Shuffle (P : in out Push_Array; G : in out Generator) is
   begin
      for I in reverse P'First + 1 .. P'Last loop
         declare
            J : constant Positive := P'First + Natural (Real'Floor (Uniform (G) * Real (I - P'First + 1)));
            T : constant Push := P (I);
         begin
            P (I) := P (Natural'Min (J, I));
            P (Natural'Min (J, I)) := T;
         end;
      end loop;
   end Shuffle;

   procedure Run (M : in out Model; H : in out Driver.Robot.Hand.Hands; Body_File : String; Ok : out Boolean) is
      --  The decider reads the models only in held beats (Driver.Beats): every
      --  read below is inside one, through Within_A_Beat or Motion.
      Waited : Natural;
      --  One generator for every order the boot draws, so no two repeat.
      Rng    : Generator;

      procedure Estimate is
      begin
         Estimate_Now (M);
      end Estimate;

      --  What the body has measured goes to its file after each stage that
      --  completes, not once at the end: a boot that fails later has kept it,
      --  and the next one reloads it (docs/body-file.md). Without a file,
      --  nothing is kept.
      procedure Keep (Stage : String) is
         Written : Boolean := True;
         procedure Write is
         begin
            Driver.Robot.Body_File.Write (M, Body_File, Written);
         end Write;
      begin
         if Body_File'Length > 0 then
            Driver.Beats.Within_A_Beat (Write'Access);
            Driver.Log.Line
              (Driver.Log.Robot,
               (if Written then "boot: the body is kept in " & Body_File & " after " & Stage
                else "the body file " & Body_File & " cannot be written"));
         end if;
      end Keep;

      --  What the group holds now (Motion.Hold_Of): moves are taken from here.
      function Holds_Of (G : Group_Id; Size : Natural) return Real_Array is
         Result : Real_Array (1 .. Size) := [others => 0.0];
         procedure Read is
         begin
            if Natural (G) <= Group_Count (M) and then Group_Size (M, G) = Size then
               for C in 1 .. Size loop
                  Result (C) := Driver.Robot.Motion.Hold_Of (M, G, C);
               end loop;
            end if;
         end Read;
      begin
         Driver.Beats.Within_A_Beat (Read'Access);
         return Result;
      end Holds_Of;

      procedure Go_To (G : Group_Id; Target : Real_Array; Report : out Driver.Robot.Motion.Step_Report) is
         C : Driver.Commands.Command;
      begin
         Driver.Commands.Set_Target (C, G, Target);
         Driver.Robot.Motion.Step (M, C, Report);
      end Go_To;

      --  Where each channel's reading first followed a probe, from the probe of
      --  every channel together on: the amount of that level.
      type Answer is record
         Ref    : Driver.Robot.Motion.Channel_Ref;
         Amount : Real := 0.0;
      end record;
      package Answer_Vectors is new Ada.Containers.Vectors (Positive, Answer);
      Answers : Answer_Vectors.Vector;

      procedure Record_Answer (Ref : Driver.Robot.Motion.Channel_Ref; Amount : Real) is
      begin
         if Amount <= 0.0 then
            return;
         end if;
         for A of Answers loop
            if A.Ref = Ref then
               A.Amount := Real'Min (A.Amount, Amount);
               return;
            end if;
         end loop;
         Answers.Append (Answer'(Ref => Ref, Amount => Amount));
      end Record_Answer;

      --  Where every other channel of the body has answered: the largest
      --  amount at which a channel other than Ref first followed; Real'Last
      --  when none has.
      function Answered_Elsewhere (Ref : Driver.Robot.Motion.Channel_Ref) return Real is
         Bound : Real := 0.0;
      begin
         for A of Answers loop
            if A.Ref /= Ref then
               Bound := Real'Max (Bound, A.Amount);
            end if;
         end loop;
         return (if Bound > 0.0 then Bound else Real'Last);
      end Answered_Elsewhere;

      --  What Recognize found of each channel: the amount an eye sees it move
      --  at (0 when none does) and the ways found at their ends.
      type Recognized_Channel is record
         Ref    : Driver.Robot.Motion.Channel_Ref;
         Amount : Real := 0.0;
         Ended  : Driver.Robot.Motion.Sense_Flags := [others => False];
      end record;
      package Recognized_Vectors is new Ada.Containers.Vectors (Positive, Recognized_Channel);
      Recognized : Recognized_Vectors.Vector;

      --  Pushes every channel of the group both ways by Factor times the
      --  amount an eye sees it move at, never into a way at its end, in an
      --  order no other group shares, for the lock-in to tell the groups
      --  apart; then settles.
      procedure Push_Both_Ways (G : Group_Id; Size : Positive; Factor : Real) is
         use type Driver.Robot.Motion.Sense;
         Start  : constant Real_Array := Holds_Of (G, Size);
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         Report : Driver.Robot.Motion.Step_Report;
         Order  : Push_Array (1 .. 2 * Size);
         Count  : Natural := 0;
      begin
         for R of Recognized loop
            if R.Ref.Group = G and then R.Ref.Channel <= Size and then R.Amount > 0.0 then
               Amount (R.Ref.Channel) := Factor * R.Amount;
               for S in Driver.Robot.Motion.Sense loop
                  if not R.Ended (S) then
                     Count := Count + 1;
                     Order (Count) := (Channel => R.Ref.Channel,
                                       Sign    => (if S = Driver.Robot.Motion.Increasing then 1.0 else -1.0));
                  end if;
               end loop;
            end if;
         end loop;
         Shuffle (Order (1 .. Count), Rng);
         for P of Order (1 .. Count) loop
            declare
               Away : Real_Array := Start;
            begin
               Away (P.Channel) := Start (P.Channel) + P.Sign * Amount (P.Channel);
               Go_To (G, Away, Report);
               Go_To (G, Start, Report);
            end;
         end loop;
         Driver.Robot.Motion.Settle (M, Waited);
      end Push_Both_Ways;

      --  Finds how far each channel of the group must move for an eye to see
      --  it, both ways from the amount at which the whole body was first seen
      --  (Motion.Probe_Both_Ways: a way at its end stops where the other one
      --  answered; a channel that answers neither way up to twice where
      --  every other one did is dead for this boot), then pushes every
      --  channel both ways by that much (Push_Both_Ways).
      procedure Recognize (G : Group_Id; Size : Positive; From : Real) is
         use type Driver.Robot.Motion.Sense;
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         function Way (S : Driver.Robot.Motion.Sense) return String is
           (if S = Driver.Robot.Motion.Increasing then "upwards" else "downwards");
      begin
         for C in 1 .. Size loop
            declare
               Ref : constant Driver.Robot.Motion.Channel_Ref := (Group => G, Channel => C);
               P   : Driver.Robot.Motion.Two_Way_Report;
            begin
               Driver.Robot.Motion.Probe_Both_Ways (M, Ref, From, Answered_Elsewhere (Ref), P);
               Record_Answer (Ref, P.Answered);
               if P.Seen then
                  Amount (C) := P.Excursion;
               end if;
               Recognized.Append (Recognized_Channel'(Ref => Ref, Amount => Amount (C), Ended => P.At_End));
               for S in Driver.Robot.Motion.Sense loop
                  if P.At_End (S) then
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image & " is at its end "
                        & Way (S) & ": it delivered nothing that way up to "
                        & Scientific (From * 2.0 ** (P.Levels (S) - 1))
                        & " reading units, while it answered the other way");
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image
                                & (if Amount (C) > 0.0
                                   then " is seen when moved by " & Scientific (Amount (C)) & " reading units"
                                   elsif P.Dead
                                   then " answers neither way up to "
                                        & Scientific (From * 2.0 ** (P.Levels (Driver.Robot.Motion.Increasing) - 1))
                                        & " reading units, past which the next level would pass twice what every other"
                                        & " channel of the body needed: dead or disconnected for this boot; it is not"
                                        & " probed further"
                                   else " moves nothing any eye sees, up to where it stops following"));
            end;
         end loop;
         Push_Both_Ways (G, Size, 1.0);
      end Recognize;

      --  Turns every joint of the arm both ways from where it rests, by steps
      --  that double from the smallest one that moves its eye's view by what
      --  one cell of it can tell (Z times the cells' displacement noise, over
      --  how far the view moves per reading unit: a keyframe that moves less
      --  tells the fit nothing), until a step is blocked or short, or the eye
      --  would have turned by half its view: beyond that a view shares less
      --  than half of itself with the one it started from. Every step is held
      --  until the eye gives the keyframe (Motion.Hold_For_Keyframe).
      procedure Sweep (A : Arm_Id) is
         G    : Group_Id := 1;
         Size : Natural := 0;
         procedure Read_Size is
         begin
            G := Arm_Group (M, A);
            Size := Group_Size (M, G);
         end Read_Size;
      begin
         Driver.Beats.Within_A_Beat (Read_Size'Access);
         Driver.Robot.Motion.Hold_For_Twin (M, A);
         declare
            First, Per_Unit : Real_Array (1 .. Size) := [others => 0.0];
            Half   : Real := 0.0;
            Eye    : Natural := 0;
            procedure Read_Plan is
            begin
               for E in 1 .. Eye_Count (M) loop
                  if Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A then
                     Eye := E;
                  end if;
               end loop;
               if Eye > 0 then
                  Half := Real (Natural'Min (M.Eyes (Eye_Id (Eye)).Grid.Width, M.Eyes (Eye_Id (Eye)).Grid.Height))
                    / 2.0;
                  for C in 1 .. Size loop
                     Per_Unit (C) := Lockin.Shift (M, Eye_Id (Eye), G, C);
                     First (C) := Driver.Robot.Motion.Sweep_Start (M, A, C);
                  end loop;
               end if;
            end Read_Plan;
            Start  : constant Real_Array := Holds_Of (G, Size);
            Report : Driver.Robot.Motion.Step_Report;
         begin
            Driver.Beats.Within_A_Beat (Read_Plan'Access);
            if Eye = 0 then
               Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " carries no eye; its joints are not swept");
               return;
            end if;
            declare
               --  Joints and directions in an order no other arm shares, for
               --  the estimates over the whole stream to tell them apart.
               Order : Push_Array (1 .. 2 * Size);
               Count : Natural := 0;
            begin
               for C in 1 .. Size loop
                  if Per_Unit (C) > 0.0 and then First (C) > 0.0 then
                     Order (Count + 1) := (Channel => C, Sign => 1.0);
                     Order (Count + 2) := (Channel => C, Sign => -1.0);
                     Count := Count + 2;
                  else
                     Driver.Log.Line (Driver.Log.Robot, "boot: arm" & A'Image & " channel" & C'Image
                                      & " does not move its eye; not swept");
                  end if;
               end loop;
               Shuffle (Order (1 .. Count), Rng);
               for P of Order (1 .. Count) loop
                  declare
                     Offset : Real := First (P.Channel);
                     Level  : Positive := 1;
                  begin
                     while Offset * Per_Unit (P.Channel) <= Half loop
                        declare
                           Pose : Real_Array := Start;
                        begin
                           Pose (P.Channel) := Start (P.Channel) + P.Sign * Offset;
                           Go_To (G, Pose, Report);
                           --  Every other level is held for its keyframe, the
                           --  first and the widest included: keyframes over
                           --  every scale from the matcher's limit to the widest
                           --  turn, one per two doublings, fit as well as one per
                           --  doubling (A9's arm: 59 keyframes against 97, the
                           --  same against truth), and the holds are most of the
                           --  sweep. An arm given up while it kept moving
                           --  (chattering against what stops it) gives no
                           --  keyframe there.
                           if Report.At_Rest
                             and then (Level mod 2 = 1 or else Report.Outcome /= Driver.Robot.Motion.Reached
                                       or else 2.0 * Offset * Per_Unit (P.Channel) > Half)
                           then
                              Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                           end if;
                           exit when Report.Outcome /= Driver.Robot.Motion.Reached;
                        end;
                        Offset := 2.0 * Offset;
                        Level := Level + 1;
                     end loop;
                  end;
                  Go_To (G, Start, Report);
               end loop;
               --  Then every swept joint at once, in cells whose signs no two
               --  share (the rows of a Sylvester-Hadamard matrix after its first,
               --  its columns after its first): a joint's own frames tell its
               --  axis but not how far that axis is from the others', which only
               --  frames that move several joints show. Each joint moves the view
               --  by its share of half of it, so the cell keeps most of the view.
               declare
                  Swept : array (1 .. Size) of Positive;
                  N     : Natural := 0;
                  Rows  : Positive := 1;
               begin
                  for C in 1 .. Size loop
                     if Per_Unit (C) > 0.0 and then First (C) > 0.0 then
                        N := N + 1;
                        Swept (N) := C;
                     end if;
                  end loop;
                  while Rows < N + 1 loop
                     Rows := 2 * Rows;
                  end loop;
                  if N > 1 then
                     for Row in 1 .. Rows - 1 loop
                        declare
                           Pose : Real_Array := Start;
                        begin
                           for K in 1 .. N loop
                              declare
                                 Bits : Natural := 0;
                                 R    : Natural := Row;
                                 Col  : Natural := K;
                              begin
                                 --  H (Row, K) = (-1) ** (the bits Row and K share)
                                 while R > 0 and then Col > 0 loop
                                    if R mod 2 = 1 and then Col mod 2 = 1 then
                                       Bits := Bits + 1;
                                    end if;
                                    R := R / 2;
                                    Col := Col / 2;
                                 end loop;
                                 Pose (Swept (K)) := Start (Swept (K))
                                   + (if Bits mod 2 = 0 then 1.0 else -1.0) * Half / (Real (N) * Per_Unit (Swept (K)));
                              end;
                           end loop;
                           Go_To (G, Pose, Report);
                           if Report.At_Rest then
                              Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                           end if;
                        end;
                     end loop;
                     Go_To (G, Start, Report);
                  end if;
               end;
            end;
            Driver.Robot.Motion.Settle (M, Waited);
         end;
      end Sweep;

      Count  : Natural := 0;
      Breach : Boolean := False;

      procedure Read_Count is
      begin
         Count := Group_Count (M);
      end Read_Count;

      procedure Read_Body is
      begin
         Driver.Log.Line (Driver.Log.Robot, "boot: the groups as recognized:" & ASCII.LF & Describe (M));
         for G in 1 .. Group_Count (M) loop
            if Is_Commandable (M, Group_Id (G)) and then Contract_Breach (M, Group_Id (G)) > 0 then
               Breach := True;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " breaks clause"
                                & Contract_Breach (M, Group_Id (G))'Image & " of the porting contract");
            end if;
         end loop;
      end Read_Body;

      --  Every arm of the current estimate is swept with the eye it carries:
      --  the body is read again before each sweep and after the fit's
      --  estimate, since later evidence can change a role or a mount (an eye
      --  can decide by a later estimate that it rides on a group). An arm is
      --  known by its group, since a new arm can renumber the others, and is
      --  swept again when it carries another eye than the one it was swept
      --  with; an arm that carries none has nothing to sweep. The fit is made
      --  when no arm is left.
      procedure Sweep_Every_Arm is
         type Arm_Eye is record
            Group : Group_Id := 1;
            Eye   : Eye_Id := 1;
         end record;
         package Pair_Lists is new Ada.Containers.Vectors (Positive, Arm_Eye);
         Swept : Pair_Lists.Vector;
         Next  : Arm_Id'Base := 0;
         Pair  : Arm_Eye;
         procedure Find_Unswept is
         begin
            Next := 0;
            for A in 1 .. Arm_Count (M) loop
               for E in 1 .. Eye_Count (M) loop
                  --  The eye the sweep moves: the last one the arm carries.
                  if Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried
                    and then Eye_Mount (M, Eye_Id (E)).Arm = Arm_Id (A)
                  then
                     Next := Arm_Id (A);
                     Pair := (Group => Arm_Group (M, Arm_Id (A)), Eye => Eye_Id (E));
                  end if;
               end loop;
               if Next > 0 and then Swept.Contains (Pair) then
                  Next := 0;
               end if;
               exit when Next > 0;
            end loop;
         end Find_Unswept;
      begin
         loop
            Driver.Beats.Within_A_Beat (Find_Unswept'Access);
            if Next > 0 then
               Driver.Log.Line (Driver.Log.Robot, "boot: sweeping arm" & Next'Image & " (group" & Pair.Group'Image
                                & ") with eye" & Pair.Eye'Image);
               Sweep (Next);
               Swept.Append (Pair);
            else
               --  The instrument answers the keyframes' matches a beat or more
               --  after they were asked: the fit waits for every answer.
               Driver.Robot.Motion.Hold_While_Matching (M);
               Driver.Beats.Within_A_Beat (Estimate'Access);
               Driver.Beats.Within_A_Beat (Find_Unswept'Access);
               exit when Next = 0;
               Driver.Log.Line (Driver.Log.Robot, "boot: the estimate after the sweeps lists arm" & Next'Image
                                & " (group" & Pair.Group'Image & ") with eye" & Pair.Eye'Image & ", not swept yet");
            end if;
         end loop;
      end Sweep_Every_Arm;
   begin
      Ok := False;
      Driver.Log.Line (Driver.Log.Robot, "boot: holding still to measure the body at rest");
      Driver.Robot.Motion.Settle (M, Waited);
      --  A body file from an earlier boot: what it holds stands, what it does
      --  not is measured below (Load_Body). Read once the robot has answered
      --  a held beat, so the key has every group that takes commands.
      if Body_File'Length > 0 then
         declare
            Loaded : Boolean := False;
            Why    : Ada.Strings.Unbounded.Unbounded_String;
            procedure Reload is
            begin
               Load_Body (M, Body_File, Loaded, Why);
            end Reload;
         begin
            Driver.Beats.Within_A_Beat (Reload'Access);
            Driver.Log.Line (Driver.Log.Robot, "boot: " & (if Loaded then "from " & Body_File & ", " else "")
                             & Ada.Strings.Unbounded.To_String (Why));
         end;
      end if;
      Driver.Beats.Within_A_Beat (Estimate'Access);
      Driver.Beats.Within_A_Beat (Read_Count'Access);
      declare
         Recognized_Before, Fitted_Before : Boolean := False;
         procedure Read_Reloaded is
         begin
            Recognized_Before := Reloaded (M, Stored_Graph);
            Fitted_Before := Reloaded (M, Stored_Kinematics);
         end Read_Reloaded;
      begin
         Driver.Beats.Within_A_Beat (Read_Reloaded'Access);
         if not Recognized_Before then
            declare
               Commandable : array (1 .. Count) of Boolean := [others => False];
               Sizes       : array (1 .. Count) of Natural := [others => 0];
               procedure Read_Groups is
               begin
                  for G in 1 .. Count loop
                     Commandable (G) := Is_Commandable (M, Group_Id (G));
                     Sizes (G) := Group_Size (M, Group_Id (G));
                  end loop;
               end Read_Groups;
            begin
               Driver.Beats.Within_A_Beat (Read_Groups'Access);
               declare
                  Total : Natural := 0;
               begin
                  for G in 1 .. Count loop
                     if Commandable (G) then
                        Total := Total + Sizes (G);
                     end if;
                  end loop;
                  if Total = 0 then
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: no group takes a command; nothing can be moved to be measured");
                     return;
                  end if;
                  declare
                     Refs : Driver.Robot.Motion.Channel_Refs (1 .. Total);
                     K    : Natural := 0;
                     P    : Driver.Robot.Motion.Probe_Report;
                  begin
                     for G in 1 .. Count loop
                        if Commandable (G) then
                           for C in 1 .. Sizes (G) loop
                              K := K + 1;
                              Refs (K) := (Group => Group_Id (G), Channel => C);
                           end loop;
                        end if;
                     end loop;
                     --  Every commandable channel together first, by one amount: it
                     --  stops at the first move an eye sees, so no channel has moved
                     --  more than twice what an eye needs to see it, whatever its
                     --  units; each channel alone then starts from there.
                     Driver.Robot.Motion.Gather_Rest (M, Total + 1);
                     declare
                        First_Followed : Real_Array (1 .. Total);
                     begin
                        Driver.Robot.Motion.Probe_Together (M, Refs, 1.0, 0.0, P, First_Followed);
                        for K in Refs'Range loop
                           Record_Answer (Refs (K), First_Followed (K));
                        end loop;
                     end;
                     if not P.Seen then
                        Driver.Beats.Within_A_Beat (Estimate'Access);
                        Driver.Beats.Within_A_Beat (Read_Body'Access);
                        Driver.Log.Line
                          (Driver.Log.Robot, "boot: nothing any eye sees moved while every commandable channel moved"
                           & " together, up to where each stopped following its command (porting contract, clause 2);"
                           & " the boot stops");
                        return;
                     end if;
                     Driver.Log.Line
                       (Driver.Log.Robot, "boot: every commandable channel moved together is first seen at "
                        & Scientific (P.Excursion) & " reading units, after" & P.Steps'Image & " doublings");
                     for G in 1 .. Count loop
                        if Commandable (G) and then Sizes (G) > 0 then
                           Recognize (Group_Id (G), Sizes (G), P.Excursion);
                        end if;
                     end loop;
                     Driver.Beats.Within_A_Beat (Estimate'Access);
                     --  An undecided verdict is too little evidence, not an answer: a
                     --  group some eye is undecided about is pushed again, at twice
                     --  the amounts of its last round, until every eye has decided, or
                     --  a round leaves the cells each undecided eye found responding
                     --  no more than they were (the evidence stopped growing).
                     declare
                        Eyes : Natural := 0;
                        procedure Read_Eyes is
                        begin
                           Eyes := Eye_Count (M);
                        end Read_Eyes;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Eyes'Access);
                        declare
                           type Count_Grid is array (1 .. Count, 1 .. Eyes) of Natural;
                           Last   : Count_Grid := [others => [others => 0]];
                           Now    : Count_Grid;
                           Factor : array (1 .. Count) of Real := [others => 1.0];
                           Again  : array (1 .. Count) of Boolean;
                           procedure Read_Undecided is
                           begin
                              for G in 1 .. Count loop
                                 Again (G) := False;
                                 for E in 1 .. Eyes loop
                                    Now (G, E) := 0;
                                    if Commandable (G) and then Response (M, Group_Id (G), Eye_Id (E)) = Undecided then
                                       Now (G, E) := Responding (M, Group_Id (G), Eye_Id (E));
                                       Again (G) := Again (G) or else Now (G, E) > Last (G, E);
                                    end if;
                                 end loop;
                              end loop;
                           end Read_Undecided;
                        begin
                           loop
                              Driver.Beats.Within_A_Beat (Read_Undecided'Access);
                              exit when (for all G in Again'Range => not Again (G));
                              for G in Again'Range loop
                                 if Again (G) then
                                    Factor (G) := 2.0 * Factor (G);
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "boot: group" & G'Image & " leaves an eye undecided;"
                                       & " pushed again at" & Integer'Image (Integer (Factor (G)))
                                       & " times its amounts");
                                    Push_Both_Ways (Group_Id (G), Sizes (G), Factor (G));
                                 end if;
                              end loop;
                              Last := Now;
                              Driver.Beats.Within_A_Beat (Estimate'Access);
                           end loop;
                        end;
                     end;
                  end;
               end;
            end;
         end if;
         Driver.Beats.Within_A_Beat (Read_Body'Access);
         if not Recognized_Before then
            Keep ("recognizing the groups");
         end if;
         if not Fitted_Before then
            Sweep_Every_Arm;
            Keep ("sweeping the arms");
         end if;
      end;
      Driver.Robot.Hand.Measure (H, M);
      declare
         procedure Store is
         begin
            Save (M, H, Body_File);
         end Store;
      begin
         Driver.Beats.Within_A_Beat (Store'Access);
      end;
      Ok := not Breach;
   end Run;

   procedure Save (M : Model; H : Driver.Robot.Hand.Hands; Body_File : String) is
      pragma Unreferenced (H);
      Written : Boolean;
   begin
      Driver.Log.Line (Driver.Log.Robot, "the body as measured:" & ASCII.LF & Describe (M));
      if Body_File'Length > 0 then
         Driver.Robot.Body_File.Write (M, Body_File, Written);
         if not Written then
            Driver.Log.Line (Driver.Log.Robot, "the body file " & Body_File & " cannot be written");
         end if;
      end if;
   end Save;

end Driver.Robot.Boot;
